#include "kx_modbus_pipeline.h"
#include "kx_modbus_dispatch.h"
#include "kx_modbus_packetizer.h"
#include "kx_modbus_shared.h"
#include "kx_modbus_publish.h"
#include "kx_modbus_uart.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "kx_telemetry.h"
#include "../../../main/kx_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "kx_pipeline";

// =============================================================
// kx_modbus_pipeline.c — Tarea principal demand RTU
// =============================================================

#define MAX_CTRLS_IN_BATCH   KX_PARAM_MAX_CONTROLS
#define MAX_PARAMS_IN_BATCH  1500
#define MAX_CTRL_VISITED     KX_PARAM_MAX_CONTROLS

#define BURST_COLLECT_MAX_MS  3000
#define BURST_STABLE_MS        300
#define BURST_POLL_MS           50

// ── Tipos internos ────────────────────────────────────────────
typedef struct {
    int  param_id;
    bool ok;
    bool dropped;
    char err_msg[48];
} _batch_result_t;

typedef struct {
    int ctrl_id;
    int param_ids[MAX_PARAMS_IN_BATCH];
    int n_params;
} _ctrl_group_t;

typedef struct { int target_param_id; int found_ctrl_id; } _find_ctrl_ctx_t;
typedef struct { int count; } _count_ctx_t;

// ── Helpers foreach ───────────────────────────────────────────
static void _find_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)ud;
    if (ctx->found_ctrl_id < 0 && param->param_id == ctx->target_param_id)
        ctx->found_ctrl_id = ctrl_id;
}

static void _count_readable(int control_id, const kx_param_t *param, void *ud)
{
    (void)control_id;
    _count_ctx_t *c = (_count_ctx_t *)ud;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE ||
        fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS)
        c->count++;
}

// ── Drain de la cola de demandas ─────────────────────────────
static int _drain_demand_queue(QueueHandle_t         demand_queue,
                                volatile uint8_t     *pending_bits,
                                kx_poll_demand_t     *snapshot,
                                int                   capacity,
                                int                  *out_expired,
                                int                  *out_dupes)
{
    int64_t now_ms  = (int64_t)(esp_timer_get_time() / 1000ULL);
    int     expired = 0, dupes = 0, count = 0;

    kx_poll_demand_t d;
    while (count < capacity &&
           xQueueReceive(demand_queue, &d, 0) == pdTRUE) {
        KX_PENDING_CLEAR(pending_bits, d.param_id);

        if ((now_ms - d.enqueued_ms) >
            (int64_t)(KX_DEMAND_TIMEOUT_S * 1000)) {
            expired++;
            continue;
        }

        bool found = false;
        for (int j = 0; j < count; j++) {
            if (snapshot[j].param_id == d.param_id) {
                if (d.enqueued_ms > snapshot[j].enqueued_ms)
                    snapshot[j].enqueued_ms = d.enqueued_ms;
                dupes++;
                found = true;
                break;
            }
        }
        if (!found) snapshot[count++] = d;
    }

    // Descartar sobrantes
    int leftovers = 0;
    kx_poll_demand_t tmp;
    while (xQueueReceive(demand_queue, &tmp, 0) == pdTRUE) leftovers++;
    if (leftovers > 0)
        ESP_LOGW(TAG, "_drain: %d leftover demands discarded", leftovers);

    if (out_expired) *out_expired = expired;
    if (out_dupes)   *out_dupes   = dupes;
    return count;
}

// ── Full cycle (demanda param_id == 0) ───────────────────────
typedef struct {
    kx_pipeline_ctx_t *pctx;
    int                visited[MAX_CTRL_VISITED];
    int                n_visited;
    int                ok;
    int                errors;
} _poll_ctrl_foreach_ud_t;

static void _poll_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    (void)param;
    _poll_ctrl_foreach_ud_t *u = (_poll_ctrl_foreach_ud_t *)ud;
    for (int i = 0; i < u->n_visited; i++)
        if (u->visited[i] == ctrl_id) return;
    if (u->n_visited < MAX_CTRL_VISITED)
        u->visited[u->n_visited++] = ctrl_id;

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    kx_packet_list_t *list = kx_pkt_build(ctrl_id, true, NULL, 0, 0, now_ms);
    if (!list) return;

    kx_dispatch_control_packets(u->pctx->pub_queue,
                                 u->pctx->foreach_mutex,
                                 list,
                                 kx_param_pub_status,
                                 &u->ok, &u->errors, -1);
    kx_pkt_free(list);
}

// ── Batch poll (demandas específicas) ────────────────────────
static void _poll_batch_packetized(kx_pipeline_ctx_t        *pctx,
                                    const kx_poll_demand_t   *snapshot,
                                    int                       valid_count,
                                    _batch_result_t          *results,
                                    int                      *out_ok,
                                    int                      *out_errors,
                                    int                      *out_packaged)
{
    _ctrl_group_t *groups = malloc(MAX_CTRLS_IN_BATCH * sizeof(_ctrl_group_t));
    if (!groups) {
        ESP_LOGE(TAG, "batch_packetized: OOM groups");
        for (int i = 0; i < valid_count; i++) {
            snprintf(results[i].err_msg, sizeof(results[i].err_msg), "OOM");
            (*out_errors)++;
        }
        return;
    }
    int n_groups = 0;

    // Agrupar param_ids por control
    for (int i = 0; i < valid_count; i++) {
        int param_id = snapshot[i].param_id;

        _find_ctrl_ctx_t fctx = { param_id, -1 };
        kx_param_store_foreach(_find_ctrl_cb, &fctx);

        if (fctx.found_ctrl_id < 0) {
            snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                     "not found in any control");
            results[i].ok = false;
            (*out_errors)++;
            continue;
        }

        int g = -1;
        for (int j = 0; j < n_groups; j++) {
            if (groups[j].ctrl_id == fctx.found_ctrl_id) { g = j; break; }
        }
        if (g < 0) {
            if (n_groups >= MAX_CTRLS_IN_BATCH) {
                ESP_LOGW(TAG, "batch: too many controls, skipping param=%d",
                         param_id);
                snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                         "too many controls");
                results[i].ok = false;
                (*out_errors)++;
                continue;
            }
            g = n_groups++;
            groups[g].ctrl_id  = fctx.found_ctrl_id;
            groups[g].n_params = 0;
        }
        if (groups[g].n_params < MAX_PARAMS_IN_BATCH)
            groups[g].param_ids[groups[g].n_params++] = param_id;
    }

    // Ejecutar por grupo
    for (int g = 0; g < n_groups && *pctx->running; g++) {
        _ctrl_group_t *grp = &groups[g];

        if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
            for (int j = 0; j < grp->n_params; j++) {
                for (int i = 0; i < valid_count; i++) {
                    if (snapshot[i].param_id == grp->param_ids[j]) {
                        snprintf(results[i].err_msg,
                                 sizeof(results[i].err_msg),
                                 "mqtt/store not ready");
                        results[i].ok = false;
                        (*out_errors)++;
                        break;
                    }
                }
            }
            continue;
        }

        xEventGroupWaitBits(pctx->poll_eg, pctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
        kx_packet_list_t *list = kx_pkt_build(grp->ctrl_id, true,
                                               grp->param_ids, grp->n_params,
                                               0, now_ms);
        if (!list) {
            for (int j = 0; j < grp->n_params; j++) {
                for (int i = 0; i < valid_count; i++) {
                    if (snapshot[i].param_id == grp->param_ids[j]) {
                        snprintf(results[i].err_msg,
                                 sizeof(results[i].err_msg),
                                 "not readable/visible");
                        results[i].ok = false;
                        (*out_errors)++;
                        break;
                    }
                }
            }
            continue;
        }

        if (out_packaged) *out_packaged += kx_pkt_real_param_count(list);

        int ctrl_ok = 0, ctrl_errors = 0;
        kx_dispatch_control_packets(pctx->pub_queue,
                                     pctx->foreach_mutex,
                                     list,
                                     kx_param_pub_status,
                                     &ctrl_ok, &ctrl_errors, -1);
        *out_ok     += ctrl_ok;
        *out_errors += ctrl_errors;

        int64_t ts_end = (int64_t)(esp_timer_get_time() / 1000ULL);

        // Marcar resultados OK por ts_last_read reciente
        for (int i = 0; i < valid_count; i++) {
            int pid = snapshot[i].param_id;
            bool in_group = false;
            for (int j = 0; j < grp->n_params; j++) {
                if (grp->param_ids[j] == pid) { in_group = true; break; }
            }
            if (!in_group || results[i].ok) continue;

            _find_ctrl_ctx_t fc2 = { pid, -1 };
            kx_param_store_foreach(_find_ctrl_cb, &fc2);
            if (fc2.found_ctrl_id >= 0) {
                const kx_param_t *p =
                    kx_param_store_get_param(fc2.found_ctrl_id, pid);
                if (p && p->ts_last_read > 0 &&
                    (ts_end - p->ts_last_read) < 5000) {
                    results[i].ok = true;
                } else if (results[i].err_msg[0] == '\0') {
                    snprintf(results[i].err_msg,
                             sizeof(results[i].err_msg), "mb_no_response");
                }
            } else if (results[i].err_msg[0] == '\0') {
                snprintf(results[i].err_msg,
                         sizeof(results[i].err_msg), "not_in_store");
            }
        }

        kx_pkt_free(list);

        // Fallback para params no resueltos del grupo
        for (int j = 0; j < grp->n_params; j++) {
            int pid = grp->param_ids[j];

            int  result_idx = -1;
            bool already_ok = false;
            for (int i = 0; i < valid_count; i++) {
                if (snapshot[i].param_id == pid) {
                    result_idx = i;
                    already_ok = results[i].ok;
                    break;
                }
            }
            if (already_ok || result_idx < 0) continue;

            _find_ctrl_ctx_t fc3 = { pid, -1 };
            kx_param_store_foreach(_find_ctrl_cb, &fc3);
            if (fc3.found_ctrl_id < 0) continue;

            const kx_param_t *p =
                kx_param_store_get_param(fc3.found_ctrl_id, pid);
            if (!p) continue;

            if (p->view == 0) { results[result_idx].ok = true; continue; }

            if (p->function_read == 0) {
                if (p->last_published_value != FLT_MAX) {
                    kx_pub_enqueue_status(pctx->pub_queue,
                                          kx_param_pub_status, PUB_KIND_STATUS,
                                          fc3.found_ctrl_id, pid,
                                          p->last_published_value);
                    results[result_idx].ok = true;
                } else {
                    kx_pub_enqueue_error(pctx->pub_queue, kx_param_pub_error,
                                          fc3.found_ctrl_id, pid,
                                          (uint16_t)p->reg,
                                          "write_only_no_value");
                    snprintf(results[result_idx].err_msg,
                             sizeof(results[result_idx].err_msg),
                             "write_only_no_value");
                }
                continue;
            }

            // Lectura individual bajo mutex
            const kx_control_t *ctrl_info =
                kx_param_store_get_ctrl(fc3.found_ctrl_id);
            if (!ctrl_info || ctrl_info->slave_addr == 0) continue;

            xSemaphoreTake(pctx->foreach_mutex, portMAX_DELAY);
            uint16_t raw = 0;
            float val = kx_modbus_read_reg(
                (uint8_t)ctrl_info->slave_addr,
                (uint16_t)p->reg,
                (uint8_t)p->function_read,
                p, &raw);
            int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

            if (val == -FLT_MAX) {
                kx_pub_enqueue_error(pctx->pub_queue, kx_param_pub_error,
                                      fc3.found_ctrl_id, pid,
                                      (uint16_t)p->reg, "modbus_timeout");
                snprintf(results[result_idx].err_msg,
                         sizeof(results[result_idx].err_msg),
                         "modbus_timeout");
            } else {
                kx_param_store_reg_upsert_read(
                    fc3.found_ctrl_id, (uint16_t)p->reg,
                    (uint8_t)p->function_read,
                    (uint8_t)p->function_write, val, ts_ms);
                kx_publish_all_params_for_reg(
                    pctx->pub_queue, kx_param_pub_status, PUB_KIND_STATUS,
                    fc3.found_ctrl_id, (uint16_t)p->reg,
                    (uint8_t)p->function_read, raw, ts_ms, -1);
                results[result_idx].ok = true;
            }
            xSemaphoreGive(pctx->foreach_mutex);
        }
    }

    // Recalcular contadores finales
    *out_ok = 0; *out_errors = 0;
    for (int i = 0; i < valid_count; i++) {
        if (results[i].ok)                      (*out_ok)++;
        else if (results[i].err_msg[0] != '\0') (*out_errors)++;
    }

    free(groups);
}

// =============================================================
// kx_modbus_task — tarea principal pipeline demand
// =============================================================
void kx_modbus_task(void *arg)
{
    kx_pipeline_ctx_t *ctx = (kx_pipeline_ctx_t *)arg;

    ESP_LOGI(TAG, "task started — waiting for entities...");
    while (!kx_param_store_is_ready())
        vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "entities ready (%d controls) — ready for poll demands",
             kx_param_store_count());

    while (*ctx->running) {
        xEventGroupSetBits(ctx->poll_eg, ctx->batch_active_bit);

        EventBits_t bits = xEventGroupWaitBits(
            ctx->poll_eg, ctx->demand_bit,
            pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & ctx->demand_bit)) continue;

        // Fase de recopilación de ráfaga
        {
            int64_t t0 = (int64_t)(esp_timer_get_time() / 1000ULL);
            int prev = -1, stable = 0;
            ESP_LOGI(TAG, "collecting burst...");
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(BURST_POLL_MS));
                int     cur = (int)uxQueueMessagesWaiting(ctx->demand_queue);
                int64_t ela = (int64_t)(esp_timer_get_time() / 1000ULL) - t0;
                if (cur == prev) {
                    stable += BURST_POLL_MS;
                    if (stable >= BURST_STABLE_MS) {
                        ESP_LOGI(TAG, "burst stable: %d demands in %" PRId64 "ms",
                                 cur, ela);
                        break;
                    }
                } else { stable = 0; }
                prev = cur;
                if (ela >= BURST_COLLECT_MAX_MS) {
                    ESP_LOGW(TAG, "burst timeout: %d demands", cur);
                    break;
                }
            }
        }

        int raw_count = (int)uxQueueMessagesWaiting(ctx->demand_queue);
        if (raw_count == 0) {
            xEventGroupClearBits(ctx->poll_eg, ctx->demand_bit);
            continue;
        }

        kx_poll_demand_t *snapshot =
            malloc((size_t)raw_count * sizeof(kx_poll_demand_t));
        if (!snapshot) {
            ESP_LOGE(TAG, "OOM snapshot (%d demands) — flushing", raw_count);
            kx_poll_demand_t tmp;
            while (xQueueReceive(ctx->demand_queue, &tmp, 0) == pdTRUE) {}
            xEventGroupClearBits(ctx->poll_eg, ctx->demand_bit);
            continue;
        }

        int expired = 0, dupes = 0;
        int valid_count = _drain_demand_queue(ctx->demand_queue,
                                               ctx->pending_bits,
                                               snapshot, raw_count,
                                               &expired, &dupes);
        xEventGroupClearBits(ctx->poll_eg, ctx->demand_bit);

        ESP_LOGI(TAG, "snapshot: raw=%d valid=%d expired=%d dupes=%d",
                 raw_count, valid_count, expired, dupes);

        if (valid_count == 0) { free(snapshot); continue; }

        // ── Full cycle? ───────────────────────────────────────
        bool has_full_cycle = false;
        for (int i = 0; i < valid_count; i++) {
            if (snapshot[i].param_id == 0) { has_full_cycle = true; break; }
        }

        if (has_full_cycle) {
            ESP_LOGI(TAG, "full poll cycle (demand_active)");
            if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
                ESP_LOGW(TAG, "full cycle skipped: no mqtt/store");
                free(snapshot);
                continue;
            }
            xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                                pdFALSE, pdTRUE, portMAX_DELAY);

            _count_ctx_t cc = { 0 };
            kx_param_store_foreach(_count_readable, &cc);

            ESP_LOGI(TAG, "full cycle: controls=%d readable=%d",
                     kx_param_store_count(), cc.count);

            _poll_ctrl_foreach_ud_t ud = {
                .pctx = ctx, .n_visited = 0, .ok = 0, .errors = 0
            };
            kx_param_store_foreach(_poll_ctrl_cb, &ud);

            ESP_LOGI(TAG, "full cycle done: ok=%d errors=%d heap=%" PRIu32,
                     ud.ok, ud.errors, kx_system_heap_free());
            free(snapshot);
            continue;
        }

        // ── Batch poll ────────────────────────────────────────
        ESP_LOGI(TAG, "batch poll: %d demands (dupes=%d expired=%d) — packetizing...",
                 valid_count, dupes, expired);

        _batch_result_t *results =
            calloc((size_t)valid_count, sizeof(_batch_result_t));
        if (!results) {
            ESP_LOGE(TAG, "OOM results array — aborting batch");
            free(snapshot);
            continue;
        }
        for (int i = 0; i < valid_count; i++)
            results[i].param_id = snapshot[i].param_id;

        int batch_ok = 0, batch_errors = 0, batch_dropped = 0,
            batch_packaged = 0;

        _poll_batch_packetized(ctx, snapshot, valid_count, results,
                               &batch_ok, &batch_errors, &batch_packaged);

        // ── Resumen ───────────────────────────────────────────
        int pub_hwm    = (int)uxQueueMessagesWaiting(ctx->pub_queue);
        int demand_hwm = (int)uxQueueMessagesWaiting(ctx->demand_queue);
        int write_hwm  = (int)uxQueueMessagesWaiting(ctx->write_queue);
        int batch_no_fc = valid_count - batch_packaged - batch_errors;
        if (batch_no_fc < 0) batch_no_fc = 0;

        printf("┌──────────────────────────────────────────────────────────────┐\n");
        printf("│                    BATCH POLL  RESUMEN                        │\n");
        printf("├──────────────────────────────────────────────────────────────┤\n");
        printf("│  Demandados  : %-5d  dupes=%-5d  expirados=%-5d           │\n",
               valid_count, dupes, expired);
        printf("│  Empaquetados: %-5d  sin_fc/view=%-5d                       │\n",
               batch_packaged, batch_no_fc);
        printf("│  Publicados  : %-5d  errores_mb=%-5d  drops=%-5d          │\n",
               batch_ok, batch_errors, batch_dropped);
        printf("├──────────────────────────────────────────────────────────────┤\n");
        printf("│  Heap libre  : %-8lu bytes                                │\n",
               (unsigned long)kx_system_heap_free());
        printf("│  pub_queue   : hwm=%-3d / %-5d slots                        │\n",
               pub_hwm, ctx->pub_queue_size);
        printf("│  demand_queue: hwm=%-3d / %-5d slots                        │\n",
               demand_hwm, ctx->demand_queue_size);
        printf("│  write_queue : hwm=%-3d / %-5d slots                        │\n",
               write_hwm, ctx->write_queue_size);

        if (batch_ok > 0) {
            printf("├──────────────────────────────────────────────────────────────┤\n");
            printf("│  ✓ Params OK:                                                │\n│    ");
            int col = 4;
            for (int i = 0; i < valid_count; i++) {
                if (!results[i].ok) continue;
                char tok[16];
                int tl = snprintf(tok, sizeof(tok), "%d", results[i].param_id);
                if (col + tl + 1 > 62) { printf("\n│    "); col = 4; }
                printf("%s ", tok);
                col += tl + 1;
            }
            printf("\n");
        }

        if (batch_errors > 0) {
            printf("├──────────────────────────────────────────────────────────────┤\n");
            printf("│  ✗ Params con error:                                         │\n");
            int shown = 0;
            for (int i = 0; i < valid_count && shown < 200; i++) {
                if (results[i].ok) continue;
                printf("│    param_id=%-10d  %s\n",
                       results[i].param_id, results[i].err_msg);
                shown++;
            }
        }

        printf("└──────────────────────────────────────────────────────────────┘\n");
        fflush(stdout);

        ESP_LOGI(TAG,
            "batch done -- demanded=%d packaged=%d no_fc=%d "
            "published=%d errors=%d drops=%d dupes=%d expired=%d heap=%" PRIu32,
            valid_count, batch_packaged, batch_no_fc,
            batch_ok, batch_errors, batch_dropped,
            dupes, expired, kx_system_heap_free());

        free(results);
        free(snapshot);
    }

    kx_modbus_uart_deinit();
    ESP_LOGI(TAG, "task stopped");
    vTaskDelete(NULL);
}