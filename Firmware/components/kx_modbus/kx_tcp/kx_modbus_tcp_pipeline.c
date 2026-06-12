#include "kx_modbus_tcp_pipeline.h"
#include "kx_modbus_tcp_dispatch.h"
#include "kx_modbus_tcp_socket.h"
#include "kx_modbus_packetizer.h"
#include "kx_modbus_shared.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../../main/kx_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

static const char *TAG = "kx_tcp_pipeline";

// =============================================================
// kx_modbus_tcp_pipeline.c — Tarea demand TCP
// =============================================================

#define MAX_CTRL_VISITED      KX_PARAM_MAX_CONTROLS
#define MAX_PARAMS_IN_BATCH   512

#define BURST_COLLECT_MAX_MS  2000
#define BURST_STABLE_MS        200
#define BURST_POLL_MS           50

typedef struct { int target; int found; } _find_ctrl_ctx_t;

static void _find_ctrl_cb(int ctrl_id, const kx_param_t *p, void *ud)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)ud;
    if (ctx->found < 0 && p->param_id == ctx->target)
        ctx->found = ctrl_id;
}

// =============================================================
// _collect_tcp_ctrls_cb / kx_tcp_get_ctrl_ids
//
// Devuelve la lista de control_id que usan proto TCP.
// Pasado como callback a kx_tcp_report_task.
// =============================================================
typedef struct { int ctrl_ids[MAX_CTRL_VISITED]; int n; } _tcp_ctrl_list_t;

static void _collect_tcp_ctrls_cb(int ctrl_id, const kx_param_t *p, void *ud)
{
    (void)p;
    _tcp_ctrl_list_t *lst = (_tcp_ctrl_list_t *)ud;
    if (kx_param_store_get_proto(ctrl_id) != KX_PROTO_TCP) return;
    for (int i = 0; i < lst->n; i++)
        if (lst->ctrl_ids[i] == ctrl_id) return;
    if (lst->n < MAX_CTRL_VISITED)
        lst->ctrl_ids[lst->n++] = ctrl_id;
}

int kx_tcp_get_ctrl_ids(int *out, int max)
{
    _tcp_ctrl_list_t lst = { .n = 0 };
    kx_param_store_foreach(_collect_tcp_ctrls_cb, &lst);
    int n = lst.n < max ? lst.n : max;
    memcpy(out, lst.ctrl_ids, (size_t)n * sizeof(int));
    return n;
}

// =============================================================
// _poll_tcp_control — full cycle de un control completo
// =============================================================
static void _poll_tcp_control(kx_tcp_pipeline_ctx_t *ctx, int ctrl_id,
                               int *out_ok, int *out_errors)
{
    char ip[40]; uint16_t port;
    if (kx_param_store_get_tcp_endpoint(ctrl_id, ip, &port) != ESP_OK) return;

    int sock_idx = kx_tcp_sock_get_or_connect(ip, port);
    if (sock_idx < 0) { ESP_LOGE(TAG, "poll ctrl=%d: no socket", ctrl_id); return; }

    const kx_control_t *ctrl = kx_param_store_get_ctrl(ctrl_id);
    if (!ctrl) return;
    uint8_t unit_id = (uint8_t)ctrl->slave_addr;

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    kx_packet_list_t *list = kx_pkt_build(ctrl_id, true, NULL, 0, 0, now_ms);
    if (!list) return;

    for (int i = 0; i < list->count; i++) {
        xSemaphoreTake(ctx->foreach_mutex, portMAX_DELAY);
        int pkt_ok = 0, pkt_err = 0;
        pkt_ok = kx_dispatch_packet_tcp(ctx->pub_queue, ctx->foreach_mutex,
                                          sock_idx, unit_id, &list->pkts[i],
                                          PUB_KIND_STATUS, &pkt_err, -1);
        xSemaphoreGive(ctx->foreach_mutex);
        if (out_ok)     *out_ok     += pkt_ok;
        if (out_errors) *out_errors += pkt_err;
        if (i + 1 < list->count) vTaskDelay(pdMS_TO_TICKS(10));
    }
    kx_pkt_free(list);
}

// =============================================================
// _poll_batch_tcp — grupo de param_ids TCP demandados
// =============================================================
typedef struct {
    int ctrl_id;
    int param_ids[MAX_PARAMS_IN_BATCH];
    int n_params;
} _ctrl_group_t;

static void _poll_batch_tcp(kx_tcp_pipeline_ctx_t   *ctx,
                             const kx_poll_demand_t  *snap,
                             int                      valid,
                             int                     *out_ok,
                             int                     *out_errors)
{
    _ctrl_group_t *groups = malloc(MAX_CTRL_VISITED * sizeof(_ctrl_group_t));
    if (!groups) { ESP_LOGE(TAG, "batch OOM"); return; }
    int n_groups = 0;

    for (int i = 0; i < valid; i++) {
        _find_ctrl_ctx_t fctx = { snap[i].param_id, -1 };
        kx_param_store_foreach(_find_ctrl_cb, &fctx);
        if (fctx.found < 0) { (*out_errors)++; continue; }
        if (kx_param_store_get_proto(fctx.found) != KX_PROTO_TCP) continue;

        int g = -1;
        for (int j = 0; j < n_groups; j++)
            if (groups[j].ctrl_id == fctx.found) { g = j; break; }
        if (g < 0) {
            if (n_groups >= MAX_CTRL_VISITED) { (*out_errors)++; continue; }
            g = n_groups++;
            groups[g].ctrl_id  = fctx.found;
            groups[g].n_params = 0;
        }
        if (groups[g].n_params < MAX_PARAMS_IN_BATCH)
            groups[g].param_ids[groups[g].n_params++] = snap[i].param_id;
    }

    for (int g = 0; g < n_groups && *ctx->running; g++) {
        _ctrl_group_t *grp = &groups[g];
        char ip[40]; uint16_t port;
        if (kx_param_store_get_tcp_endpoint(grp->ctrl_id, ip, &port) != ESP_OK)
            continue;

        int sock_idx = kx_tcp_sock_get_or_connect(ip, port);
        if (sock_idx < 0) { *out_errors += grp->n_params; continue; }

        const kx_control_t *ctrl = kx_param_store_get_ctrl(grp->ctrl_id);
        if (!ctrl) continue;
        uint8_t unit_id = (uint8_t)ctrl->slave_addr;

        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
        kx_packet_list_t *list = kx_pkt_build(grp->ctrl_id, true,
                                               grp->param_ids, grp->n_params,
                                               0, now_ms);
        if (!list) { *out_errors += grp->n_params; continue; }

        for (int i = 0; i < list->count; i++) {
            xSemaphoreTake(ctx->foreach_mutex, portMAX_DELAY);
            int pkt_err = 0;
            int pkt_ok = kx_dispatch_packet_tcp(
                            ctx->pub_queue, ctx->foreach_mutex,
                            sock_idx, unit_id, &list->pkts[i],
                            PUB_KIND_STATUS, &pkt_err, -1);
            xSemaphoreGive(ctx->foreach_mutex);
            *out_ok     += pkt_ok;
            *out_errors += pkt_err;
            if (i + 1 < list->count) vTaskDelay(pdMS_TO_TICKS(10));
        }
        kx_pkt_free(list);
    }
    free(groups);
}

// =============================================================
// _drain_demand_queue
// =============================================================
static int _drain_demand_queue(QueueHandle_t      demand_queue,
                                volatile uint8_t  *pending_bits,
                                kx_poll_demand_t  *snap, int cap,
                                int *out_exp, int *out_dup)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    int exp = 0, dup = 0, count = 0;
    kx_poll_demand_t d;

    while (count < cap && xQueueReceive(demand_queue, &d, 0) == pdTRUE) {
        KX_PENDING_CLEAR(pending_bits, d.param_id);
        if ((now_ms - d.enqueued_ms) > (int64_t)(KX_DEMAND_TIMEOUT_S * 1000)) {
            exp++; continue;
        }
        bool found = false;
        for (int j = 0; j < count; j++) {
            if (snap[j].param_id == d.param_id) {
                if (d.enqueued_ms > snap[j].enqueued_ms)
                    snap[j].enqueued_ms = d.enqueued_ms;
                dup++; found = true; break;
            }
        }
        if (!found) snap[count++] = d;
    }
    while (xQueueReceive(demand_queue, &d, 0) == pdTRUE) {}
    if (out_exp) *out_exp = exp;
    if (out_dup) *out_dup = dup;
    return count;
}

// =============================================================
// kx_tcp_demand_task
// =============================================================
void kx_tcp_demand_task(void *arg)
{
    kx_tcp_pipeline_ctx_t *ctx = (kx_tcp_pipeline_ctx_t *)arg;

    ESP_LOGI(TAG, "demand task started — waiting for entities...");
    while (!kx_param_store_is_ready()) vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "entities ready — TCP demand task running");

    while (*ctx->running) {
        EventBits_t bits = xEventGroupWaitBits(ctx->poll_eg, ctx->demand_bit,
                                               pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & ctx->demand_bit)) continue;

        // Recopilar ráfaga
        {
            int64_t t0 = (int64_t)(esp_timer_get_time() / 1000ULL);
            int prev = -1, stable = 0;
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(BURST_POLL_MS));
                int cur = (int)uxQueueMessagesWaiting(ctx->demand_queue);
                int64_t ela = (int64_t)(esp_timer_get_time() / 1000ULL) - t0;
                if (cur == prev) {
                    stable += BURST_POLL_MS;
                    if (stable >= BURST_STABLE_MS) break;
                } else { stable = 0; }
                prev = cur;
                if (ela >= BURST_COLLECT_MAX_MS) break;
            }
        }

        int raw_count = (int)uxQueueMessagesWaiting(ctx->demand_queue);
        if (raw_count == 0) {
            xEventGroupClearBits(ctx->poll_eg, ctx->demand_bit);
            continue;
        }

        kx_poll_demand_t *snap =
            malloc((size_t)raw_count * sizeof(kx_poll_demand_t));
        if (!snap) {
            kx_poll_demand_t tmp;
            while (xQueueReceive(ctx->demand_queue, &tmp, 0) == pdTRUE) {}
            xEventGroupClearBits(ctx->poll_eg, ctx->demand_bit);
            continue;
        }

        int exp = 0, dup = 0;
        int valid = _drain_demand_queue(ctx->demand_queue, ctx->pending_bits,
                                         snap, raw_count, &exp, &dup);
        xEventGroupClearBits(ctx->poll_eg, ctx->demand_bit);

        if (valid == 0) { free(snap); continue; }
        if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
            free(snap); continue;
        }

        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int ok = 0, err = 0;

        bool full_cycle = false;
        for (int i = 0; i < valid; i++)
            if (snap[i].param_id == 0) { full_cycle = true; break; }

        if (full_cycle) {
            int ctrl_ids[MAX_CTRL_VISITED];
            int n = kx_tcp_get_ctrl_ids(ctrl_ids, MAX_CTRL_VISITED);
            for (int i = 0; i < n && *ctx->running; i++)
                _poll_tcp_control(ctx, ctrl_ids[i], &ok, &err);
        } else {
            _poll_batch_tcp(ctx, snap, valid, &ok, &err);
        }

        ESP_LOGI(TAG, "TCP poll done: ok=%d err=%d dup=%d exp=%d heap=%" PRIu32,
                 ok, err, dup, exp, kx_system_heap_free());
        free(snap);
    }
    vTaskDelete(NULL);
}