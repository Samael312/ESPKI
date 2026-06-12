#include "kx_modbus_report.h"
#include "kx_modbus_dispatch.h"
#include "kx_modbus_packetizer.h"
#include "kx_modbus_shared.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "kx_telemetry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "kx_report";

// =============================================================
// kx_modbus_report.c — Reports periódicos RTU
// =============================================================

#define REPORT_TICK_PERIOD_S   864000
#define REPORT_TASK_PERIOD_MS  1000
#define REPORT_LOG_MAX_PARAMS  256

#define MAX_CTRL_VISITED  KX_PARAM_MAX_CONTROLS

// ── Contexto de un tick de report ─────────────────────────────
typedef struct {
    int64_t tick_s;
    int     sent;
    int     errors;
    int     param_ids[REPORT_LOG_MAX_PARAMS];
    int     n_param_ids;
} _report_ctx_t;

// ── foreach: visita cada control una sola vez ─────────────────
typedef struct {
    _report_ctx_t     *rctx;
    kx_report_task_ctx_t *task_ctx;
    int                visited[MAX_CTRL_VISITED];
    int                n_visited;
} _report_foreach_ud_t;

static void _report_control(int control_id, _report_ctx_t *rctx,
                             kx_report_task_ctx_t *task_ctx)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    kx_packet_list_t *list = kx_pkt_build(control_id, false, NULL, 0,
                                           rctx->tick_s, now_ms);
    if (!list) return;

    // Acumular param_ids para el log
    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];
        for (int s = 0; s < pkt->num_slots; s++) {
            if (!pkt->slots[s].is_gap &&
                rctx->n_param_ids < REPORT_LOG_MAX_PARAMS) {
                rctx->param_ids[rctx->n_param_ids++] = pkt->slots[s].param_id;
            }
        }
    }

    kx_dispatch_control_packets(task_ctx->pub_queue,
                                 task_ctx->foreach_mutex,
                                 list,
                                 kx_param_pub_report,
                                 &rctx->sent, &rctx->errors,
                                 rctx->tick_s);
    kx_pkt_free(list);
}

static void _report_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    (void)param;
    _report_foreach_ud_t *u = (_report_foreach_ud_t *)ud;

    for (int i = 0; i < u->n_visited; i++)
        if (u->visited[i] == ctrl_id) return;

    if (u->n_visited < MAX_CTRL_VISITED)
        u->visited[u->n_visited++] = ctrl_id;

    _report_control(ctrl_id, u->rctx, u->task_ctx);
}

// =============================================================
// kx_report_task
// =============================================================
void kx_report_task(void *arg)
{
    kx_report_task_ctx_t *ctx = (kx_report_task_ctx_t *)arg;
    volatile int64_t tick_s   = -1;

    ESP_LOGI(TAG, "report task started (period=%ds max=%ds)",
             REPORT_TASK_PERIOD_MS / 1000, REPORT_TICK_PERIOD_S);

    while (*ctx->running) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_TASK_PERIOD_MS));

        if (!kx_param_store_is_ready() || !kx_mqtt_is_connected()) continue;

        tick_s = (tick_s + 1) % REPORT_TICK_PERIOD_S;

        ESP_LOGD(TAG, "report tick=%" PRId64 "s", tick_s);

        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        _report_ctx_t rctx = {
            .tick_s      = tick_s,
            .sent        = 0,
            .errors      = 0,
            .n_param_ids = 0,
        };
        _report_foreach_ud_t ud = { .rctx = &rctx, .task_ctx = ctx,
                                     .n_visited = 0 };
        kx_param_store_foreach(_report_ctrl_cb, &ud);

        if (rctx.sent > 0 || rctx.errors > 0) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║            REPORT  tick=%-6" PRId64 "           ║", tick_s);
            ESP_LOGI(TAG, "╠══════════════════════════════════════════╣");
            ESP_LOGI(TAG, "║  sent=%-4d  errors=%-4d  heap=%-8" PRIu32 " ║",
                     rctx.sent, rctx.errors, kx_system_heap_free());
            ESP_LOGI(TAG, "╠══════════════════════════════════════════╣");
            ESP_LOGI(TAG, "║  params (%d):", rctx.n_param_ids);
            for (int i = 0; i < rctx.n_param_ids; i += 6) {
                char row[128];
                int rpos = snprintf(row, sizeof(row), "║    ");
                for (int j = i; j < rctx.n_param_ids && j < i + 6; j++)
                    rpos += snprintf(row + rpos, sizeof(row) - rpos,
                                     "%-10d", rctx.param_ids[j]);
                ESP_LOGI(TAG, "%s", row);
            }
            ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
        }
    }

    vTaskDelete(NULL);
}