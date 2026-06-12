#include "kx_modbus_tcp_report.h"
#include "kx_modbus_tcp_dispatch.h"
#include "kx_modbus_tcp_socket.h"
#include "kx_modbus_packetizer.h"
#include "kx_modbus_shared.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "kx_tcp_report";

// =============================================================
// kx_modbus_tcp_report.c — Reports periódicos TCP
// =============================================================

#define REPORT_TICK_PERIOD_S   864000
#define REPORT_TASK_PERIOD_MS  1000

void kx_tcp_report_task(void *arg)
{
    kx_tcp_report_ctx_t *ctx = (kx_tcp_report_ctx_t *)arg;
    volatile int64_t tick_s  = -1;

    ESP_LOGI(TAG, "report task started");

    while (*ctx->running) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_TASK_PERIOD_MS));
        if (!kx_param_store_is_ready() || !kx_mqtt_is_connected()) continue;

        tick_s = (tick_s + 1) % REPORT_TICK_PERIOD_S;

        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int ctrl_ids[KX_PARAM_MAX_CONTROLS];
        int n_ctrls = ctx->get_tcp_ctrl_ids(ctrl_ids, KX_PARAM_MAX_CONTROLS);

        int sent = 0, errs = 0;
        for (int c = 0; c < n_ctrls && *ctx->running; c++) {
            char ip[40]; uint16_t port;
            if (kx_param_store_get_tcp_endpoint(ctrl_ids[c], ip, &port) != ESP_OK)
                continue;
            int sock_idx = kx_tcp_sock_get_or_connect(ip, port);
            if (sock_idx < 0) continue;

            const kx_control_t *ctrl = kx_param_store_get_ctrl(ctrl_ids[c]);
            if (!ctrl) continue;
            uint8_t unit_id = (uint8_t)ctrl->slave_addr;

            int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
            kx_packet_list_t *list = kx_pkt_build(ctrl_ids[c], false,
                                                   NULL, 0, tick_s, now_ms);
            if (!list) continue;

            for (int i = 0; i < list->count; i++) {
                xSemaphoreTake(ctx->foreach_mutex, portMAX_DELAY);
                int pkt_err = 0;
                int pkt_ok = kx_dispatch_packet_tcp(
                                ctx->pub_queue, ctx->foreach_mutex,
                                sock_idx, unit_id, &list->pkts[i],
                                PUB_KIND_REPORT, &pkt_err, tick_s);
                xSemaphoreGive(ctx->foreach_mutex);
                sent += pkt_ok; errs += pkt_err;
                if (i + 1 < list->count) vTaskDelay(pdMS_TO_TICKS(10));
            }
            kx_pkt_free(list);
        }

        if (sent > 0 || errs > 0)
            ESP_LOGI(TAG, "TCP report tick=%" PRId64 " sent=%d err=%d heap=%" PRIu32,
                     tick_s, sent, errs, kx_system_heap_free());
    }
    vTaskDelete(NULL);
}