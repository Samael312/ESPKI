#include "kx_modbus_tcp_writer.h"
#include "kx_modbus_tcp_socket.h"
#include "kx_modbus_shared.h"
#include "kx_modbus_publish.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <float.h>
#include <inttypes.h>

static const char *TAG = "kx_tcp_writer";

// =============================================================
// kx_modbus_tcp_writer.c — Escrituras Modbus TCP
// =============================================================

static esp_err_t _execute_tcp_write(int control_id, int param_id, float value)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) return ESP_ERR_NOT_FOUND;

    char ip[40]; uint16_t port;
    if (kx_param_store_get_tcp_endpoint(control_id, ip, &port) != ESP_OK)
        return ESP_ERR_INVALID_STATE;

    int sock_idx = kx_tcp_sock_get_or_connect(ip, port);
    if (sock_idx < 0) return ESP_FAIL;

    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return ESP_ERR_INVALID_STATE;
    uint8_t unit_id = (uint8_t)ctrl->slave_addr;

    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL  &&
        fc_write != MB_FC_WRITE_SINGLE_REG   &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS)
        return ESP_ERR_NOT_SUPPORTED;

    int16_t raw;
    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        float adj = value - param->addition;
        raw = (param->offset != 0.0f && param->offset != 1.0f)
              ? (int16_t)(adj / param->offset) : (int16_t)adj;
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }

    uint8_t pdu[5] = { fc_write,
        (uint8_t)((uint16_t)param->reg >> 8),
        (uint8_t)((uint16_t)param->reg & 0xFF),
        (uint8_t)((uint16_t)raw >> 8),
        (uint8_t)((uint16_t)raw & 0xFF) };
    uint8_t resp[KX_TCP_RESPONSE_BUF];

    ESP_LOGI(TAG, "write ctrl=%d param=%d reg=0x%04x fc=0x%02x "
             "value=%.3f → raw=%d",
             control_id, param_id, param->reg, fc_write,
             value, (int)(uint16_t)raw);

    int rx = -1;
    for (int a = 0; a < KX_TCP_RETRY_COUNT && rx < 0; a++) {
        rx = kx_tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                                 resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (rx < 12) {
        ESP_LOGW(TAG, "write no response rx=%d", rx);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "write OK");
    return ESP_OK;
}

// =============================================================
// kx_tcp_writer_task
// =============================================================
void kx_tcp_writer_task(void *arg)
{
    kx_tcp_writer_ctx_t *ctx = (kx_tcp_writer_ctx_t *)arg;
    kx_write_cmd_t       cmd;

    ESP_LOGI(TAG, "writer task started");

    while (1) {
        if (xQueueReceive(ctx->write_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        xSemaphoreTake(ctx->foreach_mutex, portMAX_DELAY);

        esp_err_t err = _execute_tcp_write(cmd.control_id, cmd.param_id,
                                            cmd.value);
        if (err == ESP_OK) {
            kx_param_store_set_ts_set(cmd.control_id, cmd.param_id, cmd.ts);
            kx_pub_enqueue_status(ctx->pub_queue, kx_param_pub_status,
                                   PUB_KIND_STATUS,
                                   cmd.control_id, cmd.param_id, cmd.value);
        } else {
            kx_pub_enqueue_error(ctx->pub_queue, kx_param_pub_error,
                                  cmd.control_id, cmd.param_id,
                                  0, "tcp_write_error");
        }

        xSemaphoreGive(ctx->foreach_mutex);
    }
}