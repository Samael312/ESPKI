#include "kx_modbus_writer.h"
#include "kx_modbus_uart.h"
#include "kx_modbus_shared.h"
#include "kx_modbus_publish.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <float.h>
#include <inttypes.h>

static const char *TAG = "kx_writer";

// =============================================================
// kx_modbus_writer.c — Escrituras Modbus RTU
// =============================================================

// =============================================================
// _execute_write
// =============================================================
static esp_err_t _execute_write(int control_id, int param_id, float value)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) {
        ESP_LOGW(TAG, "write: param not found ctrl=%d param=%d",
                 control_id, param_id);
        return ESP_ERR_NOT_FOUND;
    }

    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "write: no slave_addr ctrl=%d", control_id);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL  &&
        fc_write != MB_FC_WRITE_SINGLE_REG   &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS) {
        ESP_LOGW(TAG, "write: unsupported FC 0x%02x param=%d", fc_write, param_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw;
    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        float adjusted = value - param->addition;
        if (param->offset != 0.0f && param->offset != 1.0f)
            raw = (int16_t)(adjusted / param->offset);
        else
            raw = (int16_t)adjusted;
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }

    ESP_LOGI(TAG, "write: ctrl=%d param=%d reg=0x%04x fc=0x%02x "
             "slave=%d value=%.3f → raw=%d (0x%04X)",
             control_id, param_id, param->reg, fc_write,
             ctrl->slave_addr, value, (int)(uint16_t)raw, (uint16_t)raw);

    uint8_t resp[16];
    int rx = -1;

    if (fc_write == MB_FC_WRITE_MULTIPLE_REGS) {
        uint8_t frame[9] = {
            (uint8_t)ctrl->slave_addr, MB_FC_WRITE_MULTIPLE_REGS,
            (uint8_t)((uint16_t)param->reg >> 8),
            (uint8_t)((uint16_t)param->reg & 0xFF),
            0x00, 0x01,
            0x02,
            (uint8_t)((uint16_t)raw >> 8),
            (uint8_t)((uint16_t)raw & 0xFF),
        };
        for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
            rx = kx_modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
            if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
        if (rx < 0) {
            ESP_LOGW(TAG, "write FC10: no response after %d retries ctrl=%d param=%d",
                     MODBUS_RETRY_COUNT, control_id, param_id);
            return ESP_FAIL;
        }
        if (rx < 6 || resp[0] != frame[0] || resp[1] != frame[1] ||
            resp[2] != frame[2] || resp[3] != frame[3]) {
            ESP_LOGW(TAG, "write FC10: unexpected response rx=%d", rx);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "write FC10: OK raw=%d (0x%04X)",
                 (int)(uint16_t)raw, (uint16_t)raw);
    } else {
        uint8_t frame[6] = {
            (uint8_t)ctrl->slave_addr, fc_write,
            (uint8_t)((uint16_t)param->reg >> 8),
            (uint8_t)((uint16_t)param->reg & 0xFF),
            (uint8_t)((uint16_t)raw >> 8),
            (uint8_t)((uint16_t)raw & 0xFF),
        };
        for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
            rx = kx_modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
            if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
        if (rx < 0) {
            ESP_LOGW(TAG, "write: no response after %d retries ctrl=%d param=%d",
                     MODBUS_RETRY_COUNT, control_id, param_id);
            return ESP_FAIL;
        }
        if (rx < 6 || resp[0] != frame[0] || resp[1] != frame[1] ||
            resp[2] != frame[2] || resp[3] != frame[3]) {
            ESP_LOGW(TAG, "write: unexpected response rx=%d", rx);
            return ESP_FAIL;
        }
        uint16_t echo = ((uint16_t)resp[4] << 8) | resp[5];
        ESP_LOGI(TAG, "write: OK raw_sent=%d raw_echo=%d",
                 (int)(uint16_t)raw, (int)echo);
    }

    return ESP_OK;
}

// =============================================================
// kx_writer_task
// =============================================================
void kx_writer_task(void *arg)
{
    kx_writer_ctx_t *ctx = (kx_writer_ctx_t *)arg;
    kx_write_cmd_t   cmd;

    ESP_LOGI(TAG, "writer task started");

    while (1) {
        if (xQueueReceive(ctx->write_queue, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        ESP_LOGI(TAG, "writer: cmd ctrl=%d param=%d value=%.3f ts=%.3f",
                 cmd.control_id, cmd.param_id, cmd.value, cmd.ts);

        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        xSemaphoreTake(ctx->foreach_mutex, portMAX_DELAY);

        esp_err_t err = _execute_write(cmd.control_id, cmd.param_id, cmd.value);

        if (err == ESP_OK) {
            kx_param_store_set_ts_set(cmd.control_id, cmd.param_id, cmd.ts);
            kx_pub_enqueue_status(ctx->pub_queue, kx_param_pub_status,
                                   PUB_KIND_STATUS,
                                   cmd.control_id, cmd.param_id, cmd.value);
            ESP_LOGI(TAG, "writer: OK ctrl=%d param=%d value=%.3f",
                     cmd.control_id, cmd.param_id, cmd.value);
        } else {
            kx_pub_enqueue_error(ctx->pub_queue, kx_param_pub_error,
                                  cmd.control_id, cmd.param_id,
                                  0, "modbus_write_error");
            ESP_LOGW(TAG, "writer: FAIL ctrl=%d param=%d err=%s",
                     cmd.control_id, cmd.param_id, esp_err_to_name(err));
        }

        xSemaphoreGive(ctx->foreach_mutex);
    }
}