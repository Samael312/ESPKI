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

static esp_err_t _execute_tcp_write(const kx_tcp_writer_ctx_t *ctx,
                                     int control_id, int param_id, float value)
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

    // ── Pausa el poll para exclusividad en el socket ──────────
    // El writer ya tiene foreach_mutex, pero el pipeline puede estar
    // entre paquetes (mutex liberado). Limpiar poll_allowed_bit garantiza
    // que ningún otro acceso al socket ocurra durante la escritura.
    xEventGroupClearBits(ctx->poll_eg, ctx->poll_allowed_bit);

    int rx = KX_TCP_RX_NET_ERROR;
    for (int a = 0; a < KX_TCP_RETRY_COUNT && rx == KX_TCP_RX_NET_ERROR; a++) {
        kx_tcp_sock_set_recv_timeout(sock_idx, TCP_RECV_TIMEOUT_WRITE_MS);
        rx = kx_tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                                 resp, sizeof(resp));
        kx_tcp_sock_set_recv_timeout(sock_idx, TCP_RECV_TIMEOUT_READ_MS);
        if (rx == KX_TCP_RX_NET_ERROR) vTaskDelay(pdMS_TO_TICKS(50));
    }

    xEventGroupSetBits(ctx->poll_eg, ctx->poll_allowed_bit);

    if (rx == KX_TCP_RX_MODBUS_EXCEPT) {
        ESP_LOGW(TAG, "write Modbus exception ctrl=%d param=%d reg=0x%04x",
                 control_id, param_id, param->reg);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (rx < 0) {
        ESP_LOGW(TAG, "write no response rx=%d", rx);
        return ESP_FAIL;
    }

    // Verificar respuesta mínima:
    // FC06: MBAP(6) + unit(1) + fc(1) + reg(2) + val(2) = 12 bytes
    // FC10: MBAP(6) + unit(1) + fc(1) + reg(2) + count(2) = 12 bytes
    // FC05: igual que FC06 = 12 bytes
    if (rx < 12) {
        ESP_LOGW(TAG, "write response too short: %d bytes", rx);
        return ESP_FAIL;
    }

    // Verificar que el FC de respuesta coincide (no bit de excepción)
    if (resp[7] != fc_write) {
        ESP_LOGW(TAG, "write FC mismatch: sent=0x%02x got=0x%02x",
                 fc_write, resp[7]);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "write OK ctrl=%d param=%d value=%.3f",
             control_id, param_id, value);
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

        // Esperar a que el poll esté permitido antes de tomar el mutex,
        // para no bloquear el pipeline indefinidamente durante una pausa.
        xEventGroupWaitBits(ctx->poll_eg, ctx->poll_allowed_bit,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        // Tomar el mutex para exclusividad con pipeline y report.
        // _execute_tcp_write pausará el poll internamente durante
        // la transacción para evitar colisiones de socket.
        xSemaphoreTake(ctx->foreach_mutex, portMAX_DELAY);

        esp_err_t err = _execute_tcp_write(ctx, cmd.control_id, cmd.param_id,
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