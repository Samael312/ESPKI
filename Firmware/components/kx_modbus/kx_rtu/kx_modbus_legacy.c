// =============================================================
// kx_modbus_legacy.c — Código no utilizado actualmente
//
// Estas funciones existían en kx_modbus_master.c pero no tienen
// llamadores activos en el sistema. Se conservan aquí por si
// se necesitan en el futuro.
//
// NO incluir en CMakeLists.txt SRCS hasta que se necesiten.
// =============================================================

#if 0  // Deshabilitado — descomentar si se necesita

#include "kx_modbus_master.h"
#include "kx_modbus_uart.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include <float.h>
#include <math.h>

static const char *TAG = "kx_legacy";

// ── _has_changed ──────────────────────────────────────────────
// Umbral de cambio para publicación de status.
// Declarada pero nunca llamada en la implementación actual
// (el pipeline publica siempre sin filtrar por cambio).
// Si se quiere publicación condicional, llamar desde
// _publish_all_params_for_reg antes de encolar.

#define KX_STATUS_DELTA_ABS   0.5f
#define KX_STATUS_DELTA_REL   0.01f

static inline bool _has_changed(float new_val, float last_val)
{
    if (last_val == FLT_MAX) return true;
    float delta = fabsf(new_val - last_val);
    if (delta > KX_STATUS_DELTA_ABS) return true;
    if (last_val != 0.0f &&
        (delta / fabsf(last_val)) > KX_STATUS_DELTA_REL) return true;
    return false;
}

// ── kx_modbus_read_one ────────────────────────────────────────
// Lee un parámetro individual por demanda directa (sin cola).
// Sustituido por kx_modbus_request_poll() + pipeline.
esp_err_t kx_modbus_read_one(int control_id, int param_id)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) return ESP_ERR_NOT_FOUND;

    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return ESP_ERR_INVALID_STATE;

    float value = kx_modbus_read_reg(
        (uint8_t)ctrl->slave_addr,
        (uint16_t)param->reg,
        (uint8_t)param->function_read,
        param, NULL);

    if (value == -FLT_MAX) {
        ESP_LOGW(TAG, "read_one: timeout ctrl=%d param=%d", control_id, param_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "read_one: ctrl=%d param=%d value=%.3f",
             control_id, param_id, value);
    return ESP_OK;
}

// ── kx_modbus_write_one ───────────────────────────────────────
// Escribe un parámetro directamente (sin cola).
// Sustituido por kx_modbus_enqueue_write() + writer task.
// CUIDADO: llamar esto fuera de la writer task puede generar
// colisiones en el bus UART si hay polling activo.
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value)
{
    ESP_LOGW(TAG, "write_one: llamada directa sin cola — "
             "usar kx_modbus_enqueue_write() en su lugar");
    (void)control_id;
    (void)param_id;
    (void)value;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif // 0