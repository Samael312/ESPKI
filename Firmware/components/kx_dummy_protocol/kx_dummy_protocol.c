#include "kx_dummy_protocol.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <sys/time.h>

// =============================================================
// kx_dummy_protocol.c — Simulador de protocolo de campo
//
// Replica el comportamiento exacto de kx_modbus pero sin bus
// físico: genera un valor uint16_t aleatorio dentro del rango raw
// y aplica la misma transformación (offset/addition/mask) que el
// driver real. Publica en los mismos topics.
//
// Para sustituir por el driver real:
//   1. En app_main, reemplaza kx_dummy_protocol_start()
//      por kx_modbus_start().
//   2. Elimina este componente del CMakeLists.txt principal.
// =============================================================

static const char *TAG = "kx_dummy_proto";

// ── Helpers ───────────────────────────────────────────────────

static float _rand_float(float min, float max)
{
    if (min >= max) return min;
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

// Misma lógica de transformación que kx_modbus._apply_transform:
//   valor_final = (raw * scale) + addition
// donde scale = offset si offset != 0, si no 1.
// El valor_final se clampea al rango [minvalue, maxvalue].
static float _apply_transform(const kx_param_t *p, uint16_t raw)
{
    float value = (float)raw;

    if (p->mask != 0) {
        value = (float)((uint16_t)raw & (uint16_t)p->mask);
    }

    float scale = (p->offset != 0.0f) ? p->offset : 1.0f;
    value = value * scale + p->addition;

    if (p->minvalue < p->maxvalue) {
        if (value < p->minvalue) value = p->minvalue;
        if (value > p->maxvalue) value = p->maxvalue;
    }

    return value;
}

// Genera un raw simulado proporcional al rango [minvalue, maxvalue],
// invirtiendo la transformación para que el valor final sea coherente.
//
// raw_target = (valor_deseado - addition) / scale
// valor_deseado se sortea uniforme en [minvalue, maxvalue].
static uint16_t _simulate_raw(const kx_param_t *p)
{
    float desired = _rand_float(p->minvalue, p->maxvalue);
    float scale   = (p->offset != 0.0f) ? p->offset : 1.0f;
    float raw_f   = (desired - p->addition) / scale;

    if (raw_f < 0.0f)       raw_f = 0.0f;
    if (raw_f > 65535.0f)   raw_f = 65535.0f;

    return (uint16_t)raw_f;
}

// ── Callback por cada param ───────────────────────────────────
static void _publish_param(int control_id,
                            const kx_param_t *param,
                            void *user_data)
{
    // Mismas condiciones de filtrado que el driver real
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;

    if (param->function_read != 0) {
        // Simular lectura: raw → transform → valor final
        uint16_t raw   = _simulate_raw(param);
        float    value = _apply_transform(param, raw);

        ESP_LOGI(TAG,
                 "ctrl=%d param=%d [%s] reg=%d raw=%u → %.3f",
                 control_id, param->param_id, param->name,
                 param->reg, raw, value);

        // Publica en: {uuid}/controls/{control_id}/entities/{param_id}/status
        kx_param_pub_status(control_id, param->param_id, value);

    } else {
        // Solo escritura: publicar el valor actual como "set"
        float value = _rand_float(param->minvalue, param->maxvalue);

        ESP_LOGI(TAG,
                 "ctrl=%d param=%d [%s] reg=%d (write-only) → %.3f",
                 control_id, param->param_id, param->name,
                 param->reg, value);

        // Publica en: {uuid}/controls/{control_id}/entities/{param_id}/set
        kx_param_pub_set(control_id, param->param_id, value);
    }
}

// ── Tarea del protocolo dummy ─────────────────────────────────
static void _dummy_protocol_task(void *arg)
{
    srand((unsigned int)esp_timer_get_time());
    ESP_LOGI(TAG, "task started, interval=%ds", KX_TELEMETRY_INTERVAL_S);

    ESP_LOGI(TAG, "waiting for all entities...");
    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "all entities ready: %d controls — waiting 2s",
             kx_param_store_count());
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip: mqtt not connected");
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        int total = kx_param_store_count();
        ESP_LOGI(TAG, "publishing telemetry for %d controls heap=%" PRIu32,
                 total, kx_system_heap_free());

        kx_param_store_foreach(_publish_param, NULL);

        ESP_LOGI(TAG, "telemetry done heap=%" PRIu32,
                 kx_system_heap_free());

        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
    }
}

// ── API pública ───────────────────────────────────────────────
esp_err_t kx_dummy_protocol_start(void)
{
    kx_param_store_init();

    BaseType_t ret = xTaskCreate(
        _dummy_protocol_task,
        "kx_dummy_proto",
        8192,
        NULL,
        KX_TASK_PRIO_TELEMETRY,
        NULL
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}