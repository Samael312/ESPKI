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
#include <math.h>

static const char *TAG = "kx_dummy_proto";

// ── Random con rango ──────────────────────────────────────────
static float _rand_float(float min, float max)
{
    if (min >= max) return min;
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

// ── Callback que publica un valor por cada param ──────────────
static void _publish_param(int control_id,
                            const kx_param_t *param,
                            void *user_data)
{
    // solo publicar params con lectura o escritura habilitada
    if (param->function_read == 0 && param->function_write == 0) return;

    float value = _rand_float(param->minvalue, param->maxvalue);

    // aplicar offset si existe
    if (param->offset != 0.0f && param->offset != 1.0f) {
        value = value * param->offset;
    }

    if (param->function_read != 0) {
        kx_param_pub_status(control_id, param->param_id, value);
    } else {
        kx_param_pub_set(control_id, param->param_id, value);
    }

    ESP_LOGD(TAG, "ctrl=%d param=%d [%s] value=%.3f (%.2f-%.2f)",
             control_id, param->param_id, param->name,
             value, param->minvalue, param->maxvalue);
}

// ── Tarea del protocolo dummy ─────────────────────────────────
static void _dummy_protocol_task(void *arg)
{
    srand((unsigned int)esp_timer_get_time());
    ESP_LOGI(TAG, "task started, interval=%ds", KX_TELEMETRY_INTERVAL_S);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));

        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip: mqtt not connected");
            continue;
        }

        int total_controls = kx_param_store_count();
        if (total_controls == 0) {
            ESP_LOGD(TAG, "no controls stored yet, waiting...");
            continue;
        }

        ESP_LOGI(TAG, "publishing telemetry for %d controls", total_controls);
        kx_param_store_foreach(_publish_param, NULL);
        ESP_LOGI(TAG, "telemetry done heap=%" PRIu32, kx_system_heap_free());
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