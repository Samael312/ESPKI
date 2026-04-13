#include "kx_dummy_protocol.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

static const char *TAG = "kx_dummy_proto";

// ── Random con rango ──────────────────────────────────────────
static float _rand_float(float min, float max)
{
    if (min >= max) return min;
    return min + ((float)rand() / (float)RAND_MAX) * (max - min);
}

// ── Timestamp en segundos desde epoch ─────────────────────
static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// ── Callback — publica un valor por cada param ────────────────
static void _publish_param(int control_id,
                            const kx_param_t *param,
                            void *user_data)
{
    // ignorar params sin lectura ni escritura
    if (param->function_read == 0 && param->function_write == 0) return;

    // ignorar params ocultos
    if (param->view == 0) return;

    // generar valor random dentro del rango del param
    float value = _rand_float(param->minvalue, param->maxvalue);

    // aplicar offset si es multiplicativo
    if (param->offset != 0.0f && param->offset != 1.0f) {
        value = value * param->offset;
    }

    char topic[128];
    char payload[128];

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param->param_id, value, _ts());

    if (param->function_read != 0) {
        // lectura → publicar en report Y status
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/report",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);
        ESP_LOGI(TAG, "→ report ctrl=%d param=%d [%s] value=%.3f",
                 control_id, param->param_id, param->name, value);

        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/status",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);
        ESP_LOGI(TAG, "→ status ctrl=%d param=%d [%s] value=%.3f",
                 control_id, param->param_id, param->name, value);

    } else {
        // escritura → set
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/set",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);
        ESP_LOGI(TAG, "→ set ctrl=%d param=%d [%s] value=%.3f",
                 control_id, param->param_id, param->name, value);
    }
}

// ── Tarea del protocolo dummy ─────────────────────────────────
static void _dummy_protocol_task(void *arg)
{
    srand((unsigned int)esp_timer_get_time());
    ESP_LOGI(TAG, "task started, interval=%ds", KX_TELEMETRY_INTERVAL_S);

    // esperar a que todos los controls tengan entities
    ESP_LOGI(TAG, "waiting for all entities...");
    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // margen extra para que el backend procese antes de recibir telemetría
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
        ESP_LOGI(TAG, "publishing telemetry for %d controls", total);
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