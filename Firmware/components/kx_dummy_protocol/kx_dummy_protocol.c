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

// ── Timestamp real desde epoch ────────────────────────────────
static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// ── Publica el valor de un parámetro (idéntico a kx_modbus_master) ──
//
// topic saliente: {uuid}/quiiot/entities/{param_id}/report   (si función lectura)
//                 {uuid}/quiiot/entities/{param_id}/status   (si función lectura)
//                 {uuid}/quiiot/entities/{param_id}/set      (si solo escritura)
//
static void _publish_param(int control_id,
                            const kx_param_t *param,
                            void *user_data)
{
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;

    float value = _rand_float(param->minvalue, param->maxvalue);

    if (param->offset != 0.0f && param->offset != 1.0f) {
        value = value * param->offset;
    }

    char topic[128];
    char payload[128];

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param->param_id, value, _ts());

    if (param->function_read != 0) {
        // report
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/report",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);

        // status
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/status",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);

        ESP_LOGD(TAG, "→ dummy ctrl=%d param=%d [%s] value=%.3f",
                 control_id, param->param_id, param->name, value);
    } else {
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/set",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);

        ESP_LOGD(TAG, "→ dummy set ctrl=%d param=%d [%s] value=%.3f",
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

    // persistir en NVS para el próximo arranque
    ESP_LOGI(TAG, "all entities ready (%d controls) — saving to NVS...",
             kx_param_store_count());
    esp_err_t save_err = kx_param_store_save_nvs();
    if (save_err == ESP_OK) {
        ESP_LOGI(TAG, "entities cached in NVS OK");
    } else {
        ESP_LOGW(TAG, "NVS save failed: %s", esp_err_to_name(save_err));
    }

    // margen para que el backend procese antes de recibir telemetría
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip: mqtt not connected");
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        ESP_LOGI(TAG, "publishing dummy telemetry (%d controls)",
                 kx_param_store_count());
        kx_param_store_foreach(_publish_param, NULL);
        ESP_LOGI(TAG, "telemetry done heap=%" PRIu32, kx_system_heap_free());

        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
    }
}

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