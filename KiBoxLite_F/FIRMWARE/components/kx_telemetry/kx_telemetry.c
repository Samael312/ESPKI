#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

static const char *TAG = "kx_telemetry";

static int8_t _get_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return 0;
}

static double _ts(void)
{
    return (double)esp_timer_get_time() / 1000000.0;
}

// topic: {uuid}/controls/{control_id}/status
void kx_control_pub_status(int control_id, const char *uuid,
                            const char *connection_status)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic),
             "%s/controls/%d/status",
             KX_DEVICE_UUID, control_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"_type\": \"control-status\","
        "\"id\": %d,"
        "\"uuid\": \"%s\","
        "\"connection_status\": \"%s\","
        "\"link\": {\"detected\": \"%s\"},"
        "\"timestamp\": %.3f"
        "}",
        control_id,
        uuid,
        connection_status,
        connection_status,
        _ts()
    );

    kx_mqtt_publish(topic, payload, 1, 0);
}

// ── Publica el valor de un param ──────────────────────────────

void kx_param_pub_status(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic),
             "%s/controls/%d/entities/%d/status",
             KX_DEVICE_UUID, control_id, param_id);

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

void kx_param_pub_set(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic),
             "%s/controls/%d/entities/%d/set",
             KX_DEVICE_UUID, control_id, param_id);

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

void kx_param_pub_error(int control_id, int param_id, const char *msg)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic),
             "%s/controls/%d/entities/%d/status",
             KX_DEVICE_UUID, control_id, param_id);

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"error\":true,\"error_message\":\"%s\",\"ts\":%.3f}",
             param_id, msg, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

// ── Tarea principal ───────────────────────────────────────────
// No hace nada por sí sola — el protocolo llama a kx_param_pub_*
// cuando tiene un valor listo. Esta tarea solo loguea el estado.
static void _telemetry_task(void *arg)
{
    uint32_t seq = 0;

    ESP_LOGI(TAG, "task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));  // log de estado cada 10 s

        ESP_LOGI(TAG, "alive seq=%" PRIu32 " heap=%" PRIu32 " rssi=%d mqtt=%s",
                 seq,
                 kx_system_heap_free(),
                 (int)_get_rssi(),
                 kx_mqtt_is_connected() ? "connected" : "disconnected");
        seq++;
    }
}

esp_err_t kx_telemetry_start(void)
{
    BaseType_t ret = xTaskCreate(
        _telemetry_task,
        "kx_telemetry",
        KX_TASK_STACK_TELEMETRY,
        NULL,
        KX_TASK_PRIO_TELEMETRY,
        NULL
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}