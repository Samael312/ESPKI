#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include <sys/time.h>
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <inttypes.h>

static const char *TAG = "kx_telemetry";

// --- Funciones auxiliares ---

static int8_t _get_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return 0;
}

static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// --- Publicación de Controles (Infraestructura) ---

void kx_control_pub_status(int control_id, const char *uuid,
                            const char *connection_status)
{
    char topic[128];
    char payload[256];

    // Mantiene la ruta de control
    snprintf(topic, sizeof(topic), "%s/controls/%d/status", KX_DEVICE_UUID, control_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"_type\": \"control-status\","
        "\"id\": %d,"
        "\"uuid\": \"%s\","
        "\"connection_status\": \"%s\","
        "\"link\": {\"detected\": \"%s\"},"
        "\"timestamp\": %.3f"
        "}",
        control_id, uuid, connection_status, connection_status, _ts()
    );

    kx_mqtt_publish(topic, payload, 1, 0);
}

// --- Publicación de Entidades (Datos de Modbus) ---

/**
 * Publica el estado actual (STATUS).
 * Topic: {uuid}/quiiot/entities/{id}/status
 */
void kx_param_pub_status(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic), "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

/**
 * Publica el histórico (REPORT).
 * Topic: {uuid}/quiiot/entities/{id}/report
 */
void kx_param_pub_report(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic), "%s/quiiot/entities/%d/report", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

/**
 * Publica el comando de escritura (SET).
    * Topic: {uuid}/quiiot/entities/{id}/set
 */
void kx_param_pub_set(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic), "%s/quiiot/entities/%d/set", KX_DEVICE_UUID, param_id);

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

/**
 * Publica un error genérico.
 * Topic: {uuid}/quiiot/entities/{id}/status
 */
void kx_param_pub_error(int control_id, int param_id, const char *msg)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic), "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"error\":true,\"error_message\":\"%s\",\"ts\":%.3f}",
             param_id, msg, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

/**
 * Publica un error de Modbus detallado.
 * Topic: {uuid}/quiiot/entities/{id}/status
 */
void kx_param_pub_error_modbus(int control_id, int param_id, uint16_t reg, const char *msg)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic), "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"error\":true,\"error_message\":\"%s\","
             "\"reg\":\"0x%04x\",\"ts\":%.3f}",
             param_id, msg, reg, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

// --- Tarea y Control ---

static void _telemetry_task(void *arg)
{
    uint32_t seq = 0;
    ESP_LOGI(TAG, "Telemetry task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); 

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
        4096, // Tamaño de stack recomendado
        NULL,
        2,    // Prioridad estándar
        NULL
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}