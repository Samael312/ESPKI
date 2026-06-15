#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "kx_telemetry";

// =============================================================
// kx_telemetry.c — Tarea de telemetría (alive log)
// =============================================================

static int8_t _get_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return 0;
}

static void _telemetry_task(void *arg)
{
    uint32_t seq = 0;
    ESP_LOGI(TAG, "Telemetry task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        ESP_LOGI(TAG,
                 "alive seq=%" PRIu32 " heap=%" PRIu32 " rssi=%d mqtt=%s",
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
        4096,
        NULL,
        2,
        NULL
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}