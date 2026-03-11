#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <math.h>
#include <time.h>

static const char *TAG = "kx_telemetry";

// ── RSSI ──────────────────────────────────────────────────────
static int8_t _get_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return 0;  // 0 = no disponible
}

// ── Datos dummy deterministas ─────────────────────────────────
// counter  = seq incremental
// wave     = sin(seq * 15°) → ciclo completo cada 24 muestras
// fixed    = 1.0 siempre (referencia)
static void _build_payload(char *buf, size_t len, uint32_t seq)
{
    double wave = sin((double)seq * M_PI / 12.0);  // 15° = π/12 rad

    snprintf(buf, len,
        "{"
        "\"device_id\":\"%s\","
        "\"ts\":%lu,"
        "\"seq\":%" PRIu32 ","
        "\"uptime_s\":%" PRIu32 ","
        "\"rssi\":%d,"
        "\"heap_free\":%" PRIu32 ","
        "\"fw_ver\":\"%s\","
        "\"reset_reason\":\"%s\","
        "\"payload\":{"
            "\"counter\":%" PRIu32 ","
            "\"wave\":%.4f,"
            "\"fixed\":1.0"
        "}"
        "}",
        kx_system_device_id(),
        (unsigned long)time(NULL),
        seq,
        kx_system_uptime_s(),
        (int)_get_rssi(),
        kx_system_heap_free(),
        KX_FW_VERSION,
        kx_system_reset_reason(),
        seq,
        wave
    );
}

// ── Tarea ─────────────────────────────────────────────────────
static void _telemetry_task(void *arg)
{
    char topic[64];
    char payload[KX_PAYLOAD_MAX_BYTES];
    uint32_t seq = 0;

    snprintf(topic, sizeof(topic), KX_TOPIC_TELEMETRY, kx_system_device_id());

    ESP_LOGI(TAG, "task started, interval=%ds", KX_TELEMETRY_INTERVAL_S);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));

        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip: mqtt not connected");
            continue;
        }

        _build_payload(payload, sizeof(payload), seq);

        esp_err_t err = kx_mqtt_publish(topic, payload, 0 /* QoS 0 */, 0);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "published seq=%" PRIu32 " heap=%" PRIu32,
                     seq, kx_system_heap_free());
            seq++;
        } else {
            ESP_LOGW(TAG, "publish failed seq=%" PRIu32, seq);
            // No incrementa seq para que el receptor detecte la laguna
        }
    }
}

// ── API pública ───────────────────────────────────────────────
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
