#include "kx_config.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "kx_config_handler.h"
#include "kx_telemetry.h"
#include "kx_dummy_protocol.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "main";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_events;
static int s_wifi_retry = 0;

// ── WiFi event handler ────────────────────────────────────────
static void _wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
            kx_system_set_net_state(KX_NET_CONNECTING);

        } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
            kx_system_set_net_state(KX_NET_DISCONNECTED);
            if (s_wifi_retry < KX_WIFI_MAX_RETRY) {
                s_wifi_retry++;
                ESP_LOGW(TAG, "WiFi retry %d/%d", s_wifi_retry, KX_WIFI_MAX_RETRY);
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_wifi_retry = 0;
        kx_system_set_net_state(KX_NET_CONNECTED);
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

// ── WiFi init ─────────────────────────────────────────────────
static esp_err_t _wifi_init_sta(void)
{
    s_wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t h_wifi, h_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, _wifi_event_handler, NULL, &h_wifi));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, _wifi_event_handler, NULL, &h_ip));

    wifi_config_t wifi_cfg = {};
    memcpy(wifi_cfg.sta.ssid,     KX_WIFI_SSID,     strlen(KX_WIFI_SSID));
    memcpy(wifi_cfg.sta.password, KX_WIFI_PASSWORD, strlen(KX_WIFI_PASSWORD));
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "connecting to: %s", KX_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(30000)
    );

    if (bits & WIFI_CONNECTED_BIT) return ESP_OK;

    ESP_LOGE(TAG, "WiFi failed");
    return ESP_FAIL;
}

// ── Router de mensajes MQTT entrantes ─────────────────────────
static void _on_mqtt_message(const char *topic, const char *payload, size_t len)
{
    uint32_t heap_before = kx_system_heap_free();

    ESP_LOGI(TAG, "RX topic=%s | len=%zu | heap=%" PRIu32,
             topic, len, heap_before);
    ESP_LOGD(TAG, "payload: %.*s", (int)len, payload);

    if (strstr(topic, "/controls")) {
        kx_config_handle(topic, payload, len);
        return;
    }

    if (strstr(topic, KX_DEVICE_UUID)) {
        kx_config_handle(topic, payload, len);
        return;
    }

    ESP_LOGW(TAG, "unhandled topic: %s", topic);
}

// ── app_main ──────────────────────────────────────────────────
void app_main(void)
{
    // 1. sistema base: NVS, device_id, boot count
    ESP_ERROR_CHECK(kx_system_init());

    // 2. WiFi — bloqueante hasta IP o timeout
    if (_wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "no WiFi, rebooting in 10s");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }

    // 3. MQTT — publica device-status, suscribe topics
    ESP_ERROR_CHECK(kx_mqtt_start(_on_mqtt_message));

    // 4. Telemetría — tarea de publicación de estado
    //ESP_ERROR_CHECK(kx_telemetry_start());

    // 5. Protocolo dummy — simula lecturas de campo hasta Fase 2
    //    En Fase 2: sustituir por kx_modbus_start()
    //ESP_ERROR_CHECK(kx_dummy_protocol_start());

    ESP_LOGI(TAG, "init done — device_id=%s fw=%s",
             kx_system_device_id(), KX_FW_VERSION);

    // loop de diagnóstico
    while (1) {
        ESP_LOGI(TAG, "heap=%lu mqtt=%s",
                 (unsigned long)kx_system_heap_free(),
                 kx_mqtt_is_connected() ? "connected" : "disconnected");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}