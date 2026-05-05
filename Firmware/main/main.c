#include "kx_config.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "kx_config_handler.h"
#include "kx_telemetry.h"
#include "kx_modbus_master.h"    
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string.h>

static const char *TAG = "main";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t s_wifi_events;
static int s_wifi_retry = 0;

// ── NTP ───────────────────────────────────────────────────────
static void _ntp_init(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    ESP_LOGI(TAG, "NTP sync iniciado");
}

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
                ESP_LOGW(TAG, "WiFi reintento %d/%d",
                         s_wifi_retry, KX_WIFI_MAX_RETRY);
                esp_wifi_connect();
            } else {
                xEventGroupSetBits(s_wifi_events, WIFI_FAIL_BIT);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&ev->ip_info.ip));
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

    ESP_LOGI(TAG, "conectando a: %s", KX_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_events,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(30000)
    );

    if (bits & WIFI_CONNECTED_BIT) return ESP_OK;

    ESP_LOGE(TAG, "WiFi fallido");
    return ESP_FAIL;
}

// ── Router MQTT ───────────────────────────────────────────────
static void _on_mqtt_message(const char *topic, const char *payload, size_t len)
{
    uint32_t heap_before = kx_system_heap_free();

    ESP_LOGI(TAG, "RX topic=%s | len=%zu | heap=%" PRIu32,
             topic, len, heap_before);
    ESP_LOGD(TAG, "payload: %.*s", (int)len, payload);

    if (strstr(topic, "/controls") || strstr(topic, KX_DEVICE_UUID)) {
        kx_config_handle(topic, payload, len);
        return;
    }

    ESP_LOGW(TAG, "topic sin handler: %s", topic);
}

// ── Tarea de test Modbus (solo con KX_MODBUS_TEST_MODE) ───────
#ifdef KX_MODBUS_TEST_MODE
static void _modbus_test_task(void *arg)
{
    ESP_LOGI(TAG, "║         MODO TEST MODBUS ACTIVO          ║");
    

    // Esperar a que UART esté listo
    vTaskDelay(pdMS_TO_TICKS(2000));

#ifdef KX_MODBUS_SCAN_ON_BOOT
    // 1. Escanear el bus para descubrir esclavos
    kx_modbus_scan(KX_MODBUS_SCAN_FROM, KX_MODBUS_SCAN_TO);
    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

    // 2. Test de lectura repetido cada 5 s
    int test_count = 0;
    while (1) {
        ESP_LOGI(TAG, "── Test #%d ─────────────────────────────", ++test_count);
        kx_modbus_test_read(
            KX_MODBUS_TEST_SLAVE,
            KX_MODBUS_TEST_FC,
            KX_MODBUS_TEST_REG,
            KX_MODBUS_TEST_QTY
        );
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
#endif  // KX_MODBUS_TEST_MODE

// ── app_main ──────────────────────────────────────────────────
void app_main(void)
{
    // 1. Sistema base: NVS, device_id, boot count
    ESP_ERROR_CHECK(kx_system_init());

    // 2. WiFi — bloqueante hasta IP o timeout
    if (_wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "sin WiFi, reiniciando en 10s");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }

    // 3. NTP
    _ntp_init();

    // 4. MQTT
    ESP_ERROR_CHECK(kx_mqtt_start(_on_mqtt_message));

    // 5. Telemetría (tarea de supervisión de estado, log cada 10s)
    ESP_ERROR_CHECK(kx_telemetry_start());

    // 6. Modbus RTU
#ifdef KX_MODBUS_TEST_MODE
    // — Modo test: solo inicializa UART y ejecuta lecturas manuales
    ESP_ERROR_CHECK(kx_modbus_init());
    xTaskCreate(_modbus_test_task, "mb_test", 4096, NULL,
                KX_TASK_PRIO_TELEMETRY, NULL);
#else
    // — Modo producción: polling completo guiado por kx_param_store
    ESP_ERROR_CHECK(kx_modbus_init());
#endif

    ESP_LOGI(TAG, "init completado — device_id=%s fw=%s",
             kx_system_device_id(), KX_FW_VERSION);

    while (1) {
        ESP_LOGI(TAG, "heap=%lu mqtt=%s",
                 (unsigned long)kx_system_heap_free(),
                 kx_mqtt_is_connected() ? "connected" : "disconnected");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}