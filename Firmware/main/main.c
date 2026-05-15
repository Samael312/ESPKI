#include "kx_config.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "kx_config_handler.h"
#include "kx_telemetry.h"
#include "kx_modbus_master.h"
#include "kx_param_store.h"
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
    ESP_LOGI(TAG, "NTP sync started");
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

static inline bool _is_quiiot_entities_topic(const char *topic)
{
    return (strncmp(topic, "quiiot/", 7) == 0 &&
            strstr(topic, "/entities/") != NULL);
}

static bool _topic_last_segment_ends_with_set(const char *topic)
{
    const char *last_slash = strrchr(topic, '/');
    if (!last_slash) return false;

    const char *seg  = last_slash + 1;   // p.ej. "7748348set" o "get"
    size_t      slen = strlen(seg);

    // Mínimo "1set" (4 chars): al menos un dígito + "set"
    return (slen >= 4 && strcmp(seg + slen - 3, "set") == 0);
}

// ── Router de mensajes MQTT entrantes ─────────────────────────
static void _on_mqtt_message(const char *topic, const char *payload, size_t len)
{
    uint32_t heap_before = kx_system_heap_free();

    ESP_LOGI(TAG, "RX topic=%s | len=%zu | heap=%" PRIu32,
             topic, len, heap_before);
    ESP_LOGD(TAG, "payload: %.*s", (int)len, payload);

    if (_is_quiiot_entities_topic(topic)) {

        if (_topic_last_segment_ends_with_set(topic)) {
            // Bloque 1: orden de escritura — procesar
            kx_param_handle_set(topic, payload, len);
        } else {
            // Bloque 2-3: "get" u otros — ignorar silenciosamente
            ESP_LOGD(TAG, "entities topic ignorado: %s", topic);
        }
        return;
    }

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

// ── Intentar cargar controles desde NVS ──────────────────────
// Restaura el store completo (metadatos + entities + update_ts).
// Si la caché es válida, el Modbus puede arrancar directamente.
// La comparación de update_ts ocurrirá cuando llegue el
// controls.json tras el controls, y el handler
// descartará la caché sólo si hay cambios.
static bool _try_load_from_nvs(void)
{
    kx_param_store_init();

    if (!kx_param_store_nvs_valid()) {
        ESP_LOGI(TAG, "NVS cache: miss (magic/uuid mismatch or empty)");
        return false;
    }

    ESP_LOGI(TAG, "NVS cache: HIT — loading controls + entities...");
    esp_err_t err = kx_param_store_load_nvs();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS load failed (%s), will download via MQTT",
                 esp_err_to_name(err));
        kx_param_store_clear_nvs();
        return false;
    }

    int count = kx_param_store_count();
    ESP_LOGI(TAG, "NVS cache: loaded %d controls", count);
    kx_param_store_set_expected(count);
    return true;
}

// ── Banner de arranque ────────────────────────────────────────
static void _print_boot_banner(bool from_cache)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Kiconex Box Lite  FW %-15s ║", KX_FW_VERSION);
    ESP_LOGI(TAG, "║  UUID: %.8s...                  ║", KX_DEVICE_UUID);
    ESP_LOGI(TAG, "║  Cache: %-28s ║",
             from_cache ? "controls+entities from NVS" : "will download via MQTT");
    ESP_LOGI(TAG, "║  Protocol: Modbus RTU               ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
}

// ── app_main ──────────────────────────────────────────────────
void app_main(void)
{
    // 1. Sistema base: NVS, device_id, boot count
    ESP_ERROR_CHECK(kx_system_init());

    // 2. Intentar cargar controles + entities desde NVS
    //    Si hay caché válida, el Modbus arrancará sin esperar MQTT.
    //    Cuando llegue el controls.json (tras controls)
    //    el handler comparará update_ts y actualizará lo que haya cambiado.
    bool from_cache = _try_load_from_nvs();
    _print_boot_banner(from_cache);

    // 3. WiFi — bloqueante hasta IP o timeout
    if (_wifi_init_sta() != ESP_OK) {
        ESP_LOGE(TAG, "no WiFi, rebooting in 10s");
        vTaskDelay(pdMS_TO_TICKS(10000));
        esp_restart();
    }

    // 4. NTP
    _ntp_init();

    // 5. MQTT — al conectarse publicará device-status online
    //    y el bridge responderá con device.json → controls
    //    → controls.json → entities-discovery (si ts mayor) → entities
    ESP_ERROR_CHECK(kx_mqtt_start(_on_mqtt_message));

    // 6. Telemetría periódica
    ESP_ERROR_CHECK(kx_telemetry_start());

    // 7. Modbus RTU maestro
    //    Si hay caché válida arranca de inmediato (kx_param_store_is_ready()
    //    ya es true). Si no, esperará en el bucle interno hasta que
    //    entities-discovery complete.
    ESP_ERROR_CHECK(kx_modbus_master_start());

    ESP_LOGI(TAG, "init done — device_id=%s fw=%s cache=%s",
             kx_system_device_id(), KX_FW_VERSION,
             from_cache ? "yes" : "no");

    while (1) {
        ESP_LOGI(TAG, "heap=%lu mqtt=%s modbus=%s controls=%d",
                 (unsigned long)kx_system_heap_free(),
                 kx_mqtt_is_connected()        ? "connected"    : "disconnected",
                 kx_modbus_master_is_running()  ? "running"      : "stopped",
                 kx_param_store_count());
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}