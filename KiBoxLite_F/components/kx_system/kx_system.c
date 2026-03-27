#include "kx_system.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "kx_system";

// ── Estado interno ────────────────────────────────────────────
static char s_device_id[KX_DEVICE_ID_LEN] = {0};
static uint32_t s_boot_count = 0;
static uint32_t s_reconnect_count = 0;

static volatile kx_net_state_t  s_net_state  = KX_NET_DISCONNECTED;
static volatile kx_mqtt_state_t s_mqtt_state = KX_MQTT_DISCONNECTED;

// ── NVS ───────────────────────────────────────────────────────
#define NVS_NS   "kx_sys"
#define NVS_KEY_BOOT "boot_cnt"

static void _load_boot_count(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }
    nvs_get_u32(h, NVS_KEY_BOOT, &s_boot_count);
    s_boot_count++;
    nvs_set_u32(h, NVS_KEY_BOOT, s_boot_count);
    nvs_commit(h);
    nvs_close(h);
}

// ── device_id desde MAC ───────────────────────────────────────
static void _init_device_id(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id),
             "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ── reset reason ──────────────────────────────────────────────
static const char *_reset_reason_str(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "power_on";
        case ESP_RST_SW:        return "sw_reset";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "int_watchdog";
        case ESP_RST_TASK_WDT:  return "task_watchdog";
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_SDIO:      return "sdio";
        default:                return "unknown";
    }
}

// ── API pública ───────────────────────────────────────────────
esp_err_t kx_system_init(void)
{
    // NVS init (si falla por partición llena, borrar y reintentar)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erasing and reinitializing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    _init_device_id();
    _load_boot_count();

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "  Kiconex Box Lite — FW %s", "0.1.0");
    ESP_LOGI(TAG, "  Device ID : %s", s_device_id);
    ESP_LOGI(TAG, "  Boot count: %" PRIu32, s_boot_count);
    ESP_LOGI(TAG, "  Reset reason: %s", _reset_reason_str());
    ESP_LOGI(TAG, "  Free heap : %" PRIu32 " bytes", kx_system_heap_free());
    ESP_LOGI(TAG, "================================================");

    return ESP_OK;
}

const char *kx_system_device_id(void)       { return s_device_id; }
const char *kx_system_reset_reason(void)    { return _reset_reason_str(); }
uint32_t    kx_system_boot_count(void)      { return s_boot_count; }
uint32_t    kx_system_reconnect_count(void) { return s_reconnect_count; }

uint32_t kx_system_uptime_s(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

uint32_t kx_system_heap_free(void)
{
    return (uint32_t)heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
}

void kx_system_set_net_state(kx_net_state_t s)
{
    s_net_state = s;
    ESP_LOGD(TAG, "net_state → %d", s);
}

kx_net_state_t kx_system_net_state(void) { return s_net_state; }

void kx_system_set_mqtt_state(kx_mqtt_state_t s)
{
    s_mqtt_state = s;
    ESP_LOGD(TAG, "mqtt_state → %d", s);
}

kx_mqtt_state_t kx_system_mqtt_state(void) { return s_mqtt_state; }

void kx_system_inc_reconnect_count(void) { s_reconnect_count++; }
