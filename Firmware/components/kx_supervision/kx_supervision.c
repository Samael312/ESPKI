#include "kx_supervision.h"
#include "kx_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../main/kx_config.h"

static const char *TAG = "kx_supervision";

#define WDT_FEED_MS       4000
#define LOG_INTERVAL_MS  10000
#define HEAP_WARN_THRESHOLD 30000

static void _supervision_task(void *arg)
{
    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "task started");

    int64_t last_log_ms = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(WDT_FEED_MS));
        esp_task_wdt_reset();

        int64_t now_ms = esp_timer_get_time() / 1000;
        int64_t elapsed_ms = now_ms - last_log_ms;

        if (elapsed_ms >= LOG_INTERVAL_MS) {
            last_log_ms = now_ms;

            uint32_t heap = kx_system_heap_free();

            ESP_LOGI(TAG,
                     "health | uptime=%lus heap=%lu net=%d mqtt=%d recon=%lu elapsed=%llums",
                     (unsigned long)kx_system_uptime_s(),
                     (unsigned long)heap,
                     (int)kx_system_net_state(),
                     (int)kx_system_mqtt_state(),
                     (unsigned long)kx_system_reconnect_count(),
                     (long long)elapsed_ms);

            if (heap < HEAP_WARN_THRESHOLD) {
                ESP_LOGW(TAG, "LOW HEAP WARNING: %" PRIu32 " bytes free", heap);
            }
        }
    }
}

esp_err_t kx_supervision_start(void)
{
    BaseType_t ret = xTaskCreate(
        _supervision_task,
        "kx_supervision",
        KX_TASK_STACK_SUPERVISION,
        NULL,
        KX_TASK_PRIO_SUPERVISION,
        NULL
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}