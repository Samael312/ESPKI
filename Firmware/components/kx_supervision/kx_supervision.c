#include "kx_supervision.h"
#include "kx_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "../../main/kx_config.h"

static const char *TAG = "kx_supervision";

#define SUPERVISION_INTERVAL_MS  10000   // log de salud cada 10 s
#define HEAP_WARN_THRESHOLD      30000   // warning si heap < 30 KB

static void _supervision_task(void *arg)
{
    // Registrar esta tarea en el watchdog
    esp_task_wdt_add(NULL);

    ESP_LOGI(TAG, "task started");

    while (1) {
        // Alimentar watchdog
        esp_task_wdt_reset();

        // Log de salud del sistema
        uint32_t heap = kx_system_heap_free();
        uint32_t uptime = kx_system_uptime_s();
        kx_net_state_t  net  = kx_system_net_state();
        kx_mqtt_state_t mqtt = kx_system_mqtt_state();

        ESP_LOGI(TAG,
                 "health | uptime=%lus heap=%lu net=%d mqtt=%d recon=%lu",
                 (unsigned long)uptime,
                 (unsigned long)heap,
                 (int)net,
                 (int)mqtt,
                 (unsigned long)kx_system_reconnect_count());

        // Advertir si la memoria está baja
        if (heap < HEAP_WARN_THRESHOLD) {
            ESP_LOGW(TAG, "LOW HEAP WARNING: %" PRIu32 " bytes free", heap);
        }

        vTaskDelay(pdMS_TO_TICKS(SUPERVISION_INTERVAL_MS));
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
