#include "kx_modbus_master.h"
#include "kx_modbus_pipeline.h"
#include "kx_modbus_report.h"
#include "kx_modbus_writer.h"
#include "kx_modbus_uart.h"
#include "kx_modbus_shared.h"
#include "kx_modbus_publish.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "kx_telemetry.h"
#include "../../../main/kx_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "kx_modbus";

// =============================================================
// kx_modbus_master.c — Arranque, colas y API pública RTU
// =============================================================

// ── Tamaños de cola ───────────────────────────────────────────
#define PUB_QUEUE_SIZE     500
#define DEMAND_QUEUE_SIZE  1500
#define WRITE_QUEUE_SIZE    64

// ── Bits de evento ────────────────────────────────────────────
#define POLL_ALLOWED_BIT   BIT0
#define DEMAND_BIT         BIT1
#define BATCH_ACTIVE_BIT   BIT2

// ── Estado del driver ─────────────────────────────────────────
static QueueHandle_t       s_pub_queue    = NULL;
static QueueHandle_t       s_demand_queue = NULL;
static QueueHandle_t       s_write_queue  = NULL;
static EventGroupHandle_t  s_poll_eg      = NULL;
static SemaphoreHandle_t   s_foreach_mutex = NULL;
static volatile bool       s_running      = false;
static TaskHandle_t        s_task         = NULL;

// Bitmap dedup demandas
static volatile uint8_t s_pending_bits[KX_PENDING_SET_SIZE / 8] = {0};

// Contextos de tareas (estáticos — viven mientras el driver corre)
static kx_pipeline_ctx_t  s_pipeline_ctx;
static kx_report_task_ctx_t s_report_ctx;
static kx_writer_ctx_t    s_writer_ctx;

// =============================================================
// Tarea publisher
// =============================================================
static void _publisher_task(void *arg)
{
    kx_pub_result_t r;
    ESP_LOGI(TAG, "publisher task started (queue_size=%d)", PUB_QUEUE_SIZE);
    while (1) {
        if (xQueueReceive(s_pub_queue, &r, portMAX_DELAY) == pdTRUE) {
            if (r.pub_fn)
                r.pub_fn(r.control_id, r.param_id, r.value);
            else if (r.pub_err_fn)
                r.pub_err_fn(r.control_id, r.param_id,
                             r.error_msg, r.reg);
        }
    }
}

// =============================================================
// API pública — encolar escritura
// =============================================================
esp_err_t kx_modbus_enqueue_write(int control_id, int param_id,
                                   float value, double ts)
{
    if (!s_write_queue) return ESP_ERR_INVALID_STATE;

    kx_write_cmd_t cmd = { control_id, param_id, value, ts };
    if (xQueueSend(s_write_queue, &cmd, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGE(TAG, "write_queue FULL — dropping ctrl=%d param=%d",
                 control_id, param_id);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "write enqueued: ctrl=%d param=%d value=%.3f ts=%.3f",
             control_id, param_id, value, ts);
    return ESP_OK;
}

// =============================================================
// API pública — demanda de polling
// =============================================================
void kx_modbus_request_poll(int param_id)
{
    if (!s_demand_queue) return;
    if (!kx_param_store_is_ready()) {
        ESP_LOGD(TAG, "demand rejected: store not ready (param_id=%d)", param_id);
        return;
    }
    if (param_id != 0 && KX_PENDING_TEST(s_pending_bits, param_id)) {
        ESP_LOGD(TAG, "demand dedup param_id=%d", param_id);
        return;
    }

    int     used   = (int)uxQueueMessagesWaiting(s_demand_queue);
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    if (used >= DEMAND_QUEUE_SIZE / 2)
        ESP_LOGW(TAG, "demand_queue near full: %d/%d", used, DEMAND_QUEUE_SIZE);

    kx_poll_demand_t d = { param_id, now_ms };
    if (xQueueSend(s_demand_queue, &d, 0) != pdTRUE) {
        ESP_LOGE(TAG, "demand_queue FULL (%d), dropping param_id=%d",
                 DEMAND_QUEUE_SIZE, param_id);
        return;
    }
    if (param_id != 0) KX_PENDING_SET(s_pending_bits, param_id);
    if (s_poll_eg) xEventGroupSetBits(s_poll_eg, DEMAND_BIT);
    ESP_LOGD(TAG, "demand enqueued param_id=%d queue=%d", param_id, used + 1);
}

// =============================================================
// pause / resume
// =============================================================
void kx_modbus_pause(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    ESP_LOGI(TAG, "pausing Modbus...");
    xEventGroupClearBits(s_poll_eg, POLL_ALLOWED_BIT);
    if (xSemaphoreTake(s_foreach_mutex, pdMS_TO_TICKS(60000)) != pdTRUE)
        ESP_LOGE(TAG, "pause: timeout — memory risk!");
    else
        ESP_LOGI(TAG, "Modbus paused");
}

void kx_modbus_resume(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xSemaphoreGive(s_foreach_mutex);
    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    ESP_LOGI(TAG, "Modbus resumed");
}

// =============================================================
// start
// =============================================================
esp_err_t kx_modbus_master_start(void)
{
    if (s_running) { ESP_LOGW(TAG, "already running"); return ESP_OK; }

    s_pub_queue = xQueueCreate(PUB_QUEUE_SIZE, sizeof(kx_pub_result_t));
    if (!s_pub_queue) { ESP_LOGE(TAG, "pub_queue alloc failed"); return ESP_FAIL; }

    s_demand_queue = xQueueCreate(DEMAND_QUEUE_SIZE, sizeof(kx_poll_demand_t));
    if (!s_demand_queue) { ESP_LOGE(TAG, "demand_queue alloc failed"); return ESP_FAIL; }

    s_write_queue = xQueueCreate(WRITE_QUEUE_SIZE, sizeof(kx_write_cmd_t));
    if (!s_write_queue) { ESP_LOGE(TAG, "write_queue alloc failed"); return ESP_FAIL; }

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) { ESP_LOGE(TAG, "EventGroup alloc failed"); return ESP_FAIL; }

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) { ESP_LOGE(TAG, "mutex alloc failed"); return ESP_FAIL; }

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);

    esp_err_t err = kx_modbus_uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init: %s", esp_err_to_name(err));
        return err;
    }

    s_running = true;
    memset((void *)s_pending_bits, 0, sizeof(s_pending_bits));

    // ── Contexto pipeline ─────────────────────────────────────
    s_pipeline_ctx = (kx_pipeline_ctx_t){
        .pub_queue         = s_pub_queue,
        .demand_queue      = s_demand_queue,
        .write_queue       = s_write_queue,
        .pub_queue_size    = PUB_QUEUE_SIZE,
        .demand_queue_size = DEMAND_QUEUE_SIZE,
        .write_queue_size  = WRITE_QUEUE_SIZE,
        .foreach_mutex     = s_foreach_mutex,
        .poll_eg           = s_poll_eg,
        .poll_allowed_bit  = POLL_ALLOWED_BIT,
        .demand_bit        = DEMAND_BIT,
        .batch_active_bit  = BATCH_ACTIVE_BIT,
        .pending_bits      = s_pending_bits,
        .running           = &s_running,
    };

    // ── Contexto report ───────────────────────────────────────
    s_report_ctx = (kx_report_task_ctx_t){
        .pub_queue        = s_pub_queue,
        .foreach_mutex    = s_foreach_mutex,
        .poll_eg          = s_poll_eg,
        .poll_allowed_bit = POLL_ALLOWED_BIT,
        .running          = &s_running,
    };

    // ── Contexto writer ───────────────────────────────────────
    s_writer_ctx = (kx_writer_ctx_t){
        .write_queue      = s_write_queue,
        .pub_queue        = s_pub_queue,
        .foreach_mutex    = s_foreach_mutex,
        .poll_eg          = s_poll_eg,
        .poll_allowed_bit = POLL_ALLOWED_BIT,
    };

    BaseType_t ret;

    ret = xTaskCreate(_publisher_task, "kx_publisher", 4096, NULL,
                      KX_TASK_PRIO_TELEMETRY - 1, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "publisher task failed"); return ESP_FAIL; }

    ret = xTaskCreate(kx_writer_task, "kx_writer", 4096,
                      &s_writer_ctx, KX_TASK_PRIO_TELEMETRY + 2, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "writer task failed"); return ESP_FAIL; }

    ret = xTaskCreate(kx_report_task, "kx_report", 4096,
                      &s_report_ctx, KX_TASK_PRIO_TELEMETRY, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "report task failed"); return ESP_FAIL; }

    ret = xTaskCreate(kx_modbus_task, "kx_modbus", 8192,
                      &s_pipeline_ctx, KX_TASK_PRIO_TELEMETRY + 1, &s_task);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ESP_LOGI(TAG,
             "started — writer prio=%d poll prio=%d report prio=%d pub prio=%d",
             KX_TASK_PRIO_TELEMETRY + 2,
             KX_TASK_PRIO_TELEMETRY + 1,
             KX_TASK_PRIO_TELEMETRY,
             KX_TASK_PRIO_TELEMETRY - 1);

    return ESP_OK;
}

void kx_modbus_master_stop(void)       { s_running = false; }
bool kx_modbus_master_is_running(void) { return s_running; }

esp_err_t kx_modbus_master_ensure_started(void)
{
    if (s_running) return ESP_OK;
    ESP_LOGI(TAG, "first RTU control detected — starting RTU driver");
    return kx_modbus_master_start();
}