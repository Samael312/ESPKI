#include "kx_modbus_tcp.h"
#include "kx_modbus_tcp_pipeline.h"
#include "kx_modbus_tcp_report.h"
#include "kx_modbus_tcp_writer.h"
#include "kx_modbus_tcp_socket.h"
#include "kx_modbus_shared.h"
#include "kx_param_store.h"
#include "kx_system.h"
#include "kx_telemetry.h"
#include "../../../main/kx_config.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "kx_modbus_tcp";

// =============================================================
// kx_modbus_tcp.c — Arranque, colas y API pública TCP
// =============================================================

// ── Tamaños de cola (más pequeñas que RTU — ver justificación
//    histórica: 1 PLC, LAN local, sin ráfagas masivas) ────────
#define PUB_QUEUE_SIZE       64
#define WRITE_QUEUE_SIZE     16
#define DEMAND_QUEUE_SIZE   256

// ── Stacks de tareas ──────────────────────────────────────────
#define STACK_TCP_DEMAND   5120
#define STACK_TCP_WRITER   3072
#define STACK_TCP_REPORT   3072
#define STACK_TCP_PUB      4096   // subido de 2048 — stack overflow fix

// ── Bits de evento ────────────────────────────────────────────
#define POLL_ALLOWED_BIT  BIT0
#define DEMAND_BIT        BIT1

// ── Estado del driver ─────────────────────────────────────────
static QueueHandle_t      s_pub_queue    = NULL;
static QueueHandle_t      s_write_queue  = NULL;
static QueueHandle_t      s_demand_queue = NULL;
static EventGroupHandle_t s_poll_eg      = NULL;
static SemaphoreHandle_t  s_foreach_mutex = NULL;
static volatile bool      s_running      = false;

static volatile uint8_t s_pending_bits[KX_PENDING_SET_SIZE / 8] = {0};

static kx_tcp_pipeline_ctx_t s_pipeline_ctx;
static kx_tcp_report_ctx_t   s_report_ctx;
static kx_tcp_writer_ctx_t   s_writer_ctx;

// =============================================================
// Tarea publisher
// =============================================================
static void _tcp_publisher_task(void *arg)
{
    kx_pub_result_t r;
    ESP_LOGI(TAG, "publisher task started");
    while (1) {
        if (xQueueReceive(s_pub_queue, &r, portMAX_DELAY) == pdTRUE) {
            switch (r.kind) {
            case PUB_KIND_STATUS:
                kx_param_pub_status(r.control_id, r.param_id, r.value);
                break;
            case PUB_KIND_REPORT:
                kx_param_pub_report(r.control_id, r.param_id, r.value);
                break;
            case PUB_KIND_ERROR:
                kx_param_pub_error(r.control_id, r.param_id, r.error_msg, r.reg);
                break;
            }
        }
    }
}

// =============================================================
// API pública
// =============================================================
esp_err_t kx_modbus_tcp_start(void)
{
    if (s_running) { ESP_LOGW(TAG, "already running"); return ESP_OK; }

    if (kx_tcp_socket_init() != ESP_OK) return ESP_FAIL;

    s_pub_queue = xQueueCreate(PUB_QUEUE_SIZE, sizeof(kx_pub_result_t));
    if (!s_pub_queue) return ESP_FAIL;

    {
    static StaticQueue_t s_demand_queue_struct;
    static DRAM_ATTR uint8_t s_demand_queue_storage[DEMAND_QUEUE_SIZE * sizeof(kx_poll_demand_t)];
    s_demand_queue = xQueueCreateStatic(
        DEMAND_QUEUE_SIZE,
        sizeof(kx_poll_demand_t),
        s_demand_queue_storage,
        &s_demand_queue_struct
    );
    }
    if (!s_demand_queue) return ESP_FAIL;

    s_write_queue = xQueueCreate(WRITE_QUEUE_SIZE, sizeof(kx_write_cmd_t));
    if (!s_write_queue) return ESP_FAIL;

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) return ESP_FAIL;

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) return ESP_FAIL;

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    s_running = true;
    memset((void *)s_pending_bits, 0, sizeof(s_pending_bits));

    // ── Contextos ──────────────────────────────────────────────
    s_pipeline_ctx = (kx_tcp_pipeline_ctx_t){
        .pub_queue        = s_pub_queue,
        .demand_queue     = s_demand_queue,
        .foreach_mutex    = s_foreach_mutex,
        .poll_eg          = s_poll_eg,
        .poll_allowed_bit = POLL_ALLOWED_BIT,
        .demand_bit       = DEMAND_BIT,
        .pending_bits     = s_pending_bits,
        .running          = &s_running,
    };

    s_report_ctx = (kx_tcp_report_ctx_t){
        .pub_queue        = s_pub_queue,
        .foreach_mutex    = s_foreach_mutex,
        .poll_eg          = s_poll_eg,
        .poll_allowed_bit = POLL_ALLOWED_BIT,
        .running          = &s_running,
        .get_tcp_ctrl_ids = kx_tcp_get_ctrl_ids,
    };

    s_writer_ctx = (kx_tcp_writer_ctx_t){
        .write_queue      = s_write_queue,
        .pub_queue        = s_pub_queue,
        .foreach_mutex    = s_foreach_mutex,
        .poll_eg          = s_poll_eg,
        .poll_allowed_bit = POLL_ALLOWED_BIT,
    };

    BaseType_t ret;
    ret = xTaskCreate(_tcp_publisher_task, "kx_tcp_pub",
                      STACK_TCP_PUB, NULL, KX_TASK_PRIO_TELEMETRY - 1, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ret = xTaskCreate(kx_tcp_writer_task, "kx_tcp_writer",
                      STACK_TCP_WRITER, &s_writer_ctx,
                      KX_TASK_PRIO_TELEMETRY + 2, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ret = xTaskCreate(kx_tcp_report_task, "kx_tcp_report",
                      STACK_TCP_REPORT, &s_report_ctx,
                      KX_TASK_PRIO_TELEMETRY, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ret = xTaskCreate(kx_tcp_demand_task, "kx_tcp_demand",
                      STACK_TCP_DEMAND, &s_pipeline_ctx,
                      KX_TASK_PRIO_TELEMETRY + 1, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ESP_LOGI(TAG,
             "started — heap_free=%" PRIu32 " bytes"
             " | pub_q=%d demand_q=%d write_q=%d"
             " | stacks: demand=%d writer=%d report=%d pub=%d",
             kx_system_heap_free(),
             PUB_QUEUE_SIZE, DEMAND_QUEUE_SIZE, WRITE_QUEUE_SIZE,
             STACK_TCP_DEMAND, STACK_TCP_WRITER, STACK_TCP_REPORT, STACK_TCP_PUB);
    return ESP_OK;
}

void kx_modbus_tcp_stop(void)       { s_running = false; }
bool kx_modbus_tcp_is_running(void) { return s_running; }

esp_err_t kx_modbus_tcp_ensure_started(void)
{
    if (s_running) return ESP_OK;
    ESP_LOGI(TAG, "first TCP control detected — starting TCP driver");
    return kx_modbus_tcp_start();
}

void kx_modbus_tcp_request_poll(int param_id)
{
    if (!s_demand_queue || !s_running) return;
    if (!kx_param_store_is_ready()) return;
    if (param_id != 0 && KX_PENDING_TEST(s_pending_bits, param_id)) return;

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    kx_poll_demand_t d = { param_id, now_ms };
    if (xQueueSend(s_demand_queue, &d, 0) != pdTRUE) {
        ESP_LOGE(TAG, "demand_queue FULL param_id=%d", param_id);
        return;
    }
    if (param_id != 0) KX_PENDING_SET(s_pending_bits, param_id);
    if (s_poll_eg) xEventGroupSetBits(s_poll_eg, DEMAND_BIT);
}

esp_err_t kx_modbus_tcp_enqueue_write(int    control_id,
                                       int    param_id,
                                       float  value,
                                       double ts)
{
    if (!s_write_queue) return ESP_ERR_INVALID_STATE;
    kx_write_cmd_t cmd = { control_id, param_id, value, ts };
    return (xQueueSend(s_write_queue, &cmd, pdMS_TO_TICKS(200)) == pdTRUE)
           ? ESP_OK : ESP_ERR_NO_MEM;
}

void kx_modbus_tcp_pause(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xEventGroupClearBits(s_poll_eg, POLL_ALLOWED_BIT);
    if (xSemaphoreTake(s_foreach_mutex, pdMS_TO_TICKS(60000)) != pdTRUE)
        ESP_LOGE(TAG, "pause timeout");
    else
        ESP_LOGI(TAG, "TCP paused");
}

void kx_modbus_tcp_resume(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xSemaphoreGive(s_foreach_mutex);
    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    ESP_LOGI(TAG, "TCP resumed");
}