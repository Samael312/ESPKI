#include "kx_modbus_master.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "kx_telemetry.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <float.h>
#include <math.h>

static const char *TAG = "kx_modbus";

// =============================================================
// Parámetros UART/RS-485
// =============================================================
#ifndef KX_MODBUS_UART_NUM
#define KX_MODBUS_UART_NUM   UART_NUM_1
#endif
#ifndef KX_MODBUS_BAUD
#define KX_MODBUS_BAUD       9600
#endif
#ifndef KX_MODBUS_TX_PIN
#define KX_MODBUS_TX_PIN     GPIO_NUM_4
#endif
#ifndef KX_MODBUS_RX_PIN
#define KX_MODBUS_RX_PIN     GPIO_NUM_36
#endif
#ifndef KX_MODBUS_RTS_PIN
#define KX_MODBUS_RTS_PIN    -1
#endif

#define MODBUS_RESPONSE_TIMEOUT_MS    100
#define MODBUS_INTER_FRAME_MS          20
#define MODBUS_INTER_PARAM_MS          10
#define MODBUS_RETRY_COUNT              2

#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

// =============================================================
// Pipeline: cola de resultados lectura → publicación
// =============================================================
#define PUB_QUEUE_SIZE   64

typedef enum {
    PUB_KIND_STATUS,
    PUB_KIND_REPORT,
    PUB_KIND_ERROR,
} kx_pub_kind_t;

typedef struct {
    kx_pub_kind_t kind;
    int           control_id;
    int           param_id;
    uint16_t      reg;
    float         value;
    char          error_msg[32];
} kx_pub_result_t;

static QueueHandle_t s_pub_queue = NULL;

// =============================================================
// Cola de demandas de poll
// =============================================================
#ifndef KX_DEMAND_REPEAT_MS
#define KX_DEMAND_REPEAT_MS  5000
#endif

#define DEMAND_QUEUE_SIZE   300
#define BATCH_THRESHOLD      15

// Tiempo máximo esperando que la ráfaga de gets termine de llegar
// antes de empezar a procesar (en ticks de 100 ms).
#define BURST_WAIT_MAX_TICKS  20   // 2 s máximo
#define BURST_STABLE_TICKS     2   // 2 ticks sin cambio → estable

typedef struct {
    int     param_id;
    int64_t enqueued_ms;
} kx_poll_demand_t;

static QueueHandle_t s_demand_queue = NULL;

// =============================================================
// Umbral de cambio para pub_report
// =============================================================
#define KX_STATUS_DELTA_ABS   0.5f
#define KX_STATUS_DELTA_REL   0.01f

// =============================================================
// Sincronización Modbus
// =============================================================
#define POLL_ALLOWED_BIT   BIT0
#define DEMAND_BIT         BIT1

static EventGroupHandle_t  s_poll_eg       = NULL;
static SemaphoreHandle_t   s_foreach_mutex = NULL;
static volatile bool       s_running       = false;
static TaskHandle_t        s_task          = NULL;

static volatile int64_t s_last_demand_ms = 0;

// =============================================================
// API pública — demanda de polling
// =============================================================
void kx_modbus_request_poll(int param_id)
{
    if (!s_demand_queue) return;

    int64_t now_ms   = (int64_t)(esp_timer_get_time() / 1000ULL);
    s_last_demand_ms = now_ms;

    // Deduplicar: mismo param_id en el frente de la cola < 5 s → descartar
    kx_poll_demand_t peek;
    if (xQueuePeek(s_demand_queue, &peek, 0) == pdTRUE) {
        if (peek.param_id == param_id &&
            (now_ms - peek.enqueued_ms) < (int64_t)KX_DEMAND_REPEAT_MS) {
            ESP_LOGD(TAG,
                "demand dedup param_id=%d (age=%" PRId64 "ms < %dms)",
                param_id, now_ms - peek.enqueued_ms, KX_DEMAND_REPEAT_MS);
            return;
        }
    }

    kx_poll_demand_t d = { .param_id = param_id, .enqueued_ms = now_ms };

    if (xQueueSend(s_demand_queue, &d, 0) != pdTRUE) {
        ESP_LOGW(TAG, "demand_queue full, dropping param_id=%d", param_id);
        return;
    }

    if (s_poll_eg) xEventGroupSetBits(s_poll_eg, DEMAND_BIT);

    ESP_LOGI(TAG, "poll demand enqueued param_id=%d ts=%" PRId64 "ms",
             param_id, now_ms);
}

// =============================================================
// CRC16 Modbus
// =============================================================
static uint16_t _crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

// =============================================================
// Init UART
// =============================================================
static esp_err_t _uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = KX_MODBUS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_param_config(KX_MODBUS_UART_NUM, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(KX_MODBUS_UART_NUM,
                       KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN,
                       KX_MODBUS_RTS_PIN, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(KX_MODBUS_UART_NUM, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) return err;

    if (KX_MODBUS_RTS_PIN != UART_PIN_NO_CHANGE) {
        err = uart_set_mode(KX_MODBUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "RS485 half-duplex mode failed: %s", esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "UART%d ready: baud=%d TX=%d RX=%d RTS=%d",
             KX_MODBUS_UART_NUM, KX_MODBUS_BAUD,
             KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN, KX_MODBUS_RTS_PIN);
    return ESP_OK;
}

// =============================================================
// Transacción Modbus
// =============================================================
static int _modbus_transaction(const uint8_t *frame, size_t frame_len,
                               uint8_t *resp, size_t resp_max)
{
    uint8_t tx[frame_len + 2];
    memcpy(tx, frame, frame_len);
    uint16_t crc = _crc16(frame, frame_len);
    tx[frame_len]     = (uint8_t)(crc & 0xFF);
    tx[frame_len + 1] = (uint8_t)(crc >> 8);

    uart_flush_input(KX_MODBUS_UART_NUM);
    uart_write_bytes(KX_MODBUS_UART_NUM, (const char *)tx, frame_len + 2);

    int rx_len = uart_read_bytes(KX_MODBUS_UART_NUM, resp, resp_max,
                                  pdMS_TO_TICKS(MODBUS_RESPONSE_TIMEOUT_MS));
    if (rx_len <= 0) return -1;
    if (rx_len < 4)  return -1;

    uint16_t rx_crc   = ((uint16_t)resp[rx_len - 1] << 8) | resp[rx_len - 2];
    uint16_t calc_crc = _crc16(resp, rx_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC error: got %04x expected %04x", rx_crc, calc_crc);
        return -1;
    }

    if (resp[1] & 0x80) {
        uint8_t exc = (rx_len > 2) ? resp[2] : 0;
        ESP_LOGW(TAG, "Modbus exception: fc=0x%02x exc=0x%02x", resp[1], exc);
        return -1;
    }

    return rx_len;
}

// =============================================================
// Leer un registro
// =============================================================
static float _read_register(uint8_t slave_addr, uint16_t reg_addr,
                             uint8_t fc, const kx_param_t *param)
{
    uint8_t frame[6] = {
        slave_addr,
        fc,
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        0x00,
        0x01,
    };

    uint8_t resp[16];
    int rx = -1;

    for (int attempt = 0; attempt < MODBUS_RETRY_COUNT && rx < 0; attempt++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }

    if (rx < 0) return -FLT_MAX;
    if (rx < 4 || resp[2] == 0) return -FLT_MAX;

    uint16_t raw;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE) {
        raw = resp[3] & 0x01;
    } else {
        if (rx < 5 || resp[2] < 2) return -FLT_MAX;
        raw = ((uint16_t)resp[3] << 8) | resp[4];
    }

    float value = (float)(int16_t)raw;
    if (param->offset != 0.0f && param->offset != 1.0f) value *= param->offset;
    value += param->addition;
    if (value < param->minvalue) value = param->minvalue;
    if (value > param->maxvalue) value = param->maxvalue;

    return value;
}

// =============================================================
// Helpers de decisión de publicación
// =============================================================
static inline bool _has_changed(float new_val, float last_val)
{
    if (last_val == FLT_MAX) return true;
    float delta = fabsf(new_val - last_val);
    if (delta > KX_STATUS_DELTA_ABS) return true;
    if (last_val != 0.0f && (delta / fabsf(last_val)) > KX_STATUS_DELTA_REL) return true;
    return false;
}

// =============================================================
// Contexto de iteración de poll
// =============================================================
typedef struct {
    int     total;
    int     done;
    int     ok;
    int     errors;
    int     skipped;
    int     unchanged;
    int64_t cycle_s;
    bool    demand_active;
} _poll_ctx_t;

typedef struct { int count; } _count_ctx_t;

typedef struct {
    int target_param_id;
    int found_ctrl_id;
} _find_ctrl_ctx_t;

// =============================================================
// Barra de progreso ASCII — ciclo completo
// =============================================================
#define POLL_BAR_WIDTH 30

static void _print_progress(int control_id, int done, int total)
{
    if (total <= 0) return;
    int pct  = (done * 100) / total;
    int fill = (done * POLL_BAR_WIDTH) / total;
    char bar[POLL_BAR_WIDTH + 1];
    for (int i = 0; i < POLL_BAR_WIDTH; i++) bar[i] = (i < fill) ? '#' : '-';
    bar[POLL_BAR_WIDTH] = '\0';
    printf("[poll] ctrl=%d [%s] %3d%% (%d/%d params)\r",
           control_id, bar, pct, done, total);
    fflush(stdout);
    if (pct == 25 || pct == 50 || pct == 75 || pct == 100) {
        printf("\n");
        fflush(stdout);
    }
}

// =============================================================
// Encolar resultado para el publisher
// =============================================================
static void _enqueue(kx_pub_kind_t kind, int ctrl_id, int param_id,
                     float value, uint16_t reg, const char *errmsg)
{
    kx_pub_result_t r = {
        .kind       = kind,
        .control_id = ctrl_id,
        .param_id   = param_id,
        .reg        = reg,
        .value      = value,
    };
    if (errmsg) snprintf(r.error_msg, sizeof(r.error_msg), "%s", errmsg);

    if (xQueueSend(s_pub_queue, &r, 0) != pdTRUE)
        ESP_LOGW(TAG, "pub_queue full, dropping param_id=%d", param_id);
}

// =============================================================
// Callback de iteración — poll completo
// =============================================================
static void _poll_param(int control_id, const kx_param_t *param, void *user_data)
{
    _poll_ctx_t *ctx = (_poll_ctx_t *)user_data;

    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return;

    uint8_t fc_read = (uint8_t)param->function_read;
    bool is_read_fc = (fc_read == MB_FC_READ_COILS        ||
                       fc_read == MB_FC_READ_DISCRETE      ||
                       fc_read == MB_FC_READ_HOLDING_REGS  ||
                       fc_read == MB_FC_READ_INPUT_REGS);
    if (!is_read_fc) return;

    int64_t now_ms      = (int64_t)(esp_timer_get_time() / 1000ULL);
    int     sampling_ms = (param->sampling > 0 ? param->sampling : 60) * 1000;

    bool report_due = (param->sampling > 0) &&
                      (ctx->cycle_s % (int64_t)param->sampling == 0);

    bool read_due = ctx->demand_active ||
                    (param->ts_last_read == 0) ||
                    ((now_ms - param->ts_last_read) >= (int64_t)sampling_ms);

    if (!read_due && !report_due) {
        ctx->skipped++;
        ESP_LOGD(TAG,
            "skip param_id=%d | cycle_s=%" PRId64 "s | sampling=%ds"
            " | elapsed=%" PRId64 "ms",
            param->param_id, ctx->cycle_s, param->sampling,
            now_ms - param->ts_last_read);
        return;
    }

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);

    ctx->done++;
    _print_progress(control_id, ctx->done, ctx->total);

    if (value == -FLT_MAX) {
        _enqueue(PUB_KIND_ERROR, control_id, param->param_id,
                 0.0f, (uint16_t)param->reg, "modbus_timeout");
        ctx->errors++;
        vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
        return;
    }

    ctx->ok++;

    if (ctx->demand_active) {
        _enqueue(PUB_KIND_STATUS, control_id, param->param_id, value, 0, NULL);
    } else {
        ctx->unchanged++;
    }

    if (report_due && _has_changed(value, param->last_published_value)) {
        _enqueue(PUB_KIND_REPORT, control_id, param->param_id, value, 0, NULL);
        ESP_LOGI(TAG,
            "REPORT param_id=%d | cycle_s=%" PRId64 "s | sampling=%ds",
            param->param_id, ctx->cycle_s, param->sampling);
    }

    kx_param_t *mp = kx_param_store_get_param_mutable(control_id, param->param_id);
    if (mp) {
        mp->ts_last_read         = now_ms;
        mp->last_published_value = value;
    }

    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
}

// Callback auxiliar: contar parámetros legibles
static void _count_readable(int control_id, const kx_param_t *param, void *user_data)
{
    _count_ctx_t *c = (_count_ctx_t *)user_data;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE ||
        fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS) c->count++;
}

// Callback auxiliar: encontrar el control_id dueño de un param_id
static void _find_ctrl_cb(int ctrl_id, const kx_param_t *param, void *user_data)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)user_data;
    if (ctx->found_ctrl_id < 0 && param->param_id == ctx->target_param_id)
        ctx->found_ctrl_id = ctrl_id;
}

// =============================================================
// Poll de un único param_id (modo single)
// =============================================================
static void _poll_single_param(int param_id)
{
    _find_ctrl_ctx_t fctx = { .target_param_id = param_id, .found_ctrl_id = -1 };
    kx_param_store_foreach(_find_ctrl_cb, &fctx);

    if (fctx.found_ctrl_id < 0) {
        ESP_LOGW(TAG, "poll_single: param_id=%d not found in any control", param_id);
        return;
    }

    const kx_param_t *param = kx_param_store_get_param(fctx.found_ctrl_id, param_id);
    if (!param) return;

    const kx_control_params_t *ctrl = kx_param_store_get(fctx.found_ctrl_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "poll_single: ctrl=%d has no slave_addr", fctx.found_ctrl_id);
        return;
    }

    uint8_t fc_read = (uint8_t)param->function_read;
    bool is_read_fc = (fc_read == MB_FC_READ_COILS        ||
                       fc_read == MB_FC_READ_DISCRETE      ||
                       fc_read == MB_FC_READ_HOLDING_REGS  ||
                       fc_read == MB_FC_READ_INPUT_REGS);

    if (!is_read_fc || param->view == 0) {
        ESP_LOGD(TAG, "poll_single: param_id=%d not readable/visible", param_id);
        return;
    }

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);

    if (value == -FLT_MAX) {
        _enqueue(PUB_KIND_ERROR, fctx.found_ctrl_id, param->param_id,
                 0.0f, (uint16_t)param->reg, "modbus_timeout");
        ESP_LOGW(TAG, "poll_single: timeout param_id=%d reg=0x%04x",
                 param_id, param->reg);
        return;
    }

    _enqueue(PUB_KIND_STATUS, fctx.found_ctrl_id, param->param_id, value, 0, NULL);

    kx_param_t *mp = kx_param_store_get_param_mutable(fctx.found_ctrl_id, param_id);
    if (mp) {
        mp->ts_last_read         = (int64_t)(esp_timer_get_time() / 1000ULL);
        mp->last_published_value = value;
    }

    ESP_LOGD(TAG, "poll_single: param_id=%d ctrl=%d value=%.3f",
             param_id, fctx.found_ctrl_id, value);
}

// =============================================================
// Tarea publisher
// =============================================================
static void _publisher_task(void *arg)
{
    kx_pub_result_t r;
    ESP_LOGI(TAG, "publisher task started (queue_size=%d)", PUB_QUEUE_SIZE);

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
// Tarea principal Modbus
// =============================================================
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "task started — waiting for entities...");

    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "entities ready (%d controls) — waiting for poll demand",
             kx_param_store_count());

    int64_t cycle_s = 5;

    while (s_running) {

        // ── 1. Esperar señal de demanda ───────────────────────
        xEventGroupWaitBits(s_poll_eg,
                            DEMAND_BIT,
                            pdFALSE,        // NO auto-clear
                            pdTRUE,
                            portMAX_DELAY);

        // ── 1b. Esperar a que la ráfaga termine de llegar ─────
        // El frontend manda todos los gets en ~1-2 s.
        // Esperamos hasta que el tamaño de la cola se estabilice
        // (BURST_STABLE_TICKS ticks consecutivos sin cambio)
        // o hasta BURST_WAIT_MAX_TICKS × 100 ms como máximo.
        {
            int prev  = -1;
            int stab  = 0;
            for (int i = 0; i < BURST_WAIT_MAX_TICKS; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
                int cur = (int)uxQueueMessagesWaiting(s_demand_queue);
                if (cur == prev) {
                    if (++stab >= BURST_STABLE_TICKS) break;
                } else {
                    stab = 0;
                }
                prev = cur;
            }
            ESP_LOGD(TAG, "burst wait done: queue=%d",
                     (int)uxQueueMessagesWaiting(s_demand_queue));
        }

        // ── 2. Drenar cola ────────────────────────────────────
        {
            int queued      = (int)uxQueueMessagesWaiting(s_demand_queue);
            bool batch_mode = (queued >= BATCH_THRESHOLD);

            int batch_total   = queued;
            int batch_done    = 0;
            int batch_ok      = 0;
            int batch_timeout = 0;
            int batch_skip    = 0;

            if (batch_mode) {
                printf("\n[modbus] batch poll iniciado: %d params\n", batch_total);
                fflush(stdout);
            }

            kx_poll_demand_t demand;

            while (xQueueReceive(s_demand_queue, &demand, 0) == pdTRUE) {

                // Descartar demandas caducadas
                int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
                int64_t age_ms = now_ms - demand.enqueued_ms;
                if (age_ms > (int64_t)(KX_DEMAND_TIMEOUT_S * 1000)) {
                    ESP_LOGD(TAG, "demand expired param_id=%d age=%" PRId64 "ms — skip",
                             demand.param_id, age_ms);
                    batch_skip++;
                    continue;
                }

                // Prerequisitos
                if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                    continue;
                }

                // Respetar pause/resume
                xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                                    pdFALSE, pdTRUE, portMAX_DELAY);

                xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);

                if (!s_running || !kx_param_store_is_ready()) {
                    xSemaphoreGive(s_foreach_mutex);
                    continue;
                }

                // ── Ejecutar el poll correspondiente ─────────
                if (demand.param_id > 0) {

                    int pub_before = (int)uxQueueMessagesWaiting(s_pub_queue);
                    _poll_single_param(demand.param_id);
                    int pub_after  = (int)uxQueueMessagesWaiting(s_pub_queue);

                    batch_done++;
                    if (pub_after > pub_before) batch_ok++;
                    else                        batch_timeout++;

                    if (batch_mode) {
                        // Recalcular total por si llegaron más demandas
                        int current_total = batch_done
                                          + (int)uxQueueMessagesWaiting(s_demand_queue);
                        int pct  = (batch_done * 100) / current_total;
                        int fill = (batch_done * POLL_BAR_WIDTH) / current_total;
                        char bar[POLL_BAR_WIDTH + 1];
                        for (int i = 0; i < POLL_BAR_WIDTH; i++)
                            bar[i] = (i < fill) ? '#' : '-';
                        bar[POLL_BAR_WIDTH] = '\0';
                        printf("\r[batch] [%s] %3d%% (%d/%d) ok=%d err=%d  ",
                               bar, pct, batch_done, current_total,
                               batch_ok, batch_timeout);
                        fflush(stdout);
                    }

                } else {
                    // Ciclo completo
                    _count_ctx_t cc = { .count = 0 };
                    kx_param_store_foreach(_count_readable, &cc);

                    _poll_ctx_t ctx = {
                        .total         = cc.count,
                        .done          = 0,
                        .ok            = 0,
                        .errors        = 0,
                        .skipped       = 0,
                        .unchanged     = 0,
                        .cycle_s       = cycle_s,
                        .demand_active = true,
                    };

                    KX_LOG_CYCLE_START(TAG, cycle_s, kx_param_store_count(), ctx.total);
                    kx_param_store_foreach(_poll_param, &ctx);
                    KX_LOG_CYCLE_END(TAG, ctx.ok, ctx.errors, ctx.skipped, ctx.unchanged);

                    batch_ok      += ctx.ok;
                    batch_timeout += ctx.errors;
                    batch_done    += ctx.ok + ctx.errors;

                    cycle_s = (cycle_s + KX_TELEMETRY_INTERVAL_S) % 60;
                }

                xSemaphoreGive(s_foreach_mutex);

                // Delay entre demandas para no saturar el bus RS-485
                if (uxQueueMessagesWaiting(s_demand_queue) > 0) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            // ── Resumen batch ─────────────────────────────────
            if (batch_mode) {
                printf("\n");
                printf("┌──────────────────────────────────────┐\n");
                printf("│        BATCH POLL  RESUMEN           │\n");
                printf("├──────────────────────────────────────┤\n");
                printf("│  Solicitados : %-4d                  │\n", batch_total);
                printf("│  Procesados  : %-4d                  │\n", batch_done);
                printf("│  OK (leídos) : %-4d                  │\n", batch_ok);
                printf("│  Timeout     : %-4d                  │\n", batch_timeout);
                printf("│  Expirados   : %-4d                  │\n", batch_skip);
                printf("│  Heap libre  : %-8lu bytes       │\n",
                       (unsigned long)kx_system_heap_free());
                printf("└──────────────────────────────────────┘\n");
                fflush(stdout);
            }
        }

        // ── 3. Cola vacía: apagar DEMAND_BIT ─────────────────
        xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
    }

    uart_driver_delete(KX_MODBUS_UART_NUM);
    ESP_LOGI(TAG, "task stopped");
    vTaskDelete(NULL);
}

// =============================================================
// pause / resume
// =============================================================
void kx_modbus_pause(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;

    ESP_LOGI(TAG, "pausing Modbus polling...");
    xEventGroupClearBits(s_poll_eg, POLL_ALLOWED_BIT);

    if (xSemaphoreTake(s_foreach_mutex, pdMS_TO_TICKS(60000)) != pdTRUE) {
        ESP_LOGE(TAG, "pause: timeout waiting for foreach — memory risk!");
    } else {
        ESP_LOGI(TAG, "Modbus polling paused (foreach complete)");
    }
}

void kx_modbus_resume(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xSemaphoreGive(s_foreach_mutex);
    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    ESP_LOGI(TAG, "Modbus polling resumed");
}

// =============================================================
// start / stop
// =============================================================
esp_err_t kx_modbus_master_start(void)
{
    if (s_running) { ESP_LOGW(TAG, "already running"); return ESP_OK; }

    s_pub_queue = xQueueCreate(PUB_QUEUE_SIZE, sizeof(kx_pub_result_t));
    if (!s_pub_queue) {
        ESP_LOGE(TAG, "failed to create pub_queue");
        return ESP_FAIL;
    }

    s_demand_queue = xQueueCreate(DEMAND_QUEUE_SIZE, sizeof(kx_poll_demand_t));
    if (!s_demand_queue) {
        ESP_LOGE(TAG, "failed to create demand_queue");
        return ESP_FAIL;
    }

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) { ESP_LOGE(TAG, "failed to create EventGroup"); return ESP_FAIL; }

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) { ESP_LOGE(TAG, "failed to create mutex"); return ESP_FAIL; }

    // POLL_ALLOWED_BIT siempre activo al arranque
    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);

    esp_err_t err = _uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
        return err;
    }

    BaseType_t ret = xTaskCreate(_publisher_task, "kx_publisher", 4096, NULL,
                                  KX_TASK_PRIO_TELEMETRY - 1, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create publisher task");
        return ESP_FAIL;
    }

    s_running = true;
    ret = xTaskCreate(_modbus_task, "kx_modbus", 8192, NULL,
                      KX_TASK_PRIO_TELEMETRY, &s_task);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    return ESP_OK;
}

void kx_modbus_master_stop(void) { s_running = false; }

bool kx_modbus_master_is_running(void) { return s_running; }

// =============================================================
// kx_modbus_read_one  (lectura puntual bajo demanda)
// =============================================================
esp_err_t kx_modbus_read_one(int control_id, int param_id)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) return ESP_ERR_NOT_FOUND;

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return ESP_ERR_INVALID_STATE;

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg,
                                  (uint8_t)param->function_read, param);
    if (value == -FLT_MAX) {
        kx_param_pub_error(control_id, param->param_id,
                           "modbus_timeout", (uint16_t)param->reg);
        return ESP_FAIL;
    }
    kx_param_pub_status(control_id, param->param_id, value);
    return ESP_OK;
}

// =============================================================
// kx_modbus_write_one
// =============================================================
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) {
        ESP_LOGW(TAG, "write_one: param no encontrado ctrl=%d param=%d",
                 control_id, param_id);
        return ESP_ERR_NOT_FOUND;
    }

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "write_one: slave_addr inválido ctrl=%d", control_id);
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL    &&
        fc_write != MB_FC_WRITE_SINGLE_REG     &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS) {
        ESP_LOGW(TAG, "write_one: FC de escritura no soportado fc=0x%02x param=%d",
                 fc_write, param_id);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw;

    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        float adjusted = value - param->addition;
        if (param->offset != 0.0f && param->offset != 1.0f) {
            raw = (int16_t)(adjusted / param->offset);
        } else {
            raw = (int16_t)adjusted;
        }
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }

    ESP_LOGI(TAG, "write_one: ctrl=%d param=%d reg=0x%04x fc=0x%02x "
             "slave=%d value=%.3f → raw=%d (0x%04X)",
             control_id, param_id, param->reg, fc_write,
             ctrl->slave_addr, value, (int)(uint16_t)raw, (uint16_t)raw);

    uint8_t frame[6] = {
        (uint8_t)ctrl->slave_addr,
        fc_write,
        (uint8_t)((uint16_t)param->reg >> 8),
        (uint8_t)((uint16_t)param->reg & 0xFF),
        (uint8_t)((uint16_t)raw >> 8),
        (uint8_t)((uint16_t)raw & 0xFF),
    };

    uint8_t resp[16];
    int rx = -1;

    for (int attempt = 0; attempt < MODBUS_RETRY_COUNT && rx < 0; attempt++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) {
            ESP_LOGD(TAG, "write_one: intento %d fallido, reintentando...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
    }

    if (rx < 0) {
        ESP_LOGW(TAG, "write_one: sin respuesta tras %d intentos ctrl=%d param=%d",
                 MODBUS_RETRY_COUNT, control_id, param_id);
        return ESP_FAIL;
    }

    if (rx < 6 ||
        resp[0] != frame[0] ||
        resp[1] != frame[1] ||
        resp[2] != frame[2] ||
        resp[3] != frame[3]) {
        ESP_LOGW(TAG, "write_one: respuesta inesperada (rx=%d) ctrl=%d param=%d",
                 rx, control_id, param_id);
        return ESP_FAIL;
    }

    uint16_t echo_val = ((uint16_t)resp[4] << 8) | resp[5];
    ESP_LOGI(TAG, "write_one: OK ctrl=%d param=%d raw_sent=%d raw_echo=%d",
             control_id, param_id, (int)(uint16_t)raw, (int)echo_val);

    return ESP_OK;
}