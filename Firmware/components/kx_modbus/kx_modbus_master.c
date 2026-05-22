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

#define MODBUS_RESPONSE_TIMEOUT_MS    50
#define MODBUS_INTER_FRAME_MS         20
#define MODBUS_INTER_PARAM_MS         10
#define MODBUS_RETRY_COUNT             2

#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

// =============================================================
// Pipeline: cola de resultados lectura → publicación
//
// El task de lectura Modbus escribe en s_pub_queue.
// El task publisher consume la cola y llama a kx_mqtt_publish.
// Así el bus RS-485 nunca espera al broker MQTT.
// =============================================================
#define PUB_QUEUE_SIZE   64    // slots: suficiente para un ciclo completo

typedef enum {
    PUB_KIND_STATUS,   // kx_param_pub_status  — solo si cambia
    PUB_KIND_REPORT,   // kx_param_pub_report  — por sampling
    PUB_KIND_ERROR,    // kx_param_pub_error
} kx_pub_kind_t;

typedef struct {
    kx_pub_kind_t kind;
    int           control_id;
    int           param_id;
    uint16_t      reg;         // solo para ERROR
    float         value;
    char          error_msg[32]; // solo para ERROR
} kx_pub_result_t;

static QueueHandle_t s_pub_queue = NULL;

// =============================================================
// Umbral de cambio para publicación en "status"
//
// Se considera "cambio" si:
//   |value - last_published_value| > KX_STATUS_DELTA_ABS
//   O bien  |delta / last_published_value| > KX_STATUS_DELTA_REL
//
// El umbral absoluto evita publicaciones por ruido en valores
// cercanos a cero donde el relativo explotaría.
// =============================================================
#define KX_STATUS_DELTA_ABS   0.5f   // ±0.5 unidades
#define KX_STATUS_DELTA_REL   0.01f  // 1% de cambio relativo

// =============================================================
// Sincronización Modbus
// =============================================================
#define POLL_ALLOWED_BIT   BIT0

static EventGroupHandle_t  s_poll_eg       = NULL;
static SemaphoreHandle_t   s_foreach_mutex = NULL;
static volatile bool       s_running       = false;
static TaskHandle_t        s_task          = NULL;

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

// Devuelve true si el valor ha cambiado suficiente para publicar
// en "status". Se usa doble umbral: absoluto y relativo.
static inline bool _has_changed(float new_val, float last_val)
{
    if (last_val == FLT_MAX) return true;   // primera lectura: siempre publicar
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
    int     skipped;    // parámetros saltados por sampling
    int     unchanged;  // leídos pero no publicados en status (sin cambio)
    int64_t cycle_s;    // segundo de ciclo para la lógica de sampling
} _poll_ctx_t;

typedef struct { int count; } _count_ctx_t;

// =============================================================
// Barra de progreso ASCII
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
    printf("[poll] ctrl=%d [%s] %3d%% (%d/%d params)\r", control_id, bar, pct, done, total);
    fflush(stdout);
    if (pct == 25 || pct == 50 || pct == 75 || pct == 100) { printf("\n"); fflush(stdout); }
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
    if (errmsg) {
        snprintf(r.error_msg, sizeof(r.error_msg), "%s", errmsg);
    }

    if (xQueueSend(s_pub_queue, &r, 0) != pdTRUE) {
        // Cola llena: el publisher va retrasado. No bloqueamos el bus.
        ESP_LOGW(TAG, "pub_queue full, dropping param_id=%d", param_id);
    }
}

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

    // ── 1. Decidir antes del skip si toca report ──────────────
    // report_due se evalúa SIEMPRE, independientemente de ts_last_read.
    // Si el ciclo actual es múltiplo del sampling, forzamos lectura
    // aunque ts_last_read diga que aún no toca.
    bool report_due = (param->sampling > 0) &&
                      (ctx->cycle_s % (int64_t)param->sampling == 0);

    // ── 2. ¿Toca leer por ts? ─────────────────────────────────
    bool read_due = (param->ts_last_read == 0) ||
                    ((now_ms - param->ts_last_read) >= (int64_t)sampling_ms);

    // Si no toca ni leer ni reportar → skip
    if (!read_due && !report_due) {
        ctx->skipped++;
        ESP_LOGD(TAG,
            "skip param_id=%d | cycle_s=%" PRId64 "s | sampling=%ds"
            " | elapsed=%" PRId64 "ms | rem=%" PRId64,
            param->param_id, ctx->cycle_s, param->sampling,
            now_ms - param->ts_last_read,
            (param->sampling > 0
                ? ctx->cycle_s % (int64_t)param->sampling
                : (int64_t)-1));
        return;
    }

    // ── 3. Leer por Modbus ────────────────────────────────────
    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);

    ctx->done++;
    _print_progress(control_id, ctx->done, ctx->total);

    // ── 4. Error de lectura ───────────────────────────────────
    if (value == -FLT_MAX) {
        _enqueue(PUB_KIND_ERROR, control_id, param->param_id,
                 0.0f, (uint16_t)param->reg, "modbus_timeout");
        ctx->errors++;
        vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
        return;
    }

    ctx->ok++;

    // ── 5a. STATUS — solo si el valor ha cambiado ─────────────
    if (_has_changed(value, param->last_published_value)) {
        _enqueue(PUB_KIND_STATUS, control_id, param->param_id,
                 value, 0, NULL);
    } else {
        ctx->unchanged++;
    }

    // ── 5b. REPORT — con log de diagnóstico ──────────────────
    if (report_due) {
        _enqueue(PUB_KIND_REPORT, control_id, param->param_id, value, 0, NULL);
        ESP_LOGI(TAG,
            "REPORT param_id=%d | cycle_s=%" PRId64 "s | sampling=%ds"
            " | forced_read=%s",
            param->param_id, ctx->cycle_s, param->sampling,
            read_due ? "no" : "YES");
    }

    // ── 5c. Actualizar campos de runtime (mutable) ────────────
    kx_param_t *mp = kx_param_store_get_param_mutable(control_id, param->param_id);
    if (mp) {
        mp->ts_last_read         = now_ms;
        mp->last_published_value = value;
    }

    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
}

// Contador de parámetros legibles (para inicializar ctx->total)
static void _count_readable(int control_id, const kx_param_t *param, void *user_data)
{
    _count_ctx_t *c = (_count_ctx_t *)user_data;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE ||
        fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS) c->count++;
}

// =============================================================
// Tarea publisher
//
// Consume s_pub_queue y llama a las funciones de telemetría.
// Separada del bus Modbus para que un broker lento no ralentice
// las lecturas RS-485.
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

    ESP_LOGI(TAG, "entities ready (%d controls) — starting Modbus polling",
             kx_param_store_count());
    vTaskDelay(pdMS_TO_TICKS(4000));

    int64_t cycle_s = 5;

    while (s_running) {

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        if (!kx_mqtt_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        if (!kx_param_store_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);

        if (!s_running || !kx_param_store_is_ready()) {
            xSemaphoreGive(s_foreach_mutex);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        _count_ctx_t cc = { .count = 0 };
        kx_param_store_foreach(_count_readable, &cc);

        _poll_ctx_t ctx = {
            .total     = cc.count,
            .done      = 0,
            .ok        = 0,
            .errors    = 0,
            .skipped   = 0,
            .unchanged = 0,
            .cycle_s   = cycle_s,
        };

        KX_LOG_CYCLE_START(TAG, cycle_s, kx_param_store_count(), ctx.total);

        kx_param_store_foreach(_poll_param, &ctx);

        xSemaphoreGive(s_foreach_mutex);

        KX_LOG_CYCLE_END(TAG, ctx.ok, ctx.errors, ctx.skipped, ctx.unchanged);

        // Avanzar el contador de segundos de ciclo
        cycle_s = (cycle_s + KX_TELEMETRY_INTERVAL_S) % 60;

        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
    }

    uart_driver_delete(KX_MODBUS_UART_NUM);
    ESP_LOGI(TAG, "task stopped");
    vTaskDelete(NULL);
}

// =============================================================
// pause / resume  (sin cambios respecto al original)
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

    // Cola pipeline lectura → publicación
    s_pub_queue = xQueueCreate(PUB_QUEUE_SIZE, sizeof(kx_pub_result_t));
    if (!s_pub_queue) {
        ESP_LOGE(TAG, "failed to create pub_queue");
        return ESP_FAIL;
    }

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) { ESP_LOGE(TAG, "failed to create EventGroup"); return ESP_FAIL; }

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) { ESP_LOGE(TAG, "failed to create mutex"); return ESP_FAIL; }

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);

    esp_err_t err = _uart_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err)); return err; }

    // Tarea publisher (prioridad baja: cede ante lectura Modbus)
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
// Publica directamente (no usa pipeline) para respuesta síncrona.
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
        kx_param_pub_error(control_id, param->param_id, "modbus_timeout", (uint16_t)param->reg);
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
        resp[3] != frame[3])
    {
        ESP_LOGW(TAG, "write_one: respuesta inesperada (rx=%d) ctrl=%d param=%d",
                 rx, control_id, param_id);
        return ESP_FAIL;
    }

    uint16_t echo_val = ((uint16_t)resp[4] << 8) | resp[5];
    ESP_LOGI(TAG, "write_one: OK ctrl=%d param=%d raw_sent=%d raw_echo=%d",
             control_id, param_id, (int)(uint16_t)raw, (int)echo_val);

    return ESP_OK;
}