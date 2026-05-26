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
// Cola de publicación: resultados Modbus → MQTT publisher
//
// Backpressure:
//   Si la cola supera PUB_QUEUE_BACKPRESSURE_HWM entradas
//   (señal de que el publisher MQTT va lento), el hilo Modbus
//   espera PUB_BACKPRESSURE_WAIT_MS antes de encolar el
//   siguiente resultado, evitando drops silenciosos.
//   Si tras PUB_BACKPRESSURE_TIMEOUT_MS la cola sigue llena,
//   se descarta el resultado y se registra en el resumen.
// =============================================================
#define PUB_QUEUE_SIZE               500
#define PUB_QUEUE_BACKPRESSURE_HWM   350   // 70 % de 500
#define PUB_BACKPRESSURE_WAIT_MS      20
#define PUB_BACKPRESSURE_TIMEOUT_MS 2000

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
// Cola de demandas de poll (demand_queue)
//
// Gestión de la cola:
//   - DEMAND_QUEUE_SIZE slots (capacidad máxima física).
//   - Durante la fase de recopilación se monitoriza el nivel:
//     si supera DEMAND_WARN_HWM se emite un warning.
//   - Al construir el snapshot se deduplican param_ids:
//     si el mismo param_id aparece varias veces en la cola
//     (p.ej. el frontend manda doble-click), solo se procesa
//     una vez.
//   - Demandas con edad > KX_DEMAND_TIMEOUT_S se descartan
//     antes de procesar (ya caducaron).
//   - Si el snapshot falla por OOM, la cola se vacía
//     completamente para no bloquear el sistema.
// =============================================================
#ifndef KX_DEMAND_REPEAT_MS
#define KX_DEMAND_REPEAT_MS  5000
#endif

#define DEMAND_QUEUE_SIZE    1500
#define DEMAND_WARN_HWM      700   // warning si la cola supera esto

// Fase de recopilación de ráfaga
#define BURST_COLLECT_MAX_MS  3000  // timeout máximo de espera
#define BURST_STABLE_MS        300  // ms sin cambio → ráfaga terminada
#define BURST_POLL_MS           50  // resolución del bucle

// Mínimo de demandas para activar el modo batch con barra de progreso
#define BATCH_THRESHOLD          5

// Máximo de param_ids mostrados en el resumen de errores
#define BATCH_REPORT_MAX_ERRORS  200

typedef struct {
    int     param_id;
    int64_t enqueued_ms;
} kx_poll_demand_t;

static QueueHandle_t s_demand_queue = NULL;

#define PENDING_SET_SIZE  1024   

static volatile uint8_t s_pending_bits[PENDING_SET_SIZE / 8] = {0};

static inline void _pending_set(int param_id) {
    uint32_t idx = ((uint32_t)param_id) & (PENDING_SET_SIZE - 1);
    s_pending_bits[idx / 8] |= (1u << (idx % 8));
}

static inline void _pending_clear(int param_id) {
    uint32_t idx = ((uint32_t)param_id) & (PENDING_SET_SIZE - 1);
    s_pending_bits[idx / 8] &= ~(1u << (idx % 8));
}

static inline bool _pending_test(int param_id) {
    uint32_t idx = ((uint32_t)param_id) & (PENDING_SET_SIZE - 1);
    return (s_pending_bits[idx / 8] >> (idx % 8)) & 1u;
}
// =============================================================
// Umbral de cambio para pub_report
// =============================================================
#define KX_STATUS_DELTA_ABS   0.5f
#define KX_STATUS_DELTA_REL   0.01f

// =============================================================
// Barra de progreso ASCII (sobreescribe la misma línea)
// =============================================================
#define POLL_BAR_WIDTH 40

static void _print_batch_progress(int done, int total, int ok, int errors)
{
    if (total <= 0) return;
    int pct  = (done * 100) / total;
    int fill = (done * POLL_BAR_WIDTH) / total;
    char bar[POLL_BAR_WIDTH + 1];
    for (int i = 0; i < POLL_BAR_WIDTH; i++) bar[i] = (i < fill) ? '#' : '-';
    bar[POLL_BAR_WIDTH] = '\0';
    printf("\r  [%s] %3d%% (%d/%d)  ok=%-4d err=%-4d  ",
           bar, pct, done, total, ok, errors);
    fflush(stdout);
}

// =============================================================
// Sincronización Modbus
// =============================================================
#define POLL_ALLOWED_BIT   BIT0
#define DEMAND_BIT         BIT1

static EventGroupHandle_t  s_poll_eg       = NULL;
static SemaphoreHandle_t   s_foreach_mutex = NULL;
static volatile bool       s_running       = false;
static TaskHandle_t        s_task          = NULL;

// =============================================================
// _enqueue — encola resultado con backpressure activo
//
// Si s_pub_queue supera PUB_QUEUE_BACKPRESSURE_HWM, espera en
// pasos de PUB_BACKPRESSURE_WAIT_MS hasta PUB_BACKPRESSURE_TIMEOUT_MS.
// Si tras el timeout la cola sigue llena, devuelve false (drop).
// =============================================================
static bool _enqueue(kx_pub_kind_t kind, int ctrl_id, int param_id,
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

    // Intento rápido sin espera
    if (xQueueSend(s_pub_queue, &r, 0) == pdTRUE) return true;

    // Cola por encima del HWM: aplicar backpressure
    int waited = 0;
    int hwm    = (int)uxQueueMessagesWaiting(s_pub_queue);
    ESP_LOGW(TAG, "pub_queue backpressure: %d/%d — waiting for publisher...",
             hwm, PUB_QUEUE_SIZE);

    while (waited < PUB_BACKPRESSURE_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(PUB_BACKPRESSURE_WAIT_MS));
        waited += PUB_BACKPRESSURE_WAIT_MS;
        if (xQueueSend(s_pub_queue, &r, 0) == pdTRUE) {
            ESP_LOGI(TAG, "pub_queue backpressure released after %dms", waited);
            return true;
        }
    }

    // Drop definitivo
    ESP_LOGE(TAG,
        "pub_queue DROP param_id=%d after %dms backpressure (queue=%d/%d)",
        param_id, waited,
        (int)uxQueueMessagesWaiting(s_pub_queue), PUB_QUEUE_SIZE);
    return false;
}

// =============================================================
// API pública — demanda de polling
// =============================================================
void kx_modbus_request_poll(int param_id)
{
    if (!s_demand_queue) return;

    // Rechazo O(1) si ya está pendiente en la cola
    if (param_id != 0 && _pending_test(param_id)) {
        ESP_LOGD(TAG, "demand dedup (bitmap) param_id=%d", param_id);
        return;
    }

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    int used = (int)uxQueueMessagesWaiting(s_demand_queue);
    if (used >= DEMAND_WARN_HWM) {
        ESP_LOGW(TAG, "demand_queue near full: %d/%d", used, DEMAND_QUEUE_SIZE);
    }

    kx_poll_demand_t d = { .param_id = param_id, .enqueued_ms = now_ms };
    if (xQueueSend(s_demand_queue, &d, 0) != pdTRUE) {
        ESP_LOGE(TAG, "demand_queue FULL (%d slots), dropping param_id=%d",
                 DEMAND_QUEUE_SIZE, param_id);
        return;
    }

    if (param_id != 0) _pending_set(param_id);   // marcar pendiente TRAS encolar
    if (s_poll_eg) xEventGroupSetBits(s_poll_eg, DEMAND_BIT);
    ESP_LOGD(TAG, "demand enqueued param_id=%d queue=%d", param_id, used + 1);
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
    err = uart_param_config(KX_MODBUS_UART_NUM, &cfg);   if (err != ESP_OK) return err;
    err = uart_set_pin(KX_MODBUS_UART_NUM,
                       KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN,
                       KX_MODBUS_RTS_PIN, UART_PIN_NO_CHANGE);  if (err != ESP_OK) return err;
    err = uart_driver_install(KX_MODBUS_UART_NUM, 256, 256, 0, NULL, 0); if (err != ESP_OK) return err;

    if (KX_MODBUS_RTS_PIN != UART_PIN_NO_CHANGE) {
        err = uart_set_mode(KX_MODBUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
        if (err != ESP_OK) ESP_LOGW(TAG, "RS485 mode failed: %s", esp_err_to_name(err));
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
    if (rx_len <= 0 || rx_len < 4) return -1;

    uint16_t rx_crc   = ((uint16_t)resp[rx_len - 1] << 8) | resp[rx_len - 2];
    uint16_t calc_crc = _crc16(resp, rx_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC error: got %04x expected %04x", rx_crc, calc_crc);
        return -1;
    }
    if (resp[1] & 0x80) {
        uint8_t exc = (rx_len > 2) ? resp[2] : 0;
        ESP_LOGW(TAG, "Modbus exception fc=0x%02x exc=0x%02x", resp[1], exc);
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
        slave_addr, fc,
        (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF),
        0x00, 0x01,
    };
    uint8_t resp[16];
    int rx = -1;
    for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }
    if (rx < 0 || rx < 4 || resp[2] == 0) return -FLT_MAX;

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
// Helpers
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
// Tipos de contexto para kx_param_store_foreach
// =============================================================
typedef struct {
    int     total, done, ok, errors, skipped, unchanged;
    int64_t cycle_s;
    bool    demand_active;
} _poll_ctx_t;

typedef struct { int count; } _count_ctx_t;

typedef struct { int target_param_id; int found_ctrl_id; } _find_ctrl_ctx_t;

// Contar parámetros legibles
static void _count_readable(int control_id, const kx_param_t *param, void *ud)
{
    _count_ctx_t *c = (_count_ctx_t *)ud;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE ||
        fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS) c->count++;
}

// Encontrar control_id de un param_id
static void _find_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)ud;
    if (ctx->found_ctrl_id < 0 && param->param_id == ctx->target_param_id)
        ctx->found_ctrl_id = ctrl_id;
}

// Callback ciclo completo
static void _poll_param(int control_id, const kx_param_t *param, void *ud)
{
    _poll_ctx_t *ctx = (_poll_ctx_t *)ud;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return;

    uint8_t fc_read = (uint8_t)param->function_read;
    bool is_read_fc = (fc_read == MB_FC_READ_COILS       ||
                       fc_read == MB_FC_READ_DISCRETE     ||
                       fc_read == MB_FC_READ_HOLDING_REGS ||
                       fc_read == MB_FC_READ_INPUT_REGS);
    if (!is_read_fc) return;

    int64_t now_ms      = (int64_t)(esp_timer_get_time() / 1000ULL);
    int     sampling_ms = (param->sampling > 0 ? param->sampling : 60) * 1000;
    bool    report_due  = (param->sampling > 0) &&
                          (ctx->cycle_s % (int64_t)param->sampling == 0);
    bool    read_due    = ctx->demand_active ||
                          (param->ts_last_read == 0) ||
                          ((now_ms - param->ts_last_read) >= (int64_t)sampling_ms);

    if (!read_due && !report_due) { ctx->skipped++; return; }

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);
    ctx->done++;

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
    }
    kx_param_t *mp = kx_param_store_get_param_mutable(control_id, param->param_id);
    if (mp) { mp->ts_last_read = now_ms; mp->last_published_value = value; }

    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
}

// =============================================================
// Poll de un único param_id — devuelve true si OK
// Modificado para retornar mediante out_dropped si falló la cola MQTT
// =============================================================
static bool _poll_single_param(int param_id, char *err_out, size_t err_len, bool *out_dropped)
{
    if (out_dropped) *out_dropped = false; // Por defecto no hay drop

    _find_ctrl_ctx_t fctx = { .target_param_id = param_id, .found_ctrl_id = -1 };
    kx_param_store_foreach(_find_ctrl_cb, &fctx);
    if (fctx.found_ctrl_id < 0) {
        snprintf(err_out, err_len, "not found in any control"); return false;
    }
    const kx_param_t *param = kx_param_store_get_param(fctx.found_ctrl_id, param_id);
    if (!param) { snprintf(err_out, err_len, "get_param returned NULL"); return false; }

    const kx_control_params_t *ctrl = kx_param_store_get(fctx.found_ctrl_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        snprintf(err_out, err_len, "ctrl=%d no slave_addr", fctx.found_ctrl_id);
        return false;
    }
    uint8_t fc_read = (uint8_t)param->function_read;
    bool is_read_fc = (fc_read == MB_FC_READ_COILS       ||
                       fc_read == MB_FC_READ_DISCRETE     ||
                       fc_read == MB_FC_READ_HOLDING_REGS ||
                       fc_read == MB_FC_READ_INPUT_REGS);
    if (!is_read_fc || param->view == 0) {
        snprintf(err_out, err_len, "not readable/visible"); return false;
    }

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);
    if (value == -FLT_MAX) {
        bool enq = _enqueue(PUB_KIND_ERROR, fctx.found_ctrl_id, param->param_id,
                 0.0f, (uint16_t)param->reg, "modbus_timeout");
        if (out_dropped) *out_dropped = !enq;
        snprintf(err_out, err_len, "modbus_timeout reg=0x%04x", param->reg);
        return false;
    }
    
    // Capturamos el resultado booleano directo del _enqueue
    bool enq = _enqueue(PUB_KIND_STATUS, fctx.found_ctrl_id, param->param_id, value, 0, NULL);
    if (out_dropped) *out_dropped = !enq;

    kx_param_t *mp = kx_param_store_get_param_mutable(fctx.found_ctrl_id, param_id);
    if (mp) {
        mp->ts_last_read         = (int64_t)(esp_timer_get_time() / 1000ULL);
        mp->last_published_value = value;
    }
    return true;
}

// =============================================================
// Tarea publisher — consume s_pub_queue y publica por MQTT
// =============================================================
static void _publisher_task(void *arg)
{
    kx_pub_result_t r;
    ESP_LOGI(TAG, "publisher task started (queue_size=%d)", PUB_QUEUE_SIZE);
    while (1) {
        if (xQueueReceive(s_pub_queue, &r, portMAX_DELAY) == pdTRUE) {
            switch (r.kind) {
            case PUB_KIND_STATUS: kx_param_pub_status(r.control_id, r.param_id, r.value); break;
            case PUB_KIND_REPORT: kx_param_pub_report(r.control_id, r.param_id, r.value); break;
            case PUB_KIND_ERROR:  kx_param_pub_error (r.control_id, r.param_id, r.error_msg, r.reg); break;
            }
        }
    }
}

// =============================================================
// Resultado de cada demanda individual en el batch
// =============================================================
typedef struct {
    int  param_id;
    bool ok;
    bool dropped;          // true si _enqueue devolvió false (pub_queue llena)
    char err_msg[48];
} _batch_result_t;

// =============================================================
// _drain_demand_queue
//
// Vacía s_demand_queue hacia un array snapshot[].
// Aplica:
//   1. Filtro de caducidad (age > KX_DEMAND_TIMEOUT_S).
//   2. Deduplicación O(n): para cada param_id solo se guarda
//      la entrada más reciente (mayor enqueued_ms).
//
// Devuelve el número de entradas válidas en snapshot[].
// snapshot debe tener capacidad para 'capacity' elementos.
// =============================================================
static int _drain_demand_queue(kx_poll_demand_t *snapshot, int capacity,
                                int *out_expired, int *out_dupes)
{
    int64_t now_ms  = (int64_t)(esp_timer_get_time() / 1000ULL);
    int     expired = 0;
    int     dupes   = 0;
    int     count   = 0;

    kx_poll_demand_t d;
     while (count < capacity && xQueueReceive(s_demand_queue, &d, 0) == pdTRUE) {
        _pending_clear(d.param_id);

        // 1. Filtro de caducidad
        if ((now_ms - d.enqueued_ms) > (int64_t)(KX_DEMAND_TIMEOUT_S * 1000)) {
            expired++;
            continue;
        }

        // 2. Deduplicación: buscar si ya existe en snapshot[]
        bool found = false;
        for (int j = 0; j < count; j++) {
            if (snapshot[j].param_id == d.param_id) {
                // Conservar la entrada más reciente
                if (d.enqueued_ms > snapshot[j].enqueued_ms) {
                    snapshot[j].enqueued_ms = d.enqueued_ms;
                }
                dupes++;
                found = true;
                break;
            }
        }
        if (!found) snapshot[count++] = d;
    }

    // Si la cola tenía más entradas de las que cabían en snapshot,
    // vaciar el resto (caducadas o sobrantes) para no dejar basura.
    int leftovers = 0;
    kx_poll_demand_t tmp;
    while (xQueueReceive(s_demand_queue, &tmp, 0) == pdTRUE) leftovers++;
    if (leftovers > 0) {
        ESP_LOGW(TAG,
            "_drain: %d leftover demands discarded (snapshot capacity=%d)",
            leftovers, capacity);
    }

    if (out_expired) *out_expired = expired;
    if (out_dupes)   *out_dupes   = dupes;
    return count;
}

// =============================================================
// Tarea principal Modbus
// =============================================================
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "task started — waiting for entities...");
    while (!kx_param_store_is_ready()) vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "entities ready (%d controls) — ready for poll demands",
             kx_param_store_count());

    int64_t cycle_s = 5;

    while (s_running) {

        // ── 1. Esperar primera demanda ────────────────────────
        xEventGroupWaitBits(s_poll_eg, DEMAND_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        // ── 2. Fase de recopilación de ráfaga ─────────────────
        //   Esperamos hasta que el tamaño de s_demand_queue se
        //   estabilice BURST_STABLE_MS ms o pasen BURST_COLLECT_MAX_MS.
        {
            int64_t t0      = (int64_t)(esp_timer_get_time() / 1000ULL);
            int     prev    = -1;
            int     stable  = 0;
            ESP_LOGI(TAG, "collecting burst...");

            while (1) {
                vTaskDelay(pdMS_TO_TICKS(BURST_POLL_MS));
                int cur     = (int)uxQueueMessagesWaiting(s_demand_queue);
                int64_t ela = (int64_t)(esp_timer_get_time() / 1000ULL) - t0;

                if (cur == prev) {
                    stable += BURST_POLL_MS;
                    if (stable >= BURST_STABLE_MS) {
                        ESP_LOGI(TAG, "burst stable: %d demands in %" PRId64 "ms",
                                 cur, ela);
                        break;
                    }
                } else {
                    stable = 0;
                }
                prev = cur;
                if (ela >= BURST_COLLECT_MAX_MS) {
                    ESP_LOGW(TAG, "burst timeout (%dms): %d demands",
                             BURST_COLLECT_MAX_MS, cur);
                    break;
                }
            }
        }

        // ── 3. Snapshot atómico con deduplicación ─────────────
        int raw_count = (int)uxQueueMessagesWaiting(s_demand_queue);
        if (raw_count == 0) {
            xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
            continue;
        }

        // Reservamos por el tamaño bruto; _drain reducirá si hay dupes.
        kx_poll_demand_t *snapshot = malloc((size_t)raw_count *
                                            sizeof(kx_poll_demand_t));
        if (!snapshot) {
            ESP_LOGE(TAG,
                "OOM snapshot (%d demands) — flushing demand_queue", raw_count);
            // Liberar toda la cola para no quedar bloqueados
            kx_poll_demand_t tmp;
            while (xQueueReceive(s_demand_queue, &tmp, 0) == pdTRUE) {}
            xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
            continue;
        }

        int expired = 0, dupes = 0;
        int valid_count = _drain_demand_queue(snapshot, raw_count,
                                              &expired, &dupes);

        xEventGroupClearBits(s_poll_eg, DEMAND_BIT);

        ESP_LOGI(TAG,
            "snapshot: raw=%d valid=%d expired=%d dupes=%d",
            raw_count, valid_count, expired, dupes);

        if (valid_count == 0) { free(snapshot); continue; }

        // ── 4. Ciclo completo vs batch individual ──────────────
        bool has_full_cycle = false;
        for (int i = 0; i < valid_count; i++) {
            if (snapshot[i].param_id == 0) { has_full_cycle = true; break; }
        }

        if (has_full_cycle) {
            ESP_LOGI(TAG, "full poll cycle (cycle_s=%" PRId64 ")", cycle_s);
            if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
                ESP_LOGW(TAG, "full cycle skipped: no mqtt/store");
                free(snapshot);
                continue;
            }
            xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);
            xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
            if (s_running && kx_param_store_is_ready()) {
                _count_ctx_t cc = { .count = 0 };
                kx_param_store_foreach(_count_readable, &cc);
                _poll_ctx_t ctx = {
                    .total = cc.count, .cycle_s = cycle_s, .demand_active = true
                };
                KX_LOG_CYCLE_START(TAG, cycle_s, kx_param_store_count(), ctx.total);
                kx_param_store_foreach(_poll_param, &ctx);
                KX_LOG_CYCLE_END(TAG, ctx.ok, ctx.errors, ctx.skipped, ctx.unchanged);
                cycle_s = (cycle_s + KX_TELEMETRY_INTERVAL_S) % 60;
            }
            xSemaphoreGive(s_foreach_mutex);
            free(snapshot);
            continue;
        }

        // ── 5. Batch de demandas individuales ─────────────────
        bool batch_mode = (valid_count >= BATCH_THRESHOLD);

        if (batch_mode) {
            printf("\n");
            printf("┌──────────────────────────────────────────────────────┐\n");
            printf("│  BATCH POLL — %4d params  (dupes descartados: %3d)  │\n",
                   valid_count, dupes);
            printf("└──────────────────────────────────────────────────────┘\n");
            fflush(stdout);
        } else {
            ESP_LOGI(TAG, "processing %d demand(s) (dupes=%d expired=%d)",
                     valid_count, dupes, expired);
        }

        _batch_result_t *results = malloc((size_t)valid_count *
                                           sizeof(_batch_result_t));
        if (!results) {
            ESP_LOGE(TAG, "OOM results array — aborting batch");
            free(snapshot);
            continue;
        }
        memset(results, 0, (size_t)valid_count * sizeof(_batch_result_t));

        int batch_ok      = 0;
        int batch_errors  = 0;
        int batch_dropped = 0;   // drops de pub_queue por backpressure

        for (int i = 0; i < valid_count && s_running; i++) {
            int param_id = snapshot[i].param_id;
            results[i].param_id = param_id;

            if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
                snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                         "mqtt/store not ready");
                results[i].ok = false;
                batch_errors++;
                if (batch_mode)
                    _print_batch_progress(i + 1, valid_count,
                                          batch_ok, batch_errors);
                continue;
            }

            xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);
            xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);

            bool ok = false;
            bool dropped = false; // Bandera local para determinar el drop real

            if (s_running && kx_param_store_is_ready()) {
                // Ejecutamos la función pasando la referencia de la bandera
                ok = _poll_single_param(param_id,
                                        results[i].err_msg,
                                        sizeof(results[i].err_msg),
                                        &dropped);

                // Si la lectura Modbus fue OK pero el enqueue determinó un drop real
                if (ok && dropped) {
                    results[i].dropped = true;
                    batch_dropped++;
                    // No marcamos como error general: la lectura Modbus fue correcta,
                    // solo falló de verdad la publicación MQTT.
                    snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                             "pub_queue drop");
                }
            } else {
                snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                         "task stopping");
            }

            xSemaphoreGive(s_foreach_mutex);

            results[i].ok = ok;
            if (ok)  batch_ok++;
            else     batch_errors++;

            if (batch_mode)
                _print_batch_progress(i + 1, valid_count, batch_ok, batch_errors);

            if (i + 1 < valid_count)
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
        }

        if (batch_mode) { printf("\n"); fflush(stdout); }

        // ── 6. Resumen detallado ───────────────────────────────
        printf("┌──────────────────────────────────────────────────────────────┐\n");
        printf("│                    BATCH POLL  RESUMEN                        │\n");
        printf("├──────────────────────────────────────────────────────────────┤\n");
        printf("│  Recopilados : %-5d  (dupes descartados: %-5d)             │\n",
               valid_count, dupes);
        printf("│  Expirados   : %-5d                                          │\n",
               expired);
        printf("│  OK (leídos) : %-5d                                          │\n",
               batch_ok);
        printf("│  Errores     : %-5d                                          │\n",
               batch_errors);
        printf("│  Drops MQTT  : %-5d  (pub_queue llena tras backpressure)     │\n",
               batch_dropped);
        printf("│  Heap libre  : %-8lu bytes                                │\n",
               (unsigned long)kx_system_heap_free());
        printf("│  pub_queue   : %-5d / %-5d slots usados                     │\n",
               (int)uxQueueMessagesWaiting(s_pub_queue), PUB_QUEUE_SIZE);
        printf("│  demand_queue: %-5d / %-5d slots pendientes                 │\n",
               (int)uxQueueMessagesWaiting(s_demand_queue), DEMAND_QUEUE_SIZE);

        // Lista param_ids OK
        if (batch_ok > 0) {
            printf("├──────────────────────────────────────────────────────────────┤\n");
            printf("│  ✓ Params OK:                                                │\n│    ");
            int col = 4;
            for (int i = 0; i < valid_count; i++) {
                if (!results[i].ok) continue;
                char tok[16];
                int tl = snprintf(tok, sizeof(tok), "%d", results[i].param_id);
                if (col + tl + 1 > 62) { printf("\n│    "); col = 4; }
                printf("%s ", tok);
                col += tl + 1;
            }
            printf("\n");
        }

        // Lista param_ids con error
        if (batch_errors > 0) {
            printf("├──────────────────────────────────────────────────────────────┤\n");
            printf("│  ✗ Params con error:                                         │\n");
            int shown = 0;
            for (int i = 0; i < valid_count && shown < BATCH_REPORT_MAX_ERRORS; i++) {
                if (results[i].ok) continue;
                printf("│    param_id=%-10d  %s\n",
                       results[i].param_id, results[i].err_msg);
                shown++;
            }
            if (batch_errors > BATCH_REPORT_MAX_ERRORS) {
                printf("│    ... y %d más\n",
                       batch_errors - BATCH_REPORT_MAX_ERRORS);
            }
        }

        // Lista params con drop de pub_queue
        if (batch_dropped > 0) {
            printf("├──────────────────────────────────────────────────────────────┤\n");
            printf("│  ⚠ Params leídos pero NO publicados (pub_queue drop):        │\n│    ");
            int col = 4;
            for (int i = 0; i < valid_count; i++) {
                if (!results[i].dropped) continue;
                char tok[16];
                int tl = snprintf(tok, sizeof(tok), "%d", results[i].param_id);
                if (col + tl + 1 > 62) { printf("\n│    "); col = 4; }
                printf("%s ", tok);
                col += tl + 1;
            }
            printf("\n");
        }

        printf("└──────────────────────────────────────────────────────────────┘\n");
        fflush(stdout);

        ESP_LOGI(TAG,
            "batch done — valid=%d ok=%d errors=%d drops=%d dupes=%d expired=%d heap=%" PRIu32,
            valid_count, batch_ok, batch_errors, batch_dropped,
            dupes, expired, kx_system_heap_free());

        free(results);
        free(snapshot);
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
// start / stop
// =============================================================
esp_err_t kx_modbus_master_start(void)
{
    if (s_running) { ESP_LOGW(TAG, "already running"); return ESP_OK; }

    s_pub_queue = xQueueCreate(PUB_QUEUE_SIZE, sizeof(kx_pub_result_t));
    if (!s_pub_queue) { ESP_LOGE(TAG, "pub_queue alloc failed"); return ESP_FAIL; }

    s_demand_queue = xQueueCreate(DEMAND_QUEUE_SIZE, sizeof(kx_poll_demand_t));
    if (!s_demand_queue) { ESP_LOGE(TAG, "demand_queue alloc failed"); return ESP_FAIL; }

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) { ESP_LOGE(TAG, "EventGroup alloc failed"); return ESP_FAIL; }

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) { ESP_LOGE(TAG, "mutex alloc failed"); return ESP_FAIL; }

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);

    esp_err_t err = _uart_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "UART init: %s", esp_err_to_name(err)); return err; }

    BaseType_t ret = xTaskCreate(_publisher_task, "kx_publisher", 4096, NULL,
                                  KX_TASK_PRIO_TELEMETRY - 1, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "publisher task failed"); return ESP_FAIL; }

    s_running = true;
    ret = xTaskCreate(_modbus_task, "kx_modbus", 8192, NULL,
                      KX_TASK_PRIO_TELEMETRY, &s_task);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    return ESP_OK;
}

void kx_modbus_master_stop(void) { s_running = false; }
bool kx_modbus_master_is_running(void) { return s_running; }

// =============================================================
// kx_modbus_read_one
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
    if (!param) { ESP_LOGW(TAG, "write_one: param not found"); return ESP_ERR_NOT_FOUND; }
    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "write_one: no slave_addr"); return ESP_ERR_INVALID_STATE;
    }
    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL  &&
        fc_write != MB_FC_WRITE_SINGLE_REG   &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS) {
        ESP_LOGW(TAG, "write_one: unsupported FC 0x%02x", fc_write);
        return ESP_ERR_NOT_SUPPORTED;
    }
 
    // ── Transformación inversa value → raw ───────────────────
    int16_t raw;
 
    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        // [FIX] Para Coils (FC 05), Modbus requiere estrictamente 0xFF00 para ON y 0x0000 para OFF.
        // Ignoramos por completo offsets, additions y el clamping de registros.
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        // Lógica normal para registros (FC 06, FC 16, etc.)
        // value = (float)(int16_t)raw -> inversa: adjusted = value - addition
        float adjusted = value - param->addition;
 
        if (param->offset != 0.0f && param->offset != 1.0f) {
            raw = (int16_t)(adjusted / param->offset);
        } else {
            raw = (int16_t)adjusted;
        }
 
        // Clampear al rango permitido (en unidades raw, pre-transformación)
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }
 
    // Modificado el log para imprimir también en Hexadecimal (ayuda mucho con las Coils)
    ESP_LOGI(TAG, "write_one: ctrl=%d param=%d reg=0x%04x fc=0x%02x "
             "slave=%d value=%.3f → raw=%d (0x%04X)",
             control_id, param_id, param->reg, fc_write,
             ctrl->slave_addr, value, (int)(uint16_t)raw, (uint16_t)raw);
 
    // ── Construir trama Modbus ────────────────────────────────
    // Nota: El arreglo de 6 bytes funciona idéntico tanto para FC 05 como para FC 06.
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
    for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) {
            ESP_LOGD(TAG, "write_one: intento %d fallido...", a + 1);
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
    }
 
    if (rx < 0) {
        ESP_LOGW(TAG, "write_one: sin respuesta tras %d intentos ctrl=%d param=%d",
                 MODBUS_RETRY_COUNT, control_id, param_id);
        return ESP_FAIL;
    }
 
    // ── Validar respuesta eco (FC 05 y FC 06) ─────────────────
    // Ambos comandos devuelven un espejo exacto de los primeros 4 bytes.
    if (rx < 6 ||
        resp[0] != frame[0] ||   // slave_addr
        resp[1] != frame[1] ||   // function code
        resp[2] != frame[2] ||   // reg_hi
        resp[3] != frame[3])     // reg_lo
    {
        ESP_LOGW(TAG, "write_one: respuesta inesperada (rx=%d) ctrl=%d param=%d",
                 rx, control_id, param_id);
        return ESP_FAIL;
    }
    uint16_t echo = ((uint16_t)resp[4] << 8) | resp[5];
    ESP_LOGI(TAG, "write_one: OK raw_sent=%d raw_echo=%d",
             (int)(uint16_t)raw, (int)echo);
    return ESP_OK;
}