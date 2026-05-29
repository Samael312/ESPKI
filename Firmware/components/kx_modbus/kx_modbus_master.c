#include "kx_modbus_master.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "kx_telemetry.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "kx_modbus_packetizer.h"
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
#include <inttypes.h>

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
#define MODBUS_INTER_PARAM_MS          15
#define MODBUS_RETRY_COUNT              2

#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

// =============================================================
// Cola de publicación
// =============================================================
#define PUB_QUEUE_SIZE               500
#define PUB_QUEUE_BACKPRESSURE_HWM   350
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
// Cola de escrituras
// =============================================================
#define WRITE_QUEUE_SIZE  64

typedef struct {
    int    control_id;
    int    param_id;
    float  value;
    double ts;
} kx_write_cmd_t;

static QueueHandle_t s_write_queue = NULL;

// =============================================================
// Cola de demandas de poll
// =============================================================
#ifndef KX_DEMAND_REPEAT_MS
#define KX_DEMAND_REPEAT_MS  5000
#endif

#define DEMAND_QUEUE_SIZE    1500
#define DEMAND_WARN_HWM       700

#define BURST_COLLECT_MAX_MS  3000
#define BURST_STABLE_MS        300
#define BURST_POLL_MS           50

#define BATCH_THRESHOLD          5
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
// Timer de reports
// =============================================================
#define REPORT_TICK_PERIOD_S   864000
#define REPORT_TASK_PERIOD_MS  1000

static volatile int64_t s_report_tick_s = -1;

// =============================================================
// Umbral de cambio (status)
// =============================================================
#define KX_STATUS_DELTA_ABS   0.5f
#define KX_STATUS_DELTA_REL   0.01f

// =============================================================
// Barra de progreso ASCII
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
// Forward declarations
// =============================================================
static float _read_register(uint8_t slave_addr, uint16_t reg_addr,
                             uint8_t fc, const kx_param_t *param);

// =============================================================
// _enqueue
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

    if (xQueueSend(s_pub_queue, &r, 0) == pdTRUE) return true;

    int waited = 0;
    ESP_LOGW(TAG, "pub_queue backpressure: %d/%d",
             (int)uxQueueMessagesWaiting(s_pub_queue), PUB_QUEUE_SIZE);

    while (waited < PUB_BACKPRESSURE_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(PUB_BACKPRESSURE_WAIT_MS));
        waited += PUB_BACKPRESSURE_WAIT_MS;
        if (xQueueSend(s_pub_queue, &r, 0) == pdTRUE) {
            ESP_LOGI(TAG, "pub_queue backpressure released after %dms", waited);
            return true;
        }
    }

    ESP_LOGE(TAG, "pub_queue DROP param_id=%d after %dms (queue=%d/%d)",
             param_id, waited,
             (int)uxQueueMessagesWaiting(s_pub_queue), PUB_QUEUE_SIZE);
    return false;
}

// =============================================================
// API pública — encolar escritura
// =============================================================
esp_err_t kx_modbus_enqueue_write(int control_id, int param_id,
                                   float value, double ts)
{
    if (!s_write_queue) return ESP_ERR_INVALID_STATE;

    kx_write_cmd_t cmd = {
        .control_id = control_id,
        .param_id   = param_id,
        .value      = value,
        .ts         = ts,
    };

    if (xQueueSend(s_write_queue, &cmd, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGE(TAG, "write_queue FULL — dropping write ctrl=%d param=%d",
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

    if (param_id != 0 && _pending_test(param_id)) {
        ESP_LOGD(TAG, "demand dedup (bitmap) param_id=%d", param_id);
        return;
    }

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    int     used   = (int)uxQueueMessagesWaiting(s_demand_queue);

    if (used >= DEMAND_WARN_HWM)
        ESP_LOGW(TAG, "demand_queue near full: %d/%d", used, DEMAND_QUEUE_SIZE);

    kx_poll_demand_t d = { .param_id = param_id, .enqueued_ms = now_ms };
    if (xQueueSend(s_demand_queue, &d, 0) != pdTRUE) {
        ESP_LOGE(TAG, "demand_queue FULL (%d slots), dropping param_id=%d",
                 DEMAND_QUEUE_SIZE, param_id);
        return;
    }

    if (param_id != 0) _pending_set(param_id);
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
        if (err != ESP_OK)
            ESP_LOGW(TAG, "RS485 mode failed: %s", esp_err_to_name(err));
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
        // FIX: log con slave y registro para diagnóstico
        ESP_LOGW(TAG, "Modbus exception: slave=%d fc=0x%02x exc=0x%02x "
                 "(req_fc=0x%02x reg=0x%02x%02x cnt=0x%02x%02x)",
                 frame[0], resp[1], exc,
                 frame[1], frame[2], frame[3], frame[4], frame[5]);
        return -1;
    }
    return rx_len;
}

// =============================================================
// Leer un registro individual
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
// Leer N registros en una sola trama
// =============================================================
static int _read_registers_multi(uint8_t slave_addr,
                                  uint16_t start_reg,
                                  uint16_t num_regs,
                                  uint8_t fc,
                                  uint8_t *resp_buf,
                                  size_t   resp_max)
{
    uint8_t frame[6] = {
        slave_addr,
        fc,
        (uint8_t)(start_reg >> 8),
        (uint8_t)(start_reg & 0xFF),
        (uint8_t)(num_regs  >> 8),
        (uint8_t)(num_regs  & 0xFF),
    };

    int rx = -1;
    for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp_buf, resp_max);
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }
    return rx;
}

// =============================================================
// _dispatch_packet
//
// Envía un kx_packet_t al bus, decodifica la respuesta y
// actualiza kx_param_store (niveles 2 y 3) y s_pub_queue.
//
// IMPORTANTE: si el esclavo devuelve Modbus exception (exc=0x03
// Illegal Data Value) en un packet multi-registro, el fallback
// automático reintenta registro a registro para aislar el fallo
// y maximizar los params que sí se pueden leer.
//
// Requiere que el llamador ya haya tomado s_foreach_mutex.
// Devuelve el número de params leídos con éxito.
// =============================================================
static int _dispatch_packet(const kx_packet_t *pkt,
                             kx_pub_kind_t pub_kind,
                             int *out_errors)
{
    if (!pkt || pkt->num_slots == 0) return 0;

    int ok_count  = 0;
    int err_count = 0;

    // ── Caso 1: packet individual (num_regs == 1) ─────────────
    if (pkt->num_regs == 1) {
        const kx_pkt_slot_t *slot = &pkt->slots[0];
        if (slot->is_gap) {
            // Un gap en un packet individual no debería ocurrir,
            // pero si ocurre no es un error contable.
            goto dispatch_done;
        }

        const kx_param_t *param =
            kx_param_store_get_param(slot->control_id, slot->param_id);
        if (!param) {
            ESP_LOGW(TAG, "dispatch individual: param not found ctrl=%d p=%d",
                     slot->control_id, slot->param_id);
            err_count++;
            goto dispatch_done;
        }

        float value = _read_register(pkt->slave_addr, pkt->start_reg,
                                     pkt->fc, param);
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (value == -FLT_MAX) {
            _enqueue(PUB_KIND_ERROR, slot->control_id, slot->param_id,
                     0.0f, pkt->start_reg, "modbus_timeout");
            err_count++;
        } else {
            kx_param_store_reg_upsert_read(
                slot->control_id, pkt->start_reg, pkt->fc,
                (uint8_t)param->function_write, value, ts_ms);

            kx_param_t *mp = kx_param_store_get_param_mutable(
                                slot->control_id, slot->param_id);
            if (mp) {
                mp->ts_last_read         = ts_ms;
                mp->last_published_value = value;
            }
            _enqueue(pub_kind, slot->control_id, slot->param_id,
                     value, 0, NULL);
            ok_count++;
        }
        goto dispatch_done;
    }

    // ── Caso 2: packet multi-registro ─────────────────────────
    {
        uint8_t resp[KX_PKT_MAX_REGS_PER_PKT * 2 + 8];
        int rx = _read_registers_multi(pkt->slave_addr, pkt->start_reg,
                                       pkt->num_regs, pkt->fc,
                                       resp, sizeof(resp));
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        // ── Fallback individual si el multi falla ─────────────
        // exc=0x03 (Illegal Data Value) indica que el esclavo no
        // soporta leer ese rango de registros de golpe.
        // Reintentamos slot a slot para rescatar los que sí existen.
        if (rx < 0) {
            ESP_LOGW(TAG,
                     "dispatch multi FAILED (slave=%d fc=0x%02x "
                     "reg=0x%04x num=%d) — falling back to individual reads",
                     pkt->slave_addr, pkt->fc,
                     pkt->start_reg, pkt->num_regs);

            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *slot = &pkt->slots[s];
                if (slot->is_gap || slot->param_id < 0) continue;

                const kx_param_t *param =
                    kx_param_store_get_param(slot->control_id, slot->param_id);
                if (!param) { err_count++; continue; }

                float value = _read_register(pkt->slave_addr, slot->reg,
                                             pkt->fc, param);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

                if (value == -FLT_MAX) {
                    _enqueue(PUB_KIND_ERROR, slot->control_id, slot->param_id,
                             0.0f, slot->reg, "modbus_timeout");
                    err_count++;
                } else {
                    kx_param_store_reg_upsert_read(
                        slot->control_id, slot->reg, pkt->fc,
                        (uint8_t)param->function_write, value, ts_ms);

                    kx_param_t *mp = kx_param_store_get_param_mutable(
                                        slot->control_id, slot->param_id);
                    if (mp) {
                        mp->ts_last_read         = ts_ms;
                        mp->last_published_value = value;
                    }
                    _enqueue(pub_kind, slot->control_id, slot->param_id,
                             value, 0, NULL);
                    ok_count++;
                }

                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
            }
            goto dispatch_done;
        }

        // ── Verificar byte_count ──────────────────────────────
        bool is_coil = (pkt->fc == MB_FC_READ_COILS ||
                        pkt->fc == MB_FC_READ_DISCRETE);
        int  expected_bytes = is_coil ? (pkt->num_regs + 7) / 8
                                      : pkt->num_regs * 2;

        if (rx < 3 || resp[2] != (uint8_t)expected_bytes) {
            ESP_LOGW(TAG,
                     "dispatch multi: bad byte_count=%d expected=%d rx=%d "
                     "(slave=%d reg=0x%04x num=%d) — falling back",
                     (rx >= 3) ? resp[2] : -1, expected_bytes, rx,
                     pkt->slave_addr, pkt->start_reg, pkt->num_regs);

            // Mismo fallback individual
            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *slot = &pkt->slots[s];
                if (slot->is_gap || slot->param_id < 0) continue;

                const kx_param_t *param =
                    kx_param_store_get_param(slot->control_id, slot->param_id);
                if (!param) { err_count++; continue; }

                float value = _read_register(pkt->slave_addr, slot->reg,
                                             pkt->fc, param);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

                if (value == -FLT_MAX) {
                    _enqueue(PUB_KIND_ERROR, slot->control_id, slot->param_id,
                             0.0f, slot->reg, "modbus_timeout");
                    err_count++;
                } else {
                    kx_param_store_reg_upsert_read(
                        slot->control_id, slot->reg, pkt->fc,
                        (uint8_t)param->function_write, value, ts_ms);

                    kx_param_t *mp = kx_param_store_get_param_mutable(
                                        slot->control_id, slot->param_id);
                    if (mp) {
                        mp->ts_last_read         = ts_ms;
                        mp->last_published_value = value;
                    }
                    _enqueue(pub_kind, slot->control_id, slot->param_id,
                             value, 0, NULL);
                    ok_count++;
                }
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
            }
            goto dispatch_done;
        }

        // ── Decodificar slots exitosamente ────────────────────
        for (int s = 0; s < pkt->num_slots; s++) {
            const kx_pkt_slot_t *slot = &pkt->slots[s];
            int reg_offset = (int)slot->reg - (int)pkt->start_reg;

            uint16_t raw;
            if (is_coil) {
                int byte_idx = reg_offset / 8;
                int bit_idx  = reg_offset % 8;
                if (3 + byte_idx >= rx) continue;
                raw = (resp[3 + byte_idx] >> bit_idx) & 0x01;
            } else {
                int byte_idx = reg_offset * 2;
                if (3 + byte_idx + 1 >= rx) continue;
                raw = ((uint16_t)resp[3 + byte_idx] << 8) |
                                 resp[3 + byte_idx + 1];
            }

            if (slot->is_gap) {
                kx_param_store_reg_upsert_read(
                    slot->control_id, slot->reg, pkt->fc,
                    0, (float)(int16_t)raw, ts_ms);
                continue;
            }

            const kx_param_t *param =
                kx_param_store_get_param(slot->control_id, slot->param_id);
            if (!param) {
                ESP_LOGW(TAG, "dispatch multi slot[%d]: param not found p=%d",
                         s, slot->param_id);
                err_count++;
                continue;
            }

            float value = (float)(int16_t)raw;
            if (param->offset != 0.0f && param->offset != 1.0f)
                value *= param->offset;
            value += param->addition;
            if (value < param->minvalue) value = param->minvalue;
            if (value > param->maxvalue) value = param->maxvalue;

            kx_param_store_reg_upsert_read(
                slot->control_id, slot->reg, pkt->fc,
                (uint8_t)param->function_write, value, ts_ms);

            kx_param_t *mp = kx_param_store_get_param_mutable(
                                slot->control_id, slot->param_id);
            if (mp) {
                mp->ts_last_read         = ts_ms;
                mp->last_published_value = value;
            }
            _enqueue(pub_kind, slot->control_id, slot->param_id,
                     value, 0, NULL);
            ok_count++;
        }
    }

dispatch_done:
    if (out_errors) *out_errors += err_count;
    return ok_count;
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
// Ejecutar escritura Modbus (solo desde _writer_task)
// =============================================================
static esp_err_t _execute_write(int control_id, int param_id, float value)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) {
        ESP_LOGW(TAG, "write: param not found ctrl=%d param=%d", control_id, param_id);
        return ESP_ERR_NOT_FOUND;
    }
    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "write: no slave_addr ctrl=%d", control_id);
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL  &&
        fc_write != MB_FC_WRITE_SINGLE_REG   &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS) {
        ESP_LOGW(TAG, "write: unsupported FC 0x%02x", fc_write);
        return ESP_ERR_NOT_SUPPORTED;
    }

    int16_t raw;
    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        float adjusted = value - param->addition;
        if (param->offset != 0.0f && param->offset != 1.0f)
            raw = (int16_t)(adjusted / param->offset);
        else
            raw = (int16_t)adjusted;
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }

    ESP_LOGI(TAG, "write: ctrl=%d param=%d reg=0x%04x fc=0x%02x "
             "slave=%d value=%.3f → raw=%d (0x%04X)",
             control_id, param_id, param->reg, fc_write,
             ctrl->slave_addr, value, (int)(uint16_t)raw, (uint16_t)raw);

    uint8_t frame[6] = {
        (uint8_t)ctrl->slave_addr, fc_write,
        (uint8_t)((uint16_t)param->reg >> 8),
        (uint8_t)((uint16_t)param->reg & 0xFF),
        (uint8_t)((uint16_t)raw >> 8),
        (uint8_t)((uint16_t)raw & 0xFF),
    };
    uint8_t resp[16];
    int rx = -1;
    for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }

    if (rx < 0) {
        ESP_LOGW(TAG, "write: no response after %d retries ctrl=%d param=%d",
                 MODBUS_RETRY_COUNT, control_id, param_id);
        return ESP_FAIL;
    }
    if (rx < 6 ||
        resp[0] != frame[0] || resp[1] != frame[1] ||
        resp[2] != frame[2] || resp[3] != frame[3]) {
        ESP_LOGW(TAG, "write: unexpected response rx=%d", rx);
        return ESP_FAIL;
    }

    uint16_t echo = ((uint16_t)resp[4] << 8) | resp[5];
    ESP_LOGI(TAG, "write: OK raw_sent=%d raw_echo=%d",
             (int)(uint16_t)raw, (int)echo);
    return ESP_OK;
}

// =============================================================
// Tarea writer — ALTA PRIORIDAD
// =============================================================
static void _writer_task(void *arg)
{
    kx_write_cmd_t cmd;
    ESP_LOGI(TAG, "writer task started (HIGH PRIORITY)");

    while (1) {
        if (xQueueReceive(s_write_queue, &cmd, portMAX_DELAY) != pdTRUE) continue;

        ESP_LOGI(TAG, "writer: cmd ctrl=%d param=%d value=%.3f ts=%.3f",
                 cmd.control_id, cmd.param_id, cmd.value, cmd.ts);

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);

        esp_err_t err = _execute_write(cmd.control_id, cmd.param_id, cmd.value);

        if (err == ESP_OK) {
            kx_param_store_set_ts_set(cmd.control_id, cmd.param_id, cmd.ts);
            _enqueue(PUB_KIND_STATUS, cmd.control_id, cmd.param_id,
                     cmd.value, 0, NULL);
            ESP_LOGI(TAG, "writer: OK ctrl=%d param=%d value=%.3f",
                     cmd.control_id, cmd.param_id, cmd.value);
        } else {
            _enqueue(PUB_KIND_ERROR, cmd.control_id, cmd.param_id,
                     0.0f, 0, "modbus_write_error");
            ESP_LOGW(TAG, "writer: FAIL ctrl=%d param=%d err=%s",
                     cmd.control_id, cmd.param_id, esp_err_to_name(err));
        }

        xSemaphoreGive(s_foreach_mutex);
    }
}

// =============================================================
// Tipos de contexto para foreach
// =============================================================
typedef struct {
    int     total, done, ok, errors, skipped, unchanged;
    bool    demand_active;
} _poll_ctx_t;

typedef struct {
    int64_t tick_s;
    int     sent;
    int     errors;
} _report_ctx_t;

typedef struct { int count; } _count_ctx_t;
typedef struct { int target_param_id; int found_ctrl_id; } _find_ctrl_ctx_t;

#define MAX_CTRL_VISITED  KX_PARAM_MAX_CONTROLS

// =============================================================
// _poll_control_packetized
// Ciclo completo de poll para un control usando el packetizer.
// demand_active=true → tick_s irrelevante, se lee todo.
// =============================================================
static void _poll_control_packetized(int control_id, _poll_ctx_t *ctx)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    // En modo demanda tick_s no se usa (pasamos 0)
    kx_packet_list_t *list = kx_pkt_build(control_id,
                                           ctx->demand_active,
                                           0,        // tick_s ignorado
                                           now_ms);
    if (!list) {
        ESP_LOGD(TAG, "packetizer: ctrl=%d no packets", control_id);
        return;
    }

    if (ctx->total == 0)
        ctx->total += kx_pkt_real_param_count(list);

#if CONFIG_LOG_DEFAULT_LEVEL >= 4
    kx_pkt_dump(list, TAG);
#endif

    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];

        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
        int pkt_errors = 0;
        int pkt_ok = _dispatch_packet(pkt,
                                       ctx->demand_active
                                           ? PUB_KIND_STATUS
                                           : PUB_KIND_REPORT,
                                       &pkt_errors);
        xSemaphoreGive(s_foreach_mutex);

        ctx->ok     += pkt_ok;
        ctx->errors += pkt_errors;
        ctx->done   += pkt_ok + pkt_errors;

        if (i + 1 < list->count)
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
    }

    kx_pkt_free(list);
}

// =============================================================
// _poll_ctrl_cb — callback foreach para ciclo completo de poll
// =============================================================
typedef struct {
    _poll_ctx_t *ctx;
    int          visited[MAX_CTRL_VISITED];
    int          n_visited;
} _poll_ctrl_foreach_ud_t;

static void _poll_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    (void)param;
    _poll_ctrl_foreach_ud_t *u = (_poll_ctrl_foreach_ud_t *)ud;

    for (int i = 0; i < u->n_visited; i++)
        if (u->visited[i] == ctrl_id) return;
    if (u->n_visited < MAX_CTRL_VISITED)
        u->visited[u->n_visited++] = ctrl_id;

    _poll_control_packetized(ctrl_id, u->ctx);
}

// =============================================================
// _report_control_packetized
// Ciclo de report para un control.
// Pasa tick_s al packetizer para que filtre por
// tick_s % param->sampling == 0, igual que el comportamiento
// original de _report_param_cb.
// =============================================================
static void _report_control_packetized(int control_id, _report_ctx_t *rctx)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    kx_packet_list_t *list = kx_pkt_build(control_id,
                                           false,         // demand_active=false
                                           rctx->tick_s,  // FIX: pasar tick real
                                           now_ms);
    if (!list) return;

    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];

        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
        int pkt_errors = 0;
        int pkt_ok = _dispatch_packet(pkt, PUB_KIND_REPORT, &pkt_errors);
        xSemaphoreGive(s_foreach_mutex);

        rctx->sent   += pkt_ok;
        rctx->errors += pkt_errors;   // FIX: errors ya es fresco por tick

        if (i + 1 < list->count)
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
    }

    kx_pkt_free(list);
}

// =============================================================
// _report_ctrl_cb — callback foreach para la tarea de reports
// =============================================================
typedef struct {
    _report_ctx_t *rctx;
    int            visited[MAX_CTRL_VISITED];
    int            n_visited;
} _report_ctrl_foreach_ud_t;

static void _report_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    (void)param;
    _report_ctrl_foreach_ud_t *u = (_report_ctrl_foreach_ud_t *)ud;

    for (int i = 0; i < u->n_visited; i++)
        if (u->visited[i] == ctrl_id) return;
    if (u->n_visited < MAX_CTRL_VISITED)
        u->visited[u->n_visited++] = ctrl_id;

    _report_control_packetized(ctrl_id, u->rctx);
}

// =============================================================
// Tarea de reports — tick de 1 segundo
// =============================================================
static void _report_task(void *arg)
{
    ESP_LOGI(TAG, "report task started (period=%ds max_sampling=%ds)",
             REPORT_TASK_PERIOD_MS / 1000, REPORT_TICK_PERIOD_S);

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_TASK_PERIOD_MS));

        if (!kx_param_store_is_ready() || !kx_mqtt_is_connected()) continue;

        s_report_tick_s = (s_report_tick_s + 1) % REPORT_TICK_PERIOD_S;
        int64_t tick = s_report_tick_s;

        ESP_LOGD(TAG, "report tick=%" PRId64 "s", tick);

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        // FIX: rctx se crea FRESCO cada tick → errors siempre arranca en 0
        _report_ctx_t rctx = { .tick_s = tick, .sent = 0, .errors = 0 };
        _report_ctrl_foreach_ud_t rud = { .rctx = &rctx, .n_visited = 0 };
        kx_param_store_foreach(_report_ctrl_cb, &rud);

        if (rctx.sent > 0 || rctx.errors > 0) {
            ESP_LOGI(TAG,
                     "report tick=%" PRId64 "s sent=%d errors=%d heap=%" PRIu32,
                     tick, rctx.sent, rctx.errors, kx_system_heap_free());
        }
    }

    vTaskDelete(NULL);
}

// =============================================================
// Helpers de conteo y búsqueda
// =============================================================
static void _count_readable(int control_id, const kx_param_t *param, void *ud)
{
    _count_ctx_t *c = (_count_ctx_t *)ud;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE ||
        fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS) c->count++;
}

static void _find_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)ud;
    if (ctx->found_ctrl_id < 0 && param->param_id == ctx->target_param_id)
        ctx->found_ctrl_id = ctrl_id;
}

// =============================================================
// Poll de un único param_id (batch de demandas individuales)
// =============================================================
static bool _poll_single_param(int param_id, char *err_out, size_t err_len,
                                bool *out_dropped)
{
    if (out_dropped) *out_dropped = false;

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

    xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);
    xSemaphoreGive(s_foreach_mutex);

    if (value == -FLT_MAX) {
        bool enq = _enqueue(PUB_KIND_ERROR, fctx.found_ctrl_id, param->param_id,
                            0.0f, (uint16_t)param->reg, "modbus_timeout");
        if (out_dropped) *out_dropped = !enq;
        snprintf(err_out, err_len, "modbus_timeout reg=0x%04x", param->reg);
        return false;
    }

    bool enq = _enqueue(PUB_KIND_STATUS, fctx.found_ctrl_id, param->param_id,
                        value, 0, NULL);
    if (out_dropped) *out_dropped = !enq;

    kx_param_t *mp = kx_param_store_get_param_mutable(fctx.found_ctrl_id, param_id);
    if (mp) {
        mp->ts_last_read         = (int64_t)(esp_timer_get_time() / 1000ULL);
        mp->last_published_value = value;
    }
    return true;
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
            case PUB_KIND_STATUS: kx_param_pub_status(r.control_id, r.param_id, r.value); break;
            case PUB_KIND_REPORT: kx_param_pub_report(r.control_id, r.param_id, r.value); break;
            case PUB_KIND_ERROR:  kx_param_pub_error (r.control_id, r.param_id, r.error_msg, r.reg); break;
            }
        }
    }
}

// =============================================================
// _drain_demand_queue
// =============================================================
typedef struct {
    int  param_id;
    bool ok;
    bool dropped;
    char err_msg[48];
} _batch_result_t;

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

        if ((now_ms - d.enqueued_ms) > (int64_t)(KX_DEMAND_TIMEOUT_S * 1000)) {
            expired++;
            continue;
        }

        bool found = false;
        for (int j = 0; j < count; j++) {
            if (snapshot[j].param_id == d.param_id) {
                if (d.enqueued_ms > snapshot[j].enqueued_ms)
                    snapshot[j].enqueued_ms = d.enqueued_ms;
                dupes++;
                found = true;
                break;
            }
        }
        if (!found) snapshot[count++] = d;
    }

    int leftovers = 0;
    kx_poll_demand_t tmp;
    while (xQueueReceive(s_demand_queue, &tmp, 0) == pdTRUE) leftovers++;
    if (leftovers > 0)
        ESP_LOGW(TAG, "_drain: %d leftover demands discarded", leftovers);

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

    while (s_running) {

        EventBits_t bits = xEventGroupWaitBits(
            s_poll_eg, DEMAND_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & DEMAND_BIT)) continue;

        // Fase de recopilación de ráfaga
        {
            int64_t t0   = (int64_t)(esp_timer_get_time() / 1000ULL);
            int     prev = -1;
            int   stable = 0;
            ESP_LOGI(TAG, "collecting burst...");

            while (1) {
                vTaskDelay(pdMS_TO_TICKS(BURST_POLL_MS));
                int     cur = (int)uxQueueMessagesWaiting(s_demand_queue);
                int64_t ela = (int64_t)(esp_timer_get_time() / 1000ULL) - t0;

                if (cur == prev) {
                    stable += BURST_POLL_MS;
                    if (stable >= BURST_STABLE_MS) {
                        ESP_LOGI(TAG, "burst stable: %d demands in %" PRId64 "ms",
                                 cur, ela);
                        break;
                    }
                } else { stable = 0; }
                prev = cur;

                if (ela >= BURST_COLLECT_MAX_MS) {
                    ESP_LOGW(TAG, "burst timeout: %d demands", cur);
                    break;
                }
            }
        }

        int raw_count = (int)uxQueueMessagesWaiting(s_demand_queue);
        if (raw_count == 0) {
            xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
            continue;
        }

        kx_poll_demand_t *snapshot = malloc((size_t)raw_count *
                                            sizeof(kx_poll_demand_t));
        if (!snapshot) {
            ESP_LOGE(TAG, "OOM snapshot (%d demands) — flushing", raw_count);
            kx_poll_demand_t tmp2;
            while (xQueueReceive(s_demand_queue, &tmp2, 0) == pdTRUE) {}
            xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
            continue;
        }

        int expired = 0, dupes = 0;
        int valid_count = _drain_demand_queue(snapshot, raw_count,
                                              &expired, &dupes);
        xEventGroupClearBits(s_poll_eg, DEMAND_BIT);

        ESP_LOGI(TAG, "snapshot: raw=%d valid=%d expired=%d dupes=%d",
                 raw_count, valid_count, expired, dupes);

        if (valid_count == 0) { free(snapshot); continue; }

        bool has_full_cycle = false;
        for (int i = 0; i < valid_count; i++) {
            if (snapshot[i].param_id == 0) { has_full_cycle = true; break; }
        }

        if (has_full_cycle) {
            ESP_LOGI(TAG, "full poll cycle (demand_active)");
            if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
                ESP_LOGW(TAG, "full cycle skipped: no mqtt/store");
                free(snapshot);
                continue;
            }
            xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);

            _count_ctx_t cc = { .count = 0 };
            kx_param_store_foreach(_count_readable, &cc);
            _poll_ctx_t ctx = { .total = cc.count, .demand_active = true };

            KX_LOG_CYCLE_START(TAG, 0, kx_param_store_count(), ctx.total);

            _poll_ctrl_foreach_ud_t ud = { .ctx = &ctx, .n_visited = 0 };
            kx_param_store_foreach(_poll_ctrl_cb, &ud);

            KX_LOG_CYCLE_END(TAG, ctx.ok, ctx.errors, ctx.skipped, ctx.unchanged);
            free(snapshot);
            continue;
        }

        // Batch de demandas individuales
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
        int batch_dropped = 0;
        int pub_hwm       = 0;
        int demand_hwm    = 0;
        int write_hwm     = 0;

        for (int i = 0; i < valid_count && s_running; i++) {
            int param_id = snapshot[i].param_id;
            results[i].param_id = param_id;

            int pq = (int)uxQueueMessagesWaiting(s_pub_queue);
            int dq = (int)uxQueueMessagesWaiting(s_demand_queue);
            int wq = (int)uxQueueMessagesWaiting(s_write_queue);
            if (pq > pub_hwm)    pub_hwm    = pq;
            if (dq > demand_hwm) demand_hwm = dq;
            if (wq > write_hwm)  write_hwm  = wq;

            if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
                snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                         "mqtt/store not ready");
                results[i].ok = false;
                batch_errors++;
                if (batch_mode)
                    _print_batch_progress(i + 1, valid_count, batch_ok, batch_errors);
                continue;
            }

            xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                                pdFALSE, pdTRUE, portMAX_DELAY);

            bool ok      = false;
            bool dropped = false;

            if (s_running && kx_param_store_is_ready()) {
                ok = _poll_single_param(param_id,
                                        results[i].err_msg,
                                        sizeof(results[i].err_msg),
                                        &dropped);
                if (ok && dropped) {
                    results[i].dropped = true;
                    batch_dropped++;
                    snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                             "pub_queue drop");
                }
            } else {
                snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                         "task stopping");
            }

            results[i].ok = ok;
            if (ok)  batch_ok++;
            else     batch_errors++;

            if (batch_mode)
                _print_batch_progress(i + 1, valid_count, batch_ok, batch_errors);

            if (i + 1 < valid_count)
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
        }

        if (batch_mode) { printf("\n"); fflush(stdout); }

        printf("┌──────────────────────────────────────────────────────────────┐\n");
        printf("│                    BATCH POLL  RESUMEN                        │\n");
        printf("├──────────────────────────────────────────────────────────────┤\n");
        printf("│  Recopilados : %-5d  (dupes descartados: %-5d)             │\n",
               valid_count, dupes);
        printf("│  Expirados   : %-5d                                          │\n", expired);
        printf("│  OK (leídos) : %-5d                                          │\n", batch_ok);
        printf("│  Errores     : %-5d                                          │\n", batch_errors);
        printf("│  Drops MQTT  : %-5d                                          │\n", batch_dropped);
        printf("│  Heap libre  : %-8lu bytes                                │\n",
               (unsigned long)kx_system_heap_free());
        printf("│  pub_queue   : hwm=%-3d / %-5d slots                        │\n",
               pub_hwm, PUB_QUEUE_SIZE);
        printf("│  demand_queue: hwm=%-3d / %-5d slots                        │\n",
               demand_hwm, DEMAND_QUEUE_SIZE);
        printf("│  write_queue : hwm=%-3d / %-5d slots                        │\n",
               write_hwm, WRITE_QUEUE_SIZE);

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
            if (batch_errors > BATCH_REPORT_MAX_ERRORS)
                printf("│    ... y %d más\n", batch_errors - BATCH_REPORT_MAX_ERRORS);
        }

        if (batch_dropped > 0) {
            printf("├──────────────────────────────────────────────────────────────┤\n");
            printf("│  ⚠ Drops pub_queue:                                          │\n│    ");
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

    s_write_queue = xQueueCreate(WRITE_QUEUE_SIZE, sizeof(kx_write_cmd_t));
    if (!s_write_queue) { ESP_LOGE(TAG, "write_queue alloc failed"); return ESP_FAIL; }

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) { ESP_LOGE(TAG, "EventGroup alloc failed"); return ESP_FAIL; }

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) { ESP_LOGE(TAG, "mutex alloc failed"); return ESP_FAIL; }

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);

    esp_err_t err = _uart_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "UART init: %s", esp_err_to_name(err)); return err; }

    s_running = true;
    BaseType_t ret;

    ret = xTaskCreate(_publisher_task, "kx_publisher", 4096, NULL,
                      KX_TASK_PRIO_TELEMETRY - 1, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "publisher task failed"); return ESP_FAIL; }

    ret = xTaskCreate(_writer_task, "kx_writer", 4096, NULL,
                      KX_TASK_PRIO_TELEMETRY + 2, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "writer task failed"); return ESP_FAIL; }

    ret = xTaskCreate(_report_task, "kx_report", 4096, NULL,
                      KX_TASK_PRIO_TELEMETRY, NULL);
    if (ret != pdPASS) { ESP_LOGE(TAG, "report task failed"); return ESP_FAIL; }

    ret = xTaskCreate(_modbus_task, "kx_modbus", 8192, NULL,
                      KX_TASK_PRIO_TELEMETRY + 1, &s_task);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ESP_LOGI(TAG, "started — writer prio=%d poll prio=%d report prio=%d pub prio=%d",
             KX_TASK_PRIO_TELEMETRY + 2,
             KX_TASK_PRIO_TELEMETRY + 1,
             KX_TASK_PRIO_TELEMETRY,
             KX_TASK_PRIO_TELEMETRY - 1);

    return ESP_OK;
}

void kx_modbus_master_stop(void) { s_running = false; }
bool kx_modbus_master_is_running(void) { return s_running; }

// =============================================================
// kx_modbus_read_one (API síncrona externa)
// =============================================================
esp_err_t kx_modbus_read_one(int control_id, int param_id)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) return ESP_ERR_NOT_FOUND;
    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return ESP_ERR_INVALID_STATE;

    xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg,
                                  (uint8_t)param->function_read, param);
    xSemaphoreGive(s_foreach_mutex);

    if (value == -FLT_MAX) {
        kx_param_pub_error(control_id, param->param_id,
                           "modbus_timeout", (uint16_t)param->reg);
        return ESP_FAIL;
    }
    kx_param_pub_status(control_id, param->param_id, value);
    return ESP_OK;
}

// =============================================================
// kx_modbus_write_one — encola
// =============================================================
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value)
{
    return kx_modbus_enqueue_write(control_id, param_id, value, 0.0);
}