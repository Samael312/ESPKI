#include "kx_modbus_master.h"
#include "kx_modbus_uart.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "kx_telemetry.h"
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
// Cola de publicacion
// =============================================================
#define PUB_QUEUE_SIZE               500
#define PUB_QUEUE_BACKPRESSURE_HWM   350
#define PUB_BACKPRESSURE_WAIT_MS      20
#define PUB_BACKPRESSURE_TIMEOUT_MS 2000

// Firma comun para status/report: (control_id, param_id, value)
typedef void (*kx_pub_fn_t)(int control_id, int param_id, float value);
// Firma para error: (control_id, param_id, msg, reg)
typedef void (*kx_pub_err_fn_t)(int control_id, int param_id,
                                 const char *msg, uint16_t reg);

typedef struct {
    kx_pub_fn_t     pub_fn;      // != NULL -> status o report
    kx_pub_err_fn_t pub_err_fn;  // != NULL -> error
    int             control_id;
    int             param_id;
    uint16_t        reg;
    float           value;
    char            error_msg[32];
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
#define DEMAND_QUEUE_SIZE    1500
#define DEMAND_WARN_HWM       700

#define BURST_COLLECT_MAX_MS  3000
#define BURST_STABLE_MS        300
#define BURST_POLL_MS           50

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
#define REPORT_LOG_MAX_PARAMS  256

static volatile int64_t s_report_tick_s = -1;

// =============================================================
// Umbral de cambio (status)
// =============================================================
#define KX_STATUS_DELTA_ABS   0.5f
#define KX_STATUS_DELTA_REL   0.01f

// =============================================================
// Sincronizacion Modbus
// =============================================================
#define POLL_ALLOWED_BIT   BIT0
#define DEMAND_BIT         BIT1
#define BATCH_ACTIVE_BIT   BIT2

static EventGroupHandle_t  s_poll_eg       = NULL;
static SemaphoreHandle_t   s_foreach_mutex = NULL;
static volatile bool       s_running       = false;
static TaskHandle_t        s_task          = NULL;

// =============================================================
// _enqueue
// =============================================================
static bool _enqueue(kx_pub_fn_t pub_fn, kx_pub_err_fn_t pub_err_fn,
                     int ctrl_id, int param_id,
                     float value, uint16_t reg, const char *errmsg)
{
    kx_pub_result_t r = {
        .pub_fn     = pub_fn,
        .pub_err_fn = pub_err_fn,
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
// API publica -- encolar escritura
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
        ESP_LOGE(TAG, "write_queue FULL -- dropping write ctrl=%d param=%d",
                 control_id, param_id);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "write enqueued: ctrl=%d param=%d value=%.3f ts=%.3f",
             control_id, param_id, value, ts);
    return ESP_OK;
}

// =============================================================
// API publica -- demanda de polling
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
// _publish_all_params_for_reg
//
// Dado el raw Modbus (sin transformar), publica el valor correcto
// a todos los params del control que comparten (reg, fc_read),
// aplicando el offset/addition individual de cada uno.
//
// tick_s == -1  -> modo demand: publicar todos los params del registro
// tick_s >= 0   -> modo report: solo publicar los params cuyo
//                  sampling divide exactamente al tick actual
//                  (tick_s % p->sampling == 0)
// =============================================================
static int _publish_all_params_for_reg(int           control_id,
                                        uint16_t      reg,
                                        uint8_t       fc_read,
                                        uint16_t      raw,
                                        int64_t       ts_ms,
                                        kx_pub_fn_t   pub_fn,
                                        int64_t       tick_s)
{
    int published = 0;

    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl) return 0;

    for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS; pi++) {
        kx_param_node_t *pn = ctrl->params.buckets[pi];
        while (pn) {
            kx_param_t *p = &pn->param;

            if ((uint16_t)p->reg          == reg     &&
                (uint8_t) p->function_read == fc_read &&
                p->view != 0) {

                if (tick_s >= 0) {
                    if (p->sampling <= 0 ||
                        (tick_s % (int64_t)p->sampling) != 0) {
                        pn = pn->next;
                        continue;
                    }
                }

                float value = (float)(int16_t)raw;
                if (p->offset != 0.0f && p->offset != 1.0f)
                    value *= p->offset;
                value += p->addition;
                if (value < p->minvalue) value = p->minvalue;
                if (value > p->maxvalue) value = p->maxvalue;

                p->ts_last_read         = ts_ms;
                p->last_published_value = value;

                _enqueue(pub_fn, NULL, control_id, p->param_id, value, 0, NULL);
                published++;
            }
            pn = pn->next;
        }
    }
    return published;
}


// =============================================================
// _dispatch_packet
//
// tick_s == -1  -> modo demand
// tick_s >= 0   -> modo report, se propaga a _publish_all_params_for_reg
// =============================================================
static int _dispatch_packet(const kx_packet_t *pkt,
                             kx_pub_fn_t   pub_fn,
                             int *out_errors,
                             int64_t tick_s)
{
    if (!pkt || pkt->num_slots == 0) return 0;

    int ok_count  = 0;
    int err_count = 0;

    // ── Caso 1: packet individual (num_regs == 1) ─────────────
    if (pkt->num_regs == 1) {
        const kx_pkt_slot_t *slot = &pkt->slots[0];
        if (slot->is_gap) goto dispatch_done;

        const kx_param_t *param =
            kx_param_store_get_param(slot->control_id, slot->param_id);
        if (!param) {
            ESP_LOGW(TAG, "dispatch individual: param not found ctrl=%d p=%d",
                     slot->control_id, slot->param_id);
            err_count++;
            goto dispatch_done;
        }

        uint16_t raw = 0;
        float value = kx_modbus_read_reg(pkt->slave_addr, pkt->start_reg,
                                         pkt->fc, param, &raw);
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (value == -FLT_MAX) {
            _enqueue(NULL, kx_param_pub_error, slot->control_id, slot->param_id,
                     0.0f, pkt->start_reg, "modbus_timeout");
            err_count++;
        } else {
            kx_param_store_reg_upsert_read(
                slot->control_id, pkt->start_reg, pkt->fc,
                (uint8_t)param->function_write, value, ts_ms);

            int n = _publish_all_params_for_reg(
                        slot->control_id, pkt->start_reg,
                        pkt->fc, raw, ts_ms, pub_fn, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
        goto dispatch_done;
    }

    // ── Caso 2: packet multi-registro ─────────────────────────
    {
        uint8_t resp[KX_PKT_MAX_REGS_PER_PKT * 2 + 8];
        int rx = kx_modbus_read_regs_multi(pkt->slave_addr, pkt->start_reg,
                                       pkt->num_regs, pkt->fc,
                                       resp, sizeof(resp));
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        // Fallback individual si el multi falla
        if (rx < 0) {
            ESP_LOGW(TAG,
                     "multi FAILED (slave=%d fc=0x%02x reg=0x%04x num=%d) "
                     "-- fallback individual",
                     pkt->slave_addr, pkt->fc,
                     pkt->start_reg, pkt->num_regs);
            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *slot = &pkt->slots[s];
                if (slot->is_gap || slot->param_id < 0) continue;
                const kx_param_t *param =
                    kx_param_store_get_param(slot->control_id, slot->param_id);
                if (!param) { err_count++; continue; }
                uint16_t raw = 0;
                float value = kx_modbus_read_reg(pkt->slave_addr, slot->reg,
                                                 pkt->fc, param, &raw);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
                if (value == -FLT_MAX) {
                    _enqueue(NULL, kx_param_pub_error, slot->control_id, slot->param_id,
                             0.0f, slot->reg, "modbus_timeout");
                    err_count++;
                } else {
                    kx_param_store_reg_upsert_read(
                        slot->control_id, slot->reg, pkt->fc,
                        (uint8_t)param->function_write, value, ts_ms);
                    int n = _publish_all_params_for_reg(
                                slot->control_id, slot->reg,
                                pkt->fc, raw, ts_ms, pub_fn, tick_s);
                    ok_count += (n > 0) ? n : 1;
                }
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
            }
            goto dispatch_done;
        }

        // Verificar byte_count
        bool is_coil = (pkt->fc == MB_FC_READ_COILS ||
                        pkt->fc == MB_FC_READ_DISCRETE);
        int  expected_bytes = is_coil ? (pkt->num_regs + 7) / 8
                                      : pkt->num_regs * 2;

        if (rx < 3 || resp[2] != (uint8_t)expected_bytes) {
            ESP_LOGW(TAG,
                     "multi bad byte_count=%d expected=%d rx=%d "
                     "(slave=%d reg=0x%04x num=%d) -- fallback",
                     (rx >= 3) ? resp[2] : -1, expected_bytes, rx,
                     pkt->slave_addr, pkt->start_reg, pkt->num_regs);
            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *slot = &pkt->slots[s];
                if (slot->is_gap || slot->param_id < 0) continue;
                const kx_param_t *param =
                    kx_param_store_get_param(slot->control_id, slot->param_id);
                if (!param) { err_count++; continue; }
                uint16_t raw = 0;
                float value = kx_modbus_read_reg(pkt->slave_addr, slot->reg,
                                                 pkt->fc, param, &raw);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
                if (value == -FLT_MAX) {
                    _enqueue(NULL, kx_param_pub_error, slot->control_id, slot->param_id,
                             0.0f, slot->reg, "modbus_timeout");
                    err_count++;
                } else {
                    kx_param_store_reg_upsert_read(
                        slot->control_id, slot->reg, pkt->fc,
                        (uint8_t)param->function_write, value, ts_ms);
                    int n = _publish_all_params_for_reg(
                                slot->control_id, slot->reg,
                                pkt->fc, raw, ts_ms, pub_fn, tick_s);
                    ok_count += (n > 0) ? n : 1;
                }
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
            }
            goto dispatch_done;
        }

        // Decodificar slots
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
                ESP_LOGW(TAG, "multi slot[%d]: param not found p=%d",
                         s, slot->param_id);
                err_count++;
                continue;
            }

            float value_first = (float)(int16_t)raw;
            if (param->offset != 0.0f && param->offset != 1.0f)
                value_first *= param->offset;
            value_first += param->addition;
            if (value_first < param->minvalue) value_first = param->minvalue;
            if (value_first > param->maxvalue) value_first = param->maxvalue;

            kx_param_store_reg_upsert_read(
                slot->control_id, slot->reg, pkt->fc,
                (uint8_t)param->function_write, value_first, ts_ms);

            int n = _publish_all_params_for_reg(
                        slot->control_id, slot->reg,
                        pkt->fc, raw, ts_ms, pub_fn, tick_s);
            ok_count += (n > 0) ? n : 1;
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
             "slave=%d value=%.3f -> raw=%d (0x%04X)",
             control_id, param_id, param->reg, fc_write,
             ctrl->slave_addr, value, (int)(uint16_t)raw, (uint16_t)raw);

    uint8_t resp[16];
    int rx = -1;

    if (fc_write == MB_FC_WRITE_MULTIPLE_REGS) {
        // FC 0x10: [slave][0x10][reg_hi][reg_lo][0x00][0x01][0x02][data_hi][data_lo]
        // Respuesta esperada: 6 bytes [slave][0x10][reg_hi][reg_lo][0x00][0x01]
        uint8_t frame[9] = {
            (uint8_t)ctrl->slave_addr, MB_FC_WRITE_MULTIPLE_REGS,
            (uint8_t)((uint16_t)param->reg >> 8),
            (uint8_t)((uint16_t)param->reg & 0xFF),
            0x00, 0x01,   // quantity: 1 registro
            0x02,         // byte count: 2 bytes
            (uint8_t)((uint16_t)raw >> 8),
            (uint8_t)((uint16_t)raw & 0xFF),
        };
        for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
            rx = kx_modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
            if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
        if (rx < 0) {
            ESP_LOGW(TAG, "write FC10: no response after %d retries ctrl=%d param=%d",
                     MODBUS_RETRY_COUNT, control_id, param_id);
            return ESP_FAIL;
        }
        // Respuesta válida: 6 bytes, echo de slave+FC+reg+quantity
        if (rx < 6 || resp[0] != frame[0] || resp[1] != frame[1] ||
            resp[2] != frame[2] || resp[3] != frame[3]) {
            ESP_LOGW(TAG, "write FC10: unexpected response rx=%d", rx);
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "write FC10: OK raw=%d (0x%04X)", (int)(uint16_t)raw, (uint16_t)raw);
    } else {
        // FC 0x05 / FC 0x06: trama de 6 bytes, respuesta echo de 6 bytes
        uint8_t frame[6] = {
            (uint8_t)ctrl->slave_addr, fc_write,
            (uint8_t)((uint16_t)param->reg >> 8),
            (uint8_t)((uint16_t)param->reg & 0xFF),
            (uint8_t)((uint16_t)raw >> 8),
            (uint8_t)((uint16_t)raw & 0xFF),
        };
        for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
            rx = kx_modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
            if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
        if (rx < 0) {
            ESP_LOGW(TAG, "write: no response after %d retries ctrl=%d param=%d",
                     MODBUS_RETRY_COUNT, control_id, param_id);
            return ESP_FAIL;
        }
        if (rx < 6 || resp[0] != frame[0] || resp[1] != frame[1] ||
            resp[2] != frame[2] || resp[3] != frame[3]) {
            ESP_LOGW(TAG, "write: unexpected response rx=%d", rx);
            return ESP_FAIL;
        }
        uint16_t echo = ((uint16_t)resp[4] << 8) | resp[5];
        ESP_LOGI(TAG, "write: OK raw_sent=%d raw_echo=%d",
                 (int)(uint16_t)raw, (int)echo);
    }
    return ESP_OK;
}

// =============================================================
// Tarea writer -- ALTA PRIORIDAD
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
            _enqueue(kx_param_pub_status, NULL, cmd.control_id, cmd.param_id,
                     cmd.value, 0, NULL);
            ESP_LOGI(TAG, "writer: OK ctrl=%d param=%d value=%.3f",
                     cmd.control_id, cmd.param_id, cmd.value);
        } else {
            _enqueue(NULL, kx_param_pub_error, cmd.control_id, cmd.param_id,
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
    int     param_ids[REPORT_LOG_MAX_PARAMS];
    int     n_param_ids;
} _report_ctx_t;

typedef struct { int count; } _count_ctx_t;
typedef struct { int target_param_id; int found_ctrl_id; } _find_ctrl_ctx_t;

#define MAX_CTRL_VISITED  KX_PARAM_MAX_CONTROLS

// =============================================================
// _dispatch_control_packets
// =============================================================
static void _dispatch_control_packets(kx_packet_list_t *list,
                                      kx_pub_fn_t   pub_fn,
                                      int *out_ok,
                                      int *out_errors,
                                      int64_t tick_s)
{
    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];

        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
        int pkt_errors = 0;
        int pkt_ok = _dispatch_packet(pkt, pub_fn, &pkt_errors, tick_s);
        xSemaphoreGive(s_foreach_mutex);

        if (out_ok)     *out_ok     += pkt_ok;
        if (out_errors) *out_errors += pkt_errors;

        if (i + 1 < list->count)
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
    }
}

// =============================================================
// _poll_control_packetized  (modo demand -- tick_s = -1)
// =============================================================
static void _poll_control_packetized(int control_id, _poll_ctx_t *ctx)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    kx_packet_list_t *list = kx_pkt_build(control_id, true, NULL, 0, 0, now_ms);
    if (!list) {
        ESP_LOGD(TAG, "packetizer: ctrl=%d no packets (full)", control_id);
        return;
    }

    if (ctx->total == 0)
        ctx->total += kx_pkt_real_param_count(list);

#if CONFIG_LOG_DEFAULT_LEVEL >= 4
    kx_pkt_dump(list, TAG);
#endif

    _dispatch_control_packets(list, kx_param_pub_status, &ctx->ok, &ctx->errors, -1);
    ctx->done = ctx->ok + ctx->errors;

    kx_pkt_free(list);
}

// =============================================================
// _poll_ctrl_cb
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
// _poll_batch_packetized  (modo demand -- tick_s = -1)
// =============================================================
#define MAX_CTRLS_IN_BATCH  KX_PARAM_MAX_CONTROLS
#define MAX_PARAMS_IN_BATCH 1500

static void _find_ctrl_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)ud;
    if (ctx->found_ctrl_id < 0 && param->param_id == ctx->target_param_id)
        ctx->found_ctrl_id = ctrl_id;
}

typedef struct {
    int  param_id;
    bool ok;
    bool dropped;
    char err_msg[48];
} _batch_result_t;

typedef struct {
    int ctrl_id;
    int param_ids[MAX_PARAMS_IN_BATCH];
    int n_params;
} _ctrl_group_t;

static void _poll_batch_packetized(const kx_poll_demand_t *snapshot,
                                   int valid_count,
                                   _batch_result_t *results,
                                   int *out_ok,
                                   int *out_errors,
                                   int *out_dropped,
                                   int *out_packaged)
{
    _ctrl_group_t *groups = malloc(MAX_CTRLS_IN_BATCH * sizeof(_ctrl_group_t));
    if (!groups) {
        ESP_LOGE(TAG, "batch_packetized: OOM groups");
        for (int i = 0; i < valid_count; i++) {
            snprintf(results[i].err_msg, sizeof(results[i].err_msg), "OOM");
            (*out_errors)++;
        }
        return;
    }
    int n_groups = 0;

    for (int i = 0; i < valid_count; i++) {
        int param_id = snapshot[i].param_id;

        _find_ctrl_ctx_t fctx = { .target_param_id = param_id, .found_ctrl_id = -1 };
        kx_param_store_foreach(_find_ctrl_cb, &fctx);

        if (fctx.found_ctrl_id < 0) {
            snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                     "not found in any control");
            results[i].ok = false;
            (*out_errors)++;
            continue;
        }

        int g = -1;
        for (int j = 0; j < n_groups; j++) {
            if (groups[j].ctrl_id == fctx.found_ctrl_id) { g = j; break; }
        }
        if (g < 0) {
            if (n_groups >= MAX_CTRLS_IN_BATCH) {
                ESP_LOGW(TAG, "batch: too many controls, skipping param=%d", param_id);
                snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                         "too many controls");
                results[i].ok = false;
                (*out_errors)++;
                continue;
            }
            g = n_groups++;
            groups[g].ctrl_id  = fctx.found_ctrl_id;
            groups[g].n_params = 0;
        }
        if (groups[g].n_params < MAX_PARAMS_IN_BATCH)
            groups[g].param_ids[groups[g].n_params++] = param_id;
    }

    for (int g = 0; g < n_groups && s_running; g++) {
        _ctrl_group_t *grp = &groups[g];

        if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
            for (int j = 0; j < grp->n_params; j++) {
                for (int i = 0; i < valid_count; i++) {
                    if (snapshot[i].param_id == grp->param_ids[j]) {
                        snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                                 "mqtt/store not ready");
                        results[i].ok = false;
                        (*out_errors)++;
                        break;
                    }
                }
            }
            continue;
        }

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        kx_packet_list_t *list = kx_pkt_build(grp->ctrl_id, true,
                                               grp->param_ids, grp->n_params,
                                               0, now_ms);
        if (!list) {
            for (int j = 0; j < grp->n_params; j++) {
                for (int i = 0; i < valid_count; i++) {
                    if (snapshot[i].param_id == grp->param_ids[j]) {
                        snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                                 "not readable/visible");
                        results[i].ok = false;
                        (*out_errors)++;
                        break;
                    }
                }
            }
            continue;
        }

#if CONFIG_LOG_DEFAULT_LEVEL >= 4
        kx_pkt_dump(list, TAG);
#endif

        if (out_packaged) *out_packaged += kx_pkt_real_param_count(list);

        int ctrl_ok = 0, ctrl_errors = 0;
        _dispatch_control_packets(list, kx_param_pub_status,
                                  &ctrl_ok, &ctrl_errors, -1);
        *out_ok     += ctrl_ok;
        *out_errors += ctrl_errors;

        int64_t ts_dispatch_end = (int64_t)(esp_timer_get_time() / 1000ULL);

        for (int i = 0; i < valid_count; i++) {
            int pid = snapshot[i].param_id;
            bool in_group = false;
            for (int j = 0; j < grp->n_params; j++) {
                if (grp->param_ids[j] == pid) { in_group = true; break; }
            }
            if (!in_group || results[i].ok) continue;

            _find_ctrl_ctx_t fctx2 = { .target_param_id = pid, .found_ctrl_id = -1 };
            kx_param_store_foreach(_find_ctrl_cb, &fctx2);

            if (fctx2.found_ctrl_id >= 0) {
                const kx_param_t *p =
                    kx_param_store_get_param(fctx2.found_ctrl_id, pid);
                if (p && p->ts_last_read > 0 &&
                    (ts_dispatch_end - p->ts_last_read) < 5000) {
                    results[i].ok = true;
                } else if (results[i].err_msg[0] == '\0') {
                    snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                             "mb_no_response");
                }
            } else {
                if (results[i].err_msg[0] == '\0')
                    snprintf(results[i].err_msg, sizeof(results[i].err_msg),
                             "not_in_store");
            }
        }

        kx_pkt_free(list);

        // ── Fallback para params no resueltos del grupo ───────
        // Tras el dispatch packetizado, algunos params pueden quedar
        // sin resultado por ser write-only, view=0 o fc_read>0 no
        // agrupable. Los tratamos aquí individualmente.
        for (int j = 0; j < grp->n_params; j++) {
            int pid = grp->param_ids[j];

            // Buscar si este pid ya quedó resuelto
            bool already_ok = false;
            int  result_idx = -1;
            for (int i = 0; i < valid_count; i++) {
                if (snapshot[i].param_id == pid) {
                    result_idx = i;
                    already_ok = results[i].ok;
                    break;
                }
            }
            if (already_ok || result_idx < 0) continue;

            _find_ctrl_ctx_t fctx3 = { .target_param_id = pid,
                                        .found_ctrl_id   = -1 };
            kx_param_store_foreach(_find_ctrl_cb, &fctx3);
            if (fctx3.found_ctrl_id < 0) continue;

            const kx_param_t *p =
                kx_param_store_get_param(fctx3.found_ctrl_id, pid);
            if (!p) continue;

            // view=0 → skip silencioso
            if (p->view == 0) {
                results[result_idx].ok = true;  // no es un error real
                continue;
            }

            // write-only (fc_read == 0)
            if (p->function_read == 0) {
                if (p->last_published_value != FLT_MAX) {
                    // Tiene valor previo → publicar status con ese valor
                    _enqueue(kx_param_pub_status, NULL, fctx3.found_ctrl_id, pid,
                             p->last_published_value, 0, NULL);
                    results[result_idx].ok = true;
                } else {
                    // Sin valor conocido → error
                    _enqueue(NULL, kx_param_pub_error, fctx3.found_ctrl_id, pid,
                             0.0f, (uint16_t)p->reg, "write_only_no_value");
                    snprintf(results[result_idx].err_msg,
                             sizeof(results[result_idx].err_msg),
                             "write_only_no_value");
                }
                continue;
            }

            // fc_read > 0 → lectura individual bajo mutex
            const kx_control_t *ctrl_info =
                kx_param_store_get_ctrl(fctx3.found_ctrl_id);
            if (!ctrl_info || ctrl_info->slave_addr == 0) continue;

            xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
            uint16_t raw = 0;
            float val = kx_modbus_read_reg(
                (uint8_t)ctrl_info->slave_addr,
                (uint16_t)p->reg,
                (uint8_t)p->function_read,
                p, &raw);
            int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

            if (val == -FLT_MAX) {
                _enqueue(NULL, kx_param_pub_error, fctx3.found_ctrl_id, pid,
                         0.0f, (uint16_t)p->reg, "modbus_timeout");
                snprintf(results[result_idx].err_msg,
                         sizeof(results[result_idx].err_msg),
                         "modbus_timeout");
            } else {
                kx_param_store_reg_upsert_read(
                    fctx3.found_ctrl_id, (uint16_t)p->reg,
                    (uint8_t)p->function_read,
                    (uint8_t)p->function_write, val, ts_ms);
                _publish_all_params_for_reg(
                    fctx3.found_ctrl_id, (uint16_t)p->reg,
                    (uint8_t)p->function_read,
                    raw, ts_ms, kx_param_pub_status, -1);
                results[result_idx].ok = true;
            }
            xSemaphoreGive(s_foreach_mutex);
        }
        // ── Fin fallback ──────────────────────────────────────
    }

    *out_ok = 0; *out_errors = 0;
    for (int i = 0; i < valid_count; i++) {
        if (results[i].ok)                       (*out_ok)++;
        else if (results[i].err_msg[0] != '\0')  (*out_errors)++;
    }

    free(groups);
}

// =============================================================
// _report_control_packetized  (modo report -- tick_s = rctx->tick_s)
// =============================================================
static void _report_control_packetized(int control_id, _report_ctx_t *rctx)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

    kx_packet_list_t *list = kx_pkt_build(control_id, false, NULL, 0,
                                           rctx->tick_s, now_ms);
    if (!list) return;

    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];
        for (int s = 0; s < pkt->num_slots; s++) {
            if (!pkt->slots[s].is_gap &&
                rctx->n_param_ids < REPORT_LOG_MAX_PARAMS) {
                rctx->param_ids[rctx->n_param_ids++] = pkt->slots[s].param_id;
            }
        }
    }

    _dispatch_control_packets(list, kx_param_pub_report,
                              &rctx->sent, &rctx->errors, rctx->tick_s);
    kx_pkt_free(list);
}

// =============================================================
// _report_ctrl_cb
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
// Tarea de reports -- tick de 1 segundo
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

        _report_ctx_t rctx = {
            .tick_s      = tick,
            .sent        = 0,
            .errors      = 0,
            .n_param_ids = 0,
        };
        _report_ctrl_foreach_ud_t rud = { .rctx = &rctx, .n_visited = 0 };
        kx_param_store_foreach(_report_ctrl_cb, &rud);

        if (rctx.sent > 0 || rctx.errors > 0) {
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "╔══════════════════════════════════════════╗");
            ESP_LOGI(TAG, "║            REPORT  tick=%-6" PRId64 "           ║", tick);
            ESP_LOGI(TAG, "╠══════════════════════════════════════════╣");
            ESP_LOGI(TAG, "║  sent=%-4d  errors=%-4d  heap=%-8" PRIu32 " ║",
                     rctx.sent, rctx.errors, kx_system_heap_free());
            ESP_LOGI(TAG, "╠══════════════════════════════════════════╣");
            ESP_LOGI(TAG, "║  params (%d):", rctx.n_param_ids);
            for (int i = 0; i < rctx.n_param_ids; i += 6) {
                char row[128];
                int rpos = snprintf(row, sizeof(row), "║    ");
                for (int j = i; j < rctx.n_param_ids && j < i + 6; j++)
                    rpos += snprintf(row + rpos, sizeof(row) - rpos,
                                     "%-10d", rctx.param_ids[j]);
                ESP_LOGI(TAG, "%s", row);
            }
            ESP_LOGI(TAG, "╚══════════════════════════════════════════╝");
        }
    }

    vTaskDelete(NULL);
}

// =============================================================
// Helpers de conteo
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
                r.pub_err_fn(r.control_id, r.param_id, r.error_msg, r.reg);
        }
    }
}

// =============================================================
// _drain_demand_queue
// =============================================================
static int _drain_demand_queue(kx_poll_demand_t *snapshot, int capacity,
                                int *out_expired, int *out_dupes)
{
    int64_t now_ms  = (int64_t)(esp_timer_get_time() / 1000ULL);
    int     expired = 0, dupes = 0, count = 0;

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
    ESP_LOGI(TAG, "task started -- waiting for entities...");
    while (!kx_param_store_is_ready()) vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "entities ready (%d controls) -- ready for poll demands",
             kx_param_store_count());

    while (s_running) {
        xEventGroupSetBits(s_poll_eg, BATCH_ACTIVE_BIT);

        EventBits_t bits = xEventGroupWaitBits(
            s_poll_eg, DEMAND_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & DEMAND_BIT)) continue;

        // Fase de recopilacion de rafaga
        {
            int64_t t0 = (int64_t)(esp_timer_get_time() / 1000ULL);
            int prev = -1, stable = 0;
            ESP_LOGI(TAG, "collecting burst...");

            while (1) {
                vTaskDelay(pdMS_TO_TICKS(BURST_POLL_MS));
                int     cur = (int)uxQueueMessagesWaiting(s_demand_queue);
                int64_t ela = (int64_t)(esp_timer_get_time() / 1000ULL) - t0;

                if (cur == prev) {
                    stable += BURST_POLL_MS;
                    if (stable >= BURST_STABLE_MS) {
                        ESP_LOGI(TAG, "burst stable: %d demands in %" PRId64 "ms", cur, ela);
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

        kx_poll_demand_t *snapshot = malloc((size_t)raw_count * sizeof(kx_poll_demand_t));
        if (!snapshot) {
            ESP_LOGE(TAG, "OOM snapshot (%d demands) -- flushing", raw_count);
            kx_poll_demand_t tmp2;
            while (xQueueReceive(s_demand_queue, &tmp2, 0) == pdTRUE) {}
            xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
            continue;
        }

        int expired = 0, dupes = 0;
        int valid_count = _drain_demand_queue(snapshot, raw_count, &expired, &dupes);
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

        ESP_LOGI(TAG, "batch poll: %d demands (dupes=%d expired=%d) -- packetizing...",
                 valid_count, dupes, expired);

        _batch_result_t *results = calloc((size_t)valid_count, sizeof(_batch_result_t));
        if (!results) {
            ESP_LOGE(TAG, "OOM results array -- aborting batch");
            free(snapshot);
            continue;
        }
        for (int i = 0; i < valid_count; i++)
            results[i].param_id = snapshot[i].param_id;

        int batch_ok = 0, batch_errors = 0, batch_dropped = 0, batch_packaged = 0;

        _poll_batch_packetized(snapshot, valid_count, results,
                               &batch_ok, &batch_errors, &batch_dropped,
                               &batch_packaged);

        int pub_hwm    = (int)uxQueueMessagesWaiting(s_pub_queue);
        int demand_hwm = (int)uxQueueMessagesWaiting(s_demand_queue);
        int write_hwm  = (int)uxQueueMessagesWaiting(s_write_queue);
        int batch_no_fc = valid_count - batch_packaged - batch_errors;
        if (batch_no_fc < 0) batch_no_fc = 0;

        printf("┌──────────────────────────────────────────────────────────────┐\n");
        printf("│                    BATCH POLL  RESUMEN                        │\n");
        printf("├──────────────────────────────────────────────────────────────┤\n");
        printf("│  Demandados  : %-5d  dupes=%-5d  expirados=%-5d           │\n",
               valid_count, dupes, expired);
        printf("│  Empaquetados: %-5d  sin_fc/view=%-5d                       │\n",
               batch_packaged, batch_no_fc);
        printf("│  Publicados  : %-5d  errores_mb=%-5d  drops=%-5d          │\n",
               batch_ok, batch_errors, batch_dropped);
        printf("├──────────────────────────────────────────────────────────────┤\n");
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
            for (int i = 0; i < valid_count && shown < 200; i++) {
                if (results[i].ok) continue;
                printf("│    param_id=%-10d  %s\n",
                       results[i].param_id, results[i].err_msg);
                shown++;
            }
        }

        printf("└──────────────────────────────────────────────────────────────┘\n");
        fflush(stdout);

        if (batch_no_fc > 0) {
            ESP_LOGW("kx_nofc", "── params sin fc_read valido o view=0 (%d) ──", batch_no_fc);
            for (int i = 0; i < valid_count; i++) {
                if (results[i].ok) continue;
                _find_ctrl_ctx_t fctx = {
                    .target_param_id = snapshot[i].param_id, .found_ctrl_id = -1
                };
                kx_param_store_foreach(_find_ctrl_cb, &fctx);
                const kx_param_t *p = NULL;
                if (fctx.found_ctrl_id >= 0)
                    p = kx_param_store_get_param(fctx.found_ctrl_id, snapshot[i].param_id);
                if (p) {
                    const char *reason = (p->view == 0)          ? "view=0"
                                       : (p->function_read == 0) ? "fc_r=0 (write-only)"
                                                                  : results[i].err_msg;
                    ESP_LOGW("kx_nofc",
                             "  id=%-10d reg=0x%04x fc_r=%d fc_w=%d view=%d  %-20s  %s",
                             p->param_id, p->reg, p->function_read,
                             p->function_write, p->view, p->name, reason);
                } else {
                    ESP_LOGW("kx_nofc", "  id=%-10d (no encontrado en store)  err=%s",
                             snapshot[i].param_id, results[i].err_msg);
                }
            }
        }

        ESP_LOGI(TAG,
            "batch done -- demanded=%d packaged=%d no_fc=%d "
            "published=%d errors=%d drops=%d dupes=%d expired=%d heap=%" PRIu32,
            valid_count, batch_packaged, batch_no_fc,
            batch_ok, batch_errors, batch_dropped,
            dupes, expired, kx_system_heap_free());

        free(results);
        free(snapshot);
    }

    kx_modbus_uart_deinit();
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
        ESP_LOGE(TAG, "pause: timeout -- memory risk!");
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

    esp_err_t err = kx_modbus_uart_init();
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

    ESP_LOGI(TAG, "started -- writer prio=%d poll prio=%d report prio=%d pub prio=%d",
             KX_TASK_PRIO_TELEMETRY + 2,
             KX_TASK_PRIO_TELEMETRY + 1,
             KX_TASK_PRIO_TELEMETRY,
             KX_TASK_PRIO_TELEMETRY - 1);

    return ESP_OK;
}

void kx_modbus_master_stop(void) { s_running = false; }
bool kx_modbus_master_is_running(void) { return s_running; }

esp_err_t kx_modbus_master_ensure_started(void)
{
    if (s_running) return ESP_OK;
    ESP_LOGI(TAG, "first RTU control detected — starting RTU driver");
    return kx_modbus_master_start();
}