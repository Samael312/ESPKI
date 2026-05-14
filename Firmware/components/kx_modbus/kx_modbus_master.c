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
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <float.h>

static const char *TAG = "kx_modbus";

// =============================================================
// Parámetros UART/RS-485 — definir en kx_config.h si no existen
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

// Tiempos
#define MODBUS_RESPONSE_TIMEOUT_MS   100
#define MODBUS_INTER_FRAME_MS         20
#define MODBUS_INTER_PARAM_MS         10
#define MODBUS_RETRY_COUNT             2

// Funciones Modbus soportadas
#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

static volatile bool s_running = false;
static TaskHandle_t  s_task    = NULL;
static SemaphoreHandle_t s_uart_mutex = NULL;

// ── CRC16 Modbus ──────────────────────────────────────────────
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

// ── Init UART ─────────────────────────────────────────────────
static esp_err_t _uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate           = KX_MODBUS_BAUD,
        .data_bits           = UART_DATA_8_BITS, 
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_param_config(KX_MODBUS_UART_NUM, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(KX_MODBUS_UART_NUM,
                       KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN,
                       KX_MODBUS_RTS_PIN, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(KX_MODBUS_UART_NUM,
                              256,   // rx buf
                              256,   // tx buf
                              0, NULL, 0);
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

// ── Enviar trama y recibir respuesta ──────────────────────────
// frame:     trama SIN CRC
// frame_len: longitud sin CRC
// resp:      buffer de respuesta
// resp_max:  tamaño máximo del buffer
// Devuelve bytes recibidos, o -1 en error/timeout.
static int _modbus_transaction(const uint8_t *frame, size_t frame_len,
                               uint8_t *resp,  size_t resp_max)
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
    if (rx_len <= 0) {
        return -1;  // timeout real
    }

    if (rx_len < 4) {
        ESP_LOGW(TAG, "response too short: rx=%d", rx_len);
        return -1;
    }

    // validar CRC
    uint16_t rx_crc   = ((uint16_t)resp[rx_len - 1] << 8) | resp[rx_len - 2];
    uint16_t calc_crc = _crc16(resp, rx_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC error: got %04x expected %04x", rx_crc, calc_crc);
        return -1;
    }

    // exception response: bit 7 del function code activo
    if (resp[1] & 0x80) {
        uint8_t exc = (rx_len > 2) ? resp[2] : 0;
        ESP_LOGW(TAG, "Modbus exception: fc=0x%02x exc=0x%02x", resp[1], exc);
        return -1;
    }

    return rx_len;
}

// ── Leer un registro o coil ───────────────────────────────────
// Soporta FC01, FC02, FC03, FC04.
// Devuelve el valor escalado, o -FLT_MAX en error.
static float _read_register(uint8_t slave_addr, uint16_t reg_addr,
                             uint8_t fc, const kx_param_t *param)
{
    uint8_t frame[6];
    frame[0] = slave_addr;
    frame[1] = fc;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)(reg_addr & 0xFF);
    frame[4] = 0x00;
    frame[5] = 0x01;   // leer 1 registro / 1 coil

    uint8_t resp[16];
    int rx = -1;

    if (xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "uart mutex timeout param=%d", param->param_id);
        return -FLT_MAX;
    }
    for (int attempt = 0; attempt < MODBUS_RETRY_COUNT && rx < 0; attempt++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) {
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
    }

    if (rx < 0) {
        xSemaphoreGive(s_uart_mutex);
        return -FLT_MAX;
    }

    // resp[2] = bytecount
    if (rx < 4 || resp[2] == 0) {
        ESP_LOGW(TAG, "short response: rx=%d bytecount=%d", rx, resp[2]);
        return -FLT_MAX;
    }

    uint16_t raw;

    

    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE) {
        // FC01/FC02: bytecount=1, datos en resp[3], 1 coil = bit 0
        raw = resp[3] & 0x01;

    } else {
        // FC03/FC04: bytecount=2, datos en resp[3..4] big-endian
        if (rx < 5 || resp[2] < 2) {
            ESP_LOGW(TAG, "short FC%02d response: rx=%d bytecount=%d",
                     fc, rx, resp[2]);
            return -FLT_MAX;
        }
        raw = ((uint16_t)resp[3] << 8) | resp[4];
    }

    float value = (float)(int16_t)raw;   // signed: permite valores negativos

    // escala/offset del param
    if (param->offset != 0.0f && param->offset != 1.0f) {
        value = value * param->offset;
    }
    value += param->addition;

    // clamp al rango
    if (value < param->minvalue) value = param->minvalue;
    if (value > param->maxvalue) value = param->maxvalue;

    xSemaphoreGive(s_uart_mutex);
    return value;
}

// ── Contexto de iteración ─────────────────────────────────────
typedef struct {
    int total;      // total de params a leer en este ciclo
    int done;       // params procesados (ok + error)
    int ok;         // lecturas exitosas
    int errors;     // errores Modbus
    int last_pct;   // último porcentaje de barra impreso
} _poll_ctx_t;

// ── Barra de progreso ─────────────────────────────────────────
#define POLL_BAR_WIDTH 30

static void _print_progress(int control_id, int done, int total)
{
    if (total <= 0) return;

    int pct   = (done * 100) / total;
    int fill  = (done * POLL_BAR_WIDTH) / total;

    char bar[POLL_BAR_WIDTH + 1];
    for (int i = 0; i < POLL_BAR_WIDTH; i++) {
        bar[i] = (i < fill) ? '#' : '-';
    }
    bar[POLL_BAR_WIDTH] = '\0';

    // \r para sobreescribir la línea en terminales que lo soporten
    printf("[poll] ctrl=%d [%s] %3d%% (%d/%d params)\r",
           control_id, bar, pct, done, total);
    fflush(stdout);

    // salto de línea en los hitos para que quede en el log
    if (pct == 25 || pct == 50 || pct == 75 || pct == 100) {
        printf("\n");
        fflush(stdout);
    }
}

// ── Publicar valor en MQTT ────────────────────────────────────
static void _publish_value(int control_id, const kx_param_t *param, float value)
{
    if (param->function_read != 0) {

        if (param->sampling != 0) {
        // 1. Enviamos el REPORT (Dato histórico/telemetría)
        kx_param_pub_report(control_id, param->param_id, value);
        ESP_LOGW(TAG, "report → ctrl=%d param=%d val=%.3f", control_id, param->param_id, value);
        }

        // 2. Enviamos el STATUS (Estado actual del canal)
        kx_param_pub_status(control_id, param->param_id, value);
        
        
        
        ESP_LOGD(TAG, "-> read ok ctrl=%d param=%d val=%.3f", control_id, param->param_id, value);
    } else {
        // Caso de escritura (SET)
        kx_param_pub_set(control_id, param->param_id, value);
    }
}

// ── Publicar error de lectura ─────────────────────────────────
static void _publish_error(int control_id, const kx_param_t *param, const char *reason)
{
    // Solo enviamos STATUS informando del error (no se envía REPORT si hay error)
    kx_param_pub_error_modbus(control_id, param->param_id, (uint16_t)param->reg, reason);
    
    ESP_LOGW(TAG, "modbus error ctrl=%d param=%d reg=0x%04x: %s",
             control_id, param->param_id, param->reg, reason);
}

// ── Callback de iteración: lee y publica cada param ──────────
static void _poll_param(int control_id,
                         const kx_param_t *param,
                         void *user_data)
{
    _poll_ctx_t *ctx = (_poll_ctx_t *)user_data;

    // ignorar params sin función alguna
    if (param->function_read == 0 && param->function_write == 0) return;

    // ignorar params ocultos
    if (param->view == 0) return;

    // obtener slave_addr del store
    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return;
    uint8_t slave_addr = (uint8_t)ctrl->slave_addr;

    // solo FCs de lectura
    uint8_t fc_read = (uint8_t)param->function_read;
    bool is_read_fc = (fc_read == MB_FC_READ_COILS        ||
                       fc_read == MB_FC_READ_DISCRETE      ||
                       fc_read == MB_FC_READ_HOLDING_REGS  ||
                       fc_read == MB_FC_READ_INPUT_REGS);

    if (!is_read_fc) return;   // write-only: se gestiona por comando

    float value = _read_register(slave_addr, (uint16_t)param->reg, fc_read, param);

    if (value == -FLT_MAX) {
        _publish_error(control_id, param, "modbus_timeout");
        if (ctx) ctx->errors++;
    } else {
        _publish_value(control_id, param, value);
        if (ctx) ctx->ok++;
    }

    if (ctx) {
        ctx->done++;
        _print_progress(control_id, ctx->done, ctx->total);
    }

    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
}

esp_err_t kx_modbus_read_one(int control_id, int param_id)
{
    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "read_one: ctrl=%d not found or no slave_addr", control_id);
        return ESP_ERR_NOT_FOUND;
    }

    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) {
        ESP_LOGW(TAG, "read_one: param=%d not found in ctrl=%d", param_id, control_id);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t fc = (uint8_t)param->function_read;
    bool is_read_fc = (fc == MB_FC_READ_COILS        ||
                       fc == MB_FC_READ_DISCRETE      ||
                       fc == MB_FC_READ_HOLDING_REGS  ||
                       fc == MB_FC_READ_INPUT_REGS);

    if (!is_read_fc) {
        ESP_LOGW(TAG, "read_one: param=%d has no read FC", param_id);
        return ESP_ERR_INVALID_ARG;
    }

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg,
                                  fc, param);

    if (value == -FLT_MAX) {
        _publish_error(control_id, param, "modbus_timeout");
        return ESP_FAIL;
    }

    _publish_value(control_id, param, value);
    ESP_LOGI(TAG, "read_one: ctrl=%d param=%d val=%.3f", control_id, param_id, value);
    return ESP_OK;
}

// ── Contar params legibles de un control ──────────────────────
// Necesitamos el total antes de iterar para la barra.
typedef struct { int count; } _count_ctx_t;

static void _count_readable(int control_id, const kx_param_t *param, void *user_data)
{
    _count_ctx_t *c = (_count_ctx_t *)user_data;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    bool is_read_fc = (fc == MB_FC_READ_COILS       ||
                       fc == MB_FC_READ_DISCRETE     ||
                       fc == MB_FC_READ_HOLDING_REGS ||
                       fc == MB_FC_READ_INPUT_REGS);
    if (is_read_fc) c->count++;
}



typedef struct {int control_id; int reads_triggered; } _update_ctx_t;

static void _on_param_changed(int control_id,
                               const kx_param_diff_t *diff,
                               void *user_data)
{
    _update_ctx_t *ctx = (_update_ctx_t *)user_data;

    ESP_LOGI(TAG, "param changed: ctrl=%d param=%d mask=0x%02x "
             "(sampling=%d fc_read=%d fc_write=%d)",
             control_id, diff->param_id, (unsigned)diff->changed,
             diff->new_param.sampling,
             diff->new_param.function_read,
             diff->new_param.function_write);

    // Solo releer por Modbus si cambió algo que afecta a la lectura
    bool needs_read = (diff->changed & KX_PARAM_CHANGED_FUNCTION_READ) ||
                      (diff->changed & KX_PARAM_CHANGED_REG)           ||
                      (diff->changed & KX_PARAM_CHANGED_OFFSET)        ||
                      (diff->changed & KX_PARAM_CHANGED_ADDITION)      ||
                      (diff->changed & KX_PARAM_CHANGED_MINMAX)        ||
                      (diff->changed & KX_PARAM_CHANGED_SAMPLING);     // nuevo sampling → publicar valor actual

    if (needs_read) {
        esp_err_t err = kx_modbus_read_one(control_id, diff->param_id);
        if (err == ESP_OK) ctx->reads_triggered++;
        // pequeño gap entre lecturas para no saturar el bus
        vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }
}

esp_err_t kx_modbus_update_changed(int control_id,
                                    const char *payload,
                                    size_t len)
{
    ESP_LOGI(TAG, "update_changed: ctrl=%d payload_len=%d", control_id, (int)len);

    _update_ctx_t ctx = { .control_id = control_id, .reads_triggered = 0 };

    int changed = kx_param_store_diff_and_update(payload, len, control_id,
                                                  _on_param_changed, &ctx);

    if (changed < 0) {
        ESP_LOGE(TAG, "update_changed: diff failed ctrl=%d", control_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "update_changed: ctrl=%d — %d params changed, %d reads triggered",
             control_id, changed, ctx.reads_triggered);

    return ESP_OK;
}

// ── Tarea principal ───────────────────────────────────────────
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "task started — waiting for entities...");

    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "entities ready (%d controls) — starting Modbus polling",
             kx_param_store_count());

    vTaskDelay(pdMS_TO_TICKS(4000));

    while (s_running) {
        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip poll: mqtt not connected");
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        // contar params legibles para la barra
        _count_ctx_t cc = { .count = 0 };
        kx_param_store_foreach(_count_readable, &cc);

        _poll_ctx_t ctx = {
            .total    = cc.count,
            .done     = 0,
            .ok       = 0,
            .errors   = 0,
            .last_pct = -1,
        };

        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "poll cycle: %d controls | %d params | heap=%" PRIu32,
                 kx_param_store_count(), ctx.total, kx_system_heap_free());

        kx_param_store_foreach(_poll_param, &ctx);

        // línea final de resumen
        ESP_LOGI(TAG, "poll done: read=%d errors=%d | published to platform | heap=%" PRIu32,
                 ctx.ok, ctx.errors, kx_system_heap_free());
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
    }

    uart_driver_delete(KX_MODBUS_UART_NUM);
    ESP_LOGI(TAG, "task stopped");
    vTaskDelete(NULL);
}

// ── API pública ───────────────────────────────────────────────
esp_err_t kx_modbus_master_start(void)
{
    if (s_running) {
        ESP_LOGW(TAG, "already running");
        return ESP_OK;
    }

    esp_err_t err = _uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_running = true;

    s_uart_mutex = xSemaphoreCreateMutex();
    
    BaseType_t ret = xTaskCreate(
        _modbus_task,
        "kx_modbus",
        8192,
        NULL,
        KX_TASK_PRIO_TELEMETRY,
        &s_task
    );

    if (ret != pdPASS) {
        s_running = false;
        return ESP_FAIL;
    }

    return ESP_OK;
}

void kx_modbus_master_stop(void)
{
    s_running = false;
}

bool kx_modbus_master_is_running(void)
{
    return s_running;
}