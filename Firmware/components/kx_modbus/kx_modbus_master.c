#include "kx_modbus_master.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"

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
// Pin DE/RE del transceiver RS-485 (-1 si no se usa)
#ifndef KX_MODBUS_RTS_PIN
#define KX_MODBUS_RTS_PIN    -1
#endif

// Tiempos
#define MODBUS_RESPONSE_TIMEOUT_MS   500
#define MODBUS_INTER_FRAME_MS         20   // silencio entre tramas
#define MODBUS_INTER_PARAM_MS         10   // pausa entre registros de un mismo control
#define MODBUS_RETRY_COUNT             3

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

// ── Timestamp real ────────────────────────────────────────────
static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
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

    // Modo RS-485 half-duplex con control automático de DE/RE
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
// frame:    trama completa SIN CRC
// frame_len: longitud sin CRC
// resp:     buffer de respuesta
// resp_max: tamaño máximo del buffer
// Devuelve número de bytes recibidos, o -1 en error.
static int _modbus_transaction(const uint8_t *frame, size_t frame_len,
                               uint8_t *resp,  size_t resp_max)
{
    // calcular y adjuntar CRC
    uint8_t tx[frame_len + 2];
    memcpy(tx, frame, frame_len);
    uint16_t crc = _crc16(frame, frame_len);
    tx[frame_len]     = (uint8_t)(crc & 0xFF);
    tx[frame_len + 1] = (uint8_t)(crc >> 8);

    // limpiar rx pendiente
    uart_flush_input(KX_MODBUS_UART_NUM);

    // enviar
    uart_write_bytes(KX_MODBUS_UART_NUM, (const char *)tx, frame_len + 2);

    // esperar respuesta
    int rx_len = uart_read_bytes(KX_MODBUS_UART_NUM, resp, resp_max,
                                  pdMS_TO_TICKS(MODBUS_RESPONSE_TIMEOUT_MS));
    if (rx_len <= 0) {
        return -1;  // timeout
    }

    // validar CRC de la respuesta
    if (rx_len < 4) return -1;  // mínimo: addr + fc + data + crc(2)

    uint16_t rx_crc = ((uint16_t)resp[rx_len - 1] << 8) | resp[rx_len - 2];
    uint16_t calc_crc = _crc16(resp, rx_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC error: got %04x expected %04x", rx_crc, calc_crc);
        return -1;
    }

    // comprobar exception response (bit 7 del function code)
    if (resp[1] & 0x80) {
        ESP_LOGW(TAG, "Modbus exception: fc=0x%02x code=0x%02x",
                 resp[1], rx_len > 2 ? resp[2] : 0);
        return -1;
    }

    return rx_len;
}

// ── Leer un registro (FC03 / FC04) ───────────────────────────
// Devuelve el valor crudo del registro, o FLT_MIN en error.
// slave_addr: dirección Modbus del esclavo (1-247)
// reg_addr:   dirección del registro (0-based)
// fc:         función (3 = holding, 4 = input)
static float _read_register(uint8_t slave_addr, uint16_t reg_addr,
                             uint8_t fc, const kx_param_t *param)
{
    uint8_t frame[6];
    frame[0] = slave_addr;
    frame[1] = fc;
    frame[2] = (uint8_t)(reg_addr >> 8);
    frame[3] = (uint8_t)(reg_addr & 0xFF);
    frame[4] = 0x00;
    frame[5] = 0x01;   // leer 1 registro

    uint8_t resp[16];
    int rx = -1;

    for (int attempt = 0; attempt < MODBUS_RETRY_COUNT && rx < 0; attempt++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) {
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
    }

    if (rx < 0) {
        return -FLT_MAX;   // señal de error
    }

    // resp[2] = byte count, resp[3..4] = valor del registro (big-endian)
    if (rx < 5 || resp[2] < 2) {
        ESP_LOGW(TAG, "short response: rx=%d", rx);
        return -FLT_MAX;
    }

    uint16_t raw = ((uint16_t)resp[3] << 8) | resp[4];
    float    value = (float)raw;

    // aplicar escala/offset del param (mismo criterio que dummy_protocol)
    if (param->offset != 0.0f && param->offset != 1.0f) {
        value = value * param->offset;
    }
    value += param->addition;

    // clamp al rango del param
    if (value < param->minvalue) value = param->minvalue;
    if (value > param->maxvalue) value = param->maxvalue;

    return value;
}

// ── Publicar valor en MQTT (mismo esquema que dummy_protocol) ──
//
// Esto refleja exactamente lo que hace _publish_param() en
// kx_dummy_protocol.c, pero con el valor real obtenido de Modbus.
//
static void _publish_value(int control_id, const kx_param_t *param, float value)
{
    char topic[128];
    char payload[128];

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param->param_id, value, _ts());

    if (param->function_read != 0) {
        // report
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/report",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);

        // status
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/status",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);

        ESP_LOGD(TAG, "→ read  ctrl=%d param=%d reg=0x%04x [%s] value=%.3f",
                 control_id, param->param_id, param->reg, param->name, value);

    } else {
        // parámetro de escritura: publicar en set (reflejo del setpoint actual)
        snprintf(topic, sizeof(topic),
                 "%s/quiiot/entities/%d/set",
                 KX_DEVICE_UUID, param->param_id);
        kx_mqtt_publish(topic, payload, 0, 0);

        ESP_LOGD(TAG, "→ write ctrl=%d param=%d reg=0x%04x [%s] value=%.3f",
                 control_id, param->param_id, param->reg, param->name, value);
    }
}

// ── Publicar error de lectura ─────────────────────────────────
static void _publish_error(int control_id, const kx_param_t *param,
                            const char *reason)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/status",
             KX_DEVICE_UUID, param->param_id);

    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"error\":true,\"error_message\":\"%s\","
             "\"reg\":\"0x%04x\",\"ts\":%.3f}",
             param->param_id, reason, param->reg, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
    ESP_LOGW(TAG, "modbus error ctrl=%d param=%d reg=0x%04x: %s",
             control_id, param->param_id, param->reg, reason);
}

// ── Contexto de iteración para el callback ────────────────────
// Usamos una estructura para pasar el control_id al iterador,
// ya que kx_param_store_foreach pasa el control_id como argumento.

typedef struct {
    int dummy;  // no necesario, el control_id viene del iterador
} _iter_ctx_t;

// Callback invocado por kx_param_store_foreach
// Lee cada param via Modbus y publica el resultado
static void _poll_param(int control_id,
                         const kx_param_t *param,
                         void *user_data)
{
    // ignorar params sin función de lectura ni escritura
    if (param->function_read == 0 && param->function_write == 0) return;

    // ignorar params ocultos
    if (param->view == 0) return;

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "no slave_addr for ctrl=%d, skipping", control_id);
        return;
    }
    int slave_addr = ctrl->slave_addr;

    // Determinar función: preferir function_read, fallback a function_write
    uint8_t fc = (param->function_read != 0)
               ? (uint8_t)param->function_read
               : (uint8_t)param->function_write;

    // Solo FC03, FC04 soportados para lectura; FC06/FC10 para escritura (no leemos)
    bool is_read_fc = (fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS
                    || fc == MB_FC_READ_COILS         || fc == MB_FC_READ_DISCRETE);

    if (!is_read_fc) {
        // Registro de solo escritura: no hay nada que leer en esta fase.
        // La escritura se hace cuando el broker envía un comando (Fase 3).
        ESP_LOGD(TAG, "skip write-only param=%d reg=0x%04x",
                 param->param_id, param->reg);
        return;
    }

    float value = _read_register(slave_addr, (uint16_t)param->reg, fc, param);

    if (value == -FLT_MAX) {
        _publish_error(control_id, param, "modbus_timeout");
    } else {
        _publish_value(control_id, param, value);
    }

    // pequeña pausa entre registros para no saturar el bus
    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
}

// ── Tarea principal ───────────────────────────────────────────
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "task started — waiting for entities...");

    // Esperar a que el store esté listo (puede venir de NVS o de descarga)
    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "entities ready (%d controls) — starting Modbus polling",
             kx_param_store_count());

    // pequeño margen para que el backend procese el status inicial
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (s_running) {
        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip poll: mqtt not connected");
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        ESP_LOGI(TAG, "modbus poll cycle: %d controls | heap=%" PRIu32,
                 kx_param_store_count(), kx_system_heap_free());

        kx_param_store_foreach(_poll_param, NULL);

        ESP_LOGI(TAG, "modbus cycle done | heap=%" PRIu32, kx_system_heap_free());

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
    // La tarea se autodestruye en el próximo ciclo
}

bool kx_modbus_master_is_running(void)
{
    return s_running;
}