#include "kx_modbus.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

// =============================================================
// kx_modbus.c  —  Driver Modbus RTU master
//
// Implementación mínima con driver UART del IDF (sin freemodbus).
// En cada ciclo itera sobre todos los params del kx_param_store,
// lee el registro Modbus correspondiente y publica el valor.
//
// Códigos de función soportados:
//   1  — Read Coils               (1 bit,  1 reg = 1 bit)
//   2  — Read Discrete Inputs     (1 bit,  1 reg = 1 bit)
//   3  — Read Holding Registers   (16 bit, 1 reg = 2 bytes)
//   4  — Read Input Registers     (16 bit, 1 reg = 2 bytes)
// =============================================================

static const char *TAG = "kx_modbus";

// ── Framing Modbus RTU ────────────────────────────────────────

// Tabla CRC-16 (polinomio 0xA001)
static uint16_t _crc16(const uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

// Construye una PDU de lectura (FC 01-04) en buf[].
// Retorna el número de bytes escritos.
static int _build_read_request(uint8_t *buf,
                                uint8_t slave,
                                uint8_t fc,
                                uint16_t reg,
                                uint16_t count)
{
    buf[0] = slave;
    buf[1] = fc;
    buf[2] = (reg  >> 8) & 0xFF;
    buf[3] =  reg        & 0xFF;
    buf[4] = (count >> 8) & 0xFF;
    buf[5] =  count       & 0xFF;
    uint16_t crc = _crc16(buf, 6);
    buf[6] = crc & 0xFF;
    buf[7] = (crc >> 8) & 0xFF;
    return 8;
}

// ── Lectura síncrona de un registro ──────────────────────────
esp_err_t kx_modbus_read_reg(uint8_t slave_addr,
                              uint8_t function,
                              uint16_t reg,
                              uint16_t *out_value)
{
    if (!out_value) return ESP_ERR_INVALID_ARG;

    uint8_t req[8];
    int req_len = _build_read_request(req, slave_addr, function, reg, 1);

    // Vaciar buffer UART antes de enviar
    uart_flush(KX_MODBUS_UART_PORT);

    // Enviar petición
    int written = uart_write_bytes(KX_MODBUS_UART_PORT, req, req_len);
    if (written != req_len) {
        ESP_LOGW(TAG, "uart write short: %d/%d", written, req_len);
        return ESP_FAIL;
    }

    // Esperar respuesta
    // FC 01/02 bit: byte_count = ceil(1/8) = 1 → respuesta = 5 bytes
    // FC 03/04 word: byte_count = 2          → respuesta = 7 bytes
    int expected_len = (function == 1 || function == 2) ? 6 : 7;
    uint8_t resp[16] = {0};

    int received = uart_read_bytes(KX_MODBUS_UART_PORT,
                                   resp,
                                   expected_len,
                                   pdMS_TO_TICKS(KX_MODBUS_TIMEOUT_MS));

    if (received < expected_len) {
        ESP_LOGW(TAG, "timeout/short reply: got %d, want %d "
                 "(slave=%d fc=%d reg=%d)",
                 received, expected_len, slave_addr, function, reg);
        return ESP_ERR_TIMEOUT;
    }

    // Verificar CRC
    uint16_t crc_recv = (uint16_t)(resp[received - 1] << 8) |
                         resp[received - 2];
    uint16_t crc_calc = _crc16(resp, received - 2);
    if (crc_recv != crc_calc) {
        ESP_LOGW(TAG, "CRC error: recv=0x%04X calc=0x%04X", crc_recv, crc_calc);
        return ESP_ERR_INVALID_CRC;
    }

    // Verificar que la respuesta es del esclavo correcto y la FC correcta
    if (resp[0] != slave_addr) {
        ESP_LOGW(TAG, "wrong slave in response: %d vs %d", resp[0], slave_addr);
        return ESP_FAIL;
    }

    // Detectar excepción Modbus (FC + 0x80)
    if (resp[1] == (function | 0x80)) {
        ESP_LOGW(TAG, "Modbus exception: slave=%d fc=%d code=%d",
                 slave_addr, function, resp[2]);
        return ESP_FAIL;
    }

    // Extraer valor
    if (function == 1 || function == 2) {
        // bits: resp[3] es el byte de datos
        *out_value = resp[3] & 0x01;
    } else {
        // words: resp[3] hi, resp[4] lo
        *out_value = (uint16_t)((resp[3] << 8) | resp[4]);
    }

    return ESP_OK;
}

// ── Aplicar transformación del param al valor raw ─────────────
//
//   valor_final = (raw * offset) + addition
//
//   Si offset == 0 lo tratamos como 1 (sin escala).
//   El mask se aplica antes (si != 0).
static float _apply_transform(const kx_param_t *p, uint16_t raw)
{
    float value = (float)raw;

    // Máscara de bits (si el registro comparte bits con otros params)
    if (p->mask != 0) {
        value = (float)((uint16_t)raw & (uint16_t)p->mask);
    }

    // Escala (offset en el modelo de datos de kiconex actúa como factor)
    float scale = (p->offset != 0.0f) ? p->offset : 1.0f;
    value = value * scale + p->addition;

    // Clamp al rango declarado
    if (p->minvalue < p->maxvalue) {
        if (value < p->minvalue) value = p->minvalue;
        if (value > p->maxvalue) value = p->maxvalue;
    }

    return value;
}

// ── Contexto de iteración ─────────────────────────────────────
typedef struct {
    int   ok;
    int   err;
    int   skipped;
} _poll_ctx_t;

// ── Callback por cada param — lee Modbus y publica ────────────
static void _poll_param(int control_id,
                         const kx_param_t *param,
                         void *user_data)
{
    _poll_ctx_t *ctx = (_poll_ctx_t *)user_data;

    // Ignorar params sin función de lectura
    if (param->function_read == 0) {
        ctx->skipped++;
        return;
    }

    // Ignorar params ocultos (view == 0)
    if (param->view == 0) {
        ctx->skipped++;
        return;
    }

    // La dirección del esclavo Modbus es el control_id.
    // En el protocolo kiconex cada control es un esclavo distinto.
    uint8_t slave = (uint8_t)(control_id & 0xFF);

    uint16_t raw = 0;
    esp_err_t err = kx_modbus_read_reg(slave,
                                        (uint8_t)param->function_read,
                                        (uint16_t)param->reg,
                                        &raw);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "read fail ctrl=%d param=%d [%s] reg=%d fc=%d err=%s",
                 control_id, param->param_id, param->name,
                 param->reg, param->function_read,
                 esp_err_to_name(err));
        kx_param_pub_error(control_id, param->param_id,
                           esp_err_to_name(err));
        ctx->err++;
        vTaskDelay(pdMS_TO_TICKS(KX_MODBUS_READ_DELAY_MS));
        return;
    }

    float value = _apply_transform(param, raw);

    ESP_LOGI(TAG, "ctrl=%d param=%d [%s] reg=%d raw=%u → %.3f",
             control_id, param->param_id, param->name,
             param->reg, raw, value);

    // Publicar en: {uuid}/controls/{control_id}/entities/{param_id}/status
    kx_param_pub_status(control_id, param->param_id, value);
    ctx->ok++;

    // Pausa mínima entre lecturas para no saturar el bus RS-485
    vTaskDelay(pdMS_TO_TICKS(KX_MODBUS_READ_DELAY_MS));
}

// ── Tarea de polling ──────────────────────────────────────────
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "task started, interval=%ds", KX_TELEMETRY_INTERVAL_S);

    // Esperar a que todos los controles hayan recibido sus entities
    ESP_LOGI(TAG, "waiting for all entities...");
    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "all entities ready: %d controls — starting Modbus polling",
             kx_param_store_count());
    vTaskDelay(pdMS_TO_TICKS(2000));   // margen para que el backend procese

    while (1) {
        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip poll: mqtt not connected");
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        _poll_ctx_t ctx = {0, 0, 0};
        int total = kx_param_store_count();

        ESP_LOGI(TAG, "poll start: %d controls heap=%" PRIu32,
                 total, kx_system_heap_free());

        kx_param_store_foreach(_poll_param, &ctx);

        ESP_LOGI(TAG, "poll done: ok=%d err=%d skip=%d heap=%" PRIu32,
                 ctx.ok, ctx.err, ctx.skipped, kx_system_heap_free());

        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
    }
}

// ── API pública ───────────────────────────────────────────────
esp_err_t kx_modbus_start(void)
{
    // Configurar UART estándar
    uart_config_t uart_cfg = {
        .baud_rate  = KX_MODBUS_BAUD_RATE,
        .data_bits  = KX_MODBUS_DATA_BITS,
        .parity     = KX_MODBUS_PARITY,
        .stop_bits  = KX_MODBUS_STOP_BITS,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, // Sin control de flujo hardware
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Aplicar configuración
    ESP_ERROR_CHECK(uart_param_config(KX_MODBUS_UART_PORT, &uart_cfg));

    // Configurar Pines: Eliminamos la referencia al pin 18 (RTS)
    ESP_ERROR_CHECK(uart_set_pin(KX_MODBUS_UART_PORT,
                                 KX_MODBUS_TXD_PIN,
                                 KX_MODBUS_RXD_PIN,
                                 UART_PIN_NO_CHANGE, // RTS no se usa
                                 UART_PIN_NO_CHANGE)); // CTS no se usa

    // Instalación del driver (buffers mínimos para ahorro de RAM)
    ESP_ERROR_CHECK(uart_driver_install(KX_MODBUS_UART_PORT,
                                         512, 256, 0, NULL, 0));

    // IMPORTANTE: Al no usar pin RTS para el transceptor RS-485, 
    // usamos el modo UART normal en lugar de UART_MODE_RS485_HALF_DUPLEX.
    ESP_ERROR_CHECK(uart_set_mode(KX_MODBUS_UART_PORT, UART_MODE_UART));

    ESP_LOGI(TAG, "UART%d inicializada (Modo Directo/Auto-RS485): %dbaud TX=%d RX=%d",
             KX_MODBUS_UART_PORT, KX_MODBUS_BAUD_RATE,
             KX_MODBUS_TXD_PIN, KX_MODBUS_RXD_PIN);

    // Lanzar la tarea de polling
    BaseType_t ret = xTaskCreate(
        _modbus_task,
        "kx_modbus",
        8192,
        NULL,
        KX_TASK_PRIO_TELEMETRY,
        NULL
    );

    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}