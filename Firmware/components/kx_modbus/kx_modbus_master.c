#include "kx_modbus_master.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>

static const char *TAG = "kx_modbus";

// ─────────────────────────────────────────────────────────────
// CRC-16 Modbus (polinomio 0x8005, reflectado = 0xA001)
// ─────────────────────────────────────────────────────────────
static uint16_t _crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1u) ? (crc >> 1) ^ 0xA001u : (crc >> 1);
        }
    }
    return crc;
}

// ─────────────────────────────────────────────────────────────
// Construcción de trama RTU (8 bytes para FC01-FC04, FC06)
//   [ADDR][FC][REG_HI][REG_LO][QTY_HI][QTY_LO][CRC_LO][CRC_HI]
// ─────────────────────────────────────────────────────────────
static int _build_read_frame(const kx_mb_request_t *req,
                              uint8_t *frame, size_t frame_max)
{
    if (frame_max < 8) return -1;
    frame[0] = req->slave_addr;
    frame[1] = req->function;
    frame[2] = (req->reg_addr >> 8) & 0xFF;
    frame[3] =  req->reg_addr       & 0xFF;
    frame[4] = (req->quantity >> 8) & 0xFF;
    frame[5] =  req->quantity       & 0xFF;
    uint16_t crc = _crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;
    return 8;
}

// FC06: Write Single Register
static int _build_write_single_frame(uint8_t slave, uint16_t reg,
                                      uint16_t val,
                                      uint8_t *frame, size_t frame_max)
{
    if (frame_max < 8) return -1;
    frame[0] = slave;
    frame[1] = MB_FC_WRITE_SINGLE_REG;
    frame[2] = (reg >> 8) & 0xFF;
    frame[3] =  reg       & 0xFF;
    frame[4] = (val >> 8) & 0xFF;
    frame[5] =  val       & 0xFF;
    uint16_t crc = _crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;
    return 8;
}

// ─────────────────────────────────────────────────────────────
// Lectura robusta de respuesta RTU
//   1. Espera el primer byte hasta response_timeout_ms
//   2. Lee el resto con timeout inter-carácter (fin de trama)
// ─────────────────────────────────────────────────────────────
static int _read_response(uint8_t *buf, size_t max_len)
{
    int total = 0;

    // esperar primer byte con timeout largo
    int n = uart_read_bytes(KX_MODBUS_UART_NUM, buf, 1,
                            pdMS_TO_TICKS(KX_MODBUS_RESP_TIMEOUT_MS));
    if (n <= 0) return 0;
    total = 1;

    // leer el resto con timeout inter-carácter (detecta fin de trama)
    while (total < (int)max_len) {
        n = uart_read_bytes(KX_MODBUS_UART_NUM,
                            buf + total,
                            max_len - total,
                            pdMS_TO_TICKS(KX_MODBUS_INTER_CHAR_MS));
        if (n <= 0) break;   // silencio → trama completa
        total += n;
    }
    return total;
}

// ─────────────────────────────────────────────────────────────
// Parser de respuesta RTU
// ─────────────────────────────────────────────────────────────
static esp_err_t _parse_response(const uint8_t *buf, int len,
                                  uint8_t slave_addr,
                                  uint8_t expected_fc,
                                  kx_mb_response_t *resp)
{
    memset(resp, 0, sizeof(*resp));
    resp->function = expected_fc;

    if (len < 4) {
        ESP_LOGW(TAG, "respuesta demasiado corta: %d bytes", len);
        return ESP_FAIL;
    }

    // verificar CRC
    uint16_t crc_calc = _crc16(buf, len - 2);
    uint16_t crc_recv = (uint16_t)buf[len - 2] |
                        ((uint16_t)buf[len - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGW(TAG, "CRC error calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
        return ESP_FAIL;
    }

    // verificar dirección esclavo
    if (buf[0] != slave_addr) {
        ESP_LOGW(TAG, "dirección inesperada: esperado=0x%02X recibido=0x%02X",
                 slave_addr, buf[0]);
        return ESP_FAIL;
    }

    resp->function = buf[1];

    // excepción Modbus: bit 7 del FC puesto
    if (buf[1] & 0x80) {
        resp->exception_code = buf[2];
        ESP_LOGW(TAG, "excepción Modbus FC=0x%02X código=%d (slave=0x%02X)",
                 buf[1], buf[2], slave_addr);
        return ESP_FAIL;
    }

    if (buf[1] != expected_fc) {
        ESP_LOGW(TAG, "FC inesperado: esperado=0x%02X recibido=0x%02X",
                 expected_fc, buf[1]);
        return ESP_FAIL;
    }

    // FC01..FC04: byte_count + datos
    if (expected_fc >= 0x01 && expected_fc <= 0x04) {
        resp->data_len = buf[2];
        uint8_t copy_len = resp->data_len < sizeof(resp->data)
                           ? resp->data_len : (uint8_t)sizeof(resp->data);
        memcpy(resp->data, &buf[3], copy_len);
        resp->data_len = copy_len;
    }
    // FC06, FC10: echo de dirección + valor
    else {
        resp->data_len = (uint8_t)(len - 4);  // sin addr, fc, crc×2
        if (resp->data_len > sizeof(resp->data))
            resp->data_len = sizeof(resp->data);
        memcpy(resp->data, &buf[2], resp->data_len);
    }

    resp->ok = true;
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────
// UART init (modo estándar — MAX13487E gestiona dirección solo)
// ─────────────────────────────────────────────────────────────
esp_err_t kx_modbus_init(void)
{
    // Evitar doble inicialización
    if (uart_is_driver_installed(KX_MODBUS_UART_NUM)) {
        ESP_LOGW(TAG, "UART%d ya inicializado", KX_MODBUS_UART_NUM);
        return ESP_OK;
    }

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
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(KX_MODBUS_UART_NUM,
                       KX_MODBUS_PIN_TX,    // DI
                       KX_MODBUS_PIN_RX,    // RO (GPIO36, input-only)
                       UART_PIN_NO_CHANGE,  // sin RTS (auto-dir)
                       UART_PIN_NO_CHANGE); // sin CTS
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(KX_MODBUS_UART_NUM,
                              512,  // RX buffer
                              0,    // TX buffer (0 = blocking, más seguro para Modbus)
                              0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(err));
        return err;
    }

    // Modo UART estándar: MAX13487E gestiona DE/RE con auto-dirección.
    // No usamos UART_MODE_RS485_HALF_DUPLEX porque requiere RTS para DE.
    uart_set_mode(KX_MODBUS_UART_NUM, UART_MODE_UART);

    ESP_LOGI(TAG, "UART%d OK — TX=GPIO%d(DI) RX=GPIO%d(RO) baud=%d",
             KX_MODBUS_UART_NUM,
             KX_MODBUS_PIN_TX, KX_MODBUS_PIN_RX,
             KX_MODBUS_BAUD);
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────
// Transacción RTU con reintentos
// ─────────────────────────────────────────────────────────────
esp_err_t kx_modbus_transaction(const kx_mb_request_t *req,
                                 kx_mb_response_t *resp)
{
    uint8_t frame[8];
    int frame_len = _build_read_frame(req, frame, sizeof(frame));
    if (frame_len < 0) return ESP_ERR_INVALID_ARG;

    for (int attempt = 1; attempt <= KX_MODBUS_RETRY_COUNT; attempt++) {

        // limpiar RX antes de enviar
        uart_flush_input(KX_MODBUS_UART_NUM);

        int written = uart_write_bytes(KX_MODBUS_UART_NUM,
                                       (const char *)frame, frame_len);
        if (written != frame_len) {
            ESP_LOGW(TAG, "escritura parcial: %d/%d", written, frame_len);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // esperar a que el shift register vacíe el último byte
        uart_wait_tx_done(KX_MODBUS_UART_NUM, pdMS_TO_TICKS(50));

        // margen adicional: tiempo de propagación RS485 + turnaround esclavo
        vTaskDelay(pdMS_TO_TICKS(KX_MODBUS_TX_FLUSH_DELAY_MS));

        uint8_t rx_buf[256];
        int rx_len = _read_response(rx_buf, sizeof(rx_buf));

        if (rx_len <= 0) {
            ESP_LOGW(TAG, "timeout — slave=0x%02X fc=0x%02X reg=0x%04X "
                     "intento=%d/%d",
                     req->slave_addr, req->function, req->reg_addr,
                     attempt, KX_MODBUS_RETRY_COUNT);
            continue;
        }

        // log de bytes en modo DEBUG
        if (esp_log_level_get(TAG) >= ESP_LOG_DEBUG) {
            char hex[128] = "";
            int hpos = 0;
            for (int i = 0; i < rx_len && hpos < (int)sizeof(hex) - 4; i++) {
                hpos += snprintf(hex + hpos, sizeof(hex) - hpos,
                                 "%02X ", rx_buf[i]);
            }
            ESP_LOGD(TAG, "RX [%d bytes]: %s", rx_len, hex);
        }

        if (_parse_response(rx_buf, rx_len,
                            req->slave_addr, req->function, resp) == ESP_OK) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return ESP_FAIL;
}

// ─────────────────────────────────────────────────────────────
// Lectura de registro 16 bits → float con escalado
// ─────────────────────────────────────────────────────────────
esp_err_t kx_modbus_read_reg(uint8_t slave, uint8_t fc, uint16_t reg,
                               float *out_value,
                               float scale, float addition, uint16_t mask)
{
    kx_mb_request_t req = {
        .slave_addr = slave,
        .function   = fc,
        .reg_addr   = reg,
        .quantity   = 1,
    };
    kx_mb_response_t resp;

    esp_err_t err = kx_modbus_transaction(&req, &resp);
    if (err != ESP_OK || resp.data_len < 2) return ESP_FAIL;

    uint16_t raw = ((uint16_t)resp.data[0] << 8) | resp.data[1];
    if (mask != 0) raw &= mask;

    float v = (float)raw;
    if (scale != 0.0f && scale != 1.0f) v *= scale;
    v += addition;

    *out_value = v;
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────
// Lectura de registro 32 bits / IEEE-754 float (2 registros)
// Asume orden big-endian: reg[0]=high word, reg[1]=low word
// ─────────────────────────────────────────────────────────────
esp_err_t kx_modbus_read_reg32(uint8_t slave, uint8_t fc, uint16_t reg,
                                float *out_value,
                                float scale, float addition)
{
    kx_mb_request_t req = {
        .slave_addr = slave,
        .function   = fc,
        .reg_addr   = reg,
        .quantity   = 2,
    };
    kx_mb_response_t resp;

    esp_err_t err = kx_modbus_transaction(&req, &resp);
    if (err != ESP_OK || resp.data_len < 4) return ESP_FAIL;

    uint32_t raw = ((uint32_t)resp.data[0] << 24) |
                   ((uint32_t)resp.data[1] << 16) |
                   ((uint32_t)resp.data[2] <<  8) |
                    (uint32_t)resp.data[3];

    float v;
    memcpy(&v, &raw, 4);   // reinterpret como IEEE-754

    if (scale != 0.0f && scale != 1.0f) v *= scale;
    v += addition;

    *out_value = v;
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────
// Escritura de registro único (FC06)
// ─────────────────────────────────────────────────────────────
esp_err_t kx_modbus_write_reg(uint8_t slave, uint16_t reg, uint16_t value)
{
    uint8_t frame[8];
    int len = _build_write_single_frame(slave, reg, value,
                                        frame, sizeof(frame));
    if (len < 0) return ESP_ERR_INVALID_ARG;

    uart_flush_input(KX_MODBUS_UART_NUM);
    int written = uart_write_bytes(KX_MODBUS_UART_NUM,
                                   (const char *)frame, len);
    if (written != len) return ESP_FAIL;

    uart_wait_tx_done(KX_MODBUS_UART_NUM, pdMS_TO_TICKS(50));
    vTaskDelay(pdMS_TO_TICKS(KX_MODBUS_TX_FLUSH_DELAY_MS));

    // leer eco de confirmación (el esclavo devuelve la misma trama)
    uint8_t rx_buf[256];
    int rx_len = _read_response(rx_buf, sizeof(rx_buf));

    if (rx_len < 8) {
        ESP_LOGW(TAG, "write FC06 sin confirmación — slave=0x%02X reg=0x%04X",
                 slave, reg);
        return ESP_FAIL;
    }

    kx_mb_response_t resp;
    esp_err_t err = _parse_response(rx_buf, rx_len,
                                    slave, MB_FC_WRITE_SINGLE_REG, &resp);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "write OK slave=0x%02X reg=0x%04X val=0x%04X",
                 slave, reg, value);
    }
    return err;
}

// ─────────────────────────────────────────────────────────────
// Test manual — lectura raw con volcado de bytes
// ─────────────────────────────────────────────────────────────
void kx_modbus_test_read(uint8_t slave, uint8_t fc,
                          uint16_t reg, uint16_t quantity)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "══ TEST READ ════════════════════════════════");
    ESP_LOGI(TAG, "  slave=0x%02X  fc=0x%02X  reg=0x%04X  qty=%d",
             slave, fc, reg, quantity);

    kx_mb_request_t req = {
        .slave_addr = slave,
        .function   = fc,
        .reg_addr   = reg,
        .quantity   = quantity,
    };
    kx_mb_response_t resp;

    esp_err_t err = kx_modbus_transaction(&req, &resp);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "  RESULTADO: SIN RESPUESTA / ERROR");
        ESP_LOGI(TAG, "  Verificar: baud, cableado A/B, terminaciones, slave_id");
        ESP_LOGI(TAG, "══════════════════════════════════════════");
        return;
    }

    ESP_LOGI(TAG, "  RESULTADO: OK — %d bytes de datos", resp.data_len);

    for (int i = 0; i + 1 < resp.data_len; i += 2) {
        uint16_t word = ((uint16_t)resp.data[i] << 8) | resp.data[i + 1];
        float    fval = (float)word;
        ESP_LOGI(TAG, "  reg[0x%04X] = 0x%04X (%5u) ~ %.3f",
                 (unsigned)(reg + i / 2), word, word, fval);
    }
    ESP_LOGI(TAG, "══════════════════════════════════════════");
}

// ─────────────────────────────────────────────────────────────
// Scan de esclavos en el bus
// ─────────────────────────────────────────────────────────────
void kx_modbus_scan(uint8_t addr_from, uint8_t addr_to)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "══ BUS SCAN (%d → %d) ═══════════════════════",
             addr_from, addr_to);

    int found = 0;
    for (uint8_t addr = addr_from; addr <= addr_to; addr++) {
        kx_mb_request_t req = {
            .slave_addr = addr,
            .function   = MB_FC_READ_HOLDING_REGS,
            .reg_addr   = 0,
            .quantity   = 1,
        };
        kx_mb_response_t resp;
        esp_err_t err = kx_modbus_transaction(&req, &resp);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  ✓ esclavo encontrado: 0x%02X (%d)", addr, addr);
            found++;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "  Total: %d dispositivos encontrados", found);
    ESP_LOGI(TAG, "══════════════════════════════════════════");
}

// ─────────────────────────────────────────────────────────────
// Callback de polling — llamado por kx_param_store_foreach
// ─────────────────────────────────────────────────────────────

// Estructura de contexto para el polling con seguimiento de tiempo
typedef struct {
    int64_t *last_poll_us;   // array[param_id] de timestamps en µs
    int      max_params;
} poll_ctx_t;

static void _poll_param(int control_id, const kx_param_t *param,
                         void *user_data)
{
    // saltar params ocultos o sin lectura
    if (param->view == 0) return;
    if (param->function_read == 0) return;

    uint8_t  fc      = (uint8_t)param->function_read;
    uint16_t reg     = (uint16_t)param->reg;
    float    scale   = (param->offset != 0.0f) ? param->offset : 1.0f;
    float    add     = param->addition;
    uint16_t mask    = (uint16_t)param->mask;

    // detectar tipo de longitud: "FLOAT" o "DWORD" = 2 registros
    bool is_32bit = (param->length[0] == 'F' ||   // FLOAT
                     param->length[0] == 'D' ||   // DWORD
                     param->length[0] == '2');     // "2"

    float value = 0.0f;
    esp_err_t err;

    if (is_32bit) {
        err = kx_modbus_read_reg32((uint8_t)control_id, fc, reg,
                                    &value, scale, add);
    } else {
        err = kx_modbus_read_reg((uint8_t)control_id, fc, reg,
                                  &value, scale, add, mask);
    }

    if (err == ESP_OK) {
        kx_param_pub_status(control_id, param->param_id, value);
        ESP_LOGI(TAG, "→ OK  ctrl=%d param=%d [%s] = %.3f",
                 control_id, param->param_id, param->name, value);
    } else {
        kx_param_pub_error(control_id, param->param_id, "mb_timeout");
        ESP_LOGW(TAG, "→ ERR ctrl=%d param=%d [%s] (slave=0x%02X fc=%d reg=0x%04X)",
                 control_id, param->param_id, param->name,
                 (uint8_t)control_id, fc, reg);
    }

    // gap mínimo inter-trama (≥ 3.5 char times @ baud)
    vTaskDelay(pdMS_TO_TICKS(KX_MODBUS_INTER_CHAR_MS));
}

// ─────────────────────────────────────────────────────────────
// Tarea de polling principal
// ─────────────────────────────────────────────────────────────
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "tarea iniciada — esperando param store...");

    // Esperar a que todos los controles tengan entities
    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    int n_controls = kx_param_store_count();
    ESP_LOGI(TAG, "param store listo: %d controles — iniciando polling Modbus",
             n_controls);

    // margen inicial para que el backend procese antes de recibir telemetría
    vTaskDelay(pdMS_TO_TICKS(2000));

    uint32_t cycle = 0;

    while (1) {
        if (!kx_mqtt_is_connected()) {
            ESP_LOGD(TAG, "skip: MQTT no conectado");
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        int64_t t_start = esp_timer_get_time();

        ESP_LOGI(TAG, "━━ ciclo #%lu — %d controles ━━━━━━━━━━━━━━",
                 (unsigned long)cycle, n_controls);

        kx_param_store_foreach(_poll_param, NULL);

        int64_t t_elapsed_ms = (esp_timer_get_time() - t_start) / 1000;
        ESP_LOGI(TAG, "━━ ciclo #%lu completado en %lld ms — heap=%" PRIu32,
                 (unsigned long)cycle, t_elapsed_ms,
                 kx_system_heap_free());

        cycle++;

        // ajustar el delay restando el tiempo del ciclo
        int32_t delay_ms = (int32_t)(KX_TELEMETRY_INTERVAL_S * 1000) -
                           (int32_t)t_elapsed_ms;
        if (delay_ms < 100) delay_ms = 100;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// ─────────────────────────────────────────────────────────────
// API pública: arranque
// ─────────────────────────────────────────────────────────────
esp_err_t kx_modbus_start(void)
{
    kx_param_store_init();

    esp_err_t err = kx_modbus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init UART fallido: %s", esp_err_to_name(err));
        return err;
    }

    // ═════════════════════════════════════════════════════════
    // INICIO DEL TEST DE HARDWARE (Emerson XC660D)
    // ═════════════════════════════════════════════════════════
    ESP_LOGW(TAG, "⚠️ ATENCION: Ejecutando test de hardware Modbus...");
    
    // 1. Escanea las direcciones de la 1 a la 5 (para ir rápido y confirmar el ID 2)
    kx_modbus_scan(1, 5);
    
    // 2. Test de lectura directa al esclavo ID 2.
    // Usamos FC3 (Read Holding Registers), leemos la dirección 0x0000, 1 registro.
    kx_modbus_test_read(2, MB_FC_READ_HOLDING_REGS, 0x0000, 1);
    
    ESP_LOGW(TAG, "⚠️ Fin del test de hardware. Arrancando tarea normal...");
    // ═════════════════════════════════════════════════════════

    BaseType_t ret = xTaskCreate(
        _modbus_task,
        "kx_modbus",
        8192,
        NULL,
        KX_TASK_PRIO_TELEMETRY,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate fallido");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "componente Modbus RTU Master arrancado");
    return ESP_OK;
}