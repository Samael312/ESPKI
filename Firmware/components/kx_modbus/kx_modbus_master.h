#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_master.h — Driver Modbus RTU Master
// Hardware: MAX13487E auto-direction RS-485
//           RO  → GPIO36  (receiver output   → UART RX, input-only)
//           DI  → GPIO4   (driver input       → UART TX)
//           RE/ → tied GND  (receiver always enabled)
//           SHDN/ → Q3 transistor → controlado externamente (HIGH = active)
//
// El MAX13487E gestiona la dirección automáticamente:
//   · mientras DI está en MARK (idle TX) → receiver habilitado
//   · mientras DI cambia (TX activo)     → driver habilitado
// No se necesita GPIO adicional de DE/RE.
// =============================================================

// ── Hardware ──────────────────────────────────────────────────
#define KX_MODBUS_UART_NUM   UART_NUM_2
#define KX_MODBUS_PIN_TX     4     // DI  del MAX13487E
#define KX_MODBUS_PIN_RX     36    // RO  del MAX13487E (GPIO input-only)

// ── Protocolo ─────────────────────────────────────────────────
#define KX_MODBUS_BAUD              9600
#define KX_MODBUS_RESP_TIMEOUT_MS   1000   // ms esperando 1er byte de respuesta
#define KX_MODBUS_INTER_CHAR_MS     5      // timeout inter-carácter (>3.5 chars)
#define KX_MODBUS_RETRY_COUNT       3
#define KX_MODBUS_TX_FLUSH_DELAY_MS 3      // margen tras uart_wait_tx_done

// ── Códigos de función Modbus RTU ─────────────────────────────
#define MB_FC_READ_COILS            0x01
#define MB_FC_READ_DISCRETE_INPUTS  0x02
#define MB_FC_READ_HOLDING_REGS     0x03
#define MB_FC_READ_INPUT_REGS       0x04
#define MB_FC_WRITE_SINGLE_REG      0x06
#define MB_FC_WRITE_MULTIPLE_REGS   0x10

// ── Tipos ─────────────────────────────────────────────────────
typedef struct {
    uint8_t  slave_addr;   // 1–247
    uint8_t  function;     // MB_FC_*
    uint16_t reg_addr;     // dirección del primer registro
    uint16_t quantity;     // nº de registros/bobinas
} kx_mb_request_t;

typedef struct {
    bool     ok;
    uint8_t  function;
    uint8_t  exception_code;   // válido cuando ok == false y hubo excepción
    uint8_t  data[252];        // bytes de datos sin CRC
    uint8_t  data_len;
} kx_mb_response_t;

// ── API ───────────────────────────────────────────────────────

// Inicializa el periférico UART para Modbus RTU.
// Llamar antes de cualquier otra función.
esp_err_t kx_modbus_init(void);

// Ejecuta una transacción RTU completa (request + response) con reintentos.
esp_err_t kx_modbus_transaction(const kx_mb_request_t *req,
                                 kx_mb_response_t *resp);

// Lee un registro de 16 bits, aplica offset/addition/mask y devuelve float.
// slave  = dirección Modbus (= control_id de kx_param_store)
// fc     = función de lectura (MB_FC_READ_HOLDING_REGS, etc.)
// reg    = dirección del registro
esp_err_t kx_modbus_read_reg(uint8_t slave, uint8_t fc, uint16_t reg,
                               float *out_value,
                               float scale, float addition, uint16_t mask);

// Lee registros de 32 bits / IEEE-754 float (2 registros consecutivos).
esp_err_t kx_modbus_read_reg32(uint8_t slave, uint8_t fc, uint16_t reg,
                                float *out_value,
                                float scale, float addition);

// Escribe un registro de 16 bits (FC06).
esp_err_t kx_modbus_write_reg(uint8_t slave, uint16_t reg, uint16_t value);

// Arranca la tarea de polling (reemplaza kx_dummy_protocol_start).
// Internamente llama a kx_param_store_init() y kx_modbus_init().
esp_err_t kx_modbus_start(void);

// ── Test ──────────────────────────────────────────────────────
// Envía una lectura manual y vuelca la respuesta raw en el log.
// Útil para verificar comunicación con un equipo real.
void kx_modbus_test_read(uint8_t slave, uint8_t fc,
                          uint16_t reg, uint16_t quantity);

// Escanea esclavos en el rango [addr_from, addr_to] con FC03 reg 0.
// Útil para descubrir dispositivos en el bus.
void kx_modbus_scan(uint8_t addr_from, uint8_t addr_to);