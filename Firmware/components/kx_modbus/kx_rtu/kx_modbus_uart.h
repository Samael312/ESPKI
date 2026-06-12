#pragma once
#include "esp_err.h"
#include "kx_param_store.h"
#include <stdint.h>
#include <stddef.h>
#include <float.h>

// =============================================================
// kx_modbus_uart.h — Capa de transporte Modbus RTU
//
// Sin cambios respecto a la versión original.
// =============================================================

// ── Timing ────────────────────────────────────────────────────
#define MODBUS_RESPONSE_TIMEOUT_MS    100
#define MODBUS_INTER_FRAME_MS          20
#define MODBUS_INTER_PARAM_MS          15
#define MODBUS_RETRY_COUNT              2

// ── Function codes ────────────────────────────────────────────
#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

// ── API ───────────────────────────────────────────────────────
esp_err_t kx_modbus_uart_init(void);
void      kx_modbus_uart_deinit(void);

int kx_modbus_transaction(const uint8_t *frame, size_t frame_len,
                           uint8_t *resp, size_t resp_max);

float kx_modbus_read_reg(uint8_t slave_addr, uint16_t reg_addr,
                          uint8_t fc, const kx_param_t *param,
                          uint16_t *out_raw);

int kx_modbus_read_regs_multi(uint8_t slave_addr, uint16_t start_reg,
                               uint16_t num_regs, uint8_t fc,
                               uint8_t *resp_buf, size_t resp_max);