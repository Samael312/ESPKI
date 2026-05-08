#pragma once
#include "esp_err.h"
#include "kx_param_store.h"

// =============================================================
// kx_modbus.h  —  Driver Modbus RTU master
//
// Lee cada param según function_read + reg almacenados en
// kx_param_store, aplica offset/addition y publica en MQTT.
//
// Sustituye a kx_dummy_protocol en Fase 2.
// =============================================================

// Configuración del puerto serie (ajustar a tu hardware)
#define KX_MODBUS_UART_PORT     UART_NUM_1
#define KX_MODBUS_BAUD_RATE     9600
#define KX_MODBUS_DATA_BITS     UART_DATA_8_BITS
#define KX_MODBUS_PARITY        UART_PARITY_DISABLE
#define KX_MODBUS_STOP_BITS     UART_STOP_BITS_1

// Pins RS-485 (ajustar a tu PCB)
#define KX_MODBUS_TXD_PIN       4
#define KX_MODBUS_RXD_PIN       36


// Timeout de respuesta Modbus (ms)
#define KX_MODBUS_TIMEOUT_MS    500

// Pausa entre lecturas de registros consecutivos (ms)
// Evita saturar el bus y el heap con publicaciones simultáneas.
#define KX_MODBUS_READ_DELAY_MS 20

// Inicia el driver Modbus y la tarea de polling.
// Debe llamarse después de kx_dummy_protocol_start (o en su lugar).
esp_err_t kx_modbus_start(void);

// Lee un registro Modbus de forma síncrona.
// slave_addr : dirección del esclavo (1-247)
// function   : código de función (3=holding, 4=input, 1=coil, 2=discrete)
// reg        : dirección del registro (0-based)
// out_value  : valor raw leído (uint16_t)
// Retorna ESP_OK si la lectura fue correcta.
esp_err_t kx_modbus_read_reg(uint8_t slave_addr,
                              uint8_t function,
                              uint16_t reg,
                              uint16_t *out_value);