#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_master.h  —  Driver Modbus RTU maestro
//
// Sustituye a kx_dummy_protocol en Fase 2.
// Lee los registros de cada kx_param_t usando function_read,
// aplica offset/addition, y publica el valor en MQTT con el
// mismo esquema de topics que kx_dummy_protocol:
//
//   {uuid}/quiiot/entities/{param_id}/report   (lectura)
//   {uuid}/quiiot/entities/{param_id}/status   (lectura)
//   {uuid}/quiiot/entities/{param_id}/set      (solo escritura)
//
// Configuración de UART/RS-485 en kx_config.h:
//   KX_MODBUS_UART_NUM, KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN,
//   KX_MODBUS_RTS_PIN,  KX_MODBUS_BAUD
// =============================================================

// Arranca la tarea Modbus maestro.
// Debe llamarse después de kx_param_store_init() o cuando las
// entities ya estén disponibles (NVS o descarga MQTT).
esp_err_t kx_modbus_master_start(void);

// Detiene la tarea y cierra el puerto UART.
void kx_modbus_master_stop(void);

// True si la tarea está activa.
bool kx_modbus_master_is_running(void);