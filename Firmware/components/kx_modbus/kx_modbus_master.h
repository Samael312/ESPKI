#pragma once
#include "esp_err.h"
#include "kx_param_store.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_master.h  —  Driver Modbus RTU maestro
// =============================================================

esp_err_t kx_modbus_master_start(void);
void      kx_modbus_master_stop(void);
bool      kx_modbus_master_is_running(void);

void      kx_modbus_pause(void);
void      kx_modbus_resume(void);

// Lectura puntual bajo demanda.
esp_err_t kx_modbus_read_one(int control_id, int param_id);

// =============================================================
// kx_modbus_write_one — Escritura puntual (FC 06)
//
// Escribe un valor en el registro Modbus asociado al param_id
// del control_id indicado.
//
// Transformación inversa a _read_register():
//   raw = (value - addition) / offset   si offset != 0 && offset != 1
//   raw = (value - addition)            en caso contrario
//
// El raw se clampea al rango [minvalue, maxvalue] antes de
// enviar la trama. La respuesta eco del esclavo (FC 06 devuelve
// la misma trama) se valida byte a byte.
//
// Retorna:
//   ESP_OK             — esclavo confirmó la escritura
//   ESP_ERR_NOT_FOUND  — param_id no existe en ese control
//   ESP_ERR_INVALID_STATE — slave_addr == 0
//   ESP_ERR_NOT_SUPPORTED — function_write no es FC05/06/10
//   ESP_FAIL           — sin respuesta o eco incorrecto
// =============================================================
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value);