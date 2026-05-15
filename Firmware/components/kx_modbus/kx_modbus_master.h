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
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value);