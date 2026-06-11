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

// Arranca el driver RTU solo si aún no está corriendo.
// Seguro llamarlo múltiples veces desde kx_config_handler.
esp_err_t kx_modbus_master_ensure_started(void);

void      kx_modbus_pause(void);
void      kx_modbus_resume(void);

esp_err_t kx_modbus_read_one(int control_id, int param_id);
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value);
esp_err_t kx_modbus_enqueue_write(int control_id, int param_id,
                                   float value, double ts);
void kx_modbus_request_poll(int param_id);