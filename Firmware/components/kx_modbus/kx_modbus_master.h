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

// Escritura: encola el comando en s_write_queue y retorna
// inmediatamente. La writer task lo ejecuta en segundo plano
// tomando el mutex entre lecturas del batch poll.
esp_err_t kx_modbus_enqueue_write(int control_id, int param_id,
                                   float value, double ts);

// Encola una demanda de lectura.
// param_id > 0 → leer solo ese parámetro.
// param_id == 0 → ciclo completo.
void kx_modbus_request_poll(int param_id);