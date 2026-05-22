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

// Llamar al recibir quiiot/{uuid}/entities/get.
//
// param_id > 0 → leer y publicar SOLO ese parámetro.
//               Si la misma petición se repite antes de
//               KX_DEMAND_REPEAT_MS ms, se re-encola con un
//               pequeño jitter para evitar que varias lleguen
//               al bus a la vez.
//
// param_id == 0 → ciclo completo (todos los params visibles).
//
// Las demandas se encolan; el _modbus_task las consume en orden.
void kx_modbus_request_poll(int param_id);