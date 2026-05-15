#pragma once
#include "esp_err.h"
#include "kx_param_store.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_master.h  —  Driver Modbus RTU maestro
//
// Modos de operación:
//   · Ciclo completo  : _modbus_task() — foreach() periódico
//   · Pausa/reanuda   : kx_modbus_pause() / kx_modbus_resume()
//       Usado por kx_config_handler cuando llega controls.json
//       con update_ts mayor: pausa el ciclo, actualiza entities,
//       reanuda el ciclo.
//   · Lectura puntual : kx_modbus_read_one()
// =============================================================

// Arranca la tarea de polling continuo.
esp_err_t kx_modbus_master_start(void);

// Detiene la tarea y cierra el puerto UART.
void kx_modbus_master_stop(void);

// True si la tarea de polling está activa.
bool kx_modbus_master_is_running(void);

// Pausa el ciclo de polling (bloquea la iteración actual al
// completar el param en curso). Llamada desde kx_config_handler
// antes de borrar entities y relanzar discovery.
// Bloqueante hasta que el ciclo queda en espera.
void kx_modbus_pause(void);

// Reanuda el ciclo de polling tras kx_modbus_pause().
void kx_modbus_resume(void);

// Lectura puntual bajo demanda.
esp_err_t kx_modbus_read_one(int control_id, int param_id);
esp_err_t kx_modbus_write_one(int control_id, int param_id, float value);