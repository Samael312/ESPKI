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
//   · Lectura puntual : kx_modbus_read_one()
//   · Update on change: kx_modbus_update_changed()
//       Pausa el ciclo general, actualiza la hash con los params
//       modificados, los lee por Modbus y reanuda el polling.
// =============================================================

// Arranca la tarea de polling continuo.
esp_err_t kx_modbus_master_start(void);

// Detiene la tarea y cierra el puerto UART.
void kx_modbus_master_stop(void);

// True si la tarea de polling está activa.
bool kx_modbus_master_is_running(void);

// Lectura puntual bajo demanda (topic …/entities/N/get).
// Serializada con el ciclo general vía mutex UART.
esp_err_t kx_modbus_read_one(int control_id, int param_id);

// Llamado por kx_config_handler cuando llega un JSON de entities
// en el topic quiiot/{uuid}/controls/{control_id}/entities.
//
// Flujo interno:
//   1. Parsea el JSON y compara cada param contra la hash (O(1))
//   2. Acumula los param_id que difieren en algún campo
//   3. Pausa el ciclo general (limpia POLL_ALLOWED del EventGroup)
//   4. Actualiza la hash con los nuevos valores
//   5. Ejecuta kx_modbus_read_one() para cada param cambiado
//   6. Reanuda el ciclo (setea POLL_ALLOWED)
//
//   control_id : extraído del topic por kx_config_handler
//   payload    : JSON completo recibido por MQTT
//   len        : longitud del payload
esp_err_t kx_modbus_update_changed(int control_id,
                                    const char *payload,
                                    size_t len);