#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_tcp.h — Cliente Modbus TCP sobre lwIP
//
// Las colas y tareas solo se crean cuando realmente existe
// un control TCP en el sistema. Usar kx_modbus_tcp_ensure_started()
// desde kx_config_handler al procesar un control con proto=tcp.
// =============================================================

esp_err_t kx_modbus_tcp_start(void);
esp_err_t kx_modbus_tcp_ensure_started(void);
void      kx_modbus_tcp_stop(void);
bool      kx_modbus_tcp_is_running(void);

// param_id == 0 → ciclo completo de todos los controles TCP
// param_id  > 0 → solo ese parámetro
void kx_modbus_tcp_request_poll(int param_id);

esp_err_t kx_modbus_tcp_enqueue_write(int    control_id,
                                       int    param_id,
                                       float  value,
                                       double ts);

void kx_modbus_tcp_pause(void);
void kx_modbus_tcp_resume(void);