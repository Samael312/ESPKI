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

// Inicia el driver TCP (creas colas + 4 tareas).
// Si ya está corriendo devuelve ESP_OK sin hacer nada.
// Llamar solo desde kx_config_handler cuando se confirma
// que existe al menos un control TCP.
esp_err_t kx_modbus_tcp_start(void);

// Igual que start pero solo arranca si no está ya corriendo.
// Seguro llamarlo múltiples veces (idempotente).
esp_err_t kx_modbus_tcp_ensure_started(void);

// Para el driver.
void kx_modbus_tcp_stop(void);

// True si el driver está activo (colas y tareas creadas).
bool kx_modbus_tcp_is_running(void);

// ── Demanda de poll ───────────────────────────────────────────
// param_id == 0 → ciclo completo de todos los controles TCP
// param_id  > 0 → solo ese parámetro
void kx_modbus_tcp_request_poll(int param_id);

// ── Encolar escritura ─────────────────────────────────────────
esp_err_t kx_modbus_tcp_enqueue_write(int    control_id,
                                       int    param_id,
                                       float  value,
                                       double ts);

// ── Pausa / reanuda (para config reload) ─────────────────────
void kx_modbus_tcp_pause(void);
void kx_modbus_tcp_resume(void);