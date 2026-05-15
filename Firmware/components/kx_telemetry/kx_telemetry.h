#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

esp_err_t kx_telemetry_start(void);

// ── Publicaciones salientes (device → broker) ────────────────
void kx_param_pub_status(int control_id, int param_id, float value);
void kx_param_pub_report(int control_id, int param_id, float value);
void kx_param_pub_error_modbus(int control_id, int param_id, uint16_t reg, const char *msg);
void kx_control_pub_status(int control_id, const char *uuid, const char *connection_status);

// ── Recepción de órdenes de escritura (broker → device) ──────
//
// Llamar desde el router MQTT cuando el topic encaja con:
//   quiiot/{uuid}/entities/{id}set
//
// El segmento final tiene formato "{entity_id}set" (sin barra entre
// el número y "set"). La función:
//   1. Extrae el entity_id del último segmento del topic.
//   2. Verifica que el payload contenga "operation":"set".
//   3. Compara el "ts" entrante con el ts_set almacenado en el hash.
//   4. Si ts_nuevo > ts_almacenado: actualiza ts_set y lanza
//      la escritura Modbus al registro del param.
void kx_param_handle_set(const char *topic, const char *payload, size_t len);