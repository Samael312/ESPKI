#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

esp_err_t kx_telemetry_start(void);

// ── Publicaciones salientes (device → broker) ────────────────
void kx_param_pub_status(int control_id, int param_id, float value);
void kx_param_pub_report(int control_id, int param_id, float value);
void kx_param_pub_error(int control_id, int param_id, const char *msg, uint16_t reg);
void kx_control_pub_status(int control_id, const char *uuid, const char *connection_status);

void kx_param_handle_set(const char *topic, const char *payload, size_t len);