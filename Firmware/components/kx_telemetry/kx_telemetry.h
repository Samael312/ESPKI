#pragma once
#include "esp_err.h"
#include <stdint.h>

esp_err_t kx_telemetry_start(void);

// Publicaciones de parámetros
void kx_param_pub_status(int control_id, int param_id, float value);
void kx_param_pub_report(int control_id, int param_id, float value);
void kx_param_pub_set(int control_id, int param_id, float value);
void kx_param_pub_error_modbus(int control_id, int param_id, uint16_t reg, const char *msg);

// Publicación de controles
void kx_control_pub_status(int control_id, const char *uuid, const char *connection_status);