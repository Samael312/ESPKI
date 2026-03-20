#pragma once
#include "esp_err.h"

esp_err_t kx_telemetry_start(void);

// Llamadas por el protocolo cuando tiene un valor listo
void kx_param_pub_status(int control_id, int param_id, float value);
void kx_param_pub_set(int control_id, int param_id, float value);
void kx_param_pub_error(int control_id, int param_id, const char *msg);
void kx_control_pub_status(int control_id, const char *uuid,
                            const char *connection_status);