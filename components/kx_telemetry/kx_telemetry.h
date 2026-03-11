#pragma once
#include "esp_err.h"

// =============================================================
// kx_telemetry.h  —  Tarea de publicación periódica
// =============================================================

// Arranca la tarea de telemetría (publica cada KX_TELEMETRY_INTERVAL_S)
esp_err_t kx_telemetry_start(void);
