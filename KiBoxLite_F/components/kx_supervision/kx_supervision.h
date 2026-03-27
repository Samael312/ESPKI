#pragma once
#include "esp_err.h"

// =============================================================
// kx_supervision.h  —  Tarea de supervisión y watchdog
// Monitoriza el estado del sistema cada 10 s y alimenta el WDT.
// =============================================================

esp_err_t kx_supervision_start(void);
