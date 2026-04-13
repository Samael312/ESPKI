#pragma once
#include "esp_err.h"
#include "kx_mqtt.h"

// =============================================================
// kx_dummy_protocol.h — Simulador de protocolo de campo
// En Fase 2 se sustituye por el driver Modbus real.
// =============================================================

// Arranca la tarea que simula las lecturas de campo
esp_err_t kx_dummy_protocol_start(void);
