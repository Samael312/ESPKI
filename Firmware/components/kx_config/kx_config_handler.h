#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

// =============================================================
// kx_config_handler.h  —  Recepción y parseo de configuración
// =============================================================

// Procesa un mensaje de configuración recibido por MQTT.
void kx_config_handle(const char *topic, const char *payload, size_t len);

// Informa al handler que las entities ya están disponibles (cargadas
// desde NVS), para que no vuelva a solicitarlas al recibir /controls.
void kx_config_set_entities_ready(bool ready);