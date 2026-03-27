#pragma once
#include "esp_err.h"
#include <stddef.h>

// =============================================================
// kx_config_handler.h  —  Recepción y parseo de configuración
// En MVP: recibe, valida estructura básica, publica ack/error.
// En Fase 2: aplicar y persistir en NVS.
// =============================================================

// Procesa un mensaje de configuración recibido por MQTT.
// topic   = topic completo del mensaje
// payload = cuerpo JSON (null-terminated)
// len     = longitud del payload
void kx_config_handle(const char *topic, const char *payload, size_t len);
