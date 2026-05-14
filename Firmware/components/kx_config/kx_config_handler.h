#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

// =============================================================
// kx_config_handler.h  —  Recepción y parseo de configuración
//
// Topics gestionados:
//   +/{uuid}                                    → device
//   +/{uuid}/controls                           → controls_list
//   +/{uuid}/controls/+                         → control_single
//   +/{uuid}/controls/+/entities                → entities (initial download)
//   +/{uuid}/controls/+/entities/+/get          → entity_get (lectura puntual)
//   +/{uuid}/controls/+/entities                → entities_update (cambios)
// =============================================================

void kx_config_handle(const char *topic, const char *payload, size_t len);

void kx_config_set_entities_ready(bool ready);