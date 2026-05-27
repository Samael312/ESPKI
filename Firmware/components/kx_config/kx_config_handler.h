#pragma once
#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

// =============================================================
// kx_config_handler.h  —  Recepción y parseo de configuración
//
// Flujo completo:
//
//   1. MQTT CONNECTED
//        → _publish_device_status_online()          [kx_mqtt]
//
//   2. Bridge → +/{uuid}  (device.json)
//        → kx_config_handle() tipo "device"
//        → valida uuid
//        → envía ACK
//        → lanza controls
//
//   3. Bridge → +/{uuid}/controls  (controls.json)
//        → kx_config_handle() tipo "controls_list"
//        → por cada control:
//            · lee update_ts del JSON
//            · compara con update_ts almacenado en NVS
//            · si entrante > almacenado  → pausa Modbus,
//              borra entities del control, actualiza ts,
//              lanza entities-discovery, reanuda Modbus
//            · si entrante <= almacenado → skip discovery
//        → envía ACK
//
//   4. Bridge → +/{uuid}/controls/+/entities
//        → kx_config_handle() tipo "entities"
//        → parsea y hashea entities del control
//        → cuando todos los controles tienen entities_ready
//          → guarda NVS
//        → envía ACK
//
// Topics gestionados:
//   +/{uuid}                          → device
//   +/{uuid}/controls                 → controls_list
//   +/{uuid}/controls/+               → control_single
//   +/{uuid}/controls/+/entities      → entities
// =============================================================

// Punto de entrada principal, llamado desde el router MQTT.
void kx_config_handle(const char *topic, const char *payload, size_t len);

// Lanza el controls activamente (publica al broker).
// Llamado internamente tras recibir el device.json, pero también
// puede invocarse desde main si se quiere forzar la descarga.
void kx_config_request_controls(void);
void kx_param_store_print_active_samplings(void);