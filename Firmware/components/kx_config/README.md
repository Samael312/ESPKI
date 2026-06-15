# kx_config

Recepción, parseo y orquestación de la configuración del dispositivo vía MQTT. Gestiona el ciclo de descubrimiento de controles y entidades.

## Flujo completo de configuración

```
1. MQTT CONNECTED
   └── kx_mqtt publica device-status online

2. Bridge → +/{UUID}  (device.json)
   └── kx_config_handle()  tipo "device"
       ├── valida presencia de campo "uuid"
       ├── envía ACK
       └── publica controls-discovery → {UUID}/controls

3. Bridge → +/{UUID}/controls  (controls.json)
   └── kx_config_handle()  tipo "controls_list"
       ├── kx_mqtt_resize_queue(N)
       ├── kx_param_store_set_expected(N)
       └── por cada control:
           ├── parsea slave_addr, uuid, update_ts, protocolo (RTU/TCP)
           ├── kx_param_store_set_*
           ├── kx_modbus_master_ensure_started() o kx_modbus_tcp_ensure_started()
           ├── publica control-status online
           └── si update_ts_incoming > update_ts_stored:
               ├── pausa driver(s)
               ├── kx_param_store_clear_entities()
               ├── kx_param_store_set_update_ts()
               ├── reanuda driver(s)
               └── publica entities-discovery → {UUID}/controls/{id}/entities
               
4. Bridge → +/{UUID}/controls/+/entities
   └── kx_config_handle()  tipo "entities"
       ├── kx_param_store_parse(payload, len, control_id)
       ├── si kx_param_store_is_ready():
       │   └── kx_param_store_save_nvs()
       └── envía ACK
```

## API pública

```c
// Punto de entrada principal, llamado desde el router MQTT (_on_mqtt_message)
void kx_config_handle(const char *topic, const char *payload, size_t len);

// Publica controls-discovery al broker. También lo llama internamente
// tras recibir el device.json.
void kx_config_request_controls(void);
```

## Tipos de mensaje reconocidos (por topic)

| Topic pattern | Tipo interno |
|---|---|
| `+/{UUID}` | `device` |
| `+/{UUID}/controls` | `controls_list` |
| `+/{UUID}/controls/+` | `control_single` |
| `+/{UUID}/controls/+/entities` | `entities` |

## Parseo del protocolo de un control

El campo `metadata.protocol.metadata` del JSON de control determina el transporte:

```json
{
  "metadata": {
    "protocol": {
      "metadata": {
        "active": "tcp",
        "tcp": { "ip": "172.17.123.250", "port": 502 }
      }
    }
  }
}
```

- `active = "rtu"` (o ausente) → llama a `kx_modbus_master_ensure_started()`
- `active = "tcp"` → extrae IP y puerto, llama a `kx_modbus_tcp_ensure_started()`

## Variantes de controls.json soportadas

`kx_config_handler` reconoce cuatro estructuras posibles del JSON recibido:

1. `{"controls": [...]}` — array de controles bajo clave `controls`
2. `[...]` — array en la raíz
3. `{"control": {...}}` — un único control
4. Objeto raíz que ES el control (tiene `control_id` o `id`)

## Parseo de `slave_addr`

Se intenta leer la dirección del esclavo de estas claves en orden:
`control_address` · `slave_addr` · `modbus_address` · `address` · `rtu_address`

## Topics de respuesta

| Topic | Descripción |
|---|---|
| `{PREFIX}/{UUID}/config/ack` | ACK de configuración recibida |
| `{PREFIX}/{UUID}/config/error` | Error de parseo con código y detalle |
| `{UUID}/controls/{id}/status` | Estado online de un control |

## Gestión de update_ts

Cada control tiene un `update_ts` (timestamp de última modificación de sus entidades). Al recibir el controls.json:

- Si `ts_incoming > ts_stored` → se borran las entidades actuales y se solicitan de nuevo
- Si `ts_incoming == 0` → se fuerza redescubrimiento siempre
- Si `ts_incoming <= ts_stored` → se usan las entidades en caché (NVS o memoria)

Esto permite actualizaciones parciales sin descargar todos los controles.

## Dependencias (IDF)

`kx_system` · `kx_mqtt` · `kx_param_store` · `kx_modbus` · `kx_modbus_tcp` · `freertos` · `esp_timer` · `json` (cJSON)