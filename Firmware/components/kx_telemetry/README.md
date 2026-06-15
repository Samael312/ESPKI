# kx_telemetry

Publicaciones salientes de datos de proceso y manejo de escrituras entrantes. Actúa como capa de traducción entre los drivers Modbus y los topics MQTT de entidades.

## Responsabilidades

- Publicar valores de parámetros por MQTT (`status`, `report`, `error`)
- Publicar estado de un control (`control-status`)
- Recibir y validar comandos de escritura (`entity-set`) y enrutarlos al driver correcto (RTU o TCP)
- Tarea de heartbeat de telemetría (alive log cada 10 s con heap y RSSI)

## API pública

### Arranque

```c
esp_err_t kx_telemetry_start(void);
```

Lanza la tarea `kx_telemetry` (prioridad 2, stack 4096).

### Publicaciones (device → broker)

```c
// Valor actual de una entidad (bajo demanda o tras cambio)
void kx_param_pub_status(int control_id, int param_id, float value);

// Valor periódico de una entidad (según sampling)
void kx_param_pub_report(int control_id, int param_id, float value);

// Error de lectura/escritura Modbus
void kx_param_pub_error(int control_id, int param_id,
                        const char *msg, uint16_t reg);

// Estado de conexión de un control
void kx_control_pub_status(int control_id, const char *uuid,
                            const char *connection_status);
```

### Escrituras entrantes (broker → dispositivo)

```c
void kx_param_handle_set(const char *topic, const char *payload, size_t len);
```

## Topics publicados

| Función | Topic | QoS | Retain |
|---|---|---|---|
| `kx_param_pub_status` | `{UUID}/quiiot/entities/{param_id}/status` | 0 | no |
| `kx_param_pub_report` | `{UUID}/quiiot/entities/{param_id}/report` | 0 | no |
| `kx_param_pub_error` | `{UUID}/quiiot/entities/{param_id}/status` | 0 | no |
| `kx_control_pub_status` | `{UUID}/controls/{control_id}/status` | 1 | no |

## Payloads

### status / report
```json
{"id": 12345, "value": 23.500, "ts": 1718000000.123}
```

### error
```json
{"id": 12345, "error": true, "error_message": "modbus_timeout",
 "reg": "0x0042", "ts": 1718000000.456}
```

### control-status
```json
{
  "_type": "control-status",
  "id": 7,
  "uuid": "...",
  "connection_status": "online",
  "link": {"detected": "online"},
  "timestamp": 1718000000.789
}
```

## kx_param_handle_set — Flujo de escritura

```
MQTT → +/{UUID}/controls/.../entities/.../set
  → main.c: kx_param_handle_set(topic, payload, len)
      1. Parsear JSON: _type="entity-set", operation="set", id, value, ts
      2. Buscar control_id del entity_id en param_store (foreach)
      3. Filtrar timestamp duplicado: si ts_incoming <= ts_stored → ignorar
      4. Enrutar según protocolo del control:
         · KX_PROTO_TCP → kx_modbus_tcp_enqueue_write()
         · KX_PROTO_RTU → kx_modbus_enqueue_write()
```

### Payload esperado (`entity-set`)

```json
{
  "_type": "entity-set",
  "operation": "set",
  "id": 12345,
  "value": 42.0,
  "ts": 1718000000.123
}
```

Campos opcionales: `ts` (si ausente, no se filtra por duplicado).

## Tarea de heartbeat

```
kx_telemetry (prio=2, stack=4096)
  cada 10 s → ESP_LOGI heap / rssi / mqtt_connected
```

No publica por MQTT; es solo diagnóstico local por puerto serie.

## Dependencias (IDF)

`kx_system` · `kx_mqtt` · `kx_param_store` · `kx_modbus` · `kx_modbus_tcp` · `esp_wifi` · `esp_timer` · `freertos` · `json` (cJSON)