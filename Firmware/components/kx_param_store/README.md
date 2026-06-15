# kx_param_store

Almacén en memoria con hash de tres niveles para controles, parámetros Modbus y caché de valores de registros. Soporta persistencia en NVS y transporte RTU/TCP.

## Arquitectura: hash de tres niveles

```
kx_ctrl_hash_t  (nivel 1 — control)
└── kx_control_t
    ├── kx_param_hash_t  (nivel 2 — parámetro/entidad)
    │   └── kx_param_t   { param_id, reg, fc_read, offset, sampling… }
    └── kx_reg_hash_t    (nivel 3 — caché de registro Modbus)
        └── kx_reg_entry_t { reg, fc_read, value, ts_last_read, last_write_value… }
```

Toda la memoria se aloja preferentemente en **PSRAM** con fallback a RAM interna (`_psram_alloc`).

## Flujo de uso típico

```
kx_param_store_init()
  └── _nvs_storage_init()          // inicializa partición "storage"

kx_param_store_set_expected(N)     // cuántos controles se esperan

// Por cada control recibido por MQTT:
kx_param_store_set_slave_addr(ctrl_id, addr)
kx_param_store_set_uuid(ctrl_id, uuid)
kx_param_store_set_tcp_endpoint(ctrl_id, ip, port)  // solo si TCP
kx_param_store_set_update_ts(ctrl_id, ts)
kx_param_store_parse(payload, len, ctrl_id)         // rellena nivel 2 y 3

// Cuando todos están listos:
kx_param_store_is_ready()    → true
kx_param_store_save_nvs()    // persiste en partición "storage"
```

## API pública

### Ciclo de vida

```c
void      kx_param_store_init(void);
esp_err_t kx_param_store_parse(const char *payload, size_t len, int control_id);
```

### Consulta — nivel control (1)

```c
const kx_control_t *kx_param_store_get_ctrl(int control_id);
int                 kx_param_store_count(void);
kx_proto_t          kx_param_store_get_proto(int control_id);
```

### Consulta — nivel parámetro (2)

```c
const kx_param_t *kx_param_store_get_param(int control_id, int param_id);
kx_param_t       *kx_param_store_get_param_mutable(int control_id, int param_id);
void              kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data);
```

### Caché de registros Modbus (3)

```c
kx_reg_entry_t *kx_param_store_reg_upsert_read(int control_id, uint16_t reg,
                                                uint8_t fc_read, uint8_t fc_write,
                                                float value, int64_t ts_ms);
esp_err_t       kx_param_store_reg_upsert_write(...);
const kx_reg_entry_t *kx_param_store_reg_get(...);
void            kx_param_store_reg_foreach(int control_id, kx_reg_iter_cb_t cb, void *ud);
void            kx_param_store_reg_foreach_all(kx_reg_iter_cb_t cb, void *ud);
```

### Configuración por control

```c
void kx_param_store_set_slave_addr(int control_id, int slave_addr);
void kx_param_store_set_uuid(int control_id, const char *uuid);
void kx_param_store_set_update_ts(int control_id, double ts);
void kx_param_store_clear_entities(int control_id);
void kx_param_store_set_tcp_endpoint(int control_id, const char *ip, uint16_t port);
esp_err_t kx_param_store_get_tcp_endpoint(int control_id, char *ip_out, uint16_t *port_out);
```

### Completitud

```c
void kx_param_store_set_expected(int count);
bool kx_param_store_is_ready(void);   // true cuando todos los controles tienen entities_ready
void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb);
```

### Persistencia NVS

```c
esp_err_t kx_param_store_save_nvs(void);
esp_err_t kx_param_store_load_nvs(void);
esp_err_t kx_param_store_clear_nvs(void);
bool      kx_param_store_nvs_valid(void);  // comprueba magic + UUID
```

## Estructura `kx_param_t`

| Campo | Descripción |
|---|---|
| `param_id` | ID único de la entidad |
| `reg` | Dirección del registro Modbus |
| `function_read` / `function_write` | FC Modbus (0x01–0x10) |
| `offset` / `addition` | Transformación: `value = raw * offset + addition` |
| `minvalue` / `maxvalue` | Clamp del valor resultante |
| `sampling` | Periodo de report en segundos (0 = sin report) |
| `view` | 0 = no publicar; 1 = publicar |
| `ts_last_read` | Timestamp (ms) de la última lectura |
| `last_published_value` | Último valor publicado (FLT_MAX = sin valor) |
| `ts_set` | Timestamp del último `set` recibido |

## Protocolo de transporte

```c
typedef enum {
    KX_PROTO_RTU = 0,  // Modbus RTU sobre UART/RS485 (por defecto)
    KX_PROTO_TCP,      // Modbus TCP sobre socket lwIP
} kx_proto_t;
```

El campo `slave_addr` actúa como dirección RS485 en RTU y como `unit_id` del MBAP en TCP.

## Persistencia NVS

- **Partición**: `storage` (NVS separada de la partición `nvs` del sistema)
- **Namespace**: `kx_entities`
- **Magic**: `0xE5712A05` — se invalida al cambiar la estructura de la cabecera
- **Validación**: magic + UUID del dispositivo deben coincidir
- Los parámetros se guardan en chunks de ≤ 3840 bytes para respetar el límite de blob NVS

## Límites

| Constante | Valor |
|---|---|
| `KX_PARAM_MAX_CONTROLS` | 16 controles |
| `KX_PARAM_MAX_PER_CONTROL` | 500 parámetros por control |
| `KX_CTRL_HASH_BUCKETS` | 16 |
| `KX_PARAM_HASH_BUCKETS` | 64 |
| `KX_REG_HASH_BUCKETS` | 128 |

## Dependencias (IDF)

`esp_timer` · `freertos` · `kx_modbus_tcp` · `heap` · `nvs_flash` · `json` (cJSON)