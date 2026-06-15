# kx_system

Estado global del dispositivo: inicialización de NVS, identidad, uptime, heap, y estados de red/MQTT.

## Responsabilidades

- Inicializar NVS (con borrado y reintento automático si está corrupta)
- Derivar el `device_id` de la MAC WiFi (12 chars hex sin separadores)
- Persistir y leer el contador de arranques (`boot_count`) en NVS
- Exponer métricas de sistema: uptime, heap libre, motivo de reset
- Mantener el estado de red y MQTT como variables volátiles compartidas entre tareas
- Contar reconexiones MQTT

## API pública

```c
// Inicialización — llamar una sola vez desde app_main antes de cualquier otra cosa
esp_err_t kx_system_init(void);

// Identidad
const char *kx_system_device_id(void);     // MAC WiFi como string hex

// Métricas
uint32_t    kx_system_uptime_s(void);      // segundos desde boot
const char *kx_system_reset_reason(void);  // "power_on", "panic", "watchdog"…
uint32_t    kx_system_heap_free(void);     // bytes libres (MALLOC_CAP_DEFAULT)
uint32_t    kx_system_boot_count(void);    // arranques totales (persistido en NVS)

// Estado de red (escrito por kx_net / leído por cualquier tarea)
void           kx_system_set_net_state(kx_net_state_t s);
kx_net_state_t kx_system_net_state(void);

// Estado MQTT
void            kx_system_set_mqtt_state(kx_mqtt_state_t s);
kx_mqtt_state_t kx_system_mqtt_state(void);

// Contador de reconexiones MQTT
void     kx_system_inc_reconnect_count(void);
uint32_t kx_system_reconnect_count(void);
```

## Estados

```c
typedef enum { KX_NET_DISCONNECTED, KX_NET_CONNECTING, KX_NET_CONNECTED  } kx_net_state_t;
typedef enum { KX_MQTT_DISCONNECTED, KX_MQTT_CONNECTING, KX_MQTT_CONNECTED } kx_mqtt_state_t;
```

## Dependencias (IDF)

`nvs_flash` · `esp_system` · `esp_wifi` · `driver` · `freertos`

## NVS

- Namespace: `kx_sys`
- Clave: `boot_cnt` (u32)
- Partición: la partición NVS por defecto (`nvs` en `partitions.csv`)

## Notas

- `kx_system_heap_free()` usa `MALLOC_CAP_DEFAULT`, por lo que incluye PSRAM si `CONFIG_SPIRAM_USE_MALLOC=y`.
- El `device_id` se forma al arranque y no cambia durante la sesión.
- Los estados `net_state` y `mqtt_state` son `volatile` — escritura/lectura atómica para tipos de 32 bits en Xtensa, sin mutex adicional.