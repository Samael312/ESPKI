# kx_supervision

Tarea de supervisión del sistema: alimenta el watchdog de tareas (Task WDT) y vuelca métricas de salud cada 10 segundos.

## Responsabilidades

- Registrarse en el Task Watchdog de ESP-IDF y hacer reset cada 4 s
- Loggear métricas de salud periódicamente: uptime, heap libre, estado de red/MQTT, reconexiones
- Emitir `LOGW` si el heap cae por debajo del umbral de alerta

## Configuración

```c
#define WDT_FEED_MS          4000   // intervalo de reset del WDT
#define LOG_INTERVAL_MS     10000   // intervalo de log de salud
#define HEAP_WARN_THRESHOLD 30000   // bytes libres mínimos antes de warning
```

## Salida de log

```
health | uptime=120s heap=210345 net=2 mqtt=2 recon=0 elapsed=10001ms
```

| Campo | Fuente |
|---|---|
| `uptime` | `kx_system_uptime_s()` |
| `heap` | `kx_system_heap_free()` |
| `net` | `kx_system_net_state()` (0=disc, 1=conn, 2=connected) |
| `mqtt` | `kx_system_mqtt_state()` |
| `recon` | `kx_system_reconnect_count()` |

## API pública

```c
esp_err_t kx_supervision_start(void);
```

Crea la tarea `kx_supervision` con parámetros de `kx_config.h`:

```c
#define KX_TASK_STACK_SUPERVISION  3072
#define KX_TASK_PRIO_SUPERVISION   6
```

## Dependencias (IDF)

`kx_system` · `kx_mqtt` · `esp_system` · `freertos`

## Configuración del WDT (`sdkconfig.defaults`)

```ini
CONFIG_ESP_TASK_WDT=y
CONFIG_ESP_TASK_WDT_TIMEOUT_S=120
CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y
```

La tarea de supervisión debe hacer `esp_task_wdt_reset()` antes de que expire el timeout de 120 s. Con `WDT_FEED_MS = 4000`, hay un margen de 30× sobre el timeout.