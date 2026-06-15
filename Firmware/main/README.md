# Kiconex Box Lite — Firmware

Firmware para ESP32-WROVER-IE que actúa como gateway IoT: lee registros Modbus (RTU y TCP) y publica sus valores por MQTT-TLS hacia la plataforma Kiconex.

## Arquitectura general

```
app_main
├── kx_system_init()           — NVS, device_id, boot_count
├── _try_load_from_nvs()       — caché de controles/entidades
├── _wifi_init_sta()            — WiFi STA
├── _ntp_init()                 — sincronización NTP
├── kx_mqtt_start()             — cliente MQTT-TLS + tarea de procesamiento
├── kx_telemetry_start()        — tarea heartbeat + publicaciones
│
│   [driver RTU — arranca bajo demanda desde kx_config_handler]
│   kx_modbus_master_ensure_started()
│
│   [driver TCP — arranca bajo demanda desde kx_config_handler]
│   kx_modbus_tcp_ensure_started()
│
└── loop diagnóstico (10 s)
```

## Componentes

| Componente | Descripción |
|---|---|
| [`kx_system`](components/kx_system/) | Estado global: NVS, device_id, uptime, heap, estados net/MQTT |
| [`kx_mqtt`](components/kx_mqtt/) | Cliente MQTT-TLS con LWT, reconexión automática y cola de mensajes |
| [`kx_config`](components/kx_config/) | Recepción y parseo de configuración (controles y entidades) vía MQTT |
| [`kx_param_store`](components/kx_param_store/) | Almacén en RAM (hash 3 niveles) con persistencia NVS |
| [`kx_modbus`](components/kx_modbus/) | Driver Modbus RTU: UART, packetizer, tareas de poll/report/escritura |
| [`kx_modbus_tcp`](components/kx_modbus_tcp/) | Driver Modbus TCP: sockets lwIP, mismas tareas que RTU |
| [`kx_telemetry`](components/kx_telemetry/) | Publicaciones MQTT de valores y manejo de escrituras (`entity-set`) |
| [`kx_supervision`](components/kx_supervision/) | Watchdog de tareas + log de salud cada 10 s |

## Flujo de datos completo

```
Broker MQTT
  │
  ├──[device.json]──────────────────────────────────────────────┐
  │                                                             ↓
  ├──[controls.json]──► kx_config_handler                kx_config_handler
  │                         ├── kx_param_store_set_*()       └── kx_config_request_controls()
  │                         ├── ensure_started() RTU/TCP
  │                         └── entities-discovery ──────────────┐
  │                                                              ↓
  ├──[entities.json]──► kx_param_store_parse()          controles y parámetros
  │                         └── kx_param_store_save_nvs()   en memoria (PSRAM)
  │
  ├──[entities/get]───► kx_modbus_request_poll(param_id)
  │                         └── demand_queue → _modbus_task → Modbus RTU/TCP
  │                                                ↓
  │                                     kx_param_pub_status()
  │                                                ↓
  └──────────────────────────────────── Broker MQTT (entities/{id}/status)

  ├──[entities/set]───► kx_param_handle_set()
  │                         └── enqueue_write() → writer_task → Modbus RTU/TCP
  │
  └── [periódico]      _report_task (tick 1s, filtra por sampling)
                           └── kx_param_pub_report()
```

## Protocolo dual RTU / TCP

El sistema soporta controles RTU y TCP de forma simultánea. El protocolo de cada control se determina al recibir el `controls.json`:

```
metadata.protocol.metadata.active = "rtu" | "tcp"
```

- Los drivers se arrancan **bajo demanda** (solo si hay controles de ese tipo)
- El enrutamiento en `kx_param_handle_set` y `_on_mqtt_message` consulta `kx_param_store_get_proto(ctrl_id)`

## Caché NVS

Al arrancar, el firmware intenta cargar los controles y entidades desde la partición NVS `storage`. Si el magic y el UUID coinciden, arranca sin esperar la descarga por MQTT.

```
nvs_valid() → load_nvs() → set_expected(N) → [drivers arrancan al procesar controls.json]
```

Si la caché falla o no existe, la descarga se realiza tras la conexión MQTT.

## Hardware objetivo

- **SoC**: ESP32 (Xtensa LX6 dual-core)
- **Módulo**: ESP32-WROVER-IE (4 MB Flash + 8 MB PSRAM)
- **RS485**: UART1, TX=GPIO4, RX=GPIO36, 9600 bps
- **Red**: WiFi 802.11 b/g/n

## Particiones

```
# partitions.csv
nvs,      data, nvs,      0x9000,   0x6000   — NVS sistema (boot_count, WiFi…)
phy_init, data, phy,      0xf000,   0x1000   — calibración RF
factory,  app,  factory,  0x10000,  0x300000 — firmware (3 MB)
storage,  data, nvs,      0x310000, 0x80000  — caché de entidades (512 KB)
```

## Construcción

```bash
# Con ESP-IDF instalado:
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor

# O con el devcontainer:
docker compose up
idf.py build
```

## Configuración rápida (`main/kx_config.h`)

```c
#define KX_WIFI_SSID          "mi_red"
#define KX_WIFI_PASSWORD      "mi_password"
#define KX_MQTT_BROKER_URI    "mqtts://broker.ejemplo.com:28883"
#define KX_MQTT_USERNAME      "usuario"
#define KX_MQTT_PASSWORD      "password"
#define KX_DEVICE_UUID        "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
```

## Variables clave de `sdkconfig.defaults`

| Variable | Valor | Descripción |
|---|---|---|
| `CONFIG_SPIRAM=y` | — | Habilita PSRAM |
| `CONFIG_SPIRAM_USE_MALLOC=y` | — | `malloc` usa PSRAM por defecto |
| `CONFIG_MQTT_TASK_STACK_SIZE` | 16384 | Stack de la tarea MQTT interna |
| `CONFIG_MQTT_BUFFER_SIZE` | 8192 | Buffer de fragmentación MQTT |
| `CONFIG_ESP_TASK_WDT_TIMEOUT_S` | 120 | Timeout del watchdog de tareas |
| `CONFIG_FREERTOS_HZ` | 100 | Tick del RTOS (10 ms/tick) |

## Versión de firmware

```c
#define KX_FW_VERSION  "0.2.0"
```