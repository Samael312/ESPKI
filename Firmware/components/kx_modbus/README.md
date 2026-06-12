# `kx_modbus` — Driver Modbus RTU + TCP

Componente unificado de comunicación Modbus para el **Kiconex Box Lite** (ESP32-WROVER-IE, ESP-IDF v5.5.1). Gestiona la lectura periódica y bajo demanda de registros Modbus, escrituras remotas, y la publicación de resultados vía MQTT. Soporta dos transportes: **RTU** sobre RS-485/UART y **TCP** sobre lwIP.

---

## Estructura de archivos

```
components/kx_modbus/
├── CMakeLists.txt
│
├── kx_shared/                        # Código compartido RTU y TCP
│   ├── kx_modbus_shared.h            # Tipos comunes: colas, callbacks, bitmaps
│   ├── kx_modbus_packetizer.h/.c     # Agrupador de registros en packets
│   └── kx_modbus_publish.h/.c        # Fan-out de publicaciones MQTT
│
├── kx_rtu/                           # Driver Modbus RTU
│   ├── kx_modbus_master.h/.c         # API pública RTU + arranque de tareas
│   ├── kx_modbus_pipeline.h/.c       # Tarea demand RTU (kx_modbus_task)
│   ├── kx_modbus_report.h/.c         # Tarea report periódico RTU
│   ├── kx_modbus_writer.h/.c         # Tarea de escrituras RTU
│   ├── kx_modbus_dispatch.h/.c       # Ejecución de packets sobre UART
│   ├── kx_modbus_uart.h/.c           # Capa de transporte UART/RS-485
│   └── kx_modbus_legacy.c            # Código deshabilitado (#if 0)
│
└── kx_tcp/                           # Driver Modbus TCP
    ├── kx_modbus_tcp.h/.c            # API pública TCP + arranque de tareas
    ├── kx_modbus_tcp_pipeline.h/.c   # Tarea demand TCP
    ├── kx_modbus_tcp_report.h/.c     # Tarea report periódico TCP
    ├── kx_modbus_tcp_writer.h/.c     # Tarea de escrituras TCP
    ├── kx_modbus_tcp_dispatch.h/.c   # Ejecución de packets sobre socket
    └── kx_modbus_tcp_socket.h/.c     # Gestión de sockets lwIP persistentes
```

---

## Arquitectura general

Cada driver (RTU y TCP) es independiente y comparte solo la capa `kx_shared`. Ambos replican el mismo modelo de cuatro tareas FreeRTOS:

```
                        ┌─────────────────────────────────────────────┐
                        │              kx_shared                       │
                        │  packetizer  ·  publish  ·  shared types    │
                        └──────────────────┬──────────────────────────┘
                                           │
              ┌────────────────────────────┴────────────────────────────┐
              │                                                          │
   ┌──────────▼──────────┐                              ┌──────────────▼──────────┐
   │      kx_rtu          │                              │        kx_tcp            │
   │                      │                              │                          │
   │  kx_modbus_task      │  ←── demand_queue ───────►  │  kx_tcp_demand_task      │
   │  kx_report_task      │                              │  kx_tcp_report_task      │
   │  kx_writer_task      │  ←── write_queue  ───────►  │  kx_tcp_writer_task      │
   │  _publisher_task     │  ←── pub_queue    ───────►  │  _tcp_publisher_task     │
   │                      │                              │                          │
   │  UART / RS-485       │                              │  TCP socket / lwIP       │
   └──────────────────────┘                              └──────────────────────────┘
```

### Arranque bajo demanda

Ningún driver arranca en `app_main`. `kx_config_handler` llama a `kx_modbus_master_ensure_started()` o `kx_modbus_tcp_ensure_started()` la primera vez que recibe un control de ese protocolo en el `controls.json`. Esto evita reservar stacks y colas para un driver sin controles asignados.

---

## Capa compartida (`kx_shared`)

### `kx_modbus_shared.h`

Define los tipos que circulan entre tareas:

| Tipo | Descripción |
|---|---|
| `kx_pub_result_t` | Entrada de la cola de publicación (status, report o error) |
| `kx_poll_demand_t` | Solicitud de lectura bajo demanda (param_id + timestamp) |
| `kx_write_cmd_t` | Comando de escritura (control_id, param_id, value, ts) |
| `kx_pub_kind_t` | Enum: `PUB_KIND_STATUS`, `PUB_KIND_REPORT`, `PUB_KIND_ERROR` |

El bitmap `s_pending_bits[KX_PENDING_SET_SIZE/8]` deduplica demandas: si un `param_id` ya está en la `demand_queue`, no se vuelve a encolar hasta que la tarea demand lo procese y limpie el bit.

### `kx_modbus_packetizer` — Agrupador de registros

`kx_pkt_build()` transforma la lista plana de parámetros del control en una lista de **packets Modbus** optimizados. Un packet agrupa registros contiguos del mismo FC en una sola transacción multi-registro, reduciendo el número de tramas en el bus.

**Tres modos de construcción:**

| Modo | `demand_active` | `param_ids` | Criterio de inclusión |
|---|---|---|---|
| REPORT | `false` | `NULL` | `tick_s % param->sampling == 0` |
| DEMAND-FULL | `true` | `NULL` | Todos los params visibles con FC válido |
| DEMAND-SET | `true` | array | Solo los param_ids del array dado |

**Política de gaps:** si el rango de registros agrupados tiene huecos (registros sin parámetro asociado), el packetizer emite un packet individual por cada candidato en lugar de un multi con gaps. Esto evita excepciones Modbus en esclavos que no toleran leer registros inexistentes.

**Parámetro `KX_PKT_MAX_GAP`** (definido en `kx_modbus_packetizer.h`, por defecto `0`): número máximo de registros de hueco tolerados al agrupar. Con valor 0, solo se agrupan registros estrictamente contiguos.

**Parámetro `KX_PKT_MAX_REGS_PER_PKT`** (por defecto `3`): máximo de registros por packet multi.

### `kx_modbus_publish` — Fan-out de publicaciones

`kx_publish_all_params_for_reg()` es el punto central de publicación. Dado un registro leído (raw Modbus), busca **todos** los parámetros del control que comparten `(reg, fc_read)` y publica cada uno aplicando su `offset` y `addition` individual. Esto soporta el caso de múltiples entidades mapeadas al mismo registro físico.

En modo report (`tick_s >= 0`) filtra adicionalmente por `tick_s % param->sampling == 0`. Actualiza `ts_last_read` y `last_published_value` en `kx_param_store` para cada parámetro publicado.

---

## Driver RTU (`kx_rtu`)

### Tareas y prioridades

| Tarea FreeRTOS | Función | Prioridad |
|---|---|---|
| `kx_modbus` | Pipeline demand | `KX_TASK_PRIO_TELEMETRY + 1` |
| `kx_writer` | Escrituras | `KX_TASK_PRIO_TELEMETRY + 2` |
| `kx_report` | Reports periódicos | `KX_TASK_PRIO_TELEMETRY` |
| `kx_publisher` | Publicación MQTT | `KX_TASK_PRIO_TELEMETRY - 1` |

### Colas

| Cola | Tamaño | Propósito |
|---|---|---|
| `pub_queue` | 500 | Resultados de lectura pendientes de publicar |
| `demand_queue` | 1500 | Solicitudes de poll bajo demanda |
| `write_queue` | 64 | Comandos de escritura |

### `kx_modbus_task` — Pipeline demand RTU

Es la tarea principal. Espera el bit `DEMAND_BIT` en el event group y ejecuta un ciclo de recopilación de ráfaga antes de procesar:

1. Espera `DEMAND_BIT` (set por `kx_modbus_request_poll`).
2. **Fase burst:** sondea la cola cada 50 ms durante hasta 3 s, esperando estabilidad de 300 ms antes de proceder. Evita procesar demandas una a una cuando llegan en ráfaga.
3. Drena la `demand_queue` en un snapshot, descartando expiradas (`> KX_DEMAND_TIMEOUT_S`) y deduplicando.
4. Si alguna demanda tiene `param_id == 0` → **full cycle**: lee todos los parámetros visibles de todos los controles RTU.
5. Si no → **batch poll**: agrupa por `control_id`, construye packets con `kx_pkt_build()` y los ejecuta con `kx_dispatch_control_packets()`.

**Fallback individual:** si un parámetro no quedó cubierto por ningún packet (sin FC de lectura, write-only, etc.), se intenta una lectura individual directa bajo mutex.

**Serialización del bus:** todas las operaciones sobre el UART van protegidas por `foreach_mutex`. El event group con `POLL_ALLOWED_BIT` permite pausar el driver durante actualizaciones de configuración (`kx_modbus_pause()` / `kx_modbus_resume()`).

### `kx_report_task` — Reports periódicos RTU

Itera cada segundo. Incrementa `tick_s` módulo 864000 (10 días). En cada tick llama a `kx_pkt_build()` con `demand_active=false` para cada control: solo los parámetros cuyo `sampling` divide exactamente `tick_s` son incluidos. Los resultados se despachan con `kx_param_pub_report` como función de publicación.

### `kx_writer_task` — Escrituras RTU

Bloquea en `write_queue`. Al recibir un `kx_write_cmd_t`:

1. Espera `POLL_ALLOWED_BIT`.
2. Toma `foreach_mutex` para serializar con el bus.
3. Construye la trama Modbus (FC05, FC06 o FC10) y la envía con reintentos.
4. Si OK: actualiza `ts_set` en param_store y publica el nuevo valor como status.
5. Si falla: publica un error MQTT.

### `kx_modbus_dispatch` — Ejecución de packets RTU

`kx_dispatch_packet()` ejecuta un único packet sobre el UART:

- **Individual** (`num_regs == 1`): llama a `kx_modbus_read_reg()` directamente.
- **Multi** (`num_regs > 1`): llama a `kx_modbus_read_regs_multi()`. Si falla, hace fallback a lecturas individuales por cada slot no-gap.

Tras una lectura exitosa actualiza `kx_param_store` (nivel 3, caché de registros) y llama a `kx_publish_all_params_for_reg()`.

### `kx_modbus_uart` — Transporte RS-485

Encapsula el driver UART de ESP-IDF. Cada transacción:

1. Añade CRC16 Modbus al frame.
2. Hace flush del RX buffer.
3. Escribe y lee con timeout `MODBUS_RESPONSE_TIMEOUT_MS` (100 ms).
4. Valida CRC y comprueba bit de excepción Modbus.
5. Reintenta hasta `MODBUS_RETRY_COUNT` veces (2) con pausa de `MODBUS_INTER_FRAME_MS` (20 ms) entre intentos.

---

## Driver TCP (`kx_tcp`)

Replica la misma arquitectura de cuatro tareas que RTU, adaptada a sockets TCP.

### Tareas y prioridades

| Tarea FreeRTOS | Función | Prioridad |
|---|---|---|
| `kx_tcp_demand` | Pipeline demand TCP | `KX_TASK_PRIO_TELEMETRY + 1` |
| `kx_tcp_writer` | Escrituras TCP | `KX_TASK_PRIO_TELEMETRY + 2` |
| `kx_tcp_report` | Reports periódicos TCP | `KX_TASK_PRIO_TELEMETRY` |
| `kx_tcp_pub` | Publicación MQTT | `KX_TASK_PRIO_TELEMETRY - 1` |

### Colas

| Cola | Tamaño | Propósito |
|---|---|---|
| `pub_queue` | 64 | Resultados pendientes de publicar |
| `demand_queue` | 128 | Solicitudes de poll bajo demanda |
| `write_queue` | 16 | Comandos de escritura |

Los tamaños son menores que RTU porque TCP está pensado para 1–4 PLCs en red local, sin la ráfaga masiva de un bus RS-485 con muchos esclavos.

### `kx_modbus_tcp_socket` — Gestión de sockets persistentes

Mantiene una tabla de hasta `KX_MAX_TCP_SOCKETS` (4) conexiones TCP persistentes. Cada entrada identifica un endpoint `(ip, port)` con su `fd` y el `next_tid` para el MBAP header.

`kx_tcp_sock_get_or_connect()` busca primero en la tabla por IP+puerto. Si no existe, alloca una entrada nueva. Si el socket está cerrado (fd < 0), intenta reconectar hasta `KX_TCP_MAX_RECONNECT_TRIES` (5) veces con pausa de `KX_TCP_RECONNECT_DELAY_MS` (2000 ms).

`kx_tcp_transaction()` construye el **MBAP header** (Transaction ID, Protocol ID=0, Length, Unit ID), envía la PDU y recibe la respuesta validando TID, Unit ID y bit de excepción Modbus. Toda la tabla de sockets está protegida por `s_sock_mutex`.

### `kx_modbus_tcp_dispatch` — Ejecución de packets TCP

Análogo a `kx_modbus_dispatch` para RTU. Usa `kx_tcp_read_register()` y `kx_tcp_read_multi()` en lugar del UART. El fallback individual en caso de fallo multi es idéntico en lógica.

---

## Flujo completo de una lectura bajo demanda

```
MQTT broker                 main.c              kx_modbus (RTU o TCP)
     │                        │                        │
     │── entities/get ────────►│                        │
     │                        │── kx_modbus_request_poll(param_id)
     │                        │                        │
     │                        │              demand_queue ← {param_id, ts}
     │                        │              DEMAND_BIT set
     │                        │                        │
     │                        │              [burst collection phase]
     │                        │                        │
     │                        │              kx_pkt_build(ctrl, demand, {param_id})
     │                        │                        │
     │                        │              kx_dispatch_packet[_tcp]()
     │                        │                        │
     │                        │              kx_publish_all_params_for_reg()
     │                        │                        │
     │                        │              pub_queue ← {PUB_KIND_STATUS, value}
     │                        │                        │
     │                        │              _publisher_task → kx_param_pub_status()
     │                        │                        │
     │◄── entities/{id}/status ────────────────────────│
```

---

## Flujo completo de una escritura

```
MQTT broker                 kx_telemetry            kx_modbus (RTU o TCP)
     │                           │                         │
     │── entities/set ──────────►│                         │
     │                           │── kx_modbus_enqueue_write() / kx_modbus_tcp_enqueue_write()
     │                           │                         │
     │                           │              write_queue ← {ctrl, param, value, ts}
     │                           │                         │
     │                           │              kx_writer_task / kx_tcp_writer_task
     │                           │              espera POLL_ALLOWED_BIT + foreach_mutex
     │                           │                         │
     │                           │              _execute_write[_tcp]()
     │                           │                         │
     │                           │              pub_queue ← {PUB_KIND_STATUS, value} (si OK)
     │                           │              pub_queue ← {PUB_KIND_ERROR}         (si falla)
     │                           │                         │
     │◄── entities/{id}/status ──────────────────────────── │
```

---

## Sincronización y pausa del driver

El par `kx_modbus_pause()` / `kx_modbus_resume()` (y sus equivalentes TCP) permite a `kx_config_handler` congelar el bus durante operaciones de actualización de configuración (`kx_param_store_clear_entities`):

1. `pause()` limpia `POLL_ALLOWED_BIT` e intenta tomar `foreach_mutex` con timeout de 60 s. Esto garantiza que ninguna tarea está en medio de una transacción.
2. `resume()` suelta el mutex y vuelve a setear `POLL_ALLOWED_BIT`.

El timeout de 60 s es conservador y produce un log de error si se alcanza, indicando un posible deadlock.

---

## API pública

### RTU (`kx_modbus_master.h`)

```c
// Arranque — normalmente llamado desde kx_config_handler
esp_err_t kx_modbus_master_start(void);
esp_err_t kx_modbus_master_ensure_started(void);  // idempotente
void      kx_modbus_master_stop(void);
bool      kx_modbus_master_is_running(void);

// Pausa/reanuda el bus (para actualizaciones de config)
void kx_modbus_pause(void);
void kx_modbus_resume(void);

// Solicita lectura de param_id (0 = ciclo completo)
void kx_modbus_request_poll(int param_id);

// Encola escritura de un parámetro
esp_err_t kx_modbus_enqueue_write(int control_id, int param_id,
                                   float value, double ts);
```

### TCP (`kx_modbus_tcp.h`)

```c
esp_err_t kx_modbus_tcp_start(void);
esp_err_t kx_modbus_tcp_ensure_started(void);
void      kx_modbus_tcp_stop(void);
bool      kx_modbus_tcp_is_running(void);

void      kx_modbus_tcp_pause(void);
void      kx_modbus_tcp_resume(void);

// param_id == 0 → ciclo completo de todos los controles TCP
void      kx_modbus_tcp_request_poll(int param_id);

esp_err_t kx_modbus_tcp_enqueue_write(int control_id, int param_id,
                                       float value, double ts);
```

---

## Dependencias del componente

```
kx_modbus
  ├── kx_system        (heap, uptime, estado MQTT)
  ├── kx_mqtt          (kx_mqtt_is_connected, verificación antes de publicar)
  ├── kx_param_store   (parámetros, registros, endpoints TCP)
  ├── kx_telemetry     (kx_param_pub_status/report/error)
  ├── driver           (UART ESP-IDF)
  ├── freertos
  ├── esp_timer
  ├── lwip             (sockets TCP)
  └── json             (privado, no expuesto)
```

---

## Constantes configurables

Definidas en `kx_config.h` (timing de tareas) y en los headers de cada módulo:

| Constante | Valor | Descripción |
|---|---|---|
| `MODBUS_RESPONSE_TIMEOUT_MS` | 100 | Timeout espera respuesta RTU |
| `MODBUS_INTER_FRAME_MS` | 20 | Pausa entre tramas RTU |
| `MODBUS_RETRY_COUNT` | 2 | Reintentos por transacción RTU |
| `KX_TCP_CONNECT_TIMEOUT_MS` | 3000 | Timeout de conexión TCP |
| `KX_TCP_RECV_TIMEOUT_MS` | 500 | Timeout recepción TCP |
| `KX_TCP_RETRY_COUNT` | 2 | Reintentos por transacción TCP |
| `KX_TCP_MAX_RECONNECT_TRIES` | 5 | Intentos de reconexión socket |
| `KX_PKT_MAX_REGS_PER_PKT` | 3 | Registros máx. por packet multi |
| `KX_PKT_MAX_GAP` | 0 | Huecos tolerados al agrupar |
| `KX_DEMAND_TIMEOUT_S` | 10 | TTL de demanda en cola |
| `KX_MAX_TCP_SOCKETS` | 4 | Conexiones TCP simultáneas |