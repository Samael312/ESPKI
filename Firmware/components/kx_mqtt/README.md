# kx_mqtt

Cliente MQTT con reconexión automática, LWT, recepción fragmentada y cola de mensajes con backpressure.

## Responsabilidades

- Conectar al broker MQTT (TLS vía `esp_crt_bundle`) con reconexión automática
- Publicar `device-status online` con IP en cada conexión
- Suscribirse a los cuatro topics de configuración del dispositivo
- Recibir mensajes fragmentados (chunked), ensamblarlos en RAM y enrutarlos por cola
- Aplicar backpressure si la cola o el heap están bajo mínimos
- Exponer una cola redimensionable según el número de controles activos
- Invocar un callback de usuario para cada mensaje completo recibido

## Flujo de recepción

```
Broker → MQTT_EVENT_DATA (fragmentos) → s_rx_buf (ensamblado)
       → kx_msg_t { topic, payload, len }
       → s_msg_queue
       → _processing_task → kx_mqtt_msg_cb_t (callback de usuario)
```

## API pública

```c
// Inicia el cliente MQTT y la tarea de procesamiento.
// on_message se invoca para cada mensaje recibido.
esp_err_t kx_mqtt_start(kx_mqtt_msg_cb_t on_message);

// Publica un mensaje. Devuelve ESP_FAIL si no hay conexión.
esp_err_t kx_mqtt_publish(const char *topic, const char *payload, int qos, int retain);

// Suscribe a un topic adicional.
esp_err_t kx_mqtt_subscribe(const char *topic, int qos);

// True si el cliente está conectado al broker en este momento.
bool kx_mqtt_is_connected(void);

// Redimensiona la cola de mensajes según el número de controles esperados.
// Llamar desde kx_config_handler al recibir el controls.json.
void kx_mqtt_resize_queue(int num_controls);
```

## Parámetros de cola

| Parámetro | Valor por defecto |
|---|---|
| `QUEUE_BASE_SIZE` | 128 slots |
| `QUEUE_PER_CONTROL` | +20 slots por control |
| `KX_QUEUE_BACKPRESSURE_MAX` | 40 mensajes en cola → drop |
| `KX_QUEUE_BACKPRESSURE_HEAP` | 256 KB heap libre mínimo |

## Topics suscritos al conectar

| Topic | Propósito |
|---|---|
| `+/{UUID}` | device.json |
| `+/{UUID}/controls` | lista de controles |
| `+/{UUID}/controls/+` | control individual |
| `+/{UUID}/controls/+/entities` | entidades de un control |
| `{PREFIX}/{UUID}/entities/#` | lecturas/escrituras de entidades |

## LWT (Last Will and Testament)

- Topic: `{UUID}/connection/status`
- Payload: `{"_type":"device-status","device_connection_state":"offline","ts":0}`
- QoS 1, retain

## Configuración relevante (`kx_config.h`)

```c
#define KX_MQTT_BROKER_URI        "mqtts://host:28883"
#define KX_MQTT_USERNAME          "..."
#define KX_MQTT_PASSWORD          "..."
#define KX_MQTT_KEEPALIVE_S       120
#define KX_MQTT_RECONNECT_MIN_MS  5000
#define KX_MQTT_RECONNECT_MAX_MS  60000
#define KX_PAYLOAD_MAX_BYTES      40960   // tamaño del buffer de recepción
```

## Dependencias (IDF)

`kx_system` · `mqtt` · `esp_event` · `esp_netif` · `freertos` · `esp_timer` · `mbedtls`

## Notas

- La tarea `kx_processing` corre con prioridad 6. Procesa mensajes uno a uno de forma secuencial, lo que simplifica el manejo de estado en los callbacks.
- Si el payload llega en múltiples eventos `MQTT_EVENT_DATA`, se ensambla en `s_rx_buf` antes de encolar.
- Si hay OOM al alocar el buffer, se intenta un buffer parcial y el payload llega truncado (se indica en el log con `[TRUNCADO]`).
- Los topics `*/entities/get` se silencian en los logs para reducir ruido.