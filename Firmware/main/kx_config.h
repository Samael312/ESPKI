#pragma once

// =============================================================
// kx_config.h  —  Kiconex Box Lite — Configuración por defecto
// =============================================================

// ── WiFi ─────────────────────────────────────────────────────
#define KX_WIFI_SSID            "pruebas_ansible"
//#define KX_WIFI_SSID          "WIFI-PISO"
#define KX_WIFI_PASSWORD        "KiconexWiFi"
//#define KX_WIFI_PASSWORD      "1234567890"
#define KX_WIFI_MAX_RETRY       10

// ── MQTT Broker ───────────────────────────────────────────────
#define KX_MQTT_BROKER_URI      "mqtts://pruebas.kiconex.com:28883"
#define KX_MQTT_USERNAME        "iotest"
#define KX_MQTT_PASSWORD        "iotest"

//#define KX_MQTT_BROKER_URI    "mqtt://192.168.5.108:1883"
//#define KX_MQTT_USERNAME      ""
//#define KX_MQTT_PASSWORD      ""

#define KX_MQTT_KEEPALIVE_S       120
#define KX_MQTT_RECONNECT_MIN_MS  5000
#define KX_MQTT_RECONNECT_MAX_MS  60000

// ── Telemetría ────────────────────────────────────────────────
#define KX_TELEMETRY_INTERVAL_S  5

// ── Tareas FreeRTOS ───────────────────────────────────────────
#define KX_TASK_STACK_NET         4096
#define KX_TASK_STACK_MQTT        16384
#define KX_TASK_STACK_TELEMETRY   4096
#define KX_TASK_STACK_SUPERVISION 3072

#define KX_TASK_PRIO_NET         5
#define KX_TASK_PRIO_MQTT        5
#define KX_TASK_PRIO_TELEMETRY   4
#define KX_TASK_PRIO_SUPERVISION 6

// ── Payload ───────────────────────────────────────────────────
#define KX_PAYLOAD_MAX_BYTES     40960
#define MQTT_MAX_TOPIC_SIZE      1024

// ── Identidad ─────────────────────────────────────────────────
#define KX_DEVICE_UUID   "d041dd10-bf3a-456f-851a-135e2233d577"
#define KX_TOPIC_PREFIX  "quiiot"

// ── Topics entrantes (broker → dispositivo) ───────────────────
#define KX_TOPIC_CONFIG_DEVICE          "+/" KX_DEVICE_UUID
#define KX_TOPIC_CONFIG_CONTROLS        "+/" KX_DEVICE_UUID "/controls"
#define KX_TOPIC_CONFIG_CONTROLS_ONE    "+/" KX_DEVICE_UUID "/controls/+"
#define KX_TOPIC_CONFIG_ENTITIES        "+/" KX_DEVICE_UUID "/controls/+/entities"

// ── Topics salientes (dispositivo → broker) ───────────────────
#define KX_TOPIC_STATUS       KX_DEVICE_UUID "/connection/status"
#define KX_TOPIC_TELEMETRY    KX_TOPIC_PREFIX "/" KX_DEVICE_UUID "/telemetry"
#define KX_TOPIC_CONFIG_ACK   KX_TOPIC_PREFIX "/" KX_DEVICE_UUID "/config/ack"
#define KX_TOPIC_CONFIG_ERROR KX_TOPIC_PREFIX "/" KX_DEVICE_UUID "/config/error"

// Discovery activo (dispositivo → bridge)
// Topic: {uuid}/controls  con payload {"_type":"controls-discovery","timestamp":...}
// El bridge escucha este topic y responde publicando controls.json
// en +/{uuid}/controls (recibido por el dispositivo vía suscripción)
#define KX_TOPIC_CONTROLS_DISCOVERY     KX_DEVICE_UUID "/controls"

// ── Modbus ────────────────────────────────────────────────────
#define KX_MODBUS_UART_NUM UART_NUM_1
#define KX_MODBUS_PIN_TX   GPIO_NUM_4
#define KX_MODBUS_PIN_RX   GPIO_NUM_36
#define KX_MODBUS_BAUD     9600

// ── Firmware version ──────────────────────────────────────────
#define KX_FW_VERSION  "0.2.0"

// En kx_config.h — añade macros de log estructurado
#define KX_LOG_PHASE(tag, phase, fmt, ...) \
    ESP_LOGI(tag, "[%-12s] " fmt, phase, ##__VA_ARGS__)

#define KX_LOG_CYCLE_START(tag, cycle, controls, readable) \
    ESP_LOGI(tag, \
        "\n┌─[CYCLE %-3" PRId64 "]──────────────────────────────┐\n" \
        "│  controls=%-3d  readable=%-3d  heap=%-8" PRIu32 "│", \
        cycle, controls, readable, kx_system_heap_free())

#define KX_LOG_CYCLE_END(tag, read, err, skip, nochange) \
    ESP_LOGI(tag, \
        "│  read=%-4d err=%-4d skip=%-4d nochange=%-4d  │\n" \
        "└────────────────────────────────────────────────┘", \
        read, err, skip, nochange)