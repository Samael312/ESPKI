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
#define KX_TELEMETRY_INTERVAL_S  10

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

// ── Firmware version ──────────────────────────────────────────
#define KX_FW_VERSION  "0.2.0"

// =============================================================
// Modbus RTU — MAX13487E auto-direction
//   RO  → GPIO36  (UART RX, input-only)
//   DI  → GPIO4   (UART TX)
// =============================================================
#define KX_MODBUS_BAUD              19200
#define KX_MODBUS_RESP_TIMEOUT_MS   1000
#define KX_MODBUS_INTER_CHAR_MS     5
#define KX_MODBUS_RETRY_COUNT       3
#define KX_MODBUS_TX_FLUSH_DELAY_MS 3

// ── Modo de arranque ─────────────────────────────────────────
// KX_MODBUS_TEST_MODE: si está definido, ejecuta test en lugar
// del polling normal. Descomentar para depurar hardware RS-485.
//
#define KX_MODBUS_TEST_MODE

// Parámetros del test manual (usados solo con KX_MODBUS_TEST_MODE)
#define KX_MODBUS_TEST_SLAVE    0x02   // dirección del esclavo
#define KX_MODBUS_TEST_FC       0x03   // función: 03 = Read Holding Regs
#define KX_MODBUS_TEST_REG      0x0000 // primer registro a leer
#define KX_MODBUS_TEST_QTY      10     // número de registros

// Scan de bus (descomentar para activar al inicio)
//#define KX_MODBUS_SCAN_ON_BOOT
#define KX_MODBUS_SCAN_FROM     0x01
#define KX_MODBUS_SCAN_TO       0x10