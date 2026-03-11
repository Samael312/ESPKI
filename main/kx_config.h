#pragma once

// =============================================================
// kx_config.h  —  Kiconex Box Lite — Configuración por defecto
// Todos los parámetros ajustables están aquí.
// En Fase 2 se leerán de NVS; por ahora son constantes.
// =============================================================

// ── WiFi ─────────────────────────────────────────────────────
#define KX_WIFI_SSID            "kiconex_pruebas_ansible"       // TODO: cambiar
#define KX_WIFI_PASSWORD        "KiconexWiFi"   // TODO: cambiar
#define KX_WIFI_MAX_RETRY       10

// ── MQTT Broker ───────────────────────────────────────────────
#define KX_MQTT_BROKER_URI      "mqtt://pruebas.kiconex.com:28883"  
#define KX_MQTT_USERNAME        "iotest"   
#define KX_MQTT_PASSWORD        "iotest"
#define KX_MQTT_KEEPALIVE_S       60
#define KX_MQTT_RECONNECT_MIN_MS  5000
#define KX_MQTT_RECONNECT_MAX_MS  60000

// ── Telemetría ────────────────────────────────────────────────
#define KX_TELEMETRY_INTERVAL_S  60   // segundos entre publishes

// ── Tareas FreeRTOS ───────────────────────────────────────────
#define KX_TASK_STACK_NET        4096
#define KX_TASK_STACK_MQTT       6144
#define KX_TASK_STACK_TELEMETRY  4096
#define KX_TASK_STACK_SUPERVISION 3072

#define KX_TASK_PRIO_NET         5
#define KX_TASK_PRIO_MQTT        5
#define KX_TASK_PRIO_TELEMETRY   4
#define KX_TASK_PRIO_SUPERVISION 6   // supervisión tiene prioridad alta

// ── Payload ───────────────────────────────────────────────────
#define KX_PAYLOAD_MAX_BYTES     4096

// ── Topics MQTT (patrón: kx/{device_id}/...) ─────────────────
#define KX_TOPIC_PREFIX          "kx"
#define KX_TOPIC_STATUS          KX_TOPIC_PREFIX "/%s/connection/status"
#define KX_TOPIC_TELEMETRY       KX_TOPIC_PREFIX "/%s/telemetry"
#define KX_TOPIC_CONFIG_DEVICE   KX_TOPIC_PREFIX "/%s/config/device"
#define KX_TOPIC_CONFIG_CONTROLS KX_TOPIC_PREFIX "/%s/config/controls"
#define KX_TOPIC_CONFIG_ACK      KX_TOPIC_PREFIX "/%s/config/ack"
#define KX_TOPIC_CONFIG_ERROR    KX_TOPIC_PREFIX "/%s/config/error"

// ── Firmware version ──────────────────────────────────────────
#define KX_FW_VERSION            "0.1.0"
