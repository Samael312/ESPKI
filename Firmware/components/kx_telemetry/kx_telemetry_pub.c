#include "kx_telemetry.h"
#include "kx_telemetry_internal.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include <stdio.h>

// =============================================================
// kx_telemetry_pub.c — Publicaciones salientes (device → broker)
// =============================================================

void kx_control_pub_status(int control_id, const char *uuid,
                            const char *connection_status)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic),
             "%s/controls/%d/status", KX_DEVICE_UUID, control_id);
    snprintf(payload, sizeof(payload),
        "{"
        "\"_type\": \"control-status\","
        "\"id\": %d,"
        "\"uuid\": \"%s\","
        "\"connection_status\": \"%s\","
        "\"link\": {\"detected\": \"%s\"},"
        "\"timestamp\": %.3f"
        "}",
        control_id, uuid, connection_status, connection_status, kx_telemetry_ts());

    kx_mqtt_publish(topic, payload, 1, 0);
}

void kx_param_pub_status(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    (void)control_id;

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, kx_telemetry_ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

void kx_param_pub_report(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    (void)control_id;

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/report", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, kx_telemetry_ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

void kx_param_pub_error(int control_id, int param_id,
                        const char *msg, uint16_t reg)
{
    char topic[128];
    char payload[256];

    (void)control_id;

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"error\":true,\"error_message\":\"%s\","
             "\"reg\":\"0x%04x\",\"ts\":%.3f}",
             param_id, msg, reg, kx_telemetry_ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}