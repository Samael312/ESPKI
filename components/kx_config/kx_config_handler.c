#include "kx_config_handler.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

static const char *TAG = "kx_config";

static void _send_ack(const char *config_type)
{
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%lu,\"config_type\":\"%s\",\"status\":\"ok\"}",
        KX_DEVICE_UUID, (unsigned long)time(NULL), config_type);
    kx_mqtt_publish(KX_TOPIC_CONFIG_ACK, payload, 1, 0);
    ESP_LOGI(TAG, "ack sent for '%s'", config_type);
}

static void _send_error(const char *config_type,
                        const char *error_code,
                        const char *detail)
{
    char payload[512];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%lu,"
        "\"config_type\":\"%s\","
        "\"error_code\":\"%s\","
        "\"detail\":\"%s\"}",
        KX_DEVICE_UUID, (unsigned long)time(NULL),
        config_type, error_code, detail);
    kx_mqtt_publish(KX_TOPIC_CONFIG_ERROR, payload, 1, 0);
    ESP_LOGW(TAG, "error '%s' for '%s': %s", error_code, config_type, detail);
}

static const char *_config_type_from_topic(const char *topic)
{
    if (strstr(topic, "/entities")) return "entities";
    if (strstr(topic, "/controls")) return "controls";
    if (strstr(topic, KX_DEVICE_UUID) && !strstr(topic, "/controls")) return "device";
    return "unknown";
}

static int _control_id_from_topic(const char *topic)
{
    const char *p = strstr(topic, "/controls/");
    if (!p) return -1;
    p += strlen("/controls/");
    return atoi(p);
}

static esp_err_t _validate_device_config(cJSON *root)
{
    if (!cJSON_GetObjectItem(root, "uuid")) return ESP_FAIL;
    return ESP_OK;
}

static void _publish_control_status(int control_id, const char *uuid)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic),
             "%s/controls/%d/status",
             KX_DEVICE_UUID, control_id);

    snprintf(payload, sizeof(payload),
        "{"
        "\"_type\": \"control-status\","
        "\"id\": %d,"
        "\"uuid\": \"%s\","
        "\"connection_status\": \"online\","
        "\"link\": {\"detected\": \"online\"},"
        "\"timestamp\": %.3f"
        "}",
        control_id, uuid,
        (double)esp_timer_get_time() / 1000000.0);

    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "control-status online → ctrl=%d uuid=%s",
                 control_id, uuid);
    } else {
        ESP_LOGW(TAG, "control-status publish failed → ctrl=%d", control_id);
    }
}

static void _request_entities(int control_id)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic),
             KX_DEVICE_UUID "/controls/%d/entities", control_id);

    snprintf(payload, sizeof(payload),
             "{\"_type\": \"entities-discovery\", \"timestamp\": %.3f}",
             (double)esp_timer_get_time() / 1000000.0);

    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "entities-discovery → control_id=%d", control_id);
    } else {
        ESP_LOGW(TAG, "entities-discovery failed → control_id=%d", control_id);
    }
}

static esp_err_t _validate_controls_config(cJSON *root)
{
    cJSON *controls = cJSON_GetObjectItem(root, "controls");
    if (!controls || !cJSON_IsArray(controls)) {
        ESP_LOGW(TAG, "controls: missing or not an array");
        return ESP_FAIL;
    }

    int count = cJSON_GetArraySize(controls);
    ESP_LOGI(TAG, "controls received: %d", count);

    kx_mqtt_resize_queue(count);

    for (int i = 0; i < count; i++) {
        cJSON *ctrl = cJSON_GetArrayItem(controls, i);
        if (!ctrl) continue;

        cJSON *id = cJSON_GetObjectItem(ctrl, "control_id");
        if (!id) id = cJSON_GetObjectItem(ctrl, "id");
        if (!id || !cJSON_IsNumber(id)) {
            ESP_LOGW(TAG, "control[%d]: missing id, skipping", i);
            continue;
        }
        int control_id = (int)id->valuedouble;

        char uuid[64] = "";
        cJSON *u = cJSON_GetObjectItem(ctrl, "uuid");
        if (u && cJSON_IsString(u)) {
            snprintf(uuid, sizeof(uuid), "%s", u->valuestring);
        }

        ESP_LOGI(TAG, "control[%d]: id=%d uuid=%s", i, control_id, uuid);

        _publish_control_status(control_id, uuid);
        vTaskDelay(pdMS_TO_TICKS(50));

        _request_entities(control_id);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return ESP_OK;
}

// ── Handler principal ─────────────────────────────────────────
void kx_config_handle(const char *topic, const char *payload, size_t len)
{
    const char *config_type = _config_type_from_topic(topic);

    // ── entities ─────────────────────────────────────────────
    if (strcmp(config_type, "entities") == 0) {
        uint32_t heap_before = kx_system_heap_free();
        ESP_LOGI(TAG, "entities received: topic=%s size=%d heap=%" PRIu32,
                 topic, (int)len, heap_before);

        if (len > 200) {
            ESP_LOGI(TAG, "payload start: %.100s", payload);
            ESP_LOGI(TAG, "payload end:   %.100s", payload + len - 100);
        } else {
            ESP_LOGI(TAG, "payload: %.*s", (int)len, payload);
        }

        // parsear y almacenar en kx_param_store
        int control_id = _control_id_from_topic(topic);
        if (control_id > 0) {
            kx_param_store_parse(payload, len, control_id);
        } else {
            ESP_LOGW(TAG, "could not extract control_id from topic: %s", topic);
        }

        ESP_LOGI(TAG, "entities end — heap=%" PRIu32, kx_system_heap_free());
        _send_ack(config_type);
        return;
    }

    // ── resto de tipos: limitar tamaño ────────────────────────
    if (len > KX_PAYLOAD_MAX_BYTES) {
        ESP_LOGW(TAG, "payload too large (%d bytes)", (int)len);
        _send_error(config_type, "PARSE_ERROR", "payload exceeds max size");
        return;
    }

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        const char *err_ptr = cJSON_GetErrorPtr();
        char detail[64];
        snprintf(detail, sizeof(detail), "parse error near: %.40s",
                 err_ptr ? err_ptr : "unknown");
        _send_error(config_type, "PARSE_ERROR", detail);
        return;
    }

    uint32_t heap_before = kx_system_heap_free();
    esp_err_t err = ESP_OK;

    if (strcmp(config_type, "device") == 0) {
        err = _validate_device_config(root);
        if (err != ESP_OK) {
            _send_error(config_type, "MISSING_FIELD", "uuid required");
        }
    } else if (strcmp(config_type, "controls") == 0) {
        err = _validate_controls_config(root);
    } else {
        ESP_LOGW(TAG, "unknown config type in topic: %s", topic);
        cJSON_Delete(root);
        return;
    }

    cJSON_Delete(root);

    uint32_t heap_after = kx_system_heap_free();
    ESP_LOGI(TAG, "config '%s' heap: before=%" PRIu32 " after=%" PRIu32 " delta=%d",
             config_type, heap_before, heap_after,
             (int)heap_before - (int)heap_after);

    if (heap_after < heap_before / 2) {
        ESP_LOGW(TAG, "high memory pressure after config parse!");
    }

    if (err == ESP_OK) {
        _send_ack(config_type);
    }
}