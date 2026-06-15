#include "kx_config_handler.h"
#include "kx_config_internal.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "kx_config";

// =============================================================
// kx_config_handler.c — Router principal, ACK/error, discovery
// =============================================================

// =============================================================
// ACK / Error
// =============================================================
void kx_config_send_ack(const char *config_type)
{
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%.3f,\"config_type\":\"%s\",\"status\":\"ok\"}",
        KX_DEVICE_UUID, kx_config_ts(), config_type);
    kx_mqtt_publish(KX_TOPIC_CONFIG_ACK, payload, 1, 0);
    ESP_LOGI(TAG, "ack sent for '%s'", config_type);
}

void kx_config_send_error(const char *config_type,
                           const char *error_code,
                           const char *detail)
{
    char payload[512];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%.3f,"
        "\"config_type\":\"%s\","
        "\"error_code\":\"%s\","
        "\"detail\":\"%s\"}",
        KX_DEVICE_UUID, kx_config_ts(), config_type, error_code, detail);
    kx_mqtt_publish(KX_TOPIC_CONFIG_ERROR, payload, 1, 0);
    ESP_LOGW(TAG, "error '%s' for '%s': %s", error_code, config_type, detail);
}

// =============================================================
// Helpers de topic
// =============================================================
static const char *_config_type_from_topic(const char *topic)
{
    if (strstr(topic, "/entities")) return "entities";
    const char *p = strstr(topic, "/controls");
    if (p) {
        p += strlen("/controls");
        if (*p == '\0' || *p == ' ') return "controls_list";
        if (*p == '/') return "control_single";
        return "controls_list";
    }
    if (strstr(topic, KX_DEVICE_UUID)) return "device";
    return "unknown";
}

static int _control_id_from_topic(const char *topic)
{
    const char *p = strstr(topic, "/controls/");
    if (!p) return -1;
    p += strlen("/controls/");
    return atoi(p);
}

// =============================================================
// controls-discovery
// =============================================================
void kx_config_request_controls(void)
{
    char topic[MQTT_MAX_TOPIC_SIZE];
    char payload[256];
    snprintf(topic, sizeof(topic), "%s/controls", KX_DEVICE_UUID);
    snprintf(payload, sizeof(payload),
             "{\"_type\": \"controls-discovery\", \"timestamp\": %.3f}", kx_config_ts());
    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "controls-discovery → %s", topic);
    else
        ESP_LOGW(TAG, "controls-discovery publish failed");
}

// =============================================================
// entities-discovery por control
// =============================================================
void kx_config_request_entities(int control_id)
{
    char topic[128];
    char payload[128];
    snprintf(topic, sizeof(topic),
             "%s/controls/%d/entities", KX_DEVICE_UUID, control_id);
    snprintf(payload, sizeof(payload),
             "{\"_type\": \"entities-discovery\", \"timestamp\": %.3f}", kx_config_ts());
    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "entities-discovery → control_id=%d", control_id);
    else
        ESP_LOGW(TAG, "entities-discovery failed → control_id=%d", control_id);
}

// =============================================================
// Callback de progreso de entities
// =============================================================
static void _on_entities_progress(int control_id, int received, int total)
{
    if (total <= 0) return;
    int pct = (received * 100) / total;
    if (pct == 25 || pct == 50 || pct == 75 || pct == 100)
        ESP_LOGI(TAG, "entities ctrl=%d: %d%% (%d/%d)",
                 control_id, pct, received, total);
}

// =============================================================
// controls_list
// =============================================================
static esp_err_t _handle_controls_list(cJSON *root)
{
    // Caso 1: {"controls": [...]}
    cJSON *controls = cJSON_GetObjectItem(root, "controls");
    if (controls && cJSON_IsArray(controls)) {
        int count = cJSON_GetArraySize(controls);
        ESP_LOGI(TAG, "controls_list: %d items", count);
        kx_mqtt_resize_queue(count);
        kx_param_store_set_expected(count);
        for (int i = 0; i < count; i++) {
            cJSON *ctrl = cJSON_GetArrayItem(controls, i);
            if (ctrl) kx_config_process_single_control(ctrl, -1);
        }
        return ESP_OK;
    }
    // Caso 2: array en raíz
    if (cJSON_IsArray(root)) {
        int count = cJSON_GetArraySize(root);
        ESP_LOGI(TAG, "controls_list (root array): %d items", count);
        kx_mqtt_resize_queue(count);
        kx_param_store_set_expected(count);
        for (int i = 0; i < count; i++) {
            cJSON *ctrl = cJSON_GetArrayItem(root, i);
            if (ctrl) kx_config_process_single_control(ctrl, -1);
        }
        return ESP_OK;
    }
    // Caso 3: {"control": {...}}
    cJSON *single = cJSON_GetObjectItem(root, "control");
    if (single && cJSON_IsObject(single)) {
        kx_mqtt_resize_queue(1);
        kx_param_store_set_expected(1);
        kx_config_process_single_control(single, -1);
        return ESP_OK;
    }
    // Caso 4: objeto raíz ES el control
    cJSON *id_check = cJSON_GetObjectItem(root, "control_id");
    if (!id_check) id_check = cJSON_GetObjectItem(root, "id");
    if (id_check && cJSON_IsNumber(id_check)) {
        kx_mqtt_resize_queue(1);
        kx_param_store_set_expected(1);
        kx_config_process_single_control(root, -1);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "controls_list: unrecognized JSON structure");
    cJSON *item = root->child;
    while (item) {
        ESP_LOGW(TAG, "  key: \"%s\"", item->string ? item->string : "(null)");
        item = item->next;
    }
    return ESP_FAIL;
}

// =============================================================
// control_single: topic …/controls/NUM
// =============================================================
static esp_err_t _handle_control_single(cJSON *root, int topic_control_id)
{
    cJSON *controls = cJSON_GetObjectItem(root, "controls");
    if (controls && cJSON_IsArray(controls)) {
        int count = cJSON_GetArraySize(controls);
        kx_mqtt_resize_queue(count);
        kx_param_store_set_expected(count);
        for (int i = 0; i < count; i++) {
            cJSON *ctrl = cJSON_GetArrayItem(controls, i);
            if (ctrl) kx_config_process_single_control(ctrl, topic_control_id);
        }
    } else {
        int ctrl_id = topic_control_id;
        cJSON *id_f = cJSON_GetObjectItem(root, "control_id");
        if (!id_f) id_f = cJSON_GetObjectItem(root, "id");
        if (id_f && cJSON_IsNumber(id_f)) ctrl_id = (int)id_f->valuedouble;

        bool already_exists = (kx_param_store_get_ctrl(ctrl_id) != NULL);
        kx_mqtt_resize_queue(1);
        if (!already_exists)
            kx_param_store_set_expected(kx_param_store_count() + 1);

        kx_config_process_single_control(root, topic_control_id);
    }
    return ESP_OK;
}

// =============================================================
// device: +/{uuid}
// =============================================================
static esp_err_t _handle_device(cJSON *root)
{
    if (!cJSON_GetObjectItem(root, "uuid")) return ESP_FAIL;
    ESP_LOGI(TAG, "device.json received — launching controls-discovery");
    kx_config_send_ack("device");
    vTaskDelay(pdMS_TO_TICKS(200));
    kx_config_request_controls();
    return ESP_OK;
}

// =============================================================
// Handler principal
// =============================================================
void kx_config_handle(const char *topic, const char *payload, size_t len)
{
    const char *config_type = _config_type_from_topic(topic);

    // ── entities ─────────────────────────────────────────────
    if (strcmp(config_type, "entities") == 0) {
        int control_id = _control_id_from_topic(topic);
        if (control_id <= 0) {
            ESP_LOGW(TAG, "entities: no control_id in topic: %s", topic);
            return;
        }
        ESP_LOGI(TAG, "entities received: topic=%s size=%d heap=%" PRIu32,
                 topic, (int)len, kx_system_heap_free());

        kx_param_store_set_progress_cb(_on_entities_progress);
        esp_err_t err = kx_param_store_parse(payload, len, control_id);
        kx_param_store_set_progress_cb(NULL);

        if (err == ESP_OK) {
            if (kx_param_store_is_ready()) {
                ESP_LOGI(TAG, "all entities ready — saving to NVS");
                esp_err_t nvs_err = kx_param_store_save_nvs();
                if (nvs_err == ESP_OK) {
                    ESP_LOGI(TAG, "NVS save OK");
                    //kx_param_store_print_active_samplings();
                } else {
                    ESP_LOGW(TAG, "NVS save failed: %s", esp_err_to_name(nvs_err));
                }
            }
            kx_config_send_ack(config_type);
        } else {
            kx_config_send_error(config_type, "PARSE_ERROR", "entities parse failed");
        }
        return;
    }

    // ── filtro de tamaño ──────────────────────────────────────
    if (len > KX_PAYLOAD_MAX_BYTES) {
        ESP_LOGW(TAG, "payload too large (%d bytes)", (int)len);
        kx_config_send_error(config_type, "PARSE_ERROR", "payload exceeds max size");
        return;
    }

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        const char *ep = cJSON_GetErrorPtr();
        char detail[64];
        snprintf(detail, sizeof(detail), "parse error near: %.40s", ep ? ep : "unknown");
        kx_config_send_error(config_type, "PARSE_ERROR", detail);
        return;
    }

    esp_err_t err = ESP_OK;

    if (strcmp(config_type, "device") == 0) {
        err = _handle_device(root);
        if (err != ESP_OK)
            kx_config_send_error("device", "MISSING_FIELD", "uuid required");

    } else if (strcmp(config_type, "controls_list") == 0) {
        err = _handle_controls_list(root);
        if (err == ESP_OK) kx_config_send_ack("controls");
        else kx_config_send_error("controls", "MISSING_FIELD", "controls array required");

    } else if (strcmp(config_type, "control_single") == 0) {
        err = _handle_control_single(root, _control_id_from_topic(topic));
        if (err == ESP_OK) kx_config_send_ack("controls");
        else kx_config_send_error("controls", "PARSE_ERROR", "control_single failed");

    } else {
        ESP_LOGW(TAG, "unhandled config type '%s' topic: %s",
                 config_type, topic);
    }

    cJSON_Delete(root);
}