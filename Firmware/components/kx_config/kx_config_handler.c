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
#include <sys/time.h>

static const char *TAG = "kx_config";

// ── Estado: entities ya disponibles (desde NVS) ───────────────
static volatile bool s_entities_ready = false;

void kx_config_set_entities_ready(bool ready)
{
    s_entities_ready = ready;
    if (ready) {
        ESP_LOGI(TAG, "entities marked as ready (from NVS cache)");
    }
}

// ── Timestamp ─────────────────────────────────────────────────
static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// ── ACK / Error ───────────────────────────────────────────────
static void _send_ack(const char *config_type)
{
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%.3f,\"config_type\":\"%s\",\"status\":\"ok\"}",
        KX_DEVICE_UUID, _ts(), config_type);
    kx_mqtt_publish(KX_TOPIC_CONFIG_ACK, payload, 1, 0);
    ESP_LOGI(TAG, "ack sent for '%s'", config_type);
}

static void _send_error(const char *config_type,
                        const char *error_code,
                        const char *detail)
{
    char payload[512];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%.3f,"
        "\"config_type\":\"%s\","
        "\"error_code\":\"%s\","
        "\"detail\":\"%s\"}",
        KX_DEVICE_UUID, _ts(),
        config_type, error_code, detail);
    kx_mqtt_publish(KX_TOPIC_CONFIG_ERROR, payload, 1, 0);
    ESP_LOGW(TAG, "error '%s' for '%s': %s", error_code, config_type, detail);
}

// ── Helpers de topic ──────────────────────────────────────────
// Rutas posibles:
//   …/controls                    → "controls_list"
//   …/controls/21601              → "control_single"
//   …/controls/21601/entities     → "entities"
//   …/{uuid}  (sin /controls)     → "device"
static const char *_config_type_from_topic(const char *topic)
{
    if (strstr(topic, "/entities")) return "entities";

    const char *p = strstr(topic, "/controls");
    if (p) {
        p += strlen("/controls");
        // termina aquí → lista
        if (*p == '\0' || *p == ' ') return "controls_list";
        // hay algo tras la barra → control individual
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

// ── Validación de device config ───────────────────────────────
static esp_err_t _validate_device_config(cJSON *root)
{
    if (!cJSON_GetObjectItem(root, "uuid")) return ESP_FAIL;
    return ESP_OK;
}

// ── Publicar control-status ───────────────────────────────────
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
        control_id, uuid, _ts());

    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "control-status online → ctrl=%d uuid=%s",
                 control_id, uuid);
    } else {
        ESP_LOGW(TAG, "control-status publish failed → ctrl=%d", control_id);
    }
}

// ── Solicitar entities ────────────────────────────────────────
static void _request_entities(int control_id)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic),
             KX_DEVICE_UUID "/controls/%d/entities", control_id);

    snprintf(payload, sizeof(payload),
             "{\"_type\": \"entities-discovery\", \"timestamp\": %.3f}",
             _ts());

    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "entities-discovery → control_id=%d", control_id);
    } else {
        ESP_LOGW(TAG, "entities-discovery failed → control_id=%d", control_id);
    }
}

// ── Callback de progreso ──────────────────────────────────────
static void _on_entities_progress(int control_id, int received, int total)
{
    if (total <= 0) return;
    int pct = (received * 100) / total;
    if (pct == 25 || pct == 50 || pct == 75 || pct == 100) {
        ESP_LOGI(TAG, "entities ctrl=%d: %d%% (%d/%d)",
                 control_id, pct, received, total);
    }
}

// ── Procesar UN control ───────────────────────────────────────
// Extrae control_id, uuid y slave_addr del objeto JSON del control.
// Publica control-status y lanza entities-discovery si hace falta.
static void _process_single_control(cJSON *ctrl, int hint_control_id)
{
    // control_id: del JSON o del topic como fallback
    int control_id = hint_control_id;
    cJSON *id_field = cJSON_GetObjectItem(ctrl, "control_id");
    if (!id_field) id_field = cJSON_GetObjectItem(ctrl, "id");
    if (id_field && cJSON_IsNumber(id_field)) {
        control_id = (int)id_field->valuedouble;
    }

    if (control_id <= 0) {
        ESP_LOGW(TAG, "control: could not determine control_id, skipping");
        return;
    }

    // uuid
    char uuid[64] = "";
    cJSON *u = cJSON_GetObjectItem(ctrl, "uuid");
    if (u && cJSON_IsString(u)) {
        snprintf(uuid, sizeof(uuid), "%s", u->valuestring);
    }

    // slave_addr — puede venir con distintos nombres según el broker
    int slave_addr = 0;
    const char *addr_keys[] = {
        "slave_addr", "modbus_address", "control_address",
        "address", "rtu_address", NULL
    };
    for (int k = 0; addr_keys[k] != NULL; k++) {
        cJSON *sa = cJSON_GetObjectItem(ctrl, addr_keys[k]);
        if (sa && cJSON_IsNumber(sa)) {
            slave_addr = (int)sa->valuedouble;
            ESP_LOGI(TAG, "ctrl=%d slave_addr=%d (key='%s')",
                     control_id, slave_addr, addr_keys[k]);
            break;
        }
    }

    if (slave_addr > 0) {
        kx_param_store_set_slave_addr(control_id, slave_addr);
    } else {
        ESP_LOGW(TAG, "ctrl=%d: no slave_addr found in JSON", control_id);
    }

    ESP_LOGI(TAG, "control id=%d uuid=%s | entities_cached=%s",
             control_id, uuid, s_entities_ready ? "YES" : "NO");

    _publish_control_status(control_id, uuid);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (s_entities_ready) {
        ESP_LOGI(TAG, "ctrl=%d: entities from NVS cache, skip discovery",
                 control_id);
    } else {
        _request_entities(control_id);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ── controls_list: {"controls": [...]} ───────────────────────
static esp_err_t _handle_controls_list(cJSON *root)
{
    cJSON *controls = cJSON_GetObjectItem(root, "controls");
    if (!controls || !cJSON_IsArray(controls)) {
        ESP_LOGW(TAG, "controls_list: missing 'controls' array");
        return ESP_FAIL;
    }

    int count = cJSON_GetArraySize(controls);
    ESP_LOGI(TAG, "controls_list: %d controls", count);

    kx_mqtt_resize_queue(count);
    if (!s_entities_ready) {
        kx_param_store_set_expected(count);
    }

    for (int i = 0; i < count; i++) {
        cJSON *ctrl = cJSON_GetArrayItem(controls, i);
        if (ctrl) _process_single_control(ctrl, -1);
    }

    return ESP_OK;
}

// ── control_single: topic …/controls/NUM ─────────────────────
// El payload puede ser el objeto control directamente
// o envolver un array "controls":[...].
static esp_err_t _handle_control_single(cJSON *root, int topic_control_id)
{
    cJSON *controls = cJSON_GetObjectItem(root, "controls");
    if (controls && cJSON_IsArray(controls)) {
        int count = cJSON_GetArraySize(controls);
        ESP_LOGI(TAG, "control_single: found controls array with %d items", count);
        kx_mqtt_resize_queue(count);
        if (!s_entities_ready) kx_param_store_set_expected(count);
        for (int i = 0; i < count; i++) {
            cJSON *ctrl = cJSON_GetArrayItem(controls, i);
            if (ctrl) _process_single_control(ctrl, topic_control_id);
        }
    } else {
        // El raíz ES el objeto control
        kx_mqtt_resize_queue(1);
        if (!s_entities_ready) {
            // Garantizar que expected sea al menos count+1 para que
            // is_ready() no dispare antes de tiempo cuando llegan
            // múltiples topics individuales.
            int expected = kx_param_store_count() + 1;
            kx_param_store_set_expected(expected);
        }
        _process_single_control(root, topic_control_id);
    }

    return ESP_OK;
}

// ── Handler principal ─────────────────────────────────────────
void kx_config_handle(const char *topic, const char *payload, size_t len)
{
    const char *config_type = _config_type_from_topic(topic);

    // ── entities ─────────────────────────────────────────────
    if (strcmp(config_type, "entities") == 0) {
        int control_id = _control_id_from_topic(topic);
        if (control_id <= 0) {
            ESP_LOGW(TAG, "entities: could not extract control_id from topic: %s",
                     topic);
            return;
        }

        ESP_LOGI(TAG, "entities received: topic=%s size=%d heap=%" PRIu32,
                 topic, (int)len, kx_system_heap_free());

        kx_param_store_set_progress_cb(_on_entities_progress);
        esp_err_t err = kx_param_store_parse(payload, len, control_id);
        kx_param_store_set_progress_cb(NULL);

        if (err == ESP_OK) {
            // Marcar entities como disponibles cuando el store está completo
            if (kx_param_store_is_ready()) {
                s_entities_ready = true;
                ESP_LOGI(TAG, "all entities ready — s_entities_ready=true");
            }
            _send_ack(config_type);
        } else {
            _send_error(config_type, "PARSE_ERROR", "entities parse failed");
        }
        return;
    }

    // ── filtro de tamaño para el resto ────────────────────────
    if (len > KX_PAYLOAD_MAX_BYTES) {
        ESP_LOGW(TAG, "payload too large (%d bytes)", (int)len);
        _send_error(config_type, "PARSE_ERROR", "payload exceeds max size");
        return;
    }

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        const char *ep = cJSON_GetErrorPtr();
        char detail[64];
        snprintf(detail, sizeof(detail), "parse error near: %.40s",
                 ep ? ep : "unknown");
        _send_error(config_type, "PARSE_ERROR", detail);
        return;
    }

    esp_err_t err = ESP_OK;

    if (strcmp(config_type, "controls_list") == 0) {
        err = _handle_controls_list(root);
        if (err == ESP_OK) {
            _send_ack("controls");
        } else {
            _send_error("controls", "MISSING_FIELD", "controls array required");
        }

    } else if (strcmp(config_type, "control_single") == 0) {
        err = _handle_control_single(root, _control_id_from_topic(topic));
        if (err == ESP_OK) {
            _send_ack("controls");
        } else {
            _send_error("controls", "PARSE_ERROR", "control_single failed");
        }

    } else if (strcmp(config_type, "device") == 0) {
        err = _validate_device_config(root);
        if (err == ESP_OK) {
            _send_ack("device");
        } else {
            _send_error("device", "MISSING_FIELD", "uuid required");
        }

    } else {
        ESP_LOGW(TAG, "unhandled config type '%s' for topic: %s",
                 config_type, topic);
    }

    cJSON_Delete(root);
}