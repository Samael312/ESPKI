#include "kx_config_handler.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "kx_param_store.h"
#include "kx_modbus_master.h"
#include "kx_modbus_tcp.h"
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
#include <inttypes.h>

static const char *TAG = "kx_config";

// =============================================================
// Timestamp
// =============================================================
static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// =============================================================
// ACK / Error
// =============================================================
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
        KX_DEVICE_UUID, _ts(), config_type, error_code, detail);
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
             "{\"_type\": \"controls-discovery\", \"timestamp\": %.3f}", _ts());
    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "controls-discovery → %s", topic);
    else
        ESP_LOGW(TAG, "controls-discovery publish failed");
}

// =============================================================
// entities-discovery por control
// =============================================================
static void _request_entities(int control_id)
{
    char topic[128];
    char payload[128];
    snprintf(topic, sizeof(topic),
             "%s/controls/%d/entities", KX_DEVICE_UUID, control_id);
    snprintf(payload, sizeof(payload),
             "{\"_type\": \"entities-discovery\", \"timestamp\": %.3f}", _ts());
    esp_err_t err = kx_mqtt_publish(topic, payload, 1, 0);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "entities-discovery → control_id=%d", control_id);
    else
        ESP_LOGW(TAG, "entities-discovery failed → control_id=%d", control_id);
}

// =============================================================
// control-status online
// =============================================================
static void _publish_control_status(int control_id, const char *uuid)
{
    char topic[128];
    char payload[256];
    snprintf(topic, sizeof(topic),
             "%s/controls/%d/status", KX_DEVICE_UUID, control_id);
    snprintf(payload, sizeof(payload),
        "{\"_type\":\"control-status\","
        "\"id\":%d,"
        "\"uuid\":\"%s\","
        "\"connection_status\":\"online\","
        "\"link\":{\"detected\":\"online\"},"
        "\"timestamp\":%.3f}",
        control_id, uuid, _ts());
    kx_mqtt_publish(topic, payload, 1, 0);
    ESP_LOGI(TAG, "control-status online → ctrl=%d uuid=%s", control_id, uuid);
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
// _process_single_control
//
// Parsea el JSON de un control individual y configura:
//   - slave_addr / unit_id
//   - uuid
//   - protocolo (RTU o TCP) con su endpoint
//   - lanza entities-discovery si el update_ts es mayor
// =============================================================
static void _process_single_control(cJSON *ctrl_json, int hint_control_id)
{
    // ── control_id ────────────────────────────────────────────
    int control_id = hint_control_id;
    cJSON *id_field = cJSON_GetObjectItem(ctrl_json, "control_id");
    if (!id_field) id_field = cJSON_GetObjectItem(ctrl_json, "id");
    if (id_field && cJSON_IsNumber(id_field))
        control_id = (int)id_field->valuedouble;

    if (control_id <= 0) {
        ESP_LOGW(TAG, "control: no control_id, skipping");
        return;
    }

    // ── uuid ──────────────────────────────────────────────────
    char uuid[64] = "";
    cJSON *u = cJSON_GetObjectItem(ctrl_json, "uuid");
    if (u && cJSON_IsString(u))
        snprintf(uuid, sizeof(uuid), "%s", u->valuestring);

    // ── slave_addr / unit_id ──────────────────────────────────
    // Siempre viene en control_address (tanto RTU como TCP)
    int slave_addr = 0;
    const char *addr_keys[] = {
        "control_address", "slave_addr", "modbus_address",
        "address", "rtu_address", NULL
    };
    for (int k = 0; addr_keys[k]; k++) {
        cJSON *sa = cJSON_GetObjectItem(ctrl_json, addr_keys[k]);
        if (sa && cJSON_IsNumber(sa)) { slave_addr = (int)sa->valuedouble; break; }
    }

    // ── update_ts ─────────────────────────────────────────────
    double ts_incoming = 0.0;
    cJSON *ts_field = cJSON_GetObjectItem(ctrl_json, "update_ts");
    if (ts_field && cJSON_IsNumber(ts_field))
        ts_incoming = ts_field->valuedouble;

    double ts_stored = kx_param_store_get_update_ts(control_id);

    // ── Parseo del protocolo ──────────────────────────────────
    // Ruta: metadata → protocol → metadata → active
    //                                       → tcp → ip / port
    const char *proto_active  = "rtu";
    char        tcp_ip[KX_TCP_IP_LEN] = "";
    uint16_t    tcp_port = 0;

    cJSON *meta = cJSON_GetObjectItem(ctrl_json, "metadata");
    if (meta) {
        cJSON *proto = cJSON_GetObjectItem(meta, "protocol");
        if (proto) {
            cJSON *pmeta = cJSON_GetObjectItem(proto, "metadata");
            if (pmeta) {
                cJSON *active = cJSON_GetObjectItem(pmeta, "active");
                if (active && cJSON_IsString(active))
                    proto_active = active->valuestring;

                if (strcmp(proto_active, "tcp") == 0) {
                    cJSON *tcp = cJSON_GetObjectItem(pmeta, "tcp");
                    if (tcp) {
                        cJSON *ip_j   = cJSON_GetObjectItem(tcp, "ip");
                        cJSON *port_j = cJSON_GetObjectItem(tcp, "port");
                        if (ip_j   && cJSON_IsString(ip_j))
                            snprintf(tcp_ip, sizeof(tcp_ip), "%s",
                                     ip_j->valuestring);
                        if (port_j && cJSON_IsNumber(port_j))
                            tcp_port = (uint16_t)port_j->valuedouble;
                    }
                }
            }
        }
    }

    ESP_LOGI(TAG,
             "ctrl=%d uuid=%s slave=%d proto=%s ts_in=%.3f ts_stored=%.3f",
             control_id, uuid, slave_addr, proto_active,
             ts_incoming, ts_stored);
    if (strcmp(proto_active, "tcp") == 0)
        ESP_LOGI(TAG, "  TCP endpoint: %s:%" PRIu16, tcp_ip, tcp_port);

    // ── Actualizar metadatos en param_store ───────────────────
    if (slave_addr > 0)
        kx_param_store_set_slave_addr(control_id, slave_addr);
    if (uuid[0])
        kx_param_store_set_uuid(control_id, uuid);

    // Endpoint TCP — limpia si es RTU
    if (strcmp(proto_active, "tcp") == 0 && tcp_ip[0] != '\0' && tcp_port > 0)
        kx_param_store_set_tcp_endpoint(control_id, tcp_ip, tcp_port);
    else
        kx_param_store_set_tcp_endpoint(control_id, NULL, 0);

    // ── Arrancar el driver correcto si aún no está activo ─────
    // Colas y tareas se crean la primera vez que se confirma
    // un control de ese protocolo. Idempotente: si ya corre no
    // hace nada. De este modo NO se reserva RAM para un driver
    // que no tiene ningún control asignado.
    if (strcmp(proto_active, "tcp") == 0) {
        esp_err_t dr = kx_modbus_tcp_ensure_started();
        if (dr != ESP_OK)
            ESP_LOGE(TAG, "ctrl=%d: TCP driver start failed: %s",
                     control_id, esp_err_to_name(dr));
    } else {
        esp_err_t dr = kx_modbus_master_ensure_started();
        if (dr != ESP_OK)
            ESP_LOGE(TAG, "ctrl=%d: RTU driver start failed: %s",
                     control_id, esp_err_to_name(dr));
    }

    // ── Publicar control-status ───────────────────────────────
    _publish_control_status(control_id, uuid);
    vTaskDelay(pdMS_TO_TICKS(50));

    // ── Decisión de entities-discovery ───────────────────────
    bool need_discovery = false;

    if (ts_incoming == 0.0) {
        ESP_LOGI(TAG, "ctrl=%d: no update_ts → forcing discovery", control_id);
        need_discovery = true;

    } else if (ts_incoming > ts_stored) {
        ESP_LOGI(TAG, "ctrl=%d: ts newer (%.3f > %.3f) → refreshing entities",
                 control_id, ts_incoming, ts_stored);

        // Pausar ambos drivers para evitar carreras durante el clear
        bool rtu_running = kx_modbus_master_is_running();
        bool tcp_running = kx_modbus_tcp_is_running();
        if (rtu_running) kx_modbus_pause();
        if (tcp_running) kx_modbus_tcp_pause();

        kx_param_store_clear_entities(control_id);
        kx_param_store_set_update_ts(control_id, ts_incoming);

        if (rtu_running) kx_modbus_resume();
        if (tcp_running) kx_modbus_tcp_resume();

        need_discovery = true;

    } else {
        ESP_LOGI(TAG, "ctrl=%d: ts up-to-date (%.3f) → using cached",
                 control_id, ts_stored);
        kx_param_store_print_active_samplings();
    }

    if (need_discovery) {
        _request_entities(control_id);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
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
            if (ctrl) _process_single_control(ctrl, -1);
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
            if (ctrl) _process_single_control(ctrl, -1);
        }
        return ESP_OK;
    }
    // Caso 3: {"control": {...}}
    cJSON *single = cJSON_GetObjectItem(root, "control");
    if (single && cJSON_IsObject(single)) {
        kx_mqtt_resize_queue(1);
        kx_param_store_set_expected(1);
        _process_single_control(single, -1);
        return ESP_OK;
    }
    // Caso 4: objeto raíz ES el control
    cJSON *id_check = cJSON_GetObjectItem(root, "control_id");
    if (!id_check) id_check = cJSON_GetObjectItem(root, "id");
    if (id_check && cJSON_IsNumber(id_check)) {
        kx_mqtt_resize_queue(1);
        kx_param_store_set_expected(1);
        _process_single_control(root, -1);
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
            if (ctrl) _process_single_control(ctrl, topic_control_id);
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

        _process_single_control(root, topic_control_id);
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
    _send_ack("device");
    vTaskDelay(pdMS_TO_TICKS(200));
    kx_config_request_controls();
    return ESP_OK;
}

// =============================================================
// Diagnóstico de samplings
// =============================================================
static void _print_active_sampling_cb(int control_id, const kx_param_t *param,
                                       void *user_data)
{
    (void)user_data;
    if (param->sampling > 0) {
        ESP_LOGI("STORE_DEBUG",
                 "ctrl=%d param=%d reg=0x%04x sampling=%ds name=%s",
                 control_id, param->param_id, param->reg,
                 param->sampling, param->name);
    }
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
                    kx_param_store_print_active_samplings();
                } else {
                    ESP_LOGW(TAG, "NVS save failed: %s", esp_err_to_name(nvs_err));
                }
            }
            _send_ack(config_type);
        } else {
            _send_error(config_type, "PARSE_ERROR", "entities parse failed");
        }
        return;
    }

    // ── filtro de tamaño ──────────────────────────────────────
    if (len > KX_PAYLOAD_MAX_BYTES) {
        ESP_LOGW(TAG, "payload too large (%d bytes)", (int)len);
        _send_error(config_type, "PARSE_ERROR", "payload exceeds max size");
        return;
    }

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        const char *ep = cJSON_GetErrorPtr();
        char detail[64];
        snprintf(detail, sizeof(detail), "parse error near: %.40s", ep ? ep : "unknown");
        _send_error(config_type, "PARSE_ERROR", detail);
        return;
    }

    esp_err_t err = ESP_OK;

    if (strcmp(config_type, "device") == 0) {
        err = _handle_device(root);
        if (err != ESP_OK)
            _send_error("device", "MISSING_FIELD", "uuid required");

    } else if (strcmp(config_type, "controls_list") == 0) {
        err = _handle_controls_list(root);
        if (err == ESP_OK) _send_ack("controls");
        else _send_error("controls", "MISSING_FIELD", "controls array required");

    } else if (strcmp(config_type, "control_single") == 0) {
        err = _handle_control_single(root, _control_id_from_topic(topic));
        if (err == ESP_OK) _send_ack("controls");
        else _send_error("controls", "PARSE_ERROR", "control_single failed");

    } else {
        ESP_LOGW(TAG, "unhandled config type '%s' topic: %s",
                 config_type, topic);
    }

    cJSON_Delete(root);
}