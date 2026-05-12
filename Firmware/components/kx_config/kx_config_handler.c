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
// Determina el tipo de config a partir del topic.
// Nuevos casos:
//   …/controls          → "controls_list"   (array de controls)
//   …/controls/21601    → "control_single"  (un control individual)
//   …/controls/21601/entities → "entities"
static const char *_config_type_from_topic(const char *topic)
{
    if (strstr(topic, "/entities")) return "entities";

    const char *p = strstr(topic, "/controls");
    if (p) {
        p += strlen("/controls");
        
        // ".../controls" o ".../controls " -> Lista completa
        if (*p == '\0' || *p == ' ') return "controls_list";
        
        // ".../controls/..." -> Configuración de un control (o sub-ruta)
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

// ── Procesar UN control (reutilizado por ambas rutas) ─────────
// Recibe el objeto cJSON de un control individual ya parseado.
static void _process_single_control(cJSON *ctrl, int hint_control_id)
{
    // Obtener control_id: buscar "control_id", "id", o usar hint del topic
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

    char uuid[64] = "";
    cJSON *u = cJSON_GetObjectItem(ctrl, "uuid");
    if (u && cJSON_IsString(u)) {
        snprintf(uuid, sizeof(uuid), "%s", u->valuestring);
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

// ── control_single: topic …/controls/NUM, payload = objeto control ──
// El broker envía el control directamente sin envelope "controls":[].
// El control_id puede estar en el topic y/o en el payload.
static esp_err_t _handle_control_single(cJSON *root, int topic_control_id)
{
    // El payload puede ser el objeto control directamente,
    // o puede tener una clave "controls" con un array de uno.
    cJSON *controls = cJSON_GetObjectItem(root, "controls");
    if (controls && cJSON_IsArray(controls)) {
        // Vinieron varios controles en este topic de todas formas
        int count = cJSON_GetArraySize(controls);
        ESP_LOGI(TAG, "control_single topic but controls array: %d items", count);
        kx_mqtt_resize_queue(count);
        if (!s_entities_ready) kx_param_store_set_expected(count);
        for (int i = 0; i < count; i++) {
            cJSON *ctrl = cJSON_GetArrayItem(controls, i);
            if (ctrl) _process_single_control(ctrl, topic_control_id);
        }
    } else {
        // El objeto raíz ES el control
        kx_mqtt_resize_queue(1);
        if (!s_entities_ready) {
            // Incrementar expected: podemos recibir varios topics individuales.
            // Si ya teníamos expected >= 1 lo mantenemos; si no, ponemos 1
            // como mínimo para que is_ready() funcione eventualmente.
            // El valor real se ajustará cuando llegue el control definitivo.
            // Nota: si el broker manda N topics individuales, cada uno llama
            // aquí y set_expected(1) sobreescribiría. Usamos max heurístico.
            int current = kx_param_store_count();
            int expected = current + 1;  // al menos uno más
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

    // ── 1. CONTROL_INFO (Mapeo Modbus + Activación de Control) ──────
    if (strcmp(config_type, "control_info") == 0) {
        cJSON *root = cJSON_ParseWithLength(payload, len);
        if (!root) return;

        int ctrl_id = _control_id_from_topic(topic);
        cJSON *addr = cJSON_GetObjectItem(root, "control_address");
        
        // A. Guardamos el slave_addr para Modbus
        if (ctrl_id > 0 && addr && cJSON_IsNumber(addr)) {
            int slave_addr = (int)addr->valuedouble;
            kx_param_store_set_slave_addr(ctrl_id, slave_addr);
            ESP_LOGI(TAG, "control_info: ctrl=%d mapeado a slave_addr=%d", ctrl_id, slave_addr);
        }

        // B. PROCESAMOS EL CONTROL (Esto disparará el entities-discovery)
        // Pasamos 'root' porque el payload de control_info suele ser el objeto control
        _process_single_control(root, ctrl_id);

        cJSON_Delete(root);
        _send_ack("control_info");
        return; // Aquí sí salimos, porque ya hicimos todo
    }

    // ── 2. ENTITIES (Sin cambios) ───────────────────────────────────
    if (strcmp(config_type, "entities") == 0) {
        int control_id = _control_id_from_topic(topic);
        if (control_id > 0) {
            kx_param_store_set_progress_cb(_on_entities_progress);
            kx_param_store_parse(payload, len, control_id);
            kx_param_store_set_progress_cb(NULL);
            _send_ack(config_type);
        }
        return; 
    }

    // ── 3. RESTO (Filtro de tamaño y parseo para otros tipos) ────────
    if (len > KX_PAYLOAD_MAX_BYTES) {
        _send_error(config_type, "PARSE_ERROR", "payload too large");
        return;
    }

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) return;

    if (strcmp(config_type, "controls_list") == 0) {
        _handle_controls_list(root);
    } 
    else if (strcmp(config_type, "control_single") == 0) {
        _handle_control_single(root, _control_id_from_topic(topic));
    }

    cJSON_Delete(root);
}