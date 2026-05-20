#include "kx_telemetry.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "kx_param_store.h"
#include "kx_modbus_master.h"
#include <sys/time.h>
#include "kx_param_store.h"
#include "kx_modbus_master.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

static const char *TAG = "kx_telemetry";

// ─────────────────────────────────────────────────────────────
// Helpers internos
// ─────────────────────────────────────────────────────────────

static int8_t _get_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return 0;
}

static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// ─────────────────────────────────────────────────────────────
// Publicaciones salientes (device → broker)
// ─────────────────────────────────────────────────────────────

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
        control_id, uuid, connection_status, connection_status, _ts());

    kx_mqtt_publish(topic, payload, 1, 0);
}

void kx_param_pub_status(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

void kx_param_pub_report(int control_id, int param_id, float value)
{
    char topic[128];
    char payload[128];

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/report", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"value\":%.3f,\"ts\":%.3f}",
             param_id, value, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

/**
 * Publica un error genérico.
 * Topic: {uuid}/quiiot/entities/{id}/status
 */
void kx_param_pub_error(int control_id, int param_id, const char *msg, uint16_t reg)
{
    char topic[128];
    char payload[256];

    snprintf(topic, sizeof(topic),
             "%s/quiiot/entities/%d/status", KX_DEVICE_UUID, param_id);
    snprintf(payload, sizeof(payload),
             "{\"id\":%d,\"error\":true,\"error_message\":\"%s\","
             "\"reg\":\"0x%04x\",\"ts\":%.3f}",
             param_id, msg, reg, _ts());

    kx_mqtt_publish(topic, payload, 0, 0);
}

// ─────────────────────────────────────────────────────────────
// kx_param_handle_set
typedef struct {
    int    target_param_id;
    int    found_control_id;
    double found_ts_set;
    bool   found;
} _set_search_ctx_t;

static void _find_param_control_cb(int control_id, const kx_param_t *param,
                                   void *user_data)
{
    _set_search_ctx_t *ctx = (_set_search_ctx_t *)user_data;
    if (!ctx->found && param->param_id == ctx->target_param_id) {
        ctx->found_control_id = control_id;
        ctx->found_ts_set     = param->ts_set;
        ctx->found            = true;
    }
}

void kx_param_handle_set(const char *topic, const char *payload, size_t len)
{
    // ── 2. Parsear JSON ───────────────────────────────────────
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGW(TAG, "handle_set: JSON inválido topic=%s", topic);
        return;
    }

    // Validar _type == "entity-set"
    cJSON *type = cJSON_GetObjectItem(root, "_type");
    if (!type || !cJSON_IsString(type) || strcmp(type->valuestring, "entity-set") != 0) {
        ESP_LOGD(TAG, "handle_set: _type no es 'entity-set', ignorando topic=%s", topic);
        cJSON_Delete(root);
        return;
    }

    // Validar operation == "set"
    cJSON *op = cJSON_GetObjectItem(root, "operation");
    if (!op || !cJSON_IsString(op) || strcmp(op->valuestring, "set") != 0) {
        ESP_LOGD(TAG, "handle_set: operation no es 'set', ignorando topic=%s", topic);
        cJSON_Delete(root);
        return;
    }

    // Extraer el ID único directamente del JSON (antiguo client_id de la URI)
    cJSON *id_item = cJSON_GetObjectItem(root, "id");
    if (!id_item || !cJSON_IsNumber(id_item)) {
        ESP_LOGW(TAG, "handle_set: Falta el campo numérico 'id' en el payload JSON");
        cJSON_Delete(root);
        return;
    }
    int entity_id = id_item->valueint;

    if (entity_id <= 0) {
        ESP_LOGW(TAG, "handle_set: entity_id inválido extraído del JSON (%d)", entity_id);
        cJSON_Delete(root);
        return;
    }

    // Leer value
    cJSON *val_item = cJSON_GetObjectItem(root, "value");
    if (!val_item || !cJSON_IsNumber(val_item)) {
        ESP_LOGW(TAG, "handle_set: sin campo 'value' numérico, entity_id=%d", entity_id);
        cJSON_Delete(root);
        return;
    }
    float new_value = (float)val_item->valuedouble;

    // Leer ts
    cJSON *ts_item = cJSON_GetObjectItem(root, "ts");
    double incoming_ts = (ts_item && cJSON_IsNumber(ts_item))
                         ? ts_item->valuedouble : 0.0;

    // Ya tenemos todas las variables locales, liberamos el objeto JSON de inmediato de la memoria RAM
    cJSON_Delete(root);

    ESP_LOGI(TAG, "handle_set: entity_id=%d value=%.3f ts_in=%.3f",
             entity_id, new_value, incoming_ts);

    // ── 3. Buscar el param en todos los controles ─────────────
    _set_search_ctx_t ctx = {
        .target_param_id  = entity_id,
        .found_control_id = -1,
        .found_ts_set     = 0.0,
        .found            = false,
    };
    kx_param_store_foreach(_find_param_control_cb, &ctx);

    if (!ctx.found) {
        ESP_LOGW(TAG, "handle_set: entity_id=%d no encontrado en ningún control",
                 entity_id);
        return;
    }

    int control_id = ctx.found_control_id;
    double stored_ts = ctx.found_ts_set;

    ESP_LOGI(TAG, "handle_set: entity_id=%d → control_id=%d | ts_in=%.3f ts_stored=%.3f",
             entity_id, control_id, incoming_ts, stored_ts);

    // ── 4. Comparar timestamps ────────────────────────────────
    if (incoming_ts > 0.0 && incoming_ts <= stored_ts) {
        ESP_LOGD(TAG, "handle_set: ts_in=%.3f <= ts_stored=%.3f — ignorando (ya procesado)",
                 incoming_ts, stored_ts);
        return;
    }

    // ── 5a. Actualizar ts_set en el hash ──────────────────────
    esp_err_t ts_err = kx_param_store_set_ts_set(control_id, entity_id, incoming_ts);
    if (ts_err != ESP_OK) {
        ESP_LOGW(TAG, "handle_set: no se pudo actualizar ts_set entity_id=%d", entity_id);
    }

    // ── 5b. Escribir por Modbus al esclavo ────────────────────
    ESP_LOGI(TAG, "handle_set: escribiendo Modbus → ctrl=%d entity=%d value=%.3f",
             control_id, entity_id, new_value);

    esp_err_t err = kx_modbus_write_one(control_id, entity_id, new_value);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "handle_set: Modbus OK → ctrl=%d entity=%d value=%.3f",
                 control_id, entity_id, new_value);
    } else {
        ESP_LOGW(TAG, "handle_set: Modbus FAIL → ctrl=%d entity=%d err=%s",
                 control_id, entity_id, esp_err_to_name(err));
        kx_param_pub_error(control_id, entity_id, "modbus_write_error", 0x0000);
    }
}



// --- Tarea y Control ---

static void _telemetry_task(void *arg)
{
    uint32_t seq = 0;
    ESP_LOGI(TAG, "Telemetry task started");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));

        ESP_LOGI(TAG, "alive seq=%" PRIu32 " heap=%" PRIu32 " rssi=%d mqtt=%s",
                 seq,
                 kx_system_heap_free(),
                 (int)_get_rssi(),
                 kx_mqtt_is_connected() ? "connected" : "disconnected");
        seq++;
    }
}

esp_err_t kx_telemetry_start(void)
{
    BaseType_t ret = xTaskCreate(
        _telemetry_task,
        "kx_telemetry",
        4096,
        NULL,
        2,
        NULL
    );
    return (ret == pdPASS) ? ESP_OK : ESP_FAIL;
}