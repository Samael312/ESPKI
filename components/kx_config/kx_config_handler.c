#include "kx_config_handler.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "esp_log.h"
#include "cJSON.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "kx_config";

// ── Helpers de respuesta ──────────────────────────────────────
static void _send_ack(const char *config_type)
{
    char topic[64], payload[256];
    snprintf(topic, sizeof(topic), KX_TOPIC_CONFIG_ACK, kx_system_device_id());
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%lu,\"config_type\":\"%s\",\"status\":\"ok\"}",
        kx_system_device_id(),
        (unsigned long)time(NULL),
        config_type);
    kx_mqtt_publish(topic, payload, 1, 0);
    ESP_LOGI(TAG, "ack sent for '%s'", config_type);
}

static void _send_error(const char *config_type,
                         const char *error_code,
                         const char *detail)
{
    char topic[64], payload[512];
    snprintf(topic, sizeof(topic), KX_TOPIC_CONFIG_ERROR, kx_system_device_id());
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"ts\":%lu,"
        "\"config_type\":\"%s\","
        "\"error_code\":\"%s\","
        "\"detail\":\"%s\"}",
        kx_system_device_id(),
        (unsigned long)time(NULL),
        config_type,
        error_code,
        detail);
    kx_mqtt_publish(topic, payload, 1, 0);
    ESP_LOGW(TAG, "error '%s' for '%s': %s", error_code, config_type, detail);
}

// ── Determina el tipo de config por el topic ──────────────────
static const char *_config_type_from_topic(const char *topic)
{
    if (strstr(topic, "/config/device"))   return "device";
    if (strstr(topic, "/config/controls")) return "controls";
    return "unknown";
}

// ── Validación mínima de device config ───────────────────────
// En MVP solo comprobamos que el JSON parsea y tiene campo "device_id"
// En Fase 2: validar todos los campos y aplicarlos.
static esp_err_t _validate_device_config(cJSON *root)
{
    if (!cJSON_GetObjectItem(root, "device_id")) {
        return ESP_FAIL;  // MISSING_FIELD
    }
    // TODO Fase 2: validar fw_version, platform, etc.
    return ESP_OK;
}

static esp_err_t _validate_controls_config(cJSON *root)
{
    // En MVP: solo verificar que sea un objeto o array válido
    // TODO Fase 2: iterar controles, validar tipos, aplicar límites
    (void)root;
    return ESP_OK;
}

// ── Handler principal ─────────────────────────────────────────
void kx_config_handle(const char *topic, const char *payload, size_t len)
{
    const char *config_type = _config_type_from_topic(topic);

    // Verificar tamaño antes de parsear
    if (len > KX_PAYLOAD_MAX_BYTES) {
        ESP_LOGW(TAG, "payload too large (%d bytes)", len);
        _send_error(config_type, "PARSE_ERROR", "payload exceeds max size");
        return;
    }

    // Parsear JSON
    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        const char *err_ptr = cJSON_GetErrorPtr();
        char detail[64];
        snprintf(detail, sizeof(detail), "parse error near: %.40s",
                 err_ptr ? err_ptr : "unknown");
        _send_error(config_type, "PARSE_ERROR", detail);
        return;
    }

    // Log de heap antes/después para detectar presión de memoria
    uint32_t heap_before = kx_system_heap_free();

    // Validar según tipo
    esp_err_t err = ESP_OK;
    if (strcmp(config_type, "device") == 0) {
        err = _validate_device_config(root);
        if (err != ESP_OK) {
            _send_error(config_type, "MISSING_FIELD", "device_id required");
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
    ESP_LOGI(TAG, "config '%s' heap: before=%" PRIu32 " after=%" PRIu32
             " delta=%d",
             config_type, heap_before, heap_after,
             (int)heap_before - (int)heap_after);

    // Advertir si el heap bajó mucho
    if (heap_after < heap_before / 2) {
        ESP_LOGW(TAG, "high memory pressure after config parse!");
    }

    if (err == ESP_OK) {
        _send_ack(config_type);
    }
    // Si err != ESP_OK el error ya fue enviado arriba
}
