#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

static const char *TAG = "kx_mqtt";

// ── Estado interno ────────────────────────────────────────────
static esp_mqtt_client_handle_t s_client = NULL;
static kx_mqtt_msg_cb_t         s_msg_cb = NULL;
static volatile bool            s_connected = false;

// ── LWT payload ───────────────────────────────────────────────
// Se construye una vez al iniciar y se reutiliza
static char s_lwt_topic[64];
static char s_lwt_payload[128];


// ── Publica connection/status ─────────────────────────────────
static void _publish_status(const char *status)
{
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"device_id\":\"%s\",\"status\":\"%s\",\"ts\":%lu,\"fw_ver\":\"%s\"}",
        KX_DEVICE_UUID, status, (unsigned long)time(NULL), KX_FW_VERSION);

    esp_mqtt_client_publish(s_client, KX_TOPIC_STATUS, payload, 0, 1, 1);
    ESP_LOGI(TAG, "status → %s", status);
}

static void _subscribe_all(void)
{
    ESP_LOGI(TAG, "subscribing 1: [%s] len=%d", 
             KX_TOPIC_CONFIG_DEVICE, strlen(KX_TOPIC_CONFIG_DEVICE));
    esp_mqtt_client_subscribe(s_client, KX_TOPIC_CONFIG_DEVICE, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "subscribing 2: [%s] len=%d", 
             KX_TOPIC_CONFIG_CONTROLS, strlen(KX_TOPIC_CONFIG_CONTROLS));
    esp_mqtt_client_subscribe(s_client, KX_TOPIC_CONFIG_CONTROLS, 0);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "subscribing 3: [%s] len=%d", 
             KX_TOPIC_CONFIG_ENTITIES, strlen(KX_TOPIC_CONFIG_ENTITIES));
    esp_mqtt_client_subscribe(s_client, KX_TOPIC_CONFIG_ENTITIES, 0);
}
// ── Event handler ─────────────────────────────────────────────
static void _mqtt_event_handler(void *arg, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        kx_system_set_mqtt_state(KX_MQTT_CONNECTED);
        ESP_LOGI(TAG, "CONNECTED to broker");
        ESP_LOGI(TAG, "mqtt stack watermark: %d bytes free",
                uxTaskGetStackHighWaterMark(NULL) * 4);
        _subscribe_all();
        _publish_status("online");
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        kx_system_set_mqtt_state(KX_MQTT_DISCONNECTED);
        kx_system_inc_reconnect_count();
        ESP_LOGW(TAG, "DISCONNECTED (reconnect #%" PRIu32 ")",
                 kx_system_reconnect_count());
        // esp-mqtt reconecta automáticamente con backoff interno
        break;

    case MQTT_EVENT_DATA:
        if (s_msg_cb && ev->topic_len > 0) {
            // Los buffers de esp-mqtt no son null-terminated: copiar
            char topic[128] = {0};
            char payload[KX_PAYLOAD_MAX_BYTES + 1];
            size_t tlen = ev->topic_len < sizeof(topic) - 1
                          ? ev->topic_len : sizeof(topic) - 1;
            size_t plen = ev->data_len < KX_PAYLOAD_MAX_BYTES
                          ? ev->data_len : KX_PAYLOAD_MAX_BYTES;
            memcpy(topic,   ev->topic, tlen);
            memcpy(payload, ev->data,  plen);
            payload[plen] = '\0';
            ESP_LOGD(TAG, "RX topic=%s len=%d", topic, ev->data_len);
            s_msg_cb(topic, payload, plen);
        }
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error type: %d",
                 ev->error_handle->error_type);
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGD(TAG, "msg_id=%d published", ev->msg_id);
        break;

    default:
        break;
    }
}

// ── API pública ───────────────────────────────────────────────
esp_err_t kx_mqtt_start(kx_mqtt_msg_cb_t on_message)
{
    s_msg_cb = on_message;

    // Construir LWT
    snprintf(s_lwt_topic,   sizeof(s_lwt_topic),   "%s", KX_TOPIC_STATUS);
    snprintf(s_lwt_payload, sizeof(s_lwt_payload),
        "{\"device_id\":\"%s\",\"status\":\"offline\",\"ts\":0}",
        KX_DEVICE_UUID);

    // Client ID
    char client_id[32];
    snprintf(client_id, sizeof(client_id), "kx_%s", kx_system_device_id());

    esp_mqtt_client_config_t cfg = {
    .broker.address.uri            = KX_MQTT_BROKER_URI,
    .credentials.client_id         = client_id,

    // ── Autenticación ────────────────────────────────────────
    // Desactivado para pruebas con allow_anonymous
    // Activar en producción:
    // .credentials.username                    = KX_MQTT_USERNAME,
    // .credentials.authentication.password     = KX_MQTT_PASSWORD,

    // ── TLS ──────────────────────────────────────────────────
    // OPCIÓN A: sin TLS (desarrollo local, allow_anonymous)
    // → usar URI  mqtt://host:1883   ← activo ahora
    // No hay nada que configurar aquí.

    // OPCIÓN B: TLS con CA reconocida (Let's Encrypt)
    // → cambiar URI a  mqtts://host:8883
    // → descomentar las dos líneas siguientes:
    // .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,

    // OPCIÓN C: TLS con certificado autofirmado
    // → cambiar URI a  mqtts://host:8883
    // → embed el .pem en CMakeLists con EMBED_TXTFILES
    // → descomentar las dos líneas siguientes:
    // .broker.verification.certificate     = (const char *)broker_ca_pem_start,
    // .broker.verification.certificate_len = broker_ca_pem_end - broker_ca_pem_start,

    // ── Sesión ───────────────────────────────────────────────
    .session.keepalive          = KX_MQTT_KEEPALIVE_S,
    .session.last_will.topic    = s_lwt_topic,
    .session.last_will.msg      = s_lwt_payload,
    .session.last_will.qos      = 1,
    .session.last_will.retain   = 1,

    // ── Red ──────────────────────────────────────────────────
    .network.reconnect_timeout_ms = KX_MQTT_RECONNECT_MIN_MS,
    .buffer.size                  = KX_PAYLOAD_MAX_BYTES,
};

    s_client = esp_mqtt_client_init(&cfg);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to create MQTT client");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   _mqtt_event_handler, NULL);
    kx_system_set_mqtt_state(KX_MQTT_CONNECTING);
    return esp_mqtt_client_start(s_client);
}

esp_err_t kx_mqtt_publish(const char *topic, const char *payload,
                           int qos, int retain)
{
    if (!s_connected) {
        ESP_LOGW(TAG, "publish skipped: not connected");
        return ESP_FAIL;
    }
    int id = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
    return (id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t kx_mqtt_subscribe(const char *topic, int qos)
{
    if (!s_client) return ESP_FAIL;
    return (esp_mqtt_client_subscribe(s_client, topic, qos) >= 0)
           ? ESP_OK : ESP_FAIL;
}

bool kx_mqtt_is_connected(void) { return s_connected; }
