#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "esp_heap_caps.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
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

// ── Buffers para recepción fragmentada ─────────────────────────
static char   *s_rx_buf     = NULL;
static int     s_rx_total   = 0;
static int     s_rx_written = 0;
static char    s_rx_topic[128] = {0};

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
    esp_mqtt_client_subscribe(s_client, "+/" KX_DEVICE_UUID, 0);
    ESP_LOGI(TAG, "subscribed: +/%s", KX_DEVICE_UUID);
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_mqtt_client_subscribe(s_client, "+/" KX_DEVICE_UUID "/controls", 0);
    ESP_LOGI(TAG, "subscribed: +/%s/controls", KX_DEVICE_UUID);
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_mqtt_client_subscribe(s_client, "+/" KX_DEVICE_UUID "/controls/+", 0);
    ESP_LOGI(TAG, "subscribed: +/%s/controls/+", KX_DEVICE_UUID);
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_mqtt_client_subscribe(s_client, "+/" KX_DEVICE_UUID "/controls/+/entities", 0);
    ESP_LOGI(TAG, "subscribed: +/%s/controls/+/entities", KX_DEVICE_UUID);
    vTaskDelay(pdMS_TO_TICKS(200));
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
        if (ev->topic_len > 0) {
            size_t tlen = ev->topic_len < sizeof(s_rx_topic) - 1
                        ? ev->topic_len : sizeof(s_rx_topic) - 1;
            memcpy(s_rx_topic, ev->topic, tlen);
            s_rx_topic[tlen] = '\0';

            if (s_rx_buf) {
                free(s_rx_buf);
                s_rx_buf = NULL;
            }

            s_rx_total   = ev->total_data_len;
            s_rx_written = 0;

            uint32_t available = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
            ESP_LOGI(TAG, "RX start topic=%s total=%d heap=%lu",
                    s_rx_topic, s_rx_total, (unsigned long)available);

            s_rx_buf = malloc(s_rx_total + 1);
            if (!s_rx_buf) {
                // intentar con lo que haya dejando 10 KB de margen
                uint32_t safe = available > 10480 ? available - 10480 : 0;
                ESP_LOGW(TAG, "OOM: need %d bytes — intentando con %lu bytes",
                        s_rx_total, (unsigned long)safe);
                if (safe > 0) {
                    s_rx_buf = malloc(safe + 1);
                }
                if (s_rx_buf) {
                    s_rx_total = (int)safe;
                    ESP_LOGW(TAG, "buffer parcial: se almacenarán %d de %d bytes",
                            s_rx_total, ev->total_data_len);
                } else {
                    ESP_LOGE(TAG, "OOM total: no se puede almacenar nada del payload");
                    s_rx_total = 0;
                }
            }
        }

        // acumular chunk
        if (s_rx_buf && ev->data && ev->data_len > 0) {
            int space   = s_rx_total - s_rx_written;
            int to_copy = ev->data_len < space ? ev->data_len : space;
            if (to_copy > 0) {
                memcpy(s_rx_buf + s_rx_written, ev->data, to_copy);
                s_rx_written += to_copy;
            }
            ESP_LOGI(TAG, "recibiendo... %d/%d bytes (%.1f%%)",
                    ev->current_data_offset + ev->data_len,
                    ev->total_data_len,
                    (float)(ev->current_data_offset + ev->data_len) * 100.0f / ev->total_data_len);
        }

        // último chunk
        if (ev->current_data_offset + ev->data_len >= ev->total_data_len) {
            if (s_rx_buf) {
                s_rx_buf[s_rx_written] = '\0';
                ESP_LOGI(TAG, "RX complete topic=%s almacenado=%d total=%d heap=%lu%s",
                        s_rx_topic, s_rx_written, ev->total_data_len,
                        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                        s_rx_written < ev->total_data_len ? " [TRUNCADO]" : "");

                // mostrar lo almacenado en bloques de 200 chars
                ESP_LOGI(TAG, "── payload almacenado ──────────────────");
                for (int i = 0; i < s_rx_written; i += 200) {
                    int chunk = (s_rx_written - i) < 200 ? (s_rx_written - i) : 200;
                    ESP_LOGI(TAG, "%.*s", chunk, s_rx_buf + i);
                }
                ESP_LOGI(TAG, "── fin payload ─────────────────────────");

                if (s_msg_cb) {
                    s_msg_cb(s_rx_topic, s_rx_buf, s_rx_written);
                }

                free(s_rx_buf);
                s_rx_buf = NULL;
            } else {
                ESP_LOGE(TAG, "RX complete pero sin buffer — payload descartado");
            }

            s_rx_total   = 0;
            s_rx_written = 0;
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
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "%s", KX_DEVICE_UUID);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri            = KX_MQTT_BROKER_URI,
        .credentials.client_id         = client_id,
        .credentials.username          = KX_MQTT_USERNAME,
        .credentials.authentication.password = KX_MQTT_PASSWORD,

        // ── TLS ──────────────────────────────────────────────────
        // OPCIÓN A: sin TLS (desarrollo local, allow_anonymous)
        // → usar URI  mqtt://host:1883
        // No hay nada que configurar aquí.

        // OPCIÓN B: TLS con CA reconocida (Let's Encrypt)
        // → cambiar URI a  mqtts://host:8883
        // → descomentar la línea siguiente:
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,

        // ── Sesión ───────────────────────────────────────────────
        .session.keepalive          = KX_MQTT_KEEPALIVE_S,
        .session.last_will.topic    = s_lwt_topic,
        .session.last_will.msg      = s_lwt_payload,
        .session.last_will.qos      = 1,
        .session.last_will.retain   = 1,

        // ── Red ──────────────────────────────────────────────────
        .network.reconnect_timeout_ms = KX_MQTT_RECONNECT_MIN_MS,

        // ── Buffers ───────────────────────────────────────────────

        .buffer.size     = KX_PAYLOAD_MAX_BYTES,
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
