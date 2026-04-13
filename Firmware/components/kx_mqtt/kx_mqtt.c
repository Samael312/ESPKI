#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "esp_heap_caps.h"
#include "esp_netif.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "kx_mqtt";

// ── Estado interno ────────────────────────────────────────────
static esp_mqtt_client_handle_t s_client    = NULL;
static kx_mqtt_msg_cb_t         s_msg_cb    = NULL;
static volatile bool            s_connected = false;

// ── LWT ───────────────────────────────────────────────────────
static char s_lwt_topic[64];
static char s_lwt_payload[128];

// ── Buffer de recepción fragmentada ───────────────────────────
static char  *s_rx_buf     = NULL;
static int    s_rx_total   = 0;
static int    s_rx_written = 0;
static char   s_rx_topic[128] = {0};

// ── Cola de mensajes ──────────────────────────────────────────
typedef struct {
    char  *topic;
    char  *payload;
    size_t len;
} kx_msg_t;

// ── Lista de controles detectados ─────────────────────────────
#define KX_MAX_CONTROLS 64

static char s_control_names[KX_MAX_CONTROLS][64];
static int  s_control_count  = 0;

// ── Defines de cola y backpressure ────────────────────────────
#define QUEUE_BASE_SIZE              16
#define QUEUE_PER_CONTROL             5
#define KX_QUEUE_BACKPRESSURE_MAX    10
#define KX_QUEUE_BACKPRESSURE_HEAP   (1024 * 1024)
#define KX_BACKPRESSURE_DELAY_MS     100

static QueueHandle_t s_msg_queue = NULL;

// ── Forward declarations ──────────────────────────────────────
static void _track_control_from_topic(const char *topic);
static void _log_controls_list(void);



static double _ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// ── Redimensionar cola ────────────────────────────────────────
void kx_mqtt_resize_queue(int num_controls)
{
    int new_size = QUEUE_BASE_SIZE + (num_controls * QUEUE_PER_CONTROL);

    ESP_LOGI(TAG, "resizing queue: %d controls → %d slots (~%d bytes)",
             num_controls, new_size, new_size * (int)sizeof(kx_msg_t));

    QueueHandle_t new_queue = xQueueCreate(new_size, sizeof(kx_msg_t));

    if (!new_queue) {
        while (new_size > QUEUE_BASE_SIZE) {
            new_size /= 2;
            ESP_LOGW(TAG, "queue alloc failed, retrying with %d slots", new_size);
            new_queue = xQueueCreate(new_size, sizeof(kx_msg_t));
            if (new_queue) break;
        }
        if (!new_queue) {
            ESP_LOGE(TAG, "queue resize totally failed, keeping current");
            return;
        }
        ESP_LOGW(TAG, "queue limited to %d slots — some messages may drop", new_size);
    }

    kx_msg_t msg;
    int migrated = 0;
    int dropped  = 0;
    while (xQueueReceive(s_msg_queue, &msg, 0) == pdTRUE) {
        if (xQueueSend(new_queue, &msg, 0) == pdTRUE) {
            migrated++;
        } else {
            ESP_LOGW(TAG, "migration drop: topic=%s", msg.topic);
            free(msg.topic);
            free(msg.payload);
            dropped++;
        }
    }

    QueueHandle_t old_queue = s_msg_queue;
    s_msg_queue = new_queue;
    vQueueDelete(old_queue);

    ESP_LOGI(TAG, "queue ready: %d slots migrated=%d dropped=%d",
             new_size, migrated, dropped);
}

// ── Extrae el nombre del control del topic y lo acumula ───────
static void _track_control_from_topic(const char *topic)
{
    const char *marker = strstr(topic, "/controls/");
    if (!marker) return;

    const char *control_start = marker + strlen("/controls/");
    const char *slash = strchr(control_start, '/');
    size_t name_len = slash ? (size_t)(slash - control_start)
                            : strlen(control_start);

    if (name_len == 0 || name_len >= sizeof(s_control_names[0])) return;

    for (int i = 0; i < s_control_count; i++) {
        if (strncmp(s_control_names[i], control_start, name_len) == 0
            && s_control_names[i][name_len] == '\0') {
            return;
        }
    }

    if (s_control_count >= KX_MAX_CONTROLS) {
        ESP_LOGW(TAG, "control list full (%d), ignoring: %.*s",
                 KX_MAX_CONTROLS, (int)name_len, control_start);
        return;
    }

    strncpy(s_control_names[s_control_count], control_start, name_len);
    s_control_names[s_control_count][name_len] = '\0';
    s_control_count++;

    ESP_LOGI(TAG, "control detected [%d]: %s",
             s_control_count, s_control_names[s_control_count - 1]);
}

// ── Imprime la lista completa de controles detectados ─────────
static void _log_controls_list(void)
{
    // quitar el guard s_controls_listed
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "CONTROLS DETECTED (%d total):", s_control_count);
    for (int i = 0; i < s_control_count; i++) {
        ESP_LOGI(TAG, "  [%02d] %s", i + 1, s_control_names[i]);
    }
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

// ── Tarea de procesamiento ────────────────────────────────────
static void _processing_task(void *arg)
{
    kx_msg_t msg;
    ESP_LOGI(TAG, "processing task started");

    while (1) {
        if (xQueueReceive(s_msg_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "processing: topic=%s size=%d heap=%lu",
                     msg.topic, (int)msg.len,
                     (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

            if (s_msg_cb) {
                s_msg_cb(msg.topic, msg.payload, msg.len);
            }

            _track_control_from_topic(msg.topic);

            const char *entities_suffix = strstr(msg.topic, "/entities");
            if (entities_suffix && strcmp(entities_suffix, "/entities") == 0) {
                _log_controls_list();
            }

            free(msg.topic);
            free(msg.payload);
        }
    }
}

// ── Publica device-status online con IP ───────────────────────
static void _publish_device_status_online(void)
{
    char topic[128];
    char payload[512];
    char ip_str[16] = "0.0.0.0";

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip_info.ip));
        }
    }

    snprintf(topic, sizeof(topic), "%s/connection/status", KX_DEVICE_UUID);

    snprintf(payload, sizeof(payload),
        "{"
        "\"_type\": \"device-status\","
        "\"device_connection_state\": \"online\","
        "\"device_connection_ipinfo\": {"
            "\"ip\": \"%s\","
            "\"fw_ver\": \"%s\","
            "\"device_id\": \"%s\""
        "},"
        "\"ts\": %.3f"
        "}",
        ip_str,
        KX_FW_VERSION,
        KX_DEVICE_UUID,
        _ts()
    );

    esp_mqtt_client_publish(s_client, topic, payload, 0, 1, 1);
    ESP_LOGI(TAG, "device-status online → %s (ip=%s)", topic, ip_str);
}

// ── Suscripciones ─────────────────────────────────────────────
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

// ── Backpressure ──────────────────────────────────────────────
static void _wait_backpressure(void)
{
    int      queue_used = (int)uxQueueMessagesWaiting(s_msg_queue);
    uint32_t heap_free  = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    bool     triggered  = false;

    while (queue_used > KX_QUEUE_BACKPRESSURE_MAX ||
           heap_free  < KX_QUEUE_BACKPRESSURE_HEAP) {

        if (!triggered) {
            ESP_LOGW(TAG, "backpressure: queue=%d heap=%lu — "
                     "waiting for kx_processing to catch up",
                     queue_used, (unsigned long)heap_free);
            triggered = true;
        }

        vTaskDelay(pdMS_TO_TICKS(KX_BACKPRESSURE_DELAY_MS));

        queue_used = (int)uxQueueMessagesWaiting(s_msg_queue);
        heap_free  = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    }

    if (triggered) {
        ESP_LOGI(TAG, "backpressure released: queue=%d heap=%lu",
                 queue_used, (unsigned long)heap_free);
    }
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
        _publish_device_status_online();
        _subscribe_all();
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        kx_system_set_mqtt_state(KX_MQTT_DISCONNECTED);
        kx_system_inc_reconnect_count();
        ESP_LOGW(TAG, "DISCONNECTED (reconnect #%" PRIu32 ")",
                 kx_system_reconnect_count());
        break;

    case MQTT_EVENT_DATA:
        if (ev->topic_len > 0) {
            _wait_backpressure();

            size_t tlen = ev->topic_len < sizeof(s_rx_topic) - 1
                        ? ev->topic_len : sizeof(s_rx_topic) - 1;
            memcpy(s_rx_topic, ev->topic, tlen);
            s_rx_topic[tlen] = '\0';

            if (s_rx_buf) {
                ESP_LOGW(TAG, "discarding incomplete previous buffer");
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
                uint32_t safe = available > (512 * 1024)
                                ? available - (512 * 1024) : 0;
                ESP_LOGW(TAG, "OOM: need %d — fallback a %lu bytes",
                         s_rx_total, (unsigned long)safe);
                if (safe > 0) {
                    s_rx_buf = malloc(safe + 1);
                }
                if (s_rx_buf) {
                    s_rx_total = (int)safe;
                    ESP_LOGW(TAG, "buffer parcial: %d de %d bytes [TRUNCADO]",
                             s_rx_total, ev->total_data_len);
                } else {
                    ESP_LOGE(TAG, "OOM total: payload descartado");
                    s_rx_total = 0;
                }
            }
        }

        if (s_rx_buf && ev->data && ev->data_len > 0) {
            int space   = s_rx_total - s_rx_written;
            int to_copy = ev->data_len < space ? ev->data_len : space;
            if (to_copy > 0) {
                memcpy(s_rx_buf + s_rx_written, ev->data, to_copy);
                s_rx_written += to_copy;
            }
            ESP_LOGD(TAG, "recibiendo... %d/%d (%.1f%%)",
                     ev->current_data_offset + ev->data_len,
                     ev->total_data_len,
                     (float)(ev->current_data_offset + ev->data_len) * 100.0f
                     / ev->total_data_len);
        }

        if (ev->current_data_offset + ev->data_len >= ev->total_data_len) {
            if (s_rx_buf) {
                s_rx_buf[s_rx_written] = '\0';

                ESP_LOGI(TAG, "RX complete topic=%s size=%d heap=%lu%s",
                         s_rx_topic, s_rx_written,
                         (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                         s_rx_written < ev->total_data_len ? " [TRUNCADO]" : "");

                kx_msg_t msg = {
                    .topic   = strdup(s_rx_topic),
                    .payload = s_rx_buf,
                    .len     = s_rx_written,
                };
                s_rx_buf = NULL;

                if (xQueueSend(s_msg_queue, &msg, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "queue full (%d slots used), dropping: %s",
                             (int)uxQueueMessagesWaiting(s_msg_queue),
                             s_rx_topic);
                    free(msg.topic);
                    free(msg.payload);
                }
            } else {
                ESP_LOGE(TAG, "RX complete sin buffer — descartado");
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
    s_msg_cb    = on_message;
    s_msg_queue = xQueueCreate(QUEUE_BASE_SIZE, sizeof(kx_msg_t));

    if (!s_msg_queue) {
        ESP_LOGE(TAG, "Failed to create message queue");
        return ESP_FAIL;
    }

    BaseType_t ret = xTaskCreate(
        _processing_task,
        "kx_processing",
        16384,
        NULL,
        4,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create processing task");
        return ESP_FAIL;
    }

    // LWT
    snprintf(s_lwt_topic, sizeof(s_lwt_topic),
             "%s/connection/status", KX_DEVICE_UUID);
    snprintf(s_lwt_payload, sizeof(s_lwt_payload),
             "{\"_type\": \"device-status\","
             "\"device_connection_state\": \"offline\","
             "\"ts\": 0}");

    // Client ID
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "%s", KX_DEVICE_UUID);

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri                   = KX_MQTT_BROKER_URI,
        .credentials.client_id                = client_id,
        .credentials.username                 = KX_MQTT_USERNAME,
        .credentials.authentication.password  = KX_MQTT_PASSWORD,

        // ── TLS ──────────────────────────────────────────────
        // OPCIÓN A: sin TLS → URI mqtt://host:1883
        // OPCIÓN B: TLS con CA reconocida (Let's Encrypt)
        //           → URI mqtts://host:8883 + descomentar:
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,

        // ── Sesión ───────────────────────────────────────────
        .session.keepalive         = KX_MQTT_KEEPALIVE_S,
        .session.last_will.topic   = s_lwt_topic,
        .session.last_will.msg     = s_lwt_payload,
        .session.last_will.qos     = 1,
        .session.last_will.retain  = 1,

        // ── Red ──────────────────────────────────────────────
        .network.reconnect_timeout_ms = KX_MQTT_RECONNECT_MIN_MS,

        // ── Buffer de recepción ───────────────────────────────
        .buffer.size = KX_PAYLOAD_MAX_BYTES,
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