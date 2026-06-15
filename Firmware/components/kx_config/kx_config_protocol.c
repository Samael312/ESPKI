#include "kx_config_handler.h"
#include "kx_config_internal.h"
#include "kx_system.h"
#include "kx_mqtt.h"
#include "../../main/kx_config.h"
#include "kx_param_store.h"
#include "kx_modbus_master.h"
#include "kx_modbus_tcp.h"
#include "esp_log.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "kx_config";

// =============================================================
// kx_config_protocol.c — Parseo de un control individual
//
// Configura:
//   - slave_addr / unit_id
//   - uuid
//   - protocolo (RTU o TCP) con su endpoint
//   - lanza entities-discovery si el update_ts es mayor
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
        control_id, uuid, kx_config_ts());
    kx_mqtt_publish(topic, payload, 1, 0);
    ESP_LOGI(TAG, "control-status online → ctrl=%d uuid=%s", control_id, uuid);
}

void kx_config_process_single_control(cJSON *ctrl_json, int hint_control_id)
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
        //kx_param_store_print_active_samplings();
    }

    if (need_discovery) {
        kx_config_request_entities(control_id);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}