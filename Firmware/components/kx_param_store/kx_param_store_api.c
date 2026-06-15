#include "kx_param_store_internal.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "../../../main/kx_config.h"

static const char *TAG = "kx_param_store";

// =============================================================
// kx_param_store_api.c — Ciclo de vida, parseo JSON, foreach,
//                        getters/setters, API de nivel 3 (regs),
//                        completitud y diagnóstico
// =============================================================

static int                    s_expected    = 0;
static kx_param_progress_cb_t s_progress_cb = NULL;

// =============================================================
// Helpers de parseo JSON
// =============================================================
static float _get_float(cJSON *obj, const char *key, float def)
{
    cJSON *item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsNumber(item)) return (float)item->valuedouble;
    return def;
}

static int _get_int(cJSON *obj, const char *key, int def)
{
    cJSON *item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsNumber(item)) return (int)item->valuedouble;
    return def;
}

static void _get_str(cJSON *obj, const char *key, char *buf, size_t len)
{
    cJSON *item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsString(item) && item->valuestring)
        snprintf(buf, len, "%s", item->valuestring);
    else
        buf[0] = '\0';
}

// =============================================================
// Barra de progreso ASCII
// =============================================================
#define PROGRESS_BAR_WIDTH 30

void kx_print_progress(int control_id, int done, int total)
{
    if (total <= 0) return;
    int pct    = (done * 100) / total;
    int filled = (done * PROGRESS_BAR_WIDTH) / total;
    char bar[PROGRESS_BAR_WIDTH + 1];
    for (int i = 0; i < PROGRESS_BAR_WIDTH; i++)
        bar[i] = (i < filled) ? '#' : '-';
    bar[PROGRESS_BAR_WIDTH] = '\0';
    printf("\r[entities] ctrl=%d [%s] %3d%% (%d/%d params)",
           control_id, bar, pct, done, total);
    fflush(stdout);
    if (done >= total) { printf("\n"); fflush(stdout); }
}

// =============================================================
// API pública — ciclo de vida
// =============================================================
void kx_param_store_init(void)
{
    if (s_initialized) return;
    memset(&s_hash, 0, sizeof(s_hash));
    kx_nvs_storage_init();
    s_initialized = true;
    ESP_LOGI(TAG,
             "hash store initialized "
             "(ctrl_buckets=%d param_buckets=%d reg_buckets=%d)",
             KX_CTRL_HASH_BUCKETS, KX_PARAM_HASH_BUCKETS, KX_REG_HASH_BUCKETS);
}

// =============================================================
// Parseo de entities desde JSON
// =============================================================
esp_err_t kx_param_store_parse(const char *payload, size_t len, int control_id)
{
    if (!s_initialized) kx_param_store_init();

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse failed for control %d", control_id);
        return ESP_FAIL;
    }

    kx_control_t *ctrl = kx_ctrl_find_or_create(control_id);
    if (!ctrl) { cJSON_Delete(root); return ESP_FAIL; }

    cJSON *regs = cJSON_GetObjectItem(root, "control_regs");
    if (!regs || !cJSON_IsArray(regs)) {
        ESP_LOGW(TAG, "control %d: no control_regs array", control_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    int total    = cJSON_GetArraySize(regs);
    int inserted = 0;
    ESP_LOGI(TAG, "control %d: parsing %d params...", control_id, total);
    kx_print_progress(control_id, 0, total);

    for (int i = 0; i < total; i++) {
        cJSON *reg = cJSON_GetArrayItem(regs, i);
        if (!reg) continue;

        kx_param_t p = {0};
        p.param_id       = _get_int  (reg, "control_parameter_id",                 0);
        p.reg            = _get_int  (reg, "control_parameter_register",            0);
        p.function_read  = _get_int  (reg, "control_parameter_function_read",       0);
        p.function_write = _get_int  (reg, "control_parameter_function_write",      0);
        p.minvalue       = _get_float(reg, "control_parameter_minvalue",         0.0f);
        p.maxvalue       = _get_float(reg, "control_parameter_maxvalue",       100.0f);
        p.offset         = _get_float(reg, "control_parameter_offset",           0.0f);
        p.mask           = _get_int  (reg, "control_parameter_mask",                0);
        p.view           = _get_int  (reg, "control_parameter_view",                1);
        p.sampling       = _get_int  (reg, "control_parameter_sampling",           60);

        _get_str(reg, "control_parameter_name",            p.name,     sizeof(p.name));
        _get_str(reg, "control_parameter_category_system", p.category, sizeof(p.category));
        _get_str(reg, "control_parameter_length",          p.length,   sizeof(p.length));

        cJSON *add = cJSON_GetObjectItem(reg, "control_parameter_addition");
        p.addition = (add && cJSON_IsNumber(add)) ? (float)add->valuedouble : 0.0f;

        p.ts_last_read         = 0;
        p.last_published_value = FLT_MAX;

        if (p.param_id <= 0) continue;

        if (kx_param_insert(ctrl, &p) == ESP_OK) {
            inserted++;
            kx_print_progress(control_id, inserted, total);
            if (s_progress_cb) s_progress_cb(control_id, inserted, total);
        }
    }

    cJSON_Delete(root);
    ctrl->entities_ready = true;

    ESP_LOGI(TAG,
             "control %d: stored %d/%d params | reg_entries=%d | heap=%lu",
             control_id, inserted, total,
             ctrl->regs.count,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    return ESP_OK;
}

// =============================================================
// API pública — consultas nivel 1 y 2
// =============================================================
const kx_control_t *kx_param_store_get_ctrl(int control_id)
{
    return kx_ctrl_find(control_id);
}

const kx_param_t *kx_param_store_get_param(int control_id, int param_id)
{
    const kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return NULL;
    return kx_param_find(ctrl, param_id);
}

kx_param_t *kx_param_store_get_param_mutable(int control_id, int param_id)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return NULL;
    return kx_param_find_mutable(ctrl, param_id);
}

int kx_param_store_count(void) { return s_hash.count; }

void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data)
{
    if (!cb) return;
    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            kx_control_t *ctrl = &cn->ctrl;
            for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS; pi++) {
                kx_param_node_t *pn = ctrl->params.buckets[pi];
                while (pn) {
                    cb(ctrl->control_id, &pn->param, user_data);
                    pn = pn->next;
                }
            }
            cn = cn->next;
        }
    }
}

// =============================================================
// API pública — Nivel 3
// =============================================================
kx_reg_entry_t *kx_param_store_reg_upsert_read(int      control_id,
                                                uint16_t reg,
                                                uint8_t  fc_read,
                                                uint8_t  fc_write,
                                                float    value,
                                                int64_t  ts_ms)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return NULL;

    kx_reg_entry_t *entry = kx_reg_find_mutable(&ctrl->regs, reg, fc_read, fc_write);
    if (!entry) {
        ESP_LOGW(TAG, "reg_upsert_read: creating entry ctrl=%d reg=0x%04x",
                 control_id, reg);
        entry = kx_reg_ensure(&ctrl->regs, reg, fc_read, fc_write);
        if (!entry) return NULL;
    }
    entry->value        = value;
    entry->ts_last_read = ts_ms;
    return entry;
}

esp_err_t kx_param_store_reg_upsert_write(int      control_id,
                                           uint16_t reg,
                                           uint8_t  fc_read,
                                           uint8_t  fc_write,
                                           float    value,
                                           int64_t  ts_ms)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return ESP_ERR_NOT_FOUND;
    kx_reg_entry_t *entry = kx_reg_find_mutable(&ctrl->regs, reg, fc_read, fc_write);
    if (!entry) return ESP_ERR_NOT_FOUND;
    entry->last_write_value = value;
    entry->ts_last_write    = ts_ms;
    return ESP_OK;
}

const kx_reg_entry_t *kx_param_store_reg_get(int      control_id,
                                               uint16_t reg,
                                               uint8_t  fc_read,
                                               uint8_t  fc_write)
{
    const kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return NULL;
    return kx_reg_find(&ctrl->regs, reg, fc_read, fc_write);
}

kx_reg_entry_t *kx_param_store_reg_get_mutable(int      control_id,
                                                uint16_t reg,
                                                uint8_t  fc_read,
                                                uint8_t  fc_write)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return NULL;
    return kx_reg_find_mutable(&ctrl->regs, reg, fc_read, fc_write);
}

int kx_param_store_reg_count(int control_id)
{
    const kx_control_t *ctrl = kx_ctrl_find(control_id);
    return ctrl ? ctrl->regs.count : 0;
}

void kx_param_store_reg_foreach(int control_id, kx_reg_iter_cb_t cb, void *ud)
{
    if (!cb) return;
    const kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return;
    for (int i = 0; i < KX_REG_HASH_BUCKETS; i++) {
        kx_reg_node_t *node = ctrl->regs.buckets[i];
        while (node) {
            cb(control_id, &node->entry, ud);
            node = node->next;
        }
    }
}

void kx_param_store_reg_foreach_all(kx_reg_iter_cb_t cb, void *ud)
{
    if (!cb) return;
    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            kx_control_t *ctrl = &cn->ctrl;
            for (int ri = 0; ri < KX_REG_HASH_BUCKETS; ri++) {
                kx_reg_node_t *rn = ctrl->regs.buckets[ri];
                while (rn) {
                    cb(ctrl->control_id, &rn->entry, ud);
                    rn = rn->next;
                }
            }
            cn = cn->next;
        }
    }
}

// =============================================================
// Control de completitud
// =============================================================
void kx_param_store_set_expected(int count)
{
    s_expected = count;
    ESP_LOGI(TAG, "expecting %d controls", count);
}

bool kx_param_store_is_ready(void)
{
    if (s_expected <= 0) return false;
    int ready = 0;
    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            if (cn->ctrl.entities_ready && cn->ctrl.params.count > 0)
                ready++;
            cn = cn->next;
        }
    }
    return (ready >= s_expected);
}

// =============================================================
// Configuración adicional por control
// =============================================================
void kx_param_store_set_slave_addr(int control_id, int slave_addr)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) ctrl = kx_ctrl_find_or_create_no_reset(control_id);
    if (ctrl) {
        ctrl->slave_addr = slave_addr;
        ESP_LOGI(TAG, "slave_addr set: ctrl=%d → %d", control_id, slave_addr);
    }
}

void kx_param_store_set_uuid(int control_id, const char *uuid)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) ctrl = kx_ctrl_find_or_create_no_reset(control_id);
    if (ctrl && uuid)
        snprintf(ctrl->uuid, sizeof(ctrl->uuid), "%s", uuid);
}

// =============================================================
// update_ts
// =============================================================
double kx_param_store_get_update_ts(int control_id)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    return ctrl ? ctrl->update_ts : 0.0;
}

void kx_param_store_set_update_ts(int control_id, double ts)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) ctrl = kx_ctrl_find_or_create_no_reset(control_id);
    if (ctrl) {
        ctrl->update_ts = ts;
        ESP_LOGI(TAG, "update_ts set: ctrl=%d → %.3f", control_id, ts);
    }
}

// =============================================================
// Clear entities
// =============================================================
void kx_param_store_clear_entities(int control_id)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return;

    for (int b = 0; b < KX_PARAM_HASH_BUCKETS; b++) {
        kx_param_node_t *pn = ctrl->params.buckets[b];
        while (pn) {
            kx_param_node_t *tmp = pn->next;
            free(pn); pn = tmp;
        }
        ctrl->params.buckets[b] = NULL;
    }
    ctrl->params.count   = 0;
    ctrl->entities_ready = false;

    int reg_before = ctrl->regs.count;
    kx_reg_clear_all(&ctrl->regs);

    ESP_LOGI(TAG, "entities cleared ctrl=%d (ts=%.3f) params=0 regs=0 (was %d)",
             control_id, ctrl->update_ts, reg_before);
}

// =============================================================
// Progreso
// =============================================================
void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb)
{
    s_progress_cb = cb;
}

esp_err_t kx_param_store_set_ts_set(int control_id, int param_id, double ts)
{
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return ESP_ERR_NOT_FOUND;
    kx_param_t *p = kx_param_find_mutable(ctrl, param_id);
    if (!p) return ESP_ERR_NOT_FOUND;
    p->ts_set = ts;
    return ESP_OK;
}

// =============================================================
// API de transporte TCP
// =============================================================
void kx_param_store_set_tcp_endpoint(int         control_id,
                                      const char *ip,
                                      uint16_t    port)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) ctrl = kx_ctrl_find_or_create_no_reset(control_id);
    if (!ctrl) {
        ESP_LOGE(TAG, "set_tcp_endpoint: ctrl=%d not found", control_id);
        return;
    }
    if (ip && ip[0] != '\0' && port > 0) {
        snprintf(ctrl->tcp_ip, sizeof(ctrl->tcp_ip), "%s", ip);
        ctrl->tcp_port = port;
        ctrl->proto    = KX_PROTO_TCP;
        ESP_LOGI(TAG, "tcp_endpoint set: ctrl=%d → %s:%" PRIu16 " (TCP)",
                 control_id, ctrl->tcp_ip, ctrl->tcp_port);
    } else {
        ctrl->tcp_ip[0] = '\0';
        ctrl->tcp_port  = 0;
        ctrl->proto     = KX_PROTO_RTU;
        ESP_LOGI(TAG, "tcp_endpoint cleared: ctrl=%d → RTU", control_id);
    }
}

kx_proto_t kx_param_store_get_proto(int control_id)
{
    const kx_control_t *ctrl = kx_ctrl_find(control_id);
    return ctrl ? ctrl->proto : KX_PROTO_RTU;
}

esp_err_t kx_param_store_get_tcp_endpoint(int      control_id,
                                           char    *ip_out,
                                           uint16_t *port_out)
{
    const kx_control_t *ctrl = kx_ctrl_find(control_id);
    if (!ctrl) return ESP_ERR_NOT_FOUND;
    if (ctrl->proto != KX_PROTO_TCP) return ESP_ERR_NOT_FOUND;
    if (!ip_out || !port_out) return ESP_ERR_INVALID_ARG;
    snprintf(ip_out, KX_TCP_IP_LEN, "%s", ctrl->tcp_ip);
    *port_out = ctrl->tcp_port;
    return ESP_OK;
}

// =============================================================
// Diagnóstico
// =============================================================
//static void _print_active_sampling_cb(int ctrl_id, const kx_param_t *param, void *ud)
//{
//    (void)ud;
//    if (param->sampling > 0)
//        ESP_LOGI("STORE_DEBUG",
//                 "ctrl=%d param=%d reg=0x%04x sampling=%ds name=%s",
//                 ctrl_id, param->param_id, param->reg,
//                 param->sampling, param->name);
//}

//void kx_param_store_print_active_samplings(void)
//{
//    ESP_LOGI("STORE_DEBUG", "=== PARAMS CON SAMPLING > 0 ===");
//    kx_param_store_foreach(_print_active_sampling_cb, NULL);
//    ESP_LOGI("STORE_DEBUG", "===============================");
//}