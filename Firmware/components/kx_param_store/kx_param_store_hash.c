#include "kx_param_store_internal.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "kx_param_store";

// =============================================================
// kx_param_store_hash.c — Operaciones de los tres niveles de hash
//
//   NIVEL 1 — controles      (kx_ctrl_hash_t)
//   NIVEL 2 — params         (kx_param_hash_t)
//   NIVEL 3 — registros      (kx_reg_hash_t)
//
// Estado global compartido (s_hash) vive aquí; el resto de
// archivos del componente acceden a través de las funciones
// declaradas en kx_param_store_internal.h
// =============================================================

kx_ctrl_hash_t s_hash;
bool           s_initialized = false;

// =============================================================
// PSRAM-aware alloc
// =============================================================
void *kx_psram_alloc(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(size);
    return p;
}

// =============================================================
// ── NIVEL 1 — hash de controles ──────────────────────────────
// =============================================================
static inline uint32_t _ctrl_hash(int control_id)
{
    uint32_t k = (uint32_t)control_id;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = (k >> 16) ^ k;
    return k & (KX_CTRL_HASH_BUCKETS - 1);
}

// =============================================================
// ── NIVEL 2 — hash de params ─────────────────────────────────
// =============================================================
static inline uint32_t _param_hash(int param_id)
{
    uint32_t k = (uint32_t)param_id;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = (k >> 16) ^ k;
    return k & (KX_PARAM_HASH_BUCKETS - 1);
}

// =============================================================
// ── NIVEL 3 — hash de registros Modbus ───────────────────────
// =============================================================
static inline uint32_t _reg_hash_fn(uint16_t reg,
                                     uint8_t  fc_read,
                                     uint8_t  fc_write)
{
    uint32_t k = ((uint32_t)reg     << 16)
               | ((uint32_t)fc_read <<  8)
               |  (uint32_t)fc_write;
    k ^= (k >> 16);
    k  = (k * 0x45d9f3bU) & 0xFFFFFFFFU;
    k ^= (k >> 16);
    k  = (k * 0x45d9f3bU) & 0xFFFFFFFFU;
    k ^= (k >> 16);
    return k & (KX_REG_HASH_BUCKETS - 1);
}

static inline bool _reg_key_eq(const kx_reg_key_t *a,
                                uint16_t reg,
                                uint8_t  fc_read,
                                uint8_t  fc_write)
{
    return (a->reg == reg) && (a->fc_read == fc_read) && (a->fc_write == fc_write);
}

kx_reg_entry_t *kx_reg_find_mutable(kx_reg_hash_t *rh,
                                     uint16_t reg,
                                     uint8_t  fc_read,
                                     uint8_t  fc_write)
{
    uint32_t idx = _reg_hash_fn(reg, fc_read, fc_write);
    kx_reg_node_t *node = rh->buckets[idx];
    while (node) {
        if (_reg_key_eq(&node->entry.key, reg, fc_read, fc_write))
            return &node->entry;
        node = node->next;
    }
    return NULL;
}

const kx_reg_entry_t *kx_reg_find(const kx_reg_hash_t *rh,
                                   uint16_t reg,
                                   uint8_t  fc_read,
                                   uint8_t  fc_write)
{
    return kx_reg_find_mutable((kx_reg_hash_t *)rh, reg, fc_read, fc_write);
}

kx_reg_entry_t *kx_reg_ensure(kx_reg_hash_t *rh,
                               uint16_t reg,
                               uint8_t  fc_read,
                               uint8_t  fc_write)
{
    kx_reg_entry_t *existing = kx_reg_find_mutable(rh, reg, fc_read, fc_write);
    if (existing) return existing;

    kx_reg_node_t *node = kx_psram_alloc(sizeof(kx_reg_node_t));
    if (!node) {
        ESP_LOGE(TAG, "reg_hash OOM: reg=0x%04x fc_r=%u fc_w=%u",
                 reg, fc_read, fc_write);
        return NULL;
    }
    node->entry.key.reg      = reg;
    node->entry.key.fc_read  = fc_read;
    node->entry.key.fc_write = fc_write;
    node->entry.value        = FLT_MAX;
    node->entry.ts_last_read = 0;
    node->entry.last_write_value = FLT_MAX;
    node->entry.ts_last_write    = 0;

    uint32_t idx = _reg_hash_fn(reg, fc_read, fc_write);
    node->next       = rh->buckets[idx];
    rh->buckets[idx] = node;
    rh->count++;
    return &node->entry;
}

void kx_reg_clear_all(kx_reg_hash_t *rh)
{
    for (int i = 0; i < KX_REG_HASH_BUCKETS; i++) {
        kx_reg_node_t *node = rh->buckets[i];
        while (node) {
            kx_reg_node_t *tmp = node->next;
            free(node);
            node = tmp;
        }
        rh->buckets[i] = NULL;
    }
    rh->count = 0;
}

// =============================================================
// ── NIVEL 1 — operaciones sobre controles ────────────────────
// =============================================================
kx_control_t *kx_ctrl_find(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];
    while (node) {
        if (node->ctrl.control_id == control_id) return &node->ctrl;
        node = node->next;
    }
    return NULL;
}

// Busca o crea, RESETEANDO params y regs si ya existe.
kx_control_t *kx_ctrl_find_or_create(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];

    while (node) {
        if (node->ctrl.control_id == control_id) {
            for (int b = 0; b < KX_PARAM_HASH_BUCKETS; b++) {
                kx_param_node_t *pn = node->ctrl.params.buckets[b];
                while (pn) {
                    kx_param_node_t *tmp = pn->next;
                    free(pn); pn = tmp;
                }
                node->ctrl.params.buckets[b] = NULL;
            }
            node->ctrl.params.count = 0;
            kx_reg_clear_all(&node->ctrl.regs);
            node->ctrl.entities_ready = false;
            return &node->ctrl;
        }
        node = node->next;
    }

    if (s_hash.count >= KX_PARAM_MAX_CONTROLS) {
        ESP_LOGE(TAG, "hash full: %d controls (max %d)",
                 s_hash.count, KX_PARAM_MAX_CONTROLS);
        return NULL;
    }

    kx_ctrl_node_t *new_node = kx_psram_alloc(sizeof(kx_ctrl_node_t));
    if (!new_node) { ESP_LOGE(TAG, "OOM creating control node"); return NULL; }
    memset(new_node, 0, sizeof(kx_ctrl_node_t));
    new_node->ctrl.control_id = control_id;
    new_node->ctrl.proto      = KX_PROTO_RTU;

    new_node->next      = s_hash.buckets[idx];
    s_hash.buckets[idx] = new_node;
    s_hash.count++;

    ESP_LOGI(TAG, "ctrl hash: inserted control_id=%d bucket=%u count=%d",
             control_id, idx, s_hash.count);
    return &new_node->ctrl;
}

// Busca o crea SIN resetear. Usado por setters de metadatos.
kx_control_t *kx_ctrl_find_or_create_no_reset(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];
    while (node) {
        if (node->ctrl.control_id == control_id)
            return &node->ctrl;
        node = node->next;
    }
    if (s_hash.count >= KX_PARAM_MAX_CONTROLS) {
        ESP_LOGE(TAG, "hash full: %d controls", s_hash.count);
        return NULL;
    }
    kx_ctrl_node_t *new_node = kx_psram_alloc(sizeof(kx_ctrl_node_t));
    if (!new_node) return NULL;
    memset(new_node, 0, sizeof(kx_ctrl_node_t));
    new_node->ctrl.control_id = control_id;
    new_node->ctrl.proto      = KX_PROTO_RTU;
    new_node->next      = s_hash.buckets[idx];
    s_hash.buckets[idx] = new_node;
    s_hash.count++;
    return &new_node->ctrl;
}

// =============================================================
// ── NIVEL 2 — operaciones sobre params ───────────────────────
// =============================================================
esp_err_t kx_param_insert(kx_control_t *ctrl, const kx_param_t *param)
{
    uint32_t idx = _param_hash(param->param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];

    while (node) {
        if (node->param.param_id == param->param_id) {
            int64_t saved_ts  = node->param.ts_last_read;
            float   saved_lpv = node->param.last_published_value;
            memcpy(&node->param, param, sizeof(kx_param_t));
            node->param.ts_last_read         = saved_ts;
            node->param.last_published_value = saved_lpv;
            kx_reg_ensure(&ctrl->regs,
                          (uint16_t)param->reg,
                          (uint8_t)param->function_read,
                          (uint8_t)param->function_write);
            return ESP_OK;
        }
        node = node->next;
    }

    if (ctrl->params.count >= KX_PARAM_MAX_PER_CONTROL) {
        ESP_LOGW(TAG, "ctrl %d: param limit (%d)", ctrl->control_id, KX_PARAM_MAX_PER_CONTROL);
        return ESP_ERR_NO_MEM;
    }

    kx_param_node_t *new_node = kx_psram_alloc(sizeof(kx_param_node_t));
    if (!new_node) { ESP_LOGE(TAG, "OOM param node"); return ESP_ERR_NO_MEM; }
    memcpy(&new_node->param, param, sizeof(kx_param_t));
    new_node->param.ts_last_read         = 0;
    new_node->param.last_published_value = FLT_MAX;
    new_node->next            = ctrl->params.buckets[idx];
    ctrl->params.buckets[idx] = new_node;
    ctrl->params.count++;

    kx_reg_ensure(&ctrl->regs,
                  (uint16_t)param->reg,
                  (uint8_t)param->function_read,
                  (uint8_t)param->function_write);
    return ESP_OK;
}

const kx_param_t *kx_param_find(const kx_control_t *ctrl, int param_id)
{
    uint32_t idx = _param_hash(param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];
    while (node) {
        if (node->param.param_id == param_id) return &node->param;
        node = node->next;
    }
    return NULL;
}

kx_param_t *kx_param_find_mutable(kx_control_t *ctrl, int param_id)
{
    uint32_t idx = _param_hash(param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];
    while (node) {
        if (node->param.param_id == param_id) return &node->param;
        node = node->next;
    }
    return NULL;
}