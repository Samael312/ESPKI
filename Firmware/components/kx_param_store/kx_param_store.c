#include "kx_param_store.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>

#include "../../main/kx_config.h"

static const char *TAG = "kx_param_store";

// =============================================================
// NVS — namespace y claves
// =============================================================
#define NVS_NS_STORE     "kx_entities"
#define NVS_KEY_MAGIC    "magic"
#define NVS_KEY_UUID     "uuid"
#define NVS_KEY_FW       "fw"
#define NVS_KEY_COUNT    "count"
#define NVS_MAGIC_VALUE  0xE5712A04U

#define NVS_MAX_BLOB      3840U
#define NVS_CHUNK_PARAMS  (NVS_MAX_BLOB / sizeof(kx_param_t))
#define NVS_PARTITION     "storage"

// El nivel 3 (kx_reg_hash_t) NO se persiste en NVS: se reconstruye
// automáticamente al insertar cada kx_param_t (vía _param_insert).

typedef struct {
    int    control_id;
    int    slave_addr;
    int    count;
    double update_ts;
    char   uuid[64];
} kx_nvs_ctrl_hdr_t;

// =============================================================
// Estado global
// =============================================================
static kx_ctrl_hash_t         s_hash;
static bool                   s_initialized = false;
static int                    s_expected    = 0;
static kx_param_progress_cb_t s_progress_cb = NULL;

// =============================================================
// Asignador PSRAM con fallback a RAM interna
// =============================================================
static void *_psram_alloc(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(size);
    return p;
}

// =============================================================
// ── NIVEL 1 — funciones de hash de controles ─────────────────
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
// ── NIVEL 2 — funciones de hash de params ────────────────────
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
// ── NIVEL 3 — funciones de hash de registros Modbus ──────────
//
// La clave compuesta se empaqueta en un uint32_t:
//   bits[31:16] = reg       (dirección del registro)
//   bits[15: 8] = fc_read   (función de lectura,  0 si ninguna)
//   bits[ 7: 0] = fc_write  (función de escritura, 0 si ninguna)
//
// El hash distingue entradas que comparten la misma dirección
// de registro pero difieren en función de lectura o escritura,
// y también indexa registros sin ninguna función asignada (fc=0).
// =============================================================

// Genera el índice de bucket a partir de la clave compuesta.
// Usa el mismo algoritmo Murmur3-finalizer que los niveles 1 y 2
// para coherencia y rendimiento demostrado en ESP32.
static inline uint32_t _reg_hash_fn(uint16_t reg,
                                     uint8_t  fc_read,
                                     uint8_t  fc_write)
{
    // Empaquetar en 32 bits para un solo paso de hashing
    uint32_t k = ((uint32_t)reg     << 16)
               | ((uint32_t)fc_read <<  8)
               |  (uint32_t)fc_write;

    // Murmur3 finalizer — buen avalanche effect con claves pequeñas
    k ^= (k >> 16);
    k  = (k * 0x45d9f3bU) & 0xFFFFFFFFU;
    k ^= (k >> 16);
    k  = (k * 0x45d9f3bU) & 0xFFFFFFFFU;
    k ^= (k >> 16);

    // Máscara a KX_REG_HASH_BUCKETS — DEBE ser potencia de 2
    return k & (KX_REG_HASH_BUCKETS - 1);
}

// Comprueba si la clave de una entrada coincide con la buscada.
static inline bool _reg_key_eq(const kx_reg_key_t *a,
                                uint16_t reg,
                                uint8_t  fc_read,
                                uint8_t  fc_write)
{
    return (a->reg      == reg)
        && (a->fc_read  == fc_read)
        && (a->fc_write == fc_write);
}

// ── Búsqueda interna (mutable) ────────────────────────────────
static kx_reg_entry_t *_reg_find_mutable(kx_reg_hash_t *rh,
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

// ── Búsqueda interna (inmutable) ─────────────────────────────
static const kx_reg_entry_t *_reg_find(const kx_reg_hash_t *rh,
                                        uint16_t reg,
                                        uint8_t  fc_read,
                                        uint8_t  fc_write)
{
    return _reg_find_mutable((kx_reg_hash_t *)rh, reg, fc_read, fc_write);
}

// ── Inserción de un registro nuevo (valores iniciales) ────────
//
// Se llama SIEMPRE desde _param_insert para garantizar que
// TODOS los registros — incluyendo aquellos con fc_read=0 o
// fc_write=0 — estén presentes en el nivel 3.
//
// Si la clave ya existe (otro param comparte el mismo triplete),
// la función retorna la entrada existente sin modificarla:
// el valor y los timestamps solo los actualiza el driver Modbus.
//
// Retorna puntero a la entrada (nueva o existente), NULL si OOM.
static kx_reg_entry_t *_reg_ensure(kx_reg_hash_t *rh,
                                    uint16_t reg,
                                    uint8_t  fc_read,
                                    uint8_t  fc_write)
{
    // 1. ¿Ya existe esta clave compuesta?
    kx_reg_entry_t *existing = _reg_find_mutable(rh, reg, fc_read, fc_write);
    if (existing) return existing;   // reusar — NO resetear valores

    // 2. Crear nodo nuevo
    kx_reg_node_t *node = _psram_alloc(sizeof(kx_reg_node_t));
    if (!node) {
        ESP_LOGE(TAG, "reg_hash OOM: reg=0x%04x fc_r=%u fc_w=%u",
                 reg, fc_read, fc_write);
        return NULL;
    }

    // Inicializar clave
    node->entry.key.reg      = reg;
    node->entry.key.fc_read  = fc_read;
    node->entry.key.fc_write = fc_write;

    // Inicializar estado de lectura: "nunca leído"
    node->entry.value        = FLT_MAX;
    node->entry.ts_last_read = 0;

    // Inicializar estado de escritura: "nunca escrito"
    node->entry.last_write_value = FLT_MAX;
    node->entry.ts_last_write    = 0;

    // Insertar en la cabeza de la lista del bucket (O(1))
    uint32_t idx = _reg_hash_fn(reg, fc_read, fc_write);
    node->next        = rh->buckets[idx];
    rh->buckets[idx]  = node;
    rh->count++;

    return &node->entry;
}

// ── Liberación completa de todos los nodos del nivel 3 ────────
static void _reg_clear_all(kx_reg_hash_t *rh)
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
// ── NIVEL 1 — operaciones sobre el hash de controles ─────────
// =============================================================
static kx_control_t *_ctrl_find_or_create(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];

    while (node) {
        if (node->ctrl.control_id == control_id) {
            // Reinicializar nivel 2 (params)
            for (int b = 0; b < KX_PARAM_HASH_BUCKETS; b++) {
                kx_param_node_t *pn = node->ctrl.params.buckets[b];
                while (pn) {
                    kx_param_node_t *tmp = pn->next;
                    free(pn);
                    pn = tmp;
                }
                node->ctrl.params.buckets[b] = NULL;
            }
            node->ctrl.params.count   = 0;

            // Reinicializar nivel 3 (registros)
            _reg_clear_all(&node->ctrl.regs);

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

    kx_ctrl_node_t *new_node = _psram_alloc(sizeof(kx_ctrl_node_t));
    if (!new_node) {
        ESP_LOGE(TAG, "OOM creating control node");
        return NULL;
    }
    memset(new_node, 0, sizeof(kx_ctrl_node_t));
    new_node->ctrl.control_id = control_id;
    // params.buckets y regs.buckets quedan a NULL (memset)

    new_node->next      = s_hash.buckets[idx];
    s_hash.buckets[idx] = new_node;
    s_hash.count++;

    ESP_LOGI(TAG, "ctrl hash: inserted control_id=%d bucket=%u count=%d",
             control_id, idx, s_hash.count);
    return &new_node->ctrl;
}

static kx_control_t *_ctrl_find(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];
    while (node) {
        if (node->ctrl.control_id == control_id) return &node->ctrl;
        node = node->next;
    }
    return NULL;
}

// =============================================================
// ── NIVEL 2 — operaciones sobre el hash de params ────────────
// =============================================================
static esp_err_t _param_insert(kx_control_t *ctrl, const kx_param_t *param)
{
    // ── 1. Insertar / actualizar en el hash de params (nivel 2) ──
    uint32_t idx = _param_hash(param->param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];

    while (node) {
        if (node->param.param_id == param->param_id) {
            // Preservar campos de runtime antes de sobrescribir config
            int64_t saved_ts   = node->param.ts_last_read;
            float   saved_lpv  = node->param.last_published_value;

            memcpy(&node->param, param, sizeof(kx_param_t));

            node->param.ts_last_read         = saved_ts;
            node->param.last_published_value = saved_lpv;

            // ── 2. Nivel 3: garantizar que el registro sigue indexado ──
            // (puede cambiar fc_read/fc_write en un re-parse)
            _reg_ensure(&ctrl->regs,
                        (uint16_t)param->reg,
                        (uint8_t)param->function_read,
                        (uint8_t)param->function_write);
            return ESP_OK;
        }
        node = node->next;
    }

    if (ctrl->params.count >= KX_PARAM_MAX_PER_CONTROL) {
        ESP_LOGW(TAG, "ctrl %d: param limit reached (%d)",
                 ctrl->control_id, KX_PARAM_MAX_PER_CONTROL);
        return ESP_ERR_NO_MEM;
    }

    kx_param_node_t *new_node = _psram_alloc(sizeof(kx_param_node_t));
    if (!new_node) {
        ESP_LOGE(TAG, "OOM creating param node");
        return ESP_ERR_NO_MEM;
    }
    memcpy(&new_node->param, param, sizeof(kx_param_t));

    // Inicializar campos de runtime para nodos nuevos
    new_node->param.ts_last_read         = 0;       // nunca leído
    new_node->param.last_published_value = FLT_MAX; // nunca publicado

    new_node->next            = ctrl->params.buckets[idx];
    ctrl->params.buckets[idx] = new_node;
    ctrl->params.count++;

    // ── 2. Nivel 3: registrar el par (reg, fc_read, fc_write) ────
    // Se indexan TODOS los registros sin discriminar si fc_read
    // o fc_write son 0 (requisito: hash completo de registros).
    kx_reg_entry_t *reg_entry = _reg_ensure(
        &ctrl->regs,
        (uint16_t)param->reg,
        (uint8_t)param->function_read,
        (uint8_t)param->function_write
    );

    if (reg_entry) {
        ESP_LOGD(TAG,
                 "reg_hash: ctrl=%d  reg=0x%04x fc_r=%u fc_w=%u  "
                 "(reg_entries=%d)",
                 ctrl->control_id,
                 (unsigned)param->reg,
                 (unsigned)param->function_read,
                 (unsigned)param->function_write,
                 ctrl->regs.count);
    } else {
        // No es fatal: el param ya está guardado en nivel 2
        ESP_LOGW(TAG, "reg_hash: OOM for ctrl=%d param=%d — reg index skipped",
                 ctrl->control_id, param->param_id);
    }

    return ESP_OK;
}

static const kx_param_t *_param_find(const kx_control_t *ctrl, int param_id)
{
    uint32_t idx = _param_hash(param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];
    while (node) {
        if (node->param.param_id == param_id) return &node->param;
        node = node->next;
    }
    return NULL;
}

static kx_param_t *_param_find_mutable(kx_control_t *ctrl, int param_id)
{
    uint32_t idx = _param_hash(param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];
    while (node) {
        if (node->param.param_id == param_id) return &node->param;
        node = node->next;
    }
    return NULL;
}

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


static kx_control_t *_ctrl_find_or_create_no_reset(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];

    while (node) {
        if (node->ctrl.control_id == control_id)
            return &node->ctrl;  // ← devuelve sin tocar nada
        node = node->next;
    }

    // No existe → crear nuevo (igual que antes)
    if (s_hash.count >= KX_PARAM_MAX_CONTROLS) {
        ESP_LOGE(TAG, "hash full: %d controls", s_hash.count);
        return NULL;
    }
    kx_ctrl_node_t *new_node = _psram_alloc(sizeof(kx_ctrl_node_t));
    if (!new_node) return NULL;
    memset(new_node, 0, sizeof(kx_ctrl_node_t));
    new_node->ctrl.control_id = control_id;
    new_node->next      = s_hash.buckets[idx];
    s_hash.buckets[idx] = new_node;
    s_hash.count++;
    return &new_node->ctrl;
}

// =============================================================
// Barra de progreso ASCII
// =============================================================
#define PROGRESS_BAR_WIDTH 30

static void _print_progress(int control_id, int done, int total)
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

// ── Init de la partición NVS de almacenamiento ────────────────
static void _nvs_storage_init(void)
{
    esp_err_t err = nvs_flash_init_partition(NVS_PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "storage partition: erasing and reinit");
        nvs_flash_erase_partition(NVS_PARTITION);
        err = nvs_flash_init_partition(NVS_PARTITION);
    }
    if (err != ESP_OK)
        ESP_LOGE(TAG, "storage partition init failed: %s", esp_err_to_name(err));
}

// =============================================================
// API pública — ciclo de vida
// =============================================================
void kx_param_store_init(void)
{
    if (s_initialized) return;
    memset(&s_hash, 0, sizeof(s_hash));
    _nvs_storage_init();
    s_initialized = true;
    ESP_LOGI(TAG,
             "hash store initialized "
             "(ctrl_buckets=%d param_buckets=%d reg_buckets=%d)",
             KX_CTRL_HASH_BUCKETS,
             KX_PARAM_HASH_BUCKETS,
             KX_REG_HASH_BUCKETS);
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

    kx_control_t *ctrl = _ctrl_find_or_create(control_id);
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
    _print_progress(control_id, 0, total);

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

        // _param_insert crea/actualiza el nivel 2 Y registra el nivel 3
        if (_param_insert(ctrl, &p) == ESP_OK) {
            inserted++;
            _print_progress(control_id, inserted, total);
            if (s_progress_cb) s_progress_cb(control_id, inserted, total);
        }
    }

    cJSON_Delete(root);
    ctrl->entities_ready = true;

    ESP_LOGI(TAG,
             "control %d: stored %d/%d params | "
             "reg_entries=%d | heap=%lu",
             control_id, inserted, total,
             ctrl->regs.count,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    return ESP_OK;
}

// =============================================================
// ── API pública — consultas nivel 1 y 2 ──────────────────────
// =============================================================
const kx_control_t *kx_param_store_get_ctrl(int control_id)
{
    return _ctrl_find(control_id);
}

const kx_param_t *kx_param_store_get_param(int control_id, int param_id)
{
    const kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return NULL;
    return _param_find(ctrl, param_id);
}

kx_param_t *kx_param_store_get_param_mutable(int control_id, int param_id)
{
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return NULL;
    return _param_find_mutable(ctrl, param_id);
}

int kx_param_store_count(void)
{
    return s_hash.count;
}

// ── Iteración nivel 2 ─────────────────────────────────────────
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
// ── API pública — Nivel 3 (caché de registros Modbus) ─────────
// =============================================================

// ── Actualizar tras una LECTURA exitosa del bus ───────────────
kx_reg_entry_t *kx_param_store_reg_upsert_read(int      control_id,
                                                uint16_t reg,
                                                uint8_t  fc_read,
                                                uint8_t  fc_write,
                                                float    value,
                                                int64_t  ts_ms)
{
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) {
        ESP_LOGW(TAG, "reg_upsert_read: ctrl=%d not found", control_id);
        return NULL;
    }

    // Buscar la entrada (fue creada al parsear, debería existir)
    kx_reg_entry_t *entry = _reg_find_mutable(&ctrl->regs, reg, fc_read, fc_write);

    if (!entry) {
        // Caso inesperado: crear la entrada sobre la marcha
        ESP_LOGW(TAG,
                 "reg_upsert_read: entry not found for ctrl=%d "
                 "reg=0x%04x fc_r=%u fc_w=%u — creating",
                 control_id, reg, fc_read, fc_write);
        entry = _reg_ensure(&ctrl->regs, reg, fc_read, fc_write);
        if (!entry) return NULL;
    }

    entry->value        = value;
    entry->ts_last_read = ts_ms;

    ESP_LOGD(TAG,
             "reg_read: ctrl=%d reg=0x%04x fc_r=%u → value=%.3f ts=%" PRId64,
             control_id, reg, fc_read, value, ts_ms);
    return entry;
}

// ── Actualizar tras una ESCRITURA exitosa en el bus ───────────
esp_err_t kx_param_store_reg_upsert_write(int      control_id,
                                           uint16_t reg,
                                           uint8_t  fc_read,
                                           uint8_t  fc_write,
                                           float    value,
                                           int64_t  ts_ms)
{
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return ESP_ERR_NOT_FOUND;

    kx_reg_entry_t *entry = _reg_find_mutable(&ctrl->regs, reg, fc_read, fc_write);
    if (!entry) {
        ESP_LOGW(TAG,
                 "reg_upsert_write: no entry for ctrl=%d "
                 "reg=0x%04x fc_r=%u fc_w=%u",
                 control_id, reg, fc_read, fc_write);
        return ESP_ERR_NOT_FOUND;
    }

    entry->last_write_value = value;
    entry->ts_last_write    = ts_ms;

    ESP_LOGD(TAG,
             "reg_write: ctrl=%d reg=0x%04x fc_w=%u → value=%.3f ts=%" PRId64,
             control_id, reg, fc_write, value, ts_ms);
    return ESP_OK;
}

// ── Búsqueda de solo lectura ──────────────────────────────────
const kx_reg_entry_t *kx_param_store_reg_get(int      control_id,
                                               uint16_t reg,
                                               uint8_t  fc_read,
                                               uint8_t  fc_write)
{
    const kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return NULL;
    return _reg_find(&ctrl->regs, reg, fc_read, fc_write);
}

// ── Búsqueda mutable ─────────────────────────────────────────
kx_reg_entry_t *kx_param_store_reg_get_mutable(int      control_id,
                                                uint16_t reg,
                                                uint8_t  fc_read,
                                                uint8_t  fc_write)
{
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return NULL;
    return _reg_find_mutable(&ctrl->regs, reg, fc_read, fc_write);
}

// ── Número de entradas en el nivel 3 de un control ───────────
int kx_param_store_reg_count(int control_id)
{
    const kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return 0;
    return ctrl->regs.count;
}

// ── Iteración sobre nivel 3 de un control ────────────────────
void kx_param_store_reg_foreach(int control_id,
                                 kx_reg_iter_cb_t cb,
                                 void *user_data)
{
    if (!cb) return;
    const kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return;

    for (int i = 0; i < KX_REG_HASH_BUCKETS; i++) {
        kx_reg_node_t *node = ctrl->regs.buckets[i];
        while (node) {
            cb(control_id, &node->entry, user_data);
            node = node->next;
        }
    }
}

// ── Iteración sobre nivel 3 de TODOS los controles ───────────
void kx_param_store_reg_foreach_all(kx_reg_iter_cb_t cb, void *user_data)
{
    if (!cb) return;
    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            kx_control_t *ctrl = &cn->ctrl;
            for (int ri = 0; ri < KX_REG_HASH_BUCKETS; ri++) {
                kx_reg_node_t *rn = ctrl->regs.buckets[ri];
                while (rn) {
                    cb(ctrl->control_id, &rn->entry, user_data);
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

    int ready_count = 0;
    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            if (cn->ctrl.entities_ready && cn->ctrl.params.count > 0)
                ready_count++;
            cn = cn->next;
        }
    }
    return (ready_count >= s_expected);
}

// =============================================================
// Configuración adicional por control
// =============================================================
void kx_param_store_set_slave_addr(int control_id, int slave_addr)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) ctrl = _ctrl_find_or_create_no_reset(control_id); 
    if (ctrl) {
        ctrl->slave_addr = slave_addr;
        ESP_LOGI(TAG, "slave_addr set: ctrl=%d → %d", control_id, slave_addr);
    }
}

void kx_param_store_set_uuid(int control_id, const char *uuid)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) ctrl = _ctrl_find_or_create_no_reset(control_id); 
    if (ctrl && uuid)
        snprintf(ctrl->uuid, sizeof(ctrl->uuid), "%s", uuid);
}

// =============================================================
// update_ts por control
// =============================================================
double kx_param_store_get_update_ts(int control_id)
{
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return 0.0;
    return ctrl->update_ts;
}

void kx_param_store_set_update_ts(int control_id, double ts)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) ctrl = _ctrl_find_or_create_no_reset(control_id);
    if (ctrl) {
        ctrl->update_ts = ts;
        ESP_LOGI(TAG, "update_ts set: ctrl=%d → %.3f", control_id, ts);
    }
}

// =============================================================
// kx_param_store_clear_entities
//
// Borra nivel 2 (params) Y nivel 3 (registros) del control
// indicado, y marca entities_ready=false.
// =============================================================
void kx_param_store_clear_entities(int control_id)
{
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) return;

    // Nivel 2: limpiar params
    for (int b = 0; b < KX_PARAM_HASH_BUCKETS; b++) {
        kx_param_node_t *pn = ctrl->params.buckets[b];
        while (pn) {
            kx_param_node_t *tmp = pn->next;
            free(pn);
            pn = tmp;
        }
        ctrl->params.buckets[b] = NULL;
    }
    ctrl->params.count   = 0;
    ctrl->entities_ready = false;

    // Nivel 3: limpiar registro cache
    int reg_count_before = ctrl->regs.count;
    _reg_clear_all(&ctrl->regs);

    ESP_LOGI(TAG,
             "entities cleared for ctrl=%d (ts=%.3f) "
             "— params=0 reg_entries=0 (was %d)",
             control_id, ctrl->update_ts, reg_count_before);
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
    uint32_t cidx = _ctrl_hash(control_id);
    kx_ctrl_node_t *cn = s_hash.buckets[cidx];
    while (cn) {
        if (cn->ctrl.control_id == control_id) {
            uint32_t pidx = _param_hash(param_id);
            kx_param_node_t *pn = cn->ctrl.params.buckets[pidx];
            while (pn) {
                if (pn->param.param_id == param_id) {
                    pn->param.ts_set = ts;
                    return ESP_OK;
                }
                pn = pn->next;
            }
            return ESP_ERR_NOT_FOUND;
        }
        cn = cn->next;
    }
    return ESP_ERR_NOT_FOUND;
}

// =============================================================
// Persistencia NVS
//
// El nivel 3 (kx_reg_hash_t) NO se persiste: se reconstruye
// automáticamente a partir de los params al cargar desde NVS,
// ya que _param_insert llama a _reg_ensure por cada param.
// =============================================================
bool kx_param_store_nvs_valid(void)
{
    nvs_handle_t h;
    if (nvs_open_from_partition(NVS_PARTITION, NVS_NS_STORE,
                                NVS_READONLY, &h) != ESP_OK)
        return false;

    uint32_t magic = 0;
    nvs_get_u32(h, NVS_KEY_MAGIC, &magic);

    char uuid[64] = "";
    size_t len = sizeof(uuid);
    nvs_get_str(h, NVS_KEY_UUID, uuid, &len);
    nvs_close(h);

    bool ok = (magic == NVS_MAGIC_VALUE)
           && (strcmp(uuid, KX_DEVICE_UUID) == 0);

    ESP_LOGI(TAG, "nvs_valid=%d (magic=%08lx uuid=%s)",
             ok, (unsigned long)magic, uuid);
    return ok;
}

esp_err_t kx_param_store_save_nvs(void)
{
    if (!s_initialized || s_hash.count == 0)
        return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open_from_partition(NVS_PARTITION, NVS_NS_STORE,
                                            NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open rw failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_u32(h, NVS_KEY_MAGIC, NVS_MAGIC_VALUE);
    nvs_set_str(h, NVS_KEY_UUID,  KX_DEVICE_UUID);
    nvs_set_u8 (h, NVS_KEY_COUNT, (uint8_t)s_hash.count);

    int ctrl_idx = 0;

    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            kx_control_t *ctrl = &cn->ctrl;
            int total_params   = ctrl->params.count;

            // ── Cabecera del control ──────────────────────────
            kx_nvs_ctrl_hdr_t hdr = {
                .control_id = ctrl->control_id,
                .slave_addr = ctrl->slave_addr,
                .count      = total_params,
                .update_ts  = ctrl->update_ts,
            };
            snprintf(hdr.uuid, sizeof(hdr.uuid), "%s", ctrl->uuid);

            char hdr_key[16];
            snprintf(hdr_key, sizeof(hdr_key), "hdr_%d", ctrl_idx);
            err = nvs_set_blob(h, hdr_key, &hdr, sizeof(hdr));
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "nvs: hdr write %s failed: %s",
                         hdr_key, esp_err_to_name(err));
                cn = cn->next; ctrl_idx++;
                continue;
            }

            // ── Serializar params en array temporal ───────────
            kx_param_t *flat = malloc(total_params * sizeof(kx_param_t));
            if (!flat) {
                ESP_LOGE(TAG, "OOM flat params ctrl=%d", ctrl->control_id);
                cn = cn->next; ctrl_idx++;
                continue;
            }

            int idx = 0;
            for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS && idx < total_params; pi++) {
                kx_param_node_t *pn = ctrl->params.buckets[pi];
                while (pn && idx < total_params) {
                    memcpy(&flat[idx++], &pn->param, sizeof(kx_param_t));
                    pn = pn->next;
                }
            }

            // ── Guardar en chunks ─────────────────────────────
            int chunks = (total_params + NVS_CHUNK_PARAMS - 1) / NVS_CHUNK_PARAMS;
            if (chunks == 0) chunks = 1;

            for (int j = 0; j < chunks; j++) {
                int offset     = j * NVS_CHUNK_PARAMS;
                int count      = total_params - offset;
                if (count > (int)NVS_CHUNK_PARAMS) count = NVS_CHUNK_PARAMS;
                size_t chunk_bytes = (size_t)count * sizeof(kx_param_t);

                char chunk_key[16];
                snprintf(chunk_key, sizeof(chunk_key), "p%d_%d", ctrl_idx, j);

                esp_err_t cerr = nvs_set_blob(h, chunk_key,
                                              flat + offset, chunk_bytes);
                if (cerr != ESP_OK)
                    ESP_LOGW(TAG, "nvs: chunk %s failed: %s",
                             chunk_key, esp_err_to_name(cerr));
                else
                    ESP_LOGD(TAG, "nvs: chunk %s → %d params (%zu bytes)",
                             chunk_key, count, chunk_bytes);
            }
            free(flat);

            char nchunks_key[16];
            snprintf(nchunks_key, sizeof(nchunks_key), "nc_%d", ctrl_idx);
            nvs_set_u8(h, nchunks_key, (uint8_t)chunks);

            ESP_LOGI(TAG,
                     "nvs: saved ctrl=%d uuid=%s ts=%.3f "
                     "params=%d reg_entries=%d chunks=%d",
                     ctrl->control_id, ctrl->uuid,
                     ctrl->update_ts, total_params,
                     ctrl->regs.count, chunks);

            cn = cn->next;
            ctrl_idx++;
        }
    }

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK)
        ESP_LOGI(TAG, "nvs: commit OK — %d controls saved", s_hash.count);
    return err;
}

esp_err_t kx_param_store_load_nvs(void)
{
    if (!s_initialized) kx_param_store_init();

    nvs_handle_t h;
    esp_err_t err = nvs_open_from_partition(NVS_PARTITION, NVS_NS_STORE,
                                            NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t count = 0;
    nvs_get_u8(h, NVS_KEY_COUNT, &count);
    ESP_LOGI(TAG, "nvs: loading %d controls...", count);

    for (int i = 0; i < count; i++) {

        // ── Leer cabecera ─────────────────────────────────────
        char hdr_key[16];
        snprintf(hdr_key, sizeof(hdr_key), "hdr_%d", i);

        size_t hdr_sz = sizeof(kx_nvs_ctrl_hdr_t);
        kx_nvs_ctrl_hdr_t hdr = {0};
        err = nvs_get_blob(h, hdr_key, &hdr, &hdr_sz);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "nvs: hdr %s missing: %s",
                     hdr_key, esp_err_to_name(err));
            continue;
        }

        kx_control_t *ctrl = _ctrl_find_or_create(hdr.control_id);
        if (!ctrl) continue;
        ctrl->slave_addr = hdr.slave_addr;
        ctrl->update_ts  = hdr.update_ts;
        snprintf(ctrl->uuid, sizeof(ctrl->uuid), "%s", hdr.uuid);

        // ── Número de chunks ──────────────────────────────────
        char nchunks_key[16];
        snprintf(nchunks_key, sizeof(nchunks_key), "nc_%d", i);
        uint8_t n_chunks = 1;
        nvs_get_u8(h, nchunks_key, &n_chunks);

        int total_loaded = 0;

        for (int j = 0; j < n_chunks; j++) {
            char chunk_key[16];
            snprintf(chunk_key, sizeof(chunk_key), "p%d_%d", i, j);

            size_t chunk_bytes = 0;
            err = nvs_get_blob(h, chunk_key, NULL, &chunk_bytes);
            if (err != ESP_OK || chunk_bytes == 0) {
                ESP_LOGW(TAG, "nvs: chunk %s missing", chunk_key);
                continue;
            }

            uint8_t *buf = malloc(chunk_bytes);
            if (!buf) {
                ESP_LOGE(TAG, "OOM loading chunk %s", chunk_key);
                continue;
            }

            err = nvs_get_blob(h, chunk_key, buf, &chunk_bytes);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "nvs: read %s failed: %s",
                         chunk_key, esp_err_to_name(err));
                free(buf);
                continue;
            }

            int params_in_chunk = (int)(chunk_bytes / sizeof(kx_param_t));
            kx_param_t *params  = (kx_param_t *)buf;

            for (int p = 0; p < params_in_chunk; p++) {
                // _param_insert reconstruye automáticamente el nivel 3
                if (_param_insert(ctrl, &params[p]) == ESP_OK)
                    total_loaded++;
            }
            free(buf);
        }

        ctrl->entities_ready = (total_loaded > 0);

        ESP_LOGI(TAG,
                 "nvs: ctrl=%d uuid=%s slave=%d ts=%.3f "
                 "params=%d reg_entries=%d chunks=%d",
                 hdr.control_id, hdr.uuid, hdr.slave_addr,
                 hdr.update_ts, total_loaded,
                 ctrl->regs.count, n_chunks);
    }

    nvs_close(h);
    ESP_LOGI(TAG, "nvs: loaded %d controls OK", s_hash.count);
    return (s_hash.count > 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t kx_param_store_clear_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open_from_partition(NVS_PARTITION, NVS_NS_STORE,
                                            NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "nvs: cache cleared");
    return err;
}