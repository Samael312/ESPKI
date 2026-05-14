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
#define NVS_MAGIC_VALUE  0xE5712A02U   // bumped vs versión array

// Formato blob NVS para un control:
//   [ kx_nvs_ctrl_hdr_t ][ kx_param_t × count ]
typedef struct {
    int  control_id;
    int  slave_addr;
    int  count;
} kx_nvs_ctrl_hdr_t;

// =============================================================
// Estado global
// =============================================================
static kx_ctrl_hash_t        s_hash;           // tabla nivel 1
static bool                  s_initialized = false;
static int                   s_expected    = 0;
static kx_param_progress_cb_t s_progress_cb = NULL;

// =============================================================
// Asignador PSRAM con fallback
// =============================================================
static void *_psram_alloc(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(size);
    return p;
}

// =============================================================
// Funciones hash
// =============================================================

// Hash para control_id (nivel 1)
static inline uint32_t _ctrl_hash(int control_id)
{
    // Multiplicative hash — distribuye bien IDs consecutivos
    uint32_t k = (uint32_t)control_id;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = (k >> 16) ^ k;
    return k & (KX_CTRL_HASH_BUCKETS - 1);
}

// Hash para param_id (nivel 2)
static inline uint32_t _param_hash(int param_id)
{
    uint32_t k = (uint32_t)param_id;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = ((k >> 16) ^ k) * 0x45d9f3bU;
    k = (k >> 16) ^ k;
    return k & (KX_PARAM_HASH_BUCKETS - 1);
}

// =============================================================
// Operaciones sobre la hash de controles (nivel 1)
// =============================================================

// Busca o crea un nodo de control.
// Devuelve puntero al kx_control_t interno, NULL si hay error.
static kx_control_t *_ctrl_find_or_create(int control_id)
{
    uint32_t idx = _ctrl_hash(control_id);
    kx_ctrl_node_t *node = s_hash.buckets[idx];

    // buscar en la cadena
    while (node) {
        if (node->ctrl.control_id == control_id) {
            ESP_LOGW(TAG, "control %d already exists — overwriting params",
                     control_id);
            // vaciar la hash de params existente
            for (int b = 0; b < KX_PARAM_HASH_BUCKETS; b++) {
                kx_param_node_t *pn = node->ctrl.params.buckets[b];
                while (pn) {
                    kx_param_node_t *tmp = pn->next;
                    free(pn);
                    pn = tmp;
                }
                node->ctrl.params.buckets[b] = NULL;
            }
            node->ctrl.params.count      = 0;
            node->ctrl.entities_ready    = false;
            return &node->ctrl;
        }
        node = node->next;
    }

    // límite de controles
    if (s_hash.count >= KX_PARAM_MAX_CONTROLS) {
        ESP_LOGE(TAG, "hash full: %d controls (max %d)",
                 s_hash.count, KX_PARAM_MAX_CONTROLS);
        return NULL;
    }

    // crear nodo nuevo en PSRAM
    kx_ctrl_node_t *new_node = _psram_alloc(sizeof(kx_ctrl_node_t));
    if (!new_node) {
        ESP_LOGE(TAG, "OOM creating control node");
        return NULL;
    }
    memset(new_node, 0, sizeof(kx_ctrl_node_t));
    new_node->ctrl.control_id = control_id;

    // insertar al frente del bucket
    new_node->next      = s_hash.buckets[idx];
    s_hash.buckets[idx] = new_node;
    s_hash.count++;

    ESP_LOGI(TAG, "ctrl hash: inserted control_id=%d bucket=%u count=%d",
             control_id, idx, s_hash.count);
    return &new_node->ctrl;
}

// Busca un control (lectura, no crea).
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
// Operaciones sobre la hash de params (nivel 2)
// =============================================================

// Inserta o sobreescribe un param en la hash del control.
static esp_err_t _param_insert(kx_control_t *ctrl, const kx_param_t *param)
{
    uint32_t idx = _param_hash(param->param_id);
    kx_param_node_t *node = ctrl->params.buckets[idx];

    // buscar si ya existe (sobreescribir)
    while (node) {
        if (node->param.param_id == param->param_id) {
            memcpy(&node->param, param, sizeof(kx_param_t));
            return ESP_OK;
        }
        node = node->next;
    }

    // límite
    if (ctrl->params.count >= KX_PARAM_MAX_PER_CONTROL) {
        ESP_LOGW(TAG, "ctrl %d: param limit reached (%d)",
                 ctrl->control_id, KX_PARAM_MAX_PER_CONTROL);
        return ESP_ERR_NO_MEM;
    }

    // nodo nuevo
    kx_param_node_t *new_node = _psram_alloc(sizeof(kx_param_node_t));
    if (!new_node) {
        ESP_LOGE(TAG, "OOM creating param node");
        return ESP_ERR_NO_MEM;
    }
    memcpy(&new_node->param, param, sizeof(kx_param_t));
    new_node->next               = ctrl->params.buckets[idx];
    ctrl->params.buckets[idx]    = new_node;
    ctrl->params.count++;
    return ESP_OK;
}

// Busca un param por param_id dentro de un control.
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
    if (item && cJSON_IsString(item) && item->valuestring) {
        snprintf(buf, len, "%s", item->valuestring);
    } else {
        buf[0] = '\0';
    }
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
    for (int i = 0; i < PROGRESS_BAR_WIDTH; i++) {
        bar[i] = (i < filled) ? '#' : '-';
    }
    bar[PROGRESS_BAR_WIDTH] = '\0';

    printf("\r[entities] ctrl=%d [%s] %3d%% (%d/%d params)",
           control_id, bar, pct, done, total);
    fflush(stdout);

    if (done >= total) {
        printf("\n");
        fflush(stdout);
    }
}

// =============================================================
// API pública — ciclo de vida
// =============================================================
void kx_param_store_init(void)
{
    if (s_initialized) return;
    memset(&s_hash, 0, sizeof(s_hash));
    s_initialized = true;
    ESP_LOGI(TAG, "hash store initialized (ctrl_buckets=%d param_buckets=%d)",
             KX_CTRL_HASH_BUCKETS, KX_PARAM_HASH_BUCKETS);
}

// =============================================================
// Parseo de entities desde JSON
// =============================================================
esp_err_t kx_param_store_parse(const char *payload, size_t len,
                                int control_id)
{
    if (!s_initialized) kx_param_store_init();

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse failed for control %d", control_id);
        return ESP_FAIL;
    }

    kx_control_t *ctrl = _ctrl_find_or_create(control_id);
    if (!ctrl) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    cJSON *regs = cJSON_GetObjectItem(root, "control_regs");
    if (!regs || !cJSON_IsArray(regs)) {
        ESP_LOGW(TAG, "control %d: no control_regs array", control_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    int total   = cJSON_GetArraySize(regs);
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

        if (p.param_id <= 0) continue;

        if (_param_insert(ctrl, &p) == ESP_OK) {
            inserted++;
            _print_progress(control_id, inserted, total);
            if (s_progress_cb) s_progress_cb(control_id, inserted, total);
        }
    }

    cJSON_Delete(root);

    ctrl->entities_ready = true;

    ESP_LOGI(TAG, "control %d: stored %d/%d params | heap=%lu",
             control_id, inserted, total,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    return ESP_OK;
}

// Reutilizable: rellena un kx_param_t desde un objeto cJSON
static void _parse_param_from_json(cJSON *reg, kx_param_t *p)
{
    memset(p, 0, sizeof(*p));
    p->param_id       = _get_int  (reg, "control_parameter_id",           0);
    p->reg            = _get_int  (reg, "control_parameter_register",      0);
    p->function_read  = _get_int  (reg, "control_parameter_function_read", 0);
    p->function_write = _get_int  (reg, "control_parameter_function_write",0);
    p->minvalue       = _get_float(reg, "control_parameter_minvalue",   0.0f);
    p->maxvalue       = _get_float(reg, "control_parameter_maxvalue", 100.0f);
    p->offset         = _get_float(reg, "control_parameter_offset",     0.0f);
    p->mask           = _get_int  (reg, "control_parameter_mask",          0);
    p->view           = _get_int  (reg, "control_parameter_view",          1);
    p->sampling       = _get_int  (reg, "control_parameter_sampling",     60);
    _get_str(reg, "control_parameter_name",            p->name,     sizeof(p->name));
    _get_str(reg, "control_parameter_category_system", p->category, sizeof(p->category));
    _get_str(reg, "control_parameter_length",          p->length,   sizeof(p->length));
    cJSON *add = cJSON_GetObjectItem(reg, "control_parameter_addition");
    p->addition = (add && cJSON_IsNumber(add)) ? (float)add->valuedouble : 0.0f;
}

// Compara dos params y devuelve bitmask de diferencias
static kx_param_changed_t _param_diff(const kx_param_t *old,
                                       const kx_param_t *new)
{
    kx_param_changed_t mask = KX_PARAM_CHANGED_NONE;
    if (old->sampling       != new->sampling)       mask |= KX_PARAM_CHANGED_SAMPLING;
    if (old->function_read  != new->function_read)  mask |= KX_PARAM_CHANGED_FUNCTION_READ;
    if (old->function_write != new->function_write) mask |= KX_PARAM_CHANGED_FUNCTION_WRITE;
    if (old->offset         != new->offset)         mask |= KX_PARAM_CHANGED_OFFSET;
    if (old->addition       != new->addition)       mask |= KX_PARAM_CHANGED_ADDITION;
    if (old->view           != new->view)           mask |= KX_PARAM_CHANGED_VIEW;
    if (old->reg            != new->reg)            mask |= KX_PARAM_CHANGED_REG;
    if (old->minvalue       != new->minvalue ||
        old->maxvalue       != new->maxvalue)       mask |= KX_PARAM_CHANGED_MINMAX;
    return mask;
}

int kx_param_store_diff_and_update(const char *payload, size_t len,
                                    int control_id,
                                    kx_param_diff_cb_t cb,
                                    void *user_data)
{
    if (!s_initialized) kx_param_store_init();

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGE(TAG, "diff_and_update: JSON parse failed ctrl=%d", control_id);
        return -1;
    }

    cJSON *regs = cJSON_GetObjectItem(root, "control_regs");
    if (!regs || !cJSON_IsArray(regs)) {
        ESP_LOGW(TAG, "diff_and_update: no control_regs ctrl=%d", control_id);
        cJSON_Delete(root);
        return -1;
    }

    // El control debe existir ya (fue creado en la descarga inicial)
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) {
        // Primera vez que llega este control — comportamiento normal de inserción
        ctrl = _ctrl_find_or_create(control_id);
        if (!ctrl) { cJSON_Delete(root); return -1; }
    }

    int changed_count = 0;
    int total = cJSON_GetArraySize(regs);

    for (int i = 0; i < total; i++) {
        cJSON *reg = cJSON_GetArrayItem(regs, i);
        if (!reg) continue;

        kx_param_t incoming;
        _parse_param_from_json(reg, &incoming);
        if (incoming.param_id <= 0) continue;

        const kx_param_t *stored = _param_find(ctrl, incoming.param_id);

        kx_param_changed_t diff_mask;

        if (!stored) {
            // Param nuevo — insertar y notificar como cambio total
            diff_mask = (kx_param_changed_t)0xFF;
            ESP_LOGI(TAG, "diff ctrl=%d param=%d → NEW",
                     control_id, incoming.param_id);
        } else {
            diff_mask = _param_diff(stored, &incoming);
            if (diff_mask == KX_PARAM_CHANGED_NONE) continue; // sin cambios
            ESP_LOGI(TAG, "diff ctrl=%d param=%d → changed=0x%02x",
                     control_id, incoming.param_id, (unsigned)diff_mask);
        }

        // Actualizar hash con los valores nuevos
        _param_insert(ctrl, &incoming);
        changed_count++;

        // Notificar al caller
        if (cb) {
            kx_param_diff_t d = {
                .param_id  = incoming.param_id,
                .changed   = diff_mask,
                .new_param = incoming,
            };
            cb(control_id, &d, user_data);
        }
    }

    cJSON_Delete(root);

    if (changed_count > 0) {
        ESP_LOGI(TAG, "diff ctrl=%d: %d params updated", control_id, changed_count);
        // Persistir los cambios en NVS
        kx_param_store_save_nvs();
    }

    return changed_count;
}

// =============================================================
// Consultas
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

int kx_param_store_count(void)
{
    return s_hash.count;
}

// =============================================================
// Iteración
// =============================================================
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
// Control de completitud
// =============================================================
void kx_param_store_set_expected(int count)
{
    s_expected = count;
    ESP_LOGI(TAG, "expecting %d controls", count);
}

bool kx_param_store_is_ready(void)
{
    if (s_expected <= 0 || s_hash.count < s_expected) return false;
    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            if (!cn->ctrl.entities_ready) return false;
            cn = cn->next;
        }
    }
    return true;
}

// =============================================================
// Configuración adicional por control
// =============================================================
void kx_param_store_set_slave_addr(int control_id, int slave_addr)
{
    if (!s_initialized) kx_param_store_init();
    kx_control_t *ctrl = _ctrl_find(control_id);
    if (!ctrl) {
        ctrl = _ctrl_find_or_create(control_id);
    }
    if (ctrl) {
        ctrl->slave_addr = slave_addr;
        ESP_LOGI(TAG, "slave_addr set: ctrl=%d → %d", control_id, slave_addr);
    }
}

// =============================================================
// Progreso
// =============================================================
void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb)
{
    s_progress_cb = cb;
}

// =============================================================
// Persistencia NVS
// Formato: por cada control se guarda un blob con cabecera +
// array plano de kx_param_t — igual que antes pero adaptado
// a que ahora el store es una hash.
// =============================================================
bool kx_param_store_nvs_valid(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_STORE, NVS_READONLY, &h) != ESP_OK) return false;

    uint32_t magic = 0;
    nvs_get_u32(h, NVS_KEY_MAGIC, &magic);

    char uuid[64] = "";
    char fw  [32] = "";
    size_t len;

    len = sizeof(uuid); nvs_get_str(h, NVS_KEY_UUID, uuid, &len);
    len = sizeof(fw);   nvs_get_str(h, NVS_KEY_FW,   fw,   &len);
    nvs_close(h);

    bool ok = (magic == NVS_MAGIC_VALUE)
           && (strcmp(uuid, KX_DEVICE_UUID) == 0)
           && (strcmp(fw,   KX_FW_VERSION)  == 0);

    ESP_LOGI(TAG, "nvs_valid=%d (magic=%08lx uuid=%s fw=%s)",
             ok, (unsigned long)magic, uuid, fw);
    return ok;
}

esp_err_t kx_param_store_save_nvs(void)
{
    if (!s_initialized || s_hash.count == 0) return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_STORE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open rw failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_u32(h, NVS_KEY_MAGIC, NVS_MAGIC_VALUE);
    nvs_set_str(h, NVS_KEY_UUID,  KX_DEVICE_UUID);
    nvs_set_str(h, NVS_KEY_FW,    KX_FW_VERSION);
    nvs_set_u8 (h, NVS_KEY_COUNT, (uint8_t)s_hash.count);

    int ctrl_idx = 0;

    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            kx_control_t *ctrl = &cn->ctrl;

            // construir blob: cabecera + params planos
            size_t blob_size = sizeof(kx_nvs_ctrl_hdr_t)
                             + (size_t)ctrl->params.count * sizeof(kx_param_t);
            uint8_t *blob = malloc(blob_size);
            if (!blob) {
                ESP_LOGE(TAG, "OOM building NVS blob for ctrl=%d",
                         ctrl->control_id);
                cn = cn->next;
                continue;
            }

            kx_nvs_ctrl_hdr_t hdr = {
                .control_id = ctrl->control_id,
                .slave_addr = ctrl->slave_addr,
                .count      = ctrl->params.count,
            };
            memcpy(blob, &hdr, sizeof(hdr));

            uint8_t *ptr = blob + sizeof(hdr);
            for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS; pi++) {
                kx_param_node_t *pn = ctrl->params.buckets[pi];
                while (pn) {
                    memcpy(ptr, &pn->param, sizeof(kx_param_t));
                    ptr += sizeof(kx_param_t);
                    pn   = pn->next;
                }
            }

            char key[16];
            snprintf(key, sizeof(key), "ctrl_%d", ctrl_idx++);
            err = nvs_set_blob(h, key, blob, blob_size);
            free(blob);

            if (err != ESP_OK) {
                ESP_LOGW(TAG, "nvs write %s failed: %s",
                         key, esp_err_to_name(err));
            }

            cn = cn->next;
        }
    }

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "nvs: saved %d controls (uuid=%s fw=%s)",
                 s_hash.count, KX_DEVICE_UUID, KX_FW_VERSION);
    }
    return err;
}

esp_err_t kx_param_store_load_nvs(void)
{
    if (!s_initialized) kx_param_store_init();

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_STORE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t count = 0;
    nvs_get_u8(h, NVS_KEY_COUNT, &count);
    ESP_LOGI(TAG, "nvs: loading %d controls...", count);

    for (int i = 0; i < count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ctrl_%d", i);

        size_t blob_size = 0;
        err = nvs_get_blob(h, key, NULL, &blob_size);
        if (err != ESP_OK || blob_size < sizeof(kx_nvs_ctrl_hdr_t)) {
            ESP_LOGW(TAG, "nvs: key %s missing or too small", key);
            continue;
        }

        uint8_t *blob = malloc(blob_size);
        if (!blob) {
            ESP_LOGE(TAG, "OOM loading blob %s", key);
            continue;
        }

        err = nvs_get_blob(h, key, blob, &blob_size);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "nvs: read %s failed: %s", key, esp_err_to_name(err));
            free(blob);
            continue;
        }

        kx_nvs_ctrl_hdr_t hdr;
        memcpy(&hdr, blob, sizeof(hdr));

        kx_control_t *ctrl = _ctrl_find_or_create(hdr.control_id);
        if (!ctrl) {
            free(blob);
            continue;
        }
        ctrl->slave_addr = hdr.slave_addr;

        // insertar params desde el blob plano
        kx_param_t *params = (kx_param_t *)(blob + sizeof(hdr));
        int params_in_blob = (int)((blob_size - sizeof(hdr)) / sizeof(kx_param_t));
        int loaded = 0;

        for (int p = 0; p < params_in_blob && p < hdr.count; p++) {
            if (_param_insert(ctrl, &params[p]) == ESP_OK) loaded++;
        }

        ctrl->entities_ready = true;
        free(blob);

        ESP_LOGI(TAG, "nvs: ctrl=%d slave=%d params=%d",
                 hdr.control_id, hdr.slave_addr, loaded);
    }

    nvs_close(h);
    ESP_LOGI(TAG, "nvs: loaded %d controls OK", s_hash.count);
    return (s_hash.count > 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t kx_param_store_clear_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_STORE, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "nvs: cache cleared");
    return err;
}