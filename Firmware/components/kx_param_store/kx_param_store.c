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

// Incluir kx_config.h para KX_DEVICE_UUID y KX_FW_VERSION
#include "../../main/kx_config.h"

static const char *TAG = "kx_param_store";

// ── NVS namespace / claves ────────────────────────────────────
#define NVS_NS_STORE     "kx_entities"
#define NVS_KEY_MAGIC    "magic"
#define NVS_KEY_UUID     "uuid"
#define NVS_KEY_FW       "fw"
#define NVS_KEY_COUNT    "count"
// datos del control N → clave "ctrl_N" (máx 15 chars key NVS)
#define NVS_MAGIC_VALUE  0xE5712A01U

// ── Almacén dinámico en PSRAM ─────────────────────────────────
static kx_control_params_t *s_store    = NULL;
static int                   s_count   = 0;
static int                   s_expected = 0;

// ── Callback de progreso ──────────────────────────────────────
static kx_param_progress_cb_t s_progress_cb = NULL;

void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb)
{
    s_progress_cb = cb;
}

// ── Barra de progreso ASCII ───────────────────────────────────
// Imprime en una sola línea (sin salto) para sobreescribir en terminal serie.
// En IDF el LOG siempre añade \n, así que usamos printf directo + fflush.
#define PROGRESS_BAR_WIDTH  30

static void _print_progress(int control_id, int done, int total)
{
    if (total <= 0) return;

    int pct   = (done * 100) / total;
    int filled = (done * PROGRESS_BAR_WIDTH) / total;

    char bar[PROGRESS_BAR_WIDTH + 1];
    for (int i = 0; i < PROGRESS_BAR_WIDTH; i++) {
        bar[i] = (i < filled) ? '#' : '-';
    }
    bar[PROGRESS_BAR_WIDTH] = '\0';

    // \r sin \n para sobreescribir en terminales serie (minicom, idf monitor)
    printf("\r[entities] ctrl=%d [%s] %3d%% (%d/%d params)",
           control_id, bar, pct, done, total);
    fflush(stdout);

    if (done >= total) {
        printf("\n");   // salto final al completar
        fflush(stdout);
    }
}

// ── Helpers de parsing ────────────────────────────────────────
static kx_control_params_t *_find_or_create(int control_id)
{
    if (!s_store) {
        ESP_LOGE(TAG, "_find_or_create: store not initialized");
        return NULL;
    }

    for (int i = 0; i < s_count; i++) {
        if (s_store[i].control_id == control_id) {
            ESP_LOGW(TAG, "control %d already in store, overwriting", control_id);
            s_store[i].count = 0;
            return &s_store[i];
        }
    }

    if (s_count >= KX_PARAM_MAX_CONTROLS) {
        ESP_LOGE(TAG, "store full (%d controls)", KX_PARAM_MAX_CONTROLS);
        return NULL;
    }

    s_store[s_count].control_id = control_id;
    s_store[s_count].count      = 0;
    s_store[s_count].entities_ready = false;
    return &s_store[s_count++];
}

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

void kx_param_store_set_slave_addr(int control_id, int slave_addr)
{
    if (!s_store) return;
    for (int i = 0; i < s_count; i++) {
        if (s_store[i].control_id == control_id) {
            s_store[i].slave_addr = slave_addr;
            ESP_LOGI(TAG, "slave_addr set: ctrl=%d → %d", control_id, slave_addr);
            return;
        }
    }
    // si todavía no existe el control, crearlo
    kx_control_params_t *ctrl = _find_or_create(control_id);
    if (ctrl) ctrl->slave_addr = slave_addr;
}

// ── API pública — ciclo de vida ───────────────────────────────
void kx_param_store_init(void)
{
    if (s_store) return;

    s_store = heap_caps_malloc(
        sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS,
        MALLOC_CAP_SPIRAM
    );

    if (!s_store) {
        // fallback a RAM interna si PSRAM no disponible
        s_store = malloc(sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS);
    }

    if (!s_store) {
        ESP_LOGE(TAG, "FATAL: cannot allocate param store");
        return;
    }

    memset(s_store, 0, sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS);
    s_count = 0;

    ESP_LOGI(TAG, "initialized (%d bytes)",
             (int)(sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS));
}

// ── Persistencia NVS ─────────────────────────────────────────

bool kx_param_store_nvs_valid(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_STORE, NVS_READONLY, &h) != ESP_OK) return false;

    uint32_t magic = 0;
    nvs_get_u32(h, NVS_KEY_MAGIC, &magic);

    char uuid[64] = "";
    char fw[32]   = "";
    size_t len;

    len = sizeof(uuid);
    nvs_get_str(h, NVS_KEY_UUID, uuid, &len);
    len = sizeof(fw);
    nvs_get_str(h, NVS_KEY_FW,   fw,   &len);

    nvs_close(h);

    bool ok = (magic == NVS_MAGIC_VALUE)
           && (strcmp(uuid, KX_DEVICE_UUID) == 0)
           && (strcmp(fw,   KX_FW_VERSION)  == 0);

    ESP_LOGI(TAG, "nvs_valid=%d (magic=%08lx uuid=%s fw=%s)",
             ok, (unsigned long)magic, uuid, fw);
    return ok;
}

esp_err_t kx_param_store_load_nvs(void)
{
    if (!s_store) kx_param_store_init();
    if (!s_store) return ESP_FAIL;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_STORE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t count = 0;
    nvs_get_u8(h, NVS_KEY_COUNT, &count);
    ESP_LOGI(TAG, "nvs: loading %d controls...", count);

    s_count = 0;

    for (int i = 0; i < count && i < KX_PARAM_MAX_CONTROLS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ctrl_%d", i);

        size_t blob_size = 0;
        err = nvs_get_blob(h, key, NULL, &blob_size);
        if (err != ESP_OK || blob_size == 0) {
            ESP_LOGW(TAG, "nvs: key %s missing or empty", key);
            continue;
        }

        if (blob_size != sizeof(kx_control_params_t)) {
            ESP_LOGW(TAG, "nvs: key %s size mismatch (%d vs %d)",
                     key, (int)blob_size, (int)sizeof(kx_control_params_t));
            continue;
        }

        err = nvs_get_blob(h, key, &s_store[s_count], &blob_size);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "nvs: ctrl[%d] id=%d params=%d",
                     s_count,
                     s_store[s_count].control_id,
                     s_store[s_count].count);
            s_count++;
        } else {
            ESP_LOGW(TAG, "nvs: read %s failed: %s", key, esp_err_to_name(err));
        }
    }

    nvs_close(h);
    ESP_LOGI(TAG, "nvs: loaded %d controls OK", s_count);
    return (s_count > 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t kx_param_store_save_nvs(void)
{
    if (!s_store || s_count == 0) return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_STORE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open rw failed: %s", esp_err_to_name(err));
        return err;
    }

    // guardar metadatos
    nvs_set_u32(h, NVS_KEY_MAGIC, NVS_MAGIC_VALUE);
    nvs_set_str(h, NVS_KEY_UUID,  KX_DEVICE_UUID);
    nvs_set_str(h, NVS_KEY_FW,    KX_FW_VERSION);
    nvs_set_u8 (h, NVS_KEY_COUNT, (uint8_t)s_count);

    // guardar cada control como blob
    for (int i = 0; i < s_count; i++) {
        char key[16];
        snprintf(key, sizeof(key), "ctrl_%d", i);
        err = nvs_set_blob(h, key, &s_store[i], sizeof(kx_control_params_t));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "nvs: write %s failed: %s", key, esp_err_to_name(err));
        }
    }

    err = nvs_commit(h);
    nvs_close(h);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "nvs: saved %d controls (uuid=%s fw=%s)",
                 s_count, KX_DEVICE_UUID, KX_FW_VERSION);
    }
    return err;
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

// ── Parseo desde JSON (con barra de progreso) ─────────────────
esp_err_t kx_param_store_parse(const char *payload, size_t len,
                                int control_id)
{
    if (!s_store) kx_param_store_init();
    if (!s_store) return ESP_FAIL;

    cJSON *root = cJSON_ParseWithLength(payload, len);
    if (!root) {
        ESP_LOGE(TAG, "JSON parse failed for control %d", control_id);
        return ESP_FAIL;
    }

    kx_control_params_t *ctrl = _find_or_create(control_id);
    if (!ctrl) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    ctrl->count = 0;

    cJSON *regs = cJSON_GetObjectItem(root, "control_regs");
    if (!regs || !cJSON_IsArray(regs)) {
        ESP_LOGW(TAG, "control %d: no control_regs array", control_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    int total = cJSON_GetArraySize(regs);
    ESP_LOGI(TAG, "control %d: downloading %d params...", control_id, total);

    // mostrar barra al 0%
    _print_progress(control_id, 0, total);

    for (int i = 0; i < total && ctrl->count < KX_PARAM_MAX_PER_CONTROL; i++) {
        cJSON *reg = cJSON_GetArrayItem(regs, i);
        if (!reg) continue;

        kx_param_t *p = &ctrl->params[ctrl->count];

        p->param_id       = _get_int  (reg, "control_parameter_id",                0);
        p->reg            = _get_int  (reg, "control_parameter_register",           0);
        p->function_read  = _get_int  (reg, "control_parameter_function_read",      0);
        p->function_write = _get_int  (reg, "control_parameter_function_write",     0);
        p->minvalue       = _get_float(reg, "control_parameter_minvalue",        0.0f);
        p->maxvalue       = _get_float(reg, "control_parameter_maxvalue",      100.0f);
        p->offset         = _get_float(reg, "control_parameter_offset",          0.0f);
        p->mask           = _get_int  (reg, "control_parameter_mask",               0);
        p->view           = _get_int  (reg, "control_parameter_view",               1);
        p->sampling       = _get_int  (reg, "control_parameter_sampling",          60);

        _get_str(reg, "control_parameter_name",
                 p->name,     sizeof(p->name));
        _get_str(reg, "control_parameter_category_system",
                 p->category, sizeof(p->category));
        _get_str(reg, "control_parameter_length",
                 p->length,   sizeof(p->length));

        cJSON *add = cJSON_GetObjectItem(reg, "control_parameter_addition");
        p->addition = (add && cJSON_IsNumber(add))
                      ? (float)add->valuedouble : 0.0f;

        if (p->param_id > 0) {
            ctrl->count++;

            // actualizar barra de progreso
            _print_progress(control_id, ctrl->count, total);

            // notificar al callback externo si está registrado
            if (s_progress_cb) {
                s_progress_cb(control_id, ctrl->count, total);
            }
        }
    }

    cJSON_Delete(root);

    ESP_LOGI(TAG, "control %d: stored %d/%d params | heap=%lu",
             control_id, ctrl->count, total,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    
    ctrl->entities_ready = true;    
    return ESP_OK;
}

// ── Consultas ─────────────────────────────────────────────────
const kx_control_params_t *kx_param_store_get(int control_id)
{
    if (!s_store) return NULL;
    for (int i = 0; i < s_count; i++) {
        if (s_store[i].control_id == control_id) return &s_store[i];
    }
    return NULL;
}

int kx_param_store_count(void)
{
    return s_count;
}

void kx_param_store_set_expected(int count)
{
    s_expected = count;
    ESP_LOGI(TAG, "expecting %d controls", count);
}

bool kx_param_store_is_ready(void)
{
    if (s_expected <= 0 || s_count < s_expected) return false;
    for (int i = 0; i < s_count; i++) {
        if (!s_store[i].entities_ready) return false;
    }
    return true;
}

void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data)
{
    if (!s_store || !cb) return;
    for (int i = 0; i < s_count; i++) {
        for (int j = 0; j < s_store[i].count; j++) {
            cb(s_store[i].control_id, &s_store[i].params[j], user_data);
        }
    }
}