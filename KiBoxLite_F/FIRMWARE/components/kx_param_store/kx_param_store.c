#include "kx_param_store.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "kx_param_store";

// ── Almacén dinámico en PSRAM ─────────────────────────────────
static kx_control_params_t *s_store = NULL;
static int                   s_count = 0;
static int s_expected = 0;

void kx_param_store_set_expected(int count)
{
    s_expected = count;
    ESP_LOGI(TAG, "expecting %d controls", count);
}

bool kx_param_store_is_ready(void)
{
    return s_expected > 0 && s_count >= s_expected;
}

// ── Helpers ───────────────────────────────────────────────────
static kx_control_params_t *_find_or_create(int control_id)
{
    if (!s_store) {
        ESP_LOGE(TAG, "_find_or_create: store not initialized");
        return NULL;
    }

    for (int i = 0; i < s_count; i++) {
        if (s_store[i].control_id == control_id) {
            ESP_LOGW(TAG, "control %d already in store, overwriting",
                     control_id);
            s_store[i].count = 0;  // reset para sobreescribir
            return &s_store[i];
        }
    }

    if (s_count >= KX_PARAM_MAX_CONTROLS) {
        ESP_LOGE(TAG, "store full (%d controls)", KX_PARAM_MAX_CONTROLS);
        return NULL;
    }

    s_store[s_count].control_id = control_id;
    s_store[s_count].count      = 0;
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

// ── API pública ───────────────────────────────────────────────
void kx_param_store_init(void)
{
    if (s_store) return;  // ya inicializado

    s_store = heap_caps_malloc(
        sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS,
        MALLOC_CAP_SPIRAM
    );

    if (!s_store) {
        ESP_LOGE(TAG, "Failed to allocate param store in PSRAM");
        return;
    }

    memset(s_store, 0,
           sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS);
    s_count = 0;

    ESP_LOGI(TAG, "initialized in PSRAM (%d bytes)",
             (int)(sizeof(kx_control_params_t) * KX_PARAM_MAX_CONTROLS));
}

esp_err_t kx_param_store_parse(const char *payload, size_t len,
                                int control_id)
{
    // auto-init si no se llamó explícitamente
    if (!s_store) {
        kx_param_store_init();
    }
    if (!s_store) {
        ESP_LOGE(TAG, "store not available");
        return ESP_FAIL;
    }

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

    // reset params del control
    ctrl->count = 0;

    cJSON *regs = cJSON_GetObjectItem(root, "control_regs");
    if (!regs || !cJSON_IsArray(regs)) {
        ESP_LOGW(TAG, "control %d: no control_regs array", control_id);
        cJSON_Delete(root);
        return ESP_FAIL;
    }

    int total = cJSON_GetArraySize(regs);
    ESP_LOGI(TAG, "control %d: parsing %d registers", control_id, total);

    for (int i = 0; i < total && ctrl->count < KX_PARAM_MAX_PER_CONTROL; i++) {
        cJSON *reg = cJSON_GetArrayItem(regs, i);
        if (!reg) continue;

        kx_param_t *p = &ctrl->params[ctrl->count];

        p->param_id       = _get_int(reg,   "control_parameter_id",             0);
        p->reg            = _get_int(reg,   "control_parameter_register",        0);
        p->function_read  = _get_int(reg,   "control_parameter_function_read",   0);
        p->function_write = _get_int(reg,   "control_parameter_function_write",  0);
        p->minvalue       = _get_float(reg, "control_parameter_minvalue",        0.0f);
        p->maxvalue       = _get_float(reg, "control_parameter_maxvalue",      100.0f);
        p->offset         = _get_float(reg, "control_parameter_offset",          0.0f);
        p->mask           = _get_int(reg,   "control_parameter_mask",            0);
        p->view           = _get_int(reg,   "control_parameter_view",            1);
        p->sampling       = _get_int(reg,   "control_parameter_sampling",        60);

        _get_str(reg, "control_parameter_name",
                 p->name,     sizeof(p->name));
        _get_str(reg, "control_parameter_category_system",
                 p->category, sizeof(p->category));
        _get_str(reg, "control_parameter_length",
                 p->length,   sizeof(p->length));

        // addition puede ser null
        cJSON *add = cJSON_GetObjectItem(reg, "control_parameter_addition");
        p->addition = (add && cJSON_IsNumber(add))
                      ? (float)add->valuedouble : 0.0f;

        if (p->param_id > 0) {
            ESP_LOGI(TAG, "  [%d/%d] param_id=%d name=%s min=%.2f max=%.2f fr=%d fw=%d",
                    ctrl->count + 1, total,
                    p->param_id, p->name,
                    p->minvalue, p->maxvalue,
                    p->function_read, p->function_write);
            ctrl->count++;
        }

        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "control %d: %d params stored", control_id, ctrl->count);
        for (int i = 0; i < ctrl->count; i++) {
            ESP_LOGI(TAG, "  param_id=%d [%s]",
                    ctrl->params[i].param_id,
                    ctrl->params[i].name);
        }
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }

    cJSON_Delete(root);

    ESP_LOGI(TAG, "control %d: stored %d/%d params heap=%lu",
             control_id, ctrl->count, total,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DEFAULT));

    return ESP_OK;
}

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

void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data)
{
    if (!s_store || !cb) return;
    for (int i = 0; i < s_count; i++) {
        for (int j = 0; j < s_store[i].count; j++) {
            cb(s_store[i].control_id, &s_store[i].params[j], user_data);
        }
    }
}