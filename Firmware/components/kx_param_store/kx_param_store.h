#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>       // ← size_t
#include "esp_err.h"      // ← esp_err_t

// =============================================================
// kx_param_store.h — Almacén de registros de entities
// =============================================================

#define KX_PARAM_MAX_PER_CONTROL  256
#define KX_PARAM_MAX_CONTROLS     16
#define KX_PARAM_NAME_LEN         64
#define KX_PARAM_CATEGORY_LEN     32
#define KX_PARAM_LENGTH_LEN       16

typedef struct {
    int     param_id;
    int     reg;
    int     function_read;
    int     function_write;
    char    name[KX_PARAM_NAME_LEN];
    char    category[KX_PARAM_CATEGORY_LEN];
    char    length[KX_PARAM_LENGTH_LEN];
    float   minvalue;
    float   maxvalue;
    float   offset;
    float   addition;
    int     mask;
    int     view;
    int     sampling;
} kx_param_t;

typedef struct {
    int         control_id;
    kx_param_t  params[KX_PARAM_MAX_PER_CONTROL];
    int         count;
} kx_control_params_t;

void kx_param_store_init(void);
esp_err_t kx_param_store_parse(const char *payload, size_t len, int control_id);
const kx_control_params_t *kx_param_store_get(int control_id);
int kx_param_store_count(void);

typedef void (*kx_param_iter_cb_t)(int control_id,
                                    const kx_param_t *param,
                                    void *user_data);
void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data);
void kx_param_store_set_expected(int count);
bool kx_param_store_is_ready(void);