#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// =============================================================
// kx_param_store.h — Almacén de registros de entities
//   · Fase 1: almacenamiento en RAM (PSRAM si disponible)
//   · Fase 1.5: persistencia en NVS para arranque rápido
// =============================================================

#define KX_PARAM_MAX_PER_CONTROL  1000
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
    int         slave_addr;
    kx_param_t  params[KX_PARAM_MAX_PER_CONTROL];
    int         count;
} kx_control_params_t;

// ── Callback de progreso de descarga ─────────────────────────
// Se llama durante kx_param_store_parse() con cada param parseado.
//   control_id : id del control que se está recibiendo
//   received   : params procesados hasta ahora en este control
//   total      : total de params en este control (0 = desconocido)
typedef void (*kx_param_progress_cb_t)(int control_id,
                                        int received,
                                        int total);

// ── Ciclo de vida ─────────────────────────────────────────────
void      kx_param_store_init(void);
esp_err_t kx_param_store_parse(const char *payload, size_t len,
                                int control_id);

// ── Persistencia NVS ─────────────────────────────────────────
// Guarda el store completo en NVS (llamar tras recibir todas las entities)
esp_err_t kx_param_store_save_nvs(void);
// Carga el store desde NVS. Devuelve ESP_OK si había datos válidos.
esp_err_t kx_param_store_load_nvs(void);
// Borra la caché NVS (forzar re-descarga en próximo arranque)
esp_err_t kx_param_store_clear_nvs(void);
// True si la caché NVS contiene datos válidos para este UUID/FW
bool      kx_param_store_nvs_valid(void);

// ── Consultas ─────────────────────────────────────────────────
const kx_control_params_t *kx_param_store_get(int control_id);
int  kx_param_store_count(void);

// ── Iteración ─────────────────────────────────────────────────
typedef void (*kx_param_iter_cb_t)(int control_id,
                                    const kx_param_t *param,
                                    void *user_data);
void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data);

// ── Control de completitud ────────────────────────────────────
void kx_param_store_set_expected(int count);
bool kx_param_store_is_ready(void);

// ── Progreso visual ───────────────────────────────────────────
// Registra el callback que se invocará durante el parseo
void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb);

// ── Configuración adicional por control (ej. dirección Modbus) ───
void kx_param_store_set_slave_addr(int control_id, int slave_addr);