#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// =============================================================
// kx_param_store.h — Almacén con hash de dos niveles
//
//   Nivel 1 — hash de controles
//     clave : control_id  (int)
//     tabla : KX_CTRL_HASH_BUCKETS buckets, chaining con lista
//
//   Nivel 2 — hash de params dentro de cada control
//     clave : param_id  (int)
//     tabla : KX_PARAM_HASH_BUCKETS buckets, chaining con lista
//
//   Toda la memoria se asigna en PSRAM (heap_caps_malloc) con
//   fallback a RAM interna si PSRAM no está disponible.
// =============================================================

// ── Límites y dimensiones de las tablas hash ──────────────────
#define KX_PARAM_MAX_PER_CONTROL   500   // máx params por control
#define KX_PARAM_MAX_CONTROLS       16   // máx controles

// Tamaño de las tablas hash — potencias de 2 para que el módulo
// sea un AND en lugar de una división.
#define KX_CTRL_HASH_BUCKETS        16   // ≥ KX_PARAM_MAX_CONTROLS
#define KX_PARAM_HASH_BUCKETS       64   // ≥ params esperados / LF

// ── Longitudes de strings ─────────────────────────────────────
#define KX_PARAM_NAME_LEN           64
#define KX_PARAM_CATEGORY_LEN       32
#define KX_PARAM_LENGTH_LEN         16

// =============================================================
// Registro de un parámetro / entidad
// =============================================================
typedef struct {
    int     param_id;
    int     reg;
    int     function_read;
    int     function_write;
    char    name    [KX_PARAM_NAME_LEN];
    char    category[KX_PARAM_CATEGORY_LEN];
    char    length  [KX_PARAM_LENGTH_LEN];
    float   minvalue;
    float   maxvalue;
    float   offset;
    float   addition;
    int     mask;
    int     view;
    int     sampling;
} kx_param_t;

// ── Nodo de la lista de params (hash nivel 2) ─────────────────
typedef struct kx_param_node {
    kx_param_t          param;
    struct kx_param_node *next;
} kx_param_node_t;

// ── Tabla hash de params (nivel 2) ───────────────────────────
typedef struct {
    kx_param_node_t *buckets[KX_PARAM_HASH_BUCKETS];
    int              count;   // número de params insertados
} kx_param_hash_t;

// =============================================================
// Control — contiene su propia hash de params
// =============================================================
typedef struct {
    int              control_id;
    int              slave_addr;
    kx_param_hash_t  params;          // hash nivel 2
    bool             entities_ready;
} kx_control_t;

// ── Nodo de la lista de controles (hash nivel 1) ──────────────
typedef struct kx_ctrl_node {
    kx_control_t      ctrl;
    struct kx_ctrl_node *next;
} kx_ctrl_node_t;

// ── Tabla hash de controles (nivel 1) ────────────────────────
typedef struct {
    kx_ctrl_node_t *buckets[KX_CTRL_HASH_BUCKETS];
    int             count;   // número de controles insertados
} kx_ctrl_hash_t;

// =============================================================
// Tipos de callback
// =============================================================

// Progreso de parseo de entities para un control
//   control_id : control que se está recibiendo
//   received   : params procesados hasta ahora
//   total      : total de params en este control
typedef void (*kx_param_progress_cb_t)(int control_id,
                                        int received,
                                        int total);

// Iterador sobre todos los params de todos los controles
typedef void (*kx_param_iter_cb_t)(int                control_id,
                                    const kx_param_t  *param,
                                    void              *user_data);

// =============================================================
// API pública
// =============================================================

// ── Ciclo de vida ─────────────────────────────────────────────

// Inicializa la hash (asigna memoria en PSRAM).
// Idempotente: llamadas repetidas no hacen nada.
void kx_param_store_init(void);

// Parsea un JSON de entities y lo inserta en la hash.
// control_id se obtiene del topic MQTT antes de llamar.
esp_err_t kx_param_store_parse(const char *payload, size_t len,
                                int control_id);

// ── Consulta — nivel control ──────────────────────────────────

// Devuelve el control (const) o NULL si no existe.
const kx_control_t *kx_param_store_get_ctrl(int control_id);

// Número de controles en la hash.
int kx_param_store_count(void);

// ── Consulta — nivel param ────────────────────────────────────

// Devuelve el param (const) dado control_id + param_id, o NULL.
const kx_param_t *kx_param_store_get_param(int control_id, int param_id);

// ── Iteración ─────────────────────────────────────────────────

// Recorre todos los params de todos los controles.
void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data);

// ── Control de completitud ────────────────────────────────────

// Fija cuántos controles se esperan (para saber cuándo está listo).
void kx_param_store_set_expected(int count);

// True cuando todos los controles esperados tienen entities_ready.
bool kx_param_store_is_ready(void);

// ── Progreso visual ───────────────────────────────────────────
void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb);

// ── Configuración adicional por control ──────────────────────
void kx_param_store_set_slave_addr(int control_id, int slave_addr);

// ── Persistencia NVS ─────────────────────────────────────────
esp_err_t kx_param_store_save_nvs(void);
esp_err_t kx_param_store_load_nvs(void);
esp_err_t kx_param_store_clear_nvs(void);
bool      kx_param_store_nvs_valid(void);

// ── Compatibilidad con código existente ──────────────────────
// kx_modbus_master.c y kx_dummy_protocol.c usan kx_control_params_t.
// Definimos un alias para no tocar esos archivos ahora.
typedef kx_control_t kx_control_params_t;

// Alias de función para el código que llama a kx_param_store_get()
// con el nombre antiguo.
static inline const kx_control_params_t *
kx_param_store_get(int control_id)
{
    return kx_param_store_get_ctrl(control_id);
}