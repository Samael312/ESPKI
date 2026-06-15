#pragma once
#include "kx_param_store.h"
#include <stdbool.h>
#include <stddef.h>
#include <float.h>

// =============================================================
// kx_param_store_internal.h — Compartido entre los 3 archivos
//                              del componente kx_param_store
//
// Declara el estado global (s_hash, s_initialized) y las
// operaciones internas de los 3 niveles de hash, además de
// helpers compartidos (alloc PSRAM, progreso).
//
// NO incluir desde fuera del componente — usar kx_param_store.h
// para la API pública.
// =============================================================

// ── Estado global (definido en kx_param_store_hash.c) ────────
extern kx_ctrl_hash_t s_hash;
extern bool           s_initialized;

// ── Alloc PSRAM-aware ─────────────────────────────────────────
void *kx_psram_alloc(size_t size);

// ── NIVEL 1 — controles ───────────────────────────────────────
kx_control_t *kx_ctrl_find(int control_id);
kx_control_t *kx_ctrl_find_or_create(int control_id);
kx_control_t *kx_ctrl_find_or_create_no_reset(int control_id);

// ── NIVEL 2 — params ──────────────────────────────────────────
esp_err_t          kx_param_insert(kx_control_t *ctrl, const kx_param_t *param);
const kx_param_t  *kx_param_find(const kx_control_t *ctrl, int param_id);
kx_param_t        *kx_param_find_mutable(kx_control_t *ctrl, int param_id);

// ── NIVEL 3 — registros ───────────────────────────────────────
kx_reg_entry_t       *kx_reg_find_mutable(kx_reg_hash_t *rh, uint16_t reg,
                                           uint8_t fc_read, uint8_t fc_write);
const kx_reg_entry_t *kx_reg_find(const kx_reg_hash_t *rh, uint16_t reg,
                                   uint8_t fc_read, uint8_t fc_write);
kx_reg_entry_t       *kx_reg_ensure(kx_reg_hash_t *rh, uint16_t reg,
                                     uint8_t fc_read, uint8_t fc_write);
void                  kx_reg_clear_all(kx_reg_hash_t *rh);

// ── NVS (definido en kx_param_store_nvs.c) ────────────────────
void kx_nvs_storage_init(void);

// Cabecera por control en NVS — incluye campos TCP
typedef struct {
    int      control_id;
    int      slave_addr;
    int      count;
    double   update_ts;
    char     uuid[64];
    uint8_t  proto;
    char     tcp_ip[KX_TCP_IP_LEN];
    uint16_t tcp_port;
} kx_nvs_ctrl_hdr_t;

#define KX_NVS_NS_STORE     "kx_entities"
#define KX_NVS_KEY_MAGIC    "magic"
#define KX_NVS_KEY_UUID     "uuid"
#define KX_NVS_KEY_FW       "fw"
#define KX_NVS_KEY_COUNT    "count"
#define KX_NVS_MAGIC_VALUE  0xE5712A05U
#define KX_NVS_MAX_BLOB      3840U
#define KX_NVS_CHUNK_PARAMS  (KX_NVS_MAX_BLOB / sizeof(kx_param_t))
#define KX_NVS_PARTITION     "storage"

// ── Diagnóstico (definido en kx_param_store_api.c) ────────────
void kx_print_progress(int control_id, int done, int total);