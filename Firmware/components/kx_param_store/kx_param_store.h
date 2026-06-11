#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

// =============================================================
// kx_param_store.h — Almacén con hash de TRES niveles
// =============================================================

// ── Límites y dimensiones ─────────────────────────────────────
#define KX_PARAM_MAX_PER_CONTROL   500
#define KX_PARAM_MAX_CONTROLS       16

#define KX_CTRL_HASH_BUCKETS        16
#define KX_PARAM_HASH_BUCKETS       64
#define KX_REG_HASH_BUCKETS        128

// ── Longitudes de strings ─────────────────────────────────────
#define KX_PARAM_NAME_LEN           64
#define KX_PARAM_CATEGORY_LEN       32
#define KX_PARAM_LENGTH_LEN         16
#define KX_TCP_IP_LEN               40   // IPv4 o IPv6

// =============================================================
// Protocolo de transporte del control
// =============================================================
typedef enum {
    KX_PROTO_RTU = 0,   // Modbus RTU sobre UART/RS485 (por defecto)
    KX_PROTO_TCP,       // Modbus TCP sobre socket lwIP
} kx_proto_t;

// =============================================================
// NIVEL 3 — Registro Modbus
// =============================================================
typedef struct {
    uint16_t reg;
    uint8_t  fc_read;
    uint8_t  fc_write;
} kx_reg_key_t;

typedef struct {
    kx_reg_key_t key;

    float   value;
    int64_t ts_last_read;

    float   last_write_value;
    int64_t ts_last_write;
} kx_reg_entry_t;

typedef struct kx_reg_node {
    kx_reg_entry_t       entry;
    struct kx_reg_node  *next;
} kx_reg_node_t;

typedef struct {
    kx_reg_node_t *buckets[KX_REG_HASH_BUCKETS];
    int            count;
} kx_reg_hash_t;

// =============================================================
// NIVEL 2 — Parámetro / entidad
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
    double  ts_set;

    int64_t ts_last_read;
    float   last_published_value;
} kx_param_t;

typedef struct kx_param_node {
    kx_param_t           param;
    struct kx_param_node *next;
} kx_param_node_t;

typedef struct {
    kx_param_node_t *buckets[KX_PARAM_HASH_BUCKETS];
    int              count;
} kx_param_hash_t;

// =============================================================
// NIVEL 1 — Control
//
// Nuevos campos respecto a la versión RTU-only:
//   proto      — KX_PROTO_RTU o KX_PROTO_TCP
//   tcp_ip     — IP del esclavo TCP  (ej. "172.17.123.250")
//   tcp_port   — Puerto TCP          (ej. 502)
//
// slave_addr se mantiene como unit_id del MBAP cuando proto==TCP.
// =============================================================
typedef struct {
    int              control_id;
    int              slave_addr;     // RTU: dirección RS485 | TCP: unit_id MBAP
    char             uuid[64];
    double           update_ts;
    kx_param_hash_t  params;
    kx_reg_hash_t    regs;
    bool             entities_ready;

    // ── Transporte ────────────────────────────────────────────
    kx_proto_t       proto;                  // KX_PROTO_RTU | KX_PROTO_TCP
    char             tcp_ip[KX_TCP_IP_LEN];  // "" si proto==RTU
    uint16_t         tcp_port;               // 0  si proto==RTU
} kx_control_t;

typedef struct kx_ctrl_node {
    kx_control_t         ctrl;
    struct kx_ctrl_node *next;
} kx_ctrl_node_t;

typedef struct {
    kx_ctrl_node_t *buckets[KX_CTRL_HASH_BUCKETS];
    int             count;
} kx_ctrl_hash_t;

// =============================================================
// Tipos de callback
// =============================================================
typedef void (*kx_param_progress_cb_t)(int control_id, int received, int total);
typedef void (*kx_param_iter_cb_t)(int control_id, const kx_param_t *param, void *user_data);
typedef void (*kx_reg_iter_cb_t)(int control_id, const kx_reg_entry_t *entry, void *user_data);

// =============================================================
// API pública — ciclo de vida
// =============================================================
void      kx_param_store_init(void);
esp_err_t kx_param_store_parse(const char *payload, size_t len, int control_id);

// ── Consulta — nivel control ──────────────────────────────────
const kx_control_t *kx_param_store_get_ctrl(int control_id);
int                 kx_param_store_count(void);

// ── Consulta — nivel param ────────────────────────────────────
const kx_param_t *kx_param_store_get_param(int control_id, int param_id);
kx_param_t       *kx_param_store_get_param_mutable(int control_id, int param_id);

// ── Iteración nivel 2 ─────────────────────────────────────────
void kx_param_store_foreach(kx_param_iter_cb_t cb, void *user_data);

// =============================================================
// API pública — Nivel 3 (caché de registros Modbus)
// =============================================================
kx_reg_entry_t *kx_param_store_reg_upsert_read(int      control_id,
                                                uint16_t reg,
                                                uint8_t  fc_read,
                                                uint8_t  fc_write,
                                                float    value,
                                                int64_t  ts_ms);

esp_err_t kx_param_store_reg_upsert_write(int      control_id,
                                           uint16_t reg,
                                           uint8_t  fc_read,
                                           uint8_t  fc_write,
                                           float    value,
                                           int64_t  ts_ms);

const kx_reg_entry_t *kx_param_store_reg_get(int      control_id,
                                               uint16_t reg,
                                               uint8_t  fc_read,
                                               uint8_t  fc_write);

kx_reg_entry_t *kx_param_store_reg_get_mutable(int      control_id,
                                                uint16_t reg,
                                                uint8_t  fc_read,
                                                uint8_t  fc_write);

int  kx_param_store_reg_count(int control_id);
void kx_param_store_reg_foreach(int control_id, kx_reg_iter_cb_t cb, void *user_data);
void kx_param_store_reg_foreach_all(kx_reg_iter_cb_t cb, void *user_data);

// =============================================================
// API pública — control de completitud
// =============================================================
void kx_param_store_set_expected(int count);
bool kx_param_store_is_ready(void);
void kx_param_store_set_progress_cb(kx_param_progress_cb_t cb);

// ── Configuración por control ─────────────────────────────────
void kx_param_store_set_slave_addr(int control_id, int slave_addr);
void kx_param_store_set_uuid(int control_id, const char *uuid);

// ── update_ts ─────────────────────────────────────────────────
double kx_param_store_get_update_ts(int control_id);
void   kx_param_store_set_update_ts(int control_id, double ts);

// ── Entities ─────────────────────────────────────────────────
void kx_param_store_clear_entities(int control_id);
esp_err_t kx_param_store_set_ts_set(int control_id, int param_id, double ts);

// =============================================================
// API pública — Transporte TCP
//
// Llámadas desde kx_config_handler tras parsear el controls.json.
// Si proto == KX_PROTO_RTU, tcp_ip queda vacío y tcp_port = 0.
// =============================================================

// Establece el endpoint TCP y marca el control como KX_PROTO_TCP.
// Pasa ip=NULL o port=0 para revertir a RTU.
void kx_param_store_set_tcp_endpoint(int         control_id,
                                      const char *ip,
                                      uint16_t    port);

// Devuelve el protocolo activo del control.
kx_proto_t kx_param_store_get_proto(int control_id);

// Rellena ip_out (tamaño >= KX_TCP_IP_LEN) y *port_out.
// Devuelve ESP_ERR_NOT_FOUND si el control no existe o no es TCP.
esp_err_t kx_param_store_get_tcp_endpoint(int      control_id,
                                           char    *ip_out,
                                           uint16_t *port_out);

// ── Persistencia NVS ─────────────────────────────────────────
esp_err_t kx_param_store_save_nvs(void);
esp_err_t kx_param_store_load_nvs(void);
esp_err_t kx_param_store_clear_nvs(void);
bool      kx_param_store_nvs_valid(void);

// ── Compatibilidad ───────────────────────────────────────────
typedef kx_control_t kx_control_params_t;

static inline const kx_control_params_t *kx_param_store_get(int control_id)
{
    return kx_param_store_get_ctrl(control_id);
}

void kx_param_pub_error(int control_id, int param_id,
                         const char *msg, uint16_t reg);

void kx_param_store_print_active_samplings(void);