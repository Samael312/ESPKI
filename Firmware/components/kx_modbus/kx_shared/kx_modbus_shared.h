#pragma once
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>
#include <float.h>

// =============================================================
// kx_modbus_shared.h — Tipos y helpers compartidos RTU/TCP
// =============================================================

// ── Tipos de publicación ──────────────────────────────────────
typedef enum {
    PUB_KIND_STATUS,
    PUB_KIND_REPORT,
    PUB_KIND_ERROR,
} kx_pub_kind_t;

// Firma para publicaciones status/report
typedef void (*kx_pub_fn_t)(int control_id, int param_id, float value);
// Firma para publicaciones de error
typedef void (*kx_pub_err_fn_t)(int control_id, int param_id,
                                 const char *msg, uint16_t reg);

// ── Entrada de la cola de publicación ────────────────────────
// Usada tanto por RTU como por TCP.
// RTU usa pub_fn / pub_err_fn (punteros a función de telemetría).
// TCP usa kind (enum) para el mismo propósito.
// Ambos campos coexisten en la misma struct por compatibilidad.
typedef struct {
    // RTU style
    kx_pub_fn_t     pub_fn;
    kx_pub_err_fn_t pub_err_fn;
    // TCP style
    kx_pub_kind_t   kind;
    // Payload común
    int             control_id;
    int             param_id;
    uint16_t        reg;
    float           value;
    char            error_msg[32];
} kx_pub_result_t;

// ── Demanda de poll ───────────────────────────────────────────
typedef struct {
    int     param_id;
    int64_t enqueued_ms;
} kx_poll_demand_t;

// ── Comando de escritura ──────────────────────────────────────
typedef struct {
    int    control_id;
    int    param_id;
    float  value;
    double ts;
} kx_write_cmd_t;

// ── Bitmap de demandas pendientes (dedup) ─────────────────────
#define KX_PENDING_SET_SIZE  1024

// Macros inline para operar el bitmap — cada driver declara
// su propio array s_pending_bits[KX_PENDING_SET_SIZE / 8]
#define KX_PENDING_SET(bits, param_id) \
    do { uint32_t _i = ((uint32_t)(param_id)) & (KX_PENDING_SET_SIZE-1); \
         (bits)[_i/8] |= (1u << (_i%8)); } while(0)

#define KX_PENDING_CLEAR(bits, param_id) \
    do { uint32_t _i = ((uint32_t)(param_id)) & (KX_PENDING_SET_SIZE-1); \
         (bits)[_i/8] &= ~(1u << (_i%8)); } while(0)

#define KX_PENDING_TEST(bits, param_id) \
    ({ uint32_t _i = ((uint32_t)(param_id)) & (KX_PENDING_SET_SIZE-1); \
       (bool)(((bits)[_i/8] >> (_i%8)) & 1u); })

// ── Constantes de backpressure publicación ───────────────────
#define KX_PUB_BACKPRESSURE_WAIT_MS    20
#define KX_PUB_BACKPRESSURE_TIMEOUT_MS 2000