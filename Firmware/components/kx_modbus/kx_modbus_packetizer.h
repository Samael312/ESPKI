#pragma once
#include "kx_param_store.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_packetizer.h  —  Agrupador de registros Modbus
//
// Analiza los parámetros de un control y genera una lista de
// "packets" optimizados:
//
//   · Registros con el mismo fc_read y direcciones consecutivas
//     se agrupan en UN solo packet multi-registro.
//   · Registros aislados o con FC distinto se emiten como
//     packets individuales (num_regs = 1).
//
// KX_PKT_MAX_GAP controla cuántos huecos se toleran al agrupar.
// KX_PKT_MAX_REGS_PER_PKT es el límite físico del protocolo.
//
// MODO REPORT (use_tick_filter=true):
//   El caller pasa tick_s y solo se incluyen params cuyo
//   sampling > 0 y tick_s % sampling == 0.
//   Esto garantiza que cada param se lee exactamente en su
//   período, igual que hacía _report_param_cb original.
//
// MODO DEMAND (use_tick_filter=false):
//   Se incluyen todos los params visibles con fc_read válido,
//   ignorando el tick (demanda inmediata o ciclo completo).
// =============================================================

#define KX_PKT_MAX_REGS_PER_PKT   125
#define KX_PKT_MAX_PARAMS_PER_PKT 125

#ifndef KX_PKT_MAX_GAP
#define KX_PKT_MAX_GAP   0
#endif

// =============================================================
// Tipos públicos
// =============================================================
typedef struct {
    int      control_id;
    int      param_id;
    uint16_t reg;
    bool     is_gap;
} kx_pkt_slot_t;

typedef struct {
    uint8_t       slave_addr;
    uint8_t       fc;
    uint16_t      start_reg;
    uint16_t      num_regs;
    int           num_slots;
    kx_pkt_slot_t slots[KX_PKT_MAX_PARAMS_PER_PKT];
} kx_packet_t;

typedef struct {
    kx_packet_t *pkts;
    int          count;
    int          capacity;
} kx_packet_list_t;

// =============================================================
// API pública
// =============================================================

// Construye la lista de packets para el control indicado.
//
// control_id:      control a procesar.
// demand_active:   true  → incluir todos los params visibles
//                           (ciclo completo bajo demanda).
//                  false → filtrar por tick_s % sampling == 0
//                           (modo report periódico).
// tick_s:          segundo actual del timer de reports.
//                  Ignorado cuando demand_active=true.
// now_ms:          timestamp en milisegundos (esp_timer).
//
// Devuelve NULL si no hay params que leer o si falla la memoria.
// El llamador debe liberar con kx_pkt_free().
kx_packet_list_t *kx_pkt_build(int     control_id,
                                bool    demand_active,
                                int64_t tick_s,
                                int64_t now_ms);

void kx_pkt_free(kx_packet_list_t *list);
int  kx_pkt_real_param_count(const kx_packet_list_t *list);
void kx_pkt_dump(const kx_packet_list_t *list, const char *tag);