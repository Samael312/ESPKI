#pragma once
#include "kx_param_store.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_packetizer.h  —  Agrupador de registros Modbus
//
// Tres modos de construcción (kx_pkt_build):
//
//   MODO DEMAND-FULL  (demand_active=true,  param_ids=NULL)
//     Incluye todos los params visibles con fc_read válido.
//     Usado en ciclo completo (param_id==0 en la cola).
//
//   MODO DEMAND-SET   (demand_active=true,  param_ids!=NULL)
//     Incluye solo los param_ids del array dado.
//     Usado en batch de demandas individuales: agrupa los
//     registros consecutivos del set en un solo packet.
//
//   MODO REPORT       (demand_active=false, param_ids=NULL)
//     Filtra por tick_s % param->sampling == 0.
//     Usado en la tarea de reports periódicos.
//
// KX_PKT_MAX_GAP controla cuántos huecos se toleran al agrupar.
// KX_PKT_MAX_REGS_PER_PKT es el límite físico del protocolo
// (125 holding/input regs por trama Modbus RTU).
// =============================================================

#define KX_PKT_MAX_REGS_PER_PKT   3
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
kx_packet_list_t *kx_pkt_build(int            control_id,
                                bool           demand_active,
                                const int     *param_ids,
                                int            n_param_ids,
                                int64_t        tick_s,
                                int64_t        now_ms);

void kx_pkt_free(kx_packet_list_t *list);
int  kx_pkt_real_param_count(const kx_packet_list_t *list);
void kx_pkt_dump(const kx_packet_list_t *list, const char *tag);