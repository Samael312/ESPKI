#pragma once
#include "cJSON.h"
#include <stddef.h>
#include <sys/time.h>

// =============================================================
// kx_config_internal.h — Compartido entre kx_config_handler.c
//                         y kx_config_protocol.c
// =============================================================

// ── Timestamp ─────────────────────────────────────────────────
static inline double kx_config_ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

// ── ACK / Error (definidos en kx_config_handler.c) ────────────
void kx_config_send_ack(const char *config_type);
void kx_config_send_error(const char *config_type,
                           const char *error_code,
                           const char *detail);

// ── Discovery por control (definido en kx_config_handler.c) ──
void kx_config_request_entities(int control_id);

// ── Procesamiento de un control individual
//    (definido en kx_config_protocol.c) ───────────────────────
void kx_config_process_single_control(cJSON *ctrl_json, int hint_control_id);