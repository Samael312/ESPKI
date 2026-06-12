#pragma once
#include "kx_modbus_shared.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <stdbool.h>

// =============================================================
// kx_modbus_tcp_pipeline.h — Tarea demand TCP
// =============================================================

typedef struct {
    QueueHandle_t      pub_queue;
    QueueHandle_t      demand_queue;
    SemaphoreHandle_t  foreach_mutex;
    EventGroupHandle_t poll_eg;
    EventBits_t        poll_allowed_bit;
    EventBits_t        demand_bit;
    volatile uint8_t  *pending_bits;
    volatile bool     *running;
} kx_tcp_pipeline_ctx_t;

void kx_tcp_demand_task(void *arg);

// Devuelve la lista de control_id con proto TCP.
// Usado también por kx_modbus_tcp_report.c
int kx_tcp_get_ctrl_ids(int *out, int max);