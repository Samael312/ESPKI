#pragma once
#include "kx_modbus_shared.h"
#include "kx_modbus_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <stdbool.h>

// =============================================================
// kx_modbus_pipeline.h — Tarea principal demand RTU
// =============================================================

// Contexto completo inyectado en la tarea
typedef struct {
    // Colas
    QueueHandle_t      pub_queue;
    QueueHandle_t      demand_queue;
    QueueHandle_t      write_queue;
    // Tamaños para el resumen
    int                pub_queue_size;
    int                demand_queue_size;
    int                write_queue_size;
    // Sincronización
    SemaphoreHandle_t  foreach_mutex;
    EventGroupHandle_t poll_eg;
    EventBits_t        poll_allowed_bit;
    EventBits_t        demand_bit;
    EventBits_t        batch_active_bit;
    // Bitmap dedup
    volatile uint8_t  *pending_bits;
    // Control de vida
    volatile bool     *running;
} kx_pipeline_ctx_t;

void kx_modbus_task(void *arg);