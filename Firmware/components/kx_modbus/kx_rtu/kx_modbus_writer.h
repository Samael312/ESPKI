#pragma once
#include "kx_modbus_shared.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// =============================================================
// kx_modbus_writer.h — Escrituras Modbus RTU
// =============================================================

// Contexto inyectado en la tarea writer al crearse
typedef struct {
    QueueHandle_t      write_queue;
    QueueHandle_t      pub_queue;
    SemaphoreHandle_t  foreach_mutex;
    EventGroupHandle_t poll_eg;
    EventBits_t        poll_allowed_bit;
} kx_writer_ctx_t;

// Tarea writer — alta prioridad, bloquea en write_queue
void kx_writer_task(void *arg);