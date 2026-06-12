#pragma once
#include "kx_modbus_shared.h"
#include "kx_modbus_tcp_socket.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

// =============================================================
// kx_modbus_tcp_writer.h — Escrituras Modbus TCP
// =============================================================

typedef struct {
    QueueHandle_t      write_queue;
    QueueHandle_t      pub_queue;
    SemaphoreHandle_t  foreach_mutex;
    EventGroupHandle_t poll_eg;
    EventBits_t        poll_allowed_bit;
} kx_tcp_writer_ctx_t;

void kx_tcp_writer_task(void *arg);