#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <stdbool.h>

// =============================================================
// kx_modbus_report.h — Reports periódicos RTU
// =============================================================

typedef struct {
    QueueHandle_t       pub_queue;
    SemaphoreHandle_t   foreach_mutex;
    EventGroupHandle_t  poll_eg;
    EventBits_t         poll_allowed_bit;
    volatile bool      *running;
} kx_report_task_ctx_t;

void kx_report_task(void *arg);