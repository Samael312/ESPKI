#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <stdbool.h>

// =============================================================
// kx_modbus_tcp_report.h — Reports periódicos TCP
// =============================================================

// Firma del helper que devuelve la lista de control_ids TCP
typedef int (*kx_get_tcp_ctrl_ids_fn_t)(int *out, int max);

typedef struct {
    QueueHandle_t            pub_queue;
    SemaphoreHandle_t         foreach_mutex;
    EventGroupHandle_t        poll_eg;
    EventBits_t               poll_allowed_bit;
    volatile bool            *running;
    kx_get_tcp_ctrl_ids_fn_t  get_tcp_ctrl_ids;
} kx_tcp_report_ctx_t;

void kx_tcp_report_task(void *arg);