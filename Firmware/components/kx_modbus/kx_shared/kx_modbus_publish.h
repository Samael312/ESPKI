#pragma once
#include "kx_modbus_shared.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================
// kx_modbus_publish.h — Fan-out de publicaciones RTU/TCP
// =============================================================

// Encola un resultado genérico con backpressure.
bool kx_pub_enqueue(QueueHandle_t queue, const kx_pub_result_t *r);

// Helpers de construcción + encolado
bool kx_pub_enqueue_status(QueueHandle_t  queue,
                            kx_pub_fn_t    pub_fn,
                            kx_pub_kind_t  kind,
                            int ctrl_id, int param_id, float value);

bool kx_pub_enqueue_error(QueueHandle_t   queue,
                           kx_pub_err_fn_t pub_err_fn,
                           int ctrl_id, int param_id,
                           uint16_t reg, const char *errmsg);

// Fan-out: publica a todos los params que comparten (reg, fc_read).
// tick_s == -1  → demand (todos)
// tick_s >= 0   → report (filtrado por sampling)
int kx_publish_all_params_for_reg(QueueHandle_t   queue,
                                   kx_pub_fn_t     pub_fn,
                                   kx_pub_kind_t   pub_kind,
                                   int             control_id,
                                   uint16_t        reg,
                                   uint8_t         fc_read,
                                   uint16_t        raw,
                                   int64_t         ts_ms,
                                   int64_t         tick_s);