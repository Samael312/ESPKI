#pragma once
#include "kx_modbus_shared.h"
#include "kx_modbus_packetizer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>

// =============================================================
// kx_modbus_tcp_dispatch.h — Ejecución de packets TCP
// =============================================================

int kx_dispatch_packet_tcp(QueueHandle_t      pub_queue,
                             SemaphoreHandle_t  foreach_mutex,
                             int                sock_idx,
                             uint8_t            unit_id,
                             const kx_packet_t *pkt,
                             kx_pub_kind_t      pub_kind,
                             int               *out_errors,
                             int64_t            tick_s);