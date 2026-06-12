#pragma once
#include "kx_modbus_shared.h"
#include "kx_modbus_packetizer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>

// =============================================================
// kx_modbus_dispatch.h — Ejecución de packets RTU
// =============================================================

// Ejecuta un packet individual contra el bus RTU.
// Encola resultados en pub_queue.
// tick_s == -1 → demand, tick_s >= 0 → report
int kx_dispatch_packet(QueueHandle_t         pub_queue,
                        SemaphoreHandle_t     foreach_mutex,
                        const kx_packet_t    *pkt,
                        kx_pub_fn_t           pub_fn,
                        int                  *out_errors,
                        int64_t               tick_s);

// Itera todos los packets de una lista, tomando el mutex
// entre cada uno.
void kx_dispatch_control_packets(QueueHandle_t      pub_queue,
                                  SemaphoreHandle_t  foreach_mutex,
                                  kx_packet_list_t  *list,
                                  kx_pub_fn_t        pub_fn,
                                  int               *out_ok,
                                  int               *out_errors,
                                  int64_t            tick_s);