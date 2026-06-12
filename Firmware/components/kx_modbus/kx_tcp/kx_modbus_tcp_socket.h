#pragma once
#include "kx_param_store.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stddef.h>
#include <float.h>

// =============================================================
// kx_modbus_tcp_socket.h — Gestión de sockets Modbus TCP
// =============================================================

// ── Constantes de timing y protocolo ─────────────────────────
#define KX_TCP_CONNECT_TIMEOUT_MS    3000
#define KX_TCP_RECV_TIMEOUT_MS        500
#define KX_TCP_RECONNECT_DELAY_MS    2000
#define KX_TCP_MAX_RECONNECT_TRIES      5
#define KX_TCP_RETRY_COUNT              2
#define KX_TCP_RESPONSE_BUF           260
#define KX_MBAP_PDU_OFFSET              7
#define KX_MAX_TCP_SOCKETS              4

// ── FC Modbus ─────────────────────────────────────────────────
#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

// ── Tabla de sockets ──────────────────────────────────────────
typedef struct {
    char     ip[40];
    uint16_t port;
    int      fd;
    uint16_t next_tid;
} kx_tcp_sock_t;

// Inicializa la tabla de sockets y crea el mutex interno.
esp_err_t kx_tcp_socket_init(void);

// Busca o crea + conecta un socket para ip:port.
// Devuelve índice >= 0 o -1 en error.
int kx_tcp_sock_get_or_connect(const char *ip, uint16_t port);

// Transacción MBAP completa: envía PDU, recibe respuesta.
// Devuelve bytes recibidos o -1 en error.
int kx_tcp_transaction(int            sock_idx,
                        uint8_t        unit_id,
                        const uint8_t *pdu,
                        size_t         pdu_len,
                        uint8_t       *resp,
                        size_t         resp_max);

// Lee un registro individual TCP con reintentos.
float kx_tcp_read_register(int               sock_idx,
                             uint8_t           unit_id,
                             uint16_t          reg,
                             uint8_t           fc,
                             const kx_param_t *param,
                             uint16_t         *raw_out);

// Lee num_regs registros consecutivos TCP.
int kx_tcp_read_multi(int      sock_idx,
                       uint8_t  unit_id,
                       uint16_t start_reg,
                       uint16_t num_regs,
                       uint8_t  fc,
                       uint8_t *resp_buf,
                       size_t   resp_max);