#include "kx_modbus_tcp.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "kx_telemetry.h"
#include "kx_modbus_packetizer.h"
#include "../../main/kx_config.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/errno.h"

#include <string.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <inttypes.h>
#include <sys/time.h>

static const char *TAG = "kx_modbus_tcp";

// =============================================================
// Constantes de protocolo y timing
// =============================================================
#define MBAP_PDU_OFFSET          7
#define TCP_CONNECT_TIMEOUT_MS   3000
#define TCP_RECV_TIMEOUT_MS      500
#define TCP_RECONNECT_DELAY_MS   2000
#define TCP_MAX_RECONNECT_TRIES  5
#define TCP_RESPONSE_BUF         260

#define MB_FC_READ_COILS          0x01
#define MB_FC_READ_DISCRETE       0x02
#define MB_FC_READ_HOLDING_REGS   0x03
#define MB_FC_READ_INPUT_REGS     0x04
#define MB_FC_WRITE_SINGLE_COIL   0x05
#define MB_FC_WRITE_SINGLE_REG    0x06
#define MB_FC_WRITE_MULTIPLE_REGS 0x10

// =============================================================
// Tabla de sockets — un socket por ip:port único
// =============================================================
#define MAX_TCP_SOCKETS  4   // reducido: típicamente 1-2 PLCs

typedef struct {
    char     ip[KX_TCP_IP_LEN];
    uint16_t port;
    int      fd;
    uint16_t next_tid;
} kx_tcp_sock_t;

static kx_tcp_sock_t s_socks[MAX_TCP_SOCKETS];
static int           s_n_socks = 0;
static SemaphoreHandle_t s_sock_mutex = NULL;

// =============================================================
// Colas — dimensionadas para no agotar RAM interna
//
// El driver RTU ya ocupa:
//   pub_queue:    500 × 52 bytes ≈ 26 KB  (RAM interna)
//   demand_queue: 1500 × 12 bytes ≈ 18 KB (RAM interna)
//   write_queue:   64 × 20 bytes ≈  1.3 KB
//
// TCP usa colas mucho más pequeñas porque:
//   · El PLC TCP es 1 dispositivo, no un bus compartido
//   · Las respuestas son rápidas (LAN local, 1-5ms)
//   · No hay ráfagas masivas de demandas simultáneas
// =============================================================
#define PUB_QUEUE_SIZE       64   // era 500  — ahora 64 × 52b ≈ 3.3 KB
#define WRITE_QUEUE_SIZE     16   // era 64   — suficiente para escrituras
#define DEMAND_QUEUE_SIZE   128   // era 1500 — 128 × 12b ≈ 1.5 KB
#define DEMAND_WARN_HWM      96

#define BURST_COLLECT_MAX_MS  2000
#define BURST_STABLE_MS        200
#define BURST_POLL_MS           50

#define REPORT_TICK_PERIOD_S   864000
#define REPORT_TASK_PERIOD_MS  1000
#define REPORT_LOG_MAX_PARAMS   64   // era 256

// =============================================================
// Stacks de tareas — reducidos respecto a RTU
//
// TCP no necesita stacks grandes: no hay UART, no hay packetizer
// de gran profundidad. 3072 es suficiente para las tareas ligeras.
// La tarea demand necesita algo más por el malloc de snapshot.
// =============================================================
#define STACK_TCP_DEMAND   5120   // era 8192
#define STACK_TCP_WRITER   3072   // era 4096
#define STACK_TCP_REPORT   3072   // era 4096
#define STACK_TCP_PUB      4096   

#define POLL_ALLOWED_BIT  BIT0
#define DEMAND_BIT        BIT1

typedef enum { PUB_KIND_STATUS, PUB_KIND_REPORT, PUB_KIND_ERROR } kx_pub_kind_t;

typedef struct {
    kx_pub_kind_t kind;
    int           control_id;
    int           param_id;
    uint16_t      reg;
    float         value;
    char          error_msg[32];
} kx_pub_result_t;

typedef struct {
    int    control_id;
    int    param_id;
    float  value;
    double ts;
} kx_write_cmd_t;

typedef struct {
    int     param_id;
    int64_t enqueued_ms;
} kx_poll_demand_t;

static QueueHandle_t      s_pub_queue    = NULL;
static QueueHandle_t      s_write_queue  = NULL;
static QueueHandle_t      s_demand_queue = NULL;
static EventGroupHandle_t s_poll_eg      = NULL;
static SemaphoreHandle_t  s_foreach_mutex = NULL;
static volatile bool      s_running      = false;
static volatile int64_t   s_report_tick_s = -1;

// ── Bitmap de demandas pendientes ─────────────────────────────
#define PENDING_SET_SIZE 1024
static volatile uint8_t s_pending_bits[PENDING_SET_SIZE / 8] = {0};
static inline void  _pending_set(int p)   { uint32_t i=((uint32_t)p)&(PENDING_SET_SIZE-1); s_pending_bits[i/8]|=(1u<<(i%8)); }
static inline void  _pending_clear(int p) { uint32_t i=((uint32_t)p)&(PENDING_SET_SIZE-1); s_pending_bits[i/8]&=~(1u<<(i%8)); }
static inline bool  _pending_test(int p)  { uint32_t i=((uint32_t)p)&(PENDING_SET_SIZE-1); return (s_pending_bits[i/8]>>(i%8))&1u; }

// =============================================================
// Gestión de sockets
// =============================================================
static int _sock_find(const char *ip, uint16_t port)
{
    for (int i = 0; i < s_n_socks; i++)
        if (s_socks[i].port == port && strcmp(s_socks[i].ip, ip) == 0)
            return i;
    return -1;
}

static int _sock_alloc(const char *ip, uint16_t port)
{
    if (s_n_socks >= MAX_TCP_SOCKETS) {
        ESP_LOGE(TAG, "sock table full (%d)", MAX_TCP_SOCKETS);
        return -1;
    }
    int idx = s_n_socks++;
    snprintf(s_socks[idx].ip, sizeof(s_socks[idx].ip), "%s", ip);
    s_socks[idx].port     = port;
    s_socks[idx].fd       = -1;
    s_socks[idx].next_tid = 1;
    return idx;
}

static void _sock_close(int idx)
{
    if (idx < 0 || idx >= s_n_socks) return;
    if (s_socks[idx].fd >= 0) {
        close(s_socks[idx].fd);
        s_socks[idx].fd = -1;
        ESP_LOGW(TAG, "socket closed: %s:%" PRIu16,
                 s_socks[idx].ip, s_socks[idx].port);
    }
}

static esp_err_t _sock_connect(int idx)
{
    _sock_close(idx);

    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) { ESP_LOGE(TAG, "socket() failed: %d", errno); return ESP_FAIL; }

    // Timeout de recepción
    struct timeval tv = {
        .tv_sec  = TCP_RECV_TIMEOUT_MS / 1000,
        .tv_usec = (TCP_RECV_TIMEOUT_MS % 1000) * 1000,
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Conexión no bloqueante con timeout
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port   = htons(s_socks[idx].port),
    };
    inet_pton(AF_INET, s_socks[idx].ip, &addr.sin_addr);

    int rc = connect(fd, (struct sockaddr *)&addr, sizeof(addr));
    if (rc < 0 && errno != EINPROGRESS) {
        ESP_LOGE(TAG, "connect() failed: %s:%" PRIu16 " err=%d",
                 s_socks[idx].ip, s_socks[idx].port, errno);
        close(fd);
        return ESP_FAIL;
    }

    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    struct timeval ctv = {
        .tv_sec  = TCP_CONNECT_TIMEOUT_MS / 1000,
        .tv_usec = (TCP_CONNECT_TIMEOUT_MS % 1000) * 1000,
    };
    rc = select(fd + 1, NULL, &wfds, NULL, &ctv);
    if (rc <= 0) {
        ESP_LOGE(TAG, "connect timeout: %s:%" PRIu16,
                 s_socks[idx].ip, s_socks[idx].port);
        close(fd);
        return ESP_FAIL;
    }

    int err = 0;
    socklen_t len = sizeof(err);
    getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
    if (err != 0) {
        ESP_LOGE(TAG, "connect SO_ERROR=%d: %s:%" PRIu16,
                 err, s_socks[idx].ip, s_socks[idx].port);
        close(fd);
        return ESP_FAIL;
    }

    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    s_socks[idx].fd = fd;
    ESP_LOGI(TAG, "connected: %s:%" PRIu16 " fd=%d",
             s_socks[idx].ip, s_socks[idx].port, fd);
    return ESP_OK;
}

static int _sock_get_or_connect(const char *ip, uint16_t port)
{
    xSemaphoreTake(s_sock_mutex, portMAX_DELAY);

    int idx = _sock_find(ip, port);
    if (idx < 0) idx = _sock_alloc(ip, port);
    if (idx < 0) { xSemaphoreGive(s_sock_mutex); return -1; }

    if (s_socks[idx].fd < 0) {
        for (int t = 0; t < TCP_MAX_RECONNECT_TRIES; t++) {
            if (_sock_connect(idx) == ESP_OK) break;
            if (t + 1 < TCP_MAX_RECONNECT_TRIES)
                vTaskDelay(pdMS_TO_TICKS(TCP_RECONNECT_DELAY_MS));
        }
    }

    int result = (s_socks[idx].fd >= 0) ? idx : -1;
    xSemaphoreGive(s_sock_mutex);
    return result;
}

// =============================================================
// Transacción Modbus TCP
// =============================================================
static int _tcp_transaction(int         sock_idx,
                             uint8_t     unit_id,
                             const uint8_t *pdu,
                             size_t      pdu_len,
                             uint8_t    *resp,
                             size_t      resp_max)
{
    xSemaphoreTake(s_sock_mutex, portMAX_DELAY);

    int fd = s_socks[sock_idx].fd;
    if (fd < 0) { xSemaphoreGive(s_sock_mutex); return -1; }

    uint16_t tid = s_socks[sock_idx].next_tid++;
    if (s_socks[sock_idx].next_tid == 0)
        s_socks[sock_idx].next_tid = 1;

    // MBAP(7) + PDU
    uint16_t mbap_len = (uint16_t)(1 + pdu_len);
    uint8_t  tx[MBAP_PDU_OFFSET + 260];

    tx[0] = (uint8_t)(tid >> 8);
    tx[1] = (uint8_t)(tid & 0xFF);
    tx[2] = 0x00;
    tx[3] = 0x00;
    tx[4] = (uint8_t)(mbap_len >> 8);
    tx[5] = (uint8_t)(mbap_len & 0xFF);
    tx[6] = unit_id;
    memcpy(&tx[7], pdu, pdu_len);

    int sent = send(fd, tx, 7 + pdu_len, 0);
    if (sent != (int)(7 + pdu_len)) {
        ESP_LOGW(TAG, "send failed fd=%d err=%d", fd, errno);
        _sock_close(sock_idx);
        xSemaphoreGive(s_sock_mutex);
        return -1;
    }

    int rx = recv(fd, resp, resp_max, 0);
    if (rx <= 0) {
        ESP_LOGW(TAG, "recv failed fd=%d err=%d", fd, errno);
        _sock_close(sock_idx);
        xSemaphoreGive(s_sock_mutex);
        return -1;
    }

    xSemaphoreGive(s_sock_mutex);

    if (rx < 8) { ESP_LOGW(TAG, "response too short: %d", rx); return -1; }

    uint16_t resp_tid = ((uint16_t)resp[0] << 8) | resp[1];
    if (resp_tid != tid) {
        ESP_LOGW(TAG, "TID mismatch: sent=%u got=%u", tid, resp_tid);
        return -1;
    }
    if (resp[6] != unit_id) {
        ESP_LOGW(TAG, "unit_id mismatch: sent=%u got=%u", unit_id, resp[6]);
        return -1;
    }
    if (resp[7] & 0x80) {
        uint8_t exc = (rx > 8) ? resp[8] : 0;
        ESP_LOGW(TAG, "Modbus exception fc=0x%02x exc=0x%02x", resp[7], exc);
        return -1;
    }
    return rx;
}

// =============================================================
// Lectura individual y multi con reintentos
// =============================================================
#define TCP_RETRY_COUNT 2

static float _tcp_read_register(int             sock_idx,
                                 uint8_t         unit_id,
                                 uint16_t        reg,
                                 uint8_t         fc,
                                 const kx_param_t *param,
                                 uint16_t        *raw_out)
{
    uint8_t pdu[5] = { fc,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        0x00, 0x01 };
    uint8_t resp[TCP_RESPONSE_BUF];

    int rx = -1;
    for (int a = 0; a < TCP_RETRY_COUNT && rx < 0; a++) {
        if (s_socks[sock_idx].fd < 0 && _sock_connect(sock_idx) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200)); continue;
        }
        rx = _tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                               resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (rx < 0) { if (raw_out) *raw_out = 0; return -FLT_MAX; }

    uint16_t raw;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE) {
        if (rx < 10) { if (raw_out) *raw_out = 0; return -FLT_MAX; }
        raw = resp[9] & 0x01;
    } else {
        if (rx < 11 || resp[8] < 2) { if (raw_out) *raw_out = 0; return -FLT_MAX; }
        raw = ((uint16_t)resp[9] << 8) | resp[10];
    }
    if (raw_out) *raw_out = raw;

    float value = (float)(int16_t)raw;
    if (param->offset != 0.0f && param->offset != 1.0f) value *= param->offset;
    value += param->addition;
    if (value < param->minvalue) value = param->minvalue;
    if (value > param->maxvalue) value = param->maxvalue;
    return value;
}

static int _tcp_read_multi(int sock_idx, uint8_t unit_id,
                            uint16_t start_reg, uint16_t num_regs,
                            uint8_t fc, uint8_t *resp_buf, size_t resp_max)
{
    uint8_t pdu[5] = { fc,
        (uint8_t)(start_reg >> 8), (uint8_t)(start_reg & 0xFF),
        (uint8_t)(num_regs  >> 8), (uint8_t)(num_regs  & 0xFF) };
    int rx = -1;
    for (int a = 0; a < TCP_RETRY_COUNT && rx < 0; a++) {
        if (s_socks[sock_idx].fd < 0 && _sock_connect(sock_idx) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200)); continue;
        }
        rx = _tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                               resp_buf, resp_max);
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(50));
    }
    return rx;
}

// =============================================================
// Cola de publicación
// =============================================================
#define PUB_BACKPRESSURE_WAIT_MS    20
#define PUB_BACKPRESSURE_TIMEOUT_MS 2000

static bool _enqueue(kx_pub_kind_t kind, int ctrl_id, int param_id,
                      float value, uint16_t reg, const char *errmsg)
{
    kx_pub_result_t r = { kind, ctrl_id, param_id, reg, value, {0} };
    if (errmsg) snprintf(r.error_msg, sizeof(r.error_msg), "%s", errmsg);

    if (xQueueSend(s_pub_queue, &r, 0) == pdTRUE) return true;

    int waited = 0;
    while (waited < PUB_BACKPRESSURE_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(PUB_BACKPRESSURE_WAIT_MS));
        waited += PUB_BACKPRESSURE_WAIT_MS;
        if (xQueueSend(s_pub_queue, &r, 0) == pdTRUE) return true;
    }
    ESP_LOGE(TAG, "pub_queue DROP param_id=%d", param_id);
    return false;
}

// =============================================================
// Fan-out: todos los params que comparten (reg, fc_read)
// =============================================================
static int _publish_all_params_for_reg(int control_id, uint16_t reg,
                                        uint8_t fc_read, uint16_t raw,
                                        int64_t ts_ms, kx_pub_kind_t pub_kind,
                                        int64_t tick_s)
{
    int published = 0;
    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl) return 0;

    for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS; pi++) {
        kx_param_node_t *pn = ctrl->params.buckets[pi];
        while (pn) {
            kx_param_t *p = &pn->param;
            if ((uint16_t)p->reg == reg &&
                (uint8_t)p->function_read == fc_read &&
                p->view != 0) {

                if (tick_s >= 0) {
                    if (p->sampling <= 0 ||
                        (tick_s % (int64_t)p->sampling) != 0) {
                        pn = pn->next; continue;
                    }
                }

                float value = (float)(int16_t)raw;
                if (p->offset != 0.0f && p->offset != 1.0f) value *= p->offset;
                value += p->addition;
                if (value < p->minvalue) value = p->minvalue;
                if (value > p->maxvalue) value = p->maxvalue;

                p->ts_last_read         = ts_ms;
                p->last_published_value = value;

                _enqueue(pub_kind, control_id, p->param_id, value, 0, NULL);
                published++;
            }
            pn = pn->next;
        }
    }
    return published;
}

// =============================================================
// Dispatch de un packet TCP
// =============================================================
static int _dispatch_packet_tcp(int sock_idx, uint8_t unit_id,
                                 const kx_packet_t *pkt,
                                 kx_pub_kind_t pub_kind,
                                 int *out_errors, int64_t tick_s)
{
    if (!pkt || pkt->num_slots == 0) return 0;

    int ok_count = 0, err_count = 0;

    // ── Packet individual ─────────────────────────────────────
    if (pkt->num_regs == 1) {
        const kx_pkt_slot_t *slot = &pkt->slots[0];
        if (slot->is_gap) goto done;
        const kx_param_t *param =
            kx_param_store_get_param(slot->control_id, slot->param_id);
        if (!param) { err_count++; goto done; }

        uint16_t raw = 0;
        float value = _tcp_read_register(sock_idx, unit_id,
                                          pkt->start_reg, pkt->fc, param, &raw);
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (value == -FLT_MAX) {
            _enqueue(PUB_KIND_ERROR, slot->control_id, slot->param_id,
                     0.0f, pkt->start_reg, "tcp_timeout");
            err_count++;
        } else {
            kx_param_store_reg_upsert_read(slot->control_id, pkt->start_reg,
                pkt->fc, (uint8_t)param->function_write, value, ts_ms);
            int n = _publish_all_params_for_reg(slot->control_id, pkt->start_reg,
                        pkt->fc, raw, ts_ms, pub_kind, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
        goto done;
    }

    // ── Packet multi-registro ─────────────────────────────────
    {
        uint8_t resp[TCP_RESPONSE_BUF];
        int rx = _tcp_read_multi(sock_idx, unit_id, pkt->start_reg,
                                  pkt->num_regs, pkt->fc, resp, sizeof(resp));
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (rx < 0) {
            // Fallback individual
            ESP_LOGW(TAG, "multi FAILED — fallback unit=%u reg=0x%04x num=%d",
                     unit_id, pkt->start_reg, pkt->num_regs);
            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *sl = &pkt->slots[s];
                if (sl->is_gap || sl->param_id < 0) continue;
                const kx_param_t *p =
                    kx_param_store_get_param(sl->control_id, sl->param_id);
                if (!p) { err_count++; continue; }
                uint16_t raw = 0;
                float val = _tcp_read_register(sock_idx, unit_id,
                                                sl->reg, pkt->fc, p, &raw);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
                if (val == -FLT_MAX) {
                    _enqueue(PUB_KIND_ERROR, sl->control_id, sl->param_id,
                             0.0f, sl->reg, "tcp_timeout");
                    err_count++;
                } else {
                    kx_param_store_reg_upsert_read(sl->control_id, sl->reg,
                        pkt->fc, (uint8_t)p->function_write, val, ts_ms);
                    int n = _publish_all_params_for_reg(sl->control_id, sl->reg,
                                pkt->fc, raw, ts_ms, pub_kind, tick_s);
                    ok_count += (n > 0) ? n : 1;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            goto done;
        }

        // resp[7]=FC, resp[8]=byte_count, resp[9..]=datos
        bool is_coil = (pkt->fc == MB_FC_READ_COILS ||
                        pkt->fc == MB_FC_READ_DISCRETE);
        int expected = is_coil ? (pkt->num_regs + 7) / 8 : pkt->num_regs * 2;

        if (rx < 9 || resp[8] != (uint8_t)expected) {
            ESP_LOGW(TAG, "multi bad byte_count=%d expected=%d",
                     (rx >= 9) ? resp[8] : -1, expected);
            err_count += pkt->num_slots;
            goto done;
        }

        for (int s = 0; s < pkt->num_slots; s++) {
            const kx_pkt_slot_t *sl = &pkt->slots[s];
            int reg_offset = (int)sl->reg - (int)pkt->start_reg;
            uint16_t raw;
            if (is_coil) {
                int bi = reg_offset / 8, bj = reg_offset % 8;
                if (9 + bi >= rx) continue;
                raw = (resp[9 + bi] >> bj) & 0x01;
            } else {
                int bi = reg_offset * 2;
                if (9 + bi + 1 >= rx) continue;
                raw = ((uint16_t)resp[9 + bi] << 8) | resp[9 + bi + 1];
            }
            if (sl->is_gap) {
                kx_param_store_reg_upsert_read(sl->control_id, sl->reg,
                    pkt->fc, 0, (float)(int16_t)raw, ts_ms);
                continue;
            }
            const kx_param_t *p =
                kx_param_store_get_param(sl->control_id, sl->param_id);
            if (!p) { err_count++; continue; }

            float v0 = (float)(int16_t)raw;
            if (p->offset != 0.0f && p->offset != 1.0f) v0 *= p->offset;
            v0 += p->addition;
            if (v0 < p->minvalue) v0 = p->minvalue;
            if (v0 > p->maxvalue) v0 = p->maxvalue;

            kx_param_store_reg_upsert_read(sl->control_id, sl->reg, pkt->fc,
                (uint8_t)p->function_write, v0, ts_ms);
            int n = _publish_all_params_for_reg(sl->control_id, sl->reg,
                        pkt->fc, raw, ts_ms, pub_kind, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
    }

done:
    if (out_errors) *out_errors += err_count;
    return ok_count;
}

// =============================================================
// Helpers para iterar controles TCP
// =============================================================
#define MAX_CTRL_VISITED KX_PARAM_MAX_CONTROLS

typedef struct { int ctrl_ids[MAX_CTRL_VISITED]; int n; } _tcp_ctrl_list_t;

static void _collect_tcp_ctrls_cb(int ctrl_id, const kx_param_t *p, void *ud)
{
    (void)p;
    _tcp_ctrl_list_t *lst = (_tcp_ctrl_list_t *)ud;
    if (kx_param_store_get_proto(ctrl_id) != KX_PROTO_TCP) return;
    for (int i = 0; i < lst->n; i++)
        if (lst->ctrl_ids[i] == ctrl_id) return;
    if (lst->n < MAX_CTRL_VISITED)
        lst->ctrl_ids[lst->n++] = ctrl_id;
}

static int _get_tcp_ctrl_ids(int *out, int max)
{
    _tcp_ctrl_list_t lst = { .n = 0 };
    kx_param_store_foreach(_collect_tcp_ctrls_cb, &lst);
    int n = lst.n < max ? lst.n : max;
    memcpy(out, lst.ctrl_ids, (size_t)n * sizeof(int));
    return n;
}

// =============================================================
// Poll de un control TCP completo
// =============================================================
static void _poll_tcp_control(int ctrl_id, int *out_ok, int *out_errors)
{
    char ip[KX_TCP_IP_LEN]; uint16_t port;
    if (kx_param_store_get_tcp_endpoint(ctrl_id, ip, &port) != ESP_OK) return;

    int sock_idx = _sock_get_or_connect(ip, port);
    if (sock_idx < 0) { ESP_LOGE(TAG, "poll ctrl=%d: no socket", ctrl_id); return; }

    const kx_control_t *ctrl = kx_param_store_get_ctrl(ctrl_id);
    if (!ctrl) return;
    uint8_t unit_id = (uint8_t)ctrl->slave_addr;

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    kx_packet_list_t *list = kx_pkt_build(ctrl_id, true, NULL, 0, 0, now_ms);
    if (!list) return;

    for (int i = 0; i < list->count; i++) {
        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
        int pkt_ok = 0, pkt_err = 0;
        pkt_ok = _dispatch_packet_tcp(sock_idx, unit_id, &list->pkts[i],
                                       PUB_KIND_STATUS, &pkt_err, -1);
        xSemaphoreGive(s_foreach_mutex);
        if (out_ok)     *out_ok     += pkt_ok;
        if (out_errors) *out_errors += pkt_err;
        if (i + 1 < list->count) vTaskDelay(pdMS_TO_TICKS(10));
    }
    kx_pkt_free(list);
}

// =============================================================
// Batch poll — grupo de param_ids TCP
// =============================================================
typedef struct { int target; int found; } _find_ctrl_ctx_t;
static void _find_ctrl_cb(int ctrl_id, const kx_param_t *p, void *ud)
{
    _find_ctrl_ctx_t *ctx = (_find_ctrl_ctx_t *)ud;
    if (ctx->found < 0 && p->param_id == ctx->target)
        ctx->found = ctrl_id;
}

#define MAX_PARAMS_IN_BATCH 512   // reducido de 1500

typedef struct {
    int ctrl_id;
    int param_ids[MAX_PARAMS_IN_BATCH];
    int n_params;
} _ctrl_group_t;

static void _poll_batch_tcp(const kx_poll_demand_t *snap, int valid,
                             int *out_ok, int *out_errors)
{
    _ctrl_group_t *groups = malloc(MAX_CTRL_VISITED * sizeof(_ctrl_group_t));
    if (!groups) { ESP_LOGE(TAG, "batch OOM"); return; }
    int n_groups = 0;

    for (int i = 0; i < valid; i++) {
        _find_ctrl_ctx_t fctx = { snap[i].param_id, -1 };
        kx_param_store_foreach(_find_ctrl_cb, &fctx);
        if (fctx.found < 0) { (*out_errors)++; continue; }
        if (kx_param_store_get_proto(fctx.found) != KX_PROTO_TCP) continue;

        int g = -1;
        for (int j = 0; j < n_groups; j++)
            if (groups[j].ctrl_id == fctx.found) { g = j; break; }
        if (g < 0) {
            if (n_groups >= MAX_CTRL_VISITED) { (*out_errors)++; continue; }
            g = n_groups++;
            groups[g].ctrl_id = fctx.found;
            groups[g].n_params = 0;
        }
        if (groups[g].n_params < MAX_PARAMS_IN_BATCH)
            groups[g].param_ids[groups[g].n_params++] = snap[i].param_id;
    }

    for (int g = 0; g < n_groups && s_running; g++) {
        _ctrl_group_t *grp = &groups[g];
        char ip[KX_TCP_IP_LEN]; uint16_t port;
        if (kx_param_store_get_tcp_endpoint(grp->ctrl_id, ip, &port) != ESP_OK)
            continue;

        int sock_idx = _sock_get_or_connect(ip, port);
        if (sock_idx < 0) { *out_errors += grp->n_params; continue; }

        const kx_control_t *ctrl = kx_param_store_get_ctrl(grp->ctrl_id);
        if (!ctrl) continue;
        uint8_t unit_id = (uint8_t)ctrl->slave_addr;

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
        kx_packet_list_t *list = kx_pkt_build(grp->ctrl_id, true,
                                               grp->param_ids, grp->n_params,
                                               0, now_ms);
        if (!list) { *out_errors += grp->n_params; continue; }

        for (int i = 0; i < list->count; i++) {
            xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
            int pkt_err = 0;
            int pkt_ok = _dispatch_packet_tcp(sock_idx, unit_id, &list->pkts[i],
                                               PUB_KIND_STATUS, &pkt_err, -1);
            xSemaphoreGive(s_foreach_mutex);
            *out_ok     += pkt_ok;
            *out_errors += pkt_err;
            if (i + 1 < list->count) vTaskDelay(pdMS_TO_TICKS(10));
        }
        kx_pkt_free(list);
    }
    free(groups);
}

// =============================================================
// Drain de la cola de demandas
// =============================================================
static int _drain_demand_queue(kx_poll_demand_t *snap, int cap,
                                int *out_exp, int *out_dup)
{
    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    int exp = 0, dup = 0, count = 0;
    kx_poll_demand_t d;

    while (count < cap && xQueueReceive(s_demand_queue, &d, 0) == pdTRUE) {
        _pending_clear(d.param_id);
        if ((now_ms - d.enqueued_ms) > (int64_t)(KX_DEMAND_TIMEOUT_S * 1000)) {
            exp++; continue;
        }
        bool found = false;
        for (int j = 0; j < count; j++) {
            if (snap[j].param_id == d.param_id) {
                if (d.enqueued_ms > snap[j].enqueued_ms)
                    snap[j].enqueued_ms = d.enqueued_ms;
                dup++; found = true; break;
            }
        }
        if (!found) snap[count++] = d;
    }
    while (xQueueReceive(s_demand_queue, &d, 0) == pdTRUE) {}
    if (out_exp) *out_exp = exp;
    if (out_dup) *out_dup = dup;
    return count;
}

// =============================================================
// Tarea demand TCP
// =============================================================
static void _tcp_demand_task(void *arg)
{
    ESP_LOGI(TAG, "demand task started — waiting for entities...");
    while (!kx_param_store_is_ready()) vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "entities ready — TCP demand task running");

    while (s_running) {
        EventBits_t bits = xEventGroupWaitBits(s_poll_eg, DEMAND_BIT,
                                               pdFALSE, pdTRUE, portMAX_DELAY);
        if (!(bits & DEMAND_BIT)) continue;

        // Recopilar ráfaga
        {
            int64_t t0 = (int64_t)(esp_timer_get_time() / 1000ULL);
            int prev = -1, stable = 0;
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(BURST_POLL_MS));
                int cur = (int)uxQueueMessagesWaiting(s_demand_queue);
                int64_t ela = (int64_t)(esp_timer_get_time() / 1000ULL) - t0;
                if (cur == prev) {
                    stable += BURST_POLL_MS;
                    if (stable >= BURST_STABLE_MS) break;
                } else { stable = 0; }
                prev = cur;
                if (ela >= BURST_COLLECT_MAX_MS) break;
            }
        }

        int raw_count = (int)uxQueueMessagesWaiting(s_demand_queue);
        if (raw_count == 0) { xEventGroupClearBits(s_poll_eg, DEMAND_BIT); continue; }

        kx_poll_demand_t *snap = malloc((size_t)raw_count * sizeof(kx_poll_demand_t));
        if (!snap) {
            kx_poll_demand_t tmp;
            while (xQueueReceive(s_demand_queue, &tmp, 0) == pdTRUE) {}
            xEventGroupClearBits(s_poll_eg, DEMAND_BIT);
            continue;
        }

        int exp = 0, dup = 0;
        int valid = _drain_demand_queue(snap, raw_count, &exp, &dup);
        xEventGroupClearBits(s_poll_eg, DEMAND_BIT);

        if (valid == 0) { free(snap); continue; }
        if (!kx_mqtt_is_connected() || !kx_param_store_is_ready()) {
            free(snap); continue;
        }

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int ok = 0, err = 0;

        bool full_cycle = false;
        for (int i = 0; i < valid; i++)
            if (snap[i].param_id == 0) { full_cycle = true; break; }

        if (full_cycle) {
            int ctrl_ids[MAX_CTRL_VISITED];
            int n = _get_tcp_ctrl_ids(ctrl_ids, MAX_CTRL_VISITED);
            for (int i = 0; i < n && s_running; i++)
                _poll_tcp_control(ctrl_ids[i], &ok, &err);
        } else {
            _poll_batch_tcp(snap, valid, &ok, &err);
        }

        ESP_LOGI(TAG, "TCP poll done: ok=%d err=%d dup=%d exp=%d heap=%" PRIu32,
                 ok, err, dup, exp, kx_system_heap_free());
        free(snap);
    }
    vTaskDelete(NULL);
}

// =============================================================
// Tarea writer TCP
// =============================================================
static esp_err_t _execute_tcp_write(int control_id, int param_id, float value)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) return ESP_ERR_NOT_FOUND;

    char ip[KX_TCP_IP_LEN]; uint16_t port;
    if (kx_param_store_get_tcp_endpoint(control_id, ip, &port) != ESP_OK)
        return ESP_ERR_INVALID_STATE;

    int sock_idx = _sock_get_or_connect(ip, port);
    if (sock_idx < 0) return ESP_FAIL;

    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return ESP_ERR_INVALID_STATE;
    uint8_t unit_id = (uint8_t)ctrl->slave_addr;

    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL &&
        fc_write != MB_FC_WRITE_SINGLE_REG  &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS)
        return ESP_ERR_NOT_SUPPORTED;

    int16_t raw;
    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        float adj = value - param->addition;
        raw = (param->offset != 0.0f && param->offset != 1.0f)
              ? (int16_t)(adj / param->offset) : (int16_t)adj;
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }

    uint8_t pdu[5] = { fc_write,
        (uint8_t)((uint16_t)param->reg >> 8), (uint8_t)((uint16_t)param->reg & 0xFF),
        (uint8_t)((uint16_t)raw >> 8),        (uint8_t)((uint16_t)raw & 0xFF) };
    uint8_t resp[TCP_RESPONSE_BUF];

    ESP_LOGI(TAG, "write ctrl=%d param=%d reg=0x%04x fc=0x%02x value=%.3f→raw=%d",
             control_id, param_id, param->reg, fc_write, value, (int)(uint16_t)raw);

    int rx = -1;
    for (int a = 0; a < TCP_RETRY_COUNT && rx < 0; a++) {
        if (s_socks[sock_idx].fd < 0 && _sock_connect(sock_idx) != ESP_OK) continue;
        rx = _tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                               resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (rx < 12) { ESP_LOGW(TAG, "write no response rx=%d", rx); return ESP_FAIL; }
    ESP_LOGI(TAG, "write OK");
    return ESP_OK;
}

static void _tcp_writer_task(void *arg)
{
    kx_write_cmd_t cmd;
    ESP_LOGI(TAG, "writer task started");
    while (1) {
        if (xQueueReceive(s_write_queue, &cmd, portMAX_DELAY) != pdTRUE) continue;
        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);
        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
        esp_err_t err = _execute_tcp_write(cmd.control_id, cmd.param_id, cmd.value);
        if (err == ESP_OK) {
            kx_param_store_set_ts_set(cmd.control_id, cmd.param_id, cmd.ts);
            _enqueue(PUB_KIND_STATUS, cmd.control_id, cmd.param_id, cmd.value, 0, NULL);
        } else {
            _enqueue(PUB_KIND_ERROR, cmd.control_id, cmd.param_id,
                     0.0f, 0, "tcp_write_error");
        }
        xSemaphoreGive(s_foreach_mutex);
    }
}

// =============================================================
// Tarea report TCP
// =============================================================
static void _tcp_report_task(void *arg)
{
    ESP_LOGI(TAG, "report task started");
    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(REPORT_TASK_PERIOD_MS));
        if (!kx_param_store_is_ready() || !kx_mqtt_is_connected()) continue;

        s_report_tick_s = (s_report_tick_s + 1) % REPORT_TICK_PERIOD_S;
        int64_t tick = s_report_tick_s;

        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        int ctrl_ids[MAX_CTRL_VISITED];
        int n_ctrls = _get_tcp_ctrl_ids(ctrl_ids, MAX_CTRL_VISITED);

        int sent = 0, errs = 0;
        for (int c = 0; c < n_ctrls && s_running; c++) {
            char ip[KX_TCP_IP_LEN]; uint16_t port;
            if (kx_param_store_get_tcp_endpoint(ctrl_ids[c], ip, &port) != ESP_OK)
                continue;
            int sock_idx = _sock_get_or_connect(ip, port);
            if (sock_idx < 0) continue;

            const kx_control_t *ctrl = kx_param_store_get_ctrl(ctrl_ids[c]);
            if (!ctrl) continue;
            uint8_t unit_id = (uint8_t)ctrl->slave_addr;

            int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
            kx_packet_list_t *list = kx_pkt_build(ctrl_ids[c], false,
                                                    NULL, 0, tick, now_ms);
            if (!list) continue;

            for (int i = 0; i < list->count; i++) {
                xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);
                int pkt_err = 0;
                int pkt_ok = _dispatch_packet_tcp(sock_idx, unit_id,
                                                   &list->pkts[i],
                                                   PUB_KIND_REPORT,
                                                   &pkt_err, tick);
                xSemaphoreGive(s_foreach_mutex);
                sent += pkt_ok; errs += pkt_err;
                if (i + 1 < list->count) vTaskDelay(pdMS_TO_TICKS(10));
            }
            kx_pkt_free(list);
        }

        if (sent > 0 || errs > 0)
            ESP_LOGI(TAG, "TCP report tick=%" PRId64 " sent=%d err=%d heap=%" PRIu32,
                     tick, sent, errs, kx_system_heap_free());
    }
    vTaskDelete(NULL);
}

// =============================================================
// Tarea publisher TCP
// =============================================================
static void _tcp_publisher_task(void *arg)
{
    kx_pub_result_t r;
    ESP_LOGI(TAG, "publisher task started");
    while (1) {
        if (xQueueReceive(s_pub_queue, &r, portMAX_DELAY) == pdTRUE) {
            switch (r.kind) {
            case PUB_KIND_STATUS: kx_param_pub_status(r.control_id, r.param_id, r.value); break;
            case PUB_KIND_REPORT: kx_param_pub_report(r.control_id, r.param_id, r.value); break;
            case PUB_KIND_ERROR:  kx_param_pub_error (r.control_id, r.param_id, r.error_msg, r.reg); break;
            }
        }
    }
}

// =============================================================
// API pública
// =============================================================
esp_err_t kx_modbus_tcp_start(void)
{
    if (s_running) { ESP_LOGW(TAG, "already running"); return ESP_OK; }

    memset(s_socks, 0, sizeof(s_socks));
    for (int i = 0; i < MAX_TCP_SOCKETS; i++) s_socks[i].fd = -1;
    s_n_socks = 0;

    s_sock_mutex = xSemaphoreCreateMutex();
    if (!s_sock_mutex) return ESP_FAIL;

    s_pub_queue = xQueueCreate(PUB_QUEUE_SIZE, sizeof(kx_pub_result_t));
    if (!s_pub_queue) return ESP_FAIL;

    s_demand_queue = xQueueCreate(DEMAND_QUEUE_SIZE, sizeof(kx_poll_demand_t));
    if (!s_demand_queue) return ESP_FAIL;

    s_write_queue = xQueueCreate(WRITE_QUEUE_SIZE, sizeof(kx_write_cmd_t));
    if (!s_write_queue) return ESP_FAIL;

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) return ESP_FAIL;

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) return ESP_FAIL;

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    s_running = true;

    BaseType_t ret;
    ret = xTaskCreate(_tcp_publisher_task, "kx_tcp_pub",
                      STACK_TCP_PUB, NULL, KX_TASK_PRIO_TELEMETRY - 1, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ret = xTaskCreate(_tcp_writer_task, "kx_tcp_writer",
                      STACK_TCP_WRITER, NULL, KX_TASK_PRIO_TELEMETRY + 2, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ret = xTaskCreate(_tcp_report_task, "kx_tcp_report",
                      STACK_TCP_REPORT, NULL, KX_TASK_PRIO_TELEMETRY, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    ret = xTaskCreate(_tcp_demand_task, "kx_tcp_demand",
                      STACK_TCP_DEMAND, NULL, KX_TASK_PRIO_TELEMETRY + 1, NULL);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    // Log de RAM tras arranque para detectar presión de memoria
    ESP_LOGI(TAG,
             "started — heap_free=%" PRIu32 " bytes"
             " | pub_q=%d demand_q=%d write_q=%d"
             " | stacks: demand=%d writer=%d report=%d pub=%d",
             kx_system_heap_free(),
             PUB_QUEUE_SIZE, DEMAND_QUEUE_SIZE, WRITE_QUEUE_SIZE,
             STACK_TCP_DEMAND, STACK_TCP_WRITER, STACK_TCP_REPORT, STACK_TCP_PUB);
    return ESP_OK;
}

void kx_modbus_tcp_stop(void)  { s_running = false; }
bool kx_modbus_tcp_is_running(void) { return s_running; }

// Arranca el driver TCP solo si aún no está corriendo.
// Seguro llamarlo múltiples veces desde kx_config_handler.
esp_err_t kx_modbus_tcp_ensure_started(void)
{
    if (s_running) return ESP_OK;
    ESP_LOGI(TAG, "first TCP control detected — starting TCP driver");
    return kx_modbus_tcp_start();
}

void kx_modbus_tcp_request_poll(int param_id)
{
    if (!s_demand_queue || !s_running) return;
    if (!kx_param_store_is_ready()) return;
    if (param_id != 0 && _pending_test(param_id)) return;

    int64_t now_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
    kx_poll_demand_t d = { param_id, now_ms };
    if (xQueueSend(s_demand_queue, &d, 0) != pdTRUE) {
        ESP_LOGE(TAG, "demand_queue FULL param_id=%d", param_id); return;
    }
    if (param_id != 0) _pending_set(param_id);
    if (s_poll_eg) xEventGroupSetBits(s_poll_eg, DEMAND_BIT);
}

esp_err_t kx_modbus_tcp_enqueue_write(int    control_id,
                                       int    param_id,
                                       float  value,
                                       double ts)
{
    if (!s_write_queue) return ESP_ERR_INVALID_STATE;
    kx_write_cmd_t cmd = { control_id, param_id, value, ts };
    return (xQueueSend(s_write_queue, &cmd, pdMS_TO_TICKS(200)) == pdTRUE)
           ? ESP_OK : ESP_ERR_NO_MEM;
}

void kx_modbus_tcp_pause(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xEventGroupClearBits(s_poll_eg, POLL_ALLOWED_BIT);
    if (xSemaphoreTake(s_foreach_mutex, pdMS_TO_TICKS(60000)) != pdTRUE)
        ESP_LOGE(TAG, "pause timeout");
    else
        ESP_LOGI(TAG, "TCP paused");
}

void kx_modbus_tcp_resume(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xSemaphoreGive(s_foreach_mutex);
    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    ESP_LOGI(TAG, "TCP resumed");
}