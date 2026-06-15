#include "kx_modbus_tcp_socket.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/errno.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "kx_tcp_sock";

// =============================================================
// kx_modbus_tcp_socket.c — Gestión de sockets y transacciones
// =============================================================

static kx_tcp_sock_t s_socks[KX_MAX_TCP_SOCKETS];
static int           s_n_socks = 0;
static SemaphoreHandle_t s_sock_mutex = NULL;

// =============================================================
// kx_tcp_socket_init
// =============================================================
esp_err_t kx_tcp_socket_init(void)
{
    memset(s_socks, 0, sizeof(s_socks));
    for (int i = 0; i < KX_MAX_TCP_SOCKETS; i++)
        s_socks[i].fd = -1;
    s_n_socks = 0;

    s_sock_mutex = xSemaphoreCreateMutex();
    if (!s_sock_mutex) {
        ESP_LOGE(TAG, "socket mutex alloc failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// =============================================================
// Búsqueda y alloc de socket
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
    if (s_n_socks >= KX_MAX_TCP_SOCKETS) {
        ESP_LOGE(TAG, "sock table full (%d)", KX_MAX_TCP_SOCKETS);
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

void kx_tcp_sock_set_recv_timeout(int sock_idx, int timeout_ms)
{
    if (sock_idx < 0 || sock_idx >= s_n_socks) return;
    if (s_socks[sock_idx].fd < 0) return;
    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000,
    };
    setsockopt(s_socks[sock_idx].fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

static esp_err_t _sock_connect(int idx)
{
    _sock_close(idx);

    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0) { ESP_LOGE(TAG, "socket() failed: %d", errno); return ESP_FAIL; }

    struct timeval tv = {
        .tv_sec  = TCP_RECV_TIMEOUT_READ_MS / 1000,
        .tv_usec = (TCP_RECV_TIMEOUT_READ_MS % 1000) * 1000,
    };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

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
        .tv_sec  = KX_TCP_CONNECT_TIMEOUT_MS / 1000,
        .tv_usec = (KX_TCP_CONNECT_TIMEOUT_MS % 1000) * 1000,
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

// =============================================================
// kx_tcp_sock_get_or_connect
// =============================================================
int kx_tcp_sock_get_or_connect(const char *ip, uint16_t port)
{
    xSemaphoreTake(s_sock_mutex, portMAX_DELAY);

    int idx = _sock_find(ip, port);
    if (idx < 0) idx = _sock_alloc(ip, port);
    if (idx < 0) { xSemaphoreGive(s_sock_mutex); return -1; }

    if (s_socks[idx].fd < 0) {
        for (int t = 0; t < KX_TCP_MAX_RECONNECT_TRIES; t++) {
            if (_sock_connect(idx) == ESP_OK) break;
            if (t + 1 < KX_TCP_MAX_RECONNECT_TRIES)
                vTaskDelay(pdMS_TO_TICKS(KX_TCP_RECONNECT_DELAY_MS));
        }
    }

    int result = (s_socks[idx].fd >= 0) ? idx : -1;
    xSemaphoreGive(s_sock_mutex);
    return result;
}

// =============================================================
// kx_tcp_transaction
// =============================================================
int kx_tcp_transaction(int            sock_idx,
                        uint8_t        unit_id,
                        const uint8_t *pdu,
                        size_t         pdu_len,
                        uint8_t       *resp,
                        size_t         resp_max)
{
    xSemaphoreTake(s_sock_mutex, portMAX_DELAY);

    int fd = s_socks[sock_idx].fd;
    if (fd < 0) { xSemaphoreGive(s_sock_mutex); return KX_TCP_RX_NET_ERROR; }

    uint16_t tid = s_socks[sock_idx].next_tid++;
    if (s_socks[sock_idx].next_tid == 0)
        s_socks[sock_idx].next_tid = 1;

    uint16_t mbap_len = (uint16_t)(1 + pdu_len);
    uint8_t  tx[KX_MBAP_PDU_OFFSET + 260];

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
        return KX_TCP_RX_NET_ERROR;
    }

    int rx = recv(fd, resp, resp_max, 0);
    if (rx <= 0) {
        ESP_LOGW(TAG, "recv failed fd=%d err=%d", fd, errno);
        _sock_close(sock_idx);
        xSemaphoreGive(s_sock_mutex);
        return KX_TCP_RX_NET_ERROR;
    }

    xSemaphoreGive(s_sock_mutex);

    if (rx < 8) { ESP_LOGW(TAG, "response too short: %d", rx); return KX_TCP_RX_NET_ERROR; }

    uint16_t resp_tid = ((uint16_t)resp[0] << 8) | resp[1];
    if (resp_tid != tid) {
        ESP_LOGW(TAG, "TID mismatch: sent=%u got=%u", tid, resp_tid);
        return KX_TCP_RX_NET_ERROR;
    }
    if (resp[6] != unit_id) {
        ESP_LOGW(TAG, "unit_id mismatch: sent=%u got=%u", unit_id, resp[6]);
        return KX_TCP_RX_NET_ERROR;
    }
    if (resp[7] & 0x80) {
        uint8_t exc = (rx > 8) ? resp[8] : 0;
        ESP_LOGW(TAG, "Modbus exception fc=0x%02x exc=0x%02x", resp[7], exc);
        // Respuesta válida del esclavo — no es fallo de red.
        // Se distingue de KX_TCP_RX_NET_ERROR para no reintentar
        // ni contabilizar como error de comunicación en el dispatch.
        return KX_TCP_RX_MODBUS_EXCEPT;
    }
    return rx;
}

// =============================================================
// kx_tcp_read_register
// =============================================================
float kx_tcp_read_register(int               sock_idx,
                             uint8_t           unit_id,
                             uint16_t          reg,
                             uint8_t           fc,
                             const kx_param_t *param,
                             uint16_t         *raw_out,
                             int              *out_rx_code)
{
    uint8_t pdu[5] = { fc,
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        0x00, 0x01 };
    uint8_t resp[KX_TCP_RESPONSE_BUF];

    // Solo reintenta en fallo de red (KX_TCP_RX_NET_ERROR).
    // Una excepción Modbus (KX_TCP_RX_MODBUS_EXCEPT) es definitiva:
    // reintentar no cambia el resultado y genera tráfico innecesario.
    int rx = KX_TCP_RX_NET_ERROR;
    for (int a = 0; a < KX_TCP_RETRY_COUNT && rx == KX_TCP_RX_NET_ERROR; a++) {
        if (s_socks[sock_idx].fd < 0 &&
            _sock_connect(sock_idx) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        rx = kx_tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                                 resp, sizeof(resp));
        if (rx == KX_TCP_RX_NET_ERROR) vTaskDelay(pdMS_TO_TICKS(50));
    }

    if (out_rx_code) *out_rx_code = rx;

    if (rx < 0) {
        if (raw_out) *raw_out = 0;
        return -FLT_MAX;
    }

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

// =============================================================
// kx_tcp_read_multi
// =============================================================
int kx_tcp_read_multi(int      sock_idx,
                       uint8_t  unit_id,
                       uint16_t start_reg,
                       uint16_t num_regs,
                       uint8_t  fc,
                       uint8_t *resp_buf,
                       size_t   resp_max)
{
    uint8_t pdu[5] = { fc,
        (uint8_t)(start_reg >> 8), (uint8_t)(start_reg & 0xFF),
        (uint8_t)(num_regs  >> 8), (uint8_t)(num_regs  & 0xFF) };

    // Solo reintenta en fallo de red. Una excepción Modbus es definitiva.
    int rx = KX_TCP_RX_NET_ERROR;
    for (int a = 0; a < KX_TCP_RETRY_COUNT && rx == KX_TCP_RX_NET_ERROR; a++) {
        if (s_socks[sock_idx].fd < 0 &&
            _sock_connect(sock_idx) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        rx = kx_tcp_transaction(sock_idx, unit_id, pdu, sizeof(pdu),
                                 resp_buf, resp_max);
        if (rx == KX_TCP_RX_NET_ERROR) vTaskDelay(pdMS_TO_TICKS(50));
    }
    return rx;
}