#include "kx_modbus_uart.h"
#include "../../../main/kx_config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <float.h>

static const char *TAG = "kx_modbus_uart";

// =============================================================
// kx_modbus_uart.c — Capa de transporte Modbus RTU
//
// Sin cambios respecto a la versión original.
// =============================================================

#ifndef KX_MODBUS_UART_NUM
#define KX_MODBUS_UART_NUM   UART_NUM_1
#endif
#ifndef KX_MODBUS_BAUD
#define KX_MODBUS_BAUD       9600
#endif
#ifndef KX_MODBUS_PIN_TX
#define KX_MODBUS_PIN_TX     GPIO_NUM_4
#endif
#ifndef KX_MODBUS_PIN_RX
#define KX_MODBUS_PIN_RX     GPIO_NUM_36
#endif
#ifndef KX_MODBUS_RTS_PIN
#define KX_MODBUS_RTS_PIN    UART_PIN_NO_CHANGE
#endif

// =============================================================
// CRC16 Modbus
// =============================================================
static uint16_t _crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

// =============================================================
// kx_modbus_uart_init
// =============================================================
esp_err_t kx_modbus_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = KX_MODBUS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_param_config(KX_MODBUS_UART_NUM, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(KX_MODBUS_UART_NUM,
                       KX_MODBUS_PIN_TX, KX_MODBUS_PIN_RX,
                       KX_MODBUS_RTS_PIN, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(KX_MODBUS_UART_NUM, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) return err;

    if (KX_MODBUS_RTS_PIN != UART_PIN_NO_CHANGE) {
        err = uart_set_mode(KX_MODBUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
        if (err != ESP_OK)
            ESP_LOGW(TAG, "RS485 mode failed: %s", esp_err_to_name(err));
    }

    ESP_LOGI(TAG, "UART%d ready: baud=%d TX=%d RX=%d RTS=%d",
             KX_MODBUS_UART_NUM, KX_MODBUS_BAUD,
             KX_MODBUS_PIN_TX, KX_MODBUS_PIN_RX, KX_MODBUS_RTS_PIN);
    return ESP_OK;
}

// =============================================================
// kx_modbus_uart_deinit
// =============================================================
void kx_modbus_uart_deinit(void)
{
    uart_driver_delete(KX_MODBUS_UART_NUM);
}

// =============================================================
// kx_modbus_transaction
// =============================================================
int kx_modbus_transaction(const uint8_t *frame, size_t frame_len,
                           uint8_t *resp, size_t resp_max)
{
    uint8_t tx[frame_len + 2];
    memcpy(tx, frame, frame_len);
    uint16_t crc = _crc16(frame, frame_len);
    tx[frame_len]     = (uint8_t)(crc & 0xFF);
    tx[frame_len + 1] = (uint8_t)(crc >> 8);

    uart_flush_input(KX_MODBUS_UART_NUM);
    uart_write_bytes(KX_MODBUS_UART_NUM, (const char *)tx, frame_len + 2);

    int rx_len = uart_read_bytes(KX_MODBUS_UART_NUM, resp, resp_max,
                                  pdMS_TO_TICKS(MODBUS_RESPONSE_TIMEOUT_MS));
    if (rx_len <= 0 || rx_len < 4) return -1;

    uint16_t rx_crc   = ((uint16_t)resp[rx_len - 1] << 8) | resp[rx_len - 2];
    uint16_t calc_crc = _crc16(resp, rx_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC error: got %04x expected %04x", rx_crc, calc_crc);
        return -1;
    }
    if (resp[1] & 0x80) {
        uint8_t exc = (rx_len > 2) ? resp[2] : 0;
        ESP_LOGW(TAG, "Modbus exception: slave=%d fc=0x%02x exc=0x%02x "
                 "(req_fc=0x%02x reg=0x%02x%02x cnt=0x%02x%02x)",
                 frame[0], resp[1], exc,
                 frame[1], frame[2], frame[3], frame[4], frame[5]);
        return -1;
    }
    return rx_len;
}

// =============================================================
// kx_modbus_read_reg
// =============================================================
float kx_modbus_read_reg(uint8_t slave_addr, uint16_t reg_addr,
                          uint8_t fc, const kx_param_t *param,
                          uint16_t *out_raw)
{
    uint8_t frame[6] = {
        slave_addr, fc,
        (uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF),
        0x00, 0x01,
    };
    uint8_t resp[16];
    int rx = -1;
    for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
        rx = kx_modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }

    if (rx < 0 || rx < 4 || resp[2] == 0) {
        if (out_raw) *out_raw = 0;
        return -FLT_MAX;
    }

    uint16_t raw;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE) {
        raw = resp[3] & 0x01;
    } else {
        if (rx < 5 || resp[2] < 2) {
            if (out_raw) *out_raw = 0;
            return -FLT_MAX;
        }
        raw = ((uint16_t)resp[3] << 8) | resp[4];
    }
    if (out_raw) *out_raw = raw;

    float value = (float)(int16_t)raw;
    if (param->offset != 0.0f && param->offset != 1.0f) value *= param->offset;
    value += param->addition;
    if (value < param->minvalue) value = param->minvalue;
    if (value > param->maxvalue) value = param->maxvalue;
    return value;
}

// =============================================================
// kx_modbus_read_regs_multi
// =============================================================
int kx_modbus_read_regs_multi(uint8_t slave_addr, uint16_t start_reg,
                               uint16_t num_regs, uint8_t fc,
                               uint8_t *resp_buf, size_t resp_max)
{
    uint8_t frame[6] = {
        slave_addr, fc,
        (uint8_t)(start_reg >> 8), (uint8_t)(start_reg & 0xFF),
        (uint8_t)(num_regs  >> 8), (uint8_t)(num_regs  & 0xFF),
    };
    int rx = -1;
    for (int a = 0; a < MODBUS_RETRY_COUNT && rx < 0; a++) {
        rx = kx_modbus_transaction(frame, sizeof(frame), resp_buf, resp_max);
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }
    return rx;
}