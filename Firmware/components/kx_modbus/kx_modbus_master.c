#include "kx_modbus_master.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "KX_MODBUS";

#define UART_NUM    UART_NUM_2
#define PIN_TX      4    // GPIO4 → DI
#define PIN_RX      36   // GPIO36 → RO
// RE/ a GND físicamente — receptor siempre habilitado
// SHDN/ a +5V físicamente — chip siempre activo

#define XC660D_ADDR     0x02
#define REG_P1_TEMP     0x0100

static int cycle = 0;

static uint16_t crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static void modbus_task(void *arg)
{
    uint8_t req[8];
    uint8_t resp[64];

    req[0] = XC660D_ADDR;
    req[1] = 0x03;
    req[2] = (REG_P1_TEMP >> 8) & 0xFF;
    req[3] =  REG_P1_TEMP       & 0xFF;
    req[4] = 0x00;
    req[5] = 0x01;
    uint16_t c = crc16(req, 6);
    req[6] = c & 0xFF;
    req[7] = (c >> 8) & 0xFF;

    ESP_LOGI(TAG, "Iniciando Modbus — addr=0x%02X reg=0x%04X",
             XC660D_ADDR, REG_P1_TEMP);

    while (1) {
        ESP_LOGI(TAG, "Ciclo %d", ++cycle);

        // Limpia buffer RX antes de transmitir
        uart_flush_input(UART_NUM);

        // Transmite — MAX13487 auto-direction, RE/ en GND
        uart_write_bytes(UART_NUM, (const char *)req, sizeof(req));
        uart_wait_tx_done(UART_NUM, pdMS_TO_TICKS(100));

        uint8_t eco[8];
        int eco_len = uart_read_bytes(UART_NUM, eco, 8, pdMS_TO_TICKS(20));
        ESP_LOGW(TAG, "Eco (%d bytes):", eco_len);
        if (eco_len > 0) ESP_LOG_BUFFER_HEX(TAG, eco, eco_len);

        // Espera respuesta del esclavo
        int len = uart_read_bytes(UART_NUM, resp, sizeof(resp), pdMS_TO_TICKS(500));

        if (len >= 7) {
            uint16_t crc_recv = resp[len-2] | (resp[len-1] << 8);
            uint16_t crc_calc = crc16(resp, len - 2);

            if (crc_recv != crc_calc) {
                ESP_LOGW(TAG, "CRC error: calc=0x%04X recv=0x%04X", crc_calc, crc_recv);
                goto next;
            }

            int16_t raw = (int16_t)((resp[3] << 8) | resp[4]);
            float temp  = raw / 10.0f;
            ESP_LOGI(TAG, "Temperatura P1 = %.1f °C (raw=%d)", temp, raw);

        } else if (len > 0) {
            ESP_LOGW(TAG, "Respuesta incompleta (%d bytes):", len);
            ESP_LOG_BUFFER_HEX(TAG, resp, len);
        } else {
            ESP_LOGW(TAG, "Sin respuesta — timeout");
        }

next:
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

esp_err_t kx_modbus_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM, 256, 256, 0, NULL, 0);
    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "UART2 TX=GPIO%d RX=GPIO%d baud=9600", PIN_TX, PIN_RX);

    xTaskCreate(modbus_task, "mb_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}