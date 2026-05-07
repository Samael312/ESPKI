#include "kx_modbus_master.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MB_FINAL_DEBUG";

#define KX_MODBUS_UART_NUM   UART_NUM_2
#define KX_MODBUS_PIN_TX     4   // Pin 9 UEXT
#define KX_MODBUS_PIN_RX     13  // Pin 3 UEXT
#define KX_MODBUS_PIN_DE     2   // Pin 6 UEXT (LED de la placa)

static void modbus_task(void *pvParameters) {
    uint8_t req[] = { 0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xF6 };
    uint8_t rx_buf[128];

    while (1) {
        ESP_LOGI(TAG, "Enviando... (El LED debería encenderse)");

        // 1. MODO TRANSMISIÓN (GPIO 2 = HIGH)
        gpio_set_level(KX_MODBUS_PIN_DE, 1); 
        vTaskDelay(pdMS_TO_TICKS(10));

        uart_write_bytes(KX_MODBUS_UART_NUM, (const char*)req, sizeof(req));
        uart_wait_tx_done(KX_MODBUS_UART_NUM, pdMS_TO_TICKS(100));

        // 2. MODO RECEPCIÓN (GPIO 2 = LOW)
        gpio_set_level(KX_MODBUS_PIN_DE, 0); 
        vTaskDelay(pdMS_TO_TICKS(10));

        // 3. LEER
        int len = uart_read_bytes(KX_MODBUS_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));

        if (len > 0) {
            ESP_LOGI(TAG, "¡RECIBIDO! ->");
            ESP_LOG_BUFFER_HEX(TAG, rx_buf, len);
        } else {
            ESP_LOGW(TAG, "Nada por aquí... Revisa si el LED parpadeó.");
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

esp_err_t kx_modbus_init(void) {
    uart_config_t cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Configurar GPIO 2 (LED) como salida
    gpio_reset_pin(KX_MODBUS_PIN_DE);
    gpio_set_direction(KX_MODBUS_PIN_DE, GPIO_MODE_OUTPUT);
    
    // Instalar UART
    uart_driver_install(KX_MODBUS_UART_NUM, 512, 512, 0, NULL, 0);
    uart_param_config(KX_MODBUS_UART_NUM, &cfg);
    uart_set_pin(KX_MODBUS_UART_NUM, KX_MODBUS_PIN_TX, KX_MODBUS_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    xTaskCreate(modbus_task, "mb_task", 4096, NULL, 5, NULL);
    return ESP_OK;
}