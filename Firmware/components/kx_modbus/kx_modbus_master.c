#include "kx_modbus_master.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "kx_modbus_master";

// Pines UEXT Olimex
#define KX_MODBUS_UART_NUM   UART_NUM_2
#define KX_MODBUS_PIN_TX     4   
#define KX_MODBUS_PIN_RX     36  
#define KX_MODBUS_PIN_DE     14  // Pin 9 UEXT (Jumper SCK)
#define KX_MODBUS_PIN_RE     5   // Pin 10 UEXT (Jumper #SS)

void kx_modbus_test_read_pb1()
{
    // Trama para Emerson Adre=2, FC=03, Registro=256 (0x0100), Cantidad=1
    // El CRC es 0x85E9
    uint8_t req[] = { 0x02, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xE9 };
    uint8_t rx_buf[128];

    // Enviar (El driver maneja el pin DE automáticamente)
    uart_write_bytes(KX_MODBUS_UART_NUM, req, sizeof(req));
    
    // Leer respuesta
    int len = uart_read_bytes(KX_MODBUS_UART_NUM, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(1000));

    if (len >= 7) { // Respuesta mínima válida: Slave + FC + Bytes + 2 bytes Data + 2 bytes CRC
        int16_t raw_val = (rx_buf[3] << 8) | rx_buf[4];
        float temp = raw_val / 10.0;
        ESP_LOGI(TAG, "Lectura PB1: %.1f C", temp);
    } else {
        ESP_LOGW(TAG, "Sin respuesta del Emerson (Check cableado A/B y Jumpers)");
    }
}


esp_err_t kx_modbus_init(void)
{
    // 1. Configurar la UART con Paridad PAR (Requerido por Emerson/Dixell)
    uart_config_t cfg = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,  // IMPORTANTE: Emerson usa Paridad Par
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_is_driver_installed(KX_MODBUS_UART_NUM)) return ESP_OK;

    ESP_ERROR_CHECK(uart_param_config(KX_MODBUS_UART_NUM, &cfg));
    
    // 2. Configurar pines. Nota: RX es el 36 (Input Only en ESP32)
    ESP_ERROR_CHECK(uart_set_pin(KX_MODBUS_UART_NUM, KX_MODBUS_PIN_TX, KX_MODBUS_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // 3. Instalar driver
    ESP_ERROR_CHECK(uart_driver_install(KX_MODBUS_UART_NUM, 512, 0, 0, NULL, 0));

    // 4. MODO RS485 NATIVO: El ESP32 controlará el pin DE automáticamente
    ESP_ERROR_CHECK(uart_set_mode(KX_MODBUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
    
    // Configuramos el pin DE para que la UART lo maneje
    // En MOD-RS485, DE y RE suelen unirse o controlarse juntos para Half-Duplex
    gpio_set_direction(KX_MODBUS_PIN_RE, GPIO_MODE_OUTPUT);
    gpio_set_level(KX_MODBUS_PIN_RE, 0); // Dejar RE siempre bajo (escucha activa) si el hardware lo permite

    ESP_LOGI(TAG, "Modbus iniciado (9600, 8E1) en UEXT");
    kx_modbus_test_read_pb1(); // Test de lectura inicial
    return ESP_OK;
}

