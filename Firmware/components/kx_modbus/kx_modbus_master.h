#pragma once
#include "esp_err.h"

// =============================================================
// kx_modbus_master.h — Driver Modbus RTU Master (Fase 1)
// Hardware: MAX13487E auto-direction RS-485
// =============================================================

/**
 * @brief Inicializa el periférico UART configurado para Modbus RTU.
 * Se asume el uso de un transceiver con auto-dirección (ej. MAX13487E).
 * * @return ESP_OK si la inicialización es exitosa, error en caso contrario.
 */
esp_err_t kx_modbus_init(void);

/**
 * @brief Arranca la tarea de prueba que envía una trama cruda (FC03)
 * y escucha la respuesta en el bus periódicamente.
 * * @return ESP_OK si la tarea se crea correctamente, error en caso contrario.
 */
esp_err_t kx_modbus_start(void);