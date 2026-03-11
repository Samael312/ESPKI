#pragma once
#include "esp_err.h"
#include <stdbool.h>    
#include <stddef.h>

// =============================================================
// kx_mqtt.h  —  Cliente MQTT con reconexión automática y LWT
// =============================================================

// Callback invocado cuando llega un mensaje en un topic suscrito
typedef void (*kx_mqtt_msg_cb_t)(const char *topic, const char *payload, size_t len);

// Inicia el cliente MQTT. Debe llamarse tras kx_net_start().
esp_err_t kx_mqtt_start(kx_mqtt_msg_cb_t on_message);

// Publica un mensaje. Bloqueante (espera ACK QoS1) si qos=1.
// Devuelve ESP_OK o ESP_FAIL si la cola está llena.
esp_err_t kx_mqtt_publish(const char *topic, const char *payload, int qos, int retain);

// Suscribe a un topic
esp_err_t kx_mqtt_subscribe(const char *topic, int qos);

// True si el cliente está conectado al broker
bool kx_mqtt_is_connected(void);
