#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h" 

// =============================================================
// kx_system.h  —  Estado global del dispositivo
// =============================================================

#define KX_DEVICE_ID_LEN  13   // 12 hex chars + null terminator

// Estado de conexión compartido entre tareas
typedef enum {
    KX_NET_DISCONNECTED = 0,
    KX_NET_CONNECTING,
    KX_NET_CONNECTED,
} kx_net_state_t;

typedef enum {
    KX_MQTT_DISCONNECTED = 0,
    KX_MQTT_CONNECTING,
    KX_MQTT_CONNECTED,
} kx_mqtt_state_t;

// Inicializa el subsistema: NVS, device_id, contadores
esp_err_t kx_system_init(void);

// Devuelve el device_id (MAC WiFi como string hex sin separadores)
const char *kx_system_device_id(void);

// Uptime en segundos desde arranque
uint32_t kx_system_uptime_s(void);

// Motivo del último reset como string
const char *kx_system_reset_reason(void);

// Heap libre actual en bytes
uint32_t kx_system_heap_free(void);

// Contador de reinicios (persistido en NVS)
uint32_t kx_system_boot_count(void);

// Estado de red y MQTT (leído/escrito por sus respectivas tareas)
void         kx_system_set_net_state(kx_net_state_t s);
kx_net_state_t kx_system_net_state(void);

void           kx_system_set_mqtt_state(kx_mqtt_state_t s);
kx_mqtt_state_t kx_system_mqtt_state(void);

// Contador de reconexiones MQTT
void     kx_system_inc_reconnect_count(void);
uint32_t kx_system_reconnect_count(void);
