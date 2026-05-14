idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_config
        kx_telemetry
        kx_modbus
        kx_param_store
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
        lwip
)