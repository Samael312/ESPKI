idf_component_register(
    SRCS "kx_telemetry.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_param_store
        kx_modbus
        kx_modbus_tcp
        esp_wifi
        esp_timer
        freertos
    PRIV_REQUIRES
        json
)