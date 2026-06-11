idf_component_register(
    SRCS "kx_modbus_tcp.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_param_store
        kx_telemetry
        kx_modbus
        freertos
        esp_timer
        lwip
)