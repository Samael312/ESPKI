idf_component_register(
    SRCS "kx_modbus_master.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_telemetry
        kx_param_store
        driver
        freertos
        esp_timer
)