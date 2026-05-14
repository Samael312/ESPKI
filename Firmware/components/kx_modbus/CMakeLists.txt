idf_component_register(
    SRCS "kx_modbus_master.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_param_store
        kx_telemetry
        driver
        freertos
        esp_timer
        json
)