idf_component_register(
    SRCS "kx_dummy_protocol.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_telemetry
        kx_param_store
        freertos
        esp_timer
)