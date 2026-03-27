idf_component_register(
    SRCS "kx_config_handler.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_param_store
        freertos
        esp_timer
    PRIV_REQUIRES
        json
)