idf_component_register(
    SRCS "kx_config_handler.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        freertos
    PRIV_REQUIRES
        json
)