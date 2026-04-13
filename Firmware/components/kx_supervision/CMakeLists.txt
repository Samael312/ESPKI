idf_component_register(
    SRCS "kx_supervision.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        esp_system
        freertos
)