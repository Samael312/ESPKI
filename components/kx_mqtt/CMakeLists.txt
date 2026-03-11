idf_component_register(
    SRCS "kx_mqtt.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        mqtt
        esp_event
        freertos
        esp_timer
)
