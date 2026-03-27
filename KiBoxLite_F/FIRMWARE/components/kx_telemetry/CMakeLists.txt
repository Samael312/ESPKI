idf_component_register(
    SRCS "kx_telemetry.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        esp_wifi
        esp_timer
        freertos
)
