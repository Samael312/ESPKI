idf_component_register(
    SRCS "kx_mqtt.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        mqtt
        esp_event
        esp_netif
        freertos
        esp_timer
        mbedtls    
)
