idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        kx_system
        kx_mqtt
        kx_config
        kx_telemetry
        kx_dummy_protocol
        nvs_flash
        esp_wifi
        esp_netif
        esp_event
)