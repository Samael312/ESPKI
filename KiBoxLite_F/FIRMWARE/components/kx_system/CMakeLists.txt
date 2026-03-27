idf_component_register(
    SRCS "kx_system.c"
    INCLUDE_DIRS "."
    REQUIRES
        nvs_flash
        esp_system
        esp_wifi
        driver
        freertos
)
