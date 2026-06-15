idf_component_register(
    SRCS
        "kx_param_store_hash.c"
        "kx_param_store_nvs.c"
        "kx_param_store_api.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        freertos
        kx_modbus
        heap
        nvs_flash
    PRIV_REQUIRES
        json
)