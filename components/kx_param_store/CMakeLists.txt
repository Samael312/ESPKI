idf_component_register(
    SRCS "kx_param_store.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        freertos
        heap
    PRIV_REQUIRES
        json
)