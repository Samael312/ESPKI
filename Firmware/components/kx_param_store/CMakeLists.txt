idf_component_register(
    SRCS "kx_param_store.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        freertos
	    kx_modbus_tcp
        heap
        nvs_flash
    PRIV_REQUIRES
        json
)
