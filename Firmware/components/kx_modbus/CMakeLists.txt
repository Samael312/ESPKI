idf_component_register(
    SRCS
        # ── kx_shared ──────────────────────────────────────────
        "kx_shared/kx_modbus_packetizer.c"
        "kx_shared/kx_modbus_publish.c"

        # ── kx_rtu ─────────────────────────────────────────────
        "kx_rtu/kx_modbus_master.c"
        "kx_rtu/kx_modbus_pipeline.c"
        "kx_rtu/kx_modbus_report.c"
        "kx_rtu/kx_modbus_writer.c"
        "kx_rtu/kx_modbus_dispatch.c"
        "kx_rtu/kx_modbus_uart.c"
        # kx_rtu/kx_modbus_legacy.c — deshabilitado (#if 0), no incluir

        # ── kx_tcp ─────────────────────────────────────────────
        "kx_tcp/kx_modbus_tcp.c"
        "kx_tcp/kx_modbus_tcp_pipeline.c"
        "kx_tcp/kx_modbus_tcp_report.c"
        "kx_tcp/kx_modbus_tcp_writer.c"
        "kx_tcp/kx_modbus_tcp_dispatch.c"
        "kx_tcp/kx_modbus_tcp_socket.c"

    INCLUDE_DIRS
        "."
        "kx_shared"
        "kx_rtu"
        "kx_tcp"

    REQUIRES
        kx_system
        kx_mqtt
        kx_param_store
        kx_telemetry
        driver
        freertos
        esp_timer
        lwip
        json
)