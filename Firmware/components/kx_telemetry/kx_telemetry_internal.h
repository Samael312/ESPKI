#pragma once
#include <sys/time.h>

// =============================================================
// kx_telemetry_internal.h — Helpers compartidos entre los
//                            archivos del componente kx_telemetry
// =============================================================

static inline double kx_telemetry_ts(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}