#include "kx_modbus_dispatch.h"
#include "kx_modbus_uart.h"
#include "kx_modbus_shared.h"
#include "kx_modbus_publish.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <float.h>
#include <inttypes.h>

static const char *TAG = "kx_dispatch";

// =============================================================
// kx_modbus_dispatch.c — Ejecución de packets RTU y fan-out
// =============================================================

// =============================================================
// kx_dispatch_packet
//
// tick_s == -1  → modo demand
// tick_s >= 0   → modo report
// =============================================================
int kx_dispatch_packet(QueueHandle_t         pub_queue,
                        SemaphoreHandle_t     foreach_mutex,
                        const kx_packet_t    *pkt,
                        kx_pub_fn_t           pub_fn,
                        int                  *out_errors,
                        int64_t               tick_s)
{
    if (!pkt || pkt->num_slots == 0) return 0;

    int ok_count  = 0;
    int err_count = 0;

    // ── Caso 1: packet individual (num_regs == 1) ─────────────
    if (pkt->num_regs == 1) {
        const kx_pkt_slot_t *slot = &pkt->slots[0];
        if (slot->is_gap) goto dispatch_done;

        const kx_param_t *param =
            kx_param_store_get_param(slot->control_id, slot->param_id);
        if (!param) {
            ESP_LOGW(TAG, "dispatch individual: param not found ctrl=%d p=%d",
                     slot->control_id, slot->param_id);
            err_count++;
            goto dispatch_done;
        }

        uint16_t raw     = 0;
        int      rx_code = 0;
        float    value   = kx_modbus_read_reg(pkt->slave_addr, pkt->start_reg,
                                               pkt->fc, param, &raw, &rx_code);
        int64_t  ts_ms   = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (value == -FLT_MAX) {
            if (rx_code == KX_RTU_RX_MODBUS_EXCEPT) {
                // Excepción Modbus limpia: registro no legible en este esclavo.
                // No es fallo de bus — no incrementar err_count.
                ESP_LOGD(TAG, "Modbus exc (single) param_id=%d reg=0x%04x — skipped",
                         slot->param_id, pkt->start_reg);
            } else {
                // Fallo de bus real (timeout, CRC, etc.)
                kx_pub_enqueue_error(pub_queue, kx_param_pub_error,
                                     slot->control_id, slot->param_id,
                                     pkt->start_reg, "modbus_timeout");
                err_count++;
            }
        } else {
            kx_param_store_reg_upsert_read(
                slot->control_id, pkt->start_reg, pkt->fc,
                (uint8_t)param->function_write, value, ts_ms);

            int n = kx_publish_all_params_for_reg(
                        pub_queue, pub_fn, PUB_KIND_STATUS,
                        slot->control_id, pkt->start_reg,
                        pkt->fc, raw, ts_ms, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
        goto dispatch_done;
    }

    // ── Caso 2: packet multi-registro ─────────────────────────
    {
        uint8_t resp[KX_PKT_MAX_REGS_PER_PKT * 2 + 8];
        int rx = kx_modbus_read_regs_multi(pkt->slave_addr, pkt->start_reg,
                                            pkt->num_regs, pkt->fc,
                                            resp, sizeof(resp));
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        // Fallback individual si el multi falla (red o excepción Modbus)
        if (rx < 0) {
            if (rx == KX_RTU_RX_MODBUS_EXCEPT) {
                // El bloque entero recibió excepción Modbus.
                // Intentar slot a slot por si algún registro individual
                // sí es legible (bloques parcialmente válidos).
                ESP_LOGD(TAG, "Modbus exc (multi) reg=0x%04x num=%d — fallback slot-by-slot",
                         pkt->start_reg, pkt->num_regs);
            }
            // Para KX_RTU_RX_NET_ERROR también hacemos fallback individual
            // por si el esclavo recupera la respuesta en petición unitaria.

            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *slot = &pkt->slots[s];
                if (slot->is_gap || slot->param_id < 0) continue;
                const kx_param_t *param =
                    kx_param_store_get_param(slot->control_id, slot->param_id);
                if (!param) { err_count++; continue; }

                uint16_t raw     = 0;
                int      rx_code = 0;
                float    value   = kx_modbus_read_reg(pkt->slave_addr, slot->reg,
                                                       pkt->fc, param, &raw, &rx_code);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

                if (value == -FLT_MAX) {
                    if (rx_code == KX_RTU_RX_MODBUS_EXCEPT) {
                        // Registro individual no legible — respuesta válida del esclavo.
                        ESP_LOGD(TAG, "Modbus exc (fallback) param_id=%d reg=0x%04x — skipped",
                                 slot->param_id, slot->reg);
                    } else {
                        // Fallo de bus real.
                        kx_pub_enqueue_error(pub_queue, kx_param_pub_error,
                                             slot->control_id, slot->param_id,
                                             slot->reg, "modbus_timeout");
                        err_count++;
                    }
                } else {
                    kx_param_store_reg_upsert_read(
                        slot->control_id, slot->reg, pkt->fc,
                        (uint8_t)param->function_write, value, ts_ms);
                    int n = kx_publish_all_params_for_reg(
                                pub_queue, pub_fn, PUB_KIND_STATUS,
                                slot->control_id, slot->reg,
                                pkt->fc, raw, ts_ms, tick_s);
                    ok_count += (n > 0) ? n : 1;
                }
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
            }
            goto dispatch_done;
        }

        // Verificar byte_count
        bool is_coil = (pkt->fc == MB_FC_READ_COILS ||
                        pkt->fc == MB_FC_READ_DISCRETE);
        int  expected_bytes = is_coil ? (pkt->num_regs + 7) / 8
                                      : pkt->num_regs * 2;

        if (rx < 3 || resp[2] != (uint8_t)expected_bytes) {
            ESP_LOGW(TAG,
                     "multi bad byte_count=%d expected=%d rx=%d "
                     "(slave=%d reg=0x%04x num=%d) — fallback",
                     (rx >= 3) ? resp[2] : -1, expected_bytes, rx,
                     pkt->slave_addr, pkt->start_reg, pkt->num_regs);

            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *slot = &pkt->slots[s];
                if (slot->is_gap || slot->param_id < 0) continue;
                const kx_param_t *param =
                    kx_param_store_get_param(slot->control_id, slot->param_id);
                if (!param) { err_count++; continue; }

                uint16_t raw     = 0;
                int      rx_code = 0;
                float    value   = kx_modbus_read_reg(pkt->slave_addr, slot->reg,
                                                       pkt->fc, param, &raw, &rx_code);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

                if (value == -FLT_MAX) {
                    if (rx_code == KX_RTU_RX_MODBUS_EXCEPT) {
                        ESP_LOGD(TAG, "Modbus exc (bad_bc fallback) param_id=%d reg=0x%04x — skipped",
                                 slot->param_id, slot->reg);
                    } else {
                        kx_pub_enqueue_error(pub_queue, kx_param_pub_error,
                                             slot->control_id, slot->param_id,
                                             slot->reg, "modbus_timeout");
                        err_count++;
                    }
                } else {
                    kx_param_store_reg_upsert_read(
                        slot->control_id, slot->reg, pkt->fc,
                        (uint8_t)param->function_write, value, ts_ms);
                    int n = kx_publish_all_params_for_reg(
                                pub_queue, pub_fn, PUB_KIND_STATUS,
                                slot->control_id, slot->reg,
                                pkt->fc, raw, ts_ms, tick_s);
                    ok_count += (n > 0) ? n : 1;
                }
                vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
            }
            goto dispatch_done;
        }

        // Decodificar slots
        for (int s = 0; s < pkt->num_slots; s++) {
            const kx_pkt_slot_t *slot = &pkt->slots[s];
            int reg_offset = (int)slot->reg - (int)pkt->start_reg;

            uint16_t raw;
            if (is_coil) {
                int byte_idx = reg_offset / 8;
                int bit_idx  = reg_offset % 8;
                if (3 + byte_idx >= rx) continue;
                raw = (resp[3 + byte_idx] >> bit_idx) & 0x01;
            } else {
                int byte_idx = reg_offset * 2;
                if (3 + byte_idx + 1 >= rx) continue;
                raw = ((uint16_t)resp[3 + byte_idx] << 8) |
                                 resp[3 + byte_idx + 1];
            }

            if (slot->is_gap) {
                kx_param_store_reg_upsert_read(
                    slot->control_id, slot->reg, pkt->fc,
                    0, (float)(int16_t)raw, ts_ms);
                continue;
            }

            const kx_param_t *param =
                kx_param_store_get_param(slot->control_id, slot->param_id);
            if (!param) {
                ESP_LOGW(TAG, "multi slot[%d]: param not found p=%d",
                         s, slot->param_id);
                err_count++;
                continue;
            }

            float value_first = (float)(int16_t)raw;
            if (param->offset != 0.0f && param->offset != 1.0f)
                value_first *= param->offset;
            value_first += param->addition;
            if (value_first < param->minvalue) value_first = param->minvalue;
            if (value_first > param->maxvalue) value_first = param->maxvalue;

            kx_param_store_reg_upsert_read(
                slot->control_id, slot->reg, pkt->fc,
                (uint8_t)param->function_write, value_first, ts_ms);

            int n = kx_publish_all_params_for_reg(
                        pub_queue, pub_fn, PUB_KIND_STATUS,
                        slot->control_id, slot->reg,
                        pkt->fc, raw, ts_ms, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
    }

dispatch_done:
    if (out_errors) *out_errors += err_count;
    return ok_count;
}

// =============================================================
// kx_dispatch_control_packets
// =============================================================
void kx_dispatch_control_packets(QueueHandle_t      pub_queue,
                                  SemaphoreHandle_t  foreach_mutex,
                                  kx_packet_list_t  *list,
                                  kx_pub_fn_t        pub_fn,
                                  int               *out_ok,
                                  int               *out_errors,
                                  int64_t            tick_s)
{
    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];

        xSemaphoreTake(foreach_mutex, portMAX_DELAY);
        int pkt_errors = 0;
        int pkt_ok = kx_dispatch_packet(pub_queue, foreach_mutex,
                                         pkt, pub_fn, &pkt_errors, tick_s);
        xSemaphoreGive(foreach_mutex);

        if (out_ok)     *out_ok     += pkt_ok;
        if (out_errors) *out_errors += pkt_errors;

        if (i + 1 < list->count)
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
    }
}