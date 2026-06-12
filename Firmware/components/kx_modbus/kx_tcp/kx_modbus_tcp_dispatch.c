#include "kx_modbus_tcp_dispatch.h"
#include "kx_modbus_tcp_socket.h"
#include "kx_modbus_shared.h"
#include "kx_modbus_publish.h"
#include "kx_param_store.h"
#include "kx_telemetry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <float.h>

static const char *TAG = "kx_tcp_dispatch";

// =============================================================
// kx_modbus_tcp_dispatch.c — Ejecución de packets TCP
// =============================================================

int kx_dispatch_packet_tcp(QueueHandle_t      pub_queue,
                             SemaphoreHandle_t  foreach_mutex,
                             int                sock_idx,
                             uint8_t            unit_id,
                             const kx_packet_t *pkt,
                             kx_pub_kind_t      pub_kind,
                             int               *out_errors,
                             int64_t            tick_s)
{
    if (!pkt || pkt->num_slots == 0) return 0;

    int ok_count = 0, err_count = 0;

    // ── Packet individual ─────────────────────────────────────
    if (pkt->num_regs == 1) {
        const kx_pkt_slot_t *slot = &pkt->slots[0];
        if (slot->is_gap) goto done;

        const kx_param_t *param =
            kx_param_store_get_param(slot->control_id, slot->param_id);
        if (!param) { err_count++; goto done; }

        uint16_t raw   = 0;
        float    value = kx_tcp_read_register(sock_idx, unit_id,
                                               pkt->start_reg, pkt->fc,
                                               param, &raw);
        int64_t  ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (value == -FLT_MAX) {
            kx_pub_enqueue_error(pub_queue, kx_param_pub_error,
                                  slot->control_id, slot->param_id,
                                  pkt->start_reg, "tcp_timeout");
            err_count++;
        } else {
            kx_param_store_reg_upsert_read(slot->control_id, pkt->start_reg,
                pkt->fc, (uint8_t)param->function_write, value, ts_ms);
            int n = kx_publish_all_params_for_reg(
                        pub_queue, NULL, pub_kind,
                        slot->control_id, pkt->start_reg,
                        pkt->fc, raw, ts_ms, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
        goto done;
    }

    // ── Packet multi-registro ─────────────────────────────────
    {
        uint8_t resp[KX_TCP_RESPONSE_BUF];
        int rx = kx_tcp_read_multi(sock_idx, unit_id, pkt->start_reg,
                                    pkt->num_regs, pkt->fc,
                                    resp, sizeof(resp));
        int64_t ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);

        if (rx < 0) {
            ESP_LOGW(TAG, "multi FAILED — fallback unit=%u reg=0x%04x num=%d",
                     unit_id, pkt->start_reg, pkt->num_regs);
            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *sl = &pkt->slots[s];
                if (sl->is_gap || sl->param_id < 0) continue;
                const kx_param_t *p =
                    kx_param_store_get_param(sl->control_id, sl->param_id);
                if (!p) { err_count++; continue; }
                uint16_t raw = 0;
                float    val = kx_tcp_read_register(sock_idx, unit_id,
                                                     sl->reg, pkt->fc,
                                                     p, &raw);
                ts_ms = (int64_t)(esp_timer_get_time() / 1000ULL);
                if (val == -FLT_MAX) {
                    kx_pub_enqueue_error(pub_queue, kx_param_pub_error,
                                          sl->control_id, sl->param_id,
                                          sl->reg, "tcp_timeout");
                    err_count++;
                } else {
                    kx_param_store_reg_upsert_read(sl->control_id, sl->reg,
                        pkt->fc, (uint8_t)p->function_write, val, ts_ms);
                    int n = kx_publish_all_params_for_reg(
                                pub_queue, NULL, pub_kind,
                                sl->control_id, sl->reg,
                                pkt->fc, raw, ts_ms, tick_s);
                    ok_count += (n > 0) ? n : 1;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            goto done;
        }

        // resp[7]=FC, resp[8]=byte_count, resp[9..]=datos
        bool is_coil = (pkt->fc == MB_FC_READ_COILS ||
                        pkt->fc == MB_FC_READ_DISCRETE);
        int expected = is_coil ? (pkt->num_regs + 7) / 8 : pkt->num_regs * 2;

        if (rx < 9 || resp[8] != (uint8_t)expected) {
            ESP_LOGW(TAG, "multi bad byte_count=%d expected=%d",
                     (rx >= 9) ? resp[8] : -1, expected);
            err_count += pkt->num_slots;
            goto done;
        }

        for (int s = 0; s < pkt->num_slots; s++) {
            const kx_pkt_slot_t *sl = &pkt->slots[s];
            int reg_offset = (int)sl->reg - (int)pkt->start_reg;

            uint16_t raw;
            if (is_coil) {
                int bi = reg_offset / 8, bj = reg_offset % 8;
                if (9 + bi >= rx) continue;
                raw = (resp[9 + bi] >> bj) & 0x01;
            } else {
                int bi = reg_offset * 2;
                if (9 + bi + 1 >= rx) continue;
                raw = ((uint16_t)resp[9 + bi] << 8) | resp[9 + bi + 1];
            }

            if (sl->is_gap) {
                kx_param_store_reg_upsert_read(sl->control_id, sl->reg,
                    pkt->fc, 0, (float)(int16_t)raw, ts_ms);
                continue;
            }

            const kx_param_t *p =
                kx_param_store_get_param(sl->control_id, sl->param_id);
            if (!p) { err_count++; continue; }

            float v0 = (float)(int16_t)raw;
            if (p->offset != 0.0f && p->offset != 1.0f) v0 *= p->offset;
            v0 += p->addition;
            if (v0 < p->minvalue) v0 = p->minvalue;
            if (v0 > p->maxvalue) v0 = p->maxvalue;

            kx_param_store_reg_upsert_read(sl->control_id, sl->reg, pkt->fc,
                (uint8_t)p->function_write, v0, ts_ms);
            int n = kx_publish_all_params_for_reg(
                        pub_queue, NULL, pub_kind,
                        sl->control_id, sl->reg,
                        pkt->fc, raw, ts_ms, tick_s);
            ok_count += (n > 0) ? n : 1;
        }
    }

done:
    if (out_errors) *out_errors += err_count;
    return ok_count;
}