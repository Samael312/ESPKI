#include "kx_modbus_publish.h"
#include "kx_modbus_shared.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <float.h>

static const char *TAG = "kx_pub";

// =============================================================
// kx_modbus_publish.c — Fan-out de publicaciones compartido
//                       entre driver RTU y driver TCP
// =============================================================

// =============================================================
// kx_pub_enqueue
//
// Intenta encolar un resultado en la cola dada.
// Si la cola está llena aplica backpressure con timeout.
// Devuelve true si se encoló, false si se descartó.
// =============================================================
bool kx_pub_enqueue(QueueHandle_t queue, const kx_pub_result_t *r)
{
    if (xQueueSend(queue, r, 0) == pdTRUE) return true;

    int waited = 0;
    ESP_LOGW(TAG, "pub_queue backpressure: param_id=%d", r->param_id);

    while (waited < KX_PUB_BACKPRESSURE_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(KX_PUB_BACKPRESSURE_WAIT_MS));
        waited += KX_PUB_BACKPRESSURE_WAIT_MS;
        if (xQueueSend(queue, r, 0) == pdTRUE) {
            ESP_LOGI(TAG, "pub_queue backpressure released after %dms", waited);
            return true;
        }
    }

    ESP_LOGE(TAG, "pub_queue DROP param_id=%d after %dms (queue=%d)",
             r->param_id, waited,
             (int)uxQueueMessagesWaiting(queue));
    return false;
}

// =============================================================
// kx_pub_enqueue_status
// kx_pub_enqueue_error
//
// Helpers de construcción + encolado para los dos casos más
// comunes, evitando repetir el armado de kx_pub_result_t.
// =============================================================
bool kx_pub_enqueue_status(QueueHandle_t queue,
                            kx_pub_fn_t pub_fn,
                            kx_pub_kind_t kind,
                            int ctrl_id, int param_id, float value)
{
    kx_pub_result_t r = {
        .pub_fn     = pub_fn,
        .pub_err_fn = NULL,
        .kind       = kind,
        .control_id = ctrl_id,
        .param_id   = param_id,
        .reg        = 0,
        .value      = value,
    };
    r.error_msg[0] = '\0';
    return kx_pub_enqueue(queue, &r);
}

bool kx_pub_enqueue_error(QueueHandle_t queue,
                           kx_pub_err_fn_t pub_err_fn,
                           int ctrl_id, int param_id,
                           uint16_t reg, const char *errmsg)
{
    kx_pub_result_t r = {
        .pub_fn     = NULL,
        .pub_err_fn = pub_err_fn,
        .kind       = PUB_KIND_ERROR,
        .control_id = ctrl_id,
        .param_id   = param_id,
        .reg        = reg,
        .value      = 0.0f,
    };
    if (errmsg)
        snprintf(r.error_msg, sizeof(r.error_msg), "%s", errmsg);
    else
        r.error_msg[0] = '\0';
    return kx_pub_enqueue(queue, &r);
}

// =============================================================
// kx_publish_all_params_for_reg
//
// Dado el raw Modbus (sin transformar), publica el valor
// correcto a TODOS los params del control que comparten
// (reg, fc_read), aplicando offset/addition individual.
//
// tick_s == -1  → modo demand: publicar todos los params
// tick_s >= 0   → modo report: solo params cuyo sampling
//                 divide exactamente tick_s
//
// Devuelve el número de params publicados.
// =============================================================
int kx_publish_all_params_for_reg(QueueHandle_t   queue,
                                   kx_pub_fn_t     pub_fn,
                                   kx_pub_kind_t   pub_kind,
                                   int             control_id,
                                   uint16_t        reg,
                                   uint8_t         fc_read,
                                   uint16_t        raw,
                                   int64_t         ts_ms,
                                   int64_t         tick_s)
{
    int published = 0;

    const kx_control_t *ctrl = kx_param_store_get_ctrl(control_id);
    if (!ctrl) return 0;

    for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS; pi++) {
        kx_param_node_t *pn = ctrl->params.buckets[pi];
        while (pn) {
            kx_param_t *p = &pn->param;

            if ((uint16_t)p->reg           == reg     &&
                (uint8_t) p->function_read == fc_read &&
                p->view != 0) {

                // Filtro de sampling en modo report
                if (tick_s >= 0) {
                    if (p->sampling <= 0 ||
                        (tick_s % (int64_t)p->sampling) != 0) {
                        pn = pn->next;
                        continue;
                    }
                }

                // Transformación del raw
                float value = (float)(int16_t)raw;
                if (p->offset != 0.0f && p->offset != 1.0f)
                    value *= p->offset;
                value += p->addition;
                if (value < p->minvalue) value = p->minvalue;
                if (value > p->maxvalue) value = p->maxvalue;

                // Actualizar timestamps y último valor publicado
                p->ts_last_read         = ts_ms;
                p->last_published_value = value;

                // Encolar publicación
                kx_pub_enqueue_status(queue, pub_fn, pub_kind,
                                      control_id, p->param_id, value);
                published++;
            }
            pn = pn->next;
        }
    }
    return published;
}