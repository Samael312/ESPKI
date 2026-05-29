#include "kx_modbus_packetizer.h"
#include "kx_param_store.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <inttypes.h>

#include "../../main/kx_config.h"

static const char *TAG = "kx_pkt";

// =============================================================
// FC de lectura válidos
// =============================================================
#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04

static inline bool _is_read_fc(uint8_t fc)
{
    return (fc == MB_FC_READ_COILS        ||
            fc == MB_FC_READ_DISCRETE     ||
            fc == MB_FC_READ_HOLDING_REGS ||
            fc == MB_FC_READ_INPUT_REGS);
}

// =============================================================
// Candidato: un param que debe leerse en este ciclo
// =============================================================
typedef struct {
    int      control_id;
    int      param_id;
    uint16_t reg;
    uint8_t  fc;
} _candidate_t;

typedef struct {
    _candidate_t *arr;
    int           count;
    int           capacity;
    // filtros
    bool          demand_active;   // true → ignorar tick, leer todos
    int64_t       tick_s;          // segundos actuales (solo en report)
    int64_t       now_ms;
    int           control_id;
} _collect_ctx_t;

// =============================================================
// PSRAM-aware alloc
// =============================================================
static void *_palloc(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!p) p = malloc(size);
    return p;
}

// =============================================================
// Callback de recopilación
// =============================================================
static void _collect_cb(int ctrl_id, const kx_param_t *param, void *ud)
{
    _collect_ctx_t *ctx = (_collect_ctx_t *)ud;

    if (ctrl_id != ctx->control_id) return;

    uint8_t fc = (uint8_t)param->function_read;
    if (!_is_read_fc(fc)) return;
    if (param->view == 0)  return;

    if (ctx->demand_active) {
        // Modo demanda: leer todos los params visibles sin filtro de tiempo
    } else {
        // Modo report: respetar tick_s % sampling == 0
        // igual que hacía _report_param_cb antes del packetizer
        if (param->sampling <= 0) return;
        if (ctx->tick_s % (int64_t)param->sampling != 0) return;
    }

    // Crecer el array si hace falta
    if (ctx->count >= ctx->capacity) {
        int new_cap = ctx->capacity * 2;
        _candidate_t *tmp = realloc(ctx->arr,
                                    (size_t)new_cap * sizeof(_candidate_t));
        if (!tmp) {
            ESP_LOGW(TAG, "collect_cb: realloc failed at %d", ctx->count);
            return;
        }
        ctx->arr      = tmp;
        ctx->capacity = new_cap;
    }

    ctx->arr[ctx->count++] = (_candidate_t){
        .control_id = ctrl_id,
        .param_id   = param->param_id,
        .reg        = (uint16_t)param->reg,
        .fc         = fc,
    };
}

// =============================================================
// Comparador qsort: (fc, reg)
// =============================================================
static int _cmp_candidate(const void *a, const void *b)
{
    const _candidate_t *ca = (const _candidate_t *)a;
    const _candidate_t *cb = (const _candidate_t *)b;
    if (ca->fc  != cb->fc)  return (int)ca->fc  - (int)cb->fc;
    if (ca->reg != cb->reg) return (int)ca->reg - (int)cb->reg;
    return ca->param_id - cb->param_id;
}

// =============================================================
// Garantizar capacidad en la lista de packets
// =============================================================
static bool _list_ensure(kx_packet_list_t *list)
{
    if (list->count < list->capacity) return true;
    int new_cap = list->capacity * 2;
    kx_packet_t *tmp = realloc(list->pkts,
                                (size_t)new_cap * sizeof(kx_packet_t));
    if (!tmp) { ESP_LOGE(TAG, "_list_ensure: OOM cap=%d", new_cap); return false; }
    list->pkts     = tmp;
    list->capacity = new_cap;
    return true;
}

// =============================================================
// _flush_group
//
// Cierra el grupo actual y escribe uno o más kx_packet_t.
// Si el span supera KX_PKT_MAX_REGS_PER_PKT se parte en
// sub-packets del tamaño máximo.
// =============================================================
static void _flush_group(kx_packet_list_t *list,
                         uint8_t slave_addr,
                         uint8_t fc,
                         const _candidate_t *group,
                         int n_group)
{
    if (n_group == 0) return;

    int g_start = 0;

    while (g_start < n_group) {
        uint16_t win_start = group[g_start].reg;
        uint16_t win_end   = win_start + KX_PKT_MAX_REGS_PER_PKT - 1;

        int g_end = g_start;
        while (g_end + 1 < n_group && group[g_end + 1].reg <= win_end)
            g_end++;

        if (!_list_ensure(list)) return;
        kx_packet_t *pkt = &list->pkts[list->count];
        memset(pkt, 0, sizeof(*pkt));

        pkt->slave_addr = slave_addr;
        pkt->fc         = fc;
        pkt->start_reg  = win_start;
        pkt->num_regs   = (uint16_t)(group[g_end].reg - win_start + 1);
        pkt->num_slots  = 0;

        uint16_t cur_reg = win_start;
        int      ci      = g_start;

        while (cur_reg <= group[g_end].reg) {
            if (pkt->num_slots >= KX_PKT_MAX_PARAMS_PER_PKT) {
                ESP_LOGW(TAG, "flush_group: slot overflow reg=0x%04x", cur_reg);
                break;
            }
            kx_pkt_slot_t *slot = &pkt->slots[pkt->num_slots++];

            if (ci <= g_end && group[ci].reg == cur_reg) {
                slot->control_id = group[ci].control_id;
                slot->param_id   = group[ci].param_id;
                slot->reg        = cur_reg;
                slot->is_gap     = false;
                ci++;
            } else {
                slot->control_id = group[g_start].control_id;
                slot->param_id   = -1;
                slot->reg        = cur_reg;
                slot->is_gap     = true;
            }
            cur_reg++;
        }

        list->count++;

        ESP_LOGD(TAG, "packet[%d]: slave=%d fc=0x%02x reg=0x%04x "
                 "num_regs=%d slots=%d (%s)",
                 list->count - 1, slave_addr, fc,
                 pkt->start_reg, pkt->num_regs, pkt->num_slots,
                 (pkt->num_regs == 1) ? "individual" : "multi");

        g_start = g_end + 1;
    }
}

// =============================================================
// kx_pkt_build
// =============================================================
kx_packet_list_t *kx_pkt_build(int     control_id,
                                bool    demand_active,
                                int64_t tick_s,
                                int64_t now_ms)
{
    // Metadatos del control (slave_addr)
    const kx_control_t *ctrl_info = kx_param_store_get_ctrl(control_id);
    if (!ctrl_info || ctrl_info->slave_addr == 0) {
        ESP_LOGW(TAG, "build: ctrl=%d not found or no slave_addr", control_id);
        return NULL;
    }
    uint8_t slave_addr = (uint8_t)ctrl_info->slave_addr;

    // Recopilar candidatos
    int init_cap = 64;
    _candidate_t *arr = _palloc((size_t)init_cap * sizeof(_candidate_t));
    if (!arr) { ESP_LOGE(TAG, "build: OOM candidates"); return NULL; }

    _collect_ctx_t ctx = {
        .arr           = arr,
        .count         = 0,
        .capacity      = init_cap,
        .demand_active = demand_active,
        .tick_s        = tick_s,
        .now_ms        = now_ms,
        .control_id    = control_id,
    };
    kx_param_store_foreach(_collect_cb, &ctx);

    if (ctx.count == 0) {
        ESP_LOGD(TAG, "build: ctrl=%d — no candidates (demand=%d tick=%" PRId64 ")",
                 control_id, (int)demand_active, tick_s);
        free(ctx.arr);
        return NULL;
    }

    ESP_LOGD(TAG, "build: ctrl=%d — %d candidates (demand=%d tick=%" PRId64 ")",
             control_id, ctx.count, (int)demand_active, tick_s);

    // Ordenar por (fc, reg)
    qsort(ctx.arr, (size_t)ctx.count, sizeof(_candidate_t), _cmp_candidate);

    // Crear lista de salida
    kx_packet_list_t *list = _palloc(sizeof(kx_packet_list_t));
    if (!list) { free(ctx.arr); return NULL; }

    int init_pkt_cap = (ctx.count / 4) + 4;
    list->pkts = _palloc((size_t)init_pkt_cap * sizeof(kx_packet_t));
    if (!list->pkts) { free(ctx.arr); free(list); return NULL; }
    list->count    = 0;
    list->capacity = init_pkt_cap;

    // Buffer temporal para el grupo actual
    _candidate_t *group = malloc((size_t)KX_PKT_MAX_REGS_PER_PKT *
                                  sizeof(_candidate_t));
    if (!group) {
        ESP_LOGE(TAG, "build: OOM group buffer");
        free(ctx.arr); free(list->pkts); free(list);
        return NULL;
    }
    int n_group = 0;

    for (int i = 0; i < ctx.count; i++) {
        const _candidate_t *cur = &ctx.arr[i];

        bool new_group = false;
        if (n_group == 0) {
            new_group = true;
        } else {
            const _candidate_t *prev = &group[n_group - 1];
            if (cur->fc != prev->fc)
                new_group = true;
            else if ((int)cur->reg - (int)prev->reg > KX_PKT_MAX_GAP + 1)
                new_group = true;
            else if ((int)cur->reg - (int)group[0].reg >= KX_PKT_MAX_REGS_PER_PKT)
                new_group = true;
        }

        if (new_group && n_group > 0) {
            _flush_group(list, slave_addr, group[0].fc, group, n_group);
            n_group = 0;
        }

        group[n_group++] = *cur;
    }
    if (n_group > 0)
        _flush_group(list, slave_addr, group[0].fc, group, n_group);

    free(group);
    free(ctx.arr);

    ESP_LOGI(TAG, "ctrl=%d → %d params → %d packets (demand=%d tick=%" PRId64 " gap=%d)",
             control_id, ctx.count, list->count,
             (int)demand_active, tick_s, KX_PKT_MAX_GAP);

    if (list->count == 0) { kx_pkt_free(list); return NULL; }
    return list;
}

// =============================================================
// kx_pkt_free
// =============================================================
void kx_pkt_free(kx_packet_list_t *list)
{
    if (!list) return;
    if (list->pkts) free(list->pkts);
    free(list);
}

// =============================================================
// kx_pkt_real_param_count
// =============================================================
int kx_pkt_real_param_count(const kx_packet_list_t *list)
{
    if (!list) return 0;
    int count = 0;
    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];
        for (int s = 0; s < pkt->num_slots; s++)
            if (!pkt->slots[s].is_gap) count++;
    }
    return count;
}

// =============================================================
// kx_pkt_dump
// =============================================================
void kx_pkt_dump(const kx_packet_list_t *list, const char *tag)
{
    if (!list) { ESP_LOGI(tag, "packet_list: NULL"); return; }

    ESP_LOGI(tag, "packet_list: %d packets, %d real params",
             list->count, kx_pkt_real_param_count(list));

    for (int i = 0; i < list->count; i++) {
        const kx_packet_t *pkt = &list->pkts[i];
        if (pkt->num_regs == 1) {
            ESP_LOGI(tag, "  [%02d] INDIVIDUAL slave=%d fc=0x%02x "
                     "reg=0x%04x → param_id=%d",
                     i, pkt->slave_addr, pkt->fc,
                     pkt->start_reg,
                     (pkt->num_slots > 0) ? pkt->slots[0].param_id : -1);
        } else {
            ESP_LOGI(tag, "  [%02d] MULTI      slave=%d fc=0x%02x "
                     "reg=0x%04x..0x%04x num_regs=%d slots=%d",
                     i, pkt->slave_addr, pkt->fc,
                     pkt->start_reg,
                     (uint16_t)(pkt->start_reg + pkt->num_regs - 1),
                     pkt->num_regs, pkt->num_slots);
            for (int s = 0; s < pkt->num_slots; s++) {
                const kx_pkt_slot_t *sl = &pkt->slots[s];
                if (sl->is_gap)
                    ESP_LOGD(tag, "       slot[%d] reg=0x%04x GAP", s, sl->reg);
                else
                    ESP_LOGD(tag, "       slot[%d] reg=0x%04x param_id=%d",
                             s, sl->reg, sl->param_id);
            }
        }
    }
}