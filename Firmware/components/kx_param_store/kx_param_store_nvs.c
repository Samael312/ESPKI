#include "kx_param_store_internal.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "../../../main/kx_config.h"

static const char *TAG = "kx_param_store";

// =============================================================
// kx_param_store_nvs.c — Persistencia en partición NVS "storage"
// =============================================================

// =============================================================
// Init de la partición NVS de almacenamiento
// =============================================================
void kx_nvs_storage_init(void)
{
    esp_err_t err = nvs_flash_init_partition(KX_NVS_PARTITION);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "storage partition: erasing and reinit");
        nvs_flash_erase_partition(KX_NVS_PARTITION);
        err = nvs_flash_init_partition(KX_NVS_PARTITION);
    }
    if (err != ESP_OK)
        ESP_LOGE(TAG, "storage partition init failed: %s", esp_err_to_name(err));
}

// =============================================================
// kx_param_store_nvs_valid
// =============================================================
bool kx_param_store_nvs_valid(void)
{
    nvs_handle_t h;
    if (nvs_open_from_partition(KX_NVS_PARTITION, KX_NVS_NS_STORE,
                                NVS_READONLY, &h) != ESP_OK)
        return false;

    uint32_t magic = 0;
    nvs_get_u32(h, KX_NVS_KEY_MAGIC, &magic);
    char uuid[64] = "";
    size_t len = sizeof(uuid);
    nvs_get_str(h, KX_NVS_KEY_UUID, uuid, &len);
    nvs_close(h);

    bool ok = (magic == KX_NVS_MAGIC_VALUE) && (strcmp(uuid, KX_DEVICE_UUID) == 0);
    ESP_LOGI(TAG, "nvs_valid=%d (magic=0x%08lx uuid=%s)",
             ok, (unsigned long)magic, uuid);
    return ok;
}

// =============================================================
// kx_param_store_save_nvs
// =============================================================
esp_err_t kx_param_store_save_nvs(void)
{
    if (!s_initialized || s_hash.count == 0)
        return ESP_ERR_INVALID_STATE;

    nvs_handle_t h;
    esp_err_t err = nvs_open_from_partition(KX_NVS_PARTITION, KX_NVS_NS_STORE,
                                            NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs open rw failed: %s", esp_err_to_name(err));
        return err;
    }

    nvs_set_u32(h, KX_NVS_KEY_MAGIC, KX_NVS_MAGIC_VALUE);
    nvs_set_str(h, KX_NVS_KEY_UUID,  KX_DEVICE_UUID);
    nvs_set_u8 (h, KX_NVS_KEY_COUNT, (uint8_t)s_hash.count);

    int ctrl_idx = 0;

    for (int ci = 0; ci < KX_CTRL_HASH_BUCKETS; ci++) {
        kx_ctrl_node_t *cn = s_hash.buckets[ci];
        while (cn) {
            kx_control_t *ctrl   = &cn->ctrl;
            int           total  = ctrl->params.count;

            kx_nvs_ctrl_hdr_t hdr = {
                .control_id = ctrl->control_id,
                .slave_addr = ctrl->slave_addr,
                .count      = total,
                .update_ts  = ctrl->update_ts,
                .proto      = (uint8_t)ctrl->proto,
                .tcp_port   = ctrl->tcp_port,
            };
            snprintf(hdr.uuid,   sizeof(hdr.uuid),   "%s", ctrl->uuid);
            snprintf(hdr.tcp_ip, sizeof(hdr.tcp_ip), "%s", ctrl->tcp_ip);

            char hdr_key[16];
            snprintf(hdr_key, sizeof(hdr_key), "hdr_%d", ctrl_idx);
            err = nvs_set_blob(h, hdr_key, &hdr, sizeof(hdr));
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "nvs hdr %s failed: %s",
                         hdr_key, esp_err_to_name(err));
                cn = cn->next; ctrl_idx++;
                continue;
            }

            kx_param_t *flat = malloc((size_t)total * sizeof(kx_param_t));
            if (!flat) {
                ESP_LOGE(TAG, "OOM flat ctrl=%d", ctrl->control_id);
                cn = cn->next; ctrl_idx++;
                continue;
            }
            int idx = 0;
            for (int pi = 0; pi < KX_PARAM_HASH_BUCKETS && idx < total; pi++) {
                kx_param_node_t *pn = ctrl->params.buckets[pi];
                while (pn && idx < total) {
                    memcpy(&flat[idx++], &pn->param, sizeof(kx_param_t));
                    pn = pn->next;
                }
            }

            int chunks = (total + (int)KX_NVS_CHUNK_PARAMS - 1) / (int)KX_NVS_CHUNK_PARAMS;
            if (chunks == 0) chunks = 1;

            for (int j = 0; j < chunks; j++) {
                int offset = j * (int)KX_NVS_CHUNK_PARAMS;
                int count  = total - offset;
                if (count > (int)KX_NVS_CHUNK_PARAMS) count = (int)KX_NVS_CHUNK_PARAMS;
                char chunk_key[16];
                snprintf(chunk_key, sizeof(chunk_key), "p%d_%d", ctrl_idx, j);
                nvs_set_blob(h, chunk_key, flat + offset,
                             (size_t)count * sizeof(kx_param_t));
            }
            free(flat);

            char nchunks_key[16];
            snprintf(nchunks_key, sizeof(nchunks_key), "nc_%d", ctrl_idx);
            nvs_set_u8(h, nchunks_key, (uint8_t)chunks);

            ESP_LOGI(TAG,
                     "nvs: saved ctrl=%d proto=%s tcp=%s:%" PRIu16
                     " params=%d chunks=%d",
                     ctrl->control_id,
                     ctrl->proto == KX_PROTO_TCP ? "tcp" : "rtu",
                     ctrl->tcp_ip, ctrl->tcp_port,
                     total, chunks);

            cn = cn->next;
            ctrl_idx++;
        }
    }

    err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK)
        ESP_LOGI(TAG, "nvs: commit OK — %d controls saved", s_hash.count);
    return err;
}

// =============================================================
// kx_param_store_load_nvs
// =============================================================
esp_err_t kx_param_store_load_nvs(void)
{
    if (!s_initialized) kx_param_store_init();

    nvs_handle_t h;
    esp_err_t err = nvs_open_from_partition(KX_NVS_PARTITION, KX_NVS_NS_STORE,
                                            NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t count = 0;
    nvs_get_u8(h, KX_NVS_KEY_COUNT, &count);
    ESP_LOGI(TAG, "nvs: loading %d controls...", count);

    for (int i = 0; i < count; i++) {
        char hdr_key[16];
        snprintf(hdr_key, sizeof(hdr_key), "hdr_%d", i);

        size_t hdr_sz = sizeof(kx_nvs_ctrl_hdr_t);
        kx_nvs_ctrl_hdr_t hdr = {0};
        err = nvs_get_blob(h, hdr_key, &hdr, &hdr_sz);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "nvs hdr %s missing: %s",
                     hdr_key, esp_err_to_name(err));
            continue;
        }

        kx_control_t *ctrl = kx_ctrl_find_or_create(hdr.control_id);
        if (!ctrl) continue;

        ctrl->slave_addr = hdr.slave_addr;
        ctrl->update_ts  = hdr.update_ts;
        snprintf(ctrl->uuid, sizeof(ctrl->uuid), "%s", hdr.uuid);

        ctrl->proto    = (kx_proto_t)hdr.proto;
        ctrl->tcp_port = hdr.tcp_port;
        snprintf(ctrl->tcp_ip, sizeof(ctrl->tcp_ip), "%s", hdr.tcp_ip);

        ESP_LOGI(TAG, "nvs: ctrl=%d proto=%s tcp=%s:%" PRIu16,
                 hdr.control_id,
                 ctrl->proto == KX_PROTO_TCP ? "tcp" : "rtu",
                 ctrl->tcp_ip, ctrl->tcp_port);

        char nchunks_key[16];
        snprintf(nchunks_key, sizeof(nchunks_key), "nc_%d", i);
        uint8_t n_chunks = 1;
        nvs_get_u8(h, nchunks_key, &n_chunks);

        int total_loaded = 0;
        for (int j = 0; j < n_chunks; j++) {
            char chunk_key[16];
            snprintf(chunk_key, sizeof(chunk_key), "p%d_%d", i, j);
            size_t chunk_bytes = 0;
            if (nvs_get_blob(h, chunk_key, NULL, &chunk_bytes) != ESP_OK
                || chunk_bytes == 0) continue;

            uint8_t *buf = malloc(chunk_bytes);
            if (!buf) { ESP_LOGE(TAG, "OOM chunk %s", chunk_key); continue; }

            if (nvs_get_blob(h, chunk_key, buf, &chunk_bytes) == ESP_OK) {
                int n = (int)(chunk_bytes / sizeof(kx_param_t));
                kx_param_t *params = (kx_param_t *)buf;
                for (int p = 0; p < n; p++) {
                    if (kx_param_insert(ctrl, &params[p]) == ESP_OK)
                        total_loaded++;
                }
            }
            free(buf);
        }

        ctrl->entities_ready = (total_loaded > 0);
        ESP_LOGI(TAG, "nvs: ctrl=%d loaded %d params reg_entries=%d",
                 hdr.control_id, total_loaded, ctrl->regs.count);
    }

    nvs_close(h);
    ESP_LOGI(TAG, "nvs: loaded %d controls OK", s_hash.count);
    return (s_hash.count > 0) ? ESP_OK : ESP_FAIL;
}

// =============================================================
// kx_param_store_clear_nvs
// =============================================================
esp_err_t kx_param_store_clear_nvs(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open_from_partition(KX_NVS_PARTITION, KX_NVS_NS_STORE,
                                            NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(h);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "nvs: cache cleared");
    return err;
}