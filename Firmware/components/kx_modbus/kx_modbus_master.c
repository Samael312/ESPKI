#include "kx_modbus_master.h"
#include "kx_param_store.h"
#include "kx_mqtt.h"
#include "kx_system.h"
#include "../../main/kx_config.h"
#include "kx_telemetry.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <float.h>

static const char *TAG = "kx_modbus";

// =============================================================
// Parámetros UART/RS-485
// =============================================================
#ifndef KX_MODBUS_UART_NUM
#define KX_MODBUS_UART_NUM   UART_NUM_1
#endif
#ifndef KX_MODBUS_BAUD
#define KX_MODBUS_BAUD       9600
#endif
#ifndef KX_MODBUS_TX_PIN
#define KX_MODBUS_TX_PIN     GPIO_NUM_4
#endif
#ifndef KX_MODBUS_RX_PIN
#define KX_MODBUS_RX_PIN     GPIO_NUM_36
#endif
#ifndef KX_MODBUS_RTS_PIN
#define KX_MODBUS_RTS_PIN    -1
#endif

#define MODBUS_RESPONSE_TIMEOUT_MS   100
#define MODBUS_INTER_FRAME_MS         20
#define MODBUS_INTER_PARAM_MS         10
#define MODBUS_RETRY_COUNT             2

#define MB_FC_READ_COILS           0x01
#define MB_FC_READ_DISCRETE        0x02
#define MB_FC_READ_HOLDING_REGS    0x03
#define MB_FC_READ_INPUT_REGS      0x04
#define MB_FC_WRITE_SINGLE_COIL    0x05
#define MB_FC_WRITE_SINGLE_REG     0x06
#define MB_FC_WRITE_MULTIPLE_REGS  0x10

// =============================================================
// Sincronización
//
// s_poll_eg — POLL_ALLOWED_BIT:
//   seteado   → el ciclo puede arrancar
//   limpiado  → el ciclo no arrancará en la siguiente iteración
//
// s_foreach_mutex:
//   El task lo toma ANTES del foreach y lo suelta AL TERMINAR.
//   kx_modbus_pause() limpia POLL_ALLOWED_BIT y luego toma el
//   mutex → espera a que el foreach en curso termine antes de
//   devolver el control. kx_param_store_clear_entities() se
//   llama únicamente después de que pause() devuelve, por lo
//   que nunca toca nodos que el iterador aún está usando.
// =============================================================
#define POLL_ALLOWED_BIT   BIT0

static EventGroupHandle_t  s_poll_eg       = NULL;
static SemaphoreHandle_t   s_foreach_mutex = NULL;
static volatile bool       s_running       = false;
static TaskHandle_t        s_task          = NULL;

// ── CRC16 Modbus ──────────────────────────────────────────────
static uint16_t _crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

// ── Init UART ─────────────────────────────────────────────────
static esp_err_t _uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = KX_MODBUS_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err;
    err = uart_param_config(KX_MODBUS_UART_NUM, &cfg);
    if (err != ESP_OK) return err;

    err = uart_set_pin(KX_MODBUS_UART_NUM,
                       KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN,
                       KX_MODBUS_RTS_PIN, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    err = uart_driver_install(KX_MODBUS_UART_NUM, 256, 256, 0, NULL, 0);
    if (err != ESP_OK) return err;

    if (KX_MODBUS_RTS_PIN != UART_PIN_NO_CHANGE) {
        err = uart_set_mode(KX_MODBUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "RS485 half-duplex mode failed: %s", esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "UART%d ready: baud=%d TX=%d RX=%d RTS=%d",
             KX_MODBUS_UART_NUM, KX_MODBUS_BAUD,
             KX_MODBUS_TX_PIN, KX_MODBUS_RX_PIN, KX_MODBUS_RTS_PIN);
    return ESP_OK;
}

// ── Transacción Modbus ────────────────────────────────────────
static int _modbus_transaction(const uint8_t *frame, size_t frame_len,
                               uint8_t *resp, size_t resp_max)
{
    uint8_t tx[frame_len + 2];
    memcpy(tx, frame, frame_len);
    uint16_t crc = _crc16(frame, frame_len);
    tx[frame_len]     = (uint8_t)(crc & 0xFF);
    tx[frame_len + 1] = (uint8_t)(crc >> 8);

    uart_flush_input(KX_MODBUS_UART_NUM);
    uart_write_bytes(KX_MODBUS_UART_NUM, (const char *)tx, frame_len + 2);

    int rx_len = uart_read_bytes(KX_MODBUS_UART_NUM, resp, resp_max,
                                  pdMS_TO_TICKS(MODBUS_RESPONSE_TIMEOUT_MS));
    if (rx_len <= 0) return -1;
    if (rx_len < 4)  return -1;

    uint16_t rx_crc   = ((uint16_t)resp[rx_len - 1] << 8) | resp[rx_len - 2];
    uint16_t calc_crc = _crc16(resp, rx_len - 2);
    if (rx_crc != calc_crc) {
        ESP_LOGW(TAG, "CRC error: got %04x expected %04x", rx_crc, calc_crc);
        return -1;
    }

    if (resp[1] & 0x80) {
        uint8_t exc = (rx_len > 2) ? resp[2] : 0;
        ESP_LOGW(TAG, "Modbus exception: fc=0x%02x exc=0x%02x", resp[1], exc);
        return -1;
    }

    return rx_len;
}

// ── Leer un registro ──────────────────────────────────────────
static float _read_register(uint8_t slave_addr, uint16_t reg_addr,
                             uint8_t fc, const kx_param_t *param)
{
    uint8_t frame[6] = {
        slave_addr,
        fc,
        (uint8_t)(reg_addr >> 8),
        (uint8_t)(reg_addr & 0xFF),
        0x00,
        0x01,
    };

    uint8_t resp[16];
    int rx = -1;

    for (int attempt = 0; attempt < MODBUS_RETRY_COUNT && rx < 0; attempt++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
    }

    if (rx < 0) return -FLT_MAX;
    if (rx < 4 || resp[2] == 0) return -FLT_MAX;

    uint16_t raw;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE) {
        raw = resp[3] & 0x01;
    } else {
        if (rx < 5 || resp[2] < 2) return -FLT_MAX;
        raw = ((uint16_t)resp[3] << 8) | resp[4];
    }

    float value = (float)(int16_t)raw;
    if (param->offset != 0.0f && param->offset != 1.0f) value *= param->offset;
    value += param->addition;
    if (value < param->minvalue) value = param->minvalue;
    if (value > param->maxvalue) value = param->maxvalue;

    return value;
}

// ── Contexto de iteración ─────────────────────────────────────
typedef struct { int total; int done; int ok; int errors; } _poll_ctx_t;
typedef struct { int count; } _count_ctx_t;

// ── Barra de progreso ─────────────────────────────────────────
#define POLL_BAR_WIDTH 30

static void _print_progress(int control_id, int done, int total)
{
    if (total <= 0) return;
    int pct  = (done * 100) / total;
    int fill = (done * POLL_BAR_WIDTH) / total;
    char bar[POLL_BAR_WIDTH + 1];
    for (int i = 0; i < POLL_BAR_WIDTH; i++) bar[i] = (i < fill) ? '#' : '-';
    bar[POLL_BAR_WIDTH] = '\0';
    printf("[poll] ctrl=%d [%s] %3d%% (%d/%d params)\r", control_id, bar, pct, done, total);
    fflush(stdout);
    if (pct == 25 || pct == 50 || pct == 75 || pct == 100) { printf("\n"); fflush(stdout); }
}

// ── Publicar valor / error ────────────────────────────────────
static void _publish_value(int control_id, const kx_param_t *param, float value)
{
    if (param->function_read != 0) {
        kx_param_pub_status(control_id, param->param_id, value);
        
    } else {
        ESP_LOGW(TAG, "param ctrl=%d param=%d no tiene función de lectura definida",
                 control_id, param->param_id);
    }
}

static void _publish_error(int control_id, const kx_param_t *param, const char *reason)
{
    kx_param_pub_error(control_id, param->param_id, reason, (uint16_t)param->reg);
    ESP_LOGW(TAG, "modbus error ctrl=%d param=%d reg=0x%04x: %s",
             control_id, param->param_id, param->reg, reason);
}

// ── Callback de iteración ─────────────────────────────────────
// Sin xEventGroupWaitBits interno: la pausa se gestiona a nivel
// de ciclo completo mediante s_foreach_mutex, garantizando que
// clear_entities() nunca se ejecuta con el iterador activo.
static void _poll_param(int control_id, const kx_param_t *param, void *user_data)
{
    _poll_ctx_t *ctx = (_poll_ctx_t *)user_data;

    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return;

    uint8_t fc_read = (uint8_t)param->function_read;
    bool is_read_fc = (fc_read == MB_FC_READ_COILS        ||
                       fc_read == MB_FC_READ_DISCRETE      ||
                       fc_read == MB_FC_READ_HOLDING_REGS  ||
                       fc_read == MB_FC_READ_INPUT_REGS);
    if (!is_read_fc) return;

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg, fc_read, param);

    if (value == -FLT_MAX) {
        _publish_error(control_id, param, "modbus_timeout");
        if (ctx) ctx->errors++;
    } else {
        _publish_value(control_id, param, value);
        if (ctx) ctx->ok++;
    }

    if (ctx) {
        ctx->done++;
        _print_progress(control_id, ctx->done, ctx->total);
    }

    vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_PARAM_MS));
}

static void _count_readable(int control_id, const kx_param_t *param, void *user_data)
{
    _count_ctx_t *c = (_count_ctx_t *)user_data;
    if (param->function_read == 0 && param->function_write == 0) return;
    if (param->view == 0) return;
    uint8_t fc = (uint8_t)param->function_read;
    if (fc == MB_FC_READ_COILS || fc == MB_FC_READ_DISCRETE ||
        fc == MB_FC_READ_HOLDING_REGS || fc == MB_FC_READ_INPUT_REGS) c->count++;
}

// ── Tarea principal ───────────────────────────────────────────
static void _modbus_task(void *arg)
{
    ESP_LOGI(TAG, "task started — waiting for entities...");

    while (!kx_param_store_is_ready()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "entities ready (%d controls) — starting Modbus polling",
             kx_param_store_count());
    vTaskDelay(pdMS_TO_TICKS(4000));

    while (s_running) {

        // Esperar permiso (fuera del mutex para que pause() pueda tomarlo)
        xEventGroupWaitBits(s_poll_eg, POLL_ALLOWED_BIT,
                            pdFALSE, pdTRUE, portMAX_DELAY);

        if (!kx_mqtt_is_connected()) {
            vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
            continue;
        }

        if (!kx_param_store_is_ready()) {
            // Entities borradas por update_ts mayor, esperar a que lleguen
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Proteger el foreach: pause() no puede liberar memoria hasta
        // que este mutex sea liberado al final del ciclo.
        xSemaphoreTake(s_foreach_mutex, portMAX_DELAY);

        // Re-verificar tras tomar el mutex
        if (!s_running || !kx_param_store_is_ready()) {
            xSemaphoreGive(s_foreach_mutex);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        _count_ctx_t cc = { .count = 0 };
        kx_param_store_foreach(_count_readable, &cc);

        _poll_ctx_t ctx = { .total = cc.count, .done = 0, .ok = 0, .errors = 0 };

        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        ESP_LOGI(TAG, "poll cycle: %d controls | %d params | heap=%" PRIu32,
                 kx_param_store_count(), ctx.total, kx_system_heap_free());

        kx_param_store_foreach(_poll_param, &ctx);

        xSemaphoreGive(s_foreach_mutex);

        ESP_LOGI(TAG, "poll done: read=%d errors=%d | heap=%" PRIu32,
                 ctx.ok, ctx.errors, kx_system_heap_free());
        ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

        vTaskDelay(pdMS_TO_TICKS(KX_TELEMETRY_INTERVAL_S * 1000));
    }

    uart_driver_delete(KX_MODBUS_UART_NUM);
    ESP_LOGI(TAG, "task stopped");
    vTaskDelete(NULL);
}

// =============================================================
// API pública — pause / resume
//
// pause():
//   1. Limpia POLL_ALLOWED_BIT → el task no iniciará un nuevo
//      ciclo una vez que el actual termine.
//   2. Toma s_foreach_mutex → se bloquea hasta que el foreach
//      en curso libere el mutex. Si el task está en vTaskDelay
//      entre ciclos (fuera del mutex), el take es inmediato.
//   3. Retorna con el mutex tomado: el task no puede entrar en
//      un nuevo foreach hasta resume().
//
// Es seguro llamar a kx_param_store_clear_entities() entre
// pause() y resume().
// =============================================================
void kx_modbus_pause(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;

    ESP_LOGI(TAG, "pausing Modbus polling...");
    xEventGroupClearBits(s_poll_eg, POLL_ALLOWED_BIT);

    // Timeout generoso: worst case 311 params × ~130 ms = ~40 s
    if (xSemaphoreTake(s_foreach_mutex, pdMS_TO_TICKS(60000)) != pdTRUE) {
        ESP_LOGE(TAG, "pause: timeout waiting for foreach — memory risk!");
    } else {
        ESP_LOGI(TAG, "Modbus polling paused (foreach complete)");
    }
    // Mutex retenido hasta resume()
}

void kx_modbus_resume(void)
{
    if (!s_poll_eg || !s_foreach_mutex) return;
    xSemaphoreGive(s_foreach_mutex);
    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);
    ESP_LOGI(TAG, "Modbus polling resumed");
}

// =============================================================
// API pública — start / stop
// =============================================================
esp_err_t kx_modbus_master_start(void)
{
    if (s_running) { ESP_LOGW(TAG, "already running"); return ESP_OK; }

    s_poll_eg = xEventGroupCreate();
    if (!s_poll_eg) { ESP_LOGE(TAG, "failed to create EventGroup"); return ESP_FAIL; }

    s_foreach_mutex = xSemaphoreCreateMutex();
    if (!s_foreach_mutex) { ESP_LOGE(TAG, "failed to create mutex"); return ESP_FAIL; }

    xEventGroupSetBits(s_poll_eg, POLL_ALLOWED_BIT);

    esp_err_t err = _uart_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "UART init failed: %s", esp_err_to_name(err)); return err; }

    s_running = true;
    BaseType_t ret = xTaskCreate(_modbus_task, "kx_modbus", 8192, NULL,
                                  KX_TASK_PRIO_TELEMETRY, &s_task);
    if (ret != pdPASS) { s_running = false; return ESP_FAIL; }

    return ESP_OK;
}

void kx_modbus_master_stop(void) { s_running = false; }

bool kx_modbus_master_is_running(void) { return s_running; }

esp_err_t kx_modbus_read_one(int control_id, int param_id)
{
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) return ESP_ERR_NOT_FOUND;

    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) return ESP_ERR_INVALID_STATE;

    float value = _read_register((uint8_t)ctrl->slave_addr,
                                  (uint16_t)param->reg,
                                  (uint8_t)param->function_read, param);
    if (value == -FLT_MAX) {
        _publish_error(control_id, param, "modbus_timeout");
        return ESP_FAIL;
    }
    _publish_value(control_id, param, value);
    return ESP_OK;
}

esp_err_t kx_modbus_write_one(int control_id, int param_id, float value)
{
    // ── Buscar param y control ────────────────────────────────
    const kx_param_t *param = kx_param_store_get_param(control_id, param_id);
    if (!param) {
        ESP_LOGW(TAG, "write_one: param no encontrado ctrl=%d param=%d",
                 control_id, param_id);
        return ESP_ERR_NOT_FOUND;
    }
 
    const kx_control_params_t *ctrl = kx_param_store_get(control_id);
    if (!ctrl || ctrl->slave_addr == 0) {
        ESP_LOGW(TAG, "write_one: slave_addr inválido ctrl=%d", control_id);
        return ESP_ERR_INVALID_STATE;
    }
 
    // ── Verificar que existe función de escritura ─────────────
    uint8_t fc_write = (uint8_t)param->function_write;
    if (fc_write != MB_FC_WRITE_SINGLE_COIL    &&
        fc_write != MB_FC_WRITE_SINGLE_REG     &&
        fc_write != MB_FC_WRITE_MULTIPLE_REGS) {
        ESP_LOGW(TAG, "write_one: FC de escritura no soportado fc=0x%02x param=%d",
                 fc_write, param_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
 
    // ── Transformación inversa value → raw ───────────────────
    int16_t raw;
 
    if (fc_write == MB_FC_WRITE_SINGLE_COIL) {
        // [FIX] Para Coils (FC 05), Modbus requiere estrictamente 0xFF00 para ON y 0x0000 para OFF.
        // Ignoramos por completo offsets, additions y el clamping de registros.
        raw = (value > 0.0f) ? (int16_t)0xFF00 : 0x0000;
    } else {
        // Lógica normal para registros (FC 06, FC 16, etc.)
        // value = (float)(int16_t)raw -> inversa: adjusted = value - addition
        float adjusted = value - param->addition;
 
        if (param->offset != 0.0f && param->offset != 1.0f) {
            raw = (int16_t)(adjusted / param->offset);
        } else {
            raw = (int16_t)adjusted;
        }
 
        // Clampear al rango permitido (en unidades raw, pre-transformación)
        if ((float)raw < param->minvalue) raw = (int16_t)param->minvalue;
        if ((float)raw > param->maxvalue) raw = (int16_t)param->maxvalue;
    }
 
    // Modificado el log para imprimir también en Hexadecimal (ayuda mucho con las Coils)
    ESP_LOGI(TAG, "write_one: ctrl=%d param=%d reg=0x%04x fc=0x%02x "
             "slave=%d value=%.3f → raw=%d (0x%04X)",
             control_id, param_id, param->reg, fc_write,
             ctrl->slave_addr, value, (int)(uint16_t)raw, (uint16_t)raw);
 
    // ── Construir trama Modbus ────────────────────────────────
    // Nota: El arreglo de 6 bytes funciona idéntico tanto para FC 05 como para FC 06.
    uint8_t frame[6] = {
        (uint8_t)ctrl->slave_addr,
        fc_write,
        (uint8_t)((uint16_t)param->reg >> 8),
        (uint8_t)((uint16_t)param->reg & 0xFF),
        (uint8_t)((uint16_t)raw >> 8),
        (uint8_t)((uint16_t)raw & 0xFF),
    };
 
    uint8_t resp[16];
    int rx = -1;
 
    for (int attempt = 0; attempt < MODBUS_RETRY_COUNT && rx < 0; attempt++) {
        rx = _modbus_transaction(frame, sizeof(frame), resp, sizeof(resp));
        if (rx < 0) {
            ESP_LOGD(TAG, "write_one: intento %d fallido, reintentando...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(MODBUS_INTER_FRAME_MS));
        }
    }
 
    if (rx < 0) {
        ESP_LOGW(TAG, "write_one: sin respuesta tras %d intentos ctrl=%d param=%d",
                 MODBUS_RETRY_COUNT, control_id, param_id);
        return ESP_FAIL;
    }
 
    // ── Validar respuesta eco (FC 05 y FC 06) ─────────────────
    // Ambos comandos devuelven un espejo exacto de los primeros 4 bytes.
    if (rx < 6 ||
        resp[0] != frame[0] ||   // slave_addr
        resp[1] != frame[1] ||   // function code
        resp[2] != frame[2] ||   // reg_hi
        resp[3] != frame[3])     // reg_lo
    {
        ESP_LOGW(TAG, "write_one: respuesta inesperada (rx=%d) ctrl=%d param=%d",
                 rx, control_id, param_id);
        return ESP_FAIL;
    }
 
    uint16_t echo_val = ((uint16_t)resp[4] << 8) | resp[5];
    ESP_LOGI(TAG, "write_one: OK ctrl=%d param=%d raw_sent=%d raw_echo=%d",
             control_id, param_id, (int)(uint16_t)raw, (int)echo_val);
 
    return ESP_OK;
}