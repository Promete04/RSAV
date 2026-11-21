/* main.c - ESP32-S3 (or S2/P4) as USB host driving a Roomba 980 through its micro-USB OI port.
 * ESP-IDF >= 5.1, managed component espressif/usb_host_cdc_acm 2.x (see idf_component.yml).
 * Build: idf.py set-target esp32s3 && idf.py build flash monitor
 *
 * Flow: install USB host -> open CDC-ACM device 27A6:0002 -> Start -> stream sensors
 *       -> Safe -> drive a 40 cm square using the wheel encoders -> Passive -> OI off.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

#include "oi_proto.h"

#define ROOMBA_VID 0x27A6
#define ROOMBA_PID 0x0002

static const char *TAG = "roomba";

static cdc_acm_dev_hdl_t s_cdc = NULL;
static SemaphoreHandle_t s_disconnected;

/* Latest streamed sensor values, written by the USB RX callback. */
static oi_parser_t s_parser;
static oi_frame_t  s_state;
static int64_t     s_state_us;          /* esp_timer time of the last good frame */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---------------- transport ---------------- */
static esp_err_t oi_send(const uint8_t *buf, size_t n)
{
    if (!s_cdc) return ESP_ERR_INVALID_STATE;
    return cdc_acm_host_data_tx_blocking(s_cdc, buf, n, 100);
}

static void oi_cmd(uint8_t opcode)
{
    uint8_t b[1];
    oi_send(b, oi_cmd_simple(b, opcode));
    if (opcode == OI_START || opcode == OI_SAFE || opcode == OI_FULL) vTaskDelay(pdMS_TO_TICKS(30));
}

static void drive_direct(int left, int right)
{
    uint8_t b[5];
    oi_send(b, oi_cmd_drive_direct(b, left, right));
}

static void stop_wheels(void) { drive_direct(0, 0); }

/* Runs inside the parser, inside the USB driver task: copy and return quickly. */
static void on_frame(const oi_frame_t *f, void *arg)
{
    (void)arg;
    taskENTER_CRITICAL(&s_lock);
    for (int id = 0; id <= OI_MAX_PACKET; id++)
        if (f->present & (1ULL << id)) s_state.v[id] = f->v[id];
    s_state.present |= f->present;
    s_state_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_lock);
}

/* Component 2.x signature. For 1.x, change the return type to void and drop the return. */
static bool on_rx(const uint8_t *data, size_t len, void *arg)
{
    (void)arg;
    oi_parser_feed(&s_parser, data, len, on_frame, NULL);  /* log text is skipped by the parser */
    return true;                                            /* data consumed */
}

static void on_event(const cdc_acm_host_dev_event_data_t *event, void *ctx)
{
    (void)ctx;
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC-ACM error %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "Roomba unplugged");
        cdc_acm_host_close(event->data.cdc_hdl);
        s_cdc = NULL;
        xSemaphoreGive(s_disconnected);
        break;
    default:
        break;
    }
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

/* ---------------- sensor access ---------------- */
/* Copy of the latest values; returns false if no frame arrived in the last 200 ms. */
static bool read_state(int32_t *v)
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(v, s_state.v, sizeof s_state.v);
    int64_t age = esp_timer_get_time() - s_state_us;
    taskEXIT_CRITICAL(&s_lock);
    return age < 200000;
}

/* ---------------- closed-loop moves on streamed encoders ---------------- */
typedef enum { MOVE_OK, MOVE_BUMP, MOVE_DROP, MOVE_MODE, MOVE_STALE } move_res_t;

static move_res_t guard(const int32_t *v, bool check_bump)
{
    if (check_bump && (v[PKT_BUMPS_DROPS] & (OI_BUMP_LEFT | OI_BUMP_RIGHT))) return MOVE_BUMP;
    if (v[PKT_BUMPS_DROPS] & (OI_DROP_LEFT | OI_DROP_RIGHT)) return MOVE_DROP;
    if (v[PKT_OI_MODE] < 2) return MOVE_MODE;
    return MOVE_OK;
}

static move_res_t move_mm(double mm, int speed)
{
    int32_t v0[OI_MAX_PACKET + 1], v[OI_MAX_PACKET + 1];
    int sign = mm >= 0 ? 1 : -1;
    if (!read_state(v0)) return MOVE_STALE;
    move_res_t r = guard(v0, sign > 0);
    if (r != MOVE_OK) return r;
    drive_direct(sign * speed, sign * speed);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15));                       /* one stream period */
        if (!read_state(v)) { r = MOVE_STALE; break; }       /* stream lost: stop */
        if ((r = guard(v, sign > 0)) != MOVE_OK) break;
        double dl = oi_enc_delta(v0[PKT_ENC_LEFT], v[PKT_ENC_LEFT]) * OI_MM_PER_COUNT;
        double dr = oi_enc_delta(v0[PKT_ENC_RIGHT], v[PKT_ENC_RIGHT]) * OI_MM_PER_COUNT;
        if (fabs((dl + dr) / 2) >= fabs(mm)) break;
    }
    stop_wheels();
    return r;
}

static move_res_t turn_deg(double deg, int speed)            /* positive = left */
{
    int32_t v0[OI_MAX_PACKET + 1], v[OI_MAX_PACKET + 1];
    int sign = deg >= 0 ? 1 : -1;
    double arc = M_PI * OI_WHEEL_BASE_MM * fabs(deg) / 360.0;
    if (!read_state(v0)) return MOVE_STALE;
    move_res_t r = guard(v0, false);
    if (r != MOVE_OK) return r;
    drive_direct(-sign * speed, sign * speed);
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(15));
        if (!read_state(v)) { r = MOVE_STALE; break; }
        if ((r = guard(v, false)) != MOVE_OK) break;
        double dl = oi_enc_delta(v0[PKT_ENC_LEFT], v[PKT_ENC_LEFT]) * OI_MM_PER_COUNT;
        double dr = oi_enc_delta(v0[PKT_ENC_RIGHT], v[PKT_ENC_RIGHT]) * OI_MM_PER_COUNT;
        if ((fabs(dl) + fabs(dr)) / 2 >= arc) break;
    }
    stop_wheels();
    return r;
}

/* ---------------- app ---------------- */
void app_main(void)
{
    s_disconnected = xSemaphoreCreateBinary();

    const usb_host_config_t host_cfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    const cdc_acm_host_device_config_t dev_cfg = {
        .connection_timeout_ms = 1000,
        .out_buffer_size = 64,
        .in_buffer_size = 512,
        .event_cb = on_event,
        .data_cb = on_rx,
        .user_arg = NULL,
    };

    for (;;) {
        ESP_LOGI(TAG, "waiting for Roomba %04X:%04X", ROOMBA_VID, ROOMBA_PID);
        if (cdc_acm_host_open(ROOMBA_VID, ROOMBA_PID, 0, &dev_cfg, &s_cdc) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        cdc_acm_line_coding_t lc = { .dwDTERate = 115200, .bCharFormat = 0, .bParityType = 0, .bDataBits = 8 };
        cdc_acm_host_line_coding_set(s_cdc, &lc);          /* 115200 8N1 */
        cdc_acm_host_set_control_line_state(s_cdc, true, false);

        memset(&s_parser, 0, sizeof s_parser);
        oi_cmd(OI_START);                                    /* Passive */
        static const uint8_t ids[] = { PKT_BUMPS_DROPS, PKT_OI_MODE, PKT_ENC_LEFT, PKT_ENC_RIGHT,
                                       PKT_VOLTAGE, PKT_CURRENT };
        uint8_t b[16];
        oi_send(b, oi_cmd_stream(b, ids, sizeof ids));       /* frames every 15 ms */
        vTaskDelay(pdMS_TO_TICKS(300));

        int32_t v[OI_MAX_PACKET + 1];
        if (read_state(v))
            ESP_LOGI(TAG, "battery %ld mV, %ld mA, mode %ld", (long)v[PKT_VOLTAGE], (long)v[PKT_CURRENT], (long)v[PKT_OI_MODE]);

        /* Demo: 40 cm square. Robot must be off the dock, on the floor. */
        oi_cmd(OI_SAFE);
        static const char *TXT[] = { "ok", "bumper", "wheel drop", "mode drop", "stream lost" };
        for (int side = 0; side < 4; side++) {
            move_res_t r = move_mm(400, 200);
            if (r == MOVE_OK) r = turn_deg(90, 100);
            ESP_LOGI(TAG, "side %d: %s", side + 1, TXT[r]);
            if (r != MOVE_OK) break;
        }
        stop_wheels();
        oi_cmd(OI_START);                                    /* back to Passive */

        /* Keep streaming; log the battery every 5 s until the cable is pulled. */
        while (xSemaphoreTake(s_disconnected, pdMS_TO_TICKS(5000)) != pdTRUE) {
            if (read_state(v))
                ESP_LOGI(TAG, "battery %ld mV %ld mA, %lu good / %lu bad frames", (long)v[PKT_VOLTAGE],
                         (long)v[PKT_CURRENT], (unsigned long)s_parser.good_frames, (unsigned long)s_parser.bad_frames);
        }
    }
}
