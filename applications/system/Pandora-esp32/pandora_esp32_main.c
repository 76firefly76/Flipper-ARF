/* pandora_esp32_main.c - Flipper Zero companion for the ESP32-WROOM-32 running
 * the Pandora D-605 keyfob emulator.
 *
 * SPLIT OF WORK:
 *   - The ESP32 runs the heavy emulation (the REAL keyfob firmware).
 *   - The Flipper is the DISPLAY + KEYPAD + RADIO front-end.
 *
 * FLOW:
 *   1. On start, read a firmware .hex from the Flipper SD and stream it to the
 *      ESP32 (FW_BEGIN / FW_CHUNK* / FW_END). Wait for FW_ACK.
 *   2. Loop:
 *        ESP32 -> Flipper : FB (render on screen), RF_MODE (configure CC1101),
 *                           RF_TX (transmit over CC1101), LOG, PONG, FW_ACK.
 *        Flipper -> ESP32 : BTN (keypad), RX_EDGE (CC1101 air RX), PING, ...
 *
 * UART transport follows the wifi_marauder_companion pattern (see
 * pandora_esp32_uart.c). The OLED render + button map + CC1101 bridge are
 * ported from PandoraARM (see pandora_esp32_cc1101.c and render_oled below).
 */

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <input/input.h>
#include <dialogs/dialogs.h>
#include <storage/storage.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pandora_esp32_protocol.h"
#include "pandora_esp32_uart.h"
#include "pandora_esp32_cc1101.h"

#define TAG "PandoraEsp32"

/* ------------------------------------------------------------ screen geometry
 * Copied from PandoraARM: the keyfob OLED is 92 columns x 16 pages, 2bpp. Only
 * 88 columns are on the physical glass; render those centered on the 128-wide
 * Flipper screen. */
#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_COLS 92
#define OLED_VISIBLE_COLS 88 /* physical glass width (cols 88..91 = off-glass) */
#define OLED_PAGES 16
#define OLED_FB_SIZE (OLED_COLS * OLED_PAGES) /* 1472 bytes */

/* ------------------------------------------------------------ hold timings
 * Copied from PandoraARM. */
#define HOLD_MS 500u
#define EXIT_HOLD_MS 1000u

/* Where the firmware images live on the Flipper SD. */
#define PANDORA_FW_DIR "/ext/apps_data/pandora"
#define FW_CHUNK_BYTES PROTO_MAX_PAYLOAD
#define FW_ACK_TIMEOUT_MS 60000u
#define RX_EDGE_RING_LEN 4096u
#define RX_EDGE_BATCH 128u
#define RX_PREROLL_EDGES 256u
#define RX_GATE_OPEN_DB 8.0f
#define RX_GATE_CLOSE_DB 4.0f
#define RX_GATE_HOLD_MS 120u
#define RX_UI_MIN_PERIOD_MS 100u

typedef struct __attribute__((packed)) {
    uint8_t level;
    uint32_t duration_us;
} RxEdge;

typedef enum {
    UploadCheckingLink = 0,
    UploadChoosingFile,
    UploadSending,
    UploadWaitingAck,
    UploadReady,
    UploadNoLink,
    UploadNoFile,
    UploadFailed,
} UploadState;

/* ------------------------------------------------------------ button mapping
 * Copied VERBATIM from PandoraARM (main.c FK_* / FK_TO_BTN):
 *   Flipper UP    -> keyfob button 6 (B6)
 *   Flipper OK    -> keyfob button 5 (B5)
 *   Flipper DOWN  -> keyfob button 4 (BACK)
 *   Flipper LEFT  -> keyfob button 2 (OK)
 *   Flipper RIGHT -> keyfob button 1 (UP)
 *   Flipper BACK  -> keyfob button 3 (DOWN)
 * UP held > 1s = EXIT the app (keyfob button 6 has no long-press conflict).
 *
 * The values below are the PROTO_BTN_* bits for the LOGICAL keyfob button each
 * physical Flipper key maps to. */

/* ------------------------------------------------------------ app state */
typedef struct {
    Gui* gui;
    ViewPort* view_port;
    FuriMessageQueue* input_queue;

    Storage* storage;
    DialogsApp* dialogs;

    PandoraEsp32Uart* uart;
    PandoraEsp32Cc1101* cc1101;

    FuriMutex* fb_mutex;
    uint8_t oled_fb[OLED_FB_SIZE]; /* latest framebuffer from the ESP32 (2bpp) */
    bool have_fb;

    /* current button bitmask (PROTO_BTN_* of keyfob logical buttons) */
    uint8_t btn_mask;
    uint8_t last_sent_mask;

    volatile bool fw_acked;
    volatile bool pong_seen;
    volatile UploadState upload_state;
    volatile uint32_t upload_total;
    volatile uint32_t upload_sent;
    volatile uint32_t esp_hex_seen;
    volatile bool ui_dirty;
    RxEdge rx_edge_ring[RX_EDGE_RING_LEN];
    volatile uint16_t rx_edge_head;
    volatile uint16_t rx_edge_tail;
    uint16_t rx_drain_until;
    volatile uint32_t rx_edge_dropped;
    volatile bool rf_rx_active;
    bool rx_rssi_valid;
    bool rx_gate_open;
    bool rx_drain_pending;
    float rx_noise_floor;
    uint32_t rx_gate_hold_until;
    uint32_t rx_burst_edges;
    uint32_t last_rx_stats;
    uint32_t last_ui_update;
    bool exit_requested;
    bool speaker_owned;
    bool speaker_playing;
    volatile bool beep_requested;
    uint32_t speaker_until;
    uint32_t link_deadline;
    uint32_t next_ping;
    uint32_t ack_deadline;
    uint8_t ping_attempts;
    bool upload_started;
    bool fb_requested;
} AppState;

/* ------------------------------------------------------------ input events
 * We push raw InputEvent into a queue from the ViewPort input callback and
 * process them in the main loop (holds tracked with furi_get_tick). */

/* Map a Flipper InputKey to the keyfob logical button bit (PROTO_BTN_*). */
static uint8_t flipper_key_to_btn(InputKey key) {
    switch(key) {
    case InputKeyUp: return PROTO_BTN_B6; /* Flipper UP    -> keyfob 6 (B6) */
    case InputKeyOk: return PROTO_BTN_B5; /* Flipper OK    -> keyfob 5 (B5) */
    case InputKeyDown: return PROTO_BTN_BACK; /* Flipper DOWN  -> keyfob 4 (BACK) */
    case InputKeyLeft: return PROTO_BTN_OK; /* Flipper LEFT  -> keyfob 2 (OK) */
    case InputKeyRight: return PROTO_BTN_UP; /* Flipper RIGHT -> keyfob 1 (UP) */
    case InputKeyBack: return PROTO_BTN_DOWN; /* Flipper BACK  -> keyfob 3 (DOWN) */
    default: return 0;
    }
}

static void input_callback(InputEvent* event, void* ctx) {
    AppState* app = (AppState*)ctx;
    furi_message_queue_put(app->input_queue, event, 0);
}

/* Mirror firmware-generated buzzer activity on the Flipper speaker. Keep
 * speaker ownership short and entirely on the app thread. */
static void beep_start(AppState* app) {
    if(!app->speaker_owned) {
        app->speaker_owned = furi_hal_speaker_acquire(0);
    }
    if(app->speaker_owned) {
        furi_hal_speaker_start(2000.0f, 0.25f);
        app->speaker_playing = true;
        app->speaker_until = furi_get_tick() + 45u;
    }
}

static void beep_pump(AppState* app) {
    if(app->speaker_playing && (int32_t)(furi_get_tick() - app->speaker_until) >= 0) {
        furi_hal_speaker_stop();
        furi_hal_speaker_release();
        app->speaker_playing = false;
        app->speaker_owned = false;
    }
}

/* ------------------------------------------------------------ OLED render
 * Copied from PandoraARM render_oled: collapse the 92x16 2bpp framebuffer to
 * 1bpp (pixel on if popcount of the 2 bits >= 1), render only the 88 visible
 * columns, centered on the 128-wide screen (x_off = 20). */
static void draw_callback(Canvas* canvas, void* ctx) {
    AppState* app = (AppState*)ctx;
    canvas_clear(canvas);

    if(!app->fw_acked) {
        canvas_set_font(canvas, FontPrimary);
        canvas_draw_str_aligned(canvas, 64, 20, AlignCenter, AlignCenter, "[ESP32] Pandora");
        canvas_set_font(canvas, FontSecondary);
        char status[40];
        switch(app->upload_state) {
        case UploadCheckingLink:
            snprintf(status, sizeof(status), "Checking ESP32 link...");
            break;
        case UploadChoosingFile:
            snprintf(status, sizeof(status), "Select firmware .hex");
            break;
        case UploadSending:
            snprintf(status, sizeof(status), "Sending %lu/%lu KB",
                     (unsigned long)(app->upload_sent / 1024u),
                     (unsigned long)((app->upload_total + 1023u) / 1024u));
            break;
        case UploadWaitingAck:
            snprintf(status, sizeof(status), "Waiting for ESP32 boot...");
            break;
        case UploadNoLink:
            snprintf(status, sizeof(status), "ESP32 link timeout");
            break;
        case UploadNoFile:
            snprintf(status, sizeof(status), "No firmware selected");
            break;
        case UploadFailed:
            snprintf(status, sizeof(status), "Upload failed (see log)");
            break;
        case UploadReady:
        default:
            snprintf(status, sizeof(status), "Waiting for ESP32...");
            break;
        }
        canvas_draw_str_aligned(canvas, 64, 38, AlignCenter, AlignCenter, status);
        if(app->upload_state == UploadNoLink) {
            canvas_draw_str_aligned(
                canvas, 64, 53, AlignCenter, AlignCenter, "Hold UP to exit / retry app");
        }
        return;
    }

    furi_mutex_acquire(app->fb_mutex, FuriWaitForever);
    if(!app->have_fb) {
        furi_mutex_release(app->fb_mutex);
        canvas_set_font(canvas, FontSecondary);
        canvas_draw_str_aligned(canvas, 64, 32, AlignCenter, AlignCenter, "Waiting for frame...");
        return;
    }

    const uint8_t* fbv = app->oled_fb;
    const int x_off = (SCREEN_W - OLED_VISIBLE_COLS) / 2; /* = 20 */
    /* Convert the keyfob's vertical-packed 2bpp image to a compact XBM and draw
     * it in one canvas call. Many small box calls kept the ViewPort mutex long
     * enough for view_port_update() to hit its 2-tick lockup warning. */
    uint8_t bitmap[OLED_VISIBLE_COLS * SCREEN_H / 8] = {0};
    for(int y = 0; y < SCREEN_H; y++) {
        for(int col = 0; col < OLED_VISIBLE_COLS; col++) {
            int page = y / 4;
            int bit = (y % 4) * 2;
            uint8_t v = fbv[page * OLED_COLS + col];
            if(((v >> bit) & 3u) != 0) {
                bitmap[y * (OLED_VISIBLE_COLS / 8) + col / 8] |= (uint8_t)(1u << (col % 8));
            }
        }
    }
    canvas_draw_xbm(canvas, x_off, 0, OLED_VISIBLE_COLS, SCREEN_H, bitmap);
    furi_mutex_release(app->fb_mutex);
}

/* ------------------------------------------------------------ UART frame RX
 * Runs on the UART worker thread. Keep handlers short; mark UI state dirty and
 * let the app loop call view_port_update outside the worker. */
static void uart_frame_cb(uint8_t type, const uint8_t* payload, size_t len, void* ctx) {
    AppState* app = (AppState*)ctx;
    switch(type) {
    case PROTO_MSG_FB:
        if(len >= OLED_FB_SIZE) {
            furi_mutex_acquire(app->fb_mutex, FuriWaitForever);
            memcpy(app->oled_fb, payload, OLED_FB_SIZE);
            app->have_fb = true;
            app->ui_dirty = true;
            furi_mutex_release(app->fb_mutex);
        }
        break;

    case PROTO_MSG_RF_MODE:
        if(len >= sizeof(proto_rf_mode_t)) {
            proto_rf_mode_t m;
            memcpy(&m, payload, sizeof(m));
            pandora_esp32_cc1101_set_mode(app->cc1101, m.mode, m.modulation, m.freq_hz);
            app->rf_rx_active = (m.mode == PROTO_RF_RX);
            if(!app->rf_rx_active) {
                app->rx_gate_open = false;
                app->rx_rssi_valid = false;
                app->rx_drain_pending = false;
                app->rx_edge_tail = app->rx_edge_head;
            }
        }
        break;

    case PROTO_MSG_RF_TX:
        pandora_esp32_cc1101_tx_edges(app->cc1101, payload, len);
        break;

    case PROTO_MSG_LOG: {
        /* ESP32 debug line (not NUL-terminated on the wire). */
        char line[128];
        size_t n = len < sizeof(line) - 1 ? len : sizeof(line) - 1;
        memcpy(line, payload, n);
        line[n] = 0;
        FURI_LOG_I(TAG, "ESP32: %s", line);
        break;
    }

    case PROTO_MSG_PONG:
        if(len) {
            FURI_LOG_I(TAG, "ESP32 PONG: %.*s", (int)len, (const char*)payload);
        } else {
            FURI_LOG_I(TAG, "ESP32 PONG");
        }
        app->pong_seen = true;
        app->ui_dirty = true;
        break;

    case PROTO_MSG_FW_ACK:
        if(len >= sizeof(proto_fw_ack_t)) {
            proto_fw_ack_t a;
            memcpy(&a, payload, sizeof(a));
            FURI_LOG_I(TAG, "FW_ACK ok=%u flat=%lu", a.ok, (unsigned long)a.flat_size);
            app->fw_acked = (a.ok != 0);
            app->upload_state = a.ok ? UploadReady : UploadFailed;
        } else {
            app->fw_acked = true; /* tolerate a bare ack */
            app->upload_state = UploadReady;
        }
        app->ui_dirty = true;
        break;

    case PROTO_MSG_FW_PROGRESS:
        if(len >= sizeof(proto_fw_progress_t)) {
            proto_fw_progress_t progress;
            memcpy(&progress, payload, sizeof(progress));
            app->esp_hex_seen = progress.hex_bytes_seen;
            FURI_LOG_I(TAG, "ESP32 parsed %lu/%lu bytes (flat 0x%lX)",
                       (unsigned long)progress.hex_bytes_seen,
                       (unsigned long)app->upload_total,
                       (unsigned long)progress.flat_bytes);
            app->ui_dirty = true;
        }
        break;

    case PROTO_MSG_BEEP:
        app->beep_requested = true;
        break;

    default:
        FURI_LOG_D(TAG, "unknown frame type 0x%02X len=%u", type, (unsigned)len);
        break;
    }
}

/* ------------------------------------------------------------ CC1101 RX edge
 * Runs in the subghz RX IRQ. Forward the edge to the ESP32 as RX_EDGE. */
static void cc1101_rx_edge_cb(bool level, uint32_t duration_us, void* ctx) {
    AppState* app = (AppState*)ctx;
    uint16_t head = app->rx_edge_head;
    uint16_t next = (head + 1u) & (RX_EDGE_RING_LEN - 1u);
    if(next == app->rx_edge_tail) {
        app->rx_edge_dropped++;
        return;
    }
    app->rx_edge_ring[head].level = level ? 1u : 0u;
    app->rx_edge_ring[head].duration_us = duration_us;
    app->rx_edge_head = next;
}

/* Move captured edges out of IRQ context and amortize UART framing overhead. */
static void pump_rx_edge_batch(AppState* app) {
    static uint32_t reported_drops = 0;
    uint8_t payload[sizeof(uint16_t) + RX_EDGE_BATCH * sizeof(proto_rf_edge_t)];
    uint16_t count = 0;
    uint16_t tail = app->rx_edge_tail;
    uint16_t head = app->rx_edge_head;

    /* Keep only a short pre-roll while the RSSI gate is closed. This avoids
     * forwarding idle-channel noise over the comparatively slow UART while
     * retaining the leading sync edges when a real keyfob burst opens the gate. */
    if(!app->rx_gate_open && !app->rx_drain_pending) {
        uint16_t queued = (head - tail) & (RX_EDGE_RING_LEN - 1u);
        if(queued > RX_PREROLL_EDGES) {
            tail = (head - RX_PREROLL_EDGES) & (RX_EDGE_RING_LEN - 1u);
            app->rx_edge_tail = tail;
        }
        uint32_t dropped = app->rx_edge_dropped;
        if(dropped != reported_drops &&
           (uint32_t)(furi_get_tick() - app->last_rx_stats) >= 1000u) {
            FURI_LOG_W(TAG, "RX idle ring overrun total=%lu (RSSI gate closed)",
                       (unsigned long)dropped);
            reported_drops = dropped;
            app->last_rx_stats = furi_get_tick();
        }
        return;
    }

    uint16_t stop_at = app->rx_gate_open ? head : app->rx_drain_until;
    while(tail != stop_at && count < RX_EDGE_BATCH) {
        RxEdge src = app->rx_edge_ring[tail];
        proto_rf_edge_t edge = {.level = src.level, .duration_us = src.duration_us};
        memcpy(payload + sizeof(uint16_t) + count * sizeof(edge), &edge, sizeof(edge));
        tail = (tail + 1u) & (RX_EDGE_RING_LEN - 1u);
        count++;
    }
    if(!count) return;
    payload[0] = (uint8_t)count;
    payload[1] = (uint8_t)(count >> 8);
    app->rx_edge_tail = tail;
    pandora_esp32_uart_send(app->uart, PROTO_MSG_RX_EDGES, payload,
                            sizeof(uint16_t) + count * sizeof(proto_rf_edge_t));
    app->rx_burst_edges += count;
    if(!app->rx_gate_open && tail == app->rx_drain_until) {
        app->rx_drain_pending = false;
        FURI_LOG_I(TAG, "RX burst drained sent_edges=%lu dropped=%lu",
                   (unsigned long)app->rx_burst_edges,
                   (unsigned long)app->rx_edge_dropped);
    }
    uint32_t dropped = app->rx_edge_dropped;
    if(dropped != reported_drops &&
       (uint32_t)(furi_get_tick() - app->last_rx_stats) >= 1000u) {
        FURI_LOG_W(TAG, "CC1101 RX edge queue dropped total=%lu", (unsigned long)dropped);
        reported_drops = dropped;
        app->last_rx_stats = furi_get_tick();
    }
}

/* Sample RSSI in thread context (the CC1101 callback runs from the radio IRQ).
 * Adaptive hysteresis opens above the measured noise floor and holds long
 * enough to include the complete remote frame. */
static void update_rx_rssi_gate(AppState* app) {
    if(!app->rf_rx_active) return;
    float rssi = furi_hal_subghz_get_rssi();
    uint32_t now = furi_get_tick();
    if(!app->rx_rssi_valid) {
        app->rx_noise_floor = rssi;
        app->rx_rssi_valid = true;
        FURI_LOG_I(TAG, "RX RSSI baseline %.1f dBm", (double)rssi);
        return;
    }

    if(!app->rx_gate_open) {
        /* Track ambient level slowly; exclude positive signal excursions. */
        if(rssi < app->rx_noise_floor + RX_GATE_OPEN_DB) {
            app->rx_noise_floor = app->rx_noise_floor * 0.94f + rssi * 0.06f;
        }
        if(rssi >= app->rx_noise_floor + RX_GATE_OPEN_DB) {
            app->rx_gate_open = true;
            app->rx_gate_hold_until = now + RX_GATE_HOLD_MS;
            app->rx_burst_edges = 0;
            FURI_LOG_I(TAG, "RX RSSI gate OPEN rssi=%.1f floor=%.1f preroll=%u",
                       (double)rssi, (double)app->rx_noise_floor,
                       (unsigned)RX_PREROLL_EDGES);
        }
    } else if(rssi >= app->rx_noise_floor + RX_GATE_CLOSE_DB) {
        app->rx_gate_hold_until = now + RX_GATE_HOLD_MS;
    } else if((int32_t)(now - app->rx_gate_hold_until) >= 0) {
        app->rx_gate_open = false;
        app->rx_drain_until = app->rx_edge_head;
        app->rx_drain_pending = app->rx_edge_tail != app->rx_drain_until;
        FURI_LOG_I(TAG, "RX RSSI gate CLOSE rssi=%.1f floor=%.1f queued=%u",
                   (double)rssi, (double)app->rx_noise_floor,
                   (unsigned)((app->rx_drain_until - app->rx_edge_tail) &
                              (RX_EDGE_RING_LEN - 1u)));
        if(!app->rx_drain_pending) {
            FURI_LOG_I(TAG, "RX burst done sent_edges=%lu dropped=%lu",
                       (unsigned long)app->rx_burst_edges,
                       (unsigned long)app->rx_edge_dropped);
        }
    }
}

/* ------------------------------------------------------------ firmware upload
 * Pick a .hex from the SD (file browser if needed), stream it to the ESP32. */

/* Deduce the FW profile from the file name. Per the protocol, profile 0 is
 * PANDORA_MAX, 1 is PANDORA. So "MAX" in the name => 0, otherwise => 1. */
static uint8_t deduce_profile(const char* path) {
    if(strstr(path, "MAX") || strstr(path, "max") || strstr(path, "Max")) {
        return PROTO_FW_PROFILE_PANDORA_MAX; /* 0 */
    }
    return PROTO_FW_PROFILE_PANDORA; /* 1 */
}

/* Returns true if a .hex was selected + streamed. */
static bool upload_firmware(AppState* app) {
    FuriString* path = furi_string_alloc();
    bool ok = false;

    /* Default to the pandora firmware directory; fall back to /ext. */
    app->upload_state = UploadChoosingFile;
    FURI_LOG_I(TAG, "firmware picker: updating status view");
    if(app->view_port) view_port_update(app->view_port);
    if(storage_dir_exists(app->storage, PANDORA_FW_DIR)) {
        furi_string_set(path, PANDORA_FW_DIR);
    } else {
        furi_string_set(path, "/ext");
    }

    DialogsFileBrowserOptions opts;
    dialog_file_browser_set_basic_options(&opts, ".hex", NULL);
    opts.base_path = furi_string_get_cstr(path);
    opts.hide_ext = false;
    /* The app viewport is on GuiLayerFullscreen. Leaving it attached hides
     * the modal file browser, making the app look stuck on its upload screen. */
    FURI_LOG_I(TAG, "opening firmware file browser");
    gui_remove_view_port(app->gui, app->view_port);
    bool selected = dialog_file_browser_show(app->dialogs, path, path, &opts);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);
    view_port_update(app->view_port);
    FURI_LOG_I(TAG, "firmware file browser closed (selected=%u)", selected);
    if(!selected) {
        FURI_LOG_W(TAG, "no firmware selected");
        app->upload_state = UploadNoFile;
        if(app->view_port) view_port_update(app->view_port);
        furi_string_free(path);
        return false;
    }

    const char* cpath = furi_string_get_cstr(path);
    File* f = storage_file_alloc(app->storage);
    if(!f) {
        FURI_LOG_E(TAG, "firmware file handle allocation failed");
        app->upload_state = UploadFailed;
        furi_string_free(path);
        return false;
    }
    do {
        if(!storage_file_open(f, cpath, FSAM_READ, FSOM_OPEN_EXISTING)) {
            FURI_LOG_E(TAG, "cannot open %s", cpath);
            app->upload_state = UploadFailed;
            break;
        }
        uint64_t total = storage_file_size(f);
        app->upload_total = (uint32_t)total;
        app->upload_sent = 0;
        app->esp_hex_seen = 0;
        uint8_t profile = deduce_profile(cpath);
        FURI_LOG_I(
            TAG, "upload %s (%lu bytes, profile %u)", cpath, (unsigned long)total, profile);

        /* FW_BEGIN */
        proto_fw_begin_t begin = {
            .total_hex_bytes = (uint32_t)total,
            .profile = profile,
        };
        pandora_esp32_uart_send(
            app->uart, PROTO_MSG_FW_BEGIN, (const uint8_t*)&begin, sizeof(begin));
        app->upload_state = UploadSending;
        if(app->view_port) view_port_update(app->view_port);

        /* FW_CHUNK* */
        uint8_t* chunk = malloc(FW_CHUNK_BYTES);
        if(!chunk) {
            FURI_LOG_E(TAG, "firmware upload buffer allocation failed");
            app->upload_state = UploadFailed;
            break;
        }
        size_t rd;
        uint64_t sent = 0;
        while((rd = storage_file_read(f, chunk, FW_CHUNK_BYTES)) > 0) {
            pandora_esp32_uart_send(app->uart, PROTO_MSG_FW_CHUNK, chunk, rd);
            sent += rd;
            app->upload_sent = (uint32_t)sent;
            if((sent & 0x3FFFu) < FW_CHUNK_BYTES && app->view_port)
                view_port_update(app->view_port);
            /* Frames fit in the ESP32's 4096-byte UART RX ring. The UART TX
             * call paces at line rate; an extra sleep only slows the upload. */
        }
        free(chunk);
        if(storage_file_get_error(f) != FSE_OK) {
            FURI_LOG_E(TAG, "firmware read failed after %lu bytes", (unsigned long)sent);
            app->upload_state = UploadFailed;
            break;
        }

        /* FW_END */
        pandora_esp32_uart_send(app->uart, PROTO_MSG_FW_END, NULL, 0);
        FURI_LOG_I(TAG, "firmware streamed (%lu bytes)", (unsigned long)sent);
        ok = true;
    } while(false);

    storage_file_close(f);
    storage_file_free(f);
    furi_string_free(path);
    return ok;
}

/* ------------------------------------------------------------ main */
int32_t pandora_esp32_app(void* p) {
    UNUSED(p);

    AppState* app = malloc(sizeof(AppState));
    memset(app, 0, sizeof(AppState));

    app->storage = furi_record_open(RECORD_STORAGE);
    app->dialogs = furi_record_open(RECORD_DIALOGS);
    app->gui = furi_record_open(RECORD_GUI);

    app->fb_mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->input_queue = furi_message_queue_alloc(16, sizeof(InputEvent));

    /* GUI: a direct ViewPort (no complex scene system). */
    app->view_port = view_port_alloc();
    view_port_draw_callback_set(app->view_port, draw_callback, app);
    view_port_input_callback_set(app->view_port, input_callback, app);
    gui_add_view_port(app->gui, app->view_port, GuiLayerFullscreen);

    /* UART transport + protocol parser (marauder pattern). */
    app->uart = pandora_esp32_uart_init(uart_frame_cb, app);
    if(!app->uart) {
        FURI_LOG_E(TAG, "UART init failed");
        app->upload_state = UploadFailed;
        gui_remove_view_port(app->gui, app->view_port);
        view_port_free(app->view_port);
        furi_message_queue_free(app->input_queue);
        furi_mutex_free(app->fb_mutex);
        furi_record_close(RECORD_GUI);
        furi_record_close(RECORD_DIALOGS);
        furi_record_close(RECORD_STORAGE);
        free(app);
        return 0;
    }

    /* CC1101 radio bridge (PandoraARM pattern). */
    app->cc1101 = pandora_esp32_cc1101_alloc(cc1101_rx_edge_cb, app);

    /* Keep link probing inside the normal app loop. The previous synchronous
     * handshake left the viewport and input queue unserviced while waiting for
     * UART; on some Flipper builds that looked like a permanent freeze. */
    app->upload_state = UploadCheckingLink;
    uint32_t link_started = furi_get_tick();
    app->link_deadline = link_started + 5000u;
    app->next_ping = link_started;

    /* PHASE 2: event loop. Track per-key holds like PandoraARM. */
    uint32_t up_press_t0 = 0;
    bool up_down = false;
    bool up_consumed = false;

    while(!app->exit_requested) {
        uint32_t now = furi_get_tick();
        if(app->upload_state == UploadCheckingLink) {
            if(app->pong_seen) {
                FURI_LOG_I(TAG, "ESP32 link ready after %u probe(s)",
                           (unsigned)app->ping_attempts);
                app->upload_state = UploadChoosingFile;
                app->ui_dirty = true;
            } else if((int32_t)(now - app->link_deadline) >= 0) {
                FURI_LOG_E(TAG,
                           "ESP32 PONG timeout after %u probes; check ESP32 power, UART TX/RX and GND",
                           (unsigned)app->ping_attempts);
                app->upload_state = UploadNoLink;
                app->ui_dirty = true;
            } else if((int32_t)(now - app->next_ping) >= 0) {
                pandora_esp32_uart_send(app->uart, PROTO_MSG_PING, NULL, 0);
                app->ping_attempts++;
                FURI_LOG_D(TAG, "ESP32 link probe #%u", (unsigned)app->ping_attempts);
                app->next_ping = now + 500u;
            }
        }

        /* Start the modal firmware picker only after PONG, from the app thread.
         * It runs once; link probing and timeout remain responsive before then. */
        if(app->pong_seen && !app->upload_started) {
            app->upload_started = true;
            if(upload_firmware(app)) {
                app->upload_state = UploadWaitingAck;
                app->ack_deadline = furi_get_tick() + FW_ACK_TIMEOUT_MS;
            } else {
                FURI_LOG_W(TAG, "firmware upload skipped/failed; ESP32 may hold an image");
            }
            app->ui_dirty = true;
        }

        if(app->upload_state == UploadWaitingAck && !app->fw_acked &&
           (int32_t)(now - app->ack_deadline) >= 0) {
            FURI_LOG_E(TAG, "FW_ACK timeout after upload (%lu/%lu ESP32 parsed)",
                       (unsigned long)app->esp_hex_seen,
                       (unsigned long)app->upload_total);
            app->upload_state = UploadFailed;
            app->ui_dirty = true;
        }

        if(app->fw_acked && !app->fb_requested) {
            pandora_esp32_uart_send(app->uart, PROTO_MSG_REQ_FB, NULL, 0);
            app->fb_requested = true;
        }

        InputEvent event;
        if(furi_message_queue_get(app->input_queue, &event, 5) == FuriStatusOk) {
            FURI_LOG_I(TAG, "input key=%u type=%u", (unsigned)event.key, (unsigned)event.type);
            uint8_t bit = flipper_key_to_btn(event.key);

            /* Flipper UP long-hold = EXIT (host-side). Track it specially. */
            if(event.key == InputKeyUp) {
                if(event.type == InputTypePress) {
                    up_down = true;
                    up_consumed = false;
                    up_press_t0 = furi_get_tick();
                } else if(event.type == InputTypeRelease) {
                    up_down = false;
                }
            }

            /* Button bitmask -> ESP32. Press sets the bit, release clears it. */
            if(bit) {
                if(event.type == InputTypePress) {
                    app->btn_mask |= bit;
                } else if(event.type == InputTypeRelease) {
                    app->btn_mask &= (uint8_t)~bit;
                }
            }
        }

        /* UP held > 1s => EXIT (consumed once). */
        if(up_down && !up_consumed && (furi_get_tick() - up_press_t0) >= EXIT_HOLD_MS) {
            app->exit_requested = true;
            up_consumed = true;
        }

        /* Send the button mask to the ESP32 only when it changes. */
        if(app->btn_mask != app->last_sent_mask) {
            FURI_LOG_I(TAG, "BTN send mask=0x%02X", app->btn_mask);
            pandora_esp32_uart_send(app->uart, PROTO_MSG_BTN, &app->btn_mask, 1);
            app->last_sent_mask = app->btn_mask;
        }

        /* UART callbacks only mark the image dirty. Update the viewport from
         * this app thread, avoiding ViewPort lockups from the UART worker. */
        if(app->ui_dirty &&
           (uint32_t)(furi_get_tick() - app->last_ui_update) >= RX_UI_MIN_PERIOD_MS) {
            app->ui_dirty = false;
            view_port_update(app->view_port);
            app->last_ui_update = furi_get_tick();
        }

        update_rx_rssi_gate(app);
        if(app->beep_requested) {
            app->beep_requested = false;
            beep_start(app);
        }
        beep_pump(app);
        /* Service any in-flight CC1101 async TX. */
        pump_rx_edge_batch(app);
        pandora_esp32_cc1101_pump(app->cc1101);
    }

    /* --- teardown --- */
    /* Do not leave a key logically held on the ESP32 when the frontend exits. */
    app->btn_mask = 0;
    pandora_esp32_uart_send(app->uart, PROTO_MSG_BTN, &app->btn_mask, 1);
    /* Stop the remote MCU loop before closing UART, so the next launch starts
     * from an idle ESP32 instead of inheriting the old firmware execution. */
    pandora_esp32_uart_send(app->uart, PROTO_MSG_STOP, NULL, 0);
    furi_delay_ms(40);
    FURI_LOG_I(TAG, "exit requested; released buttons and stopped ESP32 emulator");
    if(app->speaker_playing) {
        furi_hal_speaker_stop();
        app->speaker_playing = false;
    }
    if(app->speaker_owned) {
        furi_hal_speaker_release();
        app->speaker_owned = false;
    }
    gui_remove_view_port(app->gui, app->view_port);
    view_port_free(app->view_port);

    pandora_esp32_cc1101_free(app->cc1101);
    pandora_esp32_uart_free(app->uart);

    furi_message_queue_free(app->input_queue);
    furi_mutex_free(app->fb_mutex);

    furi_record_close(RECORD_GUI);
    furi_record_close(RECORD_DIALOGS);
    furi_record_close(RECORD_STORAGE);
    free(app);
    return 0;
}
