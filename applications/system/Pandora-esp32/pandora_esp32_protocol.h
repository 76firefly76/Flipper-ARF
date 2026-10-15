/*
 * pandora_esp32_protocol.h - Shared UART framing between the ESP32 (emulator
 *              backend) and the Flipper Zero (display + keypad + radio frontend).
 *
 * This is a VERBATIM COPY of esp32-pandora/main/protocol.h so BOTH SIDES use the
 * exact same wire format. The only additions over the base protocol are the
 * firmware-transfer messages (FW_BEGIN / FW_CHUNK / FW_END / FW_ACK), which the
 * ESP32-side header gains in parallel. Keep the two files byte-for-byte
 * compatible on the constants below.
 *
 * It only declares constants and the frame layout; no platform-specific types
 * are used (only <stdint.h>), so it compiles on both the ESP32 firmware and the
 * Flipper companion app.
 *
 * WIRE FORMAT (every frame, both directions):
 *
 *     +------+------+------+-----------------+------+
 *     | SYNC | TYPE | LEN  |   PAYLOAD[LEN]  | CRC8 |
 *     +------+------+------+-----------------+------+
 *       0xA5   1B     2B (LE)   LEN bytes       1B
 *
 *   - SYNC  : 0xA5 start-of-frame marker (lets the receiver resynchronise).
 *   - TYPE  : one of PROTO_* below.
 *   - LEN   : payload length, little-endian uint16 (0..PROTO_MAX_PAYLOAD).
 *   - CRC8  : CRC-8 (poly 0x07, init 0x00) over TYPE + LEN(2) + PAYLOAD.
 *
 * The CRC does NOT cover the SYNC byte (the SYNC is only a resync aid).
 */
#ifndef PANDORA_PROTOCOL_H
#define PANDORA_PROTOCOL_H

#include <stdint.h>

#define PROTO_SYNC 0xA5u

/* Keep this >= the OLED framebuffer (1472B) plus some slack for RF_TX edges. */
#define PROTO_MAX_PAYLOAD 2048u

/* OLED framebuffer geometry (keyfob UC16xx/ST7528): 92 columns x 16 pages. */
#define PROTO_OLED_COLS 92u
#define PROTO_OLED_PAGES 16u
#define PROTO_FB_SIZE (PROTO_OLED_COLS * PROTO_OLED_PAGES) /* 1472 bytes */

/* -------------------------------------------------------- ESP32 -> Flipper -- */
/* Full framebuffer dump (payload = PROTO_FB_SIZE raw bytes, page-addressed,
 * 2bpp, bit0 = top row). The Flipper renders it on its 128x64 screen. */
#define PROTO_MSG_FB 0x01u
/* Radio state. payload: struct proto_rf_mode (see below). */
#define PROTO_MSG_RF_MODE 0x02u
/* RF waveform to transmit. payload: uint16 count + count * proto_rf_edge. */
#define PROTO_MSG_RF_TX 0x03u
/* Debug log line. payload: ASCII text (not NUL-terminated on the wire). */
#define PROTO_MSG_LOG 0x04u
/* Handshake / keepalive reply. payload: ASCII banner. */
#define PROTO_MSG_PONG 0x05u
/* Firmware-transfer acknowledge (ESP32 booted the uploaded image).
 * payload: struct proto_fw_ack (see below). */
#define PROTO_MSG_FW_ACK 0x06u
/* Firmware-transfer progress (optional/informational).
 * payload: struct proto_fw_progress (see below). */
#define PROTO_MSG_FW_PROGRESS 0x07u
#define PROTO_MSG_BEEP 0x08u

/* -------------------------------------------------------- Flipper -> ESP32 -- */
/* Button state. payload: 1 byte bitmask (PROTO_BTN_*). */
#define PROTO_MSG_BTN 0x81u
/* A received RF edge from the air. payload: proto_rf_edge. Injected into PB0. */
#define PROTO_MSG_RX_EDGE 0x82u
/* Handshake / keepalive request. payload: optional ASCII. ESP32 replies PONG. */
#define PROTO_MSG_PING 0x83u
/* Request a full framebuffer resend (e.g. on Flipper (re)connect). */
#define PROTO_MSG_REQ_FB 0x84u
/* Reset + re-unlock the emulator. payload: none. */
#define PROTO_MSG_RESET 0x85u
/* Firmware-transfer begin. payload: struct proto_fw_begin (see below). */
#define PROTO_MSG_FW_BEGIN 0x86u
/* Firmware-transfer chunk. payload: raw bytes of the .hex text. */
#define PROTO_MSG_FW_CHUNK 0x87u
/* Firmware-transfer end (no payload). ESP32 flattens + boots, then FW_ACK. */
#define PROTO_MSG_FW_END 0x88u
/* Batched CC1101 RX edges: uint16 count followed by count proto_rf_edge_t. */
#define PROTO_MSG_RX_EDGES 0x89u
/* Stop/reset the remote emulator when the Flipper companion exits. */
#define PROTO_MSG_STOP 0x8Au

/* ------------------------------------------------------------ button bits --- */
/* One bit per keyfob logical button. Matches the emulator's B_* enum order. */
#define PROTO_BTN_UP (1u << 0)
#define PROTO_BTN_DOWN (1u << 1)
#define PROTO_BTN_OK (1u << 2)
#define PROTO_BTN_BACK (1u << 3)
#define PROTO_BTN_B5 (1u << 4)
#define PROTO_BTN_B6 (1u << 5)

/* ------------------------------------------------------------ RF state ------ */
/* Mirrors the Si4432 operating mode derived in the emulator. */
#define PROTO_RF_IDLE 0u
#define PROTO_RF_READY 1u
#define PROTO_RF_RX 2u
#define PROTO_RF_TX 3u

/* Radio modulation (for display/diagnostics on the Flipper). */
#define PROTO_MOD_UNMOD 0u
#define PROTO_MOD_OOK 1u
#define PROTO_MOD_FSK 2u
#define PROTO_MOD_GFSK 3u

/* ------------------------------------------------------------ FW profile ---- */
/* Which keyfob variant the uploaded .hex belongs to. NOTE the ordering is
 * historical: 0 = PANDORA_MAX, 1 = PANDORA. The ESP32 header spells the first
 * macro PROTO_FW_PROFILE_PANDORAMAX; keep both names pointing at the same value
 * so either spelling compiles. */
#define PROTO_FW_PROFILE_PANDORAMAX 0u
#define PROTO_FW_PROFILE_PANDORA_MAX 0u
#define PROTO_FW_PROFILE_PANDORA 1u

/* Packed to keep the wire layout identical on both compilers. */
#if defined(__GNUC__)
#define PROTO_PACKED __attribute__((packed))
#else
#define PROTO_PACKED
#endif

typedef struct PROTO_PACKED {
    uint8_t mode; /* PROTO_RF_* */
    uint8_t modulation; /* PROTO_MOD_* */
    uint32_t freq_hz; /* carrier frequency in Hz (0 if unknown) */
} proto_rf_mode_t;

typedef struct PROTO_PACKED {
    uint8_t level; /* 0 = low, 1 = high */
    uint32_t duration_us; /* edge duration in microseconds */
} proto_rf_edge_t;

/* FW_BEGIN payload: announces a firmware upload. */
typedef struct PROTO_PACKED {
    uint32_t total_hex_bytes; /* total number of .hex text bytes to follow */
    uint8_t profile; /* PROTO_FW_PROFILE_* */
} proto_fw_begin_t;

/* FW_END payload (optional; LEN may be 0 to skip the CRC check). */
typedef struct PROTO_PACKED {
    uint32_t crc32; /* CRC-32 of the full .hex text, or 0 if not provided */
} proto_fw_end_t;

/* FW_ACK payload: ESP32 reply once it flattened + booted the uploaded image. */
typedef struct PROTO_PACKED {
    uint8_t ok; /* 1 = booted ok, 0 = failure */
    uint32_t flat_size; /* flattened flash size in bytes (diagnostic) */
} proto_fw_ack_t;

/* FW_PROGRESS payload (ESP32 -> Flipper, optional/informational). */
typedef struct PROTO_PACKED {
    uint32_t hex_bytes_seen; /* raw .hex bytes consumed so far */
    uint32_t flat_bytes; /* highest flat address written so far + 1 */
} proto_fw_progress_t;

/* ------------------------------------------------------------ CRC-8 --------- */
/* CRC-8, polynomial 0x07, init 0x00 (a.k.a. CRC-8/SMBUS without final XOR).
 * Small, branch-light, header-only so both sides share the exact same impl. */
static inline uint8_t proto_crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0x00u;
    for(uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for(int b = 0; b < 8; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

#endif /* PANDORA_PROTOCOL_H */
