# [ESP32] Pandora

Flipper Zero **companion** app for an **ESP32-WROOM-32** running the Pandora
D-605 keyfob emulator.

## What it does

The heavy emulation (the REAL keyfob firmware) runs on the ESP32. The Flipper is
just the **display + keypad + radio** front-end:

1. **Firmware upload** — on start the app reads a keyfob firmware `.hex` from the
   Flipper SD and streams it to the ESP32 over UART (`FW_BEGIN` / `FW_CHUNK*` /
   `FW_END`). The ESP32 flattens + boots it and replies `FW_ACK`.
2. **Display** — the ESP32 streams the keyfob OLED framebuffer (`PROTO_MSG_FB`,
   92×16 @ 2bpp). The Flipper renders the 88 visible columns, centered on the
   128×64 screen.
3. **Keypad** — the Flipper D-pad is forwarded to the ESP32 as a button bitmask
   (`PROTO_MSG_BTN`).
4. **Radio** — the ESP32 tells the Flipper the RF state (`PROTO_MSG_RF_MODE`) and
   sends ready-made TX waveforms (`PROTO_MSG_RF_TX`) which the Flipper transmits
   on the internal **CC1101**. Edges received off the air are sent back to the
   ESP32 (`PROTO_MSG_RX_EDGE`). Default band: **433.92 MHz**.

## Wiring (UART)

The app uses `FuriHalSerialIdUsart` at **230400 8N1**. The ESP32 companion
firmware must be rebuilt and flashed with the matching baud rate.

| ESP32              | Flipper            |
|--------------------|--------------------|
| GPIO1 / TX0  →     | RX (pin 14, `RX`)  |
| GPIO3 / RX0  ←     | TX (pin 13, `TX`)  |
| GND                | GND (pin 8/11/18)  |
| 3V3 (or 5V/VBUS)   | 3V3 (pin 9) / 5V   |

> Cross TX↔RX. Share a common ground. Power the ESP32 from 3V3/5V as your board
> requires.

## Firmware images on the SD

Put your keyfob `.hex` files in:

```
/ext/apps_data/pandora/
```

On start the app opens a file browser there (falls back to `/ext` if the folder
is missing) filtered to `*.hex`. The **profile** is deduced from the file name:

- name contains `MAX` → profile `0` (`PANDORA_MAX`)
- otherwise           → profile `1` (`PANDORA`)

(Per the shared protocol, `0 = PANDORA_MAX`, `1 = PANDORA`.)

## Button map (Flipper D-pad → keyfob logical button)

Copied from the PandoraARM native emulator so the D-pad matches the keyfob
visually:

| Flipper key | Keyfob button |
|-------------|---------------|
| UP          | 6 (B6)        |
| OK          | 5 (B5)        |
| DOWN        | 4 (BACK)      |
| LEFT        | 2 (OK)        |
| RIGHT       | 1 (UP)        |
| BACK        | 3 (DOWN)      |

- **Hold UP > 1 s** = exit the app (host-side). This works both on the
  firmware screen and after an ESP32 link timeout; after a timeout, relaunch
  once the ESP32/UART link is ready.

The ESP32 backend must be rebuilt after updating `esp32-pandora/main/main.c`.
It converts each newly pressed button bit into the emulator's scanned key
cycle (`emu_nav_button`) and releases the virtual GPIO afterward. This keeps
short Flipper taps from falling between MCU execution bursts.

## Protocol

The UART wire format is defined in `pandora_esp32_protocol.h`, a verbatim copy of
`esp32-pandora/main/protocol.h` so both sides share the exact same framing:

```
SYNC(0xA5) | TYPE | LEN(2 LE) | PAYLOAD | CRC8(poly 0x07 over TYPE+LEN+PAYLOAD)
```

## Build

```
cd Flipper-ARF
./fbt fap_pandora_esp32
```
