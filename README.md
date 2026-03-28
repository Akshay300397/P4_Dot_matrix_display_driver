# HUB75 LED Matrix Driver — ESP32-S3 / ESP-IDF v5.x

Bare-metal HUB75 driver for **2× P4 64×32 FM6124EJ** panels chained to a **128×32** total display.  
Built with ESP32-S3 and ESP-IDF v5.x. Receives display commands over UART from an Ethernet-to-serial board.

---

## Features

- **Dual-panel 128×32** display via single SPI chain
- **FM6124EJ** driver IC initialization (REG1 brightness + REG2 enable — required at boot)
- **Double-buffered framebuffer** — flicker-free updates with atomic buffer swap
- **1:16 scan rate** with ABCD binary row addressing
- **5×7 bitmap font** with full ASCII support (32–126)
- **UART command protocol** — 11-byte fixed frames, 6 data fields, per-field scrolling
- **FreeRTOS dual-core** — refresh task pinned to Core 1, app logic on Core 0
- **Diagnostic test modes** — color bars, row scan, corner pixels, chain order test

---

## Hardware

| Component       | Specification                          |
|-----------------|----------------------------------------|
| MCU             | ESP32-S3                               |
| Panel           | P4 64×32 2121 16S (×2)                 |
| Driver IC       | FM6124EJ                               |
| Connector       | HUB75                                  |
| Configuration   | 2 panels chained → 128×32 total        |
| Display size    | 512 mm × 128 mm                        |
| Scan rate       | 1:16 (ABCD binary addressing)          |
| Power           | 5V external PSU, minimum 8A            |

---

## Panel Chaining

The two panels are chained via a **single bridge wire** — not the ribbon cable.

```
ESP32-S3 MOSI ──► Panel 1 (P1) PI:R1
                  P1 internal chain: R1 → R2 → G1 → G2 → B1 → B2
                  P1 PO:B2 ──► P2 PI:R1  ← bridge wire
                  P2 internal chain: R1 → R2 → G1 → G2 → B1 → B2
```

**Physical layout:** P2 on the left, P1 on the right.

### PI→PO Jumpers (required on EACH panel)

These connect the 6 internal shift registers into one serial chain:

| PI Pin | → PO Pin |
|--------|----------|
| R2     | ← R1     |
| G1     | ← R2     |
| G2     | ← G1     |
| B1     | ← G2     |
| B2     | ← B1     |

---

## Pin Wiring

### ESP32-S3 → Panel 1 HUB75 (PI connector)

| Signal | GPIO | HUB75 Pin | Notes                     |
|--------|------|-----------|---------------------------|
| MOSI   | 11   | R1        | SPI2 data                 |
| CLK    | 12   | CLK       | SPI2 clock                |
| LAT    | 13   | LAT       | Latch                     |
| OE     | 14   | OE        | Output Enable (active LOW) |
| A      | 1    | A         | Row address bit 0          |
| B      | 2    | B         | Row address bit 1          |
| C      | 3    | C         | Row address bit 2          |
| D      | 4    | D         | Row address bit 3          |
| GND    | GND  | GND       | Common ground              |

### UART (Ethernet-to-Serial Board)

| Signal | GPIO | Direction       |
|--------|------|-----------------|
| RX     | 16   | ESP32 receives  |
| TX     | 17   | ESP32 transmits |
| GND    | GND  | Common ground   |

### Power

- Each panel independently connected to **5V external PSU**
- Minimum rating: **5V 8A** (both panels at full brightness)
- ESP32-S3: powered separately (USB or 3.3V)
- **Common GND** between ESP32-S3, both panels, and PSU

---

## SPI Buffer Layout

The SPI buffer is **96 bytes** (12 planes × 8 bytes). Because SPI shifts MSB-first and the chain fills in reverse, **P2 occupies bytes 0–47 (first in buffer)** and **P1 occupies bytes 48–95 (last in buffer)**. Within each panel, planes are also reversed:

```
Offset 0–7   : P2 B2  ← sent first → arrives at chain end
Offset 8–15  : P2 B1
Offset 16–23 : P2 G2
Offset 24–31 : P2 G1
Offset 32–39 : P2 R2
Offset 40–47 : P2 R1
Offset 48–55 : P1 B2
Offset 56–63 : P1 B1
Offset 64–71 : P1 G2
Offset 72–79 : P1 G1
Offset 80–87 : P1 R2
Offset 88–95 : P1 R1  ← sent last → stays at chain start
```

---

## Project Structure

```
hub75_display/
├── CMakeLists.txt          ← Top-level build config
└── main/
    ├── CMakeLists.txt      ← Component registration
    ├── main_1.c            ← app_main(), startup sequence, task creation
    ├── hub75.h / hub75.c   ← HUB75 SPI driver, FM6124 init, refresh task (Core 1)
    ├── framebuffer.h / .c  ← Double-buffered framebuffer, drawing primitives
    ├── font.h / font.c     ← Font selector and interface
    ├── font_5x7.c          ← 5×7 bitmap font, ASCII 32–126
    ├── display_content.h/c ← Boot patterns, default content, diagnostic tests
    └── uart_test2.c        ← UART receiver and packet parser (Core 0)
```

---

## Build and Flash

```bash
# 1. Set up ESP-IDF environment (v5.2 or newer)
. $IDF_PATH/export.sh

# 2. Set target
idf.py set-target esp32s3

# 3. Build
idf.py build

# 4. Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Boot Sequence

```
app_main()
  ├─ framebuffer_init()          — zero both buffers, create swap mutex
  ├─ hub75_init()                — FM6124 register init, then SPI2 + GPIO
  ├─ xTaskCreatePinnedToCore()   — start hub75_refresh_task on Core 1
  ├─ run_hardware_check()        — 5 test patterns × ~1s each (~6s total)
  └─ draw_default_content()      — static screen, stays until UART updates it
```

---

## UART Packet Protocol

Fixed **11-byte** frames:

```
┌──────┬────────┬─────┬─────────────┬─────┬──────┐
│  *   │  CMD   │ LEN │   PAYLOAD   │ CRC │  #   │
│  1B  │  2B    │  1B │     4B      │  1B │  1B  │
└──────┴────────┴─────┴─────────────┴─────┴──────┘

Total: 1 + 2 + 1 + 4 + 1 + 1 = 11 bytes
CRC = simple byte sum of frame[1..8]
```

### Command Types

| CMD | Description                   |
|-----|-------------------------------|
| C0  | Control command               |
| D1–D6 | Data fields (6 total)       |

### Display Layout

| Parameter       | Value  |
|-----------------|--------|
| Fields          | 6 (D1–D6) |
| Rows            | 2 (3 fields each) |
| X offset        | 26 px  |
| Field spacing   | 34 px  |
| Row 1 Y         | 6      |
| Row 2 Y         | 19     |
| Max chars/field | 4      |
| Font            | 5×7    |

---

## Color Reference (3-bit)

| Color   | Value  | Binary |
|---------|--------|--------|
| BLACK   | 0x00   | 000    |
| RED     | 0x04   | 100    |
| GREEN   | 0x02   | 010    |
| BLUE    | 0x01   | 001    |
| YELLOW  | 0x06   | 110    |
| CYAN    | 0x03   | 011    |
| MAGENTA | 0x05   | 101    |
| WHITE   | 0x07   | 111    |

---

## Diagnostic Functions

Call these from `app_main()` instead of `run_hardware_check()` during debugging:

| Function           | What it does                                           |
|--------------------|--------------------------------------------------------|
| `run_hardware_check()` | 5 patterns: color bars, bands, all colors, corners, text |
| `chain_order_test()` | Steps through R/G/B on top/bottom halves to verify chain order |
| `row_scan_test()`  | Lights one framebuffer row at a time to verify ABCD addressing |

---

## Troubleshooting

| Symptom                  | Likely Cause                   | Fix                                      |
|--------------------------|--------------------------------|------------------------------------------|
| No display at all        | Power or OE issue              | Check 5V PSU, verify OE=GPIO14           |
| Only one panel lights    | Bridge wire missing            | Check P1:PO:B2 → P2:PI:R1 wire          |
| Random dots at boot      | FM6124 not initialized         | Verify `fm6124_init()` runs before SPI   |
| Colors wrong/swapped     | Bit plane order mismatch       | Run `chain_order_test()` and compare     |
| Image mirrored           | SPI bit/byte send direction    | Check `bit_pos = 7 - (x % 8)` in buffer |
| Ghost rows / smearing    | Mux delay too short            | Increase `HUB75_MUX_DELAY_US`           |
| Noise / random pixels    | SPI speed too high             | Reduce `HUB75_SPI_SPEED_HZ` to 10MHz    |
| Display flickering       | Watchdog reset on Core 1       | Set `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n` |
| UART data loss           | Baud rate mismatch             | Verify both sides at 115200              |
| P2 shows P1 content      | Buffer panel order wrong       | P2 bytes must come first in SPI buffer   |

---

## Key Design Notes

1. **HUB75 ribbon carries parallel data, not serial chain.** The ribbon's 6 data lines feed both panels simultaneously. True chaining requires a physical bridge wire (P1 PO → P2 PI), not the ribbon.

2. **SPI buffer ordering is counter-intuitive.** SPI shifts MSB-first and the shift register fills in reverse. The last physical panel (P2, left) must be placed first in the buffer. Within each panel, planes are also reversed (B2 first, R1 last).

3. **FM6124 requires explicit initialization.** Without writing REG1 (brightness) and REG2 (enable output), the panel shows garbage or nothing at all. This must happen before SPI is initialized.

4. **3 CLK pulses while LAT is HIGH = DATA_LATCH on FM6124.** More or fewer pulses trigger different internal commands (11 = write brightness register, 12 = write enable register). This is critical to get right in the refresh loop.

---

## License

MIT
