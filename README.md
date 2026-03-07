# HUB75 LED Matrix Driver — ESP32-S3 / ESP-IDF

Bare-metal HUB75 driver for **2× P4 64×32 ICN6124EJ** panels chained to **128×32** total display.  
Built with ESP32-S3 and ESP-IDF v5.x. Receives display commands via UART from an ethernet-to-serial board.

---

## Hardware

| Component | Specification |
|---|---|
| MCU | ESP32-S3 |
| Panel | P4 64×32 2121 16S JHT2.0 |
| Driver IC | ICN6124EJ |
| Connector | HUB75 |
| Configuration | 2 panels chained = 128×32 total |
| Display size | 512mm × 128mm |
| Scan rate | 1:16 (ABCD binary addressing) |

---

## Pin Wiring

### ESP32-S3 → Panel 1 HUB75 (PI connector)

| Signal | ESP32-S3 GPIO | HUB75 Pin | Notes |
|---|---|---|---|
| MOSI | 11 | R1 | SPI2 data — after PI→PO jumpers |
| CLK | 12 | CLK | SPI2 clock |
| LAT | 13 | LAT | Latch |
| OE | 14 | OE | Output Enable |
| A | 1 | A | Row address bit 0 |
| B | 2 | B | Row address bit 1 |
| C | 4 | C | Row address bit 2 |
| D | 5 | D | Row address bit 3 |
| GND | GND | GND | Common ground |

### Panel Chaining
```
ESP32-S3 → Panel 1 PI → [Panel 1 PO → Panel 2 PI] → Panel 2 PO (unused)
```
Connect Panel 1's HUB75 output (PO) to Panel 2's HUB75 input (PI) via ribbon cable.

### PI→PO Jumpers (on EACH panel — required)
These jumpers chain the 6 internal shift registers into one serial chain,
allowing a single MOSI line to drive all color channels.

| PI Pin | → | PO Pin |
|---|---|---|
| R2 | ← | R1 |
| G1 | ← | R2 |
| G2 | ← | G1 |
| B1 | ← | G2 |
| B2 | ← | B1 |

### UART (Ethernet-to-Serial Board)
| Signal | ESP32-S3 GPIO | Connect to |
|---|---|---|
| RX | 16 | Ethernet board TX |
| TX | 17 | Ethernet board RX |
| GND | GND | Ethernet board GND |

### Power
- Panel 1 and Panel 2: Each connected to **5V external PSU** independently
- Minimum PSU rating: **5V 8A** (both panels at full brightness)
- ESP32-S3: Powered separately (3.3V or via USB)
- **Common GND** between ESP32-S3, both panels, and PSU

---

## Project Structure

```
hub75_display/
├── CMakeLists.txt          ← Top-level, sets IDF_TARGET=esp32s3
├── sdkconfig.defaults      ← Recommended build settings
└── main/
    ├── CMakeLists.txt      ← Component registration
    ├── main.c              ← app_main(), startup pattern, task creation
    ├── hub75.h / hub75.c   ← HUB75 SPI driver, refresh task (Core 1)
    ├── framebuffer.h / .c  ← Double buffer, drawing primitives, 5×7 font
    └── eth_uart.h / .c     ← UART receiver, packet parser (Core 0)
```

---

## Build and Flash

```bash
# 1. Set up ESP-IDF environment (v5.2 or newer)
. $IDF_PATH/export.sh

# 2. Set target
idf.py set-target esp32s3

# 3. Configure (optional — sdkconfig.defaults covers most settings)
idf.py menuconfig

# 4. Build
idf.py build

# 5. Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## UART Packet Protocol

All commands follow this frame format:

```
┌────────┬────────┬────────┬──────────────────┬────────┐
│  0xAA  │  CMD   │  LEN   │    PAYLOAD       │  CRC   │
│ 1 byte │ 1 byte │ 1 byte │   LEN bytes      │ 1 byte │
└────────┴────────┴────────┴──────────────────┴────────┘
CRC = XOR of CMD + LEN + all PAYLOAD bytes
```

### Commands

| CMD | Name | Payload | Description |
|---|---|---|---|
| 0x01 | CMD_CLEAR | none | Clear display to black |
| 0x02 | CMD_PIXEL | x(1) y(1) r(1) g(1) b(1) | Set single pixel |
| 0x03 | CMD_TEXT | x(1) y(1) r(1) g(1) b(1) str+\0 | Draw text string |
| 0x04 | CMD_FILL_RECT | x(1) y(1) w(1) h(1) r(1) g(1) b(1) | Fill rectangle |
| 0x06 | CMD_FULLFRAME | chunk(1) + 64 bytes | Send full frame in 64 chunks |

### Example: Display "HELLO" in red at position (2, 12)
```
AA 03 0A 02 0C FF 00 00 48 45 4C 4C 4F 00 CRC
│  │  │  │  │  │  │  │  └──────────────── "HELLO\0"
│  │  │  │  │  └──┴──┴─────────────────── R=255, G=0, B=0
│  │  │  └──┴────────────────────────────  x=2, y=12 (position)
│  │  └───────────────────────────────── LEN=10
│  └────────────────────────────────────  CMD=TEXT
└───────────────────────────────────────  Start byte
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| No display at all | Power or OE pin issue | Check 5V PSU, verify OE=GPIO14 |
| Only one panel works | Panel chaining ribbon | Check PO→PI ribbon connection |
| Colors wrong/swapped | Bit packing order | Swap bit assignments in `build_spi_buffer()` |
| Image horizontally mirrored | SPI send direction | Reverse loop in `build_spi_buffer()` |
| Ghost rows / smearing | Mux delay too short | Increase `HUB75_MUX_DELAY_US` in hub75.h |
| Noise / random pixels | SPI speed too high | Reduce `HUB75_SPI_SPEED_HZ` to 10MHz |
| Rows scrambled | Wrong scan pattern | ICN6124 is standard BINARY — should work |
| Display flickering | Watchdog reset on Core 1 | Confirm `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1=n` |
| UART data loss | Baud rate mismatch | Verify both sides at 115200 |

---

## Color Reference (3-bit)

| Color | Value | Binary |
|---|---|---|
| BLACK | 0x00 | 000 |
| RED | 0x04 | 100 |
| GREEN | 0x02 | 010 |
| BLUE | 0x01 | 001 |
| YELLOW | 0x06 | 110 |
| CYAN | 0x03 | 011 |
| MAGENTA | 0x05 | 101 |
| WHITE | 0x07 | 111 |
