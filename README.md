# STM8S-NRF24L01-RECEIVER

[![Build](https://github.com/a5021/STM8S-NRF24L01-RECEIVER/actions/workflows/build.yml/badge.svg?branch=master)](https://github.com/a5021/STM8S-NRF24L01-RECEIVER/actions/workflows/build.yml) [![MCU](https://img.shields.io/badge/MCU-STM8S003F3-00A9E0)]() [![Radio](https://img.shields.io/badge/Radio-nRF24L01-00A9E0)]() [![License](https://img.shields.io/badge/License-MIT-yellow)]()

Wireless nRF24L01+ receiver based on STM8S003F3. Listens on RF channel 99, receives variable-length dynamic payloads, decodes sensor data (BMP180 pressure/temperature, SI7021 humidity/temperature, battery voltage, die temperature) and prints the values over UART at 115200 baud.

## Features

- Register-level, bare-metal firmware (no HAL, no SPL)
- nRF24L01+ in Enhanced ShockBurst RX mode, dynamic payload length, NoACK
- 3-byte address width, channel 99, 2 Mbps, 16-bit CRC
- Decodes multisensor composite packets: pressure, temperature, humidity, VBAT, die temperature
- Clock scaling for power saving: 16 MHz to 2 MHz during IRQ wait
- Peripheral clock gating: SPI/UART clocked only when active
- Independent watchdog (IWDG) with LSI, kicked every idle loop iteration
- Receives all available payloads from RX FIFO in burst
- Proteus simulation project included

## Build

Two compilers are supported: **SDCC** (free, cross-platform, CI-ready) and **IAR Embedded Workbench for STM8** (commercial, Windows-only).

| Feature               | SDCC                                       | IAR EWSTM8                                 |
|-----------------------|--------------------------------------------|--------------------------------------------|
| Compiler              | `sdcc` 4.6.x                               | `iccstm8`                                  |
| License               | GPLv2 (free)                               | Commercial (paid license required)         |
| Host OS               | Linux, macOS, Windows                      | Windows only                               |
| Command               | `make`                                     | Open `Project.eww` in IDE                  |
| Preprocessing         | `iar2sdcc.py` patches IAR idioms           | None (native IAR project)                  |
| Output                | Intel HEX via `packihx`                    | Intel HEX / Debug (IAR format)             |
| Debug                 | No (standalone build)                      | Full (IAR C-SPY debugger, simulator)       |
| CI (GitHub Actions)   | Yes (SDCC tarball cached)                  | No                                         |

### SDCC (recommended)

```sh
make
```

Requires: `sdcc` ≥4.6.0, `python3` ≥3.6, `packihx` (included with SDCC).

The build process:
1. `iar2sdcc.py` preprocesses `src/main.c` — patches `SPI_CR1_BR` bitfield access (IAR-specific) to read-modify-write, and replaces `snprintf` with `sprintf` (SDCC's STM8 libc lacks `snprintf`).
2. `sdcc` compiles the patched source for STM8 with `--out-fmt-ihx`.
3. `packihx` converts the Intel HEX record into final `.hex`.

CI is fully automated via `.github/workflows/build.yml` — SDCC 4.6.0-rc2 is cached from SourceForge, so no toolchain download on subsequent runs.

### IAR Embedded Workbench

Open `Project.eww` in IAR EWSTM8, build from the IDE. No patch step needed — all toolchain-specific features (bitfields, `snprintf`) work natively.

## Hardware Specification

| Component | Detail |
|-----------|--------|
| MCU | STMicroelectronics STM8S003F3 (STM8S, 8 KB Flash, 1 KB RAM) |
| Radio | Nordic nRF24L01+ (SPI, 1 MHz, channel 99, PRX mode) |
| Clock | HSI 16 MHz |

## Pin Assignment

| Signal | Pin | Notes |
|--------|-----|-------|
| NRF_IRQ | PC4 | GPIO input, floating, active low |
| NRF_CSN | PD2 | GPIO output, SPI chip select, active low |
| NRF_CE | PD3 | GPIO output, chip enable, active high |
| SPI_SCK | PD0 | |
| SPI_MISO | PD1 | |
| UART_TX | PD5 | 115200 baud |

## Radio Protocol

| Parameter | Value |
|-----------|-------|
| RF Channel | 99 |
| Air data rate | 2 Mbps |
| Address width | 3 bytes |
| CRC | 2 bytes (CRC16) |
| Auto-ACK | Disabled |
| Dynamic payload | Enabled |

The receiver decodes the following payload fields (7-11 bytes):

| Offset | Size | Field |
|--------|------|-------|
| 0-2 | 3 bytes | BMP180 pressure (big-endian) |
| 3 | 1 byte | Die temperature (signed) |
| 4-5 | 2 bytes | SI7021 temperature (signed, C x 100) |
| 6-7 | 2 bytes | SI7021 humidity (RH x 100) |
| 8-9 | 2 bytes | VBAT (mV) |

## Firmware Architecture

`
  Cold boot
     |
  +-------------------+
  | CLK_CKDIVR = FAST |  16 MHz
  | Periph clocks ON  |
  | IWDG init         |
  | GPIO init         |
  | SPI init          |
  | UART init (115200)|
  | nRF24L01+ init    |
  +-------------------+
     |
  +-------------------+
  | nRF24L01 detect   |
  | (FIFO test)       |
  +-------------------+
     |
  +=================================+
  |     Main loop                   |
  |  CLK_CKDIVR = SLOW (2 MHz)      |
  |  CLOCK_DISABLE(SPI)             |
  |  while (IRQ line HIGH) {        |
  |      IWDG refresh               |
  |  }                              |
  +=================================+
     |
     | IRQ goes LOW
     v
  +-------------------+
  | CLK = NORMAL      |
  | CLOCK_ENABLE(SPI) |
  +-------------------+
     |
     v
  +-------------------+
  | RX FIFO loop      |
  | do {              |
  |   nrf_get_size()  |
  |   if >32 flush    |
  |   nrf_read_payload|
  |   decode fields   |
  |   uprintf values  |
  | } while(!RX_EMPTY)|
  +-------------------+
     |
     v
  +-------------------+
  | CLR RX_DR         |
  | DISABLE_UART      |
  +-------------------+
     |
     v
     (back to Main loop)
`
## Project Structure

```
src/
+-- main.c                  Application entry + all drivers
inc/
+-- iostm8s003f3.h          SDCC-compatible register map (__at + bitfields)
iar2sdcc.py                 Patch script (IAR -> SDCC)
Makefile                    SDCC build
Project.eww / .ewp / .ewd   IAR Embedded Workbench project
*.pdsprj                    Proteus simulation
```