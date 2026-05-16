# GATEWAY Project Context

## What This Project Is
STM32F103C8T6 (BluePill) IoT Gateway that reads Modbus RTU sensors and publishes data via MQTT over 4G LTE (SIMCom A7670C module).

## Hardware
- **MCU:** STM32F103C8T6 (64KB flash officially, using ~65KB — relies on common 128KB hidden flash)
- **GSM:** SIMCom A7670C (4G LTE Cat-1) — has RESET pin (PB4), NOT PWRKEY. Sends "RDY" URC on boot.
- **Display:** SSD1306 0.91" 128x32 OLED on I2C1 (PB6/PB7) — currently not working, needs debugging
- **Storage:** W25Q64 8MB SPI flash (not yet received) — for OTA + telemetry logging
- **RS-485:** MAX485 on USART3 (PB10/PB11, DE=PA4) for Modbus RTU
- **LED:** RGB common anode on PB0/PB1/PB3 (inverted logic: LOW=ON), built-in PC13 for heartbeat
- **Debug:** USART1 (PA9/PA10) 115200 8N1 — CH340 USB-C on board, full CLI interface

## Project Structure
```
Core/Inc/          — ALL header files (.h)
Core/Src/          — CubeMX-generated sources only (main, gpio, i2c, spi, usart, hal_msp, it, system)
App/
  01_Communication/ — gsm_a7670c.c, modbus.c
  02_Display/       — ssd1306.c, rgb_led.c
  03_Storage/       — w25q_spi.c, config.c
  04_Telemetry/     — telemetry.c
  05_OTA/           — ota_manager.c
  06_Debug/         — debug_cli.c
Bootloader/Src/     — bootloader_main.c (separate project, 12KB at 0x08000000)
Bootloader/         — STM32F103_BOOTLOADER.ld (linker script for bootloader project)
Tools/              — fw_update.py (Python serial firmware update tool)
Docs/               — HTML pinout diagram, mqtt test script, dashboard
```

## Build System
- STM32CubeIDE 1.6.0, arm-none-eabi-gcc 9-2020-q2
- IDE auto-regenerates Debug/makefile, sources.mk, objects.list — App/ folders must be re-added to `.cproject` sourceEntries (already done)
- Linker script: `STM32F103C8TX_FLASH.ld` — ORIGIN=0x08000000, LENGTH=120K
- Config flash pages at 0x0801E000 and 0x0801E800 (120-124KB range)

## Key Design Decisions
1. **Bootloader active** — app at 0x08003000 (108KB), bootloader at 0x08000000 (12KB). VTOR set in system_stm32f1xx.c. Serial FW update via USB-C (`fwupdate` CLI command + `Tools/fw_update.py`).
2. **W25Q64 not present** — `spi_flash_ok` flag guards all SPI flash operations (OTA, telemetry log)
3. **GSM not always present** — `gsm_present` flag set after 10s timeout; if false, all MQTT/GSM code skipped
4. **Config not saved to flash on first boot** — `Config_Init()` loads defaults but skips `Config_Save()` to avoid hanging if flash pages are inaccessible
5. **Delta telemetry mode** — only publishes changed registers. With 0 registers configured, nothing publishes. Startup JSON message sent once after first MQTT connect.

## Current MQTT Config (defaults, auto-generated from MCU ID)
- Broker: `broker.emqx.io:1883` (public, no auth)
- Client ID: `delta-gw-{ID}` (auto from MCU UID)
- Data topic: `delta/gw/{ID}/data`
- Alarm topic: `delta/gw/{ID}/alarms`
- QoS: 1

## Known Issues / TODOs
- OLED display not showing anything (I2C address 0x3C confirmed in code, may need pull-ups or address scan via `i2cscan` CLI command)
- Config not persisted to flash (first boot always loads defaults) — needs testing if 0x0801E000 is writable on this specific C8 chip
- GSM module RDY URC not received (times out), but AT commands work after ~5s
- AT+CPIN? fails on first try (SIM needs time), succeeds on retry in main loop
- Bootloader is a separate project — not yet flashed

## CLI Commands
Type `help` in USART1 terminal (CH340 USB-C, 115200 baud). Key commands:
- `i2cscan` — scan I2C bus for devices
- `show config` — print all configuration
- `reg add <slave> <addr> <fc> <type> <scale> <tag>` — add Modbus register
- `save` — save config to flash
- `reboot` — system reset
- `fwupdate` — enter serial firmware update mode (reboots into bootloader)
- `uid` — show MCU unique ID and device ID

## Pin Map (quick reference)
```
PA2/PA3   — USART2 TX/RX (A7670C GSM, 115200)
PA4       — RS-485 DE/RE (directly connected, shared with USART3 Modbus)
PA5/6/7/8 — SPI1 SCK/MISO/MOSI/CS (W25Q64)
PA9/PA10  — USART1 TX/RX (Debug CLI, 115200, CH340 USB-C on board)
PB0/1/3   — RGB LED R/G/B (common anode, LOW=ON)
PB4       — A7670C RESET (active LOW pulse)
PB6/PB7   — I2C1 SCL/SDA (SSD1306 OLED)
PB10/PB11 — USART3 TX/RX (Modbus RS-485, 9600)
PC13      — Built-in LED (heartbeat blink 500ms)
```
