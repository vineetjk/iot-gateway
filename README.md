# STM32F103 IoT Gateway

Industrial IoT gateway that polls Modbus RTU sensors and publishes telemetry via MQTT over 4G LTE.

## Features

- **Modbus RTU polling** -- configurable slave addresses, registers, function codes, and data types
- **MQTT publishing** -- delta-mode telemetry (only changed values), QoS 1, auto-reconnect
- **OTA firmware updates** -- MQTT command triggers HTTPS download to SPI flash; bootloader flashes on reboot
- **Compile-time modem selection** -- SIMCom A7670C or Quectel EC200U; only one driver compiled, saving flash/RAM
- **Serial firmware update** -- USB-C serial protocol with CRC verification (no debugger needed)
- **Interactive CLI** -- full configuration, diagnostics, and control over USART1
- **Production/Debug builds** -- preprocessor switch strips all debug logging for smaller binaries
- **SSD1306 OLED display** -- status and OTA progress on 128x32 screen
- **RGB status LED** -- visual indication of connection state, errors, and OTA progress
- **Persistent configuration** -- stored in internal flash pages with dual-page wear leveling

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | STM32F103C8T6 (BluePill, 128KB flash) | -- |
| 4G Modem (option A) | SIMCom A7670C (LTE Cat-1) | USART2, 115200 baud |
| 4G Modem (option B) | Quectel EC200U (LTE Cat-1) | USART2, 115200 baud |
| Display | SSD1306 0.91" 128x32 OLED | I2C1 (PB6/PB7) |
| SPI Flash | W25Q64 8MB | SPI1 (PA5/6/7/8) |
| RS-485 | MAX485 transceiver | USART3 (PB10/PB11), DE=PA4 |
| Debug | CH340 USB-C (on-board) | USART1 (PA9/PA10), 115200 |
| LED | RGB common anode | PB0/PB1/PB3 (inverted) |

## Quick Start

1. Clone the repository:
   ```
   git clone https://github.com/vineetjk/iot-gateway.git
   ```

2. Open in **STM32CubeIDE 1.6.0** (File -> Import -> Existing Projects into Workspace)

3. Select modem in `Core/Inc/modem_config.h`:
   ```c
   #define MODEM_DRIVER  MODEM_DRV_SIMCOM   /* or MODEM_DRV_QUECTEL */
   ```

4. Build the project (Debug configuration for development)

5. Flash via ST-Link or use the serial bootloader with `Tools/gateway_tool.py`

## Project Structure

```
Core/Inc/               Header files (all .h files)
Core/Src/               CubeMX-generated HAL sources
App/
  01_Communication/     gsm_a7670c.c, gsm_quectel.c, modem_hal.c, modbus.c
  02_Display/           ssd1306.c, rgb_led.c, display_ui.c
  03_Storage/           w25q_spi.c, config.c
  04_Telemetry/         telemetry.c
  05_OTA/               ota_manager.c
  06_Debug/             debug_cli.c
Tools/                  Python utilities (GUI tool, firmware updater)
Docs/                   Pinout diagram, MQTT test scripts, dashboard
```

## Configuration

### CLI Commands

Connect to the on-board CH340 USB-C at 115200 baud (8N1). Type `help` for a full list.

| Command | Description |
|---------|-------------|
| `show config` | Print all configuration |
| `reg add <slave> <addr> <fc> <type> <scale> <tag>` | Add a Modbus register to poll |
| `reg list` | List configured registers |
| `set mqtt broker <host:port>` | Set MQTT broker |
| `set mqtt topic <topic>` | Set data publish topic |
| `save` | Persist configuration to flash |
| `reboot` | System reset |
| `fwupdate` | Enter serial firmware update mode |
| `uid` | Show MCU unique ID and device ID |
| `i2cscan` | Scan I2C bus for connected devices |

### MQTT Defaults

| Parameter | Value |
|-----------|-------|
| Broker | `broker.emqx.io:1883` |
| Client ID | `delta-gw-{MCU_UID}` (auto-generated) |
| Data topic | `delta/gw/{ID}/data` |
| Alarm topic | `delta/gw/{ID}/alarms` |
| QoS | 1 |

## OTA Update

The OTA system uses a two-stage process:

1. **Trigger** -- An MQTT command (JSON with URL, size, CRC32) is published to the device's command topic, or the device periodically checks a manifest URL.

2. **Download** -- The application opens a raw SSL/TLS socket to the firmware server and streams the binary in chunks directly to the W25Q64 SPI flash (Slot A at 0x001000). CRC32 is computed on-the-fly.

3. **Verify** -- After download completes, the running CRC32 is compared against the expected value. OTA metadata (version, size, CRC, status) is written to SPI flash address 0x000000.

4. **Reboot** -- The application sets an SRAM flag and triggers a system reset. The bootloader detects the flag, reads metadata from SPI flash, erases internal flash, and copies the new firmware to the application region (0x08003000).

5. **Confirm** -- After the new firmware boots and successfully connects to MQTT, it calls `BL_ConfirmBoot()` to mark the update as successful. If confirmation never arrives, the bootloader can roll back from Slot B on the next reset.

### W25Q64 Flash Layout

| Address | Size | Purpose |
|---------|------|---------|
| 0x000000 | 4 KB | OTA Metadata |
| 0x001000 | 128 KB | Firmware Slot A (download target) |
| 0x021000 | 128 KB | Firmware Slot B (backup/rollback) |
| 0x041000 | 16 KB | Configuration mirror |
| 0x045000 | ~7 MB | Telemetry ring buffer |

## Build Configurations

### Debug (default)

- Optimization: `-O0`
- All `Debug_Print` / `Debug_Printf` calls active
- AT command logging enabled
- Typical size: ~96 KB flash, ~19 KB RAM

### Production

- Optimization: `-Os`
- All debug logging compiled out (zero flash cost)
- CLI still functional (commands work, but no verbose output)
- Typical size: ~50-55 KB flash, ~17 KB RAM

To switch to production:

1. Add `PRODUCTION=1` to project preprocessor symbols
2. Set optimization to `-Os`
3. Optionally enable LTO (`-flto`) for additional savings
4. Rebuild all

See `Tools/BUILD_PRODUCTION.md` for detailed STM32CubeIDE steps.

### Modem Selection

Edit `Core/Inc/modem_config.h`:

```c
#define MODEM_DRIVER  MODEM_DRV_SIMCOM    /* SIMCom A7670C — AT+CMQTT, AT+HTTP */
#define MODEM_DRIVER  MODEM_DRV_QUECTEL   /* Quectel EC200U — AT+QMT, AT+QHTTP */
```

Only the selected driver is compiled, saving 2-4 KB flash and ~350 bytes RAM.

## Tools

### gateway_tool.py

Tkinter GUI application providing:
- Serial monitor with command input
- Firmware update via serial protocol
- COM port auto-detection

Requirements: `pip install pyserial paho-mqtt`

### fw_update.py

Command-line firmware update tool implementing the bootloader serial protocol.

```
python Tools/fw_update.py --port COM3 --file Release/GATEWAY.bin
```

### ota_trigger.py

MQTT-based OTA trigger script for remote firmware deployment.

## Pin Map

| Pin | Function | Notes |
|-----|----------|-------|
| PA2/PA3 | USART2 TX/RX | 4G modem, 115200 baud |
| PA4 | RS-485 DE/RE | Direction control for MAX485 |
| PA5 | SPI1 SCK | W25Q64 clock |
| PA6 | SPI1 MISO | W25Q64 data out |
| PA7 | SPI1 MOSI | W25Q64 data in |
| PA8 | SPI1 CS | W25Q64 chip select (GPIO) |
| PA9/PA10 | USART1 TX/RX | Debug CLI (CH340 USB-C) |
| PB0 | RGB Red | Common anode, LOW = ON |
| PB1 | RGB Green | Common anode, LOW = ON |
| PB3 | RGB Blue | Common anode, LOW = ON |
| PB4 | Modem RESET | Active LOW pulse |
| PB6/PB7 | I2C1 SCL/SDA | SSD1306 OLED (0x3C) |
| PB10/PB11 | USART3 TX/RX | Modbus RS-485, 9600 baud |
| PC13 | Built-in LED | Heartbeat blink (500 ms) |

## Related Repositories

- **Bootloader**: [iot-gateway-bootloader](https://github.com/vineetjk/iot-gateway-bootloader) -- 12 KB serial + OTA bootloader at 0x08000000

## License

MIT License. See [LICENSE](LICENSE) for details.
