#!/usr/bin/env python3
"""
Delta IoT Gateway — MQTT OTA Trigger Tool
==========================================
Publishes an OTA command to the device's MQTT command topic.
The device downloads the firmware from the given URL, writes it
to SPI flash, and reboots into the bootloader.

Usage:
    python ota_trigger.py --id 1A2B --url http://your-server.com/fw/GATEWAY.bin --file GATEWAY.bin

    --id       Device ID (hex, from 'show config' or OLED display)
    --url      HTTP URL where the device can download the .bin file
    --file     Local .bin file (used to compute size and CRC32)
    --broker   MQTT broker (default: broker.emqx.io)
    --port     MQTT port (default: 1883)
    --version  Firmware version string (default: 99.0.0 to force update)

Requirements:
    pip install paho-mqtt
"""

import argparse
import struct
import sys
import os
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt not installed. Run: pip install paho-mqtt")
    sys.exit(1)


def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xEDB88320) if (crc & 1) else (crc >> 1)
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF


def main():
    parser = argparse.ArgumentParser(
        description="Delta IoT Gateway — MQTT OTA Trigger")
    parser.add_argument("--id", required=True,
                        help="Device ID in hex (e.g., 1A2B)")
    parser.add_argument("--url", required=True,
                        help="HTTP URL for firmware download (device fetches this)")
    parser.add_argument("--file", required=True,
                        help="Local firmware .bin file (for size/CRC calculation)")
    parser.add_argument("--broker", default="broker.emqx.io",
                        help="MQTT broker (default: broker.emqx.io)")
    parser.add_argument("--port", type=int, default=1883,
                        help="MQTT port (default: 1883)")
    parser.add_argument("--version", default="99.0.0",
                        help="Firmware version (default: 99.0.0)")
    args = parser.parse_args()

    # Read and validate firmware
    if not os.path.isfile(args.file):
        print(f"ERROR: File not found: {args.file}")
        sys.exit(1)

    with open(args.file, "rb") as f:
        fw_data = f.read()

    fw_size = len(fw_data)
    fw_crc = crc32(fw_data)

    # Validate vector table
    if fw_size >= 8:
        sp = struct.unpack_from("<I", fw_data, 0)[0]
        pc = struct.unpack_from("<I", fw_data, 4)[0]
        if not (0x20000000 <= sp <= 0x20005000):
            print(f"WARNING: SP=0x{sp:08X} looks invalid")
        if not (0x08003000 <= pc <= 0x08003000 + 108 * 1024):
            print(f"WARNING: Reset vector=0x{pc:08X} looks invalid")
            print(f"  Make sure .bin is built with FLASH ORIGIN=0x08003000")

    device_id = args.id.upper()
    topic = f"delta/gw/{device_id}/cmd"

    # CSV format: ota,<url>,<size>,<crc32>,<version>
    csv_payload = f"ota,{args.url},{fw_size},0x{fw_crc:08X},{args.version}"

    print(f"Firmware: {args.file}")
    print(f"Size:     {fw_size} bytes ({fw_size/1024:.1f} KB)")
    print(f"CRC32:    0x{fw_crc:08X}")
    print(f"Version:  {args.version}")
    print(f"Topic:    {topic}")
    print(f"Broker:   {args.broker}:{args.port}")
    print(f"Payload:  {csv_payload}")
    print()

    # Connect, publish, disconnect
    try:
        # Support both paho-mqtt v1 and v2
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                 client_id=f"ota-trigger-{device_id}")
        except (AttributeError, TypeError):
            client = mqtt.Client(client_id=f"ota-trigger-{device_id}")

        print("Connecting to broker...")
        client.connect(args.broker, args.port, 60)
        client.loop_start()

        # Wait for connection
        for _ in range(50):
            if client.is_connected():
                break
            time.sleep(0.1)

        if not client.is_connected():
            print("ERROR: Could not connect to broker")
            sys.exit(1)

        print("Connected to broker")
        result = client.publish(topic, csv_payload, qos=1)
        result.wait_for_publish(timeout=5)
        print(f"Published OTA command to {topic}")

        client.disconnect()
        client.loop_stop()

    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print()
    print("=" * 50)
    print("  OTA command sent!")
    print("  Device will download firmware and reboot.")
    print("  Watch the CLI (log on) for progress.")
    print("=" * 50)


if __name__ == "__main__":
    main()
