#!/usr/bin/env python3
"""
Delta IoT Gateway — Serial Firmware Update Tool
================================================
Updates STM32 firmware over USB serial (CH340 USB-C).

Usage:
    python fw_update.py -p COM5 -f firmware.bin
    python fw_update.py -p /dev/ttyUSB0 -f firmware.bin

Protocol:
    1. SYNC:   Send 0x7F, wait for 0x79 ACK
    2. START:  Send [0x01][size:4 LE][crc32:4 LE], wait ACK
    3. DATA:   Send [0x02][len:2 LE][data:N][crc16:2 LE], wait ACK per chunk
    4. VERIFY: Send [0x03], wait ACK (CRC32 verified by bootloader)

Requirements:
    pip install pyserial
"""

import argparse
import struct
import sys
import time
import os

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# Protocol constants (must match bootloader.h)
SYNC_BYTE   = 0x7F
ACK         = 0x79
NACK        = 0x1F
CMD_START   = 0x01
CMD_DATA    = 0x02
CMD_VERIFY  = 0x03
CHUNK_SIZE  = 256
MAX_FW_SIZE = 108 * 1024  # 108KB max app size
BAUD_RATE   = 115200


def crc32(data: bytes) -> int:
    """CRC-32 matching the bootloader's bl_crc32()."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xEDB88320) if (crc & 1) else (crc >> 1)
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF


def crc16_ccitt(data: bytes) -> int:
    """CRC-16 CCITT matching the bootloader's bl_crc16()."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def wait_ack(ser, timeout=10.0):
    """Wait for ACK (0x79) or NACK (0x1F). Skips any stray debug text bytes."""
    deadline = time.time() + timeout
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            return None  # timeout
        ser.timeout = remaining
        resp = ser.read(1)
        if not resp:
            return None  # timeout
        if resp[0] == ACK:
            return True
        if resp[0] == NACK:
            return False
        # Skip non-protocol bytes (debug text from bootloader)


def read_bootloader_output(ser, timeout=0.5):
    """Read and print any debug text from the bootloader."""
    ser.timeout = timeout
    while True:
        line = ser.readline()
        if not line:
            break
        try:
            text = line.decode('ascii', errors='replace').rstrip()
            if text:
                print(f"  BL: {text}")
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(
        description="Delta IoT Gateway — Serial Firmware Update")
    parser.add_argument("-p", "--port", required=True,
                        help="Serial port (e.g., COM5, /dev/ttyUSB0)")
    parser.add_argument("-f", "--file", required=True,
                        help="Firmware binary file (.bin)")
    parser.add_argument("-b", "--baud", type=int, default=BAUD_RATE,
                        help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument("--no-reboot", action="store_true",
                        help="Don't send 'fwupdate' command first")
    args = parser.parse_args()

    # Read firmware file
    if not os.path.isfile(args.file):
        print(f"ERROR: File not found: {args.file}")
        sys.exit(1)

    with open(args.file, "rb") as f:
        fw_data = f.read()

    fw_size = len(fw_data)
    fw_crc = crc32(fw_data)

    print(f"Firmware: {args.file}")
    print(f"Size:     {fw_size} bytes ({fw_size/1024:.1f} KB)")
    print(f"CRC32:    0x{fw_crc:08X}")
    print()

    if fw_size == 0:
        print("ERROR: Firmware file is empty")
        sys.exit(1)
    if fw_size > MAX_FW_SIZE:
        print(f"ERROR: Firmware too large ({fw_size} > {MAX_FW_SIZE})")
        sys.exit(1)

    # Validate vector table
    if fw_size >= 8:
        sp = struct.unpack_from("<I", fw_data, 0)[0]
        pc = struct.unpack_from("<I", fw_data, 4)[0]
        if not (0x20000000 <= sp <= 0x20005000):
            print(f"WARNING: SP=0x{sp:08X} looks invalid (expected 0x2000xxxx)")
        if not (0x08003000 <= pc <= 0x08003000 + MAX_FW_SIZE):
            print(f"WARNING: Reset vector=0x{pc:08X} looks invalid")
            print(f"  Expected range: 0x08003000-0x{0x08003000+MAX_FW_SIZE:08X}")
            print(f"  Make sure the .bin is built with FLASH ORIGIN=0x08003000")
            resp = input("  Continue anyway? [y/N] ")
            if resp.lower() != 'y':
                sys.exit(1)
    print()

    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {args.port}: {e}")
        sys.exit(1)

    print(f"Opened {args.port} at {args.baud} baud")

    # Optionally send 'fwupdate' command to trigger reboot into bootloader
    if not args.no_reboot:
        print("Sending 'fwupdate' command to enter bootloader mode...")
        ser.write(b"fwupdate\r\n")
        time.sleep(0.5)  # brief wait for reboot
        ser.reset_input_buffer()  # discard any app output before reboot

    # ── SYNC ──
    print("Syncing with bootloader...")
    synced = False
    for attempt in range(100):  # try for ~10 seconds
        ser.write(bytes([SYNC_BYTE]))
        ser.timeout = 0.1
        resp = ser.read(1)
        if resp:
            if resp[0] == ACK:
                synced = True
                break
            # Skip non-ACK bytes (bootloader debug text)
            ser.timeout = 0.02
            ser.read(200)  # drain any remaining text

    if not synced:
        print("ERROR: Failed to sync with bootloader")
        print("  Make sure the device is in bootloader mode:")
        print("  - Type 'fwupdate' in CLI, or")
        print("  - Power cycle the board within 1.5s of running this tool")
        ser.close()
        sys.exit(1)

    print("Synced with bootloader!")
    read_bootloader_output(ser, timeout=0.5)

    # ── START ──
    print(f"Sending header (size={fw_size}, crc=0x{fw_crc:08X})...")
    hdr = struct.pack("<BII", CMD_START, fw_size, fw_crc)
    ser.write(hdr)

    result = wait_ack(ser, timeout=30.0)  # erase can take time
    if result is None:
        print("ERROR: Timeout waiting for erase ACK")
        ser.close()
        sys.exit(1)
    if not result:
        print("ERROR: Bootloader rejected header (NACK)")
        ser.close()
        sys.exit(1)

    print("Flash erased, starting transfer...")

    # ── DATA ──
    offset = 0
    chunk_num = 0
    total_chunks = (fw_size + CHUNK_SIZE - 1) // CHUNK_SIZE
    retries = 0
    max_retries = 3

    while offset < fw_size:
        chunk = fw_data[offset:offset + CHUNK_SIZE]
        chunk_len = len(chunk)

        # Build packet: [CMD_DATA][len:2 LE][data][crc16:2 LE]
        pkt_hdr = struct.pack("<BH", CMD_DATA, chunk_len)
        crc_data = pkt_hdr + chunk
        crc = crc16_ccitt(crc_data)
        packet = pkt_hdr + chunk + struct.pack("<H", crc)

        ser.write(packet)
        result = wait_ack(ser, timeout=5.0)

        if result is None:
            print(f"\nERROR: Timeout at chunk {chunk_num}/{total_chunks}")
            ser.close()
            sys.exit(1)
        if not result:
            retries += 1
            if retries > max_retries:
                print(f"\nERROR: Too many retries at chunk {chunk_num}")
                read_bootloader_output(ser)
                ser.close()
                sys.exit(1)
            print(f"\n  NACK at chunk {chunk_num}, retrying ({retries}/{max_retries})...")
            continue

        retries = 0
        offset += chunk_len
        chunk_num += 1

        # Progress bar
        pct = offset * 100 // fw_size
        bar_len = 40
        filled = bar_len * offset // fw_size
        bar = '█' * filled + '░' * (bar_len - filled)
        print(f"\r  [{bar}] {pct:3d}% ({offset}/{fw_size})", end='', flush=True)

    print()  # newline after progress bar

    # Read any bootloader progress messages
    read_bootloader_output(ser, timeout=1.0)

    # ── VERIFY ──
    print("Verifying firmware CRC32...")
    ser.write(bytes([CMD_VERIFY]))
    result = wait_ack(ser, timeout=10.0)

    if result is None:
        print("ERROR: Timeout waiting for verify")
        ser.close()
        sys.exit(1)
    if not result:
        print("ERROR: CRC verification FAILED!")
        read_bootloader_output(ser)
        ser.close()
        sys.exit(1)

    print("Verification OK!")
    read_bootloader_output(ser, timeout=2.0)
    print()
    print("=" * 50)
    print("  Firmware update successful!")
    print("  Device is booting new firmware...")
    print("=" * 50)

    # Read any boot messages
    time.sleep(1)
    read_bootloader_output(ser, timeout=3.0)

    ser.close()


if __name__ == "__main__":
    main()
