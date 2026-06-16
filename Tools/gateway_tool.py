#!/usr/bin/env python3
"""
Delta IoT Gateway — GUI Tool
============================
Serial monitor, firmware update, and command interface.

Requirements:
    pip install pyserial paho-mqtt
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import struct
import time
import os
import queue

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    exit(1)

# ── Protocol constants (must match bootloader) ──
SYNC_BYTE = 0x7F
ACK = 0x79
NACK = 0x1F
CMD_START = 0x01
CMD_DATA = 0x02
CMD_VERIFY = 0x03
CHUNK_SIZE = 256
MAX_FW_SIZE = 108 * 1024
BAUD_RATE = 115200


def crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xEDB88320) if (crc & 1) else (crc >> 1)
    return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


class GatewayTool:
    def __init__(self, root):
        self.root = root
        self.root.title("Delta IoT Gateway Tool")
        self.root.geometry("900x650")
        self.root.configure(bg="#1e1e1e")

        self.ser = None
        self.serial_thread = None
        self.running = False
        self.logging_enabled = True
        self.rx_queue = queue.Queue()

        self._build_ui()
        self._poll_queue()

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#1e1e1e")
        style.configure("TLabel", background="#1e1e1e", foreground="#cccccc")
        style.configure("TButton", padding=5)
        style.configure("Green.TButton", foreground="#000", background="#4ec9b0")
        style.configure("Red.TButton", foreground="#000", background="#f44747")
        style.configure("Orange.TButton", foreground="#000", background="#ce9178")

        # ── Top bar: Serial connection ──
        conn_frame = ttk.Frame(self.root)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        self._refresh_ports()

        ttk.Button(conn_frame, text="⟳", width=3,
                   command=self._refresh_ports).pack(side=tk.LEFT)

        ttk.Label(conn_frame, text="Baud:").pack(side=tk.LEFT, padx=(10, 0))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn_frame, textvariable=self.baud_var, width=8,
                     values=["9600", "115200", "230400", "460800"]).pack(side=tk.LEFT, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect",
                                      style="Green.TButton", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="#f44747")
        self.status_label.pack(side=tk.LEFT, padx=10)

        # ── Log toggle ──
        self.log_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(conn_frame, text="Show Logs", variable=self.log_var,
                        command=self._toggle_logs).pack(side=tk.RIGHT)

        # ── Serial monitor ──
        monitor_frame = ttk.Frame(self.root)
        monitor_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.monitor = tk.Text(monitor_frame, bg="#0d1117", fg="#c9d1d9",
                               font=("Consolas", 9), wrap=tk.WORD,
                               insertbackground="#c9d1d9", state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(monitor_frame, orient=tk.VERTICAL,
                                  command=self.monitor.yview)
        self.monitor.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.monitor.pack(fill=tk.BOTH, expand=True)

        # Text tags for coloring
        self.monitor.tag_configure("tx", foreground="#4ec9b0")
        self.monitor.tag_configure("rx", foreground="#c9d1d9")
        self.monitor.tag_configure("info", foreground="#569cd6")
        self.monitor.tag_configure("error", foreground="#f44747")
        self.monitor.tag_configure("success", foreground="#6a9955")

        # ── Command input ──
        cmd_frame = ttk.Frame(self.root)
        cmd_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(cmd_frame, text="Command:").pack(side=tk.LEFT)
        self.cmd_entry = tk.Entry(cmd_frame, bg="#2d2d2d", fg="#cccccc",
                                  font=("Consolas", 10), insertbackground="#cccccc")
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.cmd_entry.bind("<Return>", self._send_command)

        ttk.Button(cmd_frame, text="Send", command=self._send_command).pack(side=tk.LEFT)
        ttk.Button(cmd_frame, text="Clear", command=self._clear_monitor).pack(side=tk.LEFT, padx=5)

        # ── Firmware path ──
        fw_frame = ttk.Frame(self.root)
        fw_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Label(fw_frame, text="Firmware:").pack(side=tk.LEFT)
        self.fw_path_var = tk.StringVar()
        self.fw_entry = tk.Entry(fw_frame, textvariable=self.fw_path_var,
                                 bg="#2d2d2d", fg="#888888",
                                 font=("Consolas", 9), insertbackground="#cccccc")
        self.fw_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.fw_entry.insert(0, "C:/Users/prith/STM32CubeIDE/workspace_1.6.0/GATEWAY/Debug/GATEWAY.bin")
        self.fw_entry.configure(fg="#cccccc")

        ttk.Button(fw_frame, text="Browse", command=self._browse_firmware).pack(side=tk.LEFT)

        # ── Bottom: Action buttons ──
        action_frame = ttk.Frame(self.root)
        action_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Button(action_frame, text="Flash Firmware (Serial)",
                   style="Orange.TButton",
                   command=self._flash_firmware).pack(side=tk.LEFT, padx=5)

        ttk.Button(action_frame, text="Reboot Device",
                   command=self._reboot).pack(side=tk.LEFT, padx=5)

        ttk.Button(action_frame, text="Show Config",
                   command=lambda: self._quick_cmd("show config")).pack(side=tk.LEFT, padx=5)

        ttk.Button(action_frame, text="I2C Scan",
                   command=lambda: self._quick_cmd("i2cscan")).pack(side=tk.LEFT, padx=5)

        ttk.Button(action_frame, text="Device UID",
                   command=lambda: self._quick_cmd("uid")).pack(side=tk.LEFT, padx=5)

        # ── Progress bar (for firmware update) ──
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(self.root, variable=self.progress_var,
                                            maximum=100)
        self.progress_bar.pack(fill=tk.X, padx=10, pady=(0, 10))

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        if not port:
            messagebox.showerror("Error", "Select a serial port")
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.running = True
            self.serial_thread = threading.Thread(target=self._read_serial, daemon=True)
            self.serial_thread.start()
            self.connect_btn.configure(text="Disconnect", style="Red.TButton")
            self.status_label.configure(text=f"Connected: {port}", foreground="#4ec9b0")
            self._log(f"Connected to {port} @ {baud}\n", "info")
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", str(e))

    def _disconnect(self):
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=2)
        if self.ser:
            self.ser.close()
            self.ser = None
        self.connect_btn.configure(text="Connect", style="Green.TButton")
        self.status_label.configure(text="Disconnected", foreground="#f44747")
        self._log("Disconnected\n", "info")

    def _read_serial(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                data = self.ser.readline()
                if data:
                    text = data.decode('ascii', errors='replace').rstrip('\r\n')
                    if text:
                        self.rx_queue.put(("rx", text + "\n"))
            except (serial.SerialException, OSError):
                self.rx_queue.put(("error", "Serial connection lost\n"))
                break

    def _poll_queue(self):
        """Process queued messages in main thread."""
        try:
            while True:
                tag, text = self.rx_queue.get_nowait()
                if self.logging_enabled:
                    self._log(text, tag)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_queue)

    def _log(self, text, tag="rx"):
        self.monitor.configure(state=tk.NORMAL)
        self.monitor.insert(tk.END, text, tag)
        self.monitor.see(tk.END)
        self.monitor.configure(state=tk.DISABLED)
        # Limit buffer to 5000 lines
        lines = int(self.monitor.index('end-1c').split('.')[0])
        if lines > 5000:
            self.monitor.configure(state=tk.NORMAL)
            self.monitor.delete('1.0', '1000.0')
            self.monitor.configure(state=tk.DISABLED)

    def _toggle_logs(self):
        self.logging_enabled = self.log_var.get()
        state = "ON" if self.logging_enabled else "OFF"
        self._log(f"[Logging {state}]\n", "info")

    def _send_command(self, event=None):
        cmd = self.cmd_entry.get().strip()
        if not cmd:
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Warning", "Not connected")
            return
        self.ser.write((cmd + "\r\n").encode())
        self._log(f"> {cmd}\n", "tx")
        self.cmd_entry.delete(0, tk.END)

    def _quick_cmd(self, cmd):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Warning", "Not connected")
            return
        self.ser.write((cmd + "\r\n").encode())
        self._log(f"> {cmd}\n", "tx")

    def _clear_monitor(self):
        self.monitor.configure(state=tk.NORMAL)
        self.monitor.delete('1.0', tk.END)
        self.monitor.configure(state=tk.DISABLED)

    def _reboot(self):
        self._quick_cmd("reboot")

    def _browse_firmware(self):
        filepath = filedialog.askopenfilename(
            title="Select Firmware Binary",
            filetypes=[("Binary files", "*.bin"), ("All files", "*.*")]
        )
        if filepath:
            self.fw_path_var.set(filepath)

    def _flash_firmware(self):
        filepath = self.fw_path_var.get().strip()
        if not filepath or not os.path.isfile(filepath):
            messagebox.showerror("Error", "Select a valid firmware .bin file")
            return
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Warning", "Connect to serial port first")
            return

        # Run in background thread
        threading.Thread(target=self._do_flash, args=(filepath,), daemon=True).start()

    def _do_flash(self, filepath):
        try:
            with open(filepath, "rb") as f:
                fw_data = f.read()

            fw_size = len(fw_data)
            fw_crc = crc32(fw_data)

            if fw_size == 0 or fw_size > MAX_FW_SIZE:
                self._log(f"[ERROR] Invalid firmware size: {fw_size}\n", "error")
                return

            self._log(f"\n{'='*50}\n", "info")
            self._log(f"Firmware: {os.path.basename(filepath)}\n", "info")
            self._log(f"Size: {fw_size} bytes | CRC32: 0x{fw_crc:08X}\n", "info")
            self._log(f"{'='*50}\n\n", "info")

            # Validate vector table
            if fw_size >= 8:
                sp = struct.unpack_from("<I", fw_data, 0)[0]
                pc = struct.unpack_from("<I", fw_data, 4)[0]
                if not (0x20000000 <= sp <= 0x20005000):
                    self._log(f"[WARN] SP=0x{sp:08X} looks invalid\n", "error")
                if not (0x08003000 <= pc <= 0x08003000 + MAX_FW_SIZE):
                    self._log(f"[WARN] Reset vector=0x{pc:08X} looks invalid\n", "error")

            # Stop serial reader temporarily
            self.running = False
            time.sleep(0.3)

            # Send fwupdate command
            self._log("Sending 'fwupdate' to enter bootloader...\n", "info")
            self.ser.write(b"fwupdate\r\n")
            time.sleep(1.0)
            self.ser.reset_input_buffer()

            # SYNC
            self._log("Syncing with bootloader...\n", "info")
            synced = False
            for _ in range(100):
                self.ser.write(bytes([SYNC_BYTE]))
                self.ser.timeout = 0.1
                resp = self.ser.read(1)
                if resp and resp[0] == ACK:
                    synced = True
                    break
                if resp:
                    self.ser.timeout = 0.02
                    self.ser.read(200)

            if not synced:
                self._log("[ERROR] Failed to sync with bootloader!\n", "error")
                self._restart_reader()
                return

            self._log("Synced! Sending header...\n", "success")

            # START
            hdr = struct.pack("<BII", CMD_START, fw_size, fw_crc)
            self.ser.write(hdr)
            self.ser.timeout = 30.0
            resp = self.ser.read(1)
            if not resp or resp[0] != ACK:
                self._log("[ERROR] Header rejected by bootloader\n", "error")
                self._restart_reader()
                return

            self._log("Flash erased. Transferring...\n", "info")

            # DATA
            offset = 0
            total_chunks = (fw_size + CHUNK_SIZE - 1) // CHUNK_SIZE

            while offset < fw_size:
                chunk = fw_data[offset:offset + CHUNK_SIZE]
                chunk_len = len(chunk)

                pkt_hdr = struct.pack("<BH", CMD_DATA, chunk_len)
                crc_data = pkt_hdr + chunk
                crc = crc16_ccitt(crc_data)
                packet = pkt_hdr + chunk + struct.pack("<H", crc)

                self.ser.write(packet)
                self.ser.timeout = 5.0
                resp = self.ser.read(1)

                if not resp or resp[0] != ACK:
                    self._log(f"\n[ERROR] Failed at offset {offset}\n", "error")
                    self._restart_reader()
                    return

                offset += chunk_len
                pct = offset * 100 / fw_size
                self.progress_var.set(pct)

            # VERIFY
            self._log("\nVerifying CRC32...\n", "info")
            self.ser.write(bytes([CMD_VERIFY]))
            self.ser.timeout = 10.0
            resp = self.ser.read(1)

            if not resp or resp[0] != ACK:
                self._log("[ERROR] CRC verification FAILED!\n", "error")
                self._restart_reader()
                return

            self._log("\n" + "="*50 + "\n", "success")
            self._log("  Firmware update successful!\n", "success")
            self._log("  Device is booting new firmware...\n", "success")
            self._log("="*50 + "\n", "success")
            self.progress_var.set(100)

            time.sleep(2)
            self._restart_reader()

        except Exception as e:
            self._log(f"[ERROR] {e}\n", "error")
            self._restart_reader()

    def _restart_reader(self):
        """Restart the serial reader thread after firmware update."""
        self.running = True
        self.serial_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.serial_thread.start()


def main():
    root = tk.Tk()
    app = GatewayTool(root)
    root.mainloop()


if __name__ == "__main__":
    main()
