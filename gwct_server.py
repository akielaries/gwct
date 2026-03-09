#!/usr/bin/env python3
"""
gwct_server.py — GWCT server daemon
Runs on the machine physically connected to the FPGA(s).
Exposes services on a single TCP port via a message-type prefix byte:

  MSG_SELECT = 0x00  — select device by name (required first when multi-device)
  MSG_UART   = 0x01  — forward a GWCT UART packet, return 7-byte response
  MSG_LOAD   = 0x10  — receive bitstream, invoke openFPGALoader, return output
  MSG_FLASH  = 0x11  — same as MSG_LOAD but writes to SPI flash (-f flag)
  MSG_PING   = 0x20  — connectivity check, returns MSG_PONG

All messages start with a 1-byte type.

Multi-device usage:
  gwct_server --config devices.json

Single-device usage (backward compatible):
  gwct_server --board tangnano20k --uart /dev/ttyUSB0 [--ftdi-serial SERIAL]
"""

import argparse
import socket
import serial
import subprocess
import tempfile
import os
import json
import threading
import logging
import struct
import time
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("gwct_server")

# ---------------------------------------------------------------------------
# Message types
# ---------------------------------------------------------------------------
MSG_SELECT = 0x00  # device selection handshake (multi-device)
MSG_UART   = 0x01  # raw UART transaction: send 11 bytes, recv 7 bytes
MSG_LOAD   = 0x10  # bitstream load
MSG_FLASH  = 0x11  # bitstream flash (SPI)
MSG_PING   = 0x20
MSG_PONG   = 0x21
MSG_OK     = 0xA0
MSG_ERR    = 0xA1

# UART framing constants (must match gwct_packet.v)
UART_TX_LEN  = 11
UART_RX_LEN  = 7
UART_TIMEOUT = 2.0

DEFAULT_UART_PORT  = "/dev/gwct_port_2a881d3a6c78529c21dc0423699e6be3"
DEFAULT_UART_BAUD  = 115200
DEFAULT_BOARD      = "tangnano20k"
SERVER_HOST        = "0.0.0.0"
SERVER_PORT        = 65432


class UARTBridge:
    """Thread-safe wrapper around the FPGA UART."""

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser  = None
        self.lock = threading.Lock()

    def open(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=UART_TIMEOUT,
        )
        log.info(f"UART open: {self.port} @ {self.baud}")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def transact(self, pkt: bytes) -> bytes:
        """Send 11-byte packet, return 7-byte response. Thread-safe."""
        with self.lock:
            self.ser.reset_input_buffer()
            self.ser.write(pkt)
            self.ser.flush()

            raw = b""
            deadline = time.monotonic() + UART_TIMEOUT
            while len(raw) < UART_RX_LEN:
                chunk = self.ser.read(UART_RX_LEN - len(raw))
                if chunk:
                    raw += chunk
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"UART timeout: got {len(raw)}/{UART_RX_LEN} bytes: {raw.hex()}"
                    )
            return raw


def _ofl_cmd(board: str, ftdi_serial: str | None, bitfile: str, flash: bool) -> list[str]:
    """Build an openFPGALoader command list."""
    cmd = ["openFPGALoader", "-v", "-b", board]
    if ftdi_serial:
        cmd += ["--ftdi-serial", ftdi_serial]
    if flash:
        cmd += ["-f"]
    cmd += [bitfile]
    return cmd


def _stream_programmer(conn: socket.socket, cmd: list[str], timeout: float = 120.0):
    """Run cmd, stream its output to conn, send end-of-stream sentinel."""
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0,
    )
    buf = b""
    while True:
        ch = proc.stdout.read(1)
        if not ch:
            break
        if ch in (b"\n", b"\r"):
            if buf:
                line = buf[:65534]
                conn.sendall(struct.pack(">H", len(line)) + line)
                buf = b""
        else:
            buf += ch
    if buf:
        line = buf[:65534]
        conn.sendall(struct.pack(">H", len(line)) + line)
    proc.wait(timeout=timeout)
    conn.sendall(struct.pack(">H", 0xFFFF) + struct.pack(">i", proc.returncode))
    log.info(f"openFPGALoader done: rc={proc.returncode}")


class GWCTServer:
    def __init__(self, devices: dict, host: str = SERVER_HOST, port: int = SERVER_PORT):
        """
        devices — mapping of device name to device config dict:
            {
                "name1": {
                    "uart":         UARTBridge,
                    "board":        str,        # openFPGALoader board name
                    "ftdi_serial":  str | None, # optional FTDI serial for disambiguation
                },
                ...
            }
        """
        self.devices = devices
        self.host    = host
        self.port    = port

    def start(self):
        for name, dev in self.devices.items():
            dev["uart"].open()
            log.info(f"Device '{name}': board={dev['board']}, ftdi_serial={dev['ftdi_serial']!r}")

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
                srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                srv.bind((self.host, self.port))
                srv.listen(5)
                log.info(f"GWCT server listening on {self.host}:{self.port} "
                         f"({len(self.devices)} device(s): {', '.join(self.devices)})")
                while True:
                    try:
                        conn, addr = srv.accept()
                        t = threading.Thread(
                            target=self._handle, args=(conn, addr), daemon=True
                        )
                        t.start()
                    except KeyboardInterrupt:
                        log.info("Shutting down")
                        break
        finally:
            for dev in self.devices.values():
                dev["uart"].close()

    def _recv_exact(self, conn: socket.socket, n: int, timeout: float = 30.0) -> bytes:
        buf = b""
        conn.settimeout(timeout)
        while len(buf) < n:
            chunk = conn.recv(min(65536, n - len(buf)))
            if not chunk:
                raise ConnectionError("Client disconnected")
            buf += chunk
        conn.settimeout(None)
        return buf

    def _handle(self, conn: socket.socket, addr):
        log.info(f"Connection from {addr}")
        try:
            # ----------------------------------------------------------------
            # Device selection handshake
            # ----------------------------------------------------------------
            hdr = conn.recv(1)
            if not hdr:
                return
            first_byte = hdr[0]

            if first_byte == MSG_SELECT:
                name_len = self._recv_exact(conn, 1)[0]
                name = self._recv_exact(conn, name_len).decode()
                if name not in self.devices:
                    available = ", ".join(self.devices)
                    msg = f"unknown device '{name}'; available: {available}"
                    conn.sendall(bytes([MSG_ERR]) + msg.encode()[:64])
                    log.warning(f"{addr}: {msg}")
                    return
                device = self.devices[name]
                conn.sendall(bytes([MSG_OK]))
                log.info(f"{addr}: selected device '{name}'")
                pending = None
            else:
                # No MSG_SELECT — only valid when exactly one device is configured.
                if len(self.devices) != 1:
                    msg = b"MSG_SELECT required when multiple devices are configured"
                    conn.sendall(bytes([MSG_ERR]) + msg[:64])
                    return
                device  = next(iter(self.devices.values()))
                pending = first_byte  # process this byte as the first msg_type

            uart        = device["uart"]
            board       = device["board"]
            ftdi_serial = device["ftdi_serial"]

            # ----------------------------------------------------------------
            # Main message loop
            # ----------------------------------------------------------------
            while True:
                if pending is not None:
                    msg_type = pending
                    pending  = None
                else:
                    hdr = conn.recv(1)
                    if not hdr:
                        break
                    msg_type = hdr[0]

                # ------------------------------------------------------------
                # UART transaction
                # ------------------------------------------------------------
                if msg_type == MSG_UART:
                    pkt = self._recv_exact(conn, UART_TX_LEN)
                    try:
                        resp = uart.transact(pkt)
                        conn.sendall(bytes([MSG_OK]) + resp)
                    except Exception as e:
                        log.error(f"UART error: {e}")
                        conn.sendall(bytes([MSG_ERR]) + str(e).encode()[:64])

                # ------------------------------------------------------------
                # Bitstream load / flash
                # ------------------------------------------------------------
                elif msg_type in (MSG_LOAD, MSG_FLASH):
                    flash = (msg_type == MSG_FLASH)

                    # Receive file size
                    file_size = struct.unpack(">I", self._recv_exact(conn, 4))[0]

                    # Receive board hint from client (kept for protocol compat;
                    # we use the server-configured board instead).
                    board_len = self._recv_exact(conn, 1)[0]
                    _client_board = self._recv_exact(conn, board_len).decode()

                    # Receive bitstream data
                    recv_timeout = 180.0 if flash else 120.0
                    data = self._recv_exact(conn, file_size, timeout=recv_timeout)
                    log.info(f"{'Flash' if flash else 'Load'} {file_size} bytes "
                             f"board={board} ftdi_serial={ftdi_serial!r}")

                    with tempfile.NamedTemporaryFile(suffix=".fs", delete=False) as f:
                        f.write(data)
                        tmp = f.name
                    try:
                        cmd = _ofl_cmd(board, ftdi_serial, tmp, flash)
                        log.info(f"Running: {' '.join(cmd)}")
                        _stream_programmer(conn, cmd, timeout=recv_timeout)
                    except Exception as e:
                        log.error(f"Programming error: {e}")
                        err_line = str(e).encode("utf-8")[:65534]
                        conn.sendall(struct.pack(">H", len(err_line)) + err_line)
                        conn.sendall(struct.pack(">H", 0xFFFF) + struct.pack(">i", -1))
                    finally:
                        if os.path.exists(tmp):
                            os.unlink(tmp)

                # ------------------------------------------------------------
                # Ping
                # ------------------------------------------------------------
                elif msg_type == MSG_PING:
                    conn.sendall(bytes([MSG_PONG]))

                else:
                    log.warning(f"Unknown message type from {addr}: 0x{msg_type:02X}")
                    break

        except Exception as e:
            log.error(f"Handler error for {addr}: {e}")
        finally:
            conn.close()
            log.info(f"Disconnected {addr}")


def _load_config(path: str) -> tuple[dict, str, int]:
    """
    Parse a JSON config file and return (devices, host, port).

    Expected format:
    {
        "host": "0.0.0.0",   (optional)
        "port": 65432,        (optional)
        "devices": [
            {
                "name":        "mega138",
                "board":       "tangconsole",
                "uart":        "/dev/ttyUSB0",
                "baud":        115200,          (optional, default 115200)
                "ftdi_serial": "2025030317"     (optional)
            },
            ...
        ]
    }
    """
    raw = json.loads(Path(path).read_text())
    host = raw.get("host", SERVER_HOST)
    port = raw.get("port", SERVER_PORT)

    devices = {}
    for entry in raw["devices"]:
        name = entry["name"]
        if name in devices:
            raise ValueError(f"Duplicate device name in config: '{name}'")
        devices[name] = {
            "uart":        UARTBridge(entry["uart"], entry.get("baud", DEFAULT_UART_BAUD)),
            "board":       entry["board"],
            "ftdi_serial": entry.get("ftdi_serial"),
        }
    return devices, host, port


def main():
    p = argparse.ArgumentParser(
        description="GWCT server — single or multi-device mode",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Multi-device (config file):\n"
            "  gwct_server --config devices.json\n\n"
            "Single-device (command-line args):\n"
            "  gwct_server --board tangnano20k --uart /dev/ttyUSB0\n"
            "  gwct_server --board tangconsole  --uart /dev/ttyUSB0 --ftdi-serial 2025030317\n"
        ),
    )
    p.add_argument("--config", metavar="FILE",
                   help="JSON config file for multi-device mode")

    # Single-device arguments (used when --config is not given)
    p.add_argument("--uart",        default=DEFAULT_UART_PORT, help="UART device path")
    p.add_argument("--baud",        type=int, default=DEFAULT_UART_BAUD, help="UART baud rate")
    p.add_argument("--board",       default=DEFAULT_BOARD, help="openFPGALoader board name")
    p.add_argument("--ftdi-serial", metavar="SERIAL",
                   help="FTDI serial number passed to openFPGALoader (--ftdi-serial)")

    # Network
    p.add_argument("--host", default=SERVER_HOST)
    p.add_argument("--port", type=int, default=SERVER_PORT)

    args = p.parse_args()

    if args.config:
        try:
            devices, host, port = _load_config(args.config)
        except Exception as e:
            p.error(f"Failed to load config '{args.config}': {e}")
        # Command-line --host/--port override config file values
        host = args.host if args.host != SERVER_HOST else host
        port = args.port if args.port != SERVER_PORT else port
    else:
        devices = {
            "default": {
                "uart":        UARTBridge(args.uart, args.baud),
                "board":       args.board,
                "ftdi_serial": args.ftdi_serial,
            }
        }
        host = args.host
        port = args.port

    GWCTServer(devices, host, port).start()


if __name__ == "__main__":
    main()
