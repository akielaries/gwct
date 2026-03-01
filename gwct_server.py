#!/usr/bin/env python3
"""
gwct_server.py — GWCT server daemon
Runs on the machine physically connected to the FPGA.
Exposes two services on a single TCP port via a message-type prefix byte:

  MSG_MEMRD  = 0x01  — forward a GWCT UART packet, return 7-byte response
  MSG_MEMWR  = 0x02  — same
  MSG_LOAD   = 0x10  — receive bitstream, invoke openFPGALoader, return JSON result
  MSG_PING   = 0x20  — connectivity check, returns MSG_PONG

All messages start with a 1-byte type. The server is single-threaded per
connection to avoid UART contention.
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
MSG_UART = 0x01  # raw UART transaction: send 11 bytes, recv 7 bytes
MSG_LOAD = 0x10  # bitstream load
MSG_FLASH = 0x11  # bitstream load
MSG_PING = 0x20
MSG_PONG = 0x21
MSG_OK = 0xA0
MSG_ERR = 0xA1

# UART framing constants (must match gwct_packet.v)
UART_TX_LEN = 11
UART_RX_LEN = 7
UART_TIMEOUT = 2.0

DEFAULT_UART_PORT = "/dev/gwct_port_2a881d3a6c78529c21dc0423699e6be3"
DEFAULT_UART_BAUD = 115200
DEFAULT_BOARD = "tangnano20k"
SERVER_HOST = "0.0.0.0"
SERVER_PORT = 65432


class UARTBridge:
    """Thread-safe wrapper around the FPGA UART..."""

    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.ser = None
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


class GWCTServer:
    def __init__(
        self,
        uart_port=DEFAULT_UART_PORT,
        uart_baud=DEFAULT_UART_BAUD,
        board=DEFAULT_BOARD,
        host=SERVER_HOST,
        port=SERVER_PORT,
    ):
        self.uart = UARTBridge(uart_port, uart_baud)
        self.board = board
        self.host = host
        self.port = port

    def start(self):
        self.uart.open()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(5)
            log.info(f"GWCT server listening on {self.host}:{self.port}")
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
        self.uart.close()

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
            while True:
                # Each message starts with a 1-byte type
                hdr = conn.recv(1)
                if not hdr:
                    break
                msg_type = hdr[0]

                # --------------------------------------------------------
                # UART transaction — forward packet to FPGA, return response
                # --------------------------------------------------------
                if msg_type == MSG_UART:
                    pkt = self._recv_exact(conn, UART_TX_LEN)
                    try:
                        resp = self.uart.transact(pkt)
                        conn.sendall(bytes([MSG_OK]) + resp)
                    except Exception as e:
                        log.error(f"UART error: {e}")
                        conn.sendall(bytes([MSG_ERR]) + str(e).encode()[:64])

                # --------------------------------------------------------
                # Bitstream load
                # --------------------------------------------------------
                elif msg_type == MSG_LOAD:
                    # 4-byte file size
                    size_bytes = self._recv_exact(conn, 4)
                    file_size = struct.unpack(">I", size_bytes)[0]

                    # 1-byte board string length + board string
                    board_len = self._recv_exact(conn, 1)[0]
                    board = self._recv_exact(conn, board_len).decode()

                    # bitstream data — large transfer, use generous timeout
                    data = self._recv_exact(conn, file_size, timeout=120.0)
                    log.info(f"Received {file_size} bytes, board={board}")

                    with tempfile.NamedTemporaryFile(suffix=".fs", delete=False) as f:
                        f.write(data)
                        tmp = f.name
                    try:
                        # Stream openFPGALoader output to client line by line.
                        #
                        # Frame format: [2-byte big-endian length][line bytes]
                        # End-of-stream: length=0xFFFF followed by [4-byte signed rc]
                        #
                        # We use 0xFFFF as sentinel (not 0x0000) so an empty line
                        # can still be sent as length=0x0000 without confusion.
                        # openFPGALoader uses \r for progress lines -- we split on
                        # both \r and \n so progress bars flush immediately.
                        proc = subprocess.Popen(
                            ["openFPGALoader", "-v", "-b", board, tmp],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            bufsize=0,  # unbuffered bytes
                        )
                        buf = b""
                        while True:
                            ch = proc.stdout.read(1)
                            if not ch:
                                break
                            if ch in (b"\n", b"\r"):
                                if buf:
                                    line_bytes = buf[
                                        :65534
                                    ]  # max fits in 2-byte length
                                    conn.sendall(
                                        struct.pack(">H", len(line_bytes)) + line_bytes
                                    )
                                    buf = b""
                            else:
                                buf += ch
                        # flush any remaining partial line
                        if buf:
                            line_bytes = buf[:65534]
                            conn.sendall(
                                struct.pack(">H", len(line_bytes)) + line_bytes
                            )
                        proc.wait(timeout=120)
                        rc = proc.returncode
                        # end-of-stream sentinel 0xFFFF + 4-byte signed return code
                        conn.sendall(struct.pack(">H", 0xFFFF) + struct.pack(">i", rc))
                        log.info(f"Programming done: rc={rc}")
                    except Exception as e:
                        log.error(f"Programming error: {e}")
                        err_line = str(e).encode("utf-8")[:65534]
                        conn.sendall(struct.pack(">H", len(err_line)) + err_line)
                        conn.sendall(struct.pack(">H", 0xFFFF) + struct.pack(">i", -1))
                    finally:
                        if os.path.exists(tmp):
                            os.unlink(tmp)

                elif msg_type == MSG_FLASH:
                    # Same as MSG_LOAD but with -f flag
                    size_bytes = self._recv_exact(conn, 4)
                    file_size = struct.unpack(">I", size_bytes)[0]
                    board_len = self._recv_exact(conn, 1)[0]
                    board = self._recv_exact(conn, board_len).decode()
                    data = self._recv_exact(conn, file_size, timeout=180.0)  # Longer timeout

                    log.info(f"Flashing {file_size} bytes, board={board}")

                    with tempfile.NamedTemporaryFile(suffix=".fs", delete=False) as f:
                        f.write(data)
                        tmp = f.name
                    try:
                        proc = subprocess.Popen(
                            ["openFPGALoader", "-v", "-b", board, "-f", tmp],  # <-- Add -f
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
                                    line_bytes = buf[
                                        :65534
                                    ]  # max fits in 2-byte length
                                    conn.sendall(
                                        struct.pack(">H", len(line_bytes)) + line_bytes
                                    )
                                    buf = b""
                            else:
                                buf += ch
                        # flush any remaining partial line
                        if buf:
                            line_bytes = buf[:65534]
                            conn.sendall(
                                struct.pack(">H", len(line_bytes)) + line_bytes
                            )
                        proc.wait(timeout=120)
                        rc = proc.returncode
                        # end-of-stream sentinel 0xFFFF + 4-byte signed return code
                        conn.sendall(struct.pack(">H", 0xFFFF) + struct.pack(">i", rc))
                        log.info(f"Programming done: rc={rc}")
                    except Exception as e:
                        log.error(f"Programming error: {e}")
                        err_line = str(e).encode("utf-8")[:65534]
                        conn.sendall(struct.pack(">H", len(err_line)) + err_line)
                        conn.sendall(struct.pack(">H", 0xFFFF) + struct.pack(">i", -1))
                    finally:
                        if os.path.exists(tmp):
                            os.unlink(tmp)

                # --------------------------------------------------------
                # Ping
                # --------------------------------------------------------
                elif msg_type == MSG_PING:
                    conn.sendall(bytes([MSG_PONG]))

                else:
                    log.warning(f"Unknown message type: 0x{msg_type:02X}")
                    break

        except Exception as e:
            log.error(f"Handler error for {addr}: {e}")
        finally:
            conn.close()
            log.info(f"Disconnected {addr}")


def main():
    p = argparse.ArgumentParser(description="GWCT server")
    p.add_argument("--uart", default=DEFAULT_UART_PORT, help="UART device")
    p.add_argument("--baud", type=int, default=DEFAULT_UART_BAUD)
    p.add_argument("--board", default=DEFAULT_BOARD, help="openFPGALoader board type")
    p.add_argument("--host", default=SERVER_HOST)
    p.add_argument("--port", type=int, default=SERVER_PORT)
    args = p.parse_args()

    server = GWCTServer(
        uart_port=args.uart,
        uart_baud=args.baud,
        board=args.board,
        host=args.host,
        port=args.port,
    )
    server.start()


if __name__ == "__main__":
    main()
