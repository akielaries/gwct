#!/usr/bin/env python3
"""
gwct - Gowin Command Line Tool
UART-based debug shell for Gowin FPGA / Cortex-M1 APB memory access.

Two transport modes:
  Local  - talk directly to UART (machine connected to FPGA)
  Remote - talk to gwct_server.py over TCP (laptop / remote machine)

Usage:
    gwct --device mega60                        # local UART, Tang Mega 60K
    gwct --device nano9k                        # local UART, Tang Nano 9K
    gwct --device mega60 --remote 192.168.1.10  # remote via gwct_server
    gwct --list-devices                         # show all known devices
"""

import serial
import struct
import sys
import time
import socket
import json
import os
import subprocess
import readline
import glob
import argparse
import ast
import operator
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Device map
# ---------------------------------------------------------------------------
DEVICE_MAP = {
    "mega60": {"board": "tangconsole", "desc": "Tang Mega 60K  (GW5AT-60)"},
    "mega138": {"board": "tangconsole", "desc": "Tang Mega 138K (GW5AST-138)"},
    "nano1": {"board": "tangnano1k", "desc": "Tang Nano 1K   (GW1NZ-1)"},
    "nano4": {"board": "tangnano4k", "desc": "Tang Nano 4K   (GW1NSR-4C)"},
    "nano9": {"board": "tangnano9k", "desc": "Tang Nano 9K   (GW1NR-9)"},
    "nano20": {"board": "tangnano20k", "desc": "Tang Nano 20K  (GW2AR-18)"},
    "primer20": {"board": "tangprimer20k", "desc": "Tang Primer 20K (GW2A-18C)"},
    "primer25": {"board": "tangprimer25k", "desc": "Tang Primer 25K (GW5A-25)"},
}

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
MAGIC = 0x47
CMD_READ = 0x01
CMD_WRITE = 0x02
CMD_ERROR = 0xFF
TX_LEN = 11
RX_LEN = 7

MSG_UART = 0x01
MSG_LOAD = 0x10
MSG_FLASH = 0x11
MSG_PING = 0x20
MSG_PONG = 0x21
MSG_OK = 0xA0
MSG_ERR = 0xA1

DEFAULT_UART_PORT = "/dev/gwct_port_2a881d3a6c78529c21dc0423699e6be3"
DEFAULT_UART_BAUD = 115200
DEFAULT_SERVER_PORT = 65432
TIMEOUT_S = 5.0  # UART transactions
LOAD_TIMEOUT_S = 120.0  # bitstream transfer + programming

HISTORY_FILE = Path.home() / ".gwct_history"

FILE_COMMANDS = {"program", "flash", "script", "tcl"}

ALL_COMMANDS = [
    "memrd",
    "memwr",
    "dump",
    "program",
    "flash",
    "watch",
    "reset",
    "list",
    "info",
    "script",
    "tcl",
    "exec",
    "reconnect",
    "help",
    "quit",
    "exit",
]

# ---------------------------------------------------------------------------
# Safe integer expression evaluator
# ---------------------------------------------------------------------------

_BINOPS = {
    ast.Add: operator.add,
    ast.Sub: operator.sub,
    ast.Mult: operator.mul,
    ast.FloorDiv: operator.floordiv,
    ast.Mod: operator.mod,
    ast.Pow: operator.pow,
    ast.BitAnd: operator.and_,
    ast.BitOr: operator.or_,
    ast.BitXor: operator.xor,
    ast.LShift: operator.lshift,
    ast.RShift: operator.rshift,
}

_UNOPS = {
    ast.USub: operator.neg,
    ast.UAdd: operator.pos,
    ast.Invert: operator.invert,
}


def _eval_node(node):
    if isinstance(node, ast.Expression):
        return _eval_node(node.body)
    if isinstance(node, ast.Constant):
        if isinstance(node.value, int):
            return node.value
        raise ValueError(f"non-integer literal: {node.value!r}")
    if isinstance(node, ast.BinOp):
        op_fn = _BINOPS.get(type(node.op))
        if op_fn is None:
            raise ValueError(f"unsupported operator: {type(node.op).__name__}")
        left = _eval_node(node.left)
        right = _eval_node(node.right)
        if isinstance(node.op, ast.Pow) and right > 64:
            raise ValueError("exponent too large")
        if isinstance(node.op, (ast.LShift, ast.RShift)) and right > 64:
            raise ValueError("shift too large")
        return op_fn(left, right)
    if isinstance(node, ast.UnaryOp):
        op_fn = _UNOPS.get(type(node.op))
        if op_fn is None:
            raise ValueError(f"unsupported unary operator: {type(node.op).__name__}")
        return op_fn(_eval_node(node.operand))
    raise ValueError(f"unsupported expression node: {type(node).__name__}")


def parse_int(expr: str) -> int:
    expr = expr.strip()
    try:
        tree = ast.parse(expr, mode="eval")
        result = _eval_node(tree)
        return result & 0xFFFFFFFF
    except (ValueError, SyntaxError, ZeroDivisionError) as e:
        raise ValueError(f"bad expression {expr!r}: {e}") from None


# ---------------------------------------------------------------------------
# Packet helpers
# ---------------------------------------------------------------------------


def _xor(data: bytes) -> int:
    r = 0
    for b in data:
        r ^= b
    return r


def build_packet(cmd: int, addr: int, data: int = 0) -> bytes:
    body = struct.pack("<BBII", MAGIC, cmd, addr, data)
    return body + bytes([_xor(body)])


def parse_response(raw: bytes) -> tuple:
    if len(raw) != RX_LEN:
        raise ValueError(f"Expected {RX_LEN} bytes, got {len(raw)}: {raw.hex()}")
    magic, cmd, d0, d1, d2, d3, chk = struct.unpack("BBBBBBB", raw)
    if magic != MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:02X}")
    if chk != _xor(raw[:6]):
        raise ValueError(
            f"Checksum mismatch: got 0x{chk:02X} expected 0x{_xor(raw[:6]):02X}"
        )
    data = d0 | (d1 << 8) | (d2 << 16) | (d3 << 24)
    return (cmd != CMD_ERROR), cmd, data


# ---------------------------------------------------------------------------
# Tcl interpreter
# ---------------------------------------------------------------------------


def _make_tcl_interpreter(gwct_obj):
    """
    Build a Tcl interpreter with gwct commands registered.

    Requires tkinter (ships with Python but may need python3-tk on some
    Linux distros). If unavailable, returns None and the caller falls back
    to tclsh subprocess mode.

    Registered Tcl commands:
        mrd  <addr>          -> returns value as hex string "0xXXXXXXXX"
        mwr  <addr> <data>   -> returns ""
        dump <addr> <count>  -> returns multi-line hex dump string
        puts <msg>           -> prints to stdout (standard Tcl puts still works)

    All addr/data arguments accept the same arithmetic expressions as the
    interactive shell (0x hex, +, -, &, |, >>, etc.).

    Example .tcl script:
        set base 0x60000000
        set magic [mrd $base]
        puts "magic = $magic"
        if { $magic eq "0xDEADBEEF" } {
            puts "sysinfo OK"
            mwr [expr {$base + 0x20}] 0x00000001
        }
    """
    try:
        import tkinter

        tcl = tkinter.Tcl()
    except Exception:
        return None

    def _tcl_int(s):
        """Parse a Tcl integer arg - handles 0x hex and arithmetic."""
        s = str(s).strip()
        # Tcl already evaluates [expr ...] before calling us, so we just
        # need to handle plain hex/decimal literals and our parse_int exprs.
        return parse_int(s)

    def tcl_mrd(addr_str):
        addr = _tcl_int(addr_str)
        val = gwct_obj.memrd(addr)
        return f"0x{val:08X}"

    def tcl_mwr(addr_str, data_str):
        addr = _tcl_int(addr_str)
        data = _tcl_int(data_str)
        gwct_obj.memwr(addr, data)
        return ""

    def tcl_dump(addr_str, count_str):
        addr = _tcl_int(addr_str)
        count = _tcl_int(count_str)
        lines = []
        for row in range(0, count, 4):
            ra = (addr + row * 4) & 0xFFFFFFFF
            words = []
            for col in range(4):
                if row + col < count:
                    words.append(f"{gwct_obj.memrd((ra + col * 4) & 0xFFFFFFFF):08X}")
                else:
                    words.append("        ")
            lines.append(f"0x{ra:08X}:  {'  '.join(words)}")
        return "\n".join(lines)

    def tcl_program(path_str):
        path = str(path_str)
        if not Path(path).exists():
            raise Exception(f"file not found: {path}")
        size = Path(path).stat().st_size
        print(f"  loading {path}  ({size:,} bytes)  board={gwct_obj.board} ...")
        result = gwct_obj.load(path)
        print_load_result(result)
        return "0" if result["returncode"] == 0 else "1"

    def tcl_flash(path_str):
        path = str(path_str)
        if not Path(path).exists():
            raise Exception(f"file not found: {path}")
        size = Path(path).stat().st_size
        print(f"  flashing {path}  ({size:,} bytes)  board={gwct_obj.board} ...")
        result = gwct_obj.flash(path)
        print_load_result(result)
        return "0" if result["returncode"] == 0 else "1"

    def tcl_sleep(ms_str):
        ms = int(str(ms_str))
        time.sleep(ms / 1000.0)
        return ""

    def tcl_gwct_version():
        return "0.4.0"

    # Register all commands
    tcl.createcommand("mrd", tcl_mrd)
    tcl.createcommand("mwr", tcl_mwr)
    tcl.createcommand("dump", tcl_dump)
    tcl.createcommand("program", tcl_program)
    tcl.createcommand("flash", tcl_flash)
    tcl.createcommand("sleep_ms", tcl_sleep)
    tcl.createcommand("gwct_version", tcl_gwct_version)

    # Convenience aliases matching xsct naming style
    tcl.eval(
        """
        proc memrd  {addr}       { mrd $addr }
        proc memwr  {addr data}  { mwr $addr $data }
        proc after_ms {ms}       { sleep_ms $ms }
    """
    )

    return tcl


def run_tcl_file(gwct_obj, path: str):
    """
    Execute a Tcl script file using the embedded interpreter.
    Falls back to spawning tclsh with gwct stubs if tkinter is unavailable.
    """
    p = Path(path)
    if not p.exists():
        print(f"  [error] tcl script not found: {path}")
        return

    tcl = _make_tcl_interpreter(gwct_obj)

    if tcl is not None:
        # Embedded path full gwct integration
        print(f"  running tcl: {path}")
        try:
            tcl.eval(p.read_text())
        except Exception as e:
            # Strip the Tcl stack trace down to just the useful part
            msg = str(e)
            # Tcl errors look like: "some error\n    while executing\n..."
            first_line = msg.split("\n")[0]
            print(f"  [tcl error] {first_line}")
    else:
        # Fallback try system tclsh, with a warning that mrd/mwr won't work
        _run_tcl_tclsh_fallback(path)


def _run_tcl_tclsh_fallback(path: str):
    """Last-resort fallback: run the script under system tclsh."""
    tclsh = None
    for candidate in ("tclsh", "tclsh8.6", "tclsh9.0"):
        try:
            subprocess.run([candidate, "--version"], capture_output=True)
            tclsh = candidate
            break
        except FileNotFoundError:
            continue

    if tclsh is None:
        print("  [error] tcl support requires either:")
        print("          - python3-tk  (recommended: sudo apt install python3-tk)")
        print("          - tclsh       (sudo apt install tcl)")
        return

    print(f"  [warning] running under {tclsh} - mrd/mwr/dump not available")
    print(f"  running tcl: {path}")
    result = subprocess.run([tclsh, path], text=True)
    if result.returncode != 0:
        print(f"  [tcl] exited with code {result.returncode}")


def run_tcl_inline(gwct_obj, expr: str):
    """Evaluate a single Tcl expression string (used by 'tcl -e ...' syntax)."""
    tcl = _make_tcl_interpreter(gwct_obj)
    if tcl is None:
        print("  [error] tcl support requires python3-tk (sudo apt install python3-tk)")
        return
    try:
        result = tcl.eval(expr)
        if result:
            print(f"  {result}")
    except Exception as e:
        first_line = str(e).split("\n")[0]
        print(f"  [tcl error] {first_line}")


# ---------------------------------------------------------------------------
# Tab completion
# ---------------------------------------------------------------------------


class GWCTCompleter:
    def __init__(self):
        self._matches = []

    def complete(self, text: str, state: int) -> Optional[str]:
        if state == 0:
            self._matches = self._get_matches(text)
        try:
            return self._matches[state]
        except IndexError:
            return None

    def _get_matches(self, text: str) -> list:
        line = readline.get_line_buffer()
        tokens = line[: readline.get_endidx()].split()

        completing_cmd = len(tokens) == 0 or (
            len(tokens) == 1 and not line.endswith(" ")
        )

        if completing_cmd:
            return [c + " " for c in ALL_COMMANDS if c.startswith(text)]

        cmd = tokens[0].lower()

        if cmd == "list":
            return [s for s in ["hw "] if s.startswith(text)]

        if cmd in FILE_COMMANDS:
            return self._file_matches(text, extensions=[".fs", ".gwct", ".tcl", ""])

        if cmd == "exec":
            if len(tokens) == 1 or (len(tokens) == 2 and not line.endswith(" ")):
                return self._executable_matches(text) + self._file_matches(text)
            else:
                return self._file_matches(text)

        return []

    def _file_matches(self, text: str, extensions: list = None) -> list:
        pattern = (text or "") + "*"
        try:
            matches = glob.glob(os.path.expanduser(pattern))
        except Exception:
            return []
        results = []
        for m in sorted(matches):
            if os.path.isdir(m):
                results.append(m.rstrip("/") + "/")
            elif extensions is None or any(m.endswith(e) for e in extensions):
                results.append(m + " ")
        return results

    def _executable_matches(self, text: str) -> list:
        matches = set()
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            try:
                for name in os.listdir(directory):
                    if name.startswith(text):
                        full = os.path.join(directory, name)
                        if os.access(full, os.X_OK):
                            matches.add(name + " ")
            except OSError:
                pass
        return sorted(matches)


def setup_readline():
    try:
        readline.read_history_file(HISTORY_FILE)
    except FileNotFoundError:
        pass
    readline.set_history_length(500)

    completer = GWCTCompleter()
    readline.set_completer(completer.complete)

    if "libedit" in readline.__doc__:
        readline.parse_and_bind("bind ^I rl_complete")
    else:
        readline.parse_and_bind("tab: complete")

    readline.set_completer_delims(
        readline.get_completer_delims().replace("/", "").replace("-", "")
    )


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------


class LocalTransport:
    def __init__(self, port: str, baud: int = DEFAULT_UART_BAUD):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT_S,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def uart_transact(self, pkt: bytes) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        self.ser.flush()
        raw = b""
        deadline = time.monotonic() + TIMEOUT_S
        while len(raw) < RX_LEN:
            chunk = self.ser.read(RX_LEN - len(raw))
            if chunk:
                raw += chunk
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Timeout: got {len(raw)}/{RX_LEN} bytes: {raw.hex()}"
                )
        return raw

    def load_bitstream(self, path: str, board: str) -> dict:
        result = subprocess.run(
            ["openFPGALoader", "-v", "-b", board, path],
            capture_output=True,
            text=True,
            timeout=60,
        )
        return {
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }

    def flash_bitstream(self, path: str, board: str) -> dict:
        """Flash to SPI flash instead of SRAM."""
        result = subprocess.run(
            ["openFPGALoader", "-v", "-b", board, "-f", path],  # <-- Add -f flag
            capture_output=True,
            text=True,
            timeout=240,  # Flash takes longer than SRAM programming
        )
        return {
            "returncode": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }

    def ping(self) -> bool:
        return True

    def describe(self) -> str:
        return f"local UART {self.port} @ {self.baud}"


class RemoteTransport:
    def __init__(self, host: str, port: int = DEFAULT_SERVER_PORT):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(TIMEOUT_S)
        self.sock.connect((self.host, self.port))

    def disconnect(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _recv_exact(self, n: int, timeout: float = None) -> bytes:
        timeout = timeout or TIMEOUT_S
        buf = b""
        deadline = time.monotonic() + timeout
        while len(buf) < n:
            remaining_t = max(0.1, deadline - time.monotonic())
            self.sock.settimeout(remaining_t)
            try:
                chunk = self.sock.recv(n - len(buf))
            except socket.timeout:
                chunk = b""
            if chunk:
                buf += chunk
            if time.monotonic() > deadline:
                raise TimeoutError(f"TCP recv timeout: got {len(buf)}/{n}")
        self.sock.settimeout(TIMEOUT_S)
        return buf

    def uart_transact(self, pkt: bytes) -> bytes:
        self.sock.sendall(bytes([MSG_UART]) + pkt)
        status = self._recv_exact(1)[0]
        if status == MSG_OK:
            return self._recv_exact(RX_LEN)
        err = self._recv_exact(64).rstrip(b"\x00").decode(errors="replace")
        raise RuntimeError(f"Server UART error: {err}")

    def load_bitstream(self, path: str, board: str) -> dict:
        data = Path(path).read_bytes()
        board_enc = board.encode()
        file_size = len(data)

        self.sock.sendall(
            bytes([MSG_LOAD])
            + struct.pack(">I", file_size)
            + bytes([len(board_enc)])
            + board_enc
        )

        chunk_size = 65536
        sent = 0
        while sent < file_size:
            chunk = data[sent : sent + chunk_size]
            self.sock.sendall(chunk)
            sent += len(chunk)
            pct = sent / file_size * 100
            print(
                f"  uploading... {pct:5.1f}%  ({sent:,}/{file_size:,} bytes)", end="\r"
            )
        print()

        print("  programming...")
        self.sock.settimeout(LOAD_TIMEOUT_S)
        try:
            while True:
                hdr = self._recv_exact(2, timeout=LOAD_TIMEOUT_S)
                length = struct.unpack(">H", hdr)[0]
                if length == 0xFFFF:
                    rc_bytes = self._recv_exact(4, timeout=LOAD_TIMEOUT_S)
                    rc = struct.unpack(">i", rc_bytes)[0]
                    return {"returncode": rc, "stdout": "", "stderr": ""}
                if length == 0:
                    continue
                line = self._recv_exact(length, timeout=LOAD_TIMEOUT_S)
                print(f"  {line.decode('utf-8', errors='replace')}")
        finally:
            self.sock.settimeout(TIMEOUT_S)

    def flash_bitstream(self, path: str, board: str) -> dict:
        data = Path(path).read_bytes()
        board_enc = board.encode()
        file_size = len(data)

        # Send MSG_FLASH (new message type) instead of MSG_LOAD
        self.sock.sendall(
            bytes([MSG_FLASH])  # <-- New message type
            + struct.pack(">I", file_size)
            + bytes([len(board_enc)])
            + board_enc
        )

        chunk_size = 65536
        sent = 0
        while sent < file_size:
            chunk = data[sent : sent + chunk_size]
            self.sock.sendall(chunk)
            sent += len(chunk)
            pct = sent / file_size * 100
            print(
                f"  uploading... {pct:5.1f}%  ({sent:,}/{file_size:,} bytes)", end="\r"
            )
        print()

        print("  programming...")
        self.sock.settimeout(LOAD_TIMEOUT_S)
        try:
            while True:
                hdr = self._recv_exact(2, timeout=LOAD_TIMEOUT_S)
                length = struct.unpack(">H", hdr)[0]
                if length == 0xFFFF:
                    rc_bytes = self._recv_exact(4, timeout=LOAD_TIMEOUT_S)
                    rc = struct.unpack(">i", rc_bytes)[0]
                    return {"returncode": rc, "stdout": "", "stderr": ""}
                if length == 0:
                    continue
                line = self._recv_exact(length, timeout=LOAD_TIMEOUT_S)
                print(f"  {line.decode('utf-8', errors='replace')}")
        finally:
            self.sock.settimeout(TIMEOUT_S)

    def ping(self) -> bool:
        try:
            self.sock.sendall(bytes([MSG_PING]))
            return self._recv_exact(1)[0] == MSG_PONG
        except Exception:
            return False

    def describe(self) -> str:
        return f"remote {self.host}:{self.port}"


# ---------------------------------------------------------------------------
# GWCT device
# ---------------------------------------------------------------------------


class GWCT:
    def __init__(self, transport, device_alias: str):
        self.transport = transport
        self.device_alias = device_alias
        self.device_info = DEVICE_MAP.get(device_alias, {})
        self.board = self.device_info.get("board", device_alias)

    def connect(self):
        self.transport.connect()

    def disconnect(self):
        self.transport.disconnect()

    def memrd(self, addr: int) -> int:
        pkt = build_packet(CMD_READ, addr)
        raw = self.transport.uart_transact(pkt)
        ok, _, data = parse_response(raw)
        if not ok:
            raise RuntimeError(f"FPGA error on read @ 0x{addr:08X}")
        return data

    def memwr(self, addr: int, data: int):
        pkt = build_packet(CMD_WRITE, addr, data)
        raw = self.transport.uart_transact(pkt)
        ok, _, _ = parse_response(raw)
        if not ok:
            raise RuntimeError(f"FPGA error on write @ 0x{addr:08X}")

    def load(self, path: str) -> dict:
        return self.transport.load_bitstream(path, self.board)

    def flash(self, path: str) -> dict:
        return self.transport.flash_bitstream(path, self.board)

    def ping(self) -> bool:
        return self.transport.ping()


# ---------------------------------------------------------------------------
# Command implementations
# ---------------------------------------------------------------------------


def print_load_result(result: dict):
    if result["stdout"]:
        print(result["stdout"])
    if result["stderr"]:
        print(result["stderr"])
    rc = result["returncode"]
    print(
        f"  {'OK programming succeeded' if rc == 0 else f'FAIL programming failed (rc={rc})'}"
    )


def cmd_list_hw(gwct: GWCT):
    info = gwct.device_info
    print(f"\n  device    : {gwct.device_alias}  -  {info.get('desc', '(unknown)')}")
    print(f"  ofl board : {gwct.board}")
    print(f"  transport : {gwct.transport.describe()}")
    try:
        magic = gwct.memrd(0x60000000)
        mfg_a = gwct.memrd(0x60000004)
        mfg_b = gwct.memrd(0x60000008)
        ver = gwct.memrd(0x6000000C)
        mfg_str = mfg_a.to_bytes(4, "big").decode(
            "ascii", errors="replace"
        ) + mfg_b.to_bytes(4, "big").decode("ascii", errors="replace")
        ver_str = f"{(ver>>16)&0xFFFF}.{(ver>>8)&0xFF}.{ver&0xFF}"
        print(f"\n  sysinfo:")
        print(f"    magic   : 0x{magic:08X}  {'OK' if magic == 0xDEADBEEF else 'FAIL'}")
        print(f"    mfg     : {mfg_str}")
        print(f"    version : {ver_str}")
    except Exception as e:
        print(f"\n  sysinfo  : [error - {e}]")
    print()


def cmd_exec(parts: list):
    if len(parts) < 2:
        print("  usage: exec <command> [args...]")
        return
    host_cmd = parts[1:]
    try:
        result = subprocess.run(host_cmd, text=True, stdout=None, stderr=None)
        if result.returncode != 0:
            print(f"  [exec] exited with code {result.returncode}")
    except FileNotFoundError:
        print(f"  [exec] command not found: {host_cmd[0]}")
    except Exception as e:
        print(f"  [exec] error: {e}")


def run_command(gwct: GWCT, line: str, echo: bool = False) -> bool:
    line = line.strip()
    if not line or line.startswith("#"):
        return True
    if echo:
        print(f"gwct> {line}")

    parts = line.split()
    cmd = parts[0].lower()

    if cmd in ("quit", "exit", "q"):
        return False

    elif cmd == "help":
        print(HELP_TEXT)

    elif cmd == "list":
        if len(parts) >= 2 and parts[1].lower() == "hw":
            cmd_list_hw(gwct)
        else:
            print("  usage: list hw")

    elif cmd == "info":
        cmd_list_hw(gwct)

    elif cmd == "memrd":
        if len(parts) < 2:
            print("  usage: memrd <addr> [count]")
        else:
            try:
                tail_parts = parts[1:]
                if len(tail_parts) >= 2:
                    try:
                        count = parse_int(tail_parts[-1])
                        addr = parse_int(" ".join(tail_parts[:-1]))
                    except ValueError:
                        addr = parse_int(" ".join(tail_parts))
                        count = 1
                else:
                    addr = parse_int(tail_parts[0])
                    count = 1
                for i in range(count):
                    a = (addr + i * 4) & 0xFFFFFFFF
                    val = gwct.memrd(a)
                    print(f"  0x{a:08X} :   0x{val:08X}  ({val})")
            except Exception as e:
                print(f"  [error] {e}")

    elif cmd == "memwr":
        if len(parts) < 3:
            print("  usage: memwr <addr> <data>")
        else:
            try:
                data = parse_int(parts[-1])
                addr = parse_int(" ".join(parts[1:-1]))
                gwct.memwr(addr, data)
                print(f"  0x{addr:08X} :    0x{data:08X}  OK")
            except Exception as e:
                print(f"  [error] {e}")

    elif cmd == "dump":
        if len(parts) < 3:
            print("  usage: dump <addr> <count>")
        else:
            try:
                count = parse_int(parts[-1])
                addr = parse_int(" ".join(parts[1:-1]))
                for row in range(0, count, 4):
                    ra = (addr + row * 4) & 0xFFFFFFFF
                    words = []
                    for col in range(4):
                        if row + col < count:
                            words.append(f"{gwct.memrd((ra + col*4) & 0xFFFFFFFF):08X}")
                        else:
                            words.append("        ")
                    print(f"  0x{ra:08X}:  {'  '.join(words)}")
            except Exception as e:
                print(f"  [error] {e}")

    elif cmd == "program":
        if len(parts) < 2:
            print("  usage: program <file.fs>")
        else:
            path = parts[1]
            if not Path(path).exists():
                print(f"  [error] file not found: {path}")
            else:
                size = Path(path).stat().st_size
                print(f"  loading {path}  ({size:,} bytes)  board={gwct.board} ...")
                print_load_result(gwct.load(path))

    elif cmd == "flash":
        if len(parts) < 2:
            print("  usage: flash <file.fs>")
        else:
            path = parts[1]
            if not Path(path).exists():
                print(f"  [error] file not found: {path}")
            else:
                size = Path(path).stat().st_size
                print(f"  flashing {path}  ({size:,} bytes)  board={gwct.board} ...")
                print(f"  [warning] this writing to SPI flash will take ~60 seconds")
                result = gwct.flash(path)
                print_load_result(result)

    elif cmd == "watch":
        if len(parts) < 2:
            print("  usage: watch <addr> [interval_ms]")
        else:
            try:
                if len(parts) >= 3:
                    try:
                        interval_ms = parse_int(parts[-1])
                        addr = parse_int(" ".join(parts[1:-1]))
                    except ValueError:
                        addr = parse_int(" ".join(parts[1:]))
                        interval_ms = 500
                else:
                    addr = parse_int(" ".join(parts[1:]))
                    interval_ms = 500
                interval = interval_ms / 1000.0
                print(
                    f"  watching 0x{addr:08X} every {interval_ms} ms  (Ctrl-C to stop)\n"
                )
                prev = None
                try:
                    while True:
                        val = gwct.memrd(addr)
                        ts = time.strftime("%H:%M:%S")
                        marker = "  #" if (prev is not None and val != prev) else ""
                        print(
                            f"  [{ts}]  0x{addr:08X} :    0x{val:08X}  ({val}){marker}"
                        )
                        prev = val
                        time.sleep(interval)
                except KeyboardInterrupt:
                    print()
            except Exception as e:
                print(f"  [error] {e}")

    elif cmd == "reset":
        RESET_ADDR = 0x60000020
        gwct.memwr(RESET_ADDR, 0x00000001)
        print(f"  reset written to 0x{RESET_ADDR:08X}")

    elif cmd == "exec":
        cmd_exec(parts)

    elif cmd == "tcl":
        # tcl <file.tcl>           - run a tcl script file
        # tcl -e <expression>      - evaluate a single tcl expression
        if len(parts) < 2:
            print("  usage: tcl <file.tcl>")
            print("         tcl -e <expression>")
        elif parts[1] == "-e":
            if len(parts) < 3:
                print("  usage: tcl -e <expression>")
            else:
                expr = " ".join(parts[2:])
                run_tcl_inline(gwct, expr)
        else:
            run_tcl_file(gwct, parts[1])

    elif cmd == "script":
        if len(parts) < 2:
            print("  usage: script <file.gwct>")
        else:
            run_script(gwct, parts[1])

    elif cmd == "reconnect":
        gwct.disconnect()
        try:
            gwct.connect()
            print("  reconnected")
        except Exception as e:
            print(f"  [error] {e}")

    else:
        print(f"  unknown command: '{cmd}'  (type 'help')")

    return True


def run_script(gwct: GWCT, path: str):
    p = Path(path)
    if not p.exists():
        print(f"  [error] script not found: {path}")
        return
    print(f"  running script: {path}")
    for i, raw_line in enumerate(p.read_text().splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        readline.add_history(line)
        try:
            if not run_command(gwct, line, echo=True):
                break
        except Exception as e:
            print(f"  [error] line {i}: {e}")
            break


# ---------------------------------------------------------------------------
# Shell
# ---------------------------------------------------------------------------

HELP_TEXT = """
Gowin Command Line Tool commands:

  -- memory --
  memrd  <addr> [count]        read 32-bit word(s)
  memwr  <addr> <data>         write 32-bit word
  dump   <addr> <count>        hex dump (4 words/line)
  watch  <addr> [interval_ms]  poll register until Ctrl-C

  Addresses and values accept arithmetic expressions:
    memrd 0x60000000 + 4
    memrd 0x60000000 + 0x10 * 2
    memwr 0x60000000 + 8   0xDEAD & 0xFFFF
  Operators: + - * // % ** & | ^ ~ << >>

  -- device --
  list hw                      show device + sysinfo registers
  info                         alias for list hw
  program <file.fs>            program FPGA bitstream         [tab complete]
  flash   <file.fs>            flash FPGA bitstream (SPI flash) [tab complete]
  reset                        soft reset via register write

  -- scripting --
  tcl    <file.tcl>            run a Tcl script               [tab complete]
  tcl    -e <expression>       evaluate a single Tcl expression
  script <file.gwct>           run a gwct script file         [tab complete]

  Tcl commands available inside .tcl scripts:
    mrd  <addr>          read register, returns "0xXXXXXXXX"
    mwr  <addr> <data>   write register
    dump <addr> <count>  hex dump, returns string
    program <f>          program bitstream, returns 0/1
    flash <f>            flash bitstream, returns 0/1
    sleep_ms <ms>        sleep
    memrd / memwr        aliases for mrd / mwr

  -- shell --
  exec   <cmd> [args...]       run a host system command      [tab complete]
  reconnect                    reopen connection
  help                         this message
  quit / exit / q              exit

  Tcl support requires python3-tk:
    sudo apt install python3-tk    (Debian/Ubuntu)
    brew install python-tk         (macOS)
"""

BANNER = """
  Gowin Command Line Tool
  ==============================
"""


def run_shell(gwct: GWCT):
    print(BANNER)
    print(
        f"  device    : {gwct.device_alias}  -  {gwct.device_info.get('desc', '(unknown)')}"
    )
    print(f"  transport : {gwct.transport.describe()}")
    print(f"\n  type 'help' for commands, 'list hw' to probe device\n")

    setup_readline()

    while True:
        try:
            line = input("gwct> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        try:
            if not run_command(gwct, line):
                break
        except Exception as e:
            print(f"  [error] {e}")

    readline.write_history_file(HISTORY_FILE)
    print("[gwct] bye.....")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def list_devices():
    print("\nKnown GWCT device aliases:\n")
    print(f"  {'alias':<12}  {'openFPGALoader board':<22}  description")
    print(f"  {'-'*12}  {'-'*22}  {'-'*30}")
    for alias, info in DEVICE_MAP.items():
        print(f"  {alias:<12}  {info['board']:<22}  {info['desc']}")
    print()


def main():
    p = argparse.ArgumentParser(prog="gwct", description="Gowin Command Line Tool")
    p.add_argument(
        "--device",
        "-d",
        metavar="DEVICE",
        help="Target device alias (e.g. mega60, nano9k). Use --list-devices to see all.",
    )
    p.add_argument(
        "--list-devices",
        action="store_true",
        help="List all known device aliases and exit",
    )
    p.add_argument(
        "--remote",
        metavar="HOST",
        help="Connect to gwct_server at HOST instead of local UART",
    )
    p.add_argument(
        "--server-port", type=int, default=DEFAULT_SERVER_PORT, metavar="PORT"
    )
    p.add_argument("--port", default=DEFAULT_UART_PORT, help="Local UART device path")
    p.add_argument(
        "--baud", type=int, default=DEFAULT_UART_BAUD, help="Local UART baud rate"
    )
    p.add_argument(
        "--tcl", metavar="SCRIPT", help="Run a Tcl script non-interactively and exit"
    )
    args = p.parse_args()

    if args.list_devices:
        list_devices()
        sys.exit(0)

    if not args.device:
        p.error("--device is required. Use --list-devices to see options.")

    alias = args.device.lower()
    if alias not in DEVICE_MAP:
        print(f"[gwct] unknown device '{alias}'.")
        list_devices()
        sys.exit(1)

    transport = (
        RemoteTransport(args.remote, args.server_port)
        if args.remote
        else LocalTransport(args.port, args.baud)
    )

    gwct_inst = GWCT(transport, alias)
    try:
        gwct_inst.connect()
    except Exception as e:
        print(f"[gwct] connection failed: {e}")
        sys.exit(1)

    try:
        if args.tcl:
            # Non-interactive Tcl mode: run script and exit
            run_tcl_file(gwct_inst, args.tcl)
        else:
            run_shell(gwct_inst)
    finally:
        gwct_inst.disconnect()


if __name__ == "__main__":
    main()
