# GWCT - Gowin Command Line Tool

A lightweight UART-based debug interface for Gowin FPGAs and accessing ABP busses. This
requires IO for TX and RX pins and takes up roughly 300 LUTs?

```
  Your laptop                    FPGA host machine               FPGA
  ----------                     -----------------               ----
  gwct.py  <--[ TCP (optional) --> gwct_server.py <--[ UART ]--> gwct_top
                                                  <--[ JTAG ]--> (programming)
```

https://github.com/user-attachments/assets/e62e02b8-c00c-496a-98c0-45aa27fa334e


---

## Requirements

**RTL**
- Verilog-2001 (tested in Gowin EDA 1.9.x)
- Any Gowin FPGA with spare GPIO and a synchronous reset
- CLK_HZ parameter must match your actual clock frequency

**Host tool**
- python3-tk
- Python 3.8+
- pyserial
- openFPGALoader (for `load` command only)

## Build
```
$ pip3 install .
```


---

## RTL integration

---

## Host tool

### Install

```bash
pip install pyserial
# openFPGALoader required only for the 'load' command
```

### Run - local (laptop/desktop connected directly to the FPGA)

```bash
python3 host/gwct.py --device mega60
```

### Run - remote (laptop on your network, FPGA connected to another machine)

On the machine with the FPGA attached:
```bash
python3 host/gwct_server.py --board tangconsole
```

On your laptop:
```bash
python3 host/gwct.py --device mega60 --remote 192.168.1.10
```

### Commands

```
gwct> memrd 0x60000000          read one 32-bit word
gwct> memrd 0x60000000 8        read 8 words
gwct> memwr 0x60000004 0x1      write a word
gwct> dump  0x60000000 16       hex dump 16 words (4 per line)
gwct> watch 0x60000010 500      poll every 500ms, flag changes with #
gwct> list hw                   probe device, print sysinfo registers
gwct> load  bitstream.fs        program FPGA via openFPGALoader (live progress)
gwct> exec  ls *.fs             run a host system command
gwct> script init.gwct          run a .gwct command script
gwct> help                      full command reference
```

Addresses and data values accept arithmetic and bitwise expressions:

```
gwct> memrd 0x60000000 + 4
gwct> memrd 0x60000000 + 0x10 * 2   8
gwct> memwr 0x60000000 + 8   0xDEAD & 0xFFFF
gwct> dump  0x6000_0000   32
```

Supported operators: `+ - * // % ** & | ^ ~ << >>`
Literals: `0xHEX  0bBINARY  0oOCTAL  DECIMAL  0x6000_0000`

Tab completion works for commands, file paths (load/script), and executables (exec).
Command history is saved to `~/.gwct_history` across sessions.

### Supported devices

| --device   | openFPGALoader board | Chip            |
|------------|----------------------|-----------------|
| mega60     | tangconsole          | GW5AT-60        |
| mega138    | tangconsole          | GW5AST-138      |
| nano1k     | tangnano1k           | GW1NZ-1         |
| nano4k     | tangnano4k           | GW1NSR-4C       |
| nano9k     | tangnano9k           | GW1NR-9         |
| nano20k    | tangnano20k          | GW2AR-18        |
| primer20k  | tangprimer20k        | GW2A-18C        |
| primer25k  | tangprimer25k        | GW5A-25         |

```bash
python3 host/gwct.py --list-devices
```

---

## Protocol reference

8N1 UART, 115200 baud (configurable via `BAUD` parameter).

**Host to FPGA - 11 bytes per command:**

| Bytes  | Field | Value                              |
|--------|-------|------------------------------------|
| 0      | Magic | 0x47 ('G')                         |
| 1      | CMD   | 0x01 = read,  0x02 = write         |
| 2..5   | ADDR  | 32-bit address, little-endian      |
| 6..9   | DATA  | 32-bit write data (0 for reads)    |
| 10     | CHK   | XOR of bytes 0..9                  |

**FPGA to host - 7 bytes per response:**

| Bytes  | Field | Value                              |
|--------|-------|------------------------------------|
| 0      | Magic | 0x47                               |
| 1      | CMD   | echo of CMD byte (0xFF on error)   |
| 2..5   | DATA  | 32-bit read result, little-endian  |
| 6      | CHK   | XOR of bytes 0..5                  |

Bad checksum: silently re-syncs to next 0x47 magic byte.
Unknown command: silently ignored, re-syncs.

---

## How it works

```
gwct_uart      - deserialises the 8N1 byte stream, 3-stage sync on RX
gwct_packet    - assembles 11-byte packets, validates XOR checksum,
                 fires cmd_valid pulse, waits for cmd_ready, sends 7-byte response
gwct_apb_master - drives APB3 SETUP/ACCESS state machine, returns PRDATA
gwct_top       - wires the three together, single instantiation boundary
```

The CPU is never halted. GWCT holds the APB bus for 3-4 clock cycles per
transaction by asserting PREADY=0 to the CPU, which is indistinguishable
from any slow APB slave inserting wait states. At 115200 baud there is a
minimum of ~96 microseconds between commands, during which the CPU runs freely.

---

