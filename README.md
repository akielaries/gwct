# GWCT - Gowin Command Line Tool

A lightweight UART-based debug interface for Gowin FPGAs. Add four small
RTL files to your project, wire two UART pins, and get interactive read/write
access to your entire APB register map from a host shell

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
- Python 3.8+
- pyserial
- openFPGALoader (for `load` command only)

---

## Project layout

```
gwct/
  rtl/
    gwct_top.v          top-level wrapper. this is the only file you instantiate
    gwct_uart.v         8N1 UART RX/TX with 3-stage metastability sync
    gwct_packet.v       11-byte request / 7-byte response framing FSM
    gwct_apb_master.v   APB3 SETUP/ACCESS master FSM
  host/
    gwct.py             interactive debug shell (run this on your laptop)
    gwct_server.py      optional server daemon for remote access
  example/
    example_top.v       complete worked example: GWCT + APB mux
  README.md
```

---

## RTL integration

### Step 1 - Add files to your project

Add all four files from `rtl/` to your Gowin EDA project. They have no
external IP dependencies... pure Verilog-2001.

### Step 2 - Instantiate gwct_top

`gwct_top` is the only module you need to touch. It wires the UART, packet
framer, and APB master together internally.

```verilog
gwct_top #(
    .CLK_HZ (50_000_000),    // Hz must match your HCLK
    .BAUD   (115_200)        // must match host tool setting
) gwct (
    .clk     (HCLK),
    .rstn    (hwRstn),

    // Two UART pins connect to a USB-UART adapter
    .gwct_rx (GWCT_RX),
    .gwct_tx (GWCT_TX),

    // APB master bus signals connect to your APB mux (see Step 3)
    .PADDR   (gwct_PADDR),
    .PSEL    (gwct_PSEL),
    .PENABLE (gwct_PENABLE),
    .PWRITE  (gwct_PWRITE),
    .PWDATA  (gwct_PWDATA),
    .PSTRB   (gwct_PSTRB),
    .PPROT   (gwct_PPROT),

    // APB slave response shared with CPU path
    .PRDATA  (slave_PRDATA),
    .PREADY  (slave_PREADY),
    .PSLVERR (slave_PSLVERR)
);
```

### Step 3 - Add a 2-way APB mux

GWCT is a second APB master alongside your Cortex-M1. You need a simple
priority mux in front of your APB slave(s). GWCT gets the bus when it has
an active transaction (gwct_PSEL=1), which lasts 3-4 APB clock cycles.
During that window the CPU sees PREADY=0 and stalls harmlessly.

```verilog
// Declare GWCT APB wires
wire [31:0] gwct_PADDR;
wire        gwct_PSEL, gwct_PENABLE, gwct_PWRITE;
wire [31:0] gwct_PWDATA;
wire [3:0]  gwct_PSTRB;
wire [2:0]  gwct_PPROT;

// Mux GWCT wins when gwct_PSEL=1
wire mux_sel = gwct_PSEL;

wire [31:0] mux_PADDR   = mux_sel ? gwct_PADDR   : cpu_PADDR;
wire        mux_PSEL    = mux_sel ? gwct_PSEL     : cpu_PSEL;
wire        mux_PENABLE = mux_sel ? gwct_PENABLE  : cpu_PENABLE;
wire        mux_PWRITE  = mux_sel ? gwct_PWRITE   : cpu_PWRITE;
wire [31:0] mux_PWDATA  = mux_sel ? gwct_PWDATA   : cpu_PWDATA;

// Slave response wires
wire [31:0] slave_PRDATA;
wire        slave_PREADY;
wire        slave_PSLVERR;

// Feed slave response back to CPU stall CPU while GWCT owns the bus
assign cpu_PRDATA  = slave_PRDATA;
assign cpu_PREADY  = mux_sel ? 1'b0 : slave_PREADY;
assign cpu_PSLVERR = slave_PSLVERR;

// Your APB slave connect the muxed signals
your_apb_slave slave (
    .PADDR   (mux_PADDR),
    .PSEL    (mux_PSEL),
    .PENABLE (mux_PENABLE),
    .PWRITE  (mux_PWRITE),
    .PWDATA  (mux_PWDATA),
    .PRDATA  (slave_PRDATA),
    .PREADY  (slave_PREADY),
    .PSLVERR (slave_PSLVERR),
    ...
);
```

See `example/example_top.v` for a complete working (I think) example...

### Step 4 - Pin constraints

Add two entries to your `.cst` file. Pick any spare GPIO pins on your board:

```
IO_LOC "GWCT_RX" <pin>;
IO_LOC "GWCT_TX" <pin>;
IO_PORT "GWCT_RX" PULL_MODE=UP;
IO_PORT "GWCT_TX" PULL_MODE=UP;
```

Connect those pins to a USB-UART adapter (CP2102, CH340, FTDI, etc.).
Cross the wires: adapter TX -> GWCT_RX, adapter RX -> GWCT_TX.

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

