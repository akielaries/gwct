// =============================================================================
// gwct_top.v - Gowin Command Line Tools - FPGA-side top wrapper
//
// Drop this module into your design and connect two things:
//   1. Two UART pins  (gwct_rx, gwct_tx)
//   2. Your APB bus   (the PADDR/PSEL/... signals)
//
// GWCT will then act as a second APB master alongside your CPU.
// When GWCT is idle, it holds PSEL=0 and your CPU has full bus access.
// When GWCT is executing a command it drives the bus for 3-4 APB cycles,
// during which your CPU will see PREADY=0 and stall harmlessly.
//
// Parameters:
//   CLK_HZ   - your system clock frequency (must match your HCLK)
//   BAUD     - UART baud rate (must match gwct host tool, default 115200)
//
// Ports:
//   clk              - system clock
//   rstn             - active-low reset
//   gwct_rx          - UART RX pin (from USB-UART adapter)
//   gwct_tx          - UART TX pin (to USB-UART adapter)
//
//   -- APB master outputs (connect to your APB bus / slave) --
//   PADDR            - address
//   PSEL             - slave select
//   PENABLE          - enable
//   PWRITE           - write strobe
//   PWDATA           - write data
//   PSTRB            - byte strobes
//   PPROT            - protection
//
//   -- APB slave response inputs --
//   PRDATA           - read data
//   PREADY           - slave ready
//   PSLVERR          - slave error
//
// Typical wiring in your top.v  (see example/example_top.v for full example):
//
//   gwct_top #(.CLK_HZ(50_000_000)) gwct (
//       .clk     (HCLK),
//       .rstn    (hwRstn),
//       .gwct_rx (GWCT_RX),
//       .gwct_tx (GWCT_TX),
//       .PADDR   (mux_PADDR),   // drive your APB mux
//       .PSEL    (gwct_PSEL),
//       ...
//       .PRDATA  (slave_PRDATA),
//       .PREADY  (slave_PREADY),
//       .PSLVERR (slave_PSLVERR)
//   );
//
// =============================================================================

module gwct_top #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115_200
)(
    input        clk,
    input        rstn,

    // UART pins
    input        gwct_rx,
    output       gwct_tx,

    // APB master interface (connect to your bus)
    output [31:0] PADDR,
    output        PSEL,
    output        PENABLE,
    output        PWRITE,
    output [31:0] PWDATA,
    output [3:0]  PSTRB,
    output [2:0]  PPROT,

    // APB slave response
    input  [31:0] PRDATA,
    input         PREADY,
    input         PSLVERR
);

    // -------------------------------------------------------------------------
    // Internal wires between the three sub-modules
    // -------------------------------------------------------------------------

    // UART <-> packet
    wire [7:0] uart_rx_data;
    wire       uart_rx_valid;
    wire [7:0] uart_tx_data;
    wire       uart_tx_valid;
    wire       uart_tx_ready;

    // packet <-> APB master
    wire [31:0] cmd_addr;
    wire [31:0] cmd_wdata;
    wire        cmd_write;
    wire        cmd_valid;
    wire        cmd_ready;
    wire [31:0] cmd_rdata;
    wire        cmd_error;

    // -------------------------------------------------------------------------
    // UART byte layer
    // -------------------------------------------------------------------------
    gwct_uart #(
        .CLK_HZ (CLK_HZ),
        .BAUD   (BAUD)
    ) u_uart (
        .clk      (clk),
        .rstn     (rstn),
        .rx       (gwct_rx),
        .tx       (gwct_tx),
        .rx_data  (uart_rx_data),
        .rx_valid (uart_rx_valid),
        .tx_data  (uart_tx_data),
        .tx_valid (uart_tx_valid),
        .tx_ready (uart_tx_ready)
    );

    // -------------------------------------------------------------------------
    // Packet framing layer
    // -------------------------------------------------------------------------
    gwct_packet u_packet (
        .clk       (clk),
        .rstn      (rstn),
        .rx_data   (uart_rx_data),
        .rx_valid  (uart_rx_valid),
        .tx_data   (uart_tx_data),
        .tx_valid  (uart_tx_valid),
        .tx_ready  (uart_tx_ready),
        .cmd_addr  (cmd_addr),
        .cmd_wdata (cmd_wdata),
        .cmd_write (cmd_write),
        .cmd_valid (cmd_valid),
        .cmd_ready (cmd_ready),
        .cmd_rdata (cmd_rdata),
        .cmd_error (cmd_error)
    );

    // -------------------------------------------------------------------------
    // APB master FSM
    // -------------------------------------------------------------------------
    gwct_apb_master u_apb (
        .clk       (clk),
        .rstn      (rstn),
        .cmd_addr  (cmd_addr),
        .cmd_wdata (cmd_wdata),
        .cmd_write (cmd_write),
        .cmd_valid (cmd_valid),
        .cmd_ready (cmd_ready),
        .cmd_rdata (cmd_rdata),
        .cmd_error (cmd_error),
        .PADDR     (PADDR),
        .PSEL      (PSEL),
        .PENABLE   (PENABLE),
        .PWRITE    (PWRITE),
        .PWDATA    (PWDATA),
        .PSTRB     (PSTRB),
        .PPROT     (PPROT),
        .PRDATA    (PRDATA),
        .PREADY    (PREADY),
        .PSLVERR   (PSLVERR)
    );

endmodule
