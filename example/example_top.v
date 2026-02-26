// =============================================================================
// example_top.v - Example showing how to wire gwct_top into a design that
//                 already has a Gowin Cortex-M1 (EMPU) with an APB bus.
//
// This pattern works for any design with an APB slave. The key idea is a
// 2-way mux in front of your APB slave(s):
//
//   - When GWCT has an active transaction (gwct_PSEL=1), GWCT drives the bus
//   - Otherwise, the CPU's APB bridge drives the bus
//   - While GWCT owns the bus, the CPU sees PREADY=0 and stalls (1-4 cycles)
//
// Adapt the port list and slave instantiation to your own design.
// =============================================================================

module example_top (
    input        HCLK,
    input        hwRstn,

    // --- Your existing pins ---
    inout [15:0] GPIO,
    inout        JTAG_7_SWDIO,
    inout        JTAG_9_SWDCLK,
    input        UART1RXD,
    output       UART1TXD,
    output       LOCKUP,
    output       HALTED,

    // --- GWCT UART pins (add these to your constraints file) ---
    input        GWCT_RX,
    output       GWCT_TX
);

    // =========================================================================
    // APB wires from Cortex-M1 APB bridge
    // =========================================================================
    wire [31:0] APB1PADDR;
    wire        APB1PENABLE;
    wire        APB1PWRITE;
    wire [3:0]  APB1PSTRB;
    wire [2:0]  APB1PPROT;
    wire [31:0] APB1PWDATA;
    wire [31:0] APB1PRDATA;
    wire        APB1PREADY;
    wire        APB1PSLVERR;
    wire        APB1PCLK;
    wire        APB1PRESET;
    wire        APB1PSEL;


    // =========================================================================
    // GWCT - drop in gwct_top, wire to two pins and the APB mux below
    // =========================================================================
    wire [31:0] gwct_PADDR;
    wire        gwct_PSEL;
    wire        gwct_PENABLE;
    wire        gwct_PWRITE;
    wire [31:0] gwct_PWDATA;
    wire [3:0]  gwct_PSTRB;
    wire [2:0]  gwct_PPROT;

    gwct_top #(
        .CLK_HZ (50_000_000),   // match your HCLK frequency
        .BAUD   (115_200)
    ) gwct (
        .clk     (HCLK),
        .rstn    (hwRstn),
        .gwct_rx (GWCT_RX),
        .gwct_tx (GWCT_TX),
        .PADDR   (gwct_PADDR),
        .PSEL    (gwct_PSEL),
        .PENABLE (gwct_PENABLE),
        .PWRITE  (gwct_PWRITE),
        .PWDATA  (gwct_PWDATA),
        .PSTRB   (gwct_PSTRB),
        .PPROT   (gwct_PPROT),
        .PRDATA  (slave_PRDATA),
        .PREADY  (slave_PREADY),
        .PSLVERR (slave_PSLVERR)
    );

    // =========================================================================
    // APB bus mux - GWCT takes priority when it has an active transaction.
    //
    // While GWCT owns the bus (gwct_PSEL=1):
    //   - slave sees GWCT signals
    //   - CPU sees PREADY=0 (stall) so it waits without corrupting state
    //
    // This is safe because GWCT only holds the bus for 3-4 APB cycles per
    // command (~300 ns at 50 MHz / 115200 baud gap between commands).
    // =========================================================================
    wire mux_sel = gwct_PSEL;   // GWCT wins when it has PSEL

    wire [31:0] mux_PADDR   = mux_sel ? gwct_PADDR   : APB1PADDR;
    wire        mux_PSEL    = mux_sel ? gwct_PSEL     : APB1PSEL;
    wire        mux_PENABLE = mux_sel ? gwct_PENABLE  : APB1PENABLE;
    wire        mux_PWRITE  = mux_sel ? gwct_PWRITE   : APB1PWRITE;
    wire [31:0] mux_PWDATA  = mux_sel ? gwct_PWDATA   : APB1PWDATA;

    // Slave response wires
    wire [31:0] slave_PRDATA;
    wire        slave_PREADY;
    wire        slave_PSLVERR;

    // CPU sees PREADY=0 while GWCT owns the bus so it stalls cleanly
    assign APB1PRDATA  = slave_PRDATA;
    assign APB1PREADY  = mux_sel ? 1'b0 : slave_PREADY;
    assign APB1PSLVERR = slave_PSLVERR;

    // =========================================================================
    // Your APB slave(s) - replace with your own apb_memmap or peripheral
    // =========================================================================
    apb_memmap apb_memmap_inst (
        .APBCLK  (APB1PCLK),
        .APBRESET(APB1PRESET),
        .PADDR   (mux_PADDR),
        .PSEL    (mux_PSEL),
        .PENABLE (mux_PENABLE),
        .PWRITE  (mux_PWRITE),
        .PWDATA  (mux_PWDATA),
        .PRDATA  (slave_PRDATA),
        .PREADY  (slave_PREADY),
        .PSLVERR (slave_PSLVERR)
    );

endmodule
