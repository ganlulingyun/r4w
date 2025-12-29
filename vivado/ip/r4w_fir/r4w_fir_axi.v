//-----------------------------------------------------------------------------
// R4W FIR Filter with AXI-Lite Interface
// R4W FPGA Acceleration Layer
//
// AXI-Lite wrapper for FIR IP core. Register map matches registers.rs::fir
//
// Register Map:
// 0x00: CTRL       [0]=START, [1]=RELOAD_TAPS, [31]=RESET
// 0x04: NUM_TAPS   Number of coefficients (1-256)
// 0x08: STATUS     [0]=DONE, [1]=BUSY, [2]=ERROR
// 0x10: DATA_IN    Packed I/Q input [31:16]=I, [15:0]=Q
// 0x14: DATA_OUT   Packed I/Q output
// 0x1C: VERSION    IP core version (read-only)
// 0x20: ID         IP core ID (read-only) = 0x52344649 ("R4FI")
// 0x100-0x4FF: TAPS  256 coefficients (32-bit each, only lower 16 used)
//
// trace:FR-0033 | ai:claude
//-----------------------------------------------------------------------------

module r4w_fir_axi #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 12   // Need more bits for tap storage
)(
    // AXI4-Lite Interface
    input  wire                                S_AXI_ACLK,
    input  wire                                S_AXI_ARESETN,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       S_AXI_AWADDR,
    input  wire [2:0]                          S_AXI_AWPROT,
    input  wire                                S_AXI_AWVALID,
    output wire                                S_AXI_AWREADY,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]       S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]   S_AXI_WSTRB,
    input  wire                                S_AXI_WVALID,
    output wire                                S_AXI_WREADY,

    output wire [1:0]                          S_AXI_BRESP,
    output wire                                S_AXI_BVALID,
    input  wire                                S_AXI_BREADY,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       S_AXI_ARADDR,
    input  wire [2:0]                          S_AXI_ARPROT,
    input  wire                                S_AXI_ARVALID,
    output wire                                S_AXI_ARREADY,

    output wire [C_S_AXI_DATA_WIDTH-1:0]       S_AXI_RDATA,
    output wire [1:0]                          S_AXI_RRESP,
    output wire                                S_AXI_RVALID,
    input  wire                                S_AXI_RREADY,

    // AXI-Stream I/O (for DMA connection)
    input  wire [31:0]                         S_AXIS_TDATA,
    input  wire                                S_AXIS_TVALID,
    output wire                                S_AXIS_TREADY,

    output wire [31:0]                         M_AXIS_TDATA,
    output wire                                M_AXIS_TVALID,
    input  wire                                M_AXIS_TREADY
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam IP_ID = 32'h52344649;       // "R4FI" - FIR identifier
    localparam IP_VERSION = 32'h00010000;  // Version 1.0.0

    // Register addresses
    localparam ADDR_CTRL     = 12'h000;
    localparam ADDR_NUM_TAPS = 12'h004;
    localparam ADDR_STATUS   = 12'h008;
    localparam ADDR_DATA_IN  = 12'h010;
    localparam ADDR_DATA_OUT = 12'h014;
    localparam ADDR_VERSION  = 12'h01C;
    localparam ADDR_ID       = 12'h020;
    localparam ADDR_TAPS     = 12'h100;  // Taps at 0x100-0x4FF

    // =========================================================================
    // AXI-Lite Interface Signals
    // =========================================================================

    wire [C_S_AXI_ADDR_WIDTH-1:0]    reg_addr;
    wire [C_S_AXI_DATA_WIDTH-1:0]    reg_wdata;
    wire [(C_S_AXI_DATA_WIDTH/8)-1:0] reg_wstrb;
    wire                             reg_wen;
    wire                             reg_ren;
    reg  [C_S_AXI_DATA_WIDTH-1:0]    reg_rdata;
    reg                              reg_rvalid;

    // =========================================================================
    // Control Registers
    // =========================================================================

    reg [31:0] ctrl_reg;
    reg [8:0]  num_taps_reg;

    wire start       = ctrl_reg[0];
    wire reload_taps = ctrl_reg[1];
    wire soft_reset  = ctrl_reg[31];

    // =========================================================================
    // Coefficient Storage (CPU writes here, then triggers reload)
    // =========================================================================

    reg signed [15:0] tap_storage [0:255];
    reg [8:0] reload_index;
    reg reloading;
    reg reload_valid;
    wire reload_ready;
    wire reload_done;

    // =========================================================================
    // Input Data FIFO
    // =========================================================================

    reg [31:0] data_in_reg;
    reg data_in_valid;

    // =========================================================================
    // FIR Instance
    // =========================================================================

    wire done;
    wire busy;
    wire fir_error;
    wire [31:0] fir_out_data;
    wire fir_out_valid;

    // Stream mux: use AXI-Stream if valid, else register interface
    wire [31:0] fir_in_data = S_AXIS_TVALID ? S_AXIS_TDATA : data_in_reg;
    wire fir_in_valid = S_AXIS_TVALID || data_in_valid;

    r4w_fir #(
        .MAX_TAPS(256),
        .DATA_WIDTH(16),
        .COEF_WIDTH(16)
    ) fir_inst (
        .clk(S_AXI_ACLK),
        .rst_n(S_AXI_ARESETN && !soft_reset),
        .start(start),
        .reload_taps(reload_taps),
        .num_taps(num_taps_reg),
        .done(done),
        .busy(busy),
        .error(fir_error),
        .coef_data(tap_storage[reload_index]),
        .coef_valid(reload_valid),
        .coef_ready(reload_ready),
        .coef_last(reload_index == num_taps_reg - 1),
        .s_axis_tdata(fir_in_data),
        .s_axis_tvalid(fir_in_valid),
        .s_axis_tready(S_AXIS_TREADY),
        .m_axis_tdata(fir_out_data),
        .m_axis_tvalid(fir_out_valid),
        .m_axis_tready(M_AXIS_TREADY)
    );

    assign M_AXIS_TDATA = fir_out_data;
    assign M_AXIS_TVALID = fir_out_valid;

    // =========================================================================
    // Coefficient Reload State Machine
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN || soft_reset) begin
            reload_index <= 0;
            reloading <= 0;
            reload_valid <= 0;
        end else begin
            if (reload_taps && !reloading) begin
                reloading <= 1'b1;
                reload_index <= 0;
                reload_valid <= 1'b1;
            end else if (reloading) begin
                if (reload_ready && reload_valid) begin
                    if (reload_index >= num_taps_reg - 1) begin
                        reloading <= 1'b0;
                        reload_valid <= 1'b0;
                    end else begin
                        reload_index <= reload_index + 1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // Output Latch
    // =========================================================================

    reg [31:0] data_out_latch;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            data_out_latch <= 0;
        end else if (fir_out_valid) begin
            data_out_latch <= fir_out_data;
        end
    end

    // =========================================================================
    // AXI-Lite Slave Instance
    // =========================================================================

    axi_lite_slave #(
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH)
    ) axi_slave (
        .S_AXI_ACLK(S_AXI_ACLK),
        .S_AXI_ARESETN(S_AXI_ARESETN),
        .S_AXI_AWADDR(S_AXI_AWADDR),
        .S_AXI_AWPROT(S_AXI_AWPROT),
        .S_AXI_AWVALID(S_AXI_AWVALID),
        .S_AXI_AWREADY(S_AXI_AWREADY),
        .S_AXI_WDATA(S_AXI_WDATA),
        .S_AXI_WSTRB(S_AXI_WSTRB),
        .S_AXI_WVALID(S_AXI_WVALID),
        .S_AXI_WREADY(S_AXI_WREADY),
        .S_AXI_BRESP(S_AXI_BRESP),
        .S_AXI_BVALID(S_AXI_BVALID),
        .S_AXI_BREADY(S_AXI_BREADY),
        .S_AXI_ARADDR(S_AXI_ARADDR),
        .S_AXI_ARPROT(S_AXI_ARPROT),
        .S_AXI_ARVALID(S_AXI_ARVALID),
        .S_AXI_ARREADY(S_AXI_ARREADY),
        .S_AXI_RDATA(S_AXI_RDATA),
        .S_AXI_RRESP(S_AXI_RRESP),
        .S_AXI_RVALID(S_AXI_RVALID),
        .S_AXI_RREADY(S_AXI_RREADY),
        .reg_addr(reg_addr),
        .reg_wdata(reg_wdata),
        .reg_wstrb(reg_wstrb),
        .reg_wen(reg_wen),
        .reg_ren(reg_ren),
        .reg_rdata(reg_rdata),
        .reg_rvalid(reg_rvalid)
    );

    // =========================================================================
    // Register Write Logic
    // =========================================================================

    wire is_tap_addr = (reg_addr >= ADDR_TAPS) && (reg_addr < ADDR_TAPS + 12'h400);
    wire [7:0] tap_index = (reg_addr - ADDR_TAPS) >> 2;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            ctrl_reg <= 32'h0;
            num_taps_reg <= 9'd64;   // Default 64 taps
            data_in_reg <= 0;
            data_in_valid <= 0;
        end else begin
            data_in_valid <= 1'b0;

            // Auto-clear start and reload bits
            if (ctrl_reg[0]) ctrl_reg[0] <= 1'b0;
            if (ctrl_reg[1]) ctrl_reg[1] <= 1'b0;
            if (ctrl_reg[31]) ctrl_reg[31] <= 1'b0;

            if (reg_wen) begin
                if (is_tap_addr) begin
                    // Write to tap coefficient storage
                    tap_storage[tap_index] <= reg_wdata[15:0];
                end else begin
                    case (reg_addr)
                        ADDR_CTRL: begin
                            if (reg_wstrb[0]) ctrl_reg[7:0]   <= reg_wdata[7:0];
                            if (reg_wstrb[1]) ctrl_reg[15:8]  <= reg_wdata[15:8];
                            if (reg_wstrb[2]) ctrl_reg[23:16] <= reg_wdata[23:16];
                            if (reg_wstrb[3]) ctrl_reg[31:24] <= reg_wdata[31:24];
                        end
                        ADDR_NUM_TAPS: begin
                            if (reg_wstrb[0]) num_taps_reg[7:0] <= reg_wdata[7:0];
                            if (reg_wstrb[1]) num_taps_reg[8]   <= reg_wdata[8];
                        end
                        ADDR_DATA_IN: begin
                            data_in_reg <= reg_wdata;
                            data_in_valid <= 1'b1;
                        end
                    endcase
                end
            end
        end
    end

    // =========================================================================
    // Register Read Logic
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            reg_rdata <= 32'h0;
            reg_rvalid <= 1'b0;
        end else begin
            reg_rvalid <= reg_ren;

            if (reg_ren) begin
                if (is_tap_addr) begin
                    reg_rdata <= {{16{tap_storage[tap_index][15]}}, tap_storage[tap_index]};
                end else begin
                    case (reg_addr)
                        ADDR_CTRL:     reg_rdata <= ctrl_reg;
                        ADDR_NUM_TAPS: reg_rdata <= {23'h0, num_taps_reg};
                        ADDR_STATUS:   reg_rdata <= {29'h0, fir_error, busy, done};
                        ADDR_DATA_IN:  reg_rdata <= 32'h0;
                        ADDR_DATA_OUT: reg_rdata <= data_out_latch;
                        ADDR_VERSION:  reg_rdata <= IP_VERSION;
                        ADDR_ID:       reg_rdata <= IP_ID;
                        default:       reg_rdata <= 32'h0;
                    endcase
                end
            end
        end
    end

endmodule
