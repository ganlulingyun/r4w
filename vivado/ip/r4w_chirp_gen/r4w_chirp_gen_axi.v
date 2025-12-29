//-----------------------------------------------------------------------------
// R4W LoRa Chirp Generator with AXI-Lite Interface
// R4W FPGA Acceleration Layer
//
// AXI-Lite wrapper for chirp generator. Register map matches registers.rs::chirp
//
// Register Map:
// 0x00: CTRL       [0]=START, [1]=UPCHIRP, [2]=CONTINUOUS, [31]=RESET
// 0x04: SF         Spreading factor (5-12)
// 0x08: STATUS     [0]=DONE, [1]=BUSY, [2]=OVERFLOW
// 0x0C: SYMBOL     Symbol value to modulate (0 to 2^SF-1)
// 0x10: DATA_OUT   Packed I/Q output [31:16]=I, [15:0]=Q
// 0x14: BANDWIDTH  Bandwidth selection: 0=125k, 1=250k, 2=500k
// 0x1C: VERSION    IP core version (read-only)
// 0x20: ID         IP core ID (read-only) = 0x52344347 ("R4CG")
//
// trace:FR-0031 | ai:claude
//-----------------------------------------------------------------------------

module r4w_chirp_gen_axi #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 8
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

    // AXI-Stream output (optional, for DMA connection)
    output wire [31:0]                         M_AXIS_TDATA,
    output wire                                M_AXIS_TVALID,
    input  wire                                M_AXIS_TREADY,
    output wire                                M_AXIS_TLAST
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam IP_ID = 32'h52344347;       // "R4CG" - Chirp Generator
    localparam IP_VERSION = 32'h00010000;  // Version 1.0.0

    // Register addresses
    localparam ADDR_CTRL      = 8'h00;
    localparam ADDR_SF        = 8'h04;
    localparam ADDR_STATUS    = 8'h08;
    localparam ADDR_SYMBOL    = 8'h0C;
    localparam ADDR_DATA_OUT  = 8'h10;
    localparam ADDR_BANDWIDTH = 8'h14;
    localparam ADDR_VERSION   = 8'h1C;
    localparam ADDR_ID        = 8'h20;

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
    reg [3:0]  sf_reg;
    reg [11:0] symbol_reg;
    reg [1:0]  bw_reg;

    wire start      = ctrl_reg[0];
    wire upchirp    = ctrl_reg[1];
    wire continuous = ctrl_reg[2];
    wire soft_reset = ctrl_reg[31];

    // =========================================================================
    // Chirp Generator Instance
    // =========================================================================

    wire signed [15:0] i_out;
    wire signed [15:0] q_out;
    wire out_valid;
    wire done;
    wire busy;

    r4w_chirp_gen #(
        .PHASE_WIDTH(32),
        .OUTPUT_WIDTH(16),
        .MAX_SF(12),
        .CORDIC_STAGES(16)
    ) chirp_inst (
        .clk(S_AXI_ACLK),
        .rst_n(S_AXI_ARESETN && !soft_reset),
        .start(start),
        .upchirp(upchirp),
        .continuous(continuous),
        .sf(sf_reg),
        .bw_sel(bw_reg),
        .symbol(symbol_reg),
        .i_out(i_out),
        .q_out(q_out),
        .out_valid(out_valid),
        .done(done),
        .busy(busy)
    );

    // =========================================================================
    // Latched Output (for register reads)
    // =========================================================================

    reg [31:0] data_out_latch;
    reg overflow;

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN || soft_reset) begin
            data_out_latch <= 0;
            overflow <= 0;
        end else if (out_valid) begin
            data_out_latch <= {i_out, q_out};
            // Could implement FIFO overflow detection here
        end
    end

    // =========================================================================
    // AXI-Stream Output
    // =========================================================================

    assign M_AXIS_TDATA = {i_out, q_out};
    assign M_AXIS_TVALID = out_valid;
    assign M_AXIS_TLAST = done;

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

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            ctrl_reg <= 32'h0;
            sf_reg <= 4'd7;          // Default SF7
            symbol_reg <= 12'h0;
            bw_reg <= 2'd0;          // Default 125 kHz
        end else begin
            // Auto-clear start bit after one cycle
            if (ctrl_reg[0])
                ctrl_reg[0] <= 1'b0;

            // Auto-clear soft reset
            if (ctrl_reg[31])
                ctrl_reg[31] <= 1'b0;

            if (reg_wen) begin
                case (reg_addr)
                    ADDR_CTRL: begin
                        if (reg_wstrb[0]) ctrl_reg[7:0]   <= reg_wdata[7:0];
                        if (reg_wstrb[1]) ctrl_reg[15:8]  <= reg_wdata[15:8];
                        if (reg_wstrb[2]) ctrl_reg[23:16] <= reg_wdata[23:16];
                        if (reg_wstrb[3]) ctrl_reg[31:24] <= reg_wdata[31:24];
                    end
                    ADDR_SF: begin
                        if (reg_wstrb[0]) sf_reg <= reg_wdata[3:0];
                    end
                    ADDR_SYMBOL: begin
                        if (reg_wstrb[0]) symbol_reg[7:0]  <= reg_wdata[7:0];
                        if (reg_wstrb[1]) symbol_reg[11:8] <= reg_wdata[11:8];
                    end
                    ADDR_BANDWIDTH: begin
                        if (reg_wstrb[0]) bw_reg <= reg_wdata[1:0];
                    end
                endcase
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
                case (reg_addr)
                    ADDR_CTRL:      reg_rdata <= ctrl_reg;
                    ADDR_SF:        reg_rdata <= {28'h0, sf_reg};
                    ADDR_STATUS:    reg_rdata <= {29'h0, overflow, busy, done};
                    ADDR_SYMBOL:    reg_rdata <= {20'h0, symbol_reg};
                    ADDR_DATA_OUT:  reg_rdata <= data_out_latch;
                    ADDR_BANDWIDTH: reg_rdata <= {30'h0, bw_reg};
                    ADDR_VERSION:   reg_rdata <= IP_VERSION;
                    ADDR_ID:        reg_rdata <= IP_ID;
                    default:        reg_rdata <= 32'h0;
                endcase
            end
        end
    end

endmodule
