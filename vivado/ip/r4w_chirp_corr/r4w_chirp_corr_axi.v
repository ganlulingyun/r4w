//-----------------------------------------------------------------------------
// R4W Chirp Correlator with AXI-Lite Interface
// R4W FPGA Acceleration Layer
//
// AXI-Lite wrapper for chirp correlator. Register map matches registers.rs::correlator
//
// Register Map:
// 0x00: CTRL       [0]=START, [31]=RESET
// 0x04: SF         Spreading factor (5-12)
// 0x08: STATUS     [0]=DONE, [1]=BUSY, [2]=DETECTED
// 0x10: DATA_IN    Packed I/Q input [31:16]=I, [15:0]=Q
// 0x20: SYMBOL     Detected symbol value (0 to 2^SF-1)
// 0x24: MAGNITUDE  Correlation magnitude
// 0x28: THRESHOLD  Detection threshold
// 0x1C: VERSION    IP core version (read-only)
// 0x30: ID         IP core ID (read-only) = 0x52344343 ("R4CC")
//
// trace:FR-0034 | ai:claude
//-----------------------------------------------------------------------------

module r4w_chirp_corr_axi #(
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

    // AXI-Stream input (for DMA connection)
    input  wire [31:0]                         S_AXIS_TDATA,
    input  wire                                S_AXIS_TVALID,
    output wire                                S_AXIS_TREADY,
    input  wire                                S_AXIS_TLAST
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam IP_ID = 32'h52344343;       // "R4CC" - Chirp Correlator
    localparam IP_VERSION = 32'h00010000;  // Version 1.0.0

    // Register addresses
    localparam ADDR_CTRL      = 8'h00;
    localparam ADDR_SF        = 8'h04;
    localparam ADDR_STATUS    = 8'h08;
    localparam ADDR_DATA_IN   = 8'h10;
    localparam ADDR_VERSION   = 8'h1C;
    localparam ADDR_SYMBOL    = 8'h20;
    localparam ADDR_MAGNITUDE = 8'h24;
    localparam ADDR_THRESHOLD = 8'h28;
    localparam ADDR_ID        = 8'h30;

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
    reg [31:0] threshold_reg;
    reg [31:0] data_in_reg;
    reg data_in_valid;

    wire start      = ctrl_reg[0];
    wire soft_reset = ctrl_reg[31];

    // =========================================================================
    // Correlator Instance
    // =========================================================================

    wire done;
    wire busy;
    wire detected;
    wire [11:0] symbol_out;
    wire [31:0] magnitude;

    // Stream mux
    wire [31:0] corr_in_data = S_AXIS_TVALID ? S_AXIS_TDATA : data_in_reg;
    wire corr_in_valid = S_AXIS_TVALID || data_in_valid;

    r4w_chirp_corr #(
        .MAX_SF(12),
        .DATA_WIDTH(16),
        .CORDIC_STAGES(16)
    ) corr_inst (
        .clk(S_AXI_ACLK),
        .rst_n(S_AXI_ARESETN && !soft_reset),
        .start(start),
        .sf(sf_reg),
        .threshold(threshold_reg),
        .done(done),
        .busy(busy),
        .detected(detected),
        .s_axis_tdata(corr_in_data),
        .s_axis_tvalid(corr_in_valid),
        .s_axis_tready(S_AXIS_TREADY),
        .s_axis_tlast(S_AXIS_TLAST),
        .symbol_out(symbol_out),
        .magnitude(magnitude)
    );

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
            sf_reg <= 4'd7;              // Default SF7
            threshold_reg <= 32'h1000;   // Default threshold
            data_in_reg <= 0;
            data_in_valid <= 0;
        end else begin
            data_in_valid <= 1'b0;

            // Auto-clear start bit
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
                    ADDR_DATA_IN: begin
                        data_in_reg <= reg_wdata;
                        data_in_valid <= 1'b1;
                    end
                    ADDR_THRESHOLD: begin
                        if (reg_wstrb[0]) threshold_reg[7:0]   <= reg_wdata[7:0];
                        if (reg_wstrb[1]) threshold_reg[15:8]  <= reg_wdata[15:8];
                        if (reg_wstrb[2]) threshold_reg[23:16] <= reg_wdata[23:16];
                        if (reg_wstrb[3]) threshold_reg[31:24] <= reg_wdata[31:24];
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
                    ADDR_STATUS:    reg_rdata <= {29'h0, detected, busy, done};
                    ADDR_DATA_IN:   reg_rdata <= 32'h0;
                    ADDR_VERSION:   reg_rdata <= IP_VERSION;
                    ADDR_SYMBOL:    reg_rdata <= {20'h0, symbol_out};
                    ADDR_MAGNITUDE: reg_rdata <= magnitude;
                    ADDR_THRESHOLD: reg_rdata <= threshold_reg;
                    ADDR_ID:        reg_rdata <= IP_ID;
                    default:        reg_rdata <= 32'h0;
                endcase
            end
        end
    end

endmodule
