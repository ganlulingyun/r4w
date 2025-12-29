//-----------------------------------------------------------------------------
// R4W NCO with AXI-Lite Interface
// R4W FPGA Acceleration Layer
//
// AXI-Lite wrapper for NCO IP core. Register map matches registers.rs::nco
//
// Register Map:
// 0x00: CTRL       [0]=ENABLE, [1]=RESET_PHASE, [31]=RESET
// 0x04: FREQ       Frequency word (phase increment per sample)
// 0x08: PHASE      Phase offset (0 to 2π scaled to 32-bit)
// 0x0C: AMPLITUDE  Output amplitude scaling
// 0x10: STATUS     Status register
// 0x14: I_OUT      Cosine output (signed 16-bit)
// 0x18: Q_OUT      Sine output (signed 16-bit)
// 0x1C: VERSION    IP core version (read-only)
// 0x20: ID         IP core ID (read-only) = 0x5234494F ("R4IO")
//
// trace:FR-0030 | ai:claude
//-----------------------------------------------------------------------------

module r4w_nco_axi #(
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
    input  wire                                S_AXI_RREADY
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam IP_ID = 32'h5234494F;       // "R4IO" - NCO identifier
    localparam IP_VERSION = 32'h00010000;  // Version 1.0.0

    // Register addresses
    localparam ADDR_CTRL      = 8'h00;
    localparam ADDR_FREQ      = 8'h04;
    localparam ADDR_PHASE     = 8'h08;
    localparam ADDR_AMPLITUDE = 8'h0C;
    localparam ADDR_STATUS    = 8'h10;
    localparam ADDR_I_OUT     = 8'h14;
    localparam ADDR_Q_OUT     = 8'h18;
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
    reg [31:0] freq_reg;
    reg [31:0] phase_reg;
    reg [15:0] amplitude_reg;

    wire enable       = ctrl_reg[0];
    wire reset_phase  = ctrl_reg[1];
    wire soft_reset   = ctrl_reg[31];

    // =========================================================================
    // NCO Instance
    // =========================================================================

    wire signed [15:0] i_out;
    wire signed [15:0] q_out;
    wire out_valid;

    r4w_nco #(
        .PHASE_WIDTH(32),
        .OUTPUT_WIDTH(16),
        .CORDIC_STAGES(16)
    ) nco_inst (
        .clk(S_AXI_ACLK),
        .rst_n(S_AXI_ARESETN && !soft_reset),
        .enable(enable),
        .reset_phase(reset_phase),
        .freq_word(freq_reg),
        .phase_offset(phase_reg),
        .amplitude(amplitude_reg),
        .i_out(i_out),
        .q_out(q_out),
        .out_valid(out_valid)
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
            freq_reg <= 32'h0;
            phase_reg <= 32'h0;
            amplitude_reg <= 16'h7FFF;  // Default max amplitude
        end else begin
            // Auto-clear reset_phase bit
            if (ctrl_reg[1])
                ctrl_reg[1] <= 1'b0;

            // Auto-clear soft reset bit
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
                    ADDR_FREQ: begin
                        if (reg_wstrb[0]) freq_reg[7:0]   <= reg_wdata[7:0];
                        if (reg_wstrb[1]) freq_reg[15:8]  <= reg_wdata[15:8];
                        if (reg_wstrb[2]) freq_reg[23:16] <= reg_wdata[23:16];
                        if (reg_wstrb[3]) freq_reg[31:24] <= reg_wdata[31:24];
                    end
                    ADDR_PHASE: begin
                        if (reg_wstrb[0]) phase_reg[7:0]   <= reg_wdata[7:0];
                        if (reg_wstrb[1]) phase_reg[15:8]  <= reg_wdata[15:8];
                        if (reg_wstrb[2]) phase_reg[23:16] <= reg_wdata[23:16];
                        if (reg_wstrb[3]) phase_reg[31:24] <= reg_wdata[31:24];
                    end
                    ADDR_AMPLITUDE: begin
                        if (reg_wstrb[0]) amplitude_reg[7:0]  <= reg_wdata[7:0];
                        if (reg_wstrb[1]) amplitude_reg[15:8] <= reg_wdata[15:8];
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
                    ADDR_FREQ:      reg_rdata <= freq_reg;
                    ADDR_PHASE:     reg_rdata <= phase_reg;
                    ADDR_AMPLITUDE: reg_rdata <= {16'h0, amplitude_reg};
                    ADDR_STATUS:    reg_rdata <= {31'h0, out_valid};
                    ADDR_I_OUT:     reg_rdata <= {{16{i_out[15]}}, i_out};  // Sign-extend
                    ADDR_Q_OUT:     reg_rdata <= {{16{q_out[15]}}, q_out};  // Sign-extend
                    ADDR_VERSION:   reg_rdata <= IP_VERSION;
                    ADDR_ID:        reg_rdata <= IP_ID;
                    default:        reg_rdata <= 32'h0;
                endcase
            end
        end
    end

endmodule
