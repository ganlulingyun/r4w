//-----------------------------------------------------------------------------
// AXI-Lite Slave Interface Template
// R4W FPGA Acceleration Layer
//
// Generic AXI-Lite slave module for register-based IP cores.
// Provides standard AMBA AXI4-Lite interface with parameterized register space.
//-----------------------------------------------------------------------------

module axi_lite_slave #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 12,
    parameter NUM_REGS = 16
)(
    // AXI4-Lite Clock and Reset
    input  wire                                 S_AXI_ACLK,
    input  wire                                 S_AXI_ARESETN,

    // AXI4-Lite Write Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        S_AXI_AWADDR,
    input  wire [2:0]                           S_AXI_AWPROT,
    input  wire                                 S_AXI_AWVALID,
    output wire                                 S_AXI_AWREADY,

    // AXI4-Lite Write Data Channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]        S_AXI_WDATA,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]    S_AXI_WSTRB,
    input  wire                                 S_AXI_WVALID,
    output wire                                 S_AXI_WREADY,

    // AXI4-Lite Write Response Channel
    output wire [1:0]                           S_AXI_BRESP,
    output wire                                 S_AXI_BVALID,
    input  wire                                 S_AXI_BREADY,

    // AXI4-Lite Read Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]        S_AXI_ARADDR,
    input  wire [2:0]                           S_AXI_ARPROT,
    input  wire                                 S_AXI_ARVALID,
    output wire                                 S_AXI_ARREADY,

    // AXI4-Lite Read Data Channel
    output wire [C_S_AXI_DATA_WIDTH-1:0]        S_AXI_RDATA,
    output wire [1:0]                           S_AXI_RRESP,
    output wire                                 S_AXI_RVALID,
    input  wire                                 S_AXI_RREADY,

    // Register Interface (directly to IP core logic)
    output reg  [C_S_AXI_ADDR_WIDTH-1:0]        reg_addr,
    output reg  [C_S_AXI_DATA_WIDTH-1:0]        reg_wdata,
    output reg  [(C_S_AXI_DATA_WIDTH/8)-1:0]    reg_wstrb,
    output reg                                  reg_wen,
    output reg                                  reg_ren,
    input  wire [C_S_AXI_DATA_WIDTH-1:0]        reg_rdata,
    input  wire                                 reg_rvalid
);

    // =========================================================================
    // Local Parameters
    // =========================================================================

    localparam ADDR_LSB = (C_S_AXI_DATA_WIDTH/32) + 1;
    localparam OPT_MEM_ADDR_BITS = C_S_AXI_ADDR_WIDTH - ADDR_LSB;

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // Write channel
    reg                                 axi_awready;
    reg                                 axi_wready;
    reg [1:0]                           axi_bresp;
    reg                                 axi_bvalid;
    reg [C_S_AXI_ADDR_WIDTH-1:0]        axi_awaddr;

    // Read channel
    reg                                 axi_arready;
    reg [C_S_AXI_DATA_WIDTH-1:0]        axi_rdata;
    reg [1:0]                           axi_rresp;
    reg                                 axi_rvalid;
    reg [C_S_AXI_ADDR_WIDTH-1:0]        axi_araddr;

    // State machines
    reg                                 aw_en;

    // =========================================================================
    // AXI Output Assignments
    // =========================================================================

    assign S_AXI_AWREADY = axi_awready;
    assign S_AXI_WREADY  = axi_wready;
    assign S_AXI_BRESP   = axi_bresp;
    assign S_AXI_BVALID  = axi_bvalid;
    assign S_AXI_ARREADY = axi_arready;
    assign S_AXI_RDATA   = axi_rdata;
    assign S_AXI_RRESP   = axi_rresp;
    assign S_AXI_RVALID  = axi_rvalid;

    // =========================================================================
    // Write Address Channel
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_awready <= 1'b0;
            aw_en <= 1'b1;
            axi_awaddr <= 0;
        end else begin
            if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID && aw_en) begin
                axi_awready <= 1'b1;
                axi_awaddr <= S_AXI_AWADDR;
                aw_en <= 1'b0;
            end else if (S_AXI_BREADY && axi_bvalid) begin
                aw_en <= 1'b1;
                axi_awready <= 1'b0;
            end else begin
                axi_awready <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Write Data Channel
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_wready <= 1'b0;
        end else begin
            if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID && aw_en) begin
                axi_wready <= 1'b1;
            end else begin
                axi_wready <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Register Write Logic
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            reg_wen <= 1'b0;
            reg_addr <= 0;
            reg_wdata <= 0;
            reg_wstrb <= 0;
        end else begin
            if (axi_awready && S_AXI_AWVALID && axi_wready && S_AXI_WVALID) begin
                reg_wen <= 1'b1;
                reg_addr <= axi_awaddr;
                reg_wdata <= S_AXI_WDATA;
                reg_wstrb <= S_AXI_WSTRB;
            end else begin
                reg_wen <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Write Response Channel
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_bvalid <= 1'b0;
            axi_bresp <= 2'b0;
        end else begin
            if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID) begin
                axi_bvalid <= 1'b1;
                axi_bresp <= 2'b00; // OKAY response
            end else begin
                if (S_AXI_BREADY && axi_bvalid) begin
                    axi_bvalid <= 1'b0;
                end
            end
        end
    end

    // =========================================================================
    // Read Address Channel
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_arready <= 1'b0;
            axi_araddr <= 0;
        end else begin
            if (~axi_arready && S_AXI_ARVALID) begin
                axi_arready <= 1'b1;
                axi_araddr <= S_AXI_ARADDR;
            end else begin
                axi_arready <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Register Read Logic
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            reg_ren <= 1'b0;
        end else begin
            if (axi_arready && S_AXI_ARVALID && ~axi_rvalid) begin
                reg_ren <= 1'b1;
                reg_addr <= axi_araddr;
            end else begin
                reg_ren <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Read Data Channel
    // =========================================================================

    always @(posedge S_AXI_ACLK) begin
        if (S_AXI_ARESETN == 1'b0) begin
            axi_rvalid <= 1'b0;
            axi_rresp <= 2'b0;
            axi_rdata <= 0;
        end else begin
            if (reg_rvalid && ~axi_rvalid) begin
                axi_rvalid <= 1'b1;
                axi_rresp <= 2'b00; // OKAY response
                axi_rdata <= reg_rdata;
            end else if (axi_rvalid && S_AXI_RREADY) begin
                axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
