//-----------------------------------------------------------------------------
// R4W DMA Controller with AXI-Lite Interface
// R4W FPGA Acceleration Layer
//
// AXI-Lite wrapper for DMA controller. Provides PS-accessible registers
// for configuring and monitoring DMA transfers.
//
// Register Map:
// 0x00: CTRL       [0]=START_TX, [1]=START_RX, [2]=ABORT, [3]=CONTINUOUS, [31]=RESET
// 0x04: TX_LEN     TX transfer length in samples (16-bit)
// 0x08: RX_LEN     RX transfer length in samples (16-bit)
// 0x0C: STATUS     [0]=TX_BUSY, [1]=RX_BUSY, [2]=TX_DONE, [3]=RX_DONE,
//                  [4]=TX_ERR, [5]=RX_ERR
// 0x10: TX_COUNT   Current TX sample count (16-bit, read-only)
// 0x14: RX_COUNT   Current RX sample count (16-bit, read-only)
// 0x18: IRQ_EN     [0]=TX_DONE_EN, [1]=RX_DONE_EN, [2]=ERROR_EN
// 0x1C: IRQ_STATUS [0]=TX_DONE, [1]=RX_DONE, [2]=ERROR (write 1 to clear)
// 0x20: ID         IP core ID = 0x52344D41 ("R4DM")
// 0x24: VERSION    IP core version
//
// trace:FR-0086 | ai:claude
//-----------------------------------------------------------------------------

module r4w_dma_axi #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 8,
    parameter BUFFER_DEPTH = 4096
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

    // AXI-Stream from Xilinx DMA (S2MM - memory to PL)
    input  wire [31:0]                         S_AXIS_TDATA,
    input  wire                                S_AXIS_TVALID,
    output wire                                S_AXIS_TREADY,
    input  wire                                S_AXIS_TLAST,
    input  wire [3:0]                          S_AXIS_TKEEP,

    // AXI-Stream to Xilinx DMA (MM2S - PL to memory)
    output wire [31:0]                         M_AXIS_TDATA,
    output wire                                M_AXIS_TVALID,
    input  wire                                M_AXIS_TREADY,
    output wire                                M_AXIS_TLAST,
    output wire [3:0]                          M_AXIS_TKEEP,

    // AXI-Stream output to DSP pipeline
    output wire [31:0]                         M_DSP_TDATA,
    output wire                                M_DSP_TVALID,
    input  wire                                M_DSP_TREADY,
    output wire                                M_DSP_TLAST,

    // AXI-Stream input from DSP pipeline
    input  wire [31:0]                         S_DSP_TDATA,
    input  wire                                S_DSP_TVALID,
    output wire                                S_DSP_TREADY,
    input  wire                                S_DSP_TLAST,

    // Interrupt output (directly directly directly directly directly to directly directly PS)
    output wire                                IRQ
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam IP_ID = 32'h52344D41;       // "R4DM" - DMA identifier
    localparam IP_VERSION = 32'h00010000;  // Version 1.0.0

    // Register addresses
    localparam ADDR_CTRL       = 8'h00;
    localparam ADDR_TX_LEN     = 8'h04;
    localparam ADDR_RX_LEN     = 8'h08;
    localparam ADDR_STATUS     = 8'h0C;
    localparam ADDR_TX_COUNT   = 8'h10;
    localparam ADDR_RX_COUNT   = 8'h14;
    localparam ADDR_IRQ_EN     = 8'h18;
    localparam ADDR_IRQ_STATUS = 8'h1C;
    localparam ADDR_ID         = 8'h20;
    localparam ADDR_VERSION    = 8'h24;

    // =========================================================================
    // AXI-Lite Interface Signals
    // =========================================================================

    wire [C_S_AXI_ADDR_WIDTH-1:0]     reg_addr;
    wire [C_S_AXI_DATA_WIDTH-1:0]     reg_wdata;
    wire [(C_S_AXI_DATA_WIDTH/8)-1:0] reg_wstrb;
    wire                              reg_wen;
    wire                              reg_ren;
    reg  [C_S_AXI_DATA_WIDTH-1:0]     reg_rdata;
    reg                               reg_rvalid;

    // =========================================================================
    // Control Registers
    // =========================================================================

    reg [31:0] ctrl_reg;
    reg [15:0] tx_len_reg;
    reg [15:0] rx_len_reg;
    reg [2:0]  irq_en_reg;
    reg [2:0]  irq_status_reg;

    wire start_tx    = ctrl_reg[0];
    wire start_rx    = ctrl_reg[1];
    wire abort       = ctrl_reg[2];
    wire continuous  = ctrl_reg[3];
    wire soft_reset  = ctrl_reg[31];

    // =========================================================================
    // DMA Core Instance
    // =========================================================================

    wire status_tx_busy, status_rx_busy;
    wire status_tx_done, status_rx_done;
    wire status_tx_error, status_rx_error;
    wire [15:0] status_tx_count, status_rx_count;
    wire irq_tx_done, irq_rx_done, irq_error;

    r4w_dma #(
        .DATA_WIDTH(32),
        .BUFFER_DEPTH(BUFFER_DEPTH)
    ) dma_core (
        .clk(S_AXI_ACLK),
        .rst_n(S_AXI_ARESETN && !soft_reset),

        // Control
        .ctrl_start_tx(start_tx),
        .ctrl_start_rx(start_rx),
        .ctrl_abort(abort),
        .ctrl_tx_len(tx_len_reg),
        .ctrl_rx_len(rx_len_reg),
        .ctrl_continuous(continuous),

        // Status
        .status_tx_busy(status_tx_busy),
        .status_rx_busy(status_rx_busy),
        .status_tx_done(status_tx_done),
        .status_rx_done(status_rx_done),
        .status_tx_error(status_tx_error),
        .status_rx_error(status_rx_error),
        .status_tx_count(status_tx_count),
        .status_rx_count(status_rx_count),

        // Interrupts
        .irq_tx_done(irq_tx_done),
        .irq_rx_done(irq_rx_done),
        .irq_error(irq_error),

        // Xilinx DMA AXI-Stream
        .s_axis_tdata(S_AXIS_TDATA),
        .s_axis_tvalid(S_AXIS_TVALID),
        .s_axis_tready(S_AXIS_TREADY),
        .s_axis_tlast(S_AXIS_TLAST),
        .s_axis_tkeep(S_AXIS_TKEEP),

        .m_axis_tdata(M_AXIS_TDATA),
        .m_axis_tvalid(M_AXIS_TVALID),
        .m_axis_tready(M_AXIS_TREADY),
        .m_axis_tlast(M_AXIS_TLAST),
        .m_axis_tkeep(M_AXIS_TKEEP),

        // DSP AXI-Stream
        .m_dsp_tdata(M_DSP_TDATA),
        .m_dsp_tvalid(M_DSP_TVALID),
        .m_dsp_tready(M_DSP_TREADY),
        .m_dsp_tlast(M_DSP_TLAST),

        .s_dsp_tdata(S_DSP_TDATA),
        .s_dsp_tvalid(S_DSP_TVALID),
        .s_dsp_tready(S_DSP_TREADY),
        .s_dsp_tlast(S_DSP_TLAST)
    );

    // =========================================================================
    // Interrupt Logic
    // =========================================================================

    // Capture interrupt pulses into status register
    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            irq_status_reg <= 3'b000;
        end else begin
            // Set on interrupt pulse
            if (irq_tx_done) irq_status_reg[0] <= 1'b1;
            if (irq_rx_done) irq_status_reg[1] <= 1'b1;
            if (irq_error)   irq_status_reg[2] <= 1'b1;

            // Clear on write
            if (reg_wen && reg_addr == ADDR_IRQ_STATUS) begin
                if (reg_wstrb[0] && reg_wdata[0]) irq_status_reg[0] <= 1'b0;
                if (reg_wstrb[0] && reg_wdata[1]) irq_status_reg[1] <= 1'b0;
                if (reg_wstrb[0] && reg_wdata[2]) irq_status_reg[2] <= 1'b0;
            end
        end
    end

    // Generate IRQ when enabled and pending
    assign IRQ = |(irq_en_reg & irq_status_reg);

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
            ctrl_reg   <= 32'h0;
            tx_len_reg <= 16'd1024;  // Default 1K samples
            rx_len_reg <= 16'd1024;
            irq_en_reg <= 3'b000;
        end else begin
            // Auto-clear start and abort bits
            if (ctrl_reg[0]) ctrl_reg[0] <= 1'b0;  // START_TX
            if (ctrl_reg[1]) ctrl_reg[1] <= 1'b0;  // START_RX
            if (ctrl_reg[2]) ctrl_reg[2] <= 1'b0;  // ABORT
            if (ctrl_reg[31]) ctrl_reg[31] <= 1'b0;  // RESET

            if (reg_wen) begin
                case (reg_addr)
                    ADDR_CTRL: begin
                        if (reg_wstrb[0]) ctrl_reg[7:0]   <= reg_wdata[7:0];
                        if (reg_wstrb[1]) ctrl_reg[15:8]  <= reg_wdata[15:8];
                        if (reg_wstrb[2]) ctrl_reg[23:16] <= reg_wdata[23:16];
                        if (reg_wstrb[3]) ctrl_reg[31:24] <= reg_wdata[31:24];
                    end
                    ADDR_TX_LEN: begin
                        if (reg_wstrb[0]) tx_len_reg[7:0]  <= reg_wdata[7:0];
                        if (reg_wstrb[1]) tx_len_reg[15:8] <= reg_wdata[15:8];
                    end
                    ADDR_RX_LEN: begin
                        if (reg_wstrb[0]) rx_len_reg[7:0]  <= reg_wdata[7:0];
                        if (reg_wstrb[1]) rx_len_reg[15:8] <= reg_wdata[15:8];
                    end
                    ADDR_IRQ_EN: begin
                        if (reg_wstrb[0]) irq_en_reg <= reg_wdata[2:0];
                    end
                    // IRQ_STATUS handled above
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
                    ADDR_CTRL:       reg_rdata <= ctrl_reg;
                    ADDR_TX_LEN:     reg_rdata <= {16'h0, tx_len_reg};
                    ADDR_RX_LEN:     reg_rdata <= {16'h0, rx_len_reg};
                    ADDR_STATUS:     reg_rdata <= {26'h0,
                                                   status_rx_error,
                                                   status_tx_error,
                                                   status_rx_done,
                                                   status_tx_done,
                                                   status_rx_busy,
                                                   status_tx_busy};
                    ADDR_TX_COUNT:   reg_rdata <= {16'h0, status_tx_count};
                    ADDR_RX_COUNT:   reg_rdata <= {16'h0, status_rx_count};
                    ADDR_IRQ_EN:     reg_rdata <= {29'h0, irq_en_reg};
                    ADDR_IRQ_STATUS: reg_rdata <= {29'h0, irq_status_reg};
                    ADDR_ID:         reg_rdata <= IP_ID;
                    ADDR_VERSION:    reg_rdata <= IP_VERSION;
                    default:         reg_rdata <= 32'h0;
                endcase
            end
        end
    end

endmodule
