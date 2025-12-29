//-----------------------------------------------------------------------------
// R4W FFT with AXI-Lite Interface
// R4W FPGA Acceleration Layer
//
// AXI-Lite wrapper for FFT IP core. Register map matches registers.rs::fft
//
// Register Map:
// 0x00: CTRL       [0]=START, [1]=INVERSE, [31]=RESET
// 0x04: SIZE       log2(N): 6=64, 7=128, 8=256, 9=512, 10=1024
// 0x08: STATUS     [0]=DONE, [1]=BUSY, [2]=ERROR
// 0x10: DATA_IN    Packed I/Q input [31:16]=I, [15:0]=Q
// 0x14: DATA_OUT   Packed I/Q output
// 0x1C: VERSION    IP core version (read-only)
// 0x20: ID         IP core ID (read-only) = 0x52345746 ("R4WF")
//
// trace:FR-0032 | ai:claude
//-----------------------------------------------------------------------------

module r4w_fft_axi #(
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

    // AXI-Stream Data Input (optional, for DMA)
    input  wire [31:0]                         S_AXIS_TDATA,
    input  wire                                S_AXIS_TVALID,
    output wire                                S_AXIS_TREADY,
    input  wire                                S_AXIS_TLAST,

    // AXI-Stream Data Output (optional, for DMA)
    output wire [31:0]                         M_AXIS_TDATA,
    output wire                                M_AXIS_TVALID,
    input  wire                                M_AXIS_TREADY,
    output wire                                M_AXIS_TLAST
);

    // =========================================================================
    // Constants
    // =========================================================================

    localparam IP_ID = 32'h52345746;       // "R4WF" - FFT identifier
    localparam IP_VERSION = 32'h00010000;  // Version 1.0.0

    // Register addresses
    localparam ADDR_CTRL     = 8'h00;
    localparam ADDR_SIZE     = 8'h04;
    localparam ADDR_STATUS   = 8'h08;
    localparam ADDR_DATA_IN  = 8'h10;
    localparam ADDR_DATA_OUT = 8'h14;
    localparam ADDR_VERSION  = 8'h1C;
    localparam ADDR_ID       = 8'h20;

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
    reg [3:0]  size_reg;         // log2(FFT size)

    wire start      = ctrl_reg[0];
    wire inverse    = ctrl_reg[1];
    wire soft_reset = ctrl_reg[31];

    // =========================================================================
    // Input Data FIFO
    // =========================================================================

    reg [31:0] data_in_fifo [0:1023];
    reg [10:0] fifo_wr_ptr;
    reg [10:0] fifo_rd_ptr;
    reg data_in_written;  // Pulse when data written via register

    wire fifo_empty = (fifo_wr_ptr == fifo_rd_ptr);
    wire fifo_full = (fifo_wr_ptr[9:0] == fifo_rd_ptr[9:0]) &&
                     (fifo_wr_ptr[10] != fifo_rd_ptr[10]);

    // =========================================================================
    // FFT Instance
    // =========================================================================

    wire done;
    wire busy;
    wire fft_error;

    // Stream interface for FFT core
    wire [31:0] fft_in_data;
    wire fft_in_valid;
    wire fft_in_ready;
    wire fft_in_last;

    wire [31:0] fft_out_data;
    wire fft_out_valid;
    wire fft_out_ready;
    wire fft_out_last;

    // Use AXI-Stream if available, otherwise use register interface
    assign fft_in_data = S_AXIS_TVALID ? S_AXIS_TDATA : data_in_fifo[fifo_rd_ptr[9:0]];
    assign fft_in_valid = S_AXIS_TVALID || (!fifo_empty && busy);
    assign S_AXIS_TREADY = fft_in_ready;
    assign fft_in_last = S_AXIS_TVALID ? S_AXIS_TLAST :
                         (fifo_rd_ptr[9:0] == (1 << size_reg) - 1);

    r4w_fft #(
        .MAX_NFFT(1024),
        .DATA_WIDTH(16)
    ) fft_inst (
        .clk(S_AXI_ACLK),
        .rst_n(S_AXI_ARESETN && !soft_reset),
        .start(start),
        .inverse(inverse),
        .nfft_log2(size_reg),
        .done(done),
        .busy(busy),
        .error(fft_error),
        .s_axis_tdata(fft_in_data),
        .s_axis_tvalid(fft_in_valid),
        .s_axis_tready(fft_in_ready),
        .s_axis_tlast(fft_in_last),
        .m_axis_tdata(fft_out_data),
        .m_axis_tvalid(fft_out_valid),
        .m_axis_tready(fft_out_ready),
        .m_axis_tlast(fft_out_last)
    );

    // Connect output to AXI-Stream
    assign M_AXIS_TDATA = fft_out_data;
    assign M_AXIS_TVALID = fft_out_valid;
    assign fft_out_ready = M_AXIS_TREADY;
    assign M_AXIS_TLAST = fft_out_last;

    // =========================================================================
    // Output Latch (for register reads)
    // =========================================================================

    reg [31:0] data_out_latch;

    always @(posedge clk) begin
        if (!S_AXI_ARESETN) begin
            data_out_latch <= 0;
        end else if (fft_out_valid) begin
            data_out_latch <= fft_out_data;
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

    always @(posedge S_AXI_ACLK) begin
        if (!S_AXI_ARESETN) begin
            ctrl_reg <= 32'h0;
            size_reg <= 4'd10;       // Default 1024-point FFT
            fifo_wr_ptr <= 0;
            fifo_rd_ptr <= 0;
            data_in_written <= 0;
        end else begin
            data_in_written <= 1'b0;

            // Auto-clear start bit
            if (ctrl_reg[0])
                ctrl_reg[0] <= 1'b0;

            // Auto-clear soft reset
            if (ctrl_reg[31])
                ctrl_reg[31] <= 1'b0;

            // Advance FIFO read when FFT consumes data
            if (!fifo_empty && busy && fft_in_ready)
                fifo_rd_ptr <= fifo_rd_ptr + 1;

            if (reg_wen) begin
                case (reg_addr)
                    ADDR_CTRL: begin
                        if (reg_wstrb[0]) ctrl_reg[7:0]   <= reg_wdata[7:0];
                        if (reg_wstrb[1]) ctrl_reg[15:8]  <= reg_wdata[15:8];
                        if (reg_wstrb[2]) ctrl_reg[23:16] <= reg_wdata[23:16];
                        if (reg_wstrb[3]) ctrl_reg[31:24] <= reg_wdata[31:24];
                    end
                    ADDR_SIZE: begin
                        if (reg_wstrb[0]) size_reg <= reg_wdata[3:0];
                    end
                    ADDR_DATA_IN: begin
                        // Write to input FIFO
                        if (!fifo_full) begin
                            data_in_fifo[fifo_wr_ptr[9:0]] <= reg_wdata;
                            fifo_wr_ptr <= fifo_wr_ptr + 1;
                            data_in_written <= 1'b1;
                        end
                    end
                endcase
            end

            // Reset FIFOs on soft reset
            if (soft_reset) begin
                fifo_wr_ptr <= 0;
                fifo_rd_ptr <= 0;
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
                    ADDR_CTRL:     reg_rdata <= ctrl_reg;
                    ADDR_SIZE:     reg_rdata <= {28'h0, size_reg};
                    ADDR_STATUS:   reg_rdata <= {29'h0, fft_error, busy, done};
                    ADDR_DATA_IN:  reg_rdata <= 32'h0;  // Write-only
                    ADDR_DATA_OUT: reg_rdata <= data_out_latch;
                    ADDR_VERSION:  reg_rdata <= IP_VERSION;
                    ADDR_ID:       reg_rdata <= IP_ID;
                    default:       reg_rdata <= 32'h0;
                endcase
            end
        end
    end

endmodule
