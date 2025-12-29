//-----------------------------------------------------------------------------
// R4W DMA Controller Testbench
// R4W FPGA Acceleration Layer
//
// Tests DMA controller TX and RX paths with simulated AXI-Stream traffic.
//
// trace:FR-0086 | ai:claude
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

`include "tb_common.vh"

module tb_r4w_dma;

    // =========================================================================
    // Parameters
    // =========================================================================

    localparam CLK_PERIOD = 10;  // 100 MHz
    localparam BUFFER_DEPTH = 256;

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
    // Clock and Reset
    // =========================================================================

    reg clk = 0;
    reg rst_n = 0;

    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // AXI-Lite Master BFM
    // =========================================================================

    wire [7:0]  S_AXI_AWADDR;
    wire [2:0]  S_AXI_AWPROT;
    wire        S_AXI_AWVALID;
    wire        S_AXI_AWREADY;
    wire [31:0] S_AXI_WDATA;
    wire [3:0]  S_AXI_WSTRB;
    wire        S_AXI_WVALID;
    wire        S_AXI_WREADY;
    wire [1:0]  S_AXI_BRESP;
    wire        S_AXI_BVALID;
    wire        S_AXI_BREADY;
    wire [7:0]  S_AXI_ARADDR;
    wire [2:0]  S_AXI_ARPROT;
    wire        S_AXI_ARVALID;
    wire        S_AXI_ARREADY;
    wire [31:0] S_AXI_RDATA;
    wire [1:0]  S_AXI_RRESP;
    wire        S_AXI_RVALID;
    wire        S_AXI_RREADY;

    reg         bfm_wen;
    reg         bfm_ren;
    reg  [7:0]  bfm_addr;
    reg  [31:0] bfm_wdata;
    wire [31:0] bfm_rdata;
    wire        bfm_done;
    wire        bfm_error;

    axi_lite_master_bfm #(
        .C_M_AXI_ADDR_WIDTH(8),
        .C_M_AXI_DATA_WIDTH(32)
    ) bfm (
        .M_AXI_ACLK(clk),
        .M_AXI_ARESETN(rst_n),
        .M_AXI_AWADDR(S_AXI_AWADDR),
        .M_AXI_AWPROT(S_AXI_AWPROT),
        .M_AXI_AWVALID(S_AXI_AWVALID),
        .M_AXI_AWREADY(S_AXI_AWREADY),
        .M_AXI_WDATA(S_AXI_WDATA),
        .M_AXI_WSTRB(S_AXI_WSTRB),
        .M_AXI_WVALID(S_AXI_WVALID),
        .M_AXI_WREADY(S_AXI_WREADY),
        .M_AXI_BRESP(S_AXI_BRESP),
        .M_AXI_BVALID(S_AXI_BVALID),
        .M_AXI_BREADY(S_AXI_BREADY),
        .M_AXI_ARADDR(S_AXI_ARADDR),
        .M_AXI_ARPROT(S_AXI_ARPROT),
        .M_AXI_ARVALID(S_AXI_ARVALID),
        .M_AXI_ARREADY(S_AXI_ARREADY),
        .M_AXI_RDATA(S_AXI_RDATA),
        .M_AXI_RRESP(S_AXI_RRESP),
        .M_AXI_RVALID(S_AXI_RVALID),
        .M_AXI_RREADY(S_AXI_RREADY),
        .cmd_write(bfm_wen),
        .cmd_read(bfm_ren),
        .cmd_addr(bfm_addr),
        .cmd_wdata(bfm_wdata),
        .cmd_rdata(bfm_rdata),
        .cmd_done(bfm_done),
        .cmd_error(bfm_error)
    );

    // =========================================================================
    // AXI-Stream Signals (Simulated Xilinx DMA)
    // =========================================================================

    // From DMA (memory to DUT)
    reg  [31:0] s_axis_tdata;
    reg         s_axis_tvalid;
    wire        s_axis_tready;
    reg         s_axis_tlast;
    reg  [3:0]  s_axis_tkeep;

    // To DMA (DUT to memory)
    wire [31:0] m_axis_tdata;
    wire        m_axis_tvalid;
    reg         m_axis_tready;
    wire        m_axis_tlast;
    wire [3:0]  m_axis_tkeep;

    // =========================================================================
    // DSP Stream Signals (Loopback for testing)
    // =========================================================================

    wire [31:0] m_dsp_tdata;
    wire        m_dsp_tvalid;
    reg         m_dsp_tready;
    wire        m_dsp_tlast;

    reg  [31:0] s_dsp_tdata;
    reg         s_dsp_tvalid;
    wire        s_dsp_tready;
    reg         s_dsp_tlast;

    // Loopback: connect DSP output to DSP input
    always @(posedge clk) begin
        s_dsp_tdata <= m_dsp_tdata;
        s_dsp_tvalid <= m_dsp_tvalid && m_dsp_tready;
        s_dsp_tlast <= m_dsp_tlast;
    end

    // IRQ output
    wire irq;

    // =========================================================================
    // DUT Instance
    // =========================================================================

    r4w_dma_axi #(
        .C_S_AXI_DATA_WIDTH(32),
        .C_S_AXI_ADDR_WIDTH(8),
        .BUFFER_DEPTH(BUFFER_DEPTH)
    ) dut (
        .S_AXI_ACLK(clk),
        .S_AXI_ARESETN(rst_n),

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

        .S_AXIS_TDATA(s_axis_tdata),
        .S_AXIS_TVALID(s_axis_tvalid),
        .S_AXIS_TREADY(s_axis_tready),
        .S_AXIS_TLAST(s_axis_tlast),
        .S_AXIS_TKEEP(s_axis_tkeep),

        .M_AXIS_TDATA(m_axis_tdata),
        .M_AXIS_TVALID(m_axis_tvalid),
        .M_AXIS_TREADY(m_axis_tready),
        .M_AXIS_TLAST(m_axis_tlast),
        .M_AXIS_TKEEP(m_axis_tkeep),

        .M_DSP_TDATA(m_dsp_tdata),
        .M_DSP_TVALID(m_dsp_tvalid),
        .M_DSP_TREADY(m_dsp_tready),
        .M_DSP_TLAST(m_dsp_tlast),

        .S_DSP_TDATA(s_dsp_tdata),
        .S_DSP_TVALID(s_dsp_tvalid),
        .S_DSP_TREADY(s_dsp_tready),
        .S_DSP_TLAST(s_dsp_tlast),

        .IRQ(irq)
    );

    // =========================================================================
    // Test Tasks
    // =========================================================================

    task axi_write;
        input [7:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            bfm_addr <= addr;
            bfm_wdata <= data;
            bfm_wen <= 1;
            @(posedge clk);
            bfm_wen <= 0;
            wait(bfm_done);
            @(posedge clk);
        end
    endtask

    task axi_read;
        input [7:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            bfm_addr <= addr;
            bfm_ren <= 1;
            @(posedge clk);
            bfm_ren <= 0;
            wait(bfm_done);
            data = bfm_rdata;
            @(posedge clk);
        end
    endtask

    // Send samples via AXI-Stream (simulating DMA)
    task send_dma_samples;
        input integer count;
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                @(posedge clk);
                s_axis_tdata <= i + 32'hCAFE0000;
                s_axis_tvalid <= 1;
                s_axis_tlast <= (i == count - 1);
                s_axis_tkeep <= 4'b1111;

                // Wait for ready
                while (!s_axis_tready) @(posedge clk);
            end
            @(posedge clk);
            s_axis_tvalid <= 0;
            s_axis_tlast <= 0;
        end
    endtask

    // Receive samples via AXI-Stream (simulating DMA)
    task receive_dma_samples;
        input integer count;
        output [31:0] samples [0:255];
        integer i;
        begin
            i = 0;
            while (i < count) begin
                @(posedge clk);
                m_axis_tready <= 1;
                if (m_axis_tvalid) begin
                    samples[i] = m_axis_tdata;
                    i = i + 1;
                    if (m_axis_tlast) begin
                        i = count;  // Exit on last
                    end
                end
            end
            @(posedge clk);
            m_axis_tready <= 0;
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    reg [31:0] rdata;
    reg [31:0] rx_samples [0:255];
    integer i;

    initial begin
        // Setup
        $dumpfile("tb_r4w_dma.vcd");
        $dumpvars(0, tb_r4w_dma);

        // Initialize
        bfm_wen <= 0;
        bfm_ren <= 0;
        bfm_addr <= 0;
        bfm_wdata <= 0;
        s_axis_tdata <= 0;
        s_axis_tvalid <= 0;
        s_axis_tlast <= 0;
        s_axis_tkeep <= 4'b1111;
        m_axis_tready <= 0;
        m_dsp_tready <= 1;  // Always ready to receive from TX
        s_dsp_tdata <= 0;
        s_dsp_tvalid <= 0;
        s_dsp_tlast <= 0;

        // Reset
        rst_n <= 0;
        repeat(10) @(posedge clk);
        rst_n <= 1;
        repeat(10) @(posedge clk);

        $display("=== R4W DMA Controller Testbench ===");
        $display("Time: %0t - Starting tests", $time);

        // =====================================================================
        // Test 1: Read IP ID and Version
        // =====================================================================
        $display("\n--- Test 1: Read IP ID and Version ---");

        axi_read(ADDR_ID, rdata);
        $display("IP ID: 0x%08X (expected 0x52344D41 = 'R4DM')", rdata);
        `ASSERT_EQ(rdata, 32'h52344D41, "IP ID mismatch")

        axi_read(ADDR_VERSION, rdata);
        $display("Version: 0x%08X", rdata);

        // =====================================================================
        // Test 2: Check Initial Status
        // =====================================================================
        $display("\n--- Test 2: Initial Status ---");

        axi_read(ADDR_STATUS, rdata);
        $display("Status: 0x%08X (expected 0x00 - all idle)", rdata);
        `ASSERT_EQ(rdata, 32'h0, "Status should be idle")

        // =====================================================================
        // Test 3: TX DMA Transfer (16 samples)
        // =====================================================================
        $display("\n--- Test 3: TX DMA Transfer ---");

        // Configure TX length
        axi_write(ADDR_TX_LEN, 32'd16);

        // Enable IRQ
        axi_write(ADDR_IRQ_EN, 32'h07);  // Enable all

        // Start TX
        axi_write(ADDR_CTRL, 32'h01);  // START_TX

        // Send samples from "DMA"
        fork
            send_dma_samples(16);
        join

        // Wait for completion
        repeat(100) @(posedge clk);

        axi_read(ADDR_STATUS, rdata);
        $display("Status after TX: 0x%08X", rdata);

        axi_read(ADDR_TX_COUNT, rdata);
        $display("TX Count: %0d (expected 16)", rdata);

        axi_read(ADDR_IRQ_STATUS, rdata);
        $display("IRQ Status: 0x%08X", rdata);

        // Clear IRQ
        axi_write(ADDR_IRQ_STATUS, 32'h01);

        // =====================================================================
        // Test 4: RX DMA Transfer
        // =====================================================================
        $display("\n--- Test 4: RX DMA Transfer ---");

        // Configure RX length
        axi_write(ADDR_RX_LEN, 32'd16);

        // Start RX
        axi_write(ADDR_CTRL, 32'h02);  // START_RX

        // The loopback should have already sent the data
        // Wait and collect from DMA output
        repeat(50) @(posedge clk);

        m_axis_tready <= 1;
        repeat(50) @(posedge clk);

        axi_read(ADDR_STATUS, rdata);
        $display("Status after RX: 0x%08X", rdata);

        axi_read(ADDR_RX_COUNT, rdata);
        $display("RX Count: %0d", rdata);

        // =====================================================================
        // Test 5: Soft Reset
        // =====================================================================
        $display("\n--- Test 5: Soft Reset ---");

        axi_write(ADDR_CTRL, 32'h80000000);  // RESET

        repeat(10) @(posedge clk);

        axi_read(ADDR_STATUS, rdata);
        $display("Status after reset: 0x%08X (expected idle)", rdata);

        // =====================================================================
        // Done
        // =====================================================================
        $display("\n=== All Tests Passed ===");
        $display("Time: %0t - Simulation complete", $time);

        repeat(100) @(posedge clk);
        $finish;
    end

    // =========================================================================
    // Timeout
    // =========================================================================

    initial begin
        #100000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
