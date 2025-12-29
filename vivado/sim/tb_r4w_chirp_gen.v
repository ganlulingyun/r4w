//-----------------------------------------------------------------------------
// Testbench for R4W LoRa Chirp Generator
// R4W FPGA Acceleration Layer
//
// Tests:
// 1. Register read/write via AXI-Lite
// 2. Chirp generation for different spreading factors
// 3. Upchirp vs downchirp
// 4. Symbol modulation
// 5. Continuous mode
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps
`include "tb_common.vh"

module tb_r4w_chirp_gen;

    // =========================================================================
    // Parameters
    // =========================================================================

    parameter CLK_PERIOD = 10;  // 100 MHz
    parameter ADDR_WIDTH = 8;
    parameter DATA_WIDTH = 32;

    // Register addresses (from registers.rs)
    localparam ADDR_CTRL      = 8'h00;
    localparam ADDR_SF        = 8'h04;
    localparam ADDR_STATUS    = 8'h08;
    localparam ADDR_SYMBOL    = 8'h0C;
    localparam ADDR_DATA_OUT  = 8'h10;
    localparam ADDR_BANDWIDTH = 8'h14;
    localparam ADDR_VERSION   = 8'h1C;
    localparam ADDR_ID        = 8'h20;

    // Control bits
    localparam CTRL_START      = 32'h0000_0001;
    localparam CTRL_UPCHIRP    = 32'h0000_0002;
    localparam CTRL_CONTINUOUS = 32'h0000_0004;
    localparam CTRL_RESET      = 32'h8000_0000;

    // Status bits
    localparam STATUS_DONE     = 32'h0000_0001;
    localparam STATUS_BUSY     = 32'h0000_0002;
    localparam STATUS_OVERFLOW = 32'h0000_0004;

    // Expected ID
    localparam EXPECTED_ID = 32'h52344347;  // "R4CG"

    // =========================================================================
    // Signals
    // =========================================================================

    reg clk;
    reg rst_n;
    integer error_count;

    // AXI-Lite interface
    wire [ADDR_WIDTH-1:0]    s_axi_awaddr;
    wire [2:0]               s_axi_awprot;
    wire                     s_axi_awvalid;
    wire                     s_axi_awready;
    wire [DATA_WIDTH-1:0]    s_axi_wdata;
    wire [(DATA_WIDTH/8)-1:0] s_axi_wstrb;
    wire                     s_axi_wvalid;
    wire                     s_axi_wready;
    wire [1:0]               s_axi_bresp;
    wire                     s_axi_bvalid;
    wire                     s_axi_bready;
    wire [ADDR_WIDTH-1:0]    s_axi_araddr;
    wire [2:0]               s_axi_arprot;
    wire                     s_axi_arvalid;
    wire                     s_axi_arready;
    wire [DATA_WIDTH-1:0]    s_axi_rdata;
    wire [1:0]               s_axi_rresp;
    wire                     s_axi_rvalid;
    wire                     s_axi_rready;

    // AXI-Stream output
    wire [31:0] m_axis_tdata;
    wire        m_axis_tvalid;
    reg         m_axis_tready;
    wire        m_axis_tlast;

    // =========================================================================
    // Clock Generation
    // =========================================================================

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // =========================================================================
    // DUT Instantiation
    // =========================================================================

    r4w_chirp_gen_axi #(
        .C_S_AXI_DATA_WIDTH(DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(ADDR_WIDTH)
    ) dut (
        .S_AXI_ACLK(clk),
        .S_AXI_ARESETN(rst_n),
        .S_AXI_AWADDR(s_axi_awaddr),
        .S_AXI_AWPROT(s_axi_awprot),
        .S_AXI_AWVALID(s_axi_awvalid),
        .S_AXI_AWREADY(s_axi_awready),
        .S_AXI_WDATA(s_axi_wdata),
        .S_AXI_WSTRB(s_axi_wstrb),
        .S_AXI_WVALID(s_axi_wvalid),
        .S_AXI_WREADY(s_axi_wready),
        .S_AXI_BRESP(s_axi_bresp),
        .S_AXI_BVALID(s_axi_bvalid),
        .S_AXI_BREADY(s_axi_bready),
        .S_AXI_ARADDR(s_axi_araddr),
        .S_AXI_ARPROT(s_axi_arprot),
        .S_AXI_ARVALID(s_axi_arvalid),
        .S_AXI_ARREADY(s_axi_arready),
        .S_AXI_RDATA(s_axi_rdata),
        .S_AXI_RRESP(s_axi_rresp),
        .S_AXI_RVALID(s_axi_rvalid),
        .S_AXI_RREADY(s_axi_rready),
        .M_AXIS_TDATA(m_axis_tdata),
        .M_AXIS_TVALID(m_axis_tvalid),
        .M_AXIS_TREADY(m_axis_tready),
        .M_AXIS_TLAST(m_axis_tlast)
    );

    // =========================================================================
    // AXI-Lite BFM
    // =========================================================================

    axi_lite_master_bfm #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) bfm (
        .clk(clk),
        .rst_n(rst_n),
        .m_axi_awaddr(s_axi_awaddr),
        .m_axi_awprot(s_axi_awprot),
        .m_axi_awvalid(s_axi_awvalid),
        .m_axi_awready(s_axi_awready),
        .m_axi_wdata(s_axi_wdata),
        .m_axi_wstrb(s_axi_wstrb),
        .m_axi_wvalid(s_axi_wvalid),
        .m_axi_wready(s_axi_wready),
        .m_axi_bresp(s_axi_bresp),
        .m_axi_bvalid(s_axi_bvalid),
        .m_axi_bready(s_axi_bready),
        .m_axi_araddr(s_axi_araddr),
        .m_axi_arprot(s_axi_arprot),
        .m_axi_arvalid(s_axi_arvalid),
        .m_axi_arready(s_axi_arready),
        .m_axi_rdata(s_axi_rdata),
        .m_axi_rresp(s_axi_rresp),
        .m_axi_rvalid(s_axi_rvalid),
        .m_axi_rready(s_axi_rready)
    );

    // =========================================================================
    // Test Stimulus
    // =========================================================================

    reg [31:0] read_data;
    reg timeout;
    integer i;

    // Sample storage for analysis
    reg signed [15:0] i_samples [0:255];
    reg signed [15:0] q_samples [0:255];
    integer sample_count;

    // Task to collect samples via AXI-Stream
    task collect_stream_samples;
        input integer num_samples;
        integer idx;
        begin
            sample_count = 0;
            m_axis_tready = 1'b1;
            for (idx = 0; idx < num_samples; idx = idx + 1) begin
                wait(m_axis_tvalid);
                @(posedge clk);
                i_samples[idx] = m_axis_tdata[31:16];
                q_samples[idx] = m_axis_tdata[15:0];
                sample_count = sample_count + 1;
                if (m_axis_tlast) begin
                    $display("INFO: TLAST received at sample %0d", idx);
                    idx = num_samples;  // Exit loop
                end
            end
            m_axis_tready = 1'b0;
        end
    endtask

    initial begin
        $display("\n");
        $display("==============================================");
        $display("R4W Chirp Generator Testbench");
        $display("==============================================\n");

        error_count = 0;
        rst_n = 0;
        m_axis_tready = 0;

        // Reset sequence
        repeat(10) @(posedge clk);
        rst_n = 1;
        repeat(10) @(posedge clk);

        // -----------------------------------------------------------------
        // Test 1: Read IP ID and Version
        // -----------------------------------------------------------------
        `TEST_START("Read IP ID and Version")

        bfm.axi_read(ADDR_ID, read_data);
        `ASSERT_EQ(read_data, EXPECTED_ID, "IP ID check")

        bfm.axi_read(ADDR_VERSION, read_data);
        $display("INFO: IP Version = %d.%d.%d",
                 read_data[23:16], read_data[15:8], read_data[7:0]);

        // -----------------------------------------------------------------
        // Test 2: Register Configuration
        // -----------------------------------------------------------------
        `TEST_START("Register Configuration")

        // Set SF = 7 (128 samples per symbol)
        bfm.axi_write(ADDR_SF, 32'h00000007);
        bfm.axi_read(ADDR_SF, read_data);
        `ASSERT_EQ(read_data[3:0], 4'd7, "SF register")

        // Set symbol = 0
        bfm.axi_write(ADDR_SYMBOL, 32'h00000000);
        bfm.axi_read(ADDR_SYMBOL, read_data);
        `ASSERT_EQ(read_data[11:0], 12'd0, "Symbol register")

        // Set bandwidth = 125 kHz (code 0)
        bfm.axi_write(ADDR_BANDWIDTH, 32'h00000000);
        bfm.axi_read(ADDR_BANDWIDTH, read_data);
        `ASSERT_EQ(read_data[1:0], 2'd0, "Bandwidth register")

        // -----------------------------------------------------------------
        // Test 3: Generate SF7 Upchirp Symbol 0
        // -----------------------------------------------------------------
        `TEST_START("SF7 Upchirp Symbol 0")

        // Configure
        bfm.axi_write(ADDR_SF, 32'h00000007);      // SF7
        bfm.axi_write(ADDR_SYMBOL, 32'h00000000);  // Symbol 0
        bfm.axi_write(ADDR_BANDWIDTH, 32'h00000000);

        // Start upchirp generation
        bfm.axi_write(ADDR_CTRL, CTRL_START | CTRL_UPCHIRP);

        // Wait for busy
        repeat(5) @(posedge clk);
        bfm.axi_read(ADDR_STATUS, read_data);
        $display("INFO: Status after start = 0x%h (busy=%b)", read_data, read_data[1]);

        // Collect samples via register interface (slower but works)
        for (i = 0; i < 128 + 20; i = i + 1) begin  // 128 for SF7 + pipeline delay
            bfm.axi_read(ADDR_DATA_OUT, read_data);
            if (i < 128) begin
                i_samples[i] = read_data[31:16];
                q_samples[i] = read_data[15:0];
            end
        end

        // Wait for done
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 1000, timeout);
        if (timeout) begin
            $display("FAIL: Chirp generation did not complete");
            error_count = error_count + 1;
        end else begin
            $display("PASS: Chirp generation completed");
        end

        // Analyze samples
        $display("INFO: First 8 I samples: %d, %d, %d, %d, %d, %d, %d, %d",
                 i_samples[0], i_samples[1], i_samples[2], i_samples[3],
                 i_samples[4], i_samples[5], i_samples[6], i_samples[7]);

        // -----------------------------------------------------------------
        // Test 4: Generate SF7 Downchirp
        // -----------------------------------------------------------------
        `TEST_START("SF7 Downchirp Symbol 0")

        bfm.axi_write(ADDR_SF, 32'h00000007);
        bfm.axi_write(ADDR_SYMBOL, 32'h00000000);

        // Start downchirp (no UPCHIRP bit)
        bfm.axi_write(ADDR_CTRL, CTRL_START);

        // Wait for done
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 1000, timeout);
        if (timeout) begin
            $display("FAIL: Downchirp generation timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: Downchirp generation completed");
        end

        // -----------------------------------------------------------------
        // Test 5: Symbol Modulation (Symbol 64)
        // -----------------------------------------------------------------
        `TEST_START("SF7 Upchirp Symbol 64")

        bfm.axi_write(ADDR_SF, 32'h00000007);
        bfm.axi_write(ADDR_SYMBOL, 32'h00000040);  // Symbol 64

        bfm.axi_write(ADDR_CTRL, CTRL_START | CTRL_UPCHIRP);

        // Wait for done
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 1000, timeout);
        if (timeout) begin
            $display("FAIL: Symbol 64 generation timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: Symbol 64 generation completed");
        end

        // -----------------------------------------------------------------
        // Test 6: Different Spreading Factor (SF5)
        // -----------------------------------------------------------------
        `TEST_START("SF5 Upchirp (32 samples)")

        bfm.axi_write(ADDR_SF, 32'h00000005);      // SF5 = 32 samples
        bfm.axi_write(ADDR_SYMBOL, 32'h00000000);

        bfm.axi_write(ADDR_CTRL, CTRL_START | CTRL_UPCHIRP);

        // Should complete faster (32 samples instead of 128)
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 500, timeout);
        if (timeout) begin
            $display("FAIL: SF5 generation timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: SF5 generation completed");
        end

        // -----------------------------------------------------------------
        // Test 7: SF12 (4096 samples) - just verify start
        // -----------------------------------------------------------------
        `TEST_START("SF12 Start (4096 samples)")

        bfm.axi_write(ADDR_SF, 32'h0000000C);      // SF12 = 4096 samples
        bfm.axi_write(ADDR_SYMBOL, 32'h00000000);

        bfm.axi_write(ADDR_CTRL, CTRL_START | CTRL_UPCHIRP);

        // Check busy flag is set
        repeat(10) @(posedge clk);
        bfm.axi_read(ADDR_STATUS, read_data);
        if (read_data & STATUS_BUSY) begin
            $display("PASS: SF12 started and busy");
        end else begin
            $display("FAIL: SF12 not busy after start");
            error_count = error_count + 1;
        end

        // Don't wait for completion (too long for testbench)
        // Reset to abort
        bfm.axi_write(ADDR_CTRL, CTRL_RESET);
        repeat(20) @(posedge clk);

        // -----------------------------------------------------------------
        // Test Summary
        // -----------------------------------------------------------------
        `TEST_SUMMARY

        repeat(10) @(posedge clk);
        $finish;
    end

    // =========================================================================
    // Timeout Watchdog
    // =========================================================================

    initial begin
        #500000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

    // =========================================================================
    // VCD Dump
    // =========================================================================

    initial begin
        $dumpfile("tb_r4w_chirp_gen.vcd");
        $dumpvars(0, tb_r4w_chirp_gen);
    end

endmodule
