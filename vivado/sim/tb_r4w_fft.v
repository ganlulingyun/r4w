//-----------------------------------------------------------------------------
// Testbench for R4W FFT/IFFT Processor
// R4W FPGA Acceleration Layer
//
// Tests:
// 1. Register read/write via AXI-Lite
// 2. FFT of DC signal (all bins zero except bin 0)
// 3. FFT of single tone (peak at expected bin)
// 4. IFFT roundtrip
// 5. Different FFT sizes
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps
`include "tb_common.vh"

module tb_r4w_fft;

    // =========================================================================
    // Parameters
    // =========================================================================

    parameter CLK_PERIOD = 10;  // 100 MHz
    parameter ADDR_WIDTH = 8;
    parameter DATA_WIDTH = 32;

    // Register addresses (from registers.rs)
    localparam ADDR_CTRL     = 8'h00;
    localparam ADDR_SIZE     = 8'h04;
    localparam ADDR_STATUS   = 8'h08;
    localparam ADDR_DATA_IN  = 8'h10;
    localparam ADDR_DATA_OUT = 8'h14;
    localparam ADDR_VERSION  = 8'h1C;
    localparam ADDR_ID       = 8'h20;

    // Control bits
    localparam CTRL_START   = 32'h0000_0001;
    localparam CTRL_INVERSE = 32'h0000_0002;
    localparam CTRL_RESET   = 32'h8000_0000;

    // Status bits
    localparam STATUS_DONE  = 32'h0000_0001;
    localparam STATUS_BUSY  = 32'h0000_0002;
    localparam STATUS_ERROR = 32'h0000_0004;

    // Expected ID
    localparam EXPECTED_ID = 32'h52345746;  // "R4WF"

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

    // AXI-Stream interfaces
    reg  [31:0] s_axis_tdata;
    reg         s_axis_tvalid;
    wire        s_axis_tready;
    reg         s_axis_tlast;

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

    r4w_fft_axi #(
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
        .S_AXIS_TDATA(s_axis_tdata),
        .S_AXIS_TVALID(s_axis_tvalid),
        .S_AXIS_TREADY(s_axis_tready),
        .S_AXIS_TLAST(s_axis_tlast),
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
    integer fft_size;

    // Sample storage
    reg signed [15:0] input_i [0:1023];
    reg signed [15:0] input_q [0:1023];
    reg signed [15:0] output_i [0:1023];
    reg signed [15:0] output_q [0:1023];

    // Sine table for test signal generation
    real pi;

    initial begin
        pi = 3.14159265359;
        $display("\n");
        $display("==============================================");
        $display("R4W FFT Testbench");
        $display("==============================================\n");

        error_count = 0;
        rst_n = 0;
        s_axis_tdata = 0;
        s_axis_tvalid = 0;
        s_axis_tlast = 0;
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

        // Set FFT size to 64 (log2 = 6)
        bfm.axi_write(ADDR_SIZE, 32'h00000006);
        bfm.axi_read(ADDR_SIZE, read_data);
        `ASSERT_EQ(read_data[3:0], 4'd6, "Size register (64-point)")

        // Set FFT size to 256 (log2 = 8)
        bfm.axi_write(ADDR_SIZE, 32'h00000008);
        bfm.axi_read(ADDR_SIZE, read_data);
        `ASSERT_EQ(read_data[3:0], 4'd8, "Size register (256-point)")

        // -----------------------------------------------------------------
        // Test 3: 64-point FFT of DC Signal
        // -----------------------------------------------------------------
        `TEST_START("64-point FFT of DC Signal")

        fft_size = 64;
        bfm.axi_write(ADDR_SIZE, 32'h00000006);  // log2(64) = 6

        // Generate DC input (constant value)
        for (i = 0; i < fft_size; i = i + 1) begin
            input_i[i] = 16'h1000;  // DC level
            input_q[i] = 16'h0000;
        end

        // Write samples via register interface
        for (i = 0; i < fft_size; i = i + 1) begin
            bfm.axi_write(ADDR_DATA_IN, {input_i[i], input_q[i]});
        end

        // Start FFT
        bfm.axi_write(ADDR_CTRL, CTRL_START);

        // Wait for completion
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 5000, timeout);
        if (timeout) begin
            $display("FAIL: 64-point FFT timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: 64-point FFT completed");
        end

        // Read output samples
        for (i = 0; i < fft_size; i = i + 1) begin
            bfm.axi_read(ADDR_DATA_OUT, read_data);
            output_i[i] = read_data[31:16];
            output_q[i] = read_data[15:0];
        end

        // DC signal should have energy only in bin 0
        $display("INFO: FFT output bin 0: I=%d, Q=%d", output_i[0], output_q[0]);
        $display("INFO: FFT output bin 1: I=%d, Q=%d", output_i[1], output_q[1]);
        $display("INFO: FFT output bin 32: I=%d, Q=%d", output_i[32], output_q[32]);

        // -----------------------------------------------------------------
        // Test 4: 64-point FFT of Single Tone
        // -----------------------------------------------------------------
        `TEST_START("64-point FFT of Single Tone (bin 8)")

        fft_size = 64;
        bfm.axi_write(ADDR_SIZE, 32'h00000006);

        // Soft reset to clear previous state
        bfm.axi_write(ADDR_CTRL, CTRL_RESET);
        repeat(10) @(posedge clk);

        // Generate single tone at bin 8 (8 cycles in 64 samples)
        for (i = 0; i < fft_size; i = i + 1) begin
            input_i[i] = $rtoi(16384.0 * $cos(2.0 * pi * 8.0 * i / fft_size));
            input_q[i] = $rtoi(16384.0 * $sin(2.0 * pi * 8.0 * i / fft_size));
        end

        // Write samples
        for (i = 0; i < fft_size; i = i + 1) begin
            bfm.axi_write(ADDR_DATA_IN, {input_i[i], input_q[i]});
        end

        // Start FFT
        bfm.axi_write(ADDR_CTRL, CTRL_START);

        // Wait for completion
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 5000, timeout);
        if (timeout) begin
            $display("FAIL: Single tone FFT timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: Single tone FFT completed");
        end

        // Read output
        for (i = 0; i < fft_size; i = i + 1) begin
            bfm.axi_read(ADDR_DATA_OUT, read_data);
            output_i[i] = read_data[31:16];
            output_q[i] = read_data[15:0];
        end

        // Should have peak at bin 8
        $display("INFO: FFT output bin 0: I=%d, Q=%d", output_i[0], output_q[0]);
        $display("INFO: FFT output bin 7: I=%d, Q=%d", output_i[7], output_q[7]);
        $display("INFO: FFT output bin 8: I=%d, Q=%d (expected peak)", output_i[8], output_q[8]);
        $display("INFO: FFT output bin 9: I=%d, Q=%d", output_i[9], output_q[9]);

        // -----------------------------------------------------------------
        // Test 5: IFFT Mode
        // -----------------------------------------------------------------
        `TEST_START("64-point IFFT")

        fft_size = 64;
        bfm.axi_write(ADDR_SIZE, 32'h00000006);

        // Soft reset
        bfm.axi_write(ADDR_CTRL, CTRL_RESET);
        repeat(10) @(posedge clk);

        // Use FFT output as IFFT input (should recover original signal)
        for (i = 0; i < fft_size; i = i + 1) begin
            bfm.axi_write(ADDR_DATA_IN, {output_i[i], output_q[i]});
        end

        // Start IFFT
        bfm.axi_write(ADDR_CTRL, CTRL_START | CTRL_INVERSE);

        // Wait for completion
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 5000, timeout);
        if (timeout) begin
            $display("FAIL: IFFT timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: IFFT completed");
        end

        // Read IFFT output
        for (i = 0; i < 8; i = i + 1) begin
            bfm.axi_read(ADDR_DATA_OUT, read_data);
            $display("INFO: IFFT output[%0d]: I=%d, Q=%d", i,
                     $signed(read_data[31:16]), $signed(read_data[15:0]));
        end

        // -----------------------------------------------------------------
        // Test 6: 256-point FFT
        // -----------------------------------------------------------------
        `TEST_START("256-point FFT")

        fft_size = 256;
        bfm.axi_write(ADDR_SIZE, 32'h00000008);  // log2(256) = 8

        // Soft reset
        bfm.axi_write(ADDR_CTRL, CTRL_RESET);
        repeat(10) @(posedge clk);

        // Generate impulse (delta function)
        for (i = 0; i < fft_size; i = i + 1) begin
            if (i == 0) begin
                input_i[i] = 16'h4000;
                input_q[i] = 16'h0000;
            end else begin
                input_i[i] = 16'h0000;
                input_q[i] = 16'h0000;
            end
        end

        // Write samples
        for (i = 0; i < fft_size; i = i + 1) begin
            bfm.axi_write(ADDR_DATA_IN, {input_i[i], input_q[i]});
        end

        // Start FFT
        bfm.axi_write(ADDR_CTRL, CTRL_START);

        // Wait for completion
        bfm.axi_poll(ADDR_STATUS, STATUS_DONE, STATUS_DONE, 10000, timeout);
        if (timeout) begin
            $display("FAIL: 256-point FFT timeout");
            error_count = error_count + 1;
        end else begin
            $display("PASS: 256-point FFT completed");
        end

        // Impulse FFT should be flat (all bins equal)
        bfm.axi_read(ADDR_DATA_OUT, read_data);
        $display("INFO: 256-pt FFT bin 0: I=%d, Q=%d",
                 $signed(read_data[31:16]), $signed(read_data[15:0]));

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
        #1000000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

    // =========================================================================
    // VCD Dump
    // =========================================================================

    initial begin
        $dumpfile("tb_r4w_fft.vcd");
        $dumpvars(0, tb_r4w_fft);
    end

endmodule
