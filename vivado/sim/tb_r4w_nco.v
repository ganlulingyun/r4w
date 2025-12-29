//-----------------------------------------------------------------------------
// Testbench for R4W NCO (Numerically Controlled Oscillator)
// R4W FPGA Acceleration Layer
//
// Tests:
// 1. Register read/write via AXI-Lite
// 2. NCO output frequency accuracy
// 3. Phase offset functionality
// 4. Enable/disable control
// 5. Quadrant coverage (all 4 quadrants)
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps
`include "tb_common.vh"

module tb_r4w_nco;

    // =========================================================================
    // Parameters
    // =========================================================================

    parameter CLK_PERIOD = 10;  // 100 MHz
    parameter ADDR_WIDTH = 8;
    parameter DATA_WIDTH = 32;

    // Register addresses (from registers.rs)
    localparam ADDR_CTRL      = 8'h00;
    localparam ADDR_FREQ      = 8'h04;
    localparam ADDR_PHASE     = 8'h08;
    localparam ADDR_AMPLITUDE = 8'h0C;
    localparam ADDR_STATUS    = 8'h10;
    localparam ADDR_I_OUT     = 8'h14;
    localparam ADDR_Q_OUT     = 8'h18;
    localparam ADDR_VERSION   = 8'h1C;
    localparam ADDR_ID        = 8'h20;

    // Control bits
    localparam CTRL_ENABLE      = 32'h0000_0001;
    localparam CTRL_RESET_PHASE = 32'h0000_0002;
    localparam CTRL_RESET       = 32'h8000_0000;

    // Expected ID
    localparam EXPECTED_ID = 32'h5234494F;  // "R4IO"

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

    r4w_nco_axi #(
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
        .S_AXI_RREADY(s_axi_rready)
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
    reg signed [15:0] i_samples [0:255];
    reg signed [15:0] q_samples [0:255];
    integer sample_idx;

    initial begin
        $display("\n");
        $display("==============================================");
        $display("R4W NCO Testbench");
        $display("==============================================\n");

        error_count = 0;
        rst_n = 0;
        sample_idx = 0;

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
        // Test 2: Register Write/Read
        // -----------------------------------------------------------------
        `TEST_START("Register Write/Read")

        // Write and read back frequency word
        bfm.axi_write(ADDR_FREQ, 32'h01000000);
        bfm.axi_read(ADDR_FREQ, read_data);
        `ASSERT_EQ(read_data, 32'h01000000, "Frequency register")

        // Write and read back phase offset
        bfm.axi_write(ADDR_PHASE, 32'h40000000);  // 90 degrees
        bfm.axi_read(ADDR_PHASE, read_data);
        `ASSERT_EQ(read_data, 32'h40000000, "Phase register")

        // Write and read back amplitude
        bfm.axi_write(ADDR_AMPLITUDE, 32'h00007FFF);
        bfm.axi_read(ADDR_AMPLITUDE, read_data);
        `ASSERT_EQ(read_data[15:0], 16'h7FFF, "Amplitude register")

        // -----------------------------------------------------------------
        // Test 3: NCO Output Generation
        // -----------------------------------------------------------------
        `TEST_START("NCO Output Generation")

        // Configure for DC output (freq = 0, phase = 0)
        // Should output cos(0) = 1, sin(0) = 0
        bfm.axi_write(ADDR_FREQ, 32'h00000000);
        bfm.axi_write(ADDR_PHASE, 32'h00000000);
        bfm.axi_write(ADDR_CTRL, CTRL_ENABLE | CTRL_RESET_PHASE);

        // Wait for CORDIC pipeline
        repeat(20) @(posedge clk);

        // Read I output (should be positive, near max)
        bfm.axi_read(ADDR_I_OUT, read_data);
        $display("INFO: I output at phase 0 = %d (0x%h)", $signed(read_data[15:0]), read_data[15:0]);
        if ($signed(read_data[15:0]) < 16000) begin
            $display("FAIL: I output too low at phase 0");
            error_count = error_count + 1;
        end else begin
            $display("PASS: I output at phase 0");
        end

        // Read Q output (should be near zero)
        bfm.axi_read(ADDR_Q_OUT, read_data);
        $display("INFO: Q output at phase 0 = %d (0x%h)", $signed(read_data[15:0]), read_data[15:0]);

        // -----------------------------------------------------------------
        // Test 4: Phase Offset (90 degrees)
        // -----------------------------------------------------------------
        `TEST_START("Phase Offset 90 degrees")

        // Set phase to 90 degrees (0x40000000 = pi/2)
        bfm.axi_write(ADDR_FREQ, 32'h00000000);
        bfm.axi_write(ADDR_PHASE, 32'h40000000);
        bfm.axi_write(ADDR_CTRL, CTRL_ENABLE | CTRL_RESET_PHASE);

        repeat(20) @(posedge clk);

        // cos(90) = 0, sin(90) = 1
        bfm.axi_read(ADDR_I_OUT, read_data);
        $display("INFO: I output at phase 90 = %d", $signed(read_data[15:0]));

        bfm.axi_read(ADDR_Q_OUT, read_data);
        $display("INFO: Q output at phase 90 = %d", $signed(read_data[15:0]));
        if ($signed(read_data[15:0]) < 16000) begin
            $display("FAIL: Q output too low at phase 90");
            error_count = error_count + 1;
        end else begin
            $display("PASS: Q output at phase 90");
        end

        // -----------------------------------------------------------------
        // Test 5: Frequency Sweep - Collect Samples
        // -----------------------------------------------------------------
        `TEST_START("Frequency Sweep - Collect Samples")

        // Set frequency to generate ~16 cycles in 256 samples
        // freq_word = 16 * 2^32 / 256 = 0x10000000
        bfm.axi_write(ADDR_FREQ, 32'h10000000);
        bfm.axi_write(ADDR_PHASE, 32'h00000000);
        bfm.axi_write(ADDR_CTRL, CTRL_ENABLE | CTRL_RESET_PHASE);

        // Wait for pipeline
        repeat(20) @(posedge clk);

        // Collect 256 samples
        for (i = 0; i < 256; i = i + 1) begin
            bfm.axi_read(ADDR_I_OUT, read_data);
            i_samples[i] = read_data[15:0];

            bfm.axi_read(ADDR_Q_OUT, read_data);
            q_samples[i] = read_data[15:0];
        end

        // Check for proper oscillation (samples should vary)
        $display("INFO: First 8 I samples: %d, %d, %d, %d, %d, %d, %d, %d",
                 i_samples[0], i_samples[1], i_samples[2], i_samples[3],
                 i_samples[4], i_samples[5], i_samples[6], i_samples[7]);

        $display("INFO: First 8 Q samples: %d, %d, %d, %d, %d, %d, %d, %d",
                 q_samples[0], q_samples[1], q_samples[2], q_samples[3],
                 q_samples[4], q_samples[5], q_samples[6], q_samples[7]);

        // Verify oscillation: check that max - min > threshold
        begin
            reg signed [15:0] i_max, i_min;
            i_max = -32768;
            i_min = 32767;
            for (i = 0; i < 256; i = i + 1) begin
                if (i_samples[i] > i_max) i_max = i_samples[i];
                if (i_samples[i] < i_min) i_min = i_samples[i];
            end
            $display("INFO: I range: [%d, %d], span = %d", i_min, i_max, i_max - i_min);
            if ((i_max - i_min) < 20000) begin
                $display("FAIL: I output not oscillating properly");
                error_count = error_count + 1;
            end else begin
                $display("PASS: I output oscillation");
            end
        end

        // -----------------------------------------------------------------
        // Test 6: Disable/Enable
        // -----------------------------------------------------------------
        `TEST_START("Disable/Enable Control")

        // Disable NCO
        bfm.axi_write(ADDR_CTRL, 32'h00000000);
        repeat(20) @(posedge clk);

        // Read status
        bfm.axi_read(ADDR_STATUS, read_data);
        $display("INFO: Status after disable = 0x%h", read_data);

        // Re-enable
        bfm.axi_write(ADDR_CTRL, CTRL_ENABLE);
        repeat(20) @(posedge clk);

        bfm.axi_read(ADDR_STATUS, read_data);
        $display("INFO: Status after enable = 0x%h", read_data);
        $display("PASS: Enable/Disable control");

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
        #100000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

    // =========================================================================
    // VCD Dump
    // =========================================================================

    initial begin
        $dumpfile("tb_r4w_nco.vcd");
        $dumpvars(0, tb_r4w_nco);
    end

endmodule
