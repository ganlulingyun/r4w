//-----------------------------------------------------------------------------
// R4W iCE40 Top-Level Testbench
// R4W FPGA Acceleration Layer
//
// Tests SPI register access, chirp generation, and NCO operation
//
// trace:FR-0085 | ai:claude
//-----------------------------------------------------------------------------

`timescale 1ns / 1ps

module tb_r4w_top_ice40;

    // =========================================================================
    // Parameters
    // =========================================================================

    localparam CLK_PERIOD = 20.833;  // 48 MHz
    localparam SPI_PERIOD = 200;     // 5 MHz SPI clock (slower for synchronizer)

    // Register addresses
    localparam ADDR_CTRL      = 16'h0000;
    localparam ADDR_SF        = 16'h0004;
    localparam ADDR_STATUS    = 16'h0008;
    localparam ADDR_SYMBOL    = 16'h000C;
    localparam ADDR_NCO_FREQ  = 16'h0010;
    localparam ADDR_NCO_PHASE = 16'h0014;
    localparam ADDR_DATA_I    = 16'h0018;
    localparam ADDR_DATA_Q    = 16'h001C;
    localparam ADDR_ID        = 16'h0020;
    localparam ADDR_VERSION   = 16'h0024;

    // =========================================================================
    // Signals
    // =========================================================================

    reg clk;
    reg spi_clk;
    reg spi_cs_n;
    reg spi_mosi;
    wire spi_miso;

    wire [11:0] dac_i;
    wire [11:0] dac_q;
    wire dac_valid;
    wire led_busy;
    wire led_done;

    // SPI transaction data
    reg [31:0] spi_rdata;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================

    r4w_top_ice40 dut (
        .clk_12mhz(clk),
        .spi_clk(spi_clk),
        .spi_cs_n(spi_cs_n),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .dac_i(dac_i),
        .dac_q(dac_q),
        .dac_valid(dac_valid),
        .led_busy(led_busy),
        .led_done(led_done)
    );

    // =========================================================================
    // Clock Generation
    // =========================================================================

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // SPI Tasks
    // =========================================================================

    // SPI byte transfer (MSB first)
    task spi_byte;
        input [7:0] tx_data;
        output [7:0] rx_data;
        integer i;
        begin
            rx_data = 8'h00;
            for (i = 7; i >= 0; i = i - 1) begin
                spi_mosi = tx_data[i];
                #(SPI_PERIOD/2);
                spi_clk = 1;
                rx_data[i] = spi_miso;
                `ifdef DEBUG_SPI
                $display("  bit %0d: mosi=%b miso=%b", i, tx_data[i], spi_miso);
                `endif
                #(SPI_PERIOD/2);
                spi_clk = 0;
            end
            `ifdef DEBUG_SPI
            $display("  byte: tx=0x%02X rx=0x%02X", tx_data, rx_data);
            `endif
        end
    endtask

    // SPI write transaction (write=1, addr, data)
    task spi_write;
        input [15:0] addr;
        input [31:0] data;
        reg [7:0] dummy;
        begin
            spi_cs_n = 0;
            #(SPI_PERIOD);

            // Command byte: bit7=1 for write
            spi_byte(8'h80, dummy);

            // Address (16-bit, MSB first)
            spi_byte(addr[15:8], dummy);
            spi_byte(addr[7:0], dummy);

            // Data (32-bit, MSB first)
            spi_byte(data[31:24], dummy);
            spi_byte(data[23:16], dummy);
            spi_byte(data[15:8], dummy);
            spi_byte(data[7:0], dummy);

            #(SPI_PERIOD);
            spi_cs_n = 1;
            #(SPI_PERIOD * 2);
        end
    endtask

    // SPI read transaction (write=0, addr, read data)
    task spi_read;
        input [15:0] addr;
        output [31:0] data;
        reg [7:0] rx_byte;
        begin
            spi_cs_n = 0;
            #(SPI_PERIOD);

            // Command byte: bit7=0 for read
            spi_byte(8'h00, rx_byte);

            // Address (16-bit, MSB first)
            spi_byte(addr[15:8], rx_byte);
            spi_byte(addr[7:0], rx_byte);

            // Read data (32-bit, MSB first)
            spi_byte(8'h00, rx_byte);
            data[31:24] = rx_byte;
            spi_byte(8'h00, rx_byte);
            data[23:16] = rx_byte;
            spi_byte(8'h00, rx_byte);
            data[15:8] = rx_byte;
            spi_byte(8'h00, rx_byte);
            data[7:0] = rx_byte;

            #(SPI_PERIOD);
            spi_cs_n = 1;
            #(SPI_PERIOD * 2);
        end
    endtask

    // =========================================================================
    // Test Sequence
    // =========================================================================

    initial begin
        // Setup VCD dumping
        $dumpfile("tb_r4w_top_ice40.vcd");
        $dumpvars(0, tb_r4w_top_ice40);

        // Initialize
        spi_clk = 0;
        spi_cs_n = 1;
        spi_mosi = 0;

        $display("=== R4W iCE40 Testbench ===");
        $display("Time: %0t - Starting tests", $time);

        // Wait for reset (256 clocks @ 48MHz = ~5.3us, use 10us to be safe)
        #10000;

        // =====================================================================
        // Test 1: Read IP ID and Version
        // =====================================================================
        $display("\n--- Test 1: Read IP ID and Version ---");

        spi_read(ADDR_ID, spi_rdata);
        $display("IP ID: 0x%08X (expected 0x52344C49 = 'R4LI')", spi_rdata);
        if (spi_rdata !== 32'h52344C49) begin
            $display("ERROR: IP ID mismatch!");
            $finish;
        end

        spi_read(ADDR_VERSION, spi_rdata);
        $display("Version: 0x%08X", spi_rdata);

        // =====================================================================
        // Test 2: Write and Read Registers
        // =====================================================================
        $display("\n--- Test 2: Register Write/Read ---");

        // Set SF to 8
        spi_write(ADDR_SF, 32'h00000008);
        spi_read(ADDR_SF, spi_rdata);
        $display("SF Register: %0d (expected 8)", spi_rdata);
        if (spi_rdata !== 32'h00000008) begin
            $display("ERROR: SF register mismatch!");
            $finish;
        end

        // Set symbol to 42
        spi_write(ADDR_SYMBOL, 32'h0000002A);
        spi_read(ADDR_SYMBOL, spi_rdata);
        $display("Symbol Register: %0d (expected 42)", spi_rdata);

        // =====================================================================
        // Test 3: NCO Operation
        // =====================================================================
        $display("\n--- Test 3: NCO Operation ---");

        // Set NCO frequency
        spi_write(ADDR_NCO_FREQ, 32'h00010000);  // Higher frequency

        // Enable NCO
        spi_write(ADDR_CTRL, 32'h00000010);  // Bit 4 = NCO enable

        // Wait for some NCO output
        #10000;

        // Read I/Q data
        spi_read(ADDR_DATA_I, spi_rdata);
        $display("NCO I Data: 0x%08X (%0d signed)", spi_rdata, $signed(spi_rdata[11:0]));

        spi_read(ADDR_DATA_Q, spi_rdata);
        $display("NCO Q Data: 0x%08X (%0d signed)", spi_rdata, $signed(spi_rdata[11:0]));

        // Disable NCO
        spi_write(ADDR_CTRL, 32'h00000000);

        // =====================================================================
        // Test 4: Chirp Generation
        // =====================================================================
        $display("\n--- Test 4: Chirp Generation ---");

        // Configure for SF7
        spi_write(ADDR_SF, 32'h00000007);
        spi_write(ADDR_SYMBOL, 32'h00000000);  // Symbol 0

        // Start upchirp
        spi_write(ADDR_CTRL, 32'h00000003);  // Start + upchirp

        // Check status
        #100;
        spi_read(ADDR_STATUS, spi_rdata);
        $display("Status (during chirp): 0x%08X, busy=%0d", spi_rdata, spi_rdata[1]);

        // Wait for chirp to complete (2^7 = 128 samples at 48MHz)
        #50000;

        spi_read(ADDR_STATUS, spi_rdata);
        $display("Status (after chirp): 0x%08X, done=%0d", spi_rdata, spi_rdata[0]);

        // =====================================================================
        // Test 5: Soft Reset
        // =====================================================================
        $display("\n--- Test 5: Soft Reset ---");

        spi_write(ADDR_CTRL, 32'h00000080);  // Bit 7 = soft reset
        #100;

        spi_read(ADDR_SF, spi_rdata);
        $display("SF after reset: %0d (expected 7 - default)", spi_rdata);

        // =====================================================================
        // Done
        // =====================================================================
        $display("\n=== All Tests Passed ===");
        $display("Time: %0t - Simulation complete", $time);

        #1000;
        $finish;
    end

    // =========================================================================
    // Timeout
    // =========================================================================

    initial begin
        #1000000;
        $display("ERROR: Simulation timeout!");
        $finish;
    end

endmodule
