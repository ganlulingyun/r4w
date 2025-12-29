//-----------------------------------------------------------------------------
// R4W Top Level for Lattice ECP5
// R4W FPGA Acceleration Layer
//
// Target: Lattice ECP5-25K (e.g., ULX3S, OrangeCrab)
// Toolchain: Yosys + nextpnr-ecp5 + Trellis
//
// Provides:
//   - LoRa chirp generation via SPI
//   - NCO for frequency synthesis (CORDIC-based)
//   - Higher clock speeds (up to 100MHz+)
//   - More resources for additional features
//
// trace:FR-0085 | ai:claude
//-----------------------------------------------------------------------------

module r4w_top_ecp5 (
    // Clock (25 MHz typical, PLL to 100 MHz)
    input  wire clk_25mhz,

    // SPI interface
    input  wire spi_clk,
    input  wire spi_cs_n,
    input  wire spi_mosi,
    output wire spi_miso,

    // I/Q output (directly to DAC or as GPIO)
    output wire [11:0] dac_i,
    output wire [11:0] dac_q,
    output wire dac_valid,

    // Status LEDs
    output wire [7:0] led,

    // Optional: SDRAM interface (directly to SDRAM on many ECP5 boards)
    // For future sample buffer support
    output wire sdram_clk,
    output wire sdram_cke,
    output wire sdram_cs_n,
    output wire sdram_we_n,
    output wire sdram_ras_n,
    output wire sdram_cas_n,
    output wire [1:0] sdram_ba,
    output wire [12:0] sdram_a,
    inout  wire [15:0] sdram_dq,
    output wire [1:0] sdram_dqm
);

    // =========================================================================
    // PLL Configuration - 25 MHz to 100 MHz
    // =========================================================================

    wire clk;
    wire pll_locked;

    // ECP5 PLL primitive
    (* FREQUENCY_PIN_CLKI = "25" *)
    (* FREQUENCY_PIN_CLKOP = "100" *)
    EHXPLLL #(
        .PLLRST_ENA("DISABLED"),
        .INTFB_WAKE("DISABLED"),
        .STDBY_ENABLE("DISABLED"),
        .DPHASE_SOURCE("DISABLED"),
        .OUTDIVIDER_MUXA("DIVA"),
        .OUTDIVIDER_MUXB("DIVB"),
        .OUTDIVIDER_MUXC("DIVC"),
        .OUTDIVIDER_MUXD("DIVD"),
        .CLKI_DIV(1),           // Input divider
        .CLKOP_ENABLE("ENABLED"),
        .CLKOP_DIV(6),          // Output divider for CLKOP
        .CLKOP_CPHASE(5),
        .CLKOP_FPHASE(0),
        .FEEDBK_PATH("CLKOP"),
        .CLKFB_DIV(4)           // Feedback divider: 25 * 4 / 1 = 100 MHz
    ) pll_inst (
        .RST(1'b0),
        .STDBY(1'b0),
        .CLKI(clk_25mhz),
        .CLKOP(clk),
        .CLKFB(clk),
        .CLKINTFB(),
        .PHASESEL0(1'b0),
        .PHASESEL1(1'b0),
        .PHASEDIR(1'b1),
        .PHASESTEP(1'b1),
        .PHASELOADREG(1'b1),
        .PLLWAKESYNC(1'b0),
        .ENCLKOP(1'b0),
        .LOCK(pll_locked)
    );

    // Reset generation
    reg [15:0] reset_counter = 16'hFFFF;
    wire rst_n = (reset_counter == 0) && pll_locked;

    always @(posedge clk or negedge pll_locked) begin
        if (!pll_locked) begin
            reset_counter <= 16'hFFFF;
        end else if (reset_counter != 0) begin
            reset_counter <= reset_counter - 1;
        end
    end

    // =========================================================================
    // Register Map (same as iCE40 for compatibility)
    // =========================================================================
    // Extended addresses for ECP5-specific features

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

    // ECP5-specific extended registers
    localparam ADDR_PLL_CFG   = 16'h0100;  // PLL configuration
    localparam ADDR_DMA_CTRL  = 16'h0200;  // Future DMA control
    localparam ADDR_SDRAM_CFG = 16'h0300;  // Future SDRAM config

    localparam IP_ID      = 32'h52344C45;  // "R4LE" = R4W Lattice ECP5
    localparam IP_VERSION = 32'h00010000;

    // =========================================================================
    // SPI Slave Interface
    // =========================================================================

    wire [15:0] reg_addr;
    wire [31:0] reg_wdata;
    wire        reg_wen;
    wire        reg_ren;
    reg  [31:0] reg_rdata;

    r4w_spi_slave #(
        .ADDR_WIDTH(16),
        .DATA_WIDTH(32)
    ) spi_slave (
        .clk(clk),
        .rst_n(rst_n),
        .spi_clk(spi_clk),
        .spi_cs_n(spi_cs_n),
        .spi_mosi(spi_mosi),
        .spi_miso(spi_miso),
        .reg_addr(reg_addr),
        .reg_wdata(reg_wdata),
        .reg_wen(reg_wen),
        .reg_ren(reg_ren),
        .reg_rdata(reg_rdata)
    );

    // =========================================================================
    // Control Registers
    // =========================================================================

    reg [31:0] ctrl_reg;
    reg [3:0]  sf_reg;
    reg [11:0] symbol_reg;
    reg [23:0] nco_freq_reg;
    reg [23:0] nco_phase_reg;

    wire start_chirp = ctrl_reg[0];
    wire upchirp     = ctrl_reg[1];
    wire nco_enable  = ctrl_reg[4];
    wire soft_reset  = ctrl_reg[7];

    // =========================================================================
    // Chirp Generator
    // =========================================================================

    wire chirp_busy, chirp_done, chirp_valid;
    wire signed [11:0] chirp_i, chirp_q;

    r4w_chirp_gen #(
        .PHASE_WIDTH(24),
        .OUTPUT_WIDTH(12),
        .MAX_SF(12)
    ) chirp_gen (
        .clk(clk),
        .rst_n(rst_n && !soft_reset),
        .start(start_chirp),
        .upchirp(upchirp),
        .sf(sf_reg),
        .symbol(symbol_reg),
        .busy(chirp_busy),
        .done(chirp_done),
        .i_out(chirp_i),
        .q_out(chirp_q),
        .valid(chirp_valid)
    );

    // =========================================================================
    // Standalone NCO (CORDIC-based for ECP5)
    // =========================================================================

    wire signed [11:0] nco_i, nco_q;

    r4w_nco #(
        .PHASE_WIDTH(24),
        .OUTPUT_WIDTH(12),
        .USE_LUT(0)  // Use CORDIC for ECP5 (more logic, no BRAM)
    ) nco (
        .clk(clk),
        .rst_n(rst_n && !soft_reset),
        .enable(nco_enable),
        .freq_word(nco_freq_reg),
        .phase_offset(nco_phase_reg),
        .i_out(nco_i),
        .q_out(nco_q)
    );

    // =========================================================================
    // Output Selection
    // =========================================================================

    wire signed [11:0] out_i = chirp_busy ? chirp_i : nco_i;
    wire signed [11:0] out_q = chirp_busy ? chirp_q : nco_q;

    // Convert signed to unsigned for DAC
    assign dac_i = out_i + 12'h800;
    assign dac_q = out_q + 12'h800;
    assign dac_valid = chirp_valid || nco_enable;

    // Latch output for readback
    reg [11:0] data_i_latch, data_q_latch;
    always @(posedge clk) begin
        if (chirp_valid || nco_enable) begin
            data_i_latch <= out_i;
            data_q_latch <= out_q;
        end
    end

    // =========================================================================
    // Register Write Logic
    // =========================================================================

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ctrl_reg      <= 32'h0;
            sf_reg        <= 4'd7;
            symbol_reg    <= 12'd0;
            nco_freq_reg  <= 24'h004000; // ~1kHz at 100MHz
            nco_phase_reg <= 24'h0;
        end else begin
            // Auto-clear start bit
            if (ctrl_reg[0])
                ctrl_reg[0] <= 1'b0;

            // Auto-clear soft reset
            if (ctrl_reg[7])
                ctrl_reg[7] <= 1'b0;

            if (reg_wen) begin
                case (reg_addr)
                    ADDR_CTRL:      ctrl_reg <= reg_wdata;
                    ADDR_SF:        sf_reg <= reg_wdata[3:0];
                    ADDR_SYMBOL:    symbol_reg <= reg_wdata[11:0];
                    ADDR_NCO_FREQ:  nco_freq_reg <= reg_wdata[23:0];
                    ADDR_NCO_PHASE: nco_phase_reg <= reg_wdata[23:0];
                endcase
            end
        end
    end

    // =========================================================================
    // Register Read Logic
    // =========================================================================

    always @(*) begin
        case (reg_addr)
            ADDR_CTRL:      reg_rdata = ctrl_reg;
            ADDR_SF:        reg_rdata = {28'h0, sf_reg};
            ADDR_STATUS:    reg_rdata = {23'h0, pll_locked, 6'h0, chirp_busy, chirp_done};
            ADDR_SYMBOL:    reg_rdata = {20'h0, symbol_reg};
            ADDR_NCO_FREQ:  reg_rdata = {8'h0, nco_freq_reg};
            ADDR_NCO_PHASE: reg_rdata = {8'h0, nco_phase_reg};
            ADDR_DATA_I:    reg_rdata = {{20{data_i_latch[11]}}, data_i_latch};
            ADDR_DATA_Q:    reg_rdata = {{20{data_q_latch[11]}}, data_q_latch};
            ADDR_ID:        reg_rdata = IP_ID;
            ADDR_VERSION:   reg_rdata = IP_VERSION;
            default:        reg_rdata = 32'h0;
        endcase
    end

    // =========================================================================
    // Status LEDs
    // =========================================================================

    // LED mapping for ULX3S style board
    assign led[0] = pll_locked;
    assign led[1] = chirp_busy;
    assign led[2] = chirp_done;
    assign led[3] = nco_enable;
    assign led[4] = !spi_cs_n;  // SPI activity
    assign led[7:5] = sf_reg[2:0];  // Show SF

    // =========================================================================
    // SDRAM - Directly controlled, no interface for now (directly reserved)
    // =========================================================================

    // SDRAM directly unused - directly drive inactive states
    assign sdram_clk = 1'b0;
    assign sdram_cke = 1'b0;
    assign sdram_cs_n = 1'b1;
    assign sdram_we_n = 1'b1;
    assign sdram_ras_n = 1'b1;
    assign sdram_cas_n = 1'b1;
    assign sdram_ba = 2'b00;
    assign sdram_a = 13'b0;
    assign sdram_dqm = 2'b11;
    // sdram_dq is high-Z (inout, not driven)

endmodule
