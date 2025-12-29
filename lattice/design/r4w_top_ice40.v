//-----------------------------------------------------------------------------
// R4W Top Level for iCE40 HX8K
// R4W FPGA Acceleration Layer
//
// Target: Lattice iCE40-HX8K (e.g., iCEstick, TinyFPGA BX)
// Toolchain: Yosys + nextpnr-ice40 + IceStorm
//
// Provides:
//   - LoRa chirp generation via SPI
//   - NCO for frequency synthesis
//   - Register access via SPI slave interface
//
// trace:FR-0083 | ai:claude
//-----------------------------------------------------------------------------

module r4w_top_ice40 (
    // Clock (12 MHz typical for many iCE40 boards)
    input  wire clk_12mhz,

    // SPI interface (directly to FTDI or other SPI master)
    input  wire spi_clk,
    input  wire spi_cs_n,
    input  wire spi_mosi,
    output wire spi_miso,

    // I/Q output (directly to DAC or as GPIO for debugging)
    output wire [11:0] dac_i,
    output wire [11:0] dac_q,
    output wire dac_valid,

    // Status LEDs
    output wire led_busy,
    output wire led_done
);

    // =========================================================================
    // PLL for higher internal clock (optional)
    // =========================================================================

    // For iCE40, we can use the internal PLL to multiply clock
    // 12 MHz * 4 = 48 MHz is a common choice
    wire clk;
    wire pll_locked;

    // Simple bypass for now (no PLL - use external clock directly)
    assign clk = clk_12mhz;
    assign pll_locked = 1'b1;

    // Reset generation (simple power-on reset)
    reg [7:0] reset_counter = 8'hFF;
    wire rst_n = (reset_counter == 0) && pll_locked;

    always @(posedge clk) begin
        if (reset_counter != 0)
            reset_counter <= reset_counter - 1;
    end

    // =========================================================================
    // Register Map
    // =========================================================================
    // 0x0000: CTRL      [0]=START_CHIRP, [1]=UPCHIRP, [4]=NCO_ENABLE, [7]=SOFT_RESET
    // 0x0004: SF        Spreading factor (5-12)
    // 0x0008: STATUS    [0]=CHIRP_DONE, [1]=CHIRP_BUSY, [8]=PLL_LOCKED
    // 0x000C: SYMBOL    Symbol value for chirp
    // 0x0010: NCO_FREQ  NCO frequency word
    // 0x0014: NCO_PHASE NCO phase offset
    // 0x0018: DATA_I    Last I sample (read-only)
    // 0x001C: DATA_Q    Last Q sample (read-only)
    // 0x0020: ID        IP core ID = 0x52344C49 ("R4LI" = R4W Lattice iCE40)
    // 0x0024: VERSION   Version = 0x00010000

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

    localparam IP_ID      = 32'h52344C49;  // "R4LI"
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
    // Standalone NCO
    // =========================================================================

    wire signed [11:0] nco_i, nco_q;

    r4w_nco #(
        .PHASE_WIDTH(24),
        .OUTPUT_WIDTH(12),
        .USE_LUT(1)
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

    // Use chirp output when busy, else NCO
    wire signed [11:0] out_i = chirp_busy ? chirp_i : nco_i;
    wire signed [11:0] out_q = chirp_busy ? chirp_q : nco_q;

    // Convert signed to unsigned for DAC (add offset)
    assign dac_i = out_i + 12'h800;
    assign dac_q = out_q + 12'h800;
    assign dac_valid = chirp_valid || nco_enable;

    // Latch last output for readback
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
            sf_reg        <= 4'd7;  // Default SF7
            symbol_reg    <= 12'd0;
            nco_freq_reg  <= 24'h010000; // ~1kHz at 48MHz
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

    assign led_busy = chirp_busy;
    assign led_done = chirp_done;

endmodule
