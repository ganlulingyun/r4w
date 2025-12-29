//-----------------------------------------------------------------------------
// R4W LoRa Chirp Generator for Lattice FPGAs
// R4W FPGA Acceleration Layer
//
// Generates LoRa chirp waveforms for SF5-SF12.
// Optimized for small Lattice FPGAs (iCE40 HX8K has ~7680 LUTs).
//
// Compatible with: iCE40, ECP5
// Toolchain: Yosys + IceStorm/Trellis
//
// trace:FR-0082 | ai:claude
//-----------------------------------------------------------------------------

module r4w_chirp_gen #(
    parameter PHASE_WIDTH = 24,
    parameter OUTPUT_WIDTH = 12,
    parameter MAX_SF = 12           // Maximum spreading factor
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Control
    input  wire                     start,
    input  wire                     upchirp,    // 1=up, 0=down
    input  wire [3:0]               sf,         // Spreading factor (5-12)
    input  wire [MAX_SF-1:0]        symbol,     // Symbol value (0 to 2^SF-1)

    // Status
    output reg                      busy,
    output reg                      done,

    // Outputs (signed)
    output wire signed [OUTPUT_WIDTH-1:0] i_out,
    output wire signed [OUTPUT_WIDTH-1:0] q_out,
    output reg                      valid
);

    // =========================================================================
    // Sample Counter
    // =========================================================================

    // Number of samples = 2^SF
    wire [MAX_SF-1:0] num_samples = (1 << sf);
    reg  [MAX_SF-1:0] sample_count;

    // =========================================================================
    // Phase Accumulator for Chirp
    // =========================================================================

    // Chirp phase: φ(n) = 2π × (n² / 2N + symbol × n / N)
    // We implement this incrementally:
    //   freq(n) = dφ/dn = 2π × (n/N + symbol/N)
    //   freq increments by 2π/N per sample (linear frequency sweep)

    // For efficient implementation:
    //   freq_word = base_freq + chirp_rate × n
    // where:
    //   base_freq = symbol × (2^PHASE_WIDTH / N)
    //   chirp_rate = 2^PHASE_WIDTH / N (frequency increment per sample)

    reg [PHASE_WIDTH-1:0] phase_acc;
    reg [PHASE_WIDTH-1:0] freq_word;
    reg [PHASE_WIDTH-1:0] freq_increment;
    reg [PHASE_WIDTH-1:0] base_freq;

    // Calculate frequency parameters based on SF
    // freq_increment = 2^PHASE_WIDTH / 2^SF = 2^(PHASE_WIDTH - SF)
    wire [PHASE_WIDTH-1:0] chirp_rate;
    assign chirp_rate = (1 << (PHASE_WIDTH - sf));

    // =========================================================================
    // State Machine
    // =========================================================================

    localparam STATE_IDLE  = 2'd0;
    localparam STATE_INIT  = 2'd1;
    localparam STATE_GEN   = 2'd2;
    localparam STATE_DONE  = 2'd3;

    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= STATE_IDLE;
            sample_count   <= 0;
            phase_acc      <= 0;
            freq_word      <= 0;
            freq_increment <= 0;
            base_freq      <= 0;
            busy           <= 0;
            done           <= 0;
            valid          <= 0;
        end else begin
            done  <= 1'b0;
            valid <= 1'b0;

            case (state)
                STATE_IDLE: begin
                    if (start) begin
                        state <= STATE_INIT;
                        busy  <= 1'b1;
                    end
                end

                STATE_INIT: begin
                    // Calculate initial frequency based on symbol
                    // base_freq = symbol × chirp_rate
                    base_freq <= symbol * chirp_rate;

                    // For upchirp: frequency increases
                    // For downchirp: frequency decreases (start at high freq)
                    if (upchirp) begin
                        freq_word <= symbol * chirp_rate;
                        freq_increment <= chirp_rate;
                    end else begin
                        // Downchirp: start at max frequency, decrease
                        freq_word <= ((num_samples - 1 - symbol) * chirp_rate);
                        freq_increment <= -chirp_rate; // Negative increment
                    end

                    phase_acc    <= 0;
                    sample_count <= 0;
                    state        <= STATE_GEN;
                end

                STATE_GEN: begin
                    // Generate one sample per clock
                    valid <= 1'b1;

                    // Update phase accumulator
                    phase_acc <= phase_acc + freq_word;

                    // Update frequency (linear chirp)
                    freq_word <= freq_word + freq_increment;

                    // Wrap frequency at N (implement cyclic shift for symbol)
                    // This is automatic due to modular arithmetic

                    sample_count <= sample_count + 1;

                    if (sample_count >= num_samples - 1) begin
                        state <= STATE_DONE;
                    end
                end

                STATE_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= STATE_IDLE;
                end
            endcase
        end
    end

    // =========================================================================
    // NCO for I/Q Generation
    // =========================================================================

    // Use the Lattice-optimized NCO
    r4w_nco #(
        .PHASE_WIDTH(PHASE_WIDTH),
        .OUTPUT_WIDTH(OUTPUT_WIDTH),
        .USE_LUT(1)  // Use LUT for iCE40 compatibility
    ) nco_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(state == STATE_GEN),
        .freq_word({PHASE_WIDTH{1'b0}}),  // Frequency handled by phase_acc
        .phase_offset(phase_acc),
        .i_out(i_out),
        .q_out(q_out)
    );

endmodule
