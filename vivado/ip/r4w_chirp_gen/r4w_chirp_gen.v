//-----------------------------------------------------------------------------
// R4W LoRa Chirp Generator
// R4W FPGA Acceleration Layer
//
// Generates LoRa CSS (Chirp Spread Spectrum) signals.
// Chirp frequency linearly increases (upchirp) or decreases (downchirp)
// over the symbol period.
//
// LoRa chirp characteristics:
// - N = 2^SF samples per symbol (SF = spreading factor 5-12)
// - Frequency sweeps from 0 to BW over symbol period
// - Phase: φ(n) = 2π × (n²/(2N) + sym×n/N) for upchirp
//
// trace:FR-0031 | ai:claude
//-----------------------------------------------------------------------------

module r4w_chirp_gen #(
    parameter PHASE_WIDTH = 32,
    parameter OUTPUT_WIDTH = 16,
    parameter MAX_SF = 12,
    parameter CORDIC_STAGES = 16
)(
    input  wire                           clk,
    input  wire                           rst_n,

    // Control
    input  wire                           start,
    input  wire                           upchirp,      // 1=upchirp, 0=downchirp
    input  wire                           continuous,   // Continuous generation mode
    input  wire [3:0]                     sf,           // Spreading factor (5-12)
    input  wire [1:0]                     bw_sel,       // 0=125k, 1=250k, 2=500k
    input  wire [11:0]                    symbol,       // Symbol value (0 to 2^SF-1)

    // Output
    output reg  signed [OUTPUT_WIDTH-1:0] i_out,
    output reg  signed [OUTPUT_WIDTH-1:0] q_out,
    output reg                            out_valid,
    output reg                            done,
    output reg                            busy
);

    // =========================================================================
    // State Machine
    // =========================================================================

    localparam S_IDLE     = 2'd0;
    localparam S_GENERATE = 2'd1;
    localparam S_DONE     = 2'd2;

    reg [1:0] state;

    // =========================================================================
    // Sample Counter
    // =========================================================================

    reg [12:0] sample_count;      // Up to 4096 samples (SF=12)
    wire [12:0] samples_per_symbol;

    // N = 2^SF samples per symbol
    assign samples_per_symbol = (13'd1 << sf);

    // =========================================================================
    // Phase Accumulator for Chirp
    // =========================================================================

    // For LoRa chirp, instantaneous phase is:
    // φ(n) = 2π × [n²/(2N) + sym×n/N]
    //
    // We implement this incrementally:
    // freq(n) = n/N + sym/N   (normalized frequency)
    // phase(n+1) = phase(n) + freq(n) × 2π
    //
    // Using fixed-point: phase is 32-bit with 2π = 2^32

    reg [PHASE_WIDTH-1:0] phase_acc;
    reg [PHASE_WIDTH-1:0] freq_word;       // Current frequency word
    reg [PHASE_WIDTH-1:0] freq_increment;  // Frequency increment per sample

    // Calculate frequency word based on SF and symbol
    // freq_word = (sample_count + symbol) / N, scaled to phase units
    // freq_increment = 1/N per sample (in phase units)
    //
    // At 100 MHz clock with BW=125kHz:
    // - Sample rate = 1 MHz (decimation by 100)
    // - freq_increment = BW / (fs × N) × 2^32
    //
    // We'll compute freq_increment based on SF only (assumes normalized BW)
    // freq_increment = 2^32 / (2^SF) = 2^(32-SF)

    wire [PHASE_WIDTH-1:0] base_freq_increment;
    assign base_freq_increment = (32'h1 << (32 - sf));  // 2^(32-SF)

    // Symbol offset adds constant phase increment
    wire [PHASE_WIDTH-1:0] symbol_phase_inc;
    assign symbol_phase_inc = {20'h0, symbol} * base_freq_increment;

    // =========================================================================
    // CORDIC Sine/Cosine Generator
    // =========================================================================

    // CORDIC gain constant
    localparam signed [15:0] CORDIC_GAIN = 16'h4DBA;  // 0.6073 × 2^15

    // Arctan lookup table
    wire [31:0] atan_lut [0:15];
    assign atan_lut[0]  = 32'h20000000;
    assign atan_lut[1]  = 32'h12E4051E;
    assign atan_lut[2]  = 32'h09FB385B;
    assign atan_lut[3]  = 32'h051111D4;
    assign atan_lut[4]  = 32'h028B0D43;
    assign atan_lut[5]  = 32'h0145D7E1;
    assign atan_lut[6]  = 32'h00A2F61E;
    assign atan_lut[7]  = 32'h00517C55;
    assign atan_lut[8]  = 32'h0028BE53;
    assign atan_lut[9]  = 32'h00145F2F;
    assign atan_lut[10] = 32'h000A2F98;
    assign atan_lut[11] = 32'h000517CC;
    assign atan_lut[12] = 32'h00028BE6;
    assign atan_lut[13] = 32'h000145F3;
    assign atan_lut[14] = 32'h0000A2FA;
    assign atan_lut[15] = 32'h0000517D;

    // CORDIC pipeline registers
    reg signed [17:0] x [0:CORDIC_STAGES];
    reg signed [17:0] y [0:CORDIC_STAGES];
    reg signed [31:0] z [0:CORDIC_STAGES];
    reg [1:0] quad_pipe [0:CORDIC_STAGES];
    reg valid_pipe [0:CORDIC_STAGES];

    // Quadrant handling
    wire [1:0] quadrant = phase_acc[31:30];
    reg [31:0] normalized_phase;

    always @(*) begin
        case (quadrant)
            2'b00: normalized_phase = phase_acc;
            2'b01: normalized_phase = 32'h40000000 - phase_acc;
            2'b10: normalized_phase = phase_acc - 32'h80000000;
            2'b11: normalized_phase = 32'hC0000000 - phase_acc;
        endcase
    end

    // =========================================================================
    // State Machine Logic
    // =========================================================================

    reg generate_enable;

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            sample_count <= 0;
            phase_acc <= 0;
            freq_word <= 0;
            busy <= 0;
            done <= 0;
            generate_enable <= 0;
        end else begin
            done <= 1'b0;  // Single cycle pulse

            case (state)
                S_IDLE: begin
                    if (start) begin
                        state <= S_GENERATE;
                        sample_count <= 0;
                        phase_acc <= 0;
                        freq_word <= symbol_phase_inc;  // Start with symbol offset
                        busy <= 1'b1;
                        generate_enable <= 1'b1;
                    end
                end

                S_GENERATE: begin
                    generate_enable <= 1'b1;

                    // Update frequency (chirp rate)
                    if (upchirp) begin
                        freq_word <= freq_word + base_freq_increment;
                    end else begin
                        freq_word <= freq_word - base_freq_increment;
                    end

                    // Update phase
                    phase_acc <= phase_acc + freq_word;

                    // Increment sample counter
                    sample_count <= sample_count + 1;

                    // Check if symbol complete
                    if (sample_count >= samples_per_symbol - 1) begin
                        if (continuous) begin
                            // Reset for next symbol
                            sample_count <= 0;
                            freq_word <= symbol_phase_inc;
                        end else begin
                            state <= S_DONE;
                            generate_enable <= 1'b0;
                        end
                    end
                end

                S_DONE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // CORDIC Pipeline Stage 0
    // =========================================================================

    always @(posedge clk) begin
        if (!rst_n) begin
            x[0] <= 0;
            y[0] <= 0;
            z[0] <= 0;
            valid_pipe[0] <= 0;
            quad_pipe[0] <= 0;
        end else begin
            valid_pipe[0] <= generate_enable;
            quad_pipe[0] <= quadrant;
            if (generate_enable) begin
                x[0] <= 18'sh10000;
                y[0] <= 18'sh00000;
                z[0] <= normalized_phase;
            end
        end
    end

    // =========================================================================
    // CORDIC Iteration Pipeline
    // =========================================================================

    genvar i;
    generate
        for (i = 0; i < CORDIC_STAGES; i = i + 1) begin : cordic_stage
            wire signed [17:0] x_shift = x[i] >>> i;
            wire signed [17:0] y_shift = y[i] >>> i;
            wire rotate_ccw = z[i][31];

            always @(posedge clk) begin
                if (!rst_n) begin
                    x[i+1] <= 0;
                    y[i+1] <= 0;
                    z[i+1] <= 0;
                    valid_pipe[i+1] <= 0;
                    quad_pipe[i+1] <= 0;
                end else begin
                    valid_pipe[i+1] <= valid_pipe[i];
                    quad_pipe[i+1] <= quad_pipe[i];

                    if (valid_pipe[i]) begin
                        if (rotate_ccw) begin
                            x[i+1] <= x[i] + y_shift;
                            y[i+1] <= y[i] - x_shift;
                            z[i+1] <= z[i] + atan_lut[i];
                        end else begin
                            x[i+1] <= x[i] - y_shift;
                            y[i+1] <= y[i] + x_shift;
                            z[i+1] <= z[i] - atan_lut[i];
                        end
                    end
                end
            end
        end
    endgenerate

    // =========================================================================
    // Output Stage
    // =========================================================================

    wire signed [17:0] cos_raw = x[CORDIC_STAGES];
    wire signed [17:0] sin_raw = y[CORDIC_STAGES];
    wire [1:0] final_quadrant = quad_pipe[CORDIC_STAGES];

    wire signed [35:0] cos_scaled = (cos_raw * CORDIC_GAIN) >>> 15;
    wire signed [35:0] sin_scaled = (sin_raw * CORDIC_GAIN) >>> 15;

    always @(posedge clk) begin
        if (!rst_n) begin
            i_out <= 0;
            q_out <= 0;
            out_valid <= 0;
        end else begin
            out_valid <= valid_pipe[CORDIC_STAGES];

            if (valid_pipe[CORDIC_STAGES]) begin
                case (final_quadrant)
                    2'b00: begin
                        i_out <= cos_scaled[17:2];
                        q_out <= sin_scaled[17:2];
                    end
                    2'b01: begin
                        i_out <= -cos_scaled[17:2];
                        q_out <= sin_scaled[17:2];
                    end
                    2'b10: begin
                        i_out <= -cos_scaled[17:2];
                        q_out <= -sin_scaled[17:2];
                    end
                    2'b11: begin
                        i_out <= cos_scaled[17:2];
                        q_out <= -sin_scaled[17:2];
                    end
                endcase
            end
        end
    end

endmodule
