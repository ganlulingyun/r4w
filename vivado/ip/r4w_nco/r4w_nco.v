//-----------------------------------------------------------------------------
// R4W NCO (Numerically Controlled Oscillator)
// R4W FPGA Acceleration Layer
//
// CORDIC-based NCO generating I/Q samples at specified frequency.
// Uses iterative CORDIC algorithm for sin/cos computation.
//
// trace:FR-0030 | ai:claude
//-----------------------------------------------------------------------------

module r4w_nco #(
    parameter PHASE_WIDTH = 32,       // Phase accumulator width
    parameter OUTPUT_WIDTH = 16,      // Output sample width
    parameter CORDIC_STAGES = 16      // CORDIC pipeline stages
)(
    input  wire                       clk,
    input  wire                       rst_n,

    // Control
    input  wire                       enable,
    input  wire                       reset_phase,

    // Configuration
    input  wire [PHASE_WIDTH-1:0]     freq_word,      // Frequency tuning word
    input  wire [PHASE_WIDTH-1:0]     phase_offset,   // Phase offset
    input  wire [OUTPUT_WIDTH-1:0]    amplitude,      // Output amplitude

    // Output
    output reg  signed [OUTPUT_WIDTH-1:0] i_out,      // Cosine output
    output reg  signed [OUTPUT_WIDTH-1:0] q_out,      // Sine output
    output reg                        out_valid
);

    // =========================================================================
    // Phase Accumulator
    // =========================================================================

    reg [PHASE_WIDTH-1:0] phase_acc;
    wire [PHASE_WIDTH-1:0] phase_total;

    always @(posedge clk) begin
        if (!rst_n || reset_phase) begin
            phase_acc <= 0;
        end else if (enable) begin
            phase_acc <= phase_acc + freq_word;
        end
    end

    assign phase_total = phase_acc + phase_offset;

    // =========================================================================
    // CORDIC Algorithm
    // Rotation mode: rotate from (1,0) by angle theta to get (cos, sin)
    //
    // Iteratively: x' = x - d*y*2^(-i)
    //              y' = y + d*x*2^(-i)
    //              z' = z - d*arctan(2^(-i))
    // where d = sign(z)
    // =========================================================================

    // CORDIC gain constant (product of all sqrt(1 + 2^(-2i)))
    // K = 0.6072529... ≈ 0x4DBA (for 16-bit, scaled by 2^15)
    localparam signed [15:0] CORDIC_GAIN = 16'h4DBA;

    // Arctan lookup table (scaled to phase width)
    // arctan(2^-i) in radians, scaled to 32-bit phase (2*pi = 2^32)
    wire [31:0] atan_lut [0:15];
    assign atan_lut[0]  = 32'h20000000;  // arctan(1)     = 45.000 deg
    assign atan_lut[1]  = 32'h12E4051E;  // arctan(0.5)   = 26.565 deg
    assign atan_lut[2]  = 32'h09FB385B;  // arctan(0.25)  = 14.036 deg
    assign atan_lut[3]  = 32'h051111D4;  // arctan(0.125) = 7.125 deg
    assign atan_lut[4]  = 32'h028B0D43;  // 3.576 deg
    assign atan_lut[5]  = 32'h0145D7E1;  // 1.790 deg
    assign atan_lut[6]  = 32'h00A2F61E;  // 0.895 deg
    assign atan_lut[7]  = 32'h00517C55;  // 0.448 deg
    assign atan_lut[8]  = 32'h0028BE53;  // 0.224 deg
    assign atan_lut[9]  = 32'h00145F2F;  // 0.112 deg
    assign atan_lut[10] = 32'h000A2F98;  // 0.056 deg
    assign atan_lut[11] = 32'h000517CC;  // 0.028 deg
    assign atan_lut[12] = 32'h00028BE6;  // 0.014 deg
    assign atan_lut[13] = 32'h000145F3;  // 0.007 deg
    assign atan_lut[14] = 32'h0000A2FA;  // 0.004 deg
    assign atan_lut[15] = 32'h0000517D;  // 0.002 deg

    // CORDIC pipeline registers
    reg signed [17:0] x [0:CORDIC_STAGES];
    reg signed [17:0] y [0:CORDIC_STAGES];
    reg signed [31:0] z [0:CORDIC_STAGES];
    reg valid_pipe [0:CORDIC_STAGES];

    // Quadrant handling: normalize angle to [-pi/4, pi/4]
    // Map all quadrants to first quadrant, then fix signs at end
    wire [1:0] quadrant;
    wire [31:0] normalized_phase;

    assign quadrant = phase_total[31:30];

    // Normalize to first quadrant
    always @(*) begin
        case (quadrant)
            2'b00: normalized_phase = phase_total;                    // Q1: 0-90
            2'b01: normalized_phase = 32'h40000000 - phase_total;     // Q2: 90-180
            2'b10: normalized_phase = phase_total - 32'h80000000;     // Q3: 180-270
            2'b11: normalized_phase = 32'hC0000000 - phase_total;     // Q4: 270-360
            default: normalized_phase = phase_total;
        endcase
    end

    reg [1:0] quad_pipe [0:CORDIC_STAGES];

    // Pipeline stage 0: Initialize
    always @(posedge clk) begin
        if (!rst_n) begin
            x[0] <= 0;
            y[0] <= 0;
            z[0] <= 0;
            valid_pipe[0] <= 0;
            quad_pipe[0] <= 0;
        end else begin
            valid_pipe[0] <= enable;
            quad_pipe[0] <= quadrant;
            if (enable) begin
                // Start with unit vector on X axis, scaled by CORDIC gain
                // We'll apply final amplitude scaling at the end
                x[0] <= 18'sh10000;  // 1.0 in Q1.16 format
                y[0] <= 18'sh00000;  // 0.0
                z[0] <= normalized_phase;
            end
        end
    end

    // CORDIC iterations (unrolled pipeline)
    genvar i;
    generate
        for (i = 0; i < CORDIC_STAGES; i = i + 1) begin : cordic_stage
            wire signed [17:0] x_shift = x[i] >>> i;
            wire signed [17:0] y_shift = y[i] >>> i;
            wire rotate_ccw = z[i][31];  // Sign bit: 1 = negative angle

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
                            // Rotate counter-clockwise
                            x[i+1] <= x[i] + y_shift;
                            y[i+1] <= y[i] - x_shift;
                            z[i+1] <= z[i] + atan_lut[i];
                        end else begin
                            // Rotate clockwise
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
    // Output Stage: Apply CORDIC gain compensation and quadrant correction
    // =========================================================================

    wire signed [17:0] cos_raw = x[CORDIC_STAGES];
    wire signed [17:0] sin_raw = y[CORDIC_STAGES];
    wire [1:0] final_quadrant = quad_pipe[CORDIC_STAGES];

    // Apply CORDIC gain and amplitude scaling
    wire signed [35:0] cos_scaled;
    wire signed [35:0] sin_scaled;

    // Scale by CORDIC gain (0.6072...) and amplitude
    // CORDIC_GAIN is ~0.6073 * 2^15
    assign cos_scaled = (cos_raw * CORDIC_GAIN) >>> 15;
    assign sin_scaled = (sin_raw * CORDIC_GAIN) >>> 15;

    // Final output with quadrant correction
    always @(posedge clk) begin
        if (!rst_n) begin
            i_out <= 0;
            q_out <= 0;
            out_valid <= 0;
        end else begin
            out_valid <= valid_pipe[CORDIC_STAGES];

            if (valid_pipe[CORDIC_STAGES]) begin
                case (final_quadrant)
                    2'b00: begin  // Q1: cos positive, sin positive
                        i_out <= cos_scaled[17:2];
                        q_out <= sin_scaled[17:2];
                    end
                    2'b01: begin  // Q2: cos negative, sin positive
                        i_out <= -cos_scaled[17:2];
                        q_out <= sin_scaled[17:2];
                    end
                    2'b10: begin  // Q3: cos negative, sin negative
                        i_out <= -cos_scaled[17:2];
                        q_out <= -sin_scaled[17:2];
                    end
                    2'b11: begin  // Q4: cos positive, sin negative
                        i_out <= cos_scaled[17:2];
                        q_out <= -sin_scaled[17:2];
                    end
                endcase
            end
        end
    end

endmodule
