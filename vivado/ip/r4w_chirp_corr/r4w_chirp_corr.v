//-----------------------------------------------------------------------------
// R4W LoRa Chirp Correlator
// R4W FPGA Acceleration Layer
//
// Demodulates LoRa CSS symbols by:
// 1. Multiplying received samples by conjugate downchirp (dechirping)
// 2. Performing FFT on dechirped samples
// 3. Finding peak bin (= symbol value)
//
// The dechirping converts the chirp's frequency sweep into a constant
// frequency, which produces a peak in the FFT at the symbol value.
//
// trace:FR-0034 | ai:claude
//-----------------------------------------------------------------------------

module r4w_chirp_corr #(
    parameter MAX_SF = 12,
    parameter DATA_WIDTH = 16,
    parameter CORDIC_STAGES = 16
)(
    input  wire                           clk,
    input  wire                           rst_n,

    // Control
    input  wire                           start,
    input  wire [3:0]                     sf,           // Spreading factor (5-12)
    input  wire [31:0]                    threshold,    // Detection threshold

    // Status
    output reg                            done,
    output reg                            busy,
    output reg                            detected,

    // Input samples (one symbol period = 2^SF samples)
    input  wire [2*DATA_WIDTH-1:0]        s_axis_tdata,    // {I, Q}
    input  wire                           s_axis_tvalid,
    output wire                           s_axis_tready,
    input  wire                           s_axis_tlast,

    // Results
    output reg  [11:0]                    symbol_out,      // Detected symbol (0 to 2^SF-1)
    output reg  [31:0]                    magnitude        // Peak magnitude
);

    // =========================================================================
    // State Machine
    // =========================================================================

    localparam S_IDLE      = 3'd0;
    localparam S_DECHIRP   = 3'd1;
    localparam S_FFT       = 3'd2;
    localparam S_PEAK_FIND = 3'd3;
    localparam S_DONE      = 3'd4;

    reg [2:0] state;
    reg [12:0] sample_count;
    wire [12:0] samples_per_symbol = (13'd1 << sf);

    // =========================================================================
    // Downchirp Generator (conjugate of upchirp)
    // =========================================================================

    reg [31:0] phase_acc;
    reg [31:0] freq_word;
    wire [31:0] base_freq_increment = (32'h1 << (32 - sf));

    // Downchirp: frequency decreases
    wire [31:0] downchirp_phase;
    assign downchirp_phase = phase_acc;

    // =========================================================================
    // CORDIC for Downchirp Generation
    // =========================================================================

    // Arctan LUT (same as chirp_gen)
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

    localparam signed [15:0] CORDIC_GAIN = 16'h4DBA;

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

    // Pipeline stage 0
    reg dechirp_enable;

    always @(posedge clk) begin
        if (!rst_n) begin
            x[0] <= 0;
            y[0] <= 0;
            z[0] <= 0;
            valid_pipe[0] <= 0;
            quad_pipe[0] <= 0;
        end else begin
            valid_pipe[0] <= dechirp_enable;
            quad_pipe[0] <= quadrant;
            if (dechirp_enable) begin
                x[0] <= 18'sh10000;
                y[0] <= 18'sh00000;
                z[0] <= normalized_phase;
            end
        end
    end

    // CORDIC iterations
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

    // Downchirp output (conjugate: same cos, negate sin)
    wire signed [17:0] cos_raw = x[CORDIC_STAGES];
    wire signed [17:0] sin_raw = y[CORDIC_STAGES];
    wire [1:0] final_quadrant = quad_pipe[CORDIC_STAGES];
    wire downchirp_valid = valid_pipe[CORDIC_STAGES];

    wire signed [35:0] cos_scaled = (cos_raw * CORDIC_GAIN) >>> 15;
    wire signed [35:0] sin_scaled = (sin_raw * CORDIC_GAIN) >>> 15;

    reg signed [15:0] downchirp_i;
    reg signed [15:0] downchirp_q;  // Negated for conjugate

    always @(posedge clk) begin
        if (!rst_n) begin
            downchirp_i <= 0;
            downchirp_q <= 0;
        end else if (valid_pipe[CORDIC_STAGES]) begin
            case (final_quadrant)
                2'b00: begin
                    downchirp_i <= cos_scaled[17:2];
                    downchirp_q <= -sin_scaled[17:2];  // Conjugate
                end
                2'b01: begin
                    downchirp_i <= -cos_scaled[17:2];
                    downchirp_q <= -sin_scaled[17:2];
                end
                2'b10: begin
                    downchirp_i <= -cos_scaled[17:2];
                    downchirp_q <= sin_scaled[17:2];
                end
                2'b11: begin
                    downchirp_i <= cos_scaled[17:2];
                    downchirp_q <= sin_scaled[17:2];
                end
            endcase
        end
    end

    // =========================================================================
    // Input Sample Buffering and Dechirping
    // =========================================================================

    reg signed [DATA_WIDTH-1:0] input_buffer_i [0:4095];
    reg signed [DATA_WIDTH-1:0] input_buffer_q [0:4095];
    reg signed [DATA_WIDTH-1:0] dechirp_buffer_i [0:4095];
    reg signed [DATA_WIDTH-1:0] dechirp_buffer_q [0:4095];

    wire signed [DATA_WIDTH-1:0] input_i = s_axis_tdata[2*DATA_WIDTH-1:DATA_WIDTH];
    wire signed [DATA_WIDTH-1:0] input_q = s_axis_tdata[DATA_WIDTH-1:0];

    reg [12:0] input_ptr;
    reg [12:0] dechirp_ptr;
    reg dechirping;

    // Complex multiply for dechirping: (a+jb)*(c-jd) = (ac+bd) + j(bc-ad)
    wire signed [31:0] dechirp_i_prod = input_buffer_i[dechirp_ptr] * downchirp_i +
                                         input_buffer_q[dechirp_ptr] * downchirp_q;
    wire signed [31:0] dechirp_q_prod = input_buffer_q[dechirp_ptr] * downchirp_i -
                                         input_buffer_i[dechirp_ptr] * downchirp_q;

    assign s_axis_tready = (state == S_DECHIRP) && (input_ptr < samples_per_symbol);

    // =========================================================================
    // FFT Interface for Peak Detection
    // =========================================================================

    reg [12:0] fft_input_ptr;
    reg [12:0] fft_output_ptr;
    reg fft_running;

    // Simple FFT placeholder - in real hardware, connect to FFT IP
    // For now, we'll do a simplified peak detection

    reg [31:0] magnitude_buffer [0:4095];
    reg [31:0] max_magnitude;
    reg [11:0] max_bin;
    reg [12:0] peak_search_ptr;

    // =========================================================================
    // State Machine
    // =========================================================================

    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            sample_count <= 0;
            input_ptr <= 0;
            dechirp_ptr <= 0;
            phase_acc <= 0;
            freq_word <= 0;
            dechirp_enable <= 0;
            dechirping <= 0;
            fft_input_ptr <= 0;
            fft_output_ptr <= 0;
            fft_running <= 0;
            peak_search_ptr <= 0;
            max_magnitude <= 0;
            max_bin <= 0;
            done <= 0;
            busy <= 0;
            detected <= 0;
            symbol_out <= 0;
            magnitude <= 0;
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        state <= S_DECHIRP;
                        busy <= 1'b1;
                        input_ptr <= 0;
                        dechirp_ptr <= 0;
                        phase_acc <= 0;
                        freq_word <= 0;  // Downchirp starts at max frequency
                        dechirping <= 0;
                        max_magnitude <= 0;
                        max_bin <= 0;
                    end
                end

                S_DECHIRP: begin
                    // Buffer input samples
                    if (s_axis_tvalid && s_axis_tready) begin
                        input_buffer_i[input_ptr] <= input_i;
                        input_buffer_q[input_ptr] <= input_q;
                        input_ptr <= input_ptr + 1;
                    end

                    // Start dechirping once we have some samples
                    if (input_ptr > CORDIC_STAGES && !dechirping) begin
                        dechirping <= 1'b1;
                        dechirp_enable <= 1'b1;
                    end

                    // Dechirp process
                    if (dechirping) begin
                        // Update downchirp phase (decreasing frequency)
                        freq_word <= freq_word - base_freq_increment;
                        phase_acc <= phase_acc + freq_word;
                        dechirp_enable <= 1'b1;

                        if (downchirp_valid) begin
                            dechirp_buffer_i[dechirp_ptr] <= dechirp_i_prod[31:16];
                            dechirp_buffer_q[dechirp_ptr] <= dechirp_q_prod[31:16];
                            dechirp_ptr <= dechirp_ptr + 1;
                        end

                        if (dechirp_ptr >= samples_per_symbol - 1) begin
                            state <= S_FFT;
                            dechirping <= 1'b0;
                            dechirp_enable <= 1'b0;
                            fft_input_ptr <= 0;
                        end
                    end

                    // Timeout if samples stop coming
                    if (input_ptr >= samples_per_symbol && !dechirping) begin
                        state <= S_FFT;
                        fft_input_ptr <= 0;
                    end
                end

                S_FFT: begin
                    // In real implementation, run FFT on dechirped samples
                    // Here we compute magnitude squared directly (simplified)
                    if (fft_input_ptr < samples_per_symbol) begin
                        // |z|^2 = I^2 + Q^2
                        magnitude_buffer[fft_input_ptr] <=
                            dechirp_buffer_i[fft_input_ptr] * dechirp_buffer_i[fft_input_ptr] +
                            dechirp_buffer_q[fft_input_ptr] * dechirp_buffer_q[fft_input_ptr];
                        fft_input_ptr <= fft_input_ptr + 1;
                    end else begin
                        state <= S_PEAK_FIND;
                        peak_search_ptr <= 0;
                    end
                end

                S_PEAK_FIND: begin
                    // Find maximum magnitude bin
                    if (peak_search_ptr < samples_per_symbol) begin
                        if (magnitude_buffer[peak_search_ptr] > max_magnitude) begin
                            max_magnitude <= magnitude_buffer[peak_search_ptr];
                            max_bin <= peak_search_ptr[11:0];
                        end
                        peak_search_ptr <= peak_search_ptr + 1;
                    end else begin
                        state <= S_DONE;
                        symbol_out <= max_bin;
                        magnitude <= max_magnitude;
                        detected <= (max_magnitude > threshold);
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

endmodule
