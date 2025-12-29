//-----------------------------------------------------------------------------
// I/Q Sample Packing Utilities
// R4W FPGA Acceleration Layer
//
// Utilities for packing and unpacking I/Q samples in 32-bit words.
// Format: [31:16] = I (signed 16-bit), [15:0] = Q (signed 16-bit)
//-----------------------------------------------------------------------------

// Pack I and Q into a 32-bit word
// I goes in upper 16 bits, Q goes in lower 16 bits
function [31:0] iq_pack;
    input signed [15:0] i_sample;
    input signed [15:0] q_sample;
    begin
        iq_pack = {i_sample, q_sample};
    end
endfunction

// Unpack I from a 32-bit word
function signed [15:0] iq_unpack_i;
    input [31:0] packed;
    begin
        iq_unpack_i = packed[31:16];
    end
endfunction

// Unpack Q from a 32-bit word
function signed [15:0] iq_unpack_q;
    input [31:0] packed;
    begin
        iq_unpack_q = packed[15:0];
    end
endfunction

//-----------------------------------------------------------------------------
// I/Q Packer Module
// Combines separate I and Q streams into packed 32-bit samples
//-----------------------------------------------------------------------------
module iq_packer (
    input  wire        clk,
    input  wire        rst_n,

    // Input I/Q streams (16-bit signed)
    input  wire signed [15:0] i_in,
    input  wire signed [15:0] q_in,
    input  wire        in_valid,
    output wire        in_ready,

    // Output packed stream (32-bit)
    output wire [31:0] iq_out,
    output wire        out_valid,
    input  wire        out_ready
);

    // Simple passthrough with packing
    assign iq_out = {i_in, q_in};
    assign out_valid = in_valid;
    assign in_ready = out_ready;

endmodule

//-----------------------------------------------------------------------------
// I/Q Unpacker Module
// Splits packed 32-bit samples into separate I and Q streams
//-----------------------------------------------------------------------------
module iq_unpacker (
    input  wire        clk,
    input  wire        rst_n,

    // Input packed stream (32-bit)
    input  wire [31:0] iq_in,
    input  wire        in_valid,
    output wire        in_ready,

    // Output I/Q streams (16-bit signed)
    output wire signed [15:0] i_out,
    output wire signed [15:0] q_out,
    output wire        out_valid,
    input  wire        out_ready
);

    // Simple passthrough with unpacking
    assign i_out = iq_in[31:16];
    assign q_out = iq_in[15:0];
    assign out_valid = in_valid;
    assign in_ready = out_ready;

endmodule

//-----------------------------------------------------------------------------
// Complex Multiplier
// (a + jb) * (c + jd) = (ac - bd) + j(ad + bc)
// Uses 3 DSP slices with Karatsuba optimization:
// P1 = a*c, P2 = b*d, P3 = (a+b)*(c+d)
// Real = P1 - P2, Imag = P3 - P1 - P2
//-----------------------------------------------------------------------------
module complex_mult (
    input  wire        clk,
    input  wire        rst_n,

    // Input A (I/Q)
    input  wire signed [15:0] a_i,
    input  wire signed [15:0] a_q,
    input  wire        a_valid,

    // Input B (I/Q)
    input  wire signed [15:0] b_i,
    input  wire signed [15:0] b_q,
    input  wire        b_valid,

    // Output (I/Q) - registered
    output reg  signed [31:0] out_i,
    output reg  signed [31:0] out_q,
    output reg         out_valid
);

    // Pipeline registers
    reg signed [31:0] p1, p2, p3;
    reg signed [16:0] a_sum, b_sum;
    reg valid_d1, valid_d2;

    // Stage 1: Compute partial products
    always @(posedge clk) begin
        if (!rst_n) begin
            p1 <= 0;
            p2 <= 0;
            a_sum <= 0;
            b_sum <= 0;
            valid_d1 <= 0;
        end else begin
            if (a_valid && b_valid) begin
                p1 <= a_i * b_i;         // a*c
                p2 <= a_q * b_q;         // b*d
                a_sum <= a_i + a_q;      // a+b
                b_sum <= b_i + b_q;      // c+d
                valid_d1 <= 1'b1;
            end else begin
                valid_d1 <= 1'b0;
            end
        end
    end

    // Stage 2: Compute P3 and final result
    always @(posedge clk) begin
        if (!rst_n) begin
            p3 <= 0;
            out_i <= 0;
            out_q <= 0;
            valid_d2 <= 0;
            out_valid <= 0;
        end else begin
            valid_d2 <= valid_d1;
            if (valid_d1) begin
                p3 <= a_sum * b_sum;     // (a+b)*(c+d)
            end

            out_valid <= valid_d2;
            if (valid_d2) begin
                out_i <= p1 - p2;        // ac - bd
                out_q <= p3 - p1 - p2;   // ad + bc
            end
        end
    end

endmodule

//-----------------------------------------------------------------------------
// Magnitude Squared Calculator
// |z|^2 = I^2 + Q^2
//-----------------------------------------------------------------------------
module mag_squared (
    input  wire        clk,
    input  wire        rst_n,

    input  wire signed [15:0] i_in,
    input  wire signed [15:0] q_in,
    input  wire        in_valid,

    output reg  [31:0] mag_sq,
    output reg         out_valid
);

    reg signed [31:0] i_sq, q_sq;
    reg valid_d1;

    always @(posedge clk) begin
        if (!rst_n) begin
            i_sq <= 0;
            q_sq <= 0;
            mag_sq <= 0;
            valid_d1 <= 0;
            out_valid <= 0;
        end else begin
            // Stage 1: Square
            valid_d1 <= in_valid;
            if (in_valid) begin
                i_sq <= i_in * i_in;
                q_sq <= q_in * q_in;
            end

            // Stage 2: Add
            out_valid <= valid_d1;
            if (valid_d1) begin
                mag_sq <= i_sq + q_sq;
            end
        end
    end

endmodule
