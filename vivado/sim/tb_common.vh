//-----------------------------------------------------------------------------
// Common Testbench Utilities
// R4W FPGA Acceleration Layer
//-----------------------------------------------------------------------------

`ifndef TB_COMMON_VH
`define TB_COMMON_VH

// Simulation parameters
`define CLK_PERIOD 10  // 100 MHz clock

// AXI response codes
`define AXI_RESP_OKAY   2'b00
`define AXI_RESP_EXOKAY 2'b01
`define AXI_RESP_SLVERR 2'b10
`define AXI_RESP_DECERR 2'b11

// I/Q packing macros
`define IQ_PACK(i, q) {i[15:0], q[15:0]}
`define IQ_UNPACK_I(packed) packed[31:16]
`define IQ_UNPACK_Q(packed) packed[15:0]

// Fixed-point conversion (Q15 format)
`define FLOAT_TO_Q15(f) ($signed($rtoi((f) * 32768.0)))
`define Q15_TO_FLOAT(q) ($itor($signed(q)) / 32768.0)

// Test assertion macros
`define ASSERT_EQ(actual, expected, msg) \
    if ((actual) !== (expected)) begin \
        $display("FAIL: %s - Expected 0x%h, Got 0x%h at time %t", msg, expected, actual, $time); \
        error_count = error_count + 1; \
    end else begin \
        $display("PASS: %s", msg); \
    end

`define ASSERT_NEAR(actual, expected, tolerance, msg) \
    if (((actual) > (expected) + (tolerance)) || ((actual) < (expected) - (tolerance))) begin \
        $display("FAIL: %s - Expected %0d +/- %0d, Got %0d at time %t", msg, expected, tolerance, actual, $time); \
        error_count = error_count + 1; \
    end else begin \
        $display("PASS: %s (value=%0d)", msg, actual); \
    end

`define TEST_START(name) \
    $display("\n========================================"); \
    $display("TEST: %s", name); \
    $display("========================================");

`define TEST_SUMMARY \
    $display("\n========================================"); \
    $display("TEST SUMMARY: %0d errors", error_count); \
    $display("========================================"); \
    if (error_count == 0) $display("ALL TESTS PASSED!"); \
    else $display("SOME TESTS FAILED!");

`endif
