//-----------------------------------------------------------------------------
// R4W SPI Slave Interface for Lattice FPGAs
// R4W FPGA Acceleration Layer
//
// Simple SPI slave for CPU communication via FTDI SPI master.
// Provides register read/write interface to internal IP cores.
//
// Protocol:
//   Write: [0x80] [ADDR_H] [ADDR_L] [DATA_3] [DATA_2] [DATA_1] [DATA_0]
//   Read:  [0x00] [ADDR_H] [ADDR_L] -> [DATA_3] [DATA_2] [DATA_1] [DATA_0]
//
// Compatible with: iCE40, ECP5
// Toolchain: Yosys + IceStorm/Trellis
//
// trace:FR-0080 | ai:claude
//-----------------------------------------------------------------------------

module r4w_spi_slave #(
    parameter ADDR_WIDTH = 16,
    parameter DATA_WIDTH = 32
)(
    // System signals
    input  wire clk,
    input  wire rst_n,

    // SPI interface
    input  wire spi_clk,
    input  wire spi_cs_n,
    input  wire spi_mosi,
    output reg  spi_miso,

    // Register interface to internal logic
    output reg  [ADDR_WIDTH-1:0] reg_addr,
    output reg  [DATA_WIDTH-1:0] reg_wdata,
    output reg                   reg_wen,
    output reg                   reg_ren,
    input  wire [DATA_WIDTH-1:0] reg_rdata
);

    // =========================================================================
    // SPI Clock Domain Crossing
    // =========================================================================

    // Synchronize SPI signals to system clock
    reg [2:0] spi_clk_sync;
    reg [2:0] spi_cs_sync;
    reg [2:0] spi_mosi_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_clk_sync  <= 3'b000;
            spi_cs_sync   <= 3'b111;
            spi_mosi_sync <= 3'b000;
        end else begin
            spi_clk_sync  <= {spi_clk_sync[1:0], spi_clk};
            spi_cs_sync   <= {spi_cs_sync[1:0], spi_cs_n};
            spi_mosi_sync <= {spi_mosi_sync[1:0], spi_mosi};
        end
    end

    wire spi_clk_rising  = spi_clk_sync[1] && !spi_clk_sync[2];
    wire spi_clk_falling = !spi_clk_sync[1] && spi_clk_sync[2];
    wire spi_cs_active   = !spi_cs_sync[2];
    wire spi_mosi_in     = spi_mosi_sync[2];

    // =========================================================================
    // State Machine
    // =========================================================================

    localparam STATE_IDLE      = 3'd0;
    localparam STATE_CMD       = 3'd1;
    localparam STATE_ADDR_H    = 3'd2;
    localparam STATE_ADDR_L    = 3'd3;
    localparam STATE_WRITE_0   = 3'd4;
    localparam STATE_WRITE_1   = 3'd5;
    localparam STATE_WRITE_2   = 3'd6;
    localparam STATE_WRITE_3   = 3'd7;

    reg [2:0] state;
    reg [2:0] state_prev;        // Previous state for edge detection
    reg [2:0] bit_count;
    reg [7:0] shift_in;
    reg [7:0] shift_out;
    reg       is_write;
    reg [31:0] write_data;
    reg [31:0] read_data_latch;  // Latched read data for shifting
    reg        load_read_data;   // Flag to load read data on next cycle

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= STATE_IDLE;
            state_prev <= STATE_IDLE;
            bit_count  <= 0;
            shift_in   <= 0;
            shift_out  <= 0;
            is_write   <= 0;
            reg_addr   <= 0;
            reg_wdata  <= 0;
            reg_wen    <= 0;
            reg_ren    <= 0;
            write_data <= 0;
            spi_miso   <= 1'b0;
            read_data_latch <= 0;
            load_read_data <= 0;
        end else begin
            // Track state transitions
            state_prev <= state;

            // Default - clear single-cycle pulses
            reg_wen <= 1'b0;
            reg_ren <= 1'b0;

            // Latch read data and preload shift register one cycle after reg_ren
            if (load_read_data) begin
                read_data_latch <= reg_rdata;
                shift_out <= reg_rdata[31:24];
                load_read_data <= 1'b0;
            end

            // Preload next byte into shift_out when just entering a new state (for reads)
            // This ensures shift_out is ready before the first falling edge of the new byte
            if (!is_write && state != state_prev) begin
                case (state)
                    STATE_WRITE_0: shift_out <= read_data_latch[23:16];
                    STATE_WRITE_1: shift_out <= read_data_latch[15:8];
                    STATE_WRITE_2: shift_out <= read_data_latch[7:0];
                    default: ; // Keep current value
                endcase
            end

            if (!spi_cs_active) begin
                // CS inactive - reset state
                state     <= STATE_IDLE;
                bit_count <= 0;
            end else if (spi_clk_rising) begin
                // Sample MOSI on rising edge
                shift_in <= {shift_in[6:0], spi_mosi_in};
                bit_count <= bit_count + 1;

                if (bit_count == 7) begin
                    // Byte complete
                    bit_count <= 0;

                    case (state)
                        STATE_IDLE: begin
                            // First byte is command
                            is_write <= shift_in[6]; // Bit 7 will be shifted in
                            state <= STATE_CMD;
                        end

                        STATE_CMD: begin
                            // Address high byte
                            reg_addr[15:8] <= {shift_in[6:0], spi_mosi_in};
                            state <= STATE_ADDR_H;
                        end

                        STATE_ADDR_H: begin
                            // Address low byte
                            reg_addr[7:0] <= {shift_in[6:0], spi_mosi_in};
                            state <= STATE_ADDR_L;

                            // If read, start read operation now
                            if (!is_write) begin
                                reg_ren <= 1'b1;
                                load_read_data <= 1'b1;  // Latch read data next cycle
                            end
                        end

                        STATE_ADDR_L: begin
                            if (is_write) begin
                                // Write data byte 3 (MSB)
                                write_data[31:24] <= {shift_in[6:0], spi_mosi_in};
                            end
                            // For read: shift_out already loaded, just shift during this byte
                            state <= STATE_WRITE_0;
                        end

                        STATE_WRITE_0: begin
                            if (is_write) begin
                                write_data[23:16] <= {shift_in[6:0], spi_mosi_in};
                            end else begin
                                shift_out <= read_data_latch[23:16];
                            end
                            state <= STATE_WRITE_1;
                        end

                        STATE_WRITE_1: begin
                            if (is_write) begin
                                write_data[15:8] <= {shift_in[6:0], spi_mosi_in};
                            end else begin
                                shift_out <= read_data_latch[15:8];
                            end
                            state <= STATE_WRITE_2;
                        end

                        STATE_WRITE_2: begin
                            if (is_write) begin
                                write_data[7:0] <= {shift_in[6:0], spi_mosi_in};
                                // Complete write
                                reg_wdata <= {write_data[31:8], shift_in[6:0], spi_mosi_in};
                                reg_wen <= 1'b1;
                            end else begin
                                shift_out <= read_data_latch[7:0];
                            end
                            state <= STATE_WRITE_3;
                        end

                        STATE_WRITE_3: begin
                            // Transaction complete, wait for CS to go high
                            state <= STATE_IDLE;
                        end
                    endcase
                end
            end else if (spi_clk_falling) begin
                // Shift out MISO on falling edge (only during data phase for reads)
                spi_miso <= shift_out[7];
                // Only shift during data bytes (STATE_ADDR_L and later for reads)
                if (!is_write && (state >= STATE_ADDR_L)) begin
                    shift_out <= {shift_out[6:0], 1'b0};
                end
            end
        end
    end

endmodule
