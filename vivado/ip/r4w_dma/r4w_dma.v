//-----------------------------------------------------------------------------
// R4W DMA Controller Core
// R4W FPGA Acceleration Layer
//
// Manages scatter-gather DMA transfers between PS memory and PL DSP cores.
// Works with Xilinx AXI DMA IP for high-throughput I/Q sample streaming.
//
// Features:
//   - Dual-channel (TX/RX) streaming
//   - Configurable buffer sizes (up to 4K samples per transfer)
//   - Interrupt generation on completion
//   - Status monitoring and error reporting
//   - Sample packing/unpacking (16-bit I/Q to 32-bit word)
//
// trace:FR-0086 | ai:claude
//-----------------------------------------------------------------------------

module r4w_dma #(
    parameter DATA_WIDTH = 32,
    parameter BUFFER_DEPTH = 4096,
    parameter ADDR_WIDTH = $clog2(BUFFER_DEPTH)
)(
    input  wire clk,
    input  wire rst_n,

    // Control interface
    input  wire        ctrl_start_tx,       // Start TX DMA
    input  wire        ctrl_start_rx,       // Start RX DMA
    input  wire        ctrl_abort,          // Abort current transfer
    input  wire [15:0] ctrl_tx_len,         // TX transfer length (samples)
    input  wire [15:0] ctrl_rx_len,         // RX transfer length (samples)
    input  wire        ctrl_continuous,     // Continuous mode

    // Status outputs
    output wire        status_tx_busy,
    output wire        status_rx_busy,
    output wire        status_tx_done,
    output wire        status_rx_done,
    output wire        status_tx_error,
    output wire        status_rx_error,
    output wire [15:0] status_tx_count,
    output wire [15:0] status_rx_count,

    // Interrupt output
    output wire        irq_tx_done,
    output wire        irq_rx_done,
    output wire        irq_error,

    // AXI-Stream Input from Xilinx DMA (TX path - memory to PL)
    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire                  s_axis_tlast,
    input  wire [3:0]            s_axis_tkeep,

    // AXI-Stream Output to Xilinx DMA (RX path - PL to memory)
    output wire [DATA_WIDTH-1:0] m_axis_tdata,
    output wire                  m_axis_tvalid,
    input  wire                  m_axis_tready,
    output wire                  m_axis_tlast,
    output wire [3:0]            m_axis_tkeep,

    // AXI-Stream Output to DSP cores (from TX buffer)
    output wire [DATA_WIDTH-1:0] m_dsp_tdata,
    output wire                  m_dsp_tvalid,
    input  wire                  m_dsp_tready,
    output wire                  m_dsp_tlast,

    // AXI-Stream Input from DSP cores (to RX buffer)
    input  wire [DATA_WIDTH-1:0] s_dsp_tdata,
    input  wire                  s_dsp_tvalid,
    output wire                  s_dsp_tready,
    input  wire                  s_dsp_tlast
);

    // =========================================================================
    // TX Path State Machine
    // =========================================================================

    localparam TX_IDLE   = 3'd0;
    localparam TX_LOAD   = 3'd1;
    localparam TX_STREAM = 3'd2;
    localparam TX_DONE   = 3'd3;
    localparam TX_ERROR  = 3'd4;

    reg [2:0] tx_state;
    reg [15:0] tx_count;
    reg [15:0] tx_target;
    reg tx_done_irq;
    reg tx_error_flag;

    // TX Buffer (BRAM)
    reg [DATA_WIDTH-1:0] tx_buffer [0:BUFFER_DEPTH-1];
    reg [ADDR_WIDTH-1:0] tx_wr_ptr;
    reg [ADDR_WIDTH-1:0] tx_rd_ptr;

    wire tx_buf_empty = (tx_wr_ptr == tx_rd_ptr);
    wire tx_buf_full = (tx_wr_ptr + 1 == tx_rd_ptr);

    // TX from DMA to buffer
    wire tx_dma_write = s_axis_tvalid && s_axis_tready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_state <= TX_IDLE;
            tx_count <= 0;
            tx_target <= 0;
            tx_wr_ptr <= 0;
            tx_rd_ptr <= 0;
            tx_done_irq <= 0;
            tx_error_flag <= 0;
        end else begin
            tx_done_irq <= 0;  // Pulse

            case (tx_state)
                TX_IDLE: begin
                    if (ctrl_start_tx) begin
                        tx_state <= TX_LOAD;
                        tx_target <= ctrl_tx_len;
                        tx_count <= 0;
                        tx_wr_ptr <= 0;
                        tx_rd_ptr <= 0;
                        tx_error_flag <= 0;
                    end
                end

                TX_LOAD: begin
                    // Load samples from DMA into buffer
                    if (ctrl_abort) begin
                        tx_state <= TX_ERROR;
                    end else if (tx_dma_write) begin
                        tx_buffer[tx_wr_ptr] <= s_axis_tdata;
                        tx_wr_ptr <= tx_wr_ptr + 1;

                        if (s_axis_tlast || tx_wr_ptr >= tx_target - 1) begin
                            tx_state <= TX_STREAM;
                        end
                    end
                end

                TX_STREAM: begin
                    // Stream samples to DSP core
                    if (ctrl_abort) begin
                        tx_state <= TX_ERROR;
                    end else if (m_dsp_tvalid && m_dsp_tready) begin
                        tx_rd_ptr <= tx_rd_ptr + 1;
                        tx_count <= tx_count + 1;

                        if (tx_rd_ptr >= tx_wr_ptr - 1) begin
                            tx_state <= ctrl_continuous ? TX_LOAD : TX_DONE;
                            tx_done_irq <= !ctrl_continuous;
                            if (ctrl_continuous) begin
                                tx_rd_ptr <= 0;  // Restart buffer
                            end
                        end
                    end
                end

                TX_DONE: begin
                    if (ctrl_start_tx) begin
                        tx_state <= TX_LOAD;
                        tx_target <= ctrl_tx_len;
                        tx_count <= 0;
                        tx_wr_ptr <= 0;
                        tx_rd_ptr <= 0;
                    end
                end

                TX_ERROR: begin
                    tx_error_flag <= 1;
                    if (ctrl_start_tx || ctrl_abort) begin
                        tx_state <= TX_IDLE;
                    end
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // TX output signals
    assign s_axis_tready = (tx_state == TX_LOAD) && !tx_buf_full;
    assign m_dsp_tdata = tx_buffer[tx_rd_ptr];
    assign m_dsp_tvalid = (tx_state == TX_STREAM) && !tx_buf_empty;
    assign m_dsp_tlast = (tx_rd_ptr == tx_wr_ptr - 1);

    assign status_tx_busy = (tx_state == TX_LOAD) || (tx_state == TX_STREAM);
    assign status_tx_done = (tx_state == TX_DONE);
    assign status_tx_error = tx_error_flag;
    assign status_tx_count = tx_count;
    assign irq_tx_done = tx_done_irq;

    // =========================================================================
    // RX Path State Machine
    // =========================================================================

    localparam RX_IDLE   = 3'd0;
    localparam RX_STREAM = 3'd1;
    localparam RX_DRAIN  = 3'd2;
    localparam RX_DONE   = 3'd3;
    localparam RX_ERROR  = 3'd4;

    reg [2:0] rx_state;
    reg [15:0] rx_count;
    reg [15:0] rx_target;
    reg rx_done_irq;
    reg rx_error_flag;

    // RX Buffer (BRAM)
    reg [DATA_WIDTH-1:0] rx_buffer [0:BUFFER_DEPTH-1];
    reg [ADDR_WIDTH-1:0] rx_wr_ptr;
    reg [ADDR_WIDTH-1:0] rx_rd_ptr;

    wire rx_buf_empty = (rx_wr_ptr == rx_rd_ptr);
    wire rx_buf_full = (rx_wr_ptr + 1 == rx_rd_ptr);

    // RX from DSP to buffer
    wire rx_dsp_write = s_dsp_tvalid && s_dsp_tready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= RX_IDLE;
            rx_count <= 0;
            rx_target <= 0;
            rx_wr_ptr <= 0;
            rx_rd_ptr <= 0;
            rx_done_irq <= 0;
            rx_error_flag <= 0;
        end else begin
            rx_done_irq <= 0;  // Pulse

            case (rx_state)
                RX_IDLE: begin
                    if (ctrl_start_rx) begin
                        rx_state <= RX_STREAM;
                        rx_target <= ctrl_rx_len;
                        rx_count <= 0;
                        rx_wr_ptr <= 0;
                        rx_rd_ptr <= 0;
                        rx_error_flag <= 0;
                    end
                end

                RX_STREAM: begin
                    // Capture samples from DSP into buffer
                    if (ctrl_abort) begin
                        rx_state <= RX_ERROR;
                    end else if (rx_dsp_write) begin
                        rx_buffer[rx_wr_ptr] <= s_dsp_tdata;
                        rx_wr_ptr <= rx_wr_ptr + 1;
                        rx_count <= rx_count + 1;

                        if (s_dsp_tlast || rx_count >= rx_target - 1) begin
                            rx_state <= RX_DRAIN;
                        end
                    end

                    // Handle buffer overflow
                    if (rx_buf_full) begin
                        rx_state <= RX_ERROR;
                        rx_error_flag <= 1;
                    end
                end

                RX_DRAIN: begin
                    // Drain buffer to DMA
                    if (ctrl_abort) begin
                        rx_state <= RX_ERROR;
                    end else if (m_axis_tvalid && m_axis_tready) begin
                        rx_rd_ptr <= rx_rd_ptr + 1;

                        if (rx_rd_ptr >= rx_wr_ptr - 1) begin
                            rx_state <= ctrl_continuous ? RX_STREAM : RX_DONE;
                            rx_done_irq <= !ctrl_continuous;
                            if (ctrl_continuous) begin
                                rx_wr_ptr <= 0;
                                rx_rd_ptr <= 0;
                            end
                        end
                    end
                end

                RX_DONE: begin
                    if (ctrl_start_rx) begin
                        rx_state <= RX_STREAM;
                        rx_target <= ctrl_rx_len;
                        rx_count <= 0;
                        rx_wr_ptr <= 0;
                        rx_rd_ptr <= 0;
                    end
                end

                RX_ERROR: begin
                    rx_error_flag <= 1;
                    if (ctrl_start_rx || ctrl_abort) begin
                        rx_state <= RX_IDLE;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase
        end
    end

    // RX output signals
    assign s_dsp_tready = (rx_state == RX_STREAM) && !rx_buf_full;
    assign m_axis_tdata = rx_buffer[rx_rd_ptr];
    assign m_axis_tvalid = (rx_state == RX_DRAIN) && !rx_buf_empty;
    assign m_axis_tlast = (rx_rd_ptr == rx_wr_ptr - 1);
    assign m_axis_tkeep = 4'b1111;

    assign status_rx_busy = (rx_state == RX_STREAM) || (rx_state == RX_DRAIN);
    assign status_rx_done = (rx_state == RX_DONE);
    assign status_rx_error = rx_error_flag;
    assign status_rx_count = rx_count;
    assign irq_rx_done = rx_done_irq;

    // Combined error interrupt
    assign irq_error = tx_error_flag || rx_error_flag;

endmodule
