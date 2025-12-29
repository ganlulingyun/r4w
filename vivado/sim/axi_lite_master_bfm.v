//-----------------------------------------------------------------------------
// AXI-Lite Master Bus Functional Model (BFM)
// R4W FPGA Acceleration Layer
//
// Simple BFM for driving AXI-Lite transactions in testbenches
//-----------------------------------------------------------------------------

module axi_lite_master_bfm #(
    parameter ADDR_WIDTH = 8,
    parameter DATA_WIDTH = 32
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // AXI-Lite Master Interface
    output reg  [ADDR_WIDTH-1:0]    m_axi_awaddr,
    output reg  [2:0]               m_axi_awprot,
    output reg                      m_axi_awvalid,
    input  wire                     m_axi_awready,

    output reg  [DATA_WIDTH-1:0]    m_axi_wdata,
    output reg  [(DATA_WIDTH/8)-1:0] m_axi_wstrb,
    output reg                      m_axi_wvalid,
    input  wire                     m_axi_wready,

    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output reg                      m_axi_bready,

    output reg  [ADDR_WIDTH-1:0]    m_axi_araddr,
    output reg  [2:0]               m_axi_arprot,
    output reg                      m_axi_arvalid,
    input  wire                     m_axi_arready,

    input  wire [DATA_WIDTH-1:0]    m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rvalid,
    output reg                      m_axi_rready
);

    // Transaction storage
    reg [DATA_WIDTH-1:0] read_data;
    reg [1:0] read_resp;
    reg [1:0] write_resp;

    // Initialize
    initial begin
        m_axi_awaddr = 0;
        m_axi_awprot = 3'b000;
        m_axi_awvalid = 0;
        m_axi_wdata = 0;
        m_axi_wstrb = {(DATA_WIDTH/8){1'b1}};
        m_axi_wvalid = 0;
        m_axi_bready = 0;
        m_axi_araddr = 0;
        m_axi_arprot = 3'b000;
        m_axi_arvalid = 0;
        m_axi_rready = 0;
        read_data = 0;
        read_resp = 0;
        write_resp = 0;
    end

    //-------------------------------------------------------------------------
    // Write Task
    //-------------------------------------------------------------------------
    task axi_write;
        input [ADDR_WIDTH-1:0] addr;
        input [DATA_WIDTH-1:0] data;
        begin
            // Address phase
            @(posedge clk);
            m_axi_awaddr <= addr;
            m_axi_awvalid <= 1'b1;
            m_axi_wdata <= data;
            m_axi_wstrb <= {(DATA_WIDTH/8){1'b1}};
            m_axi_wvalid <= 1'b1;

            // Wait for address ready
            fork
                begin
                    wait(m_axi_awready);
                    @(posedge clk);
                    m_axi_awvalid <= 1'b0;
                end
                begin
                    wait(m_axi_wready);
                    @(posedge clk);
                    m_axi_wvalid <= 1'b0;
                end
            join

            // Response phase
            m_axi_bready <= 1'b1;
            wait(m_axi_bvalid);
            @(posedge clk);
            write_resp <= m_axi_bresp;
            m_axi_bready <= 1'b0;

            @(posedge clk);
        end
    endtask

    //-------------------------------------------------------------------------
    // Read Task
    //-------------------------------------------------------------------------
    task axi_read;
        input [ADDR_WIDTH-1:0] addr;
        output [DATA_WIDTH-1:0] data;
        begin
            // Address phase
            @(posedge clk);
            m_axi_araddr <= addr;
            m_axi_arvalid <= 1'b1;

            // Wait for address ready
            wait(m_axi_arready);
            @(posedge clk);
            m_axi_arvalid <= 1'b0;

            // Data phase
            m_axi_rready <= 1'b1;
            wait(m_axi_rvalid);
            @(posedge clk);
            data = m_axi_rdata;
            read_data <= m_axi_rdata;
            read_resp <= m_axi_rresp;
            m_axi_rready <= 1'b0;

            @(posedge clk);
        end
    endtask

    //-------------------------------------------------------------------------
    // Write and verify task
    //-------------------------------------------------------------------------
    task axi_write_verify;
        input [ADDR_WIDTH-1:0] addr;
        input [DATA_WIDTH-1:0] data;
        reg [DATA_WIDTH-1:0] readback;
        begin
            axi_write(addr, data);
            axi_read(addr, readback);
            if (readback !== data) begin
                $display("ERROR: Write verify failed at addr 0x%h: wrote 0x%h, read 0x%h",
                         addr, data, readback);
            end
        end
    endtask

    //-------------------------------------------------------------------------
    // Poll register until condition met
    //-------------------------------------------------------------------------
    task axi_poll;
        input [ADDR_WIDTH-1:0] addr;
        input [DATA_WIDTH-1:0] mask;
        input [DATA_WIDTH-1:0] expected;
        input integer timeout_cycles;
        output reg timeout;
        reg [DATA_WIDTH-1:0] data;
        integer count;
        begin
            timeout = 0;
            count = 0;
            while (count < timeout_cycles) begin
                axi_read(addr, data);
                if ((data & mask) == expected) begin
                    timeout = 0;
                    return;
                end
                count = count + 1;
            end
            timeout = 1;
            $display("WARNING: Poll timeout at addr 0x%h after %0d cycles", addr, timeout_cycles);
        end
    endtask

endmodule
