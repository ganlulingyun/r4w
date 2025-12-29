#-----------------------------------------------------------------------------
# R4W Block Design Creation Script
# Creates PS + R4W IP cores interconnect
#-----------------------------------------------------------------------------

# Create block design
create_bd_design "r4w_bd"

# Create Zynq PS
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0

# Apply PYNQ-Z2 board preset
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable" } \
    [get_bd_cells processing_system7_0]

# Enable FCLK_CLK0 (100 MHz)
set_property -dict [list \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
] [get_bd_cells processing_system7_0]

# Create AXI Interconnect
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_0
set_property CONFIG.NUM_MI {6} [get_bd_cells axi_interconnect_0]

# Connect PS to interconnect
connect_bd_intf_net [get_bd_intf_pins processing_system7_0/M_AXI_GP0] \
    [get_bd_intf_pins axi_interconnect_0/S00_AXI]

# Clock and reset connections
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/S00_ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/M00_ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/M01_ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/M02_ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/M03_ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/M04_ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins axi_interconnect_0/M05_ACLK]

# Create reset infrastructure
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_0
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
    [get_bd_pins proc_sys_reset_0/slowest_sync_clk]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_RESET0_N] \
    [get_bd_pins proc_sys_reset_0/ext_reset_in]

# Connect resets
connect_bd_net [get_bd_pins proc_sys_reset_0/interconnect_aresetn] \
    [get_bd_pins axi_interconnect_0/ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/S00_ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/M00_ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/M01_ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/M02_ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/M03_ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/M04_ARESETN]
connect_bd_net [get_bd_pins proc_sys_reset_0/peripheral_aresetn] \
    [get_bd_pins axi_interconnect_0/M05_ARESETN]

#-----------------------------------------------------------------------------
# Instantiate R4W IP Cores
#-----------------------------------------------------------------------------

# Note: In a real build, you would package these as Vivado IP first.
# For now, we add them as RTL modules with manual AXI connections.

# Create hierarchy for R4W cores
# create_bd_cell -type module -reference r4w_fft_axi r4w_fft_0
# create_bd_cell -type module -reference r4w_fir_axi r4w_fir_0
# create_bd_cell -type module -reference r4w_chirp_gen_axi r4w_chirp_gen_0
# create_bd_cell -type module -reference r4w_chirp_corr_axi r4w_chirp_corr_0
# create_bd_cell -type module -reference r4w_nco_axi r4w_nco_0

#-----------------------------------------------------------------------------
# Address Map
#-----------------------------------------------------------------------------

# Assign addresses (matching Rust driver config.rs)
# M00: r4w_fft       @ 0x4000_0000
# M01: r4w_fir       @ 0x4001_0000
# M02: r4w_chirp_gen @ 0x4002_0000
# M03: r4w_chirp_corr@ 0x4003_0000
# M04: r4w_nco       @ 0x4004_0000
# M05: Reserved for DMA @ 0x4040_0000

# create_bd_addr_seg -range 0x10000 -offset 0x40000000 \
#     [get_bd_addr_spaces processing_system7_0/Data] \
#     [get_bd_addr_segs r4w_fft_0/S_AXI/reg0] SEG_r4w_fft

# create_bd_addr_seg -range 0x10000 -offset 0x40010000 \
#     [get_bd_addr_spaces processing_system7_0/Data] \
#     [get_bd_addr_segs r4w_fir_0/S_AXI/reg0] SEG_r4w_fir

# create_bd_addr_seg -range 0x10000 -offset 0x40020000 \
#     [get_bd_addr_spaces processing_system7_0/Data] \
#     [get_bd_addr_segs r4w_chirp_gen_0/S_AXI/reg0] SEG_r4w_chirp_gen

# create_bd_addr_seg -range 0x10000 -offset 0x40030000 \
#     [get_bd_addr_spaces processing_system7_0/Data] \
#     [get_bd_addr_segs r4w_chirp_corr_0/S_AXI/reg0] SEG_r4w_chirp_corr

# create_bd_addr_seg -range 0x10000 -offset 0x40040000 \
#     [get_bd_addr_spaces processing_system7_0/Data] \
#     [get_bd_addr_segs r4w_nco_0/S_AXI/reg0] SEG_r4w_nco

#-----------------------------------------------------------------------------
# Validate and Save
#-----------------------------------------------------------------------------

regenerate_bd_layout
validate_bd_design
save_bd_design

puts "Block design created successfully!"
