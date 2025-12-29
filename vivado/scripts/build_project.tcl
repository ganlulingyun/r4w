#-----------------------------------------------------------------------------
# R4W Vivado Project Creation Script
# Creates Vivado project for PYNQ-Z2 (Zynq-7020)
#-----------------------------------------------------------------------------

# Set project parameters
set project_name "r4w_fpga"
set project_dir "[file dirname [info script]]/.."
set ip_repo_path "${project_dir}/ip"
set part "xc7z020clg400-1"
set board "tul.com.tw:pynq-z2:part0:1.0"

# Create project
create_project ${project_name} ${project_dir}/project -part ${part} -force

# Set board
set_property board_part ${board} [current_project]

# Add IP repository
set_property ip_repo_paths ${ip_repo_path} [current_project]
update_ip_catalog

# Add source files
add_files -fileset sources_1 [glob ${ip_repo_path}/*/*.v]
add_files -fileset sources_1 [glob ${ip_repo_path}/common/*.v]

# Add constraints
add_files -fileset constrs_1 ${project_dir}/design/constraints/pynq_z2.xdc

# Create block design
source ${project_dir}/scripts/build_bd.tcl

# Generate wrapper
make_wrapper -files [get_files */r4w_bd.bd] -top
add_files -norecurse [get_files */hdl/r4w_bd_wrapper.v]

# Set top module
set_property top r4w_bd_wrapper [current_fileset]

# Update compile order
update_compile_order -fileset sources_1

puts "Project created successfully!"
puts "Run build_bitstream.tcl to generate bitstream."
