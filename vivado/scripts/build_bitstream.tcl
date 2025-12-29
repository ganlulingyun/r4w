#-----------------------------------------------------------------------------
# R4W Bitstream Generation Script
#-----------------------------------------------------------------------------

set project_dir "[file dirname [info script]]/.."
set project_name "r4w_fpga"

# Open project
open_project ${project_dir}/project/${project_name}.xpr

# Run synthesis
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1

# Check synthesis status
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    puts "ERROR: Synthesis failed!"
    exit 1
}

# Run implementation
reset_run impl_1
launch_runs impl_1 -jobs 4
wait_on_run impl_1

# Check implementation status
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    puts "ERROR: Implementation failed!"
    exit 1
}

# Generate bitstream
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# Check bitstream status
set bit_file [glob -nocomplain ${project_dir}/project/${project_name}.runs/impl_1/*.bit]
if {$bit_file eq ""} {
    puts "ERROR: Bitstream generation failed!"
    exit 1
}

# Copy output files
file mkdir ${project_dir}/output
file copy -force $bit_file ${project_dir}/output/r4w_design.bit

# Generate hardware handoff for SDK/PetaLinux
write_hwdef -force -file ${project_dir}/output/r4w_design.hdf

puts "=============================================="
puts "Bitstream generated successfully!"
puts "Output: ${project_dir}/output/r4w_design.bit"
puts "=============================================="
