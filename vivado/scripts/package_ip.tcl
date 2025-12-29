#-----------------------------------------------------------------------------
# R4W IP Core Packaging Script
# Packages each R4W core as a Vivado IP for use in block designs
#-----------------------------------------------------------------------------

set project_dir "[file dirname [info script]]/.."
set ip_dir "${project_dir}/ip"

#-----------------------------------------------------------------------------
# Package r4w_nco
#-----------------------------------------------------------------------------
proc package_r4w_nco {ip_dir} {
    set core_name "r4w_nco"
    set core_dir "${ip_dir}/${core_name}"

    ipx::package_project -root_dir ${core_dir} -vendor anthropic.com -library r4w \
        -taxonomy /DSP -import_files -set_current false
    ipx::unload_core ${core_dir}/component.xml
    ipx::edit_ip_in_project -upgrade true -name tmp_edit_project \
        -directory ${core_dir}/tmp ${core_dir}/component.xml

    set_property display_name "R4W NCO" [ipx::current_core]
    set_property description "Numerically Controlled Oscillator for R4W" [ipx::current_core]
    set_property vendor_display_name "Anthropic" [ipx::current_core]
    set_property company_url "https://anthropic.com" [ipx::current_core]
    set_property core_revision 1 [ipx::current_core]

    ipx::create_xgui_files [ipx::current_core]
    ipx::update_checksums [ipx::current_core]
    ipx::save_core [ipx::current_core]
    close_project -delete

    puts "Packaged: ${core_name}"
}

#-----------------------------------------------------------------------------
# Package r4w_chirp_gen
#-----------------------------------------------------------------------------
proc package_r4w_chirp_gen {ip_dir} {
    set core_name "r4w_chirp_gen"
    set core_dir "${ip_dir}/${core_name}"

    ipx::package_project -root_dir ${core_dir} -vendor anthropic.com -library r4w \
        -taxonomy /DSP -import_files -set_current false
    ipx::unload_core ${core_dir}/component.xml
    ipx::edit_ip_in_project -upgrade true -name tmp_edit_project \
        -directory ${core_dir}/tmp ${core_dir}/component.xml

    set_property display_name "R4W LoRa Chirp Generator" [ipx::current_core]
    set_property description "LoRa Chirp Signal Generator for R4W" [ipx::current_core]
    set_property vendor_display_name "Anthropic" [ipx::current_core]

    ipx::create_xgui_files [ipx::current_core]
    ipx::update_checksums [ipx::current_core]
    ipx::save_core [ipx::current_core]
    close_project -delete

    puts "Packaged: ${core_name}"
}

#-----------------------------------------------------------------------------
# Package r4w_chirp_corr
#-----------------------------------------------------------------------------
proc package_r4w_chirp_corr {ip_dir} {
    set core_name "r4w_chirp_corr"
    set core_dir "${ip_dir}/${core_name}"

    ipx::package_project -root_dir ${core_dir} -vendor anthropic.com -library r4w \
        -taxonomy /DSP -import_files -set_current false
    ipx::unload_core ${core_dir}/component.xml
    ipx::edit_ip_in_project -upgrade true -name tmp_edit_project \
        -directory ${core_dir}/tmp ${core_dir}/component.xml

    set_property display_name "R4W LoRa Chirp Correlator" [ipx::current_core]
    set_property description "LoRa Chirp Correlator/Demodulator for R4W" [ipx::current_core]
    set_property vendor_display_name "Anthropic" [ipx::current_core]

    ipx::create_xgui_files [ipx::current_core]
    ipx::update_checksums [ipx::current_core]
    ipx::save_core [ipx::current_core]
    close_project -delete

    puts "Packaged: ${core_name}"
}

#-----------------------------------------------------------------------------
# Package r4w_fft
#-----------------------------------------------------------------------------
proc package_r4w_fft {ip_dir} {
    set core_name "r4w_fft"
    set core_dir "${ip_dir}/${core_name}"

    ipx::package_project -root_dir ${core_dir} -vendor anthropic.com -library r4w \
        -taxonomy /DSP -import_files -set_current false
    ipx::unload_core ${core_dir}/component.xml
    ipx::edit_ip_in_project -upgrade true -name tmp_edit_project \
        -directory ${core_dir}/tmp ${core_dir}/component.xml

    set_property display_name "R4W FFT" [ipx::current_core]
    set_property description "FFT/IFFT Processor for R4W" [ipx::current_core]
    set_property vendor_display_name "Anthropic" [ipx::current_core]

    ipx::create_xgui_files [ipx::current_core]
    ipx::update_checksums [ipx::current_core]
    ipx::save_core [ipx::current_core]
    close_project -delete

    puts "Packaged: ${core_name}"
}

#-----------------------------------------------------------------------------
# Package r4w_fir
#-----------------------------------------------------------------------------
proc package_r4w_fir {ip_dir} {
    set core_name "r4w_fir"
    set core_dir "${ip_dir}/${core_name}"

    ipx::package_project -root_dir ${core_dir} -vendor anthropic.com -library r4w \
        -taxonomy /DSP -import_files -set_current false
    ipx::unload_core ${core_dir}/component.xml
    ipx::edit_ip_in_project -upgrade true -name tmp_edit_project \
        -directory ${core_dir}/tmp ${core_dir}/component.xml

    set_property display_name "R4W FIR Filter" [ipx::current_core]
    set_property description "FIR Filter with Reloadable Coefficients for R4W" [ipx::current_core]
    set_property vendor_display_name "Anthropic" [ipx::current_core]

    ipx::create_xgui_files [ipx::current_core]
    ipx::update_checksums [ipx::current_core]
    ipx::save_core [ipx::current_core]
    close_project -delete

    puts "Packaged: ${core_name}"
}

#-----------------------------------------------------------------------------
# Main
#-----------------------------------------------------------------------------
puts "Packaging R4W IP cores..."

# Note: These will fail without proper project context
# In practice, run from Vivado with project open

# package_r4w_nco $ip_dir
# package_r4w_chirp_gen $ip_dir
# package_r4w_chirp_corr $ip_dir
# package_r4w_fft $ip_dir
# package_r4w_fir $ip_dir

puts "=============================================="
puts "IP packaging script ready."
puts "Run from Vivado: source package_ip.tcl"
puts "=============================================="
