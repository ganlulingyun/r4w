# R4W Lattice FPGA Build System
# Uses open-source toolchain: Yosys + nextpnr + IceStorm/Trellis
#
# trace:FR-0084 | ai:claude
#
# Targets:
#   make ice40      - Build for iCE40-HX8K
#   make ecp5       - Build for ECP5-25K
#   make prog_ice40 - Program iCE40 via FTDI
#   make prog_ecp5  - Program ECP5 via FTDI
#   make sim        - Run Icarus Verilog simulation
#   make clean      - Clean build artifacts
#
# Requirements:
#   - Yosys (synthesis)
#   - nextpnr-ice40 / nextpnr-ecp5 (place & route)
#   - IceStorm (iceprog for programming)
#   - Project Trellis (ecppack for ECP5)
#   - Icarus Verilog (simulation)

# Project name
PROJECT = r4w

# Directories
IP_DIR      = ../ip
DESIGN_DIR  = ../design
SIM_DIR     = ../sim
BUILD_DIR   = build
CONSTRAINTS = ../design/constraints

# Source files
COMMON_SRC = \
	$(IP_DIR)/r4w_spi_slave/r4w_spi_slave.v \
	$(IP_DIR)/r4w_nco/r4w_nco.v \
	$(IP_DIR)/r4w_chirp_gen/r4w_chirp_gen.v

ICE40_SRC = \
	$(COMMON_SRC) \
	$(DESIGN_DIR)/r4w_top_ice40.v

ECP5_SRC = \
	$(COMMON_SRC) \
	$(DESIGN_DIR)/r4w_top_ecp5.v

# Constraints
ICE40_PCF = $(CONSTRAINTS)/ice40_hx8k.pcf
ECP5_LPF  = $(CONSTRAINTS)/ecp5_25k.lpf

# Target device configurations
# iCE40-HX8K on iCEstick
ICE40_DEVICE = hx8k
ICE40_PACKAGE = ct256

# ECP5-25K on ULX3S or generic
ECP5_DEVICE = 25k
ECP5_PACKAGE = CABGA381
ECP5_SPEED = 6

# Yosys flags
YOSYS_FLAGS = -q

# nextpnr flags
NEXTPNR_ICE40_FLAGS = --hx8k --package $(ICE40_PACKAGE) --freq 48 --pcf-allow-unconstrained
NEXTPNR_ECP5_FLAGS = --$(ECP5_DEVICE) --package $(ECP5_PACKAGE) --speed $(ECP5_SPEED) --freq 100

# Default target
.PHONY: all
all: ice40

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# =============================================================================
# iCE40 Build Flow
# =============================================================================

.PHONY: ice40
ice40: $(BUILD_DIR)/$(PROJECT)_ice40.bin

# Synthesis
$(BUILD_DIR)/$(PROJECT)_ice40.json: $(ICE40_SRC) | $(BUILD_DIR)
	@echo "=== Synthesizing for iCE40 ==="
	yosys $(YOSYS_FLAGS) -p "read_verilog -DICE40 $(ICE40_SRC); synth_ice40 -top r4w_top_ice40 -json $@"

# Place and Route
$(BUILD_DIR)/$(PROJECT)_ice40.asc: $(BUILD_DIR)/$(PROJECT)_ice40.json $(ICE40_PCF)
	@echo "=== Place and Route for iCE40 ==="
	nextpnr-ice40 $(NEXTPNR_ICE40_FLAGS) --pcf $(ICE40_PCF) --json $< --asc $@

# Pack bitstream
$(BUILD_DIR)/$(PROJECT)_ice40.bin: $(BUILD_DIR)/$(PROJECT)_ice40.asc
	@echo "=== Packing iCE40 bitstream ==="
	icepack $< $@
	@echo "=== Build complete: $@ ==="

# Program iCE40
.PHONY: prog_ice40
prog_ice40: $(BUILD_DIR)/$(PROJECT)_ice40.bin
	@echo "=== Programming iCE40 ==="
	iceprog $<

# =============================================================================
# ECP5 Build Flow
# =============================================================================

.PHONY: ecp5
ecp5: $(BUILD_DIR)/$(PROJECT)_ecp5.bit

# Synthesis
$(BUILD_DIR)/$(PROJECT)_ecp5.json: $(ECP5_SRC) | $(BUILD_DIR)
	@echo "=== Synthesizing for ECP5 ==="
	yosys $(YOSYS_FLAGS) -p "read_verilog -DECP5 $(ECP5_SRC); synth_ecp5 -top r4w_top_ecp5 -json $@"

# Place and Route
$(BUILD_DIR)/$(PROJECT)_ecp5_routed.json: $(BUILD_DIR)/$(PROJECT)_ecp5.json $(ECP5_LPF)
	@echo "=== Place and Route for ECP5 ==="
	nextpnr-ecp5 $(NEXTPNR_ECP5_FLAGS) --lpf $(ECP5_LPF) --json $< --textcfg $(BUILD_DIR)/$(PROJECT)_ecp5.config

# Pack bitstream
$(BUILD_DIR)/$(PROJECT)_ecp5.bit: $(BUILD_DIR)/$(PROJECT)_ecp5.json $(ECP5_LPF)
	@echo "=== Place and Route for ECP5 ==="
	nextpnr-ecp5 $(NEXTPNR_ECP5_FLAGS) --lpf $(ECP5_LPF) --json $< --textcfg $(BUILD_DIR)/$(PROJECT)_ecp5.config
	@echo "=== Packing ECP5 bitstream ==="
	ecppack --compress --input $(BUILD_DIR)/$(PROJECT)_ecp5.config --bit $@
	@echo "=== Build complete: $@ ==="

# Program ECP5 (via FTDI/openFPGALoader)
.PHONY: prog_ecp5
prog_ecp5: $(BUILD_DIR)/$(PROJECT)_ecp5.bit
	@echo "=== Programming ECP5 ==="
	openFPGALoader -b ulx3s $<

# =============================================================================
# Simulation
# =============================================================================

SIM_TB = $(SIM_DIR)/tb_r4w_top_ice40.v

.PHONY: sim
sim: $(BUILD_DIR)/sim_ice40.vvp
	@echo "=== Running simulation ==="
	vvp $< +vcd=$(BUILD_DIR)/sim.vcd
	@echo "=== Simulation complete ==="

$(BUILD_DIR)/sim_ice40.vvp: $(ICE40_SRC) $(SIM_TB) | $(BUILD_DIR)
	@echo "=== Compiling testbench ==="
	iverilog -DSIMULATION -DICE40 -o $@ -s tb_r4w_top_ice40 $(ICE40_SRC) $(SIM_TB)

.PHONY: wave
wave: sim
	gtkwave $(BUILD_DIR)/sim.vcd &

# =============================================================================
# Linting
# =============================================================================

.PHONY: lint
lint: $(ICE40_SRC)
	@echo "=== Linting Verilog ==="
	verilator --lint-only -Wall $(ICE40_SRC)

# =============================================================================
# Resource Reports
# =============================================================================

.PHONY: report_ice40
report_ice40: $(BUILD_DIR)/$(PROJECT)_ice40.json
	@echo "=== iCE40 Resource Report ==="
	yosys -p "read_json $<; stat"

.PHONY: report_ecp5
report_ecp5: $(BUILD_DIR)/$(PROJECT)_ecp5.json
	@echo "=== ECP5 Resource Report ==="
	yosys -p "read_json $<; stat"

# =============================================================================
# Timing Analysis
# =============================================================================

.PHONY: timing_ice40
timing_ice40: $(BUILD_DIR)/$(PROJECT)_ice40.asc
	@echo "=== iCE40 Timing Report ==="
	icetime -d $(ICE40_DEVICE) -mtr $(BUILD_DIR)/timing_ice40.rpt $<
	cat $(BUILD_DIR)/timing_ice40.rpt

# =============================================================================
# Clean
# =============================================================================

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: distclean
distclean: clean
	find .. -name "*.pyc" -delete
	find .. -name "__pycache__" -delete

# =============================================================================
# Help
# =============================================================================

.PHONY: help
help:
	@echo "R4W Lattice FPGA Build System"
	@echo ""
	@echo "Build targets:"
	@echo "  ice40        - Build for iCE40-HX8K (default)"
	@echo "  ecp5         - Build for ECP5-25K"
	@echo ""
	@echo "Programming targets:"
	@echo "  prog_ice40   - Program iCE40 via iceprog"
	@echo "  prog_ecp5    - Program ECP5 via openFPGALoader"
	@echo ""
	@echo "Simulation/Debug:"
	@echo "  sim          - Run Icarus Verilog simulation"
	@echo "  wave         - Open waveform viewer"
	@echo "  lint         - Run Verilator linting"
	@echo ""
	@echo "Reports:"
	@echo "  report_ice40 - Show iCE40 resource usage"
	@echo "  report_ecp5  - Show ECP5 resource usage"
	@echo "  timing_ice40 - Show iCE40 timing report"
	@echo ""
	@echo "Cleanup:"
	@echo "  clean        - Remove build artifacts"
	@echo "  distclean    - Remove all generated files"
