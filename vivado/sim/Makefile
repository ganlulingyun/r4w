#------------------------------------------------------------------------------
# Makefile for R4W FPGA IP Core Testbenches
# R4W FPGA Acceleration Layer
#
# Supports:
#   - Icarus Verilog (iverilog) - Free, open source
#   - Verilator (optional) - Fast simulation
#   - Vivado xsim (if installed) - Xilinx simulator
#
# Usage:
#   make all           - Run all testbenches
#   make nco           - Run NCO testbench
#   make chirp_gen     - Run Chirp Generator testbench
#   make fft           - Run FFT testbench
#   make fir           - Run FIR testbench
#   make chirp_corr    - Run Chirp Correlator testbench
#   make clean         - Clean generated files
#   make waves_<test>  - Open waveform viewer
#------------------------------------------------------------------------------

# Simulator selection (iverilog, verilator, xsim)
SIM ?= iverilog

# Directories
IP_DIR := ../ip
SIM_DIR := .
WORK_DIR := work
VCD_DIR := vcd

# Common source files
COMMON_SRCS := \
    $(IP_DIR)/common/axi_lite_slave.v \
    $(IP_DIR)/common/iq_pack.v \
    axi_lite_master_bfm.v

# IP core sources
NCO_SRCS := \
    $(IP_DIR)/r4w_nco/r4w_nco.v \
    $(IP_DIR)/r4w_nco/r4w_nco_axi.v

CHIRP_GEN_SRCS := \
    $(IP_DIR)/r4w_chirp_gen/r4w_chirp_gen.v \
    $(IP_DIR)/r4w_chirp_gen/r4w_chirp_gen_axi.v

FFT_SRCS := \
    $(IP_DIR)/r4w_fft/r4w_fft.v \
    $(IP_DIR)/r4w_fft/r4w_fft_axi.v

FIR_SRCS := \
    $(IP_DIR)/r4w_fir/r4w_fir.v \
    $(IP_DIR)/r4w_fir/r4w_fir_axi.v

CHIRP_CORR_SRCS := \
    $(IP_DIR)/r4w_chirp_corr/r4w_chirp_corr.v \
    $(IP_DIR)/r4w_chirp_corr/r4w_chirp_corr_axi.v

DMA_SRCS := \
    $(IP_DIR)/r4w_dma/r4w_dma.v \
    $(IP_DIR)/r4w_dma/r4w_dma_axi.v

# Testbench files
TB_NCO := tb_r4w_nco.v
TB_CHIRP_GEN := tb_r4w_chirp_gen.v
TB_FFT := tb_r4w_fft.v
TB_FIR := tb_r4w_fir.v
TB_CHIRP_CORR := tb_r4w_chirp_corr.v
TB_DMA := tb_r4w_dma.v

# Include paths
INC_DIRS := -I$(SIM_DIR) -I$(IP_DIR)/common

# Icarus Verilog flags
IVERILOG := iverilog
IVERILOG_FLAGS := -Wall -g2012 $(INC_DIRS)
VVP := vvp
VVP_FLAGS := -n

# Verilator flags (optional)
VERILATOR := verilator
VERILATOR_FLAGS := --cc --exe --build -Wall $(INC_DIRS)

# Vivado xsim flags (optional)
XVLOG := xvlog
XELAB := xelab
XSIM := xsim
XSIM_FLAGS := -R

# Waveform viewer
WAVEFORM_VIEWER ?= gtkwave

#------------------------------------------------------------------------------
# Default target
#------------------------------------------------------------------------------

.PHONY: all clean help nco chirp_gen fft fir chirp_corr dma waves

all: nco chirp_gen fft fir chirp_corr dma
	@echo ""
	@echo "=============================================="
	@echo "All testbenches completed!"
	@echo "=============================================="

#------------------------------------------------------------------------------
# Create work directory
#------------------------------------------------------------------------------

$(WORK_DIR):
	mkdir -p $(WORK_DIR)

$(VCD_DIR):
	mkdir -p $(VCD_DIR)

#------------------------------------------------------------------------------
# NCO Testbench
#------------------------------------------------------------------------------

nco: $(WORK_DIR)
	@echo ""
	@echo "=============================================="
	@echo "Running NCO Testbench"
	@echo "=============================================="
ifeq ($(SIM),iverilog)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $(WORK_DIR)/tb_nco \
		$(COMMON_SRCS) $(NCO_SRCS) $(TB_NCO)
	cd $(WORK_DIR) && $(VVP) $(VVP_FLAGS) tb_nco
	@if [ -f tb_r4w_nco.vcd ]; then mv tb_r4w_nco.vcd $(VCD_DIR)/ 2>/dev/null || true; fi
else ifeq ($(SIM),xsim)
	$(XVLOG) $(COMMON_SRCS) $(NCO_SRCS) $(TB_NCO)
	$(XELAB) tb_r4w_nco -s tb_nco_sim
	$(XSIM) tb_nco_sim $(XSIM_FLAGS)
endif

#------------------------------------------------------------------------------
# Chirp Generator Testbench
#------------------------------------------------------------------------------

chirp_gen: $(WORK_DIR)
	@echo ""
	@echo "=============================================="
	@echo "Running Chirp Generator Testbench"
	@echo "=============================================="
ifeq ($(SIM),iverilog)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $(WORK_DIR)/tb_chirp_gen \
		$(COMMON_SRCS) $(NCO_SRCS) $(CHIRP_GEN_SRCS) $(TB_CHIRP_GEN)
	cd $(WORK_DIR) && $(VVP) $(VVP_FLAGS) tb_chirp_gen
	@if [ -f tb_r4w_chirp_gen.vcd ]; then mv tb_r4w_chirp_gen.vcd $(VCD_DIR)/ 2>/dev/null || true; fi
else ifeq ($(SIM),xsim)
	$(XVLOG) $(COMMON_SRCS) $(NCO_SRCS) $(CHIRP_GEN_SRCS) $(TB_CHIRP_GEN)
	$(XELAB) tb_r4w_chirp_gen -s tb_chirp_gen_sim
	$(XSIM) tb_chirp_gen_sim $(XSIM_FLAGS)
endif

#------------------------------------------------------------------------------
# FFT Testbench
#------------------------------------------------------------------------------

fft: $(WORK_DIR)
	@echo ""
	@echo "=============================================="
	@echo "Running FFT Testbench"
	@echo "=============================================="
ifeq ($(SIM),iverilog)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $(WORK_DIR)/tb_fft \
		$(COMMON_SRCS) $(FFT_SRCS) $(TB_FFT)
	cd $(WORK_DIR) && $(VVP) $(VVP_FLAGS) tb_fft
	@if [ -f tb_r4w_fft.vcd ]; then mv tb_r4w_fft.vcd $(VCD_DIR)/ 2>/dev/null || true; fi
else ifeq ($(SIM),xsim)
	$(XVLOG) $(COMMON_SRCS) $(FFT_SRCS) $(TB_FFT)
	$(XELAB) tb_r4w_fft -s tb_fft_sim
	$(XSIM) tb_fft_sim $(XSIM_FLAGS)
endif

#------------------------------------------------------------------------------
# FIR Testbench
#------------------------------------------------------------------------------

fir: $(WORK_DIR)
	@echo ""
	@echo "=============================================="
	@echo "Running FIR Testbench"
	@echo "=============================================="
ifeq ($(SIM),iverilog)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $(WORK_DIR)/tb_fir \
		$(COMMON_SRCS) $(FIR_SRCS) $(TB_FIR)
	cd $(WORK_DIR) && $(VVP) $(VVP_FLAGS) tb_fir
	@if [ -f tb_r4w_fir.vcd ]; then mv tb_r4w_fir.vcd $(VCD_DIR)/ 2>/dev/null || true; fi
else ifeq ($(SIM),xsim)
	$(XVLOG) $(COMMON_SRCS) $(FIR_SRCS) $(TB_FIR)
	$(XELAB) tb_r4w_fir -s tb_fir_sim
	$(XSIM) tb_fir_sim $(XSIM_FLAGS)
endif

#------------------------------------------------------------------------------
# Chirp Correlator Testbench
#------------------------------------------------------------------------------

chirp_corr: $(WORK_DIR)
	@echo ""
	@echo "=============================================="
	@echo "Running Chirp Correlator Testbench"
	@echo "=============================================="
ifeq ($(SIM),iverilog)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $(WORK_DIR)/tb_chirp_corr \
		$(COMMON_SRCS) $(NCO_SRCS) $(FFT_SRCS) $(CHIRP_CORR_SRCS) $(TB_CHIRP_CORR)
	cd $(WORK_DIR) && $(VVP) $(VVP_FLAGS) tb_chirp_corr
	@if [ -f tb_r4w_chirp_corr.vcd ]; then mv tb_r4w_chirp_corr.vcd $(VCD_DIR)/ 2>/dev/null || true; fi
else ifeq ($(SIM),xsim)
	$(XVLOG) $(COMMON_SRCS) $(NCO_SRCS) $(FFT_SRCS) $(CHIRP_CORR_SRCS) $(TB_CHIRP_CORR)
	$(XELAB) tb_r4w_chirp_corr -s tb_chirp_corr_sim
	$(XSIM) tb_chirp_corr_sim $(XSIM_FLAGS)
endif

#------------------------------------------------------------------------------
# DMA Testbench
#------------------------------------------------------------------------------

dma: $(WORK_DIR)
	@echo ""
	@echo "=============================================="
	@echo "Running DMA Controller Testbench"
	@echo "=============================================="
ifeq ($(SIM),iverilog)
	$(IVERILOG) $(IVERILOG_FLAGS) -o $(WORK_DIR)/tb_dma \
		$(COMMON_SRCS) $(DMA_SRCS) $(TB_DMA)
	cd $(WORK_DIR) && $(VVP) $(VVP_FLAGS) tb_dma
	@if [ -f tb_r4w_dma.vcd ]; then mv tb_r4w_dma.vcd $(VCD_DIR)/ 2>/dev/null || true; fi
else ifeq ($(SIM),xsim)
	$(XVLOG) $(COMMON_SRCS) $(DMA_SRCS) $(TB_DMA)
	$(XELAB) tb_r4w_dma -s tb_dma_sim
	$(XSIM) tb_dma_sim $(XSIM_FLAGS)
endif

#------------------------------------------------------------------------------
# Waveform viewing
#------------------------------------------------------------------------------

waves_nco:
	$(WAVEFORM_VIEWER) $(VCD_DIR)/tb_r4w_nco.vcd &

waves_chirp_gen:
	$(WAVEFORM_VIEWER) $(VCD_DIR)/tb_r4w_chirp_gen.vcd &

waves_fft:
	$(WAVEFORM_VIEWER) $(VCD_DIR)/tb_r4w_fft.vcd &

waves_fir:
	$(WAVEFORM_VIEWER) $(VCD_DIR)/tb_r4w_fir.vcd &

waves_chirp_corr:
	$(WAVEFORM_VIEWER) $(VCD_DIR)/tb_r4w_chirp_corr.vcd &

waves_dma:
	$(WAVEFORM_VIEWER) $(VCD_DIR)/tb_r4w_dma.vcd &

#------------------------------------------------------------------------------
# Lint check (static analysis)
#------------------------------------------------------------------------------

lint:
	@echo "Running lint check on all IP cores..."
	$(IVERILOG) $(IVERILOG_FLAGS) -Wall -o /dev/null \
		$(COMMON_SRCS) $(NCO_SRCS) $(CHIRP_GEN_SRCS) \
		$(FFT_SRCS) $(FIR_SRCS) $(CHIRP_CORR_SRCS) $(DMA_SRCS) 2>&1 || true
	@echo "Lint check complete"

#------------------------------------------------------------------------------
# Clean
#------------------------------------------------------------------------------

clean:
	rm -rf $(WORK_DIR)
	rm -rf $(VCD_DIR)
	rm -f *.vcd
	rm -f *.log
	rm -f *.wdb
	rm -f xsim.dir
	rm -f .Xil
	rm -f webtalk*.jou webtalk*.log
	rm -f xvlog*.pb xvlog*.log
	rm -f xelab*.pb xelab*.log

#------------------------------------------------------------------------------
# Help
#------------------------------------------------------------------------------

help:
	@echo "R4W FPGA Testbench Makefile"
	@echo ""
	@echo "Usage: make [target] [SIM=<simulator>]"
	@echo ""
	@echo "Targets:"
	@echo "  all          - Run all testbenches"
	@echo "  nco          - Run NCO testbench"
	@echo "  chirp_gen    - Run Chirp Generator testbench"
	@echo "  fft          - Run FFT testbench"
	@echo "  fir          - Run FIR testbench"
	@echo "  chirp_corr   - Run Chirp Correlator testbench"
	@echo "  dma          - Run DMA Controller testbench"
	@echo "  lint         - Run lint checks on all IP cores"
	@echo "  clean        - Remove generated files"
	@echo "  help         - Show this help message"
	@echo ""
	@echo "Waveform viewing:"
	@echo "  waves_nco         - View NCO waveforms"
	@echo "  waves_chirp_gen   - View Chirp Generator waveforms"
	@echo "  waves_fft         - View FFT waveforms"
	@echo "  waves_fir         - View FIR waveforms"
	@echo "  waves_chirp_corr  - View Chirp Correlator waveforms"
	@echo "  waves_dma         - View DMA Controller waveforms"
	@echo ""
	@echo "Simulators (SIM=):"
	@echo "  iverilog     - Icarus Verilog (default, free)"
	@echo "  xsim         - Xilinx Vivado simulator"
	@echo ""
	@echo "Environment variables:"
	@echo "  SIM              - Simulator to use (default: iverilog)"
	@echo "  WAVEFORM_VIEWER  - Waveform viewer (default: gtkwave)"
	@echo ""
	@echo "Examples:"
	@echo "  make nco                    - Run NCO test with Icarus"
	@echo "  make all SIM=xsim           - Run all tests with Vivado"
	@echo "  make waves_fft              - View FFT waveforms in GTKWave"
