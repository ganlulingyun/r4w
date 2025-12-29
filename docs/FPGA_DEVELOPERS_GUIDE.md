# FPGA Developer's Guide

**R4W - Rust for Waveforms**
*Comprehensive Guide for FPGA Engineers*

---

## Table of Contents

1. [Introduction](#introduction)
2. [Architecture Overview](#architecture-overview)
3. [Supported Platforms](#supported-platforms)
4. [IP Core Library](#ip-core-library)
5. [Register Maps](#register-maps)
6. [AXI Interface Specifications](#axi-interface-specifications)
7. [Xilinx Zynq Integration](#xilinx-zynq-integration)
8. [Lattice FPGA Integration](#lattice-fpga-integration)
9. [Building and Synthesis](#building-and-synthesis)
10. [Verification and Testing](#verification-and-testing)
11. [Software Driver Interface](#software-driver-interface)
12. [Performance Optimization](#performance-optimization)
13. [Adding New IP Cores](#adding-new-ip-cores)
14. [Debugging and Troubleshooting](#debugging-and-troubleshooting)
15. [Resource Utilization](#resource-utilization)
16. [Collaboration with Software Developers](#collaboration-with-software-developers)

---

## Introduction

### Purpose

This guide is designed for FPGA engineers integrating R4W (Rust for Waveforms) hardware acceleration into SDR systems. It provides detailed technical specifications, register maps, interface protocols, and integration procedures for implementing DSP accelerators on Xilinx Zynq and Lattice FPGA platforms.

### Prerequisites

- Proficiency in Verilog/SystemVerilog
- Understanding of AXI4-Lite and AXI4-Stream protocols
- Experience with FPGA synthesis (Vivado or Yosys/nextpnr)
- Familiarity with DSP concepts (FFT, FIR, NCO, modulation)

### What R4W Provides

R4W provides a complete FPGA acceleration layer:

| Component | Description |
|-----------|-------------|
| **IP Cores** | Pre-built DSP blocks (FFT, FIR, NCO, chirp, correlator, DMA) |
| **AXI Wrappers** | Standard AXI4-Lite control interfaces |
| **Streaming** | AXI4-Stream data paths for high-throughput |
| **Rust Drivers** | Memory-mapped I/O drivers for Linux |
| **Simulation** | Software FPGA simulator for development without hardware |
| **Testbenches** | Verilog testbenches for verification |

### Your Responsibilities

As an FPGA developer, you are responsible for:

1. **Platform Integration** - Instantiating IP cores in your block design
2. **Address Mapping** - Assigning base addresses to IP cores
3. **Clock Domain Crossing** - Managing clock domains if different from system
4. **Resource Optimization** - Meeting timing and resource constraints
5. **Custom IP** - Developing waveform-specific acceleration blocks
6. **Bitstream Generation** - Producing deployable bitstreams
7. **Verification** - Ensuring hardware matches software expectations

---

## Architecture Overview

### System Block Diagram

```
┌───────────────────────────────────────────────────────────────────────────┐
│                         R4W FPGA Acceleration Layer                       │
├───────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────────┐  │
│  │                      ZYNQ PS (ARM Cortex-A9/A53)                    │  │
│  │                                                                     │  │
│  │   ┌────────────────┐    ┌────────────────┐    ┌────────────────┐    │  │
│  │   │   R4W Rust     │    │  Linux Kernel  │    │   /dev/mem     │    │  │
│  │   │   Application  │───►│   (UIO/devmem) │───►│   /dev/uio*    │    │  │
│  │   └────────────────┘    └────────────────┘    └───────┬────────┘    │  │
│  │                                                       │             │  │
│  └───────────────────────────────────────────────────────┼─────────────┘  │
│                                                          │                │
│                          AXI Interconnect                │                │
│  ┌───────────────────────────────────────────────────────┼─────────────┐  │
│  │                                                       │             │  │
│  │    AXI-Lite (Control/Status)    AXI-Stream (Data)     │             │  │
│  │           │                           │               │             │  │
│  └───────────┼───────────────────────────┼───────────────┼─────────────┘  │
│              │                           │               │                │
│  ┌───────────┴───────────────────────────┴───────────────┴─────────────┐  │
│  │                        FPGA Fabric (PL)                             │  │
│  │                                                                     │  │
│  │   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐   │  │
│  │   │  r4w_   │  │  r4w_   │  │  r4w_   │  │  r4w_   │  │  r4w_   │   │  │
│  │   │  fft    │  │  fir    │  │  nco    │  │ chirp   │  │  dma    │   │  │
│  │   │         │  │         │  │         │  │  gen    │  │         │   │  │
│  │   └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘  └────┬────┘   │  │
│  │        │            │            │            │            │        │  │
│  │   ┌────┴────────────┴────────────┴────────────┴────────────┴────┐   │  │
│  │   │                    AXI-Stream Router                        │   │  │
│  │   └─────────────────────────────────────────────────────────────┘   │  │
│  │                                                                     │  │
│  └─────────────────────────────────────────────────────────────────────┘  │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘
```

### Data Flow

1. **Software → Hardware**: Rust application writes samples to DMA buffer or directly to IP core registers
2. **Processing**: FPGA fabric performs DSP operations (FFT, filtering, modulation)
3. **Hardware → Software**: Results read back via DMA or register polling

### Interface Types

| Interface | Use Case | Bandwidth | Latency |
|-----------|----------|-----------|---------|
| AXI-Lite | Control registers, status, configuration | Low | Low |
| AXI-Stream | Sample data, continuous streaming | High | Low |
| DMA | Bulk transfers, buffer management | High | Medium |
| Register | Single sample, debugging | Very Low | Very Low |

---

## Supported Platforms

### Xilinx Zynq-7000 Series

| Feature | Specification |
|---------|---------------|
| **Target Devices** | xc7z010, xc7z020, xc7z030, xc7z045 |
| **Toolchain** | Vivado 2022.2+ |
| **AXI Interface** | AXI4-Lite (32-bit), AXI4-Stream |
| **Memory Access** | `/dev/mem`, `/dev/uio*` |
| **Typical Clock** | 100 MHz fabric, 667 MHz PS |

### Xilinx Zynq UltraScale+ MPSoC

| Feature | Specification |
|---------|---------------|
| **Target Devices** | xczu3eg, xczu7ev, xczu9eg |
| **Toolchain** | Vivado 2022.2+ |
| **AXI Interface** | AXI4-Lite (32/64-bit), AXI4-Stream |
| **Memory Access** | `/dev/mem`, `/dev/uio*`, CMA |
| **Typical Clock** | 300 MHz fabric, 1.2 GHz PS |

### Lattice iCE40

| Feature | Specification |
|---------|---------------|
| **Target Devices** | iCE40-HX8K, iCE40-LP8K |
| **Toolchain** | Yosys + nextpnr-ice40 + IceStorm |
| **Interface** | SPI slave (custom protocol) |
| **Memory Access** | FTDI SPI or GPIO bit-bang |
| **Typical Clock** | 12-48 MHz |

### Lattice ECP5

| Feature | Specification |
|---------|---------------|
| **Target Devices** | LFE5U-25F, LFE5U-45F, LFE5U-85F |
| **Toolchain** | Yosys + nextpnr-ecp5 + Trellis |
| **Interface** | SPI slave (custom protocol) |
| **Memory Access** | FTDI SPI or board-specific |
| **Typical Clock** | 25-100 MHz (PLL multiplied) |

---

## IP Core Library

### Available IP Cores

| IP Core | File | ID | Description |
|---------|------|-----|-------------|
| `r4w_fft` | `vivado/ip/r4w_fft/` | 0x52345746 ("R4WF") | FFT/IFFT up to 4096 points |
| `r4w_fir` | `vivado/ip/r4w_fir/` | 0x52344649 ("R4FI") | FIR filter up to 256 taps |
| `r4w_nco` | `vivado/ip/r4w_nco/` | 0x5234494F ("R4IO") | NCO with CORDIC or LUT |
| `r4w_chirp_gen` | `vivado/ip/r4w_chirp_gen/` | 0x52344347 ("R4CG") | LoRa chirp generator |
| `r4w_chirp_corr` | `vivado/ip/r4w_chirp_corr/` | 0x52344343 ("R4CC") | Chirp correlator/demodulator |
| `r4w_dma` | `vivado/ip/r4w_dma/` | 0x52344D41 ("R4MA") | DMA controller |

### IP Core Parameters

All IP cores share common parameters:

```verilog
parameter C_S_AXI_DATA_WIDTH = 32    // AXI data width (always 32)
parameter C_S_AXI_ADDR_WIDTH = 8     // Register space size (2^N bytes)
```

Core-specific parameters:

| Core | Parameter | Default | Description |
|------|-----------|---------|-------------|
| r4w_fft | `MAX_FFT_SIZE` | 1024 | Maximum FFT size (power of 2) |
| r4w_fft | `DATA_WIDTH` | 16 | I/Q sample width |
| r4w_fir | `MAX_TAPS` | 256 | Maximum filter taps |
| r4w_fir | `COEF_WIDTH` | 16 | Coefficient bit width |
| r4w_nco | `PHASE_WIDTH` | 32 | Phase accumulator width |
| r4w_nco | `OUTPUT_WIDTH` | 16 | Output sample width |
| r4w_nco | `CORDIC_STAGES` | 16 | CORDIC pipeline stages |
| r4w_chirp_gen | `MAX_SF` | 12 | Maximum spreading factor |

---

## Register Maps

### Common Register Layout

All IP cores follow a consistent register layout:

| Offset | Name | Access | Description |
|--------|------|--------|-------------|
| 0x00 | CTRL | R/W | Control register |
| 0x04 | CONFIG | R/W | Configuration (core-specific) |
| 0x08 | STATUS | R/O | Status register |
| 0x1C | VERSION | R/O | IP core version |
| 0x20 | ID | R/O | IP core identifier |

### r4w_fft Register Map

Base address: Configurable (default 0x43C0_0000 on Zynq)

| Offset | Name | Bits | Access | Description |
|--------|------|------|--------|-------------|
| 0x00 | CTRL | [0] | R/W | START - Begin FFT/IFFT |
| | | [1] | R/W | INVERSE - 1=IFFT, 0=FFT |
| | | [31] | R/W | RESET - Soft reset (auto-clear) |
| 0x04 | SIZE | [3:0] | R/W | log2(N): 6=64, 7=128, 8=256, 9=512, 10=1024 |
| 0x08 | STATUS | [0] | R/O | DONE - Operation complete |
| | | [1] | R/O | BUSY - Operation in progress |
| | | [2] | R/O | ERROR - Error occurred |
| 0x10 | DATA_IN | [31:16] | W/O | I sample (signed 16-bit) |
| | | [15:0] | W/O | Q sample (signed 16-bit) |
| 0x14 | DATA_OUT | [31:16] | R/O | I result (signed 16-bit) |
| | | [15:0] | R/O | Q result (signed 16-bit) |
| 0x1C | VERSION | [31:0] | R/O | Version (major.minor.patch) |
| 0x20 | ID | [31:0] | R/O | 0x52345746 ("R4WF") |

**Usage Example (C/Rust pseudo-code):**

```rust
// Configure FFT size (256 points)
write_reg(base + 0x04, 8);  // log2(256) = 8

// Write input samples
for sample in input_samples {
    let packed = ((sample.i as u32) << 16) | (sample.q as u16 as u32);
    write_reg(base + 0x10, packed);
}

// Start FFT
write_reg(base + 0x00, 0x01);

// Wait for completion
while (read_reg(base + 0x08) & 0x01) == 0 {}

// Read output samples
for i in 0..256 {
    let packed = read_reg(base + 0x14);
    output[i].i = (packed >> 16) as i16;
    output[i].q = packed as i16;
}
```

### r4w_fir Register Map

| Offset | Name | Bits | Access | Description |
|--------|------|------|--------|-------------|
| 0x00 | CTRL | [0] | R/W | START - Begin filtering |
| | | [1] | R/W | RELOAD_TAPS - Load new coefficients |
| | | [31] | R/W | RESET - Soft reset |
| 0x04 | NUM_TAPS | [8:0] | R/W | Number of filter taps (1-256) |
| 0x08 | STATUS | [0] | R/O | DONE |
| | | [1] | R/O | BUSY |
| | | [2] | R/O | ERROR |
| 0x10 | DATA_IN | [31:0] | W/O | Packed I/Q input |
| 0x14 | DATA_OUT | [31:0] | R/O | Packed I/Q output |
| 0x1C | VERSION | [31:0] | R/O | Version |
| 0x20 | ID | [31:0] | R/O | 0x52344649 ("R4FI") |
| 0x100-0x4FF | TAPS[0-255] | [15:0] | R/W | Filter coefficients (signed 16-bit) |

**Coefficient Loading:**

```rust
// Write coefficients
for (i, tap) in coefficients.iter().enumerate() {
    let tap_fixed = (*tap * 32767.0) as i16;
    write_reg(base + 0x100 + i * 4, tap_fixed as u32);
}

// Set tap count and trigger reload
write_reg(base + 0x04, coefficients.len() as u32);
write_reg(base + 0x00, 0x02);  // RELOAD_TAPS bit
```

### r4w_nco Register Map

| Offset | Name | Bits | Access | Description |
|--------|------|------|--------|-------------|
| 0x00 | CTRL | [0] | R/W | ENABLE - Start NCO output |
| | | [1] | R/W | RESET_PHASE - Reset phase accumulator |
| | | [31] | R/W | RESET - Soft reset |
| 0x04 | FREQ | [31:0] | R/W | Frequency word (phase increment) |
| 0x08 | PHASE | [31:0] | R/W | Phase offset |
| 0x0C | AMPLITUDE | [15:0] | R/W | Output amplitude (0-32767) |
| 0x10 | STATUS | [0] | R/O | VALID - Output valid |
| 0x14 | I_OUT | [15:0] | R/O | Cosine output (signed) |
| 0x18 | Q_OUT | [15:0] | R/O | Sine output (signed) |
| 0x1C | VERSION | [31:0] | R/O | Version |
| 0x20 | ID | [31:0] | R/O | 0x5234494F ("R4IO") |

**Frequency Calculation:**

```
Frequency Word = (Desired_Frequency / Sample_Rate) * 2^PHASE_WIDTH

Example: 1 kHz at 100 MHz sample rate with 32-bit phase:
  freq_word = (1000 / 100_000_000) * 2^32 = 42949
```

### r4w_chirp_gen Register Map

| Offset | Name | Bits | Access | Description |
|--------|------|------|--------|-------------|
| 0x00 | CTRL | [0] | R/W | START - Generate chirp |
| | | [1] | R/W | UPCHIRP - 1=upchirp, 0=downchirp |
| | | [2] | R/W | CONTINUOUS - Repeat indefinitely |
| | | [31] | R/W | RESET - Soft reset |
| 0x04 | SF | [3:0] | R/W | Spreading factor (5-12) |
| 0x08 | STATUS | [0] | R/O | DONE |
| | | [1] | R/O | BUSY |
| | | [2] | R/O | OVERFLOW |
| 0x0C | SYMBOL | [11:0] | R/W | Symbol value (0 to 2^SF-1) |
| 0x10 | DATA_OUT | [31:0] | R/O | Packed I/Q output |
| 0x14 | BANDWIDTH | [1:0] | R/W | 0=125k, 1=250k, 2=500k |
| 0x1C | VERSION | [31:0] | R/O | Version |
| 0x20 | ID | [31:0] | R/O | 0x52344347 ("R4CG") |

### r4w_chirp_corr Register Map

| Offset | Name | Bits | Access | Description |
|--------|------|------|--------|-------------|
| 0x00 | CTRL | [0] | R/W | START - Begin correlation |
| | | [31] | R/W | RESET - Soft reset |
| 0x04 | SF | [3:0] | R/W | Spreading factor (5-12) |
| 0x08 | STATUS | [0] | R/O | DONE |
| | | [1] | R/O | BUSY |
| | | [2] | R/O | DETECTED - Symbol detected |
| 0x10 | DATA_IN | [31:0] | W/O | Packed I/Q input |
| 0x1C | VERSION | [31:0] | R/O | Version |
| 0x20 | SYMBOL | [11:0] | R/O | Detected symbol value |
| 0x24 | MAGNITUDE | [31:0] | R/O | Correlation magnitude |
| 0x28 | THRESHOLD | [31:0] | R/W | Detection threshold |
| 0x30 | ID | [31:0] | R/O | 0x52344343 ("R4CC") |

### r4w_dma Register Map

| Offset | Name | Bits | Access | Description |
|--------|------|------|--------|-------------|
| 0x00 | CTRL | [0] | R/W | START_TX - Start transmit |
| | | [1] | R/W | START_RX - Start receive |
| | | [2] | R/W | ABORT - Abort transfer |
| | | [3] | R/W | CONTINUOUS - Continuous mode |
| | | [31] | R/W | RESET - Soft reset |
| 0x04 | TX_LEN | [15:0] | R/W | TX transfer length (samples) |
| 0x08 | RX_LEN | [15:0] | R/W | RX transfer length (samples) |
| 0x0C | STATUS | [0] | R/O | TX_BUSY |
| | | [1] | R/O | RX_BUSY |
| | | [2] | R/O | TX_DONE |
| | | [3] | R/O | RX_DONE |
| | | [4] | R/O | TX_ERROR |
| | | [5] | R/O | RX_ERROR |
| 0x10 | TX_ADDR | [31:0] | R/W | TX buffer address |
| 0x14 | RX_ADDR | [31:0] | R/W | RX buffer address |
| 0x18 | IRQ_EN | [2:0] | R/W | Interrupt enables |
| 0x1C | IRQ_STATUS | [2:0] | R/W1C | Interrupt status (write 1 to clear) |

---

## AXI Interface Specifications

### AXI4-Lite Interface

All IP cores implement the standard AXI4-Lite slave interface:

```verilog
// Clock and Reset
input  wire                            S_AXI_ACLK,
input  wire                            S_AXI_ARESETN,

// Write Address Channel
input  wire [C_S_AXI_ADDR_WIDTH-1:0]   S_AXI_AWADDR,
input  wire [2:0]                      S_AXI_AWPROT,
input  wire                            S_AXI_AWVALID,
output wire                            S_AXI_AWREADY,

// Write Data Channel
input  wire [C_S_AXI_DATA_WIDTH-1:0]   S_AXI_WDATA,
input  wire [C_S_AXI_DATA_WIDTH/8-1:0] S_AXI_WSTRB,
input  wire                            S_AXI_WVALID,
output wire                            S_AXI_WREADY,

// Write Response Channel
output wire [1:0]                      S_AXI_BRESP,
output wire                            S_AXI_BVALID,
input  wire                            S_AXI_BREADY,

// Read Address Channel
input  wire [C_S_AXI_ADDR_WIDTH-1:0]   S_AXI_ARADDR,
input  wire [2:0]                      S_AXI_ARPROT,
input  wire                            S_AXI_ARVALID,
output wire                            S_AXI_ARREADY,

// Read Data Channel
output wire [C_S_AXI_DATA_WIDTH-1:0]   S_AXI_RDATA,
output wire [1:0]                      S_AXI_RRESP,
output wire                            S_AXI_RVALID,
input  wire                            S_AXI_RREADY
```

### AXI4-Stream Interface

For high-throughput data paths:

```verilog
// Input Stream (Slave)
input  wire [31:0]  S_AXIS_TDATA,   // Packed I/Q: [31:16]=I, [15:0]=Q
input  wire         S_AXIS_TVALID,
output wire         S_AXIS_TREADY,
input  wire         S_AXIS_TLAST,   // Optional end-of-frame

// Output Stream (Master)
output wire [31:0]  M_AXIS_TDATA,
output wire         M_AXIS_TVALID,
input  wire         M_AXIS_TREADY,
output wire         M_AXIS_TLAST
```

### Data Format

I/Q samples are packed as 32-bit words:

```
Bit 31                16 15                 0
┌─────────────────────┬─────────────────────┐
│   I (signed 16-bit) │   Q (signed 16-bit) │
└─────────────────────┴─────────────────────┘
```

**Conversion:**

```rust
// Rust: float to fixed-point
fn pack_iq(i: f32, q: f32) -> u32 {
    let i_fixed = (i * 32767.0) as i16;
    let q_fixed = (q * 32767.0) as i16;
    ((i_fixed as u32) << 16) | (q_fixed as u16 as u32)
}

// Rust: fixed-point to float
fn unpack_iq(packed: u32) -> (f32, f32) {
    let i_fixed = (packed >> 16) as i16;
    let q_fixed = packed as i16;
    (i_fixed as f32 / 32767.0, q_fixed as f32 / 32767.0)
}
```

---

## Xilinx Zynq Integration

### Block Design Setup

1. **Create Block Design in Vivado:**

```tcl
# Create block design
create_bd_design "r4w_system"

# Add Zynq PS
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 zynq_ps

# Apply board preset
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
  -config {make_external "FIXED_IO, DDR" Master "Disable" Slave "Disable"} \
  [get_bd_cells zynq_ps]

# Enable GP0 master
set_property -dict [list \
  CONFIG.PCW_USE_M_AXI_GP0 {1} \
] [get_bd_cells zynq_ps]

# Add R4W IP cores
add_files -norecurse {
  vivado/ip/common/axi_lite_slave.v
  vivado/ip/common/iq_pack.v
  vivado/ip/r4w_fft/r4w_fft.v
  vivado/ip/r4w_fft/r4w_fft_axi.v
  vivado/ip/r4w_fir/r4w_fir.v
  vivado/ip/r4w_fir/r4w_fir_axi.v
  vivado/ip/r4w_nco/r4w_nco.v
  vivado/ip/r4w_nco/r4w_nco_axi.v
  vivado/ip/r4w_chirp_gen/r4w_chirp_gen.v
  vivado/ip/r4w_chirp_gen/r4w_chirp_gen_axi.v
  vivado/ip/r4w_chirp_corr/r4w_chirp_corr.v
  vivado/ip/r4w_chirp_corr/r4w_chirp_corr_axi.v
  vivado/ip/r4w_dma/r4w_dma.v
  vivado/ip/r4w_dma/r4w_dma_axi.v
}
```

2. **Add AXI Interconnect:**

```tcl
# Add interconnect
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_0
set_property -dict [list CONFIG.NUM_MI {6}] [get_bd_cells axi_interconnect_0]

# Connect PS to interconnect
connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_GP0] \
  [get_bd_intf_pins axi_interconnect_0/S00_AXI]
```

3. **Assign Addresses:**

Recommended address map for Zynq-7000:

| IP Core | Base Address | Size |
|---------|--------------|------|
| r4w_fft | 0x43C0_0000 | 4 KB |
| r4w_fir | 0x43C1_0000 | 4 KB |
| r4w_nco | 0x43C2_0000 | 256 B |
| r4w_chirp_gen | 0x43C3_0000 | 256 B |
| r4w_chirp_corr | 0x43C4_0000 | 256 B |
| r4w_dma | 0x43C5_0000 | 4 KB |

### DMA Configuration

For high-throughput operations, configure AXI DMA:

```tcl
# Add AXI DMA
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_0

set_property -dict [list \
  CONFIG.c_include_sg {0} \
  CONFIG.c_sg_include_stscntrl_strm {0} \
  CONFIG.c_include_mm2s {1} \
  CONFIG.c_include_s2mm {1} \
  CONFIG.c_m_axis_mm2s_tdata_width {32} \
  CONFIG.c_s_axis_s2mm_tdata_width {32} \
] [get_bd_cells axi_dma_0]

# Enable HP port for DMA
set_property -dict [list CONFIG.PCW_USE_S_AXI_HP0 {1}] [get_bd_cells zynq_ps]
```

### Memory Mapping in Rust

The Rust driver accesses hardware via memory-mapped I/O:

```rust
use r4w_fpga::ZynqFpga;

// Auto-detect Zynq platform
let fpga = ZynqFpga::auto_detect()?;

// Or with explicit configuration
let config = ZynqConfig {
    register_regions: vec![
        ("fft".to_string(), 0x43C0_0000, 0x1000),
        ("fir".to_string(), 0x43C1_0000, 0x1000),
        ("nco".to_string(), 0x43C2_0000, 0x100),
        ("chirp_gen".to_string(), 0x43C3_0000, 0x100),
        ("chirp_corr".to_string(), 0x43C4_0000, 0x100),
        ("dma".to_string(), 0x43C5_0000, 0x1000),
    ],
    enable_dma: true,
    dma_buffer_size: 65536,
    ..Default::default()
};

let fpga = ZynqFpga::new(config)?;
```

### Device Tree Overlay

For UIO-based access (recommended over `/dev/mem`):

```dts
/dts-v1/;
/plugin/;

/ {
    fragment@0 {
        target = <&amba>;
        __overlay__ {
            r4w_fft@43C00000 {
                compatible = "r4w,fft";
                reg = <0x43C00000 0x1000>;
                interrupt-parent = <&intc>;
                interrupts = <0 29 4>;
            };

            r4w_fir@43C10000 {
                compatible = "r4w,fir";
                reg = <0x43C10000 0x1000>;
            };

            r4w_dma@43C50000 {
                compatible = "r4w,dma";
                reg = <0x43C50000 0x1000>;
                interrupt-parent = <&intc>;
                interrupts = <0 30 4>;
            };
        };
    };
};
```

---

## Lattice FPGA Integration

### iCE40 Architecture

The iCE40 design uses a simpler SPI-based interface:

```
┌─────────────────────────────────────────────────────────────┐
│                       iCE40 FPGA                            │
│                                                             │
│   ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│   │  SPI Slave   │───►│  Register    │───►│  Chirp Gen   │  │
│   │  Interface   │    │  Decoder     │    │  + NCO       │  │
│   └──────────────┘    └──────────────┘    └──────────────┘  │
│         ▲                                        │          │
│         │                                        ▼          │
│   ┌─────┴─────┐                          ┌──────────────┐   │
│   │   FTDI    │                          │   DAC I/Q    │   │
│   │   SPI     │                          │   Output     │   │
│   └───────────┘                          └──────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### SPI Protocol

**Transaction Format:**

```
Command Byte: [R/W][ADDR_HI]
Address Byte: [ADDR_LO]
Data Bytes:   [DATA_3][DATA_2][DATA_1][DATA_0]

R/W: 0=Read, 1=Write
ADDR: 16-bit register address
DATA: 32-bit data (MSB first)
```

**Timing:**

```
CS_N:  ──┐                                    ┌──
         └────────────────────────────────────┘

CLK:   ____/¯¯\__/¯¯\__/¯¯\__/¯¯\__ ... __/¯¯\__

MOSI:  ----<CMD><ADDR_H><ADDR_L><D3><D2><D1><D0>

MISO:  --------------------------------<D3><D2><D1><D0> (read)
```

### iCE40 Register Map

| Address | Name | Description |
|---------|------|-------------|
| 0x0000 | CTRL | Control register |
| 0x0004 | SF | Spreading factor |
| 0x0008 | STATUS | Status register |
| 0x000C | SYMBOL | Symbol value |
| 0x0010 | NCO_FREQ | NCO frequency word |
| 0x0014 | NCO_PHASE | NCO phase offset |
| 0x0018 | DATA_I | I output (read-only) |
| 0x001C | DATA_Q | Q output (read-only) |
| 0x0020 | ID | 0x52344C49 ("R4LI") |
| 0x0024 | VERSION | Version number |

### ECP5 Enhancements

The ECP5 version adds:

- Internal PLL (25 MHz → 100 MHz)
- SDRAM interface (future buffer support)
- CORDIC-based NCO (no BRAM usage)
- More DSP blocks for larger operations

### Building for Lattice

Using the open-source toolchain:

```bash
# iCE40
cd lattice
yosys -p "synth_ice40 -top r4w_top_ice40 -json r4w_ice40.json" \
    design/r4w_top_ice40.v ip/r4w_spi_slave/*.v ip/r4w_nco/*.v ip/r4w_chirp_gen/*.v

nextpnr-ice40 --hx8k --package ct256 --json r4w_ice40.json \
    --asc r4w_ice40.asc --pcf constraints/ice40_hx8k.pcf

icepack r4w_ice40.asc r4w_ice40.bin

# ECP5
yosys -p "synth_ecp5 -top r4w_top_ecp5 -json r4w_ecp5.json" \
    design/r4w_top_ecp5.v ip/r4w_spi_slave/*.v ip/r4w_nco/*.v ip/r4w_chirp_gen/*.v

nextpnr-ecp5 --25k --package CABGA381 --json r4w_ecp5.json \
    --lpf constraints/ulx3s_v3.lpf --textcfg r4w_ecp5.config

ecppack r4w_ecp5.config r4w_ecp5.bit
```

---

## Building and Synthesis

### Vivado Project Setup

```tcl
# Create project
create_project r4w_zynq ./r4w_zynq -part xc7z020clg400-1

# Add source files
add_files -norecurse [glob vivado/ip/*/*.v]
add_files -norecurse [glob vivado/ip/common/*.v]

# Add constraints
add_files -fileset constrs_1 -norecurse vivado/constraints/zynq_z020.xdc

# Create block design
source vivado/scripts/create_bd.tcl

# Generate wrapper
make_wrapper -files [get_files r4w_system.bd] -top
add_files -norecurse r4w_zynq.srcs/sources_1/bd/r4w_system/hdl/r4w_system_wrapper.v

# Synthesize
launch_runs synth_1 -jobs 8
wait_on_run synth_1

# Implement
launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
```

### Timing Constraints

```tcl
# vivado/constraints/zynq_z020.xdc

# System clock (from PS)
create_clock -period 10.000 -name clk_100 [get_ports FCLK_CLK0]

# Clock domain crossings (if any)
set_clock_groups -asynchronous \
  -group [get_clocks clk_100] \
  -group [get_clocks spi_clk]

# False paths for configuration registers
set_false_path -from [get_cells -hierarchical -filter {NAME =~ */ctrl_reg*}]
set_false_path -to [get_cells -hierarchical -filter {NAME =~ */status_*}]
```

### Resource Utilization Targets

For Zynq xc7z020:

| IP Core | LUTs | FFs | DSP | BRAM |
|---------|------|-----|-----|------|
| r4w_fft (1024pt) | 2,500 | 3,000 | 4 | 4 |
| r4w_fir (64 tap) | 800 | 1,200 | 2 | 0.5 |
| r4w_nco (CORDIC) | 400 | 600 | 0 | 0 |
| r4w_chirp_gen | 600 | 800 | 0 | 0.5 |
| r4w_chirp_corr | 3,000 | 3,500 | 4 | 4 |
| r4w_dma | 500 | 700 | 0 | 2 |
| **Total** | ~7,800 | ~9,800 | 10 | 11 |
| **Available (z020)** | 53,200 | 106,400 | 220 | 140 |
| **Utilization** | 15% | 9% | 5% | 8% |

---

## Verification and Testing

### Testbench Structure

Each IP core has an associated testbench:

```
vivado/sim/
├── tb_r4w_fft.v
├── tb_r4w_fir.v
├── tb_r4w_nco.v
├── tb_r4w_chirp_gen.v
├── tb_r4w_chirp_corr.v
└── tb_r4w_dma.v

lattice/sim/
├── tb_r4w_top_ice40.v
└── tb_r4w_top_ecp5.v
```

### Running Simulations

**Vivado:**

```tcl
# Create simulation fileset
add_files -fileset sim_1 -norecurse [glob vivado/sim/*.v]

# Set top-level
set_property top tb_r4w_fft [get_filesets sim_1]

# Run simulation
launch_simulation
run 100 us
```

**Icarus Verilog (Lattice):**

```bash
cd lattice

# Compile
iverilog -o sim/tb_r4w_top_ice40.vvp \
    sim/tb_r4w_top_ice40.v \
    design/r4w_top_ice40.v \
    ip/r4w_spi_slave/r4w_spi_slave.v \
    ip/r4w_nco/r4w_nco.v \
    ip/r4w_chirp_gen/r4w_chirp_gen.v

# Run
vvp sim/tb_r4w_top_ice40.vvp

# View waveforms
gtkwave tb_r4w_top_ice40.vcd
```

### Test Vector Generation

The Rust codebase can generate test vectors for verification:

```rust
use r4w_core::dsp::chirp::generate_chirp;

// Generate reference chirp
let reference = generate_chirp(7, true, 125000.0, 500000.0);

// Write to file for Verilog testbench
let mut file = File::create("test_vectors/chirp_sf7_up.mem")?;
for sample in reference {
    let i_fixed = (sample.i * 32767.0) as i16;
    let q_fixed = (sample.q * 32767.0) as i16;
    writeln!(file, "{:04X}{:04X}", i_fixed as u16, q_fixed as u16)?;
}
```

### Verification Checklist

- [ ] Register read/write functionality
- [ ] IP core ID verification
- [ ] Start/done handshaking
- [ ] Reset behavior
- [ ] Data path correctness (compare to software model)
- [ ] AXI protocol compliance
- [ ] Clock domain crossings
- [ ] Resource utilization within targets
- [ ] Timing closure

---

## Software Driver Interface

### FpgaAccelerator Trait

Software developers use the `FpgaAccelerator` trait. Your FPGA implementation must support:

```rust
pub trait FpgaAccelerator: Send + Sync {
    // Device info
    fn info(&self) -> FpgaInfo;
    fn is_available(&self) -> bool;
    fn capabilities(&self) -> FpgaCapabilities;

    // Core DSP
    fn fft(&self, samples: &[IQSample], inverse: bool) -> FpgaResult<Vec<IQSample>>;
    fn fir_filter(&self, samples: &[IQSample], taps: &[f32]) -> FpgaResult<Vec<IQSample>>;
    fn complex_multiply(&self, a: &[IQSample], b: &[IQSample]) -> FpgaResult<Vec<IQSample>>;

    // Waveform-specific
    fn generate_chirp(&self, sf: u8, upchirp: bool) -> FpgaResult<Vec<IQSample>>;
    fn chirp_correlate(&self, samples: &[IQSample], sf: u8) -> FpgaResult<(u32, f32)>;

    // Streaming
    fn start_stream(&mut self, config: StreamConfig) -> FpgaResult<StreamHandle>;
    fn stop_stream(&mut self, handle: StreamHandle) -> FpgaResult<()>;
    fn write_stream(&mut self, handle: StreamHandle, samples: &[IQSample]) -> FpgaResult<usize>;
    fn read_stream(&mut self, handle: StreamHandle, buffer: &mut [IQSample]) -> FpgaResult<usize>;

    // Low-level
    fn read_register(&self, address: usize) -> FpgaResult<u32>;
    fn write_register(&mut self, address: usize, value: u32) -> FpgaResult<()>;
    fn reset(&mut self) -> FpgaResult<()>;
}
```

### Capability Discovery

Software will query capabilities at startup:

```rust
let caps = fpga.capabilities();
println!("Max FFT: {} points", caps.max_fft_size);
println!("Max FIR: {} taps", caps.max_fir_taps);
println!("DMA: {}", if caps.supports_streaming { "yes" } else { "no" });

for core in &caps.ip_cores {
    println!("IP Core: {} v{} at 0x{:08X}",
        core.name, core.version, core.base_address);
}
```

### Error Handling

Hardware errors should map to these Rust error types:

```rust
pub enum FpgaError {
    DeviceNotFound(String),
    PlatformNotSupported,
    MmapFailed { address: usize, reason: String },
    Timeout { address: usize, timeout_ms: u32 },
    DmaError(String),
    StreamError(String),
    IpCoreNotAvailable(String),
    ConfigError(String),
    BufferSizeMismatch { expected: usize, actual: usize },
    InvalidAddress(usize),
    NotSupported(String),
}
```

---

## Performance Optimization

### Throughput Considerations

| Metric | Target | Notes |
|--------|--------|-------|
| FFT (1024pt) | < 10 μs | At 100 MHz fabric clock |
| FIR (per sample) | < 100 ns | Pipelined, 64 taps |
| Chirp generation | < 1 ms | For SF12 (4096 samples) |
| DMA transfer | > 100 MS/s | Using HP ports |

### Pipelining

For maximum throughput, implement pipelining:

```verilog
// Pipeline stages for FIR filter
always @(posedge clk) begin
    // Stage 1: Input register
    sample_d1 <= s_axis_tdata;
    valid_d1 <= s_axis_tvalid;

    // Stage 2: Multiply
    for (i = 0; i < NUM_TAPS; i = i + 1) begin
        mult_result[i] <= $signed(sample_d1[i]) * $signed(coef[i]);
    end
    valid_d2 <= valid_d1;

    // Stage 3: Accumulate (tree structure)
    sum_l1[0] <= mult_result[0] + mult_result[1];
    sum_l1[1] <= mult_result[2] + mult_result[3];
    // ...
    valid_d3 <= valid_d2;

    // Stage 4: Final sum
    output <= sum_l2[0] + sum_l2[1];
    output_valid <= valid_d3;
end
```

### DMA Best Practices

1. **Buffer Size**: Use at least 4 KB buffers to amortize DMA overhead
2. **Alignment**: Align buffers to page boundaries (4 KB)
3. **Continuous Mode**: For streaming, use circular buffers
4. **Interrupts**: Use interrupts instead of polling for efficiency

### Resource Sharing

For area-constrained designs, consider time-multiplexing:

```verilog
// Shared multiplier for NCO and FIR
always @(posedge clk) begin
    case (state)
        STATE_NCO: begin
            mult_a <= phase_sin;
            mult_b <= amplitude;
        end
        STATE_FIR: begin
            mult_a <= sample_in;
            mult_b <= current_tap;
        end
    endcase
end

assign mult_result = mult_a * mult_b;
```

---

## Adding New IP Cores

### IP Core Template

```verilog
//-----------------------------------------------------------------------------
// R4W New IP Core with AXI-Lite Interface
//-----------------------------------------------------------------------------

module r4w_newcore_axi #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 8
)(
    // AXI4-Lite Interface
    input  wire                            S_AXI_ACLK,
    input  wire                            S_AXI_ARESETN,
    // ... (standard AXI-Lite ports)

    // Optional AXI-Stream ports
    input  wire [31:0]                     S_AXIS_TDATA,
    input  wire                            S_AXIS_TVALID,
    output wire                            S_AXIS_TREADY,

    output wire [31:0]                     M_AXIS_TDATA,
    output wire                            M_AXIS_TVALID,
    input  wire                            M_AXIS_TREADY
);

    // Standard constants
    localparam IP_ID = 32'h5234xxxx;       // "R4xx" pattern
    localparam IP_VERSION = 32'h00010000;  // 1.0.0

    // Register addresses
    localparam ADDR_CTRL    = 8'h00;
    localparam ADDR_CONFIG  = 8'h04;
    localparam ADDR_STATUS  = 8'h08;
    localparam ADDR_VERSION = 8'h1C;
    localparam ADDR_ID      = 8'h20;

    // Control registers
    reg [31:0] ctrl_reg;
    wire start = ctrl_reg[0];
    wire soft_reset = ctrl_reg[31];

    // Instantiate common AXI slave
    axi_lite_slave #(
        .C_S_AXI_DATA_WIDTH(C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH)
    ) axi_slave ( /* ... */ );

    // Your processing logic here

endmodule
```

### Registration in Software

To expose your new IP core to software:

1. Add IP core type in `crates/r4w-fpga/src/types.rs`:

```rust
pub enum IpCoreType {
    Fft { size: usize },
    Fir { max_taps: usize },
    Nco,
    ChirpGenerator,
    ChirpCorrelator,
    Dma,
    NewCore { /* your params */ },  // Add here
}
```

2. Add probe function in platform-specific code:

```rust
fn probe_newcore(regions: &[MemoryRegion], config: &Config) -> Option<IpCore> {
    let base = config.newcore_base_addr?;

    for region in regions {
        if region.contains(base) {
            if let Ok(id) = region.read32(base + 0x20) {
                if id == 0x5234xxxx {  // Your IP ID
                    return Some(IpCore {
                        name: "r4w_newcore".to_string(),
                        core_type: IpCoreType::NewCore { /* params */ },
                        base_address: base,
                        // ...
                    });
                }
            }
        }
    }
    None
}
```

---

## Debugging and Troubleshooting

### Common Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| No response from IP | Read returns 0xFFFFFFFF | Check address mapping, clock, reset |
| Wrong data | Incorrect results | Verify data format, endianness |
| Timeout | DONE never asserts | Check state machine, enable signals |
| DMA failure | Transfer incomplete | Verify buffer alignment, length |
| Timing failure | Unpredictable behavior | Add pipeline stages, reduce clock |

### ILA Debug

Insert Integrated Logic Analyzer for runtime debugging:

```tcl
# Create ILA core
create_debug_core u_ila_0 ila

# Connect probes
set_property port_width 32 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {r4w_fft/ctrl_reg[*]}]]

set_property port_width 32 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {r4w_fft/status_reg[*]}]]

# Add trigger
set_property port_width 1 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list r4w_fft/start]]
```

### Register Dump

Software can dump all registers for debugging:

```rust
fn dump_registers(fpga: &dyn FpgaAccelerator, base: usize, count: usize) {
    println!("Register dump at 0x{:08X}:", base);
    for i in 0..count {
        let addr = base + i * 4;
        match fpga.read_register(addr) {
            Ok(val) => println!("  0x{:03X}: 0x{:08X}", i * 4, val),
            Err(e) => println!("  0x{:03X}: ERROR - {:?}", i * 4, e),
        }
    }
}
```

### Status LED Mapping

Use LEDs for quick status indication:

```verilog
// iCE40/ECP5 status LEDs
assign led[0] = pll_locked;      // Green = clock OK
assign led[1] = chirp_busy;      // Activity indicator
assign led[2] = chirp_done;      // Completion indicator
assign led[3] = !spi_cs_n;       // SPI activity
assign led[7:4] = sf[3:0];       // Current SF
```

---

## Resource Utilization

### Zynq-7020 Budget

| Resource | Available | Used (Typical) | Remaining |
|----------|-----------|----------------|-----------|
| LUTs | 53,200 | ~8,000 (15%) | 45,200 |
| FFs | 106,400 | ~10,000 (9%) | 96,400 |
| DSP48E1 | 220 | 10 (5%) | 210 |
| BRAM | 140 (36Kb) | 11 (8%) | 129 |

### iCE40-HX8K Budget

| Resource | Available | Used (Typical) | Remaining |
|----------|-----------|----------------|-----------|
| LCs | 7,680 | ~3,500 (46%) | 4,180 |
| FFs | 7,680 | ~2,000 (26%) | 5,680 |
| BRAM | 32 (4Kb) | 4 (13%) | 28 |

### Scaling Guidance

| Configuration | LUTs | DSP | Notes |
|--------------|------|-----|-------|
| Minimal (NCO + chirp only) | 1,000 | 0 | Good for iCE40 |
| Standard (FFT + FIR + chirp) | 5,000 | 6 | Typical deployment |
| Full (all cores + DMA) | 8,000 | 10 | Full acceleration |
| Extended (multi-FFT) | 15,000 | 16 | High throughput |

---

## Collaboration with Software Developers

### Communication Channels

1. **Register Specification Document**: Maintain accurate register maps
2. **Test Vectors**: Provide bit-accurate test vectors for verification
3. **Timing Diagrams**: Document operation timing for driver development
4. **Performance Metrics**: Share measured latencies and throughputs

### Development Workflow

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Development Workflow                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│   1. Software Developer                 FPGA Developer                  │
│      ┌─────────────────┐               ┌─────────────────┐              │
│      │ Define          │               │ Review          │              │
│      │ Requirements    │──────────────►│ Feasibility     │              │
│      └─────────────────┘               └─────────────────┘              │
│                                                │                        │
│                                                ▼                        │
│                                         ┌─────────────────┐             │
│                                         │ Design &        │             │
│                                         │ Implement       │             │
│                                         └─────────────────┘             │
│                                                │                        │
│      ┌─────────────────┐                ┌──────┴──────────┐             │
│      │ Write Driver    │◄───────────────│ Provide         │             │
│      │ Code            │                │ Register Map    │             │
│      └─────────────────┘                └─────────────────┘             │
│             │                                  │                        │
│             ▼                                  ▼                        │
│      ┌─────────────────┐               ┌─────────────────┐              │
│      │ Generate        │──────────────►│ Verify with     │              │
│      │ Test Vectors    │               │ Test Vectors    │              │
│      └─────────────────┘               └─────────────────┘              │
│             │                                  │                        │
│             └──────────────┬───────────────────┘                        │
│                            ▼                                            │
│                     ┌─────────────────┐                                 │
│                     │ Integration     │                                 │
│                     │ Testing         │                                 │
│                     └─────────────────┘                                 │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### Test Vector Format

Standard format for exchanging test data:

```
# test_vectors/fft_1024_input.csv
# Format: I,Q (floating point -1.0 to 1.0)
0.5,0.0
0.353553,0.353553
0.0,0.5
-0.353553,0.353553
...
```

```verilog
// Load in testbench
initial begin
    $readmemh("fft_1024_input.mem", input_memory);
end
```

### Simulation Mode

For development without hardware, software uses the `SimulatedFpga`:

```rust
#[cfg(feature = "sim")]
let fpga: Box<dyn FpgaAccelerator> = Box::new(SimulatedFpga::new());

#[cfg(not(feature = "sim"))]
let fpga: Box<dyn FpgaAccelerator> = Box::new(ZynqFpga::auto_detect()?);

// Same API works for both
let result = fpga.fft(&samples, false)?;
```

This allows software developers to work without FPGA hardware while you develop the bitstream.

---

## Appendix A: File Structure

```
vivado/
├── ip/
│   ├── common/
│   │   ├── axi_lite_slave.v      # Reusable AXI-Lite slave
│   │   └── iq_pack.v             # I/Q packing/unpacking
│   ├── r4w_fft/
│   │   ├── r4w_fft.v             # FFT core logic
│   │   └── r4w_fft_axi.v         # AXI wrapper
│   ├── r4w_fir/
│   ├── r4w_nco/
│   ├── r4w_chirp_gen/
│   ├── r4w_chirp_corr/
│   └── r4w_dma/
├── sim/
│   ├── tb_r4w_fft.v
│   └── ...
├── scripts/
│   └── create_bd.tcl
└── constraints/
    ├── zynq_z020.xdc
    └── zynq_ultra.xdc

lattice/
├── ip/
│   ├── r4w_spi_slave/
│   ├── r4w_nco/
│   └── r4w_chirp_gen/
├── design/
│   ├── r4w_top_ice40.v
│   └── r4w_top_ecp5.v
├── sim/
│   └── tb_r4w_top_ice40.v
└── constraints/
    ├── ice40_hx8k.pcf
    └── ulx3s_v3.lpf

crates/r4w-fpga/
├── src/
│   ├── lib.rs
│   ├── traits.rs                 # FpgaAccelerator trait
│   ├── types.rs                  # FpgaInfo, IpCore, etc.
│   ├── error.rs                  # FpgaError enum
│   ├── zynq/
│   │   ├── mod.rs                # ZynqFpga implementation
│   │   ├── config.rs
│   │   ├── dma.rs
│   │   ├── mmap.rs
│   │   └── registers.rs
│   ├── lattice/
│   │   ├── mod.rs                # LatticeFpga implementation
│   │   ├── spi.rs
│   │   └── ftdi.rs
│   └── sim/
│       └── mod.rs                # SimulatedFpga
└── Cargo.toml
```

---

## Appendix B: Quick Reference

### IP Core IDs

| Core | ID (Hex) | ID (ASCII) |
|------|----------|------------|
| FFT | 0x52345746 | "R4WF" |
| FIR | 0x52344649 | "R4FI" |
| NCO | 0x5234494F | "R4IO" |
| Chirp Gen | 0x52344347 | "R4CG" |
| Chirp Corr | 0x52344343 | "R4CC" |
| DMA | 0x52344D41 | "R4MA" |
| Lattice iCE40 | 0x52344C49 | "R4LI" |
| Lattice ECP5 | 0x52344C45 | "R4LE" |

### Common Register Bits

| Register | Bit | Meaning |
|----------|-----|---------|
| CTRL | 0 | START |
| CTRL | 31 | SOFT_RESET |
| STATUS | 0 | DONE |
| STATUS | 1 | BUSY |
| STATUS | 2 | ERROR |

### Useful Commands

```bash
# Check Zynq platform
cat /sys/firmware/devicetree/base/compatible

# Read register (devmem)
devmem2 0x43C00020 w  # Read FFT ID

# Program iCE40
iceprog r4w_ice40.bin

# Program ECP5
ecpprog r4w_ecp5.bit
```

---

## Document History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-01-01 | Initial release |

---

*For questions or contributions, please open an issue on the R4W GitHub repository.*
