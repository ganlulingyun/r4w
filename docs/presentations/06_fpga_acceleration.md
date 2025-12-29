---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>FPGA Acceleration'
subtitle: "Hardware DSP for Real-Time SDR"
author: "R4W Development Team<br>(Aida, Joe Mooney, Claude Code)"
date: "December 2025"
header-includes: |
  <style>
    .reveal pre {
      margin: 0 auto;
      width: fit-content;
    }
    .reveal pre code {
      text-align: left;
    }
    .reveal section img {
      max-height: 60vh;
    }
  </style>
---

## Why FPGA Acceleration?

Software DSP hits limits for high-rate SDR:

| Challenge | Solution |
|-----------|----------|
| **CPU latency** | Dedicated hardware pipelines |
| **Power consumption** | Efficient parallel processing |
| **Deterministic timing** | Hardware clock domains |
| **Multi-GS/s rates** | Direct RF interface |

---

## Supported Platforms

| Platform | Toolchain | Status |
|----------|-----------|--------|
| **Xilinx Zynq-7000** | Vivado 2022.2+ | Production |
| **Xilinx Zynq UltraScale+** | Vivado 2022.2+ | Production |
| **Lattice iCE40** | Yosys + nextpnr | Implemented |
| **Lattice ECP5** | Yosys + nextpnr | Implemented |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                 ZYNQ PS (ARM Cortex-A9/A53)                 │
│                                                             │
│   ┌────────────────┐    ┌────────────────┐                  │
│   │   R4W Rust     │───►│  Linux Kernel  │──► /dev/mem      │
│   │   Application  │    │   (UIO/devmem) │    /dev/uio*     │
│   └────────────────┘    └────────────────┘                  │
└─────────────────────────────┬───────────────────────────────┘
                              │ AXI Interconnect
┌─────────────────────────────┴───────────────────────────────┐
│                     FPGA Fabric (PL)                         │
│   ┌─────────┐  ┌─────────┐  ┌─────────┐  ┌─────────┐        │
│   │  r4w_   │  │  r4w_   │  │  r4w_   │  │  r4w_   │        │
│   │  fft    │  │  fir    │  │  nco    │  │  chirp  │        │
│   └─────────┘  └─────────┘  └─────────┘  └─────────┘        │
└─────────────────────────────────────────────────────────────┘
```

---

## IP Core Library

| IP Core | Description | Resources |
|---------|-------------|-----------|
| **r4w_fft** | FFT/IFFT up to 4096 points | ~15K LUTs |
| **r4w_fir** | FIR filter up to 256 taps | ~8K LUTs |
| **r4w_nco** | NCO with CORDIC or LUT | ~1.5K LUTs |
| **r4w_chirp_gen** | LoRa chirp generator | ~3K LUTs |
| **r4w_chirp_corr** | Chirp correlator | ~12K LUTs |
| **r4w_dma** | DMA controller | ~4K LUTs |

---

## Interface Types

| Interface | Use Case | Bandwidth |
|-----------|----------|-----------|
| **AXI-Lite** | Control registers, config | Low |
| **AXI-Stream** | Sample data, streaming | High |
| **DMA** | Bulk transfers, buffers | High |
| **Register** | Single sample, debug | Very Low |

---

## IP Core Directory Structure

```
vivado/ip/
├── r4w_fft/
│   ├── hdl/
│   │   └── r4w_fft.v
│   ├── tb/
│   │   └── r4w_fft_tb.v
│   └── component.xml
├── r4w_fir/
├── r4w_nco/
├── r4w_chirp_gen/
├── r4w_chirp_corr/
└── r4w_dma/
```

---

## FFT IP Core

1024-point streaming FFT with:

- **Fixed-point**: 16-bit I/Q input, 32-bit output
- **Decimation-in-time** Radix-2 algorithm
- **Streaming**: Pipeline accepts continuous data
- **Scaling**: Per-stage to prevent overflow

```
Input: 16-bit I[15:0], Q[15:0]
Output: 32-bit magnitude[31:0]
Latency: ~log2(N) * 4 cycles
```

---

## Register Map Example: FFT

| Offset | Name | R/W | Description |
|--------|------|-----|-------------|
| 0x00 | CTRL | RW | Control register |
| 0x04 | STATUS | R | Status register |
| 0x08 | FFT_SIZE | RW | FFT size (64-4096) |
| 0x0C | SCALE | RW | Scaling factors |
| 0x10 | SAMPLE_IN | W | Input sample |
| 0x14 | SAMPLE_OUT | R | Output magnitude |

---

## LoRa Chirp Generator

Hardware LoRa chirp generation:

| Parameter | Range | Description |
|-----------|-------|-------------|
| SF | 5-12 | Spreading factor |
| BW | 125/250/500 kHz | Bandwidth |
| Symbol | 0 to 2^SF-1 | Symbol value |

Generates:
- Upchirp, downchirp
- Symbol-shifted chirps
- Continuous streaming

---

## Lattice FPGA Integration

Open-source toolchain (Yosys + nextpnr):

```bash
# Build for iCE40
cd lattice
make DEVICE=ice40-hx8k synth

# Build for ECP5
make DEVICE=ecp5-45f synth
```

Lower resources, simpler peripherals, great for:
- Low-power nodes
- Educational platforms
- Prototyping

---

## Rust Driver Interface

```rust
use r4w_fpga::FftAccelerator;

// Memory-mapped I/O
let fft = FftAccelerator::new(0x43C0_0000)?;

// Configure
fft.set_fft_size(1024)?;
fft.enable()?;

// Process samples
for sample in samples {
    fft.write_sample(sample)?;
}

let magnitude = fft.read_magnitude()?;
```

---

## Performance: SW vs HW

| Operation | Software | FPGA | Speedup |
|-----------|----------|------|---------|
| FFT 1024-pt | 371 MS/s | 1+ GS/s | **3x+** |
| FIR 128-tap | ~100 MS/s | 500+ MS/s | **5x+** |
| LoRa decode | 45 MS/s | 200+ MS/s | **4x+** |

Plus: Consistent latency, lower power

---

## Resource Utilization (Zynq-7020)

| IP Core | LUTs | FFs | BRAM | DSP |
|---------|------|-----|------|-----|
| r4w_fft | 15K | 12K | 4 | 8 |
| r4w_fir | 8K | 6K | 2 | 4 |
| r4w_nco | 1.5K | 1K | 0 | 1 |
| r4w_chirp | 3K | 2K | 1 | 2 |
| **Total** | ~28K | ~21K | 7 | 15 |

Available: 53K LUTs, 106K FFs, 140 BRAM, 220 DSP

---

## Building and Synthesis

```bash
# Vivado flow
cd vivado
make ip           # Package IP cores
make project      # Create project
make synth        # Synthesize
make impl         # Implement
make bitstream    # Generate bitstream

# Deploy to Zynq
make deploy BOARD=zedboard
```

---

## Verification

Each IP includes Verilog testbenches:

```bash
# Run FFT testbench
cd vivado/ip/r4w_fft/tb
iverilog -o fft_tb r4w_fft_tb.v ../hdl/r4w_fft.v
vvp fft_tb

# Run all testbenches
make -C vivado test
```

Compares against software reference.

---

## Adding Custom IP Cores

1. Create HDL in `vivado/ip/<name>/hdl/`
2. Define AXI-Lite register map
3. Add AXI-Stream for data path
4. Write testbench in `tb/`
5. Create `component.xml` for Vivado
6. Add Rust driver in `r4w-fpga`

---

## Software Simulation

Develop without hardware:

```rust
use r4w_fpga::simulator::FpgaSimulator;

// Create simulated FPGA
let sim = FpgaSimulator::new();

// Register IP cores
sim.add_core("fft", Box::new(FftSim::new()));

// Process (uses software models)
sim.process_samples(&samples)?;
```

Same API, no board required.

---

## Clock Domains

| Clock | Typical | Use |
|-------|---------|-----|
| PS clock | 100-300 MHz | AXI interface |
| DSP clock | 100-500 MHz | Processing |
| ADC clock | 100-250 MHz | Sample rate |

CDC (Clock Domain Crossing) handled by:
- AXI async FIFOs
- Handshake protocols

---

## Debugging and Troubleshooting

| Tool | Purpose |
|------|---------|
| ILA (Integrated Logic Analyzer) | Signal capture |
| VIO (Virtual I/O) | Runtime control |
| Console (UART) | Debug messages |
| Rust tracing | Software logging |

```tcl
# Add ILA in Vivado
create_debug_core u_ila_0 ila
set_property port_width 128 [get_debug_ports]
```

---

## FPGA Developer Responsibilities

1. **Platform Integration** - Block design, addressing
2. **Resource Optimization** - Meet timing constraints
3. **Clock Management** - Proper CDCs
4. **Custom IP** - Waveform-specific blocks
5. **Bitstream Generation** - Deployable images
6. **Verification** - Hardware matches software

---

## Summary

| Component | Status |
|-----------|--------|
| **6 IP Cores** | Production ready |
| **Zynq Support** | Complete |
| **Lattice Support** | Complete |
| **Testbenches** | All cores |
| **Rust Drivers** | Memory-mapped I/O |
| **Documentation** | FPGA Developer's Guide |

**Accelerate your SDR with R4W FPGA!**

---

## Questions?

**R4W - FPGA Acceleration**

github.com/joemooney/r4w

Docs: `docs/FPGA_DEVELOPERS_GUIDE.md`
