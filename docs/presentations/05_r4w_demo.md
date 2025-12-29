---
title: '<img src="images/r4w_logo_1.png" height="200px" style="margin-bottom: 20px;"><br>R4W - Rust for Waveforms'
subtitle: "An Open Platform for SDR Waveform Development"
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

## What is R4W?

A **complete platform** for developing, testing, and deploying Software Defined Radio waveforms in Rust.

- **84,467 lines** of production code
- **66,572 lines** of Rust (527 tests)
- **38+ waveforms** implemented
- **6,324 lines** of Coq formal verification

---

## Platform Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Applications                     │
│         (r4w-explorer, r4w CLI, r4w-web)            │
├─────────────────────────────────────────────────────┤
│    Waveforms (38+)          │     Plugins (.so)     │
│  LoRa, PSK, QAM, FSK, OFDM  │  Dynamic loading      │
│  SINCGARS, HAVEQUICK, P25   │  Hot reload           │
├─────────────────────────────────────────────────────┤
│                    Core DSP                         │
│     Chirp, FFT, Filters, FEC, Timing, RT Buffers    │
├─────────────────────────────────────────────────────┤
│    Hardware Abstraction     │    Waveform Sandbox   │
│  USRP, RTL-SDR, Simulator   │  8 isolation levels   │
│  Xilinx Zynq, Lattice FPGA  │  Containers, VMs      │
└─────────────────────────────────────────────────────┘
```

---

## The R4W Explorer

Interactive GUI for learning and experimenting with waveforms.

![Overview](images/screenshots/01_overview_bpsk_intro.png)

---

## Educational Overview

Each waveform includes:

- **What is SDR?** - Introduction to concepts
- **I/Q Samples** - Complex number representation
- **Modulation theory** - How bits become signals
- **Processing pipeline** - TX and RX data flow

---

## Waveform Lab

Experiment with modulation parameters in real-time.

![Waveform Lab](images/screenshots/02_waveform_lab_bpsk.png)

---

## Interactive Features

- **Bit sequences** - Test custom payloads
- **Channel simulation** - Add AWGN noise
- **Time domain** - View I/Q waveforms
- **Constellation** - See symbol positions
- **BER measurement** - Real-time error rates

---

## Real-Time Streaming

Live signal visualization with multiple views.

![Streaming](images/screenshots/03_streaming_realtime.png)

---

## Streaming Views

Four synchronized visualizations:

| View | Purpose |
|------|---------|
| **Oscilloscope** | Time-domain I/Q |
| **Constellation** | Symbol positions |
| **Eye Diagram** | Symbol timing |
| **Waterfall** | Frequency over time |

Plus spectrum analyzer with power measurements.

---

## Code Explorer

Learn by reading the actual Rust implementation.

![Code Explorer](images/screenshots/04_code_explorer_qam.png)

---

## Educational Code

- **Syntax highlighting** for Rust
- **Expandable explanations** for each function
- **Complexity ratings** (1-5 stars)
- **Real production code** - not simplified examples

---

## Performance Benchmarks

Compare sequential vs parallel processing.

![Performance](images/screenshots/05_performance_benchmarks.png)

---

## Benchmark Results

| Metric | Value |
|--------|-------|
| Sequential throughput | 25.4 K/s |
| Parallel throughput | 45.6 K/s |
| Speedup | **1.79x** |
| Scaling at 64 items | **5.89x** |

SIMD: SSE4.2, AVX2 when available.

---

## LoRa Chirp Signals

Visualize Chirp Spread Spectrum modulation.

![Chirp Signals](images/screenshots/06_chirp_signals_lora.png)

---

## Understanding Chirps

- **I/Q components** - Real (blue) and Imaginary (red)
- **Instantaneous frequency** - Linear sweep
- **Symbol encoding** - Cyclic shift of chirp
- **SF7 @ 125 kHz** - 128 chips per symbol

---

## 38+ Waveforms

Comprehensive waveform library.

![Waveform Selector](images/screenshots/07_waveform_selector.png)

---

## Waveform Categories

| Category | Waveforms |
|----------|-----------|
| **Multi-Carrier** | OFDM |
| **Spread Spectrum** | DSSS, DSSS-QPSK, FHSS, LoRa |
| **IoT & Radar** | Zigbee |
| **Digital** | BPSK, QPSK, 8PSK, 16/64/256-QAM |
| **Analog** | AM, FM, SSB |
| **Military** | SINCGARS, HAVEQUICK, P25 |

---

## Full TX/RX Pipeline

Complete modulation to demodulation visualization.

![Full Pipeline](images/screenshots/08_full_pipeline_dsss.png)

---

## Pipeline Stages

```
Modulation → Channel → Demodulation
    │           │           │
    ▼           ▼           ▼
 Bit-to-    Add AWGN    Sample
 Symbol      Noise     Processing
    │           │           │
    ▼           ▼           ▼
 Sample     Apply       Bit
Generation  Fading    Recovery
```

BER: **0.00e0 (Perfect)** at 40 dB SNR

---

## Spectrum Analyzer

Frequency domain analysis.

![Spectrum Analyzer](images/screenshots/09_spectrum_analyzer.png)

---

## Spectrum Features

- **FFT-based** analysis (32-8192 points)
- **Magnitude and Phase** display
- **Log scale (dB)** option
- **Window position** scrubbing
- **Frequency resolution** display

Zigbee example: O-QPSK with half-sine pulse shaping.

---

## Constellation Diagrams

I/Q signal visualization.

![Constellation](images/screenshots/10_constellation_am.png)

---

## Constellation Features

- **Real-time** point plotting
- **Trajectory** visualization
- **Point count** control (10-1000)
- **Educational notes** for each modulation type

AM: Amplitude varies along I-axis, phase constant.

---

## Interactive Tutorial

Web-based learning path.

![Tutorial](images/screenshots/11_tutorial_web.png)

---

## Tutorial Curriculum

| Section | Topics |
|---------|--------|
| **Foundations** | I/Q, Sampling, Frequency Domain |
| **Waveforms** | CW → OOK → FSK → PSK → QAM → CSS |
| **Advanced** | Channel Effects, Sync, Error Correction |
| **Performance** | Waveform Comparison, Trade-offs |

Complexity levels L1 (beginner) to L5 (advanced).

---

## FPGA Acceleration

Hardware acceleration for real-time processing.

| Platform | Toolchain | Status |
|----------|-----------|--------|
| **Xilinx Zynq** | Vivado | IP cores ready |
| **Lattice iCE40** | Yosys/nextpnr | Implemented |
| **Lattice ECP5** | Yosys/nextpnr | Implemented |

---

## Zynq IP Cores

```
vivado/ip/
├── r4w_fft/         # 1024-pt FFT (~15k LUTs)
├── r4w_fir/         # 256-tap FIR (~8k LUTs)
├── r4w_chirp_gen/   # LoRa chirp generator
├── r4w_chirp_corr/  # Chirp correlator
├── r4w_nco/         # NCO (~1.5k LUTs)
└── r4w_dma/         # DMA controller
```

---

## Security: Waveform Isolation

8 levels of sandboxing for untrusted waveforms.

| Level | Mechanism | Overhead |
|-------|-----------|----------|
| L1 | Rust memory safety | Zero |
| L2 | Linux namespaces | ~1% |
| L3 | Seccomp + LSM | ~2% |
| L4 | Container (Docker) | ~5% |
| L5 | MicroVM (Firecracker) | ~10% |
| L6 | Full VM (KVM) | ~15% |
| L7 | Hardware (FPGA partition) | Varies |
| L8 | Air gap | N/A |

---

## Formal Verification

6,324 lines of Coq proofs.

- **FFT correctness** - Output matches DFT definition
- **Chirp properties** - Energy preservation
- **Timing guarantees** - RT scheduler proofs

---

## Performance vs GNU Radio

| Operation | R4W | GNU Radio | Speedup |
|-----------|-----|-----------|---------|
| FFT 1024-pt | 371 MS/s | 50 MS/s | **7.4x** |
| FFT 4096-pt | 330 MS/s | 12 MS/s | **27x** |
| FFT 2048-pt | 179 MS/s | ~25 MS/s | **7x** |

Zero-copy lock-free buffers, no Python overhead.

---

## Real-Time Validation

| Metric | Target | Actual |
|--------|--------|--------|
| FFT p99 latency | < 100 us | **18 us** |
| BPSK roundtrip p99 | < 100 us | **20 us** |
| FHSS hop timing p99 | < 500 us | **80-118 us** |
| Page faults (RT mode) | 0 | **0** |
| Hot-path allocations | 0 | **0** |

---

## Getting Started

```bash
# Clone and build
git clone https://github.com/joemooney/r4w
cd r4w && cargo build --release

# Run the explorer
cargo run --bin r4w-explorer

# Run tests (527 tests)
cargo test

# Generate documentation
cargo doc --open
```

---

## Quick Commands

| Command | Description |
|---------|-------------|
| `make run` | Start GUI explorer |
| `make test` | Run all tests |
| `make bench` | Run benchmarks |
| `make web` | Start web app (port 8089) |
| `make doc-open` | View documentation |

---

## Resources

- **GitHub**: github.com/joemooney/r4w
- **Documentation**: `cargo doc --open`
- **Workshop**: `workshop/` directory
- **Tutorial**: `tutorial/index.html`

---

## Summary

| Feature | Status |
|---------|--------|
| 38+ Waveforms | Complete |
| FPGA Acceleration | Complete |
| Plugin System | Complete |
| Formal Verification | 6,324 lines |
| Test Coverage | 527 tests |
| C/C++ FFI | Complete |

**R4W: Production-ready SDR in Rust**

---

## Questions?

**R4W - Rust for Waveforms**

github.com/joemooney/r4w

