---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>R4W: Rust for Waveforms'
subtitle: "An Open Platform for SDR Development"
author: "R4W Development Team<br>(Aida, Joe Mooney, Claude Code)"
date: "December 2025"
header-includes: |
  <style>
    .reveal pre { margin: 0 auto; width: fit-content; }
    .reveal pre code { text-align: left; }
    .reveal section img { max-height: 60vh; }
  </style>
---

# R4W Platform Overview

## What is R4W?

**R4W (Rust for Waveforms)** is an open platform for developing, testing, and deploying Software Defined Radio waveforms in Rust.

### Mission
Enable engineers to build secure, high-performance SDR systems with modern tooling.

### Vision
Replace ad-hoc C/C++ SDR code with maintainable, testable, safe Rust.

---

## Why Another SDR Platform?

| Pain Point | Traditional Approach | R4W Solution |
|------------|---------------------|--------------|
| Memory bugs | Runtime crashes, sanitizers | Compile-time safety |
| Buffer overflows | Code review, fuzzing | Bounds checking |
| Data races | Mutex discipline | Ownership model |
| Dependency hell | Manual tracking | Cargo package manager |
| Testing | Often minimal | 527 tests included |

---

## Core Capabilities

### Already Implemented

- **84,467 lines** of code across 359 files
- **66,572 lines** of Rust (527 tests)
- **6,324 lines** of Coq formal verification proofs
- **38+ waveforms**: LoRa, PSK, QAM, FSK, OFDM, FHSS, military
- **Plugin system** for dynamic waveform loading
- **FPGA acceleration** (Xilinx Zynq + Lattice iCE40/ECP5)
- **Waveform sandbox** with 8 isolation levels
- **C/C++ FFI** with CMake integration

---

## Architecture

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

## Performance vs GNU Radio

| Operation | R4W | GNU Radio | Speedup |
|-----------|-----|-----------|---------|
| FFT 1024-pt | 371 MS/s | 50 MS/s | **7.4x** |
| FFT 4096-pt | 330 MS/s | 12 MS/s | **27x** |
| BPSK roundtrip p99 | 20 µs | ~100 µs | **5x** |

*Zero-copy lock-free buffers + Rust optimizations*

---

## Real-Time Guarantees

### Validated Metrics

| Metric | Target | Actual |
|--------|--------|--------|
| FFT p99 latency | < 100 µs | **18 µs** |
| FHSS hop timing p99 | < 500 µs | **80-118 µs** |
| Page faults (RT mode) | 0 | **0** |
| Hot-path allocations | 0 | **0** |

---

## Hardware Support

### Available Now

| Hardware | Status | Notes |
|----------|--------|-------|
| USRP N210/B200 | UHD Driver | Full TX/RX/FD |
| RTL-SDR | FFI Bindings | RX only |
| Xilinx Zynq | IP cores + mmap | FFT, FIR, Chirp, NCO, DMA |
| Lattice iCE40/ECP5 | Open toolchain | Yosys + nextpnr |
| Digital Attenuator | Control API | PE43711, Mini-Circuits |
| Simulator | Complete | UDP transport |

### In Progress

- SoapySDR generic wrapper
- Real hardware driver activation

---

## Waveform Development

### The Waveform Trait

```rust
pub trait Waveform {
    fn modulate(&self, bits: &[bool]) -> Vec<IQSample>;
    fn demodulate(&self, samples: &[IQSample]) -> Vec<bool>;
    fn constellation_points(&self) -> Vec<IQSample>;
}
```

### Example: LoRa Modulation

```rust
let lora = LoRaWaveform::new(7, 125_000.0);
let samples = lora.modulate(&bits);
```

---

## Workshop Materials

### USRP Exercises (9 labs)
1. Device Discovery
2. Basic RX/TX
3. Loopback with Attenuator
4. LoRa TX/RX
5. Over-the-Air Link
6. Timing Synchronization
7. Sensitivity Testing

### Advanced DSP (30+ exercises)
- DSP Fundamentals
- Modulation Deep Dive
- Synchronization
- Channel Effects
- Error Control

---

## Measurable Success Criteria

| Objective | Target | Validation |
|-----------|--------|------------|
| LoRa interop | Decode from Semtech | Hardware test |
| BER @ 10dB SNR | < 1e-3 | Automated test |
| Sensitivity | -120 dBm | Attenuator sweep |
| Latency p99 | < 100 µs | Benchmark suite |
| Zero RT allocations | 0 malloc in hot path | Audit tool |

---

## C/C++ Migration Path

### Phase 1: Use as Library
```c
#include <r4w.h>
r4w_waveform_t* wf = r4w_waveform_bpsk(48000.0, 1200.0);
r4w_modulate(wf, bits, len, samples);
```

### Phase 2: Port Modules
Start with pure functions, gradually migrate.

### Phase 3: Full Rust
New development in Rust, legacy maintained.

---

## Development Velocity

### What Rust Gives You

- **cargo test** - Run 527 tests instantly
- **cargo bench** - Performance regression detection
- **cargo doc** - Auto-generated documentation
- **cargo clippy** - Static analysis
- **IDE support** - rust-analyzer for all editors

---

## What's Been Completed

### Core Platform ✓
- [x] 38+ waveforms (LoRa, PSK, QAM, FSK, OFDM, military)
- [x] Plugin system with dynamic loading
- [x] 6,324 lines Coq formal verification
- [x] C/C++ FFI with CMake integration

### FPGA ✓
- [x] Xilinx Zynq IP cores (FFT, FIR, Chirp, NCO, DMA)
- [x] Lattice iCE40/ECP5 with open toolchain
- [x] Vivado and Yosys/nextpnr build scripts

### Security ✓
- [x] 8-level waveform isolation (sandbox to air gap)
- [x] Crypto boundary architecture (CSI)
- [x] Secure memory primitives

### Next Steps
- [ ] Real hardware driver activation
- [ ] Over-the-air validation
- [ ] Multi-radio synchronization

---

## Getting Started

```bash
# Clone and build
git clone https://github.com/org/r4w
cd r4w && cargo build --release

# Run GUI explorer
cargo run --bin r4w-explorer

# Run CLI
cargo run --bin r4w -- simulate --sf 7 --snr 10

# Run tests
cargo test

# Run benchmarks
cargo bench
```

---

## Questions?

### Resources
- Documentation: `cargo doc --open`
- Workshops: `workshop/` directory
- Benchmarks: `cargo bench`
- Tests: `cargo test`

### Contact
- GitHub Issues for bugs/features
- Discussions for questions
