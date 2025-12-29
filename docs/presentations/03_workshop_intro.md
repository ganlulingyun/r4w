---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>R4W Workshop Introduction'
subtitle: "Setup, Goals, and Getting Started"
author: "R4W Development Team<br>(Aida, Joe Mooney, Claude Code)"
date: "December 2025"
header-includes: |
  <style>
    .reveal pre { margin: 0 auto; width: fit-content; }
    .reveal pre code { text-align: left; }
    .reveal section img { max-height: 60vh; }
  </style>
---

# Welcome to the R4W Workshop

## Workshop Goals

By the end of this workshop, you will be able to:

1. **Configure** USRP hardware for SDR experiments
2. **Understand** the R4W architecture and Waveform trait
3. **Transmit and receive** LoRa signals
4. **Measure** receiver sensitivity using automated testing
5. **Build** custom waveforms using the R4W framework

---

## Prerequisites

### Required
- Rust toolchain (1.70+): `rustup update stable`
- Linux or macOS (Windows via WSL)
- Basic signal processing knowledge

### Optional (for hardware labs)
- USRP N210 or B200 mini
- UHD driver installed
- RF cables and attenuator

---

## Environment Setup

```bash
# Clone repository
git clone https://github.com/org/r4w
cd r4w

# Build all crates
cargo build --release

# Run tests to verify setup
cargo test

# Start the GUI explorer
cargo run --bin r4w-explorer
```

---

## Repository Layout

```
r4w/
├── crates/
│   ├── r4w-core/        # Core DSP algorithms (421 tests)
│   ├── r4w-sim/         # HAL and simulation
│   ├── r4w-fpga/        # FPGA acceleration
│   └── r4w-sandbox/     # Waveform isolation
├── workshop/
│   ├── usrp/            # USRP exercises (01-09)
│   │   ├── configs/     # Device configurations
│   │   └── exercises/   # Rust exercise files
│   └── advanced/        # DSP deep dives (10-83)
```

---

## Exercise Format

Each exercise is a standalone Rust binary:

```bash
# Run with simulator (no hardware needed)
cargo run --example 01_device_discovery -- --simulator

# Run with real hardware
cargo run --example 01_device_discovery -- --device "uhd://type=b200"
```

All exercises include:
- Documentation header explaining concepts
- Working code you can modify
- `--help` for options

---

## USRP Workshop Track

| # | Exercise | Concepts |
|---|----------|----------|
| 01 | Device Discovery | Driver registry, enumeration |
| 02 | Basic RX | Spectrum analysis |
| 03 | Basic TX | Signal generation |
| 04 | Loopback | Attenuator, TX→RX |
| 05 | LoRa TX | CSS modulation |
| 06 | LoRa RX | Preamble detection, demod |
| 07 | Over-the-Air | Full link testing |
| 08 | Timing Sync | PPS, GPSDO, multi-device |
| 09 | Sensitivity | Automated SNR sweep |

---

## Advanced DSP Track

| Part | Topic | Exercises |
|------|-------|-----------|
| 1 | DSP Fundamentals | 10-13 |
| 2 | Modulation | 20-23 |
| 3 | Synchronization | 30-33 |
| 4 | Channel Effects | 40-43 |
| 5 | Error Control | 50-53 |
| 6 | Protocols | 60-63 |
| 7 | Performance | 70-73 |
| 8 | Custom Waveforms | 80-83 |

---

## Hardware Safety

### RF Transmission Rules

1. **Only transmit on frequencies you are licensed for**
2. Use low power settings during development
3. Use RF cables and attenuators when possible
4. ISM bands (915 MHz US, 868 MHz EU) for unlicensed testing
5. When in doubt, use the simulator

### Equipment Care

- Don't exceed 50Ω termination
- Never transmit into open connector
- Allow warm-up time for frequency stability

---

## Getting Help

### In Workshop
- Raise your hand
- Check exercise documentation
- Review error messages carefully

### Online
- GitHub Issues
- Documentation: `cargo doc --open`
- Code search: `rg "pattern" crates/`

---

## Let's Get Started!

### Exercise 01: Device Discovery

```bash
# With simulator
cargo run --example 01_device_discovery -- --simulator

# With hardware
cargo run --example 01_device_discovery
```

### What You'll See
- Available drivers
- Detected devices
- Device capabilities

---

## Questions Before We Start?
