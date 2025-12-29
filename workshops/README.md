# R4W Developer Workshops

Welcome to the R4W (Rust for Waveforms) workshop series! These hands-on workshops will guide you from complete beginner to confident waveform developer.

## Workshop Index

| Workshop | Title | Topics | Duration |
|----------|-------|--------|----------|
| [01](01-getting-started.md) | Getting Started with R4W | Installation, first run, CLI basics | 30 min |
| [02](02-iq-signals.md) | Understanding I/Q Signals | Complex numbers, samples, time domain | 45 min |
| [03](03-basic-modulation.md) | Basic Modulation: CW, OOK, ASK | Carrier waves, on-off keying, amplitude shift | 45 min |
| [04](04-fsk-psk.md) | FSK and PSK Modulation | Frequency shift, phase shift keying | 60 min |
| [05](05-qam.md) | QAM and Constellation Diagrams | Quadrature amplitude modulation | 60 min |
| [06](06-spread-spectrum.md) | Spread Spectrum Techniques | DSSS, FHSS, LoRa chirps | 75 min |
| [07](07-channel-effects.md) | Channel Effects and Simulation | Noise, fading, multipath | 60 min |
| [08](08-building-waveforms.md) | Building Your Own Waveform | Waveform trait, implementation | 90 min |
| [09](09-explorer-deep-dive.md) | R4W Explorer Deep Dive | Every control explained | 60 min |
| [10](10-rt-fpga.md) | Real-Time and FPGA Concepts | RT primitives, FPGA acceleration | 75 min |
| [11](11-realtime-scheduling.md) | Real-Time Scheduling | TX/RX coordination, FHSS, TDMA | 90 min |

## Prerequisites

- Rust toolchain installed (`rustup` recommended)
- Basic programming knowledge
- Git for cloning the repository

## Getting the Code

```bash
git clone https://github.com/joemooney/r4w.git
cd r4w
cargo build --release
```

## Workshop Format

Each workshop includes:
- **Objectives**: What you'll learn
- **Background**: Theory and concepts
- **Hands-On**: Exercises with code
- **Explorer**: Visual experiments in the GUI
- **Challenges**: Optional advanced exercises
- **Key Takeaways**: Summary points

## Quick Reference

### Run the Explorer GUI
```bash
cargo run --bin r4w-explorer
```

### Run CLI Commands
```bash
# Get help
cargo run --bin r4w -- --help

# Modulate a message
cargo run --bin r4w -- modulate --message "Hello" --waveform bpsk

# Get waveform info
cargo run --bin r4w -- waveform --list
```

### Run in Browser (WASM)
```bash
cd crates/r4w-web
trunk serve
```

## Workshop Progression

```
          ┌─────────────────────┐
          │   01: Installation  │
          └──────────┬──────────┘
                     │
          ┌──────────▼──────────┐
          │   02: I/Q Basics    │
          └──────────┬──────────┘
                     │
    ┌────────────────┼────────────────┐
    │                │                │
┌───▼───┐      ┌─────▼─────┐     ┌────▼───┐
│ 03:   │      │ 04: FSK/  │     │ 07:    │
│OOK/ASK│      │ PSK       │     │Channel │
└───┬───┘      └─────┬─────┘     └────┬───┘
    │                │                │
    └───────┬────────┘                │
            │                         │
       ┌────▼────┐                    │
       │ 05: QAM │◄───────────────────┘
       └────┬────┘
            │
       ┌────▼───────┐
       │06: Spread  │
       │  Spectrum  │
       └────┬───────┘
            │
       ┌────▼────────────┐
       │08: Build Your   │
       │   Own Waveform  │
       └────┬────────────┘
            │
    ┌───────┴───────┐
    │               │
┌───▼────┐     ┌────▼────┐
│   09:  │     │   10:   │
│Explorer│     │ RT/FPGA │
└────────┘     └────┬────┘
                    │
               ┌────▼────┐
               │   11:   │
               │RT Sched │
               └─────────┘
```

## Getting Help

- [GitHub Issues](https://github.com/joemooney/r4w/issues)
- [CLAUDE.md](../CLAUDE.md) - Project guidance
- [OVERVIEW.md](../OVERVIEW.md) - Architecture reference

Happy learning!
