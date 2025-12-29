---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>Workshop Series'
subtitle: "From Beginner to Waveform Developer"
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

## Workshop Overview

11 hands-on workshops covering SDR fundamentals to advanced topics.

| Duration | Topics |
|----------|--------|
| **Total** | ~11 hours |
| **Beginner** | 4 workshops |
| **Intermediate** | 4 workshops |
| **Advanced** | 3 workshops |

---

## Learning Path

```
      ┌─────────────────────┐
      │   01: Installation  │
      └──────────┬──────────┘
                 │
      ┌──────────▼──────────┐
      │   02: I/Q Basics    │
      └──────────┬──────────┘
                 │
    ┌────────────┼────────────┐
    │            │            │
┌───▼───┐  ┌─────▼─────┐  ┌───▼───┐
│03: OOK│  │04: FSK/PSK│  │07:Chan│
└───┬───┘  └─────┬─────┘  └───┬───┘
    │            │            │
    └─────┬──────┘            │
          │                   │
     ┌────▼────┐              │
     │ 05: QAM │◄─────────────┘
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
    ┌─────┴─────┐
    │           │
┌───▼────┐ ┌────▼────┐
│09:Explr│ │10:RT/FPG│
└────────┘ └────┬────┘
                │
           ┌────▼────┐
           │11:RT Sch│
           └─────────┘
```

---

## Workshop 01: Getting Started

**Duration: 30 minutes**

| Topic | Content |
|-------|---------|
| Installation | Rust, cargo, dependencies |
| First Run | Build and run explorer |
| CLI Basics | Commands, help, examples |
| Project Tour | Directory structure |

```bash
git clone https://github.com/joemooney/r4w.git
cd r4w && cargo build --release
cargo run --bin r4w-explorer
```

---

## Workshop 02: I/Q Signals

**Duration: 45 minutes**

| Topic | Content |
|-------|---------|
| Complex Numbers | Real + imaginary |
| I/Q Samples | In-phase, quadrature |
| Time Domain | Waveform visualization |
| Sampling | Nyquist, aliasing |

Learn the foundation of all SDR!

---

## Workshop 03: Basic Modulation

**Duration: 45 minutes**

| Modulation | Description |
|------------|-------------|
| **CW** | Continuous wave (carrier only) |
| **OOK** | On-Off Keying |
| **ASK** | Amplitude Shift Keying |

Simplest forms of digital modulation.

---

## Workshop 04: FSK and PSK

**Duration: 60 minutes**

| Modulation | Bits/Symbol | Description |
|------------|-------------|-------------|
| **FSK** | 1 | Frequency shift |
| **GFSK** | 1 | Gaussian filtered FSK |
| **BPSK** | 1 | Binary phase shift |
| **QPSK** | 2 | Quadrature PSK |
| **8PSK** | 3 | 8-point PSK |

Frequency vs phase modulation trade-offs.

---

## Workshop 05: QAM

**Duration: 60 minutes**

| QAM Order | Bits/Symbol | Use Case |
|-----------|-------------|----------|
| 4-QAM | 2 | Same as QPSK |
| 16-QAM | 4 | WiFi, LTE |
| 64-QAM | 6 | High-rate WiFi |
| 256-QAM | 8 | Cable modems |

Combines amplitude and phase modulation.

---

## Constellation Diagrams

Visual representation of symbol positions:

```
         Q (Imaginary)
          │
    ●     │     ●      16-QAM:
          │            16 points
   ───────┼───────     4 bits/symbol
          │
    ●     │     ●
          │
        I (Real)
```

---

## Workshop 06: Spread Spectrum

**Duration: 75 minutes**

| Technique | Description |
|-----------|-------------|
| **DSSS** | Direct Sequence - spreading code |
| **FHSS** | Frequency Hopping - channel hopping |
| **CSS** | Chirp Spread Spectrum (LoRa) |

Trade bandwidth for noise immunity.

---

## LoRa Chirps

Chirp Spread Spectrum modulation:

| Parameter | Values |
|-----------|--------|
| Spreading Factor | SF5-SF12 |
| Bandwidth | 125/250/500 kHz |
| Chips/Symbol | 2^SF |
| Range | Up to 15 km |

Symbol = cyclic shift of base chirp.

---

## Workshop 07: Channel Effects

**Duration: 60 minutes**

| Effect | Description |
|--------|-------------|
| **AWGN** | Additive White Gaussian Noise |
| **Fading** | Signal strength variation |
| **Multipath** | Reflections, delays |
| **Doppler** | Frequency shift from motion |

Simulate real-world conditions.

---

## BER and SNR

| SNR (dB) | BPSK BER | QPSK BER |
|----------|----------|----------|
| 0 | 0.08 | 0.08 |
| 5 | 0.006 | 0.006 |
| 10 | 4e-6 | 4e-6 |
| 15 | 3e-12 | 3e-12 |

Higher SNR = lower bit errors.

---

## Workshop 08: Building Waveforms

**Duration: 90 minutes**

| Step | Task |
|------|------|
| 1 | Implement `Waveform` trait |
| 2 | Write `modulate()` |
| 3 | Write `demodulate()` |
| 4 | Define constellation |
| 5 | Add unit tests |
| 6 | Register in factory |

Build your own waveform!

---

## The Waveform Trait

```rust
pub trait Waveform: Send + Sync {
    fn info(&self) -> WaveformInfo;
    fn modulate(&self, bits: &[bool]) -> Vec<IQSample>;
    fn demodulate(&self, samples: &[IQSample])
        -> Vec<bool>;
    fn constellation_points(&self) -> Vec<IQSample>;
}
```

---

## Workshop 09: Explorer Deep Dive

**Duration: 60 minutes**

Every GUI feature explained:

| Panel | Features |
|-------|----------|
| **Overview** | Waveform intro, SDR concepts |
| **Waveform Lab** | Parameter tweaking |
| **Streaming** | Real-time visualization |
| **Code Explorer** | Implementation details |
| **Performance** | Benchmarks, metrics |

---

## Workshop 10: RT and FPGA

**Duration: 75 minutes**

| Topic | Content |
|-------|---------|
| Real-Time Concepts | Latency, jitter, deadlines |
| RT Primitives | Lock-free buffers, memory locking |
| FPGA Overview | IP cores, Zynq integration |
| Acceleration | FFT, FIR, chirp in hardware |

Prepare for production systems.

---

## Workshop 11: Real-Time Scheduling

**Duration: 90 minutes**

| Topic | Content |
|-------|---------|
| TickScheduler | Virtual time simulation |
| RealTimeScheduler | Wall-clock events |
| State Machine | TX/RX coordination |
| FHSS Implementation | Frequency hopping |
| TDMA Implementation | Time slot management |

---

## Workshop Format

Each workshop includes:

| Section | Purpose |
|---------|---------|
| **Objectives** | What you'll learn |
| **Background** | Theory and concepts |
| **Hands-On** | Code exercises |
| **Explorer** | GUI experiments |
| **Challenges** | Advanced exercises |
| **Key Takeaways** | Summary points |

---

## Prerequisites

| Requirement | Details |
|-------------|---------|
| **Rust** | `rustup` installed |
| **Git** | Clone repository |
| **Build tools** | C compiler, pkg-config |
| **Programming** | Basic experience |
| **Math** | Complex numbers helpful |

---

## Quick Reference

```bash
# Run the Explorer GUI
cargo run --bin r4w-explorer

# CLI commands
cargo run --bin r4w -- --help
cargo run --bin r4w -- waveform --list
cargo run --bin r4w -- modulate --message "Hello"

# Run in browser (WASM)
cd crates/r4w-web && trunk serve

# Run all tests
cargo test
```

---

## Workshop Materials

Each workshop has:

| File | Content |
|------|---------|
| `XX-topic.md` | Full workshop content |
| Code examples | In crates/r4w-core |
| Explorer tabs | Visual experiments |
| Exercises | Hands-on practice |

Location: `workshops/`

---

## Skills Progression

| Level | Workshops | Skills |
|-------|-----------|--------|
| **Beginner** | 01-03 | Basics, simple modulation |
| **Intermediate** | 04-07 | PSK, QAM, spread spectrum |
| **Advanced** | 08-11 | Implementation, RT, FPGA |

---

## Hands-On Examples

From Workshop 04 (BPSK):

```rust
// Generate BPSK samples
let bits = vec![true, false, true, true];
let waveform = BpskWaveform::new(1e6, 100e3);

let samples = waveform.modulate(&bits);
// Visualize in Explorer!

let recovered = waveform.demodulate(&samples);
assert_eq!(bits, recovered);
```

---

## Challenges

Each workshop includes optional challenges:

| Workshop | Challenge |
|----------|-----------|
| 03 | Add pulse shaping to OOK |
| 05 | Implement Gray coding |
| 06 | Build PN sequence generator |
| 08 | Create Manchester encoding |
| 11 | Implement CSMA/CA |

---

## Getting Help

| Resource | Location |
|----------|----------|
| GitHub Issues | github.com/joemooney/r4w/issues |
| Documentation | `cargo doc --open` |
| CLAUDE.md | Project guidance |
| OVERVIEW.md | Architecture reference |

---

## Summary

| Metric | Value |
|--------|-------|
| **Workshops** | 11 |
| **Total Duration** | ~11 hours |
| **Topics** | CW to FPGA |
| **Waveforms Covered** | 20+ |
| **Hands-On Exercises** | 50+ |

**From zero to waveform developer!**

---

## Questions?

**R4W - Workshop Series**

github.com/joemooney/r4w

Start: `workshops/01-getting-started.md`
