---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>Waveform Development'
subtitle: "Building SDR Waveforms in Rust"
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

## What You'll Learn

Complete waveform development lifecycle:

| Topic | Coverage |
|-------|----------|
| **Architecture** | Waveform trait system |
| **Implementation** | Modulation/demodulation |
| **Testing** | Unit tests, BER validation |
| **Benchmarking** | Performance profiling |
| **Deployment** | Cross-compilation, targets |

---

## Project Structure

```
crates/
├── r4w-core/           # DSP and waveforms
│   ├── src/
│   │   ├── dsp/        # FFT, filters, chirp
│   │   └── waveform/   # All waveform implementations
│   │       ├── trait.rs
│   │       ├── factory.rs
│   │       ├── lora.rs
│   │       ├── psk.rs
│   │       └── fsk.rs
│   └── benches/        # Criterion benchmarks
├── r4w-sim/            # Channel simulation
├── r4w-fpga/           # FPGA acceleration
├── r4w-cli/            # Command-line interface
└── r4w-gui/            # GUI application
```

---

## The Waveform Trait

Every waveform implements:

```rust
pub trait Waveform: Send + Sync {
    fn info(&self) -> WaveformInfo;

    fn modulate(&self, bits: &[bool]) -> Vec<IQSample>;

    fn demodulate(&self, samples: &[IQSample])
        -> Vec<bool>;

    fn constellation_points(&self) -> Vec<IQSample>;

    // Educational methods
    fn get_modulation_stages(&self, bits: &[bool])
        -> Vec<ModulationStage>;
}
```

---

## IQSample Type

Complex I/Q sample representation:

```rust
#[derive(Clone, Copy, Debug)]
pub struct IQSample {
    pub i: f32,  // In-phase (real)
    pub q: f32,  // Quadrature (imaginary)
}

impl IQSample {
    pub fn magnitude(&self) -> f32 {
        (self.i * self.i + self.q * self.q).sqrt()
    }

    pub fn phase(&self) -> f32 {
        self.q.atan2(self.i)
    }
}
```

---

## Waveform Lifecycle

```
Transmit Path:
┌──────────┐   ┌──────────────┐   ┌───────────────┐
│  Data    │──►│  Modulator   │──►│  RF/Transport │
│  (bits)  │   │  (waveform)  │   │  (hardware)   │
└──────────┘   └──────────────┘   └───────────────┘

Receive Path:
┌───────────────┐   ┌──────────────┐   ┌──────────┐
│  RF/Transport │──►│ Demodulator  │──►│  Data    │
│  (hardware)   │   │  (waveform)  │   │  (bits)  │
└───────────────┘   └──────────────┘   └──────────┘
```

---

## Implementing BPSK

Basic phase shift keying:

```rust
pub struct BpskWaveform {
    sample_rate: f64,
    symbol_rate: f64,
    samples_per_symbol: usize,
}

impl Waveform for BpskWaveform {
    fn modulate(&self, bits: &[bool]) -> Vec<IQSample> {
        bits.iter()
            .flat_map(|&bit| {
                let phase = if bit { 0.0 } else { PI };
                self.generate_symbol(phase)
            })
            .collect()
    }
}
```

---

## BPSK Demodulation

```rust
fn demodulate(&self, samples: &[IQSample]) -> Vec<bool> {
    samples
        .chunks(self.samples_per_symbol)
        .map(|chunk| {
            // Integrate over symbol period
            let sum: f32 = chunk.iter()
                .map(|s| s.i)  // Real component
                .sum();
            sum > 0.0  // Decision threshold
        })
        .collect()
}
```

---

## Constellation Points

Define ideal symbol positions:

```rust
fn constellation_points(&self) -> Vec<IQSample> {
    vec![
        IQSample::new(-1.0, 0.0),  // Bit 0
        IQSample::new(1.0, 0.0),   // Bit 1
    ]
}
```

Used by GUI for visualization.

---

## 38+ Waveforms Implemented

| Category | Waveforms |
|----------|-----------|
| **Phase Shift** | BPSK, QPSK, 8PSK |
| **Amplitude** | AM, ASK, OOK |
| **Frequency** | FSK, GFSK, MSK |
| **Quadrature** | 16-QAM, 64-QAM, 256-QAM |
| **Spread Spectrum** | DSSS, FHSS, LoRa (CSS) |
| **Multi-carrier** | OFDM |
| **Military** | SINCGARS, HAVEQUICK |

---

## LoRa CSS Implementation

Chirp Spread Spectrum:

```rust
pub struct LoraWaveform {
    sf: u8,          // Spreading factor 5-12
    bw: f64,         // Bandwidth (125/250/500 kHz)
    chips_per_symbol: usize,
}

fn modulate_symbol(&self, symbol: u16) -> Vec<IQSample> {
    // Generate chirp with cyclic shift
    let shift = symbol as f64 / self.chips_per_symbol as f64;
    self.generate_chirp(shift)
}
```

---

## LoRa Demodulation

FFT-based chirp detection:

```rust
fn demodulate(&self, samples: &[IQSample]) -> Vec<bool> {
    let downchirp = self.generate_downchirp();

    samples.chunks(self.chips_per_symbol)
        .flat_map(|chunk| {
            // Multiply by downchirp
            let product = self.multiply(chunk, &downchirp);

            // FFT to find peak
            let spectrum = fft(&product);
            let peak_bin = find_peak(&spectrum);

            // Convert bin to symbol to bits
            symbol_to_bits(peak_bin, self.sf)
        })
        .collect()
}
```

---

## WaveformInfo Metadata

```rust
pub struct WaveformInfo {
    pub name: String,
    pub description: String,
    pub modulation_type: ModulationType,
    pub bits_per_symbol: usize,
    pub sample_rate: f64,
    pub symbol_rate: f64,
    pub bandwidth: f64,
}
```

Returned by `info()` method.

---

## Educational Stages

Show modulation pipeline:

```rust
fn get_modulation_stages(&self, bits: &[bool])
    -> Vec<ModulationStage>
{
    vec![
        ModulationStage {
            name: "Input Bits",
            data: StageData::Bits(bits.to_vec()),
        },
        ModulationStage {
            name: "Symbol Mapping",
            data: StageData::Symbols(self.map_symbols(bits)),
        },
        ModulationStage {
            name: "Pulse Shaping",
            data: StageData::Samples(self.modulate(bits)),
        },
    ]
}
```

---

## Waveform Factory

Dynamic waveform creation:

```rust
use r4w_core::waveform::WaveformFactory;

let waveform = WaveformFactory::create(
    "lora",
    WaveformConfig {
        sample_rate: 1e6,
        spreading_factor: Some(7),
        bandwidth: Some(125e3),
        ..Default::default()
    }
)?;
```

---

## Unit Testing

Test modulate/demodulate roundtrip:

```rust
#[test]
fn test_bpsk_roundtrip() {
    let waveform = BpskWaveform::new(1e6, 100e3);
    let bits = vec![true, false, true, true, false];

    let samples = waveform.modulate(&bits);
    let recovered = waveform.demodulate(&samples);

    assert_eq!(bits, recovered);
}
```

---

## BER Testing

Bit Error Rate with channel:

```rust
#[test]
fn test_bpsk_ber_at_10db() {
    let waveform = BpskWaveform::new(1e6, 100e3);
    let channel = AwgnChannel::new(10.0);  // 10 dB SNR

    let bits: Vec<bool> = (0..10000)
        .map(|_| rand::random())
        .collect();

    let samples = waveform.modulate(&bits);
    let noisy = channel.apply(&samples);
    let recovered = waveform.demodulate(&noisy);

    let ber = count_errors(&bits, &recovered) as f64
        / bits.len() as f64;

    assert!(ber < 0.01);  // BER < 1%
}
```

---

## Benchmarking

Criterion benchmarks:

```rust
// benches/waveform_bench.rs
use criterion::{criterion_group, Criterion};

fn bench_lora_modulate(c: &mut Criterion) {
    let waveform = LoraWaveform::new(7, 125e3);
    let bits = vec![true; 1024];

    c.bench_function("lora_sf7_modulate", |b| {
        b.iter(|| waveform.modulate(&bits))
    });
}
```

Run: `cargo bench`

---

## Performance Results

| Waveform | Throughput | Latency p99 |
|----------|------------|-------------|
| BPSK | 25.4 K/s | 20 us |
| QPSK | 22.1 K/s | 25 us |
| LoRa SF7 | 45.6 K/s | 18 us |
| 16-QAM | 18.3 K/s | 35 us |
| OFDM | 12.7 K/s | 80 us |

---

## Cross-Compilation

Build for embedded targets:

```bash
# ARM64 (Raspberry Pi 4)
rustup target add aarch64-unknown-linux-gnu
cargo build --target aarch64-unknown-linux-gnu

# ARM32 (older Pi)
rustup target add armv7-unknown-linux-gnueabihf
cargo build --target armv7-unknown-linux-gnueabihf

# Static binary
cargo build --target x86_64-unknown-linux-musl
```

---

## Deployment

```bash
# Build CLI
make build-cli-arm64

# Deploy to target
make deploy-arm64 REMOTE_HOST=user@pi

# Run on target
ssh user@pi "./r4w info --waveform lora"
```

---

## Adding a New Waveform

1. Create `crates/r4w-core/src/waveform/<name>.rs`
2. Implement `Waveform` trait
3. Add to `mod.rs` and factory
4. Write unit tests
5. Add benchmarks
6. Document in guide

---

## Example: Custom FSK

```rust
pub struct CustomFskWaveform {
    deviation: f64,
    symbol_rate: f64,
}

impl Waveform for CustomFskWaveform {
    fn modulate(&self, bits: &[bool]) -> Vec<IQSample> {
        let mut phase = 0.0;
        bits.iter()
            .flat_map(|&bit| {
                let freq = if bit {
                    self.deviation
                } else {
                    -self.deviation
                };
                self.generate_tone(freq, &mut phase)
            })
            .collect()
    }
}
```

---

## IDE Setup (VS Code)

```json
// .vscode/settings.json
{
    "rust-analyzer.cargo.features": ["all"],
    "rust-analyzer.checkOnSave.command": "clippy",
    "rust-analyzer.lens.enable": true,
    "editor.formatOnSave": true
}
```

Extensions: rust-analyzer, CodeLLDB, Error Lens

---

## Debugging Tips

| Tool | Purpose |
|------|---------|
| `cargo test -- --nocapture` | See println output |
| `RUST_BACKTRACE=1` | Full stack traces |
| CodeLLDB | Step debugging |
| `cargo flamegraph` | CPU profiling |
| `cargo bloat` | Binary size analysis |

---

## Summary

| Step | Action |
|------|--------|
| 1 | Implement `Waveform` trait |
| 2 | Add modulate/demodulate |
| 3 | Define constellation |
| 4 | Write unit tests |
| 5 | Add to factory |
| 6 | Benchmark performance |
| 7 | Cross-compile if needed |

**38+ waveforms. Your next one.**

---

## Questions?

**R4W - Waveform Development**

github.com/joemooney/r4w

Docs: `docs/WAVEFORM_DEVELOPERS_GUIDE.md`
