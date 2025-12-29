# Waveform Developer's Guide

**R4W - Rust for Waveforms**
*Comprehensive Guide for Waveform Software Engineers*

---

## Table of Contents

1. [Introduction](#introduction)
2. [Development Environment](#development-environment)
3. [Waveform Architecture](#waveform-architecture)
4. [Implementing Waveforms](#implementing-waveforms)
5. [Debugging Techniques](#debugging-techniques)
6. [Benchmarking and Profiling](#benchmarking-and-profiling)
7. [Cross-Compilation](#cross-compilation)
8. [Deployment](#deployment)
9. [System Integration](#system-integration)
10. [Memory and CPU Constraints](#memory-and-cpu-constraints)
11. [Shared Memory IPC](#shared-memory-ipc)
12. [Real-Time Processing](#real-time-processing)
13. [Testing Strategies](#testing-strategies)
14. [FPGA Acceleration](#fpga-acceleration)
15. [Security Considerations](#security-considerations)

---

## Introduction

### Purpose

This guide is designed for software engineers developing waveforms on the R4W platform. It covers the complete development lifecycle from implementation to deployment, including debugging, optimization, and system integration.

### Prerequisites

- Rust programming experience (intermediate level)
- Understanding of digital signal processing fundamentals
- Familiarity with modulation techniques (PSK, FSK, QAM)
- Basic Linux system administration skills

### What This Guide Covers

| Topic | Description |
|-------|-------------|
| **Implementation** | Creating waveforms using the R4W trait system |
| **Debugging** | Tools and techniques for finding issues |
| **Benchmarking** | Measuring and optimizing performance |
| **Cross-Compilation** | Building for ARM, PowerPC, embedded targets |
| **Deployment** | Getting waveforms running on target hardware |
| **Integration** | Working with existing systems and hardware |
| **Real-Time** | Meeting timing constraints |

---

## Development Environment

### Required Tools

```bash
# Core development tools
rustup install stable
rustup target add aarch64-unknown-linux-gnu  # ARM64
rustup target add armv7-unknown-linux-gnueabihf  # ARM32
rustup target add x86_64-unknown-linux-musl  # Static linking

# Build essentials
sudo apt install build-essential pkg-config libssl-dev

# Cross-compilation toolchains
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf

# Analysis tools
cargo install cargo-flamegraph  # Profiling
cargo install cargo-bloat       # Binary size analysis
cargo install cargo-asm         # Assembly inspection
cargo install criterion         # Benchmarking (via cargo bench)

# Optional: GUI development
sudo apt install libgtk-3-dev libxcb-render0-dev libxcb-shape0-dev
```

### Project Structure

```
ai-sdr-lora/
├── crates/
│   ├── r4w-core/           # DSP algorithms and waveform trait
│   │   ├── src/
│   │   │   ├── dsp/        # FFT, filters, chirp generation
│   │   │   ├── waveform/   # Waveform implementations
│   │   │   │   ├── mod.rs
│   │   │   │   ├── trait.rs
│   │   │   │   ├── factory.rs
│   │   │   │   ├── lora.rs
│   │   │   │   ├── psk.rs
│   │   │   │   ├── fsk.rs
│   │   │   │   └── ...
│   │   │   └── lib.rs
│   │   └── benches/        # Criterion benchmarks
│   ├── r4w-sim/            # Channel simulation
│   ├── r4w-fpga/           # FPGA acceleration
│   ├── r4w-cli/            # Command-line interface
│   └── r4w-gui/            # GUI application
├── vivado/                 # Xilinx FPGA designs
├── lattice/                # Lattice FPGA designs
└── docs/                   # Documentation
```

### IDE Setup

**VS Code (Recommended):**

```json
// .vscode/settings.json
{
    "rust-analyzer.cargo.features": ["all"],
    "rust-analyzer.checkOnSave.command": "clippy",
    "rust-analyzer.lens.enable": true,
    "rust-analyzer.inlayHints.enable": true,
    "editor.formatOnSave": true
}
```

**Recommended Extensions:**
- rust-analyzer
- CodeLLDB (debugging)
- Error Lens
- Better TOML

---

## Waveform Architecture

### The Waveform Trait

Every waveform in R4W implements the `Waveform` trait:

```rust
pub trait Waveform: Send + Sync {
    /// Get waveform metadata
    fn info(&self) -> WaveformInfo;

    /// Modulate bits to I/Q samples
    fn modulate(&self, bits: &[bool]) -> Vec<IQSample>;

    /// Demodulate I/Q samples to bits
    fn demodulate(&self, samples: &[IQSample]) -> Vec<bool>;

    /// Constellation points for visualization
    fn constellation_points(&self) -> Vec<IQSample>;

    /// Educational: show modulation pipeline stages
    fn get_modulation_stages(&self, bits: &[bool]) -> Vec<ModulationStage>;

    /// Educational: show demodulation pipeline stages
    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep>;
}
```

### Data Types

```rust
/// Complex I/Q sample (32-bit float precision)
#[derive(Clone, Copy, Debug)]
pub struct IQSample {
    pub i: f32,  // In-phase (real)
    pub q: f32,  // Quadrature (imaginary)
}

impl IQSample {
    pub fn new(i: f32, q: f32) -> Self {
        Self { i, q }
    }

    pub fn magnitude(&self) -> f32 {
        (self.i * self.i + self.q * self.q).sqrt()
    }

    pub fn phase(&self) -> f32 {
        self.q.atan2(self.i)
    }
}
```

### Waveform Lifecycle

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Waveform Processing Flow                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  Transmit Path:                                                             │
│  ┌──────────┐   ┌──────────────┐   ┌───────────────┐   ┌─────────────────┐  │
│  │  Data    │──►│  Modulator   │──►│  Channel Sim  │──►│  RF/Transport   │  │
│  │  (bits)  │   │  (waveform)  │   │  (AWGN, etc)  │   │  (UDP/hardware) │  │
│  └──────────┘   └──────────────┘   └───────────────┘   └─────────────────┘  │
│                                                                             │
│  Receive Path:                                                              │
│  ┌─────────────────┐   ┌───────────────┐   ┌──────────────┐   ┌──────────┐  │
│  │  RF/Transport   │──►│  Demodulator  │──►│  Bit Decoder │──►│  Data    │  │
│  │  (samples)      │   │  (waveform)   │   │  (optional)  │   │  (bits)  │  │
│  └─────────────────┘   └───────────────┘   └──────────────┘   └──────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Implementing Waveforms

### Step 1: Create the Waveform Module

```rust
// crates/r4w-core/src/waveform/my_waveform.rs

use crate::types::IQSample;
use crate::waveform::{
    Waveform, WaveformInfo, ModulationStage, DemodulationStep
};
use std::f32::consts::PI;

/// My custom waveform implementation
pub struct MyWaveform {
    sample_rate: f64,
    symbol_rate: f64,
    samples_per_symbol: usize,
}

impl MyWaveform {
    pub fn new(sample_rate: f64, symbol_rate: f64) -> Self {
        let samples_per_symbol = (sample_rate / symbol_rate) as usize;
        Self {
            sample_rate,
            symbol_rate,
            samples_per_symbol,
        }
    }

    /// Default configuration
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, sample_rate / 10.0)
    }
}
```

### Step 2: Implement the Waveform Trait

```rust
impl Waveform for MyWaveform {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "MyWave",
            full_name: "My Custom Waveform",
            bits_per_symbol: 2,
            sample_rate: self.sample_rate,
            symbol_rate: self.symbol_rate,
            carries_data: true,
        }
    }

    fn modulate(&self, bits: &[bool]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(
            (bits.len() / 2) * self.samples_per_symbol
        );

        // Process bits in pairs (QPSK-like)
        for chunk in bits.chunks(2) {
            let (b0, b1) = (
                chunk.get(0).copied().unwrap_or(false),
                chunk.get(1).copied().unwrap_or(false),
            );

            // Gray-coded constellation
            let symbol = match (b0, b1) {
                (false, false) => IQSample::new(-0.707, -0.707),  // 225°
                (false, true)  => IQSample::new(-0.707,  0.707),  // 135°
                (true,  true)  => IQSample::new( 0.707,  0.707),  // 45°
                (true,  false) => IQSample::new( 0.707, -0.707),  // 315°
            };

            // Repeat symbol for samples_per_symbol
            for _ in 0..self.samples_per_symbol {
                samples.push(symbol);
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> Vec<bool> {
        let mut bits = Vec::new();

        for chunk in samples.chunks(self.samples_per_symbol) {
            if chunk.is_empty() {
                continue;
            }

            // Average samples in symbol period (coherent detection)
            let sum: (f32, f32) = chunk.iter()
                .fold((0.0, 0.0), |acc, s| (acc.0 + s.i, acc.1 + s.q));
            let avg = IQSample::new(
                sum.0 / chunk.len() as f32,
                sum.1 / chunk.len() as f32,
            );

            // Decision regions (Gray-coded)
            let i_positive = avg.i > 0.0;
            let q_positive = avg.q > 0.0;

            bits.push(i_positive);
            bits.push(i_positive ^ !q_positive);  // Gray decode
        }

        bits
    }

    fn constellation_points(&self) -> Vec<IQSample> {
        vec![
            IQSample::new(-0.707, -0.707),  // 00
            IQSample::new(-0.707,  0.707),  // 01
            IQSample::new( 0.707,  0.707),  // 11
            IQSample::new( 0.707, -0.707),  // 10
        ]
    }

    fn get_modulation_stages(&self, bits: &[bool]) -> Vec<ModulationStage> {
        vec![
            ModulationStage {
                name: "Input Bits".to_string(),
                description: format!("{} bits input", bits.len()),
                samples: vec![],
            },
            ModulationStage {
                name: "Symbol Mapping".to_string(),
                description: "Map bit pairs to constellation points".to_string(),
                samples: self.modulate(bits),
            },
        ]
    }

    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        vec![
            DemodulationStep {
                name: "Symbol Detection".to_string(),
                description: "Find nearest constellation point".to_string(),
                data: samples.to_vec(),
                bits: self.demodulate(samples),
            },
        ]
    }
}
```

### Step 3: Register with Factory

```rust
// In crates/r4w-core/src/waveform/factory.rs

impl WaveformFactory {
    pub fn create(name: &str, sample_rate: f64) -> Option<Box<dyn Waveform>> {
        match name.to_uppercase().as_str() {
            "BPSK" => Some(Box::new(BpskWaveform::new(sample_rate))),
            "QPSK" => Some(Box::new(QpskWaveform::new(sample_rate))),
            "LORA" => Some(Box::new(LoRaWaveform::default_config(sample_rate))),
            "MYWAVE" => Some(Box::new(MyWaveform::default_config(sample_rate))),
            _ => None,
        }
    }

    pub fn list() -> Vec<&'static str> {
        vec!["BPSK", "QPSK", "LoRa", "MyWave"]
    }
}
```

### Step 4: Add Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use r4w_sim::channel::AwgnChannel;

    #[test]
    fn test_roundtrip_clean() {
        let waveform = MyWaveform::default_config(48000.0);
        let original = vec![true, false, true, true, false, false, true, false];

        let samples = waveform.modulate(&original);
        let recovered = waveform.demodulate(&samples);

        assert_eq!(original, recovered);
    }

    #[test]
    fn test_roundtrip_noisy() {
        let waveform = MyWaveform::default_config(48000.0);
        let original: Vec<bool> = (0..1000).map(|i| i % 3 == 0).collect();

        let samples = waveform.modulate(&original);
        let noisy = AwgnChannel::new(15.0).apply(&samples);  // 15 dB SNR
        let recovered = waveform.demodulate(&noisy);

        let errors = original.iter().zip(&recovered)
            .filter(|(a, b)| a != b)
            .count();
        let ber = errors as f64 / original.len() as f64;

        assert!(ber < 0.01, "BER {:.4} exceeds 1%", ber);
    }

    #[test]
    fn test_info() {
        let waveform = MyWaveform::default_config(48000.0);
        let info = waveform.info();

        assert_eq!(info.name, "MyWave");
        assert_eq!(info.bits_per_symbol, 2);
    }

    #[test]
    fn test_constellation() {
        let waveform = MyWaveform::default_config(48000.0);
        let points = waveform.constellation_points();

        assert_eq!(points.len(), 4);
        for point in &points {
            let magnitude = point.magnitude();
            assert!((magnitude - 1.0).abs() < 0.01, "Point not unit magnitude");
        }
    }
}
```

---

## Debugging Techniques

### Log-Based Debugging

```rust
use tracing::{debug, info, warn, error, instrument};

impl MyWaveform {
    #[instrument(skip(self, samples), fields(n_samples = samples.len()))]
    pub fn demodulate_debug(&self, samples: &[IQSample]) -> Vec<bool> {
        let mut bits = Vec::new();

        for (i, chunk) in samples.chunks(self.samples_per_symbol).enumerate() {
            let sum: (f32, f32) = chunk.iter()
                .fold((0.0, 0.0), |acc, s| (acc.0 + s.i, acc.1 + s.q));
            let avg = IQSample::new(
                sum.0 / chunk.len() as f32,
                sum.1 / chunk.len() as f32,
            );

            debug!(
                symbol = i,
                avg_i = %avg.i,
                avg_q = %avg.q,
                magnitude = %avg.magnitude(),
                phase_deg = %(avg.phase() * 180.0 / std::f32::consts::PI),
                "Processing symbol"
            );

            let (b0, b1) = (avg.i > 0.0, avg.q > 0.0);
            bits.push(b0);
            bits.push(b1);
        }

        info!(n_bits = bits.len(), "Demodulation complete");
        bits
    }
}

// Initialize tracing in your application
fn init_tracing() {
    tracing_subscriber::fmt()
        .with_env_filter("r4w_core=debug")
        .with_target(false)
        .init();
}
```

### Visual Debugging with GUI

The R4W GUI provides visual debugging tools:

```bash
# Run GUI explorer
cargo run --bin r4w-explorer

# Navigate to:
# - Constellation view: See symbol decision regions
# - Spectrum view: Frequency domain analysis
# - Waterfall view: Time-frequency visualization
# - Pipeline view: Step-by-step modulation/demodulation
```

### Dumping Samples to File

```rust
use std::fs::File;
use std::io::Write;

fn dump_samples_to_csv(samples: &[IQSample], path: &str) -> std::io::Result<()> {
    let mut file = File::create(path)?;
    writeln!(file, "index,i,q,magnitude,phase_deg")?;

    for (idx, sample) in samples.iter().enumerate() {
        writeln!(
            file,
            "{},{:.6},{:.6},{:.6},{:.2}",
            idx,
            sample.i,
            sample.q,
            sample.magnitude(),
            sample.phase() * 180.0 / std::f32::consts::PI
        )?;
    }

    Ok(())
}

// Use in tests
#[test]
fn debug_modulation() {
    let waveform = MyWaveform::default_config(48000.0);
    let bits = vec![true, false, true, true];
    let samples = waveform.modulate(&bits);

    dump_samples_to_csv(&samples, "/tmp/modulated.csv").unwrap();
    // Open in Python/MATLAB/Excel for analysis
}
```

### Interactive Debugging with LLDB

```bash
# Build with debug info
cargo build

# Debug with CodeLLDB (VS Code) or command line
lldb target/debug/r4w

(lldb) breakpoint set --name "my_waveform::MyWaveform::demodulate"
(lldb) run simulate --waveform MyWave --message "test"
```

### Signal Analysis with Python

```python
#!/usr/bin/env python3
# analyze_samples.py

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Load samples exported from R4W
data = np.loadtxt('/tmp/modulated.csv', delimiter=',', skiprows=1)
i_samples = data[:, 1]
q_samples = data[:, 2]
complex_samples = i_samples + 1j * q_samples

# Constellation diagram
plt.figure(figsize=(10, 4))
plt.subplot(1, 3, 1)
plt.scatter(i_samples, q_samples, alpha=0.5)
plt.xlabel('I')
plt.ylabel('Q')
plt.title('Constellation')
plt.axis('equal')
plt.grid(True)

# Power spectrum
plt.subplot(1, 3, 2)
f, psd = signal.welch(complex_samples, fs=48000, nperseg=256)
plt.semilogy(f, psd)
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD')
plt.title('Power Spectrum')

# Eye diagram (for PSK)
samples_per_symbol = 10
plt.subplot(1, 3, 3)
for start in range(0, len(i_samples) - 2*samples_per_symbol, samples_per_symbol):
    segment = i_samples[start:start + 2*samples_per_symbol]
    plt.plot(segment, 'b-', alpha=0.1)
plt.xlabel('Sample')
plt.ylabel('I')
plt.title('Eye Diagram')

plt.tight_layout()
plt.savefig('/tmp/signal_analysis.png')
plt.show()
```

---

## Benchmarking and Profiling

### Criterion Benchmarks

Create benchmarks in `crates/r4w-core/benches/`:

```rust
// benches/waveform_bench.rs

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use r4w_core::waveform::{Waveform, WaveformFactory};

fn bench_modulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("modulation");

    for &bit_count in &[100, 1000, 10000] {
        let bits: Vec<bool> = (0..bit_count).map(|i| i % 2 == 0).collect();

        for waveform_name in ["BPSK", "QPSK", "LoRa"] {
            let waveform = WaveformFactory::create(waveform_name, 48000.0).unwrap();

            group.bench_with_input(
                BenchmarkId::new(waveform_name, bit_count),
                &bits,
                |b, bits| {
                    b.iter(|| waveform.modulate(black_box(bits)))
                },
            );
        }
    }

    group.finish();
}

fn bench_demodulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("demodulation");

    for &sample_count in &[1000, 10000, 100000] {
        for waveform_name in ["BPSK", "QPSK", "LoRa"] {
            let waveform = WaveformFactory::create(waveform_name, 48000.0).unwrap();

            // Generate samples
            let bit_count = sample_count / 10;  // Approximate
            let bits: Vec<bool> = (0..bit_count).map(|i| i % 2 == 0).collect();
            let samples = waveform.modulate(&bits);

            group.bench_with_input(
                BenchmarkId::new(waveform_name, samples.len()),
                &samples,
                |b, samples| {
                    b.iter(|| waveform.demodulate(black_box(samples)))
                },
            );
        }
    }

    group.finish();
}

fn bench_fft(c: &mut Criterion) {
    use r4w_core::dsp::fft::Fft;

    let mut group = c.benchmark_group("fft");

    for &size in &[256, 1024, 4096] {
        let samples: Vec<_> = (0..size)
            .map(|i| IQSample::new((i as f32).cos(), (i as f32).sin()))
            .collect();

        let mut fft = Fft::new(size);

        group.bench_with_input(
            BenchmarkId::from_parameter(size),
            &samples,
            |b, samples| {
                b.iter(|| fft.forward(black_box(samples)))
            },
        );
    }

    group.finish();
}

criterion_group!(benches, bench_modulation, bench_demodulation, bench_fft);
criterion_main!(benches);
```

### Running Benchmarks

```bash
# Run all benchmarks
cargo bench

# Run specific benchmark
cargo bench -- modulation

# Generate HTML report
cargo bench -- --save-baseline main

# Compare to baseline
cargo bench -- --baseline main

# Output to JSON for CI
cargo bench -- --format json > bench_results.json
```

### Flamegraph Profiling

```bash
# Install perf and flamegraph
sudo apt install linux-tools-common linux-tools-generic
cargo install flamegraph

# Profile release build
cargo flamegraph --bin r4w -- simulate --waveform LoRa --message "test" --duration 10

# Output: flamegraph.svg
```

### Memory Profiling

```bash
# Using valgrind/massif
cargo build --release
valgrind --tool=massif target/release/r4w simulate --waveform LoRa

# Using heaptrack (better for Rust)
heaptrack target/release/r4w simulate --waveform LoRa
heaptrack_gui heaptrack.r4w.*.zst
```

### Performance Metrics

Track these metrics for your waveforms:

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Modulation throughput | >1 Msps | `criterion` benchmark |
| Demodulation throughput | >500 ksps | `criterion` benchmark |
| Latency (single symbol) | <100 μs | `std::time::Instant` |
| Memory per 1k samples | <100 KB | `heaptrack` |
| CPU usage (real-time) | <50% | `top` / `htop` |

---

## Cross-Compilation

### Target Configuration

```toml
# .cargo/config.toml

[target.aarch64-unknown-linux-gnu]
linker = "aarch64-linux-gnu-gcc"

[target.armv7-unknown-linux-gnueabihf]
linker = "arm-linux-gnueabihf-gcc"

[target.x86_64-unknown-linux-musl]
linker = "musl-gcc"

[target.powerpc-unknown-linux-gnu]
linker = "powerpc-linux-gnu-gcc"

# For Zynq (bare metal)
[target.armv7-unknown-none-eabihf]
linker = "arm-none-eabi-gcc"
```

### Building for ARM64 (Raspberry Pi 4/5)

```bash
# Install toolchain
sudo apt install gcc-aarch64-linux-gnu
rustup target add aarch64-unknown-linux-gnu

# Build
cargo build --release --target aarch64-unknown-linux-gnu --bin r4w

# Verify
file target/aarch64-unknown-linux-gnu/release/r4w
# Output: ELF 64-bit LSB executable, ARM aarch64

# Deploy
scp target/aarch64-unknown-linux-gnu/release/r4w pi@raspberrypi:/home/pi/
```

### Building for ARM32 (Raspberry Pi 3, BeagleBone)

```bash
# Install toolchain
sudo apt install gcc-arm-linux-gnueabihf
rustup target add armv7-unknown-linux-gnueabihf

# Build
cargo build --release --target armv7-unknown-linux-gnueabihf --bin r4w
```

### Static Linking (Portable Binary)

```bash
# For maximum portability
rustup target add x86_64-unknown-linux-musl
cargo build --release --target x86_64-unknown-linux-musl

# Result: static binary, no glibc dependency
ldd target/x86_64-unknown-linux-musl/release/r4w
# Output: not a dynamic executable
```

### Cross-Compilation Makefile

```makefile
# Makefile

TARGETS := x86_64-unknown-linux-gnu \
           aarch64-unknown-linux-gnu \
           armv7-unknown-linux-gnueabihf

.PHONY: all clean $(TARGETS)

all: $(TARGETS)

x86_64-unknown-linux-gnu:
	cargo build --release --target $@ --bin r4w

aarch64-unknown-linux-gnu:
	cargo build --release --target $@ --bin r4w

armv7-unknown-linux-gnueabihf:
	cargo build --release --target $@ --bin r4w

deploy-arm64: aarch64-unknown-linux-gnu
	scp target/aarch64-unknown-linux-gnu/release/r4w $(REMOTE_HOST):/opt/r4w/

clean:
	cargo clean
```

### Conditional Compilation

```rust
// Platform-specific code
#[cfg(target_arch = "aarch64")]
fn use_neon_simd(samples: &mut [IQSample]) {
    // ARM NEON optimizations
}

#[cfg(target_arch = "x86_64")]
fn use_avx_simd(samples: &mut [IQSample]) {
    // x86 AVX2 optimizations
}

#[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
fn use_scalar(samples: &mut [IQSample]) {
    // Fallback scalar implementation
}

// Feature-based compilation
#[cfg(feature = "fpga")]
mod fpga_accel;

#[cfg(not(feature = "fpga"))]
mod software_impl;
```

---

## Deployment

### Deployment Checklist

- [ ] Binary compiled for target architecture
- [ ] Dependencies met (glibc version, etc.)
- [ ] Configuration files in place
- [ ] Permissions set (GPIO, /dev/mem access)
- [ ] Systemd service created (if daemon)
- [ ] Logging configured
- [ ] Resource limits set (memory, CPU)

### Systemd Service

```ini
# /etc/systemd/system/r4w-agent.service
[Unit]
Description=R4W Waveform Agent
After=network.target

[Service]
Type=simple
User=r4w
Group=r4w
WorkingDirectory=/opt/r4w
ExecStart=/opt/r4w/r4w agent --port 6000
Restart=always
RestartSec=5

# Resource limits
MemoryMax=512M
CPUQuota=80%

# Security
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=/var/lib/r4w

# Real-time priority (if needed)
# CPUSchedulingPolicy=fifo
# CPUSchedulingPriority=50

[Install]
WantedBy=multi-user.target
```

### Docker Deployment

```dockerfile
# Dockerfile
FROM rust:1.75 as builder

WORKDIR /app
COPY . .
RUN cargo build --release --bin r4w

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y libssl3 && rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/target/release/r4w /usr/local/bin/

EXPOSE 5000/udp 6000/tcp
CMD ["r4w", "agent", "--port", "6000"]
```

```bash
# Build and run
docker build -t r4w:latest .
docker run -p 5000:5000/udp -p 6000:6000/tcp r4w:latest
```

### Configuration Management

```yaml
# /etc/r4w/config.yaml
server:
  host: "0.0.0.0"
  port: 6000

waveform:
  default: "LoRa"
  sample_rate: 500000

fpga:
  enabled: true
  platform: "zynq"
  bitstream: "/opt/r4w/bitstream/r4w_design.bit"

logging:
  level: "info"
  file: "/var/log/r4w/r4w.log"
  max_size_mb: 100
  max_files: 5

resources:
  max_memory_mb: 512
  max_dma_buffers: 8
  worker_threads: 4
```

---

## System Integration

### Hardware Interface Abstraction

```rust
/// Trait for SDR hardware backends
pub trait SdrDevice: Send + Sync {
    /// Get device info
    fn info(&self) -> DeviceInfo;

    /// Set center frequency
    fn set_frequency(&mut self, freq_hz: u64) -> Result<(), DeviceError>;

    /// Set sample rate
    fn set_sample_rate(&mut self, rate_hz: u64) -> Result<(), DeviceError>;

    /// Set gain
    fn set_gain(&mut self, gain_db: f32) -> Result<(), DeviceError>;

    /// Start streaming
    fn start_rx(&mut self) -> Result<(), DeviceError>;
    fn start_tx(&mut self) -> Result<(), DeviceError>;

    /// Stop streaming
    fn stop(&mut self) -> Result<(), DeviceError>;

    /// Read samples (blocking)
    fn read(&mut self, buffer: &mut [IQSample]) -> Result<usize, DeviceError>;

    /// Write samples (blocking)
    fn write(&mut self, samples: &[IQSample]) -> Result<usize, DeviceError>;
}
```

### UDP I/Q Transport

For integration with GNU Radio or other systems:

```rust
use std::net::UdpSocket;

pub struct UdpIqTransport {
    socket: UdpSocket,
    remote_addr: Option<std::net::SocketAddr>,
}

impl UdpIqTransport {
    pub fn bind(port: u16) -> std::io::Result<Self> {
        let socket = UdpSocket::bind(format!("0.0.0.0:{}", port))?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            socket,
            remote_addr: None,
        })
    }

    pub fn connect(&mut self, addr: &str) -> std::io::Result<()> {
        self.remote_addr = Some(addr.parse().map_err(|e|
            std::io::Error::new(std::io::ErrorKind::InvalidInput, e)
        )?);
        Ok(())
    }

    pub fn send_samples(&self, samples: &[IQSample]) -> std::io::Result<()> {
        let addr = self.remote_addr.ok_or_else(||
            std::io::Error::new(std::io::ErrorKind::NotConnected, "No remote address")
        )?;

        // Convert to bytes (I16 interleaved format, GNU Radio compatible)
        let mut buffer = Vec::with_capacity(samples.len() * 4);
        for sample in samples {
            let i = (sample.i * 32767.0) as i16;
            let q = (sample.q * 32767.0) as i16;
            buffer.extend_from_slice(&i.to_le_bytes());
            buffer.extend_from_slice(&q.to_le_bytes());
        }

        self.socket.send_to(&buffer, addr)?;
        Ok(())
    }

    pub fn recv_samples(&self, buffer: &mut [IQSample]) -> std::io::Result<usize> {
        let mut raw_buffer = vec![0u8; buffer.len() * 4];

        match self.socket.recv_from(&mut raw_buffer) {
            Ok((n_bytes, _addr)) => {
                let n_samples = n_bytes / 4;
                for i in 0..n_samples.min(buffer.len()) {
                    let i_bytes = [raw_buffer[i * 4], raw_buffer[i * 4 + 1]];
                    let q_bytes = [raw_buffer[i * 4 + 2], raw_buffer[i * 4 + 3]];
                    let i_val = i16::from_le_bytes(i_bytes);
                    let q_val = i16::from_le_bytes(q_bytes);
                    buffer[i] = IQSample::new(
                        i_val as f32 / 32767.0,
                        q_val as f32 / 32767.0,
                    );
                }
                Ok(n_samples)
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }
}
```

### GNU Radio Integration

```python
#!/usr/bin/env python3
# GNU Radio flowgraph to interface with R4W

from gnuradio import gr, blocks, uhd
import socket

class R4WSource(gr.sync_block):
    """GNU Radio source that receives I/Q from R4W"""

    def __init__(self, port=5000):
        gr.sync_block.__init__(
            self,
            name='R4W Source',
            in_sig=None,
            out_sig=[numpy.complex64]
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', port))
        self.sock.setblocking(False)

    def work(self, input_items, output_items):
        out = output_items[0]
        try:
            data, _ = self.sock.recvfrom(len(out) * 4)
            samples = numpy.frombuffer(data, dtype=numpy.int16)
            samples = samples.astype(numpy.float32) / 32767.0
            iq = samples[0::2] + 1j * samples[1::2]
            n = min(len(iq), len(out))
            out[:n] = iq[:n]
            return n
        except BlockingIOError:
            return 0
```

---

## Memory and CPU Constraints

### Memory Management Strategies

```rust
/// Pre-allocated sample buffer pool
pub struct SampleBufferPool {
    buffers: Vec<Vec<IQSample>>,
    buffer_size: usize,
}

impl SampleBufferPool {
    pub fn new(num_buffers: usize, buffer_size: usize) -> Self {
        let buffers = (0..num_buffers)
            .map(|_| vec![IQSample::new(0.0, 0.0); buffer_size])
            .collect();
        Self { buffers, buffer_size }
    }

    pub fn acquire(&mut self) -> Option<Vec<IQSample>> {
        self.buffers.pop()
    }

    pub fn release(&mut self, mut buffer: Vec<IQSample>) {
        buffer.clear();
        buffer.resize(self.buffer_size, IQSample::new(0.0, 0.0));
        self.buffers.push(buffer);
    }
}

/// Streaming processor that avoids allocations
pub struct StreamingProcessor {
    waveform: Box<dyn Waveform>,
    input_buffer: Vec<IQSample>,
    output_buffer: Vec<bool>,
    scratch: Vec<f32>,
}

impl StreamingProcessor {
    pub fn new(waveform: Box<dyn Waveform>, max_samples: usize) -> Self {
        let max_bits = max_samples / 10;  // Conservative estimate
        Self {
            waveform,
            input_buffer: Vec::with_capacity(max_samples),
            output_buffer: Vec::with_capacity(max_bits),
            scratch: Vec::with_capacity(max_samples),
        }
    }

    pub fn process(&mut self, samples: &[IQSample]) -> &[bool] {
        // Clear but don't deallocate
        self.output_buffer.clear();

        // Process using pre-allocated scratch space
        let bits = self.waveform.demodulate(samples);
        self.output_buffer.extend(bits);

        &self.output_buffer
    }
}
```

### CPU Affinity and Priority

```rust
use nix::sched::{sched_setaffinity, CpuSet};
use nix::unistd::Pid;

/// Pin current thread to specific CPU core
pub fn pin_to_cpu(cpu: usize) -> Result<(), nix::Error> {
    let mut cpuset = CpuSet::new();
    cpuset.set(cpu)?;
    sched_setaffinity(Pid::from_raw(0), &cpuset)
}

/// Set real-time scheduling priority
pub fn set_realtime_priority(priority: i32) -> Result<(), nix::Error> {
    use nix::sched::{sched_setscheduler, Scheduler};

    let param = nix::sched::sched_param { sched_priority: priority };
    sched_setscheduler(Pid::from_raw(0), Scheduler::FIFO, &param)
}

/// Example: Configure worker thread
fn setup_worker_thread() {
    // Pin to CPU 2 (avoid CPU 0 which handles interrupts)
    if let Err(e) = pin_to_cpu(2) {
        eprintln!("Warning: Could not pin to CPU: {}", e);
    }

    // Set real-time priority (requires CAP_SYS_NICE)
    if let Err(e) = set_realtime_priority(50) {
        eprintln!("Warning: Could not set RT priority: {}", e);
    }
}
```

### Resource Monitoring

```rust
use std::time::{Duration, Instant};

pub struct ResourceMonitor {
    start_time: Instant,
    samples_processed: u64,
    peak_latency_us: u64,
    total_latency_us: u64,
    measurement_count: u64,
}

impl ResourceMonitor {
    pub fn new() -> Self {
        Self {
            start_time: Instant::now(),
            samples_processed: 0,
            peak_latency_us: 0,
            total_latency_us: 0,
            measurement_count: 0,
        }
    }

    pub fn record_processing(&mut self, samples: usize, latency: Duration) {
        self.samples_processed += samples as u64;
        let latency_us = latency.as_micros() as u64;
        self.peak_latency_us = self.peak_latency_us.max(latency_us);
        self.total_latency_us += latency_us;
        self.measurement_count += 1;
    }

    pub fn report(&self) -> ResourceReport {
        let elapsed = self.start_time.elapsed();
        ResourceReport {
            throughput_sps: self.samples_processed as f64 / elapsed.as_secs_f64(),
            avg_latency_us: self.total_latency_us / self.measurement_count.max(1),
            peak_latency_us: self.peak_latency_us,
            uptime_secs: elapsed.as_secs(),
        }
    }
}

#[derive(Debug)]
pub struct ResourceReport {
    pub throughput_sps: f64,
    pub avg_latency_us: u64,
    pub peak_latency_us: u64,
    pub uptime_secs: u64,
}
```

---

## Shared Memory IPC

### POSIX Shared Memory

For high-performance inter-process communication:

```rust
use std::ffi::CString;
use std::ptr;

/// Shared memory region for I/Q samples
pub struct SharedMemoryRegion {
    name: CString,
    ptr: *mut u8,
    size: usize,
    fd: i32,
}

impl SharedMemoryRegion {
    pub fn create(name: &str, size: usize) -> Result<Self, std::io::Error> {
        let name = CString::new(name)?;

        unsafe {
            // Create shared memory object
            let fd = libc::shm_open(
                name.as_ptr(),
                libc::O_CREAT | libc::O_RDWR,
                0o666,
            );
            if fd < 0 {
                return Err(std::io::Error::last_os_error());
            }

            // Set size
            if libc::ftruncate(fd, size as libc::off_t) < 0 {
                libc::close(fd);
                return Err(std::io::Error::last_os_error());
            }

            // Map to memory
            let ptr = libc::mmap(
                ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );

            if ptr == libc::MAP_FAILED {
                libc::close(fd);
                return Err(std::io::Error::last_os_error());
            }

            Ok(Self {
                name,
                ptr: ptr as *mut u8,
                size,
                fd,
            })
        }
    }

    pub fn open(name: &str, size: usize) -> Result<Self, std::io::Error> {
        let name = CString::new(name)?;

        unsafe {
            let fd = libc::shm_open(name.as_ptr(), libc::O_RDWR, 0o666);
            if fd < 0 {
                return Err(std::io::Error::last_os_error());
            }

            let ptr = libc::mmap(
                ptr::null_mut(),
                size,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );

            if ptr == libc::MAP_FAILED {
                libc::close(fd);
                return Err(std::io::Error::last_os_error());
            }

            Ok(Self {
                name,
                ptr: ptr as *mut u8,
                size,
                fd,
            })
        }
    }

    pub fn write_samples(&self, samples: &[IQSample]) -> Result<(), std::io::Error> {
        let byte_len = samples.len() * std::mem::size_of::<IQSample>();
        if byte_len > self.size {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "Buffer too large",
            ));
        }

        unsafe {
            ptr::copy_nonoverlapping(
                samples.as_ptr() as *const u8,
                self.ptr,
                byte_len,
            );
        }

        Ok(())
    }

    pub fn read_samples(&self, count: usize) -> Vec<IQSample> {
        let mut samples = vec![IQSample::new(0.0, 0.0); count];
        let byte_len = count * std::mem::size_of::<IQSample>();

        unsafe {
            ptr::copy_nonoverlapping(
                self.ptr,
                samples.as_mut_ptr() as *mut u8,
                byte_len.min(self.size),
            );
        }

        samples
    }
}

impl Drop for SharedMemoryRegion {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.ptr as *mut libc::c_void, self.size);
            libc::close(self.fd);
            // Optionally unlink:
            // libc::shm_unlink(self.name.as_ptr());
        }
    }
}
```

### Ring Buffer for Streaming

```rust
use std::sync::atomic::{AtomicUsize, Ordering};

/// Lock-free SPSC ring buffer for real-time streaming
pub struct RingBuffer {
    buffer: Vec<IQSample>,
    capacity: usize,
    write_idx: AtomicUsize,
    read_idx: AtomicUsize,
}

impl RingBuffer {
    pub fn new(capacity: usize) -> Self {
        // Power of 2 for efficient modulo
        let capacity = capacity.next_power_of_two();
        Self {
            buffer: vec![IQSample::new(0.0, 0.0); capacity],
            capacity,
            write_idx: AtomicUsize::new(0),
            read_idx: AtomicUsize::new(0),
        }
    }

    pub fn push(&mut self, sample: IQSample) -> bool {
        let write = self.write_idx.load(Ordering::Relaxed);
        let read = self.read_idx.load(Ordering::Acquire);

        let next_write = (write + 1) & (self.capacity - 1);
        if next_write == read {
            return false;  // Full
        }

        self.buffer[write] = sample;
        self.write_idx.store(next_write, Ordering::Release);
        true
    }

    pub fn pop(&mut self) -> Option<IQSample> {
        let read = self.read_idx.load(Ordering::Relaxed);
        let write = self.write_idx.load(Ordering::Acquire);

        if read == write {
            return None;  // Empty
        }

        let sample = self.buffer[read];
        self.read_idx.store((read + 1) & (self.capacity - 1), Ordering::Release);
        Some(sample)
    }

    pub fn available(&self) -> usize {
        let write = self.write_idx.load(Ordering::Acquire);
        let read = self.read_idx.load(Ordering::Acquire);

        if write >= read {
            write - read
        } else {
            self.capacity - read + write
        }
    }
}
```

---

## Real-Time Processing

### Real-Time Design Principles

1. **Avoid Allocations**: Pre-allocate all buffers
2. **No Locks in Hot Path**: Use lock-free data structures
3. **Bounded Latency**: Know worst-case execution time
4. **Priority Inheritance**: Use appropriate mutex types

```rust
use std::sync::Arc;
use parking_lot::Mutex;  // Better for real-time than std::sync::Mutex

pub struct RealtimeProcessor {
    waveform: Arc<dyn Waveform>,
    input_pool: Mutex<SampleBufferPool>,
    output_pool: Mutex<Vec<Vec<bool>>>,

    // Pre-allocated scratch buffers
    scratch_fft: Vec<IQSample>,
    scratch_filter: Vec<IQSample>,
}

impl RealtimeProcessor {
    pub fn process_realtime(&mut self, samples: &[IQSample]) -> &[bool] {
        // Reuse pre-allocated buffers
        self.scratch_fft.clear();
        self.scratch_fft.extend_from_slice(samples);

        // Process without allocation
        let bits = self.waveform.demodulate(&self.scratch_fft);

        // Store result in pre-allocated output
        // ...

        todo!()
    }
}
```

### Latency Measurement

```rust
use std::time::Instant;

pub struct LatencyTracker {
    samples: Vec<u64>,
    position: usize,
    capacity: usize,
}

impl LatencyTracker {
    pub fn new(capacity: usize) -> Self {
        Self {
            samples: vec![0; capacity],
            position: 0,
            capacity,
        }
    }

    pub fn record(&mut self, latency_ns: u64) {
        self.samples[self.position] = latency_ns;
        self.position = (self.position + 1) % self.capacity;
    }

    pub fn percentile(&self, p: f64) -> u64 {
        let mut sorted: Vec<_> = self.samples.iter().copied().filter(|&x| x > 0).collect();
        sorted.sort_unstable();

        if sorted.is_empty() {
            return 0;
        }

        let idx = ((sorted.len() as f64 * p / 100.0) as usize).min(sorted.len() - 1);
        sorted[idx]
    }

    pub fn stats(&self) -> LatencyStats {
        LatencyStats {
            p50_ns: self.percentile(50.0),
            p95_ns: self.percentile(95.0),
            p99_ns: self.percentile(99.0),
            max_ns: self.samples.iter().copied().max().unwrap_or(0),
        }
    }
}

#[derive(Debug)]
pub struct LatencyStats {
    pub p50_ns: u64,
    pub p95_ns: u64,
    pub p99_ns: u64,
    pub max_ns: u64,
}
```

---

## Testing Strategies

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_modulation_basic() {
        let waveform = MyWaveform::default_config(48000.0);
        let bits = vec![true, false];
        let samples = waveform.modulate(&bits);

        assert!(!samples.is_empty());
        assert_eq!(samples.len() % waveform.samples_per_symbol, 0);
    }

    #[test]
    fn test_roundtrip_all_patterns() {
        let waveform = MyWaveform::default_config(48000.0);

        // Test all 2-bit patterns
        for pattern in 0..4u8 {
            let bits = vec![pattern & 1 != 0, pattern & 2 != 0];
            let samples = waveform.modulate(&bits);
            let recovered = waveform.demodulate(&samples);

            assert_eq!(bits, recovered, "Pattern {:02b} failed", pattern);
        }
    }
}
```

### Integration Tests

```rust
// tests/integration_test.rs

use r4w_core::waveform::WaveformFactory;
use r4w_sim::channel::AwgnChannel;

#[test]
fn test_waveform_through_channel() {
    let waveform = WaveformFactory::create("LoRa", 500000.0).unwrap();

    let message = "Hello, R4W!";
    let bits: Vec<bool> = message.bytes()
        .flat_map(|b| (0..8).map(move |i| (b >> i) & 1 != 0))
        .collect();

    let samples = waveform.modulate(&bits);
    let noisy = AwgnChannel::new(10.0).apply(&samples);
    let recovered_bits = waveform.demodulate(&noisy);

    let recovered_bytes: Vec<u8> = recovered_bits.chunks(8)
        .map(|chunk| {
            chunk.iter().enumerate()
                .fold(0u8, |acc, (i, &b)| acc | ((b as u8) << i))
        })
        .collect();

    let recovered_message: String = recovered_bytes.iter()
        .take_while(|&&b| b != 0)
        .map(|&b| b as char)
        .collect();

    assert_eq!(message, recovered_message);
}
```

### Property-Based Testing

```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn test_roundtrip_any_bits(bits in prop::collection::vec(any::<bool>(), 2..100)) {
        let waveform = MyWaveform::default_config(48000.0);

        // Pad to even length
        let mut bits = bits;
        if bits.len() % 2 != 0 {
            bits.push(false);
        }

        let samples = waveform.modulate(&bits);
        let recovered = waveform.demodulate(&samples);

        prop_assert_eq!(bits, recovered);
    }

    #[test]
    fn test_constellation_unit_magnitude(
        waveform_name in prop::sample::select(vec!["BPSK", "QPSK", "MyWave"])
    ) {
        let waveform = WaveformFactory::create(&waveform_name, 48000.0).unwrap();
        let points = waveform.constellation_points();

        for point in points {
            let mag = point.magnitude();
            prop_assert!((mag - 1.0).abs() < 0.1, "Magnitude {} not near 1.0", mag);
        }
    }
}
```

### Fuzz Testing

```rust
// fuzz/fuzz_targets/demodulate.rs

#![no_main]
use libfuzzer_sys::fuzz_target;
use r4w_core::types::IQSample;
use r4w_core::waveform::WaveformFactory;

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    // Convert bytes to samples
    let samples: Vec<IQSample> = data.chunks(4)
        .filter_map(|chunk| {
            if chunk.len() < 4 {
                return None;
            }
            let i = i16::from_le_bytes([chunk[0], chunk[1]]) as f32 / 32767.0;
            let q = i16::from_le_bytes([chunk[2], chunk[3]]) as f32 / 32767.0;
            Some(IQSample::new(i, q))
        })
        .collect();

    if let Some(waveform) = WaveformFactory::create("QPSK", 48000.0) {
        // Should not panic or crash
        let _ = waveform.demodulate(&samples);
    }
});
```

---

## FPGA Acceleration

### Using the FpgaAccelerator Trait

```rust
use r4w_fpga::{FpgaAccelerator, ZynqFpga, SimulatedFpga};

fn create_accelerator() -> Box<dyn FpgaAccelerator> {
    // Try real hardware first, fall back to simulation
    match ZynqFpga::auto_detect() {
        Ok(fpga) => {
            println!("Using Zynq FPGA: {:?}", fpga.info());
            Box::new(fpga)
        }
        Err(e) => {
            println!("FPGA not available ({}), using simulation", e);
            Box::new(SimulatedFpga::new())
        }
    }
}

fn process_with_acceleration(
    fpga: &dyn FpgaAccelerator,
    samples: &[IQSample],
) -> Vec<IQSample> {
    let caps = fpga.capabilities();

    // Use hardware FFT if available and large enough
    if samples.len() <= caps.max_fft_size {
        match fpga.fft(samples, false) {
            Ok(result) => return result,
            Err(e) => {
                println!("FPGA FFT failed: {}, using software", e);
            }
        }
    }

    // Fall back to software
    software_fft(samples)
}
```

### FPGA-Accelerated Waveform

```rust
/// Waveform that automatically uses FPGA when available
pub struct AcceleratedLoRa {
    software: LoRaWaveform,
    fpga: Option<Box<dyn FpgaAccelerator>>,
}

impl AcceleratedLoRa {
    pub fn new(sample_rate: f64) -> Self {
        let fpga = ZynqFpga::auto_detect().ok().map(|f| Box::new(f) as _);
        Self {
            software: LoRaWaveform::new(sample_rate),
            fpga,
        }
    }
}

impl Waveform for AcceleratedLoRa {
    fn modulate(&self, bits: &[bool]) -> Vec<IQSample> {
        if let Some(ref fpga) = self.fpga {
            if let Some(waveform_id) = fpga.waveform_id("LoRa") {
                if let Ok(samples) = fpga.modulate(waveform_id, bits) {
                    return samples;
                }
            }
        }
        self.software.modulate(bits)
    }

    fn demodulate(&self, samples: &[IQSample]) -> Vec<bool> {
        if let Some(ref fpga) = self.fpga {
            if let Some(waveform_id) = fpga.waveform_id("LoRa") {
                if let Ok(bits) = fpga.demodulate(waveform_id, samples) {
                    return bits;
                }
            }
        }
        self.software.demodulate(samples)
    }

    // Delegate other methods to software implementation
    fn info(&self) -> WaveformInfo { self.software.info() }
    fn constellation_points(&self) -> Vec<IQSample> { self.software.constellation_points() }
    fn get_modulation_stages(&self, bits: &[bool]) -> Vec<ModulationStage> {
        self.software.get_modulation_stages(bits)
    }
    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        self.software.get_demodulation_steps(samples)
    }
}
```

---

## Security Considerations

For detailed security guidance, see the [Security Guide](./SECURITY_GUIDE.md).

### Key Security Principles

1. **Memory Safety**: Rust's ownership system prevents buffer overflows
2. **Input Validation**: Always validate sample counts and parameters
3. **Zeroization**: Clear sensitive data (keys, TRANSEC) from memory
4. **Isolation**: Use process isolation for classified components
5. **Audit Logging**: Log security-relevant events

### Secure Memory Handling

```rust
use zeroize::Zeroize;

/// Key material that is automatically zeroized on drop
#[derive(Zeroize)]
#[zeroize(drop)]
pub struct TransecKey {
    key_data: [u8; 32],
}

impl TransecKey {
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() != 32 {
            return None;
        }
        let mut key_data = [0u8; 32];
        key_data.copy_from_slice(bytes);
        Some(Self { key_data })
    }
}
```

### Input Validation

```rust
pub fn validate_samples(samples: &[IQSample]) -> Result<(), ValidationError> {
    // Check for NaN or Inf
    for (i, sample) in samples.iter().enumerate() {
        if !sample.i.is_finite() || !sample.q.is_finite() {
            return Err(ValidationError::InvalidSample {
                index: i,
                i: sample.i,
                q: sample.q,
            });
        }

        // Check for unreasonable magnitudes
        let mag = sample.magnitude();
        if mag > 100.0 {
            return Err(ValidationError::MagnitudeTooLarge {
                index: i,
                magnitude: mag,
            });
        }
    }

    Ok(())
}
```

---

## Appendix A: Quick Reference

### Common Commands

```bash
# Build
cargo build --release

# Test
cargo test
cargo test --release  # Test optimized build

# Benchmark
cargo bench

# Run CLI
cargo run --bin r4w -- --help
cargo run --bin r4w -- waveform --list
cargo run --bin r4w -- simulate --waveform QPSK --snr 10

# Run GUI
cargo run --bin r4w-explorer

# Cross-compile
cargo build --release --target aarch64-unknown-linux-gnu

# Profile
cargo flamegraph --bin r4w -- simulate --duration 10
```

### Performance Targets

| Operation | Target | Platform |
|-----------|--------|----------|
| PSK modulation | >10 Msps | x86_64 |
| PSK demodulation | >5 Msps | x86_64 |
| LoRa modulation | >1 Msps | x86_64 |
| LoRa demodulation | >500 ksps | x86_64 |
| FFT (1024 pt) | <50 μs | x86_64 |
| Single symbol latency | <100 μs | ARM64 |

### Memory Estimates

| Component | Memory |
|-----------|--------|
| Base runtime | ~10 MB |
| Per waveform instance | ~1 MB |
| FFT (1024 pt) buffers | ~64 KB |
| Sample buffer (1 sec @ 1 Msps) | ~8 MB |
| GUI application | ~50 MB |

---

## Appendix B: Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Poor BER | Low SNR or sync issues | Check channel, add preamble |
| High latency | Allocations in hot path | Pre-allocate buffers |
| Memory growth | Buffer leaks | Use buffer pools |
| Cross-compile fails | Missing linker | Install cross toolchain |
| FPGA not detected | Permission denied | Add user to `uio` group |
| Timing jitter | System interrupts | Use RT priority, CPU pinning |

---

*For questions or contributions, please open an issue on the R4W GitHub repository.*
