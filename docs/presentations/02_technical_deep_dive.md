---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>R4W Technical Deep Dive'
subtitle: "Architecture, Implementation, and Demos"
author: "R4W Development Team<br>(Aida, Joe Mooney, Claude Code)"
date: "December 2025"
header-includes: |
  <style>
    .reveal pre { margin: 0 auto; width: fit-content; }
    .reveal pre code { text-align: left; }
    .reveal section img { max-height: 60vh; }
  </style>
---

# R4W Technical Deep Dive

## Agenda

1. Architecture Overview (10 min)
2. Core DSP Implementation (15 min)
3. Real-Time Primitives (10 min)
4. Hardware Abstraction (10 min)
5. FPGA Acceleration (10 min)
6. Waveform Plugins & Isolation (5 min)
7. Live Demos (varies)

---

# Part 1: Architecture

## Crate Structure

```
r4w/
├── crates/
│   ├── r4w-core/        # DSP algorithms, waveforms, plugins
│   ├── r4w-sim/         # Hardware abstraction, simulation
│   ├── r4w-fpga/        # FPGA acceleration (Zynq, Lattice)
│   ├── r4w-sandbox/     # Waveform isolation (8 levels)
│   ├── r4w-gui/         # Educational egui application
│   ├── r4w-cli/         # Command-line interface
│   ├── r4w-web/         # WebAssembly target
│   └── r4w-ffi/         # C/C++ bindings
├── vivado/              # Xilinx Zynq IP cores
├── lattice/             # Lattice iCE40/ECP5 designs
├── coq/                 # Formal verification proofs
├── workshop/            # Lab exercises
└── plugins/             # Example waveform plugins
```

---

## Data Flow

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  Bits    │───▶│ Waveform │───▶│  HAL     │───▶│ Hardware │
│          │    │ Modulate │    │ Stream   │    │ TX       │
└──────────┘    └──────────┘    └──────────┘    └──────────┘

┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│  Bits    │◀───│ Waveform │◀───│  HAL     │◀───│ Hardware │
│          │    │ Demod    │    │ Stream   │    │ RX       │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
```

---

## IQ Sample Representation

```rust
// Core data type
#[derive(Copy, Clone)]
pub struct IQSample {
    pub re: f32,  // In-phase (real)
    pub im: f32,  // Quadrature (imaginary)
}

// Operations
impl IQSample {
    pub fn magnitude(&self) -> f32 {
        (self.re * self.re + self.im * self.im).sqrt()
    }

    pub fn phase(&self) -> f32 {
        self.im.atan2(self.re)
    }
}
```

---

# Part 2: Core DSP

## Chirp Generation (LoRa)

```rust
pub fn generate_upchirp(&self, start_freq: f64) -> Vec<IQSample> {
    let samples_per_symbol = 2_usize.pow(self.spreading_factor as u32);

    (0..samples_per_symbol)
        .map(|i| {
            let t = i as f64 / self.sample_rate;

            // Instantaneous frequency: f(t) = f0 + chirp_rate * t
            // Phase: φ(t) = 2π * ∫f(t)dt = 2π * (f0*t + 0.5*k*t²)
            let phase = 2.0 * PI * (start_freq * t + 0.5 * self.chirp_rate * t * t);

            IQSample::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect()
}
```

**Key insight**: Chirp phase is quadratic in time.

---

## FFT-Based Demodulation

```rust
pub fn demodulate(&self, samples: &[IQSample]) -> Vec<bool> {
    // 1. Multiply by downchirp (dechirp)
    let dechirped: Vec<_> = samples.iter()
        .zip(self.downchirp.iter())
        .map(|(s, d)| complex_multiply(s, &d.conj()))
        .collect();

    // 2. FFT to find peak frequency
    let spectrum = fft(&dechirped);

    // 3. Peak bin = symbol value
    let peak_bin = spectrum.iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.magnitude().partial_cmp(&b.magnitude()).unwrap())
        .map(|(i, _)| i)
        .unwrap();

    // 4. Convert bin index to bits
    bin_to_bits(peak_bin, self.spreading_factor)
}
```

---

## FEC Pipeline

```
TX: Data → Whitening → Hamming FEC → Interleaving → Gray Code → CSS
RX: CSS → Gray Decode → De-interleave → Hamming Decode → De-whiten → Data
```

Each stage is a pure function:

```rust
pub fn encode_hamming_7_4(data: &[bool]) -> Vec<bool>;
pub fn decode_hamming_7_4(encoded: &[bool]) -> Vec<bool>;
pub fn interleave(bits: &[bool], sf: u8) -> Vec<bool>;
pub fn gray_encode(symbol: usize) -> usize;
```

---

## Filter Design

```rust
// FIR low-pass filter design
pub fn design_lowpass_fir(
    num_taps: usize,
    cutoff: f64,        // Normalized cutoff (0-1)
    sample_rate: f64,
) -> Vec<f64> {
    let center = num_taps / 2;
    (0..num_taps)
        .map(|n| {
            let i = n as i32 - center as i32;
            if i == 0 {
                cutoff
            } else {
                let x = PI * i as f64 * cutoff;
                x.sin() / (PI * i as f64) * hamming_window(n, num_taps)
            }
        })
        .collect()
}
```

---

# Part 3: Real-Time Primitives

## Lock-Free Ring Buffer

```rust
pub struct RingBuffer<T> {
    buffer: Box<[UnsafeCell<MaybeUninit<T>>]>,
    head: CachePadded<AtomicUsize>,  // Producer writes here
    tail: CachePadded<AtomicUsize>,  // Consumer reads here
    capacity: usize,
    mask: usize,  // Power-of-2 for branchless modulo
}
```

**Key properties**:
- Single Producer, Single Consumer (SPSC)
- No locks, no allocations
- Cache-line padding prevents false sharing
- Release-Acquire ordering for visibility

---

## Ring Buffer Performance

| Operation | Time | Notes |
|-----------|------|-------|
| Single push | ~5 ns | 1 atomic store |
| Single pop | ~5 ns | 1 atomic load |
| Batch push (1024) | ~200 ns | memcpy + 1 atomic |
| Batch pop (1024) | ~200 ns | memcpy + 1 atomic |

**Throughput**: 200+ million samples/second

---

## Buffer Pool (Zero Allocation)

```rust
pub struct BufferPool<T> {
    buffers: Vec<T>,
    free_bitmap: AtomicU64,
}

// Acquire: single CAS, O(1)
pub fn acquire(&self) -> Option<PooledBuffer<T>> {
    loop {
        let bitmap = self.free_bitmap.load(Ordering::Acquire);
        let slot = bitmap.trailing_zeros() as usize;
        if slot >= 64 { return None; }

        let new_bitmap = bitmap & !(1 << slot);
        if self.free_bitmap.compare_exchange(
            bitmap, new_bitmap, Ordering::AcqRel, Ordering::Relaxed
        ).is_ok() {
            return Some(PooledBuffer { index: slot, .. });
        }
    }
}
```

---

## RT Thread Support

```rust
pub fn spawn_rt<F>(config: RtConfig, f: F) -> JoinHandle<()> {
    std::thread::spawn(move || {
        // 1. Lock memory (no page faults)
        unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE); }

        // 2. Set SCHED_FIFO (deterministic scheduling)
        let param = libc::sched_param { sched_priority: config.priority };
        unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param); }

        // 3. Pin to CPU (no migration)
        if let Some(cpus) = config.affinity {
            set_cpu_affinity(&cpus);
        }

        f();
    })
}
```

---

# Part 4: Hardware Abstraction

## HAL Traits

```rust
pub trait SdrDeviceExt: Send + Sync {
    fn capabilities(&self) -> DeviceCapabilities;
    fn tuner(&self) -> &dyn TunerControl;
    fn clock(&self) -> &dyn ClockControl;
    fn create_rx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>>;
    fn create_tx_stream(&mut self, config: StreamConfig) -> SdrResult<Box<dyn StreamHandle>>;
}

pub trait StreamHandle: Send {
    fn start(&mut self) -> SdrResult<()>;
    fn stop(&mut self) -> SdrResult<()>;
    fn read(&mut self, buffer: &mut [IQSample]) -> SdrResult<(usize, StreamMetadata)>;
    fn write(&mut self, buffer: &[IQSample]) -> SdrResult<(usize, StreamMetadata)>;
}
```

---

## UHD Driver Implementation

```rust
pub struct UhdDevice {
    device_type: UhdDeviceType,
    serial: String,
    address: Option<String>,
    state: UhdDeviceState,
    tuner: UhdTuner,
    clock: UhdClock,
}

impl SdrDeviceExt for UhdDevice {
    fn capabilities(&self) -> DeviceCapabilities {
        match self.device_type {
            UhdDeviceType::N210 => DeviceCapabilities {
                min_frequency: 50e6,
                max_frequency: 2200e6,
                max_sample_rate: 50e6,
                can_tx: true, can_rx: true,
                full_duplex: true,
                ..
            },
            UhdDeviceType::B200 => DeviceCapabilities {
                max_frequency: 6000e6,
                ..
            },
        }
    }
}
```

---

## Attenuator Control

```rust
pub trait Attenuator: Send {
    fn name(&self) -> &str;
    fn capabilities(&self) -> AttenuatorCapabilities;
    fn set_attenuation(&mut self, db: f64) -> SdrResult<f64>;
    fn attenuation(&self) -> f64;
}

// Test harness for automated sensitivity measurement
pub struct AttenuatorTestHarness {
    attenuator: Box<dyn Attenuator>,
    tx_power_dbm: f64,
    noise_floor_dbm: f64,
    settle_time: Duration,
}
```

---

## Clock Synchronization

```rust
pub trait ClockControl: Send + Sync {
    fn set_clock_source(&self, source: ClockSource) -> SdrResult<()>;
    fn set_time_source(&self, source: TimeSource) -> SdrResult<()>;
    fn set_time(&self, time: f64) -> SdrResult<()>;
    fn hardware_time(&self) -> Option<Timestamp>;
    fn ref_locked(&self) -> bool;
    fn pps_locked(&self) -> bool;
}

pub enum ClockSource {
    Internal,
    External,  // 10 MHz reference
    Gpsdo,     // GPS-disciplined oscillator
    Mimo,      // MIMO cable
}
```

---

# Part 5: FPGA Acceleration

## FPGA Platform Support

| Platform | Toolchain | Use Case | Status |
|----------|-----------|----------|--------|
| **Xilinx Zynq** | Vivado | Production SDR | IP cores ready |
| **Lattice iCE40** | Yosys/nextpnr | Low-cost education | Implemented |
| **Lattice ECP5** | Yosys/nextpnr | Mid-range SDR | Implemented |

---

## Zynq IP Cores

```
vivado/ip/
├── r4w_fft/         # 1024-pt Radix-4 FFT (~15k LUTs)
├── r4w_fir/         # 256-tap FIR filter (~8k LUTs)
├── r4w_chirp_gen/   # LoRa chirp generator (~2k LUTs)
├── r4w_chirp_corr/  # Chirp correlator (~12k LUTs)
├── r4w_nco/         # Numerically Controlled Oscillator (~1.5k LUTs)
└── r4w_dma/         # DMA controller for PS↔PL (~3k LUTs)
```

All IP cores use AXI-Stream + AXI-Lite interfaces.

---

## FpgaAccelerator Trait

```rust
pub trait FpgaAccelerator: Send + Sync {
    fn fft(&self, samples: &[IQSample], inverse: bool)
        -> Result<Vec<IQSample>, FpgaError>;

    fn fir_filter(&self, samples: &[IQSample], taps: &[f32])
        -> Result<Vec<IQSample>, FpgaError>;

    fn modulate(&self, waveform_id: u32, bits: &[bool])
        -> Result<Vec<IQSample>, FpgaError>;
}
```

Same API for Zynq, Lattice, or software fallback.

---

# Part 6: Waveform Plugins & Isolation

## Dynamic Plugin Loading

```rust
// Plugin entry point (C ABI)
#[no_mangle]
pub extern "C" fn r4w_plugin_api_version() -> u32 {
    PLUGIN_API_VERSION
}

#[no_mangle]
pub extern "C" fn r4w_plugin_info() -> *const PluginInfo {
    static INFO: PluginInfo = PluginInfo {
        name: b"my_waveform\0".as_ptr() as *const c_char,
        waveform_count: 1,
        ..
    };
    &INFO
}
```

---

## Waveform Isolation (8 Levels)

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

# Part 7: Live Demos

## Demo 1: Device Discovery

```bash
cargo run --example 01_device_discovery
```

Shows:
- Driver registry
- Device enumeration
- Capability querying

---

## Demo 2: Loopback Test

```bash
cargo run --example 04_loopback -- --simulator
```

Shows:
- TX/RX stream creation
- Attenuator control
- Signal correlation

---

## Demo 3: BER vs SNR

```bash
cargo run --example 70_ber_testing
```

Shows:
- BPSK modulation/demodulation
- AWGN channel
- Theoretical vs measured BER

---

## Demo 4: Benchmarks

```bash
cargo bench -p r4w-core --bench dsp_bench
```

Shows:
- FFT performance
- Modulation throughput
- Real-time latency histograms

---

## Questions?

### Explore Further

```bash
# All tests
cargo test

# Documentation
cargo doc --open

# Workshop exercises
ls workshop/usrp/exercises/
ls workshop/advanced/exercises/
```
