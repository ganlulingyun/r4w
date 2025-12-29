# Workshop 10: Real-Time and FPGA Concepts

**Duration:** 75 minutes
**Difficulty:** Advanced
**Prerequisites:** All previous workshops recommended

## Objectives

By the end of this workshop, you will:
- Understand real-time constraints in SDR
- Know R4W's RT primitives (ring buffers, memory pools)
- Understand when and why to use FPGA acceleration
- See the R4W FPGA integration architecture

---

## 1. Why Real-Time Matters

### 1.1 The Problem

SDR involves **continuous** data streams:

```
ADC → [Samples] → DSP → [Samples] → DAC
       ↑                    ↑
    1 MS/s              1 MS/s
    = 1 μs/sample       = 1 μs deadline!
```

If processing takes longer than the sample period:
- **RX**: Buffer overflow, lost samples
- **TX**: Buffer underflow, glitches

### 1.2 Latency Budget

For 1 MS/s operation:
```
Available time per sample: 1 μs

Breakdown:
  DMA transfer:     ~0.1 μs
  DSP processing:   ~0.5 μs
  Buffer management: ~0.1 μs
  Margin:           ~0.3 μs
                    ─────────
  Total:             1.0 μs
```

No room for:
- Garbage collection pauses
- Page faults
- Priority inversion
- Blocking allocations

---

## 2. R4W Real-Time Primitives

### 2.1 Lock-Free Ring Buffer

The heart of RT streaming:

```rust
use r4w_core::rt::RingBuffer;

// Create buffer for 64K samples
let (tx, rx) = RingBuffer::<IQSample>::new(65536);

// Producer thread (ADC)
std::thread::spawn(move || {
    loop {
        let samples = receive_from_hardware();
        tx.push_slice(&samples);  // Never blocks!
    }
});

// Consumer thread (DSP)
loop {
    let available = rx.available();
    if available >= 1024 {
        let samples = rx.pop_slice(1024);
        process(&samples);
    }
}
```

### 2.2 Why Lock-Free?

| Mutex-Based | Lock-Free |
|-------------|-----------|
| Can block indefinitely | Never blocks |
| Priority inversion risk | No priority issues |
| Cache line bouncing | Cache-friendly |
| Simple to use | Requires careful design |

### 2.3 Buffer Pool

Pre-allocate to avoid runtime allocation:

```rust
use r4w_core::rt::BufferPool;

// Pre-allocate 16 buffers of 8192 samples each
let pool = BufferPool::<IQSample>::new(16, 8192);

// In hot path - no allocation!
fn process_samples(pool: &BufferPool<IQSample>) {
    let mut buffer = pool.acquire().expect("buffer available");

    // Use buffer...
    fill_with_samples(&mut buffer);

    // Automatic return on drop
}
```

### 2.4 Memory Locking

Prevent page faults:

```rust
use r4w_core::rt::LockedBuffer;

// Allocate and lock in physical memory
let buffer = LockedBuffer::<IQSample>::new(65536);

// Won't cause page faults during access
buffer[0] = sample;  // Guaranteed fast
```

---

## 3. RT Thread Configuration

### 3.1 Priority Scheduling

```rust
use r4w_core::rt::{RtThread, RtConfig, SchedulingPolicy};

let config = RtConfig {
    policy: SchedulingPolicy::Fifo,  // SCHED_FIFO
    priority: 80,                     // High priority
    cpu_affinity: Some(vec![0]),      // Pin to CPU 0
    lock_memory: true,                // mlockall()
};

let handle = RtThread::spawn(config, || {
    // This runs with RT priority
    loop {
        process_realtime_samples();
    }
});
```

### 3.2 CPU Affinity

Pin threads to specific cores:

```
┌─────────────────────────────────────────┐
│  CPU 0        CPU 1        CPU 2        │
│  ┌───────┐   ┌───────┐    ┌───────┐    │
│  │ RX    │   │ TX    │    │  GUI  │    │
│  │Thread │   │Thread │    │ Other │    │
│  └───────┘   └───────┘    └───────┘    │
│  Dedicated   Dedicated    Best effort  │
└─────────────────────────────────────────┘
```

### 3.3 Monitoring RT Performance

```rust
use r4w_core::rt::{RtStats, ProcessingTimer};

let stats = RtStats::new();

loop {
    let timer = ProcessingTimer::start(&stats);
    process_samples();
    timer.stop();  // Records duration
}

// Check for problems
let snapshot = stats.snapshot();
println!("Max latency: {} ns", snapshot.max_time_ns);
println!("Overruns: {}", snapshot.overruns);
println!("Underruns: {}", snapshot.underruns);
```

---

## 4. When Software Isn't Enough

### 4.1 Limits of Software SDR

| Sample Rate | CPU Per Sample | Practical? |
|-------------|---------------|------------|
| 1 MS/s | 1 μs | Yes |
| 10 MS/s | 100 ns | Challenging |
| 100 MS/s | 10 ns | Very hard |
| 1 GS/s | 1 ns | Impossible |

### 4.2 Enter the FPGA

FPGAs process samples **in parallel**:

```
Software (sequential):
  Sample 1 → Process → Done
                        Sample 2 → Process → Done
                                              ...
  Time ─────────────────────────────────────────►

FPGA (pipelined):
  Sample 1 → Stage 1 → Stage 2 → Stage 3 → Done
             Sample 2 → Stage 1 → Stage 2 → Stage 3 → Done
                        Sample 3 → Stage 1 → ...
  Time ─────────────────────────────────────────►
         All stages run simultaneously!
```

---

## 5. R4W FPGA Architecture

### 5.1 Hybrid Processing Model

```
┌───────────────────────────────────────────────────────┐
│                    Host Computer                      │
│  ┌─────────────────────────────────────────────────┐  │
│  │              R4W Software Stack                 │  │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐          │  │
│  │  │  CLI    │  │ Explorer│  │  API    │          │  │
│  │  └────┬────┘  └────┬────┘  └────┬────┘          │  │
│  │       └───────────┬┴───────────┘                │  │
│  │                   │                             │  │
│  │              ┌────▼────┐                        │  │
│  │              │ Waveform│                        │  │
│  │              │ Factory │                        │  │
│  │              └────┬────┘                        │  │
│  │                   │                             │  │
│  │    ┌──────────────┼──────────────┐              │  │
│  │    │              │              │              │  │
│  │ ┌──▼───┐     ┌────▼────┐    ┌────▼────┐         │  │
│  │ │ Soft │     │ Soft+HW │    │   HW    │         │  │
│  │ │ Only │     │ Hybrid  │    │  Only   │         │  │
│  │ └──────┘     └────┬────┘    └────┬────┘         │  │
│  └───────────────────┼──────────────┼──────────────┘  │
│                      │              │                 │
│            PCIe/USB  │              │                 │
└──────────────────────┼──────────────┼─────────────────┘
                       │              │
┌──────────────────────▼──────────────▼─────────────────┐
│                     FPGA Board                        │
│  ┌────────────────────────────────────────────────┐   │
│  │            Programmable Logic                  │   │
│  │  ┌─────────┐  ┌─────────┐  ┌─────────┐         │   │
│  │  │  FFT    │  │ Filters │  │ Chirp   │         │   │
│  │  │  Core   │  │ (FIR)   │  │ Gen     │         │   │
│  │  └────┬────┘  └────┬────┘  └────┬────┘         │   │
│  │       └───────────┬┴───────────┘               │   │
│  │              ┌────▼────┐                       │   │
│  │              │ AXI Bus │                       │   │
│  │              └────┬────┘                       │   │
│  │                   │                            │   │
│  │              ┌────▼────┐     ┌─────────┐       │   │
│  │              │  DMA    │────►│  DDR    │       │   │
│  │              └────┬────┘     └─────────┘       │   │
│  │                   │                            │   │
│  │              ┌────▼────┐                       │   │
│  │              │  ADC/   │                       │   │
│  │              │  DAC    │                       │   │
│  │              └─────────┘                       │   │
│  └────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────────┘
```

### 5.2 FPGA-Accelerated Waveforms

```rust
use r4w_core::fpga_accel::{FpgaWaveform, FpgaContext};

// Check for FPGA
let ctx = FpgaContext::detect()?;

// Create waveform with hardware acceleration
let waveform = LoRaWaveform::builder()
    .spreading_factor(10)
    .bandwidth(125_000)
    .fpga_context(Some(&ctx))  // Enable FPGA!
    .build();

// Same API, hardware acceleration transparent
let samples = waveform.modulate(&bits);
```

### 5.3 What Gets Accelerated

| Operation | FPGA Benefit | Speedup |
|-----------|--------------|---------|
| FFT | Massive parallelism | 100-1000x |
| FIR Filters | Pipelined MAC | 50-500x |
| Chirp Generation | Direct synthesis | 100x |
| Symbol Detection | Parallel correlators | 100x |
| Sample Rate Conversion | Pipelined | 50x |

---

## 6. Practical FPGA Setup

### 6.1 Supported Platforms

| Platform | Interface | Notes |
|----------|-----------|-------|
| Xilinx Zynq | AXI/DMA | Integrated ARM + FPGA |
| Intel Arria | PCIe | High performance |
| Lattice ECP5 | SPI/JTAG | Low cost |
| Xilinx Artix | PCIe | Mid-range |

### 6.2 Building FPGA Bitstream

```bash
# Install Vivado/Quartus
# Then:
cd fpga/

# Build for Zynq-7020
make PLATFORM=zynq7020

# Flash to board
make flash
```

### 6.3 Running with FPGA

```bash
# Software detects FPGA automatically
cargo run --bin r4w --features fpga -- modulate --waveform lora

# Or explicitly select
cargo run --bin r4w -- modulate --waveform lora --backend fpga
```

---

## 7. RT Debugging

### 7.1 Common Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Periodic dropouts | Page faults | Lock memory |
| Random glitches | Priority inversion | Use RT priorities |
| Buffer overflows | Too slow | Increase buffer size |
| Inconsistent latency | CPU frequency scaling | Disable scaling |

### 7.2 Diagnostic Tools

```rust
// Enable RT diagnostics
r4w_core::rt::enable_diagnostics();

// Run your workload...

// Print report
r4w_core::rt::print_diagnostics();
```

Output:
```
RT Diagnostics Report
─────────────────────
Processing Statistics:
  Iterations:       1,000,000
  Min latency:      120 ns
  Max latency:      890 ns (OK)
  Avg latency:      250 ns

Buffer Status:
  RX High Water:    12,345 samples
  RX Overflows:     0
  TX Low Water:     8,000 samples
  TX Underflows:    0

Thread Metrics:
  RT Thread 0:      99.9% on time
  RT Thread 1:      99.8% on time
```

---

## 8. Best Practices

### 8.1 RT Checklist

```
□ Pre-allocate all buffers before RT section
□ Lock memory pages (mlockall)
□ Set RT priority (SCHED_FIFO, priority 80+)
□ Pin RT threads to dedicated CPUs
□ Disable CPU frequency scaling on RT cores
□ Use lock-free data structures
□ Avoid system calls in hot path
□ Profile and test under load
```

### 8.2 Buffer Sizing

```
Buffer Size = Sample Rate × Max Latency × Safety Factor

Example:
  Sample Rate: 1 MS/s
  Max Latency: 10 ms
  Safety Factor: 4x

  Buffer Size = 1,000,000 × 0.010 × 4 = 40,000 samples
```

### 8.3 Latency vs Throughput

| Goal | Approach |
|------|----------|
| Low latency | Small buffers, frequent processing |
| High throughput | Large buffers, batch processing |
| Balanced | Adaptive buffering |

---

## 9. FPGA vs GPU

### 9.1 Comparison

| Aspect | FPGA | GPU |
|--------|------|-----|
| Latency | Microseconds | Milliseconds |
| Power | Low (5-50W) | High (100-300W) |
| Flexibility | Reconfigurable | Fixed architecture |
| Development | Hard (HDL) | Easier (CUDA) |
| Sample rates | 100+ MS/s | 10 MS/s typical |

### 9.2 When to Use Each

**Use FPGA:**
- Sample rate > 10 MS/s
- Latency critical (< 1ms)
- Power constrained
- Embedded deployment

**Use GPU:**
- Complex ML-based processing
- Existing CUDA code
- Batch processing acceptable
- Desktop/server deployment

---

## 10. Future: Accelerated Waveforms

### 10.1 What's Coming

R4W roadmap for acceleration:

1. **LoRa FPGA Core** - Full CSS in hardware
2. **Wideband FFT** - Up to 4K points at 100 MS/s
3. **Channelizer** - Extract multiple signals
4. **Neural Net Inference** - FPGA-based detection

### 10.2 Contributing

```rust
// FPGA-accelerated waveforms implement this trait
pub trait FpgaAccelerated: Waveform {
    /// Check if hardware is available
    fn fpga_available(&self) -> bool;

    /// Configure FPGA resources
    fn fpga_configure(&mut self, ctx: &FpgaContext) -> Result<()>;

    /// Modulate using FPGA
    fn fpga_modulate(&mut self, bits: &[bool]) -> Vec<IQSample>;

    /// Demodulate using FPGA
    fn fpga_demodulate(&mut self, samples: &[IQSample]) -> Vec<bool>;
}
```

---

## 11. Key Takeaways

1. **RT is about predictability**: Not just speed, but consistent speed
2. **Lock-free = Safe**: No blocking in hot paths
3. **Pre-allocate everything**: No malloc in RT section
4. **FPGA for heavy lifting**: Parallel processing at wire speed
5. **Profile always**: You can't optimize what you can't measure

---

## 12. Congratulations!

You've completed the R4W Workshop Series!

### What You've Learned:

- I/Q signals and complex samples
- Modulation: OOK → ASK → PSK → QAM
- Spread spectrum: DSSS, FHSS, LoRa CSS
- Channel effects: AWGN, fading, multipath
- Building custom waveforms
- Explorer GUI mastery
- Real-time programming
- FPGA acceleration concepts

### Next Steps:

1. **Build something!** Pick a waveform to implement
2. **Contribute** to R4W on GitHub
3. **Experiment** with real hardware (RTL-SDR, HackRF)
4. **Join** the community

---

## Quick Reference

```
┌───────────────────────────────────────────────────────────────┐
│              Real-Time / FPGA Reference                       │
├───────────────────────────────────────────────────────────────┤
│ RT Primitives:                                                │
│   RingBuffer<T>    - Lock-free SPSC queue                     │
│   BufferPool<T>    - Pre-allocated buffer pool                │
│   LockedBuffer<T>  - Memory-locked buffer                     │
│   RtThread         - RT priority thread                       │
│   RtStats          - Performance monitoring                   │
├───────────────────────────────────────────────────────────────┤
│ RT Configuration:                                             │
│   SCHED_FIFO       - Real-time scheduling                     │
│   Priority 80+     - High RT priority                         │
│   mlockall()       - Lock all pages                           │
│   CPU affinity     - Pin to cores                             │
├───────────────────────────────────────────────────────────────┤
│ FPGA Acceleration:                                            │
│   FpgaContext      - Hardware detection                       │
│   FpgaAccelerated  - Trait for HW waveforms                   │
│   AXI DMA          - High-speed data transfer                 │
├───────────────────────────────────────────────────────────────┤
│ Latency Targets:                                              │
│   1 MS/s  → 1 μs per sample   (software OK)                   │
│   10 MS/s → 100 ns per sample (optimized software)            │
│   100 MS/s → 10 ns per sample (FPGA required)                 │
└───────────────────────────────────────────────────────────────┘
```
