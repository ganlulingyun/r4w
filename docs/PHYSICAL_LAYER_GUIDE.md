# Physical Layer Developer's Guide

This guide covers the physical layer infrastructure in R4W that sits beneath the waveform abstraction. These components handle timing, hardware abstraction, real-time processing, configuration, and observability.

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Timing Model](#2-timing-model)
3. [Hardware Abstraction Layer (HAL)](#3-hardware-abstraction-layer-hal)
4. [Real-Time Primitives](#4-real-time-primitives)
5. [Configuration System](#5-configuration-system)
6. [Observability](#6-observability)
7. [Integration Examples](#7-integration-examples)

---

## 1. Architecture Overview

The physical layer provides the foundation for all SDR operations:

```
┌─────────────────────────────────────────────────────────────┐
│                    Waveform Layer                           │
│             (LoRa, PSK, QAM, FSK, etc.)                     │
├─────────────────────────────────────────────────────────────┤
│                  HAL Interface (Rust traits)                │
│   SdrDeviceExt, StreamHandle, TunerControl, ClockControl    │
├───────────────┬───────────────┬─────────────────────────────┤
│   Simulator   │   File I/O    │  Hardware Drivers           │
│               │   (SigMF)     │  (UHD, SoapySDR, RTL-SDR)   │
├───────────────┴───────────────┴─────────────────────────────┤
│                  RT Primitives Layer                        │
│   RingBuffer, BufferPool, LockedBuffer, RtThread            │
├─────────────────────────────────────────────────────────────┤
│                  Timing Layer                               │
│   SampleClock, WallClock, HardwareClock, SyncedTime         │
├─────────────────────────────────────────────────────────────┤
│                  Configuration & Observability              │
│   R4wConfig (YAML), Metrics, Logging, Tracing               │
└─────────────────────────────────────────────────────────────┘
```

### Key Crates

| Crate | Module | Purpose |
|-------|--------|---------|
| `r4w-core` | `timing` | Multi-clock timing model |
| `r4w-core` | `rt` | Real-time primitives |
| `r4w-core` | `config` | YAML configuration |
| `r4w-core` | `observe` | Logging, metrics |
| `r4w-sim` | `hal` | Hardware abstraction |

---

## 2. Timing Model

**Module**: `r4w_core::timing`

The timing model provides a unified abstraction across multiple clock domains, essential for correlating sample time with real-world time and synchronizing multiple devices.

### 2.1 Clock Types

```rust
use r4w_core::timing::{SampleClock, WallClock, HardwareClock, SyncedTime, Timestamp};

// Sample clock - counts samples monotonically
let mut sample_clock = SampleClock::new(48000.0); // 48 kHz sample rate
sample_clock.advance(48000); // Advance by 1 second of samples
let duration = sample_clock.to_duration(); // Convert to Duration

// Wall clock - system time with nanosecond precision
let wall = WallClock::now();
let unix_secs = wall.as_secs();
let unix_nanos = wall.as_nanos();

// Hardware clock - device timestamps (tick-based)
let hw = HardwareClock::new(100_000_000.0); // 100 MHz tick rate
let samples = hw.ticks_to_samples(hw.ticks(), 48000.0);

// Synced time - GPS/PTP synchronized (TAI-based)
let synced = SyncedTime::from_utc_nanos(wall.as_nanos(), TimeSource::Gps);
let tai = synced.tai_nanos();
```

### 2.2 Unified Timestamp

The `Timestamp` struct combines all clock domains:

```rust
use r4w_core::timing::Timestamp;

// Create a timestamp at the current time
let mut ts = Timestamp::new(48000.0);

// Add hardware clock info
ts = ts.with_hardware(HardwareClock::new(100_000_000.0));

// Add GPS-synced time
ts = ts.with_synced(SyncedTime::from_utc_nanos(
    WallClock::now().as_nanos(),
    TimeSource::Gps,
));

// Advance all clocks together
ts.advance_samples(48000); // Advances sample, wall, and hardware clocks

// Query best time source
match ts.best_time_source() {
    TimeSource::Gps => println!("GPS synchronized"),
    TimeSource::Ptp => println!("PTP synchronized"),
    TimeSource::Freerun => println!("Free-running"),
    _ => {}
}
```

### 2.3 Clock Domain Conversions

The `ClockDomain` trait enables conversions between clock types:

```rust
use r4w_core::timing::ClockDomain;

let sample_clock = SampleClock::at_sample(48000, 48000.0);

// Convert to samples (at a given sample rate)
let samples = sample_clock.to_samples(48000.0);

// Convert to wall clock nanoseconds
let nanos = sample_clock.to_wall_nanos();

// Convert to Duration
let duration = sample_clock.to_duration();
```

### 2.4 Rate Limiting for Real-Time

The `RateLimiter` helps maintain consistent timing for streaming:

```rust
use r4w_core::timing::RateLimiter;
use std::thread;

let mut limiter = RateLimiter::new(48000.0);

loop {
    // Process samples...
    let samples_processed = 480; // 10ms worth

    // Get sleep duration to maintain rate
    if let Some(sleep_duration) = limiter.record(samples_processed) {
        thread::sleep(sleep_duration);
    }

    // Check if we're keeping up
    if !limiter.is_realtime() {
        eprintln!("Warning: not keeping up with real-time!");
    }
}
```

### 2.5 High-Resolution Timing

For measuring intervals with nanosecond precision:

```rust
use r4w_core::timing::MonotonicTimer;

let timer = MonotonicTimer::start();

// Do some work...

let elapsed = timer.elapsed();
println!("Took {:?}", elapsed);

// Lap timing
let mut timer = MonotonicTimer::start();
for i in 0..10 {
    process_batch();
    let lap = timer.lap(); // Returns elapsed and resets
    println!("Batch {} took {:?}", i, lap);
}
```

---

## 3. Hardware Abstraction Layer (HAL)

**Module**: `r4w_sim::hal`

The HAL provides a layered abstraction for SDR hardware, enabling the same waveform code to run on simulators, file I/O, or real hardware.

### 3.1 Core Traits

#### StreamHandle - Streaming I/Q Samples

```rust
use r4w_sim::hal::{StreamHandle, StreamConfig, StreamDirection};
use std::time::Duration;

// Configure and create a stream
let config = StreamConfig {
    direction: StreamDirection::Rx,
    channels: vec![0],
    buffer_size: 8192,
    num_buffers: 4,
    ..Default::default()
};

let mut stream = device.create_rx_stream(config)?;

// Start streaming
stream.start()?;

// Read samples with timeout
let mut buffer = vec![IQSample::default(); 1024];
let (count, timestamp) = stream.read(&mut buffer, Duration::from_millis(100))?;

// Check for overflows
let status = stream.status();
if status.overflow_count > 0 {
    eprintln!("Overflow! {} samples lost", status.overflow_count);
}

// Stop streaming
stream.stop()?;
```

#### TunerControl - Frequency and Gain

```rust
use r4w_sim::hal::TunerControl;

// Get the tuner interface
let tuner = device.tuner();

// Set parameters (returns actual value set)
let actual_freq = tuner.set_frequency(915_000_000)?;
let actual_rate = tuner.set_sample_rate(1_000_000.0)?;
let actual_gain = tuner.set_rx_gain(30.0)?;

// Query capabilities
let (min_freq, max_freq) = tuner.frequency_range();
let (min_gain, max_gain) = tuner.gain_range();
let antennas = tuner.available_antennas();

// Set antenna port
tuner.set_antenna("RX2")?;
```

#### ClockControl - Synchronization

```rust
use r4w_sim::hal::{ClockControl, ClockSource};
use r4w_core::timing::TimeSource;

// Get clock control (if supported)
if let Some(clock) = device.clock() {
    // Use external reference
    clock.set_clock_source(ClockSource::External)?;
    clock.set_time_source(TimeSource::Gps)?;

    // Wait for PPS and set time
    clock.wait_for_pps(Duration::from_secs(2))?;
    clock.set_time_at_pps(timestamp)?;

    // Check lock status
    if clock.is_locked() {
        println!("Locked to external reference");
    }
}
```

### 3.2 Device Discovery and Creation

```rust
use r4w_sim::hal::{DriverRegistry, create_default_registry};

// Create registry with built-in drivers
let registry = create_default_registry();

// List available drivers
for driver_name in registry.list() {
    println!("Driver: {}", driver_name);
}

// Discover all devices
for (driver, info) in registry.discover_all() {
    println!("{}: {} ({})", driver, info.name, info.serial);
}

// Create device from URI
let device = registry.create("soapysdr://driver=hackrf")?;

// Or get specific driver
if let Some(driver) = registry.get("rtlsdr") {
    let devices = driver.discover();
    if let Some(info) = devices.first() {
        let device = driver.create(info)?;
    }
}
```

### 3.3 File I/O with SigMF

The HAL includes SigMF (Signal Metadata Format) support for recording and playback:

```rust
use r4w_sim::hal::{SigMfWriter, SigMfReader, SigMfMeta};

// Write samples to SigMF file
let meta = SigMfMeta {
    sample_rate: 1_000_000.0,
    center_frequency: 915_000_000.0,
    author: "R4W".to_string(),
    description: "LoRa capture".to_string(),
    ..Default::default()
};

let mut writer = SigMfWriter::create("capture", &meta)?;
writer.write_samples(&samples)?;
writer.close()?;

// Read samples from SigMF file
let mut reader = SigMfReader::open("capture")?;
let meta = reader.metadata();
println!("Sample rate: {}", meta.sample_rate);

let mut buffer = vec![IQSample::default(); 1024];
let count = reader.read_samples(&mut buffer)?;
```

---

## 4. Real-Time Primitives

**Module**: `r4w_core::rt`

The RT module provides lock-free, allocation-free primitives for real-time signal processing.

### 4.1 Lock-Free Ring Buffer

Single-producer, single-consumer (SPSC) ring buffer for streaming between threads:

```rust
use r4w_core::rt::RingBuffer;
use std::sync::Arc;
use std::thread;

// Create ring buffer (capacity rounds up to power of 2)
let ring: Arc<RingBuffer<f32>> = Arc::new(RingBuffer::new(1024));

// Producer thread
let ring_tx = Arc::clone(&ring);
thread::spawn(move || {
    loop {
        // Single element
        ring_tx.push(sample).unwrap();

        // Batch push (returns count actually pushed)
        let pushed = ring_tx.push_slice(&samples);

        // Blocking push (spins until complete)
        ring_tx.push_slice_blocking(&large_batch);
    }
});

// Consumer thread
let ring_rx = Arc::clone(&ring);
thread::spawn(move || {
    loop {
        // Single element
        if let Some(sample) = ring_rx.pop() {
            process(sample);
        }

        // Batch pop
        let mut buffer = [0.0f32; 256];
        let popped = ring_rx.pop_slice(&mut buffer);

        // Check status
        println!("Buffer level: {}/{}", ring_rx.len(), ring_rx.capacity());
    }
});
```

### 4.2 Buffer Pool

Pre-allocated buffers to avoid runtime allocation:

```rust
use r4w_core::rt::BufferPool;

// Create pool: 8 buffers, each 4096 samples
let pool: BufferPool<f32> = BufferPool::new(8, 4096);

// Acquire a buffer (lock-free)
let mut buf = pool.acquire()?;

// Use the buffer
buf.as_mut_slice()[0] = 1.0;
let data = buf.as_slice();

// Buffer automatically returns to pool when dropped
drop(buf);

// Check availability
println!("Available: {}/{}", pool.available(), pool.buffer_count());

// Blocking acquire (spins until available)
let buf = pool.acquire_blocking();
```

### 4.3 Memory-Locked Buffers

Prevent page faults in real-time paths:

```rust
use r4w_core::rt::LockedBuffer;

// Allocate and lock memory (requires privileges)
let mut buf: LockedBuffer<f32> = LockedBuffer::new(4096)?;

// Check if locking succeeded
if buf.is_locked() {
    println!("Memory locked - no page faults possible");
}

// Use like a normal slice
buf.as_mut_slice()[0] = 1.0;
let slice = buf.as_slice();

// Memory is unlocked and freed on drop
```

### 4.4 Real-Time Threads

Spawn threads with RT priority and CPU affinity:

```rust
use r4w_core::rt::{RtConfig, RtPriority, spawn_rt_thread};

// Configure RT thread
let config = RtConfig::builder()
    .name("rx_thread")
    .priority(RtPriority::High)      // SCHED_FIFO 80 on Linux
    .cpu_affinity(&[0])              // Pin to CPU 0
    .lock_memory(true)               // mlockall()
    .stack_size(1024 * 1024)         // 1 MB stack
    .build();

// Spawn the thread
let handle = spawn_rt_thread(config, || {
    // Real-time processing loop
    loop {
        let samples = read_samples();
        process(samples);
    }
})?;

// Get suggested CPU allocation
let (rx_cpus, tx_cpus, dsp_cpus) = suggest_cpu_allocation();
println!("RX: {:?}, TX: {:?}, DSP: {:?}", rx_cpus, tx_cpus, dsp_cpus);
```

### 4.5 Utility Functions

```rust
use r4w_core::rt::{secure_zero, prefault_pages, align_up, CACHE_LINE_SIZE};

// Securely zero memory (prevents compiler optimization)
let mut secret = [0u8; 32];
// ... use secret ...
secure_zero(&mut secret);

// Pre-touch pages to avoid page faults
let mut buffer = vec![0u8; 1_000_000];
prefault_pages(&mut buffer);

// Align to cache line
let aligned = align_up(100, CACHE_LINE_SIZE); // Returns 128
```

---

## 5. Configuration System

**Module**: `r4w_core::config`

The configuration system provides YAML-based boot configuration with sensible defaults.

### 5.1 Configuration Structure

```yaml
# r4w.yaml - Example configuration
version: "1.0"

device:
  driver: "soapysdr"
  args: "driver=hackrf"
  sample_rate: 2.4e6
  center_frequency: 915e6
  rx_gain: 40.0
  tx_gain: 30.0
  antenna: "RX"
  clock_source: "internal"
  time_source: "internal"

buffers:
  rx_ring_size: 1048576      # 1M samples
  tx_ring_size: 262144       # 256K samples
  dma_buffer_count: 8
  dma_buffer_size: 65536
  overflow_policy: drop      # drop, block, or warn

realtime:
  enable: true
  rx_priority: 80
  tx_priority: 80
  dsp_priority: 70
  lock_memory: true
  cpu_affinity:
    rx: [0]
    tx: [1]
    dsp: [2, 3]

logging:
  level: "info"              # trace, debug, info, warn, error
  format: compact            # compact, json, full
  file: "/var/log/r4w/r4w.log"
  rotate_size: "100MB"
  rotate_keep: 5

metrics:
  enable: true
  bind: "0.0.0.0:9090"
  path: "/metrics"

capture:
  enable: false
  format: sigmf              # sigmf or raw
  path: "/var/capture/r4w"
  rotate_size: "1GB"

# Hardware profiles for quick switching
profiles:
  rtlsdr_v3:
    driver: "rtlsdr"
    sample_rate: 2.4e6
    rx_gain: 40.0

  usrp_b200:
    driver: "uhd"
    args: "type=b200"
    sample_rate: 10e6
    clock_source: "gpsdo"
```

### 5.2 Loading Configuration

```rust
use r4w_core::config::R4wConfig;

// Load from default search path:
// 1. $R4W_CONFIG environment variable
// 2. ./r4w.yaml
// 3. ~/.config/r4w/config.yaml
// 4. /etc/r4w/config.yaml
let config = R4wConfig::load()?;

// Load from specific file
let config = R4wConfig::load_from(Path::new("/path/to/config.yaml"))?;

// Parse from string
let yaml = r#"
device:
  driver: "rtlsdr"
  sample_rate: 2.4e6
"#;
let config = R4wConfig::parse(yaml)?;

// Validate configuration
config.validate()?;
```

### 5.3 Using Hardware Profiles

```rust
use r4w_core::config::R4wConfig;

let config = R4wConfig::load()?;

// Switch to RTL-SDR profile
let rtl_config = config.with_profile("rtlsdr_v3")?;
assert_eq!(rtl_config.device.driver, "rtlsdr");

// Switch to USRP B200 profile
let usrp_config = config.with_profile("usrp_b200")?;
assert_eq!(usrp_config.device.driver, "uhd");
```

### 5.4 Generating Example Configuration

```rust
use r4w_core::config::R4wConfig;

// Generate example YAML
let example = R4wConfig::example_yaml();
println!("{}", example);

// Save configuration
let config = R4wConfig::default();
config.save(Path::new("./r4w.yaml"))?;
```

---

## 6. Observability

**Module**: `r4w_core::observe`

Three-pillar observability: logging, metrics, and tracing.

### 6.1 Structured Logging

```rust
use r4w_core::observe::{init_logging, LogConfig, LogLevel, LogFormat};

// Initialize logging (call once at startup)
let config = LogConfig {
    level: LogLevel::Debug,
    format: LogFormat::Json,
    timestamps: true,
    source_location: true,
    ..Default::default()
};
init_logging(&config);

// Use preset configurations
init_logging(&LogConfig::development()); // Debug, pretty, verbose
init_logging(&LogConfig::production());  // Info, JSON, minimal
init_logging(&LogConfig::quiet());       // Errors only

// Use tracing macros
tracing::info!(samples = 1024, "Processed batch");
tracing::debug!(freq = 915e6, gain = 30.0, "Tuner configured");
tracing::warn!("Buffer overflow detected");
tracing::error!(error = ?e, "Device failed");

// Spans for structured context
let span = tracing::info_span!("rx_processing", channel = 0);
let _enter = span.enter();
// All logs in this scope include channel=0
```

### 6.2 Metrics Collection

```rust
use r4w_core::observe::Metrics;

// Create metrics instance
let metrics = Metrics::new();

// Sample counters
metrics.rx_samples.inc_by(1024);
metrics.tx_samples.inc_by(512);

// Buffer level gauges
metrics.rx_buffer_level.set(4096);
metrics.tx_buffer_level.set(1024);

// Error counters
metrics.rx_overflows.inc();
metrics.crc_errors.inc();

// Signal metrics (scaled internally)
metrics.record_rssi(-80.5);  // dBm
metrics.record_snr(12.3);    // dB

// Packet counters
metrics.packets_decoded.inc();
metrics.packets_failed.inc();

// Latency histogram
metrics.processing_latency_us.observe(150.0);

// Set active waveform
metrics.set_waveform("lora");

// Get snapshot
let snap = metrics.snapshot();
println!("RX samples: {}", snap.rx_samples);
println!("Success rate: {:.1}%", snap.decode_success_rate() * 100.0);
println!("Avg latency: {:.1} us", snap.avg_latency_us());

// Export as Prometheus format
let prometheus = metrics.to_prometheus();
// Returns text like:
// # HELP r4w_rx_samples_total Total RX samples processed
// # TYPE r4w_rx_samples_total counter
// r4w_rx_samples_total 1024
```

### 6.3 Custom Metrics

```rust
use r4w_core::observe::metrics::{Counter, Gauge, Histogram};

// Custom counter
let my_counter = Counter::new();
my_counter.inc();
my_counter.inc_by(100);
println!("Count: {}", my_counter.get());

// Custom gauge
let my_gauge = Gauge::new();
my_gauge.set(50);
my_gauge.inc();
my_gauge.dec();
my_gauge.add(-10);

// Custom histogram with specific buckets
let my_hist = Histogram::new(vec![1.0, 5.0, 10.0, 50.0, 100.0]);
my_hist.observe(7.5);
println!("Count: {}, Sum: {}", my_hist.count(), my_hist.sum());
```

### 6.4 Initialization Helper

```rust
use r4w_core::observe::{init, LogConfig};

// Initialize all observability subsystems
let log_config = LogConfig::default();
let metrics = init(&log_config);

// Now use metrics and logging
metrics.rx_samples.inc();
tracing::info!("System initialized");
```

---

## 7. Integration Examples

### 7.1 Complete SDR Application Setup

```rust
use r4w_core::config::R4wConfig;
use r4w_core::observe::{init, LogConfig};
use r4w_core::rt::{RingBuffer, RtConfig, RtPriority, spawn_rt_thread};
use r4w_core::timing::Timestamp;
use r4w_sim::hal::{create_default_registry, StreamConfig};
use std::sync::Arc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // 1. Load configuration
    let config = R4wConfig::load()?;
    config.validate()?;

    // 2. Initialize observability
    let log_config = LogConfig::production();
    let metrics = init(&log_config);
    tracing::info!("Starting R4W application");

    // 3. Create ring buffer for streaming
    let ring: Arc<RingBuffer<IQSample>> = Arc::new(
        RingBuffer::new(config.buffers.rx_ring_size)
    );

    // 4. Create device
    let registry = create_default_registry();
    let uri = format!("{}://{}", config.device.driver, config.device.args);
    let mut device = registry.create(&uri)?;

    // 5. Configure device
    {
        let tuner = device.tuner();
        tuner.set_frequency(config.device.center_frequency as u64)?;
        tuner.set_sample_rate(config.device.sample_rate)?;
        tuner.set_rx_gain(config.device.rx_gain)?;
    }

    // 6. Spawn RT producer thread
    let ring_tx = Arc::clone(&ring);
    let rt_config = RtConfig::builder()
        .name("rx_thread")
        .priority(RtPriority::Custom(config.realtime.rx_priority))
        .cpu_affinity(&config.realtime.cpu_affinity.rx)
        .lock_memory(config.realtime.lock_memory)
        .build();

    let stream_config = StreamConfig::default();
    let mut stream = device.create_rx_stream(stream_config)?;
    stream.start()?;

    spawn_rt_thread(rt_config, move || {
        let mut buffer = vec![IQSample::default(); 8192];
        loop {
            if let Ok((count, _ts)) = stream.read(&mut buffer, Duration::from_millis(100)) {
                ring_tx.push_slice(&buffer[..count]);
            }
        }
    })?;

    // 7. Consumer loop (could be another RT thread)
    let ring_rx = Arc::clone(&ring);
    loop {
        let mut buffer = vec![IQSample::default(); 1024];
        let popped = ring_rx.pop_slice(&mut buffer);

        if popped > 0 {
            metrics.rx_samples.inc_by(popped as u64);
            // Process samples...
        }

        // Update buffer metrics
        metrics.rx_buffer_level.set(ring_rx.len() as i64);
    }
}
```

### 7.2 Multi-Device Synchronized Setup

```rust
use r4w_core::timing::{SyncedTime, TimeSource, Timestamp};
use r4w_sim::hal::ClockSource;

// Configure both devices for GPS sync
for device in [&mut device1, &mut device2] {
    if let Some(clock) = device.clock() {
        clock.set_clock_source(ClockSource::Gpsdo)?;
        clock.set_time_source(TimeSource::Gps)?;
    }
}

// Wait for both to lock
for device in [&mut device1, &mut device2] {
    if let Some(clock) = device.clock() {
        while !clock.is_locked() {
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

// Set synchronized time at next PPS
let sync_time = SyncedTime::from_utc_nanos(
    WallClock::now().as_nanos() + 2_000_000_000, // 2 seconds from now
    TimeSource::Gps,
);
let timestamp = Timestamp::new(sample_rate).with_synced(sync_time);

for device in [&mut device1, &mut device2] {
    if let Some(clock) = device.clock() {
        clock.set_time_at_pps(timestamp.clone())?;
    }
}

tracing::info!("Devices synchronized to GPS");
```

### 7.3 Recording Session with Metrics

```rust
use r4w_core::timing::MonotonicTimer;
use r4w_sim::hal::{SigMfWriter, SigMfMeta};

let meta = SigMfMeta {
    sample_rate: config.device.sample_rate,
    center_frequency: config.device.center_frequency,
    description: "Capture session".to_string(),
    ..Default::default()
};

let mut writer = SigMfWriter::create("capture", &meta)?;
let timer = MonotonicTimer::start();

loop {
    let mut buffer = vec![IQSample::default(); 8192];
    let (count, timestamp) = stream.read(&mut buffer, Duration::from_millis(100))?;

    writer.write_samples(&buffer[..count])?;
    metrics.rx_samples.inc_by(count as u64);

    // Check for issues
    let status = stream.status();
    if status.overflow_count > 0 {
        metrics.rx_overflows.inc_by(status.overflow_count);
        tracing::warn!(overflows = status.overflow_count, "RX overflows");
    }

    // Stop after 10 seconds
    if timer.elapsed() > Duration::from_secs(10) {
        break;
    }
}

writer.close()?;
let snap = metrics.snapshot();
tracing::info!(
    samples = snap.rx_samples,
    overflows = snap.rx_overflows,
    "Recording complete"
);
```

---

## Quick Reference

### Timing Types

| Type | Use Case |
|------|----------|
| `SampleClock` | DSP operations, sample counting |
| `WallClock` | Logging, debugging, correlation |
| `HardwareClock` | Device timestamps, sync |
| `SyncedTime` | GPS/PTP, multi-device |
| `Timestamp` | Unified timestamp |

### HAL Traits

| Trait | Purpose |
|-------|---------|
| `StreamHandle` | Read/write samples |
| `TunerControl` | Freq, gain, rate |
| `ClockControl` | Time sync, PPS |
| `SdrDeviceExt` | High-level device |
| `DeviceDriver` | Device factory |

### RT Primitives

| Type | Purpose |
|------|---------|
| `RingBuffer<T>` | Lock-free SPSC queue |
| `BufferPool<T>` | Pre-allocated buffers |
| `LockedBuffer<T>` | mlock'd memory |
| `RtConfig` | Thread configuration |

### Config Sections

| Section | Purpose |
|---------|---------|
| `device` | SDR driver and settings |
| `buffers` | Ring/DMA buffer sizes |
| `realtime` | Thread priorities, affinity |
| `logging` | Log level, format, output |
| `metrics` | Prometheus endpoint |
| `profiles` | Hardware presets |

---

## See Also

- [Waveform Developer's Guide](WAVEFORM_DEVELOPERS_GUIDE.md) - Building waveforms on top of this layer
- [FPGA Developer's Guide](FPGA_DEVELOPERS_GUIDE.md) - Hardware acceleration
- [Workshops](../workshops/) - Hands-on tutorials
