# Tick Scheduler Guide

This guide covers the discrete event tick scheduler for coordinating SDR components with controllable time progression.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Core Concepts](#2-core-concepts)
3. [Basic Usage](#3-basic-usage)
4. [Tick Resolution](#4-tick-resolution)
5. [Time Scaling](#5-time-scaling)
6. [Sleep and Wake](#6-sleep-and-wake)
7. [Integration with Timing Model](#7-integration-with-timing-model)
8. [Use Cases](#8-use-cases)
9. [Architecture Patterns](#9-architecture-patterns)
10. [API Reference](#10-api-reference)

---

## 1. Overview

The tick scheduler provides a discrete event simulation (DES) framework for R4W applications. It enables:

- **Controllable time**: Speed up, slow down, pause, or step through time
- **Deterministic simulation**: Reproducible behavior for testing
- **Component coordination**: Synchronized TX/RX/channel simulation
- **Educational visualization**: Step-by-step signal processing

### Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      TickSchedule                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ TimeScale   │  │ CurrentTick │  │ SleepQueue          │  │
│  │ 0.0=paused  │  │ (monotonic) │  │ (wake_tick, id)     │  │
│  │ 0.5=slow    │  │             │  │                     │  │
│  │ 1.0=real    │  │             │  │                     │  │
│  │ 2.0=fast    │  │             │  │                     │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│                           │                                 │
│                           ▼                                 │
│  ┌──────────────────────────────────────────────────────┐   │
│  │              Subscriber Registry                     │   │
│  │  [Transmitter, Receiver, Channel, Monitor, ...]      │   │
│  └──────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                            │
            ┌───────────────┼───────────────┐
            ▼               ▼               ▼
     ┌──────────┐    ┌──────────┐    ┌──────────┐
     │ on_tick()│    │ on_tick()│    │ on_tick()│
     │ TX Comp  │    │ Channel  │    │ RX Comp  │
     └──────────┘    └──────────┘    └──────────┘
```

---

## 2. Core Concepts

### What is a Tick?

A **tick** is an abstract unit of time progression. Each tick:
- Advances the simulation by one time quantum
- Notifies all subscribed components
- Delivers any expired sleep requests

### TickEvent

When a tick occurs, components receive a `TickEvent`:

```rust
pub struct TickEvent {
    pub tick: u64,           // Current absolute tick number
    pub elapsed: u64,        // Ticks since component's last event
    pub sleep_expired: bool, // True if this is a wake-up from sleep
    pub component_id: u64,   // Subscriber's ID
    pub timestamp: Option<Timestamp>, // Timing info (if enabled)
}
```

### TickSubscriber Trait

Components implement `TickSubscriber` to receive tick notifications:

```rust
pub trait TickSubscriber: Send {
    fn on_tick(&mut self, event: TickEvent);
    fn id(&self) -> u64;

    // Optional lifecycle hooks
    fn on_register(&mut self, scheduler: &TickScheduler) {}
    fn on_unregister(&mut self) {}
}
```

---

## 3. Basic Usage

### Creating a Scheduler

```rust
use r4w_core::scheduler::{TickScheduler, TickResolution};

// Simple millisecond-based scheduler
let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));

// Sample-based scheduler (48 samples per tick at 48kHz)
let mut scheduler = TickScheduler::with_sample_rate(48000.0, 48);

// Using the builder
let mut scheduler = TickSchedulerBuilder::new()
    .milliseconds(5)
    .time_scale(1.0)
    .build();
```

### Subscribing Components

```rust
use std::sync::{Arc, Mutex};

struct MyComponent {
    id: u64,
    sample_count: u64,
}

impl TickSubscriber for MyComponent {
    fn on_tick(&mut self, event: TickEvent) {
        // Process tick - generate samples, update state, etc.
        self.sample_count += 48; // 48 samples per tick
        println!("Tick {}: processed {} samples", event.tick, self.sample_count);
    }

    fn id(&self) -> u64 { self.id }
}

let component = Arc::new(Mutex::new(MyComponent { id: 1, sample_count: 0 }));
scheduler.subscribe(component);
```

### Running the Scheduler

```rust
// Step manually (for debugging)
scheduler.step(10);  // Advance 10 ticks

// Run continuously
scheduler.set_time_scale(1.0);  // Real-time
scheduler.run();  // Blocks until stopped

// In another thread, control the scheduler:
scheduler.pause();
scheduler.set_time_scale(0.5);  // Half speed
scheduler.resume();
scheduler.stop();
```

---

## 4. Tick Resolution

The tick resolution defines what one tick represents in terms of time or samples.

### Available Resolutions

| Resolution | Description | Use Case |
|------------|-------------|----------|
| `Samples { count, sample_rate }` | N samples at given rate | Bit-accurate DSP simulation |
| `Symbol { symbol_rate, sample_rate }` | One symbol period | Waveform testing |
| `Frame { samples_per_frame, sample_rate }` | One frame/packet | Protocol simulation |
| `Microsecond(us)` | N microseconds | High-precision timing |
| `Millisecond(ms)` | N milliseconds | General simulation |
| `Custom(Duration)` | Custom duration | Specialized needs |

### Examples

```rust
// 1 tick = 48 samples at 48kHz (1ms)
let res = TickResolution::Samples { count: 48, sample_rate: 48000.0 };

// 1 tick = 1 symbol at 9600 baud
let res = TickResolution::Symbol { symbol_rate: 9600.0, sample_rate: 48000.0 };

// 1 tick = 1 LoRa frame (4096 samples at SF12)
let res = TickResolution::Frame { samples_per_frame: 4096, sample_rate: 125000.0 };

// 1 tick = 100 microseconds
let res = TickResolution::Microsecond(100);
```

### Choosing Resolution

```
High Resolution                              Low Resolution
(fine-grained)                               (coarse-grained)
    │                                              │
    ▼                                              ▼
  Sample → Symbol → Frame → Millisecond → Second

  More ticks/sec           Fewer ticks/sec
  Higher overhead          Lower overhead
  More precision           Less precision
```

**Guidelines:**
- **Bit-level simulation**: Use `Samples` with small count
- **Symbol-level testing**: Use `Symbol`
- **Packet-level protocol**: Use `Frame`
- **Real-time visualization**: Use `Millisecond(10-50)`

---

## 5. Time Scaling

Time scaling controls simulation speed relative to real-time.

### Scale Values

| Scale | Behavior |
|-------|----------|
| `0.0` | Paused (no ticks generated) |
| `0.5` | Half speed (2x slower) |
| `1.0` | Real-time |
| `2.0` | Double speed |
| `10.0` | 10x faster than real-time |

### Example: Educational Slow Motion

```rust
// Slow motion for step-by-step visualization
scheduler.set_time_scale(0.1);  // 10x slower than real-time

// Each tick shows one symbol's processing
// User can see constellation points being placed
```

### Example: Fast Testing

```rust
// Fast-forward through simulation for testing
scheduler.set_time_scale(100.0);  // 100x faster

// Run 1 million ticks quickly
scheduler.step(1_000_000);
```

### Example: Pause and Step

```rust
// Pause for debugging
scheduler.set_time_scale(0.0);

// Step one tick at a time
scheduler.step(1);
// Inspect state...
scheduler.step(1);
// Inspect state...

// Resume at normal speed
scheduler.set_time_scale(1.0);
```

---

## 6. Sleep and Wake

Components can request wake-up after N ticks, enabling event-driven behavior.

### Basic Sleep

```rust
impl TickSubscriber for MyComponent {
    fn on_tick(&mut self, event: TickEvent) {
        if event.sleep_expired {
            // This is a wake-up from our sleep request
            println!("Woke up at tick {}", event.tick);
        } else {
            // Normal tick processing
            // Schedule wake-up in 100 ticks
            // (scheduler reference needed - see patterns below)
        }
    }

    fn id(&self) -> u64 { self.id }
}

// From outside the subscriber:
scheduler.sleep(100, component_id);  // Wake component after 100 ticks
```

### Sleep Patterns

**Pattern 1: Periodic Wake-up**
```rust
// Wake every 100 ticks
fn on_tick(&mut self, event: TickEvent) {
    if event.sleep_expired || event.tick == 1 {
        self.do_periodic_work();
        // Would need scheduler reference to schedule next wake
    }
}
```

**Pattern 2: Timeout**
```rust
// Wait for event with timeout
fn on_tick(&mut self, event: TickEvent) {
    if event.sleep_expired {
        // Timeout expired - no response received
        self.handle_timeout();
    } else if self.response_received {
        // Got response before timeout
        // Cancel pending sleep (from outside)
    }
}
```

### Canceling Sleep

```rust
// Cancel all pending sleeps for a component
scheduler.cancel_sleeps(component_id);
```

---

## 7. Integration with Timing Model

The tick scheduler integrates with R4W's multi-clock timing model.

### Tick to Timestamp Conversion

```rust
// Create scheduler with sample rate for timing integration
let scheduler = TickScheduler::with_sample_rate(48000.0, 48);

// Get timestamp at any tick
let ts = scheduler.tick_to_timestamp(1000);

// Access all clock domains
println!("Samples: {}", ts.sample.samples());      // 48000
println!("Wall time: {} ns", ts.wall.as_nanos());  // ~1 second
```

### Timestamp in TickEvent

```rust
fn on_tick(&mut self, event: TickEvent) {
    if let Some(ts) = &event.timestamp {
        // Access precise timing
        let sample = ts.sample.samples();
        let wall_ns = ts.wall.as_nanos();

        // Correlate with hardware timestamps if available
        if let Some(hw) = &ts.hardware {
            let hw_ticks = hw.ticks();
        }
    }
}
```

---

## 8. Use Cases

### Use Case 1: Educational Signal Visualization

Show modulation step-by-step:

```rust
struct ModulationVisualizer {
    id: u64,
    bits: Vec<bool>,
    current_bit: usize,
    samples: Vec<IQSample>,
}

impl TickSubscriber for ModulationVisualizer {
    fn on_tick(&mut self, event: TickEvent) {
        if self.current_bit < self.bits.len() {
            // Modulate one bit per tick
            let bit = self.bits[self.current_bit];
            let sample = if bit {
                IQSample::new(1.0, 0.0)  // BPSK '1'
            } else {
                IQSample::new(-1.0, 0.0) // BPSK '0'
            };

            self.samples.push(sample);
            self.current_bit += 1;

            // Update GUI constellation display
            self.update_display(&sample);
        }
    }

    fn id(&self) -> u64 { self.id }
}

// Run at 0.5x speed for slow-motion visualization
scheduler.set_time_scale(0.5);
```

### Use Case 2: TX/RX Channel Simulation

Simulate complete RF path:

```rust
struct ChannelSimulation {
    tx: Arc<Mutex<Transmitter>>,
    channel: Arc<Mutex<Channel>>,
    rx: Arc<Mutex<Receiver>>,
}

// TX generates samples on each tick
impl TickSubscriber for Transmitter {
    fn on_tick(&mut self, event: TickEvent) {
        let samples = self.modulate_next_symbol();
        self.output_buffer.extend(samples);
    }
    fn id(&self) -> u64 { 0 }
}

// Channel applies impairments
impl TickSubscriber for Channel {
    fn on_tick(&mut self, event: TickEvent) {
        while let Some(sample) = self.tx_buffer.pop() {
            let impaired = self.apply_awgn(sample, self.snr_db);
            let delayed = self.apply_multipath(impaired);
            self.rx_buffer.push(delayed);
        }
    }
    fn id(&self) -> u64 { 1 }
}

// RX demodulates
impl TickSubscriber for Receiver {
    fn on_tick(&mut self, event: TickEvent) {
        if self.rx_buffer.len() >= self.symbol_samples {
            let samples: Vec<_> = self.rx_buffer.drain(..self.symbol_samples).collect();
            let symbol = self.demodulate(&samples);
            self.decoded_symbols.push(symbol);
        }
    }
    fn id(&self) -> u64 { 2 }
}
```

### Use Case 3: Protocol Timing Verification

Test timing-critical protocols:

```rust
struct ProtocolTimer {
    id: u64,
    state: ProtocolState,
    timeout_ticks: u64,
}

impl TickSubscriber for ProtocolTimer {
    fn on_tick(&mut self, event: TickEvent) {
        match self.state {
            ProtocolState::WaitingForAck => {
                if event.sleep_expired {
                    // Timeout - retransmit
                    self.retransmit();
                    self.state = ProtocolState::WaitingForAck;
                    // Schedule another timeout
                }
            }
            ProtocolState::Idle => {
                // Check for new data to send
            }
            _ => {}
        }
    }
    fn id(&self) -> u64 { self.id }
}
```

### Use Case 4: Deterministic Testing

Reproducible test scenarios:

```rust
#[test]
fn test_ber_at_10db_snr() {
    let mut scheduler = TickScheduler::new(TickResolution::Samples {
        count: 48,
        sample_rate: 48000.0,
    });

    // Seed RNG for reproducibility
    let mut rng = StdRng::seed_from_u64(12345);

    let tx = Arc::new(Mutex::new(Transmitter::new(&mut rng)));
    let channel = Arc::new(Mutex::new(Channel::new(10.0, &mut rng)));  // 10 dB SNR
    let rx = Arc::new(Mutex::new(Receiver::new()));

    scheduler.subscribe(tx);
    scheduler.subscribe(channel);
    scheduler.subscribe(rx.clone());

    // Run exactly 10000 ticks
    scheduler.step(10000);

    // Check BER
    let ber = rx.lock().unwrap().calculate_ber();
    assert!(ber < 1e-3, "BER {} exceeds threshold at 10dB", ber);
}
```

### Use Case 5: Multi-Device Synchronization

Simulate coordinated radios:

```rust
// GPS-synchronized radios
struct SyncedRadio {
    id: u64,
    tx_slot: u64,  // Which tick to transmit
}

impl TickSubscriber for SyncedRadio {
    fn on_tick(&mut self, event: TickEvent) {
        // All radios see the same tick
        let slot = event.tick % 10;  // 10-slot TDMA

        if slot == self.tx_slot {
            self.transmit();
        } else {
            self.receive();
        }
    }
    fn id(&self) -> u64 { self.id }
}

// Radio 0 transmits on slot 0, Radio 1 on slot 1, etc.
for i in 0..10 {
    let radio = Arc::new(Mutex::new(SyncedRadio { id: i, tx_slot: i }));
    scheduler.subscribe(radio);
}
```

---

## 9. Architecture Patterns

### Pattern: Scheduler as Shared State

```rust
use std::sync::{Arc, RwLock};

struct SimulationContext {
    scheduler: Arc<RwLock<TickScheduler>>,
    components: Vec<Arc<Mutex<dyn TickSubscriber>>>,
}

impl SimulationContext {
    fn request_sleep(&self, ticks: u64, component_id: u64) {
        self.scheduler.write().unwrap().sleep(ticks, component_id);
    }
}
```

### Pattern: Message Passing Between Components

```rust
use std::sync::mpsc::{channel, Sender, Receiver};

struct Producer {
    id: u64,
    tx: Sender<IQSample>,
}

struct Consumer {
    id: u64,
    rx: Receiver<IQSample>,
}

impl TickSubscriber for Producer {
    fn on_tick(&mut self, event: TickEvent) {
        let sample = self.generate();
        self.tx.send(sample).ok();
    }
    fn id(&self) -> u64 { self.id }
}

impl TickSubscriber for Consumer {
    fn on_tick(&mut self, event: TickEvent) {
        while let Ok(sample) = self.rx.try_recv() {
            self.process(sample);
        }
    }
    fn id(&self) -> u64 { self.id }
}
```

### Pattern: State Machine Components

```rust
enum RadioState {
    Idle,
    Transmitting { remaining: u64 },
    Receiving,
    Processing,
}

struct StateMachineRadio {
    id: u64,
    state: RadioState,
}

impl TickSubscriber for StateMachineRadio {
    fn on_tick(&mut self, event: TickEvent) {
        self.state = match &self.state {
            RadioState::Idle => {
                if self.has_data_to_send() {
                    RadioState::Transmitting { remaining: 100 }
                } else {
                    RadioState::Receiving
                }
            }
            RadioState::Transmitting { remaining } => {
                self.transmit_sample();
                if *remaining > 1 {
                    RadioState::Transmitting { remaining: remaining - 1 }
                } else {
                    RadioState::Processing
                }
            }
            RadioState::Receiving => {
                self.receive_sample();
                RadioState::Receiving
            }
            RadioState::Processing => {
                self.process_received();
                RadioState::Idle
            }
        };
    }
    fn id(&self) -> u64 { self.id }
}
```

---

## 10. API Reference

### TickScheduler

| Method | Description |
|--------|-------------|
| `new(resolution)` | Create scheduler with given resolution |
| `with_sample_rate(rate, samples_per_tick)` | Create with sample-based timing |
| `subscribe(component)` | Register a component |
| `unsubscribe(id)` | Remove a component |
| `tick()` | Advance by one tick |
| `step(n)` | Advance by N ticks |
| `run()` | Run continuously until stopped |
| `pause()` | Pause the scheduler |
| `resume()` | Resume from pause |
| `stop()` | Stop the scheduler |
| `reset()` | Reset to initial state |
| `sleep(ticks, id)` | Schedule wake-up for component |
| `cancel_sleeps(id)` | Cancel pending sleeps |
| `set_time_scale(scale)` | Set time scaling factor |
| `current_tick()` | Get current tick number |
| `tick_to_timestamp(tick)` | Convert tick to Timestamp |
| `stats()` | Get scheduler statistics |

### TickSchedulerBuilder

| Method | Description |
|--------|-------------|
| `new()` | Create builder |
| `resolution(res)` | Set tick resolution |
| `samples(count, rate)` | Set sample-based resolution |
| `symbol_rate(sym_rate, sample_rate)` | Set symbol-based resolution |
| `milliseconds(ms)` | Set millisecond resolution |
| `time_scale(scale)` | Set initial time scale |
| `build()` | Build the scheduler |

### SchedulerStats

| Field | Description |
|-------|-------------|
| `total_ticks` | Total ticks processed |
| `sleep_requests` | Total sleep requests |
| `events_delivered` | Total events sent |
| `pending_sleeps` | Current pending sleep count |
| `subscriber_count` | Current subscriber count |
| `avg_tick_duration_us` | Average tick processing time |
| `max_tick_duration_us` | Maximum tick processing time |

---

## Quick Reference

```
┌──────────────────────────────────────────────────────────────┐
│                 Tick Scheduler Quick Reference               │
├──────────────────────────────────────────────────────────────┤
│ Time Scales:                                                 │
│   0.0  = Paused (manual stepping only)                       │
│   0.5  = Half speed (2x slower)                              │
│   1.0  = Real-time                                           │
│   2.0  = Double speed                                        │
│   10.0 = 10x faster                                          │
├──────────────────────────────────────────────────────────────┤
│ Tick Resolutions:                                            │
│   Samples    - For bit-accurate simulation                   │
│   Symbol     - For waveform testing                          │
│   Frame      - For protocol testing                          │
│   Millisecond- For visualization                             │
├──────────────────────────────────────────────────────────────┤
│ TickEvent Fields:                                            │
│   tick           - Current absolute tick (u64)               │
│   elapsed        - Ticks since last event (u64)              │
│   sleep_expired  - Is this a wake-up? (bool)                 │
│   component_id   - Subscriber's ID (u64)                     │
│   timestamp      - Optional Timestamp                        │
├──────────────────────────────────────────────────────────────┤
│ Common Patterns:                                             │
│   scheduler.step(1)     - Single step (debugging)            │
│   scheduler.step(1000)  - Batch processing                   │
│   scheduler.run()       - Continuous (blocks)                │
│   scheduler.sleep(n,id) - Wake after n ticks                 │
└──────────────────────────────────────────────────────────────┘
```
