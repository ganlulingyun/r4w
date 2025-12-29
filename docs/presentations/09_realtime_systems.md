---
title: '<img src="images/r4w_logo_1.png" height="150px" style="margin-bottom: 20px;"><br>Real-Time Systems'
subtitle: "Deterministic Scheduling for SDR"
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

## Why Real-Time for SDR?

SDR systems have strict timing requirements:

| Protocol | Timing Constraint |
|----------|-------------------|
| Frequency hopping | Hop within microseconds |
| TDMA slots | Slot boundaries exact |
| TX/RX turnaround | Hardware limits |
| Preamble detection | Symbol-accurate |

Miss a deadline = lost data or interference.

---

## Two Schedulers

R4W provides two scheduling systems:

| Scheduler | Use Case | Time Model |
|-----------|----------|------------|
| **TickScheduler** | Simulation, testing | Virtual ticks |
| **RealTimeScheduler** | Production systems | Wall clock |

Same event API, different timing backends.

---

## Real-Time Scheduler Features

- **Wall-clock timing** - Events fire at precise moments
- **TX/RX coordination** - State machine prevents conflicts
- **Guard conditions** - Events only fire when preconditions met
- **Priority scheduling** - Critical events preempt
- **Thread-safe** - Multiple threads schedule concurrently
- **Hardware clock integration** - GPS/PTP sync

---

## Radio State Machine

```
      ┌─────────┐                     ┌─────────┐
      │  Idle   │◄───────────────────►│ Hopping │
      └────┬────┘                     └─────────┘
           │
      ┌────┴────┐
      ▼         ▼
┌───────────┐     ┌───────────┐
│Transmitting│     │ Receiving │
└─────┬─────┘     └─────┬─────┘
      │                 │
      ▼                 ▼
┌───────────┐     ┌───────────┐
│TxTurnaround│────►│RxTurnaround│
└───────────┘     └───────────┘
```

---

## State Transitions

| State | Can TX? | Can RX? | Description |
|-------|---------|---------|-------------|
| Idle | Yes | Yes | Ready |
| Transmitting | No | No | TX active |
| TxTurnaround | No | After delay | TX→RX switch |
| Receiving | No | No | RX active |
| RxTurnaround | After delay | No | RX→TX switch |
| Hopping | No | No | Changing frequency |
| Error | No | No | Fault state |

---

## Creating a Scheduler

```rust
use r4w_core::rt_scheduler::{
    RealTimeScheduler, ClockSource
};
use std::time::Duration;

let scheduler = RealTimeScheduler::builder()
    .clock_source(ClockSource::System)
    .tx_rx_turnaround(Duration::from_micros(100))
    .rx_tx_turnaround(Duration::from_micros(100))
    .deadline_tolerance(Duration::from_micros(500))
    .build()?;
```

---

## Scheduling Events

```rust
use r4w_core::rt_scheduler::{ScheduledEvent, EventAction};

// Single event
let event = ScheduledEvent::new(
    scheduler.now_ns() + 10_000_000,  // 10ms from now
    EventAction::StartTx,
);
scheduler.schedule(event);

// Atomic batch
scheduler.schedule_batch(vec![
    ScheduledEvent::new(100_000_000, EventAction::StartTx),
    ScheduledEvent::new(110_000_000, EventAction::StopTx),
]);
```

---

## Event Actions

| Action | Description |
|--------|-------------|
| `StartTx` | Begin transmission |
| `StopTx` | End transmission |
| `StartRx` | Begin reception |
| `StopRx` | End reception |
| `SetFrequency(f)` | Change center frequency |
| `SetGain(g)` | Adjust gain |
| `Custom(fn)` | User-defined action |

---

## Guard Conditions

Events fire only when preconditions met:

```rust
let event = ScheduledEvent::new(
    scheduler.now_ns() + 1_000_000,
    EventAction::StartTx,
)
.with_guard(|state| state.can_transmit())
.with_priority(0);  // Highest priority
```

Guards prevent invalid state transitions.

---

## Clock Sources

| Source | Precision | Use Case |
|--------|-----------|----------|
| `System` | ~1us | Development |
| `Hpet` | ~100ns | Linux high-precision |
| `Tsc` | ~10ns | x86 cycle counter |
| `Gps` | ~50ns | Multi-device sync |
| `Ptp` | ~100ns | IEEE 1588 networks |
| `HardwareDevice` | Varies | SDR device clock |

---

## Processing Events

```rust
// Non-blocking: process all due events
let results = scheduler.process();

for result in results {
    match result {
        Ok(event) => println!("Executed: {:?}", event.action),
        Err(e) => eprintln!("Failed: {}", e),
    }
}

// Or run continuously in dedicated thread
std::thread::spawn(move || {
    scheduler.run().unwrap();
});
```

---

## Pattern: TX Burst

Schedule transmission with automatic stop:

```rust
let (start_id, stop_id) = scheduler.schedule_tx_burst(
    scheduler.now_ns() + 5_000_000,  // Start in 5ms
    Duration::from_millis(10),        // Duration
);

// Cancel if needed
scheduler.cancel(start_id);
```

---

## Pattern: RX Window

Schedule receive window:

```rust
let (start_id, stop_id) = scheduler.schedule_rx_window(
    scheduler.now_ns() + 1_000_000,  // Start in 1ms
    Duration::from_millis(50),        // Window duration
);
```

Automatic state transitions.

---

## Pattern: Frequency Hopping

```rust
let hop_sequence = vec![915.0e6, 916.0e6, 917.0e6, 918.0e6];
let dwell_time = Duration::from_millis(10);

for (i, &freq) in hop_sequence.iter().enumerate() {
    let hop_time = scheduler.now_ns()
        + (i as u64) * dwell_time.as_nanos() as u64;

    scheduler.schedule(ScheduledEvent::new(
        hop_time,
        EventAction::SetFrequency(freq),
    ).with_priority(0));  // High priority
}
```

---

## Pattern: TDMA Slot

```rust
let slot_duration = Duration::from_millis(5);
let my_slot = 3;  // Assigned slot number

loop {
    let frame_start = scheduler.next_frame_start();
    let my_slot_time = frame_start
        + my_slot * slot_duration.as_nanos() as u64;

    // TX only in assigned slot
    scheduler.schedule_tx_burst(
        my_slot_time,
        slot_duration - GUARD_TIME,
    );
}
```

---

## Tick Scheduler (Simulation)

For development and testing:

```rust
use r4w_core::tick_scheduler::TickScheduler;

let mut scheduler = TickScheduler::new();

// Schedule in virtual ticks
scheduler.schedule(10, |_| println!("Tick 10!"));
scheduler.schedule(20, |_| println!("Tick 20!"));

// Advance time manually
scheduler.advance_to(25);  // Executes both events
```

---

## Tick vs Real-Time

| Feature | TickScheduler | RealTimeScheduler |
|---------|---------------|-------------------|
| Time unit | Virtual ticks | Nanoseconds |
| Determinism | Perfect | OS-dependent |
| Debugging | Step through | Real-time |
| Testing | Reproducible | Hardware-dependent |
| Performance | Fast | Actual speed |

---

## Real-Time Validation Results

| Metric | Target | Actual |
|--------|--------|--------|
| FFT p99 latency | < 100 us | **18 us** |
| BPSK roundtrip p99 | < 100 us | **20 us** |
| FHSS hop timing p99 | < 500 us | **80-118 us** |
| Page faults (RT mode) | 0 | **0** |
| Hot-path allocations | 0 | **0** |

---

## RT-Safe Primitives

Lock-free data structures:

```rust
use r4w_core::rt::RingBuffer;

// SPSC lock-free queue
let buffer: RingBuffer<IQSample> = RingBuffer::new(1024);

// Producer (no locks)
buffer.push(sample)?;

// Consumer (no locks)
if let Some(sample) = buffer.pop() {
    process(sample);
}
```

---

## Memory Locking

Prevent page faults:

```rust
use r4w_core::rt::LockedBuffer;

// Allocate and lock memory
let buffer = LockedBuffer::<IQSample>::new(4096)?;

// Guaranteed no page faults
for sample in buffer.iter() {
    process_sample(sample);
}
```

Uses `mlockall()` on Linux.

---

## RT Thread Configuration

```rust
use r4w_core::rt::RtThreadConfig;

let config = RtThreadConfig {
    priority: 80,           // SCHED_FIFO priority
    cpu_affinity: Some(vec![0]),  // Pin to CPU 0
    lock_memory: true,      // mlockall
};

let handle = config.spawn(|| {
    // Real-time processing loop
    scheduler.run()
})?;
```

---

## Latency Budget

Typical SDR processing chain:

| Stage | Latency |
|-------|---------|
| ADC/DAC | 1-10 us |
| Driver transfer | 10-100 us |
| DSP processing | 10-50 us |
| Scheduler overhead | 1-5 us |
| **Total** | 22-165 us |

Budget: Know your deadlines!

---

## Deadline Handling

```rust
let event = ScheduledEvent::new(deadline, action)
    .with_deadline_tolerance(Duration::from_micros(100));

// If > 100us late:
// - Event is logged as missed
// - Optional callback for recovery
// - Metrics updated
```

---

## Metrics and Observability

```rust
let stats = scheduler.get_stats();

println!("Events scheduled: {}", stats.total_scheduled);
println!("Events executed: {}", stats.total_executed);
println!("Missed deadlines: {}", stats.missed_deadlines);
println!("Max latency: {} ns", stats.max_latency_ns);
println!("Avg latency: {} ns", stats.avg_latency_ns);
```

---

## Error Recovery

```rust
scheduler.set_error_handler(|error| {
    match error {
        SchedulerError::MissedDeadline(event) => {
            log::warn!("Missed: {:?}", event);
            // Attempt recovery
        }
        SchedulerError::InvalidTransition(from, to) => {
            log::error!("Invalid: {} -> {}", from, to);
            // Reset state machine
        }
    }
});
```

---

## Integration Example

Complete FHSS system:

```rust
let mut scheduler = RealTimeScheduler::builder()
    .clock_source(ClockSource::Gps)
    .build()?;

// Generate hop sequence
for (i, freq) in hop_sequence.iter().enumerate() {
    scheduler.schedule(ScheduledEvent::new(
        start + i * dwell,
        EventAction::SetFrequency(*freq),
    ));

    // TX burst each hop
    scheduler.schedule_tx_burst(
        start + i * dwell + SETTLE_TIME,
        Duration::from_micros(800),
    );
}

scheduler.run()?;
```

---

## Summary

| Component | Purpose |
|-----------|---------|
| **RealTimeScheduler** | Wall-clock event scheduling |
| **TickScheduler** | Simulation/testing |
| **State Machine** | TX/RX coordination |
| **Clock Sources** | GPS/PTP/system time |
| **RT Primitives** | Lock-free, zero-alloc |
| **Metrics** | Latency tracking |

**Deterministic timing for SDR.**

---

## Questions?

**R4W - Real-Time Systems**

github.com/joemooney/r4w

Docs: `docs/REALTIME_SCHEDULER_GUIDE.md`
