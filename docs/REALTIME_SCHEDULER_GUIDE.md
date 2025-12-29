# Real-Time Scheduler Guide

This guide covers the `RealTimeScheduler` module in R4W, designed for operational SDR systems requiring precise timing, TX/RX coordination, and hardware synchronization.

## Overview

While the [`TickScheduler`](./TICK_SCHEDULER_GUIDE.md) is designed for simulation with virtual time, the `RealTimeScheduler` is built for **production systems** with:

- **Wall-clock timing**: Events fire at precise real-time moments
- **TX/RX coordination**: State machine prevents simultaneous TX/RX in half-duplex systems
- **Guard conditions**: Events only fire when preconditions are met
- **Priority scheduling**: Critical events preempt lower priority ones
- **Thread-safe**: Multiple threads can schedule events concurrently
- **Hardware clock integration**: Sync with SDR device clocks, GPS/PTP

## When to Use Which Scheduler

| Scenario | Scheduler | Why |
|----------|-----------|-----|
| Educational visualization | TickScheduler | Virtual time, step-by-step |
| Unit testing protocols | TickScheduler | Deterministic, reproducible |
| Protocol development | TickScheduler | Pause, inspect, fast-forward |
| Production TX/RX | **RealTimeScheduler** | Wall-clock precision |
| Frequency hopping | **RealTimeScheduler** | Precise hop timing |
| TDMA/TDD systems | **RealTimeScheduler** | Slot synchronization |
| Multi-radio coordination | **RealTimeScheduler** | Prevent interference |

---

## Core Concepts

### 1. Radio State Machine

The scheduler manages radio state transitions to prevent invalid operations (e.g., TX during RX):

```
                    ┌──────────────────────────────────────────────┐
                    │              Radio State Machine             │
                    └──────────────────────────────────────────────┘

        ┌─────────────────────────────────────────────────────────────┐
        │                                                             │
        │    ┌─────────┐                           ┌─────────┐        │
        │    │  Idle   │◄─────────────────────────►│ Hopping │        │
        │    └────┬────┘                           └─────────┘        │
        │         │                                    ▲              │
        │    ┌────┴────┐                               │              │
        │    ▼         ▼                               │              │
        │ ┌─────────────────┐     ┌─────────────────┐  │              │
        │ │  Transmitting   │     │   Receiving     │  │              │
        │ └────────┬────────┘     └────────┬────────┘  │              │
        │          │                       │           │              │
        │          ▼                       ▼           │              │
        │ ┌─────────────────┐     ┌─────────────────┐  │              │
        │ │  TxTurnaround   │────►│  RxTurnaround   │──┘              │
        │ └─────────────────┘     └─────────────────┘                 │
        │          │                       │                          │
        │          └───────────────────────┘                          │
        │                    │                                        │
        │                    ▼                                        │
        │              ┌─────────┐                                    │
        │              │  Error  │──────────────────►(back to Idle)   │
        │              └─────────┘                                    │
        │                                                             │
        └─────────────────────────────────────────────────────────────┘
```

**State Descriptions:**

| State | Description | Can TX? | Can RX? |
|-------|-------------|---------|---------|
| `Idle` | Ready for operation | ✅ | ✅ |
| `Transmitting` | Actively transmitting | ❌ | ❌ |
| `TxTurnaround` | TX complete, switching to RX | ❌ | ✅ (after delay) |
| `Receiving` | Actively receiving | ❌ | ❌ |
| `RxTurnaround` | RX complete, switching to TX | ✅ (after delay) | ❌ |
| `Hopping` | Changing frequency | ❌ | ❌ |
| `Calibrating` | Calibration mode | ❌ | ❌ |
| `Error` | Error state | ❌ | ❌ |

### 2. Scheduled Events

Events have:
- **Deadline**: Absolute time in nanoseconds when the event should execute
- **Priority**: Lower number = higher priority (0 is highest)
- **Action**: What to do (StartTx, StopRx, SetFrequency, etc.)
- **Guard**: Optional condition that must be true for execution
- **Repeat interval**: For periodic events

```rust
use r4w_core::rt_scheduler::{ScheduledEvent, EventAction, RadioState};
use std::time::Duration;

// Schedule a TX start with guard condition
let event = ScheduledEvent::new(
    scheduler.now_ns() + 1_000_000,  // 1ms from now
    EventAction::StartTx,
)
.with_priority(0)                    // Highest priority
.with_guard(|state| state.can_transmit())  // Only if TX allowed
.with_source("my_protocol");         // For logging/cancellation
```

### 3. Clock Sources

The scheduler supports multiple clock sources:

| Clock Source | Use Case | Precision |
|--------------|----------|-----------|
| `System` | Development, testing | ~1μs |
| `Hpet` | Linux high-precision | ~100ns |
| `Tsc` | x86 cycle counter | ~10ns |
| `Gps` | Multi-device sync | ~50ns |
| `Ptp` | IEEE 1588 networks | ~100ns |
| `HardwareDevice` | SDR device clock | Varies |
| `Custom` | Testing, simulation | N/A |

---

## Basic Usage

### Creating a Scheduler

```rust
use r4w_core::rt_scheduler::{RealTimeScheduler, ClockSource};
use std::time::Duration;

let scheduler = RealTimeScheduler::builder()
    .clock_source(ClockSource::System)
    .tx_rx_turnaround(Duration::from_micros(100))  // 100μs TX→RX
    .rx_tx_turnaround(Duration::from_micros(100))  // 100μs RX→TX
    .deadline_tolerance(Duration::from_micros(500)) // 500μs late = missed
    .build()
    .unwrap();
```

### Scheduling Events

```rust
use r4w_core::rt_scheduler::{ScheduledEvent, EventAction};

// Schedule a single TX start
let event = ScheduledEvent::new(
    scheduler.now_ns() + 10_000_000,  // 10ms from now
    EventAction::StartTx,
);
scheduler.schedule(event);

// Schedule multiple events atomically
scheduler.schedule_batch(vec![
    ScheduledEvent::new(100_000_000, EventAction::StartTx),
    ScheduledEvent::new(110_000_000, EventAction::StopTx),
]);
```

### Processing Events

```rust
// Process all due events (non-blocking)
let results = scheduler.process();

for result in results {
    match result {
        Ok(event) => println!("Executed: {:?}", event.action),
        Err(e) => eprintln!("Failed: {}", e),
    }
}

// Or run continuously in a loop
std::thread::spawn(move || {
    scheduler.run().unwrap();
});

// Stop from another thread
scheduler.stop();
```

---

## Common Patterns

### Pattern 1: TX Burst

Schedule a transmission with automatic stop:

```rust
// Schedule 10ms TX burst starting in 5ms
let (start_id, stop_id) = scheduler.schedule_tx_burst(
    scheduler.now_ns() + 5_000_000,  // Start time
    Duration::from_millis(10),        // Duration
);

// Can cancel if needed
scheduler.cancel(start_id);
```

### Pattern 2: RX Window

Schedule a receive window:

```rust
// Listen for 50ms starting in 1ms
let (start_id, stop_id) = scheduler.schedule_rx_window(
    scheduler.now_ns() + 1_000_000,
    Duration::from_millis(50),
);
```

### Pattern 3: Frequency Hopping (FHSS)

Schedule a hop sequence:

```rust
let frequencies = vec![
    902_000_000,  // 902 MHz
    915_000_000,  // 915 MHz
    928_000_000,  // 928 MHz
];

// Hop every 10ms
let event_ids = scheduler.schedule_hop_sequence(
    scheduler.now_ns(),
    Duration::from_millis(10),
    &frequencies,
);

// Cancel all hops from a source
scheduler.cancel_from_source("fhss");
```

### Pattern 4: TDMA Frame

Schedule a TDMA frame with your TX slot:

```rust
// 4 slots, 10ms each, we transmit in slot 2
let event_ids = scheduler.schedule_tdma_frame(
    scheduler.now_ns(),
    Duration::from_millis(10),  // Slot duration
    2,                          // Our TX slot (0-indexed)
    4,                          // Total slots
);
```

### Pattern 5: Periodic Beacon

Schedule a repeating beacon:

```rust
// Beacon every 100ms, transmit for 5ms
let beacon_id = scheduler.schedule_beacon(
    scheduler.now_ns(),
    Duration::from_millis(100),  // Repeat interval
    Duration::from_millis(5),    // TX duration
);

// Cancel beacon
scheduler.cancel(beacon_id);
```

### Pattern 6: Callbacks

Schedule arbitrary code execution:

```rust
// Schedule a callback
let callback_id = scheduler.schedule_callback(
    scheduler.now_ns() + 1_000_000,
    |state| {
        println!("Callback fired! State: {:?}", state);
    },
);

// Or register callback and schedule separately
let id = scheduler.register_callback(|state| {
    // Handle event
});

let event = ScheduledEvent::new(
    deadline_ns,
    EventAction::Callback { callback_id: id },
);
scheduler.schedule(event);
```

---

## TX/RX Coordination

### Half-Duplex Operation

The scheduler automatically enforces half-duplex constraints:

```rust
// These will be serialized automatically
scheduler.schedule(ScheduledEvent::new(0, EventAction::StartTx)
    .with_guard(|s| s.can_transmit()));

scheduler.schedule(ScheduledEvent::new(1_000_000, EventAction::StartRx)
    .with_guard(|s| s.can_receive()));

// The RX will wait until TX completes + turnaround
```

### Turnaround Times

Configure turnaround delays:

```rust
let scheduler = RealTimeScheduler::builder()
    .tx_rx_turnaround(Duration::from_micros(50))   // TX→RX: 50μs
    .rx_tx_turnaround(Duration::from_micros(100))  // RX→TX: 100μs
    .build()?;
```

When `StopTx` executes:
1. State transitions to `TxTurnaround`
2. Scheduler auto-schedules transition to `Idle` after turnaround delay
3. RX can then start

### Direct State Access

For advanced control:

```rust
use r4w_core::rt_scheduler::RadioState;

// Check current state
let state = scheduler.state();
if state.can_transmit() {
    // Safe to TX
}

// Direct state manipulation (use carefully!)
scheduler.atomic_state().set(RadioState::Idle);

// Atomic transition with guard
let result = scheduler.atomic_state().transition_if(
    RadioState::Transmitting,
    |s| matches!(s, RadioState::Idle | RadioState::RxTurnaround),
);
```

---

## Event Handlers

Register handlers for custom event processing:

```rust
use r4w_core::rt_scheduler::{EventHandler, ScheduledEvent, RadioState, EventError};
use std::sync::Arc;

struct MyHandler;

impl EventHandler for MyHandler {
    fn handle(&self, event: &ScheduledEvent, state: &RadioState) -> Result<(), EventError> {
        match &event.action {
            EventAction::SetFrequency { hz } => {
                println!("Tuning to {} Hz", hz);
                // Actually tune your hardware here
                Ok(())
            }
            EventAction::Custom { id, data } => {
                println!("Custom event {}: {:?}", id, data);
                Ok(())
            }
            _ => Ok(()),  // Let default handling proceed
        }
    }

    fn name(&self) -> &str {
        "my_handler"
    }
}

scheduler.add_handler(Arc::new(MyHandler));
```

---

## Statistics and Monitoring

```rust
let stats = scheduler.stats();

println!("Events scheduled: {}", stats.events_scheduled);
println!("Events executed: {}", stats.events_executed);
println!("Deadlines missed: {}", stats.deadlines_missed);
println!("Guards blocked: {}", stats.guards_blocked);
println!("Max latency: {}ns", stats.max_latency_ns);
println!("Avg latency: {}ns", stats.avg_latency_ns);
println!("State transitions: {}", stats.state_transitions);

// Reset statistics
scheduler.reset_stats();
```

---

## Use Cases

### Use Case 1: TDD LTE-Like System

```rust
// TDD frame: 5ms downlink, 5ms uplink, repeating
fn schedule_tdd_frame(scheduler: &RealTimeScheduler, frame_start: u64) {
    let slot_duration = Duration::from_millis(5);

    // Downlink (we TX)
    scheduler.schedule_tx_burst(frame_start, slot_duration);

    // Uplink (we RX)
    let uplink_start = frame_start + slot_duration.as_nanos() as u64;
    scheduler.schedule_rx_window(uplink_start, slot_duration);

    // Schedule next frame
    let next_frame = frame_start + 2 * slot_duration.as_nanos() as u64;
    scheduler.schedule_callback(next_frame, move |_| {
        schedule_tdd_frame(scheduler, next_frame);
    });
}
```

### Use Case 2: SINCGARS-Style FHSS

```rust
// 100 hops/second with TX/RX alternation
fn schedule_sincgars_net(
    scheduler: &RealTimeScheduler,
    hop_freqs: &[u64],
    our_slot: usize,
    total_slots: usize,
) {
    let hop_interval = Duration::from_millis(10);  // 100 hops/sec
    let slot_duration = hop_interval / total_slots as u32;

    for (hop_idx, &freq) in hop_freqs.iter().enumerate() {
        let hop_start = scheduler.now_ns() + (hop_idx as u64 * hop_interval.as_nanos() as u64);

        // Schedule frequency change
        scheduler.schedule(ScheduledEvent::new(
            hop_start,
            EventAction::SetFrequency { hz: freq },
        ).with_priority(0));  // Highest priority

        // Schedule our TX slot within this hop
        let our_slot_start = hop_start + (our_slot as u64 * slot_duration.as_nanos() as u64);
        scheduler.schedule_tx_burst(our_slot_start, slot_duration);

        // RX during other slots
        for slot in 0..total_slots {
            if slot != our_slot {
                let slot_start = hop_start + (slot as u64 * slot_duration.as_nanos() as u64);
                scheduler.schedule_rx_window(slot_start, slot_duration);
            }
        }
    }
}
```

### Use Case 3: LoRa Class B Beacon Sync

```rust
// LoRa Class B: Listen for beacon, then open RX windows
fn schedule_class_b_windows(
    scheduler: &RealTimeScheduler,
    beacon_time: u64,
    ping_slots: &[u64],  // Offsets from beacon
) {
    // Open RX window for beacon
    scheduler.schedule_rx_window(
        beacon_time - Duration::from_millis(10).as_nanos() as u64,
        Duration::from_millis(20),
    );

    // Schedule ping slot windows
    for &offset in ping_slots {
        let window_start = beacon_time + offset;
        scheduler.schedule_rx_window(
            window_start,
            Duration::from_millis(30),  // Ping slot duration
        );
    }
}
```

### Use Case 4: Multi-Radio Coordination

```rust
use std::sync::Arc;

// Prevent two radios from TX simultaneously (co-site interference)
struct CoSiteCoordinator {
    radio1: Arc<RealTimeScheduler>,
    radio2: Arc<RealTimeScheduler>,
}

impl CoSiteCoordinator {
    fn schedule_tx(&self, radio: usize, start_ns: u64, duration: Duration) {
        let (sched, other) = match radio {
            1 => (&self.radio1, &self.radio2),
            2 => (&self.radio2, &self.radio1),
            _ => panic!("Invalid radio"),
        };

        // Guard: Only TX if other radio is not transmitting
        let other_state = other.atomic_state();
        let event = ScheduledEvent::new(start_ns, EventAction::StartTx)
            .with_guard(move |_| {
                !matches!(other_state.get(), RadioState::Transmitting)
            });

        sched.schedule(event);

        // Schedule stop
        sched.schedule(ScheduledEvent::new(
            start_ns + duration.as_nanos() as u64,
            EventAction::StopTx,
        ));
    }
}
```

---

## Integration with TickScheduler

Both schedulers can work together:

```rust
use r4w_core::scheduler::TickScheduler;
use r4w_core::rt_scheduler::RealTimeScheduler;

// Use TickScheduler for protocol development
let sim_scheduler = TickScheduler::builder()
    .resolution(TickResolution::Millisecond(1))
    .build();

// Test your protocol logic with virtual time...

// Then switch to RealTimeScheduler for production
let rt_scheduler = RealTimeScheduler::builder()
    .clock_source(ClockSource::System)
    .build()?;

// Same events, just scheduled on real-time scheduler
```

---

## Thread Safety

The `RealTimeScheduler` is designed for multi-threaded use:

```rust
use std::sync::Arc;
use std::thread;

let scheduler = Arc::new(RealTimeScheduler::builder().build()?);

// Scheduler thread
let sched_clone = scheduler.clone();
let handle = thread::spawn(move || {
    sched_clone.run().unwrap();
});

// Other threads can schedule events
let sched_clone = scheduler.clone();
thread::spawn(move || {
    loop {
        sched_clone.schedule(ScheduledEvent::new(
            sched_clone.now_ns() + 1_000_000,
            EventAction::StartTx,
        ).with_guard(|s| s.can_transmit()));

        thread::sleep(Duration::from_millis(100));
    }
});

// Stop from main thread
scheduler.stop();
handle.join().unwrap();
```

---

## Performance Considerations

### Deadline Tolerance

Configure based on your timing requirements:

```rust
let scheduler = RealTimeScheduler::builder()
    // Tight timing (FHSS, TDMA)
    .deadline_tolerance(Duration::from_micros(100))
    // Relaxed timing (beacons, periodic TX)
    .deadline_tolerance(Duration::from_millis(5))
    .build()?;
```

### Priority Levels

Use priorities to ensure critical events execute first:

| Priority | Use Case |
|----------|----------|
| 0 | Frequency hops, slot boundaries |
| 1 | TX/RX start/stop |
| 2 | Beacons, periodic events |
| 128 | Default |
| 255 | Low-priority maintenance |

### Memory Allocation

The scheduler uses heap allocation for the event queue. For hard real-time:

1. Pre-schedule events in batches
2. Use repeating events instead of scheduling new ones
3. Avoid callbacks that allocate

---

## Error Handling

```rust
use r4w_core::rt_scheduler::{EventError, RadioStateError};

let results = scheduler.process();
for result in results {
    match result {
        Ok(event) => { /* Success */ }
        Err(EventError::GuardBlocked) => {
            // Guard condition prevented execution
            // Event was consumed, may need to reschedule
        }
        Err(EventError::DeadlineMissed { deadline_ns, actual_ns }) => {
            // Event executed late
            let lateness = actual_ns - deadline_ns;
            eprintln!("Event late by {}ns", lateness);
        }
        Err(EventError::StateError(RadioStateError::InvalidTransition { from, to })) => {
            // State machine prevented transition
            eprintln!("Cannot go from {:?} to {:?}", from, to);
        }
        Err(e) => {
            eprintln!("Other error: {}", e);
        }
    }
}
```

---

## API Reference

### RealTimeScheduler

| Method | Description |
|--------|-------------|
| `builder()` | Create a scheduler builder |
| `now_ns()` | Current time in nanoseconds |
| `state()` | Current radio state |
| `schedule(event)` | Schedule a single event |
| `schedule_batch(events)` | Schedule multiple events atomically |
| `cancel(id)` | Cancel event by ID |
| `cancel_from_source(source)` | Cancel all events from source |
| `process()` | Process all due events |
| `run()` | Run scheduler loop until stopped |
| `stop()` | Stop the scheduler loop |
| `stats()` | Get performance statistics |

### Convenience Methods

| Method | Description |
|--------|-------------|
| `schedule_tx_burst(start, duration)` | Schedule TX with auto-stop |
| `schedule_rx_window(start, duration)` | Schedule RX window |
| `schedule_hop_sequence(start, interval, freqs)` | Schedule FHSS hops |
| `schedule_beacon(start, interval, tx_duration)` | Schedule repeating beacon |
| `schedule_tdma_frame(start, slot_duration, tx_slot, total_slots)` | Schedule TDMA frame |
| `schedule_callback(deadline, fn)` | Schedule code execution |

### EventAction Variants

| Action | Description |
|--------|-------------|
| `StartTx` | Begin transmission |
| `StopTx` | End transmission, enter turnaround |
| `StartRx` | Begin receiving |
| `StopRx` | End receiving, enter turnaround |
| `SetFrequency { hz }` | Change frequency |
| `SetPower { dbm }` | Change TX power |
| `StateTransition { target }` | Direct state change |
| `Callback { callback_id }` | Execute registered callback |
| `Reset` | Return to Idle state |
| `Calibrate` | Enter calibration mode |
| `Custom { id, data }` | Custom action for handlers |

---

## See Also

- [TICK_SCHEDULER_GUIDE.md](./TICK_SCHEDULER_GUIDE.md) - Simulation scheduler with virtual time
- [PHYSICAL_LAYER_GUIDE.md](./PHYSICAL_LAYER_GUIDE.md) - Timing model and RT primitives
- [FPGA_DEVELOPERS_GUIDE.md](./FPGA_DEVELOPERS_GUIDE.md) - Hardware timing integration
