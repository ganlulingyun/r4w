# Workshop 11: Real-Time Scheduling for SDR Systems

## Overview

**Duration:** 90 minutes
**Level:** Advanced
**Prerequisites:** Workshops 1-4, basic understanding of TDMA/TDD concepts

In this workshop, you'll learn how to use R4W's `RealTimeScheduler` to coordinate TX/RX operations, implement frequency hopping, and build TDMA-style protocols with precise timing.

## Learning Objectives

By the end of this workshop, you will be able to:

1. Understand the radio state machine and TX/RX constraints
2. Schedule events with deadlines, priorities, and guards
3. Implement frequency hopping sequences
4. Build a simple TDMA frame structure
5. Coordinate multiple protocol components with callbacks

---

## Part 1: Understanding the Radio State Machine (15 min)

### The Problem: Half-Duplex Radio

Most SDR hardware is half-duplex: it can either transmit OR receive, but not both simultaneously. Switching between TX and RX takes time (turnaround delay).

```
Bad: TX and RX overlap → Hardware conflict!

    TX: ████████████
    RX:       ████████████
              ↑ COLLISION

Good: TX and RX are serialized with turnaround

    TX: ████████████
                    ╔══╗  ← Turnaround (100μs typical)
    RX:             ╚══╝████████████
```

### The State Machine

R4W's `RealTimeScheduler` enforces these constraints with a state machine:

```rust
use r4w_core::rt_scheduler::{RadioState, AtomicRadioState};

fn main() {
    let state = AtomicRadioState::new(RadioState::Idle);

    // Check what operations are allowed
    println!("Idle - Can TX: {}", state.get().can_transmit());  // true
    println!("Idle - Can RX: {}", state.get().can_receive());   // true

    // Transition to TX
    state.transition(RadioState::Idle, RadioState::Transmitting).unwrap();
    println!("Transmitting - Can TX: {}", state.get().can_transmit());  // false
    println!("Transmitting - Can RX: {}", state.get().can_receive());   // false

    // Try invalid transition (TX → RX directly)
    let result = state.transition(RadioState::Transmitting, RadioState::Receiving);
    println!("TX→RX direct: {:?}", result);  // Err(InvalidTransition)

    // Correct path: TX → TxTurnaround → RX
    state.transition(RadioState::Transmitting, RadioState::TxTurnaround).unwrap();
    state.transition(RadioState::TxTurnaround, RadioState::Receiving).unwrap();
    println!("Now receiving!");
}
```

### Exercise 1.1: State Transitions

Draw the state diagram showing all valid transitions from `Idle`. Which states can you reach in exactly 2 transitions?

<details>
<summary>Solution</summary>

From Idle in 1 transition:
- Transmitting
- Receiving
- Hopping
- Calibrating

From Idle in 2 transitions:
- Idle → Transmitting → TxTurnaround
- Idle → Transmitting → Error
- Idle → Receiving → RxTurnaround
- Idle → Receiving → Error
- Idle → Hopping → Idle
- Idle → Hopping → Error
- Idle → Calibrating → Idle
- Idle → Calibrating → Error

</details>

---

## Part 2: Basic Event Scheduling (20 min)

### Creating a Scheduler

```rust
use r4w_core::rt_scheduler::{RealTimeScheduler, ClockSource};
use std::time::Duration;

fn main() {
    // Create with system clock
    let scheduler = RealTimeScheduler::builder()
        .clock_source(ClockSource::System)
        .tx_rx_turnaround(Duration::from_micros(100))
        .rx_tx_turnaround(Duration::from_micros(100))
        .build()
        .unwrap();

    println!("Current time: {} ns", scheduler.now_ns());
    println!("Current state: {:?}", scheduler.state());
}
```

### Scheduling a TX Burst

```rust
use r4w_core::rt_scheduler::{ScheduledEvent, EventAction};

// Schedule TX to start in 10ms, run for 5ms
let start_time = scheduler.now_ns() + 10_000_000;  // 10ms from now
let duration = Duration::from_millis(5);

let (start_id, stop_id) = scheduler.schedule_tx_burst(start_time, duration);

println!("Scheduled TX burst:");
println!("  Start event ID: {}", start_id);
println!("  Stop event ID: {}", stop_id);
println!("  Pending events: {}", scheduler.pending_count());
```

### Processing Events

```rust
use std::thread;

// Process loop
loop {
    let results = scheduler.process();

    for result in &results {
        match result {
            Ok(event) => {
                println!("[{}ns] Executed: {:?}",
                    scheduler.now_ns(), event.action);
            }
            Err(e) => {
                println!("[{}ns] Error: {}", scheduler.now_ns(), e);
            }
        }
    }

    // Check current state
    println!("State: {:?}", scheduler.state());

    // Exit when done
    if scheduler.pending_count() == 0 &&
       matches!(scheduler.state(), RadioState::Idle) {
        break;
    }

    thread::sleep(Duration::from_millis(1));
}
```

### Exercise 2.1: TX/RX Sequence

Schedule this sequence:
1. TX for 10ms starting now
2. RX for 20ms after TX completes
3. TX for 5ms after RX completes

Use `schedule_tx_burst` and `schedule_rx_window`.

<details>
<summary>Solution</summary>

```rust
let now = scheduler.now_ns();
let turnaround = 100_000;  // 100μs

// TX 1: Now for 10ms
let tx1_start = now;
let tx1_duration = Duration::from_millis(10);
scheduler.schedule_tx_burst(tx1_start, tx1_duration);

// RX: After TX1 + turnaround, for 20ms
let rx_start = tx1_start + tx1_duration.as_nanos() as u64 + turnaround;
let rx_duration = Duration::from_millis(20);
scheduler.schedule_rx_window(rx_start, rx_duration);

// TX 2: After RX + turnaround, for 5ms
let tx2_start = rx_start + rx_duration.as_nanos() as u64 + turnaround;
let tx2_duration = Duration::from_millis(5);
scheduler.schedule_tx_burst(tx2_start, tx2_duration);
```

</details>

---

## Part 3: Guard Conditions (15 min)

### Why Guards?

Guards prevent events from executing when conditions aren't right:

```rust
// This TX will only execute if radio is idle or in RX turnaround
let event = ScheduledEvent::new(
    scheduler.now_ns() + 1_000_000,
    EventAction::StartTx,
)
.with_guard(|state| state.can_transmit());

scheduler.schedule(event);
```

### Guard Failure

When a guard fails, the event is consumed but not executed:

```rust
// Put scheduler in TX state manually
scheduler.atomic_state().set(RadioState::Transmitting);

// Try to schedule another TX with guard
let event = ScheduledEvent::new(
    scheduler.now_ns(),  // Execute immediately
    EventAction::StartTx,
)
.with_guard(|state| state.can_transmit());

scheduler.schedule(event);
let results = scheduler.process();

// Check result
match &results[0] {
    Err(EventError::GuardBlocked) => {
        println!("Guard prevented TX during existing TX");
    }
    _ => panic!("Expected guard block"),
}
```

### Exercise 3.1: Safe RX After TX

Write a function that schedules an RX window that will only start if the radio is able to receive (not in TX or RX already):

```rust
fn safe_rx_window(
    scheduler: &RealTimeScheduler,
    start_ns: u64,
    duration: Duration,
) -> u64 {
    // TODO: Create event with guard condition
    // Return the event ID
}
```

<details>
<summary>Solution</summary>

```rust
fn safe_rx_window(
    scheduler: &RealTimeScheduler,
    start_ns: u64,
    duration: Duration,
) -> u64 {
    let start_event = ScheduledEvent::new(start_ns, EventAction::StartRx)
        .with_priority(1)
        .with_guard(|state| state.can_receive())
        .with_source("safe_rx");

    let start_id = start_event.id;

    let stop_event = ScheduledEvent::new(
        start_ns + duration.as_nanos() as u64,
        EventAction::StopRx,
    )
    .with_priority(1)
    .with_source("safe_rx");

    scheduler.schedule(start_event);
    scheduler.schedule(stop_event);

    start_id
}
```

</details>

---

## Part 4: Frequency Hopping (20 min)

### Basic Hop Sequence

```rust
// Define frequencies (900 MHz ISM band)
let frequencies: Vec<u64> = vec![
    902_000_000,
    904_000_000,
    906_000_000,
    908_000_000,
    910_000_000,
];

// Schedule hops every 20ms
let event_ids = scheduler.schedule_hop_sequence(
    scheduler.now_ns(),
    Duration::from_millis(20),
    &frequencies,
);

println!("Scheduled {} frequency hops", event_ids.len());
```

### Hopping with TX

For FHSS systems, you typically:
1. Hop to new frequency
2. TX during the dwell time
3. Hop to next frequency
4. Repeat

```rust
fn schedule_fhss_transmit(
    scheduler: &RealTimeScheduler,
    frequencies: &[u64],
    dwell_time: Duration,
    tx_duration: Duration,
) {
    let start = scheduler.now_ns();
    let dwell_ns = dwell_time.as_nanos() as u64;

    for (i, &freq) in frequencies.iter().enumerate() {
        let hop_time = start + (i as u64 * dwell_ns);

        // Schedule frequency change
        scheduler.schedule(ScheduledEvent::new(
            hop_time,
            EventAction::SetFrequency { hz: freq },
        ).with_priority(0));  // Highest priority for timing-critical

        // Schedule TX after frequency settles (1μs margin)
        let tx_start = hop_time + 1_000;
        scheduler.schedule_tx_burst(tx_start, tx_duration);
    }
}

// Usage: 50ms dwell, 40ms TX per hop
schedule_fhss_transmit(
    &scheduler,
    &frequencies,
    Duration::from_millis(50),
    Duration::from_millis(40),
);
```

### Exercise 4.1: SINCGARS-Style Hopping

SINCGARS hops at 100 hops/second. Implement a function that schedules 10 seconds of SINCGARS-style hopping with TX during each hop:

- Hop interval: 10ms
- TX duration: 8ms (leaves 2ms for turnaround + hop)
- Use a simple pseudo-random frequency sequence

<details>
<summary>Solution</summary>

```rust
fn schedule_sincgars_pattern(
    scheduler: &RealTimeScheduler,
    seed: u64,
    num_hops: usize,
) {
    // Simple PRNG for frequency selection
    let mut rng_state = seed;
    let base_freq = 30_000_000u64;  // 30 MHz
    let num_channels = 2320;         // SINCGARS has 2320 channels
    let channel_spacing = 25_000;    // 25 kHz spacing

    let hop_interval = Duration::from_millis(10);
    let tx_duration = Duration::from_millis(8);
    let start = scheduler.now_ns();

    for hop in 0..num_hops {
        // PRNG: simple LCG
        rng_state = rng_state.wrapping_mul(1103515245).wrapping_add(12345);
        let channel = (rng_state % num_channels as u64) as u64;
        let freq = base_freq + channel * channel_spacing;

        let hop_time = start + (hop as u64 * hop_interval.as_nanos() as u64);

        // Frequency hop
        scheduler.schedule(ScheduledEvent::new(
            hop_time,
            EventAction::SetFrequency { hz: freq },
        ).with_priority(0));

        // TX burst
        scheduler.schedule_tx_burst(hop_time + 1_000, tx_duration);
    }

    println!("Scheduled {} SINCGARS hops over {} seconds",
        num_hops,
        (num_hops as f64 * 0.01));
}

// 10 seconds = 1000 hops
schedule_sincgars_pattern(&scheduler, 12345, 1000);
```

</details>

---

## Part 5: TDMA Frame Structure (20 min)

### What is TDMA?

Time Division Multiple Access divides time into frames, each containing slots:

```
┌─────────────────────── Frame (100ms) ──────────────────────┐
│                                                            │
│  ┌─────────┬─────────┬─────────┬─────────┬─────────┐       │
│  │ Slot 0  │ Slot 1  │ Slot 2  │ Slot 3  │ Slot 4  │       │
│  │ (20ms)  │ (20ms)  │ (20ms)  │ (20ms)  │ (20ms)  │       │
│  │ Node A  │ Node B  │ Node C  │ Node D  │ Node E  │       │
│  │   TX    │   TX    │   TX    │   TX    │   TX    │       │
│  └─────────┴─────────┴─────────┴─────────┴─────────┘       │
│                                                            │
└────────────────────────────────────────────────────────────┘
```

Each node:
- Transmits during its assigned slot
- Receives during all other slots

### Using schedule_tdma_frame

```rust
// 5 slots, 20ms each, we are slot 2
let event_ids = scheduler.schedule_tdma_frame(
    scheduler.now_ns(),
    Duration::from_millis(20),
    2,  // Our TX slot (0-indexed)
    5,  // Total slots
);

println!("TDMA frame scheduled:");
println!("  Slot 0: RX (Node A)");
println!("  Slot 1: RX (Node B)");
println!("  Slot 2: TX (Us!)");
println!("  Slot 3: RX (Node D)");
println!("  Slot 4: RX (Node E)");
```

### Continuous TDMA

For continuous operation, schedule the next frame when the current one completes:

```rust
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

fn run_tdma_continuously(
    scheduler: Arc<RealTimeScheduler>,
    our_slot: usize,
    total_slots: usize,
    slot_duration: Duration,
) {
    let frame_duration = slot_duration * total_slots as u32;
    let frame_ns = frame_duration.as_nanos() as u64;

    // Schedule first frame
    let mut next_frame = scheduler.now_ns();

    loop {
        // Schedule this frame
        scheduler.schedule_tdma_frame(
            next_frame,
            slot_duration,
            our_slot,
            total_slots,
        );

        // Prepare next frame time
        next_frame += frame_ns;

        // Process events
        while scheduler.now_ns() < next_frame {
            scheduler.process();
            std::thread::sleep(Duration::from_micros(100));
        }
    }
}
```

### Exercise 5.1: Adaptive Slot Allocation

Implement a TDMA system where your slot assignment can change:

```rust
struct AdaptiveTdma {
    scheduler: Arc<RealTimeScheduler>,
    current_slot: AtomicUsize,
    total_slots: usize,
    slot_duration: Duration,
}

impl AdaptiveTdma {
    fn change_slot(&self, new_slot: usize) {
        // TODO: Cancel current slot events and reschedule
    }

    fn schedule_next_frame(&self, frame_start: u64) {
        // TODO: Schedule frame with current slot assignment
    }
}
```

<details>
<summary>Solution</summary>

```rust
use std::sync::atomic::{AtomicUsize, Ordering};

struct AdaptiveTdma {
    scheduler: Arc<RealTimeScheduler>,
    current_slot: AtomicUsize,
    total_slots: usize,
    slot_duration: Duration,
}

impl AdaptiveTdma {
    fn new(
        scheduler: Arc<RealTimeScheduler>,
        initial_slot: usize,
        total_slots: usize,
        slot_duration: Duration,
    ) -> Self {
        Self {
            scheduler,
            current_slot: AtomicUsize::new(initial_slot),
            total_slots,
            slot_duration,
        }
    }

    fn change_slot(&self, new_slot: usize) {
        // Cancel any pending TX events from our current slot
        self.scheduler.cancel_from_source("tdma_tx");

        // Update slot assignment
        self.current_slot.store(new_slot, Ordering::SeqCst);

        println!("Slot changed to {}", new_slot);
    }

    fn schedule_next_frame(&self, frame_start: u64) {
        let slot_ns = self.slot_duration.as_nanos() as u64;
        let our_slot = self.current_slot.load(Ordering::SeqCst);

        for slot in 0..self.total_slots {
            let slot_start = frame_start + (slot as u64 * slot_ns);

            if slot == our_slot {
                // Our TX slot
                let event = ScheduledEvent::new(slot_start, EventAction::StartTx)
                    .with_priority(0)
                    .with_guard(|s| s.can_transmit())
                    .with_source("tdma_tx");

                self.scheduler.schedule(event);

                let stop_event = ScheduledEvent::new(
                    slot_start + slot_ns - 1000,  // Small guard time
                    EventAction::StopTx,
                ).with_priority(0).with_source("tdma_tx");

                self.scheduler.schedule(stop_event);
            } else {
                // RX slot
                self.scheduler.schedule_rx_window(slot_start, self.slot_duration);
            }
        }
    }
}
```

</details>

---

## Part 6: Callbacks and Coordination (10 min)

### Using Callbacks

Callbacks let you execute arbitrary code at scheduled times:

```rust
use std::sync::atomic::{AtomicU32, Ordering};

let packet_count = Arc::new(AtomicU32::new(0));
let packet_count_clone = packet_count.clone();

// Schedule callback to run every 100ms
let callback_id = scheduler.register_callback(move |state| {
    let count = packet_count_clone.fetch_add(1, Ordering::SeqCst);
    println!("Callback #{} - State: {:?}", count, state);
});

// Schedule it as repeating event
let event = ScheduledEvent::new(
    scheduler.now_ns() + 100_000_000,
    EventAction::Callback { callback_id },
)
.with_repeat(Duration::from_millis(100));

scheduler.schedule(event);
```

### Coordinating Components

Use callbacks to coordinate between protocol layers:

```rust
struct ProtocolStack {
    scheduler: Arc<RealTimeScheduler>,
    tx_queue: Arc<Mutex<VecDeque<Vec<u8>>>>,
}

impl ProtocolStack {
    fn start(&self) {
        let tx_queue = self.tx_queue.clone();
        let scheduler = self.scheduler.clone();

        // Register TX completion callback
        let callback_id = self.scheduler.register_callback(move |state| {
            if matches!(state, RadioState::TxTurnaround | RadioState::Idle) {
                // TX complete, check for more data
                let mut queue = tx_queue.lock().unwrap();
                if let Some(packet) = queue.pop_front() {
                    println!("Transmitting next packet: {} bytes", packet.len());
                    // Schedule next TX
                    let now = scheduler.now_ns();
                    scheduler.schedule_tx_burst(
                        now + 10_000,  // Small delay
                        Duration::from_millis(5),
                    );
                }
            }
        });

        // Check queue every 10ms
        let event = ScheduledEvent::new(
            self.scheduler.now_ns() + 10_000_000,
            EventAction::Callback { callback_id },
        ).with_repeat(Duration::from_millis(10));

        self.scheduler.schedule(event);
    }
}
```

---

## Summary

In this workshop, you learned:

1. **Radio State Machine**: Prevents invalid TX/RX combinations
2. **Event Scheduling**: Deadlines, priorities, guards
3. **Frequency Hopping**: Precise hop timing for FHSS
4. **TDMA**: Frame-based slot allocation
5. **Callbacks**: Coordinating protocol components

### Key Takeaways

- Use guards to prevent invalid operations
- Priority 0 for timing-critical events (hops, slot boundaries)
- Turnaround delays are automatic after StopTx/StopRx
- Callbacks bridge scheduling with application logic

### Next Steps

- Implement a complete protocol (LoRa, SINCGARS, DMR)
- Integrate with hardware via r4w-fpga
- Add GPS synchronization for multi-node TDMA

---

## Exercises

### Challenge 1: Beacon Protocol

Implement a beacon/listen protocol:
- Beacon TX every 1 second for 50ms
- Listen between beacons
- Track received beacons in a callback

### Challenge 2: ALOHA with Collision Avoidance

Implement slotted ALOHA:
- Random slot selection each frame
- Back off on guard failure (collision detected)
- Exponential backoff on repeated failures

### Challenge 3: Multi-Radio Coordinator

Create a coordinator for 2 radios:
- Prevent simultaneous TX (co-site interference)
- Fair scheduling between radios
- Priority for one radio on time-critical traffic

---

## Reference

### Event Actions

| Action | Effect on State |
|--------|-----------------|
| StartTx | Idle/RxTurnaround → Transmitting |
| StopTx | Transmitting → TxTurnaround → Idle |
| StartRx | Idle/TxTurnaround → Receiving |
| StopRx | Receiving → RxTurnaround → Idle |
| SetFrequency | Brief Hopping state |
| Reset | Any → Idle |

### Priority Levels

| Priority | Use Case |
|----------|----------|
| 0 | Timing-critical (hops, sync) |
| 1 | TX/RX operations |
| 2 | Beacons, periodic |
| 128 | Default |
| 255 | Low priority |

### State Predicates

| Method | Returns true when |
|--------|-------------------|
| `can_transmit()` | Idle or RxTurnaround |
| `can_receive()` | Idle or TxTurnaround |
| `can_hop()` | Idle, TxTurnaround, or RxTurnaround |
