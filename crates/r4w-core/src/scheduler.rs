//! # Discrete Event Tick Scheduler
//!
//! This module provides a tick-based discrete event simulation (DES) framework
//! for coordinating SDR components with controllable time progression.
//!
//! ## Overview
//!
//! The tick scheduler provides:
//! - **Time scaling**: Run faster, slower, or pause simulated time
//! - **Event subscription**: Components receive tick notifications
//! - **Sleep/wake**: Components can request wake-up after N ticks
//! - **Deterministic simulation**: Reproducible timing for testing
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                      TickScheduler                           │
//! │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
//! │  │ TimeScale   │  │ CurrentTick │  │ SleepQueue          │  │
//! │  │ 0.0=paused  │  │ (monotonic) │  │ (wake_tick, id)     │  │
//! │  │ 0.5=slow    │  │             │  │                     │  │
//! │  │ 1.0=real    │  │             │  │                     │  │
//! │  │ 2.0=fast    │  │             │  │                     │  │
//! │  └─────────────┘  └─────────────┘  └─────────────────────┘  │
//! │                           │                                  │
//! │                           ▼                                  │
//! │  ┌──────────────────────────────────────────────────────┐   │
//! │  │              Subscriber Registry                      │   │
//! │  │  [Transmitter, Receiver, Channel, Monitor, ...]      │   │
//! │  └──────────────────────────────────────────────────────┘   │
//! └─────────────────────────────────────────────────────────────┘
//!                             │
//!             ┌───────────────┼───────────────┐
//!             ▼               ▼               ▼
//!      ┌──────────┐    ┌──────────┐    ┌──────────┐
//!      │ on_tick()│    │ on_tick()│    │ on_tick()│
//!      │ TX Comp  │    │ Channel  │    │ RX Comp  │
//!      └──────────┘    └──────────┘    └──────────┘
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::scheduler::{TickScheduler, TickEvent, TickSubscriber, TickResolution};
//! use std::sync::{Arc, Mutex};
//!
//! struct MyComponent {
//!     id: u64,
//!     tick_count: u64,
//! }
//!
//! impl TickSubscriber for MyComponent {
//!     fn on_tick(&mut self, event: TickEvent) {
//!         self.tick_count += 1;
//!         println!("Component {} received tick {}", self.id, event.tick);
//!     }
//!
//!     fn id(&self) -> u64 { self.id }
//! }
//!
//! // Create scheduler with 1ms tick resolution
//! let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));
//!
//! // Register component
//! let component = Arc::new(Mutex::new(MyComponent { id: 1, tick_count: 0 }));
//! scheduler.subscribe(component.clone());
//!
//! // Run for 10 ticks
//! scheduler.step(10);
//!
//! assert_eq!(component.lock().unwrap().tick_count, 10);
//! ```

use crate::timing::{SampleClock, Timestamp, WallClock};
use serde::{Deserialize, Serialize};
use std::collections::{BinaryHeap, HashMap};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::{Duration, Instant};

/// Unique identifier for a tick subscriber component.
pub type ComponentId = u64;

/// Tick event delivered to subscribed components.
#[derive(Debug, Clone)]
pub struct TickEvent {
    /// Current absolute tick number (monotonically increasing).
    pub tick: u64,

    /// Number of ticks elapsed since this component's last event.
    /// First event will have elapsed = 1.
    pub elapsed: u64,

    /// True if this event is a response to a prior sleep request.
    pub sleep_expired: bool,

    /// The component's registered ID (for correlation).
    pub component_id: ComponentId,

    /// Timestamp at this tick (if timing integration is enabled).
    pub timestamp: Option<Timestamp>,
}

/// Trait for components that want to receive tick notifications.
pub trait TickSubscriber: Send {
    /// Called on each tick (or when sleep expires).
    fn on_tick(&mut self, event: TickEvent);

    /// Return the component's unique ID.
    fn id(&self) -> ComponentId;

    /// Optional: called when component is registered.
    fn on_register(&mut self, _scheduler: &TickScheduler) {}

    /// Optional: called when component is unregistered.
    fn on_unregister(&mut self) {}
}

/// Tick resolution - defines what one tick represents.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum TickResolution {
    /// One tick = N samples at the given sample rate.
    Samples { count: u64, sample_rate: f64 },

    /// One tick = one symbol period at the given symbol rate.
    Symbol { symbol_rate: f64, sample_rate: f64 },

    /// One tick = one frame/packet of N samples.
    Frame { samples_per_frame: u64, sample_rate: f64 },

    /// One tick = N microseconds of wall-clock time.
    Microsecond(u64),

    /// One tick = N milliseconds of wall-clock time.
    Millisecond(u64),

    /// Custom duration per tick.
    Custom(Duration),
}

impl TickResolution {
    /// Get the duration of one tick.
    pub fn tick_duration(&self) -> Duration {
        match self {
            TickResolution::Samples { count, sample_rate } => {
                Duration::from_secs_f64(*count as f64 / sample_rate)
            }
            TickResolution::Symbol {
                symbol_rate,
                sample_rate: _,
            } => Duration::from_secs_f64(1.0 / symbol_rate),
            TickResolution::Frame {
                samples_per_frame,
                sample_rate,
            } => Duration::from_secs_f64(*samples_per_frame as f64 / sample_rate),
            TickResolution::Microsecond(us) => Duration::from_micros(*us),
            TickResolution::Millisecond(ms) => Duration::from_millis(*ms),
            TickResolution::Custom(d) => *d,
        }
    }

    /// Get samples per tick (if applicable).
    pub fn samples_per_tick(&self) -> Option<u64> {
        match self {
            TickResolution::Samples { count, .. } => Some(*count),
            TickResolution::Symbol {
                symbol_rate,
                sample_rate,
            } => Some((sample_rate / symbol_rate) as u64),
            TickResolution::Frame {
                samples_per_frame, ..
            } => Some(*samples_per_frame),
            _ => None,
        }
    }
}

impl Default for TickResolution {
    fn default() -> Self {
        TickResolution::Millisecond(1)
    }
}

/// Sleep request in the priority queue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SleepRequest {
    wake_tick: u64,
    component_id: ComponentId,
    request_id: u64,
}

impl Ord for SleepRequest {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Min-heap: smaller wake_tick comes first
        other
            .wake_tick
            .cmp(&self.wake_tick)
            .then_with(|| other.request_id.cmp(&self.request_id))
    }
}

impl PartialOrd for SleepRequest {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

/// Scheduler state for pause/run control.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerState {
    /// Scheduler is stopped.
    Stopped,
    /// Scheduler is paused (can be stepped manually).
    Paused,
    /// Scheduler is running continuously.
    Running,
}

/// Statistics about scheduler operation.
#[derive(Debug, Clone, Default)]
pub struct SchedulerStats {
    /// Total ticks processed.
    pub total_ticks: u64,
    /// Total sleep requests processed.
    pub sleep_requests: u64,
    /// Total events delivered.
    pub events_delivered: u64,
    /// Current number of pending sleep requests.
    pub pending_sleeps: usize,
    /// Current number of subscribers.
    pub subscriber_count: usize,
    /// Average tick processing time.
    pub avg_tick_duration_us: f64,
    /// Maximum tick processing time.
    pub max_tick_duration_us: f64,
}

/// Central tick scheduler for discrete event simulation.
pub struct TickScheduler {
    /// Current tick number (monotonically increasing).
    current_tick: u64,

    /// Time scale factor (0.0 = paused, 1.0 = real-time, 2.0 = 2x speed).
    time_scale: f64,

    /// Tick resolution (what one tick represents).
    resolution: TickResolution,

    /// Scheduler state.
    state: SchedulerState,

    /// Registered subscribers.
    subscribers: Vec<Arc<Mutex<dyn TickSubscriber>>>,

    /// Last tick each component received an event.
    last_tick: HashMap<ComponentId, u64>,

    /// Sleep request queue (min-heap by wake_tick).
    sleep_queue: BinaryHeap<SleepRequest>,

    /// Next sleep request ID (for ordering).
    next_sleep_id: u64,

    /// Sample rate for timing integration.
    sample_rate: Option<f64>,

    /// Start time for wall-clock calculations.
    start_time: Instant,

    /// Start wall clock.
    start_wall: WallClock,

    /// Statistics.
    stats: SchedulerStats,

    /// Accumulator for timing stats.
    tick_times_us: Vec<f64>,
}

impl TickScheduler {
    /// Create a new tick scheduler with the given resolution.
    pub fn new(resolution: TickResolution) -> Self {
        Self {
            current_tick: 0,
            time_scale: 1.0,
            resolution,
            state: SchedulerState::Stopped,
            subscribers: Vec::new(),
            last_tick: HashMap::new(),
            sleep_queue: BinaryHeap::new(),
            next_sleep_id: 0,
            sample_rate: None,
            start_time: Instant::now(),
            start_wall: WallClock::now(),
            stats: SchedulerStats::default(),
            tick_times_us: Vec::with_capacity(1000),
        }
    }

    /// Create a scheduler with sample-based timing.
    pub fn with_sample_rate(sample_rate: f64, samples_per_tick: u64) -> Self {
        let mut scheduler = Self::new(TickResolution::Samples {
            count: samples_per_tick,
            sample_rate,
        });
        scheduler.sample_rate = Some(sample_rate);
        scheduler
    }

    /// Get the current tick number.
    #[inline]
    pub fn current_tick(&self) -> u64 {
        self.current_tick
    }

    /// Get the current time scale.
    #[inline]
    pub fn time_scale(&self) -> f64 {
        self.time_scale
    }

    /// Set the time scale (0.0 = paused, 1.0 = real-time).
    pub fn set_time_scale(&mut self, scale: f64) {
        self.time_scale = scale.max(0.0);
    }

    /// Get the tick resolution.
    pub fn resolution(&self) -> TickResolution {
        self.resolution
    }

    /// Get the current scheduler state.
    pub fn state(&self) -> SchedulerState {
        self.state
    }

    /// Get scheduler statistics.
    pub fn stats(&self) -> SchedulerStats {
        let mut stats = self.stats.clone();
        stats.pending_sleeps = self.sleep_queue.len();
        stats.subscriber_count = self.subscribers.len();
        stats
    }

    /// Subscribe a component to receive tick events.
    pub fn subscribe(&mut self, subscriber: Arc<Mutex<dyn TickSubscriber>>) {
        let id = subscriber.lock().unwrap().id();
        self.last_tick.insert(id, self.current_tick);

        // Call on_register
        subscriber.lock().unwrap().on_register(self);

        self.subscribers.push(subscriber);
        self.stats.subscriber_count = self.subscribers.len();
    }

    /// Unsubscribe a component by ID.
    pub fn unsubscribe(&mut self, component_id: ComponentId) {
        self.subscribers.retain(|s| {
            let id = s.lock().unwrap().id();
            if id == component_id {
                s.lock().unwrap().on_unregister();
                false
            } else {
                true
            }
        });
        self.last_tick.remove(&component_id);
        self.stats.subscriber_count = self.subscribers.len();
    }

    /// Request a wake-up after N ticks for a component.
    ///
    /// When the wake tick is reached, the component will receive a TickEvent
    /// with `sleep_expired = true`.
    pub fn sleep(&mut self, ticks: u64, component_id: ComponentId) {
        let wake_tick = self.current_tick + ticks;
        let request_id = self.next_sleep_id;
        self.next_sleep_id += 1;

        self.sleep_queue.push(SleepRequest {
            wake_tick,
            component_id,
            request_id,
        });

        self.stats.sleep_requests += 1;
    }

    /// Cancel all pending sleep requests for a component.
    pub fn cancel_sleeps(&mut self, component_id: ComponentId) {
        let old_queue = std::mem::take(&mut self.sleep_queue);
        self.sleep_queue = old_queue
            .into_iter()
            .filter(|r| r.component_id != component_id)
            .collect();
    }

    /// Convert a tick number to a Timestamp.
    pub fn tick_to_timestamp(&self, tick: u64) -> Timestamp {
        let tick_duration = self.resolution.tick_duration();
        let elapsed_ns = (tick as f64 * tick_duration.as_nanos() as f64) as u64;

        let sample_rate = self.sample_rate.unwrap_or(48000.0);
        let samples = if let Some(spt) = self.resolution.samples_per_tick() {
            tick * spt
        } else {
            (tick as f64 * tick_duration.as_secs_f64() * sample_rate) as u64
        };

        Timestamp {
            sample: SampleClock::at_sample(samples, sample_rate),
            wall: WallClock::from_nanos(self.start_wall.as_nanos() + elapsed_ns),
            hardware: None,
            synced: None,
        }
    }

    /// Advance by one tick and notify all subscribers.
    pub fn tick(&mut self) {
        let tick_start = Instant::now();

        self.current_tick += 1;
        let current = self.current_tick;

        // Collect sleep expirations for this tick
        let mut expired_sleeps: Vec<ComponentId> = Vec::new();
        while let Some(req) = self.sleep_queue.peek() {
            if req.wake_tick <= current {
                let req = self.sleep_queue.pop().unwrap();
                expired_sleeps.push(req.component_id);
            } else {
                break;
            }
        }

        // Build timestamp for this tick
        let timestamp = if self.sample_rate.is_some() {
            Some(self.tick_to_timestamp(current))
        } else {
            None
        };

        // Notify all subscribers
        for subscriber in &self.subscribers {
            let mut sub = subscriber.lock().unwrap();
            let id = sub.id();

            let last = self.last_tick.get(&id).copied().unwrap_or(0);
            let elapsed = current - last;
            let sleep_expired = expired_sleeps.contains(&id);

            let event = TickEvent {
                tick: current,
                elapsed,
                sleep_expired,
                component_id: id,
                timestamp: timestamp.clone(),
            };

            sub.on_tick(event);
            self.stats.events_delivered += 1;
        }

        // Update last_tick for all subscribers
        for subscriber in &self.subscribers {
            let id = subscriber.lock().unwrap().id();
            self.last_tick.insert(id, current);
        }

        self.stats.total_ticks += 1;

        // Record timing stats
        let tick_duration_us = tick_start.elapsed().as_micros() as f64;
        self.tick_times_us.push(tick_duration_us);
        if self.tick_times_us.len() > 1000 {
            self.tick_times_us.remove(0);
        }
        self.stats.avg_tick_duration_us =
            self.tick_times_us.iter().sum::<f64>() / self.tick_times_us.len() as f64;
        if tick_duration_us > self.stats.max_tick_duration_us {
            self.stats.max_tick_duration_us = tick_duration_us;
        }
    }

    /// Step forward by N ticks (for paused/debug mode).
    pub fn step(&mut self, n: u64) {
        for _ in 0..n {
            self.tick();
        }
    }

    /// Run the scheduler continuously until stopped.
    ///
    /// Respects time_scale for pacing:
    /// - 0.0: Pauses (no ticks generated)
    /// - 1.0: Real-time (one tick per tick_duration)
    /// - 2.0: Double speed (two ticks per tick_duration)
    /// - 0.5: Half speed (one tick per 2*tick_duration)
    pub fn run(&mut self) {
        self.state = SchedulerState::Running;
        let tick_duration = self.resolution.tick_duration();

        while self.state == SchedulerState::Running {
            if self.time_scale == 0.0 {
                // Paused - just sleep briefly and check state
                thread::sleep(Duration::from_millis(10));
                continue;
            }

            let loop_start = Instant::now();

            // Calculate how many ticks to process based on time_scale
            let ticks_to_run = if self.time_scale >= 1.0 {
                self.time_scale.round() as u64
            } else {
                1
            };

            for _ in 0..ticks_to_run {
                self.tick();
            }

            // Sleep to maintain timing (for scales <= 1.0)
            if self.time_scale <= 1.0 && self.time_scale > 0.0 {
                let target_duration = Duration::from_secs_f64(
                    tick_duration.as_secs_f64() / self.time_scale,
                );
                let elapsed = loop_start.elapsed();
                if elapsed < target_duration {
                    thread::sleep(target_duration - elapsed);
                }
            }
        }
    }

    /// Pause the scheduler.
    pub fn pause(&mut self) {
        self.state = SchedulerState::Paused;
    }

    /// Stop the scheduler.
    pub fn stop(&mut self) {
        self.state = SchedulerState::Stopped;
    }

    /// Resume the scheduler from paused state.
    pub fn resume(&mut self) {
        if self.state == SchedulerState::Paused {
            self.state = SchedulerState::Running;
        }
    }

    /// Reset the scheduler to initial state.
    pub fn reset(&mut self) {
        self.current_tick = 0;
        self.sleep_queue.clear();
        self.last_tick.clear();
        self.start_time = Instant::now();
        self.start_wall = WallClock::now();
        self.stats = SchedulerStats::default();
        self.tick_times_us.clear();

        // Re-initialize last_tick for all subscribers
        for subscriber in &self.subscribers {
            let id = subscriber.lock().unwrap().id();
            self.last_tick.insert(id, 0);
        }
    }
}

/// Builder for TickScheduler with fluent configuration.
pub struct TickSchedulerBuilder {
    resolution: TickResolution,
    time_scale: f64,
    sample_rate: Option<f64>,
}

impl TickSchedulerBuilder {
    /// Create a new builder with default settings.
    pub fn new() -> Self {
        Self {
            resolution: TickResolution::default(),
            time_scale: 1.0,
            sample_rate: None,
        }
    }

    /// Set the tick resolution.
    pub fn resolution(mut self, resolution: TickResolution) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set sample-based resolution.
    pub fn samples(mut self, count: u64, sample_rate: f64) -> Self {
        self.resolution = TickResolution::Samples { count, sample_rate };
        self.sample_rate = Some(sample_rate);
        self
    }

    /// Set symbol-based resolution.
    pub fn symbol_rate(mut self, symbol_rate: f64, sample_rate: f64) -> Self {
        self.resolution = TickResolution::Symbol {
            symbol_rate,
            sample_rate,
        };
        self.sample_rate = Some(sample_rate);
        self
    }

    /// Set millisecond resolution.
    pub fn milliseconds(mut self, ms: u64) -> Self {
        self.resolution = TickResolution::Millisecond(ms);
        self
    }

    /// Set the initial time scale.
    pub fn time_scale(mut self, scale: f64) -> Self {
        self.time_scale = scale;
        self
    }

    /// Build the scheduler.
    pub fn build(self) -> TickScheduler {
        let mut scheduler = TickScheduler::new(self.resolution);
        scheduler.set_time_scale(self.time_scale);
        scheduler.sample_rate = self.sample_rate;
        scheduler
    }
}

impl Default for TickSchedulerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// A simple component that counts ticks (for testing/examples).
pub struct TickCounter {
    id: ComponentId,
    count: u64,
    sleep_count: u64,
}

impl TickCounter {
    pub fn new(id: ComponentId) -> Self {
        Self {
            id,
            count: 0,
            sleep_count: 0,
        }
    }

    pub fn count(&self) -> u64 {
        self.count
    }

    pub fn sleep_count(&self) -> u64 {
        self.sleep_count
    }
}

impl TickSubscriber for TickCounter {
    fn on_tick(&mut self, event: TickEvent) {
        self.count += 1;
        if event.sleep_expired {
            self.sleep_count += 1;
        }
    }

    fn id(&self) -> ComponentId {
        self.id
    }
}

/// Thread-safe handle for controlling a running scheduler.
#[derive(Clone)]
pub struct SchedulerHandle {
    state: Arc<RwLock<SchedulerState>>,
    time_scale: Arc<RwLock<f64>>,
}

impl SchedulerHandle {
    /// Pause the scheduler.
    pub fn pause(&self) {
        *self.state.write().unwrap() = SchedulerState::Paused;
    }

    /// Resume the scheduler.
    pub fn resume(&self) {
        let mut state = self.state.write().unwrap();
        if *state == SchedulerState::Paused {
            *state = SchedulerState::Running;
        }
    }

    /// Stop the scheduler.
    pub fn stop(&self) {
        *self.state.write().unwrap() = SchedulerState::Stopped;
    }

    /// Set the time scale.
    pub fn set_time_scale(&self, scale: f64) {
        *self.time_scale.write().unwrap() = scale.max(0.0);
    }

    /// Get the current time scale.
    pub fn time_scale(&self) -> f64 {
        *self.time_scale.read().unwrap()
    }

    /// Check if running.
    pub fn is_running(&self) -> bool {
        *self.state.read().unwrap() == SchedulerState::Running
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tick_resolution_duration() {
        let res = TickResolution::Millisecond(10);
        assert_eq!(res.tick_duration(), Duration::from_millis(10));

        let res = TickResolution::Samples {
            count: 48,
            sample_rate: 48000.0,
        };
        assert_eq!(res.tick_duration(), Duration::from_millis(1));

        let res = TickResolution::Symbol {
            symbol_rate: 1000.0,
            sample_rate: 48000.0,
        };
        assert_eq!(res.tick_duration(), Duration::from_millis(1));
    }

    #[test]
    fn test_scheduler_basic() {
        let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));
        assert_eq!(scheduler.current_tick(), 0);

        scheduler.tick();
        assert_eq!(scheduler.current_tick(), 1);

        scheduler.step(5);
        assert_eq!(scheduler.current_tick(), 6);
    }

    #[test]
    fn test_scheduler_subscriber() {
        let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));
        let counter = Arc::new(Mutex::new(TickCounter::new(1)));

        scheduler.subscribe(counter.clone());
        scheduler.step(10);

        assert_eq!(counter.lock().unwrap().count(), 10);
    }

    #[test]
    fn test_scheduler_sleep() {
        let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));
        let counter = Arc::new(Mutex::new(TickCounter::new(1)));

        scheduler.subscribe(counter.clone());
        scheduler.sleep(5, 1); // Wake after 5 ticks

        scheduler.step(3);
        assert_eq!(counter.lock().unwrap().sleep_count(), 0);

        scheduler.step(3); // Now at tick 6, sleep should have expired at tick 5
        assert_eq!(counter.lock().unwrap().sleep_count(), 1);
    }

    #[test]
    fn test_scheduler_elapsed() {
        let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));

        struct ElapsedChecker {
            id: ComponentId,
            last_elapsed: u64,
        }

        impl TickSubscriber for ElapsedChecker {
            fn on_tick(&mut self, event: TickEvent) {
                self.last_elapsed = event.elapsed;
            }
            fn id(&self) -> ComponentId {
                self.id
            }
        }

        let checker = Arc::new(Mutex::new(ElapsedChecker {
            id: 1,
            last_elapsed: 0,
        }));
        scheduler.subscribe(checker.clone());

        scheduler.tick();
        assert_eq!(checker.lock().unwrap().last_elapsed, 1);

        scheduler.tick();
        assert_eq!(checker.lock().unwrap().last_elapsed, 1);
    }

    #[test]
    fn test_scheduler_builder() {
        let scheduler = TickSchedulerBuilder::new()
            .milliseconds(5)
            .time_scale(2.0)
            .build();

        assert_eq!(scheduler.time_scale(), 2.0);
        assert_eq!(
            scheduler.resolution().tick_duration(),
            Duration::from_millis(5)
        );
    }

    #[test]
    fn test_scheduler_reset() {
        let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));
        scheduler.step(100);
        assert_eq!(scheduler.current_tick(), 100);

        scheduler.reset();
        assert_eq!(scheduler.current_tick(), 0);
    }

    #[test]
    fn test_tick_to_timestamp() {
        let scheduler = TickScheduler::with_sample_rate(48000.0, 48);
        // 48 samples per tick at 48kHz = 1ms per tick

        let ts = scheduler.tick_to_timestamp(1000);
        // 1000 ticks * 48 samples = 48000 samples = 1 second
        assert_eq!(ts.sample.samples(), 48000);
    }

    #[test]
    fn test_scheduler_stats() {
        let mut scheduler = TickScheduler::new(TickResolution::Millisecond(1));
        let counter = Arc::new(Mutex::new(TickCounter::new(1)));
        scheduler.subscribe(counter);

        scheduler.step(100);
        scheduler.sleep(10, 1);

        let stats = scheduler.stats();
        assert_eq!(stats.total_ticks, 100);
        assert_eq!(stats.events_delivered, 100);
        assert_eq!(stats.sleep_requests, 1);
        assert_eq!(stats.subscriber_count, 1);
    }
}
