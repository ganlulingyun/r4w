//! # Real-Time Scheduler
//!
//! A real-time event scheduler for operational SDR systems. Unlike the
//! [`TickScheduler`](crate::scheduler::TickScheduler) which is designed for
//! simulation with virtual time, the `RealTimeScheduler` is built for
//! production systems with wall-clock timing and hardware synchronization.
//!
//! ## Key Features
//!
//! - **Wall-clock timing**: Events fire at precise real-time moments
//! - **TX/RX coordination**: State machine prevents simultaneous TX/RX
//! - **Guard conditions**: Events only fire when conditions are met
//! - **Priority scheduling**: Critical events preempt lower priority ones
//! - **Thread-safe**: Multiple threads can schedule events concurrently
//! - **Hardware clock integration**: Sync with SDR device clocks, GPS/PTP
//!
//! ## Use Cases
//!
//! - TDMA/TDD slot timing in half-duplex systems
//! - Frequency hopping (SINCGARS, HAVEQUICK) with precise hop timing
//! - Beacon/preamble transmission at exact intervals
//! - Multi-radio coordination to prevent co-site interference
//! - Protocol state machines with timeouts and retransmissions
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::rt_scheduler::{
//!     RealTimeScheduler, ScheduledEvent, EventAction, RadioState, ClockSource
//! };
//! use std::time::Duration;
//!
//! // Create scheduler with system clock
//! let scheduler = RealTimeScheduler::builder()
//!     .clock_source(ClockSource::System)
//!     .tx_rx_turnaround(Duration::from_micros(100))
//!     .build()
//!     .unwrap();
//!
//! // Schedule a TX event with guard condition
//! let event = ScheduledEvent::new(
//!     scheduler.now_ns() + 1_000_000,  // 1ms from now
//!     EventAction::StartTx,
//! )
//! .with_priority(0)
//! .with_guard(|state| matches!(state, RadioState::Idle));
//!
//! scheduler.schedule(event);
//! ```

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::atomic::{AtomicU64, AtomicU8, Ordering as AtomicOrdering};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::{Duration, Instant};

use crate::timing::{Timestamp, WallClock};

// ============================================================================
// Clock Sources
// ============================================================================

/// Source of time for the scheduler
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockSource {
    /// System monotonic clock (std::time::Instant)
    System,

    /// High-precision event timer (Linux HPET)
    #[cfg(target_os = "linux")]
    Hpet,

    /// CPU timestamp counter (rdtsc on x86)
    #[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
    Tsc,

    /// GPS-synchronized time source
    Gps,

    /// PTP (IEEE 1588) synchronized time
    Ptp,

    /// Hardware SDR device clock
    HardwareDevice,

    /// Custom clock source (for testing or specialized hardware)
    Custom,
}

impl Default for ClockSource {
    fn default() -> Self {
        ClockSource::System
    }
}

/// Trait for clock source implementations
pub trait Clock: Send + Sync {
    /// Get current time in nanoseconds since an epoch
    fn now_ns(&self) -> u64;

    /// Get the clock source type
    fn source(&self) -> ClockSource;

    /// Check if clock is synchronized (GPS/PTP)
    fn is_synchronized(&self) -> bool {
        false
    }

    /// Get synchronization uncertainty in nanoseconds
    fn uncertainty_ns(&self) -> Option<u64> {
        None
    }
}

/// System clock implementation using std::time::Instant
#[derive(Debug)]
pub struct SystemClock {
    epoch: Instant,
}

impl SystemClock {
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
        }
    }
}

impl Default for SystemClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for SystemClock {
    fn now_ns(&self) -> u64 {
        self.epoch.elapsed().as_nanos() as u64
    }

    fn source(&self) -> ClockSource {
        ClockSource::System
    }
}

/// Mock clock for testing - allows manual time advancement
#[derive(Debug)]
pub struct MockClock {
    current_ns: AtomicU64,
}

impl MockClock {
    pub fn new() -> Self {
        Self {
            current_ns: AtomicU64::new(0),
        }
    }

    /// Advance time by the specified duration
    pub fn advance(&self, duration: Duration) {
        self.current_ns
            .fetch_add(duration.as_nanos() as u64, AtomicOrdering::SeqCst);
    }

    /// Set time to a specific value
    pub fn set_time_ns(&self, ns: u64) {
        self.current_ns.store(ns, AtomicOrdering::SeqCst);
    }
}

impl Default for MockClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for MockClock {
    fn now_ns(&self) -> u64 {
        self.current_ns.load(AtomicOrdering::SeqCst)
    }

    fn source(&self) -> ClockSource {
        ClockSource::Custom
    }
}

// ============================================================================
// Radio State Machine
// ============================================================================

/// Radio operational state for TX/RX coordination
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum RadioState {
    /// Radio is idle, ready for TX or RX
    Idle = 0,

    /// Radio is transmitting
    Transmitting = 1,

    /// TX complete, in turnaround period before RX allowed
    TxTurnaround = 2,

    /// Radio is receiving
    Receiving = 3,

    /// RX complete, in turnaround period before TX allowed
    RxTurnaround = 4,

    /// Radio is changing frequency (FHSS hop)
    Hopping = 5,

    /// Radio is in calibration mode
    Calibrating = 6,

    /// Radio has encountered an error
    Error = 7,
}

impl RadioState {
    /// Check if TX is allowed in this state
    pub fn can_transmit(&self) -> bool {
        matches!(self, RadioState::Idle | RadioState::RxTurnaround)
    }

    /// Check if RX is allowed in this state
    pub fn can_receive(&self) -> bool {
        matches!(self, RadioState::Idle | RadioState::TxTurnaround)
    }

    /// Check if frequency change is allowed in this state
    pub fn can_hop(&self) -> bool {
        matches!(self, RadioState::Idle | RadioState::TxTurnaround | RadioState::RxTurnaround)
    }

    /// Get valid transitions from this state
    pub fn valid_transitions(&self) -> &'static [RadioState] {
        match self {
            RadioState::Idle => &[
                RadioState::Transmitting,
                RadioState::Receiving,
                RadioState::Hopping,
                RadioState::Calibrating,
            ],
            RadioState::Transmitting => &[RadioState::TxTurnaround, RadioState::Error],
            RadioState::TxTurnaround => &[
                RadioState::Idle,
                RadioState::Receiving,
                RadioState::Hopping,
            ],
            RadioState::Receiving => &[RadioState::RxTurnaround, RadioState::Error],
            RadioState::RxTurnaround => &[
                RadioState::Idle,
                RadioState::Transmitting,
                RadioState::Hopping,
            ],
            RadioState::Hopping => &[RadioState::Idle, RadioState::Error],
            RadioState::Calibrating => &[RadioState::Idle, RadioState::Error],
            RadioState::Error => &[RadioState::Idle],
        }
    }

    /// Check if transition to target state is valid
    pub fn can_transition_to(&self, target: RadioState) -> bool {
        self.valid_transitions().contains(&target)
    }
}

impl From<u8> for RadioState {
    fn from(value: u8) -> Self {
        match value {
            0 => RadioState::Idle,
            1 => RadioState::Transmitting,
            2 => RadioState::TxTurnaround,
            3 => RadioState::Receiving,
            4 => RadioState::RxTurnaround,
            5 => RadioState::Hopping,
            6 => RadioState::Calibrating,
            _ => RadioState::Error,
        }
    }
}

/// Atomic radio state for lock-free state transitions
#[derive(Debug)]
pub struct AtomicRadioState {
    state: AtomicU8,
}

impl AtomicRadioState {
    pub fn new(initial: RadioState) -> Self {
        Self {
            state: AtomicU8::new(initial as u8),
        }
    }

    /// Get current state
    pub fn get(&self) -> RadioState {
        RadioState::from(self.state.load(AtomicOrdering::SeqCst))
    }

    /// Set state unconditionally
    pub fn set(&self, state: RadioState) {
        self.state.store(state as u8, AtomicOrdering::SeqCst);
    }

    /// Attempt state transition, returns Ok if successful
    pub fn transition(&self, from: RadioState, to: RadioState) -> Result<(), RadioStateError> {
        if !from.can_transition_to(to) {
            return Err(RadioStateError::InvalidTransition { from, to });
        }

        match self.state.compare_exchange(
            from as u8,
            to as u8,
            AtomicOrdering::SeqCst,
            AtomicOrdering::SeqCst,
        ) {
            Ok(_) => Ok(()),
            Err(actual) => Err(RadioStateError::StateChanged {
                expected: from,
                actual: RadioState::from(actual),
            }),
        }
    }

    /// Try to transition if current state matches predicate
    pub fn transition_if<F>(&self, to: RadioState, predicate: F) -> Result<RadioState, RadioStateError>
    where
        F: Fn(RadioState) -> bool,
    {
        let current = self.get();
        if !predicate(current) {
            return Err(RadioStateError::GuardFailed { current, target: to });
        }
        self.transition(current, to)?;
        Ok(current)
    }
}

impl Default for AtomicRadioState {
    fn default() -> Self {
        Self::new(RadioState::Idle)
    }
}

/// Errors from radio state transitions
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RadioStateError {
    /// Attempted invalid state transition
    InvalidTransition { from: RadioState, to: RadioState },

    /// State changed between check and transition (race condition)
    StateChanged { expected: RadioState, actual: RadioState },

    /// Guard condition failed
    GuardFailed { current: RadioState, target: RadioState },
}

impl std::fmt::Display for RadioStateError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RadioStateError::InvalidTransition { from, to } => {
                write!(f, "Invalid transition from {:?} to {:?}", from, to)
            }
            RadioStateError::StateChanged { expected, actual } => {
                write!(
                    f,
                    "State changed: expected {:?}, found {:?}",
                    expected, actual
                )
            }
            RadioStateError::GuardFailed { current, target } => {
                write!(
                    f,
                    "Guard condition failed: cannot transition from {:?} to {:?}",
                    current, target
                )
            }
        }
    }
}

impl std::error::Error for RadioStateError {}

// ============================================================================
// Events and Actions
// ============================================================================

/// Actions that can be scheduled
#[derive(Debug, Clone)]
pub enum EventAction {
    /// Start transmitting
    StartTx,

    /// Stop transmitting, enter turnaround
    StopTx,

    /// Start receiving
    StartRx,

    /// Stop receiving
    StopRx,

    /// Change frequency (for FHSS)
    SetFrequency { hz: u64 },

    /// Set transmit power
    SetPower { dbm: f32 },

    /// Enter calibration mode
    Calibrate,

    /// Reset to idle state
    Reset,

    /// Custom action with identifier
    Custom { id: u32, data: Vec<u8> },

    /// State transition request
    StateTransition { target: RadioState },

    /// Trigger a callback by ID (callbacks stored separately for thread safety)
    Callback { callback_id: u64 },
}

/// Type alias for guard condition function
pub type GuardFn = Arc<dyn Fn(&RadioState) -> bool + Send + Sync>;

/// A scheduled event with deadline, priority, and optional guard
#[derive(Clone)]
pub struct ScheduledEvent {
    /// Unique event ID
    pub id: u64,

    /// Absolute deadline in nanoseconds
    pub deadline_ns: u64,

    /// Priority (lower number = higher priority)
    pub priority: u8,

    /// The action to execute
    pub action: EventAction,

    /// Optional guard condition
    guard: Option<GuardFn>,

    /// Whether this event repeats
    pub repeat_interval_ns: Option<u64>,

    /// Component/subsystem that scheduled this event
    pub source: Option<String>,
}

impl ScheduledEvent {
    /// Create a new scheduled event
    pub fn new(deadline_ns: u64, action: EventAction) -> Self {
        static EVENT_COUNTER: AtomicU64 = AtomicU64::new(0);

        Self {
            id: EVENT_COUNTER.fetch_add(1, AtomicOrdering::SeqCst),
            deadline_ns,
            priority: 128, // Default middle priority
            action,
            guard: None,
            repeat_interval_ns: None,
            source: None,
        }
    }

    /// Set priority (0 = highest, 255 = lowest)
    pub fn with_priority(mut self, priority: u8) -> Self {
        self.priority = priority;
        self
    }

    /// Add a guard condition
    pub fn with_guard<F>(mut self, guard: F) -> Self
    where
        F: Fn(&RadioState) -> bool + Send + Sync + 'static,
    {
        self.guard = Some(Arc::new(guard));
        self
    }

    /// Make this a repeating event
    pub fn with_repeat(mut self, interval: Duration) -> Self {
        self.repeat_interval_ns = Some(interval.as_nanos() as u64);
        self
    }

    /// Set the source identifier
    pub fn with_source(mut self, source: impl Into<String>) -> Self {
        self.source = Some(source.into());
        self
    }

    /// Check if guard condition passes
    pub fn check_guard(&self, state: &RadioState) -> bool {
        match &self.guard {
            Some(guard) => guard(state),
            None => true,
        }
    }

    /// Create next occurrence for repeating events
    pub fn next_occurrence(&self) -> Option<Self> {
        self.repeat_interval_ns.map(|interval| {
            let mut next = self.clone();
            next.deadline_ns += interval;
            static EVENT_COUNTER: AtomicU64 = AtomicU64::new(0);
            next.id = EVENT_COUNTER.fetch_add(1, AtomicOrdering::SeqCst);
            next
        })
    }
}

impl std::fmt::Debug for ScheduledEvent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ScheduledEvent")
            .field("id", &self.id)
            .field("deadline_ns", &self.deadline_ns)
            .field("priority", &self.priority)
            .field("action", &self.action)
            .field("has_guard", &self.guard.is_some())
            .field("repeat_interval_ns", &self.repeat_interval_ns)
            .field("source", &self.source)
            .finish()
    }
}

// Ordering for priority queue (earliest deadline, then highest priority)
impl PartialEq for ScheduledEvent {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}

impl Eq for ScheduledEvent {}

impl PartialOrd for ScheduledEvent {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ScheduledEvent {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap behavior (earliest deadline first)
        match other.deadline_ns.cmp(&self.deadline_ns) {
            Ordering::Equal => other.priority.cmp(&self.priority),
            other_cmp => other_cmp,
        }
    }
}

// ============================================================================
// Event Handler Trait
// ============================================================================

/// Trait for handling scheduled events
pub trait EventHandler: Send + Sync {
    /// Handle an event, returns true if handled successfully
    fn handle(&self, event: &ScheduledEvent, state: &RadioState) -> Result<(), EventError>;

    /// Get handler name for logging
    fn name(&self) -> &str {
        "default"
    }
}

/// Errors from event handling
#[derive(Debug, Clone)]
pub enum EventError {
    /// Event was not handled
    NotHandled,

    /// Handler encountered an error
    HandlerError(String),

    /// Event deadline was missed
    DeadlineMissed { deadline_ns: u64, actual_ns: u64 },

    /// Guard condition prevented execution
    GuardBlocked,

    /// State transition failed
    StateError(RadioStateError),
}

impl std::fmt::Display for EventError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EventError::NotHandled => write!(f, "Event not handled"),
            EventError::HandlerError(msg) => write!(f, "Handler error: {}", msg),
            EventError::DeadlineMissed { deadline_ns, actual_ns } => {
                write!(
                    f,
                    "Deadline missed: expected {}ns, actual {}ns (late by {}ns)",
                    deadline_ns,
                    actual_ns,
                    actual_ns - deadline_ns
                )
            }
            EventError::GuardBlocked => write!(f, "Guard condition blocked event"),
            EventError::StateError(e) => write!(f, "State error: {}", e),
        }
    }
}

impl std::error::Error for EventError {}

impl From<RadioStateError> for EventError {
    fn from(e: RadioStateError) -> Self {
        EventError::StateError(e)
    }
}

// ============================================================================
// Scheduler Statistics
// ============================================================================

/// Statistics about scheduler performance
#[derive(Debug, Clone, Default)]
pub struct SchedulerStats {
    /// Total events scheduled
    pub events_scheduled: u64,

    /// Total events executed
    pub events_executed: u64,

    /// Events that missed their deadline
    pub deadlines_missed: u64,

    /// Events blocked by guard conditions
    pub guards_blocked: u64,

    /// Maximum observed latency (ns)
    pub max_latency_ns: u64,

    /// Average latency (ns)
    pub avg_latency_ns: u64,

    /// State transitions performed
    pub state_transitions: u64,

    /// Failed state transitions
    pub transition_failures: u64,
}

// ============================================================================
// Real-Time Scheduler
// ============================================================================

/// Configuration for the real-time scheduler
#[derive(Debug, Clone)]
pub struct SchedulerConfig {
    /// Clock source to use
    pub clock_source: ClockSource,

    /// TX to RX turnaround time
    pub tx_rx_turnaround: Duration,

    /// RX to TX turnaround time
    pub rx_tx_turnaround: Duration,

    /// Maximum acceptable lateness before deadline is considered missed
    pub deadline_tolerance_ns: u64,

    /// Enable deadline miss warnings
    pub warn_on_deadline_miss: bool,

    /// Scheduler thread priority (if running dedicated thread)
    pub thread_priority: Option<i32>,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            clock_source: ClockSource::System,
            tx_rx_turnaround: Duration::from_micros(100),
            rx_tx_turnaround: Duration::from_micros(100),
            deadline_tolerance_ns: 1_000_000, // 1ms tolerance
            warn_on_deadline_miss: true,
            thread_priority: None,
        }
    }
}

/// Builder for RealTimeScheduler
pub struct RealTimeSchedulerBuilder {
    config: SchedulerConfig,
    clock: Option<Arc<dyn Clock>>,
    handlers: Vec<Arc<dyn EventHandler>>,
}

impl RealTimeSchedulerBuilder {
    pub fn new() -> Self {
        Self {
            config: SchedulerConfig::default(),
            clock: None,
            handlers: Vec::new(),
        }
    }

    /// Set the clock source
    pub fn clock_source(mut self, source: ClockSource) -> Self {
        self.config.clock_source = source;
        self
    }

    /// Set a custom clock implementation
    pub fn custom_clock(mut self, clock: Arc<dyn Clock>) -> Self {
        self.clock = Some(clock);
        self
    }

    /// Set TX to RX turnaround time
    pub fn tx_rx_turnaround(mut self, duration: Duration) -> Self {
        self.config.tx_rx_turnaround = duration;
        self
    }

    /// Set RX to TX turnaround time
    pub fn rx_tx_turnaround(mut self, duration: Duration) -> Self {
        self.config.rx_tx_turnaround = duration;
        self
    }

    /// Set deadline tolerance
    pub fn deadline_tolerance(mut self, tolerance: Duration) -> Self {
        self.config.deadline_tolerance_ns = tolerance.as_nanos() as u64;
        self
    }

    /// Add an event handler
    pub fn add_handler(mut self, handler: Arc<dyn EventHandler>) -> Self {
        self.handlers.push(handler);
        self
    }

    /// Set thread priority for scheduler thread
    pub fn thread_priority(mut self, priority: i32) -> Self {
        self.config.thread_priority = Some(priority);
        self
    }

    /// Build the scheduler
    pub fn build(self) -> Result<RealTimeScheduler, SchedulerError> {
        let clock: Arc<dyn Clock> = self.clock.unwrap_or_else(|| {
            match self.config.clock_source {
                ClockSource::System => Arc::new(SystemClock::new()),
                // Other clock sources would be implemented here
                _ => Arc::new(SystemClock::new()),
            }
        });

        Ok(RealTimeScheduler {
            config: self.config,
            clock,
            state: Arc::new(AtomicRadioState::default()),
            events: Arc::new(Mutex::new(BinaryHeap::new())),
            handlers: Arc::new(RwLock::new(self.handlers)),
            callbacks: Arc::new(RwLock::new(std::collections::HashMap::new())),
            stats: Arc::new(RwLock::new(SchedulerStats::default())),
            running: Arc::new(std::sync::atomic::AtomicBool::new(false)),
        })
    }
}

impl Default for RealTimeSchedulerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Errors from the scheduler
#[derive(Debug, Clone)]
pub enum SchedulerError {
    /// Scheduler is not running
    NotRunning,

    /// Scheduler is already running
    AlreadyRunning,

    /// Event queue error
    QueueError(String),

    /// Clock error
    ClockError(String),

    /// Configuration error
    ConfigError(String),
}

impl std::fmt::Display for SchedulerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SchedulerError::NotRunning => write!(f, "Scheduler is not running"),
            SchedulerError::AlreadyRunning => write!(f, "Scheduler is already running"),
            SchedulerError::QueueError(msg) => write!(f, "Queue error: {}", msg),
            SchedulerError::ClockError(msg) => write!(f, "Clock error: {}", msg),
            SchedulerError::ConfigError(msg) => write!(f, "Config error: {}", msg),
        }
    }
}

impl std::error::Error for SchedulerError {}

/// Callback function type
pub type CallbackFn = Box<dyn FnMut(&RadioState) + Send>;

/// Real-time event scheduler for operational SDR systems
pub struct RealTimeScheduler {
    config: SchedulerConfig,
    clock: Arc<dyn Clock>,
    state: Arc<AtomicRadioState>,
    events: Arc<Mutex<BinaryHeap<ScheduledEvent>>>,
    handlers: Arc<RwLock<Vec<Arc<dyn EventHandler>>>>,
    callbacks: Arc<RwLock<std::collections::HashMap<u64, CallbackFn>>>,
    stats: Arc<RwLock<SchedulerStats>>,
    running: Arc<std::sync::atomic::AtomicBool>,
}

impl RealTimeScheduler {
    /// Create a new builder
    pub fn builder() -> RealTimeSchedulerBuilder {
        RealTimeSchedulerBuilder::new()
    }

    /// Get current time in nanoseconds
    pub fn now_ns(&self) -> u64 {
        self.clock.now_ns()
    }

    /// Get current radio state
    pub fn state(&self) -> RadioState {
        self.state.get()
    }

    /// Get reference to atomic state for direct manipulation
    pub fn atomic_state(&self) -> &AtomicRadioState {
        &self.state
    }

    /// Schedule an event
    pub fn schedule(&self, event: ScheduledEvent) {
        let mut events = self.events.lock().unwrap();
        events.push(event);

        if let Ok(mut stats) = self.stats.write() {
            stats.events_scheduled += 1;
        }
    }

    /// Schedule multiple events atomically
    pub fn schedule_batch(&self, batch: Vec<ScheduledEvent>) {
        let mut events = self.events.lock().unwrap();
        let count = batch.len() as u64;
        for event in batch {
            events.push(event);
        }

        if let Ok(mut stats) = self.stats.write() {
            stats.events_scheduled += count;
        }
    }

    /// Cancel an event by ID
    pub fn cancel(&self, event_id: u64) -> bool {
        let mut events = self.events.lock().unwrap();
        let original_len = events.len();

        // Rebuild heap without the cancelled event
        let remaining: Vec<_> = events.drain().filter(|e| e.id != event_id).collect();
        *events = remaining.into_iter().collect();

        events.len() < original_len
    }

    /// Cancel all events from a specific source
    pub fn cancel_from_source(&self, source: &str) -> usize {
        let mut events = self.events.lock().unwrap();
        let original_len = events.len();

        let remaining: Vec<_> = events
            .drain()
            .filter(|e| e.source.as_deref() != Some(source))
            .collect();
        *events = remaining.into_iter().collect();

        original_len - events.len()
    }

    /// Register a callback function
    pub fn register_callback<F>(&self, callback: F) -> u64
    where
        F: FnMut(&RadioState) + Send + 'static,
    {
        static CALLBACK_COUNTER: AtomicU64 = AtomicU64::new(0);
        let id = CALLBACK_COUNTER.fetch_add(1, AtomicOrdering::SeqCst);

        if let Ok(mut callbacks) = self.callbacks.write() {
            callbacks.insert(id, Box::new(callback));
        }

        id
    }

    /// Schedule a callback to run at a specific time
    pub fn schedule_callback<F>(&self, deadline_ns: u64, callback: F) -> u64
    where
        F: FnMut(&RadioState) + Send + 'static,
    {
        let callback_id = self.register_callback(callback);
        let event = ScheduledEvent::new(deadline_ns, EventAction::Callback { callback_id });
        self.schedule(event);
        callback_id
    }

    /// Add an event handler
    pub fn add_handler(&self, handler: Arc<dyn EventHandler>) {
        if let Ok(mut handlers) = self.handlers.write() {
            handlers.push(handler);
        }
    }

    /// Process pending events up to the current time
    pub fn process(&self) -> Vec<Result<ScheduledEvent, EventError>> {
        let now = self.now_ns();
        let mut results = Vec::new();
        let mut to_reschedule = Vec::new();

        loop {
            let event = {
                let mut events = self.events.lock().unwrap();
                if events.peek().map_or(true, |e| e.deadline_ns > now) {
                    break;
                }
                events.pop()
            };

            if let Some(event) = event {
                let result = self.execute_event(&event, now);

                // Handle repeating events
                if result.is_ok() {
                    if let Some(next) = event.next_occurrence() {
                        to_reschedule.push(next);
                    }
                }

                results.push(result.map(|_| event));
            }
        }

        // Reschedule repeating events
        if !to_reschedule.is_empty() {
            self.schedule_batch(to_reschedule);
        }

        results
    }

    /// Execute a single event
    fn execute_event(&self, event: &ScheduledEvent, now: u64) -> Result<(), EventError> {
        let current_state = self.state.get();

        // Check guard condition
        if !event.check_guard(&current_state) {
            if let Ok(mut stats) = self.stats.write() {
                stats.guards_blocked += 1;
            }
            return Err(EventError::GuardBlocked);
        }

        // Check deadline
        let latency = now.saturating_sub(event.deadline_ns);
        if latency > self.config.deadline_tolerance_ns {
            if let Ok(mut stats) = self.stats.write() {
                stats.deadlines_missed += 1;
            }
            if self.config.warn_on_deadline_miss {
                eprintln!(
                    "Warning: Event {} missed deadline by {}ns",
                    event.id, latency
                );
            }
            return Err(EventError::DeadlineMissed {
                deadline_ns: event.deadline_ns,
                actual_ns: now,
            });
        }

        // Update latency stats
        if let Ok(mut stats) = self.stats.write() {
            stats.events_executed += 1;
            stats.max_latency_ns = stats.max_latency_ns.max(latency);
            // Rolling average
            stats.avg_latency_ns =
                (stats.avg_latency_ns * (stats.events_executed - 1) + latency) / stats.events_executed;
        }

        // Execute action
        self.execute_action(&event.action)?;

        // Call handlers
        if let Ok(handlers) = self.handlers.read() {
            for handler in handlers.iter() {
                if let Err(e) = handler.handle(event, &current_state) {
                    eprintln!("Handler {} error: {}", handler.name(), e);
                }
            }
        }

        Ok(())
    }

    /// Execute an event action
    fn execute_action(&self, action: &EventAction) -> Result<(), EventError> {
        match action {
            EventAction::StartTx => {
                self.state
                    .transition_if(RadioState::Transmitting, |s| s.can_transmit())?;
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }
            }
            EventAction::StopTx => {
                self.state
                    .transition(RadioState::Transmitting, RadioState::TxTurnaround)?;
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }

                // Schedule automatic transition to Idle after turnaround
                let turnaround_ns = self.config.tx_rx_turnaround.as_nanos() as u64;
                let event = ScheduledEvent::new(
                    self.now_ns() + turnaround_ns,
                    EventAction::StateTransition {
                        target: RadioState::Idle,
                    },
                )
                .with_priority(0);
                self.schedule(event);
            }
            EventAction::StartRx => {
                self.state
                    .transition_if(RadioState::Receiving, |s| s.can_receive())?;
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }
            }
            EventAction::StopRx => {
                self.state
                    .transition(RadioState::Receiving, RadioState::RxTurnaround)?;
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }

                // Schedule automatic transition to Idle after turnaround
                let turnaround_ns = self.config.rx_tx_turnaround.as_nanos() as u64;
                let event = ScheduledEvent::new(
                    self.now_ns() + turnaround_ns,
                    EventAction::StateTransition {
                        target: RadioState::Idle,
                    },
                )
                .with_priority(0);
                self.schedule(event);
            }
            EventAction::SetFrequency { hz: _ } => {
                let current = self.state.get();
                if current.can_hop() {
                    self.state.transition(current, RadioState::Hopping)?;
                    // Immediately transition back to idle (frequency change is fast)
                    self.state.set(RadioState::Idle);
                    if let Ok(mut stats) = self.stats.write() {
                        stats.state_transitions += 2;
                    }
                }
            }
            EventAction::StateTransition { target } => {
                let current = self.state.get();
                self.state.transition(current, *target)?;
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }
            }
            EventAction::Callback { callback_id } => {
                let current_state = self.state.get();
                if let Ok(mut callbacks) = self.callbacks.write() {
                    if let Some(callback) = callbacks.get_mut(callback_id) {
                        callback(&current_state);
                    }
                }
            }
            EventAction::Reset => {
                self.state.set(RadioState::Idle);
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }
            }
            EventAction::Calibrate => {
                self.state
                    .transition_if(RadioState::Calibrating, |s| matches!(s, RadioState::Idle))?;
                if let Ok(mut stats) = self.stats.write() {
                    stats.state_transitions += 1;
                }
            }
            EventAction::SetPower { dbm: _ } => {
                // Power change doesn't affect state
            }
            EventAction::Custom { id: _, data: _ } => {
                // Custom actions handled by event handlers
            }
        }

        Ok(())
    }

    /// Get scheduler statistics
    pub fn stats(&self) -> SchedulerStats {
        self.stats.read().map(|s| s.clone()).unwrap_or_default()
    }

    /// Reset statistics
    pub fn reset_stats(&self) {
        if let Ok(mut stats) = self.stats.write() {
            *stats = SchedulerStats::default();
        }
    }

    /// Get number of pending events
    pub fn pending_count(&self) -> usize {
        self.events.lock().map(|e| e.len()).unwrap_or(0)
    }

    /// Get the deadline of the next event (if any)
    pub fn next_deadline(&self) -> Option<u64> {
        self.events
            .lock()
            .ok()
            .and_then(|e| e.peek().map(|ev| ev.deadline_ns))
    }

    /// Run the scheduler in a loop until stopped
    pub fn run(&self) -> Result<(), SchedulerError> {
        if self
            .running
            .swap(true, std::sync::atomic::Ordering::SeqCst)
        {
            return Err(SchedulerError::AlreadyRunning);
        }

        while self.running.load(std::sync::atomic::Ordering::SeqCst) {
            self.process();

            // Sleep until next event or 1ms, whichever is sooner
            let sleep_duration = self
                .next_deadline()
                .map(|deadline| {
                    let now = self.now_ns();
                    if deadline > now {
                        Duration::from_nanos(deadline - now).min(Duration::from_millis(1))
                    } else {
                        Duration::ZERO
                    }
                })
                .unwrap_or(Duration::from_millis(1));

            if !sleep_duration.is_zero() {
                thread::sleep(sleep_duration);
            }
        }

        Ok(())
    }

    /// Stop the scheduler loop
    pub fn stop(&self) {
        self.running
            .store(false, std::sync::atomic::Ordering::SeqCst);
    }

    /// Check if scheduler is running
    pub fn is_running(&self) -> bool {
        self.running.load(std::sync::atomic::Ordering::SeqCst)
    }

    /// Create a wall clock from the current scheduler time
    pub fn wall_clock(&self) -> WallClock {
        WallClock::from_nanos(self.now_ns())
    }

    /// Create a timestamp from the current scheduler time (requires sample rate)
    pub fn timestamp(&self, sample_rate: f64) -> Timestamp {
        let now = self.now_ns();
        let samples = (now as f64 * sample_rate / 1_000_000_000.0) as u64;
        Timestamp::at_sample(samples, sample_rate)
    }

    /// Get the configuration
    pub fn config(&self) -> &SchedulerConfig {
        &self.config
    }
}

// ============================================================================
// Convenience Functions for Common Patterns
// ============================================================================

impl RealTimeScheduler {
    /// Schedule a TX burst with automatic stop
    pub fn schedule_tx_burst(&self, start_ns: u64, duration: Duration) -> (u64, u64) {
        let start_event = ScheduledEvent::new(start_ns, EventAction::StartTx)
            .with_priority(0)
            .with_guard(|s| s.can_transmit())
            .with_source("tx_burst");

        let stop_event = ScheduledEvent::new(
            start_ns + duration.as_nanos() as u64,
            EventAction::StopTx,
        )
        .with_priority(0)
        .with_source("tx_burst");

        let start_id = start_event.id;
        let stop_id = stop_event.id;

        self.schedule(start_event);
        self.schedule(stop_event);

        (start_id, stop_id)
    }

    /// Schedule an RX window
    pub fn schedule_rx_window(&self, start_ns: u64, duration: Duration) -> (u64, u64) {
        let start_event = ScheduledEvent::new(start_ns, EventAction::StartRx)
            .with_priority(1)
            .with_guard(|s| s.can_receive())
            .with_source("rx_window");

        let stop_event = ScheduledEvent::new(
            start_ns + duration.as_nanos() as u64,
            EventAction::StopRx,
        )
        .with_priority(1)
        .with_source("rx_window");

        let start_id = start_event.id;
        let stop_id = stop_event.id;

        self.schedule(start_event);
        self.schedule(stop_event);

        (start_id, stop_id)
    }

    /// Schedule a frequency hop sequence
    pub fn schedule_hop_sequence(
        &self,
        start_ns: u64,
        hop_interval: Duration,
        frequencies: &[u64],
    ) -> Vec<u64> {
        let interval_ns = hop_interval.as_nanos() as u64;
        let mut event_ids = Vec::with_capacity(frequencies.len());

        for (i, &freq_hz) in frequencies.iter().enumerate() {
            let event = ScheduledEvent::new(
                start_ns + (i as u64 * interval_ns),
                EventAction::SetFrequency { hz: freq_hz },
            )
            .with_priority(0) // Highest priority for timing-critical hops
            .with_source("fhss");

            event_ids.push(event.id);
            self.schedule(event);
        }

        event_ids
    }

    /// Schedule a periodic beacon
    pub fn schedule_beacon(&self, start_ns: u64, interval: Duration, tx_duration: Duration) -> u64 {
        let event = ScheduledEvent::new(start_ns, EventAction::StartTx)
            .with_priority(2)
            .with_guard(|s| s.can_transmit())
            .with_repeat(interval)
            .with_source("beacon");

        let id = event.id;
        self.schedule(event);

        // Schedule the stop event (also repeating)
        let stop_event = ScheduledEvent::new(
            start_ns + tx_duration.as_nanos() as u64,
            EventAction::StopTx,
        )
        .with_priority(2)
        .with_repeat(interval)
        .with_source("beacon");

        self.schedule(stop_event);

        id
    }

    /// Schedule TDMA slots
    pub fn schedule_tdma_frame(
        &self,
        frame_start_ns: u64,
        slot_duration: Duration,
        tx_slot: usize,
        total_slots: usize,
    ) -> Vec<u64> {
        let slot_ns = slot_duration.as_nanos() as u64;
        let mut event_ids = Vec::new();

        for slot in 0..total_slots {
            let slot_start = frame_start_ns + (slot as u64 * slot_ns);

            if slot == tx_slot {
                // Our TX slot
                let (start_id, _) = self.schedule_tx_burst(slot_start, slot_duration);
                event_ids.push(start_id);
            } else {
                // RX slot
                let (start_id, _) = self.schedule_rx_window(slot_start, slot_duration);
                event_ids.push(start_id);
            }
        }

        event_ids
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_radio_state_transitions() {
        let state = AtomicRadioState::new(RadioState::Idle);

        // Valid transition
        assert!(state.transition(RadioState::Idle, RadioState::Transmitting).is_ok());
        assert_eq!(state.get(), RadioState::Transmitting);

        // Invalid transition (can't go directly from TX to RX)
        assert!(state
            .transition(RadioState::Transmitting, RadioState::Receiving)
            .is_err());

        // Valid: TX -> TxTurnaround
        assert!(state
            .transition(RadioState::Transmitting, RadioState::TxTurnaround)
            .is_ok());

        // Valid: TxTurnaround -> Receiving
        assert!(state
            .transition(RadioState::TxTurnaround, RadioState::Receiving)
            .is_ok());
    }

    #[test]
    fn test_mock_clock() {
        let clock = MockClock::new();
        assert_eq!(clock.now_ns(), 0);

        clock.advance(Duration::from_millis(100));
        assert_eq!(clock.now_ns(), 100_000_000);

        clock.set_time_ns(500_000_000);
        assert_eq!(clock.now_ns(), 500_000_000);
    }

    #[test]
    fn test_scheduler_basic() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .build()
            .unwrap();

        // Schedule event at 100ms
        let event = ScheduledEvent::new(100_000_000, EventAction::StartTx)
            .with_guard(|s| matches!(s, RadioState::Idle));

        scheduler.schedule(event);
        assert_eq!(scheduler.pending_count(), 1);

        // Process at t=0 - nothing should happen
        let results = scheduler.process();
        assert!(results.is_empty());
        assert_eq!(scheduler.state(), RadioState::Idle);

        // Advance to t=100ms and process
        clock.set_time_ns(100_000_000);
        let results = scheduler.process();
        assert_eq!(results.len(), 1);
        assert!(results[0].is_ok());
        assert_eq!(scheduler.state(), RadioState::Transmitting);
    }

    #[test]
    fn test_guard_condition() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .build()
            .unwrap();

        // Put in TX state
        scheduler.atomic_state().set(RadioState::Transmitting);

        // Try to schedule TX (should fail guard)
        let event = ScheduledEvent::new(0, EventAction::StartTx)
            .with_guard(|s| s.can_transmit());

        scheduler.schedule(event);
        let results = scheduler.process();

        assert_eq!(results.len(), 1);
        assert!(matches!(results[0], Err(EventError::GuardBlocked)));
    }

    #[test]
    fn test_tx_rx_turnaround() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .tx_rx_turnaround(Duration::from_micros(100))
            .build()
            .unwrap();

        // Schedule TX stop
        scheduler.atomic_state().set(RadioState::Transmitting);
        let event = ScheduledEvent::new(0, EventAction::StopTx);
        scheduler.schedule(event);

        // Process - should transition to TxTurnaround and schedule idle transition
        scheduler.process();
        assert_eq!(scheduler.state(), RadioState::TxTurnaround);

        // Should have scheduled turnaround completion at 100μs
        assert_eq!(scheduler.pending_count(), 1);
        assert_eq!(scheduler.next_deadline(), Some(100_000));

        // Advance past turnaround
        clock.set_time_ns(100_000);
        scheduler.process();
        assert_eq!(scheduler.state(), RadioState::Idle);
    }

    #[test]
    fn test_schedule_tx_burst() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .build()
            .unwrap();

        let (_start_id, _stop_id) =
            scheduler.schedule_tx_burst(1_000_000, Duration::from_millis(10));

        assert_eq!(scheduler.pending_count(), 2);

        // Advance to start
        clock.set_time_ns(1_000_000);
        scheduler.process();
        assert_eq!(scheduler.state(), RadioState::Transmitting);

        // Advance to stop
        clock.set_time_ns(11_000_000);
        scheduler.process();
        assert_eq!(scheduler.state(), RadioState::TxTurnaround);
    }

    #[test]
    fn test_hop_sequence() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .build()
            .unwrap();

        let frequencies = vec![900_000_000, 915_000_000, 920_000_000];
        let event_ids = scheduler.schedule_hop_sequence(
            0,
            Duration::from_millis(10),
            &frequencies,
        );

        assert_eq!(event_ids.len(), 3);
        assert_eq!(scheduler.pending_count(), 3);

        // Process first hop at t=0
        scheduler.process();

        // Process second hop at t=10ms
        clock.set_time_ns(10_000_000);
        scheduler.process();

        // Process third hop at t=20ms
        clock.set_time_ns(20_000_000);
        scheduler.process();

        assert_eq!(scheduler.pending_count(), 0);
    }

    #[test]
    fn test_repeating_event() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .deadline_tolerance(Duration::from_secs(1)) // Large tolerance for test
            .build()
            .unwrap();

        // Schedule repeating event every 10ms
        let event = ScheduledEvent::new(10_000_000, EventAction::Reset)
            .with_repeat(Duration::from_millis(10));

        scheduler.schedule(event);

        // Process at t=10ms
        clock.set_time_ns(10_000_000);
        scheduler.process();

        // Should have rescheduled for t=20ms
        assert_eq!(scheduler.pending_count(), 1);
        assert_eq!(scheduler.next_deadline(), Some(20_000_000));

        // Process at t=20ms
        clock.set_time_ns(20_000_000);
        scheduler.process();

        // Should have rescheduled for t=30ms
        assert_eq!(scheduler.pending_count(), 1);
        assert_eq!(scheduler.next_deadline(), Some(30_000_000));
    }

    #[test]
    fn test_cancel_event() {
        let scheduler = RealTimeScheduler::builder().build().unwrap();

        let event1 = ScheduledEvent::new(100, EventAction::StartTx);
        let event2 = ScheduledEvent::new(200, EventAction::StartRx);

        let id1 = event1.id;

        scheduler.schedule(event1);
        scheduler.schedule(event2);

        assert_eq!(scheduler.pending_count(), 2);

        // Cancel first event
        assert!(scheduler.cancel(id1));
        assert_eq!(scheduler.pending_count(), 1);

        // Try to cancel non-existent event
        assert!(!scheduler.cancel(99999));
    }

    #[test]
    fn test_callback() {
        use std::sync::atomic::{AtomicUsize, Ordering};

        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .build()
            .unwrap();

        let counter = Arc::new(AtomicUsize::new(0));
        let counter_clone = counter.clone();

        scheduler.schedule_callback(100_000, move |_state| {
            counter_clone.fetch_add(1, Ordering::SeqCst);
        });

        // Process before deadline
        scheduler.process();
        assert_eq!(counter.load(Ordering::SeqCst), 0);

        // Process at deadline
        clock.set_time_ns(100_000);
        scheduler.process();
        assert_eq!(counter.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn test_statistics() {
        let clock = Arc::new(MockClock::new());
        let scheduler = RealTimeScheduler::builder()
            .custom_clock(clock.clone())
            .build()
            .unwrap();

        scheduler.schedule(ScheduledEvent::new(0, EventAction::StartTx));
        scheduler.process();

        let stats = scheduler.stats();
        assert_eq!(stats.events_scheduled, 1);
        assert_eq!(stats.events_executed, 1);
        assert_eq!(stats.state_transitions, 1);
    }
}
