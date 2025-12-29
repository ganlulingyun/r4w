//! # Multi-Clock Timing Model
//!
//! This module provides a unified timing abstraction for SDR applications,
//! supporting multiple clock domains:
//!
//! - **Sample Clock**: Monotonic sample counter for DSP operations
//! - **Wall Clock**: System time for logging and debugging
//! - **Hardware Clock**: Device timestamps from SDR hardware
//! - **Synced Time**: GPS/PTP synchronized time for multi-device coordination
//!
//! ## Clock Domain Relationships
//!
//! ```text
//! ┌──────────────────────────────────────────────────────────────┐
//! │                    Unified Timestamp                         │
//! │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌──────────┐│
//! │  │ SampleClock │ │  WallClock  │ │HardwareClock│ │SyncedTime││
//! │  │ (samples)   │ │ (Unix ns)   │ │ (HW ticks)  │ │(TAI ns)  ││
//! │  └─────────────┘ └─────────────┘ └─────────────┘ └──────────┘│
//! └──────────────────────────────────────────────────────────────┘
//!                              │
//!                    ClockDomain trait
//!                    (conversions between domains)
//! ```

use serde::{Deserialize, Serialize};
use std::ops::{Add, Sub};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Time synchronization source
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TimeSource {
    /// Free-running system clock (no external sync)
    Freerun,
    /// Network Time Protocol
    Ntp,
    /// Precision Time Protocol (IEEE 1588)
    Ptp,
    /// GPS time
    Gps,
    /// Pulse-per-second signal
    Pps,
    /// External reference
    External,
}

impl Default for TimeSource {
    fn default() -> Self {
        TimeSource::Freerun
    }
}

/// Sample-based clock for DSP operations.
///
/// This clock counts samples monotonically and provides conversion
/// to real-world time units based on the sample rate.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SampleClock {
    /// Number of samples since epoch (start of stream)
    samples: u64,
    /// Sample rate in Hz
    sample_rate: f64,
}

impl SampleClock {
    /// Create a new sample clock at sample 0.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            samples: 0,
            sample_rate,
        }
    }

    /// Create a sample clock at a specific sample count.
    pub fn at_sample(samples: u64, sample_rate: f64) -> Self {
        Self {
            samples,
            sample_rate,
        }
    }

    /// Get the current sample count.
    #[inline]
    pub fn samples(&self) -> u64 {
        self.samples
    }

    /// Get the sample rate in Hz.
    #[inline]
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Convert to duration since epoch.
    #[inline]
    pub fn to_duration(&self) -> Duration {
        let secs = self.samples as f64 / self.sample_rate;
        Duration::from_secs_f64(secs)
    }

    /// Convert to nanoseconds since epoch.
    #[inline]
    pub fn to_nanos(&self) -> u64 {
        ((self.samples as f64 / self.sample_rate) * 1e9) as u64
    }

    /// Advance by a number of samples.
    #[inline]
    pub fn advance(&mut self, samples: u64) {
        self.samples = self.samples.saturating_add(samples);
    }

    /// Advance by a duration (rounded to nearest sample).
    pub fn advance_duration(&mut self, duration: Duration) {
        let samples = (duration.as_secs_f64() * self.sample_rate).round() as u64;
        self.advance(samples);
    }

    /// Calculate samples for a given duration.
    #[inline]
    pub fn samples_for_duration(&self, duration: Duration) -> u64 {
        (duration.as_secs_f64() * self.sample_rate).round() as u64
    }

    /// Calculate duration for a given number of samples.
    #[inline]
    pub fn duration_for_samples(&self, samples: u64) -> Duration {
        Duration::from_secs_f64(samples as f64 / self.sample_rate)
    }

    /// Create a new clock offset by samples.
    #[inline]
    pub fn offset_samples(&self, samples: i64) -> Self {
        Self {
            samples: if samples >= 0 {
                self.samples.saturating_add(samples as u64)
            } else {
                self.samples.saturating_sub((-samples) as u64)
            },
            sample_rate: self.sample_rate,
        }
    }

    /// Convert to seconds since epoch.
    #[inline]
    pub fn to_seconds(&self) -> f64 {
        self.samples as f64 / self.sample_rate
    }

    /// Set the sample rate (e.g., after reconfiguration).
    ///
    /// Note: This changes the time interpretation but not the sample count.
    #[inline]
    pub fn set_sample_rate(&mut self, sample_rate: f64) {
        self.sample_rate = sample_rate;
    }

    /// Reset to zero samples.
    #[inline]
    pub fn reset(&mut self) {
        self.samples = 0;
    }
}

impl Add<u64> for SampleClock {
    type Output = Self;

    fn add(self, samples: u64) -> Self::Output {
        Self {
            samples: self.samples.saturating_add(samples),
            sample_rate: self.sample_rate,
        }
    }
}

impl Sub for SampleClock {
    type Output = i64;

    fn sub(self, other: Self) -> Self::Output {
        self.samples as i64 - other.samples as i64
    }
}

/// Wall clock with nanosecond precision.
///
/// Based on Unix epoch (1970-01-01 00:00:00 UTC).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct WallClock {
    /// Nanoseconds since Unix epoch
    epoch_ns: u64,
}

impl WallClock {
    /// Create a wall clock at the current system time.
    pub fn now() -> Self {
        let duration = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default();
        Self {
            epoch_ns: duration.as_nanos() as u64,
        }
    }

    /// Create a wall clock at a specific Unix timestamp (nanoseconds).
    pub fn from_nanos(epoch_ns: u64) -> Self {
        Self { epoch_ns }
    }

    /// Create a wall clock from Unix timestamp (seconds).
    pub fn from_secs(epoch_secs: u64) -> Self {
        Self {
            epoch_ns: epoch_secs * 1_000_000_000,
        }
    }

    /// Get nanoseconds since Unix epoch.
    #[inline]
    pub fn as_nanos(&self) -> u64 {
        self.epoch_ns
    }

    /// Get seconds since Unix epoch.
    #[inline]
    pub fn as_secs(&self) -> u64 {
        self.epoch_ns / 1_000_000_000
    }

    /// Get microseconds since Unix epoch.
    #[inline]
    pub fn as_micros(&self) -> u64 {
        self.epoch_ns / 1_000
    }

    /// Convert to std::time::SystemTime.
    pub fn to_system_time(&self) -> SystemTime {
        UNIX_EPOCH + Duration::from_nanos(self.epoch_ns)
    }

    /// Advance by a duration.
    pub fn advance(&mut self, duration: Duration) {
        self.epoch_ns = self.epoch_ns.saturating_add(duration.as_nanos() as u64);
    }

    /// Calculate elapsed time since another wall clock.
    pub fn elapsed_since(&self, other: &WallClock) -> Duration {
        if self.epoch_ns >= other.epoch_ns {
            Duration::from_nanos(self.epoch_ns - other.epoch_ns)
        } else {
            Duration::ZERO
        }
    }
}

impl Default for WallClock {
    fn default() -> Self {
        Self::now()
    }
}

impl Add<Duration> for WallClock {
    type Output = Self;

    fn add(self, duration: Duration) -> Self::Output {
        Self {
            epoch_ns: self.epoch_ns.saturating_add(duration.as_nanos() as u64),
        }
    }
}

impl Sub for WallClock {
    type Output = Duration;

    fn sub(self, other: Self) -> Self::Output {
        if self.epoch_ns >= other.epoch_ns {
            Duration::from_nanos(self.epoch_ns - other.epoch_ns)
        } else {
            Duration::ZERO
        }
    }
}

/// Hardware timestamp from SDR device.
///
/// Represents the device's internal clock, which may run at a different
/// rate than the sample clock and may be synchronized to external references.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HardwareClock {
    /// Hardware tick count
    ticks: u64,
    /// Tick rate in Hz (typically derived from FPGA/device clock)
    tick_rate: f64,
    /// Whether locked to PPS reference
    pps_locked: bool,
    /// Lock status
    lock_status: ClockLockStatus,
}

/// Clock lock status for external synchronization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClockLockStatus {
    /// Not synchronized
    Unlocked,
    /// Acquiring lock
    Acquiring,
    /// Locked and stable
    Locked,
    /// Lock lost (was locked, now searching)
    LostLock,
}

impl Default for ClockLockStatus {
    fn default() -> Self {
        ClockLockStatus::Unlocked
    }
}

impl HardwareClock {
    /// Create a new hardware clock.
    pub fn new(tick_rate: f64) -> Self {
        Self {
            ticks: 0,
            tick_rate,
            pps_locked: false,
            lock_status: ClockLockStatus::Unlocked,
        }
    }

    /// Create a hardware clock at a specific tick count.
    pub fn at_tick(ticks: u64, tick_rate: f64) -> Self {
        Self {
            ticks,
            tick_rate,
            pps_locked: false,
            lock_status: ClockLockStatus::Unlocked,
        }
    }

    /// Get the current tick count.
    #[inline]
    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    /// Get the tick rate in Hz.
    #[inline]
    pub fn tick_rate(&self) -> f64 {
        self.tick_rate
    }

    /// Check if locked to PPS.
    #[inline]
    pub fn is_pps_locked(&self) -> bool {
        self.pps_locked
    }

    /// Get the lock status.
    #[inline]
    pub fn lock_status(&self) -> ClockLockStatus {
        self.lock_status
    }

    /// Set the PPS lock status.
    pub fn set_pps_locked(&mut self, locked: bool) {
        self.pps_locked = locked;
        self.lock_status = if locked {
            ClockLockStatus::Locked
        } else {
            ClockLockStatus::Unlocked
        };
    }

    /// Convert to duration since hardware epoch.
    #[inline]
    pub fn to_duration(&self) -> Duration {
        Duration::from_secs_f64(self.ticks as f64 / self.tick_rate)
    }

    /// Convert to nanoseconds since hardware epoch.
    #[inline]
    pub fn to_nanos(&self) -> u64 {
        ((self.ticks as f64 / self.tick_rate) * 1e9) as u64
    }

    /// Advance by ticks.
    pub fn advance(&mut self, ticks: u64) {
        self.ticks = self.ticks.saturating_add(ticks);
    }

    /// Convert ticks to samples at a given sample rate.
    #[inline]
    pub fn ticks_to_samples(&self, ticks: u64, sample_rate: f64) -> u64 {
        ((ticks as f64 / self.tick_rate) * sample_rate) as u64
    }

    /// Convert samples to ticks at a given sample rate.
    #[inline]
    pub fn samples_to_ticks(&self, samples: u64, sample_rate: f64) -> u64 {
        ((samples as f64 / sample_rate) * self.tick_rate) as u64
    }

    /// Set the clock time in seconds.
    ///
    /// Converts seconds to ticks based on the tick rate.
    #[inline]
    pub fn set_time_seconds(&mut self, seconds: f64) {
        self.ticks = (seconds * self.tick_rate) as u64;
    }

    /// Get the clock time in seconds.
    #[inline]
    pub fn to_seconds(&self) -> f64 {
        self.ticks as f64 / self.tick_rate
    }

    /// Set the tick count directly.
    #[inline]
    pub fn set_ticks(&mut self, ticks: u64) {
        self.ticks = ticks;
    }

    /// Reset to zero ticks.
    #[inline]
    pub fn reset(&mut self) {
        self.ticks = 0;
    }
}

/// GPS/PTP synchronized time.
///
/// Based on TAI (International Atomic Time) which does not have leap seconds.
/// This provides absolute time suitable for multi-device synchronization.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SyncedTime {
    /// TAI nanoseconds since epoch
    tai_ns: u64,
    /// Timing uncertainty in nanoseconds
    uncertainty_ns: u32,
    /// Time synchronization source
    source: TimeSource,
}

impl SyncedTime {
    /// TAI epoch offset from Unix epoch (seconds).
    /// TAI is ahead of UTC by the number of leap seconds.
    /// As of 2024, TAI = UTC + 37 seconds.
    pub const TAI_OFFSET_SECS: u64 = 37;

    /// Create a new synced time from TAI nanoseconds.
    pub fn from_tai_nanos(tai_ns: u64, source: TimeSource) -> Self {
        Self {
            tai_ns,
            uncertainty_ns: 0,
            source,
        }
    }

    /// Create from UTC nanoseconds (converts to TAI).
    pub fn from_utc_nanos(utc_ns: u64, source: TimeSource) -> Self {
        Self {
            tai_ns: utc_ns + Self::TAI_OFFSET_SECS * 1_000_000_000,
            uncertainty_ns: 0,
            source,
        }
    }

    /// Create from GPS time (GPS epoch: Jan 6, 1980).
    pub fn from_gps_time(gps_week: u16, gps_tow_ns: u64) -> Self {
        // GPS epoch in Unix time: 315964800 seconds
        const GPS_EPOCH_UNIX: u64 = 315964800;
        const WEEK_NS: u64 = 7 * 24 * 60 * 60 * 1_000_000_000;

        let unix_ns =
            (GPS_EPOCH_UNIX * 1_000_000_000) + (gps_week as u64 * WEEK_NS) + gps_tow_ns;
        Self::from_utc_nanos(unix_ns, TimeSource::Gps)
    }

    /// Get TAI nanoseconds.
    #[inline]
    pub fn tai_nanos(&self) -> u64 {
        self.tai_ns
    }

    /// Get UTC nanoseconds (converts from TAI).
    #[inline]
    pub fn utc_nanos(&self) -> u64 {
        self.tai_ns.saturating_sub(Self::TAI_OFFSET_SECS * 1_000_000_000)
    }

    /// Get timing uncertainty in nanoseconds.
    #[inline]
    pub fn uncertainty_ns(&self) -> u32 {
        self.uncertainty_ns
    }

    /// Set timing uncertainty.
    pub fn with_uncertainty(mut self, uncertainty_ns: u32) -> Self {
        self.uncertainty_ns = uncertainty_ns;
        self
    }

    /// Get the time source.
    #[inline]
    pub fn source(&self) -> TimeSource {
        self.source
    }

    /// Check if this time is synchronized (not freerun).
    #[inline]
    pub fn is_synchronized(&self) -> bool {
        self.source != TimeSource::Freerun
    }

    /// Convert to wall clock (UTC-based).
    pub fn to_wall_clock(&self) -> WallClock {
        WallClock::from_nanos(self.utc_nanos())
    }
}

impl Default for SyncedTime {
    fn default() -> Self {
        let wall = WallClock::now();
        Self::from_utc_nanos(wall.as_nanos(), TimeSource::Freerun)
    }
}

/// Unified timestamp combining all clock domains.
///
/// This is the primary timestamp type for R4W, providing a consistent
/// view across sample time, wall time, hardware time, and synchronized time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Timestamp {
    /// Sample-based timing
    pub sample: SampleClock,
    /// Wall clock (system time)
    pub wall: WallClock,
    /// Hardware clock (optional, only with real hardware)
    pub hardware: Option<HardwareClock>,
    /// Synchronized time (optional, only with GPS/PTP)
    pub synced: Option<SyncedTime>,
}

impl Timestamp {
    /// Create a new timestamp at sample 0 with current wall time.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample: SampleClock::new(sample_rate),
            wall: WallClock::now(),
            hardware: None,
            synced: None,
        }
    }

    /// Create a timestamp at a specific sample.
    pub fn at_sample(samples: u64, sample_rate: f64) -> Self {
        Self {
            sample: SampleClock::at_sample(samples, sample_rate),
            wall: WallClock::now(),
            hardware: None,
            synced: None,
        }
    }

    /// Add hardware clock.
    pub fn with_hardware(mut self, hw: HardwareClock) -> Self {
        self.hardware = Some(hw);
        self
    }

    /// Add synchronized time.
    pub fn with_synced(mut self, synced: SyncedTime) -> Self {
        self.synced = Some(synced);
        self
    }

    /// Advance all clocks by the given number of samples.
    pub fn advance_samples(&mut self, samples: u64) {
        // Advance sample clock
        self.sample.advance(samples);

        // Calculate duration
        let duration = self.sample.duration_for_samples(samples);

        // Advance wall clock
        self.wall.advance(duration);

        // Advance hardware clock if present
        if let Some(ref mut hw) = self.hardware {
            let ticks = hw.samples_to_ticks(samples, self.sample.sample_rate());
            hw.advance(ticks);
        }
    }

    /// Get sample rate.
    #[inline]
    pub fn sample_rate(&self) -> f64 {
        self.sample.sample_rate()
    }

    /// Check if hardware timestamp is available.
    #[inline]
    pub fn has_hardware_time(&self) -> bool {
        self.hardware.is_some()
    }

    /// Check if synchronized time is available.
    #[inline]
    pub fn has_synced_time(&self) -> bool {
        self.synced.is_some()
    }

    /// Get the best available time source.
    pub fn best_time_source(&self) -> TimeSource {
        if let Some(ref synced) = self.synced {
            synced.source()
        } else if self.hardware.is_some() {
            TimeSource::External
        } else {
            TimeSource::Freerun
        }
    }
}

impl Default for Timestamp {
    fn default() -> Self {
        Self::new(1.0)
    }
}

/// Clock domain conversion trait.
///
/// Enables conversion between different clock domains.
pub trait ClockDomain {
    /// Convert to sample count at the given sample rate.
    fn to_samples(&self, sample_rate: f64) -> u64;

    /// Convert to wall clock nanoseconds.
    fn to_wall_nanos(&self) -> u64;

    /// Convert to duration.
    fn to_duration(&self) -> Duration;
}

impl ClockDomain for SampleClock {
    fn to_samples(&self, _sample_rate: f64) -> u64 {
        self.samples
    }

    fn to_wall_nanos(&self) -> u64 {
        self.to_nanos()
    }

    fn to_duration(&self) -> Duration {
        SampleClock::to_duration(self)
    }
}

impl ClockDomain for WallClock {
    fn to_samples(&self, sample_rate: f64) -> u64 {
        ((self.epoch_ns as f64 / 1e9) * sample_rate) as u64
    }

    fn to_wall_nanos(&self) -> u64 {
        self.epoch_ns
    }

    fn to_duration(&self) -> Duration {
        Duration::from_nanos(self.epoch_ns)
    }
}

impl ClockDomain for HardwareClock {
    fn to_samples(&self, sample_rate: f64) -> u64 {
        self.ticks_to_samples(self.ticks, sample_rate)
    }

    fn to_wall_nanos(&self) -> u64 {
        self.to_nanos()
    }

    fn to_duration(&self) -> Duration {
        HardwareClock::to_duration(self)
    }
}

/// High-resolution monotonic timer for measuring intervals.
///
/// Uses `std::time::Instant` internally for monotonic timing.
#[derive(Debug, Clone)]
pub struct MonotonicTimer {
    start: Instant,
}

impl MonotonicTimer {
    /// Create and start a new timer.
    pub fn start() -> Self {
        Self {
            start: Instant::now(),
        }
    }

    /// Get elapsed time since start.
    pub fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    /// Get elapsed time in microseconds.
    pub fn elapsed_micros(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }

    /// Get elapsed time in nanoseconds.
    pub fn elapsed_nanos(&self) -> u128 {
        self.start.elapsed().as_nanos()
    }

    /// Reset the timer.
    pub fn reset(&mut self) {
        self.start = Instant::now();
    }

    /// Lap: return elapsed time and reset.
    pub fn lap(&mut self) -> Duration {
        let elapsed = self.elapsed();
        self.reset();
        elapsed
    }
}

impl Default for MonotonicTimer {
    fn default() -> Self {
        Self::start()
    }
}

/// Time-based rate limiter for real-time streaming.
///
/// Helps maintain consistent timing for TX/RX operations.
#[derive(Debug)]
pub struct RateLimiter {
    /// Target samples per second
    sample_rate: f64,
    /// Samples sent/received
    samples_processed: u64,
    /// Start time
    start: Instant,
}

impl RateLimiter {
    /// Create a new rate limiter.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            samples_processed: 0,
            start: Instant::now(),
        }
    }

    /// Reset the rate limiter.
    pub fn reset(&mut self) {
        self.samples_processed = 0;
        self.start = Instant::now();
    }

    /// Record samples processed and return time to sleep (if any).
    ///
    /// Returns the duration to sleep to maintain the target rate.
    pub fn record(&mut self, samples: u64) -> Option<Duration> {
        self.samples_processed += samples;

        let target_duration = Duration::from_secs_f64(self.samples_processed as f64 / self.sample_rate);
        let actual_duration = self.start.elapsed();

        if target_duration > actual_duration {
            Some(target_duration - actual_duration)
        } else {
            None
        }
    }

    /// Get current processing rate.
    pub fn current_rate(&self) -> f64 {
        let elapsed = self.start.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            self.samples_processed as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Check if we're keeping up with real-time.
    pub fn is_realtime(&self) -> bool {
        self.current_rate() >= self.sample_rate * 0.99
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_clock_basic() {
        let clock = SampleClock::new(48000.0);
        assert_eq!(clock.samples(), 0);
        assert_eq!(clock.sample_rate(), 48000.0);
    }

    #[test]
    fn test_sample_clock_to_duration() {
        let clock = SampleClock::at_sample(48000, 48000.0);
        let duration = clock.to_duration();
        assert!((duration.as_secs_f64() - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_sample_clock_advance() {
        let mut clock = SampleClock::new(1000.0);
        clock.advance(500);
        assert_eq!(clock.samples(), 500);

        clock.advance_duration(Duration::from_millis(250));
        assert_eq!(clock.samples(), 750);
    }

    #[test]
    fn test_sample_clock_add() {
        let clock = SampleClock::at_sample(100, 1000.0);
        let new_clock = clock + 50;
        assert_eq!(new_clock.samples(), 150);
    }

    #[test]
    fn test_sample_clock_sub() {
        let clock1 = SampleClock::at_sample(100, 1000.0);
        let clock2 = SampleClock::at_sample(50, 1000.0);
        assert_eq!(clock1 - clock2, 50);
    }

    #[test]
    fn test_wall_clock_now() {
        let clock = WallClock::now();
        assert!(clock.as_nanos() > 0);
    }

    #[test]
    fn test_wall_clock_from_secs() {
        let clock = WallClock::from_secs(1000);
        assert_eq!(clock.as_secs(), 1000);
    }

    #[test]
    fn test_wall_clock_elapsed() {
        let clock1 = WallClock::from_nanos(1_000_000_000);
        let clock2 = WallClock::from_nanos(2_000_000_000);
        let elapsed = clock2.elapsed_since(&clock1);
        assert_eq!(elapsed, Duration::from_secs(1));
    }

    #[test]
    fn test_hardware_clock() {
        let hw = HardwareClock::new(100_000_000.0); // 100 MHz
        assert_eq!(hw.ticks(), 0);
        assert!(!hw.is_pps_locked());
    }

    #[test]
    fn test_hardware_clock_conversion() {
        let hw = HardwareClock::at_tick(100_000_000, 100_000_000.0);
        let samples = hw.ticks_to_samples(hw.ticks(), 48000.0);
        assert_eq!(samples, 48000); // 1 second at 48kHz
    }

    #[test]
    fn test_synced_time_tai_utc() {
        let synced = SyncedTime::from_utc_nanos(1_000_000_000, TimeSource::Gps);
        // TAI should be ahead of UTC
        assert!(synced.tai_nanos() > 1_000_000_000);
        // Converting back should give original UTC
        assert_eq!(synced.utc_nanos(), 1_000_000_000);
    }

    #[test]
    fn test_timestamp_advance() {
        let mut ts = Timestamp::new(48000.0);
        let initial_samples = ts.sample.samples();
        let initial_wall = ts.wall.as_nanos();

        ts.advance_samples(48000);

        assert_eq!(ts.sample.samples(), initial_samples + 48000);
        // Wall clock should advance by ~1 second (1e9 ns)
        let wall_diff = ts.wall.as_nanos() - initial_wall;
        assert!((wall_diff as f64 - 1e9).abs() < 1e6);
    }

    #[test]
    fn test_monotonic_timer() {
        let timer = MonotonicTimer::start();
        std::thread::sleep(Duration::from_millis(10));
        let elapsed = timer.elapsed();
        assert!(elapsed >= Duration::from_millis(9));
    }

    #[test]
    fn test_rate_limiter() {
        let mut limiter = RateLimiter::new(48000.0);
        let sleep = limiter.record(480);
        // After recording 480 samples at 48kHz, we should have ~10ms target time
        // Since we just started, we probably need to sleep
        assert!(sleep.is_some() || limiter.current_rate() >= 48000.0);
    }

    #[test]
    fn test_clock_domain_trait() {
        let sample = SampleClock::at_sample(48000, 48000.0);
        assert_eq!(sample.to_samples(48000.0), 48000);
        assert!((sample.to_wall_nanos() as f64 - 1e9).abs() < 1.0);
    }
}
