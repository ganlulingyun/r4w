//! # Real-Time Statistics and Diagnostics
//!
//! Provides lock-free statistics collection for monitoring RT performance.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rt::RtStats;
//!
//! let stats = RtStats::new();
//!
//! // Record processing times
//! stats.record_processing_time_ns(1500);
//! stats.record_processing_time_ns(2000);
//! stats.record_processing_time_ns(1800);
//!
//! // Check latency stats
//! println!("Max latency: {} ns", stats.max_latency_ns());
//! println!("Avg latency: {} ns", stats.avg_latency_ns());
//!
//! // Record overruns
//! stats.record_overrun();
//! println!("Overruns: {}", stats.overruns());
//! ```

use std::sync::atomic::{AtomicU64, AtomicI64, Ordering};

/// Lock-free real-time statistics collector.
///
/// All operations are non-blocking and suitable for RT contexts.
#[derive(Debug)]
pub struct RtStats {
    /// Total processing iterations
    iterations: AtomicU64,
    /// Total processing time in nanoseconds
    total_time_ns: AtomicU64,
    /// Maximum processing time observed
    max_time_ns: AtomicU64,
    /// Minimum processing time observed
    min_time_ns: AtomicU64,
    /// Buffer overrun count
    overruns: AtomicU64,
    /// Buffer underrun count
    underruns: AtomicU64,
    /// Samples processed
    samples_processed: AtomicU64,
    /// Timestamp of last activity (nanoseconds since epoch)
    last_activity_ns: AtomicI64,
}

impl Default for RtStats {
    fn default() -> Self {
        Self::new()
    }
}

impl RtStats {
    /// Create new statistics collector.
    pub fn new() -> Self {
        Self {
            iterations: AtomicU64::new(0),
            total_time_ns: AtomicU64::new(0),
            max_time_ns: AtomicU64::new(0),
            min_time_ns: AtomicU64::new(u64::MAX),
            overruns: AtomicU64::new(0),
            underruns: AtomicU64::new(0),
            samples_processed: AtomicU64::new(0),
            last_activity_ns: AtomicI64::new(0),
        }
    }

    /// Record a processing time in nanoseconds.
    #[inline]
    pub fn record_processing_time_ns(&self, time_ns: u64) {
        self.iterations.fetch_add(1, Ordering::Relaxed);
        self.total_time_ns.fetch_add(time_ns, Ordering::Relaxed);

        // Update max (lock-free)
        let mut current_max = self.max_time_ns.load(Ordering::Relaxed);
        while time_ns > current_max {
            match self.max_time_ns.compare_exchange_weak(
                current_max,
                time_ns,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(v) => current_max = v,
            }
        }

        // Update min (lock-free)
        let mut current_min = self.min_time_ns.load(Ordering::Relaxed);
        while time_ns < current_min {
            match self.min_time_ns.compare_exchange_weak(
                current_min,
                time_ns,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(v) => current_min = v,
            }
        }
    }

    /// Record a processing time in microseconds.
    #[inline]
    pub fn record_processing_time_us(&self, time_us: u64) {
        self.record_processing_time_ns(time_us * 1000);
    }

    /// Record a buffer overrun.
    #[inline]
    pub fn record_overrun(&self) {
        self.overruns.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a buffer underrun.
    #[inline]
    pub fn record_underrun(&self) {
        self.underruns.fetch_add(1, Ordering::Relaxed);
    }

    /// Record samples processed.
    #[inline]
    pub fn record_samples(&self, count: u64) {
        self.samples_processed.fetch_add(count, Ordering::Relaxed);
    }

    /// Update last activity timestamp.
    #[inline]
    pub fn touch(&self) {
        use std::time::{SystemTime, UNIX_EPOCH};
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as i64)
            .unwrap_or(0);
        self.last_activity_ns.store(now, Ordering::Relaxed);
    }

    /// Get total iterations.
    #[inline]
    pub fn iterations(&self) -> u64 {
        self.iterations.load(Ordering::Relaxed)
    }

    /// Get average latency in nanoseconds.
    pub fn avg_latency_ns(&self) -> u64 {
        let iters = self.iterations();
        if iters == 0 {
            0
        } else {
            self.total_time_ns.load(Ordering::Relaxed) / iters
        }
    }

    /// Get maximum latency in nanoseconds.
    #[inline]
    pub fn max_latency_ns(&self) -> u64 {
        let max = self.max_time_ns.load(Ordering::Relaxed);
        if max == 0 { 0 } else { max }
    }

    /// Get minimum latency in nanoseconds.
    #[inline]
    pub fn min_latency_ns(&self) -> u64 {
        let min = self.min_time_ns.load(Ordering::Relaxed);
        if min == u64::MAX { 0 } else { min }
    }

    /// Get overrun count.
    #[inline]
    pub fn overruns(&self) -> u64 {
        self.overruns.load(Ordering::Relaxed)
    }

    /// Get underrun count.
    #[inline]
    pub fn underruns(&self) -> u64 {
        self.underruns.load(Ordering::Relaxed)
    }

    /// Get total samples processed.
    #[inline]
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed.load(Ordering::Relaxed)
    }

    /// Get last activity timestamp.
    #[inline]
    pub fn last_activity_ns(&self) -> i64 {
        self.last_activity_ns.load(Ordering::Relaxed)
    }

    /// Check if there have been any overruns or underruns.
    #[inline]
    pub fn has_errors(&self) -> bool {
        self.overruns() > 0 || self.underruns() > 0
    }

    /// Reset all statistics.
    pub fn reset(&self) {
        self.iterations.store(0, Ordering::Relaxed);
        self.total_time_ns.store(0, Ordering::Relaxed);
        self.max_time_ns.store(0, Ordering::Relaxed);
        self.min_time_ns.store(u64::MAX, Ordering::Relaxed);
        self.overruns.store(0, Ordering::Relaxed);
        self.underruns.store(0, Ordering::Relaxed);
        self.samples_processed.store(0, Ordering::Relaxed);
    }

    /// Get a snapshot of all statistics.
    pub fn snapshot(&self) -> RtStatsSnapshot {
        RtStatsSnapshot {
            iterations: self.iterations(),
            avg_latency_ns: self.avg_latency_ns(),
            max_latency_ns: self.max_latency_ns(),
            min_latency_ns: self.min_latency_ns(),
            overruns: self.overruns(),
            underruns: self.underruns(),
            samples_processed: self.samples_processed(),
        }
    }
}

/// Snapshot of RT statistics at a point in time.
#[derive(Debug, Clone, Copy)]
pub struct RtStatsSnapshot {
    /// Total processing iterations
    pub iterations: u64,
    /// Average processing latency in nanoseconds
    pub avg_latency_ns: u64,
    /// Maximum processing latency in nanoseconds
    pub max_latency_ns: u64,
    /// Minimum processing latency in nanoseconds
    pub min_latency_ns: u64,
    /// Buffer overrun count
    pub overruns: u64,
    /// Buffer underrun count
    pub underruns: u64,
    /// Total samples processed
    pub samples_processed: u64,
}

impl RtStatsSnapshot {
    /// Get average latency in microseconds.
    #[inline]
    pub fn avg_latency_us(&self) -> f64 {
        self.avg_latency_ns as f64 / 1000.0
    }

    /// Get max latency in microseconds.
    #[inline]
    pub fn max_latency_us(&self) -> f64 {
        self.max_latency_ns as f64 / 1000.0
    }

    /// Get min latency in microseconds.
    #[inline]
    pub fn min_latency_us(&self) -> f64 {
        self.min_latency_ns as f64 / 1000.0
    }

    /// Check if statistics indicate RT issues.
    pub fn has_rt_issues(&self, max_acceptable_latency_us: f64) -> bool {
        self.overruns > 0
            || self.underruns > 0
            || self.max_latency_us() > max_acceptable_latency_us
    }
}

/// RAII timer that records processing time on drop.
///
/// # Example
///
/// ```rust
/// use r4w_core::rt::{RtStats, ProcessingTimer};
///
/// let stats = RtStats::new();
///
/// {
///     let _timer = ProcessingTimer::new(&stats);
///     // Do some work...
/// } // Timer records duration on drop
///
/// println!("Recorded iterations: {}", stats.iterations());
/// ```
pub struct ProcessingTimer<'a> {
    stats: &'a RtStats,
    start: std::time::Instant,
}

impl<'a> ProcessingTimer<'a> {
    /// Start a new timer.
    #[inline]
    pub fn new(stats: &'a RtStats) -> Self {
        Self {
            stats,
            start: std::time::Instant::now(),
        }
    }
}

impl<'a> Drop for ProcessingTimer<'a> {
    fn drop(&mut self) {
        let elapsed = self.start.elapsed().as_nanos() as u64;
        self.stats.record_processing_time_ns(elapsed);
    }
}

/// Watermark tracker for monitoring buffer fill levels.
///
/// Tracks high and low watermarks to detect buffer overflow/underflow conditions.
#[derive(Debug)]
pub struct Watermarks {
    /// Current level
    current: AtomicU64,
    /// High watermark
    high: AtomicU64,
    /// Low watermark
    low: AtomicU64,
    /// Capacity
    capacity: u64,
    /// Times high watermark exceeded
    high_exceeded: AtomicU64,
    /// Times low watermark exceeded
    low_exceeded: AtomicU64,
}

impl Watermarks {
    /// Create new watermark tracker.
    ///
    /// # Arguments
    ///
    /// * `capacity` - Maximum capacity
    /// * `high_pct` - High watermark as percentage (0-100)
    /// * `low_pct` - Low watermark as percentage (0-100)
    pub fn new(capacity: u64, high_pct: u8, low_pct: u8) -> Self {
        let high = (capacity * high_pct as u64) / 100;
        let low = (capacity * low_pct as u64) / 100;

        Self {
            current: AtomicU64::new(0),
            high: AtomicU64::new(high),
            low: AtomicU64::new(low),
            capacity,
            high_exceeded: AtomicU64::new(0),
            low_exceeded: AtomicU64::new(0),
        }
    }

    /// Update current level and check watermarks.
    ///
    /// Returns `true` if level is within acceptable range.
    #[inline]
    pub fn update(&self, level: u64) -> bool {
        self.current.store(level, Ordering::Relaxed);

        let high = self.high.load(Ordering::Relaxed);
        let low = self.low.load(Ordering::Relaxed);

        if level > high {
            self.high_exceeded.fetch_add(1, Ordering::Relaxed);
            false
        } else if level < low {
            self.low_exceeded.fetch_add(1, Ordering::Relaxed);
            false
        } else {
            true
        }
    }

    /// Get current level.
    #[inline]
    pub fn current(&self) -> u64 {
        self.current.load(Ordering::Relaxed)
    }

    /// Get fill percentage (0-100).
    #[inline]
    pub fn fill_percent(&self) -> u8 {
        ((self.current() * 100) / self.capacity) as u8
    }

    /// Get high watermark exceedance count.
    #[inline]
    pub fn high_exceeded_count(&self) -> u64 {
        self.high_exceeded.load(Ordering::Relaxed)
    }

    /// Get low watermark exceedance count.
    #[inline]
    pub fn low_exceeded_count(&self) -> u64 {
        self.low_exceeded.load(Ordering::Relaxed)
    }

    /// Check if any watermark was exceeded.
    #[inline]
    pub fn has_issues(&self) -> bool {
        self.high_exceeded_count() > 0 || self.low_exceeded_count() > 0
    }

    /// Reset exceedance counters.
    pub fn reset_counters(&self) {
        self.high_exceeded.store(0, Ordering::Relaxed);
        self.low_exceeded.store(0, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stats_basic() {
        let stats = RtStats::new();

        stats.record_processing_time_ns(1000);
        stats.record_processing_time_ns(2000);
        stats.record_processing_time_ns(3000);

        assert_eq!(stats.iterations(), 3);
        assert_eq!(stats.avg_latency_ns(), 2000);
        assert_eq!(stats.max_latency_ns(), 3000);
        assert_eq!(stats.min_latency_ns(), 1000);
    }

    #[test]
    fn test_stats_errors() {
        let stats = RtStats::new();

        assert!(!stats.has_errors());

        stats.record_overrun();
        assert!(stats.has_errors());
        assert_eq!(stats.overruns(), 1);

        stats.record_underrun();
        assert_eq!(stats.underruns(), 1);
    }

    #[test]
    fn test_stats_samples() {
        let stats = RtStats::new();

        stats.record_samples(1024);
        stats.record_samples(512);

        assert_eq!(stats.samples_processed(), 1536);
    }

    #[test]
    fn test_stats_reset() {
        let stats = RtStats::new();

        stats.record_processing_time_ns(1000);
        stats.record_overrun();
        stats.record_samples(100);

        stats.reset();

        assert_eq!(stats.iterations(), 0);
        assert_eq!(stats.overruns(), 0);
        assert_eq!(stats.samples_processed(), 0);
    }

    #[test]
    fn test_stats_snapshot() {
        let stats = RtStats::new();

        stats.record_processing_time_ns(5000);
        stats.record_processing_time_ns(10000);

        let snap = stats.snapshot();

        assert_eq!(snap.iterations, 2);
        assert_eq!(snap.avg_latency_ns, 7500);
        assert_eq!(snap.max_latency_ns, 10000);
        assert_eq!(snap.min_latency_ns, 5000);
    }

    #[test]
    fn test_processing_timer() {
        let stats = RtStats::new();

        {
            let _timer = ProcessingTimer::new(&stats);
            // Simulate work
            std::thread::sleep(std::time::Duration::from_micros(100));
        }

        assert_eq!(stats.iterations(), 1);
        // Should be at least 100us (100_000 ns)
        assert!(stats.max_latency_ns() >= 50_000);
    }

    #[test]
    fn test_watermarks() {
        let wm = Watermarks::new(1000, 80, 20);

        // Normal level
        assert!(wm.update(500));
        assert!(!wm.has_issues());

        // High exceeded
        assert!(!wm.update(900));
        assert!(wm.has_issues());
        assert_eq!(wm.high_exceeded_count(), 1);

        // Low exceeded
        assert!(!wm.update(100));
        assert_eq!(wm.low_exceeded_count(), 1);

        // Reset
        wm.reset_counters();
        assert!(!wm.has_issues());
    }

    #[test]
    fn test_watermarks_fill_percent() {
        let wm = Watermarks::new(1000, 80, 20);

        wm.update(250);
        assert_eq!(wm.fill_percent(), 25);

        wm.update(750);
        assert_eq!(wm.fill_percent(), 75);
    }
}
