//! # Latency Histogram (cyclictest-style)
//!
//! Provides lock-free latency histogram collection for RT performance analysis.
//! Similar to Linux's `cyclictest` but for SDR DSP pipelines.
//!
//! ## Features
//!
//! - Lock-free atomic histogram bins
//! - Configurable bin size and range
//! - Percentile calculations (p50, p90, p99, p99.9)
//! - ASCII histogram visualization
//! - JSON export for analysis
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rt::LatencyHistogram;
//!
//! // Create histogram: 0-1000µs range, 10µs bins
//! let hist = LatencyHistogram::new_us(1000, 10);
//!
//! // Record latencies
//! hist.record_ns(50_000);   // 50µs
//! hist.record_ns(150_000);  // 150µs
//! hist.record_ns(75_000);   // 75µs
//!
//! // Get statistics
//! println!("p50: {}µs", hist.percentile_ns(50.0) / 1000);
//! println!("p99: {}µs", hist.percentile_ns(99.0) / 1000);
//!
//! // Print ASCII histogram
//! println!("{}", hist.ascii_histogram(40));
//! ```

use std::sync::atomic::{AtomicU64, Ordering};

/// Lock-free latency histogram for RT performance analysis.
///
/// Uses atomic bins for lock-free recording from RT threads.
/// Designed after cyclictest's histogram functionality.
#[derive(Debug)]
pub struct LatencyHistogram {
    /// Histogram bins (atomic for lock-free access)
    bins: Vec<AtomicU64>,
    /// Bin size in nanoseconds
    bin_size_ns: u64,
    /// Maximum trackable latency in nanoseconds
    max_ns: u64,
    /// Overflow count (latencies > max_ns)
    overflow: AtomicU64,
    /// Total samples recorded
    total: AtomicU64,
    /// Minimum latency observed
    min_ns: AtomicU64,
    /// Maximum latency observed
    max_observed_ns: AtomicU64,
    /// Sum of all latencies (for mean calculation)
    sum_ns: AtomicU64,
}

impl LatencyHistogram {
    /// Create a new histogram with nanosecond granularity.
    ///
    /// # Arguments
    ///
    /// * `max_ns` - Maximum trackable latency in nanoseconds
    /// * `bin_size_ns` - Size of each histogram bin in nanoseconds
    pub fn new(max_ns: u64, bin_size_ns: u64) -> Self {
        let num_bins = ((max_ns / bin_size_ns) + 1) as usize;
        let bins = (0..num_bins).map(|_| AtomicU64::new(0)).collect();

        Self {
            bins,
            bin_size_ns,
            max_ns,
            overflow: AtomicU64::new(0),
            total: AtomicU64::new(0),
            min_ns: AtomicU64::new(u64::MAX),
            max_observed_ns: AtomicU64::new(0),
            sum_ns: AtomicU64::new(0),
        }
    }

    /// Create a new histogram with microsecond parameters.
    ///
    /// # Arguments
    ///
    /// * `max_us` - Maximum trackable latency in microseconds
    /// * `bin_size_us` - Size of each histogram bin in microseconds
    pub fn new_us(max_us: u64, bin_size_us: u64) -> Self {
        Self::new(max_us * 1000, bin_size_us * 1000)
    }

    /// Create a new histogram with millisecond parameters.
    ///
    /// # Arguments
    ///
    /// * `max_ms` - Maximum trackable latency in milliseconds
    /// * `bin_size_ms` - Size of each histogram bin in milliseconds
    pub fn new_ms(max_ms: u64, bin_size_ms: u64) -> Self {
        Self::new(max_ms * 1_000_000, bin_size_ms * 1_000_000)
    }

    /// Record a latency value in nanoseconds.
    #[inline]
    pub fn record_ns(&self, latency_ns: u64) {
        self.total.fetch_add(1, Ordering::Relaxed);
        self.sum_ns.fetch_add(latency_ns, Ordering::Relaxed);

        // Update min
        let mut current_min = self.min_ns.load(Ordering::Relaxed);
        while latency_ns < current_min {
            match self.min_ns.compare_exchange_weak(
                current_min,
                latency_ns,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(v) => current_min = v,
            }
        }

        // Update max
        let mut current_max = self.max_observed_ns.load(Ordering::Relaxed);
        while latency_ns > current_max {
            match self.max_observed_ns.compare_exchange_weak(
                current_max,
                latency_ns,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(v) => current_max = v,
            }
        }

        // Record in bin
        if latency_ns >= self.max_ns {
            self.overflow.fetch_add(1, Ordering::Relaxed);
        } else {
            let bin_idx = (latency_ns / self.bin_size_ns) as usize;
            if bin_idx < self.bins.len() {
                self.bins[bin_idx].fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Record a latency value in microseconds.
    #[inline]
    pub fn record_us(&self, latency_us: u64) {
        self.record_ns(latency_us * 1000);
    }

    /// Record a latency value in milliseconds.
    #[inline]
    pub fn record_ms(&self, latency_ms: u64) {
        self.record_ns(latency_ms * 1_000_000);
    }

    /// Get total number of samples recorded.
    #[inline]
    pub fn count(&self) -> u64 {
        self.total.load(Ordering::Relaxed)
    }

    /// Get overflow count (samples above max_ns).
    #[inline]
    pub fn overflow_count(&self) -> u64 {
        self.overflow.load(Ordering::Relaxed)
    }

    /// Get minimum observed latency in nanoseconds.
    #[inline]
    pub fn min_ns(&self) -> u64 {
        let min = self.min_ns.load(Ordering::Relaxed);
        if min == u64::MAX { 0 } else { min }
    }

    /// Get maximum observed latency in nanoseconds.
    #[inline]
    pub fn max_ns(&self) -> u64 {
        self.max_observed_ns.load(Ordering::Relaxed)
    }

    /// Get mean latency in nanoseconds.
    pub fn mean_ns(&self) -> u64 {
        let total = self.count();
        if total == 0 {
            0
        } else {
            self.sum_ns.load(Ordering::Relaxed) / total
        }
    }

    /// Get the latency value at a given percentile.
    ///
    /// # Arguments
    ///
    /// * `percentile` - Percentile value (0.0 to 100.0)
    ///
    /// # Returns
    ///
    /// Latency in nanoseconds at the given percentile
    pub fn percentile_ns(&self, percentile: f64) -> u64 {
        let total = self.count();
        if total == 0 {
            return 0;
        }

        let target = ((percentile / 100.0) * total as f64).ceil() as u64;
        let mut cumulative = 0u64;

        for (idx, bin) in self.bins.iter().enumerate() {
            cumulative += bin.load(Ordering::Relaxed);
            if cumulative >= target {
                // Return the upper bound of this bin
                return (idx as u64 + 1) * self.bin_size_ns;
            }
        }

        // All in overflow
        self.max_ns
    }

    /// Get common percentiles (p50, p90, p99, p99.9).
    pub fn percentiles(&self) -> LatencyPercentiles {
        LatencyPercentiles {
            p50_ns: self.percentile_ns(50.0),
            p90_ns: self.percentile_ns(90.0),
            p99_ns: self.percentile_ns(99.0),
            p999_ns: self.percentile_ns(99.9),
        }
    }

    /// Get histogram statistics.
    pub fn statistics(&self) -> LatencyStatistics {
        LatencyStatistics {
            count: self.count(),
            min_ns: self.min_ns(),
            max_ns: self.max_ns(),
            mean_ns: self.mean_ns(),
            overflow: self.overflow_count(),
            percentiles: self.percentiles(),
        }
    }

    /// Generate an ASCII histogram visualization.
    ///
    /// # Arguments
    ///
    /// * `width` - Width of the histogram bars in characters
    pub fn ascii_histogram(&self, width: usize) -> String {
        let total = self.count();
        if total == 0 {
            return "No samples recorded".to_string();
        }

        // Find max bin count for scaling
        let max_count = self.bins.iter()
            .map(|b| b.load(Ordering::Relaxed))
            .max()
            .unwrap_or(0)
            .max(self.overflow.load(Ordering::Relaxed));

        if max_count == 0 {
            return "All samples are zero".to_string();
        }

        let mut output = String::new();
        output.push_str(&format!("Latency Histogram ({}  samples)\n", total));
        output.push_str(&format!("Bin size: {}ns, Max: {}ns\n", self.bin_size_ns, self.max_ns));
        output.push_str(&format!("Min: {}ns, Max: {}ns, Mean: {}ns\n\n",
            self.min_ns(), self.max_ns(), self.mean_ns()));

        // Only show non-zero bins
        for (idx, bin) in self.bins.iter().enumerate() {
            let count = bin.load(Ordering::Relaxed);
            if count > 0 {
                let bar_len = ((count as f64 / max_count as f64) * width as f64) as usize;
                let bar: String = "█".repeat(bar_len);
                let lower = idx as u64 * self.bin_size_ns;
                let upper = (idx as u64 + 1) * self.bin_size_ns;
                output.push_str(&format!("{:>8}-{:<8}ns [{:>8}] {}\n",
                    lower, upper, count, bar));
            }
        }

        // Show overflow if any
        let overflow = self.overflow.load(Ordering::Relaxed);
        if overflow > 0 {
            let bar_len = ((overflow as f64 / max_count as f64) * width as f64) as usize;
            let bar: String = "█".repeat(bar_len);
            output.push_str(&format!("{:>8}+        ns [{:>8}] {}\n",
                self.max_ns, overflow, bar));
        }

        // Percentiles summary
        let pct = self.percentiles();
        output.push_str(&format!("\nPercentiles: p50={}ns p90={}ns p99={}ns p99.9={}ns\n",
            pct.p50_ns, pct.p90_ns, pct.p99_ns, pct.p999_ns));

        output
    }

    /// Generate ASCII histogram in microseconds.
    pub fn ascii_histogram_us(&self, width: usize) -> String {
        let total = self.count();
        if total == 0 {
            return "No samples recorded".to_string();
        }

        let max_count = self.bins.iter()
            .map(|b| b.load(Ordering::Relaxed))
            .max()
            .unwrap_or(0)
            .max(self.overflow.load(Ordering::Relaxed));

        if max_count == 0 {
            return "All samples are zero".to_string();
        }

        let mut output = String::new();
        output.push_str(&format!("Latency Histogram ({} samples)\n", total));
        output.push_str(&format!("Bin size: {}µs, Max: {}µs\n",
            self.bin_size_ns / 1000, self.max_ns / 1000));
        output.push_str(&format!("Min: {}µs, Max: {}µs, Mean: {}µs\n\n",
            self.min_ns() / 1000, self.max_ns() / 1000, self.mean_ns() / 1000));

        for (idx, bin) in self.bins.iter().enumerate() {
            let count = bin.load(Ordering::Relaxed);
            if count > 0 {
                let bar_len = ((count as f64 / max_count as f64) * width as f64) as usize;
                let bar: String = "█".repeat(bar_len);
                let lower = (idx as u64 * self.bin_size_ns) / 1000;
                let upper = ((idx as u64 + 1) * self.bin_size_ns) / 1000;
                output.push_str(&format!("{:>6}-{:<6}µs [{:>8}] {}\n",
                    lower, upper, count, bar));
            }
        }

        let overflow = self.overflow.load(Ordering::Relaxed);
        if overflow > 0 {
            let bar_len = ((overflow as f64 / max_count as f64) * width as f64) as usize;
            let bar: String = "█".repeat(bar_len);
            output.push_str(&format!("{:>6}+      µs [{:>8}] {}\n",
                self.max_ns / 1000, overflow, bar));
        }

        let pct = self.percentiles();
        output.push_str(&format!("\nPercentiles: p50={}µs p90={}µs p99={}µs p99.9={}µs\n",
            pct.p50_ns / 1000, pct.p90_ns / 1000, pct.p99_ns / 1000, pct.p999_ns / 1000));

        output
    }

    /// Export histogram data as JSON.
    pub fn to_json(&self) -> String {
        let stats = self.statistics();
        let bins: Vec<(u64, u64)> = self.bins.iter()
            .enumerate()
            .map(|(idx, bin)| (idx as u64 * self.bin_size_ns, bin.load(Ordering::Relaxed)))
            .filter(|(_, count)| *count > 0)
            .collect();

        format!(
            r#"{{"count":{},"min_ns":{},"max_ns":{},"mean_ns":{},"overflow":{},"p50_ns":{},"p90_ns":{},"p99_ns":{},"p999_ns":{},"bin_size_ns":{},"bins":{:?}}}"#,
            stats.count, stats.min_ns, stats.max_ns, stats.mean_ns, stats.overflow,
            stats.percentiles.p50_ns, stats.percentiles.p90_ns,
            stats.percentiles.p99_ns, stats.percentiles.p999_ns,
            self.bin_size_ns, bins
        )
    }

    /// Reset all histogram data.
    pub fn reset(&self) {
        for bin in &self.bins {
            bin.store(0, Ordering::Relaxed);
        }
        self.overflow.store(0, Ordering::Relaxed);
        self.total.store(0, Ordering::Relaxed);
        self.min_ns.store(u64::MAX, Ordering::Relaxed);
        self.max_observed_ns.store(0, Ordering::Relaxed);
        self.sum_ns.store(0, Ordering::Relaxed);
    }
}

/// Common percentile values.
#[derive(Debug, Clone, Copy)]
pub struct LatencyPercentiles {
    /// 50th percentile (median)
    pub p50_ns: u64,
    /// 90th percentile
    pub p90_ns: u64,
    /// 99th percentile
    pub p99_ns: u64,
    /// 99.9th percentile
    pub p999_ns: u64,
}

impl LatencyPercentiles {
    /// Get p50 in microseconds.
    pub fn p50_us(&self) -> f64 { self.p50_ns as f64 / 1000.0 }
    /// Get p90 in microseconds.
    pub fn p90_us(&self) -> f64 { self.p90_ns as f64 / 1000.0 }
    /// Get p99 in microseconds.
    pub fn p99_us(&self) -> f64 { self.p99_ns as f64 / 1000.0 }
    /// Get p99.9 in microseconds.
    pub fn p999_us(&self) -> f64 { self.p999_ns as f64 / 1000.0 }
}

/// Complete latency statistics.
#[derive(Debug, Clone)]
pub struct LatencyStatistics {
    /// Total sample count
    pub count: u64,
    /// Minimum latency in nanoseconds
    pub min_ns: u64,
    /// Maximum latency in nanoseconds
    pub max_ns: u64,
    /// Mean latency in nanoseconds
    pub mean_ns: u64,
    /// Overflow count
    pub overflow: u64,
    /// Percentile values
    pub percentiles: LatencyPercentiles,
}

impl LatencyStatistics {
    /// Check if latency meets RT requirements.
    ///
    /// # Arguments
    ///
    /// * `max_p99_us` - Maximum acceptable p99 latency in microseconds
    pub fn meets_rt_requirements(&self, max_p99_us: f64) -> bool {
        self.overflow == 0 && self.percentiles.p99_us() <= max_p99_us
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_histogram_basic() {
        let hist = LatencyHistogram::new_us(1000, 10);

        hist.record_ns(50_000);   // 50µs -> bin 5
        hist.record_ns(55_000);   // 55µs -> bin 5
        hist.record_ns(100_000);  // 100µs -> bin 10

        assert_eq!(hist.count(), 3);
        assert_eq!(hist.min_ns(), 50_000);
        assert_eq!(hist.max_ns(), 100_000);
    }

    #[test]
    fn test_histogram_overflow() {
        let hist = LatencyHistogram::new_us(100, 10);

        hist.record_us(50);
        hist.record_us(150);  // overflow
        hist.record_us(200);  // overflow

        assert_eq!(hist.count(), 3);
        assert_eq!(hist.overflow_count(), 2);
    }

    #[test]
    fn test_histogram_percentiles() {
        let hist = LatencyHistogram::new_us(1000, 10);

        // Add 100 samples from 10µs to 1000µs
        for i in 1..=100 {
            hist.record_us(i * 10);
        }

        let pct = hist.percentiles();

        // p50 should be around 500µs (bin 50)
        assert!(pct.p50_ns >= 500_000 && pct.p50_ns <= 510_000);

        // p99 should be around 990µs
        assert!(pct.p99_ns >= 990_000 && pct.p99_ns <= 1_000_000);
    }

    #[test]
    fn test_histogram_mean() {
        let hist = LatencyHistogram::new_us(1000, 10);

        hist.record_us(100);
        hist.record_us(200);
        hist.record_us(300);

        // Mean should be 200µs = 200,000ns
        assert_eq!(hist.mean_ns(), 200_000);
    }

    #[test]
    fn test_histogram_reset() {
        let hist = LatencyHistogram::new_us(1000, 10);

        hist.record_us(100);
        hist.record_us(200);

        hist.reset();

        assert_eq!(hist.count(), 0);
        assert_eq!(hist.min_ns(), 0);
        assert_eq!(hist.overflow_count(), 0);
    }

    #[test]
    fn test_histogram_statistics() {
        let hist = LatencyHistogram::new_us(1000, 10);

        for i in 1..=100 {
            hist.record_us(i);
        }

        let stats = hist.statistics();

        assert_eq!(stats.count, 100);
        assert!(stats.meets_rt_requirements(200.0)); // p99 < 200µs
    }

    #[test]
    fn test_histogram_ascii() {
        let hist = LatencyHistogram::new_us(1000, 100);

        for i in 0..50 {
            hist.record_us(100 + i);  // 100-149µs
        }
        for i in 0..30 {
            hist.record_us(200 + i);  // 200-229µs
        }
        for i in 0..20 {
            hist.record_us(500 + i);  // 500-519µs
        }

        let ascii = hist.ascii_histogram_us(30);
        assert!(ascii.contains("100 samples"));
        assert!(ascii.contains("Percentiles"));
    }

    #[test]
    fn test_histogram_json() {
        let hist = LatencyHistogram::new_us(1000, 100);
        hist.record_us(150);
        hist.record_us(250);

        let json = hist.to_json();
        assert!(json.contains("\"count\":2"));
        assert!(json.contains("\"bins\""));
    }
}
