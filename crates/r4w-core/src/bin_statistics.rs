//! Bin Statistics — Per-FFT-bin statistical accumulation
//!
//! Collects min, max, mean, and variance for each frequency bin
//! across multiple FFT frames. Useful for spectrum monitoring,
//! noise floor estimation, and occupancy detection.
//! GNU Radio equivalent: `bin_statistics_f`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::bin_statistics::BinStatistics;
//!
//! let mut stats = BinStatistics::new(4); // 4 frequency bins
//! stats.accumulate(&[1.0, 2.0, 3.0, 4.0]);
//! stats.accumulate(&[2.0, 3.0, 4.0, 5.0]);
//! let means = stats.means();
//! assert!((means[0] - 1.5).abs() < 1e-10);
//! assert!((means[3] - 4.5).abs() < 1e-10);
//! ```

/// Per-bin statistical accumulator.
#[derive(Debug, Clone)]
pub struct BinStatistics {
    /// Number of bins.
    num_bins: usize,
    /// Running sum per bin.
    sums: Vec<f64>,
    /// Running sum of squares per bin.
    sum_sq: Vec<f64>,
    /// Minimum per bin.
    mins: Vec<f64>,
    /// Maximum per bin.
    maxs: Vec<f64>,
    /// Number of frames accumulated.
    count: usize,
}

impl BinStatistics {
    /// Create a bin statistics collector.
    pub fn new(num_bins: usize) -> Self {
        Self {
            num_bins,
            sums: vec![0.0; num_bins],
            sum_sq: vec![0.0; num_bins],
            mins: vec![f64::INFINITY; num_bins],
            maxs: vec![f64::NEG_INFINITY; num_bins],
            count: 0,
        }
    }

    /// Accumulate a frame of bin values.
    ///
    /// Frame length must equal num_bins.
    pub fn accumulate(&mut self, frame: &[f64]) {
        assert_eq!(
            frame.len(),
            self.num_bins,
            "frame length must match num_bins"
        );
        self.count += 1;
        for (i, &v) in frame.iter().enumerate() {
            self.sums[i] += v;
            self.sum_sq[i] += v * v;
            if v < self.mins[i] {
                self.mins[i] = v;
            }
            if v > self.maxs[i] {
                self.maxs[i] = v;
            }
        }
    }

    /// Accumulate using max-hold (only update if new value is larger).
    pub fn accumulate_max_hold(&mut self, frame: &[f64]) {
        assert_eq!(frame.len(), self.num_bins);
        self.count += 1;
        for (i, &v) in frame.iter().enumerate() {
            if v > self.maxs[i] {
                self.maxs[i] = v;
            }
            self.sums[i] += v;
        }
    }

    /// Get mean values per bin.
    pub fn means(&self) -> Vec<f64> {
        if self.count == 0 {
            return vec![0.0; self.num_bins];
        }
        let n = self.count as f64;
        self.sums.iter().map(|&s| s / n).collect()
    }

    /// Get variance per bin.
    pub fn variances(&self) -> Vec<f64> {
        if self.count < 2 {
            return vec![0.0; self.num_bins];
        }
        let n = self.count as f64;
        (0..self.num_bins)
            .map(|i| {
                let mean = self.sums[i] / n;
                self.sum_sq[i] / n - mean * mean
            })
            .collect()
    }

    /// Get standard deviation per bin.
    pub fn std_devs(&self) -> Vec<f64> {
        self.variances().into_iter().map(|v| v.max(0.0).sqrt()).collect()
    }

    /// Get minimum values per bin.
    pub fn mins(&self) -> &[f64] {
        &self.mins
    }

    /// Get maximum values per bin.
    pub fn maxs(&self) -> &[f64] {
        &self.maxs
    }

    /// Get number of accumulated frames.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Get number of bins.
    pub fn num_bins(&self) -> usize {
        self.num_bins
    }

    /// Get dynamic range per bin (max - min).
    pub fn dynamic_range(&self) -> Vec<f64> {
        (0..self.num_bins)
            .map(|i| {
                if self.maxs[i] > f64::NEG_INFINITY && self.mins[i] < f64::INFINITY {
                    self.maxs[i] - self.mins[i]
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Reset all statistics.
    pub fn reset(&mut self) {
        self.sums.fill(0.0);
        self.sum_sq.fill(0.0);
        self.mins.fill(f64::INFINITY);
        self.maxs.fill(f64::NEG_INFINITY);
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_accumulate() {
        let mut stats = BinStatistics::new(3);
        stats.accumulate(&[1.0, 2.0, 3.0]);
        stats.accumulate(&[3.0, 4.0, 5.0]);
        let means = stats.means();
        assert!((means[0] - 2.0).abs() < 1e-10);
        assert!((means[1] - 3.0).abs() < 1e-10);
        assert!((means[2] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_min_max() {
        let mut stats = BinStatistics::new(2);
        stats.accumulate(&[5.0, 1.0]);
        stats.accumulate(&[3.0, 7.0]);
        stats.accumulate(&[4.0, 2.0]);
        assert_eq!(stats.mins(), &[3.0, 1.0]);
        assert_eq!(stats.maxs(), &[5.0, 7.0]);
    }

    #[test]
    fn test_variance() {
        let mut stats = BinStatistics::new(1);
        // Values: 2, 4, 6 → mean=4, var = ((4+0+4)/3) = 8/3 ≈ 2.667
        stats.accumulate(&[2.0]);
        stats.accumulate(&[4.0]);
        stats.accumulate(&[6.0]);
        let var = stats.variances();
        assert!(
            (var[0] - 8.0 / 3.0).abs() < 1e-10,
            "variance should be 8/3, got {}",
            var[0]
        );
    }

    #[test]
    fn test_std_dev() {
        let mut stats = BinStatistics::new(1);
        stats.accumulate(&[2.0]);
        stats.accumulate(&[4.0]);
        stats.accumulate(&[6.0]);
        let sd = stats.std_devs();
        let expected = (8.0f64 / 3.0).sqrt();
        assert!((sd[0] - expected).abs() < 1e-10);
    }

    #[test]
    fn test_dynamic_range() {
        let mut stats = BinStatistics::new(2);
        stats.accumulate(&[-10.0, 5.0]);
        stats.accumulate(&[10.0, 15.0]);
        let dr = stats.dynamic_range();
        assert!((dr[0] - 20.0).abs() < 1e-10);
        assert!((dr[1] - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_count() {
        let mut stats = BinStatistics::new(4);
        assert_eq!(stats.count(), 0);
        stats.accumulate(&[0.0; 4]);
        stats.accumulate(&[0.0; 4]);
        assert_eq!(stats.count(), 2);
    }

    #[test]
    fn test_reset() {
        let mut stats = BinStatistics::new(2);
        stats.accumulate(&[1.0, 2.0]);
        stats.reset();
        assert_eq!(stats.count(), 0);
        assert_eq!(stats.means(), vec![0.0, 0.0]);
    }

    #[test]
    fn test_max_hold() {
        let mut stats = BinStatistics::new(3);
        stats.accumulate_max_hold(&[1.0, 5.0, 3.0]);
        stats.accumulate_max_hold(&[4.0, 2.0, 6.0]);
        assert_eq!(stats.maxs(), &[4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_single_frame() {
        let mut stats = BinStatistics::new(2);
        stats.accumulate(&[10.0, 20.0]);
        let means = stats.means();
        assert!((means[0] - 10.0).abs() < 1e-10);
        assert!((means[1] - 20.0).abs() < 1e-10);
        assert_eq!(stats.variances(), vec![0.0, 0.0]); // Need at least 2 for variance
    }

    #[test]
    fn test_empty_means() {
        let stats = BinStatistics::new(3);
        assert_eq!(stats.means(), vec![0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_accessors() {
        let stats = BinStatistics::new(5);
        assert_eq!(stats.num_bins(), 5);
        assert_eq!(stats.count(), 0);
    }

    #[test]
    fn test_many_frames() {
        let mut stats = BinStatistics::new(2);
        for i in 0..1000 {
            stats.accumulate(&[i as f64, (1000 - i) as f64]);
        }
        assert_eq!(stats.count(), 1000);
        let means = stats.means();
        assert!((means[0] - 499.5).abs() < 0.1);
        assert!((means[1] - 500.5).abs() < 0.1);
    }

    #[test]
    fn test_negative_values() {
        let mut stats = BinStatistics::new(1);
        stats.accumulate(&[-5.0]);
        stats.accumulate(&[-3.0]);
        assert_eq!(stats.mins(), &[-5.0]);
        assert_eq!(stats.maxs(), &[-3.0]);
        let means = stats.means();
        assert!((means[0] - (-4.0)).abs() < 1e-10);
    }
}
