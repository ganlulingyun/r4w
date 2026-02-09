//! Histogram — Signal amplitude distribution analysis
//!
//! Accumulates sample values into configurable bins to build
//! amplitude histograms. Useful for signal characterization,
//! linearity analysis, ADC assessment, and distribution testing.
//! GNU Radio equivalent: histogram display sink.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::histogram::Histogram;
//!
//! let mut hist = Histogram::new(-1.0, 1.0, 10);
//! hist.accumulate(&[0.0, 0.1, -0.5, 0.9, 0.0]);
//! assert_eq!(hist.total_count(), 5);
//! let bins = hist.counts();
//! assert_eq!(bins.len(), 10);
//! ```

/// Amplitude histogram with configurable range and bins.
#[derive(Debug, Clone)]
pub struct Histogram {
    /// Lower bound of histogram range.
    min: f64,
    /// Upper bound of histogram range.
    max: f64,
    /// Number of bins.
    num_bins: usize,
    /// Bin width.
    bin_width: f64,
    /// Bin counts.
    counts: Vec<u64>,
    /// Count of samples below min.
    underflow: u64,
    /// Count of samples above max.
    overflow: u64,
    /// Total samples processed.
    total: u64,
}

impl Histogram {
    /// Create a histogram with the given range and number of bins.
    pub fn new(min: f64, max: f64, num_bins: usize) -> Self {
        let num_bins = num_bins.max(1);
        let bin_width = (max - min) / num_bins as f64;
        Self {
            min,
            max,
            num_bins,
            bin_width,
            counts: vec![0; num_bins],
            underflow: 0,
            overflow: 0,
            total: 0,
        }
    }

    /// Create a histogram that auto-ranges from data.
    pub fn from_data(data: &[f64], num_bins: usize) -> Self {
        if data.is_empty() {
            return Self::new(0.0, 1.0, num_bins);
        }
        let min = data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let margin = (max - min) * 0.01;
        let mut hist = Self::new(min - margin, max + margin, num_bins);
        hist.accumulate(data);
        hist
    }

    /// Add a single sample.
    #[inline]
    pub fn add(&mut self, value: f64) {
        self.total += 1;
        if value < self.min {
            self.underflow += 1;
        } else if value >= self.max {
            self.overflow += 1;
        } else {
            let bin = ((value - self.min) / self.bin_width) as usize;
            let bin = bin.min(self.num_bins - 1);
            self.counts[bin] += 1;
        }
    }

    /// Accumulate a block of samples.
    pub fn accumulate(&mut self, data: &[f64]) {
        for &v in data {
            self.add(v);
        }
    }

    /// Get bin counts.
    pub fn counts(&self) -> &[u64] {
        &self.counts
    }

    /// Get normalized bin values (probability density).
    pub fn density(&self) -> Vec<f64> {
        if self.total == 0 {
            return vec![0.0; self.num_bins];
        }
        let scale = 1.0 / (self.total as f64 * self.bin_width);
        self.counts.iter().map(|&c| c as f64 * scale).collect()
    }

    /// Get normalized bin values (probability, sums to ~1).
    pub fn normalized(&self) -> Vec<f64> {
        if self.total == 0 {
            return vec![0.0; self.num_bins];
        }
        self.counts
            .iter()
            .map(|&c| c as f64 / self.total as f64)
            .collect()
    }

    /// Get bin centers.
    pub fn bin_centers(&self) -> Vec<f64> {
        (0..self.num_bins)
            .map(|i| self.min + (i as f64 + 0.5) * self.bin_width)
            .collect()
    }

    /// Get bin edges (num_bins + 1 values).
    pub fn bin_edges(&self) -> Vec<f64> {
        (0..=self.num_bins)
            .map(|i| self.min + i as f64 * self.bin_width)
            .collect()
    }

    /// Total count of accumulated samples.
    pub fn total_count(&self) -> u64 {
        self.total
    }

    /// Count of under-range samples.
    pub fn underflow_count(&self) -> u64 {
        self.underflow
    }

    /// Count of over-range samples.
    pub fn overflow_count(&self) -> u64 {
        self.overflow
    }

    /// Number of bins.
    pub fn num_bins(&self) -> usize {
        self.num_bins
    }

    /// Bin width.
    pub fn bin_width(&self) -> f64 {
        self.bin_width
    }

    /// Find the bin with the maximum count (mode).
    pub fn mode_bin(&self) -> usize {
        self.counts
            .iter()
            .enumerate()
            .max_by_key(|(_, &c)| c)
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    /// Estimate the median from the histogram.
    pub fn median(&self) -> f64 {
        let half = self.total / 2;
        let mut cumulative = 0u64;
        for (i, &c) in self.counts.iter().enumerate() {
            cumulative += c;
            if cumulative >= half {
                return self.min + (i as f64 + 0.5) * self.bin_width;
            }
        }
        (self.min + self.max) / 2.0
    }

    /// Estimate mean from the histogram.
    pub fn mean(&self) -> f64 {
        if self.total == 0 {
            return 0.0;
        }
        let centers = self.bin_centers();
        let sum: f64 = self.counts
            .iter()
            .zip(centers.iter())
            .map(|(&c, &center)| c as f64 * center)
            .sum();
        sum / self.total as f64
    }

    /// Reset all counts.
    pub fn clear(&mut self) {
        self.counts.fill(0);
        self.underflow = 0;
        self.overflow = 0;
        self.total = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_histogram() {
        let mut hist = Histogram::new(0.0, 10.0, 10);
        hist.accumulate(&[0.5, 1.5, 2.5, 3.5, 4.5]);
        assert_eq!(hist.total_count(), 5);
        // Each sample lands in a different bin
        for i in 0..5 {
            assert_eq!(hist.counts()[i], 1);
        }
    }

    #[test]
    fn test_underflow_overflow() {
        let mut hist = Histogram::new(0.0, 10.0, 10);
        hist.add(-1.0);
        hist.add(15.0);
        hist.add(10.0); // Equal to max → overflow
        assert_eq!(hist.underflow_count(), 1);
        assert_eq!(hist.overflow_count(), 2);
        assert_eq!(hist.total_count(), 3);
    }

    #[test]
    fn test_normalized() {
        let mut hist = Histogram::new(0.0, 2.0, 2);
        hist.accumulate(&[0.5, 0.5, 0.5, 1.5]);
        let norm = hist.normalized();
        assert!((norm[0] - 0.75).abs() < 1e-10);
        assert!((norm[1] - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_density() {
        let mut hist = Histogram::new(0.0, 1.0, 1);
        hist.accumulate(&[0.1, 0.5, 0.9]);
        let density = hist.density();
        // density = count / (total * bin_width) = 3 / (3 * 1.0) = 1.0
        assert!((density[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_bin_centers() {
        let hist = Histogram::new(0.0, 10.0, 5);
        let centers = hist.bin_centers();
        assert_eq!(centers, vec![1.0, 3.0, 5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_bin_edges() {
        let hist = Histogram::new(0.0, 10.0, 5);
        let edges = hist.bin_edges();
        assert_eq!(edges, vec![0.0, 2.0, 4.0, 6.0, 8.0, 10.0]);
    }

    #[test]
    fn test_mode() {
        let mut hist = Histogram::new(0.0, 10.0, 10);
        hist.accumulate(&[5.1, 5.2, 5.3, 5.4, 1.0, 2.0]);
        assert_eq!(hist.mode_bin(), 5); // Bin 5 has most samples
    }

    #[test]
    fn test_median() {
        let mut hist = Histogram::new(0.0, 10.0, 10);
        hist.accumulate(&[0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5, 9.5]);
        let median = hist.median();
        assert!((median - 4.5).abs() < 1.0); // Should be near center
    }

    #[test]
    fn test_mean() {
        let mut hist = Histogram::new(0.0, 10.0, 10);
        hist.accumulate(&[0.5, 9.5]);
        let mean = hist.mean();
        assert!((mean - 5.0).abs() < 1.0);
    }

    #[test]
    fn test_from_data() {
        let data: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let hist = Histogram::from_data(&data, 10);
        assert_eq!(hist.total_count(), 100);
    }

    #[test]
    fn test_clear() {
        let mut hist = Histogram::new(0.0, 1.0, 10);
        hist.accumulate(&[0.1, 0.2, 0.3]);
        hist.clear();
        assert_eq!(hist.total_count(), 0);
        assert!(hist.counts().iter().all(|&c| c == 0));
    }

    #[test]
    fn test_empty() {
        let hist = Histogram::new(0.0, 1.0, 10);
        assert_eq!(hist.total_count(), 0);
        assert_eq!(hist.mean(), 0.0);
        assert!(hist.normalized().iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_accessors() {
        let hist = Histogram::new(-5.0, 5.0, 20);
        assert_eq!(hist.num_bins(), 20);
        assert!((hist.bin_width() - 0.5).abs() < 1e-10);
    }
}
