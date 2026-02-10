//! Timing jitter measurement and decomposition for signal quality analysis.
//!
//! This module provides tools to measure and decompose timing jitter into its
//! constituent components: period jitter, cycle-to-cycle jitter, random (Gaussian)
//! jitter, deterministic (bounded) jitter, and periodic jitter. It supports both
//! edge-timestamp and direct-interval input modes, and offers histogram, TIE
//! (Time Interval Error) histogram, Allan variance, and phase noise estimation.
//!
//! # Example
//!
//! ```
//! use r4w_core::jitter_analyzer::{JitterAnalyzer, JitterMetrics};
//!
//! let mut analyzer = JitterAnalyzer::new(1.0); // nominal period = 1.0
//! analyzer.add_intervals(&[1.01, 0.99, 1.02, 0.98, 1.00, 1.01, 0.99, 1.00]);
//! let metrics = analyzer.compute_metrics().unwrap();
//! assert!(metrics.rms_jitter > 0.0);
//! assert!(metrics.peak_to_peak_jitter > 0.0);
//! ```

use std::f64::consts::PI;

/// Accumulated jitter statistics computed from timing measurements.
#[derive(Debug, Clone)]
pub struct JitterMetrics {
    /// RMS (root-mean-square) jitter — standard deviation of period deviations.
    pub rms_jitter: f64,
    /// Peak-to-peak jitter — max minus min observed period.
    pub peak_to_peak_jitter: f64,
    /// Period jitter — standard deviation of all measured periods around the mean.
    pub period_jitter: f64,
    /// Cycle-to-cycle jitter — RMS of successive period differences.
    pub cycle_to_cycle_jitter: f64,
    /// Mean measured period.
    pub mean_period: f64,
    /// Number of intervals analysed.
    pub count: usize,
}

/// Decomposition of jitter into deterministic and random components.
#[derive(Debug, Clone)]
pub struct JitterDecomposition {
    /// Deterministic jitter — bounded component (peak-to-peak of the
    /// distribution tails beyond 3-sigma are excluded; estimated as
    /// total peak-to-peak minus the random contribution at a given BER).
    pub deterministic_jitter: f64,
    /// Random jitter RMS — unbounded Gaussian component (equal to
    /// the standard deviation of the timing deviations).
    pub random_jitter: f64,
    /// Periodic jitter — peak-to-peak of the periodic component
    /// estimated from the dominant spectral line in the jitter spectrum.
    pub periodic_jitter: f64,
}

/// A single histogram bin.
#[derive(Debug, Clone)]
pub struct HistogramBin {
    /// Lower edge of the bin (inclusive).
    pub lower: f64,
    /// Upper edge of the bin (exclusive, except for the last bin).
    pub upper: f64,
    /// Number of samples falling in this bin.
    pub count: usize,
}

/// Analyzes timing jitter from edge timestamps or measured intervals.
///
/// Feed data via [`add_edge_times`](Self::add_edge_times) or
/// [`add_intervals`](Self::add_intervals), then call
/// [`compute_metrics`](Self::compute_metrics) or
/// [`decompose`](Self::decompose) to obtain results.
#[derive(Debug, Clone)]
pub struct JitterAnalyzer {
    /// Nominal (expected) period.
    nominal_period: f64,
    /// Collected period intervals.
    intervals: Vec<f64>,
}

impl JitterAnalyzer {
    /// Create a new analyzer with the given nominal period.
    ///
    /// The nominal period is used as the reference for TIE computation and
    /// phase noise conversion.
    pub fn new(nominal_period: f64) -> Self {
        assert!(nominal_period > 0.0, "nominal_period must be positive");
        Self {
            nominal_period,
            intervals: Vec::new(),
        }
    }

    /// Feed zero-crossing or rising-edge timestamps.
    ///
    /// Consecutive differences are computed and stored as intervals.
    /// Timestamps must be monotonically increasing.
    pub fn add_edge_times(&mut self, times: &[f64]) {
        if times.len() < 2 {
            return;
        }
        for pair in times.windows(2) {
            let dt = pair[1] - pair[0];
            debug_assert!(dt > 0.0, "timestamps must be monotonically increasing");
            self.intervals.push(dt);
        }
    }

    /// Feed measured period intervals directly.
    pub fn add_intervals(&mut self, intervals: &[f64]) {
        self.intervals.extend_from_slice(intervals);
    }

    /// Return a reference to the collected intervals.
    pub fn intervals(&self) -> &[f64] {
        &self.intervals
    }

    /// Number of intervals currently stored.
    pub fn len(&self) -> usize {
        self.intervals.len()
    }

    /// Whether no intervals have been recorded.
    pub fn is_empty(&self) -> bool {
        self.intervals.is_empty()
    }

    /// Reset all stored data.
    pub fn clear(&mut self) {
        self.intervals.clear();
    }

    // ── helpers ──────────────────────────────────────────────────────

    fn mean(data: &[f64]) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        data.iter().sum::<f64>() / data.len() as f64
    }

    fn variance(data: &[f64], mean: f64) -> f64 {
        if data.len() < 2 {
            return 0.0;
        }
        let sum_sq: f64 = data.iter().map(|&x| (x - mean).powi(2)).sum();
        sum_sq / (data.len() - 1) as f64
    }

    // ── primary API ─────────────────────────────────────────────────

    /// Compute jitter metrics from the stored intervals.
    ///
    /// Returns `None` if fewer than 2 intervals have been recorded.
    pub fn compute_metrics(&self) -> Option<JitterMetrics> {
        if self.intervals.len() < 2 {
            return None;
        }

        let mean_period = Self::mean(&self.intervals);
        let var = Self::variance(&self.intervals, mean_period);
        let rms_jitter = var.sqrt();

        let min = self
            .intervals
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let max = self
            .intervals
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let peak_to_peak_jitter = max - min;

        // Period jitter is the same as RMS jitter (std dev of periods).
        let period_jitter = rms_jitter;

        // Cycle-to-cycle: RMS of consecutive period differences.
        let diffs: Vec<f64> = self
            .intervals
            .windows(2)
            .map(|w| w[1] - w[0])
            .collect();
        let c2c_rms_sq: f64 = diffs.iter().map(|&d| d * d).sum::<f64>() / diffs.len() as f64;
        let cycle_to_cycle_jitter = c2c_rms_sq.sqrt();
        Some(JitterMetrics {
            rms_jitter,
            peak_to_peak_jitter,
            period_jitter,
            cycle_to_cycle_jitter,
            mean_period,
            count: self.intervals.len(),
        })
    }

    /// Decompose jitter into deterministic and random components.
    ///
    /// Uses the dual-Dirac model: total jitter at a given BER is
    /// `DJ + 2 * N * RJ`, where `N` is the tail quantile (e.g. 14.069
    /// for BER = 1e-12).  Random jitter is estimated as the standard
    /// deviation of the intervals.  Periodic jitter is estimated from
    /// the peak magnitude of the DFT of the TIE sequence.
    ///
    /// Returns `None` if fewer than 2 intervals have been stored.
    pub fn decompose(&self) -> Option<JitterDecomposition> {
        let metrics = self.compute_metrics()?;

        let random_jitter = metrics.rms_jitter;

        // Periodic jitter: magnitude of dominant DFT bin (excluding DC)
        // of the Time-Interval-Error sequence.
        let tie = self.tie_values();
        let periodic_jitter = Self::estimate_periodic_jitter(&tie);

        // Deterministic jitter: remove random contribution from peak-to-peak.
        // DJ ≈ PP - 2 * N_sigma * RJ, clamped to >= 0.  Use N_sigma = 6
        // (± 3σ) as a reasonable default.
        let n_sigma = 6.0;
        let deterministic_jitter =
            (metrics.peak_to_peak_jitter - n_sigma * random_jitter).max(0.0);

        Some(JitterDecomposition {
            deterministic_jitter,
            random_jitter,
            periodic_jitter,
        })
    }

    // ── TIE helpers ─────────────────────────────────────────────────

    /// Compute Time Interval Error values (cumulative phase error).
    ///
    /// TIE[k] = (sum of first k+1 intervals) - (k+1) * nominal_period
    fn tie_values(&self) -> Vec<f64> {
        let mut tie = Vec::with_capacity(self.intervals.len());
        let mut cumulative = 0.0;
        for (i, &interval) in self.intervals.iter().enumerate() {
            cumulative += interval;
            tie.push(cumulative - (i + 1) as f64 * self.nominal_period);
        }
        tie
    }

    /// Estimate peak-to-peak periodic jitter from the DFT of the TIE sequence.
    fn estimate_periodic_jitter(tie: &[f64]) -> f64 {
        if tie.len() < 4 {
            return 0.0;
        }
        let n = tie.len();
        let mut max_mag = 0.0_f64;

        // Simple DFT over positive frequencies (skip DC bin k=0).
        for k in 1..n / 2 {
            let mut re = 0.0;
            let mut im = 0.0;
            for (j, &val) in tie.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                re += val * angle.cos();
                im += val * angle.sin();
            }
            let mag = (re * re + im * im).sqrt() / n as f64;
            if mag > max_mag {
                max_mag = mag;
            }
        }
        // Peak-to-peak of a sinusoid with amplitude A is 2*A.
        2.0 * max_mag
    }

    // ── histograms ──────────────────────────────────────────────────

    /// Build a histogram of measured period intervals.
    ///
    /// `num_bins` specifies the number of equally-spaced bins spanning
    /// the range `[min_interval, max_interval]`.
    ///
    /// Returns an empty vec if there are no intervals or `num_bins` is 0.
    pub fn histogram(&self, num_bins: usize) -> Vec<HistogramBin> {
        Self::build_histogram(&self.intervals, num_bins)
    }

    /// Build a Time Interval Error histogram.
    ///
    /// TIE = cumulative_time - k * nominal_period, where k is the edge index.
    pub fn tie_histogram(&self, num_bins: usize) -> Vec<HistogramBin> {
        let tie = self.tie_values();
        Self::build_histogram(&tie, num_bins)
    }

    fn build_histogram(data: &[f64], num_bins: usize) -> Vec<HistogramBin> {
        if data.is_empty() || num_bins == 0 {
            return Vec::new();
        }

        let min_val = data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        // Avoid zero-width range.
        let range = if (max_val - min_val).abs() < f64::EPSILON {
            1.0
        } else {
            max_val - min_val
        };

        let bin_width = range / num_bins as f64;
        let mut bins: Vec<HistogramBin> = (0..num_bins)
            .map(|i| HistogramBin {
                lower: min_val + i as f64 * bin_width,
                upper: min_val + (i + 1) as f64 * bin_width,
                count: 0,
            })
            .collect();

        for &val in data {
            let mut idx = ((val - min_val) / bin_width).floor() as usize;
            if idx >= num_bins {
                idx = num_bins - 1;
            }
            bins[idx].count += 1;
        }

        bins
    }

    // ── Allan variance ──────────────────────────────────────────────

    /// Compute the overlapping Allan variance (AVAR) for a range of
    /// averaging factors tau = 1, 2, 4, ..., up to `max_tau`.
    ///
    /// Returns pairs of `(tau, allan_variance)`.  The input data are the
    /// fractional frequency deviations y_i = (T_i - T_nom) / T_nom.
    ///
    /// Returns an empty vec if there are fewer than 3 intervals.
    pub fn allan_variance(&self, max_tau: usize) -> Vec<(usize, f64)> {
        if self.intervals.len() < 3 {
            return Vec::new();
        }

        // Fractional frequency deviations.
        let y: Vec<f64> = self
            .intervals
            .iter()
            .map(|&t| (t - self.nominal_period) / self.nominal_period)
            .collect();
        let n = y.len();

        let mut results = Vec::new();
        let mut tau = 1_usize;
        while tau <= max_tau && tau <= n / 2 {
            // Overlapping Allan variance for averaging factor `tau`.
            let num_avg = n - tau + 1;
            if num_avg < 2 {
                break;
            }
            let avg: Vec<f64> = (0..num_avg)
                .map(|i| {
                    let slice = &y[i..i + tau];
                    slice.iter().sum::<f64>() / tau as f64
                })
                .collect();
            let mut sum = 0.0;
            let mut count = 0_usize;
            for i in 0..avg.len() - 1 {
                let diff = avg[i + 1] - avg[i];
                sum += diff * diff;
                count += 1;
            }
            if count > 0 {
                let avar = sum / (2.0 * count as f64);
                results.push((tau, avar));
            }
            tau *= 2;
        }
        results
    }

    // ── Phase noise from jitter ─────────────────────────────────────

    /// Convert RMS jitter to single-sideband phase noise at an offset
    /// frequency `f_offset` from the carrier.
    ///
    /// Uses the relation:
    ///   L(f) = 10 * log10( (2 * pi * f_carrier * sigma_t)^2 / (2 * BW) )
    ///
    /// where `f_carrier = 1 / nominal_period`, `sigma_t` is the RMS jitter,
    /// and `BW` is the measurement bandwidth (here assumed 1 Hz for
    /// dBc/Hz).  For a flat phase-noise floor model (white phase noise):
    ///
    ///   L(f_offset) = 10 * log10( 2 * (pi * f_carrier * sigma_t)^2 )
    ///
    /// Returns `None` if metrics cannot be computed.
    pub fn phase_noise_from_jitter(&self, _f_offset: f64) -> Option<f64> {
        let metrics = self.compute_metrics()?;
        let f_carrier = 1.0 / self.nominal_period;
        let sigma = metrics.rms_jitter;
        // White-noise floor model
        let l_f = 10.0 * (2.0 * (PI * f_carrier * sigma).powi(2)).log10();
        Some(l_f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build an analyzer with known intervals.
    fn make_analyzer(intervals: &[f64], nominal: f64) -> JitterAnalyzer {
        let mut a = JitterAnalyzer::new(nominal);
        a.add_intervals(intervals);
        a
    }

    #[test]
    fn test_basic_metrics() {
        let a = make_analyzer(&[1.01, 0.99, 1.02, 0.98, 1.00, 1.01, 0.99, 1.00], 1.0);
        let m = a.compute_metrics().unwrap();
        assert_eq!(m.count, 8);
        assert!((m.mean_period - 1.0).abs() < 0.01);
        assert!(m.rms_jitter > 0.0);
        assert!(m.peak_to_peak_jitter > 0.0);
        assert!(m.cycle_to_cycle_jitter > 0.0);
    }

    #[test]
    fn test_no_jitter() {
        // Perfectly uniform intervals should have near-zero jitter.
        let a = make_analyzer(&[1.0; 100], 1.0);
        let m = a.compute_metrics().unwrap();
        assert!(m.rms_jitter < 1e-15);
        assert!(m.peak_to_peak_jitter < 1e-15);
    }

    #[test]
    fn test_edge_times_to_intervals() {
        let mut a = JitterAnalyzer::new(1.0);
        a.add_edge_times(&[0.0, 1.01, 2.00, 3.02]);
        assert_eq!(a.len(), 3);
        let intervals = a.intervals();
        assert!((intervals[0] - 1.01).abs() < 1e-12);
        assert!((intervals[1] - 0.99).abs() < 1e-12);
        assert!((intervals[2] - 1.02).abs() < 1e-12);
    }

    #[test]
    fn test_insufficient_data() {
        let a = make_analyzer(&[1.0], 1.0);
        assert!(a.compute_metrics().is_none());
        assert!(a.decompose().is_none());
    }

    #[test]
    fn test_decompose_random_only() {
        // Purely random-looking intervals (no strong periodic component).
        let intervals: Vec<f64> = (0..200)
            .map(|i| {
                // Simple deterministic pseudo-random via sine
                1.0 + 0.001 * ((i as f64 * 0.7).sin() + (i as f64 * 1.3).cos())
            })
            .collect();
        let a = make_analyzer(&intervals, 1.0);
        let d = a.decompose().unwrap();
        assert!(d.random_jitter > 0.0);
        // Periodic jitter should be small but nonzero since we used sine.
        assert!(d.periodic_jitter >= 0.0);
    }

    #[test]
    fn test_histogram_bins() {
        let a = make_analyzer(&[0.9, 1.0, 1.0, 1.1, 1.1, 1.1], 1.0);
        let hist = a.histogram(3);
        assert_eq!(hist.len(), 3);
        let total: usize = hist.iter().map(|b| b.count).sum();
        assert_eq!(total, 6);
        // First bin should contain 0.9, last bin should contain 1.1s.
        assert!(hist[0].count >= 1);
        assert!(hist[2].count >= 1);
    }

    #[test]
    fn test_tie_histogram() {
        let a = make_analyzer(&[1.01, 0.99, 1.01, 0.99], 1.0);
        let hist = a.tie_histogram(4);
        assert_eq!(hist.len(), 4);
        let total: usize = hist.iter().map(|b| b.count).sum();
        assert_eq!(total, 4);
    }

    #[test]
    fn test_allan_variance_constant() {
        // Constant-frequency signal: AVAR should be near zero.
        let a = make_analyzer(&[1.0; 64], 1.0);
        let av = a.allan_variance(16);
        for &(_tau, var) in &av {
            assert!(var < 1e-20, "AVAR should be ~0 for constant intervals");
        }
    }

    #[test]
    fn test_allan_variance_structure() {
        let intervals: Vec<f64> = (0..128)
            .map(|i| 1.0 + 0.001 * (i as f64 * 0.1).sin())
            .collect();
        let a = make_analyzer(&intervals, 1.0);
        let av = a.allan_variance(32);
        assert!(!av.is_empty());
        // Tau values should be powers of 2.
        for (i, &(tau, _)) in av.iter().enumerate() {
            assert_eq!(tau, 1 << i);
        }
    }

    #[test]
    fn test_phase_noise_from_jitter() {
        let a = make_analyzer(&[1.01, 0.99, 1.02, 0.98, 1.00, 1.01, 0.99, 1.00], 1.0);
        let pn = a.phase_noise_from_jitter(1000.0).unwrap();
        // Phase noise should be a finite negative number (dBc/Hz).
        assert!(pn.is_finite());
        // For small jitter relative to the period, phase noise should be negative.
        assert!(pn < 0.0, "expected negative dBc/Hz for small jitter, got {pn}");
    }

    #[test]
    fn test_clear_and_empty() {
        let mut a = JitterAnalyzer::new(1.0);
        assert!(a.is_empty());
        a.add_intervals(&[1.0, 1.0, 1.0]);
        assert!(!a.is_empty());
        assert_eq!(a.len(), 3);
        a.clear();
        assert!(a.is_empty());
        assert_eq!(a.len(), 0);
    }

    #[test]
    fn test_peak_to_peak_exact() {
        let a = make_analyzer(&[0.95, 1.05, 1.0, 1.0, 1.0], 1.0);
        let m = a.compute_metrics().unwrap();
        assert!((m.peak_to_peak_jitter - 0.1).abs() < 1e-12);
    }

    #[test]
    fn test_cycle_to_cycle_large_swing() {
        // Alternating fast/slow periods should give large C2C jitter.
        let a = make_analyzer(&[0.9, 1.1, 0.9, 1.1, 0.9, 1.1], 1.0);
        let m = a.compute_metrics().unwrap();
        // Each consecutive difference is +/-0.2, so C2C RMS should be approx 0.2.
        assert!((m.cycle_to_cycle_jitter - 0.2).abs() < 0.01);
    }

    #[test]
    fn test_histogram_single_value() {
        let a = make_analyzer(&[1.0, 1.0, 1.0], 1.0);
        let hist = a.histogram(5);
        assert_eq!(hist.len(), 5);
        // All samples land in one bin.
        let total: usize = hist.iter().map(|b| b.count).sum();
        assert_eq!(total, 3);
    }

    #[test]
    fn test_empty_histogram() {
        let a = JitterAnalyzer::new(1.0);
        let hist = a.histogram(10);
        assert!(hist.is_empty());
    }
}
