//! # Time-Series Changepoint Detection
//!
//! CUSUM and kernel-based segmentation for detecting anomalous transitions
//! in streaming signal properties. Supports both real-valued and complex
//! `(f64, f64)` signal statistics.
//!
//! ## Algorithms
//!
//! - **CUSUM** (Cumulative Sum): Page's test for mean shift detection with
//!   configurable drift and threshold parameters.
//! - **Two-sided CUSUM**: Detects both upward and downward shifts simultaneously.
//! - **Binary Segmentation**: Offline recursive algorithm for multiple changepoint
//!   detection using CUSUM cost minimization.
//! - **Sliding Window**: Online comparison of mean/variance in adjacent windows.
//! - **Bayesian Probability**: Simplified posterior changepoint probability
//!   estimation using a Gaussian likelihood model.
//!
//! ## Example
//!
//! ```
//! use r4w_core::time_series_changepoint_detector::{ChangePointDetector, CusumParams};
//!
//! // Create a detector with custom CUSUM parameters
//! let params = CusumParams { drift: 0.5, threshold: 4.0 };
//! let mut detector = ChangePointDetector::new(params);
//!
//! // Feed samples from a signal with a mean shift at index 50
//! let mut samples: Vec<f64> = vec![0.0; 50];
//! samples.extend(vec![3.0; 50]);
//!
//! let changepoints = detector.detect_cusum(&samples);
//! // Should detect a changepoint near index 50
//! assert!(!changepoints.is_empty());
//! assert!((changepoints[0] as i64 - 50).unsigned_abs() <= 10);
//! ```

// ---------------------------------------------------------------------------
// Complex helper type and utilities
// ---------------------------------------------------------------------------

/// Complex number represented as `(re, im)`.
pub type Complex = (f64, f64);

/// Compute the magnitude squared of a complex number.
#[inline]
fn complex_mag_sq(c: Complex) -> f64 {
    c.0 * c.0 + c.1 * c.1
}

/// Compute the magnitude of a complex number.
#[inline]
fn complex_mag(c: Complex) -> f64 {
    complex_mag_sq(c).sqrt()
}

// ---------------------------------------------------------------------------
// CUSUM parameters
// ---------------------------------------------------------------------------

/// Configuration for the CUSUM (Cumulative Sum) algorithm.
///
/// * `drift` – allowance parameter (ν). A shift smaller than `drift` will be
///   absorbed; only shifts exceeding `drift` accumulate.
/// * `threshold` – decision threshold (h). When the cumulative sum exceeds
///   `threshold`, a changepoint is signalled.
#[derive(Debug, Clone, Copy)]
pub struct CusumParams {
    /// Allowance / drift parameter (ν ≥ 0).
    pub drift: f64,
    /// Decision threshold (h > 0).
    pub threshold: f64,
}

impl Default for CusumParams {
    fn default() -> Self {
        Self {
            drift: 0.5,
            threshold: 5.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Sliding-window parameters
// ---------------------------------------------------------------------------

/// Configuration for the sliding-window mean/variance comparison detector.
#[derive(Debug, Clone, Copy)]
pub struct SlidingWindowParams {
    /// Number of samples in each half-window.
    pub window_size: usize,
    /// Threshold on the test statistic to declare a changepoint.
    pub threshold: f64,
}

impl Default for SlidingWindowParams {
    fn default() -> Self {
        Self {
            window_size: 30,
            threshold: 3.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Binary segmentation parameters
// ---------------------------------------------------------------------------

/// Configuration for offline binary segmentation.
#[derive(Debug, Clone, Copy)]
pub struct BinarySegmentationParams {
    /// Minimum segment length (prevents over-segmentation).
    pub min_segment_len: usize,
    /// Minimum CUSUM statistic to accept a candidate changepoint.
    pub threshold: f64,
}

impl Default for BinarySegmentationParams {
    fn default() -> Self {
        Self {
            min_segment_len: 10,
            threshold: 4.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Bayesian parameters
// ---------------------------------------------------------------------------

/// Configuration for simplified Bayesian changepoint probability estimation.
#[derive(Debug, Clone, Copy)]
pub struct BayesianParams {
    /// Prior probability of a changepoint at any given sample.
    pub prior: f64,
    /// Assumed known variance of the signal (σ²).
    pub variance: f64,
}

impl Default for BayesianParams {
    fn default() -> Self {
        Self {
            prior: 0.01,
            variance: 1.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Detected changepoint
// ---------------------------------------------------------------------------

/// A detected changepoint with metadata.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChangePoint {
    /// Sample index of the changepoint.
    pub index: usize,
    /// Magnitude of the test statistic at the changepoint.
    pub statistic: f64,
    /// Optional significance / p-value (only populated by some methods).
    pub significance: Option<f64>,
}

// ---------------------------------------------------------------------------
// Run-length segment
// ---------------------------------------------------------------------------

/// A contiguous segment produced by run-length encoding of changepoints.
#[derive(Debug, Clone, PartialEq)]
pub struct Segment {
    /// Start index (inclusive).
    pub start: usize,
    /// End index (exclusive).
    pub end: usize,
    /// Mean of the segment.
    pub mean: f64,
    /// Variance of the segment.
    pub variance: f64,
}

// ---------------------------------------------------------------------------
// Main detector struct
// ---------------------------------------------------------------------------

/// Configurable changepoint detector supporting multiple algorithms.
///
/// Operates on `&[f64]` for real signals or `&[Complex]` for complex signals.
#[derive(Debug, Clone)]
pub struct ChangePointDetector {
    /// CUSUM parameters.
    pub cusum: CusumParams,
    /// Sliding-window parameters.
    pub sliding_window: SlidingWindowParams,
    /// Binary-segmentation parameters.
    pub binary_segmentation: BinarySegmentationParams,
    /// Bayesian estimation parameters.
    pub bayesian: BayesianParams,

    // Internal online state for streaming CUSUM
    cusum_pos: f64,
    cusum_neg: f64,
    cusum_mean: f64,
    cusum_n: usize,
}

impl ChangePointDetector {
    /// Create a new detector with the given CUSUM parameters and defaults
    /// for all other algorithm configurations.
    pub fn new(cusum: CusumParams) -> Self {
        Self {
            cusum,
            sliding_window: SlidingWindowParams::default(),
            binary_segmentation: BinarySegmentationParams::default(),
            bayesian: BayesianParams::default(),
            cusum_pos: 0.0,
            cusum_neg: 0.0,
            cusum_mean: 0.0,
            cusum_n: 0,
        }
    }

    /// Create a detector with all default parameters.
    pub fn with_defaults() -> Self {
        Self::new(CusumParams::default())
    }

    /// Reset the internal streaming CUSUM state.
    pub fn reset(&mut self) {
        self.cusum_pos = 0.0;
        self.cusum_neg = 0.0;
        self.cusum_mean = 0.0;
        self.cusum_n = 0;
    }

    // -----------------------------------------------------------------------
    // CUSUM (batch, one-sided upward)
    // -----------------------------------------------------------------------

    /// Detect upward mean-shift changepoints using Page's CUSUM test.
    ///
    /// Returns the sample indices where the cumulative sum exceeded the
    /// threshold. After each detection the accumulator is reset.
    pub fn detect_cusum(&self, samples: &[f64]) -> Vec<usize> {
        let mean = arithmetic_mean(samples);
        let mut s_pos = 0.0_f64;
        let mut changepoints = Vec::new();

        for (i, &x) in samples.iter().enumerate() {
            s_pos = (s_pos + (x - mean) - self.cusum.drift).max(0.0);
            if s_pos > self.cusum.threshold {
                changepoints.push(i);
                s_pos = 0.0;
            }
        }
        changepoints
    }

    // -----------------------------------------------------------------------
    // Two-sided CUSUM (batch)
    // -----------------------------------------------------------------------

    /// Detect both upward and downward mean shifts.
    ///
    /// Returns detected changepoint indices.
    pub fn detect_two_sided_cusum(&self, samples: &[f64]) -> Vec<usize> {
        let mean = arithmetic_mean(samples);
        let mut s_pos = 0.0_f64;
        let mut s_neg = 0.0_f64;
        let mut changepoints = Vec::new();

        for (i, &x) in samples.iter().enumerate() {
            s_pos = (s_pos + (x - mean) - self.cusum.drift).max(0.0);
            s_neg = (s_neg - (x - mean) - self.cusum.drift).max(0.0);
            if s_pos > self.cusum.threshold || s_neg > self.cusum.threshold {
                changepoints.push(i);
                s_pos = 0.0;
                s_neg = 0.0;
            }
        }
        changepoints
    }

    // -----------------------------------------------------------------------
    // Streaming (online) CUSUM – one sample at a time
    // -----------------------------------------------------------------------

    /// Feed a single sample into the online two-sided CUSUM detector.
    ///
    /// The running mean is updated incrementally. Returns `Some(statistic)`
    /// if a changepoint is detected on this sample, `None` otherwise.
    pub fn feed_sample(&mut self, x: f64) -> Option<f64> {
        // Update running mean
        self.cusum_n += 1;
        self.cusum_mean += (x - self.cusum_mean) / self.cusum_n as f64;

        self.cusum_pos = (self.cusum_pos + (x - self.cusum_mean) - self.cusum.drift).max(0.0);
        self.cusum_neg = (self.cusum_neg - (x - self.cusum_mean) - self.cusum.drift).max(0.0);

        let stat = self.cusum_pos.max(self.cusum_neg);
        if stat > self.cusum.threshold {
            self.cusum_pos = 0.0;
            self.cusum_neg = 0.0;
            Some(stat)
        } else {
            None
        }
    }

    // -----------------------------------------------------------------------
    // Sliding-window mean/variance comparison
    // -----------------------------------------------------------------------

    /// Detect changepoints by comparing mean and variance in adjacent windows.
    ///
    /// At each candidate position `t`, a test statistic is computed from the
    /// difference of means normalised by pooled variance. Peaks above
    /// `self.sliding_window.threshold` are reported.
    pub fn detect_sliding_window(&self, samples: &[f64]) -> Vec<ChangePoint> {
        let w = self.sliding_window.window_size;
        let n = samples.len();
        if n < 2 * w {
            return Vec::new();
        }

        let mut results = Vec::new();
        for t in w..(n - w) {
            let left = &samples[t - w..t];
            let right = &samples[t..t + w];

            let mean_l = arithmetic_mean(left);
            let mean_r = arithmetic_mean(right);
            let var_l = variance(left, mean_l);
            let var_r = variance(right, mean_r);

            let pooled = ((var_l + var_r) / 2.0).max(1e-12);
            let stat = (mean_r - mean_l).abs() / pooled.sqrt();

            if stat > self.sliding_window.threshold {
                results.push(ChangePoint {
                    index: t,
                    statistic: stat,
                    significance: None,
                });
            }
        }

        // Collapse runs of consecutive detections to the peak
        collapse_detections(results)
    }

    // -----------------------------------------------------------------------
    // Binary segmentation (offline, multiple changepoints)
    // -----------------------------------------------------------------------

    /// Recursively find multiple changepoints using binary segmentation with
    /// a CUSUM-like cost function.
    pub fn detect_binary_segmentation(&self, samples: &[f64]) -> Vec<ChangePoint> {
        let mut changepoints = Vec::new();
        self.binseg_recurse(samples, 0, samples.len(), &mut changepoints);
        changepoints.sort_by_key(|cp| cp.index);
        changepoints
    }

    fn binseg_recurse(
        &self,
        samples: &[f64],
        start: usize,
        end: usize,
        out: &mut Vec<ChangePoint>,
    ) {
        let len = end - start;
        let min_seg = self.binary_segmentation.min_segment_len;
        if len < 2 * min_seg {
            return;
        }

        let seg = &samples[start..end];
        let overall_mean = arithmetic_mean(seg);

        // Compute CUSUM statistic at each candidate split
        let mut best_stat = 0.0_f64;
        let mut best_idx = 0_usize;

        for k in min_seg..(len - min_seg) {
            let left = &seg[..k];
            let right = &seg[k..];
            let mean_l = arithmetic_mean(left);
            let mean_r = arithmetic_mean(right);

            // Weighted squared deviation of segment means from overall mean
            let stat = (k as f64) * (mean_l - overall_mean).powi(2)
                + ((len - k) as f64) * (mean_r - overall_mean).powi(2);
            if stat > best_stat {
                best_stat = stat;
                best_idx = k;
            }
        }

        let normalised = best_stat / (len as f64);
        if normalised > self.binary_segmentation.threshold {
            out.push(ChangePoint {
                index: start + best_idx,
                statistic: normalised,
                significance: None,
            });
            self.binseg_recurse(samples, start, start + best_idx, out);
            self.binseg_recurse(samples, start + best_idx, end, out);
        }
    }

    // -----------------------------------------------------------------------
    // Bayesian changepoint probability
    // -----------------------------------------------------------------------

    /// Compute a simplified posterior probability of a changepoint at each
    /// sample position.
    ///
    /// Uses a Gaussian likelihood model with known variance. Returns a
    /// vector of `(index, probability)` for positions where the posterior
    /// exceeds 0.5.
    pub fn bayesian_changepoint_probability(&self, samples: &[f64]) -> Vec<(usize, f64)> {
        let n = samples.len();
        if n < 4 {
            return Vec::new();
        }

        let sigma2 = self.bayesian.variance;
        let prior = self.bayesian.prior;
        let log_prior_ratio = (prior / (1.0 - prior)).ln();

        let mut results = Vec::new();

        for t in 2..(n - 1) {
            let left = &samples[..t];
            let right = &samples[t..];

            let mean_l = arithmetic_mean(left);
            let mean_r = arithmetic_mean(right);

            // Log-likelihood ratio for a mean shift at position t
            let ll_left: f64 = left.iter().map(|&x| -(x - mean_l).powi(2) / (2.0 * sigma2)).sum();
            let ll_right: f64 = right
                .iter()
                .map(|&x| -(x - mean_r).powi(2) / (2.0 * sigma2))
                .sum();

            let overall_mean = arithmetic_mean(samples);
            let ll_null: f64 = samples
                .iter()
                .map(|&x| -(x - overall_mean).powi(2) / (2.0 * sigma2))
                .sum();

            let log_bf = (ll_left + ll_right) - ll_null;
            let log_posterior_odds = log_bf + log_prior_ratio;
            let posterior = 1.0 / (1.0 + (-log_posterior_odds).exp());

            if posterior > 0.5 {
                results.push((t, posterior));
            }
        }
        results
    }

    // -----------------------------------------------------------------------
    // Run-length encoding of segments
    // -----------------------------------------------------------------------

    /// Given a set of changepoint indices, divide the signal into segments
    /// and compute statistics for each.
    pub fn run_length_encode(&self, samples: &[f64], changepoints: &[usize]) -> Vec<Segment> {
        if samples.is_empty() {
            return Vec::new();
        }

        let mut boundaries: Vec<usize> = vec![0];
        for &cp in changepoints {
            if cp > 0 && cp < samples.len() && !boundaries.contains(&cp) {
                boundaries.push(cp);
            }
        }
        boundaries.push(samples.len());
        boundaries.sort_unstable();

        let mut segments = Vec::new();
        for w in boundaries.windows(2) {
            let (start, end) = (w[0], w[1]);
            if start >= end {
                continue;
            }
            let seg = &samples[start..end];
            let mean = arithmetic_mean(seg);
            let var = variance(seg, mean);
            segments.push(Segment {
                start,
                end,
                mean,
                variance: var,
            });
        }
        segments
    }

    // -----------------------------------------------------------------------
    // Changepoint significance testing
    // -----------------------------------------------------------------------

    /// Estimate the significance of a candidate changepoint by comparing
    /// the CUSUM statistic against a permutation-derived null distribution
    /// (approximated analytically with a log-correction).
    ///
    /// Returns the changepoint with a populated `significance` field
    /// representing an approximate p-value.
    pub fn significance_test(
        &self,
        samples: &[f64],
        candidate: usize,
    ) -> ChangePoint {
        let n = samples.len();
        assert!(candidate > 0 && candidate < n, "candidate out of range");

        let left = &samples[..candidate];
        let right = &samples[candidate..];
        let mean_l = arithmetic_mean(left);
        let mean_r = arithmetic_mean(right);
        let overall_mean = arithmetic_mean(samples);
        let overall_var = variance(samples, overall_mean).max(1e-12);

        let stat = ((candidate * (n - candidate)) as f64 / n as f64)
            * (mean_r - mean_l).powi(2)
            / overall_var;

        // Approximate p-value using the asymptotic distribution of the
        // maximum CUSUM statistic (exponential tail bound).
        // For large stat, P(max CUSUM > h) ~ 2 * exp(-2h^2/n) (Siegmund).
        let p_value = (2.0 * (-2.0 * stat / n as f64).exp()).min(1.0);

        ChangePoint {
            index: candidate,
            statistic: stat,
            significance: Some(p_value),
        }
    }

    // -----------------------------------------------------------------------
    // Complex-signal support
    // -----------------------------------------------------------------------

    /// Detect changepoints in the magnitude of a complex signal using
    /// two-sided CUSUM.
    pub fn detect_cusum_complex(&self, samples: &[Complex]) -> Vec<usize> {
        let magnitudes: Vec<f64> = samples.iter().map(|&c| complex_mag(c)).collect();
        self.detect_two_sided_cusum(&magnitudes)
    }

    /// Detect changepoints in the instantaneous power of a complex signal.
    pub fn detect_power_change_complex(&self, samples: &[Complex]) -> Vec<ChangePoint> {
        let power: Vec<f64> = samples.iter().map(|&c| complex_mag_sq(c)).collect();
        self.detect_sliding_window(&power)
    }

    /// Binary segmentation on the magnitude of a complex signal.
    pub fn detect_binary_segmentation_complex(&self, samples: &[Complex]) -> Vec<ChangePoint> {
        let magnitudes: Vec<f64> = samples.iter().map(|&c| complex_mag(c)).collect();
        self.detect_binary_segmentation(&magnitudes)
    }

    /// Run-length encode segments of a complex signal based on magnitude.
    pub fn run_length_encode_complex(
        &self,
        samples: &[Complex],
        changepoints: &[usize],
    ) -> Vec<Segment> {
        let magnitudes: Vec<f64> = samples.iter().map(|&c| complex_mag(c)).collect();
        self.run_length_encode(&magnitudes, changepoints)
    }

    /// Bayesian changepoint probability on the magnitude of a complex signal.
    pub fn bayesian_changepoint_probability_complex(
        &self,
        samples: &[Complex],
    ) -> Vec<(usize, f64)> {
        let magnitudes: Vec<f64> = samples.iter().map(|&c| complex_mag(c)).collect();
        self.bayesian_changepoint_probability(&magnitudes)
    }

    /// Significance test for a candidate changepoint in a complex signal.
    pub fn significance_test_complex(
        &self,
        samples: &[Complex],
        candidate: usize,
    ) -> ChangePoint {
        let magnitudes: Vec<f64> = samples.iter().map(|&c| complex_mag(c)).collect();
        self.significance_test(&magnitudes, candidate)
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

fn arithmetic_mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

fn variance(data: &[f64], mean: f64) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    data.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / (data.len() - 1) as f64
}

/// Collapse a sorted list of detections into local maxima (non-maximum
/// suppression within runs of consecutive indices).
fn collapse_detections(detections: Vec<ChangePoint>) -> Vec<ChangePoint> {
    if detections.is_empty() {
        return detections;
    }

    let mut collapsed = Vec::new();
    let mut best = detections[0];

    for det in detections.iter().skip(1) {
        if det.index <= best.index + 1 {
            // Same run - keep the larger statistic
            if det.statistic > best.statistic {
                best = *det;
            }
        } else {
            collapsed.push(best);
            best = *det;
        }
    }
    collapsed.push(best);
    collapsed
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- helpers ----

    /// Create a signal with a single mean shift at `change_at`.
    fn step_signal(n: usize, change_at: usize, low: f64, high: f64) -> Vec<f64> {
        let mut v = vec![low; n];
        for x in v.iter_mut().skip(change_at) {
            *x = high;
        }
        v
    }

    /// Create a complex signal with a magnitude step.
    fn complex_step_signal(n: usize, change_at: usize, low: f64, high: f64) -> Vec<Complex> {
        (0..n)
            .map(|i| {
                let mag = if i < change_at { low } else { high };
                let phase = (i as f64) * 0.1;
                (mag * phase.cos(), mag * phase.sin())
            })
            .collect()
    }

    // ---- Test 1: default parameters ----
    #[test]
    fn test_default_params() {
        let det = ChangePointDetector::with_defaults();
        assert_eq!(det.cusum.drift, 0.5);
        assert_eq!(det.cusum.threshold, 5.0);
        assert_eq!(det.sliding_window.window_size, 30);
    }

    // ---- Test 2: CUSUM detects a clear upward shift ----
    #[test]
    fn test_cusum_upward_shift() {
        let params = CusumParams {
            drift: 0.5,
            threshold: 4.0,
        };
        let det = ChangePointDetector::new(params);
        let signal = step_signal(100, 50, 0.0, 5.0);
        let cps = det.detect_cusum(&signal);

        assert!(!cps.is_empty(), "should detect at least one changepoint");
        // First detection should be near index 50
        assert!(
            (cps[0] as i64 - 50).unsigned_abs() <= 15,
            "changepoint {} not near 50",
            cps[0]
        );
    }

    // ---- Test 3: CUSUM on constant signal -> no detection ----
    #[test]
    fn test_cusum_constant_signal() {
        let det = ChangePointDetector::with_defaults();
        let signal = vec![1.0; 200];
        let cps = det.detect_cusum(&signal);
        assert!(cps.is_empty(), "constant signal should have no changepoints");
    }

    // ---- Test 4: two-sided CUSUM detects downward shift ----
    #[test]
    fn test_two_sided_cusum_downward() {
        let params = CusumParams {
            drift: 0.3,
            threshold: 3.0,
        };
        let det = ChangePointDetector::new(params);
        let signal = step_signal(100, 50, 5.0, 0.0);
        let cps = det.detect_two_sided_cusum(&signal);

        assert!(
            !cps.is_empty(),
            "two-sided CUSUM should detect downward shift"
        );
    }

    // ---- Test 5: two-sided CUSUM detects upward shift ----
    #[test]
    fn test_two_sided_cusum_upward() {
        let params = CusumParams {
            drift: 0.3,
            threshold: 3.0,
        };
        let det = ChangePointDetector::new(params);
        let signal = step_signal(100, 50, 0.0, 5.0);
        let cps = det.detect_two_sided_cusum(&signal);
        assert!(
            !cps.is_empty(),
            "two-sided CUSUM should detect upward shift"
        );
    }

    // ---- Test 6: streaming CUSUM ----
    #[test]
    fn test_feed_sample_streaming() {
        let params = CusumParams {
            drift: 0.2,
            threshold: 3.0,
        };
        let mut det = ChangePointDetector::new(params);

        let signal = step_signal(120, 60, 0.0, 4.0);
        let mut detected = false;
        for &x in &signal {
            if det.feed_sample(x).is_some() {
                detected = true;
                break;
            }
        }
        assert!(detected, "streaming CUSUM should detect the shift");
    }

    // ---- Test 7: reset clears state ----
    #[test]
    fn test_reset() {
        let mut det = ChangePointDetector::with_defaults();
        for &x in &[10.0, 10.0, 10.0] {
            det.feed_sample(x);
        }
        det.reset();
        assert_eq!(det.cusum_pos, 0.0);
        assert_eq!(det.cusum_neg, 0.0);
        assert_eq!(det.cusum_n, 0);
    }

    // ---- Test 8: sliding window detects mean shift ----
    #[test]
    fn test_sliding_window_mean_shift() {
        let mut det = ChangePointDetector::with_defaults();
        det.sliding_window = SlidingWindowParams {
            window_size: 20,
            threshold: 3.0,
        };
        let signal = step_signal(100, 50, 0.0, 4.0);
        let cps = det.detect_sliding_window(&signal);

        assert!(!cps.is_empty(), "sliding window should find a changepoint");
        // The detection should be near index 50
        let nearest = cps.iter().min_by_key(|cp| (cp.index as i64 - 50).unsigned_abs()).unwrap();
        assert!(
            (nearest.index as i64 - 50).unsigned_abs() <= 20,
            "detection {} not near 50",
            nearest.index
        );
    }

    // ---- Test 9: sliding window on constant signal ----
    #[test]
    fn test_sliding_window_constant() {
        let mut det = ChangePointDetector::with_defaults();
        det.sliding_window = SlidingWindowParams {
            window_size: 15,
            threshold: 3.0,
        };
        let signal = vec![2.0; 100];
        let cps = det.detect_sliding_window(&signal);
        assert!(cps.is_empty(), "constant signal -> no sliding window detections");
    }

    // ---- Test 10: binary segmentation single changepoint ----
    #[test]
    fn test_binary_segmentation_single() {
        let mut det = ChangePointDetector::with_defaults();
        det.binary_segmentation = BinarySegmentationParams {
            min_segment_len: 10,
            threshold: 2.0,
        };
        let signal = step_signal(100, 50, 0.0, 5.0);
        let cps = det.detect_binary_segmentation(&signal);

        assert!(!cps.is_empty(), "binseg should find at least one cp");
        let nearest = cps.iter().min_by_key(|cp| (cp.index as i64 - 50).unsigned_abs()).unwrap();
        assert!(
            (nearest.index as i64 - 50).unsigned_abs() <= 10,
            "binseg detection {} not near 50",
            nearest.index
        );
    }

    // ---- Test 11: binary segmentation multiple changepoints ----
    #[test]
    fn test_binary_segmentation_multiple() {
        let mut det = ChangePointDetector::with_defaults();
        det.binary_segmentation = BinarySegmentationParams {
            min_segment_len: 8,
            threshold: 1.5,
        };
        // Three segments: [0..40) = 0, [40..70) = 5, [70..100) = -3
        let mut signal = vec![0.0; 40];
        signal.extend(vec![5.0; 30]);
        signal.extend(vec![-3.0; 30]);

        let cps = det.detect_binary_segmentation(&signal);
        assert!(
            cps.len() >= 2,
            "binseg should find at least 2 changepoints, found {}",
            cps.len()
        );
    }

    // ---- Test 12: Bayesian probability detects clear shift ----
    #[test]
    fn test_bayesian_clear_shift() {
        let mut det = ChangePointDetector::with_defaults();
        det.bayesian = BayesianParams {
            prior: 0.01,
            variance: 1.0,
        };
        let signal = step_signal(100, 50, 0.0, 4.0);
        let probs = det.bayesian_changepoint_probability(&signal);

        assert!(
            !probs.is_empty(),
            "Bayesian should detect high-probability changepoint"
        );
        // At least one probability should be high near index 50
        let near_50 = probs
            .iter()
            .any(|&(idx, p)| (idx as i64 - 50).unsigned_abs() <= 15 && p > 0.8);
        assert!(near_50, "should have high posterior near index 50");
    }

    // ---- Test 13: run-length encoding ----
    #[test]
    fn test_run_length_encode() {
        let det = ChangePointDetector::with_defaults();
        let signal = step_signal(100, 50, 1.0, 5.0);
        let segs = det.run_length_encode(&signal, &[50]);

        assert_eq!(segs.len(), 2, "should produce 2 segments");
        assert_eq!(segs[0].start, 0);
        assert_eq!(segs[0].end, 50);
        assert!((segs[0].mean - 1.0).abs() < 0.01);
        assert_eq!(segs[1].start, 50);
        assert_eq!(segs[1].end, 100);
        assert!((segs[1].mean - 5.0).abs() < 0.01);
    }

    // ---- Test 14: run-length encoding with no changepoints ----
    #[test]
    fn test_run_length_encode_no_changepoints() {
        let det = ChangePointDetector::with_defaults();
        let signal = vec![3.0; 50];
        let segs = det.run_length_encode(&signal, &[]);
        assert_eq!(segs.len(), 1);
        assert_eq!(segs[0].start, 0);
        assert_eq!(segs[0].end, 50);
    }

    // ---- Test 15: significance test ----
    #[test]
    fn test_significance_test() {
        let det = ChangePointDetector::with_defaults();
        let signal = step_signal(100, 50, 0.0, 5.0);
        let cp = det.significance_test(&signal, 50);

        assert_eq!(cp.index, 50);
        assert!(cp.statistic > 0.0);
        assert!(cp.significance.is_some());
        let p = cp.significance.unwrap();
        assert!(p < 0.5, "p-value {} should be small for a clear shift", p);
    }

    // ---- Test 16: complex signal CUSUM ----
    #[test]
    fn test_cusum_complex() {
        let params = CusumParams {
            drift: 0.3,
            threshold: 3.0,
        };
        let det = ChangePointDetector::new(params);
        let signal = complex_step_signal(100, 50, 1.0, 5.0);
        let cps = det.detect_cusum_complex(&signal);

        assert!(
            !cps.is_empty(),
            "complex CUSUM should detect magnitude shift"
        );
    }

    // ---- Test 17: complex signal power changepoint ----
    #[test]
    fn test_power_change_complex() {
        let mut det = ChangePointDetector::with_defaults();
        det.sliding_window = SlidingWindowParams {
            window_size: 15,
            threshold: 2.0,
        };
        let signal = complex_step_signal(100, 50, 1.0, 5.0);
        let cps = det.detect_power_change_complex(&signal);

        assert!(
            !cps.is_empty(),
            "should detect power change in complex signal"
        );
    }

    // ---- Test 18: complex binary segmentation ----
    #[test]
    fn test_binary_segmentation_complex() {
        let mut det = ChangePointDetector::with_defaults();
        det.binary_segmentation = BinarySegmentationParams {
            min_segment_len: 8,
            threshold: 1.0,
        };
        let signal = complex_step_signal(100, 50, 1.0, 6.0);
        let cps = det.detect_binary_segmentation_complex(&signal);

        assert!(
            !cps.is_empty(),
            "complex binseg should detect magnitude step"
        );
    }

    // ---- Test 19: complex run-length encoding ----
    #[test]
    fn test_run_length_encode_complex() {
        let det = ChangePointDetector::with_defaults();
        let signal = complex_step_signal(60, 30, 2.0, 8.0);
        let segs = det.run_length_encode_complex(&signal, &[30]);
        assert_eq!(segs.len(), 2);
        // First segment magnitude ~2.0, second ~8.0
        assert!(segs[0].mean < segs[1].mean);
    }

    // ---- Test 20: complex Bayesian ----
    #[test]
    fn test_bayesian_complex() {
        let mut det = ChangePointDetector::with_defaults();
        det.bayesian = BayesianParams {
            prior: 0.01,
            variance: 1.0,
        };
        let signal = complex_step_signal(100, 50, 1.0, 6.0);
        let probs = det.bayesian_changepoint_probability_complex(&signal);
        assert!(
            !probs.is_empty(),
            "Bayesian on complex magnitudes should find changepoints"
        );
    }

    // ---- Test 21: complex significance test ----
    #[test]
    fn test_significance_complex() {
        let det = ChangePointDetector::with_defaults();
        let signal = complex_step_signal(100, 50, 1.0, 6.0);
        let cp = det.significance_test_complex(&signal, 50);
        assert_eq!(cp.index, 50);
        assert!(cp.significance.is_some());
    }

    // ---- Test 22: helper - arithmetic_mean ----
    #[test]
    fn test_arithmetic_mean() {
        assert_eq!(arithmetic_mean(&[]), 0.0);
        assert!((arithmetic_mean(&[2.0, 4.0, 6.0]) - 4.0).abs() < 1e-12);
    }

    // ---- Test 23: helper - variance ----
    #[test]
    fn test_variance() {
        let data = [2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let m = arithmetic_mean(&data);
        let v = variance(&data, m);
        assert!((v - 4.571428571428571).abs() < 1e-6);
    }

    // ---- Test 24: collapse_detections ----
    #[test]
    fn test_collapse_detections() {
        let dets = vec![
            ChangePoint { index: 48, statistic: 3.0, significance: None },
            ChangePoint { index: 49, statistic: 5.0, significance: None },
            ChangePoint { index: 50, statistic: 4.0, significance: None },
            ChangePoint { index: 80, statistic: 2.0, significance: None },
        ];
        let collapsed = collapse_detections(dets);
        assert_eq!(collapsed.len(), 2);
        assert_eq!(collapsed[0].index, 49); // peak of first run
        assert_eq!(collapsed[1].index, 80);
    }
}
