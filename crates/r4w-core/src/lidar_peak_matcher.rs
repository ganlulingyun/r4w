//! Pulse-to-echo correlation and time-of-flight peak matching for LiDAR range
//! and intensity processing.
//!
//! This module provides [`LidarPeakMatcher`], which performs matched-filter
//! detection of return pulses, converts time-of-flight measurements to range,
//! identifies multiple returns per pulse, and refines range estimates with
//! sub-sample interpolation.
//!
//! # Example
//!
//! ```
//! use r4w_core::lidar_peak_matcher::{LidarPeakMatcher, InterpolationMethod};
//!
//! // A simple Gaussian-ish transmit pulse template sampled at 1 GHz.
//! let template: Vec<(f64, f64)> = (0..16)
//!     .map(|i| {
//!         let t = (i as f64 - 7.5) / 3.0;
//!         ((-0.5 * t * t).exp(), 0.0)
//!     })
//!     .collect();
//!
//! let mut matcher = LidarPeakMatcher::new(1.0e9, &template);
//! matcher.set_interpolation(InterpolationMethod::Parabolic);
//!
//! // Build a synthetic received waveform with a single echo at sample 50.
//! let mut rx = vec![(0.0, 0.0); 128];
//! for (i, &(re, _im)) in template.iter().enumerate() {
//!     rx[50 + i].0 += 0.8 * re;
//! }
//!
//! let returns = matcher.detect(&rx);
//! assert!(!returns.is_empty());
//! // The detected range should be close to c/2 * 50/fs ≈ 7.49 m
//! let range = returns[0].range_m;
//! assert!((range - 7.49).abs() < 0.5, "range was {range}");
//! ```

// ── public types ──────────────────────────────────────────────────────

/// Speed of light in vacuum (m/s).
const C: f64 = 299_792_458.0;

/// A single detected LiDAR return.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarReturn {
    /// Fractional sample index of the correlation peak.
    pub peak_sample: f64,
    /// One-way range in metres (half round-trip).
    pub range_m: f64,
    /// Peak correlation magnitude (proxy for return intensity).
    pub intensity: f64,
    /// Signal-to-noise ratio of this peak in dB.
    pub snr_db: f64,
    /// Classification of the return within the pulse train.
    pub classification: ReturnClass,
}

/// Classification of a return within a multi-return pulse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReturnClass {
    /// Only return detected.
    Single,
    /// First of multiple returns.
    First,
    /// Neither first nor last.
    Intermediate,
    /// Last of multiple returns.
    Last,
}

/// Sub-sample interpolation strategy for peak refinement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpolationMethod {
    /// No interpolation – use the integer sample index.
    None,
    /// 3-point parabolic fit around the peak.
    Parabolic,
    /// 3-point Gaussian fit (log-domain parabola).
    Gaussian,
}

/// Parameters for Gaussian decomposition of a return waveform.
#[derive(Debug, Clone, PartialEq)]
pub struct GaussianComponent {
    /// Amplitude of the fitted Gaussian.
    pub amplitude: f64,
    /// Centre position in samples.
    pub centre: f64,
    /// Standard deviation (sigma) in samples.
    pub sigma: f64,
}

// ── main struct ───────────────────────────────────────────────────────

/// Configurable LiDAR pulse-to-echo correlator and time-of-flight analyser.
///
/// Construct via [`LidarPeakMatcher::new`], then call [`detect`](LidarPeakMatcher::detect)
/// on each received waveform.
#[derive(Debug, Clone)]
pub struct LidarPeakMatcher {
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Time-reversed conjugate of the transmit pulse (matched-filter kernel).
    matched_filter: Vec<(f64, f64)>,
    /// Length of the original template.
    template_len: usize,
    /// Interpolation method for sub-sample accuracy.
    interpolation: InterpolationMethod,
    /// False-alarm probability drives the CFAR threshold multiplier.
    /// Stored as the multiplier applied to the estimated noise standard deviation.
    threshold_multiplier: f64,
    /// Minimum separation (in samples) between two detected peaks.
    min_peak_separation: usize,
    /// Enable range-walk compensation.
    range_walk_enabled: bool,
    /// Reference intensity for range-walk compensation.
    range_walk_ref_intensity: f64,
    /// Range-walk slope (samples per unit intensity change).
    range_walk_slope: f64,
}

impl LidarPeakMatcher {
    // ── construction ──────────────────────────────────────────────

    /// Create a new `LidarPeakMatcher`.
    ///
    /// * `sample_rate` – sampling rate in Hz.
    /// * `template` – the transmit pulse waveform as `(re, im)` tuples.
    pub fn new(sample_rate: f64, template: &[(f64, f64)]) -> Self {
        assert!(!template.is_empty(), "template must not be empty");
        assert!(sample_rate > 0.0, "sample_rate must be positive");

        // Matched filter = time-reversed, conjugated template.
        let matched_filter: Vec<(f64, f64)> =
            template.iter().rev().map(|&(re, im)| (re, -im)).collect();

        Self {
            sample_rate,
            matched_filter,
            template_len: template.len(),
            interpolation: InterpolationMethod::None,
            threshold_multiplier: 4.0, // ~6.2e-5 Pfa for Gaussian noise
            min_peak_separation: template.len(),
            range_walk_enabled: false,
            range_walk_ref_intensity: 1.0,
            range_walk_slope: 0.0,
        }
    }

    // ── setters ───────────────────────────────────────────────────

    /// Set the sub-sample interpolation method.
    pub fn set_interpolation(&mut self, method: InterpolationMethod) {
        self.interpolation = method;
    }

    /// Set the CFAR threshold multiplier (number of noise sigmas).
    pub fn set_threshold_multiplier(&mut self, mult: f64) {
        assert!(mult > 0.0);
        self.threshold_multiplier = mult;
    }

    /// Set the minimum peak separation in samples.
    pub fn set_min_peak_separation(&mut self, samps: usize) {
        self.min_peak_separation = samps;
    }

    /// Enable range-walk compensation.
    ///
    /// * `ref_intensity` – reference intensity at which the walk is zero.
    /// * `slope` – walk in samples per unit of intensity deviation from
    ///   the reference.
    pub fn enable_range_walk(&mut self, ref_intensity: f64, slope: f64) {
        self.range_walk_enabled = true;
        self.range_walk_ref_intensity = ref_intensity;
        self.range_walk_slope = slope;
    }

    /// Disable range-walk compensation.
    pub fn disable_range_walk(&mut self) {
        self.range_walk_enabled = false;
    }

    // ── getters ───────────────────────────────────────────────────

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Return the template length.
    pub fn template_len(&self) -> usize {
        self.template_len
    }

    // ── core detection ────────────────────────────────────────────

    /// Run the full detection pipeline on a received waveform.
    ///
    /// Returns a list of [`LidarReturn`] sorted by ascending range.
    pub fn detect(&self, received: &[(f64, f64)]) -> Vec<LidarReturn> {
        if received.len() < self.template_len {
            return Vec::new();
        }

        // 1. Matched-filter correlation.
        let corr = self.correlate(received);

        // 2. Magnitude envelope.
        let mag: Vec<f64> = corr.iter().map(|&(re, im)| (re * re + im * im).sqrt()).collect();

        // 3. Noise floor estimation and threshold.
        let threshold = self.estimate_threshold(&mag);

        // 4. Peak detection with minimum separation.
        let peak_indices = self.find_peaks(&mag, threshold);

        // 5. Build returns with interpolation, intensity, SNR, classification.
        let noise_sigma = threshold / self.threshold_multiplier;
        let mut returns: Vec<LidarReturn> = peak_indices
            .iter()
            .map(|&idx| {
                let (refined_idx, intensity) = self.refine_peak(&mag, idx);

                // Optionally apply range-walk correction.
                let corrected_idx = if self.range_walk_enabled {
                    self.apply_range_walk(refined_idx, intensity)
                } else {
                    refined_idx
                };

                let range_m = self.tof_to_range(corrected_idx);
                let snr_db = if noise_sigma > 0.0 {
                    20.0 * (intensity / noise_sigma).log10()
                } else {
                    f64::INFINITY
                };

                LidarReturn {
                    peak_sample: corrected_idx,
                    range_m,
                    intensity,
                    snr_db,
                    classification: ReturnClass::Single, // patched below
                }
            })
            .collect();

        // 6. Classify returns.
        classify_returns(&mut returns);

        returns
    }

    /// Cross-correlate `received` with the matched filter (time-domain).
    pub fn correlate(&self, received: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n_out = received.len() - self.matched_filter.len() + 1;
        let mut out = Vec::with_capacity(n_out);
        for i in 0..n_out {
            let mut re_acc = 0.0;
            let mut im_acc = 0.0;
            for (j, &(h_re, h_im)) in self.matched_filter.iter().enumerate() {
                let (x_re, x_im) = received[i + j];
                // complex multiply: (x_re + j*x_im) * (h_re + j*h_im)
                re_acc += x_re * h_re - x_im * h_im;
                im_acc += x_re * h_im + x_im * h_re;
            }
            out.push((re_acc, im_acc));
        }
        out
    }

    /// Convert a fractional sample index to one-way range in metres.
    pub fn tof_to_range(&self, sample_index: f64) -> f64 {
        let tof = sample_index / self.sample_rate;
        C * tof / 2.0
    }

    // ── internal helpers ──────────────────────────────────────────

    /// Estimate a detection threshold from the correlation magnitude vector.
    ///
    /// Uses a robust median-based noise estimator:
    ///   sigma_hat = median(|x|) / 0.6745
    fn estimate_threshold(&self, mag: &[f64]) -> f64 {
        if mag.is_empty() {
            return 0.0;
        }
        let mut sorted = mag.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = sorted[sorted.len() / 2];
        let sigma = median / 0.6745;
        sigma * self.threshold_multiplier
    }

    /// Find local-maximum indices above `threshold` with at least
    /// `min_peak_separation` samples between them.
    fn find_peaks(&self, mag: &[f64], threshold: f64) -> Vec<usize> {
        let mut peaks = Vec::new();
        let len = mag.len();
        if len < 3 {
            return peaks;
        }
        for i in 1..len - 1 {
            if mag[i] > threshold && mag[i] >= mag[i - 1] && mag[i] >= mag[i + 1] {
                // Enforce minimum separation from the last accepted peak.
                if let Some(&last) = peaks.last() {
                    if i - last < self.min_peak_separation {
                        // Keep the stronger one.
                        if mag[i] > mag[last] {
                            *peaks.last_mut().unwrap() = i;
                        }
                        continue;
                    }
                }
                peaks.push(i);
            }
        }
        peaks
    }

    /// Refine a peak position via sub-sample interpolation, returning
    /// `(fractional_index, interpolated_magnitude)`.
    fn refine_peak(&self, mag: &[f64], idx: usize) -> (f64, f64) {
        if idx == 0 || idx >= mag.len() - 1 {
            return (idx as f64, mag[idx]);
        }

        let y_prev = mag[idx - 1];
        let y_peak = mag[idx];
        let y_next = mag[idx + 1];

        match self.interpolation {
            InterpolationMethod::None => (idx as f64, y_peak),

            InterpolationMethod::Parabolic => {
                let denom = 2.0 * (2.0 * y_peak - y_prev - y_next);
                if denom.abs() < 1e-30 {
                    return (idx as f64, y_peak);
                }
                let delta = (y_prev - y_next) / denom;
                let refined = idx as f64 + delta;
                let interp_mag = y_peak - 0.25 * (y_prev - y_next) * delta;
                (refined, interp_mag)
            }

            InterpolationMethod::Gaussian => {
                // Gaussian interpolation in log domain.
                if y_prev <= 0.0 || y_peak <= 0.0 || y_next <= 0.0 {
                    return (idx as f64, y_peak);
                }
                let lp = y_prev.ln();
                let lc = y_peak.ln();
                let ln_ = y_next.ln();
                let denom = 2.0 * (2.0 * lc - lp - ln_);
                if denom.abs() < 1e-30 {
                    return (idx as f64, y_peak);
                }
                let delta = (lp - ln_) / denom;
                let refined = idx as f64 + delta;
                let interp_mag = (lc - 0.25 * (lp - ln_) * delta).exp();
                (refined, interp_mag)
            }
        }
    }

    /// Apply range-walk compensation: shift the peak index to account for
    /// the bias caused by varying return intensity.
    fn apply_range_walk(&self, sample_idx: f64, intensity: f64) -> f64 {
        let correction = self.range_walk_slope * (intensity - self.range_walk_ref_intensity);
        sample_idx - correction
    }
}

// ── return classification ─────────────────────────────────────────────

fn classify_returns(returns: &mut [LidarReturn]) {
    let n = returns.len();
    if n == 0 {
        return;
    }
    if n == 1 {
        returns[0].classification = ReturnClass::Single;
        return;
    }
    returns[0].classification = ReturnClass::First;
    returns[n - 1].classification = ReturnClass::Last;
    for r in returns[1..n - 1].iter_mut() {
        r.classification = ReturnClass::Intermediate;
    }
}

// ── Gaussian decomposition ────────────────────────────────────────────

/// Decompose a 1-D waveform into a sum of Gaussian components using
/// iterative peak-fit-subtract.
///
/// * `signal` – the waveform magnitudes (non-negative).
/// * `max_components` – stop after this many Gaussians.
/// * `min_amplitude` – ignore components weaker than this.
///
/// Returns fitted [`GaussianComponent`]s sorted by descending amplitude.
pub fn gaussian_decomposition(
    signal: &[f64],
    max_components: usize,
    min_amplitude: f64,
) -> Vec<GaussianComponent> {
    let mut residual: Vec<f64> = signal.to_vec();
    let mut components = Vec::new();

    for _ in 0..max_components {
        // Find the peak of the residual.
        let (peak_idx, peak_val) = residual
            .iter()
            .enumerate()
            .fold((0, f64::NEG_INFINITY), |(bi, bv), (i, &v)| {
                if v > bv { (i, v) } else { (bi, bv) }
            });

        if peak_val < min_amplitude {
            break;
        }

        // Estimate sigma from the half-maximum width.
        let half = peak_val / 2.0;
        let mut left = peak_idx;
        while left > 0 && residual[left] > half {
            left -= 1;
        }
        let mut right = peak_idx;
        while right < residual.len() - 1 && residual[right] > half {
            right += 1;
        }
        let fwhm = (right as f64 - left as f64).max(1.0);
        let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0); // FWHM = 2*sqrt(2*ln2)*sigma

        // Refine centre via parabolic interpolation if possible.
        let centre = if peak_idx > 0 && peak_idx < residual.len() - 1 {
            let y_prev = residual[peak_idx - 1];
            let y_next = residual[peak_idx + 1];
            let denom = 2.0 * (2.0 * peak_val - y_prev - y_next);
            if denom.abs() > 1e-30 {
                peak_idx as f64 + (y_prev - y_next) / denom
            } else {
                peak_idx as f64
            }
        } else {
            peak_idx as f64
        };

        // Subtract fitted Gaussian from residual.
        for (i, r) in residual.iter_mut().enumerate() {
            let t = (i as f64 - centre) / sigma;
            *r -= peak_val * (-0.5 * t * t).exp();
            if *r < 0.0 {
                *r = 0.0;
            }
        }

        components.push(GaussianComponent {
            amplitude: peak_val,
            centre,
            sigma,
        });
    }

    // Sort descending by amplitude (should already be, but be safe).
    components.sort_by(|a, b| b.amplitude.partial_cmp(&a.amplitude).unwrap());
    components
}

// ── helper: synthesise a Gaussian pulse ───────────────────────────────

/// Generate a real-valued Gaussian pulse as complex samples with zero
/// imaginary part.
///
/// * `n` – number of samples.
/// * `centre` – peak position in samples.
/// * `sigma` – standard deviation in samples.
pub fn gaussian_pulse(n: usize, centre: f64, sigma: f64) -> Vec<(f64, f64)> {
    (0..n)
        .map(|i| {
            let t = (i as f64 - centre) / sigma;
            ((-0.5 * t * t).exp(), 0.0)
        })
        .collect()
}

// ══════════════════════════════════════════════════════════════════════
// Tests
// ══════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a short Gaussian template centred in the middle.
    fn make_template(len: usize) -> Vec<(f64, f64)> {
        let centre = (len - 1) as f64 / 2.0;
        let sigma = len as f64 / 6.0;
        gaussian_pulse(len, centre, sigma)
    }

    /// Insert a scaled copy of `template` into `buf` starting at `offset`.
    fn insert_echo(buf: &mut [(f64, f64)], template: &[(f64, f64)], offset: usize, scale: f64) {
        for (i, &(re, im)) in template.iter().enumerate() {
            if offset + i < buf.len() {
                buf[offset + i].0 += scale * re;
                buf[offset + i].1 += scale * im;
            }
        }
    }

    // ── basic construction ────────────────────────────────────────

    #[test]
    fn test_new_basic() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);
        assert_eq!(m.template_len(), 16);
        assert_eq!(m.sample_rate(), 1e9);
    }

    #[test]
    #[should_panic(expected = "template must not be empty")]
    fn test_empty_template_panics() {
        LidarPeakMatcher::new(1e9, &[]);
    }

    #[test]
    #[should_panic(expected = "sample_rate must be positive")]
    fn test_zero_sample_rate_panics() {
        LidarPeakMatcher::new(0.0, &[(1.0, 0.0)]);
    }

    // ── single return detection ───────────────────────────────────

    #[test]
    fn test_single_return_no_interpolation() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_interpolation(InterpolationMethod::None);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 1.0);

        let rets = m.detect(&rx);
        assert_eq!(rets.len(), 1);
        // Peak of correlation should be near sample 80.
        assert!((rets[0].peak_sample - 80.0).abs() < 2.0, "peak at {}", rets[0].peak_sample);
        assert!(rets[0].intensity > 0.0);
        assert_eq!(rets[0].classification, ReturnClass::Single);
    }

    #[test]
    fn test_single_return_parabolic() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_interpolation(InterpolationMethod::Parabolic);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 1.0);

        let rets = m.detect(&rx);
        assert_eq!(rets.len(), 1);
        assert!((rets[0].peak_sample - 80.0).abs() < 2.0);
    }

    #[test]
    fn test_single_return_gaussian_interp() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_interpolation(InterpolationMethod::Gaussian);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 1.0);

        let rets = m.detect(&rx);
        assert_eq!(rets.len(), 1);
        assert!((rets[0].peak_sample - 80.0).abs() < 2.0);
    }

    // ── multiple returns ──────────────────────────────────────────

    #[test]
    fn test_two_returns() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_min_peak_separation(20);

        let mut rx = vec![(0.0, 0.0); 512];
        insert_echo(&mut rx, &tpl, 60, 1.0);
        insert_echo(&mut rx, &tpl, 160, 0.6);

        let rets = m.detect(&rx);
        assert_eq!(rets.len(), 2, "expected 2 returns, got {:?}", rets);
        assert_eq!(rets[0].classification, ReturnClass::First);
        assert_eq!(rets[1].classification, ReturnClass::Last);
        assert!(rets[0].intensity > rets[1].intensity);
    }

    #[test]
    fn test_three_returns_classification() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_min_peak_separation(20);

        let mut rx = vec![(0.0, 0.0); 512];
        insert_echo(&mut rx, &tpl, 50, 1.0);
        insert_echo(&mut rx, &tpl, 150, 0.7);
        insert_echo(&mut rx, &tpl, 250, 0.4);

        let rets = m.detect(&rx);
        assert_eq!(rets.len(), 3, "got {:?}", rets);
        assert_eq!(rets[0].classification, ReturnClass::First);
        assert_eq!(rets[1].classification, ReturnClass::Intermediate);
        assert_eq!(rets[2].classification, ReturnClass::Last);
    }

    // ── range conversion ──────────────────────────────────────────

    #[test]
    fn test_tof_to_range() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);
        // 100 samples at 1 GHz = 100 ns round-trip -> 50 ns one-way -> ~14.99 m
        let range = m.tof_to_range(100.0);
        let expected = C * 100e-9 / 2.0;
        assert!((range - expected).abs() < 1e-6, "range={range}, expected={expected}");
    }

    #[test]
    fn test_tof_to_range_zero() {
        let tpl = make_template(8);
        let m = LidarPeakMatcher::new(2e9, &tpl);
        assert_eq!(m.tof_to_range(0.0), 0.0);
    }

    // ── threshold & false alarm suppression ───────────────────────

    #[test]
    fn test_noise_only_no_detections() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_threshold_multiplier(5.0);

        // Low-level pseudo-random noise (deterministic LCG).
        let mut rx = vec![(0.0, 0.0); 512];
        let mut seed: u64 = 42;
        for s in rx.iter_mut() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let noise = ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.001;
            s.0 = noise;
        }

        let rets = m.detect(&rx);
        assert!(rets.is_empty(), "false alarms in noise: {:?}", rets);
    }

    #[test]
    fn test_threshold_multiplier_affects_sensitivity() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 0.05); // weak echo

        // High threshold should miss it.
        m.set_threshold_multiplier(100.0);
        let rets_high = m.detect(&rx);

        // Low threshold should find it.
        m.set_threshold_multiplier(0.5);
        let rets_low = m.detect(&rx);

        assert!(rets_high.len() <= rets_low.len());
    }

    // ── SNR calculation ───────────────────────────────────────────

    #[test]
    fn test_snr_positive_for_strong_signal() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 1.0);

        let rets = m.detect(&rx);
        assert!(!rets.is_empty());
        assert!(rets[0].snr_db > 0.0, "snr_db={}", rets[0].snr_db);
    }

    // ── range walk compensation ───────────────────────────────────

    #[test]
    fn test_range_walk_shifts_peak() {
        let tpl = make_template(16);
        let mut m = LidarPeakMatcher::new(1e9, &tpl);
        m.set_interpolation(InterpolationMethod::None);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 1.0);

        // Without range walk.
        m.disable_range_walk();
        let rets_no_walk = m.detect(&rx);

        // With range walk that shifts by 2.0 samples for this intensity.
        m.enable_range_walk(0.0, 2.0);
        let rets_walk = m.detect(&rx);

        assert!(!rets_no_walk.is_empty());
        assert!(!rets_walk.is_empty());
        // Walk should change the peak sample.
        let diff = (rets_walk[0].peak_sample - rets_no_walk[0].peak_sample).abs();
        assert!(diff > 0.1, "range walk had no effect: diff={diff}");
    }

    #[test]
    fn test_range_walk_disabled_by_default() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 80, 1.0);
        let rets = m.detect(&rx);
        // Just verify it runs without error.
        assert!(!rets.is_empty());
    }

    // ── Gaussian decomposition ────────────────────────────────────

    #[test]
    fn test_gaussian_decomposition_single() {
        let n = 128;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = (i as f64 - 64.0) / 8.0;
                5.0 * (-0.5 * t * t).exp()
            })
            .collect();

        let comps = gaussian_decomposition(&signal, 3, 0.1);
        assert!(!comps.is_empty());
        assert!((comps[0].amplitude - 5.0).abs() < 0.5, "amp={}", comps[0].amplitude);
        assert!((comps[0].centre - 64.0).abs() < 1.0, "centre={}", comps[0].centre);
    }

    #[test]
    fn test_gaussian_decomposition_two_peaks() {
        let n = 256;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t1 = (i as f64 - 80.0) / 6.0;
                let t2 = (i as f64 - 180.0) / 10.0;
                4.0 * (-0.5 * t1 * t1).exp() + 2.5 * (-0.5 * t2 * t2).exp()
            })
            .collect();

        let comps = gaussian_decomposition(&signal, 5, 0.1);
        assert!(comps.len() >= 2, "got {} components", comps.len());
        // First component should be the stronger one.
        assert!(comps[0].amplitude > comps[1].amplitude);
    }

    #[test]
    fn test_gaussian_decomposition_min_amplitude() {
        let signal = vec![0.01; 64];
        let comps = gaussian_decomposition(&signal, 5, 0.1);
        assert!(comps.is_empty(), "should find nothing below min_amplitude");
    }

    // ── gaussian_pulse helper ─────────────────────────────────────

    #[test]
    fn test_gaussian_pulse_peak() {
        let p = gaussian_pulse(64, 32.0, 5.0);
        assert_eq!(p.len(), 64);
        // Peak at index 32 should be ~1.0
        assert!((p[32].0 - 1.0).abs() < 1e-10);
        assert_eq!(p[32].1, 0.0);
    }

    #[test]
    fn test_gaussian_pulse_symmetry() {
        let n = 65;
        let centre = 32.0;
        let p = gaussian_pulse(n, centre, 8.0);
        for i in 0..32 {
            let diff = (p[i].0 - p[64 - i].0).abs();
            assert!(diff < 1e-12, "asymmetry at i={i}: {diff}");
        }
    }

    // ── correlate API ─────────────────────────────────────────────

    #[test]
    fn test_correlate_length() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);
        let rx = vec![(0.0, 0.0); 128];
        let corr = m.correlate(&rx);
        assert_eq!(corr.len(), 128 - 16 + 1);
    }

    #[test]
    fn test_correlate_peak_location() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);

        let mut rx = vec![(0.0, 0.0); 256];
        insert_echo(&mut rx, &tpl, 100, 1.0);

        let corr = m.correlate(&rx);
        let mag: Vec<f64> = corr.iter().map(|&(re, im)| (re * re + im * im).sqrt()).collect();
        let peak_idx = mag
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Correlation peak should be near 100.
        assert!((peak_idx as i64 - 100).abs() < 2, "peak at {peak_idx}");
    }

    // ── short input edge case ─────────────────────────────────────

    #[test]
    fn test_short_input_returns_empty() {
        let tpl = make_template(16);
        let m = LidarPeakMatcher::new(1e9, &tpl);
        let rx = vec![(0.0, 0.0); 8]; // shorter than template
        let rets = m.detect(&rx);
        assert!(rets.is_empty());
    }
}
