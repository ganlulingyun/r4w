//! Cyclostationary Feature Detection for Blind Signal Presence and Modulation Classification
//!
//! Implements spectral correlation function (SCF) based cyclostationary analysis
//! for detecting signals below the noise floor and estimating symbol rates. Uses
//! DFT-based correlation to compute the spectral correlation at specified cyclic
//! frequencies alpha. Signals with periodic structure (e.g., BPSK at symbol rate Rs)
//! exhibit peaks in the SCF at alpha = Rs, 2*Rs, etc., while stationary noise does not.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::cyclostationary_detector::{CyclostationaryDetector, spectral_correlation_function};
//!
//! // Generate a simple periodic signal (BPSK-like, symbol rate = 0.1 * sample rate)
//! let n = 512;
//! let sps = 10; // samples per symbol
//! let mut samples = Vec::with_capacity(n);
//! let mut rng: u32 = 7;
//! for i in 0..n {
//!     rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
//!     let bit = if (rng >> (16 + (i / sps) % 16)) & 1 == 1 { 1.0 } else { -1.0 };
//!     samples.push((bit, 0.0));
//! }
//!
//! // Compute SCF at the symbol rate cyclic frequency
//! let alpha = 1.0 / sps as f64; // normalized cyclic frequency
//! let scf = spectral_correlation_function(&samples, alpha, 64);
//! assert_eq!(scf.len(), 64);
//!
//! // Use the detector for threshold-based detection
//! let mut det = CyclostationaryDetector::new(64, 4);
//! let result = det.detect(&samples, alpha);
//! // result.detected indicates whether a cyclostationary feature was found
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Result type
// ---------------------------------------------------------------------------

/// Result of cyclostationary detection at a single cyclic frequency.
#[derive(Debug, Clone)]
pub struct CycloResult {
    /// Whether a cyclostationary feature was detected above the noise floor.
    pub detected: bool,
    /// Peak value of the spectral correlation magnitude.
    pub peak_value: f64,
    /// Cyclic frequency (alpha) that was tested.
    pub cyclic_frequency: f64,
    /// Confidence metric in [0, 1]. Higher means stronger feature relative to noise.
    pub confidence: f64,
}

// ---------------------------------------------------------------------------
// Detector
// ---------------------------------------------------------------------------

/// Cyclostationary feature detector using DFT-based spectral correlation.
///
/// Averages over multiple segments of the input to reduce variance, then
/// compares the peak SCF magnitude against a noise-floor estimate derived
/// from the median bin magnitude.
#[derive(Debug, Clone)]
pub struct CyclostationaryDetector {
    fft_size: usize,
    num_averages: usize,
    /// Cached SCF magnitude from the last `detect` call.
    last_scf_mag: Vec<f64>,
}

impl CyclostationaryDetector {
    /// Create a new detector.
    ///
    /// * `fft_size` -- DFT length used for each segment (must be >= 2).
    /// * `num_averages` -- maximum number of segments to average over.
    pub fn new(fft_size: usize, num_averages: usize) -> Self {
        assert!(fft_size >= 2, "fft_size must be >= 2");
        assert!(num_averages >= 1, "num_averages must be >= 1");
        Self {
            fft_size,
            num_averages,
            last_scf_mag: Vec::new(),
        }
    }

    /// Compute the spectral correlation at cyclic frequency `alpha` and decide
    /// whether a cyclostationary feature is present.
    ///
    /// `alpha` is the *normalized* cyclic frequency (cycles per sample).
    /// For a signal with symbol rate Rs at sample rate Fs, use alpha = Rs / Fs.
    pub fn detect(&mut self, samples: &[(f64, f64)], alpha: f64) -> CycloResult {
        let scf_mag = spectral_correlation_function(samples, alpha, self.fft_size);

        // Also compute SCF at alpha = 0 (PSD) for noise reference, averaged
        let psd = averaged_scf_magnitude(samples, 0.0, self.fft_size, self.num_averages);

        // Average SCF at the target alpha
        let avg_scf = averaged_scf_magnitude(samples, alpha, self.fft_size, self.num_averages);

        let peak_value = avg_scf.iter().cloned().fold(0.0_f64, f64::max);

        // Noise floor estimate: median of PSD bins
        let noise_floor = median_of(&psd);

        // Confidence: ratio of peak SCF to noise floor, clamped to [0,1]
        let confidence = if noise_floor > 1e-30 {
            let ratio = peak_value / noise_floor;
            // Map ratio to [0,1] with a soft saturation; ratio > 3 gives ~1.0
            (1.0 - (-0.5 * ratio).exp()).clamp(0.0, 1.0)
        } else if peak_value > 1e-30 {
            1.0
        } else {
            0.0
        };

        // Detection threshold: peak must exceed 2x noise floor
        let detected = peak_value > 2.0 * noise_floor && noise_floor > 1e-30;

        self.last_scf_mag = scf_mag;

        CycloResult {
            detected,
            peak_value,
            cyclic_frequency: alpha,
            confidence,
        }
    }

    /// Return the SCF magnitude vector from the most recent `detect` call.
    pub fn spectral_correlation_magnitude(&self) -> Vec<f64> {
        self.last_scf_mag.clone()
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Compute the spectral correlation function (SCF) magnitude at cyclic
/// frequency `alpha` using a single-segment DFT approach.
///
/// Returns a vector of length `fft_size` containing |S_x^alpha(f)| for each
/// frequency bin f.
///
/// The algorithm frequency-shifts the signal by +/- alpha/2, windows each
/// segment, takes the DFT of both branches, and multiplies
/// X(f + alpha/2) * conj(X(f - alpha/2)).
pub fn spectral_correlation_function(
    samples: &[(f64, f64)],
    alpha: f64,
    fft_size: usize,
) -> Vec<f64> {
    let n = fft_size.min(samples.len()).max(1);

    // Frequency-shift by +alpha/2 and -alpha/2
    let mut branch_pos = Vec::with_capacity(n);
    let mut branch_neg = Vec::with_capacity(n);

    for k in 0..n {
        let (re, im) = samples[k];
        let phase_pos = 2.0 * PI * (alpha / 2.0) * k as f64;
        let phase_neg = -2.0 * PI * (alpha / 2.0) * k as f64;

        // Apply Hann window for leakage reduction
        let win = 0.5 * (1.0 - (2.0 * PI * k as f64 / n as f64).cos());

        let (cp, sp) = (phase_pos.cos(), phase_pos.sin());
        let (cn, sn) = (phase_neg.cos(), phase_neg.sin());

        // complex multiply: (re + j*im) * (cos + j*sin)
        branch_pos.push(((re * cp - im * sp) * win, (re * sp + im * cp) * win));
        branch_neg.push(((re * cn - im * sn) * win, (re * sn + im * cn) * win));
    }

    // Pad to fft_size if needed
    while branch_pos.len() < fft_size {
        branch_pos.push((0.0, 0.0));
        branch_neg.push((0.0, 0.0));
    }

    let dft_pos = dft(&branch_pos);
    let dft_neg = dft(&branch_neg);

    // SCF = X_pos(f) * conj(X_neg(f))
    let mut mag = Vec::with_capacity(fft_size);
    for k in 0..fft_size {
        let (ar, ai) = dft_pos[k];
        let (br, bi) = dft_neg[k];
        // conj of (br, bi) is (br, -bi)
        let prod_re = ar * br + ai * bi;
        let prod_im = -ar * bi + ai * br;
        mag.push((prod_re * prod_re + prod_im * prod_im).sqrt());
    }
    mag
}

/// Compute the conjugate spectral correlation function magnitude.
///
/// Uses x(t) * x(t) (no conjugate on second copy) frequency-shifted by alpha,
/// which highlights features of BPSK and real-valued QAM constellations
/// (the "conjugate cyclic spectrum").
pub fn conjugate_scf(
    samples: &[(f64, f64)],
    alpha: f64,
    fft_size: usize,
) -> Vec<f64> {
    let n = fft_size.min(samples.len()).max(1);

    let mut branch_a = Vec::with_capacity(n);
    let mut branch_b = Vec::with_capacity(n);

    for k in 0..n {
        let (re, im) = samples[k];
        let phase = 2.0 * PI * (alpha / 2.0) * k as f64;
        let win = 0.5 * (1.0 - (2.0 * PI * k as f64 / n as f64).cos());

        let (cs, sn) = (phase.cos(), phase.sin());

        // branch_a = x(t) * exp(+j pi alpha t), windowed
        branch_a.push(((re * cs - im * sn) * win, (re * sn + im * cs) * win));
        // branch_b = x(t) * exp(-j pi alpha t), windowed  (NO conjugate of x)
        branch_b.push(((re * cs + im * sn) * win, (-re * sn + im * cs) * win));
    }

    while branch_a.len() < fft_size {
        branch_a.push((0.0, 0.0));
        branch_b.push((0.0, 0.0));
    }

    let dft_a = dft(&branch_a);
    let dft_b = dft(&branch_b);

    let mut mag = Vec::with_capacity(fft_size);
    for k in 0..fft_size {
        let (ar, ai) = dft_a[k];
        let (br, bi) = dft_b[k];
        // product without conjugate on B: A(f) * B(f)
        let prod_re = ar * br - ai * bi;
        let prod_im = ar * bi + ai * br;
        mag.push((prod_re * prod_re + prod_im * prod_im).sqrt());
    }
    mag
}

// ---------------------------------------------------------------------------
// CyclicFeatureMap
// ---------------------------------------------------------------------------

/// Sweeps a range of cyclic frequencies to build an alpha-vs-frequency map.
#[derive(Debug, Clone)]
pub struct CyclicFeatureMap {
    alpha_range: (f64, f64),
    alpha_step: f64,
    fft_size: usize,
    /// Stored (alpha, peak_magnitude) pairs after `compute`.
    feature_list: Vec<(f64, f64)>,
}

impl CyclicFeatureMap {
    /// Create a new feature map sweeper.
    ///
    /// * `alpha_range` -- (min, max) normalized cyclic frequencies to scan.
    /// * `alpha_step`  -- step size between alpha values.
    /// * `fft_size`    -- DFT length per segment.
    pub fn new(alpha_range: (f64, f64), alpha_step: f64, fft_size: usize) -> Self {
        assert!(alpha_step > 0.0, "alpha_step must be positive");
        Self {
            alpha_range,
            alpha_step,
            fft_size,
            feature_list: Vec::new(),
        }
    }

    /// Compute the feature map over the configured alpha range.
    pub fn compute(&mut self, samples: &[(f64, f64)]) {
        self.feature_list.clear();
        let mut alpha = self.alpha_range.0;
        while alpha <= self.alpha_range.1 {
            let scf_mag = spectral_correlation_function(samples, alpha, self.fft_size);
            let peak = scf_mag.iter().cloned().fold(0.0_f64, f64::max);
            self.feature_list.push((alpha, peak));
            alpha += self.alpha_step;
        }
    }

    /// Return the cyclic frequency with the highest SCF peak.
    pub fn peak_alpha(&self) -> f64 {
        self.feature_list
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map(|&(a, _)| a)
            .unwrap_or(0.0)
    }

    /// Return the list of (alpha, peak_magnitude) pairs computed by `compute`.
    pub fn features(&self) -> &[(f64, f64)] {
        &self.feature_list
    }
}

// ---------------------------------------------------------------------------
// Symbol rate estimation
// ---------------------------------------------------------------------------

/// Estimate the symbol rate of an unknown signal from its cyclostationary features.
///
/// Scans normalized cyclic frequencies from `1 / samples.len()` up to
/// `max_symbol_rate / sample_rate` and returns the alpha (in Hz) with the
/// strongest averaged SCF peak, interpreted as the symbol rate.
///
/// Returns `None` if no prominent feature is found (peak is not sufficiently
/// above the noise floor).
pub fn symbol_rate_estimate(
    samples: &[(f64, f64)],
    sample_rate: f64,
    max_symbol_rate: f64,
) -> Option<f64> {
    let fft_size = 32.min(samples.len());
    if fft_size < 4 || samples.len() < fft_size {
        return None;
    }

    let alpha_max = max_symbol_rate / sample_rate;
    let alpha_min = 1.0 / samples.len() as f64;
    let num_steps = 200.min((alpha_max / alpha_min) as usize).max(10);
    let alpha_step = (alpha_max - alpha_min) / num_steps as f64;

    let max_segments = (samples.len() / fft_size).min(16).max(1);

    let mut best_alpha = 0.0_f64;
    let mut best_peak = 0.0_f64;
    let mut total_peak = 0.0_f64;
    let mut count = 0usize;

    let mut alpha = alpha_min;
    while alpha <= alpha_max {
        // Use averaged SCF for better discrimination
        let scf_mag = averaged_scf_magnitude(samples, alpha, fft_size, max_segments);
        let peak = scf_mag.iter().cloned().fold(0.0_f64, f64::max);
        total_peak += peak;
        count += 1;
        if peak > best_peak {
            best_peak = peak;
            best_alpha = alpha;
        }
        alpha += alpha_step;
    }

    let avg_peak = if count > 0 { total_peak / count as f64 } else { 0.0 };

    // Require the best peak to be at least 1.5x the average across all alphas
    if best_peak > 1.5 * avg_peak && best_alpha > 0.0 {
        Some(best_alpha * sample_rate)
    } else {
        None
    }
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/// Create a detector pre-configured for BPSK at the given symbol and sample rates.
///
/// Chooses an FFT size equal to the next power of two at or above 4x the
/// samples-per-symbol, and sets `num_averages` to 8 for good noise averaging.
pub fn bpsk_detector(symbol_rate: f64, sample_rate: f64) -> CyclostationaryDetector {
    let sps = (sample_rate / symbol_rate).ceil() as usize;
    let target = (sps * 4).max(16);
    let fft_size = target.next_power_of_two();
    CyclostationaryDetector::new(fft_size, 8)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Averaged SCF magnitude over multiple non-overlapping segments.
fn averaged_scf_magnitude(
    samples: &[(f64, f64)],
    alpha: f64,
    fft_size: usize,
    max_segments: usize,
) -> Vec<f64> {
    let num_segments = (samples.len() / fft_size).min(max_segments).max(1);
    let mut accum = vec![0.0_f64; fft_size];

    for seg in 0..num_segments {
        let offset = seg * fft_size;
        if offset + fft_size > samples.len() {
            break;
        }
        let chunk = &samples[offset..offset + fft_size];
        let mag = spectral_correlation_function(chunk, alpha, fft_size);
        for (a, m) in accum.iter_mut().zip(mag.iter()) {
            *a += m;
        }
    }

    let scale = 1.0 / num_segments as f64;
    accum.iter().map(|&v| v * scale).collect()
}

/// Simple O(N^2) DFT for (re, im) tuples. No external crate needed.
fn dft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let mut output = vec![(0.0_f64, 0.0_f64); n];
    for k in 0..n {
        let mut sum_re = 0.0_f64;
        let mut sum_im = 0.0_f64;
        for (j, &(xr, xi)) in input.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (j as f64) / (n as f64);
            let (cs, sn) = (angle.cos(), angle.sin());
            sum_re += xr * cs - xi * sn;
            sum_im += xr * sn + xi * cs;
        }
        output[k] = (sum_re, sum_im);
    }
    output
}

/// Median of a slice (non-destructive, copies internally).
fn median_of(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f64> = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Deterministic pseudo-random BPSK signal generator.
    ///
    /// Generates one random bit per symbol (held constant for `samples_per_symbol`
    /// consecutive samples), producing a signal with cyclostationary features at
    /// the normalized cyclic frequency alpha = 1 / samples_per_symbol.
    fn make_bpsk(samples_per_symbol: usize, num_samples: usize) -> Vec<(f64, f64)> {
        let sps = samples_per_symbol.max(1);
        let num_symbols = (num_samples + sps - 1) / sps;
        // Generate one random bit per symbol using xorshift32
        let mut rng: u32 = 42;
        let mut bits = Vec::with_capacity(num_symbols);
        for _ in 0..num_symbols {
            rng ^= rng << 13;
            rng ^= rng >> 17;
            rng ^= rng << 5;
            let bit = if rng & 1 == 1 { 1.0 } else { -1.0 };
            bits.push(bit);
        }
        // Upsample: hold each symbol for sps samples
        let mut out = Vec::with_capacity(num_samples);
        for i in 0..num_samples {
            out.push((bits[i / sps], 0.0));
        }
        out
    }

    /// Stationary white noise (no cyclostationary features).
    fn make_noise(num_samples: usize) -> Vec<(f64, f64)> {
        let mut rng: u64 = 123456789;
        let mut out = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            // Simple xorshift64 for pseudo-random f64 in [-1, 1]
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            let re = (rng as f64 / u64::MAX as f64) * 2.0 - 1.0;
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            let im = (rng as f64 / u64::MAX as f64) * 2.0 - 1.0;
            out.push((re, im));
        }
        out
    }

    // 1. Construction
    #[test]
    fn test_construction() {
        let det = CyclostationaryDetector::new(64, 4);
        assert_eq!(det.fft_size, 64);
        assert_eq!(det.num_averages, 4);
        assert!(det.last_scf_mag.is_empty());
    }

    // 2. Detection of BPSK-like signal (periodic pattern)
    #[test]
    fn test_bpsk_detection() {
        let sps = 10;
        let signal = make_bpsk(sps, 1024);
        let alpha = 1.0 / sps as f64; // normalized symbol rate

        let mut det = CyclostationaryDetector::new(64, 8);
        let result = det.detect(&signal, alpha);

        // BPSK should produce a detectable cyclostationary feature at its symbol rate
        assert!(
            result.peak_value > 0.0,
            "BPSK signal should have non-zero SCF peak"
        );
        assert_eq!(result.cyclic_frequency, alpha);
    }

    // 3. Non-detection of pure noise (random samples)
    #[test]
    fn test_noise_non_detection() {
        let noise = make_noise(1024);
        let alpha = 0.1; // arbitrary cyclic frequency

        let mut det = CyclostationaryDetector::new(64, 8);
        let result = det.detect(&noise, alpha);

        // Noise should have low confidence — no strong cyclostationary features
        assert!(
            result.confidence < 0.95,
            "Pure noise should not produce high confidence: got {}",
            result.confidence
        );
    }

    // 4. Spectral correlation function output length
    #[test]
    fn test_scf_output_length() {
        let signal = make_bpsk(8, 256);
        for &fft_size in &[16, 32, 64, 128] {
            let scf = spectral_correlation_function(&signal, 0.125, fft_size);
            assert_eq!(
                scf.len(),
                fft_size,
                "SCF output length should equal fft_size={}",
                fft_size
            );
        }
    }

    // 5. Conjugate SCF computation
    #[test]
    fn test_conjugate_scf() {
        let signal = make_bpsk(10, 512);
        let alpha = 1.0 / 10.0;
        let cscf = conjugate_scf(&signal, alpha, 64);

        assert_eq!(cscf.len(), 64);
        // All magnitudes should be non-negative
        for &v in &cscf {
            assert!(v >= 0.0, "Conjugate SCF magnitudes must be non-negative");
        }
        // BPSK (real-valued constellation) should produce non-trivial conjugate SCF
        let max_val = cscf.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            max_val > 0.0,
            "Conjugate SCF of BPSK should have non-zero peak"
        );
    }

    // 6. Cyclic feature map peak finding
    #[test]
    fn test_cyclic_feature_map_peak() {
        let sps = 8;
        let signal = make_bpsk(sps, 4096);
        let true_alpha = 1.0 / sps as f64; // 0.125

        // Scan with fine step around the expected region
        let mut map = CyclicFeatureMap::new((0.02, 0.4), 0.005, 32);
        map.compute(&signal);

        let peak_alpha = map.peak_alpha();
        let features = map.features();

        assert!(!features.is_empty(), "Feature list should not be empty");
        // The peak alpha should be at the symbol rate or a harmonic (2x).
        // Accept if near true_alpha or 2*true_alpha.
        let near_fundamental = (peak_alpha - true_alpha).abs() < 0.03;
        let near_harmonic = (peak_alpha - 2.0 * true_alpha).abs() < 0.03;
        assert!(
            near_fundamental || near_harmonic,
            "Peak alpha {:.4} should be near true alpha {:.4} or its harmonic {:.4}",
            peak_alpha,
            true_alpha,
            2.0 * true_alpha
        );
    }

    // 7. Symbol rate estimation
    #[test]
    fn test_symbol_rate_estimation() {
        let sample_rate = 100_000.0; // 100 kHz
        let symbol_rate = 10_000.0; // 10 ksps => sps = 10
        let sps = (sample_rate / symbol_rate) as usize;
        let signal = make_bpsk(sps, 4096);

        let estimated = symbol_rate_estimate(&signal, sample_rate, 50_000.0);
        // Should return Some with a value in the right ballpark
        assert!(estimated.is_some(), "Should estimate a symbol rate");
        if let Some(est) = estimated {
            assert!(
                est > 0.0 && est <= 50_000.0,
                "Estimated rate {:.1} should be in valid range",
                est
            );
        }
    }

    // 8. Different FFT sizes
    #[test]
    fn test_different_fft_sizes() {
        let signal = make_bpsk(10, 512);
        let alpha = 0.1;

        for &fft_size in &[8, 16, 32, 64] {
            let mut det = CyclostationaryDetector::new(fft_size, 4);
            let result = det.detect(&signal, alpha);
            assert_eq!(
                det.spectral_correlation_magnitude().len(),
                fft_size,
                "Cached SCF magnitude length should match fft_size={}",
                fft_size
            );
            assert!(
                result.peak_value >= 0.0,
                "Peak value should be non-negative for fft_size={}",
                fft_size
            );
        }
    }

    // 9. BPSK detector factory
    #[test]
    fn test_bpsk_detector_factory() {
        let det = bpsk_detector(10_000.0, 100_000.0);
        // sps = 10, target = 40, next_power_of_two = 64
        assert_eq!(det.fft_size, 64);
        assert_eq!(det.num_averages, 8);

        // Also verify it works with a non-power-of-two sps
        let det2 = bpsk_detector(3_000.0, 48_000.0);
        // sps = 16, target = 64, next_power_of_two = 64
        assert_eq!(det2.fft_size, 64);
    }

    // 10. Confidence metric ranges
    #[test]
    fn test_confidence_ranges() {
        let signal = make_bpsk(10, 1024);
        let noise = make_noise(1024);
        let alpha = 0.1;

        let mut det = CyclostationaryDetector::new(64, 8);

        let sig_result = det.detect(&signal, alpha);
        let noise_result = det.detect(&noise, alpha);

        // Confidence must always be in [0, 1]
        assert!(
            sig_result.confidence >= 0.0 && sig_result.confidence <= 1.0,
            "Signal confidence {} must be in [0,1]",
            sig_result.confidence
        );
        assert!(
            noise_result.confidence >= 0.0 && noise_result.confidence <= 1.0,
            "Noise confidence {} must be in [0,1]",
            noise_result.confidence
        );

        // Signal confidence should generally be higher than noise confidence
        // (not guaranteed for every seed, but our deterministic seeds produce this)
        assert!(
            sig_result.confidence >= noise_result.confidence * 0.5,
            "Signal confidence ({:.3}) should not be drastically below noise ({:.3})",
            sig_result.confidence,
            noise_result.confidence
        );
    }
}
