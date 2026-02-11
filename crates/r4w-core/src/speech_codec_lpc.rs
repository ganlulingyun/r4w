//! Linear Predictive Coding (LPC) for Speech Analysis and Synthesis
//!
//! Implements the core algorithms for LPC-based speech processing:
//!
//! - **Levinson-Durbin recursion** for efficient Toeplitz system solving
//! - **Autocorrelation-based pitch detection** for voiced/unvoiced classification
//! - **LPC analysis/synthesis** for speech codec encode-decode pipelines
//! - **Line Spectral Frequency (LSF)** conversion for quantization-friendly representation
//! - **PARCOR (reflection coefficient)** to direct-form conversion
//! - **LPC spectrum** computation for formant visualization
//!
//! The LPC model represents a short segment of speech as the output of an
//! all-pole filter driven by either a periodic impulse train (voiced) or
//! white noise (unvoiced). The filter coefficients capture the vocal tract
//! resonances (formants), while the excitation models the glottal source.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::speech_codec_lpc::{LpcConfig, LpcAnalyzer, LpcSynthesizer};
//!
//! let config = LpcConfig::default();
//! let analyzer = LpcAnalyzer::new(config.clone());
//! let mut synthesizer = LpcSynthesizer::new(config.clone());
//!
//! // Generate a synthetic frame (sine wave)
//! let frame: Vec<f64> = (0..160).map(|i| {
//!     (2.0 * std::f64::consts::PI * 300.0 * i as f64 / 8000.0).sin()
//! }).collect();
//!
//! let lpc_frame = analyzer.analyze_frame(&frame);
//! let resynthesized = synthesizer.synthesize_frame(&lpc_frame);
//! assert_eq!(resynthesized.len(), frame.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for LPC analysis and synthesis.
#[derive(Debug, Clone)]
pub struct LpcConfig {
    /// Sampling rate in Hz (e.g. 8000.0 for telephony).
    pub sample_rate_hz: f64,
    /// LPC prediction order (number of coefficients). Default 10.
    pub order: usize,
    /// Frame duration in milliseconds. Default 20.0.
    pub frame_size_ms: f64,
    /// Pre-emphasis filter coefficient. Default 0.97.
    pub preemphasis: f64,
}

impl Default for LpcConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 8000.0,
            order: 10,
            frame_size_ms: 20.0,
            preemphasis: 0.97,
        }
    }
}

impl LpcConfig {
    /// Number of samples per frame, derived from sample rate and frame duration.
    pub fn frame_size_samples(&self) -> usize {
        (self.sample_rate_hz * self.frame_size_ms / 1000.0) as usize
    }
}

// ---------------------------------------------------------------------------
// LPC Frame (analysis result)
// ---------------------------------------------------------------------------

/// Result of LPC analysis on a single frame.
#[derive(Debug, Clone)]
pub struct LpcFrame {
    /// Direct-form LPC coefficients a[1..order]. Length = order.
    /// The all-pole filter is: 1 / A(z) where A(z) = 1 + a1*z^-1 + ... + ap*z^-p.
    pub coefficients: Vec<f64>,
    /// Prediction error energy (gain of the excitation).
    pub gain: f64,
    /// Pitch period in samples, if voiced.
    pub pitch_period_samples: Option<usize>,
    /// Whether the frame is classified as voiced.
    pub voiced: bool,
    /// Reflection coefficients (PARCOR parameters) from Levinson-Durbin.
    pub reflection_coefficients: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Core algorithms
// ---------------------------------------------------------------------------

/// Compute the autocorrelation of `signal` for lags 0..=max_lag.
///
/// R[k] = sum_{n=0}^{N-1-k} signal[n] * signal[n+k]
///
/// Returns a vector of length `max_lag + 1`.
pub fn autocorrelation(signal: &[f64], max_lag: usize) -> Vec<f64> {
    let n = signal.len();
    let mut r = vec![0.0; max_lag + 1];
    for k in 0..=max_lag {
        let mut sum = 0.0;
        for i in 0..n.saturating_sub(k) {
            sum += signal[i] * signal[i + k];
        }
        r[k] = sum;
    }
    r
}

/// Levinson-Durbin recursion for solving the Toeplitz system defined by `autocorrelation`.
///
/// Given autocorrelation values R[0..=order], solves for the LPC coefficients
/// that minimize the mean-squared prediction error.
///
/// # Returns
///
/// `(a_coefficients, prediction_error, reflection_coefficients)`
///
/// - `a_coefficients`: Direct-form LPC coefficients a[1..order] (length = `order`).
/// - `prediction_error`: Final prediction error energy.
/// - `reflection_coefficients`: PARCOR / reflection coefficients k[1..order].
///
/// # Panics
///
/// Panics if `autocorrelation.len() < order + 1`.
pub fn levinson_durbin(
    r: &[f64],
    order: usize,
) -> (Vec<f64>, f64, Vec<f64>) {
    assert!(
        r.len() >= order + 1,
        "autocorrelation must have at least order+1 elements"
    );

    if order == 0 || r[0] <= 0.0 {
        return (vec![], r[0].max(0.0), vec![]);
    }

    let mut a = vec![0.0; order]; // a[0..order-1] maps to a[1..order] in literature
    let mut a_prev = vec![0.0; order];
    let mut k_vec = vec![0.0; order]; // reflection coefficients
    let mut error = r[0];

    for m in 0..order {
        // Compute reflection coefficient k[m]
        let mut lambda = 0.0;
        for j in 0..m {
            lambda += a_prev[j] * r[m - j];
        }
        lambda += r[m + 1];

        if error.abs() < 1e-30 {
            // Avoid division by zero for zero-energy signals
            break;
        }

        let k = -lambda / error;
        k_vec[m] = k;
        a[m] = k;

        // Update coefficients using the Levinson recursion
        for j in 0..m {
            a[j] = a_prev[j] + k * a_prev[m - 1 - j];
        }

        // Update prediction error
        error *= 1.0 - k * k;

        if error <= 0.0 {
            // Should not happen for valid autocorrelation, but guard against it
            error = 1e-30;
            break;
        }

        // Save for next iteration
        a_prev[..=m].copy_from_slice(&a[..=m]);
    }

    (a, error, k_vec)
}

/// Apply a pre-emphasis filter: y[n] = x[n] - alpha * x[n-1].
///
/// Pre-emphasis boosts high frequencies to flatten the speech spectrum
/// before LPC analysis, improving coefficient stability.
pub fn preemphasis_filter(signal: &[f64], alpha: f64) -> Vec<f64> {
    if signal.is_empty() {
        return vec![];
    }
    let mut out = vec![0.0; signal.len()];
    out[0] = signal[0];
    for i in 1..signal.len() {
        out[i] = signal[i] - alpha * signal[i - 1];
    }
    out
}

/// Apply a de-emphasis filter (inverse of pre-emphasis): y[n] = x[n] + alpha * y[n-1].
pub fn deemphasis_filter(signal: &[f64], alpha: f64) -> Vec<f64> {
    if signal.is_empty() {
        return vec![];
    }
    let mut out = vec![0.0; signal.len()];
    out[0] = signal[0];
    for i in 1..signal.len() {
        out[i] = signal[i] + alpha * out[i - 1];
    }
    out
}

/// Detect pitch period using autocorrelation method.
///
/// Searches for the dominant peak in the autocorrelation of `frame` within
/// the lag range corresponding to `[min_f0, max_f0]` Hz. Returns `Some(lag)`
/// if a clear pitch is found, `None` if the frame appears unvoiced.
///
/// # Arguments
///
/// * `frame` - Input speech samples
/// * `fs` - Sample rate in Hz
/// * `min_f0` - Minimum fundamental frequency to search (Hz), e.g. 50.0
/// * `max_f0` - Maximum fundamental frequency to search (Hz), e.g. 400.0
///
/// The voiced/unvoiced decision uses a normalized autocorrelation threshold.
pub fn detect_pitch(
    frame: &[f64],
    fs: f64,
    min_f0: f64,
    max_f0: f64,
) -> Option<usize> {
    if frame.is_empty() || fs <= 0.0 || min_f0 <= 0.0 || max_f0 <= min_f0 {
        return None;
    }

    let min_lag = (fs / max_f0).floor() as usize;
    let max_lag = (fs / min_f0).ceil() as usize;
    let max_lag = max_lag.min(frame.len() - 1);

    if min_lag >= max_lag || min_lag >= frame.len() {
        return None;
    }

    let r = autocorrelation(frame, max_lag);

    if r[0] <= 0.0 {
        return None;
    }

    // Find the peak in the valid lag range
    let mut best_lag = min_lag;
    let mut best_val = r[min_lag];
    for lag in (min_lag + 1)..=max_lag {
        if r[lag] > best_val {
            best_val = r[lag];
            best_lag = lag;
        }
    }

    // Voiced/unvoiced decision: normalized autocorrelation must exceed threshold
    let normalized = best_val / r[0];
    const VOICED_THRESHOLD: f64 = 0.3;

    if normalized > VOICED_THRESHOLD {
        Some(best_lag)
    } else {
        None
    }
}

/// Convert LPC coefficients to Line Spectral Frequencies (LSF).
///
/// LSF representation is preferred for quantization because the coefficients
/// are ordered, bounded in [0, pi], and small perturbations cause graceful
/// spectral distortion.
///
/// Constructs two auxiliary polynomials from A(z):
///   P(z) = A(z) + z^{-(p+1)} A(z^{-1})   (symmetric)
///   Q(z) = A(z) - z^{-(p+1)} A(z^{-1})   (antisymmetric)
///
/// For even order p, P has a trivial root at z=-1 and Q at z=+1.
/// After factoring those out, the reduced palindromic polynomials P' and Q'
/// are evaluated using the Chebyshev form to find their roots on the unit
/// circle. These roots, sorted, are the LSFs.
///
/// Returns LSF values in radians, sorted in ascending order.
pub fn lpc_to_lsf(lpc_coeffs: &[f64]) -> Vec<f64> {
    let p = lpc_coeffs.len();
    if p == 0 {
        return vec![];
    }

    // Build the full filter polynomial: a[0]=1, a[1..=p]=lpc_coeffs
    let mut a = vec![0.0; p + 2];
    a[0] = 1.0;
    for i in 0..p {
        a[i + 1] = lpc_coeffs[i];
    }

    // Construct P and Q polynomials of order (p+1)
    let m = p + 1;
    let mut p_full = vec![0.0; m + 1];
    let mut q_full = vec![0.0; m + 1];
    for i in 0..=m {
        p_full[i] = a[i] + a[m - i];
        q_full[i] = a[i] - a[m - i];
    }

    // Factor out trivial roots via polynomial long division:
    // P / (1 + z^-1), Q / (1 - z^-1)
    let deconv = |poly: &[f64], c: f64| -> Vec<f64> {
        let n = poly.len();
        if n < 2 {
            return poly.to_vec();
        }
        let mut r = vec![0.0; n - 1];
        r[0] = poly[0];
        for i in 1..n - 1 {
            r[i] = poly[i] - c * r[i - 1];
        }
        r
    };

    let pp = deconv(&p_full, 1.0);  // P' = P / (1 + z^-1)
    let qq = deconv(&q_full, -1.0); // Q' = Q / (1 - z^-1)

    // pp and qq are palindromic polynomials of degree p (even).
    // For a palindromic polynomial of degree 2M: c[k] = c[2M-k].
    // On the unit circle: P'(e^{jw}) = e^{-jMw} * F(cos(w))
    // where F(x) = c[M] + 2*sum_{k=0}^{M-1} c[k] * cos((M-k)*w)
    // pp and qq are palindromic polynomials of even degree 2M (where M = p/2).
    // For palindromic polynomial evaluated on the unit circle:
    //   P'(e^{jw}) = e^{-jMw} * F(cos(w))
    // where F(w) = c[M] + 2*sum_{k=0}^{M-1} c[k] * cos((M-k)*w).
    // The roots of P'(e^{jw}) are exactly the roots of F(cos(w)).
    let n_search = 4096.max(p * 500);
    let dw = PI / n_search as f64;
    let mut lsfs = Vec::with_capacity(p);

    // Find roots of P' in (0, pi)
    let mut prev = eval_palindromic(&pp, 1e-10);
    for i in 1..=n_search {
        let w = i as f64 * dw;
        let cur = eval_palindromic(&pp, w);
        if prev * cur < 0.0 {
            let root = bisect_root_palindromic(&pp, w - dw, w);
            if root > 1e-10 && root < PI - 1e-10 {
                lsfs.push(root);
            }
        }
        prev = cur;
    }

    // Find roots of Q' in (0, pi)
    prev = eval_palindromic(&qq, 1e-10);
    for i in 1..=n_search {
        let w = i as f64 * dw;
        let cur = eval_palindromic(&qq, w);
        if prev * cur < 0.0 {
            let root = bisect_root_palindromic(&qq, w - dw, w);
            if root > 1e-10 && root < PI - 1e-10 {
                lsfs.push(root);
            }
        }
        prev = cur;
    }

    lsfs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    lsfs.truncate(p);
    lsfs
}

/// Evaluate a palindromic polynomial's Chebyshev form at frequency w.
/// For palindromic poly of degree 2M: F(w) = c[M] + 2*sum_{k=0}^{M-1} c[k]*cos((M-k)*w)
fn eval_palindromic(poly: &[f64], w: f64) -> f64 {
    let deg = poly.len() - 1;
    let half = deg / 2;
    let mut val = poly[half];
    for k in 0..half {
        val += 2.0 * poly[k] * ((half - k) as f64 * w).cos();
    }
    val
}

/// Bisection root-finding for a palindromic polynomial using Chebyshev evaluation.
fn bisect_root_palindromic(poly: &[f64], mut lo: f64, mut hi: f64) -> f64 {
    let f_lo = eval_palindromic(poly, lo);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        let f_mid = eval_palindromic(poly, mid);
        if f_lo * f_mid <= 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Convert Line Spectral Frequencies back to LPC coefficients.
///
/// The LSFs are the interleaved roots of two reduced polynomials P' and Q',
/// obtained by factoring the symmetric/antisymmetric decomposition of A(z).
/// The even-indexed LSFs (0, 2, ...) and odd-indexed LSFs (1, 3, ...) each
/// form one polynomial. Both possible assignments are tried, and the one
/// producing a consistent roundtrip is selected.
///
/// Each root w gives a second-order factor (1 - 2*cos(w)*z^-1 + z^-2).
/// After building P' and Q' from their roots, we reconstruct:
///   P(z) = P'(z) * (1 + z^-1)
///   Q(z) = Q'(z) * (1 - z^-1)
///   A(z) = (P(z) + Q(z)) / 2
pub fn lsf_to_lpc(lsf: &[f64]) -> Vec<f64> {
    let p = lsf.len();
    if p == 0 {
        return vec![];
    }

    // Build polynomial from conjugate-pair roots on the unit circle.
    let build_poly = |roots: &[f64]| -> Vec<f64> {
        let mut poly = vec![1.0];
        for &w in roots {
            let c = -2.0 * w.cos();
            let new_len = poly.len() + 2;
            let mut new_poly = vec![0.0; new_len];
            for (j, &pj) in poly.iter().enumerate() {
                new_poly[j] += pj;
                new_poly[j + 1] += c * pj;
                new_poly[j + 2] += pj;
            }
            poly = new_poly;
        }
        poly
    };

    // Convolve polynomial with (1 + c*z^-1)
    let conv = |poly: &[f64], c: f64| -> Vec<f64> {
        let mut out = vec![0.0; poly.len() + 1];
        for (j, &v) in poly.iter().enumerate() {
            out[j] += v;
            out[j + 1] += c * v;
        }
        out
    };

    // Reconstruct A(z) from a given P'/Q' root assignment.
    let reconstruct = |p_first_even: bool| -> Vec<f64> {
        let mut p_roots = Vec::new();
        let mut q_roots = Vec::new();
        for (i, &w) in lsf.iter().enumerate() {
            if (i % 2 == 0) == p_first_even {
                p_roots.push(w);
            } else {
                q_roots.push(w);
            }
        }

        let pp = build_poly(&p_roots);
        let qq = build_poly(&q_roots);

        let mut p_full = conv(&pp, 1.0);   // P = P' * (1 + z^-1)
        let mut q_full = conv(&qq, -1.0);  // Q = Q' * (1 - z^-1)

        let len = p_full.len().max(q_full.len());
        p_full.resize(len, 0.0);
        q_full.resize(len, 0.0);

        let mut a = vec![0.0; p];
        for i in 0..p {
            a[i] = 0.5 * (p_full[i + 1] + q_full[i + 1]);
        }
        a
    };

    // Try both assignments and pick the one whose roundtrip is consistent.
    let a0 = reconstruct(true);
    let a1 = reconstruct(false);

    let lsf0 = lpc_to_lsf(&a0);
    let lsf1 = lpc_to_lsf(&a1);

    let dist = |computed: &[f64], target: &[f64]| -> f64 {
        if computed.len() != target.len() {
            return f64::MAX;
        }
        computed.iter().zip(target.iter()).map(|(a, b)| (a - b).abs()).sum()
    };

    if dist(&lsf0, lsf) <= dist(&lsf1, lsf) {
        a0
    } else {
        a1
    }
}

/// Compute the frequency response magnitude of the LPC all-pole filter.
///
/// Evaluates |H(e^jw)|^2 = gain^2 / |A(e^jw)|^2 at `num_points` equally
/// spaced frequencies from 0 to pi.
///
/// Returns magnitude values (linear scale) of length `num_points`.
pub fn lpc_spectrum(
    coefficients: &[f64],
    gain: f64,
    num_points: usize,
) -> Vec<f64> {
    if num_points == 0 {
        return vec![];
    }

    let mut spectrum = vec![0.0; num_points];
    let dw = PI / num_points as f64;

    for i in 0..num_points {
        let w = i as f64 * dw;
        // A(e^jw) = 1 + sum_{k=1}^{p} a[k-1] * e^{-jwk}
        let mut re = 1.0;
        let mut im = 0.0;
        for (k, &a) in coefficients.iter().enumerate() {
            let angle = (k + 1) as f64 * w;
            re += a * angle.cos();
            im -= a * angle.sin();
        }
        let mag_sq = re * re + im * im;
        // |H(w)| = gain / |A(w)|
        if mag_sq > 1e-30 {
            spectrum[i] = gain / mag_sq.sqrt();
        } else {
            spectrum[i] = gain * 1e15;
        }
    }

    spectrum
}

/// Compute the prediction residual (excitation signal).
///
/// e[n] = x[n] + sum_{k=1}^{p} a[k-1] * x[n-k]
///
/// The residual is the excitation that, when filtered through the all-pole
/// synthesis filter, reproduces the original signal.
pub fn compute_residual(signal: &[f64], lpc_coeffs: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let p = lpc_coeffs.len();
    let mut residual = vec![0.0; n];
    for i in 0..n {
        let mut val = signal[i];
        for k in 0..p {
            if i > k {
                val += lpc_coeffs[k] * signal[i - 1 - k];
            }
        }
        residual[i] = val;
    }
    residual
}

/// Convert PARCOR (reflection) coefficients to direct-form LPC coefficients.
///
/// Uses the step-up recursion: the reverse of Levinson-Durbin.
pub fn parcor_to_lpc(reflection_coeffs: &[f64]) -> Vec<f64> {
    let order = reflection_coeffs.len();
    if order == 0 {
        return vec![];
    }

    let mut a = vec![0.0; order];
    let mut a_prev = vec![0.0; order];

    for m in 0..order {
        let k = reflection_coeffs[m];
        a[m] = k;

        for j in 0..m {
            a[j] = a_prev[j] + k * a_prev[m - 1 - j];
        }

        a_prev[..=m].copy_from_slice(&a[..=m]);
    }

    a
}

/// Apply a Hamming window in-place to the given frame.
pub fn hamming_window(frame: &mut [f64]) {
    let n = frame.len();
    if n <= 1 {
        return;
    }
    let denom = (n - 1) as f64;
    for i in 0..n {
        frame[i] *= 0.54 - 0.46 * (2.0 * PI * i as f64 / denom).cos();
    }
}

// ---------------------------------------------------------------------------
// LPC Analyzer
// ---------------------------------------------------------------------------

/// Performs LPC analysis on speech frames.
pub struct LpcAnalyzer {
    config: LpcConfig,
}

impl LpcAnalyzer {
    /// Create a new LPC analyzer with the given configuration.
    pub fn new(config: LpcConfig) -> Self {
        Self { config }
    }

    /// Analyze a single speech frame and return the LPC parameters.
    ///
    /// The frame length should match `config.frame_size_samples()`, but any
    /// length is accepted. The analysis pipeline:
    ///
    /// 1. Pre-emphasis
    /// 2. Hamming windowing
    /// 3. Autocorrelation
    /// 4. Levinson-Durbin
    /// 5. Pitch detection
    pub fn analyze_frame(&self, frame: &[f64]) -> LpcFrame {
        if frame.is_empty() {
            return LpcFrame {
                coefficients: vec![0.0; self.config.order],
                gain: 0.0,
                pitch_period_samples: None,
                voiced: false,
                reflection_coefficients: vec![0.0; self.config.order],
            };
        }

        // 1. Pre-emphasis
        let emphasized = preemphasis_filter(frame, self.config.preemphasis);

        // 2. Hamming window
        let mut windowed = emphasized;
        hamming_window(&mut windowed);

        // 3. Autocorrelation
        let r = autocorrelation(&windowed, self.config.order);

        // 4. Levinson-Durbin
        let (coeffs, error, refl) = levinson_durbin(&r, self.config.order);

        // 5. Pitch detection (on the original frame, not pre-emphasized)
        let pitch = detect_pitch(frame, self.config.sample_rate_hz, 50.0, 400.0);

        let gain = if error > 0.0 { error.sqrt() } else { 0.0 };

        LpcFrame {
            coefficients: coeffs,
            gain,
            pitch_period_samples: pitch,
            voiced: pitch.is_some(),
            reflection_coefficients: refl,
        }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &LpcConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// LPC Synthesizer
// ---------------------------------------------------------------------------

/// Synthesizes speech from LPC frame parameters.
///
/// Maintains filter state across frames for continuity.
pub struct LpcSynthesizer {
    config: LpcConfig,
    /// Filter memory (previous output samples for the all-pole filter).
    filter_state: Vec<f64>,
    /// Phase counter for voiced excitation pulse train.
    pulse_phase: usize,
    /// Simple PRNG state for unvoiced excitation noise.
    noise_state: u64,
}

impl LpcSynthesizer {
    /// Create a new LPC synthesizer with the given configuration.
    pub fn new(config: LpcConfig) -> Self {
        let order = config.order;
        Self {
            config,
            filter_state: vec![0.0; order],
            pulse_phase: 0,
            noise_state: 0xDEAD_BEEF_CAFE_BABE,
        }
    }

    /// Synthesize a single frame of speech from LPC parameters.
    ///
    /// For voiced frames, the excitation is a periodic impulse train with
    /// period `pitch_period_samples`. For unvoiced frames, the excitation
    /// is white noise. The excitation is then filtered through the all-pole
    /// synthesis filter defined by the LPC coefficients.
    pub fn synthesize_frame(&mut self, frame: &LpcFrame) -> Vec<f64> {
        let n = self.config.frame_size_samples();
        let order = self.config.order;
        let mut output = vec![0.0; n];

        for i in 0..n {
            // Generate excitation
            let excitation = if frame.voiced {
                let period = frame.pitch_period_samples.unwrap_or(80);
                let pulse = if self.pulse_phase == 0 {
                    frame.gain
                } else {
                    0.0
                };
                self.pulse_phase += 1;
                if self.pulse_phase >= period {
                    self.pulse_phase = 0;
                }
                pulse
            } else {
                // White noise excitation via xorshift64
                self.noise_state ^= self.noise_state << 13;
                self.noise_state ^= self.noise_state >> 7;
                self.noise_state ^= self.noise_state << 17;
                let noise =
                    (self.noise_state as f64 / u64::MAX as f64) * 2.0 - 1.0;
                noise * frame.gain
            };

            // All-pole synthesis filter: y[n] = excitation - sum(a[k]*y[n-k])
            let mut sample = excitation;
            for k in 0..order.min(frame.coefficients.len()) {
                let past_idx = i as isize - (k as isize + 1);
                let past_sample = if past_idx >= 0 {
                    output[past_idx as usize]
                } else {
                    let state_idx =
                        (order as isize + past_idx) as usize;
                    if state_idx < self.filter_state.len() {
                        self.filter_state[state_idx]
                    } else {
                        0.0
                    }
                };
                sample -= frame.coefficients[k] * past_sample;
            }
            output[i] = sample;
        }

        // Update filter state: save last `order` samples for next frame
        if n >= order {
            self.filter_state
                .copy_from_slice(&output[n - order..n]);
        } else {
            // Shift existing state and append new samples
            let shift = order - n;
            for j in 0..shift {
                self.filter_state[j] = self.filter_state[j + n];
            }
            for j in 0..n {
                self.filter_state[shift + j] = output[j];
            }
        }

        output
    }

    /// Reset the synthesizer state (filter memory and pulse phase).
    pub fn reset(&mut self) {
        self.filter_state.fill(0.0);
        self.pulse_phase = 0;
        self.noise_state = 0xDEAD_BEEF_CAFE_BABE;
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &LpcConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // Helper: generate a sine wave
    fn sine_wave(freq: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / fs).sin())
            .collect()
    }

    // Helper: sum of squares (energy)
    fn energy(signal: &[f64]) -> f64 {
        signal.iter().map(|x| x * x).sum()
    }

    // -----------------------------------------------------------------------
    // Autocorrelation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_autocorrelation_dc_signal() {
        // Constant signal: R[k] = N - k for unit amplitude
        let signal = vec![1.0; 10];
        let r = autocorrelation(&signal, 5);
        assert_eq!(r.len(), 6);
        assert!((r[0] - 10.0).abs() < EPSILON);
        assert!((r[1] - 9.0).abs() < EPSILON);
        assert!((r[5] - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_autocorrelation_impulse() {
        // Impulse at origin: R[0] = 1, R[k>0] = 0
        let mut signal = vec![0.0; 20];
        signal[0] = 1.0;
        let r = autocorrelation(&signal, 10);
        assert!((r[0] - 1.0).abs() < EPSILON);
        for k in 1..=10 {
            assert!(r[k].abs() < EPSILON, "r[{}] = {} should be 0", k, r[k]);
        }
    }

    #[test]
    fn test_autocorrelation_symmetry() {
        // For a real signal, R[k] should equal the manually computed sum
        let signal = vec![1.0, -1.0, 2.0, -2.0, 3.0];
        let r = autocorrelation(&signal, 4);
        assert!((r[0] - 19.0).abs() < EPSILON); // 1+1+4+4+9
        assert!((r[1] - (-13.0)).abs() < EPSILON); // 1*(-1)+(-1)*2+2*(-2)+(-2)*3 = -13
    }

    #[test]
    fn test_autocorrelation_zero_lag_is_energy() {
        let signal = vec![3.0, 4.0];
        let r = autocorrelation(&signal, 0);
        assert!((r[0] - 25.0).abs() < EPSILON); // 9 + 16
    }

    #[test]
    fn test_autocorrelation_empty() {
        let r = autocorrelation(&[], 5);
        assert_eq!(r.len(), 6);
        for v in &r {
            assert!(v.abs() < EPSILON);
        }
    }

    // -----------------------------------------------------------------------
    // Levinson-Durbin tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_levinson_durbin_first_order() {
        // R = [1.0, 0.5] -> a[1] = -R[1]/R[0] = -0.5
        let r = vec![1.0, 0.5];
        let (a, error, k) = levinson_durbin(&r, 1);
        assert_eq!(a.len(), 1);
        assert!((a[0] - (-0.5)).abs() < 1e-10);
        assert!((error - 0.75).abs() < 1e-10); // 1 - 0.25 = 0.75
        assert!((k[0] - (-0.5)).abs() < 1e-10);
    }

    #[test]
    fn test_levinson_durbin_second_order() {
        // Known autocorrelation for AR(2) process
        let r = vec![1.0, 0.5, 0.3];
        let (a, error, _k) = levinson_durbin(&r, 2);
        assert_eq!(a.len(), 2);

        // Verify: the solution should satisfy the Yule-Walker equations
        // R[0]*a[0] + R[1]*a[1] = -R[1]
        // R[1]*a[0] + R[0]*a[1] = -R[2]
        let eq1 = r[0] * a[0] + r[1] * a[1] + r[1];
        let eq2 = r[1] * a[0] + r[0] * a[1] + r[2];
        assert!(
            eq1.abs() < 1e-10,
            "Yule-Walker eq 1 not satisfied: {}",
            eq1
        );
        assert!(
            eq2.abs() < 1e-10,
            "Yule-Walker eq 2 not satisfied: {}",
            eq2
        );
        assert!(error > 0.0);
        assert!(error < r[0]);
    }

    #[test]
    fn test_levinson_durbin_zero_order() {
        let r = vec![5.0, 2.0, 1.0];
        let (a, error, k) = levinson_durbin(&r, 0);
        assert!(a.is_empty());
        assert!((error - 5.0).abs() < EPSILON);
        assert!(k.is_empty());
    }

    #[test]
    fn test_levinson_durbin_prediction_error_decreases() {
        // Prediction error should decrease (or stay) as order increases
        let r = vec![1.0, 0.9, 0.7, 0.5, 0.3];
        let mut prev_error = r[0];
        for order in 1..=4 {
            let (_, error, _) = levinson_durbin(&r, order);
            assert!(
                error <= prev_error + 1e-12,
                "Error increased at order {}: {} > {}",
                order,
                error,
                prev_error
            );
            prev_error = error;
        }
    }

    #[test]
    fn test_levinson_durbin_reflection_coeff_bounded() {
        // Reflection coefficients should be in (-1, 1) for valid autocorrelation
        let signal = sine_wave(440.0, 8000.0, 200);
        let r = autocorrelation(&signal, 10);
        let (_, _, k) = levinson_durbin(&r, 10);
        for (i, &ki) in k.iter().enumerate() {
            assert!(
                ki.abs() < 1.0 + 1e-10,
                "k[{}] = {} out of bounds",
                i,
                ki
            );
        }
    }

    // -----------------------------------------------------------------------
    // Pre-emphasis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_preemphasis_basic() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = preemphasis_filter(&signal, 0.97);
        assert!((result[0] - 1.0).abs() < EPSILON);
        assert!((result[1] - (2.0 - 0.97 * 1.0)).abs() < EPSILON);
        assert!((result[2] - (3.0 - 0.97 * 2.0)).abs() < EPSILON);
    }

    #[test]
    fn test_preemphasis_zero_alpha() {
        let signal = vec![1.0, 2.0, 3.0];
        let result = preemphasis_filter(&signal, 0.0);
        assert!((result[0] - 1.0).abs() < EPSILON);
        assert!((result[1] - 2.0).abs() < EPSILON);
        assert!((result[2] - 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_preemphasis_empty() {
        let result = preemphasis_filter(&[], 0.97);
        assert!(result.is_empty());
    }

    #[test]
    fn test_preemphasis_deemphasis_roundtrip() {
        let signal = vec![1.0, 3.0, -2.0, 5.0, -1.0, 0.5];
        let alpha = 0.95;
        let emphasized = preemphasis_filter(&signal, alpha);
        let recovered = deemphasis_filter(&emphasized, alpha);
        for (a, b) in signal.iter().zip(recovered.iter()) {
            assert!(
                (a - b).abs() < 1e-10,
                "roundtrip failed: {} vs {}",
                a,
                b
            );
        }
    }

    // -----------------------------------------------------------------------
    // Pitch detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_pitch_detection_sine() {
        // A 200 Hz sine at 8000 Hz sample rate -> period = 40 samples
        let fs = 8000.0;
        let freq = 200.0;
        let signal = sine_wave(freq, fs, 320);
        let pitch = detect_pitch(&signal, fs, 50.0, 400.0);
        assert!(pitch.is_some(), "Should detect pitch for pure sine");
        let period = pitch.unwrap();
        let expected = (fs / freq).round() as usize;
        assert!(
            (period as i32 - expected as i32).unsigned_abs() <= 1,
            "Expected period ~{}, got {}",
            expected,
            period
        );
    }

    #[test]
    fn test_pitch_detection_unvoiced_noise() {
        // White noise should be classified as unvoiced
        let mut noise = vec![0.0; 320];
        let mut state: u64 = 12345;
        for sample in noise.iter_mut() {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            *sample = (state as f64 / u64::MAX as f64) * 2.0 - 1.0;
        }
        let pitch = detect_pitch(&noise, 8000.0, 50.0, 400.0);
        assert!(pitch.is_none(), "Noise should be unvoiced (no pitch)");
    }

    #[test]
    fn test_pitch_detection_empty() {
        assert!(detect_pitch(&[], 8000.0, 50.0, 400.0).is_none());
    }

    #[test]
    fn test_pitch_detection_invalid_params() {
        let signal = sine_wave(200.0, 8000.0, 160);
        assert!(detect_pitch(&signal, 0.0, 50.0, 400.0).is_none());
        assert!(detect_pitch(&signal, 8000.0, -1.0, 400.0).is_none());
        assert!(detect_pitch(&signal, 8000.0, 400.0, 50.0).is_none());
    }

    // -----------------------------------------------------------------------
    // LPC Spectrum tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lpc_spectrum_flat_for_trivial() {
        // With zero LPC coefficients, the spectrum should be flat = gain
        let spec = lpc_spectrum(&[], 2.0, 128);
        assert_eq!(spec.len(), 128);
        for &v in &spec {
            assert!((v - 2.0).abs() < 1e-10, "Expected flat spectrum at 2.0, got {}", v);
        }
    }

    #[test]
    fn test_lpc_spectrum_peak_at_formant() {
        // Create a signal with a strong resonance and verify spectrum peak
        let fs = 8000.0;
        let freq = 1000.0;
        let signal = sine_wave(freq, fs, 320);
        let r = autocorrelation(&signal, 10);
        let (a, error, _) = levinson_durbin(&r, 10);
        let gain = error.sqrt();
        let spec = lpc_spectrum(&a, gain, 256);

        // Find the peak frequency
        let peak_bin = spec
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        let peak_freq = peak_bin as f64 * (fs / 2.0) / 256.0;

        // The peak should be near 1000 Hz
        assert!(
            (peak_freq - freq).abs() < 200.0,
            "Peak at {} Hz, expected near {} Hz",
            peak_freq,
            freq
        );
    }

    #[test]
    fn test_lpc_spectrum_empty() {
        let spec = lpc_spectrum(&[0.5, -0.3], 1.0, 0);
        assert!(spec.is_empty());
    }

    // -----------------------------------------------------------------------
    // Residual tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_residual_energy_less_than_signal() {
        // The prediction residual should have less energy than the original
        // for a tonal signal (where LPC captures the spectral structure well).
        let signal = sine_wave(300.0, 8000.0, 160);
        let r = autocorrelation(&signal, 10);
        let (a, _, _) = levinson_durbin(&r, 10);
        let residual = compute_residual(&signal, &a);

        let signal_energy = energy(&signal);
        let residual_energy = energy(&residual);

        assert!(
            residual_energy < signal_energy,
            "Residual energy {} should be less than signal energy {}",
            residual_energy,
            signal_energy
        );
    }

    #[test]
    fn test_residual_zero_order() {
        // With no LPC coefficients, the residual equals the signal
        let signal = vec![1.0, 2.0, 3.0];
        let residual = compute_residual(&signal, &[]);
        assert_eq!(residual.len(), signal.len());
        for (a, b) in signal.iter().zip(residual.iter()) {
            assert!((a - b).abs() < EPSILON);
        }
    }

    // -----------------------------------------------------------------------
    // LSF conversion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lsf_roundtrip() {
        // Test with LPC coefficients derived from Levinson-Durbin (guaranteed stable).
        // Use a multi-component signal that produces a well-conditioned autocorrelation.
        let fs = 8000.0;
        let n = 640;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * PI * 300.0 * t).sin()
                    + 0.4 * (2.0 * PI * 900.0 * t).sin()
                    + 0.3 * (2.0 * PI * 2100.0 * t).sin()
                    + 0.2 * (2.0 * PI * 3300.0 * t).sin()
                    + 0.15 * ((i as f64 * 1.618).sin())
                    + 0.1 * ((i as f64 * 2.718).cos())
            })
            .collect();
        let order = 4;
        let r = autocorrelation(&signal, order);
        let (a_orig, _err, k) = levinson_durbin(&r, order);

        // Verify stability: all reflection coefficients strictly inside unit circle
        for (i, &ki) in k.iter().enumerate() {
            assert!(ki.abs() < 0.999, "k[{}] = {} too close to 1", i, ki);
        }

        let lsf = lpc_to_lsf(&a_orig);
        assert_eq!(lsf.len(), a_orig.len(), "LSF count should match LPC order");

        // All LSFs should be in (0, pi)
        for (i, &w) in lsf.iter().enumerate() {
            assert!(w > 0.0 && w < PI, "LSF[{}]={} out of range", i, w);
        }

        let a_recovered = lsf_to_lpc(&lsf);
        assert_eq!(a_recovered.len(), a_orig.len());

        // Check roundtrip accuracy
        for (i, (&orig, &rec)) in a_orig.iter().zip(a_recovered.iter()).enumerate() {
            assert!(
                (orig - rec).abs() < 0.01,
                "LSF roundtrip mismatch at [{}]: orig={}, recovered={}",
                i,
                orig,
                rec
            );
        }
    }

    #[test]
    fn test_lsf_ordered() {
        // LSFs should be strictly increasing in (0, pi)
        // Use a multi-component signal for well-conditioned LPC
        let fs = 8000.0;
        let signal: Vec<f64> = (0..320)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * PI * 400.0 * t).sin()
                    + 0.3 * (2.0 * PI * 1200.0 * t).sin()
                    + 0.2 * (2.0 * PI * 2800.0 * t).sin()
                    + 0.1 * ((i as f64 * 3.14).sin())
            })
            .collect();
        let r = autocorrelation(&signal, 6);
        let (a, _, _) = levinson_durbin(&r, 6);
        let lsf = lpc_to_lsf(&a);

        for i in 1..lsf.len() {
            assert!(
                lsf[i] > lsf[i - 1],
                "LSFs not strictly increasing: lsf[{}]={} <= lsf[{}]={}",
                i,
                lsf[i],
                i - 1,
                lsf[i - 1]
            );
        }
        for (i, &w) in lsf.iter().enumerate() {
            assert!(
                w > 0.0 && w < PI,
                "LSF[{}]={} out of (0, pi) range",
                i,
                w
            );
        }
    }

    #[test]
    fn test_lsf_empty() {
        assert!(lpc_to_lsf(&[]).is_empty());
        assert!(lsf_to_lpc(&[]).is_empty());
    }

    // -----------------------------------------------------------------------
    // PARCOR to LPC tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_parcor_to_lpc_first_order() {
        // k = [-0.5] -> a = [-0.5]
        let a = parcor_to_lpc(&[-0.5]);
        assert_eq!(a.len(), 1);
        assert!((a[0] - (-0.5)).abs() < EPSILON);
    }

    #[test]
    fn test_parcor_to_lpc_matches_levinson() {
        // Levinson-Durbin produces both a and k; verify parcor_to_lpc(k) == a
        let signal = sine_wave(440.0, 8000.0, 320);
        let r = autocorrelation(&signal, 8);
        let (a_ld, _, k_ld) = levinson_durbin(&r, 8);

        let a_from_k = parcor_to_lpc(&k_ld);
        assert_eq!(a_from_k.len(), a_ld.len());
        for (i, (&orig, &conv)) in a_ld.iter().zip(a_from_k.iter()).enumerate() {
            assert!(
                (orig - conv).abs() < 1e-10,
                "PARCOR->LPC mismatch at [{}]: {} vs {}",
                i,
                orig,
                conv
            );
        }
    }

    #[test]
    fn test_parcor_to_lpc_empty() {
        assert!(parcor_to_lpc(&[]).is_empty());
    }

    // -----------------------------------------------------------------------
    // Analyzer / Synthesizer integration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_analyzer_sine() {
        let config = LpcConfig::default();
        let analyzer = LpcAnalyzer::new(config.clone());
        let frame = sine_wave(200.0, config.sample_rate_hz, config.frame_size_samples());

        let lpc = analyzer.analyze_frame(&frame);
        assert_eq!(lpc.coefficients.len(), config.order);
        assert!(lpc.gain > 0.0, "Gain should be positive");
        assert!(lpc.voiced, "Pure sine should be voiced");
        assert!(lpc.pitch_period_samples.is_some());

        // Pitch period should be ~40 samples for 200 Hz at 8000 Hz
        let period = lpc.pitch_period_samples.unwrap();
        assert!(
            (period as f64 - 40.0).abs() < 3.0,
            "Expected pitch period ~40, got {}",
            period
        );
    }

    #[test]
    fn test_analyzer_empty_frame() {
        let config = LpcConfig::default();
        let analyzer = LpcAnalyzer::new(config.clone());
        let lpc = analyzer.analyze_frame(&[]);
        assert_eq!(lpc.coefficients.len(), config.order);
        assert!(!lpc.voiced);
        assert!((lpc.gain - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_synthesizer_voiced_produces_periodic() {
        let config = LpcConfig::default();
        let mut synth = LpcSynthesizer::new(config.clone());

        let frame = LpcFrame {
            coefficients: vec![0.0; config.order],
            gain: 1.0,
            pitch_period_samples: Some(40),
            voiced: true,
            reflection_coefficients: vec![0.0; config.order],
        };

        let output = synth.synthesize_frame(&frame);
        assert_eq!(output.len(), config.frame_size_samples());

        // With trivial (zero) LPC coefficients, the output should be a pulse train
        // Check that pulses appear at the expected interval
        let mut pulse_indices = Vec::new();
        for (i, &s) in output.iter().enumerate() {
            if s.abs() > 0.5 {
                pulse_indices.push(i);
            }
        }
        assert!(
            pulse_indices.len() >= 2,
            "Should have multiple pulses, got {}",
            pulse_indices.len()
        );
        // Check spacing between pulses
        for pair in pulse_indices.windows(2) {
            let spacing = pair[1] - pair[0];
            assert_eq!(spacing, 40, "Pulse spacing should be 40, got {}", spacing);
        }
    }

    #[test]
    fn test_synthesizer_unvoiced_produces_noise() {
        let config = LpcConfig::default();
        let mut synth = LpcSynthesizer::new(config.clone());

        let frame = LpcFrame {
            coefficients: vec![0.0; config.order],
            gain: 1.0,
            pitch_period_samples: None,
            voiced: false,
            reflection_coefficients: vec![0.0; config.order],
        };

        let output = synth.synthesize_frame(&frame);
        assert_eq!(output.len(), config.frame_size_samples());

        // Check that output has variation (not all zeros or constant)
        let mean: f64 = output.iter().sum::<f64>() / output.len() as f64;
        let variance: f64 =
            output.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / output.len() as f64;
        assert!(variance > 0.01, "Noise output should have variance, got {}", variance);
    }

    #[test]
    fn test_synthesizer_reset() {
        let config = LpcConfig::default();
        let mut synth = LpcSynthesizer::new(config.clone());

        let frame = LpcFrame {
            coefficients: vec![0.0; config.order],
            gain: 1.0,
            pitch_period_samples: Some(40),
            voiced: true,
            reflection_coefficients: vec![0.0; config.order],
        };

        let out1 = synth.synthesize_frame(&frame);
        synth.reset();
        let out2 = synth.synthesize_frame(&frame);

        // After reset, the output should match the first frame's output
        for (a, b) in out1.iter().zip(out2.iter()) {
            assert!(
                (a - b).abs() < 1e-10,
                "Reset did not restore initial state"
            );
        }
    }

    #[test]
    fn test_codec_roundtrip() {
        // Full encode-decode pipeline: analyze then synthesize
        let config = LpcConfig::default();
        let analyzer = LpcAnalyzer::new(config.clone());
        let mut synthesizer = LpcSynthesizer::new(config.clone());

        // Create a synthetic vowel-like signal (sum of harmonics)
        let fs = config.sample_rate_hz;
        let n = config.frame_size_samples();
        let f0 = 150.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                1.0 * (2.0 * PI * f0 * t).sin()
                    + 0.7 * (2.0 * PI * 2.0 * f0 * t).sin()
                    + 0.3 * (2.0 * PI * 3.0 * f0 * t).sin()
                    + 0.1 * (2.0 * PI * 4.0 * f0 * t).sin()
            })
            .collect();

        let lpc_frame = analyzer.analyze_frame(&signal);
        let reconstructed = synthesizer.synthesize_frame(&lpc_frame);

        assert_eq!(reconstructed.len(), n);

        // The reconstructed signal should have non-trivial energy
        let recon_energy = energy(&reconstructed);
        assert!(
            recon_energy > 0.01,
            "Reconstructed signal too quiet: energy={}",
            recon_energy
        );
    }

    // -----------------------------------------------------------------------
    // Hamming window test
    // -----------------------------------------------------------------------

    #[test]
    fn test_hamming_window_endpoints() {
        let mut frame = vec![1.0; 100];
        hamming_window(&mut frame);
        // Hamming window: w[0] = 0.54 - 0.46 = 0.08
        assert!((frame[0] - 0.08).abs() < 1e-6);
        // w[N-1] = 0.08 as well
        assert!((frame[99] - 0.08).abs() < 1e-6);
        // w[N/2] ~ 1.0
        assert!((frame[50] - 1.0).abs() < 0.02);
    }

    #[test]
    fn test_hamming_window_single_sample() {
        // Single sample: w[0] with n=1 => cos(0) = 1 => 0.54 - 0.46 = 0.08
        // But N-1 = 0, division by zero edge case
        let mut frame = vec![2.0];
        hamming_window(&mut frame);
        // With N=1, we skip (N<=1 guard), so the value stays unchanged
        assert!((frame[0] - 2.0).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Edge case tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lpc_config_frame_size() {
        let config = LpcConfig {
            sample_rate_hz: 16000.0,
            order: 12,
            frame_size_ms: 25.0,
            preemphasis: 0.95,
        };
        assert_eq!(config.frame_size_samples(), 400);
    }

    #[test]
    fn test_deemphasis_filter() {
        let signal = vec![1.0, 0.0, 0.0, 0.0];
        let alpha = 0.5;
        let result = deemphasis_filter(&signal, alpha);
        // y[0] = 1.0
        // y[1] = 0.0 + 0.5*1.0 = 0.5
        // y[2] = 0.0 + 0.5*0.5 = 0.25
        // y[3] = 0.0 + 0.5*0.25 = 0.125
        assert!((result[0] - 1.0).abs() < EPSILON);
        assert!((result[1] - 0.5).abs() < EPSILON);
        assert!((result[2] - 0.25).abs() < EPSILON);
        assert!((result[3] - 0.125).abs() < EPSILON);
    }
}
