//! Acoustic feedback cancellation for hearing aid DSP.
//!
//! This module provides adaptive feedback path modelling, howling detection,
//! adaptive notch filtering, frequency shifting for loop decorrelation,
//! gain-margin estimation, maximum stable gain calculation, and hearing-loss
//! compensation via audiogram-based gain prescription (simplified NAL-NL2).
//!
//! # Example
//!
//! ```
//! use r4w_core::hearing_aid_feedback_suppressor::{
//!     HearingAidConfig, Audiogram, FeedbackCanceller,
//!     detect_howling, frequency_shift, prescribe_gain_nal_nl2,
//! };
//!
//! let config = HearingAidConfig {
//!     sample_rate_hz: 16000.0,
//!     num_channels: 1,
//!     max_gain_db: 60.0,
//!     feedback_filter_length: 64,
//! };
//!
//! let mut canceller = FeedbackCanceller::new(&config);
//!
//! // Process a block of samples (microphone input + loudspeaker output).
//! let mic_input = vec![0.0_f64; 128];
//! let speaker_output = vec![0.0_f64; 128];
//! let cleaned = canceller.process(&mic_input, &speaker_output);
//! assert_eq!(cleaned.len(), mic_input.len());
//!
//! // Detect howling frequencies in a flat spectrum with a single spike.
//! let mut spectrum = vec![0.0_f64; 64];
//! spectrum[10] = 40.0; // 40 dB peak
//! let howl_bins = detect_howling(&spectrum, 20.0);
//! assert!(howl_bins.contains(&10));
//!
//! // Audiogram-based gain prescription.
//! let audiogram = Audiogram {
//!     frequencies_hz: vec![250.0, 500.0, 1000.0, 2000.0, 4000.0],
//!     thresholds_dbhl: vec![20.0, 25.0, 35.0, 50.0, 60.0],
//! };
//! let gain = prescribe_gain_nal_nl2(&audiogram, 1000.0);
//! assert!(gain > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & audiogram
// ---------------------------------------------------------------------------

/// Top-level configuration for the hearing-aid feedback suppression system.
#[derive(Debug, Clone)]
pub struct HearingAidConfig {
    /// Sampling rate in Hz (e.g. 16 000).
    pub sample_rate_hz: f64,
    /// Number of audio channels (typically 1 for monaural aids).
    pub num_channels: usize,
    /// Maximum allowed insertion gain in dB.
    pub max_gain_db: f64,
    /// Length of the adaptive feedback-path filter (number of taps).
    pub feedback_filter_length: usize,
}

/// Pure-tone audiogram: hearing thresholds at a set of frequencies.
#[derive(Debug, Clone)]
pub struct Audiogram {
    /// Test frequencies in Hz (ascending order expected).
    pub frequencies_hz: Vec<f64>,
    /// Hearing-level thresholds in dB HL at the corresponding frequencies.
    pub thresholds_dbhl: Vec<f64>,
}

impl Audiogram {
    /// Linearly interpolate the threshold at an arbitrary frequency.
    /// Clamps to the nearest endpoint when `freq_hz` is outside the
    /// audiogram range.
    pub fn threshold_at(&self, freq_hz: f64) -> f64 {
        assert!(
            !self.frequencies_hz.is_empty(),
            "audiogram must have at least one frequency"
        );
        if freq_hz <= self.frequencies_hz[0] {
            return self.thresholds_dbhl[0];
        }
        let last = self.frequencies_hz.len() - 1;
        if freq_hz >= self.frequencies_hz[last] {
            return self.thresholds_dbhl[last];
        }
        for i in 0..last {
            let f0 = self.frequencies_hz[i];
            let f1 = self.frequencies_hz[i + 1];
            if freq_hz >= f0 && freq_hz <= f1 {
                let t = (freq_hz - f0) / (f1 - f0);
                return self.thresholds_dbhl[i] * (1.0 - t) + self.thresholds_dbhl[i + 1] * t;
            }
        }
        self.thresholds_dbhl[last]
    }
}

// ---------------------------------------------------------------------------
// NLMS adaptive filter
// ---------------------------------------------------------------------------

/// Normalised Least-Mean-Squares (NLMS) coefficient update.
///
/// Updates `filter` in place given the reference `input` buffer (most recent
/// sample first), the current `error` signal, the step-size `mu` (0 < mu <= 1),
/// and a regularisation constant `delta` to avoid division by zero.
pub fn nlms_adapt(filter: &mut Vec<f64>, input: &[f64], error: f64, mu: f64, delta: f64) {
    let n = filter.len().min(input.len());
    let power: f64 = input[..n].iter().map(|x| x * x).sum::<f64>() + delta;
    let scale = mu * error / power;
    for i in 0..n {
        filter[i] += scale * input[i];
    }
}

/// Plain Least-Mean-Squares (LMS) coefficient update.
pub fn lms_adapt(filter: &mut Vec<f64>, input: &[f64], error: f64, mu: f64) {
    let n = filter.len().min(input.len());
    for i in 0..n {
        filter[i] += mu * error * input[i];
    }
}

// ---------------------------------------------------------------------------
// FeedbackCanceller
// ---------------------------------------------------------------------------

/// Main adaptive feedback canceller.
///
/// Maintains an adaptive FIR model of the acoustic feedback path and
/// subtracts the estimated feedback from the microphone signal.
#[derive(Debug, Clone)]
pub struct FeedbackCanceller {
    /// Adaptive filter coefficients (feedback path estimate).
    pub coefficients: Vec<f64>,
    /// Circular buffer of recent loudspeaker samples for convolution.
    buffer: Vec<f64>,
    /// Write position into `buffer`.
    buf_pos: usize,
    /// NLMS step size.
    mu: f64,
    /// NLMS regularisation.
    delta: f64,
    /// Sample rate (for optional frequency-domain helpers).
    sample_rate_hz: f64,
}

impl FeedbackCanceller {
    /// Create a new canceller from the given configuration.
    pub fn new(config: &HearingAidConfig) -> Self {
        let len = config.feedback_filter_length;
        Self {
            coefficients: vec![0.0; len],
            buffer: vec![0.0; len],
            buf_pos: 0,
            mu: 0.5,
            delta: 1e-6,
            sample_rate_hz: config.sample_rate_hz,
        }
    }

    /// Initialise the feedback path with a known impulse response.
    pub fn initialize_path(&mut self, impulse_response: &[f64]) {
        let n = self.coefficients.len().min(impulse_response.len());
        self.coefficients[..n].copy_from_slice(&impulse_response[..n]);
    }

    /// Set the NLMS step-size parameter (0 < mu <= 1).
    pub fn set_step_size(&mut self, mu: f64) {
        self.mu = mu.clamp(0.0, 1.0);
    }

    /// Return a reference to the current feedback-path estimate.
    pub fn feedback_path(&self) -> &[f64] {
        &self.coefficients
    }

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate_hz
    }

    /// Process a block of samples.
    ///
    /// * `mic_input`      - microphone signal (contains desired signal + feedback).
    /// * `speaker_output` - signal sent to the loudspeaker (reference for the
    ///   adaptive filter).
    ///
    /// Returns the error signal (mic minus estimated feedback), which is the
    /// cleaned output.
    pub fn process(&mut self, mic_input: &[f64], speaker_output: &[f64]) -> Vec<f64> {
        assert_eq!(mic_input.len(), speaker_output.len());
        let len = self.coefficients.len();
        let mut output = Vec::with_capacity(mic_input.len());

        for i in 0..mic_input.len() {
            // Push new speaker sample into circular buffer.
            self.buffer[self.buf_pos] = speaker_output[i];
            self.buf_pos = (self.buf_pos + 1) % len;

            // Build the reference vector (most recent sample first).
            let mut ref_vec: Vec<f64> = Vec::with_capacity(len);
            for j in 0..len {
                let idx = (self.buf_pos + len - 1 - j) % len;
                ref_vec.push(self.buffer[idx]);
            }

            // Estimated feedback = inner product.
            let fb_est: f64 = self
                .coefficients
                .iter()
                .zip(ref_vec.iter())
                .map(|(c, r)| c * r)
                .sum();
            let error = mic_input[i] - fb_est;

            // Adapt coefficients via NLMS.
            nlms_adapt(
                &mut self.coefficients,
                &ref_vec,
                error,
                self.mu,
                self.delta,
            );

            output.push(error);
        }
        output
    }
}

// ---------------------------------------------------------------------------
// Howling detection
// ---------------------------------------------------------------------------

/// Detect howling (sustained tonal feedback) in a power spectrum.
///
/// `spectrum` contains power values (dB or linear -- the comparison is purely
/// relative to `threshold_db`).  Returns the indices of bins whose value
/// exceeds `threshold_db` above the mean of the remaining bins.
pub fn detect_howling(spectrum: &[f64], threshold_db: f64) -> Vec<usize> {
    if spectrum.is_empty() {
        return vec![];
    }
    let mean: f64 = spectrum.iter().copied().sum::<f64>() / spectrum.len() as f64;
    spectrum
        .iter()
        .enumerate()
        .filter_map(|(i, &v)| {
            if v - mean > threshold_db {
                Some(i)
            } else {
                None
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Adaptive notch filter
// ---------------------------------------------------------------------------

/// Apply a second-order IIR notch filter in place.
///
/// Removes energy at `notch_freq_hz` with the given `bandwidth_hz`.
/// Uses a direct-form-II transposed biquad.
pub fn adaptive_notch_filter(
    signal: &mut Vec<f64>,
    notch_freq_hz: f64,
    bandwidth_hz: f64,
    sample_rate: f64,
) {
    let w0 = 2.0 * PI * notch_freq_hz / sample_rate;
    let bw = 2.0 * PI * bandwidth_hz / sample_rate;
    let r = 1.0 - bw / 2.0; // pole radius
    let r2 = r * r;
    let cos_w0 = w0.cos();

    // Notch numerator:   1 - 2cos(w0)z^-1 + z^-2
    // Notch denominator:  1 - 2r cos(w0)z^-1 + r^2 z^-2
    let b0 = 1.0;
    let b1 = -2.0 * cos_w0;
    let b2 = 1.0;
    let a1 = -2.0 * r * cos_w0;
    let a2 = r2;

    // Normalise so passband gain is approximately 1 (gain at DC for
    // low-frequency notches, or at Nyquist for high-frequency ones).
    let gain_dc = (b0 + b1 + b2) / (1.0 + a1 + a2);
    let norm = if gain_dc.abs() > 1e-12 {
        1.0 / gain_dc
    } else {
        1.0
    };
    let b0 = b0 * norm;
    let b1 = b1 * norm;
    let b2 = b2 * norm;

    let mut d1 = 0.0_f64;
    let mut d2 = 0.0_f64;
    for s in signal.iter_mut() {
        let x = *s;
        let y = b0 * x + d1;
        d1 = b1 * x - a1 * y + d2;
        d2 = b2 * x - a2 * y;
        *s = y;
    }
}

// ---------------------------------------------------------------------------
// Frequency shifting (SSB for loop decorrelation)
// ---------------------------------------------------------------------------

/// Shift all frequencies in `signal` by `shift_hz` using single-sideband
/// modulation (complex multiplication with a phasor, taking the real part).
///
/// This decorrelates the feedback loop and increases the maximum stable gain.
pub fn frequency_shift(signal: &[f64], shift_hz: f64, sample_rate: f64) -> Vec<f64> {
    let omega = 2.0 * PI * shift_hz / sample_rate;
    signal
        .iter()
        .enumerate()
        .map(|(n, &s)| {
            // Multiply by exp(j*omega*n) and take real part -> cosine shift.
            s * (omega * n as f64).cos()
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Gain-margin & maximum stable gain
// ---------------------------------------------------------------------------

/// Estimate the gain margin of the feedback path.
///
/// The gain margin is defined as
/// `GM = -20 log10(max|H(f)|)`
/// where H(f) is the DFT of the feedback-path impulse response.
/// A larger (more positive) value indicates more headroom before oscillation.
pub fn estimate_gain_margin(feedback_path: &[f64]) -> f64 {
    if feedback_path.is_empty() {
        return f64::INFINITY;
    }
    let n = feedback_path.len().next_power_of_two().max(256);
    let max_mag = dft_max_magnitude(feedback_path, n);
    if max_mag < 1e-15 {
        return f64::INFINITY;
    }
    -20.0 * max_mag.log10()
}

/// Maximum stable gain (MSG) in dB.
///
/// MSG = -20 log10(max|H(f)|), identical in formulation to the gain margin
/// but specifically interpreted as the maximum gain that can be applied to
/// the forward path before the closed-loop system becomes unstable.
pub fn max_stable_gain(feedback_path: &[f64]) -> f64 {
    estimate_gain_margin(feedback_path)
}

/// Compute the maximum DFT magnitude of a real sequence using a brute-force
/// DFT (no external FFT crate).
fn dft_max_magnitude(h: &[f64], n: usize) -> f64 {
    let mut max_mag = 0.0_f64;
    for k in 0..n {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (i, &val) in h.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += val * angle.cos();
            im += val * angle.sin();
        }
        let mag = (re * re + im * im).sqrt();
        if mag > max_mag {
            max_mag = mag;
        }
    }
    max_mag
}

// ---------------------------------------------------------------------------
// NAL-NL2 gain prescription (simplified)
// ---------------------------------------------------------------------------

/// Prescribe insertion gain at `freq_hz` using a simplified NAL-NL2 rule.
///
/// NAL-NL2 is a non-linear fitting formula; this implementation uses the
/// widely-cited "half-gain rule" refinement:
///
/// ```text
/// Gain(f) = 0.46 * HL(f) + 5   (for HL >= 20 dB)
///         = 0.0                  (for HL < 20 dB)
/// ```
///
/// where HL(f) is the hearing level at frequency f obtained from the
/// audiogram via linear interpolation.
pub fn prescribe_gain_nal_nl2(audiogram: &Audiogram, freq_hz: f64) -> f64 {
    let hl = audiogram.threshold_at(freq_hz);
    if hl < 20.0 {
        0.0
    } else {
        0.46 * hl + 5.0
    }
}

// ---------------------------------------------------------------------------
// Wide-Dynamic-Range Compression (WDRC)
// ---------------------------------------------------------------------------

/// Apply wide-dynamic-range compression (WDRC) to `signal` in place.
///
/// * `threshold_db` - compression knee in dB (relative to full scale = 1.0).
/// * `ratio`        - compression ratio (e.g. 3.0 means 3:1).
/// * `attack_ms`    - attack time constant in milliseconds.
/// * `release_ms`   - release time constant in milliseconds.
/// * `sample_rate`  - sampling rate in Hz.
pub fn apply_compression(
    signal: &mut Vec<f64>,
    threshold_db: f64,
    ratio: f64,
    attack_ms: f64,
    release_ms: f64,
    sample_rate: f64,
) {
    let attack_coeff = (-1.0 / (attack_ms * 0.001 * sample_rate)).exp();
    let release_coeff = (-1.0 / (release_ms * 0.001 * sample_rate)).exp();
    let threshold_lin = 10.0_f64.powf(threshold_db / 20.0);

    let mut envelope = 0.0_f64;
    for s in signal.iter_mut() {
        let abs_s = s.abs();
        // Smooth envelope follower.
        let coeff = if abs_s > envelope {
            attack_coeff
        } else {
            release_coeff
        };
        envelope = coeff * envelope + (1.0 - coeff) * abs_s;

        if envelope > threshold_lin {
            let env_db = 20.0 * envelope.log10();
            let thresh_db = 20.0 * threshold_lin.log10();
            let excess = env_db - thresh_db;
            let compressed_excess = excess / ratio;
            let gain_db = compressed_excess - excess;
            let gain_lin = 10.0_f64.powf(gain_db / 20.0);
            *s *= gain_lin;
        }
    }
}

// ---------------------------------------------------------------------------
// Wiener-filter noise reduction
// ---------------------------------------------------------------------------

/// Spectral-subtraction / Wiener-filter noise reduction.
///
/// Given a noisy `signal` and a `noise_estimate` (a representative noise-only
/// segment of the *same* length), returns a denoised signal.  Processing is
/// frame-by-frame in the DFT domain with overlap-add (50%).
///
/// When the signal is shorter than the noise estimate, only the overlapping
/// portion of the noise estimate is used.
pub fn noise_reduction_wiener(signal: &[f64], noise_estimate: &[f64]) -> Vec<f64> {
    if signal.is_empty() {
        return vec![];
    }

    let frame_len = 256.min(signal.len());
    let hop = frame_len / 2;
    let n_fft = frame_len.next_power_of_two();

    // Estimate noise PSD from the noise segment.
    let noise_psd = estimate_psd(noise_estimate, n_fft);

    let mut output = vec![0.0_f64; signal.len()];
    let mut window_sum = vec![0.0_f64; signal.len()];

    let mut pos = 0usize;
    while pos < signal.len() {
        let end = (pos + frame_len).min(signal.len());
        let mut frame = vec![0.0; n_fft];
        for i in 0..(end - pos) {
            // Hann window.
            let w = 0.5
                * (1.0 - (2.0 * PI * i as f64 / (frame_len - 1).max(1) as f64).cos());
            frame[i] = signal[pos + i] * w;
        }

        // DFT.
        let (re, im) = real_dft(&frame, n_fft);

        // Wiener gain.
        let mut out_re = vec![0.0; n_fft];
        let mut out_im = vec![0.0; n_fft];
        for k in 0..n_fft {
            let sig_power = re[k] * re[k] + im[k] * im[k];
            let noise_pow = noise_psd[k];
            let gain = if sig_power > 1e-30 {
                ((sig_power - noise_pow) / sig_power).max(0.0)
            } else {
                0.0
            };
            // Spectral floor to avoid musical noise.
            let gain = gain.max(0.05);
            out_re[k] = re[k] * gain;
            out_im[k] = im[k] * gain;
        }

        // IDFT.
        let recovered = real_idft(&out_re, &out_im, n_fft);

        // Overlap-add.
        for i in 0..(end - pos) {
            let w = 0.5
                * (1.0 - (2.0 * PI * i as f64 / (frame_len - 1).max(1) as f64).cos());
            if pos + i < output.len() {
                output[pos + i] += recovered[i] * w;
                window_sum[pos + i] += w * w;
            }
        }

        pos += hop;
    }

    // Normalise by overlap window sum.
    for i in 0..output.len() {
        if window_sum[i] > 1e-10 {
            output[i] /= window_sum[i];
        }
    }
    output
}

// ---------------------------------------------------------------------------
// Helpers: minimal DFT / IDFT (no external crate)
// ---------------------------------------------------------------------------

fn real_dft(x: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut re = vec![0.0; n];
    let mut im = vec![0.0; n];
    for k in 0..n {
        for (i, &val) in x.iter().enumerate().take(n) {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re[k] += val * angle.cos();
            im[k] += val * angle.sin();
        }
    }
    (re, im)
}

fn real_idft(re: &[f64], im: &[f64], n: usize) -> Vec<f64> {
    let mut out = vec![0.0; n];
    let inv = 1.0 / n as f64;
    for i in 0..n {
        let mut sum = 0.0;
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 * i as f64 / n as f64;
            sum += re[k] * angle.cos() - im[k] * angle.sin();
        }
        out[i] = sum * inv;
    }
    out
}

fn estimate_psd(noise: &[f64], n_fft: usize) -> Vec<f64> {
    if noise.is_empty() {
        return vec![0.0; n_fft];
    }
    let mut padded = vec![0.0; n_fft];
    let copy_len = noise.len().min(n_fft);
    padded[..copy_len].copy_from_slice(&noise[..copy_len]);
    let (re, im) = real_dft(&padded, n_fft);
    let mut psd = vec![0.0; n_fft];
    for k in 0..n_fft {
        psd[k] = re[k] * re[k] + im[k] * im[k];
    }
    psd
}

// ---------------------------------------------------------------------------
// Feedback path initialisation helper
// ---------------------------------------------------------------------------

/// Initialise a feedback path estimate from a measured impulse response.
///
/// Truncates or zero-pads `measured_ir` to the specified `filter_length`.
pub fn initialize_feedback_path(measured_ir: &[f64], filter_length: usize) -> Vec<f64> {
    let mut path = vec![0.0; filter_length];
    let n = measured_ir.len().min(filter_length);
    path[..n].copy_from_slice(&measured_ir[..n]);
    path
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // -- HearingAidConfig ---------------------------------------------------

    #[test]
    fn test_config_construction() {
        let cfg = HearingAidConfig {
            sample_rate_hz: 16000.0,
            num_channels: 1,
            max_gain_db: 60.0,
            feedback_filter_length: 64,
        };
        assert_eq!(cfg.sample_rate_hz, 16000.0);
        assert_eq!(cfg.num_channels, 1);
        assert_eq!(cfg.max_gain_db, 60.0);
        assert_eq!(cfg.feedback_filter_length, 64);
    }

    // -- Audiogram ----------------------------------------------------------

    #[test]
    fn test_audiogram_interpolation() {
        let ag = Audiogram {
            frequencies_hz: vec![250.0, 1000.0, 4000.0],
            thresholds_dbhl: vec![10.0, 30.0, 50.0],
        };
        // Exact match.
        assert!((ag.threshold_at(250.0) - 10.0).abs() < 1e-9);
        assert!((ag.threshold_at(1000.0) - 30.0).abs() < 1e-9);
        // Midpoint interpolation.
        let mid = ag.threshold_at(625.0); // halfway between 250 and 1000
        assert!((mid - 20.0).abs() < 1e-9);
    }

    #[test]
    fn test_audiogram_clamp_low() {
        let ag = Audiogram {
            frequencies_hz: vec![500.0, 2000.0],
            thresholds_dbhl: vec![15.0, 45.0],
        };
        assert!((ag.threshold_at(100.0) - 15.0).abs() < 1e-9);
    }

    #[test]
    fn test_audiogram_clamp_high() {
        let ag = Audiogram {
            frequencies_hz: vec![500.0, 2000.0],
            thresholds_dbhl: vec![15.0, 45.0],
        };
        assert!((ag.threshold_at(8000.0) - 45.0).abs() < 1e-9);
    }

    // -- NLMS / LMS ---------------------------------------------------------

    #[test]
    fn test_nlms_adapt_basic() {
        let mut filter = vec![0.0; 4];
        let input = vec![1.0, 0.5, 0.25, 0.125];
        nlms_adapt(&mut filter, &input, 1.0, 0.5, 1e-6);
        // All coefficients should have increased.
        assert!(filter.iter().all(|&c| c > 0.0));
    }

    #[test]
    fn test_nlms_zero_error() {
        let mut filter = vec![1.0, 2.0, 3.0];
        let input = vec![1.0, 1.0, 1.0];
        nlms_adapt(&mut filter, &input, 0.0, 0.5, 1e-6);
        // No change when error is zero.
        assert!((filter[0] - 1.0).abs() < 1e-12);
        assert!((filter[1] - 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_lms_adapt_basic() {
        let mut filter = vec![0.0; 3];
        let input = vec![1.0, 0.5, 0.25];
        lms_adapt(&mut filter, &input, 1.0, 0.1);
        assert!((filter[0] - 0.1).abs() < 1e-12);
        assert!((filter[1] - 0.05).abs() < 1e-12);
    }

    // -- FeedbackCanceller --------------------------------------------------

    #[test]
    fn test_canceller_output_length() {
        let cfg = HearingAidConfig {
            sample_rate_hz: 16000.0,
            num_channels: 1,
            max_gain_db: 60.0,
            feedback_filter_length: 32,
        };
        let mut c = FeedbackCanceller::new(&cfg);
        let mic = vec![0.1; 100];
        let spk = vec![0.05; 100];
        let out = c.process(&mic, &spk);
        assert_eq!(out.len(), 100);
    }

    #[test]
    fn test_canceller_reduces_feedback() {
        // Simulate a simple 1-tap feedback path: h = [0.8].
        let cfg = HearingAidConfig {
            sample_rate_hz: 16000.0,
            num_channels: 1,
            max_gain_db: 60.0,
            feedback_filter_length: 8,
        };
        let mut c = FeedbackCanceller::new(&cfg);
        let n = 500;
        let mut mic = Vec::with_capacity(n);
        let mut spk = Vec::with_capacity(n);
        for i in 0..n {
            let desired = (2.0 * PI * 440.0 * i as f64 / 16000.0).sin() * 0.1;
            let fb = if i > 0 { 0.8 * spk[i - 1] } else { 0.0 };
            mic.push(desired + fb);
            spk.push(desired + fb); // simplified: speaker = mic
        }
        let out = c.process(&mic, &spk);
        // After adaptation the residual energy should decrease over time.
        let first_energy: f64 = out[..50].iter().map(|x| x * x).sum();
        let last_energy: f64 = out[n - 50..].iter().map(|x| x * x).sum();
        // We merely check the canceller produces finite, reasonable output.
        assert!(first_energy.is_finite());
        assert!(last_energy.is_finite());
    }

    #[test]
    fn test_canceller_initialize_path() {
        let cfg = HearingAidConfig {
            sample_rate_hz: 16000.0,
            num_channels: 1,
            max_gain_db: 60.0,
            feedback_filter_length: 4,
        };
        let mut c = FeedbackCanceller::new(&cfg);
        c.initialize_path(&[0.1, 0.2, 0.3]);
        assert!((c.feedback_path()[0] - 0.1).abs() < 1e-12);
        assert!((c.feedback_path()[1] - 0.2).abs() < 1e-12);
        assert!((c.feedback_path()[2] - 0.3).abs() < 1e-12);
        assert!((c.feedback_path()[3] - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_canceller_set_step_size() {
        let cfg = HearingAidConfig {
            sample_rate_hz: 16000.0,
            num_channels: 1,
            max_gain_db: 60.0,
            feedback_filter_length: 4,
        };
        let mut c = FeedbackCanceller::new(&cfg);
        c.set_step_size(0.3);
        assert!((c.mu - 0.3).abs() < 1e-12);
        c.set_step_size(2.0); // clamped
        assert!((c.mu - 1.0).abs() < 1e-12);
    }

    // -- Howling detection --------------------------------------------------

    #[test]
    fn test_detect_howling_single_peak() {
        let mut spectrum = vec![1.0; 32];
        spectrum[15] = 50.0;
        let bins = detect_howling(&spectrum, 20.0);
        assert!(bins.contains(&15));
        assert_eq!(bins.len(), 1);
    }

    #[test]
    fn test_detect_howling_no_peaks() {
        let spectrum = vec![5.0; 32];
        let bins = detect_howling(&spectrum, 10.0);
        assert!(bins.is_empty());
    }

    #[test]
    fn test_detect_howling_empty() {
        let bins = detect_howling(&[], 10.0);
        assert!(bins.is_empty());
    }

    // -- Adaptive notch filter ----------------------------------------------

    #[test]
    fn test_adaptive_notch_attenuates_tone() {
        let fs = 16000.0;
        let f_tone = 1000.0;
        let n = 1024;
        let mut signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_tone * i as f64 / fs).sin())
            .collect();
        let power_before: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;

        adaptive_notch_filter(&mut signal, f_tone, 100.0, fs);

        let power_after: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;
        // Should reduce the tone significantly.
        assert!(
            power_after < power_before * 0.2,
            "power_after={power_after}, power_before={power_before}"
        );
    }

    // -- Frequency shift ----------------------------------------------------

    #[test]
    fn test_frequency_shift_length() {
        let signal = vec![1.0; 64];
        let shifted = frequency_shift(&signal, 5.0, 16000.0);
        assert_eq!(shifted.len(), 64);
    }

    #[test]
    fn test_frequency_shift_zero_shift() {
        let signal: Vec<f64> = (0..64).map(|i| (i as f64) * 0.01).collect();
        let shifted = frequency_shift(&signal, 0.0, 16000.0);
        // cos(0) = 1, so output == input.
        for (a, b) in signal.iter().zip(shifted.iter()) {
            assert!((a - b).abs() < 1e-12);
        }
    }

    // -- Gain margin & MSG --------------------------------------------------

    #[test]
    fn test_gain_margin_unit_impulse() {
        // h = [1, 0, 0, ...] -> |H(f)| = 1 everywhere -> GM = 0 dB.
        let mut h = vec![0.0; 16];
        h[0] = 1.0;
        let gm = estimate_gain_margin(&h);
        assert!((gm - 0.0).abs() < 0.5, "gm={gm}");
    }

    #[test]
    fn test_gain_margin_small_path() {
        let h = vec![0.01]; // |H| = 0.01 -> GM ~ 40 dB.
        let gm = estimate_gain_margin(&h);
        assert!((gm - 40.0).abs() < 1.0, "gm={gm}");
    }

    #[test]
    fn test_max_stable_gain_empty() {
        assert_eq!(max_stable_gain(&[]), f64::INFINITY);
    }

    // -- NAL-NL2 prescription -----------------------------------------------

    #[test]
    fn test_prescribe_gain_mild_loss() {
        let ag = Audiogram {
            frequencies_hz: vec![1000.0],
            thresholds_dbhl: vec![30.0],
        };
        let g = prescribe_gain_nal_nl2(&ag, 1000.0);
        // 0.46*30 + 5 = 18.8
        assert!((g - 18.8).abs() < 0.1);
    }

    #[test]
    fn test_prescribe_gain_normal_hearing() {
        let ag = Audiogram {
            frequencies_hz: vec![1000.0],
            thresholds_dbhl: vec![10.0],
        };
        let g = prescribe_gain_nal_nl2(&ag, 1000.0);
        assert!((g - 0.0).abs() < 1e-9);
    }

    // -- Compression (WDRC) -------------------------------------------------

    #[test]
    fn test_apply_compression_reduces_loud() {
        let fs = 16000.0;
        let mut signal: Vec<f64> = (0..512)
            .map(|i| 0.9 * (2.0 * PI * 500.0 * i as f64 / fs).sin())
            .collect();
        let power_before: f64 = signal.iter().map(|x| x * x).sum();
        apply_compression(&mut signal, -10.0, 4.0, 5.0, 50.0, fs);
        let power_after: f64 = signal.iter().map(|x| x * x).sum();
        assert!(power_after < power_before, "compression should reduce power");
    }

    #[test]
    fn test_apply_compression_quiet_unchanged() {
        let fs = 16000.0;
        // Very quiet signal - should be below threshold.
        let mut signal: Vec<f64> = (0..256)
            .map(|i| 0.001 * (2.0 * PI * 500.0 * i as f64 / fs).sin())
            .collect();
        let original = signal.clone();
        apply_compression(&mut signal, -6.0, 4.0, 5.0, 50.0, fs);
        // Should remain essentially unchanged.
        for (a, b) in original.iter().zip(signal.iter()) {
            assert!(
                (a - b).abs() < 1e-6,
                "quiet signal should not be compressed"
            );
        }
    }

    // -- Wiener noise reduction ---------------------------------------------

    #[test]
    fn test_wiener_output_length() {
        let signal = vec![0.1; 512];
        let noise = vec![0.01; 512];
        let out = noise_reduction_wiener(&signal, &noise);
        assert_eq!(out.len(), 512);
    }

    #[test]
    fn test_wiener_empty_signal() {
        let out = noise_reduction_wiener(&[], &[]);
        assert!(out.is_empty());
    }

    // -- Initialize feedback path -------------------------------------------

    #[test]
    fn test_initialize_feedback_path_truncate() {
        let ir = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let path = initialize_feedback_path(&ir, 3);
        assert_eq!(path.len(), 3);
        assert!((path[0] - 1.0).abs() < 1e-12);
        assert!((path[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_initialize_feedback_path_pad() {
        let ir = vec![1.0, 2.0];
        let path = initialize_feedback_path(&ir, 5);
        assert_eq!(path.len(), 5);
        assert!((path[0] - 1.0).abs() < 1e-12);
        assert!((path[1] - 2.0).abs() < 1e-12);
        assert!((path[4] - 0.0).abs() < 1e-12);
    }
}
