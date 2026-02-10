//! Pre-emphasis and De-emphasis Filters
//!
//! Pre-emphasis and de-emphasis are complementary filters used in FM radio,
//! vinyl record equalization (RIAA), and audio processing to improve signal-to-noise
//! ratio by boosting high frequencies before transmission (pre-emphasis) and
//! attenuating them after reception (de-emphasis).
//!
//! Pre-emphasis applies a +6 dB/octave boost above the cutoff frequency
//! f_c = 1/(2*pi*tau), while de-emphasis applies the inverse -6 dB/octave
//! rolloff to restore the original spectrum.
//!
//! Common time constants:
//! - 75 us: North America / South Korea FM broadcast
//! - 50 us: Europe / Australia / Japan FM broadcast
//! - 318 us: RIAA record equalization (approximate single-pole model)
//!
//! # Example
//!
//! ```rust
//! use r4w_core::pre_emphasis::{PreEmphasis, DeEmphasis, fm_pre_emphasis, fm_de_emphasis};
//!
//! let sample_rate = 48000.0;
//!
//! // Create a matched pre/de-emphasis pair for North American FM
//! let mut pre = fm_pre_emphasis(sample_rate);
//! let mut de = fm_de_emphasis(sample_rate);
//!
//! let input = vec![0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0, -1.0];
//! let emphasized = pre.process(&input);
//! let recovered = de.process(&emphasized);
//!
//! // After settling, recovered approximates the original signal
//! ```

/// Pre-emphasis filter that boosts high-frequency content.
///
/// Implements a first-order high-pass IIR filter:
///   y[n] = (x[n] - alpha * x[n-1]) / (1 - alpha)
///
/// where alpha = exp(-1 / (tau * sample_rate)).
/// The normalization by (1 - alpha) ensures unity gain at DC and makes this
/// the exact inverse of [`DeEmphasis`].
#[derive(Debug, Clone)]
pub struct PreEmphasis {
    /// Time constant in seconds (e.g., 75e-6 for NA FM)
    pub tau: f64,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Filter coefficient: exp(-1 / (tau * sample_rate))
    pub alpha: f64,
    /// Previous output (used to hold previous input sample internally)
    pub prev_output: f64,
    /// Previous input sample
    prev_input: f64,
}

impl PreEmphasis {
    /// Create a new pre-emphasis filter.
    ///
    /// # Arguments
    /// * `tau` - Time constant in seconds (e.g., 75e-6 for North American FM)
    /// * `sample_rate` - Sample rate in Hz
    pub fn new(tau: f64, sample_rate: f64) -> Self {
        let alpha = (-1.0 / (tau * sample_rate)).exp();
        Self {
            tau,
            sample_rate,
            alpha,
            prev_output: 0.0,
            prev_input: 0.0,
        }
    }

    /// Process a block of input samples, returning pre-emphasized output.
    ///
    /// Applies the difference equation: y[n] = (x[n] - alpha * x[n-1]) / (1 - alpha)
    ///
    /// The (1 - alpha) normalization ensures this is the exact inverse of
    /// [`DeEmphasis`], so that cascading pre-emphasis then de-emphasis recovers
    /// the original signal.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        let inv_gain = 1.0 / (1.0 - self.alpha);
        for &x in input {
            let y = (x - self.alpha * self.prev_input) * inv_gain;
            self.prev_input = x;
            self.prev_output = y;
            output.push(y);
        }
        output
    }

    /// Reset filter state to zero.
    pub fn reset(&mut self) {
        self.prev_output = 0.0;
        self.prev_input = 0.0;
    }
}

/// De-emphasis filter that attenuates high-frequency content.
///
/// Implements a first-order low-pass IIR filter:
///   y[n] = (1 - alpha) * x[n] + alpha * y[n-1]
///
/// where alpha = exp(-1 / (tau * sample_rate)).
///
/// This is the inverse of [`PreEmphasis`]; applying pre-emphasis followed
/// by de-emphasis with the same tau recovers the original signal.
#[derive(Debug, Clone)]
pub struct DeEmphasis {
    /// Time constant in seconds
    pub tau: f64,
    /// Sample rate in Hz
    pub sample_rate: f64,
    /// Filter coefficient: exp(-1 / (tau * sample_rate))
    pub alpha: f64,
    /// Previous output sample
    pub prev_output: f64,
}

impl DeEmphasis {
    /// Create a new de-emphasis filter.
    ///
    /// # Arguments
    /// * `tau` - Time constant in seconds (e.g., 75e-6 for North American FM)
    /// * `sample_rate` - Sample rate in Hz
    pub fn new(tau: f64, sample_rate: f64) -> Self {
        let alpha = (-1.0 / (tau * sample_rate)).exp();
        Self {
            tau,
            sample_rate,
            alpha,
            prev_output: 0.0,
        }
    }

    /// Process a block of input samples, returning de-emphasized output.
    ///
    /// Applies the difference equation: y[n] = (1 - alpha) * x[n] + alpha * y[n-1]
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let y = (1.0 - self.alpha) * x + self.alpha * self.prev_output;
            self.prev_output = y;
            output.push(y);
        }
        output
    }

    /// Reset filter state to zero.
    pub fn reset(&mut self) {
        self.prev_output = 0.0;
    }
}

/// Convenience wrapper holding a matched pre-emphasis / de-emphasis pair.
///
/// Useful when both directions are needed, e.g., in a loopback test or
/// a transceiver that handles both TX and RX.
#[derive(Debug, Clone)]
pub struct PreDeEmphasisPair {
    pre: PreEmphasis,
    de: DeEmphasis,
}

impl PreDeEmphasisPair {
    /// Create a matched pair with the same time constant and sample rate.
    pub fn new(tau: f64, sample_rate: f64) -> Self {
        Self {
            pre: PreEmphasis::new(tau, sample_rate),
            de: DeEmphasis::new(tau, sample_rate),
        }
    }

    /// Apply pre-emphasis (boost high frequencies).
    pub fn pre_emphasize(&mut self, input: &[f64]) -> Vec<f64> {
        self.pre.process(input)
    }

    /// Apply de-emphasis (attenuate high frequencies).
    pub fn de_emphasize(&mut self, input: &[f64]) -> Vec<f64> {
        self.de.process(input)
    }
}

/// Factory: create a pre-emphasis filter for North American FM broadcast (75 us).
pub fn fm_pre_emphasis(sample_rate: f64) -> PreEmphasis {
    PreEmphasis::new(75e-6, sample_rate)
}

/// Factory: create a de-emphasis filter for North American FM broadcast (75 us).
pub fn fm_de_emphasis(sample_rate: f64) -> DeEmphasis {
    DeEmphasis::new(75e-6, sample_rate)
}

/// Factory: create a pre-emphasis filter approximating the RIAA record
/// equalization curve using a single-pole model with tau ~ 318 us.
///
/// The full RIAA EQ has three time constants (3180 us, 318 us, 75 us),
/// but the 318 us pole dominates the turnover region and serves as a
/// useful single-pole approximation.
pub fn riaa_pre_emphasis(sample_rate: f64) -> PreEmphasis {
    PreEmphasis::new(318e-6, sample_rate)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a sine tone.
    fn sine_tone(freq: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: compute RMS power of a slice (skipping the first `skip` samples).
    fn rms_power(signal: &[f64], skip: usize) -> f64 {
        let slice = &signal[skip..];
        (slice.iter().map(|x| x * x).sum::<f64>() / slice.len() as f64).sqrt()
    }

    #[test]
    fn test_pre_emphasis_boost() {
        let fs = 48000.0;
        let mut pre = PreEmphasis::new(75e-6, fs);

        // A high-frequency tone should be boosted relative to its input amplitude.
        let tone = sine_tone(10000.0, fs, 4800);
        let out = pre.process(&tone);

        let in_rms = rms_power(&tone, 500);
        let out_rms = rms_power(&out, 500);

        assert!(
            out_rms > in_rms,
            "10 kHz should be boosted: in_rms={:.4}, out_rms={:.4}",
            in_rms,
            out_rms
        );
    }

    #[test]
    fn test_de_emphasis_cut() {
        let fs = 48000.0;
        let mut de = DeEmphasis::new(75e-6, fs);

        let tone = sine_tone(10000.0, fs, 4800);
        let out = de.process(&tone);

        let in_rms = rms_power(&tone, 500);
        let out_rms = rms_power(&out, 500);

        assert!(
            out_rms < in_rms,
            "10 kHz should be attenuated: in_rms={:.4}, out_rms={:.4}",
            in_rms,
            out_rms
        );
    }

    #[test]
    fn test_roundtrip() {
        let fs = 48000.0;
        let mut pre = PreEmphasis::new(75e-6, fs);
        let mut de = DeEmphasis::new(75e-6, fs);

        // Mixed-frequency signal
        let n = 9600;
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * PI * 300.0 * t).sin() + 0.5 * (2.0 * PI * 5000.0 * t).sin()
            })
            .collect();

        let emphasized = pre.process(&input);
        let recovered = de.process(&emphasized);

        // After settling, mean squared error should be small.
        let skip = 2000;
        let mse: f64 = input[skip..]
            .iter()
            .zip(recovered[skip..].iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            / (n - skip) as f64;

        assert!(
            mse < 0.01,
            "Roundtrip MSE should be < 0.01: got {:.6}",
            mse
        );
    }

    #[test]
    fn test_dc_passthrough() {
        // A constant (DC) input should pass through de-emphasis unchanged
        // once the filter settles, since de-emphasis is a lowpass with unity
        // DC gain: H(z=1) = (1-alpha) / (1-alpha) = 1.
        let fs = 48000.0;
        let mut de = DeEmphasis::new(75e-6, fs);

        let dc = vec![1.0; 5000];
        let out = de.process(&dc);

        // After settling, output should converge to 1.0
        let tail_avg: f64 = out[4000..].iter().sum::<f64>() / 1000.0;
        assert!(
            (tail_avg - 1.0).abs() < 1e-3,
            "DC should pass through: tail_avg={:.6}",
            tail_avg
        );
    }

    #[test]
    fn test_fm_75us() {
        let fs = 44100.0;
        let pre = fm_pre_emphasis(fs);
        assert!((pre.tau - 75e-6).abs() < 1e-12);
        assert!((pre.sample_rate - fs).abs() < 1e-6);

        let expected_alpha = (-1.0 / (75e-6 * fs)).exp();
        assert!(
            (pre.alpha - expected_alpha).abs() < 1e-12,
            "alpha mismatch: got {}, expected {}",
            pre.alpha,
            expected_alpha
        );
    }

    #[test]
    fn test_fm_50us() {
        let fs = 48000.0;
        let pre_50 = PreEmphasis::new(50e-6, fs);
        let pre_75 = PreEmphasis::new(75e-6, fs);

        // 50us has a higher cutoff frequency, so its alpha should be smaller
        // (faster decay in impulse response).
        assert!(
            pre_50.alpha < pre_75.alpha,
            "50us alpha ({}) should be < 75us alpha ({})",
            pre_50.alpha,
            pre_75.alpha
        );

        // At a moderate frequency, 75us should boost more than 50us
        let tone = sine_tone(5000.0, fs, 4800);

        let mut p50 = pre_50;
        let mut p75 = pre_75;
        let out_50 = p50.process(&tone);
        let out_75 = p75.process(&tone);

        let rms_50 = rms_power(&out_50, 500);
        let rms_75 = rms_power(&out_75, 500);

        assert!(
            rms_75 > rms_50,
            "75us should boost 5 kHz more than 50us: rms_75={:.4}, rms_50={:.4}",
            rms_75,
            rms_50
        );
    }

    #[test]
    fn test_pair() {
        let fs = 48000.0;
        let mut pair = PreDeEmphasisPair::new(75e-6, fs);

        let input = sine_tone(3000.0, fs, 4800);
        let emphasized = pair.pre_emphasize(&input);
        let recovered = pair.de_emphasize(&emphasized);

        let skip = 1000;
        let mse: f64 = input[skip..]
            .iter()
            .zip(recovered[skip..].iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            / (input.len() - skip) as f64;

        assert!(
            mse < 0.01,
            "Pair roundtrip MSE should be < 0.01: got {:.6}",
            mse
        );
    }

    #[test]
    fn test_reset() {
        let fs = 48000.0;
        let mut pre = PreEmphasis::new(75e-6, fs);
        let mut de = DeEmphasis::new(75e-6, fs);

        // Process something to populate internal state
        pre.process(&[1.0, 2.0, 3.0]);
        de.process(&[1.0, 2.0, 3.0]);

        assert!(pre.prev_output.abs() > 0.0 || pre.prev_input.abs() > 0.0);
        assert!(de.prev_output.abs() > 0.0);

        pre.reset();
        de.reset();

        assert!(
            pre.prev_output.abs() < 1e-15,
            "pre.prev_output should be 0 after reset"
        );
        assert!(
            pre.prev_input.abs() < 1e-15,
            "pre.prev_input should be 0 after reset"
        );
        assert!(
            de.prev_output.abs() < 1e-15,
            "de.prev_output should be 0 after reset"
        );
    }

    #[test]
    fn test_high_freq_boost() {
        let fs = 48000.0;
        let mut pre = PreEmphasis::new(75e-6, fs);

        // Compare boost at two different frequencies: 1 kHz vs 12 kHz.
        // Higher frequency should get more boost.
        let tone_lo = sine_tone(1000.0, fs, 9600);
        let out_lo = pre.process(&tone_lo);
        let rms_out_lo = rms_power(&out_lo, 1000);
        let rms_in_lo = rms_power(&tone_lo, 1000);
        let gain_lo = rms_out_lo / rms_in_lo;

        pre.reset();

        let tone_hi = sine_tone(12000.0, fs, 9600);
        let out_hi = pre.process(&tone_hi);
        let rms_out_hi = rms_power(&out_hi, 1000);
        let rms_in_hi = rms_power(&tone_hi, 1000);
        let gain_hi = rms_out_hi / rms_in_hi;

        assert!(
            gain_hi > gain_lo,
            "12 kHz gain ({:.4}) should exceed 1 kHz gain ({:.4})",
            gain_hi,
            gain_lo
        );
    }

    #[test]
    fn test_empty_input() {
        let fs = 48000.0;
        let mut pre = PreEmphasis::new(75e-6, fs);
        let mut de = DeEmphasis::new(75e-6, fs);

        let empty: &[f64] = &[];
        let out_pre = pre.process(empty);
        let out_de = de.process(empty);

        assert!(out_pre.is_empty(), "Pre-emphasis of empty input should be empty");
        assert!(out_de.is_empty(), "De-emphasis of empty input should be empty");
    }
}
