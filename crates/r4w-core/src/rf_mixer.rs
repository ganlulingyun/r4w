//! RF Mixer Models for Frequency Conversion
//!
//! Implements RF mixer models for upconversion and downconversion in radio
//! front-end simulation. Supports complex and real-valued signals, continuous
//! phase accumulation across processing blocks, and realistic mixer impairments
//! (LO leakage, image rejection, IQ phase error).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rf_mixer::{RfMixer, nco_generate};
//!
//! // Create a mixer with 1 MHz LO at 10 MHz sample rate
//! let mut mixer = RfMixer::new(1_000_000.0, 10_000_000.0);
//!
//! // Generate a test tone at DC (baseband)
//! let baseband: Vec<(f64, f64)> = (0..100)
//!     .map(|_| (1.0, 0.0))
//!     .collect();
//!
//! // Upconvert to 1 MHz
//! let rf = mixer.mix_up(&baseband);
//! assert_eq!(rf.len(), 100);
//!
//! // Generate an NCO signal
//! let lo = nco_generate(1000.0, 48000.0, 48);
//! assert_eq!(lo.len(), 48);
//! ```

use std::f64::consts::PI;

/// Mixer impairments for realistic RF front-end simulation.
///
/// Models three common mixer non-idealities:
/// - **LO leakage**: Residual LO signal at the output (feedthrough)
/// - **Image rejection**: Incomplete suppression of the image frequency
/// - **IQ phase error**: Quadrature error between I and Q paths
#[derive(Debug, Clone, Copy)]
pub struct MixerImpairments {
    /// LO feedthrough level in dB below carrier (e.g., -30.0)
    pub lo_leakage_db: f64,
    /// Image suppression level in dB (e.g., -40.0)
    pub image_rejection_db: f64,
    /// Quadrature phase error in degrees (e.g., 1.0 for 1 degree)
    pub iq_phase_error_deg: f64,
}

impl Default for MixerImpairments {
    fn default() -> Self {
        Self {
            lo_leakage_db: -60.0,
            image_rejection_db: -60.0,
            iq_phase_error_deg: 0.0,
        }
    }
}

/// Complex RF mixer for frequency translation.
///
/// Multiplies an input signal by a complex LO signal to shift frequencies.
/// The phase accumulator maintains continuity across successive calls,
/// preventing discontinuities at block boundaries.
#[derive(Debug, Clone)]
pub struct RfMixer {
    /// LO frequency in Hz
    lo_frequency_hz: f64,
    /// Sample rate in Hz
    sample_rate: f64,
    /// Phase increment per sample (radians)
    phase_inc: f64,
    /// Phase accumulator (radians)
    phase: f64,
    /// Optional mixer impairments
    impairments: Option<MixerImpairments>,
}

impl RfMixer {
    /// Create a new RF mixer with the given LO frequency and sample rate.
    ///
    /// The phase accumulator starts at zero.
    pub fn new(lo_frequency_hz: f64, sample_rate: f64) -> Self {
        let phase_inc = 2.0 * PI * lo_frequency_hz / sample_rate;
        Self {
            lo_frequency_hz,
            sample_rate,
            phase_inc,
            phase: 0.0,
            impairments: None,
        }
    }

    /// Downconvert the input signal by multiplying with exp(-j * 2 * pi * f_lo * t).
    ///
    /// Each input sample (I, Q) is multiplied by the conjugate of the LO phasor,
    /// shifting the signal down in frequency by `lo_frequency_hz`.
    pub fn mix_down(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(i, q) in input {
            let cos_phase = self.phase.cos();
            let sin_phase = self.phase.sin();
            // Multiply by exp(-j * phase) = cos(phase) - j*sin(phase)
            let mut out_i = i * cos_phase + q * sin_phase;
            let mut out_q = -i * sin_phase + q * cos_phase;

            if let Some(ref imp) = self.impairments {
                let (ai, aq) = Self::apply_impairments(imp, out_i, out_q, cos_phase, sin_phase);
                out_i = ai;
                out_q = aq;
            }

            output.push((out_i, out_q));
            self.advance_phase();
        }
        output
    }

    /// Upconvert the input signal by multiplying with exp(+j * 2 * pi * f_lo * t).
    ///
    /// Each input sample (I, Q) is multiplied by the LO phasor,
    /// shifting the signal up in frequency by `lo_frequency_hz`.
    pub fn mix_up(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(i, q) in input {
            let cos_phase = self.phase.cos();
            let sin_phase = self.phase.sin();
            // Multiply by exp(+j * phase) = cos(phase) + j*sin(phase)
            let mut out_i = i * cos_phase - q * sin_phase;
            let mut out_q = i * sin_phase + q * cos_phase;

            if let Some(ref imp) = self.impairments {
                let (ai, aq) = Self::apply_impairments(imp, out_i, out_q, cos_phase, sin_phase);
                out_i = ai;
                out_q = aq;
            }

            output.push((out_i, out_q));
            self.advance_phase();
        }
        output
    }

    /// Set the LO frequency without resetting the phase accumulator.
    ///
    /// This allows glitch-free frequency changes during operation.
    pub fn set_lo_frequency(&mut self, freq_hz: f64) {
        self.lo_frequency_hz = freq_hz;
        self.phase_inc = 2.0 * PI * freq_hz / self.sample_rate;
    }

    /// Return the current LO frequency in Hz.
    pub fn lo_frequency(&self) -> f64 {
        self.lo_frequency_hz
    }

    /// Reset the phase accumulator to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Set mixer impairments for realistic simulation.
    pub fn set_impairments(&mut self, imp: MixerImpairments) {
        self.impairments = Some(imp);
    }

    /// Advance the phase accumulator by one sample and wrap to [-pi, pi).
    fn advance_phase(&mut self) {
        self.phase += self.phase_inc;
        // Wrap phase to prevent loss of precision for long-running signals
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        } else if self.phase < -PI {
            self.phase += 2.0 * PI;
        }
    }

    /// Apply impairments to a mixed sample.
    ///
    /// - LO leakage: adds a fraction of the LO signal to the output
    /// - IQ phase error: skews the Q channel by the quadrature error
    /// - Image rejection: adds a fraction of the conjugate signal
    fn apply_impairments(
        imp: &MixerImpairments,
        mut out_i: f64,
        mut out_q: f64,
        cos_phase: f64,
        sin_phase: f64,
    ) -> (f64, f64) {
        // LO leakage: add residual LO to output
        let lo_leak_lin = 10.0_f64.powf(imp.lo_leakage_db / 20.0);
        out_i += lo_leak_lin * cos_phase;
        out_q += lo_leak_lin * sin_phase;

        // IQ phase error: rotate Q channel by the phase error
        let phase_err_rad = imp.iq_phase_error_deg * PI / 180.0;
        let q_rotated = out_q * phase_err_rad.cos() + out_i * phase_err_rad.sin();
        out_q = q_rotated;

        // Image rejection: add fraction of conjugate (image)
        let img_lin = 10.0_f64.powf(imp.image_rejection_db / 20.0);
        out_i += img_lin * out_i;
        out_q -= img_lin * out_q;

        (out_i, out_q)
    }
}

/// Real-valued RF mixer for converting real signals to complex baseband.
///
/// Takes a real-valued input, multiplies by both cosine and sine of the LO
/// to produce an analytic (complex) signal suitable for further baseband
/// processing.
#[derive(Debug, Clone)]
pub struct RealMixer {
    /// LO frequency in Hz
    lo_frequency_hz: f64,
    /// Sample rate in Hz
    sample_rate: f64,
    /// Phase increment per sample (radians)
    phase_inc: f64,
    /// Phase accumulator (radians)
    phase: f64,
}

impl RealMixer {
    /// Create a new real-to-complex mixer with the given LO frequency and sample rate.
    pub fn new(lo_frequency_hz: f64, sample_rate: f64) -> Self {
        let phase_inc = 2.0 * PI * lo_frequency_hz / sample_rate;
        Self {
            lo_frequency_hz,
            sample_rate,
            phase_inc,
            phase: 0.0,
        }
    }

    /// Mix a real-valued signal down to complex baseband.
    ///
    /// Each real sample is multiplied by cos(phase) and -sin(phase) to produce
    /// I and Q components, effectively performing an analytic signal conversion
    /// centered at the LO frequency.
    pub fn mix(&mut self, input: &[f64]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let cos_phase = self.phase.cos();
            let sin_phase = self.phase.sin();
            // Downconvert: multiply by exp(-j*phase)
            let out_i = x * cos_phase;
            let out_q = -x * sin_phase;
            output.push((out_i, out_q));

            self.phase += self.phase_inc;
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
        }
        output
    }
}

/// Generate a complex sinusoid (NCO output) at the given frequency and sample rate.
///
/// Produces `num_samples` samples of `exp(j * 2 * pi * frequency * n / sample_rate)`.
/// All samples have unit amplitude.
pub fn nco_generate(frequency_hz: f64, sample_rate: f64, num_samples: usize) -> Vec<(f64, f64)> {
    let phase_inc = 2.0 * PI * frequency_hz / sample_rate;
    let mut phase = 0.0_f64;
    let mut output = Vec::with_capacity(num_samples);
    for _ in 0..num_samples {
        output.push((phase.cos(), phase.sin()));
        phase += phase_inc;
        if phase > PI {
            phase -= 2.0 * PI;
        } else if phase < -PI {
            phase += 2.0 * PI;
        }
    }
    output
}

/// Create a baseband (zero-LO) mixer that acts as a passthrough.
///
/// With LO frequency of zero, the mixer multiplies by exp(j*0) = 1,
/// passing the signal through unchanged.
pub fn baseband_mixer(sample_rate: f64) -> RfMixer {
    RfMixer::new(0.0, sample_rate)
}

/// Create an IF (intermediate frequency) downconverter mixer.
///
/// Pre-configured for a typical IF stage where signals are shifted
/// from an intermediate frequency down to baseband.
pub fn if_mixer(if_freq_hz: f64, sample_rate: f64) -> RfMixer {
    RfMixer::new(if_freq_hz, sample_rate)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: compute magnitude of a complex tuple
    fn mag(s: (f64, f64)) -> f64 {
        (s.0 * s.0 + s.1 * s.1).sqrt()
    }

    /// Helper: estimate dominant frequency by finding the peak FFT bin
    /// using a simple DFT (no external crate). Returns frequency in Hz.
    fn estimate_frequency(signal: &[(f64, f64)], sample_rate: f64) -> f64 {
        let n = signal.len();
        let mut best_k = 0;
        let mut best_mag = 0.0_f64;
        for k in 0..n {
            let mut sum_re = 0.0;
            let mut sum_im = 0.0;
            for (i, &(re, im)) in signal.iter().enumerate() {
                let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
                sum_re += re * angle.cos() - im * angle.sin();
                sum_im += re * angle.sin() + im * angle.cos();
            }
            let m = (sum_re * sum_re + sum_im * sum_im).sqrt();
            if m > best_mag {
                best_mag = m;
                best_k = k;
            }
        }
        // Map bin index to frequency (handle negative frequencies)
        let freq_bin = if best_k > n / 2 {
            best_k as f64 - n as f64
        } else {
            best_k as f64
        };
        freq_bin * sample_rate / n as f64
    }

    #[test]
    fn test_downconversion_shifts_frequency() {
        // Generate a tone at 1000 Hz, downconvert by 1000 Hz -> should be at DC
        let sample_rate = 10000.0;
        let f_signal = 1000.0;
        let n = 256;

        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let angle = 2.0 * PI * f_signal * t;
                (angle.cos(), angle.sin())
            })
            .collect();

        let mut mixer = RfMixer::new(f_signal, sample_rate);
        let output = mixer.mix_down(&signal);

        // After downconversion, frequency should be ~0 Hz (DC)
        let freq = estimate_frequency(&output, sample_rate);
        assert!(
            freq.abs() < sample_rate / n as f64 * 1.5,
            "Expected ~0 Hz, got {} Hz",
            freq
        );
    }

    #[test]
    fn test_upconversion_shifts_frequency() {
        // Start with DC (constant), upconvert by 1000 Hz -> tone at 1000 Hz
        let sample_rate = 10000.0;
        let f_lo = 1000.0;
        let n = 256;

        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0); n];

        let mut mixer = RfMixer::new(f_lo, sample_rate);
        let output = mixer.mix_up(&signal);

        let freq = estimate_frequency(&output, sample_rate);
        assert!(
            (freq - f_lo).abs() < sample_rate / n as f64 * 1.5,
            "Expected ~{} Hz, got {} Hz",
            f_lo,
            freq
        );
    }

    #[test]
    fn test_roundtrip_mix_down_then_up() {
        let sample_rate = 10000.0;
        let f_lo = 500.0;
        let n = 128;

        // Generate a baseband signal
        let original: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let angle = 2.0 * PI * 200.0 * t; // 200 Hz baseband tone
                (angle.cos(), angle.sin())
            })
            .collect();

        // Upconvert then downconvert
        let mut mixer_up = RfMixer::new(f_lo, sample_rate);
        let mut mixer_down = RfMixer::new(f_lo, sample_rate);

        let upconverted = mixer_up.mix_up(&original);
        let recovered = mixer_down.mix_down(&upconverted);

        // The recovered signal should closely match the original
        for i in 0..n {
            assert!(
                (recovered[i].0 - original[i].0).abs() < 1e-10,
                "I mismatch at sample {}: {} vs {}",
                i,
                recovered[i].0,
                original[i].0
            );
            assert!(
                (recovered[i].1 - original[i].1).abs() < 1e-10,
                "Q mismatch at sample {}: {} vs {}",
                i,
                recovered[i].1,
                original[i].1
            );
        }
    }

    #[test]
    fn test_phase_continuity_across_blocks() {
        let sample_rate = 10000.0;
        let f_lo = 1000.0;
        let block_size = 64;

        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0); block_size * 2];

        // Process in one block
        let mut mixer_one = RfMixer::new(f_lo, sample_rate);
        let one_block = mixer_one.mix_up(&signal);

        // Process in two blocks
        let mut mixer_two = RfMixer::new(f_lo, sample_rate);
        let first_half = mixer_two.mix_up(&signal[..block_size]);
        let second_half = mixer_two.mix_up(&signal[block_size..]);

        // Results should be identical
        for i in 0..block_size {
            assert!(
                (first_half[i].0 - one_block[i].0).abs() < 1e-10,
                "Phase discontinuity at sample {} (first half)",
                i
            );
            assert!(
                (first_half[i].1 - one_block[i].1).abs() < 1e-10,
                "Phase discontinuity at sample {} (first half Q)",
                i
            );
        }
        for i in 0..block_size {
            assert!(
                (second_half[i].0 - one_block[block_size + i].0).abs() < 1e-10,
                "Phase discontinuity at sample {} (second half)",
                i
            );
            assert!(
                (second_half[i].1 - one_block[block_size + i].1).abs() < 1e-10,
                "Phase discontinuity at sample {} (second half Q)",
                i
            );
        }
    }

    #[test]
    fn test_lo_frequency_change() {
        let sample_rate = 10000.0;
        let mut mixer = RfMixer::new(1000.0, sample_rate);

        assert!((mixer.lo_frequency() - 1000.0).abs() < 1e-10);

        mixer.set_lo_frequency(2000.0);
        assert!((mixer.lo_frequency() - 2000.0).abs() < 1e-10);

        // Verify the new frequency takes effect
        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0); 256];
        let output = mixer.mix_up(&signal);

        let freq = estimate_frequency(&output, sample_rate);
        assert!(
            (freq - 2000.0).abs() < sample_rate / 256.0 * 1.5,
            "Expected ~2000 Hz after LO change, got {} Hz",
            freq
        );
    }

    #[test]
    fn test_real_mixer_produces_complex_output() {
        let sample_rate = 10000.0;
        let f_lo = 1000.0;
        let n = 128;

        // Real-valued sinusoid at f_lo
        let real_input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * f_lo * t).cos()
            })
            .collect();

        let mut mixer = RealMixer::new(f_lo, sample_rate);
        let output = mixer.mix(&real_input);

        assert_eq!(output.len(), n);

        // Output should be complex (non-zero Q channel for non-DC frequencies)
        // After mixing cos(wt) with exp(-jwt), we get 0.5 + 0.5*exp(-j2wt)
        // The DC component should be present
        let mut sum_i = 0.0;
        let mut sum_q = 0.0;
        for &(i, q) in &output {
            sum_i += i;
            sum_q += q;
        }
        // DC component of I should be ~0.5 * n
        let avg_i = sum_i / n as f64;
        assert!(
            (avg_i - 0.5).abs() < 0.05,
            "Expected DC I component ~0.5, got {}",
            avg_i
        );
    }

    #[test]
    fn test_nco_unit_amplitude() {
        let samples = nco_generate(1000.0, 48000.0, 480);

        for (i, &s) in samples.iter().enumerate() {
            let amplitude = mag(s);
            assert!(
                (amplitude - 1.0).abs() < 1e-10,
                "NCO sample {} has amplitude {}, expected 1.0",
                i,
                amplitude
            );
        }
    }

    #[test]
    fn test_baseband_mixer_passthrough() {
        let sample_rate = 10000.0;
        let mut mixer = baseband_mixer(sample_rate);

        assert!((mixer.lo_frequency() - 0.0).abs() < 1e-10);

        let input: Vec<(f64, f64)> = vec![(1.0, 0.5), (-0.3, 0.7), (0.0, -1.0)];
        let output = mixer.mix_down(&input);

        // With zero LO, output should equal input
        for i in 0..input.len() {
            assert!(
                (output[i].0 - input[i].0).abs() < 1e-10,
                "Passthrough I mismatch at {}",
                i
            );
            assert!(
                (output[i].1 - input[i].1).abs() < 1e-10,
                "Passthrough Q mismatch at {}",
                i
            );
        }
    }

    #[test]
    fn test_if_mixer_factory() {
        let if_freq = 10_700_000.0; // 10.7 MHz IF
        let sample_rate = 50_000_000.0; // 50 MHz sample rate
        let mixer = if_mixer(if_freq, sample_rate);

        assert!((mixer.lo_frequency() - if_freq).abs() < 1e-10);
        assert!((mixer.sample_rate - sample_rate).abs() < 1e-10);
    }

    #[test]
    fn test_reset_clears_phase() {
        let sample_rate = 10000.0;
        let f_lo = 1000.0;

        let mut mixer = RfMixer::new(f_lo, sample_rate);

        // Process some samples to advance phase
        let signal: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let _ = mixer.mix_up(&signal);

        // Reset and process again
        mixer.reset();
        let after_reset = mixer.mix_up(&signal);

        // Also process with a fresh mixer
        let mut fresh = RfMixer::new(f_lo, sample_rate);
        let from_fresh = fresh.mix_up(&signal);

        // Results should be identical
        for i in 0..signal.len() {
            assert!(
                (after_reset[i].0 - from_fresh[i].0).abs() < 1e-10,
                "Reset did not clear phase at sample {} I",
                i
            );
            assert!(
                (after_reset[i].1 - from_fresh[i].1).abs() < 1e-10,
                "Reset did not clear phase at sample {} Q",
                i
            );
        }
    }
}
