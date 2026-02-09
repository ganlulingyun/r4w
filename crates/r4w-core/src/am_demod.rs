//! AM Demodulator — Amplitude modulation envelope detection
//!
//! Complete AM demodulation from baseband complex to audio. Includes
//! envelope detection, DC blocking, and optional audio lowpass filtering.
//! GNU Radio equivalent: `am_demod_cf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::am_demod::AmDemod;
//! use num_complex::Complex64;
//!
//! let mut demod = AmDemod::new(48000.0, 5000.0);
//! // AM signal: carrier with modulation
//! let signal: Vec<Complex64> = (0..100).map(|i| {
//!     let t = i as f64 / 48000.0;
//!     let modulation = 1.0 + 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
//!     Complex64::new(modulation, 0.0)
//! }).collect();
//! let audio = demod.process(&signal);
//! assert_eq!(audio.len(), 100);
//! ```

use num_complex::Complex64;

/// AM envelope demodulator.
///
/// Pipeline: |x[n]| → DC block → lowpass filter → output
#[derive(Debug, Clone)]
pub struct AmDemod {
    /// Audio sample rate.
    sample_rate: f64,
    /// Audio lowpass cutoff frequency.
    audio_cutoff: f64,
    /// DC blocker state.
    dc_prev_in: f64,
    dc_prev_out: f64,
    /// DC blocker coefficient.
    dc_alpha: f64,
    /// Lowpass filter state (single-pole IIR).
    lp_state: f64,
    /// Lowpass filter coefficient.
    lp_alpha: f64,
    /// Whether audio filtering is enabled.
    filter_enabled: bool,
}

impl AmDemod {
    /// Create an AM demodulator.
    ///
    /// `sample_rate`: Audio sample rate in Hz.
    /// `audio_cutoff`: Audio lowpass cutoff in Hz (typically 5000 for AM broadcast).
    pub fn new(sample_rate: f64, audio_cutoff: f64) -> Self {
        let lp_alpha = 1.0 - (-2.0 * std::f64::consts::PI * audio_cutoff / sample_rate).exp();
        Self {
            sample_rate,
            audio_cutoff,
            dc_prev_in: 0.0,
            dc_prev_out: 0.0,
            dc_alpha: 0.95,
            lp_state: 0.0,
            lp_alpha,
            filter_enabled: true,
        }
    }

    /// Create without audio filtering (raw envelope).
    pub fn unfiltered(sample_rate: f64) -> Self {
        let mut d = Self::new(sample_rate, 5000.0);
        d.filter_enabled = false;
        d
    }

    /// Demodulate a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            // Envelope detection
            let envelope = x.norm();

            // DC blocking filter: y[n] = x[n] - x[n-1] + α·y[n-1]
            let dc_blocked = envelope - self.dc_prev_in + self.dc_alpha * self.dc_prev_out;
            self.dc_prev_in = envelope;
            self.dc_prev_out = dc_blocked;

            // Optional audio lowpass
            let out = if self.filter_enabled {
                self.lp_state = self.lp_alpha * dc_blocked + (1.0 - self.lp_alpha) * self.lp_state;
                self.lp_state
            } else {
                dc_blocked
            };

            output.push(out);
        }
        output
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: Complex64) -> f64 {
        let envelope = x.norm();
        let dc_blocked = envelope - self.dc_prev_in + self.dc_alpha * self.dc_prev_out;
        self.dc_prev_in = envelope;
        self.dc_prev_out = dc_blocked;

        if self.filter_enabled {
            self.lp_state = self.lp_alpha * dc_blocked + (1.0 - self.lp_alpha) * self.lp_state;
            self.lp_state
        } else {
            dc_blocked
        }
    }

    /// Get the audio cutoff frequency.
    pub fn audio_cutoff(&self) -> f64 {
        self.audio_cutoff
    }

    /// Set the audio cutoff frequency.
    pub fn set_audio_cutoff(&mut self, cutoff: f64) {
        self.audio_cutoff = cutoff;
        self.lp_alpha = 1.0 - (-2.0 * std::f64::consts::PI * cutoff / self.sample_rate).exp();
    }

    /// Set the DC blocker coefficient.
    pub fn set_dc_alpha(&mut self, alpha: f64) {
        self.dc_alpha = alpha.clamp(0.9, 0.9999);
    }

    /// Enable or disable audio filtering.
    pub fn set_filter_enabled(&mut self, enabled: bool) {
        self.filter_enabled = enabled;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.dc_prev_in = 0.0;
        self.dc_prev_out = 0.0;
        self.lp_state = 0.0;
    }
}

/// Simple AM modulator (for testing).
///
/// Produces `(1 + depth·audio[n]) · exp(j·2π·f_carrier·n/fs)`.
#[derive(Debug, Clone)]
pub struct AmModulator {
    /// Carrier frequency in Hz.
    carrier_freq: f64,
    /// Sample rate.
    sample_rate: f64,
    /// Modulation depth (0..1).
    depth: f64,
    /// Current phase.
    phase: f64,
}

impl AmModulator {
    /// Create an AM modulator.
    pub fn new(carrier_freq: f64, sample_rate: f64, depth: f64) -> Self {
        Self {
            carrier_freq,
            sample_rate,
            depth: depth.clamp(0.0, 1.0),
            phase: 0.0,
        }
    }

    /// Modulate audio samples.
    pub fn modulate(&mut self, audio: &[f64]) -> Vec<Complex64> {
        let phase_inc = 2.0 * std::f64::consts::PI * self.carrier_freq / self.sample_rate;
        audio.iter().map(|&a| {
            let envelope = 1.0 + self.depth * a;
            let output = Complex64::from_polar(envelope, self.phase);
            self.phase += phase_inc;
            if self.phase > std::f64::consts::PI {
                self.phase -= 2.0 * std::f64::consts::PI;
            }
            output
        }).collect()
    }

    /// Reset phase.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_demod_constant_envelope() {
        let mut demod = AmDemod::unfiltered(48000.0);
        // Constant magnitude signal → DC output → DC blocked to ~0
        let signal = vec![Complex64::new(1.0, 0.0); 500];
        let output = demod.process(&signal);
        // After DC blocker settles, output should be near 0
        assert!(output[499].abs() < 0.1);
    }

    #[test]
    fn test_demod_modulated_signal() {
        let mut demod = AmDemod::new(48000.0, 8000.0);
        // Create AM signal with 1 kHz modulation
        let signal: Vec<Complex64> = (0..4800).map(|i| {
            let t = i as f64 / 48000.0;
            let mod_signal = 1.0 + 0.8 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin();
            Complex64::new(mod_signal, 0.0)
        }).collect();
        let output = demod.process(&signal);
        assert_eq!(output.len(), 4800);
        // Output should have variations (not all zero)
        let max = output.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = output.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(max - min > 0.1);
    }

    #[test]
    fn test_modulator_basic() {
        let mut modulator = AmModulator::new(0.0, 48000.0, 0.5);
        let audio = vec![1.0; 10]; // Max modulation
        let output = modulator.modulate(&audio);
        // At zero carrier freq, output is real: (1 + 0.5*1) = 1.5
        assert!((output[0].re - 1.5).abs() < 1e-10);
    }

    #[test]
    fn test_mod_demod_roundtrip() {
        let fs = 48000.0;
        let mut modulator = AmModulator::new(0.0, fs, 0.5);
        let mut demod = AmDemod::new(fs, 5000.0);

        // Create a low-frequency audio tone
        let audio: Vec<f64> = (0..4800).map(|i| {
            (2.0 * std::f64::consts::PI * 100.0 * i as f64 / fs).sin()
        }).collect();

        let modulated = modulator.modulate(&audio);
        let demodulated = demod.process(&modulated);

        // After settling, demodulated should have same frequency content
        assert_eq!(demodulated.len(), 4800);
    }

    #[test]
    fn test_set_audio_cutoff() {
        let mut demod = AmDemod::new(48000.0, 3000.0);
        assert!((demod.audio_cutoff() - 3000.0).abs() < 1e-10);
        demod.set_audio_cutoff(8000.0);
        assert!((demod.audio_cutoff() - 8000.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut demod = AmDemod::new(48000.0, 5000.0);
        demod.process(&vec![Complex64::new(1.0, 1.0); 100]);
        demod.reset();
        // After reset, state should be zeroed
        let output = demod.process_sample(Complex64::new(0.0, 0.0));
        assert_eq!(output, 0.0);
    }

    #[test]
    fn test_envelope_extraction() {
        let mut demod = AmDemod::unfiltered(48000.0);
        // Constant magnitude signal → after DC blocker settles, output ≈ 0
        let signal = vec![Complex64::new(3.0, 4.0); 200]; // mag = 5
        let output = demod.process(&signal);
        // After DC blocker settles (~50 samples), output should be near 0
        assert!(output[199].abs() < 0.5, "DC blocked output should be near 0, got {}", output[199]);
    }

    #[test]
    fn test_output_length() {
        let mut demod = AmDemod::new(48000.0, 5000.0);
        let input = vec![Complex64::new(1.0, 0.0); 256];
        let output = demod.process(&input);
        assert_eq!(output.len(), 256);
    }

    #[test]
    fn test_modulator_reset() {
        let mut modulator = AmModulator::new(1000.0, 48000.0, 0.5);
        modulator.modulate(&vec![0.0; 100]);
        modulator.reset();
        let out = modulator.modulate(&[0.0]);
        // After reset, phase = 0, so output should be (1.0, 0.0)
        assert!((out[0].re - 1.0).abs() < 1e-10);
    }
}
