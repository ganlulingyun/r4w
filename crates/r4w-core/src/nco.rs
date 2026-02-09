//! Numerically Controlled Oscillator (NCO) / Voltage Controlled Oscillator (VCO)
//!
//! Generates complex sinusoids at a specified frequency with phase continuity.
//! Core building block for PLLs, FM modulation/demodulation, frequency
//! translation, and Doppler simulation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::nco::Nco;
//! use std::f64::consts::PI;
//!
//! // Generate a 1 kHz tone at 48 kHz sample rate
//! let mut nco = Nco::new(1000.0, 48000.0);
//! let samples = nco.generate(48); // 1ms of samples
//! assert_eq!(samples.len(), 48);
//!
//! // All samples should have unit amplitude
//! for s in &samples {
//!     assert!((s.norm() - 1.0).abs() < 1e-10);
//! }
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Numerically Controlled Oscillator.
///
/// Generates `exp(j * (2*pi*f*n/fs + phi))` sample by sample with exact
/// phase tracking. The frequency can be adjusted at any time without
/// phase discontinuity.
///
/// This is equivalent to GNU Radio's `sig_source_c` in CW mode, or the
/// NCO used inside `costas_loop_cc` and `pll_refout_cc`.
#[derive(Debug, Clone)]
pub struct Nco {
    /// Phase accumulator (radians)
    phase: f64,
    /// Phase increment per sample (radians)
    phase_inc: f64,
    /// Sample rate (Hz)
    sample_rate: f64,
    /// Frequency (Hz)
    frequency: f64,
    /// Amplitude
    amplitude: f64,
}

impl Nco {
    /// Create a new NCO at the given frequency and sample rate.
    pub fn new(frequency: f64, sample_rate: f64) -> Self {
        let phase_inc = 2.0 * PI * frequency / sample_rate;
        Self {
            phase: 0.0,
            phase_inc,
            sample_rate,
            frequency,
            amplitude: 1.0,
        }
    }

    /// Create an NCO with a specified initial phase.
    pub fn with_phase(frequency: f64, sample_rate: f64, phase: f64) -> Self {
        let mut nco = Self::new(frequency, sample_rate);
        nco.phase = phase;
        nco
    }

    /// Create an NCO with a specified amplitude.
    pub fn with_amplitude(frequency: f64, sample_rate: f64, amplitude: f64) -> Self {
        let mut nco = Self::new(frequency, sample_rate);
        nco.amplitude = amplitude;
        nco
    }

    /// Generate a single complex sample and advance phase.
    pub fn step(&mut self) -> Complex64 {
        let sample = Complex64::new(
            self.amplitude * self.phase.cos(),
            self.amplitude * self.phase.sin(),
        );
        self.phase += self.phase_inc;
        // Wrap phase to prevent floating-point drift
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        } else if self.phase < -PI {
            self.phase += 2.0 * PI;
        }
        sample
    }

    /// Generate a block of samples.
    pub fn generate(&mut self, num_samples: usize) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            output.push(self.step());
        }
        output
    }

    /// Mix (frequency-translate) a signal by multiplying with NCO output.
    ///
    /// Shifts signal up or down in frequency depending on NCO frequency sign.
    pub fn mix(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|&s| {
                let lo = self.step();
                s * lo
            })
            .collect()
    }

    /// Mix in place (avoids allocation).
    pub fn mix_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            let lo = self.step();
            *s *= lo;
        }
    }

    /// Set frequency (with phase continuity).
    pub fn set_frequency(&mut self, frequency: f64) {
        self.frequency = frequency;
        self.phase_inc = 2.0 * PI * frequency / self.sample_rate;
    }

    /// Adjust frequency by a delta (for PLL/VCO use).
    pub fn adjust_frequency(&mut self, delta_hz: f64) {
        self.set_frequency(self.frequency + delta_hz);
    }

    /// Set the phase directly.
    pub fn set_phase(&mut self, phase: f64) {
        self.phase = phase;
    }

    /// Adjust phase by a delta (for PLL use).
    pub fn adjust_phase(&mut self, delta_rad: f64) {
        self.phase += delta_rad;
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Get current frequency.
    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Get phase increment per sample (radians).
    pub fn phase_inc(&self) -> f64 {
        self.phase_inc
    }

    /// Set amplitude.
    pub fn set_amplitude(&mut self, amplitude: f64) {
        self.amplitude = amplitude;
    }

    /// Reset phase to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

/// FM Modulator using NCO.
///
/// Converts a real-valued modulating signal to a frequency-modulated
/// complex baseband signal.
///
/// ```text
/// x_fm(t) = exp(j * 2*pi * dev * integral(m(t) dt))
/// ```
#[derive(Debug, Clone)]
pub struct FmModulator {
    /// Phase accumulator
    phase: f64,
    /// Frequency deviation (Hz)
    deviation: f64,
    /// Sample rate (Hz)
    sample_rate: f64,
    /// Amplitude
    amplitude: f64,
}

impl FmModulator {
    /// Create a new FM modulator.
    pub fn new(deviation: f64, sample_rate: f64) -> Self {
        Self {
            phase: 0.0,
            deviation,
            sample_rate,
            amplitude: 1.0,
        }
    }

    /// Modulate a real-valued signal.
    pub fn modulate(&mut self, input: &[f64]) -> Vec<Complex64> {
        let sensitivity = 2.0 * PI * self.deviation / self.sample_rate;
        input
            .iter()
            .map(|&m| {
                self.phase += sensitivity * m;
                // Wrap phase
                if self.phase > PI {
                    self.phase -= 2.0 * PI;
                } else if self.phase < -PI {
                    self.phase += 2.0 * PI;
                }
                Complex64::new(
                    self.amplitude * self.phase.cos(),
                    self.amplitude * self.phase.sin(),
                )
            })
            .collect()
    }

    /// Reset modulator state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

/// FM Demodulator (frequency discriminator).
///
/// Extracts instantaneous frequency from a complex signal using
/// the arctangent discriminator: `f = (1/2pi) * d(phase)/dt`.
#[derive(Debug, Clone)]
pub struct FmDemodulator {
    /// Previous sample for differentiation
    prev: Complex64,
    /// Sample rate
    sample_rate: f64,
    /// Frequency deviation for normalization
    deviation: f64,
}

impl FmDemodulator {
    /// Create a new FM demodulator.
    pub fn new(deviation: f64, sample_rate: f64) -> Self {
        Self {
            prev: Complex64::new(1.0, 0.0),
            sample_rate,
            deviation,
        }
    }

    /// Demodulate a complex FM signal to real-valued output.
    pub fn demodulate(&mut self, input: &[Complex64]) -> Vec<f64> {
        let sensitivity = 2.0 * PI * self.deviation / self.sample_rate;
        input
            .iter()
            .map(|&s| {
                // Conjugate multiply for phase difference
                let diff = s * self.prev.conj();
                self.prev = s;
                // Instantaneous frequency = phase difference / sensitivity
                diff.arg() / sensitivity
            })
            .collect()
    }

    /// Reset demodulator state.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nco_frequency() {
        let freq = 1000.0;
        let sr = 48000.0;
        let mut nco = Nco::new(freq, sr);
        let samples = nco.generate(480); // 10ms

        // All samples should have unit amplitude
        for s in &samples {
            assert!(
                (s.norm() - 1.0).abs() < 1e-10,
                "NCO amplitude should be 1.0"
            );
        }

        // Check that phase increment is correct
        let expected_phase_inc = 2.0 * PI * freq / sr;
        assert!(
            (nco.phase_inc() - expected_phase_inc).abs() < 1e-10,
            "Phase increment should match"
        );
        // Phase wraps to [-pi, pi], so just verify it's in valid range
        assert!(
            nco.phase().abs() <= PI + 0.01,
            "Phase should be wrapped: got {:.3}",
            nco.phase()
        );
    }

    #[test]
    fn test_nco_mix_shift() {
        let sr = 48000.0;
        let mut nco = Nco::new(1000.0, sr); // 1 kHz LO

        // Input: DC signal
        let dc = vec![Complex64::new(1.0, 0.0); 480];
        let mixed = nco.mix(&dc);

        // Output should be a 1 kHz tone
        // Check that it's not DC anymore
        let dc_power: f64 = mixed.iter().map(|s| s.re).sum::<f64>().abs() / mixed.len() as f64;
        assert!(
            dc_power < 0.5,
            "DC mixed with NCO should not be DC: dc_component = {dc_power:.3}"
        );
    }

    #[test]
    fn test_nco_phase_continuity() {
        let mut nco = Nco::new(1000.0, 48000.0);
        let _ = nco.generate(100);
        let phase1 = nco.phase();

        // Change frequency - phase should be continuous
        nco.set_frequency(2000.0);
        let s = nco.step();
        let phase2 = nco.phase();

        // Phase should have advanced by new phase_inc from phase1
        let expected_phase_inc = 2.0 * PI * 2000.0 / 48000.0;
        let actual_inc = phase2 - phase1;
        assert!(
            (actual_inc - expected_phase_inc).abs() < 0.01,
            "Frequency change should maintain phase continuity"
        );
    }

    #[test]
    fn test_nco_zero_frequency() {
        let mut nco = Nco::new(0.0, 48000.0);
        let samples = nco.generate(100);

        // Should output constant 1+0j
        for s in &samples {
            assert!(
                (s - Complex64::new(1.0, 0.0)).norm() < 1e-10,
                "Zero-frequency NCO should output DC"
            );
        }
    }

    #[test]
    fn test_nco_amplitude() {
        let mut nco = Nco::with_amplitude(1000.0, 48000.0, 0.5);
        let samples = nco.generate(100);

        for s in &samples {
            assert!(
                (s.norm() - 0.5).abs() < 1e-10,
                "NCO amplitude should be 0.5"
            );
        }
    }

    #[test]
    fn test_fm_roundtrip() {
        let sr = 48000.0;
        let dev = 5000.0;

        let mut mod_ = FmModulator::new(dev, sr);
        let mut demod = FmDemodulator::new(dev, sr);

        // Constant modulating signal (should produce tone at deviation)
        let modulating: Vec<f64> = vec![0.5; 200];
        let fm_signal = mod_.modulate(&modulating);
        let recovered = demod.demodulate(&fm_signal);

        // After initial transient, recovered should match input
        for &r in &recovered[10..] {
            assert!(
                (r - 0.5).abs() < 0.1,
                "FM roundtrip failed: got {r:.3}, expected 0.5"
            );
        }
    }

    #[test]
    fn test_fm_silence() {
        let mut mod_ = FmModulator::new(5000.0, 48000.0);
        let mut demod = FmDemodulator::new(5000.0, 48000.0);

        // Zero modulating signal (carrier only)
        let modulating: Vec<f64> = vec![0.0; 100];
        let fm_signal = mod_.modulate(&modulating);

        // All samples should have unit amplitude
        for s in &fm_signal {
            assert!(
                (s.norm() - 1.0).abs() < 1e-10,
                "FM carrier should have unit amplitude"
            );
        }

        let recovered = demod.demodulate(&fm_signal);
        for &r in &recovered[1..] {
            assert!(
                r.abs() < 0.01,
                "FM of zero should recover zero: got {r:.3}"
            );
        }
    }

    #[test]
    fn test_nco_reset() {
        let mut nco = Nco::new(1000.0, 48000.0);
        let run1 = nco.generate(100);
        nco.reset();
        let run2 = nco.generate(100);

        for (a, b) in run1.iter().zip(run2.iter()) {
            assert!(
                (a - b).norm() < 1e-10,
                "Reset should produce identical output"
            );
        }
    }

    #[test]
    fn test_nco_adjust_frequency() {
        let mut nco = Nco::new(1000.0, 48000.0);
        assert!((nco.frequency() - 1000.0).abs() < 1e-10);

        nco.adjust_frequency(500.0);
        assert!((nco.frequency() - 1500.0).abs() < 1e-10);
    }

    #[test]
    fn test_fm_modulator_varying_signal() {
        let sr = 48000.0;
        let dev = 5000.0;
        let mut mod_ = FmModulator::new(dev, sr);

        // Sinusoidal modulating signal
        let modulating: Vec<f64> = (0..480)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / sr).sin())
            .collect();

        let fm_signal = mod_.modulate(&modulating);

        // FM signal should have constant envelope
        for s in &fm_signal {
            assert!(
                (s.norm() - 1.0).abs() < 1e-10,
                "FM should have constant envelope"
            );
        }
    }
}
