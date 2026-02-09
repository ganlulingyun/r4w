//! Frequency Modulator
//!
//! Converts a baseband signal to FM by modulating the instantaneous frequency.
//! Complements `quadrature_demod.rs` (FM demodulator) for complete FM TX/RX.
//!
//! ## Variants
//!
//! - **FrequencyModulator**: Continuous-phase FM with configurable sensitivity
//! - **FskModulator**: Discrete frequency-shift keying (binary and M-ary)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::frequency_modulator::FrequencyModulator;
//!
//! let mut fm = FrequencyModulator::new(5000.0, 48000.0); // 5kHz max deviation
//! let audio = vec![0.5; 100]; // constant baseband → constant frequency
//! let modulated = fm.modulate(&audio);
//! assert_eq!(modulated.len(), 100);
//! // Output magnitude should be ~1.0 (constant envelope)
//! for z in &modulated {
//!     assert!((z.norm() - 1.0).abs() < 1e-10);
//! }
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

const TWO_PI: f64 = 2.0 * PI;

/// Continuous-phase frequency modulator.
///
/// `output[n] = exp(j * phase[n])` where
/// `phase[n] = phase[n-1] + 2π * sensitivity * input[n] / sample_rate`
#[derive(Debug, Clone)]
pub struct FrequencyModulator {
    /// Sensitivity in Hz per unit input (= max frequency deviation)
    sensitivity: f64,
    /// Sample rate in Hz
    sample_rate: f64,
    /// Phase increment scale factor: 2π * sensitivity / sample_rate
    phase_scale: f64,
    /// Current accumulated phase
    phase: f64,
}

impl FrequencyModulator {
    /// Create a new FM modulator.
    ///
    /// - `sensitivity`: Maximum frequency deviation in Hz (when input = ±1.0)
    /// - `sample_rate`: Sample rate in Hz
    pub fn new(sensitivity: f64, sample_rate: f64) -> Self {
        Self {
            sensitivity,
            sample_rate,
            phase_scale: TWO_PI * sensitivity / sample_rate,
            phase: 0.0,
        }
    }

    /// Create for NBFM (5 kHz deviation at 48 kHz sample rate).
    pub fn nbfm(sample_rate: f64) -> Self {
        Self::new(5000.0, sample_rate)
    }

    /// Create for WBFM (75 kHz deviation at 240 kHz sample rate).
    pub fn wbfm(sample_rate: f64) -> Self {
        Self::new(75000.0, sample_rate)
    }

    /// Modulate a block of real-valued baseband samples.
    pub fn modulate(&mut self, input: &[f64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            self.phase += self.phase_scale * x;
            // Wrap phase to prevent precision loss
            if self.phase > TWO_PI {
                self.phase -= TWO_PI;
            } else if self.phase < -TWO_PI {
                self.phase += TWO_PI;
            }
            output.push(Complex64::new(self.phase.cos(), self.phase.sin()));
        }
        output
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Reset phase to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Get sensitivity (Hz per unit).
    pub fn sensitivity(&self) -> f64 {
        self.sensitivity
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }
}

/// Binary/M-ary FSK modulator.
///
/// Maps integer symbol indices to frequency tones, then generates
/// continuous-phase FM output.
#[derive(Debug, Clone)]
pub struct FskModulator {
    /// Frequency tones for each symbol (Hz offset from center)
    tones: Vec<f64>,
    /// Samples per symbol
    samples_per_symbol: usize,
    /// Sample rate
    sample_rate: f64,
    /// Current phase
    phase: f64,
}

impl FskModulator {
    /// Create an M-ary FSK modulator.
    ///
    /// - `tones`: Frequency offset for each symbol index (Hz)
    /// - `samples_per_symbol`: Number of output samples per symbol
    /// - `sample_rate`: Output sample rate in Hz
    pub fn new(tones: Vec<f64>, samples_per_symbol: usize, sample_rate: f64) -> Self {
        assert!(!tones.is_empty());
        assert!(samples_per_symbol > 0);
        Self {
            tones,
            samples_per_symbol,
            sample_rate,
            phase: 0.0,
        }
    }

    /// Create binary FSK modulator (2FSK).
    ///
    /// - `deviation`: Frequency deviation from center (Hz)
    pub fn binary(deviation: f64, samples_per_symbol: usize, sample_rate: f64) -> Self {
        Self::new(vec![-deviation, deviation], samples_per_symbol, sample_rate)
    }

    /// Modulate a sequence of symbol indices to IQ samples.
    pub fn modulate(&mut self, symbols: &[usize]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(symbols.len() * self.samples_per_symbol);

        for &sym in symbols {
            let freq = if sym < self.tones.len() {
                self.tones[sym]
            } else {
                0.0
            };
            let phase_inc = TWO_PI * freq / self.sample_rate;

            for _ in 0..self.samples_per_symbol {
                self.phase += phase_inc;
                if self.phase > TWO_PI {
                    self.phase -= TWO_PI;
                } else if self.phase < -TWO_PI {
                    self.phase += TWO_PI;
                }
                output.push(Complex64::new(self.phase.cos(), self.phase.sin()));
            }
        }

        output
    }

    /// Modulate bits (binary FSK only).
    pub fn modulate_bits(&mut self, bits: &[bool]) -> Vec<Complex64> {
        let symbols: Vec<usize> = bits.iter().map(|&b| if b { 1 } else { 0 }).collect();
        self.modulate(&symbols)
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
    fn test_fm_constant_envelope() {
        let mut fm = FrequencyModulator::new(5000.0, 48000.0);
        let audio = vec![0.5; 200];
        let output = fm.modulate(&audio);
        for z in &output {
            assert!((z.norm() - 1.0).abs() < 1e-10, "Envelope should be 1.0");
        }
    }

    #[test]
    fn test_fm_zero_input() {
        let mut fm = FrequencyModulator::new(5000.0, 48000.0);
        let output = fm.modulate(&[0.0; 100]);
        // Zero input → constant phase → output stays at (1,0) approximately
        // Actually first sample: phase += 0, so output = cos(0), sin(0) = (1,0)
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!(output[0].im.abs() < 1e-10);
    }

    #[test]
    fn test_fm_nbfm_factory() {
        let fm = FrequencyModulator::nbfm(48000.0);
        assert_eq!(fm.sensitivity(), 5000.0);
        assert_eq!(fm.sample_rate(), 48000.0);
    }

    #[test]
    fn test_fm_wbfm_factory() {
        let fm = FrequencyModulator::wbfm(240000.0);
        assert_eq!(fm.sensitivity(), 75000.0);
    }

    #[test]
    fn test_fm_reset() {
        let mut fm = FrequencyModulator::new(5000.0, 48000.0);
        fm.modulate(&[1.0; 100]);
        assert!(fm.phase().abs() > 0.01);
        fm.reset();
        assert_eq!(fm.phase(), 0.0);
    }

    #[test]
    fn test_fm_positive_frequency() {
        let mut fm = FrequencyModulator::new(1000.0, 48000.0);
        let input = vec![1.0; 48]; // 1 ms at 48kHz, +1000 Hz deviation
        let output = fm.modulate(&input);
        // Phase should advance by 2π * 1000 / 48000 per sample ≈ 0.131 rad
        // After 48 samples: total phase ≈ 48 * 0.131 ≈ 6.28 ≈ 2π → one full cycle
        assert_eq!(output.len(), 48);
    }

    #[test]
    fn test_fsk_binary() {
        let mut fsk = FskModulator::binary(1200.0, 10, 48000.0);
        let symbols = vec![0, 1, 0, 1];
        let output = fsk.modulate(&symbols);
        assert_eq!(output.len(), 40); // 4 symbols * 10 sps
        // All constant envelope
        for z in &output {
            assert!((z.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_fsk_modulate_bits() {
        let mut fsk = FskModulator::binary(1200.0, 8, 48000.0);
        let bits = vec![false, true, false, true, true];
        let output = fsk.modulate_bits(&bits);
        assert_eq!(output.len(), 40); // 5 bits * 8 sps
    }

    #[test]
    fn test_fsk_4ary() {
        let tones = vec![-3000.0, -1000.0, 1000.0, 3000.0];
        let mut fsk = FskModulator::new(tones, 16, 48000.0);
        let symbols = vec![0, 1, 2, 3];
        let output = fsk.modulate(&symbols);
        assert_eq!(output.len(), 64);
    }

    #[test]
    fn test_fsk_reset() {
        let mut fsk = FskModulator::binary(1200.0, 10, 48000.0);
        fsk.modulate(&[0, 1]);
        fsk.reset();
        assert_eq!(fsk.phase, 0.0);
    }

    #[test]
    fn test_fm_empty_input() {
        let mut fm = FrequencyModulator::new(5000.0, 48000.0);
        assert!(fm.modulate(&[]).is_empty());
    }

    #[test]
    fn test_fsk_out_of_range_symbol() {
        let mut fsk = FskModulator::binary(1200.0, 4, 48000.0);
        let output = fsk.modulate(&[99]); // Out of range → 0 Hz
        assert_eq!(output.len(), 4);
        // Phase shouldn't change (0 Hz)
    }
}
