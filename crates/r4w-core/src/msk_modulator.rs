//! MSK Modulator — Minimum Shift Keying modulation and demodulation
//!
//! MSK is a continuous-phase FSK with modulation index h=0.5, providing
//! constant-envelope modulation with compact spectrum (main lobe width = 1.5/T).
//! Used in GSM (GMSK variant), satellite communications, and military radios.
//! GNU Radio equivalent: custom MSK blocks in gr-digital.
//!
//! ## Properties
//!
//! - Constant envelope (ideal for nonlinear amplifiers)
//! - Frequency deviation: Δf = 1/(4T) where T is symbol period
//! - Phase advances ±π/2 per symbol period (continuous)
//! - Spectrally efficient: 99% power within bandwidth 1.2/T
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::msk_modulator::MskModulator;
//!
//! let mut mod_msk = MskModulator::new(1000.0, 8000.0);
//! let bits = vec![true, false, true, true, false];
//! let signal = mod_msk.modulate(&bits);
//! assert_eq!(signal.len(), 5 * 8); // 5 bits × 8 samples/symbol
//! ```

use std::f64::consts::PI;

/// Complex sample type.
#[derive(Debug, Clone, Copy)]
pub struct MskComplex {
    pub re: f64,
    pub im: f64,
}

impl MskComplex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }

    pub fn mag(self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
}

impl std::ops::Mul for MskComplex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

/// MSK modulator.
#[derive(Debug, Clone)]
pub struct MskModulator {
    /// Symbol rate (symbols/second).
    symbol_rate: f64,
    /// Sample rate (samples/second).
    sample_rate: f64,
    /// Samples per symbol.
    samples_per_symbol: usize,
    /// Current phase accumulator (radians).
    phase: f64,
    /// Phase increment per sample for frequency deviation Δf = 1/(4T).
    delta_phase_per_sample: f64,
}

/// MSK demodulator.
#[derive(Debug, Clone)]
pub struct MskDemodulator {
    /// Symbol rate.
    symbol_rate: f64,
    /// Sample rate.
    sample_rate: f64,
    /// Samples per symbol.
    samples_per_symbol: usize,
}

/// GMSK (Gaussian MSK) modulator with pre-modulation Gaussian filter.
#[derive(Debug, Clone)]
pub struct GmskModulator {
    /// Base MSK modulator.
    msk: MskModulator,
    /// BT product (bandwidth-time, typically 0.3 for GSM).
    bt: f64,
    /// Gaussian filter taps.
    gaussian_taps: Vec<f64>,
    /// Filter state.
    filter_state: Vec<f64>,
    /// Filter span in symbols.
    filter_span: usize,
}

impl MskModulator {
    /// Create a new MSK modulator.
    ///
    /// # Arguments
    /// * `symbol_rate` - Symbol rate in Hz
    /// * `sample_rate` - Sample rate in Hz (must be integer multiple of symbol_rate)
    pub fn new(symbol_rate: f64, sample_rate: f64) -> Self {
        let samples_per_symbol = (sample_rate / symbol_rate).round() as usize;
        assert!(samples_per_symbol >= 2, "Need at least 2 samples per symbol");

        // Δf = 1/(4T) = symbol_rate/4
        // Phase increment per sample = 2π × Δf / sample_rate
        let delta_f = symbol_rate / 4.0;
        let delta_phase_per_sample = 2.0 * PI * delta_f / sample_rate;

        Self {
            symbol_rate,
            sample_rate,
            samples_per_symbol,
            phase: 0.0,
            delta_phase_per_sample,
        }
    }

    /// Modulate bits to MSK complex baseband signal.
    ///
    /// Bit true = +Δf, false = -Δf.
    pub fn modulate(&mut self, bits: &[bool]) -> Vec<MskComplex> {
        let mut output = Vec::with_capacity(bits.len() * self.samples_per_symbol);

        for &bit in bits {
            let direction = if bit { 1.0 } else { -1.0 };

            for _ in 0..self.samples_per_symbol {
                output.push(MskComplex::from_polar(1.0, self.phase));
                self.phase += direction * self.delta_phase_per_sample;
            }
        }

        // Wrap phase
        self.phase = wrap_phase(self.phase);

        output
    }

    /// Get the modulation index (always 0.5 for MSK).
    pub fn modulation_index(&self) -> f64 {
        0.5
    }

    /// Get the frequency deviation Δf = symbol_rate / 4.
    pub fn frequency_deviation(&self) -> f64 {
        self.symbol_rate / 4.0
    }

    /// Samples per symbol.
    pub fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    /// Reset phase accumulator.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }
}

impl MskDemodulator {
    /// Create a new MSK demodulator.
    pub fn new(symbol_rate: f64, sample_rate: f64) -> Self {
        let samples_per_symbol = (sample_rate / symbol_rate).round() as usize;
        Self {
            symbol_rate,
            sample_rate,
            samples_per_symbol,
        }
    }

    /// Demodulate MSK signal to bits using phase accumulation detection.
    ///
    /// Measures phase change within each symbol period.
    /// Positive phase change (+π/2) → bit 1, negative (−π/2) → bit 0.
    pub fn demodulate(&self, signal: &[MskComplex]) -> Vec<bool> {
        let sps = self.samples_per_symbol;
        let num_symbols = signal.len() / sps;

        let mut bits = Vec::with_capacity(num_symbols);

        for i in 0..num_symbols {
            let start = i * sps;
            let end = start + sps - 1;
            if end >= signal.len() {
                break;
            }

            let start_phase = signal[start].im.atan2(signal[start].re);
            let end_phase = signal[end].im.atan2(signal[end].re);
            let mut delta = end_phase - start_phase;

            // Phase unwrap
            while delta > PI {
                delta -= 2.0 * PI;
            }
            while delta < -PI {
                delta += 2.0 * PI;
            }

            bits.push(delta > 0.0);
        }

        bits
    }
}

impl GmskModulator {
    /// Create a GMSK modulator.
    ///
    /// # Arguments
    /// * `symbol_rate` - Symbol rate in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `bt` - Bandwidth-time product (0.3 for GSM, 0.5 common)
    pub fn new(symbol_rate: f64, sample_rate: f64, bt: f64) -> Self {
        let msk = MskModulator::new(symbol_rate, sample_rate);
        let filter_span = 4; // symbols
        let filter_len = filter_span * msk.samples_per_symbol + 1;
        let gaussian_taps = design_gaussian_filter(bt, msk.samples_per_symbol, filter_span);

        Self {
            msk,
            bt,
            gaussian_taps,
            filter_state: vec![0.0; filter_len],
            filter_span,
        }
    }

    /// Modulate bits to GMSK signal.
    ///
    /// Pre-filters the NRZ data with a Gaussian filter before MSK modulation.
    pub fn modulate(&mut self, bits: &[bool]) -> Vec<MskComplex> {
        let sps = self.msk.samples_per_symbol;

        // Convert to NRZ and upsample
        let mut nrz = Vec::with_capacity(bits.len() * sps);
        for &bit in bits {
            let val = if bit { 1.0 } else { -1.0 };
            for _ in 0..sps {
                nrz.push(val);
            }
        }

        // Apply Gaussian filter
        let filtered = self.apply_gaussian_filter(&nrz);

        // Phase modulation
        let mut output = Vec::with_capacity(filtered.len());
        for &freq_val in &filtered {
            output.push(MskComplex::from_polar(1.0, self.msk.phase));
            self.msk.phase += freq_val * self.msk.delta_phase_per_sample;
        }
        self.msk.phase = wrap_phase(self.msk.phase);

        output
    }

    /// Apply the Gaussian pre-filter.
    fn apply_gaussian_filter(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            self.filter_state.insert(0, x);
            self.filter_state.truncate(self.gaussian_taps.len());

            let mut sum = 0.0;
            for (i, &tap) in self.gaussian_taps.iter().enumerate() {
                if i < self.filter_state.len() {
                    sum += tap * self.filter_state[i];
                }
            }
            output.push(sum);
        }
        output
    }

    /// BT product.
    pub fn bt(&self) -> f64 {
        self.bt
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.msk.reset();
        self.filter_state.fill(0.0);
    }
}

/// Design a Gaussian filter for GMSK.
fn design_gaussian_filter(bt: f64, sps: usize, span: usize) -> Vec<f64> {
    let len = span * sps + 1;
    let mut taps = Vec::with_capacity(len);
    let sigma = (2.0 * PI * bt).recip() * (2.0_f64 * (2.0_f64).ln()).sqrt();

    for i in 0..len {
        let t = (i as f64 - (len - 1) as f64 / 2.0) / sps as f64;
        let val = (-t * t / (2.0 * sigma * sigma)).exp();
        taps.push(val);
    }

    // Normalize
    let sum: f64 = taps.iter().sum();
    if sum > 0.0 {
        for tap in &mut taps {
            *tap /= sum;
        }
    }

    taps
}

fn wrap_phase(mut phase: f64) -> f64 {
    while phase > PI {
        phase -= 2.0 * PI;
    }
    while phase < -PI {
        phase += 2.0 * PI;
    }
    phase
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_msk_modulation_length() {
        let mut msk = MskModulator::new(1000.0, 8000.0);
        let bits = vec![true, false, true];
        let signal = msk.modulate(&bits);
        assert_eq!(signal.len(), 3 * 8);
    }

    #[test]
    fn test_constant_envelope() {
        let mut msk = MskModulator::new(1000.0, 8000.0);
        let bits = vec![true, false, true, true, false, false, true];
        let signal = msk.modulate(&bits);

        for (i, s) in signal.iter().enumerate() {
            let mag = s.mag();
            assert!((mag - 1.0).abs() < 1e-10,
                "Sample {} magnitude {:.6} != 1.0", i, mag);
        }
    }

    #[test]
    fn test_modulation_index() {
        let msk = MskModulator::new(1000.0, 8000.0);
        assert!((msk.modulation_index() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_frequency_deviation() {
        let msk = MskModulator::new(1000.0, 8000.0);
        assert!((msk.frequency_deviation() - 250.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_continuity() {
        let mut msk = MskModulator::new(1000.0, 8000.0);
        let bits = vec![true, false, true, false];
        let signal = msk.modulate(&bits);

        // Check phase doesn't jump by more than expected per sample
        let max_phase_step = 2.0 * PI * 250.0 / 8000.0 * 1.5; // with margin
        for i in 1..signal.len() {
            let phase_prev = signal[i - 1].im.atan2(signal[i - 1].re);
            let phase_curr = signal[i].im.atan2(signal[i].re);
            let mut delta = (phase_curr - phase_prev).abs();
            if delta > PI {
                delta = 2.0 * PI - delta;
            }
            assert!(delta < max_phase_step,
                "Phase jump at sample {}: {:.4} rad", i, delta);
        }
    }

    #[test]
    fn test_demodulation_roundtrip() {
        let mut modulator = MskModulator::new(1000.0, 8000.0);
        let demodulator = MskDemodulator::new(1000.0, 8000.0);

        let bits = vec![true, false, true, true, false, true, false, false];
        let signal = modulator.modulate(&bits);
        let decoded = demodulator.demodulate(&signal);

        assert_eq!(decoded.len(), bits.len());
        for i in 0..decoded.len() {
            assert_eq!(decoded[i], bits[i],
                "Bit {} mismatch: got {}, expected {}", i, decoded[i], bits[i]);
        }
    }

    #[test]
    fn test_gmsk_modulation() {
        let mut gmsk = GmskModulator::new(1000.0, 8000.0, 0.3);
        let bits = vec![true, false, true, true, false];
        let signal = gmsk.modulate(&bits);
        assert_eq!(signal.len(), 5 * 8);

        // GMSK should also have near-constant envelope
        for s in &signal {
            let mag = s.mag();
            assert!(mag > 0.5 && mag < 1.5,
                "GMSK magnitude out of range: {:.4}", mag);
        }
    }

    #[test]
    fn test_gmsk_bt_product() {
        let gmsk = GmskModulator::new(1000.0, 8000.0, 0.3);
        assert!((gmsk.bt() - 0.3).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut msk = MskModulator::new(1000.0, 8000.0);
        let bits = vec![true, true, true];
        let _ = msk.modulate(&bits);
        assert!(msk.phase().abs() > 0.01); // Phase should have changed

        msk.reset();
        assert!((msk.phase()).abs() < 1e-10);
    }

    #[test]
    fn test_alternating_bits_spectrum() {
        let mut msk = MskModulator::new(1000.0, 8000.0);
        let bits: Vec<bool> = (0..100).map(|i| i % 2 == 0).collect();
        let signal = msk.modulate(&bits);

        // Alternating bits should produce a tone at ±Δf alternating
        // Just verify we get enough samples
        assert_eq!(signal.len(), 100 * 8);
    }
}
