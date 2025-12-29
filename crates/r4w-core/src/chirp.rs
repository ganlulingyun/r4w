//! Chirp Signal Generation
//!
//! This module implements the core chirp generation for LoRa's Chirp Spread
//! Spectrum (CSS) modulation.
//!
//! ## What is a Chirp?
//!
//! A chirp is a signal whose frequency changes linearly over time:
//!
//! ```text
//! Frequency
//!     ^
//! fmax|        ___/
//!     |     __/
//!     |  __/
//! fmin|_/
//!     +----------> Time
//!       Upchirp
//!
//! Frequency
//!     ^
//! fmax|\_
//!     |  \__
//!     |     \__
//! fmin|        \___
//!     +----------> Time
//!       Downchirp
//! ```
//!
//! ## Why Chirps?
//!
//! Chirp Spread Spectrum provides:
//! - **Noise immunity**: The signal energy is spread across the bandwidth
//! - **Multipath resistance**: Chirps can be distinguished even with reflections
//! - **Low complexity detection**: Simple FFT-based demodulation
//!
//! ## Mathematical Foundation
//!
//! The instantaneous frequency of a chirp is:
//!
//! ```text
//! f(t) = f_initial + (Δf/T) * t
//! ```
//!
//! Where:
//! - f_initial: Starting frequency
//! - Δf: Frequency sweep range (bandwidth)
//! - T: Chirp duration
//!
//! The phase is the integral of frequency:
//!
//! ```text
//! φ(t) = 2π * ∫f(t)dt = 2π * (f_initial*t + (Δf/2T)*t²)
//! ```
//!
//! And the signal is:
//!
//! ```text
//! s(t) = exp(j*φ(t))
//! ```

use std::f64::consts::PI;

use crate::params::LoRaParams;
use crate::types::{Complex, IQSample, Symbol};

/// Type of chirp (up or down)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChirpType {
    /// Frequency increases from -BW/2 to +BW/2
    Up,
    /// Frequency decreases from +BW/2 to -BW/2
    Down,
}

/// Generator for LoRa chirp signals
#[derive(Debug, Clone)]
pub struct ChirpGenerator {
    /// LoRa parameters
    params: LoRaParams,
    /// Pre-computed base upchirp for efficiency
    base_upchirp: Vec<IQSample>,
    /// Pre-computed base downchirp for efficiency
    base_downchirp: Vec<IQSample>,
}

impl ChirpGenerator {
    /// Create a new chirp generator with the given parameters
    pub fn new(params: LoRaParams) -> Self {
        let _n = params.samples_per_symbol();
        let base_upchirp = Self::generate_base_chirp_internal(&params, ChirpType::Up);
        let base_downchirp = Self::generate_base_chirp_internal(&params, ChirpType::Down);

        Self {
            params,
            base_upchirp,
            base_downchirp,
        }
    }

    /// Get the LoRa parameters
    pub fn params(&self) -> &LoRaParams {
        &self.params
    }

    /// Get the pre-computed base upchirp
    pub fn base_upchirp(&self) -> &[IQSample] {
        &self.base_upchirp
    }

    /// Get the pre-computed base downchirp
    pub fn base_downchirp(&self) -> &[IQSample] {
        &self.base_downchirp
    }

    /// Generate a base chirp (not shifted by any symbol)
    ///
    /// The base upchirp sweeps from -BW/2 to +BW/2.
    /// The base downchirp sweeps from +BW/2 to -BW/2.
    fn generate_base_chirp_internal(params: &LoRaParams, chirp_type: ChirpType) -> Vec<IQSample> {
        let n = params.samples_per_symbol();
        let bw = params.bw.hz();
        let ts = params.sample_duration();

        // Frequency sweep rate: Δf/T = BW / symbol_duration = BW² / chips
        let t_symbol = params.symbol_duration();
        let df = bw / t_symbol; // Hz per second

        // Initial frequency
        let f_init = match chirp_type {
            ChirpType::Up => -bw / 2.0,
            ChirpType::Down => bw / 2.0,
        };

        // Sign of frequency change
        let sign = match chirp_type {
            ChirpType::Up => 1.0,
            ChirpType::Down => -1.0,
        };

        let mut chirp = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 * ts;
            // Phase = 2π * (f_init * t + sign * df/2 * t²)
            let phase = 2.0 * PI * (f_init * t + sign * df / 2.0 * t * t);
            chirp.push(Complex::new(phase.cos(), phase.sin()));
        }

        chirp
    }

    /// Generate a chirp modulated with a symbol value
    ///
    /// The symbol value (0 to 2^SF - 1) determines the cyclic shift of the chirp.
    /// Symbol 0 produces the base upchirp. Higher symbols shift the frequency
    /// wrap-around point earlier in time.
    ///
    /// ```text
    /// Symbol 0:      Symbol 64 (SF7):
    /// f_max    /     f_max   /
    ///      ___/           __/
    ///   __/            __/
    /// _/           ___/
    ///              wrap
    /// ```
    pub fn generate_symbol_chirp(&self, symbol: Symbol) -> Vec<IQSample> {
        let n = self.params.samples_per_symbol();
        let _k = self.params.chips_per_symbol();
        let bw = self.params.bw.hz();
        let ts = self.params.sample_duration();
        let osf = self.params.oversample;

        // Symbol determines the cyclic shift in chips
        let shift_chips = symbol as usize;
        let shift_samples = shift_chips * osf;

        // Generate the shifted chirp
        let t_symbol = self.params.symbol_duration();
        let df = bw / t_symbol;
        let f_init = -bw / 2.0;

        let mut chirp = Vec::with_capacity(n);

        for i in 0..n {
            let t = i as f64 * ts;

            // Calculate the effective time with cyclic shift
            let t_shift = shift_samples as f64 * ts;

            // Calculate instantaneous frequency with wrap-around
            let _freq = f_init + df * ((t + t_shift) % t_symbol);

            // But we need to handle the phase continuously
            // This is done by computing phase directly with the wrap
            let t_eff = (t + t_shift) % t_symbol;
            let phase = 2.0 * PI * (f_init * t_eff + df / 2.0 * t_eff * t_eff);

            // Account for phase discontinuity at wrap point
            let wrap_count = ((t + t_shift) / t_symbol).floor() as i32;
            let phase_offset = wrap_count as f64 * 2.0 * PI * (f_init * t_symbol + df / 2.0 * t_symbol * t_symbol);

            let total_phase = phase + phase_offset;
            chirp.push(Complex::new(total_phase.cos(), total_phase.sin()));
        }

        chirp
    }

    /// Generate a symbol chirp using cyclic rotation of the base chirp
    ///
    /// This is more efficient than generating from scratch when we have
    /// the base chirp pre-computed. The symbol value causes a cyclic
    /// rotation with appropriate phase correction.
    pub fn generate_symbol_chirp_fast(&self, symbol: Symbol) -> Vec<IQSample> {
        let n = self.params.samples_per_symbol();
        let k = self.params.chips_per_symbol();
        let osf = self.params.oversample;

        // Symbol determines cyclic shift
        let shift_samples = (symbol as usize * osf) % n;

        if shift_samples == 0 {
            return self.base_upchirp.clone();
        }

        let mut chirp = Vec::with_capacity(n);

        // Phase correction for the wrap-around
        // When we wrap from the end to the beginning, we need to adjust phase
        let bw = self.params.bw.hz();
        let _ts = self.params.sample_duration();
        let t_symbol = self.params.symbol_duration();

        // Phase jump at wrap point
        let phase_correction = 2.0 * PI * bw * (symbol as f64 / k as f64) * t_symbol;
        let correction = Complex::new(phase_correction.cos(), phase_correction.sin());

        // Rotate the base chirp
        for i in 0..n {
            let src_idx = (i + shift_samples) % n;
            let sample = if i + shift_samples >= n {
                // After wrap-around, apply phase correction
                self.base_upchirp[src_idx] * correction
            } else {
                self.base_upchirp[src_idx]
            };
            chirp.push(sample);
        }

        chirp
    }

    /// Generate a raw chirp with explicit parameters
    ///
    /// This is useful for educational purposes to understand chirp generation.
    ///
    /// # Arguments
    /// * `f_init` - Initial frequency in Hz
    /// * `df` - Frequency rate of change in Hz/s
    /// * `n_samples` - Number of samples to generate
    /// * `sample_rate` - Sample rate in Hz
    /// * `phi_init` - Initial phase in radians
    ///
    /// # Returns
    /// Tuple of (samples, final_phase)
    pub fn generate_raw_chirp(
        f_init: f64,
        df: f64,
        n_samples: usize,
        sample_rate: f64,
        phi_init: f64,
    ) -> (Vec<IQSample>, f64) {
        let ts = 1.0 / sample_rate;
        let mut samples = Vec::with_capacity(n_samples);

        for i in 0..n_samples {
            let t = i as f64 * ts;
            let phase = phi_init + 2.0 * PI * (f_init * t + df / 2.0 * t * t);
            samples.push(Complex::new(phase.cos(), phase.sin()));
        }

        let t_final = n_samples as f64 * ts;
        let phi_final = phi_init + 2.0 * PI * (f_init * t_final + df / 2.0 * t_final * t_final);

        (samples, phi_final)
    }

    /// Generate the LoRa preamble
    ///
    /// The preamble consists of:
    /// 1. N base upchirps (default 8)
    /// 2. Two sync word chirps (shifted by sync word values)
    /// 3. Two full downchirps
    /// 4. One quarter downchirp (2.25 downchirps total)
    pub fn generate_preamble(&self) -> Vec<IQSample> {
        let n = self.params.samples_per_symbol();
        let k = self.params.chips_per_symbol() as u16;

        let mut preamble = Vec::with_capacity(n * 12); // Approximate size

        // Add preamble upchirps
        for _ in 0..self.params.preamble_length {
            preamble.extend_from_slice(&self.base_upchirp);
        }

        // Add sync word chirps
        // Standard sync word is 0x12 -> symbols (K-8) and (K-16)
        let sync1 = k.saturating_sub(8);
        let sync2 = k.saturating_sub(16);

        preamble.extend(self.generate_symbol_chirp_fast(sync1));
        preamble.extend(self.generate_symbol_chirp_fast(sync2));

        // Add 2.25 downchirps
        preamble.extend_from_slice(&self.base_downchirp);
        preamble.extend_from_slice(&self.base_downchirp);

        // Quarter downchirp
        let quarter_len = n / 4;
        preamble.extend_from_slice(&self.base_downchirp[..quarter_len]);

        preamble
    }

    /// Compute the instantaneous frequency at each sample point
    ///
    /// This is useful for visualization to show the chirp frequency sweep.
    pub fn compute_instantaneous_frequency(&self, samples: &[IQSample]) -> Vec<f64> {
        if samples.len() < 2 {
            return vec![];
        }

        let sample_rate = self.params.sample_rate;
        let mut frequencies = Vec::with_capacity(samples.len() - 1);

        for i in 1..samples.len() {
            // Instantaneous frequency from phase derivative
            let phase_diff = (samples[i] * samples[i - 1].conj()).arg();
            let freq = phase_diff * sample_rate / (2.0 * PI);
            frequencies.push(freq);
        }

        frequencies
    }
}

/// Educational structure showing chirp generation step by step
#[derive(Debug, Clone)]
pub struct ChirpExplanation {
    /// Time values for each sample
    pub time: Vec<f64>,
    /// Instantaneous frequency at each point
    pub frequency: Vec<f64>,
    /// Phase at each point (radians)
    pub phase: Vec<f64>,
    /// I (real) component
    pub i_component: Vec<f64>,
    /// Q (imaginary) component
    pub q_component: Vec<f64>,
    /// Complex samples
    pub samples: Vec<IQSample>,
}

impl ChirpExplanation {
    /// Generate a detailed explanation of chirp generation
    pub fn generate(
        f_init: f64,
        f_final: f64,
        duration: f64,
        sample_rate: f64,
    ) -> Self {
        let n_samples = (duration * sample_rate) as usize;
        let ts = 1.0 / sample_rate;
        let df = (f_final - f_init) / duration;

        let mut time = Vec::with_capacity(n_samples);
        let mut frequency = Vec::with_capacity(n_samples);
        let mut phase = Vec::with_capacity(n_samples);
        let mut i_component = Vec::with_capacity(n_samples);
        let mut q_component = Vec::with_capacity(n_samples);
        let mut samples = Vec::with_capacity(n_samples);

        for i in 0..n_samples {
            let t = i as f64 * ts;
            time.push(t);

            // Instantaneous frequency: f(t) = f_init + df * t
            let f_t = f_init + df * t;
            frequency.push(f_t);

            // Phase: integral of 2π*f(t)
            let phi = 2.0 * PI * (f_init * t + df / 2.0 * t * t);
            phase.push(phi);

            // I and Q components
            let i_val = phi.cos();
            let q_val = phi.sin();
            i_component.push(i_val);
            q_component.push(q_val);
            samples.push(Complex::new(i_val, q_val));
        }

        Self {
            time,
            frequency,
            phase,
            i_component,
            q_component,
            samples,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_base_chirp_length() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .oversample(1)
            .build();

        let gen = ChirpGenerator::new(params);
        assert_eq!(gen.base_upchirp().len(), 128); // 2^7 = 128
    }

    #[test]
    fn test_base_chirp_unit_magnitude() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        let gen = ChirpGenerator::new(params);

        // All samples should have magnitude close to 1
        for sample in gen.base_upchirp() {
            assert_relative_eq!(sample.norm(), 1.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_chirp_frequency_sweep() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .oversample(4)
            .build();

        let _bw = params.bw.hz();
        let gen = ChirpGenerator::new(params);
        let freqs = gen.compute_instantaneous_frequency(gen.base_upchirp());

        // First frequency should be near -BW/2, last near +BW/2
        assert!(freqs[0] < 0.0); // Starts negative
        assert!(freqs[freqs.len() - 1] > 0.0); // Ends positive
    }

    #[test]
    fn test_preamble_structure() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .preamble_length(8)
            .build();

        let gen = ChirpGenerator::new(params);
        let preamble = gen.generate_preamble();

        // Preamble should be: 8 upchirps + 2 sync + 2.25 downchirps
        // = 8 + 2 + 2.25 = 12.25 symbols
        let expected_len = (8 + 2 + 2) * 128 + 128 / 4;
        assert_eq!(preamble.len(), expected_len);
    }
}
