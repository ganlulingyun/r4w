//! Adaptive Equalizer
//!
//! Channel equalization using adaptive filtering algorithms. Compensates for
//! inter-symbol interference (ISI), multipath distortion, and channel fading.
//!
//! ## Algorithms
//!
//! - [`LmsEqualizer`] - Least Mean Squares: trained with known reference symbols
//! - [`CmaEqualizer`] - Constant Modulus Algorithm: blind equalization for PSK
//! - [`DdEqualizer`] - Decision-Directed: uses slicer decisions as reference
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::equalizer::{CmaEqualizer, CmaConfig};
//! use num_complex::Complex64;
//!
//! // Blind CMA equalizer for QPSK
//! let mut eq = CmaEqualizer::new(CmaConfig {
//!     num_taps: 11,
//!     step_size: 0.001,
//!     modulus: 1.0,  // target |y|^2 = 1 for QPSK
//!     ..Default::default()
//! });
//!
//! // Process received samples
//! let received = vec![Complex64::new(0.7, 0.3); 100];
//! let equalized = eq.process_block(&received);
//! ```

use num_complex::Complex64;

// --- LMS Equalizer ---

/// Configuration for the LMS equalizer.
#[derive(Debug, Clone)]
pub struct LmsConfig {
    /// Number of filter taps (default: 11)
    pub num_taps: usize,
    /// Step size (mu) for weight updates (default: 0.01)
    pub step_size: f64,
    /// Leakage factor for weight decay (0.0 = none, default: 0.0)
    pub leakage: f64,
}

impl Default for LmsConfig {
    fn default() -> Self {
        Self {
            num_taps: 11,
            step_size: 0.01,
            leakage: 0.0,
        }
    }
}

/// LMS (Least Mean Squares) adaptive equalizer.
///
/// Updates filter taps to minimize mean squared error between output and a
/// reference signal. Requires training symbols (known data) for convergence.
///
/// Weight update: `w[n+1] = (1-mu*lambda)*w[n] + mu * conj(error) * x[n]`
///
/// This is equivalent to GNU Radio's `lms_dd_equalizer_cc` in training mode.
#[derive(Debug, Clone)]
pub struct LmsEqualizer {
    config: LmsConfig,
    taps: Vec<Complex64>,
    delay_line: Vec<Complex64>,
    delay_idx: usize,
}

impl LmsEqualizer {
    /// Create a new LMS equalizer.
    pub fn new(config: LmsConfig) -> Self {
        let n = config.num_taps;
        // Center tap initialization (identity filter)
        let mut taps = vec![Complex64::new(0.0, 0.0); n];
        taps[n / 2] = Complex64::new(1.0, 0.0);
        Self {
            config,
            taps,
            delay_line: vec![Complex64::new(0.0, 0.0); n],
            delay_idx: 0,
        }
    }

    /// Get the current filter taps.
    pub fn taps(&self) -> &[Complex64] {
        &self.taps
    }

    /// Process a single sample with a known reference for training.
    pub fn process_with_reference(&mut self, input: Complex64, reference: Complex64) -> Complex64 {
        // Insert into delay line
        self.delay_line[self.delay_idx] = input;

        // Compute output: y = w^H * x (inner product)
        let output = self.compute_output();

        // Compute error
        let error = reference - output;

        // Update weights: w += mu * conj(error) * x
        self.update_weights(error);

        // Advance delay line
        self.delay_idx = (self.delay_idx + 1) % self.config.num_taps;

        output
    }

    /// Process a single sample without training (use current taps).
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        self.delay_line[self.delay_idx] = input;
        let output = self.compute_output();
        self.delay_idx = (self.delay_idx + 1) % self.config.num_taps;
        output
    }

    /// Process a block with reference symbols (training mode).
    pub fn train(&mut self, input: &[Complex64], reference: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .zip(reference.iter())
            .map(|(&x, &r)| self.process_with_reference(x, r))
            .collect()
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    fn compute_output(&self) -> Complex64 {
        let n = self.config.num_taps;
        let mut output = Complex64::new(0.0, 0.0);
        for i in 0..n {
            let idx = (self.delay_idx + n - i) % n;
            output += self.taps[i].conj() * self.delay_line[idx];
        }
        output
    }

    fn update_weights(&mut self, error: Complex64) {
        let n = self.config.num_taps;
        let mu = self.step_size();
        let leak = 1.0 - mu * self.config.leakage;
        for i in 0..n {
            let idx = (self.delay_idx + n - i) % n;
            self.taps[i] = self.taps[i] * leak + self.delay_line[idx] * error.conj() * mu;
        }
    }

    fn step_size(&self) -> f64 {
        self.config.step_size
    }

    /// Reset equalizer to initial state.
    pub fn reset(&mut self) {
        let n = self.config.num_taps;
        self.taps = vec![Complex64::new(0.0, 0.0); n];
        self.taps[n / 2] = Complex64::new(1.0, 0.0);
        self.delay_line = vec![Complex64::new(0.0, 0.0); n];
        self.delay_idx = 0;
    }
}

// --- CMA Equalizer ---

/// Configuration for the CMA equalizer.
#[derive(Debug, Clone)]
pub struct CmaConfig {
    /// Number of filter taps (default: 11)
    pub num_taps: usize,
    /// Step size (mu) for weight updates (default: 0.001)
    pub step_size: f64,
    /// Target modulus R (default: 1.0 for unit-amplitude PSK)
    /// For QPSK: R = E[|a|^4] / E[|a|^2] = 1.0
    pub modulus: f64,
}

impl Default for CmaConfig {
    fn default() -> Self {
        Self {
            num_taps: 11,
            step_size: 0.001,
            modulus: 1.0,
        }
    }
}

/// CMA (Constant Modulus Algorithm) blind equalizer.
///
/// Works without training symbols by exploiting the constant envelope
/// property of PSK signals. Minimizes: `E[(|y|^2 - R)^2]`
///
/// Weight update: `w[n+1] = w[n] + mu * y * (R - |y|^2) * conj(x[n])`
///
/// Best for: BPSK, QPSK, 8PSK, and other constant-envelope modulations.
///
/// This is equivalent to GNU Radio's `cma_equalizer_cc`.
#[derive(Debug, Clone)]
pub struct CmaEqualizer {
    config: CmaConfig,
    taps: Vec<Complex64>,
    delay_line: Vec<Complex64>,
    delay_idx: usize,
}

impl CmaEqualizer {
    /// Create a new CMA equalizer.
    pub fn new(config: CmaConfig) -> Self {
        let n = config.num_taps;
        let mut taps = vec![Complex64::new(0.0, 0.0); n];
        taps[n / 2] = Complex64::new(1.0, 0.0);
        Self {
            config,
            taps,
            delay_line: vec![Complex64::new(0.0, 0.0); n],
            delay_idx: 0,
        }
    }

    /// Get the current filter taps.
    pub fn taps(&self) -> &[Complex64] {
        &self.taps
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        self.delay_line[self.delay_idx] = input;

        // Compute output
        let output = self.compute_output();

        // CMA error: e = y * (R - |y|^2)
        let y_mag_sq = output.norm_sqr();
        let error = output * (self.config.modulus - y_mag_sq);

        // Update weights: w += mu * error * conj(x)
        let n = self.config.num_taps;
        let mu = self.config.step_size;
        for i in 0..n {
            let idx = (self.delay_idx + n - i) % n;
            self.taps[i] += self.delay_line[idx].conj() * error * mu;
        }

        self.delay_idx = (self.delay_idx + 1) % n;
        output
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    fn compute_output(&self) -> Complex64 {
        let n = self.config.num_taps;
        let mut output = Complex64::new(0.0, 0.0);
        for i in 0..n {
            let idx = (self.delay_idx + n - i) % n;
            output += self.taps[i] * self.delay_line[idx];
        }
        output
    }

    /// Reset equalizer to initial state.
    pub fn reset(&mut self) {
        let n = self.config.num_taps;
        self.taps = vec![Complex64::new(0.0, 0.0); n];
        self.taps[n / 2] = Complex64::new(1.0, 0.0);
        self.delay_line = vec![Complex64::new(0.0, 0.0); n];
        self.delay_idx = 0;
    }
}

// --- Decision-Directed Equalizer ---

/// Constellation decision slicer for decision-directed equalization.
#[derive(Debug, Clone)]
pub enum Constellation {
    /// BPSK: {-1, +1}
    Bpsk,
    /// QPSK: {±1/√2 ± j/√2}
    Qpsk,
    /// 8PSK: unit circle, 8 points
    Psk8,
    /// Custom constellation points
    Custom(Vec<Complex64>),
}

impl Constellation {
    /// Slice an input sample to the nearest constellation point.
    pub fn decide(&self, sample: Complex64) -> Complex64 {
        match self {
            Constellation::Bpsk => {
                if sample.re >= 0.0 {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(-1.0, 0.0)
                }
            }
            Constellation::Qpsk => {
                let v = std::f64::consts::FRAC_1_SQRT_2;
                Complex64::new(
                    if sample.re >= 0.0 { v } else { -v },
                    if sample.im >= 0.0 { v } else { -v },
                )
            }
            Constellation::Psk8 => {
                let angle = sample.im.atan2(sample.re);
                let sector = (angle * 4.0 / std::f64::consts::PI).round();
                let snapped = sector * std::f64::consts::PI / 4.0;
                Complex64::new(snapped.cos(), snapped.sin())
            }
            Constellation::Custom(points) => {
                let mut best = points[0];
                let mut best_dist = (sample - best).norm_sqr();
                for &p in &points[1..] {
                    let d = (sample - p).norm_sqr();
                    if d < best_dist {
                        best = p;
                        best_dist = d;
                    }
                }
                best
            }
        }
    }
}

/// Configuration for the decision-directed equalizer.
#[derive(Debug, Clone)]
pub struct DdConfig {
    /// Number of filter taps (default: 11)
    pub num_taps: usize,
    /// Step size for weight updates (default: 0.005)
    pub step_size: f64,
    /// Constellation for decision feedback
    pub constellation: Constellation,
}

impl Default for DdConfig {
    fn default() -> Self {
        Self {
            num_taps: 11,
            step_size: 0.005,
            constellation: Constellation::Qpsk,
        }
    }
}

/// Decision-Directed (DD) adaptive equalizer.
///
/// Combines CMA-like blind operation with LMS-like convergence by using
/// hard slicer decisions as the reference signal. Best used after initial
/// convergence with CMA.
///
/// Error: `e = decide(y) - y`
/// Weight update: `w[n+1] = w[n] + mu * conj(error) * x[n]`
///
/// This is equivalent to GNU Radio's `lms_dd_equalizer_cc`.
#[derive(Debug, Clone)]
pub struct DdEqualizer {
    config: DdConfig,
    taps: Vec<Complex64>,
    delay_line: Vec<Complex64>,
    delay_idx: usize,
}

impl DdEqualizer {
    /// Create a new decision-directed equalizer.
    pub fn new(config: DdConfig) -> Self {
        let n = config.num_taps;
        let mut taps = vec![Complex64::new(0.0, 0.0); n];
        taps[n / 2] = Complex64::new(1.0, 0.0);
        Self {
            config,
            taps,
            delay_line: vec![Complex64::new(0.0, 0.0); n],
            delay_idx: 0,
        }
    }

    /// Get the current filter taps.
    pub fn taps(&self) -> &[Complex64] {
        &self.taps
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        self.delay_line[self.delay_idx] = input;

        // Compute output
        let output = self.compute_output();

        // Decision-directed error
        let decision = self.config.constellation.decide(output);
        let error = decision - output;

        // Update weights
        let n = self.config.num_taps;
        let mu = self.config.step_size;
        for i in 0..n {
            let idx = (self.delay_idx + n - i) % n;
            self.taps[i] += self.delay_line[idx] * error.conj() * mu;
        }

        self.delay_idx = (self.delay_idx + 1) % n;
        output
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    fn compute_output(&self) -> Complex64 {
        let n = self.config.num_taps;
        let mut output = Complex64::new(0.0, 0.0);
        for i in 0..n {
            let idx = (self.delay_idx + n - i) % n;
            output += self.taps[i] * self.delay_line[idx];
        }
        output
    }

    /// Reset equalizer to initial state.
    pub fn reset(&mut self) {
        let n = self.config.num_taps;
        self.taps = vec![Complex64::new(0.0, 0.0); n];
        self.taps[n / 2] = Complex64::new(1.0, 0.0);
        self.delay_line = vec![Complex64::new(0.0, 0.0); n];
        self.delay_idx = 0;
    }
}

// Implement Filter trait for all equalizer variants
impl crate::filters::Filter for LmsEqualizer {
    fn process(&mut self, input: Complex64) -> Complex64 {
        LmsEqualizer::process(self, input)
    }

    fn reset(&mut self) {
        LmsEqualizer::reset(self);
    }

    fn group_delay(&self) -> f64 {
        (self.config.num_taps / 2) as f64
    }

    fn order(&self) -> usize {
        self.config.num_taps - 1
    }
}

impl crate::filters::Filter for CmaEqualizer {
    fn process(&mut self, input: Complex64) -> Complex64 {
        CmaEqualizer::process(self, input)
    }

    fn reset(&mut self) {
        CmaEqualizer::reset(self);
    }

    fn group_delay(&self) -> f64 {
        (self.config.num_taps / 2) as f64
    }

    fn order(&self) -> usize {
        self.config.num_taps - 1
    }
}

impl crate::filters::Filter for DdEqualizer {
    fn process(&mut self, input: Complex64) -> Complex64 {
        DdEqualizer::process(self, input)
    }

    fn reset(&mut self) {
        DdEqualizer::reset(self);
    }

    fn group_delay(&self) -> f64 {
        (self.config.num_taps / 2) as f64
    }

    fn order(&self) -> usize {
        self.config.num_taps - 1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lms_identity_channel() {
        // No distortion — output should equal input
        let mut eq = LmsEqualizer::new(LmsConfig {
            num_taps: 5,
            step_size: 0.1,
            leakage: 0.0,
        });

        let reference: Vec<Complex64> = (0..200)
            .map(|i| {
                if i % 2 == 0 {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(-1.0, 0.0)
                }
            })
            .collect();

        let output = eq.train(&reference, &reference);
        // After training, output should closely match reference
        let last = output.last().unwrap();
        let ref_last = reference.last().unwrap();
        assert!(
            (last - ref_last).norm() < 0.3,
            "LMS should track identity channel: got {last}"
        );
    }

    #[test]
    fn test_lms_simple_isi_channel() {
        // Channel: h = [1.0, 0.5] introduces ISI
        let symbols: Vec<Complex64> = (0..500)
            .map(|i| {
                if (i / 1) % 2 == 0 {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(-1.0, 0.0)
                }
            })
            .collect();

        // Apply channel distortion
        let mut received = vec![Complex64::new(0.0, 0.0); symbols.len()];
        for i in 0..symbols.len() {
            received[i] += symbols[i] * 1.0;
            if i + 1 < symbols.len() {
                received[i + 1] += symbols[i] * 0.5;
            }
        }

        let mut eq = LmsEqualizer::new(LmsConfig {
            num_taps: 11,
            step_size: 0.01,
            leakage: 0.0,
        });

        let output = eq.train(&received, &symbols);

        // After training, last samples should be close to reference
        let last_20: Vec<f64> = output[480..]
            .iter()
            .zip(symbols[480..].iter())
            .map(|(o, r)| (o - r).norm())
            .collect();
        let avg_error: f64 = last_20.iter().sum::<f64>() / last_20.len() as f64;
        assert!(
            avg_error < 0.5,
            "LMS should equalize ISI channel: avg error = {avg_error}"
        );
    }

    #[test]
    fn test_cma_unit_modulus() {
        let mut eq = CmaEqualizer::new(CmaConfig {
            num_taps: 7,
            step_size: 0.005,
            modulus: 1.0,
        });

        // QPSK signal (constant modulus)
        let v = std::f64::consts::FRAC_1_SQRT_2;
        let qpsk = [
            Complex64::new(v, v),
            Complex64::new(-v, v),
            Complex64::new(-v, -v),
            Complex64::new(v, -v),
        ];

        let input: Vec<Complex64> = (0..2000).map(|i| qpsk[i % 4]).collect();
        let output = eq.process_block(&input);

        // After convergence, output should have unit modulus
        let last_mag = output.last().unwrap().norm();
        assert!(
            (last_mag - 1.0).abs() < 0.3,
            "CMA should converge to unit modulus: got {last_mag}"
        );
    }

    #[test]
    fn test_cma_gain_correction() {
        let mut eq = CmaEqualizer::new(CmaConfig {
            num_taps: 7,
            step_size: 0.005,
            modulus: 1.0,
        });

        // Attenuated BPSK signal
        let input: Vec<Complex64> = (0..3000)
            .map(|i| {
                if i % 2 == 0 {
                    Complex64::new(0.3, 0.0)
                } else {
                    Complex64::new(-0.3, 0.0)
                }
            })
            .collect();

        let output = eq.process_block(&input);

        // CMA should restore the amplitude closer to modulus
        let last_mag = output.last().unwrap().norm();
        assert!(
            last_mag > 0.5,
            "CMA should boost attenuated signal: got {last_mag}"
        );
    }

    #[test]
    fn test_dd_equalizer_qpsk() {
        let mut eq = DdEqualizer::new(DdConfig {
            num_taps: 7,
            step_size: 0.01,
            constellation: Constellation::Qpsk,
        });

        let v = std::f64::consts::FRAC_1_SQRT_2;
        let qpsk = [
            Complex64::new(v, v),
            Complex64::new(-v, v),
            Complex64::new(-v, -v),
            Complex64::new(v, -v),
        ];

        let input: Vec<Complex64> = (0..1000).map(|i| qpsk[i % 4]).collect();
        let output = eq.process_block(&input);

        // After convergence, output should be near constellation points
        let last = output.last().unwrap();
        let decision = Constellation::Qpsk.decide(*last);
        assert!(
            (last - decision).norm() < 0.5,
            "DD should converge to constellation: got {last}"
        );
    }

    #[test]
    fn test_constellation_bpsk_slicer() {
        let c = Constellation::Bpsk;
        assert_eq!(c.decide(Complex64::new(0.5, 0.3)), Complex64::new(1.0, 0.0));
        assert_eq!(
            c.decide(Complex64::new(-0.1, -0.9)),
            Complex64::new(-1.0, 0.0)
        );
    }

    #[test]
    fn test_constellation_qpsk_slicer() {
        let c = Constellation::Qpsk;
        let v = std::f64::consts::FRAC_1_SQRT_2;

        let d = c.decide(Complex64::new(0.5, 0.8));
        assert!((d.re - v).abs() < 1e-10);
        assert!((d.im - v).abs() < 1e-10);

        let d = c.decide(Complex64::new(-0.3, -0.1));
        assert!((d.re - (-v)).abs() < 1e-10);
        assert!((d.im - (-v)).abs() < 1e-10);
    }

    #[test]
    fn test_equalizer_reset() {
        let mut eq = CmaEqualizer::new(CmaConfig::default());
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let _ = eq.process_block(&input);

        eq.reset();
        let taps = eq.taps();
        let center = taps.len() / 2;
        assert!((taps[center].re - 1.0).abs() < 1e-10);
        for (i, t) in taps.iter().enumerate() {
            if i != center {
                assert!(t.norm() < 1e-10, "Non-center tap should be zero after reset");
            }
        }
    }

    #[test]
    fn test_equalizer_filter_trait() {
        use crate::filters::Filter;
        let mut eq = CmaEqualizer::new(CmaConfig::default());
        // Push enough samples to fill the delay line past the center tap
        for _ in 0..6 {
            Filter::process(&mut eq, Complex64::new(1.0, 0.0));
        }
        let output = Filter::process(&mut eq, Complex64::new(1.0, 0.0));
        assert!(output.norm() > 0.0, "Output after filling delay line should be nonzero");
        assert_eq!(eq.group_delay(), 5.0); // 11/2 = 5
    }

    #[test]
    fn test_custom_constellation() {
        let points = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(0.0, -1.0),
        ];
        let c = Constellation::Custom(points);

        let d = c.decide(Complex64::new(0.9, 0.2));
        assert!((d - Complex64::new(1.0, 0.0)).norm() < 1e-10);

        let d = c.decide(Complex64::new(-0.1, 0.8));
        assert!((d - Complex64::new(0.0, 1.0)).norm() < 1e-10);
    }
}
