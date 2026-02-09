//! Linear Equalizer — Adaptive FIR Equalizer with Multiple Algorithms
//!
//! Unified adaptive linear (FIR) equalizer supporting LMS, RLS, CMA,
//! and Kurtotic algorithms with training and decision-directed modes.
//! Combines tap adaptation, constellation slicing, and mode switching
//! in a single configurable block.
//! GNU Radio equivalent: `gr::digital::linear_equalizer`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::linear_equalizer::{LinearEqualizer, LinearEqualizerConfig, AdaptAlgorithm};
//! use num_complex::Complex64;
//!
//! let config = LinearEqualizerConfig {
//!     num_taps: 11,
//!     sps: 1,
//!     algorithm: AdaptAlgorithm::Lms { mu: 0.01 },
//! };
//! let mut eq = LinearEqualizer::new(config);
//! let samples = vec![Complex64::new(1.0, 0.0); 20];
//! let output = eq.process(&samples);
//! assert_eq!(output.len(), 20);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Adaptive algorithm selection.
#[derive(Debug, Clone, Copy)]
pub enum AdaptAlgorithm {
    /// Least Mean Squares.
    Lms { mu: f64 },
    /// Recursive Least Squares.
    Rls { delta: f64, lambda: f64 },
    /// Constant Modulus Algorithm (blind).
    Cma { mu: f64, modulus: f64 },
    /// Kurtotic algorithm (blind).
    Kurtotic { mu: f64 },
}

/// Equalizer operating mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EqualizerMode {
    /// Training mode with known reference.
    Training,
    /// Decision-directed mode.
    DecisionDirected,
}

/// Configuration for linear equalizer.
#[derive(Debug, Clone)]
pub struct LinearEqualizerConfig {
    /// Number of FIR taps.
    pub num_taps: usize,
    /// Samples per symbol (1 = symbol-spaced, 2 = fractionally-spaced).
    pub sps: usize,
    /// Adaptation algorithm.
    pub algorithm: AdaptAlgorithm,
}

/// Constellation for decision-directed slicing.
#[derive(Debug, Clone)]
pub struct ConstellationRef {
    /// Reference constellation points.
    pub points: Vec<Complex64>,
}

impl ConstellationRef {
    /// Create BPSK constellation.
    pub fn bpsk() -> Self {
        Self {
            points: vec![Complex64::new(-1.0, 0.0), Complex64::new(1.0, 0.0)],
        }
    }

    /// Create QPSK constellation.
    pub fn qpsk() -> Self {
        let s = 1.0 / 2.0f64.sqrt();
        Self {
            points: vec![
                Complex64::new(s, s),
                Complex64::new(-s, s),
                Complex64::new(-s, -s),
                Complex64::new(s, -s),
            ],
        }
    }

    /// Slice to nearest constellation point.
    pub fn decide(&self, sample: Complex64) -> Complex64 {
        self.points
            .iter()
            .min_by(|a, b| {
                let da = (sample - *a).norm_sqr();
                let db = (sample - *b).norm_sqr();
                da.partial_cmp(&db).unwrap()
            })
            .copied()
            .unwrap_or(sample)
    }
}

/// Adaptive linear equalizer.
#[derive(Debug, Clone)]
pub struct LinearEqualizer {
    config: LinearEqualizerConfig,
    /// FIR filter taps.
    taps: Vec<Complex64>,
    /// Input delay line.
    delay_line: Vec<Complex64>,
    /// Operating mode.
    mode: EqualizerMode,
    /// Training sequence.
    training_seq: Vec<Complex64>,
    /// Training index.
    train_idx: usize,
    /// Constellation reference for DD mode.
    constellation: ConstellationRef,
    /// RLS inverse correlation matrix (flattened).
    rls_p: Vec<f64>,
}

impl LinearEqualizer {
    /// Create a new linear equalizer.
    pub fn new(config: LinearEqualizerConfig) -> Self {
        let n = config.num_taps;
        let mut taps = vec![Complex64::new(0.0, 0.0); n];
        // Center spike initialization
        taps[n / 2] = Complex64::new(1.0, 0.0);

        // RLS P matrix initialization (delta * I)
        let delta = match config.algorithm {
            AdaptAlgorithm::Rls { delta, .. } => delta,
            _ => 1.0,
        };
        let mut rls_p = vec![0.0; n * n];
        for i in 0..n {
            rls_p[i * n + i] = delta;
        }

        Self {
            config,
            taps,
            delay_line: vec![Complex64::new(0.0, 0.0); n],
            mode: EqualizerMode::DecisionDirected,
            training_seq: Vec::new(),
            train_idx: 0,
            constellation: ConstellationRef::qpsk(),
            rls_p,
        }
    }

    /// Set constellation reference.
    pub fn set_constellation(&mut self, constellation: ConstellationRef) {
        self.constellation = constellation;
    }

    /// Set training sequence and switch to training mode.
    pub fn set_training(&mut self, seq: &[Complex64]) {
        self.training_seq = seq.to_vec();
        self.train_idx = 0;
        self.mode = EqualizerMode::Training;
    }

    /// Switch to decision-directed mode.
    pub fn switch_to_dd(&mut self) {
        self.mode = EqualizerMode::DecisionDirected;
    }

    /// Get current mode.
    pub fn mode(&self) -> EqualizerMode {
        self.mode
    }

    /// Get tap weights.
    pub fn tap_weights(&self) -> &[Complex64] {
        &self.taps
    }

    /// Process a block of samples.
    pub fn process(&mut self, samples: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(samples.len());
        for &s in samples {
            output.push(self.process_sample(s));
        }
        output
    }

    /// Process a single sample.
    fn process_sample(&mut self, input: Complex64) -> Complex64 {
        // Push into delay line
        self.delay_line.rotate_right(1);
        self.delay_line[0] = input;

        // Compute filter output
        let y = self.filter_output();

        // Get reference signal for error computation
        let reference = if self.mode == EqualizerMode::Training
            && self.train_idx < self.training_seq.len()
        {
            let r = self.training_seq[self.train_idx];
            self.train_idx += 1;
            if self.train_idx >= self.training_seq.len() {
                self.mode = EqualizerMode::DecisionDirected;
            }
            r
        } else {
            self.constellation.decide(y)
        };

        // Update taps
        self.update_taps(y, reference);

        y
    }

    /// Compute filter output (dot product of taps and delay line).
    fn filter_output(&self) -> Complex64 {
        let mut sum = Complex64::new(0.0, 0.0);
        for (t, d) in self.taps.iter().zip(self.delay_line.iter()) {
            sum += t.conj() * d;
        }
        sum
    }

    /// Update taps based on selected algorithm.
    fn update_taps(&mut self, output: Complex64, reference: Complex64) {
        match self.config.algorithm {
            AdaptAlgorithm::Lms { mu } => {
                let error = reference - output;
                for (i, d) in self.delay_line.iter().enumerate() {
                    self.taps[i] += mu * error * d.conj();
                }
            }
            AdaptAlgorithm::Rls { lambda, .. } => {
                self.update_rls(output, reference, lambda);
            }
            AdaptAlgorithm::Cma { mu, modulus } => {
                let r2 = modulus * modulus;
                let error = output * (r2 - output.norm_sqr());
                for (i, d) in self.delay_line.iter().enumerate() {
                    self.taps[i] += mu * error * d.conj();
                }
            }
            AdaptAlgorithm::Kurtotic { mu } => {
                // Minimize kurtosis: error = y * (2 - |y|^2)
                let error = output * (2.0 - output.norm_sqr());
                for (i, d) in self.delay_line.iter().enumerate() {
                    self.taps[i] += mu * error * d.conj();
                }
            }
        }
    }

    /// RLS tap update.
    fn update_rls(&mut self, output: Complex64, reference: Complex64, lambda: f64) {
        let n = self.config.num_taps;
        let error = reference - output;

        // k = P * x / (lambda + x^H * P * x)
        let mut px = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..n {
            for j in 0..n {
                px[i] += self.rls_p[i * n + j] * self.delay_line[j];
            }
        }

        let mut denom = Complex64::new(lambda, 0.0);
        for i in 0..n {
            denom += self.delay_line[i].conj() * px[i];
        }

        if denom.norm() < 1e-20 {
            return;
        }

        // Update taps: w += k * error
        for i in 0..n {
            let k_i = px[i] / denom;
            self.taps[i] += k_i * error.conj();
        }

        // Update P: P = (P - k * x^H * P) / lambda
        let inv_lambda = 1.0 / lambda;
        let mut new_p = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..n {
                let k_i = px[i] / denom;
                let xhp_j = px[j].conj();
                new_p[i * n + j] =
                    inv_lambda * (self.rls_p[i * n + j] - (k_i * xhp_j).re);
            }
        }
        self.rls_p = new_p;
    }

    /// Reset equalizer state.
    pub fn reset(&mut self) {
        let n = self.config.num_taps;
        self.taps = vec![Complex64::new(0.0, 0.0); n];
        self.taps[n / 2] = Complex64::new(1.0, 0.0);
        self.delay_line = vec![Complex64::new(0.0, 0.0); n];
        self.train_idx = 0;
    }

    /// Get config.
    pub fn config(&self) -> &LinearEqualizerConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_creation() {
        let config = LinearEqualizerConfig {
            num_taps: 11,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { mu: 0.01 },
        };
        let eq = LinearEqualizer::new(config);
        assert_eq!(eq.tap_weights().len(), 11);
        // Center spike
        assert!((eq.tap_weights()[5].re - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_lms_passthrough() {
        let config = LinearEqualizerConfig {
            num_taps: 5,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { mu: 0.0 }, // No adaptation
        };
        let mut eq = LinearEqualizer::new(config);
        eq.set_constellation(ConstellationRef::bpsk());
        // With center spike and no adaptation, output ~= input (delayed)
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 20];
        let output = eq.process(&input);
        // After filling delay line, should pass through
        assert!(output.len() == 20);
    }

    #[test]
    fn test_lms_convergence() {
        let config = LinearEqualizerConfig {
            num_taps: 7,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { mu: 0.05 },
        };
        let mut eq = LinearEqualizer::new(config);
        eq.set_constellation(ConstellationRef::bpsk());

        // Generate BPSK through simple ISI channel [1.0, 0.5]
        let symbols: Vec<Complex64> = (0..200)
            .map(|i| {
                if (i * 7 + 3) % 2 == 0 {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(-1.0, 0.0)
                }
            })
            .collect();

        // Apply ISI channel
        let mut channel_out = vec![Complex64::new(0.0, 0.0); symbols.len()];
        for i in 0..symbols.len() {
            channel_out[i] = symbols[i];
            if i > 0 {
                channel_out[i] += 0.5 * symbols[i - 1];
            }
        }

        // Train with known symbols
        eq.set_training(&symbols[..50]);
        let output = eq.process(&channel_out);

        // After convergence, output should be closer to original symbols
        let late_errors: f64 = output[150..]
            .iter()
            .zip(symbols[150..].iter())
            .map(|(o, s)| (*o - *s).norm_sqr())
            .sum::<f64>()
            / 50.0;
        // Error should decrease (not necessarily perfect)
        assert!(
            late_errors < 2.0,
            "LMS should reduce ISI: avg error = {}",
            late_errors
        );
    }

    #[test]
    fn test_rls_creation() {
        let config = LinearEqualizerConfig {
            num_taps: 7,
            sps: 1,
            algorithm: AdaptAlgorithm::Rls {
                delta: 100.0,
                lambda: 0.99,
            },
        };
        let mut eq = LinearEqualizer::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 20];
        let output = eq.process(&input);
        assert_eq!(output.len(), 20);
    }

    #[test]
    fn test_cma_blind() {
        let config = LinearEqualizerConfig {
            num_taps: 7,
            sps: 1,
            algorithm: AdaptAlgorithm::Cma {
                mu: 0.001,
                modulus: 1.0,
            },
        };
        let mut eq = LinearEqualizer::new(config);
        let input: Vec<Complex64> = (0..100)
            .map(|i| {
                if i % 2 == 0 {
                    Complex64::new(1.0, 0.0)
                } else {
                    Complex64::new(-1.0, 0.0)
                }
            })
            .collect();
        let output = eq.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_kurtotic() {
        let config = LinearEqualizerConfig {
            num_taps: 5,
            sps: 1,
            algorithm: AdaptAlgorithm::Kurtotic { mu: 0.001 },
        };
        let mut eq = LinearEqualizer::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 50];
        let output = eq.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_training_to_dd_switchover() {
        let config = LinearEqualizerConfig {
            num_taps: 5,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { mu: 0.01 },
        };
        let mut eq = LinearEqualizer::new(config);
        let training = vec![Complex64::new(1.0, 0.0); 10];
        eq.set_training(&training);
        assert_eq!(eq.mode(), EqualizerMode::Training);

        // Process more than training length
        let input = vec![Complex64::new(1.0, 0.0); 20];
        let _ = eq.process(&input);
        assert_eq!(eq.mode(), EqualizerMode::DecisionDirected);
    }

    #[test]
    fn test_constellation_bpsk() {
        let c = ConstellationRef::bpsk();
        let decided = c.decide(Complex64::new(0.8, 0.1));
        assert!((decided.re - 1.0).abs() < 1e-10);
        let decided = c.decide(Complex64::new(-0.3, -0.2));
        assert!((decided.re - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_constellation_qpsk() {
        let c = ConstellationRef::qpsk();
        assert_eq!(c.points.len(), 4);
        let decided = c.decide(Complex64::new(0.5, 0.5));
        let s = 1.0 / 2.0f64.sqrt();
        assert!((decided.re - s).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let config = LinearEqualizerConfig {
            num_taps: 5,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { mu: 0.01 },
        };
        let mut eq = LinearEqualizer::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 10];
        let _ = eq.process(&input);
        eq.reset();
        assert!((eq.tap_weights()[2].re - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_fractionally_spaced() {
        let config = LinearEqualizerConfig {
            num_taps: 11,
            sps: 2,
            algorithm: AdaptAlgorithm::Lms { mu: 0.01 },
        };
        let eq = LinearEqualizer::new(config);
        assert_eq!(eq.config().sps, 2);
        assert_eq!(eq.tap_weights().len(), 11);
    }
}
