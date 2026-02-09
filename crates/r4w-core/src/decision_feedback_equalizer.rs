//! Decision Feedback Equalizer — Non-linear adaptive equalizer
//!
//! DFE with separate forward and feedback FIR filters, superior to
//! linear equalizers for channels with deep spectral nulls and
//! severe multipath. Supports LMS and CMA adaptation. Training
//! mode with known sequence, then switches to decision-directed.
//! GNU Radio equivalent: `decision_feedback_equalizer`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::decision_feedback_equalizer::{DecisionFeedbackEqualizer, DfeConfig, AdaptAlgorithm};
//! use num_complex::Complex64;
//!
//! let config = DfeConfig {
//!     num_forward: 5,
//!     num_feedback: 3,
//!     sps: 1,
//!     algorithm: AdaptAlgorithm::Lms { step_size: 0.01 },
//! };
//! let mut dfe = DecisionFeedbackEqualizer::new(config);
//! let input = vec![Complex64::new(0.8, 0.1); 20];
//! let output = dfe.process(&input);
//! assert_eq!(output.len(), 20);
//! ```

use num_complex::Complex64;

/// Adaptive algorithm selection for DFE.
#[derive(Debug, Clone, Copy)]
pub enum AdaptAlgorithm {
    /// Least Mean Squares.
    Lms { step_size: f64 },
    /// Constant Modulus Algorithm (for PSK signals).
    Cma { modulus: f64, step_size: f64 },
    /// Recursive Least Squares.
    Rls { forgetting_factor: f64 },
}

/// DFE configuration.
#[derive(Debug, Clone)]
pub struct DfeConfig {
    /// Number of forward (feedforward) filter taps.
    pub num_forward: usize,
    /// Number of feedback filter taps.
    pub num_feedback: usize,
    /// Samples per symbol.
    pub sps: usize,
    /// Adaptive algorithm.
    pub algorithm: AdaptAlgorithm,
}

/// Decision function for making hard decisions on equalized symbols.
fn hard_decision_bpsk(sample: Complex64) -> Complex64 {
    Complex64::new(
        if sample.re >= 0.0 { 1.0 } else { -1.0 },
        0.0,
    )
}

fn hard_decision_qpsk(sample: Complex64) -> Complex64 {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    Complex64::new(
        if sample.re >= 0.0 { s } else { -s },
        if sample.im >= 0.0 { s } else { -s },
    )
}

/// Decision Feedback Equalizer.
#[derive(Debug, Clone)]
pub struct DecisionFeedbackEqualizer {
    /// Forward filter taps.
    forward_taps: Vec<Complex64>,
    /// Feedback filter taps.
    feedback_taps: Vec<Complex64>,
    /// Forward delay line.
    forward_buffer: Vec<Complex64>,
    /// Feedback delay line (past decisions).
    feedback_buffer: Vec<Complex64>,
    /// Adaptive algorithm.
    algorithm: AdaptAlgorithm,
    /// Samples per symbol.
    sps: usize,
    /// Sample counter for decimation.
    sample_count: usize,
    /// Number of symbols processed.
    symbols_processed: usize,
    /// Training mode active.
    training: bool,
    /// Training sequence (if provided).
    training_seq: Vec<Complex64>,
    /// Training index.
    training_idx: usize,
    /// Constellation order for decision (2=BPSK, 4=QPSK).
    constellation_order: usize,
}

impl DecisionFeedbackEqualizer {
    /// Create a new DFE.
    pub fn new(config: DfeConfig) -> Self {
        let num_fwd = config.num_forward.max(1);
        let num_fb = config.num_feedback.max(0);

        // Initialize forward taps: center tap = 1
        let mut forward_taps = vec![Complex64::new(0.0, 0.0); num_fwd];
        forward_taps[num_fwd / 2] = Complex64::new(1.0, 0.0);

        Self {
            forward_taps,
            feedback_taps: vec![Complex64::new(0.0, 0.0); num_fb],
            forward_buffer: vec![Complex64::new(0.0, 0.0); num_fwd],
            feedback_buffer: vec![Complex64::new(0.0, 0.0); num_fb],
            algorithm: config.algorithm,
            sps: config.sps.max(1),
            sample_count: 0,
            symbols_processed: 0,
            training: false,
            training_seq: Vec::new(),
            training_idx: 0,
            constellation_order: 2,
        }
    }

    /// Set constellation order (2=BPSK, 4=QPSK).
    pub fn set_constellation_order(&mut self, order: usize) {
        self.constellation_order = order;
    }

    /// Start training mode with a known sequence.
    pub fn start_training(&mut self, sequence: &[Complex64]) {
        self.training = true;
        self.training_seq = sequence.to_vec();
        self.training_idx = 0;
    }

    /// Stop training mode (switch to decision-directed).
    pub fn stop_training(&mut self) {
        self.training = false;
    }

    /// Make a hard decision on a symbol.
    fn decide(&self, sample: Complex64) -> Complex64 {
        match self.constellation_order {
            4 => hard_decision_qpsk(sample),
            _ => hard_decision_bpsk(sample),
        }
    }

    /// Process a single input sample, returning equalized output if at symbol boundary.
    fn process_sample(&mut self, input: Complex64) -> Option<Complex64> {
        // Shift forward buffer
        self.forward_buffer.rotate_right(1);
        self.forward_buffer[0] = input;

        self.sample_count += 1;
        if self.sample_count < self.sps {
            return None;
        }
        self.sample_count = 0;

        // Compute forward filter output
        let mut y = Complex64::new(0.0, 0.0);
        for (i, &tap) in self.forward_taps.iter().enumerate() {
            y += tap * self.forward_buffer[i];
        }

        // Add feedback filter output
        for (i, &tap) in self.feedback_taps.iter().enumerate() {
            y += tap * self.feedback_buffer[i];
        }

        // Determine desired symbol
        let desired = if self.training && self.training_idx < self.training_seq.len() {
            let d = self.training_seq[self.training_idx];
            self.training_idx += 1;
            if self.training_idx >= self.training_seq.len() {
                self.training = false;
            }
            d
        } else {
            self.decide(y)
        };

        // Compute error
        let error = desired - y;

        // Update taps
        self.update_taps(error, y, desired);

        // Shift feedback buffer with the decision
        if !self.feedback_buffer.is_empty() {
            self.feedback_buffer.rotate_right(1);
            self.feedback_buffer[0] = desired;
        }

        self.symbols_processed += 1;
        Some(y)
    }

    /// Update filter taps based on the chosen algorithm.
    fn update_taps(&mut self, error: Complex64, output: Complex64, _desired: Complex64) {
        match self.algorithm {
            AdaptAlgorithm::Lms { step_size } => {
                // LMS: w += mu * error * conj(x)
                for (i, tap) in self.forward_taps.iter_mut().enumerate() {
                    *tap += Complex64::new(step_size, 0.0) * error * self.forward_buffer[i].conj();
                }
                for (i, tap) in self.feedback_taps.iter_mut().enumerate() {
                    *tap += Complex64::new(step_size, 0.0) * error * self.feedback_buffer[i].conj();
                }
            }
            AdaptAlgorithm::Cma { modulus, step_size } => {
                // CMA: error = sample * (modulus - |sample|^2)
                let cma_error = output * Complex64::new(modulus - output.norm_sqr(), 0.0);
                for (i, tap) in self.forward_taps.iter_mut().enumerate() {
                    *tap +=
                        Complex64::new(step_size, 0.0) * cma_error * self.forward_buffer[i].conj();
                }
                for (i, tap) in self.feedback_taps.iter_mut().enumerate() {
                    *tap += Complex64::new(step_size, 0.0)
                        * cma_error
                        * self.feedback_buffer[i].conj();
                }
            }
            AdaptAlgorithm::Rls { forgetting_factor: _ } => {
                // Simplified RLS update (LMS fallback for now)
                let mu = 0.01;
                for (i, tap) in self.forward_taps.iter_mut().enumerate() {
                    *tap += Complex64::new(mu, 0.0) * error * self.forward_buffer[i].conj();
                }
                for (i, tap) in self.feedback_taps.iter_mut().enumerate() {
                    *tap += Complex64::new(mu, 0.0) * error * self.feedback_buffer[i].conj();
                }
            }
        }
    }

    /// Process a block of input samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::new();
        for &sample in input {
            if let Some(y) = self.process_sample(sample) {
                output.push(y);
            }
        }
        output
    }

    /// Get forward filter taps.
    pub fn forward_taps(&self) -> &[Complex64] {
        &self.forward_taps
    }

    /// Get feedback filter taps.
    pub fn feedback_taps(&self) -> &[Complex64] {
        &self.feedback_taps
    }

    /// Number of symbols processed.
    pub fn symbols_processed(&self) -> usize {
        self.symbols_processed
    }

    /// Check if in training mode.
    pub fn is_training(&self) -> bool {
        self.training
    }

    /// Reset the equalizer state.
    pub fn reset(&mut self) {
        let n_fwd = self.forward_taps.len();
        self.forward_taps = vec![Complex64::new(0.0, 0.0); n_fwd];
        self.forward_taps[n_fwd / 2] = Complex64::new(1.0, 0.0);

        let n_fb = self.feedback_taps.len();
        self.feedback_taps = vec![Complex64::new(0.0, 0.0); n_fb];

        self.forward_buffer.fill(Complex64::new(0.0, 0.0));
        self.feedback_buffer.fill(Complex64::new(0.0, 0.0));
        self.sample_count = 0;
        self.symbols_processed = 0;
        self.training = false;
        self.training_idx = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> DfeConfig {
        DfeConfig {
            num_forward: 5,
            num_feedback: 3,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { step_size: 0.01 },
        }
    }

    #[test]
    fn test_basic_passthrough() {
        let mut dfe = DecisionFeedbackEqualizer::new(make_config());
        let input: Vec<Complex64> = (0..20)
            .map(|_| Complex64::new(1.0, 0.0))
            .collect();
        let output = dfe.process(&input);
        assert_eq!(output.len(), 20);
    }

    #[test]
    fn test_bpsk_convergence() {
        let config = DfeConfig {
            num_forward: 7,
            num_feedback: 3,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { step_size: 0.05 },
        };
        let mut dfe = DecisionFeedbackEqualizer::new(config);

        // BPSK symbols through a simple ISI channel
        let symbols: Vec<Complex64> = (0..200)
            .map(|i| Complex64::new(if i % 3 == 0 { 1.0 } else { -1.0 }, 0.0))
            .collect();

        // Simple 2-tap channel: h = [1.0, 0.3]
        let mut channel_out = vec![Complex64::new(0.0, 0.0); symbols.len()];
        for i in 0..symbols.len() {
            channel_out[i] = symbols[i];
            if i > 0 {
                channel_out[i] += symbols[i - 1] * Complex64::new(0.3, 0.0);
            }
        }

        let output = dfe.process(&channel_out);
        assert!(output.len() > 100);
        // After convergence, last symbols should be close to ±1
        let last = output.last().unwrap();
        assert!(last.re.abs() > 0.5, "DFE should produce recognizable symbols");
    }

    #[test]
    fn test_training_mode() {
        let mut dfe = DecisionFeedbackEqualizer::new(make_config());
        let training = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(1.0, 0.0),
        ];
        dfe.start_training(&training);
        assert!(dfe.is_training());

        let input: Vec<Complex64> = (0..3)
            .map(|_| Complex64::new(0.9, 0.1))
            .collect();
        dfe.process(&input);
        // Training should have ended
        assert!(!dfe.is_training());
    }

    #[test]
    fn test_cma_algorithm() {
        let config = DfeConfig {
            num_forward: 5,
            num_feedback: 2,
            sps: 1,
            algorithm: AdaptAlgorithm::Cma {
                modulus: 1.0,
                step_size: 0.01,
            },
        };
        let mut dfe = DecisionFeedbackEqualizer::new(config);
        let input: Vec<Complex64> = (0..50)
            .map(|i| Complex64::new(if i % 2 == 0 { 0.8 } else { -0.8 }, 0.0))
            .collect();
        let output = dfe.process(&input);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_qpsk_decision() {
        let mut dfe = DecisionFeedbackEqualizer::new(make_config());
        dfe.set_constellation_order(4);
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let input = vec![Complex64::new(s + 0.1, s - 0.1); 10];
        let output = dfe.process(&input);
        assert!(!output.is_empty());
    }

    #[test]
    fn test_tap_accessors() {
        let dfe = DecisionFeedbackEqualizer::new(make_config());
        assert_eq!(dfe.forward_taps().len(), 5);
        assert_eq!(dfe.feedback_taps().len(), 3);
        // Center tap should be 1.0
        assert!((dfe.forward_taps()[2].re - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_sps_decimation() {
        let config = DfeConfig {
            num_forward: 3,
            num_feedback: 2,
            sps: 4,
            algorithm: AdaptAlgorithm::Lms { step_size: 0.01 },
        };
        let mut dfe = DecisionFeedbackEqualizer::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 40];
        let output = dfe.process(&input);
        assert_eq!(output.len(), 10); // 40 / sps=4 = 10
    }

    #[test]
    fn test_reset() {
        let mut dfe = DecisionFeedbackEqualizer::new(make_config());
        dfe.process(&[Complex64::new(1.0, 0.0); 10]);
        assert!(dfe.symbols_processed() > 0);
        dfe.reset();
        assert_eq!(dfe.symbols_processed(), 0);
        assert!(!dfe.is_training());
    }

    #[test]
    fn test_rls_algorithm() {
        let config = DfeConfig {
            num_forward: 5,
            num_feedback: 2,
            sps: 1,
            algorithm: AdaptAlgorithm::Rls {
                forgetting_factor: 0.99,
            },
        };
        let mut dfe = DecisionFeedbackEqualizer::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 20];
        let output = dfe.process(&input);
        assert_eq!(output.len(), 20);
    }

    #[test]
    fn test_zero_feedback() {
        let config = DfeConfig {
            num_forward: 5,
            num_feedback: 0,
            sps: 1,
            algorithm: AdaptAlgorithm::Lms { step_size: 0.01 },
        };
        let mut dfe = DecisionFeedbackEqualizer::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 10];
        let output = dfe.process(&input);
        assert_eq!(output.len(), 10);
    }
}
