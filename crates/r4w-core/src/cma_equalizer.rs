//! # Constant Modulus Algorithm (CMA) Blind Equalizer
//!
//! Adaptive equalizer using the Godard/CMA cost function for blind equalization
//! of constant-envelope signals (PSK, QAM). Unlike LMS which requires a training
//! sequence, CMA converges using only the signal's constant modulus property.
//!
//! ## Algorithm
//!
//! CMA minimizes the Godard cost function: J = E[|y(n)|^p - R_p]^q
//! where R_p is the dispersion constant derived from the constellation.
//! The standard CMA (p=2, q=2) is also known as CMA-2-2.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cma_equalizer::{CmaEqualizer, CmaConfig};
//!
//! let config = CmaConfig {
//!     num_taps: 11,
//!     step_size: 0.001,
//!     modulus: 1.0,  // target |y| for PSK
//!     leak_factor: 0.0,
//! };
//! let mut eq = CmaEqualizer::new(config);
//!
//! // Process IQ samples
//! let input = vec![(1.0, 0.0); 100];
//! let output = eq.process(&input);
//! assert_eq!(output.len(), 100);
//! ```

/// Configuration for the CMA equalizer.
#[derive(Debug, Clone)]
pub struct CmaConfig {
    /// Number of filter taps (odd recommended for symmetry).
    pub num_taps: usize,
    /// Step size (mu) for tap adaptation. Typical: 1e-4 to 1e-2.
    pub step_size: f64,
    /// Target modulus R. For unit-circle constellations (BPSK/QPSK), R=1.0.
    /// For 16-QAM: R = E[|a|^4]/E[|a|^2] ≈ 1.32.
    pub modulus: f64,
    /// Leaky LMS factor (0.0 = no leak). Prevents tap drift.
    pub leak_factor: f64,
}

impl Default for CmaConfig {
    fn default() -> Self {
        Self {
            num_taps: 11,
            step_size: 0.001,
            modulus: 1.0,
            leak_factor: 0.0,
        }
    }
}

/// CMA equalizer variant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CmaVariant {
    /// Standard CMA (Godard p=2, q=2). Most common.
    Cma22,
    /// Modified CMA (p=1, q=2). Less sensitive to outliers.
    Cma12,
    /// Reduced Constellation Algorithm - decision-directed refinement after CMA convergence.
    Rca,
}

/// Constant Modulus Algorithm blind equalizer.
///
/// Adaptively adjusts FIR filter taps to produce output with constant modulus,
/// suitable for PSK and QAM signals without training sequences.
#[derive(Debug, Clone)]
pub struct CmaEqualizer {
    config: CmaConfig,
    variant: CmaVariant,
    /// FIR filter taps (complex).
    taps: Vec<(f64, f64)>,
    /// Input delay line (complex).
    buffer: Vec<(f64, f64)>,
    /// Write position in circular buffer.
    buf_pos: usize,
    /// Running error for monitoring convergence.
    error_power: f64,
    /// Exponential smoothing factor for error power.
    error_alpha: f64,
    /// Total samples processed.
    samples_processed: u64,
}

impl CmaEqualizer {
    /// Create a new CMA equalizer with the given configuration.
    pub fn new(config: CmaConfig) -> Self {
        let num_taps = config.num_taps.max(1);
        let mut taps = vec![(0.0, 0.0); num_taps];
        // Initialize center tap to 1 (pass-through).
        taps[num_taps / 2] = (1.0, 0.0);

        Self {
            config: CmaConfig {
                num_taps,
                ..config
            },
            variant: CmaVariant::Cma22,
            taps,
            buffer: vec![(0.0, 0.0); num_taps],
            buf_pos: 0,
            error_power: 0.0,
            error_alpha: 0.01,
            samples_processed: 0,
        }
    }

    /// Create a CMA equalizer for a specific variant.
    pub fn with_variant(config: CmaConfig, variant: CmaVariant) -> Self {
        let mut eq = Self::new(config);
        eq.variant = variant;
        eq
    }

    /// Create a CMA equalizer pre-configured for QPSK (modulus = 1.0).
    pub fn for_qpsk(num_taps: usize, step_size: f64) -> Self {
        Self::new(CmaConfig {
            num_taps,
            step_size,
            modulus: 1.0,
            leak_factor: 0.0,
        })
    }

    /// Create a CMA equalizer pre-configured for 16-QAM.
    pub fn for_16qam(num_taps: usize, step_size: f64) -> Self {
        // R_16QAM = E[|a|^4] / E[|a|^2]
        // For unit-power 16-QAM: points at ±1, ±3 (normalized)
        // E[|a|^2] = (1+1+9+9+1+9+9+1+9+1+1+9+9+9+1+9*2)/16 ...
        // Simplified: R ≈ 1.32 for standard 16-QAM
        Self::new(CmaConfig {
            num_taps,
            step_size,
            modulus: 1.32,
            leak_factor: 0.0,
        })
    }

    /// Compute the dispersion constant for a given constellation.
    pub fn compute_dispersion(constellation: &[(f64, f64)]) -> f64 {
        if constellation.is_empty() {
            return 1.0;
        }
        let n = constellation.len() as f64;
        let e4: f64 = constellation
            .iter()
            .map(|(re, im)| {
                let mag_sq = re * re + im * im;
                mag_sq * mag_sq
            })
            .sum::<f64>()
            / n;
        let e2: f64 = constellation
            .iter()
            .map(|(re, im)| re * re + im * im)
            .sum::<f64>()
            / n;
        if e2 > 0.0 {
            e4 / e2
        } else {
            1.0
        }
    }

    /// Process a block of IQ samples through the equalizer.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            output.push(self.process_sample(sample));
        }
        output
    }

    /// Process a single IQ sample and return the equalized output.
    pub fn process_sample(&mut self, input: (f64, f64)) -> (f64, f64) {
        // Insert into circular buffer.
        self.buffer[self.buf_pos] = input;

        // Compute FIR output: y = sum(w_k * x(n-k))
        let y = self.compute_output();

        // Compute error based on variant.
        let error = self.compute_error(y);

        // Update taps: w(n+1) = (1-leak)*w(n) - mu * error * x*(n-k)
        self.update_taps(error, y);

        // Advance buffer position.
        self.buf_pos = (self.buf_pos + 1) % self.config.num_taps;
        self.samples_processed += 1;

        y
    }

    fn compute_output(&self) -> (f64, f64) {
        let mut re = 0.0;
        let mut im = 0.0;
        let n = self.config.num_taps;
        for k in 0..n {
            let buf_idx = (self.buf_pos + n - k) % n;
            let (xr, xi) = self.buffer[buf_idx];
            let (wr, wi) = self.taps[k];
            // Complex multiply: (wr + j*wi) * (xr + j*xi)
            re += wr * xr - wi * xi;
            im += wr * xi + wi * xr;
        }
        (re, im)
    }

    fn compute_error(&mut self, y: (f64, f64)) -> (f64, f64) {
        let mag_sq = y.0 * y.0 + y.1 * y.1;
        let r = self.config.modulus;

        match self.variant {
            CmaVariant::Cma22 => {
                // e(n) = y(n) * (|y(n)|^2 - R^2)
                let err_scalar = mag_sq - r * r;
                let err = (y.0 * err_scalar, y.1 * err_scalar);
                self.error_power =
                    (1.0 - self.error_alpha) * self.error_power + self.error_alpha * err_scalar.abs();
                err
            }
            CmaVariant::Cma12 => {
                // e(n) = y(n) * (|y(n)| - R)
                let mag = mag_sq.sqrt();
                let err_scalar = mag - r;
                let err = (y.0 * err_scalar, y.1 * err_scalar);
                self.error_power =
                    (1.0 - self.error_alpha) * self.error_power + self.error_alpha * err_scalar.abs();
                err
            }
            CmaVariant::Rca => {
                // Decision-directed: find nearest constellation point
                // For PSK, project onto unit circle
                let mag = mag_sq.sqrt();
                if mag > 1e-12 {
                    let scale = r / mag;
                    let desired = (y.0 * scale, y.1 * scale);
                    let err = (y.0 - desired.0, y.1 - desired.1);
                    self.error_power = (1.0 - self.error_alpha) * self.error_power
                        + self.error_alpha * (err.0 * err.0 + err.1 * err.1);
                    err
                } else {
                    (0.0, 0.0)
                }
            }
        }
    }

    fn update_taps(&mut self, error: (f64, f64), _y: (f64, f64)) {
        let mu = self.config.step_size;
        let leak = self.config.leak_factor;
        let n = self.config.num_taps;

        for k in 0..n {
            let buf_idx = (self.buf_pos + n - k) % n;
            let (xr, xi) = self.buffer[buf_idx];

            // Conjugate of input: x*(n-k)
            let xr_conj = xr;
            let xi_conj = -xi;

            // Gradient: error * x*(n-k)
            let gr = error.0 * xr_conj - error.1 * xi_conj;
            let gi = error.0 * xi_conj + error.1 * xr_conj;

            // Update: w = (1-leak*mu)*w - mu * gradient
            self.taps[k].0 = (1.0 - leak * mu) * self.taps[k].0 - mu * gr;
            self.taps[k].1 = (1.0 - leak * mu) * self.taps[k].1 - mu * gi;
        }
    }

    /// Get current filter taps.
    pub fn taps(&self) -> &[(f64, f64)] {
        &self.taps
    }

    /// Get the smoothed error power (convergence metric).
    pub fn error_power(&self) -> f64 {
        self.error_power
    }

    /// Get total samples processed.
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed
    }

    /// Get the current variant.
    pub fn variant(&self) -> CmaVariant {
        self.variant
    }

    /// Reset the equalizer state (taps reinitialized to center-spike).
    pub fn reset(&mut self) {
        let n = self.config.num_taps;
        self.taps = vec![(0.0, 0.0); n];
        self.taps[n / 2] = (1.0, 0.0);
        self.buffer = vec![(0.0, 0.0); n];
        self.buf_pos = 0;
        self.error_power = 0.0;
        self.samples_processed = 0;
    }

    /// Set the step size (mu) dynamically.
    pub fn set_step_size(&mut self, mu: f64) {
        self.config.step_size = mu;
    }

    /// Get the current step size.
    pub fn step_size(&self) -> f64 {
        self.config.step_size
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_qpsk_signal(n: usize) -> Vec<(f64, f64)> {
        let symbols = [
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
        ];
        (0..n).map(|i| symbols[i % 4]).collect()
    }

    #[test]
    fn test_passthrough_clean() {
        // Clean QPSK through CMA should remain near unit circle.
        let mut eq = CmaEqualizer::for_qpsk(7, 0.001);
        let input = make_qpsk_signal(200);
        let output = eq.process(&input);
        // After convergence, output magnitude should be near 1.0.
        for &(re, im) in &output[100..] {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 0.3,
                "Magnitude {} too far from 1.0",
                mag
            );
        }
    }

    #[test]
    fn test_gain_correction() {
        // Signal with wrong gain (0.5) should be corrected toward modulus 1.0.
        let mut eq = CmaEqualizer::for_qpsk(11, 0.01);
        let input: Vec<(f64, f64)> = make_qpsk_signal(500)
            .iter()
            .map(|&(r, i)| (r * 0.5, i * 0.5))
            .collect();
        let output = eq.process(&input);
        // After convergence, magnitude should approach 1.0.
        let avg_mag: f64 = output[400..]
            .iter()
            .map(|(r, i)| (r * r + i * i).sqrt())
            .sum::<f64>()
            / 100.0;
        assert!(
            (avg_mag - 1.0).abs() < 0.3,
            "Average magnitude {} should approach 1.0",
            avg_mag
        );
    }

    #[test]
    fn test_cma12_variant() {
        let config = CmaConfig {
            num_taps: 7,
            step_size: 0.005,
            modulus: 1.0,
            leak_factor: 0.0,
        };
        let mut eq = CmaEqualizer::with_variant(config, CmaVariant::Cma12);
        assert_eq!(eq.variant(), CmaVariant::Cma12);
        let input = make_qpsk_signal(200);
        let output = eq.process(&input);
        assert_eq!(output.len(), 200);
    }

    #[test]
    fn test_rca_variant() {
        let config = CmaConfig {
            num_taps: 7,
            step_size: 0.005,
            modulus: 1.0,
            leak_factor: 0.0,
        };
        let mut eq = CmaEqualizer::with_variant(config, CmaVariant::Rca);
        assert_eq!(eq.variant(), CmaVariant::Rca);
        let input = make_qpsk_signal(200);
        let output = eq.process(&input);
        assert_eq!(output.len(), 200);
    }

    #[test]
    fn test_dispersion_constant() {
        // QPSK: all points at |a|=1, so E[|a|^4]/E[|a|^2] = 1.0
        let qpsk = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let r = CmaEqualizer::compute_dispersion(&qpsk);
        assert!((r - 1.0).abs() < 1e-10, "QPSK dispersion should be 1.0, got {}", r);
    }

    #[test]
    fn test_dispersion_16qam() {
        // 16-QAM normalized points
        let scale = 1.0 / 10.0_f64.sqrt();
        let mut constellation = Vec::new();
        for i in [-3.0, -1.0, 1.0, 3.0] {
            for q in [-3.0, -1.0, 1.0, 3.0] {
                constellation.push((i * scale, q * scale));
            }
        }
        let r = CmaEqualizer::compute_dispersion(&constellation);
        assert!(r > 1.0, "16-QAM dispersion should be >1.0, got {}", r);
        assert!(r < 2.0, "16-QAM dispersion should be <2.0, got {}", r);
    }

    #[test]
    fn test_reset() {
        let mut eq = CmaEqualizer::for_qpsk(7, 0.001);
        let input = make_qpsk_signal(100);
        eq.process(&input);
        assert!(eq.samples_processed() > 0);
        eq.reset();
        assert_eq!(eq.samples_processed(), 0);
        assert!((eq.taps()[3].0 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_error_convergence() {
        let mut eq = CmaEqualizer::for_qpsk(11, 0.005);
        let input = make_qpsk_signal(1000);
        let _ = eq.process(&input);
        let err = eq.error_power();
        // Error should be small after convergence on clean signal.
        assert!(err < 1.0, "Error power {} should be small after convergence", err);
    }

    #[test]
    fn test_phase_rotation_correction() {
        // Apply a small phase rotation to QPSK signal.
        let theta = 0.2_f64;
        let cos_t = theta.cos();
        let sin_t = theta.sin();
        let mut eq = CmaEqualizer::for_qpsk(11, 0.005);
        let input: Vec<(f64, f64)> = make_qpsk_signal(500)
            .iter()
            .map(|&(r, i)| (r * cos_t - i * sin_t, r * sin_t + i * cos_t))
            .collect();
        let output = eq.process(&input);
        // CMA restores modulus but not phase — check modulus only.
        for &(re, im) in &output[300..] {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 0.3,
                "After phase rotation, magnitude {} should be near 1.0",
                mag
            );
        }
    }

    #[test]
    fn test_set_step_size() {
        let mut eq = CmaEqualizer::for_qpsk(7, 0.001);
        assert!((eq.step_size() - 0.001).abs() < 1e-12);
        eq.set_step_size(0.01);
        assert!((eq.step_size() - 0.01).abs() < 1e-12);
    }
}
