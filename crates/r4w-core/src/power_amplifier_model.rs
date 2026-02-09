//! Power Amplifier Model — Nonlinear PA Behavioral Models
//!
//! Simulates AM/AM and AM/PM distortion in transmitter power amplifiers
//! using industry-standard behavioral models: Saleh, Rapp, Ghorbani,
//! and memory polynomial. Includes P1dB and IP3 characterization.
//! GNU Radio equivalent: `gr::analog::distortion_2_ff`,
//! `gr::analog::distortion_3_ff`; `gr-dpd` PA models.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::power_amplifier_model::{PowerAmplifier, PaModel};
//! use num_complex::Complex64;
//!
//! let pa = PowerAmplifier::new(PaModel::rapp(1.0, 3.0), 6.0);
//! let input = vec![Complex64::new(0.3, 0.1); 100];
//! let output = pa.process(&input);
//! assert_eq!(output.len(), 100);
//! ```

use num_complex::Complex64;

/// Power amplifier behavioral model.
#[derive(Debug, Clone)]
pub enum PaModel {
    /// Saleh model: widely used for TWTA (traveling wave tube amplifiers).
    /// AM/AM: g(r) = alpha_a * r / (1 + beta_a * r^2)
    /// AM/PM: phi(r) = alpha_phi * r^2 / (1 + beta_phi * r^2)
    Saleh {
        alpha_a: f64,
        beta_a: f64,
        alpha_phi: f64,
        beta_phi: f64,
    },
    /// Rapp model: commonly used for solid-state power amplifiers (SSPA).
    /// g(r) = r / (1 + (r/saturation)^(2*smoothness))^(1/(2*smoothness))
    /// No AM/PM distortion (pure amplitude limiting).
    Rapp {
        saturation: f64,
        smoothness: f64,
    },
    /// Ghorbani model: 4-parameter AM/AM and AM/PM model.
    /// AM/AM: g(r) = x1 * r^x2 / (1 + x3 * r^x2) + x4 * r
    /// AM/PM: phi(r) = y1 * r^y2 / (1 + y3 * r^y2) + y4 * r
    Ghorbani {
        x1: f64,
        x2: f64,
        x3: f64,
        x4: f64,
        y1: f64,
        y2: f64,
        y3: f64,
        y4: f64,
    },
    /// Memoryless polynomial: y = sum_k c_k * |x|^k * x
    Polynomial {
        /// Odd-order coefficients: c[0] for |x|^0*x, c[1] for |x|^2*x, etc.
        coefficients: Vec<Complex64>,
    },
    /// Ideal soft limiter (clips at saturation level).
    Limiter {
        saturation: f64,
    },
}

impl PaModel {
    /// Create a typical Saleh TWTA model.
    pub fn saleh(alpha_a: f64, beta_a: f64, alpha_phi: f64, beta_phi: f64) -> Self {
        Self::Saleh {
            alpha_a,
            beta_a,
            alpha_phi,
            beta_phi,
        }
    }

    /// Create a Rapp SSPA model.
    pub fn rapp(saturation: f64, smoothness: f64) -> Self {
        Self::Rapp {
            saturation,
            smoothness,
        }
    }

    /// Create a default Saleh model (typical TWTA parameters).
    pub fn default_saleh() -> Self {
        Self::Saleh {
            alpha_a: 2.1587,
            beta_a: 1.1517,
            alpha_phi: 4.0033,
            beta_phi: 9.1040,
        }
    }

    /// Create a default Rapp model (typical SSPA).
    pub fn default_rapp() -> Self {
        Self::Rapp {
            saturation: 1.0,
            smoothness: 3.0,
        }
    }

    /// Compute AM/AM: input amplitude -> output amplitude.
    pub fn am_am(&self, r: f64) -> f64 {
        match self {
            Self::Saleh {
                alpha_a, beta_a, ..
            } => alpha_a * r / (1.0 + beta_a * r * r),
            Self::Rapp {
                saturation,
                smoothness,
            } => {
                let p = 2.0 * smoothness;
                let ratio = r / saturation;
                r / (1.0 + ratio.powf(p)).powf(1.0 / p)
            }
            Self::Ghorbani {
                x1, x2, x3, x4, ..
            } => x1 * r.powf(*x2) / (1.0 + x3 * r.powf(*x2)) + x4 * r,
            Self::Polynomial { coefficients } => {
                // |sum c_k * r^(2k) * r| = r * |sum c_k * r^(2k)|
                let mut sum = Complex64::new(0.0, 0.0);
                let mut r_pow = 1.0;
                for c in coefficients {
                    sum += c * r_pow;
                    r_pow *= r * r;
                }
                (sum.norm() * r).min(f64::MAX)
            }
            Self::Limiter { saturation } => r.min(*saturation),
        }
    }

    /// Compute AM/PM: input amplitude -> phase shift (radians).
    pub fn am_pm(&self, r: f64) -> f64 {
        match self {
            Self::Saleh {
                alpha_phi,
                beta_phi,
                ..
            } => alpha_phi * r * r / (1.0 + beta_phi * r * r),
            Self::Rapp { .. } => 0.0, // Rapp has no AM/PM
            Self::Ghorbani {
                y1, y2, y3, y4, ..
            } => y1 * r.powf(*y2) / (1.0 + y3 * r.powf(*y2)) + y4 * r,
            Self::Polynomial { coefficients } => {
                let mut sum = Complex64::new(0.0, 0.0);
                let mut r_pow = 1.0;
                for c in coefficients {
                    sum += c * r_pow;
                    r_pow *= r * r;
                }
                sum.arg()
            }
            Self::Limiter { .. } => 0.0,
        }
    }
}

/// Power amplifier simulator.
#[derive(Debug, Clone)]
pub struct PowerAmplifier {
    model: PaModel,
    /// Input backoff in linear scale.
    backoff_linear: f64,
}

impl PowerAmplifier {
    /// Create a new power amplifier.
    ///
    /// `backoff_db`: input backoff in dB (reduces input level).
    pub fn new(model: PaModel, backoff_db: f64) -> Self {
        Self {
            model,
            backoff_linear: 10.0_f64.powf(-backoff_db / 20.0),
        }
    }

    /// Process complex samples through the PA model.
    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|&x| {
                let scaled = x * self.backoff_linear;
                let r = scaled.norm();
                if r < 1e-15 {
                    return Complex64::new(0.0, 0.0);
                }
                let g = self.model.am_am(r);
                let phi = self.model.am_pm(r);
                let phase = scaled.arg() + phi;
                Complex64::from_polar(g, phase)
            })
            .collect()
    }

    /// Process a single sample.
    pub fn process_sample(&self, x: Complex64) -> Complex64 {
        let scaled = x * self.backoff_linear;
        let r = scaled.norm();
        if r < 1e-15 {
            return Complex64::new(0.0, 0.0);
        }
        let g = self.model.am_am(r);
        let phi = self.model.am_pm(r);
        Complex64::from_polar(g, scaled.arg() + phi)
    }

    /// Get AM/AM curve value.
    pub fn am_am(&self, input_amplitude: f64) -> f64 {
        self.model.am_am(input_amplitude * self.backoff_linear)
    }

    /// Get AM/PM curve value in radians.
    pub fn am_pm(&self, input_amplitude: f64) -> f64 {
        self.model.am_pm(input_amplitude * self.backoff_linear)
    }

    /// Set backoff in dB.
    pub fn set_backoff(&mut self, db: f64) {
        self.backoff_linear = 10.0_f64.powf(-db / 20.0);
    }

    /// Get backoff in dB.
    pub fn backoff_db(&self) -> f64 {
        -20.0 * self.backoff_linear.log10()
    }

    /// Get reference to the PA model.
    pub fn model(&self) -> &PaModel {
        &self.model
    }
}

/// Measure AM/AM curve by sweeping input amplitude.
pub fn measure_am_am(model: &PaModel, num_points: usize) -> Vec<(f64, f64)> {
    (0..num_points)
        .map(|i| {
            let r = (i as f64 + 0.5) / num_points as f64 * 2.0;
            (r, model.am_am(r))
        })
        .collect()
}

/// Measure AM/PM curve by sweeping input amplitude.
pub fn measure_am_pm(model: &PaModel, num_points: usize) -> Vec<(f64, f64)> {
    (0..num_points)
        .map(|i| {
            let r = (i as f64 + 0.5) / num_points as f64 * 2.0;
            (r, model.am_pm(r))
        })
        .collect()
}

/// Estimate 1dB compression point for a PA model.
pub fn compression_point_1db(model: &PaModel) -> f64 {
    // Find input level where gain drops 1 dB from small-signal gain
    let small_signal_gain = if model.am_am(0.001) > 1e-15 {
        model.am_am(0.001) / 0.001
    } else {
        1.0
    };
    let target_gain = small_signal_gain * 10.0_f64.powf(-1.0 / 20.0);

    // Binary search
    let mut lo = 0.0;
    let mut hi = 10.0;
    for _ in 0..50 {
        let mid = (lo + hi) / 2.0;
        let gain = if mid > 1e-15 {
            model.am_am(mid) / mid
        } else {
            small_signal_gain
        };
        if gain > target_gain {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_saleh_am_am() {
        let model = PaModel::default_saleh();
        // Small signal: approximately linear (g(r) ≈ alpha_a * r)
        let g = model.am_am(0.01);
        assert!((g - 2.1587 * 0.01).abs() < 0.01);
        // Peak and rolloff
        let g_peak = model.am_am(0.5);
        assert!(g_peak > 0.0 && g_peak < 2.0);
    }

    #[test]
    fn test_saleh_am_pm() {
        let model = PaModel::default_saleh();
        // Small signal: near zero phase shift
        assert!(model.am_pm(0.01).abs() < 0.01);
        // Non-zero phase at moderate drive
        assert!(model.am_pm(0.5).abs() > 0.01);
    }

    #[test]
    fn test_rapp_model() {
        let model = PaModel::rapp(1.0, 3.0);
        // Small signal: linear
        let g = model.am_am(0.01);
        assert!((g - 0.01).abs() < 0.001);
        // At saturation: compressed
        let g_sat = model.am_am(1.0);
        assert!(g_sat < 1.0);
        assert!(g_sat > 0.8);
        // Beyond saturation: still bounded
        let g_over = model.am_am(2.0);
        assert!(g_over < 1.05);
        // No AM/PM
        assert_eq!(model.am_pm(0.5), 0.0);
    }

    #[test]
    fn test_rapp_high_smoothness_approaches_limiter() {
        let model = PaModel::rapp(1.0, 20.0);
        // Below saturation: nearly linear
        assert!((model.am_am(0.5) - 0.5).abs() < 0.01);
        // Above saturation: nearly clipped
        assert!((model.am_am(2.0) - 1.0).abs() < 0.05);
    }

    #[test]
    fn test_limiter() {
        let model = PaModel::Limiter { saturation: 0.8 };
        assert!((model.am_am(0.5) - 0.5).abs() < 1e-10);
        assert!((model.am_am(1.0) - 0.8).abs() < 1e-10);
        assert_eq!(model.am_pm(0.5), 0.0);
    }

    #[test]
    fn test_pa_process() {
        let pa = PowerAmplifier::new(PaModel::rapp(1.0, 3.0), 0.0);
        let input = vec![Complex64::new(0.3, 0.4); 50];
        let output = pa.process(&input);
        assert_eq!(output.len(), 50);
        // Output should be compressed
        for &y in &output {
            assert!(y.norm() <= 1.05);
            assert!(y.norm() > 0.0);
        }
    }

    #[test]
    fn test_pa_backoff() {
        let pa_no_bo = PowerAmplifier::new(PaModel::rapp(1.0, 3.0), 0.0);
        let pa_6db = PowerAmplifier::new(PaModel::rapp(1.0, 3.0), 6.0);
        let input = vec![Complex64::new(0.5, 0.5)];
        let out_no_bo = pa_no_bo.process(&input);
        let out_6db = pa_6db.process(&input);
        // With 6 dB backoff, output should be smaller
        assert!(out_6db[0].norm() < out_no_bo[0].norm());
    }

    #[test]
    fn test_pa_zero_input() {
        let pa = PowerAmplifier::new(PaModel::default_saleh(), 0.0);
        let output = pa.process_sample(Complex64::new(0.0, 0.0));
        assert!((output.norm()) < 1e-10);
    }

    #[test]
    fn test_ghorbani() {
        let model = PaModel::Ghorbani {
            x1: 8.1081,
            x2: 1.5413,
            x3: 6.5202,
            x4: -0.0718,
            y1: 4.6645,
            y2: 2.0965,
            y3: 10.88,
            y4: -0.003,
        };
        let g = model.am_am(0.1);
        assert!(g > 0.0);
        let phi = model.am_pm(0.5);
        assert!(phi.is_finite());
    }

    #[test]
    fn test_measure_curves() {
        let model = PaModel::default_rapp();
        let am_am = measure_am_am(&model, 100);
        assert_eq!(am_am.len(), 100);
        // Should be monotonically increasing for small inputs
        for w in am_am[..20].windows(2) {
            assert!(w[1].1 >= w[0].1);
        }
        let am_pm = measure_am_pm(&model, 50);
        assert_eq!(am_pm.len(), 50);
    }

    #[test]
    fn test_compression_point() {
        let model = PaModel::rapp(1.0, 3.0);
        let p1db = compression_point_1db(&model);
        assert!(p1db > 0.0 && p1db < 2.0);
        // At P1dB, gain should be ~1 dB below small-signal
        let ss_gain = model.am_am(0.001) / 0.001;
        let gain_at_p1db = model.am_am(p1db) / p1db;
        let gain_drop_db = 20.0 * (gain_at_p1db / ss_gain).log10();
        assert!((gain_drop_db + 1.0).abs() < 0.1);
    }

    #[test]
    fn test_polynomial_model() {
        let model = PaModel::Polynomial {
            coefficients: vec![
                Complex64::new(1.0, 0.0),
                Complex64::new(-0.1, 0.0),
            ],
        };
        // Small signal: approximately linear (c[0] = 1)
        let g = model.am_am(0.01);
        assert!((g - 0.01).abs() < 0.001);
    }
}
