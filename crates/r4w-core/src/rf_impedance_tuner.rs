//! Adaptive impedance matching network controller for dynamic load conditions.
//!
//! This module provides tools for computing impedance matching networks,
//! Smith chart transformations, and RF tuner metrics (VSWR, return loss,
//! reflection coefficient).
//!
//! Complex impedance values are represented as `(f64, f64)` tuples where
//! the first element is the real part and the second is the imaginary part.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_impedance_tuner::{ImpedanceTuner, TunerConfig, MatchingNetwork};
//!
//! let config = TunerConfig {
//!     z0: 50.0,
//!     freq_hz: 1.0e9,
//!     max_iterations: 100,
//! };
//! let tuner = ImpedanceTuner::new(config);
//!
//! // A 75-ohm resistive load
//! let z_load = (75.0, 0.0);
//! let result = tuner.tune(z_load);
//!
//! // VSWR should be reasonable for a mild mismatch
//! assert!(result.vswr >= 1.0);
//! assert!(result.return_loss_db < 0.0);
//!
//! // Reflection coefficient magnitude should be between 0 and 1
//! let gamma_mag = (result.reflection_coeff.0.powi(2)
//!     + result.reflection_coeff.1.powi(2)).sqrt();
//! assert!(gamma_mag < 1.0);
//! ```

use std::f64::consts::PI;

// --- Complex arithmetic helpers (using (f64,f64) tuples) ---

fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom == 0.0 {
        return (f64::INFINITY, f64::INFINITY);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

fn c_mag(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

fn c_recip(a: (f64, f64)) -> (f64, f64) {
    c_div((1.0, 0.0), a)
}

// --- Matching Network Enum ---

/// Types of impedance matching networks.
#[derive(Debug, Clone, PartialEq)]
pub enum MatchingNetwork {
    /// L-section: series element then shunt element.
    /// Values: (series reactance, shunt susceptance).
    LSection {
        series_reactance: f64,
        shunt_susceptance: f64,
    },
    /// Pi-section: shunt-series-shunt topology.
    /// Values: (shunt_b1, series_x, shunt_b2).
    PiSection {
        shunt_b1: f64,
        series_x: f64,
        shunt_b2: f64,
    },
    /// T-section: series-shunt-series topology.
    /// Values: (series_x1, shunt_b, series_x2).
    TSection {
        series_x1: f64,
        shunt_b: f64,
        series_x2: f64,
    },
    /// Stub matching: transmission line stub.
    /// Values: (stub_length_wavelengths, distance_wavelengths).
    StubMatch {
        stub_length: f64,
        distance: f64,
    },
}

// --- TunerConfig ---

/// Configuration for the impedance tuner.
#[derive(Debug, Clone)]
pub struct TunerConfig {
    /// Characteristic impedance in ohms (default: 50.0).
    pub z0: f64,
    /// Operating frequency in Hz.
    pub freq_hz: f64,
    /// Maximum iterations for iterative matching algorithms.
    pub max_iterations: usize,
}

impl Default for TunerConfig {
    fn default() -> Self {
        Self {
            z0: 50.0,
            freq_hz: 1.0e9,
            max_iterations: 100,
        }
    }
}

// --- TunerResult ---

/// Result of an impedance tuning operation.
#[derive(Debug, Clone)]
pub struct TunerResult {
    /// The matched impedance after the network is applied.
    pub matched_impedance: (f64, f64),
    /// Reflection coefficient of the original load (complex).
    pub reflection_coeff: (f64, f64),
    /// Voltage Standing Wave Ratio of the original load.
    pub vswr: f64,
    /// Return loss in dB of the original load (negative value).
    pub return_loss_db: f64,
    /// The matching network component values.
    pub network_values: MatchingNetwork,
}

// --- SmithChartCalculator ---

/// Provides impedance transformations on the Smith chart.
pub struct SmithChartCalculator {
    /// Characteristic impedance for normalization.
    z0: f64,
}

impl SmithChartCalculator {
    /// Create a new Smith chart calculator with the given characteristic impedance.
    pub fn new(z0: f64) -> Self {
        Self { z0 }
    }

    /// Convert impedance to reflection coefficient.
    ///
    /// Gamma = (Z - Z0) / (Z + Z0)
    pub fn impedance_to_reflection(&self, z: (f64, f64)) -> (f64, f64) {
        let z0 = (self.z0, 0.0);
        c_div(c_sub(z, z0), c_add(z, z0))
    }

    /// Convert reflection coefficient to impedance.
    ///
    /// Z = Z0 * (1 + Gamma) / (1 - Gamma)
    pub fn reflection_to_impedance(&self, gamma: (f64, f64)) -> (f64, f64) {
        let one = (1.0, 0.0);
        let z0 = (self.z0, 0.0);
        c_mul(z0, c_div(c_add(one, gamma), c_sub(one, gamma)))
    }

    /// Normalize impedance by the characteristic impedance.
    ///
    /// z_norm = Z / Z0
    pub fn normalize(&self, z: (f64, f64)) -> (f64, f64) {
        (z.0 / self.z0, z.1 / self.z0)
    }

    /// Add a series inductance and return the new impedance.
    ///
    /// Z_new = Z + j * 2 * pi * freq * L
    pub fn series_inductance(&self, z: (f64, f64), l: f64, freq: f64) -> (f64, f64) {
        let xl = 2.0 * PI * freq * l;
        c_add(z, (0.0, xl))
    }

    /// Add a shunt capacitance and return the new impedance.
    ///
    /// Y_new = Y + j * 2 * pi * freq * C, then Z = 1/Y_new
    pub fn shunt_capacitance(&self, z: (f64, f64), c: f64, freq: f64) -> (f64, f64) {
        let bc = 2.0 * PI * freq * c;
        let y = c_recip(z);
        let y_new = c_add(y, (0.0, bc));
        c_recip(y_new)
    }
}

// --- ImpedanceTuner ---

/// Adaptive impedance matching network controller.
///
/// Given a load impedance, the tuner computes an L-section matching network
/// to transform the load to the characteristic impedance, and provides
/// VSWR, return loss, and reflection coefficient metrics.
pub struct ImpedanceTuner {
    config: TunerConfig,
    smith: SmithChartCalculator,
}

impl ImpedanceTuner {
    /// Create a new impedance tuner with the given configuration.
    pub fn new(config: TunerConfig) -> Self {
        let smith = SmithChartCalculator::new(config.z0);
        Self { config, smith }
    }

    /// Compute the reflection coefficient for a given load impedance.
    ///
    /// Gamma = (Z_L - Z_0) / (Z_L + Z_0)
    pub fn reflection_coefficient(&self, z_load: (f64, f64)) -> (f64, f64) {
        self.smith.impedance_to_reflection(z_load)
    }

    /// Compute the VSWR for a given load impedance.
    ///
    /// VSWR = (1 + |Gamma|) / (1 - |Gamma|)
    pub fn vswr(&self, z_load: (f64, f64)) -> f64 {
        let gamma = self.reflection_coefficient(z_load);
        let mag = c_mag(gamma);
        if mag >= 1.0 {
            return f64::INFINITY;
        }
        (1.0 + mag) / (1.0 - mag)
    }

    /// Compute the return loss in dB for a given load impedance.
    ///
    /// Return loss = 20 * log10(|Gamma|)
    ///
    /// Convention: return loss is expressed as a negative value
    /// (higher magnitude means worse match). A perfect match yields
    /// negative infinity.
    pub fn return_loss_db(&self, z_load: (f64, f64)) -> f64 {
        let gamma = self.reflection_coefficient(z_load);
        let mag = c_mag(gamma);
        if mag <= 0.0 {
            return f64::NEG_INFINITY; // perfect match
        }
        20.0 * mag.log10()
    }

    /// Design an L-section matching network and return a full `TunerResult`.
    ///
    /// The L-section topology depends on the load resistance relative to Z0:
    ///
    /// - **R_L > Z0**: Shunt susceptance B in parallel with load, then series
    ///   reactance X_s. The shunt element brings the conductance to 1/Z0, and
    ///   the series element cancels the remaining reactance.
    ///
    /// - **R_L < Z0**: Series reactance X_s in series with load, then shunt
    ///   susceptance B. The series element adjusts the impedance so that the
    ///   real part of the admittance equals 1/Z0, and the shunt element cancels
    ///   the remaining susceptance.
    ///
    /// For purely reactive loads (R_L <= 0) or perfect matches, a trivial
    /// network is returned.
    pub fn tune(&self, z_load: (f64, f64)) -> TunerResult {
        let z0 = self.config.z0;
        let r_l = z_load.0;
        let x_l = z_load.1;
        let gamma = self.reflection_coefficient(z_load);
        let gamma_mag = c_mag(gamma);
        let vswr = if gamma_mag >= 1.0 {
            f64::INFINITY
        } else {
            (1.0 + gamma_mag) / (1.0 - gamma_mag)
        };
        let return_loss = if gamma_mag <= 0.0 {
            f64::NEG_INFINITY
        } else {
            20.0 * gamma_mag.log10()
        };

        // If the load is already matched (or very close), return trivial network
        if (r_l - z0).abs() < 1e-10 && x_l.abs() < 1e-10 {
            return TunerResult {
                matched_impedance: z_load,
                reflection_coeff: gamma,
                vswr,
                return_loss_db: return_loss,
                network_values: MatchingNetwork::LSection {
                    series_reactance: 0.0,
                    shunt_susceptance: 0.0,
                },
            };
        }

        // Cannot match purely reactive loads (r_l <= 0)
        if r_l <= 0.0 {
            return TunerResult {
                matched_impedance: z_load,
                reflection_coeff: gamma,
                vswr,
                return_loss_db: return_loss,
                network_values: MatchingNetwork::LSection {
                    series_reactance: 0.0,
                    shunt_susceptance: 0.0,
                },
            };
        }

        // L-section matching design
        let (series_x, shunt_b, z_after) = if r_l > z0 {
            // Case 1: R_L > Z0 -> shunt-first topology
            // Add shunt susceptance B in parallel with load, then series reactance X_s.
            //
            // Admittance of load: Y_L = G_L + jB_L
            //   G_L = R_L / (R_L^2 + X_L^2)
            //   B_L = -X_L / (R_L^2 + X_L^2)
            //
            // After shunt B: Y' = G_L + j(B_L + B)
            // Impedance: Z' = 1/Y'
            // Require Re(Z') = Z0:
            //   G_L / (G_L^2 + (B_L+B)^2) = Z0
            //   (B_L+B)^2 = G_L/Z0 - G_L^2 = G_L*(1/Z0 - G_L)
            //   B_L+B = +/- sqrt(G_L*(1/Z0 - G_L))
            //
            // Then X_s = -Im(Z') to cancel remaining reactance.
            let mag_sq = r_l * r_l + x_l * x_l;
            let g_l = r_l / mag_sq;
            let b_l = -x_l / mag_sq;

            let discriminant = g_l * (1.0 / z0 - g_l);
            // discriminant >= 0 when R_L >= Z0 (guaranteed by this branch)
            let b_total = discriminant.max(0.0).sqrt();

            // Choose the positive root (B_L + B = +b_total)
            let b = b_total - b_l;

            // Compute Z' after shunt
            let y_re = g_l;
            let y_im = b_l + b; // = b_total
            let y_mag_sq = y_re * y_re + y_im * y_im;
            let z_re = y_re / y_mag_sq;
            let z_im = -y_im / y_mag_sq;

            // Series reactance cancels the imaginary part
            let x_s = -z_im;

            let z_matched = (z_re, z_im + x_s); // should be (z0, 0)

            (x_s, b, z_matched)
        } else {
            // Case 2: R_L < Z0 -> series-first topology
            // Add series reactance X_s, then shunt susceptance B.
            //
            // After series X_s: Z' = R_L + j(X_L + X_s)
            // Admittance: Y' = 1/Z'
            // Require Re(Y') = 1/Z0:
            //   R_L / (R_L^2 + (X_L+X_s)^2) = 1/Z0
            //   (X_L+X_s)^2 = R_L*Z0 - R_L^2 = R_L*(Z0 - R_L)
            //   X_L+X_s = +/- sqrt(R_L*(Z0 - R_L))
            //
            // Then B = -Im(Y') to cancel remaining susceptance.
            let discriminant = r_l * (z0 - r_l);
            // discriminant >= 0 when R_L <= Z0 (guaranteed by this branch)
            let x_total = discriminant.max(0.0).sqrt();

            // Choose the positive root (X_L + X_s = +x_total)
            let x_s = x_total - x_l;

            // Compute Y' after series element
            let z_re = r_l;
            let z_im = x_l + x_s; // = x_total
            let z_mag_sq = z_re * z_re + z_im * z_im;
            let y_re = z_re / z_mag_sq;
            let y_im = -z_im / z_mag_sq;

            // Shunt susceptance cancels the imaginary part
            let b = -y_im;

            let y_matched_re = y_re;
            let y_matched_im = y_im + b; // should be 0
            let y_matched_mag_sq = y_matched_re * y_matched_re + y_matched_im * y_matched_im;
            let z_matched = (
                y_matched_re / y_matched_mag_sq,
                -y_matched_im / y_matched_mag_sq,
            );

            (x_s, b, z_matched)
        };

        TunerResult {
            matched_impedance: z_after,
            reflection_coeff: gamma,
            vswr,
            return_loss_db: return_loss,
            network_values: MatchingNetwork::LSection {
                series_reactance: series_x,
                shunt_susceptance: shunt_b,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    fn c_approx_eq(a: (f64, f64), b: (f64, f64), eps: f64) -> bool {
        approx_eq(a.0, b.0, eps) && approx_eq(a.1, b.1, eps)
    }

    // --- Complex arithmetic tests ---

    #[test]
    fn test_complex_add() {
        let a = (1.0, 2.0);
        let b = (3.0, 4.0);
        let result = c_add(a, b);
        assert!(c_approx_eq(result, (4.0, 6.0), EPSILON));
    }

    #[test]
    fn test_complex_sub() {
        let result = c_sub((5.0, 3.0), (2.0, 1.0));
        assert!(c_approx_eq(result, (3.0, 2.0), EPSILON));
    }

    #[test]
    fn test_complex_mul() {
        // (1+2j)(3+4j) = (3-8)+(4+6)j = -5+10j
        let result = c_mul((1.0, 2.0), (3.0, 4.0));
        assert!(c_approx_eq(result, (-5.0, 10.0), EPSILON));
    }

    #[test]
    fn test_complex_div() {
        let result = c_div((1.0, 2.0), (1.0, 0.0));
        assert!(c_approx_eq(result, (1.0, 2.0), EPSILON));

        let result2 = c_div((4.0, 2.0), (2.0, 0.0));
        assert!(c_approx_eq(result2, (2.0, 1.0), EPSILON));
    }

    #[test]
    fn test_complex_mag() {
        assert!(approx_eq(c_mag((3.0, 4.0)), 5.0, EPSILON));
        assert!(approx_eq(c_mag((0.0, 0.0)), 0.0, EPSILON));
        assert!(approx_eq(c_mag((1.0, 0.0)), 1.0, EPSILON));
    }

    #[test]
    fn test_complex_recip() {
        // 1/(2+0j) = (0.5+0j)
        let result = c_recip((2.0, 0.0));
        assert!(c_approx_eq(result, (0.5, 0.0), EPSILON));

        // 1/(0+1j) = (0-1j)
        let result2 = c_recip((0.0, 1.0));
        assert!(c_approx_eq(result2, (0.0, -1.0), EPSILON));
    }

    // --- SmithChartCalculator tests ---

    #[test]
    fn test_impedance_to_reflection_matched() {
        let sc = SmithChartCalculator::new(50.0);
        // Z = Z0 -> Gamma = 0
        let gamma = sc.impedance_to_reflection((50.0, 0.0));
        assert!(c_approx_eq(gamma, (0.0, 0.0), EPSILON));
    }

    #[test]
    fn test_impedance_to_reflection_open() {
        let sc = SmithChartCalculator::new(50.0);
        // Z very large -> Gamma approaches (1, 0)
        let gamma = sc.impedance_to_reflection((1e12, 0.0));
        assert!(approx_eq(c_mag(gamma), 1.0, 1e-6));
    }

    #[test]
    fn test_impedance_to_reflection_short() {
        let sc = SmithChartCalculator::new(50.0);
        // Z = 0 -> Gamma = -1
        let gamma = sc.impedance_to_reflection((0.0, 0.0));
        assert!(c_approx_eq(gamma, (-1.0, 0.0), EPSILON));
    }

    #[test]
    fn test_reflection_to_impedance_roundtrip() {
        let sc = SmithChartCalculator::new(50.0);
        let z_orig = (75.0, 30.0);
        let gamma = sc.impedance_to_reflection(z_orig);
        let z_back = sc.reflection_to_impedance(gamma);
        assert!(
            c_approx_eq(z_back, z_orig, 1e-9),
            "roundtrip failed: got ({}, {}), expected ({}, {})",
            z_back.0,
            z_back.1,
            z_orig.0,
            z_orig.1
        );
    }

    #[test]
    fn test_normalize() {
        let sc = SmithChartCalculator::new(50.0);
        let z = (100.0, 50.0);
        let z_norm = sc.normalize(z);
        assert!(c_approx_eq(z_norm, (2.0, 1.0), EPSILON));
    }

    #[test]
    fn test_series_inductance() {
        let sc = SmithChartCalculator::new(50.0);
        let z = (50.0, 0.0);
        let freq = 1.0e9;
        let l = 1.0e-9; // 1 nH
        let z_new = sc.series_inductance(z, l, freq);
        let expected_xl = 2.0 * PI * freq * l;
        assert!(approx_eq(z_new.0, 50.0, EPSILON));
        assert!(approx_eq(z_new.1, expected_xl, EPSILON));
    }

    #[test]
    fn test_shunt_capacitance() {
        let sc = SmithChartCalculator::new(50.0);
        let z = (50.0, 0.0);
        let freq = 1.0e9;
        let c = 1.0e-12; // 1 pF
        let z_new = sc.shunt_capacitance(z, c, freq);
        let y_expected = (1.0 / 50.0, 2.0 * PI * freq * c);
        let z_expected = c_recip(y_expected);
        assert!(c_approx_eq(z_new, z_expected, 1e-6));
    }

    // --- ImpedanceTuner tests ---

    #[test]
    fn test_vswr_matched() {
        let tuner = ImpedanceTuner::new(TunerConfig {
            z0: 50.0,
            freq_hz: 1.0e9,
            max_iterations: 100,
        });
        let vswr = tuner.vswr((50.0, 0.0));
        assert!(
            approx_eq(vswr, 1.0, EPSILON),
            "VSWR of matched load should be 1.0, got {}",
            vswr
        );
    }

    #[test]
    fn test_vswr_2_to_1() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        // Z=100+0j on 50 ohm -> Gamma = 50/150 = 1/3
        // VSWR = (1+1/3)/(1-1/3) = 2.0
        let vswr = tuner.vswr((100.0, 0.0));
        assert!(
            approx_eq(vswr, 2.0, EPSILON),
            "expected VSWR=2.0, got {}",
            vswr
        );
    }

    #[test]
    fn test_vswr_open_circuit() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        // Very high impedance -> VSWR approaches infinity
        let vswr = tuner.vswr((1e12, 0.0));
        assert!(vswr > 1e6, "open circuit should have very high VSWR");
    }

    #[test]
    fn test_return_loss_matched() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        let rl = tuner.return_loss_db((50.0, 0.0));
        assert!(
            rl == f64::NEG_INFINITY,
            "expected -inf return loss for matched load, got {}",
            rl
        );
    }

    #[test]
    fn test_return_loss_mismatched() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        // Z=100 -> |Gamma|=1/3 -> RL = 20*log10(1/3) ~ -9.54 dB
        let rl = tuner.return_loss_db((100.0, 0.0));
        let expected = 20.0 * (1.0_f64 / 3.0).log10();
        assert!(
            approx_eq(rl, expected, 0.01),
            "expected return loss ~ {:.2} dB, got {:.2} dB",
            expected,
            rl
        );
    }

    #[test]
    fn test_tune_already_matched() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        let result = tuner.tune((50.0, 0.0));
        assert!(approx_eq(result.vswr, 1.0, EPSILON));
        match result.network_values {
            MatchingNetwork::LSection {
                series_reactance,
                shunt_susceptance,
            } => {
                assert!(approx_eq(series_reactance, 0.0, EPSILON));
                assert!(approx_eq(shunt_susceptance, 0.0, EPSILON));
            }
            _ => panic!("expected LSection for matched load"),
        }
    }

    #[test]
    fn test_tune_resistive_high() {
        // R_L > Z0: 100 ohm load on 50 ohm line
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        let result = tuner.tune((100.0, 0.0));
        assert!(
            approx_eq(result.matched_impedance.0, 50.0, 1e-9),
            "matched real part should be ~50, got {}",
            result.matched_impedance.0
        );
        assert!(
            result.matched_impedance.1.abs() < 1e-9,
            "matched imag part should be ~0, got {}",
            result.matched_impedance.1
        );
    }

    #[test]
    fn test_tune_resistive_low() {
        // R_L < Z0: 25 ohm load on 50 ohm line
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        let result = tuner.tune((25.0, 0.0));
        assert!(
            approx_eq(result.matched_impedance.0, 50.0, 1e-9),
            "matched real part should be ~50, got {}",
            result.matched_impedance.0
        );
        assert!(
            result.matched_impedance.1.abs() < 1e-9,
            "matched imag part should be ~0, got {}",
            result.matched_impedance.1
        );
    }

    #[test]
    fn test_tune_complex_load() {
        // Complex load: 30 + j*40 ohms
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        let result = tuner.tune((30.0, 40.0));
        assert!(
            approx_eq(result.matched_impedance.0, 50.0, 1e-9),
            "matched real part should be ~50, got {:.10}",
            result.matched_impedance.0
        );
        assert!(
            result.matched_impedance.1.abs() < 1e-9,
            "matched imag part should be ~0, got {:.10}",
            result.matched_impedance.1
        );
    }

    #[test]
    fn test_tune_high_impedance_complex() {
        // Complex load with R_L > Z0: 150 + j*80
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        let result = tuner.tune((150.0, 80.0));
        assert!(
            approx_eq(result.matched_impedance.0, 50.0, 1e-6),
            "matched real part should be ~50, got {:.10}",
            result.matched_impedance.0
        );
        assert!(
            result.matched_impedance.1.abs() < 1e-6,
            "matched imag part should be ~0, got {:.10}",
            result.matched_impedance.1
        );
    }

    #[test]
    fn test_reflection_coefficient_real_load() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        // Z=75 on 50 ohm -> Gamma = (75-50)/(75+50) = 25/125 = 0.2
        let gamma = tuner.reflection_coefficient((75.0, 0.0));
        assert!(approx_eq(gamma.0, 0.2, EPSILON));
        assert!(approx_eq(gamma.1, 0.0, EPSILON));
    }

    #[test]
    fn test_reflection_coefficient_reactive_load() {
        let tuner = ImpedanceTuner::new(TunerConfig::default());
        // Purely reactive load: Z = j*50
        // Gamma = (j50 - 50) / (j50 + 50) = (-50+j50)/(50+j50)
        // |Gamma| = |(-50+j50)| / |(50+j50)| = sqrt(5000)/sqrt(5000) = 1
        let gamma = tuner.reflection_coefficient((0.0, 50.0));
        assert!(
            approx_eq(c_mag(gamma), 1.0, EPSILON),
            "|Gamma| for purely reactive load should be 1.0, got {}",
            c_mag(gamma)
        );
    }
}
