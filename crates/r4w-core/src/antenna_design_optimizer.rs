//! Parametric antenna design with radiation pattern computation, gain/directivity
//! calculations, and impedance matching.
//!
//! This module provides [`AntennaDesigner`] for analyzing common antenna types
//! (dipole, monopole, rectangular patch, Yagi-Uda) at a specified design
//! frequency.  Complex numbers are represented as `(f64, f64)` tuples `(re, im)`
//! so no external crate is required.
//!
//! # Example
//!
//! ```
//! use r4w_core::antenna_design_optimizer::{AntennaDesigner, AntennaType};
//!
//! // Design a half-wave dipole at 2.4 GHz
//! let freq = 2.4e9;
//! let lambda = 3e8 / freq;
//! let designer = AntennaDesigner::new(freq, AntennaType::Dipole { length_m: lambda / 2.0 }, 0.95);
//!
//! let result = designer.design();
//! assert!(result.gain_dbi > 1.0);
//! assert!(result.beamwidth_deg > 0.0);
//! assert!(result.bandwidth_hz > 0.0);
//! println!("Gain: {:.2} dBi, Beamwidth: {:.1}°", result.gain_dbi, result.beamwidth_deg);
//! ```

use std::f64::consts::PI;

// ── Constants ────────────────────────────────────────────────────────────────

const C: f64 = 299_792_458.0; // speed of light (m/s)
const _ETA_0: f64 = 376.730_313_668; // free-space impedance (Ω)

// ── Complex helper functions ─────────────────────────────────────────────────

/// Complex magnitude.
fn c_abs(z: (f64, f64)) -> f64 {
    (z.0 * z.0 + z.1 * z.1).sqrt()
}

/// Complex multiplication.
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex addition.
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex division.
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex conjugate.
fn c_conj(z: (f64, f64)) -> (f64, f64) {
    (z.0, -z.1)
}

// ── Types ────────────────────────────────────────────────────────────────────

/// Antenna geometry variants.
#[derive(Debug, Clone)]
pub enum AntennaType {
    /// Thin-wire dipole of total length `length_m`.
    Dipole { length_m: f64 },
    /// Quarter-wave monopole over a ground plane.
    Monopole { length_m: f64 },
    /// Rectangular microstrip patch.
    PatchRectangular {
        width_m: f64,
        length_m: f64,
        substrate_er: f64,
    },
    /// Yagi-Uda array along z-axis.
    Yagi {
        num_elements: usize,
        spacing_m: f64,
    },
}

/// Aggregated design output.
#[derive(Debug, Clone)]
pub struct DesignResult {
    /// Feed-point impedance (re, im) in Ohms.
    pub impedance: (f64, f64),
    /// Realised gain in dBi.
    pub gain_dbi: f64,
    /// Directivity in dBi.
    pub directivity_dbi: f64,
    /// Half-power (3 dB) beamwidth in the E-plane (degrees).
    pub beamwidth_deg: f64,
    /// Approximate −10 dB return-loss bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Effective aperture in m².
    pub effective_area_m2: f64,
}

/// L-network matching result.
#[derive(Debug, Clone)]
pub struct MatchingNetwork {
    /// Shunt element value – inductance in henries if positive, |capacitance|
    /// in farads if negative.
    pub shunt_value: f64,
    /// Series element value – same convention.
    pub series_value: f64,
    /// True if shunt element is an inductor; false → capacitor.
    pub shunt_is_inductor: bool,
    /// True if series element is an inductor; false → capacitor.
    pub series_is_inductor: bool,
}

// ── Designer ─────────────────────────────────────────────────────────────────

/// Parametric antenna designer.
///
/// Create with [`AntennaDesigner::new`], then call individual methods or
/// [`design()`](AntennaDesigner::design) for a full summary.
#[derive(Debug, Clone)]
pub struct AntennaDesigner {
    /// Design frequency in Hz.
    pub frequency_hz: f64,
    /// Antenna geometry.
    pub antenna_type: AntennaType,
    /// Radiation efficiency (0.0 – 1.0).
    pub efficiency: f64,
}

impl AntennaDesigner {
    /// Create a new designer.
    ///
    /// `efficiency` is clamped to [0, 1].
    pub fn new(frequency_hz: f64, antenna_type: AntennaType, efficiency: f64) -> Self {
        Self {
            frequency_hz,
            antenna_type,
            efficiency: efficiency.clamp(0.0, 1.0),
        }
    }

    /// Wavelength in metres.
    pub fn lambda(&self) -> f64 {
        C / self.frequency_hz
    }

    /// Wave-number k = 2π/λ.
    fn k(&self) -> f64 {
        2.0 * PI * self.frequency_hz / C
    }

    // ── Radiation pattern ────────────────────────────────────────────────

    /// Normalised E-field radiation pattern in dB.
    ///
    /// `theta` and `phi` are in radians.  The returned value is
    /// 20·log₁₀(|E(θ,φ)| / |E_max|) so the peak is 0 dB.
    pub fn radiation_pattern(&self, theta: f64, phi: f64) -> f64 {
        let e = self.e_field_magnitude(theta, phi);
        let e_max = self.peak_e_field();
        if e_max < 1e-30 || e < 1e-30 {
            return -100.0;
        }
        20.0 * (e / e_max).log10()
    }

    /// Un-normalised |E(θ,φ)| used internally.
    fn e_field_magnitude(&self, theta: f64, _phi: f64) -> f64 {
        let sin_t = theta.sin();
        if sin_t.abs() < 1e-12 {
            return 0.0;
        }
        match &self.antenna_type {
            AntennaType::Dipole { length_m } => {
                let kl2 = self.k() * length_m / 2.0;
                let cos_t = theta.cos();
                let num = (kl2 * cos_t).cos() - (kl2).cos();
                (num / sin_t).abs()
            }
            AntennaType::Monopole { length_m } => {
                // Image theory: same pattern as dipole of 2×length in upper hemisphere.
                if theta > PI / 2.0 {
                    return 0.0;
                }
                let kl2 = self.k() * length_m; // half of 2L
                let cos_t = theta.cos();
                let num = (kl2 * cos_t).cos() - (kl2).cos();
                (num / sin_t).abs()
            }
            AntennaType::PatchRectangular {
                width_m,
                length_m: _,
                substrate_er: _,
            } => {
                // Simplified two-slot model.
                let kw2 = self.k() * width_m / 2.0;
                let x = kw2 * sin_t;
                let sinc = if x.abs() < 1e-12 { 1.0 } else { x.sin() / x };
                (sin_t * sinc).abs()
            }
            AntennaType::Yagi {
                num_elements,
                spacing_m,
            } => {
                // Uniform array factor × dipole element pattern.
                let n = *num_elements as f64;
                let d = *spacing_m;
                let psi = self.k() * d * theta.cos();
                let af = if (psi / 2.0).sin().abs() < 1e-12 {
                    n
                } else {
                    ((n * psi / 2.0).sin() / (psi / 2.0).sin()).abs()
                };
                // Element factor: half-wave dipole.
                let cos_t = theta.cos();
                let ef = if sin_t.abs() < 1e-12 {
                    0.0
                } else {
                    ((PI / 2.0 * cos_t).cos() / sin_t).abs()
                };
                af * ef
            }
        }
    }

    /// Peak |E| over the sphere (sampled at 1° resolution).
    fn peak_e_field(&self) -> f64 {
        let mut max_e: f64 = 0.0;
        for i in 1..180 {
            let theta = (i as f64) * PI / 180.0;
            let e = self.e_field_magnitude(theta, 0.0);
            if e > max_e {
                max_e = e;
            }
        }
        max_e
    }

    // ── Directivity & Gain ───────────────────────────────────────────────

    /// Directivity in dBi computed via numerical integration of the
    /// radiation intensity over the sphere.
    pub fn directivity_dbi(&self) -> f64 {
        let n_theta: usize = 360;
        let n_phi: usize = 720;
        let d_theta = PI / n_theta as f64;
        let d_phi = 2.0 * PI / n_phi as f64;

        let mut u_max: f64 = 0.0;
        let mut p_rad: f64 = 0.0;

        for i in 0..n_theta {
            let theta = (i as f64 + 0.5) * d_theta;
            let sin_t = theta.sin();
            for j in 0..n_phi {
                let phi = (j as f64 + 0.5) * d_phi;
                let e = self.e_field_magnitude(theta, phi);
                let u = e * e; // proportional to radiation intensity
                if u > u_max {
                    u_max = u;
                }
                p_rad += u * sin_t * d_theta * d_phi;
            }
        }

        if p_rad < 1e-30 {
            return 0.0;
        }

        let d = 4.0 * PI * u_max / p_rad;
        10.0 * d.log10()
    }

    /// Realised gain in dBi (directivity × efficiency).
    pub fn gain_dbi(&self) -> f64 {
        let d = self.directivity_dbi();
        d + 10.0 * self.efficiency.log10()
    }

    // ── Beamwidth ────────────────────────────────────────────────────────

    /// Half-power (3 dB) beamwidth in the E-plane (φ = 0) in degrees.
    pub fn half_power_beamwidth(&self) -> f64 {
        // Find angles where pattern drops to −3 dB.
        let step = 0.1_f64; // degrees
        let mut left: Option<f64> = None;
        let mut right: Option<f64> = None;

        let mut i = 1;
        while (i as f64) * step < 180.0 {
            let theta_deg = i as f64 * step;
            let theta = theta_deg.to_radians();
            let db = self.radiation_pattern(theta, 0.0);
            if db >= -3.0 {
                if left.is_none() {
                    left = Some(theta_deg);
                }
                right = Some(theta_deg);
            }
            i += 1;
        }

        match (left, right) {
            (Some(l), Some(r)) => r - l,
            _ => 360.0,
        }
    }

    // ── Impedance ────────────────────────────────────────────────────────

    /// Approximate feed-point impedance (Ω) returned as `(R, X)`.
    pub fn input_impedance(&self) -> (f64, f64) {
        match &self.antenna_type {
            AntennaType::Dipole { length_m } => {
                // Empirical formula for thin-wire dipole.
                let ratio = length_m / self.lambda();
                // At half-wave: ≈73 + j42.5. Interpolate around that.
                let r = 73.0 + 200.0 * (ratio - 0.5).powi(2);
                let x = 42.5 * (2.0 * PI * ratio).tan().clamp(-10.0, 10.0);
                (r, x)
            }
            AntennaType::Monopole { length_m } => {
                // Half the dipole impedance (image theory).
                let ratio = 2.0 * length_m / self.lambda();
                let r = (73.0 + 200.0 * (ratio - 0.5).powi(2)) / 2.0;
                let x = 42.5 / 2.0 * (2.0 * PI * ratio).tan().clamp(-10.0, 10.0);
                (r, x)
            }
            AntennaType::PatchRectangular {
                width_m,
                length_m: _,
                substrate_er,
            } => {
                // Simplified transmission-line model.
                let g = width_m / (120.0 * self.lambda());
                let r_in = 1.0 / (2.0 * g);
                // Patch is inherently resonant → small reactance.
                let x = 5.0 * (substrate_er - 4.0);
                (r_in, x)
            }
            AntennaType::Yagi { num_elements, .. } => {
                // Typical driven-element impedance with mutual coupling.
                let n = *num_elements as f64;
                let r = 73.0 - 8.0 * (n - 1.0);
                let x = 20.0 - 5.0 * (n - 1.0);
                (r.max(10.0), x)
            }
        }
    }

    // ── Effective area ───────────────────────────────────────────────────

    /// Effective aperture Ae = G·λ²/(4π) in m².
    pub fn effective_area(&self) -> f64 {
        let g_linear = 10.0_f64.powf(self.gain_dbi() / 10.0);
        let lambda = self.lambda();
        g_linear * lambda * lambda / (4.0 * PI)
    }

    // ── Matching network ─────────────────────────────────────────────────

    /// Compute an L-network to match the antenna impedance to `z0` Ω (real).
    ///
    /// Returns `None` if the antenna impedance real part is zero.
    pub fn matching_network(&self, z0: f64) -> Option<MatchingNetwork> {
        let (r_a, x_a) = self.input_impedance();
        if r_a < 1e-12 {
            return None;
        }

        let omega = 2.0 * PI * self.frequency_hz;

        // Determine Q of the network.
        let (r_big, r_small, high_to_low) = if r_a > z0 {
            (r_a, z0, true)
        } else {
            (z0, r_a, false)
        };

        let q = ((r_big / r_small) - 1.0).sqrt();

        // Shunt element across the higher-impedance side.
        // Series element in line with the lower-impedance side.
        let x_shunt = r_big / q;
        let x_series_needed = q * r_small;

        // Account for existing reactance of the antenna.
        let x_series = if high_to_low {
            // Shunt across antenna side; series towards z0.
            x_series_needed
        } else {
            // Shunt across z0 side; series towards antenna.
            x_series_needed - x_a
        };

        // Choose inductor or capacitor to realise each reactance.
        let (shunt_val, shunt_is_l) = if x_shunt >= 0.0 {
            (x_shunt / omega, true) // inductor
        } else {
            (1.0 / (omega * x_shunt.abs()), false) // capacitor
        };

        let (series_val, series_is_l) = if x_series >= 0.0 {
            (x_series / omega, true)
        } else {
            (1.0 / (omega * x_series.abs()), false)
        };

        Some(MatchingNetwork {
            shunt_value: shunt_val,
            series_value: series_val,
            shunt_is_inductor: shunt_is_l,
            series_is_inductor: series_is_l,
        })
    }

    // ── Bandwidth ────────────────────────────────────────────────────────

    /// Approximate −10 dB return-loss (VSWR ≈ 2) bandwidth in Hz.
    ///
    /// Uses impedance Q to estimate fractional bandwidth.
    pub fn bandwidth_estimate(&self) -> f64 {
        let (r, x) = self.input_impedance();
        if r < 1e-12 {
            return 0.0;
        }
        let q = x.abs() / r;
        // For VSWR < 2 (return loss < −10 dB): BW ≈ 2/(Q_total * √2) for
        // moderate Q.  Use a simplified model that gives reasonable numbers.
        let fractional_bw = if q < 0.1 {
            // Very broadband – cap at ~20%.
            0.20
        } else {
            (2.0 / (q * 2.0_f64.sqrt())).min(0.50)
        };

        match &self.antenna_type {
            AntennaType::PatchRectangular { substrate_er, .. } => {
                // Patches are inherently narrow-band; scale by substrate.
                let patch_factor = 0.03 / substrate_er.sqrt();
                self.frequency_hz * patch_factor
            }
            _ => self.frequency_hz * fractional_bw,
        }
    }

    // ── Full design ──────────────────────────────────────────────────────

    /// Run a complete design analysis and return a [`DesignResult`].
    pub fn design(&self) -> DesignResult {
        DesignResult {
            impedance: self.input_impedance(),
            gain_dbi: self.gain_dbi(),
            directivity_dbi: self.directivity_dbi(),
            beamwidth_deg: self.half_power_beamwidth(),
            bandwidth_hz: self.bandwidth_estimate(),
            effective_area_m2: self.effective_area(),
        }
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const FREQ_2G4: f64 = 2.4e9;

    fn half_wave_dipole() -> AntennaDesigner {
        let lambda = C / FREQ_2G4;
        AntennaDesigner::new(FREQ_2G4, AntennaType::Dipole { length_m: lambda / 2.0 }, 1.0)
    }

    #[test]
    fn dipole_directivity_near_2_15_dbi() {
        let d = half_wave_dipole();
        let dir = d.directivity_dbi();
        // A half-wave dipole should be ~2.15 dBi.
        assert!(
            (dir - 2.15).abs() < 0.5,
            "Expected directivity ~2.15 dBi, got {dir:.2}"
        );
    }

    #[test]
    fn dipole_gain_equals_directivity_at_unity_efficiency() {
        let d = half_wave_dipole();
        let diff = (d.gain_dbi() - d.directivity_dbi()).abs();
        assert!(diff < 1e-6, "Gain should equal directivity when η=1.0, diff={diff}");
    }

    #[test]
    fn dipole_gain_with_reduced_efficiency() {
        let lambda = C / FREQ_2G4;
        let d = AntennaDesigner::new(
            FREQ_2G4,
            AntennaType::Dipole { length_m: lambda / 2.0 },
            0.5,
        );
        let expected_diff = 10.0 * 0.5_f64.log10(); // −3.01 dB
        let actual_diff = d.gain_dbi() - d.directivity_dbi();
        assert!(
            (actual_diff - expected_diff).abs() < 0.01,
            "Expected {expected_diff:.2}, got {actual_diff:.2}"
        );
    }

    #[test]
    fn dipole_pattern_peak_at_broadside() {
        let d = half_wave_dipole();
        let peak = d.radiation_pattern(PI / 2.0, 0.0);
        assert!(
            peak.abs() < 0.5,
            "Peak should be ~0 dB at θ=90°, got {peak:.2}"
        );
    }

    #[test]
    fn dipole_pattern_null_on_axis() {
        let d = half_wave_dipole();
        let on_axis = d.radiation_pattern(0.01, 0.0);
        assert!(
            on_axis < -20.0,
            "Should be deep null near θ=0, got {on_axis:.2} dB"
        );
    }

    #[test]
    fn dipole_beamwidth_around_78_deg() {
        let d = half_wave_dipole();
        let bw = d.half_power_beamwidth();
        assert!(
            (bw - 78.0).abs() < 8.0,
            "Expected beamwidth ~78°, got {bw:.1}°"
        );
    }

    #[test]
    fn dipole_impedance_reasonable() {
        let d = half_wave_dipole();
        let (r, _x) = d.input_impedance();
        assert!(
            (60.0..=90.0).contains(&r),
            "Expected R ∈ [60,90] Ω for half-wave dipole, got {r:.1}"
        );
    }

    #[test]
    fn monopole_impedance_half_of_dipole() {
        let lambda = C / FREQ_2G4;
        let mono = AntennaDesigner::new(
            FREQ_2G4,
            AntennaType::Monopole { length_m: lambda / 4.0 },
            1.0,
        );
        let (r_m, _) = mono.input_impedance();
        assert!(
            (30.0..=45.0).contains(&r_m),
            "Expected monopole R ∈ [30,45] Ω, got {r_m:.1}"
        );
    }

    #[test]
    fn patch_impedance_positive() {
        let d = AntennaDesigner::new(
            FREQ_2G4,
            AntennaType::PatchRectangular {
                width_m: 0.038,
                length_m: 0.029,
                substrate_er: 4.4,
            },
            0.9,
        );
        let (r, _) = d.input_impedance();
        assert!(r > 0.0, "Patch resistance must be positive, got {r}");
    }

    #[test]
    fn effective_area_positive() {
        let d = half_wave_dipole();
        let ae = d.effective_area();
        assert!(ae > 0.0, "Effective area must be > 0, got {ae}");
        // For a half-wave dipole: Ae ≈ 0.13 λ².
        let lambda = d.lambda();
        let ratio = ae / (lambda * lambda);
        assert!(
            (0.05..=0.25).contains(&ratio),
            "Ae/λ² should be ~0.13, got {ratio:.3}"
        );
    }

    #[test]
    fn matching_network_produces_values() {
        let d = half_wave_dipole();
        let mn = d.matching_network(50.0).expect("matching network should exist");
        assert!(mn.shunt_value > 0.0);
        assert!(mn.series_value > 0.0);
    }

    #[test]
    fn bandwidth_positive_for_all_types() {
        let lambda = C / FREQ_2G4;
        let types: Vec<AntennaType> = vec![
            AntennaType::Dipole { length_m: lambda / 2.0 },
            AntennaType::Monopole { length_m: lambda / 4.0 },
            AntennaType::PatchRectangular {
                width_m: 0.038,
                length_m: 0.029,
                substrate_er: 4.4,
            },
            AntennaType::Yagi {
                num_elements: 5,
                spacing_m: lambda * 0.3,
            },
        ];
        for at in types {
            let d = AntennaDesigner::new(FREQ_2G4, at.clone(), 0.9);
            let bw = d.bandwidth_estimate();
            assert!(bw > 0.0, "Bandwidth must be > 0 for {at:?}, got {bw}");
        }
    }

    #[test]
    fn yagi_higher_gain_than_dipole() {
        let lambda = C / FREQ_2G4;
        let dipole = AntennaDesigner::new(
            FREQ_2G4,
            AntennaType::Dipole { length_m: lambda / 2.0 },
            1.0,
        );
        let yagi = AntennaDesigner::new(
            FREQ_2G4,
            AntennaType::Yagi {
                num_elements: 5,
                spacing_m: lambda * 0.3,
            },
            1.0,
        );
        assert!(
            yagi.gain_dbi() > dipole.gain_dbi(),
            "Yagi ({:.2}) should have higher gain than dipole ({:.2})",
            yagi.gain_dbi(),
            dipole.gain_dbi()
        );
    }

    #[test]
    fn design_result_coherent() {
        let d = half_wave_dipole();
        let r = d.design();
        assert!((r.gain_dbi - d.gain_dbi()).abs() < 1e-9);
        assert!((r.directivity_dbi - d.directivity_dbi()).abs() < 1e-9);
        assert!((r.beamwidth_deg - d.half_power_beamwidth()).abs() < 1e-9);
        assert!((r.bandwidth_hz - d.bandwidth_estimate()).abs() < 1e-9);
        assert!((r.effective_area_m2 - d.effective_area()).abs() < 1e-12);
    }

    #[test]
    fn complex_helpers_basic() {
        let a = (3.0, 4.0);
        assert!((c_abs(a) - 5.0).abs() < 1e-12);
        let b = (1.0, 2.0);
        let prod = c_mul(a, b);
        // (3+4i)(1+2i) = 3+6i+4i+8i² = -5+10i
        assert!((prod.0 - (-5.0)).abs() < 1e-12);
        assert!((prod.1 - 10.0).abs() < 1e-12);
        let sum = c_add(a, b);
        assert!((sum.0 - 4.0).abs() < 1e-12);
        assert!((sum.1 - 6.0).abs() < 1e-12);
        let diff = c_sub(a, b);
        assert!((diff.0 - 2.0).abs() < 1e-12);
        assert!((diff.1 - 2.0).abs() < 1e-12);
        let ratio = c_div(a, b);
        // (3+4i)/(1+2i) = (3+4i)(1-2i)/5 = (11-2i)/5
        assert!((ratio.0 - 2.2).abs() < 1e-12);
        assert!((ratio.1 - (-0.4)).abs() < 1e-12);
        let conj = c_conj(a);
        assert!((conj.0 - 3.0).abs() < 1e-12);
        assert!((conj.1 - (-4.0)).abs() < 1e-12);
    }
}
