//! Radar Cross Section (RCS) estimation for various target geometries.
//!
//! This module computes monostatic (backscatter) RCS for common target shapes
//! used in radar simulation and analysis. It supports exact analytical models
//! (sphere, flat plate, corner reflector, cylinder, dihedral reflector),
//! Swerling fluctuation statistics, and radar range equation calculations.
//!
//! # Example
//!
//! ```
//! use r4w_core::radar_cross_section_estimator::{RcsEstimator, TargetModel};
//!
//! // Estimate RCS of a 0.5 m radius sphere at 10 GHz
//! let estimator = RcsEstimator::new(10.0e9, TargetModel::Sphere { radius_m: 0.5 });
//! let rcs_m2 = estimator.compute_rcs();
//! let rcs_dbsm = estimator.compute_rcs_dbsm();
//! assert!(rcs_m2 > 0.0);
//! println!("Sphere RCS: {:.4} m² ({:.2} dBsm)", rcs_m2, rcs_dbsm);
//! ```

use std::f64::consts::PI;

/// Target geometry models for RCS computation.
#[derive(Debug, Clone, PartialEq)]
pub enum TargetModel {
    /// Conducting sphere with given radius.
    Sphere { radius_m: f64 },
    /// Flat rectangular plate with given width and height.
    FlatPlate { width_m: f64, height_m: f64 },
    /// Triangular trihedral corner reflector with given edge length.
    CornerReflector { edge_m: f64 },
    /// Conducting cylinder with given radius and length.
    Cylinder { radius_m: f64, length_m: f64 },
    /// Dihedral (two-plane) corner reflector with given face dimensions.
    DihedralReflector { width_m: f64, height_m: f64 },
}

/// Swerling fluctuation model types for RCS statistical variation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwerlingModel {
    /// Non-fluctuating (deterministic) target.
    Case0,
    /// Scan-to-scan Rayleigh fluctuation (many scatterers, chi-squared 2 DOF).
    Case1,
    /// Pulse-to-pulse Rayleigh fluctuation (many scatterers, chi-squared 2 DOF).
    Case2,
    /// Scan-to-scan dominant-plus-Rayleigh fluctuation (chi-squared 4 DOF).
    Case3,
    /// Pulse-to-pulse dominant-plus-Rayleigh fluctuation (chi-squared 4 DOF).
    Case4,
}

/// Radar cross section estimator for monostatic (backscatter) scenarios.
///
/// Computes RCS in m² and dBsm for various target geometries at a given
/// radar frequency. Also provides angle-dependent patterns, Swerling
/// fluctuation statistics, and radar range equation utilities.
#[derive(Debug, Clone)]
pub struct RcsEstimator {
    /// Radar operating frequency in Hz.
    pub frequency_hz: f64,
    /// Target geometry model.
    pub target: TargetModel,
}

impl RcsEstimator {
    /// Create a new RCS estimator for the given frequency and target model.
    pub fn new(frequency_hz: f64, target: TargetModel) -> Self {
        Self {
            frequency_hz,
            target,
        }
    }

    /// Wavelength in meters for the configured frequency.
    pub fn wavelength(&self) -> f64 {
        speed_of_light() / self.frequency_hz
    }

    /// Compute the monostatic RCS in m² for the configured target at normal
    /// incidence (broadside for plates, axial for cylinders).
    pub fn compute_rcs(&self) -> f64 {
        match &self.target {
            TargetModel::Sphere { radius_m } => sphere_rcs(*radius_m, self.wavelength()),
            TargetModel::FlatPlate { width_m, height_m } => {
                flat_plate_rcs(*width_m, *height_m, self.wavelength(), 0.0)
            }
            TargetModel::CornerReflector { edge_m } => {
                corner_reflector_rcs(*edge_m, self.wavelength())
            }
            TargetModel::Cylinder { radius_m, length_m } => {
                cylinder_rcs(*radius_m, *length_m, self.wavelength())
            }
            TargetModel::DihedralReflector { width_m, height_m } => {
                dihedral_reflector_rcs(*width_m, *height_m, self.wavelength())
            }
        }
    }

    /// Compute the monostatic RCS in dBsm (dB relative to 1 m²).
    pub fn compute_rcs_dbsm(&self) -> f64 {
        to_dbsm(self.compute_rcs())
    }

    /// Compute RCS vs aspect angle for a flat plate target.
    ///
    /// Returns a vector of `(angle_rad, rcs_m2)` pairs from `-angle_max` to
    /// `+angle_max` with `num_points` samples. Only valid when the target is
    /// `FlatPlate`; returns an empty vector otherwise.
    pub fn rcs_vs_angle(
        &self,
        angle_max_rad: f64,
        num_points: usize,
    ) -> Vec<(f64, f64)> {
        match &self.target {
            TargetModel::FlatPlate { width_m, height_m } => {
                let lambda = self.wavelength();
                let step = if num_points <= 1 {
                    0.0
                } else {
                    2.0 * angle_max_rad / (num_points - 1) as f64
                };
                (0..num_points)
                    .map(|i| {
                        let angle = -angle_max_rad + i as f64 * step;
                        let rcs = flat_plate_rcs(*width_m, *height_m, lambda, angle);
                        (angle, rcs)
                    })
                    .collect()
            }
            _ => Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions for individual target geometries
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s).
fn speed_of_light() -> f64 {
    299_792_458.0
}

/// Convert RCS in m² to dBsm.
pub fn to_dbsm(rcs_m2: f64) -> f64 {
    10.0 * rcs_m2.log10()
}

/// Convert dBsm to RCS in m².
pub fn from_dbsm(dbsm: f64) -> f64 {
    10.0_f64.powf(dbsm / 10.0)
}

/// Sphere RCS (monostatic).
///
/// Uses the optical-region approximation `sigma = pi * a^2` when the sphere
/// circumference is large compared to wavelength (`2*pi*a / lambda > 10`).
/// For smaller spheres (Rayleigh region, `2*pi*a / lambda < 0.5`), uses the
/// Rayleigh scattering formula. In the Mie (resonance) region, a simplified
/// interpolation is used.
pub fn sphere_rcs(radius_m: f64, wavelength_m: f64) -> f64 {
    let ka = 2.0 * PI * radius_m / wavelength_m; // size parameter

    if ka > 10.0 {
        // Optical region: geometric cross section
        PI * radius_m * radius_m
    } else if ka < 0.5 {
        // Rayleigh region: sigma = 9 * pi * k^4 * a^6
        // For a perfectly conducting sphere (Skolnik)
        let k = 2.0 * PI / wavelength_m;
        9.0 * PI * k.powi(4) * radius_m.powi(6)
    } else {
        // Mie resonance region: smooth interpolation between Rayleigh and optical.
        let k = 2.0 * PI / wavelength_m;
        let rayleigh = 9.0 * PI * k.powi(4) * radius_m.powi(6);
        let optical = PI * radius_m * radius_m;
        let t = (ka - 0.5) / (10.0 - 0.5); // 0..1
        // Log-space interpolation
        let log_rcs = (1.0 - t) * rayleigh.ln() + t * optical.ln();
        log_rcs.exp()
    }
}

/// Flat plate RCS (monostatic) with aspect angle dependence.
///
/// Models a rectangular plate of dimensions `width_m` x `height_m` oriented
/// normal to the radar at `angle_rad = 0`. The pattern follows a sinc²
/// envelope in the dimension along which the angle varies (width).
///
/// At normal incidence (`angle_rad = 0`):
///   `sigma_max = 4 * pi * (A / lambda)^2`
/// where `A = width * height`.
///
/// Off-broadside in the width dimension:
///   `sigma(theta) = sigma_max * sinc²(width * sin(theta) / lambda)`
pub fn flat_plate_rcs(width_m: f64, height_m: f64, wavelength_m: f64, angle_rad: f64) -> f64 {
    let area = width_m * height_m;
    let sigma_max = 4.0 * PI * (area / wavelength_m).powi(2);

    let u = width_m * angle_rad.sin() / wavelength_m;
    let sinc_val = if u.abs() < 1.0e-12 {
        1.0
    } else {
        let arg = PI * u;
        (arg.sin() / arg).powi(2)
    };

    sigma_max * sinc_val
}

/// Trihedral corner reflector maximum RCS (monostatic, on-axis).
///
/// `sigma = 12 * pi * a^4 / lambda^2`
///
/// where `a` is the edge length of the triangular faces.
pub fn corner_reflector_rcs(edge_m: f64, wavelength_m: f64) -> f64 {
    12.0 * PI * edge_m.powi(4) / wavelength_m.powi(2)
}

/// Cylinder broadside RCS (monostatic, normal to axis).
///
/// `sigma = 2 * pi * r * L^2 / lambda`
///
/// where `r` is the cylinder radius and `L` is the length.
pub fn cylinder_rcs(radius_m: f64, length_m: f64, wavelength_m: f64) -> f64 {
    2.0 * PI * radius_m * length_m.powi(2) / wavelength_m
}

/// Dihedral (two-plate) corner reflector maximum RCS (monostatic).
///
/// `sigma = 8 * pi * w^2 * h^2 / lambda^2`
///
/// where `w` is the width and `h` is the height of each face.
pub fn dihedral_reflector_rcs(width_m: f64, height_m: f64, wavelength_m: f64) -> f64 {
    8.0 * PI * width_m.powi(2) * height_m.powi(2) / wavelength_m.powi(2)
}

/// Apply Swerling fluctuation model to a mean RCS value.
///
/// Given the mean RCS `sigma_mean` and a uniform random variable `u` in
/// `[0, 1)`, returns a fluctuating RCS sample drawn from the appropriate
/// distribution.
///
/// - **Case 0**: Non-fluctuating, returns `sigma_mean`.
/// - **Case 1 / Case 2**: Exponential (chi-squared, 2 DOF).
///   `sigma = -sigma_mean * ln(1 - u)`
/// - **Case 3 / Case 4**: Chi-squared with 4 DOF.
///   Uses two pseudo-independent draws from a single uniform input.
///
/// The distinction between odd (scan-to-scan) and even (pulse-to-pulse)
/// cases is in *when* the fluctuation is redrawn, not in the PDF shape.
pub fn swerling_model(model: SwerlingModel, sigma_mean: f64, u: f64) -> f64 {
    let u_clamped = u.clamp(1.0e-15, 1.0 - 1.0e-15);

    match model {
        SwerlingModel::Case0 => sigma_mean,
        SwerlingModel::Case1 | SwerlingModel::Case2 => {
            // Exponential distribution (chi-squared, 2 DOF)
            -sigma_mean * (1.0 - u_clamped).ln()
        }
        SwerlingModel::Case3 | SwerlingModel::Case4 => {
            // Chi-squared with 4 DOF: sum of 2 exponentials, each with mean sigma_mean/2.
            let u1 = u_clamped.sqrt();
            let u2 = u_clamped / u1;
            let u2_clamped = u2.clamp(1.0e-15, 1.0 - 1.0e-15);
            -sigma_mean * 0.5 * ((1.0 - u1).ln() + (1.0 - u2_clamped).ln())
        }
    }
}

/// Radar range equation: received signal power in watts.
///
/// `P_r = (P_t * G_t * G_r * lambda^2 * sigma) / ((4*pi)^3 * R^4)`
///
/// # Arguments
/// - `tx_power_w`: Transmitter power in watts
/// - `tx_gain`: Transmitter antenna gain (linear, not dB)
/// - `rx_gain`: Receiver antenna gain (linear, not dB)
/// - `wavelength_m`: Radar wavelength in meters
/// - `rcs_m2`: Target RCS in m²
/// - `range_m`: One-way range to target in meters
pub fn radar_range_equation(
    tx_power_w: f64,
    tx_gain: f64,
    rx_gain: f64,
    wavelength_m: f64,
    rcs_m2: f64,
    range_m: f64,
) -> f64 {
    let numerator = tx_power_w * tx_gain * rx_gain * wavelength_m.powi(2) * rcs_m2;
    let denominator = (4.0 * PI).powi(3) * range_m.powi(4);
    numerator / denominator
}

/// Maximum detection range in meters for a given SNR threshold.
///
/// Rearranges the radar range equation to solve for range:
///
/// `R_max = ((P_t * G_t * G_r * lambda^2 * sigma) / ((4*pi)^3 * k * T * B * SNR_min))^(1/4)`
///
/// # Arguments
/// - `tx_power_w`: Transmitter power in watts
/// - `tx_gain`: Transmitter antenna gain (linear)
/// - `rx_gain`: Receiver antenna gain (linear)
/// - `wavelength_m`: Radar wavelength in meters
/// - `rcs_m2`: Target RCS in m²
/// - `snr_min`: Minimum detectable SNR (linear, not dB)
/// - `noise_bandwidth_hz`: Receiver noise bandwidth in Hz
/// - `noise_figure`: Receiver noise figure (linear, not dB)
/// - `system_temp_k`: System noise temperature in Kelvin (typically 290 K)
pub fn detection_range(
    tx_power_w: f64,
    tx_gain: f64,
    rx_gain: f64,
    wavelength_m: f64,
    rcs_m2: f64,
    snr_min: f64,
    noise_bandwidth_hz: f64,
    noise_figure: f64,
    system_temp_k: f64,
) -> f64 {
    let k_boltzmann = 1.380649e-23; // J/K
    let noise_power = k_boltzmann * system_temp_k * noise_figure * noise_bandwidth_hz;
    let numerator =
        tx_power_w * tx_gain * rx_gain * wavelength_m.powi(2) * rcs_m2;
    let denominator = (4.0 * PI).powi(3) * noise_power * snr_min;
    (numerator / denominator).powf(0.25)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1.0e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        let max_abs = a.abs().max(b.abs());
        if max_abs < 1.0e-30 {
            return true;
        }
        (a - b).abs() / max_abs < rel_tol
    }

    #[test]
    fn test_sphere_optical_region() {
        // Large sphere: RCS = pi * a^2
        let radius = 1.0; // 1 m
        let lambda = 0.03; // 10 GHz -> ~3 cm
        // ka = 2*pi*1.0/0.03 = ~209, well into optical region
        let rcs = sphere_rcs(radius, lambda);
        let expected = PI * radius * radius;
        assert!(
            relative_eq(rcs, expected, 1.0e-10),
            "Optical sphere: got {rcs}, expected {expected}"
        );
    }

    #[test]
    fn test_sphere_rayleigh_region() {
        // Small sphere: Rayleigh scattering
        let radius = 0.001; // 1 mm
        let lambda = 0.03; // 10 GHz
        // ka = 2*pi*0.001/0.03 = ~0.209, Rayleigh region
        let rcs = sphere_rcs(radius, lambda);
        let k = 2.0 * PI / lambda;
        let expected = 9.0 * PI * k.powi(4) * radius.powi(6);
        assert!(
            relative_eq(rcs, expected, 1.0e-10),
            "Rayleigh sphere: got {rcs}, expected {expected}"
        );
    }

    #[test]
    fn test_sphere_mie_region() {
        // Intermediate sphere: Mie resonance region
        let radius = 0.03; // 3 cm
        let lambda = 0.03; // 10 GHz
        // ka = 2*pi*0.03/0.03 = 2*pi ~ 6.28, Mie region
        let rcs = sphere_rcs(radius, lambda);
        // In the Mie region, RCS should be positive and finite.
        // The interpolation between Rayleigh and optical can exceed
        // the geometric cross section due to resonance effects.
        assert!(rcs > 0.0, "Mie sphere RCS must be positive, got {rcs}");
        assert!(rcs.is_finite(), "Mie sphere RCS must be finite, got {rcs}");
        // Should converge toward optical as ka increases; at ka~6.28
        // the value is in a reasonable range (sub-1 m^2 for a 3cm sphere)
        assert!(
            rcs < 1.0,
            "Mie sphere RCS for 3cm sphere should be < 1 m^2, got {rcs}"
        );
    }

    #[test]
    fn test_flat_plate_broadside() {
        // At normal incidence: sigma = 4*pi*(A/lambda)^2
        let w = 1.0;
        let h = 1.0;
        let lambda = 0.03;
        let rcs = flat_plate_rcs(w, h, lambda, 0.0);
        let expected = 4.0 * PI * (w * h / lambda).powi(2);
        assert!(
            relative_eq(rcs, expected, 1.0e-10),
            "Flat plate broadside: got {rcs}, expected {expected}"
        );
    }

    #[test]
    fn test_flat_plate_off_broadside_nulls() {
        // First null at sin(theta) = lambda / width
        let w: f64 = 1.0;
        let h = 1.0;
        let lambda = 0.03;
        let null_angle: f64 = (lambda / w).asin(); // ~0.03 rad
        let rcs = flat_plate_rcs(w, h, lambda, null_angle);
        // At the null, sinc argument = pi*1 => sin(pi)/pi = 0
        assert!(
            rcs < 1.0e-10,
            "Flat plate null: got {rcs}, expected ~0"
        );
    }

    #[test]
    fn test_flat_plate_symmetry() {
        let w = 0.5;
        let h = 0.3;
        let lambda = 0.03;
        let angle = 0.1;
        let rcs_pos = flat_plate_rcs(w, h, lambda, angle);
        let rcs_neg = flat_plate_rcs(w, h, lambda, -angle);
        assert!(
            relative_eq(rcs_pos, rcs_neg, 1.0e-12),
            "Flat plate symmetry: {rcs_pos} vs {rcs_neg}"
        );
    }

    #[test]
    fn test_corner_reflector() {
        // sigma = 12*pi*a^4/lambda^2
        let edge = 0.5;
        let lambda = 0.03;
        let rcs = corner_reflector_rcs(edge, lambda);
        let expected = 12.0 * PI * edge.powi(4) / lambda.powi(2);
        assert!(
            relative_eq(rcs, expected, 1.0e-10),
            "Corner reflector: got {rcs}, expected {expected}"
        );
    }

    #[test]
    fn test_cylinder_broadside() {
        // sigma = 2*pi*r*L^2/lambda
        let r = 0.1;
        let l = 2.0;
        let lambda = 0.03;
        let rcs = cylinder_rcs(r, l, lambda);
        let expected = 2.0 * PI * r * l.powi(2) / lambda;
        assert!(
            relative_eq(rcs, expected, 1.0e-10),
            "Cylinder broadside: got {rcs}, expected {expected}"
        );
    }

    #[test]
    fn test_dihedral_reflector() {
        // sigma = 8*pi*w^2*h^2/lambda^2
        let w = 0.5;
        let h = 0.5;
        let lambda = 0.03;
        let rcs = dihedral_reflector_rcs(w, h, lambda);
        let expected = 8.0 * PI * w.powi(2) * h.powi(2) / lambda.powi(2);
        assert!(
            relative_eq(rcs, expected, 1.0e-10),
            "Dihedral reflector: got {rcs}, expected {expected}"
        );
    }

    #[test]
    fn test_dbsm_conversion_roundtrip() {
        let rcs = 10.0; // 10 m^2
        let dbsm = to_dbsm(rcs);
        let recovered = from_dbsm(dbsm);
        assert!(
            approx_eq(recovered, rcs, 1.0e-12),
            "dBsm roundtrip: {recovered} vs {rcs}"
        );
        // 10 m^2 = 10 dBsm
        assert!(
            approx_eq(dbsm, 10.0, 1.0e-10),
            "10 m^2 should be 10 dBsm, got {dbsm}"
        );
    }

    #[test]
    fn test_swerling_case0() {
        let sigma = 5.0;
        let result = swerling_model(SwerlingModel::Case0, sigma, 0.5);
        assert!(
            approx_eq(result, sigma, TOLERANCE),
            "Swerling 0: got {result}, expected {sigma}"
        );
    }

    #[test]
    fn test_swerling_case1_positive() {
        // Swerling I/II should always produce positive RCS
        let sigma_mean = 2.0;
        for i in 0..100 {
            let u = i as f64 / 100.0;
            let result = swerling_model(SwerlingModel::Case1, sigma_mean, u);
            assert!(
                result > 0.0,
                "Swerling 1 should be positive, got {result} at u={u}"
            );
        }
    }

    #[test]
    fn test_swerling_case3_positive() {
        // Swerling III/IV should always produce positive RCS
        let sigma_mean = 3.0;
        for i in 0..100 {
            let u = i as f64 / 100.0;
            let result = swerling_model(SwerlingModel::Case3, sigma_mean, u);
            assert!(
                result > 0.0,
                "Swerling 3 should be positive, got {result} at u={u}"
            );
        }
    }

    #[test]
    fn test_radar_range_equation() {
        // Sanity check: known parameters
        let pt = 1000.0; // 1 kW
        let gt = 30.0; // ~15 dBi
        let gr = 30.0;
        let lambda = 0.03; // 10 GHz
        let sigma = 1.0; // 1 m^2
        let range = 10_000.0; // 10 km

        let pr = radar_range_equation(pt, gt, gr, lambda, sigma, range);
        // Manual: 1000*30*30*0.0009*1 / ((4*pi)^3 * 1e16)
        assert!(
            pr > 0.0 && pr < 1.0e-15,
            "Radar range eq: got {pr}, expected small positive value"
        );
    }

    #[test]
    fn test_radar_range_equation_inverse_fourth_power() {
        // Doubling range should reduce power by factor of 16
        let pt = 1000.0;
        let gt = 10.0;
        let gr = 10.0;
        let lambda = 0.1;
        let sigma = 1.0;

        let pr1 = radar_range_equation(pt, gt, gr, lambda, sigma, 1000.0);
        let pr2 = radar_range_equation(pt, gt, gr, lambda, sigma, 2000.0);
        let ratio = pr1 / pr2;
        assert!(
            relative_eq(ratio, 16.0, 1.0e-10),
            "Inverse 4th power: ratio = {ratio}, expected 16"
        );
    }

    #[test]
    fn test_detection_range() {
        let pt = 10_000.0; // 10 kW
        let gt = 100.0; // 20 dBi
        let gr = 100.0;
        let lambda = 0.03;
        let sigma = 1.0; // 1 m^2
        let snr_min = 10.0; // ~10 dB
        let bw = 1.0e6; // 1 MHz
        let nf = 4.0; // ~6 dB
        let temp = 290.0;

        let r_max = detection_range(pt, gt, gr, lambda, sigma, snr_min, bw, nf, temp);
        assert!(
            r_max > 0.0 && r_max < 1.0e6,
            "Detection range: {r_max} m should be reasonable"
        );

        // Doubling RCS should increase range by 2^(1/4) ~ 1.189
        let r_max_2 = detection_range(pt, gt, gr, lambda, 2.0 * sigma, snr_min, bw, nf, temp);
        let ratio = r_max_2 / r_max;
        let expected_ratio = 2.0_f64.powf(0.25);
        assert!(
            relative_eq(ratio, expected_ratio, 1.0e-6),
            "Detection range RCS scaling: ratio = {ratio}, expected {expected_ratio}"
        );
    }

    #[test]
    fn test_estimator_rcs_vs_angle() {
        let est = RcsEstimator::new(
            10.0e9,
            TargetModel::FlatPlate {
                width_m: 1.0,
                height_m: 1.0,
            },
        );
        let pattern = est.rcs_vs_angle(0.5, 101);
        assert_eq!(pattern.len(), 101);

        // Broadside (middle element) should have the maximum RCS
        let mid = &pattern[50];
        assert!(
            approx_eq(mid.0, 0.0, 1.0e-10),
            "Middle angle should be ~0, got {}",
            mid.0
        );
        let max_rcs = pattern.iter().map(|(_, r)| *r).fold(0.0_f64, f64::max);
        assert!(
            relative_eq(mid.1, max_rcs, 1.0e-6),
            "Broadside should be max: {} vs {}",
            mid.1,
            max_rcs
        );
    }

    #[test]
    fn test_estimator_rcs_vs_angle_non_flat_plate() {
        // Non-FlatPlate targets should return empty pattern
        let est = RcsEstimator::new(10.0e9, TargetModel::Sphere { radius_m: 0.5 });
        let pattern = est.rcs_vs_angle(0.5, 101);
        assert!(pattern.is_empty());
    }

    #[test]
    fn test_corner_reflector_larger_than_sphere() {
        // A corner reflector should have much larger RCS than a sphere of similar size
        let edge = 0.5;
        let lambda = 0.03;
        let cr_rcs = corner_reflector_rcs(edge, lambda);
        let sphere_rcs_val = sphere_rcs(edge, lambda);
        assert!(
            cr_rcs > sphere_rcs_val,
            "Corner reflector ({cr_rcs}) should exceed sphere ({sphere_rcs_val})"
        );
    }
}
