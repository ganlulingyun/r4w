//! Soil moisture and salinity estimation from impedance/capacitance measurements
//! for precision agriculture.
//!
//! This module provides algorithms for converting raw sensor measurements
//! (impedance, capacitance, dielectric constant) into agronomically useful
//! quantities such as volumetric water content (VWC), electrical conductivity
//! (EC), and salinity. It includes the Topp equation, Hilhorst model for bulk
//! EC extraction, temperature compensation, depth profiling, data quality
//! assessment, and field mapping via inverse distance weighting (IDW).
//!
//! # Example
//!
//! ```
//! use r4w_core::precision_ag_soil_sensor::{
//!     SoilConfig, SoilSensor, topp_equation, ec_to_salinity_ppm,
//! };
//!
//! // Estimate VWC from a dielectric constant of 15.0
//! let vwc = topp_equation(15.0);
//! assert!(vwc > 20.0 && vwc < 40.0, "VWC should be in a reasonable range");
//!
//! // Convert EC to salinity
//! let salinity = ec_to_salinity_ppm(2.0);
//! assert!(salinity > 1000.0);
//!
//! // Use the sensor struct for a full reading pipeline
//! let config = SoilConfig {
//!     sensor_freq_hz: 70e6,
//!     probe_length_m: 0.30,
//!     probe_spacing_m: 0.03,
//!     temperature_c: 25.0,
//! };
//! let sensor = SoilSensor::new(config);
//! let reading = sensor.read_impedance(150.0, -50.0, 0.005);
//! assert!(reading.vwc_pct >= 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & data types
// ---------------------------------------------------------------------------

/// Sensor configuration parameters.
#[derive(Debug, Clone)]
pub struct SoilConfig {
    /// Operating frequency of the capacitance/impedance sensor in Hz.
    pub sensor_freq_hz: f64,
    /// Length of the sensor probe in metres.
    pub probe_length_m: f64,
    /// Spacing between probe rods in metres.
    pub probe_spacing_m: f64,
    /// Ambient soil temperature in degrees Celsius.
    pub temperature_c: f64,
}

impl Default for SoilConfig {
    fn default() -> Self {
        Self {
            sensor_freq_hz: 70e6,
            probe_length_m: 0.30,
            probe_spacing_m: 0.03,
            temperature_c: 25.0,
        }
    }
}

/// A single soil sensor reading with derived quantities and quality score.
#[derive(Debug, Clone)]
pub struct SoilReading {
    /// Real part of the relative dielectric constant (permittivity).
    pub dielectric_constant: f64,
    /// Bulk electrical conductivity in dS/m.
    pub ec_dsm: f64,
    /// Volumetric water content as a percentage (0-100).
    pub vwc_pct: f64,
    /// Soil temperature at the time of reading in degrees Celsius.
    pub temperature_c: f64,
    /// Data quality score in the range [0, 1].
    pub quality: f64,
}

// ---------------------------------------------------------------------------
// Core equations
// ---------------------------------------------------------------------------

/// Topp equation -- estimate volumetric water content (%) from the real part
/// of the soil dielectric constant.
///
/// Reference: Topp, Davis & Annan (1980).
///
/// ```
/// use r4w_core::precision_ag_soil_sensor::topp_equation;
/// let vwc = topp_equation(3.0); // dry soil
/// assert!(vwc < 10.0);
/// ```
pub fn topp_equation(dielectric_constant: f64) -> f64 {
    let k = dielectric_constant;
    // Polynomial coefficients from Topp et al. (1980)
    let theta = -5.3e-2 + 2.92e-2 * k - 5.5e-4 * k * k + 4.3e-6 * k * k * k;
    // Result is in m^3/m^3; convert to percentage
    (theta * 100.0).clamp(0.0, 100.0)
}

/// Inverse Topp equation -- estimate dielectric constant from VWC (%).
///
/// Uses Newton-Raphson iteration to invert [`topp_equation`].
pub fn inverse_topp(vwc_pct: f64) -> f64 {
    let target = vwc_pct / 100.0; // m^3/m^3
    // Initial guess via simple linear approximation
    let mut k = 1.0 + vwc_pct * 0.5;
    for _ in 0..50 {
        let theta = -5.3e-2 + 2.92e-2 * k - 5.5e-4 * k * k + 4.3e-6 * k * k * k;
        let dtheta = 2.92e-2 - 2.0 * 5.5e-4 * k + 3.0 * 4.3e-6 * k * k;
        if dtheta.abs() < 1e-30 {
            break;
        }
        let step = (theta - target) / dtheta;
        k -= step;
        if step.abs() < 1e-12 {
            break;
        }
    }
    k.max(1.0)
}

/// Convert electrical conductivity (dS/m) to approximate total dissolved
/// solids (ppm / mg per litre).
///
/// Uses the common empirical factor of 640 ppm per dS/m.
pub fn ec_to_salinity_ppm(ec_dsm: f64) -> f64 {
    ec_dsm * 640.0
}

/// Temperature-compensate an EC measurement to a reference temperature.
///
/// Uses a linear temperature coefficient of 2% per degree C, which is standard
/// for most soil solutions.
pub fn temperature_compensate_ec(ec_dsm: f64, temp_c: f64, ref_temp_c: f64) -> f64 {
    let alpha = 0.02; // 2% per degree C
    ec_dsm / (1.0 + alpha * (temp_c - ref_temp_c))
}

/// Hilhorst model -- extract bulk soil EC from the complex dielectric
/// measurement at a given frequency.
///
/// Reference: Hilhorst (2000).
///
/// * `dielectric` -- real part of relative permittivity (epsilon_r').
/// * `imaginary_dielectric` -- imaginary part of relative permittivity (epsilon_r'').
/// * `freq_hz` -- measurement frequency in Hz.
///
/// Returns EC in dS/m.
pub fn hilhorst_bulk_ec(dielectric: f64, imaginary_dielectric: f64, freq_hz: f64) -> f64 {
    let epsilon_0 = 8.854187817e-12; // F/m
    let omega = 2.0 * PI * freq_hz;
    // sigma = epsilon_0 * omega * epsilon_r''
    let sigma = epsilon_0 * omega * imaginary_dielectric.abs();
    // Offset dielectric of bound water (~4.1 for most mineral soils)
    let epsilon_offset = 4.1;
    if dielectric <= epsilon_offset {
        return 0.0;
    }
    // Hilhorst relation: EC_bulk = sigma * (epsilon_r' - epsilon_offset) / epsilon_r'
    sigma * (dielectric - epsilon_offset) / dielectric
}

/// Convert complex impedance to complex relative permittivity.
///
/// * `z_real` -- real part of impedance (ohms).
/// * `z_imag` -- imaginary part of impedance (ohms).
/// * `freq_hz` -- measurement frequency (Hz).
/// * `cell_constant` -- geometric cell constant of the probe (1/m).
///
/// Returns `(epsilon_real, epsilon_imag)`.
pub fn impedance_to_dielectric(
    z_real: f64,
    z_imag: f64,
    freq_hz: f64,
    cell_constant: f64,
) -> (f64, f64) {
    let epsilon_0 = 8.854187817e-12; // F/m
    let omega = 2.0 * PI * freq_hz;
    let z_mag_sq = z_real * z_real + z_imag * z_imag;
    if z_mag_sq < 1e-30 {
        return (1.0, 0.0);
    }
    // Y = 1/Z  (admittance)
    let y_real = z_real / z_mag_sq;
    let y_imag = -z_imag / z_mag_sq;

    // Capacitance model: Y = G + j*omega*C -> C = y_imag / omega, G = y_real
    // epsilon* = C / (epsilon_0 * cell_constant)  (complex)
    let denom = epsilon_0 * cell_constant * omega;
    if denom.abs() < 1e-30 {
        return (1.0, 0.0);
    }
    let epsilon_real = y_imag / denom;
    let epsilon_imag = y_real / denom;
    (epsilon_real.max(1.0), epsilon_imag.max(0.0))
}

// ---------------------------------------------------------------------------
// Depth profiling & field mapping
// ---------------------------------------------------------------------------

/// Build a depth-vs-VWC profile from a set of (depth_m, [`SoilReading`]) pairs.
///
/// Returns `(depth_m, vwc_pct)` sorted by depth (ascending).
pub fn depth_profile(readings: &[(f64, SoilReading)]) -> Vec<(f64, f64)> {
    let mut profile: Vec<(f64, f64)> = readings
        .iter()
        .map(|(d, r)| (*d, r.vwc_pct))
        .collect();
    profile.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    profile
}

/// Inverse Distance Weighting (IDW) interpolation for field mapping.
///
/// * `points` -- known sample locations and values: `(x, y, value)`.
/// * `query` -- the `(x, y)` location to estimate.
/// * `power` -- distance exponent (commonly 2).
///
/// Returns the interpolated value.
pub fn field_map_idw(points: &[(f64, f64, f64)], query: (f64, f64), power: f64) -> f64 {
    if points.is_empty() {
        return 0.0;
    }
    let mut weight_sum = 0.0_f64;
    let mut value_sum = 0.0_f64;

    for &(px, py, val) in points {
        let dx = px - query.0;
        let dy = py - query.1;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist < 1e-12 {
            // Query point coincides with a known point
            return val;
        }
        let w = 1.0 / dist.powf(power);
        weight_sum += w;
        value_sum += w * val;
    }
    if weight_sum.abs() < 1e-30 {
        return 0.0;
    }
    value_sum / weight_sum
}

// ---------------------------------------------------------------------------
// Quality assessment
// ---------------------------------------------------------------------------

/// Assess data quality of a [`SoilReading`] based on physical plausibility.
///
/// Returns a score in [0, 1] where 1 is the highest quality.
pub fn data_quality_check(reading: &SoilReading) -> f64 {
    let mut score = 1.0_f64;

    // Dielectric constant should be in [1, 80] for soils
    if reading.dielectric_constant < 1.0 || reading.dielectric_constant > 80.0 {
        score -= 0.4;
    }
    // VWC should be in [0, 100]
    if reading.vwc_pct < 0.0 || reading.vwc_pct > 100.0 {
        score -= 0.3;
    }
    // EC should be non-negative and below ~20 dS/m for most soils
    if reading.ec_dsm < 0.0 {
        score -= 0.3;
    } else if reading.ec_dsm > 20.0 {
        score -= 0.15;
    }
    // Temperature plausibility (-40 to +60 deg C)
    if reading.temperature_c < -40.0 || reading.temperature_c > 60.0 {
        score -= 0.2;
    }
    score.clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Agronomic estimates
// ---------------------------------------------------------------------------

/// Estimate the permanent wilting point (VWC %) from soil clay content (%).
///
/// Uses the Rawls (1982) pedotransfer approximation.
pub fn wilting_point_estimate(clay_pct: f64) -> f64 {
    // theta_wp ~ 0.026 + 0.005 * clay% + 0.0001 * clay%^2  (m^3/m^3)
    let theta = 0.026 + 0.005 * clay_pct + 0.0001 * clay_pct * clay_pct;
    (theta * 100.0).clamp(0.0, 100.0)
}

/// Estimate field capacity (VWC %) from soil clay content (%).
///
/// Uses the Rawls (1982) pedotransfer approximation.
pub fn field_capacity_estimate(clay_pct: f64) -> f64 {
    // theta_fc ~ 0.10 + 0.008 * clay% + 0.00012 * clay%^2  (m^3/m^3)
    let theta = 0.10 + 0.008 * clay_pct + 0.00012 * clay_pct * clay_pct;
    (theta * 100.0).clamp(0.0, 100.0)
}

// ---------------------------------------------------------------------------
// SoilSensor -- high-level API
// ---------------------------------------------------------------------------

/// High-level soil sensor that converts raw impedance measurements into
/// [`SoilReading`]s.
#[derive(Debug, Clone)]
pub struct SoilSensor {
    config: SoilConfig,
}

impl SoilSensor {
    /// Create a new sensor with the given configuration.
    pub fn new(config: SoilConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &SoilConfig {
        &self.config
    }

    /// Compute the geometric cell constant for the parallel-rod probe.
    ///
    /// Approximation: `K = ln(s/a) / (pi * L)` where `s` is spacing, `a` is
    /// the effective rod radius (~1 mm), and `L` is probe length.
    fn cell_constant(&self) -> f64 {
        let a = 0.001; // 1 mm rod radius
        let s = self.config.probe_spacing_m;
        let l = self.config.probe_length_m;
        if s <= a || l <= 0.0 {
            return 1.0;
        }
        (s / a).ln() / (PI * l)
    }

    /// Produce a [`SoilReading`] from raw impedance values and a conductance
    /// measurement.
    ///
    /// * `z_real` -- real impedance in ohms.
    /// * `z_imag` -- imaginary impedance in ohms.
    /// * `conductance_sm` -- measured bulk conductance in S/m (used for EC).
    pub fn read_impedance(&self, z_real: f64, z_imag: f64, conductance_sm: f64) -> SoilReading {
        let cell_k = self.cell_constant();
        let (eps_r, eps_i) = impedance_to_dielectric(
            z_real,
            z_imag,
            self.config.sensor_freq_hz,
            cell_k,
        );
        let vwc = topp_equation(eps_r);
        // EC from Hilhorst model
        let ec_hilhorst = hilhorst_bulk_ec(eps_r, eps_i, self.config.sensor_freq_hz);
        // Also derive EC directly from conductance: sigma / K
        let ec_direct = if cell_k > 0.0 {
            conductance_sm / cell_k / 10.0 // convert S/m to dS/m
        } else {
            0.0
        };
        // Average the two estimates
        let ec_avg = (ec_hilhorst + ec_direct) / 2.0;
        let ec_comp = temperature_compensate_ec(
            ec_avg,
            self.config.temperature_c,
            25.0,
        );

        let mut reading = SoilReading {
            dielectric_constant: eps_r,
            ec_dsm: ec_comp,
            vwc_pct: vwc,
            temperature_c: self.config.temperature_c,
            quality: 0.0,
        };
        reading.quality = data_quality_check(&reading);
        reading
    }

    /// Multi-frequency impedance analysis.
    ///
    /// Accepts impedance readings at several frequencies and returns
    /// `(freq_hz, dielectric_real, dielectric_imag)` tuples for each.
    pub fn multi_frequency_analysis(
        &self,
        measurements: &[(f64, f64, f64)], // (freq_hz, z_real, z_imag)
    ) -> Vec<(f64, f64, f64)> {
        let cell_k = self.cell_constant();
        measurements
            .iter()
            .map(|&(freq, zr, zi)| {
                let (er, ei) = impedance_to_dielectric(zr, zi, freq, cell_k);
                (freq, er, ei)
            })
            .collect()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Helpers
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- topp_equation tests ---

    #[test]
    fn test_topp_dry_soil() {
        // Air-dry soil epsilon ~ 3-5 -> VWC should be low
        let vwc = topp_equation(3.5);
        assert!(vwc < 10.0, "Dry soil VWC too high: {vwc}");
        assert!(vwc >= 0.0, "VWC should be non-negative: {vwc}");
    }

    #[test]
    fn test_topp_saturated_soil() {
        // Saturated soil epsilon ~ 30-40 -> VWC should be high
        let vwc = topp_equation(35.0);
        assert!(vwc > 30.0, "Saturated VWC too low: {vwc}");
        assert!(vwc < 80.0, "Saturated VWC unreasonably high: {vwc}");
    }

    #[test]
    fn test_topp_water() {
        // Pure water epsilon ~ 80 -> VWC clamped to 100
        let vwc = topp_equation(80.0);
        assert!(vwc <= 100.0);
    }

    #[test]
    fn test_topp_monotonic() {
        // VWC should increase with dielectric constant in the soil range
        let mut prev = topp_equation(1.0);
        for k in 2..50 {
            let current = topp_equation(k as f64);
            assert!(
                current >= prev,
                "Topp not monotonic at k={k}: {prev} > {current}"
            );
            prev = current;
        }
    }

    // --- inverse_topp tests ---

    #[test]
    fn test_inverse_topp_roundtrip() {
        for &vwc in &[5.0, 15.0, 25.0, 35.0, 45.0] {
            let k = inverse_topp(vwc);
            let recovered = topp_equation(k);
            assert!(
                approx_eq(recovered, vwc, 0.5),
                "Roundtrip failed for VWC={vwc}: got {recovered} (k={k})"
            );
        }
    }

    #[test]
    fn test_inverse_topp_minimum() {
        // Very low VWC should give a small dielectric constant
        let k = inverse_topp(0.0);
        assert!(k >= 1.0 && k < 5.0, "k for VWC=0 should be small: {k}");
    }

    // --- ec_to_salinity_ppm tests ---

    #[test]
    fn test_ec_to_salinity_basic() {
        let ppm = ec_to_salinity_ppm(1.0);
        assert!(
            approx_eq(ppm, 640.0, 1.0),
            "1 dS/m should be ~640 ppm: {ppm}"
        );
    }

    #[test]
    fn test_ec_to_salinity_zero() {
        assert_eq!(ec_to_salinity_ppm(0.0), 0.0);
    }

    // --- temperature_compensate_ec tests ---

    #[test]
    fn test_temp_comp_at_reference() {
        // At reference temperature, compensation should be identity
        let ec = 2.5;
        let comp = temperature_compensate_ec(ec, 25.0, 25.0);
        assert!(approx_eq(comp, ec, 1e-10));
    }

    #[test]
    fn test_temp_comp_warmer() {
        // Warmer -> measured EC is higher -> corrected should be lower
        let ec = 3.0;
        let comp = temperature_compensate_ec(ec, 30.0, 25.0);
        assert!(comp < ec, "Warm-corrected should be lower: {comp}");
    }

    #[test]
    fn test_temp_comp_cooler() {
        // Cooler -> corrected should be higher
        let ec = 3.0;
        let comp = temperature_compensate_ec(ec, 20.0, 25.0);
        assert!(comp > ec, "Cool-corrected should be higher: {comp}");
    }

    // --- hilhorst_bulk_ec tests ---

    #[test]
    fn test_hilhorst_low_dielectric() {
        // Below offset (4.1), EC should be 0
        let ec = hilhorst_bulk_ec(3.0, 1.0, 70e6);
        assert_eq!(ec, 0.0);
    }

    #[test]
    fn test_hilhorst_positive() {
        let ec = hilhorst_bulk_ec(20.0, 5.0, 70e6);
        assert!(ec > 0.0, "EC should be positive for wet soil: {ec}");
    }

    // --- impedance_to_dielectric tests ---

    #[test]
    fn test_impedance_to_dielectric_basic() {
        let (er, ei) = impedance_to_dielectric(100.0, -50.0, 70e6, 10.0);
        assert!(er >= 1.0, "epsilon_r should be >= 1: {er}");
        assert!(ei >= 0.0, "epsilon_i should be >= 0: {ei}");
    }

    #[test]
    fn test_impedance_to_dielectric_zero_impedance() {
        let (er, ei) = impedance_to_dielectric(0.0, 0.0, 70e6, 10.0);
        assert_eq!(er, 1.0);
        assert_eq!(ei, 0.0);
    }

    // --- depth_profile tests ---

    #[test]
    fn test_depth_profile_sorting() {
        let readings = vec![
            (0.30, SoilReading {
                dielectric_constant: 20.0,
                ec_dsm: 1.0,
                vwc_pct: 25.0,
                temperature_c: 22.0,
                quality: 0.9,
            }),
            (0.10, SoilReading {
                dielectric_constant: 10.0,
                ec_dsm: 0.5,
                vwc_pct: 15.0,
                temperature_c: 22.0,
                quality: 0.95,
            }),
            (0.20, SoilReading {
                dielectric_constant: 15.0,
                ec_dsm: 0.8,
                vwc_pct: 20.0,
                temperature_c: 22.0,
                quality: 0.92,
            }),
        ];
        let profile = depth_profile(&readings);
        assert_eq!(profile.len(), 3);
        assert!(profile[0].0 < profile[1].0);
        assert!(profile[1].0 < profile[2].0);
        assert!(approx_eq(profile[0].1, 15.0, 1e-10));
    }

    // --- field_map_idw tests ---

    #[test]
    fn test_idw_at_known_point() {
        let points = vec![(0.0, 0.0, 10.0), (1.0, 0.0, 20.0)];
        let val = field_map_idw(&points, (0.0, 0.0), 2.0);
        assert!(approx_eq(val, 10.0, 1e-10));
    }

    #[test]
    fn test_idw_midpoint() {
        // Equidistant from two points with equal power -> should be average
        let points = vec![(0.0, 0.0, 10.0), (2.0, 0.0, 20.0)];
        let val = field_map_idw(&points, (1.0, 0.0), 2.0);
        assert!(approx_eq(val, 15.0, 1e-10));
    }

    #[test]
    fn test_idw_empty() {
        let val = field_map_idw(&[], (1.0, 1.0), 2.0);
        assert_eq!(val, 0.0);
    }

    // --- data_quality_check tests ---

    #[test]
    fn test_quality_good_reading() {
        let r = SoilReading {
            dielectric_constant: 15.0,
            ec_dsm: 2.0,
            vwc_pct: 25.0,
            temperature_c: 22.0,
            quality: 0.0,
        };
        let q = data_quality_check(&r);
        assert!(approx_eq(q, 1.0, 1e-10), "Good reading should be 1.0: {q}");
    }

    #[test]
    fn test_quality_bad_dielectric() {
        let r = SoilReading {
            dielectric_constant: -5.0,
            ec_dsm: 1.0,
            vwc_pct: 10.0,
            temperature_c: 22.0,
            quality: 0.0,
        };
        let q = data_quality_check(&r);
        assert!(q < 0.7, "Bad dielectric should lower quality: {q}");
    }

    // --- wilting_point / field_capacity tests ---

    #[test]
    fn test_wilting_point_sandy() {
        // Sandy soil ~5% clay -> low wilting point
        let wp = wilting_point_estimate(5.0);
        assert!(wp > 0.0 && wp < 15.0, "Sandy WP out of range: {wp}");
    }

    #[test]
    fn test_field_capacity_clay() {
        // Clay soil ~60% -> high field capacity
        let fc = field_capacity_estimate(60.0);
        assert!(fc > 30.0, "Clay FC should be high: {fc}");
    }

    #[test]
    fn test_fc_greater_than_wp() {
        // Field capacity should always exceed wilting point
        for clay in [5.0, 15.0, 30.0, 50.0, 70.0] {
            let wp = wilting_point_estimate(clay);
            let fc = field_capacity_estimate(clay);
            assert!(
                fc > wp,
                "FC ({fc}) should exceed WP ({wp}) at clay={clay}%"
            );
        }
    }

    // --- SoilSensor tests ---

    #[test]
    fn test_sensor_read_impedance() {
        let config = SoilConfig::default();
        let sensor = SoilSensor::new(config);
        let reading = sensor.read_impedance(150.0, -50.0, 0.005);
        assert!(reading.vwc_pct >= 0.0 && reading.vwc_pct <= 100.0);
        assert!(reading.quality >= 0.0 && reading.quality <= 1.0);
    }

    #[test]
    fn test_sensor_multi_frequency() {
        let config = SoilConfig::default();
        let sensor = SoilSensor::new(config);
        let measurements = vec![
            (10e6, 200.0, -80.0),
            (50e6, 180.0, -60.0),
            (100e6, 160.0, -40.0),
        ];
        let results = sensor.multi_frequency_analysis(&measurements);
        assert_eq!(results.len(), 3);
        for (freq, er, ei) in &results {
            assert!(*freq > 0.0);
            assert!(*er >= 1.0, "epsilon_r should be >= 1: {er}");
            assert!(*ei >= 0.0, "epsilon_i should be >= 0: {ei}");
        }
    }

    #[test]
    fn test_sensor_config_access() {
        let config = SoilConfig {
            sensor_freq_hz: 100e6,
            probe_length_m: 0.20,
            probe_spacing_m: 0.025,
            temperature_c: 18.0,
        };
        let sensor = SoilSensor::new(config);
        assert!(approx_eq(sensor.config().sensor_freq_hz, 100e6, 1e-3));
        assert!(approx_eq(sensor.config().temperature_c, 18.0, 1e-10));
    }
}
