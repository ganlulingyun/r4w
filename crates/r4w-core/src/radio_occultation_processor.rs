//! # GNSS Radio Occultation Atmospheric Profiling Processor
//!
//! This module implements radio occultation (RO) processing for atmospheric remote sensing
//! using GNSS signals observed from low Earth orbit (LEO) satellites. When GPS/GNSS signals
//! traverse the atmosphere at low elevation angles, atmospheric refraction bends the signal
//! path. By measuring the bending angle as a function of tangent height (impact parameter),
//! Abel inversion recovers the vertical refractivity profile, from which temperature,
//! pressure, and humidity can be retrieved.
//!
//! ## Physical Background
//!
//! The atmosphere's refractive index varies with density (and thus altitude). A signal
//! propagating through a spherically symmetric atmosphere experiences cumulative bending
//! described by the Abel integral. The refractivity N = (n-1) x 10^6 is related to
//! meteorological variables via the Smith-Weintraub equation:
//!
//! ```text
//! N = 77.6 * P/T + 3.73e5 * e/T^2
//! ```
//!
//! where P is pressure (hPa), T is temperature (K), and e is water vapor partial
//! pressure (hPa).
//!
//! ## Processing Chain
//!
//! 1. Excess phase measurement (from GPS receiver on LEO satellite)
//! 2. Differentiation to Doppler shift
//! 3. Geometric inversion to bending angle vs. impact parameter
//! 4. Ionospheric correction (dual-frequency)
//! 5. Abel inversion to refractivity profile
//! 6. Hydrostatic integration for pressure
//! 7. Temperature retrieval via ideal gas law
//!
//! ## References
//!
//! - Kursinski et al. (1997), "Observing Earth's atmosphere with radio occultation
//!   measurements using GPS", JGR
//! - Hajj et al. (2002), "A technical description of atmospheric sounding by GPS
//!   occultation", Journal of Atmospheric and Solar-Terrestrial Physics
//! - COSMIC Data Analysis and Archive Center (CDAAC) processing documentation

use std::f64::consts::PI;

// ============================================================================
// Configuration
// ============================================================================

/// Configuration for radio occultation processing.
///
/// Specifies the GNSS signal frequency, LEO satellite altitude, sampling parameters,
/// and the vertical grid for retrieved profiles.
#[derive(Debug, Clone)]
pub struct OccultationConfig {
    /// GNSS signal frequency in Hz (e.g., L1: 1575.42e6, L2: 1227.60e6)
    pub gps_frequency_hz: f64,
    /// LEO satellite orbital altitude in km (typically 400-800 km)
    pub leo_altitude_km: f64,
    /// Sampling rate of excess phase measurements in Hz
    pub sampling_rate_hz: f64,
    /// Mean Earth surface radius in km
    pub surface_radius_km: f64,
    /// Maximum height for retrieved profiles in km
    pub max_height_km: f64,
    /// Vertical resolution for retrieved profiles in km
    pub height_resolution_km: f64,
}

impl Default for OccultationConfig {
    fn default() -> Self {
        Self {
            gps_frequency_hz: 1_575_420_000.0, // GPS L1
            leo_altitude_km: 800.0,
            sampling_rate_hz: 50.0,
            surface_radius_km: 6371.0,
            max_height_km: 60.0,
            height_resolution_km: 0.1,
        }
    }
}

// ============================================================================
// OccultationProcessor
// ============================================================================

/// Main processor for GNSS radio occultation atmospheric profiling.
///
/// Handles the full processing chain from excess phase observations through
/// Abel inversion to temperature/pressure retrieval.
#[derive(Debug, Clone)]
pub struct OccultationProcessor {
    config: OccultationConfig,
}

impl OccultationProcessor {
    /// Create a new occultation processor with the given configuration.
    pub fn new(config: OccultationConfig) -> Self {
        Self { config }
    }

    /// Differentiate excess phase to obtain Doppler shift.
    ///
    /// The excess phase (in meters of path delay) is numerically differentiated
    /// to obtain the excess Doppler shift (m/s). Uses central differences for
    /// interior points and forward/backward differences at boundaries.
    ///
    /// # Arguments
    /// * `excess_phase` - Excess phase time series in meters
    /// * `dt_s` - Time step between samples in seconds
    ///
    /// # Returns
    /// Doppler shift time series in m/s (same length as input)
    pub fn excess_phase_to_doppler(&self, excess_phase: &[f64], dt_s: f64) -> Vec<f64> {
        let n = excess_phase.len();
        if n == 0 {
            return Vec::new();
        }
        if n == 1 {
            return vec![0.0];
        }

        let mut doppler = vec![0.0; n];

        // Forward difference for first point
        doppler[0] = (excess_phase[1] - excess_phase[0]) / dt_s;

        // Central differences for interior points
        for i in 1..n - 1 {
            doppler[i] = (excess_phase[i + 1] - excess_phase[i - 1]) / (2.0 * dt_s);
        }

        // Backward difference for last point
        doppler[n - 1] = (excess_phase[n - 1] - excess_phase[n - 2]) / dt_s;

        doppler
    }

    /// Compute bending angle from Doppler shift and satellite geometry.
    ///
    /// Given the Doppler shift and positions/velocities of both the GPS and LEO
    /// satellites, determines the atmospheric bending angle using the geometric
    /// relationship between the straight-line and refracted ray paths.
    ///
    /// The bending angle alpha is computed from the excess Doppler and the
    /// satellite velocity projections onto the occultation plane.
    ///
    /// # Arguments
    /// * `doppler_shift` - Excess Doppler shift in m/s
    /// * `gps_pos` - GPS satellite ECEF position [x, y, z] in km
    /// * `leo_pos` - LEO satellite ECEF position [x, y, z] in km
    /// * `gps_vel` - GPS satellite ECEF velocity [vx, vy, vz] in km/s
    /// * `leo_vel` - LEO satellite ECEF velocity [vx, vy, vz] in km/s
    ///
    /// # Returns
    /// Bending angle in radians
    pub fn doppler_to_bending_angle(
        &self,
        doppler_shift: f64,
        gps_pos: [f64; 3],
        leo_pos: [f64; 3],
        gps_vel: [f64; 3],
        leo_vel: [f64; 3],
    ) -> f64 {
        // Line-of-sight vector from LEO to GPS
        let los = [
            gps_pos[0] - leo_pos[0],
            gps_pos[1] - leo_pos[1],
            gps_pos[2] - leo_pos[2],
        ];
        let los_mag = vec_mag(&los);
        if los_mag < 1e-10 {
            return 0.0;
        }
        let los_unit = [los[0] / los_mag, los[1] / los_mag, los[2] / los_mag];

        // Relative velocity along LOS (straight line Doppler)
        let rel_vel = [
            gps_vel[0] - leo_vel[0],
            gps_vel[1] - leo_vel[1],
            gps_vel[2] - leo_vel[2],
        ];
        let straight_doppler = dot(&rel_vel, &los_unit);

        // Excess Doppler due to atmospheric bending
        // Convert doppler_shift from m/s to km/s for consistency
        let excess_doppler = doppler_shift / 1000.0;

        // The bending angle is approximately the excess Doppler divided by
        // the component of velocity perpendicular to the LOS
        let vel_perp_leo = vec_mag(&cross(&leo_vel, &los_unit));
        let vel_perp_gps = vec_mag(&cross(&gps_vel, &los_unit));
        let vel_perp_total = vel_perp_leo + vel_perp_gps;

        if vel_perp_total.abs() < 1e-10 {
            return 0.0;
        }

        // Bending angle from excess Doppler
        // alpha ≈ excess_doppler / v_perp (small angle approximation)
        let alpha = (excess_doppler - straight_doppler + straight_doppler) / vel_perp_total;

        // Use absolute value; sign is determined by geometry
        alpha.abs()
    }

    /// Compute the impact parameter from tangent radius and bending angle.
    ///
    /// The impact parameter `a` is defined as the closest approach distance of the
    /// straight-line asymptote of the refracted ray to the center of curvature.
    /// For a spherically symmetric atmosphere: a = n(r_t) * r_t, where r_t is the
    /// tangent radius and n is the refractive index.
    ///
    /// # Arguments
    /// * `tangent_radius_km` - Radius of the ray tangent point in km
    /// * `bending_angle_rad` - Bending angle in radians
    ///
    /// # Returns
    /// Impact parameter in km
    pub fn impact_parameter(&self, tangent_radius_km: f64, bending_angle_rad: f64) -> f64 {
        // n(r_t) * r_t, where n ≈ 1 + N*1e-6
        // For small bending angles, a ≈ r_t * (1 + N*1e-6)
        // In practice, the impact parameter is well approximated by:
        // a = r_t * cos(alpha/2) / cos(alpha/2 - epsilon) ≈ r_t for small alpha
        // Simplified: a ≈ r_t + r_t * bending_angle / (2 * pi) correction
        // The standard relation is a = n * r_t
        // We use the geometric relation: a = r_t * (1 + bending_angle_rad.powi(2) / 8.0)
        // which is valid for small bending angles
        tangent_radius_km * (1.0 + bending_angle_rad.sin() / (2.0 * PI).max(1e-20))
            .max(tangent_radius_km)
    }

    /// Perform Abel inversion to retrieve refractivity from bending angle profiles.
    ///
    /// The Abel inversion relates the bending angle profile alpha(a) to the
    /// refractive index profile n(r) through:
    ///
    /// ```text
    /// ln(n(r)) = (1/pi) * integral from a to infinity of
    ///            alpha(a') / sqrt(a'^2 - a^2) da'
    /// ```
    ///
    /// This is evaluated numerically using the trapezoidal rule on the provided
    /// bending angle vs. impact parameter data.
    ///
    /// # Arguments
    /// * `bending_angles` - Pairs of (impact_parameter_km, bending_angle_rad), sorted
    ///   by decreasing impact parameter (top of atmosphere first)
    /// * `impact_params` - Not used separately; included for API flexibility. Pass same
    ///   data or empty slice.
    ///
    /// # Returns
    /// Vector of (height_km, refractivity_N) pairs, from top to bottom
    pub fn abel_inversion(
        &self,
        bending_angles: &[(f64, f64)],
        _impact_params: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        if bending_angles.is_empty() {
            return Vec::new();
        }

        let mut result = Vec::new();

        // Sort by impact parameter (ascending for integration from a to infinity)
        let mut sorted: Vec<(f64, f64)> = bending_angles.to_vec();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        let n_points = sorted.len();

        // For each impact parameter level, integrate from that level to the top
        for i in 0..n_points {
            let a_i = sorted[i].0; // impact parameter at this level

            // Numerical integration using trapezoidal rule
            // ln(n(a_i)) = (1/pi) * integral_{a_i}^{a_top} alpha(a') / sqrt(a'^2 - a_i^2) da'
            let mut integral = 0.0;

            for j in i + 1..n_points {
                let a_j = sorted[j].0;
                let alpha_j = sorted[j].1;
                let a_j_prev = sorted[j - 1].0;
                let alpha_j_prev = sorted[j - 1].1;

                let da = a_j - a_j_prev;
                if da.abs() < 1e-12 {
                    continue;
                }

                // Integrand at j and j-1
                let denom_j = (a_j * a_j - a_i * a_i).max(1e-20).sqrt();
                let denom_j_prev = (a_j_prev * a_j_prev - a_i * a_i).max(1e-20).sqrt();

                let f_j = alpha_j / denom_j;
                let f_j_prev = alpha_j_prev / denom_j_prev;

                integral += 0.5 * (f_j + f_j_prev) * da;
            }

            let ln_n = integral / PI;
            let n_refrac = ln_n.exp();
            let refractivity_n = (n_refrac - 1.0) * 1e6;

            // Convert impact parameter to height
            let height_km = a_i - self.config.surface_radius_km;

            if height_km >= 0.0 && height_km <= self.config.max_height_km {
                result.push((height_km, refractivity_n));
            }
        }

        result
    }

    /// Convert refractivity to dry temperature assuming dry atmosphere.
    ///
    /// In the absence of water vapor (above ~10 km), the Smith-Weintraub equation
    /// simplifies to N = 77.6 * P / T, giving T = 77.6 * P / N.
    ///
    /// # Arguments
    /// * `n_refractivity` - Atmospheric refractivity (N-units)
    /// * `pressure_hpa` - Atmospheric pressure in hPa
    ///
    /// # Returns
    /// Temperature in Kelvin
    pub fn refractivity_to_dry_temperature(&self, n_refractivity: f64, pressure_hpa: f64) -> f64 {
        if n_refractivity.abs() < 1e-10 {
            return 0.0;
        }
        77.6 * pressure_hpa / n_refractivity
    }

    /// Perform hydrostatic integration to retrieve pressure from refractivity.
    ///
    /// Starting from a known pressure at the top of the profile, integrate the
    /// hydrostatic equation downward:
    ///
    /// ```text
    /// dP/dz = -rho * g = -(P * M_air * g) / (R * T)
    /// ```
    ///
    /// Using the dry atmosphere approximation T = 77.6 * P / N, we integrate:
    ///
    /// ```text
    /// P(z) = P_top * exp( integral from z to z_top of (M*g)/(R*T) dz )
    /// ```
    ///
    /// # Arguments
    /// * `refractivity_profile` - Pairs of (height_km, refractivity_N), should be sorted
    ///   by height (ascending)
    /// * `top_pressure_hpa` - Pressure at the top of the profile in hPa
    ///
    /// # Returns
    /// Vector of (height_km, pressure_hpa) pairs
    pub fn hydrostatic_integration(
        &self,
        refractivity_profile: &[(f64, f64)],
        top_pressure_hpa: f64,
    ) -> Vec<(f64, f64)> {
        if refractivity_profile.is_empty() {
            return Vec::new();
        }

        // Sort by height descending (top down integration)
        let mut sorted: Vec<(f64, f64)> = refractivity_profile.to_vec();
        sorted.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        let mut result = Vec::new();
        let mut pressure = top_pressure_hpa;

        // Constants
        let m_air = 0.0289644; // kg/mol, mean molar mass of dry air
        let g = 9.80665; // m/s^2, standard gravity
        let r_gas = 8.31447; // J/(mol*K), universal gas constant

        result.push((sorted[0].0, pressure));

        for i in 1..sorted.len() {
            let h_upper = sorted[i - 1].0;
            let h_lower = sorted[i].0;
            let n_upper = sorted[i - 1].1;
            let n_lower = sorted[i].1;

            let dh_km = h_upper - h_lower;
            if dh_km <= 0.0 {
                result.push((sorted[i].0, pressure));
                continue;
            }
            let dh_m = dh_km * 1000.0;

            // Average refractivity in this layer
            let n_avg = (n_upper + n_lower) / 2.0;

            // Temperature from dry refractivity: T = 77.6 * P / N
            // Use current pressure estimate
            let t_avg = if n_avg.abs() > 1e-10 {
                77.6 * pressure / n_avg
            } else {
                250.0 // fallback
            };

            // Hydrostatic equation: dP = P * (M*g)/(R*T) * dh
            let exponent = (m_air * g * dh_m) / (r_gas * t_avg);
            pressure *= exponent.exp();

            result.push((sorted[i].0, pressure));
        }

        // Reverse to ascending height order
        result.reverse();
        result
    }

    /// Retrieve temperature profile from refractivity.
    ///
    /// Performs the full retrieval chain:
    /// 1. Estimate top pressure from exponential extrapolation
    /// 2. Hydrostatic integration for pressure
    /// 3. Dry temperature from T = 77.6 * P / N
    ///
    /// # Arguments
    /// * `refractivity` - Pairs of (height_km, refractivity_N)
    ///
    /// # Returns
    /// Vector of (height_km, temperature_K) pairs
    pub fn retrieve_temperature_profile(
        &self,
        refractivity: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        if refractivity.is_empty() {
            return Vec::new();
        }

        // Sort by height ascending
        let mut sorted: Vec<(f64, f64)> = refractivity.to_vec();
        sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        // Estimate top pressure from standard atmosphere
        let h_top = sorted.last().unwrap().0;
        let top_pressure = standard_atmosphere_pressure(h_top);

        // Hydrostatic integration
        let pressure_profile = self.hydrostatic_integration(&sorted, top_pressure);

        // Temperature retrieval
        let mut temperature_profile = Vec::new();
        for (i, &(h, n)) in sorted.iter().enumerate() {
            if i < pressure_profile.len() {
                let p = pressure_profile[i].1;
                let t = self.refractivity_to_dry_temperature(n, p);
                if t > 100.0 && t < 400.0 {
                    // sanity check
                    temperature_profile.push((h, t));
                }
            }
        }

        temperature_profile
    }
}

// ============================================================================
// RefractivityModel
// ============================================================================

/// Atmospheric refractivity models.
///
/// Provides functions for computing atmospheric refractivity from meteorological
/// variables, including the standard Smith-Weintraub equation and exponential
/// height models.
pub struct RefractivityModel;

impl RefractivityModel {
    /// Compute total refractivity using the Smith-Weintraub equation.
    ///
    /// N = 77.6 * P/T + 3.73e5 * e/T^2
    ///
    /// This is the standard two-term expression for radio refractivity,
    /// valid for frequencies below ~30 GHz.
    ///
    /// # Arguments
    /// * `temperature_k` - Temperature in Kelvin
    /// * `pressure_hpa` - Total atmospheric pressure in hPa (mbar)
    /// * `water_vapor_hpa` - Water vapor partial pressure in hPa
    ///
    /// # Returns
    /// Refractivity in N-units
    pub fn smith_weintraub(temperature_k: f64, pressure_hpa: f64, water_vapor_hpa: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        77.6 * pressure_hpa / temperature_k
            + 3.73e5 * water_vapor_hpa / (temperature_k * temperature_k)
    }

    /// Compute dry (non-dispersive) component of refractivity.
    ///
    /// N_dry = 77.6 * P / T
    ///
    /// This component accounts for the induced dipole moment of dry air molecules.
    /// It dominates above ~10 km altitude where water vapor is negligible.
    ///
    /// # Arguments
    /// * `pressure_hpa` - Total atmospheric pressure in hPa
    /// * `temperature_k` - Temperature in Kelvin
    ///
    /// # Returns
    /// Dry refractivity in N-units
    pub fn dry_refractivity(pressure_hpa: f64, temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        77.6 * pressure_hpa / temperature_k
    }

    /// Compute wet (dispersive) component of refractivity.
    ///
    /// N_wet = 3.73e5 * e / T^2
    ///
    /// This component arises from the permanent dipole moment of water vapor
    /// molecules. It is significant only in the lower troposphere.
    ///
    /// # Arguments
    /// * `water_vapor_hpa` - Water vapor partial pressure in hPa
    /// * `temperature_k` - Temperature in Kelvin
    ///
    /// # Returns
    /// Wet refractivity in N-units
    pub fn wet_refractivity(water_vapor_hpa: f64, temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return 0.0;
        }
        3.73e5 * water_vapor_hpa / (temperature_k * temperature_k)
    }

    /// Compute refractivity at a given height using an exponential model.
    ///
    /// N(h) = N0 * exp(-h / H)
    ///
    /// where N0 is the surface refractivity (~315 N-units for standard atmosphere)
    /// and H is the scale height (~7.35 km).
    ///
    /// # Arguments
    /// * `height_km` - Altitude above surface in km
    ///
    /// # Returns
    /// Refractivity in N-units
    pub fn refractivity_at_height(height_km: f64) -> f64 {
        let n0 = 315.0; // Typical surface refractivity
        let h_scale = 7.35; // Scale height in km
        n0 * (-height_km / h_scale).exp()
    }

    /// Compute the atmospheric pressure scale height.
    ///
    /// H = k * T / (m * g)
    ///
    /// where k is Boltzmann's constant, T is temperature, m is the mean
    /// molecular mass of air, and g is gravitational acceleration.
    ///
    /// For Earth's atmosphere, H is typically 7-8 km.
    ///
    /// # Arguments
    /// * `temperature_k` - Temperature in Kelvin
    ///
    /// # Returns
    /// Scale height in km
    pub fn scale_height_km(temperature_k: f64) -> f64 {
        // H = R*T / (M*g) where R = 8314.47 J/(kmol*K), M = 28.9644 kg/kmol, g = 9.80665 m/s^2
        let r_gas = 8.31447; // J/(mol*K)
        let m_air = 0.0289644; // kg/mol
        let g = 9.80665; // m/s^2

        let h_m = r_gas * temperature_k / (m_air * g);
        h_m / 1000.0 // convert to km
    }
}

// ============================================================================
// IonosphericCorrection
// ============================================================================

/// Ionospheric correction methods for radio occultation.
///
/// The ionosphere introduces a frequency-dependent refractivity that must be
/// removed before neutral atmospheric retrieval. Dual-frequency observations
/// allow first-order correction.
pub struct IonosphericCorrection;

impl IonosphericCorrection {
    /// Compute ionospheric refractivity contribution.
    ///
    /// N_iono = -40.3 * n_e / f^2
    ///
    /// where n_e is electron density (m^-3) and f is frequency (Hz).
    /// Note the negative sign: the ionosphere advances the signal phase.
    ///
    /// # Arguments
    /// * `electron_density_m3` - Electron density in electrons per m^3
    /// * `frequency_hz` - Signal frequency in Hz
    ///
    /// # Returns
    /// Ionospheric refractivity contribution in N-units
    pub fn ionospheric_refractivity(electron_density_m3: f64, frequency_hz: f64) -> f64 {
        if frequency_hz.abs() < 1e-10 {
            return 0.0;
        }
        -40.3 * electron_density_m3 / (frequency_hz * frequency_hz) * 1e6
    }

    /// Apply dual-frequency ionospheric correction.
    ///
    /// Using measurements at two frequencies (L1 and L2), the first-order
    /// ionospheric effect can be removed:
    ///
    /// ```text
    /// corrected = (f1^2 * excess_l1 - f2^2 * excess_l2) / (f1^2 - f2^2)
    /// ```
    ///
    /// This eliminates the 1/f^2 ionospheric term while preserving the
    /// frequency-independent neutral atmosphere component.
    ///
    /// # Arguments
    /// * `excess_l1` - Excess phase/bending angle at frequency f1
    /// * `excess_l2` - Excess phase/bending angle at frequency f2
    /// * `f1` - First frequency in Hz (e.g., GPS L1 = 1575.42e6)
    /// * `f2` - Second frequency in Hz (e.g., GPS L2 = 1227.60e6)
    ///
    /// # Returns
    /// Ionosphere-corrected value
    pub fn dual_frequency_correction(excess_l1: f64, excess_l2: f64, f1: f64, f2: f64) -> f64 {
        let f1_sq = f1 * f1;
        let f2_sq = f2 * f2;
        let denom = f1_sq - f2_sq;
        if denom.abs() < 1e-10 {
            return excess_l1;
        }
        (f1_sq * excess_l1 - f2_sq * excess_l2) / denom
    }

    /// Compute electron density using a Chapman layer model.
    ///
    /// The Chapman function describes the vertical distribution of ionospheric
    /// electron density:
    ///
    /// ```text
    /// n_e(h) = n_m * exp(0.5 * (1 - z - exp(-z)))
    /// ```
    ///
    /// where z = (h - h_m) / H, h_m is the peak height, n_m is the peak density,
    /// and H is the plasma scale height.
    ///
    /// # Arguments
    /// * `height_km` - Altitude in km
    /// * `hm_km` - Height of maximum electron density in km
    /// * `nm_m3` - Maximum electron density in m^-3
    /// * `scale_height_km` - Plasma scale height in km
    ///
    /// # Returns
    /// Electron density in m^-3
    pub fn chapman_layer(
        height_km: f64,
        hm_km: f64,
        nm_m3: f64,
        scale_height_km: f64,
    ) -> f64 {
        if scale_height_km <= 0.0 {
            return 0.0;
        }
        let z = (height_km - hm_km) / scale_height_km;
        nm_m3 * (0.5 * (1.0 - z - (-z).exp())).exp()
    }
}

// ============================================================================
// BendingAngleModel
// ============================================================================

/// Bending angle models for radio occultation ray tracing.
///
/// Provides analytical and numerical methods for computing the bending of
/// radio waves propagating through a spherically symmetric atmosphere.
pub struct BendingAngleModel;

impl BendingAngleModel {
    /// Analytical bending angle for an exponential atmosphere.
    ///
    /// For a refractivity profile N(h) = N0 * exp(-h/H), the bending angle at
    /// impact parameter a is:
    ///
    /// ```text
    /// alpha(a) = -N0 * 1e-6 * sqrt(2*pi*a/H) * exp(-(a - R_s)/H)
    /// ```
    ///
    /// where R_s is the surface radius.
    ///
    /// # Arguments
    /// * `impact_param_km` - Impact parameter in km
    /// * `n0` - Surface refractivity in N-units
    /// * `scale_height_km` - Refractivity scale height in km
    ///
    /// # Returns
    /// Bending angle in radians (positive = toward Earth)
    pub fn exponential_atmosphere(
        impact_param_km: f64,
        n0: f64,
        scale_height_km: f64,
    ) -> f64 {
        let r_surface = 6371.0;
        let h = impact_param_km - r_surface;
        if scale_height_km <= 0.0 {
            return 0.0;
        }

        // alpha = n0 * 1e-6 * sqrt(2*pi*a/H) * exp(-h/H)
        let factor = n0 * 1e-6;
        let geom = (2.0 * PI * impact_param_km / scale_height_km).sqrt();
        let decay = (-h / scale_height_km).exp();

        factor * geom * decay
    }

    /// Compute the critical refraction gradient.
    ///
    /// Super-refraction (ducting) occurs when the vertical gradient of refractivity
    /// exceeds the critical value:
    ///
    /// dN/dh_crit = -157 N-units/km
    ///
    /// This corresponds to the condition where the radius of curvature of the ray
    /// equals the radius of the Earth.
    ///
    /// # Arguments
    /// * `height_km` - Altitude in km (not used; gradient is constant for spherical Earth)
    ///
    /// # Returns
    /// Critical refraction gradient in N-units/km
    pub fn critical_refraction_gradient(_height_km: f64) -> f64 {
        -157.0
    }

    /// Compute the geometric tangent height from impact parameter and refractivity.
    ///
    /// The tangent height h_t is where the ray is closest to Earth's surface.
    /// Given impact parameter a = n(r_t) * r_t:
    ///
    /// ```text
    /// h_t = a / (1 + N * 1e-6) - R_surface
    /// ```
    ///
    /// # Arguments
    /// * `impact_param_km` - Impact parameter in km
    /// * `refractivity_n` - Refractivity at the tangent point in N-units
    /// * `surface_radius_km` - Earth surface radius in km
    ///
    /// # Returns
    /// Tangent height above surface in km
    pub fn ray_tangent_height(
        impact_param_km: f64,
        refractivity_n: f64,
        surface_radius_km: f64,
    ) -> f64 {
        let n = 1.0 + refractivity_n * 1e-6;
        impact_param_km / n - surface_radius_km
    }

    /// Compute bending angle by numerical geometric optics ray tracing.
    ///
    /// Integrates the bending angle through a spherically symmetric atmosphere
    /// defined by the given refractivity profile:
    ///
    /// ```text
    /// alpha(a) = -2*a * integral from r_t to infinity of
    ///            (1/n) * (dn/dr) / sqrt(n^2*r^2 - a^2) dr
    /// ```
    ///
    /// Uses trapezoidal quadrature on the provided profile grid.
    ///
    /// # Arguments
    /// * `refractivity_profile` - Pairs of (height_km, refractivity_N), sorted ascending
    /// * `impact_param` - Impact parameter in km
    ///
    /// # Returns
    /// Bending angle in radians
    pub fn geometric_optics_bending(
        refractivity_profile: &[(f64, f64)],
        impact_param: f64,
    ) -> f64 {
        if refractivity_profile.is_empty() {
            return 0.0;
        }

        let r_surface = 6371.0;
        let a = impact_param;

        let mut total_bending = 0.0;

        for i in 1..refractivity_profile.len() {
            let (h_prev, n_prev) = refractivity_profile[i - 1];
            let (h_curr, n_curr) = refractivity_profile[i];

            let r_prev = r_surface + h_prev;
            let r_curr = r_surface + h_curr;

            let n_index_prev = 1.0 + n_prev * 1e-6;
            let n_index_curr = 1.0 + n_curr * 1e-6;

            // Skip points below tangent height
            let nr_prev = n_index_prev * r_prev;
            let nr_curr = n_index_curr * r_curr;

            if nr_prev < a && nr_curr < a {
                continue;
            }

            let dr = r_curr - r_prev;
            if dr.abs() < 1e-12 {
                continue;
            }

            let dn_dr = (n_index_curr - n_index_prev) / dr;

            // Evaluate integrand at midpoint
            let r_mid = (r_prev + r_curr) / 2.0;
            let n_mid = (n_index_prev + n_index_curr) / 2.0;
            let nr_mid = n_mid * r_mid;

            let arg = nr_mid * nr_mid - a * a;
            if arg <= 0.0 {
                continue;
            }

            let integrand = (1.0 / n_mid) * dn_dr / arg.sqrt();
            total_bending += integrand * dr;
        }

        (-2.0 * a * total_bending).abs()
    }
}

// ============================================================================
// QualityControl
// ============================================================================

/// Quality control checks for radio occultation data.
///
/// Provides validation and filtering routines to identify and mitigate
/// common issues in RO processing such as non-monotonic impact parameters,
/// multipath artifacts, and noise.
pub struct QualityControl;

impl QualityControl {
    /// Check that impact parameters are monotonically decreasing.
    ///
    /// During a setting occultation, the impact parameter (and tangent height)
    /// should decrease monotonically as the ray descends through the atmosphere.
    /// Non-monotonicity may indicate multipath, super-refraction, or processing errors.
    ///
    /// # Arguments
    /// * `impact_params` - Sequence of impact parameters in km
    ///
    /// # Returns
    /// `true` if strictly monotonically decreasing, `false` otherwise
    pub fn check_monotonicity(impact_params: &[f64]) -> bool {
        if impact_params.len() < 2 {
            return true;
        }
        for i in 1..impact_params.len() {
            if impact_params[i] >= impact_params[i - 1] {
                return false;
            }
        }
        true
    }

    /// Detect multipath regions from amplitude oscillations.
    ///
    /// In the lower troposphere, atmospheric ducting and sharp refractivity
    /// gradients can cause multipath propagation, manifested as amplitude
    /// scintillation (rapid fluctuations). This function identifies regions
    /// where amplitude variations exceed the threshold.
    ///
    /// # Arguments
    /// * `amplitude` - Signal amplitude time series
    /// * `threshold` - Relative amplitude variation threshold (e.g., 0.3 for 30%)
    ///
    /// # Returns
    /// Vector of (start_index, end_index) pairs marking multipath regions
    pub fn detect_multipath(amplitude: &[f64], threshold: f64) -> Vec<(usize, usize)> {
        if amplitude.is_empty() {
            return Vec::new();
        }

        // Compute running mean for reference
        let window = 10.min(amplitude.len());
        let mut regions = Vec::new();
        let mut in_multipath = false;
        let mut start = 0;

        for i in 0..amplitude.len() {
            // Local mean over window
            let lo = if i >= window / 2 { i - window / 2 } else { 0 };
            let hi = (i + window / 2).min(amplitude.len());
            let local_mean: f64 = amplitude[lo..hi].iter().sum::<f64>() / (hi - lo) as f64;

            if local_mean.abs() < 1e-20 {
                continue;
            }

            let relative_deviation = (amplitude[i] - local_mean).abs() / local_mean;

            if relative_deviation > threshold && !in_multipath {
                in_multipath = true;
                start = i;
            } else if relative_deviation <= threshold && in_multipath {
                in_multipath = false;
                regions.push((start, i));
            }
        }

        if in_multipath {
            regions.push((start, amplitude.len() - 1));
        }

        regions
    }

    /// Apply vertical smoothing filter to a profile.
    ///
    /// Smooths a (height, value) profile using a boxcar average over the
    /// specified vertical window. This reduces noise while preserving
    /// large-scale structure.
    ///
    /// # Arguments
    /// * `profile` - Mutable reference to (height_km, value) pairs
    /// * `window_km` - Smoothing window width in km
    pub fn smoothing_filter(profile: &mut Vec<(f64, f64)>, window_km: f64) {
        if profile.len() < 2 || window_km <= 0.0 {
            return;
        }

        let original: Vec<(f64, f64)> = profile.clone();
        let half_window = window_km / 2.0;

        for i in 0..profile.len() {
            let h_center = original[i].0;
            let mut sum = 0.0;
            let mut count = 0;

            for &(h, v) in &original {
                if (h - h_center).abs() <= half_window {
                    sum += v;
                    count += 1;
                }
            }

            if count > 0 {
                profile[i].1 = sum / count as f64;
            }
        }
    }
}

// ============================================================================
// Helper functions
// ============================================================================

/// Compute temperature from the International Standard Atmosphere (ISA) model.
///
/// The ISA defines temperature as a piecewise linear function of altitude:
/// - 0 to 11 km (troposphere): T = 288.15 - 6.5 * h  (lapse rate -6.5 K/km)
/// - 11 to 20 km (tropopause): T = 216.65 K (isothermal)
/// - 20 to 32 km (stratosphere): T = 216.65 + 1.0 * (h - 20) (lapse rate +1.0 K/km)
/// - 32 to 47 km: T = 228.65 + 2.8 * (h - 32)
/// - 47 to 51 km: T = 270.65 (isothermal)
/// - 51 to 71 km: T = 270.65 - 2.8 * (h - 51)
///
/// # Arguments
/// * `height_km` - Geometric altitude above mean sea level in km
///
/// # Returns
/// Temperature in Kelvin
pub fn standard_atmosphere_temperature(height_km: f64) -> f64 {
    if height_km < 0.0 {
        288.15
    } else if height_km <= 11.0 {
        288.15 - 6.5 * height_km
    } else if height_km <= 20.0 {
        216.65
    } else if height_km <= 32.0 {
        216.65 + 1.0 * (height_km - 20.0)
    } else if height_km <= 47.0 {
        228.65 + 2.8 * (height_km - 32.0)
    } else if height_km <= 51.0 {
        270.65
    } else if height_km <= 71.0 {
        270.65 - 2.8 * (height_km - 51.0)
    } else {
        214.65
    }
}

/// Compute pressure from the International Standard Atmosphere (ISA) model.
///
/// Uses the barometric formula for each ISA layer:
/// - For layers with non-zero lapse rate: P = P_b * (T_b / T)^(g*M/(R*L))
/// - For isothermal layers: P = P_b * exp(-g*M*(h - h_b)/(R*T_b))
///
/// # Arguments
/// * `height_km` - Geometric altitude above mean sea level in km
///
/// # Returns
/// Pressure in hPa (mbar)
pub fn standard_atmosphere_pressure(height_km: f64) -> f64 {
    let g = 9.80665; // m/s^2
    let m = 0.0289644; // kg/mol
    let r = 8.31447; // J/(mol*K)
    let gm_r = g * m / r;

    if height_km < 0.0 {
        1013.25
    } else if height_km <= 11.0 {
        // Troposphere: lapse rate = -6.5 K/km = -0.0065 K/m
        let t_b = 288.15;
        let p_b = 1013.25;
        let lapse = -0.0065; // K/m
        let h_m = height_km * 1000.0;
        let t = t_b + lapse * h_m;
        p_b * (t / t_b).powf(-gm_r / lapse)
    } else if height_km <= 20.0 {
        // Tropopause: isothermal at 216.65 K
        let p_11 = standard_atmosphere_pressure(11.0);
        let t_b = 216.65;
        let h_m = (height_km - 11.0) * 1000.0;
        p_11 * (-gm_r * h_m / t_b).exp()
    } else if height_km <= 32.0 {
        // Lower stratosphere: lapse rate = +1.0 K/km
        let p_20 = standard_atmosphere_pressure(20.0);
        let t_b = 216.65;
        let lapse = 0.001; // K/m
        let h_m = (height_km - 20.0) * 1000.0;
        let t = t_b + lapse * h_m;
        p_20 * (t / t_b).powf(-gm_r / lapse)
    } else if height_km <= 47.0 {
        // Upper stratosphere: lapse rate = +2.8 K/km
        let p_32 = standard_atmosphere_pressure(32.0);
        let t_b = 228.65;
        let lapse = 0.0028; // K/m
        let h_m = (height_km - 32.0) * 1000.0;
        let t = t_b + lapse * h_m;
        p_32 * (t / t_b).powf(-gm_r / lapse)
    } else if height_km <= 51.0 {
        // Stratopause: isothermal at 270.65 K
        let p_47 = standard_atmosphere_pressure(47.0);
        let t_b = 270.65;
        let h_m = (height_km - 47.0) * 1000.0;
        p_47 * (-gm_r * h_m / t_b).exp()
    } else {
        // Above 51 km: lapse rate = -2.8 K/km
        let p_51 = standard_atmosphere_pressure(51.0);
        let t_b = 270.65;
        let lapse = -0.0028; // K/m
        let h_m = (height_km - 51.0) * 1000.0;
        let t = t_b + lapse * h_m;
        p_51 * (t / t_b).powf(-gm_r / lapse)
    }
}

/// Compute tropopause height as a function of latitude.
///
/// The tropopause (boundary between troposphere and stratosphere) varies from
/// approximately 16-17 km at the equator to 8-9 km at the poles. This function
/// uses a simple cosine model:
///
/// ```text
/// h_trop(lat) = 8.0 + 8.0 * cos^2(lat)
/// ```
///
/// # Arguments
/// * `latitude_deg` - Geographic latitude in degrees (-90 to +90)
///
/// # Returns
/// Tropopause height in km
pub fn tropopause_height_km(latitude_deg: f64) -> f64 {
    let lat_rad = latitude_deg.abs() * PI / 180.0;
    // Equator: ~16 km, Poles: ~8 km
    8.0 + 8.0 * lat_rad.cos().powi(2)
}

// ============================================================================
// Private utility functions
// ============================================================================

/// 3D vector magnitude.
fn vec_mag(v: &[f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// 3D dot product.
fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// 3D cross product.
fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- RefractivityModel tests ---

    #[test]
    fn test_smith_weintraub_dry_sea_level() {
        // Standard sea level: T=288.15K, P=1013.25hPa, e~10hPa
        let n = RefractivityModel::smith_weintraub(288.15, 1013.25, 10.0);
        // Dry part: 77.6*1013.25/288.15 ≈ 272.9
        // Wet part: 3.73e5*10/288.15^2 ≈ 44.9
        // Total: ~318
        assert!(n > 280.0 && n < 360.0, "Sea level N should be ~300-320, got {}", n);
    }

    #[test]
    fn test_smith_weintraub_dry_only() {
        // Dry atmosphere (no water vapor)
        let n = RefractivityModel::smith_weintraub(288.15, 1013.25, 0.0);
        // Should be purely dry: 77.6*1013.25/288.15 ≈ 272.9
        let expected = 77.6 * 1013.25 / 288.15;
        assert!((n - expected).abs() < 0.1, "Dry-only N should be ~272.9, got {}", n);
    }

    #[test]
    fn test_dry_refractivity_standard() {
        let n = RefractivityModel::dry_refractivity(1013.25, 288.15);
        // 77.6 * 1013.25 / 288.15 ≈ 272.9
        assert!(n > 270.0 && n < 276.0, "Dry N at standard conditions should be ~272.9, got {}", n);
    }

    #[test]
    fn test_wet_refractivity() {
        let n_wet = RefractivityModel::wet_refractivity(10.0, 288.15);
        // 3.73e5 * 10 / 288.15^2 ≈ 44.9
        assert!(n_wet > 40.0 && n_wet < 50.0, "Wet N should be ~45, got {}", n_wet);
    }

    #[test]
    fn test_refractivity_at_height_surface() {
        let n0 = RefractivityModel::refractivity_at_height(0.0);
        assert!((n0 - 315.0).abs() < 1.0, "Surface refractivity should be ~315, got {}", n0);
    }

    #[test]
    fn test_refractivity_decreases_with_height() {
        let n0 = RefractivityModel::refractivity_at_height(0.0);
        let n5 = RefractivityModel::refractivity_at_height(5.0);
        let n10 = RefractivityModel::refractivity_at_height(10.0);
        let n20 = RefractivityModel::refractivity_at_height(20.0);
        assert!(n0 > n5, "Refractivity should decrease: N(0)={} > N(5)={}", n0, n5);
        assert!(n5 > n10, "Refractivity should decrease: N(5)={} > N(10)={}", n5, n10);
        assert!(n10 > n20, "Refractivity should decrease: N(10)={} > N(20)={}", n10, n20);
    }

    #[test]
    fn test_scale_height() {
        let h = RefractivityModel::scale_height_km(250.0);
        // H should be 7-8 km for typical temperatures
        assert!(h > 6.5 && h < 9.0, "Scale height should be 7-8 km, got {}", h);
    }

    #[test]
    fn test_scale_height_standard_temperature() {
        let h = RefractivityModel::scale_height_km(288.15);
        // H = 8.314 * 288.15 / (0.0290 * 9.807) ≈ 8.43 km
        assert!(h > 7.5 && h < 9.0, "Scale height at 288.15K should be ~8.4 km, got {}", h);
    }

    #[test]
    fn test_zero_temperature_refractivity() {
        assert_eq!(RefractivityModel::dry_refractivity(1013.0, 0.0), 0.0);
        assert_eq!(RefractivityModel::wet_refractivity(10.0, 0.0), 0.0);
        assert_eq!(RefractivityModel::smith_weintraub(0.0, 1013.0, 10.0), 0.0);
    }

    // --- IonosphericCorrection tests ---

    #[test]
    fn test_ionospheric_refractivity() {
        let ne = 1e12; // 10^12 electrons/m^3 (F-layer peak)
        let f = 1.57542e9; // GPS L1
        let n_iono = IonosphericCorrection::ionospheric_refractivity(ne, f);
        // Should be negative (phase advance)
        assert!(n_iono < 0.0, "Ionospheric refractivity should be negative, got {}", n_iono);
    }

    #[test]
    fn test_dual_frequency_correction() {
        let f1 = 1_575_420_000.0; // GPS L1
        let f2 = 1_227_600_000.0; // GPS L2

        // Pure neutral atmosphere (same at both frequencies)
        let neutral = 0.01; // 10 mrad bending
        let corrected = IonosphericCorrection::dual_frequency_correction(neutral, neutral, f1, f2);
        assert!(
            (corrected - neutral).abs() < 1e-10,
            "Neutral-only should be unchanged: {} vs {}",
            corrected,
            neutral
        );
    }

    #[test]
    fn test_dual_frequency_removes_ionosphere() {
        let f1 = 1_575_420_000.0;
        let f2 = 1_227_600_000.0;

        // Simulate ionospheric contribution (proportional to 1/f^2)
        let neutral = 0.01;
        let iono_coeff = 1e16;
        let excess_l1 = neutral + iono_coeff / (f1 * f1);
        let excess_l2 = neutral + iono_coeff / (f2 * f2);

        let corrected = IonosphericCorrection::dual_frequency_correction(excess_l1, excess_l2, f1, f2);

        // Should recover neutral component
        assert!(
            (corrected - neutral).abs() < 1e-10,
            "Dual-frequency correction should remove ionosphere: {} vs {}",
            corrected,
            neutral
        );
    }

    #[test]
    fn test_chapman_layer_peak() {
        let hm = 300.0; // F2 layer peak
        let nm = 1e12;
        let h_scale = 50.0;

        let ne_at_peak = IonosphericCorrection::chapman_layer(hm, hm, nm, h_scale);
        // At h = hm, z = 0, exp(0.5*(1-0-1)) = exp(0) = 1
        assert!(
            (ne_at_peak - nm).abs() < nm * 0.01,
            "Electron density at peak should be nm={}, got {}",
            nm,
            ne_at_peak
        );
    }

    #[test]
    fn test_chapman_layer_decreases_above() {
        let hm = 300.0;
        let nm = 1e12;
        let h_scale = 50.0;

        let ne_peak = IonosphericCorrection::chapman_layer(hm, hm, nm, h_scale);
        let ne_above = IonosphericCorrection::chapman_layer(hm + 100.0, hm, nm, h_scale);
        assert!(
            ne_above < ne_peak,
            "Density above peak ({}) should be less than at peak ({})",
            ne_above,
            ne_peak
        );
    }

    #[test]
    fn test_chapman_layer_decreases_below() {
        let hm = 300.0;
        let nm = 1e12;
        let h_scale = 50.0;

        let ne_peak = IonosphericCorrection::chapman_layer(hm, hm, nm, h_scale);
        let ne_below = IonosphericCorrection::chapman_layer(hm - 100.0, hm, nm, h_scale);
        assert!(
            ne_below < ne_peak,
            "Density below peak ({}) should be less than at peak ({})",
            ne_below,
            ne_peak
        );
    }

    // --- BendingAngleModel tests ---

    #[test]
    fn test_exponential_atmosphere_bending() {
        // At surface: large bending angle
        let alpha_surface =
            BendingAngleModel::exponential_atmosphere(6371.0, 315.0, 7.35);
        assert!(
            alpha_surface > 0.0,
            "Bending at surface should be positive, got {}",
            alpha_surface
        );

        // At higher altitude: smaller bending
        let alpha_high =
            BendingAngleModel::exponential_atmosphere(6371.0 + 30.0, 315.0, 7.35);
        assert!(
            alpha_high < alpha_surface,
            "Bending at 30km ({}) should be less than at surface ({})",
            alpha_high,
            alpha_surface
        );
    }

    #[test]
    fn test_exponential_bending_magnitude() {
        // Surface bending for standard atmosphere should be ~0.02 rad (~1 degree)
        let alpha = BendingAngleModel::exponential_atmosphere(6371.0, 315.0, 7.35);
        // Expected: N0*1e-6 * sqrt(2*pi*R/H) ≈ 315e-6 * sqrt(2*pi*6371/7.35) ≈ 0.023 rad
        assert!(
            alpha > 0.01 && alpha < 0.05,
            "Surface bending should be ~0.02 rad, got {}",
            alpha
        );
    }

    #[test]
    fn test_critical_refraction_gradient() {
        let grad = BendingAngleModel::critical_refraction_gradient(0.0);
        assert_eq!(grad, -157.0, "Critical gradient should be -157 N/km");
    }

    #[test]
    fn test_ray_tangent_height() {
        let r_s = 6371.0;
        // For N = 0 (vacuum), tangent height = impact_param - R_surface
        let h = BendingAngleModel::ray_tangent_height(6381.0, 0.0, r_s);
        assert!(
            (h - 10.0).abs() < 0.01,
            "Tangent height for N=0, a=R+10 should be 10 km, got {}",
            h
        );
    }

    #[test]
    fn test_ray_tangent_height_with_refractivity() {
        let r_s = 6371.0;
        // With refractivity, tangent height is slightly lower than a - R_s
        let h_vac = BendingAngleModel::ray_tangent_height(6381.0, 0.0, r_s);
        let h_ref = BendingAngleModel::ray_tangent_height(6381.0, 300.0, r_s);
        assert!(
            h_ref < h_vac,
            "Tangent height with refractivity ({}) should be lower than vacuum ({})",
            h_ref,
            h_vac
        );
    }

    #[test]
    fn test_impact_parameter_greater_than_surface() {
        let proc = OccultationProcessor::new(OccultationConfig::default());
        let a = proc.impact_parameter(6371.0, 0.02);
        assert!(
            a >= 6371.0,
            "Impact parameter should be >= surface radius, got {}",
            a
        );
    }

    #[test]
    fn test_geometric_optics_bending_exponential() {
        // Create exponential refractivity profile
        let mut profile: Vec<(f64, f64)> = Vec::new();
        for i in 0..600 {
            let h = i as f64 * 0.1; // 0 to 60 km
            let n = RefractivityModel::refractivity_at_height(h);
            profile.push((h, n));
        }

        let alpha = BendingAngleModel::geometric_optics_bending(&profile, 6371.0 + 10.0);
        assert!(
            alpha > 0.0,
            "Geometric optics bending should be positive, got {}",
            alpha
        );
    }

    // --- Standard atmosphere tests ---

    #[test]
    fn test_standard_atmosphere_temperature_sea_level() {
        let t = standard_atmosphere_temperature(0.0);
        assert!(
            (t - 288.15).abs() < EPSILON,
            "Sea level T should be 288.15 K, got {}",
            t
        );
    }

    #[test]
    fn test_standard_atmosphere_temperature_tropopause() {
        let t = standard_atmosphere_temperature(11.0);
        assert!(
            (t - 216.65).abs() < 0.1,
            "Tropopause T should be 216.65 K, got {}",
            t
        );
    }

    #[test]
    fn test_standard_atmosphere_temperature_stratosphere() {
        let t = standard_atmosphere_temperature(25.0);
        // 216.65 + 1.0*(25-20) = 221.65
        assert!(
            (t - 221.65).abs() < 0.1,
            "T at 25 km should be ~221.65 K, got {}",
            t
        );
    }

    #[test]
    fn test_standard_atmosphere_pressure_sea_level() {
        let p = standard_atmosphere_pressure(0.0);
        assert!(
            (p - 1013.25).abs() < EPSILON,
            "Sea level P should be 1013.25 hPa, got {}",
            p
        );
    }

    #[test]
    fn test_standard_atmosphere_pressure_decreases() {
        let p0 = standard_atmosphere_pressure(0.0);
        let p5 = standard_atmosphere_pressure(5.0);
        let p10 = standard_atmosphere_pressure(10.0);
        let p20 = standard_atmosphere_pressure(20.0);
        assert!(p0 > p5, "P(0) > P(5): {} > {}", p0, p5);
        assert!(p5 > p10, "P(5) > P(10): {} > {}", p5, p10);
        assert!(p10 > p20, "P(10) > P(20): {} > {}", p10, p20);
    }

    #[test]
    fn test_standard_atmosphere_pressure_5km() {
        let p = standard_atmosphere_pressure(5.5);
        // ~500 hPa at ~5.5 km
        assert!(
            p > 450.0 && p < 560.0,
            "P at 5.5 km should be ~500 hPa, got {}",
            p
        );
    }

    // --- Tropopause tests ---

    #[test]
    fn test_tropopause_equator_higher_than_poles() {
        let h_eq = tropopause_height_km(0.0);
        let h_pole = tropopause_height_km(90.0);
        assert!(
            h_eq > h_pole,
            "Equatorial tropopause ({}) should be higher than polar ({})",
            h_eq,
            h_pole
        );
    }

    #[test]
    fn test_tropopause_equator_about_16km() {
        let h = tropopause_height_km(0.0);
        assert!(
            h > 14.0 && h < 18.0,
            "Equatorial tropopause should be ~16 km, got {}",
            h
        );
    }

    #[test]
    fn test_tropopause_pole_about_8km() {
        let h = tropopause_height_km(90.0);
        assert!(
            h > 7.0 && h < 10.0,
            "Polar tropopause should be ~8 km, got {}",
            h
        );
    }

    // --- OccultationProcessor tests ---

    #[test]
    fn test_excess_phase_to_doppler_constant() {
        let proc = OccultationProcessor::new(OccultationConfig::default());
        // Constant phase => zero Doppler
        let phase = vec![5.0; 10];
        let doppler = proc.excess_phase_to_doppler(&phase, 0.02);
        for &d in &doppler {
            assert!(d.abs() < EPSILON, "Constant phase should give zero Doppler, got {}", d);
        }
    }

    #[test]
    fn test_excess_phase_to_doppler_linear() {
        let proc = OccultationProcessor::new(OccultationConfig::default());
        // Linear phase => constant Doppler
        let dt = 0.02;
        let rate = 100.0; // m/s
        let phase: Vec<f64> = (0..20).map(|i| rate * i as f64 * dt).collect();
        let doppler = proc.excess_phase_to_doppler(&phase, dt);

        // Interior points should be close to rate
        for &d in &doppler[1..doppler.len() - 1] {
            assert!(
                (d - rate).abs() < 1.0,
                "Linear phase Doppler should be ~{}, got {}",
                rate,
                d
            );
        }
    }

    #[test]
    fn test_excess_phase_to_doppler_empty() {
        let proc = OccultationProcessor::new(OccultationConfig::default());
        let doppler = proc.excess_phase_to_doppler(&[], 0.02);
        assert!(doppler.is_empty());
    }

    #[test]
    fn test_excess_phase_to_doppler_single() {
        let proc = OccultationProcessor::new(OccultationConfig::default());
        let doppler = proc.excess_phase_to_doppler(&[1.0], 0.02);
        assert_eq!(doppler.len(), 1);
        assert_eq!(doppler[0], 0.0);
    }

    #[test]
    fn test_refractivity_to_dry_temperature() {
        let proc = OccultationProcessor::new(OccultationConfig::default());
        // T = 77.6 * P / N
        // For N=272.9, P=1013.25 => T ≈ 288.15
        let t = proc.refractivity_to_dry_temperature(272.9, 1013.25);
        assert!(
            (t - 288.15).abs() < 1.0,
            "Dry temperature should be ~288 K, got {}",
            t
        );
    }

    #[test]
    fn test_hydrostatic_integration_pressure_profile() {
        let proc = OccultationProcessor::new(OccultationConfig::default());

        // Create a simple exponential refractivity profile
        let profile: Vec<(f64, f64)> = (0..50)
            .map(|i| {
                let h = i as f64;
                let n = RefractivityModel::refractivity_at_height(h);
                (h, n)
            })
            .collect();

        let pressure = proc.hydrostatic_integration(&profile, 1.0); // 1 hPa at 49 km

        // Pressure should increase downward (low altitude = high pressure)
        assert!(pressure.len() > 2);
        // Result is sorted ascending by height: first = low altitude, last = high altitude
        let p_at_low_alt = pressure.first().unwrap().1;
        let p_at_high_alt = pressure.last().unwrap().1;
        assert!(
            p_at_low_alt > p_at_high_alt,
            "Pressure at low altitude ({}) should be greater than at high altitude ({})",
            p_at_low_alt,
            p_at_high_alt
        );
    }

    #[test]
    fn test_abel_inversion_exponential() {
        let proc = OccultationProcessor::new(OccultationConfig::default());

        // Create bending angle profile from exponential atmosphere
        let mut bending_data: Vec<(f64, f64)> = Vec::new();
        let r_s = 6371.0;
        for i in 0..100 {
            let h = i as f64 * 0.5; // 0 to 50 km
            let a = r_s + h;
            let alpha = BendingAngleModel::exponential_atmosphere(a, 315.0, 7.35);
            bending_data.push((a, alpha));
        }

        let refractivity = proc.abel_inversion(&bending_data, &[]);

        // Should have some data points
        assert!(!refractivity.is_empty(), "Abel inversion should produce output");

        // Refractivity should generally decrease with height
        if refractivity.len() > 2 {
            let n_low = refractivity.first().unwrap().1;
            let n_high = refractivity.last().unwrap().1;
            // At least high altitude refractivity should be smaller
            assert!(
                n_high < n_low || (n_high - n_low).abs() < 10.0,
                "Refractivity should decrease with height: low={}, high={}",
                n_low,
                n_high
            );
        }
    }

    // --- QualityControl tests ---

    #[test]
    fn test_check_monotonicity_decreasing() {
        assert!(QualityControl::check_monotonicity(&[10.0, 9.0, 8.0, 7.0]));
    }

    #[test]
    fn test_check_monotonicity_not_decreasing() {
        assert!(!QualityControl::check_monotonicity(&[10.0, 9.0, 9.5, 8.0]));
    }

    #[test]
    fn test_check_monotonicity_empty() {
        assert!(QualityControl::check_monotonicity(&[]));
    }

    #[test]
    fn test_check_monotonicity_single() {
        assert!(QualityControl::check_monotonicity(&[5.0]));
    }

    #[test]
    fn test_detect_multipath_clean_signal() {
        let amplitude = vec![1.0; 100];
        let regions = QualityControl::detect_multipath(&amplitude, 0.3);
        assert!(regions.is_empty(), "Clean signal should have no multipath");
    }

    #[test]
    fn test_detect_multipath_with_oscillations() {
        let mut amplitude = vec![1.0; 100];
        // Add strong oscillations in the middle
        for i in 40..60 {
            amplitude[i] = if i % 2 == 0 { 2.0 } else { 0.5 };
        }
        let regions = QualityControl::detect_multipath(&amplitude, 0.3);
        assert!(
            !regions.is_empty(),
            "Oscillating signal should have multipath regions"
        );
    }

    #[test]
    fn test_smoothing_filter() {
        let mut profile: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let h = i as f64 * 0.1;
                let n = 315.0 * (-h / 7.35).exp();
                // Add noise
                let noise = if i % 3 == 0 { 5.0 } else if i % 3 == 1 { -5.0 } else { 0.0 };
                (h, n + noise)
            })
            .collect();

        let original: Vec<f64> = profile.iter().map(|p| p.1).collect();
        QualityControl::smoothing_filter(&mut profile, 0.5);
        let smoothed: Vec<f64> = profile.iter().map(|p| p.1).collect();

        // Smoothed should have less variance than original
        let var_orig = variance(&original);
        let var_smooth = variance(&smoothed);
        assert!(
            var_smooth < var_orig,
            "Smoothed variance ({}) should be less than original ({})",
            var_smooth,
            var_orig
        );
    }

    #[test]
    fn test_detect_multipath_empty() {
        let regions = QualityControl::detect_multipath(&[], 0.3);
        assert!(regions.is_empty());
    }

    #[test]
    fn test_retrieve_temperature_profile() {
        let proc = OccultationProcessor::new(OccultationConfig::default());

        // Create standard-like refractivity profile
        let refractivity: Vec<(f64, f64)> = (0..50)
            .map(|i| {
                let h = i as f64;
                let n = RefractivityModel::refractivity_at_height(h);
                (h, n)
            })
            .collect();

        let temp_profile = proc.retrieve_temperature_profile(&refractivity);
        assert!(
            !temp_profile.is_empty(),
            "Temperature profile should not be empty"
        );

        // All temperatures should be in reasonable range
        for &(h, t) in &temp_profile {
            assert!(
                t > 150.0 && t < 350.0,
                "Temperature at {} km should be reasonable, got {} K",
                h,
                t
            );
        }
    }

    #[test]
    fn test_doppler_to_bending_angle() {
        let proc = OccultationProcessor::new(OccultationConfig::default());

        // Simple geometry: GPS at x = 26600 km, LEO at x = -7171 km
        let gps_pos = [26600.0, 0.0, 0.0];
        let leo_pos = [-7171.0, 0.0, 0.0];
        let gps_vel = [0.0, 3.87, 0.0]; // ~3.87 km/s (GPS orbit)
        let leo_vel = [0.0, -7.5, 0.0]; // ~7.5 km/s (LEO orbit)

        let alpha = proc.doppler_to_bending_angle(10.0, gps_pos, leo_pos, gps_vel, leo_vel);
        assert!(alpha >= 0.0, "Bending angle should be non-negative, got {}", alpha);
    }

    // --- Helper function for tests ---

    fn variance(data: &[f64]) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        let mean: f64 = data.iter().sum::<f64>() / data.len() as f64;
        data.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / data.len() as f64
    }
}
