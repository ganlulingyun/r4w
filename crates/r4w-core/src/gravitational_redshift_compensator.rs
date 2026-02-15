//! # Gravitational Redshift Compensator
//!
//! Implements relativistic frequency correction for satellite-based navigation
//! and communication systems. General Relativity predicts that clocks at higher
//! gravitational potential tick faster than clocks on the Earth's surface.
//!
//! ## Physics Background
//!
//! GPS satellites at ~20,200 km altitude experience a combined relativistic
//! clock drift of approximately +38 microseconds per day:
//!
//! - **Gravitational blueshift (GR)**: ~+45.850 us/day (clocks tick faster at higher altitude)
//! - **Special relativistic dilation (SR)**: ~-7.214 us/day (clocks tick slower at higher velocity)
//! - **Net effect**: ~+38.6 us/day
//!
//! Without correction, this would produce ~11 km/day of positioning error.
//!
//! ## GPS Factory Offset
//!
//! To compensate for the constant relativistic drift, GPS satellite clocks are
//! manufactured to run at 10.22999999543 MHz instead of the nominal 10.23 MHz.
//! This corresponds to a fractional frequency offset of approximately
//! -4.465 x 10^-10.
//!
//! ## Eccentric Orbit Corrections
//!
//! For non-circular orbits, a periodic relativistic correction is applied:
//!
//! ```text
//! dt_rel = -2 * sqrt(GM * a) / c^2 * e * sin(E)
//! ```
//!
//! where `a` is the semi-major axis, `e` is eccentricity, and `E` is the
//! eccentric anomaly.
//!
//! ## Shapiro Delay
//!
//! Signal propagation near massive bodies is delayed by the Shapiro effect:
//!
//! ```text
//! dt_shapiro = (2 * GM / c^3) * ln((r1 + r2 + d) / (r1 + r2 - d))
//! ```
//!
//! ## Usage
//!
//! ```rust
//! use r4w_core::gravitational_redshift_compensator::{
//!     GravitationalRedshiftCompensator, GravitationalRedshiftConfig,
//!     SatelliteClockModel, gps_preset, schwarzschild_radius,
//! };
//!
//! // Use GPS preset
//! let config = gps_preset();
//! let comp = GravitationalRedshiftCompensator::new(config);
//!
//! // Compute daily drift at GPS altitude
//! let drift = comp.daily_drift_seconds(20_200_000.0);
//! assert!((drift - 38.6e-6).abs() < 1.0e-6);
//!
//! // Schwarzschild radius of Earth
//! let rs = schwarzschild_radius(5.972e24);
//! assert!((rs - 0.00887).abs() < 0.0001);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Standard gravitational parameter for Earth (m^3/s^2)
const EARTH_GM: f64 = 3.986004418e14;

/// Mean Earth radius (m)
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// Speed of light in vacuum (m/s)
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Seconds per day
const SECONDS_PER_DAY: f64 = 86_400.0;

/// GPS L1 carrier frequency (Hz)
const GPS_L1_FREQ: f64 = 1_575.42e6;

/// Galileo E1 carrier frequency (Hz)
const GALILEO_E1_FREQ: f64 = 1_575.42e6;

/// GLONASS L1 center frequency (Hz) - channel 0
const GLONASS_L1_FREQ: f64 = 1_602.0e6;

// ---------------------------------------------------------------------------
// ReferenceFrame
// ---------------------------------------------------------------------------

/// Reference frame for relativistic calculations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReferenceFrame {
    /// Calculations referenced to the Earth's surface (ground observer).
    EarthCentered,
    /// Calculations referenced to the satellite frame.
    SatelliteCentered,
}

// ---------------------------------------------------------------------------
// GravitationalRedshiftConfig
// ---------------------------------------------------------------------------

/// Configuration for the gravitational redshift compensator.
#[derive(Debug, Clone)]
pub struct GravitationalRedshiftConfig {
    /// Satellite orbital altitude above Earth's surface (m).
    pub orbital_altitude_m: f64,
    /// Mean Earth radius (m). Default: 6,371,000.
    pub earth_radius_m: f64,
    /// Earth's standard gravitational parameter GM (m^3/s^2).
    /// Default: 3.986004418e14.
    pub earth_gm: f64,
    /// Speed of light (m/s). Default: 299,792,458.
    pub speed_of_light: f64,
    /// Carrier frequency of the signal (Hz), e.g. 1575.42e6 for GPS L1.
    pub carrier_frequency_hz: f64,
    /// Include special-relativistic time dilation. Default: true.
    pub include_special_relativity: bool,
    /// Include general-relativistic gravitational time dilation. Default: true.
    pub include_general_relativity: bool,
    /// Orbital eccentricity (0.0 for circular orbit). Default: 0.0.
    pub orbital_eccentricity: f64,
    /// Reference frame for the computation. Default: EarthCentered.
    pub reference_frame: ReferenceFrame,
}

impl Default for GravitationalRedshiftConfig {
    fn default() -> Self {
        Self {
            orbital_altitude_m: 20_200_000.0, // GPS MEO
            earth_radius_m: EARTH_RADIUS_M,
            earth_gm: EARTH_GM,
            speed_of_light: SPEED_OF_LIGHT,
            carrier_frequency_hz: GPS_L1_FREQ,
            include_special_relativity: true,
            include_general_relativity: true,
            orbital_eccentricity: 0.0,
            reference_frame: ReferenceFrame::EarthCentered,
        }
    }
}

// ---------------------------------------------------------------------------
// GravitationalRedshiftCompensator
// ---------------------------------------------------------------------------

/// Computes relativistic frequency and clock corrections for satellite systems.
///
/// The compensator models both general-relativistic gravitational time dilation
/// and special-relativistic velocity-based time dilation to produce net
/// fractional frequency offsets, daily clock drifts, corrected frequencies,
/// and IQ sample phase corrections.
pub struct GravitationalRedshiftCompensator {
    config: GravitationalRedshiftConfig,
}

impl GravitationalRedshiftCompensator {
    /// Create a new compensator with the given configuration.
    pub fn new(config: GravitationalRedshiftConfig) -> Self {
        Self { config }
    }

    /// Return the gravitational potential at radius `radius_m` from Earth's
    /// center.
    ///
    /// Phi(r) = -GM / r
    pub fn gravitational_potential(&self, radius_m: f64) -> f64 {
        -self.config.earth_gm / radius_m
    }

    /// Compute the gravitational (GR) time-dilation factor at `altitude_m`
    /// above Earth's surface, relative to a clock on the surface.
    ///
    /// In the weak-field limit the fractional rate difference is:
    ///
    /// ```text
    /// (f_sat - f_ground) / f_ground = (Phi_ground - Phi_sat) / c^2
    ///                                = GM/c^2 * (1/R_e - 1/(R_e + h))
    /// ```
    ///
    /// A positive value means the satellite clock ticks faster.
    pub fn gravitational_time_dilation(&self, altitude_m: f64) -> f64 {
        let c2 = self.config.speed_of_light * self.config.speed_of_light;
        let r_earth = self.config.earth_radius_m;
        let r_sat = r_earth + altitude_m;
        self.config.earth_gm / c2 * (1.0 / r_earth - 1.0 / r_sat)
    }

    /// Compute the special-relativistic (SR) time-dilation factor for a
    /// clock moving at `velocity_ms`.
    ///
    /// ```text
    /// ratio = sqrt(1 - v^2/c^2)
    /// ```
    ///
    /// Returns a value slightly less than 1.0 (moving clock runs slow).
    pub fn special_relativistic_dilation(&self, velocity_ms: f64) -> f64 {
        let beta2 =
            (velocity_ms * velocity_ms) / (self.config.speed_of_light * self.config.speed_of_light);
        (1.0 - beta2).sqrt()
    }

    /// Compute the circular orbital velocity at `altitude_m` above Earth's
    /// surface.
    ///
    /// v = sqrt(GM / r)
    pub fn orbital_velocity(&self, altitude_m: f64) -> f64 {
        let r = self.config.earth_radius_m + altitude_m;
        (self.config.earth_gm / r).sqrt()
    }

    /// Compute the combined (GR + SR) fractional clock-rate offset at
    /// `altitude_m`, relative to a clock on the Earth's surface.
    ///
    /// Positive means the satellite clock is fast.
    ///
    /// The GR contribution (gravitational blueshift) makes the satellite
    /// clock tick faster. The SR contribution (velocity time dilation) makes
    /// the satellite clock tick slower. The net is computed as:
    ///
    /// ```text
    /// delta_f / f = GR_term - SR_term
    /// ```
    ///
    /// where:
    /// - GR_term = GM/c^2 * (1/R_e - 1/r_sat)
    /// - SR_term = v^2 / (2 * c^2)   (first-order approximation)
    pub fn combined_clock_rate(&self, altitude_m: f64) -> f64 {
        let c2 = self.config.speed_of_light * self.config.speed_of_light;
        let mut offset = 0.0;

        if self.config.include_general_relativity {
            offset += self.gravitational_time_dilation(altitude_m);
        }

        if self.config.include_special_relativity {
            let v = self.orbital_velocity(altitude_m);
            // SR slows the clock: subtract v^2 / (2c^2)
            offset -= (v * v) / (2.0 * c2);
        }

        match self.config.reference_frame {
            ReferenceFrame::EarthCentered => offset,
            ReferenceFrame::SatelliteCentered => -offset,
        }
    }

    /// Compute the cumulative clock drift in seconds per day at `altitude_m`.
    ///
    /// A positive value means the satellite clock gains time relative to
    /// the ground clock.
    pub fn daily_drift_seconds(&self, altitude_m: f64) -> f64 {
        self.combined_clock_rate(altitude_m) * SECONDS_PER_DAY
    }

    /// Compute the corrected carrier frequency observed on the ground for a
    /// signal transmitted from `altitude_m`.
    ///
    /// ```text
    /// f_corrected = nominal_freq * (1 + delta_f/f)
    /// ```
    pub fn frequency_correction(&self, nominal_freq_hz: f64, altitude_m: f64) -> f64 {
        nominal_freq_hz * (1.0 + self.combined_clock_rate(altitude_m))
    }

    /// Compute the periodic relativistic correction for an eccentric orbit.
    ///
    /// ```text
    /// dt_rel = -2 * sqrt(GM * a) / c^2 * e * sin(E)
    /// ```
    ///
    /// where `a` is the semi-major axis (earth_radius + altitude for the
    /// nominal orbit), `e` is eccentricity, and `E` is the eccentric anomaly.
    ///
    /// Returns the correction in seconds.
    pub fn eccentric_anomaly_correction(
        &self,
        eccentricity: f64,
        eccentric_anomaly_rad: f64,
    ) -> f64 {
        let a = self.config.earth_radius_m + self.config.orbital_altitude_m;
        let c2 = self.config.speed_of_light * self.config.speed_of_light;
        -2.0 * (self.config.earth_gm * a).sqrt() / c2 * eccentricity * eccentric_anomaly_rad.sin()
    }

    /// Apply relativistic phase correction to IQ samples.
    ///
    /// For a clock running fast by fractional offset `delta`, each sample
    /// accumulated over `dt_s` seconds acquires additional phase:
    ///
    /// ```text
    /// dphi = 2 * pi * carrier_freq * delta * dt_s
    /// ```
    ///
    /// This method rotates each sample by the accumulated phase.
    pub fn compensate_iq_samples(&self, samples: &mut [(f64, f64)], dt_s: f64) {
        let delta = self.combined_clock_rate(self.config.orbital_altitude_m);
        let dphi_per_sample = 2.0 * PI * self.config.carrier_frequency_hz * delta * dt_s;

        for (i, sample) in samples.iter_mut().enumerate() {
            let phi = dphi_per_sample * (i as f64);
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();
            let (re, im) = *sample;
            sample.0 = re * cos_phi - im * sin_phi;
            sample.1 = re * sin_phi + im * cos_phi;
        }
    }
}

// ---------------------------------------------------------------------------
// SatelliteClockModel
// ---------------------------------------------------------------------------

/// Models the satellite on-board clock including factory offset and periodic
/// relativistic corrections.
///
/// GPS satellites intentionally lower their clock frequency before launch to
/// compensate for the constant relativistic drift experienced in orbit.
pub struct SatelliteClockModel {
    altitude_m: f64,
    eccentricity: f64,
    compensator: GravitationalRedshiftCompensator,
}

impl SatelliteClockModel {
    /// Create a new satellite clock model for the given orbital parameters.
    pub fn new(altitude_m: f64, eccentricity: f64) -> Self {
        let config = GravitationalRedshiftConfig {
            orbital_altitude_m: altitude_m,
            orbital_eccentricity: eccentricity,
            ..Default::default()
        };
        let compensator = GravitationalRedshiftCompensator::new(config);
        Self {
            altitude_m,
            eccentricity,
            compensator,
        }
    }

    /// Compute the factory frequency offset applied before launch.
    ///
    /// GPS uses 10.22999999543 MHz instead of 10.23 MHz. This method
    /// returns the corrected frequency:
    ///
    /// ```text
    /// f_factory = nominal_freq * (1 - delta)
    /// ```
    ///
    /// where `delta` is the combined relativistic fractional offset.
    pub fn factory_offset_hz(&self, nominal_freq_hz: f64) -> f64 {
        let delta = self.compensator.combined_clock_rate(self.altitude_m);
        nominal_freq_hz * (1.0 - delta)
    }

    /// Compute the total clock correction at elapsed time `t_seconds`.
    ///
    /// For a circular orbit (eccentricity = 0), this is simply:
    ///
    /// ```text
    /// dt = delta * t
    /// ```
    ///
    /// For an eccentric orbit, a periodic term from
    /// `eccentric_anomaly_correction` would be added. Here we return
    /// just the secular (constant-rate) drift.
    pub fn clock_correction_at_time(&self, t_seconds: f64) -> f64 {
        let delta = self.compensator.combined_clock_rate(self.altitude_m);
        delta * t_seconds
    }

    /// Compute the periodic relativistic correction for an eccentric orbit.
    ///
    /// Uses the Kepler equation relation: the periodic term is
    ///
    /// ```text
    /// dt_periodic = -2 * sqrt(GM * a) / c^2 * e * sin(E)
    /// ```
    ///
    /// Here we approximate `E` from `mean_anomaly` using a single Newton
    /// iteration of Kepler's equation: `E = M + e * sin(M)` (first-order).
    pub fn periodic_correction(&self, eccentricity: f64, mean_anomaly: f64) -> f64 {
        // First-order Kepler equation solution
        let eccentric_anomaly = mean_anomaly + eccentricity * mean_anomaly.sin();
        self.compensator
            .eccentric_anomaly_correction(eccentricity, eccentric_anomaly)
    }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

/// GPS constellation preset (MEO ~20,200 km, L1 1575.42 MHz).
pub fn gps_preset() -> GravitationalRedshiftConfig {
    GravitationalRedshiftConfig {
        orbital_altitude_m: 20_200_000.0,
        earth_radius_m: EARTH_RADIUS_M,
        earth_gm: EARTH_GM,
        speed_of_light: SPEED_OF_LIGHT,
        carrier_frequency_hz: GPS_L1_FREQ,
        include_special_relativity: true,
        include_general_relativity: true,
        orbital_eccentricity: 0.02, // typical GPS eccentricity
        reference_frame: ReferenceFrame::EarthCentered,
    }
}

/// Galileo constellation preset (MEO ~23,222 km, E1 1575.42 MHz).
pub fn galileo_preset() -> GravitationalRedshiftConfig {
    GravitationalRedshiftConfig {
        orbital_altitude_m: 23_222_000.0,
        earth_radius_m: EARTH_RADIUS_M,
        earth_gm: EARTH_GM,
        speed_of_light: SPEED_OF_LIGHT,
        carrier_frequency_hz: GALILEO_E1_FREQ,
        include_special_relativity: true,
        include_general_relativity: true,
        orbital_eccentricity: 0.0,
        reference_frame: ReferenceFrame::EarthCentered,
    }
}

/// GLONASS constellation preset (MEO ~19,100 km, L1 1602.0 MHz).
pub fn glonass_preset() -> GravitationalRedshiftConfig {
    GravitationalRedshiftConfig {
        orbital_altitude_m: 19_100_000.0,
        earth_radius_m: EARTH_RADIUS_M,
        earth_gm: EARTH_GM,
        speed_of_light: SPEED_OF_LIGHT,
        carrier_frequency_hz: GLONASS_L1_FREQ,
        include_special_relativity: true,
        include_general_relativity: true,
        orbital_eccentricity: 0.0,
        reference_frame: ReferenceFrame::EarthCentered,
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the Schwarzschild radius for a body of given mass.
///
/// ```text
/// r_s = 2 * G * M / c^2
/// ```
///
/// For Earth (M = 5.972e24 kg), r_s ~ 8.87 mm.
///
/// Note: we use G = GM_earth / M_earth to get G, but to keep this function
/// general we accept `mass_kg` and use the gravitational constant
/// G = 6.674e-11 m^3 kg^-1 s^-2.
pub fn schwarzschild_radius(mass_kg: f64) -> f64 {
    const G: f64 = 6.67430e-11; // gravitational constant
    2.0 * G * mass_kg / (SPEED_OF_LIGHT * SPEED_OF_LIGHT)
}

/// Compute the gravitational frequency shift for a signal of frequency `f0`
/// experiencing a gravitational potential difference `potential_diff`.
///
/// ```text
/// f_shifted = f0 * (1 + delta_phi / c^2)
/// ```
///
/// `potential_diff` is Phi_receiver - Phi_transmitter (positive if receiver
/// is at higher potential, i.e. blueshift).
pub fn gravitational_frequency_shift(f0: f64, potential_diff: f64) -> f64 {
    f0 * (1.0 + potential_diff / (SPEED_OF_LIGHT * SPEED_OF_LIGHT))
}

/// Compute the Shapiro delay for a signal traveling between two points at
/// radii `r1` and `r2` from Earth's center, with closest approach distance
/// `closest_approach` to the center of mass.
///
/// ```text
/// dt_shapiro = (2 * GM / c^3) * ln((r1 + r2 + d) / (r1 + r2 - d))
/// ```
///
/// where `d` is the straight-line distance between the two points.
///
/// Returns the additional delay in seconds.
pub fn shapiro_delay(r1: f64, r2: f64, closest_approach: f64) -> f64 {
    let c3 = SPEED_OF_LIGHT * SPEED_OF_LIGHT * SPEED_OF_LIGHT;
    let d = ((r1 * r1 - closest_approach * closest_approach).max(0.0).sqrt()
        + (r2 * r2 - closest_approach * closest_approach).max(0.0).sqrt());
    let numerator = r1 + r2 + d;
    let denominator = r1 + r2 - d;
    if denominator <= 0.0 {
        return 0.0;
    }
    (2.0 * EARTH_GM / c3) * (numerator / denominator).ln()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-12;

    // --- GravitationalRedshiftConfig defaults ---

    #[test]
    fn test_default_config() {
        let cfg = GravitationalRedshiftConfig::default();
        assert!((cfg.orbital_altitude_m - 20_200_000.0).abs() < TOLERANCE);
        assert!((cfg.earth_radius_m - 6_371_000.0).abs() < TOLERANCE);
        assert!((cfg.earth_gm - 3.986004418e14).abs() < 1.0);
        assert!((cfg.speed_of_light - 299_792_458.0).abs() < TOLERANCE);
        assert!((cfg.carrier_frequency_hz - 1_575.42e6).abs() < 1.0);
        assert!(cfg.include_special_relativity);
        assert!(cfg.include_general_relativity);
        assert!((cfg.orbital_eccentricity).abs() < TOLERANCE);
        assert_eq!(cfg.reference_frame, ReferenceFrame::EarthCentered);
    }

    // --- Gravitational potential ---

    #[test]
    fn test_gravitational_potential_surface() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let phi = comp.gravitational_potential(EARTH_RADIUS_M);
        // -GM/R ~ -6.25e7 J/kg
        assert!(phi < -6.0e7);
        assert!(phi > -7.0e7);
    }

    #[test]
    fn test_gravitational_potential_higher_altitude() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let phi_surface = comp.gravitational_potential(EARTH_RADIUS_M);
        let phi_orbit = comp.gravitational_potential(EARTH_RADIUS_M + 20_200_000.0);
        // Potential is less negative at higher altitude (weaker field)
        assert!(phi_orbit > phi_surface);
    }

    // --- Gravitational time dilation (GR) ---

    #[test]
    fn test_gr_dilation_zero_altitude() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let dilation = comp.gravitational_time_dilation(0.0);
        assert!(dilation.abs() < 1e-20, "Should be zero at surface");
    }

    #[test]
    fn test_gr_dilation_gps_altitude() {
        // GR blueshift at GPS altitude should be ~5.3e-10 fractional
        // which gives ~45.7 us/day
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let dilation = comp.gravitational_time_dilation(20_200_000.0);
        let us_per_day = dilation * SECONDS_PER_DAY * 1e6;
        // Should be approximately 45-46 us/day (positive = clock runs fast)
        assert!(
            us_per_day > 44.0 && us_per_day < 47.0,
            "GR blueshift should be ~45.8 us/day, got {} us/day",
            us_per_day
        );
    }

    #[test]
    fn test_gr_dilation_positive() {
        // GR effect makes satellite clock tick FASTER (positive offset)
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let dilation = comp.gravitational_time_dilation(20_200_000.0);
        assert!(dilation > 0.0, "GR dilation should be positive (blueshift)");
    }

    // --- Special relativistic dilation (SR) ---

    #[test]
    fn test_sr_dilation_zero_velocity() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let ratio = comp.special_relativistic_dilation(0.0);
        assert!((ratio - 1.0).abs() < TOLERANCE, "At rest, ratio should be 1.0");
    }

    #[test]
    fn test_sr_dilation_gps_velocity() {
        // GPS orbital velocity ~3874 m/s
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let v = comp.orbital_velocity(20_200_000.0);
        let ratio = comp.special_relativistic_dilation(v);
        // Should be slightly less than 1.0 (clock runs slow)
        assert!(ratio < 1.0, "Moving clock should run slow");
        assert!(ratio > 0.99999999, "Effect is tiny");
    }

    #[test]
    fn test_sr_daily_drift_gps() {
        // SR at GPS altitude should produce ~-7.2 us/day
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let v = comp.orbital_velocity(20_200_000.0);
        let c2 = SPEED_OF_LIGHT * SPEED_OF_LIGHT;
        let sr_fractional = -(v * v) / (2.0 * c2);
        let us_per_day = sr_fractional * SECONDS_PER_DAY * 1e6;
        assert!(
            us_per_day > -8.0 && us_per_day < -6.5,
            "SR dilation should be ~-7.2 us/day, got {} us/day",
            us_per_day
        );
    }

    // --- Orbital velocity ---

    #[test]
    fn test_orbital_velocity_gps() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let v = comp.orbital_velocity(20_200_000.0);
        // GPS velocity ~3874 m/s
        assert!(
            v > 3800.0 && v < 3950.0,
            "GPS orbital velocity should be ~3874 m/s, got {} m/s",
            v
        );
    }

    #[test]
    fn test_orbital_velocity_leo() {
        // LEO at 400 km: ~7672 m/s
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let v = comp.orbital_velocity(400_000.0);
        assert!(
            v > 7600.0 && v < 7800.0,
            "LEO velocity should be ~7672 m/s, got {} m/s",
            v
        );
    }

    #[test]
    fn test_orbital_velocity_decreases_with_altitude() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let v_leo = comp.orbital_velocity(400_000.0);
        let v_meo = comp.orbital_velocity(20_200_000.0);
        let v_geo = comp.orbital_velocity(35_786_000.0);
        assert!(v_leo > v_meo);
        assert!(v_meo > v_geo);
    }

    // --- Combined clock rate ---

    #[test]
    fn test_combined_gps_drift_38us() {
        // Net GPS drift should be ~38.6 us/day
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let drift_us = comp.daily_drift_seconds(20_200_000.0) * 1e6;
        assert!(
            drift_us > 37.0 && drift_us < 40.0,
            "GPS combined drift should be ~38.6 us/day, got {} us/day",
            drift_us
        );
    }

    #[test]
    fn test_combined_clock_rate_positive_for_gps() {
        // Net effect at GPS altitude is positive (satellite clock is fast)
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let rate = comp.combined_clock_rate(20_200_000.0);
        assert!(rate > 0.0, "Net relativistic offset should be positive for GPS");
    }

    #[test]
    fn test_combined_zero_at_surface() {
        // At altitude 0, both GR and SR terms should be ~0
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        // At surface, we need the surface velocity to also be zero for this test.
        // Actually orbital_velocity(0) gives the surface circular velocity which
        // is nonzero. But the SR term would represent actual orbital motion.
        // At altitude 0, GR = 0, and v = sqrt(GM/Re) ~ 7905 m/s if orbiting.
        // For a stationary ground clock, the combined rate should reflect that
        // the GR effect is zero at surface.
        let gr_only = GravitationalRedshiftConfig {
            include_special_relativity: false,
            ..Default::default()
        };
        let comp = GravitationalRedshiftCompensator::new(gr_only);
        let rate = comp.combined_clock_rate(0.0);
        assert!(rate.abs() < 1e-20, "GR at surface should be zero");
    }

    #[test]
    fn test_gr_only_mode() {
        let config = GravitationalRedshiftConfig {
            include_special_relativity: false,
            include_general_relativity: true,
            ..Default::default()
        };
        let comp = GravitationalRedshiftCompensator::new(config);
        let drift_us = comp.daily_drift_seconds(20_200_000.0) * 1e6;
        // GR only: ~45.8 us/day
        assert!(
            drift_us > 44.0 && drift_us < 47.0,
            "GR-only drift should be ~45.8 us/day, got {} us/day",
            drift_us
        );
    }

    #[test]
    fn test_sr_only_mode() {
        let config = GravitationalRedshiftConfig {
            include_special_relativity: true,
            include_general_relativity: false,
            ..Default::default()
        };
        let comp = GravitationalRedshiftCompensator::new(config);
        let drift_us = comp.daily_drift_seconds(20_200_000.0) * 1e6;
        // SR only: ~-7.2 us/day
        assert!(
            drift_us > -8.0 && drift_us < -6.5,
            "SR-only drift should be ~-7.2 us/day, got {} us/day",
            drift_us
        );
    }

    #[test]
    fn test_neither_gr_nor_sr() {
        let config = GravitationalRedshiftConfig {
            include_special_relativity: false,
            include_general_relativity: false,
            ..Default::default()
        };
        let comp = GravitationalRedshiftCompensator::new(config);
        let rate = comp.combined_clock_rate(20_200_000.0);
        assert!(rate.abs() < TOLERANCE, "No effects enabled should give zero offset");
    }

    #[test]
    fn test_satellite_centered_frame() {
        let earth_config = GravitationalRedshiftConfig {
            reference_frame: ReferenceFrame::EarthCentered,
            ..Default::default()
        };
        let sat_config = GravitationalRedshiftConfig {
            reference_frame: ReferenceFrame::SatelliteCentered,
            ..Default::default()
        };
        let comp_earth = GravitationalRedshiftCompensator::new(earth_config);
        let comp_sat = GravitationalRedshiftCompensator::new(sat_config);

        let rate_earth = comp_earth.combined_clock_rate(20_200_000.0);
        let rate_sat = comp_sat.combined_clock_rate(20_200_000.0);

        assert!(
            (rate_earth + rate_sat).abs() < TOLERANCE,
            "Satellite-centered should be negative of Earth-centered"
        );
    }

    // --- Frequency correction ---

    #[test]
    fn test_frequency_correction_gps_l1() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let f_corrected = comp.frequency_correction(GPS_L1_FREQ, 20_200_000.0);
        // Satellite clock is fast, so observed frequency is slightly higher
        assert!(f_corrected > GPS_L1_FREQ);
        // The shift should be on the order of ~0.7 Hz
        let shift = f_corrected - GPS_L1_FREQ;
        assert!(shift > 0.5 && shift < 1.0, "Frequency shift ~0.7 Hz, got {}", shift);
    }

    // --- GPS factory offset ---

    #[test]
    fn test_gps_factory_offset() {
        // GPS uses 10.22999999543 MHz instead of 10.23 MHz
        // Fractional offset = -4.4647e-10
        let model = SatelliteClockModel::new(20_200_000.0, 0.0);
        let f_factory = model.factory_offset_hz(10.23e6);
        let fractional = (f_factory - 10.23e6) / 10.23e6;
        // Should be approximately -4.465e-10
        assert!(
            fractional < -4.0e-10 && fractional > -5.0e-10,
            "GPS factory offset should be ~-4.465e-10, got {}",
            fractional
        );
    }

    #[test]
    fn test_gps_factory_offset_10_23mhz() {
        let model = SatelliteClockModel::new(20_200_000.0, 0.0);
        let f = model.factory_offset_hz(10.23e6);
        // 10.22999999543 MHz expected (approximately)
        let expected_approx = 10_229_999.99543;
        let diff = (f - expected_approx).abs();
        assert!(
            diff < 0.01,
            "Factory offset should be near 10.22999999543 MHz, got {} (diff={})",
            f,
            diff
        );
    }

    // --- Clock correction at time ---

    #[test]
    fn test_clock_correction_at_one_day() {
        let model = SatelliteClockModel::new(20_200_000.0, 0.0);
        let correction = model.clock_correction_at_time(SECONDS_PER_DAY);
        let us = correction * 1e6;
        // Should be ~38.6 us
        assert!(
            us > 37.0 && us < 40.0,
            "One-day clock correction should be ~38.6 us, got {} us",
            us
        );
    }

    #[test]
    fn test_clock_correction_scales_linearly() {
        let model = SatelliteClockModel::new(20_200_000.0, 0.0);
        let corr_1h = model.clock_correction_at_time(3600.0);
        let corr_2h = model.clock_correction_at_time(7200.0);
        assert!(
            (corr_2h - 2.0 * corr_1h).abs() < 1e-20,
            "Clock correction should scale linearly"
        );
    }

    // --- Periodic correction (eccentric orbits) ---

    #[test]
    fn test_periodic_correction_zero_eccentricity() {
        let model = SatelliteClockModel::new(20_200_000.0, 0.0);
        let corr = model.periodic_correction(0.0, 1.0);
        assert!(corr.abs() < 1e-20, "Zero eccentricity should give zero periodic correction");
    }

    #[test]
    fn test_periodic_correction_nonzero() {
        let model = SatelliteClockModel::new(20_200_000.0, 0.02);
        let corr = model.periodic_correction(0.02, PI / 2.0);
        // For GPS with e=0.02, peak periodic correction is a few tens of nanoseconds
        assert!(corr.abs() > 0.0, "Non-zero eccentricity should give periodic correction");
        // Should be on the order of nanoseconds to microseconds
        let ns = corr.abs() * 1e9;
        assert!(
            ns > 0.1 && ns < 1000.0,
            "Periodic correction should be nanoseconds scale, got {} ns",
            ns
        );
    }

    #[test]
    fn test_eccentric_anomaly_correction_symmetry() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let corr_pos = comp.eccentric_anomaly_correction(0.02, PI / 4.0);
        let corr_neg = comp.eccentric_anomaly_correction(0.02, -PI / 4.0);
        assert!(
            (corr_pos + corr_neg).abs() < 1e-20,
            "Eccentric anomaly correction should be antisymmetric"
        );
    }

    #[test]
    fn test_eccentric_anomaly_correction_at_zero() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let corr = comp.eccentric_anomaly_correction(0.02, 0.0);
        assert!(corr.abs() < 1e-20, "At E=0, sin(E)=0, correction should be zero");
    }

    // --- IQ sample compensation ---

    #[test]
    fn test_iq_compensation_identity_at_zero() {
        let config = GravitationalRedshiftConfig {
            include_special_relativity: false,
            include_general_relativity: false,
            ..Default::default()
        };
        let comp = GravitationalRedshiftCompensator::new(config);
        let mut samples = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let orig = samples.clone();
        comp.compensate_iq_samples(&mut samples, 1.0 / 1e6);
        for (s, o) in samples.iter().zip(orig.iter()) {
            assert!((s.0 - o.0).abs() < 1e-10);
            assert!((s.1 - o.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_iq_compensation_preserves_magnitude() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let mut samples: Vec<(f64, f64)> = vec![(1.0, 0.0), (0.5, 0.5), (0.0, 1.0)];
        let orig_mags: Vec<f64> = samples
            .iter()
            .map(|s| {
                let mag_sq: f64 = s.0 * s.0 + s.1 * s.1;
                mag_sq.sqrt()
            })
            .collect();
        comp.compensate_iq_samples(&mut samples, 1.0 / 1e6);
        for (s, m) in samples.iter().zip(orig_mags.iter()) {
            let mag = (s.0 * s.0 + s.1 * s.1).sqrt();
            assert!(
                (mag - m).abs() < 1e-10,
                "Phase rotation should preserve magnitude"
            );
        }
    }

    #[test]
    fn test_iq_compensation_applies_phase() {
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let mut samples = vec![(1.0, 0.0); 100];
        comp.compensate_iq_samples(&mut samples, 1.0 / 1e6);
        // Last sample should have accumulated some phase
        let last = samples[99];
        // Phase should not be exactly zero for the last sample
        assert!(
            last.1.abs() > 1e-15,
            "Compensation should rotate samples"
        );
    }

    // --- Schwarzschild radius ---

    #[test]
    fn test_schwarzschild_radius_earth() {
        // Earth mass = 5.972e24 kg, Schwarzschild radius ~ 8.87 mm
        let rs = schwarzschild_radius(5.972e24);
        assert!(
            (rs - 0.00887).abs() < 0.0001,
            "Earth Schwarzschild radius should be ~8.87 mm, got {} m",
            rs
        );
    }

    #[test]
    fn test_schwarzschild_radius_sun() {
        // Sun mass = 1.989e30 kg, Schwarzschild radius ~ 2953 m
        let rs = schwarzschild_radius(1.989e30);
        assert!(
            (rs - 2953.0).abs() < 5.0,
            "Sun Schwarzschild radius should be ~2953 m, got {} m",
            rs
        );
    }

    #[test]
    fn test_schwarzschild_radius_zero_mass() {
        let rs = schwarzschild_radius(0.0);
        assert!(rs.abs() < TOLERANCE, "Zero mass should give zero radius");
    }

    // --- Gravitational frequency shift helper ---

    #[test]
    fn test_gravitational_frequency_shift_zero_potential() {
        let f = gravitational_frequency_shift(1e9, 0.0);
        assert!((f - 1e9).abs() < 1e-3, "Zero potential diff should give same frequency");
    }

    #[test]
    fn test_gravitational_frequency_shift_positive() {
        // Positive potential difference means receiver is at higher potential (blueshift)
        let f = gravitational_frequency_shift(1e9, 1e6);
        assert!(f > 1e9, "Positive potential difference should blueshift");
    }

    // --- Shapiro delay ---

    #[test]
    fn test_shapiro_delay_positive() {
        // Earth surface to GPS satellite
        let r1 = EARTH_RADIUS_M; // ground
        let r2 = EARTH_RADIUS_M + 20_200_000.0; // GPS
        let closest = EARTH_RADIUS_M; // signal just grazes Earth
        let delay = shapiro_delay(r1, r2, closest);
        assert!(delay > 0.0, "Shapiro delay should be positive");
    }

    #[test]
    fn test_shapiro_delay_order_of_magnitude() {
        // For GPS signals, Shapiro delay is on the order of tens of nanoseconds
        let r1 = EARTH_RADIUS_M;
        let r2 = EARTH_RADIUS_M + 20_200_000.0;
        let closest = EARTH_RADIUS_M;
        let delay = shapiro_delay(r1, r2, closest);
        let ns = delay * 1e9;
        assert!(
            ns > 0.01 && ns < 1000.0,
            "Shapiro delay should be nanoseconds-scale, got {} ns",
            ns
        );
    }

    #[test]
    fn test_shapiro_delay_increases_with_closer_approach() {
        let r1 = EARTH_RADIUS_M + 20_200_000.0;
        let r2 = EARTH_RADIUS_M + 20_200_000.0;
        let delay_far = shapiro_delay(r1, r2, r1 * 0.5);
        let delay_close = shapiro_delay(r1, r2, r1 * 0.1);
        assert!(
            delay_close > delay_far,
            "Closer approach should give larger Shapiro delay"
        );
    }

    // --- Presets ---

    #[test]
    fn test_gps_preset_values() {
        let p = gps_preset();
        assert!((p.orbital_altitude_m - 20_200_000.0).abs() < 1.0);
        assert!((p.carrier_frequency_hz - 1_575.42e6).abs() < 1.0);
        assert!((p.orbital_eccentricity - 0.02).abs() < 1e-6);
    }

    #[test]
    fn test_galileo_preset_values() {
        let p = galileo_preset();
        assert!((p.orbital_altitude_m - 23_222_000.0).abs() < 1.0);
        assert!((p.carrier_frequency_hz - 1_575.42e6).abs() < 1.0);
    }

    #[test]
    fn test_glonass_preset_values() {
        let p = glonass_preset();
        assert!((p.orbital_altitude_m - 19_100_000.0).abs() < 1.0);
        assert!((p.carrier_frequency_hz - 1_602.0e6).abs() < 1.0);
    }

    #[test]
    fn test_galileo_drift_larger_than_gps() {
        // Galileo is at higher altitude, so GR effect is slightly larger
        let gps = GravitationalRedshiftCompensator::new(gps_preset());
        let gal = GravitationalRedshiftCompensator::new(galileo_preset());
        let drift_gps = gps.daily_drift_seconds(gps_preset().orbital_altitude_m);
        let drift_gal = gal.daily_drift_seconds(galileo_preset().orbital_altitude_m);
        assert!(
            drift_gal > drift_gps,
            "Galileo at higher altitude should have larger net drift"
        );
    }

    // --- Edge cases and additional coverage ---

    #[test]
    fn test_very_high_altitude() {
        // At very high altitude, GR dominates and SR becomes negligible
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let alt = 1_000_000_000.0; // 1 million km
        let rate = comp.combined_clock_rate(alt);
        assert!(rate > 0.0, "At extreme altitude, GR should dominate");
    }

    #[test]
    fn test_geo_altitude() {
        // GEO at ~35,786 km
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let drift_us = comp.daily_drift_seconds(35_786_000.0) * 1e6;
        // GEO has even larger net drift than GPS (~45.5 us/day)
        assert!(
            drift_us > 40.0 && drift_us < 55.0,
            "GEO drift should be ~45-50 us/day, got {} us/day",
            drift_us
        );
    }

    #[test]
    fn test_leo_drift_smaller_than_gps() {
        // At low altitude, SR and GR are more balanced
        let comp = GravitationalRedshiftCompensator::new(Default::default());
        let drift_leo = comp.daily_drift_seconds(400_000.0).abs();
        let drift_gps = comp.daily_drift_seconds(20_200_000.0).abs();
        assert!(
            drift_leo < drift_gps,
            "LEO drift magnitude should be smaller than GPS"
        );
    }

    #[test]
    fn test_satellite_clock_model_construction() {
        let model = SatelliteClockModel::new(20_200_000.0, 0.02);
        assert!((model.altitude_m - 20_200_000.0).abs() < TOLERANCE);
        assert!((model.eccentricity - 0.02).abs() < TOLERANCE);
    }
}
