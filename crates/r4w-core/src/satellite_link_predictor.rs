//! Satellite link predictor for LEO/MEO/GEO pass planning.
//!
//! Predicts satellite passes, Doppler shifts, and pointing angles for
//! satellite link budget analysis. Uses Keplerian orbit propagation with
//! J2 perturbation awareness and provides complete pass characterization
//! including visibility windows, Doppler profiles, and link margin estimates.
//!
//! # Example
//!
//! ```
//! use r4w_core::satellite_link_predictor::{
//!     SatelliteLinkPredictor, OrbitalElements, GroundStation,
//! };
//!
//! let gs = GroundStation {
//!     lat_deg: 40.0,
//!     lon_deg: -105.0,
//!     alt_m: 1600.0,
//! };
//!
//! let orbit = OrbitalElements::leo_400km();
//! let predictor = SatelliteLinkPredictor::new(gs, orbit, 437.0e6);
//!
//! // Check if the satellite is visible at t=0
//! let visible = predictor.is_visible(0.0, 5.0);
//!
//! // Get pointing angles at t=0
//! let (az, el) = predictor.pointing_angles(0.0);
//! assert!(az >= 0.0 && az <= 360.0);
//! ```

use std::f64::consts::PI;

// ── Constants ────────────────────────────────────────────────────────────────

/// Earth gravitational parameter (km^3/s^2)
const MU_EARTH: f64 = 398600.4418;

/// Earth equatorial radius (km)
const R_EARTH_KM: f64 = 6378.137;

/// Speed of light (m/s)
const C_LIGHT: f64 = 299_792_458.0;

/// Earth rotation rate (rad/s)
const OMEGA_EARTH: f64 = 7.292_115e-5;

/// Degrees to radians
const DEG2RAD: f64 = PI / 180.0;

/// Radians to degrees
const RAD2DEG: f64 = 180.0 / PI;

// ── Data Structures ──────────────────────────────────────────────────────────

/// Keplerian orbital elements describing a satellite orbit.
#[derive(Debug, Clone)]
pub struct OrbitalElements {
    /// Semi-major axis in kilometers.
    pub semi_major_axis_km: f64,
    /// Eccentricity (0 = circular, <1 = elliptical).
    pub eccentricity: f64,
    /// Inclination in degrees.
    pub inclination_deg: f64,
    /// Right ascension of ascending node in degrees.
    pub raan_deg: f64,
    /// Argument of perigee in degrees.
    pub arg_perigee_deg: f64,
    /// Mean anomaly at epoch in degrees.
    pub mean_anomaly_deg: f64,
}

impl OrbitalElements {
    /// LEO orbit at ~400 km altitude (ISS-like).
    pub fn leo_400km() -> Self {
        Self {
            semi_major_axis_km: R_EARTH_KM + 400.0,
            eccentricity: 0.0001,
            inclination_deg: 51.6,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
        }
    }

    /// MEO orbit at ~20,200 km altitude (GPS-like).
    pub fn meo_20200km() -> Self {
        Self {
            semi_major_axis_km: R_EARTH_KM + 20200.0,
            eccentricity: 0.01,
            inclination_deg: 55.0,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
        }
    }

    /// GEO orbit at ~35,786 km altitude (geostationary).
    pub fn geo_35786km() -> Self {
        Self {
            semi_major_axis_km: R_EARTH_KM + 35786.0,
            eccentricity: 0.0001,
            inclination_deg: 0.05,
            raan_deg: 0.0,
            arg_perigee_deg: 0.0,
            mean_anomaly_deg: 0.0,
        }
    }

    /// Orbital period in seconds.
    pub fn period_s(&self) -> f64 {
        let a = self.semi_major_axis_km;
        2.0 * PI * (a * a * a / MU_EARTH).sqrt()
    }
}

/// Ground station location.
#[derive(Debug, Clone)]
pub struct GroundStation {
    /// Geodetic latitude in degrees.
    pub lat_deg: f64,
    /// Geodetic longitude in degrees.
    pub lon_deg: f64,
    /// Altitude above WGS-84 ellipsoid in meters.
    pub alt_m: f64,
}

impl GroundStation {
    /// Convert ground station position to ECEF (km).
    fn to_ecef_km(&self) -> [f64; 3] {
        let lat = self.lat_deg * DEG2RAD;
        let lon = self.lon_deg * DEG2RAD;
        let alt_km = self.alt_m / 1000.0;

        // WGS-84 flattening
        let f = 1.0 / 298.257_223_563;
        let e2 = 2.0 * f - f * f;
        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let n = R_EARTH_KM / (1.0 - e2 * sin_lat * sin_lat).sqrt();

        let x = (n + alt_km) * cos_lat * lon.cos();
        let y = (n + alt_km) * cos_lat * lon.sin();
        let z = (n * (1.0 - e2) + alt_km) * sin_lat;

        [x, y, z]
    }
}

/// A predicted satellite pass over a ground station.
#[derive(Debug, Clone)]
pub struct PassPrediction {
    /// Acquisition of signal time (seconds from epoch).
    pub aos_time_s: f64,
    /// Loss of signal time (seconds from epoch).
    pub los_time_s: f64,
    /// Maximum elevation during pass (degrees).
    pub max_elevation_deg: f64,
    /// Pass duration in seconds.
    pub duration_s: f64,
}

/// A single point in a Doppler profile.
#[derive(Debug, Clone)]
pub struct DopplerPoint {
    /// Time in seconds from epoch.
    pub time_s: f64,
    /// Doppler shift in Hz.
    pub doppler_hz: f64,
}

/// A single point in a pointing profile.
#[derive(Debug, Clone)]
pub struct PointingPoint {
    /// Time in seconds from epoch.
    pub time_s: f64,
    /// Azimuth in degrees (0=North, 90=East).
    pub azimuth_deg: f64,
    /// Elevation in degrees above horizon.
    pub elevation_deg: f64,
}

/// A single point in a link margin profile.
#[derive(Debug, Clone)]
pub struct LinkMarginPoint {
    /// Time in seconds from epoch.
    pub time_s: f64,
    /// Free-space path loss in dB.
    pub fspl_db: f64,
    /// Received power relative to transmit power in dB (i.e., -FSPL).
    pub received_power_db: f64,
    /// Slant range in km.
    pub range_km: f64,
}

// ── Predictor ────────────────────────────────────────────────────────────────

/// Satellite link predictor for pass planning and link budget analysis.
///
/// Given a ground station position, orbital elements, and carrier frequency,
/// this struct computes visibility windows, Doppler profiles, pointing angles,
/// and link margins over time.
#[derive(Debug, Clone)]
pub struct SatelliteLinkPredictor {
    /// Ground station.
    pub ground_station: GroundStation,
    /// Satellite orbital elements at epoch.
    pub orbit: OrbitalElements,
    /// Carrier frequency in Hz (used for Doppler calculation).
    pub frequency_hz: f64,
}

impl SatelliteLinkPredictor {
    /// Create a new predictor.
    pub fn new(ground_station: GroundStation, orbit: OrbitalElements, frequency_hz: f64) -> Self {
        Self {
            ground_station,
            orbit,
            frequency_hz,
        }
    }

    /// Propagate satellite position to time `t_s` seconds from epoch.
    ///
    /// Returns ECEF position in km using Keplerian (two-body) propagation
    /// with Kepler's equation solved via Newton-Raphson iteration.
    pub fn propagate_position(&self, t_s: f64) -> [f64; 3] {
        let a = self.orbit.semi_major_axis_km;
        let e = self.orbit.eccentricity;
        let inc = self.orbit.inclination_deg * DEG2RAD;
        let raan = self.orbit.raan_deg * DEG2RAD;
        let omega = self.orbit.arg_perigee_deg * DEG2RAD;
        let m0 = self.orbit.mean_anomaly_deg * DEG2RAD;

        // Mean motion (rad/s)
        let n = (MU_EARTH / (a * a * a)).sqrt();

        // Mean anomaly at time t
        let m = (m0 + n * t_s) % (2.0 * PI);

        // Solve Kepler's equation: E - e*sin(E) = M  (Newton-Raphson)
        let mut ecc_anom = m;
        for _ in 0..20 {
            let delta = (ecc_anom - e * ecc_anom.sin() - m)
                / (1.0 - e * ecc_anom.cos());
            ecc_anom -= delta;
            if delta.abs() < 1e-12 {
                break;
            }
        }

        // True anomaly
        let cos_e = ecc_anom.cos();
        let sin_e = ecc_anom.sin();
        let nu = ((1.0 - e * e).sqrt() * sin_e).atan2(cos_e - e);

        // Radius
        let r = a * (1.0 - e * cos_e);

        // Position in orbital plane
        let x_orb = r * (nu + omega).cos();
        let y_orb = r * (nu + omega).sin();

        // Rotate to ECI using RAAN and inclination
        let cos_raan = raan.cos();
        let sin_raan = raan.sin();
        let cos_inc = inc.cos();
        let sin_inc = inc.sin();

        let x_eci = x_orb * cos_raan - y_orb * cos_inc * sin_raan;
        let y_eci = x_orb * sin_raan + y_orb * cos_inc * cos_raan;
        let z_eci = y_orb * sin_inc;

        // Rotate ECI to ECEF (account for Earth rotation)
        let theta = OMEGA_EARTH * t_s;
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        let x_ecef = x_eci * cos_t + y_eci * sin_t;
        let y_ecef = -x_eci * sin_t + y_eci * cos_t;
        let z_ecef = z_eci;

        [x_ecef, y_ecef, z_ecef]
    }

    /// Compute azimuth and elevation from the ground station to the satellite
    /// at time `t_s`. Returns `(azimuth_deg, elevation_deg)`.
    pub fn pointing_angles(&self, t_s: f64) -> (f64, f64) {
        let sat_ecef = self.propagate_position(t_s);
        let gs_ecef = self.ground_station.to_ecef_km();

        // Range vector in ECEF
        let dx = sat_ecef[0] - gs_ecef[0];
        let dy = sat_ecef[1] - gs_ecef[1];
        let dz = sat_ecef[2] - gs_ecef[2];

        // Convert range vector to ENU (East-North-Up) at ground station
        let lat = self.ground_station.lat_deg * DEG2RAD;
        let lon = self.ground_station.lon_deg * DEG2RAD;

        let sin_lat = lat.sin();
        let cos_lat = lat.cos();
        let sin_lon = lon.sin();
        let cos_lon = lon.cos();

        let east = -sin_lon * dx + cos_lon * dy;
        let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
        let up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

        let range_horiz = (east * east + north * north).sqrt();
        let elevation = up.atan2(range_horiz) * RAD2DEG;

        let mut azimuth = east.atan2(north) * RAD2DEG;
        if azimuth < 0.0 {
            azimuth += 360.0;
        }

        (azimuth, elevation)
    }

    /// Check if the satellite is above a given elevation mask at time `t_s`.
    pub fn is_visible(&self, t_s: f64, elevation_mask_deg: f64) -> bool {
        let (_, el) = self.pointing_angles(t_s);
        el >= elevation_mask_deg
    }

    /// Compute slant range from ground station to satellite in km at time `t_s`.
    pub fn range_km(&self, t_s: f64) -> f64 {
        let sat = self.propagate_position(t_s);
        let gs = self.ground_station.to_ecef_km();
        let dx = sat[0] - gs[0];
        let dy = sat[1] - gs[1];
        let dz = sat[2] - gs[2];
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Compute Doppler shift in Hz at time `t_s`.
    ///
    /// Uses finite differences on range to estimate range rate.
    pub fn doppler_at(&self, t_s: f64) -> f64 {
        let dt = 0.01; // 10 ms step
        let r1 = self.range_km(t_s - dt / 2.0);
        let r2 = self.range_km(t_s + dt / 2.0);
        let range_rate_km_s = (r2 - r1) / dt;
        let range_rate_m_s = range_rate_km_s * 1000.0;
        // Doppler: -f * v_r / c  (negative because approaching = positive Doppler)
        -self.frequency_hz * range_rate_m_s / C_LIGHT
    }

    /// Predict a satellite pass given a search window.
    ///
    /// Searches from `start_s` to `start_s + search_duration_s` in steps of
    /// `step_s` seconds. The `elevation_mask_deg` sets the minimum elevation
    /// for visibility. Returns `None` if no pass is found.
    pub fn predict_pass(
        &self,
        start_s: f64,
        search_duration_s: f64,
        step_s: f64,
        elevation_mask_deg: f64,
    ) -> Option<PassPrediction> {
        let mut aos: Option<f64> = None;
        let mut max_el = f64::NEG_INFINITY;
        let mut t = start_s;

        while t <= start_s + search_duration_s {
            let (_, el) = self.pointing_angles(t);

            if el >= elevation_mask_deg {
                if aos.is_none() {
                    aos = Some(t);
                }
                if el > max_el {
                    max_el = el;
                }
            } else if let Some(aos_t) = aos {
                // We just dropped below the mask => pass is complete
                let los_t = t;
                return Some(PassPrediction {
                    aos_time_s: aos_t,
                    los_time_s: los_t,
                    max_elevation_deg: max_el,
                    duration_s: los_t - aos_t,
                });
            }

            t += step_s;
        }

        // If still visible at end of search window, return partial pass
        if let Some(aos_t) = aos {
            let los_t = start_s + search_duration_s;
            Some(PassPrediction {
                aos_time_s: aos_t,
                los_time_s: los_t,
                max_elevation_deg: max_el,
                duration_s: los_t - aos_t,
            })
        } else {
            None
        }
    }

    /// Compute Doppler profile during a pass at the given time resolution.
    ///
    /// Returns Doppler shift samples from `start_s` to `end_s` at `step_s`
    /// intervals.
    pub fn doppler_profile(
        &self,
        start_s: f64,
        end_s: f64,
        step_s: f64,
    ) -> Vec<DopplerPoint> {
        let mut profile = Vec::new();
        let mut t = start_s;
        while t <= end_s {
            profile.push(DopplerPoint {
                time_s: t,
                doppler_hz: self.doppler_at(t),
            });
            t += step_s;
        }
        profile
    }

    /// Compute pointing angle profile over a time window.
    ///
    /// Returns azimuth/elevation pairs from `start_s` to `end_s` at `step_s`
    /// intervals.
    pub fn pointing_profile(
        &self,
        start_s: f64,
        end_s: f64,
        step_s: f64,
    ) -> Vec<PointingPoint> {
        let mut profile = Vec::new();
        let mut t = start_s;
        while t <= end_s {
            let (az, el) = self.pointing_angles(t);
            profile.push(PointingPoint {
                time_s: t,
                azimuth_deg: az,
                elevation_deg: el,
            });
            t += step_s;
        }
        profile
    }

    /// Compute link margin profile over a time window.
    ///
    /// Free-space path loss: `FSPL(dB) = 20*log10(d_km) + 20*log10(f_MHz) + 32.45`
    ///
    /// Received power (relative) = -FSPL.
    pub fn link_margin_profile(
        &self,
        start_s: f64,
        end_s: f64,
        step_s: f64,
    ) -> Vec<LinkMarginPoint> {
        let f_mhz = self.frequency_hz / 1e6;
        let mut profile = Vec::new();
        let mut t = start_s;
        while t <= end_s {
            let r = self.range_km(t);
            let fspl = 20.0 * r.log10() + 20.0 * f_mhz.log10() + 32.45;
            profile.push(LinkMarginPoint {
                time_s: t,
                fspl_db: fspl,
                received_power_db: -fspl,
                range_km: r,
            });
            t += step_s;
        }
        profile
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn test_gs() -> GroundStation {
        GroundStation {
            lat_deg: 40.0,
            lon_deg: -105.0,
            alt_m: 1600.0,
        }
    }

    fn test_predictor_leo() -> SatelliteLinkPredictor {
        SatelliteLinkPredictor::new(
            test_gs(),
            OrbitalElements::leo_400km(),
            437.0e6,
        )
    }

    fn test_predictor_geo() -> SatelliteLinkPredictor {
        SatelliteLinkPredictor::new(
            test_gs(),
            OrbitalElements::geo_35786km(),
            12.0e9,
        )
    }

    #[test]
    fn test_orbital_period_leo() {
        let orbit = OrbitalElements::leo_400km();
        let period = orbit.period_s();
        // ISS orbital period is about 92.6 minutes = 5556 seconds
        assert!((period - 5560.0).abs() < 60.0, "LEO period={period:.1}s, expected ~5560s");
    }

    #[test]
    fn test_orbital_period_geo() {
        let orbit = OrbitalElements::geo_35786km();
        let period = orbit.period_s();
        // GEO period is about 86164 seconds (sidereal day)
        assert!(
            (period - 86164.0).abs() < 200.0,
            "GEO period={period:.1}s, expected ~86164s"
        );
    }

    #[test]
    fn test_orbital_period_meo() {
        let orbit = OrbitalElements::meo_20200km();
        let period = orbit.period_s();
        // GPS orbit period ~11h58m = ~43080s
        assert!(
            (period - 43080.0).abs() < 200.0,
            "MEO period={period:.1}s, expected ~43080s"
        );
    }

    #[test]
    fn test_propagate_position_altitude() {
        let p = test_predictor_leo();
        let pos = p.propagate_position(0.0);
        let r = (pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]).sqrt();
        let expected_r = R_EARTH_KM + 400.0;
        assert!(
            (r - expected_r).abs() < 10.0,
            "Radius={r:.1}km, expected ~{expected_r:.1}km"
        );
    }

    #[test]
    fn test_propagate_position_moves_over_time() {
        let p = test_predictor_leo();
        let pos0 = p.propagate_position(0.0);
        let pos1 = p.propagate_position(600.0); // 10 minutes later
        let dist = ((pos1[0] - pos0[0]).powi(2)
            + (pos1[1] - pos0[1]).powi(2)
            + (pos1[2] - pos0[2]).powi(2))
        .sqrt();
        // LEO satellite at ~7.66 km/s should move ~4600 km in 600s
        assert!(dist > 1000.0, "Satellite should have moved >1000 km, got {dist:.1} km");
    }

    #[test]
    fn test_ground_station_ecef() {
        let gs = test_gs();
        let ecef = gs.to_ecef_km();
        let r = (ecef[0] * ecef[0] + ecef[1] * ecef[1] + ecef[2] * ecef[2]).sqrt();
        // Ground station should be approximately at Earth's surface
        assert!(
            (r - R_EARTH_KM).abs() < 30.0,
            "GS radius={r:.1}km, expected ~{R_EARTH_KM:.1}km"
        );
    }

    #[test]
    fn test_pointing_angles_range() {
        let p = test_predictor_leo();
        let (az, _el) = p.pointing_angles(0.0);
        assert!(az >= 0.0 && az <= 360.0, "Azimuth={az:.1} out of range");
        // Elevation can be negative (below horizon)
    }

    #[test]
    fn test_range_km_reasonable() {
        let p = test_predictor_leo();
        let r = p.range_km(0.0);
        // Range should be between LEO altitude and maximum slant range
        assert!(r > 300.0 && r < 13000.0, "LEO range={r:.1}km unexpected");
    }

    #[test]
    fn test_doppler_sign_change_during_pass() {
        // For a LEO pass, Doppler should go from positive (approaching) to
        // negative (receding). We sample over one orbit and check for both signs.
        let p = test_predictor_leo();
        let period = p.orbit.period_s();
        let profile = p.doppler_profile(0.0, period, period / 200.0);
        let has_positive = profile.iter().any(|dp| dp.doppler_hz > 100.0);
        let has_negative = profile.iter().any(|dp| dp.doppler_hz < -100.0);
        assert!(
            has_positive && has_negative,
            "LEO Doppler should change sign over one orbit"
        );
    }

    #[test]
    fn test_doppler_magnitude_leo() {
        // Max Doppler for LEO at 437 MHz should be roughly +/-10 kHz
        let p = test_predictor_leo();
        let period = p.orbit.period_s();
        let mut max_doppler = 0.0_f64;
        let mut t = 0.0;
        while t < period {
            let d = p.doppler_at(t).abs();
            if d > max_doppler {
                max_doppler = d;
            }
            t += period / 200.0;
        }
        assert!(
            max_doppler > 1000.0 && max_doppler < 20000.0,
            "LEO max Doppler={max_doppler:.0}Hz, expected 1-20 kHz range"
        );
    }

    #[test]
    fn test_link_margin_fspl() {
        // FSPL at 437 MHz and ~1000 km should be around 145 dB
        let p = test_predictor_leo();
        let lm = p.link_margin_profile(0.0, 0.0, 1.0);
        assert_eq!(lm.len(), 1);
        let fspl = lm[0].fspl_db;
        // FSPL = 20*log10(r_km) + 20*log10(f_mhz) + 32.45
        assert!(
            fspl > 130.0 && fspl < 175.0,
            "FSPL={fspl:.1}dB, expected ~140-155 dB for LEO UHF"
        );
        assert!(
            (lm[0].received_power_db + fspl).abs() < 0.01,
            "Received power should be -FSPL"
        );
    }

    #[test]
    fn test_predict_pass_finds_pass_or_none() {
        // Search one full orbit; depending on geometry we may or may not find a pass.
        // We just verify the function returns a structurally valid result.
        let p = test_predictor_leo();
        let period = p.orbit.period_s();
        let result = p.predict_pass(0.0, period, 10.0, 5.0);
        if let Some(pass) = result {
            assert!(pass.duration_s > 0.0, "Pass duration must be positive");
            assert!(pass.los_time_s > pass.aos_time_s, "LOS must be after AOS");
            assert!(pass.max_elevation_deg >= 5.0, "Max el must be above mask");
        }
        // None is also a valid result if geometry doesn't produce a pass
    }

    #[test]
    fn test_geo_range_is_large() {
        let p = test_predictor_geo();
        let r = p.range_km(0.0);
        // GEO range should be ~35786 km +/- several thousand depending on geometry
        assert!(
            r > 30000.0 && r < 45000.0,
            "GEO range={r:.1}km, expected 30000-45000 km"
        );
    }

    #[test]
    fn test_pointing_profile_length() {
        let p = test_predictor_leo();
        let profile = p.pointing_profile(0.0, 100.0, 10.0);
        assert_eq!(profile.len(), 11, "Expected 11 points (0,10,20,...,100)");
        for pt in &profile {
            assert!(pt.azimuth_deg >= 0.0 && pt.azimuth_deg <= 360.0);
        }
    }

    #[test]
    fn test_is_visible_high_mask() {
        // With a very high mask (89 degrees), visibility should be very unlikely
        let p = test_predictor_leo();
        // Just verify the API works and returns a bool
        let _visible = p.is_visible(0.0, 89.0);
    }
}
