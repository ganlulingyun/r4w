//! Klobuchar ionospheric delay model
//!
//! Implements the broadcast Klobuchar model used in GPS for single-frequency
//! ionospheric delay estimation. The model uses 8 coefficients (4 alpha, 4 beta)
//! broadcast in the GPS navigation message.

use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Speed of light in m/s
const C: f64 = 299_792_458.0;
/// GPS L1 frequency in Hz
const F_L1: f64 = 1_575_420_000.0;

/// Klobuchar ionospheric delay model
// trace:FR-040 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KlobucharModel {
    /// Alpha coefficients (seconds, seconds/semicircle, seconds/semicircle^2, seconds/semicircle^3)
    pub alpha: [f64; 4],
    /// Beta coefficients (seconds, seconds/semicircle, seconds/semicircle^2, seconds/semicircle^3)
    pub beta: [f64; 4],
}

impl KlobucharModel {
    /// Create model with default broadcast coefficients (typical values)
    pub fn default_broadcast() -> Self {
        Self {
            alpha: [0.1118e-7, 0.7451e-8, -0.5961e-7, -0.1192e-6],
            beta: [0.1167e6, -0.4267e5, -0.2621e6, 0.1311e6],
        }
    }

    /// Compute ionospheric delay in seconds
    ///
    /// # Parameters
    /// - `elevation_rad`: Satellite elevation angle in radians
    /// - `azimuth_rad`: Satellite azimuth angle in radians
    /// - `user_lat_rad`: User geodetic latitude in radians
    /// - `user_lon_rad`: User geodetic longitude in radians
    /// - `gps_time_s`: GPS time of week in seconds
    ///
    /// # Returns
    /// Ionospheric group delay in seconds at L1 frequency
    pub fn delay_seconds(
        &self,
        elevation_rad: f64,
        azimuth_rad: f64,
        user_lat_rad: f64,
        user_lon_rad: f64,
        gps_time_s: f64,
    ) -> f64 {
        // Convert to semicircles
        let el_sc = elevation_rad / PI;
        let az_sc = azimuth_rad / PI;
        let lat_sc = user_lat_rad / PI;
        let lon_sc = user_lon_rad / PI;

        // Earth-centered angle
        let psi = 0.0137 / (el_sc + 0.11) - 0.022;

        // Ionospheric pierce point (IPP) latitude
        let mut lat_ipp = lat_sc + psi * az_sc.cos() * PI;
        if lat_ipp > 0.416 {
            lat_ipp = 0.416;
        }
        if lat_ipp < -0.416 {
            lat_ipp = -0.416;
        }

        // IPP longitude
        let lon_ipp = lon_sc + psi * (az_sc * PI).sin() / (lat_ipp * PI).cos();

        // Geomagnetic latitude
        let lat_mag = lat_ipp + 0.064 * (lon_ipp - 1.617).cos();

        // Local time
        let t_local = 43200.0 * lon_ipp + gps_time_s;
        let t_local = t_local % 86400.0;

        // Obliquity factor
        let f_obl = 1.0 + 16.0 * (0.53 - el_sc).powi(3);

        // Amplitude and period of cosine
        let amp = self.alpha[0]
            + self.alpha[1] * lat_mag
            + self.alpha[2] * lat_mag.powi(2)
            + self.alpha[3] * lat_mag.powi(3);
        let amp = amp.max(0.0);

        let per = self.beta[0]
            + self.beta[1] * lat_mag
            + self.beta[2] * lat_mag.powi(2)
            + self.beta[3] * lat_mag.powi(3);
        let per = per.max(72000.0);

        // Ionospheric delay
        let x = 2.0 * PI * (t_local - 50400.0) / per;

        let delay = if x.abs() < 1.57 {
            f_obl * (5.0e-9 + amp * (1.0 - x * x / 2.0 + x.powi(4) / 24.0))
        } else {
            f_obl * 5.0e-9
        };

        delay
    }

    /// Scale delay from L1 to another frequency
    ///
    /// Ionospheric delay scales as 1/f^2
    pub fn scale_to_frequency(&self, delay_l1_s: f64, frequency_hz: f64) -> f64 {
        delay_l1_s * (F_L1 / frequency_hz).powi(2)
    }

    /// Convert delay in seconds to range error in meters
    pub fn delay_to_range_m(delay_s: f64) -> f64 {
        delay_s * C
    }

    /// Convert delay in seconds to carrier phase in radians at given frequency
    pub fn delay_to_phase_rad(delay_s: f64, frequency_hz: f64) -> f64 {
        -2.0 * PI * frequency_hz * delay_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_klobuchar_delay_range() {
        let model = KlobucharModel::default_broadcast();
        let delay = model.delay_seconds(
            45.0_f64.to_radians(),  // 45 deg elevation
            0.0,                     // north azimuth
            40.0_f64.to_radians(),  // ~Philadelphia latitude
            -75.0_f64.to_radians(), // ~Philadelphia longitude
            43200.0,                 // noon GPS time
        );
        // Typical ionospheric delay: 2-15 nanoseconds at L1
        let range_m = KlobucharModel::delay_to_range_m(delay);
        assert!(
            range_m > 0.5 && range_m < 50.0,
            "Iono range error: {} m",
            range_m
        );
    }

    #[test]
    fn test_low_elevation_more_delay() {
        let model = KlobucharModel::default_broadcast();
        let delay_low = model.delay_seconds(
            10.0_f64.to_radians(), 0.0,
            40.0_f64.to_radians(), -75.0_f64.to_radians(), 43200.0,
        );
        let delay_high = model.delay_seconds(
            80.0_f64.to_radians(), 0.0,
            40.0_f64.to_radians(), -75.0_f64.to_radians(), 43200.0,
        );
        assert!(delay_low > delay_high, "Low elevation should have more delay");
    }

    #[test]
    fn test_frequency_scaling() {
        let model = KlobucharModel::default_broadcast();
        let delay_l1 = 10.0e-9; // 10 ns at L1
        let delay_l5 = model.scale_to_frequency(delay_l1, 1_176_450_000.0);
        // L5 delay should be larger (lower frequency)
        assert!(delay_l5 > delay_l1);
        let ratio = delay_l5 / delay_l1;
        let expected_ratio = (F_L1 / 1_176_450_000.0).powi(2);
        assert!((ratio - expected_ratio).abs() < 0.01);
    }
}
