//! Saastamoinen tropospheric delay model
//!
//! Computes zenith tropospheric delay with hydrostatic (dry) and wet components,
//! mapped to arbitrary elevation angles via a simple mapping function.

use serde::{Deserialize, Serialize};

/// Speed of light in m/s
const C: f64 = 299_792_458.0;

/// Saastamoinen tropospheric delay model
// trace:FR-040 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SaastamoinenModel {
    /// Station height above sea level in meters
    pub height_m: f64,
    /// Temperature in Kelvin
    pub temperature_k: f64,
    /// Atmospheric pressure in hPa (mbar)
    pub pressure_hpa: f64,
    /// Relative humidity (0.0 to 1.0)
    pub relative_humidity: f64,
}

impl SaastamoinenModel {
    /// Standard atmosphere at sea level (ISA conditions)
    pub fn standard_atmosphere() -> Self {
        Self {
            height_m: 0.0,
            temperature_k: 288.15, // 15°C
            pressure_hpa: 1013.25,
            relative_humidity: 0.5,
        }
    }

    /// Create model for a given altitude using standard atmosphere lapse rate
    pub fn at_altitude(height_m: f64) -> Self {
        let lapse_rate = 0.0065; // K/m
        let t = 288.15 - lapse_rate * height_m;
        let p = 1013.25 * (1.0 - lapse_rate * height_m / 288.15).powf(5.2561);
        Self {
            height_m,
            temperature_k: t,
            pressure_hpa: p,
            relative_humidity: 0.5,
        }
    }

    /// Compute zenith hydrostatic (dry) delay in meters
    pub fn zenith_hydrostatic_delay_m(&self) -> f64 {
        // Saastamoinen hydrostatic delay
        0.002_277 * self.pressure_hpa / (1.0 - 0.00266 * (2.0 * self.height_m.to_radians()).cos() - 0.00028 * self.height_m / 1000.0)
    }

    /// Water vapor partial pressure in hPa
    fn water_vapor_pressure(&self) -> f64 {
        // Buck equation for saturation vapor pressure
        let t_c = self.temperature_k - 273.15;
        let es = 6.1121 * ((18.678 - t_c / 234.5) * t_c / (257.14 + t_c)).exp();
        self.relative_humidity * es
    }

    /// Compute zenith wet delay in meters
    pub fn zenith_wet_delay_m(&self) -> f64 {
        let e = self.water_vapor_pressure();
        0.002_277 * (1255.0 / self.temperature_k + 0.05) * e
    }

    /// Total zenith tropospheric delay in meters
    pub fn zenith_total_delay_m(&self) -> f64 {
        self.zenith_hydrostatic_delay_m() + self.zenith_wet_delay_m()
    }

    /// Compute slant tropospheric delay in seconds at given elevation
    ///
    /// Uses a simple 1/sin(elevation) mapping function with a correction
    /// for low elevation angles.
    pub fn delay_seconds(&self, elevation_rad: f64) -> f64 {
        let zenith_m = self.zenith_total_delay_m();
        let mapping = self.mapping_function(elevation_rad);
        zenith_m * mapping / C
    }

    /// Compute slant delay in meters
    pub fn delay_meters(&self, elevation_rad: f64) -> f64 {
        self.zenith_total_delay_m() * self.mapping_function(elevation_rad)
    }

    /// Simple mapping function: 1/sin(el) with Niell-like correction for low elevations
    fn mapping_function(&self, elevation_rad: f64) -> f64 {
        let el = elevation_rad.max(0.05); // Clamp at ~3 degrees
        let sin_el = el.sin();
        // Chao mapping function approximation
        1.0 / (sin_el + 0.00143 / (0.0455 + sin_el).tan())
    }

    /// Convert delay to carrier phase in radians
    pub fn delay_to_phase_rad(delay_s: f64, frequency_hz: f64) -> f64 {
        -2.0 * std::f64::consts::PI * frequency_hz * delay_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zenith_delay_standard() {
        let model = SaastamoinenModel::standard_atmosphere();
        let total = model.zenith_total_delay_m();
        // Typical zenith total delay: ~2.3 m
        assert!(
            total > 2.0 && total < 2.8,
            "Zenith total delay: {} m",
            total
        );
    }

    #[test]
    fn test_hydrostatic_dominates() {
        let model = SaastamoinenModel::standard_atmosphere();
        let dry = model.zenith_hydrostatic_delay_m();
        let wet = model.zenith_wet_delay_m();
        assert!(dry > wet, "Hydrostatic ({}) should dominate wet ({})", dry, wet);
    }

    #[test]
    fn test_low_elevation_more_delay() {
        let model = SaastamoinenModel::standard_atmosphere();
        let delay_low = model.delay_seconds(10.0_f64.to_radians());
        let delay_high = model.delay_seconds(80.0_f64.to_radians());
        assert!(
            delay_low > delay_high,
            "Low elevation delay {} > high {}",
            delay_low,
            delay_high
        );
    }

    #[test]
    fn test_altitude_reduces_delay() {
        let low = SaastamoinenModel::at_altitude(0.0);
        let high = SaastamoinenModel::at_altitude(2000.0);
        assert!(
            low.zenith_total_delay_m() > high.zenith_total_delay_m(),
            "Higher altitude should have less delay"
        );
    }
}
