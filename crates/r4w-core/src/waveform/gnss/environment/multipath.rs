//! GNSS multipath environment presets
//!
//! Maps environment types to tapped delay line configurations compatible
//! with the existing `TappedDelayLine` in r4w-sim.

use serde::{Deserialize, Serialize};

/// GNSS multipath environment preset
// trace:FR-040 | ai:claude
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum GnssMultipathPreset {
    /// Clear sky, minimal reflections (e.g., open field, rooftop)
    OpenSky,
    /// Light multipath from nearby buildings (residential area)
    Suburban,
    /// Heavy multipath with deep fading (downtown, tall buildings)
    UrbanCanyon,
    /// Very heavy multipath, no direct LOS likely
    Indoor,
}

impl std::fmt::Display for GnssMultipathPreset {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::OpenSky => write!(f, "Open Sky"),
            Self::Suburban => write!(f, "Suburban"),
            Self::UrbanCanyon => write!(f, "Urban Canyon"),
            Self::Indoor => write!(f, "Indoor"),
        }
    }
}

/// A single multipath tap definition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultipathTap {
    /// Delay in seconds relative to direct path
    pub delay_s: f64,
    /// Power relative to direct path in dB (negative = weaker)
    pub power_db: f64,
    /// Phase offset in radians
    pub phase_rad: f64,
}

impl GnssMultipathPreset {
    /// Get multipath tap configuration for this environment
    ///
    /// Returns (delay_s, power_db, phase_rad) for each tap.
    /// The first tap (index 0) is always the direct path at delay=0, power=0dB.
    pub fn taps(&self) -> Vec<MultipathTap> {
        match self {
            Self::OpenSky => vec![
                MultipathTap { delay_s: 0.0, power_db: 0.0, phase_rad: 0.0 },
            ],
            Self::Suburban => vec![
                MultipathTap { delay_s: 0.0, power_db: 0.0, phase_rad: 0.0 },
                MultipathTap { delay_s: 50e-9, power_db: -6.0, phase_rad: 0.5 },
                MultipathTap { delay_s: 120e-9, power_db: -12.0, phase_rad: 1.2 },
            ],
            Self::UrbanCanyon => vec![
                MultipathTap { delay_s: 0.0, power_db: 0.0, phase_rad: 0.0 },
                MultipathTap { delay_s: 30e-9, power_db: -3.0, phase_rad: 0.8 },
                MultipathTap { delay_s: 80e-9, power_db: -5.0, phase_rad: 2.1 },
                MultipathTap { delay_s: 200e-9, power_db: -8.0, phase_rad: 3.5 },
                MultipathTap { delay_s: 500e-9, power_db: -14.0, phase_rad: 5.0 },
            ],
            Self::Indoor => vec![
                MultipathTap { delay_s: 0.0, power_db: -3.0, phase_rad: 0.0 }, // attenuated direct
                MultipathTap { delay_s: 20e-9, power_db: -2.0, phase_rad: 0.3 },
                MultipathTap { delay_s: 50e-9, power_db: -4.0, phase_rad: 1.0 },
                MultipathTap { delay_s: 100e-9, power_db: -6.0, phase_rad: 2.0 },
                MultipathTap { delay_s: 200e-9, power_db: -10.0, phase_rad: 3.0 },
                MultipathTap { delay_s: 400e-9, power_db: -15.0, phase_rad: 4.5 },
            ],
        }
    }

    /// Get elevation-dependent multipath configuration
    ///
    /// At high elevation angles, multipath is typically reduced.
    /// Returns taps scaled by elevation factor.
    pub fn taps_at_elevation(&self, elevation_deg: f64) -> Vec<MultipathTap> {
        let mut taps = self.taps();
        if taps.len() <= 1 {
            return taps;
        }

        // At high elevation, reduce multipath power
        // At low elevation, reflections are stronger
        let el_factor = if elevation_deg > 60.0 {
            // High elevation: reduce multipath by 3-6 dB
            -3.0 * (elevation_deg - 60.0) / 30.0
        } else if elevation_deg < 20.0 {
            // Low elevation: increase multipath by up to 3 dB
            3.0 * (20.0 - elevation_deg) / 20.0
        } else {
            0.0
        };

        // Apply factor to all taps except the direct path
        for tap in taps.iter_mut().skip(1) {
            tap.power_db += el_factor;
        }
        taps
    }

    /// All preset variants
    pub fn all() -> &'static [GnssMultipathPreset] {
        &[
            Self::OpenSky,
            Self::Suburban,
            Self::UrbanCanyon,
            Self::Indoor,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_open_sky_single_tap() {
        let taps = GnssMultipathPreset::OpenSky.taps();
        assert_eq!(taps.len(), 1);
        assert_eq!(taps[0].delay_s, 0.0);
    }

    #[test]
    fn test_urban_canyon_has_multipath() {
        let taps = GnssMultipathPreset::UrbanCanyon.taps();
        assert!(taps.len() > 3);
        // All reflected paths should be weaker than direct
        for tap in &taps[1..] {
            assert!(tap.power_db < 0.0);
        }
    }

    #[test]
    fn test_elevation_reduces_multipath() {
        let low_el = GnssMultipathPreset::Suburban.taps_at_elevation(10.0);
        let high_el = GnssMultipathPreset::Suburban.taps_at_elevation(80.0);
        // Reflected path power at high elevation should be lower
        assert!(high_el[1].power_db < low_el[1].power_db);
    }
}
