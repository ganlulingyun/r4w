//! Antenna gain patterns for GNSS receivers
//!
//! Models the gain variation of different antenna types as a function of
//! elevation angle, which affects the received signal strength per satellite.

use serde::{Deserialize, Serialize};

/// Antenna gain pattern type
// trace:FR-040 | ai:claude
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AntennaPattern {
    /// Isotropic: uniform gain in all directions (0 dBi)
    Isotropic,
    /// Hemispherical: uniform gain above horizon, zero below
    Hemispherical {
        /// Peak gain in dBi
        peak_gain_dbi: f64,
    },
    /// Microstrip patch: typical GNSS receiver antenna
    Patch {
        /// Peak gain at zenith in dBi (typically 5-7 dBi)
        peak_gain_dbi: f64,
        /// 3dB beamwidth in degrees (typically 140-160 deg)
        beamwidth_deg: f64,
    },
    /// Choke ring: high-precision geodetic antenna with multipath rejection
    ChokeRing {
        /// Peak gain at zenith in dBi (typically 7-9 dBi)
        peak_gain_dbi: f64,
    },
}

impl AntennaPattern {
    /// Compute antenna gain in dBi at the given elevation angle
    pub fn gain_dbi(&self, elevation_deg: f64) -> f64 {
        match self {
            AntennaPattern::Isotropic => 0.0,
            AntennaPattern::Hemispherical { peak_gain_dbi } => {
                if elevation_deg >= 0.0 {
                    *peak_gain_dbi
                } else {
                    -30.0 // effectively zero below horizon
                }
            }
            AntennaPattern::Patch { peak_gain_dbi, beamwidth_deg } => {
                if elevation_deg < -5.0 {
                    return -30.0;
                }
                // cos^n model: gain = peak * cos^n(theta)
                // where theta = 90 - elevation (angle from zenith)
                let theta = (90.0 - elevation_deg).to_radians();
                // n chosen so that gain drops 3dB at beamwidth/2
                let half_bw = (beamwidth_deg / 2.0).to_radians();
                let n = 3.0_f64.log10() / (1.0 / half_bw.cos()).log10();
                let gain_lin = theta.cos().abs().powf(n);
                let gain_db = if gain_lin > 1e-6 {
                    10.0 * gain_lin.log10()
                } else {
                    -30.0
                };
                peak_gain_dbi + gain_db
            }
            AntennaPattern::ChokeRing { peak_gain_dbi } => {
                if elevation_deg < 0.0 {
                    return -40.0; // Very good below-horizon rejection
                }
                // Choke ring: flatter pattern with sharp cutoff below horizon
                let theta = (90.0 - elevation_deg).to_radians();
                let gain_lin = theta.cos().abs().powf(1.5);
                let gain_db = if gain_lin > 1e-6 {
                    10.0 * gain_lin.log10()
                } else {
                    -40.0
                };
                peak_gain_dbi + gain_db
            }
        }
    }

    /// Default GNSS patch antenna
    pub fn default_patch() -> Self {
        AntennaPattern::Patch {
            peak_gain_dbi: 5.0,
            beamwidth_deg: 150.0,
        }
    }
}

/// Body attitude (orientation) for antenna pattern rotation
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct BodyAttitude {
    /// Roll angle in degrees (rotation around forward axis)
    pub roll_deg: f64,
    /// Pitch angle in degrees (rotation around right axis)
    pub pitch_deg: f64,
    /// Yaw angle in degrees (rotation around up axis)
    pub yaw_deg: f64,
}

impl Default for BodyAttitude {
    fn default() -> Self {
        Self {
            roll_deg: 0.0,
            pitch_deg: 0.0,
            yaw_deg: 0.0,
        }
    }
}

impl BodyAttitude {
    /// Apply body rotation to an elevation angle
    ///
    /// Returns the effective elevation seen by the antenna, accounting for tilt.
    /// Simplified: only accounts for pitch effect on elevation.
    pub fn effective_elevation(&self, elevation_deg: f64, azimuth_deg: f64) -> f64 {
        let pitch = self.pitch_deg.to_radians();
        let roll = self.roll_deg.to_radians();
        let az = azimuth_deg.to_radians();

        // Tilt correction: elevation changes with pitch and roll
        let delta = pitch * az.cos() + roll * az.sin();
        elevation_deg + delta.to_degrees()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isotropic_uniform() {
        let ant = AntennaPattern::Isotropic;
        assert_eq!(ant.gain_dbi(0.0), 0.0);
        assert_eq!(ant.gain_dbi(90.0), 0.0);
        assert_eq!(ant.gain_dbi(-45.0), 0.0);
    }

    #[test]
    fn test_patch_peak_at_zenith() {
        let ant = AntennaPattern::default_patch();
        let zenith_gain = ant.gain_dbi(90.0);
        let horizon_gain = ant.gain_dbi(0.0);
        assert!(zenith_gain > horizon_gain, "Zenith {} > Horizon {}", zenith_gain, horizon_gain);
        assert!((zenith_gain - 5.0).abs() < 0.1); // peak gain
    }

    #[test]
    fn test_patch_below_horizon() {
        let ant = AntennaPattern::default_patch();
        let gain = ant.gain_dbi(-10.0);
        assert!(gain < -20.0, "Below horizon gain: {} dBi", gain);
    }

    #[test]
    fn test_choke_ring_sharp_cutoff() {
        let ant = AntennaPattern::ChokeRing { peak_gain_dbi: 7.0 };
        let above = ant.gain_dbi(5.0);
        let below = ant.gain_dbi(-5.0);
        assert!(above > below + 20.0, "Choke ring: above={}, below={}", above, below);
    }

    #[test]
    fn test_body_attitude_default() {
        let att = BodyAttitude::default();
        let eff = att.effective_elevation(45.0, 0.0);
        assert!((eff - 45.0).abs() < 0.01);
    }
}
