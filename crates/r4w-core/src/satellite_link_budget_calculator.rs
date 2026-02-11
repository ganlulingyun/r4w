//! Satellite Communications Link Budget Calculator
//!
//! Comprehensive link budget analysis for satellite uplink, downlink, and
//! transponder paths. Includes rain fade (ITU-R P.838), atmospheric loss
//! (ITU-R P.676 simplified), antenna pointing loss, cascaded noise
//! temperature, G/T figure of merit, BER estimation, and slant range
//! geometry.
//!
//! All math implemented from scratch with no external dependencies beyond `std`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::satellite_link_budget_calculator::{LinkBudgetConfig, LinkBudget};
//!
//! let config = LinkBudgetConfig {
//!     frequency_ghz: 12.0,
//!     distance_km: 35_786.0,
//!     tx_power_dbw: 20.0,
//!     tx_antenna_gain_dbi: 50.0,
//!     rx_antenna_gain_dbi: 40.0,
//!     rx_noise_temp_k: 150.0,
//!     bandwidth_hz: 36e6,
//!     data_rate_bps: 50e6,
//! };
//!
//! let budget = LinkBudget::new(config);
//! let result = budget.compute();
//! assert!(result.link_margin_db > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Speed of light in m/s.
const C_M_S: f64 = 299_792_458.0;

/// Boltzmann constant in dBW/K/Hz  (= 10*log10(1.380649e-23)).
const K_DBW_K_HZ: f64 = -228.6;

/// Mean Earth radius in km (for slant range geometry).
const EARTH_RADIUS_KM: f64 = 6_371.0;

// ---------------------------------------------------------------------------
// Modulation enum
// ---------------------------------------------------------------------------

/// Modulation scheme for BER estimation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    /// Binary Phase Shift Keying.
    Bpsk,
    /// Quadrature Phase Shift Keying (same BER performance as BPSK per bit).
    Qpsk,
    /// 16-Quadrature Amplitude Modulation.
    Qam16,
    /// 64-Quadrature Amplitude Modulation.
    Qam64,
    /// Binary Frequency Shift Keying (non-coherent detection).
    Fsk,
}

impl Modulation {
    /// Bits per symbol for the modulation scheme.
    pub fn bits_per_symbol(&self) -> f64 {
        match self {
            Modulation::Bpsk => 1.0,
            Modulation::Qpsk => 2.0,
            Modulation::Qam16 => 4.0,
            Modulation::Qam64 => 6.0,
            Modulation::Fsk => 1.0,
        }
    }

    /// Typical required Eb/N0 (dB) for BER = 1e-5.
    pub fn required_eb_n0_db(&self) -> f64 {
        match self {
            Modulation::Bpsk => 9.6,
            Modulation::Qpsk => 9.6,
            Modulation::Qam16 => 13.4,
            Modulation::Qam64 => 17.8,
            Modulation::Fsk => 12.6,
        }
    }
}

// ---------------------------------------------------------------------------
// LinkBudgetConfig
// ---------------------------------------------------------------------------

/// Configuration for a satellite link budget calculation.
#[derive(Debug, Clone)]
pub struct LinkBudgetConfig {
    /// Carrier frequency in GHz.
    pub frequency_ghz: f64,
    /// Distance (slant range) from transmitter to receiver in km.
    pub distance_km: f64,
    /// Transmitter power in dBW.
    pub tx_power_dbw: f64,
    /// Transmitter antenna gain in dBi.
    pub tx_antenna_gain_dbi: f64,
    /// Receiver antenna gain in dBi.
    pub rx_antenna_gain_dbi: f64,
    /// Receiver system noise temperature in Kelvin.
    pub rx_noise_temp_k: f64,
    /// Channel bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Data rate in bits per second.
    pub data_rate_bps: f64,
}

// ---------------------------------------------------------------------------
// LinkBudgetResult
// ---------------------------------------------------------------------------

/// Result of a link budget computation.
#[derive(Debug, Clone)]
pub struct LinkBudgetResult {
    /// Effective Isotropic Radiated Power (dBW).
    pub eirp_dbw: f64,
    /// Free-space path loss (dB, positive value).
    pub fspl_db: f64,
    /// Received signal power (dBW).
    pub received_power_dbw: f64,
    /// Noise power in the bandwidth (dBW).
    pub noise_power_dbw: f64,
    /// Carrier-to-noise-density ratio (dB-Hz).
    pub cn0_db_hz: f64,
    /// Energy-per-bit to noise spectral density (dB).
    pub eb_n0_db: f64,
    /// Link margin above required Eb/N0 for QPSK BER=1e-5 (dB).
    pub link_margin_db: f64,
    /// Required Eb/N0 assumed for margin calculation (dB).
    pub required_eb_n0_db: f64,
}

// ---------------------------------------------------------------------------
// LinkBudget
// ---------------------------------------------------------------------------

/// Satellite link budget calculator.
///
/// Wraps a [`LinkBudgetConfig`] and computes the full link budget.
#[derive(Debug, Clone)]
pub struct LinkBudget {
    config: LinkBudgetConfig,
}

impl LinkBudget {
    /// Create a new link budget calculator from the given configuration.
    pub fn new(config: LinkBudgetConfig) -> Self {
        Self { config }
    }

    /// Compute the link budget and return the result.
    ///
    /// Uses QPSK required Eb/N0 = 9.6 dB (BER 1e-5) for the default
    /// margin calculation.
    pub fn compute(&self) -> LinkBudgetResult {
        let c = &self.config;

        let eirp_dbw = c.tx_power_dbw + c.tx_antenna_gain_dbi;
        let fspl_db = free_space_path_loss_db(c.frequency_ghz, c.distance_km);

        let received_power_dbw = eirp_dbw - fspl_db + c.rx_antenna_gain_dbi;

        // Noise power = k * T * B  (in dBW)
        let noise_power_dbw = K_DBW_K_HZ + 10.0 * c.rx_noise_temp_k.log10()
            + 10.0 * c.bandwidth_hz.log10();

        // C/N0 = received_power - k - T  (dB-Hz)
        let cn0_db_hz = received_power_dbw - K_DBW_K_HZ - 10.0 * c.rx_noise_temp_k.log10();

        // Eb/N0 = C/N0 - 10*log10(Rb)
        let eb_n0_db = cn0_db_hz - 10.0 * c.data_rate_bps.log10();

        let required_eb_n0_db = Modulation::Qpsk.required_eb_n0_db();
        let link_margin_db = eb_n0_db - required_eb_n0_db;

        LinkBudgetResult {
            eirp_dbw,
            fspl_db,
            received_power_dbw,
            noise_power_dbw,
            cn0_db_hz,
            eb_n0_db,
            link_margin_db,
            required_eb_n0_db,
        }
    }
}

// ---------------------------------------------------------------------------
// Free-standing functions
// ---------------------------------------------------------------------------

/// Free-space path loss in dB.
///
/// FSPL = 92.45 + 20*log10(f_GHz) + 20*log10(d_km)
///
/// Valid for f > 0 and d > 0.
pub fn free_space_path_loss_db(freq_ghz: f64, distance_km: f64) -> f64 {
    92.45 + 20.0 * freq_ghz.log10() + 20.0 * distance_km.log10()
}

/// Simplified atmospheric attenuation in dB (ITU-R P.676 approximation).
///
/// Models the zenith attenuation at sea level and scales by 1/sin(elevation)
/// for the slant path. The zenith attenuation is approximated as a piecewise
/// function of frequency that captures the O2 peak near 60 GHz and the water
/// vapor line at 22 GHz.
///
/// # Arguments
/// * `freq_ghz` - Carrier frequency in GHz (valid 1..350 GHz).
/// * `elevation_deg` - Elevation angle in degrees (must be > 0).
///
/// # Returns
/// Atmospheric attenuation in dB for the slant path.
pub fn atmospheric_loss_db(freq_ghz: f64, elevation_deg: f64) -> f64 {
    if elevation_deg <= 0.0 || freq_ghz <= 0.0 {
        return 0.0;
    }

    // Approximate zenith attenuation (dB) at sea level.
    // Piecewise model capturing key spectral features:
    //   - Gentle rise below 10 GHz
    //   - Water vapor resonance near 22 GHz
    //   - Strong O2 absorption 50-70 GHz
    //   - Increase above 100 GHz
    let zenith_db = if freq_ghz < 1.0 {
        0.01
    } else if freq_ghz < 10.0 {
        // ~ 0.01 to 0.04 dB
        0.01 + 0.003 * (freq_ghz - 1.0)
    } else if freq_ghz < 15.0 {
        // Gentle rise toward water vapor
        0.04 + 0.006 * (freq_ghz - 10.0)
    } else if freq_ghz < 25.0 {
        // Water vapor bump around 22 GHz
        let delta = freq_ghz - 22.0;
        0.07 + 0.12 * (-delta * delta / 8.0).exp()
    } else if freq_ghz < 50.0 {
        // Valley between water vapor and O2
        0.05 + 0.003 * (freq_ghz - 25.0)
    } else if freq_ghz < 70.0 {
        // O2 absorption complex (peak ~15 dB at 60 GHz zenith)
        let delta = freq_ghz - 60.0;
        0.13 + 15.0 * (-delta * delta / 50.0).exp()
    } else if freq_ghz < 100.0 {
        // Decay from O2 peak
        0.13 + 0.005 * (freq_ghz - 70.0)
    } else {
        // Above 100 GHz: gradual rise
        0.28 + 0.01 * (freq_ghz - 100.0)
    };

    let elev_rad = elevation_deg * PI / 180.0;
    zenith_db / elev_rad.sin()
}

/// Rain attenuation in dB (ITU-R P.838 simplified model).
///
/// Specific attenuation: gamma_R = k * R^alpha  (dB/km)
/// where k and alpha are frequency-dependent coefficients.
///
/// The total attenuation is gamma_R * effective_path_length / sin(elevation).
///
/// # Arguments
/// * `freq_ghz` - Carrier frequency in GHz.
/// * `rain_rate_mm_hr` - Rain rate in mm/hr (e.g., 25 for heavy rain).
/// * `elevation_deg` - Elevation angle in degrees (> 0).
/// * `path_length_km` - Effective rain path length in km.
///
/// # Returns
/// Rain attenuation in dB.
pub fn rain_attenuation_db(
    freq_ghz: f64,
    rain_rate_mm_hr: f64,
    elevation_deg: f64,
    path_length_km: f64,
) -> f64 {
    if freq_ghz <= 0.0 || rain_rate_mm_hr <= 0.0 || elevation_deg <= 0.0 || path_length_km <= 0.0
    {
        return 0.0;
    }

    // ITU-R P.838 simplified coefficients (circular polarization average).
    // k and alpha as piecewise log-linear approximation of the ITU tables.
    let (k, alpha) = rain_coefficients(freq_ghz);

    // Specific attenuation (dB/km)
    let gamma_r = k * rain_rate_mm_hr.powf(alpha);

    let elev_rad = elevation_deg * PI / 180.0;
    let slant_factor = 1.0 / elev_rad.sin();

    gamma_r * path_length_km * slant_factor
}

/// Approximate ITU-R P.838 rain coefficients (k, alpha) for a given frequency.
///
/// These are simplified circular-polarization-average values.
fn rain_coefficients(freq_ghz: f64) -> (f64, f64) {
    // Piecewise log-linear approximation of ITU-R P.838 Table 1.
    // Selected anchor points (freq_GHz, k, alpha):
    //   1 GHz:  (0.0000387, 0.912)
    //   4 GHz:  (0.000650,  1.121)
    //  10 GHz:  (0.0101,    1.276)
    //  20 GHz:  (0.0751,    1.099)
    //  30 GHz:  (0.187,     1.021)
    //  50 GHz:  (0.536,     0.873)
    // 100 GHz:  (1.06,      0.748)
    let log_f = freq_ghz.ln();

    // k: log-log interpolation
    let log_k = if freq_ghz <= 1.0 {
        (-10.16_f64).ln() // k ~ 0.0000387
    } else if freq_ghz <= 4.0 {
        lerp(
            1.0_f64.ln(),
            4.0_f64.ln(),
            0.0000387_f64.ln(),
            0.000650_f64.ln(),
            log_f,
        )
    } else if freq_ghz <= 10.0 {
        lerp(
            4.0_f64.ln(),
            10.0_f64.ln(),
            0.000650_f64.ln(),
            0.0101_f64.ln(),
            log_f,
        )
    } else if freq_ghz <= 20.0 {
        lerp(
            10.0_f64.ln(),
            20.0_f64.ln(),
            0.0101_f64.ln(),
            0.0751_f64.ln(),
            log_f,
        )
    } else if freq_ghz <= 30.0 {
        lerp(
            20.0_f64.ln(),
            30.0_f64.ln(),
            0.0751_f64.ln(),
            0.187_f64.ln(),
            log_f,
        )
    } else if freq_ghz <= 50.0 {
        lerp(
            30.0_f64.ln(),
            50.0_f64.ln(),
            0.187_f64.ln(),
            0.536_f64.ln(),
            log_f,
        )
    } else {
        lerp(
            50.0_f64.ln(),
            100.0_f64.ln(),
            0.536_f64.ln(),
            1.06_f64.ln(),
            log_f,
        )
    };

    // alpha: linear interpolation in log-frequency space
    let alpha = if freq_ghz <= 1.0 {
        0.912
    } else if freq_ghz <= 10.0 {
        lerp(1.0_f64.ln(), 10.0_f64.ln(), 0.912, 1.276, log_f)
    } else if freq_ghz <= 30.0 {
        lerp(10.0_f64.ln(), 30.0_f64.ln(), 1.276, 1.021, log_f)
    } else if freq_ghz <= 100.0 {
        lerp(30.0_f64.ln(), 100.0_f64.ln(), 1.021, 0.748, log_f)
    } else {
        0.748
    };

    (log_k.exp(), alpha)
}

/// Linear interpolation helper.
fn lerp(x0: f64, x1: f64, y0: f64, y1: f64, x: f64) -> f64 {
    let t = (x - x0) / (x1 - x0);
    y0 + t * (y1 - y0)
}

/// Antenna pointing loss in dB.
///
/// Approximation: loss = 12 * (pointing_error / beamwidth)^2
///
/// # Arguments
/// * `pointing_error_deg` - Antenna pointing error in degrees.
/// * `beamwidth_deg` - Antenna half-power (3 dB) beamwidth in degrees.
///
/// # Returns
/// Pointing loss in dB (always >= 0).
pub fn antenna_pointing_loss_db(pointing_error_deg: f64, beamwidth_deg: f64) -> f64 {
    if beamwidth_deg <= 0.0 {
        return 0.0;
    }
    let ratio = pointing_error_deg / beamwidth_deg;
    12.0 * ratio * ratio
}

/// Antenna gain in dBi from physical parameters.
///
/// G = 10*log10( eta * (pi * D / lambda)^2 )
///
/// where lambda = c / f.
///
/// # Arguments
/// * `diameter_m` - Antenna diameter in meters.
/// * `freq_ghz` - Carrier frequency in GHz.
/// * `efficiency` - Aperture efficiency (0.0 to 1.0, typically 0.55-0.70).
///
/// # Returns
/// Antenna gain in dBi.
pub fn antenna_gain_dbi(diameter_m: f64, freq_ghz: f64, efficiency: f64) -> f64 {
    let freq_hz = freq_ghz * 1e9;
    let lambda = C_M_S / freq_hz;
    let ratio = PI * diameter_m / lambda;
    10.0 * (efficiency * ratio * ratio).log10()
}

/// System noise temperature using cascaded (Friis) noise formula.
///
/// T_sys = T_antenna + T_lna + T_following / G_lna
///
/// where G_lna is the LNA power gain (linear).
///
/// # Arguments
/// * `t_antenna_k` - Antenna noise temperature (K).
/// * `t_lna_k` - LNA noise temperature (K).
/// * `lna_gain_db` - LNA gain in dB.
/// * `t_following_k` - Noise temperature of the following stage (K).
///
/// # Returns
/// System noise temperature in Kelvin.
pub fn noise_temperature_system(
    t_antenna_k: f64,
    t_lna_k: f64,
    lna_gain_db: f64,
    t_following_k: f64,
) -> f64 {
    let lna_gain_linear = db_to_linear(lna_gain_db);
    t_antenna_k + t_lna_k + t_following_k / lna_gain_linear
}

/// Figure of merit G/T in dB/K.
///
/// G/T = G_dBi - 10*log10(T_sys)
///
/// # Arguments
/// * `gain_dbi` - Antenna gain in dBi.
/// * `system_temp_k` - System noise temperature in Kelvin.
///
/// # Returns
/// G/T in dB/K.
pub fn g_over_t_db(gain_dbi: f64, system_temp_k: f64) -> f64 {
    gain_dbi - 10.0 * system_temp_k.log10()
}

/// Estimate BER from Eb/N0 for a given modulation scheme.
///
/// Uses closed-form approximations:
/// - **BPSK/QPSK**: Q(sqrt(2 * Eb/N0)) where Q(x) = 0.5 * erfc(x/sqrt(2))
/// - **16-QAM**: (3/8) * erfc(sqrt(2/5 * Eb/N0))
/// - **64-QAM**: (7/24) * erfc(sqrt(1/7 * Eb/N0))
/// - **FSK** (non-coherent): 0.5 * exp(-Eb/(2*N0))
///
/// # Arguments
/// * `eb_n0_db` - Eb/N0 in dB.
/// * `modulation` - The modulation scheme.
///
/// # Returns
/// Estimated bit error rate (BER), clamped to [0.0, 0.5].
pub fn eb_n0_to_ber(eb_n0_db: f64, modulation: Modulation) -> f64 {
    let eb_n0_lin = db_to_linear(eb_n0_db);

    let ber = match modulation {
        Modulation::Bpsk | Modulation::Qpsk => {
            // Q(sqrt(2*Eb/N0))
            q_function((2.0 * eb_n0_lin).sqrt())
        }
        Modulation::Qam16 => {
            // Approximate: (3/8) * erfc(sqrt(2/5 * Eb/N0))
            // erfc(x) = 2*Q(x*sqrt(2))
            let arg = (2.0 / 5.0 * eb_n0_lin).sqrt();
            (3.0 / 8.0) * erfc_approx(arg)
        }
        Modulation::Qam64 => {
            // Approximate: (7/24) * erfc(sqrt(1/7 * Eb/N0))
            let arg = (eb_n0_lin / 7.0).sqrt();
            (7.0 / 24.0) * erfc_approx(arg)
        }
        Modulation::Fsk => {
            // Non-coherent FSK: 0.5 * exp(-Eb/N0 / 2)
            0.5 * (-eb_n0_lin / 2.0).exp()
        }
    };

    ber.clamp(0.0, 0.5)
}

/// Slant range from a ground station to a satellite in km.
///
/// Uses the geometric relation:
///   d = sqrt( (R_e + h)^2 - (R_e * cos(el))^2 ) - R_e * sin(el)
///
/// where R_e is Earth's mean radius, h is satellite altitude, and el is
/// the elevation angle.
///
/// # Arguments
/// * `altitude_km` - Satellite altitude above mean sea level in km.
/// * `elevation_deg` - Elevation angle from the ground station in degrees.
///
/// # Returns
/// Slant range in km.
pub fn slant_range_km(altitude_km: f64, elevation_deg: f64) -> f64 {
    let r_e = EARTH_RADIUS_KM;
    let r_s = r_e + altitude_km;
    let el_rad = elevation_deg * PI / 180.0;

    let cos_el = el_rad.cos();
    let sin_el = el_rad.sin();

    // d = -R_e*sin(el) + sqrt( (R_e*sin(el))^2 + r_s^2 - R_e^2 )
    let inner = (r_e * sin_el) * (r_e * sin_el) + r_s * r_s - r_e * r_e;
    if inner < 0.0 {
        return 0.0;
    }

    let d = -r_e * sin_el + inner.sqrt();
    // Sanity: negative distance shouldn't happen for valid inputs.
    // But this can occur if cos_el > r_s/r_e (satellite below horizon).
    // We just return the geometric value; caller interprets.
    let _ = cos_el; // suppress unused warning; used in derivation context
    d.max(0.0)
}

// ---------------------------------------------------------------------------
// Transponder analysis helpers
// ---------------------------------------------------------------------------

/// Compute total C/N0 for a transparent (bent-pipe) transponder.
///
/// 1/(C/N0_total) = 1/(C/N0_up) + 1/(C/N0_down) + 1/(C/N0_intermod)
///
/// All arguments and the return value are in dB-Hz.
pub fn transponder_cn0_db_hz(
    cn0_up_db_hz: f64,
    cn0_down_db_hz: f64,
    cn0_intermod_db_hz: f64,
) -> f64 {
    let up_lin = db_to_linear(cn0_up_db_hz);
    let down_lin = db_to_linear(cn0_down_db_hz);
    let im_lin = db_to_linear(cn0_intermod_db_hz);

    let total_inv = 1.0 / up_lin + 1.0 / down_lin + 1.0 / im_lin;
    10.0 * (1.0 / total_inv).log10()
}

/// Back-off calculation: output power given saturated power and back-off.
///
/// P_out = P_sat - OBO
///
/// Returns power in dBW.
pub fn output_backoff_dbw(saturated_power_dbw: f64, output_backoff_db: f64) -> f64 {
    saturated_power_dbw - output_backoff_db
}

// ---------------------------------------------------------------------------
// Internal math helpers
// ---------------------------------------------------------------------------

/// Convert dB to linear power ratio.
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

/// Convert linear power ratio to dB.
#[allow(dead_code)]
fn linear_to_db(lin: f64) -> f64 {
    10.0 * lin.log10()
}

/// Q-function: Q(x) = 0.5 * erfc(x / sqrt(2)).
///
/// Uses a rational approximation of erfc for accuracy.
fn q_function(x: f64) -> f64 {
    0.5 * erfc_approx(x / core::f64::consts::SQRT_2)
}

/// Complementary error function approximation (Abramowitz & Stegun 7.1.26).
///
/// Maximum error < 1.5e-7 for x >= 0.
fn erfc_approx(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }

    // Abramowitz & Stegun rational approximation
    let p = 0.3275911;
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;

    let t = 1.0 / (1.0 + p * x);
    let t2 = t * t;
    let t3 = t2 * t;
    let t4 = t3 * t;
    let t5 = t4 * t;

    let poly = a1 * t + a2 * t2 + a3 * t3 + a4 * t4 + a5 * t5;
    poly * (-x * x).exp()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 0.1; // dB tolerance for most link budget comparisons
    const EPSILON_FINE: f64 = 0.01;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // FSPL tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fspl_1ghz_100km() {
        // FSPL = 92.45 + 20*log10(1) + 20*log10(100) = 92.45 + 0 + 40 = 132.45
        let fspl = free_space_path_loss_db(1.0, 100.0);
        assert!(
            approx_eq(fspl, 132.45, EPSILON_FINE),
            "FSPL at 1 GHz, 100 km: expected 132.45, got {fspl}"
        );
    }

    #[test]
    fn test_fspl_12ghz_geo() {
        // GEO distance ~35,786 km, 12 GHz (Ku-band)
        // FSPL = 92.45 + 20*log10(12) + 20*log10(35786)
        //      = 92.45 + 21.58 + 91.07 = 205.10
        let fspl = free_space_path_loss_db(12.0, 35_786.0);
        assert!(
            approx_eq(fspl, 205.10, 0.15),
            "FSPL at 12 GHz, GEO: expected ~205.10, got {fspl}"
        );
    }

    #[test]
    fn test_fspl_4ghz_1km() {
        // FSPL = 92.45 + 20*log10(4) + 20*log10(1) = 92.45 + 12.04 + 0 = 104.49
        let fspl = free_space_path_loss_db(4.0, 1.0);
        assert!(
            approx_eq(fspl, 104.49, EPSILON),
            "FSPL at 4 GHz, 1 km: expected 104.49, got {fspl}"
        );
    }

    #[test]
    fn test_fspl_frequency_doubling() {
        // Doubling frequency adds 6.02 dB
        let fspl_f = free_space_path_loss_db(2.0, 100.0);
        let fspl_2f = free_space_path_loss_db(4.0, 100.0);
        let diff = fspl_2f - fspl_f;
        assert!(
            approx_eq(diff, 6.02, EPSILON),
            "Doubling frequency should add ~6 dB, got {diff}"
        );
    }

    #[test]
    fn test_fspl_distance_doubling() {
        // Doubling distance adds 6.02 dB
        let fspl_d = free_space_path_loss_db(10.0, 100.0);
        let fspl_2d = free_space_path_loss_db(10.0, 200.0);
        let diff = fspl_2d - fspl_d;
        assert!(
            approx_eq(diff, 6.02, EPSILON),
            "Doubling distance should add ~6 dB, got {diff}"
        );
    }

    // -----------------------------------------------------------------------
    // Antenna gain tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_antenna_gain_1m_12ghz() {
        // lambda = 0.025 m, D = 1m, eta = 0.6
        // G = 10*log10(0.6 * (pi * 1.0 / 0.025)^2)
        //   = 10*log10(0.6 * 15791.4) = 10*log10(9474.8) = 39.76 dBi
        let gain = antenna_gain_dbi(1.0, 12.0, 0.6);
        assert!(
            approx_eq(gain, 39.76, 0.2),
            "1m dish at 12 GHz, 60% eff: expected ~39.76 dBi, got {gain}"
        );
    }

    #[test]
    fn test_antenna_gain_3m_4ghz() {
        // lambda = 0.075m, D = 3m, eta = 0.55
        // ratio = pi*3/0.075 = 125.66
        // G = 10*log10(0.55 * 125.66^2) = 10*log10(0.55 * 15790.6) = 10*log10(8684.8) = 39.39
        let gain = antenna_gain_dbi(3.0, 4.0, 0.55);
        assert!(
            approx_eq(gain, 39.39, 0.2),
            "3m dish at 4 GHz, 55% eff: expected ~39.39 dBi, got {gain}"
        );
    }

    #[test]
    fn test_antenna_gain_increases_with_diameter() {
        let g1 = antenna_gain_dbi(1.0, 12.0, 0.6);
        let g2 = antenna_gain_dbi(2.0, 12.0, 0.6);
        // Doubling diameter should add ~6 dB (area quadruples)
        let diff = g2 - g1;
        assert!(
            approx_eq(diff, 6.02, EPSILON),
            "Doubling diameter should add ~6 dB, got {diff}"
        );
    }

    // -----------------------------------------------------------------------
    // Noise temperature tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_noise_temperature_basic() {
        // T_ant=30K, T_lna=35K, G_lna=40dB (10000x), T_follow=1000K
        // T_sys = 30 + 35 + 1000/10000 = 65.1 K
        let t_sys = noise_temperature_system(30.0, 35.0, 40.0, 1000.0);
        assert!(
            approx_eq(t_sys, 65.1, 0.2),
            "System temp: expected ~65.1 K, got {t_sys}"
        );
    }

    #[test]
    fn test_noise_temperature_low_gain_lna() {
        // With lower LNA gain, following stages contribute more
        // T_ant=50K, T_lna=50K, G_lna=10dB (10x), T_follow=1000K
        // T_sys = 50 + 50 + 1000/10 = 200 K
        let t_sys = noise_temperature_system(50.0, 50.0, 10.0, 1000.0);
        assert!(
            approx_eq(t_sys, 200.0, 0.5),
            "System temp with low-gain LNA: expected ~200 K, got {t_sys}"
        );
    }

    #[test]
    fn test_noise_temperature_high_gain_suppresses_following() {
        // With very high LNA gain, following stages are negligible
        let t_low = noise_temperature_system(30.0, 35.0, 60.0, 5000.0);
        let t_high = noise_temperature_system(30.0, 35.0, 60.0, 50000.0);
        // Both should be very close to 65 K
        assert!(
            (t_high - t_low).abs() < 0.5,
            "High LNA gain should suppress following stage noise: diff={}",
            (t_high - t_low).abs()
        );
    }

    // -----------------------------------------------------------------------
    // G/T tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_g_over_t_basic() {
        // G=40 dBi, T=200K => G/T = 40 - 10*log10(200) = 40 - 23.01 = 16.99 dB/K
        let gt = g_over_t_db(40.0, 200.0);
        assert!(
            approx_eq(gt, 16.99, EPSILON),
            "G/T: expected ~16.99 dB/K, got {gt}"
        );
    }

    #[test]
    fn test_g_over_t_low_temp() {
        // G=30 dBi, T=50K => G/T = 30 - 16.99 = 13.01 dB/K
        let gt = g_over_t_db(30.0, 50.0);
        assert!(
            approx_eq(gt, 13.01, EPSILON),
            "G/T low temp: expected ~13.01 dB/K, got {gt}"
        );
    }

    // -----------------------------------------------------------------------
    // Atmospheric loss tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_atmospheric_loss_low_freq_zenith() {
        // At 4 GHz, zenith (90 deg), should be < 0.1 dB
        let loss = atmospheric_loss_db(4.0, 90.0);
        assert!(
            loss < 0.1 && loss > 0.0,
            "Atmospheric loss at 4 GHz zenith should be small, got {loss}"
        );
    }

    #[test]
    fn test_atmospheric_loss_60ghz_zenith() {
        // Near 60 GHz O2 absorption peak, zenith loss should be large (>10 dB)
        let loss = atmospheric_loss_db(60.0, 90.0);
        assert!(
            loss > 10.0,
            "Atmospheric loss at 60 GHz zenith should be >10 dB, got {loss}"
        );
    }

    #[test]
    fn test_atmospheric_loss_increases_at_low_elevation() {
        // Lower elevation means longer path through atmosphere
        let loss_high = atmospheric_loss_db(12.0, 90.0);
        let loss_low = atmospheric_loss_db(12.0, 10.0);
        assert!(
            loss_low > loss_high,
            "Loss at 10 deg ({loss_low}) should exceed loss at 90 deg ({loss_high})"
        );
    }

    #[test]
    fn test_atmospheric_loss_zero_elevation() {
        // At 0 degrees elevation, function should return 0 (degenerate)
        let loss = atmospheric_loss_db(12.0, 0.0);
        assert_eq!(loss, 0.0, "Loss at 0 deg elevation should be 0");
    }

    // -----------------------------------------------------------------------
    // Rain attenuation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rain_attenuation_12ghz() {
        // Ku-band, moderate rain (25 mm/hr), 30 deg, 5 km path
        let atten = rain_attenuation_db(12.0, 25.0, 30.0, 5.0);
        // Should be in the range of a few dB
        assert!(
            atten > 1.0 && atten < 30.0,
            "Rain atten at 12 GHz, 25 mm/hr: expected 1-30 dB, got {atten}"
        );
    }

    #[test]
    fn test_rain_attenuation_increases_with_frequency() {
        let atten_4 = rain_attenuation_db(4.0, 25.0, 30.0, 5.0);
        let atten_20 = rain_attenuation_db(20.0, 25.0, 30.0, 5.0);
        assert!(
            atten_20 > atten_4,
            "Rain atten at 20 GHz ({atten_20}) should exceed 4 GHz ({atten_4})"
        );
    }

    #[test]
    fn test_rain_attenuation_increases_with_rain_rate() {
        let atten_low = rain_attenuation_db(12.0, 5.0, 30.0, 5.0);
        let atten_high = rain_attenuation_db(12.0, 50.0, 30.0, 5.0);
        assert!(
            atten_high > atten_low,
            "Higher rain rate should give higher attenuation"
        );
    }

    #[test]
    fn test_rain_attenuation_zero_rain() {
        let atten = rain_attenuation_db(12.0, 0.0, 30.0, 5.0);
        assert_eq!(atten, 0.0, "Zero rain rate should give zero attenuation");
    }

    // -----------------------------------------------------------------------
    // Pointing loss tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_pointing_loss_zero_error() {
        let loss = antenna_pointing_loss_db(0.0, 2.0);
        assert_eq!(loss, 0.0, "Zero pointing error should give zero loss");
    }

    #[test]
    fn test_pointing_loss_half_beamwidth() {
        // 12 * (0.5)^2 = 3 dB
        let loss = antenna_pointing_loss_db(1.0, 2.0);
        assert!(
            approx_eq(loss, 3.0, EPSILON_FINE),
            "Pointing loss at half-beamwidth: expected 3 dB, got {loss}"
        );
    }

    #[test]
    fn test_pointing_loss_full_beamwidth() {
        // 12 * (1.0)^2 = 12 dB
        let loss = antenna_pointing_loss_db(2.0, 2.0);
        assert!(
            approx_eq(loss, 12.0, EPSILON_FINE),
            "Pointing loss at full beamwidth: expected 12 dB, got {loss}"
        );
    }

    #[test]
    fn test_pointing_loss_zero_beamwidth() {
        let loss = antenna_pointing_loss_db(1.0, 0.0);
        assert_eq!(loss, 0.0, "Zero beamwidth should return 0 (guard)");
    }

    // -----------------------------------------------------------------------
    // BER tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_ber_bpsk_high_snr() {
        // At Eb/N0 = 10 dB, BPSK BER should be around 3.87e-6
        let ber = eb_n0_to_ber(10.0, Modulation::Bpsk);
        assert!(
            ber < 1e-4 && ber > 1e-8,
            "BPSK BER at 10 dB: expected ~3.87e-6, got {ber}"
        );
    }

    #[test]
    fn test_ber_bpsk_low_snr() {
        // At Eb/N0 = 0 dB, BPSK BER should be around 0.079
        let ber = eb_n0_to_ber(0.0, Modulation::Bpsk);
        assert!(
            approx_eq(ber, 0.079, 0.01),
            "BPSK BER at 0 dB: expected ~0.079, got {ber}"
        );
    }

    #[test]
    fn test_ber_qpsk_same_as_bpsk() {
        // QPSK has identical BER per bit as BPSK
        let ber_bpsk = eb_n0_to_ber(8.0, Modulation::Bpsk);
        let ber_qpsk = eb_n0_to_ber(8.0, Modulation::Qpsk);
        assert!(
            approx_eq(ber_bpsk, ber_qpsk, 1e-10),
            "BPSK and QPSK BER should match: {ber_bpsk} vs {ber_qpsk}"
        );
    }

    #[test]
    fn test_ber_16qam_worse_than_bpsk() {
        // At same Eb/N0, 16-QAM has higher BER than BPSK
        let ber_bpsk = eb_n0_to_ber(10.0, Modulation::Bpsk);
        let ber_qam16 = eb_n0_to_ber(10.0, Modulation::Qam16);
        assert!(
            ber_qam16 > ber_bpsk,
            "16-QAM BER ({ber_qam16}) should exceed BPSK ({ber_bpsk}) at same Eb/N0"
        );
    }

    #[test]
    fn test_ber_64qam_worse_than_16qam() {
        let ber_16 = eb_n0_to_ber(15.0, Modulation::Qam16);
        let ber_64 = eb_n0_to_ber(15.0, Modulation::Qam64);
        assert!(
            ber_64 > ber_16,
            "64-QAM BER ({ber_64}) should exceed 16-QAM ({ber_16}) at same Eb/N0"
        );
    }

    #[test]
    fn test_ber_fsk_noncoherent() {
        // FSK non-coherent at Eb/N0 = 10 dB: 0.5*exp(-5) ~ 0.00337
        let ber = eb_n0_to_ber(10.0, Modulation::Fsk);
        assert!(
            approx_eq(ber, 0.00337, 0.001),
            "FSK BER at 10 dB: expected ~0.00337, got {ber}"
        );
    }

    #[test]
    fn test_ber_clamped_at_half() {
        // Very negative Eb/N0 should give BER capped at 0.5
        let ber = eb_n0_to_ber(-20.0, Modulation::Bpsk);
        assert!(
            ber <= 0.5,
            "BER should be capped at 0.5, got {ber}"
        );
    }

    // -----------------------------------------------------------------------
    // Slant range tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_slant_range_geo_zenith() {
        // GEO at 35786 km, 90 deg elevation: slant range should equal altitude
        let range = slant_range_km(35_786.0, 90.0);
        assert!(
            approx_eq(range, 35_786.0, 1.0),
            "GEO slant range at zenith: expected ~35786 km, got {range}"
        );
    }

    #[test]
    fn test_slant_range_geo_low_elevation() {
        // At lower elevation, slant range increases
        let range_90 = slant_range_km(35_786.0, 90.0);
        let range_10 = slant_range_km(35_786.0, 10.0);
        assert!(
            range_10 > range_90,
            "Slant range at 10 deg ({range_10}) should exceed 90 deg ({range_90})"
        );
    }

    #[test]
    fn test_slant_range_leo_zenith() {
        // LEO at 550 km, zenith: should be ~550 km
        let range = slant_range_km(550.0, 90.0);
        assert!(
            approx_eq(range, 550.0, 1.0),
            "LEO slant range at zenith: expected ~550 km, got {range}"
        );
    }

    #[test]
    fn test_slant_range_increases_with_lower_elevation() {
        let r_high = slant_range_km(550.0, 45.0);
        let r_low = slant_range_km(550.0, 15.0);
        assert!(
            r_low > r_high,
            "Slant range at 15 deg ({r_low}) should exceed 45 deg ({r_high})"
        );
    }

    // -----------------------------------------------------------------------
    // Full link budget tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_link_budget_geo_ku_band() {
        let config = LinkBudgetConfig {
            frequency_ghz: 12.0,
            distance_km: 35_786.0,
            tx_power_dbw: 20.0,
            tx_antenna_gain_dbi: 50.0,
            rx_antenna_gain_dbi: 40.0,
            rx_noise_temp_k: 150.0,
            bandwidth_hz: 36e6,
            data_rate_bps: 50e6,
        };

        let result = LinkBudget::new(config).compute();

        // EIRP = 20 + 50 = 70 dBW
        assert!(
            approx_eq(result.eirp_dbw, 70.0, EPSILON_FINE),
            "EIRP: expected 70 dBW, got {}",
            result.eirp_dbw
        );

        // FSPL at 12 GHz, 35786 km: ~205.1 dB
        assert!(
            approx_eq(result.fspl_db, 205.1, 0.2),
            "FSPL: expected ~205.1, got {}",
            result.fspl_db
        );

        // Received power: 70 - 205.1 + 40 = -95.1 dBW
        assert!(
            approx_eq(result.received_power_dbw, -95.1, 0.3),
            "Rx power: expected ~-95.1 dBW, got {}",
            result.received_power_dbw
        );

        // C/N0: received_power - k - 10*log10(T)
        //     = -95.1 - (-228.6) - 10*log10(150) = -95.1 + 228.6 - 21.76 = 111.74
        assert!(
            approx_eq(result.cn0_db_hz, 111.7, 0.5),
            "C/N0: expected ~111.7 dB-Hz, got {}",
            result.cn0_db_hz
        );

        // Eb/N0: C/N0 - 10*log10(50e6) = 111.7 - 76.99 = 34.7
        assert!(
            result.eb_n0_db > 30.0,
            "Eb/N0 should be > 30 dB, got {}",
            result.eb_n0_db
        );

        // Margin should be positive
        assert!(
            result.link_margin_db > 20.0,
            "Link margin should be > 20 dB, got {}",
            result.link_margin_db
        );
    }

    #[test]
    fn test_link_budget_eirp_composition() {
        let config = LinkBudgetConfig {
            frequency_ghz: 4.0,
            distance_km: 1000.0,
            tx_power_dbw: 10.0,
            tx_antenna_gain_dbi: 30.0,
            rx_antenna_gain_dbi: 20.0,
            rx_noise_temp_k: 290.0,
            bandwidth_hz: 1e6,
            data_rate_bps: 1e6,
        };

        let result = LinkBudget::new(config).compute();
        assert!(
            approx_eq(result.eirp_dbw, 40.0, EPSILON_FINE),
            "EIRP = Pt + Gt: expected 40 dBW, got {}",
            result.eirp_dbw
        );
    }

    // -----------------------------------------------------------------------
    // Transponder tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_transponder_cn0_equal_links() {
        // When all three C/N0 values are the same, total is ~4.77 dB lower
        // 1/total = 3/x => total = x/3 => 10*log10(1/3) = -4.77 dB
        let cn0 = 90.0;
        let total = transponder_cn0_db_hz(cn0, cn0, cn0);
        assert!(
            approx_eq(total, cn0 - 4.77, 0.1),
            "Transponder C/N0: expected {}, got {total}",
            cn0 - 4.77
        );
    }

    #[test]
    fn test_transponder_cn0_dominated_by_weakest() {
        // One very weak link dominates
        let total = transponder_cn0_db_hz(90.0, 70.0, 100.0);
        // Should be close to 70 dB-Hz (weakest link dominates)
        assert!(
            total < 70.5,
            "Transponder C/N0 should be near 70 dB-Hz, got {total}"
        );
    }

    // -----------------------------------------------------------------------
    // Output back-off test
    // -----------------------------------------------------------------------

    #[test]
    fn test_output_backoff() {
        let p_out = output_backoff_dbw(20.0, 3.0);
        assert!(
            approx_eq(p_out, 17.0, EPSILON_FINE),
            "Output with 3 dB back-off: expected 17 dBW, got {p_out}"
        );
    }

    // -----------------------------------------------------------------------
    // Edge cases and helpers
    // -----------------------------------------------------------------------

    #[test]
    fn test_db_to_linear_roundtrip() {
        let db_val = 13.0;
        let lin = db_to_linear(db_val);
        let back = linear_to_db(lin);
        assert!(
            approx_eq(back, db_val, 1e-10),
            "dB roundtrip: expected {db_val}, got {back}"
        );
    }

    #[test]
    fn test_erfc_approx_at_zero() {
        // erfc(0) = 1.0
        let val = erfc_approx(0.0);
        assert!(
            approx_eq(val, 1.0, 1e-6),
            "erfc(0) should be 1.0, got {val}"
        );
    }

    #[test]
    fn test_erfc_approx_large_x() {
        // erfc(4) should be very small (~1.54e-8)
        let val = erfc_approx(4.0);
        assert!(
            val < 1e-6 && val > 0.0,
            "erfc(4) should be very small and positive, got {val}"
        );
    }

    #[test]
    fn test_erfc_approx_negative() {
        // erfc(-x) = 2 - erfc(x)
        let val_pos = erfc_approx(1.0);
        let val_neg = erfc_approx(-1.0);
        assert!(
            approx_eq(val_neg, 2.0 - val_pos, 1e-6),
            "erfc(-1) should be 2-erfc(1): expected {}, got {val_neg}",
            2.0 - val_pos
        );
    }

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(Modulation::Bpsk.bits_per_symbol(), 1.0);
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2.0);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4.0);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6.0);
        assert_eq!(Modulation::Fsk.bits_per_symbol(), 1.0);
    }

    #[test]
    fn test_rain_coefficients_monotonic_k() {
        // k should increase with frequency
        let (k4, _) = rain_coefficients(4.0);
        let (k12, _) = rain_coefficients(12.0);
        let (k30, _) = rain_coefficients(30.0);
        assert!(
            k4 < k12 && k12 < k30,
            "Rain coefficient k should increase with frequency: k4={k4}, k12={k12}, k30={k30}"
        );
    }

    #[test]
    fn test_slant_range_horizon() {
        // At 0 degrees elevation, range should be maximum for the orbit
        let range = slant_range_km(35_786.0, 0.01);
        // Should be very large (> 40,000 km)
        assert!(
            range > 40_000.0,
            "Slant range near horizon should be very large, got {range}"
        );
    }
}
