//! Radiated electromagnetic immunity simulation per IEC 61000-4-3.
//!
//! This module simulates radiated electromagnetic field coupling to receiver
//! systems for EMC/EMI analysis and immunity testing. It provides electric field
//! strength computation, antenna factor conversion, field-to-cable coupling
//! models (common-mode and differential-mode), shield effectiveness calculation,
//! frequency sweep generation, susceptibility threshold detection, and test level
//! classification.
//!
//! Note: This is distinct from [`crate::emi_conducted_analyzer`] which handles
//! conducted EMI per CISPR. This module focuses on **radiated immunity**
//! simulation and field coupling per IEC 61000-4-3.
//!
//! # Example
//!
//! ```
//! use r4w_core::emc_radiated_immunity::{
//!     EmcConfig, EmcSimulator, Polarization, TestLevel,
//!     field_strength_from_eirp, classify_test_level,
//! };
//!
//! // Compute the electric field at 3 m from a 37 dBm EIRP source
//! let e_field = field_strength_from_eirp(37.0, 3.0);
//! assert!(e_field > 0.0);
//!
//! // Classify the resulting test level
//! let level = classify_test_level(e_field);
//!
//! // Set up a full simulation
//! let config = EmcConfig {
//!     frequency_hz: 100e6,
//!     field_strength_vm: 10.0,
//!     distance_m: 3.0,
//!     polarization: Polarization::Vertical,
//! };
//! let sim = EmcSimulator::new(config);
//! let sweep = sim.generate_frequency_sweep(80e6, 1e9, 10);
//! assert!(!sweep.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Polarization of the radiated field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarization {
    /// Electric field vector is horizontal.
    Horizontal,
    /// Electric field vector is vertical.
    Vertical,
}

/// IEC 61000-4-3 test levels.
///
/// Each variant specifies the field strength used during the immunity test.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TestLevel {
    /// 1 V/m — residential, light industrial.
    Level1,
    /// 3 V/m — industrial.
    Level2,
    /// 10 V/m — heavy industrial.
    Level3,
    /// 30 V/m — extreme / military.
    Level4,
}

impl TestLevel {
    /// Return the nominal field strength in V/m for this test level.
    pub fn field_strength_vm(&self) -> f64 {
        match self {
            TestLevel::Level1 => 1.0,
            TestLevel::Level2 => 3.0,
            TestLevel::Level3 => 10.0,
            TestLevel::Level4 => 30.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for a radiated immunity simulation.
#[derive(Debug, Clone)]
pub struct EmcConfig {
    /// Frequency of the test signal in Hz.
    pub frequency_hz: f64,
    /// Electric field strength at the EUT in V/m.
    pub field_strength_vm: f64,
    /// Distance from antenna to EUT in metres.
    pub distance_m: f64,
    /// Polarization of the incident field.
    pub polarization: Polarization,
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Speed of light in m/s.
const C: f64 = 299_792_458.0;

/// Compute the electric field strength (V/m) at a given distance from a source
/// with the specified EIRP (in dBm).
///
/// Uses the far-field relation:  E = sqrt(30 * P) / d  where P is EIRP in watts.
pub fn field_strength_from_eirp(eirp_dbm: f64, distance_m: f64) -> f64 {
    assert!(distance_m > 0.0, "distance must be positive");
    let eirp_w = 10.0_f64.powf((eirp_dbm - 30.0) / 10.0);
    (30.0 * eirp_w).sqrt() / distance_m
}

/// Compute the required EIRP (dBm) to produce a given field strength (V/m) at
/// a specified distance.
///
/// Inverse of [`field_strength_from_eirp`].
pub fn eirp_from_field_strength(field_vm: f64, distance_m: f64) -> f64 {
    assert!(distance_m > 0.0, "distance must be positive");
    assert!(field_vm > 0.0, "field strength must be positive");
    let eirp_w = (field_vm * distance_m).powi(2) / 30.0;
    10.0 * eirp_w.log10() + 30.0
}

/// Convert antenna factor (dB/m) to antenna gain (dBi) at the given frequency.
///
/// Relationship: `G(dBi) = -29.79 + 20*log10(f_MHz) - AF(dB/m)`
pub fn antenna_factor_to_gain(af_db: f64, freq_hz: f64) -> f64 {
    assert!(freq_hz > 0.0, "frequency must be positive");
    let freq_mhz = freq_hz / 1e6;
    -29.79 + 20.0 * freq_mhz.log10() - af_db
}

/// Compute common-mode induced voltage (V) on a cable from an incident field.
///
/// A simplified transfer-impedance model:
/// `V_cm = E * L_eff`  where  `L_eff = min(cable_length, lambda/4)`.
/// For electrically short cables the coupling is proportional to length;
/// beyond a quarter wavelength coupling saturates.
pub fn cable_coupling_cm(field_vm: f64, cable_length_m: f64, freq_hz: f64) -> f64 {
    assert!(freq_hz > 0.0, "frequency must be positive");
    assert!(cable_length_m >= 0.0, "cable length must be non-negative");
    let wavelength = C / freq_hz;
    let l_eff = cable_length_m.min(wavelength / 4.0);
    field_vm * l_eff
}

/// Compute differential-mode voltage from a common-mode voltage given the
/// cable/circuit imbalance in dB.
///
/// `V_dm = V_cm * 10^(-imbalance_dB / 20)`
pub fn cable_coupling_dm(cm_voltage: f64, imbalance_db: f64) -> f64 {
    cm_voltage * 10.0_f64.powf(-imbalance_db / 20.0)
}

/// Compute shielding effectiveness (dB) of a solid conductive barrier.
///
/// Uses the plane-wave absorption loss approximation:
/// `SE = 8.686 * t / delta` where `delta = sqrt(1 / (pi*f*mu_0*sigma))` is the
/// skin depth, `t` is thickness in metres, `sigma` is conductivity in S/m, and
/// `f` is frequency in Hz.
pub fn shield_effectiveness(thickness_mm: f64, conductivity_sm: f64, freq_hz: f64) -> f64 {
    assert!(thickness_mm >= 0.0, "thickness must be non-negative");
    assert!(conductivity_sm > 0.0, "conductivity must be positive");
    assert!(freq_hz > 0.0, "frequency must be positive");
    let mu_0: f64 = 4.0 * PI * 1e-7;
    let thickness_m = thickness_mm / 1000.0;
    let skin_depth = (1.0 / (PI * freq_hz * mu_0 * conductivity_sm)).sqrt();
    let ratio = thickness_m / skin_depth;
    8.686 * ratio
}

/// Generate a logarithmically spaced frequency sweep.
///
/// Returns `points_per_decade` frequencies per decade from `start_hz` to
/// `stop_hz` (inclusive of endpoints).
pub fn generate_frequency_sweep(
    start_hz: f64,
    stop_hz: f64,
    points_per_decade: usize,
) -> Vec<f64> {
    assert!(start_hz > 0.0, "start frequency must be positive");
    assert!(stop_hz > start_hz, "stop must be greater than start");
    assert!(points_per_decade > 0, "must have at least 1 point per decade");

    let decades = (stop_hz / start_hz).log10();
    let total_points = (decades * points_per_decade as f64).ceil() as usize + 1;
    let mut freqs = Vec::with_capacity(total_points);
    for i in 0..total_points {
        let f = start_hz * 10.0_f64.powf(i as f64 / points_per_decade as f64);
        if f > stop_hz {
            break;
        }
        freqs.push(f);
    }
    // Ensure stop frequency is included
    if let Some(&last) = freqs.last() {
        if (last - stop_hz).abs() / stop_hz > 1e-9 {
            freqs.push(stop_hz);
        }
    }
    freqs
}

/// Classify an electric field strength into the closest IEC 61000-4-3 test
/// level that is >= the given value.
///
/// If the field exceeds 30 V/m, [`TestLevel::Level4`] is returned.
pub fn classify_test_level(field_vm: f64) -> TestLevel {
    if field_vm <= 1.0 {
        TestLevel::Level1
    } else if field_vm <= 3.0 {
        TestLevel::Level2
    } else if field_vm <= 10.0 {
        TestLevel::Level3
    } else {
        TestLevel::Level4
    }
}

/// Compute susceptibility margin in dB.
///
/// `margin = 20 * log10(threshold_vm / test_level_vm)`
///
/// A positive value means the EUT can withstand more than the test level; a
/// negative value indicates the EUT is susceptible.
pub fn susceptibility_margin(threshold_vm: f64, test_level_vm: f64) -> f64 {
    assert!(threshold_vm > 0.0, "threshold must be positive");
    assert!(test_level_vm > 0.0, "test level must be positive");
    20.0 * (threshold_vm / test_level_vm).log10()
}

/// Apply AM modulation envelope to a carrier signal.
///
/// IEC 61000-4-3 specifies 80% AM at 1 kHz for standard immunity testing.
/// `mod_depth` is expressed as a fraction (0.0-1.0).
///
/// The output sample at index `i` is:
/// `carrier[i] * (1.0 + mod_depth * sin(2*pi * mod_freq * i / sample_rate))`
pub fn modulation_envelope(
    carrier: &[f64],
    mod_freq_hz: f64,
    mod_depth: f64,
    sample_rate: f64,
) -> Vec<f64> {
    assert!(sample_rate > 0.0, "sample rate must be positive");
    carrier
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let t = i as f64 / sample_rate;
            let envelope = 1.0 + mod_depth * (2.0 * PI * mod_freq_hz * t).sin();
            s * envelope
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Simulator
// ---------------------------------------------------------------------------

/// Main radiated immunity simulator.
///
/// Wraps an [`EmcConfig`] and provides convenience methods that delegate to the
/// module-level free functions using the stored configuration parameters.
#[derive(Debug, Clone)]
pub struct EmcSimulator {
    /// The active configuration.
    pub config: EmcConfig,
}

impl EmcSimulator {
    /// Create a new simulator from the given configuration.
    pub fn new(config: EmcConfig) -> Self {
        Self { config }
    }

    /// Compute the electric field at the configured distance from a source with
    /// the given EIRP (dBm).
    pub fn field_strength_from_eirp(&self, eirp_dbm: f64) -> f64 {
        field_strength_from_eirp(eirp_dbm, self.config.distance_m)
    }

    /// Compute the required EIRP (dBm) for the configured field strength and
    /// distance.
    pub fn required_eirp_dbm(&self) -> f64 {
        eirp_from_field_strength(self.config.field_strength_vm, self.config.distance_m)
    }

    /// Compute common-mode coupling voltage for a cable of given length at the
    /// configured frequency and field strength.
    pub fn cable_coupling_cm(&self, cable_length_m: f64) -> f64 {
        cable_coupling_cm(
            self.config.field_strength_vm,
            cable_length_m,
            self.config.frequency_hz,
        )
    }

    /// Compute differential-mode coupling from the common-mode voltage.
    pub fn cable_coupling_dm(&self, cable_length_m: f64, imbalance_db: f64) -> f64 {
        let v_cm = self.cable_coupling_cm(cable_length_m);
        cable_coupling_dm(v_cm, imbalance_db)
    }

    /// Compute shielding effectiveness for a barrier at the configured
    /// frequency.
    pub fn shield_effectiveness(&self, thickness_mm: f64, conductivity_sm: f64) -> f64 {
        shield_effectiveness(thickness_mm, conductivity_sm, self.config.frequency_hz)
    }

    /// Generate a logarithmic frequency sweep.
    pub fn generate_frequency_sweep(
        &self,
        start_hz: f64,
        stop_hz: f64,
        points_per_decade: usize,
    ) -> Vec<f64> {
        generate_frequency_sweep(start_hz, stop_hz, points_per_decade)
    }

    /// Classify the configured field strength into a test level.
    pub fn classify_test_level(&self) -> TestLevel {
        classify_test_level(self.config.field_strength_vm)
    }

    /// Compute the susceptibility margin for a given threshold.
    pub fn susceptibility_margin(&self, threshold_vm: f64) -> f64 {
        susceptibility_margin(threshold_vm, self.config.field_strength_vm)
    }

    /// Apply AM modulation envelope to a carrier signal using the configured
    /// frequency as the modulation frequency.
    pub fn modulation_envelope(
        &self,
        carrier: &[f64],
        mod_depth: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        modulation_envelope(carrier, self.config.frequency_hz, mod_depth, sample_rate)
    }

    /// Compute the effective field strength behind a shield.
    ///
    /// `E_out = E_in * 10^(-SE / 20)`
    pub fn shielded_field_strength(
        &self,
        thickness_mm: f64,
        conductivity_sm: f64,
    ) -> f64 {
        let se = self.shield_effectiveness(thickness_mm, conductivity_sm);
        self.config.field_strength_vm * 10.0_f64.powf(-se / 20.0)
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- field_strength_from_eirp -----------------------------------------

    #[test]
    fn test_field_strength_from_eirp_known_value() {
        // 0 dBm = 1 mW => 0.001 W, E = sqrt(30*0.001) / 1 = sqrt(0.03)
        let e = field_strength_from_eirp(0.0, 1.0);
        assert!(approx_eq(e, (0.03_f64).sqrt(), TOLERANCE));
    }

    #[test]
    fn test_field_strength_inverse_square_law() {
        let e1 = field_strength_from_eirp(30.0, 1.0);
        let e2 = field_strength_from_eirp(30.0, 2.0);
        // E is proportional to 1/d, so doubling distance halves the field.
        assert!(approx_eq(e1 / e2, 2.0, 1e-4));
    }

    #[test]
    fn test_field_strength_from_eirp_high_power() {
        // 37 dBm = 5 W, d = 3 m => E = sqrt(150)/3 ~ 4.082 V/m
        let e = field_strength_from_eirp(37.0, 3.0);
        assert!(e > 4.0 && e < 4.2);
    }

    // -- eirp_from_field_strength -----------------------------------------

    #[test]
    fn test_eirp_from_field_strength_roundtrip() {
        let eirp_in = 25.0_f64; // dBm
        let d = 3.0;
        let e = field_strength_from_eirp(eirp_in, d);
        let eirp_out = eirp_from_field_strength(e, d);
        assert!(approx_eq(eirp_in, eirp_out, 1e-6));
    }

    #[test]
    fn test_eirp_from_field_strength_known() {
        // 10 V/m at 3 m => P = (10*3)^2 / 30 = 900/30 = 30 W => 10*log10(30)+30
        let eirp = eirp_from_field_strength(10.0, 3.0);
        assert!(approx_eq(eirp, 10.0 * 30.0_f64.log10() + 30.0, 1e-4));
    }

    // -- antenna_factor_to_gain -------------------------------------------

    #[test]
    fn test_antenna_factor_to_gain_at_100mhz() {
        // At 100 MHz: G = -29.79 + 20*log10(100) - AF
        // With AF = 10 dB/m: G = -29.79 + 40 - 10 = 0.21 dBi
        let g = antenna_factor_to_gain(10.0, 100e6);
        assert!(approx_eq(g, 0.21, 0.01));
    }

    #[test]
    fn test_antenna_factor_to_gain_higher_freq() {
        // At 1 GHz: G = -29.79 + 60 - 20 = 10.21 dBi
        let g = antenna_factor_to_gain(20.0, 1e9);
        assert!(approx_eq(g, 10.21, 0.01));
    }

    // -- cable_coupling_cm ------------------------------------------------

    #[test]
    fn test_cable_coupling_cm_short_cable() {
        // At 100 MHz, wavelength = 3 m, wavelength/4 = 0.75 m.  A 0.5 m cable is electrically short.
        let v = cable_coupling_cm(10.0, 0.5, 100e6);
        assert!(approx_eq(v, 10.0 * 0.5, TOLERANCE));
    }

    #[test]
    fn test_cable_coupling_cm_long_cable_saturates() {
        // Cable longer than wavelength/4 should saturate at wavelength/4.
        let wavelength = C / 100e6;
        let quarter = wavelength / 4.0;
        let v = cable_coupling_cm(10.0, 10.0, 100e6); // 10 m >> wavelength/4
        assert!(approx_eq(v, 10.0 * quarter, 1e-4));
    }

    #[test]
    fn test_cable_coupling_cm_zero_length() {
        let v = cable_coupling_cm(10.0, 0.0, 100e6);
        assert!(approx_eq(v, 0.0, TOLERANCE));
    }

    // -- cable_coupling_dm ------------------------------------------------

    #[test]
    fn test_cable_coupling_dm_40db_imbalance() {
        // 40 dB imbalance -> factor of 1/100 = 0.01
        let v_dm = cable_coupling_dm(1.0, 40.0);
        assert!(approx_eq(v_dm, 0.01, 1e-4));
    }

    #[test]
    fn test_cable_coupling_dm_zero_imbalance() {
        // 0 dB imbalance -> V_dm = V_cm
        let v_dm = cable_coupling_dm(5.0, 0.0);
        assert!(approx_eq(v_dm, 5.0, TOLERANCE));
    }

    // -- shield_effectiveness ---------------------------------------------

    #[test]
    fn test_shield_effectiveness_copper_at_1ghz() {
        // Copper sigma ~ 5.8e7 S/m, t = 1 mm, f = 1 GHz
        let se = shield_effectiveness(1.0, 5.8e7, 1e9);
        // Skin depth at 1 GHz ~ 2.1 um, so t/delta ~ 476, SE ~ 4132 dB (very high)
        assert!(se > 100.0); // Definitely very effective
    }

    #[test]
    fn test_shield_effectiveness_zero_thickness() {
        let se = shield_effectiveness(0.0, 5.8e7, 1e9);
        assert!(approx_eq(se, 0.0, TOLERANCE));
    }

    #[test]
    fn test_shield_effectiveness_increases_with_thickness() {
        let se1 = shield_effectiveness(0.5, 5.8e7, 1e9);
        let se2 = shield_effectiveness(1.0, 5.8e7, 1e9);
        assert!(se2 > se1);
        // Should be roughly proportional to thickness
        assert!(approx_eq(se2 / se1, 2.0, 0.01));
    }

    // -- generate_frequency_sweep -----------------------------------------

    #[test]
    fn test_sweep_includes_endpoints() {
        let sweep = generate_frequency_sweep(80e6, 1e9, 10);
        assert!(approx_eq(*sweep.first().unwrap(), 80e6, 1.0));
        assert!(approx_eq(*sweep.last().unwrap(), 1e9, 1.0));
    }

    #[test]
    fn test_sweep_monotonically_increasing() {
        let sweep = generate_frequency_sweep(1e6, 1e9, 20);
        for pair in sweep.windows(2) {
            assert!(pair[1] > pair[0]);
        }
    }

    #[test]
    fn test_sweep_point_count_one_decade() {
        // Exactly 1 decade: 100 to 1000 Hz, 10 ppd -> 11 points
        let sweep = generate_frequency_sweep(100.0, 1000.0, 10);
        assert_eq!(sweep.len(), 11);
    }

    // -- classify_test_level ----------------------------------------------

    #[test]
    fn test_classify_level1() {
        assert_eq!(classify_test_level(0.5), TestLevel::Level1);
        assert_eq!(classify_test_level(1.0), TestLevel::Level1);
    }

    #[test]
    fn test_classify_level2() {
        assert_eq!(classify_test_level(1.1), TestLevel::Level2);
        assert_eq!(classify_test_level(3.0), TestLevel::Level2);
    }

    #[test]
    fn test_classify_level3() {
        assert_eq!(classify_test_level(3.1), TestLevel::Level3);
        assert_eq!(classify_test_level(10.0), TestLevel::Level3);
    }

    #[test]
    fn test_classify_level4() {
        assert_eq!(classify_test_level(10.1), TestLevel::Level4);
        assert_eq!(classify_test_level(100.0), TestLevel::Level4);
    }

    // -- susceptibility_margin --------------------------------------------

    #[test]
    fn test_susceptibility_margin_positive() {
        // Threshold 20 V/m, test level 10 V/m => margin = 20*log10(2) ~ 6.02 dB
        let m = susceptibility_margin(20.0, 10.0);
        assert!(approx_eq(m, 20.0 * 2.0_f64.log10(), 1e-4));
    }

    #[test]
    fn test_susceptibility_margin_negative() {
        // Threshold 5 V/m, test level 10 V/m => margin = 20*log10(0.5) ~ -6.02 dB
        let m = susceptibility_margin(5.0, 10.0);
        assert!(m < 0.0);
    }

    #[test]
    fn test_susceptibility_margin_zero() {
        let m = susceptibility_margin(10.0, 10.0);
        assert!(approx_eq(m, 0.0, TOLERANCE));
    }

    // -- modulation_envelope ----------------------------------------------

    #[test]
    fn test_modulation_envelope_zero_depth() {
        let carrier = vec![1.0; 100];
        let out = modulation_envelope(&carrier, 1000.0, 0.0, 48000.0);
        for (a, b) in carrier.iter().zip(out.iter()) {
            assert!(approx_eq(*a, *b, TOLERANCE));
        }
    }

    #[test]
    fn test_modulation_envelope_peak_amplitude() {
        // With mod_depth = 0.8 (80% AM), the peak envelope is 1.8x carrier
        let carrier = vec![1.0; 10000];
        let out = modulation_envelope(&carrier, 1000.0, 0.8, 100000.0);
        let max_val = out.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(approx_eq(max_val, 1.8, 0.01));
    }

    #[test]
    fn test_modulation_envelope_preserves_length() {
        let carrier = vec![0.5; 256];
        let out = modulation_envelope(&carrier, 50.0, 0.5, 1000.0);
        assert_eq!(out.len(), 256);
    }

    // -- TestLevel --------------------------------------------------------

    #[test]
    fn test_test_level_field_strengths() {
        assert!(approx_eq(TestLevel::Level1.field_strength_vm(), 1.0, TOLERANCE));
        assert!(approx_eq(TestLevel::Level2.field_strength_vm(), 3.0, TOLERANCE));
        assert!(approx_eq(TestLevel::Level3.field_strength_vm(), 10.0, TOLERANCE));
        assert!(approx_eq(TestLevel::Level4.field_strength_vm(), 30.0, TOLERANCE));
    }

    // -- EmcSimulator methods ---------------------------------------------

    #[test]
    fn test_simulator_field_from_eirp() {
        let sim = EmcSimulator::new(EmcConfig {
            frequency_hz: 100e6,
            field_strength_vm: 10.0,
            distance_m: 3.0,
            polarization: Polarization::Horizontal,
        });
        let e = sim.field_strength_from_eirp(37.0);
        assert!(e > 0.0);
    }

    #[test]
    fn test_simulator_required_eirp_roundtrip() {
        let sim = EmcSimulator::new(EmcConfig {
            frequency_hz: 200e6,
            field_strength_vm: 10.0,
            distance_m: 3.0,
            polarization: Polarization::Vertical,
        });
        let eirp = sim.required_eirp_dbm();
        let e = field_strength_from_eirp(eirp, 3.0);
        assert!(approx_eq(e, 10.0, 1e-6));
    }

    #[test]
    fn test_simulator_shielded_field_strength() {
        let sim = EmcSimulator::new(EmcConfig {
            frequency_hz: 1e9,
            field_strength_vm: 10.0,
            distance_m: 3.0,
            polarization: Polarization::Vertical,
        });
        let shielded = sim.shielded_field_strength(1.0, 5.8e7);
        assert!(shielded < sim.config.field_strength_vm);
        assert!(shielded > 0.0);
    }

    #[test]
    fn test_simulator_classify() {
        let sim = EmcSimulator::new(EmcConfig {
            frequency_hz: 100e6,
            field_strength_vm: 10.0,
            distance_m: 3.0,
            polarization: Polarization::Horizontal,
        });
        assert_eq!(sim.classify_test_level(), TestLevel::Level3);
    }

    #[test]
    fn test_simulator_cable_coupling_dm() {
        let sim = EmcSimulator::new(EmcConfig {
            frequency_hz: 100e6,
            field_strength_vm: 10.0,
            distance_m: 3.0,
            polarization: Polarization::Vertical,
        });
        let dm = sim.cable_coupling_dm(0.3, 30.0);
        let cm = sim.cable_coupling_cm(0.3);
        // DM should be less than CM with positive imbalance
        assert!(dm < cm);
        assert!(dm > 0.0);
    }
}
