//! Plasma diagnostics from Langmuir probe measurements.
//!
//! This module implements IV-curve analysis for fusion and plasma physics research,
//! including electron temperature estimation, plasma density calculation, Debye length
//! computation, plasma frequency estimation, and sheath theory calculations.
//!
//! # Overview
//!
//! A Langmuir probe is a conducting wire inserted into a plasma. By sweeping the
//! probe voltage and measuring the collected current, an IV characteristic curve is
//! obtained that encodes fundamental plasma parameters:
//!
//! - **Ion saturation region** (V << V_f): dominated by ion collection
//! - **Electron retarding region** (V_f < V < V_p): exponential electron current
//! - **Electron saturation region** (V >> V_p): full electron collection
//!
//! # Example
//!
//! ```
//! use r4w_core::plasma_impedance_analyzer::{PlasmaAnalyzer, PlasmaConfig, GasType};
//!
//! let config = PlasmaConfig {
//!     probe_area_m2: 1e-6,
//!     ion_mass_kg: GasType::Argon.ion_mass_kg(),
//!     gas: GasType::Argon,
//! };
//! let analyzer = PlasmaAnalyzer::new(config);
//!
//! // Compute Debye length for 2 eV electrons at 1e18 m^-3
//! let lambda_d = analyzer.debye_length(2.0, 1e18);
//! assert!(lambda_d > 0.0);
//! assert!(lambda_d < 1e-3); // sub-millimeter for these parameters
//! ```

use std::f64::consts::PI;

// ---------- physical constants ----------

/// Elementary charge (C).
const Q_E: f64 = 1.602_176_634e-19;

/// Electron mass (kg).
const M_E: f64 = 9.109_383_7015e-31;

/// Boltzmann constant (J/K).
#[allow(dead_code)]
const K_B: f64 = 1.380_649e-23;

/// Vacuum permittivity (F/m).
const EPS_0: f64 = 8.854_187_8128e-12;

// ---------- GasType ----------

/// Common working gases encountered in plasma experiments.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GasType {
    /// Hydrogen (H+), mass ~ 1.673e-27 kg
    Hydrogen,
    /// Helium (He+), mass ~ 6.646e-27 kg
    Helium,
    /// Argon (Ar+), mass ~ 6.634e-26 kg
    Argon,
    /// Nitrogen (N2+), mass ~ 4.652e-26 kg
    Nitrogen,
    /// Oxygen (O2+), mass ~ 5.314e-26 kg
    Oxygen,
}

impl GasType {
    /// Return the singly-ionised ion mass in kg.
    pub fn ion_mass_kg(self) -> f64 {
        match self {
            GasType::Hydrogen => 1.672_621_923_69e-27,
            GasType::Helium => 6.646_476_73e-27,
            GasType::Argon => 6.633_521_0e-26,
            GasType::Nitrogen => 4.651_792_0e-26,
            GasType::Oxygen => 5.313_730_0e-26,
        }
    }
}

// ---------- PlasmaConfig ----------

/// Configuration for a Langmuir probe measurement.
#[derive(Debug, Clone)]
pub struct PlasmaConfig {
    /// Effective probe collection area (m^2).
    pub probe_area_m2: f64,
    /// Ion mass (kg).  Convenience: use `GasType::ion_mass_kg()`.
    pub ion_mass_kg: f64,
    /// Working gas species.
    pub gas: GasType,
}

// ---------- PlasmaParams ----------

/// Extracted plasma parameters from an IV-curve analysis.
#[derive(Debug, Clone)]
pub struct PlasmaParams {
    /// Electron temperature (eV).
    pub electron_temp_ev: f64,
    /// Electron density (m^-3).
    pub electron_density_m3: f64,
    /// Plasma potential (V) -- the "knee" of the IV curve.
    pub plasma_potential_v: f64,
    /// Floating potential (V) -- voltage where net current is zero.
    pub floating_potential_v: f64,
    /// Debye length (m).
    pub debye_length_m: f64,
    /// Electron plasma frequency (Hz).
    pub plasma_freq_hz: f64,
}

// ---------- PlasmaAnalyzer ----------

/// Main analyser that wraps a [`PlasmaConfig`] and provides all diagnostic routines.
#[derive(Debug, Clone)]
pub struct PlasmaAnalyzer {
    config: PlasmaConfig,
}

impl PlasmaAnalyzer {
    /// Create a new analyser from the given configuration.
    pub fn new(config: PlasmaConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &PlasmaConfig {
        &self.config
    }

    // ------------------------------------------------------------------
    // Full IV-curve analysis
    // ------------------------------------------------------------------

    /// Perform a complete IV-curve analysis and return all derived parameters.
    ///
    /// `voltage` and `current` must be equal-length slices sorted by ascending
    /// voltage.  The probe area and ion mass are taken from the internal
    /// [`PlasmaConfig`].
    pub fn analyze_iv_curve(&self, voltage: &[f64], current: &[f64]) -> PlasmaParams {
        analyze_iv_curve(
            voltage,
            current,
            self.config.probe_area_m2,
            self.config.ion_mass_kg,
        )
    }

    // ------------------------------------------------------------------
    // Individual diagnostics (delegated to free functions)
    // ------------------------------------------------------------------

    /// Find the floating potential (V where I = 0) by linear interpolation.
    pub fn find_floating_potential(&self, voltage: &[f64], current: &[f64]) -> f64 {
        find_floating_potential(voltage, current)
    }

    /// Find the plasma potential (knee of the IV curve).
    pub fn find_plasma_potential(&self, voltage: &[f64], current: &[f64]) -> f64 {
        find_plasma_potential(voltage, current)
    }

    /// Estimate electron temperature (eV) from the slope of ln(I_e) in the
    /// electron retarding region between `vf` and `vp`.
    pub fn estimate_electron_temp(
        &self,
        voltage: &[f64],
        current: &[f64],
        vf: f64,
        vp: f64,
    ) -> f64 {
        estimate_electron_temp(voltage, current, vf, vp)
    }

    /// Compute electron density (m^-3) from the ion saturation current via
    /// the Bohm criterion.
    pub fn electron_density(
        &self,
        ion_sat_current: f64,
        probe_area: f64,
        electron_temp_ev: f64,
        ion_mass_kg: f64,
    ) -> f64 {
        electron_density(ion_sat_current, probe_area, electron_temp_ev, ion_mass_kg)
    }

    /// Compute the Debye length (m).
    pub fn debye_length(&self, electron_temp_ev: f64, electron_density_m3: f64) -> f64 {
        debye_length(electron_temp_ev, electron_density_m3)
    }

    /// Compute the electron plasma frequency (Hz).
    pub fn plasma_frequency(&self, electron_density_m3: f64) -> f64 {
        plasma_frequency(electron_density_m3)
    }

    /// Compute the ion sound (acoustic) speed (m/s).
    pub fn ion_sound_speed(&self, electron_temp_ev: f64, ion_mass_kg: f64) -> f64 {
        ion_sound_speed(electron_temp_ev, ion_mass_kg)
    }

    /// Compute the Bohm sheath-edge current (A).
    pub fn bohm_current(
        &self,
        electron_density_m3: f64,
        electron_temp_ev: f64,
        probe_area: f64,
        ion_mass_kg: f64,
    ) -> f64 {
        bohm_current(electron_density_m3, electron_temp_ev, probe_area, ion_mass_kg)
    }

    /// Apply a moving-average smoothing filter to measured currents.
    pub fn smooth_iv_curve(
        &self,
        voltage: &[f64],
        current: &[f64],
        window: usize,
    ) -> Vec<f64> {
        smooth_iv_curve(voltage, current, window)
    }
}

// ======================================================================
// Free (public) functions
// ======================================================================

/// Perform a complete IV-curve analysis.
///
/// Returns a [`PlasmaParams`] struct populated with electron temperature,
/// density, potentials, Debye length, and plasma frequency.
///
/// # Panics
///
/// Panics if `voltage` and `current` have different lengths or contain fewer
/// than 3 points.
pub fn analyze_iv_curve(
    voltage: &[f64],
    current: &[f64],
    probe_area: f64,
    ion_mass_kg: f64,
) -> PlasmaParams {
    assert_eq!(voltage.len(), current.len(), "voltage and current must have equal length");
    assert!(voltage.len() >= 3, "need at least 3 data points");

    let vf = find_floating_potential(voltage, current);
    let vp = find_plasma_potential(voltage, current);
    let te = estimate_electron_temp(voltage, current, vf, vp);

    // Ion saturation current: average of currents well below V_f.
    let i_sat = ion_saturation_current(voltage, current, vf);

    let ne = electron_density(i_sat, probe_area, te, ion_mass_kg);
    let ld = debye_length(te, ne);
    let fp = plasma_frequency(ne);

    PlasmaParams {
        electron_temp_ev: te,
        electron_density_m3: ne,
        plasma_potential_v: vp,
        floating_potential_v: vf,
        debye_length_m: ld,
        plasma_freq_hz: fp,
    }
}

/// Find the floating potential by linearly interpolating the zero-crossing of current.
///
/// Returns the voltage at which the probe current crosses zero.
pub fn find_floating_potential(voltage: &[f64], current: &[f64]) -> f64 {
    assert_eq!(voltage.len(), current.len());
    assert!(!voltage.is_empty());

    // Look for sign change in current.
    for i in 0..current.len() - 1 {
        let (i0, i1) = (current[i], current[i + 1]);
        if i0 * i1 <= 0.0 && (i0 - i1).abs() > 1e-30 {
            // Linear interpolation.
            let frac = i0.abs() / (i0.abs() + i1.abs());
            return voltage[i] + frac * (voltage[i + 1] - voltage[i]);
        }
    }

    // Fallback: return the voltage of the smallest |I|.
    let (mut best_idx, mut best_abs) = (0, current[0].abs());
    for (idx, &c) in current.iter().enumerate() {
        if c.abs() < best_abs {
            best_abs = c.abs();
            best_idx = idx;
        }
    }
    voltage[best_idx]
}

/// Find the plasma potential as the voltage of maximum dI/dV (the "knee").
///
/// Uses a simple finite-difference first derivative of the IV curve.
pub fn find_plasma_potential(voltage: &[f64], current: &[f64]) -> f64 {
    assert_eq!(voltage.len(), current.len());
    assert!(voltage.len() >= 2);

    let mut max_deriv = f64::NEG_INFINITY;
    let mut vp_idx = 0usize;

    for i in 0..voltage.len() - 1 {
        let dv = voltage[i + 1] - voltage[i];
        if dv.abs() < 1e-30 {
            continue;
        }
        let di_dv = (current[i + 1] - current[i]) / dv;
        if di_dv > max_deriv {
            max_deriv = di_dv;
            vp_idx = i;
        }
    }

    // Interpolate to mid-point of the segment with steepest slope.
    (voltage[vp_idx] + voltage[(vp_idx + 1).min(voltage.len() - 1)]) / 2.0
}

/// Estimate electron temperature (eV) from the slope of ln(I_e) vs V in the
/// electron retarding region `[vf, vp]`.
///
/// Uses a least-squares linear fit of ln(I_e) against V; Te = 1/slope (eV).
pub fn estimate_electron_temp(
    voltage: &[f64],
    current: &[f64],
    vf: f64,
    vp: f64,
) -> f64 {
    assert_eq!(voltage.len(), current.len());

    // Estimate ion saturation current to subtract it from total current.
    let i_ion = ion_saturation_current(voltage, current, vf);

    // Collect (V, ln(I_e)) pairs in the retarding region.
    let mut sum_v = 0.0_f64;
    let mut sum_ln = 0.0_f64;
    let mut sum_v2 = 0.0_f64;
    let mut sum_v_ln = 0.0_f64;
    let mut n = 0u64;

    for (idx, &v) in voltage.iter().enumerate() {
        if v >= vf && v <= vp {
            let ie = current[idx] - i_ion;
            if ie > 0.0 {
                let ln_ie = ie.ln();
                sum_v += v;
                sum_ln += ln_ie;
                sum_v2 += v * v;
                sum_v_ln += v * ln_ie;
                n += 1;
            }
        }
    }

    if n < 2 {
        // Fallback: rough estimate from kT ~ (Vp - Vf) / ln(2)
        return ((vp - vf).abs() / 2.0_f64.ln()).max(0.1);
    }

    let nf = n as f64;
    let slope = (nf * sum_v_ln - sum_v * sum_ln) / (nf * sum_v2 - sum_v * sum_v);

    if slope > 1e-10 {
        1.0 / slope // Te in eV (since V is in Volts, slope is 1/Te)
    } else {
        // Fallback for pathological data.
        ((vp - vf).abs() / 2.0_f64.ln()).max(0.1)
    }
}

/// Compute electron density (m^-3) from the ion saturation current using the
/// Bohm criterion:
///
/// ```text
/// I_sat = n_e * q_e * A * sqrt(kTe / m_i)
/// => n_e = I_sat / (q_e * A * sqrt(kTe / m_i))
/// ```
///
/// Here `electron_temp_ev` is in eV and converted internally to Joules.
pub fn electron_density(
    ion_sat_current: f64,
    probe_area: f64,
    electron_temp_ev: f64,
    ion_mass_kg: f64,
) -> f64 {
    let te_j = electron_temp_ev * Q_E; // eV -> J
    let cs = (te_j / ion_mass_kg).sqrt();
    let denom = Q_E * probe_area * cs;
    if denom.abs() < 1e-60 {
        return 0.0;
    }
    (ion_sat_current.abs() / denom).abs()
}

/// Compute the Debye length (m):
///
/// ```text
/// lambda_D = sqrt(eps_0 * kT_e / (n_e * q_e^2))
/// ```
pub fn debye_length(electron_temp_ev: f64, electron_density_m3: f64) -> f64 {
    if electron_density_m3 <= 0.0 {
        return 0.0;
    }
    let te_j = electron_temp_ev * Q_E;
    (EPS_0 * te_j / (electron_density_m3 * Q_E * Q_E)).sqrt()
}

/// Compute the electron plasma frequency (Hz):
///
/// ```text
/// f_pe = (1 / 2*pi) * sqrt(n_e * q_e^2 / (eps_0 * m_e))
/// ```
pub fn plasma_frequency(electron_density_m3: f64) -> f64 {
    if electron_density_m3 <= 0.0 {
        return 0.0;
    }
    (1.0 / (2.0 * PI)) * (electron_density_m3 * Q_E * Q_E / (EPS_0 * M_E)).sqrt()
}

/// Compute the ion sound (acoustic) speed (m/s):
///
/// ```text
/// c_s = sqrt(k T_e / m_i)
/// ```
///
/// Assumes cold ions (T_i ~ 0) and singly-charged ions.
pub fn ion_sound_speed(electron_temp_ev: f64, ion_mass_kg: f64) -> f64 {
    let te_j = electron_temp_ev * Q_E;
    (te_j / ion_mass_kg).sqrt()
}

/// Compute the Bohm sheath-edge current (A):
///
/// ```text
/// I_Bohm = n_e * q_e * A * c_s = n_e * q_e * A * sqrt(kTe / m_i)
/// ```
pub fn bohm_current(
    electron_density_m3: f64,
    electron_temp_ev: f64,
    probe_area: f64,
    ion_mass_kg: f64,
) -> f64 {
    let cs = ion_sound_speed(electron_temp_ev, ion_mass_kg);
    electron_density_m3 * Q_E * probe_area * cs
}

/// Smooth the measured current array using a centred moving-average of the
/// given `window` size.
///
/// The voltage array is accepted for API symmetry but is not modified.
/// Returns the smoothed current vector (same length as input).
pub fn smooth_iv_curve(
    _voltage: &[f64],
    current: &[f64],
    window: usize,
) -> Vec<f64> {
    let n = current.len();
    if n == 0 || window == 0 {
        return current.to_vec();
    }
    let w = window.max(1);
    let half = (w / 2) as i64;

    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = (i as i64 - half).max(0) as usize;
        let hi = (i + (w - w / 2)).min(n);
        let count = (hi - lo) as f64;
        let sum: f64 = current[lo..hi].iter().sum();
        out.push(sum / count);
    }
    out
}

// ======================================================================
// Internal helpers
// ======================================================================

/// Estimate the ion saturation current by averaging currents at voltages well
/// below the floating potential.
fn ion_saturation_current(voltage: &[f64], current: &[f64], vf: f64) -> f64 {
    let mut sum = 0.0_f64;
    let mut count = 0u64;

    for (idx, &v) in voltage.iter().enumerate() {
        if v < vf {
            sum += current[idx];
            count += 1;
        }
    }

    if count == 0 {
        // Fallback: use the most negative current value.
        current.iter().cloned().fold(f64::INFINITY, f64::min)
    } else {
        sum / count as f64
    }
}

// ======================================================================
// Tests
// ======================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a synthetic Langmuir-probe IV curve.
    ///
    /// Parameters:
    ///   te    - electron temperature (eV)
    ///   ne    - electron density (m^-3)
    ///   vp    - plasma potential (V)
    ///   mi    - ion mass (kg)
    ///   area  - probe area (m^2)
    ///   n_pts - number of sample points
    ///   v_min, v_max - sweep range (V)
    ///
    /// Returns (voltage, current) vectors.
    fn synthetic_iv(
        te: f64,
        ne: f64,
        vp: f64,
        mi: f64,
        area: f64,
        n_pts: usize,
        v_min: f64,
        v_max: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let cs = (te * Q_E / mi).sqrt();
        let i_sat = -ne * Q_E * area * cs; // ion saturation (negative)
        let ie_sat = ne * Q_E * area * (te * Q_E / (2.0 * PI * M_E)).sqrt() * 0.25;

        let mut voltage = Vec::with_capacity(n_pts);
        let mut current = Vec::with_capacity(n_pts);

        for k in 0..n_pts {
            let v = v_min + (v_max - v_min) * (k as f64) / ((n_pts - 1) as f64);
            let ie = if v < vp {
                ie_sat * ((v - vp) / te).exp()
            } else {
                ie_sat
            };
            let ii = i_sat;
            voltage.push(v);
            current.push(ie + ii);
        }
        (voltage, current)
    }

    const TE: f64 = 3.0; // eV
    const NE: f64 = 1e17; // m^-3
    const VP: f64 = 10.0; // V
    const AREA: f64 = 1e-6; // m^2

    fn argon_mass() -> f64 {
        GasType::Argon.ion_mass_kg()
    }

    // ---------- test: GasType masses ----------

    #[test]
    fn test_gas_type_hydrogen_mass() {
        let m = GasType::Hydrogen.ion_mass_kg();
        assert!(m > 1.67e-27 && m < 1.68e-27, "H mass out of range: {m}");
    }

    #[test]
    fn test_gas_type_argon_mass() {
        let m = GasType::Argon.ion_mass_kg();
        assert!(m > 6.6e-26 && m < 6.7e-26, "Ar mass out of range: {m}");
    }

    #[test]
    fn test_gas_type_all_positive() {
        for g in [GasType::Hydrogen, GasType::Helium, GasType::Argon, GasType::Nitrogen, GasType::Oxygen] {
            assert!(g.ion_mass_kg() > 0.0);
        }
    }

    // ---------- test: debye_length ----------

    #[test]
    fn test_debye_length_positive() {
        let ld = debye_length(2.0, 1e18);
        assert!(ld > 0.0, "Debye length must be positive");
    }

    #[test]
    fn test_debye_length_known_value() {
        // lambda_D = sqrt(eps0 * kTe / (ne * qe^2))
        // For Te=1 eV, ne=1e18 m^-3: lambda_D ~ 7.434e-6 m
        let ld = debye_length(1.0, 1e18);
        let expected = 7.434e-6;
        let rel_err = (ld - expected).abs() / expected;
        assert!(rel_err < 0.02, "Debye length {ld} not near expected {expected}, rel err={rel_err}");
    }

    #[test]
    fn test_debye_length_zero_density() {
        assert_eq!(debye_length(1.0, 0.0), 0.0);
    }

    // ---------- test: plasma_frequency ----------

    #[test]
    fn test_plasma_frequency_positive() {
        let fp = plasma_frequency(1e18);
        assert!(fp > 0.0);
    }

    #[test]
    fn test_plasma_frequency_known_value() {
        // f_pe ~ 9 * sqrt(ne) Hz where ne is in m^-3
        // For ne = 1e18: f_pe ~ 9e9 Hz
        let fp = plasma_frequency(1e18);
        let expected = 9.0e9;
        let rel_err = (fp - expected).abs() / expected;
        assert!(rel_err < 0.01, "Plasma freq {fp} not near {expected}, rel err={rel_err}");
    }

    #[test]
    fn test_plasma_frequency_zero_density() {
        assert_eq!(plasma_frequency(0.0), 0.0);
    }

    // ---------- test: ion_sound_speed ----------

    #[test]
    fn test_ion_sound_speed_positive() {
        let cs = ion_sound_speed(2.0, argon_mass());
        assert!(cs > 0.0);
    }

    #[test]
    fn test_ion_sound_speed_argon() {
        // For Ar, Te=2 eV: cs ~ sqrt(2*1.6e-19 / 6.63e-26) ~ 2197 m/s
        let cs = ion_sound_speed(2.0, argon_mass());
        assert!(cs > 2000.0 && cs < 2500.0, "cs = {cs}");
    }

    // ---------- test: bohm_current ----------

    #[test]
    fn test_bohm_current_positive() {
        let ib = bohm_current(1e18, 2.0, 1e-6, argon_mass());
        assert!(ib > 0.0);
    }

    #[test]
    fn test_bohm_current_proportional_to_area() {
        let ib1 = bohm_current(1e18, 2.0, 1e-6, argon_mass());
        let ib2 = bohm_current(1e18, 2.0, 2e-6, argon_mass());
        let ratio = ib2 / ib1;
        assert!((ratio - 2.0).abs() < 1e-10, "Current should scale linearly with area");
    }

    // ---------- test: electron_density ----------

    #[test]
    fn test_electron_density_roundtrip() {
        // Compute Bohm current, then recover density.
        let ne_in = 1e17;
        let te = 3.0;
        let area = 1e-6;
        let mi = argon_mass();
        let ib = bohm_current(ne_in, te, area, mi);
        let ne_out = electron_density(ib, area, te, mi);
        let rel_err = (ne_out - ne_in).abs() / ne_in;
        assert!(rel_err < 1e-10, "Density roundtrip failed: {ne_out} vs {ne_in}");
    }

    // ---------- test: find_floating_potential ----------

    #[test]
    fn test_find_floating_potential_synthetic() {
        let (v, i) = synthetic_iv(TE, NE, VP, argon_mass(), AREA, 1000, -30.0, 20.0);
        let vf = find_floating_potential(&v, &i);
        // Floating potential should be below plasma potential.
        assert!(vf < VP, "V_f={vf} should be < V_p={VP}");
        assert!(vf > -30.0, "V_f should be within sweep range");
    }

    // ---------- test: find_plasma_potential ----------

    #[test]
    fn test_find_plasma_potential_synthetic() {
        let (v, i) = synthetic_iv(TE, NE, VP, argon_mass(), AREA, 1000, -30.0, 20.0);
        let vp_est = find_plasma_potential(&v, &i);
        let err = (vp_est - VP).abs();
        assert!(err < 1.0, "V_p estimate {vp_est} too far from true {VP}, err={err}");
    }

    // ---------- test: estimate_electron_temp ----------

    #[test]
    fn test_estimate_electron_temp_synthetic() {
        let (v, i) = synthetic_iv(TE, NE, VP, argon_mass(), AREA, 2000, -30.0, 20.0);
        let vf = find_floating_potential(&v, &i);
        let vp = find_plasma_potential(&v, &i);
        let te_est = estimate_electron_temp(&v, &i, vf, vp);
        let rel_err = (te_est - TE).abs() / TE;
        assert!(rel_err < 0.20, "Te estimate {te_est} too far from {TE}, rel_err={rel_err}");
    }

    // ---------- test: full analyze_iv_curve ----------

    #[test]
    fn test_analyze_iv_curve_synthetic() {
        let mi = argon_mass();
        let (v, i) = synthetic_iv(TE, NE, VP, mi, AREA, 2000, -30.0, 20.0);
        let params = analyze_iv_curve(&v, &i, AREA, mi);

        // Floating potential below plasma potential.
        assert!(params.floating_potential_v < params.plasma_potential_v);
        // Electron temp within 50% of expected.
        let te_err = (params.electron_temp_ev - TE).abs() / TE;
        assert!(te_err < 0.5, "Te err {te_err}");
        // Debye length positive.
        assert!(params.debye_length_m > 0.0);
        // Plasma frequency positive.
        assert!(params.plasma_freq_hz > 0.0);
    }

    // ---------- test: smooth_iv_curve ----------

    #[test]
    fn test_smooth_iv_curve_preserves_length() {
        let v: Vec<f64> = (0..100).map(|x| x as f64 * 0.1).collect();
        let i: Vec<f64> = v.iter().map(|x| x.sin()).collect();
        let smoothed = smooth_iv_curve(&v, &i, 5);
        assert_eq!(smoothed.len(), i.len());
    }

    #[test]
    fn test_smooth_iv_curve_window_one_identity() {
        let v = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let i = vec![1.0, 3.0, 5.0, 7.0, 9.0];
        let smoothed = smooth_iv_curve(&v, &i, 1);
        for (a, b) in smoothed.iter().zip(i.iter()) {
            assert!((a - b).abs() < 1e-14);
        }
    }

    #[test]
    fn test_smooth_iv_curve_reduces_noise() {
        // Constant signal with noise should become less noisy after smoothing.
        let n = 200;
        let v: Vec<f64> = (0..n).map(|x| x as f64).collect();
        let i: Vec<f64> = (0..n)
            .map(|x| 5.0 + if x % 2 == 0 { 0.5 } else { -0.5 })
            .collect();
        let smoothed = smooth_iv_curve(&v, &i, 7);

        let var_original: f64 = i.iter().map(|x| (x - 5.0).powi(2)).sum::<f64>() / n as f64;
        let var_smoothed: f64 = smoothed.iter().map(|x| (x - 5.0).powi(2)).sum::<f64>() / n as f64;
        assert!(
            var_smoothed < var_original,
            "Smoothing should reduce variance: {var_smoothed} >= {var_original}"
        );
    }

    // ---------- test: PlasmaAnalyzer struct ----------

    #[test]
    fn test_plasma_analyzer_config_roundtrip() {
        let cfg = PlasmaConfig {
            probe_area_m2: 1e-6,
            ion_mass_kg: argon_mass(),
            gas: GasType::Argon,
        };
        let analyzer = PlasmaAnalyzer::new(cfg.clone());
        assert_eq!(analyzer.config().probe_area_m2, 1e-6);
        assert_eq!(analyzer.config().gas, GasType::Argon);
    }

    #[test]
    fn test_plasma_analyzer_debye_matches_free_fn() {
        let cfg = PlasmaConfig {
            probe_area_m2: 1e-6,
            ion_mass_kg: argon_mass(),
            gas: GasType::Argon,
        };
        let analyzer = PlasmaAnalyzer::new(cfg);
        let ld_method = analyzer.debye_length(2.0, 1e18);
        let ld_free = debye_length(2.0, 1e18);
        assert!((ld_method - ld_free).abs() < 1e-30);
    }

    #[test]
    fn test_plasma_analyzer_full_analysis() {
        let mi = argon_mass();
        let cfg = PlasmaConfig {
            probe_area_m2: AREA,
            ion_mass_kg: mi,
            gas: GasType::Argon,
        };
        let analyzer = PlasmaAnalyzer::new(cfg);
        let (v, i) = synthetic_iv(TE, NE, VP, mi, AREA, 2000, -30.0, 20.0);
        let params = analyzer.analyze_iv_curve(&v, &i);
        assert!(params.electron_temp_ev > 0.0);
        assert!(params.electron_density_m3 > 0.0);
        assert!(params.debye_length_m > 0.0);
        assert!(params.plasma_freq_hz > 0.0);
    }

    // ---------- edge cases ----------

    #[test]
    fn test_smooth_empty() {
        let smoothed = smooth_iv_curve(&[], &[], 5);
        assert!(smoothed.is_empty());
    }
}
