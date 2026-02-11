//! Plasma diagnostics signal processing for fusion research and industrial plasma applications.
//!
//! Implements Langmuir probe I-V curve analysis, Thomson scattering parameters,
//! reflectometry cutoff densities, and fundamental plasma physics calculations
//! for measuring electron temperature, density, and plasma frequency.
//!
//! All physics constants and math are implemented from scratch with no external
//! crate dependencies beyond `std`.
//!
//! # Example
//!
//! ```
//! use r4w_core::plasma_diagnostics_processor::{
//!     PlasmaConfig, PlasmaProcessor, plasma_frequency_hz, debye_length_m,
//! };
//!
//! let config = PlasmaConfig::default();
//! let processor = PlasmaProcessor::new(config);
//!
//! // Plasma frequency for a typical tokamak edge density
//! let ne = 1e18; // 1e18 m^-3
//! let f_pe = plasma_frequency_hz(ne);
//! assert!(f_pe > 8e9); // ~9 GHz
//!
//! // Debye length at 10 eV, 1e18 m^-3
//! let lambda_d = debye_length_m(10.0, ne);
//! assert!(lambda_d > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants (SI units)
// ---------------------------------------------------------------------------

/// Elementary charge (C)
pub const ELECTRON_CHARGE: f64 = 1.602_176_634e-19;

/// Electron mass (kg)
pub const ELECTRON_MASS: f64 = 9.109_383_7015e-31;

/// Boltzmann constant (J/K)
pub const BOLTZMANN_K: f64 = 1.380_649e-23;

/// Vacuum permittivity (F/m)
pub const EPSILON_0: f64 = 8.854_187_8128e-12;

/// Vacuum permeability (H/m) -- mu_0 = 4*pi*1e-7
pub const MU_0: f64 = 1.256_637_062_12e-6;

/// Atomic mass unit (kg)
pub const AMU: f64 = 1.660_539_066_60e-27;

/// Bremsstrahlung constant C_brem for P = C * n_e^2 * Z_eff * sqrt(T_e)
/// where T_e in eV, n_e in m^-3, result in W/m^3.
/// C_brem ~ 5.35e-37 W m^3 eV^{-1/2} (commonly used approximation)
pub const BREMSSTRAHLUNG_C: f64 = 5.35e-37;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for plasma diagnostics processing.
#[derive(Debug, Clone)]
pub struct PlasmaConfig {
    /// Langmuir probe collecting area (m^2).
    pub probe_area_m2: f64,
    /// Ion mass in atomic mass units (default 2.0 for deuterium).
    pub ion_mass_amu: f64,
    /// Background magnetic field strength (Tesla).
    pub magnetic_field_t: f64,
    /// Microwave frequency for reflectometry (GHz).
    pub microwave_freq_ghz: f64,
}

impl Default for PlasmaConfig {
    fn default() -> Self {
        Self {
            probe_area_m2: 1e-6,        // 1 mm^2
            ion_mass_amu: 2.0,           // Deuterium
            magnetic_field_t: 2.0,       // Typical tokamak on-axis
            microwave_freq_ghz: 60.0,    // V-band reflectometry
        }
    }
}

impl PlasmaConfig {
    /// Return ion mass in kg.
    pub fn ion_mass_kg(&self) -> f64 {
        self.ion_mass_amu * AMU
    }
}

// ---------------------------------------------------------------------------
// Output parameters
// ---------------------------------------------------------------------------

/// Results from Langmuir probe I-V curve analysis.
#[derive(Debug, Clone)]
pub struct PlasmaParameters {
    /// Electron temperature (eV).
    pub electron_temp_ev: f64,
    /// Electron density (m^-3).
    pub electron_density_m3: f64,
    /// Plasma (space) potential (V).
    pub plasma_potential_v: f64,
    /// Floating potential (V).
    pub floating_potential_v: f64,
    /// Debye length (m).
    pub debye_length_m: f64,
    /// Ion saturation current (A).
    pub ion_saturation_current_a: f64,
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// Plasma diagnostics processor.
///
/// Wraps a [`PlasmaConfig`] and provides high-level methods for
/// analysing Langmuir probe I-V characteristics.
pub struct PlasmaProcessor {
    config: PlasmaConfig,
}

impl PlasmaProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: PlasmaConfig) -> Self {
        Self { config }
    }

    /// Analyse a Langmuir probe I-V curve.
    ///
    /// `voltage` and `current` are parallel slices of the swept probe
    /// voltage (V) and measured probe current (A).  They must have the
    /// same length and at least 5 points.
    ///
    /// The algorithm:
    /// 1. Identify the floating potential V_f where the current crosses zero.
    /// 2. Estimate the ion saturation current I_sat as the mean current in the
    ///    most negative voltage region (lowest 20% of voltage range).
    /// 3. Estimate the plasma potential V_p as the voltage of maximum dI/dV.
    /// 4. Fit the electron retardation region (V_f < V < V_p) in the ln(I - I_sat)
    ///    vs V plane to extract electron temperature T_e.
    /// 5. Compute electron density from the Bohm criterion using I_sat.
    /// 6. Compute Debye length from T_e and n_e.
    pub fn analyze_iv_curve(
        &self,
        voltage: &[f64],
        current: &[f64],
    ) -> PlasmaParameters {
        assert_eq!(voltage.len(), current.len(), "voltage and current must have same length");
        assert!(voltage.len() >= 5, "need at least 5 data points");

        // Sort by voltage (create index permutation)
        let n = voltage.len();
        let mut idx: Vec<usize> = (0..n).collect();
        idx.sort_by(|&a, &b| voltage[a].partial_cmp(&voltage[b]).unwrap());

        let v_sorted: Vec<f64> = idx.iter().map(|&i| voltage[i]).collect();
        let i_sorted: Vec<f64> = idx.iter().map(|&i| current[i]).collect();

        // --- Floating potential: linear interpolation of zero crossing ---
        let v_float = find_zero_crossing(&v_sorted, &i_sorted);

        // --- Ion saturation current: mean of bottom 20% voltage region ---
        let n_sat = (n as f64 * 0.2).ceil() as usize;
        let n_sat = n_sat.max(1);
        let i_sat: f64 = i_sorted[..n_sat].iter().sum::<f64>() / n_sat as f64;

        // --- Plasma potential: voltage of maximum dI/dV ---
        let v_plasma = find_plasma_potential(&v_sorted, &i_sorted);

        // --- Electron temperature from retardation region ---
        // Build ln(I - I_sat) for points between V_float and V_plasma
        let mut v_ret = Vec::new();
        let mut ln_i_ret = Vec::new();
        for k in 0..n {
            if v_sorted[k] >= v_float && v_sorted[k] <= v_plasma {
                let i_electron = i_sorted[k] - i_sat;
                if i_electron > 0.0 {
                    v_ret.push(v_sorted[k]);
                    ln_i_ret.push(i_electron.ln());
                }
            }
        }

        let te_ev = if v_ret.len() >= 2 {
            fit_electron_temperature(&v_ret, &ln_i_ret, v_float, v_plasma)
        } else {
            // Fallback: use rough V_p - V_f relationship  T_e ~ (V_p - V_f) / ln(2)
            ((v_plasma - v_float) / (2.0_f64).ln()).abs().max(0.1)
        };

        // --- Electron density ---
        let ne = electron_density_from_saturation(
            i_sat.abs(),
            te_ev,
            self.config.probe_area_m2,
            self.config.ion_mass_kg(),
        );

        let lambda_d = debye_length_m(te_ev, ne);

        PlasmaParameters {
            electron_temp_ev: te_ev,
            electron_density_m3: ne,
            plasma_potential_v: v_plasma,
            floating_potential_v: v_float,
            debye_length_m: lambda_d,
            ion_saturation_current_a: i_sat,
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions -- plasma physics formulae
// ---------------------------------------------------------------------------

/// Fit electron temperature from the slope of ln(I_e) vs V in the electron
/// retardation region.
///
/// Uses ordinary least-squares linear regression on the provided `(voltage,
/// ln_current)` data restricted between `v_float` and `v_plasma`.  Returns
/// `T_e` in eV (= 1/slope).
pub fn fit_electron_temperature(
    voltage: &[f64],
    current_ln: &[f64],
    _v_float: f64,
    _v_plasma: f64,
) -> f64 {
    // Simple linear regression: ln(I) = a + b*V  =>  T_e = 1/b
    let n = voltage.len().min(current_ln.len());
    if n < 2 {
        return 1.0; // fallback
    }

    let sum_v: f64 = voltage[..n].iter().sum();
    let sum_y: f64 = current_ln[..n].iter().sum();
    let sum_vv: f64 = voltage[..n].iter().map(|v| v * v).sum();
    let sum_vy: f64 = voltage[..n]
        .iter()
        .zip(current_ln[..n].iter())
        .map(|(v, y)| v * y)
        .sum();

    let nf = n as f64;
    let denom = nf * sum_vv - sum_v * sum_v;
    if denom.abs() < 1e-30 {
        return 1.0;
    }

    let slope = (nf * sum_vy - sum_v * sum_y) / denom;

    if slope <= 0.0 || !slope.is_finite() {
        return 1.0;
    }

    1.0 / slope
}

/// Compute electron density from the ion saturation current using the Bohm
/// criterion.
///
/// n_e = I_sat / (0.6 * e * A * sqrt(k * T_e / m_i))
///
/// where `te_ev` is in eV so k*T_e = te_ev * e (Joules).
pub fn electron_density_from_saturation(
    i_sat: f64,
    te_ev: f64,
    probe_area: f64,
    ion_mass_kg: f64,
) -> f64 {
    if probe_area <= 0.0 || ion_mass_kg <= 0.0 || te_ev <= 0.0 {
        return 0.0;
    }

    let kt_e = te_ev * ELECTRON_CHARGE; // Joules
    let v_bohm = (kt_e / ion_mass_kg).sqrt();
    let denom = 0.6 * ELECTRON_CHARGE * probe_area * v_bohm;

    if denom <= 0.0 {
        return 0.0;
    }

    i_sat / denom
}

/// Electron plasma frequency (Hz).
///
/// f_pe = (1 / 2*pi) * sqrt(n_e * e^2 / (m_e * epsilon_0))
///
/// Commonly approximated as f_pe ~ 9 * sqrt(n_e) Hz.
pub fn plasma_frequency_hz(density_m3: f64) -> f64 {
    if density_m3 <= 0.0 {
        return 0.0;
    }
    // Exact formula
    let omega_pe = (density_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE
        / (ELECTRON_MASS * EPSILON_0))
        .sqrt();
    omega_pe / (2.0 * PI)
}

/// Debye length (m).
///
/// lambda_D = sqrt(epsilon_0 * k * T_e / (n_e * e^2))
///
/// where `te_ev` is in eV so k*T_e = te_ev * e.
pub fn debye_length_m(te_ev: f64, ne_m3: f64) -> f64 {
    if te_ev <= 0.0 || ne_m3 <= 0.0 {
        return 0.0;
    }

    let kt_e = te_ev * ELECTRON_CHARGE;
    (EPSILON_0 * kt_e / (ne_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE)).sqrt()
}

/// Cyclotron (gyro) frequency (Hz).
///
/// f_c = q * B / (2*pi*m)
pub fn cyclotron_frequency_hz(charge: f64, mass_kg: f64, b_tesla: f64) -> f64 {
    if mass_kg <= 0.0 {
        return 0.0;
    }
    (charge.abs() * b_tesla.abs()) / (2.0 * PI * mass_kg)
}

/// Larmor (gyro) radius (m).
///
/// r_L = v_thermal / omega_c
///
/// where v_thermal = sqrt(k * T_e / m) and omega_c = qB/m.
pub fn larmor_radius_m(te_ev: f64, mass_kg: f64, b_tesla: f64) -> f64 {
    if mass_kg <= 0.0 || b_tesla.abs() < 1e-30 || te_ev <= 0.0 {
        return 0.0;
    }

    let kt = te_ev * ELECTRON_CHARGE;
    let v_th = (kt / mass_kg).sqrt();
    let omega_c = ELECTRON_CHARGE * b_tesla.abs() / mass_kg;

    v_th / omega_c
}

/// O-mode reflectometry cutoff density (m^-3).
///
/// The ordinary-mode (O-mode) electromagnetic wave is reflected when the
/// plasma frequency equals the probing frequency:
///
///   f_pe = f_probe  =>  n_e = (2*pi*f)^2 * m_e * epsilon_0 / e^2
pub fn reflectometry_cutoff_density(freq_ghz: f64) -> f64 {
    if freq_ghz <= 0.0 {
        return 0.0;
    }

    let f_hz = freq_ghz * 1e9;
    let omega = 2.0 * PI * f_hz;
    omega * omega * ELECTRON_MASS * EPSILON_0
        / (ELECTRON_CHARGE * ELECTRON_CHARGE)
}

/// Bremsstrahlung radiated power density (W/m^3).
///
/// P_brem = C_brem * n_e^2 * Z_eff * sqrt(T_e)
///
/// where T_e is in eV, n_e in m^-3, Z_eff is the effective ion charge.
pub fn bremsstrahlung_power_density(ne_m3: f64, te_ev: f64, z_eff: f64) -> f64 {
    if ne_m3 <= 0.0 || te_ev <= 0.0 || z_eff <= 0.0 {
        return 0.0;
    }

    BREMSSTRAHLUNG_C * ne_m3 * ne_m3 * z_eff * te_ev.sqrt()
}

/// Plasma beta -- ratio of kinetic pressure to magnetic pressure.
///
/// beta = 2 * mu_0 * p / B^2
pub fn plasma_beta(pressure_pa: f64, b_tesla: f64) -> f64 {
    if b_tesla.abs() < 1e-30 {
        return f64::INFINITY;
    }

    2.0 * MU_0 * pressure_pa / (b_tesla * b_tesla)
}

/// Alfven velocity (m/s).
///
/// v_A = B / sqrt(mu_0 * n * m_i)
pub fn alfven_velocity(b_tesla: f64, density_m3: f64, ion_mass_kg: f64) -> f64 {
    if density_m3 <= 0.0 || ion_mass_kg <= 0.0 {
        return 0.0;
    }

    let denom = (MU_0 * density_m3 * ion_mass_kg).sqrt();
    if denom < 1e-30 {
        return 0.0;
    }

    b_tesla.abs() / denom
}

/// Thermal velocity (m/s) for a particle of given mass and temperature (eV).
///
/// v_th = sqrt(k*T / m)
pub fn thermal_velocity(te_ev: f64, mass_kg: f64) -> f64 {
    if te_ev <= 0.0 || mass_kg <= 0.0 {
        return 0.0;
    }
    (te_ev * ELECTRON_CHARGE / mass_kg).sqrt()
}

/// Coulomb logarithm for electron-ion collisions.
///
/// ln(Lambda) ~ 23 - ln(sqrt(n_e) * T_e^{-3/2})   (for T_e > 10 eV regime)
///
/// A simplified approximation suitable for fusion-relevant plasmas.
pub fn coulomb_logarithm(ne_m3: f64, te_ev: f64) -> f64 {
    if ne_m3 <= 0.0 || te_ev <= 0.0 {
        return 10.0; // reasonable default
    }

    let val = 23.0 - (ne_m3.sqrt() * te_ev.powf(-1.5)).ln();
    val.max(1.0) // Coulomb log is always > 0 in practice
}

/// Spitzer resistivity (Ohm-m).
///
/// eta = (pi * Z * e^2 * m_e^{1/2} * ln_Lambda) / ((4*pi*epsilon_0)^2 * (k*T_e)^{3/2})
///
/// Simplified: eta ~ 5.2e-5 * Z_eff * ln_Lambda / T_e^{3/2}  (Ohm-m, T_e in eV)
pub fn spitzer_resistivity(te_ev: f64, z_eff: f64, ln_lambda: f64) -> f64 {
    if te_ev <= 0.0 {
        return f64::INFINITY;
    }

    5.2e-5 * z_eff * ln_lambda / te_ev.powf(1.5)
}

/// Skin depth (m) for electromagnetic wave penetration into a plasma.
///
/// delta = c / omega_pe
pub fn skin_depth_m(density_m3: f64) -> f64 {
    if density_m3 <= 0.0 {
        return f64::INFINITY;
    }

    let omega_pe = (density_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE
        / (ELECTRON_MASS * EPSILON_0))
        .sqrt();

    if omega_pe < 1e-30 {
        return f64::INFINITY;
    }

    // Speed of light
    let c = 2.997_924_58e8;
    c / omega_pe
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Find the voltage at which the current crosses zero via linear
/// interpolation. If no crossing is found, return the midpoint voltage.
fn find_zero_crossing(voltage: &[f64], current: &[f64]) -> f64 {
    for k in 0..voltage.len() - 1 {
        if current[k] * current[k + 1] <= 0.0 {
            // Linear interpolation
            let di = current[k + 1] - current[k];
            if di.abs() < 1e-30 {
                return (voltage[k] + voltage[k + 1]) * 0.5;
            }
            let t = -current[k] / di;
            return voltage[k] + t * (voltage[k + 1] - voltage[k]);
        }
    }
    // No crossing found -- return midpoint
    (voltage[0] + voltage[voltage.len() - 1]) * 0.5
}

/// Find the plasma potential as the voltage of maximum dI/dV.
fn find_plasma_potential(voltage: &[f64], current: &[f64]) -> f64 {
    let n = voltage.len();
    if n < 3 {
        return voltage[n - 1];
    }

    let mut max_didv = f64::NEG_INFINITY;
    let mut v_plasma = voltage[n - 1];

    for k in 1..n {
        let dv = voltage[k] - voltage[k - 1];
        if dv.abs() < 1e-30 {
            continue;
        }
        let didv = (current[k] - current[k - 1]) / dv;
        if didv > max_didv {
            max_didv = didv;
            v_plasma = (voltage[k] + voltage[k - 1]) * 0.5;
        }
    }

    v_plasma
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if a == b {
            return true;
        }
        let diff = (a - b).abs();
        let max_ab = a.abs().max(b.abs());
        if max_ab < 1e-30 {
            return diff < 1e-30;
        }
        diff / max_ab < rel_tol
    }

    // --- Plasma frequency ---

    #[test]
    fn test_plasma_frequency_known_density() {
        // f_pe ~ 9 * sqrt(n_e) as quick check
        let ne = 1e18;
        let f_pe = plasma_frequency_hz(ne);
        let approx = 9.0 * ne.sqrt(); // ~9 GHz
        // The exact formula gives f_pe = 8.9798... GHz
        assert!(
            approx_eq(f_pe, approx, 0.01),
            "f_pe={f_pe}, approx={approx}"
        );
    }

    #[test]
    fn test_plasma_frequency_typical_ionosphere() {
        // Ionospheric F-layer: n_e ~ 1e12 m^-3, f_pe ~ 9 MHz
        let ne = 1e12;
        let f_pe = plasma_frequency_hz(ne);
        assert!(f_pe > 8e6 && f_pe < 10e6, "f_pe = {f_pe}");
    }

    #[test]
    fn test_plasma_frequency_zero_density() {
        assert_eq!(plasma_frequency_hz(0.0), 0.0);
    }

    #[test]
    fn test_plasma_frequency_negative_density() {
        assert_eq!(plasma_frequency_hz(-1e18), 0.0);
    }

    // --- Debye length ---

    #[test]
    fn test_debye_length_fusion_plasma() {
        // T_e = 1000 eV, n_e = 1e20 m^-3 -> lambda_D ~ 2.35e-5 m
        let te = 1000.0;
        let ne = 1e20;
        let ld = debye_length_m(te, ne);
        // lambda_D = sqrt(eps0 * Te_eV * e / (ne * e^2))
        //          = sqrt(eps0 * Te_eV / (ne * e))
        let expected = (EPSILON_0 * te * ELECTRON_CHARGE / (ne * ELECTRON_CHARGE * ELECTRON_CHARGE)).sqrt();
        assert!(
            approx_eq(ld, expected, 1e-10),
            "ld={ld}, expected={expected}"
        );
        // Rough check: should be on order of tens of microns
        assert!(ld > 1e-6 && ld < 1e-3, "ld = {ld}");
    }

    #[test]
    fn test_debye_length_cold_plasma() {
        // Lower temperature -> shorter Debye length
        let ld_hot = debye_length_m(100.0, 1e18);
        let ld_cold = debye_length_m(1.0, 1e18);
        assert!(ld_hot > ld_cold);
    }

    #[test]
    fn test_debye_length_zero_temp() {
        assert_eq!(debye_length_m(0.0, 1e18), 0.0);
    }

    #[test]
    fn test_debye_length_zero_density() {
        assert_eq!(debye_length_m(10.0, 0.0), 0.0);
    }

    // --- Cyclotron frequency ---

    #[test]
    fn test_electron_cyclotron_frequency() {
        // f_ce = eB/(2*pi*m_e)
        let b = 2.0; // Tesla
        let f_ce = cyclotron_frequency_hz(ELECTRON_CHARGE, ELECTRON_MASS, b);
        let expected = ELECTRON_CHARGE * b / (2.0 * PI * ELECTRON_MASS);
        assert!(approx_eq(f_ce, expected, 1e-10));
        // Should be ~56 GHz at 2T
        assert!(f_ce > 50e9 && f_ce < 60e9, "f_ce = {f_ce}");
    }

    #[test]
    fn test_ion_cyclotron_frequency() {
        // Deuterium cyclotron at 5 T
        let m_d = 2.0 * AMU;
        let b = 5.0;
        let f_ci = cyclotron_frequency_hz(ELECTRON_CHARGE, m_d, b);
        // Should be ~38 MHz
        assert!(f_ci > 30e6 && f_ci < 50e6, "f_ci = {f_ci}");
    }

    #[test]
    fn test_cyclotron_zero_mass() {
        assert_eq!(cyclotron_frequency_hz(ELECTRON_CHARGE, 0.0, 1.0), 0.0);
    }

    // --- Larmor radius ---

    #[test]
    fn test_electron_larmor_radius() {
        // At 10 eV, 2 T for electrons: v_th = sqrt(e*10/m_e) ~ 1.87e6 m/s
        // omega_c = eB/m_e ~ 3.52e11 rad/s, r_L ~ 5.3e-6 m
        let r_l = larmor_radius_m(10.0, ELECTRON_MASS, 2.0);
        assert!(r_l > 1e-6 && r_l < 1e-4, "r_L = {r_l}");
    }

    #[test]
    fn test_ion_larmor_radius_larger() {
        // Ion Larmor radius >> electron Larmor radius at same temperature
        let te = 100.0;
        let b = 2.0;
        let r_e = larmor_radius_m(te, ELECTRON_MASS, b);
        let r_d = larmor_radius_m(te, 2.0 * AMU, b);
        assert!(r_d > r_e * 10.0, "r_d={r_d}, r_e={r_e}");
    }

    #[test]
    fn test_larmor_radius_zero_field() {
        assert_eq!(larmor_radius_m(10.0, ELECTRON_MASS, 0.0), 0.0);
    }

    #[test]
    fn test_larmor_radius_zero_temp() {
        assert_eq!(larmor_radius_m(0.0, ELECTRON_MASS, 2.0), 0.0);
    }

    // --- Reflectometry cutoff ---

    #[test]
    fn test_reflectometry_cutoff_density() {
        // At 60 GHz, cutoff density ~ 4.5e19 m^-3
        let nc = reflectometry_cutoff_density(60.0);
        assert!(nc > 4e19 && nc < 5e19, "nc = {nc}");
    }

    #[test]
    fn test_reflectometry_roundtrip() {
        // If we compute n_c from f, then f_pe(n_c) should equal f
        let freq_ghz = 90.0;
        let nc = reflectometry_cutoff_density(freq_ghz);
        let f_pe = plasma_frequency_hz(nc);
        assert!(
            approx_eq(f_pe, freq_ghz * 1e9, 1e-6),
            "f_pe={f_pe}, expected={}",
            freq_ghz * 1e9
        );
    }

    #[test]
    fn test_reflectometry_cutoff_zero() {
        assert_eq!(reflectometry_cutoff_density(0.0), 0.0);
        assert_eq!(reflectometry_cutoff_density(-10.0), 0.0);
    }

    // --- Bremsstrahlung ---

    #[test]
    fn test_bremsstrahlung_power() {
        // Typical tokamak: n_e=1e20, T_e=10000 eV, Z_eff=1.5
        let p = bremsstrahlung_power_density(1e20, 10000.0, 1.5);
        // P = 5.35e-37 * (1e20)^2 * 1.5 * sqrt(10000) = 5.35e-37 * 1e40 * 1.5 * 100
        //   = 5.35e-37 * 1.5e42 = 8.025e5 W/m^3
        let expected = BREMSSTRAHLUNG_C * 1e40 * 1.5 * 100.0;
        assert!(approx_eq(p, expected, 1e-10), "p={p}, expected={expected}");
    }

    #[test]
    fn test_bremsstrahlung_zero_density() {
        assert_eq!(bremsstrahlung_power_density(0.0, 1000.0, 1.0), 0.0);
    }

    #[test]
    fn test_bremsstrahlung_scales_with_zeff() {
        let p1 = bremsstrahlung_power_density(1e19, 100.0, 1.0);
        let p2 = bremsstrahlung_power_density(1e19, 100.0, 2.0);
        assert!(approx_eq(p2, 2.0 * p1, 1e-10));
    }

    // --- Plasma beta ---

    #[test]
    fn test_plasma_beta_low_beta() {
        // Tokamak: p ~ 1e5 Pa, B=5T -> beta ~ 2*mu0*1e5/25 ~ 1e-2
        let beta = plasma_beta(1e5, 5.0);
        assert!(beta > 0.005 && beta < 0.02, "beta = {beta}");
    }

    #[test]
    fn test_plasma_beta_unity() {
        // Solve for p that gives beta=1: p = B^2 / (2*mu_0)
        let b = 1.0;
        let p = b * b / (2.0 * MU_0);
        let beta = plasma_beta(p, b);
        assert!(approx_eq(beta, 1.0, 1e-10), "beta = {beta}");
    }

    #[test]
    fn test_plasma_beta_zero_field() {
        let beta = plasma_beta(1e5, 0.0);
        assert!(beta.is_infinite());
    }

    // --- Alfven velocity ---

    #[test]
    fn test_alfven_velocity_tokamak() {
        // B=5T, n=1e20 m^-3, deuterium -> v_A ~ 7.7e6 m/s
        let m_d = 2.0 * AMU;
        let va = alfven_velocity(5.0, 1e20, m_d);
        assert!(va > 5e6 && va < 1e7, "va = {va}");
    }

    #[test]
    fn test_alfven_velocity_zero_density() {
        assert_eq!(alfven_velocity(5.0, 0.0, 2.0 * AMU), 0.0);
    }

    #[test]
    fn test_alfven_velocity_proportional_to_b() {
        let m_d = 2.0 * AMU;
        let va1 = alfven_velocity(1.0, 1e19, m_d);
        let va2 = alfven_velocity(2.0, 1e19, m_d);
        assert!(approx_eq(va2, 2.0 * va1, 1e-10));
    }

    // --- I-V curve analysis ---

    #[test]
    fn test_iv_curve_synthetic() {
        // Synthesise a Langmuir probe I-V characteristic:
        // I = I_sat + I_e0 * exp((V - V_p) / T_e)    for V < V_p
        // I = I_sat + I_e0                               for V >= V_p (electron saturation)
        let te = 5.0; // eV
        let v_p = 10.0; // plasma potential
        let i_sat = -0.001; // ion saturation current (negative)
        let i_e0 = 0.01; // electron saturation current magnitude

        let n_points = 200;
        let v_min = -30.0;
        let v_max = 20.0;

        let mut voltage = Vec::with_capacity(n_points);
        let mut current = Vec::with_capacity(n_points);

        for k in 0..n_points {
            let v = v_min + (v_max - v_min) * k as f64 / (n_points - 1) as f64;
            let i = if v < v_p {
                i_sat + i_e0 * ((v - v_p) / te).exp()
            } else {
                i_sat + i_e0
            };
            voltage.push(v);
            current.push(i);
        }

        let config = PlasmaConfig {
            probe_area_m2: 1e-6,
            ion_mass_amu: 2.0,
            magnetic_field_t: 2.0,
            microwave_freq_ghz: 60.0,
        };
        let processor = PlasmaProcessor::new(config);
        let params = processor.analyze_iv_curve(&voltage, &current);

        // Floating potential: I=0 => I_sat + I_e0*exp((Vf-Vp)/Te) = 0
        // Vf = Vp + Te*ln(-I_sat/I_e0) = 10 + 5*ln(0.1) = 10 - 11.51 ~ -1.51
        let expected_vf = v_p + te * ((-i_sat) / i_e0).ln();
        assert!(
            approx_eq(params.floating_potential_v, expected_vf, 0.05),
            "V_float={}, expected={}",
            params.floating_potential_v,
            expected_vf
        );

        // Electron temperature should be close to 5 eV
        assert!(
            approx_eq(params.electron_temp_ev, te, 0.15),
            "T_e={}, expected={}",
            params.electron_temp_ev,
            te
        );

        // Plasma potential should be near 10 V
        assert!(
            (params.plasma_potential_v - v_p).abs() < 2.0,
            "V_p={}, expected={}",
            params.plasma_potential_v,
            v_p
        );

        // Density should be positive
        assert!(params.electron_density_m3 > 0.0);

        // Debye length should be positive
        assert!(params.debye_length_m > 0.0);
    }

    #[test]
    fn test_iv_curve_cold_plasma() {
        // Colder plasma -> steeper exponential
        let te = 1.0;
        let v_p = 5.0;
        let i_sat = -0.0005;
        let i_e0 = 0.005;

        let n_points = 300;
        let v_min = -20.0;
        let v_max = 10.0;

        let mut voltage = Vec::with_capacity(n_points);
        let mut current = Vec::with_capacity(n_points);

        for k in 0..n_points {
            let v = v_min + (v_max - v_min) * k as f64 / (n_points - 1) as f64;
            let i = if v < v_p {
                i_sat + i_e0 * ((v - v_p) / te).exp()
            } else {
                i_sat + i_e0
            };
            voltage.push(v);
            current.push(i);
        }

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let params = processor.analyze_iv_curve(&voltage, &current);

        assert!(
            approx_eq(params.electron_temp_ev, te, 0.2),
            "T_e={}, expected={}",
            params.electron_temp_ev,
            te
        );
    }

    // --- Electron density from saturation ---

    #[test]
    fn test_electron_density_from_saturation_basic() {
        // Known: I_sat=1mA, Te=10eV, A=1mm^2, deuterium
        let i_sat = 1e-3;
        let te = 10.0;
        let area = 1e-6;
        let m_d = 2.0 * AMU;
        let ne = electron_density_from_saturation(i_sat, te, area, m_d);
        // Should give a sensible tokamak edge density ~1e17-1e19
        assert!(ne > 1e16 && ne < 1e20, "ne = {ne}");
    }

    #[test]
    fn test_electron_density_zero_area() {
        assert_eq!(
            electron_density_from_saturation(1e-3, 10.0, 0.0, 2.0 * AMU),
            0.0
        );
    }

    // --- Thermal velocity ---

    #[test]
    fn test_thermal_velocity_electron() {
        // At 10 eV: v_th_e = sqrt(e*10/m_e) ~ 1.87e6 m/s
        let v = thermal_velocity(10.0, ELECTRON_MASS);
        assert!(v > 1e6 && v < 3e6, "v_th = {v}");
    }

    // --- Coulomb logarithm ---

    #[test]
    fn test_coulomb_logarithm_fusion() {
        // Typical fusion: n_e=1e20, T_e=10000 eV -> ln(Lambda) ~ 17
        let ln_l = coulomb_logarithm(1e20, 10000.0);
        assert!(ln_l > 10.0 && ln_l < 25.0, "ln_Lambda = {ln_l}");
    }

    // --- Spitzer resistivity ---

    #[test]
    fn test_spitzer_resistivity_hot_plasma() {
        // At 10 keV, Z=1, ln_Lambda=17 -> very low resistivity
        let eta = spitzer_resistivity(10000.0, 1.0, 17.0);
        assert!(eta > 0.0 && eta < 1e-6, "eta = {eta}");
    }

    #[test]
    fn test_spitzer_resistivity_cold_is_higher() {
        let eta_cold = spitzer_resistivity(1.0, 1.0, 17.0);
        let eta_hot = spitzer_resistivity(1000.0, 1.0, 17.0);
        assert!(eta_cold > eta_hot);
    }

    // --- Skin depth ---

    #[test]
    fn test_skin_depth() {
        // At n_e = 1e18 -> omega_pe ~ 5.6e10 -> delta ~ c/omega_pe ~ 5.3e-3 m
        let delta = skin_depth_m(1e18);
        assert!(delta > 1e-3 && delta < 1e-2, "delta = {delta}");
    }

    #[test]
    fn test_skin_depth_zero_density() {
        assert!(skin_depth_m(0.0).is_infinite());
    }

    // --- PlasmaConfig ---

    #[test]
    fn test_config_default() {
        let config = PlasmaConfig::default();
        assert_eq!(config.ion_mass_amu, 2.0);
        assert!(config.probe_area_m2 > 0.0);
    }

    #[test]
    fn test_config_ion_mass_kg() {
        let config = PlasmaConfig::default();
        let m_d = config.ion_mass_kg();
        // Deuterium mass ~ 3.34e-27 kg
        assert!(approx_eq(m_d, 2.0 * AMU, 1e-10));
    }

    // --- Edge cases and consistency ---

    #[test]
    fn test_debye_length_consistency_with_frequency() {
        // lambda_D = v_th_e / (omega_pe * sqrt(2))  (for Maxwellian)
        // Actually lambda_D = sqrt(eps0 * kTe / ne e^2) and
        // v_th = sqrt(kTe/me), omega_pe = sqrt(ne e^2/(me eps0))
        // So lambda_D = v_th / omega_pe
        let te = 100.0;
        let ne = 1e19;
        let ld = debye_length_m(te, ne);
        let v_th = thermal_velocity(te, ELECTRON_MASS);
        let omega_pe = 2.0 * PI * plasma_frequency_hz(ne);
        let ld_from_vth = v_th / omega_pe;
        assert!(
            approx_eq(ld, ld_from_vth, 1e-6),
            "ld={ld}, ld_from_vth={ld_from_vth}"
        );
    }

    #[test]
    #[should_panic]
    fn test_iv_curve_mismatched_lengths() {
        let config = PlasmaConfig::default();
        let proc = PlasmaProcessor::new(config);
        proc.analyze_iv_curve(&[1.0, 2.0], &[1.0]);
    }

    #[test]
    #[should_panic]
    fn test_iv_curve_too_few_points() {
        let config = PlasmaConfig::default();
        let proc = PlasmaProcessor::new(config);
        proc.analyze_iv_curve(&[1.0, 2.0, 3.0, 4.0], &[0.1, 0.2, 0.3, 0.4]);
    }
}
