//! Plasma diagnostics signal processing for fusion research and industrial plasma applications.
//!
//! Implements Langmuir probe I-V curve analysis, Electron Energy Distribution Function (EEDF),
//! microwave/mm-wave interferometry, Optical Emission Spectroscopy (OES), Thomson scattering,
//! and fundamental plasma physics calculations for measuring electron temperature, density,
//! and plasma frequency.
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

/// Vacuum permeability (H/m)
pub const MU_0: f64 = 1.256_637_062_12e-6;

/// Speed of light (m/s)
pub const SPEED_OF_LIGHT: f64 = 2.997_924_58e8;

/// Atomic mass unit (kg)
pub const AMU: f64 = 1.660_539_066_60e-27;

/// Bremsstrahlung constant C_brem for P = C * n_e^2 * Z_eff * sqrt(T_e)
/// where T_e in eV, n_e in m^-3, result in W/m^3.
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
    /// Microwave frequency for reflectometry/interferometry (GHz).
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

/// Results from EEDF computation via Druyvesteyn method.
#[derive(Debug, Clone)]
pub struct EedfResult {
    /// Energy values (eV).
    pub energy_ev: Vec<f64>,
    /// EEDF values f(E) (eV^{-3/2} m^{-3}).
    pub eedf: Vec<f64>,
    /// Mean electron energy (eV).
    pub mean_energy_ev: f64,
}

/// Results from a Boltzmann plot analysis.
#[derive(Debug, Clone)]
pub struct BoltzmannPlotResult {
    /// Excitation temperature (eV).
    pub excitation_temp_ev: f64,
    /// Quality of fit (R-squared).
    pub r_squared: f64,
    /// Y-values of the Boltzmann plot: ln(I*lambda/(g*A)).
    pub y_values: Vec<f64>,
}

/// Results from Thomson scattering analysis.
#[derive(Debug, Clone)]
pub struct ThomsonResult {
    /// Electron temperature (eV).
    pub electron_temp_ev: f64,
    /// Electron density (m^-3).
    pub electron_density_m3: f64,
    /// Spectral width of scattered light (nm).
    pub spectral_width_nm: f64,
}

/// A single spectral line for OES analysis.
#[derive(Debug, Clone)]
pub struct SpectralLine {
    /// Measured intensity (arbitrary units).
    pub intensity: f64,
    /// Wavelength (nm).
    pub wavelength_nm: f64,
    /// Statistical weight of upper level.
    pub g_upper: f64,
    /// Transition probability (s^-1).
    pub a_coefficient: f64,
    /// Upper level energy (eV).
    pub e_upper_ev: f64,
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// Plasma diagnostics processor.
///
/// Wraps a [`PlasmaConfig`] and provides high-level methods for
/// analysing Langmuir probe I-V characteristics, EEDF, OES, Thomson
/// scattering, and microwave interferometry.
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
    /// voltage (V) and measured probe current (A). They must have the
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
            fit_electron_temperature(&v_ret, &ln_i_ret)
        } else {
            // Fallback: use rough V_p - V_f relationship T_e ~ (V_p - V_f) / ln(2)
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

    /// Compute the Electron Energy Distribution Function (EEDF) from
    /// a Langmuir probe I-V curve using the Druyvesteyn method.
    ///
    /// f(E) is proportional to d^2 I / dV^2 evaluated at V = V_p - E/e.
    ///
    /// Returns energies and corresponding EEDF values, plus the mean energy.
    pub fn compute_eedf(
        &self,
        voltage: &[f64],
        current: &[f64],
        v_plasma: f64,
        smoothing_points: usize,
    ) -> EedfResult {
        assert_eq!(voltage.len(), current.len());
        let n = voltage.len();
        assert!(n >= 5);

        // Sort by voltage
        let mut idx: Vec<usize> = (0..n).collect();
        idx.sort_by(|&a, &b| voltage[a].partial_cmp(&voltage[b]).unwrap());
        let v_sorted: Vec<f64> = idx.iter().map(|&i| voltage[i]).collect();
        let i_sorted: Vec<f64> = idx.iter().map(|&i| current[i]).collect();

        // Optional smoothing pass (boxcar average)
        let smoothed = if smoothing_points > 1 {
            smooth_signal(&i_sorted, smoothing_points)
        } else {
            i_sorted.clone()
        };

        // Numerical second derivative using central differences
        let d2i = numerical_second_derivative(&v_sorted, &smoothed);

        // Druyvesteyn: f(E) = (2/(e^2 * A)) * sqrt(2*m_e*E/e) * |d2I/dV2|
        // Energy E = e * (V_p - V), only for V < V_p
        let probe_area = self.config.probe_area_m2;
        let prefactor = 2.0 / (ELECTRON_CHARGE * ELECTRON_CHARGE * probe_area);

        let mut energy_ev = Vec::new();
        let mut eedf = Vec::new();

        for k in 0..d2i.len() {
            let e_ev = v_plasma - v_sorted[k + 1]; // offset by 1 due to second derivative
            if e_ev > 0.0 {
                let e_j = e_ev * ELECTRON_CHARGE;
                let sqrt_term = (2.0 * ELECTRON_MASS * e_j).sqrt();
                let f_e = prefactor * sqrt_term * d2i[k].abs();
                energy_ev.push(e_ev);
                eedf.push(f_e);
            }
        }

        // Mean energy: <E> = integral(E * f(E) dE) / integral(f(E) dE)
        let mean_energy = if energy_ev.len() >= 2 {
            let mut num = 0.0;
            let mut den = 0.0;
            for k in 0..energy_ev.len() - 1 {
                let de = (energy_ev[k] - energy_ev[k + 1]).abs();
                let e_mid = (energy_ev[k] + energy_ev[k + 1]) * 0.5;
                let f_mid = (eedf[k] + eedf[k + 1]) * 0.5;
                num += e_mid * f_mid * de;
                den += f_mid * de;
            }
            if den > 0.0 { num / den } else { 0.0 }
        } else {
            0.0
        };

        EedfResult {
            energy_ev,
            eedf,
            mean_energy_ev: mean_energy,
        }
    }

    /// Microwave interferometry: compute phase shift for a given
    /// line-integrated electron density.
    ///
    /// delta_phi = (e^2 / (2 * m_e * c * epsilon_0 * omega)) * integral(n_e dl)
    ///
    /// `line_integrated_density` is in m^-2 (integral of n_e along the path).
    pub fn interferometry_phase_shift(&self, line_integrated_density: f64) -> f64 {
        let freq_hz = self.config.microwave_freq_ghz * 1e9;
        let omega = 2.0 * PI * freq_hz;
        if omega <= 0.0 {
            return 0.0;
        }
        let coeff = ELECTRON_CHARGE * ELECTRON_CHARGE
            / (2.0 * ELECTRON_MASS * SPEED_OF_LIGHT * EPSILON_0 * omega);
        coeff * line_integrated_density
    }

    /// Compute line-integrated density from the measured phase shift (radians).
    pub fn interferometry_density_from_phase(&self, phase_shift_rad: f64) -> f64 {
        let freq_hz = self.config.microwave_freq_ghz * 1e9;
        let omega = 2.0 * PI * freq_hz;
        if omega <= 0.0 {
            return 0.0;
        }
        let coeff = ELECTRON_CHARGE * ELECTRON_CHARGE
            / (2.0 * ELECTRON_MASS * SPEED_OF_LIGHT * EPSILON_0 * omega);
        if coeff <= 0.0 {
            return 0.0;
        }
        phase_shift_rad / coeff
    }

    /// Fringe counting: each full fringe (2*pi phase shift) corresponds
    /// to a fixed line-integrated density increment.
    pub fn interferometry_density_per_fringe(&self) -> f64 {
        self.interferometry_density_from_phase(2.0 * PI)
    }

    /// Boltzmann plot analysis from spectral line data.
    ///
    /// Plots ln(I * lambda / (g * A)) vs E_upper. The slope is -1/(k_B * T_exc).
    /// Returns excitation temperature in eV, R-squared, and the y-values.
    pub fn boltzmann_plot(&self, lines: &[SpectralLine]) -> BoltzmannPlotResult {
        assert!(lines.len() >= 2, "need at least 2 spectral lines");

        let mut x_vals = Vec::with_capacity(lines.len()); // E_upper (eV)
        let mut y_vals = Vec::with_capacity(lines.len()); // ln(I*lambda/(g*A))

        for line in lines {
            let y = (line.intensity * line.wavelength_nm
                / (line.g_upper * line.a_coefficient))
                .ln();
            x_vals.push(line.e_upper_ev);
            y_vals.push(y);
        }

        // Linear regression: y = a + b*x where b = -1/(k_B * T_exc)
        let (slope, _intercept, r_sq) = linear_regression(&x_vals, &y_vals);

        // slope = -1/T_exc (in eV since E is in eV)
        let t_exc_ev = if slope.abs() > 1e-30 {
            -1.0 / slope
        } else {
            0.0
        };

        BoltzmannPlotResult {
            excitation_temp_ev: t_exc_ev,
            r_squared: r_sq,
            y_values: y_vals,
        }
    }

    /// Line ratio method for T_e estimation from two spectral lines.
    ///
    /// For lines with known upper level energies E1, E2 and known g*A values:
    /// I1/I2 = (g1*A1*lambda2) / (g2*A2*lambda1) * exp(-(E1-E2)/T_e)
    ///
    /// Returns T_e in eV.
    pub fn line_ratio_temperature(
        &self,
        line1: &SpectralLine,
        line2: &SpectralLine,
    ) -> f64 {
        let ratio = line1.intensity / line2.intensity;
        let ga_ratio = (line1.g_upper * line1.a_coefficient * line2.wavelength_nm)
            / (line2.g_upper * line2.a_coefficient * line1.wavelength_nm);

        if ga_ratio <= 0.0 || ratio <= 0.0 {
            return 0.0;
        }

        let de = line1.e_upper_ev - line2.e_upper_ev;
        let ln_term = (ratio / ga_ratio).ln();

        if ln_term.abs() < 1e-30 {
            return 0.0;
        }

        // T_e = -(E1-E2) / ln(I1/I2 / (g1A1 lambda2 / g2A2 lambda1))
        -de / ln_term
    }

    /// Stark broadening: estimate n_e from hydrogen line width.
    ///
    /// delta_lambda = C * n_e^(2/3)
    ///
    /// where C is the Stark broadening coefficient. For H-beta at ~486.1 nm,
    /// a typical C ~ 2.5e-16 nm / (m^-3)^(2/3).
    ///
    /// Returns n_e in m^-3.
    pub fn stark_broadening_density(
        &self,
        delta_lambda_nm: f64,
        stark_coefficient: f64,
    ) -> f64 {
        if delta_lambda_nm <= 0.0 || stark_coefficient <= 0.0 {
            return 0.0;
        }
        // n_e = (delta_lambda / C)^(3/2)
        (delta_lambda_nm / stark_coefficient).powf(1.5)
    }

    /// Thomson scattering analysis.
    ///
    /// The scattered spectrum width is related to T_e:
    ///   delta_lambda = 2 * lambda_0 * sqrt(2 * k_B * T_e / (m_e * c^2)) * sin(theta/2)
    ///
    /// And n_e is proportional to the integrated scattered power.
    ///
    /// `wavelength_nm`: probe laser wavelength (e.g. 1064.0 for Nd:YAG)
    /// `scattering_angle_deg`: scattering angle in degrees
    /// `spectral_width_nm`: measured 1/e half-width of scattered spectrum
    /// `integrated_power`: integrated scattered signal (arbitrary units)
    /// `calibration_factor`: relates integrated power to n_e (m^-3 per unit power)
    pub fn analyze_thomson_scattering(
        &self,
        wavelength_nm: f64,
        scattering_angle_deg: f64,
        spectral_width_nm: f64,
        integrated_power: f64,
        calibration_factor: f64,
    ) -> ThomsonResult {
        let theta_rad = scattering_angle_deg * PI / 180.0;
        let sin_half = (theta_rad / 2.0).sin();

        // Solve for T_e from spectral width:
        // delta_lambda = 2 * lambda_0 * sqrt(2*kB*Te / (me*c^2)) * sin(theta/2)
        // => Te = (delta_lambda / (2*lambda_0*sin(theta/2)))^2 * me*c^2 / (2*kB)
        let te_ev = if sin_half.abs() > 1e-30 && wavelength_nm > 0.0 {
            let ratio = spectral_width_nm / (2.0 * wavelength_nm * sin_half);
            let te_joules = ratio * ratio * ELECTRON_MASS * SPEED_OF_LIGHT * SPEED_OF_LIGHT / 2.0;
            te_joules / ELECTRON_CHARGE // convert J to eV
        } else {
            0.0
        };

        let ne = integrated_power * calibration_factor;

        // Compute expected spectral width for verification
        let expected_width = thomson_spectral_width(wavelength_nm, te_ev, scattering_angle_deg);

        ThomsonResult {
            electron_temp_ev: te_ev,
            electron_density_m3: ne,
            spectral_width_nm: expected_width,
        }
    }
}

// ---------------------------------------------------------------------------
// Free functions -- plasma physics formulae
// ---------------------------------------------------------------------------

/// Fit electron temperature from the slope of ln(I_e) vs V in the electron
/// retardation region using ordinary least-squares.
/// Returns T_e in eV (= 1/slope).
pub fn fit_electron_temperature(voltage: &[f64], current_ln: &[f64]) -> f64 {
    let n = voltage.len().min(current_ln.len());
    if n < 2 {
        return 1.0;
    }

    let (slope, _intercept, _r_sq) = linear_regression(&voltage[..n], &current_ln[..n]);

    if slope <= 0.0 || !slope.is_finite() {
        return 1.0;
    }

    1.0 / slope
}

/// Compute electron density from the ion saturation current using the Bohm
/// criterion.
///
/// n_e = I_sat / (0.6 * e * A * sqrt(k * T_e / m_i))
pub fn electron_density_from_saturation(
    i_sat: f64,
    te_ev: f64,
    probe_area: f64,
    ion_mass_kg: f64,
) -> f64 {
    if probe_area <= 0.0 || ion_mass_kg <= 0.0 || te_ev <= 0.0 {
        return 0.0;
    }

    let kt_e = te_ev * ELECTRON_CHARGE;
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
pub fn plasma_frequency_hz(density_m3: f64) -> f64 {
    if density_m3 <= 0.0 {
        return 0.0;
    }
    let omega_pe = (density_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE
        / (ELECTRON_MASS * EPSILON_0))
        .sqrt();
    omega_pe / (2.0 * PI)
}

/// Debye length (m).
///
/// lambda_D = sqrt(epsilon_0 * k_B * T_e / (n_e * e^2))
///
/// where `te_ev` is in eV so k_B*T_e = te_ev * e.
pub fn debye_length_m(te_ev: f64, ne_m3: f64) -> f64 {
    if te_ev <= 0.0 || ne_m3 <= 0.0 {
        return 0.0;
    }

    let kt_e = te_ev * ELECTRON_CHARGE;
    (EPSILON_0 * kt_e / (ne_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE)).sqrt()
}

/// Thermal velocity (m/s) for a particle of given mass and temperature (eV).
///
/// v_th = sqrt(k_B * T_e / m)
pub fn thermal_velocity(te_ev: f64, mass_kg: f64) -> f64 {
    if te_ev <= 0.0 || mass_kg <= 0.0 {
        return 0.0;
    }
    (te_ev * ELECTRON_CHARGE / mass_kg).sqrt()
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
/// n_c = epsilon_0 * m_e * omega^2 / e^2
pub fn reflectometry_cutoff_density(freq_ghz: f64) -> f64 {
    if freq_ghz <= 0.0 {
        return 0.0;
    }

    let f_hz = freq_ghz * 1e9;
    let omega = 2.0 * PI * f_hz;
    omega * omega * ELECTRON_MASS * EPSILON_0
        / (ELECTRON_CHARGE * ELECTRON_CHARGE)
}

/// Cutoff density for a given angular frequency.
///
/// n_c = epsilon_0 * m_e * omega^2 / e^2
pub fn cutoff_density(omega: f64) -> f64 {
    if omega <= 0.0 {
        return 0.0;
    }
    EPSILON_0 * ELECTRON_MASS * omega * omega
        / (ELECTRON_CHARGE * ELECTRON_CHARGE)
}

/// Bremsstrahlung radiated power density (W/m^3).
///
/// P_brem = C_brem * n_e^2 * Z_eff * sqrt(T_e)
pub fn bremsstrahlung_power_density(ne_m3: f64, te_ev: f64, z_eff: f64) -> f64 {
    if ne_m3 <= 0.0 || te_ev <= 0.0 || z_eff <= 0.0 {
        return 0.0;
    }

    BREMSSTRAHLUNG_C * ne_m3 * ne_m3 * z_eff * te_ev.sqrt()
}

/// Plasma beta -- ratio of kinetic pressure to magnetic pressure.
///
/// beta = n * k_B * T / (B^2 / (2*mu_0))
///
/// This form takes pressure directly: beta = 2 * mu_0 * p / B^2
pub fn plasma_beta(pressure_pa: f64, b_tesla: f64) -> f64 {
    if b_tesla.abs() < 1e-30 {
        return f64::INFINITY;
    }

    2.0 * MU_0 * pressure_pa / (b_tesla * b_tesla)
}

/// Plasma beta from density (m^-3), temperature (eV), and B field (T).
///
/// beta = n * k_B * T / (B^2 / (2*mu_0))
pub fn plasma_beta_from_nt(ne_m3: f64, te_ev: f64, b_tesla: f64) -> f64 {
    let pressure = ne_m3 * te_ev * ELECTRON_CHARGE; // p = n * kT (eV -> J)
    plasma_beta(pressure, b_tesla)
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

/// Coulomb logarithm for electron-ion collisions.
///
/// ln(Lambda) ~ 23 - ln(sqrt(n_e) * T_e^{-3/2}) (for T_e > 10 eV regime)
pub fn coulomb_logarithm(ne_m3: f64, te_ev: f64) -> f64 {
    if ne_m3 <= 0.0 || te_ev <= 0.0 {
        return 10.0;
    }

    let val = 23.0 - (ne_m3.sqrt() * te_ev.powf(-1.5)).ln();
    val.max(1.0)
}

/// Electron-ion collision frequency (Hz).
///
/// nu_ei = (n_e * e^4 * ln_Lambda) / (12 * pi^{3/2} * epsilon_0^2 * m_e^{1/2} * (k_B*T_e)^{3/2})
///
/// Simplified: nu_ei ~ 2.91e-6 * n_e * ln_Lambda * T_e^{-3/2} (T_e in eV)
pub fn collision_frequency_ei(ne_m3: f64, te_ev: f64) -> f64 {
    if ne_m3 <= 0.0 || te_ev <= 0.0 {
        return 0.0;
    }
    let ln_lambda = coulomb_logarithm(ne_m3, te_ev);
    2.91e-6 * ne_m3 * ln_lambda * te_ev.powf(-1.5)
}

/// Electron-electron collision frequency (Hz).
///
/// nu_ee ~ 4.8e-6 * n_e * ln_Lambda * T_e^{-3/2}
pub fn collision_frequency_ee(ne_m3: f64, te_ev: f64) -> f64 {
    if ne_m3 <= 0.0 || te_ev <= 0.0 {
        return 0.0;
    }
    let ln_lambda = coulomb_logarithm(ne_m3, te_ev);
    4.8e-6 * ne_m3 * ln_lambda * te_ev.powf(-1.5)
}

/// Spitzer resistivity (Ohm-m).
///
/// eta ~ 5.2e-5 * Z_eff * ln_Lambda / T_e^{3/2} (Ohm-m, T_e in eV)
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

    SPEED_OF_LIGHT / omega_pe
}

/// Thomson scattering spectral width (nm) for given parameters.
///
/// delta_lambda = 2 * lambda_0 * sqrt(2*k_B*T_e / (m_e*c^2)) * sin(theta/2)
pub fn thomson_spectral_width(wavelength_nm: f64, te_ev: f64, angle_deg: f64) -> f64 {
    if te_ev <= 0.0 || wavelength_nm <= 0.0 {
        return 0.0;
    }
    let theta_rad = angle_deg * PI / 180.0;
    let sin_half = (theta_rad / 2.0).sin();
    let kt_e = te_ev * ELECTRON_CHARGE;
    2.0 * wavelength_nm * (2.0 * kt_e / (ELECTRON_MASS * SPEED_OF_LIGHT * SPEED_OF_LIGHT)).sqrt()
        * sin_half
}

/// Stark broadening: estimate n_e from hydrogen line FWHM.
///
/// delta_lambda = C * n_e^(2/3)
/// n_e = (delta_lambda / C)^(3/2)
pub fn stark_broadening_ne(delta_lambda_nm: f64, stark_coefficient: f64) -> f64 {
    if delta_lambda_nm <= 0.0 || stark_coefficient <= 0.0 {
        return 0.0;
    }
    (delta_lambda_nm / stark_coefficient).powf(1.5)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Find the voltage at which the current crosses zero via linear interpolation.
fn find_zero_crossing(voltage: &[f64], current: &[f64]) -> f64 {
    for k in 0..voltage.len() - 1 {
        if current[k] * current[k + 1] <= 0.0 {
            let di = current[k + 1] - current[k];
            if di.abs() < 1e-30 {
                return (voltage[k] + voltage[k + 1]) * 0.5;
            }
            let t = -current[k] / di;
            return voltage[k] + t * (voltage[k + 1] - voltage[k]);
        }
    }
    (voltage[0] + voltage[voltage.len() - 1]) * 0.5
}

/// Find the plasma potential as the voltage of maximum dI/dV (first derivative).
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

/// Simple boxcar (moving average) smoother.
fn smooth_signal(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    let half = window / 2;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = (i + half + 1).min(n);
        let sum: f64 = data[lo..hi].iter().sum();
        out.push(sum / (hi - lo) as f64);
    }
    out
}

/// Numerical second derivative using central differences.
/// Returns a vector of length n-2.
fn numerical_second_derivative(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    assert!(n >= 3);
    let mut d2 = Vec::with_capacity(n - 2);
    for k in 1..n - 1 {
        let dx1 = x[k] - x[k - 1];
        let dx2 = x[k + 1] - x[k];
        let dx_avg = (dx1 + dx2) * 0.5;
        if dx_avg.abs() < 1e-30 {
            d2.push(0.0);
        } else {
            let d2y = (y[k + 1] - 2.0 * y[k] + y[k - 1]) / (dx_avg * dx_avg);
            d2.push(d2y);
        }
    }
    d2
}

/// Ordinary least-squares linear regression: y = a + b*x.
/// Returns (slope, intercept, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len().min(y.len());
    if n < 2 {
        return (0.0, 0.0, 0.0);
    }

    let nf = n as f64;
    let sum_x: f64 = x[..n].iter().sum();
    let sum_y: f64 = y[..n].iter().sum();
    let sum_xx: f64 = x[..n].iter().map(|v| v * v).sum();
    let sum_xy: f64 = x[..n].iter().zip(y[..n].iter()).map(|(xi, yi)| xi * yi).sum();

    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / nf, 0.0);
    }

    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / nf;

    // R-squared
    let mean_y = sum_y / nf;
    let ss_tot: f64 = y[..n].iter().map(|yi| (yi - mean_y).powi(2)).sum();
    let ss_res: f64 = x[..n]
        .iter()
        .zip(y[..n].iter())
        .map(|(xi, yi)| {
            let pred = intercept + slope * xi;
            (yi - pred).powi(2)
        })
        .sum();

    let r_sq = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

    (slope, intercept, r_sq)
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

    // -----------------------------------------------------------------------
    // Physical constants sanity
    // -----------------------------------------------------------------------

    #[test]
    fn test_speed_of_light_value() {
        assert!(approx_eq(SPEED_OF_LIGHT, 2.998e8, 1e-3));
    }

    #[test]
    fn test_mu0_epsilon0_c_relation() {
        // c = 1/sqrt(mu_0 * epsilon_0)
        let c_derived = 1.0 / (MU_0 * EPSILON_0).sqrt();
        assert!(approx_eq(c_derived, SPEED_OF_LIGHT, 1e-4),
            "c_derived={c_derived}, SPEED_OF_LIGHT={SPEED_OF_LIGHT}");
    }

    // -----------------------------------------------------------------------
    // Plasma frequency
    // -----------------------------------------------------------------------

    #[test]
    fn test_plasma_frequency_known_density() {
        let ne = 1e18;
        let f_pe = plasma_frequency_hz(ne);
        let approx = 9.0 * ne.sqrt();
        assert!(approx_eq(f_pe, approx, 0.01), "f_pe={f_pe}, approx={approx}");
    }

    #[test]
    fn test_plasma_frequency_typical_ionosphere() {
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

    // -----------------------------------------------------------------------
    // Debye length
    // -----------------------------------------------------------------------

    #[test]
    fn test_debye_length_fusion_plasma() {
        let te = 1000.0;
        let ne = 1e20;
        let ld = debye_length_m(te, ne);
        let expected = (EPSILON_0 * te * ELECTRON_CHARGE
            / (ne * ELECTRON_CHARGE * ELECTRON_CHARGE))
            .sqrt();
        assert!(approx_eq(ld, expected, 1e-10));
        assert!(ld > 1e-6 && ld < 1e-3, "ld = {ld}");
    }

    #[test]
    fn test_debye_length_cold_plasma() {
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

    // -----------------------------------------------------------------------
    // Cyclotron frequency
    // -----------------------------------------------------------------------

    #[test]
    fn test_electron_cyclotron_frequency() {
        let b = 2.0;
        let f_ce = cyclotron_frequency_hz(ELECTRON_CHARGE, ELECTRON_MASS, b);
        let expected = ELECTRON_CHARGE * b / (2.0 * PI * ELECTRON_MASS);
        assert!(approx_eq(f_ce, expected, 1e-10));
        assert!(f_ce > 50e9 && f_ce < 60e9, "f_ce = {f_ce}");
    }

    #[test]
    fn test_ion_cyclotron_frequency() {
        let m_d = 2.0 * AMU;
        let b = 5.0;
        let f_ci = cyclotron_frequency_hz(ELECTRON_CHARGE, m_d, b);
        assert!(f_ci > 30e6 && f_ci < 50e6, "f_ci = {f_ci}");
    }

    #[test]
    fn test_cyclotron_zero_mass() {
        assert_eq!(cyclotron_frequency_hz(ELECTRON_CHARGE, 0.0, 1.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // Larmor radius
    // -----------------------------------------------------------------------

    #[test]
    fn test_electron_larmor_radius() {
        let r_l = larmor_radius_m(10.0, ELECTRON_MASS, 2.0);
        assert!(r_l > 1e-6 && r_l < 1e-4, "r_L = {r_l}");
    }

    #[test]
    fn test_ion_larmor_radius_larger() {
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

    // -----------------------------------------------------------------------
    // Reflectometry cutoff
    // -----------------------------------------------------------------------

    #[test]
    fn test_reflectometry_cutoff_density() {
        let nc = reflectometry_cutoff_density(60.0);
        assert!(nc > 4e19 && nc < 5e19, "nc = {nc}");
    }

    #[test]
    fn test_reflectometry_roundtrip() {
        let freq_ghz = 90.0;
        let nc = reflectometry_cutoff_density(freq_ghz);
        let f_pe = plasma_frequency_hz(nc);
        assert!(approx_eq(f_pe, freq_ghz * 1e9, 1e-6));
    }

    #[test]
    fn test_cutoff_density_from_omega() {
        let freq_ghz = 60.0;
        let omega = 2.0 * PI * freq_ghz * 1e9;
        let nc1 = reflectometry_cutoff_density(freq_ghz);
        let nc2 = cutoff_density(omega);
        assert!(approx_eq(nc1, nc2, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Bremsstrahlung
    // -----------------------------------------------------------------------

    #[test]
    fn test_bremsstrahlung_power() {
        let p = bremsstrahlung_power_density(1e20, 10000.0, 1.5);
        let expected = BREMSSTRAHLUNG_C * 1e40 * 1.5 * 100.0;
        assert!(approx_eq(p, expected, 1e-10));
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

    // -----------------------------------------------------------------------
    // Plasma beta
    // -----------------------------------------------------------------------

    #[test]
    fn test_plasma_beta_low_beta() {
        let beta = plasma_beta(1e5, 5.0);
        assert!(beta > 0.005 && beta < 0.02, "beta = {beta}");
    }

    #[test]
    fn test_plasma_beta_unity() {
        let b = 1.0;
        let p = b * b / (2.0 * MU_0);
        let beta = plasma_beta(p, b);
        assert!(approx_eq(beta, 1.0, 1e-10), "beta = {beta}");
    }

    #[test]
    fn test_plasma_beta_from_nt() {
        let ne = 1e20;
        let te = 1000.0; // eV
        let b = 5.0;
        let beta = plasma_beta_from_nt(ne, te, b);
        // p = ne * Te * e = 1e20 * 1000 * 1.6e-19 = 1.6e4 Pa
        // beta = 2*mu0*p/B^2
        let p = ne * te * ELECTRON_CHARGE;
        let expected = 2.0 * MU_0 * p / (b * b);
        assert!(approx_eq(beta, expected, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Alfven velocity
    // -----------------------------------------------------------------------

    #[test]
    fn test_alfven_velocity_tokamak() {
        let m_d = 2.0 * AMU;
        let va = alfven_velocity(5.0, 1e20, m_d);
        assert!(va > 5e6 && va < 1e7, "va = {va}");
    }

    #[test]
    fn test_alfven_velocity_proportional_to_b() {
        let m_d = 2.0 * AMU;
        let va1 = alfven_velocity(1.0, 1e19, m_d);
        let va2 = alfven_velocity(2.0, 1e19, m_d);
        assert!(approx_eq(va2, 2.0 * va1, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Collision frequencies
    // -----------------------------------------------------------------------

    #[test]
    fn test_collision_frequency_ei_positive() {
        let nu = collision_frequency_ei(1e20, 100.0);
        assert!(nu > 0.0, "nu_ei = {nu}");
    }

    #[test]
    fn test_collision_frequency_ei_higher_at_lower_temp() {
        let nu_cold = collision_frequency_ei(1e19, 1.0);
        let nu_hot = collision_frequency_ei(1e19, 100.0);
        assert!(nu_cold > nu_hot, "nu_cold={nu_cold}, nu_hot={nu_hot}");
    }

    #[test]
    fn test_collision_frequency_ee_positive() {
        let nu = collision_frequency_ee(1e19, 10.0);
        assert!(nu > 0.0);
    }

    #[test]
    fn test_collision_frequency_ee_vs_ei() {
        // ee collision frequency is typically higher than ei
        let ne = 1e19;
        let te = 10.0;
        let nu_ee = collision_frequency_ee(ne, te);
        let nu_ei = collision_frequency_ei(ne, te);
        assert!(nu_ee > nu_ei, "nu_ee={nu_ee}, nu_ei={nu_ei}");
    }

    // -----------------------------------------------------------------------
    // Spitzer resistivity and skin depth
    // -----------------------------------------------------------------------

    #[test]
    fn test_spitzer_resistivity_hot_plasma() {
        let eta = spitzer_resistivity(10000.0, 1.0, 17.0);
        assert!(eta > 0.0 && eta < 1e-6, "eta = {eta}");
    }

    #[test]
    fn test_spitzer_resistivity_cold_is_higher() {
        let eta_cold = spitzer_resistivity(1.0, 1.0, 17.0);
        let eta_hot = spitzer_resistivity(1000.0, 1.0, 17.0);
        assert!(eta_cold > eta_hot);
    }

    #[test]
    fn test_skin_depth() {
        let delta = skin_depth_m(1e18);
        assert!(delta > 1e-3 && delta < 1e-2, "delta = {delta}");
    }

    #[test]
    fn test_skin_depth_zero_density() {
        assert!(skin_depth_m(0.0).is_infinite());
    }

    // -----------------------------------------------------------------------
    // Thermal velocity
    // -----------------------------------------------------------------------

    #[test]
    fn test_thermal_velocity_electron() {
        let v = thermal_velocity(10.0, ELECTRON_MASS);
        assert!(v > 1e6 && v < 3e6, "v_th = {v}");
    }

    // -----------------------------------------------------------------------
    // Coulomb logarithm
    // -----------------------------------------------------------------------

    #[test]
    fn test_coulomb_logarithm_fusion() {
        let ln_l = coulomb_logarithm(1e20, 10000.0);
        assert!(ln_l > 10.0 && ln_l < 25.0, "ln_Lambda = {ln_l}");
    }

    // -----------------------------------------------------------------------
    // Debye length consistency
    // -----------------------------------------------------------------------

    #[test]
    fn test_debye_length_consistency_with_frequency() {
        let te = 100.0;
        let ne = 1e19;
        let ld = debye_length_m(te, ne);
        let v_th = thermal_velocity(te, ELECTRON_MASS);
        let omega_pe = 2.0 * PI * plasma_frequency_hz(ne);
        let ld_from_vth = v_th / omega_pe;
        assert!(approx_eq(ld, ld_from_vth, 1e-6));
    }

    // -----------------------------------------------------------------------
    // I-V curve analysis
    // -----------------------------------------------------------------------

    /// Generate a synthetic Langmuir probe I-V curve.
    fn generate_iv_curve(
        te: f64,
        v_p: f64,
        i_sat: f64,
        i_e0: f64,
        n_points: usize,
        v_min: f64,
        v_max: f64,
    ) -> (Vec<f64>, Vec<f64>) {
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
        (voltage, current)
    }

    #[test]
    fn test_iv_curve_synthetic() {
        let te = 5.0;
        let v_p = 10.0;
        let i_sat = -0.001;
        let i_e0 = 0.01;

        let (voltage, current) = generate_iv_curve(te, v_p, i_sat, i_e0, 200, -30.0, 20.0);

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let params = processor.analyze_iv_curve(&voltage, &current);

        let expected_vf = v_p + te * ((-i_sat) / i_e0).ln();
        assert!(
            approx_eq(params.floating_potential_v, expected_vf, 0.05),
            "V_float={}, expected={}",
            params.floating_potential_v, expected_vf
        );

        assert!(
            approx_eq(params.electron_temp_ev, te, 0.15),
            "T_e={}, expected={}", params.electron_temp_ev, te
        );

        assert!(
            (params.plasma_potential_v - v_p).abs() < 2.0,
            "V_p={}, expected={}", params.plasma_potential_v, v_p
        );

        assert!(params.electron_density_m3 > 0.0);
        assert!(params.debye_length_m > 0.0);
    }

    #[test]
    fn test_iv_curve_cold_plasma() {
        let te = 1.0;
        let v_p = 5.0;
        let i_sat = -0.0005;
        let i_e0 = 0.005;

        let (voltage, current) = generate_iv_curve(te, v_p, i_sat, i_e0, 300, -20.0, 10.0);

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let params = processor.analyze_iv_curve(&voltage, &current);

        assert!(
            approx_eq(params.electron_temp_ev, te, 0.2),
            "T_e={}, expected={}", params.electron_temp_ev, te
        );
    }

    #[test]
    fn test_electron_density_from_saturation_basic() {
        let i_sat = 1e-3;
        let te = 10.0;
        let area = 1e-6;
        let m_d = 2.0 * AMU;
        let ne = electron_density_from_saturation(i_sat, te, area, m_d);
        assert!(ne > 1e16 && ne < 1e20, "ne = {ne}");
    }

    #[test]
    fn test_electron_density_zero_area() {
        assert_eq!(electron_density_from_saturation(1e-3, 10.0, 0.0, 2.0 * AMU), 0.0);
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

    // -----------------------------------------------------------------------
    // EEDF (Druyvesteyn method)
    // -----------------------------------------------------------------------

    #[test]
    fn test_eedf_basic() {
        let te = 5.0;
        let v_p = 10.0;
        let i_sat = -0.001;
        let i_e0 = 0.01;

        let (voltage, current) = generate_iv_curve(te, v_p, i_sat, i_e0, 500, -30.0, 15.0);

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let eedf = processor.compute_eedf(&voltage, &current, v_p, 5);

        assert!(!eedf.energy_ev.is_empty(), "EEDF should have energy values");
        assert!(!eedf.eedf.is_empty(), "EEDF should have distribution values");
        // All EEDF values should be non-negative
        for &f in &eedf.eedf {
            assert!(f >= 0.0, "EEDF value should be non-negative, got {f}");
        }
    }

    #[test]
    fn test_eedf_mean_energy_positive() {
        let te = 10.0;
        let v_p = 15.0;
        let i_sat = -0.002;
        let i_e0 = 0.02;

        let (voltage, current) = generate_iv_curve(te, v_p, i_sat, i_e0, 500, -40.0, 20.0);

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let eedf = processor.compute_eedf(&voltage, &current, v_p, 3);

        assert!(eedf.mean_energy_ev >= 0.0, "mean energy = {}", eedf.mean_energy_ev);
    }

    #[test]
    fn test_numerical_second_derivative() {
        // f(x) = x^2 => f''(x) = 2
        let x: Vec<f64> = (0..11).map(|i| i as f64 * 0.1).collect();
        let y: Vec<f64> = x.iter().map(|xi| xi * xi).collect();
        let d2 = numerical_second_derivative(&x, &y);
        for &val in &d2 {
            assert!(approx_eq(val, 2.0, 0.01), "d2={val}, expected=2.0");
        }
    }

    // -----------------------------------------------------------------------
    // Microwave interferometry
    // -----------------------------------------------------------------------

    #[test]
    fn test_interferometry_phase_shift_positive() {
        let config = PlasmaConfig {
            microwave_freq_ghz: 60.0,
            ..PlasmaConfig::default()
        };
        let processor = PlasmaProcessor::new(config);
        // Line-integrated density of 1e19 m^-2
        let phi = processor.interferometry_phase_shift(1e19);
        assert!(phi > 0.0, "phase shift = {phi}");
    }

    #[test]
    fn test_interferometry_roundtrip() {
        let config = PlasmaConfig {
            microwave_freq_ghz: 140.0,
            ..PlasmaConfig::default()
        };
        let processor = PlasmaProcessor::new(config);
        let nel = 5e19; // line-integrated density
        let phi = processor.interferometry_phase_shift(nel);
        let nel_recovered = processor.interferometry_density_from_phase(phi);
        assert!(
            approx_eq(nel_recovered, nel, 1e-10),
            "nel={nel}, recovered={nel_recovered}"
        );
    }

    #[test]
    fn test_interferometry_density_per_fringe() {
        let config = PlasmaConfig {
            microwave_freq_ghz: 60.0,
            ..PlasmaConfig::default()
        };
        let processor = PlasmaProcessor::new(config);
        let dpf = processor.interferometry_density_per_fringe();
        // Should be a large positive number
        assert!(dpf > 0.0, "density per fringe = {dpf}");
        // At 60 GHz, fringe corresponds to ~few 1e19 m^-2
        assert!(dpf > 1e17 && dpf < 1e21, "dpf = {dpf}");
    }

    // -----------------------------------------------------------------------
    // Boltzmann plot (OES)
    // -----------------------------------------------------------------------

    #[test]
    fn test_boltzmann_plot_known_temperature() {
        // Create synthetic spectral lines with known T_exc = 2 eV
        let t_exc = 2.0; // eV
        let lines: Vec<SpectralLine> = (0..5)
            .map(|i| {
                let e_upper = 10.0 + i as f64 * 1.0;
                let g = 3.0;
                let a = 1e8;
                let lambda = 500.0 + i as f64 * 20.0;
                // From Boltzmann: I*lambda/(g*A) = C * exp(-E/T)
                let intensity = g * a / lambda * (-e_upper / t_exc).exp() * 1e6;
                SpectralLine {
                    intensity,
                    wavelength_nm: lambda,
                    g_upper: g,
                    a_coefficient: a,
                    e_upper_ev: e_upper,
                }
            })
            .collect();

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let result = processor.boltzmann_plot(&lines);

        assert!(
            approx_eq(result.excitation_temp_ev, t_exc, 0.01),
            "T_exc={}, expected={}", result.excitation_temp_ev, t_exc
        );
        assert!(result.r_squared > 0.99, "R^2 = {}", result.r_squared);
    }

    #[test]
    fn test_boltzmann_plot_two_lines() {
        // Use physically consistent data: higher energy -> lower intensity
        let lines = vec![
            SpectralLine {
                intensity: 1000.0,
                wavelength_nm: 400.0,
                g_upper: 5.0,
                a_coefficient: 1e7,
                e_upper_ev: 12.0,
            },
            SpectralLine {
                intensity: 10.0,
                wavelength_nm: 400.0,
                g_upper: 5.0,
                a_coefficient: 1e7,
                e_upper_ev: 14.0,
            },
        ];

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let result = processor.boltzmann_plot(&lines);
        assert!(result.excitation_temp_ev > 0.0, "T_exc = {}", result.excitation_temp_ev);
    }

    // -----------------------------------------------------------------------
    // Line ratio method
    // -----------------------------------------------------------------------

    #[test]
    fn test_line_ratio_temperature() {
        let t_e: f64 = 3.0; // eV
        let g: f64 = 5.0;
        let a: f64 = 1e8;
        let lambda: f64 = 500.0;
        let e1: f64 = 12.0;
        let e2: f64 = 14.0;

        // Construct lines consistent with T_e = 3 eV
        let i1 = g * a / lambda * (-e1 / t_e).exp();
        let i2 = g * a / lambda * (-e2 / t_e).exp();

        let line1 = SpectralLine {
            intensity: i1,
            wavelength_nm: lambda,
            g_upper: g,
            a_coefficient: a,
            e_upper_ev: e1,
        };
        let line2 = SpectralLine {
            intensity: i2,
            wavelength_nm: lambda,
            g_upper: g,
            a_coefficient: a,
            e_upper_ev: e2,
        };

        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);
        let te_result = processor.line_ratio_temperature(&line1, &line2);

        assert!(
            approx_eq(te_result, t_e, 0.01),
            "T_e={te_result}, expected={t_e}"
        );
    }

    // -----------------------------------------------------------------------
    // Stark broadening
    // -----------------------------------------------------------------------

    #[test]
    fn test_stark_broadening_density() {
        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);

        // H-beta Stark coefficient ~ 2.5e-16 nm / (m^-3)^(2/3)
        let c_stark = 2.5e-16;
        let ne_true: f64 = 1e22; // m^-3
        let delta_lambda = c_stark * ne_true.powf(2.0 / 3.0);

        let ne_est = processor.stark_broadening_density(delta_lambda, c_stark);
        assert!(
            approx_eq(ne_est, ne_true, 0.01),
            "ne_est={ne_est}, ne_true={ne_true}"
        );
    }

    #[test]
    fn test_stark_broadening_free_fn() {
        let c = 2.5e-16;
        let ne: f64 = 1e21;
        let dl = c * ne.powf(2.0 / 3.0);
        let ne_back = stark_broadening_ne(dl, c);
        assert!(approx_eq(ne_back, ne, 0.01));
    }

    #[test]
    fn test_stark_broadening_zero() {
        assert_eq!(stark_broadening_ne(0.0, 1e-16), 0.0);
        assert_eq!(stark_broadening_ne(0.1, 0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // Thomson scattering
    // -----------------------------------------------------------------------

    #[test]
    fn test_thomson_spectral_width() {
        // At 10 eV, 1064 nm, 90 deg
        let w = thomson_spectral_width(1064.0, 10.0, 90.0);
        assert!(w > 0.0, "width = {w}");
        // Rough check: width ~ few nm for 10 eV
        assert!(w > 0.1 && w < 100.0, "width = {w} nm");
    }

    #[test]
    fn test_thomson_width_increases_with_temperature() {
        let w1 = thomson_spectral_width(1064.0, 1.0, 90.0);
        let w2 = thomson_spectral_width(1064.0, 100.0, 90.0);
        assert!(w2 > w1);
    }

    #[test]
    fn test_thomson_scattering_roundtrip() {
        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);

        let te_input = 50.0; // eV
        let wavelength = 1064.0; // nm
        let angle = 90.0; // degrees

        // Compute expected spectral width
        let width = thomson_spectral_width(wavelength, te_input, angle);

        // Use width to recover Te
        let result = processor.analyze_thomson_scattering(
            wavelength,
            angle,
            width,
            1e16,
            1.0,
        );

        assert!(
            approx_eq(result.electron_temp_ev, te_input, 0.01),
            "Te={}, expected={}", result.electron_temp_ev, te_input
        );
    }

    #[test]
    fn test_thomson_ne_from_power() {
        let config = PlasmaConfig::default();
        let processor = PlasmaProcessor::new(config);

        let cal_factor = 1e15; // m^-3 per unit power
        let power = 5.0;
        let result = processor.analyze_thomson_scattering(
            1064.0, 90.0, 1.0, power, cal_factor,
        );

        assert!(
            approx_eq(result.electron_density_m3, power * cal_factor, 1e-10),
            "ne={}", result.electron_density_m3
        );
    }

    // -----------------------------------------------------------------------
    // Config
    // -----------------------------------------------------------------------

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
        assert!(approx_eq(m_d, 2.0 * AMU, 1e-10));
    }

    // -----------------------------------------------------------------------
    // Smoothing helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_smooth_signal_identity() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let smoothed = smooth_signal(&data, 1);
        for (a, b) in data.iter().zip(smoothed.iter()) {
            assert!(approx_eq(*a, *b, 1e-10));
        }
    }

    #[test]
    fn test_smooth_signal_averaging() {
        let data = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let smoothed = smooth_signal(&data, 3);
        // Center value (index 2) should be average of [0, 10, 0] = 3.33
        assert!(approx_eq(smoothed[2], 10.0 / 3.0, 0.01));
    }

    // -----------------------------------------------------------------------
    // Linear regression
    // -----------------------------------------------------------------------

    #[test]
    fn test_linear_regression_perfect_fit() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0]; // y = 1 + 2x
        let (slope, intercept, r_sq) = linear_regression(&x, &y);
        assert!(approx_eq(slope, 2.0, 1e-10));
        assert!(approx_eq(intercept, 1.0, 1e-10));
        assert!(approx_eq(r_sq, 1.0, 1e-10));
    }
}
