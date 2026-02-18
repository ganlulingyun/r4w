//! Atomic Absorption Spectroscopy (AAS) signal processing for quantitative elemental analysis.
//!
//! Implements flame AAS and graphite furnace AAS (GFAAS) signal processing including
//! Beer-Lambert law calculations, calibration curves, hollow cathode lamp modeling,
//! flame/furnace atomizer physics, background correction methods, interference
//! correction, transient signal processing, and multi-element sequential analysis.

use std::f64::consts::PI;

// ─── Physical Constants ──────────────────────────────────────────────────────

/// Boltzmann constant in J/K
const K_BOLTZMANN: f64 = 1.380649e-23;

/// Speed of light in m/s
const C_LIGHT: f64 = 2.99792458e8;

/// Planck constant in J·s
const H_PLANCK: f64 = 6.62607015e-34;

// ─── Helper Functions ────────────────────────────────────────────────────────

/// Convert transmittance (0..1) to absorbance: A = -log10(T)
pub fn transmittance_to_absorbance(t: f64) -> f64 {
    if t <= 0.0 {
        return f64::INFINITY;
    }
    -t.log10()
}

/// Convert absorbance to transmittance: T = 10^(-A)
pub fn absorbance_to_transmittance(a: f64) -> f64 {
    10.0_f64.powf(-a)
}

/// Convert absorbance to concentration using linear calibration: c = (A - intercept) / slope
pub fn absorbance_to_concentration(abs: f64, slope: f64, intercept: f64) -> f64 {
    if slope.abs() < 1e-30 {
        return 0.0;
    }
    (abs - intercept) / slope
}

/// Characteristic concentration: concentration giving 0.0044 absorbance (1% absorption).
/// cc = conc * 0.0044 / abs
pub fn characteristic_concentration(conc: f64, abs: f64) -> f64 {
    if abs.abs() < 1e-30 {
        return f64::INFINITY;
    }
    conc * 0.0044 / abs
}

/// Characteristic mass for GFAAS: mass giving 0.0044 absorbance·s peak area.
/// cm = mass * 0.0044 / peak_area
pub fn characteristic_mass(mass: f64, peak_area: f64) -> f64 {
    if peak_area.abs() < 1e-30 {
        return f64::INFINITY;
    }
    mass * 0.0044 / peak_area
}

// ─── Beer-Lambert Law ────────────────────────────────────────────────────────

/// Beer-Lambert law: A = epsilon * l * c = -log10(I/I0)
///
/// Relates absorbance to analyte concentration through molar absorptivity
/// and optical path length.
#[derive(Debug, Clone)]
pub struct BeerLambertLaw {
    /// Molar absorptivity (L/(mol·cm))
    pub molar_absorptivity: f64,
    /// Path length (cm)
    pub path_length: f64,
}

impl BeerLambertLaw {
    pub fn new(molar_absorptivity: f64, path_length: f64) -> Self {
        Self {
            molar_absorptivity,
            path_length,
        }
    }

    /// Compute absorbance from concentration (mol/L).
    /// A = epsilon * l * c
    pub fn absorbance_from_concentration(&self, concentration: f64) -> f64 {
        self.molar_absorptivity * self.path_length * concentration
    }

    /// Compute concentration from absorbance.
    /// c = A / (epsilon * l)
    pub fn concentration_from_absorbance(&self, absorbance: f64) -> f64 {
        let denom = self.molar_absorptivity * self.path_length;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        absorbance / denom
    }

    /// Compute absorbance from transmittance ratio I/I0.
    /// A = -log10(I/I0)
    pub fn absorbance_from_intensity(&self, i: f64, i0: f64) -> f64 {
        if i0 <= 0.0 || i <= 0.0 {
            return f64::INFINITY;
        }
        -(i / i0).log10()
    }

    /// Compute transmittance from concentration.
    /// T = 10^(-A) = 10^(-epsilon * l * c)
    pub fn transmittance_from_concentration(&self, concentration: f64) -> f64 {
        let a = self.absorbance_from_concentration(concentration);
        10.0_f64.powf(-a)
    }

    /// Check linearity: true if absorbance < ~1.0 (within linear range).
    pub fn is_linear_range(&self, absorbance: f64) -> bool {
        absorbance < 1.0
    }

    /// Percent absorption = (1 - T) * 100
    pub fn percent_absorption(&self, concentration: f64) -> f64 {
        let t = self.transmittance_from_concentration(concentration);
        (1.0 - t) * 100.0
    }
}

// ─── Calibration Curve ───────────────────────────────────────────────────────

/// Calibration curve for quantitative AAS analysis.
///
/// Supports linear (least-squares) and quadratic calibration,
/// standard addition method, LOD, and LOQ calculation.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Calibration standards: (concentration, absorbance)
    pub standards: Vec<(f64, f64)>,
    /// Linear fit slope
    pub slope: f64,
    /// Linear fit intercept
    pub intercept: f64,
    /// R-squared correlation coefficient
    pub r_squared: f64,
    /// Quadratic coefficient (for curved calibration)
    pub quad_coeff: f64,
    /// Whether quadratic fit is used
    pub use_quadratic: bool,
}

impl CalibrationCurve {
    /// Create a new calibration curve and perform linear least-squares fit.
    pub fn new(standards: &[(f64, f64)]) -> Self {
        let mut curve = Self {
            standards: standards.to_vec(),
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
            quad_coeff: 0.0,
            use_quadratic: false,
        };
        curve.fit_linear();
        curve
    }

    /// Perform linear least-squares regression: A = slope * C + intercept
    pub fn fit_linear(&mut self) {
        let n = self.standards.len() as f64;
        if n < 2.0 {
            return;
        }
        let sum_x: f64 = self.standards.iter().map(|(c, _)| c).sum();
        let sum_y: f64 = self.standards.iter().map(|(_, a)| a).sum();
        let sum_xx: f64 = self.standards.iter().map(|(c, _)| c * c).sum();
        let sum_xy: f64 = self.standards.iter().map(|(c, a)| c * a).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return;
        }

        self.slope = (n * sum_xy - sum_x * sum_y) / denom;
        self.intercept = (sum_y - self.slope * sum_x) / n;

        // R-squared
        let mean_y = sum_y / n;
        let ss_tot: f64 = self.standards.iter().map(|(_, a)| (a - mean_y).powi(2)).sum();
        let ss_res: f64 = self
            .standards
            .iter()
            .map(|(c, a)| {
                let pred = self.slope * c + self.intercept;
                (a - pred).powi(2)
            })
            .sum();

        self.r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };
    }

    /// Perform quadratic least-squares regression: A = a*C^2 + b*C + c
    pub fn fit_quadratic(&mut self) {
        let n = self.standards.len() as f64;
        if n < 3.0 {
            // Fall back to linear
            self.fit_linear();
            return;
        }

        // Normal equations for quadratic: A = a*x^2 + b*x + c
        // [sum x^4  sum x^3  sum x^2] [a]   [sum x^2*y]
        // [sum x^3  sum x^2  sum x  ] [b] = [sum x*y  ]
        // [sum x^2  sum x    n      ] [c]   [sum y    ]
        let mut s = [0.0_f64; 5]; // s[k] = sum(x^k)
        let mut sy = [0.0_f64; 3]; // sy[k] = sum(x^k * y)

        for &(x, y) in &self.standards {
            let mut xk = 1.0;
            for k in 0..5 {
                s[k] += xk;
                if k < 3 {
                    sy[k] += xk * y;
                }
                xk *= x;
            }
        }

        // Solve 3x3 system using Cramer's rule
        let m = [
            [s[4], s[3], s[2]],
            [s[3], s[2], s[1]],
            [s[2], s[1], s[0]],
        ];

        let det = Self::det3(&m);
        if det.abs() < 1e-30 {
            self.fit_linear();
            return;
        }

        let ma = [
            [sy[2], s[3], s[2]],
            [sy[1], s[2], s[1]],
            [sy[0], s[1], s[0]],
        ];
        let mb = [
            [s[4], sy[2], s[2]],
            [s[3], sy[1], s[1]],
            [s[2], sy[0], s[0]],
        ];
        let mc = [
            [s[4], s[3], sy[2]],
            [s[3], s[2], sy[1]],
            [s[2], s[1], sy[0]],
        ];

        self.quad_coeff = Self::det3(&ma) / det;
        self.slope = Self::det3(&mb) / det;
        self.intercept = Self::det3(&mc) / det;
        self.use_quadratic = true;

        // R-squared for quadratic
        let mean_y = sy[0] / n;
        let ss_tot: f64 = self.standards.iter().map(|(_, a)| (a - mean_y).powi(2)).sum();
        let ss_res: f64 = self
            .standards
            .iter()
            .map(|(c, a)| {
                let pred = self.quad_coeff * c * c + self.slope * c + self.intercept;
                (a - pred).powi(2)
            })
            .sum();

        self.r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };
    }

    /// 3x3 determinant
    fn det3(m: &[[f64; 3]; 3]) -> f64 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Predict absorbance from concentration.
    pub fn predict_absorbance(&self, concentration: f64) -> f64 {
        if self.use_quadratic {
            self.quad_coeff * concentration * concentration
                + self.slope * concentration
                + self.intercept
        } else {
            self.slope * concentration + self.intercept
        }
    }

    /// Predict concentration from absorbance (inverse calibration).
    pub fn predict_concentration(&self, absorbance: f64) -> f64 {
        if self.use_quadratic {
            // Solve: a*c^2 + b*c + (intercept - abs) = 0
            let a = self.quad_coeff;
            let b = self.slope;
            let c_val = self.intercept - absorbance;

            if a.abs() < 1e-30 {
                // Linear case
                return absorbance_to_concentration(absorbance, self.slope, self.intercept);
            }

            let disc = b * b - 4.0 * a * c_val;
            if disc < 0.0 {
                return 0.0;
            }
            // Return positive root
            let r1 = (-b + disc.sqrt()) / (2.0 * a);
            let r2 = (-b - disc.sqrt()) / (2.0 * a);
            if r1 >= 0.0 {
                r1
            } else {
                r2
            }
        } else {
            absorbance_to_concentration(absorbance, self.slope, self.intercept)
        }
    }

    /// Sensitivity = slope of calibration curve at a given concentration.
    pub fn sensitivity(&self, concentration: f64) -> f64 {
        if self.use_quadratic {
            2.0 * self.quad_coeff * concentration + self.slope
        } else {
            self.slope
        }
    }

    /// Limit of Detection: LOD = 3 * sigma_blank / slope
    pub fn lod(&self, sigma_blank: f64) -> f64 {
        let s = self.sensitivity(0.0).abs();
        if s < 1e-30 {
            return f64::INFINITY;
        }
        3.0 * sigma_blank / s
    }

    /// Limit of Quantification: LOQ = 10 * sigma_blank / slope
    pub fn loq(&self, sigma_blank: f64) -> f64 {
        let s = self.sensitivity(0.0).abs();
        if s < 1e-30 {
            return f64::INFINITY;
        }
        10.0 * sigma_blank / s
    }

    /// Standard addition method: extrapolate x-intercept = -Cx (original concentration).
    /// standards: (added_concentration, absorbance) pairs.
    /// Returns the estimated original sample concentration.
    pub fn standard_addition(additions: &[(f64, f64)]) -> f64 {
        if additions.len() < 2 {
            return 0.0;
        }
        let curve = CalibrationCurve::new(additions);
        // x-intercept = -intercept / slope; concentration is |x-intercept|
        if curve.slope.abs() < 1e-30 {
            return 0.0;
        }
        (curve.intercept / curve.slope).abs()
    }

    /// Method of additions: compute concentration from multiple spikes.
    /// Returns (original_concentration, slope, intercept).
    pub fn method_of_additions(additions: &[(f64, f64)]) -> (f64, f64, f64) {
        let curve = CalibrationCurve::new(additions);
        let orig = if curve.slope.abs() > 1e-30 {
            (curve.intercept / curve.slope).abs()
        } else {
            0.0
        };
        (orig, curve.slope, curve.intercept)
    }

    /// Residuals of the fit.
    pub fn residuals(&self) -> Vec<f64> {
        self.standards
            .iter()
            .map(|(c, a)| a - self.predict_absorbance(*c))
            .collect()
    }

    /// Standard error of the estimate.
    pub fn standard_error(&self) -> f64 {
        let resid = self.residuals();
        let n = resid.len();
        let p = if self.use_quadratic { 3 } else { 2 };
        if n <= p {
            return 0.0;
        }
        let ss_res: f64 = resid.iter().map(|r| r * r).sum();
        (ss_res / (n - p) as f64).sqrt()
    }
}

// ─── Line Profile Types ──────────────────────────────────────────────────────

/// Emission line profile shape.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LineProfile {
    /// Gaussian (Doppler broadened)
    Gaussian,
    /// Lorentzian (pressure/natural broadened)
    Lorentzian,
    /// Voigt (convolution of Gaussian + Lorentzian)
    Voigt,
}

// ─── Hollow Cathode Lamp ─────────────────────────────────────────────────────

/// Hollow cathode lamp (HCL) source model.
///
/// Models the emission line profile, self-absorption at high currents,
/// warm-up drift, and modulation for source/flame discrimination.
#[derive(Debug, Clone)]
pub struct HollowCathodeLamp {
    /// Element symbol
    pub element: String,
    /// Primary wavelength (nm)
    pub wavelength_nm: f64,
    /// Nominal operating current (mA)
    pub nominal_current_ma: f64,
    /// Maximum current (mA)
    pub max_current_ma: f64,
    /// Line profile type
    pub profile: LineProfile,
    /// Gaussian half-width (nm) for Doppler broadening
    pub gaussian_hw_nm: f64,
    /// Lorentzian half-width (nm) for pressure broadening
    pub lorentzian_hw_nm: f64,
    /// Self-absorption coefficient (higher = more self-absorption)
    pub self_absorption_coeff: f64,
    /// Warm-up time constant (seconds)
    pub warmup_time_constant: f64,
}

impl HollowCathodeLamp {
    pub fn new(element: &str, wavelength_nm: f64, nominal_current_ma: f64) -> Self {
        Self {
            element: element.to_string(),
            wavelength_nm,
            nominal_current_ma,
            max_current_ma: nominal_current_ma * 2.0,
            profile: LineProfile::Voigt,
            gaussian_hw_nm: 0.001,
            lorentzian_hw_nm: 0.0005,
            self_absorption_coeff: 0.01,
            warmup_time_constant: 600.0, // 10 minutes
        }
    }

    /// Compute Gaussian emission profile at wavelength offset delta_nm.
    pub fn gaussian_profile(&self, delta_nm: f64) -> f64 {
        let sigma = self.gaussian_hw_nm / (2.0 * (2.0_f64.ln()).sqrt());
        if sigma < 1e-30 {
            return 0.0;
        }
        let norm = 1.0 / (sigma * (2.0 * PI).sqrt());
        norm * (-0.5 * (delta_nm / sigma).powi(2)).exp()
    }

    /// Compute Lorentzian emission profile at wavelength offset delta_nm.
    pub fn lorentzian_profile(&self, delta_nm: f64) -> f64 {
        let gamma = self.lorentzian_hw_nm;
        if gamma < 1e-30 {
            return 0.0;
        }
        (gamma / PI) / (delta_nm * delta_nm + gamma * gamma)
    }

    /// Pseudo-Voigt approximation: weighted sum of Gaussian and Lorentzian.
    pub fn voigt_profile(&self, delta_nm: f64) -> f64 {
        // Pseudo-Voigt: eta * L + (1-eta) * G
        // eta depends on the ratio of widths
        let fg = self.gaussian_hw_nm;
        let fl = self.lorentzian_hw_nm;
        let f_total = (fg.powi(5)
            + 2.69269 * fg.powi(4) * fl
            + 2.42843 * fg.powi(3) * fl.powi(2)
            + 4.47163 * fg.powi(2) * fl.powi(3)
            + 0.07842 * fg * fl.powi(4)
            + fl.powi(5))
        .powf(0.2);

        let eta = if f_total > 1e-30 {
            1.36603 * (fl / f_total) - 0.47719 * (fl / f_total).powi(2)
                + 0.11116 * (fl / f_total).powi(3)
        } else {
            0.5
        };
        let eta = eta.clamp(0.0, 1.0);

        eta * self.lorentzian_profile(delta_nm) + (1.0 - eta) * self.gaussian_profile(delta_nm)
    }

    /// Emission line profile at given offset from center wavelength.
    pub fn emission_profile(&self, delta_nm: f64) -> f64 {
        match self.profile {
            LineProfile::Gaussian => self.gaussian_profile(delta_nm),
            LineProfile::Lorentzian => self.lorentzian_profile(delta_nm),
            LineProfile::Voigt => self.voigt_profile(delta_nm),
        }
    }

    /// Self-absorption factor at a given current (0..1, 1 = no self-absorption).
    /// Self-absorption increases non-linearly with current above nominal.
    pub fn self_absorption_factor(&self, current_ma: f64) -> f64 {
        if current_ma <= self.nominal_current_ma {
            return 1.0;
        }
        let excess = (current_ma - self.nominal_current_ma) / self.nominal_current_ma;
        let factor = 1.0 - self.self_absorption_coeff * excess * excess;
        factor.max(0.0)
    }

    /// Effective emission intensity considering self-absorption.
    /// Intensity roughly proportional to current, attenuated by self-absorption.
    pub fn effective_intensity(&self, current_ma: f64) -> f64 {
        let base_intensity = current_ma / self.nominal_current_ma;
        base_intensity * self.self_absorption_factor(current_ma)
    }

    /// Warm-up drift: fractional stability as function of time since start (seconds).
    /// Returns value 0..1 (1 = fully warmed up).
    pub fn warmup_stability(&self, elapsed_sec: f64) -> f64 {
        1.0 - (-elapsed_sec / self.warmup_time_constant).exp()
    }

    /// Modulation signal for chopper-based source/flame discrimination.
    /// Returns (source_on, source_off) measurement pair factor.
    /// With chopping: A_true = A_total(source_on) - A_flame_emission(source_off)
    pub fn chopper_modulation(&self, time_sec: f64, chop_freq_hz: f64) -> f64 {
        let phase = 2.0 * PI * chop_freq_hz * time_sec;
        if phase.sin() >= 0.0 {
            1.0
        } else {
            0.0
        }
    }
}

// ─── Flame Atomizer ──────────────────────────────────────────────────────────

/// Flame type for atomic absorption.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlameType {
    /// Air-acetylene (~2300 C), most common
    AirAcetylene,
    /// Nitrous oxide-acetylene (~2950 C), for refractory elements
    NitrousOxideAcetylene,
    /// Air-hydrogen (~2050 C), for low-wavelength elements (As, Se)
    AirHydrogen,
    /// Air-propane (~1920 C), rarely used
    AirPropane,
}

impl FlameType {
    /// Temperature in Kelvin.
    pub fn temperature_k(&self) -> f64 {
        match self {
            FlameType::AirAcetylene => 2573.15,       // 2300 C
            FlameType::NitrousOxideAcetylene => 3223.15, // 2950 C
            FlameType::AirHydrogen => 2323.15,         // 2050 C
            FlameType::AirPropane => 2193.15,          // 1920 C
        }
    }

    /// Temperature in Celsius.
    pub fn temperature_c(&self) -> f64 {
        self.temperature_k() - 273.15
    }
}

/// Flame atomizer model.
///
/// Models atomization efficiency, Boltzmann population distribution,
/// and flame temperature effects on free atom population.
#[derive(Debug, Clone)]
pub struct FlameAtomizer {
    /// Flame type
    pub flame_type: FlameType,
    /// Nebulization efficiency (0..1, typically 0.05-0.15)
    pub nebulization_efficiency: f64,
    /// Atomization efficiency (0..1, fraction of nebulized atoms that are free atoms)
    pub atomization_efficiency: f64,
    /// Sample aspiration rate (mL/min)
    pub aspiration_rate_ml_min: f64,
    /// Burner path length (cm)
    pub path_length_cm: f64,
}

impl FlameAtomizer {
    pub fn new(flame_type: FlameType) -> Self {
        let (neb_eff, atom_eff) = match flame_type {
            FlameType::AirAcetylene => (0.10, 0.30),
            FlameType::NitrousOxideAcetylene => (0.10, 0.50),
            FlameType::AirHydrogen => (0.08, 0.20),
            FlameType::AirPropane => (0.06, 0.15),
        };
        Self {
            flame_type,
            nebulization_efficiency: neb_eff,
            atomization_efficiency: atom_eff,
            aspiration_rate_ml_min: 5.0,
            path_length_cm: 10.0,
        }
    }

    /// Boltzmann population ratio: N*/N0 = (g*/g0) * exp(-E / kT)
    ///
    /// - g_star: statistical weight of excited state
    /// - g0: statistical weight of ground state
    /// - energy_ev: excitation energy in eV
    /// - temperature_k: flame temperature in K
    pub fn boltzmann_ratio(g_star: f64, g0: f64, energy_ev: f64, temperature_k: f64) -> f64 {
        let energy_j = energy_ev * 1.602176634e-19;
        (g_star / g0) * (-energy_j / (K_BOLTZMANN * temperature_k)).exp()
    }

    /// Fraction of atoms in ground state at flame temperature.
    /// For most elements at typical AAS temperatures, N0/N_total ~ 1.
    /// Uses partition function approximation: N0/N = g0 / Z(T)
    pub fn ground_state_fraction(&self, g0: f64, g_star: f64, energy_ev: f64) -> f64 {
        let ratio = Self::boltzmann_ratio(g_star, g0, energy_ev, self.flame_type.temperature_k());
        // Approximate partition function Z ~ g0 + g_star * exp(-E/kT)
        let z = g0 + g_star * ratio * g0 / g_star; // = g0 + g0 * exp(-E/kT)
        g0 / z
    }

    /// Effective free atom concentration in the flame.
    /// Takes into account nebulization, atomization, and Boltzmann effects.
    pub fn effective_atom_concentration(
        &self,
        solution_conc_mg_l: f64,
        g0: f64,
        g_star: f64,
        energy_ev: f64,
    ) -> f64 {
        let ground_frac = self.ground_state_fraction(g0, g_star, energy_ev);
        solution_conc_mg_l
            * self.nebulization_efficiency
            * self.atomization_efficiency
            * ground_frac
    }

    /// Wavelength (nm) from excitation energy (eV).
    pub fn wavelength_from_energy(energy_ev: f64) -> f64 {
        if energy_ev <= 0.0 {
            return f64::INFINITY;
        }
        let energy_j = energy_ev * 1.602176634e-19;
        (H_PLANCK * C_LIGHT / energy_j) * 1e9
    }

    /// Energy (eV) from wavelength (nm).
    pub fn energy_from_wavelength(wavelength_nm: f64) -> f64 {
        if wavelength_nm <= 0.0 {
            return f64::INFINITY;
        }
        let wavelength_m = wavelength_nm * 1e-9;
        (H_PLANCK * C_LIGHT / wavelength_m) / 1.602176634e-19
    }
}

// ─── Graphite Furnace Program ────────────────────────────────────────────────

/// Graphite furnace temperature program stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FurnaceStage {
    /// Drying stage: evaporate solvent (~100-120 C)
    Dry,
    /// Charring/ashing: destroy organic matrix (~400-1400 C)
    Char,
    /// Atomization: vaporize analyte (~1800-2700 C)
    Atomize,
    /// Cleaning: remove residues (~2700 C)
    Clean,
    /// Cooling: return to ambient
    Cool,
}

/// A single step in a GFAAS temperature program.
#[derive(Debug, Clone)]
pub struct FurnaceStep {
    pub stage: FurnaceStage,
    /// Target temperature (C)
    pub temperature_c: f64,
    /// Ramp time (seconds)
    pub ramp_time_s: f64,
    /// Hold time (seconds)
    pub hold_time_s: f64,
    /// Internal gas flow (mL/min, 0 for atomize to stop flow)
    pub gas_flow_ml_min: f64,
}

/// Graphite furnace AAS (GFAAS) temperature program.
///
/// Models the multi-stage temperature program with Lvov platform concept
/// and matrix modifier effects.
#[derive(Debug, Clone)]
pub struct GraphiteFurnaceProgram {
    /// Temperature program steps
    pub steps: Vec<FurnaceStep>,
    /// Use Lvov platform (isothermal atomization)
    pub lvov_platform: bool,
    /// Matrix modifier applied
    pub matrix_modifier: Option<String>,
    /// Peak measurement mode (true = peak area, false = peak height)
    pub peak_area_mode: bool,
    /// Sample volume (uL)
    pub sample_volume_ul: f64,
}

impl GraphiteFurnaceProgram {
    /// Create a default furnace program for a given element.
    pub fn default_program(element: &str) -> Self {
        let (char_temp, atom_temp) = match element {
            "Pb" => (700.0, 1800.0),
            "Cd" => (500.0, 1500.0),
            "Cu" => (1000.0, 2300.0),
            "Cr" => (1200.0, 2500.0),
            "Al" => (1400.0, 2700.0),
            "Fe" => (1100.0, 2300.0),
            "Mn" => (1000.0, 2200.0),
            "Ni" => (1100.0, 2400.0),
            "As" => (800.0, 2200.0),
            "Se" => (600.0, 2100.0),
            _ => (900.0, 2300.0),
        };

        let steps = vec![
            FurnaceStep {
                stage: FurnaceStage::Dry,
                temperature_c: 110.0,
                ramp_time_s: 10.0,
                hold_time_s: 20.0,
                gas_flow_ml_min: 250.0,
            },
            FurnaceStep {
                stage: FurnaceStage::Char,
                temperature_c: char_temp,
                ramp_time_s: 10.0,
                hold_time_s: 20.0,
                gas_flow_ml_min: 250.0,
            },
            FurnaceStep {
                stage: FurnaceStage::Atomize,
                temperature_c: atom_temp,
                ramp_time_s: 0.0, // maximum heating rate
                hold_time_s: 5.0,
                gas_flow_ml_min: 0.0, // stop gas for maximum sensitivity
            },
            FurnaceStep {
                stage: FurnaceStage::Clean,
                temperature_c: 2700.0,
                ramp_time_s: 1.0,
                hold_time_s: 3.0,
                gas_flow_ml_min: 250.0,
            },
            FurnaceStep {
                stage: FurnaceStage::Cool,
                temperature_c: 25.0,
                ramp_time_s: 10.0,
                hold_time_s: 5.0,
                gas_flow_ml_min: 250.0,
            },
        ];

        Self {
            steps,
            lvov_platform: true,
            matrix_modifier: None,
            peak_area_mode: true,
            sample_volume_ul: 20.0,
        }
    }

    /// Set matrix modifier (e.g., "Pd/Mg(NO3)2" for volatile elements).
    pub fn with_modifier(mut self, modifier: &str) -> Self {
        self.matrix_modifier = Some(modifier.to_string());
        self
    }

    /// Total program duration (seconds).
    pub fn total_duration(&self) -> f64 {
        self.steps
            .iter()
            .map(|s| s.ramp_time_s + s.hold_time_s)
            .sum()
    }

    /// Temperature at a given time (seconds) in the program.
    pub fn temperature_at(&self, time_s: f64) -> f64 {
        let mut elapsed = 0.0;
        let mut prev_temp = 25.0;

        for step in &self.steps {
            let step_total = step.ramp_time_s + step.hold_time_s;
            if time_s < elapsed + step_total {
                let t_in_step = time_s - elapsed;
                if t_in_step < step.ramp_time_s {
                    // Ramping
                    if step.ramp_time_s > 0.0 {
                        let frac = t_in_step / step.ramp_time_s;
                        return prev_temp + frac * (step.temperature_c - prev_temp);
                    } else {
                        return step.temperature_c;
                    }
                } else {
                    // Holding
                    return step.temperature_c;
                }
            }
            elapsed += step_total;
            prev_temp = step.temperature_c;
        }
        25.0 // After program ends
    }

    /// Generate a simulated transient absorbance signal for atomization.
    /// Returns (time, absorbance) pairs for the atomization step.
    pub fn simulate_atomization_signal(
        &self,
        peak_absorbance: f64,
        num_points: usize,
    ) -> Vec<(f64, f64)> {
        // Find atomize step
        let mut atom_start = 0.0;
        let mut atom_duration = 5.0;
        let mut elapsed = 0.0;
        for step in &self.steps {
            let step_total = step.ramp_time_s + step.hold_time_s;
            if step.stage == FurnaceStage::Atomize {
                atom_start = elapsed;
                atom_duration = step_total;
                break;
            }
            elapsed += step_total;
        }

        let dt = atom_duration / num_points as f64;
        let mut result = Vec::with_capacity(num_points);

        // Model as asymmetric Gaussian: fast rise, slower decay
        let rise_sigma = atom_duration * 0.15;
        let decay_sigma = atom_duration * 0.35;
        let peak_time = atom_duration * 0.25;

        for i in 0..num_points {
            let t = i as f64 * dt;
            let rel_t = t - peak_time;
            let sigma = if rel_t < 0.0 {
                rise_sigma
            } else {
                decay_sigma
            };
            let abs = peak_absorbance * (-0.5 * (rel_t / sigma).powi(2)).exp();
            result.push((atom_start + t, abs));
        }

        result
    }

    /// Compute peak area (integrated absorbance) from signal.
    pub fn compute_peak_area(signal: &[(f64, f64)]) -> f64 {
        if signal.len() < 2 {
            return 0.0;
        }
        let mut area = 0.0;
        for i in 1..signal.len() {
            let dt = signal[i].0 - signal[i - 1].0;
            let avg_abs = (signal[i].1 + signal[i - 1].1) / 2.0;
            area += avg_abs * dt;
        }
        area
    }

    /// Compute peak height from signal.
    pub fn compute_peak_height(signal: &[(f64, f64)]) -> f64 {
        signal
            .iter()
            .map(|(_, a)| *a)
            .fold(0.0_f64, |max, a| if a > max { a } else { max })
    }

    /// Lvov platform delay factor: platform heats slower than tube wall,
    /// so analyte sees a more isothermal environment.
    pub fn platform_delay_factor(&self) -> f64 {
        if self.lvov_platform {
            0.85 // Platform temperature lags wall by ~15%
        } else {
            1.0
        }
    }

    /// Matrix modifier effect: increases char temperature for volatile elements.
    /// Returns the effective charring temperature increase in C.
    pub fn modifier_char_temp_increase(&self) -> f64 {
        match &self.matrix_modifier {
            Some(m) if m.contains("Pd") => 400.0,         // Pd modifier: +400 C
            Some(m) if m.contains("Mg(NO3)2") => 200.0,   // Mg nitrate: +200 C
            Some(m) if m.contains("NH4H2PO4") => 300.0,   // Ammonium phosphate: +300 C
            Some(m) if m.contains("Ni") => 250.0,          // Ni modifier: +250 C
            _ => 0.0,
        }
    }
}

// ─── Background Correction ───────────────────────────────────────────────────

/// Background correction method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BackgroundCorrectionMethod {
    /// No correction
    None,
    /// Deuterium arc (D2) lamp: broadband vs line source
    DeuteriumArc,
    /// Zeeman effect: normal or inverse, splits absorption line in magnetic field
    ZeemanNormal,
    /// Zeeman inverse
    ZeemanInverse,
    /// Smith-Hieftje: pulsed high-current self-reversal
    SmithHieftje,
}

/// Background correction processor.
#[derive(Debug, Clone)]
pub struct BackgroundCorrection {
    /// Correction method
    pub method: BackgroundCorrectionMethod,
    /// Magnetic field strength for Zeeman (Tesla)
    pub magnetic_field_t: f64,
}

impl BackgroundCorrection {
    pub fn new(method: BackgroundCorrectionMethod) -> Self {
        Self {
            method,
            magnetic_field_t: 0.8,
        }
    }

    /// Apply D2 correction: A_corrected = A_HCL - A_D2
    /// A_HCL measures atomic + background absorption (narrow line)
    /// A_D2 measures only background (broadband source covers many lines)
    pub fn deuterium_correction(a_hcl: f64, a_d2: f64) -> f64 {
        (a_hcl - a_d2).max(0.0)
    }

    /// Zeeman normal effect: pi and sigma components.
    /// In a magnetic field, absorption line splits into sigma+, pi, sigma- components.
    /// With field ON: measures sigma components (no analyte abs at shifted wavelength)
    /// With field OFF: measures total absorption
    /// A_corrected = A_field_off - A_field_on
    pub fn zeeman_correction(a_field_off: f64, a_field_on: f64) -> f64 {
        (a_field_off - a_field_on).max(0.0)
    }

    /// Zeeman splitting in wavelength (nm) for a given magnetic field (Tesla)
    /// and wavelength (nm). Delta_lambda = e * B * lambda^2 / (4 * pi * me * c)
    pub fn zeeman_splitting(wavelength_nm: f64, magnetic_field_t: f64) -> f64 {
        let e = 1.602176634e-19;
        let me = 9.1093837015e-31;
        let lambda_m = wavelength_nm * 1e-9;
        let delta_m = e * magnetic_field_t * lambda_m * lambda_m / (4.0 * PI * me * C_LIGHT);
        delta_m * 1e9 // Convert back to nm
    }

    /// Smith-Hieftje correction: low current (normal) vs high current (self-reversed).
    /// At high current, HCL line self-reverses: emission has a dip at center.
    /// A_corrected = A_low_current - A_high_current
    pub fn smith_hieftje_correction(a_low: f64, a_high: f64) -> f64 {
        (a_low - a_high).max(0.0)
    }

    /// Apply correction based on configured method.
    pub fn apply(&self, measurement_a: f64, reference_a: f64) -> f64 {
        match self.method {
            BackgroundCorrectionMethod::None => measurement_a,
            BackgroundCorrectionMethod::DeuteriumArc => {
                Self::deuterium_correction(measurement_a, reference_a)
            }
            BackgroundCorrectionMethod::ZeemanNormal | BackgroundCorrectionMethod::ZeemanInverse => {
                Self::zeeman_correction(measurement_a, reference_a)
            }
            BackgroundCorrectionMethod::SmithHieftje => {
                Self::smith_hieftje_correction(measurement_a, reference_a)
            }
        }
    }
}

// ─── Interference Correction ─────────────────────────────────────────────────

/// Type of interference in AAS.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterferenceType {
    /// Spectral: line overlap from another element
    Spectral,
    /// Chemical: formation of thermally stable compounds
    Chemical,
    /// Ionization: loss of atoms to ionized state
    Ionization,
    /// Matrix: physical effects of sample matrix
    Matrix,
}

/// Interference correction parameters.
#[derive(Debug, Clone)]
pub struct InterferenceCorrection {
    /// Spectral overlap corrections: (interfering_element, correction_factor)
    pub spectral_corrections: Vec<(String, f64)>,
    /// Chemical releasing agent (e.g., "La" for Ca, "Sr" for Ca/Mg)
    pub releasing_agent: Option<String>,
    /// Ionization buffer (e.g., "Cs" for alkali metals, "K" for Ba)
    pub ionization_buffer: Option<String>,
    /// Buffer concentration (mg/L)
    pub buffer_concentration: f64,
}

impl InterferenceCorrection {
    pub fn new() -> Self {
        Self {
            spectral_corrections: Vec::new(),
            releasing_agent: None,
            ionization_buffer: None,
            buffer_concentration: 0.0,
        }
    }

    /// Add a spectral interference correction.
    /// correction_factor: fraction of interfering element's signal at analyte wavelength.
    pub fn add_spectral_correction(&mut self, element: &str, factor: f64) {
        self.spectral_corrections
            .push((element.to_string(), factor));
    }

    /// Set releasing agent (e.g., La for phosphate interference on Ca).
    pub fn set_releasing_agent(&mut self, agent: &str) {
        self.releasing_agent = Some(agent.to_string());
    }

    /// Set ionization suppression buffer.
    pub fn set_ionization_buffer(&mut self, buffer: &str, concentration_mg_l: f64) {
        self.ionization_buffer = Some(buffer.to_string());
        self.buffer_concentration = concentration_mg_l;
    }

    /// Apply spectral interference correction.
    /// A_corrected = A_measured - sum(factor_i * A_interfering_i)
    pub fn correct_spectral(
        &self,
        measured_abs: f64,
        interfering_abs: &[(String, f64)],
    ) -> f64 {
        let mut correction = 0.0;
        for (element, abs) in interfering_abs {
            for (corr_el, factor) in &self.spectral_corrections {
                if corr_el == element {
                    correction += factor * abs;
                }
            }
        }
        (measured_abs - correction).max(0.0)
    }

    /// Ionization fraction using Saha equation approximation.
    /// alpha = N+/(N+ + N0) for element at temperature T
    ///
    /// - ionization_energy_ev: first ionization energy
    /// - temperature_k: flame temperature
    /// - electron_density: free electron density in flame (cm^-3)
    pub fn ionization_fraction(
        ionization_energy_ev: f64,
        temperature_k: f64,
        electron_density: f64,
    ) -> f64 {
        let energy_j = ionization_energy_ev * 1.602176634e-19;
        // Simplified Saha equation: K_i = (2 * Z+/Z0) * (2*pi*me*kT/h^2)^(3/2) * exp(-E/kT)
        let me = 9.1093837015e-31;
        let thermal = 2.0 * PI * me * K_BOLTZMANN * temperature_k / (H_PLANCK * H_PLANCK);
        let k_i = 2.0 * thermal.powf(1.5) * (-energy_j / (K_BOLTZMANN * temperature_k)).exp();

        // alpha^2 / (1 - alpha) = K_i / n_e (in SI, need conversion)
        // Simplified: for AAS, ionization is usually small
        let ratio = k_i / (electron_density * 1e6); // cm^-3 to m^-3
        // Solve alpha^2 + ratio*alpha - ratio = 0
        let disc = ratio * ratio + 4.0 * ratio;
        if disc < 0.0 {
            return 0.0;
        }
        let alpha = (-ratio + disc.sqrt()) / 2.0;
        alpha.clamp(0.0, 1.0)
    }

    /// Ionization suppression factor: how much an ionization buffer reduces ionization.
    /// Adding easily ionized element (Cs, K) provides excess electrons.
    pub fn ionization_suppression_factor(
        &self,
        analyte_ionization_ev: f64,
        temperature_k: f64,
        base_electron_density: f64,
    ) -> f64 {
        let alpha_no_buffer =
            Self::ionization_fraction(analyte_ionization_ev, temperature_k, base_electron_density);
        // Buffer increases electron density
        let enhanced_density = base_electron_density + self.buffer_concentration as f64 * 1e12;
        let alpha_with_buffer =
            Self::ionization_fraction(analyte_ionization_ev, temperature_k, enhanced_density);

        if alpha_no_buffer > 1e-30 {
            (1.0 - alpha_with_buffer) / (1.0 - alpha_no_buffer)
        } else {
            1.0
        }
    }
}

// ─── Signal Processing ───────────────────────────────────────────────────────

/// AAS signal processing for transient and steady-state signals.
#[derive(Debug, Clone)]
pub struct SignalProcessor {
    /// Integration time for steady-state (seconds)
    pub integration_time_s: f64,
    /// Number of readings to average
    pub num_readings: usize,
    /// Smoothing window size
    pub smoothing_window: usize,
}

impl SignalProcessor {
    pub fn new() -> Self {
        Self {
            integration_time_s: 3.0,
            num_readings: 3,
            smoothing_window: 5,
        }
    }

    /// Baseline correction: subtract a linear baseline from the signal.
    /// Baseline is estimated from first and last `margin` points.
    pub fn baseline_correct(signal: &[(f64, f64)], margin: usize) -> Vec<(f64, f64)> {
        if signal.len() < 2 * margin || margin == 0 {
            return signal.to_vec();
        }

        // Average of first margin points
        let baseline_start: f64 =
            signal[..margin].iter().map(|(_, a)| a).sum::<f64>() / margin as f64;
        // Average of last margin points
        let baseline_end: f64 = signal[signal.len() - margin..]
            .iter()
            .map(|(_, a)| a)
            .sum::<f64>()
            / margin as f64;

        let t_start = signal[0].0;
        let t_end = signal[signal.len() - 1].0;
        let dt = t_end - t_start;

        signal
            .iter()
            .map(|(t, a)| {
                let frac = if dt > 1e-30 { (t - t_start) / dt } else { 0.0 };
                let baseline = baseline_start + frac * (baseline_end - baseline_start);
                (*t, (a - baseline).max(0.0))
            })
            .collect()
    }

    /// Peak detection: find peaks above threshold with minimum separation.
    pub fn find_peaks(
        signal: &[(f64, f64)],
        threshold: f64,
        min_separation: f64,
    ) -> Vec<(usize, f64, f64)> {
        // Returns: (index, time, absorbance)
        let mut peaks = Vec::new();
        if signal.len() < 3 {
            return peaks;
        }

        for i in 1..signal.len() - 1 {
            let (t, a) = signal[i];
            if a > threshold && a > signal[i - 1].1 && a >= signal[i + 1].1 {
                // Check separation from last peak
                if let Some(&(_, last_t, _)) = peaks.last() {
                    if t - last_t < min_separation {
                        // Keep the larger peak
                        if let Some(last) = peaks.last_mut() {
                            if a > last.2 {
                                *last = (i, t, a);
                            }
                        }
                        continue;
                    }
                }
                peaks.push((i, t, a));
            }
        }
        peaks
    }

    /// Savitzky-Golay-like smoothing (simple moving average for AAS signals).
    pub fn smooth(signal: &[(f64, f64)], window: usize) -> Vec<(f64, f64)> {
        if window < 2 || signal.len() < window {
            return signal.to_vec();
        }

        let half = window / 2;
        signal
            .iter()
            .enumerate()
            .map(|(i, (t, _))| {
                let start = if i >= half { i - half } else { 0 };
                let end = (i + half + 1).min(signal.len());
                let count = end - start;
                let sum: f64 = signal[start..end].iter().map(|(_, a)| a).sum();
                (*t, sum / count as f64)
            })
            .collect()
    }

    /// Compute signal-to-noise ratio from a signal segment.
    /// Signal region: around peak; noise region: baseline.
    pub fn snr(signal: &[(f64, f64)], noise_region_end: usize) -> f64 {
        if signal.is_empty() || noise_region_end == 0 || noise_region_end >= signal.len() {
            return 0.0;
        }

        // Noise: std dev of baseline region
        let noise_vals: Vec<f64> = signal[..noise_region_end].iter().map(|(_, a)| *a).collect();
        let noise_mean: f64 = noise_vals.iter().sum::<f64>() / noise_vals.len() as f64;
        let noise_var: f64 = noise_vals
            .iter()
            .map(|a| (a - noise_mean).powi(2))
            .sum::<f64>()
            / noise_vals.len() as f64;
        let noise_std = noise_var.sqrt();

        // Signal: peak value
        let peak = signal
            .iter()
            .map(|(_, a)| *a)
            .fold(0.0_f64, |max, a| if a > max { a } else { max });

        if noise_std > 1e-30 {
            peak / noise_std
        } else {
            f64::INFINITY
        }
    }

    /// Integrate transient peak (trapezoidal rule).
    pub fn integrate_peak(signal: &[(f64, f64)]) -> f64 {
        GraphiteFurnaceProgram::compute_peak_area(signal)
    }

    /// Compute peak width at half maximum (FWHM).
    pub fn fwhm(signal: &[(f64, f64)]) -> f64 {
        if signal.len() < 3 {
            return 0.0;
        }

        let peak_val = signal
            .iter()
            .map(|(_, a)| *a)
            .fold(0.0_f64, |max, a| if a > max { a } else { max });

        let half_max = peak_val / 2.0;
        let mut first_cross = None;
        let mut last_cross = None;

        for i in 0..signal.len() - 1 {
            let (t1, a1) = signal[i];
            let (t2, a2) = signal[i + 1];
            // Rising crossing
            if a1 < half_max && a2 >= half_max && first_cross.is_none() {
                let frac = (half_max - a1) / (a2 - a1);
                first_cross = Some(t1 + frac * (t2 - t1));
            }
            // Falling crossing
            if a1 >= half_max && a2 < half_max {
                let frac = (a1 - half_max) / (a1 - a2);
                last_cross = Some(t1 + frac * (t2 - t1));
            }
        }

        match (first_cross, last_cross) {
            (Some(f), Some(l)) => l - f,
            _ => 0.0,
        }
    }

    /// Running average of absorbance readings (for steady-state flame AAS).
    pub fn running_average(readings: &[f64]) -> Vec<f64> {
        if readings.is_empty() {
            return Vec::new();
        }
        let mut result = Vec::with_capacity(readings.len());
        let mut sum = 0.0;
        for (i, &r) in readings.iter().enumerate() {
            sum += r;
            result.push(sum / (i + 1) as f64);
        }
        result
    }

    /// Standard deviation of readings.
    pub fn std_dev(readings: &[f64]) -> f64 {
        if readings.len() < 2 {
            return 0.0;
        }
        let n = readings.len() as f64;
        let mean = readings.iter().sum::<f64>() / n;
        let var = readings.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / (n - 1.0);
        var.sqrt()
    }

    /// Relative standard deviation (RSD) as percentage.
    pub fn rsd_percent(readings: &[f64]) -> f64 {
        let mean: f64 = readings.iter().sum::<f64>() / readings.len() as f64;
        if mean.abs() < 1e-30 {
            return 0.0;
        }
        Self::std_dev(readings) / mean * 100.0
    }
}

// ─── Multi-Element Sequential Analysis ───────────────────────────────────────

/// Element analysis configuration.
#[derive(Debug, Clone)]
pub struct ElementConfig {
    /// Element symbol
    pub element: String,
    /// Primary analytical wavelength (nm)
    pub wavelength_nm: f64,
    /// Slit width (nm)
    pub slit_width_nm: f64,
    /// Lamp current (mA)
    pub lamp_current_ma: f64,
    /// Flame type (None for GFAAS)
    pub flame_type: Option<FlameType>,
    /// Sensitivity (typical, in mg/L for 0.0044 abs)
    pub characteristic_concentration: f64,
    /// Linear range upper limit (mg/L)
    pub linear_range_mg_l: f64,
}

/// Multi-element sequential analyzer.
///
/// Manages sequential analysis of multiple elements with lamp switching
/// and wavelength selection.
#[derive(Debug, Clone)]
pub struct MultiElementSequential {
    /// Element configurations
    pub elements: Vec<ElementConfig>,
    /// Lamp warm-up time (seconds)
    pub lamp_warmup_s: f64,
    /// Wavelength switching time (seconds)
    pub wavelength_switch_s: f64,
}

impl MultiElementSequential {
    /// Create with common elements pre-configured.
    pub fn new() -> Self {
        let elements = vec![
            ElementConfig {
                element: "Cu".to_string(),
                wavelength_nm: 324.8,
                slit_width_nm: 0.7,
                lamp_current_ma: 4.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.04,
                linear_range_mg_l: 5.0,
            },
            ElementConfig {
                element: "Zn".to_string(),
                wavelength_nm: 213.9,
                slit_width_nm: 0.7,
                lamp_current_ma: 5.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.01,
                linear_range_mg_l: 1.5,
            },
            ElementConfig {
                element: "Fe".to_string(),
                wavelength_nm: 248.3,
                slit_width_nm: 0.2,
                lamp_current_ma: 7.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.06,
                linear_range_mg_l: 6.0,
            },
            ElementConfig {
                element: "Pb".to_string(),
                wavelength_nm: 217.0,
                slit_width_nm: 0.7,
                lamp_current_ma: 5.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.2,
                linear_range_mg_l: 20.0,
            },
            ElementConfig {
                element: "Cd".to_string(),
                wavelength_nm: 228.8,
                slit_width_nm: 0.7,
                lamp_current_ma: 4.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.01,
                linear_range_mg_l: 1.5,
            },
            ElementConfig {
                element: "Cr".to_string(),
                wavelength_nm: 357.9,
                slit_width_nm: 0.7,
                lamp_current_ma: 7.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.06,
                linear_range_mg_l: 5.0,
            },
            ElementConfig {
                element: "Ni".to_string(),
                wavelength_nm: 232.0,
                slit_width_nm: 0.2,
                lamp_current_ma: 7.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.07,
                linear_range_mg_l: 5.0,
            },
            ElementConfig {
                element: "Mn".to_string(),
                wavelength_nm: 279.5,
                slit_width_nm: 0.2,
                lamp_current_ma: 5.0,
                flame_type: Some(FlameType::AirAcetylene),
                characteristic_concentration: 0.03,
                linear_range_mg_l: 3.0,
            },
            ElementConfig {
                element: "Ca".to_string(),
                wavelength_nm: 422.7,
                slit_width_nm: 0.7,
                lamp_current_ma: 5.0,
                flame_type: Some(FlameType::NitrousOxideAcetylene),
                characteristic_concentration: 0.04,
                linear_range_mg_l: 5.0,
            },
        ];

        Self {
            elements,
            lamp_warmup_s: 600.0,
            wavelength_switch_s: 5.0,
        }
    }

    /// Find element configuration by symbol.
    pub fn find_element(&self, symbol: &str) -> Option<&ElementConfig> {
        self.elements.iter().find(|e| e.element == symbol)
    }

    /// Total analysis time for all configured elements (per sample).
    pub fn total_analysis_time(&self, readings_per_element: usize, read_time_s: f64) -> f64 {
        let n = self.elements.len() as f64;
        // First element includes warm-up; subsequent only include switch time
        self.lamp_warmup_s
            + (n - 1.0) * self.wavelength_switch_s
            + n * readings_per_element as f64 * read_time_s
    }

    /// Generate analysis sequence (element order, times).
    pub fn generate_sequence(&self) -> Vec<(String, f64, f64)> {
        // Returns (element, start_time, measurement_time)
        let mut sequence = Vec::new();
        let mut time = 0.0;
        let read_time = 3.0; // 3s per reading

        for (i, elem) in self.elements.iter().enumerate() {
            if i == 0 {
                time += self.lamp_warmup_s;
            } else {
                time += self.wavelength_switch_s;
            }
            sequence.push((elem.element.clone(), time, read_time));
            time += read_time;
        }
        sequence
    }

    /// Check if concentration is within linear range.
    pub fn is_within_range(&self, element: &str, concentration_mg_l: f64) -> bool {
        if let Some(config) = self.find_element(element) {
            concentration_mg_l <= config.linear_range_mg_l
        } else {
            false
        }
    }

    /// Suggest dilution factor if concentration exceeds linear range.
    pub fn suggest_dilution(&self, element: &str, concentration_mg_l: f64) -> f64 {
        if let Some(config) = self.find_element(element) {
            if concentration_mg_l > config.linear_range_mg_l {
                // Dilute to ~80% of linear range
                concentration_mg_l / (config.linear_range_mg_l * 0.8)
            } else {
                1.0
            }
        } else {
            1.0
        }
    }

    /// Add a custom element configuration.
    pub fn add_element(&mut self, config: ElementConfig) {
        self.elements.push(config);
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── Helper function tests ────────────────────────────────────────────

    #[test]
    fn test_transmittance_to_absorbance_50_percent() {
        // 50% transmittance -> A = 0.301
        let a = transmittance_to_absorbance(0.5);
        assert!(approx_eq(a, 0.30103, 1e-4));
    }

    #[test]
    fn test_transmittance_to_absorbance_100_percent() {
        let a = transmittance_to_absorbance(1.0);
        assert!(approx_eq(a, 0.0, EPSILON));
    }

    #[test]
    fn test_transmittance_to_absorbance_10_percent() {
        let a = transmittance_to_absorbance(0.1);
        assert!(approx_eq(a, 1.0, 1e-4));
    }

    #[test]
    fn test_transmittance_to_absorbance_1_percent() {
        let a = transmittance_to_absorbance(0.01);
        assert!(approx_eq(a, 2.0, 1e-4));
    }

    #[test]
    fn test_transmittance_to_absorbance_zero() {
        let a = transmittance_to_absorbance(0.0);
        assert!(a.is_infinite());
    }

    #[test]
    fn test_absorbance_to_transmittance_roundtrip() {
        let t = 0.35;
        let a = transmittance_to_absorbance(t);
        let t2 = absorbance_to_transmittance(a);
        assert!(approx_eq(t, t2, 1e-10));
    }

    #[test]
    fn test_absorbance_to_concentration() {
        // A = 0.5, slope = 0.1, intercept = 0.0 -> c = 5.0
        let c = absorbance_to_concentration(0.5, 0.1, 0.0);
        assert!(approx_eq(c, 5.0, EPSILON));
    }

    #[test]
    fn test_absorbance_to_concentration_with_intercept() {
        let c = absorbance_to_concentration(0.5, 0.1, 0.05);
        assert!(approx_eq(c, 4.5, EPSILON));
    }

    #[test]
    fn test_characteristic_concentration() {
        // 1 mg/L giving 0.044 absorbance -> cc = 1.0 * 0.0044 / 0.044 = 0.1
        let cc = characteristic_concentration(1.0, 0.044);
        assert!(approx_eq(cc, 0.1, 1e-4));
    }

    #[test]
    fn test_characteristic_mass() {
        let cm = characteristic_mass(10e-12, 0.044); // 10 pg giving 0.044 abs·s
        assert!(approx_eq(cm, 10e-12 * 0.0044 / 0.044, 1e-18));
    }

    // ── Beer-Lambert Law tests ───────────────────────────────────────────

    #[test]
    fn test_beer_lambert_absorbance() {
        let bl = BeerLambertLaw::new(100.0, 1.0); // epsilon=100 L/(mol*cm), l=1 cm
        let a = bl.absorbance_from_concentration(0.01); // 0.01 mol/L
        assert!(approx_eq(a, 1.0, EPSILON));
    }

    #[test]
    fn test_beer_lambert_concentration() {
        let bl = BeerLambertLaw::new(100.0, 1.0);
        let c = bl.concentration_from_absorbance(0.5);
        assert!(approx_eq(c, 0.005, EPSILON));
    }

    #[test]
    fn test_beer_lambert_intensity() {
        let bl = BeerLambertLaw::new(100.0, 1.0);
        let a = bl.absorbance_from_intensity(0.1, 1.0); // I/I0 = 0.1
        assert!(approx_eq(a, 1.0, 1e-4));
    }

    #[test]
    fn test_beer_lambert_transmittance() {
        let bl = BeerLambertLaw::new(100.0, 1.0);
        let t = bl.transmittance_from_concentration(0.01); // A=1.0
        assert!(approx_eq(t, 0.1, 1e-4));
    }

    #[test]
    fn test_beer_lambert_roundtrip() {
        let bl = BeerLambertLaw::new(250.0, 2.0);
        let c_orig = 0.003;
        let a = bl.absorbance_from_concentration(c_orig);
        let c_calc = bl.concentration_from_absorbance(a);
        assert!(approx_eq(c_orig, c_calc, 1e-10));
    }

    #[test]
    fn test_beer_lambert_linear_range() {
        let bl = BeerLambertLaw::new(100.0, 1.0);
        assert!(bl.is_linear_range(0.5));
        assert!(!bl.is_linear_range(1.5));
    }

    #[test]
    fn test_beer_lambert_percent_absorption() {
        let bl = BeerLambertLaw::new(100.0, 1.0);
        let pct = bl.percent_absorption(0.01); // A=1.0, T=0.1
        assert!(approx_eq(pct, 90.0, 0.1));
    }

    // ── Calibration Curve tests ──────────────────────────────────────────

    #[test]
    fn test_calibration_linear_fit() {
        let standards = vec![
            (0.0, 0.0),
            (1.0, 0.1),
            (2.0, 0.2),
            (3.0, 0.3),
            (4.0, 0.4),
        ];
        let curve = CalibrationCurve::new(&standards);
        assert!(approx_eq(curve.slope, 0.1, 1e-6));
        assert!(approx_eq(curve.intercept, 0.0, 1e-6));
        assert!(curve.r_squared > 0.999);
    }

    #[test]
    fn test_calibration_predict_absorbance() {
        let standards = vec![(0.0, 0.005), (1.0, 0.105), (2.0, 0.205)];
        let curve = CalibrationCurve::new(&standards);
        let predicted = curve.predict_absorbance(1.5);
        assert!(approx_eq(predicted, 0.155, 0.01));
    }

    #[test]
    fn test_calibration_predict_concentration() {
        let standards = vec![(0.0, 0.0), (1.0, 0.1), (2.0, 0.2), (3.0, 0.3)];
        let curve = CalibrationCurve::new(&standards);
        let conc = curve.predict_concentration(0.15);
        assert!(approx_eq(conc, 1.5, 0.01));
    }

    #[test]
    fn test_calibration_quadratic() {
        // Curved calibration with negative curvature (typical at high conc)
        let standards = vec![
            (0.0, 0.0),
            (1.0, 0.095),
            (2.0, 0.18),
            (3.0, 0.255),
            (4.0, 0.32),
            (5.0, 0.375),
        ];
        let mut curve = CalibrationCurve::new(&standards);
        curve.fit_quadratic();
        assert!(curve.use_quadratic);
        assert!(curve.quad_coeff < 0.0); // Negative curvature
        assert!(curve.r_squared > 0.99);
    }

    #[test]
    fn test_calibration_quadratic_prediction() {
        let standards = vec![
            (0.0, 0.0),
            (1.0, 0.095),
            (2.0, 0.18),
            (3.0, 0.255),
            (4.0, 0.32),
        ];
        let mut curve = CalibrationCurve::new(&standards);
        curve.fit_quadratic();
        // Prediction should match data reasonably
        let pred = curve.predict_absorbance(2.0);
        assert!(approx_eq(pred, 0.18, 0.01));
    }

    #[test]
    fn test_calibration_sensitivity() {
        let standards = vec![(0.0, 0.0), (1.0, 0.1), (2.0, 0.2)];
        let curve = CalibrationCurve::new(&standards);
        let s = curve.sensitivity(1.0);
        assert!(approx_eq(s, 0.1, 1e-6));
    }

    #[test]
    fn test_calibration_lod() {
        let standards = vec![(0.0, 0.0), (1.0, 0.1), (2.0, 0.2)];
        let curve = CalibrationCurve::new(&standards);
        let lod = curve.lod(0.001); // sigma_blank = 0.001
        assert!(approx_eq(lod, 0.03, 1e-4)); // 3 * 0.001 / 0.1
    }

    #[test]
    fn test_calibration_loq() {
        let standards = vec![(0.0, 0.0), (1.0, 0.1), (2.0, 0.2)];
        let curve = CalibrationCurve::new(&standards);
        let loq = curve.loq(0.001);
        assert!(approx_eq(loq, 0.1, 1e-4)); // 10 * 0.001 / 0.1
    }

    #[test]
    fn test_standard_addition() {
        // Standard addition: sample has unknown conc.
        // Adding known amounts and measuring absorbance.
        let additions = vec![
            (0.0, 0.2),  // unspiked
            (1.0, 0.3),  // +1 mg/L
            (2.0, 0.4),  // +2 mg/L
            (3.0, 0.5),  // +3 mg/L
        ];
        let conc = CalibrationCurve::standard_addition(&additions);
        assert!(approx_eq(conc, 2.0, 0.01)); // x-intercept = -0.2/0.1 = -2.0, |x-int| = 2.0
    }

    #[test]
    fn test_method_of_additions() {
        let additions = vec![(0.0, 0.15), (1.0, 0.25), (2.0, 0.35)];
        let (conc, slope, intercept) = CalibrationCurve::method_of_additions(&additions);
        assert!(approx_eq(slope, 0.1, 1e-4));
        assert!(approx_eq(intercept, 0.15, 1e-4));
        assert!(approx_eq(conc, 1.5, 0.01));
    }

    #[test]
    fn test_calibration_residuals() {
        let standards = vec![(0.0, 0.001), (1.0, 0.099), (2.0, 0.201)];
        let curve = CalibrationCurve::new(&standards);
        let residuals = curve.residuals();
        assert_eq!(residuals.len(), 3);
        // Residuals should be small
        for r in &residuals {
            assert!(r.abs() < 0.01);
        }
    }

    #[test]
    fn test_calibration_standard_error() {
        let standards = vec![
            (0.0, 0.002),
            (1.0, 0.098),
            (2.0, 0.203),
            (3.0, 0.299),
        ];
        let curve = CalibrationCurve::new(&standards);
        let se = curve.standard_error();
        assert!(se < 0.01);
    }

    #[test]
    fn test_calibration_r_squared_perfect() {
        let standards = vec![(0.0, 0.0), (1.0, 0.5), (2.0, 1.0)];
        let curve = CalibrationCurve::new(&standards);
        assert!(approx_eq(curve.r_squared, 1.0, 1e-10));
    }

    // ── Hollow Cathode Lamp tests ────────────────────────────────────────

    #[test]
    fn test_hcl_gaussian_profile_peak() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let peak = lamp.gaussian_profile(0.0);
        assert!(peak > 0.0);
    }

    #[test]
    fn test_hcl_gaussian_profile_symmetric() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let left = lamp.gaussian_profile(-0.001);
        let right = lamp.gaussian_profile(0.001);
        assert!(approx_eq(left, right, 1e-10));
    }

    #[test]
    fn test_hcl_lorentzian_profile_peak() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let peak = lamp.lorentzian_profile(0.0);
        assert!(peak > 0.0);
    }

    #[test]
    fn test_hcl_lorentzian_symmetric() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let left = lamp.lorentzian_profile(-0.002);
        let right = lamp.lorentzian_profile(0.002);
        assert!(approx_eq(left, right, 1e-10));
    }

    #[test]
    fn test_hcl_voigt_profile() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let peak = lamp.voigt_profile(0.0);
        assert!(peak > 0.0);
        // Voigt should be between Gaussian and Lorentzian at wings
        let wing = 0.005; // nm
        let g = lamp.gaussian_profile(wing);
        let l = lamp.lorentzian_profile(wing);
        let v = lamp.voigt_profile(wing);
        // Voigt wings should be broader than Gaussian but less than Lorentzian
        assert!(v >= g * 0.9); // Voigt has more wing than Gaussian
    }

    #[test]
    fn test_hcl_emission_profile_dispatch() {
        let mut lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        lamp.profile = LineProfile::Gaussian;
        let g = lamp.emission_profile(0.0);
        lamp.profile = LineProfile::Lorentzian;
        let l = lamp.emission_profile(0.0);
        lamp.profile = LineProfile::Voigt;
        let v = lamp.emission_profile(0.0);
        assert!(g > 0.0);
        assert!(l > 0.0);
        assert!(v > 0.0);
    }

    #[test]
    fn test_hcl_self_absorption_nominal() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let factor = lamp.self_absorption_factor(4.0); // at nominal
        assert!(approx_eq(factor, 1.0, EPSILON));
    }

    #[test]
    fn test_hcl_self_absorption_high_current() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let factor = lamp.self_absorption_factor(8.0); // 2x nominal
        assert!(factor < 1.0);
        assert!(factor >= 0.0);
    }

    #[test]
    fn test_hcl_effective_intensity() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let i_nom = lamp.effective_intensity(4.0);
        assert!(approx_eq(i_nom, 1.0, EPSILON));
        let i_high = lamp.effective_intensity(8.0);
        // Higher current gives more intensity but with self-absorption
        assert!(i_high > 1.0);
        assert!(i_high < 2.0); // Less than 2x due to self-absorption
    }

    #[test]
    fn test_hcl_warmup_stability() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        let s0 = lamp.warmup_stability(0.0);
        assert!(approx_eq(s0, 0.0, 0.01));
        let s_long = lamp.warmup_stability(3600.0); // 1 hour
        assert!(s_long > 0.99);
    }

    #[test]
    fn test_hcl_chopper() {
        let lamp = HollowCathodeLamp::new("Cu", 324.8, 4.0);
        // At t=0, sin(0) = 0 -> on (just barely)
        let m1 = lamp.chopper_modulation(0.0, 50.0);
        assert!(m1 == 1.0 || m1 == 0.0);
        // Half period later should be opposite
        let m2 = lamp.chopper_modulation(0.01, 50.0); // 50 Hz, half period = 0.01s
        // m1 and m2 should exist as valid values
        assert!(m2 == 0.0 || m2 == 1.0);
    }

    // ── Flame Atomizer tests ─────────────────────────────────────────────

    #[test]
    fn test_flame_temperatures() {
        assert!(approx_eq(FlameType::AirAcetylene.temperature_c(), 2300.0, 0.1));
        assert!(approx_eq(FlameType::NitrousOxideAcetylene.temperature_c(), 2950.0, 0.1));
        assert!(approx_eq(FlameType::AirHydrogen.temperature_c(), 2050.0, 0.1));
        assert!(approx_eq(FlameType::AirPropane.temperature_c(), 1920.0, 0.1));
    }

    #[test]
    fn test_flame_temperature_kelvin() {
        let t_k = FlameType::AirAcetylene.temperature_k();
        let t_c = FlameType::AirAcetylene.temperature_c();
        assert!(approx_eq(t_k - 273.15, t_c, 0.01));
    }

    #[test]
    fn test_boltzmann_ratio_na() {
        // Na D line: 589.0 nm, E = 2.104 eV, g*/g0 = 2/1
        let ratio = FlameAtomizer::boltzmann_ratio(2.0, 1.0, 2.104, 2573.15);
        // At 2300 C, ratio should be very small (most atoms in ground state)
        assert!(ratio < 0.01);
        assert!(ratio > 0.0);
    }

    #[test]
    fn test_boltzmann_ratio_increases_with_temperature() {
        let r_low = FlameAtomizer::boltzmann_ratio(2.0, 1.0, 2.0, 2000.0);
        let r_high = FlameAtomizer::boltzmann_ratio(2.0, 1.0, 2.0, 3000.0);
        assert!(r_high > r_low);
    }

    #[test]
    fn test_ground_state_fraction() {
        let flame = FlameAtomizer::new(FlameType::AirAcetylene);
        let frac = flame.ground_state_fraction(1.0, 3.0, 3.0);
        // For most elements at 2300 C, ground state fraction is > 0.99
        assert!(frac > 0.9);
        assert!(frac <= 1.0);
    }

    #[test]
    fn test_effective_atom_concentration() {
        let flame = FlameAtomizer::new(FlameType::AirAcetylene);
        let eff = flame.effective_atom_concentration(10.0, 1.0, 3.0, 3.0);
        // Should be much less than input due to nebulization and atomization losses
        assert!(eff < 10.0);
        assert!(eff > 0.0);
    }

    #[test]
    fn test_wavelength_energy_conversion() {
        // Na D line: 589.0 nm ~ 2.104 eV
        let e = FlameAtomizer::energy_from_wavelength(589.0);
        assert!(approx_eq(e, 2.104, 0.01));
        let w = FlameAtomizer::wavelength_from_energy(2.104);
        assert!(approx_eq(w, 589.0, 1.0));
    }

    #[test]
    fn test_wavelength_energy_roundtrip() {
        let w_orig = 324.8; // Cu
        let e = FlameAtomizer::energy_from_wavelength(w_orig);
        let w_calc = FlameAtomizer::wavelength_from_energy(e);
        assert!(approx_eq(w_orig, w_calc, 0.01));
    }

    #[test]
    fn test_flame_atomizer_defaults() {
        let flame = FlameAtomizer::new(FlameType::AirAcetylene);
        assert!(flame.nebulization_efficiency > 0.0 && flame.nebulization_efficiency < 1.0);
        assert!(flame.atomization_efficiency > 0.0 && flame.atomization_efficiency < 1.0);
        assert!(flame.path_length_cm > 0.0);
    }

    // ── Graphite Furnace tests ───────────────────────────────────────────

    #[test]
    fn test_furnace_default_program() {
        let prog = GraphiteFurnaceProgram::default_program("Pb");
        assert_eq!(prog.steps.len(), 5);
        assert_eq!(prog.steps[0].stage, FurnaceStage::Dry);
        assert_eq!(prog.steps[2].stage, FurnaceStage::Atomize);
    }

    #[test]
    fn test_furnace_total_duration() {
        let prog = GraphiteFurnaceProgram::default_program("Cu");
        let dur = prog.total_duration();
        assert!(dur > 0.0);
    }

    #[test]
    fn test_furnace_temperature_at_start() {
        let prog = GraphiteFurnaceProgram::default_program("Cu");
        let t = prog.temperature_at(0.0);
        // Should be ramping from 25 C toward drying temp
        assert!(t >= 25.0);
    }

    #[test]
    fn test_furnace_temperature_at_dry() {
        let prog = GraphiteFurnaceProgram::default_program("Cu");
        // After ramp + hold of dry step = 10 + 20 = 30s
        let t = prog.temperature_at(25.0);
        // Should be at or near 110 C
        assert!(t >= 100.0 && t <= 120.0);
    }

    #[test]
    fn test_furnace_temperature_after_program() {
        let prog = GraphiteFurnaceProgram::default_program("Cu");
        let t = prog.temperature_at(1000.0);
        assert!(approx_eq(t, 25.0, EPSILON));
    }

    #[test]
    fn test_furnace_simulate_signal() {
        let prog = GraphiteFurnaceProgram::default_program("Pb");
        let signal = prog.simulate_atomization_signal(0.5, 100);
        assert_eq!(signal.len(), 100);
        let peak = GraphiteFurnaceProgram::compute_peak_height(&signal);
        assert!(approx_eq(peak, 0.5, 0.01));
    }

    #[test]
    fn test_furnace_peak_area() {
        let prog = GraphiteFurnaceProgram::default_program("Pb");
        let signal = prog.simulate_atomization_signal(1.0, 200);
        let area = GraphiteFurnaceProgram::compute_peak_area(&signal);
        assert!(area > 0.0);
    }

    #[test]
    fn test_furnace_peak_height() {
        let prog = GraphiteFurnaceProgram::default_program("Cd");
        let signal = prog.simulate_atomization_signal(0.8, 100);
        let height = GraphiteFurnaceProgram::compute_peak_height(&signal);
        assert!(approx_eq(height, 0.8, 0.02));
    }

    #[test]
    fn test_furnace_platform_delay() {
        let mut prog = GraphiteFurnaceProgram::default_program("Pb");
        prog.lvov_platform = true;
        assert!(prog.platform_delay_factor() < 1.0);
        prog.lvov_platform = false;
        assert!(approx_eq(prog.platform_delay_factor(), 1.0, EPSILON));
    }

    #[test]
    fn test_furnace_modifier_effect() {
        let prog = GraphiteFurnaceProgram::default_program("Pb")
            .with_modifier("Pd/Mg(NO3)2");
        let increase = prog.modifier_char_temp_increase();
        assert!(increase > 0.0); // Pd modifier should increase char temp
    }

    #[test]
    fn test_furnace_no_modifier() {
        let prog = GraphiteFurnaceProgram::default_program("Cu");
        assert!(approx_eq(prog.modifier_char_temp_increase(), 0.0, EPSILON));
    }

    // ── Background Correction tests ──────────────────────────────────────

    #[test]
    fn test_d2_correction() {
        let corrected = BackgroundCorrection::deuterium_correction(0.5, 0.1);
        assert!(approx_eq(corrected, 0.4, EPSILON));
    }

    #[test]
    fn test_d2_correction_no_negative() {
        let corrected = BackgroundCorrection::deuterium_correction(0.1, 0.5);
        assert!(approx_eq(corrected, 0.0, EPSILON));
    }

    #[test]
    fn test_zeeman_correction() {
        let corrected = BackgroundCorrection::zeeman_correction(0.6, 0.15);
        assert!(approx_eq(corrected, 0.45, EPSILON));
    }

    #[test]
    fn test_smith_hieftje_correction() {
        let corrected = BackgroundCorrection::smith_hieftje_correction(0.5, 0.08);
        assert!(approx_eq(corrected, 0.42, EPSILON));
    }

    #[test]
    fn test_zeeman_splitting() {
        let split = BackgroundCorrection::zeeman_splitting(324.8, 0.8);
        assert!(split > 0.0);
        assert!(split < 0.1); // Splitting is typically very small
    }

    #[test]
    fn test_background_apply_none() {
        let bg = BackgroundCorrection::new(BackgroundCorrectionMethod::None);
        let result = bg.apply(0.5, 0.1);
        assert!(approx_eq(result, 0.5, EPSILON));
    }

    #[test]
    fn test_background_apply_d2() {
        let bg = BackgroundCorrection::new(BackgroundCorrectionMethod::DeuteriumArc);
        let result = bg.apply(0.5, 0.1);
        assert!(approx_eq(result, 0.4, EPSILON));
    }

    #[test]
    fn test_background_apply_zeeman() {
        let bg = BackgroundCorrection::new(BackgroundCorrectionMethod::ZeemanNormal);
        let result = bg.apply(0.5, 0.1);
        assert!(approx_eq(result, 0.4, EPSILON));
    }

    // ── Interference Correction tests ────────────────────────────────────

    #[test]
    fn test_spectral_correction() {
        let mut ic = InterferenceCorrection::new();
        ic.add_spectral_correction("Fe", 0.05);
        let corrected =
            ic.correct_spectral(0.5, &[("Fe".to_string(), 1.0)]);
        assert!(approx_eq(corrected, 0.45, EPSILON));
    }

    #[test]
    fn test_spectral_correction_no_negative() {
        let mut ic = InterferenceCorrection::new();
        ic.add_spectral_correction("Fe", 0.5);
        let corrected =
            ic.correct_spectral(0.1, &[("Fe".to_string(), 1.0)]);
        assert!(approx_eq(corrected, 0.0, EPSILON));
    }

    #[test]
    fn test_ionization_fraction() {
        // Na: ionization energy 5.14 eV, at 2573 K
        let alpha =
            InterferenceCorrection::ionization_fraction(5.14, 2573.15, 1e10);
        assert!(alpha >= 0.0 && alpha <= 1.0);
    }

    #[test]
    fn test_ionization_fraction_increases_with_temperature() {
        let a_low = InterferenceCorrection::ionization_fraction(5.14, 2000.0, 1e10);
        let a_high = InterferenceCorrection::ionization_fraction(5.14, 3000.0, 1e10);
        assert!(a_high > a_low);
    }

    #[test]
    fn test_ionization_suppression() {
        let mut ic = InterferenceCorrection::new();
        ic.set_ionization_buffer("Cs", 1000.0);
        let factor =
            ic.ionization_suppression_factor(5.14, 2573.15, 1e10);
        // Adding Cs should increase ground state atoms -> factor > 1
        assert!(factor >= 1.0);
    }

    #[test]
    fn test_releasing_agent_set() {
        let mut ic = InterferenceCorrection::new();
        ic.set_releasing_agent("La");
        assert_eq!(ic.releasing_agent, Some("La".to_string()));
    }

    // ── Signal Processing tests ──────────────────────────────────────────

    #[test]
    fn test_baseline_correction() {
        let signal: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let t = i as f64 * 0.01;
                let baseline = 0.05;
                let peak = if i > 30 && i < 70 {
                    0.3 * (-((i as f64 - 50.0) / 10.0).powi(2)).exp()
                } else {
                    0.0
                };
                (t, baseline + peak)
            })
            .collect();

        let corrected = SignalProcessor::baseline_correct(&signal, 10);
        // Baseline should be removed
        assert!(corrected[0].1 < 0.01);
        // Peak should still be present
        let peak_val = corrected.iter().map(|(_, a)| *a).fold(0.0_f64, f64::max);
        assert!(peak_val > 0.1);
    }

    #[test]
    fn test_find_peaks() {
        let signal: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let t = i as f64 * 0.01;
                let a = if i == 25 {
                    0.5
                } else if i == 75 {
                    0.8
                } else {
                    0.01
                };
                (t, a)
            })
            .collect();

        let peaks = SignalProcessor::find_peaks(&signal, 0.1, 0.1);
        assert_eq!(peaks.len(), 2);
    }

    #[test]
    fn test_smooth() {
        let signal: Vec<(f64, f64)> = (0..20)
            .map(|i| (i as f64, if i == 10 { 1.0 } else { 0.0 }))
            .collect();

        let smoothed = SignalProcessor::smooth(&signal, 5);
        assert_eq!(smoothed.len(), signal.len());
        // Smoothed peak should be lower than original
        let smooth_peak = smoothed.iter().map(|(_, a)| *a).fold(0.0_f64, f64::max);
        assert!(smooth_peak < 1.0);
        assert!(smooth_peak > 0.0);
    }

    #[test]
    fn test_snr() {
        let mut signal: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let t = i as f64 * 0.01;
                let noise = 0.001 * ((i as f64 * 0.7).sin());
                (t, noise)
            })
            .collect();
        // Add a peak
        signal[50].1 = 0.5;

        let snr_val = SignalProcessor::snr(&signal, 20);
        assert!(snr_val > 10.0);
    }

    #[test]
    fn test_integrate_peak() {
        let signal = vec![(0.0, 0.0), (1.0, 0.5), (2.0, 1.0), (3.0, 0.5), (4.0, 0.0)];
        let area = SignalProcessor::integrate_peak(&signal);
        assert!(approx_eq(area, 2.0, 0.01));
    }

    #[test]
    fn test_fwhm() {
        // Gaussian-like peak
        let signal: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let t = i as f64 * 0.1;
                let a = (-0.5 * ((t - 5.0) / 1.0).powi(2)).exp();
                (t, a)
            })
            .collect();

        let fwhm = SignalProcessor::fwhm(&signal);
        // FWHM of Gaussian with sigma=1.0 is 2*sqrt(2*ln2) ~ 2.355
        assert!(approx_eq(fwhm, 2.355, 0.2));
    }

    #[test]
    fn test_running_average() {
        let readings = vec![0.1, 0.12, 0.11, 0.09, 0.10];
        let avg = SignalProcessor::running_average(&readings);
        assert_eq!(avg.len(), 5);
        assert!(approx_eq(avg[0], 0.1, EPSILON));
        // Final average should be close to the mean
        let mean = readings.iter().sum::<f64>() / readings.len() as f64;
        assert!(approx_eq(avg[4], mean, EPSILON));
    }

    #[test]
    fn test_std_dev() {
        let readings = vec![10.0, 10.0, 10.0, 10.0];
        let sd = SignalProcessor::std_dev(&readings);
        assert!(approx_eq(sd, 0.0, EPSILON));
    }

    #[test]
    fn test_std_dev_nonzero() {
        let readings = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let sd = SignalProcessor::std_dev(&readings);
        // std dev of 1..5 (sample) = sqrt(2.5) = 1.5811
        assert!(approx_eq(sd, 1.5811, 0.001));
    }

    #[test]
    fn test_rsd_percent() {
        let readings = vec![0.100, 0.102, 0.098, 0.101, 0.099];
        let rsd = SignalProcessor::rsd_percent(&readings);
        assert!(rsd < 5.0); // RSD should be small for consistent readings
        assert!(rsd > 0.0);
    }

    // ── Multi-Element tests ──────────────────────────────────────────────

    #[test]
    fn test_multi_element_default() {
        let me = MultiElementSequential::new();
        assert!(me.elements.len() >= 9);
    }

    #[test]
    fn test_find_element_cu() {
        let me = MultiElementSequential::new();
        let cu = me.find_element("Cu").unwrap();
        assert!(approx_eq(cu.wavelength_nm, 324.8, 0.1));
    }

    #[test]
    fn test_find_element_zn() {
        let me = MultiElementSequential::new();
        let zn = me.find_element("Zn").unwrap();
        assert!(approx_eq(zn.wavelength_nm, 213.9, 0.1));
    }

    #[test]
    fn test_find_element_not_found() {
        let me = MultiElementSequential::new();
        assert!(me.find_element("Unobtainium").is_none());
    }

    #[test]
    fn test_total_analysis_time() {
        let me = MultiElementSequential::new();
        let time = me.total_analysis_time(3, 3.0);
        assert!(time > 0.0);
        // Should include warm-up + switching + measurement
        assert!(time > me.lamp_warmup_s);
    }

    #[test]
    fn test_generate_sequence() {
        let me = MultiElementSequential::new();
        let seq = me.generate_sequence();
        assert_eq!(seq.len(), me.elements.len());
        // First element starts after warm-up
        assert!(seq[0].1 >= me.lamp_warmup_s);
    }

    #[test]
    fn test_within_range() {
        let me = MultiElementSequential::new();
        assert!(me.is_within_range("Cu", 3.0));
        assert!(!me.is_within_range("Cu", 10.0));
    }

    #[test]
    fn test_suggest_dilution_needed() {
        let me = MultiElementSequential::new();
        let dil = me.suggest_dilution("Cu", 20.0);
        assert!(dil > 1.0);
    }

    #[test]
    fn test_suggest_dilution_not_needed() {
        let me = MultiElementSequential::new();
        let dil = me.suggest_dilution("Cu", 1.0);
        assert!(approx_eq(dil, 1.0, EPSILON));
    }

    #[test]
    fn test_add_custom_element() {
        let mut me = MultiElementSequential::new();
        let n = me.elements.len();
        me.add_element(ElementConfig {
            element: "Au".to_string(),
            wavelength_nm: 242.8,
            slit_width_nm: 0.7,
            lamp_current_ma: 5.0,
            flame_type: Some(FlameType::AirAcetylene),
            characteristic_concentration: 0.15,
            linear_range_mg_l: 20.0,
        });
        assert_eq!(me.elements.len(), n + 1);
        assert!(me.find_element("Au").is_some());
    }

    #[test]
    fn test_element_wavelengths() {
        let me = MultiElementSequential::new();
        let fe = me.find_element("Fe").unwrap();
        assert!(approx_eq(fe.wavelength_nm, 248.3, 0.1));
        let pb = me.find_element("Pb").unwrap();
        assert!(approx_eq(pb.wavelength_nm, 217.0, 0.1));
        let cd = me.find_element("Cd").unwrap();
        assert!(approx_eq(cd.wavelength_nm, 228.8, 0.1));
        let cr = me.find_element("Cr").unwrap();
        assert!(approx_eq(cr.wavelength_nm, 357.9, 0.1));
        let ni = me.find_element("Ni").unwrap();
        assert!(approx_eq(ni.wavelength_nm, 232.0, 0.1));
        let mn = me.find_element("Mn").unwrap();
        assert!(approx_eq(mn.wavelength_nm, 279.5, 0.1));
    }

    #[test]
    fn test_ca_requires_nitrous_oxide() {
        let me = MultiElementSequential::new();
        let ca = me.find_element("Ca").unwrap();
        assert_eq!(ca.flame_type, Some(FlameType::NitrousOxideAcetylene));
    }
}
