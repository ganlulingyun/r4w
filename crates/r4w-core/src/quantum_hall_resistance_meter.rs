//! # Quantum Hall Effect Resistance Metrology
//!
//! This module implements precision resistance measurement based on the quantum
//! Hall effect (QHE), the gold standard for electrical resistance standards
//! worldwide. When a 2D electron gas (typically in a GaAs/AlGaAs heterostructure)
//! is subjected to a strong perpendicular magnetic field at cryogenic temperatures,
//! the Hall resistance becomes exactly quantized to R_K / i, where R_K = h/e^2
//! is the von Klitzing constant and i is an integer filling factor.
//!
//! Since the 2019 SI redefinition, R_K = 25812.80745... ohms is exact, making the
//! QHE a primary realization of the ohm with no measurement uncertainty from the
//! defining constant itself.
//!
//! ## Key Components
//!
//! - [`VonKlitzingConstant`] -- The exact value R_K = h/e^2 and derived plateau values
//! - [`HallBarConfig`] -- Physical parameters of a GaAs/AlGaAs Hall bar device
//! - [`LandauLevel`] -- Landau level energies, cyclotron frequency, magnetic length
//! - [`MagnetoresistanceMeasurement`] -- R_xx and R_xy vs magnetic field
//! - [`CccBridge`] -- Cryogenic Current Comparator bridge simulation
//! - [`TemperatureEffects`] -- Thermal activation and plateau width
//! - [`CurrentDependence`] -- Critical current and breakdown regime
//! - [`PrecisionMetrics`] -- ppb deviation, Allan variance, uncertainty budget
//! - [`CalibrationChain`] -- Traceability from QHR to working standards
//!
//! ## Physical Constants (2019 SI exact values)
//!
//! | Constant | Symbol | Value |
//! |----------|--------|-------|
//! | Planck constant | h | 6.62607015e-34 J s |
//! | Elementary charge | e | 1.602176634e-19 C |
//! | Boltzmann constant | k_B | 1.380649e-23 J/K |
//! | Electron mass | m_e | 9.1093837015e-31 kg |
//! | Reduced Planck | h_bar | h/(2pi) |
//! | Von Klitzing | R_K | h/e^2 = 25812.80745... ohm |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quantum_hall_resistance_meter::*;
//!
//! // Von Klitzing constant
//! let rk = VON_KLITZING_CONSTANT;
//! assert!((rk - 25812.807).abs() < 0.001);
//!
//! // i=2 plateau (most commonly used for metrology)
//! let r_i2 = hall_plateau_resistance(2);
//! assert!((r_i2 - 12906.4037).abs() < 0.001);
//!
//! // Configure a Hall bar
//! let bar = HallBarConfig::default_gaas();
//! let ll = LandauLevel::new(10.0); // 10 Tesla
//! let nu = ll.filling_factor(bar.carrier_density_m2);
//! assert!(nu > 0.0);
//! ```

use std::f64::consts::PI;

// ============================================================================
// Physical constants (2019 SI exact values)
// ============================================================================

/// Planck constant h in J*s (exact since 2019 SI)
pub const PLANCK_CONSTANT: f64 = 6.626_070_15e-34;

/// Reduced Planck constant h_bar = h/(2*pi)
pub const HBAR: f64 = PLANCK_CONSTANT / (2.0 * PI);

/// Elementary charge e in coulombs (exact since 2019 SI)
pub const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

/// Boltzmann constant k_B in J/K (exact since 2019 SI)
pub const BOLTZMANN_CONSTANT: f64 = 1.380_649e-23;

/// Electron rest mass in kg
pub const ELECTRON_MASS: f64 = 9.109_383_7015e-31;

/// GaAs effective mass: m* = 0.067 * m_e
pub const GAAS_EFFECTIVE_MASS: f64 = 0.067 * ELECTRON_MASS;

/// Von Klitzing constant R_K = h/e^2 in ohms (exact since 2019 SI)
pub const VON_KLITZING_CONSTANT: f64 =
    PLANCK_CONSTANT / (ELEMENTARY_CHARGE * ELEMENTARY_CHARGE);

// ============================================================================
// Hall resistance plateaus
// ============================================================================

/// Returns the quantized Hall resistance R_H = R_K / i for filling factor i.
///
/// # Panics
/// Panics if `filling_factor` is zero.
pub fn hall_plateau_resistance(filling_factor: u32) -> f64 {
    assert!(filling_factor > 0, "filling factor must be >= 1");
    VON_KLITZING_CONSTANT / filling_factor as f64
}

/// Returns plateau resistances for filling factors 1..=max_i.
pub fn plateau_table(max_i: u32) -> Vec<(u32, f64)> {
    (1..=max_i).map(|i| (i, hall_plateau_resistance(i))).collect()
}

// ============================================================================
// HallBarConfig
// ============================================================================

/// Physical configuration of a GaAs/AlGaAs Hall bar device.
#[derive(Debug, Clone)]
pub struct HallBarConfig {
    /// Hall bar width in metres (typical: 400 um)
    pub width_m: f64,
    /// Hall bar length in metres (typical: several mm)
    pub length_m: f64,
    /// 2DEG sheet carrier density in m^-2 (typical: 4.6e15)
    pub carrier_density_m2: f64,
    /// Electron mobility in m^2/(V*s) (typical: 50-100 for high-quality GaAs)
    pub mobility_m2_per_vs: f64,
    /// Operating temperature in kelvin (typical: 1.3-1.5 K)
    pub temperature_k: f64,
}

impl HallBarConfig {
    /// Standard high-quality GaAs/AlGaAs Hall bar for metrology.
    pub fn default_gaas() -> Self {
        Self {
            width_m: 400.0e-6,
            length_m: 4.0e-3,
            carrier_density_m2: 4.6e15,
            mobility_m2_per_vs: 50.0,
            temperature_k: 1.3,
        }
    }

    /// Lower-mobility device typical of routine calibration.
    pub fn routine_calibration() -> Self {
        Self {
            width_m: 200.0e-6,
            length_m: 2.0e-3,
            carrier_density_m2: 3.5e15,
            mobility_m2_per_vs: 20.0,
            temperature_k: 1.5,
        }
    }

    /// Classical Hall coefficient R_H = 1/(n_s * e) in ohms/Tesla.
    pub fn classical_hall_coefficient(&self) -> f64 {
        1.0 / (self.carrier_density_m2 * ELEMENTARY_CHARGE)
    }

    /// Classical sheet resistance rho_s = 1/(n_s * e * mu).
    pub fn sheet_resistance(&self) -> f64 {
        1.0 / (self.carrier_density_m2 * ELEMENTARY_CHARGE * self.mobility_m2_per_vs)
    }

    /// Scattering time tau = m* * mu / e.
    pub fn scattering_time(&self) -> f64 {
        GAAS_EFFECTIVE_MASS * self.mobility_m2_per_vs / ELEMENTARY_CHARGE
    }

    /// Magnetic field for filling factor nu: B = n_s * h / (e * nu).
    pub fn magnetic_field_for_filling(&self, nu: f64) -> f64 {
        assert!(nu > 0.0, "filling factor must be positive");
        self.carrier_density_m2 * PLANCK_CONSTANT / (ELEMENTARY_CHARGE * nu)
    }
}

// ============================================================================
// Landau levels
// ============================================================================

/// Landau level calculator for a given magnetic field.
#[derive(Debug, Clone)]
pub struct LandauLevel {
    /// Magnetic field strength in Tesla
    pub b_field: f64,
}

impl LandauLevel {
    /// Create for a given magnetic field in Tesla.
    pub fn new(b_field: f64) -> Self {
        assert!(b_field > 0.0, "magnetic field must be positive");
        Self { b_field }
    }

    /// Cyclotron angular frequency omega_c = eB/m* in rad/s.
    pub fn cyclotron_omega(&self) -> f64 {
        ELEMENTARY_CHARGE * self.b_field / GAAS_EFFECTIVE_MASS
    }

    /// Cyclotron frequency f_c = eB/(2*pi*m*) in Hz.
    pub fn cyclotron_frequency(&self) -> f64 {
        self.cyclotron_omega() / (2.0 * PI)
    }

    /// Landau level energy: E_n = h_bar * omega_c * (n + 1/2) in Joules.
    pub fn energy(&self, n: u32) -> f64 {
        HBAR * self.cyclotron_omega() * (n as f64 + 0.5)
    }

    /// Energy gap between adjacent Landau levels: delta_E = h_bar * omega_c.
    pub fn energy_gap(&self) -> f64 {
        HBAR * self.cyclotron_omega()
    }

    /// Magnetic length l_B = sqrt(h_bar / (eB)) in metres.
    pub fn magnetic_length(&self) -> f64 {
        (HBAR / (ELEMENTARY_CHARGE * self.b_field)).sqrt()
    }

    /// Degeneracy per Landau level per unit area: n_L = eB/h in m^-2.
    pub fn degeneracy_per_area(&self) -> f64 {
        ELEMENTARY_CHARGE * self.b_field / PLANCK_CONSTANT
    }

    /// Filling factor nu = n_s / n_L = n_s * h / (eB).
    pub fn filling_factor(&self, carrier_density: f64) -> f64 {
        carrier_density / self.degeneracy_per_area()
    }

    /// Number of completely filled Landau levels (integer part of nu).
    pub fn filled_levels(&self, carrier_density: f64) -> u32 {
        self.filling_factor(carrier_density).floor() as u32
    }

    /// Activation energy for plateau: delta_E/2 = h_bar * omega_c / 2.
    pub fn activation_energy(&self) -> f64 {
        self.energy_gap() / 2.0
    }

    /// Thermal energy k_B * T for comparison with Landau gap.
    pub fn thermal_energy(temperature_k: f64) -> f64 {
        BOLTZMANN_CONSTANT * temperature_k
    }

    /// Ratio of Landau gap to thermal energy: h_bar*omega_c / (k_B*T).
    /// Should be >> 1 for well-resolved plateaus.
    pub fn gap_to_thermal_ratio(&self, temperature_k: f64) -> f64 {
        self.energy_gap() / Self::thermal_energy(temperature_k)
    }
}

// ============================================================================
// Magnetoresistance measurement
// ============================================================================

/// Models R_xx and R_xy as functions of magnetic field for a Hall bar.
#[derive(Debug, Clone)]
pub struct MagnetoresistanceMeasurement {
    config: HallBarConfig,
}

impl MagnetoresistanceMeasurement {
    pub fn new(config: HallBarConfig) -> Self {
        Self { config }
    }

    /// Hall resistance R_xy at given magnetic field.
    ///
    /// On plateaus (integer filling), R_xy = R_K / i exactly.
    /// Between plateaus, R_xy interpolates linearly (classical regime at low B).
    pub fn r_xy(&self, b_field: f64) -> f64 {
        if b_field <= 0.0 {
            return 0.0;
        }
        let ll = LandauLevel::new(b_field);
        let nu = ll.filling_factor(self.config.carrier_density_m2);

        if nu < 0.5 {
            // Very high field, beyond i=1; clamp at i=1 plateau
            return VON_KLITZING_CONSTANT;
        }

        let i_floor = nu.floor() as u32;
        let i_ceil = i_floor + 1;
        let frac = nu - nu.floor();

        // Check if we're on a plateau (within ~0.2 of integer filling)
        let plateau_width = self.plateau_half_width(b_field);
        let nearest_int = nu.round() as u32;
        if nearest_int >= 1 {
            let dist = (nu - nearest_int as f64).abs();
            if dist < plateau_width {
                return VON_KLITZING_CONSTANT / nearest_int as f64;
            }
        }

        // Between plateaus: linear interpolation in 1/nu space
        if i_floor == 0 {
            // Above i=1, approaching classical regime
            return VON_KLITZING_CONSTANT / nu;
        }
        let r_low = VON_KLITZING_CONSTANT / i_ceil as f64;
        let r_high = VON_KLITZING_CONSTANT / i_floor as f64;
        r_low + (r_high - r_low) * (1.0 - frac)
    }

    /// Longitudinal resistance R_xx at given magnetic field.
    ///
    /// R_xx = 0 on plateaus (dissipationless), with peaks at half-integer filling.
    /// Peaks are exponentially suppressed at low temperature.
    pub fn r_xx(&self, b_field: f64) -> f64 {
        if b_field <= 0.0 {
            return self.config.sheet_resistance();
        }
        let ll = LandauLevel::new(b_field);
        let nu = ll.filling_factor(self.config.carrier_density_m2);

        if nu < 0.5 {
            return 0.0;
        }

        // Distance to nearest integer filling
        let nearest_int = nu.round();
        let delta_nu = (nu - nearest_int).abs();

        // Plateau half-width narrows with temperature
        let plateau_hw = self.plateau_half_width(b_field);

        if delta_nu < plateau_hw {
            // On plateau: exponentially small R_xx
            let gap_ratio = ll.gap_to_thermal_ratio(self.config.temperature_k);
            let r_min = self.config.sheet_resistance() * (-gap_ratio / 2.0).exp();
            return r_min;
        }

        // Between plateaus: Gaussian-like peak centered at half-integer filling
        let peak_dist = delta_nu - plateau_hw;
        let sigma = 0.15; // width of transition region
        let peak_height = self.config.sheet_resistance() * 0.5;
        peak_height * (-peak_dist * peak_dist / (2.0 * sigma * sigma)).exp()
    }

    /// Half-width of plateau in filling factor units.
    /// Narrows exponentially with temperature.
    fn plateau_half_width(&self, b_field: f64) -> f64 {
        let ll = LandauLevel::new(b_field);
        let gap_ratio = ll.gap_to_thermal_ratio(self.config.temperature_k);
        // At T=0, plateau width ~0.4; narrows with temperature
        let base_width = 0.4;
        let thermal_factor = 1.0 - (-gap_ratio / 4.0).exp();
        base_width * thermal_factor.max(0.01)
    }

    /// Sweep R_xy and R_xx over a magnetic field range.
    /// Returns Vec of (B, R_xy, R_xx).
    pub fn sweep(&self, b_min: f64, b_max: f64, num_points: usize) -> Vec<(f64, f64, f64)> {
        assert!(num_points >= 2);
        let step = (b_max - b_min) / (num_points - 1) as f64;
        (0..num_points)
            .map(|i| {
                let b = b_min + i as f64 * step;
                (b, self.r_xy(b), self.r_xx(b))
            })
            .collect()
    }
}

// ============================================================================
// CCC Bridge simulation
// ============================================================================

/// Cryogenic Current Comparator (CCC) bridge for precision resistance ratios.
///
/// The CCC uses superconducting coils and a SQUID to compare currents with
/// ratios determined by winding turns: N1*I1 = N2*I2, giving R1/R2 = N2/N1.
#[derive(Debug, Clone)]
pub struct CccBridge {
    /// Primary winding turns
    pub n1: u32,
    /// Secondary winding turns
    pub n2: u32,
    /// Type A uncertainty (statistical) in ppb
    pub type_a_uncertainty_ppb: f64,
    /// Type B uncertainty (systematic) in ppb
    pub type_b_uncertainty_ppb: f64,
}

impl CccBridge {
    /// Standard 1:1 bridge for comparing two QHR devices.
    pub fn one_to_one() -> Self {
        Self {
            n1: 1,
            n2: 1,
            type_a_uncertainty_ppb: 0.1,
            type_b_uncertainty_ppb: 0.05,
        }
    }

    /// Bridge for deriving 1 ohm from i=2 QHR (12906.4 ohm).
    /// Uses ratio 12906:1 (approximately).
    pub fn qhr_to_one_ohm() -> Self {
        Self {
            n1: 12906,
            n2: 1,
            type_a_uncertainty_ppb: 1.0,
            type_b_uncertainty_ppb: 0.5,
        }
    }

    /// Bridge for deriving 10 kohm from i=2 QHR (12906.4 ohm).
    pub fn qhr_to_10k() -> Self {
        Self {
            n1: 12906,
            n2: 10000,
            type_a_uncertainty_ppb: 0.5,
            type_b_uncertainty_ppb: 0.3,
        }
    }

    /// Ideal resistance ratio: N2/N1.
    pub fn ideal_ratio(&self) -> f64 {
        self.n2 as f64 / self.n1 as f64
    }

    /// Measured resistance ratio including ppb-level correction delta.
    /// R_unknown = R_reference * (N2/N1) * (1 + delta * 1e-9)
    pub fn measured_ratio(&self, delta_ppb: f64) -> f64 {
        self.ideal_ratio() * (1.0 + delta_ppb * 1e-9)
    }

    /// Derive unknown resistance from reference using CCC bridge.
    pub fn derive_resistance(&self, r_reference: f64, delta_ppb: f64) -> f64 {
        r_reference * self.measured_ratio(delta_ppb)
    }

    /// Combined standard uncertainty (k=1) in ppb.
    /// u_c = sqrt(u_A^2 + u_B^2)
    pub fn combined_uncertainty_ppb(&self) -> f64 {
        (self.type_a_uncertainty_ppb.powi(2) + self.type_b_uncertainty_ppb.powi(2)).sqrt()
    }

    /// Expanded uncertainty (k=2, ~95% confidence) in ppb.
    pub fn expanded_uncertainty_ppb(&self) -> f64 {
        2.0 * self.combined_uncertainty_ppb()
    }
}

// ============================================================================
// Temperature effects
// ============================================================================

/// Models temperature dependence of QHE plateaus.
pub struct TemperatureEffects;

impl TemperatureEffects {
    /// Thermal excitation probability across half the Landau gap.
    /// P = exp(-delta_E / (2 * k_B * T))
    pub fn excitation_probability(b_field: f64, temperature_k: f64) -> f64 {
        let ll = LandauLevel::new(b_field);
        let activation = ll.activation_energy();
        (-activation / (BOLTZMANN_CONSTANT * temperature_k)).exp()
    }

    /// Plateau width in Tesla as a function of temperature.
    /// delta_B proportional to B * exp(-delta_E / (2*k_B*T))
    /// Returns the narrowing factor (1 = full width, 0 = collapsed).
    pub fn plateau_width_factor(b_field: f64, temperature_k: f64) -> f64 {
        let p_exc = Self::excitation_probability(b_field, temperature_k);
        (1.0 - p_exc).max(0.0)
    }

    /// R_xx deviation from zero on plateau due to thermal activation.
    /// R_xx ~ R_0 * exp(-delta_E / (2*k_B*T))
    pub fn r_xx_thermal_deviation(b_field: f64, temperature_k: f64, r_sheet: f64) -> f64 {
        r_sheet * Self::excitation_probability(b_field, temperature_k)
    }

    /// Maximum temperature for a given target R_xx deviation.
    /// T_max = -delta_E / (2 * k_B * ln(r_xx_target / r_sheet))
    pub fn max_temperature_for_deviation(
        b_field: f64,
        r_xx_target: f64,
        r_sheet: f64,
    ) -> f64 {
        assert!(r_xx_target > 0.0 && r_xx_target < r_sheet);
        let ll = LandauLevel::new(b_field);
        let activation = ll.activation_energy();
        -activation / (BOLTZMANN_CONSTANT * (r_xx_target / r_sheet).ln())
    }
}

// ============================================================================
// Current dependence
// ============================================================================

/// Models current-dependent breakdown of QHE quantization.
pub struct CurrentDependence;

impl CurrentDependence {
    /// Critical current for QHE breakdown.
    /// I_c ~ w * e/h * (eB - n_s * h) for filling factor slightly below integer.
    /// Simplified model: I_c proportional to width * excess Landau degeneracy.
    pub fn critical_current(config: &HallBarConfig, b_field: f64) -> f64 {
        let ll = LandauLevel::new(b_field);
        let n_l = ll.degeneracy_per_area();
        let excess = (n_l - config.carrier_density_m2).abs();
        // I_c ~ w * e * v_drift, where v_drift scales with excess density
        config.width_m * ELEMENTARY_CHARGE * excess * config.mobility_m2_per_vs
            / config.carrier_density_m2
            * b_field
    }

    /// Check if current exceeds breakdown threshold.
    pub fn is_breakdown(config: &HallBarConfig, b_field: f64, current: f64) -> bool {
        current > Self::critical_current(config, b_field)
    }

    /// R_xx at given current (finite above I_c).
    /// Below I_c: R_xx ~ 0 (on plateau).
    /// Above I_c: R_xx grows as (I - I_c)^2 / I_c^2 * R_sheet.
    pub fn r_xx_vs_current(
        config: &HallBarConfig,
        b_field: f64,
        current: f64,
    ) -> f64 {
        let i_c = Self::critical_current(config, b_field);
        if current <= i_c {
            return 0.0;
        }
        let excess = (current - i_c) / i_c;
        config.sheet_resistance() * excess * excess
    }

    /// Safe operating current: fraction of I_c for ppb-level accuracy.
    /// Typically 0.5 * I_c or less.
    pub fn safe_current(config: &HallBarConfig, b_field: f64, margin: f64) -> f64 {
        assert!((0.0..=1.0).contains(&margin), "margin must be in [0, 1]");
        margin * Self::critical_current(config, b_field)
    }
}

// ============================================================================
// Precision metrics
// ============================================================================

/// Precision assessment tools for QHR measurements.
pub struct PrecisionMetrics;

impl PrecisionMetrics {
    /// Deviation from exact quantization in parts per billion.
    pub fn deviation_ppb(measured: f64, expected: f64) -> f64 {
        (measured - expected) / expected * 1e9
    }

    /// Allan variance from a time series of resistance measurements.
    /// sigma^2(tau) = 1/(2*(N-1)) * sum((x_{i+1} - x_i)^2)
    pub fn allan_variance(measurements: &[f64]) -> f64 {
        if measurements.len() < 2 {
            return 0.0;
        }
        let n = measurements.len() - 1;
        let sum_sq: f64 = measurements
            .windows(2)
            .map(|w| {
                let diff = w[1] - w[0];
                diff * diff
            })
            .sum();
        sum_sq / (2.0 * n as f64)
    }

    /// Allan deviation (square root of Allan variance).
    pub fn allan_deviation(measurements: &[f64]) -> f64 {
        Self::allan_variance(measurements).sqrt()
    }

    /// Relative Allan deviation normalized by mean value, in ppb.
    pub fn relative_allan_deviation_ppb(measurements: &[f64]) -> f64 {
        if measurements.is_empty() {
            return 0.0;
        }
        let mean = measurements.iter().sum::<f64>() / measurements.len() as f64;
        if mean == 0.0 {
            return 0.0;
        }
        Self::allan_deviation(measurements) / mean * 1e9
    }

    /// Type A (statistical) uncertainty from repeated measurements.
    /// u_A = s / sqrt(n) where s is sample standard deviation.
    pub fn type_a_uncertainty(measurements: &[f64]) -> f64 {
        let n = measurements.len();
        if n < 2 {
            return f64::INFINITY;
        }
        let mean = measurements.iter().sum::<f64>() / n as f64;
        let variance =
            measurements.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
        variance.sqrt() / (n as f64).sqrt()
    }

    /// Combined uncertainty from Type A and Type B contributions.
    pub fn combined_uncertainty(type_a: f64, type_b: f64) -> f64 {
        (type_a * type_a + type_b * type_b).sqrt()
    }

    /// Standard deviation of a measurement series.
    pub fn std_deviation(measurements: &[f64]) -> f64 {
        let n = measurements.len();
        if n < 2 {
            return 0.0;
        }
        let mean = measurements.iter().sum::<f64>() / n as f64;
        let variance =
            measurements.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1) as f64;
        variance.sqrt()
    }
}

// ============================================================================
// Calibration chain
// ============================================================================

/// Resistance calibration traceability from QHR to working standards.
#[derive(Debug, Clone)]
pub struct CalibrationChain {
    /// Steps in the chain: (description, resistance in ohms, uncertainty ppb)
    steps: Vec<CalibrationStep>,
}

/// A single step in the calibration chain.
#[derive(Debug, Clone)]
pub struct CalibrationStep {
    pub description: String,
    pub resistance_ohms: f64,
    pub uncertainty_ppb: f64,
}

impl CalibrationChain {
    /// Create a new empty calibration chain.
    pub fn new() -> Self {
        Self { steps: Vec::new() }
    }

    /// Start from QHR at given filling factor.
    pub fn from_qhr(filling_factor: u32) -> Self {
        let r = hall_plateau_resistance(filling_factor);
        let mut chain = Self::new();
        chain.steps.push(CalibrationStep {
            description: format!("QHR i={}", filling_factor),
            resistance_ohms: r,
            uncertainty_ppb: 0.0, // Exact by definition
        });
        chain
    }

    /// Add a CCC bridge step to derive a new resistance.
    pub fn add_bridge_step(
        &mut self,
        bridge: &CccBridge,
        delta_ppb: f64,
        description: &str,
    ) {
        let last = self.steps.last().expect("chain must have at least one step");
        let new_r = bridge.derive_resistance(last.resistance_ohms, delta_ppb);
        let new_unc = (last.uncertainty_ppb.powi(2)
            + bridge.combined_uncertainty_ppb().powi(2))
        .sqrt();
        self.steps.push(CalibrationStep {
            description: description.to_string(),
            resistance_ohms: new_r,
            uncertainty_ppb: new_unc,
        });
    }

    /// Get all steps in the chain.
    pub fn steps(&self) -> &[CalibrationStep] {
        &self.steps
    }

    /// Final resistance value.
    pub fn final_resistance(&self) -> f64 {
        self.steps.last().map(|s| s.resistance_ohms).unwrap_or(0.0)
    }

    /// Total accumulated uncertainty in ppb.
    pub fn total_uncertainty_ppb(&self) -> f64 {
        self.steps.last().map(|s| s.uncertainty_ppb).unwrap_or(0.0)
    }

    /// Number of steps in the chain.
    pub fn num_steps(&self) -> usize {
        self.steps.len()
    }

    /// Derive standard resistances from i=2 QHR.
    /// Returns a chain that goes QHR(i=2) -> 10 kohm -> 1 kohm -> 100 ohm -> 1 ohm.
    pub fn standard_chain() -> Self {
        let mut chain = Self::from_qhr(2);

        // i=2 QHR (12906.4 ohm) -> 10 kohm via CCC
        let bridge_10k = CccBridge {
            n1: 12906,
            n2: 10000,
            type_a_uncertainty_ppb: 0.5,
            type_b_uncertainty_ppb: 0.3,
        };
        chain.add_bridge_step(&bridge_10k, 0.1, "10 kohm standard");

        // 10 kohm -> 1 kohm via CCC
        let bridge_1k = CccBridge {
            n1: 10,
            n2: 1,
            type_a_uncertainty_ppb: 0.3,
            type_b_uncertainty_ppb: 0.2,
        };
        chain.add_bridge_step(&bridge_1k, 0.05, "1 kohm standard");

        // 1 kohm -> 100 ohm via CCC
        let bridge_100 = CccBridge {
            n1: 10,
            n2: 1,
            type_a_uncertainty_ppb: 0.3,
            type_b_uncertainty_ppb: 0.2,
        };
        chain.add_bridge_step(&bridge_100, 0.02, "100 ohm standard");

        // 100 ohm -> 1 ohm via CCC
        let bridge_1 = CccBridge {
            n1: 100,
            n2: 1,
            type_a_uncertainty_ppb: 0.5,
            type_b_uncertainty_ppb: 0.4,
        };
        chain.add_bridge_step(&bridge_1, -0.01, "1 ohm standard");

        chain
    }
}

impl Default for CalibrationChain {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Uncertainty budget
// ============================================================================

/// Uncertainty budget entry for a QHR measurement.
#[derive(Debug, Clone)]
pub struct UncertaintyEntry {
    pub source: String,
    pub contribution_ppb: f64,
}

/// Build a full uncertainty budget for a QHR measurement.
pub fn uncertainty_budget(
    config: &HallBarConfig,
    b_field: f64,
    current: f64,
    bridge: &CccBridge,
) -> Vec<UncertaintyEntry> {
    let mut entries = Vec::new();

    // 1. Contact resistance
    entries.push(UncertaintyEntry {
        source: "Contact resistance".to_string(),
        contribution_ppb: 0.1,
    });

    // 2. Temperature effects
    let p_exc = TemperatureEffects::excitation_probability(b_field, config.temperature_k);
    let temp_ppb = p_exc * 1e9; // thermal excitation contributes proportionally
    entries.push(UncertaintyEntry {
        source: "Temperature excitation".to_string(),
        contribution_ppb: temp_ppb.min(100.0), // cap at 100 ppb for model validity
    });

    // 3. Current-induced breakdown
    let i_c = CurrentDependence::critical_current(config, b_field);
    let current_ratio = current / i_c;
    let current_ppb = if current_ratio < 0.5 {
        0.01
    } else if current_ratio < 0.8 {
        1.0
    } else {
        10.0
    };
    entries.push(UncertaintyEntry {
        source: "Current dependence".to_string(),
        contribution_ppb: current_ppb,
    });

    // 4. CCC bridge
    entries.push(UncertaintyEntry {
        source: "CCC bridge (Type A)".to_string(),
        contribution_ppb: bridge.type_a_uncertainty_ppb,
    });
    entries.push(UncertaintyEntry {
        source: "CCC bridge (Type B)".to_string(),
        contribution_ppb: bridge.type_b_uncertainty_ppb,
    });

    // 5. Leakage and guarding
    entries.push(UncertaintyEntry {
        source: "Leakage/guarding".to_string(),
        contribution_ppb: 0.05,
    });

    entries
}

/// Compute total combined uncertainty from a budget.
pub fn total_budget_uncertainty(budget: &[UncertaintyEntry]) -> f64 {
    budget
        .iter()
        .map(|e| e.contribution_ppb.powi(2))
        .sum::<f64>()
        .sqrt()
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // --- Physical constants ---

    #[test]
    fn test_von_klitzing_constant_value() {
        // R_K = h/e^2 ≈ 25812.80745 ohms
        assert!(
            (VON_KLITZING_CONSTANT - 25812.80745).abs() < 0.001,
            "R_K = {} should be ~25812.807",
            VON_KLITZING_CONSTANT
        );
    }

    #[test]
    fn test_hbar_value() {
        let expected = PLANCK_CONSTANT / (2.0 * PI);
        assert!((HBAR - expected).abs() < 1e-50);
    }

    #[test]
    fn test_gaas_effective_mass() {
        let expected = 0.067 * ELECTRON_MASS;
        assert!((GAAS_EFFECTIVE_MASS - expected).abs() < 1e-45);
    }

    // --- Hall plateaus ---

    #[test]
    fn test_hall_plateau_i1() {
        let r = hall_plateau_resistance(1);
        assert!(
            (r - 25812.807).abs() < 0.01,
            "i=1 plateau should be ~25812.8 ohm, got {}",
            r
        );
    }

    #[test]
    fn test_hall_plateau_i2() {
        let r = hall_plateau_resistance(2);
        assert!(
            (r - 12906.403).abs() < 0.01,
            "i=2 plateau should be ~12906.4 ohm, got {}",
            r
        );
    }

    #[test]
    fn test_hall_plateau_i4() {
        let r = hall_plateau_resistance(4);
        assert!(
            (r - 6453.20).abs() < 0.01,
            "i=4 plateau should be ~6453.2 ohm, got {}",
            r
        );
    }

    #[test]
    fn test_plateau_table() {
        let table = plateau_table(4);
        assert_eq!(table.len(), 4);
        assert_eq!(table[0].0, 1);
        assert_eq!(table[1].0, 2);
        // R_K/1 > R_K/2 > R_K/3 > R_K/4
        for w in table.windows(2) {
            assert!(w[0].1 > w[1].1);
        }
    }

    #[test]
    #[should_panic]
    fn test_hall_plateau_zero_panics() {
        hall_plateau_resistance(0);
    }

    // --- HallBarConfig ---

    #[test]
    fn test_default_gaas_config() {
        let bar = HallBarConfig::default_gaas();
        assert!((bar.width_m - 400.0e-6).abs() < 1e-10);
        assert!(bar.carrier_density_m2 > 1e15);
        assert!(bar.temperature_k > 0.0 && bar.temperature_k < 4.2);
    }

    #[test]
    fn test_classical_hall_coefficient() {
        let bar = HallBarConfig::default_gaas();
        let r_h = bar.classical_hall_coefficient();
        // R_H = 1/(n*e) ~ 1/(4.6e15 * 1.6e-19) ~ 1358 ohm/T
        assert!(r_h > 1000.0 && r_h < 2000.0, "R_H = {}", r_h);
    }

    #[test]
    fn test_sheet_resistance() {
        let bar = HallBarConfig::default_gaas();
        let r_s = bar.sheet_resistance();
        // Should be a reasonable value for GaAs 2DEG
        assert!(r_s > 0.0 && r_s < 1000.0, "R_sheet = {}", r_s);
    }

    #[test]
    fn test_scattering_time() {
        let bar = HallBarConfig::default_gaas();
        let tau = bar.scattering_time();
        // tau = m* * mu / e ~ 6.1e-32 * 50 / 1.6e-19 ~ 1.9e-11 s
        assert!(tau > 1e-12 && tau < 1e-9, "tau = {} s", tau);
    }

    #[test]
    fn test_magnetic_field_for_filling() {
        let bar = HallBarConfig::default_gaas();
        // B for i=2: B = n_s * h / (e * 2)
        let b2 = bar.magnetic_field_for_filling(2.0);
        assert!(b2 > 5.0 && b2 < 20.0, "B(i=2) = {} T", b2);
        // B for i=4 should be half of B for i=2
        let b4 = bar.magnetic_field_for_filling(4.0);
        assert!((b2 / b4 - 2.0).abs() < 0.01);
    }

    // --- Landau levels ---

    #[test]
    fn test_cyclotron_frequency() {
        let ll = LandauLevel::new(10.0);
        let fc = ll.cyclotron_frequency();
        // f_c = eB/(2*pi*m*) ~ 1.6e-19*10/(2*pi*6.1e-32) ~ 4.2 THz
        assert!(fc > 1e12 && fc < 1e13, "f_c = {} Hz", fc);
    }

    #[test]
    fn test_landau_energy_ordering() {
        let ll = LandauLevel::new(10.0);
        let e0 = ll.energy(0);
        let e1 = ll.energy(1);
        let e2 = ll.energy(2);
        assert!(e1 > e0);
        assert!(e2 > e1);
        // Equal spacing
        let gap1 = e1 - e0;
        let gap2 = e2 - e1;
        assert!((gap1 - gap2).abs() / gap1 < 1e-10);
    }

    #[test]
    fn test_landau_energy_gap() {
        let ll = LandauLevel::new(10.0);
        let gap = ll.energy_gap();
        let e0 = ll.energy(0);
        let e1 = ll.energy(1);
        assert!((gap - (e1 - e0)).abs() < 1e-30);
    }

    #[test]
    fn test_magnetic_length() {
        let ll = LandauLevel::new(10.0);
        let lb = ll.magnetic_length();
        // l_B = sqrt(hbar/(eB)) ~ sqrt(1.05e-34/(1.6e-19*10)) ~ 8.1 nm
        assert!(lb > 1e-9 && lb < 100e-9, "l_B = {} m", lb);
    }

    #[test]
    fn test_filling_factor() {
        let ll = LandauLevel::new(10.0);
        let bar = HallBarConfig::default_gaas();
        let nu = ll.filling_factor(bar.carrier_density_m2);
        // nu = n_s * h / (eB) ~ 4.6e15 * 6.63e-34 / (1.6e-19 * 10) ~ 1.9
        assert!(nu > 1.0 && nu < 5.0, "nu = {}", nu);
    }

    #[test]
    fn test_degeneracy_per_area() {
        let ll = LandauLevel::new(10.0);
        let n_l = ll.degeneracy_per_area();
        // n_L = eB/h ~ 1.6e-19*10/6.63e-34 ~ 2.41e15 m^-2
        assert!(n_l > 1e15 && n_l < 1e16, "n_L = {} m^-2", n_l);
    }

    #[test]
    fn test_gap_to_thermal_ratio() {
        let ll = LandauLevel::new(10.0);
        // At 1.3 K, ratio should be >> 1 for well-resolved plateaus
        let ratio = ll.gap_to_thermal_ratio(1.3);
        assert!(ratio > 10.0, "gap/kT = {}, should be >> 1", ratio);
    }

    // --- Magnetoresistance ---

    #[test]
    fn test_r_xy_zero_field() {
        let bar = HallBarConfig::default_gaas();
        let mr = MagnetoresistanceMeasurement::new(bar);
        assert_eq!(mr.r_xy(0.0), 0.0);
    }

    #[test]
    fn test_r_xy_on_plateau() {
        let bar = HallBarConfig::default_gaas();
        let b2 = bar.magnetic_field_for_filling(2.0);
        let mr = MagnetoresistanceMeasurement::new(bar);
        let r_xy = mr.r_xy(b2);
        let expected = hall_plateau_resistance(2);
        assert!(
            (r_xy - expected).abs() / expected < 1e-6,
            "R_xy = {} should be {} on i=2 plateau",
            r_xy,
            expected
        );
    }

    #[test]
    fn test_r_xx_on_plateau_is_small() {
        let bar = HallBarConfig::default_gaas();
        let mr = MagnetoresistanceMeasurement::new(bar.clone());
        let b2 = bar.magnetic_field_for_filling(2.0);
        let r_xx = mr.r_xx(b2);
        // R_xx should be exponentially small on plateau
        assert!(
            r_xx < 1.0,
            "R_xx = {} should be very small on plateau",
            r_xx
        );
    }

    #[test]
    fn test_r_xy_monotonically_increases_with_b() {
        let bar = HallBarConfig::default_gaas();
        let mr = MagnetoresistanceMeasurement::new(bar);
        let sweep = mr.sweep(1.0, 15.0, 100);
        for w in sweep.windows(2) {
            assert!(
                w[1].1 >= w[0].1 - 1e-6,
                "R_xy should be non-decreasing: {} -> {} at B={} -> {}",
                w[0].1,
                w[1].1,
                w[0].0,
                w[1].0
            );
        }
    }

    #[test]
    fn test_sweep_returns_correct_count() {
        let bar = HallBarConfig::default_gaas();
        let mr = MagnetoresistanceMeasurement::new(bar);
        let sweep = mr.sweep(0.0, 15.0, 50);
        assert_eq!(sweep.len(), 50);
    }

    // --- CCC Bridge ---

    #[test]
    fn test_ccc_ideal_ratio() {
        let bridge = CccBridge::one_to_one();
        assert!((bridge.ideal_ratio() - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_ccc_derive_resistance() {
        let bridge = CccBridge::qhr_to_10k();
        let r_qhr = hall_plateau_resistance(2); // 12906.4 ohm
        let r_10k = bridge.derive_resistance(r_qhr, 0.0);
        // 12906.4 * 10000/12906 ≈ 10000.03 (not exactly 10k due to integer ratio)
        assert!(
            (r_10k - 10000.0).abs() < 1.0,
            "derived 10k = {}",
            r_10k
        );
    }

    #[test]
    fn test_ccc_measured_ratio_with_correction() {
        let bridge = CccBridge::one_to_one();
        let ratio = bridge.measured_ratio(1.0); // 1 ppb correction
        assert!((ratio - 1.000_000_001).abs() < 1e-12);
    }

    #[test]
    fn test_ccc_combined_uncertainty() {
        let bridge = CccBridge {
            n1: 1,
            n2: 1,
            type_a_uncertainty_ppb: 3.0,
            type_b_uncertainty_ppb: 4.0,
        };
        let u_c = bridge.combined_uncertainty_ppb();
        assert!((u_c - 5.0).abs() < TOLERANCE); // 3-4-5 triangle
    }

    #[test]
    fn test_ccc_expanded_uncertainty() {
        let bridge = CccBridge {
            n1: 1,
            n2: 1,
            type_a_uncertainty_ppb: 3.0,
            type_b_uncertainty_ppb: 4.0,
        };
        assert!((bridge.expanded_uncertainty_ppb() - 10.0).abs() < TOLERANCE);
    }

    // --- Temperature effects ---

    #[test]
    fn test_excitation_probability_low_temp() {
        // At very low temperature, excitation should be negligible
        let p = TemperatureEffects::excitation_probability(10.0, 0.3);
        assert!(p < 1e-10, "P_exc = {} at 0.3K", p);
    }

    #[test]
    fn test_excitation_probability_increases_with_temp() {
        let p1 = TemperatureEffects::excitation_probability(10.0, 1.0);
        let p2 = TemperatureEffects::excitation_probability(10.0, 4.0);
        assert!(p2 > p1, "excitation should increase with temperature");
    }

    #[test]
    fn test_plateau_width_factor_cold() {
        let f = TemperatureEffects::plateau_width_factor(10.0, 0.3);
        assert!(f > 0.99, "plateau should be fully formed at 0.3K: factor = {}", f);
    }

    #[test]
    fn test_max_temperature_for_deviation() {
        let bar = HallBarConfig::default_gaas();
        let r_sheet = bar.sheet_resistance();
        let t_max = TemperatureEffects::max_temperature_for_deviation(10.0, 1e-6, r_sheet);
        assert!(
            t_max > 0.0 && t_max < 10.0,
            "T_max = {} K for 1 uohm deviation",
            t_max
        );
    }

    // --- Current dependence ---

    #[test]
    fn test_critical_current_positive() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let i_c = CurrentDependence::critical_current(&bar, b);
        assert!(i_c > 0.0, "I_c = {} should be positive", i_c);
    }

    #[test]
    fn test_breakdown_below_ic() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let i_c = CurrentDependence::critical_current(&bar, b);
        assert!(!CurrentDependence::is_breakdown(&bar, b, 0.1 * i_c));
    }

    #[test]
    fn test_breakdown_above_ic() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let i_c = CurrentDependence::critical_current(&bar, b);
        assert!(CurrentDependence::is_breakdown(&bar, b, 2.0 * i_c));
    }

    #[test]
    fn test_r_xx_zero_below_ic() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let i_c = CurrentDependence::critical_current(&bar, b);
        let r_xx = CurrentDependence::r_xx_vs_current(&bar, b, 0.1 * i_c);
        assert_eq!(r_xx, 0.0);
    }

    #[test]
    fn test_r_xx_finite_above_ic() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let i_c = CurrentDependence::critical_current(&bar, b);
        let r_xx = CurrentDependence::r_xx_vs_current(&bar, b, 2.0 * i_c);
        assert!(r_xx > 0.0, "R_xx = {} should be > 0 above I_c", r_xx);
    }

    #[test]
    fn test_safe_current() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let i_c = CurrentDependence::critical_current(&bar, b);
        let i_safe = CurrentDependence::safe_current(&bar, b, 0.5);
        assert!((i_safe - 0.5 * i_c).abs() < TOLERANCE);
    }

    // --- Precision metrics ---

    #[test]
    fn test_deviation_ppb() {
        let expected = 12906.4037;
        let measured = 12906.4037 + 12906.4037 * 1e-9; // 1 ppb high
        let dev = PrecisionMetrics::deviation_ppb(measured, expected);
        assert!((dev - 1.0).abs() < 0.01, "deviation = {} ppb", dev);
    }

    #[test]
    fn test_allan_variance_constant() {
        // Constant values -> zero Allan variance
        let data = vec![12906.4; 100];
        let avar = PrecisionMetrics::allan_variance(&data);
        assert!(avar < 1e-20, "AVAR = {} for constant data", avar);
    }

    #[test]
    fn test_allan_variance_alternating() {
        // Alternating +/- delta gives maximum Allan variance
        let delta = 1e-6;
        let data: Vec<f64> = (0..100)
            .map(|i| 12906.4 + if i % 2 == 0 { delta } else { -delta })
            .collect();
        let avar = PrecisionMetrics::allan_variance(&data);
        // For alternating +d, -d: differences are 2*delta, so AVAR = (2*delta)^2/2 = 2*delta^2
        let expected = 2.0 * delta * delta;
        assert!(
            (avar - expected).abs() / expected < 0.02,
            "AVAR = {}, expected {}",
            avar,
            expected
        );
    }

    #[test]
    fn test_type_a_uncertainty() {
        let data = vec![100.0, 100.1, 99.9, 100.05, 99.95];
        let u_a = PrecisionMetrics::type_a_uncertainty(&data);
        assert!(u_a > 0.0 && u_a < 0.1, "u_A = {}", u_a);
    }

    #[test]
    fn test_combined_uncertainty_pythagorean() {
        let u = PrecisionMetrics::combined_uncertainty(3.0, 4.0);
        assert!((u - 5.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_relative_allan_deviation_ppb() {
        let data = vec![12906.4037; 50];
        let rad = PrecisionMetrics::relative_allan_deviation_ppb(&data);
        assert!(rad < 1e-6, "relative ADEV = {} ppb for constant data", rad);
    }

    // --- Calibration chain ---

    #[test]
    fn test_calibration_chain_from_qhr() {
        let chain = CalibrationChain::from_qhr(2);
        assert_eq!(chain.num_steps(), 1);
        let r = chain.final_resistance();
        assert!((r - hall_plateau_resistance(2)).abs() < TOLERANCE);
        assert_eq!(chain.total_uncertainty_ppb(), 0.0);
    }

    #[test]
    fn test_standard_chain() {
        let chain = CalibrationChain::standard_chain();
        assert_eq!(chain.num_steps(), 5);
        // Final step should be ~1 ohm
        let r_final = chain.final_resistance();
        assert!(
            (r_final - 1.0).abs() < 0.1,
            "final = {} ohm, should be ~1 ohm",
            r_final
        );
        // Uncertainty should accumulate
        assert!(chain.total_uncertainty_ppb() > 0.0);
    }

    #[test]
    fn test_calibration_chain_uncertainty_grows() {
        let chain = CalibrationChain::standard_chain();
        let steps = chain.steps();
        for w in steps.windows(2) {
            assert!(
                w[1].uncertainty_ppb >= w[0].uncertainty_ppb,
                "uncertainty should not decrease in chain"
            );
        }
    }

    // --- Uncertainty budget ---

    #[test]
    fn test_uncertainty_budget_entries() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.0);
        let bridge = CccBridge::one_to_one();
        let budget = uncertainty_budget(&bar, b, 1e-6, &bridge);
        assert!(budget.len() >= 5, "budget should have multiple entries");
    }

    #[test]
    fn test_total_budget_uncertainty() {
        let entries = vec![
            UncertaintyEntry {
                source: "A".to_string(),
                contribution_ppb: 3.0,
            },
            UncertaintyEntry {
                source: "B".to_string(),
                contribution_ppb: 4.0,
            },
        ];
        let total = total_budget_uncertainty(&entries);
        assert!((total - 5.0).abs() < TOLERANCE);
    }

    // --- Edge cases ---

    #[test]
    fn test_std_deviation_single_value() {
        let s = PrecisionMetrics::std_deviation(&[42.0]);
        assert_eq!(s, 0.0);
    }

    #[test]
    fn test_allan_variance_empty() {
        let avar = PrecisionMetrics::allan_variance(&[]);
        assert_eq!(avar, 0.0);
    }

    #[test]
    fn test_landau_level_ground_state() {
        let ll = LandauLevel::new(10.0);
        let e0 = ll.energy(0);
        // E_0 = hbar * omega_c / 2 = half the gap
        assert!((e0 - ll.energy_gap() / 2.0).abs() < 1e-30);
    }

    #[test]
    fn test_filled_levels() {
        let bar = HallBarConfig::default_gaas();
        let b = bar.magnetic_field_for_filling(2.5);
        let ll = LandauLevel::new(b);
        let filled = ll.filled_levels(bar.carrier_density_m2);
        assert_eq!(filled, 2, "at nu=2.5, 2 levels should be completely filled");
    }
}
