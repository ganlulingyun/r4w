//! Neutron porosity well-log analysis for formation evaluation.
//!
//! This module implements signal processing and petrophysical interpretation for
//! compensated neutron porosity (CNP) well logging tools. Applications include
//! petroleum exploration, groundwater assessment, formation evaluation, and
//! mineral exploration.
//!
//! # Physics Background
//!
//! A neutron source (AmBe, PuBe, or D-T generator) emits fast neutrons into the
//! formation. Hydrogen nuclei in pore fluids are the most effective moderators,
//! slowing the neutrons to thermal and epithermal energies. Near and far detectors
//! measure count rates; the ratio compensates for borehole effects and yields
//! apparent porosity.
//!
//! Higher hydrogen content (higher porosity) → more moderation → fewer counts
//! reaching the far detector → higher near/far ratio.
//!
//! # Key Equations
//!
//! **Porosity from ratio method:**
//! ```text
//! phi = a + b * ln(R)      where R = near_counts / far_counts
//! ```
//!
//! **Lithology correction:**
//! ```text
//! phi_true = phi_apparent + delta_phi_lithology
//! ```
//! Offsets: sandstone −2 p.u., limestone 0 p.u., dolomite +1 p.u. (referenced
//! to a limestone matrix).
//!
//! **Salinity correction:**
//! ```text
//! phi_corr = phi * (1 + k * C_sal)
//! ```
//! where `C_sal` is NaCl concentration in ppm and `k` is a tool-dependent
//! coefficient (~1.0e-7 per ppm).
//!
//! **Gas detection (neutron-density crossover):**
//! ```text
//! separation = phi_density - phi_neutron
//! ```
//! Positive separation indicates gas-filled porosity.
//!
//! **Shale volume (neutron-density crossplot):**
//! ```text
//! V_sh = (phi_N - phi_N_clean) / (phi_N_shale - phi_N_clean)
//! ```
//!
//! **Sigma log (thermal neutron capture cross-section):**
//! ```text
//! Sigma = Sigma_ma*(1-phi) + phi*(Sw*Sigma_w + (1-Sw)*Sigma_hc)
//! ```
//! Units: capture units (c.u.) where 1 c.u. = 10^-3 cm^-1.
//!
//! # Example
//!
//! ```
//! use r4w_core::neutron_porosity_analyzer::{
//!     NeutronPorosityConfig, ToolType, SourceType,
//!     PorosityCalculator, MatrixType,
//!     LithologyCorrector, GasDetector,
//! };
//!
//! // Configure a compensated neutron tool
//! let config = NeutronPorosityConfig {
//!     tool_type: ToolType::Compensated,
//!     source_type: SourceType::AmBe,
//!     near_detector_spacing_m: 0.30,
//!     far_detector_spacing_m: 0.60,
//!     near_detector_efficiency: 0.85,
//!     far_detector_efficiency: 0.80,
//! };
//!
//! // Compute porosity from count-rate ratio
//! let calc = PorosityCalculator::limestone();
//! let phi = calc.porosity_from_ratio(2.5);
//! assert!(phi > 0.0 && phi < 1.0);
//!
//! // Lithology correction: limestone apparent → sandstone true
//! let corrector = LithologyCorrector::new();
//! let phi_ss = corrector.correct(phi, MatrixType::Sandstone);
//! assert!(phi_ss < phi); // sandstone reads lower
//!
//! // Gas detection
//! let gas = GasDetector::default();
//! let sep = gas.separation(0.25, 0.10); // phi_density=0.25, phi_neutron=0.10
//! assert!(gas.is_gas(sep));
//! ```

// ─────────────────────────────────────────────────────────────────────
// Configuration and enums
// ─────────────────────────────────────────────────────────────────────

/// Neutron source type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceType {
    /// Americium-241 / Beryllium chemical source (~4.5 MeV avg).
    AmBe,
    /// Plutonium-238 / Beryllium chemical source (~4.2 MeV avg).
    PuBe,
    /// Deuterium-Tritium electronic neutron generator (14.1 MeV).
    DtGenerator,
}

/// Neutron tool type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ToolType {
    /// Compensated dual-detector tool (near + far).
    Compensated,
    /// Single sidewall-pad detector tool.
    Sidewall,
}

/// Lithology matrix type for porosity correction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MatrixType {
    /// Limestone (CaCO3) — reference matrix.
    Limestone,
    /// Sandstone (SiO2).
    Sandstone,
    /// Dolomite (CaMg(CO3)2).
    Dolomite,
}

/// Detector type for epithermal vs thermal neutron processing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType {
    /// Thermal neutron detector (He-3 proportional counter).
    Thermal,
    /// Epithermal neutron detector (reduced salinity sensitivity).
    Epithermal,
}

/// Configuration for a compensated neutron porosity tool.
#[derive(Debug, Clone)]
pub struct NeutronPorosityConfig {
    /// Tool type (compensated or sidewall).
    pub tool_type: ToolType,
    /// Neutron source type.
    pub source_type: SourceType,
    /// Near detector spacing from source (metres).
    pub near_detector_spacing_m: f64,
    /// Far detector spacing from source (metres).
    pub far_detector_spacing_m: f64,
    /// Near detector counting efficiency (0–1).
    pub near_detector_efficiency: f64,
    /// Far detector counting efficiency (0–1).
    pub far_detector_efficiency: f64,
}

impl Default for NeutronPorosityConfig {
    fn default() -> Self {
        Self {
            tool_type: ToolType::Compensated,
            source_type: SourceType::AmBe,
            near_detector_spacing_m: 0.30,
            far_detector_spacing_m: 0.60,
            near_detector_efficiency: 0.85,
            far_detector_efficiency: 0.80,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Porosity calculator
// ─────────────────────────────────────────────────────────────────────

/// Converts near/far detector count-rate ratio to porosity.
///
/// Uses the logarithmic ratio method:
/// ```text
/// phi = a + b * ln(ratio)
/// ```
/// Calibration constants `a` and `b` depend on the matrix lithology.
#[derive(Debug, Clone)]
pub struct PorosityCalculator {
    /// Intercept of ratio–porosity calibration.
    pub a: f64,
    /// Slope of ratio–porosity calibration.
    pub b: f64,
    /// Minimum porosity clamp (fraction).
    pub phi_min: f64,
    /// Maximum porosity clamp (fraction).
    pub phi_max: f64,
}

impl PorosityCalculator {
    /// Create a calculator with arbitrary calibration constants.
    pub fn new(a: f64, b: f64) -> Self {
        Self {
            a,
            b,
            phi_min: 0.0,
            phi_max: 0.60,
        }
    }

    /// Limestone calibration (reference matrix).
    ///
    /// At ratio=1.0 → phi≈0, at ratio=e^(0.40/0.30)≈3.79 → phi≈0.40.
    pub fn limestone() -> Self {
        // Calibrated so phi(1.0)≈0 and phi(3.79)≈0.40
        Self::new(-0.02, 0.30)
    }

    /// Sandstone calibration.
    pub fn sandstone() -> Self {
        Self::new(-0.04, 0.28)
    }

    /// Dolomite calibration.
    pub fn dolomite() -> Self {
        Self::new(-0.01, 0.32)
    }

    /// Compute apparent porosity from near/far count-rate ratio.
    ///
    /// Returns porosity as a fraction (0 to phi_max).
    pub fn porosity_from_ratio(&self, ratio: f64) -> f64 {
        if ratio <= 0.0 {
            return self.phi_min;
        }
        let phi = self.a + self.b * ratio.ln();
        phi.clamp(self.phi_min, self.phi_max)
    }

    /// Compute apparent porosity from raw near and far count rates.
    pub fn porosity_from_counts(&self, near_counts: f64, far_counts: f64) -> f64 {
        if far_counts <= 0.0 {
            return self.phi_max;
        }
        self.porosity_from_ratio(near_counts / far_counts)
    }

    /// Inverse: compute expected ratio for a given porosity.
    pub fn ratio_from_porosity(&self, phi: f64) -> f64 {
        ((phi - self.a) / self.b).exp()
    }

    /// Process a depth log of count-rate ratios into porosity values.
    pub fn process_log(&self, ratios: &[f64]) -> Vec<f64> {
        ratios.iter().map(|r| self.porosity_from_ratio(*r)).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Lithology correction
// ─────────────────────────────────────────────────────────────────────

/// Lithology matrix-effect corrections.
///
/// Neutron tools are calibrated on a limestone reference. Other lithologies
/// produce systematic offsets:
/// - Sandstone: −2 porosity units (−0.02)
/// - Dolomite:  +1 porosity unit  (+0.01)
/// - Limestone:  0 (reference)
#[derive(Debug, Clone)]
pub struct LithologyCorrector {
    /// Sandstone offset (fraction, typically −0.02).
    pub sandstone_offset: f64,
    /// Dolomite offset (fraction, typically +0.01).
    pub dolomite_offset: f64,
}

impl LithologyCorrector {
    /// Create with standard API-calibrated offsets.
    pub fn new() -> Self {
        Self {
            sandstone_offset: -0.02,
            dolomite_offset: 0.01,
        }
    }

    /// Create with custom offsets.
    pub fn with_offsets(sandstone_offset: f64, dolomite_offset: f64) -> Self {
        Self {
            sandstone_offset,
            dolomite_offset,
        }
    }

    /// Matrix offset for a given lithology (porosity units as fraction).
    pub fn offset(&self, matrix: MatrixType) -> f64 {
        match matrix {
            MatrixType::Limestone => 0.0,
            MatrixType::Sandstone => self.sandstone_offset,
            MatrixType::Dolomite => self.dolomite_offset,
        }
    }

    /// Correct apparent (limestone-calibrated) porosity for true lithology.
    pub fn correct(&self, phi_apparent: f64, matrix: MatrixType) -> f64 {
        (phi_apparent + self.offset(matrix)).max(0.0)
    }

    /// Process a depth log with lithology correction.
    pub fn correct_log(&self, phi_log: &[f64], matrix: MatrixType) -> Vec<f64> {
        phi_log.iter().map(|phi| self.correct(*phi, matrix)).collect()
    }
}

impl Default for LithologyCorrector {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Salinity correction
// ─────────────────────────────────────────────────────────────────────

/// Borehole fluid salinity correction.
///
/// Chlorine has a high thermal neutron capture cross-section, reducing
/// thermal neutron counts and biasing porosity high in saline muds.
///
/// ```text
/// phi_corr = phi * (1 + k * C_sal)
/// ```
///
/// where `C_sal` is NaCl concentration in ppm and `k` is a tool-dependent
/// coefficient.
#[derive(Debug, Clone)]
pub struct SalinityCorrector {
    /// Salinity coefficient (per ppm). Typical: ~1.0e-7 for thermal tools.
    pub k: f64,
}

impl SalinityCorrector {
    /// Create for a thermal neutron tool.
    pub fn thermal() -> Self {
        Self { k: 1.0e-7 }
    }

    /// Create for an epithermal neutron tool (reduced salinity sensitivity).
    pub fn epithermal() -> Self {
        Self { k: 2.0e-8 }
    }

    /// Create with a custom salinity coefficient.
    pub fn with_coefficient(k: f64) -> Self {
        Self { k }
    }

    /// Correct porosity for borehole fluid salinity.
    ///
    /// * `phi` — apparent porosity (fraction)
    /// * `salinity_ppm` — NaCl concentration in ppm
    pub fn correct(&self, phi: f64, salinity_ppm: f64) -> f64 {
        (phi * (1.0 + self.k * salinity_ppm)).max(0.0)
    }

    /// Process a depth log with constant salinity.
    pub fn correct_log(&self, phi_log: &[f64], salinity_ppm: f64) -> Vec<f64> {
        phi_log.iter().map(|phi| self.correct(*phi, salinity_ppm)).collect()
    }
}

impl Default for SalinityCorrector {
    fn default() -> Self {
        Self::thermal()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Gas detection
// ─────────────────────────────────────────────────────────────────────

/// Gas detection from neutron-density crossover.
///
/// Gas has a very low hydrogen index compared to liquids. In gas-bearing
/// zones the neutron porosity reads low (less hydrogen) while density
/// porosity reads high (low bulk density). The separation is diagnostic:
///
/// ```text
/// separation = phi_density - phi_neutron
/// ```
///
/// Positive separation indicates gas.
#[derive(Debug, Clone)]
pub struct GasDetector {
    /// Minimum separation to declare gas (fraction). Default 0.04.
    pub threshold: f64,
}

impl GasDetector {
    /// Create with a custom separation threshold.
    pub fn with_threshold(threshold: f64) -> Self {
        Self { threshold }
    }

    /// Compute neutron-density separation.
    ///
    /// * `phi_density` — density-derived porosity (fraction)
    /// * `phi_neutron` — neutron-derived porosity (fraction)
    pub fn separation(&self, phi_density: f64, phi_neutron: f64) -> f64 {
        phi_density - phi_neutron
    }

    /// Returns `true` if the separation indicates gas.
    pub fn is_gas(&self, separation: f64) -> bool {
        separation > self.threshold
    }

    /// Classify a depth log into gas/no-gas flags.
    pub fn detect_log(&self, phi_density: &[f64], phi_neutron: &[f64]) -> Vec<bool> {
        phi_density
            .iter()
            .zip(phi_neutron.iter())
            .map(|(pd, pn)| self.is_gas(self.separation(*pd, *pn)))
            .collect()
    }

    /// Compute separations for a full depth log.
    pub fn separation_log(&self, phi_density: &[f64], phi_neutron: &[f64]) -> Vec<f64> {
        phi_density
            .iter()
            .zip(phi_neutron.iter())
            .map(|(pd, pn)| self.separation(*pd, *pn))
            .collect()
    }

    /// Estimate gas saturation from separation (simplified model).
    ///
    /// ```text
    /// Sg ≈ separation / (phi_density * (HI_liquid - HI_gas))
    /// ```
    ///
    /// where HI_liquid ≈ 1.0 and HI_gas ≈ 0.0 for a first approximation.
    pub fn estimate_gas_saturation(
        &self,
        phi_density: f64,
        phi_neutron: f64,
        hi_gas: f64,
    ) -> f64 {
        if phi_density <= 0.0 {
            return 0.0;
        }
        let sep = self.separation(phi_density, phi_neutron);
        if sep <= 0.0 {
            return 0.0;
        }
        let sg = sep / (phi_density * (1.0 - hi_gas));
        sg.clamp(0.0, 1.0)
    }
}

impl Default for GasDetector {
    fn default() -> Self {
        Self { threshold: 0.04 }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Shale volume estimator
// ─────────────────────────────────────────────────────────────────────

/// Shale volume from neutron-density crossplot.
///
/// ```text
/// V_sh = (phi_N - phi_N_clean) / (phi_N_shale - phi_N_clean)
/// ```
///
/// The neutron log reads higher in shaly zones due to bound water in
/// clay minerals.
#[derive(Debug, Clone)]
pub struct ShaleVolumeEstimator {
    /// Neutron porosity at the clean (sand/carbonate) endpoint (fraction).
    pub phi_n_clean: f64,
    /// Neutron porosity at the 100% shale endpoint (fraction).
    pub phi_n_shale: f64,
}

impl ShaleVolumeEstimator {
    /// Create with clean and shale endpoint neutron porosities.
    pub fn new(phi_n_clean: f64, phi_n_shale: f64) -> Self {
        Self {
            phi_n_clean,
            phi_n_shale,
        }
    }

    /// Typical sand-shale system: clean sand phi_N = 0.05, shale phi_N = 0.40.
    pub fn sand_shale() -> Self {
        Self::new(0.05, 0.40)
    }

    /// Typical carbonate-shale system: clean limestone phi_N = 0.0, shale = 0.35.
    pub fn carbonate_shale() -> Self {
        Self::new(0.0, 0.35)
    }

    /// Compute shale volume (V_sh) from neutron porosity reading.
    ///
    /// Returns a fraction 0–1.
    pub fn v_shale(&self, phi_neutron: f64) -> f64 {
        let denom = self.phi_n_shale - self.phi_n_clean;
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        let vsh = (phi_neutron - self.phi_n_clean) / denom;
        vsh.clamp(0.0, 1.0)
    }

    /// Process a depth log of neutron porosities into V_shale.
    pub fn process_log(&self, phi_neutron: &[f64]) -> Vec<f64> {
        phi_neutron.iter().map(|pn| self.v_shale(*pn)).collect()
    }

    /// Clavier non-linear shale correction.
    ///
    /// ```text
    /// V_sh_clavier = 1.7 - sqrt(3.38 - (V_sh_linear + 0.7)^2)
    /// ```
    pub fn v_shale_clavier(&self, phi_neutron: f64) -> f64 {
        let vsh_lin = self.v_shale(phi_neutron);
        let arg = 3.38 - (vsh_lin + 0.7) * (vsh_lin + 0.7);
        if arg < 0.0 {
            return 1.0;
        }
        let vsh = 1.7 - arg.sqrt();
        vsh.clamp(0.0, 1.0)
    }

    /// Steiber non-linear shale correction.
    ///
    /// ```text
    /// V_sh_steiber = V_sh_linear / (3.0 - 2.0 * V_sh_linear)
    /// ```
    pub fn v_shale_steiber(&self, phi_neutron: f64) -> f64 {
        let vsh_lin = self.v_shale(phi_neutron);
        let denom = 3.0 - 2.0 * vsh_lin;
        if denom.abs() < 1e-12 {
            return 1.0;
        }
        let vsh = vsh_lin / denom;
        vsh.clamp(0.0, 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────
// Environmental corrections
// ─────────────────────────────────────────────────────────────────────

/// Borehole environmental corrections for neutron porosity.
///
/// Accounts for:
/// - Borehole diameter (larger hole → more standoff → higher counts)
/// - Mud weight (barite-loaded mud absorbs neutrons)
/// - Temperature (detector efficiency varies)
/// - Pressure (formation fluid density changes)
#[derive(Debug, Clone)]
pub struct EnvironmentalCorrector {
    /// Reference borehole diameter (metres). Typically 0.2159 m (8.5 in).
    pub reference_hole_diameter_m: f64,
    /// Reference mud weight (kg/m³). Typically 1080 (9.0 ppg).
    pub reference_mud_weight_kg_m3: f64,
    /// Reference temperature (°C). Typically 24.
    pub reference_temperature_c: f64,
    /// Reference pressure (MPa). Typically 0.1 (atmospheric).
    pub reference_pressure_mpa: f64,
    /// Borehole size correction coefficient (porosity units per metre deviation).
    pub hole_size_coeff: f64,
    /// Mud weight correction coefficient (porosity units per kg/m³ deviation).
    pub mud_weight_coeff: f64,
    /// Temperature correction coefficient (porosity units per °C deviation).
    pub temperature_coeff: f64,
    /// Pressure correction coefficient (porosity units per MPa deviation).
    pub pressure_coeff: f64,
}

impl EnvironmentalCorrector {
    /// Default corrections for a compensated neutron tool.
    pub fn new() -> Self {
        Self {
            reference_hole_diameter_m: 0.2159, // 8.5 inches
            reference_mud_weight_kg_m3: 1080.0, // 9.0 ppg
            reference_temperature_c: 24.0,
            reference_pressure_mpa: 0.1,
            // Typical correction sensitivities
            hole_size_coeff: 0.05,     // 0.05 p.u. per cm deviation ≈ 5 p.u./m
            mud_weight_coeff: 2.0e-5,  // per kg/m³
            temperature_coeff: 1.5e-4, // per °C
            pressure_coeff: 5.0e-4,    // per MPa
        }
    }

    /// Borehole diameter correction (porosity fraction).
    pub fn hole_size_correction(&self, actual_diameter_m: f64) -> f64 {
        self.hole_size_coeff * (actual_diameter_m - self.reference_hole_diameter_m)
    }

    /// Mud weight correction (porosity fraction).
    pub fn mud_weight_correction(&self, actual_mud_weight_kg_m3: f64) -> f64 {
        self.mud_weight_coeff * (actual_mud_weight_kg_m3 - self.reference_mud_weight_kg_m3)
    }

    /// Temperature correction (porosity fraction).
    pub fn temperature_correction(&self, actual_temperature_c: f64) -> f64 {
        self.temperature_coeff * (actual_temperature_c - self.reference_temperature_c)
    }

    /// Pressure correction (porosity fraction).
    pub fn pressure_correction(&self, actual_pressure_mpa: f64) -> f64 {
        self.pressure_coeff * (actual_pressure_mpa - self.reference_pressure_mpa)
    }

    /// Total environmental correction (porosity fraction).
    pub fn total_correction(
        &self,
        hole_diameter_m: f64,
        mud_weight_kg_m3: f64,
        temperature_c: f64,
        pressure_mpa: f64,
    ) -> f64 {
        self.hole_size_correction(hole_diameter_m)
            + self.mud_weight_correction(mud_weight_kg_m3)
            + self.temperature_correction(temperature_c)
            + self.pressure_correction(pressure_mpa)
    }

    /// Apply all environmental corrections to apparent porosity.
    pub fn correct(
        &self,
        phi_apparent: f64,
        hole_diameter_m: f64,
        mud_weight_kg_m3: f64,
        temperature_c: f64,
        pressure_mpa: f64,
    ) -> f64 {
        let corr =
            self.total_correction(hole_diameter_m, mud_weight_kg_m3, temperature_c, pressure_mpa);
        (phi_apparent + corr).max(0.0)
    }
}

impl Default for EnvironmentalCorrector {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Epithermal detector processing
// ─────────────────────────────────────────────────────────────────────

/// Epithermal neutron processing.
///
/// Epithermal detectors respond to neutrons above the thermal energy
/// cut-off (~0.4 eV). This makes them largely insensitive to thermal
/// neutron absorbers such as chlorine and boron, which cause spuriously
/// high porosity readings on thermal tools in saline environments.
#[derive(Debug, Clone)]
pub struct EpithermalDetector {
    /// Energy threshold (eV) separating epithermal from thermal.
    pub energy_threshold_ev: f64,
    /// Cadmium cut-off ratio (epithermal/thermal sensitivity).
    pub cadmium_ratio: f64,
    /// Porosity calibration for epithermal counts.
    pub porosity_calc: PorosityCalculator,
}

impl EpithermalDetector {
    /// Create with standard cadmium cut-off (~0.4 eV).
    pub fn new() -> Self {
        Self {
            energy_threshold_ev: 0.4,
            cadmium_ratio: 3.0,
            porosity_calc: PorosityCalculator::new(-0.01, 0.29),
        }
    }

    /// Compute epithermal porosity from ratio.
    pub fn porosity_from_ratio(&self, ratio: f64) -> f64 {
        self.porosity_calc.porosity_from_ratio(ratio)
    }

    /// Compute the thermal/epithermal ratio for salinity estimation.
    ///
    /// A high ratio indicates significant thermal absorption (high salinity).
    pub fn thermal_epithermal_ratio(
        &self,
        thermal_counts: f64,
        epithermal_counts: f64,
    ) -> f64 {
        if epithermal_counts <= 0.0 {
            return 0.0;
        }
        thermal_counts / epithermal_counts
    }

    /// Estimate apparent salinity from thermal/epithermal ratio difference
    /// relative to a fresh-water baseline.
    ///
    /// Returns estimated NaCl equivalent in ppm.
    pub fn estimate_salinity_ppm(&self, t_e_ratio: f64, freshwater_ratio: f64) -> f64 {
        // Empirical: roughly 50_000 ppm per unit ratio deviation
        let delta = t_e_ratio - freshwater_ratio;
        (delta * 50_000.0).max(0.0)
    }
}

impl Default for EpithermalDetector {
    fn default() -> Self {
        Self::new()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Sigma log processor
// ─────────────────────────────────────────────────────────────────────

/// Thermal neutron capture cross-section (Sigma) log processor.
///
/// Sigma logging measures the macroscopic thermal neutron absorption
/// cross-section of the formation. It is particularly useful for
/// determining water saturation in saline formation waters where
/// chlorine provides a strong contrast:
///
/// ```text
/// Sigma_formation = Sigma_ma*(1-phi) + phi*(Sw*Sigma_w + (1-Sw)*Sigma_hc)
/// ```
///
/// Typical Sigma values (capture units, c.u.):
/// - Fresh water: ~22
/// - Oil: ~20
/// - Gas: ~8
/// - Sandstone matrix: ~8
/// - Limestone matrix: ~12
/// - Dolomite matrix: ~9
/// - Shale: ~30–50
///
/// 1 c.u. = 10⁻³ cm⁻¹
#[derive(Debug, Clone)]
pub struct SigmaLogProcessor {
    /// Matrix capture cross-section (c.u.).
    pub sigma_matrix: f64,
    /// Formation water capture cross-section (c.u.).
    pub sigma_water: f64,
    /// Hydrocarbon capture cross-section (c.u.).
    pub sigma_hc: f64,
}

impl SigmaLogProcessor {
    /// Create for a limestone formation with saline water.
    pub fn limestone_saline() -> Self {
        Self {
            sigma_matrix: 12.0,
            sigma_water: 80.0, // high salinity (~200k ppm NaCl)
            sigma_hc: 20.0,
        }
    }

    /// Create for a sandstone formation.
    pub fn sandstone_saline() -> Self {
        Self {
            sigma_matrix: 8.0,
            sigma_water: 80.0,
            sigma_hc: 20.0,
        }
    }

    /// Create with custom Sigma values.
    pub fn new(sigma_matrix: f64, sigma_water: f64, sigma_hc: f64) -> Self {
        Self {
            sigma_matrix,
            sigma_water,
            sigma_hc,
        }
    }

    /// Compute forward model: formation Sigma given porosity and Sw.
    pub fn forward_model(&self, phi: f64, sw: f64) -> f64 {
        self.sigma_matrix * (1.0 - phi)
            + phi * (sw * self.sigma_water + (1.0 - sw) * self.sigma_hc)
    }

    /// Invert for water saturation given measured Sigma and porosity.
    ///
    /// ```text
    /// Sw = (Sigma - Sigma_ma*(1-phi) - phi*Sigma_hc) / (phi*(Sigma_w - Sigma_hc))
    /// ```
    pub fn water_saturation(&self, sigma_measured: f64, phi: f64) -> f64 {
        if phi <= 0.0 {
            return 0.0;
        }
        let denom = phi * (self.sigma_water - self.sigma_hc);
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        let sw =
            (sigma_measured - self.sigma_matrix * (1.0 - phi) - phi * self.sigma_hc) / denom;
        sw.clamp(0.0, 1.0)
    }

    /// Estimate formation water salinity from Sigma_water.
    ///
    /// Empirical: Sigma_w ≈ 22 + 0.000348 * C_ppm (for NaCl).
    pub fn sigma_water_from_salinity(salinity_ppm: f64) -> f64 {
        22.0 + 0.000_348 * salinity_ppm
    }

    /// Inverse: estimate salinity from Sigma_water.
    pub fn salinity_from_sigma_water(sigma_w: f64) -> f64 {
        ((sigma_w - 22.0) / 0.000_348).max(0.0)
    }

    /// Process a depth log of Sigma readings.
    pub fn process_log(&self, sigma_log: &[f64], phi_log: &[f64]) -> Vec<f64> {
        sigma_log
            .iter()
            .zip(phi_log.iter())
            .map(|(s, p)| self.water_saturation(*s, *p))
            .collect()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Crossplot analyzer
// ─────────────────────────────────────────────────────────────────────

/// Lithology identification point in a crossplot.
#[derive(Debug, Clone)]
pub struct CrossplotPoint {
    /// Neutron porosity (fraction).
    pub phi_neutron: f64,
    /// Density porosity or sonic DT (depending on crossplot type).
    pub secondary_value: f64,
}

/// Crossplot type for multi-log analysis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrossplotType {
    /// Neutron vs density porosity.
    NeutronDensity,
    /// Neutron vs sonic transit time.
    NeutronSonic,
}

/// Multi-log crossplot analyzer for lithology identification.
///
/// Uses neutron-density or neutron-sonic crossplots to identify
/// lithology and detect gas effects.
#[derive(Debug, Clone)]
pub struct CrossplotAnalyzer {
    /// Type of crossplot.
    pub crossplot_type: CrossplotType,
    /// Limestone endpoint.
    pub limestone_point: CrossplotPoint,
    /// Sandstone endpoint.
    pub sandstone_point: CrossplotPoint,
    /// Dolomite endpoint.
    pub dolomite_point: CrossplotPoint,
}

impl CrossplotAnalyzer {
    /// Neutron-density crossplot with standard matrix endpoints.
    ///
    /// Endpoints at zero porosity:
    /// - Limestone: phi_N = 0.0, rho_b = 2.71 g/cc
    /// - Sandstone: phi_N = −0.02, rho_b = 2.65 g/cc
    /// - Dolomite: phi_N = 0.01, rho_b = 2.87 g/cc
    pub fn neutron_density() -> Self {
        Self {
            crossplot_type: CrossplotType::NeutronDensity,
            limestone_point: CrossplotPoint {
                phi_neutron: 0.0,
                secondary_value: 2.71,
            },
            sandstone_point: CrossplotPoint {
                phi_neutron: -0.02,
                secondary_value: 2.65,
            },
            dolomite_point: CrossplotPoint {
                phi_neutron: 0.01,
                secondary_value: 2.87,
            },
        }
    }

    /// Neutron-sonic crossplot with standard matrix endpoints.
    ///
    /// Endpoints at zero porosity:
    /// - Limestone: phi_N = 0.0, DT = 47.5 us/ft
    /// - Sandstone: phi_N = −0.02, DT = 55.5 us/ft
    /// - Dolomite: phi_N = 0.01, DT = 43.5 us/ft
    pub fn neutron_sonic() -> Self {
        Self {
            crossplot_type: CrossplotType::NeutronSonic,
            limestone_point: CrossplotPoint {
                phi_neutron: 0.0,
                secondary_value: 47.5,
            },
            sandstone_point: CrossplotPoint {
                phi_neutron: -0.02,
                secondary_value: 55.5,
            },
            dolomite_point: CrossplotPoint {
                phi_neutron: 0.01,
                secondary_value: 43.5,
            },
        }
    }

    /// Identify nearest lithology for a data point using Euclidean distance.
    ///
    /// Returns the nearest `MatrixType` and the normalised distance (0 = exact match).
    pub fn identify_lithology(
        &self,
        phi_neutron: f64,
        secondary_value: f64,
    ) -> (MatrixType, f64) {
        let dist = |pt: &CrossplotPoint| -> f64 {
            let dn = phi_neutron - pt.phi_neutron;
            let ds = secondary_value - pt.secondary_value;
            // Normalise secondary axis: density range ~0.3, sonic range ~15
            let scale = match self.crossplot_type {
                CrossplotType::NeutronDensity => 10.0, // ~0.3 range → comparable to phi
                CrossplotType::NeutronSonic => 0.03,   // ~15 range → comparable to phi
            };
            (dn * dn + (ds * scale) * (ds * scale)).sqrt()
        };

        let d_ls = dist(&self.limestone_point);
        let d_ss = dist(&self.sandstone_point);
        let d_do = dist(&self.dolomite_point);

        if d_ls <= d_ss && d_ls <= d_do {
            (MatrixType::Limestone, d_ls)
        } else if d_ss <= d_do {
            (MatrixType::Sandstone, d_ss)
        } else {
            (MatrixType::Dolomite, d_do)
        }
    }

    /// Process a depth log, classifying each level.
    pub fn classify_log(
        &self,
        phi_neutron: &[f64],
        secondary: &[f64],
    ) -> Vec<(MatrixType, f64)> {
        phi_neutron
            .iter()
            .zip(secondary.iter())
            .map(|(pn, sv)| self.identify_lithology(*pn, *sv))
            .collect()
    }

    /// Compute the apparent porosity on the selected matrix line
    /// using neutron-density interpolation.
    ///
    /// The fluid point is assumed at phi_N=phi_D=1.0 (water-filled).
    /// Returns the estimated true porosity.
    pub fn apparent_porosity_on_matrix(
        &self,
        phi_neutron: f64,
        secondary_value: f64,
        matrix: MatrixType,
    ) -> f64 {
        let pt = match matrix {
            MatrixType::Limestone => &self.limestone_point,
            MatrixType::Sandstone => &self.sandstone_point,
            MatrixType::Dolomite => &self.dolomite_point,
        };
        // Simple linear interpolation between matrix point (phi=0)
        // and fluid point (phi=1). Use neutron axis as primary.
        let phi_from_neutron = phi_neutron - pt.phi_neutron;
        let phi_from_secondary = match self.crossplot_type {
            CrossplotType::NeutronDensity => {
                // rho_b = rho_ma*(1-phi) + rho_f*phi => phi = (rho_ma - rho_b)/(rho_ma - rho_f)
                let rho_f = 1.0; // water
                let rho_ma = pt.secondary_value;
                if (rho_ma - rho_f).abs() < 1e-12 {
                    0.0
                } else {
                    (rho_ma - secondary_value) / (rho_ma - rho_f)
                }
            }
            CrossplotType::NeutronSonic => {
                // DT = DT_ma*(1-phi) + DT_f*phi => phi = (DT - DT_ma)/(DT_f - DT_ma)
                let dt_f = 189.0; // water (us/ft)
                let dt_ma = pt.secondary_value;
                if (dt_f - dt_ma).abs() < 1e-12 {
                    0.0
                } else {
                    (secondary_value - dt_ma) / (dt_f - dt_ma)
                }
            }
        };
        // Average of the two estimates
        let phi = (phi_from_neutron + phi_from_secondary) / 2.0;
        phi.clamp(0.0, 1.0)
    }
}

// ─────────────────────────────────────────────────────────────────────
// Hydrogen index utilities
// ─────────────────────────────────────────────────────────────────────

/// Hydrogen Index: ratio of hydrogen density to that of fresh water.
///
/// Water HI = 1.0 by definition. Gas has HI ≈ 0.01–0.40 depending on
/// pressure and temperature. Oil HI ≈ 0.8–1.0.
pub fn hydrogen_index_gas(pressure_mpa: f64, temperature_c: f64) -> f64 {
    // Ideal gas approximation for methane (CH4):
    // rho_gas ≈ P*M/(R*T) where M=16 g/mol, R=8.314 J/(mol·K)
    // HI = (n_H_per_mol * rho_gas) / (n_H_water * rho_water)
    // CH4 has 4 H per 16 g; water has 2 H per 18 g
    // HI ≈ (4/16 * rho_gas) / (2/18 * 1000)
    let t_k = temperature_c + 273.15;
    if t_k <= 0.0 {
        return 0.0;
    }
    let rho_gas = pressure_mpa * 1e6 * 16.04e-3 / (8.314 * t_k); // kg/m³
    let h_per_kg_gas = 4.0 / 16.04e-3; // mol_H per kg CH4 * doesn't matter, ratio
    let h_per_kg_water = 2.0 / 18.015e-3;
    let rho_water = 1000.0;
    let hi = (h_per_kg_gas * rho_gas) / (h_per_kg_water * rho_water);
    hi.clamp(0.0, 2.0)
}

/// Hydrogen Index of oil (simplified, API gravity dependent).
///
/// Heavy oils (low API) have slightly higher HI due to higher density
/// despite lower H/C ratio. The net effect is dominated by density:
///
/// ```text
/// rho_oil = 141.5 / (131.5 + API)   (g/cc, from API definition)
/// HI_oil ≈ rho_oil * (H_weight_fraction / 0.1119)
/// ```
///
/// Typical range: 0.8–1.1.
pub fn hydrogen_index_oil(api_gravity: f64) -> f64 {
    // Specific gravity from API definition: SG = 141.5 / (131.5 + API)
    let sg = 141.5 / (131.5 + api_gravity);
    // H weight fraction decreases with heavier oils (~0.14 for light, ~0.11 for heavy)
    // Empirical fit: H_wt ≈ 0.1175 + 0.000_45 * API
    let h_wt = 0.1175 + 0.000_45 * api_gravity;
    // HI = (rho_oil * H_wt) / (rho_water * H_wt_water)
    // H_wt_water = 2/18.015 = 0.1119
    let hi = (sg * h_wt) / 0.1119;
    hi.clamp(0.5, 1.5)
}

/// Count-rate model: expected counts as a function of porosity.
///
/// ```text
/// N(phi) = N_0 * exp(-mu * phi)
/// ```
///
/// where `mu` is an empirical attenuation coefficient and `N_0` is
/// count rate in zero-porosity matrix.
pub fn expected_count_rate(n_zero: f64, mu: f64, porosity: f64) -> f64 {
    n_zero * (-mu * porosity).exp()
}

/// Near/far ratio from expected count model.
pub fn expected_ratio(
    n0_near: f64,
    mu_near: f64,
    n0_far: f64,
    mu_far: f64,
    porosity: f64,
) -> f64 {
    let far = expected_count_rate(n0_far, mu_far, porosity);
    if far <= 0.0 {
        return 0.0;
    }
    expected_count_rate(n0_near, mu_near, porosity) / far
}

/// Sigma (capture cross-section) typical values.
pub fn sigma_typical(matrix: MatrixType) -> f64 {
    match matrix {
        MatrixType::Limestone => 12.0,
        MatrixType::Sandstone => 8.0,
        MatrixType::Dolomite => 9.0,
    }
}

/// Sigma of fresh water (~22 c.u.).
pub const SIGMA_FRESH_WATER: f64 = 22.0;

/// Sigma of oil (~20 c.u.).
pub const SIGMA_OIL: f64 = 20.0;

/// Sigma of gas (~8 c.u.).
pub const SIGMA_GAS: f64 = 8.0;

// ─────────────────────────────────────────────────────────────────────
// Statistical helpers for count data
// ─────────────────────────────────────────────────────────────────────

/// Poisson counting statistics: standard deviation = sqrt(N).
pub fn count_uncertainty(counts: f64) -> f64 {
    if counts <= 0.0 {
        return 0.0;
    }
    counts.sqrt()
}

/// Relative statistical precision of a ratio measurement R = N_near / N_far.
///
/// ```text
/// sigma_R / R = sqrt(1/N_near + 1/N_far)
/// ```
pub fn ratio_relative_uncertainty(near_counts: f64, far_counts: f64) -> f64 {
    if near_counts <= 0.0 || far_counts <= 0.0 {
        return 1.0;
    }
    (1.0 / near_counts + 1.0 / far_counts).sqrt()
}

/// Dead-time correction for count rate.
///
/// ```text
/// N_true = N_measured / (1 - N_measured * tau)
/// ```
///
/// where `tau` is detector dead time in seconds.
pub fn dead_time_correction(measured_cps: f64, dead_time_s: f64) -> f64 {
    let denom = 1.0 - measured_cps * dead_time_s;
    if denom <= 0.0 {
        return measured_cps * 10.0; // saturated
    }
    measured_cps / denom
}

// ─────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ---------- NeutronPorosityConfig ----------

    #[test]
    fn test_config_default() {
        let cfg = NeutronPorosityConfig::default();
        assert_eq!(cfg.tool_type, ToolType::Compensated);
        assert_eq!(cfg.source_type, SourceType::AmBe);
        assert!(cfg.near_detector_spacing_m < cfg.far_detector_spacing_m);
        assert!(cfg.near_detector_efficiency > 0.0 && cfg.near_detector_efficiency <= 1.0);
    }

    #[test]
    fn test_config_sidewall() {
        let cfg = NeutronPorosityConfig {
            tool_type: ToolType::Sidewall,
            source_type: SourceType::DtGenerator,
            ..NeutronPorosityConfig::default()
        };
        assert_eq!(cfg.tool_type, ToolType::Sidewall);
        assert_eq!(cfg.source_type, SourceType::DtGenerator);
    }

    // ---------- PorosityCalculator ----------

    #[test]
    fn test_porosity_from_ratio_limestone() {
        let calc = PorosityCalculator::limestone();
        // ratio=1 → ln(1)=0 → phi = a = -0.02, clamped to 0
        let phi = calc.porosity_from_ratio(1.0);
        assert!(phi >= 0.0);
        assert!(phi < 0.01);
    }

    #[test]
    fn test_porosity_increases_with_ratio() {
        let calc = PorosityCalculator::limestone();
        let phi1 = calc.porosity_from_ratio(1.5);
        let phi2 = calc.porosity_from_ratio(3.0);
        assert!(phi2 > phi1, "Porosity should increase with ratio");
    }

    #[test]
    fn test_porosity_from_counts() {
        let calc = PorosityCalculator::limestone();
        let phi = calc.porosity_from_counts(3000.0, 1200.0);
        let phi_direct = calc.porosity_from_ratio(3000.0 / 1200.0);
        assert!((phi - phi_direct).abs() < 1e-12);
    }

    #[test]
    fn test_porosity_clamped() {
        let calc = PorosityCalculator::limestone();
        let phi_low = calc.porosity_from_ratio(0.1);
        assert!(phi_low >= calc.phi_min);
        let phi_high = calc.porosity_from_ratio(100.0);
        assert!(phi_high <= calc.phi_max);
    }

    #[test]
    fn test_porosity_negative_ratio() {
        let calc = PorosityCalculator::limestone();
        let phi = calc.porosity_from_ratio(-1.0);
        assert_eq!(phi, calc.phi_min);
    }

    #[test]
    fn test_porosity_zero_far_counts() {
        let calc = PorosityCalculator::limestone();
        let phi = calc.porosity_from_counts(1000.0, 0.0);
        assert_eq!(phi, calc.phi_max);
    }

    #[test]
    fn test_ratio_from_porosity_roundtrip() {
        let calc = PorosityCalculator::limestone();
        let phi_original = 0.15;
        let ratio = calc.ratio_from_porosity(phi_original);
        let phi_back = calc.porosity_from_ratio(ratio);
        assert!((phi_back - phi_original).abs() < 1e-6);
    }

    #[test]
    fn test_process_log() {
        let calc = PorosityCalculator::limestone();
        let ratios = vec![1.5, 2.0, 2.5, 3.0];
        let phis = calc.process_log(&ratios);
        assert_eq!(phis.len(), 4);
        for i in 0..3 {
            assert!(phis[i + 1] > phis[i], "Porosity should increase with ratio");
        }
    }

    #[test]
    fn test_sandstone_dolomite_calibrations() {
        let ss = PorosityCalculator::sandstone();
        let dol = PorosityCalculator::dolomite();
        let ratio = 2.5;
        let phi_ss = ss.porosity_from_ratio(ratio);
        let phi_dol = dol.porosity_from_ratio(ratio);
        // Both should yield reasonable porosity
        assert!(phi_ss > 0.0 && phi_ss < 0.60);
        assert!(phi_dol > 0.0 && phi_dol < 0.60);
    }

    // ---------- LithologyCorrector ----------

    #[test]
    fn test_lithology_limestone_no_change() {
        let lc = LithologyCorrector::new();
        assert_eq!(lc.offset(MatrixType::Limestone), 0.0);
        let phi = lc.correct(0.20, MatrixType::Limestone);
        assert!((phi - 0.20).abs() < 1e-12);
    }

    #[test]
    fn test_lithology_sandstone_negative_offset() {
        let lc = LithologyCorrector::new();
        let phi = lc.correct(0.20, MatrixType::Sandstone);
        assert!(phi < 0.20, "Sandstone correction should reduce porosity");
        assert!((phi - 0.18).abs() < 1e-6);
    }

    #[test]
    fn test_lithology_dolomite_positive_offset() {
        let lc = LithologyCorrector::new();
        let phi = lc.correct(0.20, MatrixType::Dolomite);
        assert!(phi > 0.20, "Dolomite correction should increase porosity");
        assert!((phi - 0.21).abs() < 1e-6);
    }

    #[test]
    fn test_lithology_no_negative_porosity() {
        let lc = LithologyCorrector::new();
        let phi = lc.correct(0.01, MatrixType::Sandstone);
        assert!(phi >= 0.0);
    }

    #[test]
    fn test_lithology_log_processing() {
        let lc = LithologyCorrector::new();
        let log = vec![0.10, 0.20, 0.30];
        let corrected = lc.correct_log(&log, MatrixType::Sandstone);
        assert_eq!(corrected.len(), 3);
        for (orig, corr) in log.iter().zip(corrected.iter()) {
            assert!(*corr < *orig);
        }
    }

    // ---------- SalinityCorrector ----------

    #[test]
    fn test_salinity_zero_gives_no_change() {
        let sc = SalinityCorrector::thermal();
        let phi_corr = sc.correct(0.20, 0.0);
        assert!((phi_corr - 0.20).abs() < 1e-12);
    }

    #[test]
    fn test_salinity_increases_porosity() {
        let sc = SalinityCorrector::thermal();
        let phi_corr = sc.correct(0.20, 200_000.0); // 200k ppm
        assert!(phi_corr > 0.20, "Salinity should increase apparent porosity");
    }

    #[test]
    fn test_epithermal_less_sensitive() {
        let thermal = SalinityCorrector::thermal();
        let epithermal = SalinityCorrector::epithermal();
        let sal = 200_000.0;
        let corr_th = thermal.correct(0.20, sal);
        let corr_ep = epithermal.correct(0.20, sal);
        assert!(
            (corr_th - 0.20).abs() > (corr_ep - 0.20).abs(),
            "Epithermal should have smaller salinity effect"
        );
    }

    // ---------- GasDetector ----------

    #[test]
    fn test_gas_detection_positive() {
        let gd = GasDetector::default();
        let sep = gd.separation(0.25, 0.10);
        assert_eq!(sep, 0.15);
        assert!(gd.is_gas(sep));
    }

    #[test]
    fn test_gas_detection_negative() {
        let gd = GasDetector::default();
        let sep = gd.separation(0.10, 0.25);
        assert_eq!(sep, -0.15);
        assert!(!gd.is_gas(sep));
    }

    #[test]
    fn test_gas_below_threshold() {
        let gd = GasDetector::default();
        let sep = gd.separation(0.20, 0.18);
        assert!((sep - 0.02).abs() < 1e-10);
        assert!(!gd.is_gas(sep), "Small separation should not flag gas");
    }

    #[test]
    fn test_gas_log() {
        let gd = GasDetector::default();
        let pd = vec![0.25, 0.20, 0.15];
        let pn = vec![0.10, 0.19, 0.15];
        let flags = gd.detect_log(&pd, &pn);
        assert_eq!(flags, vec![true, false, false]);
    }

    #[test]
    fn test_gas_saturation_estimate() {
        let gd = GasDetector::default();
        let sg = gd.estimate_gas_saturation(0.25, 0.10, 0.0);
        assert!(sg > 0.0 && sg <= 1.0);
        // With HI_gas=0, Sg = (0.25-0.10)/0.25 = 0.60
        assert!((sg - 0.60).abs() < 1e-6);
    }

    #[test]
    fn test_gas_saturation_no_gas() {
        let gd = GasDetector::default();
        let sg = gd.estimate_gas_saturation(0.20, 0.25, 0.0);
        assert_eq!(sg, 0.0);
    }

    // ---------- ShaleVolumeEstimator ----------

    #[test]
    fn test_vshale_clean() {
        let sv = ShaleVolumeEstimator::sand_shale();
        let vsh = sv.v_shale(sv.phi_n_clean);
        assert!((vsh - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_vshale_shale() {
        let sv = ShaleVolumeEstimator::sand_shale();
        let vsh = sv.v_shale(sv.phi_n_shale);
        assert!((vsh - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_vshale_midpoint() {
        let sv = ShaleVolumeEstimator::sand_shale();
        let mid = (sv.phi_n_clean + sv.phi_n_shale) / 2.0;
        let vsh = sv.v_shale(mid);
        assert!((vsh - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_vshale_clamped() {
        let sv = ShaleVolumeEstimator::sand_shale();
        let vsh = sv.v_shale(0.60); // above shale endpoint
        assert!(vsh <= 1.0);
        let vsh2 = sv.v_shale(-0.10); // below clean endpoint
        assert!(vsh2 >= 0.0);
    }

    #[test]
    fn test_vshale_clavier() {
        let sv = ShaleVolumeEstimator::sand_shale();
        let phi_mid = (sv.phi_n_clean + sv.phi_n_shale) / 2.0;
        let vsh_lin = sv.v_shale(phi_mid);
        let vsh_clav = sv.v_shale_clavier(phi_mid);
        // Clavier gives lower V_sh than linear for moderate values
        assert!(vsh_clav < vsh_lin);
    }

    #[test]
    fn test_vshale_steiber() {
        let sv = ShaleVolumeEstimator::sand_shale();
        let phi_mid = (sv.phi_n_clean + sv.phi_n_shale) / 2.0;
        let vsh_lin = sv.v_shale(phi_mid);
        let vsh_steiber = sv.v_shale_steiber(phi_mid);
        // Steiber also gives lower V_sh than linear for moderate values
        assert!(vsh_steiber < vsh_lin);
    }

    // ---------- EnvironmentalCorrector ----------

    #[test]
    fn test_env_no_deviation() {
        let ec = EnvironmentalCorrector::new();
        let corr = ec.total_correction(
            ec.reference_hole_diameter_m,
            ec.reference_mud_weight_kg_m3,
            ec.reference_temperature_c,
            ec.reference_pressure_mpa,
        );
        assert!(corr.abs() < 1e-12);
    }

    #[test]
    fn test_env_larger_hole() {
        let ec = EnvironmentalCorrector::new();
        let corr = ec.hole_size_correction(0.30); // 12 inch hole
        assert!(corr > 0.0, "Larger hole should increase correction");
    }

    #[test]
    fn test_env_full_correction() {
        let ec = EnvironmentalCorrector::new();
        let phi = ec.correct(0.20, 0.30, 1200.0, 100.0, 50.0);
        // Should differ from 0.20 due to corrections
        assert!((phi - 0.20).abs() > 1e-6);
        assert!(phi >= 0.0);
    }

    // ---------- EpithermalDetector ----------

    #[test]
    fn test_epithermal_porosity() {
        let det = EpithermalDetector::new();
        let phi = det.porosity_from_ratio(2.5);
        assert!(phi > 0.0 && phi < 0.60);
    }

    #[test]
    fn test_thermal_epithermal_ratio() {
        let det = EpithermalDetector::new();
        let ratio = det.thermal_epithermal_ratio(5000.0, 2000.0);
        assert!((ratio - 2.5).abs() < 1e-12);
    }

    #[test]
    fn test_salinity_estimation() {
        let det = EpithermalDetector::new();
        let freshwater = 2.0;
        let saline_ratio = 3.0; // higher ratio in saline mud
        let sal = det.estimate_salinity_ppm(saline_ratio, freshwater);
        assert!(sal > 0.0);
        assert!((sal - 50_000.0).abs() < 1.0);
    }

    // ---------- SigmaLogProcessor ----------

    #[test]
    fn test_sigma_forward_model() {
        let slp = SigmaLogProcessor::limestone_saline();
        // Zero porosity → pure matrix
        let sigma = slp.forward_model(0.0, 1.0);
        assert!((sigma - slp.sigma_matrix).abs() < 1e-12);
    }

    #[test]
    fn test_sigma_water_saturation_full() {
        let slp = SigmaLogProcessor::limestone_saline();
        let phi = 0.20;
        let sigma = slp.forward_model(phi, 1.0); // 100% water
        let sw = slp.water_saturation(sigma, phi);
        assert!((sw - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_sigma_water_saturation_zero() {
        let slp = SigmaLogProcessor::limestone_saline();
        let phi = 0.20;
        let sigma = slp.forward_model(phi, 0.0); // 100% hydrocarbon
        let sw = slp.water_saturation(sigma, phi);
        assert!((sw - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_sigma_water_saturation_partial() {
        let slp = SigmaLogProcessor::limestone_saline();
        let phi = 0.20;
        let sw_true = 0.65;
        let sigma = slp.forward_model(phi, sw_true);
        let sw_calc = slp.water_saturation(sigma, phi);
        assert!((sw_calc - sw_true).abs() < 1e-6);
    }

    #[test]
    fn test_sigma_water_from_salinity() {
        let sigma_w = SigmaLogProcessor::sigma_water_from_salinity(200_000.0);
        // 22 + 0.000348 * 200000 = 22 + 69.6 = 91.6
        assert!((sigma_w - 91.6).abs() < 0.1);
    }

    #[test]
    fn test_sigma_salinity_roundtrip() {
        let sal = 150_000.0;
        let sigma_w = SigmaLogProcessor::sigma_water_from_salinity(sal);
        let sal_back = SigmaLogProcessor::salinity_from_sigma_water(sigma_w);
        assert!((sal_back - sal).abs() < 1.0);
    }

    // ---------- CrossplotAnalyzer ----------

    #[test]
    fn test_crossplot_limestone_identification() {
        let xp = CrossplotAnalyzer::neutron_density();
        let (litho, dist) = xp.identify_lithology(0.0, 2.71);
        assert_eq!(litho, MatrixType::Limestone);
        assert!(dist < 1e-6);
    }

    #[test]
    fn test_crossplot_sandstone_identification() {
        let xp = CrossplotAnalyzer::neutron_density();
        let (litho, _) = xp.identify_lithology(-0.02, 2.65);
        assert_eq!(litho, MatrixType::Sandstone);
    }

    #[test]
    fn test_crossplot_dolomite_identification() {
        let xp = CrossplotAnalyzer::neutron_density();
        let (litho, _) = xp.identify_lithology(0.01, 2.87);
        assert_eq!(litho, MatrixType::Dolomite);
    }

    #[test]
    fn test_crossplot_sonic() {
        let xp = CrossplotAnalyzer::neutron_sonic();
        let (litho, _) = xp.identify_lithology(0.0, 47.5);
        assert_eq!(litho, MatrixType::Limestone);
    }

    #[test]
    fn test_crossplot_apparent_porosity() {
        let xp = CrossplotAnalyzer::neutron_density();
        // At the matrix point: porosity should be ~0
        let phi = xp.apparent_porosity_on_matrix(0.0, 2.71, MatrixType::Limestone);
        assert!(phi.abs() < 0.01);

        // A porous limestone (phi_N=0.20, rho=2.37 → phi_D ≈ 0.20)
        let phi2 = xp.apparent_porosity_on_matrix(0.20, 2.37, MatrixType::Limestone);
        assert!((phi2 - 0.20).abs() < 0.02);
    }

    // ---------- Hydrogen index utilities ----------

    #[test]
    fn test_hydrogen_index_gas() {
        // At high P and low T, gas HI is higher
        let hi_low_p = hydrogen_index_gas(5.0, 80.0);
        let hi_high_p = hydrogen_index_gas(50.0, 80.0);
        assert!(hi_high_p > hi_low_p, "Higher pressure → higher HI");
        assert!(hi_low_p > 0.0);
        assert!(hi_high_p < 2.0);
    }

    #[test]
    fn test_hydrogen_index_oil() {
        // Heavy oil (low API, higher density) has slightly higher HI
        let hi_light = hydrogen_index_oil(40.0);
        let hi_heavy = hydrogen_index_oil(20.0);
        assert!(hi_heavy > hi_light, "Heavy oil should have higher HI due to density");
        assert!(hi_light > 0.8 && hi_light < 1.2);
        assert!(hi_heavy > 0.8 && hi_heavy < 1.2);
    }

    // ---------- Count-rate model ----------

    #[test]
    fn test_expected_count_rate() {
        let n0 = 10000.0;
        let mu = 5.0;
        let n_at_zero = expected_count_rate(n0, mu, 0.0);
        assert!((n_at_zero - n0).abs() < 1e-6);

        let n_at_20 = expected_count_rate(n0, mu, 0.20);
        assert!(n_at_20 < n0, "Counts should decrease with porosity");
    }

    #[test]
    fn test_expected_ratio() {
        // Near detector has lower mu (less attenuation)
        let ratio = expected_ratio(10000.0, 3.0, 8000.0, 5.0, 0.20);
        assert!(ratio > 1.0, "Near/far ratio should be > 1 when mu_far > mu_near");
    }

    // ---------- Statistical helpers ----------

    #[test]
    fn test_count_uncertainty() {
        assert!((count_uncertainty(10000.0) - 100.0).abs() < 1e-6);
        assert_eq!(count_uncertainty(0.0), 0.0);
    }

    #[test]
    fn test_ratio_uncertainty() {
        let u = ratio_relative_uncertainty(10000.0, 5000.0);
        // sqrt(1/10000 + 1/5000) = sqrt(0.0001 + 0.0002) = sqrt(0.0003) ≈ 0.01732
        assert!((u - 0.01732).abs() < 0.001);
    }

    #[test]
    fn test_dead_time_correction() {
        let true_rate = dead_time_correction(9000.0, 5e-6);
        // N_true = 9000 / (1 - 9000*5e-6) = 9000/0.955 ≈ 9424
        assert!((true_rate - 9424.08).abs() < 1.0);
    }

    #[test]
    fn test_dead_time_saturated() {
        // When N*tau >= 1, detector is saturated
        let rate = dead_time_correction(250_000.0, 5e-6);
        assert!(rate > 250_000.0);
    }

    // ---------- Sigma constants ----------

    #[test]
    fn test_sigma_typical_values() {
        assert_eq!(sigma_typical(MatrixType::Limestone), 12.0);
        assert_eq!(sigma_typical(MatrixType::Sandstone), 8.0);
        assert_eq!(sigma_typical(MatrixType::Dolomite), 9.0);
        assert!((SIGMA_FRESH_WATER - 22.0).abs() < 1e-12);
        assert!((SIGMA_OIL - 20.0).abs() < 1e-12);
        assert!((SIGMA_GAS - 8.0).abs() < 1e-12);
    }
}
