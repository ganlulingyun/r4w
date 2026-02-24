//! # Glow Discharge Optical Emission Spectrometry (GD-OES) Processor
//!
//! Implements bulk and depth-profile elemental analysis using Glow Discharge
//! Optical Emission Spectrometry (GD-OES), a powerful technique for rapid
//! characterisation of layered materials and coatings.
//!
//! ## Science Background
//!
//! In GD-OES a sample (cathode) is bombarded by Ar+ ions in a low-pressure
//! (typically 200–1000 Pa) plasma sustained between the sample and an anode.
//! Sputtered atoms enter the plasma, are excited by electron collisions and
//! emit characteristic optical radiation that is dispersed and detected as
//! intensities I(λ,t).
//!
//! **Grimm-type source** geometry is the industry standard.  The hollow anode
//! encircles the discharge and defines a flat-crater sputtering geometry.
//!
//! ### Sputtering rate
//! The instantaneous sputter rate q (µg s⁻¹) is proportional to the ion
//! current density at the cathode surface and the sputter yield S:
//! ```text
//! q = S · j · M / (N_A · e)
//! ```
//! where j is current density (A m⁻²), M molar mass (g mol⁻¹), N_A
//! Avogadro's number, e elementary charge.
//!
//! ### Depth conversion
//! Cumulative sputtered depth z (µm):
//! ```text
//! z(t) = ∫₀ᵗ q(τ)/(ρ·A) dτ
//! ```
//! where ρ is density (g cm⁻³) and A is crater area (cm²).
//!
//! ### Calibration
//! Concentration Cₑ of element e is related to emission intensity Iₑ via:
//! ```text
//! Cₑ = f(Iₑ, I_matrix) calibration curve
//! ```
//! with optional matrix correction factors Rᵢⱼ.
//!
//! ### Interface detection
//! Layer boundaries are detected as peaks of |dI/dz| or inflection points
//! of a sigmoid fitted to the step-response profile.
//!
//! ## Key Components
//!
//! - **GrimSource**: Grimm-type discharge source model (voltage, current, pressure)
//! - **EmissionLineDb**: Database of analytical emission lines for 15 elements
//! - **SputterRateCalculator**: Mass-based and current-based sputter rate
//! - **DepthProfileBuilder**: Convert intensity–time to concentration–depth
//! - **CalibrationCurve**: Linear / quadratic intensity–concentration curves
//! - **MatrixCorrector**: Inter-element matrix correction
//! - **InterfaceDetector**: Layer boundary detection
//! - **CraterShapeCorrector**: Non-uniform sputtering correction
//! - **CoatingPreset**: Pre-built presets for common coating systems
//! - **GdoesProcessor**: Top-level workflow orchestrator

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Avogadro's number (mol⁻¹).
pub const AVOGADRO: f64 = 6.022_140_76e23;

/// Elementary charge (C).
pub const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Discharge source mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SourceMode {
    /// DC mode – for electrically conductive samples.
    Dc,
    /// RF mode – for non-conductive samples (oxides, polymers, …).
    Rf,
}

/// Calibration curve type for intensity → concentration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibCurveType {
    /// C = a·I  (single-point origin calibration).
    Linear,
    /// C = a·I + b·I²  (quadratic).
    Quadratic,
    /// C = a·log(I) + b  (logarithmic, for wide concentration ranges).
    Logarithmic,
}

/// Method for interface/layer-boundary detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterfaceMethod {
    /// Peak in |dI/dz| first derivative.
    FirstDerivativePeak,
    /// Inflection point of sigmoid fit to intensity step.
    SigmoidInflection,
    /// Threshold crossing at a fixed fraction of the step height.
    ThresholdCrossing(f64),
}

/// Common coating preset identifiers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoatingPreset {
    /// Hot-dip galvanised steel: Zn coating on Fe substrate.
    GalvanisedSteel,
    /// Physical vapour deposited TiN hard coating on steel.
    TiNHardCoating,
    /// Thermally grown Al₂O₃ on aluminium alloy.
    AluminaOnAluminium,
    /// Electrodeposited Cr₂O₃ passive film on stainless steel.
    ChromiaOnSteel,
    /// Duplex coating: Zn–Fe intermetallic / Fe substrate.
    ZincIronDuplex,
}

// ---------------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------------

/// A single spectral emission line entry.
#[derive(Debug, Clone)]
pub struct EmissionLine {
    /// Element symbol (e.g. "Fe").
    pub element: &'static str,
    /// Atomic number.
    pub atomic_number: u32,
    /// Wavelength (nm).
    pub wavelength_nm: f64,
    /// Excitation energy (eV).
    pub excitation_energy_ev: f64,
    /// Relative sensitivity factor (arbitrary units, normalised to Fe 371.99 nm = 1.0).
    pub rsf: f64,
    /// Notes (e.g. interference warnings).
    pub notes: &'static str,
}

/// Grimm-type glow discharge source parameters.
#[derive(Debug, Clone)]
pub struct GrimSource {
    /// Discharge voltage (V).  Typical: 500–1500 V (DC), 500–2000 V (RF).
    pub voltage_v: f64,
    /// Discharge current (A).  Typical: 20–100 mA.
    pub current_a: f64,
    /// Argon pressure (Pa).  Typical: 200–1000 Pa.
    pub pressure_pa: f64,
    /// Crater diameter (mm).  Standard Grimm: 4 mm or 8 mm.
    pub crater_diameter_mm: f64,
    /// Source mode (DC / RF).
    pub mode: SourceMode,
    /// RF frequency (MHz) – only relevant for RF mode.
    pub rf_frequency_mhz: f64,
}

impl GrimSource {
    /// Create a standard DC Grimm source (4 mm crater).
    pub fn standard_dc() -> Self {
        Self {
            voltage_v: 800.0,
            current_a: 0.040,
            pressure_pa: 500.0,
            crater_diameter_mm: 4.0,
            mode: SourceMode::Dc,
            rf_frequency_mhz: 13.56,
        }
    }

    /// Create an RF Grimm source for non-conductors (4 mm crater).
    pub fn standard_rf() -> Self {
        Self {
            voltage_v: 1200.0,
            current_a: 0.030,
            pressure_pa: 300.0,
            crater_diameter_mm: 4.0,
            mode: SourceMode::Rf,
            rf_frequency_mhz: 13.56,
        }
    }

    /// Crater area (cm²).
    pub fn crater_area_cm2(&self) -> f64 {
        let r_cm = (self.crater_diameter_mm / 2.0) * 0.1;
        PI * r_cm * r_cm
    }

    /// Current density at the cathode surface (A cm⁻²).
    pub fn current_density_a_cm2(&self) -> f64 {
        self.current_a / self.crater_area_cm2()
    }

    /// Power delivered to the discharge (W).
    pub fn power_w(&self) -> f64 {
        self.voltage_v * self.current_a
    }

    /// Estimate reduced electric field E/p (V cm⁻¹ Pa⁻¹) – controls ion energy.
    pub fn reduced_field_v_cm_pa(&self) -> f64 {
        // Approximate cathode sheath thickness d_sheath ~ 0.5 cm at 500 Pa
        let d_cm = 0.5 * (500.0 / self.pressure_pa).sqrt();
        (self.voltage_v / d_cm) / self.pressure_pa
    }
}

/// Material properties for sputter rate calculation.
#[derive(Debug, Clone)]
pub struct MaterialProperties {
    /// Density (g cm⁻³).
    pub density_g_cm3: f64,
    /// Molar mass (g mol⁻¹).
    pub molar_mass_g_mol: f64,
    /// Sputter yield S (atoms per Ar+ ion) at typical source conditions.
    pub sputter_yield: f64,
}

impl MaterialProperties {
    /// Properties for pure iron.
    pub fn iron() -> Self {
        Self { density_g_cm3: 7.874, molar_mass_g_mol: 55.845, sputter_yield: 1.05 }
    }

    /// Properties for pure zinc.
    pub fn zinc() -> Self {
        Self { density_g_cm3: 7.133, molar_mass_g_mol: 65.38, sputter_yield: 2.10 }
    }

    /// Properties for pure aluminium.
    pub fn aluminium() -> Self {
        Self { density_g_cm3: 2.700, molar_mass_g_mol: 26.982, sputter_yield: 0.80 }
    }

    /// Properties for TiN (titanium nitride).
    pub fn tin_coating() -> Self {
        Self { density_g_cm3: 5.430, molar_mass_g_mol: 61.874, sputter_yield: 0.55 }
    }
}

/// Calculates sputter rates and cumulative depth from source parameters.
#[derive(Debug, Clone)]
pub struct SputterRateCalculator {
    /// Source parameters.
    pub source: GrimSource,
    /// Sample material properties.
    pub material: MaterialProperties,
}

impl SputterRateCalculator {
    /// Create a new calculator.
    pub fn new(source: GrimSource, material: MaterialProperties) -> Self {
        Self { source, material }
    }

    /// Instantaneous sputter rate (µg s⁻¹) using emission current model.
    ///
    /// q = S · j · A · M / (N_A · e)
    pub fn sputter_rate_ug_per_s(&self) -> f64 {
        let j = self.source.current_density_a_cm2(); // A cm⁻²
        let area = self.source.crater_area_cm2();    // cm²
        let s = self.material.sputter_yield;
        let m = self.material.molar_mass_g_mol;
        // Convert to µg s⁻¹
        let rate_g_per_s =
            s * j * area * m / (AVOGADRO * ELEMENTARY_CHARGE);
        rate_g_per_s * 1.0e6 // → µg s⁻¹
    }

    /// Instantaneous depth removal rate (µm s⁻¹).
    ///
    /// dz/dt = q / (ρ · A)
    pub fn depth_rate_um_per_s(&self) -> f64 {
        let q_ug = self.sputter_rate_ug_per_s(); // µg s⁻¹
        let rho = self.material.density_g_cm3;   // g cm⁻³
        let area_cm2 = self.source.crater_area_cm2();
        // q_ug [µg s⁻¹] → g s⁻¹ = q_ug * 1e-6
        // depth cm s⁻¹ = (g s⁻¹) / (g cm⁻³ · cm²)
        let depth_cm_per_s = (q_ug * 1.0e-6) / (rho * area_cm2);
        depth_cm_per_s * 1.0e4 // cm → µm
    }

    /// Compute cumulative sputtered depth profile from a time vector.
    ///
    /// Returns a vector of depths (µm) at each time step.
    /// The depth rate may vary if the material composition changes; here
    /// we integrate a constant rate (single-material assumption).
    pub fn cumulative_depth_um(&self, times_s: &[f64]) -> Vec<f64> {
        let rate = self.depth_rate_um_per_s();
        times_s.iter().map(|&t| rate * t).collect()
    }

    /// Depth from crater volume measurement (gravimetric / profilometry).
    ///
    /// mass_loss_ug: measured mass loss (µg).
    /// Returns estimated mean crater depth (µm).
    pub fn depth_from_mass_loss_um(&self, mass_loss_ug: f64) -> f64 {
        let rho = self.material.density_g_cm3; // g cm⁻³
        let area_cm2 = self.source.crater_area_cm2();
        // mass_loss_g = mass_loss_ug * 1e-6
        let depth_cm = (mass_loss_ug * 1.0e-6) / (rho * area_cm2);
        depth_cm * 1.0e4 // → µm
    }
}

// ---------------------------------------------------------------------------
// Emission line database
// ---------------------------------------------------------------------------

/// Database of key GD-OES analytical emission lines.
pub struct EmissionLineDb;

impl EmissionLineDb {
    /// Return all emission lines in the built-in database.
    pub fn all_lines() -> Vec<EmissionLine> {
        vec![
            // Iron
            EmissionLine { element: "Fe", atomic_number: 26, wavelength_nm: 371.994,
                excitation_energy_ev: 3.33, rsf: 1.00, notes: "primary Fe line" },
            EmissionLine { element: "Fe", atomic_number: 26, wavelength_nm: 259.940,
                excitation_energy_ev: 4.77, rsf: 0.62, notes: "secondary Fe line" },
            // Chromium
            EmissionLine { element: "Cr", atomic_number: 24, wavelength_nm: 425.435,
                excitation_energy_ev: 2.91, rsf: 1.42, notes: "strong Cr line" },
            EmissionLine { element: "Cr", atomic_number: 24, wavelength_nm: 357.869,
                excitation_energy_ev: 3.46, rsf: 0.95, notes: "" },
            // Nickel
            EmissionLine { element: "Ni", atomic_number: 28, wavelength_nm: 341.476,
                excitation_energy_ev: 3.63, rsf: 0.88, notes: "" },
            EmissionLine { element: "Ni", atomic_number: 28, wavelength_nm: 352.454,
                excitation_energy_ev: 3.52, rsf: 0.74, notes: "" },
            // Copper
            EmissionLine { element: "Cu", atomic_number: 29, wavelength_nm: 324.754,
                excitation_energy_ev: 3.82, rsf: 1.55, notes: "resonance line" },
            EmissionLine { element: "Cu", atomic_number: 29, wavelength_nm: 521.820,
                excitation_energy_ev: 3.82, rsf: 0.45, notes: "" },
            // Zinc
            EmissionLine { element: "Zn", atomic_number: 30, wavelength_nm: 213.857,
                excitation_energy_ev: 5.80, rsf: 2.30, notes: "primary Zn line" },
            EmissionLine { element: "Zn", atomic_number: 30, wavelength_nm: 481.053,
                excitation_energy_ev: 4.08, rsf: 0.80, notes: "" },
            // Aluminium
            EmissionLine { element: "Al", atomic_number: 13, wavelength_nm: 396.152,
                excitation_energy_ev: 3.14, rsf: 0.72, notes: "" },
            EmissionLine { element: "Al", atomic_number: 13, wavelength_nm: 309.271,
                excitation_energy_ev: 4.02, rsf: 0.50, notes: "" },
            // Silicon
            EmissionLine { element: "Si", atomic_number: 14, wavelength_nm: 288.158,
                excitation_energy_ev: 5.08, rsf: 0.34, notes: "" },
            EmissionLine { element: "Si", atomic_number: 14, wavelength_nm: 251.611,
                excitation_energy_ev: 4.95, rsf: 0.28, notes: "" },
            // Carbon
            EmissionLine { element: "C", atomic_number: 6, wavelength_nm: 156.140,
                excitation_energy_ev: 7.95, rsf: 0.18, notes: "VUV – purged optics required" },
            EmissionLine { element: "C", atomic_number: 6, wavelength_nm: 193.090,
                excitation_energy_ev: 7.68, rsf: 0.22, notes: "VUV" },
            // Nitrogen
            EmissionLine { element: "N", atomic_number: 7, wavelength_nm: 149.263,
                excitation_energy_ev: 10.33, rsf: 0.12, notes: "VUV – may overlap with N₂ band" },
            EmissionLine { element: "N", atomic_number: 7, wavelength_nm: 174.272,
                excitation_energy_ev: 9.14, rsf: 0.16, notes: "VUV" },
            // Oxygen
            EmissionLine { element: "O", atomic_number: 8, wavelength_nm: 130.217,
                excitation_energy_ev: 10.74, rsf: 0.10, notes: "VUV – requires vacuum spectrometer" },
            EmissionLine { element: "O", atomic_number: 8, wavelength_nm: 777.194,
                excitation_energy_ev: 10.74, rsf: 0.06, notes: "triplet – less sensitive" },
            // Titanium
            EmissionLine { element: "Ti", atomic_number: 22, wavelength_nm: 365.350,
                excitation_energy_ev: 3.39, rsf: 0.88, notes: "" },
            EmissionLine { element: "Ti", atomic_number: 22, wavelength_nm: 334.941,
                excitation_energy_ev: 3.70, rsf: 0.72, notes: "" },
            // Manganese
            EmissionLine { element: "Mn", atomic_number: 25, wavelength_nm: 403.076,
                excitation_energy_ev: 3.08, rsf: 1.22, notes: "" },
            EmissionLine { element: "Mn", atomic_number: 25, wavelength_nm: 257.610,
                excitation_energy_ev: 4.81, rsf: 0.65, notes: "" },
            // Molybdenum
            EmissionLine { element: "Mo", atomic_number: 42, wavelength_nm: 386.411,
                excitation_energy_ev: 3.21, rsf: 0.92, notes: "" },
            EmissionLine { element: "Mo", atomic_number: 42, wavelength_nm: 379.825,
                excitation_energy_ev: 3.26, rsf: 0.78, notes: "" },
            // Sulphur
            EmissionLine { element: "S", atomic_number: 16, wavelength_nm: 180.731,
                excitation_energy_ev: 6.86, rsf: 0.24, notes: "VUV" },
            EmissionLine { element: "S", atomic_number: 16, wavelength_nm: 166.672,
                excitation_energy_ev: 7.44, rsf: 0.19, notes: "VUV" },
            // Phosphorus
            EmissionLine { element: "P", atomic_number: 15, wavelength_nm: 178.287,
                excitation_energy_ev: 6.95, rsf: 0.20, notes: "VUV" },
            EmissionLine { element: "P", atomic_number: 15, wavelength_nm: 213.618,
                excitation_energy_ev: 5.81, rsf: 0.30, notes: "" },
        ]
    }

    /// Return lines for a specific element.
    pub fn lines_for_element(element: &str) -> Vec<EmissionLine> {
        Self::all_lines().into_iter().filter(|l| l.element == element).collect()
    }

    /// Return the primary (highest RSF) line for an element, if available.
    pub fn primary_line(element: &str) -> Option<EmissionLine> {
        let mut lines = Self::lines_for_element(element);
        lines.sort_by(|a, b| b.rsf.partial_cmp(&a.rsf).unwrap_or(std::cmp::Ordering::Equal));
        lines.into_iter().next()
    }

    /// Find lines within ±delta_nm of a target wavelength.
    pub fn find_near(wavelength_nm: f64, delta_nm: f64) -> Vec<EmissionLine> {
        Self::all_lines()
            .into_iter()
            .filter(|l| (l.wavelength_nm - wavelength_nm).abs() <= delta_nm)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Calibration curve
// ---------------------------------------------------------------------------

/// Calibration curve: maps measured intensity → concentration (wt%).
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    /// Curve type.
    pub curve_type: CalibCurveType,
    /// Coefficient a.
    pub a: f64,
    /// Coefficient b (used for Quadratic and Logarithmic).
    pub b: f64,
    /// Concentration range (wt%) for validity checking.
    pub range_wt_pct: (f64, f64),
}

impl CalibrationCurve {
    /// Construct a linear calibration: C = a·I.
    pub fn linear(a: f64) -> Self {
        Self { curve_type: CalibCurveType::Linear, a, b: 0.0, range_wt_pct: (0.0, 100.0) }
    }

    /// Construct a quadratic calibration: C = a·I + b·I².
    pub fn quadratic(a: f64, b: f64) -> Self {
        Self { curve_type: CalibCurveType::Quadratic, a, b, range_wt_pct: (0.0, 100.0) }
    }

    /// Construct a logarithmic calibration: C = a·ln(I) + b.
    pub fn logarithmic(a: f64, b: f64) -> Self {
        Self { curve_type: CalibCurveType::Logarithmic, a, b, range_wt_pct: (0.0, 100.0) }
    }

    /// Apply the calibration to convert intensity to concentration (wt%).
    ///
    /// Returns `None` if intensity is non-positive for logarithmic curve.
    pub fn intensity_to_concentration(&self, intensity: f64) -> Option<f64> {
        let c = match self.curve_type {
            CalibCurveType::Linear => self.a * intensity,
            CalibCurveType::Quadratic => self.a * intensity + self.b * intensity * intensity,
            CalibCurveType::Logarithmic => {
                if intensity <= 0.0 { return None; }
                self.a * intensity.ln() + self.b
            }
        };
        Some(c.max(0.0))
    }

    /// Inverse: concentration → expected intensity (linear only).
    pub fn concentration_to_intensity(&self, concentration: f64) -> Option<f64> {
        match self.curve_type {
            CalibCurveType::Linear => {
                if self.a.abs() < f64::EPSILON { None } else { Some(concentration / self.a) }
            }
            _ => None, // not implemented for non-linear
        }
    }

    /// Fit a linear calibration from standard pairs (intensity, conc_wt_pct).
    pub fn fit_linear(pairs: &[(f64, f64)]) -> Self {
        // Least-squares through origin: a = sum(I·C) / sum(I²)
        let sum_ic: f64 = pairs.iter().map(|(i, c)| i * c).sum();
        let sum_i2: f64 = pairs.iter().map(|(i, _)| i * i).sum();
        let a = if sum_i2 > 0.0 { sum_ic / sum_i2 } else { 0.0 };
        Self::linear(a)
    }

    /// Fit a quadratic calibration from standard pairs (intensity, conc_wt_pct).
    ///
    /// Uses ordinary least-squares with design matrix [I, I²].
    pub fn fit_quadratic(pairs: &[(f64, f64)]) -> Self {
        let n = pairs.len() as f64;
        if n < 2.0 {
            return Self::linear(0.0);
        }
        let mut s1 = 0.0_f64;  // sum(I)
        let mut s2 = 0.0_f64;  // sum(I²)
        let mut s3 = 0.0_f64;  // sum(I³)
        let mut s4 = 0.0_f64;  // sum(I⁴)
        let mut t1 = 0.0_f64;  // sum(C·I)
        let mut t2 = 0.0_f64;  // sum(C·I²)
        for &(i, c) in pairs {
            s1 += i; s2 += i*i; s3 += i*i*i; s4 += i*i*i*i;
            t1 += c*i; t2 += c*i*i;
        }
        // Normal equations: [s2 s3; s3 s4] [a; b] = [t1; t2]
        let det = s2 * s4 - s3 * s3;
        if det.abs() < 1.0e-30 {
            return Self::fit_linear(pairs);
        }
        let a = (t1 * s4 - t2 * s3) / det;
        let b = (s2 * t2 - s3 * t1) / det;
        Self::quadratic(a, b)
    }
}

// ---------------------------------------------------------------------------
// Matrix correction
// ---------------------------------------------------------------------------

/// Inter-element matrix correction factor for element i from matrix element j.
///
/// I_corrected = I_raw · product_j (1 + R_ij · C_j)
#[derive(Debug, Clone)]
pub struct MatrixCorrector {
    /// List of (element_j, correction_coefficient Rij).
    pub corrections: Vec<(String, f64)>,
}

impl MatrixCorrector {
    /// Create an empty (identity) corrector.
    pub fn identity() -> Self {
        Self { corrections: Vec::new() }
    }

    /// Add a correction term for a matrix element.
    pub fn add_correction(&mut self, matrix_element: &str, r_ij: f64) {
        self.corrections.push((matrix_element.to_string(), r_ij));
    }

    /// Apply correction: returns corrected intensity.
    ///
    /// `matrix_concentrations`: slice of (element, concentration_wt_pct) for
    /// all matrix elements that influence this line.
    pub fn apply(&self, raw_intensity: f64, matrix_concentrations: &[(&str, f64)]) -> f64 {
        let mut factor = 1.0_f64;
        for (elem_j, r_ij) in &self.corrections {
            if let Some(&(_, c_j)) = matrix_concentrations.iter().find(|(e, _)| e == elem_j) {
                factor += r_ij * c_j;
            }
        }
        raw_intensity * factor
    }
}

// ---------------------------------------------------------------------------
// Depth profile
// ---------------------------------------------------------------------------

/// A single measured intensity channel at one time step.
#[derive(Debug, Clone)]
pub struct IntensityPoint {
    /// Time since start of sputtering (s).
    pub time_s: f64,
    /// Emission intensities keyed by emission wavelength index (counts s⁻¹).
    pub intensities: Vec<f64>,
}

/// Built depth-profile result for one element channel.
#[derive(Debug, Clone)]
pub struct DepthProfileChannel {
    /// Element symbol.
    pub element: String,
    /// Wavelength (nm) of the emission line used.
    pub wavelength_nm: f64,
    /// Depth axis (µm) for each time point.
    pub depth_um: Vec<f64>,
    /// Raw emission intensities (counts s⁻¹).
    pub raw_intensity: Vec<f64>,
    /// Calibrated concentration (wt%).
    pub concentration_wt_pct: Vec<f64>,
}

/// Builds a depth profile from time-series intensities.
#[derive(Debug, Clone)]
pub struct DepthProfileBuilder {
    /// Sputtering rate calculator.
    pub sputter_calc: SputterRateCalculator,
}

impl DepthProfileBuilder {
    /// Create a new builder.
    pub fn new(source: GrimSource, material: MaterialProperties) -> Self {
        Self { sputter_calc: SputterRateCalculator::new(source, material) }
    }

    /// Convert time-series intensities to depth profile with calibration.
    ///
    /// `times_s`: measurement times (s).
    /// `intensities`: emission intensity at each time (counts s⁻¹).
    /// `element`: element symbol for labelling.
    /// `wavelength_nm`: wavelength of the emission line.
    /// `calib`: optional calibration curve (if None, raw intensities used as conc).
    pub fn build(
        &self,
        times_s: &[f64],
        intensities: &[f64],
        element: &str,
        wavelength_nm: f64,
        calib: Option<&CalibrationCurve>,
    ) -> DepthProfileChannel {
        assert_eq!(times_s.len(), intensities.len(), "times and intensities must match");
        let depth_um = self.sputter_calc.cumulative_depth_um(times_s);
        let concentration_wt_pct = intensities
            .iter()
            .map(|&i| {
                calib
                    .and_then(|c| c.intensity_to_concentration(i))
                    .unwrap_or(i)
            })
            .collect();
        DepthProfileChannel {
            element: element.to_string(),
            wavelength_nm,
            depth_um,
            raw_intensity: intensities.to_vec(),
            concentration_wt_pct,
        }
    }
}

// ---------------------------------------------------------------------------
// Crater shape correction
// ---------------------------------------------------------------------------

/// Models non-uniform sputtering across the crater diameter.
///
/// In a Grimm source the plasma density is highest at the crater edge,
/// causing a slightly non-flat crater bottom.  A Gaussian correction
/// can be applied to the depth axis.
#[derive(Debug, Clone)]
pub struct CraterShapeCorrector {
    /// Relative depth variation (edge depth / centre depth – 1).
    /// Typical values: 0.05–0.15.
    pub edge_to_centre_ratio: f64,
    /// Fraction of crater radius at which the Gaussian knee occurs.
    pub knee_fraction: f64,
}

impl CraterShapeCorrector {
    /// Create a corrector with default Grimm-source parameters.
    pub fn default_grimm() -> Self {
        Self { edge_to_centre_ratio: 0.08, knee_fraction: 0.70 }
    }

    /// Compute the correction factor for a given normalised radial position r/R.
    ///
    /// Returns a depth multiplier: corrected_depth = nominal_depth * factor(r).
    pub fn factor(&self, r_over_r_max: f64) -> f64 {
        // Gaussian falloff from edge
        let x = (r_over_r_max - self.knee_fraction) / (1.0 - self.knee_fraction);
        let x = x.max(0.0);
        1.0 + self.edge_to_centre_ratio * (1.0 - (-x * x * 4.0).exp())
    }

    /// Area-averaged correction factor (integral over circular cross-section).
    ///
    /// Computed numerically with 1000 radial steps.
    pub fn area_averaged_factor(&self) -> f64 {
        let n = 1000usize;
        let mut sum = 0.0_f64;
        for k in 0..n {
            let r = (k as f64 + 0.5) / n as f64; // normalised radius in [0,1)
            sum += self.factor(r) * r; // weight by r (area element = r dr dθ)
        }
        // Normalise: ∫₀¹ r dr = 0.5
        sum / (n as f64 * 0.5)
    }

    /// Apply the area-averaged correction to a depth profile.
    pub fn correct_depth_profile(&self, depth_um: &[f64]) -> Vec<f64> {
        let f = self.area_averaged_factor();
        depth_um.iter().map(|&d| d / f).collect()
    }
}

// ---------------------------------------------------------------------------
// Interface detection
// ---------------------------------------------------------------------------

/// Detected interface between two layers.
#[derive(Debug, Clone)]
pub struct Interface {
    /// Depth at the interface centre (µm).
    pub depth_um: f64,
    /// Interface width (10–90 % transition width, µm).
    pub width_um: f64,
    /// Channel index (element) that showed the strongest transition.
    pub channel_index: usize,
    /// Gradient magnitude at the interface (wt% µm⁻¹).
    pub gradient_magnitude: f64,
}

/// Detects layer boundaries in a depth profile.
#[derive(Debug, Clone)]
pub struct InterfaceDetector {
    /// Detection method.
    pub method: InterfaceMethod,
    /// Minimum gradient to count as an interface (wt% µm⁻¹).
    pub min_gradient: f64,
    /// Minimum depth spacing between successive interfaces (µm).
    pub min_spacing_um: f64,
}

impl InterfaceDetector {
    /// Create a detector with default parameters.
    pub fn new(method: InterfaceMethod) -> Self {
        Self { method, min_gradient: 1.0, min_spacing_um: 0.5 }
    }

    /// Compute numerical first derivative of y with respect to x.
    pub fn first_derivative(x: &[f64], y: &[f64]) -> Vec<f64> {
        let n = x.len().min(y.len());
        let mut dy = vec![0.0_f64; n];
        for i in 1..n - 1 {
            let dx = x[i + 1] - x[i - 1];
            if dx.abs() > f64::EPSILON {
                dy[i] = (y[i + 1] - y[i - 1]) / dx;
            }
        }
        if n >= 2 {
            dy[0] = (y[1] - y[0]) / (x[1] - x[0] + f64::EPSILON);
            dy[n - 1] = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2] + f64::EPSILON);
        }
        dy
    }

    /// Fit sigmoid C = C_lo + (C_hi – C_lo) / (1 + exp(–k(z – z0))) to a step.
    ///
    /// Returns (z0, k) – centre depth and steepness (µm⁻¹).
    pub fn fit_sigmoid(depth: &[f64], conc: &[f64]) -> (f64, f64) {
        let n = conc.len();
        if n < 4 {
            return (depth[n / 2], 1.0);
        }
        let c_lo = conc[..n / 4].iter().copied().fold(0.0, f64::min);
        let c_hi = conc[3 * n / 4..].iter().copied().fold(0.0, f64::max);
        let c_mid = (c_lo + c_hi) / 2.0;
        // z0: find index closest to c_mid
        let idx = conc
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                ((*a - c_mid).abs()).partial_cmp(&((*b - c_mid).abs())).unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)
            .unwrap_or(n / 2);
        let z0 = depth[idx];
        // Estimate k from 10-90% width
        let c10 = c_lo + 0.10 * (c_hi - c_lo);
        let c90 = c_lo + 0.90 * (c_hi - c_lo);
        let z10 = depth.iter().zip(conc.iter())
            .find(|(_, &c)| c >= c10).map(|(&z, _)| z).unwrap_or(depth[0]);
        let z90 = depth.iter().zip(conc.iter())
            .find(|(_, &c)| c >= c90).map(|(&z, _)| z).unwrap_or(depth[n - 1]);
        let width = (z90 - z10).abs().max(f64::EPSILON);
        let k = 4.394 / width; // ln(9) / width ≈ 4.394
        (z0, k)
    }

    /// Detect interfaces in a single depth-profile channel.
    ///
    /// Returns list of detected interfaces sorted by depth.
    pub fn detect(&self, channel: &DepthProfileChannel) -> Vec<Interface> {
        let depth = &channel.depth_um;
        let conc = &channel.concentration_wt_pct;
        if depth.len() < 4 {
            return Vec::new();
        }
        let grad = Self::first_derivative(depth, conc);
        let abs_grad: Vec<f64> = grad.iter().map(|g| g.abs()).collect();

        let mut interfaces = Vec::new();
        let n = abs_grad.len();
        let mut last_depth = f64::NEG_INFINITY;

        for i in 1..n - 1 {
            // Peak detection: local maximum in |dC/dz|.
            // Use >= on left to handle plateaus at transition edges (sharp steps
            // produce equal gradient values at consecutive points).
            if abs_grad[i] >= abs_grad[i - 1]
                && abs_grad[i] > abs_grad[i + 1]
                && abs_grad[i] >= self.min_gradient
                && (depth[i] - last_depth) >= self.min_spacing_um
            {
                let iface = match self.method {
                    InterfaceMethod::FirstDerivativePeak => Interface {
                        depth_um: depth[i],
                        width_um: 0.0,
                        channel_index: 0,
                        gradient_magnitude: abs_grad[i],
                    },
                    InterfaceMethod::SigmoidInflection => {
                        // Take a window of ±10 points around the peak
                        let lo = i.saturating_sub(10);
                        let hi = (i + 10).min(n - 1);
                        let (z0, k) = Self::fit_sigmoid(&depth[lo..=hi], &conc[lo..=hi]);
                        Interface {
                            depth_um: z0,
                            width_um: 4.394 / k.max(f64::EPSILON), // 10–90% width
                            channel_index: 0,
                            gradient_magnitude: abs_grad[i],
                        }
                    }
                    InterfaceMethod::ThresholdCrossing(thresh) => {
                        // depth where conc crosses (c_lo + thresh*(c_hi - c_lo))
                        let c_lo = conc[..i].iter().copied().fold(f64::INFINITY, f64::min);
                        let c_hi = conc[i..].iter().copied().fold(f64::NEG_INFINITY, f64::max);
                        let c_thresh = c_lo + thresh * (c_hi - c_lo);
                        let idx_cross = conc
                            .iter()
                            .enumerate()
                            .find(|(_, &c)| c >= c_thresh)
                            .map(|(idx, _)| idx)
                            .unwrap_or(i);
                        Interface {
                            depth_um: depth[idx_cross],
                            width_um: 0.0,
                            channel_index: 0,
                            gradient_magnitude: abs_grad[i],
                        }
                    }
                };
                last_depth = iface.depth_um;
                interfaces.push(iface);
            }
        }
        interfaces.sort_by(|a, b| a.depth_um.partial_cmp(&b.depth_um).unwrap_or(std::cmp::Ordering::Equal));
        interfaces
    }

    /// Detect interfaces across multiple channels and merge close-spaced ones.
    pub fn detect_multi(&self, channels: &[DepthProfileChannel]) -> Vec<Interface> {
        let mut all: Vec<Interface> = Vec::new();
        for (ch_idx, ch) in channels.iter().enumerate() {
            let mut detected = self.detect(ch);
            for iface in &mut detected {
                iface.channel_index = ch_idx;
            }
            all.extend(detected);
        }
        // Sort by depth and merge within min_spacing_um
        all.sort_by(|a, b| a.depth_um.partial_cmp(&b.depth_um).unwrap_or(std::cmp::Ordering::Equal));
        let mut merged: Vec<Interface> = Vec::new();
        for iface in all {
            if let Some(last) = merged.last_mut() {
                if (iface.depth_um - last.depth_um).abs() < self.min_spacing_um {
                    if iface.gradient_magnitude > last.gradient_magnitude {
                        *last = iface;
                    }
                    continue;
                }
            }
            merged.push(iface);
        }
        merged
    }
}

// ---------------------------------------------------------------------------
// Layer thickness measurement
// ---------------------------------------------------------------------------

/// Measures layer thicknesses from interface positions.
pub struct LayerThicknessMeasurer;

impl LayerThicknessMeasurer {
    /// Compute layer thicknesses (µm) from a sorted list of interface depths.
    ///
    /// The first layer starts at depth 0 (surface) and ends at the first interface.
    pub fn from_interfaces(interfaces: &[Interface]) -> Vec<f64> {
        let mut thicknesses = Vec::new();
        let mut prev_depth = 0.0_f64;
        for iface in interfaces {
            thicknesses.push(iface.depth_um - prev_depth);
            prev_depth = iface.depth_um;
        }
        thicknesses
    }
}

// ---------------------------------------------------------------------------
// Emission yield and self-absorption
// ---------------------------------------------------------------------------

/// Models emission yield and self-absorption for quantification correction.
#[derive(Debug, Clone)]
pub struct EmissionYieldCorrector {
    /// Optical depth factor τ₀ (dimensionless).  Typical values 0–0.5.
    pub tau_0: f64,
    /// Linear emission yield coefficient (counts s⁻¹ per wt% per µg s⁻¹ sputter rate).
    pub yield_coeff: f64,
}

impl EmissionYieldCorrector {
    /// Create a corrector with given parameters.
    pub fn new(tau_0: f64, yield_coeff: f64) -> Self {
        Self { tau_0, yield_coeff }
    }

    /// Self-absorption correction factor (Beer-Lambert type).
    ///
    /// I_corrected = I_measured / (1 – exp(–τ₀)) · τ₀   (for τ₀ > 0)
    /// For τ₀ → 0 the correction factor → 1.
    pub fn self_absorption_factor(&self, concentration_wt_pct: f64) -> f64 {
        let tau = self.tau_0 * concentration_wt_pct / 100.0;
        if tau < 1.0e-6 {
            1.0
        } else {
            tau / (1.0 - (-tau).exp())
        }
    }

    /// Correct an array of intensities for self-absorption given concentration values.
    pub fn correct_intensities(&self, intensities: &[f64], concentrations: &[f64]) -> Vec<f64> {
        intensities
            .iter()
            .zip(concentrations.iter())
            .map(|(&i, &c)| i * self.self_absorption_factor(c))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Hydrogen analysis
// ---------------------------------------------------------------------------

/// Handles hydrogen analysis and molecular band interference corrections.
#[derive(Debug, Clone)]
pub struct HydrogenAnalyzer {
    /// H emission line wavelength (nm) – typically 656.28 nm (Hα).
    pub h_wavelength_nm: f64,
    /// Background correction factor for OH molecular band overlap (0–1).
    pub oh_band_correction: f64,
}

impl HydrogenAnalyzer {
    /// Create a standard hydrogen analyser (Hα line).
    pub fn new() -> Self {
        Self { h_wavelength_nm: 656.279, oh_band_correction: 0.03 }
    }

    /// Apply OH molecular band interference correction.
    ///
    /// `raw_h_intensity`: measured H intensity.
    /// `o_intensity`: measured oxygen intensity at its reference line.
    /// Returns corrected H intensity.
    pub fn correct_oh_interference(&self, raw_h_intensity: f64, o_intensity: f64) -> f64 {
        (raw_h_intensity - self.oh_band_correction * o_intensity).max(0.0)
    }

    /// Estimate H concentration (wppm) from corrected intensity using
    /// empirical calibration constant k_H.
    pub fn concentration_wppm(&self, corrected_intensity: f64, k_h: f64) -> f64 {
        (k_h * corrected_intensity).max(0.0)
    }
}

impl Default for HydrogenAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Coating presets
// ---------------------------------------------------------------------------

/// Information about a coating layer in a preset.
#[derive(Debug, Clone)]
pub struct CoatingLayer {
    /// Layer name.
    pub name: String,
    /// Nominal thickness (µm).
    pub thickness_um: f64,
    /// Primary elements and their concentrations (wt%).
    pub composition: Vec<(String, f64)>,
    /// Material density (g cm⁻³).
    pub density_g_cm3: f64,
}

/// Pre-built coating system definition.
#[derive(Debug, Clone)]
pub struct CoatingSystem {
    /// Preset identifier.
    pub preset: CoatingPreset,
    /// Human-readable name.
    pub name: String,
    /// Ordered layers (index 0 = surface, last = substrate).
    pub layers: Vec<CoatingLayer>,
    /// Recommended analytical lines (element, wavelength_nm).
    pub analytical_lines: Vec<(String, f64)>,
}

impl CoatingSystem {
    /// Create a CoatingSystem from a preset.
    pub fn from_preset(preset: CoatingPreset) -> Self {
        match preset {
            CoatingPreset::GalvanisedSteel => Self {
                preset,
                name: "Galvanised Steel (Zn / Fe)".to_string(),
                layers: vec![
                    CoatingLayer {
                        name: "Zn coating".to_string(),
                        thickness_um: 20.0,
                        composition: vec![("Zn".to_string(), 100.0)],
                        density_g_cm3: 7.133,
                    },
                    CoatingLayer {
                        name: "Fe substrate".to_string(),
                        thickness_um: f64::INFINITY,
                        composition: vec![("Fe".to_string(), 99.5), ("C".to_string(), 0.5)],
                        density_g_cm3: 7.874,
                    },
                ],
                analytical_lines: vec![
                    ("Zn".to_string(), 213.857),
                    ("Fe".to_string(), 371.994),
                ],
            },
            CoatingPreset::TiNHardCoating => Self {
                preset,
                name: "TiN Hard Coating on Steel".to_string(),
                layers: vec![
                    CoatingLayer {
                        name: "TiN".to_string(),
                        thickness_um: 3.0,
                        composition: vec![("Ti".to_string(), 77.4), ("N".to_string(), 22.6)],
                        density_g_cm3: 5.430,
                    },
                    CoatingLayer {
                        name: "Steel substrate".to_string(),
                        thickness_um: f64::INFINITY,
                        composition: vec![
                            ("Fe".to_string(), 97.0),
                            ("Cr".to_string(), 1.5),
                            ("C".to_string(), 1.5),
                        ],
                        density_g_cm3: 7.874,
                    },
                ],
                analytical_lines: vec![
                    ("Ti".to_string(), 365.350),
                    ("N".to_string(), 149.263),
                    ("Fe".to_string(), 371.994),
                ],
            },
            CoatingPreset::AluminaOnAluminium => Self {
                preset,
                name: "Al₂O₃ oxide on Aluminium alloy".to_string(),
                layers: vec![
                    CoatingLayer {
                        name: "Al2O3".to_string(),
                        thickness_um: 0.5,
                        composition: vec![("Al".to_string(), 52.9), ("O".to_string(), 47.1)],
                        density_g_cm3: 3.987,
                    },
                    CoatingLayer {
                        name: "Al alloy".to_string(),
                        thickness_um: f64::INFINITY,
                        composition: vec![
                            ("Al".to_string(), 95.0),
                            ("Mg".to_string(), 3.0),
                            ("Si".to_string(), 2.0),
                        ],
                        density_g_cm3: 2.700,
                    },
                ],
                analytical_lines: vec![
                    ("Al".to_string(), 396.152),
                    ("O".to_string(), 130.217),
                ],
            },
            CoatingPreset::ChromiaOnSteel => Self {
                preset,
                name: "Cr₂O₃ passive film on stainless steel".to_string(),
                layers: vec![
                    CoatingLayer {
                        name: "Cr2O3".to_string(),
                        thickness_um: 0.03,
                        composition: vec![("Cr".to_string(), 68.4), ("O".to_string(), 31.6)],
                        density_g_cm3: 5.220,
                    },
                    CoatingLayer {
                        name: "Stainless steel".to_string(),
                        thickness_um: f64::INFINITY,
                        composition: vec![
                            ("Fe".to_string(), 70.0),
                            ("Cr".to_string(), 18.0),
                            ("Ni".to_string(), 10.0),
                            ("Mo".to_string(), 2.0),
                        ],
                        density_g_cm3: 7.970,
                    },
                ],
                analytical_lines: vec![
                    ("Cr".to_string(), 425.435),
                    ("O".to_string(), 130.217),
                    ("Fe".to_string(), 371.994),
                ],
            },
            CoatingPreset::ZincIronDuplex => Self {
                preset,
                name: "Duplex Zn–Fe / Fe substrate".to_string(),
                layers: vec![
                    CoatingLayer {
                        name: "Pure Zn".to_string(),
                        thickness_um: 10.0,
                        composition: vec![("Zn".to_string(), 100.0)],
                        density_g_cm3: 7.133,
                    },
                    CoatingLayer {
                        name: "Zn-Fe intermetallic (delta)".to_string(),
                        thickness_um: 5.0,
                        composition: vec![("Zn".to_string(), 90.0), ("Fe".to_string(), 10.0)],
                        density_g_cm3: 7.250,
                    },
                    CoatingLayer {
                        name: "Fe substrate".to_string(),
                        thickness_um: f64::INFINITY,
                        composition: vec![("Fe".to_string(), 99.5), ("C".to_string(), 0.5)],
                        density_g_cm3: 7.874,
                    },
                ],
                analytical_lines: vec![
                    ("Zn".to_string(), 213.857),
                    ("Fe".to_string(), 371.994),
                ],
            },
        }
    }
}

// ---------------------------------------------------------------------------
// Top-level processor
// ---------------------------------------------------------------------------

/// Complete GD-OES depth-profile measurement result.
#[derive(Debug, Clone)]
pub struct GdoesMeasurement {
    /// Source parameters used.
    pub source: GrimSource,
    /// Depth profile channels (one per element/line).
    pub channels: Vec<DepthProfileChannel>,
    /// Detected interfaces.
    pub interfaces: Vec<Interface>,
    /// Layer thicknesses (µm) between successive interfaces.
    pub layer_thicknesses_um: Vec<f64>,
    /// Total sputtered depth (µm).
    pub total_depth_um: f64,
    /// Total sputtering time (s).
    pub total_time_s: f64,
}

/// Orchestrates a complete GD-OES depth-profile analysis.
#[derive(Debug, Clone)]
pub struct GdoesProcessor {
    /// Discharge source.
    pub source: GrimSource,
    /// Bulk material properties (best estimate for the primary layer).
    pub material: MaterialProperties,
    /// Crater shape corrector.
    pub crater_corrector: CraterShapeCorrector,
    /// Interface detector.
    pub interface_detector: InterfaceDetector,
}

impl GdoesProcessor {
    /// Create a processor with default settings for a DC Grimm source on steel.
    pub fn new_steel_dc() -> Self {
        Self {
            source: GrimSource::standard_dc(),
            material: MaterialProperties::iron(),
            crater_corrector: CraterShapeCorrector::default_grimm(),
            interface_detector: InterfaceDetector::new(InterfaceMethod::FirstDerivativePeak),
        }
    }

    /// Run a complete depth-profile analysis.
    ///
    /// `times_s`: measurement time points (s).
    /// `channel_data`: slice of (element, wavelength_nm, intensities, optional calibration).
    pub fn process(
        &self,
        times_s: &[f64],
        channel_data: &[(&str, f64, Vec<f64>, Option<CalibrationCurve>)],
    ) -> GdoesMeasurement {
        let builder = DepthProfileBuilder::new(self.source.clone(), self.material.clone());

        let channels: Vec<DepthProfileChannel> = channel_data
            .iter()
            .map(|(elem, wl, intensities, calib)| {
                builder.build(times_s, intensities, elem, *wl, calib.as_ref())
            })
            .collect();

        // Apply crater shape correction to the depth axis of the first channel
        // (all channels share the same depth axis by assumption)
        let corrected_depths = if let Some(first) = channels.first() {
            self.crater_corrector.correct_depth_profile(&first.depth_um)
        } else {
            vec![]
        };

        // Build corrected channels
        let mut corrected_channels = channels.clone();
        for ch in &mut corrected_channels {
            ch.depth_um = corrected_depths.clone();
        }

        let interfaces = self.interface_detector.detect_multi(&corrected_channels);
        let layer_thicknesses_um = LayerThicknessMeasurer::from_interfaces(&interfaces);
        let total_depth_um = corrected_depths.last().copied().unwrap_or(0.0);
        let total_time_s = times_s.last().copied().unwrap_or(0.0);

        GdoesMeasurement {
            source: self.source.clone(),
            channels: corrected_channels,
            interfaces,
            layer_thicknesses_um,
            total_depth_um,
            total_time_s,
        }
    }

    /// Estimate bulk concentration from a steady-state intensity plateau.
    ///
    /// Averages the concentration over the given depth range [z_lo, z_hi] (µm).
    pub fn bulk_concentration(
        channel: &DepthProfileChannel,
        z_lo_um: f64,
        z_hi_um: f64,
    ) -> Option<f64> {
        let values: Vec<f64> = channel
            .depth_um
            .iter()
            .zip(channel.concentration_wt_pct.iter())
            .filter(|(&z, _)| z >= z_lo_um && z <= z_hi_um)
            .map(|(_, &c)| c)
            .collect();
        if values.is_empty() {
            return None;
        }
        Some(values.iter().sum::<f64>() / values.len() as f64)
    }
}

// ---------------------------------------------------------------------------
// Utility: simple signal smoothing (Savitzky-Golay 5-point quadratic)
// ---------------------------------------------------------------------------

/// Apply a simple 5-point quadratic Savitzky–Golay smooth to a signal.
pub fn smooth_sg5(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n < 5 {
        return signal.to_vec();
    }
    let mut out = signal.to_vec();
    for i in 2..n - 2 {
        out[i] = (-3.0 * signal[i - 2]
            + 12.0 * signal[i - 1]
            + 17.0 * signal[i]
            + 12.0 * signal[i + 1]
            - 3.0 * signal[i + 2])
            / 35.0;
    }
    out
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- GrimSource tests --------------------------------------------------

    #[test]
    fn test_grimm_dc_defaults() {
        let src = GrimSource::standard_dc();
        assert_eq!(src.mode, SourceMode::Dc);
        assert!((src.voltage_v - 800.0).abs() < f64::EPSILON);
        assert!((src.current_a - 0.040).abs() < f64::EPSILON);
    }

    #[test]
    fn test_grimm_rf_defaults() {
        let src = GrimSource::standard_rf();
        assert_eq!(src.mode, SourceMode::Rf);
        assert!(src.voltage_v > 0.0);
    }

    #[test]
    fn test_crater_area_4mm() {
        let src = GrimSource::standard_dc();
        let area = src.crater_area_cm2();
        // Expected: pi * (0.2 cm)^2 = pi * 0.04 ≈ 0.12566 cm²
        assert!((area - PI * 0.04).abs() < 1.0e-9);
    }

    #[test]
    fn test_current_density_positive() {
        let src = GrimSource::standard_dc();
        assert!(src.current_density_a_cm2() > 0.0);
    }

    #[test]
    fn test_power_calculation() {
        let src = GrimSource::standard_dc();
        let power = src.power_w();
        assert!((power - 32.0).abs() < 1.0e-10); // 800 V * 0.040 A = 32 W
    }

    #[test]
    fn test_reduced_field_positive() {
        let src = GrimSource::standard_dc();
        assert!(src.reduced_field_v_cm_pa() > 0.0);
    }

    // ---- SputterRateCalculator tests ---------------------------------------

    #[test]
    fn test_sputter_rate_iron_positive() {
        let calc = SputterRateCalculator::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let rate = calc.sputter_rate_ug_per_s();
        assert!(rate > 0.0, "sputter rate should be positive, got {rate}");
    }

    #[test]
    fn test_depth_rate_iron_positive() {
        let calc = SputterRateCalculator::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let dr = calc.depth_rate_um_per_s();
        assert!(dr > 0.0);
    }

    #[test]
    fn test_depth_rate_zinc_vs_iron() {
        // Zinc has higher sputter yield, so should have higher depth rate
        let iron = SputterRateCalculator::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let zinc = SputterRateCalculator::new(
            GrimSource::standard_dc(),
            MaterialProperties::zinc(),
        );
        assert!(zinc.depth_rate_um_per_s() > iron.depth_rate_um_per_s());
    }

    #[test]
    fn test_cumulative_depth_monotonic() {
        let calc = SputterRateCalculator::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let times: Vec<f64> = (0..=10).map(|i| i as f64).collect();
        let depths = calc.cumulative_depth_um(&times);
        for i in 1..depths.len() {
            assert!(depths[i] >= depths[i - 1]);
        }
    }

    #[test]
    fn test_depth_from_mass_loss() {
        let calc = SputterRateCalculator::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        // 1 µg mass loss on iron: depth = 1e-6 g / (7.874 g/cm³ * pi*(0.2)^2 cm²)
        let expected_cm = 1.0e-6 / (7.874 * PI * 0.04);
        let expected_um = expected_cm * 1.0e4;
        let result = calc.depth_from_mass_loss_um(1.0);
        assert!((result - expected_um).abs() < 1.0e-10);
    }

    // ---- EmissionLineDb tests ----------------------------------------------

    #[test]
    fn test_all_lines_not_empty() {
        assert!(!EmissionLineDb::all_lines().is_empty());
    }

    #[test]
    fn test_all_lines_wavelength_positive() {
        for line in EmissionLineDb::all_lines() {
            assert!(line.wavelength_nm > 0.0, "Zero wavelength for {}", line.element);
        }
    }

    #[test]
    fn test_lines_for_element_fe() {
        let fe_lines = EmissionLineDb::lines_for_element("Fe");
        assert!(!fe_lines.is_empty());
        for l in &fe_lines {
            assert_eq!(l.element, "Fe");
        }
    }

    #[test]
    fn test_primary_line_zn_highest_rsf() {
        let primary = EmissionLineDb::primary_line("Zn").expect("Zn primary line");
        let all = EmissionLineDb::lines_for_element("Zn");
        let max_rsf = all.iter().map(|l| l.rsf).fold(f64::NEG_INFINITY, f64::max);
        assert!((primary.rsf - max_rsf).abs() < f64::EPSILON);
    }

    #[test]
    fn test_find_near_wavelength() {
        let found = EmissionLineDb::find_near(371.994, 0.5);
        assert!(!found.is_empty());
        assert!(found.iter().any(|l| l.element == "Fe"));
    }

    #[test]
    fn test_find_near_empty_for_out_of_range() {
        let found = EmissionLineDb::find_near(999.0, 0.1);
        assert!(found.is_empty());
    }

    // ---- CalibrationCurve tests --------------------------------------------

    #[test]
    fn test_linear_calibration_zero_at_zero() {
        let c = CalibrationCurve::linear(2.5);
        assert!((c.intensity_to_concentration(0.0).unwrap() - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_linear_calibration_proportional() {
        let c = CalibrationCurve::linear(3.0);
        let conc = c.intensity_to_concentration(10.0).unwrap();
        assert!((conc - 30.0).abs() < 1.0e-10);
    }

    #[test]
    fn test_quadratic_calibration() {
        let c = CalibrationCurve::quadratic(2.0, 0.1);
        let conc = c.intensity_to_concentration(5.0).unwrap();
        // 2*5 + 0.1*25 = 10 + 2.5 = 12.5
        assert!((conc - 12.5).abs() < 1.0e-10);
    }

    #[test]
    fn test_logarithmic_calibration_negative_intensity_returns_none() {
        let c = CalibrationCurve::logarithmic(1.0, 0.0);
        assert!(c.intensity_to_concentration(0.0).is_none());
    }

    #[test]
    fn test_logarithmic_calibration_positive() {
        let c = CalibrationCurve::logarithmic(1.0, 0.0);
        let val = c.intensity_to_concentration(1.0).unwrap();
        // ln(1) = 0 → concentration = 0 (clamped to 0)
        assert!((val - 0.0).abs() < 1.0e-10);
    }

    #[test]
    fn test_fit_linear_single_point() {
        let pairs = vec![(10.0, 30.0)]; // a = 30/10 = 3
        let c = CalibrationCurve::fit_linear(&pairs);
        assert!((c.a - 3.0).abs() < 1.0e-10);
    }

    #[test]
    fn test_fit_linear_multiple_points() {
        // Perfect linear data: C = 5 * I
        let pairs: Vec<(f64, f64)> = (1..=5).map(|i| (i as f64, 5.0 * i as f64)).collect();
        let c = CalibrationCurve::fit_linear(&pairs);
        assert!((c.a - 5.0).abs() < 1.0e-9);
    }

    #[test]
    fn test_inverse_linear_calibration() {
        let c = CalibrationCurve::linear(4.0);
        let intensity = c.concentration_to_intensity(20.0).unwrap();
        assert!((intensity - 5.0).abs() < 1.0e-10);
    }

    #[test]
    fn test_fit_quadratic_returns_calib() {
        // C = 2I + 0.5 I², test that fitting recovers approx coefficients
        let pairs: Vec<(f64, f64)> = (1..=6)
            .map(|i| { let x = i as f64; (x, 2.0 * x + 0.5 * x * x) })
            .collect();
        let c = CalibrationCurve::fit_quadratic(&pairs);
        // Check that C(4) ≈ 2*4 + 0.5*16 = 16
        let pred = c.intensity_to_concentration(4.0).unwrap();
        assert!((pred - 16.0).abs() < 0.5, "Predicted {pred}, expected ~16");
    }

    // ---- MatrixCorrector tests ---------------------------------------------

    #[test]
    fn test_matrix_corrector_identity() {
        let mc = MatrixCorrector::identity();
        let result = mc.apply(100.0, &[("Fe", 99.0)]);
        assert!((result - 100.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_matrix_corrector_applies_factor() {
        let mut mc = MatrixCorrector::identity();
        mc.add_correction("Cr", 0.02);
        // factor = 1 + 0.02 * 18.0 = 1.36
        let result = mc.apply(100.0, &[("Cr", 18.0)]);
        assert!((result - 136.0).abs() < 1.0e-9);
    }

    // ---- DepthProfileBuilder tests -----------------------------------------

    #[test]
    fn test_depth_profile_builder_length() {
        let builder = DepthProfileBuilder::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let times: Vec<f64> = (0..=20).map(|i| i as f64).collect();
        let intensities: Vec<f64> = vec![1.0; times.len()];
        let channel = builder.build(&times, &intensities, "Fe", 371.994, None);
        assert_eq!(channel.depth_um.len(), times.len());
        assert_eq!(channel.concentration_wt_pct.len(), times.len());
    }

    #[test]
    fn test_depth_profile_depth_zero_at_start() {
        let builder = DepthProfileBuilder::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let times = vec![0.0, 1.0, 2.0];
        let intensities = vec![10.0, 10.0, 10.0];
        let channel = builder.build(&times, &intensities, "Fe", 371.994, None);
        assert!((channel.depth_um[0] - 0.0).abs() < f64::EPSILON);
    }

    // ---- CraterShapeCorrector tests ----------------------------------------

    #[test]
    fn test_crater_factor_at_centre_is_one() {
        let csc = CraterShapeCorrector::default_grimm();
        // At r/R = 0 (centre), factor should be ≈ 1.0 (very small correction)
        let f = csc.factor(0.0);
        assert!((f - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_crater_factor_at_edge_greater_than_centre() {
        let csc = CraterShapeCorrector::default_grimm();
        assert!(csc.factor(1.0) >= csc.factor(0.0));
    }

    #[test]
    fn test_area_averaged_factor_near_one() {
        let csc = CraterShapeCorrector::default_grimm();
        let f = csc.area_averaged_factor();
        // Should be close to 1 + edge_to_centre_ratio/2 roughly
        assert!(f >= 1.0 && f < 1.15);
    }

    #[test]
    fn test_correct_depth_profile_reduces_depth() {
        let csc = CraterShapeCorrector::default_grimm();
        let depth = vec![0.0, 1.0, 2.0, 3.0];
        let corrected = csc.correct_depth_profile(&depth);
        // Correction factor > 1, so corrected depths should be smaller
        for i in 1..depth.len() {
            assert!(corrected[i] <= depth[i] + 1.0e-9);
        }
    }

    // ---- InterfaceDetector tests -------------------------------------------

    #[test]
    fn test_first_derivative_flat_signal() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![5.0; 5];
        let dy = InterfaceDetector::first_derivative(&x, &y);
        for d in &dy {
            assert!(d.abs() < 1.0e-10);
        }
    }

    #[test]
    fn test_first_derivative_ramp() {
        // y = x → dy = 1 everywhere (interior)
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y = x.clone();
        let dy = InterfaceDetector::first_derivative(&x, &y);
        for i in 1..dy.len() - 1 {
            assert!((dy[i] - 1.0).abs() < 1.0e-10, "dy[{i}] = {}", dy[i]);
        }
    }

    #[test]
    fn test_interface_detect_step() {
        // Simulate a step in Zn concentration at depth 10 µm
        let n = 100usize;
        let depth: Vec<f64> = (0..n).map(|i| i as f64 * 0.5).collect(); // 0..49.5 µm
        let conc: Vec<f64> = depth.iter().map(|&z| if z < 10.0 { 100.0 } else { 0.0 }).collect();
        let channel = DepthProfileChannel {
            element: "Zn".to_string(),
            wavelength_nm: 213.857,
            depth_um: depth,
            raw_intensity: conc.clone(),
            concentration_wt_pct: conc,
        };
        let detector = InterfaceDetector::new(InterfaceMethod::FirstDerivativePeak);
        let ifaces = detector.detect(&channel);
        assert!(!ifaces.is_empty(), "Should detect at least one interface");
        // Interface should be near 10 µm
        let iface_depth = ifaces[0].depth_um;
        assert!((iface_depth - 10.0).abs() < 2.0, "Interface at {iface_depth}, expected ~10 µm");
    }

    #[test]
    fn test_sigmoid_fit_centre() {
        let n = 50usize;
        // Sigmoid step from 0 to 100 centred at z=5
        let depth: Vec<f64> = (0..n).map(|i| i as f64 * 0.2).collect(); // 0..9.8
        let k = 2.0_f64;
        let z0 = 5.0_f64;
        let conc: Vec<f64> = depth.iter().map(|&z| 100.0 / (1.0 + (-k * (z - z0)).exp())).collect();
        let (z0_est, _k_est) = InterfaceDetector::fit_sigmoid(&depth, &conc);
        assert!((z0_est - z0).abs() < 1.0, "z0 estimated as {z0_est}, expected {z0}");
    }

    // ---- LayerThicknessMeasurer tests -------------------------------------

    #[test]
    fn test_layer_thickness_single_interface() {
        let ifaces = vec![Interface { depth_um: 5.0, width_um: 0.2, channel_index: 0, gradient_magnitude: 10.0 }];
        let t = LayerThicknessMeasurer::from_interfaces(&ifaces);
        assert_eq!(t.len(), 1);
        assert!((t[0] - 5.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_layer_thickness_two_interfaces() {
        let ifaces = vec![
            Interface { depth_um: 5.0, width_um: 0.0, channel_index: 0, gradient_magnitude: 5.0 },
            Interface { depth_um: 15.0, width_um: 0.0, channel_index: 1, gradient_magnitude: 3.0 },
        ];
        let t = LayerThicknessMeasurer::from_interfaces(&ifaces);
        assert_eq!(t.len(), 2);
        assert!((t[0] - 5.0).abs() < f64::EPSILON);
        assert!((t[1] - 10.0).abs() < f64::EPSILON);
    }

    // ---- EmissionYieldCorrector tests -------------------------------------

    #[test]
    fn test_self_absorption_zero_tau_is_one() {
        let eyc = EmissionYieldCorrector::new(0.0, 1.0);
        assert!((eyc.self_absorption_factor(50.0) - 1.0).abs() < 1.0e-9);
    }

    #[test]
    fn test_self_absorption_factor_increases_with_conc() {
        let eyc = EmissionYieldCorrector::new(0.5, 1.0);
        let f10 = eyc.self_absorption_factor(10.0);
        let f50 = eyc.self_absorption_factor(50.0);
        assert!(f50 > f10, "Self-absorption should increase with concentration");
    }

    #[test]
    fn test_correct_intensities_length() {
        let eyc = EmissionYieldCorrector::new(0.1, 1.0);
        let intensities = vec![100.0, 90.0, 80.0];
        let concs = vec![10.0, 20.0, 30.0];
        let corrected = eyc.correct_intensities(&intensities, &concs);
        assert_eq!(corrected.len(), 3);
    }

    // ---- HydrogenAnalyzer tests -------------------------------------------

    #[test]
    fn test_hydrogen_analyzer_default_wavelength() {
        let ha = HydrogenAnalyzer::new();
        assert!((ha.h_wavelength_nm - 656.279).abs() < 0.001);
    }

    #[test]
    fn test_oh_correction_reduces_signal() {
        let ha = HydrogenAnalyzer::new();
        let corrected = ha.correct_oh_interference(100.0, 200.0);
        // 100 - 0.03*200 = 94
        assert!((corrected - 94.0).abs() < 1.0e-9);
    }

    #[test]
    fn test_oh_correction_clamps_to_zero() {
        let ha = HydrogenAnalyzer::new();
        let corrected = ha.correct_oh_interference(1.0, 10000.0);
        assert!((corrected - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_h_concentration_wppm_positive() {
        let ha = HydrogenAnalyzer::new();
        let conc = ha.concentration_wppm(50.0, 0.1);
        assert!((conc - 5.0).abs() < 1.0e-10);
    }

    // ---- CoatingSystem tests ----------------------------------------------

    #[test]
    fn test_galvanised_steel_layers() {
        let sys = CoatingSystem::from_preset(CoatingPreset::GalvanisedSteel);
        assert_eq!(sys.layers.len(), 2);
        assert_eq!(sys.layers[0].name, "Zn coating");
        assert_eq!(sys.layers[1].name, "Fe substrate");
    }

    #[test]
    fn test_tin_coating_analytical_lines() {
        let sys = CoatingSystem::from_preset(CoatingPreset::TiNHardCoating);
        assert!(!sys.analytical_lines.is_empty());
        assert!(sys.analytical_lines.iter().any(|(e, _)| e == "Ti"));
        assert!(sys.analytical_lines.iter().any(|(e, _)| e == "N"));
    }

    #[test]
    fn test_all_presets_have_layers() {
        for preset in [
            CoatingPreset::GalvanisedSteel,
            CoatingPreset::TiNHardCoating,
            CoatingPreset::AluminaOnAluminium,
            CoatingPreset::ChromiaOnSteel,
            CoatingPreset::ZincIronDuplex,
        ] {
            let sys = CoatingSystem::from_preset(preset);
            assert!(!sys.layers.is_empty(), "Preset {:?} has no layers", preset);
        }
    }

    #[test]
    fn test_zinc_iron_duplex_three_layers() {
        let sys = CoatingSystem::from_preset(CoatingPreset::ZincIronDuplex);
        assert_eq!(sys.layers.len(), 3);
    }

    // ---- GdoesProcessor tests ---------------------------------------------

    #[test]
    fn test_processor_new_steel_dc() {
        let proc = GdoesProcessor::new_steel_dc();
        assert_eq!(proc.source.mode, SourceMode::Dc);
    }

    #[test]
    fn test_processor_process_returns_channels() {
        let proc = GdoesProcessor::new_steel_dc();
        let times: Vec<f64> = (0..=30).map(|i| i as f64).collect();
        let intensities: Vec<f64> = (0..=30)
            .map(|i| if i <= 15 { 100.0 } else { 10.0 })
            .collect();
        let calib = CalibrationCurve::linear(0.5);
        let channel_data = vec![("Fe", 371.994, intensities, Some(calib))];
        let result = proc.process(&times, &channel_data);
        assert_eq!(result.channels.len(), 1);
        assert!(!result.channels[0].depth_um.is_empty());
    }

    #[test]
    fn test_bulk_concentration_in_range() {
        let builder = DepthProfileBuilder::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let times: Vec<f64> = (0..=100).map(|i| i as f64).collect();
        let intensities: Vec<f64> = vec![200.0; times.len()];
        let calib = CalibrationCurve::linear(0.5); // C = 0.5 * 200 = 100 wt%
        let channel = builder.build(&times, &intensities, "Fe", 371.994, Some(&calib));
        let bulk = GdoesProcessor::bulk_concentration(&channel, 1.0, 5.0);
        assert!(bulk.is_some());
        let b = bulk.unwrap();
        assert!((b - 100.0).abs() < 1.0e-6);
    }

    #[test]
    fn test_bulk_concentration_out_of_range_none() {
        let builder = DepthProfileBuilder::new(
            GrimSource::standard_dc(),
            MaterialProperties::iron(),
        );
        let times = vec![0.0, 0.001];
        let intensities = vec![100.0, 100.0];
        let channel = builder.build(&times, &intensities, "Fe", 371.994, None);
        // Ask for a range beyond the total depth
        let bulk = GdoesProcessor::bulk_concentration(&channel, 1000.0, 2000.0);
        assert!(bulk.is_none());
    }

    // ---- smooth_sg5 tests -------------------------------------------------

    #[test]
    fn test_smooth_sg5_flat_signal() {
        let signal = vec![5.0; 20];
        let smoothed = smooth_sg5(&signal);
        for s in &smoothed {
            assert!((s - 5.0).abs() < 1.0e-9);
        }
    }

    #[test]
    fn test_smooth_sg5_short_signal_passthrough() {
        let signal = vec![1.0, 2.0, 3.0];
        let smoothed = smooth_sg5(&signal);
        assert_eq!(smoothed, signal);
    }

    #[test]
    fn test_smooth_sg5_preserves_length() {
        let signal: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let smoothed = smooth_sg5(&signal);
        assert_eq!(smoothed.len(), signal.len());
    }

    // ---- Integration test: galvanised steel profile -----------------------

    #[test]
    fn test_galvanised_steel_depth_profile_end_to_end() {
        // Simulate a 20 µm Zn coating on Fe substrate
        // Grimm source sputters Zn at start (Zn=100, Fe=0), then Fe (Zn=0, Fe=100)
        let proc = GdoesProcessor::new_steel_dc();
        let n = 60usize;
        let times: Vec<f64> = (0..n).map(|i| i as f64).collect();

        // Zn: high intensity in first 30 s, drops after
        let zn_int: Vec<f64> = times.iter().map(|&t| if t < 30.0 { 200.0 } else { 5.0 }).collect();
        // Fe: low intensity first 30 s, rises after
        let fe_int: Vec<f64> = times.iter().map(|&t| if t < 30.0 { 5.0 } else { 200.0 }).collect();

        let zn_calib = CalibrationCurve::linear(0.5); // C = 0.5 * I  → 100 wt% at 200 counts
        let fe_calib = CalibrationCurve::linear(0.5);

        let channel_data = vec![
            ("Zn", 213.857, zn_int, Some(zn_calib)),
            ("Fe", 371.994, fe_int, Some(fe_calib)),
        ];

        let result = proc.process(&times, &channel_data);
        assert_eq!(result.channels.len(), 2);
        // Total depth should be positive
        assert!(result.total_depth_um > 0.0);
        // At least one interface should be detected
        assert!(!result.interfaces.is_empty(), "Expected interface between Zn and Fe layers");
    }
}
