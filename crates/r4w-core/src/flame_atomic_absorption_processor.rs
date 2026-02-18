// trace:FR-FAAS | ai:claude
//! # Flame Atomic Absorption Spectroscopy (FAAS) Signal Processor
//!
//! Implements signal processing for quantitative elemental analysis of metals
//! in solution via Flame Atomic Absorption Spectroscopy (FAAS) and Graphite
//! Furnace Atomic Absorption Spectroscopy (GFAAS).
//!
//! ## Key Components
//!
//! - **FaasConfig** - Instrument configuration (wavelength, slit, lamp, flame, nebulizer)
//! - **FaasProcessor** - Core processing: Beer-Lambert law, calibration, background correction
//! - **FaasSpectrum** - Spectral data container with blank subtraction
//! - **FlameType** - Flame chemistry enumeration with temperature models
//! - **ElementDatabase** - Primary/secondary wavelengths and sensitivities for 25+ elements
//!
//! ## Beer-Lambert Law
//!
//! ```text
//! A = -log10(I / I0) = epsilon * b * c
//! ```
//!
//! where `A` is absorbance, `I/I0` is transmittance, `epsilon` is the molar
//! absorptivity (L/(mol*cm)), `b` is the path length (cm), and `c` is the
//! analyte concentration (mol/L or mg/L).
//!
//! ## Background Correction Methods
//!
//! - D2 (deuterium) lamp continuum source correction
//! - Zeeman effect (longitudinal/transverse) magnetic field splitting
//! - Smith-Hieftje pulsed hollow cathode self-reversal correction
//!
//! ## Calibration Approaches
//!
//! - Linear least squares: A = a + b*C
//! - Quadratic calibration for curved response: A = a + b*C + c*C^2
//! - Method of standard additions for matrix-matched analysis
//!
//! ## Interference Corrections
//!
//! - Ionization interference (alkali metal suppression)
//! - Chemical interference (refractory oxide formation, e.g., Ca + PO4)
//! - Spectral overlap corrections

// ============================================================================
// Physical constants
// ============================================================================

/// Boltzmann constant in eV/K
const K_BOLTZMANN_EV: f64 = 8.617333262e-5;

/// Boltzmann constant in J/K
const K_BOLTZMANN_J: f64 = 1.380649e-23;

/// Speed of light in m/s
const C_LIGHT: f64 = 2.99792458e8;

/// Planck constant in J*s
const H_PLANCK: f64 = 6.62607015e-34;

/// Characteristic absorbance: 1% absorption = 0.0044 absorbance units
const CHARACTERISTIC_ABS: f64 = 0.0044;

// ============================================================================
// FlameType
// ============================================================================

/// Types of flame used in FAAS atomization.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlameType {
    /// Air-acetylene flame, ~2300 degC. Suitable for most elements.
    AirAcetylene,
    /// Nitrous oxide-acetylene flame, ~2950 degC. Required for refractory
    /// elements (Al, Si, Ti, V, Zr, etc.) and lanthanides.
    NitrousOxideAcetylene,
}

impl FlameType {
    /// Approximate flame temperature in Kelvin.
    pub fn temperature_k(&self) -> f64 {
        match self {
            FlameType::AirAcetylene => 2573.0,        // ~2300 C
            FlameType::NitrousOxideAcetylene => 3223.0, // ~2950 C
        }
    }

    /// Approximate flame temperature in Celsius.
    pub fn temperature_c(&self) -> f64 {
        self.temperature_k() - 273.15
    }

    /// Name string for display.
    pub fn name(&self) -> &'static str {
        match self {
            FlameType::AirAcetylene => "Air-Acetylene",
            FlameType::NitrousOxideAcetylene => "N2O-Acetylene",
        }
    }
}

// ============================================================================
// Element database entry
// ============================================================================

/// Database entry for an element's FAAS parameters.
#[derive(Debug, Clone)]
pub struct ElementEntry {
    /// Element symbol (e.g., "Cu")
    pub symbol: &'static str,
    /// Element name (e.g., "Copper")
    pub name: &'static str,
    /// Primary analytical wavelength in nm
    pub primary_wavelength_nm: f64,
    /// Secondary wavelength in nm (less sensitive, for high concentrations)
    pub secondary_wavelength_nm: f64,
    /// Characteristic concentration in ug/mL (mg/L) for 1% absorption
    pub characteristic_conc_ug_ml: f64,
    /// Recommended slit width in nm
    pub slit_width_nm: f64,
    /// Recommended flame type
    pub flame_type: FlameType,
    /// Recommended lamp current in mA
    pub lamp_current_ma: f64,
    /// Detection limit in ug/mL (mg/L) for flame AAS
    pub detection_limit_ug_ml: f64,
    /// Linear range upper limit in ug/mL
    pub linear_range_max_ug_ml: f64,
}

/// Returns the built-in wavelength database for 25 common FAAS elements.
///
/// Values are representative of typical instrument manufacturers' cookbook
/// data (Varian, PerkinElmer, Shimadzu, Agilent).
pub fn wavelength_database() -> Vec<ElementEntry> {
    vec![
        ElementEntry {
            symbol: "Ag", name: "Silver",
            primary_wavelength_nm: 328.1, secondary_wavelength_nm: 338.3,
            characteristic_conc_ug_ml: 0.04, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 4.0,
            detection_limit_ug_ml: 0.002, linear_range_max_ug_ml: 4.0,
        },
        ElementEntry {
            symbol: "Al", name: "Aluminum",
            primary_wavelength_nm: 309.3, secondary_wavelength_nm: 396.2,
            characteristic_conc_ug_ml: 1.0, slit_width_nm: 0.5,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.03, linear_range_max_ug_ml: 100.0,
        },
        ElementEntry {
            symbol: "Au", name: "Gold",
            primary_wavelength_nm: 242.8, secondary_wavelength_nm: 267.6,
            characteristic_conc_ug_ml: 0.15, slit_width_nm: 1.0,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 4.0,
            detection_limit_ug_ml: 0.01, linear_range_max_ug_ml: 20.0,
        },
        ElementEntry {
            symbol: "Ba", name: "Barium",
            primary_wavelength_nm: 553.6, secondary_wavelength_nm: 350.1,
            characteristic_conc_ug_ml: 0.4, slit_width_nm: 0.5,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.02, linear_range_max_ug_ml: 20.0,
        },
        ElementEntry {
            symbol: "Ca", name: "Calcium",
            primary_wavelength_nm: 422.7, secondary_wavelength_nm: 239.9,
            characteristic_conc_ug_ml: 0.08, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.001, linear_range_max_ug_ml: 5.0,
        },
        ElementEntry {
            symbol: "Cd", name: "Cadmium",
            primary_wavelength_nm: 228.8, secondary_wavelength_nm: 326.1,
            characteristic_conc_ug_ml: 0.02, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 4.0,
            detection_limit_ug_ml: 0.001, linear_range_max_ug_ml: 2.0,
        },
        ElementEntry {
            symbol: "Co", name: "Cobalt",
            primary_wavelength_nm: 240.7, secondary_wavelength_nm: 252.1,
            characteristic_conc_ug_ml: 0.12, slit_width_nm: 0.2,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 7.0,
            detection_limit_ug_ml: 0.005, linear_range_max_ug_ml: 5.0,
        },
        ElementEntry {
            symbol: "Cr", name: "Chromium",
            primary_wavelength_nm: 357.9, secondary_wavelength_nm: 425.4,
            characteristic_conc_ug_ml: 0.08, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 7.0,
            detection_limit_ug_ml: 0.003, linear_range_max_ug_ml: 5.0,
        },
        ElementEntry {
            symbol: "Cu", name: "Copper",
            primary_wavelength_nm: 324.8, secondary_wavelength_nm: 327.4,
            characteristic_conc_ug_ml: 0.04, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 4.0,
            detection_limit_ug_ml: 0.002, linear_range_max_ug_ml: 5.0,
        },
        ElementEntry {
            symbol: "Fe", name: "Iron",
            primary_wavelength_nm: 248.3, secondary_wavelength_nm: 372.0,
            characteristic_conc_ug_ml: 0.1, slit_width_nm: 0.2,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 7.0,
            detection_limit_ug_ml: 0.005, linear_range_max_ug_ml: 5.0,
        },
        ElementEntry {
            symbol: "K", name: "Potassium",
            primary_wavelength_nm: 766.5, secondary_wavelength_nm: 769.9,
            characteristic_conc_ug_ml: 0.04, slit_width_nm: 1.0,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 6.0,
            detection_limit_ug_ml: 0.002, linear_range_max_ug_ml: 2.0,
        },
        ElementEntry {
            symbol: "Li", name: "Lithium",
            primary_wavelength_nm: 670.8, secondary_wavelength_nm: 323.3,
            characteristic_conc_ug_ml: 0.04, slit_width_nm: 1.0,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 5.0,
            detection_limit_ug_ml: 0.001, linear_range_max_ug_ml: 2.0,
        },
        ElementEntry {
            symbol: "Mg", name: "Magnesium",
            primary_wavelength_nm: 285.2, secondary_wavelength_nm: 202.6,
            characteristic_conc_ug_ml: 0.004, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 4.0,
            detection_limit_ug_ml: 0.0002, linear_range_max_ug_ml: 0.5,
        },
        ElementEntry {
            symbol: "Mn", name: "Manganese",
            primary_wavelength_nm: 279.5, secondary_wavelength_nm: 403.1,
            characteristic_conc_ug_ml: 0.05, slit_width_nm: 0.2,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 5.0,
            detection_limit_ug_ml: 0.002, linear_range_max_ug_ml: 3.0,
        },
        ElementEntry {
            symbol: "Mo", name: "Molybdenum",
            primary_wavelength_nm: 313.3, secondary_wavelength_nm: 317.0,
            characteristic_conc_ug_ml: 0.5, slit_width_nm: 0.5,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 7.0,
            detection_limit_ug_ml: 0.03, linear_range_max_ug_ml: 100.0,
        },
        ElementEntry {
            symbol: "Na", name: "Sodium",
            primary_wavelength_nm: 589.0, secondary_wavelength_nm: 589.6,
            characteristic_conc_ug_ml: 0.01, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 5.0,
            detection_limit_ug_ml: 0.001, linear_range_max_ug_ml: 1.0,
        },
        ElementEntry {
            symbol: "Ni", name: "Nickel",
            primary_wavelength_nm: 232.0, secondary_wavelength_nm: 341.5,
            characteristic_conc_ug_ml: 0.1, slit_width_nm: 0.2,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 7.0,
            detection_limit_ug_ml: 0.005, linear_range_max_ug_ml: 5.0,
        },
        ElementEntry {
            symbol: "Pb", name: "Lead",
            primary_wavelength_nm: 217.0, secondary_wavelength_nm: 283.3,
            characteristic_conc_ug_ml: 0.2, slit_width_nm: 1.0,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 5.0,
            detection_limit_ug_ml: 0.01, linear_range_max_ug_ml: 20.0,
        },
        ElementEntry {
            symbol: "Pt", name: "Platinum",
            primary_wavelength_nm: 265.9, secondary_wavelength_nm: 306.5,
            characteristic_conc_ug_ml: 2.0, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.05, linear_range_max_ug_ml: 100.0,
        },
        ElementEntry {
            symbol: "Se", name: "Selenium",
            primary_wavelength_nm: 196.0, secondary_wavelength_nm: 204.0,
            characteristic_conc_ug_ml: 0.5, slit_width_nm: 2.0,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.5, linear_range_max_ug_ml: 30.0,
        },
        ElementEntry {
            symbol: "Si", name: "Silicon",
            primary_wavelength_nm: 251.6, secondary_wavelength_nm: 288.2,
            characteristic_conc_ug_ml: 2.5, slit_width_nm: 0.2,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.1, linear_range_max_ug_ml: 150.0,
        },
        ElementEntry {
            symbol: "Sn", name: "Tin",
            primary_wavelength_nm: 224.6, secondary_wavelength_nm: 286.3,
            characteristic_conc_ug_ml: 1.5, slit_width_nm: 0.5,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 8.0,
            detection_limit_ug_ml: 0.1, linear_range_max_ug_ml: 100.0,
        },
        ElementEntry {
            symbol: "Ti", name: "Titanium",
            primary_wavelength_nm: 364.3, secondary_wavelength_nm: 399.0,
            characteristic_conc_ug_ml: 2.0, slit_width_nm: 0.2,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.1, linear_range_max_ug_ml: 100.0,
        },
        ElementEntry {
            symbol: "V", name: "Vanadium",
            primary_wavelength_nm: 318.5, secondary_wavelength_nm: 306.6,
            characteristic_conc_ug_ml: 1.5, slit_width_nm: 0.2,
            flame_type: FlameType::NitrousOxideAcetylene, lamp_current_ma: 10.0,
            detection_limit_ug_ml: 0.05, linear_range_max_ug_ml: 100.0,
        },
        ElementEntry {
            symbol: "Zn", name: "Zinc",
            primary_wavelength_nm: 213.9, secondary_wavelength_nm: 307.6,
            characteristic_conc_ug_ml: 0.01, slit_width_nm: 0.5,
            flame_type: FlameType::AirAcetylene, lamp_current_ma: 5.0,
            detection_limit_ug_ml: 0.001, linear_range_max_ug_ml: 1.0,
        },
    ]
}

/// Look up an element by symbol from the wavelength database.
pub fn lookup_element(symbol: &str) -> Option<ElementEntry> {
    wavelength_database().into_iter().find(|e| e.symbol == symbol)
}

// ============================================================================
// FaasSpectrum
// ============================================================================

/// Container for FAAS spectral/signal data.
#[derive(Debug, Clone)]
pub struct FaasSpectrum {
    /// Wavelength points in nm
    pub wavelengths_nm: Vec<f64>,
    /// Measured absorbances at each wavelength point
    pub absorbances: Vec<f64>,
    /// Blank (reagent blank) absorbances for baseline correction
    pub blank_absorbances: Vec<f64>,
}

impl FaasSpectrum {
    /// Create a new spectrum container.
    pub fn new(
        wavelengths_nm: Vec<f64>,
        absorbances: Vec<f64>,
        blank_absorbances: Vec<f64>,
    ) -> Self {
        Self {
            wavelengths_nm,
            absorbances,
            blank_absorbances,
        }
    }

    /// Compute blank-corrected absorbances.
    pub fn corrected_absorbances(&self) -> Vec<f64> {
        self.absorbances
            .iter()
            .zip(self.blank_absorbances.iter())
            .map(|(a, b)| a - b)
            .collect()
    }

    /// Mean absorbance of the corrected spectrum.
    pub fn mean_corrected_absorbance(&self) -> f64 {
        let corrected = self.corrected_absorbances();
        if corrected.is_empty() {
            return 0.0;
        }
        corrected.iter().sum::<f64>() / corrected.len() as f64
    }

    /// Standard deviation of blank absorbances (used for detection limit).
    pub fn blank_std_dev(&self) -> f64 {
        let n = self.blank_absorbances.len();
        if n < 2 {
            return 0.0;
        }
        let mean = self.blank_absorbances.iter().sum::<f64>() / n as f64;
        let var = self.blank_absorbances.iter().map(|v| (v - mean).powi(2)).sum::<f64>()
            / (n - 1) as f64;
        var.sqrt()
    }
}

// ============================================================================
// Calibration result types
// ============================================================================

/// Linear calibration result: A = intercept + slope * C
#[derive(Debug, Clone)]
pub struct LinearCalibration {
    /// y-intercept (absorbance at zero concentration)
    pub intercept: f64,
    /// slope (sensitivity = dA/dC)
    pub slope: f64,
    /// Coefficient of determination (R^2)
    pub r_squared: f64,
}

/// Quadratic calibration result: A = a + b*C + c*C^2
#[derive(Debug, Clone)]
pub struct QuadraticCalibration {
    /// Constant term
    pub a: f64,
    /// Linear coefficient
    pub b: f64,
    /// Quadratic coefficient
    pub c: f64,
    /// Coefficient of determination (R^2)
    pub r_squared: f64,
}

/// Standard addition result
#[derive(Debug, Clone)]
pub struct StandardAdditionResult {
    /// Computed sample concentration (ug/mL)
    pub concentration: f64,
    /// Slope of the addition line (sensitivity)
    pub slope: f64,
    /// y-intercept of the addition line
    pub intercept: f64,
    /// R-squared of the fit
    pub r_squared: f64,
}

// ============================================================================
// FaasConfig
// ============================================================================

/// Configuration for a FAAS measurement.
#[derive(Debug, Clone)]
pub struct FaasConfig {
    /// Analytical wavelength in nm
    pub wavelength_nm: f64,
    /// Spectral slit width in nm
    pub slit_width_nm: f64,
    /// Hollow cathode lamp current in mA
    pub lamp_current_ma: f64,
    /// Flame type
    pub flame_type: FlameType,
    /// Nebulizer uptake flow rate in mL/min
    pub nebulizer_flow_ml_min: f64,
    /// Burner observation height in mm above burner slot
    pub burner_height_mm: f64,
    /// Signal integration time in seconds
    pub integration_time_s: f64,
}

impl FaasConfig {
    /// Create a default configuration for a given element.
    pub fn for_element(entry: &ElementEntry) -> Self {
        Self {
            wavelength_nm: entry.primary_wavelength_nm,
            slit_width_nm: entry.slit_width_nm,
            lamp_current_ma: entry.lamp_current_ma,
            flame_type: entry.flame_type,
            nebulizer_flow_ml_min: 5.0,
            burner_height_mm: 7.0,
            integration_time_s: 3.0,
        }
    }

    /// Create a custom configuration.
    pub fn new(
        wavelength_nm: f64,
        slit_width_nm: f64,
        lamp_current_ma: f64,
        flame_type: FlameType,
        nebulizer_flow_ml_min: f64,
        burner_height_mm: f64,
        integration_time_s: f64,
    ) -> Self {
        Self {
            wavelength_nm,
            slit_width_nm,
            lamp_current_ma,
            flame_type,
            nebulizer_flow_ml_min,
            burner_height_mm,
            integration_time_s,
        }
    }
}

// ============================================================================
// FaasProcessor
// ============================================================================

/// Core processor for FAAS signal analysis.
#[derive(Debug, Clone)]
pub struct FaasProcessor {
    /// Instrument configuration
    pub config: FaasConfig,
}

impl FaasProcessor {
    /// Create a new FAAS processor with the given configuration.
    pub fn new(config: FaasConfig) -> Self {
        Self { config }
    }

    // ── Beer-Lambert conversions ─────────────────────────────────────────

    /// Convert transmittance to absorbance via Beer-Lambert law.
    ///
    /// A = -log10(I/I0)
    ///
    /// where `i` is transmitted intensity and `i0` is reference intensity.
    pub fn absorbance_from_transmittance(&self, i: f64, i0: f64) -> f64 {
        if i0 <= 0.0 || i <= 0.0 {
            return f64::INFINITY;
        }
        -(i / i0).log10()
    }

    /// Convert absorbance to transmittance ratio.
    ///
    /// T = 10^(-A), returns value in [0, 1].
    pub fn transmittance_from_absorbance(&self, absorbance: f64) -> f64 {
        10.0_f64.powf(-absorbance)
    }

    /// Compute concentration from absorbance using a linear calibration.
    ///
    /// C = (A - intercept) / slope
    pub fn concentration_from_absorbance(
        &self,
        absorbance: f64,
        cal: &LinearCalibration,
    ) -> f64 {
        if cal.slope.abs() < 1e-30 {
            return 0.0;
        }
        (absorbance - cal.intercept) / cal.slope
    }

    /// Compute concentration from absorbance using a quadratic calibration.
    ///
    /// Solves A = a + b*C + c*C^2 for C using the quadratic formula.
    /// Returns the positive root.
    pub fn concentration_from_absorbance_quadratic(
        &self,
        absorbance: f64,
        cal: &QuadraticCalibration,
    ) -> f64 {
        // Rearrange: c*C^2 + b*C + (a - A) = 0
        let a_coeff = cal.c;
        let b_coeff = cal.b;
        let c_coeff = cal.a - absorbance;

        if a_coeff.abs() < 1e-30 {
            // Degenerate to linear
            if b_coeff.abs() < 1e-30 {
                return 0.0;
            }
            return -c_coeff / b_coeff;
        }

        let discriminant = b_coeff * b_coeff - 4.0 * a_coeff * c_coeff;
        if discriminant < 0.0 {
            return 0.0;
        }

        let sqrt_d = discriminant.sqrt();
        let root1 = (-b_coeff + sqrt_d) / (2.0 * a_coeff);
        let root2 = (-b_coeff - sqrt_d) / (2.0 * a_coeff);

        // Return the positive root, preferring the smaller positive one
        if root1 >= 0.0 && root2 >= 0.0 {
            root1.min(root2)
        } else if root1 >= 0.0 {
            root1
        } else if root2 >= 0.0 {
            root2
        } else {
            0.0
        }
    }

    // ── Calibration ─────────────────────────────────────────────────────

    /// Perform least-squares linear calibration: A = intercept + slope * C.
    ///
    /// `concentrations` and `absorbances` must have the same length (>=2).
    /// Returns `None` if insufficient data or degenerate.
    pub fn calibrate_linear(
        &self,
        concentrations: &[f64],
        absorbances: &[f64],
    ) -> Option<LinearCalibration> {
        let n = concentrations.len();
        if n < 2 || n != absorbances.len() {
            return None;
        }
        let nf = n as f64;

        let sum_x: f64 = concentrations.iter().sum();
        let sum_y: f64 = absorbances.iter().sum();
        let sum_xy: f64 = concentrations.iter().zip(absorbances.iter()).map(|(x, y)| x * y).sum();
        let sum_x2: f64 = concentrations.iter().map(|x| x * x).sum();

        let denom = nf * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (nf * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / nf;

        // R-squared
        let y_mean = sum_y / nf;
        let ss_tot: f64 = absorbances.iter().map(|y| (y - y_mean).powi(2)).sum();
        let ss_res: f64 = concentrations
            .iter()
            .zip(absorbances.iter())
            .map(|(x, y)| {
                let y_pred = intercept + slope * x;
                (y - y_pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot.abs() < 1e-30 {
            1.0
        } else {
            1.0 - ss_res / ss_tot
        };

        Some(LinearCalibration {
            intercept,
            slope,
            r_squared,
        })
    }

    /// Perform least-squares quadratic calibration: A = a + b*C + c*C^2.
    ///
    /// `concentrations` and `absorbances` must have the same length (>=3).
    /// Returns `None` if insufficient data or singular matrix.
    pub fn calibrate_quadratic(
        &self,
        concentrations: &[f64],
        absorbances: &[f64],
    ) -> Option<QuadraticCalibration> {
        let n = concentrations.len();
        if n < 3 || n != absorbances.len() {
            return None;
        }

        // Normal equations for A = a + b*x + c*x^2
        // [n,    Sx,   Sx2 ] [a]   [Sy  ]
        // [Sx,   Sx2,  Sx3 ] [b] = [Sxy ]
        // [Sx2,  Sx3,  Sx4 ] [c]   [Sx2y]
        let nf = n as f64;
        let sx: f64 = concentrations.iter().sum();
        let sx2: f64 = concentrations.iter().map(|x| x.powi(2)).sum();
        let sx3: f64 = concentrations.iter().map(|x| x.powi(3)).sum();
        let sx4: f64 = concentrations.iter().map(|x| x.powi(4)).sum();
        let sy: f64 = absorbances.iter().sum();
        let sxy: f64 = concentrations.iter().zip(absorbances.iter()).map(|(x, y)| x * y).sum();
        let sx2y: f64 = concentrations
            .iter()
            .zip(absorbances.iter())
            .map(|(x, y)| x * x * y)
            .sum();

        // Solve 3x3 system via Cramer's rule
        let m = [
            [nf, sx, sx2],
            [sx, sx2, sx3],
            [sx2, sx3, sx4],
        ];
        let det = det3x3(&m);
        if det.abs() < 1e-30 {
            return None;
        }

        let rhs = [sy, sxy, sx2y];

        let m_a = [
            [rhs[0], m[0][1], m[0][2]],
            [rhs[1], m[1][1], m[1][2]],
            [rhs[2], m[2][1], m[2][2]],
        ];
        let m_b = [
            [m[0][0], rhs[0], m[0][2]],
            [m[1][0], rhs[1], m[1][2]],
            [m[2][0], rhs[2], m[2][2]],
        ];
        let m_c = [
            [m[0][0], m[0][1], rhs[0]],
            [m[1][0], m[1][1], rhs[1]],
            [m[2][0], m[2][1], rhs[2]],
        ];

        let a = det3x3(&m_a) / det;
        let b = det3x3(&m_b) / det;
        let c = det3x3(&m_c) / det;

        // R-squared
        let y_mean = sy / nf;
        let ss_tot: f64 = absorbances.iter().map(|y| (y - y_mean).powi(2)).sum();
        let ss_res: f64 = concentrations
            .iter()
            .zip(absorbances.iter())
            .map(|(x, y)| {
                let y_pred = a + b * x + c * x * x;
                (y - y_pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot.abs() < 1e-30 {
            1.0
        } else {
            1.0 - ss_res / ss_tot
        };

        Some(QuadraticCalibration {
            a,
            b,
            c,
            r_squared,
        })
    }

    /// Method of standard additions.
    ///
    /// The sample is spiked with known concentrations of analyte. The
    /// x-intercept of the line A vs added_concentration gives the original
    /// concentration (negated).
    ///
    /// `added_concentrations`: [0.0, c1, c2, ...] (the first is the unspiked sample)
    /// `absorbances`: corresponding measured absorbances
    pub fn method_of_additions(
        &self,
        added_concentrations: &[f64],
        absorbances: &[f64],
    ) -> Option<StandardAdditionResult> {
        let cal = self.calibrate_linear(added_concentrations, absorbances)?;
        if cal.slope.abs() < 1e-30 {
            return None;
        }

        // x-intercept: 0 = intercept + slope * x  =>  x = -intercept / slope
        let concentration = -cal.intercept / cal.slope;
        // The sample concentration is the absolute value of the x-intercept
        let concentration = concentration.abs();

        Some(StandardAdditionResult {
            concentration,
            slope: cal.slope,
            intercept: cal.intercept,
            r_squared: cal.r_squared,
        })
    }

    // ── Performance metrics ─────────────────────────────────────────────

    /// Characteristic concentration: the analyte concentration (ug/mL)
    /// that produces 0.0044 absorbance (1% absorption).
    ///
    /// CC = C * 0.0044 / A
    pub fn characteristic_concentration(&self, concentration: f64, absorbance: f64) -> f64 {
        if absorbance.abs() < 1e-30 {
            return f64::INFINITY;
        }
        concentration * CHARACTERISTIC_ABS / absorbance
    }

    /// Characteristic mass for GFAAS: mass (pg) that produces 0.0044
    /// absorbance-seconds of integrated peak area.
    ///
    /// CM = mass * 0.0044 / peak_area
    pub fn characteristic_mass(&self, mass_pg: f64, peak_area_abs_s: f64) -> f64 {
        if peak_area_abs_s.abs() < 1e-30 {
            return f64::INFINITY;
        }
        mass_pg * CHARACTERISTIC_ABS / peak_area_abs_s
    }

    /// Detection limit: 3 * sigma_blank / sensitivity.
    ///
    /// `blank_std_dev`: standard deviation of replicate blank readings
    /// `sensitivity`: calibration slope (dA/dC)
    pub fn detection_limit(&self, blank_std_dev: f64, sensitivity: f64) -> f64 {
        if sensitivity.abs() < 1e-30 {
            return f64::INFINITY;
        }
        3.0 * blank_std_dev / sensitivity
    }

    /// Sensitivity check: verify that the instrument's measured
    /// characteristic concentration meets the specification.
    ///
    /// Returns `true` if measured CC is within `tolerance_factor` of the
    /// expected CC (e.g., tolerance_factor = 2.0 means within 2x).
    pub fn sensitivity_check(
        &self,
        measured_conc: f64,
        measured_abs: f64,
        expected_cc: f64,
        tolerance_factor: f64,
    ) -> bool {
        let cc = self.characteristic_concentration(measured_conc, measured_abs);
        cc <= expected_cc * tolerance_factor
    }

    // ── Background correction ───────────────────────────────────────────

    /// D2 (deuterium) lamp continuum-source background correction.
    ///
    /// The HCL measures atomic + background absorption; the D2 lamp
    /// (broad continuum) measures only background. Corrected absorbance
    /// is the difference.
    ///
    /// `abs_hcl`: absorbance measured with HCL (atomic + background)
    /// `abs_d2`: absorbance measured with D2 continuum lamp (background only)
    pub fn background_correction_d2(&self, abs_hcl: f64, abs_d2: f64) -> f64 {
        abs_hcl - abs_d2
    }

    /// Zeeman effect background correction model.
    ///
    /// In longitudinal Zeeman-effect AAS, a magnetic field splits the
    /// absorption line into sigma+ and sigma- components. The pi component
    /// disappears. Background absorption is measured in the presence of
    /// the field (shifted atomic line no longer absorbs at analytical
    /// wavelength), while total absorption (atomic + background) is
    /// measured with no field.
    ///
    /// `abs_no_field`: absorbance with no magnetic field (atomic + BG)
    /// `abs_with_field`: absorbance with field on (BG only, atomic line shifted)
    /// `rollover_correction`: for high absorbances, Zeeman can exhibit
    ///   characteristic rollover. This factor (0..1) attenuates the
    ///   apparent BG correction at high A to account for incomplete sigma
    ///   component separation. Typical value 1.0 for A < 0.8.
    pub fn background_correction_zeeman(
        &self,
        abs_no_field: f64,
        abs_with_field: f64,
        rollover_correction: f64,
    ) -> f64 {
        let bg = abs_with_field * rollover_correction;
        let corrected = abs_no_field - bg;
        if corrected < 0.0 { 0.0 } else { corrected }
    }

    /// Smith-Hieftje background correction using pulsed hollow cathode
    /// lamp self-reversal.
    ///
    /// At low lamp current, a narrow emission line is produced (measures
    /// atomic + background). At high current, the emission profile
    /// broadens and self-reverses (hollow center), so atomic absorption
    /// is negligible (measures background only).
    ///
    /// `abs_low_current`: absorbance at normal (low) lamp current
    /// `abs_high_current`: absorbance at high lamp current (self-reversed)
    pub fn smith_hieftje_correction(
        &self,
        abs_low_current: f64,
        abs_high_current: f64,
    ) -> f64 {
        let corrected = abs_low_current - abs_high_current;
        if corrected < 0.0 { 0.0 } else { corrected }
    }

    // ── Interference models ─────────────────────────────────────────────

    /// Model ionization interference for easily ionized elements.
    ///
    /// In the flame, atoms of low ionization energy (Na, K, Ca, etc.) can
    /// become ionized, reducing the ground-state atom population available
    /// for absorption. Adding an ionization suppressor (e.g., KCl for Ca)
    /// provides free electrons that suppress analyte ionization.
    ///
    /// Uses the Saha equation approximation:
    ///   n_ion * n_e / n_atom = K_saha
    /// where K_saha depends on temperature and ionization energy.
    ///
    /// Returns the fraction of atoms remaining neutral (0..1).
    ///
    /// `ionization_energy_ev`: first ionization energy of the analyte
    /// `temperature_k`: flame temperature
    /// `electron_density`: free electron density in cm^-3 (from suppressant)
    ///   Typical: 1e10 (no suppressant) to 1e13 (with KCl suppressant)
    pub fn ionization_interference(
        &self,
        ionization_energy_ev: f64,
        temperature_k: f64,
        electron_density: f64,
    ) -> f64 {
        // Simplified Saha equation: K = (2 * g_ion / g_atom) * (2*pi*m_e*k*T/h^2)^1.5 * exp(-Ei/kT)
        // For simplicity, use partition function ratio ~ 1, and g_ion/g_atom ~ 1
        let kt = K_BOLTZMANN_EV * temperature_k;
        if kt < 1e-30 {
            return 1.0;
        }

        let thermal_factor = (2.0 * std::f64::consts::PI * 9.109e-31 * K_BOLTZMANN_J * temperature_k
            / (H_PLANCK * H_PLANCK))
            .powf(1.5);
        let k_saha = 2.0 * thermal_factor * (-ionization_energy_ev / kt).exp();

        // Convert k_saha from m^-3 to cm^-3
        let k_saha_cm3 = k_saha * 1e-6;

        // n_ion * n_e / n_atom = K
        // If total = n_atom + n_ion, and ionization fraction alpha = n_ion / total:
        //   alpha * n_e / (1 - alpha) = K
        //   alpha = K / (K + n_e)
        let alpha = k_saha_cm3 / (k_saha_cm3 + electron_density);

        // Fraction remaining neutral
        1.0 - alpha
    }

    /// Model chemical interference from refractory compound formation.
    ///
    /// Example: Ca + PO4 -> Ca3(PO4)2 in air-acetylene flame, reducing
    /// free Ca atoms. Adding a releasing agent (La or Sr) or using N2O
    /// flame overcomes this.
    ///
    /// Returns the fraction of free analyte atoms after chemical binding.
    ///
    /// `interferent_conc`: interferent concentration (e.g., PO4 in mg/L)
    /// `analyte_conc`: analyte concentration (e.g., Ca in mg/L)
    /// `binding_constant`: formation constant for the refractory compound
    ///   (dimensionless, relative). Higher = more interference.
    ///   Typical: 0.01 (weak) to 10.0 (strong like Ca-PO4 in air-C2H2)
    /// `releasing_agent_factor`: 0..1 suppression of the interference
    ///   (0 = no releasing agent, 1 = complete release). La at 1% ~ 0.9
    pub fn chemical_interference(
        &self,
        interferent_conc: f64,
        analyte_conc: f64,
        binding_constant: f64,
        releasing_agent_factor: f64,
    ) -> f64 {
        if analyte_conc <= 0.0 {
            return 0.0;
        }
        // Fraction bound to interferent (Langmuir-type model)
        let ratio = interferent_conc / analyte_conc;
        let effective_k = binding_constant * (1.0 - releasing_agent_factor);
        let fraction_bound = effective_k * ratio / (1.0 + effective_k * ratio);
        1.0 - fraction_bound
    }

    // ── Nebulizer and flame ─────────────────────────────────────────────

    /// Calculate nebulizer efficiency (aspiration + transport efficiency).
    ///
    /// The total efficiency is the fraction of aspirated analyte that
    /// reaches the flame as aerosol droplets small enough to be atomized.
    ///
    /// `uptake_rate_ml_min`: liquid uptake rate
    /// `drain_rate_ml_min`: liquid lost to drain/waste
    /// `droplet_fraction_small`: fraction of aerosol droplets < 10 um
    ///   (only small droplets reach the flame). Typical: 0.05-0.15
    pub fn nebulizer_efficiency(
        &self,
        uptake_rate_ml_min: f64,
        drain_rate_ml_min: f64,
        droplet_fraction_small: f64,
    ) -> f64 {
        if uptake_rate_ml_min <= 0.0 {
            return 0.0;
        }
        let transport_eff = (uptake_rate_ml_min - drain_rate_ml_min) / uptake_rate_ml_min;
        let eff = transport_eff * droplet_fraction_small;
        eff.clamp(0.0, 1.0)
    }

    /// Calculate the approximate flame atomization temperature for a
    /// given fuel/oxidant flow ratio.
    ///
    /// For air-acetylene: stoichiometric at ratio ~1.0, fuel-rich < 1,
    /// fuel-lean > 1. Peak temperature at slightly lean.
    ///
    /// `fuel_flow`: fuel flow rate (arbitrary units)
    /// `oxidant_flow`: oxidant flow rate (same units)
    ///
    /// Returns approximate temperature in Kelvin.
    pub fn flame_atomization_temperature(
        &self,
        fuel_flow: f64,
        oxidant_flow: f64,
    ) -> f64 {
        if fuel_flow <= 0.0 || oxidant_flow <= 0.0 {
            return 300.0; // room temperature if no flame
        }
        let base_temp = self.config.flame_type.temperature_k();
        let ratio = fuel_flow / oxidant_flow;

        // Model: temperature peaks near stoichiometric (ratio ~ 0.5 for
        // air-C2H2, ~0.3 for N2O-C2H2). Use Gaussian profile around peak.
        let stoich = match self.config.flame_type {
            FlameType::AirAcetylene => 0.5,
            FlameType::NitrousOxideAcetylene => 0.33,
        };
        let deviation = (ratio - stoich) / stoich;
        // Temperature drops ~300K at 50% deviation from stoichiometric
        let temp_drop = 600.0 * deviation * deviation;
        (base_temp - temp_drop).max(1500.0)
    }

    // ── Signal processing ───────────────────────────────────────────────

    /// Apply boxcar (moving average) smoothing to absorbance signal.
    ///
    /// `window_size`: number of points in the smoothing window (must be odd).
    pub fn signal_smoothing_boxcar(&self, signal: &[f64], window_size: usize) -> Vec<f64> {
        if signal.is_empty() || window_size == 0 {
            return signal.to_vec();
        }
        let w = if window_size % 2 == 0 {
            window_size + 1
        } else {
            window_size
        };
        let half = w / 2;
        let n = signal.len();
        let mut smoothed = Vec::with_capacity(n);

        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = (i + half + 1).min(n);
            let count = (end - start) as f64;
            let sum: f64 = signal[start..end].iter().sum();
            smoothed.push(sum / count);
        }
        smoothed
    }

    /// Apply exponential smoothing to absorbance signal.
    ///
    /// `alpha`: smoothing factor (0..1). Smaller = smoother.
    pub fn signal_smoothing_exponential(&self, signal: &[f64], alpha: f64) -> Vec<f64> {
        if signal.is_empty() {
            return vec![];
        }
        let a = alpha.clamp(0.001, 1.0);
        let mut smoothed = Vec::with_capacity(signal.len());
        smoothed.push(signal[0]);

        for i in 1..signal.len() {
            let prev = smoothed[i - 1];
            smoothed.push(a * signal[i] + (1.0 - a) * prev);
        }
        smoothed
    }

    /// Integrate peak area for GFAAS transient signals.
    ///
    /// Uses trapezoidal integration of the absorbance-time profile.
    /// The signal is baseline-corrected by subtracting the average of
    /// the first `baseline_points` samples.
    ///
    /// `absorbances`: time series of absorbance readings
    /// `time_step_s`: time between samples in seconds
    /// `baseline_points`: number of initial points for baseline estimation
    pub fn peak_area_integration(
        &self,
        absorbances: &[f64],
        time_step_s: f64,
        baseline_points: usize,
    ) -> f64 {
        if absorbances.is_empty() || time_step_s <= 0.0 {
            return 0.0;
        }
        let bp = baseline_points.min(absorbances.len());
        let baseline = if bp > 0 {
            absorbances[..bp].iter().sum::<f64>() / bp as f64
        } else {
            0.0
        };

        // Trapezoidal integration of baseline-corrected signal
        let mut area = 0.0;
        for i in 1..absorbances.len() {
            let y0 = (absorbances[i - 1] - baseline).max(0.0);
            let y1 = (absorbances[i] - baseline).max(0.0);
            area += 0.5 * (y0 + y1) * time_step_s;
        }
        area
    }

    // ── Lamp and QC checks ──────────────────────────────────────────────

    /// Hollow cathode lamp energy and noise check.
    ///
    /// Evaluates the lamp by computing the signal-to-noise ratio from
    /// replicate intensity readings. A healthy lamp should have
    /// SNR > 200 and relative noise < 0.5%.
    ///
    /// Returns `(mean_intensity, snr, relative_noise_pct, pass)`.
    pub fn hollow_cathode_lamp_check(
        &self,
        intensities: &[f64],
        min_snr: f64,
    ) -> (f64, f64, f64, bool) {
        if intensities.len() < 2 {
            return (0.0, 0.0, 100.0, false);
        }
        let n = intensities.len() as f64;
        let mean = intensities.iter().sum::<f64>() / n;
        let var = intensities.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0);
        let std_dev = var.sqrt();

        let snr = if std_dev > 0.0 { mean / std_dev } else { f64::INFINITY };
        let rel_noise_pct = if mean > 0.0 {
            100.0 * std_dev / mean
        } else {
            100.0
        };
        let pass = snr >= min_snr;

        (mean, snr, rel_noise_pct, pass)
    }

    /// Quality control check: verify that a check standard measurement
    /// falls within an acceptable tolerance of the expected value.
    ///
    /// `measured`: measured concentration of QC standard
    /// `expected`: certified/expected value
    /// `tolerance_pct`: acceptable tolerance in percent (e.g., 10.0 for 10%)
    ///
    /// Returns `(recovery_pct, pass)`.
    pub fn qc_check(&self, measured: f64, expected: f64, tolerance_pct: f64) -> (f64, bool) {
        if expected.abs() < 1e-30 {
            return (0.0, false);
        }
        let recovery = 100.0 * measured / expected;
        let pass = (recovery - 100.0).abs() <= tolerance_pct;
        (recovery, pass)
    }

    /// Correct a measured concentration for sample dilution.
    ///
    /// `measured_conc`: concentration measured in the diluted solution
    /// `dilution_factor`: total dilution factor (e.g., 10 for 1:10 dilution)
    pub fn dilution_factor_correction(&self, measured_conc: f64, dilution_factor: f64) -> f64 {
        measured_conc * dilution_factor
    }

    /// Model the effect of a matrix modifier in GFAAS.
    ///
    /// Matrix modifiers (e.g., Pd/Mg(NO3)2, NH4H2PO4) stabilize the
    /// analyte, allowing higher pyrolysis temperatures and better
    /// separation from the matrix.
    ///
    /// `analyte_volatilization_temp_c`: analyte appearance temperature
    ///   without modifier
    /// `modifier_stabilization_c`: temperature increase from modifier
    /// `pyrolysis_temp_c`: applied pyrolysis temperature
    ///
    /// Returns the fraction of analyte retained at the pyrolysis temperature.
    /// 1.0 = no loss, 0.0 = complete loss.
    pub fn matrix_modifier_effect(
        &self,
        analyte_volatilization_temp_c: f64,
        modifier_stabilization_c: f64,
        pyrolysis_temp_c: f64,
    ) -> f64 {
        let effective_appearance = analyte_volatilization_temp_c + modifier_stabilization_c;
        if pyrolysis_temp_c >= effective_appearance {
            // Analyte is already volatilizing at pyrolysis temp
            // Model exponential loss above appearance temperature
            let excess = pyrolysis_temp_c - effective_appearance;
            (-excess / 200.0).exp()
        } else {
            // Analyte is stable: no loss
            1.0
        }
    }

    // ── Wavelength database access ──────────────────────────────────────

    /// Return the element database (convenience method).
    pub fn wavelength_database(&self) -> Vec<ElementEntry> {
        wavelength_database()
    }

    /// Look up an element by symbol.
    pub fn lookup_element(&self, symbol: &str) -> Option<ElementEntry> {
        lookup_element(symbol)
    }

    /// Flame atomization temperature for the configured flame type.
    pub fn flame_temperature_k(&self) -> f64 {
        self.config.flame_type.temperature_k()
    }
}

// ============================================================================
// Helper: 3x3 determinant
// ============================================================================

fn det3x3(m: &[[f64; 3]; 3]) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> FaasProcessor {
        let config = FaasConfig::new(
            324.8, // Cu primary
            0.5,
            4.0,
            FlameType::AirAcetylene,
            5.0,
            7.0,
            3.0,
        );
        FaasProcessor::new(config)
    }

    // ── FlameType tests ─────────────────────────────────────────────────

    #[test]
    fn test_flame_type_temperatures() {
        assert!((FlameType::AirAcetylene.temperature_k() - 2573.0).abs() < 1.0);
        assert!((FlameType::NitrousOxideAcetylene.temperature_k() - 3223.0).abs() < 1.0);
    }

    #[test]
    fn test_flame_type_celsius() {
        let t_c = FlameType::AirAcetylene.temperature_c();
        assert!((t_c - 2299.85).abs() < 0.1);
    }

    #[test]
    fn test_flame_type_names() {
        assert_eq!(FlameType::AirAcetylene.name(), "Air-Acetylene");
        assert_eq!(FlameType::NitrousOxideAcetylene.name(), "N2O-Acetylene");
    }

    #[test]
    fn test_n2o_hotter_than_air() {
        assert!(
            FlameType::NitrousOxideAcetylene.temperature_k()
                > FlameType::AirAcetylene.temperature_k()
        );
    }

    // ── Beer-Lambert law tests ──────────────────────────────────────────

    #[test]
    fn test_absorbance_from_transmittance_100pct() {
        let p = default_processor();
        // T = 1.0 => A = 0
        let a = p.absorbance_from_transmittance(1.0, 1.0);
        assert!((a - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_absorbance_from_transmittance_10pct() {
        let p = default_processor();
        // T = 0.1 => A = 1.0
        let a = p.absorbance_from_transmittance(0.1, 1.0);
        assert!((a - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_absorbance_from_transmittance_1pct() {
        let p = default_processor();
        // T = 0.01 => A = 2.0
        let a = p.absorbance_from_transmittance(0.01, 1.0);
        assert!((a - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_absorbance_zero_intensity() {
        let p = default_processor();
        let a = p.absorbance_from_transmittance(0.0, 1.0);
        assert!(a.is_infinite());
    }

    #[test]
    fn test_transmittance_from_absorbance_roundtrip() {
        let p = default_processor();
        for a_val in [0.0, 0.1, 0.5, 1.0, 1.5, 2.0] {
            let t = p.transmittance_from_absorbance(a_val);
            let a_back = p.absorbance_from_transmittance(t, 1.0);
            assert!(
                (a_back - a_val).abs() < 1e-10,
                "Roundtrip failed for A = {a_val}"
            );
        }
    }

    #[test]
    fn test_transmittance_zero_absorbance() {
        let p = default_processor();
        let t = p.transmittance_from_absorbance(0.0);
        assert!((t - 1.0).abs() < 1e-10);
    }

    // ── Linear calibration tests ────────────────────────────────────────

    #[test]
    fn test_calibrate_linear_perfect_fit() {
        let p = default_processor();
        let concs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let abs_vals: Vec<f64> = concs.iter().map(|c| 0.01 + 0.1 * c).collect();
        let cal = p.calibrate_linear(&concs, &abs_vals).unwrap();
        assert!((cal.slope - 0.1).abs() < 1e-10);
        assert!((cal.intercept - 0.01).abs() < 1e-10);
        assert!((cal.r_squared - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_calibrate_linear_insufficient_data() {
        let p = default_processor();
        let result = p.calibrate_linear(&[1.0], &[0.1]);
        assert!(result.is_none());
    }

    #[test]
    fn test_concentration_from_absorbance_linear() {
        let p = default_processor();
        let cal = LinearCalibration {
            intercept: 0.005,
            slope: 0.1,
            r_squared: 0.999,
        };
        // A = 0.505 => C = (0.505 - 0.005) / 0.1 = 5.0
        let conc = p.concentration_from_absorbance(0.505, &cal);
        assert!((conc - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_calibrate_linear_realistic_copper() {
        let p = default_processor();
        // Typical Cu calibration data (ug/mL vs absorbance)
        let concs = vec![0.0, 0.5, 1.0, 2.0, 4.0, 5.0];
        let abs_vals = vec![0.003, 0.050, 0.098, 0.195, 0.388, 0.485];
        let cal = p.calibrate_linear(&concs, &abs_vals).unwrap();
        assert!(cal.r_squared > 0.999, "R^2 = {}", cal.r_squared);
        assert!((cal.slope - 0.0966).abs() < 0.005);
    }

    // ── Quadratic calibration tests ─────────────────────────────────────

    #[test]
    fn test_calibrate_quadratic_perfect() {
        let p = default_processor();
        // A = 0.01 + 0.08*C - 0.002*C^2
        let concs = vec![0.0, 2.0, 5.0, 10.0, 15.0, 20.0];
        let abs_vals: Vec<f64> = concs
            .iter()
            .map(|c| 0.01 + 0.08 * c - 0.002 * c * c)
            .collect();
        let cal = p.calibrate_quadratic(&concs, &abs_vals).unwrap();
        assert!((cal.a - 0.01).abs() < 1e-6);
        assert!((cal.b - 0.08).abs() < 1e-6);
        assert!((cal.c - (-0.002)).abs() < 1e-6);
        assert!((cal.r_squared - 1.0).abs() < 1e-8);
    }

    #[test]
    fn test_calibrate_quadratic_insufficient_data() {
        let p = default_processor();
        let result = p.calibrate_quadratic(&[1.0, 2.0], &[0.1, 0.2]);
        assert!(result.is_none());
    }

    #[test]
    fn test_concentration_from_absorbance_quadratic() {
        let p = default_processor();
        let cal = QuadraticCalibration {
            a: 0.01,
            b: 0.1,
            c: -0.001,
            r_squared: 0.999,
        };
        // For C = 5: A = 0.01 + 0.5 - 0.025 = 0.485
        let target_a = 0.01 + 0.1 * 5.0 - 0.001 * 25.0;
        let conc = p.concentration_from_absorbance_quadratic(target_a, &cal);
        assert!((conc - 5.0).abs() < 0.01, "Got C = {conc}");
    }

    // ── Method of additions tests ───────────────────────────────────────

    #[test]
    fn test_method_of_additions() {
        let p = default_processor();
        // Sample has unknown conc. Spiked at 0, 1, 2, 3 ug/mL.
        // True concentration = 2.0 ug/mL, sensitivity = 0.1 A/(ug/mL)
        let true_conc = 2.0;
        let sensitivity = 0.1;
        let added = vec![0.0, 1.0, 2.0, 3.0];
        let abs_vals: Vec<f64> = added
            .iter()
            .map(|a| sensitivity * (true_conc + a))
            .collect();
        let result = p.method_of_additions(&added, &abs_vals).unwrap();
        assert!(
            (result.concentration - true_conc).abs() < 0.01,
            "Got conc = {}",
            result.concentration
        );
    }

    // ── Characteristic concentration / mass ─────────────────────────────

    #[test]
    fn test_characteristic_concentration() {
        let p = default_processor();
        // If 1 ug/mL gives A = 0.1, CC = 1.0 * 0.0044 / 0.1 = 0.044
        let cc = p.characteristic_concentration(1.0, 0.1);
        assert!((cc - 0.044).abs() < 1e-6);
    }

    #[test]
    fn test_characteristic_concentration_zero_abs() {
        let p = default_processor();
        let cc = p.characteristic_concentration(1.0, 0.0);
        assert!(cc.is_infinite());
    }

    #[test]
    fn test_characteristic_mass() {
        let p = default_processor();
        // 50 pg gives peak area 0.11 A*s: CM = 50 * 0.0044 / 0.11 = 2.0 pg
        let cm = p.characteristic_mass(50.0, 0.11);
        assert!((cm - 2.0).abs() < 0.01);
    }

    // ── Detection limit tests ───────────────────────────────────────────

    #[test]
    fn test_detection_limit() {
        let p = default_processor();
        // sigma_blank = 0.001, sensitivity = 0.1 => DL = 3 * 0.001 / 0.1 = 0.03
        let dl = p.detection_limit(0.001, 0.1);
        assert!((dl - 0.03).abs() < 1e-6);
    }

    #[test]
    fn test_detection_limit_zero_sensitivity() {
        let p = default_processor();
        let dl = p.detection_limit(0.001, 0.0);
        assert!(dl.is_infinite());
    }

    // ── Sensitivity check tests ─────────────────────────────────────────

    #[test]
    fn test_sensitivity_check_pass() {
        let p = default_processor();
        // Cu: expected CC = 0.04 ug/mL. Measured: 1 ug/mL gives A = 0.1
        // => CC = 0.044. Within 2x of 0.04? 0.044 <= 0.08? Yes.
        assert!(p.sensitivity_check(1.0, 0.1, 0.04, 2.0));
    }

    #[test]
    fn test_sensitivity_check_fail() {
        let p = default_processor();
        // Poor sensitivity: 1 ug/mL gives A = 0.01 => CC = 0.44
        // 0.44 <= 0.04 * 2 = 0.08? No.
        assert!(!p.sensitivity_check(1.0, 0.01, 0.04, 2.0));
    }

    // ── Background correction tests ─────────────────────────────────────

    #[test]
    fn test_d2_correction() {
        let p = default_processor();
        // HCL: 0.5 (atomic + BG), D2: 0.15 (BG only)
        let corrected = p.background_correction_d2(0.5, 0.15);
        assert!((corrected - 0.35).abs() < 1e-10);
    }

    #[test]
    fn test_zeeman_correction() {
        let p = default_processor();
        // No field: 0.4, with field: 0.05, rollover = 1.0
        let corrected = p.background_correction_zeeman(0.4, 0.05, 1.0);
        assert!((corrected - 0.35).abs() < 1e-10);
    }

    #[test]
    fn test_zeeman_correction_negative_clamp() {
        let p = default_processor();
        // Background exceeds total (artifact): should clamp to 0
        let corrected = p.background_correction_zeeman(0.1, 0.2, 1.0);
        assert!((corrected - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_smith_hieftje_correction() {
        let p = default_processor();
        let corrected = p.smith_hieftje_correction(0.45, 0.10);
        assert!((corrected - 0.35).abs() < 1e-10);
    }

    // ── Interference tests ──────────────────────────────────────────────

    #[test]
    fn test_ionization_high_electron_density_suppresses() {
        let p = default_processor();
        // Ca: IE ~ 6.11 eV, air-C2H2 ~ 2573 K
        let frac_no_supp = p.ionization_interference(6.11, 2573.0, 1e10);
        let frac_supp = p.ionization_interference(6.11, 2573.0, 1e14);
        // With suppressant, more atoms should be neutral
        assert!(
            frac_supp > frac_no_supp,
            "Suppressed {frac_supp} should > unsuppressed {frac_no_supp}"
        );
    }

    #[test]
    fn test_ionization_high_ie_element() {
        let p = default_processor();
        // Zn IE = 9.39 eV: very little ionization at 2573K
        let frac = p.ionization_interference(9.39, 2573.0, 1e10);
        assert!(frac > 0.99, "Zn should be >99% neutral, got {frac}");
    }

    #[test]
    fn test_chemical_interference_no_interferent() {
        let p = default_processor();
        let frac = p.chemical_interference(0.0, 5.0, 1.0, 0.0);
        assert!((frac - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_chemical_interference_with_releasing_agent() {
        let p = default_processor();
        let frac_no_la = p.chemical_interference(50.0, 5.0, 1.0, 0.0);
        let frac_la = p.chemical_interference(50.0, 5.0, 1.0, 0.9);
        assert!(
            frac_la > frac_no_la,
            "La should improve free atom fraction"
        );
    }

    // ── Nebulizer efficiency tests ──────────────────────────────────────

    #[test]
    fn test_nebulizer_efficiency_typical() {
        let p = default_processor();
        // 5 mL/min uptake, 4.5 mL/min drain, 10% small droplets
        let eff = p.nebulizer_efficiency(5.0, 4.5, 0.10);
        // (5-4.5)/5 * 0.10 = 0.1 * 0.10 = 0.01 = 1%
        assert!((eff - 0.01).abs() < 1e-6);
    }

    #[test]
    fn test_nebulizer_efficiency_zero_uptake() {
        let p = default_processor();
        let eff = p.nebulizer_efficiency(0.0, 0.0, 0.10);
        assert!((eff - 0.0).abs() < 1e-10);
    }

    // ── Flame temperature model tests ───────────────────────────────────

    #[test]
    fn test_flame_temperature_stoichiometric() {
        let p = default_processor();
        // Air-C2H2 stoichiometric at ratio ~0.5
        let t = p.flame_atomization_temperature(0.5, 1.0);
        // Should be near peak temperature
        assert!((t - 2573.0).abs() < 10.0, "T = {t}");
    }

    #[test]
    fn test_flame_temperature_fuel_rich() {
        let p = default_processor();
        // Far from stoichiometric: temperature drops
        let t_stoich = p.flame_atomization_temperature(0.5, 1.0);
        let t_rich = p.flame_atomization_temperature(1.0, 1.0);
        assert!(t_rich < t_stoich, "Fuel-rich should be cooler");
    }

    // ── Signal smoothing tests ──────────────────────────────────────────

    #[test]
    fn test_boxcar_smoothing() {
        let p = default_processor();
        let signal = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let smoothed = p.signal_smoothing_boxcar(&signal, 3);
        assert_eq!(smoothed.len(), 5);
        // Center point: (0 + 1 + 0) / 3 = 0.333...
        assert!((smoothed[2] - 1.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_exponential_smoothing() {
        let p = default_processor();
        let signal = vec![1.0, 1.0, 1.0, 1.0, 1.0];
        let smoothed = p.signal_smoothing_exponential(&signal, 0.5);
        // Constant signal should remain constant
        for v in &smoothed {
            assert!((v - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_exponential_smoothing_step() {
        let p = default_processor();
        let signal = vec![0.0, 0.0, 1.0, 1.0, 1.0];
        let smoothed = p.signal_smoothing_exponential(&signal, 0.5);
        // After step at index 2, output should gradually approach 1.0
        assert!(smoothed[2] > 0.0 && smoothed[2] < 1.0);
        assert!(smoothed[3] > smoothed[2]);
    }

    // ── Peak area integration tests ─────────────────────────────────────

    #[test]
    fn test_peak_area_integration_simple() {
        let p = default_processor();
        // Rectangular peak: 0,0,0.5,0.5,0.5,0,0
        let signal = vec![0.0, 0.0, 0.5, 0.5, 0.5, 0.0, 0.0];
        let area = p.peak_area_integration(&signal, 1.0, 2);
        // After baseline subtraction (baseline ~0), area ~
        // trapezoidal of [0, 0, 0.5, 0.5, 0.5, 0, 0]
        // = 0.5*(0+0) + 0.5*(0+0.5) + 0.5*(0.5+0.5) + 0.5*(0.5+0.5) + 0.5*(0.5+0) + 0.5*(0+0)
        // = 0 + 0.25 + 0.5 + 0.5 + 0.25 + 0 = 1.5
        assert!((area - 1.5).abs() < 0.01, "Area = {area}");
    }

    #[test]
    fn test_peak_area_with_baseline() {
        let p = default_processor();
        // Baseline of 0.02, peak on top
        let signal = vec![0.02, 0.02, 0.02, 0.52, 0.52, 0.02, 0.02];
        let area = p.peak_area_integration(&signal, 1.0, 3);
        // Baseline = 0.02, corrected peak = 0.50
        // Area should be approximately 2 * 0.5 = 1.0 (two points at 0.5)
        assert!(area > 0.5 && area < 1.5, "Area = {area}");
    }

    // ── HCL check tests ─────────────────────────────────────────────────

    #[test]
    fn test_hcl_check_good_lamp() {
        let p = default_processor();
        let readings = vec![1000.0, 1001.0, 999.5, 1000.5, 1000.0];
        let (mean, snr, rel_noise, pass) = p.hollow_cathode_lamp_check(&readings, 200.0);
        assert!((mean - 1000.2).abs() < 0.1);
        assert!(snr > 200.0, "SNR = {snr}");
        assert!(rel_noise < 0.5, "Rel noise = {rel_noise}%");
        assert!(pass);
    }

    #[test]
    fn test_hcl_check_noisy_lamp() {
        let p = default_processor();
        let readings = vec![900.0, 1100.0, 800.0, 1200.0, 950.0];
        let (_, snr, _, pass) = p.hollow_cathode_lamp_check(&readings, 200.0);
        assert!(snr < 200.0, "Noisy lamp SNR should be low");
        assert!(!pass);
    }

    // ── QC check tests ──────────────────────────────────────────────────

    #[test]
    fn test_qc_check_pass() {
        let p = default_processor();
        let (recovery, pass) = p.qc_check(1.02, 1.0, 10.0);
        assert!((recovery - 102.0).abs() < 0.1);
        assert!(pass);
    }

    #[test]
    fn test_qc_check_fail() {
        let p = default_processor();
        let (recovery, pass) = p.qc_check(0.8, 1.0, 10.0);
        assert!((recovery - 80.0).abs() < 0.1);
        assert!(!pass);
    }

    // ── Dilution correction tests ───────────────────────────────────────

    #[test]
    fn test_dilution_correction() {
        let p = default_processor();
        let result = p.dilution_factor_correction(0.5, 10.0);
        assert!((result - 5.0).abs() < 1e-10);
    }

    // ── Matrix modifier effect tests ────────────────────────────────────

    #[test]
    fn test_matrix_modifier_stable() {
        let p = default_processor();
        // Analyte appears at 400C, modifier adds 500C, pyrolysis at 800C
        // effective_appearance = 900C, pyrolysis < 900 => no loss
        let frac = p.matrix_modifier_effect(400.0, 500.0, 800.0);
        assert!((frac - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_matrix_modifier_loss() {
        let p = default_processor();
        // No modifier: analyte at 400C, pyrolysis at 600C => loss
        let frac_no_mod = p.matrix_modifier_effect(400.0, 0.0, 600.0);
        assert!(frac_no_mod < 1.0, "Should have some loss");
        // With modifier: stable
        let frac_mod = p.matrix_modifier_effect(400.0, 500.0, 600.0);
        assert!(
            frac_mod > frac_no_mod,
            "Modifier should reduce analyte loss"
        );
    }

    // ── Element database tests ──────────────────────────────────────────

    #[test]
    fn test_wavelength_database_size() {
        let db = wavelength_database();
        assert!(db.len() >= 25, "Database has {} entries", db.len());
    }

    #[test]
    fn test_lookup_copper() {
        let cu = lookup_element("Cu").unwrap();
        assert_eq!(cu.symbol, "Cu");
        assert!((cu.primary_wavelength_nm - 324.8).abs() < 0.1);
        assert_eq!(cu.flame_type, FlameType::AirAcetylene);
    }

    #[test]
    fn test_lookup_aluminum_needs_n2o() {
        let al = lookup_element("Al").unwrap();
        assert_eq!(al.flame_type, FlameType::NitrousOxideAcetylene);
    }

    #[test]
    fn test_lookup_nonexistent() {
        assert!(lookup_element("Unobtanium").is_none());
    }

    #[test]
    fn test_element_wavelengths_in_range() {
        for entry in wavelength_database() {
            assert!(
                entry.primary_wavelength_nm >= 190.0 && entry.primary_wavelength_nm <= 900.0,
                "{}: primary {} nm out of range",
                entry.symbol,
                entry.primary_wavelength_nm
            );
        }
    }

    // ── FaasSpectrum tests ──────────────────────────────────────────────

    #[test]
    fn test_spectrum_corrected_absorbances() {
        let spectrum = FaasSpectrum::new(
            vec![324.7, 324.8, 324.9],
            vec![0.150, 0.300, 0.155],
            vec![0.010, 0.012, 0.011],
        );
        let corrected = spectrum.corrected_absorbances();
        assert!((corrected[1] - 0.288).abs() < 1e-6);
    }

    #[test]
    fn test_spectrum_mean_corrected() {
        let spectrum = FaasSpectrum::new(
            vec![324.8],
            vec![0.500],
            vec![0.020],
        );
        let mean = spectrum.mean_corrected_absorbance();
        assert!((mean - 0.480).abs() < 1e-6);
    }

    #[test]
    fn test_spectrum_blank_std_dev() {
        let spectrum = FaasSpectrum::new(
            vec![1.0, 2.0, 3.0, 4.0, 5.0],
            vec![0.0; 5],
            vec![0.010, 0.012, 0.009, 0.011, 0.010],
        );
        let sd = spectrum.blank_std_dev();
        // Mean = 0.0104, stdev of [0.010, 0.012, 0.009, 0.011, 0.010]
        assert!(sd > 0.0 && sd < 0.005, "Blank SD = {sd}");
    }

    // ── FaasConfig for element tests ────────────────────────────────────

    #[test]
    fn test_config_for_element() {
        let cu = lookup_element("Cu").unwrap();
        let cfg = FaasConfig::for_element(&cu);
        assert!((cfg.wavelength_nm - 324.8).abs() < 0.1);
        assert!((cfg.slit_width_nm - 0.5).abs() < 0.1);
        assert_eq!(cfg.flame_type, FlameType::AirAcetylene);
    }

    // ── Integration: full analysis workflow ─────────────────────────────

    #[test]
    fn test_full_copper_analysis_workflow() {
        let cu = lookup_element("Cu").unwrap();
        let config = FaasConfig::for_element(&cu);
        let p = FaasProcessor::new(config);

        // 1. Calibrate with standards
        let concs = vec![0.0, 0.5, 1.0, 2.0, 4.0];
        let abs_vals = vec![0.002, 0.048, 0.098, 0.196, 0.390];
        let cal = p.calibrate_linear(&concs, &abs_vals).unwrap();
        assert!(cal.r_squared > 0.999);

        // 2. Measure unknown sample (A = 0.150)
        let sample_conc = p.concentration_from_absorbance(0.150, &cal);
        assert!(sample_conc > 1.0 && sample_conc < 2.0, "Conc = {sample_conc}");

        // 3. Check QC standard (2.0 ug/mL expected)
        let qc_abs = 0.196;
        let qc_conc = p.concentration_from_absorbance(qc_abs, &cal);
        let (recovery, pass) = p.qc_check(qc_conc, 2.0, 10.0);
        assert!(pass, "QC recovery = {recovery}%");

        // 4. Characteristic concentration
        let cc = p.characteristic_concentration(2.0, 0.196);
        assert!(cc < 0.1, "CC = {cc} ug/mL");

        // 5. Detection limit from blank noise
        let dl = p.detection_limit(0.001, cal.slope);
        assert!(dl < 0.1, "DL = {dl} ug/mL");
    }
}
