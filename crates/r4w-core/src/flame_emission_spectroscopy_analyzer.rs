//! # Flame Emission Spectroscopy Analyzer
//!
//! Flame emission (flame photometry) and atomic absorption spectroscopy data analysis
//! for quantitative elemental analysis of alkali/alkaline earth metals, including
//! calibration, interference correction, and multi-element analysis.
//!
//! ## Overview
//!
//! This module implements the complete analytical workflow for flame-based spectrometry:
//!
//! - **Emission Spectrum**: Wavelength vs intensity with element identification
//! - **Calibration Curve**: External standard calibration (linear/quadratic)
//! - **Standard Addition**: Method of standard additions for matrix-matched quantification
//! - **Internal Standard**: Signal ratio correction for drift compensation
//! - **Interference Correction**: Spectral overlap, ionization suppression, matrix effects
//! - **Flame Parameters**: Flame type characteristics and Boltzmann population ratios
//! - **Absorption Mode**: Atomic absorption (Beer-Lambert, characteristic concentration)
//! - **Background Correction**: Deuterium, Zeeman, Smith-Hieftje methods
//! - **Simulator**: Synthetic spectra and calibration data generation
//! - **Multi-Element Analyzer**: Simultaneous multi-element quantification
//!
//! ## Physics
//!
//! Flame emission spectroscopy excites atoms thermally in a flame, causing them to emit
//! photons at element-specific wavelengths. The intensity of emission is proportional to
//! the concentration of the element in the sample.
//!
//! Key equations:
//! - Boltzmann: N_upper/N_lower = (g_u/g_l) * exp(-dE / kT)
//! - Beer-Lambert: A = epsilon * l * c = log10(I0/I)
//! - LOD = 3 * sigma_blank / m (3-sigma criterion)
//! - LOQ = 10 * sigma_blank / m

/// Known element emission line entry.
#[derive(Debug, Clone, PartialEq)]
pub struct ElementLine {
    /// Element symbol (e.g., "Na", "K")
    pub element: String,
    /// Emission wavelength in nm
    pub wavelength_nm: f64,
    /// Relative intensity (0.0 to 1.0)
    pub relative_intensity: f64,
    /// Description of the transition
    pub description: String,
}

/// Emission spectrum: wavelength vs intensity.
#[derive(Debug, Clone)]
pub struct EmissionSpectrum {
    pub wavelength_nm: Vec<f64>,
    pub intensity: Vec<f64>,
}

impl EmissionSpectrum {
    /// Create a new emission spectrum from matched wavelength and intensity vectors.
    pub fn new(wavelength_nm: Vec<f64>, intensity: Vec<f64>) -> Self {
        assert_eq!(
            wavelength_nm.len(),
            intensity.len(),
            "wavelength and intensity must have same length"
        );
        assert!(!wavelength_nm.is_empty(), "spectrum must not be empty");
        Self {
            wavelength_nm,
            intensity,
        }
    }

    /// Wavelength of the strongest emission line.
    pub fn peak_wavelength(&self) -> f64 {
        let mut max_idx = 0;
        let mut max_val = self.intensity[0];
        for (i, &v) in self.intensity.iter().enumerate() {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }
        self.wavelength_nm[max_idx]
    }

    /// Intensity of the strongest emission line.
    pub fn peak_intensity(&self) -> f64 {
        let mut max_val = self.intensity[0];
        for &v in &self.intensity {
            if v > max_val {
                max_val = v;
            }
        }
        max_val
    }

    /// Identify elements matching a given wavelength within tolerance.
    pub fn identify_element(&self, wavelength_nm: f64, tolerance_nm: f64) -> Vec<ElementLine> {
        element_database()
            .into_iter()
            .filter(|el| (el.wavelength_nm - wavelength_nm).abs() <= tolerance_nm)
            .collect()
    }

    /// Find all peaks in the spectrum (local maxima above a threshold fraction of max).
    pub fn find_peaks(&self, threshold_fraction: f64) -> Vec<(f64, f64)> {
        let max_intensity = self.peak_intensity();
        let threshold = max_intensity * threshold_fraction;
        let n = self.intensity.len();
        let mut peaks = Vec::new();

        for i in 1..n.saturating_sub(1) {
            if self.intensity[i] > self.intensity[i - 1]
                && self.intensity[i] > self.intensity[i + 1]
                && self.intensity[i] >= threshold
            {
                peaks.push((self.wavelength_nm[i], self.intensity[i]));
            }
        }
        // Check endpoints
        if n >= 2 && self.intensity[0] > self.intensity[1] && self.intensity[0] >= threshold {
            peaks.insert(0, (self.wavelength_nm[0], self.intensity[0]));
        }
        if n >= 2
            && self.intensity[n - 1] > self.intensity[n - 2]
            && self.intensity[n - 1] >= threshold
        {
            peaks.push((self.wavelength_nm[n - 1], self.intensity[n - 1]));
        }
        peaks
    }

    /// Integrate emission intensity over a wavelength range (trapezoidal rule).
    pub fn integrate(&self, start_nm: f64, end_nm: f64) -> f64 {
        let mut sum = 0.0;
        for i in 1..self.wavelength_nm.len() {
            let w0 = self.wavelength_nm[i - 1];
            let w1 = self.wavelength_nm[i];
            if w1 < start_nm || w0 > end_nm {
                continue;
            }
            let dw = w1 - w0;
            sum += 0.5 * (self.intensity[i - 1] + self.intensity[i]) * dw;
        }
        sum
    }

    /// Signal-to-noise ratio estimate (peak / RMS of baseline region).
    pub fn snr(&self, baseline_start_nm: f64, baseline_end_nm: f64) -> f64 {
        let peak = self.peak_intensity();
        let mut sum_sq = 0.0;
        let mut count = 0usize;
        for (i, &w) in self.wavelength_nm.iter().enumerate() {
            if w >= baseline_start_nm && w <= baseline_end_nm {
                sum_sq += self.intensity[i] * self.intensity[i];
                count += 1;
            }
        }
        if count == 0 {
            return 0.0;
        }
        let rms = (sum_sq / count as f64).sqrt();
        if rms == 0.0 {
            return f64::INFINITY;
        }
        peak / rms
    }
}

/// Element emission line database.
pub fn element_database() -> Vec<ElementLine> {
    vec![
        ElementLine {
            element: "Na".to_string(),
            wavelength_nm: 589.0,
            relative_intensity: 1.0,
            description: "Na D2 line".to_string(),
        },
        ElementLine {
            element: "Na".to_string(),
            wavelength_nm: 589.6,
            relative_intensity: 0.5,
            description: "Na D1 line".to_string(),
        },
        ElementLine {
            element: "K".to_string(),
            wavelength_nm: 766.5,
            relative_intensity: 1.0,
            description: "K primary line".to_string(),
        },
        ElementLine {
            element: "K".to_string(),
            wavelength_nm: 769.9,
            relative_intensity: 0.5,
            description: "K secondary line".to_string(),
        },
        ElementLine {
            element: "Li".to_string(),
            wavelength_nm: 670.8,
            relative_intensity: 1.0,
            description: "Li primary line".to_string(),
        },
        ElementLine {
            element: "Ca".to_string(),
            wavelength_nm: 422.7,
            relative_intensity: 1.0,
            description: "Ca primary line".to_string(),
        },
        ElementLine {
            element: "Ba".to_string(),
            wavelength_nm: 553.6,
            relative_intensity: 1.0,
            description: "Ba primary line".to_string(),
        },
        ElementLine {
            element: "Sr".to_string(),
            wavelength_nm: 460.7,
            relative_intensity: 1.0,
            description: "Sr primary line".to_string(),
        },
        ElementLine {
            element: "Cu".to_string(),
            wavelength_nm: 324.8,
            relative_intensity: 1.0,
            description: "Cu primary line".to_string(),
        },
        ElementLine {
            element: "Cs".to_string(),
            wavelength_nm: 852.1,
            relative_intensity: 1.0,
            description: "Cs primary line".to_string(),
        },
    ]
}

/// Result of a linear calibration fit.
#[derive(Debug, Clone)]
pub struct LinearCalResult {
    /// Slope m in y = mx + b
    pub slope: f64,
    /// Intercept b in y = mx + b
    pub intercept: f64,
    /// Coefficient of determination
    pub r_squared: f64,
}

/// Result of a quadratic calibration fit.
#[derive(Debug, Clone)]
pub struct QuadCalResult {
    /// Coefficient a in y = ax^2 + bx + c
    pub a: f64,
    /// Coefficient b in y = ax^2 + bx + c
    pub b: f64,
    /// Coefficient c in y = ax^2 + bx + c
    pub c: f64,
    /// Coefficient of determination
    pub r_squared: f64,
}

/// External standard calibration curve.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    pub concentrations: Vec<f64>,
    pub signals: Vec<f64>,
}

impl CalibrationCurve {
    /// Create from matched concentration and signal vectors.
    pub fn new(concentrations: Vec<f64>, signals: Vec<f64>) -> Self {
        assert_eq!(
            concentrations.len(),
            signals.len(),
            "concentrations and signals must have same length"
        );
        assert!(
            concentrations.len() >= 2,
            "need at least 2 calibration points"
        );
        Self {
            concentrations,
            signals,
        }
    }

    /// Perform a linear least-squares fit: y = mx + b.
    pub fn linear_fit(&self) -> LinearCalResult {
        let (slope, intercept, r_sq) =
            linear_regression(&self.concentrations, &self.signals);
        LinearCalResult {
            slope,
            intercept,
            r_squared: r_sq,
        }
    }

    /// Perform a quadratic least-squares fit: y = ax^2 + bx + c.
    pub fn quadratic_fit(&self) -> QuadCalResult {
        let n = self.concentrations.len() as f64;
        let x = &self.concentrations;
        let y = &self.signals;

        // Sums for normal equations
        let sx: f64 = x.iter().sum();
        let sx2: f64 = x.iter().map(|v| v * v).sum();
        let sx3: f64 = x.iter().map(|v| v * v * v).sum();
        let sx4: f64 = x.iter().map(|v| v.powi(4)).sum();
        let sy: f64 = y.iter().sum();
        let sxy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
        let sx2y: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * xi * yi).sum();

        // Solve 3x3 system via Cramer's rule:
        // [n    sx   sx2] [c]   [sy  ]
        // [sx   sx2  sx3] [b] = [sxy ]
        // [sx2  sx3  sx4] [a]   [sx2y]
        let det = n * (sx2 * sx4 - sx3 * sx3)
            - sx * (sx * sx4 - sx3 * sx2)
            + sx2 * (sx * sx3 - sx2 * sx2);

        let det_c = sy * (sx2 * sx4 - sx3 * sx3)
            - sx * (sxy * sx4 - sx2y * sx3)
            + sx2 * (sxy * sx3 - sx2y * sx2);

        let det_b = n * (sxy * sx4 - sx2y * sx3)
            - sy * (sx * sx4 - sx3 * sx2)
            + sx2 * (sx * sx2y - sxy * sx2);

        let det_a = n * (sx2 * sx2y - sx3 * sxy)
            - sx * (sx * sx2y - sxy * sx2)
            + sy * (sx * sx3 - sx2 * sx2);

        let c = det_c / det;
        let b = det_b / det;
        let a = det_a / det;

        // R^2 calculation
        let y_mean = sy / n;
        let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean).powi(2)).sum();
        let ss_res: f64 = x
            .iter()
            .zip(y.iter())
            .map(|(xi, yi)| {
                let pred = a * xi * xi + b * xi + c;
                (yi - pred).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            1.0
        };

        QuadCalResult {
            a,
            b,
            c,
            r_squared,
        }
    }

    /// Determine the unknown concentration from a measured signal (using linear fit).
    pub fn unknown_concentration(&self, signal: f64) -> f64 {
        let fit = self.linear_fit();
        if fit.slope.abs() < 1e-30 {
            return 0.0;
        }
        (signal - fit.intercept) / fit.slope
    }

    /// R-squared of the linear fit.
    pub fn r_squared(&self) -> f64 {
        self.linear_fit().r_squared
    }

    /// Sensitivity: slope of the calibration curve at origin.
    pub fn sensitivity(&self) -> f64 {
        self.linear_fit().slope
    }

    /// Limit of detection: LOD = 3 * sigma_blank / m.
    pub fn detection_limit(&self, blank_std: f64) -> f64 {
        let slope = self.sensitivity();
        if slope.abs() < 1e-30 {
            return f64::INFINITY;
        }
        3.0 * blank_std / slope
    }

    /// Limit of quantification: LOQ = 10 * sigma_blank / m.
    pub fn quantification_limit(&self, blank_std: f64) -> f64 {
        let slope = self.sensitivity();
        if slope.abs() < 1e-30 {
            return f64::INFINITY;
        }
        10.0 * blank_std / slope
    }
}

/// Result of standard addition analysis.
#[derive(Debug, Clone)]
pub struct StandardAddResult {
    /// Estimated sample concentration
    pub concentration: f64,
    /// Slope of the additions line
    pub slope: f64,
    /// Y-intercept
    pub intercept: f64,
    /// Coefficient of determination
    pub r_squared: f64,
}

/// Method of standard additions.
#[derive(Debug, Clone)]
pub struct StandardAddition {
    /// Signal from the unspiked sample
    pub sample_signal: f64,
    /// (added_concentration, signal) pairs
    pub additions: Vec<(f64, f64)>,
}

impl StandardAddition {
    /// Create from sample signal and addition pairs.
    pub fn new(sample_signal: f64, additions: Vec<(f64, f64)>) -> Self {
        assert!(
            !additions.is_empty(),
            "need at least one standard addition"
        );
        Self {
            sample_signal,
            additions,
        }
    }

    /// Linear fit with x-intercept extrapolation for sample concentration.
    pub fn fit(&self) -> StandardAddResult {
        // Include the unspiked sample as the (0, sample_signal) point
        let mut x_vals = vec![0.0];
        let mut y_vals = vec![self.sample_signal];
        for &(conc, sig) in &self.additions {
            x_vals.push(conc);
            y_vals.push(sig);
        }

        let (slope, intercept, r_sq) = linear_regression(&x_vals, &y_vals);

        // x-intercept: y = 0 => x = -intercept / slope
        let concentration = if slope.abs() > 1e-30 {
            (intercept / slope).abs()
        } else {
            0.0
        };

        StandardAddResult {
            concentration,
            slope,
            intercept,
            r_squared: r_sq,
        }
    }
}

/// Internal standard correction methods.
pub struct InternalStandard;

impl InternalStandard {
    /// Compute signal ratios: analyte_signal / internal_standard_signal.
    pub fn ratio_method(analyte_signals: &[f64], is_signals: &[f64]) -> Vec<f64> {
        assert_eq!(
            analyte_signals.len(),
            is_signals.len(),
            "signal vectors must match"
        );
        analyte_signals
            .iter()
            .zip(is_signals.iter())
            .map(|(a, is)| {
                if is.abs() < 1e-30 {
                    0.0
                } else {
                    a / is
                }
            })
            .collect()
    }

    /// Calibrate using internal standard: fit ratio vs concentration.
    pub fn calibrate_with_is(
        concs: &[f64],
        analyte: &[f64],
        is: &[f64],
    ) -> LinearCalResult {
        let ratios = Self::ratio_method(analyte, is);
        let (slope, intercept, r_sq) = linear_regression(concs, &ratios);
        LinearCalResult {
            slope,
            intercept,
            r_squared: r_sq,
        }
    }

    /// Determine unknown concentration using internal standard correction.
    pub fn unknown_with_is(
        analyte_signal: f64,
        is_signal: f64,
        cal: &LinearCalResult,
    ) -> f64 {
        let ratio = if is_signal.abs() > 1e-30 {
            analyte_signal / is_signal
        } else {
            0.0
        };
        if cal.slope.abs() < 1e-30 {
            return 0.0;
        }
        (ratio - cal.intercept) / cal.slope
    }
}

/// Interference correction methods.
pub struct InterferenceCorrection;

impl InterferenceCorrection {
    /// Correct for spectral overlap: corrected = signal - (interferent_signal * correction_factor).
    pub fn spectral_overlap_correction(
        signal: f64,
        interferent_signal: f64,
        correction_factor: f64,
    ) -> f64 {
        let corrected = signal - interferent_signal * correction_factor;
        if corrected < 0.0 {
            0.0
        } else {
            corrected
        }
    }

    /// Ionization suppression effect: ratio of signal with suppressant vs without.
    pub fn ionization_suppression(
        signal_without: f64,
        signal_with_suppressant: f64,
    ) -> f64 {
        if signal_without.abs() < 1e-30 {
            return 1.0;
        }
        signal_with_suppressant / signal_without
    }

    /// Suggested matrix-matching additives for a given sample matrix.
    pub fn matrix_matching(sample_matrix: &str) -> Vec<(String, f64)> {
        match sample_matrix {
            "seawater" | "brine" => vec![
                ("NaCl".to_string(), 35.0),      // g/L typical salinity
                ("CsCl".to_string(), 1.0),        // ionization suppressor
                ("LaCl3".to_string(), 0.5),       // releasing agent
            ],
            "blood" | "serum" => vec![
                ("Triton X-100".to_string(), 0.1), // surfactant
                ("HNO3".to_string(), 1.0),         // % v/v acid matrix
                ("CsCl".to_string(), 1.0),         // ionization suppressor
            ],
            "soil" | "geological" => vec![
                ("LaCl3".to_string(), 1.0),        // releasing agent for Ca/Mg
                ("CsCl".to_string(), 2.0),         // ionization suppressor
                ("HCl".to_string(), 5.0),          // acid matrix matching
            ],
            "urine" => vec![
                ("HNO3".to_string(), 0.5),
                ("CsCl".to_string(), 1.0),
            ],
            _ => vec![
                ("CsCl".to_string(), 1.0),         // generic ionization suppressor
                ("LaCl3".to_string(), 0.5),        // generic releasing agent
            ],
        }
    }

    /// Self-absorption correction for high concentration samples.
    /// Uses: corrected_intensity = measured / (1 - ka * concentration)
    /// where ka is the self-absorption coefficient.
    pub fn self_absorption_correction(concentration: f64, ka: f64) -> f64 {
        let denom = 1.0 - ka * concentration;
        if denom <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / denom
    }
}

/// Flame type enumeration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlameType {
    /// Air-acetylene (~2300 K)
    AirAcetylene,
    /// Nitrous oxide-acetylene (~2900 K)
    NitrousOxideAcetylene,
    /// Air-propane (~1900 K)
    AirPropane,
    /// Air-hydrogen (~2100 K)
    AirHydrogen,
}

/// Flame parameters and characteristics.
pub struct FlameParameters;

impl FlameParameters {
    /// Temperature in Kelvin for a given flame type.
    pub fn temperature_k(flame: FlameType) -> f64 {
        match flame {
            FlameType::AirAcetylene => 2300.0,
            FlameType::NitrousOxideAcetylene => 2900.0,
            FlameType::AirPropane => 1900.0,
            FlameType::AirHydrogen => 2100.0,
        }
    }

    /// Elements suitable for analysis in a given flame type.
    pub fn suitable_elements(flame: FlameType) -> Vec<&'static str> {
        match flame {
            FlameType::AirAcetylene => {
                vec!["Na", "K", "Li", "Ca", "Sr", "Ba", "Cu", "Cs", "Rb", "Mg", "Fe", "Mn", "Zn", "Cr"]
            }
            FlameType::NitrousOxideAcetylene => {
                vec![
                    "Al", "Si", "Ti", "V", "Mo", "W", "Zr", "B", "Be",
                    "Na", "K", "Li", "Ca", "Sr", "Ba",
                ]
            }
            FlameType::AirPropane => {
                vec!["Na", "K", "Li", "Cs", "Rb"]
            }
            FlameType::AirHydrogen => {
                vec!["Na", "K", "Li", "Cs", "Sn", "As", "Se"]
            }
        }
    }

    /// Boltzmann population ratio: N_upper / N_lower.
    ///
    /// energy_ev: energy difference in electron-volts
    /// temperature_k: flame temperature in Kelvin
    /// g_upper: statistical weight (degeneracy) of upper level
    /// g_lower: statistical weight (degeneracy) of lower level
    pub fn boltzmann_population(
        energy_ev: f64,
        temperature_k: f64,
        g_upper: f64,
        g_lower: f64,
    ) -> f64 {
        // Boltzmann constant in eV/K
        const KB_EV: f64 = 8.617333262e-5;
        let exponent = -energy_ev / (KB_EV * temperature_k);
        (g_upper / g_lower) * exp_approx(exponent)
    }

    /// Emission intensity relative factor (proportional to population of excited state).
    pub fn emission_intensity_factor(
        energy_ev: f64,
        temperature_k: f64,
        g_upper: f64,
        g_lower: f64,
    ) -> f64 {
        // Intensity ~ N_upper * A (transition probability)
        // Simplified: proportional to Boltzmann factor
        Self::boltzmann_population(energy_ev, temperature_k, g_upper, g_lower)
    }
}

/// Atomic absorption spectroscopy functions.
pub struct AbsorptionMode;

impl AbsorptionMode {
    /// Absorbance: A = log10(I0 / I).
    pub fn absorbance(i0: f64, i_transmitted: f64) -> f64 {
        if i_transmitted <= 0.0 || i0 <= 0.0 {
            return f64::INFINITY;
        }
        log10_approx(i0 / i_transmitted)
    }

    /// Transmittance: T = I / I0.
    pub fn transmittance(i0: f64, i_transmitted: f64) -> f64 {
        if i0 <= 0.0 {
            return 0.0;
        }
        i_transmitted / i0
    }

    /// Beer-Lambert law: A = epsilon * l * c.
    pub fn beer_lambert(epsilon: f64, path_cm: f64, conc_mol_per_l: f64) -> f64 {
        epsilon * path_cm * conc_mol_per_l
    }

    /// Concentration from absorbance: c = A / (epsilon * l).
    pub fn concentration_from_absorbance(
        a: f64,
        epsilon: f64,
        path_cm: f64,
    ) -> f64 {
        let denom = epsilon * path_cm;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        a / denom
    }

    /// Characteristic concentration: concentration that gives 0.0044 absorbance (1% absorption).
    pub fn characteristic_concentration(conc_for_0_0044_abs: f64) -> f64 {
        conc_for_0_0044_abs
    }

    /// Convert absorbance to transmittance: T = 10^(-A).
    pub fn absorbance_to_transmittance(a: f64) -> f64 {
        pow10_approx(-a)
    }

    /// Convert transmittance to absorbance: A = -log10(T).
    pub fn transmittance_to_absorbance(t: f64) -> f64 {
        if t <= 0.0 {
            return f64::INFINITY;
        }
        -log10_approx(t)
    }
}

/// Background correction methods for AAS.
pub struct BackgroundCorrection;

impl BackgroundCorrection {
    /// Deuterium lamp correction: atomic_absorbance = total - D2_background.
    pub fn deuterium_correction(total_abs: f64, d2_abs: f64) -> f64 {
        let corrected = total_abs - d2_abs;
        if corrected < 0.0 {
            0.0
        } else {
            corrected
        }
    }

    /// Zeeman background correction using polarized light components.
    /// sigma+/sigma- contain atomic + background, pi contains only background.
    pub fn zeeman_correction(sigma_plus: f64, sigma_minus: f64, pi: f64) -> f64 {
        let avg_sigma = (sigma_plus + sigma_minus) / 2.0;
        let corrected = avg_sigma - pi;
        if corrected < 0.0 {
            0.0
        } else {
            corrected
        }
    }

    /// Smith-Hieftje correction: uses self-reversal of HCL at high current.
    /// Low current = atomic + background, high current = mostly background.
    pub fn smith_hieftje(low_current: f64, high_current: f64) -> f64 {
        let corrected = low_current - high_current;
        if corrected < 0.0 {
            0.0
        } else {
            corrected
        }
    }
}

/// Synthetic spectrum and calibration data generator.
pub struct FesSimulator;

impl FesSimulator {
    /// Simulate a single emission line with a Gaussian profile.
    ///
    /// Returns (wavelengths, intensities) covering center +/- 5*width.
    pub fn simulate_emission_line(
        center_nm: f64,
        intensity: f64,
        width_nm: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = 201;
        let span = 5.0 * width_nm;
        let start = center_nm - span;
        let step = 2.0 * span / (n - 1) as f64;

        let mut wavelengths = Vec::with_capacity(n);
        let mut intensities = Vec::with_capacity(n);

        for i in 0..n {
            let w = start + i as f64 * step;
            let diff = w - center_nm;
            let gauss = intensity * exp_approx(-0.5 * (diff / width_nm).powi(2));
            wavelengths.push(w);
            intensities.push(gauss);
        }

        (wavelengths, intensities)
    }

    /// Simulate calibration data with Gaussian noise.
    ///
    /// Returns signals for given concentrations with y = slope * x + intercept + noise.
    pub fn simulate_calibration(
        slope: f64,
        intercept: f64,
        concentrations: &[f64],
        noise: f64,
    ) -> Vec<f64> {
        let mut signals = Vec::with_capacity(concentrations.len());
        let mut seed: u64 = 42;
        for &c in concentrations {
            let ideal = slope * c + intercept;
            // Simple deterministic pseudo-random noise
            let n = pseudo_gaussian_noise(&mut seed) * noise;
            signals.push(ideal + n);
        }
        signals
    }

    /// Simulate standard addition data.
    pub fn simulate_standard_addition(
        true_conc: f64,
        slope: f64,
        additions: &[f64],
        noise: f64,
    ) -> Vec<(f64, f64)> {
        let mut result = Vec::with_capacity(additions.len());
        let mut seed: u64 = 123;
        for &added in additions {
            let total_conc = true_conc + added;
            let signal = slope * total_conc + pseudo_gaussian_noise(&mut seed) * noise;
            result.push((added, signal));
        }
        result
    }

    /// Simulate a multi-element emission spectrum.
    pub fn simulate_multi_element(
        elements: &[(&str, f64)], // (element_name, relative_intensity)
        width_nm: f64,
        baseline: f64,
    ) -> EmissionSpectrum {
        let db = element_database();
        let mut all_centers: Vec<(f64, f64)> = Vec::new();

        for &(elem, intensity) in elements {
            for line in &db {
                if line.element == elem {
                    all_centers.push((line.wavelength_nm, intensity * line.relative_intensity));
                }
            }
        }

        // Generate wavelength range covering all lines
        let min_w = all_centers
            .iter()
            .map(|(w, _)| *w)
            .fold(f64::INFINITY, f64::min)
            - 20.0;
        let max_w = all_centers
            .iter()
            .map(|(w, _)| *w)
            .fold(f64::NEG_INFINITY, f64::max)
            + 20.0;

        let n = 1001;
        let step = (max_w - min_w) / (n - 1) as f64;
        let mut wavelengths = Vec::with_capacity(n);
        let mut intensities = Vec::with_capacity(n);

        for i in 0..n {
            let w = min_w + i as f64 * step;
            let mut total = baseline;
            for &(center, amp) in &all_centers {
                let diff = w - center;
                total += amp * exp_approx(-0.5 * (diff / width_nm).powi(2));
            }
            wavelengths.push(w);
            intensities.push(total);
        }

        EmissionSpectrum::new(wavelengths, intensities)
    }
}

/// Result for a single element in multi-element analysis.
#[derive(Debug, Clone)]
pub struct ElementResult {
    pub element: String,
    pub wavelength_nm: f64,
    pub intensity: f64,
    pub concentration: Option<f64>,
}

/// Multi-element analyzer.
pub struct MultiElementAnalyzer;

impl MultiElementAnalyzer {
    /// Analyze a spectrum for specified elements, returning detected lines.
    pub fn analyze_spectrum(
        spectrum: &EmissionSpectrum,
        elements: &[&str],
    ) -> Vec<ElementResult> {
        let db = element_database();
        let tolerance = 1.0; // nm
        let mut results = Vec::new();

        for elem_name in elements {
            // Find database lines for this element
            for line in &db {
                if line.element != *elem_name {
                    continue;
                }
                // Find the closest wavelength in the spectrum
                let mut best_idx = 0;
                let mut best_dist = f64::INFINITY;
                for (i, &w) in spectrum.wavelength_nm.iter().enumerate() {
                    let dist = (w - line.wavelength_nm).abs();
                    if dist < best_dist {
                        best_dist = dist;
                        best_idx = i;
                    }
                }
                if best_dist <= tolerance {
                    results.push(ElementResult {
                        element: elem_name.to_string(),
                        wavelength_nm: spectrum.wavelength_nm[best_idx],
                        intensity: spectrum.intensity[best_idx],
                        concentration: None,
                    });
                }
            }
        }
        results
    }

    /// Sequential analysis: use calibration curves to convert intensities to concentrations.
    pub fn sequential_analysis(
        spectra: &[(f64, f64)], // (wavelength, intensity) measurement points
        cal_curves: &[(String, CalibrationCurve)],
    ) -> Vec<(String, f64)> {
        let db = element_database();
        let tolerance = 1.0;
        let mut results = Vec::new();

        for (elem_name, cal) in cal_curves {
            // Find the element's primary line
            let primary_line = db
                .iter()
                .find(|l| l.element == *elem_name && l.relative_intensity >= 0.99);

            if let Some(line) = primary_line {
                // Find the measured intensity at this wavelength
                let measured = spectra.iter().find(|(w, _)| (w - line.wavelength_nm).abs() <= tolerance);

                if let Some((_, intensity)) = measured {
                    let conc = cal.unknown_concentration(*intensity);
                    results.push((elem_name.clone(), conc));
                }
            }
        }
        results
    }
}

// ---- Utility math functions (no external crates) ----

/// Linear regression: returns (slope, intercept, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len() as f64;
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sx2: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sx2 - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n, 0.0);
    }

    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;

    // R^2
    let y_mean = sy / n;
    let ss_tot: f64 = y.iter().map(|yi| (yi - y_mean).powi(2)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(xi, yi)| {
            let pred = slope * xi + intercept;
            (yi - pred).powi(2)
        })
        .sum();
    let r_squared = if ss_tot > 0.0 {
        1.0 - ss_res / ss_tot
    } else {
        1.0
    };

    (slope, intercept, r_squared)
}

/// Approximate exp(x) using the Taylor series with range reduction.
fn exp_approx(x: f64) -> f64 {
    if x > 709.0 {
        return f64::INFINITY;
    }
    if x < -745.0 {
        return 0.0;
    }

    // Range reduction: exp(x) = 2^k * exp(r)
    // where k = round(x / ln2), r = x - k * ln2
    const LN2: f64 = 0.693147180559945309417;
    let k = (x / LN2).round() as i64;
    let r = x - k as f64 * LN2;

    // Taylor series for exp(r) where |r| < ln(2)/2 ~ 0.347
    let mut sum = 1.0;
    let mut term = 1.0;
    for i in 1..=20 {
        term *= r / i as f64;
        sum += term;
        if term.abs() < 1e-16 {
            break;
        }
    }

    // Apply 2^k
    ldexp(sum, k)
}

/// Multiply by 2^k.
fn ldexp(x: f64, k: i64) -> f64 {
    if k >= 0 && k <= 1023 {
        x * (1u64 << k as u32) as f64
    } else if k < 0 && k >= -1074 {
        x / (1u64 << (-k).min(63) as u32) as f64
    } else if k > 1023 {
        f64::INFINITY
    } else {
        0.0
    }
}

/// Approximate ln(x) using the series expansion.
fn ln_approx(x: f64) -> f64 {
    if x <= 0.0 {
        return f64::NEG_INFINITY;
    }
    if x == 1.0 {
        return 0.0;
    }

    // Extract mantissa and exponent: x = m * 2^e, 0.5 <= m < 1.0
    // Use the identity: ln(x) = e*ln(2) + ln(m)
    let mut e = 0i64;
    let mut m = x;
    while m >= 2.0 {
        m /= 2.0;
        e += 1;
    }
    while m < 0.5 {
        m *= 2.0;
        e -= 1;
    }

    const LN2: f64 = 0.693147180559945309417;
    // ln(m) using series: ln(m) = ln(1 + (m-1)) via series for |m-1| < 0.5
    // Actually use: let t = (m-1)/(m+1), ln(m) = 2*(t + t^3/3 + t^5/5 + ...)
    let t = (m - 1.0) / (m + 1.0);
    let t2 = t * t;
    let mut sum = t;
    let mut power = t;
    for k in 1..=30 {
        power *= t2;
        sum += power / (2 * k + 1) as f64;
        if power.abs() < 1e-16 {
            break;
        }
    }
    e as f64 * LN2 + 2.0 * sum
}

/// Approximate log10(x) = ln(x) / ln(10).
fn log10_approx(x: f64) -> f64 {
    const LN10: f64 = 2.302585092994046;
    ln_approx(x) / LN10
}

/// Approximate 10^x = exp(x * ln(10)).
fn pow10_approx(x: f64) -> f64 {
    const LN10: f64 = 2.302585092994046;
    exp_approx(x * LN10)
}

/// Simple pseudo-random number generator (LCG) returning approximate Gaussian noise.
fn pseudo_gaussian_noise(seed: &mut u64) -> f64 {
    // Box-Muller-like approximation using sum of uniform randoms (CLT)
    let mut sum = 0.0;
    for _ in 0..12 {
        *seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u = (*seed >> 33) as f64 / (1u64 << 31) as f64;
        sum += u;
    }
    sum - 6.0 // Approximate standard normal
}

#[cfg(test)]
mod tests {
    use super::*;

    // Helper for approximate floating-point comparison
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ==================== EmissionSpectrum tests ====================

    #[test]
    fn test_emission_spectrum_creation() {
        let s = EmissionSpectrum::new(vec![580.0, 589.0, 600.0], vec![0.1, 1.0, 0.2]);
        assert_eq!(s.wavelength_nm.len(), 3);
        assert_eq!(s.intensity.len(), 3);
    }

    #[test]
    #[should_panic(expected = "wavelength and intensity must have same length")]
    fn test_emission_spectrum_mismatched_lengths() {
        EmissionSpectrum::new(vec![580.0, 589.0], vec![0.1]);
    }

    #[test]
    #[should_panic(expected = "spectrum must not be empty")]
    fn test_emission_spectrum_empty() {
        EmissionSpectrum::new(vec![], vec![]);
    }

    #[test]
    fn test_peak_wavelength() {
        let s = EmissionSpectrum::new(
            vec![580.0, 589.0, 600.0, 670.0],
            vec![0.1, 1.0, 0.2, 0.8],
        );
        assert!(approx_eq(s.peak_wavelength(), 589.0, 0.01));
    }

    #[test]
    fn test_peak_intensity() {
        let s = EmissionSpectrum::new(vec![580.0, 589.0, 600.0], vec![0.1, 0.95, 0.2]);
        assert!(approx_eq(s.peak_intensity(), 0.95, 0.001));
    }

    #[test]
    fn test_identify_element_na() {
        let s = EmissionSpectrum::new(vec![589.0], vec![1.0]);
        let matches = s.identify_element(589.0, 0.5);
        assert_eq!(matches.len(), 1);
        assert_eq!(matches[0].element, "Na");
    }

    #[test]
    fn test_identify_element_na_doublet() {
        let s = EmissionSpectrum::new(vec![589.0], vec![1.0]);
        let matches = s.identify_element(589.3, 0.5);
        assert_eq!(matches.len(), 2); // Both Na D1 and D2
    }

    #[test]
    fn test_identify_element_no_match() {
        let s = EmissionSpectrum::new(vec![500.0], vec![1.0]);
        let matches = s.identify_element(500.0, 0.1);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_identify_element_potassium() {
        let s = EmissionSpectrum::new(vec![766.5], vec![1.0]);
        let matches = s.identify_element(766.5, 0.5);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "K");
    }

    #[test]
    fn test_find_peaks() {
        let s = EmissionSpectrum::new(
            vec![580.0, 585.0, 589.0, 595.0, 600.0, 670.0, 680.0],
            vec![0.05, 0.1, 1.0, 0.3, 0.1, 0.8, 0.1],
        );
        let peaks = s.find_peaks(0.1);
        assert!(peaks.len() >= 2); // Should find peaks near 589 and 670
    }

    #[test]
    fn test_integrate_spectrum() {
        // Rectangular approximation: uniform intensity of 1.0 over 10 nm
        let wavelengths: Vec<f64> = (0..11).map(|i| 580.0 + i as f64).collect();
        let intensities = vec![1.0; 11];
        let s = EmissionSpectrum::new(wavelengths, intensities);
        let integral = s.integrate(580.0, 590.0);
        assert!(approx_eq(integral, 10.0, 0.1));
    }

    #[test]
    fn test_snr() {
        let s = EmissionSpectrum::new(
            vec![500.0, 510.0, 589.0, 700.0, 710.0],
            vec![0.01, 0.02, 1.0, 0.01, 0.02],
        );
        let snr = s.snr(500.0, 520.0);
        assert!(snr > 10.0); // High SNR expected
    }

    // ==================== Element database tests ====================

    #[test]
    fn test_element_database_has_entries() {
        let db = element_database();
        assert!(db.len() >= 10);
    }

    #[test]
    fn test_element_database_contains_na() {
        let db = element_database();
        let na_lines: Vec<_> = db.iter().filter(|e| e.element == "Na").collect();
        assert_eq!(na_lines.len(), 2); // D1 and D2
    }

    #[test]
    fn test_element_database_wavelengths() {
        let db = element_database();
        let li = db.iter().find(|e| e.element == "Li").unwrap();
        assert!(approx_eq(li.wavelength_nm, 670.8, 0.1));
    }

    // ==================== CalibrationCurve tests ====================

    #[test]
    fn test_calibration_linear_fit() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 0.5, 1.0, 1.5, 2.0],
        );
        let fit = cal.linear_fit();
        assert!(approx_eq(fit.slope, 0.5, 0.01));
        assert!(approx_eq(fit.intercept, 0.0, 0.01));
        assert!(fit.r_squared > 0.999);
    }

    #[test]
    fn test_calibration_with_intercept() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.1, 0.6, 1.1, 1.6],
        );
        let fit = cal.linear_fit();
        assert!(approx_eq(fit.slope, 0.5, 0.01));
        assert!(approx_eq(fit.intercept, 0.1, 0.01));
    }

    #[test]
    fn test_quadratic_fit() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 1.0, 4.0, 9.0, 16.0], // y = x^2
        );
        let fit = cal.quadratic_fit();
        assert!(approx_eq(fit.a, 1.0, 0.01));
        assert!(approx_eq(fit.b, 0.0, 0.1));
        assert!(approx_eq(fit.c, 0.0, 0.1));
        assert!(fit.r_squared > 0.999);
    }

    #[test]
    fn test_unknown_concentration() {
        let cal = CalibrationCurve::new(
            vec![0.0, 10.0, 20.0, 30.0],
            vec![0.0, 0.5, 1.0, 1.5],
        );
        let conc = cal.unknown_concentration(0.75);
        assert!(approx_eq(conc, 15.0, 0.5));
    }

    #[test]
    fn test_r_squared_perfect() {
        let cal = CalibrationCurve::new(
            vec![1.0, 2.0, 3.0, 4.0],
            vec![2.0, 4.0, 6.0, 8.0],
        );
        assert!(cal.r_squared() > 0.999);
    }

    #[test]
    fn test_sensitivity() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 0.25, 0.5, 0.75],
        );
        assert!(approx_eq(cal.sensitivity(), 0.25, 0.01));
    }

    #[test]
    fn test_detection_limit() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 0.5, 1.0, 1.5, 2.0],
        );
        let lod = cal.detection_limit(0.01); // blank_std = 0.01
        // LOD = 3 * 0.01 / 0.5 = 0.06
        assert!(approx_eq(lod, 0.06, 0.01));
    }

    #[test]
    fn test_quantification_limit() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0, 4.0],
            vec![0.0, 0.5, 1.0, 1.5, 2.0],
        );
        let loq = cal.quantification_limit(0.01);
        // LOQ = 10 * 0.01 / 0.5 = 0.20
        assert!(approx_eq(loq, 0.20, 0.01));
    }

    #[test]
    fn test_loq_greater_than_lod() {
        let cal = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0],
            vec![0.0, 1.0, 2.0],
        );
        let lod = cal.detection_limit(0.05);
        let loq = cal.quantification_limit(0.05);
        assert!(loq > lod);
    }

    // ==================== StandardAddition tests ====================

    #[test]
    fn test_standard_addition_basic() {
        // True concentration 5 ppm, slope 0.1 signal/ppm
        let sample_signal = 0.5; // 5 * 0.1
        let additions = vec![
            (5.0, 1.0),   // (5+5)*0.1 = 1.0
            (10.0, 1.5),  // (5+10)*0.1 = 1.5
            (15.0, 2.0),  // (5+15)*0.1 = 2.0
        ];
        let sa = StandardAddition::new(sample_signal, additions);
        let result = sa.fit();
        assert!(approx_eq(result.concentration, 5.0, 0.5));
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_standard_addition_slope() {
        let sa = StandardAddition::new(1.0, vec![(1.0, 2.0), (2.0, 3.0), (3.0, 4.0)]);
        let result = sa.fit();
        assert!(approx_eq(result.slope, 1.0, 0.01));
    }

    #[test]
    fn test_standard_addition_r_squared() {
        let sa = StandardAddition::new(0.5, vec![(1.0, 1.5), (2.0, 2.5), (3.0, 3.5)]);
        let result = sa.fit();
        assert!(result.r_squared > 0.999);
    }

    // ==================== InternalStandard tests ====================

    #[test]
    fn test_internal_standard_ratio() {
        let ratios = InternalStandard::ratio_method(&[1.0, 2.0, 3.0], &[0.5, 0.5, 0.5]);
        assert!(approx_eq(ratios[0], 2.0, 0.001));
        assert!(approx_eq(ratios[1], 4.0, 0.001));
        assert!(approx_eq(ratios[2], 6.0, 0.001));
    }

    #[test]
    fn test_internal_standard_zero_is() {
        let ratios = InternalStandard::ratio_method(&[1.0], &[0.0]);
        assert!(approx_eq(ratios[0], 0.0, 0.001));
    }

    #[test]
    fn test_internal_standard_calibration() {
        let concs = vec![0.0, 1.0, 2.0, 3.0];
        let analyte = vec![0.0, 0.5, 1.0, 1.5];
        let is_signals = vec![1.0, 1.0, 1.0, 1.0]; // constant IS
        let cal = InternalStandard::calibrate_with_is(&concs, &analyte, &is_signals);
        assert!(approx_eq(cal.slope, 0.5, 0.01));
    }

    #[test]
    fn test_internal_standard_unknown() {
        let cal = LinearCalResult {
            slope: 0.5,
            intercept: 0.0,
            r_squared: 1.0,
        };
        let conc = InternalStandard::unknown_with_is(0.75, 1.0, &cal);
        assert!(approx_eq(conc, 1.5, 0.01));
    }

    // ==================== InterferenceCorrection tests ====================

    #[test]
    fn test_spectral_overlap_correction() {
        let corrected = InterferenceCorrection::spectral_overlap_correction(1.0, 0.3, 0.5);
        assert!(approx_eq(corrected, 0.85, 0.001));
    }

    #[test]
    fn test_spectral_overlap_floor_zero() {
        let corrected = InterferenceCorrection::spectral_overlap_correction(0.1, 1.0, 0.5);
        assert!(approx_eq(corrected, 0.0, 0.001));
    }

    #[test]
    fn test_ionization_suppression() {
        let ratio = InterferenceCorrection::ionization_suppression(0.5, 0.8);
        assert!(approx_eq(ratio, 1.6, 0.001));
    }

    #[test]
    fn test_matrix_matching_seawater() {
        let additives = InterferenceCorrection::matrix_matching("seawater");
        assert!(additives.len() >= 2);
        assert!(additives.iter().any(|(name, _)| name == "CsCl"));
    }

    #[test]
    fn test_matrix_matching_blood() {
        let additives = InterferenceCorrection::matrix_matching("blood");
        assert!(!additives.is_empty());
    }

    #[test]
    fn test_matrix_matching_unknown() {
        let additives = InterferenceCorrection::matrix_matching("unknown_matrix");
        assert!(!additives.is_empty());
    }

    #[test]
    fn test_self_absorption_correction() {
        let factor = InterferenceCorrection::self_absorption_correction(0.1, 0.5);
        // 1 / (1 - 0.5*0.1) = 1 / 0.95 ~ 1.0526
        assert!(approx_eq(factor, 1.0 / 0.95, 0.001));
    }

    #[test]
    fn test_self_absorption_high_conc() {
        let factor = InterferenceCorrection::self_absorption_correction(2.0, 0.5);
        // 1 / (1 - 1.0) = infinity
        assert!(factor.is_infinite());
    }

    // ==================== FlameParameters tests ====================

    #[test]
    fn test_flame_temperature_air_acetylene() {
        assert!(approx_eq(FlameParameters::temperature_k(FlameType::AirAcetylene), 2300.0, 1.0));
    }

    #[test]
    fn test_flame_temperature_n2o_acetylene() {
        assert!(approx_eq(
            FlameParameters::temperature_k(FlameType::NitrousOxideAcetylene),
            2900.0,
            1.0
        ));
    }

    #[test]
    fn test_flame_temperature_ordering() {
        let t_propane = FlameParameters::temperature_k(FlameType::AirPropane);
        let t_h2 = FlameParameters::temperature_k(FlameType::AirHydrogen);
        let t_c2h2 = FlameParameters::temperature_k(FlameType::AirAcetylene);
        let t_n2o = FlameParameters::temperature_k(FlameType::NitrousOxideAcetylene);
        assert!(t_propane < t_h2);
        assert!(t_h2 < t_c2h2);
        assert!(t_c2h2 < t_n2o);
    }

    #[test]
    fn test_suitable_elements_air_acetylene() {
        let elements = FlameParameters::suitable_elements(FlameType::AirAcetylene);
        assert!(elements.contains(&"Na"));
        assert!(elements.contains(&"K"));
        assert!(elements.contains(&"Ca"));
    }

    #[test]
    fn test_suitable_elements_n2o() {
        let elements = FlameParameters::suitable_elements(FlameType::NitrousOxideAcetylene);
        assert!(elements.contains(&"Al"));
        assert!(elements.contains(&"Si"));
    }

    #[test]
    fn test_suitable_elements_propane() {
        let elements = FlameParameters::suitable_elements(FlameType::AirPropane);
        assert!(elements.contains(&"Na"));
        assert!(elements.contains(&"K"));
        // Propane is limited
        assert!(!elements.contains(&"Al"));
    }

    #[test]
    fn test_boltzmann_population() {
        // Na D line: 2.1 eV energy, air-acetylene 2300 K
        let ratio = FlameParameters::boltzmann_population(2.1, 2300.0, 2.0, 1.0);
        // Should be a small number (excited state is much less populated)
        assert!(ratio > 0.0);
        assert!(ratio < 0.01);
    }

    #[test]
    fn test_boltzmann_higher_temp_more_population() {
        let ratio_low = FlameParameters::boltzmann_population(2.0, 2000.0, 2.0, 2.0);
        let ratio_high = FlameParameters::boltzmann_population(2.0, 3000.0, 2.0, 2.0);
        assert!(ratio_high > ratio_low);
    }

    #[test]
    fn test_boltzmann_degeneracy_effect() {
        let ratio_1 = FlameParameters::boltzmann_population(2.0, 2300.0, 1.0, 1.0);
        let ratio_3 = FlameParameters::boltzmann_population(2.0, 2300.0, 3.0, 1.0);
        assert!(approx_eq(ratio_3, 3.0 * ratio_1, 1e-10));
    }

    // ==================== AbsorptionMode tests ====================

    #[test]
    fn test_absorbance() {
        let a = AbsorptionMode::absorbance(100.0, 10.0);
        assert!(approx_eq(a, 1.0, 0.001)); // log10(100/10) = 1
    }

    #[test]
    fn test_absorbance_full_transmission() {
        let a = AbsorptionMode::absorbance(100.0, 100.0);
        assert!(approx_eq(a, 0.0, 0.001));
    }

    #[test]
    fn test_transmittance() {
        let t = AbsorptionMode::transmittance(100.0, 50.0);
        assert!(approx_eq(t, 0.5, 0.001));
    }

    #[test]
    fn test_beer_lambert() {
        // A = 100 * 1.0 * 0.01 = 1.0
        let a = AbsorptionMode::beer_lambert(100.0, 1.0, 0.01);
        assert!(approx_eq(a, 1.0, 0.001));
    }

    #[test]
    fn test_concentration_from_absorbance() {
        let c = AbsorptionMode::concentration_from_absorbance(0.5, 100.0, 1.0);
        assert!(approx_eq(c, 0.005, 0.0001));
    }

    #[test]
    fn test_characteristic_concentration() {
        let cc = AbsorptionMode::characteristic_concentration(0.05);
        assert!(approx_eq(cc, 0.05, 0.001));
    }

    #[test]
    fn test_absorbance_to_transmittance() {
        let t = AbsorptionMode::absorbance_to_transmittance(1.0);
        assert!(approx_eq(t, 0.1, 0.001)); // 10^(-1) = 0.1
    }

    #[test]
    fn test_transmittance_to_absorbance() {
        let a = AbsorptionMode::transmittance_to_absorbance(0.01);
        assert!(approx_eq(a, 2.0, 0.01)); // -log10(0.01) = 2
    }

    #[test]
    fn test_abs_trans_roundtrip() {
        let original_abs = 0.75;
        let t = AbsorptionMode::absorbance_to_transmittance(original_abs);
        let recovered = AbsorptionMode::transmittance_to_absorbance(t);
        assert!(approx_eq(recovered, original_abs, 0.001));
    }

    // ==================== BackgroundCorrection tests ====================

    #[test]
    fn test_deuterium_correction() {
        let corrected = BackgroundCorrection::deuterium_correction(0.5, 0.1);
        assert!(approx_eq(corrected, 0.4, 0.001));
    }

    #[test]
    fn test_deuterium_correction_floor() {
        let corrected = BackgroundCorrection::deuterium_correction(0.1, 0.5);
        assert!(approx_eq(corrected, 0.0, 0.001));
    }

    #[test]
    fn test_zeeman_correction() {
        let corrected = BackgroundCorrection::zeeman_correction(0.8, 0.6, 0.3);
        // avg_sigma = 0.7, corrected = 0.7 - 0.3 = 0.4
        assert!(approx_eq(corrected, 0.4, 0.001));
    }

    #[test]
    fn test_smith_hieftje() {
        let corrected = BackgroundCorrection::smith_hieftje(0.6, 0.2);
        assert!(approx_eq(corrected, 0.4, 0.001));
    }

    #[test]
    fn test_smith_hieftje_negative_floor() {
        let corrected = BackgroundCorrection::smith_hieftje(0.1, 0.5);
        assert!(approx_eq(corrected, 0.0, 0.001));
    }

    // ==================== FesSimulator tests ====================

    #[test]
    fn test_simulate_emission_line() {
        let (wavelengths, intensities) = FesSimulator::simulate_emission_line(589.0, 1.0, 0.5);
        assert!(!wavelengths.is_empty());
        assert_eq!(wavelengths.len(), intensities.len());

        // Peak should be near center
        let max_idx = intensities
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        assert!(approx_eq(wavelengths[max_idx], 589.0, 0.1));
        assert!(approx_eq(intensities[max_idx], 1.0, 0.01));
    }

    #[test]
    fn test_simulate_emission_line_width() {
        let (w1, i1) = FesSimulator::simulate_emission_line(589.0, 1.0, 0.2);
        let (w2, i2) = FesSimulator::simulate_emission_line(589.0, 1.0, 1.0);

        // Wider line should have wider spectral range
        let span1 = w1.last().unwrap() - w1.first().unwrap();
        let span2 = w2.last().unwrap() - w2.first().unwrap();
        assert!(span2 > span1);

        // But same peak
        assert!(approx_eq(
            *i1.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(),
            *i2.iter().max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap(),
            0.01
        ));
    }

    #[test]
    fn test_simulate_calibration() {
        let concs = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let signals = FesSimulator::simulate_calibration(0.5, 0.1, &concs, 0.0);
        for (i, &c) in concs.iter().enumerate() {
            assert!(approx_eq(signals[i], 0.5 * c + 0.1, 0.001));
        }
    }

    #[test]
    fn test_simulate_calibration_with_noise() {
        let concs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let signals = FesSimulator::simulate_calibration(1.0, 0.0, &concs, 0.01);
        // Should be close to ideal but not exact
        for (i, &c) in concs.iter().enumerate() {
            assert!(approx_eq(signals[i], c, 0.1));
        }
    }

    #[test]
    fn test_simulate_standard_addition() {
        let additions = vec![0.0, 1.0, 2.0, 3.0];
        let data = FesSimulator::simulate_standard_addition(5.0, 0.1, &additions, 0.0);
        assert_eq!(data.len(), 4);
        // First point: conc = 5+0 = 5, signal ~ 0.5
        assert!(approx_eq(data[0].1, 0.5, 0.1));
    }

    #[test]
    fn test_simulate_multi_element() {
        let spectrum = FesSimulator::simulate_multi_element(
            &[("Na", 1.0), ("K", 0.5)],
            0.5,
            0.01,
        );
        assert!(spectrum.wavelength_nm.len() > 100);
        // Should show Na peak near 589
        let peak_w = spectrum.peak_wavelength();
        assert!(approx_eq(peak_w, 589.0, 1.0));
    }

    // ==================== MultiElementAnalyzer tests ====================

    #[test]
    fn test_analyze_spectrum_finds_na() {
        let spectrum = FesSimulator::simulate_multi_element(
            &[("Na", 1.0)],
            0.5,
            0.001,
        );
        let results = MultiElementAnalyzer::analyze_spectrum(&spectrum, &["Na"]);
        assert!(!results.is_empty());
        assert_eq!(results[0].element, "Na");
    }

    #[test]
    fn test_analyze_spectrum_multiple_elements() {
        let spectrum = FesSimulator::simulate_multi_element(
            &[("Na", 1.0), ("K", 0.8), ("Li", 0.5)],
            0.5,
            0.001,
        );
        let results = MultiElementAnalyzer::analyze_spectrum(&spectrum, &["Na", "K", "Li"]);
        let found_elements: Vec<_> = results.iter().map(|r| r.element.as_str()).collect();
        assert!(found_elements.contains(&"Na"));
        assert!(found_elements.contains(&"K"));
        assert!(found_elements.contains(&"Li"));
    }

    #[test]
    fn test_analyze_spectrum_absent_element() {
        let spectrum = FesSimulator::simulate_multi_element(
            &[("Na", 1.0)],
            0.5,
            0.001,
        );
        // Cu at 324.8 nm is outside our Na spectrum range
        let results = MultiElementAnalyzer::analyze_spectrum(&spectrum, &["Cu"]);
        // Cu is outside the generated range, so shouldn't be found
        assert!(results.is_empty() || results[0].intensity < 0.01);
    }

    #[test]
    fn test_sequential_analysis() {
        let cal_na = CalibrationCurve::new(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![0.0, 0.5, 1.0, 1.5],
        );
        let spectra = vec![(589.0, 0.75)]; // Na at 589.0 nm with intensity 0.75
        let cal_curves = vec![("Na".to_string(), cal_na)];
        let results = MultiElementAnalyzer::sequential_analysis(&spectra, &cal_curves);
        assert_eq!(results.len(), 1);
        assert_eq!(results[0].0, "Na");
        assert!(approx_eq(results[0].1, 1.5, 0.1)); // conc ~ 0.75/0.5 = 1.5
    }

    // ==================== Math utility tests ====================

    #[test]
    fn test_exp_approx_zero() {
        assert!(approx_eq(exp_approx(0.0), 1.0, 1e-10));
    }

    #[test]
    fn test_exp_approx_one() {
        assert!(approx_eq(exp_approx(1.0), std::f64::consts::E, 1e-8));
    }

    #[test]
    fn test_exp_approx_negative() {
        assert!(approx_eq(exp_approx(-1.0), 1.0 / std::f64::consts::E, 1e-8));
    }

    #[test]
    fn test_exp_approx_large() {
        assert!(exp_approx(710.0).is_infinite());
    }

    #[test]
    fn test_exp_approx_very_negative() {
        assert!(approx_eq(exp_approx(-750.0), 0.0, 1e-30));
    }

    #[test]
    fn test_ln_approx_one() {
        assert!(approx_eq(ln_approx(1.0), 0.0, 1e-12));
    }

    #[test]
    fn test_ln_approx_e() {
        assert!(approx_eq(ln_approx(std::f64::consts::E), 1.0, 1e-8));
    }

    #[test]
    fn test_log10_approx() {
        assert!(approx_eq(log10_approx(100.0), 2.0, 1e-6));
        assert!(approx_eq(log10_approx(1000.0), 3.0, 1e-6));
    }

    #[test]
    fn test_pow10_approx() {
        assert!(approx_eq(pow10_approx(2.0), 100.0, 0.01));
        assert!(approx_eq(pow10_approx(-1.0), 0.1, 0.001));
    }

    #[test]
    fn test_linear_regression_perfect() {
        let (slope, intercept, r_sq) = linear_regression(
            &[0.0, 1.0, 2.0, 3.0],
            &[1.0, 3.0, 5.0, 7.0],
        );
        assert!(approx_eq(slope, 2.0, 0.001));
        assert!(approx_eq(intercept, 1.0, 0.001));
        assert!(r_sq > 0.999);
    }

    // ==================== Integration / end-to-end tests ====================

    #[test]
    fn test_full_calibration_workflow() {
        // Simulate a calibration, then analyze an unknown
        let concs = vec![0.0, 1.0, 2.0, 5.0, 10.0];
        let signals = FesSimulator::simulate_calibration(0.5, 0.05, &concs, 0.0);

        let cal = CalibrationCurve::new(concs, signals);
        assert!(cal.r_squared() > 0.99);

        let unknown_signal = 2.55; // Should give ~5.0 ppm
        let conc = cal.unknown_concentration(unknown_signal);
        assert!(approx_eq(conc, 5.0, 0.1));
    }

    #[test]
    fn test_emission_intensity_factor() {
        let factor = FlameParameters::emission_intensity_factor(2.1, 2300.0, 3.0, 1.0);
        assert!(factor > 0.0);
        assert!(factor < 1.0);
    }

    #[test]
    fn test_calibration_min_points() {
        let cal = CalibrationCurve::new(vec![0.0, 1.0], vec![0.0, 1.0]);
        let fit = cal.linear_fit();
        assert!(approx_eq(fit.slope, 1.0, 0.001));
    }

    #[test]
    fn test_matrix_matching_soil() {
        let additives = InterferenceCorrection::matrix_matching("soil");
        assert!(additives.iter().any(|(name, _)| name == "LaCl3"));
    }

    #[test]
    fn test_matrix_matching_urine() {
        let additives = InterferenceCorrection::matrix_matching("urine");
        assert!(!additives.is_empty());
    }
}
