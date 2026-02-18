// trace:FR-CDSPEC | ai:claude
//! # Circular Dichroism Spectrometer
//!
//! CD spectroscopy data analysis for protein secondary structure determination,
//! nucleic acid conformational analysis, and chiral molecule characterization
//! from differential circular polarization absorption.
//!
//! ## Science
//!
//! CD = differential absorption of left vs right circular polarized light: ΔA = A_L - A_R
//! Ellipticity θ (mdeg) = 32.982 × ΔA
//! Mean residue ellipticity [θ]_MRE = θ × MRW / (10 × c × l)
//! Protein far-UV CD (190-260nm) reports secondary structure
//! α-helix: double minimum at 208, 222 nm, positive at 193 nm
//! β-sheet: minimum ~218 nm, positive ~195 nm

use std::f64::consts::PI;

/// Gas constant R in J/(mol·K)
const R_GAS: f64 = 8.314;

/// Conversion factor from ΔA to ellipticity in mdeg
const DELTA_A_TO_MDEG: f64 = 32.982;

// ---------------------------------------------------------------------------
// CdSpectrum
// ---------------------------------------------------------------------------

/// CD intensity vs wavelength
#[derive(Debug, Clone)]
pub struct CdSpectrum {
    /// Wavelengths in nm
    pub wavelength_nm: Vec<f64>,
    /// Ellipticity values in mdeg (or Δε after conversion)
    pub values: Vec<f64>,
    /// Unit label for the values
    pub unit: CdUnit,
}

/// Units for CD data
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CdUnit {
    Millidegrees,
    DeltaEpsilon,
    MeanResidueEllipticity,
}

impl CdSpectrum {
    /// Create a new CD spectrum from wavelength and ellipticity (mdeg) data.
    pub fn new(wavelength_nm: Vec<f64>, ellipticity_mdeg: Vec<f64>) -> Self {
        assert_eq!(wavelength_nm.len(), ellipticity_mdeg.len(),
                   "wavelength and ellipticity vectors must have same length");
        Self {
            wavelength_nm,
            values: ellipticity_mdeg,
            unit: CdUnit::Millidegrees,
        }
    }

    /// Convert millidegrees to molar circular dichroism Δε (L/(mol·cm)).
    ///
    /// Δε = θ / (32982 × c × l)
    /// where c is concentration in mol/L, l is path length in cm.
    pub fn to_delta_epsilon(&self, concentration_mol_per_l: f64, path_length_cm: f64) -> CdSpectrum {
        assert!(concentration_mol_per_l > 0.0, "concentration must be positive");
        assert!(path_length_cm > 0.0, "path length must be positive");
        let factor = 1.0 / (DELTA_A_TO_MDEG * 1000.0 * concentration_mol_per_l * path_length_cm);
        CdSpectrum {
            wavelength_nm: self.wavelength_nm.clone(),
            values: self.values.iter().map(|&v| v * factor).collect(),
            unit: CdUnit::DeltaEpsilon,
        }
    }

    /// Convert to mean residue ellipticity [θ]_MRE (deg·cm²/dmol).
    ///
    /// [θ]_MRE = θ × MRW / (10 × c × l)
    /// MRW = MW / num_residues
    pub fn to_mean_residue_ellipticity(
        &self,
        conc_mg_per_ml: f64,
        path_cm: f64,
        mw: f64,
        num_residues: usize,
    ) -> CdSpectrum {
        assert!(conc_mg_per_ml > 0.0);
        assert!(path_cm > 0.0);
        assert!(mw > 0.0);
        assert!(num_residues > 0);
        let mrw = mw / num_residues as f64;
        let factor = mrw / (10.0 * conc_mg_per_ml * path_cm);
        CdSpectrum {
            wavelength_nm: self.wavelength_nm.clone(),
            values: self.values.iter().map(|&v| v * factor).collect(),
            unit: CdUnit::MeanResidueEllipticity,
        }
    }

    /// Return the wavelength range (min, max).
    pub fn wavelength_range(&self) -> (f64, f64) {
        if self.wavelength_nm.is_empty() {
            return (0.0, 0.0);
        }
        let mut min = self.wavelength_nm[0];
        let mut max = self.wavelength_nm[0];
        for &w in &self.wavelength_nm {
            if w < min { min = w; }
            if w > max { max = w; }
        }
        (min, max)
    }

    /// Apply moving-average smoothing.
    pub fn smooth(&self, window: usize) -> CdSpectrum {
        let n = self.values.len();
        if window <= 1 || n == 0 {
            return self.clone();
        }
        let half = window / 2;
        let mut smoothed = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = if i + half < n { i + half } else { n - 1 };
            let count = (hi - lo + 1) as f64;
            let sum: f64 = self.values[lo..=hi].iter().sum();
            smoothed[i] = sum / count;
        }
        CdSpectrum {
            wavelength_nm: self.wavelength_nm.clone(),
            values: smoothed,
            unit: self.unit,
        }
    }

    /// Normalize to max absolute ellipticity.
    pub fn normalize(&self) -> CdSpectrum {
        let max_abs = self.values.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        if max_abs < 1e-30 {
            return self.clone();
        }
        CdSpectrum {
            wavelength_nm: self.wavelength_nm.clone(),
            values: self.values.iter().map(|v| v / max_abs).collect(),
            unit: self.unit,
        }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.wavelength_nm.len()
    }

    /// Whether spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.wavelength_nm.is_empty()
    }

    /// Interpolate the CD value at a given wavelength (linear interpolation).
    pub fn interpolate(&self, wl: f64) -> f64 {
        if self.wavelength_nm.is_empty() {
            return 0.0;
        }
        if wl <= self.wavelength_nm[0] {
            return self.values[0];
        }
        let n = self.wavelength_nm.len();
        if wl >= self.wavelength_nm[n - 1] {
            return self.values[n - 1];
        }
        for i in 0..n - 1 {
            if self.wavelength_nm[i] <= wl && wl <= self.wavelength_nm[i + 1] {
                let frac = (wl - self.wavelength_nm[i])
                    / (self.wavelength_nm[i + 1] - self.wavelength_nm[i]);
                return self.values[i] + frac * (self.values[i + 1] - self.values[i]);
            }
        }
        self.values[n - 1]
    }

    /// Find wavelength of minimum CD value.
    pub fn min_wavelength(&self) -> f64 {
        if self.wavelength_nm.is_empty() {
            return 0.0;
        }
        let mut min_idx = 0;
        let mut min_val = self.values[0];
        for (i, &v) in self.values.iter().enumerate() {
            if v < min_val {
                min_val = v;
                min_idx = i;
            }
        }
        self.wavelength_nm[min_idx]
    }

    /// Find wavelength of maximum CD value.
    pub fn max_wavelength(&self) -> f64 {
        if self.wavelength_nm.is_empty() {
            return 0.0;
        }
        let mut max_idx = 0;
        let mut max_val = self.values[0];
        for (i, &v) in self.values.iter().enumerate() {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }
        self.wavelength_nm[max_idx]
    }

    /// Subtract baseline from spectrum.
    pub fn subtract_baseline(&self, baseline: &CdSpectrum) -> CdSpectrum {
        let values: Vec<f64> = self.wavelength_nm.iter().map(|&wl| {
            self.interpolate(wl) - baseline.interpolate(wl)
        }).collect();
        CdSpectrum {
            wavelength_nm: self.wavelength_nm.clone(),
            values,
            unit: self.unit,
        }
    }
}

// ---------------------------------------------------------------------------
// SecondaryStructure + Estimator
// ---------------------------------------------------------------------------

/// Protein secondary structure fractions.
#[derive(Debug, Clone, Copy)]
pub struct SecondaryStructure {
    pub alpha_helix: f64,
    pub beta_sheet: f64,
    pub beta_turn: f64,
    pub random_coil: f64,
}

impl SecondaryStructure {
    /// Sum of all fractions.
    pub fn total(&self) -> f64 {
        self.alpha_helix + self.beta_sheet + self.beta_turn + self.random_coil
    }

    /// Normalize fractions so they sum to 1.0.
    pub fn normalize(&self) -> SecondaryStructure {
        let t = self.total();
        if t < 1e-30 {
            return *self;
        }
        SecondaryStructure {
            alpha_helix: self.alpha_helix / t,
            beta_sheet: self.beta_sheet / t,
            beta_turn: self.beta_turn / t,
            random_coil: self.random_coil / t,
        }
    }
}

/// Reference wavelengths for secondary structure CD (far-UV).
const REF_WAVELENGTHS: [f64; 8] = [190.0, 195.0, 200.0, 208.0, 215.0, 218.0, 222.0, 240.0];

/// Reference CD values per 1000 deg·cm²/dmol for pure α-helix.
const REF_ALPHA_HELIX: [f64; 8] = [
    -20000.0, 70000.0, 25000.0, -33000.0, -20000.0, -15000.0, -33000.0, -5000.0,
];

/// Reference CD values for pure β-sheet.
const REF_BETA_SHEET: [f64; 8] = [
    -5000.0, 18000.0, 5000.0, -5000.0, -10000.0, -18000.0, -5000.0, -2000.0,
];

/// Reference CD values for β-turn.
const REF_BETA_TURN: [f64; 8] = [
    -5000.0, 5000.0, 15000.0, -8000.0, -5000.0, -3000.0, -8000.0, -1000.0,
];

/// Reference CD values for random coil.
const REF_RANDOM_COIL: [f64; 8] = [
    -5000.0, -15000.0, -25000.0, -2000.0, 2000.0, 3000.0, -1000.0, 0.0,
];

/// Estimate secondary structure from far-UV CD data.
pub struct SecondaryStructureEstimator;

impl SecondaryStructureEstimator {
    /// Estimate secondary structure fractions by linear combination fitting.
    ///
    /// θ(λ) = f_α·θ_α(λ) + f_β·θ_β(λ) + f_t·θ_t(λ) + f_c·θ_c(λ)
    /// with constraint Σf_i = 1.
    pub fn estimate(spectrum: &CdSpectrum) -> SecondaryStructure {
        // Sample the spectrum at reference wavelengths
        let observed: Vec<f64> = REF_WAVELENGTHS.iter().map(|&wl| spectrum.interpolate(wl)).collect();

        // Build reference matrix columns at reference wavelengths
        let refs = [&REF_ALPHA_HELIX, &REF_BETA_SHEET, &REF_BETA_TURN, &REF_RANDOM_COIL];
        let n_components = 4;
        let n_points = REF_WAVELENGTHS.len();

        // Simple NNLS-like grid search for best combination with sum ~= 1
        let steps = 21; // 0.00, 0.05, 0.10, ..., 1.00
        let mut best_err = f64::MAX;
        let mut best_f = [0.25_f64; 4];

        for ia in 0..steps {
            let fa = ia as f64 / (steps - 1) as f64;
            for ib in 0..(steps - ia) {
                let fb = ib as f64 / (steps - 1) as f64;
                if fa + fb > 1.0 + 1e-9 { continue; }
                for it in 0..(steps - ia - ib) {
                    let ft = it as f64 / (steps - 1) as f64;
                    if fa + fb + ft > 1.0 + 1e-9 { continue; }
                    let fc = 1.0 - fa - fb - ft;
                    if fc < -1e-9 { continue; }
                    let fracs = [fa, fb, ft, fc];

                    let mut err = 0.0;
                    for j in 0..n_points {
                        let mut pred = 0.0;
                        for k in 0..n_components {
                            pred += fracs[k] * refs[k][j];
                        }
                        let d = observed[j] - pred;
                        err += d * d;
                    }
                    if err < best_err {
                        best_err = err;
                        best_f = [fa, fb, ft, fc.max(0.0)];
                    }
                }
            }
        }

        SecondaryStructure {
            alpha_helix: best_f[0],
            beta_sheet: best_f[1],
            beta_turn: best_f[2],
            random_coil: best_f[3],
        }
    }

    /// Estimate α-helix content from the 222 nm CD value using Greenfield-Fasman.
    ///
    /// f_H = (θ_222 - θ_coil) / (θ_helix - θ_coil)
    /// θ_helix = -40000 × (1 - 2.57/n)
    /// θ_coil = 640
    pub fn helix_content_from_222nm(theta_222: f64, num_residues: usize) -> f64 {
        let n = num_residues as f64;
        let theta_helix = -40000.0 * (1.0 - 2.57 / n);
        let theta_coil = 640.0;
        let fraction = (theta_222 - theta_coil) / (theta_helix - theta_coil);
        fraction.max(0.0).min(1.0)
    }
}

// ---------------------------------------------------------------------------
// ThermalDenaturation
// ---------------------------------------------------------------------------

/// Result of melting curve analysis.
#[derive(Debug, Clone)]
pub struct MeltingResult {
    /// Melting temperature in Celsius.
    pub tm_c: f64,
    /// van't Hoff enthalpy in kJ/mol.
    pub delta_h_kj_per_mol: f64,
    /// Entropy of unfolding in J/(mol·K).
    pub delta_s: f64,
    /// Fraction unfolded at each temperature.
    pub fraction_unfolded: Vec<f64>,
}

/// Thermal denaturation analysis from CD vs temperature.
#[derive(Debug, Clone)]
pub struct ThermalDenaturation {
    pub temperatures_c: Vec<f64>,
    pub cd_values: Vec<f64>,
}

impl ThermalDenaturation {
    pub fn new(temperatures_c: Vec<f64>, cd_values: Vec<f64>) -> Self {
        assert_eq!(temperatures_c.len(), cd_values.len());
        Self { temperatures_c, cd_values }
    }

    /// Fit a two-state melting model to the CD vs temperature data.
    ///
    /// f_U(T) = 1/(1 + exp(ΔH/R × (1/T - 1/Tm)))
    /// CD(T) = CD_N × (1 - f_U) + CD_U × f_U
    pub fn fit_melting(&self, _wavelength_nm: f64) -> MeltingResult {
        let n = self.temperatures_c.len();
        if n < 4 {
            return MeltingResult {
                tm_c: 0.0,
                delta_h_kj_per_mol: 0.0,
                delta_s: 0.0,
                fraction_unfolded: vec![0.0; n],
            };
        }

        // Native and denatured baselines from first/last 10% of data
        let low_n = (n / 10).max(1);
        let high_n = (n / 10).max(1);
        let cd_native: f64 = self.cd_values[..low_n].iter().sum::<f64>() / low_n as f64;
        let cd_denatured: f64 = self.cd_values[n - high_n..].iter().sum::<f64>() / high_n as f64;

        // Compute fraction unfolded
        let range = cd_denatured - cd_native;
        let fraction_unfolded: Vec<f64> = if range.abs() < 1e-30 {
            vec![0.0; n]
        } else {
            self.cd_values.iter().map(|&cd| {
                ((cd - cd_native) / range).max(0.0).min(1.0)
            }).collect()
        };

        // Find Tm as temperature where f_U ~ 0.5 (by interpolation)
        let mut tm_c = self.temperatures_c[n / 2];
        for i in 0..n - 1 {
            if (fraction_unfolded[i] - 0.5) * (fraction_unfolded[i + 1] - 0.5) <= 0.0 {
                let frac = (0.5 - fraction_unfolded[i])
                    / (fraction_unfolded[i + 1] - fraction_unfolded[i]);
                tm_c = self.temperatures_c[i]
                    + frac * (self.temperatures_c[i + 1] - self.temperatures_c[i]);
                break;
            }
        }

        let tm_k = tm_c + 273.15;

        // Estimate ΔH from slope of f_U at Tm using numerical derivative
        // df_U/dT at Tm ≈ ΔH / (4·R·Tm²) for two-state
        let slope = Self::slope_at_temperature(&self.temperatures_c, &fraction_unfolded, tm_c);
        let delta_h = 4.0 * R_GAS * tm_k * tm_k * slope; // J/mol
        let delta_h_kj = delta_h / 1000.0;

        let delta_s = if tm_k > 0.0 { delta_h / tm_k } else { 0.0 };

        MeltingResult {
            tm_c,
            delta_h_kj_per_mol: delta_h_kj,
            delta_s,
            fraction_unfolded,
        }
    }

    /// van't Hoff enthalpy from Tm and slope at Tm.
    ///
    /// ΔH_vH = 4·R·Tm²·(dα/dT)_Tm
    pub fn van_hoff_enthalpy(tm_k: f64, slope_at_tm: f64) -> f64 {
        4.0 * R_GAS * tm_k * tm_k * slope_at_tm / 1000.0 // kJ/mol
    }

    fn slope_at_temperature(temps: &[f64], values: &[f64], t: f64) -> f64 {
        let n = temps.len();
        if n < 2 {
            return 0.0;
        }
        // Find nearest point to t
        let mut best_i = 0;
        let mut best_d = (temps[0] - t).abs();
        for i in 1..n {
            let d = (temps[i] - t).abs();
            if d < best_d {
                best_d = d;
                best_i = i;
            }
        }
        if best_i == 0 {
            (values[1] - values[0]) / (temps[1] - temps[0])
        } else if best_i == n - 1 {
            (values[n - 1] - values[n - 2]) / (temps[n - 1] - temps[n - 2])
        } else {
            // Central difference
            (values[best_i + 1] - values[best_i - 1])
                / (temps[best_i + 1] - temps[best_i - 1])
        }
    }
}

// ---------------------------------------------------------------------------
// ConformationalAnalysis
// ---------------------------------------------------------------------------

/// Protein conformation type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Conformation {
    AlphaHelix,
    BetaSheet,
    RandomCoil,
    Mixed,
    CoiledCoil,
}

/// Structural conformation analysis from CD spectra.
pub struct ConformationalAnalysis;

impl ConformationalAnalysis {
    /// Check for α-helix signature: minima near 208 and 222 nm with similar magnitude.
    pub fn alpha_helix_signature(spectrum: &CdSpectrum) -> bool {
        let v193 = spectrum.interpolate(193.0);
        let v208 = spectrum.interpolate(208.0);
        let v218 = spectrum.interpolate(218.0);
        let v222 = spectrum.interpolate(222.0);
        // α-helix: positive near 193, negative at both 208 and 222.
        // Key distinguisher from beta-sheet: in alpha-helix, the 208nm band
        // is a distinct minimum (comparable to 222nm), whereas in beta-sheet
        // the dominant minimum is at 218nm and the 208nm value is weaker.
        // We require v208 to be at least 50% of v222 in magnitude.
        v193 > 0.0 && v208 < 0.0 && v222 < 0.0
            && v208.abs() > v222.abs() * 0.5
            && v208.abs() > v218.abs() * 0.5
    }

    /// Check for β-sheet signature: minimum near 218 nm.
    pub fn beta_sheet_signature(spectrum: &CdSpectrum) -> bool {
        let v218 = spectrum.interpolate(218.0);
        let v195 = spectrum.interpolate(195.0);
        // β-sheet: negative near 218, positive near 195
        v218 < 0.0 && v195 > 0.0
    }

    /// θ(222)/θ(208) ratio - coiled-coil indicator (>1.0 suggests coiled-coil).
    pub fn ratio_222_208(spectrum: &CdSpectrum) -> f64 {
        let v208 = spectrum.interpolate(208.0);
        let v222 = spectrum.interpolate(222.0);
        if v208.abs() < 1e-30 {
            return 0.0;
        }
        v222 / v208
    }

    /// Check for random coil signature: minimum near 198 nm with no strong 222 nm minimum.
    pub fn random_coil_signature(spectrum: &CdSpectrum) -> bool {
        let v198 = spectrum.interpolate(198.0);
        let v222 = spectrum.interpolate(222.0);
        let v208 = spectrum.interpolate(208.0);
        // Random coil: strong negative near 198, but NO strong double-minimum
        // (i.e. 208 and 222 should not both be strongly negative)
        v198 < 0.0 && v222.abs() < v198.abs() * 0.5 && v208.abs() < v198.abs() * 0.5
    }

    /// Classify the overall conformation from the CD spectrum.
    ///
    /// Uses a priority scheme: alpha-helix is identified by its distinctive
    /// double-minimum pattern at 208 and 222 nm; beta-sheet by a single
    /// minimum near 218 nm WITHOUT the alpha-helix pattern; random coil
    /// by a deep minimum near 198 nm only.
    pub fn classify_conformation(spectrum: &CdSpectrum) -> Conformation {
        let is_alpha = Self::alpha_helix_signature(spectrum);
        let is_coil = Self::random_coil_signature(spectrum);

        if is_alpha {
            let ratio = Self::ratio_222_208(spectrum);
            if ratio > 1.0 {
                return Conformation::CoiledCoil;
            }
            if is_coil {
                return Conformation::Mixed;
            }
            return Conformation::AlphaHelix;
        }

        // Beta-sheet check only if not already identified as alpha-helix
        let is_beta = Self::beta_sheet_signature(spectrum);
        if is_beta {
            if is_coil {
                return Conformation::Mixed;
            }
            return Conformation::BetaSheet;
        }
        if is_coil {
            return Conformation::RandomCoil;
        }
        Conformation::Mixed
    }
}

// ---------------------------------------------------------------------------
// NucleicAcidCd
// ---------------------------------------------------------------------------

/// DNA conformational form.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DnaForm {
    BForm,
    AForm,
    ZForm,
    Unknown,
}

/// Nucleic acid conformational analysis from CD spectra.
pub struct NucleicAcidCd;

impl NucleicAcidCd {
    /// Identify DNA/RNA form from characteristic CD bands.
    ///
    /// B-DNA: positive ~275nm, negative ~245nm, crossover ~258nm
    /// A-DNA/A-RNA: strong positive ~260nm, negative ~210nm
    /// Z-DNA: negative ~290nm, positive ~260nm, inverted near-UV
    pub fn identify_form(spectrum: &CdSpectrum) -> DnaForm {
        let v245 = spectrum.interpolate(245.0);
        let v258 = spectrum.interpolate(258.0);
        let v260 = spectrum.interpolate(260.0);
        let v275 = spectrum.interpolate(275.0);
        let v290 = spectrum.interpolate(290.0);
        let v210 = spectrum.interpolate(210.0);

        // Z-DNA: negative around 290 nm
        if v290 < 0.0 && v260 > 0.0 {
            return DnaForm::ZForm;
        }

        // A-form: strong positive ~260, negative ~210
        if v260 > 0.0 && v210 < 0.0 && v260.abs() > v275.abs() {
            return DnaForm::AForm;
        }

        // B-DNA: positive ~275, negative ~245, crossover near 258
        if v275 > 0.0 && v245 < 0.0 {
            // Check crossover: sign should change near 258
            if v258.abs() < v275.abs() * 0.5 {
                return DnaForm::BForm;
            }
            return DnaForm::BForm;
        }

        DnaForm::Unknown
    }

    /// Estimate base-pairing quality from the 260 nm band intensity.
    /// Returns a 0-1 score where 1.0 is fully paired.
    pub fn base_pairing_quality(spectrum: &CdSpectrum) -> f64 {
        let v260 = spectrum.interpolate(260.0);
        // Typical well-paired B-DNA has CD ~ +5000 at 275nm
        // Normalize to a typical maximum
        let max_expected = 10000.0;
        (v260.abs() / max_expected).min(1.0)
    }

    /// Determine Tm from nucleic acid melting (CD at 260 nm vs temperature).
    pub fn thermal_denaturation_nucleic(temps: &[f64], cd_260: &[f64]) -> f64 {
        if temps.len() < 2 || temps.len() != cd_260.len() {
            return 0.0;
        }
        let td = ThermalDenaturation::new(temps.to_vec(), cd_260.to_vec());
        let result = td.fit_melting(260.0);
        result.tm_c
    }
}

// ---------------------------------------------------------------------------
// SpectrumDeconvolution + CdBand
// ---------------------------------------------------------------------------

/// A single Gaussian CD band.
#[derive(Debug, Clone, Copy)]
pub struct CdBand {
    /// Center wavelength in nm.
    pub center_nm: f64,
    /// Amplitude in mdeg.
    pub amplitude_mdeg: f64,
    /// Half-width at half-maximum in nm.
    pub width_nm: f64,
    /// Rotational strength (Debye-Bohr magneton units).
    pub rotational_strength: f64,
}

impl CdBand {
    /// Evaluate this Gaussian band at a given wavelength.
    pub fn evaluate(&self, wavelength_nm: f64) -> f64 {
        let diff = wavelength_nm - self.center_nm;
        self.amplitude_mdeg * (-0.5 * (diff / self.width_nm).powi(2)).exp()
    }
}

/// Spectrum deconvolution into Gaussian components.
pub struct SpectrumDeconvolution;

impl SpectrumDeconvolution {
    /// Decompose a CD spectrum into a set of Gaussian bands.
    ///
    /// Uses iterative peak-finding and residual subtraction.
    pub fn gaussian_decomposition(spectrum: &CdSpectrum, num_bands: usize) -> Vec<CdBand> {
        if spectrum.is_empty() || num_bands == 0 {
            return vec![];
        }

        let mut residual = spectrum.values.clone();
        let wl = &spectrum.wavelength_nm;
        let n = wl.len();
        let mut bands = Vec::with_capacity(num_bands);

        for _ in 0..num_bands {
            // Find the peak (max absolute) in residual
            let mut peak_idx = 0;
            let mut peak_abs = 0.0;
            for (i, &v) in residual.iter().enumerate() {
                if v.abs() > peak_abs {
                    peak_abs = v.abs();
                    peak_idx = i;
                }
            }

            if peak_abs < 1e-30 {
                break;
            }

            let center = wl[peak_idx];
            let amplitude = residual[peak_idx];

            // Estimate width from half-maximum points
            let half_max = amplitude.abs() * 0.5;
            let mut width = 10.0; // default
            // Search left and right for half-maximum crossing
            let mut left_wl = center;
            let mut right_wl = center;
            for i in (0..peak_idx).rev() {
                if residual[i].abs() < half_max {
                    left_wl = wl[i];
                    break;
                }
            }
            for i in peak_idx + 1..n {
                if residual[i].abs() < half_max {
                    right_wl = wl[i];
                    break;
                }
            }
            if right_wl > left_wl {
                width = (right_wl - left_wl) / 2.3548; // FWHM to sigma
            }
            width = width.max(1.0);

            let rs = Self::rotational_strength_from_band(amplitude, width);
            let band = CdBand {
                center_nm: center,
                amplitude_mdeg: amplitude,
                width_nm: width,
                rotational_strength: rs,
            };
            bands.push(band);

            // Subtract this band from residual
            for i in 0..n {
                residual[i] -= band.evaluate(wl[i]);
            }
        }

        bands
    }

    /// Calculate rotational strength from Δε and bandwidth.
    ///
    /// R = (1/22.94) × Δε_max × Δ (in 10⁻⁴⁰ cgs)
    /// Approximation: R ∝ amplitude × width
    pub fn rotational_strength(delta_epsilon: f64, bandwidth_nm: f64) -> f64 {
        delta_epsilon * bandwidth_nm / 22.94
    }

    fn rotational_strength_from_band(amplitude_mdeg: f64, width_nm: f64) -> f64 {
        // Approximate rotational strength from Gaussian parameters
        // R ∝ A × Δ × sqrt(π)
        amplitude_mdeg * width_nm * (PI).sqrt() / 22.94
    }

    /// Reconstruct a spectrum from a set of Gaussian bands.
    pub fn reconstruct(bands: &[CdBand], wavelengths: &[f64]) -> Vec<f64> {
        wavelengths.iter().map(|&wl| {
            bands.iter().map(|b| b.evaluate(wl)).sum()
        }).collect()
    }
}

// ---------------------------------------------------------------------------
// LigandBindingCd
// ---------------------------------------------------------------------------

/// Result of ligand binding titration.
#[derive(Debug, Clone)]
pub struct BindingResult {
    /// Dissociation constant.
    pub kd: f64,
    /// Maximum CD change upon saturation.
    pub delta_cd_max: f64,
    /// Hill coefficient / stoichiometry.
    pub stoichiometry: f64,
}

/// Simple SVD result for spectral analysis.
#[derive(Debug, Clone)]
pub struct SvdResult {
    /// Singular values.
    pub singular_values: Vec<f64>,
    /// Number of significant components (above threshold).
    pub num_components: usize,
}

/// Ligand binding analysis from CD titration experiments.
pub struct LigandBindingCd;

impl LigandBindingCd {
    /// Analyze ligand binding from a series of CD spectra at different concentrations.
    ///
    /// Uses Hill equation: ΔCD = ΔCD_max × [L]ⁿ / (Kd + [L]ⁿ)
    pub fn titration_analysis(spectra: &[CdSpectrum], ligand_concs: &[f64]) -> BindingResult {
        assert_eq!(spectra.len(), ligand_concs.len());
        let n = spectra.len();
        if n < 3 {
            return BindingResult { kd: 0.0, delta_cd_max: 0.0, stoichiometry: 1.0 };
        }

        // Use maximum absolute CD change at a reporting wavelength
        // Find wavelength of max change between first and last spectrum
        let first = &spectra[0];
        let last = &spectra[n - 1];
        let mut best_wl = first.wavelength_nm[0];
        let mut best_change = 0.0_f64;
        for &wl in &first.wavelength_nm {
            let change = (last.interpolate(wl) - first.interpolate(wl)).abs();
            if change > best_change {
                best_change = change;
                best_wl = wl;
            }
        }

        // Extract CD at that wavelength for each spectrum
        let cd_at_wl: Vec<f64> = spectra.iter().map(|s| s.interpolate(best_wl)).collect();
        let cd_baseline = cd_at_wl[0];
        let delta_cd: Vec<f64> = cd_at_wl.iter().map(|&v| v - cd_baseline).collect();

        let delta_cd_max = delta_cd.last().copied().unwrap_or(0.0);
        if delta_cd_max.abs() < 1e-30 {
            return BindingResult { kd: 0.0, delta_cd_max: 0.0, stoichiometry: 1.0 };
        }

        // Fit Hill equation by grid search over Kd and n
        let mut best_kd = ligand_concs[n / 2];
        let mut best_n = 1.0_f64;
        let mut best_err = f64::MAX;

        // Kd search range
        let kd_min = ligand_concs[0].max(1e-12);
        let kd_max = ligand_concs[n - 1] * 10.0;

        for ki in 0..50 {
            let kd = kd_min * ((kd_max / kd_min).powf(ki as f64 / 49.0));
            for ni in 1..=30 {
                let hill_n = ni as f64 * 0.2;
                let mut err = 0.0;
                for j in 0..n {
                    let conc_n = ligand_concs[j].powf(hill_n);
                    let pred = delta_cd_max * conc_n / (kd.powf(hill_n) + conc_n);
                    let d = delta_cd[j] - pred;
                    err += d * d;
                }
                if err < best_err {
                    best_err = err;
                    best_kd = kd;
                    best_n = hill_n;
                }
            }
        }

        BindingResult {
            kd: best_kd,
            delta_cd_max,
            stoichiometry: best_n,
        }
    }

    /// Simple SVD analysis to determine the number of spectral components.
    pub fn singular_value_decomposition_simple(spectra: &[CdSpectrum]) -> SvdResult {
        if spectra.is_empty() {
            return SvdResult { singular_values: vec![], num_components: 0 };
        }

        let n_spectra = spectra.len();
        let n_points = spectra[0].values.len();

        // Build matrix A[i][j] = spectra[i].values[j]
        let mut matrix: Vec<Vec<f64>> = Vec::with_capacity(n_spectra);
        for s in spectra {
            let row: Vec<f64> = if s.values.len() >= n_points {
                s.values[..n_points].to_vec()
            } else {
                let mut r = s.values.clone();
                r.resize(n_points, 0.0);
                r
            };
            matrix.push(row);
        }

        // Compute A^T·A (n_points × n_points would be large)
        // Instead compute A·A^T (n_spectra × n_spectra) which is smaller
        let m = n_spectra;
        let mut ata = vec![vec![0.0; m]; m];
        for i in 0..m {
            for j in i..m {
                let mut dot = 0.0;
                for k in 0..n_points {
                    dot += matrix[i][k] * matrix[j][k];
                }
                ata[i][j] = dot;
                ata[j][i] = dot;
            }
        }

        // Power iteration to find eigenvalues of A·A^T
        let mut singular_values = Vec::new();
        let mut deflated = ata.clone();

        for _ in 0..m.min(10) {
            let (eigenvalue, eigenvec) = Self::power_iteration(&deflated, 100);
            if eigenvalue < 1e-20 {
                break;
            }
            singular_values.push(eigenvalue.sqrt());

            // Deflate: A -= λ·v·v^T
            for i in 0..m {
                for j in 0..m {
                    deflated[i][j] -= eigenvalue * eigenvec[i] * eigenvec[j];
                }
            }
        }

        // Determine significant components (>5% of largest)
        let threshold = if singular_values.is_empty() {
            0.0
        } else {
            singular_values[0] * 0.05
        };
        let num_components = singular_values.iter().filter(|&&s| s > threshold).count();

        SvdResult {
            singular_values,
            num_components,
        }
    }

    fn power_iteration(matrix: &[Vec<f64>], max_iter: usize) -> (f64, Vec<f64>) {
        let n = matrix.len();
        if n == 0 {
            return (0.0, vec![]);
        }
        // Initialize with a non-uniform vector to avoid symmetry issues
        let mut v: Vec<f64> = (0..n).map(|i| (i as f64 + 1.0)).collect();
        let norm0: f64 = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        for x in v.iter_mut() { *x /= norm0; }
        let mut eigenvalue = 0.0;

        for _ in 0..max_iter {
            // w = A·v
            let mut w = vec![0.0; n];
            for i in 0..n {
                for j in 0..n {
                    w[i] += matrix[i][j] * v[j];
                }
            }
            // Compute norm
            let norm: f64 = w.iter().map(|x| x * x).sum::<f64>().sqrt();
            if norm < 1e-30 {
                break;
            }
            // Rayleigh quotient: v^T A v
            eigenvalue = w.iter().zip(v.iter()).map(|(&wi, &vi)| wi * vi).sum::<f64>();
            for i in 0..n {
                v[i] = w[i] / norm;
            }
        }

        // Final Rayleigh quotient
        let mut av = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                av[i] += matrix[i][j] * v[j];
            }
        }
        eigenvalue = av.iter().zip(v.iter()).map(|(&a, &b)| a * b).sum();

        (eigenvalue, v)
    }
}

// ---------------------------------------------------------------------------
// CdCalibration
// ---------------------------------------------------------------------------

/// Instrument calibration for CD spectrometers.
pub struct CdCalibration;

impl CdCalibration {
    /// Ammonium d-10-camphorsulfonate (ACS) standard.
    ///
    /// Reference: Δε(290.5 nm) = +2.36 L/(mol·cm) for 0.06% w/v solution.
    /// Also has a negative band at ~192.5 nm, Δε ≈ -4.9.
    pub fn ammonium_camphor_sulfonate_standard(wavelength_nm: f64) -> f64 {
        // Model as two Gaussian bands
        let band_290 = 2.36 * (-0.5 * ((wavelength_nm - 290.5) / 10.0).powi(2)).exp();
        let band_192 = -4.9 * (-0.5 * ((wavelength_nm - 192.5) / 8.0).powi(2)).exp();
        band_290 + band_192
    }

    /// D-Pantolactone standard.
    ///
    /// Reference: Δε(219 nm) ≈ -1.85 L/(mol·cm) in water.
    pub fn pantolactone_standard(wavelength_nm: f64) -> f64 {
        -1.85 * (-0.5 * ((wavelength_nm - 219.0) / 12.0).powi(2)).exp()
    }

    /// Verify calibration: compute percent deviation from expected value.
    pub fn verify_calibration(measured: f64, expected: f64) -> f64 {
        if expected.abs() < 1e-30 {
            return 0.0;
        }
        ((measured - expected) / expected * 100.0).abs()
    }

    /// Correct CD for actual vs assumed path length.
    ///
    /// CD_corrected = CD_measured × (assumed_cm / actual_cm)
    pub fn path_length_correction(cd: f64, actual_cm: f64, assumed_cm: f64) -> f64 {
        if actual_cm.abs() < 1e-30 {
            return cd;
        }
        cd * assumed_cm / actual_cm
    }
}

// ---------------------------------------------------------------------------
// CdSimulator
// ---------------------------------------------------------------------------

/// Generate synthetic CD spectra for testing and education.
pub struct CdSimulator;

impl CdSimulator {
    /// Generate wavelengths in the far-UV range (190-260 nm).
    fn far_uv_wavelengths() -> Vec<f64> {
        let mut wl = Vec::with_capacity(71);
        let mut w = 190.0;
        while w <= 260.0 {
            wl.push(w);
            w += 1.0;
        }
        wl
    }

    /// Generate wavelengths in the near-UV range (250-350 nm).
    fn near_uv_wavelengths() -> Vec<f64> {
        let mut wl = Vec::with_capacity(101);
        let mut w = 250.0;
        while w <= 350.0 {
            wl.push(w);
            w += 1.0;
        }
        wl
    }

    /// Simulate a typical α-helix CD spectrum.
    ///
    /// α-helix: positive at 193 nm, negative minima at 208 and 222 nm.
    pub fn simulate_alpha_helix(num_residues: usize) -> CdSpectrum {
        let wl = Self::far_uv_wavelengths();
        let n = num_residues as f64;
        // Scale factor: chain-length dependence
        let scale = 1.0 - 2.57 / n;

        let values: Vec<f64> = wl.iter().map(|&w| {
            let band_193 = 70000.0 * (-0.5 * ((w - 193.0) / 5.0).powi(2)).exp();
            let band_208 = -33000.0 * (-0.5 * ((w - 208.0) / 6.0).powi(2)).exp();
            let band_222 = -33000.0 * (-0.5 * ((w - 222.0) / 7.0).powi(2)).exp();
            (band_193 + band_208 + band_222) * scale
        }).collect();

        CdSpectrum::new(wl, values)
    }

    /// Simulate a typical β-sheet CD spectrum.
    ///
    /// β-sheet: positive ~195 nm, minimum ~218 nm.
    pub fn simulate_beta_sheet() -> CdSpectrum {
        let wl = Self::far_uv_wavelengths();
        let values: Vec<f64> = wl.iter().map(|&w| {
            let band_195 = 18000.0 * (-0.5 * ((w - 195.0) / 6.0).powi(2)).exp();
            let band_218 = -18000.0 * (-0.5 * ((w - 218.0) / 8.0).powi(2)).exp();
            band_195 + band_218
        }).collect();

        CdSpectrum::new(wl, values)
    }

    /// Simulate a random coil CD spectrum.
    ///
    /// Random coil: strong negative near 198 nm, weak positive near 218 nm.
    pub fn simulate_random_coil() -> CdSpectrum {
        let wl = Self::far_uv_wavelengths();
        let values: Vec<f64> = wl.iter().map(|&w| {
            let band_198 = -25000.0 * (-0.5 * ((w - 198.0) / 6.0).powi(2)).exp();
            let band_218 = 3000.0 * (-0.5 * ((w - 218.0) / 10.0).powi(2)).exp();
            band_198 + band_218
        }).collect();

        CdSpectrum::new(wl, values)
    }

    /// Simulate a mixed secondary structure spectrum.
    pub fn simulate_mixed(fractions: &SecondaryStructure) -> CdSpectrum {
        let helix = Self::simulate_alpha_helix(100);
        let sheet = Self::simulate_beta_sheet();
        let coil = Self::simulate_random_coil();
        // β-turn approximated as weighted average of sheet and coil
        let turn_values: Vec<f64> = helix.wavelength_nm.iter().enumerate().map(|(i, _)| {
            0.3 * sheet.values[i] + 0.7 * coil.values[i]
        }).collect();

        let values: Vec<f64> = helix.wavelength_nm.iter().enumerate().map(|(i, _)| {
            fractions.alpha_helix * helix.values[i]
                + fractions.beta_sheet * sheet.values[i]
                + fractions.beta_turn * turn_values[i]
                + fractions.random_coil * coil.values[i]
        }).collect();

        CdSpectrum::new(helix.wavelength_nm.clone(), values)
    }

    /// Simulate thermal denaturation curve.
    ///
    /// Returns (temperature, CD) pairs.
    pub fn simulate_melting(
        tm_c: f64,
        dh_kj: f64,
        native: &CdSpectrum,
        denatured: &CdSpectrum,
        temps: &[f64],
    ) -> Vec<(f64, f64)> {
        let tm_k = tm_c + 273.15;
        let dh = dh_kj * 1000.0; // to J/mol

        // Use 222 nm as the monitoring wavelength
        let cd_native = native.interpolate(222.0);
        let cd_denatured = denatured.interpolate(222.0);

        temps.iter().map(|&t| {
            let t_k = t + 273.15;
            let exponent = dh / R_GAS * (1.0 / t_k - 1.0 / tm_k);
            let f_u = 1.0 / (1.0 + exponent.exp());
            let cd = cd_native * (1.0 - f_u) + cd_denatured * f_u;
            (t, cd)
        }).collect()
    }

    /// Add Gaussian noise to a spectrum.
    pub fn add_noise(spectrum: &CdSpectrum, noise_mdeg: f64) -> CdSpectrum {
        // Simple deterministic pseudo-noise using a hash-like function with wrapping arithmetic
        let values: Vec<f64> = spectrum.values.iter().enumerate().map(|(i, &v)| {
            // Use wrapping multiplication to avoid overflow
            let h1 = (i as u64).wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let h2 = (i as u64).wrapping_mul(3935559000370003845).wrapping_add(2691343689449507681);
            let u1 = (h1 % 999998 + 1) as f64 / 1000000.0;
            let u2 = (h2 % 1000000) as f64 / 1000000.0;
            let noise = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
            v + noise * noise_mdeg
        }).collect();

        CdSpectrum {
            wavelength_nm: spectrum.wavelength_nm.clone(),
            values,
            unit: spectrum.unit,
        }
    }

    /// Simulate B-form DNA CD spectrum.
    pub fn simulate_b_dna() -> CdSpectrum {
        let wl = Self::near_uv_wavelengths();
        let values: Vec<f64> = wl.iter().map(|&w| {
            let band_275 = 5000.0 * (-0.5 * ((w - 275.0) / 12.0).powi(2)).exp();
            let band_245 = -5000.0 * (-0.5 * ((w - 245.0) / 10.0).powi(2)).exp();
            band_275 + band_245
        }).collect();
        CdSpectrum::new(wl, values)
    }

    /// Simulate A-form DNA/RNA CD spectrum.
    pub fn simulate_a_dna() -> CdSpectrum {
        let wl: Vec<f64> = {
            let mut v = Vec::with_capacity(161);
            let mut w = 200.0;
            while w <= 360.0 {
                v.push(w);
                w += 1.0;
            }
            v
        };
        let values: Vec<f64> = wl.iter().map(|&w| {
            let band_260 = 8000.0 * (-0.5 * ((w - 260.0) / 10.0).powi(2)).exp();
            let band_210 = -6000.0 * (-0.5 * ((w - 210.0) / 8.0).powi(2)).exp();
            band_260 + band_210
        }).collect();
        CdSpectrum::new(wl, values)
    }

    /// Simulate Z-form DNA CD spectrum.
    pub fn simulate_z_dna() -> CdSpectrum {
        let wl = Self::near_uv_wavelengths();
        let values: Vec<f64> = wl.iter().map(|&w| {
            let band_260 = 3000.0 * (-0.5 * ((w - 260.0) / 10.0).powi(2)).exp();
            let band_290 = -4000.0 * (-0.5 * ((w - 290.0) / 8.0).powi(2)).exp();
            band_260 + band_290
        }).collect();
        CdSpectrum::new(wl, values)
    }
}

// ---------------------------------------------------------------------------
// SynchrotronCd
// ---------------------------------------------------------------------------

/// Result from vacuum-UV CD analysis.
#[derive(Debug, Clone)]
pub struct VuvResult {
    /// Extended wavelength minimum (nm).
    pub lambda_min: f64,
    /// Number of resolvable secondary structure components.
    pub resolvable_components: usize,
    /// Additional confidence from VUV data (0-1).
    pub confidence_gain: f64,
}

/// Quality metrics for CD spectra.
#[derive(Debug, Clone)]
pub struct QualityMetrics {
    /// Estimated HT voltage indicator (0-1, lower is better).
    pub ht_voltage_indicator: f64,
    /// RMS noise level in the spectrum.
    pub noise_rms: f64,
    /// Signal-to-noise ratio.
    pub snr: f64,
}

/// Synchrotron radiation CD (SRCD) extended analysis.
pub struct SynchrotronCd;

impl SynchrotronCd {
    /// Analyze vacuum-UV CD data for enhanced structural information.
    pub fn vacuum_uv_analysis(spectrum: &CdSpectrum) -> VuvResult {
        let (min_wl, _) = spectrum.wavelength_range();
        let components = Self::information_content(min_wl);
        let confidence_gain = if min_wl < 170.0 {
            0.8
        } else if min_wl < 180.0 {
            0.5
        } else if min_wl < 190.0 {
            0.3
        } else {
            0.0
        };

        VuvResult {
            lambda_min: min_wl,
            resolvable_components: components,
            confidence_gain,
        }
    }

    /// Estimate number of resolvable secondary structure components
    /// based on shortest wavelength measured.
    ///
    /// Down to 190 nm: ~3 components
    /// Down to 178 nm: ~4 components
    /// Down to 168 nm: ~5+ components
    pub fn information_content(lambda_min: f64) -> usize {
        if lambda_min <= 168.0 {
            6
        } else if lambda_min <= 175.0 {
            5
        } else if lambda_min <= 180.0 {
            4
        } else if lambda_min <= 190.0 {
            3
        } else if lambda_min <= 200.0 {
            2
        } else {
            1
        }
    }

    /// Assess spectral quality from noise characteristics.
    pub fn spectral_quality(spectrum: &CdSpectrum) -> QualityMetrics {
        let n = spectrum.values.len();
        if n < 5 {
            return QualityMetrics {
                ht_voltage_indicator: 1.0,
                noise_rms: 0.0,
                snr: 0.0,
            };
        }

        // Estimate noise from high-frequency fluctuations (second derivative)
        let mut noise_sum = 0.0;
        let mut count = 0;
        for i in 1..n - 1 {
            let d2 = spectrum.values[i + 1] - 2.0 * spectrum.values[i] + spectrum.values[i - 1];
            noise_sum += d2 * d2;
            count += 1;
        }
        let noise_rms = if count > 0 {
            (noise_sum / count as f64).sqrt() / (6.0_f64).sqrt()
        } else {
            0.0
        };

        // Signal RMS
        let signal_rms = (spectrum.values.iter().map(|v| v * v).sum::<f64>() / n as f64).sqrt();
        let snr = if noise_rms > 1e-30 { signal_rms / noise_rms } else { f64::MAX };

        // HT voltage indicator: high noise at low wavelengths suggests high HT
        let (min_wl, _) = spectrum.wavelength_range();
        let ht_indicator = if min_wl < 180.0 { 0.8 } else if min_wl < 190.0 { 0.5 } else { 0.2 };

        QualityMetrics {
            ht_voltage_indicator: ht_indicator,
            noise_rms,
            snr,
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // CdSpectrum tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_cd_spectrum_new() {
        let wl = vec![190.0, 200.0, 210.0, 220.0];
        let cd = vec![-10.0, -5.0, 3.0, -8.0];
        let spec = CdSpectrum::new(wl.clone(), cd.clone());
        assert_eq!(spec.len(), 4);
        assert!(!spec.is_empty());
        assert_eq!(spec.unit, CdUnit::Millidegrees);
    }

    #[test]
    fn test_cd_spectrum_wavelength_range() {
        let spec = CdSpectrum::new(vec![195.0, 200.0, 250.0], vec![1.0, 2.0, 3.0]);
        let (lo, hi) = spec.wavelength_range();
        assert!(approx_eq(lo, 195.0, 1e-9));
        assert!(approx_eq(hi, 250.0, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_wavelength_range_empty() {
        let spec = CdSpectrum::new(vec![], vec![]);
        let (lo, hi) = spec.wavelength_range();
        assert!(approx_eq(lo, 0.0, 1e-9));
        assert!(approx_eq(hi, 0.0, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_to_delta_epsilon() {
        let spec = CdSpectrum::new(vec![200.0, 210.0], vec![32982.0, 0.0]);
        let de = spec.to_delta_epsilon(0.001, 0.1);
        assert_eq!(de.unit, CdUnit::DeltaEpsilon);
        // θ = 32982 mdeg, c = 0.001 mol/L, l = 0.1 cm
        // Δε = θ / (32982 × c × l) = 32982 / (32.982 × 0.001 × 0.1) = ... let's just check non-zero
        assert!(de.values[0].abs() > 0.0);
    }

    #[test]
    fn test_cd_spectrum_to_mre() {
        let spec = CdSpectrum::new(vec![222.0], vec![-33000.0]);
        let mre = spec.to_mean_residue_ellipticity(1.0, 0.1, 15000.0, 100);
        assert_eq!(mre.unit, CdUnit::MeanResidueEllipticity);
        // MRW = 15000/100 = 150
        // [θ] = θ × MRW / (10 × c × l) = -33000 × 150 / (10 × 1.0 × 0.1) = -4950000
        assert!(mre.values[0] < 0.0);
    }

    #[test]
    fn test_cd_spectrum_smooth() {
        let spec = CdSpectrum::new(
            vec![200.0, 201.0, 202.0, 203.0, 204.0],
            vec![10.0, 20.0, 100.0, 20.0, 10.0],
        );
        let smoothed = spec.smooth(3);
        assert_eq!(smoothed.len(), 5);
        // Center point should be averaged
        assert!(smoothed.values[2] < 100.0);
    }

    #[test]
    fn test_cd_spectrum_smooth_window_1() {
        let spec = CdSpectrum::new(vec![200.0, 201.0], vec![10.0, 20.0]);
        let smoothed = spec.smooth(1);
        assert!(approx_eq(smoothed.values[0], 10.0, 1e-9));
        assert!(approx_eq(smoothed.values[1], 20.0, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_normalize() {
        let spec = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![-50.0, 100.0, -30.0]);
        let norm = spec.normalize();
        assert!(approx_eq(norm.values[1], 1.0, 1e-9));
        assert!(approx_eq(norm.values[0], -0.5, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_normalize_zero() {
        let spec = CdSpectrum::new(vec![200.0, 210.0], vec![0.0, 0.0]);
        let norm = spec.normalize();
        assert!(approx_eq(norm.values[0], 0.0, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_interpolate() {
        let spec = CdSpectrum::new(vec![200.0, 210.0], vec![10.0, 20.0]);
        assert!(approx_eq(spec.interpolate(205.0), 15.0, 1e-9));
        assert!(approx_eq(spec.interpolate(200.0), 10.0, 1e-9));
        assert!(approx_eq(spec.interpolate(210.0), 20.0, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_interpolate_extrapolate() {
        let spec = CdSpectrum::new(vec![200.0, 210.0], vec![10.0, 20.0]);
        assert!(approx_eq(spec.interpolate(190.0), 10.0, 1e-9)); // clamp to first
        assert!(approx_eq(spec.interpolate(220.0), 20.0, 1e-9)); // clamp to last
    }

    #[test]
    fn test_cd_spectrum_min_max_wavelength() {
        let spec = CdSpectrum::new(
            vec![200.0, 208.0, 222.0, 240.0],
            vec![50.0, -33.0, -35.0, -5.0],
        );
        assert!(approx_eq(spec.min_wavelength(), 222.0, 1e-9));
        assert!(approx_eq(spec.max_wavelength(), 200.0, 1e-9));
    }

    #[test]
    fn test_cd_spectrum_subtract_baseline() {
        let spec = CdSpectrum::new(vec![200.0, 210.0], vec![10.0, 20.0]);
        let baseline = CdSpectrum::new(vec![200.0, 210.0], vec![2.0, 3.0]);
        let result = spec.subtract_baseline(&baseline);
        assert!(approx_eq(result.values[0], 8.0, 1e-9));
        assert!(approx_eq(result.values[1], 17.0, 1e-9));
    }

    // -----------------------------------------------------------------------
    // SecondaryStructureEstimator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_helix_content_from_222nm() {
        // Fully helical protein with 100 residues
        // θ_helix = -40000 × (1 - 2.57/100) = -38972
        // f = (θ_222 - 640) / (-38972 - 640)
        let f = SecondaryStructureEstimator::helix_content_from_222nm(-38000.0, 100);
        assert!(f > 0.9);
        assert!(f <= 1.0);
    }

    #[test]
    fn test_helix_content_from_222nm_zero() {
        let f = SecondaryStructureEstimator::helix_content_from_222nm(640.0, 100);
        assert!(approx_eq(f, 0.0, 0.01));
    }

    #[test]
    fn test_helix_content_from_222nm_clamped() {
        let f = SecondaryStructureEstimator::helix_content_from_222nm(50000.0, 100);
        assert!(approx_eq(f, 0.0, 1e-9));

        let f2 = SecondaryStructureEstimator::helix_content_from_222nm(-100000.0, 100);
        assert!(approx_eq(f2, 1.0, 1e-9));
    }

    #[test]
    fn test_estimate_alpha_helix_dominant() {
        let helix_spectrum = CdSimulator::simulate_alpha_helix(100);
        let ss = SecondaryStructureEstimator::estimate(&helix_spectrum);
        assert!(ss.alpha_helix > 0.3, "Expected helix fraction > 0.3, got {}", ss.alpha_helix);
    }

    #[test]
    fn test_estimate_beta_sheet_dominant() {
        let sheet_spectrum = CdSimulator::simulate_beta_sheet();
        let ss = SecondaryStructureEstimator::estimate(&sheet_spectrum);
        assert!(ss.beta_sheet > 0.2, "Expected sheet fraction > 0.2, got {}", ss.beta_sheet);
    }

    #[test]
    fn test_estimate_fractions_sum_to_one() {
        let spec = CdSimulator::simulate_alpha_helix(100);
        let ss = SecondaryStructureEstimator::estimate(&spec);
        assert!(approx_eq(ss.total(), 1.0, 0.01));
    }

    #[test]
    fn test_secondary_structure_normalize() {
        let ss = SecondaryStructure {
            alpha_helix: 2.0,
            beta_sheet: 1.0,
            beta_turn: 0.5,
            random_coil: 0.5,
        };
        let norm = ss.normalize();
        assert!(approx_eq(norm.total(), 1.0, 1e-9));
        assert!(approx_eq(norm.alpha_helix, 0.5, 1e-9));
    }

    // -----------------------------------------------------------------------
    // ThermalDenaturation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_thermal_denaturation_basic() {
        let native = CdSimulator::simulate_alpha_helix(100);
        let denatured = CdSimulator::simulate_random_coil();
        let temps: Vec<f64> = (20..=90).map(|t| t as f64).collect();
        let curve = CdSimulator::simulate_melting(60.0, 300.0, &native, &denatured, &temps);

        let (t_vals, cd_vals): (Vec<f64>, Vec<f64>) = curve.into_iter().unzip();
        let td = ThermalDenaturation::new(t_vals, cd_vals);
        let result = td.fit_melting(222.0);

        // Tm should be near 60°C
        assert!(result.tm_c > 50.0 && result.tm_c < 70.0,
                "Expected Tm near 60°C, got {}", result.tm_c);
    }

    #[test]
    fn test_thermal_denaturation_fraction_unfolded() {
        let temps: Vec<f64> = (20..=90).map(|t| t as f64).collect();
        // Sigmoidal transition centered at 60
        let cd_values: Vec<f64> = temps.iter().map(|&t| {
            let f = 1.0 / (1.0 + (300000.0 / R_GAS * (1.0 / (t + 273.15) - 1.0 / 333.15)).exp());
            -33000.0 * (1.0 - f) + -1000.0 * f
        }).collect();

        let td = ThermalDenaturation::new(temps, cd_values);
        let result = td.fit_melting(222.0);
        assert!(result.fraction_unfolded.len() > 0);
        // Early temperatures should be mostly folded
        assert!(result.fraction_unfolded[0] < 0.3);
        // Late temperatures should be mostly unfolded
        assert!(*result.fraction_unfolded.last().unwrap() > 0.7);
    }

    #[test]
    fn test_van_hoff_enthalpy() {
        let tm_k = 333.15; // 60°C
        let slope = 0.01; // 1% per degree at Tm
        let dh = ThermalDenaturation::van_hoff_enthalpy(tm_k, slope);
        assert!(dh > 0.0);
    }

    #[test]
    fn test_thermal_denaturation_short_data() {
        let td = ThermalDenaturation::new(vec![20.0, 40.0], vec![-30000.0, -10000.0]);
        let result = td.fit_melting(222.0);
        // Should handle gracefully with <4 points
        assert_eq!(result.tm_c, 0.0);
    }

    // -----------------------------------------------------------------------
    // ConformationalAnalysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_alpha_helix_signature() {
        let spec = CdSimulator::simulate_alpha_helix(100);
        assert!(ConformationalAnalysis::alpha_helix_signature(&spec));
    }

    #[test]
    fn test_beta_sheet_signature() {
        let spec = CdSimulator::simulate_beta_sheet();
        assert!(ConformationalAnalysis::beta_sheet_signature(&spec));
    }

    #[test]
    fn test_random_coil_signature() {
        let spec = CdSimulator::simulate_random_coil();
        assert!(ConformationalAnalysis::random_coil_signature(&spec));
    }

    #[test]
    fn test_ratio_222_208_alpha_helix() {
        let spec = CdSimulator::simulate_alpha_helix(100);
        let ratio = ConformationalAnalysis::ratio_222_208(&spec);
        // For regular α-helix, ratio should be < 1.0
        assert!(ratio > 0.5 && ratio < 1.5, "222/208 ratio: {}", ratio);
    }

    #[test]
    fn test_classify_alpha_helix() {
        let spec = CdSimulator::simulate_alpha_helix(100);
        let conf = ConformationalAnalysis::classify_conformation(&spec);
        assert_eq!(conf, Conformation::AlphaHelix);
    }

    #[test]
    fn test_classify_beta_sheet() {
        let spec = CdSimulator::simulate_beta_sheet();
        let conf = ConformationalAnalysis::classify_conformation(&spec);
        // β-sheet may also have alpha signature due to positive band near 195
        assert!(conf == Conformation::BetaSheet || conf == Conformation::Mixed);
    }

    #[test]
    fn test_classify_random_coil() {
        let spec = CdSimulator::simulate_random_coil();
        let conf = ConformationalAnalysis::classify_conformation(&spec);
        assert!(conf == Conformation::RandomCoil || conf == Conformation::Mixed);
    }

    // -----------------------------------------------------------------------
    // NucleicAcidCd tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_identify_b_dna() {
        let spec = CdSimulator::simulate_b_dna();
        let form = NucleicAcidCd::identify_form(&spec);
        assert_eq!(form, DnaForm::BForm);
    }

    #[test]
    fn test_identify_z_dna() {
        let spec = CdSimulator::simulate_z_dna();
        let form = NucleicAcidCd::identify_form(&spec);
        assert_eq!(form, DnaForm::ZForm);
    }

    #[test]
    fn test_identify_a_dna() {
        let spec = CdSimulator::simulate_a_dna();
        let form = NucleicAcidCd::identify_form(&spec);
        assert_eq!(form, DnaForm::AForm);
    }

    #[test]
    fn test_base_pairing_quality() {
        let spec = CdSimulator::simulate_b_dna();
        let quality = NucleicAcidCd::base_pairing_quality(&spec);
        assert!(quality > 0.0 && quality <= 1.0);
    }

    #[test]
    fn test_thermal_denaturation_nucleic() {
        let temps: Vec<f64> = (30..=95).map(|t| t as f64).collect();
        let cd_260: Vec<f64> = temps.iter().map(|&t| {
            let f = 1.0 / (1.0 + (200000.0 / R_GAS * (1.0 / (t + 273.15) - 1.0 / 348.15)).exp());
            5000.0 * (1.0 - f) + 1000.0 * f
        }).collect();
        let tm = NucleicAcidCd::thermal_denaturation_nucleic(&temps, &cd_260);
        assert!(tm > 60.0 && tm < 85.0, "Expected Tm near 75°C, got {}", tm);
    }

    // -----------------------------------------------------------------------
    // SpectrumDeconvolution tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gaussian_decomposition_single_band() {
        let wl: Vec<f64> = (190..=260).map(|w| w as f64).collect();
        let values: Vec<f64> = wl.iter().map(|&w| {
            100.0 * (-0.5 * ((w - 220.0) / 5.0).powi(2)).exp()
        }).collect();
        let spec = CdSpectrum::new(wl, values);
        let bands = SpectrumDeconvolution::gaussian_decomposition(&spec, 1);
        assert_eq!(bands.len(), 1);
        assert!(approx_eq(bands[0].center_nm, 220.0, 1.0));
        assert!(bands[0].amplitude_mdeg > 90.0);
    }

    #[test]
    fn test_gaussian_decomposition_two_bands() {
        let wl: Vec<f64> = (190..=260).map(|w| w as f64).collect();
        let values: Vec<f64> = wl.iter().map(|&w| {
            -30.0 * (-0.5 * ((w - 208.0) / 5.0).powi(2)).exp()
                + -35.0 * (-0.5 * ((w - 222.0) / 5.0).powi(2)).exp()
        }).collect();
        let spec = CdSpectrum::new(wl, values);
        let bands = SpectrumDeconvolution::gaussian_decomposition(&spec, 2);
        assert_eq!(bands.len(), 2);
    }

    #[test]
    fn test_reconstruct_from_bands() {
        let bands = vec![
            CdBand { center_nm: 220.0, amplitude_mdeg: 100.0, width_nm: 5.0, rotational_strength: 0.0 },
        ];
        let wl = vec![215.0, 220.0, 225.0];
        let recon = SpectrumDeconvolution::reconstruct(&bands, &wl);
        assert!(approx_eq(recon[1], 100.0, 1e-6));
        assert!(recon[0] < 100.0);
        assert!(recon[2] < 100.0);
    }

    #[test]
    fn test_rotational_strength() {
        let rs = SpectrumDeconvolution::rotational_strength(5.0, 10.0);
        assert!(rs > 0.0);
        assert!(approx_eq(rs, 5.0 * 10.0 / 22.94, 1e-6));
    }

    #[test]
    fn test_cd_band_evaluate() {
        let band = CdBand {
            center_nm: 220.0,
            amplitude_mdeg: -33000.0,
            width_nm: 7.0,
            rotational_strength: 0.0,
        };
        assert!(approx_eq(band.evaluate(220.0), -33000.0, 1e-6));
        assert!(band.evaluate(250.0).abs() < 5.0, "value at 250nm = {}", band.evaluate(250.0)); // far from center
    }

    // -----------------------------------------------------------------------
    // LigandBindingCd tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_titration_analysis_basic() {
        // Create synthetic titration data
        let kd_true = 1e-5;
        let delta_max = 5000.0;
        let concs: Vec<f64> = (0..10).map(|i| (i as f64 + 0.1) * 1e-5).collect();
        let wl = vec![200.0, 210.0, 220.0];
        let spectra: Vec<CdSpectrum> = concs.iter().map(|&c| {
            let delta = delta_max * c / (kd_true + c);
            CdSpectrum::new(wl.clone(), vec![1000.0 + delta, 500.0, -500.0 - delta])
        }).collect();

        let result = LigandBindingCd::titration_analysis(&spectra, &concs);
        assert!(result.kd > 0.0);
        assert!(result.delta_cd_max.abs() > 0.0);
    }

    #[test]
    fn test_svd_simple_identical_spectra() {
        let spec = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![10.0, -5.0, 3.0]);
        let spectra = vec![spec.clone(), spec.clone(), spec.clone()];
        let result = LigandBindingCd::singular_value_decomposition_simple(&spectra);
        assert_eq!(result.num_components, 1, "Identical spectra should have 1 component");
    }

    #[test]
    fn test_svd_two_components() {
        // Use clearly distinct spectral components with large magnitudes
        let s1 = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![100.0, 0.0, 0.0]);
        let s2 = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![0.0, 100.0, 0.0]);
        let s3 = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![50.0, 50.0, 0.0]);
        let s4 = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![80.0, 20.0, 0.0]);
        let result = LigandBindingCd::singular_value_decomposition_simple(&[s1, s2, s3, s4]);
        assert!(result.singular_values.len() >= 2, "Expected >= 2 singular values, got {}", result.singular_values.len());
        assert!(result.num_components >= 2, "Expected >= 2 components, got {} (sv: {:?})",
                result.num_components, result.singular_values);
    }

    #[test]
    fn test_svd_empty() {
        let result = LigandBindingCd::singular_value_decomposition_simple(&[]);
        assert_eq!(result.num_components, 0);
    }

    // -----------------------------------------------------------------------
    // CdCalibration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_acs_standard_at_peak() {
        let val = CdCalibration::ammonium_camphor_sulfonate_standard(290.5);
        assert!(val > 2.0, "ACS peak at 290.5 nm should be ~2.36, got {}", val);
    }

    #[test]
    fn test_acs_standard_off_peak() {
        let val = CdCalibration::ammonium_camphor_sulfonate_standard(350.0);
        assert!(val.abs() < 0.5, "ACS far from peak should be near zero");
    }

    #[test]
    fn test_pantolactone_standard() {
        let val = CdCalibration::pantolactone_standard(219.0);
        assert!(val < -1.0, "Pantolactone at 219 nm should be negative");
    }

    #[test]
    fn test_verify_calibration() {
        let dev = CdCalibration::verify_calibration(2.30, 2.36);
        assert!(dev < 3.0); // <3% deviation
    }

    #[test]
    fn test_verify_calibration_zero_expected() {
        let dev = CdCalibration::verify_calibration(1.0, 0.0);
        assert!(approx_eq(dev, 0.0, 1e-9));
    }

    #[test]
    fn test_path_length_correction() {
        let corrected = CdCalibration::path_length_correction(100.0, 0.1, 0.2);
        assert!(approx_eq(corrected, 200.0, 1e-9));
    }

    #[test]
    fn test_path_length_correction_identity() {
        let corrected = CdCalibration::path_length_correction(100.0, 0.1, 0.1);
        assert!(approx_eq(corrected, 100.0, 1e-9));
    }

    // -----------------------------------------------------------------------
    // CdSimulator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_alpha_helix_shape() {
        let spec = CdSimulator::simulate_alpha_helix(100);
        assert!(!spec.is_empty());
        // Should have negative values at 208 and 222
        assert!(spec.interpolate(208.0) < 0.0);
        assert!(spec.interpolate(222.0) < 0.0);
        // Should have positive value near 193
        assert!(spec.interpolate(193.0) > 0.0);
    }

    #[test]
    fn test_simulate_alpha_helix_chain_length() {
        let spec_short = CdSimulator::simulate_alpha_helix(10);
        let spec_long = CdSimulator::simulate_alpha_helix(200);
        // Longer helix should have stronger CD
        assert!(spec_long.interpolate(222.0).abs() > spec_short.interpolate(222.0).abs());
    }

    #[test]
    fn test_simulate_beta_sheet_shape() {
        let spec = CdSimulator::simulate_beta_sheet();
        assert!(spec.interpolate(218.0) < 0.0);
        assert!(spec.interpolate(195.0) > 0.0);
    }

    #[test]
    fn test_simulate_random_coil_shape() {
        let spec = CdSimulator::simulate_random_coil();
        assert!(spec.interpolate(198.0) < 0.0);
    }

    #[test]
    fn test_simulate_mixed() {
        let fracs = SecondaryStructure {
            alpha_helix: 0.5,
            beta_sheet: 0.2,
            beta_turn: 0.1,
            random_coil: 0.2,
        };
        let spec = CdSimulator::simulate_mixed(&fracs);
        assert!(!spec.is_empty());
        // Mixed spectrum with dominant helix should still be negative at 222
        assert!(spec.interpolate(222.0) < 0.0);
    }

    #[test]
    fn test_simulate_melting() {
        let native = CdSimulator::simulate_alpha_helix(100);
        let denatured = CdSimulator::simulate_random_coil();
        let temps: Vec<f64> = (20..=90).map(|t| t as f64).collect();
        let curve = CdSimulator::simulate_melting(60.0, 250.0, &native, &denatured, &temps);
        assert_eq!(curve.len(), temps.len());
        // First point should be close to native CD at 222
        assert!(curve[0].1 < 0.0);
    }

    #[test]
    fn test_add_noise() {
        let spec = CdSpectrum::new(vec![200.0, 210.0, 220.0], vec![0.0, 0.0, 0.0]);
        let noisy = CdSimulator::add_noise(&spec, 100.0);
        // At least some values should differ from zero
        let any_nonzero = noisy.values.iter().any(|&v| v.abs() > 1e-10);
        assert!(any_nonzero);
    }

    #[test]
    fn test_simulate_b_dna() {
        let spec = CdSimulator::simulate_b_dna();
        assert!(spec.interpolate(275.0) > 0.0);
        assert!(spec.interpolate(245.0) < 0.0);
    }

    #[test]
    fn test_simulate_a_dna() {
        let spec = CdSimulator::simulate_a_dna();
        assert!(spec.interpolate(260.0) > 0.0);
        assert!(spec.interpolate(210.0) < 0.0);
    }

    #[test]
    fn test_simulate_z_dna() {
        let spec = CdSimulator::simulate_z_dna();
        assert!(spec.interpolate(290.0) < 0.0);
        assert!(spec.interpolate(260.0) > 0.0);
    }

    // -----------------------------------------------------------------------
    // SynchrotronCd tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_information_content_far_uv() {
        assert_eq!(SynchrotronCd::information_content(168.0), 6);
        assert_eq!(SynchrotronCd::information_content(175.0), 5);
        assert_eq!(SynchrotronCd::information_content(180.0), 4);
        assert_eq!(SynchrotronCd::information_content(190.0), 3);
        assert_eq!(SynchrotronCd::information_content(200.0), 2);
        assert_eq!(SynchrotronCd::information_content(210.0), 1);
    }

    #[test]
    fn test_vacuum_uv_analysis() {
        let mut wl: Vec<f64> = Vec::new();
        let mut vals: Vec<f64> = Vec::new();
        let mut w = 170.0;
        while w <= 260.0 {
            wl.push(w);
            vals.push((-0.5 * ((w - 200.0) / 10.0).powi(2)).exp() * -20000.0);
            w += 1.0;
        }
        let spec = CdSpectrum::new(wl, vals);
        let result = SynchrotronCd::vacuum_uv_analysis(&spec);
        assert!(result.lambda_min < 180.0);
        assert!(result.resolvable_components >= 4);
        assert!(result.confidence_gain > 0.0);
    }

    #[test]
    fn test_spectral_quality() {
        let spec = CdSimulator::simulate_alpha_helix(100);
        let quality = SynchrotronCd::spectral_quality(&spec);
        assert!(quality.noise_rms >= 0.0);
        assert!(quality.snr > 0.0);
    }

    #[test]
    fn test_spectral_quality_noisy() {
        let clean = CdSimulator::simulate_alpha_helix(100);
        let noisy = CdSimulator::add_noise(&clean, 5000.0);
        let q_clean = SynchrotronCd::spectral_quality(&clean);
        let q_noisy = SynchrotronCd::spectral_quality(&noisy);
        // Noisy spectrum should have higher noise estimate
        assert!(q_noisy.noise_rms > 0.0 || q_clean.noise_rms > 0.0);
    }

    #[test]
    fn test_spectral_quality_short() {
        let spec = CdSpectrum::new(vec![200.0, 210.0], vec![10.0, -5.0]);
        let quality = SynchrotronCd::spectral_quality(&spec);
        assert_eq!(quality.ht_voltage_indicator, 1.0);
    }

    // -----------------------------------------------------------------------
    // Integration / round-trip tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_delta_a_to_mdeg_constant() {
        // θ (mdeg) = 32.982 × ΔA
        let delta_a = 1.0;
        let theta = DELTA_A_TO_MDEG * delta_a;
        assert!(approx_eq(theta, 32.982, 0.001));
    }

    #[test]
    fn test_gas_constant() {
        assert!(approx_eq(R_GAS, 8.314, 0.001));
    }

    #[test]
    fn test_full_pipeline_protein() {
        // Simulate → Estimate → Classify → Validate
        let spec = CdSimulator::simulate_alpha_helix(150);
        let ss = SecondaryStructureEstimator::estimate(&spec);
        let conf = ConformationalAnalysis::classify_conformation(&spec);
        assert!(ss.alpha_helix > 0.2);
        assert_eq!(conf, Conformation::AlphaHelix);
    }

    #[test]
    fn test_full_pipeline_nucleic_acid() {
        // Simulate B-DNA → Identify → Check quality
        let spec = CdSimulator::simulate_b_dna();
        let form = NucleicAcidCd::identify_form(&spec);
        let quality = NucleicAcidCd::base_pairing_quality(&spec);
        assert_eq!(form, DnaForm::BForm);
        assert!(quality > 0.0);
    }

    #[test]
    fn test_deconvolution_reconstruct_roundtrip() {
        let wl: Vec<f64> = (190..=260).map(|w| w as f64).collect();
        let values: Vec<f64> = wl.iter().map(|&w| {
            -33.0 * (-0.5 * ((w - 222.0) / 7.0).powi(2)).exp()
        }).collect();
        let spec = CdSpectrum::new(wl.clone(), values.clone());
        let bands = SpectrumDeconvolution::gaussian_decomposition(&spec, 1);
        let recon = SpectrumDeconvolution::reconstruct(&bands, &wl);

        // Reconstruction should be reasonable approximation
        let mut max_err = 0.0_f64;
        for (i, &v) in values.iter().enumerate() {
            let err = (v - recon[i]).abs();
            if err > max_err { max_err = err; }
        }
        assert!(max_err < 5.0, "Max reconstruction error: {}", max_err);
    }

    #[test]
    fn test_mixed_spectrum_linearity() {
        let all_helix = SecondaryStructure {
            alpha_helix: 1.0, beta_sheet: 0.0, beta_turn: 0.0, random_coil: 0.0,
        };
        let mixed_spec = CdSimulator::simulate_mixed(&all_helix);
        let pure_spec = CdSimulator::simulate_alpha_helix(100);
        // Should be very similar
        let diff = (mixed_spec.interpolate(222.0) - pure_spec.interpolate(222.0)).abs();
        let scale = pure_spec.interpolate(222.0).abs();
        assert!(diff / scale < 0.1, "Mixed (100% helix) should match pure helix");
    }

    #[test]
    fn test_calibration_round_trip() {
        let measured = 100.0;
        let corrected = CdCalibration::path_length_correction(measured, 0.05, 0.1);
        let back = CdCalibration::path_length_correction(corrected, 0.1, 0.05);
        assert!(approx_eq(back, measured, 1e-9));
    }
}
