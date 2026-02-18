//! ICP-OES (Inductively Coupled Plasma Optical Emission Spectrometry) signal processing
//! for multi-element quantitative analysis.
//!
//! This module implements the complete ICP-OES analytical pipeline:
//! - Plasma modeling (Boltzmann distribution, Saha ionization)
//! - Spectral line database with emission line presets
//! - Multi-point calibration with internal standard correction
//! - Spectral interference correction (IEC factors, background correction)
//! - CCD/CID detector modeling (dark current, flat-field, pixel mapping)
//! - Nebulizer efficiency and sample introduction modeling
//! - Signal processing (peak finding, Gaussian/Voigt fitting, deconvolution)
//! - Method detection limits (MDL, IDL, PQL)
//! - Quality control (ICV/CCV, spike recovery, duplicate RPD)

use std::f64::consts::PI;

// Physical constants
const BOLTZMANN_EV: f64 = 8.617_333_262e-5; // eV/K
const PLANCK: f64 = 6.626_070_15e-34; // J*s
const ELECTRON_MASS: f64 = 9.109_383_7015e-31; // kg
const SPEED_OF_LIGHT: f64 = 2.997_924_58e8; // m/s
const EV_TO_JOULE: f64 = 1.602_176_634e-19; // J/eV

// ─── Helper functions ───

/// Convert wavelength in nm to photon energy in eV.
pub fn wavelength_to_energy(nm: f64) -> f64 {
    // E = hc / lambda
    let lambda_m = nm * 1e-9;
    (PLANCK * SPEED_OF_LIGHT / lambda_m) / EV_TO_JOULE
}

/// Boltzmann population for an upper level.
/// Returns fractional population N_u / N_total.
pub fn boltzmann_population(g_u: f64, e_u_ev: f64, temp_k: f64, partition: f64) -> f64 {
    if partition <= 0.0 || temp_k <= 0.0 {
        return 0.0;
    }
    (g_u / partition) * (-e_u_ev / (BOLTZMANN_EV * temp_k)).exp()
}

/// Saha ionization ratio N_{i+1} * n_e / N_i.
/// Returns the ratio for given temperature, electron density, and ionization potential.
pub fn saha_ionization_ratio(temp_k: f64, ne: f64, ip_ev: f64) -> f64 {
    if temp_k <= 0.0 || ne <= 0.0 {
        return 0.0;
    }
    let kt = BOLTZMANN_EV * temp_k; // eV
    let kt_j = kt * EV_TO_JOULE; // Joules
    // Saha: (N_{i+1} * n_e) / N_i = (2 / lambda_dB^3) * (Z_{i+1}/Z_i) * exp(-IP/kT)
    // Simplified with partition ratio ~ 2 (often used for single-species estimates)
    let lambda_db = PLANCK / (2.0 * PI * ELECTRON_MASS * kt_j).sqrt();
    let prefactor = 2.0 / (lambda_db * lambda_db * lambda_db);
    // Convert ne from cm^-3 to m^-3 for SI
    let ne_m3 = ne * 1e6;
    (prefactor / ne_m3) * (-ip_ev / kt).exp()
}

// ─── Ionization state ───

/// Ionization state of an element (I = neutral, II = singly ionized, etc.)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IonizationState {
    I,
    II,
    III,
}

impl IonizationState {
    /// Numeric charge (0 for neutral, 1 for singly ionized, etc.)
    pub fn charge(&self) -> u8 {
        match self {
            IonizationState::I => 0,
            IonizationState::II => 1,
            IonizationState::III => 2,
        }
    }
}

// ─── Spectral line database ───

/// A single emission line entry.
#[derive(Debug, Clone)]
pub struct SpectralLine {
    /// Wavelength in nm.
    pub wavelength_nm: f64,
    /// Element symbol.
    pub element: String,
    /// Ionization state.
    pub ionization: IonizationState,
    /// Einstein transition probability A_ki (s^-1).
    pub transition_prob: f64,
    /// Upper energy level in eV.
    pub upper_energy_ev: f64,
    /// Statistical weight of the upper level.
    pub g_upper: f64,
}

impl SpectralLine {
    pub fn new(
        wavelength_nm: f64,
        element: &str,
        ionization: IonizationState,
        transition_prob: f64,
        upper_energy_ev: f64,
        g_upper: f64,
    ) -> Self {
        Self {
            wavelength_nm,
            element: element.to_string(),
            ionization,
            transition_prob,
            upper_energy_ev,
            g_upper,
        }
    }
}

/// Standard ICP-OES emission line presets.
pub fn preset_lines() -> Vec<SpectralLine> {
    vec![
        SpectralLine::new(393.366, "Ca", IonizationState::II, 1.47e8, 3.151, 4.0),
        SpectralLine::new(259.940, "Fe", IonizationState::II, 2.20e8, 4.767, 10.0),
        SpectralLine::new(213.856, "Zn", IonizationState::I, 7.14e8, 5.796, 3.0),
        SpectralLine::new(324.754, "Cu", IonizationState::I, 1.39e8, 3.817, 4.0),
        SpectralLine::new(257.610, "Mn", IonizationState::II, 2.60e8, 4.812, 7.0),
        SpectralLine::new(267.716, "Cr", IonizationState::II, 2.08e8, 4.629, 8.0),
        SpectralLine::new(220.353, "Pb", IonizationState::II, 1.2e8, 5.624, 2.0),
        SpectralLine::new(193.696, "As", IonizationState::I, 2.0e8, 6.400, 4.0),
        // Internal standards
        SpectralLine::new(371.029, "Y", IonizationState::II, 1.3e8, 3.341, 8.0),
        SpectralLine::new(361.383, "Sc", IonizationState::II, 1.17e8, 3.430, 9.0),
    ]
}

// ─── Plasma model ───

/// ICP plasma model for emission intensity calculations.
#[derive(Debug, Clone)]
pub struct PlasmaModel {
    /// Electron temperature in Kelvin (typical 6000-8000 K).
    pub electron_temp_k: f64,
    /// Electron density in cm^-3 (typical ~1e15).
    pub electron_density: f64,
    /// Partition function estimate (simplified).
    pub partition_function: f64,
}

impl PlasmaModel {
    pub fn new(electron_temp_k: f64, electron_density: f64) -> Self {
        Self {
            electron_temp_k,
            electron_density,
            partition_function: 1.0,
        }
    }

    /// Default ICP argon plasma conditions.
    pub fn default_icp() -> Self {
        Self {
            electron_temp_k: 7000.0,
            electron_density: 1e15,
            partition_function: 1.0,
        }
    }

    /// Compute relative emission intensity for a given spectral line.
    /// I ~ g_u * A_ki * exp(-E_u / kT) / Z
    pub fn emission_intensity(&self, line: &SpectralLine) -> f64 {
        let pop = boltzmann_population(
            line.g_upper,
            line.upper_energy_ev,
            self.electron_temp_k,
            self.partition_function,
        );
        pop * line.transition_prob
    }

    /// Saha ionization ratio for a given ionization potential.
    pub fn ionization_ratio(&self, ip_ev: f64) -> f64 {
        saha_ionization_ratio(self.electron_temp_k, self.electron_density, ip_ev)
    }

    /// Estimate the fraction of atoms in the desired ionization state.
    /// For ion lines (state II), fraction = ratio / (1 + ratio).
    /// For atom lines (state I), fraction = 1 / (1 + ratio).
    pub fn ionization_fraction(&self, state: IonizationState, ip_ev: f64) -> f64 {
        let ratio = self.ionization_ratio(ip_ev);
        match state {
            IonizationState::I => 1.0 / (1.0 + ratio),
            IonizationState::II => ratio / (1.0 + ratio),
            IonizationState::III => {
                // Simplified: assume negligible III unless very high temp
                0.0
            }
        }
    }
}

impl Default for PlasmaModel {
    fn default() -> Self {
        Self::default_icp()
    }
}

// ─── Calibration system ───

/// A single calibration point (concentration, measured intensity).
#[derive(Debug, Clone, Copy)]
pub struct CalibrationPoint {
    pub concentration: f64,
    pub intensity: f64,
}

/// Calibration curve for one analyte, with optional internal standard correction.
#[derive(Debug, Clone)]
pub struct CalibrationCurve {
    pub element: String,
    pub wavelength_nm: f64,
    pub points: Vec<CalibrationPoint>,
    /// Slope of the linear fit.
    pub slope: f64,
    /// Intercept of the linear fit.
    pub intercept: f64,
    /// R-squared correlation coefficient.
    pub r_squared: f64,
}

impl CalibrationCurve {
    pub fn new(element: &str, wavelength_nm: f64) -> Self {
        Self {
            element: element.to_string(),
            wavelength_nm,
            points: Vec::new(),
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
        }
    }

    /// Add a calibration point.
    pub fn add_point(&mut self, concentration: f64, intensity: f64) {
        self.points.push(CalibrationPoint {
            concentration,
            intensity,
        });
    }

    /// Fit a linear calibration curve (least-squares).
    pub fn fit(&mut self) -> bool {
        let n = self.points.len();
        if n < 2 {
            return false;
        }
        let nf = n as f64;
        let sum_x: f64 = self.points.iter().map(|p| p.concentration).sum();
        let sum_y: f64 = self.points.iter().map(|p| p.intensity).sum();
        let sum_xy: f64 = self
            .points
            .iter()
            .map(|p| p.concentration * p.intensity)
            .sum();
        let sum_x2: f64 = self
            .points
            .iter()
            .map(|p| p.concentration * p.concentration)
            .sum();

        let denom = nf * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return false;
        }
        self.slope = (nf * sum_xy - sum_x * sum_y) / denom;
        self.intercept = (sum_y - self.slope * sum_x) / nf;

        // R-squared
        let y_mean = sum_y / nf;
        let ss_tot: f64 = self.points.iter().map(|p| (p.intensity - y_mean).powi(2)).sum();
        let ss_res: f64 = self
            .points
            .iter()
            .map(|p| {
                let pred = self.slope * p.concentration + self.intercept;
                (p.intensity - pred).powi(2)
            })
            .sum();
        self.r_squared = if ss_tot > 0.0 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };
        true
    }

    /// Compute concentration from measured intensity.
    pub fn concentration_from_intensity(&self, intensity: f64) -> f64 {
        if self.slope.abs() < 1e-30 {
            return 0.0;
        }
        (intensity - self.intercept) / self.slope
    }

    /// Check if calibration curve meets minimum R-squared criterion.
    pub fn is_valid(&self, min_r_squared: f64) -> bool {
        self.r_squared >= min_r_squared
    }
}

/// Multi-element calibration system with internal standard support.
#[derive(Debug, Clone)]
pub struct CalibrationSystem {
    pub curves: Vec<CalibrationCurve>,
    /// Internal standard element wavelength (e.g., Y 371.029 nm).
    pub internal_standard_wavelength: Option<f64>,
}

impl CalibrationSystem {
    pub fn new() -> Self {
        Self {
            curves: Vec::new(),
            internal_standard_wavelength: None,
        }
    }

    /// Set the internal standard wavelength (e.g., Y 371.029 nm or Sc 361.383 nm).
    pub fn set_internal_standard(&mut self, wavelength_nm: f64) {
        self.internal_standard_wavelength = Some(wavelength_nm);
    }

    /// Add a calibration curve.
    pub fn add_curve(&mut self, curve: CalibrationCurve) {
        self.curves.push(curve);
    }

    /// Find calibration curve by element and wavelength.
    pub fn find_curve(&self, element: &str, wavelength_nm: f64) -> Option<&CalibrationCurve> {
        self.curves.iter().find(|c| {
            c.element == element && (c.wavelength_nm - wavelength_nm).abs() < 0.01
        })
    }

    /// Apply internal standard correction: divide analyte intensity by IS intensity.
    pub fn correct_with_internal_standard(
        &self,
        analyte_intensity: f64,
        is_intensity: f64,
    ) -> f64 {
        if is_intensity.abs() < 1e-30 {
            return analyte_intensity;
        }
        analyte_intensity / is_intensity
    }

    /// Fit all calibration curves.
    pub fn fit_all(&mut self) {
        for curve in &mut self.curves {
            curve.fit();
        }
    }
}

impl Default for CalibrationSystem {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Spectral interference correction ───

/// Inter-element correction factor for spectral overlap.
#[derive(Debug, Clone)]
pub struct IecFactor {
    /// Interfering element.
    pub interferent: String,
    /// Target analyte.
    pub analyte: String,
    /// Correction coefficient (intensity ratio).
    pub coefficient: f64,
}

/// Spectral interference correction system.
#[derive(Debug, Clone)]
pub struct SpectralInterference {
    /// IEC correction factors.
    pub iec_factors: Vec<IecFactor>,
}

impl SpectralInterference {
    pub fn new() -> Self {
        Self {
            iec_factors: Vec::new(),
        }
    }

    /// Add an inter-element correction factor.
    pub fn add_iec(&mut self, interferent: &str, analyte: &str, coefficient: f64) {
        self.iec_factors.push(IecFactor {
            interferent: interferent.to_string(),
            analyte: analyte.to_string(),
            coefficient,
        });
    }

    /// Apply IEC corrections: corrected = measured - sum(coeff_i * interferent_i_intensity).
    pub fn correct_intensity(
        &self,
        analyte: &str,
        measured_intensity: f64,
        interferent_intensities: &[(&str, f64)],
    ) -> f64 {
        let mut correction = 0.0;
        for factor in &self.iec_factors {
            if factor.analyte == analyte {
                for &(elem, intens) in interferent_intensities {
                    if elem == factor.interferent {
                        correction += factor.coefficient * intens;
                    }
                }
            }
        }
        (measured_intensity - correction).max(0.0)
    }

    /// Polynomial background correction.
    /// Fits a polynomial to off-peak points and subtracts from signal.
    pub fn background_correction_polynomial(
        wavelengths: &[f64],
        intensities: &[f64],
        bg_indices: &[usize],
        order: usize,
    ) -> Vec<f64> {
        if bg_indices.len() < order + 1 || wavelengths.len() != intensities.len() {
            return intensities.to_vec();
        }

        // Collect background points
        let bg_x: Vec<f64> = bg_indices.iter().map(|&i| wavelengths[i]).collect();
        let bg_y: Vec<f64> = bg_indices.iter().map(|&i| intensities[i]).collect();

        // Fit polynomial of given order using least squares
        let coeffs = poly_fit(&bg_x, &bg_y, order);

        // Subtract background
        wavelengths
            .iter()
            .zip(intensities.iter())
            .map(|(&w, &intens)| {
                let bg = poly_eval(&coeffs, w);
                (intens - bg).max(0.0)
            })
            .collect()
    }

    /// Off-peak background correction using two flanking points.
    pub fn off_peak_correction(
        signal_intensity: f64,
        left_bg: f64,
        right_bg: f64,
    ) -> f64 {
        let bg = (left_bg + right_bg) / 2.0;
        (signal_intensity - bg).max(0.0)
    }
}

impl Default for SpectralInterference {
    fn default() -> Self {
        Self::new()
    }
}

/// Polynomial fit (least-squares) - returns coefficients [a0, a1, ..., a_order].
fn poly_fit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let m = order + 1;
    if n < m {
        return vec![0.0; m];
    }

    // Build normal equations A^T A c = A^T y
    let mut ata = vec![0.0; m * m];
    let mut aty = vec![0.0; m];

    for i in 0..n {
        let mut xi_powers = vec![1.0; 2 * order + 1];
        for j in 1..xi_powers.len() {
            xi_powers[j] = xi_powers[j - 1] * x[i];
        }
        for r in 0..m {
            for c in 0..m {
                ata[r * m + c] += xi_powers[r + c];
            }
            aty[r] += xi_powers[r] * y[i];
        }
    }

    // Solve via Gaussian elimination with partial pivoting
    gauss_solve(&ata, &aty, m)
}

/// Evaluate polynomial at point x.
fn poly_eval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xp = 1.0;
    for &c in coeffs {
        result += c * xp;
        xp *= x;
    }
    result
}

/// Gaussian elimination with partial pivoting.
fn gauss_solve(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut aug = vec![0.0; n * (n + 1)];
    for r in 0..n {
        for c in 0..n {
            aug[r * (n + 1) + c] = a[r * n + c];
        }
        aug[r * (n + 1) + n] = b[r];
    }

    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col * (n + 1) + col].abs();
        for row in (col + 1)..n {
            let v = aug[row * (n + 1) + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_row != col {
            for c in 0..=n {
                let tmp = aug[col * (n + 1) + c];
                aug[col * (n + 1) + c] = aug[max_row * (n + 1) + c];
                aug[max_row * (n + 1) + c] = tmp;
            }
        }

        let pivot = aug[col * (n + 1) + col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for row in (col + 1)..n {
            let factor = aug[row * (n + 1) + col] / pivot;
            for c in col..=n {
                aug[row * (n + 1) + c] -= factor * aug[col * (n + 1) + c];
            }
        }
    }

    // Back-substitution
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let pivot = aug[row * (n + 1) + row];
        if pivot.abs() < 1e-30 {
            continue;
        }
        let mut s = aug[row * (n + 1) + n];
        for c in (row + 1)..n {
            s -= aug[row * (n + 1) + c] * x[c];
        }
        x[row] = s / pivot;
    }
    x
}

// ─── Detector model ───

/// CCD/CID detector model for ICP-OES.
#[derive(Debug, Clone)]
pub struct DetectorModel {
    /// Number of pixels.
    pub num_pixels: usize,
    /// Wavelength range: (start_nm, end_nm).
    pub wavelength_range: (f64, f64),
    /// Dark current counts per pixel per second.
    pub dark_current_rate: f64,
    /// Flat-field correction factors (per-pixel relative sensitivity).
    pub flat_field: Vec<f64>,
    /// Integration time in seconds.
    pub integration_time: f64,
}

impl DetectorModel {
    pub fn new(num_pixels: usize, wl_start: f64, wl_end: f64) -> Self {
        Self {
            num_pixels,
            wavelength_range: (wl_start, wl_end),
            dark_current_rate: 10.0, // counts/pixel/s
            flat_field: vec![1.0; num_pixels],
            integration_time: 1.0,
        }
    }

    /// Map pixel index to wavelength (linear dispersion).
    pub fn pixel_to_wavelength(&self, pixel: usize) -> f64 {
        let (start, end) = self.wavelength_range;
        if self.num_pixels <= 1 {
            return start;
        }
        start + (pixel as f64) * (end - start) / (self.num_pixels - 1) as f64
    }

    /// Map wavelength to fractional pixel position.
    pub fn wavelength_to_pixel(&self, wavelength_nm: f64) -> f64 {
        let (start, end) = self.wavelength_range;
        if (end - start).abs() < 1e-10 {
            return 0.0;
        }
        (wavelength_nm - start) / (end - start) * (self.num_pixels - 1) as f64
    }

    /// Spectral resolution (nm per pixel).
    pub fn spectral_resolution(&self) -> f64 {
        let (start, end) = self.wavelength_range;
        if self.num_pixels <= 1 {
            return end - start;
        }
        (end - start) / (self.num_pixels - 1) as f64
    }

    /// Subtract dark current from raw counts.
    pub fn dark_current_subtract(&self, raw_counts: &[f64]) -> Vec<f64> {
        let dark = self.dark_current_rate * self.integration_time;
        raw_counts.iter().map(|&c| (c - dark).max(0.0)).collect()
    }

    /// Apply flat-field correction.
    pub fn flat_field_correct(&self, counts: &[f64]) -> Vec<f64> {
        counts
            .iter()
            .zip(self.flat_field.iter())
            .map(|(&c, &ff)| if ff > 0.0 { c / ff } else { c })
            .collect()
    }

    /// Set flat-field from a uniformly illuminated frame.
    pub fn set_flat_field_from_uniform(&mut self, uniform_frame: &[f64]) {
        if uniform_frame.len() != self.num_pixels {
            return;
        }
        let mean: f64 = uniform_frame.iter().sum::<f64>() / self.num_pixels as f64;
        if mean.abs() < 1e-30 {
            return;
        }
        self.flat_field = uniform_frame.iter().map(|&v| v / mean).collect();
    }

    /// Bin adjacent pixels (sum groups of `bin_size`).
    pub fn pixel_binning(counts: &[f64], bin_size: usize) -> Vec<f64> {
        if bin_size == 0 {
            return counts.to_vec();
        }
        counts
            .chunks(bin_size)
            .map(|chunk| chunk.iter().sum())
            .collect()
    }

    /// Full detector pipeline: dark subtract -> flat-field correct.
    pub fn process_frame(&self, raw_counts: &[f64]) -> Vec<f64> {
        let dark_sub = self.dark_current_subtract(raw_counts);
        self.flat_field_correct(&dark_sub)
    }
}

// ─── Nebulizer efficiency ───

/// Nebulizer / sample introduction model.
#[derive(Debug, Clone)]
pub struct NebulizerEfficiency {
    /// Sample uptake rate in mL/min.
    pub uptake_rate_ml_min: f64,
    /// Transport efficiency (fraction, typically 0.01-0.05).
    pub transport_efficiency: f64,
    /// Mean droplet diameter in micrometers.
    pub mean_droplet_um: f64,
}

impl NebulizerEfficiency {
    pub fn new(uptake_rate: f64, transport_eff: f64) -> Self {
        Self {
            uptake_rate_ml_min: uptake_rate,
            transport_efficiency: transport_eff.clamp(0.0, 1.0),
            mean_droplet_um: 10.0,
        }
    }

    /// Default concentric nebulizer parameters.
    pub fn concentric_default() -> Self {
        Self {
            uptake_rate_ml_min: 1.0,
            transport_efficiency: 0.02,
            mean_droplet_um: 10.0,
        }
    }

    /// Analyte mass delivered to plasma per second (in micrograms).
    /// concentration_ppm is in mg/L (= ppm for aqueous solutions).
    pub fn analyte_delivery_rate(&self, concentration_ppm: f64) -> f64 {
        // uptake mL/min -> mL/s
        let uptake_ml_s = self.uptake_rate_ml_min / 60.0;
        // mass rate = conc (mg/L) * uptake (mL/s) * 1e-3 (L/mL) * transport_eff
        // = mg/s, convert to ug/s * 1000
        concentration_ppm * uptake_ml_s * 1e-3 * self.transport_efficiency * 1000.0
    }

    /// Nukiyama-Tanasawa droplet size distribution.
    /// Returns probability density at diameter d_um.
    pub fn droplet_size_pdf(&self, d_um: f64) -> f64 {
        if d_um <= 0.0 || self.mean_droplet_um <= 0.0 {
            return 0.0;
        }
        // Simplified Nukiyama-Tanasawa: f(d) = a * d^2 * exp(-b * d^p)
        // Using p=1 and normalizing to mean
        let b = 3.0 / self.mean_droplet_um;
        let a = b.powi(3) / 2.0; // normalization factor for integral ~ 1
        a * d_um * d_um * (-b * d_um).exp()
    }

    /// Drain correction factor.
    /// Returns the fraction of sample that reaches the plasma accounting for drain loss.
    pub fn drain_correction(&self) -> f64 {
        self.transport_efficiency
    }

    /// Volume of sample consumed over a given time (mL).
    pub fn sample_volume_consumed(&self, time_s: f64) -> f64 {
        self.uptake_rate_ml_min * time_s / 60.0
    }
}

// ─── Signal processing ───

/// Gaussian line profile.
pub fn gaussian_profile(x: f64, center: f64, amplitude: f64, sigma: f64) -> f64 {
    if sigma <= 0.0 {
        return 0.0;
    }
    amplitude * (-0.5 * ((x - center) / sigma).powi(2)).exp()
}

/// Lorentzian line profile.
pub fn lorentzian_profile(x: f64, center: f64, amplitude: f64, gamma: f64) -> f64 {
    if gamma <= 0.0 {
        return 0.0;
    }
    amplitude * gamma * gamma / ((x - center).powi(2) + gamma * gamma)
}

/// Pseudo-Voigt profile: linear combination of Gaussian and Lorentzian.
/// eta = 0 is pure Gaussian, eta = 1 is pure Lorentzian.
pub fn voigt_profile(x: f64, center: f64, amplitude: f64, sigma: f64, gamma: f64, eta: f64) -> f64 {
    let eta_c = eta.clamp(0.0, 1.0);
    let g = gaussian_profile(x, center, amplitude, sigma);
    let l = lorentzian_profile(x, center, amplitude, gamma);
    (1.0 - eta_c) * g + eta_c * l
}

/// Result of a peak fit.
#[derive(Debug, Clone)]
pub struct FittedPeak {
    /// Center wavelength (nm).
    pub center_nm: f64,
    /// Peak amplitude (counts).
    pub amplitude: f64,
    /// Gaussian sigma (nm).
    pub sigma_nm: f64,
    /// FWHM (nm).
    pub fwhm_nm: f64,
    /// Integrated (net) area.
    pub area: f64,
}

/// Signal processing engine for ICP-OES spectra.
#[derive(Debug)]
pub struct SignalProcessing {
    /// Minimum peak height above noise for detection.
    pub min_peak_height: f64,
    /// Minimum distance between peaks in pixel units.
    pub min_peak_distance: usize,
}

impl SignalProcessing {
    pub fn new(min_peak_height: f64, min_peak_distance: usize) -> Self {
        Self {
            min_peak_height,
            min_peak_distance,
        }
    }

    /// Find peaks in a 1D spectrum.
    /// Returns indices of local maxima above min_peak_height.
    pub fn find_peaks(&self, spectrum: &[f64]) -> Vec<usize> {
        let n = spectrum.len();
        if n < 3 {
            return Vec::new();
        }
        let mut peaks = Vec::new();
        for i in 1..(n - 1) {
            if spectrum[i] > spectrum[i - 1]
                && spectrum[i] > spectrum[i + 1]
                && spectrum[i] >= self.min_peak_height
            {
                // Check minimum distance from previous peak
                if let Some(&last) = peaks.last() {
                    if i - last < self.min_peak_distance {
                        // Keep the taller peak
                        if spectrum[i] > spectrum[last] {
                            peaks.pop();
                            peaks.push(i);
                        }
                        continue;
                    }
                }
                peaks.push(i);
            }
        }
        peaks
    }

    /// Fit a Gaussian to a peak region.
    /// Uses three-point parabolic interpolation on log(intensity).
    pub fn fit_gaussian(
        &self,
        wavelengths: &[f64],
        intensities: &[f64],
        peak_idx: usize,
    ) -> Option<FittedPeak> {
        if peak_idx == 0 || peak_idx >= wavelengths.len() - 1 {
            return None;
        }
        let y0 = intensities[peak_idx - 1];
        let y1 = intensities[peak_idx];
        let y2 = intensities[peak_idx + 1];

        if y0 <= 0.0 || y1 <= 0.0 || y2 <= 0.0 {
            return None;
        }

        let ln0 = y0.ln();
        let ln1 = y1.ln();
        let ln2 = y2.ln();

        let x0 = wavelengths[peak_idx - 1];
        let x1 = wavelengths[peak_idx];
        let x2 = wavelengths[peak_idx + 1];
        let dx = x1 - x0;
        if dx.abs() < 1e-15 {
            return None;
        }

        // Parabolic interpolation on log values
        let denom = 2.0 * (ln0 - 2.0 * ln1 + ln2);
        if denom.abs() < 1e-30 {
            return None;
        }

        let center = x1 - dx * (ln0 - ln2) / denom;
        let sigma_sq = -dx * dx / (ln0 - 2.0 * ln1 + ln2);
        if sigma_sq <= 0.0 {
            return None;
        }
        let sigma = sigma_sq.sqrt();
        let amplitude = (ln1 + (x1 - center).powi(2) / (2.0 * sigma_sq)).exp();
        let fwhm = 2.0 * (2.0 * (2.0_f64).ln()).sqrt() * sigma;
        let area = amplitude * sigma * (2.0 * PI).sqrt();

        Some(FittedPeak {
            center_nm: center,
            amplitude,
            sigma_nm: sigma,
            fwhm_nm: fwhm,
            area,
        })
    }

    /// Deconvolve two overlapping peaks using iterative fitting.
    /// Returns (peak1, peak2) fitted parameters.
    pub fn deconvolve_two_peaks(
        &self,
        wavelengths: &[f64],
        intensities: &[f64],
        center1_guess: f64,
        center2_guess: f64,
    ) -> Option<(FittedPeak, FittedPeak)> {
        let n = wavelengths.len();
        if n < 5 || intensities.len() != n {
            return None;
        }

        // Initial guesses
        let max_intens = intensities.iter().cloned().fold(0.0_f64, f64::max);
        if max_intens <= 0.0 {
            return None;
        }

        let mut c1 = center1_guess;
        let mut a1 = max_intens * 0.6;
        let mut s1 = 0.02; // nm
        let mut c2 = center2_guess;
        let mut a2 = max_intens * 0.4;
        let mut s2 = 0.02; // nm

        // Simple alternating least-squares iterations
        for _ in 0..50 {
            // Subtract peak2 model, fit peak1
            let residual1: Vec<f64> = wavelengths
                .iter()
                .zip(intensities.iter())
                .map(|(&w, &i)| (i - gaussian_profile(w, c2, a2, s2)).max(0.0))
                .collect();
            if let Some(fit) = self.fit_region(&wavelengths, &residual1, c1) {
                c1 = fit.center_nm;
                a1 = fit.amplitude;
                s1 = fit.sigma_nm;
            }

            // Subtract peak1 model, fit peak2
            let residual2: Vec<f64> = wavelengths
                .iter()
                .zip(intensities.iter())
                .map(|(&w, &i)| (i - gaussian_profile(w, c1, a1, s1)).max(0.0))
                .collect();
            if let Some(fit) = self.fit_region(&wavelengths, &residual2, c2) {
                c2 = fit.center_nm;
                a2 = fit.amplitude;
                s2 = fit.sigma_nm;
            }
        }

        let fwhm1 = 2.0 * (2.0 * 2.0_f64.ln()).sqrt() * s1;
        let fwhm2 = 2.0 * (2.0 * 2.0_f64.ln()).sqrt() * s2;

        Some((
            FittedPeak {
                center_nm: c1,
                amplitude: a1,
                sigma_nm: s1,
                fwhm_nm: fwhm1,
                area: a1 * s1 * (2.0 * PI).sqrt(),
            },
            FittedPeak {
                center_nm: c2,
                amplitude: a2,
                sigma_nm: s2,
                fwhm_nm: fwhm2,
                area: a2 * s2 * (2.0 * PI).sqrt(),
            },
        ))
    }

    /// Fit a Gaussian to a spectral region near a given center wavelength.
    fn fit_region(
        &self,
        wavelengths: &[f64],
        intensities: &[f64],
        center_guess: f64,
    ) -> Option<FittedPeak> {
        // Find nearest pixel to center guess
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, &w) in wavelengths.iter().enumerate() {
            let d = (w - center_guess).abs();
            if d < best_dist {
                best_dist = d;
                best_idx = i;
            }
        }
        // Find local maximum near that pixel
        let search_range = 5;
        let start = if best_idx > search_range {
            best_idx - search_range
        } else {
            0
        };
        let end = (best_idx + search_range + 1).min(wavelengths.len());
        let mut peak_idx = best_idx;
        for i in start..end {
            if intensities[i] > intensities[peak_idx] {
                peak_idx = i;
            }
        }
        self.fit_gaussian(wavelengths, intensities, peak_idx)
    }

    /// Extract net peak intensity after local background subtraction.
    pub fn net_intensity(
        intensities: &[f64],
        peak_idx: usize,
        bg_left_idx: usize,
        bg_right_idx: usize,
    ) -> f64 {
        if peak_idx >= intensities.len()
            || bg_left_idx >= intensities.len()
            || bg_right_idx >= intensities.len()
        {
            return 0.0;
        }
        let bg = (intensities[bg_left_idx] + intensities[bg_right_idx]) / 2.0;
        (intensities[peak_idx] - bg).max(0.0)
    }
}

// ─── Method Detection Limit ───

/// Method detection limit calculations per 40 CFR Part 136.
#[derive(Debug, Clone)]
pub struct MethodDetectionLimit {
    /// Replicate blank measurements.
    pub blank_replicates: Vec<f64>,
}

impl MethodDetectionLimit {
    pub fn new() -> Self {
        Self {
            blank_replicates: Vec::new(),
        }
    }

    pub fn add_replicate(&mut self, value: f64) {
        self.blank_replicates.push(value);
    }

    /// Number of replicates.
    pub fn count(&self) -> usize {
        self.blank_replicates.len()
    }

    /// Mean of blank replicates.
    pub fn mean(&self) -> f64 {
        if self.blank_replicates.is_empty() {
            return 0.0;
        }
        self.blank_replicates.iter().sum::<f64>() / self.blank_replicates.len() as f64
    }

    /// Standard deviation of blank replicates.
    pub fn std_dev(&self) -> f64 {
        let n = self.blank_replicates.len();
        if n < 2 {
            return 0.0;
        }
        let mean = self.mean();
        let var: f64 = self
            .blank_replicates
            .iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>()
            / (n - 1) as f64;
        var.sqrt()
    }

    /// Student's t-value for n-1 degrees of freedom at 99% confidence.
    fn t_value(n: usize) -> f64 {
        // Common t-values for 99% confidence (one-sided)
        match n.saturating_sub(1) {
            0 => 0.0,
            1 => 6.314,
            2 => 2.920,
            3 => 2.353,
            4 => 2.132,
            5 => 2.015,
            6 => 1.943,
            7 => 1.895,
            8 => 1.860,
            9 => 1.833,
            10 => 1.812,
            11 => 1.796,
            12 => 1.782,
            13 => 1.771,
            14 => 1.761,
            15..=19 => 1.746,
            20..=29 => 1.725,
            _ => 1.645,
        }
    }

    /// Method Detection Limit: MDL = t(n-1, 0.99) * s
    pub fn mdl(&self) -> f64 {
        let n = self.count();
        if n < 2 {
            return 0.0;
        }
        Self::t_value(n) * self.std_dev()
    }

    /// Instrument Detection Limit (IDL): 3 * standard deviation.
    pub fn idl(&self) -> f64 {
        3.0 * self.std_dev()
    }

    /// Practical Quantitation Limit: PQL = 10 * MDL.
    pub fn pql(&self) -> f64 {
        10.0 * self.mdl()
    }

    /// Check if MDL is valid (requires 7+ replicates per EPA method).
    pub fn is_valid(&self) -> bool {
        self.count() >= 7
    }
}

impl Default for MethodDetectionLimit {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Quality control ───

/// Quality control check result.
#[derive(Debug, Clone)]
pub struct QcResult {
    pub check_name: String,
    pub measured_value: f64,
    pub expected_value: f64,
    pub tolerance_percent: f64,
    pub passed: bool,
}

/// Quality control system for ICP-OES analysis.
#[derive(Debug)]
pub struct QualityControl {
    pub results: Vec<QcResult>,
}

impl QualityControl {
    pub fn new() -> Self {
        Self {
            results: Vec::new(),
        }
    }

    /// Initial Calibration Verification (ICV): second-source standard.
    /// Must recover within `tolerance_pct` of true value (typically 10%).
    pub fn check_icv(
        &mut self,
        element: &str,
        measured: f64,
        expected: f64,
        tolerance_pct: f64,
    ) -> bool {
        let recovery_pct = if expected.abs() > 1e-30 {
            (measured / expected) * 100.0
        } else {
            0.0
        };
        let passed = (recovery_pct - 100.0).abs() <= tolerance_pct;
        self.results.push(QcResult {
            check_name: format!("ICV-{}", element),
            measured_value: measured,
            expected_value: expected,
            tolerance_percent: tolerance_pct,
            passed,
        });
        passed
    }

    /// Continuing Calibration Verification (CCV): periodic drift check.
    pub fn check_ccv(
        &mut self,
        element: &str,
        measured: f64,
        expected: f64,
        tolerance_pct: f64,
    ) -> bool {
        let recovery_pct = if expected.abs() > 1e-30 {
            (measured / expected) * 100.0
        } else {
            0.0
        };
        let passed = (recovery_pct - 100.0).abs() <= tolerance_pct;
        self.results.push(QcResult {
            check_name: format!("CCV-{}", element),
            measured_value: measured,
            expected_value: expected,
            tolerance_percent: tolerance_pct,
            passed,
        });
        passed
    }

    /// Spike recovery check.
    /// Recovery% = (spiked - unspiked) / spike_amount * 100
    /// Must be within 75-125% for environmental methods.
    pub fn check_spike_recovery(
        &mut self,
        element: &str,
        spiked: f64,
        unspiked: f64,
        spike_amount: f64,
    ) -> bool {
        if spike_amount.abs() < 1e-30 {
            return false;
        }
        let recovery = (spiked - unspiked) / spike_amount * 100.0;
        let passed = recovery >= 75.0 && recovery <= 125.0;
        self.results.push(QcResult {
            check_name: format!("Spike-{}", element),
            measured_value: recovery,
            expected_value: 100.0,
            tolerance_percent: 25.0,
            passed,
        });
        passed
    }

    /// Duplicate Relative Percent Difference (RPD).
    /// RPD = |d1 - d2| / ((d1 + d2)/2) * 100
    /// Must be <= max_rpd (typically 20%).
    pub fn check_duplicate_rpd(
        &mut self,
        element: &str,
        result1: f64,
        result2: f64,
        max_rpd: f64,
    ) -> bool {
        let mean = (result1 + result2) / 2.0;
        let rpd = if mean.abs() > 1e-30 {
            ((result1 - result2).abs() / mean) * 100.0
        } else {
            0.0
        };
        let passed = rpd <= max_rpd;
        self.results.push(QcResult {
            check_name: format!("DupRPD-{}", element),
            measured_value: rpd,
            expected_value: 0.0,
            tolerance_percent: max_rpd,
            passed,
        });
        passed
    }

    /// Linear dynamic range (LDR) verification.
    /// The highest standard should read within `tolerance_pct` of its true value.
    pub fn check_ldr(
        &mut self,
        element: &str,
        measured_high_std: f64,
        expected_high_std: f64,
        tolerance_pct: f64,
    ) -> bool {
        let recovery_pct = if expected_high_std.abs() > 1e-30 {
            (measured_high_std / expected_high_std) * 100.0
        } else {
            0.0
        };
        let passed = (recovery_pct - 100.0).abs() <= tolerance_pct;
        self.results.push(QcResult {
            check_name: format!("LDR-{}", element),
            measured_value: measured_high_std,
            expected_value: expected_high_std,
            tolerance_percent: tolerance_pct,
            passed,
        });
        passed
    }

    /// Count of total checks performed.
    pub fn total_checks(&self) -> usize {
        self.results.len()
    }

    /// Count of passed checks.
    pub fn passed_checks(&self) -> usize {
        self.results.iter().filter(|r| r.passed).count()
    }

    /// Overall pass rate (0.0 to 1.0).
    pub fn pass_rate(&self) -> f64 {
        if self.results.is_empty() {
            return 1.0;
        }
        self.passed_checks() as f64 / self.total_checks() as f64
    }

    /// Check if all QC checks passed.
    pub fn all_passed(&self) -> bool {
        self.results.iter().all(|r| r.passed)
    }
}

impl Default for QualityControl {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Spectrum generation utility ───

/// Generate a synthetic emission spectrum with Gaussian peaks.
pub fn generate_emission_spectrum(
    wavelengths: &[f64],
    lines: &[(f64, f64, f64)], // (center_nm, amplitude, sigma_nm)
    noise_level: f64,
) -> Vec<f64> {
    let mut spectrum = vec![0.0; wavelengths.len()];
    for &(center, amp, sigma) in lines {
        for (i, &w) in wavelengths.iter().enumerate() {
            spectrum[i] += gaussian_profile(w, center, amp, sigma);
        }
    }
    // Add simple deterministic "noise" for testing
    if noise_level > 0.0 {
        for (i, s) in spectrum.iter_mut().enumerate() {
            // Simple deterministic noise pattern based on index
            let noise = noise_level * ((i as f64 * 0.1).sin() * 0.5 + 0.5);
            *s += noise;
        }
    }
    spectrum
}

// ═══════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // ─── Helper function tests ───

    #[test]
    fn test_wavelength_to_energy_visible() {
        // 500 nm ~ 2.48 eV
        let e = wavelength_to_energy(500.0);
        assert!((e - 2.48).abs() < 0.02, "500nm energy: {}", e);
    }

    #[test]
    fn test_wavelength_to_energy_uv() {
        // 200 nm ~ 6.20 eV
        let e = wavelength_to_energy(200.0);
        assert!((e - 6.20).abs() < 0.02, "200nm energy: {}", e);
    }

    #[test]
    fn test_wavelength_to_energy_inversely_proportional() {
        let e1 = wavelength_to_energy(200.0);
        let e2 = wavelength_to_energy(400.0);
        assert!((e1 / e2 - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_boltzmann_population_ground_state() {
        // Ground state (E=0) should have highest population
        let pop = boltzmann_population(1.0, 0.0, 7000.0, 1.0);
        assert!((pop - 1.0).abs() < TOL);
    }

    #[test]
    fn test_boltzmann_population_high_energy() {
        // High energy level should have low population
        let pop = boltzmann_population(1.0, 5.0, 7000.0, 1.0);
        assert!(pop < 0.001);
        assert!(pop > 0.0);
    }

    #[test]
    fn test_boltzmann_population_temperature_dependence() {
        // Higher temp -> more population in excited states
        let pop_low = boltzmann_population(1.0, 3.0, 5000.0, 1.0);
        let pop_high = boltzmann_population(1.0, 3.0, 10000.0, 1.0);
        assert!(pop_high > pop_low);
    }

    #[test]
    fn test_boltzmann_population_degeneracy() {
        let pop_g1 = boltzmann_population(1.0, 3.0, 7000.0, 1.0);
        let pop_g4 = boltzmann_population(4.0, 3.0, 7000.0, 1.0);
        assert!((pop_g4 / pop_g1 - 4.0).abs() < TOL);
    }

    #[test]
    fn test_boltzmann_zero_temp() {
        let pop = boltzmann_population(1.0, 1.0, 0.0, 1.0);
        assert_eq!(pop, 0.0);
    }

    #[test]
    fn test_saha_ratio_positive() {
        let ratio = saha_ionization_ratio(7000.0, 1e15, 6.0);
        assert!(ratio > 0.0);
    }

    #[test]
    fn test_saha_ratio_temperature_dependence() {
        let r1 = saha_ionization_ratio(5000.0, 1e15, 6.0);
        let r2 = saha_ionization_ratio(10000.0, 1e15, 6.0);
        assert!(r2 > r1, "Higher temp should give more ionization");
    }

    #[test]
    fn test_saha_ratio_density_dependence() {
        let r1 = saha_ionization_ratio(7000.0, 1e14, 6.0);
        let r2 = saha_ionization_ratio(7000.0, 1e16, 6.0);
        assert!(r1 > r2, "Lower ne should give more ionization");
    }

    #[test]
    fn test_saha_zero_inputs() {
        assert_eq!(saha_ionization_ratio(0.0, 1e15, 6.0), 0.0);
        assert_eq!(saha_ionization_ratio(7000.0, 0.0, 6.0), 0.0);
    }

    // ─── Ionization state ───

    #[test]
    fn test_ionization_state_charge() {
        assert_eq!(IonizationState::I.charge(), 0);
        assert_eq!(IonizationState::II.charge(), 1);
        assert_eq!(IonizationState::III.charge(), 2);
    }

    // ─── Spectral lines ───

    #[test]
    fn test_preset_lines_count() {
        let lines = preset_lines();
        assert_eq!(lines.len(), 10);
    }

    #[test]
    fn test_preset_lines_calcium() {
        let lines = preset_lines();
        let ca = lines.iter().find(|l| l.element == "Ca").unwrap();
        assert!((ca.wavelength_nm - 393.366).abs() < 0.001);
        assert_eq!(ca.ionization, IonizationState::II);
    }

    #[test]
    fn test_preset_lines_internal_standards() {
        let lines = preset_lines();
        let y = lines.iter().find(|l| l.element == "Y").unwrap();
        assert!((y.wavelength_nm - 371.029).abs() < 0.001);
        let sc = lines.iter().find(|l| l.element == "Sc").unwrap();
        assert!((sc.wavelength_nm - 361.383).abs() < 0.001);
    }

    #[test]
    fn test_spectral_line_new() {
        let line = SpectralLine::new(324.754, "Cu", IonizationState::I, 1.39e8, 3.817, 4.0);
        assert_eq!(line.element, "Cu");
        assert!((line.wavelength_nm - 324.754).abs() < 0.001);
    }

    // ─── Plasma model ───

    #[test]
    fn test_plasma_model_default() {
        let pm = PlasmaModel::default_icp();
        assert!((pm.electron_temp_k - 7000.0).abs() < TOL);
        assert!((pm.electron_density - 1e15).abs() < 1.0);
    }

    #[test]
    fn test_plasma_emission_intensity_positive() {
        let pm = PlasmaModel::default_icp();
        let line = &preset_lines()[0]; // Ca II
        let intens = pm.emission_intensity(line);
        assert!(intens > 0.0);
    }

    #[test]
    fn test_plasma_emission_stronger_for_lower_energy() {
        let pm = PlasmaModel::default_icp();
        let low_e = SpectralLine::new(400.0, "X", IonizationState::I, 1e8, 2.0, 4.0);
        let high_e = SpectralLine::new(200.0, "X", IonizationState::I, 1e8, 6.0, 4.0);
        assert!(pm.emission_intensity(&low_e) > pm.emission_intensity(&high_e));
    }

    #[test]
    fn test_plasma_ionization_fraction_atom() {
        let pm = PlasmaModel::new(6000.0, 1e15);
        let frac = pm.ionization_fraction(IonizationState::I, 7.0);
        assert!(frac >= 0.0 && frac <= 1.0);
    }

    #[test]
    fn test_plasma_ionization_fraction_ion() {
        let pm = PlasmaModel::new(8000.0, 1e15);
        let frac_i = pm.ionization_fraction(IonizationState::I, 6.0);
        let frac_ii = pm.ionization_fraction(IonizationState::II, 6.0);
        assert!((frac_i + frac_ii - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_plasma_ionization_fraction_iii() {
        let pm = PlasmaModel::default_icp();
        let frac = pm.ionization_fraction(IonizationState::III, 6.0);
        assert_eq!(frac, 0.0);
    }

    // ─── Calibration system ───

    #[test]
    fn test_calibration_curve_linear_fit() {
        let mut curve = CalibrationCurve::new("Fe", 259.940);
        curve.add_point(0.0, 100.0);
        curve.add_point(1.0, 1100.0);
        curve.add_point(2.0, 2100.0);
        curve.add_point(5.0, 5100.0);
        assert!(curve.fit());
        assert!((curve.slope - 1000.0).abs() < 1.0);
        assert!((curve.intercept - 100.0).abs() < 1.0);
        assert!(curve.r_squared > 0.999);
    }

    #[test]
    fn test_calibration_concentration_from_intensity() {
        let mut curve = CalibrationCurve::new("Cu", 324.754);
        curve.add_point(0.0, 0.0);
        curve.add_point(10.0, 5000.0);
        curve.fit();
        let conc = curve.concentration_from_intensity(2500.0);
        assert!((conc - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_calibration_r_squared_validity() {
        let mut curve = CalibrationCurve::new("Zn", 213.856);
        curve.add_point(0.0, 0.0);
        curve.add_point(1.0, 1000.0);
        curve.add_point(2.0, 2000.0);
        curve.fit();
        assert!(curve.is_valid(0.99));
    }

    #[test]
    fn test_calibration_system_find_curve() {
        let mut sys = CalibrationSystem::new();
        let mut c = CalibrationCurve::new("Fe", 259.940);
        c.add_point(0.0, 0.0);
        c.add_point(1.0, 100.0);
        c.fit();
        sys.add_curve(c);

        assert!(sys.find_curve("Fe", 259.940).is_some());
        assert!(sys.find_curve("Cu", 324.754).is_none());
    }

    #[test]
    fn test_calibration_internal_standard() {
        let sys = CalibrationSystem::new();
        let corrected = sys.correct_with_internal_standard(5000.0, 2500.0);
        assert!((corrected - 2.0).abs() < TOL);
    }

    #[test]
    fn test_calibration_fit_all() {
        let mut sys = CalibrationSystem::new();
        let mut c1 = CalibrationCurve::new("Fe", 259.940);
        c1.add_point(0.0, 0.0);
        c1.add_point(1.0, 100.0);
        let mut c2 = CalibrationCurve::new("Cu", 324.754);
        c2.add_point(0.0, 0.0);
        c2.add_point(5.0, 500.0);
        sys.add_curve(c1);
        sys.add_curve(c2);
        sys.fit_all();
        assert!(sys.curves[0].slope > 0.0);
        assert!(sys.curves[1].slope > 0.0);
    }

    #[test]
    fn test_calibration_insufficient_points() {
        let mut curve = CalibrationCurve::new("X", 100.0);
        curve.add_point(1.0, 100.0);
        assert!(!curve.fit());
    }

    // ─── Spectral interference ───

    #[test]
    fn test_iec_correction() {
        let mut si = SpectralInterference::new();
        si.add_iec("Fe", "Mn", 0.05);
        let corrected = si.correct_intensity("Mn", 1000.0, &[("Fe", 2000.0)]);
        assert!((corrected - 900.0).abs() < TOL);
    }

    #[test]
    fn test_iec_no_correction_needed() {
        let si = SpectralInterference::new();
        let corrected = si.correct_intensity("Cu", 500.0, &[("Fe", 1000.0)]);
        assert!((corrected - 500.0).abs() < TOL);
    }

    #[test]
    fn test_iec_multiple_interferents() {
        let mut si = SpectralInterference::new();
        si.add_iec("Fe", "Mn", 0.05);
        si.add_iec("Cr", "Mn", 0.03);
        let corrected =
            si.correct_intensity("Mn", 1000.0, &[("Fe", 2000.0), ("Cr", 1000.0)]);
        // 1000 - 0.05*2000 - 0.03*1000 = 1000 - 100 - 30 = 870
        assert!((corrected - 870.0).abs() < TOL);
    }

    #[test]
    fn test_off_peak_correction() {
        let net = SpectralInterference::off_peak_correction(500.0, 100.0, 120.0);
        assert!((net - 390.0).abs() < TOL);
    }

    #[test]
    fn test_polynomial_background_correction() {
        let wl: Vec<f64> = (0..20).map(|i| 250.0 + i as f64 * 0.1).collect();
        // Linear background + peak
        let intens: Vec<f64> = wl
            .iter()
            .map(|&w| {
                let bg = 100.0 + 10.0 * (w - 250.0);
                let peak = gaussian_profile(w, 251.0, 500.0, 0.1);
                bg + peak
            })
            .collect();
        let bg_indices: Vec<usize> = vec![0, 1, 2, 17, 18, 19];
        let corrected =
            SpectralInterference::background_correction_polynomial(&wl, &intens, &bg_indices, 1);
        // Peak region should still show peak, background should be ~0
        assert!(corrected[10] > 10.0); // peak vicinity
        assert!(corrected[0] < 5.0); // background region
    }

    // ─── Detector model ───

    #[test]
    fn test_detector_pixel_to_wavelength() {
        let det = DetectorModel::new(1000, 200.0, 400.0);
        assert!((det.pixel_to_wavelength(0) - 200.0).abs() < TOL);
        assert!((det.pixel_to_wavelength(999) - 400.0).abs() < TOL);
    }

    #[test]
    fn test_detector_wavelength_to_pixel() {
        let det = DetectorModel::new(1000, 200.0, 400.0);
        let px = det.wavelength_to_pixel(300.0);
        assert!((px - 499.5).abs() < 0.5);
    }

    #[test]
    fn test_detector_spectral_resolution() {
        let det = DetectorModel::new(1000, 200.0, 400.0);
        let res = det.spectral_resolution();
        assert!((res - 0.2002).abs() < 0.001);
    }

    #[test]
    fn test_detector_dark_current_subtraction() {
        let det = DetectorModel::new(5, 200.0, 400.0);
        let raw = vec![100.0, 50.0, 200.0, 5.0, 150.0];
        let corrected = det.dark_current_subtract(&raw);
        assert!((corrected[0] - 90.0).abs() < TOL);
        assert!((corrected[3] - 0.0).abs() < TOL); // clamped to 0
    }

    #[test]
    fn test_detector_flat_field_correction() {
        let mut det = DetectorModel::new(4, 200.0, 400.0);
        det.flat_field = vec![1.0, 0.9, 1.1, 1.0];
        let counts = vec![100.0, 90.0, 110.0, 100.0];
        let corrected = det.flat_field_correct(&counts);
        assert!((corrected[0] - 100.0).abs() < TOL);
        assert!((corrected[1] - 100.0).abs() < TOL);
        assert!((corrected[2] - 100.0).abs() < TOL);
    }

    #[test]
    fn test_detector_set_flat_field() {
        let mut det = DetectorModel::new(4, 200.0, 400.0);
        det.set_flat_field_from_uniform(&[1000.0, 900.0, 1100.0, 1000.0]);
        assert!((det.flat_field[1] - 0.9).abs() < TOL);
    }

    #[test]
    fn test_detector_pixel_binning() {
        let counts = vec![10.0, 20.0, 30.0, 40.0, 50.0, 60.0];
        let binned = DetectorModel::pixel_binning(&counts, 2);
        assert_eq!(binned.len(), 3);
        assert!((binned[0] - 30.0).abs() < TOL);
        assert!((binned[1] - 70.0).abs() < TOL);
        assert!((binned[2] - 110.0).abs() < TOL);
    }

    #[test]
    fn test_detector_process_frame() {
        let det = DetectorModel::new(3, 200.0, 400.0);
        let raw = vec![110.0, 210.0, 310.0];
        let processed = det.process_frame(&raw);
        // dark_current = 10 counts, flat_field = all 1.0
        assert!((processed[0] - 100.0).abs() < TOL);
        assert!((processed[1] - 200.0).abs() < TOL);
        assert!((processed[2] - 300.0).abs() < TOL);
    }

    // ─── Nebulizer efficiency ───

    #[test]
    fn test_nebulizer_default() {
        let neb = NebulizerEfficiency::concentric_default();
        assert!((neb.uptake_rate_ml_min - 1.0).abs() < TOL);
        assert!((neb.transport_efficiency - 0.02).abs() < TOL);
    }

    #[test]
    fn test_nebulizer_delivery_rate() {
        let neb = NebulizerEfficiency::new(1.0, 0.02);
        // 1 ppm, 1 mL/min, 2% efficiency
        let rate = neb.analyte_delivery_rate(1.0);
        // 1 mg/L * (1/60 mL/s) * 1e-3 L/mL * 0.02 * 1000 ug/mg
        // = 1 * 0.01667 * 0.001 * 0.02 * 1000 = 0.000333 ug/s
        assert!((rate - 0.000333).abs() < 0.0001);
    }

    #[test]
    fn test_nebulizer_droplet_pdf() {
        let neb = NebulizerEfficiency::concentric_default();
        let p = neb.droplet_size_pdf(10.0);
        assert!(p > 0.0);
    }

    #[test]
    fn test_nebulizer_droplet_pdf_zero() {
        let neb = NebulizerEfficiency::concentric_default();
        assert_eq!(neb.droplet_size_pdf(0.0), 0.0);
        assert_eq!(neb.droplet_size_pdf(-1.0), 0.0);
    }

    #[test]
    fn test_nebulizer_drain_correction() {
        let neb = NebulizerEfficiency::new(1.0, 0.03);
        assert!((neb.drain_correction() - 0.03).abs() < TOL);
    }

    #[test]
    fn test_nebulizer_sample_volume() {
        let neb = NebulizerEfficiency::new(2.0, 0.02);
        let vol = neb.sample_volume_consumed(60.0);
        assert!((vol - 2.0).abs() < TOL);
    }

    // ─── Signal processing ───

    #[test]
    fn test_gaussian_profile_peak() {
        let v = gaussian_profile(5.0, 5.0, 100.0, 0.1);
        assert!((v - 100.0).abs() < TOL);
    }

    #[test]
    fn test_gaussian_profile_symmetry() {
        let v1 = gaussian_profile(4.9, 5.0, 100.0, 0.1);
        let v2 = gaussian_profile(5.1, 5.0, 100.0, 0.1);
        assert!((v1 - v2).abs() < TOL);
    }

    #[test]
    fn test_lorentzian_profile_peak() {
        let v = lorentzian_profile(5.0, 5.0, 100.0, 0.1);
        assert!((v - 100.0).abs() < TOL);
    }

    #[test]
    fn test_voigt_profile_gaussian_limit() {
        let g = gaussian_profile(4.95, 5.0, 100.0, 0.1);
        let v = voigt_profile(4.95, 5.0, 100.0, 0.1, 0.1, 0.0);
        assert!((g - v).abs() < TOL);
    }

    #[test]
    fn test_voigt_profile_lorentzian_limit() {
        let l = lorentzian_profile(4.95, 5.0, 100.0, 0.1);
        let v = voigt_profile(4.95, 5.0, 100.0, 0.1, 0.1, 1.0);
        assert!((l - v).abs() < TOL);
    }

    #[test]
    fn test_find_peaks_single() {
        let data = vec![0.0, 1.0, 3.0, 5.0, 3.0, 1.0, 0.0];
        let sp = SignalProcessing::new(2.0, 1);
        let peaks = sp.find_peaks(&data);
        assert_eq!(peaks, vec![3]);
    }

    #[test]
    fn test_find_peaks_multiple() {
        let data = vec![0.0, 5.0, 0.0, 0.0, 8.0, 0.0, 0.0, 3.0, 0.0];
        let sp = SignalProcessing::new(2.0, 1);
        let peaks = sp.find_peaks(&data);
        assert_eq!(peaks, vec![1, 4, 7]);
    }

    #[test]
    fn test_find_peaks_min_height() {
        let data = vec![0.0, 1.0, 0.0, 0.0, 5.0, 0.0];
        let sp = SignalProcessing::new(3.0, 1);
        let peaks = sp.find_peaks(&data);
        assert_eq!(peaks, vec![4]);
    }

    #[test]
    fn test_find_peaks_min_distance() {
        // Two peaks close together; keep the taller one
        let data = vec![0.0, 5.0, 3.0, 8.0, 0.0];
        let sp = SignalProcessing::new(1.0, 3);
        let peaks = sp.find_peaks(&data);
        assert_eq!(peaks, vec![3]); // 8.0 is taller
    }

    #[test]
    fn test_find_peaks_empty() {
        let sp = SignalProcessing::new(1.0, 1);
        assert!(sp.find_peaks(&[]).is_empty());
        assert!(sp.find_peaks(&[1.0, 2.0]).is_empty());
    }

    #[test]
    fn test_fit_gaussian_symmetric_peak() {
        let wavelengths: Vec<f64> = (0..100).map(|i| 250.0 + i as f64 * 0.01).collect();
        let intensities: Vec<f64> = wavelengths
            .iter()
            .map(|&w| gaussian_profile(w, 250.5, 1000.0, 0.05))
            .collect();
        let sp = SignalProcessing::new(10.0, 1);
        let peak_idx = 50; // center at 250.50
        let fit = sp.fit_gaussian(&wavelengths, &intensities, peak_idx).unwrap();
        assert!((fit.center_nm - 250.5).abs() < 0.01);
        assert!((fit.amplitude - 1000.0).abs() < 5.0);
    }

    #[test]
    fn test_net_intensity() {
        let intens = vec![50.0, 55.0, 200.0, 500.0, 200.0, 55.0, 50.0];
        let net = SignalProcessing::net_intensity(&intens, 3, 0, 6);
        assert!((net - 450.0).abs() < TOL);
    }

    #[test]
    fn test_deconvolve_two_peaks() {
        let wavelengths: Vec<f64> = (0..200).map(|i| 250.0 + i as f64 * 0.005).collect();
        let intensities: Vec<f64> = wavelengths
            .iter()
            .map(|&w| {
                gaussian_profile(w, 250.4, 800.0, 0.03)
                    + gaussian_profile(w, 250.5, 600.0, 0.03)
            })
            .collect();
        let sp = SignalProcessing::new(10.0, 1);
        let result = sp.deconvolve_two_peaks(&wavelengths, &intensities, 250.4, 250.5);
        assert!(result.is_some());
        let (p1, p2) = result.unwrap();
        assert!((p1.center_nm - 250.4).abs() < 0.02);
        assert!((p2.center_nm - 250.5).abs() < 0.02);
    }

    // ─── Method detection limit ───

    #[test]
    fn test_mdl_seven_replicates() {
        let mut mdl = MethodDetectionLimit::new();
        for &v in &[0.10, 0.12, 0.09, 0.11, 0.13, 0.10, 0.11] {
            mdl.add_replicate(v);
        }
        assert!(mdl.is_valid());
        assert!(mdl.mdl() > 0.0);
        assert!(mdl.idl() > 0.0);
        assert!(mdl.pql() > mdl.mdl());
    }

    #[test]
    fn test_mdl_pql_relationship() {
        let mut mdl = MethodDetectionLimit::new();
        for &v in &[0.5, 0.6, 0.4, 0.55, 0.45, 0.5, 0.52] {
            mdl.add_replicate(v);
        }
        assert!((mdl.pql() - 10.0 * mdl.mdl()).abs() < TOL);
    }

    #[test]
    fn test_mdl_idl_three_sigma() {
        let mut mdl = MethodDetectionLimit::new();
        for &v in &[1.0, 1.1, 0.9, 1.05, 0.95, 1.0, 1.02] {
            mdl.add_replicate(v);
        }
        assert!((mdl.idl() - 3.0 * mdl.std_dev()).abs() < TOL);
    }

    #[test]
    fn test_mdl_insufficient_replicates() {
        let mut mdl = MethodDetectionLimit::new();
        mdl.add_replicate(1.0);
        mdl.add_replicate(2.0);
        assert!(!mdl.is_valid());
    }

    #[test]
    fn test_mdl_mean() {
        let mut mdl = MethodDetectionLimit::new();
        for &v in &[2.0, 4.0, 6.0] {
            mdl.add_replicate(v);
        }
        assert!((mdl.mean() - 4.0).abs() < TOL);
    }

    #[test]
    fn test_mdl_std_dev() {
        let mut mdl = MethodDetectionLimit::new();
        for &v in &[2.0, 4.0, 6.0] {
            mdl.add_replicate(v);
        }
        assert!((mdl.std_dev() - 2.0).abs() < TOL);
    }

    #[test]
    fn test_mdl_empty() {
        let mdl = MethodDetectionLimit::new();
        assert_eq!(mdl.mean(), 0.0);
        assert_eq!(mdl.std_dev(), 0.0);
        assert_eq!(mdl.mdl(), 0.0);
    }

    // ─── Quality control ───

    #[test]
    fn test_qc_icv_pass() {
        let mut qc = QualityControl::new();
        assert!(qc.check_icv("Fe", 1.05, 1.0, 10.0));
    }

    #[test]
    fn test_qc_icv_fail() {
        let mut qc = QualityControl::new();
        assert!(!qc.check_icv("Fe", 1.15, 1.0, 10.0));
    }

    #[test]
    fn test_qc_ccv_pass() {
        let mut qc = QualityControl::new();
        assert!(qc.check_ccv("Cu", 5.1, 5.0, 10.0));
    }

    #[test]
    fn test_qc_ccv_fail() {
        let mut qc = QualityControl::new();
        assert!(!qc.check_ccv("Cu", 4.0, 5.0, 10.0));
    }

    #[test]
    fn test_qc_spike_recovery_pass() {
        let mut qc = QualityControl::new();
        // unspiked=10, spiked=20, amount=10 -> recovery 100%
        assert!(qc.check_spike_recovery("Mn", 20.0, 10.0, 10.0));
    }

    #[test]
    fn test_qc_spike_recovery_low_fail() {
        let mut qc = QualityControl::new();
        // recovery = (15-10)/10*100 = 50%, below 75%
        assert!(!qc.check_spike_recovery("Mn", 15.0, 10.0, 10.0));
    }

    #[test]
    fn test_qc_spike_recovery_high_fail() {
        let mut qc = QualityControl::new();
        // recovery = (24-10)/10*100 = 140%, above 125%
        assert!(!qc.check_spike_recovery("Mn", 24.0, 10.0, 10.0));
    }

    #[test]
    fn test_qc_duplicate_rpd_pass() {
        let mut qc = QualityControl::new();
        assert!(qc.check_duplicate_rpd("Cr", 10.0, 10.5, 20.0));
    }

    #[test]
    fn test_qc_duplicate_rpd_fail() {
        let mut qc = QualityControl::new();
        assert!(!qc.check_duplicate_rpd("Cr", 10.0, 15.0, 20.0));
    }

    #[test]
    fn test_qc_ldr_pass() {
        let mut qc = QualityControl::new();
        assert!(qc.check_ldr("Pb", 98.0, 100.0, 5.0));
    }

    #[test]
    fn test_qc_ldr_fail() {
        let mut qc = QualityControl::new();
        assert!(!qc.check_ldr("Pb", 80.0, 100.0, 5.0));
    }

    #[test]
    fn test_qc_all_passed() {
        let mut qc = QualityControl::new();
        qc.check_icv("Fe", 1.02, 1.0, 10.0);
        qc.check_ccv("Fe", 0.99, 1.0, 10.0);
        assert!(qc.all_passed());
    }

    #[test]
    fn test_qc_pass_rate() {
        let mut qc = QualityControl::new();
        qc.check_icv("Fe", 1.02, 1.0, 10.0); // pass
        qc.check_icv("Cu", 1.50, 1.0, 10.0); // fail
        assert!((qc.pass_rate() - 0.5).abs() < TOL);
    }

    #[test]
    fn test_qc_total_checks() {
        let mut qc = QualityControl::new();
        qc.check_icv("Fe", 1.0, 1.0, 10.0);
        qc.check_ccv("Fe", 1.0, 1.0, 10.0);
        qc.check_duplicate_rpd("Fe", 1.0, 1.0, 20.0);
        assert_eq!(qc.total_checks(), 3);
    }

    // ─── Spectrum generation ───

    #[test]
    fn test_generate_emission_spectrum() {
        let wl: Vec<f64> = (0..100).map(|i| 250.0 + i as f64 * 0.1).collect();
        let lines = vec![(255.0, 1000.0, 0.1)];
        let spec = generate_emission_spectrum(&wl, &lines, 0.0);
        assert_eq!(spec.len(), 100);
        // Peak should be near index 50 (255.0 nm)
        let peak_idx = spec
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 50);
    }

    #[test]
    fn test_generate_spectrum_with_noise() {
        let wl: Vec<f64> = (0..50).map(|i| 250.0 + i as f64 * 0.1).collect();
        let spec_no_noise = generate_emission_spectrum(&wl, &[], 0.0);
        let spec_noise = generate_emission_spectrum(&wl, &[], 10.0);
        // With noise, values should be >= 0 but some > 0
        assert!(spec_noise.iter().any(|&v| v > 0.0));
        assert!(spec_no_noise.iter().all(|&v| v == 0.0));
    }

    // ─── Polynomial fitting ───

    #[test]
    fn test_poly_fit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0]; // y = 1 + 2x
        let coeffs = poly_fit(&x, &y, 1);
        assert!((coeffs[0] - 1.0).abs() < 0.01);
        assert!((coeffs[1] - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_poly_fit_quadratic() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y: Vec<f64> = x.iter().map(|&xi| 1.0 + 2.0 * xi + 0.5 * xi * xi).collect();
        let coeffs = poly_fit(&x, &y, 2);
        assert!((coeffs[0] - 1.0).abs() < 0.01);
        assert!((coeffs[1] - 2.0).abs() < 0.01);
        assert!((coeffs[2] - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_poly_eval() {
        let coeffs = vec![1.0, 2.0, 3.0]; // 1 + 2x + 3x^2
        assert!((poly_eval(&coeffs, 0.0) - 1.0).abs() < TOL);
        assert!((poly_eval(&coeffs, 1.0) - 6.0).abs() < TOL);
        assert!((poly_eval(&coeffs, 2.0) - 17.0).abs() < TOL);
    }

    // ─── Integration / end-to-end ───

    #[test]
    fn test_full_workflow_calibrate_and_measure() {
        // 1. Create calibration
        let mut curve = CalibrationCurve::new("Fe", 259.940);
        curve.add_point(0.0, 50.0);
        curve.add_point(1.0, 1050.0);
        curve.add_point(5.0, 5050.0);
        curve.add_point(10.0, 10050.0);
        curve.fit();
        assert!(curve.r_squared > 0.999);

        // 2. Measure unknown
        let unknown_intensity = 3050.0;
        let conc = curve.concentration_from_intensity(unknown_intensity);
        assert!((conc - 3.0).abs() < 0.01);
    }

    #[test]
    fn test_full_workflow_with_internal_standard() {
        let sys = CalibrationSystem::new();

        // Analyte and IS raw signals
        let analyte_raw = 5000.0;
        let is_raw = 2500.0;
        let ratio = sys.correct_with_internal_standard(analyte_raw, is_raw);
        assert!((ratio - 2.0).abs() < TOL);
    }

    #[test]
    fn test_full_workflow_detector_pipeline() {
        let mut det = DetectorModel::new(5, 250.0, 260.0);
        det.dark_current_rate = 20.0;
        det.integration_time = 2.0;
        det.flat_field = vec![0.95, 1.0, 1.05, 1.0, 0.98];

        let raw = vec![140.0, 240.0, 342.0, 240.0, 138.4];
        let processed = det.process_frame(&raw);
        // After dark subtraction (40 counts) and flat-field correction
        assert!((processed[0] - 100.0 / 0.95).abs() < 0.1);
    }
}
