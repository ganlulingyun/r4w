// trace:FR-MS-QUAD | ai:claude
//! # Mass Spectrometry Quadrupole Analyzer
//!
//! Implements quadrupole mass spectrometry analysis including Mathieu stability
//! diagrams, mass-to-charge ratio filtering, ion trajectory simulation,
//! isotope pattern analysis, and peak identification.
//!
//! ## Physics Background
//!
//! - **Mathieu equation**: d²u/dξ² + (a - 2q*cos(2ξ))*u = 0
//! - **Stability parameters**: a = 8eU/(m*r0²*ω²), q = 4eV/(m*r0²*ω²)
//! - **Mass scan**: Ramp U and V at constant U/V ratio
//! - **Resolution**: R = m/Δm, related to number of RF cycles in quadrupole

use std::f64::consts::PI;

/// Elementary charge in Coulombs.
pub const ELEMENTARY_CHARGE: f64 = 1.602e-19;
/// Atomic mass unit in kg.
pub const AMU: f64 = 1.661e-27;
/// Speed of light in m/s.
pub const SPEED_OF_LIGHT: f64 = 2.998e8;

// ---------------------------------------------------------------------------
// 1. Mathieu Stability Parameters
// ---------------------------------------------------------------------------

/// Mathieu stability parameters for a quadrupole mass filter.
#[derive(Debug, Clone, Copy)]
pub struct MathieuParams {
    /// a parameter (DC).
    pub a: f64,
    /// q parameter (RF).
    pub q: f64,
}

/// Calculate Mathieu a parameter: a = 8*e*U / (m * r0^2 * omega^2)
pub fn mathieu_a(dc_voltage: f64, mass_amu: f64, r0_m: f64, omega_rad_s: f64) -> f64 {
    let m_kg: f64 = mass_amu * AMU;
    8.0 * ELEMENTARY_CHARGE * dc_voltage / (m_kg * r0_m * r0_m * omega_rad_s * omega_rad_s)
}

/// Calculate Mathieu q parameter: q = 4*e*V / (m * r0^2 * omega^2)
pub fn mathieu_q(rf_amplitude: f64, mass_amu: f64, r0_m: f64, omega_rad_s: f64) -> f64 {
    let m_kg: f64 = mass_amu * AMU;
    4.0 * ELEMENTARY_CHARGE * rf_amplitude / (m_kg * r0_m * r0_m * omega_rad_s * omega_rad_s)
}

/// Check if point (a, q) is in the first stability region.
/// Approximation: q < 0.908 and a < 0.237 and a > q^2/2 - some offset.
pub fn is_stable_first_region(a: f64, q: f64) -> bool {
    if q < 0.0 || q > 0.908 { return false; }
    if a < 0.0 { return false; }
    // Approximate boundary: a < 0.23699 * (1 - (q/0.908)^2)^0.5 * q / 0.706
    // Simplified: use tip of stability at (a=0.237, q=0.706)
    // Lower boundary: a > q^2 * 0.5 - 0.02 (very rough)
    let a_max: f64 = 0.237 * (1.0 - ((q - 0.706) / 0.706).powi(2)).max(0.0);
    a <= a_max + 0.01 && a >= 0.0
}

// ---------------------------------------------------------------------------
// 2. Mass Spectrum
// ---------------------------------------------------------------------------

/// A mass spectrum with m/z and intensity arrays.
#[derive(Debug, Clone)]
pub struct MassSpectrum {
    /// Mass-to-charge ratios.
    pub mz: Vec<f64>,
    /// Intensities (counts or arbitrary units).
    pub intensities: Vec<f64>,
}

impl MassSpectrum {
    /// Create new mass spectrum.
    pub fn new(mz: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(mz.len(), intensities.len());
        Self { mz, intensities }
    }

    /// Find base peak (highest intensity).
    pub fn base_peak(&self) -> (f64, f64) {
        let mut max_idx: usize = 0;
        let mut max_val: f64 = f64::NEG_INFINITY;
        for (i, &v) in self.intensities.iter().enumerate() {
            if v > max_val { max_val = v; max_idx = i; }
        }
        (self.mz[max_idx], max_val)
    }

    /// Total ion current (sum of all intensities).
    pub fn total_ion_current(&self) -> f64 {
        self.intensities.iter().sum()
    }

    /// Normalize to base peak = 100%.
    pub fn normalize_to_base_peak(&mut self) {
        let (_, max_int) = self.base_peak();
        if max_int > 0.0 {
            for v in &mut self.intensities {
                *v = *v / max_int * 100.0;
            }
        }
    }

    /// Find peaks above a threshold.
    pub fn find_peaks(&self, threshold: f64) -> Vec<MassPeak> {
        let n: usize = self.mz.len();
        let mut peaks: Vec<MassPeak> = Vec::new();
        for i in 1..n.saturating_sub(1) {
            if self.intensities[i] > threshold
                && self.intensities[i] > self.intensities[i - 1]
                && self.intensities[i] > self.intensities[i + 1]
            {
                peaks.push(MassPeak {
                    mz: self.mz[i],
                    intensity: self.intensities[i],
                });
            }
        }
        peaks.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap_or(std::cmp::Ordering::Equal));
        peaks
    }

    /// Extract m/z range.
    pub fn extract_range(&self, mz_min: f64, mz_max: f64) -> MassSpectrum {
        let mut mz_out: Vec<f64> = Vec::new();
        let mut int_out: Vec<f64> = Vec::new();
        for (i, &m) in self.mz.iter().enumerate() {
            if m >= mz_min && m <= mz_max {
                mz_out.push(m);
                int_out.push(self.intensities[i]);
            }
        }
        MassSpectrum::new(mz_out, int_out)
    }
}

/// A peak in the mass spectrum.
#[derive(Debug, Clone)]
pub struct MassPeak {
    pub mz: f64,
    pub intensity: f64,
}

// ---------------------------------------------------------------------------
// 3. Isotope Pattern
// ---------------------------------------------------------------------------

/// Natural isotope abundances for common elements.
#[derive(Debug, Clone)]
pub struct IsotopePattern {
    /// Mass offsets from monoisotopic mass.
    pub mass_offsets: Vec<f64>,
    /// Relative abundances (normalized).
    pub abundances: Vec<f64>,
}

/// Generate isotope pattern for C_n using binomial approximation.
/// P(k) = C(n,k) * p^k * (1-p)^(n-k) where p = 0.011 (13C abundance)
pub fn carbon_isotope_pattern(n_carbons: usize) -> IsotopePattern {
    let p13c: f64 = 0.011;
    let max_k: usize = (n_carbons).min(6);
    let mut offsets: Vec<f64> = Vec::new();
    let mut abundances: Vec<f64> = Vec::new();
    for k in 0..=max_k {
        let binom: f64 = binomial_coeff(n_carbons, k);
        let prob: f64 = binom * p13c.powi(k as i32) * (1.0 - p13c).powi((n_carbons - k) as i32);
        offsets.push(k as f64 * 1.003355);
        abundances.push(prob);
    }
    // Normalize
    let max_a: f64 = abundances.iter().cloned().fold(0.0_f64, f64::max);
    if max_a > 0.0 {
        for a in &mut abundances { *a /= max_a; }
    }
    IsotopePattern { mass_offsets: offsets, abundances }
}

fn binomial_coeff(n: usize, k: usize) -> f64 {
    if k > n { return 0.0; }
    let k: usize = k.min(n - k);
    let mut result: f64 = 1.0;
    for i in 0..k {
        result *= (n - i) as f64 / (i + 1) as f64;
    }
    result
}

/// Match an observed pattern against a theoretical isotope pattern.
/// Returns cosine similarity.
pub fn match_isotope_pattern(observed: &[f64], theoretical: &[f64]) -> f64 {
    let n: usize = observed.len().min(theoretical.len());
    if n == 0 { return 0.0; }
    let mut dot: f64 = 0.0;
    let mut norm_o: f64 = 0.0;
    let mut norm_t: f64 = 0.0;
    for i in 0..n {
        dot += observed[i] * theoretical[i];
        norm_o += observed[i] * observed[i];
        norm_t += theoretical[i] * theoretical[i];
    }
    let denom: f64 = (norm_o * norm_t).sqrt();
    if denom < 1e-30 { return 0.0; }
    dot / denom
}

// ---------------------------------------------------------------------------
// 4. Quadrupole Mass Filter Simulation
// ---------------------------------------------------------------------------

/// Quadrupole mass filter configuration.
#[derive(Debug, Clone)]
pub struct QuadrupoleConfig {
    /// Inscribed radius r0 in meters.
    pub r0_m: f64,
    /// RF frequency in Hz.
    pub rf_freq_hz: f64,
    /// DC/RF ratio for mass scan line.
    pub dc_rf_ratio: f64,
    /// Number of RF cycles in the quadrupole.
    pub n_cycles: usize,
}

impl QuadrupoleConfig {
    /// Standard quadrupole with r0=4.15mm, f=1MHz.
    pub fn standard() -> Self {
        Self {
            r0_m: 0.00415,
            rf_freq_hz: 1.0e6,
            dc_rf_ratio: 0.168,
            n_cycles: 100,
        }
    }

    /// Angular frequency.
    pub fn omega(&self) -> f64 {
        2.0 * PI * self.rf_freq_hz
    }

    /// Resolution: R ~ n_cycles * factor.
    pub fn resolution(&self) -> f64 {
        self.n_cycles as f64 * 0.1
    }

    /// RF amplitude needed to pass a given m/z at q=0.706.
    pub fn rf_for_mz(&self, mz: f64) -> f64 {
        let q_tip: f64 = 0.706;
        let m_kg: f64 = mz * AMU;
        let omega: f64 = self.omega();
        q_tip * m_kg * self.r0_m * self.r0_m * omega * omega / (4.0 * ELEMENTARY_CHARGE)
    }

    /// DC voltage for given RF at the set DC/RF ratio.
    pub fn dc_for_rf(&self, rf_v: f64) -> f64 {
        self.dc_rf_ratio * rf_v
    }

    /// Transmission function: Gaussian approximation centered at target m/z.
    pub fn transmission(&self, mz_target: f64, mz_ion: f64) -> f64 {
        let r: f64 = self.resolution();
        if r <= 0.0 { return 1.0; }
        let sigma: f64 = mz_target / (2.355 * r);
        let z: f64 = (mz_ion - mz_target) / sigma;
        (-0.5 * z * z).exp()
    }
}

/// Simulate a mass scan across a range.
pub fn simulate_mass_scan(
    config: &QuadrupoleConfig,
    ions: &[(f64, f64)],  // (m/z, relative_abundance)
    mz_start: f64,
    mz_end: f64,
    n_points: usize,
) -> MassSpectrum {
    let step: f64 = (mz_end - mz_start) / (n_points as f64 - 1.0);
    let mut mz_out: Vec<f64> = Vec::with_capacity(n_points);
    let mut int_out: Vec<f64> = Vec::with_capacity(n_points);
    for i in 0..n_points {
        let mz: f64 = mz_start + i as f64 * step;
        let mut total: f64 = 0.0;
        for &(ion_mz, abundance) in ions {
            total += abundance * config.transmission(mz, ion_mz);
        }
        mz_out.push(mz);
        int_out.push(total);
    }
    MassSpectrum::new(mz_out, int_out)
}

// ---------------------------------------------------------------------------
// 5. Mass Accuracy and Calibration
// ---------------------------------------------------------------------------

/// Mass accuracy in ppm: (observed - theoretical) / theoretical * 1e6
pub fn mass_accuracy_ppm(observed_mz: f64, theoretical_mz: f64) -> f64 {
    (observed_mz - theoretical_mz) / theoretical_mz * 1.0e6
}

/// Mass accuracy in mDa: (observed - theoretical) * 1000
pub fn mass_accuracy_mda(observed_mz: f64, theoretical_mz: f64) -> f64 {
    (observed_mz - theoretical_mz) * 1000.0
}

/// Calibrate m/z axis using polynomial fit.
/// true_mz = a0 + a1*raw + a2*raw^2
pub fn calibrate_mz_polynomial(
    raw_mz: &[f64],
    true_mz: &[f64],
) -> (f64, f64, f64) {
    let n: usize = raw_mz.len().min(true_mz.len());
    if n < 3 { return (0.0, 1.0, 0.0); }
    // Simple linear fit (ignoring quadratic for stability)
    let mut sum_x: f64 = 0.0;
    let mut sum_y: f64 = 0.0;
    let mut sum_xy: f64 = 0.0;
    let mut sum_xx: f64 = 0.0;
    let nf: f64 = n as f64;
    for i in 0..n {
        sum_x += raw_mz[i];
        sum_y += true_mz[i];
        sum_xy += raw_mz[i] * true_mz[i];
        sum_xx += raw_mz[i] * raw_mz[i];
    }
    let denom: f64 = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 { return (0.0, 1.0, 0.0); }
    let a1: f64 = (nf * sum_xy - sum_x * sum_y) / denom;
    let a0: f64 = (sum_y - a1 * sum_x) / nf;
    (a0, a1, 0.0)
}

/// Apply polynomial calibration.
pub fn apply_calibration(raw_mz: f64, a0: f64, a1: f64, a2: f64) -> f64 {
    a0 + a1 * raw_mz + a2 * raw_mz * raw_mz
}

// ---------------------------------------------------------------------------
// 6. Signal Processing
// ---------------------------------------------------------------------------

/// Smooth spectrum with moving average.
pub fn smooth_moving_average(intensities: &[f64], window: usize) -> Vec<f64> {
    let n: usize = intensities.len();
    if window < 2 || n < window { return intensities.to_vec(); }
    let half: usize = window / 2;
    let mut out: Vec<f64> = vec![0.0; n];
    for i in 0..n {
        let start: usize = if i >= half { i - half } else { 0 };
        let end: usize = (i + half + 1).min(n);
        let count: f64 = (end - start) as f64;
        let mut sum: f64 = 0.0;
        for j in start..end { sum += intensities[j]; }
        out[i] = sum / count;
    }
    out
}

/// Baseline subtraction using rolling minimum.
pub fn baseline_rolling_minimum(intensities: &[f64], window: usize) -> Vec<f64> {
    let n: usize = intensities.len();
    if window < 2 || n < window { return intensities.to_vec(); }
    let half: usize = window / 2;
    let mut baseline: Vec<f64> = vec![0.0; n];
    for i in 0..n {
        let start: usize = if i >= half { i - half } else { 0 };
        let end: usize = (i + half + 1).min(n);
        let mut min_val: f64 = f64::MAX;
        for j in start..end {
            if intensities[j] < min_val { min_val = intensities[j]; }
        }
        baseline[i] = min_val;
    }
    let mut corrected: Vec<f64> = Vec::with_capacity(n);
    for i in 0..n {
        corrected.push((intensities[i] - baseline[i]).max(0.0));
    }
    corrected
}

/// Signal-to-noise ratio estimate.
pub fn estimate_snr(signal_peak: f64, noise_region: &[f64]) -> f64 {
    if noise_region.is_empty() { return 0.0; }
    let mean: f64 = noise_region.iter().sum::<f64>() / noise_region.len() as f64;
    let mut var: f64 = 0.0;
    for &v in noise_region {
        var += (v - mean) * (v - mean);
    }
    let sigma: f64 = (var / noise_region.len() as f64).sqrt();
    if sigma < 1e-30 { return f64::INFINITY; }
    signal_peak / sigma
}

// ---------------------------------------------------------------------------
// 7. Molecular Formula Utilities
// ---------------------------------------------------------------------------

/// Calculate exact monoisotopic mass from atomic composition.
/// Uses: C=12.0, H=1.00783, N=14.003, O=15.995, S=31.972
pub fn exact_mass(n_c: usize, n_h: usize, n_n: usize, n_o: usize, n_s: usize) -> f64 {
    12.0 * n_c as f64
        + 1.00783 * n_h as f64
        + 14.003074 * n_n as f64
        + 15.994915 * n_o as f64
        + 31.97207 * n_s as f64
}

/// Calculate [M+H]+ m/z.
pub fn mh_plus(exact_mass: f64) -> f64 {
    exact_mass + 1.00728  // proton mass
}

/// Calculate [M-H]- m/z.
pub fn mh_minus(exact_mass: f64) -> f64 {
    exact_mass - 1.00728
}

/// Calculate [M+Na]+ m/z.
pub fn m_na_plus(exact_mass: f64) -> f64 {
    exact_mass + 22.98922
}

/// Double bond equivalence (DBE): (2C + 2 + N - H) / 2
pub fn double_bond_equivalence(n_c: usize, n_h: usize, n_n: usize) -> f64 {
    (2 * n_c + 2 + n_n - n_h) as f64 / 2.0
}

// ---------------------------------------------------------------------------
// 8. QuadrupoleAnalyzer Orchestrator
// ---------------------------------------------------------------------------

/// Orchestrator for quadrupole mass spectrometry analysis.
#[derive(Debug, Clone)]
pub struct QuadrupoleAnalyzer {
    pub config: QuadrupoleConfig,
    pub spectrum: Option<MassSpectrum>,
    pub calibration: (f64, f64, f64),
}

impl QuadrupoleAnalyzer {
    /// Create with standard config.
    pub fn new() -> Self {
        Self {
            config: QuadrupoleConfig::standard(),
            spectrum: None,
            calibration: (0.0, 1.0, 0.0),
        }
    }

    /// Create with custom config.
    pub fn with_config(config: QuadrupoleConfig) -> Self {
        Self {
            config,
            spectrum: None,
            calibration: (0.0, 1.0, 0.0),
        }
    }

    /// Set spectrum data.
    pub fn set_spectrum(&mut self, spec: MassSpectrum) {
        self.spectrum = Some(spec);
    }

    /// Calibrate using reference masses.
    pub fn calibrate(&mut self, raw_mz: &[f64], true_mz: &[f64]) {
        self.calibration = calibrate_mz_polynomial(raw_mz, true_mz);
    }

    /// Apply calibration to spectrum.
    pub fn apply_calibration(&mut self) {
        if let Some(ref mut spec) = self.spectrum {
            let (a0, a1, a2) = self.calibration;
            for mz in &mut spec.mz {
                *mz = apply_calibration(*mz, a0, a1, a2);
            }
        }
    }

    /// Find peaks in the current spectrum.
    pub fn find_peaks(&self, threshold: f64) -> Vec<MassPeak> {
        match &self.spectrum {
            Some(spec) => spec.find_peaks(threshold),
            None => Vec::new(),
        }
    }

    /// Simulate scan with given ions.
    pub fn simulate_scan(
        &mut self,
        ions: &[(f64, f64)],
        mz_start: f64,
        mz_end: f64,
        n_points: usize,
    ) -> &MassSpectrum {
        let spec = simulate_mass_scan(&self.config, ions, mz_start, mz_end, n_points);
        self.spectrum = Some(spec);
        self.spectrum.as_ref().unwrap()
    }
}

impl Default for QuadrupoleAnalyzer {
    fn default() -> Self {
        Self::new()
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

    #[test]
    fn test_mathieu_a() {
        let a: f64 = mathieu_a(10.0, 100.0, 0.00415, 2.0 * PI * 1.0e6);
        assert!(a > 0.0);
    }

    #[test]
    fn test_mathieu_q() {
        let q: f64 = mathieu_q(100.0, 100.0, 0.00415, 2.0 * PI * 1.0e6);
        assert!(q > 0.0);
    }

    #[test]
    fn test_stability_region() {
        assert!(is_stable_first_region(0.1, 0.5));
        assert!(!is_stable_first_region(0.5, 0.5)); // too high a
        assert!(!is_stable_first_region(0.1, 1.0)); // q > 0.908
    }

    #[test]
    fn test_mass_spectrum_base_peak() {
        let spec = MassSpectrum::new(
            vec![100.0, 150.0, 200.0],
            vec![50.0, 100.0, 30.0],
        );
        let (mz, int) = spec.base_peak();
        assert!(approx_eq(mz, 150.0, 0.01));
        assert!(approx_eq(int, 100.0, 0.01));
    }

    #[test]
    fn test_mass_spectrum_tic() {
        let spec = MassSpectrum::new(
            vec![100.0, 200.0, 300.0],
            vec![10.0, 20.0, 30.0],
        );
        assert!(approx_eq(spec.total_ion_current(), 60.0, 0.01));
    }

    #[test]
    fn test_normalize_to_base_peak() {
        let mut spec = MassSpectrum::new(
            vec![100.0, 200.0],
            vec![50.0, 100.0],
        );
        spec.normalize_to_base_peak();
        assert!(approx_eq(spec.intensities[0], 50.0, 0.01));
        assert!(approx_eq(spec.intensities[1], 100.0, 0.01));
    }

    #[test]
    fn test_find_peaks() {
        let spec = MassSpectrum::new(
            vec![100.0, 110.0, 120.0, 130.0, 140.0],
            vec![10.0, 50.0, 20.0, 80.0, 10.0],
        );
        let peaks = spec.find_peaks(15.0);
        assert_eq!(peaks.len(), 2);
    }

    #[test]
    fn test_extract_range() {
        let spec = MassSpectrum::new(
            vec![100.0, 200.0, 300.0, 400.0],
            vec![1.0, 2.0, 3.0, 4.0],
        );
        let sub = spec.extract_range(150.0, 350.0);
        assert_eq!(sub.mz.len(), 2);
    }

    #[test]
    fn test_carbon_isotope_pattern() {
        let pat = carbon_isotope_pattern(10);
        assert!(pat.abundances[0] > 0.8); // M+0 should be dominant
        assert!(pat.abundances.len() >= 2);
    }

    #[test]
    fn test_carbon_isotope_pattern_large() {
        let pat = carbon_isotope_pattern(100);
        // M+1 should be significant for 100 carbons
        assert!(pat.abundances[1] > 0.5);
    }

    #[test]
    fn test_match_isotope_pattern() {
        let obs: Vec<f64> = vec![1.0, 0.5, 0.1];
        let theo: Vec<f64> = vec![1.0, 0.5, 0.1];
        let sim: f64 = match_isotope_pattern(&obs, &theo);
        assert!(approx_eq(sim, 1.0, 0.001));
    }

    #[test]
    fn test_match_isotope_orthogonal() {
        let obs: Vec<f64> = vec![1.0, 0.0];
        let theo: Vec<f64> = vec![0.0, 1.0];
        let sim: f64 = match_isotope_pattern(&obs, &theo);
        assert!(approx_eq(sim, 0.0, 0.001));
    }

    #[test]
    fn test_quadrupole_config_standard() {
        let cfg = QuadrupoleConfig::standard();
        assert!(approx_eq(cfg.r0_m, 0.00415, 1e-5));
        assert!(cfg.resolution() > 0.0);
    }

    #[test]
    fn test_quadrupole_rf_for_mz() {
        let cfg = QuadrupoleConfig::standard();
        let rf_100: f64 = cfg.rf_for_mz(100.0);
        let rf_200: f64 = cfg.rf_for_mz(200.0);
        assert!(rf_200 > rf_100); // Higher mass needs more RF
    }

    #[test]
    fn test_quadrupole_transmission() {
        let cfg = QuadrupoleConfig::standard();
        let t_on: f64 = cfg.transmission(100.0, 100.0);
        let t_off: f64 = cfg.transmission(100.0, 200.0);
        assert!(approx_eq(t_on, 1.0, 0.01));
        assert!(t_off < 0.01);
    }

    #[test]
    fn test_simulate_mass_scan() {
        let cfg = QuadrupoleConfig::standard();
        let ions: Vec<(f64, f64)> = vec![(100.0, 1.0), (200.0, 0.5)];
        let spec = simulate_mass_scan(&cfg, &ions, 50.0, 250.0, 201);
        assert_eq!(spec.mz.len(), 201);
        let (bp_mz, _) = spec.base_peak();
        assert!(approx_eq(bp_mz, 100.0, 2.0));
    }

    #[test]
    fn test_mass_accuracy_ppm() {
        let ppm: f64 = mass_accuracy_ppm(100.005, 100.0);
        assert!(approx_eq(ppm, 50.0, 1.0));
    }

    #[test]
    fn test_mass_accuracy_mda() {
        let mda: f64 = mass_accuracy_mda(100.005, 100.0);
        assert!(approx_eq(mda, 5.0, 0.01));
    }

    #[test]
    fn test_calibrate_mz() {
        let raw: Vec<f64> = vec![99.5, 199.5, 299.5];
        let true_mz: Vec<f64> = vec![100.0, 200.0, 300.0];
        let (a0, a1, _) = calibrate_mz_polynomial(&raw, &true_mz);
        let cal: f64 = apply_calibration(99.5, a0, a1, 0.0);
        assert!(approx_eq(cal, 100.0, 0.1));
    }

    #[test]
    fn test_smooth_moving_average() {
        let data: Vec<f64> = vec![1.0, 10.0, 1.0, 10.0, 1.0, 10.0, 1.0];
        let smooth = smooth_moving_average(&data, 3);
        // Central values should be averaged
        assert!(smooth[3] > 3.0 && smooth[3] < 8.0);
    }

    #[test]
    fn test_baseline_rolling_minimum() {
        let data: Vec<f64> = vec![5.0, 15.0, 5.0, 20.0, 5.0];
        let corrected = baseline_rolling_minimum(&data, 3);
        // Peaks should remain, baseline removed
        assert!(corrected[1] > 0.0);
        assert!(corrected[3] > 0.0);
    }

    #[test]
    fn test_estimate_snr() {
        let noise: Vec<f64> = vec![1.0, 1.1, 0.9, 1.0, 1.1, 0.9];
        let snr: f64 = estimate_snr(100.0, &noise);
        assert!(snr > 100.0);
    }

    #[test]
    fn test_exact_mass_water() {
        let m: f64 = exact_mass(0, 2, 0, 1, 0);
        // H2O = 2*1.00783 + 15.995 = 18.011
        assert!(approx_eq(m, 18.011, 0.01));
    }

    #[test]
    fn test_exact_mass_benzene() {
        let m: f64 = exact_mass(6, 6, 0, 0, 0);
        // C6H6 = 72 + 6.047 = 78.047
        assert!(approx_eq(m, 78.047, 0.01));
    }

    #[test]
    fn test_mh_plus() {
        let m: f64 = exact_mass(6, 6, 0, 0, 0);
        let mh: f64 = mh_plus(m);
        assert!(mh > m);
        assert!(approx_eq(mh - m, 1.00728, 0.001));
    }

    #[test]
    fn test_mh_minus() {
        let m: f64 = exact_mass(6, 6, 0, 0, 0);
        let mhm: f64 = mh_minus(m);
        assert!(mhm < m);
    }

    #[test]
    fn test_m_na_plus() {
        let m: f64 = exact_mass(6, 6, 0, 0, 0);
        let mna: f64 = m_na_plus(m);
        assert!(approx_eq(mna - m, 22.989, 0.01));
    }

    #[test]
    fn test_dbe_benzene() {
        let dbe: f64 = double_bond_equivalence(6, 6, 0);
        // C6H6: (12+2-6)/2 = 4 (3 double bonds + 1 ring)
        assert!(approx_eq(dbe, 4.0, 0.01));
    }

    #[test]
    fn test_dbe_methane() {
        let dbe: f64 = double_bond_equivalence(1, 4, 0);
        // CH4: (2+2-4)/2 = 0
        assert!(approx_eq(dbe, 0.0, 0.01));
    }

    #[test]
    fn test_analyzer_new() {
        let qa = QuadrupoleAnalyzer::new();
        assert!(qa.spectrum.is_none());
    }

    #[test]
    fn test_analyzer_simulate() {
        let mut qa = QuadrupoleAnalyzer::new();
        let ions: Vec<(f64, f64)> = vec![(100.0, 1.0), (150.0, 0.7)];
        qa.simulate_scan(&ions, 50.0, 200.0, 151);
        assert!(qa.spectrum.is_some());
    }

    #[test]
    fn test_analyzer_find_peaks() {
        let mut qa = QuadrupoleAnalyzer::new();
        qa.simulate_scan(&[(100.0, 1.0), (150.0, 0.5)], 50.0, 200.0, 301);
        let peaks = qa.find_peaks(0.1);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_analyzer_calibrate() {
        let mut qa = QuadrupoleAnalyzer::new();
        qa.calibrate(&[99.0, 199.0, 299.0], &[100.0, 200.0, 300.0]);
        assert!(qa.calibration.1 > 0.9);
    }

    #[test]
    fn test_analyzer_default() {
        let qa = QuadrupoleAnalyzer::default();
        assert!(qa.spectrum.is_none());
    }

    #[test]
    fn test_binomial_coeff() {
        assert!(approx_eq(binomial_coeff(5, 2), 10.0, 0.01));
        assert!(approx_eq(binomial_coeff(10, 0), 1.0, 0.01));
        assert!(approx_eq(binomial_coeff(10, 10), 1.0, 0.01));
    }

    #[test]
    fn test_empty_noise() {
        let snr: f64 = estimate_snr(100.0, &[]);
        assert!(approx_eq(snr, 0.0, 0.01));
    }

    #[test]
    fn test_smooth_short() {
        let data: Vec<f64> = vec![1.0, 2.0];
        let smooth = smooth_moving_average(&data, 5);
        assert_eq!(smooth, data);
    }

    #[test]
    fn test_quadrupole_omega() {
        let cfg = QuadrupoleConfig::standard();
        assert!(approx_eq(cfg.omega(), 2.0 * PI * 1.0e6, 1.0));
    }

    #[test]
    fn test_mass_spectrum_single() {
        let spec = MassSpectrum::new(vec![100.0], vec![50.0]);
        let (mz, _) = spec.base_peak();
        assert!(approx_eq(mz, 100.0, 0.01));
    }

    #[test]
    fn test_isotope_pattern_c1() {
        let pat = carbon_isotope_pattern(1);
        assert!(approx_eq(pat.abundances[0], 1.0, 0.01));
        assert!(pat.abundances[1] < 0.02);
    }

    #[test]
    fn test_mass_accuracy_zero() {
        assert!(approx_eq(mass_accuracy_ppm(100.0, 100.0), 0.0, 0.01));
    }

    #[test]
    fn test_dc_for_rf() {
        let cfg = QuadrupoleConfig::standard();
        let rf: f64 = 100.0;
        let dc: f64 = cfg.dc_for_rf(rf);
        assert!(approx_eq(dc, 16.8, 0.01));
    }
}
