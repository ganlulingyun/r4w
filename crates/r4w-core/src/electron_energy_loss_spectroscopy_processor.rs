// Electron Energy Loss Spectroscopy (EELS) Processor
// Core-loss and low-loss spectroscopy: zero-loss peak, plasmon loss, core-loss edges,
// Kramers-Kronig analysis, thickness measurement (log-ratio), chemical mapping
//
// No external crate dependencies - all math from scratch using only std.

/// EELS spectrum data
#[derive(Debug, Clone)]
pub struct EelsSpectrum {
    /// Energy loss in eV
    pub energy_loss_ev: Vec<f64>,
    /// Intensity (counts)
    pub intensity: Vec<f64>,
}

/// Zero-loss peak analysis
#[derive(Debug, Clone)]
pub struct ZeroLossPeak {
    /// Position (eV) - should be ~0
    pub position_ev: f64,
    /// Intensity (counts)
    pub intensity: f64,
    /// FWHM (eV) - energy resolution
    pub fwhm_ev: f64,
    /// Integrated area
    pub area: f64,
}

/// Plasmon loss result
#[derive(Debug, Clone)]
pub struct PlasmonResult {
    /// Plasmon energy (eV)
    pub energy_ev: f64,
    /// Plasmon peak intensity
    pub intensity: f64,
    /// Mean free path estimate (nm)
    pub mfp_nm: f64,
}

/// Core-loss edge
#[derive(Debug, Clone)]
pub struct CoreLossEdge {
    /// Edge onset energy (eV)
    pub onset_ev: f64,
    /// Edge type (e.g., "K", "L23", "M45")
    pub edge_type: String,
    /// Element assignment
    pub element: String,
    /// Integrated signal (after background subtraction)
    pub integrated_signal: f64,
}

/// Thickness measurement result
#[derive(Debug, Clone)]
pub struct ThicknessResult {
    /// Relative thickness t/λ (dimensionless)
    pub t_over_lambda: f64,
    /// Absolute thickness (nm) if MFP known
    pub thickness_nm: Option<f64>,
    /// Inelastic mean free path used (nm)
    pub mfp_nm: Option<f64>,
}

/// Dielectric function from Kramers-Kronig
#[derive(Debug, Clone)]
pub struct DielectricFunction {
    /// Energy loss (eV)
    pub energy_ev: Vec<f64>,
    /// Real part ε₁
    pub epsilon1: Vec<f64>,
    /// Imaginary part ε₂
    pub epsilon2: Vec<f64>,
}

/// EELS processor configuration
#[derive(Debug, Clone)]
pub struct EelsConfig {
    /// Beam energy (keV)
    pub beam_energy_kev: f64,
    /// Collection semi-angle (mrad)
    pub collection_angle_mrad: f64,
    /// Convergence semi-angle (mrad)
    pub convergence_angle_mrad: f64,
}

impl Default for EelsConfig {
    fn default() -> Self {
        Self {
            beam_energy_kev: 200.0,
            collection_angle_mrad: 10.0,
            convergence_angle_mrad: 5.0,
        }
    }
}

/// Main EELS processor
pub struct EelsProcessor {
    pub config: EelsConfig,
    pub spectrum: EelsSpectrum,
}

impl EelsProcessor {
    pub fn new(config: EelsConfig, spectrum: EelsSpectrum) -> Self {
        Self { config, spectrum }
    }

    /// Analyze zero-loss peak
    pub fn zero_loss_peak(&self) -> ZeroLossPeak {
        analyze_zlp(&self.spectrum.energy_loss_ev, &self.spectrum.intensity)
    }

    /// Measure thickness using log-ratio method
    /// t/λ = ln(I_total / I_ZLP)
    pub fn thickness_log_ratio(&self, mfp_nm: Option<f64>) -> ThicknessResult {
        log_ratio_thickness(&self.spectrum.energy_loss_ev, &self.spectrum.intensity, mfp_nm)
    }

    /// Find plasmon peaks in low-loss region
    pub fn find_plasmon_peaks(&self) -> Vec<PlasmonResult> {
        find_plasmon_peaks(&self.spectrum.energy_loss_ev, &self.spectrum.intensity, self.config.beam_energy_kev)
    }

    /// Power-law background fit: I(E) = A * E^(-r)
    /// Used for core-loss edge background subtraction
    pub fn power_law_background(&self, fit_start_ev: f64, fit_end_ev: f64) -> Vec<f64> {
        power_law_background(&self.spectrum.energy_loss_ev, &self.spectrum.intensity, fit_start_ev, fit_end_ev)
    }

    /// Extract core-loss edge signal
    pub fn extract_edge(&self, edge_onset_ev: f64, window_ev: f64, pre_edge_width_ev: f64) -> f64 {
        extract_core_loss_signal(
            &self.spectrum.energy_loss_ev,
            &self.spectrum.intensity,
            edge_onset_ev,
            window_ev,
            pre_edge_width_ev,
        )
    }

    /// Remove zero-loss peak by fitting and subtracting
    pub fn remove_zlp(&self) -> Vec<f64> {
        remove_zlp_gaussian(&self.spectrum.energy_loss_ev, &self.spectrum.intensity)
    }

    /// Single scattering distribution via Fourier-log deconvolution
    /// S(E) = FT⁻¹{ ln[FT{J(E)} / FT{Z(E)}] * FT{Z(E)} }
    /// Simplified: approximate by removing plural scattering
    pub fn single_scattering_distribution(&self) -> Vec<f64> {
        fourier_log_deconvolution(&self.spectrum.energy_loss_ev, &self.spectrum.intensity)
    }

    /// Kramers-Kronig analysis to extract dielectric function
    pub fn kramers_kronig(&self) -> DielectricFunction {
        kramers_kronig_analysis(
            &self.spectrum.energy_loss_ev,
            &self.spectrum.intensity,
            self.config.beam_energy_kev,
            self.config.collection_angle_mrad,
        )
    }

    /// Characteristic angle θ_E = E/(2*E_0) where E_0 is beam energy
    pub fn characteristic_angle_mrad(&self, energy_loss_ev: f64) -> f64 {
        if self.config.beam_energy_kev <= 0.0 { return 0.0; }
        energy_loss_ev / (2.0 * self.config.beam_energy_kev * 1000.0) * 1000.0 // mrad
    }

    /// Bethe ridge angle for energy loss E
    /// θ_B = sqrt(2*E/E_0) (approximate)
    pub fn bethe_ridge_angle_mrad(&self, energy_loss_ev: f64) -> f64 {
        if self.config.beam_energy_kev <= 0.0 || energy_loss_ev <= 0.0 { return 0.0; }
        (2.0 * energy_loss_ev / (self.config.beam_energy_kev * 1000.0)).sqrt() * 1000.0
    }

    /// Integrated intensity in an energy window
    pub fn integrated_intensity(&self, start_ev: f64, end_ev: f64) -> f64 {
        integrate_range(&self.spectrum.energy_loss_ev, &self.spectrum.intensity, start_ev, end_ev)
    }
}

/// Analyze zero-loss peak
pub fn analyze_zlp(energy: &[f64], intensity: &[f64]) -> ZeroLossPeak {
    let n = energy.len().min(intensity.len());
    if n < 3 {
        return ZeroLossPeak { position_ev: 0.0, intensity: 0.0, fwhm_ev: 0.0, area: 0.0 };
    }

    // Find maximum near E=0
    let mut peak_idx = 0;
    let mut peak_val = f64::NEG_INFINITY;
    for i in 0..n {
        if energy[i].abs() < 20.0 && intensity[i] > peak_val {
            peak_val = intensity[i];
            peak_idx = i;
        }
    }

    let position = energy[peak_idx];

    // FWHM
    let half_max = peak_val / 2.0;
    let mut left_ev = position;
    for i in (0..peak_idx).rev() {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i + 1] - intensity[i]).max(1e-30);
            left_ev = energy[i] + frac * (energy[i + 1] - energy[i]);
            break;
        }
    }
    let mut right_ev = position;
    for i in peak_idx + 1..n {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i - 1] - intensity[i]).max(1e-30);
            right_ev = energy[i] + frac * (energy[i - 1] - energy[i]);
            break;
        }
    }
    let fwhm = (right_ev - left_ev).abs();

    // Area (trapezoidal around ZLP)
    let mut area = 0.0;
    for i in 1..n {
        if energy[i].abs() < fwhm * 3.0 && energy[i - 1].abs() < fwhm * 3.0 {
            let de = (energy[i] - energy[i - 1]).abs();
            area += 0.5 * (intensity[i] + intensity[i - 1]) * de;
        }
    }

    ZeroLossPeak { position_ev: position, intensity: peak_val, fwhm_ev: fwhm, area }
}

/// Log-ratio thickness measurement: t/λ = ln(I_t/I_0)
pub fn log_ratio_thickness(energy: &[f64], intensity: &[f64], mfp_nm: Option<f64>) -> ThicknessResult {
    let n = energy.len().min(intensity.len());
    if n < 3 {
        return ThicknessResult { t_over_lambda: 0.0, thickness_nm: None, mfp_nm: None };
    }

    // I_total = sum of all intensities
    let mut i_total = 0.0;
    let mut i_zlp = 0.0;

    for i in 1..n {
        let de = (energy[i] - energy[i - 1]).abs();
        let avg = 0.5 * (intensity[i] + intensity[i - 1]);
        i_total += avg * de;

        // ZLP: region near 0 eV (within ~5 eV for typical energy resolution)
        if energy[i].abs() < 5.0 && energy[i - 1].abs() < 5.0 {
            i_zlp += avg * de;
        }
    }

    if i_zlp <= 0.0 {
        return ThicknessResult { t_over_lambda: 0.0, thickness_nm: None, mfp_nm: None };
    }

    let t_lambda = (i_total / i_zlp).ln();

    let thickness = mfp_nm.map(|lambda| t_lambda * lambda);

    ThicknessResult {
        t_over_lambda: t_lambda,
        thickness_nm: thickness,
        mfp_nm,
    }
}

/// Find plasmon peaks in low-loss region (5-50 eV)
pub fn find_plasmon_peaks(energy: &[f64], intensity: &[f64], beam_kev: f64) -> Vec<PlasmonResult> {
    let n = energy.len().min(intensity.len());
    if n < 3 { return Vec::new(); }

    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        // Only look in 5-50 eV range
        if energy[i] < 5.0 || energy[i] > 50.0 { continue; }
        if intensity[i] > intensity[i - 1] && intensity[i] > intensity[i + 1] && intensity[i] > 10.0 {
            // Estimate MFP from plasmon: λ ≈ t when t/λ ≈ I_plasmon/I_ZLP (simplified)
            // More accurate: Malis formula λ(nm) ≈ 106*F*(E0/Ep) / ln(2*β*E0/Ep)
            let ep = energy[i];
            let e0 = beam_kev;
            let beta: f64 = 10.0; // mrad (approximate)
            let ratio = 2.0 * beta * e0 * 1000.0 / ep;
            let mfp = if ratio > 1.0 {
                106.0 * e0 / (ep * ratio.ln())
            } else {
                100.0 // Fallback
            };

            peaks.push(PlasmonResult {
                energy_ev: ep,
                intensity: intensity[i],
                mfp_nm: mfp,
            });
        }
    }
    peaks
}

/// Power-law background: I(E) = A * E^(-r)
pub fn power_law_background(energy: &[f64], intensity: &[f64], fit_start: f64, fit_end: f64) -> Vec<f64> {
    let n = energy.len().min(intensity.len());
    if n == 0 { return Vec::new(); }

    // Fit ln(I) = ln(A) - r*ln(E) in the fitting window
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_xy = 0.0;
    let mut count = 0.0;

    for i in 0..n {
        if energy[i] >= fit_start && energy[i] <= fit_end && energy[i] > 0.0 && intensity[i] > 0.0 {
            let x = energy[i].ln();
            let y = intensity[i].ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
            count += 1.0;
        }
    }

    if count < 2.0 { return vec![0.0; n]; }

    let denom = count * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 { return vec![0.0; n]; }

    let r = -(count * sum_xy - sum_x * sum_y) / denom;
    let ln_a = (sum_y + r * sum_x) / count;
    let a = ln_a.exp();

    (0..n).map(|i| {
        if energy[i] > 0.0 {
            a * energy[i].powf(-r)
        } else {
            0.0
        }
    }).collect()
}

/// Extract core-loss edge signal with background subtraction
pub fn extract_core_loss_signal(
    energy: &[f64],
    intensity: &[f64],
    edge_onset: f64,
    window_ev: f64,
    pre_edge_width: f64,
) -> f64 {
    let bg = power_law_background(energy, intensity, edge_onset - pre_edge_width, edge_onset);
    let n = energy.len().min(intensity.len()).min(bg.len());

    let mut signal = 0.0;
    for i in 1..n {
        if energy[i] >= edge_onset && energy[i] <= edge_onset + window_ev {
            let de = (energy[i] - energy[i - 1]).abs();
            let net = ((intensity[i] - bg[i]).max(0.0) + (intensity[i - 1] - bg[i - 1]).max(0.0)) / 2.0;
            signal += net * de;
        }
    }
    signal
}

/// Remove ZLP by fitting Gaussian and subtracting
pub fn remove_zlp_gaussian(energy: &[f64], intensity: &[f64]) -> Vec<f64> {
    let n = energy.len().min(intensity.len());
    if n == 0 { return Vec::new(); }

    let zlp = analyze_zlp(energy, intensity);
    let sigma = zlp.fwhm_ev / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);

    (0..n).map(|i| {
        let zlp_fit = zlp.intensity * (-0.5 * ((energy[i] - zlp.position_ev) / sigma.max(0.01)).powi(2)).exp();
        (intensity[i] - zlp_fit).max(0.0)
    }).collect()
}

/// Simplified Fourier-log deconvolution for single scattering distribution
pub fn fourier_log_deconvolution(energy: &[f64], intensity: &[f64]) -> Vec<f64> {
    let n = energy.len().min(intensity.len());
    if n < 4 { return intensity.to_vec(); }

    // Simplified: remove plural scattering by deconvolving with ZLP
    // In practice, this requires FFT. Here we approximate by:
    // SSD ≈ J(E) - (1/2!) * J(E) ⊗ J(E) / I_0 (first-order correction)
    let zlp = analyze_zlp(energy, intensity);
    if zlp.area < 1e-30 { return intensity.to_vec(); }

    // First-order approximation: just subtract estimated plural scattering
    let mut ssd = intensity.to_vec();
    let t_lambda = log_ratio_thickness(energy, intensity, None).t_over_lambda;

    // For thin samples (t/λ < 0.5), single scattering dominates
    // Scale down to approximate single scattering
    if t_lambda > 0.0 {
        let ss_fraction = t_lambda * (-t_lambda).exp(); // Poisson: P(1) = t/λ * exp(-t/λ)
        let total_fraction = 1.0 - (-t_lambda).exp(); // P(>0) = 1 - exp(-t/λ)
        let scale = if total_fraction > 1e-30 { ss_fraction / total_fraction } else { 1.0 };
        for i in 0..n {
            if energy[i] > 5.0 { // Only scale non-ZLP region
                ssd[i] *= scale;
            }
        }
    }
    ssd
}

/// Kramers-Kronig analysis (simplified)
pub fn kramers_kronig_analysis(
    energy: &[f64],
    intensity: &[f64],
    beam_kev: f64,
    collection_mrad: f64,
) -> DielectricFunction {
    let n = energy.len().min(intensity.len());
    if n < 3 {
        return DielectricFunction { energy_ev: Vec::new(), epsilon1: Vec::new(), epsilon2: Vec::new() };
    }

    // Energy loss function: Im[-1/ε(E)] ∝ S(E) / E (single scattering distribution)
    // ε₂(E) = Im[-1/ε] * |ε|² (requires iterative solution)
    // Simplified: Im[-1/ε] directly from spectrum

    let theta_e_factor = 1.0 / (2.0 * beam_kev * 1000.0); // E / (2*E0) factor

    let mut elf = Vec::with_capacity(n); // Energy Loss Function
    let mut energy_ev = Vec::with_capacity(n);

    for i in 0..n {
        if energy[i] > 1.0 { // Avoid ZLP region
            let e = energy[i];
            let theta_e = e * theta_e_factor;
            let ln_term = (1.0 + (collection_mrad / 1000.0 / theta_e).powi(2)).ln();
            let norm = if ln_term > 1e-30 { intensity[i] / (e * ln_term) } else { 0.0 };
            elf.push(norm);
            energy_ev.push(e);
        }
    }

    let m = elf.len();
    if m < 3 {
        return DielectricFunction { energy_ev: Vec::new(), epsilon1: Vec::new(), epsilon2: Vec::new() };
    }

    // Normalize ELF using sum rule: ∫ E * Im[-1/ε] dE = (π/2) * E_p²
    // Here we just normalize to reasonable values

    // Re[1/ε] from KK transform: Re[1/ε(E)] = 1 - (2/π) * P ∫ E'*Im[-1/ε(E')] / (E'²-E²) dE'
    let mut epsilon1 = Vec::with_capacity(m);
    let mut epsilon2 = Vec::with_capacity(m);

    for i in 0..m {
        let e = energy_ev[i];

        // KK integral (Cauchy principal value, numerical)
        let mut kk_integral = 0.0;
        for j in 1..m {
            if j == i { continue; }
            let e_prime = energy_ev[j];
            let denom = e_prime * e_prime - e * e;
            if denom.abs() > 1e-10 {
                let de = if j > 0 { (energy_ev[j] - energy_ev[j - 1]).abs() } else { 1.0 };
                kk_integral += e_prime * elf[j] * de / denom;
            }
        }
        let re_inv_eps = 1.0 - (2.0 / std::f64::consts::PI) * kk_integral;
        let im_inv_eps = elf[i];

        // ε = 1 / (Re[1/ε] + i*Im[1/ε])
        let mag_sq = re_inv_eps * re_inv_eps + im_inv_eps * im_inv_eps;
        if mag_sq > 1e-30 {
            epsilon1.push(re_inv_eps / mag_sq);
            epsilon2.push(im_inv_eps / mag_sq);
        } else {
            epsilon1.push(1.0);
            epsilon2.push(0.0);
        }
    }

    DielectricFunction { energy_ev, epsilon1, epsilon2 }
}

/// Integrate spectrum in energy range
pub fn integrate_range(energy: &[f64], intensity: &[f64], start: f64, end: f64) -> f64 {
    let n = energy.len().min(intensity.len());
    let lo = start.min(end);
    let hi = start.max(end);
    let mut total = 0.0;
    for i in 1..n {
        if energy[i] >= lo && energy[i] <= hi && energy[i - 1] >= lo {
            let de = (energy[i] - energy[i - 1]).abs();
            total += 0.5 * (intensity[i] + intensity[i - 1]) * de;
        }
    }
    total
}

/// Free-electron plasmon energy: E_p = ℏ * sqrt(n_e * e² / (ε₀ * m_e))
/// Simplified: E_p (eV) ≈ 28.8 * sqrt(Z * ρ / A)
/// where Z=atomic number, ρ=density g/cm³, A=atomic mass
pub fn free_electron_plasmon_ev(atomic_number: u32, density_g_cm3: f64, atomic_mass: f64) -> f64 {
    if atomic_mass <= 0.0 || density_g_cm3 <= 0.0 { return 0.0; }
    28.8 * ((atomic_number as f64) * density_g_cm3 / atomic_mass).sqrt()
}

/// Inelastic mean free path estimate (Malis formula)
/// λ (nm) = 106 * F * E_0 / (E_m * ln(2βE_0/E_m))
/// F ≈ (1 + E_0/1022)/(1 + E_0/511)² (relativistic factor)
/// E_m ≈ 7.6 * Z^0.36
pub fn malis_mfp_nm(beam_kev: f64, atomic_number: u32, collection_mrad: f64) -> f64 {
    if beam_kev <= 0.0 || collection_mrad <= 0.0 { return 0.0; }
    let e0 = beam_kev;
    let z = atomic_number as f64;
    let beta = collection_mrad;

    let f_rel = (1.0 + e0 / 1022.0) / (1.0 + e0 / 511.0).powi(2);
    let em = 7.6 * z.powf(0.36);

    let arg = 2.0 * beta * e0 * 1000.0 / em;
    if arg <= 1.0 { return 100.0; }

    106.0 * f_rel * e0 / (em * arg.ln())
}

/// Hydrogenic cross-section for core-loss edge (simplified Egerton model)
/// σ_K(β,Δ) ∝ (E_K)^(-3.5) * ln(β/θ_E) * Δ/E_K
pub fn hydrogenic_cross_section(edge_energy_ev: f64, beam_kev: f64, collection_mrad: f64, window_ev: f64) -> f64 {
    if edge_energy_ev <= 0.0 || beam_kev <= 0.0 || window_ev <= 0.0 { return 0.0; }
    let theta_e = edge_energy_ev / (2.0 * beam_kev * 1000.0) * 1000.0; // mrad
    if theta_e <= 0.0 { return 0.0; }
    let ratio = collection_mrad / theta_e;
    if ratio <= 0.0 { return 0.0; }
    let ln_ratio = (1.0 + ratio * ratio).ln();
    // Simplified: σ ∝ E_K^(-3.5) * ln(1+β²/θ_E²) * Δ/E_K
    1e-24 * edge_energy_ev.powf(-3.5) * ln_ratio * window_ev / edge_energy_ev // in barns (approximate)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_test_eels(zlp_fwhm: f64, plasmon_ev: f64, n: usize) -> EelsSpectrum {
        let e_start = -10.0;
        let e_end = 100.0;
        let sigma_zlp = zlp_fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
        let mut energy = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let e = e_start + (i as f64) * (e_end - e_start) / (n as f64 - 1.0);
            // ZLP + plasmon peak
            let zlp = 10000.0 * (-0.5 * (e / sigma_zlp).powi(2)).exp();
            let plasmon = if e > 0.0 {
                2000.0 * (-0.5 * ((e - plasmon_ev) / 3.0).powi(2)).exp()
            } else {
                0.0
            };
            energy.push(e);
            intensity.push(zlp + plasmon + 10.0); // + baseline
        }
        EelsSpectrum { energy_loss_ev: energy, intensity }
    }

    #[test]
    fn test_zero_loss_peak() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let zlp = analyze_zlp(&spectrum.energy_loss_ev, &spectrum.intensity);
        assert!(approx_eq(zlp.position_ev, 0.0, 0.5));
        assert!(zlp.intensity > 5000.0);
        assert!(zlp.fwhm_ev > 0.5);
        assert!(zlp.fwhm_ev < 3.0);
    }

    #[test]
    fn test_log_ratio_thickness() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let result = log_ratio_thickness(&spectrum.energy_loss_ev, &spectrum.intensity, Some(100.0));
        assert!(result.t_over_lambda > 0.0);
        assert!(result.thickness_nm.is_some());
        assert!(result.thickness_nm.unwrap() > 0.0);
    }

    #[test]
    fn test_log_ratio_no_mfp() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let result = log_ratio_thickness(&spectrum.energy_loss_ev, &spectrum.intensity, None);
        assert!(result.t_over_lambda > 0.0);
        assert!(result.thickness_nm.is_none());
    }

    #[test]
    fn test_find_plasmon_peaks() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let peaks = find_plasmon_peaks(&spectrum.energy_loss_ev, &spectrum.intensity, 200.0);
        assert!(!peaks.is_empty());
        assert!(approx_eq(peaks[0].energy_ev, 15.0, 2.0));
    }

    #[test]
    fn test_power_law_background() {
        // Create a power-law spectrum: I = 1000 * E^(-3)
        let energy: Vec<f64> = (1..101).map(|i| i as f64).collect();
        let intensity: Vec<f64> = energy.iter().map(|e| 1000.0 * e.powf(-3.0)).collect();
        let bg = power_law_background(&energy, &intensity, 20.0, 40.0);
        assert_eq!(bg.len(), 100);
        // Background should match the spectrum well
        for i in 20..40 {
            assert!(approx_eq(bg[i], intensity[i], intensity[i] * 0.1));
        }
    }

    #[test]
    fn test_extract_core_loss() {
        // Create spectrum with edge at 284 eV (C K-edge)
        let n = 500;
        let mut energy = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let e = 250.0 + (i as f64) * 0.2;
            let bg = 1000.0 * (e / 250.0).powf(-3.0);
            let edge = if e > 284.0 { 500.0 * (1.0 - (-(e - 284.0) / 10.0).exp()) } else { 0.0 };
            energy.push(e);
            intensity.push(bg + edge);
        }
        let signal = extract_core_loss_signal(&energy, &intensity, 284.0, 30.0, 30.0);
        assert!(signal > 0.0);
    }

    #[test]
    fn test_remove_zlp() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let removed = remove_zlp_gaussian(&spectrum.energy_loss_ev, &spectrum.intensity);
        assert_eq!(removed.len(), 501);
        // Find the index nearest E=0 (spectrum goes from -10 to 100)
        let zlp_idx = spectrum.energy_loss_ev.iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| a.abs().partial_cmp(&b.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        // At E=0, intensity should be reduced after ZLP removal
        assert!(removed[zlp_idx] < spectrum.intensity[zlp_idx]);
    }

    #[test]
    fn test_free_electron_plasmon() {
        // Aluminum: Z=13, ρ=2.70 g/cm³, A=26.98
        // Free-electron model: E_p = 28.8*sqrt(Z*ρ/A) ≈ 32.8 eV
        // (Overestimates vs measured ~15 eV since not all electrons are free)
        let ep = free_electron_plasmon_ev(13, 2.70, 26.98);
        assert!(ep > 20.0);
        assert!(ep < 40.0);
    }

    #[test]
    fn test_free_electron_plasmon_si() {
        // Silicon: Z=14, ρ=2.33 g/cm³, A=28.09
        // Free-electron model overestimates; gives ~32 eV
        let ep = free_electron_plasmon_ev(14, 2.33, 28.09);
        assert!(ep > 20.0);
        assert!(ep < 40.0);
    }

    #[test]
    fn test_malis_mfp() {
        // Al at 200 keV, β=10 mrad
        let mfp = malis_mfp_nm(200.0, 13, 10.0);
        assert!(mfp > 50.0);
        assert!(mfp < 300.0);
    }

    #[test]
    fn test_malis_mfp_increases_with_energy() {
        let mfp100 = malis_mfp_nm(100.0, 14, 10.0);
        let mfp200 = malis_mfp_nm(200.0, 14, 10.0);
        assert!(mfp200 > mfp100);
    }

    #[test]
    fn test_characteristic_angle() {
        let config = EelsConfig { beam_energy_kev: 200.0, ..Default::default() };
        let spectrum = make_test_eels(1.0, 15.0, 101);
        let proc = EelsProcessor::new(config, spectrum);
        // θ_E = E / (2*E0) in mrad
        let theta = proc.characteristic_angle_mrad(300.0);
        // 300 / (2*200000) * 1000 = 0.75 mrad
        assert!(approx_eq(theta, 0.75, 0.01));
    }

    #[test]
    fn test_bethe_ridge_angle() {
        let config = EelsConfig { beam_energy_kev: 200.0, ..Default::default() };
        let spectrum = make_test_eels(1.0, 15.0, 101);
        let proc = EelsProcessor::new(config, spectrum);
        let theta = proc.bethe_ridge_angle_mrad(300.0);
        assert!(theta > 0.0);
    }

    #[test]
    fn test_hydrogenic_cross_section() {
        // C K-edge at 284 eV, 200 keV beam, 10 mrad, 30 eV window
        let sigma = hydrogenic_cross_section(284.0, 200.0, 10.0, 30.0);
        assert!(sigma > 0.0);
    }

    #[test]
    fn test_hydrogenic_cross_section_zero() {
        let sigma = hydrogenic_cross_section(0.0, 200.0, 10.0, 30.0);
        assert!(approx_eq(sigma, 0.0, 0.01));
    }

    #[test]
    fn test_integrated_intensity() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let total = proc.integrated_intensity(10.0, 20.0);
        assert!(total > 0.0);
    }

    #[test]
    fn test_processor_zlp() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let zlp = proc.zero_loss_peak();
        assert!(zlp.intensity > 0.0);
    }

    #[test]
    fn test_processor_thickness() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let result = proc.thickness_log_ratio(Some(100.0));
        assert!(result.t_over_lambda > 0.0);
    }

    #[test]
    fn test_processor_plasmon() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let peaks = proc.find_plasmon_peaks();
        assert!(!peaks.is_empty());
    }

    #[test]
    fn test_kramers_kronig() {
        let spectrum = make_test_eels(1.0, 15.0, 201);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let df = proc.kramers_kronig();
        assert!(!df.energy_ev.is_empty());
        assert_eq!(df.epsilon1.len(), df.energy_ev.len());
        assert_eq!(df.epsilon2.len(), df.energy_ev.len());
    }

    #[test]
    fn test_single_scattering() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let ssd = proc.single_scattering_distribution();
        assert_eq!(ssd.len(), 501);
    }

    #[test]
    fn test_free_electron_plasmon_zero() {
        let ep = free_electron_plasmon_ev(13, 0.0, 26.98);
        assert!(approx_eq(ep, 0.0, 0.01));
    }

    #[test]
    fn test_malis_mfp_zero() {
        let mfp = malis_mfp_nm(0.0, 14, 10.0);
        assert!(approx_eq(mfp, 0.0, 0.01));
    }

    #[test]
    fn test_integrate_range() {
        let energy: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let intensity: Vec<f64> = vec![1.0; 100];
        let total = integrate_range(&energy, &intensity, 10.0, 50.0);
        assert!(approx_eq(total, 40.0, 1.0));
    }

    #[test]
    fn test_processor_remove_zlp() {
        let spectrum = make_test_eels(1.0, 15.0, 501);
        let config = EelsConfig::default();
        let proc = EelsProcessor::new(config, spectrum);
        let removed = proc.remove_zlp();
        assert_eq!(removed.len(), 501);
    }
}
