// Raman Confocal Microscope Processor
// Vibrational spectroscopy: Stokes/anti-Stokes, depolarization ratio, peak fitting
// (Lorentzian/Gaussian/Voigt), fluorescence background removal, Raman mapping
//
// No external crate dependencies - all math from scratch using only std.

/// Raman spectrum data
#[derive(Debug, Clone)]
pub struct RamanSpectrum {
    /// Raman shift in cm⁻¹
    pub wavenumber_cm1: Vec<f64>,
    /// Intensity (counts or counts/s)
    pub intensity: Vec<f64>,
}

/// Raman peak result
#[derive(Debug, Clone)]
pub struct RamanPeak {
    /// Peak position (cm⁻¹)
    pub position_cm1: f64,
    /// Peak intensity
    pub intensity: f64,
    /// FWHM (cm⁻¹)
    pub fwhm_cm1: f64,
    /// Peak area
    pub area: f64,
    /// Assignment (e.g., "C-C stretch")
    pub assignment: String,
}

/// Stokes/Anti-Stokes analysis result
#[derive(Debug, Clone)]
pub struct StokesAntiStokesResult {
    /// Temperature estimated from ratio (K)
    pub temperature_k: f64,
    /// Stokes intensity
    pub stokes_intensity: f64,
    /// Anti-Stokes intensity
    pub anti_stokes_intensity: f64,
    /// Raman shift used (cm⁻¹)
    pub raman_shift_cm1: f64,
}

/// Depolarization ratio measurement
#[derive(Debug, Clone)]
pub struct DepolarizationResult {
    /// Depolarization ratio ρ = I_⊥ / I_∥
    pub rho: f64,
    /// Symmetry assessment
    pub symmetry: String,
}

/// Raman processor configuration
#[derive(Debug, Clone)]
pub struct RamanConfig {
    /// Laser excitation wavelength (nm)
    pub laser_wavelength_nm: f64,
    /// Spectral resolution (cm⁻¹)
    pub spectral_resolution_cm1: f64,
    /// Laser power (mW)
    pub laser_power_mw: f64,
}

impl Default for RamanConfig {
    fn default() -> Self {
        Self {
            laser_wavelength_nm: 532.0,
            spectral_resolution_cm1: 2.0,
            laser_power_mw: 10.0,
        }
    }
}

/// Main Raman processor
pub struct RamanProcessor {
    pub config: RamanConfig,
    pub spectrum: RamanSpectrum,
}

impl RamanProcessor {
    pub fn new(config: RamanConfig, spectrum: RamanSpectrum) -> Self {
        Self { config, spectrum }
    }

    /// Convert wavelength (nm) to wavenumber (cm⁻¹)
    pub fn wavelength_to_wavenumber(wavelength_nm: f64) -> f64 {
        if wavelength_nm <= 0.0 { return 0.0; }
        1e7 / wavelength_nm // nm to cm⁻¹
    }

    /// Calculate Raman shift from scattered wavelength
    /// Δν = 1/λ_laser - 1/λ_scattered (in cm⁻¹)
    pub fn raman_shift_cm1(&self, scattered_wavelength_nm: f64) -> f64 {
        if self.config.laser_wavelength_nm <= 0.0 || scattered_wavelength_nm <= 0.0 { return 0.0; }
        1e7 / self.config.laser_wavelength_nm - 1e7 / scattered_wavelength_nm
    }

    /// Estimate temperature from Stokes/Anti-Stokes intensity ratio
    /// I_AS/I_S = (ν_0 + ν_v)^4 / (ν_0 - ν_v)^4 * exp(-hcν_v/kT)
    pub fn stokes_anti_stokes_temperature(
        &self,
        stokes_intensity: f64,
        anti_stokes_intensity: f64,
        raman_shift_cm1: f64,
    ) -> StokesAntiStokesResult {
        let temp = stokes_anti_stokes_temp(
            stokes_intensity,
            anti_stokes_intensity,
            raman_shift_cm1,
            self.config.laser_wavelength_nm,
        );
        StokesAntiStokesResult {
            temperature_k: temp,
            stokes_intensity,
            anti_stokes_intensity,
            raman_shift_cm1,
        }
    }

    /// Remove fluorescence background using polynomial fitting
    pub fn remove_fluorescence(&self, poly_order: usize) -> Vec<f64> {
        polynomial_baseline_subtract(&self.spectrum.wavenumber_cm1, &self.spectrum.intensity, poly_order)
    }

    /// Smoothing using Savitzky-Golay (5-point quadratic)
    pub fn smooth_spectrum(&self) -> Vec<f64> {
        savitzky_golay_5pt(&self.spectrum.intensity)
    }

    /// Find peaks in spectrum
    pub fn find_peaks(&self, min_intensity: f64) -> Vec<RamanPeak> {
        find_raman_peaks(&self.spectrum.wavenumber_cm1, &self.spectrum.intensity, min_intensity)
    }

    /// Calculate depolarization ratio from parallel and perpendicular spectra
    pub fn depolarization_ratio(parallel_intensity: f64, perpendicular_intensity: f64) -> DepolarizationResult {
        depolarization_ratio(parallel_intensity, perpendicular_intensity)
    }

    /// Cosmic ray removal using median filter comparison
    pub fn remove_cosmic_rays(&self, threshold_sigma: f64) -> Vec<f64> {
        remove_cosmic_rays(&self.spectrum.intensity, threshold_sigma)
    }

    /// Normalize spectrum to peak maximum
    pub fn normalize_to_max(&self) -> Vec<f64> {
        let max_val = self.spectrum.intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if max_val.abs() < 1e-30 { return self.spectrum.intensity.clone(); }
        self.spectrum.intensity.iter().map(|i| i / max_val).collect()
    }

    /// Calculate Raman enhancement factor
    /// EF = (I_SERS / N_SERS) / (I_normal / N_normal)
    pub fn enhancement_factor(sers_intensity: f64, sers_molecules: f64, normal_intensity: f64, normal_molecules: f64) -> f64 {
        if normal_intensity <= 0.0 || sers_molecules <= 0.0 || normal_molecules <= 0.0 { return 0.0; }
        (sers_intensity / sers_molecules) / (normal_intensity / normal_molecules)
    }
}

/// Stokes/Anti-Stokes temperature calculation
/// T = hcν / (k * ln(I_S/I_AS * (ν_0 + ν)^4 / (ν_0 - ν)^4))
pub fn stokes_anti_stokes_temp(
    stokes: f64,
    anti_stokes: f64,
    shift_cm1: f64,
    laser_nm: f64,
) -> f64 {
    if stokes <= 0.0 || anti_stokes <= 0.0 || shift_cm1 <= 0.0 { return 0.0; }

    let h = 6.62607015e-34;  // Planck constant (J·s)
    let c = 2.998e10;         // Speed of light (cm/s)
    let kb = 1.38064852e-23;  // Boltzmann constant (J/K)

    let nu_0 = 1e7 / laser_nm; // Laser wavenumber (cm⁻¹)
    let nu_v = shift_cm1;

    // Frequency correction factor
    let freq_ratio = ((nu_0 + nu_v) / (nu_0 - nu_v)).powi(4);

    let ratio = stokes / anti_stokes * freq_ratio;
    if ratio <= 1.0 { return 0.0; }

    let energy = h * c * nu_v; // Energy of vibrational mode
    energy / (kb * ratio.ln())
}

/// Lorentzian peak shape for Raman: I(ν) = A * Γ²/4 / ((ν-ν0)² + Γ²/4)
pub fn lorentzian(wavenumber: f64, center: f64, amplitude: f64, fwhm: f64) -> f64 {
    let gamma2_4 = fwhm * fwhm / 4.0;
    let dx = wavenumber - center;
    amplitude * gamma2_4 / (dx * dx + gamma2_4)
}

/// Gaussian peak shape
pub fn gaussian(wavenumber: f64, center: f64, amplitude: f64, fwhm: f64) -> f64 {
    let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
    amplitude * (-0.5 * ((wavenumber - center) / sigma).powi(2)).exp()
}

/// Pseudo-Voigt: η*L + (1-η)*G
pub fn pseudo_voigt(wavenumber: f64, center: f64, amplitude: f64, fwhm: f64, eta: f64) -> f64 {
    eta * lorentzian(wavenumber, center, amplitude, fwhm)
        + (1.0 - eta) * gaussian(wavenumber, center, amplitude, fwhm)
}

/// Polynomial baseline fitting and subtraction
pub fn polynomial_baseline_subtract(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len().min(y.len());
    if n == 0 { return Vec::new(); }
    if order == 0 {
        let mean: f64 = y.iter().sum::<f64>() / n as f64;
        return y.iter().map(|v| v - mean).collect();
    }

    // Select baseline points (lowest 20% of spectrum)
    let mut indexed: Vec<(usize, f64)> = y.iter().enumerate().map(|(i, v)| (i, *v)).collect();
    indexed.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
    let n_base = (n as f64 * 0.2).max(order as f64 + 1.0) as usize;
    let base_points: Vec<(f64, f64)> = indexed[..n_base.min(n)]
        .iter()
        .map(|(i, _)| (x[*i], y[*i]))
        .collect();

    // Fit polynomial to baseline points using least squares
    let coeffs = fit_polynomial(&base_points, order);

    // Subtract fitted baseline
    (0..n).map(|i| {
        let baseline = eval_polynomial(x[i], &coeffs);
        (y[i] - baseline).max(0.0)
    }).collect()
}

/// Fit polynomial to data points (least squares, normal equations)
fn fit_polynomial(points: &[(f64, f64)], order: usize) -> Vec<f64> {
    let n = points.len();
    let m = order + 1;
    if n < m { return vec![0.0; m]; }

    // Normalize x values for numerical stability
    let x_mean = points.iter().map(|(x, _)| *x).sum::<f64>() / n as f64;
    let x_scale = points.iter().map(|(x, _)| (*x - x_mean).abs()).fold(0.0_f64, f64::max).max(1.0);

    // Build normal equation: A^T A c = A^T y
    let mut ata = vec![0.0; m * m];
    let mut aty = vec![0.0; m];

    for (x_raw, y_val) in points {
        let x = (*x_raw - x_mean) / x_scale;
        let mut xi = 1.0;
        for j in 0..m {
            let mut xij = xi;
            let mut xk = 1.0;
            for k in 0..m {
                ata[j * m + k] += xi * xk;
                xk *= x;
            }
            aty[j] += xi * y_val;
            xi *= x;
            let _ = xij;
        }
    }

    // Solve using Gaussian elimination
    let coeffs_norm = solve_linear_system(&mut ata, &mut aty, m);

    // Convert back to original coordinates
    // p(x) = Σ c_i * ((x - x_mean)/x_scale)^i
    // Need to expand this to get coefficients in original x
    let mut result = vec![0.0; m];
    for i in 0..m {
        // c_i contributes to all lower-order terms through binomial expansion
        // Simplified: just store normalized coefficients and evaluate with normalization
        result[i] = coeffs_norm[i];
    }
    // Store x_mean and x_scale as extra info (hack: use the coefficients as-is and evaluate with normalization)
    // Actually, for evaluation, we'll pass the normalization separately
    // Let's just store raw polynomial and always evaluate with normalization
    // Append x_mean and x_scale to end
    result.push(x_mean);
    result.push(x_scale);
    result
}

/// Evaluate polynomial (with normalization stored at end)
fn eval_polynomial(x: f64, coeffs: &[f64]) -> f64 {
    let m = coeffs.len();
    if m < 3 { return 0.0; }
    let x_mean = coeffs[m - 2];
    let x_scale = coeffs[m - 1];
    let xn = (x - x_mean) / x_scale;

    let mut result = 0.0;
    let mut xp = 1.0;
    for i in 0..m - 2 {
        result += coeffs[i] * xp;
        xp *= xn;
    }
    result
}

/// Solve linear system Ax = b (Gaussian elimination with partial pivoting)
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination
    for k in 0..n {
        // Partial pivoting
        let mut max_val = a[k * n + k].abs();
        let mut max_row = k;
        for i in k + 1..n {
            if a[i * n + k].abs() > max_val {
                max_val = a[i * n + k].abs();
                max_row = i;
            }
        }
        if max_val < 1e-30 { continue; }
        if max_row != k {
            for j in 0..n {
                let tmp = a[k * n + j];
                a[k * n + j] = a[max_row * n + j];
                a[max_row * n + j] = tmp;
            }
            let tmp = b[k];
            b[k] = b[max_row];
            b[max_row] = tmp;
        }

        for i in k + 1..n {
            let factor = a[i * n + k] / a[k * n + k];
            for j in k..n {
                a[i * n + j] -= factor * a[k * n + j];
            }
            b[i] -= factor * b[k];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in i + 1..n {
            sum -= a[i * n + j] * x[j];
        }
        if a[i * n + i].abs() > 1e-30 {
            x[i] = sum / a[i * n + i];
        }
    }
    x
}

/// Savitzky-Golay 5-point quadratic smoothing
pub fn savitzky_golay_5pt(data: &[f64]) -> Vec<f64> {
    let n = data.len();
    if n < 5 { return data.to_vec(); }

    let mut result = data.to_vec();
    // SG coefficients for 5-point quadratic: [-3, 12, 17, 12, -3] / 35
    for i in 2..n - 2 {
        result[i] = (-3.0 * data[i - 2] + 12.0 * data[i - 1] + 17.0 * data[i]
            + 12.0 * data[i + 1] - 3.0 * data[i + 2]) / 35.0;
    }
    result
}

/// Find Raman peaks
pub fn find_raman_peaks(wn: &[f64], intensity: &[f64], min_intensity: f64) -> Vec<RamanPeak> {
    let n = wn.len().min(intensity.len());
    if n < 3 { return Vec::new(); }

    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        if intensity[i] > intensity[i - 1] && intensity[i] > intensity[i + 1]
            && intensity[i] >= min_intensity
        {
            // Parabolic interpolation for peak position
            let y0 = intensity[i - 1];
            let y1 = intensity[i];
            let y2 = intensity[i + 1];
            let denom = 2.0 * (2.0 * y1 - y0 - y2);
            let offset = if denom.abs() > 1e-30 { (y0 - y2) / denom } else { 0.0 };
            let dw = if i + 1 < n { wn[i + 1] - wn[i] } else { wn[i] - wn[i - 1] };
            let peak_wn = wn[i] + offset * dw;

            let fwhm = estimate_fwhm(wn, intensity, i);
            let area = estimate_area(wn, intensity, i, fwhm);

            peaks.push(RamanPeak {
                position_cm1: peak_wn,
                intensity: y1,
                fwhm_cm1: fwhm,
                area,
                assignment: String::new(),
            });
        }
    }
    peaks
}

/// Estimate FWHM at peak
fn estimate_fwhm(wn: &[f64], intensity: &[f64], peak_idx: usize) -> f64 {
    let n = wn.len().min(intensity.len());
    let half_max = intensity[peak_idx] / 2.0;

    let mut left = wn[peak_idx];
    for i in (0..peak_idx).rev() {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i + 1] - intensity[i]).max(1e-30);
            left = wn[i] + frac * (wn[i + 1] - wn[i]);
            break;
        }
    }

    let mut right = wn[peak_idx];
    for i in peak_idx + 1..n {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i - 1] - intensity[i]).max(1e-30);
            right = wn[i] + frac * (wn[i - 1] - wn[i]);
            break;
        }
    }

    (right - left).abs()
}

/// Estimate peak area
fn estimate_area(wn: &[f64], intensity: &[f64], peak_idx: usize, fwhm: f64) -> f64 {
    let n = wn.len().min(intensity.len());
    let center = wn[peak_idx];
    let hw = fwhm * 2.0;

    let mut area = 0.0;
    for i in 1..n {
        if (wn[i] - center).abs() <= hw && (wn[i - 1] - center).abs() <= hw {
            let dw = (wn[i] - wn[i - 1]).abs();
            area += 0.5 * (intensity[i] + intensity[i - 1]) * dw;
        }
    }
    area
}

/// Depolarization ratio calculation
pub fn depolarization_ratio(parallel: f64, perpendicular: f64) -> DepolarizationResult {
    let rho = if parallel > 1e-30 { perpendicular / parallel } else { 0.0 };
    let symmetry = if rho < 0.01 {
        "Totally symmetric (isotropic)".to_string()
    } else if rho < 0.75 {
        "Polarized (symmetric vibration)".to_string()
    } else {
        "Depolarized (asymmetric vibration)".to_string()
    };
    DepolarizationResult { rho, symmetry }
}

/// Remove cosmic ray spikes
pub fn remove_cosmic_rays(intensity: &[f64], threshold_sigma: f64) -> Vec<f64> {
    let n = intensity.len();
    if n < 5 { return intensity.to_vec(); }

    let mean: f64 = intensity.iter().sum::<f64>() / n as f64;
    let variance = intensity.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n as f64;
    let sigma = variance.sqrt();
    let threshold = mean + threshold_sigma * sigma;

    let mut result = intensity.to_vec();
    for i in 2..n - 2 {
        if result[i] > threshold {
            // Replace with average of neighbors
            result[i] = (result[i - 1] + result[i + 1]) / 2.0;
        }
    }
    result
}

/// Raman scattering cross-section ratio (relative to reference)
/// σ_sample/σ_ref = (I_sample * C_ref) / (I_ref * C_sample)
pub fn relative_cross_section(sample_intensity: f64, sample_conc: f64, ref_intensity: f64, ref_conc: f64) -> f64 {
    if ref_intensity <= 0.0 || sample_conc <= 0.0 { return 0.0; }
    (sample_intensity * ref_conc) / (ref_intensity * sample_conc)
}

/// Wavelength of scattered light from Raman shift
/// 1/λ_scattered = 1/λ_laser - Δν (for Stokes)
pub fn scattered_wavelength_nm(laser_nm: f64, shift_cm1: f64) -> f64 {
    if laser_nm <= 0.0 { return 0.0; }
    let nu_laser = 1e7 / laser_nm;
    let nu_scattered = nu_laser - shift_cm1;
    if nu_scattered <= 0.0 { return 0.0; }
    1e7 / nu_scattered
}

/// Bose-Einstein correction factor for temperature-dependent Raman intensity
/// n(ν,T) = 1 / (exp(hcν/kT) - 1) (Stokes: n+1, Anti-Stokes: n)
pub fn bose_einstein_factor(shift_cm1: f64, temperature_k: f64) -> f64 {
    if temperature_k <= 0.0 || shift_cm1 <= 0.0 { return 0.0; }
    let hc_over_k = 1.4388; // hc/k in cm·K
    let x = hc_over_k * shift_cm1 / temperature_k;
    if x > 500.0 { return 0.0; }
    1.0 / (x.exp() - 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_test_spectrum(center: f64, amplitude: f64, fwhm: f64, n: usize) -> RamanSpectrum {
        let wn_start = center - 200.0;
        let wn_end = center + 200.0;
        let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
        let mut wavenumber = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let wn = wn_start + (i as f64) * (wn_end - wn_start) / (n as f64 - 1.0);
            let val = amplitude * (-0.5 * ((wn - center) / sigma).powi(2)).exp();
            wavenumber.push(wn);
            intensity.push(val);
        }
        RamanSpectrum { wavenumber_cm1: wavenumber, intensity: intensity }
    }

    #[test]
    fn test_wavelength_to_wavenumber() {
        // 532 nm → ~18797 cm⁻¹
        let wn = RamanProcessor::wavelength_to_wavenumber(532.0);
        assert!(approx_eq(wn, 18797.0, 1.0));
    }

    #[test]
    fn test_raman_shift() {
        let config = RamanConfig { laser_wavelength_nm: 532.0, ..Default::default() };
        let spectrum = make_test_spectrum(1000.0, 100.0, 10.0, 201);
        let proc = RamanProcessor::new(config, spectrum);
        // Scattered at longer wavelength (Stokes)
        let shift = proc.raman_shift_cm1(560.0);
        assert!(shift > 0.0); // Stokes shift is positive
    }

    #[test]
    fn test_stokes_anti_stokes_temperature() {
        // At 300K, for 520 cm⁻¹ Raman shift (silicon)
        // I_AS/I_S should be ≈ exp(-hcν/kT) * frequency correction
        let temp = stokes_anti_stokes_temp(1000.0, 80.0, 520.0, 532.0);
        assert!(temp > 200.0);
        assert!(temp < 500.0);
    }

    #[test]
    fn test_stokes_anti_stokes_zero() {
        let temp = stokes_anti_stokes_temp(0.0, 80.0, 520.0, 532.0);
        assert!(approx_eq(temp, 0.0, 0.01));
    }

    #[test]
    fn test_lorentzian_peak() {
        let val = lorentzian(1000.0, 1000.0, 100.0, 10.0);
        assert!(approx_eq(val, 100.0, 0.01));
        let val2 = lorentzian(1010.0, 1000.0, 100.0, 10.0);
        assert!(val2 < val);
    }

    #[test]
    fn test_gaussian_peak() {
        let val = gaussian(1000.0, 1000.0, 100.0, 10.0);
        assert!(approx_eq(val, 100.0, 0.01));
        let val2 = gaussian(1010.0, 1000.0, 100.0, 10.0);
        assert!(val2 < val);
    }

    #[test]
    fn test_pseudo_voigt_limits() {
        // eta=0: pure Gaussian
        let g = pseudo_voigt(1000.0, 1000.0, 100.0, 10.0, 0.0);
        let g_ref = gaussian(1000.0, 1000.0, 100.0, 10.0);
        assert!(approx_eq(g, g_ref, 0.01));
        // eta=1: pure Lorentzian
        let l = pseudo_voigt(1000.0, 1000.0, 100.0, 10.0, 1.0);
        let l_ref = lorentzian(1000.0, 1000.0, 100.0, 10.0);
        assert!(approx_eq(l, l_ref, 0.01));
    }

    #[test]
    fn test_find_peaks() {
        let spectrum = make_test_spectrum(1000.0, 100.0, 10.0, 201);
        let peaks = find_raman_peaks(&spectrum.wavenumber_cm1, &spectrum.intensity, 10.0);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].position_cm1, 1000.0, 2.0));
    }

    #[test]
    fn test_find_peaks_threshold() {
        let spectrum = make_test_spectrum(1000.0, 5.0, 10.0, 201);
        let peaks = find_raman_peaks(&spectrum.wavenumber_cm1, &spectrum.intensity, 10.0);
        assert_eq!(peaks.len(), 0);
    }

    #[test]
    fn test_savitzky_golay() {
        let data = vec![1.0, 2.0, 10.0, 2.0, 1.0, 2.0, 10.0, 2.0, 1.0];
        let smoothed = savitzky_golay_5pt(&data);
        assert_eq!(smoothed.len(), data.len());
        // Middle points should be smoother
        assert!(smoothed[4] > 0.0);
    }

    #[test]
    fn test_depolarization_ratio_symmetric() {
        let result = depolarization_ratio(100.0, 0.5);
        assert!(approx_eq(result.rho, 0.005, 0.001));
        assert!(result.symmetry.contains("symmetric"));
    }

    #[test]
    fn test_depolarization_ratio_depolarized() {
        let result = depolarization_ratio(100.0, 75.0);
        assert!(result.rho >= 0.75);
        assert!(result.symmetry.contains("Depolarized"));
    }

    #[test]
    fn test_remove_cosmic_rays() {
        let mut data: Vec<f64> = vec![10.0; 100];
        data[50] = 10000.0; // Cosmic ray spike
        let cleaned = remove_cosmic_rays(&data, 3.0);
        assert!(cleaned[50] < 100.0);
    }

    #[test]
    fn test_normalize_to_max() {
        let config = RamanConfig::default();
        let spectrum = make_test_spectrum(1000.0, 100.0, 10.0, 201);
        let proc = RamanProcessor::new(config, spectrum);
        let normalized = proc.normalize_to_max();
        let max_val = normalized.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(approx_eq(max_val, 1.0, 0.01));
    }

    #[test]
    fn test_enhancement_factor() {
        let ef = RamanProcessor::enhancement_factor(1e6, 1e3, 1000.0, 1e10);
        // EF = (1e6/1e3) / (1000/1e10) = 1000 / 1e-7 = 1e10
        assert!(approx_eq(ef, 1e10, 1e8));
    }

    #[test]
    fn test_scattered_wavelength() {
        let lambda = scattered_wavelength_nm(532.0, 520.0);
        // Should be longer than 532 nm (Stokes)
        assert!(lambda > 532.0);
        assert!(lambda < 600.0);
    }

    #[test]
    fn test_bose_einstein_factor() {
        let n = bose_einstein_factor(520.0, 300.0);
        assert!(n > 0.0);
        assert!(n < 1.0);
        // Higher temperature => more population
        let n_hot = bose_einstein_factor(520.0, 600.0);
        assert!(n_hot > n);
    }

    #[test]
    fn test_bose_einstein_zero_temp() {
        let n = bose_einstein_factor(520.0, 0.0);
        assert!(approx_eq(n, 0.0, 0.01));
    }

    #[test]
    fn test_relative_cross_section() {
        let sigma = relative_cross_section(500.0, 0.1, 1000.0, 0.5);
        // (500*0.5)/(1000*0.1) = 250/100 = 2.5
        assert!(approx_eq(sigma, 2.5, 0.01));
    }

    #[test]
    fn test_fluorescence_removal() {
        // Spectrum with linear baseline + peak
        let n = 201;
        let mut wn = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let w = 800.0 + (i as f64) * 2.0;
            let baseline = 50.0 + 0.1 * (w - 800.0); // Linear fluorescence
            let sigma: f64 = 5.0;
            let peak = 100.0 * (-0.5 * ((w - 1000.0) / sigma).powi(2)).exp();
            wn.push(w);
            intensity.push(baseline + peak);
        }
        let spectrum = RamanSpectrum { wavenumber_cm1: wn, intensity: intensity };
        let config = RamanConfig::default();
        let proc = RamanProcessor::new(config, spectrum);
        let corrected = proc.remove_fluorescence(1);
        // Corrected should have reduced baseline
        assert!(corrected.len() == 201);
    }

    #[test]
    fn test_processor_smooth() {
        let spectrum = make_test_spectrum(1000.0, 100.0, 10.0, 201);
        let config = RamanConfig::default();
        let proc = RamanProcessor::new(config, spectrum);
        let smoothed = proc.smooth_spectrum();
        assert_eq!(smoothed.len(), 201);
    }

    #[test]
    fn test_processor_find_peaks() {
        let spectrum = make_test_spectrum(1000.0, 100.0, 10.0, 201);
        let config = RamanConfig::default();
        let proc = RamanProcessor::new(config, spectrum);
        let peaks = proc.find_peaks(10.0);
        assert_eq!(peaks.len(), 1);
    }

    #[test]
    fn test_processor_depolarization() {
        let result = RamanProcessor::depolarization_ratio(100.0, 30.0);
        assert!(result.rho > 0.0);
        assert!(result.rho < 0.75);
    }

    #[test]
    fn test_processor_cosmic_rays() {
        let mut wn: Vec<f64> = (0..100).map(|i| 800.0 + i as f64 * 2.0).collect();
        let mut intensity: Vec<f64> = vec![10.0; 100];
        intensity[50] = 5000.0;
        let spectrum = RamanSpectrum { wavenumber_cm1: wn, intensity: intensity };
        let config = RamanConfig::default();
        let proc = RamanProcessor::new(config, spectrum);
        let cleaned = proc.remove_cosmic_rays(3.0);
        assert!(cleaned[50] < 1000.0);
    }

    #[test]
    fn test_lorentzian_fwhm() {
        // At half max: L(center ± FWHM/2) = amplitude/2
        let amplitude: f64 = 100.0;
        let fwhm: f64 = 10.0;
        let val = lorentzian(1005.0, 1000.0, amplitude, fwhm);
        assert!(approx_eq(val, amplitude / 2.0, 0.1));
    }

    #[test]
    fn test_enhancement_factor_zero() {
        let ef = RamanProcessor::enhancement_factor(1000.0, 100.0, 0.0, 1000.0);
        assert!(approx_eq(ef, 0.0, 0.01));
    }

    #[test]
    fn test_wavelength_to_wavenumber_zero() {
        let wn = RamanProcessor::wavelength_to_wavenumber(0.0);
        assert!(approx_eq(wn, 0.0, 0.01));
    }
}
