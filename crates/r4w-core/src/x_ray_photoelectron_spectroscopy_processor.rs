// X-ray Photoelectron Spectroscopy (XPS) Processor
// Surface chemical analysis: binding energy, chemical shifts, Shirley/Tougaard backgrounds,
// quantification via sensitivity factors, Auger parameter, peak fitting
//
// No external crate dependencies - all math from scratch using only std.

/// XPS spectral data point
#[derive(Debug, Clone)]
pub struct XpsSpectrum {
    /// Binding energies in eV
    pub binding_energy_ev: Vec<f64>,
    /// Counts per second (intensity)
    pub intensity_cps: Vec<f64>,
}

/// Peak fitting result
#[derive(Debug, Clone)]
pub struct XpsPeak {
    /// Peak binding energy (eV)
    pub binding_energy_ev: f64,
    /// Peak intensity (cps)
    pub intensity_cps: f64,
    /// Full width at half maximum (eV)
    pub fwhm_ev: f64,
    /// Peak area (cps*eV)
    pub area: f64,
    /// Chemical assignment
    pub assignment: String,
}

/// Elemental quantification result
#[derive(Debug, Clone)]
pub struct XpsQuantResult {
    /// Element name
    pub element: String,
    /// Atomic concentration (fraction 0-1)
    pub atomic_fraction: f64,
    /// Peak area used
    pub peak_area: f64,
    /// Sensitivity factor used
    pub sensitivity_factor: f64,
}

/// XPS sensitivity factor entry
#[derive(Debug, Clone)]
pub struct SensitivityFactor {
    pub element: String,
    pub orbital: String,
    pub binding_energy_ev: f64,
    pub rsf: f64, // Relative sensitivity factor (Scofield cross-section based)
}

/// Auger parameter result
#[derive(Debug, Clone)]
pub struct AugerParameter {
    pub element: String,
    pub photoelectron_be_ev: f64,
    pub auger_ke_ev: f64,
    pub modified_auger_param_ev: f64, // α' = KE_Auger + BE_photoelectron
}

/// XPS processor configuration
#[derive(Debug, Clone)]
pub struct XpsConfig {
    /// X-ray source energy in eV (Al Kα = 1486.6, Mg Kα = 1253.6)
    pub source_energy_ev: f64,
    /// Work function of spectrometer (eV)
    pub work_function_ev: f64,
    /// Energy step size (eV)
    pub step_size_ev: f64,
}

impl Default for XpsConfig {
    fn default() -> Self {
        Self {
            source_energy_ev: 1486.6, // Al Kα
            work_function_ev: 4.5,
            step_size_ev: 0.1,
        }
    }
}

/// Main XPS processor
pub struct XpsProcessor {
    pub config: XpsConfig,
    pub spectrum: XpsSpectrum,
    sensitivity_factors: Vec<SensitivityFactor>,
}

impl XpsProcessor {
    pub fn new(config: XpsConfig, spectrum: XpsSpectrum) -> Self {
        let sensitivity_factors = default_sensitivity_factors();
        Self { config, spectrum, sensitivity_factors }
    }

    /// Kinetic energy to binding energy: BE = hν - KE - φ
    pub fn ke_to_be(&self, kinetic_energy_ev: f64) -> f64 {
        self.config.source_energy_ev - kinetic_energy_ev - self.config.work_function_ev
    }

    /// Binding energy to kinetic energy: KE = hν - BE - φ
    pub fn be_to_ke(&self, binding_energy_ev: f64) -> f64 {
        self.config.source_energy_ev - binding_energy_ev - self.config.work_function_ev
    }

    /// Shirley background subtraction
    /// Iterative method: background at point i is proportional to integrated area above background
    /// from the high-BE side
    pub fn shirley_background(&self, max_iterations: usize) -> Vec<f64> {
        shirley_background(&self.spectrum.binding_energy_ev, &self.spectrum.intensity_cps, max_iterations)
    }

    /// Linear background between two endpoint regions
    pub fn linear_background(&self) -> Vec<f64> {
        linear_background(&self.spectrum.binding_energy_ev, &self.spectrum.intensity_cps)
    }

    /// Tougaard background: B(E) = B_coeff * integral from E to inf of I(E') * L(E'-E) dE'
    /// Using universal cross-section: L(T) = BT / (C + T^2)^2
    /// B = 2866 eV^2, C = 1643 eV^2 (Tougaard universal)
    pub fn tougaard_background(&self) -> Vec<f64> {
        tougaard_background(&self.spectrum.binding_energy_ev, &self.spectrum.intensity_cps)
    }

    /// Subtract background from spectrum
    pub fn background_subtracted(&self, background: &[f64]) -> Vec<f64> {
        self.spectrum.intensity_cps.iter()
            .zip(background.iter())
            .map(|(i, b)| (i - b).max(0.0))
            .collect()
    }

    /// Find peaks in spectrum using derivative zero-crossing
    pub fn find_peaks(&self, min_intensity: f64) -> Vec<XpsPeak> {
        find_peaks(&self.spectrum.binding_energy_ev, &self.spectrum.intensity_cps, min_intensity)
    }

    /// Calculate peak area by numerical integration (trapezoidal)
    pub fn peak_area(&self, be_start: f64, be_end: f64, background: &[f64]) -> f64 {
        peak_area_range(
            &self.spectrum.binding_energy_ev,
            &self.spectrum.intensity_cps,
            background,
            be_start,
            be_end,
        )
    }

    /// Quantify atomic concentrations from peak areas and sensitivity factors
    pub fn quantify(&self, peak_areas: &[(String, f64)]) -> Vec<XpsQuantResult> {
        quantify_atomic(peak_areas, &self.sensitivity_factors)
    }

    /// Calculate modified Auger parameter: α' = KE_Auger + BE_photoelectron
    pub fn auger_parameter(&self, element: &str, photoelectron_be_ev: f64, auger_ke_ev: f64) -> AugerParameter {
        AugerParameter {
            element: element.to_string(),
            photoelectron_be_ev,
            auger_ke_ev,
            modified_auger_param_ev: auger_ke_ev + photoelectron_be_ev,
        }
    }

    /// Lookup sensitivity factor for an element
    pub fn get_sensitivity_factor(&self, element: &str) -> Option<f64> {
        self.sensitivity_factors.iter()
            .find(|sf| sf.element == element)
            .map(|sf| sf.rsf)
    }

    /// Estimate inelastic mean free path (IMFP) using TPP-2M formula (simplified)
    /// λ(nm) ≈ E / (E_p^2 * [β * ln(γ * E) - C/E + D/E^2])
    /// Simplified: λ ≈ 0.41 * a^1.5 * sqrt(E) / Z^0.45
    /// where a = atomic size (nm), E = kinetic energy (eV), Z = atomic number
    pub fn imfp_estimate_nm(&self, kinetic_energy_ev: f64, atomic_number: u32) -> f64 {
        imfp_tpp2m_simplified(kinetic_energy_ev, atomic_number)
    }

    /// Sampling depth (95% of signal): d = 3λ cos(θ)
    pub fn sampling_depth_nm(&self, imfp_nm: f64, emission_angle_deg: f64) -> f64 {
        3.0 * imfp_nm * (emission_angle_deg * std::f64::consts::PI / 180.0).cos()
    }

    /// Angle-resolved XPS: intensity vs emission angle for overlayer model
    /// I = I_inf * [1 - exp(-d / (λ cos(θ)))]
    pub fn overlayer_intensity(&self, i_inf: f64, thickness_nm: f64, imfp_nm: f64, angle_deg: f64) -> f64 {
        let cos_theta = (angle_deg * std::f64::consts::PI / 180.0).cos();
        if cos_theta <= 0.0 { return 0.0; }
        i_inf * (1.0 - (-thickness_nm / (imfp_nm * cos_theta)).exp())
    }

    /// Estimate overlayer thickness from intensity ratio
    /// d = -λ cos(θ) * ln(1 - I/I_inf)
    pub fn estimate_overlayer_thickness_nm(&self, intensity_ratio: f64, imfp_nm: f64, angle_deg: f64) -> f64 {
        if intensity_ratio >= 1.0 || intensity_ratio <= 0.0 { return 0.0; }
        let cos_theta = (angle_deg * std::f64::consts::PI / 180.0).cos();
        -imfp_nm * cos_theta * (1.0 - intensity_ratio).ln()
    }
}

/// Shirley background calculation
pub fn shirley_background(be: &[f64], intensity: &[f64], max_iter: usize) -> Vec<f64> {
    let n = be.len().min(intensity.len());
    if n < 2 { return vec![0.0; n]; }

    // Determine high-BE and low-BE endpoints
    let i_left = intensity[0];
    let i_right = intensity[n - 1];

    let mut bg = vec![0.0; n];

    // Iterative Shirley: background[i] = I_right + (I_left - I_right) * S_right(i) / S_total
    // where S_right(i) = integral of (intensity - background) from i to n-1
    for _iter in 0..max_iter {
        // Compute cumulative sum from right
        let subtracted: Vec<f64> = (0..n).map(|i| (intensity[i] - bg[i]).max(0.0)).collect();
        let mut cum_right = vec![0.0; n];
        for i in (0..n - 1).rev() {
            let de = (be[i] - be[i + 1]).abs();
            cum_right[i] = cum_right[i + 1] + 0.5 * (subtracted[i] + subtracted[i + 1]) * de;
        }
        let total = cum_right[0];
        if total < 1e-30 { break; }

        for i in 0..n {
            bg[i] = i_right + (i_left - i_right) * cum_right[i] / total;
        }
    }
    bg
}

/// Linear background between first and last points
pub fn linear_background(be: &[f64], intensity: &[f64]) -> Vec<f64> {
    let n = be.len().min(intensity.len());
    if n < 2 { return vec![0.0; n]; }
    let i_start = intensity[0];
    let i_end = intensity[n - 1];
    let be_start = be[0];
    let be_end = be[n - 1];
    let range = be_end - be_start;
    if range.abs() < 1e-30 { return vec![i_start; n]; }
    (0..n).map(|i| {
        let frac = (be[i] - be_start) / range;
        i_start + frac * (i_end - i_start)
    }).collect()
}

/// Tougaard universal background
pub fn tougaard_background(be: &[f64], intensity: &[f64]) -> Vec<f64> {
    let n = be.len().min(intensity.len());
    if n < 2 { return vec![0.0; n]; }

    let b_coeff: f64 = 2866.0; // eV^2
    let c_coeff: f64 = 1643.0; // eV^2

    let mut bg = vec![0.0; n];
    for i in 0..n {
        let mut integral = 0.0;
        for j in 0..i {
            // T = energy loss = BE[j] - BE[i] (for increasing BE array)
            // or use absolute difference
            let t = (be[j] - be[i]).abs();
            if t < 0.01 { continue; }
            let loss_fn = b_coeff * t / (c_coeff + t * t).powi(2);
            let de = if j > 0 { (be[j] - be[j - 1]).abs() } else { (be[1] - be[0]).abs() };
            integral += intensity[j] * loss_fn * de;
        }
        bg[i] = integral;
    }
    bg
}

/// Gaussian peak shape for XPS fitting
pub fn gaussian_peak(be: f64, center: f64, amplitude: f64, fwhm: f64) -> f64 {
    let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
    amplitude * (-0.5 * ((be - center) / sigma).powi(2)).exp()
}

/// Lorentzian peak shape for XPS fitting
pub fn lorentzian_peak(be: f64, center: f64, amplitude: f64, fwhm: f64) -> f64 {
    let gamma = fwhm / 2.0;
    amplitude * gamma * gamma / ((be - center).powi(2) + gamma * gamma)
}

/// Voigt profile approximation (pseudo-Voigt: linear combination)
pub fn pseudo_voigt_peak(be: f64, center: f64, amplitude: f64, fwhm: f64, eta: f64) -> f64 {
    let g = gaussian_peak(be, center, amplitude, fwhm);
    let l = lorentzian_peak(be, center, amplitude, fwhm);
    eta * l + (1.0 - eta) * g
}

/// Find peaks in XPS spectrum
pub fn find_peaks(be: &[f64], intensity: &[f64], min_intensity: f64) -> Vec<XpsPeak> {
    let n = be.len().min(intensity.len());
    if n < 3 { return Vec::new(); }

    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        if intensity[i] > intensity[i - 1] && intensity[i] > intensity[i + 1]
            && intensity[i] >= min_intensity
        {
            let fwhm = estimate_fwhm_at(be, intensity, i);
            let area = estimate_peak_area_local(be, intensity, i, fwhm);
            peaks.push(XpsPeak {
                binding_energy_ev: be[i],
                intensity_cps: intensity[i],
                fwhm_ev: fwhm,
                area,
                assignment: String::new(),
            });
        }
    }
    peaks
}

/// Estimate FWHM at a peak index
fn estimate_fwhm_at(be: &[f64], intensity: &[f64], peak_idx: usize) -> f64 {
    let n = be.len().min(intensity.len());
    let half_max = intensity[peak_idx] / 2.0;

    // Search left for half-max crossing
    let mut left_be = be[peak_idx];
    for i in (0..peak_idx).rev() {
        if intensity[i] <= half_max {
            // Linear interpolation
            let frac = (half_max - intensity[i]) / (intensity[i + 1] - intensity[i]).max(1e-30);
            left_be = be[i] + frac * (be[i + 1] - be[i]);
            break;
        }
    }

    // Search right for half-max crossing
    let mut right_be = be[peak_idx];
    for i in peak_idx + 1..n {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i - 1] - intensity[i]).max(1e-30);
            right_be = be[i] + frac * (be[i - 1] - be[i]);
            break;
        }
    }

    (right_be - left_be).abs()
}

/// Estimate peak area from local region around peak
fn estimate_peak_area_local(be: &[f64], intensity: &[f64], peak_idx: usize, fwhm: f64) -> f64 {
    let n = be.len().min(intensity.len());
    let center = be[peak_idx];
    let half_width = fwhm * 2.0; // Integrate over ±2*FWHM

    let mut area = 0.0;
    for i in 1..n {
        if (be[i] - center).abs() <= half_width && (be[i - 1] - center).abs() <= half_width {
            let de = (be[i] - be[i - 1]).abs();
            area += 0.5 * (intensity[i] + intensity[i - 1]) * de;
        }
    }
    area
}

/// Calculate peak area in a BE range with background subtraction
pub fn peak_area_range(be: &[f64], intensity: &[f64], background: &[f64], be_start: f64, be_end: f64) -> f64 {
    let n = be.len().min(intensity.len()).min(background.len());
    let lo = be_start.min(be_end);
    let hi = be_start.max(be_end);

    let mut area = 0.0;
    for i in 1..n {
        if be[i] >= lo && be[i] <= hi && be[i - 1] >= lo && be[i - 1] <= hi {
            let de = (be[i] - be[i - 1]).abs();
            let net_i = ((intensity[i] - background[i]).max(0.0) + (intensity[i - 1] - background[i - 1]).max(0.0)) / 2.0;
            area += net_i * de;
        }
    }
    area
}

/// Atomic quantification from peak areas and sensitivity factors
pub fn quantify_atomic(peak_areas: &[(String, f64)], sf_table: &[SensitivityFactor]) -> Vec<XpsQuantResult> {
    if peak_areas.is_empty() { return Vec::new(); }

    let mut normalized: Vec<(String, f64, f64)> = Vec::new();
    let mut total = 0.0;

    for (element, area) in peak_areas {
        let rsf = sf_table.iter()
            .find(|sf| sf.element == *element)
            .map(|sf| sf.rsf)
            .unwrap_or(1.0);
        let norm = area / rsf;
        normalized.push((element.clone(), *area, rsf));
        total += norm;
    }

    if total < 1e-30 { return Vec::new(); }

    normalized.iter().map(|(el, area, rsf)| {
        XpsQuantResult {
            element: el.clone(),
            atomic_fraction: (area / rsf) / total,
            peak_area: *area,
            sensitivity_factor: *rsf,
        }
    }).collect()
}

/// Chemical shift: difference in BE from elemental reference
pub fn chemical_shift_ev(measured_be: f64, reference_be: f64) -> f64 {
    measured_be - reference_be
}

/// Spin-orbit splitting: doublet separation
/// For p orbitals: j = l ± 1/2 → p1/2 and p3/2 with ratio 1:2
/// For d orbitals: d3/2 and d5/2 with ratio 2:3
/// For f orbitals: f5/2 and f7/2 with ratio 3:4
pub fn spin_orbit_ratio(orbital: &str) -> f64 {
    match orbital {
        "p" => 0.5,   // p1/2:p3/2 = 1:2
        "d" => 2.0 / 3.0, // d3/2:d5/2 = 2:3
        "f" => 3.0 / 4.0, // f5/2:f7/2 = 3:4
        _ => 1.0,
    }
}

/// Simplified TPP-2M IMFP estimate
/// λ ≈ 0.1 * sqrt(E) for kinetic energies in eV, result in nm
pub fn imfp_tpp2m_simplified(kinetic_energy_ev: f64, _atomic_number: u32) -> f64 {
    if kinetic_energy_ev <= 0.0 { return 0.0; }
    // Simplified universal curve: λ(nm) ≈ 0.054 * sqrt(E) for most materials
    0.054 * kinetic_energy_ev.sqrt()
}

/// Wagner plot coordinates for chemical state identification
/// x = modified Auger parameter α', y = binding energy
pub fn wagner_plot_point(be_ev: f64, auger_ke_ev: f64) -> (f64, f64) {
    let alpha_prime = auger_ke_ev + be_ev;
    (alpha_prime, be_ev)
}

/// Charge correction: shift all BEs to align C 1s to 284.8 eV (adventitious carbon)
pub fn charge_correct(be: &[f64], c1s_measured: f64) -> Vec<f64> {
    let shift = 284.8 - c1s_measured;
    be.iter().map(|e| e + shift).collect()
}

/// Scofield cross-section based sensitivity factors (commonly used)
fn default_sensitivity_factors() -> Vec<SensitivityFactor> {
    vec![
        SensitivityFactor { element: "C".into(), orbital: "1s".into(), binding_energy_ev: 284.8, rsf: 1.0 },
        SensitivityFactor { element: "O".into(), orbital: "1s".into(), binding_energy_ev: 532.0, rsf: 2.93 },
        SensitivityFactor { element: "N".into(), orbital: "1s".into(), binding_energy_ev: 399.0, rsf: 1.80 },
        SensitivityFactor { element: "Si".into(), orbital: "2p".into(), binding_energy_ev: 99.0, rsf: 0.87 },
        SensitivityFactor { element: "Fe".into(), orbital: "2p".into(), binding_energy_ev: 710.0, rsf: 16.42 },
        SensitivityFactor { element: "Ti".into(), orbital: "2p".into(), binding_energy_ev: 459.0, rsf: 7.91 },
        SensitivityFactor { element: "Al".into(), orbital: "2p".into(), binding_energy_ev: 74.0, rsf: 0.54 },
        SensitivityFactor { element: "Cu".into(), orbital: "2p".into(), binding_energy_ev: 933.0, rsf: 22.05 },
        SensitivityFactor { element: "Zn".into(), orbital: "2p".into(), binding_energy_ev: 1022.0, rsf: 23.0 },
        SensitivityFactor { element: "Au".into(), orbital: "4f".into(), binding_energy_ev: 84.0, rsf: 17.12 },
        SensitivityFactor { element: "Ag".into(), orbital: "3d".into(), binding_energy_ev: 368.0, rsf: 18.04 },
        SensitivityFactor { element: "F".into(), orbital: "1s".into(), binding_energy_ev: 685.0, rsf: 4.43 },
        SensitivityFactor { element: "S".into(), orbital: "2p".into(), binding_energy_ev: 164.0, rsf: 1.68 },
        SensitivityFactor { element: "P".into(), orbital: "2p".into(), binding_energy_ev: 133.0, rsf: 1.19 },
        SensitivityFactor { element: "Na".into(), orbital: "1s".into(), binding_energy_ev: 1072.0, rsf: 8.52 },
    ]
}

/// Asymmetric Doniach-Sunjic line shape for metallic peaks
/// DS(x) = cos[πα/2 + (1-α)*atan(x/γ)] / (1 + (x/γ)^2)^((1-α)/2)
pub fn doniach_sunjic(be: f64, center: f64, amplitude: f64, gamma: f64, alpha: f64) -> f64 {
    let x = be - center;
    let ratio = x / gamma;
    let base = (1.0 + ratio * ratio).powf((1.0 - alpha) / 2.0);
    if base < 1e-30 { return 0.0; }
    let angle = std::f64::consts::PI * alpha / 2.0 + (1.0 - alpha) * ratio.atan();
    amplitude * angle.cos() / base
}

/// Calculate relative surface composition from overlayer model
/// Intensity ratio: I_overlayer / I_substrate = (λ_ov / λ_sub) * (1 - exp(-d/λ_ov)) / exp(-d/λ_sub)
pub fn overlayer_intensity_ratio(thickness_nm: f64, imfp_overlayer_nm: f64, imfp_substrate_nm: f64) -> f64 {
    if imfp_substrate_nm <= 0.0 || imfp_overlayer_nm <= 0.0 { return 0.0; }
    let numerator = imfp_overlayer_nm * (1.0 - (-thickness_nm / imfp_overlayer_nm).exp());
    let denominator = imfp_substrate_nm * (-thickness_nm / imfp_substrate_nm).exp();
    if denominator < 1e-30 { return f64::INFINITY; }
    numerator / denominator
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_test_spectrum(center: f64, amplitude: f64, fwhm: f64, n: usize) -> XpsSpectrum {
        let be_start = center - 10.0;
        let be_end = center + 10.0;
        let mut be = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
        for i in 0..n {
            let e = be_start + (i as f64) * (be_end - be_start) / (n as f64 - 1.0);
            let val = amplitude * (-0.5 * ((e - center) / sigma).powi(2)).exp() + 100.0; // baseline
            be.push(e);
            intensity.push(val);
        }
        XpsSpectrum { binding_energy_ev: be, intensity_cps: intensity }
    }

    #[test]
    fn test_ke_to_be() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 201);
        let proc = XpsProcessor::new(config, spectrum);
        // BE = 1486.6 - KE - 4.5
        let be = proc.ke_to_be(1000.0);
        assert!(approx_eq(be, 482.1, 0.1));
    }

    #[test]
    fn test_be_to_ke() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 201);
        let proc = XpsProcessor::new(config, spectrum);
        let ke = proc.be_to_ke(285.0);
        assert!(approx_eq(ke, 1197.1, 0.1));
    }

    #[test]
    fn test_ke_be_roundtrip() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 201);
        let proc = XpsProcessor::new(config, spectrum);
        let be: f64 = 285.0;
        let ke = proc.be_to_ke(be);
        let be2 = proc.ke_to_be(ke);
        assert!(approx_eq(be, be2, 1e-10));
    }

    #[test]
    fn test_linear_background() {
        let be = vec![280.0, 282.0, 284.0, 286.0, 288.0];
        let intensity = vec![100.0, 200.0, 500.0, 200.0, 100.0];
        let bg = linear_background(&be, &intensity);
        assert_eq!(bg.len(), 5);
        assert!(approx_eq(bg[0], 100.0, 0.01));
        assert!(approx_eq(bg[4], 100.0, 0.01));
        // Middle should be average
        assert!(approx_eq(bg[2], 100.0, 0.01));
    }

    #[test]
    fn test_shirley_background_flat() {
        // Flat spectrum => Shirley background = constant
        let be: Vec<f64> = (0..100).map(|i| 280.0 + 0.1 * i as f64).collect();
        let intensity: Vec<f64> = vec![100.0; 100];
        let bg = shirley_background(&be, &intensity, 10);
        assert_eq!(bg.len(), 100);
        // All should be approximately 100
        for b in &bg {
            assert!(approx_eq(*b, 100.0, 1.0));
        }
    }

    #[test]
    fn test_shirley_background_with_peak() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 201);
        let bg = shirley_background(&spectrum.binding_energy_ev, &spectrum.intensity_cps, 20);
        assert_eq!(bg.len(), 201);
        // Background should be less than peak intensity
        let peak_idx = 100; // center
        assert!(bg[peak_idx] < spectrum.intensity_cps[peak_idx]);
    }

    #[test]
    fn test_tougaard_background() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let bg = tougaard_background(&spectrum.binding_energy_ev, &spectrum.intensity_cps);
        assert_eq!(bg.len(), 101);
    }

    #[test]
    fn test_gaussian_peak() {
        let val = gaussian_peak(285.0, 285.0, 1000.0, 1.5);
        assert!(approx_eq(val, 1000.0, 1.0));
        // Off-center should be less
        let val2 = gaussian_peak(286.0, 285.0, 1000.0, 1.5);
        assert!(val2 < val);
    }

    #[test]
    fn test_lorentzian_peak() {
        let val = lorentzian_peak(285.0, 285.0, 1000.0, 1.5);
        assert!(approx_eq(val, 1000.0, 1.0));
        let val2 = lorentzian_peak(286.0, 285.0, 1000.0, 1.5);
        assert!(val2 < val);
    }

    #[test]
    fn test_pseudo_voigt() {
        // Pure Gaussian (eta=0)
        let g = pseudo_voigt_peak(285.0, 285.0, 1000.0, 1.5, 0.0);
        let g_ref = gaussian_peak(285.0, 285.0, 1000.0, 1.5);
        assert!(approx_eq(g, g_ref, 0.01));
        // Pure Lorentzian (eta=1)
        let l = pseudo_voigt_peak(285.0, 285.0, 1000.0, 1.5, 1.0);
        let l_ref = lorentzian_peak(285.0, 285.0, 1000.0, 1.5);
        assert!(approx_eq(l, l_ref, 0.01));
    }

    #[test]
    fn test_find_peaks_single() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 201);
        let peaks = find_peaks(&spectrum.binding_energy_ev, &spectrum.intensity_cps, 200.0);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].binding_energy_ev, 285.0, 0.2));
    }

    #[test]
    fn test_find_peaks_threshold() {
        let spectrum = make_test_spectrum(285.0, 100.0, 1.5, 201);
        // baseline=100, peak=200, threshold=250 => no peaks
        let peaks = find_peaks(&spectrum.binding_energy_ev, &spectrum.intensity_cps, 250.0);
        assert_eq!(peaks.len(), 0);
    }

    #[test]
    fn test_quantify_single_element() {
        let sf = default_sensitivity_factors();
        let areas = vec![("C".to_string(), 1000.0)];
        let results = quantify_atomic(&areas, &sf);
        assert_eq!(results.len(), 1);
        assert!(approx_eq(results[0].atomic_fraction, 1.0, 0.01));
    }

    #[test]
    fn test_quantify_two_elements() {
        let sf = default_sensitivity_factors();
        // C (RSF=1.0) area=1000, O (RSF=2.93) area=2930
        // Normalized: C=1000/1=1000, O=2930/2.93=1000 => 50:50
        let areas = vec![("C".to_string(), 1000.0), ("O".to_string(), 2930.0)];
        let results = quantify_atomic(&areas, &sf);
        assert_eq!(results.len(), 2);
        assert!(approx_eq(results[0].atomic_fraction, 0.5, 0.01));
        assert!(approx_eq(results[1].atomic_fraction, 0.5, 0.01));
    }

    #[test]
    fn test_quantify_fractions_sum_to_one() {
        let sf = default_sensitivity_factors();
        let areas = vec![
            ("C".to_string(), 500.0),
            ("O".to_string(), 1500.0),
            ("N".to_string(), 900.0),
        ];
        let results = quantify_atomic(&areas, &sf);
        let total: f64 = results.iter().map(|r| r.atomic_fraction).sum();
        assert!(approx_eq(total, 1.0, 1e-10));
    }

    #[test]
    fn test_chemical_shift() {
        let shift = chemical_shift_ev(286.5, 284.8);
        assert!(approx_eq(shift, 1.7, 0.01));
    }

    #[test]
    fn test_spin_orbit_ratio() {
        assert!(approx_eq(spin_orbit_ratio("p"), 0.5, 0.01));
        assert!(approx_eq(spin_orbit_ratio("d"), 2.0 / 3.0, 0.01));
        assert!(approx_eq(spin_orbit_ratio("f"), 0.75, 0.01));
    }

    #[test]
    fn test_auger_parameter() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        let ap = proc.auger_parameter("Cu", 932.7, 918.6);
        assert!(approx_eq(ap.modified_auger_param_ev, 1851.3, 0.1));
    }

    #[test]
    fn test_imfp_estimate() {
        let imfp = imfp_tpp2m_simplified(1000.0, 29); // Cu at 1000 eV
        assert!(imfp > 0.5); // Should be ~1-2 nm
        assert!(imfp < 5.0);
    }

    #[test]
    fn test_imfp_increases_with_energy() {
        let imfp_low = imfp_tpp2m_simplified(100.0, 29);
        let imfp_high = imfp_tpp2m_simplified(1000.0, 29);
        assert!(imfp_high > imfp_low);
    }

    #[test]
    fn test_sampling_depth() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        let depth = proc.sampling_depth_nm(2.0, 0.0); // Normal emission
        assert!(approx_eq(depth, 6.0, 0.01)); // 3λ
        // At 60 degrees, cos(60)=0.5 => depth = 3.0
        let depth60 = proc.sampling_depth_nm(2.0, 60.0);
        assert!(approx_eq(depth60, 3.0, 0.1));
    }

    #[test]
    fn test_overlayer_intensity() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        // Thick overlayer => approaches I_inf
        let i_thick = proc.overlayer_intensity(1000.0, 100.0, 2.0, 0.0);
        assert!(approx_eq(i_thick, 1000.0, 1.0));
        // Zero thickness => zero intensity
        let i_zero = proc.overlayer_intensity(1000.0, 0.0, 2.0, 0.0);
        assert!(approx_eq(i_zero, 0.0, 0.01));
    }

    #[test]
    fn test_estimate_overlayer_thickness() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        let d = proc.estimate_overlayer_thickness_nm(0.5, 2.0, 0.0);
        // I/I_inf = 0.5 => d = -λ*ln(0.5) = λ*0.693
        assert!(approx_eq(d, 2.0 * 0.693, 0.01));
    }

    #[test]
    fn test_charge_correct() {
        let be = vec![280.0, 285.0, 290.0];
        let corrected = charge_correct(&be, 285.5); // measured C1s at 285.5 instead of 284.8
        assert!(approx_eq(corrected[1], 284.3, 0.01)); // shifted by -0.7 eV
    }

    #[test]
    fn test_doniach_sunjic_at_center() {
        let val = doniach_sunjic(285.0, 285.0, 1000.0, 0.5, 0.0);
        // At zero asymmetry, at center, should be amplitude
        assert!(approx_eq(val, 1000.0, 1.0));
    }

    #[test]
    fn test_doniach_sunjic_asymmetry() {
        // With asymmetry, the peak becomes skewed
        let val_left = doniach_sunjic(284.0, 285.0, 1000.0, 0.5, 0.1);
        let val_right = doniach_sunjic(286.0, 285.0, 1000.0, 0.5, 0.1);
        // Asymmetry creates a tail on the high-BE side
        // Both should be positive, magnitudes differ
        assert!(val_left.abs() > 0.0);
        assert!(val_right.abs() > 0.0);
    }

    #[test]
    fn test_wagner_plot_point() {
        let (alpha_prime, be) = wagner_plot_point(932.7, 918.6);
        assert!(approx_eq(alpha_prime, 1851.3, 0.1));
        assert!(approx_eq(be, 932.7, 0.1));
    }

    #[test]
    fn test_overlayer_intensity_ratio() {
        // Zero thickness => zero ratio
        let r0 = overlayer_intensity_ratio(0.0, 2.0, 2.0);
        assert!(approx_eq(r0, 0.0, 0.01));
        // Thick overlayer => large ratio
        let r_thick = overlayer_intensity_ratio(100.0, 2.0, 2.0);
        assert!(r_thick > 10.0);
    }

    #[test]
    fn test_sensitivity_factor_lookup() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        let sf = proc.get_sensitivity_factor("C");
        assert!(sf.is_some());
        assert!(approx_eq(sf.unwrap(), 1.0, 0.01));
        let sf_o = proc.get_sensitivity_factor("O");
        assert!(sf_o.is_some());
        assert!(approx_eq(sf_o.unwrap(), 2.93, 0.01));
    }

    #[test]
    fn test_sensitivity_factor_unknown() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        assert!(proc.get_sensitivity_factor("Xx").is_none());
    }

    #[test]
    fn test_processor_find_peaks() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 201);
        let config = XpsConfig::default();
        let proc = XpsProcessor::new(config, spectrum);
        let peaks = proc.find_peaks(200.0);
        assert_eq!(peaks.len(), 1);
    }

    #[test]
    fn test_background_subtracted() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let config = XpsConfig::default();
        let proc = XpsProcessor::new(config, spectrum);
        let bg = proc.linear_background();
        let sub = proc.background_subtracted(&bg);
        assert_eq!(sub.len(), 101);
        // All values should be non-negative
        for v in &sub {
            assert!(*v >= 0.0);
        }
    }

    #[test]
    fn test_peak_area_range() {
        let be: Vec<f64> = (0..101).map(|i| 280.0 + 0.1 * i as f64).collect();
        let sigma: f64 = 0.5;
        let intensity: Vec<f64> = be.iter().map(|e| 1000.0 * (-0.5 * ((e - 285.0) / sigma).powi(2)).exp()).collect();
        let bg = vec![0.0; 101];
        let area = peak_area_range(&be, &intensity, &bg, 283.0, 287.0);
        // Gaussian area ≈ amplitude * sigma * sqrt(2π) ≈ 1000 * 0.5 * 2.507 ≈ 1253
        assert!(area > 1000.0);
        assert!(area < 1500.0);
    }

    #[test]
    fn test_quantify_empty() {
        let sf = default_sensitivity_factors();
        let results = quantify_atomic(&[], &sf);
        assert!(results.is_empty());
    }

    #[test]
    fn test_imfp_zero_energy() {
        let imfp = imfp_tpp2m_simplified(0.0, 14);
        assert!(approx_eq(imfp, 0.0, 0.01));
    }

    #[test]
    fn test_processor_shirley() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let config = XpsConfig::default();
        let proc = XpsProcessor::new(config, spectrum);
        let bg = proc.shirley_background(10);
        assert_eq!(bg.len(), 101);
    }

    #[test]
    fn test_processor_tougaard() {
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let config = XpsConfig::default();
        let proc = XpsProcessor::new(config, spectrum);
        let bg = proc.tougaard_background();
        assert_eq!(bg.len(), 101);
    }

    #[test]
    fn test_overlayer_at_grazing() {
        let config = XpsConfig::default();
        let spectrum = make_test_spectrum(285.0, 1000.0, 1.5, 101);
        let proc = XpsProcessor::new(config, spectrum);
        // At grazing angle (85°), intensity approaches I_inf (signal from overlayer only)
        let i = proc.overlayer_intensity(1000.0, 5.0, 2.0, 85.0);
        assert!(i > 900.0); // Nearly saturated for thick overlayer at grazing
    }
}
