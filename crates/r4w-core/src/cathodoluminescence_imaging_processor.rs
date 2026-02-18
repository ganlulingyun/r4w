// Cathodoluminescence (CL) Imaging Processor
// Luminescence spectroscopy in SEM: spectral analysis, band gap determination,
// defect identification, Varshni temperature dependence, depth profiling, dead layer
//
// No external crate dependencies - all math from scratch using only std.

/// CL spectrum data
#[derive(Debug, Clone)]
pub struct ClSpectrum {
    /// Wavelength in nm
    pub wavelength_nm: Vec<f64>,
    /// CL intensity (counts)
    pub intensity: Vec<f64>,
}

/// CL peak analysis result
#[derive(Debug, Clone)]
pub struct ClPeak {
    /// Peak wavelength (nm)
    pub wavelength_nm: f64,
    /// Peak energy (eV)
    pub energy_ev: f64,
    /// Peak intensity
    pub intensity: f64,
    /// FWHM in nm
    pub fwhm_nm: f64,
    /// FWHM in eV
    pub fwhm_ev: f64,
    /// Peak area
    pub area: f64,
    /// Assignment (e.g., "Near-band-edge", "Deep-level defect")
    pub assignment: String,
}

/// Band gap determination result
#[derive(Debug, Clone)]
pub struct BandGapResult {
    /// Band gap energy (eV)
    pub band_gap_ev: f64,
    /// Method used
    pub method: String,
    /// Band gap wavelength (nm)
    pub band_gap_nm: f64,
}

/// Varshni temperature dependence
/// E_g(T) = E_g(0) - αT² / (T + β)
#[derive(Debug, Clone)]
pub struct VarshniParams {
    /// Zero-temperature band gap (eV)
    pub eg0_ev: f64,
    /// α parameter (eV/K)
    pub alpha_ev_per_k: f64,
    /// β parameter (K)
    pub beta_k: f64,
}

/// Monte Carlo electron penetration result
#[derive(Debug, Clone)]
pub struct PenetrationDepthResult {
    /// Maximum penetration depth (nm)
    pub depth_nm: f64,
    /// Lateral spread (nm)
    pub lateral_nm: f64,
    /// Generation volume (nm³)
    pub volume_nm3: f64,
}

/// CL processor configuration
#[derive(Debug, Clone)]
pub struct ClConfig {
    /// Electron beam energy (keV)
    pub beam_energy_kev: f64,
    /// Beam current (nA)
    pub beam_current_na: f64,
    /// Sample temperature (K)
    pub temperature_k: f64,
}

impl Default for ClConfig {
    fn default() -> Self {
        Self {
            beam_energy_kev: 10.0,
            beam_current_na: 1.0,
            temperature_k: 300.0,
        }
    }
}

/// Main CL processor
pub struct ClProcessor {
    pub config: ClConfig,
    pub spectrum: ClSpectrum,
}

impl ClProcessor {
    pub fn new(config: ClConfig, spectrum: ClSpectrum) -> Self {
        Self { config, spectrum }
    }

    /// Convert wavelength (nm) to energy (eV): E = hc/λ = 1239.84/λ
    pub fn wavelength_to_ev(wavelength_nm: f64) -> f64 {
        if wavelength_nm <= 0.0 { return 0.0; }
        1239.84 / wavelength_nm
    }

    /// Convert energy (eV) to wavelength (nm)
    pub fn ev_to_wavelength(energy_ev: f64) -> f64 {
        if energy_ev <= 0.0 { return 0.0; }
        1239.84 / energy_ev
    }

    /// Convert spectrum to energy scale (eV)
    pub fn to_energy_scale(&self) -> (Vec<f64>, Vec<f64>) {
        let n = self.spectrum.wavelength_nm.len().min(self.spectrum.intensity.len());
        let mut energy_ev = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let e = Self::wavelength_to_ev(self.spectrum.wavelength_nm[i]);
            // Jacobian: I(E) = I(λ) * |dλ/dE| = I(λ) * hc/E²
            let jacobian = 1239.84 / (e * e);
            energy_ev.push(e);
            intensity.push(self.spectrum.intensity[i] * jacobian);
        }
        // Reverse to get increasing energy
        energy_ev.reverse();
        intensity.reverse();
        (energy_ev, intensity)
    }

    /// Find CL emission peaks
    pub fn find_peaks(&self, min_intensity: f64) -> Vec<ClPeak> {
        find_cl_peaks(&self.spectrum.wavelength_nm, &self.spectrum.intensity, min_intensity)
    }

    /// Determine band gap from CL onset
    /// Uses tangent line method on the high-energy (short wavelength) edge
    pub fn determine_band_gap(&self) -> BandGapResult {
        let (energy, intensity) = self.to_energy_scale();
        let eg = band_gap_tangent(&energy, &intensity);
        BandGapResult {
            band_gap_ev: eg,
            method: "Tangent line".to_string(),
            band_gap_nm: Self::ev_to_wavelength(eg),
        }
    }

    /// Varshni equation: E_g(T) = E_g(0) - αT²/(T+β)
    pub fn varshni_band_gap(params: &VarshniParams, temperature_k: f64) -> f64 {
        params.eg0_ev - params.alpha_ev_per_k * temperature_k * temperature_k / (temperature_k + params.beta_k)
    }

    /// Electron penetration depth (Kanaya-Okayama model)
    /// R_KO (nm) = 27.6 * A * E^1.67 / (Z^0.89 * ρ)
    /// where A=atomic mass, E=keV, Z=atomic number, ρ=g/cm³
    pub fn penetration_depth_nm(beam_energy_kev: f64, atomic_mass: f64, atomic_number: u32, density_g_cm3: f64) -> PenetrationDepthResult {
        kanaya_okayama(beam_energy_kev, atomic_mass, atomic_number, density_g_cm3)
    }

    /// Dead layer thickness estimation
    /// Below dead layer, electrons don't generate useful CL
    /// Estimated from CL intensity vs beam energy
    pub fn estimate_dead_layer(&self, energies_kev: &[f64], intensities: &[f64]) -> f64 {
        estimate_dead_layer(energies_kev, intensities)
    }

    /// CL quantum efficiency
    /// η_CL = (photons emitted) / (electron-hole pairs generated)
    pub fn quantum_efficiency(photon_count: f64, eh_pairs: f64) -> f64 {
        if eh_pairs <= 0.0 { return 0.0; }
        photon_count / eh_pairs
    }

    /// Number of electron-hole pairs generated per incident electron
    /// n_eh ≈ E_beam / (3 * E_g) (simplified)
    pub fn eh_pairs_per_electron(beam_energy_kev: f64, band_gap_ev: f64) -> f64 {
        if band_gap_ev <= 0.0 { return 0.0; }
        beam_energy_kev * 1000.0 / (3.0 * band_gap_ev)
    }

    /// Normalize spectrum
    pub fn normalize_to_max(&self) -> Vec<f64> {
        let max_val = self.spectrum.intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if max_val.abs() < 1e-30 { return self.spectrum.intensity.clone(); }
        self.spectrum.intensity.iter().map(|i| i / max_val).collect()
    }

    /// Calculate integrated CL intensity
    pub fn integrated_intensity(&self) -> f64 {
        let n = self.spectrum.wavelength_nm.len().min(self.spectrum.intensity.len());
        if n < 2 { return 0.0; }
        let mut total = 0.0;
        for i in 1..n {
            let dl = (self.spectrum.wavelength_nm[i] - self.spectrum.wavelength_nm[i - 1]).abs();
            total += 0.5 * (self.spectrum.intensity[i] + self.spectrum.intensity[i - 1]) * dl;
        }
        total
    }
}

/// Find CL peaks
pub fn find_cl_peaks(wavelength: &[f64], intensity: &[f64], min_intensity: f64) -> Vec<ClPeak> {
    let n = wavelength.len().min(intensity.len());
    if n < 3 { return Vec::new(); }

    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        if intensity[i] > intensity[i - 1] && intensity[i] > intensity[i + 1]
            && intensity[i] >= min_intensity
        {
            let fwhm_nm = estimate_fwhm_cl(wavelength, intensity, i);
            let energy = ClProcessor::wavelength_to_ev(wavelength[i]);
            // Convert FWHM from nm to eV: ΔE ≈ (hc/λ²) * Δλ
            let fwhm_ev = 1239.84 / (wavelength[i] * wavelength[i]) * fwhm_nm;

            let area = estimate_cl_area(wavelength, intensity, i, fwhm_nm);

            peaks.push(ClPeak {
                wavelength_nm: wavelength[i],
                energy_ev: energy,
                intensity: intensity[i],
                fwhm_nm,
                fwhm_ev,
                area,
                assignment: classify_cl_emission(energy),
            });
        }
    }
    peaks
}

/// Estimate FWHM at CL peak
fn estimate_fwhm_cl(wl: &[f64], intensity: &[f64], peak_idx: usize) -> f64 {
    let n = wl.len().min(intensity.len());
    let half_max = intensity[peak_idx] / 2.0;

    let mut left = wl[peak_idx];
    for i in (0..peak_idx).rev() {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i + 1] - intensity[i]).max(1e-30);
            left = wl[i] + frac * (wl[i + 1] - wl[i]);
            break;
        }
    }

    let mut right = wl[peak_idx];
    for i in peak_idx + 1..n {
        if intensity[i] <= half_max {
            let frac = (half_max - intensity[i]) / (intensity[i - 1] - intensity[i]).max(1e-30);
            right = wl[i] + frac * (wl[i - 1] - wl[i]);
            break;
        }
    }

    (right - left).abs()
}

/// Estimate peak area
fn estimate_cl_area(wl: &[f64], intensity: &[f64], peak_idx: usize, fwhm: f64) -> f64 {
    let n = wl.len().min(intensity.len());
    let center = wl[peak_idx];
    let hw = fwhm * 2.0;

    let mut area = 0.0;
    for i in 1..n {
        if (wl[i] - center).abs() <= hw && (wl[i - 1] - center).abs() <= hw {
            let dl = (wl[i] - wl[i - 1]).abs();
            area += 0.5 * (intensity[i] + intensity[i - 1]) * dl;
        }
    }
    area
}

/// Classify CL emission based on energy
fn classify_cl_emission(energy_ev: f64) -> String {
    // Generic classification
    if energy_ev > 3.0 {
        "UV emission (possible NBE)".to_string()
    } else if energy_ev > 2.0 {
        "Visible emission (defect-related)".to_string()
    } else if energy_ev > 1.0 {
        "Near-IR emission (deep level)".to_string()
    } else {
        "IR emission".to_string()
    }
}

/// Band gap from tangent line method on energy-scale CL spectrum
pub fn band_gap_tangent(energy_ev: &[f64], intensity: &[f64]) -> f64 {
    let n = energy_ev.len().min(intensity.len());
    if n < 5 { return 0.0; }

    // Find the peak
    let mut peak_idx = 0;
    let mut peak_val = f64::NEG_INFINITY;
    for i in 0..n {
        if intensity[i] > peak_val {
            peak_val = intensity[i];
            peak_idx = i;
        }
    }

    // Find the steepest point on the high-energy side
    let mut max_slope = 0.0;
    let mut slope_idx = peak_idx;
    for i in peak_idx + 1..n - 1 {
        let slope = (intensity[i] - intensity[i + 1]) / (energy_ev[i + 1] - energy_ev[i]).max(1e-30);
        if slope > max_slope {
            max_slope = slope;
            slope_idx = i;
        }
    }

    if max_slope < 1e-30 { return 0.0; }

    // Tangent line: y = slope * (x - x0) + y0
    // Intercept with y=0: x = x0 - y0/slope
    let x0 = energy_ev[slope_idx];
    let y0 = intensity[slope_idx];
    x0 - y0 / max_slope
}

/// Kanaya-Okayama electron penetration depth
pub fn kanaya_okayama(energy_kev: f64, atomic_mass: f64, atomic_number: u32, density: f64) -> PenetrationDepthResult {
    if energy_kev <= 0.0 || density <= 0.0 {
        return PenetrationDepthResult { depth_nm: 0.0, lateral_nm: 0.0, volume_nm3: 0.0 };
    }
    let z = atomic_number as f64;
    let depth = 27.6 * atomic_mass * energy_kev.powf(1.67) / (z.powf(0.89) * density);
    let lateral = depth * 0.5; // Lateral spread ≈ 0.5 * depth (approximate)
    let volume = std::f64::consts::PI / 6.0 * depth * lateral * lateral; // Ellipsoidal approximation
    PenetrationDepthResult { depth_nm: depth, lateral_nm: lateral, volume_nm3: volume }
}

/// Estimate dead layer thickness from CL intensity vs beam energy
/// CL onset occurs when beam energy exceeds threshold for penetrating dead layer
pub fn estimate_dead_layer(energies_kev: &[f64], intensities: &[f64]) -> f64 {
    let n = energies_kev.len().min(intensities.len());
    if n < 3 { return 0.0; }

    // Find the energy where CL intensity first rises significantly
    let max_intensity = intensities.iter().cloned().fold(0.0_f64, f64::max);
    if max_intensity < 1e-30 { return 0.0; }
    let threshold = max_intensity * 0.1;

    for i in 0..n {
        if intensities[i] >= threshold {
            // Interpolate the onset energy
            let onset_kev = if i > 0 {
                let frac = (threshold - intensities[i - 1]) / (intensities[i] - intensities[i - 1]).max(1e-30);
                energies_kev[i - 1] + frac * (energies_kev[i] - energies_kev[i - 1])
            } else {
                energies_kev[i]
            };
            // Rough conversion: dead layer ≈ R(E_onset) using simplified model
            // R ≈ 40 * E^1.75 / ρ (nm) for silicon-like material
            return 40.0 * onset_kev.powf(1.75) / 2.33; // Assuming Si density
        }
    }
    0.0
}

/// Varshni equation for common semiconductors
pub fn varshni_params_gaas() -> VarshniParams {
    VarshniParams { eg0_ev: 1.519, alpha_ev_per_k: 5.405e-4, beta_k: 204.0 }
}

pub fn varshni_params_gan() -> VarshniParams {
    VarshniParams { eg0_ev: 3.510, alpha_ev_per_k: 9.09e-4, beta_k: 830.0 }
}

pub fn varshni_params_si() -> VarshniParams {
    VarshniParams { eg0_ev: 1.166, alpha_ev_per_k: 4.73e-4, beta_k: 636.0 }
}

pub fn varshni_params_zno() -> VarshniParams {
    VarshniParams { eg0_ev: 3.437, alpha_ev_per_k: 7.2e-4, beta_k: 1077.0 }
}

/// Radiative lifetime estimate from CL decay
/// τ = 1 / (A * n) where A = Einstein coefficient, n = carrier density
pub fn radiative_lifetime_ns(einstein_a_per_s: f64) -> f64 {
    if einstein_a_per_s <= 0.0 { return 0.0; }
    1e9 / einstein_a_per_s
}

/// CL intensity temperature dependence (Arrhenius quenching)
/// I(T) = I_0 / (1 + C * exp(-E_a/(k*T)))
pub fn thermal_quenching(i0: f64, activation_ev: f64, temperature_k: f64, c_factor: f64) -> f64 {
    if temperature_k <= 0.0 { return i0; }
    let kb = 8.617333e-5; // eV/K
    let exponent = -activation_ev / (kb * temperature_k);
    i0 / (1.0 + c_factor * exponent.exp())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_test_cl_spectrum(peak_nm: f64, amplitude: f64, fwhm_nm: f64, n: usize) -> ClSpectrum {
        let wl_start = peak_nm - 100.0;
        let wl_end = peak_nm + 100.0;
        let sigma = fwhm_nm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
        let mut wavelength = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let wl = wl_start + (i as f64) * (wl_end - wl_start) / (n as f64 - 1.0);
            let val = amplitude * (-0.5 * ((wl - peak_nm) / sigma).powi(2)).exp();
            wavelength.push(wl);
            intensity.push(val);
        }
        ClSpectrum { wavelength_nm: wavelength, intensity }
    }

    #[test]
    fn test_wavelength_to_ev() {
        // 1240 nm ≈ 1 eV
        let e = ClProcessor::wavelength_to_ev(1239.84);
        assert!(approx_eq(e, 1.0, 0.001));
        // 620 nm ≈ 2 eV
        let e2 = ClProcessor::wavelength_to_ev(619.92);
        assert!(approx_eq(e2, 2.0, 0.001));
    }

    #[test]
    fn test_ev_to_wavelength() {
        let wl = ClProcessor::ev_to_wavelength(1.0);
        assert!(approx_eq(wl, 1239.84, 0.01));
    }

    #[test]
    fn test_ev_wavelength_roundtrip() {
        let e: f64 = 2.5;
        let wl = ClProcessor::ev_to_wavelength(e);
        let e2 = ClProcessor::wavelength_to_ev(wl);
        assert!(approx_eq(e, e2, 1e-10));
    }

    #[test]
    fn test_find_cl_peaks() {
        let spectrum = make_test_cl_spectrum(500.0, 1000.0, 20.0, 201);
        let peaks = find_cl_peaks(&spectrum.wavelength_nm, &spectrum.intensity, 100.0);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].wavelength_nm, 500.0, 2.0));
    }

    #[test]
    fn test_cl_peak_energy() {
        let spectrum = make_test_cl_spectrum(500.0, 1000.0, 20.0, 201);
        let peaks = find_cl_peaks(&spectrum.wavelength_nm, &spectrum.intensity, 100.0);
        assert!(peaks[0].energy_ev > 2.0);
        assert!(peaks[0].energy_ev < 3.0);
    }

    #[test]
    fn test_varshni_gaas() {
        let params = varshni_params_gaas();
        // At 0K: should be E_g(0)
        let eg0 = ClProcessor::varshni_band_gap(&params, 0.0);
        assert!(approx_eq(eg0, 1.519, 0.001));
        // At 300K: GaAs Eg ≈ 1.42 eV
        let eg300 = ClProcessor::varshni_band_gap(&params, 300.0);
        assert!(eg300 < eg0);
        assert!(approx_eq(eg300, 1.42, 0.02));
    }

    #[test]
    fn test_varshni_si() {
        let params = varshni_params_si();
        let eg0 = ClProcessor::varshni_band_gap(&params, 0.0);
        assert!(approx_eq(eg0, 1.166, 0.001));
        let eg300 = ClProcessor::varshni_band_gap(&params, 300.0);
        assert!(approx_eq(eg300, 1.12, 0.02));
    }

    #[test]
    fn test_varshni_decreases_with_temp() {
        let params = varshni_params_gan();
        let eg100 = ClProcessor::varshni_band_gap(&params, 100.0);
        let eg300 = ClProcessor::varshni_band_gap(&params, 300.0);
        let eg500 = ClProcessor::varshni_band_gap(&params, 500.0);
        assert!(eg100 > eg300);
        assert!(eg300 > eg500);
    }

    #[test]
    fn test_kanaya_okayama_si() {
        // Silicon: A=28.09, Z=14, ρ=2.33 g/cm³, E=10 keV
        let result = kanaya_okayama(10.0, 28.09, 14, 2.33);
        assert!(result.depth_nm > 500.0);
        assert!(result.depth_nm < 5000.0);
        assert!(result.lateral_nm > 0.0);
        assert!(result.volume_nm3 > 0.0);
    }

    #[test]
    fn test_penetration_depth_increases() {
        let d5 = kanaya_okayama(5.0, 28.09, 14, 2.33);
        let d10 = kanaya_okayama(10.0, 28.09, 14, 2.33);
        let d20 = kanaya_okayama(20.0, 28.09, 14, 2.33);
        assert!(d5.depth_nm < d10.depth_nm);
        assert!(d10.depth_nm < d20.depth_nm);
    }

    #[test]
    fn test_penetration_depth_zero_energy() {
        let result = kanaya_okayama(0.0, 28.09, 14, 2.33);
        assert!(approx_eq(result.depth_nm, 0.0, 0.01));
    }

    #[test]
    fn test_eh_pairs() {
        // 10 keV in GaAs (Eg=1.42 eV): n ≈ 10000/(3*1.42) ≈ 2347
        let n = ClProcessor::eh_pairs_per_electron(10.0, 1.42);
        assert!(approx_eq(n, 2347.0, 10.0));
    }

    #[test]
    fn test_quantum_efficiency() {
        let eta = ClProcessor::quantum_efficiency(500.0, 2000.0);
        assert!(approx_eq(eta, 0.25, 0.01));
    }

    #[test]
    fn test_quantum_efficiency_zero() {
        let eta = ClProcessor::quantum_efficiency(500.0, 0.0);
        assert!(approx_eq(eta, 0.0, 0.01));
    }

    #[test]
    fn test_thermal_quenching() {
        let i0: f64 = 1000.0;
        // At very low temperature, intensity should be near I0
        let i_low = thermal_quenching(i0, 0.1, 10.0, 100.0);
        assert!(approx_eq(i_low, i0, 1.0));
        // At high temperature, intensity should decrease
        let i_high = thermal_quenching(i0, 0.1, 500.0, 100.0);
        assert!(i_high < i0);
    }

    #[test]
    fn test_thermal_quenching_zero_activation() {
        let i = thermal_quenching(1000.0, 0.0, 300.0, 100.0);
        // C*exp(0) = C, so I = I0/(1+C)
        assert!(approx_eq(i, 1000.0 / 101.0, 0.1));
    }

    #[test]
    fn test_radiative_lifetime() {
        // A = 1e8 /s => τ = 10 ns
        let tau = radiative_lifetime_ns(1e8);
        assert!(approx_eq(tau, 10.0, 0.01));
    }

    #[test]
    fn test_radiative_lifetime_zero() {
        let tau = radiative_lifetime_ns(0.0);
        assert!(approx_eq(tau, 0.0, 0.01));
    }

    #[test]
    fn test_classify_uv() {
        let cls = classify_cl_emission(3.5);
        assert!(cls.contains("UV"));
    }

    #[test]
    fn test_classify_visible() {
        let cls = classify_cl_emission(2.5);
        assert!(cls.contains("Visible"));
    }

    #[test]
    fn test_classify_ir() {
        let cls = classify_cl_emission(0.8);
        assert!(cls.contains("IR"));
    }

    #[test]
    fn test_processor_find_peaks() {
        let spectrum = make_test_cl_spectrum(500.0, 1000.0, 20.0, 201);
        let config = ClConfig::default();
        let proc = ClProcessor::new(config, spectrum);
        let peaks = proc.find_peaks(100.0);
        assert_eq!(peaks.len(), 1);
    }

    #[test]
    fn test_to_energy_scale() {
        let spectrum = make_test_cl_spectrum(500.0, 1000.0, 20.0, 201);
        let config = ClConfig::default();
        let proc = ClProcessor::new(config, spectrum);
        let (energy, intensity) = proc.to_energy_scale();
        assert_eq!(energy.len(), 201);
        assert!(energy[0] < energy[energy.len() - 1]); // Increasing energy
    }

    #[test]
    fn test_normalize_to_max() {
        let spectrum = make_test_cl_spectrum(500.0, 1000.0, 20.0, 201);
        let config = ClConfig::default();
        let proc = ClProcessor::new(config, spectrum);
        let norm = proc.normalize_to_max();
        let max_val = norm.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(approx_eq(max_val, 1.0, 0.01));
    }

    #[test]
    fn test_integrated_intensity() {
        let spectrum = make_test_cl_spectrum(500.0, 1000.0, 20.0, 201);
        let config = ClConfig::default();
        let proc = ClProcessor::new(config, spectrum);
        let total = proc.integrated_intensity();
        assert!(total > 0.0);
    }

    #[test]
    fn test_dead_layer_estimate() {
        // CL intensity rises after some threshold beam energy
        let energies: Vec<f64> = vec![1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0];
        let intensities: Vec<f64> = vec![0.0, 0.0, 10.0, 100.0, 500.0, 800.0, 1000.0, 1000.0];
        let dead_layer = estimate_dead_layer(&energies, &intensities);
        assert!(dead_layer > 0.0);
    }

    #[test]
    fn test_band_gap_determination() {
        // Create a spectrum with NBE at 365 nm (GaN, 3.4 eV)
        let spectrum = make_test_cl_spectrum(365.0, 1000.0, 10.0, 201);
        let config = ClConfig::default();
        let proc = ClProcessor::new(config, spectrum);
        let bg = proc.determine_band_gap();
        // Band gap should be near 3.4 eV
        assert!(bg.band_gap_ev > 2.0);
    }

    #[test]
    fn test_varshni_zno() {
        let params = varshni_params_zno();
        let eg0 = ClProcessor::varshni_band_gap(&params, 0.0);
        assert!(approx_eq(eg0, 3.437, 0.001));
    }

    #[test]
    fn test_eh_pairs_zero_gap() {
        let n = ClProcessor::eh_pairs_per_electron(10.0, 0.0);
        assert!(approx_eq(n, 0.0, 0.01));
    }
}
