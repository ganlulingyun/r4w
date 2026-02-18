// sims_secondary_ion_mass_analyzer.rs
//
// Secondary Ion Mass Spectrometry (SIMS) signal processing.
// Depth profiling, mass spectrum analysis, RSF quantification, isotope ratios,
// matrix effects correction, MRI mixing model, detection limits.

/// Elementary charge (C).
pub const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

// ---------------------------------------------------------------------------
// 1. Depth Profile
// ---------------------------------------------------------------------------

/// A single depth profile data point.
#[derive(Debug, Clone, Copy)]
pub struct DepthProfilePoint {
    pub time_s: f64,
    pub intensity_cps: f64,
}

/// Depth profile time series for one species.
#[derive(Debug, Clone)]
pub struct DepthProfile {
    pub species: String,
    pub points: Vec<DepthProfilePoint>,
}

impl DepthProfile {
    /// Create a new empty depth profile for the given species label.
    pub fn new(species: &str) -> Self {
        Self { species: species.to_string(), points: Vec::new() }
    }
    /// Add a data point.
    pub fn add_point(&mut self, time_s: f64, intensity_cps: f64) {
        self.points.push(DepthProfilePoint { time_s, intensity_cps });
    }
    /// Number of data points.
    pub fn len(&self) -> usize { self.points.len() }
    /// Whether the profile is empty.
    pub fn is_empty(&self) -> bool { self.points.is_empty() }
    /// Time values.
    pub fn times(&self) -> Vec<f64> { self.points.iter().map(|p| p.time_s).collect() }
    /// Intensity values.
    pub fn intensities(&self) -> Vec<f64> { self.points.iter().map(|p| p.intensity_cps).collect() }
    /// Maximum intensity.
    pub fn max_intensity(&self) -> f64 {
        self.points.iter().map(|p| p.intensity_cps).fold(0.0_f64, f64::max)
    }
    /// Integrated counts (trapezoidal rule over time).
    pub fn integrated_counts(&self) -> f64 {
        if self.points.len() < 2 { return 0.0; }
        let mut sum = 0.0;
        for i in 1..self.points.len() {
            let dt = self.points[i].time_s - self.points[i - 1].time_s;
            sum += 0.5 * (self.points[i].intensity_cps + self.points[i - 1].intensity_cps) * dt;
        }
        sum
    }
}

// ---------------------------------------------------------------------------
// 2. Sputter Yield and Erosion Rate
// ---------------------------------------------------------------------------

/// Sputter yield result.
#[derive(Debug, Clone, Copy)]
pub struct SputterYieldResult {
    pub yield_atoms_per_ion: f64,
    pub erosion_rate_nm_s: f64,
    pub crater_depth_nm: f64,
}

/// Calculate sputter yield from measured crater depth.
pub fn sputter_yield(
    crater_depth_nm: f64, sputter_time_s: f64, primary_current_na: f64,
    raster_area_um2: f64, target_density_atoms_cm3: f64,
) -> SputterYieldResult {
    let erosion_rate = crater_depth_nm / sputter_time_s;
    let atoms_removed = (crater_depth_nm * 1e-7) * (raster_area_um2 * 1e-8) * target_density_atoms_cm3;
    let ions = (primary_current_na * 1e-9) * sputter_time_s / ELEMENTARY_CHARGE;
    let y = if ions > 0.0 { atoms_removed / ions } else { 0.0 };
    SputterYieldResult { yield_atoms_per_ion: y, erosion_rate_nm_s: erosion_rate, crater_depth_nm }
}

/// Erosion rate from sputter yield (returns nm/s).
pub fn erosion_rate_from_yield(
    yield_val: f64, primary_current_na: f64, raster_area_um2: f64,
    target_density_atoms_cm3: f64,
) -> f64 {
    let area_cm2 = raster_area_um2 * 1e-8;
    if area_cm2.abs() < 1e-30 || target_density_atoms_cm3.abs() < 1e-10 { return 0.0; }
    let ion_flux = (primary_current_na * 1e-9) / ELEMENTARY_CHARGE;
    yield_val * ion_flux / (area_cm2 * target_density_atoms_cm3) * 1e7
}

// ---------------------------------------------------------------------------
// 3. Mass Spectrum
// ---------------------------------------------------------------------------

/// A peak in a SIMS mass spectrum.
#[derive(Debug, Clone)]
pub struct MassPeak {
    pub mz: f64,
    pub intensity: f64,
    pub species: Option<String>,
}

/// A SIMS mass spectrum.
#[derive(Debug, Clone)]
pub struct MassSpectrum {
    pub peaks: Vec<MassPeak>,
}

impl MassSpectrum {
    /// Create a new empty mass spectrum.
    pub fn new() -> Self { Self { peaks: Vec::new() } }
    /// Add a peak.
    pub fn add_peak(&mut self, mz: f64, intensity: f64, species: Option<&str>) {
        self.peaks.push(MassPeak { mz, intensity, species: species.map(|s| s.to_string()) });
    }
    /// Number of peaks.
    pub fn len(&self) -> usize { self.peaks.len() }
    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool { self.peaks.is_empty() }
    /// Find peaks within a tolerance of a given m/z.
    pub fn find_peaks(&self, target_mz: f64, tol: f64) -> Vec<&MassPeak> {
        self.peaks.iter().filter(|p| (p.mz - target_mz).abs() < tol).collect()
    }
    /// Base peak (highest intensity).
    pub fn base_peak(&self) -> Option<&MassPeak> {
        self.peaks.iter().max_by(|a, b| a.intensity.partial_cmp(&b.intensity).unwrap_or(std::cmp::Ordering::Equal))
    }
    /// Total ion count.
    pub fn total_ion_count(&self) -> f64 { self.peaks.iter().map(|p| p.intensity).sum() }
    /// Mass resolution R = m / delta_m.
    pub fn mass_resolution(&self, peak_mz: f64, fwhm: f64) -> f64 {
        if fwhm.abs() < 1e-30 { 0.0 } else { peak_mz / fwhm }
    }
}

// ---------------------------------------------------------------------------
// 4. Relative Sensitivity Factors (RSF)
// ---------------------------------------------------------------------------

/// Convert secondary ion intensity to concentration: C = RSF * (I_species / I_matrix).
pub fn quantify_rsf(intensity_species: f64, intensity_matrix: f64, rsf: f64) -> f64 {
    if intensity_matrix.abs() < 1e-30 { return 0.0; }
    rsf * intensity_species / intensity_matrix
}

/// Build concentration depth profile from raw intensity profiles.
pub fn concentration_depth_profile(
    species: &DepthProfile, matrix: &DepthProfile, rsf: f64, erosion_rate_nm_s: f64,
) -> Vec<(f64, f64)> {
    let n = species.len().min(matrix.len());
    (0..n).map(|i| {
        let depth = species.points[i].time_s * erosion_rate_nm_s;
        let conc = quantify_rsf(species.points[i].intensity_cps, matrix.points[i].intensity_cps, rsf);
        (depth, conc)
    }).collect()
}

// ---------------------------------------------------------------------------
// 5. Matrix Effects Correction
// ---------------------------------------------------------------------------

/// Matrix effect correction: scales raw intensity by reference/current matrix ratio.
pub fn matrix_effect_correction(raw: f64, matrix_sig: f64, matrix_ref: f64) -> f64 {
    if matrix_sig.abs() < 1e-30 { return raw; }
    raw * matrix_ref / matrix_sig
}

/// Apply matrix correction to an entire depth profile.
pub fn correct_matrix_effects(
    profile: &DepthProfile, matrix_profile: &DepthProfile, matrix_ref: f64,
) -> DepthProfile {
    let n = profile.len().min(matrix_profile.len());
    let mut corrected = DepthProfile::new(&profile.species);
    for i in 0..n {
        let c = matrix_effect_correction(
            profile.points[i].intensity_cps, matrix_profile.points[i].intensity_cps, matrix_ref,
        );
        corrected.add_point(profile.points[i].time_s, c);
    }
    corrected
}

// ---------------------------------------------------------------------------
// 6. Isotope Ratio Measurement and Natural Abundance
// ---------------------------------------------------------------------------

/// Isotope record.
#[derive(Debug, Clone, Copy)]
pub struct Isotope {
    pub mass_number: u32,
    pub atomic_mass: f64,
    pub abundance: f64,
}

/// Look up natural abundances for common SIMS elements.
pub fn natural_abundances(element: &str) -> &'static [Isotope] {
    static SI: [Isotope; 3] = [
        Isotope { mass_number: 28, atomic_mass: 27.977, abundance: 0.9223 },
        Isotope { mass_number: 29, atomic_mass: 28.976, abundance: 0.0467 },
        Isotope { mass_number: 30, atomic_mass: 29.974, abundance: 0.0310 },
    ];
    static B: [Isotope; 2] = [
        Isotope { mass_number: 10, atomic_mass: 10.013, abundance: 0.199 },
        Isotope { mass_number: 11, atomic_mass: 11.009, abundance: 0.801 },
    ];
    static O: [Isotope; 3] = [
        Isotope { mass_number: 16, atomic_mass: 15.995, abundance: 0.99757 },
        Isotope { mass_number: 17, atomic_mass: 16.999, abundance: 0.00038 },
        Isotope { mass_number: 18, atomic_mass: 17.999, abundance: 0.00205 },
    ];
    static C: [Isotope; 2] = [
        Isotope { mass_number: 12, atomic_mass: 12.000, abundance: 0.9893 },
        Isotope { mass_number: 13, atomic_mass: 13.003, abundance: 0.0107 },
    ];
    static GE: [Isotope; 5] = [
        Isotope { mass_number: 70, atomic_mass: 69.924, abundance: 0.2057 },
        Isotope { mass_number: 72, atomic_mass: 71.922, abundance: 0.2745 },
        Isotope { mass_number: 73, atomic_mass: 72.923, abundance: 0.0775 },
        Isotope { mass_number: 74, atomic_mass: 73.921, abundance: 0.3650 },
        Isotope { mass_number: 76, atomic_mass: 75.921, abundance: 0.0773 },
    ];
    static AS: [Isotope; 1] = [Isotope { mass_number: 75, atomic_mass: 74.922, abundance: 1.0 }];
    static P: [Isotope; 1] = [Isotope { mass_number: 31, atomic_mass: 30.974, abundance: 1.0 }];
    match element {
        "Si" => &SI, "B" => &B, "O" => &O, "C" => &C,
        "Ge" => &GE, "As" => &AS, "P" => &P, _ => &[],
    }
}

/// Measured isotope ratio from two peak intensities.
pub fn isotope_ratio(a: f64, b: f64) -> f64 {
    if b.abs() < 1e-30 { 0.0 } else { a / b }
}

/// Expected natural isotope ratio for two mass numbers of an element.
pub fn expected_isotope_ratio(element: &str, mass_a: u32, mass_b: u32) -> f64 {
    let iso = natural_abundances(element);
    let aa = iso.iter().find(|i| i.mass_number == mass_a).map_or(0.0, |i| i.abundance);
    let ab = iso.iter().find(|i| i.mass_number == mass_b).map_or(0.0, |i| i.abundance);
    if ab.abs() < 1e-30 { 0.0 } else { aa / ab }
}

/// Isotope ratio deviation from natural (per-mil).
pub fn isotope_ratio_deviation_permil(measured: f64, natural: f64) -> f64 {
    if natural.abs() < 1e-30 { 0.0 } else { (measured / natural - 1.0) * 1000.0 }
}

// ---------------------------------------------------------------------------
// 7. Depth Scale Calibration
// ---------------------------------------------------------------------------

/// Convert sputter-time profile to depth profile. Returns (depth_nm, intensity).
pub fn calibrate_depth_scale(profile: &DepthProfile, rate_nm_s: f64) -> Vec<(f64, f64)> {
    profile.points.iter().map(|p| (p.time_s * rate_nm_s, p.intensity_cps)).collect()
}

/// Erosion rate from crater depth measurement (nm/s).
pub fn erosion_rate_from_crater(depth_nm: f64, time_s: f64) -> f64 {
    if time_s.abs() < 1e-30 { 0.0 } else { depth_nm / time_s }
}

/// Multi-layer depth calibration with per-layer erosion rates.
pub fn multilayer_depth_calibration(
    layers: &[(f64, f64)], profile: &DepthProfile,
) -> Vec<(f64, f64)> {
    let mut bounds: Vec<(f64, f64, f64)> = Vec::new(); // (t_start, t_end, rate)
    let mut t_acc = 0.0;
    for &(thickness, rate) in layers {
        if rate.abs() < 1e-30 { continue; }
        let dt = thickness / rate;
        bounds.push((t_acc, t_acc + dt, rate));
        t_acc += dt;
    }
    let fallback = layers.last().map_or(1.0, |l| l.1);
    profile.points.iter().map(|p| {
        let mut depth = 0.0;
        let mut rem = p.time_s;
        for &(_ts, te, rate) in &bounds {
            let lt = te - _ts;
            if rem <= 0.0 { break; }
            if rem >= lt { depth += lt * rate; rem -= lt; }
            else { depth += rem * rate; rem = 0.0; }
        }
        if rem > 0.0 { depth += rem * fallback; }
        (depth, p.intensity_cps)
    }).collect()
}

// ---------------------------------------------------------------------------
// 8. MRI Mixing-Roughness-Information Depth Model
// ---------------------------------------------------------------------------

/// MRI model parameters for profile broadening.
#[derive(Debug, Clone, Copy)]
pub struct MriParams {
    pub mixing_length_nm: f64,
    pub roughness_nm: f64,
    pub info_depth_nm: f64,
}

impl MriParams {
    /// Create new MRI parameters.
    pub fn new(mixing: f64, roughness: f64, info_depth: f64) -> Self {
        Self { mixing_length_nm: mixing, roughness_nm: roughness, info_depth_nm: info_depth }
    }
    /// Combined depth resolution (quadrature sum).
    pub fn combined_resolution_nm(&self) -> f64 {
        (self.mixing_length_nm.powi(2) + self.roughness_nm.powi(2) + self.info_depth_nm.powi(2)).sqrt()
    }
}

/// MRI step response: 1 (film, z<0) to 0 (substrate, z>0) broadened by roughness + mixing.
pub fn mri_response(z: f64, params: &MriParams) -> f64 {
    let sqrt2 = std::f64::consts::SQRT_2;
    let sigma = (params.roughness_nm.powi(2) + params.info_depth_nm.powi(2)).sqrt().max(0.01);
    let w = params.mixing_length_nm.max(0.01);
    let gauss = 0.5 * erfc(z / (sigma * sqrt2));
    if z <= 0.0 { gauss } else { gauss.max(0.5 * (-z / w).exp()) }
}

/// Broaden a step interface at a given depth.
pub fn broaden_interface(depths: &[f64], iface: f64, params: &MriParams) -> Vec<f64> {
    depths.iter().map(|&d| mri_response(d - iface, params)).collect()
}

/// Broaden a delta-doped layer, returning concentration (atoms/cm^3).
pub fn broaden_delta_layer(
    depths: &[f64], delta_depth: f64, dose: f64, params: &MriParams,
) -> Vec<f64> {
    let dz = if depths.len() > 1 {
        (depths[depths.len() - 1] - depths[0]) / (depths.len() - 1) as f64
    } else { 1.0 };
    let step = broaden_interface(depths, delta_depth, params);
    (0..step.len()).map(|i| {
        let d = if i == 0 { (step[1] - step[0]) / dz }
        else if i == step.len() - 1 { (step[i] - step[i - 1]) / dz }
        else { (step[i + 1] - step[i - 1]) / (2.0 * dz) };
        dose * d.abs() * 1e7
    }).collect()
}

// ---------------------------------------------------------------------------
// 9. Detection Limit
// ---------------------------------------------------------------------------

/// Detection limit result.
#[derive(Debug, Clone, Copy)]
pub struct DetectionLimit {
    pub concentration_atoms_cm3: f64,
    pub background_cps: f64,
    pub rsf_atoms_cm3: f64,
}

/// DL = RSF * 3*sqrt(bg*t) / (matrix*t).
pub fn detection_limit(bg_cps: f64, matrix_cps: f64, rsf: f64, t_s: f64) -> DetectionLimit {
    if matrix_cps.abs() < 1e-30 || t_s.abs() < 1e-30 {
        return DetectionLimit { concentration_atoms_cm3: 0.0, background_cps: bg_cps, rsf_atoms_cm3: rsf };
    }
    let dl = rsf * 3.0 * (bg_cps * t_s).sqrt() / (matrix_cps * t_s);
    DetectionLimit { concentration_atoms_cm3: dl, background_cps: bg_cps, rsf_atoms_cm3: rsf }
}

/// Detection limit from profile (background = last fraction of data).
pub fn detection_limit_from_profile(
    sp: &DepthProfile, mp: &DepthProfile, rsf: f64, t_s: f64, bg_frac: f64,
) -> DetectionLimit {
    let n = sp.len().min(mp.len());
    if n < 3 { return DetectionLimit { concentration_atoms_cm3: 0.0, background_cps: 0.0, rsf_atoms_cm3: rsf }; }
    let start = ((n as f64 * (1.0 - bg_frac)) as usize).max(1);
    let cnt = (n - start) as f64;
    let bg: f64 = sp.points[start..n].iter().map(|p| p.intensity_cps).sum::<f64>() / cnt;
    let mx: f64 = mp.points[start..n].iter().map(|p| p.intensity_cps).sum::<f64>() / cnt;
    detection_limit(bg, mx, rsf, t_s)
}

// ---------------------------------------------------------------------------
// 10. SIMS Processor
// ---------------------------------------------------------------------------

/// Main SIMS signal processor orchestrating quantification workflows.
pub struct SimsProcessor {
    pub erosion_rate_nm_s: f64,
    pub primary_current_na: f64,
    pub raster_area_um2: f64,
    pub mri_params: Option<MriParams>,
}

impl SimsProcessor {
    /// Create a new SIMS processor.
    pub fn new(erosion: f64, current: f64, area: f64) -> Self {
        Self { erosion_rate_nm_s: erosion, primary_current_na: current, raster_area_um2: area, mri_params: None }
    }
    /// Set MRI broadening model.
    pub fn set_mri_params(&mut self, mix: f64, rough: f64, info: f64) {
        self.mri_params = Some(MriParams::new(mix, rough, info));
    }
    /// Quantify a depth profile.
    pub fn quantify_profile(&self, sp: &DepthProfile, mp: &DepthProfile, rsf: f64) -> Vec<(f64, f64)> {
        concentration_depth_profile(sp, mp, rsf, self.erosion_rate_nm_s)
    }
    /// Estimate detection limit.
    pub fn estimate_detection_limit(&self, sp: &DepthProfile, mp: &DepthProfile, rsf: f64, t: f64) -> DetectionLimit {
        detection_limit_from_profile(sp, mp, rsf, t, 0.2)
    }
    /// Calculate sputter yield.
    pub fn calculate_sputter_yield(&self, depth: f64, time: f64, density: f64) -> SputterYieldResult {
        sputter_yield(depth, time, self.primary_current_na, self.raster_area_um2, density)
    }
    /// Apply matrix correction.
    pub fn correct_matrix(&self, p: &DepthProfile, mp: &DepthProfile, mref: f64) -> DepthProfile {
        correct_matrix_effects(p, mp, mref)
    }
    /// Broaden interface with MRI model (if set).
    pub fn broaden_profile(&self, depths: &[f64], iface: f64) -> Option<Vec<f64>> {
        self.mri_params.map(|p| broaden_interface(depths, iface, &p))
    }
    /// Depth resolution from MRI model.
    pub fn depth_resolution_nm(&self) -> Option<f64> {
        self.mri_params.map(|p| p.combined_resolution_nm())
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Complementary error function (Abramowitz & Stegun 7.1.26).
fn erfc(x: f64) -> f64 {
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t * (0.254829592 + t * (-0.284496736
        + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let r = poly * (-x * x).exp();
    if x >= 0.0 { r } else { 2.0 - r }
}

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn test_depth_profile_new() {
        let p = DepthProfile::new("30Si");
        assert_eq!(p.species, "30Si"); assert!(p.is_empty()); assert_eq!(p.len(), 0);
    }
    #[test] fn test_depth_profile_add_len() {
        let mut p = DepthProfile::new("11B");
        p.add_point(0.0, 100.0); p.add_point(1.0, 200.0);
        assert_eq!(p.len(), 2); assert!(!p.is_empty());
    }
    #[test] fn test_depth_profile_times() {
        let mut p = DepthProfile::new("11B");
        p.add_point(0.0, 100.0); p.add_point(5.0, 200.0);
        assert!(approx_eq(p.times()[1], 5.0, 1e-10));
    }
    #[test] fn test_depth_profile_max_intensity() {
        let mut p = DepthProfile::new("11B");
        p.add_point(0.0, 100.0); p.add_point(1.0, 500.0); p.add_point(2.0, 300.0);
        assert!(approx_eq(p.max_intensity(), 500.0, 1e-10));
    }
    #[test] fn test_integrated_counts_rect() {
        let mut p = DepthProfile::new("t");
        p.add_point(0.0, 100.0); p.add_point(10.0, 100.0);
        assert!(approx_eq(p.integrated_counts(), 1000.0, 1e-6));
    }
    #[test] fn test_integrated_counts_triangle() {
        let mut p = DepthProfile::new("t");
        p.add_point(0.0, 0.0); p.add_point(10.0, 100.0);
        assert!(approx_eq(p.integrated_counts(), 500.0, 1e-6));
    }
    #[test] fn test_sputter_yield_positive() {
        let r = sputter_yield(200.0, 1000.0, 50.0, 62500.0, 5e22);
        assert!(r.yield_atoms_per_ion > 0.0 && r.yield_atoms_per_ion < 100.0);
    }
    #[test] fn test_sputter_yield_erosion_rate() {
        let r = sputter_yield(500.0, 2000.0, 100.0, 40000.0, 5e22);
        assert!(approx_eq(r.erosion_rate_nm_s, 0.25, 1e-6));
    }
    #[test] fn test_erosion_rate_from_yield() {
        assert!(erosion_rate_from_yield(2.0, 50.0, 62500.0, 5e22) > 0.0);
    }
    #[test] fn test_erosion_rate_roundtrip() {
        let r = sputter_yield(500.0, 1000.0, 50.0, 62500.0, 5e22);
        assert!(approx_eq(r.erosion_rate_nm_s, 0.5, 1e-6));
    }
    #[test] fn test_mass_spectrum_new() { assert!(MassSpectrum::new().is_empty()); }
    #[test] fn test_mass_spectrum_add() {
        let mut ms = MassSpectrum::new();
        ms.add_peak(28.0, 1e6, Some("28Si")); ms.add_peak(11.0, 1e4, Some("11B"));
        assert_eq!(ms.len(), 2);
    }
    #[test] fn test_mass_spectrum_find() {
        let mut ms = MassSpectrum::new();
        ms.add_peak(27.977, 1e6, Some("28Si")); ms.add_peak(28.976, 5e4, Some("29Si"));
        assert_eq!(ms.find_peaks(28.0, 0.1).len(), 1);
    }
    #[test] fn test_mass_spectrum_base_peak() {
        let mut ms = MassSpectrum::new();
        ms.add_peak(28.0, 1e6, Some("28Si")); ms.add_peak(11.0, 5e3, None);
        assert!(approx_eq(ms.base_peak().unwrap().mz, 28.0, 0.01));
    }
    #[test] fn test_mass_spectrum_tic() {
        let mut ms = MassSpectrum::new();
        ms.add_peak(28.0, 1000.0, None); ms.add_peak(11.0, 500.0, None);
        assert!(approx_eq(ms.total_ion_count(), 1500.0, 1e-6));
    }
    #[test] fn test_mass_resolution() {
        assert!(approx_eq(MassSpectrum::new().mass_resolution(28.0, 0.01), 2800.0, 1e-6));
    }
    #[test] fn test_quantify_rsf() { assert!(approx_eq(quantify_rsf(1e3, 1e6, 1e20), 1e17, 1e12)); }
    #[test] fn test_quantify_rsf_zero() { assert!(approx_eq(quantify_rsf(1e3, 0.0, 1e20), 0.0, 1e-10)); }
    #[test] fn test_conc_depth_profile() {
        let mut sp = DepthProfile::new("11B"); let mut mp = DepthProfile::new("30Si");
        for i in 0..10 { sp.add_point(i as f64, 1e3); mp.add_point(i as f64, 1e6); }
        let c = concentration_depth_profile(&sp, &mp, 1e20, 0.5);
        assert_eq!(c.len(), 10);
        assert!(approx_eq(c[2].0, 1.0, 1e-6)); assert!(approx_eq(c[0].1, 1e17, 1e12));
    }
    #[test] fn test_matrix_correction_unity() {
        assert!(approx_eq(matrix_effect_correction(500.0, 1e6, 1e6), 500.0, 1e-6));
    }
    #[test] fn test_matrix_correction_2x() {
        assert!(approx_eq(matrix_effect_correction(500.0, 0.5e6, 1e6), 1000.0, 1e-6));
    }
    #[test] fn test_correct_matrix_profile() {
        let mut sp = DepthProfile::new("11B"); let mut mp = DepthProfile::new("30Si");
        sp.add_point(0.0, 100.0); sp.add_point(1.0, 200.0);
        mp.add_point(0.0, 1e6); mp.add_point(1.0, 0.5e6);
        let c = correct_matrix_effects(&sp, &mp, 1e6);
        assert!(approx_eq(c.points[0].intensity_cps, 100.0, 1e-6));
        assert!(approx_eq(c.points[1].intensity_cps, 400.0, 1e-6));
    }
    #[test] fn test_abundances_si() {
        let iso = natural_abundances("Si"); assert_eq!(iso.len(), 3);
        let t: f64 = iso.iter().map(|i| i.abundance).sum(); assert!(approx_eq(t, 1.0, 0.001));
    }
    #[test] fn test_abundances_b() {
        assert!(approx_eq(natural_abundances("B")[1].abundance, 0.801, 0.001));
    }
    #[test] fn test_abundances_unknown() { assert_eq!(natural_abundances("Zz").len(), 0); }
    #[test] fn test_abundances_ge() {
        let iso = natural_abundances("Ge"); assert_eq!(iso.len(), 5);
        let t: f64 = iso.iter().map(|i| i.abundance).sum(); assert!(approx_eq(t, 1.0, 0.001));
    }
    #[test] fn test_isotope_ratio() { assert!(approx_eq(isotope_ratio(100.0, 200.0), 0.5, 1e-10)); }
    #[test] fn test_expected_ratio_si() {
        assert!(approx_eq(expected_isotope_ratio("Si", 28, 29), 0.9223 / 0.0467, 0.1));
    }
    #[test] fn test_ratio_deviation_permil() {
        let d = isotope_ratio_deviation_permil(20.0, 19.75);
        assert!(d > 10.0 && d < 20.0);
    }
    #[test] fn test_calibrate_depth_scale() {
        let mut p = DepthProfile::new("t"); p.add_point(0.0, 100.0); p.add_point(10.0, 200.0);
        let c = calibrate_depth_scale(&p, 2.0);
        assert!(approx_eq(c[1].0, 20.0, 1e-10));
    }
    #[test] fn test_erosion_from_crater() {
        assert!(approx_eq(erosion_rate_from_crater(500.0, 1000.0), 0.5, 1e-10));
    }
    #[test] fn test_multilayer_calib() {
        let mut p = DepthProfile::new("t");
        for i in 0..10 { p.add_point(i as f64 * 2.0, 100.0 + i as f64); }
        let layers = vec![(10.0, 1.0), (20.0, 2.0)];
        let c = multilayer_depth_calibration(&layers, &p);
        assert!(approx_eq(c[0].0, 0.0, 1e-10));
        assert!(approx_eq(c[5].0, 10.0, 1e-6)); // t=10: layer1 done
        assert!(approx_eq(c[9].0, 26.0, 1e-6)); // t=18: 10+8*2
    }
    #[test] fn test_mri_resolution() {
        let p = MriParams::new(2.0, 1.5, 1.0);
        assert!(approx_eq(p.combined_resolution_nm(), 7.25_f64.sqrt(), 0.01));
    }
    #[test] fn test_mri_at_interface() {
        let v = mri_response(0.0, &MriParams::new(2.0, 1.0, 0.5));
        assert!(v > 0.3 && v < 0.7);
    }
    #[test] fn test_mri_deep() {
        assert!(mri_response(50.0, &MriParams::new(2.0, 1.0, 0.5)) < 0.01);
    }
    #[test] fn test_mri_far_negative() {
        assert!(mri_response(-50.0, &MriParams::new(2.0, 1.0, 0.5)) > 0.9);
    }
    #[test] fn test_broaden_interface() {
        let p = MriParams::new(2.0, 1.0, 0.5);
        let d: Vec<f64> = (-20..=20).map(|i| i as f64).collect();
        let b = broaden_interface(&d, 0.0, &p);
        assert!(b[0] > 0.9); assert!(*b.last().unwrap() < 0.1);
    }
    #[test] fn test_broaden_delta_peak() {
        let p = MriParams::new(1.0, 0.5, 0.3);
        let d: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
        let r = broaden_delta_layer(&d, 20.0, 1e14, &p);
        let (mi, _) = r.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap();
        assert!((d[mi] - 20.0).abs() < 5.0);
    }
    #[test] fn test_detection_limit_basic() {
        let dl = detection_limit(10.0, 1e6, 1e20, 1.0);
        assert!(dl.concentration_atoms_cm3 > 1e14 && dl.concentration_atoms_cm3 < 1e16);
    }
    #[test] fn test_detection_limit_longer() {
        let s = detection_limit(10.0, 1e6, 1e20, 1.0);
        let l = detection_limit(10.0, 1e6, 1e20, 10.0);
        assert!(l.concentration_atoms_cm3 < s.concentration_atoms_cm3);
    }
    #[test] fn test_dl_from_profile() {
        let mut sp = DepthProfile::new("11B"); let mut mp = DepthProfile::new("30Si");
        for i in 0..100 {
            let bg = if i > 80 { 5.0 } else { 1e4 * (-(i as f64 / 20.0).powi(2)).exp() + 5.0 };
            sp.add_point(i as f64, bg); mp.add_point(i as f64, 1e6);
        }
        assert!(detection_limit_from_profile(&sp, &mp, 1e20, 1.0, 0.2).concentration_atoms_cm3 > 0.0);
    }
    #[test] fn test_processor_new() {
        assert!(approx_eq(SimsProcessor::new(0.5, 50.0, 62500.0).erosion_rate_nm_s, 0.5, 1e-10));
    }
    #[test] fn test_processor_mri() {
        let mut p = SimsProcessor::new(0.5, 50.0, 62500.0);
        p.set_mri_params(2.0, 1.0, 0.5);
        assert!(p.mri_params.is_some());
        assert!(approx_eq(p.depth_resolution_nm().unwrap(), 2.29, 0.1));
    }
    #[test] fn test_processor_quantify() {
        let proc = SimsProcessor::new(0.5, 50.0, 62500.0);
        let mut sp = DepthProfile::new("11B"); let mut mp = DepthProfile::new("30Si");
        for i in 0..5 { sp.add_point(i as f64, 1e3); mp.add_point(i as f64, 1e6); }
        let q = proc.quantify_profile(&sp, &mp, 1e20);
        assert_eq!(q.len(), 5); assert!(approx_eq(q[2].0, 1.0, 1e-6));
    }
    #[test] fn test_processor_sputter_yield() {
        assert!(SimsProcessor::new(0.5, 50.0, 62500.0).calculate_sputter_yield(500.0, 1000.0, 5e22).yield_atoms_per_ion > 0.0);
    }
    #[test] fn test_processor_matrix() {
        let proc = SimsProcessor::new(0.5, 50.0, 62500.0);
        let mut sp = DepthProfile::new("11B"); let mut mp = DepthProfile::new("O");
        sp.add_point(0.0, 200.0); mp.add_point(0.0, 5e5);
        assert!(approx_eq(proc.correct_matrix(&sp, &mp, 1e6).points[0].intensity_cps, 400.0, 1e-6));
    }
    #[test] fn test_processor_broaden_none() {
        assert!(SimsProcessor::new(0.5, 50.0, 62500.0).broaden_profile(&[0.0, 1.0], 0.5).is_none());
    }
    #[test] fn test_processor_broaden_some() {
        let mut p = SimsProcessor::new(0.5, 50.0, 62500.0);
        p.set_mri_params(2.0, 1.0, 0.5);
        let d: Vec<f64> = (0..20).map(|i| i as f64).collect();
        assert_eq!(p.broaden_profile(&d, 10.0).unwrap().len(), 20);
    }
    #[test] fn test_processor_dl() {
        let proc = SimsProcessor::new(0.5, 50.0, 62500.0);
        let mut sp = DepthProfile::new("11B"); let mut mp = DepthProfile::new("30Si");
        for i in 0..50 { sp.add_point(i as f64, 5.0); mp.add_point(i as f64, 1e6); }
        assert!(proc.estimate_detection_limit(&sp, &mp, 1e20, 1.0).concentration_atoms_cm3 > 0.0);
    }
    #[test] fn test_erfc_zero() { assert!(approx_eq(erfc(0.0), 1.0, 0.001)); }
    #[test] fn test_erfc_large() { assert!(erfc(3.0) < 0.001); }
    #[test] fn test_erfc_negative() { assert!(approx_eq(erfc(-3.0), 2.0 - erfc(3.0), 0.001)); }
}
