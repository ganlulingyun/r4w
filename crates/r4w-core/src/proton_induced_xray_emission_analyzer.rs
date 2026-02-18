// proton_induced_xray_emission_analyzer.rs
//
// Proton-Induced X-ray Emission (PIXE) analysis: Moseley's law, Gaussian peak
// fitting, ECPSSR cross-sections, detector efficiency, Bethe stopping power,
// quantification with matrix absorption corrections.

use std::f64::consts::PI;

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

// --- Physical Constants ---
/// Bohr radius (cm).
pub const BOHR_RADIUS_CM: f64 = 5.2918e-9;
/// Electron mass (MeV/c^2).
pub const ELECTRON_MASS_MEV: f64 = 0.51100;
/// Proton mass (MeV/c^2).
pub const PROTON_MASS_MEV: f64 = 938.272;
/// Classical electron radius (cm).
pub const CLASSICAL_ELECTRON_RADIUS_CM: f64 = 2.818e-13;
/// Avogadro's number.
pub const AVOGADRO: f64 = 6.022e23;
/// Rydberg energy (eV).
pub const RYDBERG_EV: f64 = 13.6;

// ---------------------------------------------------------------------------
// 1. X-ray Line Energies and Moseley's Law
// ---------------------------------------------------------------------------

/// X-ray transition line type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XrayLine { KAlpha, KBeta, LAlpha, LBeta }

/// Tabulated X-ray line energies (keV): (Z, symbol, Ka, Kb, La, Lb).
pub const ELEMENT_XRAY_LINES: &[(u32, &str, f64, f64, f64, f64)] = &[
    (11,"Na",1.041,1.071,0.0,0.0),  (12,"Mg",1.254,1.302,0.0,0.0),
    (13,"Al",1.487,1.557,0.0,0.0),  (14,"Si",1.740,1.836,0.0,0.0),
    (15,"P", 2.013,2.139,0.0,0.0),  (16,"S", 2.308,2.464,0.0,0.0),
    (17,"Cl",2.622,2.816,0.0,0.0),  (19,"K", 3.314,3.590,0.0,0.0),
    (20,"Ca",3.692,4.013,0.344,0.345),(22,"Ti",4.511,4.932,0.452,0.458),
    (24,"Cr",5.415,5.947,0.573,0.583),(25,"Mn",5.899,6.490,0.637,0.649),
    (26,"Fe",6.404,7.058,0.705,0.719),(28,"Ni",7.478,8.265,0.852,0.870),
    (29,"Cu",8.048,8.905,0.930,0.950),(30,"Zn",8.639,9.572,1.012,1.035),
    (33,"As",10.544,11.726,1.282,1.317),(35,"Br",11.924,13.292,1.480,1.526),
    (38,"Sr",14.165,15.836,1.807,1.872),(42,"Mo",17.479,19.608,2.293,2.395),
    (47,"Ag",22.163,24.942,2.984,3.151),(50,"Sn",25.271,28.486,3.444,3.663),
    (56,"Ba",32.194,36.378,4.466,4.828),(79,"Au",68.804,77.984,9.713,11.443),
    (82,"Pb",74.969,84.936,10.551,12.614),
];

/// Look up X-ray line energy (keV) by atomic number and line type.
pub fn lookup_xray_energy(z: u32, line: XrayLine) -> Option<f64> {
    for &(ez, _, ka, kb, la, lb) in ELEMENT_XRAY_LINES {
        if ez == z {
            let e = match line {
                XrayLine::KAlpha => ka, XrayLine::KBeta => kb,
                XrayLine::LAlpha => la, XrayLine::LBeta => lb,
            };
            return if e > 0.0 { Some(e) } else { None };
        }
    }
    None
}

/// Look up element symbol by atomic number.
pub fn element_symbol(z: u32) -> Option<&'static str> {
    ELEMENT_XRAY_LINES.iter().find(|e| e.0 == z).map(|e| e.1)
}

/// Moseley's law K-alpha energy (keV): E = 13.6*(Z-sigma)^2*(1-1/4)/1000.
pub fn moseley_k_alpha(z: u32, sigma: f64) -> f64 {
    let ze = z as f64 - sigma;
    RYDBERG_EV * ze * ze * 0.75 / 1000.0
}

/// Moseley's law K-beta energy (keV): E = 13.6*(Z-sigma)^2*(1-1/9)/1000.
pub fn moseley_k_beta(z: u32, sigma: f64) -> f64 {
    let ze = z as f64 - sigma;
    RYDBERG_EV * ze * ze * (8.0 / 9.0) / 1000.0
}

/// Moseley's law L-alpha energy (keV): E = 13.6*(Z-sigma_L)^2*(1/4-1/9)/1000.
pub fn moseley_l_alpha(z: u32, sigma_l: f64) -> f64 {
    let ze = z as f64 - sigma_l;
    RYDBERG_EV * ze * ze * (5.0 / 36.0) / 1000.0
}

// ---------------------------------------------------------------------------
// 2. Spectrum Data
// ---------------------------------------------------------------------------

/// X-ray energy spectrum with linear energy calibration.
#[derive(Debug, Clone)]
pub struct XraySpectrum {
    /// Raw counts per channel.
    pub counts: Vec<f64>,
    /// Energy offset: energy(keV) = offset + gain * channel.
    pub energy_offset_kev: f64,
    /// Energy gain (keV/channel).
    pub energy_gain_kev: f64,
}

impl XraySpectrum {
    /// Create a zero-filled spectrum with given calibration.
    pub fn new(num_channels: usize, offset_kev: f64, gain_kev: f64) -> Self {
        Self { counts: vec![0.0; num_channels], energy_offset_kev: offset_kev, energy_gain_kev: gain_kev }
    }
    /// Channel to energy (keV).
    pub fn channel_to_energy(&self, ch: f64) -> f64 { self.energy_offset_kev + self.energy_gain_kev * ch }
    /// Energy (keV) to channel.
    pub fn energy_to_channel(&self, e: f64) -> f64 {
        if self.energy_gain_kev.abs() < 1e-30 { 0.0 } else { (e - self.energy_offset_kev) / self.energy_gain_kev }
    }
    /// Number of channels.
    pub fn num_channels(&self) -> usize { self.counts.len() }
    /// Total counts in entire spectrum.
    pub fn total_counts(&self) -> f64 { self.counts.iter().sum() }
    /// Counts in a region of interest (inclusive).
    pub fn roi_counts(&self, start: usize, end: usize) -> f64 {
        let e = end.min(self.counts.len().saturating_sub(1));
        self.counts[start.min(e)..=e].iter().sum()
    }
    /// Channel with maximum counts.
    pub fn peak_channel(&self) -> usize {
        self.counts.iter().enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
            .map_or(0, |p| p.0)
    }
}

// ---------------------------------------------------------------------------
// 3. Peak Fitting: Gaussian + Linear Background
// ---------------------------------------------------------------------------

/// Result of Gaussian peak fit.
#[derive(Debug, Clone)]
pub struct PeakFitResult {
    pub centroid_kev: f64, pub amplitude: f64, pub sigma_kev: f64,
    pub bg_slope: f64, pub bg_intercept: f64, pub net_area: f64,
    pub chi_squared: f64,
}

/// Gaussian function: A * exp(-0.5 * ((x-c)/s)^2).
pub fn gaussian(x: f64, amp: f64, center: f64, sigma: f64) -> f64 {
    if sigma.abs() < 1e-30 { return 0.0; }
    let a = (x - center) / sigma;
    amp * (-0.5 * a * a).exp()
}

/// Fit Gaussian + linear background to (energy, counts) data.
pub fn fit_peak(energies: &[f64], counts: &[f64], max_iter: usize) -> PeakFitResult {
    let n = energies.len().min(counts.len());
    let zero = PeakFitResult { centroid_kev:0.0, amplitude:0.0, sigma_kev:0.01,
        bg_slope:0.0, bg_intercept:0.0, net_area:0.0, chi_squared:f64::MAX };
    if n < 5 { return zero; }
    // Background from edges
    let bl = (counts[0]+counts[1])/2.0;
    let br = (counts[n-2]+counts[n-1])/2.0;
    let el = (energies[0]+energies[1])/2.0;
    let er = (energies[n-2]+energies[n-1])/2.0;
    let de = er - el;
    let bg_slope = if de.abs()>1e-30 { (br-bl)/de } else { 0.0 };
    let bg_int = bl - bg_slope * el;
    // Background-subtracted net counts
    let net: Vec<f64> = (0..n).map(|i| (counts[i] - bg_slope*energies[i] - bg_int).max(0.0)).collect();
    let total: f64 = net.iter().sum();
    if total < 1.0 { return PeakFitResult { bg_slope, bg_intercept: bg_int, ..zero }; }
    let mut cen: f64 = net.iter().enumerate().map(|(i,&v)| energies[i]*v).sum::<f64>() / total;
    let var: f64 = net.iter().enumerate().map(|(i,&v)| { let d=energies[i]-cen; d*d*v }).sum::<f64>() / total;
    let mut sig = var.sqrt().max(0.001);
    let mut amp = total / (sig * (2.0*PI).sqrt());
    for _ in 0..max_iter {
        let (mut dc, mut ds) = (0.0, 0.0);
        for i in 0..n {
            let model = gaussian(energies[i], amp, cen, sig) + bg_slope*energies[i] + bg_int;
            let res = counts[i] - model;
            let w = 1.0 / counts[i].max(1.0);
            let a = (energies[i]-cen)/sig;
            let g = (-0.5*a*a).exp();
            dc += res*w*amp*g*a/sig;
            ds += res*w*amp*g*(a*a-1.0)/sig;
        }
        cen += 0.001*dc; sig = (sig + 0.0005*ds).max(0.001);
        amp = total / (sig * (2.0*PI).sqrt());
    }
    let chi2: f64 = (0..n).map(|i| {
        let m = gaussian(energies[i],amp,cen,sig) + bg_slope*energies[i]+bg_int;
        let r = counts[i]-m; r*r/counts[i].max(1.0)
    }).sum();
    PeakFitResult { centroid_kev:cen, amplitude:amp, sigma_kev:sig,
        bg_slope, bg_intercept:bg_int, net_area: amp*sig*(2.0*PI).sqrt(), chi_squared:chi2 }
}

/// Detector resolution FWHM (keV): sqrt(noise^2 + 2.355^2 * F * 3.58e-3 * E).
pub fn detector_resolution_fwhm(energy_kev: f64, noise_kev: f64, fano: f64) -> f64 {
    (noise_kev*noise_kev + 2.355*2.355*fano*3.58e-3*energy_kev).sqrt()
}

// ---------------------------------------------------------------------------
// 4. Ionization Cross-Section (ECPSSR Approximation)
// ---------------------------------------------------------------------------

/// Simplified ECPSSR K-shell ionization cross-section (barn).
pub fn ecpssr_k_cross_section(z_target: u32, proton_energy_mev: f64) -> f64 {
    let z = z_target as f64;
    let u_k_mev = RYDBERG_EV * z * z / 1e6;
    let eta = ((proton_energy_mev/PROTON_MASS_MEV) / (u_k_mev/ELECTRON_MASS_MEV)).sqrt();
    if eta < 0.01 { return 0.0; }
    let f = if eta > 5.0 { eta.ln().max(0.1)/eta } else { eta*eta*(-1.0/eta).exp() };
    let s0 = 8.0*PI*BOHR_RADIUS_CM*BOHR_RADIUS_CM*1e24;
    let sig = s0 * f / (z*z);
    let cc = 1.0/(1.0+0.3/eta);
    let bc = if eta < 1.0 { (0.5*eta).exp() } else { 1.0 };
    (sig * cc * bc).max(0.0)
}

/// Simplified ECPSSR L-shell ionization cross-section (barn).
pub fn ecpssr_l_cross_section(z_target: u32, proton_energy_mev: f64) -> f64 {
    let ze = (z_target as f64 - 7.4).max(1.0);
    let u_l_mev = RYDBERG_EV * ze * ze / (4.0*1e6);
    let eta = ((proton_energy_mev/PROTON_MASS_MEV) / (u_l_mev/ELECTRON_MASS_MEV)).sqrt();
    if eta < 0.01 { return 0.0; }
    let f = if eta > 5.0 { eta.ln().max(0.1)/eta } else { eta*eta*(-1.0/eta).exp() };
    let s0 = 8.0*PI*BOHR_RADIUS_CM*BOHR_RADIUS_CM*1e24;
    (s0 * 8.0 * f / (ze*ze) / (1.0+0.2/eta)).max(0.0)
}

/// K-shell fluorescence yield: omega_K ~ Z^4 / (Z^4 + 24^4).
pub fn fluorescence_yield_k(z: u32) -> f64 {
    let z4 = (z as f64).powi(4);
    z4 / (z4 + 24.0_f64.powi(4))
}

/// L-shell fluorescence yield (approximate).
pub fn fluorescence_yield_l(z: u32) -> f64 {
    let z4 = (z as f64).powi(4);
    z4 / (z4 + 30.0_f64.powi(4))
}

// ---------------------------------------------------------------------------
// 5. Detector Efficiency
// ---------------------------------------------------------------------------

/// Detector type for PIXE.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType { SiLi, Sdd }

/// Detector efficiency model (Be window + Si dead layer + active layer).
#[derive(Debug, Clone)]
pub struct DetectorEfficiency {
    pub detector_type: DetectorType,
    pub active_thickness_cm: f64,
    pub be_window_cm: f64,
    pub dead_layer_cm: f64,
}

impl DetectorEfficiency {
    /// Standard Si(Li) (3 mm active, 8 um Be window).
    pub fn standard_sili() -> Self {
        Self { detector_type: DetectorType::SiLi, active_thickness_cm: 0.3,
            be_window_cm: 8.0e-4, dead_layer_cm: 1.0e-5 }
    }
    /// Standard SDD (0.45 mm active, 8 um Be window).
    pub fn standard_sdd() -> Self {
        Self { detector_type: DetectorType::Sdd, active_thickness_cm: 0.045,
            be_window_cm: 8.0e-4, dead_layer_cm: 0.5e-5 }
    }
    /// Intrinsic efficiency: T_window * T_dead * (1 - exp(-mu*d_active)).
    pub fn efficiency(&self, energy_kev: f64) -> f64 {
        let mu_be = 600.0 * energy_kev.powf(-2.5) * 1.848;
        let mu_si = mass_atten_si(energy_kev) * 2.33;
        (-mu_be * self.be_window_cm).exp()
            * (-mu_si * self.dead_layer_cm).exp()
            * (1.0 - (-mu_si * self.active_thickness_cm).exp())
    }
}

fn mass_atten_si(e: f64) -> f64 {
    if e < 0.1 { return 1e5; }
    let b = 350.0 * e.powf(-2.8);
    if e >= 1.839 && e < 2.2 { b * 7.0 } else { b }
}

// ---------------------------------------------------------------------------
// 6. Stopping Power (Bethe Formula)
// ---------------------------------------------------------------------------

/// Bethe stopping power for protons (MeV cm^2/g).
pub fn bethe_stopping_power(z_t: u32, a_t: f64, ep: f64) -> f64 {
    let z = z_t as f64;
    let gamma = 1.0 + ep / PROTON_MASS_MEV;
    let b2 = 1.0 - 1.0/(gamma*gamma);
    if b2 < 1e-15 { return 0.0; }
    let i_ev = if z < 13.0 { 12.0*z+7.0 } else { 9.76*z+58.8*z.powf(-0.19) };
    let wmax = 2.0*ELECTRON_MASS_MEV*b2*gamma*gamma;
    let la = wmax / (i_ev*1e-6);
    if la <= 0.0 { return 0.0; }
    4.0*PI*AVOGADRO*z*CLASSICAL_ELECTRON_RADIUS_CM.powi(2)*ELECTRON_MASS_MEV/(a_t*b2)
        * (la.ln() - b2)
}

/// Proton range in material (mg/cm^2) via numerical integration.
pub fn proton_range(z_t: u32, a_t: f64, energy_mev: f64, steps: usize) -> f64 {
    let de = energy_mev / steps as f64;
    let mut r = 0.0; let mut e = energy_mev;
    for _ in 0..steps {
        let sp = bethe_stopping_power(z_t, a_t, e);
        if sp > 1e-10 { r += de/sp; }
        e -= de;
        if e < 0.01 { break; }
    }
    r * 1000.0
}

// ---------------------------------------------------------------------------
// 7. Matrix Absorption Correction
// ---------------------------------------------------------------------------

/// Generic mass attenuation (cm^2/g): power-law approximation.
pub fn mass_attenuation_generic(z: u32, energy_kev: f64) -> f64 {
    if energy_kev < 0.1 { return 1e6; }
    let zf = z as f64;
    0.02 * zf.powi(4) / (energy_kev.powi(3) * (zf*2.0))
}

/// Matrix absorption correction factor for thick targets.
pub fn matrix_correction(sp: f64, mu_x: f64, takeoff_deg: f64) -> f64 {
    let csc = 1.0 / (takeoff_deg*PI/180.0).sin().abs().max(1e-10);
    let s = sp.abs();
    let d = s + mu_x*csc*0.001;
    if d < 1e-30 { 1.0 } else { s/d }
}

// ---------------------------------------------------------------------------
// 8. PIXE Quantification
// ---------------------------------------------------------------------------

/// Quantification result for a single element.
#[derive(Debug, Clone)]
pub struct PixeElementResult {
    pub z: u32, pub symbol: String, pub line: XrayLine,
    pub net_area: f64, pub cross_section_barn: f64,
    pub detector_efficiency: f64, pub fluorescence_yield: f64,
    pub concentration: f64, pub detection_limit: f64,
}

/// PIXE concentration: C = (Y*H) / (sigma*epsilon*omega*Omega*Q).
pub fn pixe_concentration(yield_c: f64, sigma_barn: f64, eff: f64,
    omega: f64, solid_angle: f64, charge_uc: f64, h: f64) -> f64 {
    let q = charge_uc * 1e-6 / 1.602e-19;
    let d = sigma_barn*1e-24 * eff * omega * solid_angle * q;
    if d.abs() < 1e-40 { 0.0 } else { yield_c * h / d }
}

/// Minimum Detection Limit (3-sigma): MDL = 3*sqrt(bg)*C/Y.
pub fn detection_limit(bg: f64, net: f64, conc: f64) -> f64 {
    if net.abs() < 1e-10 { f64::INFINITY } else { 3.0*bg.sqrt()*conc/net }
}

// ---------------------------------------------------------------------------
// 9. PixeAnalyzer Orchestrator
// ---------------------------------------------------------------------------

/// PIXE analysis configuration.
#[derive(Debug, Clone)]
pub struct PixeConfig {
    pub beam_energy_mev: f64, pub charge_uc: f64, pub solid_angle_sr: f64,
    pub takeoff_angle_deg: f64, pub h_factor: f64, pub detector: DetectorEfficiency,
}

impl PixeConfig {
    /// Typical: 3 MeV protons, SDD, 10 msr, 45 deg.
    pub fn typical() -> Self {
        Self { beam_energy_mev: 3.0, charge_uc: 1.0, solid_angle_sr: 0.01,
            takeoff_angle_deg: 45.0, h_factor: 1.0, detector: DetectorEfficiency::standard_sdd() }
    }
}

/// Main PIXE analyzer orchestrator.
#[derive(Debug, Clone)]
pub struct PixeAnalyzer {
    pub config: PixeConfig,
    pub spectrum: Option<XraySpectrum>,
    pub results: Vec<PixeElementResult>,
}

impl PixeAnalyzer {
    /// Create analyzer with given configuration.
    pub fn new(config: PixeConfig) -> Self { Self { config, spectrum: None, results: Vec::new() } }
    /// Create with typical configuration.
    pub fn typical() -> Self { Self::new(PixeConfig::typical()) }
    /// Load spectrum data.
    pub fn load_spectrum(&mut self, s: XraySpectrum) { self.spectrum = Some(s); }
    /// Clear results.
    pub fn clear_results(&mut self) { self.results.clear(); }
    /// Summary of results: (label, concentration).
    pub fn summary(&self) -> Vec<(String, f64)> {
        self.results.iter().map(|r| (format!("{} ({:?})", r.symbol, r.line), r.concentration)).collect()
    }

    /// Analyze a specific element from loaded spectrum.
    pub fn analyze_element(&mut self, z: u32, line: XrayLine) -> Option<PixeElementResult> {
        let spec = self.spectrum.as_ref()?;
        let le = lookup_xray_energy(z, line)?;
        let cs = spec.energy_to_channel(le - 0.5).max(0.0) as usize;
        let ce = (spec.energy_to_channel(le + 0.5) as usize).min(spec.num_channels().saturating_sub(1));
        if ce <= cs + 4 { return None; }
        let en: Vec<f64> = (cs..=ce).map(|c| spec.channel_to_energy(c as f64)).collect();
        let ct: Vec<f64> = (cs..=ce).map(|c| spec.counts[c]).collect();
        let fit = fit_peak(&en, &ct, 50);
        let sigma = match line {
            XrayLine::KAlpha|XrayLine::KBeta => ecpssr_k_cross_section(z, self.config.beam_energy_mev),
            _ => ecpssr_l_cross_section(z, self.config.beam_energy_mev),
        };
        let omega = match line {
            XrayLine::KAlpha|XrayLine::KBeta => fluorescence_yield_k(z),
            _ => fluorescence_yield_l(z),
        };
        let eff = self.config.detector.efficiency(le);
        let conc = pixe_concentration(fit.net_area, sigma, eff, omega,
            self.config.solid_angle_sr, self.config.charge_uc, self.config.h_factor);
        let bg_c = (fit.bg_intercept+fit.bg_slope*le).abs()*(2.355*fit.sigma_kev/spec.energy_gain_kev).max(1.0);
        let mdl = detection_limit(bg_c, fit.net_area, conc);
        let sym = element_symbol(z).unwrap_or("??").to_string();
        let r = PixeElementResult { z, symbol: sym, line, net_area: fit.net_area,
            cross_section_barn: sigma, detector_efficiency: eff, fluorescence_yield: omega,
            concentration: conc, detection_limit: mdl };
        self.results.push(r.clone());
        Some(r)
    }

    /// Analyze all tabulated elements for K-alpha lines.
    pub fn analyze_all_k_alpha(&mut self) -> Vec<PixeElementResult> {
        let zs: Vec<u32> = ELEMENT_XRAY_LINES.iter().map(|e| e.0).collect();
        zs.iter().filter_map(|&z| self.analyze_element(z, XrayLine::KAlpha)).collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    fn make_fe_spectrum() -> XraySpectrum {
        let mut s = XraySpectrum::new(2048, 0.0, 0.01);
        for ch in 590..690 {
            let a = (ch as f64 - 640.0) / 7.0;
            s.counts[ch] = 500.0 * (-0.5*a*a).exp() + 5.0;
        }
        s
    }

    // Moseley's law
    #[test] fn moseley_ka_iron() {
        let e = moseley_k_alpha(26, 1.7);
        assert!(e > 5.0 && e < 8.0, "Fe Ka={}", e);
    }
    #[test] fn moseley_ka_increases_with_z() {
        assert!(moseley_k_alpha(29, 1.7) > moseley_k_alpha(26, 1.7));
    }
    #[test] fn moseley_kb_gt_ka() {
        assert!(moseley_k_beta(26, 1.7) > moseley_k_alpha(26, 1.7));
    }
    #[test] fn moseley_la_copper() {
        let e = moseley_l_alpha(29, 7.4);
        assert!(e > 0.5 && e < 2.0, "Cu La={}", e);
    }
    #[test] fn moseley_screening() {
        assert!(moseley_k_alpha(20, 1.0) > moseley_k_alpha(20, 2.0));
    }

    // X-ray line lookup
    #[test] fn lookup_fe_ka() { assert!(approx_eq(lookup_xray_energy(26, XrayLine::KAlpha).unwrap(), 6.404, 0.01)); }
    #[test] fn lookup_cu_kb() { assert!(approx_eq(lookup_xray_energy(29, XrayLine::KBeta).unwrap(), 8.905, 0.01)); }
    #[test] fn lookup_pb_la() { assert!(approx_eq(lookup_xray_energy(82, XrayLine::LAlpha).unwrap(), 10.551, 0.01)); }
    #[test] fn lookup_na_no_l() { assert!(lookup_xray_energy(11, XrayLine::LAlpha).is_none()); }
    #[test] fn lookup_unknown_z() { assert!(lookup_xray_energy(99, XrayLine::KAlpha).is_none()); }
    #[test] fn symbol_fe() { assert_eq!(element_symbol(26), Some("Fe")); }
    #[test] fn symbol_unknown() { assert_eq!(element_symbol(99), None); }

    // Spectrum
    #[test] fn spectrum_energy_conv() {
        let s = XraySpectrum::new(2048, 0.0, 0.01);
        assert!(approx_eq(s.channel_to_energy(640.0), 6.4, 0.01));
        assert!(approx_eq(s.energy_to_channel(6.4), 640.0, 0.5));
    }
    #[test] fn spectrum_total() {
        let mut s = XraySpectrum::new(100, 0.0, 0.1);
        s.counts[50] = 1000.0; s.counts[51] = 500.0;
        assert!(approx_eq(s.total_counts(), 1500.0, 0.1));
    }
    #[test] fn spectrum_roi() {
        let mut s = XraySpectrum::new(100, 0.0, 0.1);
        for i in 40..60 { s.counts[i] = 100.0; }
        assert!(approx_eq(s.roi_counts(40, 59), 2000.0, 0.1));
    }
    #[test] fn spectrum_peak() {
        let mut s = XraySpectrum::new(100, 0.0, 0.1);
        s.counts[42] = 999.0;
        assert_eq!(s.peak_channel(), 42);
    }

    // Gaussian + fitting
    #[test] fn gauss_at_center() { assert!(approx_eq(gaussian(5.0, 100.0, 5.0, 0.1), 100.0, 0.01)); }
    #[test] fn gauss_at_tail() { assert!(gaussian(0.0, 100.0, 5.0, 0.1) < 1e-10); }
    #[test] fn fit_synthetic_peak() {
        let n = 50; let (c, s, a) = (6.4, 0.07, 500.0);
        let en: Vec<f64> = (0..n).map(|i| 5.9 + i as f64*0.02).collect();
        let ct: Vec<f64> = en.iter().map(|&e| gaussian(e, a, c, s)+10.0).collect();
        let fit = fit_peak(&en, &ct, 100);
        assert!(approx_eq(fit.centroid_kev, c, 0.05), "cen={}", fit.centroid_kev);
        assert!(fit.net_area > 0.0);
    }
    #[test] fn det_resolution() {
        let f = detector_resolution_fwhm(6.4, 0.06, 0.115);
        assert!(f > 0.05 && f < 0.2);
    }

    // Cross-sections
    #[test] fn ecpssr_k_fe() { assert!(ecpssr_k_cross_section(26, 3.0) > 0.0); }
    #[test] fn ecpssr_k_increases() { assert!(ecpssr_k_cross_section(26, 3.0) > ecpssr_k_cross_section(26, 1.5)); }
    #[test] fn ecpssr_l_positive() { assert!(ecpssr_l_cross_section(82, 3.0) > 0.0); }
    #[test] fn ecpssr_below_threshold() { assert!(approx_eq(ecpssr_k_cross_section(82, 0.001), 0.0, 1e-10)); }
    #[test] fn fluor_k_low_z() { assert!(fluorescence_yield_k(11) < 0.1); }
    #[test] fn fluor_k_high_z() { assert!(fluorescence_yield_k(82) > 0.9); }
    #[test] fn fluor_k_increases() { assert!(fluorescence_yield_k(29) > fluorescence_yield_k(14)); }
    #[test] fn fluor_l_range() { let w = fluorescence_yield_l(82); assert!(w > 0.0 && w < 1.0); }

    // Detector efficiency
    #[test] fn eff_sili() { let e = DetectorEfficiency::standard_sili().efficiency(6.4); assert!(e > 0.5 && e < 1.0, "e={}", e); }
    #[test] fn eff_sdd() { let e = DetectorEfficiency::standard_sdd().efficiency(6.4); assert!(e > 0.1 && e < 1.0, "e={}", e); }
    #[test] fn eff_low_energy_loss() {
        let d = DetectorEfficiency::standard_sili();
        assert!(d.efficiency(0.5) < d.efficiency(5.0));
    }

    // Stopping power
    #[test] fn bethe_positive() { assert!(bethe_stopping_power(14, 28.09, 3.0) > 0.0); }
    #[test] fn bethe_decreases() { assert!(bethe_stopping_power(14, 28.09, 1.0) > bethe_stopping_power(14, 28.09, 3.0)); }
    #[test] fn bethe_finite() { assert!(bethe_stopping_power(82, 207.2, 3.0).is_finite()); }
    #[test] fn range_positive() { assert!(proton_range(14, 28.09, 3.0, 1000) > 0.0); }
    #[test] fn range_increases() { assert!(proton_range(14, 28.09, 3.0, 1000) > proton_range(14, 28.09, 2.0, 1000)); }

    // Matrix correction
    #[test] fn matrix_corr_range() {
        let sp = bethe_stopping_power(14, 28.09, 3.0);
        let mu = mass_attenuation_generic(26, 6.4);
        let f = matrix_correction(sp, mu, 45.0);
        assert!(f > 0.0 && f <= 1.0, "f={}", f);
    }
    #[test] fn matrix_corr_grazing() {
        let sp = bethe_stopping_power(14, 28.09, 3.0);
        let mu = mass_attenuation_generic(26, 6.4);
        assert!(matrix_correction(sp, mu, 10.0) < matrix_correction(sp, mu, 45.0));
    }

    // Quantification
    #[test] fn conc_nonzero() { assert!(pixe_concentration(1000.0, 100.0, 0.9, 0.5, 0.01, 1.0, 1.0) > 0.0); }
    #[test] fn conc_scales() {
        let c1 = pixe_concentration(1000.0, 100.0, 0.9, 0.5, 0.01, 1.0, 1.0);
        assert!(approx_eq(pixe_concentration(2000.0, 100.0, 0.9, 0.5, 0.01, 1.0, 1.0), 2.0*c1, c1*0.01));
    }
    #[test] fn conc_zero_yield() { assert!(approx_eq(pixe_concentration(0.0, 100.0, 0.9, 0.5, 0.01, 1.0, 1.0), 0.0, 1e-20)); }
    #[test] fn mdl_positive() { assert!(detection_limit(100.0, 1000.0, 50.0) > 0.0); }
    #[test] fn mdl_improves() { assert!(detection_limit(100.0, 2000.0, 50.0) < detection_limit(100.0, 500.0, 50.0)); }

    // Attenuation
    #[test] fn atten_positive() { assert!(mass_attenuation_generic(26, 6.4) > 0.0); }
    #[test] fn atten_decreases() { assert!(mass_attenuation_generic(26, 2.0) > mass_attenuation_generic(26, 10.0)); }

    // PixeAnalyzer
    #[test] fn analyzer_creation() {
        let a = PixeAnalyzer::typical();
        assert!(approx_eq(a.config.beam_energy_mev, 3.0, 0.01));
        assert!(a.results.is_empty());
    }
    #[test] fn analyzer_load() {
        let mut a = PixeAnalyzer::typical();
        a.load_spectrum(XraySpectrum::new(2048, 0.0, 0.01));
        assert!(a.spectrum.is_some());
    }
    #[test] fn analyzer_fe() {
        let mut a = PixeAnalyzer::typical();
        a.load_spectrum(make_fe_spectrum());
        let r = a.analyze_element(26, XrayLine::KAlpha).unwrap();
        assert_eq!(r.z, 26);
        assert!(r.net_area > 0.0);
    }
    #[test] fn analyzer_no_spectrum() {
        assert!(PixeAnalyzer::typical().analyze_element(26, XrayLine::KAlpha).is_none());
    }
    #[test] fn analyzer_clear() {
        let mut a = PixeAnalyzer::typical();
        a.load_spectrum(make_fe_spectrum());
        a.analyze_element(26, XrayLine::KAlpha);
        assert!(!a.results.is_empty());
        a.clear_results();
        assert!(a.results.is_empty());
    }
    #[test] fn analyzer_summary() {
        let mut a = PixeAnalyzer::typical();
        a.load_spectrum(make_fe_spectrum());
        a.analyze_element(26, XrayLine::KAlpha);
        let s = a.summary();
        assert_eq!(s.len(), 1);
        assert!(s[0].0.contains("Fe"));
    }
}
