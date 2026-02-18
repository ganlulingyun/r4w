// X-Ray Fluorescence (XRF) Spectrometer Processor
// Bulk elemental analysis using characteristic X-ray fluorescence
// Implements: fundamental parameters method, empirical calibration,
// matrix effects (absorption/enhancement), Compton/Rayleigh scatter,
// detection limits, thin film correction

use std::f64::consts::PI;

/// XRF configuration
#[derive(Clone, Debug)]
pub struct XrfConfig {
    /// Tube voltage in kV
    pub tube_voltage_kv: f64,
    /// Tube current in mA
    pub tube_current_ma: f64,
    /// Tube anode material
    pub anode: AnodeMaterial,
    /// Incident angle in degrees
    pub incident_angle_deg: f64,
    /// Takeoff angle in degrees
    pub takeoff_angle_deg: f64,
    /// Analysis atmosphere
    pub atmosphere: Atmosphere,
}

#[derive(Clone, Debug)]
pub enum AnodeMaterial {
    Rhodium,
    Molybdenum,
    Tungsten,
    Chromium,
}

#[derive(Clone, Debug)]
pub enum Atmosphere {
    Vacuum,
    Helium,
    Air,
}

impl Default for XrfConfig {
    fn default() -> Self {
        Self {
            tube_voltage_kv: 50.0,
            tube_current_ma: 1.0,
            anode: AnodeMaterial::Rhodium,
            incident_angle_deg: 60.0,
            takeoff_angle_deg: 40.0,
            atmosphere: Atmosphere::Vacuum,
        }
    }
}

/// XRF element data
#[derive(Clone, Debug)]
pub struct XrfElement {
    pub symbol: &'static str,
    pub z: u32,
    /// Ka energy in keV
    pub ka_kev: f64,
    /// Kb energy in keV
    pub kb_kev: f64,
    /// La energy in keV (0 if N/A)
    pub la_kev: f64,
    /// K-edge energy in keV
    pub k_edge_kev: f64,
    /// Fluorescence yield (omega_K)
    pub fluorescence_yield: f64,
    /// Jump ratio (r_K)
    pub jump_ratio: f64,
}

/// XRF element database
pub fn xrf_element_database() -> Vec<XrfElement> {
    vec![
        XrfElement { symbol: "Na", z: 11, ka_kev: 1.041, kb_kev: 1.071, la_kev: 0.0, k_edge_kev: 1.072, fluorescence_yield: 0.023, jump_ratio: 10.4 },
        XrfElement { symbol: "Mg", z: 12, ka_kev: 1.254, kb_kev: 1.302, la_kev: 0.0, k_edge_kev: 1.305, fluorescence_yield: 0.032, jump_ratio: 10.3 },
        XrfElement { symbol: "Al", z: 13, ka_kev: 1.487, kb_kev: 1.557, la_kev: 0.0, k_edge_kev: 1.560, fluorescence_yield: 0.042, jump_ratio: 10.1 },
        XrfElement { symbol: "Si", z: 14, ka_kev: 1.740, kb_kev: 1.836, la_kev: 0.0, k_edge_kev: 1.839, fluorescence_yield: 0.054, jump_ratio: 9.8 },
        XrfElement { symbol: "P",  z: 15, ka_kev: 2.013, kb_kev: 2.139, la_kev: 0.0, k_edge_kev: 2.145, fluorescence_yield: 0.068, jump_ratio: 9.5 },
        XrfElement { symbol: "S",  z: 16, ka_kev: 2.308, kb_kev: 2.464, la_kev: 0.0, k_edge_kev: 2.472, fluorescence_yield: 0.084, jump_ratio: 9.2 },
        XrfElement { symbol: "K",  z: 19, ka_kev: 3.314, kb_kev: 3.590, la_kev: 0.0, k_edge_kev: 3.608, fluorescence_yield: 0.140, jump_ratio: 8.4 },
        XrfElement { symbol: "Ca", z: 20, ka_kev: 3.692, kb_kev: 4.013, la_kev: 0.344, k_edge_kev: 4.039, fluorescence_yield: 0.163, jump_ratio: 8.2 },
        XrfElement { symbol: "Ti", z: 22, ka_kev: 4.511, kb_kev: 4.932, la_kev: 0.452, k_edge_kev: 4.966, fluorescence_yield: 0.214, jump_ratio: 7.8 },
        XrfElement { symbol: "Cr", z: 24, ka_kev: 5.415, kb_kev: 5.947, la_kev: 0.573, k_edge_kev: 5.989, fluorescence_yield: 0.272, jump_ratio: 7.5 },
        XrfElement { symbol: "Mn", z: 25, ka_kev: 5.899, kb_kev: 6.490, la_kev: 0.637, k_edge_kev: 6.539, fluorescence_yield: 0.303, jump_ratio: 7.3 },
        XrfElement { symbol: "Fe", z: 26, ka_kev: 6.404, kb_kev: 7.058, la_kev: 0.705, k_edge_kev: 7.112, fluorescence_yield: 0.335, jump_ratio: 7.1 },
        XrfElement { symbol: "Ni", z: 28, ka_kev: 7.472, kb_kev: 8.265, la_kev: 0.851, k_edge_kev: 8.333, fluorescence_yield: 0.402, jump_ratio: 6.8 },
        XrfElement { symbol: "Cu", z: 29, ka_kev: 8.048, kb_kev: 8.905, la_kev: 0.930, k_edge_kev: 8.979, fluorescence_yield: 0.437, jump_ratio: 6.7 },
        XrfElement { symbol: "Zn", z: 30, ka_kev: 8.639, kb_kev: 9.572, la_kev: 1.012, k_edge_kev: 9.659, fluorescence_yield: 0.471, jump_ratio: 6.5 },
        XrfElement { symbol: "Pb", z: 82, ka_kev: 74.97, kb_kev: 84.94, la_kev: 10.55, k_edge_kev: 88.00, fluorescence_yield: 0.968, jump_ratio: 4.6 },
    ]
}

/// XRF spectrum
#[derive(Clone, Debug)]
pub struct XrfSpectrum {
    pub energy_kev: Vec<f64>,
    pub counts: Vec<f64>,
    pub live_time_s: f64,
}

/// Calibration standard
#[derive(Clone, Debug)]
pub struct CalibrationStandard {
    pub name: String,
    pub element: String,
    pub known_concentration_ppm: f64,
    pub measured_intensity: f64,
}

/// XRF Processor
pub struct XrfProcessor {
    pub config: XrfConfig,
    pub spectrum: XrfSpectrum,
}

impl XrfProcessor {
    pub fn new(config: XrfConfig, spectrum: XrfSpectrum) -> Self {
        Self { config, spectrum }
    }

    /// Fundamental parameters: intensity for element i
    /// I_i = G * C_i * Q_i * omega_i * p_i * (1/mu_tot)
    /// Simplified: I proportional to C * omega * (r-1)/r
    pub fn fundamental_intensity(
        concentration: f64,
        fluorescence_yield: f64,
        jump_ratio: f64,
        absorption_factor: f64,
    ) -> f64 {
        concentration * fluorescence_yield * (jump_ratio - 1.0) / jump_ratio * absorption_factor
    }

    /// Absorption correction factor for thick sample
    /// f = 1 / (mu_inc / sin(psi1) + mu_emit / sin(psi2))
    /// where mu is total mass absorption coefficient
    pub fn absorption_correction(
        mu_incident: f64,
        mu_emission: f64,
        incident_angle_deg: f64,
        takeoff_angle_deg: f64,
    ) -> f64 {
        let csc_inc = 1.0 / (incident_angle_deg * PI / 180.0).sin();
        let csc_take = 1.0 / (takeoff_angle_deg * PI / 180.0).sin();
        let mu_total = mu_incident * csc_inc + mu_emission * csc_take;
        if mu_total > 0.0 { 1.0 / mu_total } else { 0.0 }
    }

    /// Enhancement (secondary fluorescence) correction
    /// Element j enhances element i if E_j > E_edge_i
    pub fn enhancement_factor(
        conc_j: f64,
        mu_j_at_edge_i: f64,
        omega_j: f64,
        jump_ratio_j: f64,
    ) -> f64 {
        conc_j * mu_j_at_edge_i * omega_j * (jump_ratio_j - 1.0) / jump_ratio_j * 0.5
    }

    /// Empirical calibration: C = a + b * I (linear)
    pub fn calibrate_linear(standards: &[CalibrationStandard]) -> (f64, f64) {
        if standards.len() < 2 {
            return (0.0, 1.0);
        }
        let n = standards.len() as f64;
        let sx: f64 = standards.iter().map(|s| s.measured_intensity).sum();
        let sy: f64 = standards.iter().map(|s| s.known_concentration_ppm).sum();
        let sxx: f64 = standards.iter().map(|s| s.measured_intensity.powi(2)).sum();
        let sxy: f64 = standards.iter().map(|s| s.measured_intensity * s.known_concentration_ppm).sum();

        let det = n * sxx - sx * sx;
        if det.abs() < 1e-15 {
            return (0.0, if sx > 0.0 { sy / sx } else { 1.0 });
        }

        let slope = (n * sxy - sx * sy) / det;
        let intercept = (sy - slope * sx) / n;
        (intercept, slope)
    }

    /// Apply calibration to measured intensity
    pub fn apply_calibration(intensity: f64, intercept: f64, slope: f64) -> f64 {
        (intercept + slope * intensity).max(0.0)
    }

    /// Net peak intensity: gross - background
    pub fn net_intensity(
        spectrum: &XrfSpectrum,
        center_kev: f64,
        width_kev: f64,
        bg_method: BgMethod,
    ) -> f64 {
        let mut peak_sum = 0.0;
        let mut peak_count = 0;
        let mut bg_left_sum = 0.0;
        let mut bg_left_count = 0;
        let mut bg_right_sum = 0.0;
        let mut bg_right_count = 0;

        let half_w = width_kev / 2.0;
        let bg_w = width_kev;

        for (i, &e) in spectrum.energy_kev.iter().enumerate() {
            let de = e - center_kev;
            if de.abs() <= half_w {
                peak_sum += spectrum.counts[i];
                peak_count += 1;
            } else if de > -half_w - bg_w && de < -half_w {
                bg_left_sum += spectrum.counts[i];
                bg_left_count += 1;
            } else if de > half_w && de < half_w + bg_w {
                bg_right_sum += spectrum.counts[i];
                bg_right_count += 1;
            }
        }

        let bg_per_channel = match bg_method {
            BgMethod::Linear => {
                let bl = if bg_left_count > 0 { bg_left_sum / bg_left_count as f64 } else { 0.0 };
                let br = if bg_right_count > 0 { bg_right_sum / bg_right_count as f64 } else { 0.0 };
                (bl + br) / 2.0
            }
            BgMethod::Snip => {
                // Simplified: use minimum of left/right
                let bl = if bg_left_count > 0 { bg_left_sum / bg_left_count as f64 } else { 0.0 };
                let br = if bg_right_count > 0 { bg_right_sum / bg_right_count as f64 } else { 0.0 };
                bl.min(br)
            }
        };

        (peak_sum - bg_per_channel * peak_count as f64).max(0.0)
    }

    /// Detection limit (3-sigma): C_DL = 3 * C_std * sqrt(I_bg) / I_net
    pub fn detection_limit_ppm(
        std_conc_ppm: f64,
        net_intensity: f64,
        bg_intensity: f64,
        time_s: f64,
    ) -> f64 {
        if net_intensity > 0.0 && time_s > 0.0 {
            3.0 * std_conc_ppm * (bg_intensity * time_s).sqrt() / (net_intensity * time_s)
        } else {
            f64::INFINITY
        }
    }

    /// Compton-to-Rayleigh scatter ratio for matrix estimation
    /// Higher C/R = lower average Z (lighter matrix)
    pub fn compton_rayleigh_ratio(compton_intensity: f64, rayleigh_intensity: f64) -> f64 {
        if rayleigh_intensity > 0.0 {
            compton_intensity / rayleigh_intensity
        } else {
            0.0
        }
    }

    /// Thin film thickness from intensity ratio: I/I_inf = 1 - exp(-mu*rho*d)
    pub fn thin_film_thickness_um(
        intensity_ratio: f64,
        mu_total: f64,
        density_g_cm3: f64,
    ) -> f64 {
        if mu_total > 0.0 && density_g_cm3 > 0.0 && intensity_ratio > 0.0 && intensity_ratio < 1.0 {
            -(1.0 - intensity_ratio).ln() / (mu_total * density_g_cm3) * 1e4
        } else {
            0.0
        }
    }

    /// Infinite thickness (saturation) depth: d_inf ~ 3 / (mu * rho)
    pub fn infinite_thickness_um(mu_total: f64, density_g_cm3: f64) -> f64 {
        if mu_total > 0.0 && density_g_cm3 > 0.0 {
            3.0 / (mu_total * density_g_cm3) * 1e4
        } else {
            0.0
        }
    }

    /// Mass absorption coefficient (simplified, Victoreen): mu/rho ~ C * Z^4 * lambda^3
    pub fn mass_absorption_approx(z: u32, energy_kev: f64) -> f64 {
        if energy_kev > 0.0 {
            let lambda = 12.398 / energy_kev; // Angstroms
            0.02 * (z as f64).powf(4.0) * lambda.powf(3.0) / 1e6
        } else {
            0.0
        }
    }

    /// Moseley's law: E_Ka = 0.01 * (Z - 1)^2 keV (approximate)
    pub fn moseley_ka_kev(z: u32) -> f64 {
        0.01 * ((z as f64) - 1.0).powi(2)
    }

    /// Identify element from peak energy
    pub fn identify_element(energy_kev: f64, tolerance_kev: f64) -> Vec<(String, String)> {
        let db = xrf_element_database();
        let mut matches = Vec::new();
        for elem in &db {
            if (elem.ka_kev - energy_kev).abs() <= tolerance_kev {
                matches.push((elem.symbol.to_string(), "Ka".to_string()));
            }
            if elem.kb_kev > 0.0 && (elem.kb_kev - energy_kev).abs() <= tolerance_kev {
                matches.push((elem.symbol.to_string(), "Kb".to_string()));
            }
            if elem.la_kev > 0.0 && (elem.la_kev - energy_kev).abs() <= tolerance_kev {
                matches.push((elem.symbol.to_string(), "La".to_string()));
            }
        }
        matches
    }

    /// Spectral overlap correction: subtract interfering line
    pub fn overlap_correction(
        gross_intensity: f64,
        interferer_intensity: f64,
        overlap_fraction: f64,
    ) -> f64 {
        (gross_intensity - interferer_intensity * overlap_fraction).max(0.0)
    }
}

#[derive(Clone, Debug)]
pub enum BgMethod {
    Linear,
    Snip,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_spectrum(peaks: &[(f64, f64, f64)], n_channels: usize) -> XrfSpectrum {
        let e_max = 40.0;
        let energy: Vec<f64> = (0..n_channels)
            .map(|i| e_max * i as f64 / (n_channels - 1) as f64)
            .collect();
        let counts: Vec<f64> = energy.iter().map(|&e| {
            let mut val = 50.0; // flat background
            for &(center, height, sigma) in peaks {
                val += height * (-(e - center).powi(2) / (2.0 * sigma * sigma)).exp();
            }
            val
        }).collect();
        XrfSpectrum { energy_kev: energy, counts, live_time_s: 100.0 }
    }

    #[test]
    fn test_default_config() {
        let cfg = XrfConfig::default();
        assert_eq!(cfg.tube_voltage_kv, 50.0);
        assert_eq!(cfg.takeoff_angle_deg, 40.0);
    }

    #[test]
    fn test_xrf_database() {
        let db = xrf_element_database();
        assert!(db.len() >= 15);
        let fe = db.iter().find(|e| e.symbol == "Fe").unwrap();
        assert!((fe.ka_kev - 6.404).abs() < 0.01);
        assert!((fe.fluorescence_yield - 0.335).abs() < 0.01);
    }

    #[test]
    fn test_fundamental_intensity() {
        let i = XrfProcessor::fundamental_intensity(0.1, 0.335, 7.1, 1.0);
        assert!(i > 0.0);
        // Higher concentration = higher intensity
        let i2 = XrfProcessor::fundamental_intensity(0.2, 0.335, 7.1, 1.0);
        assert!(i2 > i);
    }

    #[test]
    fn test_absorption_correction() {
        let f = XrfProcessor::absorption_correction(10.0, 20.0, 60.0, 40.0);
        assert!(f > 0.0);
    }

    #[test]
    fn test_enhancement_factor() {
        let ef = XrfProcessor::enhancement_factor(0.5, 100.0, 0.335, 7.1);
        assert!(ef > 0.0);
    }

    #[test]
    fn test_calibrate_linear() {
        let stds = vec![
            CalibrationStandard { name: "S1".into(), element: "Fe".into(), known_concentration_ppm: 100.0, measured_intensity: 500.0 },
            CalibrationStandard { name: "S2".into(), element: "Fe".into(), known_concentration_ppm: 500.0, measured_intensity: 2500.0 },
            CalibrationStandard { name: "S3".into(), element: "Fe".into(), known_concentration_ppm: 1000.0, measured_intensity: 5000.0 },
        ];
        let (intercept, slope) = XrfProcessor::calibrate_linear(&stds);
        // Should be roughly linear: C ~ 0.2 * I
        assert!(slope > 0.0);
        let predicted = XrfProcessor::apply_calibration(2500.0, intercept, slope);
        assert!((predicted - 500.0).abs() < 50.0, "prediction should be ~500 ppm, got {predicted}");
    }

    #[test]
    fn test_calibrate_single_std() {
        let stds = vec![
            CalibrationStandard { name: "S1".into(), element: "Fe".into(), known_concentration_ppm: 100.0, measured_intensity: 500.0 },
        ];
        let (_, _) = XrfProcessor::calibrate_linear(&stds);
        // Shouldn't panic with single standard
    }

    #[test]
    fn test_net_intensity() {
        let spectrum = make_test_spectrum(&[(6.404, 10000.0, 0.1)], 4001);
        let net = XrfProcessor::net_intensity(&spectrum, 6.404, 0.4, BgMethod::Linear);
        assert!(net > 0.0, "net intensity should be positive");
    }

    #[test]
    fn test_detection_limit() {
        let dl = XrfProcessor::detection_limit_ppm(1000.0, 50000.0, 100.0, 100.0);
        assert!(dl > 0.0 && dl < 100.0, "detection limit should be reasonable, got {dl}");
    }

    #[test]
    fn test_compton_rayleigh() {
        let cr = XrfProcessor::compton_rayleigh_ratio(5000.0, 2000.0);
        assert!((cr - 2.5).abs() < 0.01);
    }

    #[test]
    fn test_compton_rayleigh_zero() {
        assert_eq!(XrfProcessor::compton_rayleigh_ratio(5000.0, 0.0), 0.0);
    }

    #[test]
    fn test_thin_film_thickness() {
        let d = XrfProcessor::thin_film_thickness_um(0.5, 50.0, 5.0);
        assert!(d > 0.0, "thickness should be positive");
    }

    #[test]
    fn test_thin_film_edge_cases() {
        assert_eq!(XrfProcessor::thin_film_thickness_um(0.0, 50.0, 5.0), 0.0);
        assert_eq!(XrfProcessor::thin_film_thickness_um(1.0, 50.0, 5.0), 0.0);
    }

    #[test]
    fn test_infinite_thickness() {
        let d = XrfProcessor::infinite_thickness_um(50.0, 5.0);
        assert!(d > 0.0);
    }

    #[test]
    fn test_mass_absorption() {
        let mu_fe = XrfProcessor::mass_absorption_approx(26, 6.404);
        assert!(mu_fe > 0.0);
        // Higher Z = higher absorption
        let mu_pb = XrfProcessor::mass_absorption_approx(82, 6.404);
        assert!(mu_pb > mu_fe);
    }

    #[test]
    fn test_moseley_law() {
        let e_fe = XrfProcessor::moseley_ka_kev(26);
        assert!(e_fe > 5.0 && e_fe < 8.0, "Fe Ka from Moseley should be ~6-7 keV, got {e_fe}");
    }

    #[test]
    fn test_identify_element() {
        let matches = XrfProcessor::identify_element(6.404, 0.05);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|(e, l)| e == "Fe" && l == "Ka"));
    }

    #[test]
    fn test_identify_element_no_match() {
        let matches = XrfProcessor::identify_element(15.0, 0.01);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_overlap_correction() {
        let corrected = XrfProcessor::overlap_correction(1000.0, 200.0, 0.3);
        assert!((corrected - 940.0).abs() < 0.1);
    }

    #[test]
    fn test_overlap_correction_negative() {
        let corrected = XrfProcessor::overlap_correction(100.0, 500.0, 0.3);
        assert_eq!(corrected, 0.0); // Clamped to 0
    }

    #[test]
    fn test_bg_snip_method() {
        let spectrum = make_test_spectrum(&[(6.404, 10000.0, 0.1)], 4001);
        let net = XrfProcessor::net_intensity(&spectrum, 6.404, 0.4, BgMethod::Snip);
        assert!(net > 0.0);
    }

    #[test]
    fn test_apply_calibration_negative() {
        let c = XrfProcessor::apply_calibration(10.0, -1000.0, 1.0);
        assert_eq!(c, 0.0); // Clamped to 0
    }
}
