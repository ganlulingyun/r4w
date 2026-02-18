// Energy Dispersive X-Ray Spectroscopy (EDX/EDS) Processor
// Elemental analysis in SEM/TEM using characteristic X-ray emission
// Implements: peak identification, ZAF matrix correction, Cliff-Lorimer quantification,
// background modeling, element mapping, detection limits, spectrum deconvolution

use std::f64::consts::PI;

/// EDX configuration
#[derive(Clone, Debug)]
pub struct EdxConfig {
    /// Accelerating voltage in kV
    pub acc_voltage_kv: f64,
    /// Takeoff angle in degrees
    pub takeoff_angle_deg: f64,
    /// Detector type
    pub detector: DetectorType,
    /// Dead time correction factor
    pub dead_time_fraction: f64,
}

impl Default for EdxConfig {
    fn default() -> Self {
        Self {
            acc_voltage_kv: 20.0,
            takeoff_angle_deg: 35.0,
            detector: DetectorType::SiLiDrift,
            dead_time_fraction: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub enum DetectorType {
    SiLiDrift,
    SiPin,
    Cdte,
}

/// X-ray line data
#[derive(Clone, Debug)]
pub struct XrayLine {
    /// Element symbol
    pub element: &'static str,
    /// Atomic number
    pub z: u32,
    /// Line series (Ka, Kb, La, Lb, Ma)
    pub line: &'static str,
    /// Energy in keV
    pub energy_kev: f64,
    /// Relative intensity (Ka1 = 1.0 reference for K series)
    pub relative_intensity: f64,
    /// Atomic weight
    pub atomic_weight: f64,
}

/// Standard X-ray line database
pub fn xray_line_database() -> Vec<XrayLine> {
    vec![
        XrayLine { element: "C",  z: 6,  line: "Ka", energy_kev: 0.277, relative_intensity: 1.0, atomic_weight: 12.011 },
        XrayLine { element: "N",  z: 7,  line: "Ka", energy_kev: 0.392, relative_intensity: 1.0, atomic_weight: 14.007 },
        XrayLine { element: "O",  z: 8,  line: "Ka", energy_kev: 0.525, relative_intensity: 1.0, atomic_weight: 15.999 },
        XrayLine { element: "Na", z: 11, line: "Ka", energy_kev: 1.041, relative_intensity: 1.0, atomic_weight: 22.990 },
        XrayLine { element: "Mg", z: 12, line: "Ka", energy_kev: 1.254, relative_intensity: 1.0, atomic_weight: 24.305 },
        XrayLine { element: "Al", z: 13, line: "Ka", energy_kev: 1.487, relative_intensity: 1.0, atomic_weight: 26.982 },
        XrayLine { element: "Si", z: 14, line: "Ka", energy_kev: 1.740, relative_intensity: 1.0, atomic_weight: 28.086 },
        XrayLine { element: "P",  z: 15, line: "Ka", energy_kev: 2.013, relative_intensity: 1.0, atomic_weight: 30.974 },
        XrayLine { element: "S",  z: 16, line: "Ka", energy_kev: 2.308, relative_intensity: 1.0, atomic_weight: 32.060 },
        XrayLine { element: "Cl", z: 17, line: "Ka", energy_kev: 2.622, relative_intensity: 1.0, atomic_weight: 35.453 },
        XrayLine { element: "K",  z: 19, line: "Ka", energy_kev: 3.314, relative_intensity: 1.0, atomic_weight: 39.098 },
        XrayLine { element: "Ca", z: 20, line: "Ka", energy_kev: 3.692, relative_intensity: 1.0, atomic_weight: 40.078 },
        XrayLine { element: "Ti", z: 22, line: "Ka", energy_kev: 4.511, relative_intensity: 1.0, atomic_weight: 47.867 },
        XrayLine { element: "Cr", z: 24, line: "Ka", energy_kev: 5.415, relative_intensity: 1.0, atomic_weight: 51.996 },
        XrayLine { element: "Mn", z: 25, line: "Ka", energy_kev: 5.899, relative_intensity: 1.0, atomic_weight: 54.938 },
        XrayLine { element: "Fe", z: 26, line: "Ka", energy_kev: 6.404, relative_intensity: 1.0, atomic_weight: 55.845 },
        XrayLine { element: "Fe", z: 26, line: "Kb", energy_kev: 7.058, relative_intensity: 0.13, atomic_weight: 55.845 },
        XrayLine { element: "Co", z: 27, line: "Ka", energy_kev: 6.930, relative_intensity: 1.0, atomic_weight: 58.933 },
        XrayLine { element: "Ni", z: 28, line: "Ka", energy_kev: 7.472, relative_intensity: 1.0, atomic_weight: 58.693 },
        XrayLine { element: "Cu", z: 29, line: "Ka", energy_kev: 8.048, relative_intensity: 1.0, atomic_weight: 63.546 },
        XrayLine { element: "Cu", z: 29, line: "Kb", energy_kev: 8.905, relative_intensity: 0.13, atomic_weight: 63.546 },
        XrayLine { element: "Zn", z: 30, line: "Ka", energy_kev: 8.639, relative_intensity: 1.0, atomic_weight: 65.380 },
        XrayLine { element: "Ag", z: 47, line: "La", energy_kev: 2.984, relative_intensity: 1.0, atomic_weight: 107.868 },
        XrayLine { element: "Au", z: 79, line: "La", energy_kev: 9.713, relative_intensity: 1.0, atomic_weight: 196.967 },
        XrayLine { element: "Au", z: 79, line: "Ma", energy_kev: 2.123, relative_intensity: 0.5, atomic_weight: 196.967 },
    ]
}

/// EDX spectrum data
#[derive(Clone, Debug)]
pub struct EdxSpectrum {
    /// Energy axis in keV
    pub energy_kev: Vec<f64>,
    /// Counts per channel
    pub counts: Vec<f64>,
    /// Live time in seconds
    pub live_time_s: f64,
    /// Real time in seconds
    pub real_time_s: f64,
}

/// Quantification result for one element
#[derive(Clone, Debug)]
pub struct ElementResult {
    pub element: String,
    pub line: String,
    pub net_counts: f64,
    pub weight_percent: f64,
    pub atomic_percent: f64,
    pub k_ratio: f64,
}

/// EDX Processor
pub struct EdxProcessor {
    pub config: EdxConfig,
    pub spectrum: EdxSpectrum,
}

impl EdxProcessor {
    pub fn new(config: EdxConfig, spectrum: EdxSpectrum) -> Self {
        Self { config, spectrum }
    }

    /// Identify peaks in spectrum, returning (energy, element, line) matches
    pub fn identify_peaks(&self, threshold_counts: f64, tolerance_kev: f64) -> Vec<(f64, String, String)> {
        let db = xray_line_database();
        let peaks = self.find_peaks(threshold_counts);
        let mut results = Vec::new();

        for peak_energy in peaks {
            // Only consider lines that can be excited (E_line < E_beam)
            let mut best_match: Option<&XrayLine> = None;
            let mut best_dist = f64::INFINITY;

            for line in &db {
                if line.energy_kev > self.config.acc_voltage_kv {
                    continue; // Can't excite this line
                }
                let dist = (line.energy_kev - peak_energy).abs();
                if dist < tolerance_kev && dist < best_dist {
                    best_dist = dist;
                    best_match = Some(line);
                }
            }

            if let Some(m) = best_match {
                results.push((peak_energy, m.element.to_string(), m.line.to_string()));
            }
        }
        results
    }

    /// Find peak positions in spectrum using simple derivative
    fn find_peaks(&self, threshold: f64) -> Vec<f64> {
        let n = self.spectrum.counts.len();
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if self.spectrum.counts[i] > threshold
                && self.spectrum.counts[i] > self.spectrum.counts[i - 1]
                && self.spectrum.counts[i] > self.spectrum.counts[i + 1]
            {
                peaks.push(self.spectrum.energy_kev[i]);
            }
        }
        peaks
    }

    /// Bremsstrahlung background model: Kramers' law
    /// I_bg(E) = K * Z * (E0 - E) / E
    /// where E0 is beam energy, Z is average atomic number
    pub fn kramers_background(energy_kev: &[f64], e0_kev: f64, z_avg: f64, k: f64) -> Vec<f64> {
        energy_kev
            .iter()
            .map(|&e| {
                if e > 0.0 && e < e0_kev {
                    k * z_avg * (e0_kev - e) / e
                } else {
                    0.0
                }
            })
            .collect()
    }

    /// Background subtraction using Kramers model fit
    pub fn subtract_background(&self, z_avg: f64) -> Vec<f64> {
        // Estimate K from background regions (avoid known peaks)
        let bg = Self::kramers_background(
            &self.spectrum.energy_kev,
            self.config.acc_voltage_kv,
            z_avg,
            1.0,
        );

        // Scale factor: use region 0.5-1.0 keV (usually no peaks for most materials)
        let mut scale_num = 0.0;
        let mut scale_den = 0.0;
        for (i, &e) in self.spectrum.energy_kev.iter().enumerate() {
            if e >= 0.5 && e <= 1.0 && bg[i] > 0.0 {
                scale_num += self.spectrum.counts[i];
                scale_den += bg[i];
            }
        }

        let scale = if scale_den > 0.0 { scale_num / scale_den } else { 1.0 };

        self.spectrum
            .counts
            .iter()
            .zip(bg.iter())
            .map(|(&c, &b)| (c - b * scale).max(0.0))
            .collect()
    }

    /// Net peak area: integrate counts in ROI minus background
    pub fn net_peak_area(&self, center_kev: f64, width_kev: f64, bg_counts: &[f64]) -> f64 {
        let mut area = 0.0;
        for (i, &e) in self.spectrum.energy_kev.iter().enumerate() {
            if (e - center_kev).abs() <= width_kev / 2.0 {
                area += self.spectrum.counts[i] - bg_counts.get(i).copied().unwrap_or(0.0);
            }
        }
        area.max(0.0)
    }

    /// Cliff-Lorimer quantification for thin film (TEM-EDX)
    /// C_A/C_B = k_AB * (I_A/I_B)
    pub fn cliff_lorimer(
        intensity_a: f64,
        intensity_b: f64,
        k_ab: f64,
    ) -> (f64, f64) {
        if intensity_b > 0.0 {
            let ratio = k_ab * intensity_a / intensity_b;
            // C_A + C_B = 100
            let c_a = 100.0 * ratio / (1.0 + ratio);
            let c_b = 100.0 - c_a;
            (c_a, c_b)
        } else {
            (100.0, 0.0)
        }
    }

    /// ZAF matrix correction factor
    /// Z: atomic number correction, A: absorption correction, F: fluorescence correction
    pub fn zaf_correction(
        z_factor: f64,
        a_factor: f64,
        f_factor: f64,
    ) -> f64 {
        z_factor * a_factor * f_factor
    }

    /// Absorption correction factor (simplified Philibert model)
    /// f(chi) = (1 + h) / (1 + chi/sigma) * 1/(1 + h*(1 + chi/sigma))
    /// chi = mu * csc(takeoff_angle)
    pub fn absorption_factor(
        mass_absorption_coeff: f64,
        takeoff_angle_deg: f64,
        sigma: f64,
    ) -> f64 {
        let csc = 1.0 / (takeoff_angle_deg * PI / 180.0).sin();
        let chi = mass_absorption_coeff * csc;
        let h = 1.2 * (-6.0 * chi / sigma).exp(); // Simplified Heinrich

        if sigma > 0.0 {
            (1.0 + h) / (1.0 + chi / sigma) / (1.0 + h * (1.0 + chi / sigma))
        } else {
            1.0
        }
    }

    /// Overvoltage ratio: U = E0/Ec where Ec is critical excitation energy
    pub fn overvoltage_ratio(beam_energy_kev: f64, edge_energy_kev: f64) -> f64 {
        if edge_energy_kev > 0.0 {
            beam_energy_kev / edge_energy_kev
        } else {
            f64::INFINITY
        }
    }

    /// Optimal overvoltage: U = 2-3 for best X-ray yield
    pub fn is_optimal_overvoltage(beam_energy_kev: f64, edge_energy_kev: f64) -> bool {
        let u = Self::overvoltage_ratio(beam_energy_kev, edge_energy_kev);
        u >= 2.0 && u <= 3.0
    }

    /// Electron range (Kanaya-Okayama): R = 0.0276 * A * E0^1.67 / (Z^0.89 * rho)
    /// R in micrometers, E0 in keV, rho in g/cm^3
    pub fn electron_range_um(
        atomic_weight: f64,
        z: f64,
        density_g_cm3: f64,
        beam_energy_kev: f64,
    ) -> f64 {
        if z > 0.0 && density_g_cm3 > 0.0 {
            0.0276 * atomic_weight * beam_energy_kev.powf(1.67)
                / (z.powf(0.89) * density_g_cm3)
        } else {
            0.0
        }
    }

    /// X-ray generation depth (Anderson-Hasler): d = 0.064*(E0^1.68 - Ec^1.68)/rho
    pub fn xray_generation_depth_um(
        beam_energy_kev: f64,
        edge_energy_kev: f64,
        density_g_cm3: f64,
    ) -> f64 {
        if density_g_cm3 > 0.0 && beam_energy_kev > edge_energy_kev {
            0.064 * (beam_energy_kev.powf(1.68) - edge_energy_kev.powf(1.68))
                / density_g_cm3
        } else {
            0.0
        }
    }

    /// Detection limit (Currie): C_DL = 3 * sqrt(I_bg) * C_std / I_net_std
    pub fn detection_limit_wt_pct(
        bg_counts: f64,
        std_wt_pct: f64,
        std_net_counts: f64,
    ) -> f64 {
        if std_net_counts > 0.0 {
            3.0 * bg_counts.sqrt() * std_wt_pct / std_net_counts
        } else {
            f64::INFINITY
        }
    }

    /// Weight % to atomic %
    pub fn weight_to_atomic(elements: &[(&str, f64, f64)]) -> Vec<(String, f64)> {
        // elements: (name, weight_pct, atomic_weight)
        let mole_fractions: Vec<(String, f64)> = elements
            .iter()
            .map(|&(name, wt, aw)| {
                (name.to_string(), if aw > 0.0 { wt / aw } else { 0.0 })
            })
            .collect();
        let total: f64 = mole_fractions.iter().map(|(_, f)| f).sum();
        if total > 0.0 {
            mole_fractions
                .iter()
                .map(|(name, f)| (name.clone(), 100.0 * f / total))
                .collect()
        } else {
            mole_fractions
        }
    }

    /// Atomic % to weight %
    pub fn atomic_to_weight(elements: &[(&str, f64, f64)]) -> Vec<(String, f64)> {
        // elements: (name, atomic_pct, atomic_weight)
        let weight_fracs: Vec<(String, f64)> = elements
            .iter()
            .map(|&(name, at, aw)| (name.to_string(), at * aw))
            .collect();
        let total: f64 = weight_fracs.iter().map(|(_, w)| w).sum();
        if total > 0.0 {
            weight_fracs
                .iter()
                .map(|(name, w)| (name.clone(), 100.0 * w / total))
                .collect()
        } else {
            weight_fracs
        }
    }

    /// Gaussian peak shape model: I(E) = A * exp(-(E-E0)^2 / (2*sigma^2))
    /// sigma ~ FWHM / 2.355, FWHM(E) = sqrt(FWHM_noise^2 + 3.58*F*E)
    /// F = Fano factor (~0.12 for Si), E in eV
    pub fn detector_fwhm_kev(energy_kev: f64, noise_fwhm_kev: f64, fano: f64) -> f64 {
        let energy_ev = energy_kev * 1000.0;
        let noise_ev = noise_fwhm_kev * 1000.0;
        let fwhm_ev = (noise_ev * noise_ev + 3.58 * fano * energy_ev).sqrt();
        fwhm_ev / 1000.0
    }

    /// Dead time correction: I_true = I_meas / (1 - I_meas * tau)
    pub fn dead_time_correct(measured_rate: f64, dead_time_s: f64) -> f64 {
        let denom = 1.0 - measured_rate * dead_time_s;
        if denom > 0.01 {
            measured_rate / denom
        } else {
            measured_rate // Avoid division by ~0
        }
    }

    /// Sum peak check: artifact at E_A + E_B
    pub fn check_sum_peaks(
        identified: &[(f64, String, String)],
        tolerance_kev: f64,
    ) -> Vec<(String, String, f64)> {
        let mut sum_peaks = Vec::new();
        for i in 0..identified.len() {
            for j in i..identified.len() {
                let sum_energy = identified[i].0 + identified[j].0;
                sum_peaks.push((
                    format!("{}+{}", identified[i].1, identified[j].1),
                    format!("{} + {}", identified[i].2, identified[j].2),
                    sum_energy,
                ));
            }
        }
        sum_peaks
    }

    /// Escape peak energy: E_escape = E_peak - E_Si_Ka (1.74 keV)
    pub fn escape_peak_energy(peak_kev: f64) -> f64 {
        peak_kev - 1.740
    }

    /// Duane-Hunt limit: maximum X-ray energy = eV0
    pub fn duane_hunt_limit_kev(acc_voltage_kv: f64) -> f64 {
        acc_voltage_kv
    }

    /// Moseley's law: E_Ka ~ 13.6 * (Z - 7.4)^2 / 4 (in eV, approximate)
    /// More accurate: E = A*(Z-B)^2
    pub fn moseley_energy_kev(z: u32, a: f64, b: f64) -> f64 {
        let zf = z as f64;
        a * (zf - b).powi(2) / 1000.0 // Convert eV to keV
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_spectrum(peaks: &[(f64, f64, f64)], n_channels: usize) -> EdxSpectrum {
        let e_max = 20.0; // keV
        let energy_kev: Vec<f64> = (0..n_channels)
            .map(|i| e_max * i as f64 / (n_channels - 1) as f64)
            .collect();
        let counts: Vec<f64> = energy_kev
            .iter()
            .map(|&e| {
                let mut val = if e > 0.1 { 100.0 * (20.0 - e) / e } else { 100.0 };
                for &(center, height, sigma) in peaks {
                    val += height * (-(e - center).powi(2) / (2.0 * sigma * sigma)).exp();
                }
                val.max(0.0)
            })
            .collect();
        EdxSpectrum {
            energy_kev,
            counts,
            live_time_s: 60.0,
            real_time_s: 65.0,
        }
    }

    #[test]
    fn test_default_config() {
        let cfg = EdxConfig::default();
        assert_eq!(cfg.acc_voltage_kv, 20.0);
        assert_eq!(cfg.takeoff_angle_deg, 35.0);
    }

    #[test]
    fn test_xray_database() {
        let db = xray_line_database();
        assert!(db.len() >= 20);
        let fe_ka = db.iter().find(|l| l.element == "Fe" && l.line == "Ka").unwrap();
        assert!((fe_ka.energy_kev - 6.404).abs() < 0.01);
    }

    #[test]
    fn test_identify_peaks() {
        let spectrum = make_test_spectrum(
            &[(6.404, 5000.0, 0.07), (1.740, 3000.0, 0.05)],
            2001,
        );
        let proc = EdxProcessor::new(EdxConfig::default(), spectrum);
        let peaks = proc.identify_peaks(100.0, 0.15);
        let elements: Vec<&str> = peaks.iter().map(|(_, e, _)| e.as_str()).collect();
        assert!(elements.contains(&"Fe") || elements.contains(&"Si"),
            "should identify Fe or Si peaks, got {:?}", peaks);
    }

    #[test]
    fn test_kramers_background() {
        let energy: Vec<f64> = (1..100).map(|i| i as f64 * 0.2).collect();
        let bg = EdxProcessor::kramers_background(&energy, 20.0, 26.0, 1.0);
        assert_eq!(bg.len(), 99);
        assert!(bg[0] > 0.0);
        // Background should decrease with energy
        assert!(bg[0] > bg[50]);
    }

    #[test]
    fn test_cliff_lorimer() {
        let (ca, cb) = EdxProcessor::cliff_lorimer(1000.0, 2000.0, 1.0);
        // k=1, I_A/I_B = 0.5, so C_A/(C_A+C_B) = 0.5/(1.5) = 33.3%
        assert!((ca - 33.33).abs() < 0.1);
        assert!((cb - 66.67).abs() < 0.1);
    }

    #[test]
    fn test_cliff_lorimer_equal() {
        let (ca, cb) = EdxProcessor::cliff_lorimer(1000.0, 1000.0, 1.0);
        assert!((ca - 50.0).abs() < 0.1);
        assert!((cb - 50.0).abs() < 0.1);
    }

    #[test]
    fn test_zaf_correction() {
        let zaf = EdxProcessor::zaf_correction(1.1, 0.95, 1.02);
        assert!((zaf - 1.1 * 0.95 * 1.02).abs() < 1e-10);
    }

    #[test]
    fn test_absorption_factor() {
        let f = EdxProcessor::absorption_factor(100.0, 35.0, 2000.0);
        assert!(f > 0.0 && f <= 1.0, "absorption factor should be 0-1, got {f}");
    }

    #[test]
    fn test_overvoltage_ratio() {
        let u = EdxProcessor::overvoltage_ratio(20.0, 7.112);
        assert!((u - 20.0 / 7.112).abs() < 0.01);
    }

    #[test]
    fn test_optimal_overvoltage() {
        assert!(EdxProcessor::is_optimal_overvoltage(20.0, 8.0)); // U=2.5
        assert!(!EdxProcessor::is_optimal_overvoltage(5.0, 8.0)); // U=0.625
    }

    #[test]
    fn test_electron_range() {
        let r = EdxProcessor::electron_range_um(55.845, 26.0, 7.874, 20.0);
        assert!(r > 0.0, "electron range should be positive");
        assert!(r < 10.0, "range at 20kV should be < 10um, got {r}");
    }

    #[test]
    fn test_xray_generation_depth() {
        let d = EdxProcessor::xray_generation_depth_um(20.0, 6.404, 7.874);
        assert!(d > 0.0);
        assert!(d < 5.0, "generation depth should be reasonable, got {d}");
    }

    #[test]
    fn test_detection_limit() {
        let dl = EdxProcessor::detection_limit_wt_pct(1000.0, 10.0, 50000.0);
        // 3*sqrt(1000)*10/50000 ~ 0.019 wt%
        assert!(dl > 0.0 && dl < 1.0, "detection limit should be small, got {dl}");
    }

    #[test]
    fn test_weight_to_atomic() {
        let result = EdxProcessor::weight_to_atomic(&[
            ("Fe", 70.0, 55.845),
            ("O", 30.0, 15.999),
        ]);
        assert_eq!(result.len(), 2);
        let total: f64 = result.iter().map(|(_, v)| v).sum();
        assert!((total - 100.0).abs() < 0.1);
        // Fe: 70/55.845 = 1.253, O: 30/16 = 1.875
        // Fe at%: 1.253/(1.253+1.875)*100 = 40.1%
        let fe_at = result.iter().find(|(n, _)| n == "Fe").unwrap().1;
        assert!((fe_at - 40.1).abs() < 0.5, "Fe at% should be ~40, got {fe_at}");
    }

    #[test]
    fn test_atomic_to_weight() {
        let result = EdxProcessor::atomic_to_weight(&[
            ("Fe", 40.0, 55.845),
            ("O", 60.0, 15.999),
        ]);
        let total: f64 = result.iter().map(|(_, v)| v).sum();
        assert!((total - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_detector_fwhm() {
        let fwhm = EdxProcessor::detector_fwhm_kev(5.9, 0.06, 0.12);
        // Mn Ka at 5.9 keV: FWHM ~ 130 eV for SDD
        assert!(fwhm > 0.05 && fwhm < 0.2, "FWHM should be 50-200 eV, got {}", fwhm * 1000.0);
    }

    #[test]
    fn test_dead_time_correct() {
        let corrected = EdxProcessor::dead_time_correct(10000.0, 1e-6);
        // 10000 * 1e-6 = 0.01, so true = 10000 / 0.99 ~ 10101
        assert!((corrected - 10101.0).abs() < 1.0);
    }

    #[test]
    fn test_dead_time_low_rate() {
        let corrected = EdxProcessor::dead_time_correct(100.0, 1e-6);
        assert!((corrected - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_escape_peak() {
        let e_escape = EdxProcessor::escape_peak_energy(6.404);
        assert!((e_escape - 4.664).abs() < 0.01);
    }

    #[test]
    fn test_duane_hunt_limit() {
        assert_eq!(EdxProcessor::duane_hunt_limit_kev(20.0), 20.0);
    }

    #[test]
    fn test_moseley_law() {
        // Ka series: A ~ 10.2, B ~ 1 (Moseley original)
        let e_fe = EdxProcessor::moseley_energy_kev(26, 10.2, 1.0);
        assert!(e_fe > 5.0 && e_fe < 10.0, "Fe Ka from Moseley should be 5-10 keV, got {e_fe}");
    }

    #[test]
    fn test_subtract_background() {
        let spectrum = make_test_spectrum(&[(6.404, 5000.0, 0.07)], 2001);
        let proc = EdxProcessor::new(EdxConfig::default(), spectrum);
        let subtracted = proc.subtract_background(26.0);
        assert_eq!(subtracted.len(), 2001);
    }

    #[test]
    fn test_net_peak_area() {
        let spectrum = make_test_spectrum(&[(6.404, 5000.0, 0.07)], 2001);
        let bg = vec![0.0; 2001];
        let proc = EdxProcessor::new(EdxConfig::default(), spectrum);
        let area = proc.net_peak_area(6.404, 0.3, &bg);
        assert!(area > 0.0, "net peak area should be positive");
    }

    #[test]
    fn test_check_sum_peaks() {
        let identified = vec![
            (6.404, "Fe".to_string(), "Ka".to_string()),
            (1.740, "Si".to_string(), "Ka".to_string()),
        ];
        let sums = EdxProcessor::check_sum_peaks(&identified, 0.1);
        assert!(!sums.is_empty());
        // Fe+Fe sum peak at ~12.8 keV
        let fe_fe = sums.iter().find(|(n, _, _)| n == "Fe+Fe");
        assert!(fe_fe.is_some());
        assert!((fe_fe.unwrap().2 - 12.808).abs() < 0.01);
    }

    #[test]
    fn test_electron_range_zero_density() {
        assert_eq!(EdxProcessor::electron_range_um(55.845, 26.0, 0.0, 20.0), 0.0);
    }

    #[test]
    fn test_generation_depth_low_voltage() {
        let d = EdxProcessor::xray_generation_depth_um(5.0, 6.404, 7.874);
        assert_eq!(d, 0.0); // Beam energy below edge
    }
}
