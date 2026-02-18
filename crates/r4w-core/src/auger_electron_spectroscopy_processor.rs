// Auger Electron Spectroscopy (AES) Processor
// Surface-sensitive elemental analysis using Auger electron transitions
// Implements: derivative spectra, quantification via sensitivity factors,
// Auger parameter, sputter depth profiling, chemical state identification

/// AES configuration
#[derive(Clone, Debug)]
pub struct AesConfig {
    /// Primary beam energy in eV
    pub beam_energy_ev: f64,
    /// Modulation voltage for derivative spectra in eV
    pub modulation_v: f64,
    /// Electron inelastic mean free path in nm
    pub imfp_nm: f64,
}

impl Default for AesConfig {
    fn default() -> Self {
        Self {
            beam_energy_ev: 5000.0,
            modulation_v: 3.0,
            imfp_nm: 1.0,
        }
    }
}

/// Auger transition data
#[derive(Clone, Debug)]
pub struct AugerTransition {
    /// Element symbol
    pub element: &'static str,
    /// Transition label (e.g., "KLL", "LMM")
    pub transition: &'static str,
    /// Kinetic energy in eV
    pub kinetic_energy_ev: f64,
    /// Relative sensitivity factor
    pub sensitivity_factor: f64,
}

/// Standard Auger transition database
pub fn auger_database() -> Vec<AugerTransition> {
    vec![
        AugerTransition { element: "C",  transition: "KLL", kinetic_energy_ev: 272.0,  sensitivity_factor: 0.14 },
        AugerTransition { element: "N",  transition: "KLL", kinetic_energy_ev: 379.0,  sensitivity_factor: 0.28 },
        AugerTransition { element: "O",  transition: "KLL", kinetic_energy_ev: 510.0,  sensitivity_factor: 0.40 },
        AugerTransition { element: "Si", transition: "LMM", kinetic_energy_ev: 92.0,   sensitivity_factor: 0.28 },
        AugerTransition { element: "Ti", transition: "LMM", kinetic_energy_ev: 418.0,  sensitivity_factor: 0.42 },
        AugerTransition { element: "Fe", transition: "LMM", kinetic_energy_ev: 703.0,  sensitivity_factor: 0.21 },
        AugerTransition { element: "Cu", transition: "LMM", kinetic_energy_ev: 920.0,  sensitivity_factor: 0.22 },
        AugerTransition { element: "Al", transition: "KLL", kinetic_energy_ev: 1396.0, sensitivity_factor: 0.07 },
        AugerTransition { element: "Zn", transition: "LMM", kinetic_energy_ev: 994.0,  sensitivity_factor: 0.25 },
        AugerTransition { element: "Cr", transition: "LMM", kinetic_energy_ev: 529.0,  sensitivity_factor: 0.34 },
        AugerTransition { element: "Ni", transition: "LMM", kinetic_energy_ev: 848.0,  sensitivity_factor: 0.27 },
        AugerTransition { element: "Ag", transition: "MNN", kinetic_energy_ev: 351.0,  sensitivity_factor: 0.35 },
    ]
}

/// AES spectrum data
#[derive(Clone, Debug)]
pub struct AesSpectrum {
    /// Kinetic energy axis in eV
    pub energy_ev: Vec<f64>,
    /// Counts (N(E) direct spectrum)
    pub counts: Vec<f64>,
}

/// Depth profile point
#[derive(Clone, Debug)]
pub struct DepthProfilePoint {
    /// Sputter depth in nm
    pub depth_nm: f64,
    /// Elemental concentrations (atomic %)
    pub concentrations: Vec<(String, f64)>,
}

/// AES Processor
pub struct AesProcessor {
    pub config: AesConfig,
    pub spectrum: AesSpectrum,
}

impl AesProcessor {
    pub fn new(config: AesConfig, spectrum: AesSpectrum) -> Self {
        Self { config, spectrum }
    }

    /// Compute numerical derivative dN/dE (lock-in amplifier simulation)
    /// Uses central difference: dN/dE[i] = (N[i+1] - N[i-1]) / (E[i+1] - E[i-1])
    pub fn derivative_spectrum(&self) -> Vec<f64> {
        let n = self.spectrum.counts.len();
        if n < 3 {
            return vec![0.0; n];
        }
        let mut deriv = vec![0.0; n];
        for i in 1..n - 1 {
            let de = self.spectrum.energy_ev[i + 1] - self.spectrum.energy_ev[i - 1];
            if de.abs() > 1e-15 {
                deriv[i] = (self.spectrum.counts[i + 1] - self.spectrum.counts[i - 1]) / de;
            }
        }
        // Forward/backward difference at endpoints
        if n >= 2 {
            let de0 = self.spectrum.energy_ev[1] - self.spectrum.energy_ev[0];
            if de0.abs() > 1e-15 {
                deriv[0] = (self.spectrum.counts[1] - self.spectrum.counts[0]) / de0;
            }
            let den = self.spectrum.energy_ev[n - 1] - self.spectrum.energy_ev[n - 2];
            if den.abs() > 1e-15 {
                deriv[n - 1] = (self.spectrum.counts[n - 1] - self.spectrum.counts[n - 2]) / den;
            }
        }
        deriv
    }

    /// Savitzky-Golay 5-point smoothing for derivative spectra
    pub fn smooth_5pt(data: &[f64]) -> Vec<f64> {
        let n = data.len();
        if n < 5 {
            return data.to_vec();
        }
        let mut out = vec![0.0; n];
        // Coefficients for 5-point SG (quadratic, 0th derivative): [-3, 12, 17, 12, -3]/35
        for i in 2..n - 2 {
            out[i] = (-3.0 * data[i - 2] + 12.0 * data[i - 1] + 17.0 * data[i]
                + 12.0 * data[i + 1] - 3.0 * data[i + 2])
                / 35.0;
        }
        out[0] = data[0];
        out[1] = data[1];
        if n >= 2 {
            out[n - 1] = data[n - 1];
            out[n - 2] = data[n - 2];
        }
        out
    }

    /// Peak-to-peak height in derivative spectrum for given energy range
    pub fn peak_to_peak(&self, center_ev: f64, window_ev: f64) -> f64 {
        let deriv = self.derivative_spectrum();
        let mut max_val = f64::NEG_INFINITY;
        let mut min_val = f64::INFINITY;
        for (i, &e) in self.spectrum.energy_ev.iter().enumerate() {
            if (e - center_ev).abs() <= window_ev / 2.0 {
                if deriv[i] > max_val {
                    max_val = deriv[i];
                }
                if deriv[i] < min_val {
                    min_val = deriv[i];
                }
            }
        }
        if max_val > f64::NEG_INFINITY && min_val < f64::INFINITY {
            max_val - min_val
        } else {
            0.0
        }
    }

    /// Quantification using relative sensitivity factors
    /// Returns atomic % for each element
    pub fn quantify(&self, elements: &[(&str, f64, f64)]) -> Vec<(String, f64)> {
        // elements: (name, center_energy_ev, window_ev)
        let db = auger_database();
        let mut raw: Vec<(String, f64)> = Vec::new();
        let mut total = 0.0;

        for &(name, center, window) in elements {
            let ptp = self.peak_to_peak(center, window);
            // Find sensitivity factor
            let sf = db
                .iter()
                .find(|t| t.element == name)
                .map(|t| t.sensitivity_factor)
                .unwrap_or(1.0);
            let conc = if sf > 1e-15 { ptp / sf } else { 0.0 };
            raw.push((name.to_string(), conc));
            total += conc;
        }

        if total > 1e-15 {
            raw.iter()
                .map(|(name, c)| (name.clone(), 100.0 * c / total))
                .collect()
        } else {
            raw.iter().map(|(name, _)| (name.clone(), 0.0)).collect()
        }
    }

    /// Auger parameter: alpha' = E_kin(Auger) + E_b(XPS)
    /// Modified Auger parameter for chemical state identification
    pub fn auger_parameter(kinetic_energy_auger_ev: f64, binding_energy_xps_ev: f64) -> f64 {
        kinetic_energy_auger_ev + binding_energy_xps_ev
    }

    /// Estimate sampling depth (95% of signal): d = 3 * lambda * cos(theta)
    /// lambda: IMFP in nm, theta: emission angle in degrees
    pub fn sampling_depth_nm(&self, theta_deg: f64) -> f64 {
        3.0 * self.config.imfp_nm * (theta_deg * std::f64::consts::PI / 180.0).cos()
    }

    /// Backscatter factor correction
    /// r(E_p, Z) = 1 + 2.8 * (1 - 0.9 * E_p/Z_eff) * (Z_eff/100)^0.5
    /// Simplified empirical formula
    pub fn backscatter_factor(beam_energy_ev: f64, atomic_number: f64) -> f64 {
        let e_kev = beam_energy_ev / 1000.0;
        let z_eff = atomic_number;
        1.0 + 2.8 * (1.0 - 0.9 * (e_kev / z_eff).min(1.0)) * (z_eff / 100.0).sqrt()
    }

    /// Sputter rate estimation from ion beam parameters
    /// Y = sputter yield (atoms/ion), I = beam current (A), rho = density (atoms/cm^3), A = area (cm^2)
    /// Rate (nm/s) = Y * I / (e * rho * A) * 1e7
    pub fn sputter_rate_nm_per_s(
        yield_atoms_per_ion: f64,
        beam_current_a: f64,
        density_atoms_cm3: f64,
        area_cm2: f64,
    ) -> f64 {
        let e = 1.602e-19;
        if density_atoms_cm3 > 0.0 && area_cm2 > 0.0 {
            yield_atoms_per_ion * beam_current_a / (e * density_atoms_cm3 * area_cm2) * 1e7
        } else {
            0.0
        }
    }

    /// Process depth profile: convert sputter time to depth
    pub fn depth_profile(
        time_s: &[f64],
        spectra: &[Vec<(String, f64)>],
        sputter_rate: f64,
    ) -> Vec<DepthProfilePoint> {
        time_s
            .iter()
            .zip(spectra.iter())
            .map(|(&t, conc)| DepthProfilePoint {
                depth_nm: t * sputter_rate,
                concentrations: conc.clone(),
            })
            .collect()
    }

    /// Find interface location in depth profile (50% concentration crossing)
    pub fn interface_depth(profile: &[DepthProfilePoint], element: &str, threshold_pct: f64) -> Option<f64> {
        for i in 1..profile.len() {
            let c_prev = profile[i - 1]
                .concentrations
                .iter()
                .find(|(e, _)| e == element)
                .map(|(_, c)| *c)
                .unwrap_or(0.0);
            let c_curr = profile[i]
                .concentrations
                .iter()
                .find(|(e, _)| e == element)
                .map(|(_, c)| *c)
                .unwrap_or(0.0);

            if (c_prev >= threshold_pct && c_curr < threshold_pct)
                || (c_prev < threshold_pct && c_curr >= threshold_pct)
            {
                // Linear interpolation
                let frac = if (c_curr - c_prev).abs() > 1e-15 {
                    (threshold_pct - c_prev) / (c_curr - c_prev)
                } else {
                    0.5
                };
                let depth = profile[i - 1].depth_nm
                    + frac * (profile[i].depth_nm - profile[i - 1].depth_nm);
                return Some(depth);
            }
        }
        None
    }

    /// Auger electron energy from energy levels: E_Auger = E_X - E_Y - E_Z - phi
    /// where X, Y, Z are the three levels involved and phi is work function
    pub fn auger_energy(e_x_ev: f64, e_y_ev: f64, e_z_ev: f64, work_function_ev: f64) -> f64 {
        e_x_ev - e_y_ev - e_z_ev - work_function_ev
    }

    /// Estimate IMFP using TPP-2M formula (simplified)
    /// lambda (nm) = E / (E_p^2 * [beta * ln(gamma * E) - (C/E) + (D/E^2)])
    /// Simplified: lambda ~ 0.054 * sqrt(E) for most elements at >100 eV
    pub fn estimate_imfp_nm(kinetic_energy_ev: f64) -> f64 {
        if kinetic_energy_ev > 0.0 {
            0.054 * kinetic_energy_ev.sqrt()
        } else {
            0.0
        }
    }

    /// Chemical shift: difference between measured and reference Auger energy
    pub fn chemical_shift(measured_ev: f64, reference_ev: f64) -> f64 {
        measured_ev - reference_ev
    }

    /// Peak identification: find closest match in database
    pub fn identify_peak(energy_ev: f64, tolerance_ev: f64) -> Vec<AugerTransition> {
        let db = auger_database();
        db.into_iter()
            .filter(|t| (t.kinetic_energy_ev - energy_ev).abs() <= tolerance_ev)
            .collect()
    }

    /// Background subtraction using linear fit
    pub fn subtract_linear_background(
        energy: &[f64],
        counts: &[f64],
        bg_start_ev: f64,
        bg_end_ev: f64,
    ) -> Vec<f64> {
        // Find indices for background regions
        let mut bg_points: Vec<(f64, f64)> = Vec::new();
        for (i, &e) in energy.iter().enumerate() {
            if (e - bg_start_ev).abs() < 5.0 || (e - bg_end_ev).abs() < 5.0 {
                bg_points.push((e, counts[i]));
            }
        }
        if bg_points.len() < 2 {
            return counts.to_vec();
        }

        // Linear fit
        let n = bg_points.len() as f64;
        let sx: f64 = bg_points.iter().map(|(x, _)| x).sum();
        let sy: f64 = bg_points.iter().map(|(_, y)| y).sum();
        let sxx: f64 = bg_points.iter().map(|(x, _)| x * x).sum();
        let sxy: f64 = bg_points.iter().map(|(x, y)| x * y).sum();

        let det = n * sxx - sx * sx;
        if det.abs() < 1e-15 {
            return counts.to_vec();
        }

        let slope = (n * sxy - sx * sy) / det;
        let intercept = (sy - slope * sx) / n;

        energy
            .iter()
            .zip(counts.iter())
            .map(|(&e, &c)| c - (slope * e + intercept))
            .collect()
    }

    /// Matrix correction factor for binary alloy AB
    /// F_A = (1 + r_A * I_A/S_A) / (I_A/S_A + I_B/S_B)
    pub fn matrix_factor(
        intensity_a: f64,
        sf_a: f64,
        intensity_b: f64,
        sf_b: f64,
        backscatter_a: f64,
    ) -> f64 {
        let norm_a = if sf_a > 1e-15 { intensity_a / sf_a } else { 0.0 };
        let norm_b = if sf_b > 1e-15 { intensity_b / sf_b } else { 0.0 };
        let denom = norm_a + norm_b;
        if denom > 1e-15 {
            (1.0 + backscatter_a * norm_a) / denom
        } else {
            0.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_test_spectrum(peaks: &[(f64, f64, f64)], n_points: usize) -> AesSpectrum {
        let e_min = 50.0;
        let e_max = 1100.0;
        let energy_ev: Vec<f64> = (0..n_points)
            .map(|i| e_min + (e_max - e_min) * i as f64 / (n_points - 1) as f64)
            .collect();
        let counts: Vec<f64> = energy_ev
            .iter()
            .map(|&e| {
                let mut val = 1000.0; // background
                for &(center, height, width) in peaks {
                    val += height * (-(e - center).powi(2) / (2.0 * width * width)).exp();
                }
                val
            })
            .collect();
        AesSpectrum { energy_ev, counts }
    }

    #[test]
    fn test_default_config() {
        let cfg = AesConfig::default();
        assert_eq!(cfg.beam_energy_ev, 5000.0);
        assert_eq!(cfg.modulation_v, 3.0);
        assert_eq!(cfg.imfp_nm, 1.0);
    }

    #[test]
    fn test_auger_database() {
        let db = auger_database();
        assert!(db.len() >= 10);
        let c = db.iter().find(|t| t.element == "C").unwrap();
        assert!((c.kinetic_energy_ev - 272.0).abs() < 1.0);
        assert!((c.sensitivity_factor - 0.14).abs() < 0.01);
    }

    #[test]
    fn test_derivative_spectrum() {
        let spectrum = make_test_spectrum(&[(510.0, 5000.0, 10.0)], 501);
        let proc = AesProcessor::new(AesConfig::default(), spectrum);
        let deriv = proc.derivative_spectrum();
        assert_eq!(deriv.len(), 501);
        // Derivative should cross zero near peak center
        let center_idx = 501 * (510 - 50) / (1100 - 50);
        // Check that derivative changes sign near center
        let near_center: Vec<f64> = deriv[center_idx - 5..center_idx + 5].to_vec();
        let has_pos = near_center.iter().any(|&v| v > 0.0);
        let has_neg = near_center.iter().any(|&v| v < 0.0);
        assert!(has_pos && has_neg);
    }

    #[test]
    fn test_smooth_5pt() {
        let data = vec![1.0, 3.0, 2.0, 4.0, 3.0, 5.0, 4.0];
        let smoothed = AesProcessor::smooth_5pt(&data);
        assert_eq!(smoothed.len(), 7);
        // Interior points should be smoothed: SG formula at i=3
        // data = [1, 3, 2, 4, 3, 5, 4], so smoothed[3] = (-3*3 + 12*2 + 17*4 + 12*3 - 3*5)/35
        let expected = (-3.0 * 3.0 + 12.0 * 2.0 + 17.0 * 4.0 + 12.0 * 3.0 - 3.0 * 5.0) / 35.0;
        assert!((smoothed[3] - expected).abs() < 0.01);
    }

    #[test]
    fn test_smooth_short() {
        let data = vec![1.0, 2.0, 3.0];
        let smoothed = AesProcessor::smooth_5pt(&data);
        assert_eq!(smoothed, data);
    }

    #[test]
    fn test_peak_to_peak() {
        let spectrum = make_test_spectrum(&[(510.0, 5000.0, 10.0)], 1001);
        let proc = AesProcessor::new(AesConfig::default(), spectrum);
        let ptp = proc.peak_to_peak(510.0, 60.0);
        assert!(ptp > 0.0, "peak-to-peak should be positive");
    }

    #[test]
    fn test_quantify() {
        let spectrum = make_test_spectrum(
            &[(272.0, 1000.0, 8.0), (510.0, 2000.0, 8.0)],
            1001,
        );
        let proc = AesProcessor::new(AesConfig::default(), spectrum);
        let result = proc.quantify(&[("C", 272.0, 40.0), ("O", 510.0, 40.0)]);
        assert_eq!(result.len(), 2);
        let total: f64 = result.iter().map(|(_, c)| c).sum();
        assert!((total - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_auger_parameter() {
        let ap = AesProcessor::auger_parameter(920.0, 932.0);
        assert!((ap - 1852.0).abs() < 0.1);
    }

    #[test]
    fn test_sampling_depth() {
        let proc = AesProcessor::new(
            AesConfig { imfp_nm: 1.5, ..Default::default() },
            AesSpectrum { energy_ev: vec![], counts: vec![] },
        );
        let d = proc.sampling_depth_nm(0.0);
        assert!((d - 4.5).abs() < 0.01); // 3 * 1.5 * cos(0)
    }

    #[test]
    fn test_sampling_depth_angle() {
        let proc = AesProcessor::new(
            AesConfig { imfp_nm: 1.0, ..Default::default() },
            AesSpectrum { energy_ev: vec![], counts: vec![] },
        );
        let d = proc.sampling_depth_nm(60.0);
        assert!((d - 1.5).abs() < 0.01); // 3 * 1.0 * cos(60) = 1.5
    }

    #[test]
    fn test_backscatter_factor() {
        let r = AesProcessor::backscatter_factor(5000.0, 29.0); // Cu
        assert!(r > 1.0, "backscatter factor should be > 1");
        assert!(r < 3.0, "backscatter factor should be reasonable");
    }

    #[test]
    fn test_backscatter_factor_high_z() {
        let r_low = AesProcessor::backscatter_factor(5000.0, 6.0);
        let r_high = AesProcessor::backscatter_factor(5000.0, 79.0);
        assert!(r_high > r_low, "higher Z should have larger backscatter");
    }

    #[test]
    fn test_sputter_rate() {
        let rate = AesProcessor::sputter_rate_nm_per_s(2.0, 1e-6, 8.5e22, 0.01);
        assert!(rate > 0.0, "sputter rate should be positive");
    }

    #[test]
    fn test_depth_profile() {
        let times = vec![0.0, 60.0, 120.0, 180.0];
        let spectra = vec![
            vec![("O".into(), 60.0), ("Si".into(), 40.0)],
            vec![("O".into(), 50.0), ("Si".into(), 50.0)],
            vec![("O".into(), 10.0), ("Si".into(), 90.0)],
            vec![("O".into(), 2.0), ("Si".into(), 98.0)],
        ];
        let profile = AesProcessor::depth_profile(&times, &spectra, 0.5);
        assert_eq!(profile.len(), 4);
        assert!((profile[0].depth_nm - 0.0).abs() < 0.01);
        assert!((profile[1].depth_nm - 30.0).abs() < 0.01);
        assert!((profile[3].depth_nm - 90.0).abs() < 0.01);
    }

    #[test]
    fn test_interface_depth() {
        let profile = vec![
            DepthProfilePoint { depth_nm: 0.0, concentrations: vec![("O".into(), 60.0)] },
            DepthProfilePoint { depth_nm: 10.0, concentrations: vec![("O".into(), 55.0)] },
            DepthProfilePoint { depth_nm: 20.0, concentrations: vec![("O".into(), 45.0)] },
            DepthProfilePoint { depth_nm: 30.0, concentrations: vec![("O".into(), 5.0)] },
        ];
        let iface = AesProcessor::interface_depth(&profile, "O", 50.0);
        assert!(iface.is_some());
        let d = iface.unwrap();
        assert!(d > 10.0 && d < 20.0, "interface should be between 10 and 20 nm, got {d}");
    }

    #[test]
    fn test_interface_depth_not_found() {
        let profile = vec![
            DepthProfilePoint { depth_nm: 0.0, concentrations: vec![("O".into(), 80.0)] },
            DepthProfilePoint { depth_nm: 10.0, concentrations: vec![("O".into(), 70.0)] },
        ];
        let iface = AesProcessor::interface_depth(&profile, "O", 50.0);
        assert!(iface.is_none());
    }

    #[test]
    fn test_auger_energy() {
        // KLL for carbon: ~284 - 6 - 6 - ~4.5 = ~267.5
        let e = AesProcessor::auger_energy(284.0, 6.0, 6.0, 4.5);
        assert!((e - 267.5).abs() < 0.1);
    }

    #[test]
    fn test_estimate_imfp() {
        let imfp_100 = AesProcessor::estimate_imfp_nm(100.0);
        let imfp_1000 = AesProcessor::estimate_imfp_nm(1000.0);
        assert!(imfp_100 > 0.0);
        assert!(imfp_1000 > imfp_100, "IMFP should increase with energy");
        assert!((imfp_100 - 0.54).abs() < 0.01); // 0.054 * sqrt(100)
    }

    #[test]
    fn test_estimate_imfp_zero() {
        assert_eq!(AesProcessor::estimate_imfp_nm(0.0), 0.0);
    }

    #[test]
    fn test_chemical_shift() {
        let shift = AesProcessor::chemical_shift(270.0, 272.0);
        assert!((shift - (-2.0)).abs() < 0.01);
    }

    #[test]
    fn test_identify_peak() {
        let matches = AesProcessor::identify_peak(510.0, 5.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "O");
    }

    #[test]
    fn test_identify_peak_no_match() {
        let matches = AesProcessor::identify_peak(600.0, 2.0);
        assert!(matches.is_empty());
    }

    #[test]
    fn test_subtract_linear_background() {
        let energy: Vec<f64> = (0..100).map(|i| i as f64 * 10.0).collect();
        let counts: Vec<f64> = energy.iter().map(|&e| 100.0 + 0.5 * e).collect();
        let sub = AesProcessor::subtract_linear_background(&energy, &counts, 0.0, 990.0);
        // After removing linear background, should be near zero
        let mean: f64 = sub.iter().sum::<f64>() / sub.len() as f64;
        assert!(mean.abs() < 10.0, "mean after BG subtraction should be near 0, got {mean}");
    }

    #[test]
    fn test_matrix_factor() {
        let mf = AesProcessor::matrix_factor(100.0, 0.14, 200.0, 0.40, 1.5);
        assert!(mf > 0.0, "matrix factor should be positive");
    }

    #[test]
    fn test_matrix_factor_zero() {
        let mf = AesProcessor::matrix_factor(0.0, 0.14, 0.0, 0.40, 1.5);
        assert_eq!(mf, 0.0);
    }

    #[test]
    fn test_derivative_short_spectrum() {
        let spectrum = AesSpectrum {
            energy_ev: vec![100.0, 200.0],
            counts: vec![50.0, 60.0],
        };
        let proc = AesProcessor::new(AesConfig::default(), spectrum);
        let deriv = proc.derivative_spectrum();
        assert_eq!(deriv.len(), 2);
    }

    #[test]
    fn test_quantify_single_element() {
        let spectrum = make_test_spectrum(&[(272.0, 3000.0, 8.0)], 501);
        let proc = AesProcessor::new(AesConfig::default(), spectrum);
        let result = proc.quantify(&[("C", 272.0, 40.0)]);
        assert_eq!(result.len(), 1);
        assert!((result[0].1 - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_spectrum_construction() {
        let s = make_test_spectrum(&[(300.0, 1000.0, 5.0)], 201);
        assert_eq!(s.energy_ev.len(), 201);
        assert_eq!(s.counts.len(), 201);
        assert!(s.energy_ev[0] < s.energy_ev[200]);
    }

    #[test]
    fn test_depth_profile_point() {
        let p = DepthProfilePoint {
            depth_nm: 15.0,
            concentrations: vec![("Si".into(), 80.0), ("O".into(), 20.0)],
        };
        assert_eq!(p.depth_nm, 15.0);
        assert_eq!(p.concentrations.len(), 2);
    }

    #[test]
    fn test_sputter_rate_zero_area() {
        let rate = AesProcessor::sputter_rate_nm_per_s(2.0, 1e-6, 8.5e22, 0.0);
        assert_eq!(rate, 0.0);
    }
}
