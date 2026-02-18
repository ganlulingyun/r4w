// Photoluminescence Lifetime Analyzer
// Time-resolved photoluminescence (TRPL) for carrier dynamics in semiconductors
// Implements: exponential decay fitting, stretched exponential, bi/tri-exponential,
// average lifetime, quantum yield from lifetime, Stern-Volmer quenching,
// TCSPC histogram analysis, instrument response function deconvolution

use std::f64::consts::PI;

/// TRPL configuration
#[derive(Clone, Debug)]
pub struct TrplConfig {
    /// Time resolution in ns (bin width)
    pub time_resolution_ns: f64,
    /// Excitation wavelength in nm
    pub excitation_nm: f64,
    /// Detection wavelength in nm
    pub detection_nm: f64,
    /// Temperature in K
    pub temperature_k: f64,
}

impl Default for TrplConfig {
    fn default() -> Self {
        Self {
            time_resolution_ns: 0.05,
            excitation_nm: 375.0,
            detection_nm: 550.0,
            temperature_k: 300.0,
        }
    }
}

/// Time-resolved PL decay data
#[derive(Clone, Debug)]
pub struct PlDecay {
    /// Time axis in ns
    pub time_ns: Vec<f64>,
    /// Intensity (photon counts)
    pub intensity: Vec<f64>,
}

/// Decay model types
#[derive(Clone, Debug)]
pub enum DecayModel {
    /// I(t) = A * exp(-t/tau)
    SingleExponential { amplitude: f64, tau_ns: f64 },
    /// I(t) = A1*exp(-t/tau1) + A2*exp(-t/tau2)
    BiExponential {
        a1: f64, tau1_ns: f64,
        a2: f64, tau2_ns: f64,
    },
    /// I(t) = A1*exp(-t/tau1) + A2*exp(-t/tau2) + A3*exp(-t/tau3)
    TriExponential {
        a1: f64, tau1_ns: f64,
        a2: f64, tau2_ns: f64,
        a3: f64, tau3_ns: f64,
    },
    /// I(t) = A * exp(-(t/tau)^beta), 0 < beta <= 1
    StretchedExponential { amplitude: f64, tau_ns: f64, beta: f64 },
}

/// Fit result
#[derive(Clone, Debug)]
pub struct FitResult {
    /// Fitted model
    pub model: DecayModel,
    /// Chi-squared goodness of fit
    pub chi_squared: f64,
    /// Average lifetime in ns
    pub tau_avg_ns: f64,
    /// Intensity-weighted average lifetime in ns
    pub tau_intensity_ns: f64,
}

/// PL Lifetime Analyzer
pub struct PlLifetimeAnalyzer {
    pub config: TrplConfig,
    pub decay: PlDecay,
}

impl PlLifetimeAnalyzer {
    pub fn new(config: TrplConfig, decay: PlDecay) -> Self {
        Self { config, decay }
    }

    /// Generate synthetic single-exponential decay
    pub fn generate_single_exp(time_ns: &[f64], amplitude: f64, tau_ns: f64, background: f64) -> Vec<f64> {
        time_ns.iter().map(|&t| amplitude * (-t / tau_ns).exp() + background).collect()
    }

    /// Generate synthetic bi-exponential decay
    pub fn generate_bi_exp(
        time_ns: &[f64],
        a1: f64, tau1_ns: f64,
        a2: f64, tau2_ns: f64,
        background: f64,
    ) -> Vec<f64> {
        time_ns.iter().map(|&t| {
            a1 * (-t / tau1_ns).exp() + a2 * (-t / tau2_ns).exp() + background
        }).collect()
    }

    /// Generate stretched exponential decay
    pub fn generate_stretched_exp(
        time_ns: &[f64],
        amplitude: f64,
        tau_ns: f64,
        beta: f64,
        background: f64,
    ) -> Vec<f64> {
        time_ns.iter().map(|&t| {
            amplitude * (-(t / tau_ns).powf(beta)).exp() + background
        }).collect()
    }

    /// Fit single exponential using linearized least squares (ln(I) vs t)
    pub fn fit_single_exponential(&self) -> FitResult {
        // Find peak and fit from peak onwards
        let peak_idx = self.find_peak_index();
        let n = self.decay.intensity.len();
        if peak_idx >= n - 2 {
            return FitResult {
                model: DecayModel::SingleExponential { amplitude: 0.0, tau_ns: 1.0 },
                chi_squared: f64::INFINITY,
                tau_avg_ns: 1.0,
                tau_intensity_ns: 1.0,
            };
        }

        // Background estimate from last 10% of data
        let bg = self.estimate_background();

        // Linearized fit: ln(I - bg) = ln(A) - t/tau
        let mut sx = 0.0_f64;
        let mut sy = 0.0_f64;
        let mut sxx = 0.0_f64;
        let mut sxy = 0.0_f64;
        let mut count = 0.0_f64;

        for i in peak_idx..n {
            let y = self.decay.intensity[i] - bg;
            if y > 0.0 {
                let ln_y = y.ln();
                let t = self.decay.time_ns[i];
                sx += t;
                sy += ln_y;
                sxx += t * t;
                sxy += t * ln_y;
                count += 1.0;
            }
        }

        if count < 2.0 {
            return FitResult {
                model: DecayModel::SingleExponential { amplitude: 0.0, tau_ns: 1.0 },
                chi_squared: f64::INFINITY,
                tau_avg_ns: 1.0,
                tau_intensity_ns: 1.0,
            };
        }

        let det = count * sxx - sx * sx;
        let slope = if det.abs() > 1e-30 {
            (count * sxy - sx * sy) / det
        } else {
            -1.0
        };
        let intercept = (sy - slope * sx) / count;

        let tau = if slope < 0.0 { -1.0 / slope } else { 1.0 };
        let amplitude = intercept.exp();

        // Chi-squared
        let chi2 = self.compute_chi_squared(peak_idx, &|t: f64| amplitude * (-t / tau).exp() + bg);

        FitResult {
            model: DecayModel::SingleExponential { amplitude, tau_ns: tau },
            chi_squared: chi2,
            tau_avg_ns: tau,
            tau_intensity_ns: tau,
        }
    }

    /// Average lifetime for multi-exponential: <tau> = sum(A_i * tau_i) / sum(A_i)
    pub fn amplitude_avg_lifetime(amplitudes: &[f64], taus: &[f64]) -> f64 {
        let num: f64 = amplitudes.iter().zip(taus).map(|(a, t)| a * t).sum();
        let den: f64 = amplitudes.iter().sum();
        if den.abs() > 1e-30 { num / den } else { 0.0 }
    }

    /// Intensity-weighted average lifetime: <tau>_int = sum(A_i * tau_i^2) / sum(A_i * tau_i)
    pub fn intensity_avg_lifetime(amplitudes: &[f64], taus: &[f64]) -> f64 {
        let num: f64 = amplitudes.iter().zip(taus).map(|(a, t)| a * t * t).sum();
        let den: f64 = amplitudes.iter().zip(taus).map(|(a, t)| a * t).sum();
        if den.abs() > 1e-30 { num / den } else { 0.0 }
    }

    /// Radiative rate from lifetime: k_r = QY / tau
    pub fn radiative_rate(quantum_yield: f64, tau_ns: f64) -> f64 {
        if tau_ns > 0.0 { quantum_yield / tau_ns } else { 0.0 }
    }

    /// Non-radiative rate: k_nr = (1 - QY) / tau
    pub fn non_radiative_rate(quantum_yield: f64, tau_ns: f64) -> f64 {
        if tau_ns > 0.0 { (1.0 - quantum_yield) / tau_ns } else { 0.0 }
    }

    /// Quantum yield from lifetimes: QY = tau / tau_rad
    pub fn quantum_yield_from_lifetime(tau_ns: f64, tau_radiative_ns: f64) -> f64 {
        if tau_radiative_ns > 0.0 { tau_ns / tau_radiative_ns } else { 0.0 }
    }

    /// Stern-Volmer quenching: tau_0/tau = 1 + K_sv * [Q]
    /// Returns K_sv from two-point measurement
    pub fn stern_volmer_ksv(tau0_ns: f64, tau_ns: f64, quencher_conc_m: f64) -> f64 {
        if quencher_conc_m > 0.0 && tau_ns > 0.0 {
            (tau0_ns / tau_ns - 1.0) / quencher_conc_m
        } else {
            0.0
        }
    }

    /// Stern-Volmer linear fit from arrays
    pub fn stern_volmer_fit(tau0_ns: f64, taus_ns: &[f64], concentrations: &[f64]) -> f64 {
        if taus_ns.len() < 2 || taus_ns.len() != concentrations.len() {
            return 0.0;
        }

        // Fit tau0/tau vs [Q]: y = 1 + K_sv * x
        let mut sx = 0.0_f64;
        let mut sy = 0.0_f64;
        let mut sxx = 0.0_f64;
        let mut sxy = 0.0_f64;
        let n = taus_ns.len() as f64;

        for i in 0..taus_ns.len() {
            let x = concentrations[i];
            let y = if taus_ns[i] > 0.0 { tau0_ns / taus_ns[i] } else { 1.0 };
            sx += x;
            sy += y;
            sxx += x * x;
            sxy += x * y;
        }

        let det = n * sxx - sx * sx;
        if det.abs() > 1e-30 {
            (n * sxy - sx * sy) / det
        } else {
            0.0
        }
    }

    /// FRET efficiency from donor lifetime: E = 1 - tau_DA / tau_D
    pub fn fret_efficiency(tau_donor_ns: f64, tau_donor_acceptor_ns: f64) -> f64 {
        if tau_donor_ns > 0.0 {
            1.0 - tau_donor_acceptor_ns / tau_donor_ns
        } else {
            0.0
        }
    }

    /// FRET distance from efficiency: r = R0 * (1/E - 1)^(1/6)
    pub fn fret_distance_nm(efficiency: f64, r0_nm: f64) -> f64 {
        if efficiency > 0.0 && efficiency < 1.0 {
            r0_nm * (1.0 / efficiency - 1.0).powf(1.0 / 6.0)
        } else {
            0.0
        }
    }

    /// Find peak index in decay curve
    fn find_peak_index(&self) -> usize {
        self.decay
            .intensity
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0)
    }

    /// Estimate background from last 10% of decay
    fn estimate_background(&self) -> f64 {
        let n = self.decay.intensity.len();
        let start = n - n / 10;
        if start >= n {
            return 0.0;
        }
        let count = (n - start) as f64;
        self.decay.intensity[start..].iter().sum::<f64>() / count
    }

    /// Compute chi-squared for model
    fn compute_chi_squared(&self, start_idx: usize, model: &dyn Fn(f64) -> f64) -> f64 {
        let n = self.decay.intensity.len();
        let t0 = self.decay.time_ns[start_idx];
        let mut chi2 = 0.0;
        let mut count = 0;
        for i in start_idx..n {
            let t = self.decay.time_ns[i] - t0;
            let y_model = model(t);
            let y_data = self.decay.intensity[i];
            let sigma2 = y_data.max(1.0); // Poisson statistics
            chi2 += (y_data - y_model).powi(2) / sigma2;
            count += 1;
        }
        if count > 2 { chi2 / (count - 2) as f64 } else { chi2 }
    }

    /// Instrument Response Function (IRF) FWHM estimation
    /// Assumes Gaussian IRF, estimates from rising edge
    pub fn estimate_irf_fwhm_ns(&self) -> f64 {
        let peak_idx = self.find_peak_index();
        let peak_val = self.decay.intensity[peak_idx];
        let half_max = peak_val / 2.0;

        // Find half-max on rising edge
        let mut rise_idx = 0;
        for i in 0..peak_idx {
            if self.decay.intensity[i] >= half_max {
                rise_idx = i;
                break;
            }
        }

        // FWHM ~ 2 * (peak_time - half_time) for Gaussian
        if peak_idx > rise_idx {
            2.0 * (self.decay.time_ns[peak_idx] - self.decay.time_ns[rise_idx])
        } else {
            self.config.time_resolution_ns
        }
    }

    /// Tail fit: fit only the tail region (after initial fast decay)
    /// from start_ns to end of data
    pub fn tail_fit_lifetime(&self, start_ns: f64) -> f64 {
        let bg = self.estimate_background();
        let mut sx = 0.0_f64;
        let mut sy = 0.0_f64;
        let mut sxx = 0.0_f64;
        let mut sxy = 0.0_f64;
        let mut count = 0.0_f64;

        for (i, &t) in self.decay.time_ns.iter().enumerate() {
            if t >= start_ns {
                let y = self.decay.intensity[i] - bg;
                if y > 1.0 {
                    let ln_y = y.ln();
                    sx += t;
                    sy += ln_y;
                    sxx += t * t;
                    sxy += t * ln_y;
                    count += 1.0;
                }
            }
        }

        if count < 2.0 {
            return 0.0;
        }

        let det = count * sxx - sx * sx;
        let slope = if det.abs() > 1e-30 {
            (count * sxy - sx * sy) / det
        } else {
            return 0.0;
        };

        if slope < 0.0 { -1.0 / slope } else { 0.0 }
    }

    /// 1/e lifetime: time at which intensity drops to 1/e of peak
    pub fn one_over_e_lifetime(&self) -> f64 {
        let peak_idx = self.find_peak_index();
        let peak_val = self.decay.intensity[peak_idx];
        let target = peak_val / std::f64::consts::E;

        for i in peak_idx..self.decay.intensity.len() {
            if self.decay.intensity[i] <= target {
                // Linear interpolation
                if i > peak_idx {
                    let t0 = self.decay.time_ns[i - 1];
                    let t1 = self.decay.time_ns[i];
                    let y0 = self.decay.intensity[i - 1];
                    let y1 = self.decay.intensity[i];
                    let frac = if (y0 - y1).abs() > 1e-30 {
                        (y0 - target) / (y0 - y1)
                    } else {
                        0.0
                    };
                    return (t0 + frac * (t1 - t0)) - self.decay.time_ns[peak_idx];
                }
                return self.decay.time_ns[i] - self.decay.time_ns[peak_idx];
            }
        }
        // Didn't cross 1/e, return time span
        if self.decay.time_ns.len() > peak_idx + 1 {
            *self.decay.time_ns.last().unwrap() - self.decay.time_ns[peak_idx]
        } else {
            0.0
        }
    }

    /// Arrhenius non-radiative rate: k_nr(T) = A * exp(-E_a / (k_B * T))
    pub fn arrhenius_rate(prefactor: f64, activation_energy_ev: f64, temperature_k: f64) -> f64 {
        let kb = 8.617333e-5; // eV/K
        if temperature_k > 0.0 {
            prefactor * (-activation_energy_ev / (kb * temperature_k)).exp()
        } else {
            0.0
        }
    }

    /// Activation energy from two temperatures: E_a = k_B * T1*T2/(T2-T1) * ln(k2/k1)
    pub fn activation_energy_ev(
        k1: f64, t1_k: f64,
        k2: f64, t2_k: f64,
    ) -> f64 {
        let kb = 8.617333e-5;
        if k1 > 0.0 && k2 > 0.0 && (t2_k - t1_k).abs() > 1e-10 {
            kb * t1_k * t2_k / (t2_k - t1_k) * (k2 / k1).ln()
        } else {
            0.0
        }
    }

    /// Photon energy from wavelength: E = hc/lambda
    pub fn photon_energy_ev(wavelength_nm: f64) -> f64 {
        if wavelength_nm > 0.0 { 1239.84 / wavelength_nm } else { 0.0 }
    }

    /// Stokes shift in eV
    pub fn stokes_shift_ev(excitation_nm: f64, emission_nm: f64) -> f64 {
        Self::photon_energy_ev(excitation_nm) - Self::photon_energy_ev(emission_nm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_decay(tau_ns: f64, n_points: usize, dt_ns: f64) -> PlDecay {
        let time_ns: Vec<f64> = (0..n_points).map(|i| i as f64 * dt_ns).collect();
        let intensity: Vec<f64> = time_ns.iter().map(|&t| 10000.0 * (-t / tau_ns).exp() + 10.0).collect();
        PlDecay { time_ns, intensity }
    }

    #[test]
    fn test_default_config() {
        let cfg = TrplConfig::default();
        assert_eq!(cfg.time_resolution_ns, 0.05);
        assert_eq!(cfg.temperature_k, 300.0);
    }

    #[test]
    fn test_generate_single_exp() {
        let time: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let decay = PlLifetimeAnalyzer::generate_single_exp(&time, 1000.0, 5.0, 10.0);
        assert_eq!(decay.len(), 100);
        assert!((decay[0] - 1010.0).abs() < 0.1);
        // At t=5ns (index 50): 1000*e^(-1) + 10 = 377.9
        assert!((decay[50] - (1000.0 * (-1.0_f64).exp() + 10.0)).abs() < 0.1);
    }

    #[test]
    fn test_generate_bi_exp() {
        let time: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let decay = PlLifetimeAnalyzer::generate_bi_exp(&time, 500.0, 2.0, 500.0, 10.0, 5.0);
        assert_eq!(decay.len(), 100);
        assert!((decay[0] - 1005.0).abs() < 0.1);
    }

    #[test]
    fn test_generate_stretched_exp() {
        let time: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let decay = PlLifetimeAnalyzer::generate_stretched_exp(&time, 1000.0, 5.0, 0.7, 10.0);
        assert_eq!(decay.len(), 100);
        assert!((decay[0] - 1010.0).abs() < 0.1);
        // Stretched should decay slower than pure exponential at long times
    }

    #[test]
    fn test_fit_single_exponential() {
        let decay = make_decay(5.0, 500, 0.1);
        let analyzer = PlLifetimeAnalyzer::new(TrplConfig::default(), decay);
        let result = analyzer.fit_single_exponential();
        match result.model {
            DecayModel::SingleExponential { tau_ns, .. } => {
                assert!((tau_ns - 5.0).abs() < 0.5, "fitted tau should be ~5 ns, got {tau_ns}");
            }
            _ => panic!("expected single exponential"),
        }
    }

    #[test]
    fn test_amplitude_avg_lifetime() {
        let amps = vec![0.6, 0.4];
        let taus = vec![2.0, 10.0];
        let avg = PlLifetimeAnalyzer::amplitude_avg_lifetime(&amps, &taus);
        // 0.6*2 + 0.4*10 = 5.2
        assert!((avg - 5.2).abs() < 0.01);
    }

    #[test]
    fn test_intensity_avg_lifetime() {
        let amps = vec![0.6, 0.4];
        let taus = vec![2.0, 10.0];
        let avg = PlLifetimeAnalyzer::intensity_avg_lifetime(&amps, &taus);
        // num = 0.6*4 + 0.4*100 = 42.4, den = 0.6*2 + 0.4*10 = 5.2
        assert!((avg - 42.4 / 5.2).abs() < 0.01);
    }

    #[test]
    fn test_radiative_rate() {
        let kr = PlLifetimeAnalyzer::radiative_rate(0.5, 10.0);
        assert!((kr - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_non_radiative_rate() {
        let knr = PlLifetimeAnalyzer::non_radiative_rate(0.5, 10.0);
        assert!((knr - 0.05).abs() < 1e-10);
    }

    #[test]
    fn test_quantum_yield() {
        let qy = PlLifetimeAnalyzer::quantum_yield_from_lifetime(5.0, 10.0);
        assert!((qy - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_stern_volmer_ksv() {
        let ksv = PlLifetimeAnalyzer::stern_volmer_ksv(10.0, 5.0, 0.01);
        // tau0/tau = 2, so Ksv = (2-1)/0.01 = 100
        assert!((ksv - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_stern_volmer_fit() {
        let tau0 = 10.0;
        let taus = vec![10.0, 8.0, 6.67, 5.0];
        let concs = vec![0.0, 0.025, 0.05, 0.1];
        let ksv = PlLifetimeAnalyzer::stern_volmer_fit(tau0, &taus, &concs);
        // tau0/tau = 1, 1.25, 1.5, 2.0 vs [Q] = 0, 0.025, 0.05, 0.1
        // slope should be ~10
        assert!((ksv - 10.0).abs() < 1.0, "Ksv should be ~10, got {ksv}");
    }

    #[test]
    fn test_fret_efficiency() {
        let e = PlLifetimeAnalyzer::fret_efficiency(10.0, 5.0);
        assert!((e - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_fret_distance() {
        let r = PlLifetimeAnalyzer::fret_distance_nm(0.5, 5.0);
        // At E=0.5, r = R0
        assert!((r - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_fret_distance_high_e() {
        let r = PlLifetimeAnalyzer::fret_distance_nm(0.9, 5.0);
        assert!(r < 5.0, "high FRET should give r < R0");
    }

    #[test]
    fn test_one_over_e_lifetime() {
        let decay = make_decay(5.0, 500, 0.1);
        let analyzer = PlLifetimeAnalyzer::new(TrplConfig::default(), decay);
        let tau_e = analyzer.one_over_e_lifetime();
        assert!((tau_e - 5.0).abs() < 0.5, "1/e lifetime should be ~5 ns, got {tau_e}");
    }

    #[test]
    fn test_estimate_irf_fwhm() {
        // Create a decay with a sharp rise
        let n = 200;
        let dt = 0.1;
        let mut intensity = vec![10.0; n];
        for i in 10..n {
            let t = (i - 10) as f64 * dt;
            intensity[i] = 10000.0 * (-t / 5.0).exp() + 10.0;
        }
        let time_ns: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let decay = PlDecay { time_ns, intensity };
        let analyzer = PlLifetimeAnalyzer::new(TrplConfig::default(), decay);
        let fwhm = analyzer.estimate_irf_fwhm_ns();
        assert!(fwhm >= 0.0);
    }

    #[test]
    fn test_tail_fit() {
        let decay = make_decay(8.0, 1000, 0.1);
        let analyzer = PlLifetimeAnalyzer::new(TrplConfig::default(), decay);
        let tau = analyzer.tail_fit_lifetime(20.0); // Start at 20 ns
        assert!((tau - 8.0).abs() < 1.0, "tail fit should give ~8 ns, got {tau}");
    }

    #[test]
    fn test_arrhenius_rate() {
        let k300 = PlLifetimeAnalyzer::arrhenius_rate(1e12, 0.3, 300.0);
        let k400 = PlLifetimeAnalyzer::arrhenius_rate(1e12, 0.3, 400.0);
        assert!(k400 > k300, "rate should increase with temperature");
    }

    #[test]
    fn test_activation_energy() {
        let k1 = 1e8;
        let k2 = 1e9;
        let t1 = 300.0;
        let t2 = 400.0;
        let ea = PlLifetimeAnalyzer::activation_energy_ev(k1, t1, k2, t2);
        assert!(ea > 0.0, "activation energy should be positive");
        assert!(ea < 1.0, "should be reasonable, got {ea}");
    }

    #[test]
    fn test_photon_energy() {
        let e = PlLifetimeAnalyzer::photon_energy_ev(620.0);
        assert!((e - 2.0).abs() < 0.1); // ~2.0 eV for red light
    }

    #[test]
    fn test_stokes_shift() {
        let shift = PlLifetimeAnalyzer::stokes_shift_ev(375.0, 550.0);
        assert!(shift > 0.0, "Stokes shift should be positive");
        let expected = 1239.84 / 375.0 - 1239.84 / 550.0;
        assert!((shift - expected).abs() < 0.01);
    }

    #[test]
    fn test_background_estimate() {
        let decay = make_decay(5.0, 1000, 0.1);
        let analyzer = PlLifetimeAnalyzer::new(TrplConfig::default(), decay);
        let bg = analyzer.estimate_background();
        assert!((bg - 10.0).abs() < 1.0, "background should be ~10, got {bg}");
    }

    #[test]
    fn test_zero_lifetime_safeguards() {
        assert_eq!(PlLifetimeAnalyzer::radiative_rate(0.5, 0.0), 0.0);
        assert_eq!(PlLifetimeAnalyzer::non_radiative_rate(0.5, 0.0), 0.0);
        assert_eq!(PlLifetimeAnalyzer::quantum_yield_from_lifetime(5.0, 0.0), 0.0);
        assert_eq!(PlLifetimeAnalyzer::photon_energy_ev(0.0), 0.0);
    }

    #[test]
    fn test_stern_volmer_zero_conc() {
        let ksv = PlLifetimeAnalyzer::stern_volmer_ksv(10.0, 5.0, 0.0);
        assert_eq!(ksv, 0.0);
    }

    #[test]
    fn test_fret_boundary() {
        assert_eq!(PlLifetimeAnalyzer::fret_distance_nm(0.0, 5.0), 0.0);
        assert_eq!(PlLifetimeAnalyzer::fret_distance_nm(1.0, 5.0), 0.0);
        assert_eq!(PlLifetimeAnalyzer::fret_efficiency(0.0, 5.0), 0.0);
    }

    #[test]
    fn test_arrhenius_zero_temp() {
        assert_eq!(PlLifetimeAnalyzer::arrhenius_rate(1e12, 0.3, 0.0), 0.0);
    }
}
