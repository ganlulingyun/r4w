// Neutron Activation Analysis (NAA) Processor
//
// Implements core NAA algorithms:
// - Activation equation: A = N*σ*φ * S * D * C
// - Decay constant from half-life
// - Saturation/decay/counting factors
// - Comparator method (sample vs standard)
// - k0 standardization
// - Gamma-ray peak fitting (Gaussian + linear background)
// - Net peak area with Compton background subtraction
// - Interference corrections
// - Detection limit (Currie criterion)
// - Multi-element analysis
// - Self-shielding correction
// - Coincidence summing correction

use std::f64::consts::PI;

/// Avogadro's number
const NA: f64 = 6.02214076e23;

/// Decay constant from half-life: lambda = ln(2) / t_half
pub fn decay_constant(half_life_s: f64) -> f64 {
    if half_life_s <= 0.0 {
        return 0.0;
    }
    2.0_f64.ln() / half_life_s
}

/// Half-life from decay constant
pub fn half_life(lambda_s: f64) -> f64 {
    if lambda_s <= 0.0 {
        return f64::INFINITY;
    }
    2.0_f64.ln() / lambda_s
}

/// Saturation factor: S = 1 - exp(-lambda * t_irr)
pub fn saturation_factor(lambda_s: f64, irradiation_time_s: f64) -> f64 {
    if lambda_s <= 0.0 || irradiation_time_s <= 0.0 {
        return 0.0;
    }
    1.0 - (-lambda_s * irradiation_time_s).exp()
}

/// Decay factor: D = exp(-lambda * t_d)
pub fn decay_factor(lambda_s: f64, decay_time_s: f64) -> f64 {
    if lambda_s <= 0.0 {
        return 1.0;
    }
    (-lambda_s * decay_time_s).exp()
}

/// Counting factor: C = (1 - exp(-lambda * t_c)) / (lambda * t_c)
pub fn counting_factor(lambda_s: f64, counting_time_s: f64) -> f64 {
    if lambda_s <= 0.0 || counting_time_s <= 0.0 {
        return 1.0;
    }
    let lt = lambda_s * counting_time_s;
    if lt < 1e-6 {
        // Taylor expansion for small lt: 1 - lt/2 + lt^2/6 ...
        return 1.0 - lt / 2.0 + lt * lt / 6.0;
    }
    (1.0 - (-lt).exp()) / lt
}

/// Full activation equation
/// A = N * sigma * phi * S * D * C
/// where N = number of target atoms, sigma = cross section, phi = neutron flux
pub fn activity(
    n_target_atoms: f64,
    cross_section_barn: f64,  // 1 barn = 1e-24 cm²
    neutron_flux_cm2_s: f64,
    lambda_s: f64,
    irradiation_time_s: f64,
    decay_time_s: f64,
    counting_time_s: f64,
) -> f64 {
    let sigma_cm2 = cross_section_barn * 1e-24;
    let s = saturation_factor(lambda_s, irradiation_time_s);
    let d = decay_factor(lambda_s, decay_time_s);
    let c = counting_factor(lambda_s, counting_time_s);
    n_target_atoms * sigma_cm2 * neutron_flux_cm2_s * s * d * c
}

/// Number of target atoms from mass and isotopic abundance
/// N = m * NA * theta / M
pub fn target_atoms(
    mass_g: f64,
    atomic_mass_g: f64,
    isotopic_abundance: f64, // fraction 0-1
) -> f64 {
    if atomic_mass_g <= 0.0 {
        return 0.0;
    }
    mass_g * NA * isotopic_abundance / atomic_mass_g
}

/// Comparator method: mass of analyte from sample/standard comparison
/// m_sample = m_std * (A_sample * D_std * C_std) / (A_std * D_sample * C_sample)
pub fn comparator_mass(
    std_mass_g: f64,
    sample_counts: f64,
    std_counts: f64,
    lambda_s: f64,
    sample_decay_time_s: f64,
    std_decay_time_s: f64,
    sample_counting_time_s: f64,
    std_counting_time_s: f64,
) -> f64 {
    if std_counts <= 0.0 {
        return 0.0;
    }
    let d_sample = decay_factor(lambda_s, sample_decay_time_s);
    let d_std = decay_factor(lambda_s, std_decay_time_s);
    let c_sample = counting_factor(lambda_s, sample_counting_time_s);
    let c_std = counting_factor(lambda_s, std_counting_time_s);

    if d_sample * c_sample < 1e-30 {
        return 0.0;
    }

    std_mass_g * (sample_counts * d_std * c_std) / (std_counts * d_sample * c_sample)
}

/// k0 method: concentration from single comparator
/// C_a = (Np_a / (t_c * SDC * epsilon * gamma * k0)) *
///       (M_a / (NA * theta * sigma * phi * m_sample))
/// Simplified: C_a = (A_sample / A_monitor) / (k0 * f_factor)
pub fn k0_concentration(
    sample_count_rate: f64,
    monitor_count_rate: f64,
    k0_factor: f64,
    sample_mass_g: f64,
) -> f64 {
    if monitor_count_rate <= 0.0 || k0_factor <= 0.0 || sample_mass_g <= 0.0 {
        return 0.0;
    }
    (sample_count_rate / monitor_count_rate) / (k0_factor * sample_mass_g)
}

/// Gaussian peak fitting: fit I(E) = A * exp(-(E-E0)^2/(2*sigma^2)) + bkg
/// Returns (amplitude, center, sigma, background)
pub fn fit_gaussian_peak(
    channels: &[f64],
    counts: &[f64],
) -> (f64, f64, f64, f64) {
    let n = channels.len().min(counts.len());
    if n < 5 {
        return (0.0, 0.0, 0.0, 0.0);
    }

    // Estimate background from edges
    let n_bg = (n / 5).max(2);
    let bg_left: f64 = counts[..n_bg].iter().sum::<f64>() / n_bg as f64;
    let bg_right: f64 = counts[n - n_bg..].iter().sum::<f64>() / n_bg as f64;
    let background = (bg_left + bg_right) / 2.0;

    // Find peak
    let mut max_val = f64::NEG_INFINITY;
    let mut max_idx = 0;
    for i in 0..n {
        if counts[i] > max_val {
            max_val = counts[i];
            max_idx = i;
        }
    }

    let amplitude = max_val - background;
    let center = channels[max_idx];

    // Estimate sigma from FWHM
    let half_max = background + amplitude / 2.0;
    let mut left_ch = channels[0];
    let mut right_ch = channels[n - 1];

    for i in 0..max_idx {
        if counts[i] <= half_max && counts[i + 1] > half_max {
            let frac = (half_max - counts[i]) / (counts[i + 1] - counts[i]);
            left_ch = channels[i] + frac * (channels[i + 1] - channels[i]);
            break;
        }
    }

    for i in max_idx..n - 1 {
        if counts[i] >= half_max && counts[i + 1] < half_max {
            let frac = (half_max - counts[i + 1]) / (counts[i] - counts[i + 1]);
            right_ch = channels[i + 1] + frac * (channels[i] - channels[i + 1]);
            break;
        }
    }

    let fwhm = (right_ch - left_ch).abs();
    let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0_f64.sqrt());

    (amplitude, center, sigma, background)
}

/// Net peak area: total counts minus background
pub fn net_peak_area(
    counts: &[f64],
    peak_start: usize,
    peak_end: usize,
    bg_left_start: usize,
    bg_left_end: usize,
    bg_right_start: usize,
    bg_right_end: usize,
) -> f64 {
    let n = counts.len();
    let peak_end = peak_end.min(n);
    let bg_left_end = bg_left_end.min(n);
    let bg_right_end = bg_right_end.min(n);

    if peak_start >= peak_end {
        return 0.0;
    }

    let total_peak: f64 = counts[peak_start..peak_end].iter().sum();
    let n_peak = (peak_end - peak_start) as f64;

    let n_bg_left = if bg_left_end > bg_left_start { (bg_left_end - bg_left_start) as f64 } else { 0.0 };
    let n_bg_right = if bg_right_end > bg_right_start { (bg_right_end - bg_right_start) as f64 } else { 0.0 };

    let bg_left_sum: f64 = if n_bg_left > 0.0 {
        counts[bg_left_start..bg_left_end].iter().sum()
    } else {
        0.0
    };
    let bg_right_sum: f64 = if n_bg_right > 0.0 {
        counts[bg_right_start..bg_right_end].iter().sum()
    } else {
        0.0
    };

    let n_bg_total = n_bg_left + n_bg_right;
    let bg_rate = if n_bg_total > 0.0 {
        (bg_left_sum + bg_right_sum) / n_bg_total
    } else {
        0.0
    };

    total_peak - bg_rate * n_peak
}

/// Detection limit (Currie criterion)
/// LD = 2.71 + 4.65 * sqrt(B)
/// where B = total background counts in the peak region
pub fn detection_limit_currie(background_counts: f64) -> f64 {
    if background_counts < 0.0 {
        return 2.71;
    }
    2.71 + 4.65 * background_counts.sqrt()
}

/// Critical level (Currie)
/// LC = 2.33 * sqrt(B)
pub fn critical_level_currie(background_counts: f64) -> f64 {
    if background_counts <= 0.0 {
        return 0.0;
    }
    2.33 * background_counts.sqrt()
}

/// Spectral interference correction
/// Net_corrected = Net_measured - sum_j(IF_j * Net_interfering_j)
pub fn interference_correction(
    measured_area: f64,
    interfering_areas: &[f64],
    interference_factors: &[f64],
) -> f64 {
    let n = interfering_areas.len().min(interference_factors.len());
    let correction: f64 = (0..n)
        .map(|i| interference_factors[i] * interfering_areas[i])
        .sum();
    measured_area - correction
}

/// Self-shielding correction factor for thermal neutrons
/// G_th = (1 / (Sigma_t * d)) * (1 - exp(-Sigma_t * d))
/// where Sigma_t = macroscopic cross section (cm^-1), d = sample thickness (cm)
pub fn self_shielding_factor(
    macro_cross_section_cm: f64,  // cm^-1
    thickness_cm: f64,
) -> f64 {
    let st = macro_cross_section_cm * thickness_cm;
    if st < 1e-6 {
        return 1.0 - st / 2.0; // Taylor expansion
    }
    (1.0 - (-st).exp()) / st
}

/// Coincidence summing correction (simplified, for two-cascade gamma)
/// True count rate = Measured * (1 + epsilon_total_sum)
/// For true coincidence: correction ≈ 1 / (1 - epsilon_total * P_cascade)
pub fn coincidence_correction(
    total_efficiency: f64,
    cascade_probability: f64,
) -> f64 {
    let product = total_efficiency * cascade_probability;
    if product >= 1.0 {
        return 1.0; // cap
    }
    1.0 / (1.0 - product)
}

/// Dead time correction (non-paralyzable model)
/// N_true = N_measured / (1 - N_measured * tau)
pub fn dead_time_correction(
    measured_rate_cps: f64,
    dead_time_s: f64,
) -> f64 {
    let denom = 1.0 - measured_rate_cps * dead_time_s;
    if denom <= 0.0 {
        return measured_rate_cps; // saturated
    }
    measured_rate_cps / denom
}

/// Energy calibration: channel to energy (linear)
/// E(keV) = a + b * channel
pub fn energy_from_channel(channel: f64, offset_kev: f64, gain_kev_per_ch: f64) -> f64 {
    offset_kev + gain_kev_per_ch * channel
}

/// Channel from energy
pub fn channel_from_energy(energy_kev: f64, offset_kev: f64, gain_kev_per_ch: f64) -> f64 {
    if gain_kev_per_ch.abs() < 1e-15 {
        return 0.0;
    }
    (energy_kev - offset_kev) / gain_kev_per_ch
}

/// NAA Processor
pub struct NaaProcessor {
    pub neutron_flux: f64,           // n/(cm²·s)
    pub irradiation_time_s: f64,
    pub elements: Vec<NaaElement>,
}

/// Element data for NAA
#[derive(Debug, Clone)]
pub struct NaaElement {
    pub symbol: String,
    pub atomic_mass: f64,
    pub isotopic_abundance: f64,
    pub cross_section_barn: f64,
    pub half_life_s: f64,
    pub gamma_energy_kev: f64,
    pub gamma_intensity: f64,  // fraction
}

impl NaaProcessor {
    pub fn new(neutron_flux: f64, irradiation_time_s: f64) -> Self {
        Self {
            neutron_flux,
            irradiation_time_s,
            elements: Vec::new(),
        }
    }

    pub fn add_element(&mut self, elem: NaaElement) {
        self.elements.push(elem);
    }

    /// Calculate expected activity for a given mass of element
    pub fn expected_activity(&self, symbol: &str, mass_g: f64, decay_time_s: f64, counting_time_s: f64) -> Option<f64> {
        let elem = self.elements.iter().find(|e| e.symbol == symbol)?;
        let n_atoms = target_atoms(mass_g, elem.atomic_mass, elem.isotopic_abundance);
        let lambda = decay_constant(elem.half_life_s);
        Some(activity(
            n_atoms,
            elem.cross_section_barn,
            self.neutron_flux,
            lambda,
            self.irradiation_time_s,
            decay_time_s,
            counting_time_s,
        ))
    }

    /// Determine mass from measured counts (comparator method)
    pub fn determine_mass(
        &self,
        symbol: &str,
        sample_counts: f64,
        std_counts: f64,
        std_mass_g: f64,
        sample_decay_s: f64,
        std_decay_s: f64,
        counting_time_s: f64,
    ) -> Option<f64> {
        let elem = self.elements.iter().find(|e| e.symbol == symbol)?;
        let lambda = decay_constant(elem.half_life_s);
        Some(comparator_mass(
            std_mass_g,
            sample_counts,
            std_counts,
            lambda,
            sample_decay_s,
            std_decay_s,
            counting_time_s,
            counting_time_s,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_decay_constant() {
        // Co-60: t½ = 5.27 years = 1.663e8 s
        let lambda = decay_constant(1.663e8);
        assert!(approx_eq(lambda, 4.167e-9, 1e-11));
    }

    #[test]
    fn test_decay_constant_zero() {
        assert_eq!(decay_constant(0.0), 0.0);
    }

    #[test]
    fn test_half_life_roundtrip() {
        let lambda = decay_constant(3600.0);
        let t_half = half_life(lambda);
        assert!(approx_eq(t_half, 3600.0, 0.01));
    }

    #[test]
    fn test_saturation_factor_short() {
        // Very short irradiation: S ≈ lambda * t_irr
        let lambda = decay_constant(3600.0);
        let s = saturation_factor(lambda, 1.0);
        assert!(approx_eq(s, lambda * 1.0, lambda * 0.01));
    }

    #[test]
    fn test_saturation_factor_long() {
        // Very long irradiation: S → 1
        let lambda = decay_constant(3600.0);
        let s = saturation_factor(lambda, 1e6);
        assert!(s > 0.99);
    }

    #[test]
    fn test_saturation_factor_zero() {
        assert_eq!(saturation_factor(0.001, 0.0), 0.0);
    }

    #[test]
    fn test_decay_factor() {
        let lambda = decay_constant(3600.0);
        let d = decay_factor(lambda, 3600.0);
        assert!(approx_eq(d, 0.5, 0.001));
    }

    #[test]
    fn test_decay_factor_zero_time() {
        let d = decay_factor(0.001, 0.0);
        assert!(approx_eq(d, 1.0, 1e-10));
    }

    #[test]
    fn test_counting_factor_short() {
        // Short counting time: C ≈ 1
        let lambda = decay_constant(3600.0);
        let c = counting_factor(lambda, 0.001);
        assert!(approx_eq(c, 1.0, 0.001));
    }

    #[test]
    fn test_counting_factor_moderate() {
        let lambda = decay_constant(3600.0);
        let c = counting_factor(lambda, 3600.0);
        // C = (1 - 0.5) / (ln2) = 0.721
        assert!(approx_eq(c, 0.721, 0.01), "c = {}", c);
    }

    #[test]
    fn test_activity_positive() {
        let n = target_atoms(0.001, 59.0, 1.0); // 1 mg Co-59
        let lambda = decay_constant(5.272 * 365.25 * 86400.0);
        let a = activity(n, 37.18, 1e13, lambda, 86400.0, 0.0, 3600.0);
        assert!(a > 0.0);
    }

    #[test]
    fn test_target_atoms() {
        // 1 g of pure element with M=60
        let n = target_atoms(1.0, 60.0, 1.0);
        assert!(approx_eq(n, NA / 60.0, 1e18));
    }

    #[test]
    fn test_target_atoms_zero_mass() {
        assert_eq!(target_atoms(1.0, 0.0, 1.0), 0.0);
    }

    #[test]
    fn test_comparator_method() {
        let lambda = decay_constant(3600.0);
        // Same decay/counting times: mass ratio = count ratio
        let m = comparator_mass(
            0.001,   // 1 mg std
            5000.0,  // sample counts
            10000.0, // std counts
            lambda,
            100.0,   // same decay time
            100.0,
            300.0,   // same counting time
            300.0,
        );
        assert!(approx_eq(m, 0.0005, 0.0001));
    }

    #[test]
    fn test_comparator_zero_std() {
        let m = comparator_mass(0.001, 5000.0, 0.0, 0.001, 100.0, 100.0, 300.0, 300.0);
        assert_eq!(m, 0.0);
    }

    #[test]
    fn test_k0_concentration() {
        let c = k0_concentration(100.0, 1000.0, 0.5, 0.1);
        assert!(approx_eq(c, 2.0, 0.01));
    }

    #[test]
    fn test_k0_concentration_zero() {
        assert_eq!(k0_concentration(100.0, 0.0, 0.5, 0.1), 0.0);
    }

    #[test]
    fn test_fit_gaussian_peak() {
        // Generate Gaussian peak + background
        let n = 100;
        let mut channels = Vec::with_capacity(n);
        let mut counts = Vec::with_capacity(n);
        let center = 50.0;
        let sigma = 3.0;
        let amplitude = 1000.0;
        let bg = 50.0;

        for i in 0..n {
            let ch = i as f64;
            channels.push(ch);
            let peak = amplitude * (-(ch - center).powi(2) / (2.0 * sigma * sigma)).exp();
            counts.push(peak + bg);
        }

        let (amp, ctr, sig, bkg) = fit_gaussian_peak(&channels, &counts);
        assert!(approx_eq(amp, amplitude, amplitude * 0.1), "amp = {}", amp);
        assert!(approx_eq(ctr, center, 1.0), "center = {}", ctr);
        assert!(approx_eq(sig, sigma, 1.0), "sigma = {}", sig);
        assert!(approx_eq(bkg, bg, bg * 0.2), "bg = {}", bkg);
    }

    #[test]
    fn test_fit_gaussian_peak_short() {
        let (a, _, _, _) = fit_gaussian_peak(&[0.0, 1.0], &[10.0, 20.0]);
        assert_eq!(a, 0.0);
    }

    #[test]
    fn test_net_peak_area() {
        let mut counts = vec![10.0; 100];
        // Add peak at channels 40-60
        for i in 40..60 {
            counts[i] += 100.0;
        }
        let net = net_peak_area(&counts, 40, 60, 10, 30, 70, 90);
        // Net should be ~20 * 100 = 2000
        assert!(approx_eq(net, 2000.0, 10.0), "net = {}", net);
    }

    #[test]
    fn test_net_peak_area_empty() {
        let net = net_peak_area(&[10.0; 10], 5, 3, 0, 2, 8, 10);
        assert_eq!(net, 0.0);
    }

    #[test]
    fn test_detection_limit_currie() {
        let ld = detection_limit_currie(100.0);
        // LD = 2.71 + 4.65 * 10 = 49.21
        assert!(approx_eq(ld, 49.21, 0.1));
    }

    #[test]
    fn test_detection_limit_zero_bg() {
        let ld = detection_limit_currie(0.0);
        assert!(approx_eq(ld, 2.71, 0.01));
    }

    #[test]
    fn test_critical_level() {
        let lc = critical_level_currie(100.0);
        assert!(approx_eq(lc, 23.3, 0.1));
    }

    #[test]
    fn test_interference_correction() {
        let corrected = interference_correction(
            1000.0,
            &[200.0, 100.0],
            &[0.1, 0.05],
        );
        // 1000 - (0.1*200 + 0.05*100) = 1000 - 25 = 975
        assert!(approx_eq(corrected, 975.0, 0.01));
    }

    #[test]
    fn test_self_shielding_thin() {
        // Thin sample: G ≈ 1
        let g = self_shielding_factor(0.1, 0.001);
        assert!(approx_eq(g, 1.0, 0.001));
    }

    #[test]
    fn test_self_shielding_thick() {
        // Thick sample: G < 1
        let g = self_shielding_factor(1.0, 5.0);
        assert!(g > 0.0 && g < 1.0, "g = {}", g);
    }

    #[test]
    fn test_coincidence_correction() {
        let c = coincidence_correction(0.1, 0.5);
        // 1/(1-0.05) = 1.0526
        assert!(approx_eq(c, 1.0526, 0.001));
    }

    #[test]
    fn test_coincidence_correction_zero() {
        let c = coincidence_correction(0.0, 0.5);
        assert!(approx_eq(c, 1.0, 1e-10));
    }

    #[test]
    fn test_dead_time_correction() {
        let true_rate = dead_time_correction(10000.0, 5e-6);
        // N_true = 10000 / (1 - 0.05) = 10526
        assert!(approx_eq(true_rate, 10526.3, 1.0));
    }

    #[test]
    fn test_dead_time_saturated() {
        // Saturated detector
        let rate = dead_time_correction(200000.0, 5e-6);
        // denom = 1 - 1 = 0 → returns measured
        assert!(approx_eq(rate, 200000.0, 1.0));
    }

    #[test]
    fn test_energy_calibration() {
        let e = energy_from_channel(500.0, 0.5, 0.5);
        assert!(approx_eq(e, 250.5, 0.01));
    }

    #[test]
    fn test_channel_from_energy() {
        let ch = channel_from_energy(250.5, 0.5, 0.5);
        assert!(approx_eq(ch, 500.0, 0.01));
    }

    #[test]
    fn test_channel_from_energy_zero_gain() {
        assert_eq!(channel_from_energy(100.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn test_processor_new() {
        let proc = NaaProcessor::new(1e13, 3600.0);
        assert!(approx_eq(proc.neutron_flux, 1e13, 1.0));
    }

    #[test]
    fn test_processor_add_element() {
        let mut proc = NaaProcessor::new(1e13, 3600.0);
        proc.add_element(NaaElement {
            symbol: "Na".to_string(),
            atomic_mass: 22.99,
            isotopic_abundance: 1.0,
            cross_section_barn: 0.53,
            half_life_s: 15.0 * 3600.0, // Na-24: 15 hours
            gamma_energy_kev: 1368.6,
            gamma_intensity: 1.0,
        });
        assert_eq!(proc.elements.len(), 1);
    }

    #[test]
    fn test_processor_expected_activity() {
        let mut proc = NaaProcessor::new(1e13, 3600.0);
        proc.add_element(NaaElement {
            symbol: "Na".to_string(),
            atomic_mass: 22.99,
            isotopic_abundance: 1.0,
            cross_section_barn: 0.53,
            half_life_s: 54000.0,
            gamma_energy_kev: 1368.6,
            gamma_intensity: 1.0,
        });
        let a = proc.expected_activity("Na", 0.001, 0.0, 3600.0);
        assert!(a.is_some());
        assert!(a.unwrap() > 0.0);
    }

    #[test]
    fn test_processor_expected_activity_unknown() {
        let proc = NaaProcessor::new(1e13, 3600.0);
        assert!(proc.expected_activity("Uq", 1.0, 0.0, 100.0).is_none());
    }

    #[test]
    fn test_processor_determine_mass() {
        let mut proc = NaaProcessor::new(1e13, 3600.0);
        proc.add_element(NaaElement {
            symbol: "Au".to_string(),
            atomic_mass: 196.97,
            isotopic_abundance: 1.0,
            cross_section_barn: 98.65,
            half_life_s: 2.695 * 86400.0, // Au-198: 2.695 days
            gamma_energy_kev: 411.8,
            gamma_intensity: 0.956,
        });

        // Equal counts, equal conditions → equal mass
        let m = proc.determine_mass("Au", 5000.0, 5000.0, 0.001, 100.0, 100.0, 300.0);
        assert!(m.is_some());
        assert!(approx_eq(m.unwrap(), 0.001, 0.0001));
    }

    #[test]
    fn test_processor_determine_mass_unknown() {
        let proc = NaaProcessor::new(1e13, 3600.0);
        assert!(proc.determine_mass("Uq", 100.0, 100.0, 0.001, 0.0, 0.0, 100.0).is_none());
    }

    #[test]
    fn test_decay_chain() {
        // After 3 half-lives, activity should be 1/8
        let lambda = decay_constant(100.0); // t½ = 100s
        let d = decay_factor(lambda, 300.0); // 3 half-lives
        assert!(approx_eq(d, 0.125, 0.001));
    }
}
