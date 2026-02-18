// Inductively Coupled Plasma Optical Emission Spectrometry (ICP-OES/AES) Processor
//
// Implements core ICP-OES analysis algorithms:
// - Emission line wavelength lookup (common elements)
// - Boltzmann distribution intensity
// - Plasma temperature from line pair ratio (Boltzmann plot)
// - Electron density from Stark broadening
// - Internal standard ratio method
// - Multi-element calibration (linear regression)
// - Detection limit estimation
// - Spectral interference correction (IEC)
// - Background correction (off-peak)
// - Signal-to-background ratio (SBR)
// - Saha ionization equation

use std::f64::consts::PI;

const K_B: f64 = 1.380649e-23;   // Boltzmann constant (J/K)
const H: f64 = 6.62607e-34;       // Planck constant (J·s)
const C_LIGHT: f64 = 2.998e8;     // Speed of light (m/s)
const EV_TO_J: f64 = 1.602176634e-19;

/// Emission line data
#[derive(Debug, Clone)]
pub struct EmissionLine {
    pub element: &'static str,
    pub wavelength_nm: f64,
    pub energy_upper_ev: f64,
    pub energy_lower_ev: f64,
    pub g_upper: f64,            // statistical weight of upper level
    pub transition_prob_s: f64,  // Einstein A coefficient (s^-1)
}

/// Common ICP-OES emission lines
pub fn emission_line_database() -> Vec<EmissionLine> {
    vec![
        EmissionLine { element: "Fe", wavelength_nm: 259.940, energy_upper_ev: 4.767, energy_lower_ev: 0.000, g_upper: 9.0, transition_prob_s: 2.2e8 },
        EmissionLine { element: "Fe", wavelength_nm: 238.204, energy_upper_ev: 5.204, energy_lower_ev: 0.000, g_upper: 7.0, transition_prob_s: 3.0e8 },
        EmissionLine { element: "Cu", wavelength_nm: 324.754, energy_upper_ev: 3.817, energy_lower_ev: 0.000, g_upper: 4.0, transition_prob_s: 1.4e8 },
        EmissionLine { element: "Cu", wavelength_nm: 327.396, energy_upper_ev: 3.786, energy_lower_ev: 0.000, g_upper: 2.0, transition_prob_s: 1.37e8 },
        EmissionLine { element: "Zn", wavelength_nm: 213.856, energy_upper_ev: 5.796, energy_lower_ev: 0.000, g_upper: 3.0, transition_prob_s: 7.1e8 },
        EmissionLine { element: "Pb", wavelength_nm: 220.353, energy_upper_ev: 5.624, energy_lower_ev: 0.000, g_upper: 3.0, transition_prob_s: 1.2e8 },
        EmissionLine { element: "Cd", wavelength_nm: 228.802, energy_upper_ev: 5.418, energy_lower_ev: 0.000, g_upper: 3.0, transition_prob_s: 5.3e8 },
        EmissionLine { element: "Mn", wavelength_nm: 257.610, energy_upper_ev: 4.812, energy_lower_ev: 0.000, g_upper: 8.0, transition_prob_s: 3.7e8 },
        EmissionLine { element: "Cr", wavelength_nm: 267.716, energy_upper_ev: 4.633, energy_lower_ev: 0.000, g_upper: 9.0, transition_prob_s: 2.1e8 },
        EmissionLine { element: "Ni", wavelength_nm: 231.604, energy_upper_ev: 5.353, energy_lower_ev: 0.000, g_upper: 5.0, transition_prob_s: 2.0e8 },
        EmissionLine { element: "Ca", wavelength_nm: 393.366, energy_upper_ev: 3.151, energy_lower_ev: 0.000, g_upper: 4.0, transition_prob_s: 1.5e8 },
        EmissionLine { element: "Na", wavelength_nm: 589.592, energy_upper_ev: 2.104, energy_lower_ev: 0.000, g_upper: 4.0, transition_prob_s: 6.16e7 },
        EmissionLine { element: "K",  wavelength_nm: 766.490, energy_upper_ev: 1.617, energy_lower_ev: 0.000, g_upper: 4.0, transition_prob_s: 3.87e7 },
        EmissionLine { element: "Mg", wavelength_nm: 285.213, energy_upper_ev: 4.346, energy_lower_ev: 0.000, g_upper: 3.0, transition_prob_s: 4.91e8 },
        EmissionLine { element: "Al", wavelength_nm: 396.152, energy_upper_ev: 3.143, energy_lower_ev: 0.013, g_upper: 4.0, transition_prob_s: 9.85e7 },
    ]
}

/// Look up emission lines for an element
pub fn lookup_element(element: &str) -> Vec<EmissionLine> {
    emission_line_database()
        .into_iter()
        .filter(|l| l.element == element)
        .collect()
}

/// Boltzmann distribution: intensity of an emission line
/// I = A * g_u * N / Z(T) * exp(-E_u / (k*T))
/// Returns relative intensity (proportional to actual)
pub fn boltzmann_intensity(
    line: &EmissionLine,
    temperature_k: f64,
    partition_function: f64,
) -> f64 {
    if temperature_k <= 0.0 || partition_function <= 0.0 {
        return 0.0;
    }
    let e_upper_j = line.energy_upper_ev * EV_TO_J;
    line.transition_prob_s * line.g_upper
        * (-e_upper_j / (K_B * temperature_k)).exp()
        / partition_function
}

/// Plasma temperature from two-line method (Boltzmann plot)
/// T = (E2 - E1) / (k * ln((I1*A2*g2)/(I2*A1*g1)))
pub fn plasma_temperature_two_line(
    line1: &EmissionLine,
    intensity1: f64,
    line2: &EmissionLine,
    intensity2: f64,
) -> f64 {
    if intensity1 <= 0.0 || intensity2 <= 0.0 {
        return 0.0;
    }

    let de_ev = line2.energy_upper_ev - line1.energy_upper_ev;
    let de_j = de_ev * EV_TO_J;

    let ratio = (intensity1 * line2.transition_prob_s * line2.g_upper)
        / (intensity2 * line1.transition_prob_s * line1.g_upper);

    if ratio <= 0.0 {
        return 0.0;
    }

    let ln_ratio = ratio.ln();
    if ln_ratio.abs() < 1e-30 {
        return 0.0;
    }

    de_j / (K_B * ln_ratio)
}

/// Boltzmann plot: ln(I*lambda/(g*A)) vs E_upper
/// Returns (temperature_k, intercept) from linear fit
pub fn boltzmann_plot(
    lines: &[EmissionLine],
    intensities: &[f64],
) -> (f64, f64) {
    let n = lines.len().min(intensities.len());
    if n < 2 {
        return (0.0, 0.0);
    }

    let mut xs = Vec::with_capacity(n);
    let mut ys = Vec::with_capacity(n);

    for i in 0..n {
        if intensities[i] <= 0.0 || lines[i].transition_prob_s <= 0.0 || lines[i].g_upper <= 0.0 {
            continue;
        }
        let x = lines[i].energy_upper_ev;
        let y = (intensities[i] * lines[i].wavelength_nm * 1e-9
            / (lines[i].g_upper * lines[i].transition_prob_s))
            .ln();
        xs.push(x);
        ys.push(y);
    }

    if xs.len() < 2 {
        return (0.0, 0.0);
    }

    let (slope, intercept) = linear_fit(&xs, &ys);
    // slope = -1/(k*T) where k in eV/K = 8.617e-5
    let k_ev = 8.617e-5;
    let temp = if slope.abs() > 1e-30 { -1.0 / (k_ev * slope) } else { 0.0 };

    (temp, intercept)
}

/// Linear least squares fit: y = slope*x + intercept
fn linear_fit(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len().min(y.len());
    if n < 2 {
        return (0.0, 0.0);
    }
    let nf = n as f64;
    let sx: f64 = x[..n].iter().sum();
    let sy: f64 = y[..n].iter().sum();
    let sxx: f64 = x[..n].iter().map(|&v| v * v).sum();
    let sxy: f64 = x[..n].iter().zip(y[..n].iter()).map(|(&a, &b)| a * b).sum();

    let denom = nf * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / nf);
    }
    let slope = (nf * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / nf;
    (slope, intercept)
}

/// Electron density from Stark broadening (Hβ line at 486.1 nm)
/// ne = (delta_lambda / alpha_s)^(3/2) where alpha_s is tabulated
/// Simplified: ne ≈ (delta_lambda_nm / 4.0)^1.5 × 10^23 m^-3 for Hβ
pub fn electron_density_stark(
    hbeta_fwhm_nm: f64,
) -> f64 {
    if hbeta_fwhm_nm <= 0.0 {
        return 0.0;
    }
    // Simplified Stark broadening for Hβ
    let alpha_ref = 4.0; // nm at ne = 10^23 m^-3 (approximate)
    (hbeta_fwhm_nm / alpha_ref).powf(1.5) * 1e23
}

/// Internal standard method: ratio of analyte to IS line
/// C_analyte = (I_analyte / I_IS) * factor
pub fn internal_standard_ratio(
    analyte_intensity: f64,
    is_intensity: f64,
    calibration_factor: f64,
) -> f64 {
    if is_intensity <= 0.0 {
        return 0.0;
    }
    (analyte_intensity / is_intensity) * calibration_factor
}

/// Multi-element calibration result
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    pub element: String,
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
}

/// Linear calibration for an element
pub fn calibrate_element(
    element: &str,
    concentrations: &[f64],
    intensities: &[f64],
) -> CalibrationResult {
    let n = concentrations.len().min(intensities.len());
    if n < 2 {
        return CalibrationResult {
            element: element.to_string(),
            slope: 0.0,
            intercept: 0.0,
            r_squared: 0.0,
        };
    }

    let (slope, intercept) = linear_fit(concentrations, intensities);

    // R² calculation
    let mean_y: f64 = intensities[..n].iter().sum::<f64>() / n as f64;
    let ss_tot: f64 = intensities[..n].iter().map(|&y| (y - mean_y).powi(2)).sum();
    let ss_res: f64 = (0..n)
        .map(|i| (intensities[i] - (slope * concentrations[i] + intercept)).powi(2))
        .sum();
    let r2 = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

    CalibrationResult {
        element: element.to_string(),
        slope,
        intercept,
        r_squared: r2,
    }
}

/// Concentration from calibration
pub fn concentration_from_cal(
    intensity: f64,
    cal: &CalibrationResult,
) -> f64 {
    if cal.slope.abs() < 1e-30 {
        return 0.0;
    }
    (intensity - cal.intercept) / cal.slope
}

/// Detection limit: DL = 3 * sigma_blank * C_std / (I_std - I_blank)
pub fn detection_limit_icp(
    blank_std_dev: f64,
    std_concentration: f64,
    std_intensity: f64,
    blank_intensity: f64,
) -> f64 {
    let net = std_intensity - blank_intensity;
    if net.abs() < 1e-30 {
        return f64::INFINITY;
    }
    3.0 * blank_std_dev * std_concentration / net
}

/// Signal-to-background ratio
pub fn sbr(signal_intensity: f64, background_intensity: f64) -> f64 {
    if background_intensity.abs() < 1e-30 {
        return 0.0;
    }
    (signal_intensity - background_intensity) / background_intensity
}

/// Spectral interference correction
/// I_corrected = I_measured - sum(IEC_j * I_interfering_j)
pub fn spectral_interference_correction(
    measured_intensity: f64,
    interfering_intensities: &[f64],
    iec_factors: &[f64],
) -> f64 {
    let n = interfering_intensities.len().min(iec_factors.len());
    let correction: f64 = (0..n)
        .map(|i| iec_factors[i] * interfering_intensities[i])
        .sum();
    measured_intensity - correction
}

/// Background correction: subtract average of two off-peak measurements
pub fn background_correction(
    peak_intensity: f64,
    bg_left: f64,
    bg_right: f64,
) -> f64 {
    peak_intensity - (bg_left + bg_right) / 2.0
}

/// Saha ionization equation
/// N_i+1 * ne / N_i = (2 * Z_i+1 / Z_i) * (2*pi*me*kT/h^2)^(3/2) * exp(-Eion/(kT))
/// Returns ionization ratio N_i+1/N_i given ne
pub fn saha_ionization_ratio(
    ionization_energy_ev: f64,
    temperature_k: f64,
    electron_density_m3: f64,
    partition_ratio: f64, // Z_{i+1} / Z_i
) -> f64 {
    if temperature_k <= 0.0 || electron_density_m3 <= 0.0 {
        return 0.0;
    }
    let me = 9.1094e-31; // electron mass (kg)
    let e_ion_j = ionization_energy_ev * EV_TO_J;

    let thermal_factor = (2.0 * PI * me * K_B * temperature_k / (H * H)).powf(1.5);
    let boltzmann_factor = (-e_ion_j / (K_B * temperature_k)).exp();

    2.0 * partition_ratio * thermal_factor * boltzmann_factor / electron_density_m3
}

/// Ionization suppression factor
/// When adding easily ionized element (EIE), electron density increases,
/// shifting equilibrium to reduce ionization of analyte
pub fn ionization_fraction(
    ionization_energy_ev: f64,
    temperature_k: f64,
    electron_density_m3: f64,
) -> f64 {
    let ratio = saha_ionization_ratio(ionization_energy_ev, temperature_k, electron_density_m3, 1.0);
    ratio / (1.0 + ratio)
}

/// Spectral line profile: Voigt (Gaussian + Lorentzian convolution approximation)
/// Uses pseudo-Voigt approximation: V ≈ eta*L + (1-eta)*G
pub fn voigt_profile(
    wavelength_nm: f64,
    center_nm: f64,
    gaussian_fwhm_nm: f64,
    lorentzian_fwhm_nm: f64,
) -> f64 {
    let fg = gaussian_fwhm_nm;
    let fl = lorentzian_fwhm_nm;
    // Total FWHM
    let fv = (fg.powi(5) + 2.69269 * fg.powi(4) * fl
        + 2.42843 * fg.powi(3) * fl.powi(2)
        + 4.47163 * fg.powi(2) * fl.powi(3)
        + 0.07842 * fg * fl.powi(4)
        + fl.powi(5))
        .powf(0.2);

    // Mixing parameter
    let eta = 1.36603 * (fl / fv) - 0.47719 * (fl / fv).powi(2) + 0.11116 * (fl / fv).powi(3);
    let eta = eta.clamp(0.0, 1.0);

    let dw = wavelength_nm - center_nm;

    // Gaussian
    let sg = fg / (2.0 * (2.0_f64.ln()).sqrt());
    let g = if sg > 1e-30 {
        (1.0 / (sg * (2.0 * PI).sqrt())) * (-dw * dw / (2.0 * sg * sg)).exp()
    } else {
        0.0
    };

    // Lorentzian
    let gamma = fl / 2.0;
    let l = if gamma > 1e-30 {
        gamma / (PI * (dw * dw + gamma * gamma))
    } else {
        0.0
    };

    eta * l + (1.0 - eta) * g
}

/// ICP-OES Processor
pub struct IcpOesProcessor {
    pub calibrations: Vec<CalibrationResult>,
    pub temperature_k: f64,
}

impl IcpOesProcessor {
    pub fn new() -> Self {
        Self {
            calibrations: Vec::new(),
            temperature_k: 7500.0, // typical ICP temperature
        }
    }

    pub fn add_calibration(&mut self, cal: CalibrationResult) {
        self.calibrations.push(cal);
    }

    pub fn get_calibration(&self, element: &str) -> Option<&CalibrationResult> {
        self.calibrations.iter().find(|c| c.element == element)
    }

    pub fn quantify(&self, element: &str, intensity: f64) -> Option<f64> {
        let cal = self.get_calibration(element)?;
        Some(concentration_from_cal(intensity, cal))
    }

    pub fn lookup_lines(&self, element: &str) -> Vec<EmissionLine> {
        lookup_element(element)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_emission_line_database() {
        let db = emission_line_database();
        assert!(db.len() >= 10);
    }

    #[test]
    fn test_lookup_element_fe() {
        let lines = lookup_element("Fe");
        assert!(lines.len() >= 2);
        assert!(lines.iter().all(|l| l.element == "Fe"));
    }

    #[test]
    fn test_lookup_element_unknown() {
        let lines = lookup_element("Unobtanium");
        assert!(lines.is_empty());
    }

    #[test]
    fn test_boltzmann_intensity_positive() {
        let line = EmissionLine {
            element: "Fe", wavelength_nm: 259.94, energy_upper_ev: 4.767,
            energy_lower_ev: 0.0, g_upper: 9.0, transition_prob_s: 2.2e8,
        };
        let i = boltzmann_intensity(&line, 7500.0, 50.0);
        assert!(i > 0.0);
    }

    #[test]
    fn test_boltzmann_intensity_increases_with_temp() {
        let line = EmissionLine {
            element: "Fe", wavelength_nm: 259.94, energy_upper_ev: 4.767,
            energy_lower_ev: 0.0, g_upper: 9.0, transition_prob_s: 2.2e8,
        };
        let i1 = boltzmann_intensity(&line, 5000.0, 50.0);
        let i2 = boltzmann_intensity(&line, 8000.0, 50.0);
        assert!(i2 > i1);
    }

    #[test]
    fn test_boltzmann_intensity_zero_temp() {
        let line = EmissionLine {
            element: "Fe", wavelength_nm: 259.94, energy_upper_ev: 4.767,
            energy_lower_ev: 0.0, g_upper: 9.0, transition_prob_s: 2.2e8,
        };
        assert_eq!(boltzmann_intensity(&line, 0.0, 50.0), 0.0);
    }

    #[test]
    fn test_plasma_temperature_two_line() {
        let fe_lines = lookup_element("Fe");
        assert!(fe_lines.len() >= 2);
        let l1 = &fe_lines[0];
        let l2 = &fe_lines[1];

        // Generate intensities at known temperature
        let t_true = 7000.0;
        let z = 50.0;
        let i1 = boltzmann_intensity(l1, t_true, z);
        let i2 = boltzmann_intensity(l2, t_true, z);

        let t_calc = plasma_temperature_two_line(l1, i1, l2, i2);
        assert!(approx_eq(t_calc, t_true, 100.0), "T = {}", t_calc);
    }

    #[test]
    fn test_plasma_temperature_zero_intensity() {
        let l1 = &emission_line_database()[0];
        let l2 = &emission_line_database()[1];
        assert_eq!(plasma_temperature_two_line(l1, 0.0, l2, 100.0), 0.0);
    }

    #[test]
    fn test_boltzmann_plot() {
        let db = emission_line_database();
        let fe_lines: Vec<_> = db.iter().filter(|l| l.element == "Fe").cloned().collect();
        if fe_lines.len() < 2 {
            return;
        }

        let t_true = 7500.0;
        let z = 50.0;
        let intensities: Vec<f64> = fe_lines.iter().map(|l| boltzmann_intensity(l, t_true, z) * z).collect();

        let (t_calc, _) = boltzmann_plot(&fe_lines, &intensities);
        // With only 2 Fe lines, result should be exact
        assert!(t_calc > 5000.0 && t_calc < 10000.0, "T = {}", t_calc);
    }

    #[test]
    fn test_electron_density_stark() {
        // Hβ FWHM of 1 nm
        let ne = electron_density_stark(1.0);
        assert!(ne > 0.0);
        // Should be on order of 10^22
        assert!(ne > 1e20 && ne < 1e25, "ne = {}", ne);
    }

    #[test]
    fn test_electron_density_stark_zero() {
        assert_eq!(electron_density_stark(0.0), 0.0);
    }

    #[test]
    fn test_electron_density_stark_scaling() {
        let ne1 = electron_density_stark(1.0);
        let ne2 = electron_density_stark(2.0);
        assert!(ne2 > ne1);
    }

    #[test]
    fn test_internal_standard_ratio() {
        let conc = internal_standard_ratio(1000.0, 500.0, 10.0);
        assert!(approx_eq(conc, 20.0, 0.01));
    }

    #[test]
    fn test_internal_standard_zero_is() {
        assert_eq!(internal_standard_ratio(1000.0, 0.0, 10.0), 0.0);
    }

    #[test]
    fn test_calibrate_element() {
        let concs = vec![0.0, 1.0, 2.0, 5.0, 10.0];
        let intensities = vec![100.0, 1100.0, 2100.0, 5100.0, 10100.0];
        let cal = calibrate_element("Cu", &concs, &intensities);
        assert_eq!(cal.element, "Cu");
        assert!(approx_eq(cal.slope, 1000.0, 10.0));
        assert!(approx_eq(cal.intercept, 100.0, 10.0));
        assert!(cal.r_squared > 0.999);
    }

    #[test]
    fn test_calibrate_insufficient() {
        let cal = calibrate_element("Cu", &[1.0], &[100.0]);
        assert_eq!(cal.slope, 0.0);
    }

    #[test]
    fn test_concentration_from_cal() {
        let cal = CalibrationResult {
            element: "Cu".to_string(),
            slope: 1000.0,
            intercept: 100.0,
            r_squared: 0.999,
        };
        let c = concentration_from_cal(5100.0, &cal);
        assert!(approx_eq(c, 5.0, 0.01));
    }

    #[test]
    fn test_detection_limit_icp() {
        let dl = detection_limit_icp(10.0, 1.0, 1000.0, 100.0);
        // DL = 3 * 10 * 1.0 / 900 = 0.0333
        assert!(approx_eq(dl, 0.0333, 0.001));
    }

    #[test]
    fn test_detection_limit_zero_net() {
        let dl = detection_limit_icp(10.0, 1.0, 100.0, 100.0);
        assert!(dl.is_infinite());
    }

    #[test]
    fn test_sbr() {
        let s = sbr(1000.0, 100.0);
        assert!(approx_eq(s, 9.0, 0.01));
    }

    #[test]
    fn test_sbr_zero_bg() {
        assert_eq!(sbr(1000.0, 0.0), 0.0);
    }

    #[test]
    fn test_spectral_interference_correction() {
        let corrected = spectral_interference_correction(
            1000.0,
            &[500.0, 200.0],
            &[0.1, 0.05],
        );
        // 1000 - (0.1*500 + 0.05*200) = 1000 - 60 = 940
        assert!(approx_eq(corrected, 940.0, 0.01));
    }

    #[test]
    fn test_background_correction() {
        let net = background_correction(1000.0, 100.0, 120.0);
        assert!(approx_eq(net, 890.0, 0.01));
    }

    #[test]
    fn test_saha_ionization_ratio() {
        let ratio = saha_ionization_ratio(7.64, 7500.0, 1e21, 1.0); // Na ionization
        assert!(ratio > 0.0);
    }

    #[test]
    fn test_saha_zero_temp() {
        assert_eq!(saha_ionization_ratio(7.64, 0.0, 1e21, 1.0), 0.0);
    }

    #[test]
    fn test_saha_higher_temp_more_ionized() {
        let r1 = saha_ionization_ratio(7.64, 6000.0, 1e21, 1.0);
        let r2 = saha_ionization_ratio(7.64, 8000.0, 1e21, 1.0);
        assert!(r2 > r1);
    }

    #[test]
    fn test_ionization_fraction() {
        let frac = ionization_fraction(5.14, 7500.0, 1e21); // Na
        assert!(frac >= 0.0 && frac <= 1.0, "frac = {}", frac);
    }

    #[test]
    fn test_ionization_fraction_high_ne() {
        // Higher ne should suppress ionization
        let f1 = ionization_fraction(7.64, 7500.0, 1e20);
        let f2 = ionization_fraction(7.64, 7500.0, 1e22);
        assert!(f2 < f1, "f1={}, f2={}", f1, f2);
    }

    #[test]
    fn test_voigt_profile_peak() {
        let v = voigt_profile(324.754, 324.754, 0.01, 0.005);
        assert!(v > 0.0);
    }

    #[test]
    fn test_voigt_profile_symmetric() {
        let v_left = voigt_profile(324.744, 324.754, 0.01, 0.005);
        let v_right = voigt_profile(324.764, 324.754, 0.01, 0.005);
        assert!(approx_eq(v_left, v_right, v_left * 0.01));
    }

    #[test]
    fn test_voigt_profile_decays() {
        let v_center = voigt_profile(324.754, 324.754, 0.01, 0.005);
        let v_wing = voigt_profile(324.8, 324.754, 0.01, 0.005);
        assert!(v_wing < v_center);
    }

    #[test]
    fn test_processor_new() {
        let proc = IcpOesProcessor::new();
        assert!(approx_eq(proc.temperature_k, 7500.0, 1.0));
    }

    #[test]
    fn test_processor_add_calibration() {
        let mut proc = IcpOesProcessor::new();
        let cal = calibrate_element("Cu", &[0.0, 1.0, 5.0], &[100.0, 1100.0, 5100.0]);
        proc.add_calibration(cal);
        assert!(proc.get_calibration("Cu").is_some());
        assert!(proc.get_calibration("Fe").is_none());
    }

    #[test]
    fn test_processor_quantify() {
        let mut proc = IcpOesProcessor::new();
        let cal = calibrate_element("Cu", &[0.0, 1.0, 5.0, 10.0], &[100.0, 1100.0, 5100.0, 10100.0]);
        proc.add_calibration(cal);
        let conc = proc.quantify("Cu", 3100.0).unwrap();
        assert!(approx_eq(conc, 3.0, 0.1), "conc = {}", conc);
    }

    #[test]
    fn test_processor_quantify_unknown() {
        let proc = IcpOesProcessor::new();
        assert!(proc.quantify("Cu", 1000.0).is_none());
    }

    #[test]
    fn test_processor_lookup() {
        let proc = IcpOesProcessor::new();
        let lines = proc.lookup_lines("Cu");
        assert!(!lines.is_empty());
    }

    #[test]
    fn test_linear_fit() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let (slope, intercept) = linear_fit(&x, &y);
        assert!(approx_eq(slope, 2.0, 0.01));
        assert!(approx_eq(intercept, 1.0, 0.01));
    }
}
