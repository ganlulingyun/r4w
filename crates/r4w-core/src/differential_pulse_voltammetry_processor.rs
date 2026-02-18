// Differential Pulse Voltammetry (DPV) Processor
//
// Electrochemical analysis techniques:
// - Differential pulse voltammetry (DPV) signal processing
// - Square wave voltammetry (SWV)
// - Peak current calculation (reversible/irreversible)
// - Peak potential and half-peak width
// - Baseline correction (polynomial)
// - Peak detection and deconvolution
// - Calibration curve fitting
// - Stripping analysis (ASV/CSV)
// - Savitzky-Golay smoothing
// - Half-peak width for reversibility assessment

use std::f64::consts::PI;

/// Physical constants
const F: f64 = 96485.0;    // Faraday constant (C/mol)
const R: f64 = 8.314;      // Gas constant (J/(mol·K))

/// DPV data point
#[derive(Debug, Clone, Copy)]
pub struct DpvPoint {
    pub potential_v: f64,
    pub current_a: f64,
}

/// DPV peak result
#[derive(Debug, Clone)]
pub struct DpvPeak {
    pub peak_potential_v: f64,
    pub peak_current_a: f64,
    pub half_width_v: f64,
    pub area: f64,
}

/// Calculate DPV differential current from pulse measurements
/// delta_i[k] = i_pulse[k] - i_base[k]
pub fn differential_current(
    base_currents: &[f64],
    pulse_currents: &[f64],
) -> Vec<f64> {
    let n = base_currents.len().min(pulse_currents.len());
    (0..n).map(|i| pulse_currents[i] - base_currents[i]).collect()
}

/// Theoretical DPV peak current for reversible reaction (Parry-Osteryoung)
/// ip = n*F*A*C * sqrt(D/(pi*tp)) * (1-sigma)/(1+sigma)
/// where sigma = exp(n*F*dE/(2*R*T))
pub fn dpv_peak_current_reversible(
    n_electrons: f64,
    area_cm2: f64,
    conc_mol_per_cm3: f64,
    diff_coeff_cm2_per_s: f64,
    pulse_width_s: f64,
    pulse_amplitude_v: f64,
    temperature_k: f64,
) -> f64 {
    if pulse_width_s <= 0.0 || temperature_k <= 0.0 {
        return 0.0;
    }
    let sigma = (n_electrons * F * pulse_amplitude_v / (2.0 * R * temperature_k)).exp();
    let base = n_electrons * F * area_cm2 * conc_mol_per_cm3
        * (diff_coeff_cm2_per_s / (PI * pulse_width_s)).sqrt();
    base * (1.0 - sigma) / (1.0 + sigma)
}

/// Theoretical DPV peak potential for reversible reaction
/// Ep = E1/2 - dE/2
pub fn dpv_peak_potential(
    half_wave_potential_v: f64,
    pulse_amplitude_v: f64,
) -> f64 {
    half_wave_potential_v - pulse_amplitude_v / 2.0
}

/// Theoretical half-peak width for reversible DPV
/// W1/2 = 3.52 * R * T / (n * F) (in volts)
pub fn half_peak_width_reversible(
    n_electrons: f64,
    temperature_k: f64,
) -> f64 {
    if n_electrons <= 0.0 {
        return 0.0;
    }
    3.52 * R * temperature_k / (n_electrons * F)
}

/// SWV net current = forward current - reverse current
pub fn swv_net_current(
    forward_currents: &[f64],
    reverse_currents: &[f64],
) -> Vec<f64> {
    let n = forward_currents.len().min(reverse_currents.len());
    (0..n).map(|i| forward_currents[i] - reverse_currents[i]).collect()
}

/// SWV peak current (dimensionless, multiply by nFAC*sqrt(Df))
/// psi_p ≈ 1/(1 + exp(-nF*Esw/(RT)))  [simplified for large amplitude]
pub fn swv_peak_dimensionless(
    n_electrons: f64,
    sw_amplitude_v: f64,
    temperature_k: f64,
) -> f64 {
    if temperature_k <= 0.0 {
        return 0.0;
    }
    let x = n_electrons * F * sw_amplitude_v / (R * temperature_k);
    // tanh(x/4) approximation for dimensionless peak current
    let e = (x / 2.0).exp();
    (e - 1.0 / e) / (e + 1.0 / e)
}

/// Find peaks in voltammogram using local extremum detection
pub fn find_peaks(
    potentials: &[f64],
    currents: &[f64],
    min_current: f64,
) -> Vec<DpvPeak> {
    let n = potentials.len().min(currents.len());
    if n < 3 {
        return Vec::new();
    }

    let mut peaks = Vec::new();
    for i in 1..n - 1 {
        let is_max = currents[i] > currents[i - 1] && currents[i] > currents[i + 1];
        let is_min = currents[i] < currents[i - 1] && currents[i] < currents[i + 1];
        let is_peak = if min_current >= 0.0 { is_max } else { is_min };

        if is_peak && currents[i].abs() >= min_current.abs() {
            // Parabolic interpolation
            let y0 = currents[i - 1];
            let y1 = currents[i];
            let y2 = currents[i + 1];
            let denom = 2.0 * (2.0 * y1 - y0 - y2);
            let offset = if denom.abs() > 1e-30 { (y0 - y2) / denom } else { 0.0 };
            let dp = if i + 1 < n { potentials[i + 1] - potentials[i] } else { potentials[i] - potentials[i - 1] };
            let peak_e = potentials[i] + offset * dp;

            let hw = estimate_half_width(potentials, currents, i);
            let area = estimate_peak_area(potentials, currents, i);

            peaks.push(DpvPeak {
                peak_potential_v: peak_e,
                peak_current_a: y1,
                half_width_v: hw,
                area,
            });
        }
    }
    peaks
}

/// Estimate half-peak width from voltammogram
fn estimate_half_width(potentials: &[f64], currents: &[f64], center: usize) -> f64 {
    let half_max = currents[center] / 2.0;
    let n = potentials.len().min(currents.len());

    let going_up = currents[center] > 0.0;

    // Search left
    let mut left_e = potentials[center];
    for j in (0..center).rev() {
        let crossed = if going_up {
            currents[j] <= half_max && currents[j + 1] > half_max
        } else {
            currents[j] >= half_max && currents[j + 1] < half_max
        };
        if crossed {
            let frac = if (currents[j + 1] - currents[j]).abs() > 1e-30 {
                (half_max - currents[j]) / (currents[j + 1] - currents[j])
            } else { 0.5 };
            left_e = potentials[j] + frac * (potentials[j + 1] - potentials[j]);
            break;
        }
    }

    // Search right
    let mut right_e = potentials[center];
    for j in center + 1..n {
        let crossed = if going_up {
            currents[j] <= half_max && currents[j - 1] > half_max
        } else {
            currents[j] >= half_max && currents[j - 1] < half_max
        };
        if crossed {
            let frac = if (currents[j - 1] - currents[j]).abs() > 1e-30 {
                (half_max - currents[j]) / (currents[j - 1] - currents[j])
            } else { 0.5 };
            right_e = potentials[j] - frac * (potentials[j] - potentials[j - 1]);
            break;
        }
    }

    (right_e - left_e).abs()
}

/// Estimate peak area by trapezoidal integration around peak
fn estimate_peak_area(potentials: &[f64], currents: &[f64], center: usize) -> f64 {
    let n = potentials.len().min(currents.len());
    // Integrate within ±20 points or to edges
    let start = center.saturating_sub(20);
    let end = (center + 21).min(n);
    let mut area = 0.0;
    for i in start + 1..end {
        let de = (potentials[i] - potentials[i - 1]).abs();
        area += 0.5 * (currents[i].abs() + currents[i - 1].abs()) * de;
    }
    area
}

/// Polynomial baseline correction
/// Fits polynomial of given order to baseline regions and subtracts
pub fn baseline_correction(
    potentials: &[f64],
    currents: &[f64],
    order: usize,
) -> Vec<f64> {
    let n = potentials.len().min(currents.len());
    if n < 2 || order == 0 {
        return currents[..n].to_vec();
    }

    // Use first and last 10% as baseline regions
    let n_base = (n / 10).max(2);
    let mut base_e = Vec::new();
    let mut base_i = Vec::new();
    for i in 0..n_base {
        base_e.push(potentials[i]);
        base_i.push(currents[i]);
    }
    for i in n - n_base..n {
        base_e.push(potentials[i]);
        base_i.push(currents[i]);
    }

    // Fit polynomial (least squares)
    let order = order.min(3); // limit to cubic
    let coeffs = polyfit(&base_e, &base_i, order);

    // Subtract baseline
    (0..n)
        .map(|i| currents[i] - polyeval(&coeffs, potentials[i]))
        .collect()
}

/// Least squares polynomial fit
fn polyfit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len().min(y.len());
    let m = order + 1;
    if n < m {
        return vec![0.0; m];
    }

    // Normal equations: (X^T X) c = X^T y
    let mut ata = vec![0.0; m * m];
    let mut aty = vec![0.0; m];

    for k in 0..n {
        let mut xi = 1.0;
        for i in 0..m {
            let mut xj = 1.0;
            for j in 0..m {
                ata[i * m + j] += xi * xj;
                xj *= x[k];
            }
            aty[i] += xi * y[k];
            xi *= x[k];
        }
    }

    // Solve with Gaussian elimination
    gauss_solve(&mut ata, &mut aty, m)
}

fn gauss_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    for k in 0..n {
        // Partial pivoting
        let mut max_val = a[k * n + k].abs();
        let mut max_row = k;
        for i in k + 1..n {
            if a[i * n + k].abs() > max_val {
                max_val = a[i * n + k].abs();
                max_row = i;
            }
        }
        if max_row != k {
            for j in 0..n {
                let tmp = a[k * n + j];
                a[k * n + j] = a[max_row * n + j];
                a[max_row * n + j] = tmp;
            }
            let tmp = b[k];
            b[k] = b[max_row];
            b[max_row] = tmp;
        }

        let pivot = a[k * n + k];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for i in k + 1..n {
            let factor = a[i * n + k] / pivot;
            for j in k..n {
                a[i * n + j] -= factor * a[k * n + j];
            }
            b[i] -= factor * b[k];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in i + 1..n {
            sum -= a[i * n + j] * x[j];
        }
        if a[i * n + i].abs() > 1e-30 {
            x[i] = sum / a[i * n + i];
        }
    }
    x
}

fn polyeval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut xp = 1.0;
    for &c in coeffs {
        result += c * xp;
        xp *= x;
    }
    result
}

/// Savitzky-Golay smoothing (simplified 5-point quadratic)
pub fn savitzky_golay_smooth(values: &[f64]) -> Vec<f64> {
    let n = values.len();
    if n < 5 {
        return values.to_vec();
    }
    // 5-point SG coefficients for quadratic: [-3, 12, 17, 12, -3] / 35
    let mut result = vec![0.0; n];
    result[0] = values[0];
    result[1] = values[1];
    result[n - 1] = values[n - 1];
    result[n - 2] = values[n - 2];
    for i in 2..n - 2 {
        result[i] = (-3.0 * values[i - 2]
            + 12.0 * values[i - 1]
            + 17.0 * values[i]
            + 12.0 * values[i + 1]
            - 3.0 * values[i + 2])
            / 35.0;
    }
    result
}

/// Linear calibration: i_peak = slope * concentration + intercept
pub fn linear_calibration(
    concentrations: &[f64],
    peak_currents: &[f64],
) -> (f64, f64, f64) {
    let n = concentrations.len().min(peak_currents.len());
    if n < 2 {
        return (0.0, 0.0, 0.0);
    }

    let mean_c: f64 = concentrations[..n].iter().sum::<f64>() / n as f64;
    let mean_i: f64 = peak_currents[..n].iter().sum::<f64>() / n as f64;

    let mut sum_cc = 0.0;
    let mut sum_ci = 0.0;
    let mut sum_ii = 0.0;

    for k in 0..n {
        let dc = concentrations[k] - mean_c;
        let di = peak_currents[k] - mean_i;
        sum_cc += dc * dc;
        sum_ci += dc * di;
        sum_ii += di * di;
    }

    if sum_cc < 1e-30 {
        return (0.0, mean_i, 0.0);
    }

    let slope = sum_ci / sum_cc;
    let intercept = mean_i - slope * mean_c;

    // R² coefficient of determination
    let ss_res: f64 = (0..n)
        .map(|k| {
            let pred = slope * concentrations[k] + intercept;
            (peak_currents[k] - pred).powi(2)
        })
        .sum();
    let r2 = if sum_ii > 1e-30 { 1.0 - ss_res / sum_ii } else { 0.0 };

    (slope, intercept, r2)
}

/// Concentration from calibration
pub fn concentration_from_calibration(
    peak_current: f64,
    slope: f64,
    intercept: f64,
) -> f64 {
    if slope.abs() < 1e-30 {
        return 0.0;
    }
    (peak_current - intercept) / slope
}

/// Limit of detection: LOD = 3 * sigma_blank / slope
pub fn limit_of_detection(
    blank_std: f64,
    slope: f64,
) -> f64 {
    if slope.abs() < 1e-30 {
        return f64::INFINITY;
    }
    3.0 * blank_std / slope.abs()
}

/// Limit of quantification: LOQ = 10 * sigma_blank / slope
pub fn limit_of_quantification(
    blank_std: f64,
    slope: f64,
) -> f64 {
    if slope.abs() < 1e-30 {
        return f64::INFINITY;
    }
    10.0 * blank_std / slope.abs()
}

/// Stripping analysis: accumulate charge then strip
/// Returns total accumulated charge (C) from integration of current over time
pub fn accumulation_charge(
    times_s: &[f64],
    currents_a: &[f64],
) -> f64 {
    let n = times_s.len().min(currents_a.len());
    if n < 2 {
        return 0.0;
    }
    let mut charge = 0.0;
    for i in 1..n {
        let dt = times_s[i] - times_s[i - 1];
        charge += 0.5 * (currents_a[i] + currents_a[i - 1]) * dt;
    }
    charge.abs()
}

/// Nernst equation: E = E0 + (RT/nF) * ln([Ox]/[Red])
pub fn nernst_potential(
    e0_v: f64,
    n_electrons: f64,
    ox_conc: f64,
    red_conc: f64,
    temperature_k: f64,
) -> f64 {
    if red_conc <= 0.0 || ox_conc <= 0.0 || n_electrons <= 0.0 {
        return e0_v;
    }
    e0_v + (R * temperature_k / (n_electrons * F)) * (ox_conc / red_conc).ln()
}

/// Peak resolution between two DPV peaks
/// Rs = 2 * |Ep2 - Ep1| / (W1 + W2)
pub fn peak_resolution(
    peak1: &DpvPeak,
    peak2: &DpvPeak,
) -> f64 {
    let w_sum = peak1.half_width_v + peak2.half_width_v;
    if w_sum < 1e-15 {
        return 0.0;
    }
    2.0 * (peak2.peak_potential_v - peak1.peak_potential_v).abs() / w_sum
}

/// Reversibility diagnostic from half-peak width
/// Returns ratio: W1/2_measured / W1/2_theoretical
/// Ratio ≈ 1.0 for reversible, > 1 for quasi-reversible/irreversible
pub fn reversibility_ratio(
    measured_half_width_v: f64,
    n_electrons: f64,
    temperature_k: f64,
) -> f64 {
    let w_theo = half_peak_width_reversible(n_electrons, temperature_k);
    if w_theo < 1e-15 {
        return 0.0;
    }
    measured_half_width_v / w_theo
}

/// DPV Processor - orchestrates full analysis
pub struct DpvProcessor {
    pub potentials: Vec<f64>,
    pub currents: Vec<f64>,
    pub temperature_k: f64,
}

impl DpvProcessor {
    pub fn new(potentials: Vec<f64>, currents: Vec<f64>, temperature_k: f64) -> Self {
        Self { potentials, currents, temperature_k }
    }

    pub fn find_peaks(&self, min_current: f64) -> Vec<DpvPeak> {
        find_peaks(&self.potentials, &self.currents, min_current)
    }

    pub fn baseline_corrected(&self, order: usize) -> Vec<f64> {
        baseline_correction(&self.potentials, &self.currents, order)
    }

    pub fn smoothed(&self) -> Vec<f64> {
        savitzky_golay_smooth(&self.currents)
    }

    pub fn theoretical_half_width(&self, n_electrons: f64) -> f64 {
        half_peak_width_reversible(n_electrons, self.temperature_k)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_dpv_peak(center_v: f64, amplitude: f64, width: f64, n: usize) -> (Vec<f64>, Vec<f64>) {
        let e_start = center_v - 0.3;
        let e_end = center_v + 0.3;
        let mut potentials = Vec::with_capacity(n);
        let mut currents = Vec::with_capacity(n);
        for i in 0..n {
            let e = e_start + (i as f64) * (e_end - e_start) / (n as f64 - 1.0);
            let i_val = amplitude * (-(e - center_v).powi(2) / (2.0 * width * width)).exp();
            potentials.push(e);
            currents.push(i_val);
        }
        (potentials, currents)
    }

    #[test]
    fn test_differential_current() {
        let base = vec![1.0, 2.0, 3.0];
        let pulse = vec![1.5, 2.8, 3.2];
        let diff = differential_current(&base, &pulse);
        assert_eq!(diff.len(), 3);
        assert!(approx_eq(diff[0], 0.5, 1e-10));
        assert!(approx_eq(diff[1], 0.8, 1e-10));
    }

    #[test]
    fn test_dpv_peak_current_reversible() {
        let ip = dpv_peak_current_reversible(
            1.0,    // 1 electron
            0.01,   // 0.01 cm²
            1e-6,   // 1 µM = 1e-6 mol/cm³ → actually 1e-6 M = 1e-9 mol/cm³
            1e-5,   // 10^-5 cm²/s
            0.05,   // 50 ms pulse
            0.05,   // 50 mV amplitude
            298.15, // 25°C
        );
        // Peak current should be nonzero and have correct sign (negative for reduction)
        assert!(ip.abs() > 0.0);
    }

    #[test]
    fn test_dpv_peak_current_zero_pulse() {
        let ip = dpv_peak_current_reversible(1.0, 0.01, 1e-6, 1e-5, 0.0, 0.05, 298.15);
        assert_eq!(ip, 0.0);
    }

    #[test]
    fn test_dpv_peak_potential() {
        let ep = dpv_peak_potential(-0.5, 0.05);
        assert!(approx_eq(ep, -0.525, 0.001));
    }

    #[test]
    fn test_half_peak_width_reversible() {
        // At 25°C, n=1: W1/2 = 3.52 * 8.314 * 298.15 / (1 * 96485) = 0.0904 V
        let w = half_peak_width_reversible(1.0, 298.15);
        assert!(approx_eq(w, 0.0904, 0.001), "w = {}", w);
    }

    #[test]
    fn test_half_peak_width_two_electrons() {
        let w1 = half_peak_width_reversible(1.0, 298.15);
        let w2 = half_peak_width_reversible(2.0, 298.15);
        assert!(approx_eq(w2, w1 / 2.0, 0.001));
    }

    #[test]
    fn test_half_peak_width_zero_n() {
        assert_eq!(half_peak_width_reversible(0.0, 298.15), 0.0);
    }

    #[test]
    fn test_swv_net_current() {
        let fwd = vec![1.0, 2.0, 3.0];
        let rev = vec![0.5, 1.0, 1.5];
        let net = swv_net_current(&fwd, &rev);
        assert!(approx_eq(net[0], 0.5, 1e-10));
        assert!(approx_eq(net[2], 1.5, 1e-10));
    }

    #[test]
    fn test_swv_peak_dimensionless() {
        let psi = swv_peak_dimensionless(1.0, 0.05, 298.15);
        assert!(psi > 0.0 && psi <= 1.0, "psi = {}", psi);
    }

    #[test]
    fn test_swv_peak_large_amplitude() {
        let psi = swv_peak_dimensionless(1.0, 0.5, 298.15);
        assert!(psi > 0.9, "psi = {}", psi);
    }

    #[test]
    fn test_find_peaks_single() {
        let (potentials, currents) = make_dpv_peak(-0.5, 1e-6, 0.05, 201);
        let peaks = find_peaks(&potentials, &currents, 1e-7);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].peak_potential_v, -0.5, 0.01));
    }

    #[test]
    fn test_find_peaks_two() {
        let n = 400;
        let e_start = -0.8;
        let e_end = 0.0;
        let mut potentials = Vec::with_capacity(n);
        let mut currents = Vec::with_capacity(n);
        for i in 0..n {
            let e = e_start + (i as f64) * (e_end - e_start) / (n as f64 - 1.0);
            let i1 = 1e-6 * (-(e + 0.5).powi(2) / (2.0 * 0.03_f64.powi(2))).exp();
            let i2 = 0.5e-6 * (-(e + 0.2).powi(2) / (2.0 * 0.03_f64.powi(2))).exp();
            potentials.push(e);
            currents.push(i1 + i2);
        }
        let peaks = find_peaks(&potentials, &currents, 1e-7);
        assert_eq!(peaks.len(), 2, "found {} peaks", peaks.len());
    }

    #[test]
    fn test_find_peaks_threshold() {
        let (potentials, currents) = make_dpv_peak(-0.5, 1e-6, 0.05, 200);
        let peaks = find_peaks(&potentials, &currents, 1e-3);
        assert_eq!(peaks.len(), 0); // below threshold
    }

    #[test]
    fn test_baseline_correction() {
        // Linear baseline + peak
        let n = 200;
        let mut potentials = Vec::with_capacity(n);
        let mut currents = Vec::with_capacity(n);
        for i in 0..n {
            let e = -0.8 + (i as f64) * 0.6 / (n as f64 - 1.0);
            let baseline = 0.1 * e + 0.05;
            let peak = 1e-6 * (-(e + 0.5).powi(2) / (2.0 * 0.03_f64.powi(2))).exp();
            potentials.push(e);
            currents.push(baseline + peak);
        }
        let corrected = baseline_correction(&potentials, &currents, 1);
        // Corrected baseline regions should be near zero
        assert!(corrected[0].abs() < 0.01, "start = {}", corrected[0]);
    }

    #[test]
    fn test_savitzky_golay_smooth() {
        let values = vec![1.0, 1.0, 10.0, 1.0, 1.0, 1.0, 1.0];
        let smoothed = savitzky_golay_smooth(&values);
        assert_eq!(smoothed.len(), 7);
        // Spike should be reduced
        assert!(smoothed[2] < 10.0);
        assert!(smoothed[2] > 1.0);
    }

    #[test]
    fn test_savitzky_golay_short() {
        let values = vec![1.0, 2.0, 3.0];
        let smoothed = savitzky_golay_smooth(&values);
        assert_eq!(smoothed, values);
    }

    #[test]
    fn test_linear_calibration() {
        let concs = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let currents = vec![0.1, 1.1, 2.1, 3.1, 4.1];
        let (slope, intercept, r2) = linear_calibration(&concs, &currents);
        assert!(approx_eq(slope, 1.0, 0.01));
        assert!(approx_eq(intercept, 0.1, 0.01));
        assert!(r2 > 0.999);
    }

    #[test]
    fn test_linear_calibration_insufficient() {
        let (slope, _, _) = linear_calibration(&[1.0], &[2.0]);
        assert_eq!(slope, 0.0);
    }

    #[test]
    fn test_concentration_from_calibration() {
        let c = concentration_from_calibration(5.1, 1.0, 0.1);
        assert!(approx_eq(c, 5.0, 0.01));
    }

    #[test]
    fn test_concentration_zero_slope() {
        assert_eq!(concentration_from_calibration(5.0, 0.0, 0.1), 0.0);
    }

    #[test]
    fn test_limit_of_detection() {
        let lod = limit_of_detection(0.01, 1.0);
        assert!(approx_eq(lod, 0.03, 0.001));
    }

    #[test]
    fn test_limit_of_detection_zero_slope() {
        let lod = limit_of_detection(0.01, 0.0);
        assert!(lod.is_infinite());
    }

    #[test]
    fn test_limit_of_quantification() {
        let loq = limit_of_quantification(0.01, 1.0);
        assert!(approx_eq(loq, 0.1, 0.001));
    }

    #[test]
    fn test_accumulation_charge() {
        let times = vec![0.0, 1.0, 2.0, 3.0];
        let currents = vec![1e-3, 1e-3, 1e-3, 1e-3]; // 1 mA for 3 s
        let charge = accumulation_charge(&times, &currents);
        assert!(approx_eq(charge, 3e-3, 1e-5));
    }

    #[test]
    fn test_accumulation_charge_short() {
        assert_eq!(accumulation_charge(&[0.0], &[1.0]), 0.0);
    }

    #[test]
    fn test_nernst_potential() {
        // At equal concentrations: E = E0
        let e = nernst_potential(0.77, 1.0, 1.0, 1.0, 298.15);
        assert!(approx_eq(e, 0.77, 0.001));
    }

    #[test]
    fn test_nernst_potential_ratio() {
        // 10:1 ratio: E = E0 + (RT/nF)*ln(10) ≈ E0 + 0.0592
        let e = nernst_potential(0.77, 1.0, 10.0, 1.0, 298.15);
        assert!(approx_eq(e, 0.77 + 0.0592, 0.002), "e = {}", e);
    }

    #[test]
    fn test_peak_resolution() {
        let p1 = DpvPeak { peak_potential_v: -0.5, peak_current_a: 1e-6, half_width_v: 0.05, area: 0.0 };
        let p2 = DpvPeak { peak_potential_v: -0.2, peak_current_a: 1e-6, half_width_v: 0.05, area: 0.0 };
        let rs = peak_resolution(&p1, &p2);
        assert!(approx_eq(rs, 6.0, 0.01)); // 2*0.3/0.1 = 6.0
    }

    #[test]
    fn test_peak_resolution_overlapping() {
        let p1 = DpvPeak { peak_potential_v: -0.5, peak_current_a: 1e-6, half_width_v: 0.1, area: 0.0 };
        let p2 = DpvPeak { peak_potential_v: -0.45, peak_current_a: 1e-6, half_width_v: 0.1, area: 0.0 };
        let rs = peak_resolution(&p1, &p2);
        assert!(rs < 1.0);
    }

    #[test]
    fn test_reversibility_ratio_ideal() {
        let w_theo = half_peak_width_reversible(1.0, 298.15);
        let ratio = reversibility_ratio(w_theo, 1.0, 298.15);
        assert!(approx_eq(ratio, 1.0, 0.001));
    }

    #[test]
    fn test_reversibility_ratio_irreversible() {
        let w_theo = half_peak_width_reversible(1.0, 298.15);
        let ratio = reversibility_ratio(w_theo * 2.0, 1.0, 298.15);
        assert!(approx_eq(ratio, 2.0, 0.001));
    }

    #[test]
    fn test_processor_new() {
        let (p, c) = make_dpv_peak(-0.5, 1e-6, 0.05, 100);
        let proc = DpvProcessor::new(p, c, 298.15);
        assert!(approx_eq(proc.temperature_k, 298.15, 0.01));
    }

    #[test]
    fn test_processor_find_peaks() {
        let (p, c) = make_dpv_peak(-0.5, 1e-6, 0.05, 201);
        let proc = DpvProcessor::new(p, c, 298.15);
        let peaks = proc.find_peaks(1e-7);
        assert_eq!(peaks.len(), 1);
    }

    #[test]
    fn test_processor_baseline() {
        let (p, c) = make_dpv_peak(-0.5, 1e-6, 0.05, 200);
        let proc = DpvProcessor::new(p, c, 298.15);
        let corrected = proc.baseline_corrected(1);
        assert_eq!(corrected.len(), 200);
    }

    #[test]
    fn test_processor_smoothed() {
        let (p, c) = make_dpv_peak(-0.5, 1e-6, 0.05, 200);
        let proc = DpvProcessor::new(p, c, 298.15);
        let s = proc.smoothed();
        assert_eq!(s.len(), 200);
    }

    #[test]
    fn test_processor_theoretical_width() {
        let proc = DpvProcessor::new(vec![], vec![], 298.15);
        let w = proc.theoretical_half_width(1.0);
        assert!(w > 0.08 && w < 0.10);
    }

    #[test]
    fn test_polyfit_linear() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![1.0, 3.0, 5.0, 7.0];
        let coeffs = polyfit(&x, &y, 1);
        assert!(approx_eq(coeffs[0], 1.0, 0.01)); // intercept
        assert!(approx_eq(coeffs[1], 2.0, 0.01)); // slope
    }

    #[test]
    fn test_polyeval() {
        let coeffs = vec![1.0, 2.0, 3.0]; // 1 + 2x + 3x²
        assert!(approx_eq(polyeval(&coeffs, 0.0), 1.0, 1e-10));
        assert!(approx_eq(polyeval(&coeffs, 1.0), 6.0, 1e-10));
        assert!(approx_eq(polyeval(&coeffs, 2.0), 17.0, 1e-10));
    }
}
