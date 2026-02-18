// capillary_electrophoresis_processor.rs
//
// Capillary Electrophoresis (CE) signal processing for separation science.
// Electrophoretic mobility, plate count, resolution, EOF correction, peak deconvolution.

/// An electropherogram: time vs detector response.
#[derive(Debug, Clone)]
pub struct Electropherogram {
    pub time_min: Vec<f64>,
    pub signal: Vec<f64>,
}

/// A detected peak in an electropherogram.
#[derive(Debug, Clone)]
pub struct CePeak {
    pub index: usize,
    pub time_min: f64,
    pub height: f64,
    pub area: f64,
    pub width_half: f64,
    pub asymmetry: f64,
}

/// CE system parameters.
#[derive(Debug, Clone)]
pub struct CeParams {
    pub total_length_cm: f64,
    pub effective_length_cm: f64,
    pub voltage_kv: f64,
    pub capillary_id_um: f64,
    pub temperature_c: f64,
}

impl CeParams {
    pub fn new(total_cm: f64, effective_cm: f64, voltage_kv: f64) -> Self {
        Self {
            total_length_cm: total_cm,
            effective_length_cm: effective_cm,
            voltage_kv,
            capillary_id_um: 50.0,
            temperature_c: 25.0,
        }
    }

    /// Electric field strength (V/cm).
    pub fn field_strength(&self) -> f64 {
        self.voltage_kv * 1000.0 / self.total_length_cm
    }
}

impl Electropherogram {
    pub fn new(time_min: Vec<f64>, signal: Vec<f64>) -> Self {
        Self { time_min, signal }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.time_min.len().min(self.signal.len())
    }

    /// Find peaks above threshold with simple local maximum detection.
    pub fn find_peaks(&self, threshold: f64) -> Vec<CePeak> {
        let n = self.len();
        if n < 3 { return vec![]; }
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if self.signal[i] > threshold
                && self.signal[i] >= self.signal[i - 1]
                && self.signal[i] >= self.signal[i + 1]
            {
                let half = self.signal[i] * 0.5;
                let w_half = estimate_width_at_half(
                    &self.time_min, &self.signal, i, half,
                );
                let area = estimate_peak_area(
                    &self.time_min, &self.signal, i,
                );
                let asym = peak_asymmetry(
                    &self.time_min, &self.signal, i, half,
                );
                peaks.push(CePeak {
                    index: i,
                    time_min: self.time_min[i],
                    height: self.signal[i],
                    area,
                    width_half: w_half,
                    asymmetry: asym,
                });
            }
        }
        peaks
    }

    /// Baseline subtraction using linear interpolation between endpoints.
    pub fn subtract_baseline(&mut self) {
        let n = self.len();
        if n < 2 { return; }
        let s0 = self.signal[0];
        let s1 = self.signal[n - 1];
        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            self.signal[i] -= s0 + frac * (s1 - s0);
        }
    }
}

// ---------------------------------------------------------------------------
// Width, area, asymmetry helpers
// ---------------------------------------------------------------------------

fn estimate_width_at_half(
    time: &[f64], sig: &[f64], peak_idx: usize, half_max: f64,
) -> f64 {
    let n = time.len();
    let mut left = time[0];
    for i in (0..peak_idx).rev() {
        if sig[i] <= half_max {
            let frac = (half_max - sig[i]) / (sig[i + 1] - sig[i]).max(1e-30);
            left = time[i] + frac * (time[i + 1] - time[i]);
            break;
        }
    }
    let mut right = time[n - 1];
    for i in peak_idx + 1..n {
        if sig[i] <= half_max {
            let frac = (half_max - sig[i]) / (sig[i - 1] - sig[i]).max(1e-30);
            right = time[i] - frac * (time[i] - time[i - 1]);
            break;
        }
    }
    (right - left).abs()
}

fn estimate_peak_area(time: &[f64], sig: &[f64], peak_idx: usize) -> f64 {
    let n = time.len();
    let start = if peak_idx >= 5 { peak_idx - 5 } else { 0 };
    let end = if peak_idx + 5 < n { peak_idx + 5 } else { n - 1 };
    let mut area = 0.0;
    for i in start..end {
        let dt = time[i + 1] - time[i];
        area += 0.5 * (sig[i] + sig[i + 1]) * dt;
    }
    area
}

fn peak_asymmetry(
    time: &[f64], sig: &[f64], peak_idx: usize, level: f64,
) -> f64 {
    let t_peak = time[peak_idx];
    let mut left_t = t_peak;
    for i in (0..peak_idx).rev() {
        if sig[i] <= level {
            let frac = (level - sig[i]) / (sig[i + 1] - sig[i]).max(1e-30);
            left_t = time[i] + frac * (time[i + 1] - time[i]);
            break;
        }
    }
    let n = time.len();
    let mut right_t = t_peak;
    for i in peak_idx + 1..n {
        if sig[i] <= level {
            let frac = (level - sig[i]) / (sig[i - 1] - sig[i]).max(1e-30);
            right_t = time[i] - frac * (time[i] - time[i - 1]);
            break;
        }
    }
    let a = t_peak - left_t;
    let b = right_t - t_peak;
    if a.abs() < 1e-30 { return 1.0; }
    b / a
}

// ---------------------------------------------------------------------------
// 2. Electrophoretic Mobility
// ---------------------------------------------------------------------------

/// Electrophoretic mobility: μ_ep = (L_d * L_t) / (V * t_m).
/// L_d = effective length (cm), L_t = total length (cm), V = voltage (V), t_m = migration time (s).
/// Returns cm²/(V·s).
pub fn electrophoretic_mobility(
    effective_length_cm: f64,
    total_length_cm: f64,
    voltage_v: f64,
    migration_time_s: f64,
) -> f64 {
    if voltage_v.abs() < 1e-30 || migration_time_s.abs() < 1e-30 {
        return 0.0;
    }
    effective_length_cm * total_length_cm / (voltage_v * migration_time_s)
}

/// Apparent mobility (includes EOF).
pub fn apparent_mobility(
    effective_length_cm: f64,
    total_length_cm: f64,
    voltage_v: f64,
    migration_time_s: f64,
) -> f64 {
    electrophoretic_mobility(effective_length_cm, total_length_cm, voltage_v, migration_time_s)
}

/// EOF mobility from neutral marker migration time.
pub fn eof_mobility(
    effective_length_cm: f64,
    total_length_cm: f64,
    voltage_v: f64,
    eof_time_s: f64,
) -> f64 {
    electrophoretic_mobility(effective_length_cm, total_length_cm, voltage_v, eof_time_s)
}

/// Effective (true) electrophoretic mobility = apparent - EOF.
pub fn effective_mobility(apparent: f64, eof: f64) -> f64 {
    apparent - eof
}

/// Corrected migration time: t_corr = t_m / t_eof (EOF-normalized).
pub fn corrected_migration_time(t_migration: f64, t_eof: f64) -> f64 {
    if t_eof.abs() < 1e-30 { return t_migration; }
    t_migration / t_eof
}

// ---------------------------------------------------------------------------
// 3. Plate Count and Efficiency
// ---------------------------------------------------------------------------

/// Theoretical plate count: N = 5.54 * (t_m / w_half)^2.
pub fn plate_count(migration_time: f64, width_half: f64) -> f64 {
    if width_half.abs() < 1e-30 { return 0.0; }
    5.54 * (migration_time / width_half).powi(2)
}

/// Height equivalent to a theoretical plate: HETP = L / N (cm).
pub fn hetp(effective_length_cm: f64, plates: f64) -> f64 {
    if plates.abs() < 1e-30 { return f64::INFINITY; }
    effective_length_cm / plates
}

/// Plates per meter.
pub fn plates_per_meter(effective_length_cm: f64, plates: f64) -> f64 {
    if effective_length_cm.abs() < 1e-30 { return 0.0; }
    plates / (effective_length_cm / 100.0)
}

// ---------------------------------------------------------------------------
// 4. Resolution
// ---------------------------------------------------------------------------

/// Resolution between two peaks: R = 2*(t2-t1) / (w1+w2).
pub fn resolution(t1: f64, t2: f64, w1: f64, w2: f64) -> f64 {
    let denom = w1 + w2;
    if denom.abs() < 1e-30 { return 0.0; }
    2.0 * (t2 - t1).abs() / denom
}

/// Selectivity factor: α = μ2/μ1 (where μ2 > μ1).
pub fn selectivity_factor(mu1: f64, mu2: f64) -> f64 {
    if mu1.abs() < 1e-30 { return f64::INFINITY; }
    (mu2 / mu1).abs()
}

/// Resolution from N, α, and effective mobility ratio.
/// R = sqrt(N)/4 * (α-1)/α * μ_eff / (μ_eff + μ_eof)
pub fn resolution_from_efficiency(
    plates: f64, alpha: f64, mu_eff: f64, mu_eof: f64,
) -> f64 {
    if alpha.abs() < 1e-30 { return 0.0; }
    let mu_sum = mu_eff.abs() + mu_eof.abs();
    if mu_sum < 1e-30 { return 0.0; }
    plates.sqrt() / 4.0 * (alpha - 1.0) / alpha * mu_eff.abs() / mu_sum
}

// ---------------------------------------------------------------------------
// 5. Joule Heating
// ---------------------------------------------------------------------------

/// Joule heating power: P = I * V (watts).
pub fn joule_heating_power(current_ua: f64, voltage_kv: f64) -> f64 {
    (current_ua * 1e-6) * (voltage_kv * 1e3)
}

/// Power per unit length (W/m).
pub fn power_per_length(current_ua: f64, voltage_kv: f64, length_cm: f64) -> f64 {
    let p = joule_heating_power(current_ua, voltage_kv);
    if length_cm.abs() < 1e-30 { return 0.0; }
    p / (length_cm / 100.0)
}

/// Estimated temperature rise (°C) from Joule heating.
/// ΔT ≈ P / (h * A_surface) where h ~50 W/(m²·K) for air cooling.
pub fn temperature_rise_estimate(
    current_ua: f64, voltage_kv: f64, length_cm: f64, od_um: f64,
) -> f64 {
    let p = joule_heating_power(current_ua, voltage_kv);
    let circumference = std::f64::consts::PI * od_um * 1e-6;
    let area = circumference * length_cm / 100.0;
    let h = 50.0; // air convection W/(m²·K)
    if area < 1e-30 { return 0.0; }
    p / (h * area)
}

// ---------------------------------------------------------------------------
// 6. Injection Volume
// ---------------------------------------------------------------------------

/// Hydrodynamic injection volume (Poiseuille): V = ΔP * π * d⁴ * t / (128 * η * L).
/// Returns volume in nL.
pub fn hydrodynamic_injection_nl(
    pressure_mbar: f64,
    id_um: f64,
    injection_time_s: f64,
    viscosity_cp: f64,
    total_length_cm: f64,
) -> f64 {
    let d_m = id_um * 1e-6;
    let l_m = total_length_cm / 100.0;
    let eta = viscosity_cp * 1e-3;
    let dp = pressure_mbar * 100.0; // Pa
    let pi = std::f64::consts::PI;
    let vol_m3 = dp * pi * d_m.powi(4) * injection_time_s / (128.0 * eta * l_m);
    vol_m3 * 1e12 // m³ to nL
}

/// Electrokinetic injection volume (approximate).
/// V ≈ μ_app * E * π * r² * t
pub fn electrokinetic_injection_nl(
    mu_app_cm2_vs: f64,
    voltage_kv: f64,
    total_length_cm: f64,
    id_um: f64,
    injection_time_s: f64,
) -> f64 {
    let e_field = voltage_kv * 1000.0 / total_length_cm; // V/cm
    let r_cm = id_um * 1e-4 / 2.0;
    let pi = std::f64::consts::PI;
    let vol_cm3 = mu_app_cm2_vs * e_field * pi * r_cm * r_cm * injection_time_s;
    vol_cm3 * 1e6 // cm³ to nL
}

// ---------------------------------------------------------------------------
// 7. Gaussian Peak Fitting / Deconvolution
// ---------------------------------------------------------------------------

/// Gaussian peak function.
pub fn gaussian_peak(t: f64, center: f64, height: f64, sigma: f64) -> f64 {
    height * (-0.5 * ((t - center) / sigma).powi(2)).exp()
}

/// Sum of Gaussian peaks.
pub fn multi_gaussian(t: f64, params: &[(f64, f64, f64)]) -> Vec<f64> {
    let mut result = Vec::with_capacity(params.len());
    for &(center, height, sigma) in params {
        result.push(gaussian_peak(t, center, height, sigma));
    }
    result
}

/// Simple two-peak Gaussian deconvolution by grid search.
/// Returns (center1, height1, sigma1, center2, height2, sigma2).
pub fn deconvolve_two_peaks(
    time: &[f64], signal: &[f64],
    t1_guess: f64, t2_guess: f64,
) -> (f64, f64, f64, f64, f64, f64) {
    let n = time.len().min(signal.len());
    if n < 5 { return (t1_guess, 1.0, 0.1, t2_guess, 1.0, 0.1); }
    let max_sig = signal.iter().cloned().fold(0.0_f64, f64::max);
    let dt = (time[n - 1] - time[0]) / n as f64;
    let mut best_err = f64::MAX;
    let mut best = (t1_guess, max_sig, dt * 2.0, t2_guess, max_sig, dt * 2.0);
    // Grid search over sigma and heights
    for s1_i in 1..10 {
        let sigma1 = dt * s1_i as f64;
        for s2_i in 1..10 {
            let sigma2 = dt * s2_i as f64;
            // Solve for heights by least squares at fixed sigma/center
            let (h1, h2) = solve_heights(time, signal, n, t1_guess, sigma1, t2_guess, sigma2);
            if h1 < 0.0 || h2 < 0.0 { continue; }
            let err: f64 = (0..n).map(|i| {
                let model = gaussian_peak(time[i], t1_guess, h1, sigma1)
                    + gaussian_peak(time[i], t2_guess, h2, sigma2);
                (signal[i] - model).powi(2)
            }).sum();
            if err < best_err {
                best_err = err;
                best = (t1_guess, h1, sigma1, t2_guess, h2, sigma2);
            }
        }
    }
    best
}

fn solve_heights(
    time: &[f64], signal: &[f64], n: usize,
    c1: f64, s1: f64, c2: f64, s2: f64,
) -> (f64, f64) {
    let mut a11 = 0.0_f64;
    let mut a12 = 0.0_f64;
    let mut a22 = 0.0_f64;
    let mut b1 = 0.0_f64;
    let mut b2 = 0.0_f64;
    for i in 0..n {
        let g1 = (-0.5 * ((time[i] - c1) / s1).powi(2)).exp();
        let g2 = (-0.5 * ((time[i] - c2) / s2).powi(2)).exp();
        a11 += g1 * g1;
        a12 += g1 * g2;
        a22 += g2 * g2;
        b1 += signal[i] * g1;
        b2 += signal[i] * g2;
    }
    let det = a11 * a22 - a12 * a12;
    if det.abs() < 1e-30 { return (0.0, 0.0); }
    let h1 = (a22 * b1 - a12 * b2) / det;
    let h2 = (a11 * b2 - a12 * b1) / det;
    (h1, h2)
}

// ---------------------------------------------------------------------------
// 8. Charge-to-Mass Estimation
// ---------------------------------------------------------------------------

/// Estimate charge-to-size ratio from effective mobility.
/// For a sphere: μ = q / (6πηr) → q/m ∝ μ * η / r_h
/// This gives a qualitative charge estimate.
pub fn charge_from_mobility(mu_eff_cm2_vs: f64, viscosity_cp: f64) -> f64 {
    let eta = viscosity_cp * 1e-3; // Pa·s
    let pi = std::f64::consts::PI;
    // μ = q / (6πηr), but without r we return μ * 6πη as proxy
    mu_eff_cm2_vs.abs() * 1e-4 * 6.0 * pi * eta
}

/// Stokes radius from mobility: r = q / (6πημ).
/// For singly charged ions (q = e).
pub fn stokes_radius_nm(mu_eff_cm2_vs: f64, viscosity_cp: f64) -> f64 {
    let e = 1.602e-19; // C
    let eta = viscosity_cp * 1e-3;
    let pi = std::f64::consts::PI;
    let mu_si = mu_eff_cm2_vs.abs() * 1e-4; // m²/(V·s)
    if mu_si < 1e-30 { return 0.0; }
    let r = e / (6.0 * pi * eta * mu_si);
    r * 1e9 // m to nm
}

// ---------------------------------------------------------------------------
// 9. CE Processor
// ---------------------------------------------------------------------------

/// Main processor combining CE analysis steps.
pub struct CeProcessor {
    pub params: CeParams,
    pub eof_time_min: Option<f64>,
    pub peaks: Vec<CePeak>,
    pub mobilities: Vec<f64>,
}

impl CeProcessor {
    pub fn new(params: CeParams) -> Self {
        Self { params, eof_time_min: None, peaks: vec![], mobilities: vec![] }
    }

    /// Set EOF marker time.
    pub fn set_eof(&mut self, eof_time_min: f64) {
        self.eof_time_min = Some(eof_time_min);
    }

    /// Analyze an electropherogram.
    pub fn analyze(&mut self, epg: &Electropherogram, threshold: f64) {
        self.peaks = epg.find_peaks(threshold);
        self.mobilities.clear();
        let v_v = self.params.voltage_kv * 1000.0;
        for p in &self.peaks {
            let t_s = p.time_min * 60.0;
            let mu_app = apparent_mobility(
                self.params.effective_length_cm,
                self.params.total_length_cm,
                v_v, t_s,
            );
            if let Some(eof_t) = self.eof_time_min {
                let mu_eof = eof_mobility(
                    self.params.effective_length_cm,
                    self.params.total_length_cm,
                    v_v, eof_t * 60.0,
                );
                self.mobilities.push(effective_mobility(mu_app, mu_eof));
            } else {
                self.mobilities.push(mu_app);
            }
        }
    }

    /// Get plate count for peak at index.
    pub fn plates_for_peak(&self, idx: usize) -> f64 {
        if idx >= self.peaks.len() { return 0.0; }
        plate_count(self.peaks[idx].time_min, self.peaks[idx].width_half)
    }

    /// Resolution between two adjacent peaks.
    pub fn resolution_between(&self, i: usize, j: usize) -> f64 {
        if i >= self.peaks.len() || j >= self.peaks.len() { return 0.0; }
        resolution(
            self.peaks[i].time_min, self.peaks[j].time_min,
            self.peaks[i].width_half, self.peaks[j].width_half,
        )
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

#[cfg(test)]
mod tests {
    use super::*;

    fn make_gaussian_epg(center: f64, sigma: f64, n: usize) -> Electropherogram {
        let mut time = Vec::with_capacity(n);
        let mut sig = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 * 0.01;
            time.push(t);
            sig.push(gaussian_peak(t, center, 1.0, sigma));
        }
        Electropherogram::new(time, sig)
    }

    #[test]
    fn test_electrophoretic_mobility() {
        // 40 cm effective, 50 cm total, 20 kV, 300 s migration
        let mu = electrophoretic_mobility(40.0, 50.0, 20000.0, 300.0);
        // 40*50/(20000*300) = 2000/6000000 = 3.33e-4 cm²/(V·s)
        assert!(approx_eq(mu, 3.333e-4, 1e-5));
    }

    #[test]
    fn test_effective_mobility() {
        let mu_app = 4.0e-4;
        let mu_eof = 6.0e-4;
        let mu_eff = effective_mobility(mu_app, mu_eof);
        assert!(approx_eq(mu_eff, -2.0e-4, 1e-6));
    }

    #[test]
    fn test_corrected_migration_time() {
        let t_corr = corrected_migration_time(10.0, 5.0);
        assert!(approx_eq(t_corr, 2.0, 0.01));
    }

    #[test]
    fn test_plate_count() {
        let n = plate_count(5.0, 0.1);
        // 5.54 * (50)^2 = 13850
        assert!(approx_eq(n, 13850.0, 1.0));
    }

    #[test]
    fn test_plate_count_wide_peak() {
        let n = plate_count(10.0, 1.0);
        // 5.54 * (10)^2 = 554
        assert!(approx_eq(n, 554.0, 1.0));
    }

    #[test]
    fn test_hetp() {
        let h = hetp(40.0, 10000.0);
        assert!(approx_eq(h, 0.004, 0.0001));
    }

    #[test]
    fn test_plates_per_meter() {
        let ppm = plates_per_meter(50.0, 100000.0);
        assert!(approx_eq(ppm, 200000.0, 1.0));
    }

    #[test]
    fn test_resolution() {
        let r = resolution(5.0, 6.0, 0.2, 0.3);
        // 2*1.0 / 0.5 = 4.0
        assert!(approx_eq(r, 4.0, 0.01));
    }

    #[test]
    fn test_selectivity() {
        let alpha = selectivity_factor(2.0e-4, 3.0e-4);
        assert!(approx_eq(alpha, 1.5, 0.01));
    }

    #[test]
    fn test_resolution_from_efficiency() {
        let r = resolution_from_efficiency(10000.0, 1.1, 3.0e-4, 5.0e-4);
        assert!(r > 0.0);
    }

    #[test]
    fn test_joule_heating() {
        let p = joule_heating_power(50.0, 20.0);
        // 50e-6 * 20e3 = 1.0 W
        assert!(approx_eq(p, 1.0, 0.001));
    }

    #[test]
    fn test_power_per_length() {
        let ppl = power_per_length(50.0, 20.0, 50.0);
        // 1.0 W / 0.5 m = 2.0 W/m
        assert!(approx_eq(ppl, 2.0, 0.01));
    }

    #[test]
    fn test_temperature_rise() {
        let dt = temperature_rise_estimate(50.0, 20.0, 50.0, 375.0);
        assert!(dt > 0.0 && dt < 100.0);
    }

    #[test]
    fn test_hydrodynamic_injection() {
        let v = hydrodynamic_injection_nl(50.0, 50.0, 5.0, 1.0, 50.0);
        assert!(v > 0.0 && v < 100.0);
    }

    #[test]
    fn test_electrokinetic_injection() {
        let v = electrokinetic_injection_nl(3.0e-4, 10.0, 50.0, 50.0, 5.0);
        assert!(v > 0.0);
    }

    #[test]
    fn test_gaussian_peak_center() {
        let y = gaussian_peak(5.0, 5.0, 1.0, 0.1);
        assert!(approx_eq(y, 1.0, 1e-10));
    }

    #[test]
    fn test_gaussian_peak_off_center() {
        let y = gaussian_peak(5.1, 5.0, 1.0, 0.1);
        assert!(y < 1.0 && y > 0.0);
    }

    #[test]
    fn test_find_peaks_single() {
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        let peaks = epg.find_peaks(0.1);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].time_min, 1.0, 0.02));
    }

    #[test]
    fn test_find_peaks_two() {
        let mut time = Vec::new();
        let mut sig = Vec::new();
        for i in 0..500 {
            let t = i as f64 * 0.01;
            time.push(t);
            sig.push(gaussian_peak(t, 1.5, 1.0, 0.1) + gaussian_peak(t, 3.0, 0.8, 0.1));
        }
        let epg = Electropherogram::new(time, sig);
        let peaks = epg.find_peaks(0.1);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_peak_area_positive() {
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        let peaks = epg.find_peaks(0.1);
        assert!(peaks[0].area > 0.0);
    }

    #[test]
    fn test_peak_width() {
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        let peaks = epg.find_peaks(0.1);
        // FWHM ≈ 2.355 * sigma ≈ 0.118
        assert!(approx_eq(peaks[0].width_half, 0.118, 0.03));
    }

    #[test]
    fn test_peak_asymmetry_symmetric() {
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        let peaks = epg.find_peaks(0.1);
        assert!(approx_eq(peaks[0].asymmetry, 1.0, 0.2));
    }

    #[test]
    fn test_baseline_subtraction() {
        let mut epg = Electropherogram::new(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![1.0, 2.0, 3.0, 4.0],
        );
        epg.subtract_baseline();
        assert!(approx_eq(epg.signal[0], 0.0, 1e-10));
        assert!(approx_eq(epg.signal[3], 0.0, 1e-10));
    }

    #[test]
    fn test_charge_from_mobility() {
        let q = charge_from_mobility(3.0e-4, 1.0);
        assert!(q > 0.0);
    }

    #[test]
    fn test_stokes_radius() {
        let r = stokes_radius_nm(3.0e-4, 1.0);
        assert!(r > 0.0 && r < 10.0);
    }

    #[test]
    fn test_ce_processor_basic() {
        let params = CeParams::new(50.0, 40.0, 20.0);
        let mut proc = CeProcessor::new(params);
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        proc.analyze(&epg, 0.1);
        assert_eq!(proc.peaks.len(), 1);
        assert_eq!(proc.mobilities.len(), 1);
    }

    #[test]
    fn test_ce_processor_with_eof() {
        let params = CeParams::new(50.0, 40.0, 20.0);
        let mut proc = CeProcessor::new(params);
        proc.set_eof(0.5);
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        proc.analyze(&epg, 0.1);
        assert!(!proc.mobilities.is_empty());
    }

    #[test]
    fn test_ce_processor_plates() {
        let params = CeParams::new(50.0, 40.0, 20.0);
        let mut proc = CeProcessor::new(params);
        let epg = make_gaussian_epg(1.0, 0.05, 300);
        proc.analyze(&epg, 0.1);
        let n = proc.plates_for_peak(0);
        assert!(n > 100.0);
    }

    #[test]
    fn test_ce_processor_resolution() {
        let params = CeParams::new(50.0, 40.0, 20.0);
        let mut proc = CeProcessor::new(params);
        let mut time = Vec::new();
        let mut sig = Vec::new();
        for i in 0..500 {
            let t = i as f64 * 0.01;
            time.push(t);
            sig.push(gaussian_peak(t, 1.5, 1.0, 0.1) + gaussian_peak(t, 3.0, 0.8, 0.1));
        }
        let epg = Electropherogram::new(time, sig);
        proc.analyze(&epg, 0.1);
        if proc.peaks.len() >= 2 {
            let r = proc.resolution_between(0, 1);
            assert!(r > 0.0);
        }
    }

    #[test]
    fn test_deconvolve_two_peaks() {
        let mut time = Vec::new();
        let mut sig = Vec::new();
        for i in 0..300 {
            let t = i as f64 * 0.01;
            time.push(t);
            sig.push(gaussian_peak(t, 1.0, 1.0, 0.1) + gaussian_peak(t, 1.5, 0.8, 0.12));
        }
        let (c1, h1, _s1, c2, h2, _s2) = deconvolve_two_peaks(&time, &sig, 1.0, 1.5);
        assert!(approx_eq(c1, 1.0, 0.01));
        assert!(approx_eq(c2, 1.5, 0.01));
        assert!(h1 > 0.5);
        assert!(h2 > 0.3);
    }

    #[test]
    fn test_multi_gaussian() {
        let params = vec![(1.0, 1.0, 0.1), (2.0, 0.5, 0.2)];
        let vals = multi_gaussian(1.0, &params);
        assert_eq!(vals.len(), 2);
        assert!(approx_eq(vals[0], 1.0, 1e-10));
    }

    #[test]
    fn test_field_strength() {
        let params = CeParams::new(50.0, 40.0, 20.0);
        let e = params.field_strength();
        // 20000 V / 50 cm = 400 V/cm
        assert!(approx_eq(e, 400.0, 0.01));
    }

    #[test]
    fn test_epg_len() {
        let epg = Electropherogram::new(vec![1.0, 2.0], vec![0.5, 0.6]);
        assert_eq!(epg.len(), 2);
    }

    #[test]
    fn test_mobility_zero_voltage() {
        let mu = electrophoretic_mobility(40.0, 50.0, 0.0, 300.0);
        assert!(approx_eq(mu, 0.0, 1e-10));
    }

    #[test]
    fn test_plate_count_zero_width() {
        let n = plate_count(5.0, 0.0);
        assert!(approx_eq(n, 0.0, 1e-10));
    }

    #[test]
    fn test_resolution_zero_width() {
        let r = resolution(5.0, 6.0, 0.0, 0.0);
        assert!(approx_eq(r, 0.0, 1e-10));
    }

    #[test]
    fn test_eof_mobility() {
        let mu = eof_mobility(40.0, 50.0, 20000.0, 200.0);
        assert!(mu > 0.0);
    }

    #[test]
    fn test_hydrodynamic_injection_increases_with_pressure() {
        let v1 = hydrodynamic_injection_nl(25.0, 50.0, 5.0, 1.0, 50.0);
        let v2 = hydrodynamic_injection_nl(50.0, 50.0, 5.0, 1.0, 50.0);
        assert!(v2 > v1);
    }

    #[test]
    fn test_stokes_radius_zero_mobility() {
        let r = stokes_radius_nm(0.0, 1.0);
        assert!(approx_eq(r, 0.0, 1e-10));
    }
}
