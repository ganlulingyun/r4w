// laser_ablation_icp_mass_processor.rs
//
// Laser Ablation ICP-MS (LA-ICP-MS) signal processing for elemental mapping.
// Transient signal integration, background subtraction, internal standard
// normalization, external calibration, LOD estimation, washout smoothing,
// spot size / fluence effects, and orchestrated data reduction.

/// A single time-resolved data point for one isotope channel.
#[derive(Debug, Clone, Copy)]
pub struct IsotopeReading {
    /// Time in seconds from acquisition start.
    pub time_s: f64,
    /// Counts per second (CPS) measured by the mass spectrometer.
    pub cps: f64,
}

/// Transient signal for a single isotope across an ablation event.
#[derive(Debug, Clone)]
pub struct TransientSignal {
    /// Isotope label (e.g. "238U", "208Pb", "29Si").
    pub isotope: String,
    /// Time-resolved CPS readings.
    pub readings: Vec<IsotopeReading>,
}

impl TransientSignal {
    /// Create a new transient signal for the given isotope.
    pub fn new(isotope: &str) -> Self {
        Self { isotope: isotope.to_string(), readings: Vec::new() }
    }
    /// Append a reading.
    pub fn add_reading(&mut self, time_s: f64, cps: f64) {
        self.readings.push(IsotopeReading { time_s, cps });
    }
    /// Number of readings.
    pub fn len(&self) -> usize { self.readings.len() }
    /// Whether the signal is empty.
    pub fn is_empty(&self) -> bool { self.readings.is_empty() }
    /// CPS values as a vector.
    pub fn cps_values(&self) -> Vec<f64> { self.readings.iter().map(|r| r.cps).collect() }
    /// Time axis as a vector.
    pub fn times(&self) -> Vec<f64> { self.readings.iter().map(|r| r.time_s).collect() }
    /// Maximum CPS in the transient.
    pub fn peak_cps(&self) -> f64 { self.readings.iter().map(|r| r.cps).fold(0.0_f64, f64::max) }
    /// Mean CPS over the entire transient.
    pub fn mean_cps(&self) -> f64 {
        if self.readings.is_empty() { return 0.0; }
        self.readings.iter().map(|r| r.cps).sum::<f64>() / self.readings.len() as f64
    }
}

// ---- 1. Signal Integration ------------------------------------------------

/// Integrate total counts (trapezoidal rule). Returns CPS * seconds.
pub fn integrate_signal(readings: &[IsotopeReading]) -> f64 {
    if readings.len() < 2 { return 0.0; }
    let mut total = 0.0;
    for i in 1..readings.len() {
        let dt = readings[i].time_s - readings[i - 1].time_s;
        total += (readings[i].cps + readings[i - 1].cps) * 0.5 * dt;
    }
    total
}

/// Integrate CPS within [t_start, t_end].
pub fn integrate_window(readings: &[IsotopeReading], t_start: f64, t_end: f64) -> f64 {
    let w: Vec<IsotopeReading> = readings.iter()
        .filter(|r| r.time_s >= t_start && r.time_s <= t_end).copied().collect();
    integrate_signal(&w)
}

/// Net integrated signal = signal integral minus background integral scaled to same duration.
pub fn net_integrated_signal(
    readings: &[IsotopeReading], sig_start: f64, sig_end: f64, bg_start: f64, bg_end: f64,
) -> f64 {
    let sig = integrate_window(readings, sig_start, sig_end);
    let bg = integrate_window(readings, bg_start, bg_end);
    let bg_dur = bg_end - bg_start;
    if bg_dur.abs() < 1e-30 { return sig; }
    sig - bg * ((sig_end - sig_start) / bg_dur)
}

// ---- 2. Background Subtraction (Gas Blank) --------------------------------

/// Background estimation statistics.
#[derive(Debug, Clone)]
pub struct BackgroundStats {
    pub mean_cps: f64,
    pub std_cps: f64,
    pub n_points: usize,
}

/// Compute background statistics in a time window.
pub fn background_stats(readings: &[IsotopeReading], t_start: f64, t_end: f64) -> BackgroundStats {
    let v: Vec<f64> = readings.iter()
        .filter(|r| r.time_s >= t_start && r.time_s <= t_end).map(|r| r.cps).collect();
    let n = v.len();
    if n == 0 { return BackgroundStats { mean_cps: 0.0, std_cps: 0.0, n_points: 0 }; }
    let mean = v.iter().sum::<f64>() / n as f64;
    let var = if n > 1 { v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1) as f64 } else { 0.0 };
    BackgroundStats { mean_cps: mean, std_cps: var.sqrt(), n_points: n }
}

/// Subtract constant background, clamping at zero.
pub fn subtract_background(readings: &[IsotopeReading], bg_mean: f64) -> Vec<IsotopeReading> {
    readings.iter().map(|r| IsotopeReading { time_s: r.time_s, cps: (r.cps - bg_mean).max(0.0) }).collect()
}

// ---- 3. Internal Standard Normalization -----------------------------------

/// CPS_analyte / CPS_IS for matched indices.
pub fn normalize_by_internal_standard(analyte: &[IsotopeReading], is: &[IsotopeReading]) -> Vec<f64> {
    let n = analyte.len().min(is.len());
    (0..n).map(|i| if is[i].cps.abs() < 1e-30 { 0.0 } else { analyte[i].cps / is[i].cps }).collect()
}

/// Mean analyte/IS ratio in a time window.
pub fn mean_is_ratio(analyte: &[IsotopeReading], is: &[IsotopeReading], t0: f64, t1: f64) -> f64 {
    let n = analyte.len().min(is.len());
    let (mut sum, mut cnt) = (0.0, 0usize);
    for i in 0..n {
        let t = analyte[i].time_s;
        if t >= t0 && t <= t1 && is[i].cps.abs() > 1e-30 {
            sum += analyte[i].cps / is[i].cps;
            cnt += 1;
        }
    }
    if cnt == 0 { 0.0 } else { sum / cnt as f64 }
}

// ---- 4. External Calibration (SRM) ----------------------------------------

/// Calibration point from a Standard Reference Material.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationPoint {
    /// Known concentration (ppm or wt%).
    pub concentration: f64,
    /// Measured IS-normalized ratio.
    pub is_ratio: f64,
}

/// Linear calibration: concentration = slope * ratio + intercept.
#[derive(Debug, Clone, Copy)]
pub struct CalibrationCurve {
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
}

/// Least-squares linear fit. X = ratio, Y = concentration.
pub fn fit_calibration(pts: &[CalibrationPoint]) -> CalibrationCurve {
    let n = pts.len() as f64;
    if n < 2.0 { return CalibrationCurve { slope: 0.0, intercept: 0.0, r_squared: 0.0 }; }
    let (mut sx, mut sy, mut sxx, mut sxy) = (0.0, 0.0, 0.0, 0.0);
    for p in pts { sx += p.is_ratio; sy += p.concentration; sxx += p.is_ratio * p.is_ratio; sxy += p.is_ratio * p.concentration; }
    let d = n * sxx - sx * sx;
    if d.abs() < 1e-30 { return CalibrationCurve { slope: 0.0, intercept: sy / n, r_squared: 0.0 }; }
    let slope = (n * sxy - sx * sy) / d;
    let intercept = (sy - slope * sx) / n;
    let ym = sy / n;
    let ss_tot: f64 = pts.iter().map(|p| (p.concentration - ym).powi(2)).sum();
    let ss_res: f64 = pts.iter().map(|p| (p.concentration - slope * p.is_ratio - intercept).powi(2)).sum();
    let r2 = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    CalibrationCurve { slope, intercept, r_squared: r2 }
}

/// Apply calibration curve to a ratio.
pub fn apply_calibration(c: &CalibrationCurve, ratio: f64) -> f64 { c.slope * ratio + c.intercept }

// ---- 5. Elemental Ratio Mapping -------------------------------------------

/// Elemental ratio at a single pixel / spot.
#[derive(Debug, Clone)]
pub struct ElementalRatio {
    pub analyte: String,
    pub internal_std: String,
    pub ratio: f64,
    pub concentration: f64,
}

/// Build elemental ratio map from corrected analyte & IS signals + calibration.
pub fn elemental_ratio_map(
    a_label: &str, is_label: &str, analyte: &[IsotopeReading],
    is: &[IsotopeReading], curve: &CalibrationCurve,
) -> Vec<ElementalRatio> {
    normalize_by_internal_standard(analyte, is).iter().map(|&r| ElementalRatio {
        analyte: a_label.to_string(), internal_std: is_label.to_string(),
        ratio: r, concentration: apply_calibration(curve, r),
    }).collect()
}

// ---- 6. Limit of Detection -----------------------------------------------

/// LOD = 3 * sigma_blank / sensitivity.
pub fn limit_of_detection(bg: &BackgroundStats, sensitivity: f64) -> f64 {
    if sensitivity.abs() < 1e-30 { return f64::INFINITY; }
    3.0 * bg.std_cps / sensitivity
}

/// LOD in concentration: 3 * (blank_std / IS_cps) / cal_slope.
pub fn lod_concentration(blank_std: f64, is_cps: f64, cal_slope: f64) -> f64 {
    if is_cps.abs() < 1e-30 || cal_slope.abs() < 1e-30 { return f64::INFINITY; }
    3.0 * (blank_std / is_cps) / cal_slope
}

// ---- 7. Washout Time and Signal Smoothing ---------------------------------

/// Time for signal to decay from peak to `fraction` of peak (seconds).
pub fn washout_time(readings: &[IsotopeReading], fraction: f64) -> Option<f64> {
    if readings.is_empty() { return None; }
    let peak = readings.iter().map(|r| r.cps).fold(0.0_f64, f64::max);
    if peak < 1e-30 { return None; }
    let thr = peak * fraction;
    let pi = readings.iter().enumerate()
        .max_by(|a, b| a.1.cps.partial_cmp(&b.1.cps).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i).unwrap_or(0);
    for r in &readings[pi..] { if r.cps <= thr { return Some(r.time_s - readings[pi].time_s); } }
    None
}

/// Exponential moving average smoothing. alpha in (0,1].
pub fn ema_smooth(readings: &[IsotopeReading], alpha: f64) -> Vec<IsotopeReading> {
    if readings.is_empty() { return Vec::new(); }
    let a = alpha.clamp(0.001, 1.0);
    let mut out = vec![readings[0]];
    let mut prev = readings[0].cps;
    for r in &readings[1..] {
        prev = a * r.cps + (1.0 - a) * prev;
        out.push(IsotopeReading { time_s: r.time_s, cps: prev });
    }
    out
}

/// Moving-average smoothing.
pub fn moving_average_smooth(readings: &[IsotopeReading], window: usize) -> Vec<IsotopeReading> {
    let n = readings.len();
    if n == 0 || window == 0 { return Vec::new(); }
    let half = window.max(1) / 2;
    (0..n).map(|i| {
        let lo = i.saturating_sub(half);
        let hi = (i + half + 1).min(n);
        let s: f64 = readings[lo..hi].iter().map(|r| r.cps).sum();
        IsotopeReading { time_s: readings[i].time_s, cps: s / (hi - lo) as f64 }
    }).collect()
}

// ---- 8. Spot Size and Fluence Effects -------------------------------------

/// Laser ablation parameters.
#[derive(Debug, Clone, Copy)]
pub struct LaserParams {
    pub spot_diameter_um: f64,
    pub energy_mj: f64,
    pub rep_rate_hz: f64,
    pub wavelength_nm: f64,
}

/// Spot area in cm^2.
pub fn spot_area_cm2(diameter_um: f64) -> f64 {
    let r = diameter_um * 1e-4 * 0.5;
    std::f64::consts::PI * r * r
}

/// Fluence (J/cm^2).
pub fn fluence_j_cm2(p: &LaserParams) -> f64 {
    let a = spot_area_cm2(p.spot_diameter_um);
    if a < 1e-30 { 0.0 } else { (p.energy_mj * 1e-3) / a }
}

/// Irradiance (W/cm^2) for a given pulse width in ns.
pub fn irradiance_w_cm2(p: &LaserParams, pulse_ns: f64) -> f64 {
    if pulse_ns < 1e-30 { return 0.0; }
    fluence_j_cm2(p) / (pulse_ns * 1e-9)
}

/// Sensitivity scales as spot area ratio.
pub fn sensitivity_scaling_factor(ref_um: f64, new_um: f64) -> f64 {
    let ar = spot_area_cm2(ref_um);
    if ar < 1e-30 { 0.0 } else { spot_area_cm2(new_um) / ar }
}

/// Empirical depth per pulse (nm): k * (fluence - threshold).
pub fn depth_per_pulse_nm(fluence: f64, threshold: f64, k: f64) -> f64 {
    if fluence <= threshold { 0.0 } else { k * (fluence - threshold) }
}

// ---- 9. LaIcpMsProcessor Orchestrator -------------------------------------

/// Result for a single ablation spot.
#[derive(Debug, Clone)]
pub struct SpotResult {
    pub isotope: String,
    pub bg_mean_cps: f64,
    pub bg_std_cps: f64,
    pub net_counts: f64,
    pub is_ratio: f64,
    pub concentration: f64,
    pub lod: f64,
}

/// Orchestrator for complete LA-ICP-MS data reduction.
pub struct LaIcpMsProcessor {
    pub laser: LaserParams,
    pub bg_window: (f64, f64),
    pub sig_window: (f64, f64),
    pub is_isotope: String,
    calibrations: Vec<(String, CalibrationCurve)>,
}

impl LaIcpMsProcessor {
    /// Create a new processor.
    pub fn new(laser: LaserParams, bg_window: (f64, f64), sig_window: (f64, f64), is_isotope: &str) -> Self {
        Self { laser, bg_window, sig_window, is_isotope: is_isotope.to_string(), calibrations: Vec::new() }
    }
    /// Standard 193 nm excimer, 50 um spot.
    pub fn standard_193nm() -> Self {
        Self::new(
            LaserParams { spot_diameter_um: 50.0, energy_mj: 4.0, rep_rate_hz: 10.0, wavelength_nm: 193.0 },
            (0.0, 20.0), (25.0, 55.0), "29Si",
        )
    }
    /// Add a calibration curve for an analyte isotope.
    pub fn add_calibration(&mut self, isotope: &str, curve: CalibrationCurve) {
        self.calibrations.push((isotope.to_string(), curve));
    }
    /// Look up calibration for an isotope.
    pub fn get_calibration(&self, isotope: &str) -> Option<&CalibrationCurve> {
        self.calibrations.iter().find(|(n, _)| n == isotope).map(|(_, c)| c)
    }
    /// Process a single spot: background-subtract, integrate, normalize, quantify.
    pub fn process_spot(&self, analyte: &TransientSignal, is: &TransientSignal) -> SpotResult {
        let bg_a = background_stats(&analyte.readings, self.bg_window.0, self.bg_window.1);
        let bg_i = background_stats(&is.readings, self.bg_window.0, self.bg_window.1);
        let ca = subtract_background(&analyte.readings, bg_a.mean_cps);
        let ci = subtract_background(&is.readings, bg_i.mean_cps);
        let ratio = mean_is_ratio(&ca, &ci, self.sig_window.0, self.sig_window.1);
        let conc = self.get_calibration(&analyte.isotope).map(|c| apply_calibration(c, ratio)).unwrap_or(f64::NAN);
        let net = net_integrated_signal(&analyte.readings, self.sig_window.0, self.sig_window.1, self.bg_window.0, self.bg_window.1);
        let lod = self.get_calibration(&analyte.isotope).map(|c| {
            let im = mean_cps_window(&ci, self.sig_window.0, self.sig_window.1);
            lod_concentration(bg_a.std_cps, im, c.slope)
        }).unwrap_or(f64::INFINITY);
        SpotResult { isotope: analyte.isotope.clone(), bg_mean_cps: bg_a.mean_cps, bg_std_cps: bg_a.std_cps, net_counts: net, is_ratio: ratio, concentration: conc, lod }
    }
    /// Fluence of current laser configuration.
    pub fn fluence(&self) -> f64 { fluence_j_cm2(&self.laser) }
}

fn mean_cps_window(readings: &[IsotopeReading], t0: f64, t1: f64) -> f64 {
    let v: Vec<f64> = readings.iter().filter(|r| r.time_s >= t0 && r.time_s <= t1).map(|r| r.cps).collect();
    if v.is_empty() { 0.0 } else { v.iter().sum::<f64>() / v.len() as f64 }
}

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------
#[cfg(test)]
mod tests {
    use super::*;

    fn make_signal(isotope: &str, base: f64, peak: f64) -> TransientSignal {
        let mut s = TransientSignal::new(isotope);
        for i in 0..20 { s.add_reading(i as f64, base + i as f64 * 0.1); }
        for i in 25..55 {
            let t = i as f64;
            let f = ((t - 25.0) / 5.0).min(1.0);
            let d = (-(t - 30.0).max(0.0) / 10.0).exp();
            s.add_reading(t, base + peak * f * d);
        }
        s
    }

    #[test] fn test_transient_new() {
        let s = TransientSignal::new("238U");
        assert_eq!(s.isotope, "238U"); assert!(s.is_empty()); assert_eq!(s.len(), 0);
    }
    #[test] fn test_transient_add() {
        let mut s = TransientSignal::new("208Pb");
        s.add_reading(0.0, 100.0); s.add_reading(1.0, 200.0);
        assert_eq!(s.len(), 2); assert!(!s.is_empty());
    }
    #[test] fn test_cps_values() {
        let mut s = TransientSignal::new("29Si");
        s.add_reading(0.0, 10.0); s.add_reading(1.0, 20.0);
        assert!(approx_eq(s.cps_values()[0], 10.0, 1e-10));
    }
    #[test] fn test_peak_cps() {
        let mut s = TransientSignal::new("57Fe");
        s.add_reading(0.0, 100.0); s.add_reading(1.0, 500.0); s.add_reading(2.0, 200.0);
        assert!(approx_eq(s.peak_cps(), 500.0, 1e-10));
    }
    #[test] fn test_mean_cps() {
        let mut s = TransientSignal::new("43Ca");
        s.add_reading(0.0, 100.0); s.add_reading(1.0, 200.0); s.add_reading(2.0, 300.0);
        assert!(approx_eq(s.mean_cps(), 200.0, 1e-10));
    }
    #[test] fn test_integrate_constant() {
        let r: Vec<_> = (0..11).map(|i| IsotopeReading { time_s: i as f64, cps: 1000.0 }).collect();
        assert!(approx_eq(integrate_signal(&r), 10000.0, 1.0));
    }
    #[test] fn test_integrate_linear() {
        let r: Vec<_> = (0..11).map(|i| IsotopeReading { time_s: i as f64, cps: i as f64 * 10.0 }).collect();
        assert!(approx_eq(integrate_signal(&r), 500.0, 1.0));
    }
    #[test] fn test_integrate_window() {
        let r: Vec<_> = (0..20).map(|i| IsotopeReading { time_s: i as f64, cps: 1000.0 }).collect();
        assert!(approx_eq(integrate_window(&r, 5.0, 10.0), 5000.0, 1.0));
    }
    #[test] fn test_net_integrated() {
        let mut r = Vec::new();
        for i in 0..6 { r.push(IsotopeReading { time_s: i as f64, cps: 100.0 }); }
        for i in 10..16 { r.push(IsotopeReading { time_s: i as f64, cps: 1100.0 }); }
        assert!(approx_eq(net_integrated_signal(&r, 10.0, 15.0, 0.0, 5.0), 5000.0, 10.0));
    }
    #[test] fn test_bg_stats_mean() {
        let r: Vec<_> = (0..10).map(|i| IsotopeReading { time_s: i as f64, cps: 100.0 }).collect();
        let bg = background_stats(&r, 0.0, 9.0);
        assert!(approx_eq(bg.mean_cps, 100.0, 1e-10)); assert_eq!(bg.n_points, 10);
    }
    #[test] fn test_bg_stats_std() {
        let r = vec![
            IsotopeReading { time_s: 0.0, cps: 90.0 },
            IsotopeReading { time_s: 1.0, cps: 110.0 },
            IsotopeReading { time_s: 2.0, cps: 100.0 },
        ];
        let bg = background_stats(&r, 0.0, 2.0);
        assert!(bg.std_cps > 0.0);
    }
    #[test] fn test_bg_stats_empty() {
        let bg = background_stats(&[], 0.0, 10.0);
        assert_eq!(bg.n_points, 0);
    }
    #[test] fn test_subtract_bg() {
        let r = vec![
            IsotopeReading { time_s: 0.0, cps: 150.0 },
            IsotopeReading { time_s: 1.0, cps: 50.0 },
        ];
        let c = subtract_background(&r, 100.0);
        assert!(approx_eq(c[0].cps, 50.0, 1e-10));
        assert!(approx_eq(c[1].cps, 0.0, 1e-10));
    }
    #[test] fn test_normalize_is() {
        let a = vec![IsotopeReading { time_s: 0.0, cps: 500.0 }, IsotopeReading { time_s: 1.0, cps: 1000.0 }];
        let is = vec![IsotopeReading { time_s: 0.0, cps: 100.0 }, IsotopeReading { time_s: 1.0, cps: 200.0 }];
        let r = normalize_by_internal_standard(&a, &is);
        assert!(approx_eq(r[0], 5.0, 1e-10)); assert!(approx_eq(r[1], 5.0, 1e-10));
    }
    #[test] fn test_normalize_zero_is() {
        let a = vec![IsotopeReading { time_s: 0.0, cps: 500.0 }];
        let is = vec![IsotopeReading { time_s: 0.0, cps: 0.0 }];
        assert!(approx_eq(normalize_by_internal_standard(&a, &is)[0], 0.0, 1e-10));
    }
    #[test] fn test_mean_is_ratio() {
        let a: Vec<_> = (0..10).map(|i| IsotopeReading { time_s: i as f64, cps: 1000.0 }).collect();
        let is: Vec<_> = (0..10).map(|i| IsotopeReading { time_s: i as f64, cps: 500.0 }).collect();
        assert!(approx_eq(mean_is_ratio(&a, &is, 2.0, 8.0), 2.0, 1e-10));
    }
    #[test] fn test_fit_cal_two_pt() {
        let pts = vec![
            CalibrationPoint { concentration: 0.0, is_ratio: 0.0 },
            CalibrationPoint { concentration: 100.0, is_ratio: 10.0 },
        ];
        let c = fit_calibration(&pts);
        assert!(approx_eq(c.slope, 10.0, 0.1)); assert!(c.r_squared > 0.99);
    }
    #[test] fn test_fit_cal_multi() {
        let pts = vec![
            CalibrationPoint { concentration: 0.0, is_ratio: 0.0 },
            CalibrationPoint { concentration: 50.0, is_ratio: 5.0 },
            CalibrationPoint { concentration: 100.0, is_ratio: 10.0 },
            CalibrationPoint { concentration: 200.0, is_ratio: 20.0 },
        ];
        let c = fit_calibration(&pts);
        assert!(approx_eq(c.slope, 10.0, 0.5)); assert!(c.r_squared > 0.99);
    }
    #[test] fn test_apply_cal() {
        let c = CalibrationCurve { slope: 10.0, intercept: 5.0, r_squared: 1.0 };
        assert!(approx_eq(apply_calibration(&c, 3.0), 35.0, 1e-10));
    }
    #[test] fn test_ratio_map() {
        let a: Vec<_> = (0..5).map(|i| IsotopeReading { time_s: i as f64, cps: 200.0 }).collect();
        let is: Vec<_> = (0..5).map(|i| IsotopeReading { time_s: i as f64, cps: 100.0 }).collect();
        let c = CalibrationCurve { slope: 50.0, intercept: 0.0, r_squared: 1.0 };
        let m = elemental_ratio_map("238U", "29Si", &a, &is, &c);
        assert_eq!(m.len(), 5);
        assert!(approx_eq(m[0].ratio, 2.0, 1e-10));
        assert!(approx_eq(m[0].concentration, 100.0, 1e-10));
    }
    #[test] fn test_lod_basic() {
        let bg = BackgroundStats { mean_cps: 100.0, std_cps: 10.0, n_points: 30 };
        assert!(approx_eq(limit_of_detection(&bg, 5.0), 6.0, 0.01));
    }
    #[test] fn test_lod_conc() {
        assert!(approx_eq(lod_concentration(30.0, 100000.0, 10.0), 9e-5, 1e-6));
    }
    #[test] fn test_lod_zero_sens() {
        let bg = BackgroundStats { mean_cps: 100.0, std_cps: 10.0, n_points: 30 };
        assert!(limit_of_detection(&bg, 0.0).is_infinite());
    }
    #[test] fn test_washout_basic() {
        let r: Vec<_> = (0..100).map(|i| {
            let t = i as f64 * 0.1;
            IsotopeReading { time_s: t, cps: 10000.0 * (-t / 2.0).exp() }
        }).collect();
        let w = washout_time(&r, 0.01).unwrap();
        assert!(w > 5.0 && w < 15.0);
    }
    #[test] fn test_washout_none() {
        let r: Vec<_> = (0..10).map(|i| IsotopeReading { time_s: i as f64, cps: 1000.0 }).collect();
        assert!(washout_time(&r, 0.01).is_none());
    }
    #[test] fn test_ema_smooth() {
        let r: Vec<_> = (0..10).map(|i| IsotopeReading {
            time_s: i as f64, cps: if i == 5 { 1000.0 } else { 100.0 },
        }).collect();
        let s = ema_smooth(&r, 0.3);
        assert_eq!(s.len(), 10); assert!(s[5].cps < 1000.0 && s[5].cps > 100.0);
    }
    #[test] fn test_ema_alpha_one() {
        let r: Vec<_> = (0..5).map(|i| IsotopeReading { time_s: i as f64, cps: i as f64 * 10.0 }).collect();
        let s = ema_smooth(&r, 1.0);
        for i in 0..5 { assert!(approx_eq(s[i].cps, r[i].cps, 1e-10)); }
    }
    #[test] fn test_mavg_smooth() {
        let r: Vec<_> = (0..10).map(|i| IsotopeReading {
            time_s: i as f64, cps: if i == 5 { 1000.0 } else { 100.0 },
        }).collect();
        let s = moving_average_smooth(&r, 3);
        assert_eq!(s.len(), 10); assert!(s[5].cps < 1000.0);
    }
    #[test] fn test_mavg_w1() {
        let r: Vec<_> = (0..5).map(|i| IsotopeReading { time_s: i as f64, cps: i as f64 }).collect();
        let s = moving_average_smooth(&r, 1);
        for i in 0..5 { assert!(approx_eq(s[i].cps, r[i].cps, 1e-10)); }
    }
    #[test] fn test_spot_area() {
        assert!(approx_eq(spot_area_cm2(100.0), 7.854e-5, 1e-7));
    }
    #[test] fn test_fluence() {
        let p = LaserParams { spot_diameter_um: 100.0, energy_mj: 5.0, rep_rate_hz: 10.0, wavelength_nm: 193.0 };
        let f = fluence_j_cm2(&p);
        assert!(f > 50.0 && f < 80.0);
    }
    #[test] fn test_irradiance() {
        let p = LaserParams { spot_diameter_um: 100.0, energy_mj: 5.0, rep_rate_hz: 10.0, wavelength_nm: 193.0 };
        assert!(irradiance_w_cm2(&p, 20.0) > 1e9);
    }
    #[test] fn test_sens_scaling() {
        assert!(approx_eq(sensitivity_scaling_factor(50.0, 100.0), 4.0, 0.01));
    }
    #[test] fn test_sens_same() {
        assert!(approx_eq(sensitivity_scaling_factor(50.0, 50.0), 1.0, 1e-10));
    }
    #[test] fn test_depth() {
        assert!(approx_eq(depth_per_pulse_nm(10.0, 2.0, 50.0), 400.0, 1e-10));
    }
    #[test] fn test_depth_below() {
        assert!(approx_eq(depth_per_pulse_nm(1.0, 2.0, 50.0), 0.0, 1e-10));
    }
    #[test] fn test_proc_new() {
        let p = LaIcpMsProcessor::standard_193nm();
        assert!(approx_eq(p.laser.wavelength_nm, 193.0, 1e-10));
        assert_eq!(p.is_isotope, "29Si");
    }
    #[test] fn test_proc_fluence() {
        assert!(LaIcpMsProcessor::standard_193nm().fluence() > 0.0);
    }
    #[test] fn test_proc_cal() {
        let mut p = LaIcpMsProcessor::standard_193nm();
        p.add_calibration("238U", CalibrationCurve { slope: 10.0, intercept: 0.0, r_squared: 0.99 });
        assert!(p.get_calibration("238U").is_some());
        assert!(p.get_calibration("x").is_none());
    }
    #[test] fn test_proc_spot() {
        let mut p = LaIcpMsProcessor::standard_193nm();
        p.add_calibration("238U", CalibrationCurve { slope: 10.0, intercept: 0.0, r_squared: 0.99 });
        let r = p.process_spot(&make_signal("238U", 100.0, 50000.0), &make_signal("29Si", 200.0, 100000.0));
        assert_eq!(r.isotope, "238U"); assert!(r.bg_mean_cps > 0.0);
        assert!(r.net_counts > 0.0); assert!(r.concentration.is_finite());
    }
    #[test] fn test_proc_lod() {
        let mut p = LaIcpMsProcessor::standard_193nm();
        p.add_calibration("208Pb", CalibrationCurve { slope: 10.0, intercept: 0.0, r_squared: 0.99 });
        let r = p.process_spot(&make_signal("208Pb", 50.0, 10000.0), &make_signal("29Si", 200.0, 100000.0));
        assert!(r.lod >= 0.0 && r.lod < f64::INFINITY);
    }
    #[test] fn test_cal_r2() {
        let pts = vec![
            CalibrationPoint { concentration: 0.0, is_ratio: 0.0 },
            CalibrationPoint { concentration: 10.0, is_ratio: 1.0 },
            CalibrationPoint { concentration: 20.0, is_ratio: 2.0 },
        ];
        assert!(approx_eq(fit_calibration(&pts).r_squared, 1.0, 1e-10));
    }
    #[test] fn test_cal_single() {
        let pts = vec![CalibrationPoint { concentration: 50.0, is_ratio: 5.0 }];
        assert!(approx_eq(fit_calibration(&pts).slope, 0.0, 1e-10));
    }
    #[test] fn test_integrate_empty() { assert!(approx_eq(integrate_signal(&[]), 0.0, 1e-10)); }
    #[test] fn test_integrate_single() {
        let r = vec![IsotopeReading { time_s: 0.0, cps: 1000.0 }];
        assert!(approx_eq(integrate_signal(&r), 0.0, 1e-10));
    }
}
