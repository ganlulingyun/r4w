// chemiluminescence_detector_processor.rs
//
// Chemiluminescence (CL) detection signal processing.
// Flash/glow kinetics, quantum yield, luminol modeling, calibration, decay fitting.

/// CL signal: time vs light intensity (relative light units, RLU).
#[derive(Debug, Clone)]
pub struct ClSignal {
    pub time_s: Vec<f64>,
    pub intensity_rlu: Vec<f64>,
}

/// CL kinetic model type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KineticModel {
    /// Flash: I(t) = I0 * exp(-k*t)
    Flash,
    /// Glow: I(t) = A * (1 - exp(-k1*t)) * exp(-k2*t)
    Glow,
}

/// Decay fit result.
#[derive(Debug, Clone)]
pub struct DecayFit {
    pub model: KineticModel,
    pub amplitude: f64,
    pub k1: f64,
    pub k2: f64,
    pub half_life_s: f64,
    pub residual_rms: f64,
}

/// Calibration result.
#[derive(Debug, Clone)]
pub struct ClCalibration {
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
    pub lod: f64,
    pub loq: f64,
    pub log_linear: bool,
}

impl ClSignal {
    pub fn new(time_s: Vec<f64>, intensity_rlu: Vec<f64>) -> Self {
        Self { time_s, intensity_rlu }
    }

    pub fn len(&self) -> usize {
        self.time_s.len().min(self.intensity_rlu.len())
    }

    /// Peak intensity (maximum RLU).
    pub fn peak_intensity(&self) -> f64 {
        self.intensity_rlu.iter().cloned().fold(0.0_f64, f64::max)
    }

    /// Time of peak intensity.
    pub fn peak_time(&self) -> f64 {
        let n = self.len();
        let mut max_idx = 0;
        let mut max_val = f64::NEG_INFINITY;
        for i in 0..n {
            if self.intensity_rlu[i] > max_val {
                max_val = self.intensity_rlu[i];
                max_idx = i;
            }
        }
        self.time_s[max_idx]
    }

    /// Total light output (integrated area) using trapezoidal rule.
    pub fn total_light(&self) -> f64 {
        let n = self.len();
        if n < 2 { return 0.0; }
        let mut area = 0.0_f64;
        for i in 0..n - 1 {
            let dt = self.time_s[i + 1] - self.time_s[i];
            area += 0.5 * (self.intensity_rlu[i] + self.intensity_rlu[i + 1]) * dt;
        }
        area
    }

    /// Partial integration between t_start and t_end.
    pub fn partial_light(&self, t_start: f64, t_end: f64) -> f64 {
        let n = self.len();
        if n < 2 { return 0.0; }
        let mut area = 0.0_f64;
        for i in 0..n - 1 {
            if self.time_s[i + 1] < t_start || self.time_s[i] > t_end { continue; }
            let t0 = self.time_s[i].max(t_start);
            let t1 = self.time_s[i + 1].min(t_end);
            let dt = t1 - t0;
            if dt <= 0.0 { continue; }
            // Linear interpolation for values at window boundaries
            let frac0 = if (self.time_s[i + 1] - self.time_s[i]).abs() < 1e-30 { 0.0 }
                else { (t0 - self.time_s[i]) / (self.time_s[i + 1] - self.time_s[i]) };
            let frac1 = if (self.time_s[i + 1] - self.time_s[i]).abs() < 1e-30 { 0.0 }
                else { (t1 - self.time_s[i]) / (self.time_s[i + 1] - self.time_s[i]) };
            let v0 = self.intensity_rlu[i] + frac0 * (self.intensity_rlu[i + 1] - self.intensity_rlu[i]);
            let v1 = self.intensity_rlu[i] + frac1 * (self.intensity_rlu[i + 1] - self.intensity_rlu[i]);
            area += 0.5 * (v0 + v1) * dt;
        }
        area
    }

    /// Subtract dark current (constant offset).
    pub fn subtract_dark_current(&mut self, dark_rlu: f64) {
        for v in &mut self.intensity_rlu {
            *v = (*v - dark_rlu).max(0.0);
        }
    }

    /// Signal-to-noise ratio: peak / std_dev(baseline region).
    pub fn snr(&self, baseline_end_s: f64) -> f64 {
        let n = self.len();
        let mut bl_vals = Vec::new();
        for i in 0..n {
            if self.time_s[i] <= baseline_end_s {
                bl_vals.push(self.intensity_rlu[i]);
            }
        }
        if bl_vals.len() < 2 { return 0.0; }
        let mean: f64 = bl_vals.iter().sum::<f64>() / bl_vals.len() as f64;
        let var: f64 = bl_vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (bl_vals.len() - 1) as f64;
        let std = var.sqrt();
        if std < 1e-30 { return f64::INFINITY; }
        (self.peak_intensity() - mean) / std
    }
}

// ---------------------------------------------------------------------------
// 2. Flash Kinetics
// ---------------------------------------------------------------------------

/// Flash CL model: I(t) = I0 * exp(-k * t).
pub fn flash_model(t: f64, i0: f64, k: f64) -> f64 {
    i0 * (-k * t).exp()
}

/// Flash half-life: t_1/2 = ln(2) / k.
pub fn flash_half_life(k: f64) -> f64 {
    if k <= 0.0 { return f64::INFINITY; }
    2.0_f64.ln() / k
}

/// Fit single-exponential flash decay: ln(I) = ln(I0) - k*t.
pub fn fit_flash(time: &[f64], intensity: &[f64]) -> DecayFit {
    let n = time.len().min(intensity.len());
    if n < 3 {
        return DecayFit { model: KineticModel::Flash, amplitude: 0.0, k1: 0.0, k2: 0.0, half_life_s: 0.0, residual_rms: 0.0 };
    }
    let mut sx = 0.0_f64;
    let mut sy = 0.0_f64;
    let mut sxy = 0.0_f64;
    let mut sxx = 0.0_f64;
    let mut count = 0.0_f64;
    for i in 0..n {
        if intensity[i] <= 0.0 { continue; }
        let x = time[i];
        let y = intensity[i].ln();
        sx += x; sy += y; sxy += x * y; sxx += x * x;
        count += 1.0;
    }
    if count < 2.0 {
        return DecayFit { model: KineticModel::Flash, amplitude: 0.0, k1: 0.0, k2: 0.0, half_life_s: 0.0, residual_rms: 0.0 };
    }
    let denom = count * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return DecayFit { model: KineticModel::Flash, amplitude: 0.0, k1: 0.0, k2: 0.0, half_life_s: 0.0, residual_rms: 0.0 };
    }
    let slope = (count * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / count;
    let i0 = intercept.exp();
    let k = -slope;
    let rms = residual_rms_flash(time, intensity, n, i0, k);
    DecayFit {
        model: KineticModel::Flash,
        amplitude: i0, k1: k, k2: 0.0,
        half_life_s: flash_half_life(k),
        residual_rms: rms,
    }
}

fn residual_rms_flash(time: &[f64], intensity: &[f64], n: usize, i0: f64, k: f64) -> f64 {
    let mut ss = 0.0_f64;
    for i in 0..n {
        let pred = flash_model(time[i], i0, k);
        ss += (intensity[i] - pred).powi(2);
    }
    (ss / n as f64).sqrt()
}

// ---------------------------------------------------------------------------
// 3. Glow Kinetics
// ---------------------------------------------------------------------------

/// Glow CL model: I(t) = A * (1 - exp(-k1*t)) * exp(-k2*t).
pub fn glow_model(t: f64, a: f64, k1: f64, k2: f64) -> f64 {
    a * (1.0 - (-k1 * t).exp()) * (-k2 * t).exp()
}

/// Time of peak for glow model: t_peak = ln(1 + k1/k2) / k1.
pub fn glow_peak_time(k1: f64, k2: f64) -> f64 {
    if k1 <= 0.0 { return 0.0; }
    (1.0 + k1 / k2).ln() / k1
}

/// Fit glow model by grid search over k1, k2.
pub fn fit_glow(time: &[f64], intensity: &[f64]) -> DecayFit {
    let n = time.len().min(intensity.len());
    if n < 5 {
        return DecayFit { model: KineticModel::Glow, amplitude: 0.0, k1: 0.0, k2: 0.0, half_life_s: 0.0, residual_rms: 0.0 };
    }
    let i_max: f64 = intensity.iter().cloned().fold(0.0_f64, f64::max);
    let t_max = time[n - 1];
    let mut best_err = f64::MAX;
    let mut best = (i_max, 1.0_f64, 0.5_f64);
    for k1_i in 1..20 {
        let k1 = k1_i as f64 * 0.5;
        for k2_i in 1..20 {
            let k2 = k2_i as f64 * 0.1 / t_max.max(1.0);
            // Solve for A by least squares
            let mut num = 0.0_f64;
            let mut den = 0.0_f64;
            for i in 0..n {
                let g = (1.0 - (-k1 * time[i]).exp()) * (-k2 * time[i]).exp();
                num += intensity[i] * g;
                den += g * g;
            }
            if den < 1e-30 { continue; }
            let a = num / den;
            if a < 0.0 { continue; }
            let err: f64 = (0..n).map(|i| {
                let pred = glow_model(time[i], a, k1, k2);
                (intensity[i] - pred).powi(2)
            }).sum();
            if err < best_err {
                best_err = err;
                best = (a, k1, k2);
            }
        }
    }
    let (a, k1, k2) = best;
    DecayFit {
        model: KineticModel::Glow,
        amplitude: a, k1, k2,
        half_life_s: flash_half_life(k2),
        residual_rms: (best_err / n as f64).sqrt(),
    }
}

// ---------------------------------------------------------------------------
// 4. Quantum Yield
// ---------------------------------------------------------------------------

/// Chemiluminescence quantum yield: Φ_CL = total_photons / molecules_reacted.
pub fn quantum_yield(total_photons: f64, molecules_reacted: f64) -> f64 {
    if molecules_reacted.abs() < 1e-30 { return 0.0; }
    total_photons / molecules_reacted
}

/// Estimate total photons from integrated RLU and detector sensitivity.
/// photons = total_rlu / (detector_efficiency * solid_angle_fraction).
pub fn estimate_photons(total_rlu: f64, detector_efficiency: f64, solid_angle_frac: f64) -> f64 {
    let denom = detector_efficiency * solid_angle_frac;
    if denom < 1e-30 { return 0.0; }
    total_rlu / denom
}

// ---------------------------------------------------------------------------
// 5. Luminol Reaction Kinetics
// ---------------------------------------------------------------------------

/// Luminol-H2O2 second-order rate: d[H2O2]/dt = -k * [luminol] * [H2O2].
/// Returns [H2O2] at time t for equal initial concentrations.
/// [H2O2](t) = c0 / (1 + k * c0 * t) for second-order.
pub fn luminol_h2o2_concentration(c0: f64, k: f64, t: f64) -> f64 {
    if k <= 0.0 || c0 <= 0.0 { return c0; }
    c0 / (1.0 + k * c0 * t)
}

/// Luminol CL intensity proportional to rate of reaction.
/// I(t) ∝ k * [luminol] * [H2O2] = k * c(t)²
pub fn luminol_intensity(c0: f64, k: f64, t: f64) -> f64 {
    let c = luminol_h2o2_concentration(c0, k, t);
    k * c * c
}

// ---------------------------------------------------------------------------
// 6. Calibration
// ---------------------------------------------------------------------------

/// Log-linear calibration: log10(response) = slope * log10(conc) + intercept.
pub fn calibrate_log_linear(concentrations: &[f64], responses: &[f64]) -> ClCalibration {
    let n = concentrations.len().min(responses.len());
    if n < 2 {
        return ClCalibration { slope: 0.0, intercept: 0.0, r_squared: 0.0, lod: 0.0, loq: 0.0, log_linear: true };
    }
    let mut log_c = Vec::new();
    let mut log_r = Vec::new();
    for i in 0..n {
        if concentrations[i] > 0.0 && responses[i] > 0.0 {
            log_c.push(concentrations[i].log10());
            log_r.push(responses[i].log10());
        }
    }
    let m = log_c.len() as f64;
    if m < 2.0 {
        return ClCalibration { slope: 0.0, intercept: 0.0, r_squared: 0.0, lod: 0.0, loq: 0.0, log_linear: true };
    }
    let sx: f64 = log_c.iter().sum();
    let sy: f64 = log_r.iter().sum();
    let sxy: f64 = log_c.iter().zip(log_r.iter()).map(|(x, y)| x * y).sum();
    let sxx: f64 = log_c.iter().map(|x| x * x).sum();
    let denom = m * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return ClCalibration { slope: 0.0, intercept: 0.0, r_squared: 0.0, lod: 0.0, loq: 0.0, log_linear: true };
    }
    let slope = (m * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / m;
    let y_mean = sy / m;
    let ss_tot: f64 = log_r.iter().map(|y| (y - y_mean).powi(2)).sum();
    let ss_res: f64 = log_c.iter().zip(log_r.iter())
        .map(|(x, y)| (y - (slope * x + intercept)).powi(2)).sum();
    let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    let s_res = if m > 2.0 { (ss_res / (m - 2.0)).sqrt() } else { 0.0 };
    let lod = 10.0_f64.powf(3.0 * s_res / slope.abs().max(1e-30));
    let loq = 10.0_f64.powf(10.0 * s_res / slope.abs().max(1e-30));
    ClCalibration { slope, intercept, r_squared, lod, loq, log_linear: true }
}

/// Predict concentration from RLU using log-linear calibration.
pub fn predict_concentration_log(cal: &ClCalibration, rlu: f64) -> f64 {
    if cal.slope.abs() < 1e-30 || rlu <= 0.0 { return 0.0; }
    let log_c = (rlu.log10() - cal.intercept) / cal.slope;
    10.0_f64.powf(log_c)
}

// ---------------------------------------------------------------------------
// 7. Bi-exponential Decay
// ---------------------------------------------------------------------------

/// Bi-exponential model: I(t) = A1*exp(-k1*t) + A2*exp(-k2*t).
pub fn biexponential(t: f64, a1: f64, k1: f64, a2: f64, k2: f64) -> f64 {
    a1 * (-k1 * t).exp() + a2 * (-k2 * t).exp()
}

// ---------------------------------------------------------------------------
// 8. CL Detector Processor
// ---------------------------------------------------------------------------

/// Main CL detector processor.
pub struct ClDetectorProcessor {
    pub kinetic_model: KineticModel,
    pub dark_current_rlu: f64,
    pub decay_fit: Option<DecayFit>,
    pub calibration: Option<ClCalibration>,
}

impl ClDetectorProcessor {
    pub fn new(model: KineticModel) -> Self {
        Self {
            kinetic_model: model,
            dark_current_rlu: 0.0,
            decay_fit: None,
            calibration: None,
        }
    }

    pub fn flash() -> Self { Self::new(KineticModel::Flash) }
    pub fn glow() -> Self { Self::new(KineticModel::Glow) }

    /// Set dark current for subtraction.
    pub fn set_dark_current(&mut self, dark_rlu: f64) {
        self.dark_current_rlu = dark_rlu;
    }

    /// Fit kinetic model to signal.
    pub fn fit(&mut self, signal: &mut ClSignal) {
        signal.subtract_dark_current(self.dark_current_rlu);
        self.decay_fit = Some(match self.kinetic_model {
            KineticModel::Flash => fit_flash(&signal.time_s, &signal.intensity_rlu),
            KineticModel::Glow => fit_glow(&signal.time_s, &signal.intensity_rlu),
        });
    }

    /// Set calibration from standards.
    pub fn set_calibration(&mut self, conc: &[f64], responses: &[f64]) {
        self.calibration = Some(calibrate_log_linear(conc, responses));
    }

    /// Predict concentration.
    pub fn predict(&self, rlu: f64) -> Option<f64> {
        self.calibration.as_ref().map(|c| predict_concentration_log(c, rlu))
    }

    /// Get total light from a signal.
    pub fn total_light(&self, signal: &ClSignal) -> f64 {
        signal.total_light()
    }

    /// Get SNR.
    pub fn snr(&self, signal: &ClSignal, baseline_end_s: f64) -> f64 {
        signal.snr(baseline_end_s)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

#[cfg(test)]
mod tests {
    use super::*;

    fn make_flash_signal(i0: f64, k: f64, n: usize) -> ClSignal {
        let mut time = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 * 0.01;
            time.push(t);
            intensity.push(flash_model(t, i0, k));
        }
        ClSignal::new(time, intensity)
    }

    fn make_glow_signal(a: f64, k1: f64, k2: f64, n: usize) -> ClSignal {
        let mut time = Vec::with_capacity(n);
        let mut intensity = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 * 0.02;
            time.push(t);
            intensity.push(glow_model(t, a, k1, k2));
        }
        ClSignal::new(time, intensity)
    }

    #[test]
    fn test_flash_model_at_zero() {
        let i = flash_model(0.0, 1000.0, 1.0);
        assert!(approx_eq(i, 1000.0, 0.01));
    }

    #[test]
    fn test_flash_model_decay() {
        let i = flash_model(1.0, 1000.0, 1.0);
        assert!(approx_eq(i, 1000.0 * (-1.0_f64).exp(), 1.0));
    }

    #[test]
    fn test_flash_half_life() {
        let t = flash_half_life(1.0);
        assert!(approx_eq(t, 0.693, 0.001));
    }

    #[test]
    fn test_flash_half_life_zero_k() {
        let t = flash_half_life(0.0);
        assert!(t.is_infinite());
    }

    #[test]
    fn test_glow_model_at_zero() {
        let i = glow_model(0.0, 1000.0, 2.0, 0.5);
        assert!(approx_eq(i, 0.0, 0.01));
    }

    #[test]
    fn test_glow_model_rises() {
        let i0 = glow_model(0.0, 1000.0, 2.0, 0.5);
        let i1 = glow_model(0.5, 1000.0, 2.0, 0.5);
        assert!(i1 > i0);
    }

    #[test]
    fn test_glow_peak_time() {
        let tp = glow_peak_time(2.0, 0.5);
        assert!(tp > 0.0);
    }

    #[test]
    fn test_fit_flash() {
        let sig = make_flash_signal(1000.0, 2.0, 200);
        let fit = fit_flash(&sig.time_s, &sig.intensity_rlu);
        assert!(approx_eq(fit.amplitude, 1000.0, 100.0));
        assert!(approx_eq(fit.k1, 2.0, 0.3));
    }

    #[test]
    fn test_fit_glow() {
        let sig = make_glow_signal(1000.0, 5.0, 0.5, 200);
        let fit = fit_glow(&sig.time_s, &sig.intensity_rlu);
        assert!(fit.amplitude > 0.0);
        assert!(fit.k1 > 0.0);
    }

    #[test]
    fn test_quantum_yield() {
        let qy = quantum_yield(1e10, 1e12);
        assert!(approx_eq(qy, 0.01, 0.001));
    }

    #[test]
    fn test_quantum_yield_zero() {
        let qy = quantum_yield(100.0, 0.0);
        assert!(approx_eq(qy, 0.0, 1e-10));
    }

    #[test]
    fn test_estimate_photons() {
        let p = estimate_photons(1000.0, 0.1, 0.01);
        assert!(approx_eq(p, 1e6, 1.0));
    }

    #[test]
    fn test_luminol_concentration() {
        let c = luminol_h2o2_concentration(1.0, 1.0, 0.0);
        assert!(approx_eq(c, 1.0, 1e-10));
        let c2 = luminol_h2o2_concentration(1.0, 1.0, 1.0);
        assert!(approx_eq(c2, 0.5, 0.01));
    }

    #[test]
    fn test_luminol_intensity_decays() {
        let i0 = luminol_intensity(1.0, 1.0, 0.0);
        let i1 = luminol_intensity(1.0, 1.0, 5.0);
        assert!(i0 > i1);
    }

    #[test]
    fn test_calibrate_log_linear() {
        let conc = vec![0.01, 0.1, 1.0, 10.0, 100.0];
        let resp: Vec<f64> = conc.iter().map(|&c: &f64| 50.0 * c.powf(0.9)).collect();
        let cal = calibrate_log_linear(&conc, &resp);
        assert!(approx_eq(cal.slope, 0.9, 0.05));
        assert!(cal.r_squared > 0.99);
    }

    #[test]
    fn test_predict_concentration_log() {
        let cal = ClCalibration { slope: 1.0, intercept: 2.0, r_squared: 1.0, lod: 0.0, loq: 0.0, log_linear: true };
        // log10(rlu) = 1.0 * log10(c) + 2.0 → log10(c) = log10(rlu) - 2
        let c = predict_concentration_log(&cal, 1000.0);
        // log10(1000)=3, log10(c)=3-2=1, c=10
        assert!(approx_eq(c, 10.0, 0.1));
    }

    #[test]
    fn test_peak_intensity() {
        let sig = make_flash_signal(500.0, 1.0, 100);
        assert!(approx_eq(sig.peak_intensity(), 500.0, 1.0));
    }

    #[test]
    fn test_peak_time() {
        let sig = make_flash_signal(500.0, 1.0, 100);
        assert!(approx_eq(sig.peak_time(), 0.0, 0.01));
    }

    #[test]
    fn test_total_light() {
        let sig = make_flash_signal(1000.0, 1.0, 500);
        let tl = sig.total_light();
        // Integral of 1000*exp(-t) from 0 to ~5 ≈ 1000*(1-exp(-5)) ≈ 993
        assert!(approx_eq(tl, 993.0, 20.0));
    }

    #[test]
    fn test_partial_light() {
        let sig = make_flash_signal(1000.0, 1.0, 500);
        let full = sig.total_light();
        let partial = sig.partial_light(0.0, 1.0);
        assert!(partial > 0.0 && partial < full);
    }

    #[test]
    fn test_subtract_dark_current() {
        let mut sig = ClSignal::new(vec![0.0, 1.0], vec![100.0, 50.0]);
        sig.subtract_dark_current(10.0);
        assert!(approx_eq(sig.intensity_rlu[0], 90.0, 0.01));
        assert!(approx_eq(sig.intensity_rlu[1], 40.0, 0.01));
    }

    #[test]
    fn test_snr() {
        let mut time = Vec::new();
        let mut intensity = Vec::new();
        for i in 0..100 {
            let t = i as f64 * 0.1;
            time.push(t);
            if t < 3.0 { intensity.push(1.0); } // baseline
            else { intensity.push(1.0 + 100.0 * (-(t-5.0).powi(2)/0.5).exp()); }
        }
        let sig = ClSignal::new(time, intensity);
        let s = sig.snr(2.5);
        assert!(s > 10.0); // Good SNR
    }

    #[test]
    fn test_biexponential() {
        let y = biexponential(0.0, 500.0, 1.0, 300.0, 0.5);
        assert!(approx_eq(y, 800.0, 0.01));
    }

    #[test]
    fn test_cl_processor_flash() {
        let mut proc = ClDetectorProcessor::flash();
        let mut sig = make_flash_signal(1000.0, 2.0, 200);
        proc.fit(&mut sig);
        assert!(proc.decay_fit.is_some());
        let fit = proc.decay_fit.as_ref().unwrap();
        assert_eq!(fit.model, KineticModel::Flash);
    }

    #[test]
    fn test_cl_processor_glow() {
        let mut proc = ClDetectorProcessor::glow();
        let mut sig = make_glow_signal(1000.0, 5.0, 0.5, 200);
        proc.fit(&mut sig);
        assert!(proc.decay_fit.is_some());
    }

    #[test]
    fn test_cl_processor_calibration() {
        let mut proc = ClDetectorProcessor::flash();
        let conc = vec![0.01, 0.1, 1.0, 10.0];
        let resp: Vec<f64> = conc.iter().map(|&c| 100.0 * c).collect();
        proc.set_calibration(&conc, &resp);
        let c = proc.predict(500.0);
        assert!(c.is_some());
        assert!(c.unwrap() > 0.0);
    }

    #[test]
    fn test_cl_processor_dark_current() {
        let mut proc = ClDetectorProcessor::flash();
        proc.set_dark_current(5.0);
        let mut sig = ClSignal::new(vec![0.0, 1.0], vec![100.0, 50.0]);
        proc.fit(&mut sig);
        assert!(approx_eq(sig.intensity_rlu[0], 95.0, 0.01));
    }

    #[test]
    fn test_signal_len() {
        let sig = ClSignal::new(vec![0.0, 1.0, 2.0], vec![1.0, 2.0]);
        assert_eq!(sig.len(), 2);
    }

    #[test]
    fn test_cl_processor_total_light() {
        let proc = ClDetectorProcessor::flash();
        let sig = make_flash_signal(1000.0, 1.0, 200);
        let tl = proc.total_light(&sig);
        assert!(tl > 0.0);
    }

    #[test]
    fn test_glow_model_decays_eventually() {
        let early = glow_model(1.0, 1000.0, 5.0, 0.5);
        let late = glow_model(10.0, 1000.0, 5.0, 0.5);
        assert!(early > late);
    }

    #[test]
    fn test_partial_light_full_range() {
        let sig = make_flash_signal(1000.0, 1.0, 200);
        let full = sig.total_light();
        let partial = sig.partial_light(0.0, 2.0);
        assert!(approx_eq(partial, full, 1.0));
    }
}
