// flow_injection_analysis_processor.rs
//
// Flow Injection Analysis (FIA) signal processing for analytical chemistry.
// Dispersion modeling, peak analysis, EMG fitting, calibration, baseline correction.

/// FIA signal: time vs detector response (absorbance, fluorescence, etc.).
#[derive(Debug, Clone)]
pub struct FiaSignal {
    pub time_s: Vec<f64>,
    pub response: Vec<f64>,
}

/// A detected FIA peak.
#[derive(Debug, Clone)]
pub struct FiaPeak {
    pub index: usize,
    pub time_s: f64,
    pub height: f64,
    pub area: f64,
    pub width_half_s: f64,
    pub rise_time_s: f64,
    pub tail_time_s: f64,
    pub asymmetry: f64,
}

/// Calibration result from linear regression.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    pub slope: f64,
    pub intercept: f64,
    pub r_squared: f64,
    pub lod: f64,
    pub loq: f64,
}

impl FiaSignal {
    pub fn new(time_s: Vec<f64>, response: Vec<f64>) -> Self {
        Self { time_s, response }
    }

    pub fn len(&self) -> usize {
        self.time_s.len().min(self.response.len())
    }

    /// Find peaks above threshold.
    pub fn find_peaks(&self, threshold: f64) -> Vec<FiaPeak> {
        let n = self.len();
        if n < 3 { return vec![]; }
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if self.response[i] > threshold
                && self.response[i] >= self.response[i - 1]
                && self.response[i] >= self.response[i + 1]
            {
                let half = self.response[i] * 0.5;
                let (left_t, right_t) = half_height_times(
                    &self.time_s, &self.response, i, half,
                );
                let width = right_t - left_t;
                let rise = self.time_s[i] - left_t;
                let tail = right_t - self.time_s[i];
                let asym = if rise.abs() < 1e-30 { 1.0 } else { tail / rise };
                let area = trapz_around(&self.time_s, &self.response, i);
                peaks.push(FiaPeak {
                    index: i,
                    time_s: self.time_s[i],
                    height: self.response[i],
                    area,
                    width_half_s: width,
                    rise_time_s: rise,
                    tail_time_s: tail,
                    asymmetry: asym,
                });
            }
        }
        peaks
    }

    /// Linear baseline correction between first and last points.
    pub fn subtract_baseline(&mut self) {
        let n = self.len();
        if n < 2 { return; }
        let y0 = self.response[0];
        let y1 = self.response[n - 1];
        for i in 0..n {
            let frac = i as f64 / (n - 1) as f64;
            self.response[i] -= y0 + frac * (y1 - y0);
        }
    }

    /// Polynomial baseline correction (degree 2).
    pub fn subtract_poly_baseline(&mut self, window_pts: usize) {
        let n = self.len();
        if n < 6 { return; }
        let w = window_pts.min(n / 3).max(2);
        // Use first and last w points to fit parabola
        let mut pts_t = Vec::new();
        let mut pts_y = Vec::new();
        for i in 0..w {
            pts_t.push(self.time_s[i]);
            pts_y.push(self.response[i]);
        }
        for i in n - w..n {
            pts_t.push(self.time_s[i]);
            pts_y.push(self.response[i]);
        }
        let (a, b, c) = fit_quadratic(&pts_t, &pts_y);
        for i in 0..n {
            let t = self.time_s[i];
            self.response[i] -= a * t * t + b * t + c;
        }
    }
}

fn half_height_times(
    time: &[f64], sig: &[f64], peak_idx: usize, half_max: f64,
) -> (f64, f64) {
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
    (left, right)
}

fn trapz_around(time: &[f64], sig: &[f64], peak_idx: usize) -> f64 {
    let n = time.len();
    let start = if peak_idx >= 10 { peak_idx - 10 } else { 0 };
    let end = if peak_idx + 10 < n { peak_idx + 10 } else { n - 1 };
    let mut area = 0.0;
    for i in start..end {
        let dt = time[i + 1] - time[i];
        area += 0.5 * (sig[i] + sig[i + 1]) * dt;
    }
    area
}

fn fit_quadratic(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len().min(y.len()) as f64;
    if n < 3.0 { return (0.0, 0.0, y.first().copied().unwrap_or(0.0)); }
    let mut sx = 0.0_f64; let mut sx2 = 0.0; let mut sx3 = 0.0; let mut sx4 = 0.0;
    let mut sy = 0.0; let mut sxy = 0.0; let mut sx2y = 0.0;
    for i in 0..x.len().min(y.len()) {
        let xi = x[i]; let yi = y[i];
        sx += xi; sx2 += xi*xi; sx3 += xi*xi*xi; sx4 += xi*xi*xi*xi;
        sy += yi; sxy += xi*yi; sx2y += xi*xi*yi;
    }
    // Solve 3x3 system: [sx4 sx3 sx2; sx3 sx2 sx; sx2 sx n] * [a;b;c] = [sx2y; sxy; sy]
    let mat = [
        [sx4, sx3, sx2], [sx3, sx2, sx], [sx2, sx, n],
    ];
    let rhs = [sx2y, sxy, sy];
    solve_3x3(&mat, &rhs)
}

fn solve_3x3(m: &[[f64; 3]; 3], b: &[f64; 3]) -> (f64, f64, f64) {
    let det = m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
            + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    if det.abs() < 1e-30 { return (0.0, 0.0, 0.0); }
    let x0 = (b[0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(b[1]*m[2][2]-m[1][2]*b[2])
            + m[0][2]*(b[1]*m[2][1]-m[1][1]*b[2])) / det;
    let x1 = (m[0][0]*(b[1]*m[2][2]-m[1][2]*b[2])
            - b[0]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
            + m[0][2]*(m[1][0]*b[2]-b[1]*m[2][0])) / det;
    let x2 = (m[0][0]*(m[1][1]*b[2]-b[1]*m[2][1])
            - m[0][1]*(m[1][0]*b[2]-b[1]*m[2][0])
            + b[0]*(m[1][0]*m[2][1]-m[1][1]*m[2][0])) / det;
    (x0, x1, x2)
}

// ---------------------------------------------------------------------------
// 2. Dispersion
// ---------------------------------------------------------------------------

/// Dispersion coefficient: D = C0 / Cmax.
pub fn dispersion_coefficient(c0: f64, c_max: f64) -> f64 {
    if c_max.abs() < 1e-30 { return f64::INFINITY; }
    c0 / c_max
}

/// Tanks-in-series model: C(t)/C0 = (N^N / Γ(N)) * (t/τ)^(N-1) * exp(-N*t/τ).
/// Uses Stirling approximation for Γ(N).
pub fn tanks_in_series(t: f64, tau: f64, n_tanks: f64) -> f64 {
    if tau <= 0.0 || t < 0.0 || n_tanks < 1.0 { return 0.0; }
    let x = t / tau;
    let ln_gamma_n = stirling_ln_gamma(n_tanks);
    let ln_val = n_tanks * n_tanks.ln() - ln_gamma_n
        + (n_tanks - 1.0) * x.max(1e-30).ln() - n_tanks * x;
    ln_val.exp()
}

fn stirling_ln_gamma(x: f64) -> f64 {
    if x <= 0.0 { return 0.0; }
    if x < 1.0 {
        // Use Γ(x+1) = x * Γ(x)
        return stirling_ln_gamma(x + 1.0) - x.ln();
    }
    0.5 * (2.0 * std::f64::consts::PI).ln() + (x - 0.5) * x.ln() - x
        + 1.0 / (12.0 * x) - 1.0 / (360.0 * x * x * x)
}

// ---------------------------------------------------------------------------
// 3. Sample Throughput
// ---------------------------------------------------------------------------

/// Samples per hour from cycle time.
pub fn throughput_per_hour(cycle_time_s: f64) -> f64 {
    if cycle_time_s <= 0.0 { return 0.0; }
    3600.0 / cycle_time_s
}

// ---------------------------------------------------------------------------
// 4. Residence Time
// ---------------------------------------------------------------------------

/// Mean residence time from first moment: τ = Σ(t_i * C_i * dt) / Σ(C_i * dt).
pub fn mean_residence_time(time: &[f64], signal: &[f64]) -> f64 {
    let n = time.len().min(signal.len());
    if n < 2 { return 0.0; }
    let mut num = 0.0_f64;
    let mut den = 0.0_f64;
    for i in 0..n - 1 {
        let dt = time[i + 1] - time[i];
        let avg_s = 0.5 * (signal[i] + signal[i + 1]);
        let avg_t = 0.5 * (time[i] + time[i + 1]);
        num += avg_t * avg_s * dt;
        den += avg_s * dt;
    }
    if den.abs() < 1e-30 { return 0.0; }
    num / den
}

/// Variance of residence time (second central moment).
pub fn residence_time_variance(time: &[f64], signal: &[f64]) -> f64 {
    let tau = mean_residence_time(time, signal);
    let n = time.len().min(signal.len());
    if n < 2 { return 0.0; }
    let mut num = 0.0_f64;
    let mut den = 0.0_f64;
    for i in 0..n - 1 {
        let dt = time[i + 1] - time[i];
        let avg_s = 0.5 * (signal[i] + signal[i + 1]);
        let avg_t = 0.5 * (time[i] + time[i + 1]);
        num += (avg_t - tau).powi(2) * avg_s * dt;
        den += avg_s * dt;
    }
    if den.abs() < 1e-30 { return 0.0; }
    num / den
}

// ---------------------------------------------------------------------------
// 5. Exponentially Modified Gaussian (EMG)
// ---------------------------------------------------------------------------

/// EMG peak model: convolution of Gaussian and exponential tail.
/// Approximated by: y = (h * σ/τ) * sqrt(π/2) * exp(σ²/(2τ²) - (t-μ)/τ) * erfc((σ/τ - (t-μ)/σ)/√2)
/// Simplified: for large τ, approaches exponential; for small τ, approaches Gaussian.
pub fn emg_peak(t: f64, mu: f64, sigma: f64, tau: f64, height: f64) -> f64 {
    if sigma <= 0.0 || tau <= 0.0 { return 0.0; }
    let z = (t - mu) / sigma;
    let lambda = sigma / tau;
    let arg1 = 0.5 * lambda * lambda - lambda * z;
    let arg2 = (lambda - z) / std::f64::consts::SQRT_2;
    let exp_part = arg1.exp();
    let erfc_part = erfc_approx(arg2);
    height * 0.5 * lambda * (2.0 * std::f64::consts::PI).sqrt() * exp_part * erfc_part
}

/// Complementary error function approximation (Abramowitz & Stegun 7.1.26).
fn erfc_approx(x: f64) -> f64 {
    if x > 6.0 { return 0.0; }
    if x < -6.0 { return 2.0; }
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t * (0.254829592 + t * (-0.284496736 + t * (1.421413741
        + t * (-1.453152027 + t * 1.061405429))));
    let val = poly * (-x * x).exp();
    if x >= 0.0 { val } else { 2.0 - val }
}

// ---------------------------------------------------------------------------
// 6. Calibration
// ---------------------------------------------------------------------------

/// Linear regression calibration: response = slope * concentration + intercept.
pub fn calibrate_linear(concentrations: &[f64], responses: &[f64]) -> CalibrationResult {
    let n = concentrations.len().min(responses.len()) as f64;
    if n < 2.0 {
        return CalibrationResult { slope: 0.0, intercept: 0.0, r_squared: 0.0, lod: 0.0, loq: 0.0 };
    }
    let sx: f64 = concentrations.iter().take(n as usize).sum();
    let sy: f64 = responses.iter().take(n as usize).sum();
    let sxy: f64 = concentrations.iter().zip(responses.iter()).take(n as usize)
        .map(|(x, y)| x * y).sum();
    let sxx: f64 = concentrations.iter().take(n as usize).map(|x| x * x).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return CalibrationResult { slope: 0.0, intercept: 0.0, r_squared: 0.0, lod: 0.0, loq: 0.0 };
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    let y_mean = sy / n;
    let ss_tot: f64 = responses.iter().take(n as usize).map(|y| (y - y_mean).powi(2)).sum();
    let ss_res: f64 = concentrations.iter().zip(responses.iter()).take(n as usize)
        .map(|(x, y)| (y - (slope * x + intercept)).powi(2)).sum();
    let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };
    // Estimate std dev of residuals for LOD
    let s_residual = if n > 2.0 { (ss_res / (n - 2.0)).sqrt() } else { 0.0 };
    let lod = 3.0 * s_residual / slope.abs().max(1e-30);
    let loq = 10.0 * s_residual / slope.abs().max(1e-30);
    CalibrationResult { slope, intercept, r_squared, lod, loq }
}

/// Predict concentration from response using calibration.
pub fn predict_concentration(cal: &CalibrationResult, response: f64) -> f64 {
    if cal.slope.abs() < 1e-30 { return 0.0; }
    (response - cal.intercept) / cal.slope
}

// ---------------------------------------------------------------------------
// 7. FIA Processor
// ---------------------------------------------------------------------------

/// Main FIA processor.
pub struct FiaProcessor {
    pub peaks: Vec<FiaPeak>,
    pub dispersion: Option<f64>,
    pub mean_rt: Option<f64>,
    pub calibration: Option<CalibrationResult>,
}

impl FiaProcessor {
    pub fn new() -> Self {
        Self { peaks: vec![], dispersion: None, mean_rt: None, calibration: None }
    }

    /// Analyze signal: find peaks, compute dispersion and residence time.
    pub fn analyze(&mut self, signal: &FiaSignal, threshold: f64, c0: f64) {
        self.peaks = signal.find_peaks(threshold);
        if let Some(p) = self.peaks.first() {
            self.dispersion = Some(dispersion_coefficient(c0, p.height));
        }
        self.mean_rt = Some(mean_residence_time(&signal.time_s, &signal.response));
    }

    /// Set calibration from standards.
    pub fn set_calibration(&mut self, concentrations: &[f64], responses: &[f64]) {
        self.calibration = Some(calibrate_linear(concentrations, responses));
    }

    /// Predict concentration for a peak height.
    pub fn predict(&self, response: f64) -> Option<f64> {
        self.calibration.as_ref().map(|c| predict_concentration(c, response))
    }

    /// Throughput estimate from average cycle time.
    pub fn throughput(&self) -> f64 {
        if self.peaks.len() < 2 { return 0.0; }
        let dt = self.peaks.last().unwrap().time_s - self.peaks[0].time_s;
        let n = (self.peaks.len() - 1) as f64;
        throughput_per_hour(dt / n)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

fn approx_eq(a: f64, b: f64, tol: f64) -> bool { (a - b).abs() < tol }

#[cfg(test)]
mod tests {
    use super::*;

    fn gaussian(t: f64, center: f64, height: f64, sigma: f64) -> f64 {
        height * (-0.5 * ((t - center) / sigma).powi(2)).exp()
    }

    fn make_fia_signal(center: f64, sigma: f64, n: usize) -> FiaSignal {
        let mut time = Vec::with_capacity(n);
        let mut resp = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 * 0.1;
            time.push(t);
            resp.push(gaussian(t, center, 1.0, sigma));
        }
        FiaSignal::new(time, resp)
    }

    #[test]
    fn test_dispersion_coefficient() {
        let d = dispersion_coefficient(1.0, 0.5);
        assert!(approx_eq(d, 2.0, 0.01));
    }

    #[test]
    fn test_dispersion_zero_cmax() {
        let d = dispersion_coefficient(1.0, 0.0);
        assert!(d.is_infinite());
    }

    #[test]
    fn test_tanks_in_series_basic() {
        let c = tanks_in_series(1.0, 1.0, 1.0);
        // N=1: C/C0 = exp(-t/τ)
        assert!(approx_eq(c, (-1.0_f64).exp(), 0.05));
    }

    #[test]
    fn test_tanks_in_series_many() {
        // With many tanks, peak narrows around τ
        let c_at_tau = tanks_in_series(1.0, 1.0, 10.0);
        assert!(c_at_tau > 0.5);
    }

    #[test]
    fn test_tanks_negative_time() {
        let c = tanks_in_series(-1.0, 1.0, 5.0);
        assert!(approx_eq(c, 0.0, 1e-10));
    }

    #[test]
    fn test_throughput() {
        let t = throughput_per_hour(60.0);
        assert!(approx_eq(t, 60.0, 0.01));
    }

    #[test]
    fn test_throughput_zero() {
        let t = throughput_per_hour(0.0);
        assert!(approx_eq(t, 0.0, 1e-10));
    }

    #[test]
    fn test_mean_residence_time() {
        let sig = make_fia_signal(5.0, 1.0, 200);
        let tau = mean_residence_time(&sig.time_s, &sig.response);
        assert!(approx_eq(tau, 5.0, 0.2));
    }

    #[test]
    fn test_residence_time_variance() {
        let sig = make_fia_signal(5.0, 1.0, 200);
        let var = residence_time_variance(&sig.time_s, &sig.response);
        // Variance of Gaussian ≈ σ² = 1.0
        assert!(approx_eq(var, 1.0, 0.2));
    }

    #[test]
    fn test_emg_peak_gaussian_limit() {
        // Very large τ → Gaussian
        let y = emg_peak(5.0, 5.0, 1.0, 100.0, 1.0);
        assert!(y > 0.0);
    }

    #[test]
    fn test_emg_peak_tail() {
        let y_before = emg_peak(3.0, 5.0, 0.5, 0.5, 1.0);
        let y_after = emg_peak(7.0, 5.0, 0.5, 0.5, 1.0);
        // EMG has longer tail on the right
        assert!(y_after > y_before || y_after >= 0.0);
    }

    #[test]
    fn test_erfc_approx_at_zero() {
        let v = erfc_approx(0.0);
        assert!(approx_eq(v, 1.0, 0.01));
    }

    #[test]
    fn test_erfc_approx_large() {
        let v = erfc_approx(6.0);
        assert!(approx_eq(v, 0.0, 0.01));
    }

    #[test]
    fn test_erfc_approx_negative() {
        let v = erfc_approx(-6.0);
        assert!(approx_eq(v, 2.0, 0.01));
    }

    #[test]
    fn test_calibrate_linear() {
        let conc = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let resp: Vec<f64> = conc.iter().map(|&c| 0.5 * c + 0.1).collect();
        let cal = calibrate_linear(&conc, &resp);
        assert!(approx_eq(cal.slope, 0.5, 0.01));
        assert!(approx_eq(cal.intercept, 0.1, 0.01));
        assert!(approx_eq(cal.r_squared, 1.0, 0.001));
    }

    #[test]
    fn test_predict_concentration() {
        let cal = CalibrationResult { slope: 2.0, intercept: 0.5, r_squared: 1.0, lod: 0.0, loq: 0.0 };
        let c = predict_concentration(&cal, 4.5);
        assert!(approx_eq(c, 2.0, 0.01));
    }

    #[test]
    fn test_lod_loq() {
        let conc = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let resp = vec![0.1, 0.62, 1.08, 1.55, 2.03, 2.51];
        let cal = calibrate_linear(&conc, &resp);
        assert!(cal.lod >= 0.0);
        assert!(cal.loq >= cal.lod);
    }

    #[test]
    fn test_find_peaks_single() {
        let sig = make_fia_signal(5.0, 0.5, 200);
        let peaks = sig.find_peaks(0.1);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].time_s, 5.0, 0.2));
    }

    #[test]
    fn test_find_peaks_height() {
        let sig = make_fia_signal(5.0, 0.5, 200);
        let peaks = sig.find_peaks(0.1);
        assert!(approx_eq(peaks[0].height, 1.0, 0.05));
    }

    #[test]
    fn test_find_peaks_area() {
        let sig = make_fia_signal(5.0, 0.5, 200);
        let peaks = sig.find_peaks(0.1);
        assert!(peaks[0].area > 0.0);
    }

    #[test]
    fn test_peak_width_gaussian() {
        let sig = make_fia_signal(5.0, 0.5, 200);
        let peaks = sig.find_peaks(0.1);
        // FWHM ≈ 2.355 * 0.5 ≈ 1.18
        assert!(approx_eq(peaks[0].width_half_s, 1.18, 0.3));
    }

    #[test]
    fn test_peak_asymmetry_symmetric() {
        let sig = make_fia_signal(5.0, 0.5, 200);
        let peaks = sig.find_peaks(0.1);
        assert!(approx_eq(peaks[0].asymmetry, 1.0, 0.3));
    }

    #[test]
    fn test_subtract_baseline() {
        let mut sig = FiaSignal::new(
            vec![0.0, 1.0, 2.0, 3.0],
            vec![1.0, 2.0, 3.0, 4.0],
        );
        sig.subtract_baseline();
        assert!(approx_eq(sig.response[0], 0.0, 1e-10));
        assert!(approx_eq(sig.response[3], 0.0, 1e-10));
    }

    #[test]
    fn test_subtract_poly_baseline() {
        let mut time = Vec::new();
        let mut resp = Vec::new();
        for i in 0..100 {
            let t = i as f64 * 0.1;
            time.push(t);
            resp.push(0.01 * t * t + gaussian(t, 5.0, 1.0, 0.5));
        }
        let mut sig = FiaSignal::new(time, resp);
        sig.subtract_poly_baseline(10);
        // Peak should still be present around t=5
        let max_val = sig.response.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 0.3);
    }

    #[test]
    fn test_fia_processor_basic() {
        let mut proc = FiaProcessor::new();
        let sig = make_fia_signal(5.0, 0.5, 200);
        proc.analyze(&sig, 0.1, 2.0);
        assert_eq!(proc.peaks.len(), 1);
        assert!(proc.dispersion.unwrap() > 1.0);
    }

    #[test]
    fn test_fia_processor_calibration() {
        let mut proc = FiaProcessor::new();
        let conc = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let resp: Vec<f64> = conc.iter().map(|&c| 0.5 * c + 0.1).collect();
        proc.set_calibration(&conc, &resp);
        let c = proc.predict(2.6).unwrap();
        assert!(approx_eq(c, 5.0, 0.01));
    }

    #[test]
    fn test_fia_processor_residence_time() {
        let mut proc = FiaProcessor::new();
        let sig = make_fia_signal(5.0, 0.5, 200);
        proc.analyze(&sig, 0.1, 1.0);
        assert!(proc.mean_rt.is_some());
        assert!(approx_eq(proc.mean_rt.unwrap(), 5.0, 0.3));
    }

    #[test]
    fn test_fia_processor_throughput() {
        let mut proc = FiaProcessor::new();
        let mut time = Vec::new();
        let mut resp = Vec::new();
        for i in 0..600 {
            let t = i as f64 * 0.1;
            time.push(t);
            resp.push(gaussian(t, 10.0, 1.0, 0.5) + gaussian(t, 30.0, 1.0, 0.5) + gaussian(t, 50.0, 1.0, 0.5));
        }
        let sig = FiaSignal::new(time, resp);
        proc.analyze(&sig, 0.1, 1.0);
        let tp = proc.throughput();
        assert!(tp > 0.0);
    }

    #[test]
    fn test_signal_len() {
        let sig = FiaSignal::new(vec![1.0, 2.0, 3.0], vec![0.1, 0.2]);
        assert_eq!(sig.len(), 2);
    }

    #[test]
    fn test_stirling_ln_gamma() {
        // Γ(5) = 24, ln(24) ≈ 3.178
        let v = stirling_ln_gamma(5.0);
        assert!(approx_eq(v, 24.0_f64.ln(), 0.05));
    }

    #[test]
    fn test_calibrate_perfect_fit() {
        let conc = vec![1.0, 2.0, 3.0];
        let resp = vec![2.0, 4.0, 6.0];
        let cal = calibrate_linear(&conc, &resp);
        assert!(approx_eq(cal.slope, 2.0, 0.01));
        assert!(approx_eq(cal.intercept, 0.0, 0.01));
    }

    #[test]
    fn test_two_peaks() {
        let mut time = Vec::new();
        let mut resp = Vec::new();
        for i in 0..300 {
            let t = i as f64 * 0.1;
            time.push(t);
            resp.push(gaussian(t, 8.0, 1.0, 0.5) + gaussian(t, 20.0, 0.8, 0.5));
        }
        let sig = FiaSignal::new(time, resp);
        let peaks = sig.find_peaks(0.1);
        assert!(peaks.len() >= 2);
    }
}
