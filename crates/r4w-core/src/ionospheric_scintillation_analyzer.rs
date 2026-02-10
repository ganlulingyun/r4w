//! # Ionospheric Scintillation Analyzer
//!
//! Quantifies phase and amplitude scintillation indices (S4, σφ) on satellite
//! signals for ionospheric monitoring. Scintillation occurs when irregularities
//! in the ionospheric electron density cause rapid fluctuations in signal
//! amplitude and phase, degrading GNSS receiver performance.
//!
//! ## Key Metrics
//!
//! - **S4 index**: Normalized standard deviation of signal intensity (amplitude²).
//!   Values: 0–1+, where < 0.3 is weak, 0.3–0.6 moderate, > 0.6 strong.
//! - **σφ (sigma-phi)**: Standard deviation of detrended carrier phase (radians).
//!   Values: < 0.1 rad weak, 0.1–0.5 rad moderate, > 0.5 rad strong.
//! - **Spectral slope p**: Power-law exponent of the phase power spectral density.
//! - **Decorrelation time τ₀**: Time for intensity autocorrelation to drop to 1/e.
//! - **Fresnel frequency**: Related to the first Fresnel zone size and scan velocity.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ionospheric_scintillation_analyzer::{
//!     IonoScintillationAnalyzer, DetrendMethod, ScintillationSeverity,
//! };
//!
//! // Build an analyzer with 50 Hz sample rate and 1-second detrending window
//! let analyzer = IonoScintillationAnalyzer::builder()
//!     .sample_rate(50.0)
//!     .detrend_window_sec(1.0)
//!     .detrend_method(DetrendMethod::MovingAverage)
//!     .build();
//!
//! // Synthetic signal: constant amplitude, no scintillation
//! let n = 100;
//! let samples: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let phase = 0.1 * i as f64;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let report = analyzer.analyze(&samples);
//! assert!(report.s4 < 0.3, "Constant-amplitude signal should have low S4");
//! assert!(matches!(report.amplitude_severity, ScintillationSeverity::Weak));
//! ```

use std::f64::consts::PI;

// ─── public types ───────────────────────────────────────────────────────────

/// Method used to remove the low-frequency trend from amplitude / phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetrendMethod {
    /// Simple moving-average filter.
    MovingAverage,
    /// Polynomial fit (degree stored in `detrend_poly_order`).
    PolynomialFit,
}

/// Scintillation severity classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScintillationSeverity {
    /// S4 < 0.3 or σφ < 0.1 rad
    Weak,
    /// 0.3 ≤ S4 < 0.6 or 0.1 ≤ σφ < 0.5 rad
    Moderate,
    /// S4 ≥ 0.6 or σφ ≥ 0.5 rad
    Strong,
}

/// Full scintillation analysis report.
#[derive(Debug, Clone)]
pub struct ScintillationReport {
    /// Amplitude scintillation index (normalized intensity std-dev).
    pub s4: f64,
    /// Phase scintillation index (std-dev of detrended phase, radians).
    pub sigma_phi: f64,
    /// Classification based on S4.
    pub amplitude_severity: ScintillationSeverity,
    /// Classification based on σφ.
    pub phase_severity: ScintillationSeverity,
    /// Spectral slope *p* of phase PSD (typically 1.5–4).
    pub spectral_slope_p: f64,
    /// Decorrelation time τ₀ (seconds), `None` if not computable.
    pub decorrelation_time: Option<f64>,
    /// Estimated Fresnel frequency (Hz), `None` if not set.
    pub fresnel_frequency: Option<f64>,
    /// Fade depth CDF: `(fade_dB, probability_fade_exceeds)` pairs sorted by fade_dB.
    pub fade_depth_cdf: Vec<(f64, f64)>,
}

/// Builder for [`IonoScintillationAnalyzer`].
#[derive(Debug, Clone)]
pub struct AnalyzerBuilder {
    sample_rate: f64,
    detrend_window_sec: f64,
    detrend_method: DetrendMethod,
    detrend_poly_order: usize,
    hp_cutoff_hz: f64,
    scan_velocity: Option<f64>,
    ipp_height_m: Option<f64>,
}

impl Default for AnalyzerBuilder {
    fn default() -> Self {
        Self {
            sample_rate: 50.0,
            detrend_window_sec: 1.0,
            detrend_method: DetrendMethod::MovingAverage,
            detrend_poly_order: 6,
            hp_cutoff_hz: 0.1,
            scan_velocity: None,
            ipp_height_m: None,
        }
    }
}

impl AnalyzerBuilder {
    /// Sampling rate in Hz (default 50).
    pub fn sample_rate(mut self, hz: f64) -> Self {
        self.sample_rate = hz;
        self
    }

    /// Detrending window length in seconds (default 1.0).
    pub fn detrend_window_sec(mut self, sec: f64) -> Self {
        self.detrend_window_sec = sec;
        self
    }

    /// Detrending method (default `MovingAverage`).
    pub fn detrend_method(mut self, m: DetrendMethod) -> Self {
        self.detrend_method = m;
        self
    }

    /// Polynomial order when using `PolynomialFit` (default 6).
    pub fn detrend_poly_order(mut self, order: usize) -> Self {
        self.detrend_poly_order = order;
        self
    }

    /// High-pass cutoff for phase detrending in Hz (default 0.1).
    pub fn hp_cutoff_hz(mut self, hz: f64) -> Self {
        self.hp_cutoff_hz = hz;
        self
    }

    /// Ionospheric pierce-point scan velocity in m/s (for Fresnel freq).
    pub fn scan_velocity(mut self, v: f64) -> Self {
        self.scan_velocity = Some(v);
        self
    }

    /// Ionospheric pierce-point height in metres (for Fresnel freq).
    pub fn ipp_height_m(mut self, h: f64) -> Self {
        self.ipp_height_m = Some(h);
        self
    }

    /// Consume the builder and produce the analyzer.
    pub fn build(self) -> IonoScintillationAnalyzer {
        let detrend_window_samples =
            (self.detrend_window_sec * self.sample_rate).round().max(1.0) as usize;
        IonoScintillationAnalyzer {
            sample_rate: self.sample_rate,
            detrend_method: self.detrend_method,
            detrend_window_samples,
            detrend_poly_order: self.detrend_poly_order,
            hp_cutoff_hz: self.hp_cutoff_hz,
            scan_velocity: self.scan_velocity,
            ipp_height_m: self.ipp_height_m,
        }
    }
}

/// Ionospheric scintillation analyzer.
///
/// Computes S4, σφ, spectral slope, decorrelation time, Fresnel frequency,
/// and fade-depth statistics from a stream of complex IQ samples.
#[derive(Debug, Clone)]
pub struct IonoScintillationAnalyzer {
    sample_rate: f64,
    detrend_method: DetrendMethod,
    detrend_window_samples: usize,
    detrend_poly_order: usize,
    hp_cutoff_hz: f64,
    scan_velocity: Option<f64>,
    ipp_height_m: Option<f64>,
}

impl IonoScintillationAnalyzer {
    /// Create a builder with default parameters.
    pub fn builder() -> AnalyzerBuilder {
        AnalyzerBuilder::default()
    }

    /// Run a full scintillation analysis on the provided IQ samples.
    ///
    /// Each sample is an `(re, im)` tuple.  The vector should contain at least
    /// a few seconds of data at the configured sample rate for meaningful
    /// results.
    pub fn analyze(&self, samples: &[(f64, f64)]) -> ScintillationReport {
        if samples.is_empty() {
            return ScintillationReport {
                s4: 0.0,
                sigma_phi: 0.0,
                amplitude_severity: ScintillationSeverity::Weak,
                phase_severity: ScintillationSeverity::Weak,
                spectral_slope_p: 0.0,
                decorrelation_time: None,
                fresnel_frequency: None,
                fade_depth_cdf: Vec::new(),
            };
        }

        // ── amplitude & phase extraction ─────────────────────────────
        let amplitudes: Vec<f64> = samples
            .iter()
            .map(|&(re, im)| (re * re + im * im).sqrt())
            .collect();
        let phases: Vec<f64> = samples.iter().map(|&(re, im)| im.atan2(re)).collect();

        // ── S4: amplitude scintillation index ────────────────────────
        let detrended_amp = self.detrend_amplitude(&amplitudes);
        let s4 = Self::compute_s4(&detrended_amp, &amplitudes);

        // ── σφ: phase scintillation index ────────────────────────────
        let detrended_phase = self.detrend_phase(&phases);
        let sigma_phi = std_dev(&detrended_phase);

        // ── spectral slope p ─────────────────────────────────────────
        let spectral_slope_p = self.compute_spectral_slope(&detrended_phase);

        // ── decorrelation time ───────────────────────────────────────
        let intensities: Vec<f64> = detrended_amp.iter().map(|a| a * a).collect();
        let decorrelation_time = self.compute_decorrelation_time(&intensities);

        // ── Fresnel frequency ────────────────────────────────────────
        let fresnel_frequency = self.compute_fresnel_frequency();

        // ── fade depth CDF ───────────────────────────────────────────
        let fade_depth_cdf = Self::compute_fade_depth_cdf(&amplitudes);

        ScintillationReport {
            s4,
            sigma_phi,
            amplitude_severity: Self::classify_amplitude(s4),
            phase_severity: Self::classify_phase(sigma_phi),
            spectral_slope_p,
            decorrelation_time,
            fresnel_frequency,
            fade_depth_cdf,
        }
    }

    // ── S4 computation ──────────────────────────────────────────────────

    /// Compute S4 from detrended amplitudes and raw amplitudes.
    ///
    /// S4 = sqrt( (<I²> - <I>²) / <I>² ) where I = amplitude².
    /// We use the detrended amplitudes for the numerator to remove
    /// slow fading, and raw amplitudes for the normalizing denominator.
    fn compute_s4(detrended_amp: &[f64], raw_amp: &[f64]) -> f64 {
        if raw_amp.is_empty() {
            return 0.0;
        }
        let intensity: Vec<f64> = detrended_amp.iter().map(|a| a * a).collect();
        let mean_i = mean(&intensity);
        let mean_i2 = intensity.iter().map(|i| i * i).fold(0.0, |s, v| s + v)
            / intensity.len() as f64;

        let raw_intensity: Vec<f64> = raw_amp.iter().map(|a| a * a).collect();
        let mean_raw = mean(&raw_intensity);

        if mean_raw <= 0.0 {
            return 0.0;
        }
        let var = (mean_i2 - mean_i * mean_i).max(0.0);
        (var.sqrt() / mean_raw).min(2.0) // cap at 2.0 for numerical safety
    }

    /// Compute S4 index directly from a slice of amplitudes (no detrending).
    pub fn compute_s4_raw(amplitudes: &[f64]) -> f64 {
        if amplitudes.is_empty() {
            return 0.0;
        }
        let intensity: Vec<f64> = amplitudes.iter().map(|a| a * a).collect();
        let mean_i = mean(&intensity);
        let mean_i2 =
            intensity.iter().map(|i| i * i).fold(0.0, |s, v| s + v) / intensity.len() as f64;
        if mean_i <= 0.0 {
            return 0.0;
        }
        let var = (mean_i2 - mean_i * mean_i).max(0.0);
        (var / (mean_i * mean_i)).sqrt()
    }

    // ── amplitude detrending ────────────────────────────────────────────

    fn detrend_amplitude(&self, amplitudes: &[f64]) -> Vec<f64> {
        match self.detrend_method {
            DetrendMethod::MovingAverage => {
                let trend = moving_average(amplitudes, self.detrend_window_samples);
                amplitudes
                    .iter()
                    .zip(trend.iter())
                    .map(|(&a, &t)| if t > 1e-15 { a / t } else { 1.0 })
                    .collect()
            }
            DetrendMethod::PolynomialFit => {
                let trend = polyfit_trend(amplitudes, self.detrend_poly_order);
                amplitudes
                    .iter()
                    .zip(trend.iter())
                    .map(|(&a, &t)| if t.abs() > 1e-15 { a / t } else { 1.0 })
                    .collect()
            }
        }
    }

    // ── phase detrending ────────────────────────────────────────────────

    fn detrend_phase(&self, phases: &[f64]) -> Vec<f64> {
        // Unwrap phase first
        let unwrapped = unwrap_phase(phases);
        // Apply 6th-order Butterworth high-pass filter
        highpass_butterworth6(&unwrapped, self.hp_cutoff_hz, self.sample_rate)
    }

    // ── spectral slope ──────────────────────────────────────────────────

    fn compute_spectral_slope(&self, detrended_phase: &[f64]) -> f64 {
        let n = detrended_phase.len();
        if n < 8 {
            return 0.0;
        }
        // Compute single-sided PSD via DFT magnitude squared
        let nfft = n.next_power_of_two();
        let psd = psd_periodogram(detrended_phase, nfft, self.sample_rate);

        // Fit log-log line through upper-half frequencies (avoid DC)
        let start = nfft / 8;
        let end = nfft / 2;
        if end <= start + 2 {
            return 0.0;
        }
        let df = self.sample_rate / nfft as f64;
        let mut log_f = Vec::with_capacity(end - start);
        let mut log_p = Vec::with_capacity(end - start);
        for i in start..end {
            let f = (i as f64) * df;
            let p = psd[i];
            if f > 0.0 && p > 1e-30 {
                log_f.push(f.ln());
                log_p.push(p.ln());
            }
        }
        if log_f.len() < 3 {
            return 0.0;
        }
        // linear regression: log_p = -p * log_f + c  → slope = -p
        let (slope, _) = linear_regression(&log_f, &log_p);
        -slope // convention: PSD ∝ f^{-p}, so p = -slope
    }

    // ── decorrelation time ──────────────────────────────────────────────

    fn compute_decorrelation_time(&self, intensity: &[f64]) -> Option<f64> {
        if intensity.len() < 4 {
            return None;
        }
        let ac = autocorrelation_normalized(intensity);
        let dt = 1.0 / self.sample_rate;
        let threshold = 1.0 / std::f64::consts::E; // 1/e ≈ 0.368
        for (i, &val) in ac.iter().enumerate().skip(1) {
            if val <= threshold {
                // linear interpolation between i-1 and i
                let prev = ac[i - 1];
                let frac = if (prev - val).abs() > 1e-15 {
                    (prev - threshold) / (prev - val)
                } else {
                    0.5
                };
                return Some(((i - 1) as f64 + frac) * dt);
            }
        }
        None // autocorrelation never dropped below threshold
    }

    // ── Fresnel frequency ───────────────────────────────────────────────

    fn compute_fresnel_frequency(&self) -> Option<f64> {
        match (self.scan_velocity, self.ipp_height_m) {
            (Some(v), Some(h)) => {
                // GPS L1 wavelength ≈ 0.1903 m
                let lambda = 0.1903;
                // Fresnel zone size: r_f = sqrt(lambda * h)
                // Fresnel frequency: f_f = v / r_f
                let r_f = (lambda * h).sqrt();
                if r_f > 0.0 {
                    Some(v / r_f)
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    // ── fade depth CDF ──────────────────────────────────────────────────

    /// Compute the complementary CDF of signal fades.
    ///
    /// Returns `(fade_depth_dB, probability)` pairs, where
    /// `probability` is the fraction of samples whose fade exceeds that depth.
    fn compute_fade_depth_cdf(amplitudes: &[f64]) -> Vec<(f64, f64)> {
        if amplitudes.is_empty() {
            return Vec::new();
        }
        let mean_power: f64 =
            amplitudes.iter().map(|a| a * a).sum::<f64>() / amplitudes.len() as f64;
        if mean_power <= 0.0 {
            return Vec::new();
        }
        // Fade depth in dB for each sample: 10*log10(mean_power / sample_power)
        // positive values mean the sample is *below* the mean.
        let mut fades_db: Vec<f64> = amplitudes
            .iter()
            .filter_map(|&a| {
                let p = a * a;
                if p > 1e-30 {
                    Some(10.0 * (mean_power / p).log10())
                } else {
                    Some(60.0) // clamp deep fades
                }
            })
            .collect();
        fades_db.sort_by(|a, b| a.partial_cmp(b).unwrap());

        // Build complementary CDF at selected thresholds
        let thresholds: Vec<f64> = (-10..=30).map(|i| i as f64).collect();
        let n = fades_db.len() as f64;
        thresholds
            .iter()
            .map(|&thr| {
                let count = fades_db.iter().filter(|&&f| f >= thr).count();
                (thr, count as f64 / n)
            })
            .collect()
    }

    // ── severity classification ─────────────────────────────────────────

    /// Classify amplitude scintillation severity from S4.
    pub fn classify_amplitude(s4: f64) -> ScintillationSeverity {
        if s4 < 0.3 {
            ScintillationSeverity::Weak
        } else if s4 < 0.6 {
            ScintillationSeverity::Moderate
        } else {
            ScintillationSeverity::Strong
        }
    }

    /// Classify phase scintillation severity from σφ.
    pub fn classify_phase(sigma_phi: f64) -> ScintillationSeverity {
        if sigma_phi < 0.1 {
            ScintillationSeverity::Weak
        } else if sigma_phi < 0.5 {
            ScintillationSeverity::Moderate
        } else {
            ScintillationSeverity::Strong
        }
    }

    /// Return the configured sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Return the detrend window length in samples.
    pub fn detrend_window_samples(&self) -> usize {
        self.detrend_window_samples
    }
}

// ─── helper functions (private) ─────────────────────────────────────────────

fn mean(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    data.iter().sum::<f64>() / data.len() as f64
}

fn std_dev(data: &[f64]) -> f64 {
    if data.len() < 2 {
        return 0.0;
    }
    let m = mean(data);
    let var = data.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / data.len() as f64;
    var.sqrt()
}

/// Causal moving average with centered output.
fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if n == 0 || window == 0 {
        return vec![0.0; n];
    }
    let w = window.min(n);
    let half = w / 2;
    let mut out = Vec::with_capacity(n);
    // We use a symmetric window centered at each point, clamped at edges.
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = (i + w - half).min(n);
        let count = hi - lo;
        let sum: f64 = data[lo..hi].iter().sum();
        out.push(sum / count as f64);
    }
    out
}

/// Least-squares polynomial fit trend (returns evaluated trend values).
fn polyfit_trend(data: &[f64], order: usize) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return Vec::new();
    }
    let order = order.min(n.saturating_sub(1));
    // Normalize x to [-1, 1] for numerical stability
    let xs: Vec<f64> = (0..n)
        .map(|i| if n > 1 { 2.0 * i as f64 / (n - 1) as f64 - 1.0 } else { 0.0 })
        .collect();
    // Build Vandermonde-style normal equations: (AᵀA) c = Aᵀy
    let dim = order + 1;
    let mut ata = vec![0.0; dim * dim];
    let mut aty = vec![0.0; dim];
    for i in 0..n {
        let mut xpow = vec![1.0; dim];
        for j in 1..dim {
            xpow[j] = xpow[j - 1] * xs[i];
        }
        for r in 0..dim {
            for c in 0..dim {
                ata[r * dim + c] += xpow[r] * xpow[c];
            }
            aty[r] += xpow[r] * data[i];
        }
    }
    // Solve via Gaussian elimination with partial pivoting
    let coeffs = solve_linear_system(&mut ata, &mut aty, dim);
    // Evaluate polynomial
    xs.iter()
        .map(|&x| {
            let mut val = 0.0;
            let mut xp = 1.0;
            for &c in &coeffs {
                val += c * xp;
                xp *= x;
            }
            val
        })
        .collect()
}

/// Solve Ax = b in-place via Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let v = a[row * n + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_row != col {
            for c in 0..n {
                a.swap(col * n + c, max_row * n + c);
            }
            b.swap(col, max_row);
        }
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-15 {
            continue;
        }
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for c in col..n {
                let v = a[col * n + c];
                a[row * n + c] -= factor * v;
            }
            let bv = b[col];
            b[row] -= factor * bv;
        }
    }
    // Back substitution
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let pivot = a[row * n + row];
        if pivot.abs() < 1e-15 {
            x[row] = 0.0;
            continue;
        }
        let mut s = b[row];
        for c in (row + 1)..n {
            s -= a[row * n + c] * x[c];
        }
        x[row] = s / pivot;
    }
    x
}

/// Unwrap phase (remove 2π discontinuities).
fn unwrap_phase(phases: &[f64]) -> Vec<f64> {
    if phases.is_empty() {
        return Vec::new();
    }
    let mut out = Vec::with_capacity(phases.len());
    out.push(phases[0]);
    for i in 1..phases.len() {
        let mut d = phases[i] - phases[i - 1];
        while d > PI {
            d -= 2.0 * PI;
        }
        while d < -PI {
            d += 2.0 * PI;
        }
        out.push(out[i - 1] + d);
    }
    out
}

/// 6th-order Butterworth high-pass filter implemented as 3 cascaded biquads.
///
/// Applied forward-backward (filtfilt) for zero-phase.
fn highpass_butterworth6(data: &[f64], cutoff_hz: f64, sample_rate: f64) -> Vec<f64> {
    if data.len() < 4 || sample_rate <= 0.0 || cutoff_hz <= 0.0 {
        return data.to_vec();
    }
    // Pre-warp
    let wc = (PI * cutoff_hz / sample_rate).tan();
    // 6th order → 3 biquad sections
    let biquads = butterworth_hp_biquads(6, wc);

    // Forward-backward filtering
    let mut y = data.to_vec();
    for bq in &biquads {
        y = biquad_filter(bq, &y);
    }
    y.reverse();
    for bq in &biquads {
        y = biquad_filter(bq, &y);
    }
    y.reverse();
    y
}

/// High-pass Butterworth biquad sections via bilinear transform.
///
/// Returns `Vec<[f64; 6]>` where each `[b0,b1,b2,a0,a1,a2]`.
fn butterworth_hp_biquads(order: usize, wc: f64) -> Vec<[f64; 6]> {
    let n_sections = order / 2;
    let mut sections = Vec::with_capacity(n_sections);
    for k in 0..n_sections {
        // Analog prototype pole angle for Butterworth
        let angle = PI * (2 * k + 1) as f64 / (2 * order) as f64;
        let re = -angle.sin(); // real part of s-plane pole
        let _im = angle.cos(); // imaginary part

        // HP transform: H_HP(s) = s² / (s² - 2*re*wc*s + wc²)
        // Bilinear: s = 2*(z-1)/(z+1), let c = 2.0
        let c = 2.0;
        let wc2 = wc * wc;
        let two_re_wc = 2.0 * re * wc;

        // Numerator of H_HP in z: c²(z-1)²
        let nb0 = c * c;
        let nb1 = -2.0 * c * c;
        let nb2 = c * c;

        // Denominator: c²(z-1)² - 2*re*wc*c*(z-1)(z+1) + wc²*(z+1)²
        let da0 = c * c - two_re_wc * c + wc2;
        let da1 = -2.0 * c * c + 2.0 * wc2;
        let da2 = c * c + two_re_wc * c + wc2;

        // Normalize so a0 = 1
        sections.push([
            nb0 / da0,
            nb1 / da0,
            nb2 / da0,
            1.0,
            da1 / da0,
            da2 / da0,
        ]);
    }
    sections
}

/// Apply a single biquad filter section `[b0,b1,b2,1,a1,a2]` using Direct Form II Transposed.
fn biquad_filter(coeffs: &[f64; 6], data: &[f64]) -> Vec<f64> {
    let (b0, b1, b2) = (coeffs[0], coeffs[1], coeffs[2]);
    let (a1, a2) = (coeffs[4], coeffs[5]);
    let mut d1 = 0.0;
    let mut d2 = 0.0;
    let mut out = Vec::with_capacity(data.len());
    for &x in data {
        let y = b0 * x + d1;
        d1 = b1 * x - a1 * y + d2;
        d2 = b2 * x - a2 * y;
        out.push(y);
    }
    out
}

/// Compute a single-sided power spectral density via periodogram.
fn psd_periodogram(data: &[f64], nfft: usize, sample_rate: f64) -> Vec<f64> {
    // Zero-pad to nfft
    let mut buf = vec![0.0; nfft];
    let n = data.len().min(nfft);
    buf[..n].copy_from_slice(&data[..n]);

    // Apply Hann window
    for i in 0..n {
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos());
        buf[i] *= w;
    }

    // DFT (real input → only need first nfft/2+1 bins)
    let half = nfft / 2 + 1;
    let mut psd = vec![0.0; half];
    for k in 0..half {
        let mut re = 0.0;
        let mut im = 0.0;
        for i in 0..nfft {
            let angle = 2.0 * PI * k as f64 * i as f64 / nfft as f64;
            re += buf[i] * angle.cos();
            im -= buf[i] * angle.sin();
        }
        psd[k] = (re * re + im * im) / (sample_rate * nfft as f64);
    }
    // Double one-sided (except DC and Nyquist)
    for i in 1..(half - 1) {
        psd[i] *= 2.0;
    }
    psd
}

/// Normalized autocorrelation: R[k] / R[0].
fn autocorrelation_normalized(data: &[f64]) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return Vec::new();
    }
    let m = mean(data);
    let centered: Vec<f64> = data.iter().map(|x| x - m).collect();
    let max_lag = n / 2;
    let mut ac = Vec::with_capacity(max_lag);
    for lag in 0..max_lag {
        let mut s = 0.0;
        for i in 0..(n - lag) {
            s += centered[i] * centered[i + lag];
        }
        ac.push(s);
    }
    let r0 = if ac[0].abs() > 1e-30 { ac[0] } else { 1.0 };
    ac.iter().map(|&v| v / r0).collect()
}

/// Simple linear regression: y = slope*x + intercept.
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    if n < 2.0 {
        return (0.0, 0.0);
    }
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(a, b)| a * b).sum();
    let sxx: f64 = x.iter().map(|a| a * a).sum();
    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n);
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (slope, intercept)
}

// ─── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a constant-amplitude IQ signal with linear phase ramp.
    fn constant_signal(n: usize, amplitude: f64, phase_rate: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = phase_rate * i as f64;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    /// Helper: simple PRNG (xorshift64) for deterministic tests.
    fn xorshift64(state: &mut u64) -> f64 {
        *state ^= *state << 13;
        *state ^= *state >> 7;
        *state ^= *state << 17;
        (*state as f64) / (u64::MAX as f64)
    }

    fn default_analyzer() -> IonoScintillationAnalyzer {
        IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .detrend_window_sec(1.0)
            .build()
    }

    // ── 1. constant-amplitude → low S4 ─────────────────────────────

    #[test]
    fn test_constant_amplitude_low_s4() {
        let analyzer = default_analyzer();
        let signal = constant_signal(500, 1.0, 0.05);
        let report = analyzer.analyze(&signal);
        assert!(
            report.s4 < 0.05,
            "S4 for constant amplitude should be near zero, got {}",
            report.s4
        );
    }

    // ── 2. constant amplitude → weak severity ──────────────────────

    #[test]
    fn test_constant_amplitude_weak_severity() {
        let analyzer = default_analyzer();
        let signal = constant_signal(500, 1.0, 0.05);
        let report = analyzer.analyze(&signal);
        assert_eq!(report.amplitude_severity, ScintillationSeverity::Weak);
    }

    // ── 3. σφ low for linear phase ──────────────────────────────────

    #[test]
    fn test_linear_phase_low_sigma_phi() {
        // Use a very slow phase rate (0.01 Hz) well below the 0.1 Hz HP cutoff
        // so the detrending filter removes it, and a long signal to reduce edge effects.
        let analyzer = IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .detrend_window_sec(1.0)
            .hp_cutoff_hz(0.1)
            .build();
        // phase_rate = 2*pi*0.01 / 50 = 0.001257 rad/sample (0.01 Hz carrier)
        let signal = constant_signal(2500, 1.0, 0.001257);
        let report = analyzer.analyze(&signal);
        assert!(
            report.sigma_phi < 0.15,
            "sigma_phi for slow linear phase should be low, got {}",
            report.sigma_phi
        );
    }

    // ── 4. classify_amplitude thresholds ────────────────────────────

    #[test]
    fn test_classify_amplitude() {
        assert_eq!(
            IonoScintillationAnalyzer::classify_amplitude(0.0),
            ScintillationSeverity::Weak
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_amplitude(0.29),
            ScintillationSeverity::Weak
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_amplitude(0.3),
            ScintillationSeverity::Moderate
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_amplitude(0.59),
            ScintillationSeverity::Moderate
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_amplitude(0.6),
            ScintillationSeverity::Strong
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_amplitude(1.2),
            ScintillationSeverity::Strong
        );
    }

    // ── 5. classify_phase thresholds ────────────────────────────────

    #[test]
    fn test_classify_phase() {
        assert_eq!(
            IonoScintillationAnalyzer::classify_phase(0.0),
            ScintillationSeverity::Weak
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_phase(0.09),
            ScintillationSeverity::Weak
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_phase(0.1),
            ScintillationSeverity::Moderate
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_phase(0.49),
            ScintillationSeverity::Moderate
        );
        assert_eq!(
            IonoScintillationAnalyzer::classify_phase(0.5),
            ScintillationSeverity::Strong
        );
    }

    // ── 6. empty input ──────────────────────────────────────────────

    #[test]
    fn test_empty_input() {
        let analyzer = default_analyzer();
        let report = analyzer.analyze(&[]);
        assert_eq!(report.s4, 0.0);
        assert_eq!(report.sigma_phi, 0.0);
        assert!(report.fade_depth_cdf.is_empty());
    }

    // ── 7. moving average detrending preserves length ───────────────

    #[test]
    fn test_moving_average_length() {
        let data: Vec<f64> = (0..100).map(|i| 1.0 + 0.01 * i as f64).collect();
        let ma = moving_average(&data, 10);
        assert_eq!(ma.len(), data.len());
    }

    // ── 8. polyfit_trend basic sanity ───────────────────────────────

    #[test]
    fn test_polyfit_trend_linear() {
        // Fit a linear trend to y = 2x + 3
        let data: Vec<f64> = (0..50).map(|i| 2.0 * i as f64 + 3.0).collect();
        let trend = polyfit_trend(&data, 1);
        assert_eq!(trend.len(), data.len());
        // Should match closely
        for (i, (&d, &t)) in data.iter().zip(trend.iter()).enumerate() {
            assert!(
                (d - t).abs() < 0.1,
                "Mismatch at {}: data={}, trend={}",
                i,
                d,
                t
            );
        }
    }

    // ── 9. unwrap_phase removes discontinuities ─────────────────────

    #[test]
    fn test_unwrap_phase() {
        // Phase that wraps around
        let phases: Vec<f64> = (0..100).map(|i| (0.1 * i as f64) % (2.0 * PI) - PI).collect();
        let unwrapped = unwrap_phase(&phases);
        // Should be monotonically increasing (since input increases before wrapping)
        for i in 1..unwrapped.len() {
            let diff = unwrapped[i] - unwrapped[i - 1];
            assert!(
                diff.abs() < PI + 0.01,
                "Phase jump at {}: {}",
                i,
                diff
            );
        }
    }

    // ── 10. S4 raw computation ──────────────────────────────────────

    #[test]
    fn test_s4_raw_constant() {
        let amps = vec![1.0; 100];
        let s4 = IonoScintillationAnalyzer::compute_s4_raw(&amps);
        assert!(s4.abs() < 1e-10, "S4 of constant should be 0, got {}", s4);
    }

    // ── 11. S4 raw with variation ───────────────────────────────────

    #[test]
    fn test_s4_raw_with_variation() {
        // Alternating amplitudes → non-zero S4
        let amps: Vec<f64> = (0..200).map(|i| if i % 2 == 0 { 1.5 } else { 0.5 }).collect();
        let s4 = IonoScintillationAnalyzer::compute_s4_raw(&amps);
        assert!(s4 > 0.3, "S4 should be significant for alternating amplitudes, got {}", s4);
    }

    // ── 12. autocorrelation_normalized at lag 0 is 1.0 ─────────────

    #[test]
    fn test_autocorrelation_normalized_lag0() {
        let data: Vec<f64> = (0..100).map(|i| (0.1 * i as f64).sin()).collect();
        let ac = autocorrelation_normalized(&data);
        assert!(!ac.is_empty());
        assert!(
            (ac[0] - 1.0).abs() < 1e-10,
            "Autocorrelation at lag 0 should be 1.0, got {}",
            ac[0]
        );
    }

    // ── 13. Fresnel frequency computation ───────────────────────────

    #[test]
    fn test_fresnel_frequency() {
        let analyzer = IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .scan_velocity(100.0) // 100 m/s
            .ipp_height_m(350_000.0) // 350 km
            .build();
        let ff = analyzer.compute_fresnel_frequency();
        assert!(ff.is_some());
        let ff = ff.unwrap();
        // r_f = sqrt(0.1903 * 350000) ≈ 258 m, f_f = 100/258 ≈ 0.387 Hz
        assert!(
            (ff - 0.387).abs() < 0.05,
            "Fresnel frequency should be ~0.387 Hz, got {}",
            ff
        );
    }

    // ── 14. Fresnel frequency None without parameters ───────────────

    #[test]
    fn test_fresnel_frequency_none() {
        let analyzer = default_analyzer();
        assert!(analyzer.compute_fresnel_frequency().is_none());
    }

    // ── 15. fade depth CDF is non-empty for valid signal ────────────

    #[test]
    fn test_fade_depth_cdf_nonempty() {
        let amps: Vec<f64> = (0..200)
            .map(|i| 1.0 + 0.3 * (0.1 * i as f64).sin())
            .collect();
        let cdf = IonoScintillationAnalyzer::compute_fade_depth_cdf(&amps);
        assert!(!cdf.is_empty());
        // At very negative fade threshold, all samples should exceed it
        let (_, prob_at_neg10) = cdf[0]; // first entry is -10 dB
        assert!(
            prob_at_neg10 > 0.9,
            "Most samples should exceed -10 dB fade, got {}",
            prob_at_neg10
        );
    }

    // ── 16. builder defaults ────────────────────────────────────────

    #[test]
    fn test_builder_defaults() {
        let a = IonoScintillationAnalyzer::builder().build();
        assert_eq!(a.sample_rate(), 50.0);
        assert_eq!(a.detrend_window_samples(), 50); // 1.0 sec * 50 Hz
    }

    // ── 17. polynomial detrend method ───────────────────────────────

    #[test]
    fn test_polynomial_detrend_low_s4() {
        let analyzer = IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .detrend_method(DetrendMethod::PolynomialFit)
            .detrend_poly_order(3)
            .build();
        let signal = constant_signal(500, 1.0, 0.05);
        let report = analyzer.analyze(&signal);
        assert!(
            report.s4 < 0.1,
            "S4 with poly detrend should be low for constant amp, got {}",
            report.s4
        );
    }

    // ── 18. spectral slope is finite for phase scintillation ────────

    #[test]
    fn test_spectral_slope_finite() {
        // Generate a signal with deliberate phase fluctuations
        let mut rng: u64 = 0xDEAD_BEEF_1234;
        let n = 512;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 0.05 * i as f64 + 0.5 * (xorshift64(&mut rng) - 0.5);
                (phase.cos(), phase.sin())
            })
            .collect();
        let analyzer = IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .detrend_window_sec(1.0)
            .build();
        let report = analyzer.analyze(&signal);
        assert!(
            report.spectral_slope_p.is_finite(),
            "Spectral slope should be finite, got {}",
            report.spectral_slope_p
        );
    }

    // ── 19. decorrelation time for white noise is short ─────────────

    #[test]
    fn test_decorrelation_time_white_noise() {
        let mut rng: u64 = 0xCAFE_BABE_5678;
        let n = 1000;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|_| {
                let a = 0.5 + 0.5 * xorshift64(&mut rng);
                let p = 2.0 * PI * xorshift64(&mut rng);
                (a * p.cos(), a * p.sin())
            })
            .collect();
        let analyzer = IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .detrend_window_sec(0.5)
            .build();
        let report = analyzer.analyze(&signal);
        // White noise → decorrelation should happen quickly (a few samples)
        if let Some(tau) = report.decorrelation_time {
            assert!(
                tau < 1.0,
                "Decorrelation time for noise should be short, got {} s",
                tau
            );
        }
        // It's also OK if None (never dropped below threshold for short data)
    }

    // ── 20. linear regression sanity ────────────────────────────────

    #[test]
    fn test_linear_regression() {
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| 2.5 * xi + 1.0).collect();
        let (slope, intercept) = linear_regression(&x, &y);
        assert!(
            (slope - 2.5).abs() < 1e-10,
            "Slope should be 2.5, got {}",
            slope
        );
        assert!(
            (intercept - 1.0).abs() < 1e-10,
            "Intercept should be 1.0, got {}",
            intercept
        );
    }

    // ── 21. biquad filter identity ──────────────────────────────────

    #[test]
    fn test_biquad_identity() {
        // Identity filter: y = x
        let coeffs = [1.0, 0.0, 0.0, 1.0, 0.0, 0.0];
        let data: Vec<f64> = (0..50).map(|i| (0.2 * i as f64).sin()).collect();
        let out = biquad_filter(&coeffs, &data);
        for (i, (&d, &o)) in data.iter().zip(out.iter()).enumerate() {
            assert!(
                (d - o).abs() < 1e-12,
                "Identity filter mismatch at {}: {} vs {}",
                i,
                d,
                o
            );
        }
    }

    // ── 22. high-amplitude scintillation → elevated S4 ─────────────

    #[test]
    fn test_high_amplitude_scintillation() {
        // Signal with severe amplitude fluctuations
        let n = 1000;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / 50.0;
                // Deep fades at ~1 Hz
                let amp = (0.5 + 0.5 * (2.0 * PI * 1.0 * t).cos()).max(0.01);
                let phase = 0.1 * t;
                (amp * phase.cos(), amp * phase.sin())
            })
            .collect();
        let analyzer = IonoScintillationAnalyzer::builder()
            .sample_rate(50.0)
            .detrend_window_sec(0.5)
            .build();
        let report = analyzer.analyze(&signal);
        assert!(
            report.s4 > 0.2,
            "S4 should be elevated for scintillating signal, got {}",
            report.s4
        );
    }
}
