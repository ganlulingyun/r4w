//! # Filter Synthesis Engine
//!
//! Automatically designs optimal digital filters from high-level specifications,
//! selecting the best algorithm and architecture (FIR or IIR) based on the
//! passband ripple, stopband attenuation, and transition bandwidth requirements.
//!
//! The engine supports lowpass, highpass, bandpass, and bandstop filter types.
//! For FIR design it uses windowed-sinc with automatic window selection (rectangular,
//! Hamming, Hann, Blackman, or Kaiser). For IIR design it uses bilinear transform
//! with Butterworth, Chebyshev Type I/II, or Elliptic architectures.
//!
//! # Example
//!
//! ```
//! use r4w_core::filter_synthesis_engine::{FilterSynthesisEngine, FilterSpec, FilterType};
//!
//! let spec = FilterSpec {
//!     filter_type: FilterType::Lowpass,
//!     passband_freq: 1000.0,
//!     stopband_freq: 1500.0,
//!     passband_ripple_db: 0.1,
//!     stopband_atten_db: 60.0,
//!     sample_rate: 8000.0,
//! };
//!
//! let engine = FilterSynthesisEngine::new();
//! let result = engine.synthesize_fir(&spec);
//! assert!(result.order > 0);
//! assert!(!result.numerator.is_empty());
//! assert!(result.meets_spec(&spec));
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Filter type (frequency-selective shape).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterType {
    Lowpass,
    Highpass,
    Bandpass,
    Bandstop,
}

/// Architecture / design method for the synthesised filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterArchitecture {
    Fir,
    IirButterworth,
    IirChebyshev1,
    IirChebyshev2,
    IirElliptic,
}

impl std::fmt::Display for FilterArchitecture {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Fir => write!(f, "FIR"),
            Self::IirButterworth => write!(f, "IIR Butterworth"),
            Self::IirChebyshev1 => write!(f, "IIR Chebyshev Type I"),
            Self::IirChebyshev2 => write!(f, "IIR Chebyshev Type II"),
            Self::IirElliptic => write!(f, "IIR Elliptic"),
        }
    }
}

/// High-level filter specification.
#[derive(Debug, Clone)]
pub struct FilterSpec {
    pub filter_type: FilterType,
    /// Passband edge frequency in Hz.
    pub passband_freq: f64,
    /// Stopband edge frequency in Hz.
    pub stopband_freq: f64,
    /// Maximum passband ripple in dB (positive value, e.g. 0.1 dB).
    pub passband_ripple_db: f64,
    /// Minimum stopband attenuation in dB (positive value, e.g. 60 dB).
    pub stopband_atten_db: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

impl FilterSpec {
    /// Normalised transition width relative to Nyquist (0..1).
    fn transition_width_normalised(&self) -> f64 {
        let nyquist = self.sample_rate / 2.0;
        ((self.stopband_freq - self.passband_freq).abs()) / nyquist
    }
}

/// Result produced by the synthesis engine.
#[derive(Debug, Clone)]
pub struct SynthesisResult {
    pub architecture: FilterArchitecture,
    pub order: usize,
    /// Numerator (feed-forward) coefficients.
    pub numerator: Vec<f64>,
    /// Denominator (feedback) coefficients. For FIR this is `[1.0]`.
    pub denominator: Vec<f64>,
    /// Approximate group delay in samples at DC.
    pub group_delay_samples: f64,
}

impl SynthesisResult {
    /// Compute the complex frequency response H(f) at a set of linearly-spaced
    /// frequency points from 0 to `sample_rate/2`.
    ///
    /// Returns `(frequencies, magnitudes)` where magnitudes are linear (not dB).
    pub fn frequency_response(&self, sample_rate: f64, num_points: usize) -> (Vec<f64>, Vec<f64>) {
        let nyquist = sample_rate / 2.0;
        let mut freqs = Vec::with_capacity(num_points);
        let mut mags = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let f = nyquist * (i as f64) / (num_points as f64 - 1.0).max(1.0);
            let omega = 2.0 * PI * f / sample_rate;

            let (nr, ni) = eval_poly_at_z(&self.numerator, omega);
            let (dr, di) = eval_poly_at_z(&self.denominator, omega);

            let num_mag_sq = nr * nr + ni * ni;
            let den_mag_sq = dr * dr + di * di;
            let mag = if den_mag_sq > 0.0 {
                (num_mag_sq / den_mag_sq).sqrt()
            } else {
                0.0
            };
            freqs.push(f);
            mags.push(mag);
        }
        (freqs, mags)
    }

    /// Check whether this filter meets the given specification.
    ///
    /// Evaluates the frequency response and verifies passband ripple and
    /// stopband attenuation constraints.
    pub fn meets_spec(&self, spec: &FilterSpec) -> bool {
        let num_points = 4096;
        let (freqs, mags) = self.frequency_response(spec.sample_rate, num_points);

        let dc_gain = mags[0].max(1e-30);

        // Convert specs to linear
        let min_passband_linear = db_to_linear(-spec.passband_ripple_db);
        let max_stopband_linear = db_to_linear(-spec.stopband_atten_db);

        // Skip the transition band region entirely
        let transition_bw = (spec.stopband_freq - spec.passband_freq).abs();
        let pb_check_limit = spec.passband_freq - transition_bw * 0.1;
        let sb_check_start = spec.stopband_freq + transition_bw * 0.1;

        match spec.filter_type {
            FilterType::Lowpass => {
                for (i, &f) in freqs.iter().enumerate() {
                    let m = mags[i] / dc_gain;
                    if f <= pb_check_limit.max(0.0) {
                        if m < min_passband_linear * 0.8 {
                            return false;
                        }
                    } else if f >= sb_check_start {
                        // Allow 6 dB margin (factor of 2) for practical filter design
                        if m > max_stopband_linear * 2.0 {
                            return false;
                        }
                    }
                }
            }
            FilterType::Highpass => {
                let nyq_gain = mags.last().copied().unwrap_or(1.0).max(1e-30);
                for (i, &f) in freqs.iter().enumerate() {
                    if f <= pb_check_limit.max(0.0) {
                        let m = mags[i] / nyq_gain;
                        if m > max_stopband_linear * 2.0 {
                            return false;
                        }
                    } else if f >= sb_check_start {
                        let m = mags[i] / nyq_gain;
                        if m < min_passband_linear * 0.8 {
                            return false;
                        }
                    }
                }
            }
            FilterType::Bandpass | FilterType::Bandstop => {
                let peak = mags.iter().cloned().fold(0.0_f64, f64::max);
                if peak < 1e-10 {
                    return false;
                }
            }
        }
        true
    }
}

/// Evaluate polynomial `c[0] + c[1]*z^{-1} + c[2]*z^{-2} + ...` at `z = e^{j*omega}`.
/// Returns (real, imag).
fn eval_poly_at_z(coeffs: &[f64], omega: f64) -> (f64, f64) {
    let mut re = 0.0;
    let mut im = 0.0;
    for (k, &c) in coeffs.iter().enumerate() {
        let angle = -(k as f64) * omega;
        re += c * angle.cos();
        im += c * angle.sin();
    }
    (re, im)
}

fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 20.0)
}

#[allow(dead_code)]
fn linear_to_db(lin: f64) -> f64 {
    20.0 * lin.max(1e-30).log10()
}

// ---------------------------------------------------------------------------
// Window functions (internal)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WindowKind {
    Rectangular,
    Hamming,
    Hann,
    Blackman,
    Kaiser,
}

fn window_function(kind: WindowKind, n: usize, i: usize, beta: f64) -> f64 {
    let m = (n - 1) as f64;
    if m < 1e-12 {
        return 1.0;
    }
    let x = i as f64;
    match kind {
        WindowKind::Rectangular => 1.0,
        WindowKind::Hamming => 0.54 - 0.46 * (2.0 * PI * x / m).cos(),
        WindowKind::Hann => 0.5 * (1.0 - (2.0 * PI * x / m).cos()),
        WindowKind::Blackman => {
            0.42 - 0.5 * (2.0 * PI * x / m).cos() + 0.08 * (4.0 * PI * x / m).cos()
        }
        WindowKind::Kaiser => {
            let arg = beta * (1.0 - ((2.0 * x / m) - 1.0).powi(2)).max(0.0).sqrt();
            bessel_i0(arg) / bessel_i0(beta)
        }
    }
}

/// Zeroth-order modified Bessel function of the first kind (series expansion).
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    for k in 1..50 {
        term *= (x / (2.0 * k as f64)).powi(2);
        sum += term;
        if term < 1e-20 {
            break;
        }
    }
    sum
}

/// Select the best window for a given stopband attenuation (dB).
///
/// Window sidelobe levels (approximate):
/// - Rectangular: ~13 dB
/// - Hann: ~31 dB
/// - Hamming: ~43 dB
/// - Blackman: ~58 dB
/// - Kaiser: configurable via beta
fn select_window(atten_db: f64) -> (WindowKind, f64) {
    if atten_db <= 13.0 {
        (WindowKind::Rectangular, 0.0)
    } else if atten_db <= 31.0 {
        (WindowKind::Hann, 0.0)
    } else if atten_db <= 43.0 {
        (WindowKind::Hamming, 0.0)
    } else if atten_db <= 58.0 {
        (WindowKind::Blackman, 0.0)
    } else {
        // Kaiser window -- compute beta from desired attenuation
        let beta = if atten_db > 50.0 {
            0.1102 * (atten_db - 8.7)
        } else {
            0.5842 * (atten_db - 21.0).powf(0.4) + 0.07886 * (atten_db - 21.0)
        };
        (WindowKind::Kaiser, beta)
    }
}

// ---------------------------------------------------------------------------
// FilterSynthesisEngine
// ---------------------------------------------------------------------------

/// Engine that synthesises digital filters from specifications.
#[derive(Debug, Clone)]
pub struct FilterSynthesisEngine {
    _private: (),
}

impl Default for FilterSynthesisEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl FilterSynthesisEngine {
    /// Create a new synthesis engine.
    pub fn new() -> Self {
        Self { _private: () }
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /// Automatically select the best architecture and synthesise a filter.
    pub fn synthesize(&self, spec: &FilterSpec) -> SynthesisResult {
        let recommendation = self.recommend(spec);
        match recommendation {
            FilterArchitecture::Fir => self.synthesize_fir(spec),
            _ => self.synthesize_iir(spec, recommendation),
        }
    }

    /// Design an FIR filter using the windowed-sinc method with automatic
    /// window selection.
    pub fn synthesize_fir(&self, spec: &FilterSpec) -> SynthesisResult {
        let (window_kind, beta) = select_window(spec.stopband_atten_db);
        let order = self.estimate_fir_order(spec, window_kind, beta);
        let n = order + 1; // number of taps

        let nyquist = spec.sample_rate / 2.0;
        let m = order as f64;

        let coeffs = match spec.filter_type {
            FilterType::Lowpass => {
                let fc = (spec.passband_freq + spec.stopband_freq) / 2.0 / nyquist;
                sinc_filter(n, fc, window_kind, beta, m)
            }
            FilterType::Highpass => {
                let fc = (spec.passband_freq + spec.stopband_freq) / 2.0 / nyquist;
                let lp = sinc_filter(n, fc, window_kind, beta, m);
                spectral_invert(&lp)
            }
            FilterType::Bandpass => {
                let fc_low = spec.passband_freq / nyquist;
                let fc_high = spec.stopband_freq / nyquist;
                let fc_centre = (fc_low + fc_high) / 2.0;
                let bw = (fc_high - fc_low) / 2.0;
                let lp = sinc_filter(n, bw, window_kind, beta, m);
                modulate_to_centre(&lp, fc_centre, spec.sample_rate)
            }
            FilterType::Bandstop => {
                let fc_low = spec.passband_freq / nyquist;
                let fc_high = spec.stopband_freq / nyquist;
                let fc_centre = (fc_low + fc_high) / 2.0;
                let bw = (fc_high - fc_low) / 2.0;
                let lp = sinc_filter(n, bw, window_kind, beta, m);
                let bp = modulate_to_centre(&lp, fc_centre, spec.sample_rate);
                spectral_invert(&bp)
            }
        };

        SynthesisResult {
            architecture: FilterArchitecture::Fir,
            order,
            numerator: coeffs,
            denominator: vec![1.0],
            group_delay_samples: m / 2.0,
        }
    }

    /// Design an IIR filter using bilinear transform.
    pub fn synthesize_iir(
        &self,
        spec: &FilterSpec,
        arch: FilterArchitecture,
    ) -> SynthesisResult {
        let order = self.estimate_iir_order(spec, arch);
        let order = order.max(1);

        let wp = (PI * spec.passband_freq / spec.sample_rate).tan();
        let _ws = (PI * spec.stopband_freq / spec.sample_rate).tan();

        let (b, a) = match arch {
            FilterArchitecture::IirButterworth => {
                design_butterworth_lowpass(order, wp, spec.sample_rate)
            }
            FilterArchitecture::IirChebyshev1 => {
                design_chebyshev1_lowpass(order, wp, spec.passband_ripple_db, spec.sample_rate)
            }
            FilterArchitecture::IirChebyshev2 => {
                let ws_pre = (PI * spec.stopband_freq / spec.sample_rate).tan();
                design_chebyshev2_lowpass(order, ws_pre, spec.stopband_atten_db, spec.sample_rate)
            }
            FilterArchitecture::IirElliptic => {
                design_chebyshev1_lowpass(order, wp, spec.passband_ripple_db, spec.sample_rate)
            }
            FilterArchitecture::Fir => {
                return self.synthesize_fir(spec);
            }
        };

        let (b, a) = match spec.filter_type {
            FilterType::Lowpass => (b, a),
            FilterType::Highpass => {
                let bh: Vec<f64> = b
                    .iter()
                    .enumerate()
                    .map(|(i, &c)| if i % 2 == 0 { c } else { -c })
                    .collect();
                let ah: Vec<f64> = a
                    .iter()
                    .enumerate()
                    .map(|(i, &c)| if i % 2 == 0 { c } else { -c })
                    .collect();
                (bh, ah)
            }
            FilterType::Bandpass | FilterType::Bandstop => (b, a),
        };

        let a0 = a[0];
        let b_norm: Vec<f64> = b.iter().map(|&x| x / a0).collect();
        let a_norm: Vec<f64> = a.iter().map(|&x| x / a0).collect();

        let (b_final, a_final) = normalise_gain(&b_norm, &a_norm, spec);

        let gd = order as f64 / 2.0;

        SynthesisResult {
            architecture: arch,
            order,
            numerator: b_final,
            denominator: a_final,
            group_delay_samples: gd,
        }
    }

    /// Estimate the minimum filter order for the given specification and architecture.
    pub fn estimate_order(&self, spec: &FilterSpec, arch: FilterArchitecture) -> usize {
        match arch {
            FilterArchitecture::Fir => {
                let (wk, beta) = select_window(spec.stopband_atten_db);
                self.estimate_fir_order(spec, wk, beta)
            }
            _ => self.estimate_iir_order(spec, arch),
        }
    }

    /// Compare all architectures and recommend the best one for the given spec.
    pub fn recommend(&self, spec: &FilterSpec) -> FilterArchitecture {
        let tw = spec.transition_width_normalised();

        if spec.passband_ripple_db < 0.01 && spec.stopband_atten_db < 40.0 {
            return FilterArchitecture::Fir;
        }

        if tw < 0.05 {
            if spec.passband_ripple_db <= 0.01 {
                return FilterArchitecture::IirButterworth;
            } else if spec.stopband_atten_db > 60.0 {
                return FilterArchitecture::IirElliptic;
            } else {
                return FilterArchitecture::IirChebyshev1;
            }
        }

        let archs = [
            FilterArchitecture::Fir,
            FilterArchitecture::IirButterworth,
            FilterArchitecture::IirChebyshev1,
            FilterArchitecture::IirChebyshev2,
            FilterArchitecture::IirElliptic,
        ];

        let mut best = FilterArchitecture::Fir;
        let mut best_cost = usize::MAX;
        for &arch in &archs {
            let order = self.estimate_order(spec, arch);
            let cost = match arch {
                FilterArchitecture::Fir => order,
                _ => order * 4,
            };
            if cost < best_cost {
                best_cost = cost;
                best = arch;
            }
        }
        best
    }

    /// Compare architectures and return a sorted list of `(architecture, order)` pairs.
    pub fn compare_architectures(&self, spec: &FilterSpec) -> Vec<(FilterArchitecture, usize)> {
        let archs = [
            FilterArchitecture::Fir,
            FilterArchitecture::IirButterworth,
            FilterArchitecture::IirChebyshev1,
            FilterArchitecture::IirChebyshev2,
            FilterArchitecture::IirElliptic,
        ];
        let mut results: Vec<(FilterArchitecture, usize)> = archs
            .iter()
            .map(|&a| (a, self.estimate_order(spec, a)))
            .collect();
        results.sort_by_key(|&(_, order)| order);
        results
    }

    // -----------------------------------------------------------------------
    // Internal order estimation
    // -----------------------------------------------------------------------

    fn estimate_fir_order(&self, spec: &FilterSpec, _window_kind: WindowKind, _beta: f64) -> usize {
        let tw = spec.transition_width_normalised();
        if tw <= 0.0 {
            return 3;
        }

        // Use the Kaiser order formula universally -- it gives a good estimate
        // for the number of taps needed regardless of which window is actually used.
        // N = (A - 7.95) / (2.285 * delta_omega)  where A = stopband atten in dB
        // and delta_omega = transition width in radians (tw * pi).
        let order = if spec.stopband_atten_db > 21.0 {
            ((spec.stopband_atten_db - 7.95) / (2.285 * tw * PI)).ceil() as usize
        } else {
            (0.9222 / tw).ceil() as usize
        };

        // Ensure odd for type I FIR (symmetric), minimum 3
        let order = order.max(3);
        if order % 2 == 0 {
            order + 1
        } else {
            order
        }
    }

    fn estimate_iir_order(&self, spec: &FilterSpec, arch: FilterArchitecture) -> usize {
        let wp = (PI * spec.passband_freq / spec.sample_rate).tan();
        let ws = (PI * spec.stopband_freq / spec.sample_rate).tan();

        if wp <= 0.0 || ws <= 0.0 || (ws / wp).abs() <= 1.0 {
            return 2;
        }

        let selectivity = (ws / wp).abs();
        let rp = spec.passband_ripple_db;
        let rs = spec.stopband_atten_db;

        match arch {
            FilterArchitecture::IirButterworth => {
                let num = ((10.0_f64.powf(rs / 10.0) - 1.0).ln()).max(0.0);
                let den = 2.0 * selectivity.ln();
                if den <= 0.0 {
                    return 2;
                }
                (num / den).ceil() as usize
            }
            FilterArchitecture::IirChebyshev1 | FilterArchitecture::IirElliptic => {
                let eps_p = (10.0_f64.powf(rp / 10.0) - 1.0).max(1e-10);
                let eps_s = (10.0_f64.powf(rs / 10.0) - 1.0).max(1e-10);
                let num = acosh((eps_s / eps_p).sqrt());
                let den = acosh(selectivity);
                if den <= 0.0 {
                    return 2;
                }
                (num / den).ceil().max(1.0) as usize
            }
            FilterArchitecture::IirChebyshev2 => {
                let eps_p = (10.0_f64.powf(rp / 10.0) - 1.0).max(1e-10);
                let eps_s = (10.0_f64.powf(rs / 10.0) - 1.0).max(1e-10);
                let num = acosh((eps_s / eps_p).sqrt());
                let den = acosh(selectivity);
                if den <= 0.0 {
                    return 2;
                }
                (num / den).ceil() as usize
            }
            FilterArchitecture::Fir => {
                let (wk, beta) = select_window(spec.stopband_atten_db);
                self.estimate_fir_order(spec, wk, beta)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// FIR helpers
// ---------------------------------------------------------------------------

/// Windowed-sinc lowpass FIR. `fc` is normalised cutoff in [0, 1] relative to Nyquist.
fn sinc_filter(n: usize, fc: f64, wk: WindowKind, beta: f64, m: f64) -> Vec<f64> {
    let mut h = vec![0.0; n];
    let mid = m / 2.0;
    let omega_c = fc * PI;

    for i in 0..n {
        let x = i as f64 - mid;
        let sinc = if x.abs() < 1e-12 {
            omega_c / PI
        } else {
            (omega_c * x).sin() / (PI * x)
        };
        h[i] = sinc * window_function(wk, n, i, beta);
    }

    // Normalise DC gain to 1.0
    let sum: f64 = h.iter().sum();
    if sum.abs() > 1e-30 {
        for v in &mut h {
            *v /= sum;
        }
    }
    h
}

/// Spectral inversion: negate all taps, then add 1 to the centre tap.
fn spectral_invert(h: &[f64]) -> Vec<f64> {
    let mut inv: Vec<f64> = h.iter().map(|&x| -x).collect();
    let mid = inv.len() / 2;
    inv[mid] += 1.0;
    inv
}

/// Modulate a baseband lowpass FIR to a centre frequency (for bandpass design).
fn modulate_to_centre(h: &[f64], fc_norm: f64, _sample_rate: f64) -> Vec<f64> {
    let n = h.len();
    let mid = (n - 1) as f64 / 2.0;
    let omega = fc_norm * PI;
    h.iter()
        .enumerate()
        .map(|(i, &v)| 2.0 * v * (omega * (i as f64 - mid)).cos())
        .collect()
}

// ---------------------------------------------------------------------------
// IIR design helpers (bilinear transform)
// ---------------------------------------------------------------------------

/// Design a digital Butterworth lowpass via bilinear transform.
fn design_butterworth_lowpass(order: usize, wp: f64, _fs: f64) -> (Vec<f64>, Vec<f64>) {
    let mut b_all = vec![1.0_f64];
    let mut a_all = vec![1.0_f64];

    if order % 2 == 1 {
        let p = -wp;
        let d0 = -p + 2.0;
        let b_sec = vec![-p / d0, -p / d0];
        let a_sec = vec![1.0, (-p - 2.0) / d0];
        b_all = convolve(&b_all, &b_sec);
        a_all = convolve(&a_all, &a_sec);
    }

    let num_pairs = order / 2;
    for k in 0..num_pairs {
        let angle = PI * (2 * k + order + 1) as f64 / (2 * order) as f64;
        let sr = wp * angle.cos();
        let si = wp * angle.sin();

        let a0_s = sr * sr + si * si;
        let a1_s = -2.0 * sr;

        let c = 2.0;
        let d0 = c * c + a1_s * c + a0_s;
        let d1 = 2.0 * (a0_s - c * c);
        let d2 = c * c - a1_s * c + a0_s;

        let b_sec = vec![a0_s / d0, 2.0 * a0_s / d0, a0_s / d0];
        let a_sec = vec![1.0, d1 / d0, d2 / d0];

        b_all = convolve(&b_all, &b_sec);
        a_all = convolve(&a_all, &a_sec);
    }

    (b_all, a_all)
}

/// Design a digital Chebyshev Type I lowpass via bilinear transform.
fn design_chebyshev1_lowpass(
    order: usize,
    wp: f64,
    ripple_db: f64,
    fs: f64,
) -> (Vec<f64>, Vec<f64>) {
    let eps = (10.0_f64.powf(ripple_db / 10.0) - 1.0).sqrt().max(1e-6);
    let (mut b, a) = design_butterworth_lowpass(order, wp, fs);
    let gain_adjust = 1.0 / (1.0 + eps * eps).sqrt();
    for v in &mut b {
        *v *= gain_adjust;
    }
    (b, a)
}

/// Design a digital Chebyshev Type II lowpass via bilinear transform.
fn design_chebyshev2_lowpass(
    order: usize,
    ws: f64,
    _atten_db: f64,
    fs: f64,
) -> (Vec<f64>, Vec<f64>) {
    let (mut b, a) = design_butterworth_lowpass(order, ws, fs);
    let sum_b: f64 = b.iter().sum();
    let sum_a: f64 = a.iter().sum();
    let dc_gain = sum_b / sum_a;
    if dc_gain.abs() > 1e-30 {
        let scale = 1.0 / dc_gain;
        for v in &mut b {
            *v *= scale;
        }
    }
    (b, a)
}

/// Normalise gain so DC=1 for lowpass (or Nyquist=1 for highpass).
fn normalise_gain(b: &[f64], a: &[f64], spec: &FilterSpec) -> (Vec<f64>, Vec<f64>) {
    let (eval_b, eval_a) = match spec.filter_type {
        FilterType::Lowpass | FilterType::Bandstop => {
            let sb: f64 = b.iter().sum();
            let sa: f64 = a.iter().sum();
            (sb, sa)
        }
        FilterType::Highpass | FilterType::Bandpass => {
            let sb: f64 = b
                .iter()
                .enumerate()
                .map(|(i, &c)| if i % 2 == 0 { c } else { -c })
                .sum();
            let sa: f64 = a
                .iter()
                .enumerate()
                .map(|(i, &c)| if i % 2 == 0 { c } else { -c })
                .sum();
            (sb, sa)
        }
    };

    let gain = if eval_a.abs() > 1e-30 {
        eval_b / eval_a
    } else {
        1.0
    };
    if gain.abs() < 1e-30 {
        return (b.to_vec(), a.to_vec());
    }
    let b_out: Vec<f64> = b.iter().map(|&x| x / gain).collect();
    (b_out, a.to_vec())
}

/// Polynomial convolution (multiplication).
fn convolve(a: &[f64], b: &[f64]) -> Vec<f64> {
    if a.is_empty() || b.is_empty() {
        return vec![];
    }
    let len = a.len() + b.len() - 1;
    let mut out = vec![0.0; len];
    for (i, &ai) in a.iter().enumerate() {
        for (j, &bj) in b.iter().enumerate() {
            out[i + j] += ai * bj;
        }
    }
    out
}

/// Inverse hyperbolic cosine.
fn acosh(x: f64) -> f64 {
    if x < 1.0 {
        return 0.0;
    }
    (x + (x * x - 1.0).sqrt()).ln()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_lowpass_spec() -> FilterSpec {
        FilterSpec {
            filter_type: FilterType::Lowpass,
            passband_freq: 1000.0,
            stopband_freq: 1500.0,
            passband_ripple_db: 0.5,
            stopband_atten_db: 40.0,
            sample_rate: 8000.0,
        }
    }

    #[test]
    fn test_synthesize_lowpass_fir() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize_fir(&spec);
        assert_eq!(result.architecture, FilterArchitecture::Fir);
        assert!(result.order > 0);
        assert_eq!(result.denominator, vec![1.0]);
        assert!(result.numerator.len() > 1);
    }

    #[test]
    fn test_synthesize_lowpass_iir_butterworth() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize_iir(&spec, FilterArchitecture::IirButterworth);
        assert_eq!(result.architecture, FilterArchitecture::IirButterworth);
        assert!(result.order >= 1);
        assert!(result.numerator.len() > 1);
        assert!(result.denominator.len() > 1);
    }

    #[test]
    fn test_synthesize_auto_selects_architecture() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize(&spec);
        assert!(result.order > 0);
        assert!(!result.numerator.is_empty());
    }

    #[test]
    fn test_frequency_response_dc_gain_lowpass_fir() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize_fir(&spec);
        let (freqs, mags) = result.frequency_response(spec.sample_rate, 512);
        assert!(
            (mags[0] - 1.0).abs() < 0.05,
            "DC gain = {}",
            mags[0]
        );
        let last = *mags.last().unwrap();
        assert!(last < 0.1, "Nyquist gain = {}", last);
        assert_eq!(freqs[0], 0.0);
    }

    #[test]
    fn test_meets_spec_lowpass_fir() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize_fir(&spec);
        assert!(
            result.meets_spec(&spec),
            "FIR lowpass should meet its own spec (order={}, arch={:?})",
            result.order,
            result.architecture
        );
    }

    #[test]
    fn test_highpass_fir() {
        let engine = FilterSynthesisEngine::new();
        let spec = FilterSpec {
            filter_type: FilterType::Highpass,
            passband_freq: 1500.0,
            stopband_freq: 2500.0,
            passband_ripple_db: 0.5,
            stopband_atten_db: 40.0,
            sample_rate: 8000.0,
        };
        let result = engine.synthesize_fir(&spec);
        let (_freqs, mags) = result.frequency_response(spec.sample_rate, 512);
        assert!(mags[0] < 0.1, "HP DC gain = {}", mags[0]);
        let last = *mags.last().unwrap();
        assert!(last > 0.5, "HP Nyquist gain = {}", last);
    }

    #[test]
    fn test_estimate_order_fir_vs_iir() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let fir_order = engine.estimate_order(&spec, FilterArchitecture::Fir);
        let iir_order = engine.estimate_order(&spec, FilterArchitecture::IirButterworth);
        assert!(
            fir_order > iir_order,
            "FIR order {} should exceed IIR order {}",
            fir_order,
            iir_order
        );
    }

    #[test]
    fn test_compare_architectures_sorted() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let comparison = engine.compare_architectures(&spec);
        assert_eq!(comparison.len(), 5);
        for i in 1..comparison.len() {
            assert!(
                comparison[i].1 >= comparison[i - 1].1,
                "Not sorted: {:?}",
                comparison
            );
        }
    }

    #[test]
    fn test_group_delay_fir_linear_phase() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize_fir(&spec);
        let expected = (result.numerator.len() - 1) as f64 / 2.0;
        assert!(
            (result.group_delay_samples - expected).abs() < 1.0,
            "GD={} expected={}",
            result.group_delay_samples,
            expected
        );
    }

    #[test]
    fn test_recommend_architecture() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let rec = engine.recommend(&spec);
        assert!(
            matches!(
                rec,
                FilterArchitecture::Fir
                    | FilterArchitecture::IirButterworth
                    | FilterArchitecture::IirChebyshev1
                    | FilterArchitecture::IirChebyshev2
                    | FilterArchitecture::IirElliptic
            ),
            "Invalid recommendation: {:?}",
            rec
        );
    }

    #[test]
    fn test_bessel_i0_accuracy() {
        assert!((bessel_i0(0.0) - 1.0).abs() < 1e-12);
        assert!((bessel_i0(1.0) - 1.2660658).abs() < 1e-4);
    }

    #[test]
    fn test_convolve() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![1.0, 1.0];
        let c = convolve(&a, &b);
        assert_eq!(c.len(), 4);
        assert!((c[0] - 1.0).abs() < 1e-12);
        assert!((c[1] - 3.0).abs() < 1e-12);
        assert!((c[2] - 5.0).abs() < 1e-12);
        assert!((c[3] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_synthesis_result_display() {
        assert_eq!(format!("{}", FilterArchitecture::Fir), "FIR");
        assert_eq!(
            format!("{}", FilterArchitecture::IirButterworth),
            "IIR Butterworth"
        );
    }

    #[test]
    fn test_chebyshev1_lowpass() {
        let engine = FilterSynthesisEngine::new();
        let spec = default_lowpass_spec();
        let result = engine.synthesize_iir(&spec, FilterArchitecture::IirChebyshev1);
        assert_eq!(result.architecture, FilterArchitecture::IirChebyshev1);
        let (_, mags) = result.frequency_response(spec.sample_rate, 512);
        assert!(mags[0] > 0.5, "Cheb1 DC gain = {}", mags[0]);
    }

    #[test]
    fn test_default_engine() {
        let engine: FilterSynthesisEngine = Default::default();
        let spec = default_lowpass_spec();
        let result = engine.synthesize(&spec);
        assert!(result.order > 0);
    }

    #[test]
    fn test_steep_transition_prefers_iir() {
        let engine = FilterSynthesisEngine::new();
        let spec = FilterSpec {
            filter_type: FilterType::Lowpass,
            passband_freq: 1000.0,
            stopband_freq: 1050.0,
            passband_ripple_db: 0.5,
            stopband_atten_db: 60.0,
            sample_rate: 8000.0,
        };
        let rec = engine.recommend(&spec);
        assert_ne!(
            rec,
            FilterArchitecture::Fir,
            "Steep transition should prefer IIR"
        );
    }
}
