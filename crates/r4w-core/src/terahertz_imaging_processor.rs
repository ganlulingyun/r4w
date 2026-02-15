//! # Terahertz Imaging Processor
//!
//! Terahertz (THz) time-domain spectroscopy (TDS) and imaging signal processing
//! for non-destructive testing and material characterization. Operates in the
//! 0.1-10 THz frequency range (wavelengths ~30 um to 3 mm).
//!
//! ## Processing Pipeline
//!
//! ```text
//! THz Pulse -> Time Window -> FFT -> Transfer Function -> Material Parameters
//!                                                      -> n(f), alpha(f), epsilon(f)
//! ```
//!
//! ## Key Equations
//!
//! - **Transfer function**: H(f) = E_sample(f) / E_reference(f)
//! - **Refractive index**: n(f) = 1 + c * phi(f) / (2 * pi * f * d)
//! - **Absorption coeff**: alpha(f) = -2 * ln(|H(f)| * FP_correction) / d
//! - **Fresnel transmission**: t12 = 2*n1 / (n1 + n2)  (normal incidence)
//! - **Debye relaxation**: epsilon(f) = eps_inf + (eps_s - eps_inf) / (1 + j*2*pi*f*tau)
//! - **Drude model**: epsilon(f) = eps_inf - omega_p^2 / (omega^2 + j*omega*gamma)
//! - **Layer thickness**: d = c * delta_t / (2 * n)  (reflection mode)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::terahertz_imaging_processor::{ThzPulse, ThzProcessor, ThzConfig};
//!
//! // Create a simple reference pulse (Gaussian-like)
//! let n = 256;
//! let dt = 0.05; // 50 fs steps
//! let time: Vec<f64> = (0..n).map(|i| (i as f64 - n as f64 / 2.0) * dt).collect();
//! let amp: Vec<f64> = time.iter().map(|&t| (-t * t / 0.5).exp()).collect();
//! let reference = ThzPulse::new(time.clone(), amp, 1.0 / dt);
//!
//! let config = ThzConfig {
//!     time_window_ps: 10.0,
//!     freq_min_thz: 0.1,
//!     freq_max_thz: 5.0,
//!     sample_thickness_m: 1.0e-3,
//! };
//! let processor = ThzProcessor::new(config, reference);
//! let snr = processor.estimate_snr(&processor.reference());
//! assert!(snr > 0.0);
//! ```

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s).
const C_LIGHT: f64 = 2.997_924_58e8;

/// Pi.
const PI: f64 = std::f64::consts::PI;

/// Two pi.
const TWO_PI: f64 = 2.0 * PI;

// ---------------------------------------------------------------------------
// Complex number helpers using (f64, f64) tuples
// ---------------------------------------------------------------------------

/// Complex addition.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex division.
#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex magnitude.
#[inline]
fn c_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex phase (argument).
#[inline]
fn c_arg(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// Complex from polar.
#[inline]
fn c_from_polar(mag: f64, phase: f64) -> (f64, f64) {
    (mag * phase.cos(), mag * phase.sin())
}

/// Complex conjugate.
#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex square: z^2.
#[inline]
fn c_sq(a: (f64, f64)) -> (f64, f64) {
    c_mul(a, a)
}

/// Complex exponential: e^(j*theta).
#[inline]
fn c_exp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// Scale complex by real.
#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

// ---------------------------------------------------------------------------
// FFT (radix-2 Cooley-Tukey)
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT. `data` must have power-of-2 length.
fn fft(data: &mut [(f64, f64)], inverse: bool) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    debug_assert!(n.is_power_of_two(), "FFT length must be power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * TWO_PI / len as f64;
        let wn = c_exp_j(angle);
        let mut start = 0;
        while start < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = data[start + k];
                let t = c_mul(w, data[start + k + half]);
                data[start + k] = c_add(u, t);
                data[start + k + half] = c_sub(u, t);
                w = c_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    // Normalize for inverse
    if inverse {
        let inv_n = 1.0 / n as f64;
        for d in data.iter_mut() {
            d.0 *= inv_n;
            d.1 *= inv_n;
        }
    }
}

/// Next power of two >= n.
fn next_pow2(n: usize) -> usize {
    if n == 0 {
        return 1;
    }
    let mut v = n - 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    v + 1
}

// ---------------------------------------------------------------------------
// Window functions
// ---------------------------------------------------------------------------

/// Window type for time-domain windowing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WindowType {
    /// Rectangular (no windowing).
    Rectangular,
    /// Hann window.
    Hann,
    /// Tukey window with configurable alpha (0 = rectangular, 1 = Hann).
    Tukey(f64),
}

/// Apply a window function to a signal in place.
fn apply_window(data: &mut [f64], window: WindowType) {
    let n = data.len();
    if n == 0 {
        return;
    }
    match window {
        WindowType::Rectangular => {}
        WindowType::Hann => {
            for i in 0..n {
                let w = 0.5 * (1.0 - (TWO_PI * i as f64 / (n - 1).max(1) as f64).cos());
                data[i] *= w;
            }
        }
        WindowType::Tukey(alpha) => {
            let alpha = alpha.clamp(0.0, 1.0);
            let nm1 = (n - 1).max(1) as f64;
            let boundary = alpha * nm1 / 2.0;
            for i in 0..n {
                let x = i as f64;
                let w = if x < boundary {
                    0.5 * (1.0 - (PI * x / boundary).cos())
                } else if x > nm1 - boundary {
                    0.5 * (1.0 - (PI * (nm1 - x) / boundary).cos())
                } else {
                    1.0
                };
                data[i] *= w;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ThzPulse
// ---------------------------------------------------------------------------

/// A THz time-domain pulse: electric field amplitude vs. time in picoseconds.
#[derive(Debug, Clone)]
pub struct ThzPulse {
    /// Time axis in picoseconds.
    pub time_ps: Vec<f64>,
    /// Electric field amplitude (arbitrary units).
    pub amplitude: Vec<f64>,
    /// Sampling rate in THz (= 1 / dt_ps).
    pub sample_rate_thz: f64,
}

impl ThzPulse {
    /// Create a new THz pulse.
    pub fn new(time_ps: Vec<f64>, amplitude: Vec<f64>, sample_rate_thz: f64) -> Self {
        assert_eq!(time_ps.len(), amplitude.len(), "time and amplitude must match length");
        assert!(sample_rate_thz > 0.0, "sample rate must be positive");
        Self {
            time_ps,
            amplitude,
            sample_rate_thz,
        }
    }

    /// Number of samples.
    pub fn len(&self) -> usize {
        self.amplitude.len()
    }

    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.amplitude.is_empty()
    }

    /// Find peak amplitude and its index (by absolute value).
    pub fn peak(&self) -> (usize, f64) {
        let mut best_idx = 0;
        let mut best_abs = -1.0f64;
        for (i, &a) in self.amplitude.iter().enumerate() {
            if a.abs() > best_abs {
                best_idx = i;
                best_abs = a.abs();
            }
        }
        (best_idx, self.amplitude.get(best_idx).copied().unwrap_or(0.0))
    }

    /// Peak time in picoseconds.
    pub fn peak_time_ps(&self) -> f64 {
        let (idx, _) = self.peak();
        self.time_ps[idx]
    }

    /// Pulse width at half-maximum (FWHM) in picoseconds.
    pub fn fwhm_ps(&self) -> f64 {
        let (peak_idx, peak_val) = self.peak();
        let half = peak_val.abs() / 2.0;

        // Search left
        let mut left = peak_idx;
        for i in (0..peak_idx).rev() {
            if self.amplitude[i].abs() < half {
                left = i;
                break;
            }
        }

        // Search right
        let mut right = peak_idx;
        for i in (peak_idx + 1)..self.amplitude.len() {
            if self.amplitude[i].abs() < half {
                right = i;
                break;
            }
        }

        if right > left {
            self.time_ps[right] - self.time_ps[left]
        } else {
            0.0
        }
    }

    /// Align (shift) the pulse so the peak is at time zero. Returns a new pulse.
    pub fn aligned(&self) -> ThzPulse {
        let peak_t = self.peak_time_ps();
        let new_time: Vec<f64> = self.time_ps.iter().map(|&t| t - peak_t).collect();
        ThzPulse {
            time_ps: new_time,
            amplitude: self.amplitude.clone(),
            sample_rate_thz: self.sample_rate_thz,
        }
    }

    /// Apply a time window to suppress reflections.
    pub fn windowed(&self, window: WindowType) -> ThzPulse {
        let mut amp = self.amplitude.clone();
        apply_window(&mut amp, window);
        ThzPulse {
            time_ps: self.time_ps.clone(),
            amplitude: amp,
            sample_rate_thz: self.sample_rate_thz,
        }
    }
}

// ---------------------------------------------------------------------------
// ThzSpectrum
// ---------------------------------------------------------------------------

/// THz frequency-domain spectrum.
#[derive(Debug, Clone)]
pub struct ThzSpectrum {
    /// Frequency axis in THz.
    pub frequency_thz: Vec<f64>,
    /// Magnitude at each frequency.
    pub magnitude: Vec<f64>,
    /// Phase at each frequency (radians).
    pub phase: Vec<f64>,
}

impl ThzSpectrum {
    /// Number of frequency bins.
    pub fn len(&self) -> usize {
        self.frequency_thz.len()
    }

    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.frequency_thz.is_empty()
    }

    /// Dynamic range in dB: 20*log10(peak_mag / noise_floor).
    pub fn dynamic_range_db(&self) -> f64 {
        if self.magnitude.is_empty() {
            return 0.0;
        }
        let peak = self.magnitude.iter().cloned().fold(0.0f64, f64::max);
        // Use last 10% of spectrum as noise floor estimate
        let n = self.magnitude.len();
        let noise_start = n * 9 / 10;
        let noise_count = n - noise_start;
        if noise_count == 0 || peak < 1e-30 {
            return 0.0;
        }
        let noise_floor: f64 =
            self.magnitude[noise_start..].iter().sum::<f64>() / noise_count as f64;
        if noise_floor < 1e-30 {
            return 100.0; // clip
        }
        20.0 * (peak / noise_floor).log10()
    }

    /// Amplitude at a specific frequency (linear interpolation).
    pub fn amplitude_at(&self, freq_thz: f64) -> f64 {
        interpolate_at(&self.frequency_thz, &self.magnitude, freq_thz)
    }

    /// Phase at a specific frequency (linear interpolation).
    pub fn phase_at(&self, freq_thz: f64) -> f64 {
        interpolate_at(&self.frequency_thz, &self.phase, freq_thz)
    }
}

/// Linear interpolation helper.
fn interpolate_at(x: &[f64], y: &[f64], target: f64) -> f64 {
    if x.is_empty() {
        return 0.0;
    }
    if target <= x[0] {
        return y[0];
    }
    if target >= x[x.len() - 1] {
        return y[y.len() - 1];
    }
    for i in 0..x.len() - 1 {
        if target >= x[i] && target <= x[i + 1] {
            let frac = (target - x[i]) / (x[i + 1] - x[i]);
            return y[i] + frac * (y[i + 1] - y[i]);
        }
    }
    y[y.len() - 1]
}

// ---------------------------------------------------------------------------
// Material optical properties
// ---------------------------------------------------------------------------

/// Complex refractive index n_tilde = n + j*kappa.
#[derive(Debug, Clone, Copy)]
pub struct ComplexRefractiveIndex {
    /// Real refractive index.
    pub n: f64,
    /// Extinction coefficient.
    pub kappa: f64,
}

impl ComplexRefractiveIndex {
    /// Create from n and kappa.
    pub fn new(n: f64, kappa: f64) -> Self {
        Self { n, kappa }
    }

    /// Absorption coefficient alpha = 4*pi*kappa*f/c  [1/m].
    /// `freq_thz` is the frequency in THz.
    pub fn absorption_coeff(&self, freq_thz: f64) -> f64 {
        let f_hz = freq_thz * 1e12;
        4.0 * PI * self.kappa * f_hz / C_LIGHT
    }

    /// Power absorption coefficient: 2 * alpha.
    pub fn power_absorption_coeff(&self, freq_thz: f64) -> f64 {
        2.0 * self.absorption_coeff(freq_thz)
    }

    /// Complex dielectric function: epsilon = n_tilde^2.
    /// Returns (real, imag).
    pub fn dielectric(&self) -> (f64, f64) {
        c_sq((self.n, self.kappa))
    }

    /// As complex tuple (n, kappa).
    pub fn as_complex(&self) -> (f64, f64) {
        (self.n, self.kappa)
    }
}

// ---------------------------------------------------------------------------
// Fresnel coefficients (normal incidence)
// ---------------------------------------------------------------------------

/// Fresnel coefficients at normal incidence for THz radiation.
pub struct FresnelCoefficients;

impl FresnelCoefficients {
    /// Transmission coefficient from medium 1 to medium 2 (normal incidence).
    /// t12 = 2*n1 / (n1 + n2)
    pub fn transmission(n1: f64, n2: f64) -> f64 {
        2.0 * n1 / (n1 + n2)
    }

    /// Complex transmission coefficient for complex refractive indices.
    pub fn transmission_complex(n1: (f64, f64), n2: (f64, f64)) -> (f64, f64) {
        c_div(c_scale(n1, 2.0), c_add(n1, n2))
    }

    /// Reflection coefficient from medium 1 to medium 2 (normal incidence).
    /// r12 = (n1 - n2) / (n1 + n2)
    pub fn reflection(n1: f64, n2: f64) -> f64 {
        (n1 - n2) / (n1 + n2)
    }

    /// Complex reflection coefficient.
    pub fn reflection_complex(n1: (f64, f64), n2: (f64, f64)) -> (f64, f64) {
        c_div(c_sub(n1, n2), c_add(n1, n2))
    }

    /// Fabry-Perot correction factor for a slab of thickness d between
    /// media with refractive indices n1 (outside) and n2 (slab).
    /// Accounts for multiple internal reflections at interfaces.
    /// FP = 4*n1*n2 / (n1+n2)^2  (first-order approximation for incoherent sum).
    pub fn fabry_perot_correction(n1: f64, n2: f64) -> f64 {
        4.0 * n1 * n2 / ((n1 + n2) * (n1 + n2))
    }
}

// ---------------------------------------------------------------------------
// Dispersion models
// ---------------------------------------------------------------------------

/// Debye relaxation model for polar materials.
/// epsilon(f) = eps_inf + (eps_s - eps_inf) / (1 + j*2*pi*f*tau_D)
#[derive(Debug, Clone, Copy)]
pub struct DebyeModel {
    /// Static (low frequency) permittivity.
    pub epsilon_s: f64,
    /// Optical (high frequency) permittivity.
    pub epsilon_inf: f64,
    /// Relaxation time in picoseconds.
    pub tau_ps: f64,
}

impl DebyeModel {
    /// Evaluate complex permittivity at frequency f (THz).
    pub fn permittivity(&self, freq_thz: f64) -> (f64, f64) {
        let omega_tau = TWO_PI * freq_thz * self.tau_ps; // dimensionless (THz * ps = 1)
        let delta_eps = self.epsilon_s - self.epsilon_inf;
        // 1 / (1 + j*omega*tau) = (1 - j*omega*tau) / (1 + (omega*tau)^2)
        let denom = 1.0 + omega_tau * omega_tau;
        let re = self.epsilon_inf + delta_eps / denom;
        let im = -delta_eps * omega_tau / denom;
        (re, im)
    }

    /// Extract Debye parameters from permittivity measurements at two frequencies.
    /// Returns (epsilon_s, epsilon_inf, tau_ps).
    pub fn fit_two_freq(
        freq1_thz: f64,
        eps1: (f64, f64),
        freq2_thz: f64,
        eps2: (f64, f64),
    ) -> Option<DebyeModel> {
        // From Im{epsilon} = -delta_eps * omega*tau / (1 + (omega*tau)^2)
        // Ratio of imaginary parts at two frequencies gives tau estimate
        let w1 = TWO_PI * freq1_thz;
        let w2 = TWO_PI * freq2_thz;
        if eps1.1.abs() < 1e-15 || eps2.1.abs() < 1e-15 {
            return None;
        }
        // Im1/Im2 = (w1*tau*(1+(w2*tau)^2)) / (w2*tau*(1+(w1*tau)^2))
        // Simplified: use high-freq epsilon_inf from real part at highest freq
        let epsilon_inf = eps2.0; // approximate
        let delta_eps = eps1.0 - epsilon_inf;
        if delta_eps.abs() < 1e-15 {
            return None;
        }
        // From Re: eps_re = eps_inf + delta_eps / (1 + (w*tau)^2)
        // => (w*tau)^2 = delta_eps / (eps_re - eps_inf) - 1
        let ratio = delta_eps / (eps1.0 - epsilon_inf) - 1.0;
        if ratio < 0.0 {
            // Try alternative estimate from imaginary part
            let tau_est = -eps1.1 / (w1 * delta_eps);
            if tau_est <= 0.0 {
                return None;
            }
            return Some(DebyeModel {
                epsilon_s: eps1.0 + delta_eps,
                epsilon_inf,
                tau_ps: tau_est,
            });
        }
        let tau = ratio.sqrt() / w1;
        let epsilon_s = epsilon_inf + delta_eps;
        Some(DebyeModel {
            epsilon_s,
            epsilon_inf,
            tau_ps: tau,
        })
    }
}

/// Drude model for conductors/free carriers.
/// epsilon(f) = eps_inf - omega_p^2 / (omega^2 + j*omega*gamma)
#[derive(Debug, Clone, Copy)]
pub struct DrudeModel {
    /// Background permittivity.
    pub epsilon_inf: f64,
    /// Plasma frequency in THz.
    pub omega_p_thz: f64,
    /// Scattering rate (damping) in THz.
    pub gamma_thz: f64,
}

impl DrudeModel {
    /// Evaluate complex permittivity at frequency f (THz).
    pub fn permittivity(&self, freq_thz: f64) -> (f64, f64) {
        let omega = TWO_PI * freq_thz;
        let omega_p = TWO_PI * self.omega_p_thz;
        let gamma = TWO_PI * self.gamma_thz;
        // denom = omega^2 + j*omega*gamma = (omega^2, omega*gamma) in complex
        // But we want: -omega_p^2 / (omega^2 + j*omega*gamma)
        // = -omega_p^2 * (omega^2 - j*omega*gamma) / (omega^4 + omega^2*gamma^2)
        let denom = omega * omega * omega * omega + omega * omega * gamma * gamma;
        if denom < 1e-30 {
            return (self.epsilon_inf, 0.0);
        }
        let wp2 = omega_p * omega_p;
        let re = self.epsilon_inf - wp2 * omega * omega / denom;
        let im = wp2 * omega * gamma / denom;
        (re, im)
    }
}

// ---------------------------------------------------------------------------
// ThzConfig and ThzProcessor
// ---------------------------------------------------------------------------

/// Configuration for THz processing.
#[derive(Debug, Clone)]
pub struct ThzConfig {
    /// Time window in picoseconds for windowed processing.
    pub time_window_ps: f64,
    /// Minimum frequency of interest (THz).
    pub freq_min_thz: f64,
    /// Maximum frequency of interest (THz).
    pub freq_max_thz: f64,
    /// Sample thickness in meters.
    pub sample_thickness_m: f64,
}

impl Default for ThzConfig {
    fn default() -> Self {
        Self {
            time_window_ps: 20.0,
            freq_min_thz: 0.1,
            freq_max_thz: 4.0,
            sample_thickness_m: 1.0e-3,
        }
    }
}

/// THz time-domain spectroscopy processor.
#[derive(Debug, Clone)]
pub struct ThzProcessor {
    config: ThzConfig,
    reference: ThzPulse,
}

impl ThzProcessor {
    /// Create a new processor with configuration and reference pulse.
    pub fn new(config: ThzConfig, reference: ThzPulse) -> Self {
        Self { config, reference }
    }

    /// Access the reference pulse.
    pub fn reference(&self) -> &ThzPulse {
        &self.reference
    }

    /// Compute FFT spectrum of a THz pulse.
    pub fn compute_spectrum(&self, pulse: &ThzPulse) -> ThzSpectrum {
        let n = next_pow2(pulse.len());
        let mut buf: Vec<(f64, f64)> = Vec::with_capacity(n);
        for &a in &pulse.amplitude {
            buf.push((a, 0.0));
        }
        // Zero-pad
        buf.resize(n, (0.0, 0.0));

        fft(&mut buf, false);

        let df_thz = pulse.sample_rate_thz / n as f64;
        let n_positive = n / 2 + 1;

        let mut frequency_thz = Vec::with_capacity(n_positive);
        let mut magnitude = Vec::with_capacity(n_positive);
        let mut phase = Vec::with_capacity(n_positive);

        for i in 0..n_positive {
            let f = i as f64 * df_thz;
            frequency_thz.push(f);
            magnitude.push(c_abs(buf[i]));
            phase.push(c_arg(buf[i]));
        }

        ThzSpectrum {
            frequency_thz,
            magnitude,
            phase,
        }
    }

    /// Compute transfer function H(f) = E_sample(f) / E_reference(f).
    /// Returns complex transfer function as (magnitude, phase) spectrum.
    pub fn transfer_function(&self, sample_pulse: &ThzPulse) -> ThzSpectrum {
        let n = next_pow2(sample_pulse.len().max(self.reference.len()));

        // FFT of reference
        let mut ref_buf: Vec<(f64, f64)> = Vec::with_capacity(n);
        for &a in &self.reference.amplitude {
            ref_buf.push((a, 0.0));
        }
        ref_buf.resize(n, (0.0, 0.0));
        fft(&mut ref_buf, false);

        // FFT of sample
        let mut sam_buf: Vec<(f64, f64)> = Vec::with_capacity(n);
        for &a in &sample_pulse.amplitude {
            sam_buf.push((a, 0.0));
        }
        sam_buf.resize(n, (0.0, 0.0));
        fft(&mut sam_buf, false);

        // H(f) = sample / reference
        let df_thz = sample_pulse.sample_rate_thz / n as f64;
        let n_positive = n / 2 + 1;

        let mut frequency_thz = Vec::with_capacity(n_positive);
        let mut magnitude = Vec::with_capacity(n_positive);
        let mut phase = Vec::with_capacity(n_positive);

        for i in 0..n_positive {
            let f = i as f64 * df_thz;
            let h = c_div(sam_buf[i], ref_buf[i]);
            frequency_thz.push(f);
            magnitude.push(c_abs(h));
            phase.push(c_arg(h));
        }

        ThzSpectrum {
            frequency_thz,
            magnitude,
            phase,
        }
    }

    /// Phase unwrapping: removes 2*pi discontinuities.
    pub fn unwrap_phase(phase: &[f64]) -> Vec<f64> {
        if phase.is_empty() {
            return vec![];
        }
        let mut result = vec![phase[0]];
        let mut offset = 0.0;
        for i in 1..phase.len() {
            let mut diff = phase[i] - phase[i - 1];
            if diff > PI {
                diff -= TWO_PI;
                offset -= TWO_PI;
            } else if diff < -PI {
                diff += TWO_PI;
                offset += TWO_PI;
            }
            let _ = diff; // used via offset accumulation
            result.push(phase[i] + offset);
        }
        result
    }

    /// Extract material parameters from transfer function.
    /// Returns (refractive_index, absorption_coeff) at each frequency.
    ///
    /// Uses thick-sample single-pass model:
    ///   n(f) = 1 + c * phi(f) / (2*pi*f*d)
    ///   alpha(f) = -2 * ln(|H(f)| * FP_correction) / d
    pub fn extract_material_params(
        &self,
        sample_pulse: &ThzPulse,
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let tf = self.transfer_function(sample_pulse);
        let unwrapped = Self::unwrap_phase(&tf.phase);

        let d = self.config.sample_thickness_m;
        let fp_correction = FresnelCoefficients::fabry_perot_correction(1.0, 1.5); // air/sample approx

        let mut freqs = Vec::new();
        let mut n_values = Vec::new();
        let mut alpha_values = Vec::new();

        for i in 0..tf.frequency_thz.len() {
            let f = tf.frequency_thz[i];
            if f < self.config.freq_min_thz || f > self.config.freq_max_thz {
                continue;
            }
            // n(f) = 1 + c * phi(f) / (2*pi*f*d)
            let f_hz = f * 1e12;
            let n = 1.0 + C_LIGHT * unwrapped[i] / (TWO_PI * f_hz * d);

            // alpha(f) = -2 * ln(|H| * FP_correction) / d
            let h_mag = tf.magnitude[i];
            let alpha = if h_mag * fp_correction > 1e-15 {
                -2.0 * (h_mag / fp_correction).ln() / d
            } else {
                0.0
            };

            freqs.push(f);
            n_values.push(n);
            alpha_values.push(alpha);
        }

        (freqs, n_values, alpha_values)
    }

    /// Estimate SNR: peak signal / RMS noise in time domain.
    pub fn estimate_snr(&self, pulse: &ThzPulse) -> f64 {
        if pulse.is_empty() {
            return 0.0;
        }
        let (_, peak_val) = pulse.peak();
        let peak_abs = peak_val.abs();

        // Use first and last 10% as noise estimate
        let n = pulse.len();
        let noise_samples = n / 10;
        if noise_samples == 0 {
            return peak_abs;
        }

        let mut noise_power = 0.0;
        let mut count = 0;
        for i in 0..noise_samples {
            noise_power += pulse.amplitude[i] * pulse.amplitude[i];
            count += 1;
        }
        for i in (n - noise_samples)..n {
            noise_power += pulse.amplitude[i] * pulse.amplitude[i];
            count += 1;
        }
        let rms_noise = (noise_power / count as f64).sqrt();
        if rms_noise < 1e-30 {
            return 1e6; // effectively infinite
        }
        peak_abs / rms_noise
    }

    /// Estimate dynamic range from spectrum.
    pub fn dynamic_range_db(&self, pulse: &ThzPulse) -> f64 {
        let spec = self.compute_spectrum(pulse);
        spec.dynamic_range_db()
    }

    /// Layer thickness from time delay between echoes (reflection mode).
    /// d = c * delta_t_ps * 1e-12 / (2 * n_material)
    pub fn layer_thickness_from_echoes(delta_t_ps: f64, n_material: f64) -> f64 {
        C_LIGHT * delta_t_ps * 1e-12 / (2.0 * n_material)
    }

    /// Multi-layer thickness extraction via echo detection.
    /// Returns thicknesses of detected layers.
    pub fn extract_layer_thicknesses(
        &self,
        pulse: &ThzPulse,
        n_material: f64,
        min_echo_fraction: f64,
    ) -> Vec<f64> {
        // Find echoes as local maxima above threshold
        let (_, peak_val) = pulse.peak();
        let threshold = peak_val.abs() * min_echo_fraction;

        let mut echo_times_ps = Vec::new();
        for i in 1..pulse.len().saturating_sub(1) {
            let a = pulse.amplitude[i].abs();
            if a > threshold
                && a >= pulse.amplitude[i - 1].abs()
                && a >= pulse.amplitude[i + 1].abs()
            {
                echo_times_ps.push(pulse.time_ps[i]);
            }
        }

        // Compute thicknesses from consecutive echo pairs
        let mut thicknesses = Vec::new();
        for i in 0..echo_times_ps.len().saturating_sub(1) {
            let dt = echo_times_ps[i + 1] - echo_times_ps[i];
            if dt > 0.0 {
                thicknesses.push(Self::layer_thickness_from_echoes(dt, n_material));
            }
        }
        thicknesses
    }
}

// ---------------------------------------------------------------------------
// THz Imaging
// ---------------------------------------------------------------------------

/// A 2D THz image pixel.
#[derive(Debug, Clone, Copy)]
pub struct ThzPixel {
    /// Peak amplitude of THz pulse.
    pub peak_amplitude: f64,
    /// Peak delay in picoseconds (time-of-flight).
    pub peak_delay_ps: f64,
    /// Pulse width (FWHM) in picoseconds.
    pub pulse_width_ps: f64,
    /// Amplitude at a specific frequency.
    pub freq_amplitude: f64,
    /// Phase at a specific frequency.
    pub freq_phase: f64,
}

/// THz raster scan image.
#[derive(Debug, Clone)]
pub struct ThzImage {
    /// Number of rows (Y scan positions).
    pub rows: usize,
    /// Number of columns (X scan positions).
    pub cols: usize,
    /// Pixel data in row-major order.
    pub pixels: Vec<ThzPixel>,
}

impl ThzImage {
    /// Create a new empty image.
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            pixels: vec![
                ThzPixel {
                    peak_amplitude: 0.0,
                    peak_delay_ps: 0.0,
                    pulse_width_ps: 0.0,
                    freq_amplitude: 0.0,
                    freq_phase: 0.0,
                };
                rows * cols
            ],
        }
    }

    /// Get pixel at (row, col).
    pub fn get(&self, row: usize, col: usize) -> &ThzPixel {
        &self.pixels[row * self.cols + col]
    }

    /// Set pixel at (row, col).
    pub fn set(&mut self, row: usize, col: usize, pixel: ThzPixel) {
        self.pixels[row * self.cols + col] = pixel;
    }

    /// Extract 2D array of a specific property (e.g., peak amplitude).
    pub fn peak_amplitude_map(&self) -> Vec<Vec<f64>> {
        let mut map = vec![vec![0.0; self.cols]; self.rows];
        for r in 0..self.rows {
            for c in 0..self.cols {
                map[r][c] = self.get(r, c).peak_amplitude;
            }
        }
        map
    }

    /// Extract peak delay map (time-of-flight image).
    pub fn peak_delay_map(&self) -> Vec<Vec<f64>> {
        let mut map = vec![vec![0.0; self.cols]; self.rows];
        for r in 0..self.rows {
            for c in 0..self.cols {
                map[r][c] = self.get(r, c).peak_delay_ps;
            }
        }
        map
    }

    /// Extract frequency-domain amplitude map.
    pub fn freq_amplitude_map(&self) -> Vec<Vec<f64>> {
        let mut map = vec![vec![0.0; self.cols]; self.rows];
        for r in 0..self.rows {
            for c in 0..self.cols {
                map[r][c] = self.get(r, c).freq_amplitude;
            }
        }
        map
    }

    /// B-scan (cross-section) at a specific row.
    /// Returns (col_index, peak_delay_ps) pairs.
    pub fn b_scan_row(&self, row: usize) -> Vec<(usize, f64)> {
        (0..self.cols)
            .map(|c| (c, self.get(row, c).peak_delay_ps))
            .collect()
    }

    /// B-scan at a specific column.
    pub fn b_scan_col(&self, col: usize) -> Vec<(usize, f64)> {
        (0..self.rows)
            .map(|r| (r, self.get(r, col).peak_delay_ps))
            .collect()
    }

    /// C-scan (depth slice): pixels with peak_delay in specified range.
    pub fn c_scan(&self, min_delay_ps: f64, max_delay_ps: f64) -> Vec<Vec<f64>> {
        let mut map = vec![vec![0.0; self.cols]; self.rows];
        for r in 0..self.rows {
            for c in 0..self.cols {
                let p = self.get(r, c);
                if p.peak_delay_ps >= min_delay_ps && p.peak_delay_ps <= max_delay_ps {
                    map[r][c] = p.peak_amplitude;
                }
            }
        }
        map
    }

    /// Edge detection via gradient magnitude on peak amplitude map.
    /// Uses Sobel-like 3x3 operator.
    pub fn edge_detect(&self) -> Vec<Vec<f64>> {
        let amp = self.peak_amplitude_map();
        let mut edges = vec![vec![0.0; self.cols]; self.rows];

        for r in 1..self.rows.saturating_sub(1) {
            for c in 1..self.cols.saturating_sub(1) {
                // Sobel gradient
                let gx = -amp[r - 1][c - 1] + amp[r - 1][c + 1] - 2.0 * amp[r][c - 1]
                    + 2.0 * amp[r][c + 1]
                    - amp[r + 1][c - 1]
                    + amp[r + 1][c + 1];
                let gy = -amp[r - 1][c - 1] - 2.0 * amp[r - 1][c] - amp[r - 1][c + 1]
                    + amp[r + 1][c - 1]
                    + 2.0 * amp[r + 1][c]
                    + amp[r + 1][c + 1];
                edges[r][c] = (gx * gx + gy * gy).sqrt();
            }
        }
        edges
    }

    /// Spatial resolution estimate at given frequency (diffraction limit).
    /// resolution ~= lambda/2 = c/(2*f)
    pub fn diffraction_limit_m(freq_thz: f64) -> f64 {
        C_LIGHT / (2.0 * freq_thz * 1e12)
    }
}

/// Build a THz image from raster-scanned pulses.
pub fn build_thz_image(
    processor: &ThzProcessor,
    pulses: &[Vec<ThzPulse>], // [row][col]
    imaging_freq_thz: f64,
) -> ThzImage {
    let rows = pulses.len();
    if rows == 0 {
        return ThzImage::new(0, 0);
    }
    let cols = pulses[0].len();

    let mut image = ThzImage::new(rows, cols);

    for r in 0..rows {
        for c in 0..cols {
            let pulse = &pulses[r][c];
            let (peak_idx, peak_val) = pulse.peak();
            let peak_delay = pulse.time_ps[peak_idx];
            let fwhm = pulse.fwhm_ps();

            // Frequency-domain analysis
            let spec = processor.compute_spectrum(pulse);
            let freq_amp = spec.amplitude_at(imaging_freq_thz);
            let freq_phase = spec.phase_at(imaging_freq_thz);

            image.set(
                r,
                c,
                ThzPixel {
                    peak_amplitude: peak_val,
                    peak_delay_ps: peak_delay,
                    pulse_width_ps: fwhm,
                    freq_amplitude: freq_amp,
                    freq_phase,
                },
            );
        }
    }

    image
}

// ---------------------------------------------------------------------------
// Test helpers
// ---------------------------------------------------------------------------

/// Generate a Gaussian THz pulse for testing.
fn generate_gaussian_pulse(n: usize, dt_ps: f64, width_ps: f64, delay_ps: f64) -> ThzPulse {
    let t0 = -(n as f64 / 2.0) * dt_ps;
    let time: Vec<f64> = (0..n).map(|i| t0 + i as f64 * dt_ps).collect();
    let sigma2 = width_ps * width_ps;
    let amplitude: Vec<f64> = time
        .iter()
        .map(|&t| {
            let td = t - delay_ps;
            (-(td * td) / (2.0 * sigma2)).exp()
        })
        .collect();
    ThzPulse::new(time, amplitude, 1.0 / dt_ps)
}

/// Generate a derivative-Gaussian pulse (more THz-like single cycle).
fn generate_thz_pulse(n: usize, dt_ps: f64, width_ps: f64, delay_ps: f64) -> ThzPulse {
    let t0 = -(n as f64 / 2.0) * dt_ps;
    let time: Vec<f64> = (0..n).map(|i| t0 + i as f64 * dt_ps).collect();
    let sigma2 = width_ps * width_ps;
    let amplitude: Vec<f64> = time
        .iter()
        .map(|&t| {
            let td = t - delay_ps;
            -td / sigma2 * (-(td * td) / (2.0 * sigma2)).exp()
        })
        .collect();
    ThzPulse::new(time, amplitude, 1.0 / dt_ps)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    // ---- Complex helpers ----

    #[test]
    fn test_complex_arithmetic() {
        let a = (3.0, 4.0);
        let b = (1.0, 2.0);
        assert_eq!(c_add(a, b), (4.0, 6.0));
        assert_eq!(c_sub(a, b), (2.0, 2.0));
        // (3+4j)*(1+2j) = 3+6j+4j+8j^2 = -5+10j
        let m = c_mul(a, b);
        assert!((m.0 - (-5.0)).abs() < EPS);
        assert!((m.1 - 10.0).abs() < EPS);
        // (3+4j)/(1+2j) = (3+4j)(1-2j)/(1+4) = (11-2j)/5
        let d = c_div(a, b);
        assert!((d.0 - 11.0 / 5.0).abs() < EPS);
        assert!((d.1 - (-2.0 / 5.0)).abs() < EPS);
    }

    #[test]
    fn test_complex_magnitude_phase() {
        let z = (3.0, 4.0);
        assert!((c_abs(z) - 5.0).abs() < EPS);
        let z2 = (1.0, 1.0);
        assert!((c_arg(z2) - PI / 4.0).abs() < EPS);
    }

    #[test]
    fn test_complex_conjugate() {
        let z = (3.0, 4.0);
        let zc = c_conj(z);
        assert_eq!(zc, (3.0, -4.0));
    }

    #[test]
    fn test_complex_from_polar() {
        let z = c_from_polar(2.0, PI / 3.0);
        assert!((z.0 - 1.0).abs() < 1e-8);
        assert!((z.1 - 3.0f64.sqrt()).abs() < 1e-8);
    }

    // ---- FFT ----

    #[test]
    fn test_fft_impulse() {
        // FFT of delta should be all ones
        let mut data = vec![(0.0, 0.0); 8];
        data[0] = (1.0, 0.0);
        fft(&mut data, false);
        for d in &data {
            assert!((d.0 - 1.0).abs() < EPS);
            assert!(d.1.abs() < EPS);
        }
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let original = vec![
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (4.0, 0.0),
        ];
        let mut data = original.clone();
        fft(&mut data, false);
        fft(&mut data, true);
        for (o, d) in original.iter().zip(data.iter()) {
            assert!((o.0 - d.0).abs() < 1e-10);
            assert!((o.1 - d.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_fft_parseval() {
        // Sum of |x|^2 in time = sum of |X|^2 / N in freq
        let data_orig = vec![
            (1.0, 0.0),
            (0.5, 0.0),
            (-0.3, 0.0),
            (0.7, 0.0),
        ];
        let time_energy: f64 = data_orig.iter().map(|z| z.0 * z.0 + z.1 * z.1).sum();
        let mut data = data_orig;
        fft(&mut data, false);
        let freq_energy: f64 =
            data.iter().map(|z| z.0 * z.0 + z.1 * z.1).sum::<f64>() / data.len() as f64;
        assert!((time_energy - freq_energy).abs() < 1e-10);
    }

    // ---- Window functions ----

    #[test]
    fn test_hann_window() {
        let mut data = vec![1.0; 64];
        apply_window(&mut data, WindowType::Hann);
        assert!(data[0].abs() < 1e-10); // zero at edges
        assert!((data[32] - 1.0).abs() < 0.02); // near 1 at center
    }

    #[test]
    fn test_tukey_window_limits() {
        // Tukey(0) = rectangular
        let mut data = vec![1.0; 64];
        apply_window(&mut data, WindowType::Tukey(0.0));
        for &d in &data {
            assert!((d - 1.0).abs() < EPS);
        }
    }

    // ---- ThzPulse ----

    #[test]
    fn test_pulse_creation_and_peak() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.5, 0.0);
        assert_eq!(pulse.len(), 256);
        let (_, peak_val) = pulse.peak();
        assert!((peak_val - 1.0).abs() < 0.01); // Gaussian peak ~1
    }

    #[test]
    fn test_pulse_peak_time() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.5, 1.0);
        let pt = pulse.peak_time_ps();
        assert!((pt - 1.0).abs() < 0.1); // peak should be near delay
    }

    #[test]
    fn test_pulse_fwhm() {
        let pulse = generate_gaussian_pulse(1024, 0.01, 0.5, 0.0);
        let fwhm = pulse.fwhm_ps();
        // FWHM of Gaussian = 2*sqrt(2*ln2)*sigma ~ 1.177*sigma
        let expected = 2.0 * (2.0 * 2.0f64.ln()).sqrt() * 0.5;
        assert!((fwhm - expected).abs() < 0.1);
    }

    #[test]
    fn test_pulse_alignment() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.5, 2.0);
        let aligned = pulse.aligned();
        let pt = aligned.peak_time_ps();
        assert!(pt.abs() < 0.1);
    }

    #[test]
    fn test_pulse_windowed() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.5, 0.0);
        let windowed = pulse.windowed(WindowType::Hann);
        // Window should reduce or maintain edge samples
        assert!(windowed.amplitude[0].abs() <= pulse.amplitude[0].abs() + EPS);
    }

    // ---- ThzSpectrum ----

    #[test]
    fn test_spectrum_computation() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.5, 0.0);
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, pulse.clone());
        let spec = processor.compute_spectrum(&pulse);
        assert!(!spec.is_empty());
        assert!(spec.magnitude[0] > 0.0); // DC component
    }

    #[test]
    fn test_spectrum_dynamic_range() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.3, 0.0);
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, pulse.clone());
        let spec = processor.compute_spectrum(&pulse);
        let dr = spec.dynamic_range_db();
        assert!(dr > 0.0);
    }

    #[test]
    fn test_spectrum_interpolation() {
        let spec = ThzSpectrum {
            frequency_thz: vec![0.0, 1.0, 2.0, 3.0],
            magnitude: vec![0.0, 1.0, 2.0, 3.0],
            phase: vec![0.0, 0.1, 0.2, 0.3],
        };
        assert!((spec.amplitude_at(1.5) - 1.5).abs() < EPS);
        assert!((spec.phase_at(1.5) - 0.15).abs() < EPS);
    }

    // ---- Material properties ----

    #[test]
    fn test_complex_refractive_index() {
        let nri = ComplexRefractiveIndex::new(1.5, 0.01);
        assert!((nri.n - 1.5).abs() < EPS);
        assert!((nri.kappa - 0.01).abs() < EPS);
    }

    #[test]
    fn test_absorption_coefficient() {
        let nri = ComplexRefractiveIndex::new(1.5, 0.01);
        let alpha = nri.absorption_coeff(1.0); // 1 THz
        // alpha = 4*pi*0.01*1e12 / 3e8 ~ 418.9 /m
        let expected = 4.0 * PI * 0.01 * 1e12 / C_LIGHT;
        assert!((alpha - expected).abs() < 0.1);
    }

    #[test]
    fn test_power_absorption() {
        let nri = ComplexRefractiveIndex::new(1.5, 0.01);
        let alpha = nri.absorption_coeff(1.0);
        let alpha_power = nri.power_absorption_coeff(1.0);
        assert!((alpha_power - 2.0 * alpha).abs() < EPS);
    }

    #[test]
    fn test_dielectric_function() {
        let nri = ComplexRefractiveIndex::new(2.0, 0.0);
        let (re, im) = nri.dielectric();
        assert!((re - 4.0).abs() < EPS); // n^2 = 4
        assert!(im.abs() < EPS); // no loss
    }

    // ---- Fresnel coefficients ----

    #[test]
    fn test_fresnel_transmission() {
        // Air (n=1) to glass (n=1.5)
        let t12 = FresnelCoefficients::transmission(1.0, 1.5);
        assert!((t12 - 2.0 / 2.5).abs() < EPS);
    }

    #[test]
    fn test_fresnel_reflection() {
        let r12 = FresnelCoefficients::reflection(1.0, 1.5);
        assert!((r12 - (-0.5 / 2.5)).abs() < EPS);
    }

    #[test]
    fn test_fresnel_energy_conservation() {
        // r^2 + n2/n1 * t^2 = 1 (energy conservation at normal incidence)
        let n1 = 1.0;
        let n2 = 1.5;
        let r = FresnelCoefficients::reflection(n1, n2);
        let t = FresnelCoefficients::transmission(n1, n2);
        let energy = r * r + (n2 / n1) * t * t;
        assert!((energy - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_fabry_perot_correction() {
        let fp = FresnelCoefficients::fabry_perot_correction(1.0, 1.5);
        // 4*1*1.5 / (2.5)^2 = 6/6.25 = 0.96
        assert!((fp - 0.96).abs() < EPS);
    }

    // ---- Debye model ----

    #[test]
    fn test_debye_dc_limit() {
        let model = DebyeModel {
            epsilon_s: 80.0,
            epsilon_inf: 4.0,
            tau_ps: 8.3,
        };
        let eps = model.permittivity(0.0);
        assert!((eps.0 - 80.0).abs() < EPS); // DC: should be epsilon_s
        assert!(eps.1.abs() < EPS);
    }

    #[test]
    fn test_debye_high_freq_limit() {
        let model = DebyeModel {
            epsilon_s: 80.0,
            epsilon_inf: 4.0,
            tau_ps: 8.3,
        };
        let eps = model.permittivity(100.0); // very high freq
        assert!((eps.0 - 4.0).abs() < 1.0); // should approach epsilon_inf
    }

    #[test]
    fn test_debye_loss_peak() {
        let model = DebyeModel {
            epsilon_s: 80.0,
            epsilon_inf: 4.0,
            tau_ps: 8.3,
        };
        // Loss peak near f = 1/(2*pi*tau)
        let f_peak = 1.0 / (TWO_PI * 8.3);
        let eps_at_peak = model.permittivity(f_peak);
        // Imaginary part should be significant at loss peak
        assert!(eps_at_peak.1.abs() > 1.0);
    }

    // ---- Drude model ----

    #[test]
    fn test_drude_dc_divergence() {
        let model = DrudeModel {
            epsilon_inf: 1.0,
            omega_p_thz: 10.0,
            gamma_thz: 1.0,
        };
        // At very low frequency, Drude gives large negative real part
        let eps = model.permittivity(0.01);
        assert!(eps.0 < 0.0); // negative for metals below plasma freq
    }

    #[test]
    fn test_drude_high_freq() {
        let model = DrudeModel {
            epsilon_inf: 1.0,
            omega_p_thz: 10.0,
            gamma_thz: 1.0,
        };
        let eps = model.permittivity(100.0); // well above plasma freq
        // Should approach epsilon_inf
        assert!((eps.0 - 1.0).abs() < 0.5);
    }

    // ---- Transfer function and material extraction ----

    #[test]
    fn test_transfer_function() {
        let reference = generate_gaussian_pulse(256, 0.05, 0.5, 0.0);
        let sample = generate_gaussian_pulse(256, 0.05, 0.5, 0.2); // slightly delayed
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, reference);
        let tf = processor.transfer_function(&sample);
        assert!(!tf.is_empty());
        // Magnitude should be roughly 1 (same pulse shape, different delay)
        // Check at low frequency where both have significant energy
        assert!(tf.magnitude[1] > 0.5);
    }

    #[test]
    fn test_phase_unwrapping() {
        let phase = vec![0.0, 0.5, 1.0, 1.5, -2.8, -2.3, -1.8]; // wrap at index 4
        let unwrapped = ThzProcessor::unwrap_phase(&phase);
        // Should be monotonically increasing after unwrap
        for i in 1..unwrapped.len() {
            assert!(unwrapped[i] >= unwrapped[i - 1] - 0.1);
        }
    }

    #[test]
    fn test_material_param_extraction() {
        let reference = generate_gaussian_pulse(512, 0.02, 0.3, 0.0);
        // Sample with slight attenuation and delay (simulates material)
        let n = 512;
        let dt = 0.02;
        let t0 = -(n as f64 / 2.0) * dt;
        let time: Vec<f64> = (0..n).map(|i| t0 + i as f64 * dt).collect();
        let amplitude: Vec<f64> = time
            .iter()
            .map(|&t| {
                let td = t - 0.1; // 0.1 ps delay
                0.9 * (-(td * td) / (2.0 * 0.3 * 0.3)).exp()
            })
            .collect();
        let sample = ThzPulse::new(time, amplitude, 1.0 / dt);

        let config = ThzConfig {
            time_window_ps: 10.0,
            freq_min_thz: 0.5,
            freq_max_thz: 4.0,
            sample_thickness_m: 1.0e-3,
        };
        let processor = ThzProcessor::new(config, reference);
        let (freqs, n_vals, alpha_vals) = processor.extract_material_params(&sample);
        assert!(!freqs.is_empty());
        assert!(!n_vals.is_empty());
        assert!(!alpha_vals.is_empty());
    }

    // ---- SNR and dynamic range ----

    #[test]
    fn test_snr_estimation() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.5, 0.0);
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, pulse.clone());
        let snr = processor.estimate_snr(&pulse);
        assert!(snr > 1.0); // Gaussian with zero edges should have high SNR
    }

    #[test]
    fn test_dynamic_range() {
        let pulse = generate_gaussian_pulse(256, 0.05, 0.3, 0.0);
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, pulse.clone());
        let dr = processor.dynamic_range_db(&pulse);
        assert!(dr > 0.0);
    }

    // ---- Layer thickness ----

    #[test]
    fn test_layer_thickness_calculation() {
        // 1 mm polyethylene (n ~ 1.53), reflection mode
        // delta_t = 2*d*n/c = 2*1e-3*1.53/3e8 = 10.2 ps
        let delta_t_ps = 2.0 * 1e-3 * 1.53 / C_LIGHT * 1e12;
        let d = ThzProcessor::layer_thickness_from_echoes(delta_t_ps, 1.53);
        assert!((d - 1e-3).abs() < 1e-6);
    }

    #[test]
    fn test_echo_layer_extraction() {
        // Create a pulse with two echoes
        let n = 512;
        let dt = 0.05;
        let t0 = 0.0;
        let time: Vec<f64> = (0..n).map(|i| t0 + i as f64 * dt).collect();
        let echo_delay = 5.0; // ps between echoes
        let amplitude: Vec<f64> = time
            .iter()
            .map(|&t| {
                let p1 = (-((t - 5.0).powi(2)) / 0.5).exp();
                let p2 = 0.5 * (-((t - 5.0 - echo_delay).powi(2)) / 0.5).exp();
                p1 + p2
            })
            .collect();
        let pulse = ThzPulse::new(time, amplitude, 1.0 / dt);

        let reference = generate_gaussian_pulse(512, dt, 0.5, 0.0);
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, reference);
        let thicknesses = processor.extract_layer_thicknesses(&pulse, 1.5, 0.3);
        assert!(!thicknesses.is_empty());
    }

    // ---- THz Imaging ----

    #[test]
    fn test_image_creation() {
        let img = ThzImage::new(4, 4);
        assert_eq!(img.rows, 4);
        assert_eq!(img.cols, 4);
        assert_eq!(img.pixels.len(), 16);
    }

    #[test]
    fn test_image_set_get() {
        let mut img = ThzImage::new(3, 3);
        img.set(
            1,
            1,
            ThzPixel {
                peak_amplitude: 42.0,
                peak_delay_ps: 1.5,
                pulse_width_ps: 0.3,
                freq_amplitude: 10.0,
                freq_phase: 0.5,
            },
        );
        assert!((img.get(1, 1).peak_amplitude - 42.0).abs() < EPS);
        assert!((img.get(1, 1).peak_delay_ps - 1.5).abs() < EPS);
    }

    #[test]
    fn test_image_maps() {
        let mut img = ThzImage::new(2, 2);
        for r in 0..2 {
            for c in 0..2 {
                img.set(
                    r,
                    c,
                    ThzPixel {
                        peak_amplitude: (r * 2 + c) as f64,
                        peak_delay_ps: (r + c) as f64,
                        pulse_width_ps: 0.5,
                        freq_amplitude: (r * 2 + c) as f64 * 0.1,
                        freq_phase: 0.0,
                    },
                );
            }
        }
        let amp_map = img.peak_amplitude_map();
        assert!((amp_map[0][0] - 0.0).abs() < EPS);
        assert!((amp_map[1][1] - 3.0).abs() < EPS);

        let delay_map = img.peak_delay_map();
        assert!((delay_map[0][1] - 1.0).abs() < EPS);

        let freq_map = img.freq_amplitude_map();
        assert!((freq_map[1][0] - 0.2).abs() < EPS);
    }

    #[test]
    fn test_b_scan() {
        let mut img = ThzImage::new(3, 4);
        for c in 0..4 {
            img.set(
                1,
                c,
                ThzPixel {
                    peak_amplitude: 1.0,
                    peak_delay_ps: c as f64 * 0.5,
                    pulse_width_ps: 0.3,
                    freq_amplitude: 0.0,
                    freq_phase: 0.0,
                },
            );
        }
        let bscan = img.b_scan_row(1);
        assert_eq!(bscan.len(), 4);
        assert!((bscan[2].1 - 1.0).abs() < EPS);
    }

    #[test]
    fn test_c_scan() {
        let mut img = ThzImage::new(3, 3);
        for r in 0..3 {
            for c in 0..3 {
                img.set(
                    r,
                    c,
                    ThzPixel {
                        peak_amplitude: 1.0,
                        peak_delay_ps: (r as f64) * 2.0,
                        pulse_width_ps: 0.3,
                        freq_amplitude: 0.0,
                        freq_phase: 0.0,
                    },
                );
            }
        }
        let cscan = img.c_scan(1.0, 3.0);
        // Only row 1 (delay=2.0) should pass
        assert!((cscan[0][0] - 0.0).abs() < EPS);
        assert!((cscan[1][0] - 1.0).abs() < EPS);
    }

    #[test]
    fn test_edge_detection() {
        let mut img = ThzImage::new(5, 5);
        // Left half = 0, right half = 10
        for r in 0..5 {
            for c in 0..5 {
                let amp = if c >= 3 { 10.0 } else { 0.0 };
                img.set(
                    r,
                    c,
                    ThzPixel {
                        peak_amplitude: amp,
                        peak_delay_ps: 0.0,
                        pulse_width_ps: 0.3,
                        freq_amplitude: 0.0,
                        freq_phase: 0.0,
                    },
                );
            }
        }
        let edges = img.edge_detect();
        // Edge should be strong at column 2-3 boundary
        assert!(edges[2][2] > 1.0);
    }

    #[test]
    fn test_diffraction_limit() {
        // At 1 THz: lambda = 300 um, resolution ~ 150 um
        let res = ThzImage::diffraction_limit_m(1.0);
        assert!((res - 1.5e-4).abs() < 1e-6);
    }

    #[test]
    fn test_build_thz_image() {
        let reference = generate_gaussian_pulse(64, 0.1, 0.5, 0.0);
        let config = ThzConfig::default();
        let processor = ThzProcessor::new(config, reference);

        // 2x2 raster
        let mut pulses = Vec::new();
        for r in 0..2 {
            let mut row = Vec::new();
            for c in 0..2 {
                let delay = (r + c) as f64 * 0.5;
                row.push(generate_gaussian_pulse(64, 0.1, 0.5, delay));
            }
            pulses.push(row);
        }
        let image = build_thz_image(&processor, &pulses, 1.0);
        assert_eq!(image.rows, 2);
        assert_eq!(image.cols, 2);
        // Different delays should produce different peak_delay values
        let d00 = image.get(0, 0).peak_delay_ps;
        let d11 = image.get(1, 1).peak_delay_ps;
        assert!((d11 - d00).abs() > 0.1);
    }

    // ---- Thz-like pulse ----

    #[test]
    fn test_thz_derivative_pulse() {
        let pulse = generate_thz_pulse(256, 0.05, 0.5, 0.0);
        // Derivative Gaussian should cross zero at center
        let (peak_idx, _) = pulse.peak();
        assert!(peak_idx > 0 && peak_idx < 255);
    }

    // ---- Next power of 2 ----

    #[test]
    fn test_next_pow2() {
        assert_eq!(next_pow2(1), 1);
        assert_eq!(next_pow2(3), 4);
        assert_eq!(next_pow2(8), 8);
        assert_eq!(next_pow2(100), 128);
        assert_eq!(next_pow2(256), 256);
    }

    // ---- Complex Fresnel ----

    #[test]
    fn test_complex_fresnel() {
        let n1 = (1.0, 0.0);
        let n2 = (1.5, 0.01);
        let t = FresnelCoefficients::transmission_complex(n1, n2);
        let r = FresnelCoefficients::reflection_complex(n1, n2);
        // t + r should be related: t = 1 + r for normal incidence (field amplitudes from medium 1 side)
        let sum = c_add(r, (1.0, 0.0));
        assert!((sum.0 - t.0).abs() < 0.01);
        assert!((sum.1 - t.1).abs() < 0.01);
    }

    #[test]
    fn test_debye_two_freq_fit() {
        let model = DebyeModel {
            epsilon_s: 80.0,
            epsilon_inf: 4.0,
            tau_ps: 8.3,
        };
        let eps1 = model.permittivity(0.01);
        let eps2 = model.permittivity(1.0);
        let fit = DebyeModel::fit_two_freq(0.01, eps1, 1.0, eps2);
        assert!(fit.is_some());
    }
}
