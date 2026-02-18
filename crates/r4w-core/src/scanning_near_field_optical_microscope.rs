//! Scanning Near-Field Optical Microscope (SNOM/NSOM) Signal Processor
//!
//! Implements signal processing for Scanning Near-field Optical Microscopy,
//! enabling sub-wavelength optical imaging beyond the diffraction limit.
//!
//! # Overview
//!
//! SNOM/NSOM exploits the evanescent near-field to achieve spatial resolution
//! of λ/10 to λ/1000, far beyond the classical Abbe diffraction limit (λ/2).
//!
//! ## Key Techniques
//!
//! - **Aperture SNOM**: Light transmitted through a sub-wavelength aperture at
//!   the tip apex (aluminum-coated tapered fiber)
//! - **Apertureless / s-SNOM**: Metallic tip scatters near-field light;
//!   higher harmonics of tapping frequency suppress far-field background
//! - **Pseudoheterodyne detection**: Reference mirror dithers at frequency M,
//!   sidebands at nΩ ± kM carry complex optical signal (amplitude + phase)
//!
//! ## Signal Model
//!
//! Near-field signal at n-th harmonic: `S_n = A_n · exp(iφ_n)`
//!
//! Extracted via lock-in demodulation at tapping harmonic nΩ:
//! ```text
//! S_n = (2/T) ∫₀ᵀ D(t) · exp(-i·n·Ω·t) dt
//! ```
//!
//! Approach curve (near-field verification):
//! ```text
//! S(z) ∝ exp(-z / d_nf)
//! ```
//! where `d_nf` ≈ tip radius (typically 10–25 nm).
//!
//! ## References
//!
//! - Keilmann & Hillenbrand, "Near-field microscopy by elastic light scattering
//!   from a tip," Phil. Trans. R. Soc. A 362 (2004)
//! - Ocelic, Huber & Hillenbrand, "Pseudoheterodyne detection for background-free
//!   near-field spectroscopy," APL 89, 101124 (2006)
//! - Huth et al., "Nano-FTIR absorption spectroscopy of molecular fingerprints
//!   at 20 nm spatial resolution," Nano Lett. 12, 3973 (2012)

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Core data types
// ---------------------------------------------------------------------------

/// Complex number (real + imaginary) used throughout SNOM processing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }
    pub fn from_polar(amp: f64, phase: f64) -> Self {
        Self {
            re: amp * phase.cos(),
            im: amp * phase.sin(),
        }
    }
    pub fn abs(&self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }
    pub fn arg(&self) -> f64 {
        self.im.atan2(self.re)
    }
    pub fn conj(&self) -> Self {
        Self { re: self.re, im: -self.im }
    }
    pub fn mul(&self, rhs: &Complex) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
    pub fn add(&self, rhs: &Complex) -> Self {
        Self { re: self.re + rhs.re, im: self.im + rhs.im }
    }
    pub fn scale(&self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }
}

/// SNOM operating mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SnomMode {
    /// Light passes through a sub-wavelength aperture at tip apex.
    Aperture,
    /// Tip scatters near-field; background-free via harmonic demodulation.
    ScatteringType,
    /// Collection-mode: tip collects locally excited light.
    Collection,
}

/// Detection geometry for SNOM.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionGeometry {
    Transmission,
    Reflection,
    BackScattering,
}

/// Interferometric detection scheme.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum InterferometricScheme {
    /// Fixed reference phase; only amplitude information.
    Homodyne,
    /// Reference mirror dithers at M; sidebands carry amplitude + phase.
    Pseudoheterodyne { mirror_frequency_hz: f64 },
}

/// Tapping-mode regulation type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TappingRegulation {
    /// Amplitude modulation: tip oscillates; maintain amplitude setpoint.
    AmplitudeModulation { setpoint_fraction: f64 },
    /// Shear-force tuning fork (qPlus): frequency shift tracks distance.
    ShearForce { resonant_frequency_hz: f64, q_factor: f64 },
}

// ---------------------------------------------------------------------------
// Instrument parameters
// ---------------------------------------------------------------------------

/// Full set of SNOM instrument parameters.
#[derive(Debug, Clone)]
pub struct SnomParameters {
    /// Tapping (cantilever oscillation) frequency in Hz.
    pub tapping_frequency_hz: f64,
    /// Free-air tapping amplitude in nm.
    pub tapping_amplitude_nm: f64,
    /// Highest harmonic order to demodulate (typically 3–5).
    pub max_harmonic: u32,
    /// Laser wavelength in nm.
    pub wavelength_nm: f64,
    /// Nominal tip radius in nm (sets near-field decay length).
    pub tip_radius_nm: f64,
    /// Operating mode.
    pub mode: SnomMode,
    /// Detection geometry.
    pub geometry: DetectionGeometry,
    /// Interferometric detection scheme.
    pub scheme: InterferometricScheme,
    /// Tapping-mode regulation.
    pub regulation: TappingRegulation,
    /// Lock-in time constant in seconds.
    pub lockin_time_constant_s: f64,
}

impl Default for SnomParameters {
    fn default() -> Self {
        Self {
            tapping_frequency_hz: 280_000.0, // typical AFM resonance
            tapping_amplitude_nm: 30.0,
            max_harmonic: 4,
            wavelength_nm: 10_600.0, // CO₂ laser (IR)
            tip_radius_nm: 20.0,
            mode: SnomMode::ScatteringType,
            geometry: DetectionGeometry::BackScattering,
            scheme: InterferometricScheme::Pseudoheterodyne {
                mirror_frequency_hz: 300.0,
            },
            regulation: TappingRegulation::AmplitudeModulation {
                setpoint_fraction: 0.8,
            },
            lockin_time_constant_s: 1e-3,
        }
    }
}

// ---------------------------------------------------------------------------
// Lock-in demodulation
// ---------------------------------------------------------------------------

/// Lock-in demodulator result: amplitude and phase at a single harmonic.
#[derive(Debug, Clone, Copy)]
pub struct LockInResult {
    pub harmonic: u32,
    pub amplitude: f64,
    pub phase_rad: f64,
    pub complex: Complex,
}

/// Software lock-in amplifier for harmonic demodulation.
///
/// Multiplies input signal by cos(nΩt) and sin(nΩt), then low-pass filters
/// to extract the in-phase (X) and quadrature (Y) components.
///
/// S_n = (2/T) ∫₀ᵀ D(t) exp(-i·n·Ω·t) dt
pub struct LockInAmplifier {
    harmonic: u32,
    reference_frequency_hz: f64,
    sample_rate_hz: f64,
    /// IIR low-pass filter state for X channel.
    x_state: f64,
    /// IIR low-pass filter state for Y channel.
    y_state: f64,
    /// IIR coefficient α = exp(-1/(τ·fs)).
    alpha: f64,
    /// Running sample phase accumulator.
    phase_acc: f64,
}

impl LockInAmplifier {
    /// Create a new lock-in at harmonic `n` of `reference_frequency_hz`.
    pub fn new(
        harmonic: u32,
        reference_frequency_hz: f64,
        sample_rate_hz: f64,
        time_constant_s: f64,
    ) -> Self {
        let alpha = (-1.0 / (time_constant_s * sample_rate_hz)).exp();
        Self {
            harmonic,
            reference_frequency_hz,
            sample_rate_hz,
            x_state: 0.0,
            y_state: 0.0,
            alpha,
            phase_acc: 0.0,
        }
    }

    /// Process one sample, return current (X, Y) estimate.
    pub fn process(&mut self, sample: f64) -> (f64, f64) {
        let omega = 2.0 * PI * self.reference_frequency_hz
            * self.harmonic as f64
            / self.sample_rate_hz;
        let cos_ref = self.phase_acc.cos();
        let sin_ref = self.phase_acc.sin();

        let x_in = sample * cos_ref;
        let y_in = sample * sin_ref;

        self.x_state = self.alpha * self.x_state + (1.0 - self.alpha) * x_in;
        self.y_state = self.alpha * self.y_state + (1.0 - self.alpha) * y_in;

        self.phase_acc += omega;
        if self.phase_acc > 2.0 * PI {
            self.phase_acc -= 2.0 * PI;
        }

        (2.0 * self.x_state, 2.0 * self.y_state)
    }

    /// Process a block of samples, return final lock-in result.
    pub fn process_block(&mut self, samples: &[f64]) -> LockInResult {
        let mut x = 0.0;
        let mut y = 0.0;
        for &s in samples {
            let (xi, yi) = self.process(s);
            x = xi;
            y = yi;
        }
        let amplitude = (x * x + y * y).sqrt();
        let phase_rad = y.atan2(x);
        LockInResult {
            harmonic: self.harmonic,
            amplitude,
            phase_rad,
            complex: Complex::new(x, y),
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.x_state = 0.0;
        self.y_state = 0.0;
        self.phase_acc = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Near-field signal extraction
// ---------------------------------------------------------------------------

/// Result of multi-harmonic near-field extraction.
#[derive(Debug, Clone)]
pub struct NearFieldSignal {
    /// Harmonic-indexed lock-in results (index 0 → 1st harmonic, etc.).
    pub harmonics: Vec<LockInResult>,
    /// Background contribution estimate (0th harmonic / DC offset).
    pub background_estimate: f64,
}

/// Extract near-field signal at multiple harmonics from a raw detector trace.
///
/// Higher harmonics (n ≥ 2) suppress far-field background because the
/// near-field interaction is strongly nonlinear with tip-sample distance
/// and produces higher harmonic content.
pub fn extract_near_field_harmonics(
    detector_trace: &[f64],
    tapping_frequency_hz: f64,
    sample_rate_hz: f64,
    max_harmonic: u32,
    time_constant_s: f64,
) -> NearFieldSignal {
    let mut harmonics = Vec::with_capacity(max_harmonic as usize);
    for n in 1..=max_harmonic {
        let mut lia = LockInAmplifier::new(
            n,
            tapping_frequency_hz,
            sample_rate_hz,
            time_constant_s,
        );
        let result = lia.process_block(detector_trace);
        harmonics.push(result);
    }

    // DC background: mean of the raw signal
    let background_estimate = if detector_trace.is_empty() {
        0.0
    } else {
        detector_trace.iter().sum::<f64>() / detector_trace.len() as f64
    };

    NearFieldSignal {
        harmonics,
        background_estimate,
    }
}

// ---------------------------------------------------------------------------
// Approach curve analysis
// ---------------------------------------------------------------------------

/// Data point on an approach curve (tip retraction experiment).
#[derive(Debug, Clone, Copy)]
pub struct ApproachPoint {
    /// Tip-sample separation in nm.
    pub distance_nm: f64,
    /// Near-field signal amplitude (normalized, arbitrary units).
    pub signal: f64,
}

/// Result of exponential fit to approach curve.
#[derive(Debug, Clone, Copy)]
pub struct ApproachFitResult {
    /// Amplitude at contact (z = 0).
    pub s0: f64,
    /// Near-field decay length in nm (≈ tip radius for s-SNOM).
    pub decay_length_nm: f64,
    /// Residual (sum of squared errors, normalized).
    pub rmse: f64,
    /// True if the fit confirms near-field (decay_length < 2× tip radius).
    pub is_near_field: bool,
}

/// Fit approach curve to exponential model: S(z) = S₀ · exp(-z / d).
///
/// Uses iterative linearization: ln(S) = ln(S₀) - z/d → linear regression.
pub fn fit_approach_curve(
    points: &[ApproachPoint],
    tip_radius_nm: f64,
) -> Option<ApproachFitResult> {
    if points.len() < 3 {
        return None;
    }

    // Only use positive-signal points for log-linear regression
    let valid: Vec<_> = points
        .iter()
        .filter(|p| p.signal > 0.0)
        .collect();
    if valid.len() < 3 {
        return None;
    }

    // Linear regression on (z, ln S)
    let n = valid.len() as f64;
    let sum_z: f64 = valid.iter().map(|p| p.distance_nm).sum();
    let sum_lns: f64 = valid.iter().map(|p| p.signal.ln()).sum();
    let sum_z2: f64 = valid.iter().map(|p| p.distance_nm * p.distance_nm).sum();
    let sum_zlns: f64 = valid.iter().map(|p| p.distance_nm * p.signal.ln()).sum();

    let denom = n * sum_z2 - sum_z * sum_z;
    if denom.abs() < 1e-15 {
        return None;
    }

    let slope = (n * sum_zlns - sum_z * sum_lns) / denom;
    let intercept = (sum_lns - slope * sum_z) / n;

    let s0 = intercept.exp();
    // slope = -1/d  → d = -1/slope
    if slope >= 0.0 {
        return None; // signal not decaying
    }
    let decay_length_nm = -1.0 / slope;

    // RMSE
    let sse: f64 = valid
        .iter()
        .map(|p| {
            let pred = s0 * (-p.distance_nm / decay_length_nm).exp();
            (p.signal - pred).powi(2)
        })
        .sum();
    let rmse = (sse / valid.len() as f64).sqrt();

    let is_near_field = decay_length_nm < 2.0 * tip_radius_nm;

    Some(ApproachFitResult {
        s0,
        decay_length_nm,
        rmse,
        is_near_field,
    })
}

/// Generate a synthetic approach curve with optional noise.
pub fn generate_approach_curve(
    s0: f64,
    decay_length_nm: f64,
    z_min_nm: f64,
    z_max_nm: f64,
    n_points: usize,
    noise_fraction: f64,
) -> Vec<ApproachPoint> {
    (0..n_points)
        .map(|i| {
            let t = i as f64 / (n_points - 1).max(1) as f64;
            let z = z_min_nm + t * (z_max_nm - z_min_nm);
            let signal = s0 * (-z / decay_length_nm).exp();
            // Deterministic pseudo-noise using bit pattern
            let noise = noise_fraction
                * signal
                * (0.5 - ((i as f64 * 1.234).sin().abs()));
            ApproachPoint {
                distance_nm: z,
                signal: (signal + noise).max(1e-10),
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tapping-mode AFM feedback simulation
// ---------------------------------------------------------------------------

/// State of the AFM feedback loop.
#[derive(Debug, Clone)]
pub struct AfmFeedback {
    /// Cantilever resonant frequency in Hz.
    pub resonant_frequency_hz: f64,
    /// Free-air amplitude in nm.
    pub free_amplitude_nm: f64,
    /// Amplitude setpoint as a fraction of free amplitude.
    pub setpoint_fraction: f64,
    /// PID integral state.
    integral: f64,
    /// PID proportional gain.
    kp: f64,
    /// PID integral gain.
    ki: f64,
    /// Current estimated tip-sample distance in nm.
    pub current_distance_nm: f64,
}

impl AfmFeedback {
    pub fn new(
        resonant_frequency_hz: f64,
        free_amplitude_nm: f64,
        setpoint_fraction: f64,
    ) -> Self {
        Self {
            resonant_frequency_hz,
            free_amplitude_nm,
            setpoint_fraction,
            integral: 0.0,
            kp: 0.5,
            ki: 0.1,
            current_distance_nm: 50.0,
        }
    }

    /// Simulate one feedback step given measured cantilever amplitude.
    ///
    /// Returns the piezo-z correction in nm.
    pub fn step(&mut self, measured_amplitude_nm: f64, dt: f64) -> f64 {
        let setpoint = self.free_amplitude_nm * self.setpoint_fraction;
        let error = setpoint - measured_amplitude_nm;
        self.integral += error * dt;
        let correction = self.kp * error + self.ki * self.integral;
        self.current_distance_nm = (self.current_distance_nm - correction).max(0.0);
        correction
    }

    /// Compute expected tapping amplitude at a given distance.
    ///
    /// Uses a simplified Lennard-Jones-like damping model.
    pub fn amplitude_at_distance(&self, distance_nm: f64) -> f64 {
        // Damping scale ≈ tip radius
        let damping_scale = 15.0_f64;
        let damping = (1.0 + (damping_scale / (distance_nm + 1.0)).powi(2)).sqrt();
        self.free_amplitude_nm / damping
    }
}

// ---------------------------------------------------------------------------
// Interferometric detection
// ---------------------------------------------------------------------------

/// Pseudoheterodyne demodulation result: amplitude AND phase.
///
/// The reference mirror dithers at frequency M, producing sidebands at
/// nΩ ± kM in the detector spectrum. The complex near-field signal is
/// recovered from the ratio of J1 and J0 Bessel-function sidebands.
#[derive(Debug, Clone)]
pub struct PseudohetResult {
    pub harmonic: u32,
    pub amplitude: f64,
    pub phase_rad: f64,
    /// Sideband at nΩ + M
    pub sideband_plus: Complex,
    /// Sideband at nΩ - M
    pub sideband_minus: Complex,
}

/// Pseudoheterodyne demodulator.
///
/// Extracts complex near-field signal from sidebands produced by
/// mirror modulation at frequency `mirror_freq_hz`.
pub struct PseudohetDemodulator {
    pub harmonic: u32,
    pub tapping_freq_hz: f64,
    pub mirror_freq_hz: f64,
    pub sample_rate_hz: f64,
    pub time_constant_s: f64,
}

impl PseudohetDemodulator {
    pub fn new(
        harmonic: u32,
        tapping_freq_hz: f64,
        mirror_freq_hz: f64,
        sample_rate_hz: f64,
        time_constant_s: f64,
    ) -> Self {
        Self {
            harmonic,
            tapping_freq_hz,
            mirror_freq_hz,
            sample_rate_hz,
            time_constant_s,
        }
    }

    /// Demodulate detector trace at nΩ ± M sidebands.
    pub fn demodulate(&self, detector: &[f64]) -> PseudohetResult {
        let freq_plus = self.tapping_freq_hz * self.harmonic as f64 + self.mirror_freq_hz;
        let freq_minus = self.tapping_freq_hz * self.harmonic as f64 - self.mirror_freq_hz;

        let sb_plus = self.lock_in_at(detector, freq_plus);
        let sb_minus = self.lock_in_at(detector, freq_minus);

        // Complex near-field from sidebands (simplified Ocelic/Hillenbrand recovery):
        // E_nf ∝ S_plus + conj(S_minus)  (proportional to J1(φ0))
        let nf = sb_plus.add(&sb_minus.conj());
        let amplitude = nf.abs();
        let phase_rad = nf.arg();

        PseudohetResult {
            harmonic: self.harmonic,
            amplitude,
            phase_rad,
            sideband_plus: sb_plus,
            sideband_minus: sb_minus,
        }
    }

    fn lock_in_at(&self, samples: &[f64], freq_hz: f64) -> Complex {
        let alpha = (-1.0 / (self.time_constant_s * self.sample_rate_hz)).exp();
        let omega = 2.0 * PI * freq_hz / self.sample_rate_hz;
        let mut x_state = 0.0_f64;
        let mut y_state = 0.0_f64;
        let mut phase = 0.0_f64;
        for &s in samples {
            x_state = alpha * x_state + (1.0 - alpha) * s * phase.cos();
            y_state = alpha * y_state + (1.0 - alpha) * s * phase.sin();
            phase += omega;
            if phase > 2.0 * PI {
                phase -= 2.0 * PI;
            }
        }
        Complex::new(2.0 * x_state, 2.0 * y_state)
    }
}

// ---------------------------------------------------------------------------
// Background suppression
// ---------------------------------------------------------------------------

/// Background suppression via harmonic ratio.
///
/// At higher harmonics (n ≥ 2), far-field background is exponentially
/// suppressed relative to the near-field signal. The suppression factor
/// is approximately: η_n ≈ (a/R)^n where `a` is tip amplitude, `R` is
/// the effective interaction radius.
pub fn background_suppression_factor(
    harmonic: u32,
    tapping_amplitude_nm: f64,
    tip_radius_nm: f64,
) -> f64 {
    let ratio = tapping_amplitude_nm / (tapping_amplitude_nm + tip_radius_nm);
    ratio.powi(harmonic as i32)
}

/// Suppress background by subtracting a scaled 1st-harmonic reference.
///
/// Uses the assumption that higher harmonics are near-field pure,
/// while 1st harmonic contains mixed near-field + far-field.
pub fn subtract_background(
    signal: &[f64],
    background_reference: &[f64],
    scale: f64,
) -> Vec<f64> {
    signal
        .iter()
        .zip(background_reference.iter())
        .map(|(&s, &b)| s - scale * b)
        .collect()
}

// ---------------------------------------------------------------------------
// Image processing
// ---------------------------------------------------------------------------

/// SNOM image represented as a 2D grid.
#[derive(Debug, Clone)]
pub struct SnomImage {
    /// Width in pixels (fast scan axis).
    pub width: usize,
    /// Height in pixels (slow scan axis).
    pub height: usize,
    /// Pixel data in row-major order.
    pub data: Vec<f64>,
    /// Pixel size in nm.
    pub pixel_size_nm: f64,
}

impl SnomImage {
    pub fn new(width: usize, height: usize, pixel_size_nm: f64) -> Self {
        Self {
            width,
            height,
            pixel_size_nm,
            data: vec![0.0; width * height],
        }
    }

    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.width + col]
    }

    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        self.data[row * self.width + col] = val;
    }

    /// Line-by-line polynomial leveling (removes tilt and bow per line).
    ///
    /// Fits a polynomial of given `order` to each scan line and subtracts it.
    pub fn line_level(&mut self, order: usize) {
        let w = self.width;
        for row in 0..self.height {
            let line: Vec<f64> = (0..w).map(|c| self.get(row, c)).collect();
            let leveled = poly_subtract(&line, order);
            for (c, &v) in leveled.iter().enumerate() {
                self.set(row, c, v);
            }
        }
    }

    /// Global plane subtraction (removes overall tilt across the image).
    pub fn subtract_plane(&mut self) {
        let (w, h) = (self.width, self.height);
        // Fit z = a + b·x + c·y by least squares
        let n = (w * h) as f64;
        let mut sx = 0.0_f64;
        let mut sy = 0.0_f64;
        let mut sz = 0.0_f64;
        let mut sx2 = 0.0_f64;
        let mut sy2 = 0.0_f64;
        let mut sxy = 0.0_f64;
        let mut sxz = 0.0_f64;
        let mut syz = 0.0_f64;
        for row in 0..h {
            for col in 0..w {
                let x = col as f64;
                let y = row as f64;
                let z = self.get(row, col);
                sx += x; sy += y; sz += z;
                sx2 += x * x; sy2 += y * y;
                sxy += x * y; sxz += x * z; syz += y * z;
            }
        }
        // Normal equations (simplified 3-param plane fit)
        let det = n * (sx2 * sy2 - sxy * sxy)
            - sx * (sx * sy2 - sxy * sy)
            + sy * (sx * sxy - sx2 * sy);
        if det.abs() < 1e-30 {
            return;
        }
        let b = (n * (sxz * sy2 - syz * sxy)
            - sx * (sz * sy2 - syz * sy)
            + sy * (sz * sxy - sxz * sy))
            / det;
        let c = (n * (sx2 * syz - sxy * sxz)
            - sx * (sx * syz - sxy * sz)
            + sy * (sx * sxz - sx2 * sz))
            / det;
        let a = (sz - b * sx - c * sy) / n;
        for row in 0..h {
            for col in 0..w {
                let x = col as f64;
                let y = row as f64;
                let plane = a + b * x + c * y;
                let z = self.get(row, col);
                self.set(row, col, z - plane);
            }
        }
    }

    /// Correct scanner bow (parabolic background from piezo nonlinearity).
    pub fn correct_bow(&mut self) {
        // Each line: subtract 2nd-order polynomial (captures parabolic bow)
        let w = self.width;
        for row in 0..self.height {
            let line: Vec<f64> = (0..w).map(|c| self.get(row, c)).collect();
            let corrected = poly_subtract(&line, 2);
            for (c, &v) in corrected.iter().enumerate() {
                self.set(row, c, v);
            }
        }
    }

    /// Flatten image to zero mean.
    pub fn normalize_mean(&mut self) {
        let mean = self.data.iter().sum::<f64>() / self.data.len() as f64;
        for v in self.data.iter_mut() {
            *v -= mean;
        }
    }

    /// Return min and max values.
    pub fn range(&self) -> (f64, f64) {
        let min = self.data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self.data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }

    /// Compute RMS roughness (Rq).
    pub fn rms_roughness(&self) -> f64 {
        let n = self.data.len() as f64;
        let mean = self.data.iter().sum::<f64>() / n;
        let variance = self.data.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / n;
        variance.sqrt()
    }
}

/// Subtract a polynomial of given order from a 1D line.
fn poly_subtract(line: &[f64], order: usize) -> Vec<f64> {
    let n = line.len();
    if n == 0 {
        return vec![];
    }
    // Build Vandermonde matrix and solve by normal equations (simple approach)
    let order = order.min(n.saturating_sub(1));
    let deg = order + 1;
    // Build A^T A and A^T y
    let mut ata = vec![0.0_f64; deg * deg];
    let mut aty = vec![0.0_f64; deg];
    for i in 0..n {
        let x = i as f64 / (n - 1).max(1) as f64; // normalized [0,1]
        let mut xpow = vec![1.0_f64; deg];
        for k in 1..deg {
            xpow[k] = xpow[k - 1] * x;
        }
        for r in 0..deg {
            aty[r] += xpow[r] * line[i];
            for c in 0..deg {
                ata[r * deg + c] += xpow[r] * xpow[c];
            }
        }
    }
    // Solve with Gaussian elimination
    let coeffs = gauss_solve(&ata, &aty, deg).unwrap_or_else(|| vec![0.0; deg]);
    // Subtract polynomial
    (0..n)
        .map(|i| {
            let x = i as f64 / (n - 1).max(1) as f64;
            let poly: f64 = coeffs
                .iter()
                .enumerate()
                .map(|(k, &c)| c * x.powi(k as i32))
                .sum();
            line[i] - poly
        })
        .collect()
}

/// Solve Ax = b by Gaussian elimination with partial pivoting.
fn gauss_solve(a: &[f64], b: &[f64], n: usize) -> Option<Vec<f64>> {
    let mut mat: Vec<f64> = (0..n)
        .flat_map(|r| {
            let mut row: Vec<f64> = a[r * n..(r + 1) * n].to_vec();
            row.push(b[r]);
            row
        })
        .collect();
    let nc = n + 1;
    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = mat[col * nc + col].abs();
        for row in (col + 1)..n {
            let v = mat[row * nc + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return None;
        }
        for c in 0..nc {
            let tmp = mat[col * nc + c];
            mat[col * nc + c] = mat[max_row * nc + c];
            mat[max_row * nc + c] = tmp;
        }
        let pivot = mat[col * nc + col];
        for row in (col + 1)..n {
            let factor = mat[row * nc + col] / pivot;
            for c in col..nc {
                let v = mat[col * nc + c];
                mat[row * nc + c] -= factor * v;
            }
        }
    }
    // Back substitution
    let mut x = vec![0.0_f64; n];
    for row in (0..n).rev() {
        x[row] = mat[row * nc + n];
        for c in (row + 1)..n {
            x[row] -= mat[row * nc + c] * x[c];
        }
        x[row] /= mat[row * nc + row];
    }
    Some(x)
}

// ---------------------------------------------------------------------------
// Spatial resolution and PSF
// ---------------------------------------------------------------------------

/// Estimate SNOM spatial resolution for a given tip radius and wavelength.
///
/// Returns estimated lateral resolution in nm.
/// For s-SNOM: resolution ≈ tip radius.
/// For aperture SNOM: resolution ≈ aperture diameter (typically 50–100 nm).
pub fn estimate_resolution_nm(
    mode: SnomMode,
    tip_radius_nm: f64,
    wavelength_nm: f64,
) -> f64 {
    match mode {
        SnomMode::ScatteringType => tip_radius_nm,
        SnomMode::Aperture => {
            // Aperture resolution limited by aperture size, typically 0.3–1× tip radius
            // and minimum ~50 nm (skin depth limitation)
            (tip_radius_nm * 2.0).max(50.0)
        }
        SnomMode::Collection => {
            // Collection mode: limited by tip-sample coupling volume
            (tip_radius_nm * 1.5).max(wavelength_nm / 100.0)
        }
    }
}

/// Abbe diffraction limit for reference.
pub fn abbe_limit_nm(wavelength_nm: f64, numerical_aperture: f64) -> f64 {
    wavelength_nm / (2.0 * numerical_aperture)
}

/// Point spread function (PSF) for near-field tip interaction.
///
/// Models the lateral PSF as a Lorentzian: PSF(r) = 1 / (1 + (r/r_tip)^2).
pub fn near_field_psf(radial_distance_nm: f64, tip_radius_nm: f64) -> f64 {
    1.0 / (1.0 + (radial_distance_nm / tip_radius_nm).powi(2))
}

// ---------------------------------------------------------------------------
// Dielectric function extraction
// ---------------------------------------------------------------------------

/// Complex dielectric function ε = ε₁ + i·ε₂.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DielectricFunction {
    /// Real part (dispersion).
    pub epsilon1: f64,
    /// Imaginary part (absorption).
    pub epsilon2: f64,
}

impl DielectricFunction {
    pub fn new(epsilon1: f64, epsilon2: f64) -> Self {
        Self { epsilon1, epsilon2 }
    }

    /// Complex refractive index n_tilde = n + ik.
    pub fn refractive_index(&self) -> (f64, f64) {
        let modulus = (self.epsilon1.powi(2) + self.epsilon2.powi(2)).sqrt();
        let n = ((modulus + self.epsilon1) / 2.0).sqrt();
        let k = ((modulus - self.epsilon1) / 2.0).sqrt();
        (n, k)
    }

    /// Near-field contrast (simplified quasistatic approximation).
    ///
    /// The scattering SNOM signal amplitude is related to the effective
    /// polarizability of the tip-sample system via the dielectric function.
    /// Using the point-dipole model: α_eff ∝ (ε-1)/(ε+2) for a sphere.
    pub fn near_field_contrast_amplitude(&self) -> f64 {
        let eps = (self.epsilon1, self.epsilon2);
        // α = (ε-1)/(ε+2) in quasistatic limit
        let num_re = eps.0 - 1.0;
        let num_im = eps.1;
        let den_re = eps.0 + 2.0;
        let den_im = eps.1;
        let denom_sq = den_re * den_re + den_im * den_im;
        let alpha_re = (num_re * den_re + num_im * den_im) / denom_sq;
        let alpha_im = (num_im * den_re - num_re * den_im) / denom_sq;
        (alpha_re * alpha_re + alpha_im * alpha_im).sqrt()
    }

    /// Phase of near-field contrast (relative to a reference).
    pub fn near_field_contrast_phase(&self) -> f64 {
        let num_re = self.epsilon1 - 1.0;
        let num_im = self.epsilon2;
        let den_re = self.epsilon1 + 2.0;
        let den_im = self.epsilon2;
        let denom_sq = den_re * den_re + den_im * den_im;
        let alpha_re = (num_re * den_re + num_im * den_im) / denom_sq;
        let alpha_im = (num_im * den_re - num_re * den_im) / denom_sq;
        alpha_im.atan2(alpha_re)
    }
}

/// Drude model for free-electron metals.
///
/// ε(ω) = 1 - ω_p² / (ω² + iγω)
pub fn drude_model(
    frequency_hz: f64,
    plasma_frequency_hz: f64,
    scattering_rate_hz: f64,
) -> DielectricFunction {
    let omega = 2.0 * PI * frequency_hz;
    let omega_p = 2.0 * PI * plasma_frequency_hz;
    let gamma = 2.0 * PI * scattering_rate_hz;
    let denom = omega * omega + gamma * gamma;
    let eps1 = 1.0 - omega_p * omega_p / denom;
    let eps2 = omega_p * omega_p * gamma / (omega * denom);
    DielectricFunction::new(eps1, eps2)
}

// ---------------------------------------------------------------------------
// Material database
// ---------------------------------------------------------------------------

/// Known material with dielectric constants at typical SNOM wavelengths.
#[derive(Debug, Clone)]
pub struct SnomMaterial {
    pub name: &'static str,
    /// Wavelength in nm at which constants are tabulated.
    pub wavelength_nm: f64,
    pub epsilon: DielectricFunction,
}

/// Return dielectric constants for common SNOM materials.
/// Data for representative IR/visible wavelengths.
pub fn material_database() -> Vec<SnomMaterial> {
    vec![
        // Gold at 10 600 nm (CO₂ laser, mid-IR) — Drude-like
        SnomMaterial {
            name: "Gold",
            wavelength_nm: 10_600.0,
            epsilon: DielectricFunction::new(-3800.0, 2100.0),
        },
        // Gold at 633 nm (HeNe, visible)
        SnomMaterial {
            name: "Gold-VIS",
            wavelength_nm: 633.0,
            epsilon: DielectricFunction::new(-12.0, 1.3),
        },
        // Silicon at 10 600 nm (IR)
        SnomMaterial {
            name: "Silicon",
            wavelength_nm: 10_600.0,
            epsilon: DielectricFunction::new(11.7, 0.0035),
        },
        // SiO2 (glass) at 10 000 nm — phonon resonance
        SnomMaterial {
            name: "SiO2",
            wavelength_nm: 10_000.0,
            epsilon: DielectricFunction::new(-3.5, 3.2),
        },
        // PMMA at 5800 nm (carbonyl C=O stretch)
        SnomMaterial {
            name: "PMMA",
            wavelength_nm: 5800.0,
            epsilon: DielectricFunction::new(2.1, 0.15),
        },
        // Polystyrene (PS) at 3300 nm (C-H stretch)
        SnomMaterial {
            name: "PS",
            wavelength_nm: 3300.0,
            epsilon: DielectricFunction::new(2.4, 0.08),
        },
    ]
}

/// Look up a material by name.
pub fn find_material(name: &str) -> Option<SnomMaterial> {
    material_database()
        .into_iter()
        .find(|m| m.name.eq_ignore_ascii_case(name))
}

// ---------------------------------------------------------------------------
// Nano-FTIR / nano-spectroscopy helpers
// ---------------------------------------------------------------------------

/// Nano-spectroscopy spectrum: near-field amplitude and phase vs wavenumber.
#[derive(Debug, Clone)]
pub struct NanoSpectrum {
    /// Wavenumbers in cm⁻¹.
    pub wavenumbers: Vec<f64>,
    /// Near-field amplitude at each wavenumber (normalized).
    pub amplitude: Vec<f64>,
    /// Near-field phase in radians.
    pub phase: Vec<f64>,
}

impl NanoSpectrum {
    pub fn new(wavenumbers: Vec<f64>, amplitude: Vec<f64>, phase: Vec<f64>) -> Self {
        Self {
            wavenumbers,
            amplitude,
            phase,
        }
    }

    /// Number of spectral points.
    pub fn len(&self) -> usize {
        self.wavenumbers.len()
    }

    pub fn is_empty(&self) -> bool {
        self.wavenumbers.is_empty()
    }

    /// Find peak wavenumber in amplitude spectrum.
    pub fn peak_wavenumber(&self) -> Option<f64> {
        self.amplitude
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| self.wavenumbers[i])
    }

    /// Baseline correction: subtract linear interpolation between endpoints.
    pub fn baseline_correct(&self) -> NanoSpectrum {
        let n = self.len();
        if n < 2 {
            return self.clone();
        }
        let a0 = self.amplitude[0];
        let a1 = self.amplitude[n - 1];
        let corrected: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                self.amplitude[i] - (a0 + t * (a1 - a0))
            })
            .collect();
        NanoSpectrum::new(self.wavenumbers.clone(), corrected, self.phase.clone())
    }
}

// ---------------------------------------------------------------------------
// Signal-to-noise estimation
// ---------------------------------------------------------------------------

/// Estimate SNR of near-field signal from harmonic amplitude and noise floor.
///
/// Signal: amplitude at n-th harmonic.
/// Noise: RMS of detector trace residual after removing harmonics.
pub fn estimate_snr_db(signal_amplitude: f64, noise_rms: f64) -> f64 {
    if noise_rms < 1e-30 {
        return f64::INFINITY;
    }
    20.0 * (signal_amplitude / noise_rms).log10()
}

/// Estimate noise floor from the tail of harmonic amplitudes.
///
/// Assumes harmonics above `signal_max_harmonic` are noise-dominated.
pub fn estimate_noise_floor(harmonics: &[LockInResult], signal_max_harmonic: u32) -> f64 {
    let noise_harmonics: Vec<_> = harmonics
        .iter()
        .filter(|h| h.harmonic > signal_max_harmonic)
        .collect();
    if noise_harmonics.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = noise_harmonics.iter().map(|h| h.amplitude * h.amplitude).sum();
    (sum_sq / noise_harmonics.len() as f64).sqrt()
}

// ---------------------------------------------------------------------------
// Scan artifact correction
// ---------------------------------------------------------------------------

/// Detect and correct periodic scan artifacts (e.g., from electrical interference).
///
/// Identifies lines with anomalously high RMS deviation and replaces them
/// with interpolation from neighbouring lines.
pub fn correct_scan_artifacts(image: &SnomImage, threshold_sigma: f64) -> SnomImage {
    let mut result = image.clone();
    let h = image.height;
    let w = image.width;

    // Per-line RMS
    let line_rms: Vec<f64> = (0..h)
        .map(|row| {
            let vals: Vec<f64> = (0..w).map(|c| image.get(row, c)).collect();
            let mean = vals.iter().sum::<f64>() / w as f64;
            let var = vals.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / w as f64;
            var.sqrt()
        })
        .collect();

    let global_mean = line_rms.iter().sum::<f64>() / h as f64;
    let global_std = {
        let var = line_rms
            .iter()
            .map(|&r| (r - global_mean).powi(2))
            .sum::<f64>()
            / h as f64;
        var.sqrt()
    };

    for row in 0..h {
        if line_rms[row] > global_mean + threshold_sigma * global_std {
            // Replace with average of neighbours
            let prev = if row > 0 { row - 1 } else { row };
            let next = if row + 1 < h { row + 1 } else { row };
            for col in 0..w {
                let v = (image.get(prev, col) + image.get(next, col)) / 2.0;
                result.set(row, col, v);
            }
        }
    }
    result
}

// ---------------------------------------------------------------------------
// Tip-enhanced Raman / infrared helpers
// ---------------------------------------------------------------------------

/// Enhancement factor for tip-enhanced spectroscopy (TERS/TEIR).
///
/// Electric field enhancement: E_tip / E_0 ≈ (R/d + 1)^3 for metallic tip
/// near resonance (dipole approximation). Raman enhancement ∝ |E/E0|^4.
pub fn field_enhancement(tip_radius_nm: f64, gap_nm: f64) -> f64 {
    let ratio = tip_radius_nm / (gap_nm + 1.0);
    (ratio + 1.0).powi(3)
}

pub fn raman_enhancement(tip_radius_nm: f64, gap_nm: f64) -> f64 {
    field_enhancement(tip_radius_nm, gap_nm).powi(2)
}

// ---------------------------------------------------------------------------
// Full processor struct
// ---------------------------------------------------------------------------

/// Top-level SNOM signal processor.
pub struct SnomProcessor {
    pub params: SnomParameters,
    lock_in_amplifiers: Vec<LockInAmplifier>,
    afm_feedback: AfmFeedback,
}

impl SnomProcessor {
    pub fn new(params: SnomParameters, sample_rate_hz: f64) -> Self {
        let mut lock_in_amplifiers = Vec::new();
        for n in 1..=params.max_harmonic {
            lock_in_amplifiers.push(LockInAmplifier::new(
                n,
                params.tapping_frequency_hz,
                sample_rate_hz,
                params.lockin_time_constant_s,
            ));
        }
        let setpoint = match params.regulation {
            TappingRegulation::AmplitudeModulation { setpoint_fraction } => setpoint_fraction,
            _ => 0.8,
        };
        let afm_feedback = AfmFeedback::new(
            params.tapping_frequency_hz,
            params.tapping_amplitude_nm,
            setpoint,
        );
        Self {
            params,
            lock_in_amplifiers,
            afm_feedback,
        }
    }

    /// Process one detector buffer, return near-field harmonics.
    pub fn process(&mut self, detector: &[f64]) -> NearFieldSignal {
        extract_near_field_harmonics(
            detector,
            self.params.tapping_frequency_hz,
            // Use a nominal sample rate recovered from LIA parameters
            self.lock_in_amplifiers
                .first()
                .map(|l| l.sample_rate_hz)
                .unwrap_or(1_000_000.0),
            self.params.max_harmonic,
            self.params.lockin_time_constant_s,
        )
    }

    /// Estimated near-field decay length at current parameters.
    pub fn near_field_decay_length_nm(&self) -> f64 {
        self.params.tip_radius_nm
    }

    /// Spatial resolution estimate.
    pub fn resolution_nm(&self) -> f64 {
        estimate_resolution_nm(
            self.params.mode,
            self.params.tip_radius_nm,
            self.params.wavelength_nm,
        )
    }

    /// Abbe diffraction limit for reference.
    pub fn diffraction_limit_nm(&self) -> f64 {
        abbe_limit_nm(self.params.wavelength_nm, 0.7) // typical NA
    }

    /// Ratio of diffraction limit to near-field resolution (enhancement factor).
    pub fn resolution_enhancement(&self) -> f64 {
        self.diffraction_limit_nm() / self.resolution_nm()
    }
}

// ---------------------------------------------------------------------------
// Unit tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: generate a sinusoidal test signal
    fn gen_sinusoid(freq_hz: f64, sample_rate_hz: f64, n: usize, amplitude: f64) -> Vec<f64> {
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 / sample_rate_hz).sin())
            .collect()
    }

    // --- Complex arithmetic ---

    #[test]
    fn test_complex_abs() {
        let c = Complex::new(3.0, 4.0);
        assert!((c.abs() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_arg() {
        let c = Complex::from_polar(1.0, PI / 4.0);
        assert!((c.arg() - PI / 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_mul() {
        let a = Complex::new(1.0, 0.0);
        let b = Complex::new(0.0, 1.0);
        let ab = a.mul(&b);
        assert!((ab.re - 0.0).abs() < 1e-12);
        assert!((ab.im - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_conj() {
        let c = Complex::new(2.0, 3.0);
        let cc = c.conj();
        assert!((cc.re - 2.0).abs() < 1e-12 && (cc.im + 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_complex_polar_roundtrip() {
        let amp = 2.5;
        let phase = 1.2;
        let c = Complex::from_polar(amp, phase);
        assert!((c.abs() - amp).abs() < 1e-10);
        assert!((c.arg() - phase).abs() < 1e-10);
    }

    // --- Lock-in amplifier ---

    #[test]
    fn test_lockin_detects_tone() {
        let fs = 1_000_000.0_f64;
        let omega = 50_000.0_f64; // 50 kHz tapping
        let n_samples = 10000;
        let signal = gen_sinusoid(omega, fs, n_samples, 1.0);

        let mut lia = LockInAmplifier::new(1, omega, fs, 1e-4);
        let result = lia.process_block(&signal);
        // Amplitude should be close to 1.0 at fundamental
        assert!(result.amplitude > 0.5, "LIA should detect fundamental tone");
    }

    #[test]
    fn test_lockin_harmonic_rejection() {
        let fs = 1_000_000.0_f64;
        let omega = 50_000.0_f64;
        let n_samples = 10000;
        // Signal at 1st harmonic only
        let signal = gen_sinusoid(omega, fs, n_samples, 1.0);

        // Demodulate at 2nd harmonic — should be very small
        let mut lia2 = LockInAmplifier::new(2, omega, fs, 1e-4);
        let result2 = lia2.process_block(&signal);
        assert!(result2.amplitude < 0.1, "2nd harmonic LIA should suppress 1st-harmonic signal");
    }

    #[test]
    fn test_lockin_second_harmonic() {
        let fs = 2_000_000.0_f64;
        let omega = 100_000.0_f64;
        let n_samples = 20000;
        // Signal contains 2nd harmonic at amplitude 0.5
        let signal: Vec<f64> = (0..n_samples)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * 2.0 * PI * omega * t).sin()
            })
            .collect();

        let mut lia = LockInAmplifier::new(2, omega, fs, 5e-5);
        let result = lia.process_block(&signal);
        assert!(result.amplitude > 0.2, "Should detect 2nd harmonic");
    }

    #[test]
    fn test_lockin_reset() {
        let fs = 1_000_000.0_f64;
        let omega = 50_000.0_f64;
        let signal = gen_sinusoid(omega, fs, 1000, 1.0);
        let mut lia = LockInAmplifier::new(1, omega, fs, 1e-3);
        lia.process_block(&signal);
        lia.reset();
        assert!((lia.x_state).abs() < 1e-30 && (lia.y_state).abs() < 1e-30);
    }

    // --- Near-field extraction ---

    #[test]
    fn test_extract_harmonics_count() {
        let fs = 1_000_000.0_f64;
        let omega = 50_000.0_f64;
        let signal = gen_sinusoid(omega, fs, 5000, 1.0);
        let nf = extract_near_field_harmonics(&signal, omega, fs, 3, 1e-4);
        assert_eq!(nf.harmonics.len(), 3);
        assert_eq!(nf.harmonics[0].harmonic, 1);
        assert_eq!(nf.harmonics[2].harmonic, 3);
    }

    #[test]
    fn test_extract_harmonics_background() {
        let signal = vec![2.0; 1000];
        let nf = extract_near_field_harmonics(&signal, 50_000.0, 1_000_000.0, 2, 1e-4);
        assert!((nf.background_estimate - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_extract_harmonics_empty() {
        let nf = extract_near_field_harmonics(&[], 50_000.0, 1_000_000.0, 3, 1e-4);
        assert_eq!(nf.harmonics.len(), 3);
        assert!((nf.background_estimate).abs() < 1e-30);
    }

    // --- Approach curve ---

    #[test]
    fn test_approach_curve_fit() {
        let points = generate_approach_curve(1.0, 20.0, 0.0, 100.0, 50, 0.0);
        let fit = fit_approach_curve(&points, 20.0).unwrap();
        assert!((fit.s0 - 1.0).abs() < 0.01, "s0 should be ≈1.0, got {}", fit.s0);
        assert!(
            (fit.decay_length_nm - 20.0).abs() < 2.0,
            "Decay length should be ≈20 nm, got {}",
            fit.decay_length_nm
        );
        assert!(fit.is_near_field, "Should be classified as near-field");
    }

    #[test]
    fn test_approach_curve_not_near_field() {
        // Long decay — not near-field
        let points = generate_approach_curve(1.0, 500.0, 0.0, 2000.0, 50, 0.0);
        let fit = fit_approach_curve(&points, 20.0).unwrap();
        assert!(!fit.is_near_field, "Long decay should not be classified as near-field");
    }

    #[test]
    fn test_approach_curve_too_few_points() {
        let points = vec![
            ApproachPoint { distance_nm: 0.0, signal: 1.0 },
            ApproachPoint { distance_nm: 10.0, signal: 0.5 },
        ];
        let fit = fit_approach_curve(&points, 20.0);
        assert!(fit.is_none(), "Should return None for < 3 points");
    }

    #[test]
    fn test_approach_curve_with_noise() {
        let points = generate_approach_curve(1.0, 15.0, 0.0, 80.0, 60, 0.05);
        let fit = fit_approach_curve(&points, 15.0).unwrap();
        // With 5% noise, fit should still be within ±5 nm
        assert!(
            (fit.decay_length_nm - 15.0).abs() < 5.0,
            "Noisy fit should be within 5 nm, got {}",
            fit.decay_length_nm
        );
    }

    // --- AFM feedback ---

    #[test]
    fn test_afm_feedback_setpoint() {
        let fb = AfmFeedback::new(280_000.0_f64, 30.0_f64, 0.8_f64);
        // At large distance, amplitude should be near free amplitude
        let amp_far = fb.amplitude_at_distance(1000.0_f64);
        assert!((amp_far - 30.0_f64).abs() < 1.0_f64, "Far amplitude should be ≈free amplitude");
    }

    #[test]
    fn test_afm_feedback_close_damping() {
        let fb = AfmFeedback::new(280_000.0_f64, 30.0_f64, 0.8_f64);
        let amp_close = fb.amplitude_at_distance(0.1_f64);
        let amp_far = fb.amplitude_at_distance(1000.0_f64);
        assert!(amp_close < amp_far, "Amplitude should decrease at close distance");
    }

    #[test]
    fn test_afm_feedback_step() {
        let mut fb = AfmFeedback::new(280_000.0_f64, 30.0_f64, 0.8_f64);
        // If measured amplitude < setpoint, feedback should increase z
        let correction = fb.step(20.0_f64, 1e-4_f64);
        assert!(correction > 0.0, "Feedback should push tip away when amplitude too low");
    }

    // --- Background suppression ---

    #[test]
    fn test_background_suppression_factor() {
        let f1 = background_suppression_factor(1, 30.0, 20.0);
        let f2 = background_suppression_factor(2, 30.0, 20.0);
        let f3 = background_suppression_factor(3, 30.0, 20.0);
        assert!(f3 < f2 && f2 < f1, "Higher harmonics should have lower suppression factor");
        assert!(f1 > 0.0 && f1 <= 1.0);
    }

    #[test]
    fn test_subtract_background() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let bg = vec![0.5, 0.5, 0.5, 0.5];
        let result = subtract_background(&signal, &bg, 1.0);
        assert!((result[0] - 0.5).abs() < 1e-12);
        assert!((result[3] - 3.5).abs() < 1e-12);
    }

    // --- Image processing ---

    #[test]
    fn test_image_creation() {
        let img = SnomImage::new(16, 16, 10.0);
        assert_eq!(img.data.len(), 256);
        assert!((img.get(0, 0)).abs() < 1e-30);
    }

    #[test]
    fn test_image_line_level() {
        let mut img = SnomImage::new(10, 4, 5.0);
        // Add linear ramp to each line
        for row in 0..4 {
            for col in 0..10 {
                img.set(row, col, col as f64 * 1.5 + 3.0);
            }
        }
        img.line_level(1);
        // After 1st-order leveling, residual should be ~0
        let rms = img.rms_roughness();
        assert!(rms < 0.5, "Line-leveled image should have low RMS, got {}", rms);
    }

    #[test]
    fn test_image_plane_subtract() {
        let mut img = SnomImage::new(8, 8, 5.0);
        // Add a tilted plane: z = 0.1*x + 0.2*y
        for row in 0..8 {
            for col in 0..8 {
                img.set(row, col, 0.1 * col as f64 + 0.2 * row as f64);
            }
        }
        img.subtract_plane();
        let rms = img.rms_roughness();
        assert!(rms < 0.1, "After plane subtraction, RMS should be small, got {}", rms);
    }

    #[test]
    fn test_image_normalize_mean() {
        let mut img = SnomImage::new(4, 4, 1.0);
        for v in img.data.iter_mut() {
            *v = 5.0;
        }
        img.normalize_mean();
        let mean = img.data.iter().sum::<f64>() / 16.0;
        assert!(mean.abs() < 1e-10, "Mean should be 0 after normalization");
    }

    #[test]
    fn test_image_range() {
        let mut img = SnomImage::new(2, 2, 1.0);
        img.data = vec![1.0, 3.0, 2.0, -1.0];
        let (min, max) = img.range();
        assert!((min + 1.0).abs() < 1e-12 && (max - 3.0).abs() < 1e-12);
    }

    // --- Spatial resolution ---

    #[test]
    fn test_snom_resolution_scattering() {
        let res = estimate_resolution_nm(SnomMode::ScatteringType, 25.0, 10600.0);
        assert!((res - 25.0).abs() < 1e-10, "s-SNOM resolution should equal tip radius");
    }

    #[test]
    fn test_snom_resolution_aperture() {
        let res = estimate_resolution_nm(SnomMode::Aperture, 25.0, 10600.0);
        assert!(res >= 50.0, "Aperture SNOM resolution should be at least 50 nm");
    }

    #[test]
    fn test_abbe_limit() {
        let limit = abbe_limit_nm(500.0, 1.0);
        assert!((limit - 250.0).abs() < 1e-10, "Abbe limit at λ=500 nm, NA=1.0 should be 250 nm");
    }

    #[test]
    fn test_near_field_psf() {
        let tip = 20.0;
        assert!((near_field_psf(0.0, tip) - 1.0).abs() < 1e-12, "PSF at r=0 should be 1");
        assert!((near_field_psf(tip, tip) - 0.5).abs() < 1e-12, "PSF at r=tip should be 0.5");
        assert!(near_field_psf(100.0, tip) < 0.05, "PSF at r=5*tip should be <5%");
    }

    // --- Dielectric function ---

    #[test]
    fn test_dielectric_gold_ir() {
        let eps = DielectricFunction::new(-3800.0, 2100.0);
        let (n, k) = eps.refractive_index();
        assert!(k > n, "Gold in IR should have k > n (metallic)");
    }

    #[test]
    fn test_drude_model_plasma() {
        // At ω = ω_p, ε₁ should be ≈ 0
        let f_plasma = 2.18e15; // gold plasma freq ~2.18 PHz
        let eps = drude_model(f_plasma, f_plasma, f_plasma / 50.0);
        assert!(eps.epsilon1.abs() < 0.2, "ε₁ should be ≈0 at ω_p");
    }

    #[test]
    fn test_drude_model_low_freq() {
        // Below plasma frequency, ε₁ should be strongly negative (metallic)
        let f_plasma = 1e15;
        let f = 1e13; // well below plasma freq
        let eps = drude_model(f, f_plasma, 1e12);
        assert!(eps.epsilon1 < -100.0, "ε₁ should be very negative below ω_p");
    }

    #[test]
    fn test_near_field_contrast() {
        let eps_metal = DielectricFunction::new(-10.0, 2.0);
        let eps_dielectric = DielectricFunction::new(2.5, 0.01);
        let contrast_m = eps_metal.near_field_contrast_amplitude();
        let contrast_d = eps_dielectric.near_field_contrast_amplitude();
        assert!(
            contrast_m > contrast_d,
            "Metal should have higher near-field contrast than dielectric"
        );
    }

    // --- Material database ---

    #[test]
    fn test_material_database_not_empty() {
        assert!(!material_database().is_empty());
    }

    #[test]
    fn test_find_material_gold() {
        let mat = find_material("Gold").unwrap();
        assert!((mat.wavelength_nm - 10_600.0).abs() < 1.0);
        assert!(mat.epsilon.epsilon1 < 0.0, "Gold should have negative ε₁ in IR");
    }

    #[test]
    fn test_find_material_not_found() {
        assert!(find_material("Unobtanium").is_none());
    }

    // --- Pseudoheterodyne ---

    #[test]
    fn test_pseudohet_produces_result() {
        let fs = 2_000_000.0_f64;
        let omega = 100_000.0_f64;
        let mirror = 300.0_f64;
        let n = 40000;
        // Simulate: sideband at 2Ω + M with amplitude 0.6
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let freq_plus = 2.0 * omega + mirror;
                0.6 * (2.0 * PI * freq_plus * t).sin()
            })
            .collect();

        let phd = PseudohetDemodulator::new(2, omega, mirror, fs, 2e-4);
        let result = phd.demodulate(&signal);
        // sideband_plus should detect the tone; amplitude > 0
        assert!(
            result.sideband_plus.abs() > 0.1,
            "sideband_plus should detect tone at 2Ω+M, got {}",
            result.sideband_plus.abs()
        );
    }

    // --- Nano-spectrum ---

    #[test]
    fn test_nano_spectrum_peak() {
        let wn = vec![900.0, 950.0, 1000.0, 1050.0, 1100.0];
        let amp = vec![0.1, 0.3, 0.9, 0.4, 0.2];
        let phase = vec![0.0; 5];
        let spec = NanoSpectrum::new(wn, amp, phase);
        let peak = spec.peak_wavenumber().unwrap();
        assert!((peak - 1000.0).abs() < 1.0);
    }

    #[test]
    fn test_nano_spectrum_baseline() {
        let wn = vec![900.0, 1000.0, 1100.0];
        let amp = vec![1.0, 2.0, 3.0]; // linear baseline → should become 0 after correction
        let spec = NanoSpectrum::new(wn, amp, vec![0.0; 3]);
        let corrected = spec.baseline_correct();
        // After correcting linear baseline, values should be near 0 at endpoints
        assert!(corrected.amplitude[0].abs() < 1e-10);
        assert!(corrected.amplitude[2].abs() < 1e-10);
    }

    // --- SNR ---

    #[test]
    fn test_snr_db() {
        let snr = estimate_snr_db(1.0, 0.001);
        assert!((snr - 60.0).abs() < 0.1, "SNR should be 60 dB");
    }

    #[test]
    fn test_snr_zero_noise() {
        let snr = estimate_snr_db(1.0, 0.0);
        assert!(snr.is_infinite());
    }

    // --- Field enhancement ---

    #[test]
    fn test_field_enhancement_tip() {
        let fe = field_enhancement(20.0, 1.0);
        assert!(fe > 1.0, "Field enhancement should exceed 1");
    }

    #[test]
    fn test_raman_enhancement() {
        let re = raman_enhancement(20.0, 1.0);
        let fe = field_enhancement(20.0, 1.0);
        assert!((re - fe.powi(2)).abs() < 1e-10, "Raman enhancement = |E/E0|^4");
    }

    // --- Processor ---

    #[test]
    fn test_snom_processor_resolution() {
        let params = SnomParameters::default();
        let proc = SnomProcessor::new(params.clone(), 2_000_000.0);
        let res = proc.resolution_nm();
        assert!((res - params.tip_radius_nm).abs() < 1e-10, "s-SNOM resolution should equal tip radius");
    }

    #[test]
    fn test_snom_processor_enhancement() {
        let params = SnomParameters::default();
        let proc = SnomProcessor::new(params, 2_000_000.0);
        let enh = proc.resolution_enhancement();
        assert!(enh > 1.0, "Near-field should enhance resolution vs diffraction limit");
    }

    #[test]
    fn test_scan_artifact_correction() {
        let mut img = SnomImage::new(8, 8, 5.0);
        // All normal lines: small sinusoidal variation (so per-line RMS > 0)
        for row in 0..8 {
            for col in 0..8 {
                img.set(row, col, 0.1 * (col as f64 * 0.5).sin());
            }
        }
        // Make one line an artifact: high-amplitude noise (large RMS)
        for col in 0..8 {
            img.set(3, col, 500.0 * ((col as f64 * 1.7).sin() + 1.1));
        }
        let corrected = correct_scan_artifacts(&img, 2.0);
        // The artifact row should have been replaced with neighbour interpolation
        let row3_max: f64 = (0..8).map(|c| corrected.get(3, c).abs()).fold(0.0_f64, f64::max);
        assert!(
            row3_max < 10.0,
            "Artifact row should be replaced with neighbour values, max abs = {}",
            row3_max
        );
    }
}
