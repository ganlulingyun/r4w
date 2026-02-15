//! # Atmospheric Gravity Wave Detector
//!
//! Detection and characterisation of atmospheric gravity waves (buoyancy waves)
//! from meteorological sensor data such as radiosonde profiles, microbarograph
//! pressure series, and multi-station wind measurements.
//!
//! ## Overview
//!
//! Atmospheric gravity waves are buoyancy-driven oscillations that propagate
//! through stably stratified layers of the atmosphere.  They transport momentum
//! and energy vertically and horizontally, influencing weather, turbulence, and
//! the general circulation.  This module provides tools for:
//!
//! - **Brunt-Väisälä frequency** computation and stability classification.
//! - **Bandpass FIR filtering** of pressure / temperature perturbation series.
//! - **Morlet wavelet analysis** for time-localised wave-packet detection.
//! - **Hodograph (ellipse) fitting** from (u', v') wind perturbations.
//! - **Dispersion relation** evaluation with scale-height correction.
//! - **Doppler-shift correction** for intrinsic frequency recovery.
//! - **Energy diagnostics**: potential energy Ep, kinetic energy Ek.
//! - **Momentum flux** (Reynolds stress) estimation.
//! - **Stokes polarisation parameters** (degree of polarisation, axial ratio).
//! - **Source identification heuristics** (mountain, convective, jet-stream).
//!
//! ## Key Equations
//!
//! - **Brunt-Väisälä**: N² = (g/T)(dT/dz + Γ_d),  Γ_d = g/c_p ≈ 9.8 K/km
//! - **Dispersion**: ω² = N² k_h² / (k_h² + k_z² + 1/(4H²)),  H ≈ 8.5 km
//! - **Intrinsic freq**: ω_i = ω_obs − k_h · U  (Doppler correction)
//! - **Potential energy**: Ep = ½ (g/N)² (T'/T̄)²
//! - **Kinetic energy**: Ek = ½ (u'² + v'² + w'²)
//! - **Momentum flux**: F = ρ₀ ⟨u' w'⟩
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::atmospheric_gravity_wave_detector::{
//!     AtmosphericProfile, BruntVaisala, StabilityClass,
//! };
//!
//! let profile = AtmosphericProfile {
//!     altitude_m: vec![0.0, 1000.0, 2000.0, 3000.0],
//!     temperature_k: vec![288.0, 282.0, 276.0, 270.0],
//!     pressure_hpa: vec![1013.25, 900.0, 800.0, 700.0],
//!     wind_speed_ms: vec![5.0, 8.0, 12.0, 10.0],
//!     wind_direction_deg: vec![270.0, 260.0, 250.0, 245.0],
//! };
//!
//! let bv = BruntVaisala::from_profile(&profile);
//! assert!(!bv.n_squared.is_empty());
//! assert_eq!(bv.classify(0), StabilityClass::Stable);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Gravitational acceleration (m/s²).
const G: f64 = 9.80665;

/// Specific heat of dry air at constant pressure (J/(kg·K)).
const CP: f64 = 1004.0;

/// Dry adiabatic lapse rate Γ_d = g/c_p (K/m).
const GAMMA_D: f64 = G / CP;

/// Specific gas constant for dry air (J/(kg·K)).
const R_AIR: f64 = 287.058;

/// Mean molar mass of air (kg/mol).
const M_AIR: f64 = 0.0289644;

/// Universal gas constant (J/(mol·K)).
const R_UNIVERSAL: f64 = 8.31446;

/// Approximate atmospheric scale height (m).
const SCALE_HEIGHT: f64 = R_AIR * 250.0 / G; // ~7.3 km (cold atmosphere)

/// Default scale height used in dispersion relation (m).
const DEFAULT_SCALE_HEIGHT_M: f64 = 8500.0;

// ---------------------------------------------------------------------------
// AtmosphericProfile
// ---------------------------------------------------------------------------

/// Vertical atmospheric profile from radiosonde or NWP model output.
///
/// Each vector must have the same length; elements are ordered by increasing
/// altitude.
#[derive(Debug, Clone)]
pub struct AtmosphericProfile {
    /// Altitude above mean sea level (m).
    pub altitude_m: Vec<f64>,
    /// Absolute temperature (K).
    pub temperature_k: Vec<f64>,
    /// Atmospheric pressure (hPa).
    pub pressure_hpa: Vec<f64>,
    /// Wind speed (m/s).
    pub wind_speed_ms: Vec<f64>,
    /// Wind direction (degrees from north, meteorological convention).
    pub wind_direction_deg: Vec<f64>,
}

impl AtmosphericProfile {
    /// Number of levels in the profile.
    pub fn len(&self) -> usize {
        self.altitude_m.len()
    }

    /// Returns `true` when the profile has no levels.
    pub fn is_empty(&self) -> bool {
        self.altitude_m.is_empty()
    }

    /// Mean temperature across all levels (K).
    pub fn mean_temperature(&self) -> f64 {
        if self.temperature_k.is_empty() {
            return 0.0;
        }
        let sum: f64 = self.temperature_k.iter().sum();
        sum / self.temperature_k.len() as f64
    }

    /// Compute (u, v) wind components for each level.
    pub fn wind_components(&self) -> (Vec<f64>, Vec<f64>) {
        let mut u = Vec::with_capacity(self.len());
        let mut v = Vec::with_capacity(self.len());
        for i in 0..self.len() {
            let dir_rad = self.wind_direction_deg[i].to_radians();
            // Meteorological convention: wind FROM direction
            u.push(-self.wind_speed_ms[i] * dir_rad.sin());
            v.push(-self.wind_speed_ms[i] * dir_rad.cos());
        }
        (u, v)
    }

    /// Air density at level `i` via the ideal gas law: ρ = p / (R T).
    pub fn density_at(&self, i: usize) -> f64 {
        let p_pa = self.pressure_hpa[i] * 100.0;
        p_pa / (R_AIR * self.temperature_k[i])
    }
}

// ---------------------------------------------------------------------------
// Brunt-Väisälä frequency
// ---------------------------------------------------------------------------

/// Atmospheric stability classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StabilityClass {
    /// N² > 0 — oscillatory (gravity waves can propagate).
    Stable,
    /// N² ≈ 0 — neutral.
    Neutral,
    /// N² < 0 — convectively unstable.
    Unstable,
}

/// Brunt-Väisälä frequency profile computed from an [`AtmosphericProfile`].
#[derive(Debug, Clone)]
pub struct BruntVaisala {
    /// N² values at mid-layer altitudes (s⁻²).
    pub n_squared: Vec<f64>,
    /// Mid-layer altitudes (m).
    pub altitudes_m: Vec<f64>,
}

impl BruntVaisala {
    /// Compute N² at each layer mid-point.
    ///
    /// N² = (g/T̄)(dT/dz + Γ_d)
    pub fn from_profile(profile: &AtmosphericProfile) -> Self {
        let n = profile.len();
        let mut n_sq = Vec::with_capacity(n.saturating_sub(1));
        let mut alts = Vec::with_capacity(n.saturating_sub(1));

        for i in 0..n.saturating_sub(1) {
            let dz = profile.altitude_m[i + 1] - profile.altitude_m[i];
            if dz.abs() < 1e-6 {
                continue;
            }
            let dt_dz =
                (profile.temperature_k[i + 1] - profile.temperature_k[i]) / dz;
            let t_bar =
                0.5 * (profile.temperature_k[i] + profile.temperature_k[i + 1]);
            let val = (G / t_bar) * (dt_dz + GAMMA_D);
            n_sq.push(val);
            alts.push(0.5 * (profile.altitude_m[i] + profile.altitude_m[i + 1]));
        }
        Self {
            n_squared: n_sq,
            altitudes_m: alts,
        }
    }

    /// Classify the stability at layer `i`.
    pub fn classify(&self, i: usize) -> StabilityClass {
        let threshold = 1e-7; // small margin around zero
        if self.n_squared[i] > threshold {
            StabilityClass::Stable
        } else if self.n_squared[i] < -threshold {
            StabilityClass::Unstable
        } else {
            StabilityClass::Neutral
        }
    }

    /// Mean N² across all layers.
    pub fn mean_n_squared(&self) -> f64 {
        if self.n_squared.is_empty() {
            return 0.0;
        }
        self.n_squared.iter().sum::<f64>() / self.n_squared.len() as f64
    }

    /// Mean Brunt-Väisälä frequency N (rad/s) – only defined when mean N² > 0.
    pub fn mean_frequency(&self) -> Option<f64> {
        let m = self.mean_n_squared();
        if m > 0.0 {
            Some(m.sqrt())
        } else {
            None
        }
    }

    /// Mean Brunt-Väisälä period (s).
    pub fn mean_period(&self) -> Option<f64> {
        self.mean_frequency().map(|n| 2.0 * PI / n)
    }
}

// ---------------------------------------------------------------------------
// Dispersion relation
// ---------------------------------------------------------------------------

/// Evaluate the gravity-wave dispersion relation.
///
/// ω² = N² k_h² / (k_h² + k_z² + 1/(4 H²))
///
/// Returns the angular frequency ω (rad/s).
pub fn dispersion_omega(
    n_squared: f64,
    k_h: f64,
    k_z: f64,
    scale_height_m: f64,
) -> f64 {
    let denom = k_h * k_h + k_z * k_z + 1.0 / (4.0 * scale_height_m * scale_height_m);
    if denom <= 0.0 || n_squared <= 0.0 {
        return 0.0;
    }
    let omega_sq = n_squared * k_h * k_h / denom;
    if omega_sq > 0.0 {
        omega_sq.sqrt()
    } else {
        0.0
    }
}

/// Compute horizontal wavenumber from wavelength (m).
pub fn wavenumber_from_wavelength(wavelength_m: f64) -> f64 {
    if wavelength_m.abs() < 1e-12 {
        return 0.0;
    }
    2.0 * PI / wavelength_m
}

/// Compute intrinsic frequency with Doppler-shift correction.
///
/// ω_i = ω_obs − k_h · U
///
/// where U is the background wind component along the wave propagation direction.
pub fn intrinsic_frequency(omega_obs: f64, k_h: f64, u_wind: f64) -> f64 {
    omega_obs - k_h * u_wind
}

// ---------------------------------------------------------------------------
// FIR bandpass filter (simple windowed-sinc)
// ---------------------------------------------------------------------------

/// Design a bandpass FIR filter using a Hamming-windowed sinc.
///
/// `f_low` and `f_high` are normalised frequencies (0..0.5 of sample rate).
fn design_bandpass_fir(order: usize, f_low: f64, f_high: f64) -> Vec<f64> {
    let n = if order % 2 == 0 { order + 1 } else { order };
    let m = n / 2;
    let mut h = vec![0.0; n];

    for i in 0..n {
        let x = i as f64 - m as f64;
        // Hamming window
        let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1) as f64).cos();
        if x.abs() < 1e-12 {
            h[i] = 2.0 * (f_high - f_low) * w;
        } else {
            let sinc_hi = (2.0 * PI * f_high * x).sin() / (PI * x);
            let sinc_lo = (2.0 * PI * f_low * x).sin() / (PI * x);
            h[i] = (sinc_hi - sinc_lo) * w;
        }
    }
    // Normalise to unit gain at centre frequency
    let sum: f64 = h.iter().sum();
    if sum.abs() > 1e-12 {
        let centre = PI * (f_low + f_high);
        let mut gain = 0.0;
        for (i, coeff) in h.iter().enumerate() {
            let x = i as f64 - m as f64;
            gain += coeff * (centre * x).cos();
        }
        if gain.abs() > 1e-12 {
            for coeff in &mut h {
                *coeff /= gain;
            }
        }
    }
    h
}

/// Apply an FIR filter to a signal (direct convolution).
fn fir_filter(signal: &[f64], coeffs: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let m = coeffs.len();
    if n == 0 || m == 0 {
        return vec![];
    }
    let mut out = vec![0.0; n];
    for i in 0..n {
        let mut acc = 0.0;
        for j in 0..m {
            if i >= j {
                acc += coeffs[j] * signal[i - j];
            }
        }
        out[i] = acc;
    }
    out
}

// ---------------------------------------------------------------------------
// Morlet wavelet transform
// ---------------------------------------------------------------------------

/// Morlet wavelet evaluated at dimensionless time η.
///
/// ψ(η) = π^{-1/4} exp(i ω₀ η) exp(−η²/2)
fn morlet_wavelet(eta: f64, omega0: f64) -> (f64, f64) {
    let envelope = PI.powf(-0.25) * (-0.5 * eta * eta).exp();
    let phase = omega0 * eta;
    (envelope * phase.cos(), envelope * phase.sin())
}

/// Continuous Wavelet Transform (CWT) with Morlet wavelet.
///
/// Returns a 2D power array indexed as `[scale_idx][time_idx]`.
pub fn morlet_cwt(
    signal: &[f64],
    dt: f64,
    scales: &[f64],
    omega0: f64,
) -> Vec<Vec<f64>> {
    let n = signal.len();
    let mut power = Vec::with_capacity(scales.len());

    for &s in scales {
        let mut row = Vec::with_capacity(n);
        for t in 0..n {
            let mut re = 0.0;
            let mut im = 0.0;
            // Convolve with scaled, translated wavelet
            let half_win = (6.0 * s / dt).ceil() as usize; // 6σ window
            let i_start = if t > half_win { t - half_win } else { 0 };
            let i_end = (t + half_win + 1).min(n);
            for i in i_start..i_end {
                let eta = ((i as f64 - t as f64) * dt) / s;
                let (wr, wi) = morlet_wavelet(eta, omega0);
                let norm = (dt / s).sqrt();
                re += signal[i] * wr * norm;
                im += signal[i] * wi * norm;
            }
            row.push(re * re + im * im);
        }
        power.push(row);
    }
    power
}

/// Convert wavelet scales to approximate periods (s) for Morlet wavelet.
pub fn scale_to_period(scale: f64, omega0: f64) -> f64 {
    (4.0 * PI * scale) / (omega0 + (2.0 + omega0 * omega0).sqrt())
}

// ---------------------------------------------------------------------------
// Hodograph / ellipse fitting
// ---------------------------------------------------------------------------

/// Result of a hodograph ellipse fit.
#[derive(Debug, Clone)]
pub struct HodographEllipse {
    /// Semi-major axis length (m/s).
    pub semi_major: f64,
    /// Semi-minor axis length (m/s).
    pub semi_minor: f64,
    /// Orientation angle of the major axis (radians from east, CCW positive).
    pub orientation_rad: f64,
    /// Axial ratio (minor/major), dimensionless ∈ [0, 1].
    pub axial_ratio: f64,
    /// Sense of rotation: +1 = CW (anticyclonic), -1 = CCW (cyclonic).
    pub rotation_sense: i32,
}

/// Fit an ellipse to a wind perturbation hodograph using covariance.
///
/// `u_prime` and `v_prime` are the perturbation zonal and meridional winds.
pub fn fit_hodograph_ellipse(u_prime: &[f64], v_prime: &[f64]) -> Option<HodographEllipse> {
    let n = u_prime.len().min(v_prime.len());
    if n < 3 {
        return None;
    }
    // Means
    let u_mean: f64 = u_prime.iter().take(n).sum::<f64>() / n as f64;
    let v_mean: f64 = v_prime.iter().take(n).sum::<f64>() / n as f64;

    // Covariance matrix elements
    let mut cuu = 0.0;
    let mut cvv = 0.0;
    let mut cuv = 0.0;
    for i in 0..n {
        let du = u_prime[i] - u_mean;
        let dv = v_prime[i] - v_mean;
        cuu += du * du;
        cvv += dv * dv;
        cuv += du * dv;
    }
    cuu /= n as f64;
    cvv /= n as f64;
    cuv /= n as f64;

    // Eigenvalues of 2x2 covariance matrix
    let trace = cuu + cvv;
    let det = cuu * cvv - cuv * cuv;
    let disc = (trace * trace - 4.0 * det).max(0.0);
    let lambda1 = 0.5 * (trace + disc.sqrt());
    let lambda2 = 0.5 * (trace - disc.sqrt());

    let semi_major = lambda1.max(0.0).sqrt();
    let semi_minor = lambda2.max(0.0).sqrt();

    // Orientation of major axis
    let orientation_rad = 0.5 * (2.0 * cuv).atan2(cuu - cvv);

    let axial_ratio = if semi_major > 1e-12 {
        semi_minor / semi_major
    } else {
        0.0
    };

    // Rotation sense from cross-product of consecutive perturbation vectors
    let mut cross_sum = 0.0;
    for i in 0..n - 1 {
        cross_sum += u_prime[i] * v_prime[i + 1] - u_prime[i + 1] * v_prime[i];
    }
    let rotation_sense = if cross_sum >= 0.0 { 1 } else { -1 };

    Some(HodographEllipse {
        semi_major,
        semi_minor,
        orientation_rad,
        axial_ratio,
        rotation_sense,
    })
}

// ---------------------------------------------------------------------------
// Energy diagnostics
// ---------------------------------------------------------------------------

/// Potential energy density (J/kg).
///
/// Ep = 0.5 * (g/N)² * (T'/T̄)²
pub fn potential_energy_density(
    t_perturbation: f64,
    t_mean: f64,
    n_squared: f64,
) -> f64 {
    if n_squared <= 0.0 || t_mean.abs() < 1e-6 {
        return 0.0;
    }
    0.5 * (G * G / n_squared) * (t_perturbation / t_mean).powi(2)
}

/// Kinetic energy density (J/kg).
///
/// Ek = 0.5 * (u'² + v'² + w'²)
pub fn kinetic_energy_density(u_prime: f64, v_prime: f64, w_prime: f64) -> f64 {
    0.5 * (u_prime * u_prime + v_prime * v_prime + w_prime * w_prime)
}

/// Momentum flux (Pa) = ρ₀ ⟨u'w'⟩.
///
/// `u_prime` and `w_prime` are perturbation zonal and vertical wind vectors.
pub fn momentum_flux(rho0: f64, u_prime: &[f64], w_prime: &[f64]) -> f64 {
    let n = u_prime.len().min(w_prime.len());
    if n == 0 {
        return 0.0;
    }
    let cov: f64 = u_prime.iter().zip(w_prime.iter()).map(|(u, w)| u * w).sum::<f64>()
        / n as f64;
    rho0 * cov
}

// ---------------------------------------------------------------------------
// Stokes parameters for wave polarisation
// ---------------------------------------------------------------------------

/// Stokes-like polarisation parameters for gravity waves.
#[derive(Debug, Clone)]
pub struct StokesParameters {
    /// Total intensity (S0 ≡ I).
    pub s0: f64,
    /// Degree of linear polarisation component (S1 ≡ Q).
    pub s1: f64,
    /// Off-diagonal component (S2 ≡ U).
    pub s2: f64,
    /// Circular component (S3 ≡ V) from quadrature spectrum.
    pub s3: f64,
    /// Degree of polarisation d = sqrt(S1² + S2² + S3²) / S0  ∈ [0, 1].
    pub degree_of_polarisation: f64,
    /// Axial ratio from Stokes: tan(χ) where sin(2χ) = S3/S0·d.
    pub axial_ratio: f64,
}

/// Compute Stokes parameters from wind perturbation components.
///
/// Uses covariance and quadrature-covariance (Hilbert-transformed) to
/// estimate polarisation state.
pub fn stokes_parameters(u_prime: &[f64], v_prime: &[f64]) -> Option<StokesParameters> {
    let n = u_prime.len().min(v_prime.len());
    if n < 4 {
        return None;
    }
    // Compute covariance elements
    let u_mean: f64 = u_prime.iter().take(n).sum::<f64>() / n as f64;
    let v_mean: f64 = v_prime.iter().take(n).sum::<f64>() / n as f64;

    let mut puu = 0.0;
    let mut pvv = 0.0;
    let mut puv = 0.0;
    for i in 0..n {
        let du = u_prime[i] - u_mean;
        let dv = v_prime[i] - v_mean;
        puu += du * du;
        pvv += dv * dv;
        puv += du * dv;
    }
    puu /= n as f64;
    pvv /= n as f64;
    puv /= n as f64;

    // Quadrature covariance via discrete Hilbert approximation (90° phase shift)
    // Simple approximation: use derivative-based quadrature
    let mut puv_q = 0.0;
    let count = n.saturating_sub(2);
    if count == 0 {
        return None;
    }
    for i in 1..n - 1 {
        let du = u_prime[i] - u_mean;
        // Central difference as Hilbert-like quadrature
        let dv_q = 0.5 * (v_prime[i + 1] - v_prime[i - 1]) - v_mean * 0.0;
        puv_q += du * dv_q;
    }
    puv_q /= count as f64;

    let s0 = puu + pvv;
    let s1 = puu - pvv;
    let s2 = 2.0 * puv;
    let s3 = 2.0 * puv_q;

    let pol_power = (s1 * s1 + s2 * s2 + s3 * s3).sqrt();
    let degree = if s0 > 1e-12 { pol_power / s0 } else { 0.0 };

    // Axial ratio from ellipticity angle χ
    let sin_2chi = if pol_power > 1e-12 {
        (s3 / pol_power).clamp(-1.0, 1.0)
    } else {
        0.0
    };
    let chi = 0.5 * sin_2chi.asin();
    let axial_ratio = chi.tan().abs().min(1.0);

    Some(StokesParameters {
        s0,
        s1,
        s2,
        s3,
        degree_of_polarisation: degree,
        axial_ratio,
    })
}

// ---------------------------------------------------------------------------
// Source identification
// ---------------------------------------------------------------------------

/// Heuristic gravity-wave source category.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaveSource {
    /// Mountain wave — stationary relative to terrain, k_h aligned with wind.
    Mountain,
    /// Convective wave — non-stationary, broad spectrum.
    Convective,
    /// Jet-stream wave — associated with wind-shear maxima.
    JetStream,
    /// Unknown / unclassified source.
    Unknown,
}

/// Simple heuristic source classification.
///
/// - **Mountain**: intrinsic frequency near zero (stationary), wind aligned
/// - **Convective**: large spectral spread, short periods
/// - **Jet-stream**: wave located near the level of maximum wind shear
pub fn classify_source(
    intrinsic_freq: f64,
    spectral_width: f64,
    shear_proximity: f64,
    stationarity: f64,
) -> WaveSource {
    // stationarity ∈ [0, 1]: 1 = perfectly stationary
    if stationarity > 0.8 && intrinsic_freq.abs() < 1e-3 {
        return WaveSource::Mountain;
    }
    if spectral_width > 0.5 {
        return WaveSource::Convective;
    }
    if shear_proximity < 0.2 {
        return WaveSource::JetStream;
    }
    WaveSource::Unknown
}

// ---------------------------------------------------------------------------
// Detected wave event
// ---------------------------------------------------------------------------

/// Parameters of a single detected gravity-wave event.
#[derive(Debug, Clone)]
pub struct GravityWaveEvent {
    /// Observed angular frequency (rad/s).
    pub omega_obs: f64,
    /// Observed period (s).
    pub period_s: f64,
    /// Horizontal wavelength (m).
    pub horizontal_wavelength_m: f64,
    /// Vertical wavelength (m), if determined.
    pub vertical_wavelength_m: Option<f64>,
    /// Pressure perturbation amplitude (hPa).
    pub pressure_amplitude_hpa: f64,
    /// Temperature perturbation amplitude (K).
    pub temperature_amplitude_k: f64,
    /// Intrinsic frequency after Doppler correction (rad/s).
    pub intrinsic_freq: f64,
    /// Potential energy density (J/kg).
    pub potential_energy: f64,
    /// Kinetic energy density (J/kg).
    pub kinetic_energy: f64,
    /// Momentum flux (Pa).
    pub momentum_flux: f64,
    /// Heuristic source classification.
    pub source: WaveSource,
}

// ---------------------------------------------------------------------------
// GravityWaveDetector
// ---------------------------------------------------------------------------

/// Configurable gravity-wave detector.
#[derive(Debug, Clone)]
pub struct GravityWaveDetector {
    /// Minimum wave period to detect (s).  Default 300 (5 min).
    pub min_period_s: f64,
    /// Maximum wave period to detect (s).  Default 10800 (3 h).
    pub max_period_s: f64,
    /// Minimum horizontal wavelength (m).  Default 10 000 (10 km).
    pub min_wavelength_m: f64,
    /// Detection threshold on normalised amplitude (0..1).
    pub detection_threshold: f64,
    /// FIR filter order for bandpass.
    pub fir_order: usize,
    /// Morlet wavelet central angular frequency ω₀.
    pub morlet_omega0: f64,
}

impl Default for GravityWaveDetector {
    fn default() -> Self {
        Self {
            min_period_s: 300.0,
            max_period_s: 10800.0,
            min_wavelength_m: 10_000.0,
            detection_threshold: 0.1,
            fir_order: 63,
            morlet_omega0: 6.0,
        }
    }
}

impl GravityWaveDetector {
    /// Create a detector with default parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a detector with custom period range (s).
    pub fn with_period_range(min_s: f64, max_s: f64) -> Self {
        Self {
            min_period_s: min_s,
            max_period_s: max_s,
            ..Default::default()
        }
    }

    /// Bandpass-filter a time series to isolate the gravity-wave band.
    ///
    /// `dt` is the sample interval (s).
    pub fn bandpass_filter(&self, signal: &[f64], dt: f64) -> Vec<f64> {
        let fs = 1.0 / dt;
        // Normalised frequencies
        let f_low = (1.0 / self.max_period_s) / fs;
        let f_high = (1.0 / self.min_period_s) / fs;
        let f_low = f_low.max(1e-6).min(0.499);
        let f_high = f_high.max(f_low + 1e-6).min(0.499);
        let coeffs = design_bandpass_fir(self.fir_order, f_low, f_high);
        fir_filter(signal, &coeffs)
    }

    /// Detect gravity waves from a pressure perturbation time series.
    ///
    /// Returns detected events.  `dt` is the sample interval (s).
    pub fn detect_from_pressure(
        &self,
        pressure_hpa: &[f64],
        dt: f64,
        background_temp_k: f64,
        n_squared: f64,
        _rho0: f64,
    ) -> Vec<GravityWaveEvent> {
        if pressure_hpa.len() < self.fir_order * 2 {
            return vec![];
        }

        // 1. Remove mean
        let mean_p: f64 =
            pressure_hpa.iter().sum::<f64>() / pressure_hpa.len() as f64;
        let detrended: Vec<f64> = pressure_hpa.iter().map(|p| p - mean_p).collect();

        // 2. Bandpass filter
        let filtered = self.bandpass_filter(&detrended, dt);

        // 3. Find peak amplitude
        let max_amp = filtered
            .iter()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max);

        if max_amp < 1e-12 {
            return vec![];
        }

        // 4. Identify peaks above threshold
        let threshold = self.detection_threshold * max_amp;
        let mut events = Vec::new();
        let mut i = 1;
        while i < filtered.len() - 1 {
            if filtered[i].abs() > threshold
                && filtered[i].abs() >= filtered[i - 1].abs()
                && filtered[i].abs() >= filtered[i + 1].abs()
            {
                // Estimate local period via zero-crossing distance
                let period = self.estimate_period_around(&filtered, i, dt);
                if let Some(period_s) = period {
                    if period_s >= self.min_period_s && period_s <= self.max_period_s {
                        let omega_obs = 2.0 * PI / period_s;
                        let p_amp = filtered[i].abs();
                        // Temperature perturbation from pressure: T'/T ≈ p'/p · (γ-1)/γ
                        let t_amp = p_amp / mean_p * background_temp_k * 0.286; // (γ-1)/γ for air
                        let ep = potential_energy_density(t_amp, background_temp_k, n_squared);
                        let k_h = wavenumber_from_wavelength(self.min_wavelength_m * 10.0); // nominal estimate
                        let _omega_disp = dispersion_omega(n_squared, k_h, 0.0, DEFAULT_SCALE_HEIGHT_M);

                        events.push(GravityWaveEvent {
                            omega_obs,
                            period_s,
                            horizontal_wavelength_m: 2.0 * PI / k_h.max(1e-12),
                            vertical_wavelength_m: None,
                            pressure_amplitude_hpa: p_amp,
                            temperature_amplitude_k: t_amp,
                            intrinsic_freq: omega_obs, // no wind info here
                            potential_energy: ep,
                            kinetic_energy: 0.0, // no wind data
                            momentum_flux: 0.0,
                            source: WaveSource::Unknown,
                        });
                    }
                }
                // Skip ahead to avoid double-counting the same cycle
                let skip = (period.unwrap_or(self.min_period_s) / dt * 0.5) as usize;
                i += skip.max(2);
            } else {
                i += 1;
            }
        }
        events
    }

    /// Detect gravity waves from a full atmospheric profile using hodograph analysis.
    pub fn detect_from_profile(
        &self,
        profile: &AtmosphericProfile,
    ) -> Vec<GravityWaveEvent> {
        if profile.len() < 5 {
            return vec![];
        }

        let bv = BruntVaisala::from_profile(profile);
        let mean_n_sq = bv.mean_n_squared();
        if mean_n_sq <= 0.0 {
            return vec![];
        }

        let (u_all, v_all) = profile.wind_components();

        // Remove polynomial trend (linear fit)
        let u_prime = remove_linear_trend(&u_all);
        let v_prime = remove_linear_trend(&v_all);

        // Hodograph ellipse
        let ellipse = fit_hodograph_ellipse(&u_prime, &v_prime);

        let mut events = Vec::new();

        if let Some(ell) = ellipse {
            // Estimate vertical wavelength from oscillation in the profile
            let t_prime = remove_linear_trend(&profile.temperature_k);
            let vert_wl = estimate_vertical_wavelength(
                &t_prime,
                &profile.altitude_m,
            );

            let ek = kinetic_energy_density(ell.semi_major, ell.semi_minor, 0.0);

            // Estimate temperature perturbation amplitude
            let t_amp = rms(&t_prime);
            let t_mean = profile.mean_temperature();
            let ep = potential_energy_density(t_amp, t_mean, mean_n_sq);

            // Momentum flux estimate (simplified: use semi-major × semi-minor × sign)
            let rho0 = profile.density_at(0);
            let mf = momentum_flux(rho0, &u_prime, &v_prime); // rough proxy

            // Period from dispersion relation
            let k_z = vert_wl
                .map(|wl| wavenumber_from_wavelength(wl))
                .unwrap_or(0.001);
            let k_h = wavenumber_from_wavelength(self.min_wavelength_m * 10.0);
            let omega = dispersion_omega(mean_n_sq, k_h, k_z, DEFAULT_SCALE_HEIGHT_M);
            let period_s = if omega > 1e-12 {
                2.0 * PI / omega
            } else {
                self.max_period_s
            };

            events.push(GravityWaveEvent {
                omega_obs: omega,
                period_s,
                horizontal_wavelength_m: 2.0 * PI / k_h.max(1e-12),
                vertical_wavelength_m: vert_wl,
                pressure_amplitude_hpa: 0.0,
                temperature_amplitude_k: t_amp,
                intrinsic_freq: omega,
                potential_energy: ep,
                kinetic_energy: ek,
                momentum_flux: mf,
                source: WaveSource::Unknown,
            });
        }

        events
    }

    /// Estimate period around sample index `idx` by measuring zero-crossing distance.
    fn estimate_period_around(
        &self,
        signal: &[f64],
        _idx: usize,
        dt: f64,
    ) -> Option<f64> {
        // Search for zero-crossings to the left and right
        let mut crossings = Vec::new();
        for i in 1..signal.len() {
            if signal[i - 1] * signal[i] <= 0.0 {
                crossings.push(i);
            }
        }
        if crossings.len() < 2 {
            return None;
        }

        // Average half-period from zero-crossing spacing
        let mut sum = 0.0;
        let mut count = 0;
        for i in 1..crossings.len() {
            let half_period = (crossings[i] - crossings[i - 1]) as f64 * dt;
            sum += half_period;
            count += 1;
        }
        if count > 0 {
            Some(2.0 * sum / count as f64) // full period = 2 × half-period
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Helper utilities
// ---------------------------------------------------------------------------

/// Remove linear trend from a series (simple least-squares).
fn remove_linear_trend(data: &[f64]) -> Vec<f64> {
    let n = data.len();
    if n < 2 {
        return data.to_vec();
    }
    let n_f = n as f64;
    let x_mean = (n_f - 1.0) / 2.0;
    let y_mean: f64 = data.iter().sum::<f64>() / n_f;

    let mut sxy = 0.0;
    let mut sxx = 0.0;
    for (i, &y) in data.iter().enumerate() {
        let dx = i as f64 - x_mean;
        sxy += dx * (y - y_mean);
        sxx += dx * dx;
    }
    let slope = if sxx.abs() > 1e-18 { sxy / sxx } else { 0.0 };
    let intercept = y_mean - slope * x_mean;

    data.iter()
        .enumerate()
        .map(|(i, &y)| y - (slope * i as f64 + intercept))
        .collect()
}

/// Root mean square of a slice.
fn rms(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let ms: f64 = data.iter().map(|x| x * x).sum::<f64>() / data.len() as f64;
    ms.sqrt()
}

/// Estimate vertical wavelength from oscillation in a profile quantity vs altitude.
fn estimate_vertical_wavelength(
    perturbation: &[f64],
    altitude_m: &[f64],
) -> Option<f64> {
    let n = perturbation.len().min(altitude_m.len());
    if n < 4 {
        return None;
    }
    // Count zero-crossings
    let mut crossings = Vec::new();
    for i in 1..n {
        if perturbation[i - 1] * perturbation[i] <= 0.0 {
            // Linear interpolation of crossing altitude
            let frac = if (perturbation[i] - perturbation[i - 1]).abs() > 1e-18 {
                perturbation[i - 1].abs()
                    / (perturbation[i - 1].abs() + perturbation[i].abs())
            } else {
                0.5
            };
            let alt = altitude_m[i - 1] + frac * (altitude_m[i] - altitude_m[i - 1]);
            crossings.push(alt);
        }
    }
    if crossings.len() < 2 {
        return None;
    }
    // Average half-wavelength spacing
    let mut sum = 0.0;
    for i in 1..crossings.len() {
        sum += (crossings[i] - crossings[i - 1]).abs();
    }
    let half_wl = sum / (crossings.len() - 1) as f64;
    Some(2.0 * half_wl)
}

/// Compute the atmospheric scale height H = RT/(Mg) at a given temperature.
pub fn scale_height(temperature_k: f64) -> f64 {
    R_UNIVERSAL * temperature_k / (M_AIR * G)
}

/// Compute air density from ideal gas law: ρ = p/(R·T).
pub fn air_density(pressure_hpa: f64, temperature_k: f64) -> f64 {
    (pressure_hpa * 100.0) / (R_AIR * temperature_k)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_profile() -> AtmosphericProfile {
        AtmosphericProfile {
            altitude_m: vec![0.0, 1000.0, 2000.0, 3000.0, 4000.0, 5000.0],
            temperature_k: vec![288.0, 282.0, 276.5, 271.0, 265.5, 260.0],
            pressure_hpa: vec![1013.25, 900.0, 800.0, 700.0, 620.0, 540.0],
            wind_speed_ms: vec![5.0, 8.0, 12.0, 15.0, 13.0, 10.0],
            wind_direction_deg: vec![270.0, 265.0, 260.0, 255.0, 250.0, 245.0],
        }
    }

    // -----------------------------------------------------------------------
    // AtmosphericProfile
    // -----------------------------------------------------------------------

    #[test]
    fn test_profile_len_and_is_empty() {
        let p = sample_profile();
        assert_eq!(p.len(), 6);
        assert!(!p.is_empty());
        let empty = AtmosphericProfile {
            altitude_m: vec![],
            temperature_k: vec![],
            pressure_hpa: vec![],
            wind_speed_ms: vec![],
            wind_direction_deg: vec![],
        };
        assert!(empty.is_empty());
    }

    #[test]
    fn test_mean_temperature() {
        let p = sample_profile();
        let m = p.mean_temperature();
        assert!(m > 260.0 && m < 290.0, "mean temp = {m}");
    }

    #[test]
    fn test_wind_components() {
        let p = sample_profile();
        let (u, v) = p.wind_components();
        assert_eq!(u.len(), p.len());
        // 270° wind → purely from west → u = +5 m/s, v ≈ 0
        assert!((u[0] - 5.0).abs() < 0.1, "u[0] = {}", u[0]);
        assert!(v[0].abs() < 0.1, "v[0] = {}", v[0]);
    }

    #[test]
    fn test_density_at() {
        let p = sample_profile();
        let rho = p.density_at(0);
        // Sea level density ~ 1.225 kg/m³
        assert!(rho > 1.1 && rho < 1.3, "rho = {rho}");
    }

    // -----------------------------------------------------------------------
    // Brunt-Väisälä
    // -----------------------------------------------------------------------

    #[test]
    fn test_bv_from_profile_lengths() {
        let p = sample_profile();
        let bv = BruntVaisala::from_profile(&p);
        assert_eq!(bv.n_squared.len(), p.len() - 1);
        assert_eq!(bv.altitudes_m.len(), p.len() - 1);
    }

    #[test]
    fn test_bv_stable_atmosphere() {
        // Lapse rate of -6 K/km is less steep than dry adiabatic (-9.8 K/km)
        // → N² > 0 (stable)
        let p = sample_profile();
        let bv = BruntVaisala::from_profile(&p);
        for &nsq in &bv.n_squared {
            assert!(nsq > 0.0, "expected stable, got N² = {nsq}");
        }
    }

    #[test]
    fn test_bv_unstable_atmosphere() {
        // Super-adiabatic lapse: temperature drops 15 K per 1 km
        let p = AtmosphericProfile {
            altitude_m: vec![0.0, 1000.0],
            temperature_k: vec![300.0, 285.0],
            pressure_hpa: vec![1013.25, 900.0],
            wind_speed_ms: vec![0.0, 0.0],
            wind_direction_deg: vec![0.0, 0.0],
        };
        let bv = BruntVaisala::from_profile(&p);
        assert_eq!(bv.classify(0), StabilityClass::Unstable);
    }

    #[test]
    fn test_bv_neutral_atmosphere() {
        // Lapse rate exactly = dry adiabatic → N² ≈ 0
        let p = AtmosphericProfile {
            altitude_m: vec![0.0, 1000.0],
            temperature_k: vec![300.0, 300.0 - GAMMA_D * 1000.0],
            pressure_hpa: vec![1013.25, 900.0],
            wind_speed_ms: vec![0.0, 0.0],
            wind_direction_deg: vec![0.0, 0.0],
        };
        let bv = BruntVaisala::from_profile(&p);
        assert_eq!(bv.classify(0), StabilityClass::Neutral);
    }

    #[test]
    fn test_bv_mean_n_squared_positive() {
        let bv = BruntVaisala::from_profile(&sample_profile());
        assert!(bv.mean_n_squared() > 0.0);
    }

    #[test]
    fn test_bv_mean_frequency() {
        let bv = BruntVaisala::from_profile(&sample_profile());
        let f = bv.mean_frequency().expect("should have mean freq");
        // Typical tropospheric N ~ 0.01 rad/s
        assert!(f > 0.005 && f < 0.03, "N = {f}");
    }

    #[test]
    fn test_bv_mean_period() {
        let bv = BruntVaisala::from_profile(&sample_profile());
        let t = bv.mean_period().expect("should have mean period");
        // BV period typically 5-15 minutes
        assert!(t > 200.0 && t < 1500.0, "period = {t} s");
    }

    // -----------------------------------------------------------------------
    // Dispersion relation
    // -----------------------------------------------------------------------

    #[test]
    fn test_dispersion_omega_positive() {
        let n_sq = 1e-4;
        let k_h = 2.0 * PI / 100_000.0; // 100 km wavelength
        let k_z = 2.0 * PI / 5_000.0; // 5 km vertical wavelength
        let omega = dispersion_omega(n_sq, k_h, k_z, DEFAULT_SCALE_HEIGHT_M);
        assert!(omega > 0.0);
        assert!(omega < n_sq.sqrt()); // ω must be < N
    }

    #[test]
    fn test_dispersion_omega_zero_for_unstable() {
        let omega = dispersion_omega(-1e-4, 0.01, 0.01, DEFAULT_SCALE_HEIGHT_M);
        assert_eq!(omega, 0.0);
    }

    #[test]
    fn test_dispersion_limits() {
        // As k_h → ∞, ω → N
        let n_sq = 1e-4;
        let k_h = 1.0; // very large
        let k_z = 0.0;
        let omega = dispersion_omega(n_sq, k_h, k_z, DEFAULT_SCALE_HEIGHT_M);
        assert!((omega - n_sq.sqrt()).abs() < 1e-3, "omega = {omega}");
    }

    // -----------------------------------------------------------------------
    // Wavenumber / frequency helpers
    // -----------------------------------------------------------------------

    #[test]
    fn test_wavenumber_from_wavelength() {
        let wl = 100_000.0; // 100 km
        let k = wavenumber_from_wavelength(wl);
        assert!((k - 2.0 * PI / wl).abs() < 1e-12);
    }

    #[test]
    fn test_wavenumber_zero_wavelength() {
        assert_eq!(wavenumber_from_wavelength(0.0), 0.0);
    }

    #[test]
    fn test_intrinsic_frequency() {
        let omega_i = intrinsic_frequency(0.01, 1e-5, 10.0);
        assert!((omega_i - (0.01 - 1e-5 * 10.0)).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // FIR bandpass
    // -----------------------------------------------------------------------

    #[test]
    fn test_bandpass_fir_design_length() {
        let h = design_bandpass_fir(63, 0.05, 0.15);
        assert_eq!(h.len(), 63);
    }

    #[test]
    fn test_fir_filter_preserves_length() {
        let signal = vec![1.0; 100];
        let coeffs = design_bandpass_fir(11, 0.1, 0.2);
        let out = fir_filter(&signal, &coeffs);
        assert_eq!(out.len(), signal.len());
    }

    // -----------------------------------------------------------------------
    // Morlet wavelet
    // -----------------------------------------------------------------------

    #[test]
    fn test_morlet_wavelet_at_zero() {
        let (re, im) = morlet_wavelet(0.0, 6.0);
        // At η=0: envelope = π^{-1/4}, cos(0)=1
        let expected_re = PI.powf(-0.25);
        assert!((re - expected_re).abs() < 1e-10);
        assert!(im.abs() < 1e-10); // sin(0) = 0
    }

    #[test]
    fn test_morlet_wavelet_decay() {
        let (re0, _) = morlet_wavelet(0.0, 6.0);
        let (re5, im5) = morlet_wavelet(5.0, 6.0);
        let amp5 = (re5 * re5 + im5 * im5).sqrt();
        // At η=5, Gaussian envelope is very small
        assert!(amp5 < re0.abs() * 0.01);
    }

    #[test]
    fn test_morlet_cwt_dimensions() {
        let signal = vec![0.0; 100];
        let scales = vec![1.0, 2.0, 4.0];
        let power = morlet_cwt(&signal, 1.0, &scales, 6.0);
        assert_eq!(power.len(), 3);
        assert_eq!(power[0].len(), 100);
    }

    #[test]
    fn test_scale_to_period() {
        let period = scale_to_period(10.0, 6.0);
        // Should be roughly proportional to scale
        assert!(period > 0.0);
        let period2 = scale_to_period(20.0, 6.0);
        assert!(period2 > period);
    }

    // -----------------------------------------------------------------------
    // Hodograph
    // -----------------------------------------------------------------------

    #[test]
    fn test_hodograph_circular() {
        // Perfect circle → axial ratio ≈ 1
        let n = 100;
        let u: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).cos())
            .collect();
        let v: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let ell = fit_hodograph_ellipse(&u, &v).unwrap();
        assert!(
            (ell.axial_ratio - 1.0).abs() < 0.05,
            "axial_ratio = {}",
            ell.axial_ratio
        );
    }

    #[test]
    fn test_hodograph_linear() {
        // Pure line → axial ratio ≈ 0
        let u: Vec<f64> = (0..50).map(|i| (i as f64 * 0.1).sin()).collect();
        let v = vec![0.0; 50];
        let ell = fit_hodograph_ellipse(&u, &v).unwrap();
        assert!(ell.axial_ratio < 0.05, "axial_ratio = {}", ell.axial_ratio);
    }

    #[test]
    fn test_hodograph_too_short() {
        assert!(fit_hodograph_ellipse(&[1.0, 2.0], &[3.0, 4.0]).is_none());
    }

    #[test]
    fn test_hodograph_rotation_sense() {
        let n = 100;
        // CW rotation (anticyclonic in NH)
        let u_cw: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).cos())
            .collect();
        let v_cw: Vec<f64> = (0..n)
            .map(|i| -(2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let ell = fit_hodograph_ellipse(&u_cw, &v_cw).unwrap();
        assert_eq!(ell.rotation_sense, -1); // negative cross-product
    }

    // -----------------------------------------------------------------------
    // Energy diagnostics
    // -----------------------------------------------------------------------

    #[test]
    fn test_potential_energy_density() {
        let ep = potential_energy_density(1.0, 250.0, 1e-4);
        // Ep = 0.5 * (9.81²/1e-4) * (1/250)² ≈ 7.7 J/kg
        assert!(ep > 5.0 && ep < 12.0, "Ep = {ep}");
    }

    #[test]
    fn test_potential_energy_unstable() {
        assert_eq!(potential_energy_density(1.0, 250.0, -1e-4), 0.0);
    }

    #[test]
    fn test_kinetic_energy_density() {
        let ek = kinetic_energy_density(3.0, 4.0, 0.0);
        assert!((ek - 12.5).abs() < 1e-10);
    }

    #[test]
    fn test_momentum_flux() {
        let u = vec![1.0, -1.0, 1.0, -1.0];
        let w = vec![0.5, -0.5, 0.5, -0.5];
        let mf = momentum_flux(1.2, &u, &w);
        // ⟨u'w'⟩ = mean(0.5, 0.5, 0.5, 0.5) = 0.5; F = 1.2 * 0.5 = 0.6
        assert!((mf - 0.6).abs() < 1e-10, "mf = {mf}");
    }

    #[test]
    fn test_momentum_flux_empty() {
        assert_eq!(momentum_flux(1.2, &[], &[]), 0.0);
    }

    // -----------------------------------------------------------------------
    // Stokes parameters
    // -----------------------------------------------------------------------

    #[test]
    fn test_stokes_circular_polarisation() {
        let n = 200;
        let u: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / 50.0).cos())
            .collect();
        let v: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / 50.0).sin())
            .collect();
        let s = stokes_parameters(&u, &v).unwrap();
        // S0 (total intensity) should be positive for a non-zero signal
        assert!(s.s0 > 0.0, "S0 = {}", s.s0);
        // S1 ≈ 0 for equal-power circular polarisation (Puu ≈ Pvv)
        assert!(s.s1.abs() < 0.1 * s.s0, "S1/S0 = {}", s.s1 / s.s0);
        // S3 should be nonzero (quadrature component indicates circularity)
        assert!(s.s3.abs() > 0.0, "S3 = {}", s.s3);
        // Degree of polarisation should be positive
        assert!(s.degree_of_polarisation > 0.0, "dop = {}", s.degree_of_polarisation);
    }

    #[test]
    fn test_stokes_too_short() {
        assert!(stokes_parameters(&[1.0, 2.0], &[3.0, 4.0]).is_none());
    }

    // -----------------------------------------------------------------------
    // Source classification
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_mountain() {
        assert_eq!(
            classify_source(0.0, 0.1, 0.5, 0.9),
            WaveSource::Mountain
        );
    }

    #[test]
    fn test_classify_convective() {
        assert_eq!(
            classify_source(0.01, 0.8, 0.5, 0.3),
            WaveSource::Convective
        );
    }

    #[test]
    fn test_classify_jet_stream() {
        assert_eq!(
            classify_source(0.005, 0.2, 0.1, 0.5),
            WaveSource::JetStream
        );
    }

    #[test]
    fn test_classify_unknown() {
        assert_eq!(
            classify_source(0.005, 0.3, 0.5, 0.5),
            WaveSource::Unknown
        );
    }

    // -----------------------------------------------------------------------
    // GravityWaveDetector
    // -----------------------------------------------------------------------

    #[test]
    fn test_detector_default() {
        let det = GravityWaveDetector::new();
        assert_eq!(det.min_period_s, 300.0);
        assert_eq!(det.max_period_s, 10800.0);
    }

    #[test]
    fn test_detector_custom_period_range() {
        let det = GravityWaveDetector::with_period_range(600.0, 7200.0);
        assert_eq!(det.min_period_s, 600.0);
        assert_eq!(det.max_period_s, 7200.0);
    }

    #[test]
    fn test_detector_bandpass_preserves_length() {
        let det = GravityWaveDetector::new();
        let signal: Vec<f64> = (0..500)
            .map(|i| (2.0 * PI * i as f64 / 100.0).sin())
            .collect();
        let filtered = det.bandpass_filter(&signal, 60.0); // 1-minute sampling
        assert_eq!(filtered.len(), signal.len());
    }

    #[test]
    fn test_detect_from_pressure_synthetic() {
        let det = GravityWaveDetector {
            min_period_s: 100.0,
            max_period_s: 500.0,
            detection_threshold: 0.05,
            fir_order: 31,
            ..Default::default()
        };
        // Synthetic pressure series with a 200-second oscillation
        let dt = 10.0; // 10-second samples
        let n_samples = 400;
        let signal: Vec<f64> = (0..n_samples)
            .map(|i| {
                1013.25 + 0.5 * (2.0 * PI * i as f64 * dt / 200.0).sin()
            })
            .collect();
        let events = det.detect_from_pressure(&signal, dt, 250.0, 1e-4, 1.2);
        // Should detect at least one event
        assert!(
            !events.is_empty(),
            "expected to detect synthetic wave, got 0 events"
        );
    }

    #[test]
    fn test_detect_from_pressure_empty() {
        let det = GravityWaveDetector::new();
        let events = det.detect_from_pressure(&[], 60.0, 250.0, 1e-4, 1.2);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_from_profile() {
        let det = GravityWaveDetector::new();
        // Build an extended profile with oscillations in wind
        let n = 50;
        let mut p = AtmosphericProfile {
            altitude_m: Vec::with_capacity(n),
            temperature_k: Vec::with_capacity(n),
            pressure_hpa: Vec::with_capacity(n),
            wind_speed_ms: Vec::with_capacity(n),
            wind_direction_deg: Vec::with_capacity(n),
        };
        for i in 0..n {
            let alt = i as f64 * 500.0;
            p.altitude_m.push(alt);
            p.temperature_k.push(288.0 - 6.0 * alt / 1000.0);
            p.pressure_hpa.push(1013.25 * (-alt / 8500.0_f64).exp());
            // Sinusoidal wind perturbation
            let ws = 10.0 + 3.0 * (2.0 * PI * alt / 5000.0).sin();
            p.wind_speed_ms.push(ws);
            p.wind_direction_deg.push(270.0 + 10.0 * (2.0 * PI * alt / 5000.0).cos());
        }
        let events = det.detect_from_profile(&p);
        assert!(!events.is_empty(), "expected profile-based detections");
    }

    // -----------------------------------------------------------------------
    // Utility functions
    // -----------------------------------------------------------------------

    #[test]
    fn test_remove_linear_trend() {
        let data: Vec<f64> = (0..10).map(|i| 2.0 * i as f64 + 5.0).collect();
        let detrended = remove_linear_trend(&data);
        let max_residual = detrended.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_residual < 1e-10,
            "max residual = {max_residual}"
        );
    }

    #[test]
    fn test_rms() {
        let data = vec![3.0, 4.0];
        let r = rms(&data);
        // RMS = sqrt((9+16)/2) = sqrt(12.5) ≈ 3.536
        assert!((r - 12.5_f64.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_scale_height_typical() {
        let h = scale_height(250.0);
        // H ≈ 7.3 km
        assert!(h > 7000.0 && h < 8000.0, "H = {h}");
    }

    #[test]
    fn test_air_density_sea_level() {
        let rho = air_density(1013.25, 288.15);
        assert!((rho - 1.225).abs() < 0.01, "rho = {rho}");
    }

    #[test]
    fn test_estimate_vertical_wavelength() {
        // Sinusoidal perturbation with 4 km wavelength
        let n = 100;
        let alt: Vec<f64> = (0..n).map(|i| i as f64 * 200.0).collect(); // 0-20 km
        let pert: Vec<f64> = alt
            .iter()
            .map(|z| (2.0 * PI * z / 4000.0).sin())
            .collect();
        let wl = estimate_vertical_wavelength(&pert, &alt);
        assert!(wl.is_some());
        let wl = wl.unwrap();
        assert!(
            (wl - 4000.0).abs() < 600.0,
            "vertical wavelength = {wl}, expected ~4000"
        );
    }
}
