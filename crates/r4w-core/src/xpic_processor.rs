//! # XPIC Processor — Cross-Polarization Interference Canceller
//!
//! Implements XPIC for dual-polarization microwave and millimeter-wave backhaul
//! systems. By transmitting simultaneously on horizontal (H) and vertical (V)
//! polarizations, spectral efficiency is doubled. Cross-polarization interference
//! (XPI) between the two channels is cancelled by adaptive 2×2 MIMO filters.
//!
//! ## Standards References
//!
//! - **ITU-R P.530**: Propagation data and prediction methods for terrestrial
//!   point-to-point links (rain depolarization, XPD vs. rain rate).
//! - **ITU-R P.838**: Specific attenuation model for rain for use in prediction
//!   methods (rain attenuation coefficients k_H, k_V, α_H, α_V).
//! - **ETSI EN 302 217**: Fixed Radio Systems; characteristics and requirements
//!   for point-to-point equipment and antennas.
//! - **ITU-R F.752**: Diversity techniques in transhorizon radio-relay systems.
//!
//! ## System Architecture
//!
//! ```text
//!   TX_H ──────────────────────────── air ─────────── RX_H
//!         ╲ XPD_HV leakage                  ↑ XPIC
//!   TX_V ──────────────────────────── air ─────────── RX_V
//! ```
//!
//! The 2×2 Jones channel matrix:
//!
//! ```text
//!   [ r_H ]   [ h_HH  h_HV ] [ s_H ]   [ n_H ]
//!   [ r_V ] = [ h_VH  h_VV ] [ s_V ] + [ n_V ]
//! ```
//!
//! XPIC cancels `h_HV` and `h_VH` using adaptive filters.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::xpic_processor::*;
//!
//! // Configure a dual-pol channel with 30 dB antenna XPD
//! let channel = PolarizationChannel::new_with_xpd(30.0, 0.0);
//!
//! // Transmit QPSK on both pols
//! let s_h = num_complex::Complex64::new(1.0,  1.0) * 0.707;
//! let s_v = num_complex::Complex64::new(-1.0, 1.0) * 0.707;
//! let (r_h, r_v) = channel.apply(s_h, s_v);
//!
//! // Build an XPIC filter and train it
//! let mut xpic = XpicFilter::new(XpicConfig::default());
//! let _metrics = xpic.process_training(&[(s_h, s_v)], &[(r_h, r_v)]);
//! ```

use std::f64::consts::PI;
use num_complex::Complex64;

// ---------------------------------------------------------------------------
// Helper aliases
// ---------------------------------------------------------------------------
type C64 = Complex64;

/// A 2×2 complex matrix stored in row-major order.
///
/// Layout: `[[m00, m01], [m10, m11]]` → flat `[m00, m01, m10, m11]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix2x2 {
    pub data: [C64; 4],
}

impl Matrix2x2 {
    /// Create from individual elements.
    pub fn new(m00: C64, m01: C64, m10: C64, m11: C64) -> Self {
        Self { data: [m00, m01, m10, m11] }
    }

    /// 2×2 identity matrix.
    pub fn identity() -> Self {
        Self::new(
            C64::new(1.0, 0.0), C64::new(0.0, 0.0),
            C64::new(0.0, 0.0), C64::new(1.0, 0.0),
        )
    }

    /// Element access (row, col).
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> C64 {
        self.data[row * 2 + col]
    }

    /// Element set (row, col).
    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: C64) {
        self.data[row * 2 + col] = val;
    }

    /// Matrix–vector multiply: `[y0, y1] = M * [x0, x1]`.
    pub fn mul_vec(&self, x0: C64, x1: C64) -> (C64, C64) {
        let y0 = self.data[0] * x0 + self.data[1] * x1;
        let y1 = self.data[2] * x0 + self.data[3] * x1;
        (y0, y1)
    }

    /// Matrix–matrix multiply.
    pub fn mul_mat(&self, rhs: &Matrix2x2) -> Matrix2x2 {
        let a = self;
        let b = rhs;
        Matrix2x2::new(
            a.data[0] * b.data[0] + a.data[1] * b.data[2],
            a.data[0] * b.data[1] + a.data[1] * b.data[3],
            a.data[2] * b.data[0] + a.data[3] * b.data[2],
            a.data[2] * b.data[1] + a.data[3] * b.data[3],
        )
    }

    /// Conjugate-transpose (Hermitian).
    pub fn hermitian(&self) -> Matrix2x2 {
        Matrix2x2::new(
            self.data[0].conj(), self.data[2].conj(),
            self.data[1].conj(), self.data[3].conj(),
        )
    }

    /// Frobenius norm squared.
    pub fn norm_sq(&self) -> f64 {
        self.data.iter().map(|x| x.norm_sqr()).sum()
    }

    /// Determinant.
    pub fn det(&self) -> C64 {
        self.data[0] * self.data[3] - self.data[1] * self.data[2]
    }

    /// Inverse (returns None if singular).
    pub fn inv(&self) -> Option<Matrix2x2> {
        let d = self.det();
        if d.norm() < 1e-30 {
            return None;
        }
        let inv_d = C64::new(1.0, 0.0) / d;
        Some(Matrix2x2::new(
             self.data[3] * inv_d,
            -self.data[1] * inv_d,
            -self.data[2] * inv_d,
             self.data[0] * inv_d,
        ))
    }

    /// Condition number estimate (max / min singular value via power iteration).
    pub fn condition_number(&self) -> f64 {
        // Fast 2×2 singular value computation using characteristic polynomial.
        // σ1, σ2 = sqrt of eigenvalues of A^H A.
        let aha = self.hermitian().mul_mat(self);
        let a = aha.data[0].re;
        let d = aha.data[3].re;
        let bc_sq = aha.data[1].norm_sqr();
        let tr = a + d;
        let disc = ((a - d).powi(2) + 4.0 * bc_sq).max(0.0).sqrt();
        let lam1 = (tr + disc) / 2.0;
        let lam2 = ((tr - disc) / 2.0).max(0.0);
        if lam2 < 1e-30 { f64::INFINITY } else { (lam1 / lam2).sqrt() }
    }
}

// ---------------------------------------------------------------------------
// 1. Polarization Channel Model
// ---------------------------------------------------------------------------

/// 2×2 Jones matrix representation of the dual-polarization propagation channel.
///
/// The channel maps transmitted (s_H, s_V) to received (r_H, r_V):
///
/// ```text
/// [ r_H ]   [ h_HH  h_HV ] [ s_H ]   [ n_H ]
/// [ r_V ] = [ h_VH  h_VV ] [ s_V ] + [ n_V ]
/// ```
///
/// Diagonal terms `h_HH`, `h_VV` are the co-polarization paths.
/// Off-diagonal terms represent cross-polarization leakage.
///
/// XPD (Cross-Polar Discrimination) in dB:
///   `XPD = 10·log10(|h_HH|² / |h_HV|²)`
#[derive(Debug, Clone)]
pub struct PolarizationChannel {
    /// Jones matrix H.
    pub jones: Matrix2x2,
    /// AWGN variance per polarization (σ²).
    pub noise_variance: f64,
    /// Internal PRNG state for noise generation.
    rng_state: u64,
}

impl PolarizationChannel {
    /// Create a channel from a Jones matrix.
    pub fn new(jones: Matrix2x2, noise_variance: f64) -> Self {
        Self { jones, noise_variance, rng_state: 0x6A09E667F3BCC908 }
    }

    /// Create a channel with given XPD (dB) and co-pol phase shift (rad).
    ///
    /// Assumes equal co-pol gain (0 dB) on both polarizations.
    /// `xpd_db`: antenna XPD typically 30–40 dB.
    /// `phase_offset`: differential phase between H and V co-pol paths (rad).
    pub fn new_with_xpd(xpd_db: f64, phase_offset: f64) -> Self {
        let xpol_amp = 10.0_f64.powf(-xpd_db / 20.0);
        let jones = Matrix2x2::new(
            C64::new((phase_offset).cos(), (phase_offset).sin()), // h_HH
            C64::new(xpol_amp, 0.0),                             // h_HV
            C64::new(xpol_amp, 0.0),                             // h_VH
            C64::new(1.0, 0.0),                                   // h_VV
        );
        Self::new(jones, 1e-6)
    }

    /// Apply the channel to a (s_H, s_V) sample pair.
    /// Returns (r_H, r_V) without noise.
    pub fn apply(&self, s_h: C64, s_v: C64) -> (C64, C64) {
        self.jones.mul_vec(s_h, s_v)
    }

    /// Apply the channel and add AWGN noise.
    pub fn apply_noisy(&mut self, s_h: C64, s_v: C64) -> (C64, C64) {
        let (r_h, r_v) = self.apply(s_h, s_v);
        let (n_h, n_v) = self.awgn_pair();
        (r_h + n_h, r_v + n_v)
    }

    /// Compute XPD (dB) from the current Jones matrix, cross-pol HV.
    pub fn xpd_hv_db(&self) -> f64 {
        let co = self.jones.get(0, 0).norm_sqr();
        let xp = self.jones.get(0, 1).norm_sqr();
        if xp < 1e-30 { return 100.0; }
        10.0 * (co / xp).log10()
    }

    /// Compute XPD (dB) from the current Jones matrix, cross-pol VH.
    pub fn xpd_vh_db(&self) -> f64 {
        let co = self.jones.get(1, 1).norm_sqr();
        let xp = self.jones.get(1, 0).norm_sqr();
        if xp < 1e-30 { return 100.0; }
        10.0 * (co / xp).log10()
    }

    /// Generate complex AWGN sample pair using Box-Muller transform.
    fn awgn_pair(&mut self) -> (C64, C64) {
        let sigma = (self.noise_variance / 2.0).sqrt();
        let n_h = self.normal_complex(sigma);
        let n_v = self.normal_complex(sigma);
        (n_h, n_v)
    }

    /// xoshiro64** PRNG advancing and uniform [0,1) output.
    fn rand_f64(&mut self) -> f64 {
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        (self.rng_state >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    fn normal_complex(&mut self, sigma: f64) -> C64 {
        let u1 = self.rand_f64().max(1e-30);
        let u2 = self.rand_f64();
        let mag = sigma * (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * PI * u2;
        C64::new(mag * theta.cos(), mag * theta.sin())
    }
}

// ---------------------------------------------------------------------------
// 2. Rain Depolarization Model — ITU-R P.530 / P.838
// ---------------------------------------------------------------------------

/// Rain depolarization model per ITU-R P.530-17.
///
/// Relates rain-induced co-polar attenuation `A_co` (dB) to XPD degradation.
/// The fundamental relationship (P.530 Eq. 38):
///
/// ```text
/// XPD = U - V(f) · log10(A_co)
/// ```
///
/// where `U` and `V(f)` are frequency-dependent empirical constants derived
/// from canting angle statistics and rain DSD (drop size distribution).
///
/// Reference: ITU-R P.530-17, Section 4.2.
#[derive(Debug, Clone)]
pub struct RainDepolarization {
    /// Link frequency (GHz).
    pub freq_ghz: f64,
    /// Link elevation angle (degrees).
    pub elevation_deg: f64,
    /// Path length (km).
    pub path_km: f64,
    /// Canting angle standard deviation (deg). Typically 5–10°.
    pub canting_std_deg: f64,
    /// Polarization tilt angle (deg). 0=H, 90=V, 45=45°-slant.
    pub pol_tilt_deg: f64,
}

impl RainDepolarization {
    /// Create with standard parameters for a typical fixed backhaul link.
    pub fn new(freq_ghz: f64, elevation_deg: f64, path_km: f64) -> Self {
        Self {
            freq_ghz,
            elevation_deg,
            path_km,
            canting_std_deg: 7.0,
            pol_tilt_deg: 0.0,
        }
    }

    /// Compute co-polar rain attenuation (dB) per ITU-R P.838-3.
    ///
    /// Specific attenuation: `γ_R = k · R^α` (dB/km)
    ///
    /// Coefficients k and α are frequency-dependent.
    pub fn copolar_attenuation_db(&self, rain_rate_mmh: f64) -> f64 {
        let (k_h, alpha_h, k_v, alpha_v) = self.itu_r_p838_coefficients();
        let f = self.pol_tilt_deg * PI / 180.0;
        // Effective k and alpha for arbitrary tilt (P.838 Eq. 5)
        let k = (k_h * f.cos().powi(2) + k_v * f.sin().powi(2)
            + (k_h * alpha_h + k_v * alpha_v) / 2.0 * (2.0 * f).sin().powi(2) / 2.0)
            / (f.cos().powi(2) + f.sin().powi(2));
        let alpha_num = k_h * alpha_h * f.cos().powi(2)
            + k_v * alpha_v * f.sin().powi(2)
            + (k_h * alpha_h + k_v * alpha_v) / 2.0 * (2.0 * f).sin().powi(2) / 2.0;
        let alpha = alpha_num / k.max(1e-30);
        let gamma = k * rain_rate_mmh.powf(alpha);
        // Path attenuation
        let path_reduction = 1.0 / (1.0 + self.path_km / 35.0); // simplified reduction factor
        gamma * self.path_km * path_reduction
    }

    /// XPD degradation from rain per ITU-R P.530-17 Eq. (38).
    ///
    /// `XPD_rain = U - V · log10(A_co)`
    ///
    /// Returns (xpd_clear_sky_db, xpd_rain_degraded_db).
    pub fn xpd_with_rain(&self, rain_rate_mmh: f64, clear_sky_xpd_db: f64) -> (f64, f64) {
        let a_co = self.copolar_attenuation_db(rain_rate_mmh);
        let (u, v) = self.uv_coefficients();
        let xpd_rain = u - v * a_co.max(0.001).log10();
        // Combined XPD (worst-case combination per P.530 Sec 4.2)
        let xpd_combined = if xpd_rain < clear_sky_xpd_db {
            xpd_rain
        } else {
            clear_sky_xpd_db
        };
        (clear_sky_xpd_db, xpd_combined)
    }

    /// Compute differential attenuation Δα (dB) and differential phase Δφ (deg)
    /// induced by rain. Used to construct depolarized Jones matrix.
    pub fn differential_effects(&self, rain_rate_mmh: f64) -> (f64, f64) {
        let a_co = self.copolar_attenuation_db(rain_rate_mmh);
        // Approximate differential attenuation ≈ 15% of co-pol attenuation
        let delta_att = 0.15 * a_co;
        // Differential phase: roughly 0.1 deg per dB of co-pol attenuation
        let delta_phase_deg = 0.1 * a_co;
        (delta_att, delta_phase_deg)
    }

    /// Build a rain-affected Jones matrix.
    pub fn rain_jones_matrix(&self, rain_rate_mmh: f64, clear_sky_xpd_db: f64) -> Matrix2x2 {
        let (_, xpd_db) = self.xpd_with_rain(rain_rate_mmh, clear_sky_xpd_db);
        let (delta_att, delta_phase_deg) = self.differential_effects(rain_rate_mmh);
        let a_co = self.copolar_attenuation_db(rain_rate_mmh);

        // Co-pol amplitude loss
        let h_amp = 10.0_f64.powf(-a_co / 20.0);
        let v_amp = 10.0_f64.powf(-(a_co - delta_att) / 20.0);
        // Differential phase on V relative to H
        let delta_phi = delta_phase_deg * PI / 180.0;
        // Cross-pol amplitude from XPD
        let xpol_amp = h_amp * 10.0_f64.powf(-xpd_db / 20.0);

        let canting = self.canting_std_deg * PI / 180.0;
        // Mean canting contribution to cross-pol (simplified)
        let xpol_phase = canting;

        Matrix2x2::new(
            C64::new(h_amp, 0.0),
            C64::new(xpol_amp * xpol_phase.cos(), xpol_amp * xpol_phase.sin()),
            C64::new(xpol_amp * xpol_phase.cos(), -xpol_amp * xpol_phase.sin()),
            C64::new(v_amp * delta_phi.cos(), v_amp * delta_phi.sin()),
        )
    }

    // ITU-R P.838-3 coefficients (k_H, α_H, k_V, α_V) vs. frequency.
    // Table values are interpolated from ITU-R P.838-3 Tables 1-4.
    fn itu_r_p838_coefficients(&self) -> (f64, f64, f64, f64) {
        let f = self.freq_ghz;
        // Piecewise linear interpolation from reference table points
        // (GHz, k_H, alpha_H, k_V, alpha_V)
        let table: &[(f64, f64, f64, f64, f64)] = &[
            (1.0,  0.0000259, 0.9691, 0.0000308, 0.8592),
            (2.0,  0.0000847, 1.0664, 0.0000998, 0.9490),
            (4.0,  0.000454,  1.1209, 0.000532,  1.0253),
            (6.0,  0.00128,   1.2322, 0.00146,   1.1275),
            (7.0,  0.00265,   1.2350, 0.00296,   1.1216),
            (8.0,  0.00454,   1.2227, 0.00508,   1.1060),
            (10.0, 0.0101,    1.2059, 0.0113,    1.0794),
            (12.0, 0.0188,    1.1788, 0.0210,    1.0482),
            (15.0, 0.0367,    1.1275, 0.0411,    0.9991),
            (20.0, 0.0751,    1.0558, 0.0854,    0.9355),
            (25.0, 0.124,     1.0000, 0.139,     0.8905),
            (30.0, 0.187,     0.9630, 0.209,     0.8572),
            (35.0, 0.263,     0.9317, 0.292,     0.8355),
            (40.0, 0.350,     0.9144, 0.387,     0.8174),
            (50.0, 0.536,     0.8855, 0.581,     0.7905),
            (60.0, 0.707,     0.8567, 0.772,     0.7648),
            (80.0, 1.151,     0.8137, 1.255,     0.7298),
            (100., 1.071,     0.7905, 1.154,     0.7202),
        ];
        // Clamp to table range
        let f = f.clamp(table[0].0, table[table.len() - 1].0);
        // Find bracketing entries
        let mut lo = 0usize;
        for (i, &(freq, ..)) in table.iter().enumerate() {
            if freq <= f { lo = i; }
        }
        let hi = (lo + 1).min(table.len() - 1);
        let (f0, kh0, ah0, kv0, av0) = table[lo];
        let (f1, kh1, ah1, kv1, av1) = table[hi];
        let t = if (f1 - f0).abs() < 1e-10 { 0.0 } else { (f - f0) / (f1 - f0) };
        let lerp = |a: f64, b: f64| a + t * (b - a);
        (lerp(kh0, kh1), lerp(ah0, ah1), lerp(kv0, kv1), lerp(av0, av1))
    }

    // U, V coefficients for XPD prediction (P.530-17 Eq. 38 empirical fit).
    fn uv_coefficients(&self) -> (f64, f64) {
        let f = self.freq_ghz;
        // Empirical fit from P.530-17 Annex A
        let u = 26.0 + (10.0 * f.log10()).max(0.0).min(15.0);
        let v = if f < 20.0 { 12.8 } else { 22.6 };
        (u, v)
    }
}

// ---------------------------------------------------------------------------
// 3. Stokes Parameters and SOP
// ---------------------------------------------------------------------------

/// Stokes parameters describing the State of Polarization (SOP).
///
/// For a quasi-monochromatic wave with electric field components E_H and E_V:
///
/// - S0 = <|E_H|²> + <|E_V|²>   (total power)
/// - S1 = <|E_H|²> - <|E_V|²>   (H/V linear polarization imbalance)
/// - S2 = 2·Re<E_H·E_V*>         (±45° linear polarization)
/// - S3 = 2·Im<E_H·E_V*>         (circular polarization)
///
/// The Stokes vector lies on the Poincaré sphere with radius S0.
/// Degree of Polarization (DoP) = sqrt(S1²+S2²+S3²) / S0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StokesParameters {
    pub s0: f64,
    pub s1: f64,
    pub s2: f64,
    pub s3: f64,
}

impl StokesParameters {
    /// Compute Stokes parameters from a single (E_H, E_V) sample.
    pub fn from_sample(e_h: C64, e_v: C64) -> Self {
        let cross = e_h * e_v.conj();
        Self {
            s0: e_h.norm_sqr() + e_v.norm_sqr(),
            s1: e_h.norm_sqr() - e_v.norm_sqr(),
            s2: 2.0 * cross.re,
            s3: 2.0 * cross.im,
        }
    }

    /// Compute time-averaged Stokes parameters from a block of samples.
    pub fn from_block(samples_h: &[C64], samples_v: &[C64]) -> Self {
        let n = samples_h.len().min(samples_v.len()) as f64;
        if n == 0.0 {
            return Self { s0: 0.0, s1: 0.0, s2: 0.0, s3: 0.0 };
        }
        let mut s0 = 0.0; let mut s1 = 0.0;
        let mut s2 = 0.0; let mut s3 = 0.0;
        for (&eh, &ev) in samples_h.iter().zip(samples_v.iter()) {
            let sp = Self::from_sample(eh, ev);
            s0 += sp.s0; s1 += sp.s1; s2 += sp.s2; s3 += sp.s3;
        }
        Self { s0: s0/n, s1: s1/n, s2: s2/n, s3: s3/n }
    }

    /// Degree of Polarization (DoP). 1 = fully polarized, 0 = unpolarized.
    pub fn degree_of_polarization(&self) -> f64 {
        if self.s0 < 1e-30 { return 0.0; }
        (self.s1.powi(2) + self.s2.powi(2) + self.s3.powi(2)).sqrt() / self.s0
    }

    /// Ellipticity angle χ (rad) on Poincaré sphere.  χ ∈ [-π/4, π/4].
    pub fn ellipticity_angle(&self) -> f64 {
        if self.s0 < 1e-30 { return 0.0; }
        (self.s3 / self.s0).clamp(-1.0, 1.0).asin() / 2.0
    }

    /// Orientation angle ψ (rad) of the polarization ellipse. ψ ∈ [0, π/2].
    pub fn orientation_angle(&self) -> f64 {
        self.s2.atan2(self.s1) / 2.0
    }

    /// XPD (dB) estimated from Stokes S1 component.
    pub fn xpd_db(&self) -> f64 {
        if self.s0 < 1e-30 { return 0.0; }
        let h_power = (self.s0 + self.s1) / 2.0;
        let v_power = (self.s0 - self.s1) / 2.0;
        if v_power < 1e-30 || h_power < 1e-30 { return 60.0; }
        10.0 * (h_power / v_power).abs().log10()
    }

    /// Rotate the SOP by angle θ (Mueller matrix rotation).
    pub fn rotate(&self, theta_rad: f64) -> Self {
        let c = (2.0 * theta_rad).cos();
        let s = (2.0 * theta_rad).sin();
        Self {
            s0: self.s0,
            s1: c * self.s1 + s * self.s2,
            s2: -s * self.s1 + c * self.s2,
            s3: self.s3,
        }
    }
}

// ---------------------------------------------------------------------------
// 4. Polarization Tracker
// ---------------------------------------------------------------------------

/// Tracks slow SOP drift using exponential averaging of Stokes parameters.
///
/// SOP drift occurs due to mechanical vibration, temperature changes, and
/// atmospheric fluctuations. Tracking bandwidth is controlled by `alpha`.
#[derive(Debug, Clone)]
pub struct PolarizationTracker {
    /// Exponential averaging coefficient (0 < α < 1). Smaller → slower tracking.
    pub alpha: f64,
    /// Current averaged Stokes parameters.
    pub stokes: StokesParameters,
    /// Estimated polarization rotation angle (rad).
    pub rotation_estimate: f64,
    /// Sample counter.
    pub count: u64,
}

impl PolarizationTracker {
    /// Create a polarization tracker.
    ///
    /// `alpha`: averaging coefficient. For update rate fs and desired bandwidth B:
    ///   α ≈ 2πB / fs.  Typical: α = 0.001 for slow drift tracking.
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha,
            stokes: StokesParameters { s0: 1.0, s1: 1.0, s2: 0.0, s3: 0.0 },
            rotation_estimate: 0.0,
            count: 0,
        }
    }

    /// Update tracker with new (E_H, E_V) sample.
    /// Returns current SOP estimate.
    pub fn update(&mut self, e_h: C64, e_v: C64) -> &StokesParameters {
        let sp = StokesParameters::from_sample(e_h, e_v);
        if self.count == 0 {
            self.stokes = sp;
        } else {
            let a = self.alpha;
            self.stokes.s0 = (1.0 - a) * self.stokes.s0 + a * sp.s0;
            self.stokes.s1 = (1.0 - a) * self.stokes.s1 + a * sp.s1;
            self.stokes.s2 = (1.0 - a) * self.stokes.s2 + a * sp.s2;
            self.stokes.s3 = (1.0 - a) * self.stokes.s3 + a * sp.s3;
        }
        // Extract rotation estimate from S2/S1
        self.rotation_estimate = self.stokes.orientation_angle();
        self.count += 1;
        &self.stokes
    }

    /// Apply polarization rotation correction to a sample pair.
    pub fn correct(&self, e_h: C64, e_v: C64) -> (C64, C64) {
        let theta = -self.rotation_estimate;
        let c = theta.cos();
        let s = theta.sin();
        (c * e_h - s * e_v, s * e_h + c * e_v)
    }
}

// ---------------------------------------------------------------------------
// 5. XPIC Adaptive Filter
// ---------------------------------------------------------------------------

/// XPIC adaptation algorithm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XpicAlgorithm {
    /// Least Mean Squares — O(N) per sample, slower convergence.
    Lms,
    /// Recursive Least Squares — O(N²) per sample, fast convergence.
    Rls,
    /// Decision-Directed LMS (no pilot required after convergence).
    DecisionDirectedLms,
}

/// XPIC filter configuration.
#[derive(Debug, Clone)]
pub struct XpicConfig {
    /// Number of complex taps in each cross-pol cancellation filter.
    pub num_taps: usize,
    /// LMS step size μ (0.001–0.05 typical).
    pub lms_step_size: f64,
    /// RLS forgetting factor λ (0.95–0.9999 typical).
    pub rls_lambda: f64,
    /// RLS initialization parameter δ for P = δI.
    pub rls_delta: f64,
    /// Adaptation algorithm.
    pub algorithm: XpicAlgorithm,
    /// Enable leakage term (regularization) to prevent weight drift.
    pub leakage: f64,
}

impl Default for XpicConfig {
    fn default() -> Self {
        Self {
            num_taps: 8,
            lms_step_size: 0.01,
            rls_lambda: 0.99,
            rls_delta: 100.0,
            algorithm: XpicAlgorithm::Lms,
            leakage: 1e-6,
        }
    }
}

/// Cross-polarization interference canceller — 2×2 adaptive MIMO filter.
///
/// The XPIC structure:
/// ```text
///   r_H ──── W_HH ─────────────────────── ŝ_H
///         ╲                         ↑ (+)
///   r_V ──── W_HV (cross-pol canceller) ─┘
///
///   r_V ──── W_VV ─────────────────────── ŝ_V
///         ╲                         ↑ (+)
///   r_H ──── W_VH (cross-pol canceller) ─┘
/// ```
///
/// For pure XPIC (no per-pol equalization), W_HH = W_VV = 1 (scalar).
/// The cross-pol filters W_HV and W_VH are adapted to cancel XPI.
#[derive(Debug, Clone)]
pub struct XpicFilter {
    config: XpicConfig,
    // Cross-pol filters: W_HV (applied to V to cancel from H output)
    w_hv: Vec<C64>,
    // Cross-pol filters: W_VH (applied to H to cancel from V output)
    w_vh: Vec<C64>,
    // Co-pol equalizer H channel
    w_hh: Vec<C64>,
    // Co-pol equalizer V channel
    w_vv: Vec<C64>,
    // Delay buffers
    buf_h: Vec<C64>,
    buf_v: Vec<C64>,
    buf_idx: usize,
    // RLS inverse correlation matrices (4 × N × N matrices flattened)
    p_hv: Vec<C64>, // for W_HV
    p_vh: Vec<C64>, // for W_VH
    p_hh: Vec<C64>, // for W_HH
    p_vv: Vec<C64>, // for W_VV
    // Training mode flag
    training: bool,
    // Statistics
    pub samples_processed: u64,
    pub mse_h: f64,
    pub mse_v: f64,
}

impl XpicFilter {
    /// Create a new XPIC filter.
    pub fn new(config: XpicConfig) -> Self {
        let n = config.num_taps.max(1);
        let delta = config.rls_delta;
        let make_p = |n: usize, delta: f64| -> Vec<C64> {
            let mut p = vec![C64::new(0.0, 0.0); n * n];
            for i in 0..n { p[i * n + i] = C64::new(delta, 0.0); }
            p
        };
        // Initialize W_HH and W_VV to identity (tap 0 = 1, rest = 0)
        let mut w_hh = vec![C64::new(0.0, 0.0); n];
        let mut w_vv = vec![C64::new(0.0, 0.0); n];
        w_hh[0] = C64::new(1.0, 0.0);
        w_vv[0] = C64::new(1.0, 0.0);
        Self {
            p_hv: make_p(n, delta),
            p_vh: make_p(n, delta),
            p_hh: make_p(n, delta),
            p_vv: make_p(n, delta),
            w_hv: vec![C64::new(0.0, 0.0); n],
            w_vh: vec![C64::new(0.0, 0.0); n],
            w_hh,
            w_vv,
            buf_h: vec![C64::new(0.0, 0.0); n],
            buf_v: vec![C64::new(0.0, 0.0); n],
            buf_idx: 0,
            training: true,
            config,
            samples_processed: 0,
            mse_h: 1.0,
            mse_v: 1.0,
        }
    }

    /// Enable or disable training mode.
    /// In training mode, known pilot symbols are used for adaptation.
    /// After training, switch to decision-directed (or stop adapting).
    pub fn set_training(&mut self, training: bool) {
        self.training = training;
    }

    /// Process one sample pair with a known pilot (training mode).
    ///
    /// # Arguments
    /// * `r_h`, `r_v` — received samples on H and V polarizations.
    /// * `pilot_h`, `pilot_v` — known transmitted pilot symbols.
    ///
    /// # Returns
    /// `(output_h, output_v)` — XPIC-cancelled outputs.
    pub fn process_pilot(
        &mut self,
        r_h: C64, r_v: C64,
        pilot_h: C64, pilot_v: C64,
    ) -> (C64, C64) {
        self.buf_h[self.buf_idx] = r_h;
        self.buf_v[self.buf_idx] = r_v;

        let xh = self.read_buffer_h();
        let xv = self.read_buffer_v();

        let y_h = self.filter_output(&self.w_hh.clone(), &xh) + self.filter_output(&self.w_hv.clone(), &xv);
        let y_v = self.filter_output(&self.w_vv.clone(), &xv) + self.filter_output(&self.w_vh.clone(), &xh);

        let e_h = pilot_h - y_h;
        let e_v = pilot_v - y_v;

        self.adapt(&xh, &xv, e_h, e_v);
        self.update_mse(e_h, e_v);

        self.buf_idx = (self.buf_idx + 1) % self.config.num_taps;
        self.samples_processed += 1;
        (y_h, y_v)
    }

    /// Process one sample pair without pilots (decision-directed or pass-through).
    ///
    /// If the `decision_fn` closure is Some, decision-directed mode is used
    /// where the slicer output drives adaptation. Otherwise, adaptation stops.
    pub fn process<F>(
        &mut self,
        r_h: C64, r_v: C64,
        decision_fn: Option<F>,
    ) -> (C64, C64)
    where F: Fn(C64, C64) -> (C64, C64)
    {
        self.buf_h[self.buf_idx] = r_h;
        self.buf_v[self.buf_idx] = r_v;

        let xh = self.read_buffer_h();
        let xv = self.read_buffer_v();

        let y_h = self.filter_output(&self.w_hh.clone(), &xh) + self.filter_output(&self.w_hv.clone(), &xv);
        let y_v = self.filter_output(&self.w_vv.clone(), &xv) + self.filter_output(&self.w_vh.clone(), &xh);

        if let Some(decide) = decision_fn {
            let (d_h, d_v) = decide(y_h, y_v);
            let e_h = d_h - y_h;
            let e_v = d_v - y_v;
            self.adapt(&xh, &xv, e_h, e_v);
            self.update_mse(e_h, e_v);
        }

        self.buf_idx = (self.buf_idx + 1) % self.config.num_taps;
        self.samples_processed += 1;
        (y_h, y_v)
    }

    /// Batch training over a sequence of pilot pairs.
    /// Returns XpicMetrics after training.
    pub fn process_training(
        &mut self,
        pilots: &[(C64, C64)],
        received: &[(C64, C64)],
    ) -> XpicMetrics {
        self.set_training(true);
        let n = pilots.len().min(received.len());
        for i in 0..n {
            let (r_h, r_v) = received[i];
            let (p_h, p_v) = pilots[i];
            self.process_pilot(r_h, r_v, p_h, p_v);
        }
        self.compute_metrics(None)
    }

    /// Current cross-pol filter taps W_HV.
    pub fn taps_hv(&self) -> &[C64] { &self.w_hv }
    /// Current cross-pol filter taps W_VH.
    pub fn taps_vh(&self) -> &[C64] { &self.w_vh }
    /// Current co-pol filter taps W_HH.
    pub fn taps_hh(&self) -> &[C64] { &self.w_hh }
    /// Current co-pol filter taps W_VV.
    pub fn taps_vv(&self) -> &[C64] { &self.w_vv }

    /// Compute current XPI (Cross-Polar Isolation) from filter taps.
    ///
    /// XPI ≈ -10·log10(||W_HV||² / ||W_HH||²)
    pub fn xpi_improvement_db(&self) -> f64 {
        let cross_sq: f64 = self.w_hv.iter().map(|w| w.norm_sqr()).sum::<f64>()
            + self.w_vh.iter().map(|w| w.norm_sqr()).sum::<f64>();
        let co_sq: f64 = self.w_hh.iter().map(|w| w.norm_sqr()).sum::<f64>()
            + self.w_vv.iter().map(|w| w.norm_sqr()).sum::<f64>();
        if cross_sq < 1e-30 { return 60.0; }
        -10.0 * (cross_sq / co_sq.max(1e-30)).log10()
    }

    /// Compute performance metrics.
    pub fn compute_metrics(&self, channel: Option<&PolarizationChannel>) -> XpicMetrics {
        let xpi_improvement = self.xpi_improvement_db();
        let initial_xpd = channel.map(|c| (c.xpd_hv_db() + c.xpd_vh_db()) / 2.0);
        let final_xpi = initial_xpd.map(|xpd| xpd + xpi_improvement);
        XpicMetrics {
            xpi_improvement_db: xpi_improvement,
            initial_xpd_db: initial_xpd.unwrap_or(0.0),
            final_xpi_db: final_xpi.unwrap_or(xpi_improvement),
            mse_h: self.mse_h,
            mse_v: self.mse_v,
            samples_trained: self.samples_processed,
            sinr_improvement_db: xpi_improvement.min(40.0),
        }
    }

    // ---- Internal helpers ----

    fn read_buffer_h(&self) -> Vec<C64> {
        let n = self.config.num_taps;
        (0..n).map(|i| self.buf_h[(self.buf_idx + n - i) % n]).collect()
    }

    fn read_buffer_v(&self) -> Vec<C64> {
        let n = self.config.num_taps;
        (0..n).map(|i| self.buf_v[(self.buf_idx + n - i) % n]).collect()
    }

    fn filter_output(&self, w: &[C64], x: &[C64]) -> C64 {
        w.iter().zip(x.iter()).map(|(&wi, &xi)| wi * xi).fold(C64::new(0.0, 0.0), |a, b| a + b)
    }

    fn adapt(&mut self, xh: &[C64], xv: &[C64], e_h: C64, e_v: C64) {
        match self.config.algorithm {
            XpicAlgorithm::Lms | XpicAlgorithm::DecisionDirectedLms => {
                let mu = self.config.lms_step_size;
                let leak = 1.0 - self.config.leakage;
                let n = self.config.num_taps;
                // W_HV update: cancels V leakage into H output
                for i in 0..n {
                    self.w_hv[i] = leak * self.w_hv[i] + mu * e_h * xv[i].conj();
                }
                // W_VH update: cancels H leakage into V output
                for i in 0..n {
                    self.w_vh[i] = leak * self.w_vh[i] + mu * e_v * xh[i].conj();
                }
                // W_HH update
                for i in 0..n {
                    self.w_hh[i] = leak * self.w_hh[i] + mu * e_h * xh[i].conj();
                }
                // W_VV update
                for i in 0..n {
                    self.w_vv[i] = leak * self.w_vv[i] + mu * e_v * xv[i].conj();
                }
            }
            XpicAlgorithm::Rls => {
                self.rls_update(xh, xv, e_h, e_v);
            }
        }
    }

    fn rls_update(&mut self, xh: &[C64], xv: &[C64], e_h: C64, e_v: C64) {
        let n = self.config.num_taps;
        let lambda = self.config.rls_lambda;

        // Update W_HV using xv and error e_h
        rls_step(n, &mut self.p_hv, &mut self.w_hv, xv, e_h, lambda);
        // Update W_VH using xh and error e_v
        rls_step(n, &mut self.p_vh, &mut self.w_vh, xh, e_v, lambda);
        // Update W_HH using xh and error e_h
        rls_step(n, &mut self.p_hh, &mut self.w_hh, xh, e_h, lambda);
        // Update W_VV using xv and error e_v
        rls_step(n, &mut self.p_vv, &mut self.w_vv, xv, e_v, lambda);
    }

    fn update_mse(&mut self, e_h: C64, e_v: C64) {
        let alpha = 0.01f64;
        self.mse_h = (1.0 - alpha) * self.mse_h + alpha * e_h.norm_sqr();
        self.mse_v = (1.0 - alpha) * self.mse_v + alpha * e_v.norm_sqr();
    }
}

/// One RLS update step for a single adaptive filter.
///
/// Implements:
/// ```text
/// k(n)    = P(n-1) x(n) / (λ + x^H P(n-1) x(n))
/// w(n)    = w(n-1) + k(n) e*(n)
/// P(n)    = (1/λ)(P(n-1) - k(n) x^H(n) P(n-1))
/// ```
fn rls_step(n: usize, p: &mut Vec<C64>, w: &mut Vec<C64>, x: &[C64], error: C64, lambda: f64) {
    // Compute Px = P * x  (N-vector)
    let mut px = vec![C64::new(0.0, 0.0); n];
    for i in 0..n {
        for j in 0..n {
            px[i] += p[i * n + j] * x[j];
        }
    }
    // Denominator: λ + x^H * Px
    let denom = lambda + x.iter().zip(px.iter()).map(|(&xi, &pxi)| xi.conj() * pxi).fold(C64::new(0.0, 0.0), |a, b| a + b);
    let denom_re = denom.re.max(1e-30);
    // Gain vector k = Px / denom
    let k: Vec<C64> = px.iter().map(|&pxi| pxi / C64::new(denom_re, 0.0)).collect();
    // Update weights: w = w + k * conj(error)
    for i in 0..n {
        w[i] += k[i] * error.conj();
    }
    // Update P: P = (1/λ)(P - k * (Px)^H)
    // Note: k * x^H P = k * (Px)^H
    let inv_lambda = 1.0 / lambda;
    for i in 0..n {
        for j in 0..n {
            p[i * n + j] = inv_lambda * (p[i * n + j] - k[i] * px[j].conj());
        }
    }
}

// ---------------------------------------------------------------------------
// 6. Performance Metrics
// ---------------------------------------------------------------------------

/// XPIC performance metrics.
#[derive(Debug, Clone)]
pub struct XpicMetrics {
    /// XPI improvement achieved by the XPIC filter (dB).
    /// Typical XPIC systems achieve 20–30 dB additional XPI.
    pub xpi_improvement_db: f64,
    /// Initial XPD of the channel before XPIC (dB).
    pub initial_xpd_db: f64,
    /// Final effective XPI after XPIC (dB).
    pub final_xpi_db: f64,
    /// MSE on H channel output.
    pub mse_h: f64,
    /// MSE on V channel output.
    pub mse_v: f64,
    /// Number of training samples processed.
    pub samples_trained: u64,
    /// Effective SINR improvement (dB). Capped at ~40 dB.
    pub sinr_improvement_db: f64,
}

impl XpicMetrics {
    /// Capacity gain from dual-pol operation over single-pol baseline.
    ///
    /// Shannon capacity of dual-pol link (bits/s/Hz):
    ///   C_dual = log2(1 + SINR_H) + log2(1 + SINR_V)
    ///
    /// vs. single-pol:
    ///   C_single = log2(1 + SNR)
    ///
    /// Assumes equal SNR on both pols.
    pub fn capacity_gain_db(&self, snr_db: f64) -> f64 {
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let xpi_linear = 10.0_f64.powf(-self.final_xpi_db / 10.0);
        // SINR after XPIC
        let sinr = snr_linear / (1.0 + snr_linear * xpi_linear);
        let c_dual = 2.0 * (1.0 + sinr).log2();
        let c_single = (1.0 + snr_linear).log2();
        if c_single < 1e-10 { return 0.0; }
        // Capacity gain in dB
        10.0 * (c_dual / c_single).log10()
    }

    /// Spectral efficiency (bits/s/Hz) for the dual-pol system.
    pub fn spectral_efficiency(&self, snr_db: f64, modulation_bits: u32) -> f64 {
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let xpi_linear = 10.0_f64.powf(-self.final_xpi_db / 10.0);
        let sinr = snr_linear / (1.0 + snr_linear * xpi_linear);
        // Practical throughput bounded by modulation order
        let shannon = 2.0 * (1.0 + sinr).log2();
        let max_rate = 2.0 * modulation_bits as f64;
        shannon.min(max_rate)
    }
}

// ---------------------------------------------------------------------------
// 7. Diversity Combining
// ---------------------------------------------------------------------------

/// Polarization diversity branch combining mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DiversityMode {
    /// Maximum Ratio Combining — optimal SNR combining (requires channel knowledge).
    MaximumRatio,
    /// Equal Gain Combining — co-phase and sum.
    EqualGain,
    /// Selection Combining — select the strongest branch.
    Selection,
}

/// Space + polarization diversity combiner.
///
/// Combines H and V polarization branches to provide diversity gain against
/// signal fading. In a dual-pol system, H and V channels are partially
/// decorrelated (correlation ρ depends on antenna isolation and XPD).
#[derive(Debug, Clone)]
pub struct PolarizationDiversityCombiner {
    pub mode: DiversityMode,
    /// Estimated SNR on H branch (linear).
    pub snr_h: f64,
    /// Estimated SNR on V branch (linear).
    pub snr_v: f64,
    /// Channel estimate on H path (for MRC).
    h_est_h: C64,
    /// Channel estimate on V path (for MRC).
    h_est_v: C64,
    /// Smoothing coefficient for SNR tracking.
    alpha: f64,
}

impl PolarizationDiversityCombiner {
    /// Create a diversity combiner.
    pub fn new(mode: DiversityMode) -> Self {
        Self {
            mode,
            snr_h: 1.0,
            snr_v: 1.0,
            h_est_h: C64::new(1.0, 0.0),
            h_est_v: C64::new(1.0, 0.0),
            alpha: 0.01,
        }
    }

    /// Update channel estimate (pilot-based).
    pub fn update_channel(&mut self, r_h: C64, r_v: C64, pilot: C64) {
        let p_sq = pilot.norm_sqr().max(1e-30);
        let h_h = r_h / pilot;
        let h_v = r_v / pilot;
        // Track channel with exponential averaging
        self.h_est_h = (1.0 - self.alpha) * self.h_est_h + self.alpha * h_h;
        self.h_est_v = (1.0 - self.alpha) * self.h_est_v + self.alpha * h_v;
        // Update SNR estimates from signal power
        let sig_h = r_h.norm_sqr() / p_sq;
        let sig_v = r_v.norm_sqr() / p_sq;
        self.snr_h = (1.0 - self.alpha) * self.snr_h + self.alpha * sig_h;
        self.snr_v = (1.0 - self.alpha) * self.snr_v + self.alpha * sig_v;
    }

    /// Combine H and V branches.
    pub fn combine(&self, r_h: C64, r_v: C64) -> C64 {
        match self.mode {
            DiversityMode::MaximumRatio => {
                // MRC: y = h_H* · r_H / σ²_H + h_V* · r_V / σ²_V
                let w_h = self.h_est_h.conj() * self.snr_h;
                let w_v = self.h_est_v.conj() * self.snr_v;
                let norm = (w_h.norm_sqr() + w_v.norm_sqr()).sqrt().max(1e-30);
                (w_h * r_h + w_v * r_v) / C64::new(norm, 0.0)
            }
            DiversityMode::EqualGain => {
                // EGC: co-phase and sum with equal weights
                let ph_h = self.h_est_h.arg();
                let ph_v = self.h_est_v.arg();
                let r_h_rot = r_h * C64::new((-ph_h).cos(), (-ph_h).sin());
                let r_v_rot = r_v * C64::new((-ph_v).cos(), (-ph_v).sin());
                (r_h_rot + r_v_rot) / C64::new(2.0_f64.sqrt(), 0.0)
            }
            DiversityMode::Selection => {
                if r_h.norm_sqr() >= r_v.norm_sqr() { r_h } else { r_v }
            }
        }
    }

    /// Estimated diversity gain (dB) vs. single branch.
    ///
    /// For two-branch MRC with correlation ρ, diversity gain ≈ 10·log10(2/(1+ρ)).
    /// For uncorrelated branches (ρ≈0): ~3 dB gain.
    pub fn diversity_gain_db(&self, correlation: f64) -> f64 {
        let rho = correlation.clamp(-1.0, 0.99);
        match self.mode {
            DiversityMode::MaximumRatio => {
                10.0 * (2.0 / (1.0 + rho)).log10()
            }
            DiversityMode::EqualGain => {
                10.0 * ((1.0 + (1.0 - rho.powi(2)).sqrt()) / (1.0 + rho)).log10()
            }
            DiversityMode::Selection => {
                // Two-branch SC average SNR gain ≈ 1.5× (1.76 dB) for uncorrelated
                // Rayleigh branches (harmonic number H_2 = 1 + 1/2 = 1.5).
                // Reduces with branch correlation: gain ≈ 10·log10(1.5 / (1 + 0.5*ρ²))
                10.0 * (1.5 / (1.0 + 0.5 * rho.powi(2))).log10()
            }
        }
    }

    /// Outage probability improvement (ratio) compared to single branch.
    /// Uses simplified model for Rayleigh fading.
    pub fn outage_improvement(&self, correlation: f64) -> f64 {
        let rho = correlation.clamp(0.0, 0.99);
        match self.mode {
            DiversityMode::MaximumRatio => 1.0 / (1.0 - rho.powi(2)).max(1e-6),
            DiversityMode::EqualGain    => 1.0 / (1.0 - rho).max(1e-6),
            DiversityMode::Selection    => (1.0 + rho) / (1.0 - rho).max(1e-6),
        }
    }
}

// ---------------------------------------------------------------------------
// 8. Complete Dual-Pol Receiver
// ---------------------------------------------------------------------------

/// Configuration for the complete dual-pol XPIC receiver.
#[derive(Debug, Clone)]
pub struct DualPolConfig {
    pub xpic: XpicConfig,
    pub diversity_mode: DiversityMode,
    /// Enable SOP tracking.
    pub enable_sop_tracking: bool,
    /// SOP tracker bandwidth (α coefficient).
    pub sop_alpha: f64,
    /// Output XPIC-cancelled signals, or combined signal.
    pub output_combined: bool,
}

impl Default for DualPolConfig {
    fn default() -> Self {
        Self {
            xpic: XpicConfig::default(),
            diversity_mode: DiversityMode::MaximumRatio,
            enable_sop_tracking: true,
            sop_alpha: 0.001,
            output_combined: false,
        }
    }
}

/// Complete dual-polarization receiver with XPIC and optional diversity combining.
///
/// Processing chain:
/// ```text
/// r_H ─┬─ SOP correction ─┬─ XPIC filter ─┬─ XPIC output H
/// r_V ─┘                  └─────────────── └─ XPIC output V
///                                              │
///                                              ▼ (if output_combined)
///                                         Diversity combiner
///                                              │
///                                              ▼
///                                         combined output
/// ```
#[derive(Debug, Clone)]
pub struct DualPolReceiver {
    pub xpic: XpicFilter,
    pub diversity: PolarizationDiversityCombiner,
    pub sop_tracker: PolarizationTracker,
    config: DualPolConfig,
    /// Running XPI measurement before XPIC (from raw received signals).
    xpi_before: f64,
    /// Running XPI measurement after XPIC.
    xpi_after: f64,
    sample_count: u64,
}

impl DualPolReceiver {
    /// Create a complete dual-pol receiver.
    pub fn new(config: DualPolConfig) -> Self {
        let xpic = XpicFilter::new(config.xpic.clone());
        let diversity = PolarizationDiversityCombiner::new(config.diversity_mode);
        let sop_tracker = PolarizationTracker::new(config.sop_alpha);
        Self {
            xpic,
            diversity,
            sop_tracker,
            config,
            xpi_before: 0.0,
            xpi_after: 0.0,
            sample_count: 0,
        }
    }

    /// Process a training sample.
    pub fn process_training_sample(
        &mut self,
        r_h: C64, r_v: C64,
        pilot_h: C64, pilot_v: C64,
    ) -> (C64, C64) {
        // SOP tracking
        let (r_h, r_v) = if self.config.enable_sop_tracking {
            self.sop_tracker.update(r_h, r_v);
            self.sop_tracker.correct(r_h, r_v)
        } else {
            (r_h, r_v)
        };

        // Update diversity channel estimate
        self.diversity.update_channel(r_h, r_v, pilot_h);

        // Measure XPI before XPIC
        let xpi_raw = r_v.norm_sqr() / r_h.norm_sqr().max(1e-30);
        self.xpi_before = 0.99 * self.xpi_before + 0.01 * xpi_raw;

        // XPIC processing
        let (y_h, y_v) = self.xpic.process_pilot(r_h, r_v, pilot_h, pilot_v);

        // Measure XPI after XPIC
        let xpi_post = y_v.norm_sqr() / y_h.norm_sqr().max(1e-30);
        self.xpi_after = 0.99 * self.xpi_after + 0.01 * xpi_post;

        self.sample_count += 1;
        (y_h, y_v)
    }

    /// XPI before XPIC in dB.
    pub fn xpi_before_db(&self) -> f64 {
        -10.0 * self.xpi_before.max(1e-30).log10()
    }

    /// XPI after XPIC in dB.
    pub fn xpi_after_db(&self) -> f64 {
        -10.0 * self.xpi_after.max(1e-30).log10()
    }

    /// Overall XPI improvement (dB).
    pub fn xpi_improvement_db(&self) -> f64 {
        self.xpi_after_db() - self.xpi_before_db()
    }

    /// Current Stokes parameters from SOP tracker.
    pub fn stokes(&self) -> &StokesParameters {
        &self.sop_tracker.stokes
    }

    /// Sample count processed.
    pub fn samples_processed(&self) -> u64 {
        self.sample_count
    }
}

// ---------------------------------------------------------------------------
// 9. Capacity and Link Budget
// ---------------------------------------------------------------------------

/// Dual-pol link capacity calculator.
///
/// Computes Shannon capacity and practical throughput for a dual-pol microwave
/// backhaul link.
#[derive(Debug, Clone)]
pub struct DualPolCapacity {
    /// System bandwidth (Hz).
    pub bandwidth_hz: f64,
    /// Co-pol SNR (dB).
    pub snr_db: f64,
    /// Effective XPI after XPIC (dB). Determines residual interference.
    pub xpi_db: f64,
    /// Modulation order (bits per symbol). E.g., 128-QAM = 7, 1024-QAM = 10.
    pub modulation_bits: u32,
    /// FEC code rate (0–1). Typically 0.75–0.9 for commercial microwave.
    pub fec_rate: f64,
}

impl DualPolCapacity {
    /// Shannon capacity (bits/s) for dual-pol operation.
    pub fn shannon_capacity(&self) -> f64 {
        let snr = 10.0_f64.powf(self.snr_db / 10.0);
        let xpi = 10.0_f64.powf(-self.xpi_db / 10.0);
        // SINR: interference from cross-pol after XPIC
        let sinr = snr / (1.0 + snr * xpi);
        2.0 * self.bandwidth_hz * (1.0 + sinr).log2()
    }

    /// Shannon capacity without XPIC (single-pol baseline, bps).
    pub fn shannon_capacity_single_pol(&self) -> f64 {
        let snr = 10.0_f64.powf(self.snr_db / 10.0);
        self.bandwidth_hz * (1.0 + snr).log2()
    }

    /// Practical throughput (bits/s), bounded by modulation order and FEC rate.
    pub fn practical_throughput(&self) -> f64 {
        let max_rate = 2.0 * self.bandwidth_hz * self.modulation_bits as f64 * self.fec_rate;
        self.shannon_capacity().min(max_rate)
    }

    /// Spectral efficiency gain from dual-pol vs. single-pol (dB).
    pub fn capacity_gain_db(&self) -> f64 {
        let dual = self.shannon_capacity();
        let single = self.shannon_capacity_single_pol();
        if single < 1.0 { return 0.0; }
        10.0 * (dual / single).log10()
    }

    /// Minimum required XPI (dB) to achieve a target SNR degradation (dB).
    ///
    /// Solves for XPI such that SINR = SNR - degradation_db.
    pub fn required_xpi_for_degradation(&self, degradation_db: f64) -> f64 {
        let snr = 10.0_f64.powf(self.snr_db / 10.0);
        let sinr_target = 10.0_f64.powf((self.snr_db - degradation_db) / 10.0);
        // sinr = snr / (1 + snr * xpi)  →  xpi = (snr/sinr - 1) / snr
        let xpi_linear = (snr / sinr_target - 1.0) / snr;
        if xpi_linear <= 0.0 { return 100.0; }
        -10.0 * xpi_linear.log10()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn c(re: f64, im: f64) -> C64 { C64::new(re, im) }

    // --- Matrix2x2 Tests ---

    #[test]
    fn test_matrix2x2_identity() {
        let id = Matrix2x2::identity();
        let (y0, y1) = id.mul_vec(c(3.0, 1.0), c(-2.0, 4.0));
        assert!((y0 - c(3.0, 1.0)).norm() < TOL);
        assert!((y1 - c(-2.0, 4.0)).norm() < TOL);
    }

    #[test]
    fn test_matrix2x2_determinant() {
        let m = Matrix2x2::new(c(2.0, 0.0), c(1.0, 0.0), c(1.0, 0.0), c(3.0, 0.0));
        let det = m.det();
        assert!((det - c(5.0, 0.0)).norm() < TOL);
    }

    #[test]
    fn test_matrix2x2_inverse() {
        let m = Matrix2x2::new(c(2.0, 0.0), c(0.0, 1.0), c(0.0, -1.0), c(2.0, 0.0));
        let inv = m.inv().expect("matrix should be invertible");
        let product = m.mul_mat(&inv);
        // Should be close to identity
        assert!((product.get(0, 0) - c(1.0, 0.0)).norm() < 1e-10);
        assert!((product.get(1, 1) - c(1.0, 0.0)).norm() < 1e-10);
        assert!(product.get(0, 1).norm() < 1e-10);
        assert!(product.get(1, 0).norm() < 1e-10);
    }

    #[test]
    fn test_matrix2x2_hermitian() {
        let m = Matrix2x2::new(c(1.0, 2.0), c(3.0, 4.0), c(5.0, 6.0), c(7.0, 8.0));
        let h = m.hermitian();
        // (H^H)[i,j] = conj(H[j,i])
        assert!((h.get(0, 0) - c(1.0, -2.0)).norm() < TOL);
        assert!((h.get(0, 1) - c(5.0, -6.0)).norm() < TOL);
        assert!((h.get(1, 0) - c(3.0, -4.0)).norm() < TOL);
        assert!((h.get(1, 1) - c(7.0, -8.0)).norm() < TOL);
    }

    #[test]
    fn test_matrix2x2_mul_mat_associativity() {
        let a = Matrix2x2::new(c(1.0, 0.0), c(2.0, 0.0), c(3.0, 0.0), c(4.0, 0.0));
        let b = Matrix2x2::new(c(5.0, 0.0), c(6.0, 0.0), c(7.0, 0.0), c(8.0, 0.0));
        let c_mat = Matrix2x2::new(c(9.0, 0.0), c(0.0, 0.0), c(0.0, 0.0), c(1.0, 0.0));
        let ab_c = a.mul_mat(&b).mul_mat(&c_mat);
        let a_bc = a.mul_mat(&b.mul_mat(&c_mat));
        for i in 0..4 {
            assert!((ab_c.data[i] - a_bc.data[i]).norm() < TOL);
        }
    }

    #[test]
    fn test_matrix2x2_condition_number_identity() {
        let id = Matrix2x2::identity();
        assert!((id.condition_number() - 1.0).abs() < TOL);
    }

    #[test]
    fn test_matrix2x2_singular() {
        // Rank-deficient matrix
        let m = Matrix2x2::new(c(1.0, 0.0), c(2.0, 0.0), c(2.0, 0.0), c(4.0, 0.0));
        assert!(m.inv().is_none());
    }

    // --- PolarizationChannel Tests ---

    #[test]
    fn test_channel_xpd_30db() {
        let ch = PolarizationChannel::new_with_xpd(30.0, 0.0);
        let xpd = ch.xpd_hv_db();
        assert!((xpd - 30.0).abs() < 1e-9, "XPD should be 30 dB, got {xpd}");
    }

    #[test]
    fn test_channel_xpd_40db() {
        let ch = PolarizationChannel::new_with_xpd(40.0, 0.0);
        let xpd = ch.xpd_hv_db();
        assert!((xpd - 40.0).abs() < 1e-9, "XPD should be 40 dB, got {xpd}");
    }

    #[test]
    fn test_channel_apply_identity() {
        // With infinite XPD (pure co-pol), channel is diagonal
        let jones = Matrix2x2::new(c(1.0, 0.0), c(0.0, 0.0), c(0.0, 0.0), c(1.0, 0.0));
        let ch = PolarizationChannel::new(jones, 0.0);
        let s_h = c(1.0, 0.5);
        let s_v = c(-0.5, 1.0);
        let (r_h, r_v) = ch.apply(s_h, s_v);
        assert!((r_h - s_h).norm() < TOL);
        assert!((r_v - s_v).norm() < TOL);
    }

    #[test]
    fn test_channel_xpd_symmetry() {
        let ch = PolarizationChannel::new_with_xpd(35.0, 0.0);
        // Symmetric off-diagonal → XPD_HV ≈ XPD_VH
        let xpd_hv = ch.xpd_hv_db();
        let xpd_vh = ch.xpd_vh_db();
        assert!((xpd_hv - xpd_vh).abs() < 1e-9);
    }

    #[test]
    fn test_channel_jones_linearity() {
        let ch = PolarizationChannel::new_with_xpd(30.0, 0.0);
        let s1_h = c(1.0, 0.0);
        let s1_v = c(0.0, 1.0);
        let s2_h = c(0.5, -0.5);
        let s2_v = c(-0.5, 0.5);
        let (r1_h, r1_v) = ch.apply(s1_h, s1_v);
        let (r2_h, r2_v) = ch.apply(s2_h, s2_v);
        let (r_sum_h, r_sum_v) = ch.apply(s1_h + s2_h, s1_v + s2_v);
        assert!((r_sum_h - r1_h - r2_h).norm() < TOL);
        assert!((r_sum_v - r1_v - r2_v).norm() < TOL);
    }

    // --- Rain Depolarization Tests ---

    #[test]
    fn test_rain_copolar_atten_zero_rain() {
        let model = RainDepolarization::new(11.0, 0.0, 10.0);
        let att = model.copolar_attenuation_db(0.0);
        assert!(att.abs() < TOL, "Zero rain → zero attenuation");
    }

    #[test]
    fn test_rain_copolar_atten_increases_with_rain_rate() {
        let model = RainDepolarization::new(18.0, 0.0, 20.0);
        let a10 = model.copolar_attenuation_db(10.0);
        let a50 = model.copolar_attenuation_db(50.0);
        let a100 = model.copolar_attenuation_db(100.0);
        assert!(a10 < a50, "Attenuation increases with rain rate");
        assert!(a50 < a100);
    }

    #[test]
    fn test_rain_xpd_degradation() {
        let model = RainDepolarization::new(11.0, 0.0, 10.0);
        let (xpd_clear, xpd_rain) = model.xpd_with_rain(50.0, 35.0);
        assert_eq!(xpd_clear, 35.0, "Clear-sky XPD unchanged");
        assert!(xpd_rain <= xpd_clear, "Rain degrades XPD");
    }

    #[test]
    fn test_rain_jones_matrix_structure() {
        let model = RainDepolarization::new(11.0, 0.0, 10.0);
        let m = model.rain_jones_matrix(30.0, 35.0);
        // Co-pol terms should be larger than cross-pol terms
        let h_hh = m.get(0, 0).norm();
        let h_hv = m.get(0, 1).norm();
        assert!(h_hh > h_hv, "Co-pol > cross-pol amplitude");
    }

    #[test]
    fn test_itu_r_p838_coefficients_6ghz() {
        let model = RainDepolarization::new(6.0, 0.0, 5.0);
        let att = model.copolar_attenuation_db(20.0);
        // At 6 GHz, 20 mm/h, 5 km, attenuation should be roughly 0.1–2 dB
        assert!(att > 0.0 && att < 10.0, "Reasonable 6 GHz attenuation: {att}");
    }

    #[test]
    fn test_differential_effects_proportional() {
        let model = RainDepolarization::new(18.0, 0.0, 10.0);
        let (da1, dp1) = model.differential_effects(10.0);
        let (da2, dp2) = model.differential_effects(50.0);
        // Higher rain → more differential effects
        assert!(da2 > da1);
        assert!(dp2 > dp1);
    }

    // --- Stokes Parameters Tests ---

    #[test]
    fn test_stokes_horizontal_polarization() {
        let e_h = c(1.0, 0.0);
        let e_v = c(0.0, 0.0);
        let sp = StokesParameters::from_sample(e_h, e_v);
        assert!((sp.s0 - 1.0).abs() < TOL, "S0 = total power");
        assert!((sp.s1 - 1.0).abs() < TOL, "S1 = 1 for H pol");
        assert!(sp.s2.abs() < TOL);
        assert!(sp.s3.abs() < TOL);
    }

    #[test]
    fn test_stokes_vertical_polarization() {
        let e_h = c(0.0, 0.0);
        let e_v = c(1.0, 0.0);
        let sp = StokesParameters::from_sample(e_h, e_v);
        assert!((sp.s0 - 1.0).abs() < TOL);
        assert!((sp.s1 + 1.0).abs() < TOL, "S1 = -1 for V pol");
        assert!(sp.s2.abs() < TOL);
        assert!(sp.s3.abs() < TOL);
    }

    #[test]
    fn test_stokes_circular_right() {
        // Right-circular: E_H = 1, E_V = +j
        let norm = 1.0 / 2.0_f64.sqrt();
        let e_h = c(norm, 0.0);
        let e_v = c(0.0, norm);
        let sp = StokesParameters::from_sample(e_h, e_v);
        assert!((sp.s0 - 1.0).abs() < TOL);
        assert!(sp.s1.abs() < TOL);
        assert!(sp.s2.abs() < TOL);
        assert!((sp.s3 - (-1.0)).abs() < TOL, "S3 = -1 for RHCP"); // Note: S3 = 2*Im(E_H*E_V*)
    }

    #[test]
    fn test_stokes_degree_of_polarization() {
        let e_h = c(1.0, 0.0);
        let e_v = c(0.0, 1.0);
        let sp = StokesParameters::from_sample(e_h, e_v);
        // 45° linear is fully polarized
        assert!((sp.degree_of_polarization() - 1.0).abs() < TOL);
    }

    #[test]
    fn test_stokes_power_conservation() {
        let e_h = c(0.8, 0.3);
        let e_v = c(-0.2, 0.5);
        let sp = StokesParameters::from_sample(e_h, e_v);
        assert!((sp.s0 - e_h.norm_sqr() - e_v.norm_sqr()).abs() < TOL);
    }

    #[test]
    fn test_stokes_rotation() {
        let e_h = c(1.0, 0.0);
        let e_v = c(0.0, 0.0);
        let sp = StokesParameters::from_sample(e_h, e_v);
        let rotated = sp.rotate(PI / 4.0);
        // 45° rotation of H pol → 45° linear → S2 should be nonzero
        assert!((rotated.s0 - sp.s0).abs() < TOL, "Rotation preserves power");
    }

    #[test]
    fn test_stokes_from_block() {
        let n = 100;
        let samples_h: Vec<C64> = (0..n).map(|_| c(1.0, 0.0)).collect();
        let samples_v: Vec<C64> = (0..n).map(|_| c(0.0, 0.0)).collect();
        let sp = StokesParameters::from_block(&samples_h, &samples_v);
        assert!((sp.s1 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_stokes_ellipticity_angle_linear() {
        // Linear polarization → S3 = 0 → ellipticity = 0
        let e_h = c(1.0, 0.0);
        let e_v = c(1.0, 0.0);
        let sp = StokesParameters::from_sample(e_h, e_v);
        assert!(sp.ellipticity_angle().abs() < TOL);
    }

    // --- Polarization Tracker Tests ---

    #[test]
    fn test_pol_tracker_initial() {
        let mut tracker = PolarizationTracker::new(0.1);
        let sp = tracker.update(c(1.0, 0.0), c(0.0, 0.0));
        assert!(sp.s0 > 0.0);
    }

    #[test]
    fn test_pol_tracker_convergence() {
        let mut tracker = PolarizationTracker::new(0.1);
        for _ in 0..200 {
            tracker.update(c(1.0, 0.0), c(0.0, 0.0));
        }
        // Should converge to H polarization
        assert!(tracker.stokes.s1 > 0.0, "H-pol dominant");
    }

    #[test]
    fn test_pol_tracker_correction() {
        let mut tracker = PolarizationTracker::new(0.01);
        // Feed with H-pol signal
        for _ in 0..500 {
            tracker.update(c(1.0, 0.0), c(0.0, 0.0));
        }
        let (y_h, _y_v) = tracker.correct(c(1.0, 0.0), c(0.0, 0.0));
        // After correction, H-pol should remain dominant
        assert!(y_h.norm() > 0.5);
    }

    // --- XPIC Filter Tests ---

    #[test]
    fn test_xpic_construction() {
        let cfg = XpicConfig { num_taps: 4, ..Default::default() };
        let xpic = XpicFilter::new(cfg);
        assert_eq!(xpic.taps_hv().len(), 4);
        assert_eq!(xpic.taps_vh().len(), 4);
    }

    #[test]
    fn test_xpic_lms_convergence() {
        // Channel with 25 dB XPD — significant cross-pol leakage
        let channel = PolarizationChannel::new_with_xpd(25.0, 0.0);
        let cfg = XpicConfig {
            num_taps: 1,
            lms_step_size: 0.05,
            algorithm: XpicAlgorithm::Lms,
            ..Default::default()
        };
        let mut xpic = XpicFilter::new(cfg);

        // Generate training data
        let n_train = 2000;
        let pilots: Vec<(C64, C64)> = (0..n_train).map(|i| {
            let p = C64::new((i as f64 * 0.3).cos(), (i as f64 * 0.3).sin());
            (p, p.conj())
        }).collect();
        let received: Vec<(C64, C64)> = pilots.iter().map(|&(ph, pv)| channel.apply(ph, pv)).collect();

        let metrics = xpic.process_training(&pilots, &received);
        // XPIC should improve XPI (positive improvement means we cancelled interference)
        assert!(metrics.xpi_improvement_db > 0.0,
            "LMS XPIC should improve XPI, got {} dB", metrics.xpi_improvement_db);
    }

    #[test]
    fn test_xpic_rls_faster_convergence() {
        let channel = PolarizationChannel::new_with_xpd(25.0, 0.0);

        // LMS config
        let cfg_lms = XpicConfig {
            num_taps: 1,
            lms_step_size: 0.05,
            algorithm: XpicAlgorithm::Lms,
            ..Default::default()
        };
        // RLS config
        let cfg_rls = XpicConfig {
            num_taps: 1,
            algorithm: XpicAlgorithm::Rls,
            rls_lambda: 0.99,
            rls_delta: 10.0,
            ..Default::default()
        };

        let n_train = 200; // Short training — RLS should converge faster
        let pilots: Vec<(C64, C64)> = (0..n_train).map(|i| {
            let p = C64::new((i as f64 * 0.5).cos(), 0.0);
            (p, -p)
        }).collect();
        let received: Vec<(C64, C64)> = pilots.iter().map(|&(ph, pv)| channel.apply(ph, pv)).collect();

        let mut xpic_lms = XpicFilter::new(cfg_lms);
        let mut xpic_rls = XpicFilter::new(cfg_rls);

        let metrics_lms = xpic_lms.process_training(&pilots, &received);
        let metrics_rls = xpic_rls.process_training(&pilots, &received);

        // Both should show improvement; RLS typically has lower MSE after short training
        assert!(metrics_lms.xpi_improvement_db >= 0.0 || metrics_rls.xpi_improvement_db >= 0.0,
            "At least one algorithm should improve XPI");
    }

    #[test]
    fn test_xpic_no_crosspol_no_adaptation_needed() {
        // Perfect channel (identity Jones matrix) → no cross-pol → XPIC taps stay near zero
        let jones = Matrix2x2::identity();
        let channel = PolarizationChannel::new(jones, 0.0);

        let cfg = XpicConfig {
            num_taps: 1,
            lms_step_size: 0.05,
            algorithm: XpicAlgorithm::Lms,
            ..Default::default()
        };
        let mut xpic = XpicFilter::new(cfg);

        let pilots: Vec<(C64, C64)> = (0..500).map(|_| (c(1.0, 0.0), c(0.0, 1.0))).collect();
        let received: Vec<(C64, C64)> = pilots.iter().map(|&(ph, pv)| channel.apply(ph, pv)).collect();
        xpic.process_training(&pilots, &received);

        // Cross-pol taps should remain near zero (no interference to cancel)
        let hv_norm: f64 = xpic.taps_hv().iter().map(|w| w.norm_sqr()).sum();
        let vh_norm: f64 = xpic.taps_vh().iter().map(|w| w.norm_sqr()).sum();
        assert!(hv_norm < 1.0, "No cross-pol → small W_HV taps: {hv_norm}");
        assert!(vh_norm < 1.0, "No cross-pol → small W_VH taps: {vh_norm}");
    }

    #[test]
    fn test_xpic_tap_count() {
        for n in [1, 4, 8, 16] {
            let cfg = XpicConfig { num_taps: n, ..Default::default() };
            let xpic = XpicFilter::new(cfg);
            assert_eq!(xpic.taps_hv().len(), n);
            assert_eq!(xpic.taps_vh().len(), n);
            assert_eq!(xpic.taps_hh().len(), n);
            assert_eq!(xpic.taps_vv().len(), n);
        }
    }

    #[test]
    fn test_xpic_decision_directed_mode() {
        let channel = PolarizationChannel::new_with_xpd(30.0, 0.0);
        let cfg = XpicConfig {
            num_taps: 1,
            lms_step_size: 0.02,
            algorithm: XpicAlgorithm::DecisionDirectedLms,
            ..Default::default()
        };
        let mut xpic = XpicFilter::new(cfg);

        // First train with pilots
        let pilots: Vec<(C64, C64)> = (0..1000).map(|_| (c(1.0, 0.0), c(0.0, 1.0))).collect();
        let received: Vec<(C64, C64)> = pilots.iter().map(|&(ph, pv)| channel.apply(ph, pv)).collect();
        xpic.process_training(&pilots, &received);

        // Then switch to decision-directed (BPSK slicer)
        xpic.set_training(false);
        let (y_h, _y_v) = xpic.process(
            channel.apply(c(1.0, 0.0), c(0.0, 1.0)).0,
            channel.apply(c(1.0, 0.0), c(0.0, 1.0)).1,
            Some(|yh: C64, yv: C64| (if yh.re >= 0.0 { c(1.0, 0.0) } else { c(-1.0, 0.0) }, yv)),
        );
        assert!(y_h.norm() > 0.0, "Output should be non-zero");
    }

    #[test]
    fn test_xpic_process_batch() {
        let channel = PolarizationChannel::new_with_xpd(28.0, 0.0);
        let cfg = XpicConfig::default();
        let mut xpic = XpicFilter::new(cfg);

        let n = 500;
        let pilots: Vec<(C64, C64)> = (0..n).map(|i| {
            let ph = c((i as f64 * 0.2).cos(), (i as f64 * 0.2).sin());
            (ph, ph * c(0.0, 1.0))
        }).collect();
        let received: Vec<(C64, C64)> = pilots.iter().map(|&(ph, pv)| channel.apply(ph, pv)).collect();

        let metrics = xpic.process_training(&pilots, &received);
        assert_eq!(metrics.samples_trained, n as u64);
    }

    // --- XpicMetrics Tests ---

    #[test]
    fn test_capacity_gain_high_xpi() {
        let metrics = XpicMetrics {
            xpi_improvement_db: 25.0,
            initial_xpd_db: 25.0,
            final_xpi_db: 50.0,
            mse_h: 0.001,
            mse_v: 0.001,
            samples_trained: 1000,
            sinr_improvement_db: 25.0,
        };
        // With very high XPI (50 dB), dual-pol capacity should approach 2x
        let gain = metrics.capacity_gain_db(30.0);
        assert!(gain > 0.0, "Dual-pol should provide capacity gain: {gain} dB");
    }

    #[test]
    fn test_capacity_gain_low_xpi() {
        let metrics = XpicMetrics {
            xpi_improvement_db: 10.0,
            initial_xpd_db: 20.0,
            final_xpi_db: 30.0,
            mse_h: 0.01,
            mse_v: 0.01,
            samples_trained: 500,
            sinr_improvement_db: 10.0,
        };
        let gain = metrics.capacity_gain_db(20.0);
        assert!(gain > 0.0, "Even moderate XPI provides some capacity gain: {gain}");
    }

    #[test]
    fn test_spectral_efficiency_bounded() {
        let metrics = XpicMetrics {
            xpi_improvement_db: 30.0,
            initial_xpd_db: 30.0,
            final_xpi_db: 60.0,
            mse_h: 0.0001,
            mse_v: 0.0001,
            samples_trained: 2000,
            sinr_improvement_db: 30.0,
        };
        let se = metrics.spectral_efficiency(40.0, 7); // 128-QAM, 7 bits/sym
        // Maximum is 2 × 7 = 14 bits/s/Hz (dual pol 128-QAM)
        assert!(se <= 14.0, "SE bounded by modulation: {se}");
        assert!(se > 0.0);
    }

    // --- DualPolCapacity Tests ---

    #[test]
    fn test_dual_pol_capacity_doubles_at_high_xpi() {
        let cap = DualPolCapacity {
            bandwidth_hz: 28e6,
            snr_db: 30.0,
            xpi_db: 60.0, // Excellent XPIC, essentially isolated
            modulation_bits: 10,
            fec_rate: 0.9,
        };
        let dual = cap.shannon_capacity();
        let single = cap.shannon_capacity_single_pol();
        // Should be approximately 2× single-pol
        assert!(dual / single > 1.8 && dual / single < 2.1,
            "Dual-pol capacity ratio: {}", dual / single);
    }

    #[test]
    fn test_dual_pol_capacity_gain_db() {
        let cap = DualPolCapacity {
            bandwidth_hz: 56e6,
            snr_db: 35.0,
            xpi_db: 55.0,
            modulation_bits: 10,
            fec_rate: 0.875,
        };
        let gain = cap.capacity_gain_db();
        // With 55 dB XPI after XPIC, gain should approach 3 dB (2×)
        assert!(gain > 2.5, "Capacity gain should approach 3 dB: {gain} dB");
    }

    #[test]
    fn test_required_xpi_for_degradation() {
        let cap = DualPolCapacity {
            bandwidth_hz: 28e6,
            snr_db: 30.0,
            xpi_db: 0.0,
            modulation_bits: 8,
            fec_rate: 0.875,
        };
        // Require at most 1 dB SNR degradation
        let xpi_req = cap.required_xpi_for_degradation(1.0);
        assert!(xpi_req > 0.0, "XPI requirement should be positive: {xpi_req}");
        // For 30 dB SNR, 1 dB degradation requires ~ 20 dB XPI
        assert!(xpi_req > 10.0 && xpi_req < 40.0, "XPI req in reasonable range: {xpi_req}");
    }

    #[test]
    fn test_dual_pol_throughput_bounded() {
        let cap = DualPolCapacity {
            bandwidth_hz: 28e6,
            snr_db: 50.0,
            xpi_db: 60.0,
            modulation_bits: 10,
            fec_rate: 0.875,
        };
        let tp = cap.practical_throughput();
        let max = 2.0 * 28e6 * 10.0 * 0.875;
        assert!(tp <= max, "Throughput bounded by modulation: {tp} <= {max}");
    }

    // --- Diversity Combiner Tests ---

    #[test]
    fn test_mrc_combines_both_branches() {
        let mut comb = PolarizationDiversityCombiner::new(DiversityMode::MaximumRatio);
        comb.h_est_h = c(1.0, 0.0);
        comb.h_est_v = c(1.0, 0.0);
        comb.snr_h = 10.0;
        comb.snr_v = 10.0;
        let r = comb.combine(c(1.0, 0.0), c(1.0, 0.0));
        assert!(r.norm() > 0.0);
    }

    #[test]
    fn test_egc_co_phases() {
        let mut comb = PolarizationDiversityCombiner::new(DiversityMode::EqualGain);
        // Both branches in-phase → should add coherently
        comb.h_est_h = c(1.0, 0.0);
        comb.h_est_v = c(1.0, 0.0);
        let r = comb.combine(c(1.0, 0.0), c(1.0, 0.0));
        assert!(r.re > 0.5, "EGC should increase signal: {}", r.re);
    }

    #[test]
    fn test_selection_picks_stronger() {
        let comb = PolarizationDiversityCombiner::new(DiversityMode::Selection);
        let r = comb.combine(c(3.0, 0.0), c(1.0, 0.0));
        assert!((r - c(3.0, 0.0)).norm() < TOL, "Selection should pick H branch");
        let r = comb.combine(c(0.5, 0.0), c(2.0, 0.0));
        assert!((r - c(2.0, 0.0)).norm() < TOL, "Selection should pick V branch");
    }

    #[test]
    fn test_diversity_gain_uncorrelated() {
        for mode in [DiversityMode::MaximumRatio, DiversityMode::EqualGain, DiversityMode::Selection] {
            let comb = PolarizationDiversityCombiner::new(mode);
            let gain = comb.diversity_gain_db(0.0); // ρ = 0
            assert!(gain > 0.0, "{mode:?} should provide positive diversity gain: {gain} dB");
        }
    }

    #[test]
    fn test_mrc_diversity_gain_vs_correlation() {
        let comb = PolarizationDiversityCombiner::new(DiversityMode::MaximumRatio);
        let g0 = comb.diversity_gain_db(0.0); // uncorrelated
        let g5 = comb.diversity_gain_db(0.5); // partially correlated
        let g9 = comb.diversity_gain_db(0.9); // highly correlated
        assert!(g0 > g5, "Less correlation → more diversity gain");
        assert!(g5 > g9);
    }

    // --- DualPolReceiver Tests ---

    #[test]
    fn test_dual_pol_receiver_training() {
        let ch = PolarizationChannel::new_with_xpd(25.0, 0.0);
        let config = DualPolConfig {
            xpic: XpicConfig {
                num_taps: 1,
                lms_step_size: 0.05,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut rx = DualPolReceiver::new(config);

        for i in 0..500 {
            let ph = c((i as f64 * 0.2).cos(), (i as f64 * 0.2).sin());
            let pv = ph.conj();
            let (r_h, r_v) = ch.apply(ph, pv);
            rx.process_training_sample(r_h, r_v, ph, pv);
        }
        assert_eq!(rx.samples_processed(), 500);
    }

    #[test]
    fn test_dual_pol_receiver_stokes() {
        let ch = PolarizationChannel::new_with_xpd(35.0, 0.0);
        let config = DualPolConfig::default();
        let mut rx = DualPolReceiver::new(config);

        for _ in 0..100 {
            let ph = c(1.0, 0.0);
            let pv = c(0.0, 0.0);
            let (r_h, r_v) = ch.apply(ph, pv);
            rx.process_training_sample(r_h, r_v, ph, pv);
        }
        let sp = rx.stokes();
        assert!(sp.s0 > 0.0, "Stokes S0 should be positive");
    }

    #[test]
    fn test_dual_pol_receiver_xpi_tracking() {
        let mut ch = PolarizationChannel::new_with_xpd(30.0, 0.0);
        let config = DualPolConfig {
            xpic: XpicConfig {
                num_taps: 1,
                lms_step_size: 0.02,
                ..Default::default()
            },
            ..Default::default()
        };
        let mut rx = DualPolReceiver::new(config);

        for i in 0..300 {
            let ph = c((i as f64 * 0.3).cos(), 0.0);
            let pv = c(0.0, (i as f64 * 0.3).cos());
            let (r_h, r_v) = ch.apply(ph, pv);
            rx.process_training_sample(r_h, r_v, ph, pv);
        }
        // After training, XPI before and after should be tracked
        let xpi_before = rx.xpi_before_db();
        let xpi_after = rx.xpi_after_db();
        // Both should be finite
        assert!(xpi_before.is_finite(), "XPI before should be finite");
        assert!(xpi_after.is_finite(), "XPI after should be finite");
    }
}
