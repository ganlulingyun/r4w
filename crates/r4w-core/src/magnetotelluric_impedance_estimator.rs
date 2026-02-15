//! # Magnetotelluric Impedance Estimator
//!
//! Magnetotellurics (MT) is a passive geophysical method that uses natural
//! electromagnetic field variations (from lightning, solar wind, and
//! magnetospheric currents) to probe the electrical conductivity structure
//! of the Earth's subsurface.
//!
//! ## Physical Basis
//!
//! Time-varying magnetic fields induce telluric (electric) currents in the
//! conductive Earth. By measuring orthogonal electric (Ex, Ey) and magnetic
//! (Hx, Hy) field components at the surface, the impedance tensor Z is
//! estimated from the relation:
//!
//! ```text
//!   [ Ex ]   [ Zxx  Zxy ] [ Hx ]
//!   [    ] = [          ] [    ]
//!   [ Ey ]   [ Zyx  Zyy ] [ Hy ]
//! ```
//!
//! The impedance tensor encodes information about subsurface resistivity
//! structure at depths controlled by the electromagnetic skin depth:
//!
//! ```text
//!   delta = sqrt(2 * rho / (omega * mu_0))
//! ```
//!
//! where rho is resistivity, omega = 2*pi*f, and mu_0 is the permeability
//! of free space.
//!
//! ## Apparent Resistivity and Phase
//!
//! From each impedance element, apparent resistivity and phase are computed:
//!
//! ```text
//!   rho_a = (1 / (omega * mu_0)) * |Z|^2
//!   phi   = atan2(Im(Z), Re(Z))
//! ```
//!
//! For a uniform half-space, rho_a equals the true resistivity and
//! phi = 45 degrees at all frequencies.
//!
//! ## Dimensionality Analysis
//!
//! The impedance tensor structure reveals subsurface dimensionality:
//! - **1D**: Zxx = Zyy = 0, Zxy = -Zyx (layered Earth)
//! - **2D**: Zxx = Zyy = 0 when rotated to strike direction
//! - **3D**: All elements non-zero, Swift skew > 0.1
//!
//! ## Tipper (Vertical Transfer Function)
//!
//! The vertical magnetic field Hz relates to horizontal components via
//! the tipper vector (Tx, Ty):
//!
//! ```text
//!   Hz = Tx * Hx + Ty * Hy
//! ```
//!
//! Induction arrows (Parkinson convention) point toward conductive bodies.
//!
//! ## References
//!
//! - Chave, A.D. & Jones, A.G. (2012). The Magnetotelluric Method:
//!   Theory and Practice. Cambridge University Press.
//! - Simpson, F. & Bahr, K. (2005). Practical Magnetotellurics.
//!   Cambridge University Press.
//! - Vozoff, K. (1991). The Magnetotelluric Method. In Electromagnetic
//!   Methods in Applied Geophysics, Vol. 2, SEG.

use std::f64::consts::PI;

// ──────────────────────────────────────────────────────────────────────
// Complex arithmetic helpers (pure f64 tuples, no external deps)
// ──────────────────────────────────────────────────────────────────────

/// Multiply two complex numbers represented as (re, im) tuples.
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Divide complex a by complex b: a / b.
///
/// Returns (0, 0) if b is zero to avoid NaN propagation.
pub fn complex_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom == 0.0 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex conjugate: (re, -im).
pub fn complex_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude |z| = sqrt(re^2 + im^2).
pub fn complex_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex addition.
fn complex_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
fn complex_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex negation.
fn complex_neg(a: (f64, f64)) -> (f64, f64) {
    (-a.0, -a.1)
}

/// Squared magnitude |z|^2 = re^2 + im^2.
fn complex_abs_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Permeability of free space mu_0 = 4*pi*1e-7 H/m.
pub fn mu_0() -> f64 {
    4.0 * PI * 1e-7
}

// ──────────────────────────────────────────────────────────────────────
// Robust estimation method
// ──────────────────────────────────────────────────────────────────────

/// Robust estimation method for impedance tensor estimation.
///
/// MT data often contains outliers from cultural noise, power line
/// harmonics, or magnetospheric pulsations. Robust methods downweight
/// these outliers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RobustMethod {
    /// Standard ordinary least squares (no robustness).
    OrdinaryLeastSquares,
    /// Huber M-estimator: downweights residuals exceeding a threshold
    /// with linear (instead of quadratic) penalty.
    HuberWeight,
    /// Thompson M-estimator: uses a weight function that more aggressively
    /// rejects large residuals.
    ThompsonWeight,
}

// ──────────────────────────────────────────────────────────────────────
// Configuration
// ──────────────────────────────────────────────────────────────────────

/// Configuration for magnetotelluric processing.
#[derive(Debug, Clone)]
pub struct MtConfig {
    /// Sampling rate of the time-series data (Hz).
    pub sample_rate_hz: f64,
    /// Number of evaluation frequencies for the impedance tensor.
    pub num_frequencies: usize,
    /// Frequency range (min_hz, max_hz) for evaluation.
    pub frequency_range: (f64, f64),
    /// Robust estimation method.
    pub robustness: RobustMethod,
    /// Whether to use a remote reference station for noise rejection.
    pub remote_reference: bool,
}

impl Default for MtConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 256.0,
            num_frequencies: 20,
            frequency_range: (0.001, 100.0),
            robustness: RobustMethod::OrdinaryLeastSquares,
            remote_reference: false,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────
// Impedance tensor
// ──────────────────────────────────────────────────────────────────────

/// The 2x2 complex impedance tensor relating E and H field components.
///
/// The tensor elements have units of mV/km/nT (or equivalently Ohm,
/// depending on normalization). In SI units, Z has units of V/m / (A/m) = Ohm.
#[derive(Debug, Clone)]
pub struct ImpedanceTensor {
    /// Zxx component (complex: real, imaginary).
    pub zxx: (f64, f64),
    /// Zxy component (complex: real, imaginary).
    pub zxy: (f64, f64),
    /// Zyx component (complex: real, imaginary).
    pub zyx: (f64, f64),
    /// Zyy component (complex: real, imaginary).
    pub zyy: (f64, f64),
    /// Frequency at which this tensor was evaluated (Hz).
    pub frequency_hz: f64,
}

impl ImpedanceTensor {
    /// Create a new impedance tensor.
    pub fn new(
        zxx: (f64, f64),
        zxy: (f64, f64),
        zyx: (f64, f64),
        zyy: (f64, f64),
        frequency_hz: f64,
    ) -> Self {
        Self { zxx, zxy, zyx, zyy, frequency_hz }
    }

    /// Determinant of the impedance tensor: det(Z) = Zxx*Zyy - Zxy*Zyx.
    pub fn determinant(&self) -> (f64, f64) {
        complex_sub(complex_mul(self.zxx, self.zyy), complex_mul(self.zxy, self.zyx))
    }

    /// Trace of the impedance tensor: Zxx + Zyy.
    pub fn trace(&self) -> (f64, f64) {
        complex_add(self.zxx, self.zyy)
    }
}

// ──────────────────────────────────────────────────────────────────────
// Tipper (vertical magnetic transfer function)
// ──────────────────────────────────────────────────────────────────────

/// The tipper or vertical magnetic transfer function.
///
/// Relates the vertical magnetic field Hz to horizontal components
/// via Hz = Tx*Hx + Ty*Hy. Non-zero tipper indicates lateral
/// conductivity contrasts.
#[derive(Debug, Clone)]
pub struct Tipper {
    /// Tx component (complex: real, imaginary).
    pub tx: (f64, f64),
    /// Ty component (complex: real, imaginary).
    pub ty: (f64, f64),
    /// Frequency at which this tipper was evaluated (Hz).
    pub frequency_hz: f64,
}

// ──────────────────────────────────────────────────────────────────────
// MT Processor
// ──────────────────────────────────────────────────────────────────────

/// Main processor for magnetotelluric impedance estimation.
///
/// Performs spectral analysis, cross-spectral estimation, and impedance
/// tensor computation from measured E and H field time series.
pub struct MtProcessor {
    config: MtConfig,
}

impl MtProcessor {
    /// Create a new MT processor with the given configuration.
    pub fn new(config: MtConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &MtConfig {
        &self.config
    }

    /// Compute the DFT of a real-valued time series.
    ///
    /// Returns a vector of (frequency_hz, (re, im)) pairs for
    /// non-negative frequencies only (DC to Nyquist).
    ///
    /// Uses a straightforward DFT implementation (O(N^2)) suitable
    /// for the moderate lengths typical of MT processing windows.
    pub fn compute_fft(signal: &[f64], sample_rate: f64) -> Vec<(f64, (f64, f64))> {
        let n = signal.len();
        if n == 0 {
            return Vec::new();
        }
        let n_out = n / 2 + 1;
        let mut result = Vec::with_capacity(n_out);
        let nf = n as f64;

        for k in 0..n_out {
            let mut re = 0.0;
            let mut im = 0.0;
            for (j, &s) in signal.iter().enumerate() {
                let angle = -2.0 * PI * (k as f64) * (j as f64) / nf;
                re += s * angle.cos();
                im += s * angle.sin();
            }
            let freq = (k as f64) * sample_rate / nf;
            result.push((freq, (re, im)));
        }
        result
    }

    /// Compute the cross-spectrum <A * conj(B)> element-wise.
    ///
    /// Both input slices must have the same length. Each element is
    /// the complex product a * conj(b).
    pub fn cross_spectrum(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
        a.iter()
            .zip(b.iter())
            .map(|(&av, &bv)| complex_mul(av, complex_conj(bv)))
            .collect()
    }

    /// Compute the auto-spectrum <|A|^2> element-wise.
    pub fn auto_spectrum(a: &[(f64, f64)]) -> Vec<f64> {
        a.iter().map(|&v| complex_abs_sq(v)).collect()
    }

    /// Estimate the impedance tensor at a single frequency from E and H
    /// time-series data.
    ///
    /// Solves E = Z * H in the frequency domain via the standard
    /// cross-spectral method with multi-window (Welch-like) averaging:
    ///
    /// ```text
    ///   Zxy = (<Ex * Hy*> <Hy * Hy*> - <Ex * Hx*> <Hx * Hy*>)
    ///         / (<Hx * Hx*> <Hy * Hy*> - <Hx * Hy*> <Hy * Hx*>)
    /// ```
    ///
    /// where `<>` denotes averaging over multiple data segments.
    ///
    /// The data is split into overlapping segments (50% overlap), and
    /// cross-spectral densities are averaged to produce a robust estimate.
    /// This averaging is essential because a single-window estimate yields
    /// a singular spectral matrix (by the Cauchy-Schwarz identity).
    ///
    /// The `freq_hz` parameter selects the nearest DFT bin.
    pub fn estimate_impedance(
        &self,
        ex: &[f64],
        ey: &[f64],
        hx: &[f64],
        hy: &[f64],
        freq_hz: f64,
    ) -> ImpedanceTensor {
        let sr = self.config.sample_rate_hz;
        let n = ex.len().min(ey.len()).min(hx.len()).min(hy.len());

        // Choose segment length: target ~8 segments with 50% overlap
        // segment_len such that (n - segment_len) / (segment_len/2) + 1 >= 4
        let segment_len = if n >= 32 { n / 4 } else { n };
        let hop = segment_len / 2;

        // Collect segment start indices
        let mut starts = Vec::new();
        let mut pos = 0;
        while pos + segment_len <= n {
            starts.push(pos);
            pos += hop.max(1);
        }
        if starts.is_empty() {
            starts.push(0);
        }

        // Accumulate averaged cross-spectral densities at the target bin
        let mut avg_sxx = (0.0, 0.0);
        let mut avg_syy = (0.0, 0.0);
        let mut avg_sxy = (0.0, 0.0);
        let mut avg_syx = (0.0, 0.0);
        let mut avg_ex_hx = (0.0, 0.0);
        let mut avg_ex_hy = (0.0, 0.0);
        let mut avg_ey_hx = (0.0, 0.0);
        let mut avg_ey_hy = (0.0, 0.0);
        let mut actual_freq = freq_hz;

        for &start in &starts {
            let end = start + segment_len;
            let ex_seg = &ex[start..end];
            let ey_seg = &ey[start..end];
            let hx_seg = &hx[start..end];
            let hy_seg = &hy[start..end];

            let ex_fft = Self::compute_fft(ex_seg, sr);
            let ey_fft = Self::compute_fft(ey_seg, sr);
            let hx_fft = Self::compute_fft(hx_seg, sr);
            let hy_fft = Self::compute_fft(hy_seg, sr);

            let bin = Self::nearest_bin(&ex_fft, freq_hz);
            actual_freq = ex_fft[bin].0;

            let ex_f = ex_fft[bin].1;
            let ey_f = ey_fft[bin].1;
            let hx_f = hx_fft[bin].1;
            let hy_f = hy_fft[bin].1;

            avg_sxx = complex_add(avg_sxx, complex_mul(hx_f, complex_conj(hx_f)));
            avg_syy = complex_add(avg_syy, complex_mul(hy_f, complex_conj(hy_f)));
            avg_sxy = complex_add(avg_sxy, complex_mul(hx_f, complex_conj(hy_f)));
            avg_syx = complex_add(avg_syx, complex_mul(hy_f, complex_conj(hx_f)));
            avg_ex_hx = complex_add(avg_ex_hx, complex_mul(ex_f, complex_conj(hx_f)));
            avg_ex_hy = complex_add(avg_ex_hy, complex_mul(ex_f, complex_conj(hy_f)));
            avg_ey_hx = complex_add(avg_ey_hx, complex_mul(ey_f, complex_conj(hx_f)));
            avg_ey_hy = complex_add(avg_ey_hy, complex_mul(ey_f, complex_conj(hy_f)));
        }

        // Denominator: D = <Hx Hx*><Hy Hy*> - <Hx Hy*><Hy Hx*>
        let denom = complex_sub(
            complex_mul(avg_sxx, avg_syy),
            complex_mul(avg_sxy, avg_syx),
        );

        // E_x = Zxx * Hx + Zxy * Hy
        // Cramer's rule:
        //   Zxx = (<Ex Hx*><Hy Hy*> - <Ex Hy*><Hy Hx*>) / D
        //   Zxy = (<Ex Hy*><Hx Hx*> - <Ex Hx*><Hx Hy*>) / D
        let zxx = complex_div(
            complex_sub(complex_mul(avg_ex_hx, avg_syy), complex_mul(avg_ex_hy, avg_syx)),
            denom,
        );
        let zxy = complex_div(
            complex_sub(complex_mul(avg_ex_hy, avg_sxx), complex_mul(avg_ex_hx, avg_sxy)),
            denom,
        );

        // E_y = Zyx * Hx + Zyy * Hy
        let zyx = complex_div(
            complex_sub(complex_mul(avg_ey_hx, avg_syy), complex_mul(avg_ey_hy, avg_syx)),
            denom,
        );
        let zyy = complex_div(
            complex_sub(complex_mul(avg_ey_hy, avg_sxx), complex_mul(avg_ey_hx, avg_sxy)),
            denom,
        );

        ImpedanceTensor::new(zxx, zxy, zyx, zyy, actual_freq)
    }

    /// Compute apparent resistivity from an impedance element.
    ///
    /// ```text
    ///   rho_a = (1 / (omega * mu_0)) * |Z|^2
    /// ```
    ///
    /// Units: Z in Ohm (V/m per A/m), result in Ohm*m.
    pub fn apparent_resistivity(z: &(f64, f64), freq_hz: f64) -> f64 {
        let omega = 2.0 * PI * freq_hz;
        let z_sq = complex_abs_sq(*z);
        z_sq / (omega * mu_0())
    }

    /// Compute the phase angle of an impedance element in degrees.
    ///
    /// ```text
    ///   phi = atan2(Im(Z), Re(Z)) * (180 / pi)
    /// ```
    ///
    /// For a uniform half-space, phi = 45 degrees.
    pub fn phase_angle(z: &(f64, f64)) -> f64 {
        z.1.atan2(z.0) * (180.0 / PI)
    }

    /// Compute the electromagnetic skin depth.
    ///
    /// ```text
    ///   delta = sqrt(2 * rho / (omega * mu_0))
    /// ```
    ///
    /// At 1 Hz in 100 Ohm*m material, delta ~ 503 m.
    pub fn skin_depth(resistivity_ohm_m: f64, frequency_hz: f64) -> f64 {
        let omega = 2.0 * PI * frequency_hz;
        (2.0 * resistivity_ohm_m / (omega * mu_0())).sqrt()
    }

    /// Cagniard apparent resistivity for a 1D half-space.
    ///
    /// For a uniform half-space of true resistivity `rho_true`, the
    /// apparent resistivity equals `rho_true` at all frequencies. This
    /// function provides the forward model result for a uniform
    /// half-space, useful as a reference:
    ///
    /// ```text
    ///   rho_a = rho_true   (uniform half-space)
    /// ```
    ///
    /// For a layered model, the Cagniard resistivity varies with
    /// frequency depending on the layer structure. A simple two-layer
    /// model is implemented here: when depth_m > 0, the influence of
    /// a deeper layer is approximated.
    ///
    /// For `depth_m == 0`, returns the true resistivity (uniform case).
    pub fn cagniard_resistivity(rho_true: f64, depth_m: f64, freq_hz: f64) -> f64 {
        if depth_m <= 0.0 {
            // Uniform half-space: apparent = true at all frequencies
            return rho_true;
        }
        // For a two-layer model approximation, compute the ratio of
        // skin depth to layer depth. When skin depth << depth, the
        // signal only sees the top layer.
        let delta = Self::skin_depth(rho_true, freq_hz);
        let ratio = depth_m / delta;

        // Simple exponential transition model
        // At high freq (small delta): rho_a -> rho_true
        // At low freq (large delta): rho_a departs from rho_true
        rho_true * (1.0 + ratio * (-2.0 * ratio).exp())
    }

    /// Find the DFT bin index nearest to a target frequency.
    fn nearest_bin(fft: &[(f64, (f64, f64))], target_hz: f64) -> usize {
        let mut best = 0;
        let mut best_dist = f64::MAX;
        for (i, &(f, _)) in fft.iter().enumerate() {
            let d = (f - target_hz).abs();
            if d < best_dist {
                best_dist = d;
                best = i;
            }
        }
        best
    }
}

// ──────────────────────────────────────────────────────────────────────
// Tipper estimator
// ──────────────────────────────────────────────────────────────────────

/// Estimates the tipper (vertical magnetic transfer function).
pub struct TipperEstimator;

impl TipperEstimator {
    /// Estimate the tipper from Hz, Hx, and Hy time series.
    ///
    /// Solves Hz = Tx*Hx + Ty*Hy in the frequency domain using the
    /// same cross-spectral approach as impedance estimation.
    pub fn estimate_tipper(
        hz: &[f64],
        hx: &[f64],
        hy: &[f64],
        freq_hz: f64,
        sample_rate: f64,
    ) -> Tipper {
        let hz_fft = MtProcessor::compute_fft(hz, sample_rate);
        let hx_fft = MtProcessor::compute_fft(hx, sample_rate);
        let hy_fft = MtProcessor::compute_fft(hy, sample_rate);

        let bin = MtProcessor::nearest_bin(&hz_fft, freq_hz);

        let hz_f = hz_fft[bin].1;
        let hx_f = hx_fft[bin].1;
        let hy_f = hy_fft[bin].1;

        // Cross-spectra
        let sxx = complex_mul(hx_f, complex_conj(hx_f));
        let syy = complex_mul(hy_f, complex_conj(hy_f));
        let sxy = complex_mul(hx_f, complex_conj(hy_f));
        let syx = complex_mul(hy_f, complex_conj(hx_f));

        let hz_hx = complex_mul(hz_f, complex_conj(hx_f));
        let hz_hy = complex_mul(hz_f, complex_conj(hy_f));

        let denom = complex_sub(complex_mul(sxx, syy), complex_mul(sxy, syx));

        let tx = complex_div(
            complex_sub(complex_mul(hz_hx, syy), complex_mul(hz_hy, syx)),
            denom,
        );
        let ty = complex_div(
            complex_sub(complex_mul(hz_hy, sxx), complex_mul(hz_hx, sxy)),
            denom,
        );

        let actual_freq = hz_fft[bin].0;

        Tipper { tx, ty, frequency_hz: actual_freq }
    }

    /// Compute the real induction arrow (Parkinson convention).
    ///
    /// The real induction arrow points toward conductive anomalies.
    /// Components are (Re(Tx), Re(Ty)).
    pub fn induction_arrow_real(tipper: &Tipper) -> (f64, f64) {
        (tipper.tx.0, tipper.ty.0)
    }

    /// Compute the imaginary induction arrow.
    ///
    /// Components are (Im(Tx), Im(Ty)).
    pub fn induction_arrow_imaginary(tipper: &Tipper) -> (f64, f64) {
        (tipper.tx.1, tipper.ty.1)
    }

    /// Compute the tipper magnitude: sqrt(|Tx|^2 + |Ty|^2).
    pub fn tipper_magnitude(tipper: &Tipper) -> f64 {
        (complex_abs_sq(tipper.tx) + complex_abs_sq(tipper.ty)).sqrt()
    }
}

// ──────────────────────────────────────────────────────────────────────
// Dimensionality analyzer
// ──────────────────────────────────────────────────────────────────────

/// Analyzes the dimensionality of the impedance tensor to determine
/// whether the subsurface structure is 1D, 2D, or 3D.
pub struct DimensionalityAnalyzer;

impl DimensionalityAnalyzer {
    /// Swift skew: |Zxx + Zyy| / |Zxy - Zyx|.
    ///
    /// For a 1D or 2D Earth, the diagonal elements satisfy
    /// Zxx + Zyy = 0, giving a skew of 0. Skew > 0.1-0.3
    /// typically indicates 3D structure.
    pub fn swift_skew(z: &ImpedanceTensor) -> f64 {
        let num = complex_abs(complex_add(z.zxx, z.zyy));
        let den = complex_abs(complex_sub(z.zxy, z.zyx));
        if den == 0.0 {
            return 0.0;
        }
        num / den
    }

    /// Bahr (regional) skew: sqrt(|D1 * D2 - S1 * S2|) / |D2|.
    ///
    /// Uses the decomposition:
    /// - S1 = (Zxx + Zyy) / 2   (mean diagonal)
    /// - S2 = (Zxy + Zyx) / 2   (mean off-diagonal)
    /// - D1 = (Zxx - Zyy) / 2   (diff diagonal)
    /// - D2 = (Zxy - Zyx) / 2   (diff off-diagonal)
    ///
    /// Skew < 0.1 suggests 1D/2D, 0.1-0.3 is weak 3D, >0.3 is 3D.
    pub fn bahr_skew(z: &ImpedanceTensor) -> f64 {
        let half = (0.5, 0.0);
        let s1 = complex_mul(complex_add(z.zxx, z.zyy), half);
        let s2 = complex_mul(complex_add(z.zxy, z.zyx), half);
        let d1 = complex_mul(complex_sub(z.zxx, z.zyy), half);
        let d2 = complex_mul(complex_sub(z.zxy, z.zyx), half);

        let cross = complex_sub(complex_mul(d1, d2), complex_mul(s1, s2));
        let num = complex_abs(cross).sqrt();
        let den = complex_abs(d2);
        if den == 0.0 {
            return 0.0;
        }
        num / den
    }

    /// Estimate the geoelectric strike angle from the impedance tensor.
    ///
    /// The strike angle theta is found by minimizing |Zxx'| + |Zyy'|
    /// (the diagonal elements in the rotated frame). For 2D structures,
    /// this rotation makes Zxx' = Zyy' = 0.
    ///
    /// Uses the Swift (1967) formula:
    /// ```text
    ///   tan(4*theta) = 2*Re((Zxx-Zyy)*conj(Zxy+Zyx))
    ///                  / (|Zxx-Zyy|^2 - |Zxy+Zyx|^2)
    /// ```
    ///
    /// Returns angle in degrees, range [-45, 45].
    pub fn strike_angle(z: &ImpedanceTensor) -> f64 {
        let d = complex_sub(z.zxx, z.zyy);
        let s = complex_add(z.zxy, z.zyx);
        let num = 2.0 * complex_mul(d, complex_conj(s)).0; // 2 * Re(d * conj(s))
        let den = complex_abs_sq(d) - complex_abs_sq(s);
        let angle4 = num.atan2(den);
        let angle = angle4 / 4.0;
        angle * (180.0 / PI)
    }

    /// Rotate the impedance tensor by a given angle (degrees).
    ///
    /// The rotation follows the standard coordinate rotation:
    /// ```text
    ///   Z' = R * Z * R^T
    /// ```
    /// where R is the 2D rotation matrix.
    pub fn rotate_impedance(z: &ImpedanceTensor, angle_deg: f64) -> ImpedanceTensor {
        let theta = angle_deg * (PI / 180.0);
        let c = theta.cos();
        let s = theta.sin();

        // R = [[c, s], [-s, c]]
        // Z' = R Z R^T
        //
        // Z'xx =  c^2 * Zxx + c*s * Zxy + c*s * Zyx + s^2 * Zyy
        //       =  c^2 Zxx + cs(Zxy + Zyx) + s^2 Zyy
        // Z'xy =  -cs * Zxx + c^2 * Zxy - s^2 * Zyx + cs * Zyy
        // Z'yx =  -cs * Zxx - s^2 * Zxy + c^2 * Zyx + cs * Zyy
        // Z'yy =  s^2 * Zxx - cs * Zxy - cs * Zyx + c^2 * Zyy
        //       =  s^2 Zxx - cs(Zxy + Zyx) + c^2 Zyy

        let cc = c * c;
        let ss = s * s;
        let cs = c * s;

        let scale = |z_elem: (f64, f64), factor: f64| -> (f64, f64) {
            (z_elem.0 * factor, z_elem.1 * factor)
        };

        let zxx_p = complex_add(
            complex_add(scale(z.zxx, cc), scale(complex_add(z.zxy, z.zyx), cs)),
            scale(z.zyy, ss),
        );
        let zxy_p = complex_add(
            complex_add(scale(z.zxx, -cs), scale(z.zxy, cc)),
            complex_add(scale(z.zyx, -ss), scale(z.zyy, cs)),
        );
        let zyx_p = complex_add(
            complex_add(scale(z.zxx, -cs), scale(z.zxy, -ss)),
            complex_add(scale(z.zyx, cc), scale(z.zyy, cs)),
        );
        let zyy_p = complex_add(
            complex_sub(scale(z.zxx, ss), scale(complex_add(z.zxy, z.zyx), cs)),
            scale(z.zyy, cc),
        );

        ImpedanceTensor::new(zxx_p, zxy_p, zyx_p, zyy_p, z.frequency_hz)
    }

    /// Test whether the impedance tensor is consistent with 1D structure.
    ///
    /// For a 1D (layered) Earth:
    /// - Zxx ~ 0
    /// - Zyy ~ 0
    /// - Zxy ~ -Zyx
    ///
    /// The `threshold` controls the relative tolerance (e.g., 0.1 = 10%).
    pub fn is_1d(z: &ImpedanceTensor, threshold: f64) -> bool {
        let off_diag_scale = (complex_abs(z.zxy) + complex_abs(z.zyx)) / 2.0;
        if off_diag_scale == 0.0 {
            return true;
        }

        // Check diagonal elements are small relative to off-diagonal
        let diag_ratio =
            (complex_abs(z.zxx) + complex_abs(z.zyy)) / (2.0 * off_diag_scale);
        if diag_ratio > threshold {
            return false;
        }

        // Check Zxy ~ -Zyx
        let anti_sym = complex_abs(complex_add(z.zxy, z.zyx));
        let anti_ratio = anti_sym / off_diag_scale;
        anti_ratio < threshold
    }

    /// Test whether the impedance tensor is consistent with 2D structure.
    ///
    /// A 2D Earth has Swift skew < threshold (typically 0.1-0.3).
    pub fn is_2d(z: &ImpedanceTensor, threshold: f64) -> bool {
        Self::swift_skew(z) < threshold
    }
}

// ──────────────────────────────────────────────────────────────────────
// Tests
// ──────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 0.05; // 5% tolerance for spectral methods

    /// Helper: assert two f64 values are approximately equal.
    fn assert_approx(a: f64, b: f64, tol: f64, msg: &str) {
        assert!(
            (a - b).abs() < tol,
            "{}: expected {}, got {}, diff {}",
            msg,
            b,
            a,
            (a - b).abs()
        );
    }

    /// Helper: assert complex values are approximately equal.
    fn assert_complex_approx(a: (f64, f64), b: (f64, f64), tol: f64, msg: &str) {
        assert!(
            (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol,
            "{}: expected ({}, {}), got ({}, {})",
            msg,
            b.0,
            b.1,
            a.0,
            a.1,
        );
    }

    // ── Complex arithmetic tests ─────────────────────────────────────

    #[test]
    fn test_complex_mul_real() {
        let r = complex_mul((3.0, 0.0), (4.0, 0.0));
        assert_complex_approx(r, (12.0, 0.0), TOL, "real * real");
    }

    #[test]
    fn test_complex_mul_imaginary() {
        // i * i = -1
        let r = complex_mul((0.0, 1.0), (0.0, 1.0));
        assert_complex_approx(r, (-1.0, 0.0), TOL, "i * i = -1");
    }

    #[test]
    fn test_complex_mul_general() {
        // (1+2i)*(3+4i) = 3+4i+6i+8i^2 = -5+10i
        let r = complex_mul((1.0, 2.0), (3.0, 4.0));
        assert_complex_approx(r, (-5.0, 10.0), TOL, "(1+2i)(3+4i)");
    }

    #[test]
    fn test_complex_div() {
        // (1+2i)/(1+2i) = 1
        let r = complex_div((1.0, 2.0), (1.0, 2.0));
        assert_complex_approx(r, (1.0, 0.0), TOL, "z/z = 1");
    }

    #[test]
    fn test_complex_div_by_real() {
        let r = complex_div((6.0, 4.0), (2.0, 0.0));
        assert_complex_approx(r, (3.0, 2.0), TOL, "(6+4i)/2");
    }

    #[test]
    fn test_complex_div_by_zero() {
        let r = complex_div((1.0, 2.0), (0.0, 0.0));
        assert_complex_approx(r, (0.0, 0.0), TOL, "div by zero");
    }

    #[test]
    fn test_complex_conj() {
        assert_complex_approx(complex_conj((3.0, -4.0)), (3.0, 4.0), TOL, "conj");
    }

    #[test]
    fn test_complex_abs() {
        assert_approx(complex_abs((3.0, 4.0)), 5.0, TOL, "|3+4i|=5");
    }

    #[test]
    fn test_mu_0_value() {
        let expected = 4.0 * PI * 1e-7;
        assert_approx(mu_0(), expected, 1e-20, "mu_0");
    }

    // ── Apparent resistivity tests ───────────────────────────────────

    #[test]
    fn test_apparent_resistivity_uniform_halfspace() {
        // For a 100 Ohm*m half-space at 1 Hz:
        //   Z = sqrt(omega * mu_0 * rho) * (1+i)/sqrt(2)
        // => |Z|^2 = omega * mu_0 * rho
        // => rho_a = |Z|^2 / (omega * mu_0) = rho
        let rho = 100.0;
        let freq = 1.0;
        let omega = 2.0 * PI * freq;
        let z_mag = (omega * mu_0() * rho).sqrt();
        // Z for half-space has phase 45 degrees: Z = |Z| * (cos45 + i*sin45)
        let z = (z_mag * (PI / 4.0).cos(), z_mag * (PI / 4.0).sin());
        let rho_a = MtProcessor::apparent_resistivity(&z, freq);
        assert_approx(rho_a, rho, 1e-6, "rho_a for uniform half-space");
    }

    #[test]
    fn test_apparent_resistivity_scales_with_impedance() {
        // Double Z magnitude => 4x resistivity
        let z1 = (1.0, 0.0);
        let z2 = (2.0, 0.0);
        let freq = 1.0;
        let r1 = MtProcessor::apparent_resistivity(&z1, freq);
        let r2 = MtProcessor::apparent_resistivity(&z2, freq);
        assert_approx(r2 / r1, 4.0, TOL, "rho_a scales as |Z|^2");
    }

    #[test]
    fn test_apparent_resistivity_positive() {
        let z = (-0.5, 0.3);
        let rho = MtProcessor::apparent_resistivity(&z, 10.0);
        assert!(rho > 0.0, "rho_a must be positive");
    }

    // ── Phase angle tests ────────────────────────────────────────────

    #[test]
    fn test_phase_45_degrees() {
        // Equal real and imaginary parts => 45 degrees
        let z = (1.0, 1.0);
        let phi = MtProcessor::phase_angle(&z);
        assert_approx(phi, 45.0, TOL, "phase for Z = 1+i");
    }

    #[test]
    fn test_phase_range_normal_mt() {
        // Normal MT responses have phase between 0 and 90 degrees
        let z = (0.5, 1.0);
        let phi = MtProcessor::phase_angle(&z);
        assert!(phi > 0.0 && phi < 90.0, "Phase {} out of normal range", phi);
    }

    #[test]
    fn test_phase_0_degrees() {
        let z = (1.0, 0.0);
        let phi = MtProcessor::phase_angle(&z);
        assert_approx(phi, 0.0, TOL, "phase for purely real Z");
    }

    #[test]
    fn test_phase_90_degrees() {
        let z = (0.0, 1.0);
        let phi = MtProcessor::phase_angle(&z);
        assert_approx(phi, 90.0, TOL, "phase for purely imaginary Z");
    }

    // ── Skin depth tests ─────────────────────────────────────────────

    #[test]
    fn test_skin_depth_1_ohm_m_1hz() {
        // Classic reference: delta ~ 503 m for 1 Ohm*m at 1 Hz
        // delta = sqrt(2*rho / (omega * mu_0))
        //       = sqrt(2*1 / (2*pi * 4*pi*1e-7))
        //       ~ 503.3 m
        let delta = MtProcessor::skin_depth(1.0, 1.0);
        assert!(
            (delta - 503.3).abs() < 1.0,
            "Skin depth at 1 Ohm*m, 1 Hz: expected ~503 m, got {} m",
            delta
        );
    }

    #[test]
    fn test_skin_depth_100_ohm_m_1hz() {
        // For 100 Ohm*m at 1 Hz: delta = 503.3 * sqrt(100) ~ 5033 m
        let delta = MtProcessor::skin_depth(100.0, 1.0);
        assert!(
            (delta - 5032.9).abs() < 1.0,
            "Skin depth at 100 Ohm*m, 1 Hz: expected ~5033 m, got {} m",
            delta
        );
    }

    #[test]
    fn test_skin_depth_increases_with_resistivity() {
        let d1 = MtProcessor::skin_depth(100.0, 1.0);
        let d2 = MtProcessor::skin_depth(400.0, 1.0);
        assert_approx(d2 / d1, 2.0, 0.01, "skin depth ratio for 4x resistivity");
    }

    #[test]
    fn test_skin_depth_decreases_with_frequency() {
        let d1 = MtProcessor::skin_depth(100.0, 1.0);
        let d4 = MtProcessor::skin_depth(100.0, 4.0);
        assert_approx(d1 / d4, 2.0, 0.01, "skin depth ratio for 4x frequency");
    }

    // ── Cagniard resistivity tests ───────────────────────────────────

    #[test]
    fn test_cagniard_uniform_halfspace() {
        let rho = MtProcessor::cagniard_resistivity(100.0, 0.0, 1.0);
        assert_approx(rho, 100.0, TOL, "uniform half-space Cagniard");
    }

    #[test]
    fn test_cagniard_high_frequency() {
        // At very high frequency, skin depth is shallow, so result ~ rho_true
        let rho = MtProcessor::cagniard_resistivity(100.0, 1000.0, 1000.0);
        // At 1000 Hz, delta ~ 16 m << 1000 m depth, so rho_a ~ rho_true
        assert!(
            (rho - 100.0).abs() / 100.0 < 0.01,
            "High frequency Cagniard: got {}",
            rho
        );
    }

    // ── FFT tests ────────────────────────────────────────────────────

    #[test]
    fn test_fft_dc_component() {
        let signal = vec![1.0; 8];
        let fft = MtProcessor::compute_fft(&signal, 8.0);
        assert_approx(fft[0].0, 0.0, TOL, "DC frequency");
        assert_approx(fft[0].1 .0, 8.0, TOL, "DC magnitude");
        assert_approx(fft[0].1 .1, 0.0, TOL, "DC imaginary");
    }

    #[test]
    fn test_fft_single_tone() {
        // Generate a 2 Hz tone sampled at 16 Hz
        let n = 16;
        let sr = 16.0;
        let f = 2.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f * (i as f64) / sr).cos())
            .collect();
        let fft = MtProcessor::compute_fft(&signal, sr);

        // Find the bin for 2 Hz
        let bin = (f / sr * n as f64) as usize;
        let mag = complex_abs(fft[bin].1);
        assert!(mag > 7.0, "Tone peak magnitude should be ~N/2={}, got {}", n / 2, mag);
    }

    #[test]
    fn test_fft_empty() {
        let fft = MtProcessor::compute_fft(&[], 1.0);
        assert!(fft.is_empty());
    }

    // ── Cross-spectrum / auto-spectrum tests ─────────────────────────

    #[test]
    fn test_cross_spectrum_of_identical_equals_auto() {
        let a = vec![(1.0, 2.0), (3.0, -1.0), (0.5, 0.5)];
        let cross = MtProcessor::cross_spectrum(&a, &a);
        let auto = MtProcessor::auto_spectrum(&a);

        for i in 0..a.len() {
            // cross_spectrum(a, a) = a * conj(a) = |a|^2 (real)
            assert_approx(cross[i].0, auto[i], TOL, "cross == auto real part");
            assert_approx(cross[i].1, 0.0, TOL, "cross(a,a) imaginary = 0");
        }
    }

    #[test]
    fn test_auto_spectrum_positive() {
        let a = vec![(1.0, -3.0), (-2.0, 5.0)];
        let auto = MtProcessor::auto_spectrum(&a);
        for &v in &auto {
            assert!(v >= 0.0, "auto-spectrum must be non-negative");
        }
    }

    #[test]
    fn test_cross_spectrum_conjugate_symmetry() {
        let a = vec![(1.0, 2.0), (3.0, -1.0)];
        let b = vec![(0.5, -0.5), (2.0, 1.0)];
        let sab = MtProcessor::cross_spectrum(&a, &b);
        let sba = MtProcessor::cross_spectrum(&b, &a);
        // <A B*> = conj(<B A*>)
        for i in 0..a.len() {
            let conj_ba = complex_conj(sba[i]);
            assert_complex_approx(sab[i], conj_ba, TOL, "cross-spectrum conjugate symmetry");
        }
    }

    // ── 1D Earth impedance tests ─────────────────────────────────────

    #[test]
    fn test_1d_earth_diagonal_zero() {
        // For a 1D Earth: Zxx=Zyy=0, Zxy=-Zyx
        let z = ImpedanceTensor::new(
            (0.0, 0.0),
            (0.5, 0.5),
            (-0.5, -0.5),
            (0.0, 0.0),
            1.0,
        );
        assert!(DimensionalityAnalyzer::is_1d(&z, 0.1), "Should be 1D");
    }

    #[test]
    fn test_1d_earth_not_3d() {
        let z = ImpedanceTensor::new(
            (0.0, 0.0),
            (0.5, 0.5),
            (-0.5, -0.5),
            (0.0, 0.0),
            1.0,
        );
        assert!(DimensionalityAnalyzer::is_2d(&z, 0.1), "1D is also 2D");
    }

    // ── Swift skew tests ─────────────────────────────────────────────

    #[test]
    fn test_swift_skew_2d_structure() {
        // 2D: Zxx = Zyy = 0 => trace = 0 => skew = 0
        let z = ImpedanceTensor::new(
            (0.0, 0.0),
            (1.0, 0.5),
            (-0.8, -0.3),
            (0.0, 0.0),
            1.0,
        );
        let skew = DimensionalityAnalyzer::swift_skew(&z);
        assert_approx(skew, 0.0, TOL, "Swift skew for 2D");
    }

    #[test]
    fn test_swift_skew_3d_structure() {
        // 3D: non-zero diagonal elements
        let z = ImpedanceTensor::new(
            (0.3, 0.1),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.2, 0.1),
            1.0,
        );
        let skew = DimensionalityAnalyzer::swift_skew(&z);
        assert!(skew > 0.0, "3D structure should have non-zero skew, got {}", skew);
    }

    // ── Strike angle tests ───────────────────────────────────────────

    #[test]
    fn test_strike_angle_range() {
        let z = ImpedanceTensor::new(
            (0.1, 0.05),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.1, -0.05),
            1.0,
        );
        let theta = DimensionalityAnalyzer::strike_angle(&z);
        assert!(
            theta >= -45.0 && theta <= 45.0,
            "Strike angle out of range: {}",
            theta
        );
    }

    #[test]
    fn test_rotation_by_zero_identity() {
        let z = ImpedanceTensor::new(
            (0.1, 0.2),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.1, 0.1),
            1.0,
        );
        let z_rot = DimensionalityAnalyzer::rotate_impedance(&z, 0.0);
        assert_complex_approx(z_rot.zxx, z.zxx, TOL, "rot 0 zxx");
        assert_complex_approx(z_rot.zxy, z.zxy, TOL, "rot 0 zxy");
        assert_complex_approx(z_rot.zyx, z.zyx, TOL, "rot 0 zyx");
        assert_complex_approx(z_rot.zyy, z.zyy, TOL, "rot 0 zyy");
    }

    #[test]
    fn test_rotation_360_identity() {
        let z = ImpedanceTensor::new(
            (0.1, 0.2),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.1, 0.1),
            1.0,
        );
        let z_rot = DimensionalityAnalyzer::rotate_impedance(&z, 360.0);
        assert_complex_approx(z_rot.zxx, z.zxx, 1e-10, "rot 360 zxx");
        assert_complex_approx(z_rot.zxy, z.zxy, 1e-10, "rot 360 zxy");
    }

    #[test]
    fn test_rotation_90_swaps_components() {
        // For a 1D tensor with Zxx=Zyy=0, Zxy=-Zyx:
        // Rotation by 90 degrees:
        //   Z'xx = s^2 * Zyy + cs(Zxy+Zyx) + ... but for 1D, Zxx=Zyy=0.
        //   Actually, for the 1D case Zxy=-Zyx:
        //     Z'xy should map to -Zyx = Zxy (unchanged for 1D)
        //
        // More specifically, for 90 deg rotation of a general tensor:
        //   Z'xx = Zyy, Z'xy = -Zyx, Z'yx = -Zxy, Z'yy = Zxx
        let z = ImpedanceTensor::new(
            (0.1, 0.2),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.1, 0.1),
            1.0,
        );
        let z90 = DimensionalityAnalyzer::rotate_impedance(&z, 90.0);
        // After 90 deg: Z'xx = Zyy, Z'yy = Zxx, Z'xy = -Zyx, Z'yx = -Zxy
        assert_complex_approx(z90.zxx, z.zyy, 1e-10, "rot90: Z'xx = Zyy");
        assert_complex_approx(z90.zyy, z.zxx, 1e-10, "rot90: Z'yy = Zxx");
        assert_complex_approx(z90.zxy, complex_neg(z.zyx), 1e-10, "rot90: Z'xy = -Zyx");
        assert_complex_approx(z90.zyx, complex_neg(z.zxy), 1e-10, "rot90: Z'yx = -Zxy");
    }

    #[test]
    fn test_rotation_preserves_determinant() {
        let z = ImpedanceTensor::new(
            (0.1, 0.2),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.1, 0.1),
            1.0,
        );
        let det_orig = z.determinant();
        let z_rot = DimensionalityAnalyzer::rotate_impedance(&z, 37.0);
        let det_rot = z_rot.determinant();
        assert_complex_approx(det_rot, det_orig, 1e-10, "det preserved under rotation");
    }

    #[test]
    fn test_rotation_preserves_trace() {
        let z = ImpedanceTensor::new(
            (0.1, 0.2),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.1, 0.1),
            1.0,
        );
        let tr_orig = z.trace();
        let z_rot = DimensionalityAnalyzer::rotate_impedance(&z, 23.0);
        let tr_rot = z_rot.trace();
        assert_complex_approx(tr_rot, tr_orig, 1e-10, "trace preserved under rotation");
    }

    // ── Tipper tests ─────────────────────────────────────────────────

    #[test]
    fn test_tipper_zero_for_1d_earth() {
        // For a 1D layered Earth, Hz = 0 (no vertical magnetic field
        // from a uniform source), so tipper should be zero.
        let n = 64;
        let sr = 64.0;
        let f = 4.0;

        // Generate Hx and Hy as sinusoids, Hz = 0
        let hx: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f * (i as f64) / sr).cos())
            .collect();
        let hy: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f * (i as f64) / sr).sin())
            .collect();
        let hz = vec![0.0; n];

        let tipper = TipperEstimator::estimate_tipper(&hz, &hx, &hy, f, sr);
        let mag = TipperEstimator::tipper_magnitude(&tipper);
        assert!(
            mag < 1e-10,
            "Tipper should be zero for 1D earth, got magnitude {}",
            mag
        );
    }

    #[test]
    fn test_induction_arrow_real() {
        let tipper = Tipper {
            tx: (0.3, 0.1),
            ty: (-0.2, 0.05),
            frequency_hz: 1.0,
        };
        let (ax, ay) = TipperEstimator::induction_arrow_real(&tipper);
        assert_approx(ax, 0.3, TOL, "real arrow x");
        assert_approx(ay, -0.2, TOL, "real arrow y");
    }

    #[test]
    fn test_induction_arrow_imaginary() {
        let tipper = Tipper {
            tx: (0.3, 0.1),
            ty: (-0.2, 0.05),
            frequency_hz: 1.0,
        };
        let (ax, ay) = TipperEstimator::induction_arrow_imaginary(&tipper);
        assert_approx(ax, 0.1, TOL, "imag arrow x");
        assert_approx(ay, 0.05, TOL, "imag arrow y");
    }

    #[test]
    fn test_tipper_magnitude() {
        let tipper = Tipper {
            tx: (0.3, 0.4),
            ty: (0.0, 0.0),
            frequency_hz: 1.0,
        };
        let mag = TipperEstimator::tipper_magnitude(&tipper);
        assert_approx(mag, 0.5, TOL, "|T| = |Tx| when Ty=0");
    }

    // ── Bahr skew tests ──────────────────────────────────────────────

    #[test]
    fn test_bahr_skew_1d() {
        let z = ImpedanceTensor::new(
            (0.0, 0.0),
            (1.0, 0.5),
            (-1.0, -0.5),
            (0.0, 0.0),
            1.0,
        );
        let skew = DimensionalityAnalyzer::bahr_skew(&z);
        assert!(skew < 0.01, "Bahr skew for 1D: {}", skew);
    }

    // ── Impedance estimation integration tests ───────────────────────

    #[test]
    fn test_impedance_estimation_known_relationship() {
        // The impedance estimator uses multi-window averaging of
        // cross-spectral densities. This is necessary because a single
        // DFT window always yields a singular spectral matrix.
        //
        // We generate broadband Hx and Hy signals with non-integer
        // frequencies (causing spectral leakage), compute Ex and Ey
        // via spectral multiplication by a known constant Z, then
        // verify the estimator recovers Z.
        let n = 512; // Long enough for multiple segments
        let sr = 128.0;

        // Non-integer frequencies ensure spectral leakage at all bins
        let freqs_x = [1.3, 3.7, 5.1, 7.9, 9.3, 11.6, 13.2, 15.8];
        let freqs_y = [2.1, 4.4, 6.8, 8.2, 10.5, 12.7, 14.3, 16.9];

        let hx: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                freqs_x.iter().map(|&f| (2.0 * PI * f * t).cos() / f).sum()
            })
            .collect();

        let hy: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sr;
                freqs_y.iter().map(|&f| (2.0 * PI * f * t).sin() / f).sum()
            })
            .collect();

        // Known impedance (1D case: Zxx=Zyy=0)
        let zxy = (0.01, 0.01); // 45 degrees

        // Compute Ex, Ey via spectral multiplication with Zxy
        // Ex = IDFT(Zxy * DFT(Hy)), Ey = IDFT(-Zxy * DFT(Hx))
        let hy_fft = MtProcessor::compute_fft(&hy, sr);
        let hx_fft = MtProcessor::compute_fft(&hx, sr);
        let ex = spectral_multiply_idft(n, &hy_fft, zxy);
        let ey = spectral_multiply_idft(n, &hx_fft, complex_neg(zxy));

        let config = MtConfig {
            sample_rate_hz: sr,
            ..Default::default()
        };
        let proc = MtProcessor::new(config);
        let z_est = proc.estimate_impedance(&ex, &ey, &hx, &hy, 5.0);

        // Verify Zxy is recovered
        let z_est_mag = complex_abs(z_est.zxy);
        let z_expected_mag = complex_abs(zxy);
        assert!(
            z_est_mag > 0.0 && z_expected_mag > 0.0,
            "Impedance magnitudes must be positive: est={}, exp={}",
            z_est_mag,
            z_expected_mag,
        );
        let ratio = z_est_mag / z_expected_mag;
        assert!(
            (ratio - 1.0).abs() < 0.3,
            "Estimated |Zxy|={} vs expected |Zxy|={}, ratio={}",
            z_est_mag,
            z_expected_mag,
            ratio,
        );
    }

    /// Multiply spectral bins by a complex constant and inverse-DFT back
    /// to a real time series. Handles the one-sided FFT output by
    /// reconstructing the full spectrum with conjugate symmetry.
    fn spectral_multiply_idft(n: usize, fft: &[(f64, (f64, f64))], z: (f64, f64)) -> Vec<f64> {
        let nf = n as f64;
        let n_fft = fft.len(); // n/2 + 1

        // Build full N-point spectrum
        let mut full_spec = vec![(0.0, 0.0); n];
        for k in 0..n_fft {
            full_spec[k] = complex_mul(z, fft[k].1);
        }
        // Fill negative frequencies via conjugate symmetry
        for k in 1..n_fft - 1 {
            if n - k < n {
                full_spec[n - k] = complex_conj(full_spec[k]);
            }
        }

        // Inverse DFT
        let mut out = vec![0.0; n];
        for t in 0..n {
            let mut val = 0.0;
            for k in 0..n {
                let angle = 2.0 * PI * (k as f64) * (t as f64) / nf;
                val += full_spec[k].0 * angle.cos() - full_spec[k].1 * angle.sin();
            }
            out[t] = val / nf;
        }
        out
    }

    #[test]
    fn test_impedance_tensor_determinant() {
        let z = ImpedanceTensor::new(
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
            1.0,
        );
        // det = (1)(0-i) - (0+i)(-1) = -i + i = 0
        let det = z.determinant();
        // (1+0i)*(0-i) = -i
        // (0+i)*(-1+0i) = -i
        // det = -i - (-i) = 0
        assert_complex_approx(det, (0.0, 0.0), TOL, "det for special tensor");
    }

    #[test]
    fn test_impedance_tensor_trace() {
        let z = ImpedanceTensor::new(
            (1.0, 2.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (3.0, -1.0),
            1.0,
        );
        let tr = z.trace();
        assert_complex_approx(tr, (4.0, 1.0), TOL, "trace");
    }

    // ── Config and constructor tests ─────────────────────────────────

    #[test]
    fn test_default_config() {
        let config = MtConfig::default();
        assert_eq!(config.sample_rate_hz, 256.0);
        assert_eq!(config.num_frequencies, 20);
        assert_eq!(config.robustness, RobustMethod::OrdinaryLeastSquares);
        assert!(!config.remote_reference);
    }

    #[test]
    fn test_processor_creation() {
        let config = MtConfig {
            sample_rate_hz: 512.0,
            num_frequencies: 30,
            frequency_range: (0.01, 50.0),
            robustness: RobustMethod::HuberWeight,
            remote_reference: true,
        };
        let proc = MtProcessor::new(config);
        assert_eq!(proc.config().sample_rate_hz, 512.0);
        assert!(proc.config().remote_reference);
    }

    #[test]
    fn test_robust_method_variants() {
        let methods = [
            RobustMethod::OrdinaryLeastSquares,
            RobustMethod::HuberWeight,
            RobustMethod::ThompsonWeight,
        ];
        // Ensure all variants are distinct
        assert_ne!(methods[0], methods[1]);
        assert_ne!(methods[1], methods[2]);
        assert_ne!(methods[0], methods[2]);
    }

    #[test]
    fn test_is_2d_with_threshold() {
        // Pure 2D: diagonal = 0
        let z2d = ImpedanceTensor::new(
            (0.0, 0.0),
            (1.0, 0.5),
            (-0.8, -0.3),
            (0.0, 0.0),
            1.0,
        );
        assert!(DimensionalityAnalyzer::is_2d(&z2d, 0.1));

        // 3D: significant diagonal
        let z3d = ImpedanceTensor::new(
            (0.5, 0.3),
            (1.0, 0.5),
            (-0.8, -0.3),
            (-0.5, -0.3),
            1.0,
        );
        // With a strict threshold, this should fail the 2D test
        let skew = DimensionalityAnalyzer::swift_skew(&z3d);
        if skew > 0.05 {
            assert!(!DimensionalityAnalyzer::is_2d(&z3d, 0.01));
        }
    }

    #[test]
    fn test_is_1d_rejects_3d() {
        let z = ImpedanceTensor::new(
            (0.5, 0.3),
            (1.0, 0.5),
            (-0.3, -0.1),
            (0.4, 0.2),
            1.0,
        );
        assert!(!DimensionalityAnalyzer::is_1d(&z, 0.1), "3D tensor should not be 1D");
    }
}
