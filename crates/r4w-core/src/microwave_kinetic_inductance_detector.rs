//! # Microwave Kinetic Inductance Detector (MKID) Readout Processor
//!
//! Signal processing for superconducting pair-breaking photon detectors used in
//! submillimeter and optical astronomy. MKIDs exploit the kinetic inductance change
//! of a superconducting thin film when Cooper pairs are broken by absorbed photons,
//! shifting the resonance frequency of a lithographed microwave resonator.
//!
//! ## Resonator Model
//!
//! The forward transmission scattering parameter follows a Lorentzian:
//!
//! ```text
//! S21(f) = 1 - (Q/Qc) * exp(jφ) / (1 + 2jQ * (f - f0) / f0)
//! ```
//!
//! where Q is total quality factor, Qc is coupling quality factor, f0 is resonance
//! frequency, and φ is asymmetry (impedance mismatch) angle.
//!
//! ## Key Algorithms
//!
//! - **IQ loop fitting**: Circle fit in I-Q plane (Kasa method) to extract Q_i, Q_c, f_0
//! - **Phase/dissipation readout**: Δθ and Δr photon pulse extraction
//! - **Optimal filtering**: Wiener-matched filter for photon energy estimation
//! - **Quasiparticle lifetime**: τ_qp from exponential pulse decay fitting
//! - **Noise characterization**: Amplifier, TLS, generation-recombination noise
//! - **NEP calculation**: Noise Equivalent Power from spectral noise density
//! - **Frequency comb readout**: Multiplexed tone generation for 1000+ resonators
//! - **Duffing nonlinearity**: Bifurcation correction at high readout power
//! - **Cosmic ray detection**: Glitch flagging via amplitude/coincidence criteria
//! - **Resonator tracking**: Identification across temperature sweeps
//! - **Mattis-Bardeen theory**: Complex conductivity σ1, σ2 from Δ and T
//!
//! ## References
//!
//! - Day et al., Nature 425, 817 (2003) — original MKID concept
//! - Zmuidzinas, Annu. Rev. Condens. Matter Phys. 3, 169 (2012)
//! - Gao, "The Physics of Superconducting Microwave Resonators" (Caltech thesis, 2008)
//! - Mattis & Bardeen, Phys. Rev. 111, 412 (1958)

use std::f64::consts::PI;

// ─── Complex number type (no external crates) ───────────────────────────────

/// Minimal complex number for internal use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    #[inline]
    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }

    #[inline]
    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    #[inline]
    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }

    #[inline]
    pub fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }
}

impl std::ops::Add for Complex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.re + rhs.re, self.im + rhs.im)
    }
}

impl std::ops::Sub for Complex {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.re - rhs.re, self.im - rhs.im)
    }
}

impl std::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.re * rhs.re - self.im * rhs.im,
            self.re * rhs.im + self.im * rhs.re,
        )
    }
}

impl std::ops::Div for Complex {
    type Output = Self;
    fn div(self, rhs: Self) -> Self {
        let d = rhs.norm_sqr();
        Self::new(
            (self.re * rhs.re + self.im * rhs.im) / d,
            (self.im * rhs.re - self.re * rhs.im) / d,
        )
    }
}

impl std::ops::Mul<f64> for Complex {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.re * rhs, self.im * rhs)
    }
}

impl std::ops::Div<f64> for Complex {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self::new(self.re / rhs, self.im / rhs)
    }
}

impl std::ops::AddAssign for Complex {
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

// ─── Resonator model ────────────────────────────────────────────────────────

/// Parameters for a single microwave resonator.
#[derive(Debug, Clone)]
pub struct ResonatorParams {
    /// Resonance frequency f0 in Hz (typically 1–10 GHz).
    pub f0: f64,
    /// Total quality factor Q = 1/(1/Qi + 1/Qc).
    pub q_total: f64,
    /// Internal (intrinsic) quality factor Qi.
    pub q_internal: f64,
    /// Coupling quality factor Qc.
    pub q_coupling: f64,
    /// Asymmetry angle φ in radians (impedance mismatch).
    pub phi: f64,
}

impl ResonatorParams {
    /// Create resonator parameters, computing Q_total from Qi and Qc.
    pub fn new(f0: f64, q_internal: f64, q_coupling: f64, phi: f64) -> Self {
        let q_total = 1.0 / (1.0 / q_internal + 1.0 / q_coupling);
        Self {
            f0,
            q_total,
            q_internal,
            q_coupling,
            phi,
        }
    }

    /// Evaluate S21 at frequency f.
    ///
    /// S21(f) = 1 - (Q/Qc) * exp(jφ) / (1 + 2jQ(f-f0)/f0)
    pub fn s21(&self, f: f64) -> Complex {
        let x = (f - self.f0) / self.f0;
        let denom = Complex::new(1.0, 2.0 * self.q_total * x);
        let phase_factor = Complex::from_polar(1.0, self.phi);
        let coupling = self.q_total / self.q_coupling;
        let one = Complex::new(1.0, 0.0);
        one - phase_factor * coupling / denom
    }

    /// Generate S21 sweep across a frequency array.
    pub fn s21_sweep(&self, freqs: &[f64]) -> Vec<Complex> {
        freqs.iter().map(|&f| self.s21(f)).collect()
    }

    /// 3 dB bandwidth of the resonator.
    pub fn bandwidth_3db(&self) -> f64 {
        self.f0 / self.q_total
    }

    /// Fractional frequency shift for a given internal Q change.
    pub fn fractional_freq_shift(&self, delta_qi_inv: f64) -> f64 {
        // δf/f0 ≈ -α * δ(1/Qi) / (4 * Q)  (kinetic inductance fraction α ≈ 1)
        -delta_qi_inv / (4.0 * self.q_total)
    }
}

// ─── IQ circle fit (Kasa method) ────────────────────────────────────────────

/// Result of IQ circle fitting.
#[derive(Debug, Clone)]
pub struct CircleFitResult {
    /// Center of the fitted circle (I, Q).
    pub center: (f64, f64),
    /// Radius of the fitted circle.
    pub radius: f64,
    /// Residual sum of squares.
    pub residual: f64,
}

/// Fit a circle to IQ data using the Kasa (algebraic) method.
///
/// Minimizes Σ(x² + y² - 2ax - 2by - c)² where center = (a, b).
pub fn circle_fit(i_data: &[f64], q_data: &[f64]) -> Option<CircleFitResult> {
    let n = i_data.len();
    if n < 3 || q_data.len() != n {
        return None;
    }
    let nf = n as f64;

    // Compute sums for the Kasa normal equations
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sx2 = 0.0;
    let mut sy2 = 0.0;
    let mut sxy = 0.0;
    let mut sx3 = 0.0;
    let mut sy3 = 0.0;
    let mut sx2y = 0.0;
    let mut sxy2 = 0.0;

    for k in 0..n {
        let x = i_data[k];
        let y = q_data[k];
        let x2 = x * x;
        let y2 = y * y;
        sx += x;
        sy += y;
        sx2 += x2;
        sy2 += y2;
        sxy += x * y;
        sx3 += x2 * x;
        sy3 += y2 * y;
        sx2y += x2 * y;
        sxy2 += x * y2;
    }

    // Kasa normal equations: A * [a, b]^T = B
    // where center = (a, b)
    let a11 = sx2 - sx * sx / nf;
    let a12 = sxy - sx * sy / nf;
    let a22 = sy2 - sy * sy / nf;

    let b1 = 0.5 * (sx3 + sxy2 - sx * (sx2 + sy2) / nf);
    let b2 = 0.5 * (sx2y + sy3 - sy * (sx2 + sy2) / nf);

    let det = a11 * a22 - a12 * a12;
    if det.abs() < 1e-30 {
        return None;
    }

    let cx = (a22 * b1 - a12 * b2) / det;
    let cy = (a11 * b2 - a12 * b1) / det;

    // Radius
    let r = ((sx2 + sy2 - 2.0 * cx * sx - 2.0 * cy * sy) / nf
        + cx * cx
        + cy * cy)
        .sqrt();

    // Residual
    let mut residual = 0.0;
    for k in 0..n {
        let di = i_data[k] - cx;
        let dq = q_data[k] - cy;
        let d = (di * di + dq * dq).sqrt() - r;
        residual += d * d;
    }

    Some(CircleFitResult {
        center: (cx, cy),
        radius: r,
        residual,
    })
}

/// Extract resonator parameters from a frequency sweep and IQ circle fit.
pub fn fit_resonator(
    freqs: &[f64],
    i_data: &[f64],
    q_data: &[f64],
) -> Option<ResonatorParams> {
    let fit = circle_fit(i_data, q_data)?;
    let n = freqs.len();
    if n < 3 {
        return None;
    }

    // Find the frequency closest to the resonance (maximum phase velocity on circle)
    let (cx, cy) = fit.center;

    // Phase angles relative to circle center
    let mut phases: Vec<f64> = Vec::with_capacity(n);
    for k in 0..n {
        let angle = (q_data[k] - cy).atan2(i_data[k] - cx);
        phases.push(angle);
    }

    // Find steepest phase change => resonance location
    let mut max_dphase = 0.0_f64;
    let mut res_idx = 0;
    for k in 1..n {
        let mut dp = phases[k] - phases[k - 1];
        // Unwrap
        while dp > PI {
            dp -= 2.0 * PI;
        }
        while dp < -PI {
            dp += 2.0 * PI;
        }
        if dp.abs() > max_dphase.abs() {
            max_dphase = dp;
            res_idx = k;
        }
    }

    let f0 = freqs[res_idx];

    // Diameter of the IQ circle gives Q/Qc
    // |S21_min| = 1 - Q/Qc  (on resonance, phi=0 limit)
    // Circle diameter d = 2r, and the off-resonance point is at (1,0) in normalized S21
    let diameter = 2.0 * fit.radius;

    // Q/Qc from circle geometry
    let q_over_qc = diameter; // approximate for small asymmetry

    // Estimate Q from the 3 dB bandwidth
    // Find frequencies where |S21| crosses the -3 dB level
    let s21_on_res = ((i_data[res_idx]) * (i_data[res_idx])
        + (q_data[res_idx]) * (q_data[res_idx]))
    .sqrt();
    let s21_off = 1.0; // normalized off-resonance
    let s21_3db = (s21_off * s21_off + s21_on_res * s21_on_res).sqrt() / 2.0_f64.sqrt();

    let mut f_low = freqs[0];
    let mut f_high = freqs[n - 1];
    let mut found_low = false;

    for k in 0..n {
        let mag = (i_data[k] * i_data[k] + q_data[k] * q_data[k]).sqrt();
        if !found_low && mag < s21_3db {
            f_low = freqs[k];
            found_low = true;
        }
        if found_low && mag > s21_3db && freqs[k] > f0 {
            f_high = freqs[k];
            break;
        }
    }

    let bw = (f_high - f_low).abs();
    let q_total = if bw > 0.0 { f0 / bw } else { 1e5 };

    let q_coupling = if q_over_qc > 0.0 {
        q_total / q_over_qc
    } else {
        q_total * 2.0
    };
    let q_internal = 1.0 / (1.0 / q_total - 1.0 / q_coupling).max(1e-12);

    // Estimate asymmetry angle from off-resonance point offset
    let phi = cy.atan2(1.0 - cx); // approximate

    Some(ResonatorParams::new(f0, q_internal, q_coupling, phi))
}

// ─── Phase and dissipation readout ──────────────────────────────────────────

/// Phase and dissipation response from a photon event.
#[derive(Debug, Clone)]
pub struct PhotonPulse {
    /// Phase shift Δθ in radians (proportional to frequency shift).
    pub delta_theta: Vec<f64>,
    /// Dissipation shift Δr (fractional radius change).
    pub delta_r: Vec<f64>,
    /// Timestamp of pulse peak (sample index).
    pub peak_index: usize,
    /// Peak amplitude of phase response.
    pub peak_phase: f64,
    /// Peak amplitude of dissipation response.
    pub peak_dissipation: f64,
    /// Estimated energy (arbitrary units, from optimal filter).
    pub energy: f64,
    /// Quasiparticle lifetime τ_qp in samples.
    pub tau_qp: f64,
    /// Flag: cosmic ray glitch detected.
    pub is_cosmic_ray: bool,
}

/// Extract phase and dissipation time streams from IQ data relative to a
/// reference (steady-state) point on the IQ loop.
pub fn extract_phase_dissipation(
    i_data: &[f64],
    q_data: &[f64],
    ref_i: f64,
    ref_q: f64,
    circle_center: (f64, f64),
) -> (Vec<f64>, Vec<f64>) {
    let (cx, cy) = circle_center;
    let ref_angle = (ref_q - cy).atan2(ref_i - cx);
    let ref_radius = ((ref_i - cx).powi(2) + (ref_q - cy).powi(2)).sqrt();

    let n = i_data.len();
    let mut phase = Vec::with_capacity(n);
    let mut dissipation = Vec::with_capacity(n);

    for k in 0..n {
        let angle = (q_data[k] - cy).atan2(i_data[k] - cx);
        let radius = ((i_data[k] - cx).powi(2) + (q_data[k] - cy).powi(2)).sqrt();

        let mut dtheta = angle - ref_angle;
        while dtheta > PI {
            dtheta -= 2.0 * PI;
        }
        while dtheta < -PI {
            dtheta += 2.0 * PI;
        }
        phase.push(dtheta);

        let dr = if ref_radius > 0.0 {
            (radius - ref_radius) / ref_radius
        } else {
            0.0
        };
        dissipation.push(dr);
    }

    (phase, dissipation)
}

// ─── Optimal filter for photon energy estimation ────────────────────────────

/// Optimal (Wiener) filter for pulse energy estimation.
///
/// Given a template pulse shape s[n] and noise power spectral density N(f),
/// the optimal filter is h[n] = IFFT(S*(f) / N(f)) normalized so that
/// the output peak equals the pulse energy.
#[derive(Debug, Clone)]
pub struct OptimalFilter {
    /// Filter template in time domain.
    template: Vec<f64>,
    /// Noise PSD (one-sided, same length as template).
    noise_psd: Vec<f64>,
    /// Filter coefficients (time domain matched filter).
    coefficients: Vec<f64>,
}

impl OptimalFilter {
    /// Create an optimal filter from a pulse template and noise PSD.
    ///
    /// For simplicity, uses time-domain matched filtering with noise weighting.
    pub fn new(template: &[f64], noise_psd: &[f64]) -> Self {
        let n = template.len();
        let npsd = if noise_psd.len() >= n {
            noise_psd[..n].to_vec()
        } else {
            let mut v = noise_psd.to_vec();
            v.resize(n, noise_psd.last().copied().unwrap_or(1.0));
            v
        };

        // Compute matched filter coefficients with noise weighting
        // In time domain: h[k] = s[N-1-k] / noise_weight
        // We use average noise as a simple weighting
        let avg_noise: f64 = npsd.iter().sum::<f64>() / n as f64;
        let noise_weight = if avg_noise > 0.0 { avg_noise } else { 1.0 };

        let mut coeffs = Vec::with_capacity(n);
        let mut norm = 0.0;
        for k in 0..n {
            let c = template[n - 1 - k] / noise_weight;
            coeffs.push(c);
            norm += c * template[k];
        }

        // Normalize so peak output = 1.0 for unit-energy pulse
        if norm.abs() > 1e-30 {
            for c in coeffs.iter_mut() {
                *c /= norm;
            }
        }

        Self {
            template: template.to_vec(),
            noise_psd: npsd,
            coefficients: coeffs,
        }
    }

    /// Apply the optimal filter to a data stream. Returns filtered output.
    pub fn apply(&self, data: &[f64]) -> Vec<f64> {
        let n = self.coefficients.len();
        let m = data.len();
        if m < n {
            return vec![0.0; m];
        }

        let mut output = Vec::with_capacity(m);
        for k in 0..m {
            let mut sum = 0.0;
            for j in 0..n {
                if k >= j {
                    sum += self.coefficients[j] * data[k - j];
                }
            }
            output.push(sum);
        }
        output
    }

    /// Estimate pulse energy from filtered data.
    pub fn estimate_energy(&self, data: &[f64]) -> f64 {
        let filtered = self.apply(data);
        filtered
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Return filter coefficients.
    pub fn coefficients(&self) -> &[f64] {
        &self.coefficients
    }
}

// ─── Quasiparticle lifetime extraction ──────────────────────────────────────

/// Fit an exponential decay A * exp(-t/τ) + offset to the tail of a photon pulse.
///
/// Returns (amplitude, tau_qp_samples, offset, residual).
pub fn fit_exponential_decay(
    data: &[f64],
    start_idx: usize,
) -> (f64, f64, f64, f64) {
    let n = data.len();
    if start_idx >= n || n - start_idx < 3 {
        return (0.0, 1.0, 0.0, f64::MAX);
    }

    let tail = &data[start_idx..];
    let m = tail.len();

    // Estimate offset from final samples
    let tail_avg: f64 = tail[m * 3 / 4..].iter().sum::<f64>()
        / (m - m * 3 / 4) as f64;

    // Linear regression on log(data - offset) vs time
    let mut sum_t = 0.0;
    let mut sum_y = 0.0;
    let mut sum_tt = 0.0;
    let mut sum_ty = 0.0;
    let mut count = 0.0;

    for k in 0..m {
        let val = tail[k] - tail_avg;
        if val > 1e-20 {
            let t = k as f64;
            let y = val.ln();
            sum_t += t;
            sum_y += y;
            sum_tt += t * t;
            sum_ty += t * y;
            count += 1.0;
        }
    }

    if count < 2.0 {
        return (tail[0] - tail_avg, 1.0, tail_avg, f64::MAX);
    }

    let slope = (count * sum_ty - sum_t * sum_y) / (count * sum_tt - sum_t * sum_t);
    let intercept = (sum_y - slope * sum_t) / count;

    let tau = if slope.abs() > 1e-30 {
        -1.0 / slope
    } else {
        1.0
    };
    let amplitude = intercept.exp();

    // Compute residual
    let mut residual = 0.0;
    for k in 0..m {
        let predicted = amplitude * (-(k as f64) / tau).exp() + tail_avg;
        let err = tail[k] - predicted;
        residual += err * err;
    }

    (amplitude, tau.abs(), tail_avg, residual)
}

// ─── Noise characterization ─────────────────────────────────────────────────

/// Types of noise in MKID readout systems.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NoiseType {
    /// White amplifier noise (HEMT or SiGe).
    Amplifier,
    /// Two-Level System (TLS) noise: 1/f spectrum from dielectric fluctuations.
    Tls,
    /// Generation-Recombination noise from quasiparticle fluctuations.
    GenerationRecombination,
    /// Photon noise (shot noise from incident radiation).
    Photon,
}

/// Noise spectral analysis result.
#[derive(Debug, Clone)]
pub struct NoiseSpectrum {
    /// Frequency bins (Hz).
    pub frequencies: Vec<f64>,
    /// Power spectral density (units²/Hz).
    pub psd: Vec<f64>,
    /// Identified dominant noise type.
    pub dominant_noise: NoiseType,
    /// Fitted 1/f knee frequency (Hz).
    pub f_knee: f64,
    /// White noise floor level.
    pub white_floor: f64,
}

/// Estimate noise PSD using Welch's method (simplified).
///
/// Splits data into overlapping segments, computes periodograms, averages.
pub fn estimate_noise_psd(
    data: &[f64],
    sample_rate: f64,
    segment_len: usize,
) -> NoiseSpectrum {
    let n = data.len();
    let seg = segment_len.min(n).max(4);
    let overlap = seg / 2;
    let step = seg - overlap;

    let mut num_segments = 0;
    let fft_len = seg;
    let mut avg_psd = vec![0.0; fft_len / 2 + 1];

    // Window: Hann
    let window: Vec<f64> = (0..seg)
        .map(|k| 0.5 * (1.0 - (2.0 * PI * k as f64 / seg as f64).cos()))
        .collect();
    let win_power: f64 = window.iter().map(|w| w * w).sum::<f64>() / seg as f64;

    let mut pos = 0;
    while pos + seg <= n {
        // Apply window and compute DFT magnitudes
        let mut segment: Vec<f64> = (0..seg)
            .map(|k| data[pos + k] * window[k])
            .collect();

        // Remove mean
        let mean: f64 = segment.iter().sum::<f64>() / seg as f64;
        for s in segment.iter_mut() {
            *s -= mean;
        }

        // Compute periodogram via DFT (real input)
        for fi in 0..=fft_len / 2 {
            let freq = fi as f64 / fft_len as f64;
            let mut sum_re = 0.0;
            let mut sum_im = 0.0;
            for k in 0..seg {
                let angle = -2.0 * PI * freq * k as f64;
                sum_re += segment[k] * angle.cos();
                sum_im += segment[k] * angle.sin();
            }
            let power = (sum_re * sum_re + sum_im * sum_im) / (seg as f64 * sample_rate * win_power);
            avg_psd[fi] += power;
        }

        num_segments += 1;
        pos += step;
    }

    if num_segments == 0 {
        num_segments = 1;
    }
    for p in avg_psd.iter_mut() {
        *p /= num_segments as f64;
    }

    let frequencies: Vec<f64> = (0..=fft_len / 2)
        .map(|k| k as f64 * sample_rate / fft_len as f64)
        .collect();

    // Identify noise characteristics
    // White noise floor: average of upper quarter of spectrum
    let upper_start = avg_psd.len() * 3 / 4;
    let white_floor = if upper_start < avg_psd.len() {
        avg_psd[upper_start..].iter().sum::<f64>() / (avg_psd.len() - upper_start) as f64
    } else {
        avg_psd.last().copied().unwrap_or(0.0)
    };

    // 1/f knee: frequency where PSD = 2 * white floor
    let f_knee_threshold = 2.0 * white_floor;
    let mut f_knee = frequencies[0];
    for k in 1..avg_psd.len() {
        if avg_psd[k] < f_knee_threshold {
            f_knee = frequencies[k];
            break;
        }
    }

    // Classify dominant noise
    let dominant_noise = if f_knee > sample_rate / 10.0 {
        NoiseType::Tls
    } else if white_floor > 1e-10 {
        NoiseType::Amplifier
    } else {
        NoiseType::GenerationRecombination
    };

    NoiseSpectrum {
        frequencies,
        psd: avg_psd,
        dominant_noise,
        f_knee,
        white_floor,
    }
}

// ─── Noise Equivalent Power ─────────────────────────────────────────────────

/// Calculate Noise Equivalent Power (NEP) from noise spectral density and responsivity.
///
/// NEP = S_noise^{1/2} / R  [W/Hz^{1/2}]
///
/// where S_noise is the spectral noise density (rad²/Hz or fractional²/Hz)
/// and R is the responsivity (rad/W or fraction/W).
pub fn calculate_nep(noise_psd_at_freq: f64, responsivity: f64) -> f64 {
    if responsivity.abs() < 1e-30 {
        return f64::MAX;
    }
    noise_psd_at_freq.sqrt() / responsivity.abs()
}

/// Calculate photon-noise-limited NEP for a background-limited detector.
///
/// NEP_photon = sqrt(2 * P * h * ν + 2 * P² / Δν)
///
/// where P is optical power, h is Planck's constant, ν is frequency, Δν is bandwidth.
pub fn nep_photon_limited(
    optical_power_w: f64,
    optical_freq_hz: f64,
    optical_bandwidth_hz: f64,
) -> f64 {
    const H_PLANCK: f64 = 6.62607015e-34; // J·s
    let term1 = 2.0 * optical_power_w * H_PLANCK * optical_freq_hz;
    let term2 = if optical_bandwidth_hz > 0.0 {
        2.0 * optical_power_w * optical_power_w / optical_bandwidth_hz
    } else {
        0.0
    };
    (term1 + term2).sqrt()
}

// ─── Frequency comb readout ─────────────────────────────────────────────────

/// Configuration for a frequency-multiplexed MKID readout comb.
#[derive(Debug, Clone)]
pub struct FrequencyComb {
    /// Center frequencies of each resonator tone (Hz).
    pub tone_freqs: Vec<f64>,
    /// Amplitudes for each tone (linear scale).
    pub tone_amplitudes: Vec<f64>,
    /// Phase offsets for each tone (radians).
    pub tone_phases: Vec<f64>,
    /// Sample rate of the DAC/ADC (Hz).
    pub sample_rate: f64,
}

impl FrequencyComb {
    /// Create a uniform comb of `num_tones` tones spanning `f_start` to `f_end`.
    pub fn uniform(f_start: f64, f_end: f64, num_tones: usize, sample_rate: f64) -> Self {
        let spacing = if num_tones > 1 {
            (f_end - f_start) / (num_tones - 1) as f64
        } else {
            0.0
        };

        let tone_freqs: Vec<f64> = (0..num_tones)
            .map(|k| f_start + k as f64 * spacing)
            .collect();
        let tone_amplitudes = vec![1.0 / (num_tones as f64).sqrt(); num_tones];
        let tone_phases = vec![0.0; num_tones];

        Self {
            tone_freqs,
            tone_amplitudes,
            tone_phases,
            sample_rate,
        }
    }

    /// Create from explicit resonator frequencies.
    pub fn from_frequencies(freqs: &[f64], sample_rate: f64) -> Self {
        let n = freqs.len();
        Self {
            tone_freqs: freqs.to_vec(),
            tone_amplitudes: vec![1.0 / (n as f64).sqrt(); n],
            tone_phases: vec![0.0; n],
            sample_rate,
        }
    }

    /// Number of tones in the comb.
    pub fn num_tones(&self) -> usize {
        self.tone_freqs.len()
    }

    /// Generate the frequency comb waveform (complex baseband).
    pub fn generate(&self, num_samples: usize) -> Vec<Complex> {
        let mut output = vec![Complex::new(0.0, 0.0); num_samples];
        let dt = 1.0 / self.sample_rate;

        for (idx, &freq) in self.tone_freqs.iter().enumerate() {
            let amp = self.tone_amplitudes[idx];
            let phi = self.tone_phases[idx];
            for k in 0..num_samples {
                let t = k as f64 * dt;
                let angle = 2.0 * PI * freq * t + phi;
                output[k] += Complex::new(amp * angle.cos(), amp * angle.sin());
            }
        }

        output
    }

    /// Channelize received data: extract each resonator's IQ time stream.
    ///
    /// Digital down-converts each tone and applies a simple boxcar filter.
    pub fn channelize(
        &self,
        data_i: &[f64],
        data_q: &[f64],
        decimation: usize,
    ) -> Vec<Vec<Complex>> {
        let n = data_i.len().min(data_q.len());
        let dt = 1.0 / self.sample_rate;
        let dec = decimation.max(1);
        let out_len = n / dec;

        let mut channels = Vec::with_capacity(self.tone_freqs.len());

        for (idx, &freq) in self.tone_freqs.iter().enumerate() {
            let mut channel = Vec::with_capacity(out_len);
            let phi = self.tone_phases[idx];

            for out_k in 0..out_len {
                let mut acc = Complex::new(0.0, 0.0);
                for d in 0..dec {
                    let k = out_k * dec + d;
                    if k >= n {
                        break;
                    }
                    let t = k as f64 * dt;
                    let angle = -2.0 * PI * freq * t - phi;
                    let mixer = Complex::new(angle.cos(), angle.sin());
                    let sample = Complex::new(data_i[k], data_q[k]);
                    acc += sample * mixer;
                }
                channel.push(acc / dec as f64);
            }

            channels.push(channel);
            let _ = idx; // suppress unused warning
        }

        channels
    }

    /// Compute the crest factor (PAPR) of the comb waveform.
    pub fn crest_factor(&self, num_samples: usize) -> f64 {
        let waveform = self.generate(num_samples);
        let mut peak = 0.0_f64;
        let mut avg_power = 0.0;
        for s in &waveform {
            let p = s.norm_sqr();
            avg_power += p;
            if p > peak {
                peak = p;
            }
        }
        avg_power /= num_samples as f64;
        if avg_power > 0.0 {
            10.0 * (peak / avg_power).log10()
        } else {
            0.0
        }
    }
}

// ─── Duffing nonlinearity correction ────────────────────────────────────────

/// Duffing oscillator model for resonator nonlinearity at high readout power.
///
/// The Duffing equation introduces an amplitude-dependent frequency shift:
///   f_eff = f0 * (1 + α * |a|²)
///
/// where α is the Duffing parameter (negative for KIDs) and |a|² is the
/// intracavity photon number.
#[derive(Debug, Clone)]
pub struct DuffingCorrection {
    /// Duffing parameter α (typically negative, ~-1e-6 for Al KIDs).
    pub alpha: f64,
    /// Critical intracavity power for bifurcation onset.
    pub p_bifurcation: f64,
    /// Reference resonance frequency.
    pub f0: f64,
}

impl DuffingCorrection {
    /// Create a Duffing correction model.
    ///
    /// `alpha` is the Duffing parameter, `f0` is the unperturbed resonance,
    /// and `q` is the quality factor.
    pub fn new(alpha: f64, f0: f64, q: f64) -> Self {
        // Bifurcation threshold: P_bif = 4/(3√3) * f0/(Q*|α|)
        let p_bif = if alpha.abs() > 1e-30 {
            4.0 / (3.0 * 3.0_f64.sqrt()) * f0 / (q * alpha.abs())
        } else {
            f64::MAX
        };
        Self {
            alpha,
            p_bifurcation: p_bif,
            f0,
        }
    }

    /// Compute the effective (shifted) resonance frequency given intracavity power.
    pub fn effective_frequency(&self, intracavity_power: f64) -> f64 {
        self.f0 * (1.0 + self.alpha * intracavity_power)
    }

    /// Check if the readout is in the bistable (bifurcation) regime.
    pub fn is_bifurcated(&self, intracavity_power: f64) -> bool {
        intracavity_power > self.p_bifurcation
    }

    /// Correct a frequency shift measurement for Duffing nonlinearity.
    ///
    /// Given an observed frequency and estimated intracavity power,
    /// return the linearized (corrected) frequency.
    pub fn correct_frequency(
        &self,
        observed_freq: f64,
        intracavity_power: f64,
    ) -> f64 {
        observed_freq / (1.0 + self.alpha * intracavity_power)
    }

    /// Correct a phase shift measurement for nonlinearity.
    pub fn correct_phase(&self, observed_phase: f64, intracavity_power: f64) -> f64 {
        let correction = 1.0 + self.alpha * intracavity_power;
        if correction.abs() > 1e-30 {
            observed_phase / correction
        } else {
            observed_phase
        }
    }
}

// ─── Cosmic ray glitch detection ────────────────────────────────────────────

/// Cosmic ray glitch detection configuration.
#[derive(Debug, Clone)]
pub struct CosmicRayDetector {
    /// Threshold multiplier above noise RMS for glitch detection.
    pub threshold_sigma: f64,
    /// Minimum number of coincident pixels for a cosmic ray event.
    pub min_coincident: usize,
    /// Maximum rise time in samples (cosmic rays are faster than photons).
    pub max_rise_samples: usize,
    /// Dead time after detection in samples.
    pub dead_time: usize,
}

impl CosmicRayDetector {
    /// Create a default cosmic ray detector.
    pub fn new() -> Self {
        Self {
            threshold_sigma: 10.0,
            min_coincident: 3,
            max_rise_samples: 2,
            dead_time: 100,
        }
    }

    /// Create with custom parameters.
    pub fn with_params(
        threshold_sigma: f64,
        min_coincident: usize,
        max_rise_samples: usize,
        dead_time: usize,
    ) -> Self {
        Self {
            threshold_sigma,
            min_coincident,
            max_rise_samples,
            dead_time,
        }
    }

    /// Detect cosmic ray glitches in a single-channel phase time stream.
    ///
    /// Returns indices of flagged samples.
    pub fn detect_single_channel(&self, phase_data: &[f64]) -> Vec<usize> {
        let n = phase_data.len();
        if n < 3 {
            return Vec::new();
        }

        // Compute noise RMS from the median absolute deviation (robust)
        let mut diffs: Vec<f64> = Vec::with_capacity(n - 1);
        for k in 1..n {
            diffs.push((phase_data[k] - phase_data[k - 1]).abs());
        }
        diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_diff = if diffs.is_empty() {
            0.0
        } else {
            diffs[diffs.len() / 2]
        };
        let noise_rms = median_diff / 0.6745; // MAD to sigma conversion

        let threshold = self.threshold_sigma * noise_rms;

        let mut flagged = Vec::new();
        let mut last_flag = 0_usize;

        for k in 1..n {
            if k < last_flag + self.dead_time {
                continue;
            }

            let diff = (phase_data[k] - phase_data[k - 1]).abs();
            if diff > threshold {
                // Check rise time
                let mut rise_end = k;
                for j in (k + 1)..n.min(k + self.max_rise_samples + 1) {
                    if (phase_data[j] - phase_data[j - 1]).abs() > threshold * 0.5 {
                        rise_end = j;
                    }
                }
                let rise_time = rise_end - k + 1;
                if rise_time <= self.max_rise_samples {
                    flagged.push(k);
                    last_flag = k;
                }
            }
        }

        flagged
    }

    /// Detect cosmic rays using multi-pixel coincidence.
    ///
    /// `channel_data` is a slice of phase time streams for nearby pixels.
    /// Returns sample indices where coincidence exceeds the threshold.
    pub fn detect_coincident(
        &self,
        channel_data: &[&[f64]],
    ) -> Vec<usize> {
        if channel_data.is_empty() {
            return Vec::new();
        }

        let n = channel_data[0].len();
        let single_flags: Vec<Vec<usize>> = channel_data
            .iter()
            .map(|ch| self.detect_single_channel(ch))
            .collect();

        let mut coincident = Vec::new();
        let mut last_flag = 0_usize;

        for k in 0..n {
            if k < last_flag + self.dead_time {
                continue;
            }

            let mut count = 0;
            for flags in &single_flags {
                if flags.contains(&k) {
                    count += 1;
                }
            }

            if count >= self.min_coincident {
                coincident.push(k);
                last_flag = k;
            }
        }

        coincident
    }
}

impl Default for CosmicRayDetector {
    fn default() -> Self {
        Self::new()
    }
}

// ─── Resonator identification and tracking ──────────────────────────────────

/// A tracked resonator with identity across sweeps.
#[derive(Debug, Clone)]
pub struct TrackedResonator {
    /// Unique identifier.
    pub id: usize,
    /// Current resonance frequency (Hz).
    pub frequency: f64,
    /// Current quality factor.
    pub q_total: f64,
    /// History of frequencies at different temperatures.
    pub freq_history: Vec<(f64, f64)>, // (temperature_K, frequency_Hz)
    /// Estimated df/dT (Hz/K) from history.
    pub df_dt: f64,
}

/// Identify and track resonators across temperature sweeps.
#[derive(Debug, Clone)]
pub struct ResonatorTracker {
    /// Known resonators.
    pub resonators: Vec<TrackedResonator>,
    /// Maximum frequency shift allowed for identification (Hz).
    pub max_freq_shift: f64,
    /// Next available ID.
    next_id: usize,
}

impl ResonatorTracker {
    /// Create a new tracker with a maximum frequency matching tolerance.
    pub fn new(max_freq_shift: f64) -> Self {
        Self {
            resonators: Vec::new(),
            max_freq_shift,
            next_id: 0,
        }
    }

    /// Update tracker with a new sweep at a given temperature.
    ///
    /// `new_freqs` are the detected resonance frequencies.
    /// Returns the mapping: new_freq_index -> resonator_id.
    pub fn update_sweep(
        &mut self,
        new_freqs: &[f64],
        temperature_k: f64,
    ) -> Vec<usize> {
        let mut assignments = vec![0_usize; new_freqs.len()];

        if self.resonators.is_empty() {
            // First sweep: create new resonators
            for (idx, &f) in new_freqs.iter().enumerate() {
                let res = TrackedResonator {
                    id: self.next_id,
                    frequency: f,
                    q_total: 0.0,
                    freq_history: vec![(temperature_k, f)],
                    df_dt: 0.0,
                };
                assignments[idx] = self.next_id;
                self.resonators.push(res);
                self.next_id += 1;
            }
            return assignments;
        }

        // Hungarian-style greedy nearest-neighbor matching
        let mut used_res = vec![false; self.resonators.len()];

        // Sort by closeness for greedy matching
        let mut pairs: Vec<(usize, usize, f64)> = Vec::new();
        for (ni, &nf) in new_freqs.iter().enumerate() {
            for (ri, res) in self.resonators.iter().enumerate() {
                let dist = (nf - res.frequency).abs();
                if dist < self.max_freq_shift {
                    pairs.push((ni, ri, dist));
                }
            }
        }
        pairs.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

        let mut used_new = vec![false; new_freqs.len()];

        for (ni, ri, _dist) in &pairs {
            if !used_new[*ni] && !used_res[*ri] {
                // Match found
                self.resonators[*ri].frequency = new_freqs[*ni];
                self.resonators[*ri]
                    .freq_history
                    .push((temperature_k, new_freqs[*ni]));

                // Update df/dT from history
                if self.resonators[*ri].freq_history.len() >= 2 {
                    let hist = &self.resonators[*ri].freq_history;
                    let last = hist.len() - 1;
                    let dt = hist[last].0 - hist[last - 1].0;
                    let df = hist[last].1 - hist[last - 1].1;
                    if dt.abs() > 1e-12 {
                        self.resonators[*ri].df_dt = df / dt;
                    }
                }

                assignments[*ni] = self.resonators[*ri].id;
                used_new[*ni] = true;
                used_res[*ri] = true;
            }
        }

        // Create new resonators for unmatched frequencies
        for (ni, &nf) in new_freqs.iter().enumerate() {
            if !used_new[ni] {
                let res = TrackedResonator {
                    id: self.next_id,
                    frequency: nf,
                    q_total: 0.0,
                    freq_history: vec![(temperature_k, nf)],
                    df_dt: 0.0,
                };
                assignments[ni] = self.next_id;
                self.resonators.push(res);
                self.next_id += 1;
            }
        }

        assignments
    }

    /// Get the number of tracked resonators.
    pub fn num_resonators(&self) -> usize {
        self.resonators.len()
    }

    /// Find resonator by ID.
    pub fn find(&self, id: usize) -> Option<&TrackedResonator> {
        self.resonators.iter().find(|r| r.id == id)
    }

    /// Predict resonance frequencies at a given temperature using df/dT.
    pub fn predict_frequencies(&self, temperature_k: f64) -> Vec<(usize, f64)> {
        self.resonators
            .iter()
            .filter_map(|res| {
                if let Some(&(last_t, last_f)) = res.freq_history.last() {
                    let predicted = last_f + res.df_dt * (temperature_k - last_t);
                    Some((res.id, predicted))
                } else {
                    None
                }
            })
            .collect()
    }
}

// ─── Mattis-Bardeen complex conductivity ────────────────────────────────────

/// Mattis-Bardeen complex conductivity σ = σ1 - jσ2.
///
/// Computes the ratio σ1/σn and σ2/σn where σn is the normal-state conductivity,
/// as functions of temperature T and frequency f, given the superconducting gap Δ(0).
///
/// Valid for T << Tc and hf << 2Δ.
#[derive(Debug, Clone)]
pub struct MattisBardeen {
    /// Superconducting gap energy Δ(0) in Joules.
    pub delta_0: f64,
    /// Critical temperature Tc in Kelvin.
    pub tc: f64,
    /// Normal-state conductivity σ_n in S/m.
    pub sigma_n: f64,
}

impl MattisBardeen {
    /// Create from gap energy and critical temperature.
    ///
    /// For aluminum: Δ(0) ≈ 0.17 meV = 2.72e-23 J, Tc ≈ 1.2 K.
    pub fn new(delta_0_joules: f64, tc_kelvin: f64, sigma_n: f64) -> Self {
        Self {
            delta_0: delta_0_joules,
            tc: tc_kelvin,
            sigma_n,
        }
    }

    /// Aluminum thin film preset.
    pub fn aluminum() -> Self {
        Self::new(2.72e-23, 1.2, 3.77e7)
    }

    /// Niobium thin film preset.
    pub fn niobium() -> Self {
        Self::new(2.41e-22, 9.3, 6.93e6)
    }

    /// Titanium nitride (TiN) preset.
    pub fn titanium_nitride() -> Self {
        Self::new(3.2e-23, 4.5, 5.0e5)
    }

    /// BCS gap temperature dependence: Δ(T) ≈ Δ(0) * tanh(1.74 * sqrt(Tc/T - 1)).
    pub fn gap_at_temperature(&self, temperature_k: f64) -> f64 {
        if temperature_k >= self.tc {
            return 0.0;
        }
        if temperature_k < 1e-10 {
            return self.delta_0;
        }
        self.delta_0 * (1.74 * (self.tc / temperature_k - 1.0).sqrt()).tanh()
    }

    /// Thermal quasiparticle density n_qp(T).
    ///
    /// n_qp ≈ 2 * N(0) * sqrt(2π * kB * T * Δ) * exp(-Δ/kBT)
    ///
    /// Returns n_qp in arbitrary units (normalized to N(0)).
    pub fn quasiparticle_density(&self, temperature_k: f64) -> f64 {
        const KB: f64 = 1.380649e-23; // Boltzmann constant (J/K)
        let delta = self.gap_at_temperature(temperature_k);
        if temperature_k < 1e-10 || delta < 1e-30 {
            return 0.0;
        }
        let kbt = KB * temperature_k;
        2.0 * (2.0 * PI * kbt * delta).sqrt() * (-delta / kbt).exp()
    }

    /// Compute σ1/σn (real part) — dissipative component.
    ///
    /// Simplified Mattis-Bardeen for hf << 2Δ and T << Tc:
    /// σ1/σn ≈ (2Δ/hf) * exp(-Δ/kBT) * sinh(hf/2kBT) * K0(hf/2kBT)
    ///
    /// We use an approximation for K0 (modified Bessel function).
    pub fn sigma1_ratio(&self, temperature_k: f64, frequency_hz: f64) -> f64 {
        const KB: f64 = 1.380649e-23;
        const H_PLANCK: f64 = 6.62607015e-34;

        let delta = self.gap_at_temperature(temperature_k);
        if temperature_k < 1e-10 || delta < 1e-30 || frequency_hz < 1.0 {
            return 0.0;
        }

        let kbt = KB * temperature_k;
        let hf = H_PLANCK * frequency_hz;
        let x = hf / (2.0 * kbt);

        // K0(x) approximation for small to moderate x
        let k0 = if x < 0.01 {
            -(x / 2.0).ln() - 0.5772 // Euler-Mascheroni
        } else if x < 2.0 {
            // Polynomial approximation
            let t = x / 2.0;
            -t.ln() * (1.0 + 3.5156229 * t * t)
                + (-0.57721566 + 0.42278420 * t * t + 0.23069756 * t.powi(4))
        } else {
            // Asymptotic expansion
            (PI / (2.0 * x)).sqrt() * (-x).exp()
        };

        let ratio = (2.0 * delta / hf) * (-delta / kbt).exp() * x.sinh() * k0.abs();
        ratio.max(0.0)
    }

    /// Compute σ2/σn (imaginary part) — reactive (kinetic inductance) component.
    ///
    /// σ2/σn ≈ πΔ/(hf) * [1 - 2*exp(-Δ/kBT)*exp(-hf/2kBT)*I0(hf/2kBT)]
    ///
    /// For T → 0: σ2/σn → πΔ/(hf).
    pub fn sigma2_ratio(&self, temperature_k: f64, frequency_hz: f64) -> f64 {
        const KB: f64 = 1.380649e-23;
        const H_PLANCK: f64 = 6.62607015e-34;

        let delta = self.gap_at_temperature(temperature_k);
        if frequency_hz < 1.0 || delta < 1e-30 {
            return 0.0;
        }

        let hf = H_PLANCK * frequency_hz;
        let sigma2_0 = PI * delta / hf;

        if temperature_k < 1e-10 {
            return sigma2_0;
        }

        let kbt = KB * temperature_k;
        let x = hf / (2.0 * kbt);

        // I0(x) approximation (modified Bessel function of the first kind)
        let i0 = if x < 3.75 {
            let t = (x / 3.75) * (x / 3.75);
            1.0 + 3.5156229 * t
                + 3.0899424 * t * t
                + 1.2067492 * t.powi(3)
                + 0.2659732 * t.powi(4)
                + 0.0360768 * t.powi(5)
        } else {
            (1.0 / (2.0 * PI * x).sqrt()) * x.exp()
        };

        let thermal_correction = 2.0 * (-delta / kbt).exp() * (-x).exp() * i0;
        sigma2_0 * (1.0 - thermal_correction).max(0.0)
    }

    /// Surface impedance Zs = Rs + jXs from σ1 and σ2.
    ///
    /// Zs ≈ sqrt(jωμ0 / (σ1 - jσ2)) for thick films, or
    /// Zs ≈ 1/(σ*d) for thin films (d << λ_L).
    pub fn surface_impedance(
        &self,
        temperature_k: f64,
        frequency_hz: f64,
        film_thickness_m: f64,
    ) -> (f64, f64) {
        let s1 = self.sigma1_ratio(temperature_k, frequency_hz) * self.sigma_n;
        let s2 = self.sigma2_ratio(temperature_k, frequency_hz) * self.sigma_n;

        let d = film_thickness_m;
        let sigma_mag_sq = s1 * s1 + s2 * s2;

        if sigma_mag_sq < 1e-30 || d < 1e-15 {
            return (f64::MAX, f64::MAX);
        }

        // Thin-film approximation: Zs = 1/(σ*d) = (σ1 + jσ2)/(|σ|²*d)
        let rs = s1 / (sigma_mag_sq * d);
        let xs = s2 / (sigma_mag_sq * d);

        (rs, xs)
    }

    /// Kinetic inductance fraction α from σ2.
    ///
    /// α = Lk / (Lk + Lg) where Lk ∝ 1/σ2 and Lg is geometric inductance.
    pub fn kinetic_inductance_fraction(
        &self,
        temperature_k: f64,
        frequency_hz: f64,
        geometric_inductance: f64,
    ) -> f64 {
        let s2 = self.sigma2_ratio(temperature_k, frequency_hz) * self.sigma_n;
        if s2 < 1e-30 || geometric_inductance < 1e-30 {
            return 0.0;
        }
        // Lk ∝ 1/σ2, approximate Lk = C/σ2 where C normalizes
        // For a simple model: α = 1/(1 + σ2 * Lg * C)
        // We use the ratio form: α ≈ Lk/(Lk + Lg)
        let lk = 1.0 / s2; // proportional to kinetic inductance
        lk / (lk + geometric_inductance)
    }
}

// ─── Photon pulse detection ─────────────────────────────────────────────────

/// Configuration for photon pulse detection.
#[derive(Debug, Clone)]
pub struct PulseDetectorConfig {
    /// Detection threshold in units of noise sigma.
    pub threshold_sigma: f64,
    /// Minimum separation between pulses in samples.
    pub min_separation: usize,
    /// Number of samples before the trigger for the pulse window.
    pub pre_trigger: usize,
    /// Number of samples after the trigger for the pulse window.
    pub post_trigger: usize,
    /// Quasiparticle lifetime estimate (samples) for template generation.
    pub tau_qp_estimate: f64,
}

impl PulseDetectorConfig {
    /// Default configuration for optical/NIR photon detection.
    pub fn default_optical() -> Self {
        Self {
            threshold_sigma: 5.0,
            min_separation: 200,
            pre_trigger: 50,
            post_trigger: 500,
            tau_qp_estimate: 100.0,
        }
    }

    /// Configuration for submillimeter photon detection.
    pub fn default_submm() -> Self {
        Self {
            threshold_sigma: 3.0,
            min_separation: 100,
            pre_trigger: 30,
            post_trigger: 300,
            tau_qp_estimate: 50.0,
        }
    }
}

/// Detect photon pulses in a phase time stream.
///
/// Returns a vector of `PhotonPulse` structures.
pub fn detect_photon_pulses(
    phase_data: &[f64],
    config: &PulseDetectorConfig,
) -> Vec<PhotonPulse> {
    let n = phase_data.len();
    if n < 10 {
        return Vec::new();
    }

    // Estimate noise RMS using MAD of first differences
    let mut diffs: Vec<f64> = Vec::with_capacity(n - 1);
    for k in 1..n {
        diffs.push((phase_data[k] - phase_data[k - 1]).abs());
    }
    diffs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median_diff = diffs[diffs.len() / 2];
    let noise_rms = median_diff / 0.6745;

    let threshold = config.threshold_sigma * noise_rms;

    // Compute baseline (running median approximation)
    let baseline = {
        let mut bl = Vec::with_capacity(n);
        let win = 50.min(n);
        for k in 0..n {
            let start = k.saturating_sub(win / 2);
            let end = (k + win / 2).min(n);
            let mut window: Vec<f64> = phase_data[start..end].to_vec();
            window.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            bl.push(window[window.len() / 2]);
        }
        bl
    };

    let mut pulses = Vec::new();
    let mut last_trigger = 0_usize;

    for k in 1..n {
        if k < last_trigger + config.min_separation {
            continue;
        }

        let deviation = phase_data[k] - baseline[k];
        if deviation.abs() > threshold {
            // Found a trigger point
            let pre = k.saturating_sub(config.pre_trigger);
            let post = (k + config.post_trigger).min(n);

            // Extract pulse window
            let window = &phase_data[pre..post];
            let bl_window = &baseline[pre..post];

            // Subtract baseline
            let pulse_data: Vec<f64> = window
                .iter()
                .zip(bl_window.iter())
                .map(|(s, b)| s - b)
                .collect();

            // Find peak
            let mut peak_idx = 0;
            let mut peak_val = 0.0_f64;
            for (j, &v) in pulse_data.iter().enumerate() {
                if v.abs() > peak_val.abs() {
                    peak_val = v;
                    peak_idx = j;
                }
            }

            // Fit exponential decay to extract tau_qp
            let decay_start = peak_idx + 2;
            let (amp, tau, offset, _residual) =
                if decay_start < pulse_data.len() {
                    fit_exponential_decay(
                        &pulse_data.iter().map(|v| v.abs()).collect::<Vec<_>>(),
                        decay_start,
                    )
                } else {
                    (peak_val.abs(), config.tau_qp_estimate, 0.0, 0.0)
                };

            // Dissipation: approximate from the pulse shape
            let delta_r: Vec<f64> = pulse_data.iter().map(|v| v.abs() * 0.1).collect();
            let peak_dissipation = delta_r
                .iter()
                .cloned()
                .fold(0.0_f64, f64::max);

            // Estimate energy from integral of pulse
            let energy = pulse_data.iter().map(|v| v.abs()).sum::<f64>();

            let pulse = PhotonPulse {
                delta_theta: pulse_data,
                delta_r,
                peak_index: pre + peak_idx,
                peak_phase: peak_val,
                peak_dissipation,
                energy,
                tau_qp: tau,
                is_cosmic_ray: false,
            };

            pulses.push(pulse);
            last_trigger = k;
            let _ = amp;
            let _ = offset;
        }
    }

    pulses
}

// ─── Generate pulse template ────────────────────────────────────────────────

/// Generate an ideal photon pulse template.
///
/// Template: A * (1 - exp(-t/τ_rise)) * exp(-t/τ_fall) for t >= 0.
pub fn generate_pulse_template(
    tau_rise_samples: f64,
    tau_fall_samples: f64,
    num_samples: usize,
) -> Vec<f64> {
    let mut template = Vec::with_capacity(num_samples);
    for k in 0..num_samples {
        let t = k as f64;
        let rise = 1.0 - (-t / tau_rise_samples).exp();
        let fall = (-t / tau_fall_samples).exp();
        template.push(rise * fall);
    }

    // Normalize to unit peak
    let peak = template
        .iter()
        .cloned()
        .fold(0.0_f64, f64::max);
    if peak > 1e-30 {
        for v in template.iter_mut() {
            *v /= peak;
        }
    }

    template
}

// ─── Photon energy histogram ────────────────────────────────────────────────

/// Histogram of photon energies for spectral analysis.
#[derive(Debug, Clone)]
pub struct EnergyHistogram {
    /// Bin edges.
    pub bin_edges: Vec<f64>,
    /// Counts per bin.
    pub counts: Vec<usize>,
    /// Total number of events.
    pub total_events: usize,
}

impl EnergyHistogram {
    /// Create a histogram with uniform bins.
    pub fn new(min_energy: f64, max_energy: f64, num_bins: usize) -> Self {
        let bin_width = (max_energy - min_energy) / num_bins as f64;
        let bin_edges: Vec<f64> = (0..=num_bins)
            .map(|k| min_energy + k as f64 * bin_width)
            .collect();
        let counts = vec![0; num_bins];
        Self {
            bin_edges,
            counts,
            total_events: 0,
        }
    }

    /// Add an event to the histogram.
    pub fn add_event(&mut self, energy: f64) {
        let n = self.counts.len();
        for k in 0..n {
            if energy >= self.bin_edges[k] && energy < self.bin_edges[k + 1] {
                self.counts[k] += 1;
                self.total_events += 1;
                return;
            }
        }
        // Overflow: add to last bin
        if energy >= self.bin_edges[n] && n > 0 {
            self.counts[n - 1] += 1;
            self.total_events += 1;
        }
    }

    /// Add multiple events.
    pub fn add_events(&mut self, energies: &[f64]) {
        for &e in energies {
            self.add_event(e);
        }
    }

    /// Get bin centers.
    pub fn bin_centers(&self) -> Vec<f64> {
        let n = self.counts.len();
        (0..n)
            .map(|k| (self.bin_edges[k] + self.bin_edges[k + 1]) / 2.0)
            .collect()
    }

    /// Energy resolution: FWHM of the dominant peak.
    pub fn energy_resolution(&self) -> f64 {
        let n = self.counts.len();
        if n < 3 {
            return 0.0;
        }

        // Find peak bin
        let mut peak_idx = 0;
        let mut peak_count = 0;
        for k in 0..n {
            if self.counts[k] > peak_count {
                peak_count = self.counts[k];
                peak_idx = k;
            }
        }

        let half_max = peak_count as f64 / 2.0;

        // Find FWHM
        let mut left = peak_idx;
        let mut right = peak_idx;

        for k in (0..peak_idx).rev() {
            if (self.counts[k] as f64) < half_max {
                left = k;
                break;
            }
        }

        for k in (peak_idx + 1)..n {
            if (self.counts[k] as f64) < half_max {
                right = k;
                break;
            }
        }

        let centers = self.bin_centers();
        if right < centers.len() && left < centers.len() {
            centers[right] - centers[left]
        } else {
            0.0
        }
    }
}

// ─── MKID Readout Processor (top-level) ─────────────────────────────────────

/// Top-level MKID readout processor for a multiplexed array.
#[derive(Debug, Clone)]
pub struct MkidReadoutProcessor {
    /// Frequency comb for multiplexed readout.
    pub comb: FrequencyComb,
    /// Resonator parameters for each channel.
    pub resonators: Vec<ResonatorParams>,
    /// Pulse detection configuration.
    pub pulse_config: PulseDetectorConfig,
    /// Cosmic ray detector.
    pub cosmic_ray_detector: CosmicRayDetector,
    /// Duffing correction models (one per resonator).
    pub duffing_corrections: Vec<Option<DuffingCorrection>>,
    /// Superconductor model.
    pub material: MattisBardeen,
    /// Operating temperature (K).
    pub temperature_k: f64,
    /// Accumulated photon events per channel.
    pub photon_counts: Vec<usize>,
}

impl MkidReadoutProcessor {
    /// Create a new MKID readout processor.
    pub fn new(
        comb: FrequencyComb,
        resonators: Vec<ResonatorParams>,
        material: MattisBardeen,
        temperature_k: f64,
    ) -> Self {
        let n = resonators.len();
        let pulse_config = PulseDetectorConfig::default_optical();
        let cosmic_ray_detector = CosmicRayDetector::new();
        let duffing_corrections = vec![None; n];
        let photon_counts = vec![0; n];

        Self {
            comb,
            resonators,
            pulse_config,
            cosmic_ray_detector,
            duffing_corrections,
            material,
            temperature_k,
            photon_counts,
        }
    }

    /// Set pulse detection configuration.
    pub fn set_pulse_config(&mut self, config: PulseDetectorConfig) {
        self.pulse_config = config;
    }

    /// Set Duffing correction for a specific channel.
    pub fn set_duffing_correction(&mut self, channel: usize, correction: DuffingCorrection) {
        if channel < self.duffing_corrections.len() {
            self.duffing_corrections[channel] = Some(correction);
        }
    }

    /// Process a block of raw IQ data from the readout system.
    ///
    /// Returns detected photon pulses per channel.
    pub fn process_block(
        &mut self,
        data_i: &[f64],
        data_q: &[f64],
        decimation: usize,
    ) -> Vec<Vec<PhotonPulse>> {
        // Channelize
        let channels = self.comb.channelize(data_i, data_q, decimation);

        let mut all_pulses = Vec::with_capacity(channels.len());

        for (ch_idx, channel) in channels.iter().enumerate() {
            if ch_idx >= self.resonators.len() {
                all_pulses.push(Vec::new());
                continue;
            }

            // Extract I and Q
            let ch_i: Vec<f64> = channel.iter().map(|c| c.re).collect();
            let ch_q: Vec<f64> = channel.iter().map(|c| c.im).collect();

            // Get circle fit for this resonator
            let fit = circle_fit(&ch_i, &ch_q);
            let center = fit
                .as_ref()
                .map(|f| f.center)
                .unwrap_or((0.0, 0.0));

            // Extract phase time stream
            let ref_i = ch_i.first().copied().unwrap_or(0.0);
            let ref_q = ch_q.first().copied().unwrap_or(0.0);
            let (phase, _dissipation) =
                extract_phase_dissipation(&ch_i, &ch_q, ref_i, ref_q, center);

            // Detect photon pulses
            let mut pulses = detect_photon_pulses(&phase, &self.pulse_config);

            // Check for cosmic rays
            let cosmic_flags = self.cosmic_ray_detector.detect_single_channel(&phase);
            for pulse in pulses.iter_mut() {
                if cosmic_flags.contains(&pulse.peak_index) {
                    pulse.is_cosmic_ray = true;
                }
            }

            // Apply Duffing correction if configured
            if let Some(ref correction) = self.duffing_corrections[ch_idx] {
                for pulse in pulses.iter_mut() {
                    pulse.peak_phase =
                        correction.correct_phase(pulse.peak_phase, 0.01); // estimate power
                }
            }

            self.photon_counts[ch_idx] += pulses.iter().filter(|p| !p.is_cosmic_ray).count();
            all_pulses.push(pulses);
        }

        all_pulses
    }

    /// Get total photon count for a channel.
    pub fn photon_count(&self, channel: usize) -> usize {
        self.photon_counts.get(channel).copied().unwrap_or(0)
    }

    /// Get the expected quasiparticle density at operating temperature.
    pub fn expected_nqp(&self) -> f64 {
        self.material.quasiparticle_density(self.temperature_k)
    }

    /// Calculate NEP for a channel given its noise and responsivity.
    pub fn channel_nep(&self, noise_psd: f64, responsivity: f64) -> f64 {
        calculate_nep(noise_psd, responsivity)
    }
}

// ─── Helper: linspace ───────────────────────────────────────────────────────

/// Generate linearly spaced values (like numpy.linspace).
fn linspace(start: f64, end: f64, n: usize) -> Vec<f64> {
    if n < 2 {
        return vec![start];
    }
    let step = (end - start) / (n - 1) as f64;
    (0..n).map(|k| start + k as f64 * step).collect()
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- Complex type tests ---

    #[test]
    fn test_complex_basic_ops() {
        let a = Complex::new(3.0, 4.0);
        let b = Complex::new(1.0, 2.0);

        let sum = a + b;
        assert!(approx_eq(sum.re, 4.0, TOL));
        assert!(approx_eq(sum.im, 6.0, TOL));

        let diff = a - b;
        assert!(approx_eq(diff.re, 2.0, TOL));
        assert!(approx_eq(diff.im, 2.0, TOL));
    }

    #[test]
    fn test_complex_multiply() {
        let a = Complex::new(3.0, 4.0);
        let b = Complex::new(1.0, 2.0);
        let prod = a * b;
        // (3+4j)(1+2j) = 3+6j+4j+8j² = 3-8+10j = -5+10j
        assert!(approx_eq(prod.re, -5.0, TOL));
        assert!(approx_eq(prod.im, 10.0, TOL));
    }

    #[test]
    fn test_complex_divide() {
        let a = Complex::new(3.0, 4.0);
        let b = Complex::new(1.0, 2.0);
        let quot = a / b;
        // (3+4j)/(1+2j) = (3+4j)(1-2j)/(1+4) = (11-2j)/5
        assert!(approx_eq(quot.re, 11.0 / 5.0, TOL));
        assert!(approx_eq(quot.im, -2.0 / 5.0, TOL));
    }

    #[test]
    fn test_complex_norm() {
        let c = Complex::new(3.0, 4.0);
        assert!(approx_eq(c.norm(), 5.0, TOL));
        assert!(approx_eq(c.norm_sqr(), 25.0, TOL));
    }

    #[test]
    fn test_complex_arg() {
        let c = Complex::new(1.0, 1.0);
        assert!(approx_eq(c.arg(), PI / 4.0, TOL));
    }

    #[test]
    fn test_complex_conj() {
        let c = Complex::new(3.0, 4.0);
        let cc = c.conj();
        assert!(approx_eq(cc.re, 3.0, TOL));
        assert!(approx_eq(cc.im, -4.0, TOL));
    }

    #[test]
    fn test_complex_from_polar() {
        let c = Complex::from_polar(2.0, PI / 3.0);
        assert!(approx_eq(c.re, 1.0, TOL));
        assert!(approx_eq(c.im, 3.0_f64.sqrt(), TOL));
    }

    #[test]
    fn test_complex_mul_scalar() {
        let c = Complex::new(2.0, 3.0) * 4.0;
        assert!(approx_eq(c.re, 8.0, TOL));
        assert!(approx_eq(c.im, 12.0, TOL));
    }

    #[test]
    fn test_complex_div_scalar() {
        let c = Complex::new(6.0, 8.0) / 2.0;
        assert!(approx_eq(c.re, 3.0, TOL));
        assert!(approx_eq(c.im, 4.0, TOL));
    }

    // --- Resonator model tests ---

    #[test]
    fn test_resonator_params_creation() {
        let res = ResonatorParams::new(5.0e9, 1e6, 5e4, 0.0);
        assert!(approx_eq(res.f0, 5.0e9, 1.0));
        assert!(res.q_internal > 0.0);
        assert!(res.q_coupling > 0.0);
        let expected_q = 1.0 / (1.0 / 1e6 + 1.0 / 5e4);
        assert!(approx_eq(res.q_total, expected_q, 1.0));
    }

    #[test]
    fn test_s21_on_resonance() {
        // On resonance with phi=0: S21 = 1 - Q/Qc
        let res = ResonatorParams::new(5.0e9, 1e6, 5e4, 0.0);
        let s = res.s21(res.f0);
        let expected = 1.0 - res.q_total / res.q_coupling;
        assert!(approx_eq(s.re, expected, 1e-4));
        assert!(approx_eq(s.im, 0.0, 1e-4));
    }

    #[test]
    fn test_s21_off_resonance() {
        let res = ResonatorParams::new(5.0e9, 1e6, 5e4, 0.0);
        // Far off resonance: S21 → 1
        let s = res.s21(6.0e9);
        assert!((s - Complex::new(1.0, 0.0)).norm() < 0.01);
    }

    #[test]
    fn test_s21_sweep_length() {
        let res = ResonatorParams::new(5.0e9, 1e5, 5e4, 0.0);
        let freqs = linspace(4.99e9, 5.01e9, 100);
        let sweep = res.s21_sweep(&freqs);
        assert_eq!(sweep.len(), 100);
    }

    #[test]
    fn test_s21_symmetry() {
        // S21 should be symmetric about f0 when phi=0
        let res = ResonatorParams::new(5.0e9, 1e5, 5e4, 0.0);
        let df = 1e4;
        let s_plus = res.s21(res.f0 + df);
        let s_minus = res.s21(res.f0 - df);
        assert!(approx_eq(s_plus.norm(), s_minus.norm(), 1e-6));
    }

    #[test]
    fn test_bandwidth_3db() {
        let res = ResonatorParams::new(5.0e9, 1e5, 5e4, 0.0);
        let bw = res.bandwidth_3db();
        assert!(bw > 0.0);
        assert!(approx_eq(bw, 5.0e9 / res.q_total, 1.0));
    }

    #[test]
    fn test_fractional_freq_shift() {
        let res = ResonatorParams::new(5.0e9, 1e5, 5e4, 0.0);
        let shift = res.fractional_freq_shift(1e-6);
        assert!(shift < 0.0); // Should be negative
    }

    // --- Circle fit tests ---

    #[test]
    fn test_circle_fit_known_circle() {
        // Generate points on a circle centered at (1, 2) with radius 3
        let n = 50;
        let cx = 1.0;
        let cy = 2.0;
        let r = 3.0;

        let mut i_data = Vec::new();
        let mut q_data = Vec::new();
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 / n as f64;
            i_data.push(cx + r * angle.cos());
            q_data.push(cy + r * angle.sin());
        }

        let fit = circle_fit(&i_data, &q_data).unwrap();
        assert!(approx_eq(fit.center.0, cx, 0.01));
        assert!(approx_eq(fit.center.1, cy, 0.01));
        assert!(approx_eq(fit.radius, r, 0.01));
    }

    #[test]
    fn test_circle_fit_too_few_points() {
        let result = circle_fit(&[1.0, 2.0], &[3.0, 4.0]);
        assert!(result.is_none());
    }

    #[test]
    fn test_circle_fit_residual() {
        // Perfect circle should have near-zero residual
        let n = 20;
        let mut i_data = Vec::new();
        let mut q_data = Vec::new();
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 / n as f64;
            i_data.push(5.0 * angle.cos());
            q_data.push(5.0 * angle.sin());
        }
        let fit = circle_fit(&i_data, &q_data).unwrap();
        assert!(fit.residual < 0.001);
    }

    // --- Fit resonator test ---

    #[test]
    fn test_fit_resonator_from_sweep() {
        let res = ResonatorParams::new(5.0e9, 1e5, 5e4, 0.0);
        let freqs = linspace(4.999e9, 5.001e9, 200);
        let sweep = res.s21_sweep(&freqs);
        let i_data: Vec<f64> = sweep.iter().map(|s| s.re).collect();
        let q_data: Vec<f64> = sweep.iter().map(|s| s.im).collect();

        let fitted = fit_resonator(&freqs, &i_data, &q_data);
        assert!(fitted.is_some());
        let fitted = fitted.unwrap();
        // Resonance frequency should be close
        assert!((fitted.f0 - 5.0e9).abs() < 1e7);
    }

    // --- Phase/dissipation extraction tests ---

    #[test]
    fn test_extract_phase_dissipation_baseline() {
        // Constant IQ at reference point => zero phase/dissipation
        let n = 100;
        let ref_i = 1.0;
        let ref_q = 0.0;
        let center = (0.0, 0.0);
        let i_data = vec![ref_i; n];
        let q_data = vec![ref_q; n];

        let (phase, diss) = extract_phase_dissipation(&i_data, &q_data, ref_i, ref_q, center);
        assert_eq!(phase.len(), n);
        assert_eq!(diss.len(), n);
        for k in 0..n {
            assert!(approx_eq(phase[k], 0.0, 1e-10));
            assert!(approx_eq(diss[k], 0.0, 1e-10));
        }
    }

    #[test]
    fn test_extract_phase_shift() {
        // Moving along the circle changes phase
        let center = (0.0, 0.0);
        let ref_i = 1.0;
        let ref_q = 0.0;
        let i_data = vec![0.0]; // 90 degrees from reference
        let q_data = vec![1.0];

        let (phase, _) = extract_phase_dissipation(&i_data, &q_data, ref_i, ref_q, center);
        assert!(approx_eq(phase[0], PI / 2.0, 0.01));
    }

    // --- Optimal filter tests ---

    #[test]
    fn test_optimal_filter_creation() {
        let template = vec![0.0, 0.5, 1.0, 0.8, 0.6, 0.4, 0.2, 0.1];
        let noise_psd = vec![1.0; 8];
        let filter = OptimalFilter::new(&template, &noise_psd);
        assert_eq!(filter.coefficients().len(), 8);
    }

    #[test]
    fn test_optimal_filter_apply() {
        let template = vec![0.0, 0.5, 1.0, 0.5, 0.0];
        let noise_psd = vec![1.0; 5];
        let filter = OptimalFilter::new(&template, &noise_psd);

        let data = vec![0.0, 0.0, 0.0, 0.5, 1.0, 0.5, 0.0, 0.0, 0.0, 0.0];
        let output = filter.apply(&data);
        assert_eq!(output.len(), data.len());
    }

    #[test]
    fn test_optimal_filter_energy() {
        let template = vec![0.0, 1.0, 0.5, 0.25];
        let noise_psd = vec![1.0; 4];
        let filter = OptimalFilter::new(&template, &noise_psd);
        let energy = filter.estimate_energy(&template);
        assert!(energy > 0.0);
    }

    // --- Exponential decay fit tests ---

    #[test]
    fn test_exponential_decay_fit() {
        // Generate A * exp(-t/tau) + offset
        let tau = 50.0;
        let amp = 10.0;
        let offset = 0.5;
        let data: Vec<f64> = (0..200)
            .map(|k| amp * (-(k as f64) / tau).exp() + offset)
            .collect();

        let (fitted_amp, fitted_tau, fitted_offset, residual) =
            fit_exponential_decay(&data, 0);

        assert!((fitted_tau - tau).abs() < tau * 0.2); // within 20%
        assert!(residual < amp * amp);
        let _ = fitted_amp;
        let _ = fitted_offset;
    }

    #[test]
    fn test_exponential_decay_short_data() {
        let (_, tau, _, _) = fit_exponential_decay(&[1.0, 0.5], 0);
        assert!(tau > 0.0);
    }

    // --- Noise characterization tests ---

    #[test]
    fn test_noise_psd_estimation() {
        // Generate white noise (pseudo)
        let n = 1024;
        let sample_rate = 1000.0;
        let mut data = Vec::with_capacity(n);
        let mut x = 0.123456789_f64;
        for _ in 0..n {
            // Simple LCG pseudo-random
            x = (x * 6364136223846793005.0 + 1.0) % (2.0_f64.powi(32));
            data.push(x / 2.0_f64.powi(32) - 0.5);
        }

        let result = estimate_noise_psd(&data, sample_rate, 128);
        assert!(!result.frequencies.is_empty());
        assert!(!result.psd.is_empty());
        assert!(result.white_floor > 0.0);
    }

    #[test]
    fn test_noise_psd_frequency_range() {
        let n = 256;
        let sr = 1000.0;
        let data = vec![0.1; n]; // constant signal
        let result = estimate_noise_psd(&data, sr, 64);
        assert!(result.frequencies[0] >= 0.0);
        assert!(*result.frequencies.last().unwrap() <= sr / 2.0 + 1.0);
    }

    // --- NEP tests ---

    #[test]
    fn test_nep_calculation() {
        let noise_psd = 1e-18; // rad²/Hz
        let responsivity = 1e6; // rad/W
        let nep = calculate_nep(noise_psd, responsivity);
        // NEP = sqrt(1e-18) / 1e6 = 1e-9 / 1e6 = 1e-15 W/Hz^{1/2}
        assert!(approx_eq(nep, 1e-15, 1e-18));
    }

    #[test]
    fn test_nep_zero_responsivity() {
        let nep = calculate_nep(1e-18, 0.0);
        assert_eq!(nep, f64::MAX);
    }

    #[test]
    fn test_nep_photon_limited() {
        let nep = nep_photon_limited(1e-15, 350e9, 50e9);
        assert!(nep > 0.0);
        assert!(nep < 1e-15); // should be very small
    }

    // --- Frequency comb tests ---

    #[test]
    fn test_freq_comb_uniform() {
        let comb = FrequencyComb::uniform(4.0e9, 6.0e9, 10, 20.0e9);
        assert_eq!(comb.num_tones(), 10);
        assert!(approx_eq(comb.tone_freqs[0], 4.0e9, 1.0));
        assert!(approx_eq(*comb.tone_freqs.last().unwrap(), 6.0e9, 1.0));
    }

    #[test]
    fn test_freq_comb_from_frequencies() {
        let freqs = vec![4.0e9, 4.5e9, 5.0e9, 5.5e9];
        let comb = FrequencyComb::from_frequencies(&freqs, 20.0e9);
        assert_eq!(comb.num_tones(), 4);
    }

    #[test]
    fn test_freq_comb_generate() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 3, 1000.0);
        let waveform = comb.generate(100);
        assert_eq!(waveform.len(), 100);
        // Should not be all zeros
        let total_power: f64 = waveform.iter().map(|c| c.norm_sqr()).sum();
        assert!(total_power > 0.0);
    }

    #[test]
    fn test_freq_comb_channelize() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 2, 1000.0);
        let n = 100;
        let waveform = comb.generate(n);
        let data_i: Vec<f64> = waveform.iter().map(|c| c.re).collect();
        let data_q: Vec<f64> = waveform.iter().map(|c| c.im).collect();
        let channels = comb.channelize(&data_i, &data_q, 1);
        assert_eq!(channels.len(), 2);
        assert_eq!(channels[0].len(), n);
    }

    #[test]
    fn test_freq_comb_crest_factor() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 4, 1000.0);
        let cf = comb.crest_factor(1000);
        assert!(cf > 0.0); // Multi-tone signal has PAPR > 0 dB
    }

    // --- Duffing correction tests ---

    #[test]
    fn test_duffing_creation() {
        let duffing = DuffingCorrection::new(-1e-6, 5.0e9, 1e5);
        assert!(duffing.p_bifurcation > 0.0);
        assert!(duffing.alpha < 0.0);
    }

    #[test]
    fn test_duffing_effective_frequency() {
        let duffing = DuffingCorrection::new(-1e-6, 5.0e9, 1e5);
        let f_eff = duffing.effective_frequency(1000.0);
        assert!(f_eff < 5.0e9); // Negative alpha shifts frequency down
    }

    #[test]
    fn test_duffing_zero_power() {
        let duffing = DuffingCorrection::new(-1e-6, 5.0e9, 1e5);
        let f_eff = duffing.effective_frequency(0.0);
        assert!(approx_eq(f_eff, 5.0e9, 1.0));
    }

    #[test]
    fn test_duffing_bifurcation_check() {
        let duffing = DuffingCorrection::new(-1e-6, 5.0e9, 1e5);
        assert!(!duffing.is_bifurcated(0.01));
        assert!(duffing.is_bifurcated(duffing.p_bifurcation * 2.0));
    }

    #[test]
    fn test_duffing_correction() {
        let duffing = DuffingCorrection::new(-1e-6, 5.0e9, 1e5);
        let power = 100.0;
        let observed = duffing.effective_frequency(power);
        let corrected = duffing.correct_frequency(observed, power);
        assert!(approx_eq(corrected, 5.0e9, 1.0));
    }

    // --- Cosmic ray detector tests ---

    #[test]
    fn test_cosmic_ray_detector_default() {
        let det = CosmicRayDetector::new();
        assert!(det.threshold_sigma > 0.0);
        assert!(det.min_coincident > 0);
    }

    #[test]
    fn test_cosmic_ray_detect_glitch() {
        let mut data = vec![0.0; 1000];
        // Insert a sudden glitch
        data[500] = 100.0;
        data[501] = 50.0;

        let det = CosmicRayDetector::with_params(5.0, 1, 3, 50);
        let flags = det.detect_single_channel(&data);
        assert!(!flags.is_empty());
        // Should detect near index 500
        assert!(flags.iter().any(|&f| (f as i64 - 500).abs() < 5));
    }

    #[test]
    fn test_cosmic_ray_no_false_positives() {
        // Gentle signal should not trigger
        let data: Vec<f64> = (0..1000).map(|k| 0.01 * (k as f64 * 0.1).sin()).collect();
        let det = CosmicRayDetector::with_params(10.0, 1, 2, 50);
        let flags = det.detect_single_channel(&data);
        assert!(flags.is_empty());
    }

    #[test]
    fn test_cosmic_ray_coincident() {
        let mut ch1 = vec![0.0; 500];
        let mut ch2 = vec![0.0; 500];
        let mut ch3 = vec![0.0; 500];

        // Cosmic ray hit at sample 250 in all channels
        ch1[250] = 100.0;
        ch2[250] = 80.0;
        ch3[250] = 90.0;

        let det = CosmicRayDetector::with_params(5.0, 3, 3, 50);
        let flags = det.detect_coincident(&[&ch1, &ch2, &ch3]);
        assert!(!flags.is_empty());
    }

    // --- Resonator tracker tests ---

    #[test]
    fn test_tracker_first_sweep() {
        let mut tracker = ResonatorTracker::new(1e6);
        let freqs = vec![4.0e9, 5.0e9, 6.0e9];
        let ids = tracker.update_sweep(&freqs, 0.1);
        assert_eq!(ids.len(), 3);
        assert_eq!(tracker.num_resonators(), 3);
    }

    #[test]
    fn test_tracker_second_sweep_match() {
        let mut tracker = ResonatorTracker::new(1e6);
        let freqs1 = vec![4.0e9, 5.0e9, 6.0e9];
        let ids1 = tracker.update_sweep(&freqs1, 0.1);

        // Second sweep with small shifts
        let freqs2 = vec![4.0001e9, 5.0001e9, 6.0001e9];
        let ids2 = tracker.update_sweep(&freqs2, 0.2);

        // IDs should match
        assert_eq!(ids1, ids2);
        assert_eq!(tracker.num_resonators(), 3);
    }

    #[test]
    fn test_tracker_new_resonator() {
        let mut tracker = ResonatorTracker::new(1e6);
        let freqs1 = vec![4.0e9, 5.0e9];
        tracker.update_sweep(&freqs1, 0.1);

        // New resonator appears
        let freqs2 = vec![4.0e9, 5.0e9, 6.0e9];
        tracker.update_sweep(&freqs2, 0.2);
        assert_eq!(tracker.num_resonators(), 3);
    }

    #[test]
    fn test_tracker_predict() {
        let mut tracker = ResonatorTracker::new(1e6);
        tracker.update_sweep(&[5.0e9], 0.1);
        tracker.update_sweep(&[5.001e9], 0.2);

        let predictions = tracker.predict_frequencies(0.3);
        assert_eq!(predictions.len(), 1);
        // Should extrapolate linearly
        assert!((predictions[0].1 - 5.002e9).abs() < 1e6);
    }

    #[test]
    fn test_tracker_find() {
        let mut tracker = ResonatorTracker::new(1e6);
        let ids = tracker.update_sweep(&[5.0e9], 0.1);
        let found = tracker.find(ids[0]);
        assert!(found.is_some());
        assert!(approx_eq(found.unwrap().frequency, 5.0e9, 1.0));
    }

    // --- Mattis-Bardeen tests ---

    #[test]
    fn test_mattis_bardeen_aluminum() {
        let mb = MattisBardeen::aluminum();
        assert!(approx_eq(mb.tc, 1.2, 0.01));
        assert!(mb.delta_0 > 0.0);
    }

    #[test]
    fn test_mattis_bardeen_niobium() {
        let mb = MattisBardeen::niobium();
        assert!(approx_eq(mb.tc, 9.3, 0.01));
    }

    #[test]
    fn test_mattis_bardeen_titanium_nitride() {
        let mb = MattisBardeen::titanium_nitride();
        assert!(approx_eq(mb.tc, 4.5, 0.01));
    }

    #[test]
    fn test_gap_at_zero_temp() {
        let mb = MattisBardeen::aluminum();
        let gap = mb.gap_at_temperature(0.001);
        assert!(approx_eq(gap, mb.delta_0, mb.delta_0 * 0.01));
    }

    #[test]
    fn test_gap_at_tc() {
        let mb = MattisBardeen::aluminum();
        let gap = mb.gap_at_temperature(mb.tc);
        assert!(approx_eq(gap, 0.0, 1e-25));
    }

    #[test]
    fn test_gap_above_tc() {
        let mb = MattisBardeen::aluminum();
        let gap = mb.gap_at_temperature(mb.tc + 1.0);
        assert!(approx_eq(gap, 0.0, 1e-30));
    }

    #[test]
    fn test_quasiparticle_density_low_temp() {
        let mb = MattisBardeen::aluminum();
        let nqp_low = mb.quasiparticle_density(0.1);
        let nqp_high = mb.quasiparticle_density(0.5);
        // Higher temperature => more quasiparticles
        assert!(nqp_high > nqp_low);
    }

    #[test]
    fn test_sigma2_dominates_at_low_temp() {
        let mb = MattisBardeen::aluminum();
        let s1 = mb.sigma1_ratio(0.1, 5.0e9);
        let s2 = mb.sigma2_ratio(0.1, 5.0e9);
        // At low T, σ2 >> σ1 (mostly reactive/lossless)
        assert!(s2 > s1);
    }

    #[test]
    fn test_sigma2_positive() {
        let mb = MattisBardeen::aluminum();
        let s2 = mb.sigma2_ratio(0.1, 5.0e9);
        assert!(s2 > 0.0);
    }

    #[test]
    fn test_sigma1_non_negative() {
        let mb = MattisBardeen::aluminum();
        let s1 = mb.sigma1_ratio(0.1, 5.0e9);
        assert!(s1 >= 0.0);
    }

    #[test]
    fn test_surface_impedance() {
        let mb = MattisBardeen::aluminum();
        let (rs, xs) = mb.surface_impedance(0.1, 5.0e9, 20e-9);
        assert!(rs >= 0.0);
        assert!(xs >= 0.0);
    }

    #[test]
    fn test_kinetic_inductance_fraction() {
        let mb = MattisBardeen::aluminum();
        let alpha = mb.kinetic_inductance_fraction(0.1, 5.0e9, 1e-12);
        assert!(alpha >= 0.0);
        assert!(alpha <= 1.0);
    }

    // --- Pulse detection tests ---

    #[test]
    fn test_pulse_template_generation() {
        let template = generate_pulse_template(2.0, 50.0, 200);
        assert_eq!(template.len(), 200);
        // Peak should be 1.0 (normalized)
        let peak = template.iter().cloned().fold(0.0_f64, f64::max);
        assert!(approx_eq(peak, 1.0, 0.01));
        // Template starts near zero
        assert!(template[0] < 0.1);
    }

    #[test]
    fn test_pulse_template_decay() {
        let template = generate_pulse_template(2.0, 50.0, 200);
        // Tail should decay
        assert!(template[199] < template[10]);
    }

    #[test]
    fn test_detect_pulses_in_clean_data() {
        // No pulses in constant data
        let data = vec![0.0; 1000];
        let config = PulseDetectorConfig::default_optical();
        let pulses = detect_photon_pulses(&data, &config);
        assert!(pulses.is_empty());
    }

    #[test]
    fn test_detect_single_pulse() {
        let mut data = vec![0.0; 1000];
        // Insert a pulse at sample 500
        let template = generate_pulse_template(2.0, 50.0, 100);
        for (k, &v) in template.iter().enumerate() {
            data[500 + k] = v * 10.0; // 10x above noise
        }

        let config = PulseDetectorConfig {
            threshold_sigma: 3.0,
            min_separation: 50,
            pre_trigger: 10,
            post_trigger: 150,
            tau_qp_estimate: 50.0,
        };
        let pulses = detect_photon_pulses(&data, &config);
        assert!(!pulses.is_empty());
    }

    #[test]
    fn test_energy_histogram() {
        let mut hist = EnergyHistogram::new(0.0, 10.0, 10);
        hist.add_event(5.5);
        hist.add_event(5.7);
        hist.add_event(5.3);
        assert_eq!(hist.total_events, 3);
        let centers = hist.bin_centers();
        assert_eq!(centers.len(), 10);
    }

    #[test]
    fn test_energy_histogram_resolution() {
        let mut hist = EnergyHistogram::new(0.0, 10.0, 100);
        // Add events clustered around 5.0
        for k in 0..100 {
            let energy = 5.0 + 0.1 * (k as f64 / 100.0 - 0.5);
            hist.add_event(energy);
        }
        let fwhm = hist.energy_resolution();
        assert!(fwhm > 0.0);
        assert!(fwhm < 1.0); // Should be narrow
    }

    // --- MKID Readout Processor tests ---

    #[test]
    fn test_mkid_processor_creation() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 3, 1000.0);
        let resonators = vec![
            ResonatorParams::new(100.0, 1e5, 5e4, 0.0),
            ResonatorParams::new(150.0, 1e5, 5e4, 0.0),
            ResonatorParams::new(200.0, 1e5, 5e4, 0.0),
        ];
        let mb = MattisBardeen::aluminum();
        let proc = MkidReadoutProcessor::new(comb, resonators, mb, 0.1);
        assert_eq!(proc.photon_count(0), 0);
        assert_eq!(proc.photon_count(1), 0);
        assert_eq!(proc.photon_count(2), 0);
    }

    #[test]
    fn test_mkid_processor_nqp() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 1, 1000.0);
        let resonators = vec![ResonatorParams::new(150.0, 1e5, 5e4, 0.0)];
        let mb = MattisBardeen::aluminum();
        let proc = MkidReadoutProcessor::new(comb, resonators, mb, 0.1);
        let nqp = proc.expected_nqp();
        assert!(nqp >= 0.0);
    }

    #[test]
    fn test_mkid_processor_channel_nep() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 1, 1000.0);
        let resonators = vec![ResonatorParams::new(150.0, 1e5, 5e4, 0.0)];
        let mb = MattisBardeen::aluminum();
        let proc = MkidReadoutProcessor::new(comb, resonators, mb, 0.1);
        let nep = proc.channel_nep(1e-18, 1e6);
        assert!(nep > 0.0);
        assert!(nep < 1e-10);
    }

    #[test]
    fn test_mkid_set_duffing() {
        let comb = FrequencyComb::uniform(100.0, 200.0, 2, 1000.0);
        let resonators = vec![
            ResonatorParams::new(100.0, 1e5, 5e4, 0.0),
            ResonatorParams::new(200.0, 1e5, 5e4, 0.0),
        ];
        let mb = MattisBardeen::aluminum();
        let mut proc = MkidReadoutProcessor::new(comb, resonators, mb, 0.1);
        let duffing = DuffingCorrection::new(-1e-6, 100.0, 1e5);
        proc.set_duffing_correction(0, duffing);
        assert!(proc.duffing_corrections[0].is_some());
        assert!(proc.duffing_corrections[1].is_none());
    }

    // --- Linspace helper test ---

    #[test]
    fn test_linspace() {
        let v = linspace(0.0, 10.0, 11);
        assert_eq!(v.len(), 11);
        assert!(approx_eq(v[0], 0.0, TOL));
        assert!(approx_eq(v[10], 10.0, TOL));
        assert!(approx_eq(v[5], 5.0, TOL));
    }

    #[test]
    fn test_linspace_single() {
        let v = linspace(5.0, 10.0, 1);
        assert_eq!(v.len(), 1);
        assert!(approx_eq(v[0], 5.0, TOL));
    }

    #[test]
    fn test_pulse_config_submm() {
        let config = PulseDetectorConfig::default_submm();
        assert!(config.threshold_sigma > 0.0);
        assert!(config.pre_trigger > 0);
        assert!(config.post_trigger > config.pre_trigger);
    }

    #[test]
    fn test_noise_type_enum() {
        let nt = NoiseType::Amplifier;
        assert_eq!(nt, NoiseType::Amplifier);
        assert_ne!(nt, NoiseType::Tls);
    }
}
