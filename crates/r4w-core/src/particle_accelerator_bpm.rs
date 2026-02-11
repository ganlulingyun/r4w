//! Beam Position Monitor (BPM) Signal Processing for Particle Accelerators
//!
//! Extracts beam position, intensity, and timing from pickup electrode signals.
//! Implements the standard difference-over-sum algorithm for button BPMs, plus
//! orbit analysis, betatron tune extraction, emittance measurement, and
//! orbit correction matrix computation.
//!
//! ## Background
//!
//! A Beam Position Monitor (BPM) uses four button electrodes arranged around the
//! beam pipe (top, bottom, left, right). The induced signal on each electrode is
//! proportional to beam intensity and inversely related to the distance from the
//! beam to that electrode. By computing difference-over-sum ratios, the beam's
//! transverse position can be determined.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::particle_accelerator_bpm::{
//!     BpmConfig, BpmProcessor, difference_over_sum,
//!     position_from_electrodes, generate_bpm_signals,
//! };
//!
//! // Generate synthetic signals for a beam at (2.0, -1.5) mm
//! let (top, bottom, left, right) = generate_bpm_signals(2.0, -1.5, 1.0, 35.0, 0.0);
//!
//! // Recover position
//! let k = 35.0 / 1.0; // simplified sensitivity factor
//! let (x, y) = position_from_electrodes(top, bottom, left, right, k, k);
//! assert!((x - 2.0).abs() < 0.5);
//! assert!((y - (-1.5)).abs() < 0.5);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration & core types
// ---------------------------------------------------------------------------

/// Configuration for a Beam Position Monitor system.
#[derive(Debug, Clone)]
pub struct BpmConfig {
    /// Distance between opposing electrode centres (mm).
    pub electrode_spacing_mm: f64,
    /// Inner aperture radius of the beam pipe (mm). Default 35.0.
    pub aperture_radius_mm: f64,
    /// ADC sample rate (Hz).
    pub sample_rate_hz: f64,
    /// Number of pickup electrodes (typically 4 for a button BPM).
    pub num_electrodes: usize,
    /// RF frequency of the circulating beam (Hz).
    pub beam_frequency_hz: f64,
}

impl Default for BpmConfig {
    fn default() -> Self {
        Self {
            electrode_spacing_mm: 70.0,
            aperture_radius_mm: 35.0,
            sample_rate_hz: 125.0e6,
            num_electrodes: 4,
            beam_frequency_hz: 352.2e6,
        }
    }
}

/// Measured beam position and associated quantities for one turn.
#[derive(Debug, Clone, Copy)]
pub struct BeamPosition {
    /// Horizontal position (mm).
    pub x_mm: f64,
    /// Vertical position (mm).
    pub y_mm: f64,
    /// Beam intensity (arbitrary, proportional to current).
    pub intensity: f64,
    /// RF phase of the bunch centroid (degrees).
    pub phase_deg: f64,
    /// Horizontal position RMS spread (mm).
    pub sigma_x_mm: f64,
    /// Vertical position RMS spread (mm).
    pub sigma_y_mm: f64,
}

// ---------------------------------------------------------------------------
// BpmProcessor
// ---------------------------------------------------------------------------

/// Main processor that converts raw electrode waveforms into beam parameters.
#[derive(Debug, Clone)]
pub struct BpmProcessor {
    config: BpmConfig,
    /// Horizontal sensitivity factor (mm).
    k_x: f64,
    /// Vertical sensitivity factor (mm).
    k_y: f64,
}

impl BpmProcessor {
    /// Create a new BPM processor from the given configuration.
    pub fn new(config: BpmConfig) -> Self {
        let k = compute_sensitivity_factor(config.aperture_radius_mm);
        Self {
            k_x: k,
            k_y: k,
            config,
        }
    }

    /// Process one turn of electrode data and return the beam position.
    ///
    /// `electrode_signals` must contain one `Vec<f64>` per electrode, in the
    /// order: top, bottom, left, right. Each inner vector holds the ADC
    /// samples captured during one revolution.
    pub fn process_turn(&self, electrode_signals: &[Vec<f64>]) -> BeamPosition {
        assert!(
            electrode_signals.len() >= 4,
            "Need at least 4 electrode signals"
        );

        // Peak-detect each electrode (use absolute-value peak as amplitude proxy)
        let peaks: Vec<f64> = electrode_signals
            .iter()
            .map(|sig| {
                sig.iter()
                    .map(|&s| s.abs())
                    .fold(0.0_f64, f64::max)
            })
            .collect();

        let top = peaks[0];
        let bottom = peaks[1];
        let left = peaks[2];
        let right = peaks[3];

        let (x_mm, y_mm) =
            position_from_electrodes(top, bottom, left, right, self.k_x, self.k_y);

        let intensity = beam_intensity_from_sum(&peaks);

        // Phase estimation: find peak sample index in the sum signal, convert to degrees
        let sum_signal: Vec<f64> = {
            let n = electrode_signals[0].len();
            (0..n)
                .map(|i| electrode_signals.iter().map(|s| s[i]).sum::<f64>())
                .collect()
        };
        let peak_idx = sum_signal
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);
        let samples_per_period = self.config.sample_rate_hz / self.config.beam_frequency_hz;
        let phase_deg = (peak_idx as f64 / samples_per_period) * 360.0 % 360.0;

        // Estimate position spread from sample-by-sample positions
        let n = electrode_signals[0].len();
        let mut x_vals = Vec::with_capacity(n);
        let mut y_vals = Vec::with_capacity(n);
        for i in 0..n {
            let t = electrode_signals[0][i].abs();
            let b = electrode_signals[1][i].abs();
            let l = electrode_signals[2][i].abs();
            let r = electrode_signals[3][i].abs();
            let sum = t + b + l + r;
            if sum > 1e-12 {
                let xi = self.k_x * (r - l) / sum;
                let yi = self.k_y * (t - b) / sum;
                x_vals.push(xi);
                y_vals.push(yi);
            }
        }
        let sigma_x_mm = rms_spread(&x_vals);
        let sigma_y_mm = rms_spread(&y_vals);

        BeamPosition {
            x_mm,
            y_mm,
            intensity,
            phase_deg,
            sigma_x_mm,
            sigma_y_mm,
        }
    }
}

/// Compute the RMS spread (standard deviation) of a slice of values.
fn rms_spread(values: &[f64]) -> f64 {
    if values.len() < 2 {
        return 0.0;
    }
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|&v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0);
    variance.sqrt()
}

// ---------------------------------------------------------------------------
// Fundamental BPM algorithms
// ---------------------------------------------------------------------------

/// Difference-over-sum: `(A - B) / (A + B)`.
///
/// This is the fundamental BPM position computation. The result is
/// dimensionless and lies in \[-1, 1\] when both inputs are non-negative.
/// Returns 0.0 when A + B is effectively zero.
pub fn difference_over_sum(a: f64, b: f64) -> f64 {
    let sum = a + b;
    if sum.abs() < 1e-15 {
        0.0
    } else {
        (a - b) / sum
    }
}

/// Compute horizontal (x) and vertical (y) beam position from four button
/// electrode amplitudes and their sensitivity factors.
///
/// Standard formula:
/// ```text
/// x = k_x * (Right - Left) / (Top + Bottom + Left + Right)
/// y = k_y * (Top - Bottom) / (Top + Bottom + Left + Right)
/// ```
pub fn position_from_electrodes(
    top: f64,
    bottom: f64,
    left: f64,
    right: f64,
    k_x: f64,
    k_y: f64,
) -> (f64, f64) {
    let sum = top + bottom + left + right;
    if sum.abs() < 1e-15 {
        return (0.0, 0.0);
    }
    let x = k_x * (right - left) / sum;
    let y = k_y * (top - bottom) / sum;
    (x, y)
}

/// Compute the sensitivity factor `k` from the aperture radius.
///
/// For a standard four-button BPM the geometric sensitivity factor is
/// approximately `k = aperture_radius / coverage_factor` where the coverage
/// factor accounts for the finite button solid angle. A typical coverage
/// factor for standard button BPMs is ~1.0 (i.e. k ~ aperture radius in mm).
///
/// More precisely, for circular electrodes subtending an angle `alpha` at
/// the pipe centre, the linearised sensitivity is
/// `k = (pi * a) / (2 * sin(alpha))` where `a` is the aperture radius.
/// We use a default button half-angle of pi/4 which gives k = a * pi / (2 * sin(pi/4)).
pub fn compute_sensitivity_factor(aperture_radius_mm: f64) -> f64 {
    let alpha = PI / 4.0; // button half-angle
    aperture_radius_mm * PI / (2.0 * alpha.sin())
}

// ---------------------------------------------------------------------------
// Orbit analysis
// ---------------------------------------------------------------------------

/// Average beam position over `num_turns` turns (closed orbit).
///
/// Takes a slice of (x, y) position pairs and returns the mean position.
/// If `positions.len() < num_turns`, all available data are used.
pub fn closed_orbit_average(positions: &[(f64, f64)], num_turns: usize) -> (f64, f64) {
    if positions.is_empty() {
        return (0.0, 0.0);
    }
    let n = positions.len().min(num_turns);
    let (sum_x, sum_y) = positions[..n]
        .iter()
        .fold((0.0, 0.0), |(sx, sy), &(x, y)| (sx + x, sy + y));
    (sum_x / n as f64, sum_y / n as f64)
}

/// Extract fractional betatron tune from turn-by-turn position data.
///
/// Uses a discrete Fourier transform to find the peak frequency in the
/// turn-by-turn oscillation. The fractional tune `Q_frac` is the peak
/// frequency expressed as a fraction of the revolution frequency.
///
/// The DFT is computed from scratch (Goertzel-like sweep) to avoid
/// external FFT dependencies.
pub fn betatron_tune(positions: &[f64]) -> f64 {
    let n = positions.len();
    if n < 4 {
        return 0.0;
    }

    // Remove DC component
    let mean = positions.iter().sum::<f64>() / n as f64;
    let centered: Vec<f64> = positions.iter().map(|&p| p - mean).collect();

    // Sweep frequencies from 0 to 0.5 (Nyquist) in steps of 1/N
    // and find the peak magnitude.
    let n_f64 = n as f64;
    let mut max_mag = 0.0_f64;
    let mut max_freq = 0.0_f64;

    // Number of frequency bins to test: N/2 (positive frequencies only)
    let num_bins = n / 2;
    for k in 1..=num_bins {
        // DFT at bin k: X[k] = sum_n x[n] * exp(-j 2pi k n / N)
        let freq = k as f64 / n_f64;
        let (mut re, mut im) = (0.0, 0.0);
        for (i, &x) in centered.iter().enumerate() {
            let angle = -2.0 * PI * freq * i as f64;
            re += x * angle.cos();
            im += x * angle.sin();
        }
        let mag = (re * re + im * im).sqrt();
        if mag > max_mag {
            max_mag = mag;
            max_freq = freq;
        }
    }

    // Refine with parabolic interpolation around the peak bin
    let peak_bin = (max_freq * n_f64).round() as usize;
    if peak_bin >= 2 && peak_bin <= num_bins {
        let mag_at = |k: usize| -> f64 {
            let freq = k as f64 / n_f64;
            let (mut re, mut im) = (0.0, 0.0);
            for (i, &x) in centered.iter().enumerate() {
                let angle = -2.0 * PI * freq * i as f64;
                re += x * angle.cos();
                im += x * angle.sin();
            }
            (re * re + im * im).sqrt()
        };
        let m_prev = mag_at(peak_bin - 1);
        let m_peak = mag_at(peak_bin);
        let m_next = mag_at(peak_bin + 1);
        let denom = 2.0 * (2.0 * m_peak - m_prev - m_next);
        if denom.abs() > 1e-15 {
            let delta = (m_prev - m_next) / denom;
            max_freq = (peak_bin as f64 + delta) / n_f64;
        }
    }

    max_freq
}

/// Beam intensity proportional to total beam current.
///
/// The sum of all electrode signals is proportional to beam charge.
pub fn beam_intensity_from_sum(electrode_signals: &[f64]) -> f64 {
    electrode_signals.iter().sum::<f64>()
}

// ---------------------------------------------------------------------------
// Signal generation (for testing / simulation)
// ---------------------------------------------------------------------------

/// Generate synthetic button BPM electrode signals for a beam at
/// position (`x_mm`, `y_mm`) with given intensity and aperture.
///
/// Returns `(top, bottom, left, right)` electrode amplitudes.
///
/// The model assumes a coasting beam with image-charge coupling. For a
/// beam displaced by `(x, y)` inside a circular pipe of radius `a`, the
/// signal on each electrode is approximately:
///
/// ```text
/// V_electrode ~ intensity * (1 + 2*displacement_component / a)
/// ```
///
/// Gaussian noise of amplitude `noise_level` is added deterministically
/// using a simple LCG for reproducibility (no `rand` dependency).
pub fn generate_bpm_signals(
    x_mm: f64,
    y_mm: f64,
    intensity: f64,
    aperture_mm: f64,
    noise_level: f64,
) -> (f64, f64, f64, f64) {
    let a = aperture_mm;
    // Linear approximation for small displacements
    let base = intensity / 4.0;

    // Each electrode sees a signal that increases as the beam approaches it
    let right = base * (1.0 + 2.0 * x_mm / a);
    let left = base * (1.0 - 2.0 * x_mm / a);
    let top = base * (1.0 + 2.0 * y_mm / a);
    let bottom = base * (1.0 - 2.0 * y_mm / a);

    if noise_level.abs() < 1e-15 {
        return (top, bottom, left, right);
    }

    // Deterministic pseudo-random noise using a simple LCG
    let mut seed: u64 = (x_mm.to_bits() ^ y_mm.to_bits()) | 1;
    let lcg_next = |s: &mut u64| -> f64 {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        // Map to [-1, 1]
        ((*s >> 33) as f64 / (1u64 << 31) as f64) - 1.0
    };

    let n_top = noise_level * lcg_next(&mut seed);
    let n_bottom = noise_level * lcg_next(&mut seed);
    let n_left = noise_level * lcg_next(&mut seed);
    let n_right = noise_level * lcg_next(&mut seed);

    (top + n_top, bottom + n_bottom, left + n_left, right + n_right)
}

// ---------------------------------------------------------------------------
// Orbit correction
// ---------------------------------------------------------------------------

/// Compute orbit correction strengths via pseudo-inverse of the response
/// matrix.
///
/// Given `bpm_positions` (measured orbit distortion at each BPM) and
/// `corrector_responses` (the response of each corrector at each BPM, i.e.
/// the orbit response matrix columns), compute corrector kicks that best
/// flatten the orbit.
///
/// This uses a simple least-squares solution via the normal equations
/// (A^T A) x = A^T b, solved with Cholesky-like Gaussian elimination.
/// For production use with many correctors, a full SVD with singular-value
/// truncation would be preferred; this implementation is adequate for
/// small-to-medium systems and avoids external linear algebra dependencies.
pub fn orbit_correction_matrix(
    bpm_positions: &[(f64, f64)],
    corrector_responses: &[Vec<(f64, f64)>],
) -> Vec<f64> {
    let n_bpm = bpm_positions.len();
    let n_corr = corrector_responses.len();

    if n_bpm == 0 || n_corr == 0 {
        return vec![0.0; n_corr];
    }

    // Flatten to 1-D problem in x for simplicity, then y.
    // Build response matrix A (n_bpm x n_corr) for x-plane
    // and measurement vector b (n_bpm) for x-plane.
    let m = 2 * n_bpm; // rows (x and y interleaved)
    let n = n_corr; // columns

    // Build A (m x n) and b (m x 1)
    let mut a = vec![0.0; m * n];
    let mut b = vec![0.0; m];

    for i in 0..n_bpm {
        b[2 * i] = -bpm_positions[i].0; // negative: we want to cancel the distortion
        b[2 * i + 1] = -bpm_positions[i].1;
    }

    for j in 0..n_corr {
        let resp = &corrector_responses[j];
        for i in 0..n_bpm.min(resp.len()) {
            a[2 * i * n + j] = resp[i].0;
            a[(2 * i + 1) * n + j] = resp[i].1;
        }
    }

    // Normal equations: (A^T A) x = A^T b
    // A^T A is n x n, A^T b is n x 1
    let mut ata = vec![0.0; n * n];
    let mut atb = vec![0.0; n];

    for i in 0..n {
        for j in 0..n {
            let mut sum = 0.0;
            for k in 0..m {
                sum += a[k * n + i] * a[k * n + j];
            }
            ata[i * n + j] = sum;
        }
        let mut sum = 0.0;
        for k in 0..m {
            sum += a[k * n + i] * b[k];
        }
        atb[i] = sum;
    }

    // Solve via Gaussian elimination with partial pivoting
    solve_linear_system(&mut ata, &mut atb, n)
}

/// Gaussian elimination with partial pivoting for an n x n system.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Regularisation: add small diagonal to prevent singularity
    for i in 0..n {
        a[i * n + i] += 1e-10;
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            b.swap(col, max_row);
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        // Eliminate below
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * x[j];
        }
        let diag = a[i * n + i];
        x[i] = if diag.abs() > 1e-30 { sum / diag } else { 0.0 };
    }

    x
}

// ---------------------------------------------------------------------------
// Beam quality measurements
// ---------------------------------------------------------------------------

/// Compute RMS beam emittance from position and angle distributions.
///
/// The geometric RMS emittance is:
/// ```text
/// epsilon = sqrt( <x^2><x'^2> - <x*x'>^2 )
/// ```
/// where `<>` denotes the mean over the distribution.
pub fn compute_emittance(positions: &[f64], angles: &[f64]) -> f64 {
    let n = positions.len().min(angles.len());
    if n < 2 {
        return 0.0;
    }
    let nf = n as f64;

    let mean_x = positions[..n].iter().sum::<f64>() / nf;
    let mean_xp = angles[..n].iter().sum::<f64>() / nf;

    let mut sum_x2 = 0.0;
    let mut sum_xp2 = 0.0;
    let mut sum_x_xp = 0.0;

    for i in 0..n {
        let dx = positions[i] - mean_x;
        let dxp = angles[i] - mean_xp;
        sum_x2 += dx * dx;
        sum_xp2 += dxp * dxp;
        sum_x_xp += dx * dxp;
    }

    let var_x = sum_x2 / nf;
    let var_xp = sum_xp2 / nf;
    let cov = sum_x_xp / nf;

    let det = var_x * var_xp - cov * cov;
    if det < 0.0 {
        0.0
    } else {
        det.sqrt()
    }
}

/// Measure dispersion from position changes versus momentum offset.
///
/// Dispersion `D = dx / d(dp/p)`. Computed as the slope of a linear
/// regression of position versus fractional momentum offset.
pub fn dispersion_measurement(positions: &[f64], momentum_offsets: &[f64]) -> f64 {
    let n = positions.len().min(momentum_offsets.len());
    if n < 2 {
        return 0.0;
    }
    let nf = n as f64;

    let mean_dp = momentum_offsets[..n].iter().sum::<f64>() / nf;
    let mean_x = positions[..n].iter().sum::<f64>() / nf;

    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..n {
        let ddp = momentum_offsets[i] - mean_dp;
        let dx = positions[i] - mean_x;
        num += ddp * dx;
        den += ddp * ddp;
    }

    if den.abs() < 1e-30 {
        0.0
    } else {
        num / den
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // --- difference_over_sum ---

    #[test]
    fn test_dos_equal_signals() {
        assert!((difference_over_sum(1.0, 1.0)).abs() < TOL);
    }

    #[test]
    fn test_dos_a_only() {
        assert!((difference_over_sum(1.0, 0.0) - 1.0).abs() < TOL);
    }

    #[test]
    fn test_dos_b_only() {
        assert!((difference_over_sum(0.0, 1.0) - (-1.0)).abs() < TOL);
    }

    #[test]
    fn test_dos_both_zero() {
        assert!((difference_over_sum(0.0, 0.0)).abs() < TOL);
    }

    #[test]
    fn test_dos_large_values() {
        let a = 1e12;
        let b = 1e12 + 1.0;
        let result = difference_over_sum(a, b);
        assert!(result < 0.0);
        assert!(result > -1.0);
    }

    #[test]
    fn test_dos_symmetry() {
        let r1 = difference_over_sum(3.0, 7.0);
        let r2 = difference_over_sum(7.0, 3.0);
        assert!((r1 + r2).abs() < TOL);
    }

    // --- position_from_electrodes ---

    #[test]
    fn test_centered_beam() {
        let (x, y) = position_from_electrodes(1.0, 1.0, 1.0, 1.0, 35.0, 35.0);
        assert!(x.abs() < TOL);
        assert!(y.abs() < TOL);
    }

    #[test]
    fn test_beam_displaced_right() {
        // Right electrode stronger
        let (x, y) = position_from_electrodes(1.0, 1.0, 0.8, 1.2, 35.0, 35.0);
        assert!(x > 0.0, "Beam should be to the right, got x={}", x);
        assert!(y.abs() < TOL, "Beam should be centred vertically, got y={}", y);
    }

    #[test]
    fn test_beam_displaced_up() {
        let (x, y) = position_from_electrodes(1.2, 0.8, 1.0, 1.0, 35.0, 35.0);
        assert!(x.abs() < TOL);
        assert!(y > 0.0, "Beam should be up, got y={}", y);
    }

    #[test]
    fn test_position_zero_sum() {
        let (x, y) = position_from_electrodes(0.0, 0.0, 0.0, 0.0, 35.0, 35.0);
        assert!(x.abs() < TOL);
        assert!(y.abs() < TOL);
    }

    #[test]
    fn test_position_different_k_factors() {
        let (x1, y1) = position_from_electrodes(1.0, 1.0, 0.9, 1.1, 35.0, 35.0);
        let (x2, y2) = position_from_electrodes(1.0, 1.0, 0.9, 1.1, 70.0, 70.0);
        assert!((x2 - 2.0 * x1).abs() < TOL);
        assert!((y2 - 2.0 * y1).abs() < TOL);
    }

    // --- compute_sensitivity_factor ---

    #[test]
    fn test_sensitivity_factor_positive() {
        let k = compute_sensitivity_factor(35.0);
        assert!(k > 0.0);
    }

    #[test]
    fn test_sensitivity_factor_scales_with_aperture() {
        let k1 = compute_sensitivity_factor(35.0);
        let k2 = compute_sensitivity_factor(70.0);
        assert!((k2 / k1 - 2.0).abs() < TOL);
    }

    #[test]
    fn test_sensitivity_factor_value() {
        // k = a * pi / (2 * sin(pi/4)) = a * pi / sqrt(2)
        let a = 35.0;
        let expected = a * PI / (2.0_f64.sqrt());
        let k = compute_sensitivity_factor(a);
        assert!((k - expected).abs() < TOL);
    }

    // --- closed_orbit_average ---

    #[test]
    fn test_orbit_average_single() {
        let positions = vec![(1.0, 2.0)];
        let (x, y) = closed_orbit_average(&positions, 10);
        assert!((x - 1.0).abs() < TOL);
        assert!((y - 2.0).abs() < TOL);
    }

    #[test]
    fn test_orbit_average_symmetric() {
        let positions = vec![(1.0, 1.0), (-1.0, -1.0)];
        let (x, y) = closed_orbit_average(&positions, 2);
        assert!(x.abs() < TOL);
        assert!(y.abs() < TOL);
    }

    #[test]
    fn test_orbit_average_num_turns_limit() {
        let positions = vec![(1.0, 0.0), (3.0, 0.0), (100.0, 100.0)];
        let (x, _) = closed_orbit_average(&positions, 2);
        assert!((x - 2.0).abs() < TOL);
    }

    #[test]
    fn test_orbit_average_empty() {
        let (x, y) = closed_orbit_average(&[], 10);
        assert!(x.abs() < TOL);
        assert!(y.abs() < TOL);
    }

    // --- betatron_tune ---

    #[test]
    fn test_tune_pure_sine() {
        // Inject a sine wave at fractional tune Q = 0.25
        let n = 256;
        let q = 0.25;
        let data: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * q * i as f64).sin())
            .collect();
        let measured_q = betatron_tune(&data);
        assert!(
            (measured_q - q).abs() < 0.01,
            "Expected tune ~{}, got {}",
            q,
            measured_q
        );
    }

    #[test]
    fn test_tune_different_frequency() {
        let n = 512;
        let q = 0.31;
        let data: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * q * i as f64).sin())
            .collect();
        let measured_q = betatron_tune(&data);
        assert!(
            (measured_q - q).abs() < 0.01,
            "Expected tune ~{}, got {}",
            q,
            measured_q
        );
    }

    #[test]
    fn test_tune_dc_signal() {
        let data = vec![5.0; 128];
        let q = betatron_tune(&data);
        // A DC signal has no oscillation; tune should be ~0
        assert!(q.abs() < 0.02, "DC signal tune should be ~0, got {}", q);
    }

    #[test]
    fn test_tune_short_data() {
        let data = vec![1.0, 2.0];
        let q = betatron_tune(&data);
        assert!(q.abs() < TOL, "Too-short data should return 0");
    }

    // --- beam_intensity_from_sum ---

    #[test]
    fn test_intensity_positive() {
        let signals = vec![1.0, 1.1, 0.9, 1.0];
        let intensity = beam_intensity_from_sum(&signals);
        assert!((intensity - 4.0).abs() < 0.2);
    }

    #[test]
    fn test_intensity_zero() {
        let signals = vec![0.0, 0.0, 0.0, 0.0];
        let intensity = beam_intensity_from_sum(&signals);
        assert!(intensity.abs() < TOL);
    }

    #[test]
    fn test_intensity_proportional() {
        let s1 = vec![1.0, 1.0, 1.0, 1.0];
        let s2 = vec![2.0, 2.0, 2.0, 2.0];
        let i1 = beam_intensity_from_sum(&s1);
        let i2 = beam_intensity_from_sum(&s2);
        assert!((i2 / i1 - 2.0).abs() < TOL);
    }

    // --- generate_bpm_signals ---

    #[test]
    fn test_generate_centered_beam() {
        let (t, b, l, r) = generate_bpm_signals(0.0, 0.0, 4.0, 35.0, 0.0);
        assert!((t - 1.0).abs() < TOL);
        assert!((b - 1.0).abs() < TOL);
        assert!((l - 1.0).abs() < TOL);
        assert!((r - 1.0).abs() < TOL);
    }

    #[test]
    fn test_generate_displaced_x() {
        let (t, b, l, r) = generate_bpm_signals(5.0, 0.0, 4.0, 35.0, 0.0);
        assert!(r > l, "Right should be stronger than left for +x");
        assert!((t - b).abs() < TOL, "Vertical should be symmetric");
    }

    #[test]
    fn test_generate_displaced_y() {
        let (t, b, l, r) = generate_bpm_signals(0.0, 5.0, 4.0, 35.0, 0.0);
        assert!(t > b, "Top should be stronger than bottom for +y");
        assert!((l - r).abs() < TOL, "Horizontal should be symmetric");
    }

    #[test]
    fn test_generate_roundtrip() {
        // Generate signals, then recover position
        let x_in = 3.0;
        let y_in = -2.0;
        let aperture = 35.0;
        let (t, b, l, r) = generate_bpm_signals(x_in, y_in, 4.0, aperture, 0.0);

        // Use the linearised model sensitivity: k = a/2 for this signal model
        // From the generate model: V = I/4 * (1 + 2*d/a)
        // So (R-L)/(T+B+L+R) = (2*x/a) * (I/4) * 2 / I = x/a  ... wait
        // Let's just use the signal model directly.
        // Sum = I, and (R-L)/Sum = 2*x/a * (1/4) * ... let me compute:
        //   R - L = I/4 * 4*x/a = I*x/a
        //   Sum = I
        //   So (R-L)/Sum = x/a, meaning k = a
        let k = aperture;
        let (x_out, y_out) = position_from_electrodes(t, b, l, r, k, k);
        assert!(
            (x_out - x_in).abs() < 0.1,
            "Round-trip x: expected {}, got {}",
            x_in,
            x_out
        );
        assert!(
            (y_out - y_in).abs() < 0.1,
            "Round-trip y: expected {}, got {}",
            y_in,
            y_out
        );
    }

    #[test]
    fn test_generate_with_noise() {
        let (t1, b1, l1, r1) = generate_bpm_signals(1.0, 1.0, 4.0, 35.0, 0.0);
        let (t2, b2, l2, r2) = generate_bpm_signals(1.0, 1.0, 4.0, 35.0, 0.01);
        // With noise, values should differ slightly
        let diff = (t1 - t2).abs() + (b1 - b2).abs() + (l1 - l2).abs() + (r1 - r2).abs();
        assert!(diff > 0.0, "Noise should change signals");
    }

    // --- compute_emittance ---

    #[test]
    fn test_emittance_zero_correlation() {
        // Positions and angles uncorrelated: emittance = sigma_x * sigma_xp
        let positions = vec![1.0, -1.0, 1.0, -1.0];
        let angles = vec![0.5, 0.5, -0.5, -0.5];
        let eps = compute_emittance(&positions, &angles);
        // sigma_x = 1.0, sigma_xp = 0.5, correlation = 0
        // emittance = sqrt(1.0 * 0.25 - 0) = 0.5
        assert!(
            (eps - 0.5).abs() < 0.1,
            "Emittance should be ~0.5, got {}",
            eps
        );
    }

    #[test]
    fn test_emittance_fully_correlated() {
        // If x and x' are perfectly correlated, emittance ~ 0
        let positions: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let angles: Vec<f64> = positions.iter().map(|&x| 2.0 * x).collect();
        let eps = compute_emittance(&positions, &angles);
        assert!(eps < 0.01, "Fully correlated should have ~0 emittance, got {}", eps);
    }

    #[test]
    fn test_emittance_empty() {
        assert!(compute_emittance(&[], &[]).abs() < TOL);
    }

    // --- dispersion_measurement ---

    #[test]
    fn test_dispersion_linear() {
        // D = 1.5 m: x = 1.5 * dp/p
        let dp: Vec<f64> = vec![-0.002, -0.001, 0.0, 0.001, 0.002];
        let x: Vec<f64> = dp.iter().map(|&d| 1.5 * d).collect();
        let d = dispersion_measurement(&x, &dp);
        assert!(
            (d - 1.5).abs() < 0.01,
            "Dispersion should be 1.5, got {}",
            d
        );
    }

    #[test]
    fn test_dispersion_with_offset() {
        // D = 2.0, but with a constant offset in position
        let dp: Vec<f64> = vec![-0.001, 0.0, 0.001, 0.002];
        let x: Vec<f64> = dp.iter().map(|&d| 5.0 + 2.0 * d).collect();
        let d = dispersion_measurement(&x, &dp);
        assert!(
            (d - 2.0).abs() < 0.01,
            "Dispersion should be 2.0, got {}",
            d
        );
    }

    #[test]
    fn test_dispersion_zero() {
        // No position change with momentum
        let dp = vec![-0.001, 0.0, 0.001];
        let x = vec![1.0, 1.0, 1.0];
        let d = dispersion_measurement(&x, &dp);
        assert!(d.abs() < 0.01, "Zero dispersion expected, got {}", d);
    }

    #[test]
    fn test_dispersion_empty() {
        assert!(dispersion_measurement(&[], &[]).abs() < TOL);
    }

    // --- orbit_correction_matrix ---

    #[test]
    fn test_orbit_correction_single_corrector() {
        // One BPM at x=1.0, one corrector with unit response
        let bpm = vec![(1.0, 0.0)];
        let resp = vec![vec![(1.0, 0.0)]];
        let kicks = orbit_correction_matrix(&bpm, &resp);
        assert_eq!(kicks.len(), 1);
        // Should kick to cancel +1.0mm distortion
        assert!(
            (kicks[0] - (-1.0)).abs() < 0.1,
            "Kick should be ~-1.0, got {}",
            kicks[0]
        );
    }

    #[test]
    fn test_orbit_correction_two_correctors() {
        let bpm = vec![(2.0, 0.0), (0.0, 3.0)];
        let resp = vec![
            vec![(1.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (0.0, 1.0)],
        ];
        let kicks = orbit_correction_matrix(&bpm, &resp);
        assert_eq!(kicks.len(), 2);
        // Corrector 1 should cancel x=2.0 at BPM 1
        assert!(
            (kicks[0] - (-2.0)).abs() < 0.5,
            "Kick[0] should be ~-2.0, got {}",
            kicks[0]
        );
    }

    #[test]
    fn test_orbit_correction_empty() {
        let kicks = orbit_correction_matrix(&[], &[]);
        assert!(kicks.is_empty());
    }

    // --- BpmProcessor ---

    #[test]
    fn test_processor_centered_beam() {
        let config = BpmConfig::default();
        let proc = BpmProcessor::new(config);

        // Generate 100 samples of a centred beam (all electrodes equal)
        let n = 100;
        let sig = vec![1.0; n];
        let signals = vec![sig.clone(), sig.clone(), sig.clone(), sig];

        let pos = proc.process_turn(&signals);
        assert!(
            pos.x_mm.abs() < 1.0,
            "x should be ~0, got {}",
            pos.x_mm
        );
        assert!(
            pos.y_mm.abs() < 1.0,
            "y should be ~0, got {}",
            pos.y_mm
        );
        assert!(pos.intensity > 0.0);
    }

    #[test]
    fn test_processor_displaced_beam() {
        let config = BpmConfig::default();
        let proc = BpmProcessor::new(config);

        let n = 100;
        // Beam displaced to the right: right electrode stronger
        let top = vec![1.0; n];
        let bottom = vec![1.0; n];
        let left = vec![0.8; n];
        let right = vec![1.2; n];
        let signals = vec![top, bottom, left, right];

        let pos = proc.process_turn(&signals);
        assert!(pos.x_mm > 0.0, "x should be positive, got {}", pos.x_mm);
    }

    // --- rms_spread ---

    #[test]
    fn test_rms_spread_constant() {
        let vals = vec![5.0; 100];
        let sigma = rms_spread(&vals);
        assert!(sigma.abs() < TOL, "Constant values should have zero spread");
    }

    #[test]
    fn test_rms_spread_known() {
        // [-1, 1]: mean=0, variance = ((1+1)/(2-1)) = 2, sigma = sqrt(2)
        let vals = vec![-1.0, 1.0];
        let sigma = rms_spread(&vals);
        let expected = 2.0_f64.sqrt();
        assert!(
            (sigma - expected).abs() < TOL,
            "Expected {}, got {}",
            expected,
            sigma
        );
    }
}
