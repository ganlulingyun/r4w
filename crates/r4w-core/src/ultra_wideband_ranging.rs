//! Ultra-Wideband (UWB) impulse radio for precision time-of-flight positioning
//! per IEEE 802.15.4a/z.
//!
//! This module provides UWB pulse generation, double-sided two-way ranging (DS-TWR),
//! first-path detection in multipath channels, trilateration, and GDOP calculation.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::ultra_wideband_ranging::{
//!     UwbPulseGenerator, PulseShape, TwoWayRangingEngine, FirstPathDetector, DetectionMethod,
//! };
//!
//! // Generate a Gaussian monocycle pulse
//! let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
//! let pulse = gen.generate_pulse(128, 10.0e9);
//! assert_eq!(pulse.len(), 128);
//!
//! // Double-sided two-way ranging
//! let engine = TwoWayRangingEngine::new();
//! // Symmetric exchange: 100 ns round-trip, 10 ns reply on each side
//! let result = engine.compute_range(100.0, 10.0, 100.0, 10.0);
//! // ToF should be ~45 ns => ~13.5 m
//! assert!((result.distance_m - 13.486).abs() < 0.1);
//!
//! // First-path detection
//! let detector = FirstPathDetector::new(DetectionMethod::Threshold, 0.5);
//! let cir = vec![(0.1, 0.0), (0.2, 0.0), (0.9, 0.0), (0.3, 0.0)];
//! let idx = detector.detect_first_path(&cir);
//! assert_eq!(idx, Some(2));
//! ```

use std::f64::consts::PI;

// ──────────────────────────── Pulse Generation ────────────────────────────

/// Supported UWB pulse shapes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PulseShape {
    /// Gaussian monocycle (first derivative of Gaussian).
    GaussianMonocycle,
    /// Rayleigh pulse (second derivative of Gaussian).
    Rayleigh,
    /// Raised-cosine pulse with roll-off factor derived from bandwidth.
    RaisedCosine,
}

/// Generates UWB impulse waveforms with configurable pulse shape and bandwidth.
#[derive(Debug, Clone)]
pub struct UwbPulseGenerator {
    /// Pulse shape variant.
    pub shape: PulseShape,
    /// Nominal bandwidth in GHz.
    pub bandwidth_ghz: f64,
}

impl UwbPulseGenerator {
    /// Create a new UWB pulse generator.
    ///
    /// # Arguments
    /// * `shape` - The desired pulse shape.
    /// * `bandwidth_ghz` - Nominal bandwidth in GHz (affects pulse width).
    pub fn new(shape: PulseShape, bandwidth_ghz: f64) -> Self {
        Self { shape, bandwidth_ghz }
    }

    /// Generate a baseband impulse waveform.
    ///
    /// Returns `num_samples` IQ samples at the given `sample_rate` (in Hz).
    /// The pulse is centred in the output buffer.
    pub fn generate_pulse(&self, num_samples: usize, sample_rate: f64) -> Vec<(f64, f64)> {
        if num_samples == 0 {
            return Vec::new();
        }

        // Pulse width parameter (sigma for Gaussian variants, symbol period for RC).
        // Derived from bandwidth: sigma ≈ 1 / (pi * BW)
        let bw_hz = self.bandwidth_ghz * 1.0e9;
        let sigma = 1.0 / (PI * bw_hz);
        let dt = 1.0 / sample_rate;
        let centre = num_samples as f64 / 2.0;

        let mut samples: Vec<(f64, f64)> = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let t = (i as f64 - centre) * dt;
            let val = match self.shape {
                PulseShape::GaussianMonocycle => {
                    // First derivative of Gaussian: p(t) = -t/σ² * exp(-t²/(2σ²))
                    let norm_t = t / sigma;
                    -norm_t * (-0.5 * norm_t * norm_t).exp()
                }
                PulseShape::Rayleigh => {
                    // Second derivative of Gaussian:
                    // p(t) = (1 - (t/σ)²) * exp(-t²/(2σ²))
                    let norm_t = t / sigma;
                    (1.0 - norm_t * norm_t) * (-0.5 * norm_t * norm_t).exp()
                }
                PulseShape::RaisedCosine => {
                    // Raised-cosine pulse: p(t) = sinc(t/T) * cos(π β t / T) / (1 - (2βt/T)²)
                    // T = 1/BW, β = 0.5 (moderate roll-off)
                    let t_sym = 1.0 / bw_hz;
                    let beta = 0.5;
                    let t_norm = t / t_sym;
                    let sinc_val = if t_norm.abs() < 1e-12 {
                        1.0
                    } else {
                        (PI * t_norm).sin() / (PI * t_norm)
                    };
                    let denom = 1.0 - (2.0 * beta * t_norm) * (2.0 * beta * t_norm);
                    let cos_val = (PI * beta * t_norm).cos();
                    if denom.abs() < 1e-12 {
                        sinc_val * PI / 4.0
                    } else {
                        sinc_val * cos_val / denom
                    }
                }
            };
            samples.push((val, 0.0));
        }

        // Normalize peak to 1.0
        let peak = samples
            .iter()
            .map(|(re, im)| (re * re + im * im).sqrt())
            .fold(0.0_f64, f64::max);
        if peak > 1e-30 {
            for s in &mut samples {
                s.0 /= peak;
                s.1 /= peak;
            }
        }

        samples
    }
}

// ──────────────────────────── Two-Way Ranging ────────────────────────────

/// Result of a DS-TWR measurement.
#[derive(Debug, Clone, Copy)]
pub struct RangingResult {
    /// Estimated one-way distance in metres.
    pub distance_m: f64,
    /// Estimated one-way time of flight in nanoseconds.
    pub tof_ns: f64,
    /// Estimated clock drift between initiator and responder in ppm.
    pub clock_drift_ppm: f64,
    /// Confidence metric in \[0, 1\] (higher is better).
    pub confidence: f64,
}

/// Double-sided two-way ranging engine with clock-drift compensation.
///
/// Uses the DS-TWR formula from IEEE 802.15.4z:
///
/// ```text
/// tof = (t_round1 * t_round2 - t_reply1 * t_reply2)
///     / (t_round1 + t_round2 + t_reply1 + t_reply2)
/// ```
#[derive(Debug, Clone)]
pub struct TwoWayRangingEngine {
    /// Speed of light in m/ns.
    speed_of_light_m_per_ns: f64,
}

impl TwoWayRangingEngine {
    /// Speed of light in m/s.
    const C: f64 = 299_792_458.0;

    /// Create a new ranging engine.
    pub fn new() -> Self {
        Self {
            speed_of_light_m_per_ns: Self::C / 1.0e9,
        }
    }

    /// Compute range from DS-TWR timestamps.
    ///
    /// # Arguments
    /// * `t_round1_ns` - First round-trip time at initiator (ns).
    /// * `t_reply1_ns` - First reply time at responder (ns).
    /// * `t_round2_ns` - Second round-trip time at responder (ns).
    /// * `t_reply2_ns` - Second reply time at initiator (ns).
    ///
    /// # Returns
    /// A [`RangingResult`] with distance, ToF, clock drift estimate, and confidence.
    pub fn compute_range(
        &self,
        t_round1_ns: f64,
        t_reply1_ns: f64,
        t_round2_ns: f64,
        t_reply2_ns: f64,
    ) -> RangingResult {
        let numerator = t_round1_ns * t_round2_ns - t_reply1_ns * t_reply2_ns;
        let denominator = t_round1_ns + t_round2_ns + t_reply1_ns + t_reply2_ns;

        let tof_ns = if denominator.abs() < 1e-15 {
            0.0
        } else {
            numerator / denominator
        };

        let distance_m = tof_ns * self.speed_of_light_m_per_ns;

        // Clock drift estimate:  drift ≈ (t_round1 - t_round2) / (t_round1 + t_round2)
        // expressed in ppm.
        let drift_ratio = if (t_round1_ns + t_round2_ns).abs() < 1e-15 {
            0.0
        } else {
            (t_round1_ns - t_round2_ns) / (t_round1_ns + t_round2_ns)
        };
        let clock_drift_ppm = drift_ratio * 1.0e6;

        // Confidence heuristic: penalise negative ToF and asymmetric exchanges.
        let base_confidence = if tof_ns < 0.0 { 0.0 } else { 1.0 };
        let symmetry = if denominator.abs() < 1e-15 {
            1.0
        } else {
            let ratio = (t_round1_ns * t_reply2_ns) / (t_round2_ns * t_reply1_ns).max(1e-15);
            1.0 / (1.0 + (ratio - 1.0).abs())
        };
        let confidence = (base_confidence * symmetry).clamp(0.0, 1.0);

        RangingResult {
            distance_m,
            tof_ns,
            clock_drift_ppm,
            confidence,
        }
    }
}

impl Default for TwoWayRangingEngine {
    fn default() -> Self {
        Self::new()
    }
}

// ──────────────────────────── First-Path Detection ────────────────────────────

/// Method used for first-path detection in the channel impulse response.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionMethod {
    /// Simple threshold-based leading-edge detection.
    Threshold,
    /// Matched-filter detection using the known pulse template.
    MatchedFilter,
    /// Teager-Kaiser energy operator for sharp onset detection.
    TeagerKaiser,
}

/// Detects the first arriving path in a multipath channel impulse response (CIR).
#[derive(Debug, Clone)]
pub struct FirstPathDetector {
    /// Detection algorithm to use.
    pub method: DetectionMethod,
    /// Detection threshold (interpretation depends on method).
    pub threshold: f64,
}

impl FirstPathDetector {
    /// Create a new first-path detector.
    ///
    /// # Arguments
    /// * `method` - The detection algorithm.
    /// * `threshold` - Relative threshold in \[0, 1\] for peak-normalised CIR.
    pub fn new(method: DetectionMethod, threshold: f64) -> Self {
        Self { method, threshold }
    }

    /// Detect the sample index of the first arriving path in `cir`.
    ///
    /// Returns `None` if the CIR is empty or no path exceeds the threshold.
    pub fn detect_first_path(&self, cir: &[(f64, f64)]) -> Option<usize> {
        if cir.is_empty() {
            return None;
        }

        match self.method {
            DetectionMethod::Threshold => self.detect_threshold(cir),
            DetectionMethod::MatchedFilter => self.detect_matched_filter(cir),
            DetectionMethod::TeagerKaiser => self.detect_teager_kaiser(cir),
        }
    }

    /// Simple threshold detection: find first sample whose magnitude exceeds
    /// `threshold * peak_magnitude`.
    fn detect_threshold(&self, cir: &[(f64, f64)]) -> Option<usize> {
        let magnitudes: Vec<f64> = cir.iter().map(|(re, im)| (re * re + im * im).sqrt()).collect();
        let peak = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        if peak < 1e-30 {
            return None;
        }
        let abs_thresh = self.threshold * peak;
        magnitudes.iter().position(|&m| m >= abs_thresh)
    }

    /// Matched-filter detection: correlate with Gaussian monocycle template,
    /// then apply threshold on correlation magnitude.
    fn detect_matched_filter(&self, cir: &[(f64, f64)]) -> Option<usize> {
        // Use a short matched filter template (Gaussian monocycle).
        let template_len = 16.min(cir.len());
        let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
        let template = gen.generate_pulse(template_len, 10.0e9);

        // Sliding correlation.
        let corr_len = if cir.len() >= template.len() {
            cir.len() - template.len() + 1
        } else {
            return None;
        };

        let mut corr_mag: Vec<f64> = Vec::with_capacity(corr_len);
        for i in 0..corr_len {
            let mut acc_re = 0.0;
            let mut acc_im = 0.0;
            for (j, (tre, tim)) in template.iter().enumerate() {
                let (cre, cim) = cir[i + j];
                // Complex conjugate multiply: cir * conj(template)
                acc_re += cre * tre + cim * tim;
                acc_im += cim * tre - cre * tim;
            }
            corr_mag.push((acc_re * acc_re + acc_im * acc_im).sqrt());
        }

        let peak = corr_mag.iter().cloned().fold(0.0_f64, f64::max);
        if peak < 1e-30 {
            return None;
        }
        let abs_thresh = self.threshold * peak;
        corr_mag.iter().position(|&m| m >= abs_thresh)
    }

    /// Teager-Kaiser energy operator: Ψ[x(n)] = |x(n)|² - Re{x(n-1) * conj(x(n+1))}
    /// Detects first sample where TK energy exceeds threshold * peak TK energy.
    fn detect_teager_kaiser(&self, cir: &[(f64, f64)]) -> Option<usize> {
        if cir.len() < 3 {
            // Fall back to threshold for very short CIRs.
            return self.detect_threshold(cir);
        }

        let mut tk_energy: Vec<f64> = vec![0.0; cir.len()];
        for n in 1..cir.len() - 1 {
            let (xr, xi) = cir[n];
            let (pr, pi) = cir[n - 1]; // x(n-1)
            let (nr, ni) = cir[n + 1]; // x(n+1)
            let mag_sq = xr * xr + xi * xi;
            // Re{x(n-1) * conj(x(n+1))} = pr*nr + pi*ni
            let cross = pr * nr + pi * ni;
            tk_energy[n] = (mag_sq - cross).abs();
        }

        let peak = tk_energy.iter().cloned().fold(0.0_f64, f64::max);
        if peak < 1e-30 {
            return None;
        }
        let abs_thresh = self.threshold * peak;
        tk_energy.iter().position(|&e| e >= abs_thresh)
    }
}

// ──────────────────────────── Trilateration & GDOP ────────────────────────────

/// Estimate 3-D position from anchor positions and measured ranges using
/// linearised least-squares trilateration.
///
/// Requires at least 4 non-coplanar anchors for a unique 3-D fix.
/// Returns `None` if the system is under-determined or singular.
pub fn trilaterate(
    anchors: &[(f64, f64, f64)],
    ranges: &[f64],
) -> Option<(f64, f64, f64)> {
    let n = anchors.len();
    if n < 4 || n != ranges.len() {
        return None;
    }

    // Linearise by subtracting equation for anchor 0 from all others:
    // 2(x_i - x_0)x + 2(y_i - y_0)y + 2(z_i - z_0)z
    //   = (r_0² - r_i²) + (x_i² - x_0²) + (y_i² - y_0²) + (z_i² - z_0²)
    let (x0, y0, z0) = anchors[0];
    let r0_sq = ranges[0] * ranges[0];
    let p0_sq = x0 * x0 + y0 * y0 + z0 * z0;

    let m = n - 1; // number of equations
    // Build A (m×3) and b (m×1)
    let mut a_flat: Vec<f64> = Vec::with_capacity(m * 3);
    let mut b_vec: Vec<f64> = Vec::with_capacity(m);

    for i in 1..n {
        let (xi, yi, zi) = anchors[i];
        let ri_sq = ranges[i] * ranges[i];
        let pi_sq = xi * xi + yi * yi + zi * zi;

        a_flat.push(2.0 * (xi - x0));
        a_flat.push(2.0 * (yi - y0));
        a_flat.push(2.0 * (zi - z0));
        b_vec.push((r0_sq - ri_sq) + (pi_sq - p0_sq));
    }

    // Solve via normal equations: (A^T A) x = A^T b
    // A^T A is 3×3, A^T b is 3×1
    let mut ata = [0.0_f64; 9]; // row-major 3×3
    let mut atb = [0.0_f64; 3];

    for row in 0..m {
        for c in 0..3 {
            let a_rc = a_flat[row * 3 + c];
            atb[c] += a_rc * b_vec[row];
            for c2 in 0..3 {
                ata[c * 3 + c2] += a_rc * a_flat[row * 3 + c2];
            }
        }
    }

    // Solve 3×3 system with Cramer's rule
    solve_3x3(&ata, &atb)
}

/// Compute Geometric Dilution of Precision (GDOP) for a given geometry.
///
/// GDOP = sqrt(trace(H^T H)^{-1}) where H is the geometry matrix of unit vectors
/// from the position to each anchor.
pub fn gdop(anchors: &[(f64, f64, f64)], position: (f64, f64, f64)) -> f64 {
    let n = anchors.len();
    if n < 4 {
        return f64::INFINITY;
    }

    let (px, py, pz) = position;

    // Build H matrix (n × 4): each row = [-dx/r, -dy/r, -dz/r, 1]
    let mut h: Vec<[f64; 4]> = Vec::with_capacity(n);
    for &(ax, ay, az) in anchors {
        let dx = ax - px;
        let dy = ay - py;
        let dz = az - pz;
        let r = (dx * dx + dy * dy + dz * dz).sqrt();
        if r < 1e-12 {
            return f64::INFINITY;
        }
        h.push([-dx / r, -dy / r, -dz / r, 1.0]);
    }

    // Compute H^T H (4×4)
    let mut hth = [[0.0_f64; 4]; 4];
    for row in &h {
        for i in 0..4 {
            for j in 0..4 {
                hth[i][j] += row[i] * row[j];
            }
        }
    }

    // Invert 4×4 via cofactor expansion
    if let Some(inv) = invert_4x4(&hth) {
        let trace = inv[0][0] + inv[1][1] + inv[2][2] + inv[3][3];
        if trace >= 0.0 {
            trace.sqrt()
        } else {
            f64::INFINITY
        }
    } else {
        f64::INFINITY
    }
}

// ──────────────────────── Linear algebra helpers ──────────────────────────

/// Solve a 3×3 linear system Ax = b via Cramer's rule.
fn solve_3x3(a: &[f64; 9], b: &[f64; 3]) -> Option<(f64, f64, f64)> {
    let det = a[0] * (a[4] * a[8] - a[5] * a[7])
        - a[1] * (a[3] * a[8] - a[5] * a[6])
        + a[2] * (a[3] * a[7] - a[4] * a[6]);

    if det.abs() < 1e-30 {
        return None;
    }

    let inv_det = 1.0 / det;

    let x = (b[0] * (a[4] * a[8] - a[5] * a[7])
        - a[1] * (b[1] * a[8] - a[5] * b[2])
        + a[2] * (b[1] * a[7] - a[4] * b[2]))
        * inv_det;

    let y = (a[0] * (b[1] * a[8] - a[5] * b[2])
        - b[0] * (a[3] * a[8] - a[5] * a[6])
        + a[2] * (a[3] * b[2] - b[1] * a[6]))
        * inv_det;

    let z = (a[0] * (a[4] * b[2] - b[1] * a[7])
        - a[1] * (a[3] * b[2] - b[1] * a[6])
        + b[0] * (a[3] * a[7] - a[4] * a[6]))
        * inv_det;

    Some((x, y, z))
}

/// Invert a 4×4 matrix via cofactor expansion. Returns `None` if singular.
fn invert_4x4(m: &[[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    // Flatten for easier indexing
    let s = |r: usize, c: usize| m[r][c];

    // Compute 2×2 sub-determinants (Laplace expansion on first two rows)
    let s0 = s(0, 0) * s(1, 1) - s(1, 0) * s(0, 1);
    let s1 = s(0, 0) * s(1, 2) - s(1, 0) * s(0, 2);
    let s2 = s(0, 0) * s(1, 3) - s(1, 0) * s(0, 3);
    let s3 = s(0, 1) * s(1, 2) - s(1, 1) * s(0, 2);
    let s4 = s(0, 1) * s(1, 3) - s(1, 1) * s(0, 3);
    let s5 = s(0, 2) * s(1, 3) - s(1, 2) * s(0, 3);

    let c5 = s(2, 2) * s(3, 3) - s(3, 2) * s(2, 3);
    let c4 = s(2, 1) * s(3, 3) - s(3, 1) * s(2, 3);
    let c3 = s(2, 1) * s(3, 2) - s(3, 1) * s(2, 2);
    let c2 = s(2, 0) * s(3, 3) - s(3, 0) * s(2, 3);
    let c1 = s(2, 0) * s(3, 2) - s(3, 0) * s(2, 2);
    let c0 = s(2, 0) * s(3, 1) - s(3, 0) * s(2, 1);

    let det = s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    if det.abs() < 1e-30 {
        return None;
    }
    let inv_det = 1.0 / det;

    let mut inv = [[0.0_f64; 4]; 4];

    inv[0][0] = (s(1, 1) * c5 - s(1, 2) * c4 + s(1, 3) * c3) * inv_det;
    inv[0][1] = (-s(0, 1) * c5 + s(0, 2) * c4 - s(0, 3) * c3) * inv_det;
    inv[0][2] = (s(3, 1) * s5 - s(3, 2) * s4 + s(3, 3) * s3) * inv_det;
    inv[0][3] = (-s(2, 1) * s5 + s(2, 2) * s4 - s(2, 3) * s3) * inv_det;

    inv[1][0] = (-s(1, 0) * c5 + s(1, 2) * c2 - s(1, 3) * c1) * inv_det;
    inv[1][1] = (s(0, 0) * c5 - s(0, 2) * c2 + s(0, 3) * c1) * inv_det;
    inv[1][2] = (-s(3, 0) * s5 + s(3, 2) * s2 - s(3, 3) * s1) * inv_det;
    inv[1][3] = (s(2, 0) * s5 - s(2, 2) * s2 + s(2, 3) * s1) * inv_det;

    inv[2][0] = (s(1, 0) * c4 - s(1, 1) * c2 + s(1, 3) * c0) * inv_det;
    inv[2][1] = (-s(0, 0) * c4 + s(0, 1) * c2 - s(0, 3) * c0) * inv_det;
    inv[2][2] = (s(3, 0) * s4 - s(3, 1) * s2 + s(3, 3) * s0) * inv_det;
    inv[2][3] = (-s(2, 0) * s4 + s(2, 1) * s2 - s(2, 3) * s0) * inv_det;

    inv[3][0] = (-s(1, 0) * c3 + s(1, 1) * c1 - s(1, 2) * c0) * inv_det;
    inv[3][1] = (s(0, 0) * c3 - s(0, 1) * c1 + s(0, 2) * c0) * inv_det;
    inv[3][2] = (-s(3, 0) * s3 + s(3, 1) * s1 - s(3, 2) * s0) * inv_det;
    inv[3][3] = (s(2, 0) * s3 - s(2, 1) * s1 + s(2, 2) * s0) * inv_det;

    Some(inv)
}

// ──────────────────────────── Tests ────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // ── Pulse generator tests ──

    #[test]
    fn test_gaussian_monocycle_pulse_length() {
        let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
        let pulse = gen.generate_pulse(256, 10.0e9);
        assert_eq!(pulse.len(), 256);
    }

    #[test]
    fn test_gaussian_monocycle_zero_crossing() {
        // Gaussian monocycle should cross zero at the centre.
        let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
        let pulse = gen.generate_pulse(256, 10.0e9);
        let mid = pulse.len() / 2;
        // Near centre the value should be near zero (exact zero at t=0).
        assert!(
            pulse[mid].0.abs() < 0.05,
            "Centre value {} should be near zero",
            pulse[mid].0
        );
    }

    #[test]
    fn test_gaussian_monocycle_normalised() {
        let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
        let pulse = gen.generate_pulse(256, 10.0e9);
        let peak = pulse
            .iter()
            .map(|(re, im)| (re * re + im * im).sqrt())
            .fold(0.0_f64, f64::max);
        assert!((peak - 1.0).abs() < 1e-10, "Peak should be normalised to 1.0");
    }

    #[test]
    fn test_rayleigh_pulse_peak_at_centre() {
        // Rayleigh (second derivative of Gaussian) peaks at t=0.
        let gen = UwbPulseGenerator::new(PulseShape::Rayleigh, 0.5);
        let pulse = gen.generate_pulse(256, 10.0e9);
        let mid = pulse.len() / 2;
        let peak_idx = pulse
            .iter()
            .enumerate()
            .max_by(|a, b| {
                let ma = (a.1 .0 * a.1 .0 + a.1 .1 * a.1 .1).sqrt();
                let mb = (b.1 .0 * b.1 .0 + b.1 .1 * b.1 .1).sqrt();
                ma.partial_cmp(&mb).unwrap()
            })
            .unwrap()
            .0;
        // Peak should be within a few samples of the centre.
        assert!(
            (peak_idx as i64 - mid as i64).unsigned_abs() <= 2,
            "Peak index {} should be near centre {}",
            peak_idx,
            mid
        );
    }

    #[test]
    fn test_raised_cosine_pulse() {
        let gen = UwbPulseGenerator::new(PulseShape::RaisedCosine, 0.5);
        let pulse = gen.generate_pulse(128, 10.0e9);
        assert_eq!(pulse.len(), 128);
        // Imaginary part should be zero for all samples.
        for (i, (_re, im)) in pulse.iter().enumerate() {
            assert!(
                im.abs() < 1e-15,
                "Imaginary part at sample {} should be zero, got {}",
                i,
                im
            );
        }
    }

    #[test]
    fn test_empty_pulse() {
        let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
        let pulse = gen.generate_pulse(0, 10.0e9);
        assert!(pulse.is_empty());
    }

    // ── DS-TWR tests ──

    #[test]
    fn test_ds_twr_symmetric() {
        let engine = TwoWayRangingEngine::new();
        // Symmetric exchange: round = 100 ns, reply = 10 ns each side.
        // tof = (100*100 - 10*10) / (100+100+10+10) = (10000 - 100) / 220 = 9900/220 ≈ 45 ns
        let result = engine.compute_range(100.0, 10.0, 100.0, 10.0);
        let expected_tof = 9900.0 / 220.0;
        assert!(
            (result.tof_ns - expected_tof).abs() < TOLERANCE,
            "ToF {} should be {}",
            result.tof_ns,
            expected_tof
        );
        let expected_dist = expected_tof * (299_792_458.0 / 1.0e9);
        assert!(
            (result.distance_m - expected_dist).abs() < 0.001,
            "Distance {} should be {}",
            result.distance_m,
            expected_dist
        );
    }

    #[test]
    fn test_ds_twr_zero_drift_symmetric() {
        let engine = TwoWayRangingEngine::new();
        let result = engine.compute_range(100.0, 10.0, 100.0, 10.0);
        // Symmetric exchange => zero clock drift.
        assert!(
            result.clock_drift_ppm.abs() < TOLERANCE,
            "Clock drift should be zero for symmetric exchange, got {} ppm",
            result.clock_drift_ppm
        );
    }

    #[test]
    fn test_ds_twr_with_drift() {
        let engine = TwoWayRangingEngine::new();
        // Asymmetric: responder clock runs 20 ppm fast => round2 slightly shorter.
        let result = engine.compute_range(100.0, 10.0, 99.998, 10.0);
        // drift should be positive (initiator sees longer round trip).
        assert!(
            result.clock_drift_ppm > 0.0,
            "Clock drift should be positive when round1 > round2"
        );
    }

    #[test]
    fn test_ds_twr_confidence() {
        let engine = TwoWayRangingEngine::new();
        let result = engine.compute_range(100.0, 10.0, 100.0, 10.0);
        assert!(
            result.confidence > 0.5 && result.confidence <= 1.0,
            "Confidence {} should be in (0.5, 1.0] for good exchange",
            result.confidence
        );
    }

    // ── First-path detection tests ──

    #[test]
    fn test_threshold_detection_basic() {
        let detector = FirstPathDetector::new(DetectionMethod::Threshold, 0.5);
        let cir = vec![(0.1, 0.0), (0.2, 0.0), (0.9, 0.0), (0.3, 0.0)];
        assert_eq!(detector.detect_first_path(&cir), Some(2));
    }

    #[test]
    fn test_threshold_detection_early_arrival() {
        let detector = FirstPathDetector::new(DetectionMethod::Threshold, 0.3);
        // First path at index 1 (0.4 > 0.3 * 1.0 = 0.3).
        let cir = vec![(0.0, 0.0), (0.4, 0.0), (1.0, 0.0), (0.5, 0.0)];
        assert_eq!(detector.detect_first_path(&cir), Some(1));
    }

    #[test]
    fn test_threshold_detection_empty() {
        let detector = FirstPathDetector::new(DetectionMethod::Threshold, 0.5);
        assert_eq!(detector.detect_first_path(&[]), None);
    }

    #[test]
    fn test_teager_kaiser_detection() {
        let detector = FirstPathDetector::new(DetectionMethod::TeagerKaiser, 0.3);
        // Impulse at index 5 with low noise floor.
        let mut cir = vec![(0.01, 0.0); 20];
        cir[5] = (1.0, 0.0);
        cir[6] = (0.5, 0.0);
        let idx = detector.detect_first_path(&cir);
        // TK energy should spike at or near the impulse.
        assert!(idx.is_some());
        assert!(idx.unwrap() <= 6, "First path should be at or before index 6");
    }

    #[test]
    fn test_matched_filter_detection() {
        let detector = FirstPathDetector::new(DetectionMethod::MatchedFilter, 0.3);
        // Create a CIR with a pulse-like feature at index 10.
        let gen = UwbPulseGenerator::new(PulseShape::GaussianMonocycle, 0.5);
        let template = gen.generate_pulse(16, 10.0e9);
        let mut cir = vec![(0.0, 0.0); 64];
        for (j, &(re, im)) in template.iter().enumerate() {
            cir[10 + j] = (re, im);
        }
        let idx = detector.detect_first_path(&cir);
        assert!(idx.is_some());
        // Should detect near the start of the pulse.
        let detected = idx.unwrap();
        assert!(
            detected <= 12,
            "Matched filter should detect near pulse onset, got {}",
            detected
        );
    }

    // ── Trilateration tests ──

    #[test]
    fn test_trilaterate_known_position() {
        // Place target at origin. Anchors at known positions.
        let anchors = vec![
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
            (-10.0, 0.0, 0.0),
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|&(x, y, z): &(f64, f64, f64)| (x * x + y * y + z * z).sqrt())
            .collect();
        let pos = trilaterate(&anchors, &ranges);
        assert!(pos.is_some(), "Trilateration should succeed");
        let (x, y, z) = pos.unwrap();
        assert!((x).abs() < 0.01, "x = {} should be ~0", x);
        assert!((y).abs() < 0.01, "y = {} should be ~0", y);
        assert!((z).abs() < 0.01, "z = {} should be ~0", z);
    }

    #[test]
    fn test_trilaterate_offset_position() {
        let target = (3.0, 4.0, 5.0);
        let anchors = vec![
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
            (-10.0, -10.0, -10.0),
            (5.0, 5.0, 5.0),
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|&(ax, ay, az): &(f64, f64, f64)| {
                let dx = ax - target.0;
                let dy = ay - target.1;
                let dz = az - target.2;
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .collect();
        let pos = trilaterate(&anchors, &ranges);
        assert!(pos.is_some());
        let (x, y, z) = pos.unwrap();
        assert!((x - target.0).abs() < 0.01, "x = {} should be ~{}", x, target.0);
        assert!((y - target.1).abs() < 0.01, "y = {} should be ~{}", y, target.1);
        assert!((z - target.2).abs() < 0.01, "z = {} should be ~{}", z, target.2);
    }

    #[test]
    fn test_trilaterate_insufficient_anchors() {
        let anchors = vec![(10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (0.0, 0.0, 10.0)];
        let ranges = vec![10.0, 10.0, 10.0];
        assert!(trilaterate(&anchors, &ranges).is_none());
    }

    // ── GDOP tests ──

    #[test]
    fn test_gdop_good_geometry() {
        // Well-spread anchors around origin should give low GDOP.
        let anchors = vec![
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
            (-10.0, 0.0, 0.0),
            (0.0, -10.0, 0.0),
        ];
        let g = gdop(&anchors, (0.0, 0.0, 0.0));
        assert!(g.is_finite(), "GDOP should be finite");
        assert!(g < 10.0, "GDOP {} should be reasonable for good geometry", g);
    }

    #[test]
    fn test_gdop_insufficient_anchors() {
        let anchors = vec![(10.0, 0.0, 0.0), (0.0, 10.0, 0.0)];
        let g = gdop(&anchors, (0.0, 0.0, 0.0));
        assert!(g.is_infinite(), "GDOP should be infinite with < 4 anchors");
    }
}
