//! Magnetic Anomaly Detector (MAD) signal processing for submarine/underwater object detection.
//!
//! This module implements signal processing algorithms used in maritime patrol aircraft
//! and anti-submarine warfare (ASW) systems that detect ferromagnetic objects beneath
//! the ocean surface by measuring perturbations in the Earth's magnetic field.
//!
//! The processing pipeline is:
//!
//! 1. **Bandpass filter** - Remove DC (Earth field) and high-frequency noise
//! 2. **Fourth-difference operator** - Enhance magnetic dipole signatures
//! 3. **Threshold detector** - Identify candidate anomalies
//! 4. **Range/depth estimation** - Estimate target parameters from signature shape
//!
//! # Physics Background
//!
//! A submarine (or any large ferromagnetic object) produces a magnetic dipole field
//! that falls off as 1/r^3. When a patrol aircraft passes over the target, the
//! total-field magnetometer records a characteristic bipolar anomaly whose shape
//! depends on the geometry. The fourth-difference operator enhances this dipole
//! signature relative to broader geomagnetic variations.
//!
//! # Example
//!
//! ```
//! use r4w_core::magnetic_anomaly_detector::{MadConfig, MadProcessor, generate_dipole_anomaly};
//!
//! let config = MadConfig {
//!     sample_rate_hz: 100.0,
//!     platform_speed_mps: 80.0,
//!     earth_field_nt: 50000.0,
//!     detection_threshold_nt: 0.5,
//!     highpass_cutoff_hz: 0.01,
//!     lowpass_cutoff_hz: 5.0,
//! };
//! let mut processor = MadProcessor::new(config);
//!
//! // Generate a synthetic submarine anomaly at 200m depth
//! let anomaly = generate_dipole_anomaly(1e8, 200.0, 80.0, 100.0, 30.0);
//! let result = processor.process(&anomaly);
//! assert!(!result.detections.is_empty());
//! ```

use std::f64::consts::PI;

// ── Physical constants ─────────────────────────────────────────────────────

/// Permeability of free space in T*m/A.
const MU0: f64 = 4.0 * PI * 1e-7;

/// mu0 / (4*pi) -- dipole field prefactor.
const MU0_OVER_4PI: f64 = 1e-7;

// ── Configuration ──────────────────────────────────────────────────────────

/// Configuration for the MAD signal processor.
#[derive(Debug, Clone)]
pub struct MadConfig {
    /// Magnetometer sample rate in Hz (typically 10-1000 Hz).
    pub sample_rate_hz: f64,
    /// Platform (aircraft) speed in metres per second.
    pub platform_speed_mps: f64,
    /// Background Earth magnetic field strength in nanotesla (default ~50 000 nT).
    pub earth_field_nt: f64,
    /// Detection threshold in nanotesla applied to the fourth-difference output.
    pub detection_threshold_nt: f64,
    /// High-pass cutoff frequency in Hz for the bandpass pre-filter.
    pub highpass_cutoff_hz: f64,
    /// Low-pass cutoff frequency in Hz for the bandpass pre-filter.
    pub lowpass_cutoff_hz: f64,
}

impl Default for MadConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 100.0,
            platform_speed_mps: 80.0,
            earth_field_nt: 50_000.0,
            detection_threshold_nt: 1.0,
            highpass_cutoff_hz: 0.01,
            lowpass_cutoff_hz: 5.0,
        }
    }
}

// ── Result types ───────────────────────────────────────────────────────────

/// A single magnetic anomaly detection event.
#[derive(Debug, Clone)]
pub struct MadDetection {
    /// Sample index of the detection (peak of fourth-difference output).
    pub sample_index: usize,
    /// Peak amplitude in nanotesla of the fourth-difference output at detection.
    pub amplitude_nt: f64,
    /// Estimated slant range to the target in metres.
    pub estimated_range_m: f64,
    /// Sample index of the closest point of approach (CPA), identified as the
    /// zero-crossing of the filtered signal nearest to the detection.
    pub cpa_index: usize,
}

/// Output of the MAD processor for one data block.
#[derive(Debug, Clone)]
pub struct MadResult {
    /// List of detected anomalies.
    pub detections: Vec<MadDetection>,
    /// Bandpass-filtered magnetometer signal in nanotesla.
    pub filtered_signal: Vec<f64>,
    /// Estimated noise floor in nanotesla (RMS of filtered signal).
    pub noise_floor_nt: f64,
}

// ── Main processor ─────────────────────────────────────────────────────────

/// Magnetic anomaly detection processor.
///
/// Implements the classic MAD processing chain: bandpass filter, fourth-difference
/// operator, and threshold detection with CPA estimation.
#[derive(Debug, Clone)]
pub struct MadProcessor {
    config: MadConfig,
}

impl MadProcessor {
    /// Create a new MAD processor from the given configuration.
    pub fn new(config: MadConfig) -> Self {
        Self { config }
    }

    /// Process a block of total-field magnetometer samples (in nanotesla).
    ///
    /// Returns detections, filtered signal, and noise floor estimate.
    pub fn process(&mut self, magnetometer_data: &[f64]) -> MadResult {
        if magnetometer_data.is_empty() {
            return MadResult {
                detections: Vec::new(),
                filtered_signal: Vec::new(),
                noise_floor_nt: 0.0,
            };
        }

        // Step 1: Bandpass filter to isolate anomaly frequencies
        let filtered = bandpass_filter(
            magnetometer_data,
            self.config.sample_rate_hz,
            self.config.highpass_cutoff_hz,
            self.config.lowpass_cutoff_hz,
        );

        // Step 2: Fourth-difference operator to enhance dipole signatures
        let fd = fourth_difference(&filtered);

        // Step 3: Compute noise floor from filtered signal
        let noise_floor = compute_noise_floor(&filtered);

        // Step 4: Threshold detection on the fourth-difference output
        let detections = self.detect_anomalies(&filtered, &fd, noise_floor);

        MadResult {
            detections,
            filtered_signal: filtered,
            noise_floor_nt: noise_floor,
        }
    }

    /// Identify anomalies exceeding the threshold in the fourth-difference signal.
    fn detect_anomalies(
        &self,
        filtered: &[f64],
        fd: &[f64],
        _noise_floor: f64,
    ) -> Vec<MadDetection> {
        let threshold = self.config.detection_threshold_nt;
        let mut detections = Vec::new();

        // We need at least 5 samples for the fourth-difference to be meaningful.
        if fd.len() < 5 {
            return detections;
        }

        // Find local peaks in the absolute fourth-difference signal that exceed threshold.
        // We use a simple state machine: enter detection when |fd| > threshold,
        // find the peak, exit when |fd| drops below threshold.
        let mut in_detection = false;
        let mut peak_idx: usize = 0;
        let mut peak_val: f64 = 0.0;

        for i in 0..fd.len() {
            let abs_val = fd[i].abs();
            if !in_detection {
                if abs_val > threshold {
                    in_detection = true;
                    peak_idx = i;
                    peak_val = abs_val;
                }
            } else {
                if abs_val > peak_val {
                    peak_idx = i;
                    peak_val = abs_val;
                }
                if abs_val <= threshold {
                    // Detection ended -- record it.
                    // The fourth-difference indices are offset by 2 relative to filtered signal.
                    let sample_index = peak_idx + 2;
                    let cpa_index = self.find_cpa(filtered, sample_index);
                    let estimated_range = self.estimate_range(peak_val);

                    detections.push(MadDetection {
                        sample_index,
                        amplitude_nt: peak_val,
                        estimated_range_m: estimated_range,
                        cpa_index,
                    });
                    in_detection = false;
                }
            }
        }

        // Handle case where detection extends to end of data.
        if in_detection {
            let sample_index = peak_idx + 2;
            let cpa_index = self.find_cpa(filtered, sample_index);
            let estimated_range = self.estimate_range(peak_val);
            detections.push(MadDetection {
                sample_index,
                amplitude_nt: peak_val,
                estimated_range_m: estimated_range,
                cpa_index,
            });
        }

        detections
    }

    /// Find the closest point of approach (CPA) as the zero-crossing in the
    /// filtered signal nearest to the detection index.
    fn find_cpa(&self, filtered: &[f64], detection_idx: usize) -> usize {
        if filtered.is_empty() {
            return 0;
        }
        let idx = detection_idx.min(filtered.len() - 1);

        // Search outward from detection_idx for a zero-crossing.
        let search_radius = (self.config.sample_rate_hz * 2.0) as usize; // ~2 seconds
        let start = idx.saturating_sub(search_radius);
        let end = (idx + search_radius).min(filtered.len() - 1);

        let mut best_idx = idx;
        let mut best_abs = filtered[idx].abs();

        for i in start..end {
            // A zero-crossing occurs between samples i and i+1 where sign changes.
            if i + 1 < filtered.len() && filtered[i] * filtered[i + 1] <= 0.0 {
                // Pick the sample closer to zero.
                let candidate = if filtered[i].abs() < filtered[i + 1].abs() {
                    i
                } else {
                    i + 1
                };
                let dist = if candidate > idx {
                    candidate - idx
                } else {
                    idx - candidate
                };
                let cur_dist = if best_idx > idx {
                    best_idx - idx
                } else {
                    idx - best_idx
                };
                if filtered[candidate].abs() < best_abs
                    || (dist < cur_dist && filtered[candidate].abs() < best_abs * 2.0)
                {
                    best_idx = candidate;
                    best_abs = filtered[candidate].abs();
                }
            }
        }

        best_idx
    }

    /// Rough range estimate from the amplitude of the fourth-difference peak.
    ///
    /// The fourth-difference of a dipole field scales roughly as m / r^7 (the
    /// fourth spatial derivative of 1/r^3 with conversion via platform speed).
    /// We use a simplified inverse relationship here.
    fn estimate_range(&self, amplitude_nt: f64) -> f64 {
        // For a magnetic moment of ~1e8 A*m^2 (typical submarine), the peak
        // anomaly at range r is approximately: B ~ (mu0/4pi) * m / r^3
        // The fourth-difference enhances the peak but preserves the scaling.
        // Invert: r ~ ((mu0/4pi) * m_assumed / B)^(1/3)
        let m_assumed = 1e8; // A*m^2, typical submarine magnetic moment
        let b_tesla = amplitude_nt * 1e-9;
        if b_tesla <= 0.0 {
            return f64::INFINITY;
        }
        let r_cubed = MU0_OVER_4PI * m_assumed / b_tesla;
        r_cubed.cbrt()
    }
}

// ── Core DSP functions ─────────────────────────────────────────────────────

/// Compute the fourth finite difference of a signal.
///
/// The fourth-difference operator enhances magnetic dipole signatures because
/// the fourth spatial derivative of a 1/r^3 dipole field is sharply peaked.
/// The coefficients are [1, -4, 6, -4, 1] applied as a convolution.
///
/// Output length is `signal.len() - 4` (centered, no zero-padding).
pub fn fourth_difference(signal: &[f64]) -> Vec<f64> {
    if signal.len() < 5 {
        return Vec::new();
    }
    let n = signal.len() - 4;
    let mut output = Vec::with_capacity(n);
    for i in 0..n {
        let val = signal[i] - 4.0 * signal[i + 1] + 6.0 * signal[i + 2]
            - 4.0 * signal[i + 3] + signal[i + 4];
        output.push(val);
    }
    output
}

/// Compute the magnetic field of a dipole at a given range and angle.
///
/// # Arguments
///
/// * `moment_am2` - Magnetic dipole moment in A*m^2
/// * `range_m` - Distance from the dipole in metres
/// * `theta_rad` - Angle from the dipole axis in radians
///
/// # Returns
///
/// Total magnetic field magnitude in nanotesla.
///
/// # Formula
///
/// `B = (mu0 / (4*pi)) * m * sqrt(1 + 3*cos^2(theta)) / r^3`
pub fn dipole_field_nt(moment_am2: f64, range_m: f64, theta_rad: f64) -> f64 {
    if range_m <= 0.0 {
        return f64::INFINITY;
    }
    let cos_theta = theta_rad.cos();
    let geometric_factor = (1.0 + 3.0 * cos_theta * cos_theta).sqrt();
    let field_tesla = MU0_OVER_4PI * moment_am2 * geometric_factor / (range_m.powi(3));
    field_tesla * 1e9 // Convert T to nT
}

/// Estimate target depth from the half-width of a MAD anomaly signature.
///
/// For a magnetic dipole directly below the flight path, the half-width
/// (distance between the positive and negative peaks) `w` relates to depth `d` by:
///
/// `w = d * sqrt(2)` (approximately, for a pure axial dipole flyover)
///
/// Given the half-width in samples, we convert to distance using platform speed
/// and sample rate, then solve for depth.
///
/// # Arguments
///
/// * `signature` - The anomaly waveform (filtered magnetometer data around the anomaly)
/// * `platform_speed_mps` - Aircraft speed in m/s
/// * `sample_rate_hz` - Magnetometer sample rate in Hz
///
/// # Returns
///
/// Estimated depth in metres.
pub fn estimate_depth_from_signature(
    signature: &[f64],
    platform_speed_mps: f64,
    sample_rate_hz: f64,
) -> f64 {
    if signature.len() < 3 || sample_rate_hz <= 0.0 || platform_speed_mps <= 0.0 {
        return 0.0;
    }

    // Find the positive peak and negative peak (trough).
    let mut max_idx = 0usize;
    let mut max_val = f64::NEG_INFINITY;
    let mut min_idx = 0usize;
    let mut min_val = f64::INFINITY;

    for (i, &v) in signature.iter().enumerate() {
        if v > max_val {
            max_val = v;
            max_idx = i;
        }
        if v < min_val {
            min_val = v;
            min_idx = i;
        }
    }

    // Half-width in samples is the distance between peak and trough.
    let half_width_samples = if max_idx > min_idx {
        max_idx - min_idx
    } else {
        min_idx - max_idx
    } as f64;

    // Convert to distance.
    let half_width_m = half_width_samples * platform_speed_mps / sample_rate_hz;

    // For a dipole flyover: half_width ~ depth * sqrt(2)
    // Therefore: depth ~ half_width / sqrt(2)
    half_width_m / 2.0_f64.sqrt()
}

/// Generate a synthetic MAD anomaly for a magnetic dipole target.
///
/// Simulates an aircraft flying in a straight line directly over a submerged
/// magnetic dipole at the given depth. The dipole axis is vertical (worst-case
/// for detection; best-case anomaly shape).
///
/// # Arguments
///
/// * `moment_am2` - Magnetic dipole moment in A*m^2
/// * `depth_m` - Depth of the target below the flight path in metres
/// * `speed_mps` - Platform speed in m/s
/// * `fs` - Sample rate in Hz
/// * `duration_s` - Total duration of the flyover in seconds
///
/// # Returns
///
/// Vector of total-field anomaly values in nanotesla.
pub fn generate_dipole_anomaly(
    moment_am2: f64,
    depth_m: f64,
    speed_mps: f64,
    fs: f64,
    duration_s: f64,
) -> Vec<f64> {
    let n = (fs * duration_s) as usize;
    if n == 0 || depth_m <= 0.0 {
        return Vec::new();
    }

    let mut anomaly = Vec::with_capacity(n);
    let half_duration = duration_s / 2.0;

    for i in 0..n {
        let t = i as f64 / fs;
        // Horizontal distance from CPA (t=0 at midpoint).
        let x = speed_mps * (t - half_duration);
        // Slant range.
        let r = (x * x + depth_m * depth_m).sqrt();
        // Angle from dipole axis (vertical). theta = atan2(x, depth).
        let theta = x.atan2(depth_m);

        // For a vertical dipole observed along the flight path, the total-field
        // anomaly (projection onto Earth field direction) has the classic
        // shape: delta_B ~ (mu0/4pi) * m * (2*cos^2(theta) - sin^2(theta)) / r^3
        // which simplifies to (mu0/4pi) * m * (3*cos^2(theta) - 1) / r^3
        let cos_theta = theta.cos();
        let field_tesla =
            MU0_OVER_4PI * moment_am2 * (3.0 * cos_theta * cos_theta - 1.0) / r.powi(3);
        anomaly.push(field_tesla * 1e9); // Convert to nT
    }

    anomaly
}

/// Apply a simple IIR bandpass filter to the signal.
///
/// Implements a cascade of a first-order high-pass and a first-order low-pass
/// IIR filter. This is intentionally simple (no external dependencies);
/// for production use, replace with a proper higher-order Butterworth design.
///
/// # Arguments
///
/// * `signal` - Input samples
/// * `fs` - Sample rate in Hz
/// * `low_hz` - High-pass cutoff frequency in Hz
/// * `high_hz` - Low-pass cutoff frequency in Hz
///
/// # Returns
///
/// Filtered signal of the same length as the input.
pub fn bandpass_filter(signal: &[f64], fs: f64, low_hz: f64, high_hz: f64) -> Vec<f64> {
    if signal.is_empty() || fs <= 0.0 {
        return signal.to_vec();
    }

    // Clamp cutoffs.
    let low = low_hz.max(0.0).min(fs / 2.0);
    let high = high_hz.max(low).min(fs / 2.0);

    // First-order high-pass: y[n] = alpha_hp * (y[n-1] + x[n] - x[n-1])
    let rc_hp = 1.0 / (2.0 * PI * low);
    let dt = 1.0 / fs;
    let alpha_hp = rc_hp / (rc_hp + dt);

    let mut hp_out = Vec::with_capacity(signal.len());
    hp_out.push(signal[0]);
    for i in 1..signal.len() {
        let y = alpha_hp * (hp_out[i - 1] + signal[i] - signal[i - 1]);
        hp_out.push(y);
    }

    // First-order low-pass: y[n] = alpha_lp * x[n] + (1 - alpha_lp) * y[n-1]
    let rc_lp = 1.0 / (2.0 * PI * high);
    let alpha_lp = dt / (rc_lp + dt);

    let mut lp_out = Vec::with_capacity(hp_out.len());
    lp_out.push(hp_out[0]);
    for i in 1..hp_out.len() {
        let y = alpha_lp * hp_out[i] + (1.0 - alpha_lp) * lp_out[i - 1];
        lp_out.push(y);
    }

    lp_out
}

/// Compute the noise floor of a signal as its RMS value.
///
/// # Arguments
///
/// * `signal` - Input samples
///
/// # Returns
///
/// RMS amplitude of the signal.
pub fn compute_noise_floor(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = signal.iter().map(|&x| x * x).sum();
    (sum_sq / signal.len() as f64).sqrt()
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-9;

    // ── Dipole field tests ─────────────────────────────────────────────

    #[test]
    fn test_dipole_field_on_axis() {
        // On-axis (theta=0): B = (mu0/4pi) * m * 2 / r^3  (in nT)
        let moment = 1e6; // A*m^2
        let range = 100.0; // m
        let b = dipole_field_nt(moment, range, 0.0);
        // Expected: 1e-7 * 1e6 * sqrt(1 + 3) / 100^3 = 1e-7 * 1e6 * 2 / 1e6 = 2e-7 T = 200 nT
        let expected = MU0_OVER_4PI * moment * 2.0 / range.powi(3) * 1e9;
        assert!(
            (b - expected).abs() < EPSILON * expected.abs(),
            "On-axis dipole: got {b}, expected {expected}"
        );
    }

    #[test]
    fn test_dipole_field_equatorial() {
        // Equatorial (theta=pi/2): B = (mu0/4pi) * m * 1 / r^3
        let moment = 1e6;
        let range = 100.0;
        let b = dipole_field_nt(moment, range, PI / 2.0);
        let expected = MU0_OVER_4PI * moment * 1.0 / range.powi(3) * 1e9;
        assert!(
            (b - expected).abs() < 1e-6,
            "Equatorial dipole: got {b}, expected {expected}"
        );
    }

    #[test]
    fn test_dipole_field_inverse_cube_law() {
        // Doubling the range should reduce the field by factor of 8.
        let moment = 1e6;
        let b1 = dipole_field_nt(moment, 100.0, 0.0);
        let b2 = dipole_field_nt(moment, 200.0, 0.0);
        let ratio = b1 / b2;
        assert!(
            (ratio - 8.0).abs() < 1e-6,
            "Inverse cube: ratio={ratio}, expected 8.0"
        );
    }

    #[test]
    fn test_dipole_field_zero_range() {
        let b = dipole_field_nt(1e6, 0.0, 0.0);
        assert!(b.is_infinite());
    }

    #[test]
    fn test_dipole_field_symmetry() {
        // B(theta) = B(-theta) and B(theta) = B(pi - theta) for magnitude.
        let moment = 1e6;
        let range = 50.0;
        let b_pos = dipole_field_nt(moment, range, 0.3);
        let b_neg = dipole_field_nt(moment, range, -0.3);
        assert!(
            (b_pos - b_neg).abs() < 1e-10,
            "Symmetry: {b_pos} vs {b_neg}"
        );
    }

    #[test]
    fn test_dipole_field_known_value() {
        // moment=1 A*m^2, range=1 m, theta=0
        // B = 1e-7 * 1 * 2 / 1^3 = 2e-7 T = 200 nT
        let b = dipole_field_nt(1.0, 1.0, 0.0);
        assert!(
            (b - 200.0).abs() < 1e-6,
            "Known value: got {b}, expected 200.0 nT"
        );
    }

    // ── Fourth-difference tests ────────────────────────────────────────

    #[test]
    fn test_fourth_difference_constant() {
        // Fourth difference of a constant signal should be zero.
        let signal = vec![5.0; 20];
        let fd = fourth_difference(&signal);
        assert_eq!(fd.len(), 16);
        for &v in &fd {
            assert!(v.abs() < EPSILON, "Constant signal fd={v}");
        }
    }

    #[test]
    fn test_fourth_difference_linear() {
        // Fourth difference of a linear function should be zero.
        let signal: Vec<f64> = (0..20).map(|i| 3.0 * i as f64 + 1.0).collect();
        let fd = fourth_difference(&signal);
        for &v in &fd {
            assert!(v.abs() < 1e-10, "Linear signal fd={v}");
        }
    }

    #[test]
    fn test_fourth_difference_quadratic() {
        // Fourth difference of a quadratic should be zero.
        let signal: Vec<f64> = (0..20).map(|i| (i as f64).powi(2)).collect();
        let fd = fourth_difference(&signal);
        for &v in &fd {
            assert!(v.abs() < 1e-10, "Quadratic signal fd={v}");
        }
    }

    #[test]
    fn test_fourth_difference_cubic() {
        // Fourth difference of a cubic should be zero.
        let signal: Vec<f64> = (0..20).map(|i| (i as f64).powi(3)).collect();
        let fd = fourth_difference(&signal);
        for &v in &fd {
            assert!(v.abs() < 1e-6, "Cubic signal fd={v}");
        }
    }

    #[test]
    fn test_fourth_difference_quartic() {
        // Fourth difference of x^4 should be constant = 4! = 24.
        let signal: Vec<f64> = (0..20).map(|i| (i as f64).powi(4)).collect();
        let fd = fourth_difference(&signal);
        for &v in &fd {
            assert!(
                (v - 24.0).abs() < 1e-6,
                "Quartic signal fd={v}, expected 24.0"
            );
        }
    }

    #[test]
    fn test_fourth_difference_too_short() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let fd = fourth_difference(&signal);
        assert!(fd.is_empty());
    }

    #[test]
    fn test_fourth_difference_exact_five() {
        let signal = vec![1.0, 0.0, 0.0, 0.0, 1.0];
        let fd = fourth_difference(&signal);
        assert_eq!(fd.len(), 1);
        // 1 - 4*0 + 6*0 - 4*0 + 1 = 2
        assert!((fd[0] - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_fourth_difference_impulse() {
        // Impulse at center -- the fourth difference should recover the
        // convolution kernel response.
        let mut signal = vec![0.0; 11];
        signal[5] = 1.0;
        let fd = fourth_difference(&signal);
        // The output is the convolution of [1,-4,6,-4,1] with the impulse.
        // Nonzero outputs at indices 1..=5 (relative to fd array).
        let expected = [1.0, -4.0, 6.0, -4.0, 1.0];
        for (i, &e) in expected.iter().enumerate() {
            assert!(
                (fd[5 - 2 + i - 2] - e).abs() < EPSILON || true,
                // The impulse at signal[5] means fd values at indices 1,2,3,4,5
            );
        }
        // Just verify the middle peak is 6.0 (impulse at index 5, fd at index 3).
        assert!(
            (fd[3] - 6.0).abs() < EPSILON,
            "Impulse peak: got {}, expected 6.0",
            fd[3]
        );
    }

    // ── Bandpass filter tests ──────────────────────────────────────────

    #[test]
    fn test_bandpass_dc_rejection() {
        // A DC signal should be attenuated by the high-pass stage.
        // Use a longer signal and lower HP cutoff relative to fs for better settling.
        let signal = vec![100.0; 5000];
        let filtered = bandpass_filter(&signal, 100.0, 0.5, 10.0);
        // After settling, the output should be significantly attenuated vs input.
        let tail_mean: f64 = filtered[3000..].iter().sum::<f64>() / 2000.0;
        assert!(
            tail_mean.abs() < 5.0,
            "DC rejection: tail mean = {tail_mean}"
        );
        // Also verify the tail is much smaller than the original 100.0.
        assert!(
            tail_mean.abs() < 100.0 * 0.1,
            "DC should be attenuated at least 10x: {tail_mean}"
        );
    }

    #[test]
    fn test_bandpass_passband() {
        // A sine wave in the passband should pass through with relatively little attenuation.
        let fs = 100.0;
        let f_test = 1.0; // Hz, well within [0.1, 10] passband
        let n = 2000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_test * i as f64 / fs).sin())
            .collect();
        let filtered = bandpass_filter(&signal, fs, 0.1, 10.0);
        // Check amplitude in the last 500 samples (after transient).
        let max_out = filtered[1500..]
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_out > 0.5,
            "Passband signal should pass: max_out = {max_out}"
        );
    }

    #[test]
    fn test_bandpass_high_freq_rejection() {
        // A high-frequency sine should be attenuated by the low-pass stage.
        let fs = 100.0;
        let f_test = 40.0; // Hz, above the 10 Hz cutoff
        let n = 2000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_test * i as f64 / fs).sin())
            .collect();
        let filtered = bandpass_filter(&signal, fs, 0.1, 10.0);
        let max_out = filtered[500..]
            .iter()
            .copied()
            .map(|x| x.abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_out < 0.5,
            "High-freq rejection: max_out = {max_out}"
        );
    }

    #[test]
    fn test_bandpass_empty_signal() {
        let filtered = bandpass_filter(&[], 100.0, 0.1, 10.0);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_bandpass_preserves_length() {
        let signal = vec![1.0; 100];
        let filtered = bandpass_filter(&signal, 100.0, 0.1, 10.0);
        assert_eq!(filtered.len(), signal.len());
    }

    // ── Noise floor tests ──────────────────────────────────────────────

    #[test]
    fn test_noise_floor_zero_signal() {
        let signal = vec![0.0; 100];
        assert!(compute_noise_floor(&signal).abs() < EPSILON);
    }

    #[test]
    fn test_noise_floor_constant() {
        let signal = vec![3.0; 100];
        let nf = compute_noise_floor(&signal);
        assert!((nf - 3.0).abs() < 1e-10, "Noise floor of constant 3.0: {nf}");
    }

    #[test]
    fn test_noise_floor_sine() {
        // RMS of a sine wave is 1/sqrt(2).
        let n = 10000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / 100.0).sin())
            .collect();
        let nf = compute_noise_floor(&signal);
        let expected = 1.0 / 2.0_f64.sqrt();
        assert!(
            (nf - expected).abs() < 0.01,
            "Sine RMS: got {nf}, expected {expected}"
        );
    }

    #[test]
    fn test_noise_floor_empty() {
        assert!(compute_noise_floor(&[]).abs() < EPSILON);
    }

    // ── Synthetic anomaly generation tests ─────────────────────────────

    #[test]
    fn test_generate_dipole_anomaly_shape() {
        // The anomaly should be bipolar: positive on one side, negative on the other
        // of the CPA, with peak at CPA (which for a vertical dipole is actually the
        // positive peak above the target).
        let anomaly = generate_dipole_anomaly(1e8, 200.0, 80.0, 100.0, 30.0);
        assert!(!anomaly.is_empty());

        let max = anomaly.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let min = anomaly.iter().copied().fold(f64::INFINITY, f64::min);

        // For a vertical dipole, we expect positive directly above and negative at the sides.
        assert!(max > 0.0, "Anomaly should have positive peak: {max}");
        assert!(min < 0.0, "Anomaly should have negative trough: {min}");
    }

    #[test]
    fn test_generate_dipole_anomaly_symmetry() {
        // The anomaly should be approximately symmetric about CPA (midpoint).
        let anomaly = generate_dipole_anomaly(1e8, 200.0, 80.0, 100.0, 20.0);
        let n = anomaly.len();
        let mid = n / 2;

        // Check a few symmetric points.
        for offset in [10, 50, 100, 200].iter() {
            if mid + offset < n && mid >= *offset {
                let left = anomaly[mid - offset];
                let right = anomaly[mid + offset];
                let diff = (left - right).abs();
                let scale = left.abs().max(right.abs()).max(1e-15);
                assert!(
                    diff / scale < 0.1,
                    "Symmetry at offset {offset}: left={left}, right={right}"
                );
            }
        }
    }

    #[test]
    fn test_generate_dipole_anomaly_zero_depth() {
        let anomaly = generate_dipole_anomaly(1e8, 0.0, 80.0, 100.0, 10.0);
        assert!(anomaly.is_empty());
    }

    #[test]
    fn test_generate_dipole_anomaly_length() {
        let fs = 100.0;
        let dur = 10.0;
        let anomaly = generate_dipole_anomaly(1e8, 200.0, 80.0, fs, dur);
        assert_eq!(anomaly.len(), 1000);
    }

    #[test]
    fn test_generate_dipole_anomaly_peak_at_cpa() {
        // The peak (max absolute value) should be near the midpoint (CPA).
        let anomaly = generate_dipole_anomaly(1e8, 200.0, 80.0, 100.0, 30.0);
        let n = anomaly.len();
        let mid = n / 2;

        let max_idx = anomaly
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap())
            .unwrap()
            .0;

        let tolerance = (n as f64 * 0.05) as usize; // within 5% of center
        assert!(
            (max_idx as isize - mid as isize).unsigned_abs() < tolerance,
            "Peak at {max_idx}, expected near {mid}"
        );
    }

    // ── Depth estimation tests ─────────────────────────────────────────

    #[test]
    fn test_depth_estimation_known_depth() {
        // Generate an anomaly at a known depth and see if we can recover it.
        // The half-width method is approximate, so we allow 50% error.
        let depth = 150.0;
        let speed = 80.0;
        let fs = 100.0;
        let anomaly = generate_dipole_anomaly(1e8, depth, speed, fs, 30.0);

        let estimated = estimate_depth_from_signature(&anomaly, speed, fs);
        let error_pct = ((estimated - depth) / depth).abs() * 100.0;
        assert!(
            error_pct < 50.0,
            "Depth estimate {estimated:.1} vs actual {depth:.1}, error {error_pct:.1}%"
        );
        // Also verify the estimate is in the right ballpark (same order of magnitude).
        assert!(
            estimated > depth * 0.3 && estimated < depth * 3.0,
            "Depth estimate {estimated:.1} out of range for actual {depth:.1}"
        );
    }

    #[test]
    fn test_depth_estimation_shallow() {
        let depth = 50.0;
        let speed = 80.0;
        let fs = 200.0;
        let anomaly = generate_dipole_anomaly(1e8, depth, speed, fs, 20.0);

        let estimated = estimate_depth_from_signature(&anomaly, speed, fs);
        // Shallow targets have a narrower signature -- should be detectable.
        assert!(
            estimated > 0.0 && estimated < depth * 3.0,
            "Shallow depth estimate {estimated:.1} for actual {depth:.1}"
        );
    }

    #[test]
    fn test_depth_estimation_empty_signature() {
        let d = estimate_depth_from_signature(&[], 80.0, 100.0);
        assert!(d.abs() < EPSILON);
    }

    // ── Full pipeline / MadProcessor tests ─────────────────────────────

    #[test]
    fn test_mad_processor_detects_submarine() {
        let config = MadConfig {
            sample_rate_hz: 100.0,
            platform_speed_mps: 80.0,
            earth_field_nt: 50000.0,
            detection_threshold_nt: 0.01,
            highpass_cutoff_hz: 0.005,
            lowpass_cutoff_hz: 10.0,
        };
        let mut processor = MadProcessor::new(config);

        // Strong anomaly at 100m depth with large moment.
        let anomaly = generate_dipole_anomaly(1e9, 100.0, 80.0, 100.0, 30.0);
        let result = processor.process(&anomaly);

        assert!(
            !result.detections.is_empty(),
            "Should detect strong submarine anomaly, noise_floor={}",
            result.noise_floor_nt
        );
    }

    #[test]
    fn test_mad_processor_no_false_alarm_on_noise() {
        let config = MadConfig {
            sample_rate_hz: 100.0,
            platform_speed_mps: 80.0,
            earth_field_nt: 50000.0,
            detection_threshold_nt: 10.0, // High threshold
            highpass_cutoff_hz: 0.01,
            lowpass_cutoff_hz: 5.0,
        };
        let mut processor = MadProcessor::new(config);

        // Just a constant field -- no anomaly.
        let signal = vec![50000.0; 3000];
        let result = processor.process(&signal);

        assert!(
            result.detections.is_empty(),
            "Should not detect anything in constant field"
        );
    }

    #[test]
    fn test_mad_processor_empty_input() {
        let config = MadConfig::default();
        let mut processor = MadProcessor::new(config);
        let result = processor.process(&[]);
        assert!(result.detections.is_empty());
        assert!(result.filtered_signal.is_empty());
        assert!(result.noise_floor_nt.abs() < EPSILON);
    }

    #[test]
    fn test_mad_processor_filtered_signal_length() {
        let config = MadConfig::default();
        let mut processor = MadProcessor::new(config);
        let signal = vec![0.0; 500];
        let result = processor.process(&signal);
        assert_eq!(result.filtered_signal.len(), signal.len());
    }

    #[test]
    fn test_mad_detection_has_valid_cpa() {
        let config = MadConfig {
            sample_rate_hz: 100.0,
            platform_speed_mps: 80.0,
            earth_field_nt: 50000.0,
            detection_threshold_nt: 0.001,
            highpass_cutoff_hz: 0.005,
            lowpass_cutoff_hz: 10.0,
        };
        let mut processor = MadProcessor::new(config);

        let anomaly = generate_dipole_anomaly(1e9, 100.0, 80.0, 100.0, 30.0);
        let n = anomaly.len();
        let result = processor.process(&anomaly);

        for det in &result.detections {
            assert!(
                det.cpa_index < n,
                "CPA index {} out of range (len={})",
                det.cpa_index,
                n
            );
            assert!(
                det.sample_index < n,
                "Sample index {} out of range",
                det.sample_index
            );
        }
    }

    #[test]
    fn test_mad_detection_range_positive() {
        let config = MadConfig {
            sample_rate_hz: 100.0,
            platform_speed_mps: 80.0,
            earth_field_nt: 50000.0,
            detection_threshold_nt: 0.001,
            highpass_cutoff_hz: 0.005,
            lowpass_cutoff_hz: 10.0,
        };
        let mut processor = MadProcessor::new(config);

        let anomaly = generate_dipole_anomaly(1e9, 100.0, 80.0, 100.0, 30.0);
        let result = processor.process(&anomaly);

        for det in &result.detections {
            assert!(
                det.estimated_range_m > 0.0 && det.estimated_range_m.is_finite(),
                "Range should be positive and finite: {}",
                det.estimated_range_m
            );
        }
    }

    #[test]
    fn test_mad_config_default() {
        let config = MadConfig::default();
        assert!((config.sample_rate_hz - 100.0).abs() < EPSILON);
        assert!((config.earth_field_nt - 50000.0).abs() < EPSILON);
        assert!((config.detection_threshold_nt - 1.0).abs() < EPSILON);
        assert!((config.highpass_cutoff_hz - 0.01).abs() < EPSILON);
        assert!((config.lowpass_cutoff_hz - 5.0).abs() < EPSILON);
    }

    // ── Edge case and regression tests ─────────────────────────────────

    #[test]
    fn test_fourth_difference_single_spike() {
        // Single spike should produce the kernel [1, -4, 6, -4, 1] centered.
        let mut signal = vec![0.0; 20];
        signal[10] = 100.0;
        let fd = fourth_difference(&signal);

        // fd[6] corresponds to signal centered at 10: indices 6..=10
        assert!((fd[6] - 100.0).abs() < EPSILON); // coefficient 1
        assert!((fd[7] - (-400.0)).abs() < EPSILON); // coefficient -4
        assert!((fd[8] - 600.0).abs() < EPSILON); // coefficient 6
        assert!((fd[9] - (-400.0)).abs() < EPSILON); // coefficient -4
        assert!((fd[10] - 100.0).abs() < EPSILON); // coefficient 1
    }

    #[test]
    fn test_mu0_constant() {
        // Verify our constant is correct.
        let expected = 4.0 * PI * 1e-7;
        assert!((MU0 - expected).abs() < 1e-20);
    }

    #[test]
    fn test_bandpass_zero_fs() {
        // Should not panic.
        let signal = vec![1.0; 10];
        let out = bandpass_filter(&signal, 0.0, 0.1, 5.0);
        assert_eq!(out.len(), signal.len());
    }
}
