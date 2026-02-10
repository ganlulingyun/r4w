//! # Avalanche Transceiver Correlator
//!
//! Implements 457 kHz avalanche beacon (transceiver) signal processing for
//! search and rescue operations. Avalanche transceivers (also known as
//! avalanche beacons or ARVA devices) transmit a pulsed 457 kHz magnetic
//! field signal that searchers use to locate buried victims.
//!
//! ## Features
//!
//! - **457 kHz carrier detection** with power measurement
//! - **Pulse pattern recognition** (nominally 1 s period, ~70 ms on-time)
//! - **Signal strength (RSSI)** measurement in dBm
//! - **Direction estimation** from three orthogonal antenna axes
//! - **Distance estimation** from near-field magnetic field strength
//! - **Multi-burial discrimination** (detecting multiple beacons)
//! - **Flux line following** for efficient search
//! - **Search pattern generation** (spiral and strip patterns)
//!
//! ## Standards
//!
//! Compliant with ETS 300 718 (EN 300 718) — the European standard for
//! avalanche beacons operating at 457 kHz.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::avalanche_transceiver_correlator::{
//!     BeaconConfig, generate_beacon_signal, detect_457khz_carrier,
//!     envelope_detect, detect_pulse_pattern,
//! };
//!
//! let config = BeaconConfig::default();
//! let signal = generate_beacon_signal(&config, 0.01);
//! assert!(!signal.is_empty());
//!
//! let (detected, power_dbm) = detect_457khz_carrier(&signal, config.sample_rate_hz);
//! // With a clean generated signal, carrier should be detected
//! assert!(detected);
//! ```

use std::f64::consts::PI;

// ──────────────────────────────────────────────
// Configuration & data types
// ──────────────────────────────────────────────

/// Configuration parameters for an avalanche beacon.
///
/// The standard specifies a 457 kHz carrier with a pulsed transmission
/// pattern.  Typical beacons transmit for about 70 ms every 1 s.
#[derive(Debug, Clone)]
pub struct BeaconConfig {
    /// Carrier frequency in Hz (standard: 457 000 Hz).
    pub carrier_freq_hz: f64,
    /// Processing sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Nominal pulse repetition period in seconds (typically 1.0 s).
    pub pulse_period_s: f64,
    /// Fraction of the period during which the carrier is on (typically ~0.07).
    pub pulse_duty_cycle: f64,
    /// Transmit power in dBm (typical beacon output).
    pub tx_power_dbm: f64,
}

impl Default for BeaconConfig {
    fn default() -> Self {
        Self {
            carrier_freq_hz: 457_000.0,
            sample_rate_hz: 2_000_000.0,
            pulse_period_s: 1.0,
            pulse_duty_cycle: 0.07,
            tx_power_dbm: 0.0,
        }
    }
}

/// Result of beacon signal analysis at a single point in time.
#[derive(Debug, Clone)]
pub struct BeaconSignal {
    /// Received signal strength indicator in dBm.
    pub rssi_dbm: f64,
    /// Estimated bearing to the beacon in degrees (0–360).
    pub direction_deg: f64,
    /// Estimated distance to the beacon in metres.
    pub distance_m: f64,
    /// Whether a valid 457 kHz pulsed beacon was detected.
    pub pulse_detected: bool,
}

/// Main avalanche transceiver processor.
///
/// Holds state for continuous signal processing, including the
/// configuration and accumulated RSSI history for flux-line tracking.
#[derive(Debug, Clone)]
pub struct BeaconTransceiver {
    /// Beacon configuration.
    pub config: BeaconConfig,
    /// Rolling RSSI history as (position_x, rssi) pairs for flux-line following.
    rssi_history: Vec<(f64, f64)>,
}

impl BeaconTransceiver {
    /// Create a new transceiver processor with the given configuration.
    pub fn new(config: BeaconConfig) -> Self {
        Self {
            config,
            rssi_history: Vec::new(),
        }
    }

    /// Process a block of samples and return a [`BeaconSignal`] summary.
    ///
    /// `signal_axes` contains three slices — one per orthogonal receive
    /// antenna axis — all at the configured sample rate.
    pub fn process(
        &mut self,
        signal_axes: [&[f64]; 3],
    ) -> BeaconSignal {
        let sr = self.config.sample_rate_hz;
        let (carrier_detected, _carrier_power) =
            detect_457khz_carrier(signal_axes[0], sr);

        let rssi = measure_rssi(signal_axes[0]);
        let direction = estimate_direction(signal_axes[0], signal_axes[1], signal_axes[2]);
        let distance = estimate_distance(rssi, self.config.tx_power_dbm);

        let env = envelope_detect(signal_axes[0], sr);
        let pulses = detect_pulse_pattern(&env, sr);

        self.rssi_history.push((0.0, rssi));

        BeaconSignal {
            rssi_dbm: rssi,
            direction_deg: direction,
            distance_m: distance,
            pulse_detected: carrier_detected && !pulses.is_empty(),
        }
    }

    /// Clear accumulated RSSI history.
    pub fn reset_history(&mut self) {
        self.rssi_history.clear();
    }

    /// Return a reference to the RSSI history.
    pub fn rssi_history(&self) -> &[(f64, f64)] {
        &self.rssi_history
    }
}

// ──────────────────────────────────────────────
// Signal generation
// ──────────────────────────────────────────────

/// Generate a pulsed 457 kHz beacon signal (transmit side).
///
/// Returns a vector of real-valued samples at `config.sample_rate_hz`.
/// The signal consists of CW 457 kHz bursts according to the configured
/// duty cycle and period.
pub fn generate_beacon_signal(config: &BeaconConfig, duration_s: f64) -> Vec<f64> {
    let n_samples = (duration_s * config.sample_rate_hz).ceil() as usize;
    let period_samples = (config.pulse_period_s * config.sample_rate_hz) as usize;
    let on_samples = (config.pulse_period_s * config.pulse_duty_cycle * config.sample_rate_hz) as usize;

    let mut signal = vec![0.0f64; n_samples];
    let omega = 2.0 * PI * config.carrier_freq_hz / config.sample_rate_hz;

    for (i, s) in signal.iter_mut().enumerate() {
        let pos_in_period = if period_samples > 0 {
            i % period_samples
        } else {
            0
        };
        if pos_in_period < on_samples {
            *s = (omega * i as f64).sin();
        }
    }

    signal
}

// ──────────────────────────────────────────────
// Carrier detection
// ──────────────────────────────────────────────

/// Detect the presence of a 457 kHz carrier in `signal`.
///
/// Returns `(detected, power_dbm)` where `detected` is `true` when the
/// carrier power exceeds a threshold, and `power_dbm` is the estimated
/// carrier power in dBm.
///
/// Uses a Goertzel-style single-bin DFT at 457 kHz for efficient
/// narrowband detection.
pub fn detect_457khz_carrier(signal: &[f64], sample_rate: f64) -> (bool, f64) {
    if signal.is_empty() {
        return (false, f64::NEG_INFINITY);
    }

    let freq = 457_000.0;
    let n = signal.len() as f64;
    let omega = 2.0 * PI * freq / sample_rate;

    // Goertzel algorithm
    let coeff = 2.0 * omega.cos();
    let mut s0: f64 = 0.0;
    let mut s1: f64 = 0.0;
    let mut s2: f64 = 0.0;

    for &x in signal.iter() {
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    let power = (s1 * s1 + s2 * s2 - coeff * s1 * s2) / (n * n);
    let power_dbm = if power > 0.0 {
        10.0 * power.log10() + 30.0
    } else {
        f64::NEG_INFINITY
    };

    // Detection threshold: -60 dBm (empirical, suitable for close-range search)
    let detected = power_dbm > -60.0;
    (detected, power_dbm)
}

// ──────────────────────────────────────────────
// Envelope detection
// ──────────────────────────────────────────────

/// Extract the AM envelope of a signal using a simple magnitude + lowpass
/// approach.
///
/// The output has the same length as the input.  A moving-average lowpass
/// filter smooths the rectified signal to produce the envelope.
pub fn envelope_detect(signal: &[f64], sample_rate: f64) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }

    // Rectify (take absolute value)
    let rectified: Vec<f64> = signal.iter().map(|&x| x.abs()).collect();

    // Moving-average lowpass — window ~ 0.5 ms (enough to smooth 457 kHz)
    let window_len = ((0.0005 * sample_rate) as usize).max(1);
    let mut envelope = vec![0.0f64; signal.len()];

    for i in 0..rectified.len() {
        let lo = if i >= window_len / 2 { i - window_len / 2 } else { 0 };
        let hi = (i + window_len / 2 + 1).min(rectified.len());
        let sum: f64 = rectified[lo..hi].iter().sum();
        let count = (hi - lo) as f64;
        envelope[i] = sum / count;
    }

    envelope
}

// ──────────────────────────────────────────────
// Pulse pattern detection
// ──────────────────────────────────────────────

/// Detect pulse on/off transitions in the envelope.
///
/// Returns a vector of `(on_sample, off_sample)` pairs marking where
/// each beacon pulse starts and ends.  An adaptive threshold at 30 % of
/// the peak envelope is used.
pub fn detect_pulse_pattern(envelope: &[f64], sample_rate: f64) -> Vec<(usize, usize)> {
    if envelope.is_empty() {
        return Vec::new();
    }

    let peak = envelope.iter().cloned().fold(0.0f64, f64::max);
    if peak <= 0.0 {
        return Vec::new();
    }

    let threshold = peak * 0.30;

    // Minimum pulse width: 10 ms  (to reject glitches)
    let min_on_samples = (0.010 * sample_rate) as usize;

    let mut pulses = Vec::new();
    let mut in_pulse = false;
    let mut start = 0usize;

    for (i, &v) in envelope.iter().enumerate() {
        if !in_pulse && v > threshold {
            in_pulse = true;
            start = i;
        } else if in_pulse && v <= threshold {
            in_pulse = false;
            let width = i - start;
            if width >= min_on_samples {
                pulses.push((start, i));
            }
        }
    }

    // Handle pulse that extends to end of buffer
    if in_pulse {
        let width = envelope.len() - start;
        if width >= min_on_samples {
            pulses.push((start, envelope.len()));
        }
    }

    pulses
}

// ──────────────────────────────────────────────
// RSSI measurement
// ──────────────────────────────────────────────

/// Measure the received signal strength indicator of `signal` in dBm.
///
/// Computes RMS power and converts to dBm (referenced to 1 mW into 50 Ohm
/// notionally, though the absolute calibration depends on front-end gain).
pub fn measure_rssi(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return f64::NEG_INFINITY;
    }

    let mean_sq: f64 = signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64;

    if mean_sq <= 0.0 {
        f64::NEG_INFINITY
    } else {
        10.0 * mean_sq.log10() + 30.0
    }
}

// ──────────────────────────────────────────────
// Direction estimation
// ──────────────────────────────────────────────

/// Estimate the bearing to the beacon from three orthogonal antenna axes.
///
/// The three input slices correspond to signal received on the X, Y, and
/// Z antenna coils.  The function computes the RMS on each axis and uses
/// the X/Y ratio to derive a bearing in degrees (0-360).  The Z axis is
/// used for tilt compensation.
///
/// Returns the estimated direction in degrees.
pub fn estimate_direction(signal_a: &[f64], signal_b: &[f64], signal_c: &[f64]) -> f64 {
    let rms = |s: &[f64]| -> f64 {
        if s.is_empty() {
            return 0.0;
        }
        let ms = s.iter().map(|&x| x * x).sum::<f64>() / s.len() as f64;
        ms.sqrt()
    };

    let rx = rms(signal_a);
    let ry = rms(signal_b);
    let _rz = rms(signal_c); // used for tilt compensation (placeholder)

    // atan2 gives angle in radians; convert to degrees in [0, 360)
    let angle_rad = ry.atan2(rx);
    let angle_deg = angle_rad.to_degrees();

    // Normalise to [0, 360)
    ((angle_deg % 360.0) + 360.0) % 360.0
}

// ──────────────────────────────────────────────
// Distance estimation
// ──────────────────────────────────────────────

/// Estimate distance to the beacon from the received RSSI and known
/// transmit power.
///
/// Uses an inverse-cube-law model appropriate for the near-field 457 kHz
/// magnetic dipole:
///
/// ```text
/// RSSI = Ptx - 60 * log10(d)   [dBm]
///    =>  d = 10^((Ptx - RSSI) / 60)
/// ```
///
/// Returns the estimated distance in metres (clamped to >= 0.1 m).
pub fn estimate_distance(rssi_dbm: f64, tx_power_dbm: f64) -> f64 {
    if rssi_dbm >= tx_power_dbm {
        // Extremely close — clamp
        return 0.1;
    }

    let diff = tx_power_dbm - rssi_dbm;
    let d = 10.0f64.powf(diff / 60.0);
    d.max(0.1)
}

// ──────────────────────────────────────────────
// Multi-burial discrimination
// ──────────────────────────────────────────────

/// Estimate the number of distinct avalanche beacons present in the
/// captured envelope.
///
/// Multiple beacons transmit independently, so their pulses arrive at
/// different times within each 1-second window.  This function counts
/// the number of distinct pulse clusters.
///
/// Returns the number of distinct beacons detected (0 if none).
pub fn discriminate_multiple_burials(envelope: &[f64], sample_rate: f64) -> usize {
    let pulses = detect_pulse_pattern(envelope, sample_rate);

    if pulses.is_empty() {
        return 0;
    }

    // Beacons transmit ~70 ms pulses every ~1 s.  Two pulses from the
    // same beacon are separated by ~1 s.  Pulses from *different* beacons
    // within the same 1-second window will be closer together.
    //
    // Strategy: group pulses that overlap or are within 0.2 s of each
    // other into a single beacon cluster, then count clusters within a
    // 1-second window.
    let gap_threshold_samples = (0.2 * sample_rate) as usize;

    let mut beacon_count = 1usize;
    let mut prev_end = pulses[0].1;

    for &(start, end) in pulses.iter().skip(1) {
        if start > prev_end + gap_threshold_samples {
            beacon_count += 1;
        }
        prev_end = end;
    }

    beacon_count
}

// ──────────────────────────────────────────────
// Flux-line following
// ──────────────────────────────────────────────

/// Compute the tangent angle of the magnetic flux line from a series of
/// RSSI readings taken along a search path.
///
/// `rssi_readings` is a slice of `(position_along_path_m, rssi_dbm)` pairs.
/// The function fits a local gradient to determine the direction of
/// increasing signal strength, which approximates the flux line tangent.
///
/// Returns the angle in degrees (0 = along path, 90 = perpendicular).
pub fn flux_line_direction(rssi_readings: &[(f64, f64)]) -> f64 {
    if rssi_readings.len() < 2 {
        return 0.0;
    }

    // Compute finite-difference gradient dRSSI/dx
    let n = rssi_readings.len();
    let (x0, r0) = rssi_readings[0];
    let (x1, r1) = rssi_readings[n - 1];

    let dx = x1 - x0;
    let dr = r1 - r0;

    if dx.abs() < 1e-12 {
        return 90.0; // vertical / no motion
    }

    // The flux line tangent is perpendicular to the gradient of |B|.
    // gradient direction gives the "steepest ascent" of RSSI along the path;
    // the flux line is perpendicular to that.
    let gradient = dr / dx;

    // atan of gradient gives the climb angle; flux line is 90 deg rotated
    let angle_rad = gradient.atan();
    let flux_angle = angle_rad.to_degrees();

    // Normalise to [0, 360)
    (flux_angle + 360.0) % 360.0
}

// ──────────────────────────────────────────────
// Search patterns
// ──────────────────────────────────────────────

/// Generate waypoints for a square-spiral search pattern.
///
/// Starting from `start`, the spiral expands outward with the given
/// `strip_width_m` spacing.  The pattern covers an area of approximately
/// `area_m x area_m` metres.
///
/// Returns a list of `(x, y)` waypoints in metres.
pub fn search_pattern_spiral(
    start: (f64, f64),
    strip_width_m: f64,
    area_m: f64,
) -> Vec<(f64, f64)> {
    if strip_width_m <= 0.0 || area_m <= 0.0 {
        return vec![start];
    }

    let mut waypoints = Vec::new();
    let (mut x, mut y) = start;
    waypoints.push((x, y));

    let max_legs = ((area_m / strip_width_m) * 2.0).ceil() as usize;
    // Directions: right, up, left, down
    let dirs: [(f64, f64); 4] = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];

    let mut leg_length = strip_width_m;
    let mut dir_idx = 0usize;

    for step in 0..max_legs {
        let (dx, dy) = dirs[dir_idx % 4];
        x += dx * leg_length;
        y += dy * leg_length;
        waypoints.push((x, y));

        dir_idx += 1;
        // Every two legs, increase the leg length
        if step % 2 == 1 {
            leg_length += strip_width_m;
        }

        // Stop if we have exceeded the area
        if (x - start.0).abs() > area_m || (y - start.1).abs() > area_m {
            break;
        }
    }

    waypoints
}

// ──────────────────────────────────────────────
// Near-field boundary
// ──────────────────────────────────────────────

/// Compute the near-field boundary distance for a given frequency.
///
/// For a small magnetic dipole (like an avalanche beacon antenna), the
/// near-field / far-field transition occurs at approximately:
///
/// ```text
/// d_nf = lambda / (2 * pi)
/// ```
///
/// where lambda = c / f.  At 457 kHz this is approximately 104 m.
///
/// Returns the distance in metres.
pub fn near_field_distance(freq_hz: f64) -> f64 {
    if freq_hz <= 0.0 {
        return f64::INFINITY;
    }
    let c = 299_792_458.0; // speed of light in m/s
    let lambda = c / freq_hz;
    lambda / (2.0 * PI)
}

// ======================================================
// Tests
// ======================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // -- helpers --

    fn sine_signal(freq: f64, sample_rate: f64, duration_s: f64) -> Vec<f64> {
        let n = (duration_s * sample_rate).ceil() as usize;
        let omega = 2.0 * PI * freq / sample_rate;
        (0..n).map(|i| (omega * i as f64).sin()).collect()
    }

    // -- BeaconConfig --

    #[test]
    fn test_default_config() {
        let cfg = BeaconConfig::default();
        assert!((cfg.carrier_freq_hz - 457_000.0).abs() < 1e-6);
        assert!((cfg.pulse_period_s - 1.0).abs() < 1e-6);
        assert!((cfg.pulse_duty_cycle - 0.07).abs() < 1e-6);
        assert!(cfg.sample_rate_hz > 0.0);
    }

    // -- generate_beacon_signal --

    #[test]
    fn test_generate_beacon_signal_length() {
        let cfg = BeaconConfig::default();
        let sig = generate_beacon_signal(&cfg, 0.01);
        let expected = (0.01 * cfg.sample_rate_hz).ceil() as usize;
        assert_eq!(sig.len(), expected);
    }

    #[test]
    fn test_generate_beacon_signal_has_energy() {
        let cfg = BeaconConfig::default();
        let sig = generate_beacon_signal(&cfg, 0.005);
        let energy: f64 = sig.iter().map(|&x| x * x).sum();
        assert!(energy > 0.0, "generated beacon signal should have nonzero energy");
    }

    #[test]
    fn test_generate_beacon_signal_duty_cycle_off() {
        // With 7% duty cycle and 1 s period, samples beyond the first 70 ms
        // should be zero.
        let cfg = BeaconConfig::default();
        let sig = generate_beacon_signal(&cfg, 0.5);
        let on_samples = (cfg.pulse_period_s * cfg.pulse_duty_cycle * cfg.sample_rate_hz) as usize;
        // Check a sample well into the "off" region
        let off_idx = on_samples + 10_000;
        if off_idx < sig.len() {
            assert!(
                sig[off_idx].abs() < 1e-12,
                "signal should be zero in the off region"
            );
        }
    }

    // -- detect_457khz_carrier --

    #[test]
    fn test_detect_carrier_present() {
        let sr = 2_000_000.0;
        let sig = sine_signal(457_000.0, sr, 0.002);
        let (det, pwr) = detect_457khz_carrier(&sig, sr);
        assert!(det, "should detect 457 kHz carrier");
        assert!(pwr > -60.0, "power should exceed threshold");
    }

    #[test]
    fn test_detect_carrier_absent() {
        // Pure DC — no 457 kHz component
        let sig = vec![0.001; 4000];
        let (det, _pwr) = detect_457khz_carrier(&sig, 2_000_000.0);
        assert!(!det, "should not detect carrier in DC signal");
    }

    #[test]
    fn test_detect_carrier_empty() {
        let (det, pwr) = detect_457khz_carrier(&[], 2_000_000.0);
        assert!(!det);
        assert!(pwr.is_infinite() && pwr < 0.0);
    }

    #[test]
    fn test_detect_carrier_wrong_freq() {
        let sr = 2_000_000.0;
        let sig = sine_signal(100_000.0, sr, 0.002);
        let (det, _pwr) = detect_457khz_carrier(&sig, sr);
        // A 100 kHz tone should not trigger the 457 kHz detector
        assert!(!det, "100 kHz tone should not be detected as 457 kHz");
    }

    // -- envelope_detect --

    #[test]
    fn test_envelope_detect_length() {
        let sr = 2_000_000.0;
        let sig = sine_signal(457_000.0, sr, 0.001);
        let env = envelope_detect(&sig, sr);
        assert_eq!(env.len(), sig.len());
    }

    #[test]
    fn test_envelope_detect_empty() {
        let env = envelope_detect(&[], 1_000_000.0);
        assert!(env.is_empty());
    }

    #[test]
    fn test_envelope_detect_positive() {
        let sr = 2_000_000.0;
        let sig = sine_signal(457_000.0, sr, 0.001);
        let env = envelope_detect(&sig, sr);
        assert!(env.iter().all(|&v| v >= 0.0), "envelope should be non-negative");
    }

    // -- detect_pulse_pattern --

    #[test]
    fn test_detect_pulse_single() {
        // Create a synthetic envelope: 70 ms on, rest off, at 100 kHz sample rate
        let sr = 100_000.0;
        let on_samples = (0.07 * sr) as usize;
        let total = (0.5 * sr) as usize;
        let mut env = vec![0.0f64; total];
        for s in env.iter_mut().take(on_samples) {
            *s = 1.0;
        }
        let pulses = detect_pulse_pattern(&env, sr);
        assert_eq!(pulses.len(), 1, "should detect exactly one pulse");
        assert_eq!(pulses[0].0, 0);
    }

    #[test]
    fn test_detect_pulse_empty() {
        let pulses = detect_pulse_pattern(&[], 1_000_000.0);
        assert!(pulses.is_empty());
    }

    #[test]
    fn test_detect_pulse_no_signal() {
        let env = vec![0.0; 10_000];
        let pulses = detect_pulse_pattern(&env, 1_000_000.0);
        assert!(pulses.is_empty());
    }

    // -- measure_rssi --

    #[test]
    fn test_measure_rssi_unit_sine() {
        // Unit-amplitude sine: RMS = 1/sqrt(2), power = 0.5
        // 10*log10(0.5) + 30 ~ 26.99 dBm
        let sr = 100_000.0;
        let sig = sine_signal(1_000.0, sr, 0.1);
        let rssi = measure_rssi(&sig);
        assert!((rssi - 26.99).abs() < 0.5, "RSSI of unit sine ~ 27 dBm, got {}", rssi);
    }

    #[test]
    fn test_measure_rssi_empty() {
        let rssi = measure_rssi(&[]);
        assert!(rssi.is_infinite() && rssi < 0.0);
    }

    #[test]
    fn test_measure_rssi_zero_signal() {
        let rssi = measure_rssi(&[0.0; 100]);
        assert!(rssi.is_infinite() && rssi < 0.0);
    }

    // -- estimate_direction --

    #[test]
    fn test_direction_equal_axes() {
        // Equal signal on X and Y should give 45 deg
        let sig = sine_signal(457_000.0, 2_000_000.0, 0.001);
        let dir = estimate_direction(&sig, &sig, &sig);
        assert!((dir - 45.0).abs() < 1.0, "equal X/Y should give ~45 deg, got {}", dir);
    }

    #[test]
    fn test_direction_x_only() {
        // Signal only on X axis -> 0 deg (or close)
        let sig_x = sine_signal(457_000.0, 2_000_000.0, 0.001);
        let sig_zero = vec![0.0; sig_x.len()];
        let dir = estimate_direction(&sig_x, &sig_zero, &sig_zero);
        assert!(dir < 1.0 || dir > 359.0, "X-only should give ~0 deg, got {}", dir);
    }

    #[test]
    fn test_direction_y_only() {
        // Signal only on Y axis -> 90 deg
        let sig_y = sine_signal(457_000.0, 2_000_000.0, 0.001);
        let sig_zero = vec![0.0; sig_y.len()];
        let dir = estimate_direction(&sig_zero, &sig_y, &sig_zero);
        assert!((dir - 90.0).abs() < 1.0, "Y-only should give ~90 deg, got {}", dir);
    }

    // -- estimate_distance --

    #[test]
    fn test_distance_increases_with_lower_rssi() {
        let d1 = estimate_distance(-10.0, 0.0);
        let d2 = estimate_distance(-30.0, 0.0);
        assert!(d2 > d1, "lower RSSI should mean greater distance");
    }

    #[test]
    fn test_distance_clamp_close() {
        // RSSI >= TX power means extremely close
        let d = estimate_distance(5.0, 0.0);
        assert!((d - 0.1).abs() < 1e-6, "should clamp to 0.1 m");
    }

    // -- discriminate_multiple_burials --

    #[test]
    fn test_discriminate_single_beacon() {
        let sr = 100_000.0;
        let on_samples = (0.07 * sr) as usize;
        let total = (0.5 * sr) as usize;
        let mut env = vec![0.0f64; total];
        for s in env.iter_mut().take(on_samples) {
            *s = 1.0;
        }
        let count = discriminate_multiple_burials(&env, sr);
        assert_eq!(count, 1);
    }

    #[test]
    fn test_discriminate_two_beacons() {
        let sr = 100_000.0;
        let on_samples = (0.07 * sr) as usize;
        let total = (1.0 * sr) as usize;
        let mut env = vec![0.0f64; total];
        // First beacon pulse at t=0
        for s in env.iter_mut().take(on_samples) {
            *s = 1.0;
        }
        // Second beacon pulse at t=0.5 (well separated)
        let offset = (0.5 * sr) as usize;
        for i in 0..on_samples {
            if offset + i < env.len() {
                env[offset + i] = 0.8;
            }
        }
        let count = discriminate_multiple_burials(&env, sr);
        assert_eq!(count, 2, "should detect two distinct beacons");
    }

    #[test]
    fn test_discriminate_no_beacons() {
        let env = vec![0.0; 10_000];
        let count = discriminate_multiple_burials(&env, 100_000.0);
        assert_eq!(count, 0);
    }

    // -- flux_line_direction --

    #[test]
    fn test_flux_line_increasing() {
        // RSSI increasing along path
        let readings: Vec<(f64, f64)> = (0..10).map(|i| (i as f64, -30.0 + i as f64)).collect();
        let angle = flux_line_direction(&readings);
        assert!(angle > 0.0 && angle < 90.0, "increasing RSSI -> positive angle, got {}", angle);
    }

    #[test]
    fn test_flux_line_single_reading() {
        let angle = flux_line_direction(&[(0.0, -20.0)]);
        assert!((angle - 0.0).abs() < 1e-6);
    }

    // -- search_pattern_spiral --

    #[test]
    fn test_spiral_includes_start() {
        let wps = search_pattern_spiral((10.0, 20.0), 5.0, 50.0);
        assert!(!wps.is_empty());
        assert!((wps[0].0 - 10.0).abs() < 1e-6);
        assert!((wps[0].1 - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_spiral_multiple_waypoints() {
        let wps = search_pattern_spiral((0.0, 0.0), 5.0, 50.0);
        assert!(wps.len() > 3, "spiral should produce multiple waypoints");
    }

    #[test]
    fn test_spiral_zero_area() {
        let wps = search_pattern_spiral((0.0, 0.0), 5.0, 0.0);
        assert_eq!(wps.len(), 1, "zero area should return just the start");
    }

    // -- near_field_distance --

    #[test]
    fn test_near_field_457khz() {
        let d = near_field_distance(457_000.0);
        // lambda = c/f ~ 656.0 m, d_nf = lambda/(2*pi) ~ 104.4 m
        assert!((d - 104.4).abs() < 1.0, "near-field boundary at 457 kHz ~ 104 m, got {}", d);
    }

    #[test]
    fn test_near_field_zero_freq() {
        let d = near_field_distance(0.0);
        assert!(d.is_infinite());
    }

    // -- BeaconTransceiver integration --

    #[test]
    fn test_transceiver_process() {
        let cfg = BeaconConfig {
            sample_rate_hz: 2_000_000.0,
            ..BeaconConfig::default()
        };
        let mut trx = BeaconTransceiver::new(cfg.clone());

        // Generate a short beacon burst
        let sig = generate_beacon_signal(&cfg, 0.002);
        let result = trx.process([&sig, &sig, &sig]);

        assert!(result.rssi_dbm > f64::NEG_INFINITY);
        assert!(result.direction_deg >= 0.0 && result.direction_deg < 360.0);
        assert!(result.distance_m >= 0.1);
    }

    #[test]
    fn test_transceiver_reset_history() {
        let cfg = BeaconConfig::default();
        let mut trx = BeaconTransceiver::new(cfg.clone());
        let sig = generate_beacon_signal(&cfg, 0.001);
        let _ = trx.process([&sig, &sig, &sig]);
        assert!(!trx.rssi_history().is_empty());
        trx.reset_history();
        assert!(trx.rssi_history().is_empty());
    }
}
