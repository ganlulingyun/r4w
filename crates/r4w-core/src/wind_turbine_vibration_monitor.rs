//! Vibration-based condition monitoring for wind turbine drivetrain components.
//!
//! This module provides tools for monitoring the health of wind turbine drivetrains
//! through vibration analysis. It includes bearing fault frequency calculation
//! (BPFO, BPFI, BSF, FTF), envelope spectrum analysis for bearing diagnosis,
//! order tracking for variable-speed operation, gear mesh frequency analysis,
//! statistical indicators (crest factor, kurtosis), RMS velocity/acceleration
//! severity classification per ISO 10816, alarm threshold management, and
//! degradation trend prediction via linear extrapolation.
//!
//! # Example
//!
//! ```
//! use r4w_core::wind_turbine_vibration_monitor::*;
//!
//! // Calculate bearing fault frequencies for a typical main bearing
//! let freqs = bearing_fault_frequencies(9, 25.4, 127.0, 0.0, 1800.0);
//! assert!(freqs.bpfo > 0.0);
//! assert!(freqs.bpfi > 0.0);
//!
//! // Classify vibration severity per ISO 10816
//! let level = classify_severity_iso10816(3.5, 1);
//! assert_eq!(level, SeverityLevel::Unsatisfactory);
//!
//! // Gear mesh frequency
//! let gmf = gear_mesh_frequency(1500.0, 97);
//! assert!((gmf - 2425.0).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

/// Configuration for a wind turbine drivetrain monitoring session.
#[derive(Debug, Clone)]
pub struct TurbineConfig {
    /// BPFO characteristic order (bearing outer race fault order, dimensionless).
    pub bearing_bpfo_order: f64,
    /// BPFI characteristic order (bearing inner race fault order, dimensionless).
    pub bearing_bpfi_order: f64,
    /// Number of teeth on the monitored gear stage.
    pub gear_teeth_count: u32,
    /// Rated rotational speed of the shaft in RPM.
    pub rated_rpm: f64,
    /// Sample rate of the vibration sensor in Hz.
    pub sample_rate_hz: f64,
}

/// Bearing fault characteristic frequencies at a given shaft speed.
#[derive(Debug, Clone, PartialEq)]
pub struct BearingFaultFreqs {
    /// Ball Pass Frequency Outer race (Hz).
    pub bpfo: f64,
    /// Ball Pass Frequency Inner race (Hz).
    pub bpfi: f64,
    /// Ball Spin Frequency (Hz).
    pub bsf: f64,
    /// Fundamental Train Frequency / cage frequency (Hz).
    pub ftf: f64,
}

/// ISO 10816 vibration severity classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeverityLevel {
    /// Zone A — good condition, typical of new machines.
    Good,
    /// Zone B — acceptable for unrestricted long-term operation.
    Satisfactory,
    /// Zone C — tolerable only for limited periods.
    Unsatisfactory,
    /// Zone D — vibration severe enough to cause damage.
    Unacceptable,
}

/// Alarm threshold configuration for a monitored parameter.
#[derive(Debug, Clone)]
pub struct AlarmThreshold {
    /// Warning level (Zone B/C boundary equivalent).
    pub warning: f64,
    /// Alert level (Zone C/D boundary equivalent).
    pub alert: f64,
    /// Danger / trip level.
    pub danger: f64,
}

impl AlarmThreshold {
    /// Create a new alarm threshold set.
    pub fn new(warning: f64, alert: f64, danger: f64) -> Self {
        Self {
            warning,
            alert,
            danger,
        }
    }

    /// Evaluate a measured value against the thresholds.
    ///
    /// Returns the corresponding [`SeverityLevel`].
    pub fn evaluate(&self, value: f64) -> SeverityLevel {
        if value >= self.danger {
            SeverityLevel::Unacceptable
        } else if value >= self.alert {
            SeverityLevel::Unsatisfactory
        } else if value >= self.warning {
            SeverityLevel::Satisfactory
        } else {
            SeverityLevel::Good
        }
    }
}

/// Main turbine vibration monitor that aggregates diagnostics.
#[derive(Debug, Clone)]
pub struct TurbineMonitor {
    /// Turbine drivetrain configuration.
    pub config: TurbineConfig,
    /// RMS velocity alarm thresholds (mm/s).
    pub velocity_alarm: AlarmThreshold,
    /// Historical RMS velocity readings for trend analysis.
    history: Vec<f64>,
}

impl TurbineMonitor {
    /// Create a new monitor with the given configuration and default ISO 10816
    /// Class I thresholds.
    pub fn new(config: TurbineConfig) -> Self {
        Self {
            config,
            velocity_alarm: AlarmThreshold::new(1.8, 4.5, 11.2),
            history: Vec::new(),
        }
    }

    /// Set custom alarm thresholds for RMS velocity (mm/s).
    pub fn set_velocity_alarm(&mut self, threshold: AlarmThreshold) {
        self.velocity_alarm = threshold;
    }

    /// Record an RMS velocity measurement for trending.
    pub fn record_measurement(&mut self, rms_velocity_mms: f64) {
        self.history.push(rms_velocity_mms);
    }

    /// Return the measurement history.
    pub fn history(&self) -> &[f64] {
        &self.history
    }

    /// Evaluate the most recent measurement against alarm thresholds.
    ///
    /// Returns `None` if no measurements have been recorded.
    pub fn current_severity(&self) -> Option<SeverityLevel> {
        self.history.last().map(|v| self.velocity_alarm.evaluate(*v))
    }

    /// Predict future RMS velocity values using linear trend extrapolation.
    ///
    /// Returns an empty vector if fewer than 2 data points exist.
    pub fn predict_trend(&self, forecast_steps: usize) -> Vec<f64> {
        trend_predict(&self.history, forecast_steps)
    }

    /// Compute the gear mesh frequency at the current rated RPM.
    pub fn rated_gear_mesh_frequency(&self) -> f64 {
        gear_mesh_frequency(self.config.rated_rpm, self.config.gear_teeth_count)
    }
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Calculate bearing fault characteristic frequencies.
///
/// # Arguments
///
/// * `num_balls` — number of rolling elements
/// * `ball_diameter_mm` — rolling element diameter in mm
/// * `pitch_diameter_mm` — bearing pitch diameter in mm
/// * `contact_angle_deg` — contact angle in degrees
/// * `rpm` — shaft rotational speed in RPM
///
/// # Returns
///
/// A [`BearingFaultFreqs`] struct with BPFO, BPFI, BSF, and FTF in Hz.
pub fn bearing_fault_frequencies(
    num_balls: u8,
    ball_diameter_mm: f64,
    pitch_diameter_mm: f64,
    contact_angle_deg: f64,
    rpm: f64,
) -> BearingFaultFreqs {
    let n = num_balls as f64;
    let bd = ball_diameter_mm;
    let pd = pitch_diameter_mm;
    let alpha = contact_angle_deg * PI / 180.0;
    let fr = rpm / 60.0; // shaft frequency in Hz

    let ratio = bd / pd;
    let cos_alpha = alpha.cos();

    let ftf = 0.5 * fr * (1.0 - ratio * cos_alpha);
    let bpfo = 0.5 * n * fr * (1.0 - ratio * cos_alpha);
    let bpfi = 0.5 * n * fr * (1.0 + ratio * cos_alpha);
    let bsf = 0.5 * (pd / bd) * fr * (1.0 - (ratio * cos_alpha).powi(2));

    BearingFaultFreqs {
        bpfo,
        bpfi,
        bsf,
        ftf,
    }
}

/// Compute the envelope spectrum of a vibration signal.
///
/// Performs an analytic signal approximation (Hilbert-like transform via DFT),
/// extracts the envelope, removes the DC component, and returns the magnitude
/// spectrum of the envelope.
///
/// The returned vector has length `signal.len() / 2 + 1` (one-sided spectrum).
pub fn envelope_spectrum(signal: &[f64], sample_rate: f64) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }
    let _ = sample_rate; // used conceptually for frequency axis scaling

    // Step 1: DFT of the input signal (real-valued)
    let mut real = signal.to_vec();
    let mut imag = vec![0.0; n];
    dft_in_place(&mut real, &mut imag, false);

    // Step 2: Create analytic signal by zeroing negative frequencies
    // DC and Nyquist (if present) kept as-is; positive freqs doubled; negative zeroed.
    if n > 1 {
        let half = if n % 2 == 0 { n / 2 } else { (n + 1) / 2 };
        for i in 1..half {
            real[i] *= 2.0;
            imag[i] *= 2.0;
        }
        let start = if n % 2 == 0 { n / 2 + 1 } else { (n + 1) / 2 };
        for i in start..n {
            real[i] = 0.0;
            imag[i] = 0.0;
        }
    }

    // Step 3: IDFT to get analytic signal in time domain
    dft_in_place(&mut real, &mut imag, true);

    // Step 4: Envelope = magnitude of analytic signal
    let mut envelope: Vec<f64> = real
        .iter()
        .zip(imag.iter())
        .map(|(r, i)| (r * r + i * i).sqrt())
        .collect();

    // Step 5: Remove DC (mean) from envelope
    let mean = envelope.iter().sum::<f64>() / n as f64;
    for v in &mut envelope {
        *v -= mean;
    }

    // Step 6: DFT of envelope and return magnitude (one-sided)
    let mut env_imag = vec![0.0; n];
    dft_in_place(&mut envelope, &mut env_imag, false);

    let out_len = n / 2 + 1;
    (0..out_len)
        .map(|i| {
            let mag = (envelope[i] * envelope[i] + env_imag[i] * env_imag[i]).sqrt() / n as f64;
            mag
        })
        .collect()
}

/// Compute order-domain spectrum from a vibration signal with varying RPM.
///
/// Resamples the time-domain signal to constant angular increments based on the
/// RPM profile, then computes the magnitude spectrum in the order domain.
///
/// # Arguments
///
/// * `signal` — vibration signal samples
/// * `rpm_profile` — instantaneous RPM for each sample (same length as signal)
/// * `sample_rate` — sampling rate in Hz
/// * `orders` — number of orders to return in the spectrum
///
/// # Returns
///
/// A vector of `orders` magnitude values representing orders 0..(orders-1).
pub fn order_track(
    signal: &[f64],
    rpm_profile: &[f64],
    sample_rate: f64,
    orders: usize,
) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || rpm_profile.is_empty() || orders == 0 {
        return vec![0.0; orders];
    }

    let len = n.min(rpm_profile.len());

    // Step 1: Compute cumulative angle (in revolutions) from RPM profile
    let dt = 1.0 / sample_rate;
    let mut cum_revs = vec![0.0; len];
    for i in 1..len {
        let rpm_avg = (rpm_profile[i - 1] + rpm_profile[i]) / 2.0;
        let revs_per_sec = rpm_avg / 60.0;
        cum_revs[i] = cum_revs[i - 1] + revs_per_sec * dt;
    }

    let total_revs = cum_revs[len - 1];
    if total_revs <= 0.0 {
        return vec![0.0; orders];
    }

    // Step 2: Resample to uniform angular spacing
    // Use at least 2 * orders samples per revolution for adequate resolution
    let samples_per_rev = (2 * orders).max(64);
    let total_resampled = (total_revs * samples_per_rev as f64).ceil() as usize;
    if total_resampled < 2 {
        return vec![0.0; orders];
    }

    let mut resampled = Vec::with_capacity(total_resampled);
    let rev_step = total_revs / total_resampled as f64;
    let mut src_idx = 0usize;

    for k in 0..total_resampled {
        let target_rev = k as f64 * rev_step;
        // Find bracketing source samples
        while src_idx + 1 < len && cum_revs[src_idx + 1] < target_rev {
            src_idx += 1;
        }
        if src_idx + 1 >= len {
            resampled.push(signal[len - 1]);
        } else {
            let span = cum_revs[src_idx + 1] - cum_revs[src_idx];
            if span.abs() < 1e-15 {
                resampled.push(signal[src_idx]);
            } else {
                let frac = (target_rev - cum_revs[src_idx]) / span;
                let val =
                    signal[src_idx] * (1.0 - frac) + signal[(src_idx + 1).min(len - 1)] * frac;
                resampled.push(val);
            }
        }
    }

    // Step 3: DFT of resampled signal
    let m = resampled.len();
    let mut re = resampled;
    let mut im = vec![0.0; m];
    dft_in_place(&mut re, &mut im, false);

    // Step 4: Extract order magnitudes
    // The resampled signal has total_resampled samples spanning total_revs revolutions.
    // DFT bin j represents frequency j cycles per total_resampled samples.
    // Since total_resampled samples span total_revs revolutions,
    // bin j corresponds to j / total_revs orders.
    // So for order k, the bin is k * total_revs.
    let mut result = Vec::with_capacity(orders);
    for k in 0..orders {
        let bin_f = k as f64 * total_revs;
        let bin_lo = bin_f.floor() as usize;
        let bin_hi = bin_lo + 1;
        let frac = bin_f - bin_lo as f64;

        let mag = |idx: usize| -> f64 {
            if idx < m {
                (re[idx] * re[idx] + im[idx] * im[idx]).sqrt() / m as f64
            } else {
                0.0
            }
        };

        let val = mag(bin_lo) * (1.0 - frac) + mag(bin_hi) * frac;
        result.push(val);
    }

    result
}

/// Compute the gear mesh frequency (GMF) in Hz.
///
/// GMF = (RPM / 60) * number_of_teeth
pub fn gear_mesh_frequency(rpm: f64, teeth: u32) -> f64 {
    (rpm / 60.0) * teeth as f64
}

/// Compute the crest factor (peak-to-RMS ratio) of a signal.
///
/// Crest factor = peak absolute value / RMS.
/// Returns 0.0 for empty or all-zero signals.
pub fn crest_factor(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let rms = rms_value(signal);
    if rms == 0.0 {
        return 0.0;
    }
    let peak = signal
        .iter()
        .map(|x| x.abs())
        .fold(0.0_f64, |a, b| a.max(b));
    peak / rms
}

/// Compute the kurtosis (4th standardized moment) of a signal.
///
/// Uses the standard definition: kurtosis = E[(x - mean)^4] / (stddev^4).
/// A Gaussian signal has kurtosis of approximately 3. Returns 0.0 for signals with zero variance.
pub fn kurtosis(signal: &[f64]) -> f64 {
    let n = signal.len();
    if n < 2 {
        return 0.0;
    }
    let mean = signal.iter().sum::<f64>() / n as f64;
    let m2 = signal.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
    if m2 == 0.0 {
        return 0.0;
    }
    let m4 = signal.iter().map(|x| (x - mean).powi(4)).sum::<f64>() / n as f64;
    m4 / (m2 * m2)
}

/// Compute RMS velocity from an acceleration signal.
///
/// Integrates the acceleration signal (assumed in m/s squared) in the frequency domain
/// (dividing by j*omega), then computes RMS of the resulting velocity. The result is
/// returned in mm/s.
///
/// DC component is discarded during integration.
pub fn rms_velocity(signal: &[f64], sample_rate: f64) -> f64 {
    let n = signal.len();
    if n < 2 || sample_rate <= 0.0 {
        return 0.0;
    }

    // DFT of acceleration signal
    let mut re = signal.to_vec();
    let mut im = vec![0.0; n];
    dft_in_place(&mut re, &mut im, false);

    // Integrate in frequency domain: V(f) = A(f) / (j * 2*pi*f)
    let freq_resolution = sample_rate / n as f64;
    let re_orig = re.clone();
    let im_orig = im.clone();

    // DC bin (k=0): set to zero (no DC in velocity)
    re[0] = 0.0;
    im[0] = 0.0;

    for k in 1..n {
        let freq_k = if k <= n / 2 {
            k as f64 * freq_resolution
        } else {
            -((n - k) as f64) * freq_resolution
        };
        let omega = 2.0 * PI * freq_k;
        if omega.abs() < 1e-12 {
            re[k] = 0.0;
            im[k] = 0.0;
        } else {
            // Divide by j*omega: (a + jb) / (j*omega) = b/omega - j*a/omega
            re[k] = im_orig[k] / omega;
            im[k] = -re_orig[k] / omega;
        }
    }

    // IDFT to get velocity in time domain
    dft_in_place(&mut re, &mut im, true);

    // RMS of velocity, convert m/s to mm/s (* 1000)
    let rms = rms_value(&re);
    rms * 1000.0
}

/// Classify vibration severity per ISO 10816.
///
/// # Arguments
///
/// * `rms_velocity_mms` — RMS vibration velocity in mm/s
/// * `machine_class` — ISO 10816 machine class (1, 2, 3, or 4)
///
/// # Machine Classes
///
/// * Class 1: Small machines up to 15 kW
/// * Class 2: Medium machines 15-75 kW or up to 300 kW on special foundations
/// * Class 3: Large machines on rigid foundations (>300 kW)
/// * Class 4: Large machines on soft foundations (turbo-sets)
///
/// # Returns
///
/// The [`SeverityLevel`] zone for the given velocity and machine class.
pub fn classify_severity_iso10816(rms_velocity_mms: f64, machine_class: u8) -> SeverityLevel {
    // ISO 10816 velocity thresholds in mm/s [A/B boundary, B/C boundary, C/D boundary]
    let (ab, bc, cd) = match machine_class {
        1 => (0.71, 1.8, 4.5),
        2 => (1.12, 2.8, 7.1),
        3 => (1.8, 4.5, 11.2),
        4 => (2.8, 7.1, 18.0),
        _ => (2.8, 7.1, 18.0),
    };

    if rms_velocity_mms <= ab {
        SeverityLevel::Good
    } else if rms_velocity_mms <= bc {
        SeverityLevel::Satisfactory
    } else if rms_velocity_mms <= cd {
        SeverityLevel::Unsatisfactory
    } else {
        SeverityLevel::Unacceptable
    }
}

/// Predict future values by linear extrapolation (least-squares fit).
///
/// Fits a line y = a + b*x to the history data (x = 0, 1, 2, ...)
/// and extrapolates `forecast_steps` beyond the last data point.
///
/// Returns an empty vector if fewer than 2 data points or `forecast_steps` is 0.
pub fn trend_predict(history: &[f64], forecast_steps: usize) -> Vec<f64> {
    let n = history.len();
    if n < 2 || forecast_steps == 0 {
        return Vec::new();
    }

    // Least-squares linear regression: y = a + b*x
    let n_f = n as f64;
    let sum_x: f64 = (0..n).map(|i| i as f64).sum();
    let sum_y: f64 = history.iter().sum();
    let sum_xy: f64 = history.iter().enumerate().map(|(i, y)| i as f64 * y).sum();
    let sum_x2: f64 = (0..n).map(|i| (i as f64).powi(2)).sum();

    let denom = n_f * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-15 {
        // Degenerate case — return flat forecast
        let last = *history.last().unwrap();
        return vec![last; forecast_steps];
    }

    let b = (n_f * sum_xy - sum_x * sum_y) / denom;
    let a = (sum_y - b * sum_x) / n_f;

    (0..forecast_steps)
        .map(|step| {
            let x = (n + step) as f64;
            a + b * x
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// RMS value of a signal.
fn rms_value(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = signal.iter().map(|x| x * x).sum();
    (sum_sq / signal.len() as f64).sqrt()
}

/// In-place DFT / IDFT (naive O(N^2) implementation for correctness).
///
/// When `inverse` is true, performs the inverse transform and divides by N.
fn dft_in_place(real: &mut [f64], imag: &mut [f64], inverse: bool) {
    let n = real.len();
    if n <= 1 {
        return;
    }

    let sign = if inverse { 1.0 } else { -1.0 };
    let re_in = real.to_vec();
    let im_in = imag.to_vec();

    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for j in 0..n {
            let angle = sign * 2.0 * PI * (k as f64) * (j as f64) / (n as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            sum_re += re_in[j] * cos_a - im_in[j] * sin_a;
            sum_im += re_in[j] * sin_a + im_in[j] * cos_a;
        }
        if inverse {
            real[k] = sum_re / n as f64;
            imag[k] = sum_im / n as f64;
        } else {
            real[k] = sum_re;
            imag[k] = sum_im;
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- Bearing fault frequencies ---

    #[test]
    fn test_bearing_fault_frequencies_basic() {
        // 9-ball bearing, zero contact angle, 1800 RPM
        let f = bearing_fault_frequencies(9, 25.4, 127.0, 0.0, 1800.0);
        let fr = 1800.0 / 60.0; // 30 Hz

        let ratio = 25.4 / 127.0;
        let expected_bpfo = 0.5 * 9.0 * fr * (1.0 - ratio);
        let expected_bpfi = 0.5 * 9.0 * fr * (1.0 + ratio);
        let expected_ftf = 0.5 * fr * (1.0 - ratio);
        let expected_bsf = 0.5 * (127.0 / 25.4) * fr * (1.0 - ratio * ratio);

        assert!((f.bpfo - expected_bpfo).abs() < EPSILON);
        assert!((f.bpfi - expected_bpfi).abs() < EPSILON);
        assert!((f.bsf - expected_bsf).abs() < EPSILON);
        assert!((f.ftf - expected_ftf).abs() < EPSILON);
    }

    #[test]
    fn test_bearing_fault_frequencies_with_contact_angle() {
        let f = bearing_fault_frequencies(12, 20.0, 100.0, 15.0, 3600.0);
        assert!(f.bpfo > 0.0);
        assert!(f.bpfi > f.bpfo); // BPFI > BPFO always
        assert!(f.bsf > 0.0);
        assert!(f.ftf > 0.0);
    }

    #[test]
    fn test_bearing_fault_frequencies_zero_rpm() {
        let f = bearing_fault_frequencies(9, 25.4, 127.0, 0.0, 0.0);
        assert_eq!(f.bpfo, 0.0);
        assert_eq!(f.bpfi, 0.0);
        assert_eq!(f.bsf, 0.0);
        assert_eq!(f.ftf, 0.0);
    }

    #[test]
    fn test_bpfi_always_greater_than_bpfo() {
        // For any physical bearing, BPFI > BPFO when contact_angle < 90 degrees
        for balls in [5, 8, 12, 16] {
            for angle in [0.0, 10.0, 20.0, 30.0] {
                let f = bearing_fault_frequencies(balls, 20.0, 100.0, angle, 1000.0);
                assert!(
                    f.bpfi > f.bpfo,
                    "BPFI should exceed BPFO for {} balls, {} deg",
                    balls,
                    angle
                );
            }
        }
    }

    // --- Gear mesh frequency ---

    #[test]
    fn test_gear_mesh_frequency() {
        let gmf = gear_mesh_frequency(1500.0, 97);
        // 1500/60 * 97 = 25 * 97 = 2425 Hz
        assert!((gmf - 2425.0).abs() < EPSILON);
    }

    #[test]
    fn test_gear_mesh_frequency_zero_rpm() {
        assert_eq!(gear_mesh_frequency(0.0, 50), 0.0);
    }

    // --- Crest factor ---

    #[test]
    fn test_crest_factor_sine() {
        // Crest factor of a sine wave is sqrt(2) ~ 1.4142
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let cf = crest_factor(&signal);
        assert!(
            (cf - std::f64::consts::SQRT_2).abs() < 0.02,
            "Sine crest factor should be ~sqrt(2), got {}",
            cf
        );
    }

    #[test]
    fn test_crest_factor_empty() {
        assert_eq!(crest_factor(&[]), 0.0);
    }

    #[test]
    fn test_crest_factor_dc() {
        // Constant signal: crest factor = 1.0
        let signal = vec![5.0; 100];
        let cf = crest_factor(&signal);
        assert!((cf - 1.0).abs() < EPSILON);
    }

    // --- Kurtosis ---

    #[test]
    fn test_kurtosis_uniform() {
        // Kurtosis of uniform distribution = 9/5 = 1.8
        let n = 10000;
        let signal: Vec<f64> = (0..n).map(|i| (i as f64 / n as f64) * 2.0 - 1.0).collect();
        let k = kurtosis(&signal);
        assert!(
            (k - 1.8).abs() < 0.05,
            "Uniform kurtosis should be ~1.8, got {}",
            k
        );
    }

    #[test]
    fn test_kurtosis_impulsive() {
        // Signal with a large spike should have high kurtosis
        let mut signal = vec![0.0; 1000];
        signal[500] = 100.0;
        let k = kurtosis(&signal);
        assert!(
            k > 100.0,
            "Impulsive signal should have very high kurtosis, got {}",
            k
        );
    }

    #[test]
    fn test_kurtosis_empty() {
        assert_eq!(kurtosis(&[]), 0.0);
    }

    // --- ISO 10816 severity ---

    #[test]
    fn test_classify_severity_class1() {
        assert_eq!(classify_severity_iso10816(0.5, 1), SeverityLevel::Good);
        assert_eq!(
            classify_severity_iso10816(1.0, 1),
            SeverityLevel::Satisfactory
        );
        assert_eq!(
            classify_severity_iso10816(3.0, 1),
            SeverityLevel::Unsatisfactory
        );
        assert_eq!(
            classify_severity_iso10816(5.0, 1),
            SeverityLevel::Unacceptable
        );
    }

    #[test]
    fn test_classify_severity_class3() {
        assert_eq!(classify_severity_iso10816(1.0, 3), SeverityLevel::Good);
        assert_eq!(
            classify_severity_iso10816(3.0, 3),
            SeverityLevel::Satisfactory
        );
        assert_eq!(
            classify_severity_iso10816(8.0, 3),
            SeverityLevel::Unsatisfactory
        );
        assert_eq!(
            classify_severity_iso10816(15.0, 3),
            SeverityLevel::Unacceptable
        );
    }

    #[test]
    fn test_classify_severity_boundary_values() {
        // Exactly at boundary should be the lower zone (<=)
        assert_eq!(
            classify_severity_iso10816(0.71, 1),
            SeverityLevel::Good
        );
        assert_eq!(
            classify_severity_iso10816(1.8, 1),
            SeverityLevel::Satisfactory
        );
        assert_eq!(
            classify_severity_iso10816(4.5, 1),
            SeverityLevel::Unsatisfactory
        );
    }

    // --- Trend prediction ---

    #[test]
    fn test_trend_predict_linear() {
        // Perfect linear data: 1, 2, 3, 4, 5 -> next should be 6, 7, 8
        let history = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let forecast = trend_predict(&history, 3);
        assert_eq!(forecast.len(), 3);
        assert!((forecast[0] - 6.0).abs() < EPSILON);
        assert!((forecast[1] - 7.0).abs() < EPSILON);
        assert!((forecast[2] - 8.0).abs() < EPSILON);
    }

    #[test]
    fn test_trend_predict_constant() {
        let history = vec![5.0, 5.0, 5.0, 5.0];
        let forecast = trend_predict(&history, 2);
        assert!((forecast[0] - 5.0).abs() < EPSILON);
        assert!((forecast[1] - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_trend_predict_insufficient_data() {
        assert!(trend_predict(&[1.0], 5).is_empty());
        assert!(trend_predict(&[], 5).is_empty());
    }

    #[test]
    fn test_trend_predict_zero_steps() {
        assert!(trend_predict(&[1.0, 2.0, 3.0], 0).is_empty());
    }

    // --- Envelope spectrum ---

    #[test]
    fn test_envelope_spectrum_length() {
        let signal = vec![1.0; 64];
        let spec = envelope_spectrum(&signal, 1000.0);
        assert_eq!(spec.len(), 33); // 64/2 + 1
    }

    #[test]
    fn test_envelope_spectrum_empty() {
        let spec = envelope_spectrum(&[], 1000.0);
        assert!(spec.is_empty());
    }

    // --- Order tracking ---

    #[test]
    fn test_order_track_basic() {
        // Constant RPM with a known frequency component
        let n = 256;
        let rpm = 600.0; // 10 Hz shaft
        let sample_rate = 1000.0;
        let rpm_profile = vec![rpm; n];

        // Generate a signal at 2nd order (20 Hz)
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 20.0 * i as f64 / sample_rate).sin())
            .collect();

        let orders = order_track(&signal, &rpm_profile, sample_rate, 8);
        assert_eq!(orders.len(), 8);
        // Order 2 should have the dominant amplitude
        let max_idx = orders
            .iter()
            .enumerate()
            .skip(1) // skip DC
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_idx, 2, "Expected peak at order 2, found at {}", max_idx);
    }

    #[test]
    fn test_order_track_empty() {
        let result = order_track(&[], &[], 1000.0, 5);
        assert_eq!(result.len(), 5);
    }

    // --- RMS velocity ---

    #[test]
    fn test_rms_velocity_pure_tone() {
        // A sine acceleration at frequency f has velocity amplitude = accel / (2*pi*f)
        let sample_rate = 4000.0;
        let n = 4000; // 1 second
        let f = 100.0; // 100 Hz
        let accel_amplitude = 10.0; // m/s^2

        let signal: Vec<f64> = (0..n)
            .map(|i| accel_amplitude * (2.0 * PI * f * i as f64 / sample_rate).sin())
            .collect();

        let v = rms_velocity(&signal, sample_rate);
        // Expected velocity amplitude = accel_amplitude / (2*pi*f)
        // RMS of sine = amplitude / sqrt(2)
        // Convert m/s to mm/s (* 1000)
        let expected = accel_amplitude / (2.0 * PI * f) / std::f64::consts::SQRT_2 * 1000.0;
        assert!(
            (v - expected).abs() / expected < 0.05,
            "RMS velocity expected ~{:.2} mm/s, got {:.2} mm/s",
            expected,
            v
        );
    }

    // --- TurbineMonitor ---

    #[test]
    fn test_turbine_monitor_workflow() {
        let config = TurbineConfig {
            bearing_bpfo_order: 5.43,
            bearing_bpfi_order: 7.57,
            gear_teeth_count: 97,
            rated_rpm: 1500.0,
            sample_rate_hz: 25600.0,
        };

        let mut monitor = TurbineMonitor::new(config);

        // Record some measurements showing degradation
        monitor.record_measurement(0.5);
        monitor.record_measurement(1.0);
        monitor.record_measurement(1.5);
        monitor.record_measurement(2.0);

        assert_eq!(monitor.history().len(), 4);
        assert_eq!(monitor.current_severity(), Some(SeverityLevel::Satisfactory));

        let forecast = monitor.predict_trend(3);
        assert_eq!(forecast.len(), 3);
        // Trend should be increasing
        assert!(forecast[0] > 2.0);
        assert!(forecast[2] > forecast[0]);
    }

    #[test]
    fn test_turbine_monitor_rated_gmf() {
        let config = TurbineConfig {
            bearing_bpfo_order: 5.0,
            bearing_bpfi_order: 7.0,
            gear_teeth_count: 97,
            rated_rpm: 1500.0,
            sample_rate_hz: 25600.0,
        };
        let monitor = TurbineMonitor::new(config);
        assert!((monitor.rated_gear_mesh_frequency() - 2425.0).abs() < EPSILON);
    }

    // --- Alarm threshold ---

    #[test]
    fn test_alarm_threshold_evaluate() {
        let thresh = AlarmThreshold::new(2.0, 5.0, 10.0);
        assert_eq!(thresh.evaluate(1.0), SeverityLevel::Good);
        assert_eq!(thresh.evaluate(3.0), SeverityLevel::Satisfactory);
        assert_eq!(thresh.evaluate(7.0), SeverityLevel::Unsatisfactory);
        assert_eq!(thresh.evaluate(15.0), SeverityLevel::Unacceptable);
    }

    #[test]
    fn test_custom_alarm_thresholds() {
        let config = TurbineConfig {
            bearing_bpfo_order: 5.0,
            bearing_bpfi_order: 7.0,
            gear_teeth_count: 50,
            rated_rpm: 1800.0,
            sample_rate_hz: 10000.0,
        };
        let mut monitor = TurbineMonitor::new(config);
        monitor.set_velocity_alarm(AlarmThreshold::new(1.0, 3.0, 8.0));
        monitor.record_measurement(2.0);
        assert_eq!(monitor.current_severity(), Some(SeverityLevel::Satisfactory));
    }

    // --- DFT roundtrip ---

    #[test]
    fn test_dft_roundtrip() {
        let original = vec![1.0, 2.0, 3.0, 4.0];
        let mut re = original.clone();
        let mut im = vec![0.0; 4];

        dft_in_place(&mut re, &mut im, false);
        dft_in_place(&mut re, &mut im, true);

        for (i, &val) in original.iter().enumerate() {
            assert!(
                (re[i] - val).abs() < EPSILON,
                "DFT roundtrip failed at index {}",
                i
            );
            assert!(im[i].abs() < EPSILON);
        }
    }
}
