//! Medical Doppler Ultrasound Signal Processor
//!
//! Implements signal processing algorithms for continuous-wave (CW) and
//! pulsed-wave (PW) Doppler ultrasound blood flow velocity measurement.
//! Processes baseband IQ data to compute flow velocities, vascular indices,
//! and spectrograms.
//!
//! ## Doppler Equation
//!
//! The Doppler frequency shift for a moving scatterer (e.g., red blood cells):
//!
//! ```text
//!         fd * c
//! v = ─────────────────
//!     2 * f0 * cos(θ)
//! ```
//!
//! Where:
//! - `fd` = Doppler frequency shift (Hz)
//! - `c` = speed of sound in tissue (typically 1540 m/s)
//! - `f0` = transmit frequency (Hz)
//! - `θ` = beam-to-flow angle (degrees)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::doppler_ultrasound_processor::{
//!     DopplerConfig, DopplerProcessor, doppler_shift_to_velocity,
//! };
//!
//! // Configure a 5 MHz CW Doppler probe
//! let config = DopplerConfig {
//!     transmit_freq_mhz: 5.0,
//!     prf_hz: 8000.0,
//!     sample_rate_hz: 16000.0,
//!     speed_of_sound_mps: 1540.0,
//!     beam_angle_deg: 60.0,
//! };
//!
//! let processor = DopplerProcessor::new(config);
//!
//! // Convert a 1 kHz Doppler shift at 5 MHz, 60-degree angle
//! let v = doppler_shift_to_velocity(1000.0, 5.0e6, 60.0, 1540.0);
//! assert!((v - 0.308).abs() < 0.01); // ~0.308 m/s
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for Doppler ultrasound signal processing.
#[derive(Debug, Clone, PartialEq)]
pub struct DopplerConfig {
    /// Transmit (carrier) frequency in MHz.
    pub transmit_freq_mhz: f64,
    /// Pulse repetition frequency in Hz (determines Nyquist velocity for PW).
    pub prf_hz: f64,
    /// Baseband sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Speed of sound in tissue in m/s (default: 1540.0 for soft tissue).
    pub speed_of_sound_mps: f64,
    /// Beam-to-flow angle in degrees (default: 60.0).
    pub beam_angle_deg: f64,
}

impl Default for DopplerConfig {
    fn default() -> Self {
        Self {
            transmit_freq_mhz: 5.0,
            prf_hz: 8000.0,
            sample_rate_hz: 16000.0,
            speed_of_sound_mps: 1540.0,
            beam_angle_deg: 60.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Result
// ---------------------------------------------------------------------------

/// Result of Doppler ultrasound frame processing.
#[derive(Debug, Clone)]
pub struct DopplerResult {
    /// Mean flow velocity in m/s (signed: positive = toward probe).
    pub mean_velocity_mps: f64,
    /// Peak systolic velocity in m/s.
    pub peak_velocity_mps: f64,
    /// Pulsatility index: PI = (Vs - Vd) / Vm.
    pub pulsatility_index: f64,
    /// Resistive index: RI = (Vs - Vd) / Vs.
    pub resistive_index: f64,
    /// Power spectrum (linear scale) of the current frame.
    pub spectrum: Vec<f64>,
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

/// Main Doppler ultrasound signal processor.
///
/// Processes IQ frames from the Doppler demodulator, applies wall filtering
/// to remove clutter from stationary tissue, computes the power spectrum,
/// and derives flow velocity estimates.
pub struct DopplerProcessor {
    config: DopplerConfig,
}

impl DopplerProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: DopplerConfig) -> Self {
        Self { config }
    }

    /// Process a single IQ frame and return Doppler flow results.
    ///
    /// Steps:
    /// 1. Apply wall (high-pass) filter to remove tissue clutter
    /// 2. Compute power spectrum via FFT
    /// 3. Estimate mean and peak Doppler frequencies
    /// 4. Convert to velocities using the Doppler equation
    /// 5. Compute vascular indices (PI, RI)
    pub fn process_frame(&self, iq_data: &[(f64, f64)]) -> DopplerResult {
        if iq_data.is_empty() {
            return DopplerResult {
                mean_velocity_mps: 0.0,
                peak_velocity_mps: 0.0,
                pulsatility_index: 0.0,
                resistive_index: 0.0,
                spectrum: Vec::new(),
            };
        }

        // Wall filter: remove clutter below ~50 Hz (configurable fraction of fs)
        let wall_cutoff = 50.0_f64.min(self.config.sample_rate_hz * 0.02);
        let filtered = wall_filter(iq_data, wall_cutoff, self.config.sample_rate_hz);

        // Power spectrum
        let n = filtered.len();
        let spectrum = compute_power_spectrum(&filtered, n);

        // Frequency resolution
        let df = self.config.sample_rate_hz / n as f64;

        // Mean Doppler frequency via first-moment estimator
        let mean_fd = mean_frequency_estimator_with_df(&spectrum, df, self.config.sample_rate_hz);

        // Peak Doppler frequency: bin with max power (excluding DC)
        let peak_fd = peak_frequency(&spectrum, df, self.config.sample_rate_hz);

        let f0_hz = self.config.transmit_freq_mhz * 1.0e6;
        let c = self.config.speed_of_sound_mps;
        let angle = self.config.beam_angle_deg;

        let mean_v = doppler_shift_to_velocity(mean_fd, f0_hz, angle, c);
        let peak_v = doppler_shift_to_velocity(peak_fd, f0_hz, angle, c);

        // For single-frame PI/RI, use peak as systolic and a fraction as
        // diastolic estimate. In a real system these would come from envelope
        // analysis over multiple cardiac cycles.
        let v_systolic = peak_v.abs();
        let v_diastolic = mean_v.abs() * 0.3; // simplified estimate
        let v_mean = mean_v.abs();

        let pi = if v_mean.abs() > 1e-12 {
            pulsatility_index(v_systolic, v_diastolic, v_mean)
        } else {
            0.0
        };
        let ri = if v_systolic.abs() > 1e-12 {
            resistive_index(v_systolic, v_diastolic)
        } else {
            0.0
        };

        DopplerResult {
            mean_velocity_mps: mean_v,
            peak_velocity_mps: peak_v,
            pulsatility_index: pi,
            resistive_index: ri,
            spectrum,
        }
    }
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Convert a Doppler frequency shift to flow velocity.
///
/// ```text
///         fd * c
/// v = ─────────────────
///     2 * f0 * cos(θ)
/// ```
///
/// # Arguments
/// * `fd_hz` - Doppler frequency shift in Hz
/// * `f0_hz` - Transmit (carrier) frequency in Hz
/// * `angle_deg` - Beam-to-flow angle in degrees
/// * `c` - Speed of sound in m/s
pub fn doppler_shift_to_velocity(fd_hz: f64, f0_hz: f64, angle_deg: f64, c: f64) -> f64 {
    let theta = angle_deg * PI / 180.0;
    let cos_theta = theta.cos();
    if cos_theta.abs() < 1e-15 || f0_hz.abs() < 1e-15 {
        return 0.0;
    }
    (fd_hz * c) / (2.0 * f0_hz * cos_theta)
}

/// Convert a flow velocity to the expected Doppler frequency shift.
///
/// ```text
/// fd = 2 * v * f0 * cos(θ) / c
/// ```
///
/// # Arguments
/// * `v_mps` - Flow velocity in m/s
/// * `f0_hz` - Transmit (carrier) frequency in Hz
/// * `angle_deg` - Beam-to-flow angle in degrees
/// * `c` - Speed of sound in m/s
pub fn velocity_to_doppler_shift(v_mps: f64, f0_hz: f64, angle_deg: f64, c: f64) -> f64 {
    let theta = angle_deg * PI / 180.0;
    if c.abs() < 1e-15 {
        return 0.0;
    }
    (2.0 * v_mps * f0_hz * theta.cos()) / c
}

/// High-pass wall (clutter) filter for Doppler IQ data.
///
/// Removes low-frequency components caused by slowly moving or stationary
/// tissue (vessel walls, probe motion). Implements a simple first-order
/// IIR high-pass filter.
///
/// # Arguments
/// * `iq_data` - Input IQ samples as (I, Q) tuples
/// * `cutoff_hz` - High-pass cutoff frequency in Hz
/// * `fs` - Sample rate in Hz
pub fn wall_filter(iq_data: &[(f64, f64)], cutoff_hz: f64, fs: f64) -> Vec<(f64, f64)> {
    if iq_data.is_empty() || fs <= 0.0 {
        return Vec::new();
    }

    // First-order IIR high-pass: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
    // alpha = RC / (RC + dt), RC = 1 / (2*pi*fc)
    let rc = 1.0 / (2.0 * PI * cutoff_hz.max(0.1));
    let dt = 1.0 / fs;
    let alpha = rc / (rc + dt);

    let mut out = Vec::with_capacity(iq_data.len());
    let mut prev_x = iq_data[0];
    let mut prev_y = (0.0_f64, 0.0_f64);

    out.push((0.0, 0.0)); // first sample zeroed by HP filter

    for i in 1..iq_data.len() {
        let yi = alpha * (prev_y.0 + iq_data[i].0 - prev_x.0);
        let yq = alpha * (prev_y.1 + iq_data[i].1 - prev_x.1);
        prev_x = iq_data[i];
        prev_y = (yi, yq);
        out.push((yi, yq));
    }

    out
}

/// Compute a short-time FFT spectrogram from IQ data.
///
/// Segments the data into overlapping windows, applies a Hann window, and
/// computes the magnitude-squared spectrum for each segment.
///
/// # Arguments
/// * `iq_data` - Input IQ samples
/// * `fft_size` - Number of samples per FFT window
/// * `overlap` - Number of overlapping samples between consecutive windows
///
/// # Returns
/// A `Vec<Vec<f64>>` where each inner vector is the power spectrum of one
/// time segment. Frequency bins run from -fs/2 to +fs/2 (fftshifted).
pub fn compute_spectrogram(
    iq_data: &[(f64, f64)],
    fft_size: usize,
    overlap: usize,
) -> Vec<Vec<f64>> {
    if iq_data.is_empty() || fft_size == 0 {
        return Vec::new();
    }

    let hop = if fft_size > overlap {
        fft_size - overlap
    } else {
        1
    };

    let mut spectrogram = Vec::new();
    let mut start = 0;

    while start + fft_size <= iq_data.len() {
        let segment = &iq_data[start..start + fft_size];

        // Apply Hann window
        let windowed: Vec<(f64, f64)> = segment
            .iter()
            .enumerate()
            .map(|(k, &(i, q))| {
                let w = 0.5 * (1.0 - (2.0 * PI * k as f64 / fft_size as f64).cos());
                (i * w, q * w)
            })
            .collect();

        let spectrum = compute_power_spectrum(&windowed, fft_size);
        spectrogram.push(spectrum);

        start += hop;
    }

    spectrogram
}

/// Estimate the mean Doppler frequency from a power spectrum using the
/// first spectral moment.
///
/// ```text
///        Σ f_k * P(f_k)
/// f_m = ─────────────────
///          Σ P(f_k)
/// ```
///
/// The returned value is a normalised frequency in the range [0, 1) where
/// 1 corresponds to the sample rate. Multiply by `fs` to get Hz.
pub fn mean_frequency_estimator(spectrum: &[f64]) -> f64 {
    if spectrum.is_empty() {
        return 0.0;
    }

    let n = spectrum.len() as f64;
    let total_power: f64 = spectrum.iter().sum();

    if total_power.abs() < 1e-30 {
        return 0.0;
    }

    let weighted_sum: f64 = spectrum
        .iter()
        .enumerate()
        .map(|(k, &p)| {
            // Map bin index to signed frequency: k < N/2 positive, k >= N/2 negative
            let f_norm = if k as f64 <= n / 2.0 {
                k as f64 / n
            } else {
                (k as f64 - n) / n
            };
            f_norm * p
        })
        .sum();

    weighted_sum / total_power
}

/// Estimate mean velocity using the Kasai autocorrelation method.
///
/// The Kasai estimator computes the phase shift between consecutive IQ
/// samples using complex autocorrelation at lag 1:
///
/// ```text
///            PRF          ⎛    Im(R(1)) ⎞
/// v = ─────────────── · arctan⎜ ───────── ⎟
///     2π · (2f0/c) · cos(θ)  ⎝    Re(R(1)) ⎠
/// ```
///
/// where R(1) = Σ z[n]·conj(z[n-1]).
///
/// # Arguments
/// * `iq_data` - Input IQ samples
/// * `prf_hz` - Pulse repetition frequency in Hz
/// * `f0_hz` - Transmit frequency in Hz
/// * `c` - Speed of sound in m/s
/// * `angle_deg` - Beam-to-flow angle in degrees
pub fn autocorrelation_velocity(
    iq_data: &[(f64, f64)],
    prf_hz: f64,
    f0_hz: f64,
    c: f64,
    angle_deg: f64,
) -> f64 {
    if iq_data.len() < 2 {
        return 0.0;
    }

    // Complex autocorrelation at lag 1: R(1) = Σ z[n] * conj(z[n-1])
    let mut re_sum = 0.0;
    let mut im_sum = 0.0;

    for i in 1..iq_data.len() {
        let (i1, q1) = iq_data[i];
        let (i0, q0) = iq_data[i - 1];
        // z[n] * conj(z[n-1]) = (i1+jq1)(i0-jq0)
        re_sum += i1 * i0 + q1 * q0;
        im_sum += q1 * i0 - i1 * q0;
    }

    let phase = im_sum.atan2(re_sum);
    let theta = angle_deg * PI / 180.0;
    let cos_theta = theta.cos();

    if cos_theta.abs() < 1e-15 || f0_hz.abs() < 1e-15 {
        return 0.0;
    }

    // v = (phase * PRF * c) / (4 * pi * f0 * cos(theta))
    (phase * prf_hz * c) / (4.0 * PI * f0_hz * cos_theta)
}

/// Compute the pulsatility index (Gosling index).
///
/// ```text
/// PI = (V_systolic - V_diastolic) / V_mean
/// ```
///
/// Higher PI indicates greater vascular resistance.
pub fn pulsatility_index(v_systolic: f64, v_diastolic: f64, v_mean: f64) -> f64 {
    if v_mean.abs() < 1e-15 {
        return 0.0;
    }
    (v_systolic - v_diastolic) / v_mean
}

/// Compute the resistive index (Pourcelot index).
///
/// ```text
/// RI = (V_systolic - V_diastolic) / V_systolic
/// ```
///
/// Normal RI for renal artery: 0.5-0.7.
pub fn resistive_index(v_systolic: f64, v_diastolic: f64) -> f64 {
    if v_systolic.abs() < 1e-15 {
        return 0.0;
    }
    (v_systolic - v_diastolic) / v_systolic
}

/// Compute the maximum unambiguous (Nyquist) velocity for PW Doppler.
///
/// ```text
///            PRF * c
/// v_max = ─────────────────
///         4 * f0 * cos(θ)
/// ```
///
/// Velocities above this threshold will alias in pulsed-wave mode.
pub fn nyquist_velocity(prf_hz: f64, f0_hz: f64, c: f64, angle_deg: f64) -> f64 {
    let theta = angle_deg * PI / 180.0;
    let cos_theta = theta.cos();
    if cos_theta.abs() < 1e-15 || f0_hz.abs() < 1e-15 {
        return 0.0;
    }
    (prf_hz * c) / (4.0 * f0_hz * cos_theta)
}

/// Compute the maximum imaging depth for pulsed-wave Doppler.
///
/// ```text
/// d_max = c / (2 * PRF)
/// ```
///
/// There is a fundamental trade-off: higher PRF allows higher Nyquist
/// velocity but limits the maximum depth.
pub fn max_depth_m(prf_hz: f64, c: f64) -> f64 {
    if prf_hz.abs() < 1e-15 {
        return 0.0;
    }
    c / (2.0 * prf_hz)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the power spectrum via a manual radix-2 DIT FFT (pure Rust).
///
/// Returns |X[k]|^2 for each bin, fft-shifted so that DC is in the center.
fn compute_power_spectrum(iq: &[(f64, f64)], n: usize) -> Vec<f64> {
    // Zero-pad to next power of 2
    let fft_len = n.next_power_of_two();

    let mut re = vec![0.0; fft_len];
    let mut im = vec![0.0; fft_len];
    for (k, &(i, q)) in iq.iter().enumerate().take(fft_len) {
        re[k] = i;
        im[k] = q;
    }

    fft_inplace(&mut re, &mut im);

    // Power spectrum: |X[k]|^2, fft-shifted
    let mut power = vec![0.0; fft_len];
    let half = fft_len / 2;
    for k in 0..fft_len {
        let mag_sq = re[k] * re[k] + im[k] * im[k];
        // fftshift: move DC to center
        let shifted_k = if k < half { k + half } else { k - half };
        power[shifted_k] = mag_sq;
    }

    power
}

/// In-place radix-2 decimation-in-time FFT (Cooley-Tukey).
///
/// `re` and `im` must have power-of-2 length.
fn fft_inplace(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let wr = angle.cos();
                let wi = angle.sin();

                let a = start + k;
                let b = start + k + half;

                let tr = wr * re[b] - wi * im[b];
                let ti = wr * im[b] + wi * re[b];

                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        len <<= 1;
    }
}

/// Mean frequency estimator that returns frequency in Hz.
fn mean_frequency_estimator_with_df(spectrum: &[f64], df: f64, fs: f64) -> f64 {
    if spectrum.is_empty() {
        return 0.0;
    }

    let n = spectrum.len();
    let total_power: f64 = spectrum.iter().sum();

    if total_power.abs() < 1e-30 {
        return 0.0;
    }

    let half = n / 2;
    let weighted_sum: f64 = spectrum
        .iter()
        .enumerate()
        .map(|(k, &p)| {
            // Spectrum is fftshifted: bin 0 = -fs/2, bin half = DC, bin n-1 = +fs/2-df
            let freq = (k as f64 - half as f64) * df;
            freq * p
        })
        .sum();

    let result = weighted_sum / total_power;
    result.clamp(-fs / 2.0, fs / 2.0)
}

/// Find the frequency of the peak spectral bin (excluding DC neighbourhood).
fn peak_frequency(spectrum: &[f64], df: f64, _fs: f64) -> f64 {
    if spectrum.is_empty() {
        return 0.0;
    }

    let n = spectrum.len();
    let half = n / 2;

    // Skip DC +/- 2 bins
    let dc_guard = 3.min(half);
    let mut max_val = f64::NEG_INFINITY;
    let mut max_k = half; // default to DC

    for k in 0..n {
        let dist_from_dc = if k > half {
            k - half
        } else {
            half - k
        };
        if dist_from_dc < dc_guard {
            continue;
        }
        if spectrum[k] > max_val {
            max_val = spectrum[k];
            max_k = k;
        }
    }

    // Convert shifted bin to frequency
    (max_k as f64 - half as f64) * df
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-6;
    const C_TISSUE: f64 = 1540.0;

    // -----------------------------------------------------------------------
    // doppler_shift_to_velocity
    // -----------------------------------------------------------------------

    #[test]
    fn test_doppler_shift_to_velocity_basic() {
        // 1 kHz shift at 5 MHz, 60 degrees, c = 1540 m/s
        // v = 1000 * 1540 / (2 * 5e6 * cos(60)) = 1540000 / 5000000 = 0.308
        let v = doppler_shift_to_velocity(1000.0, 5.0e6, 60.0, C_TISSUE);
        assert!(
            (v - 0.308).abs() < 0.001,
            "Expected ~0.308 m/s, got {}",
            v
        );
    }

    #[test]
    fn test_doppler_shift_to_velocity_zero_angle() {
        // At 0 degrees: cos(0) = 1 => v = fd*c / (2*f0)
        let v = doppler_shift_to_velocity(1000.0, 5.0e6, 0.0, C_TISSUE);
        let expected = 1000.0 * C_TISSUE / (2.0 * 5.0e6);
        assert!(
            (v - expected).abs() < EPSILON,
            "Expected {}, got {}",
            expected,
            v
        );
    }

    #[test]
    fn test_doppler_shift_to_velocity_negative_shift() {
        // Negative Doppler shift => blood flowing away
        let v = doppler_shift_to_velocity(-2000.0, 5.0e6, 60.0, C_TISSUE);
        assert!(v < 0.0, "Negative shift should give negative velocity");
    }

    #[test]
    fn test_doppler_shift_to_velocity_90_deg() {
        // At 90 degrees cos(90) = 0, should return 0 (undefined)
        let v = doppler_shift_to_velocity(1000.0, 5.0e6, 90.0, C_TISSUE);
        assert!(
            v.abs() < EPSILON,
            "90-degree angle should give zero velocity, got {}",
            v
        );
    }

    #[test]
    fn test_doppler_shift_to_velocity_zero_freq() {
        let v = doppler_shift_to_velocity(1000.0, 0.0, 60.0, C_TISSUE);
        assert!(v.abs() < EPSILON, "Zero transmit freq should give zero velocity");
    }

    // -----------------------------------------------------------------------
    // velocity_to_doppler_shift
    // -----------------------------------------------------------------------

    #[test]
    fn test_velocity_to_doppler_shift_basic() {
        // fd = 2 * v * f0 * cos(theta) / c
        let fd = velocity_to_doppler_shift(0.308, 5.0e6, 60.0, C_TISSUE);
        assert!(
            (fd - 1000.0).abs() < 5.0,
            "Expected ~1000 Hz, got {}",
            fd
        );
    }

    #[test]
    fn test_velocity_doppler_roundtrip() {
        // Convert velocity -> shift -> velocity should be identity
        let v_orig = 1.5;
        let fd = velocity_to_doppler_shift(v_orig, 3.5e6, 45.0, C_TISSUE);
        let v_back = doppler_shift_to_velocity(fd, 3.5e6, 45.0, C_TISSUE);
        assert!(
            (v_orig - v_back).abs() < EPSILON,
            "Roundtrip failed: {} -> {} -> {}",
            v_orig,
            fd,
            v_back
        );
    }

    #[test]
    fn test_velocity_to_doppler_shift_zero_c() {
        let fd = velocity_to_doppler_shift(1.0, 5.0e6, 60.0, 0.0);
        assert!(fd.abs() < EPSILON, "Zero speed-of-sound should give zero shift");
    }

    // -----------------------------------------------------------------------
    // wall_filter
    // -----------------------------------------------------------------------

    #[test]
    fn test_wall_filter_removes_dc() {
        // Constant DC signal should be attenuated
        let dc: Vec<(f64, f64)> = vec![(1.0, 0.5); 200];
        let filtered = wall_filter(&dc, 50.0, 8000.0);
        assert_eq!(filtered.len(), 200);

        // Last samples should be near zero since DC is removed
        let last = filtered.last().unwrap();
        assert!(
            last.0.abs() < 0.05 && last.1.abs() < 0.05,
            "DC should be attenuated, got {:?}",
            last
        );
    }

    #[test]
    fn test_wall_filter_passes_high_freq() {
        // High-frequency tone (1 kHz at 8 kHz sample rate) should pass
        let fs = 8000.0;
        let freq = 1000.0;
        let n = 200;
        let tone: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let filtered = wall_filter(&tone, 50.0, fs);

        // Compute power of the last 100 samples (after transient)
        let power: f64 = filtered[100..]
            .iter()
            .map(|(i, q)| i * i + q * q)
            .sum::<f64>()
            / 100.0;

        assert!(power > 0.3, "High-freq signal should pass through, power={}", power);
    }

    #[test]
    fn test_wall_filter_empty() {
        let filtered = wall_filter(&[], 50.0, 8000.0);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_wall_filter_zero_sample_rate() {
        let filtered = wall_filter(&[(1.0, 0.0)], 50.0, 0.0);
        assert!(filtered.is_empty());
    }

    // -----------------------------------------------------------------------
    // compute_spectrogram
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectrogram_dimensions() {
        let n = 1024;
        let data: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / 8000.0;
                ((2.0 * PI * 500.0 * t).cos(), (2.0 * PI * 500.0 * t).sin())
            })
            .collect();

        let fft_size = 256;
        let overlap = 128;
        let sg = compute_spectrogram(&data, fft_size, overlap);

        // Expected number of frames: (1024 - 256) / (256 - 128) + 1 = 7
        let expected_frames = (n - fft_size) / (fft_size - overlap) + 1;
        assert_eq!(
            sg.len(),
            expected_frames,
            "Expected {} frames, got {}",
            expected_frames,
            sg.len()
        );
        assert_eq!(sg[0].len(), fft_size, "Each frame should have fft_size bins");
    }

    #[test]
    fn test_spectrogram_empty() {
        let sg = compute_spectrogram(&[], 256, 128);
        assert!(sg.is_empty());
    }

    #[test]
    fn test_spectrogram_zero_fft_size() {
        let sg = compute_spectrogram(&[(1.0, 0.0)], 0, 0);
        assert!(sg.is_empty());
    }

    #[test]
    fn test_spectrogram_tone_detection() {
        // A tone at fs/4 should produce a peak at the corresponding bin
        let fs = 8000.0;
        let freq = 2000.0; // fs/4
        let n = 256;
        let data: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let sg = compute_spectrogram(&data, n, 0);
        assert_eq!(sg.len(), 1);

        // Find peak bin (should be near bin n/2 + n/4 = 192 for +2000 Hz in fftshifted spectrum)
        let max_bin = sg[0]
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;

        // Expected: positive freq at +fs/4, fftshifted bin = n/2 + n/4 = 192
        let expected_bin = n / 2 + n / 4;
        assert!(
            (max_bin as i32 - expected_bin as i32).unsigned_abs() <= 2,
            "Peak at bin {}, expected near {}",
            max_bin,
            expected_bin
        );
    }

    // -----------------------------------------------------------------------
    // mean_frequency_estimator
    // -----------------------------------------------------------------------

    #[test]
    fn test_mean_frequency_estimator_dc() {
        // All energy at DC (bin 0 normalised = 0)
        let mut spectrum = vec![0.0; 256];
        spectrum[0] = 1.0;
        let f = mean_frequency_estimator(&spectrum);
        assert!(f.abs() < 0.01, "DC-only spectrum should give ~0 freq, got {}", f);
    }

    #[test]
    fn test_mean_frequency_estimator_positive_freq() {
        // Energy concentrated at bin n/4 -> normalised freq = 0.25
        let n = 256;
        let mut spectrum = vec![0.0; n];
        spectrum[n / 4] = 1.0;
        let f = mean_frequency_estimator(&spectrum);
        assert!(
            (f - 0.25).abs() < 0.01,
            "Expected ~0.25, got {}",
            f
        );
    }

    #[test]
    fn test_mean_frequency_estimator_empty() {
        let f = mean_frequency_estimator(&[]);
        assert!(f.abs() < EPSILON);
    }

    #[test]
    fn test_mean_frequency_estimator_zero_power() {
        let spectrum = vec![0.0; 128];
        let f = mean_frequency_estimator(&spectrum);
        assert!(f.abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // autocorrelation_velocity (Kasai estimator)
    // -----------------------------------------------------------------------

    #[test]
    fn test_autocorrelation_velocity_known_phase() {
        // Create IQ data with a known constant phase advance per sample
        // Phase advance = 2*pi*fd/PRF per pulse
        let prf = 8000.0;
        let f0 = 5.0e6;
        let c = C_TISSUE;
        let angle_deg = 60.0;

        // Target velocity => expected Doppler
        let target_v = 0.5; // m/s
        let fd = velocity_to_doppler_shift(target_v, f0, angle_deg, c);

        // Phase advance per sample at PRF
        let dphi = 2.0 * PI * fd / prf;

        let n = 64;
        let iq: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let phase = dphi * k as f64;
                (phase.cos(), phase.sin())
            })
            .collect();

        let v = autocorrelation_velocity(&iq, prf, f0, c, angle_deg);
        assert!(
            (v - target_v).abs() < 0.02,
            "Expected ~{} m/s, got {} m/s",
            target_v,
            v
        );
    }

    #[test]
    fn test_autocorrelation_velocity_zero_signal() {
        let iq = vec![(0.0, 0.0); 32];
        let v = autocorrelation_velocity(&iq, 8000.0, 5.0e6, C_TISSUE, 60.0);
        assert!(v.abs() < EPSILON, "Zero signal should give zero velocity");
    }

    #[test]
    fn test_autocorrelation_velocity_single_sample() {
        let v = autocorrelation_velocity(&[(1.0, 0.0)], 8000.0, 5.0e6, C_TISSUE, 60.0);
        assert!(v.abs() < EPSILON, "Single sample should give zero velocity");
    }

    // -----------------------------------------------------------------------
    // pulsatility_index
    // -----------------------------------------------------------------------

    #[test]
    fn test_pulsatility_index_normal() {
        // Typical: Vs=1.2, Vd=0.4, Vm=0.7
        let pi = pulsatility_index(1.2, 0.4, 0.7);
        let expected = (1.2 - 0.4) / 0.7;
        assert!(
            (pi - expected).abs() < EPSILON,
            "PI: expected {}, got {}",
            expected,
            pi
        );
    }

    #[test]
    fn test_pulsatility_index_zero_mean() {
        let pi = pulsatility_index(1.0, 0.5, 0.0);
        assert!(pi.abs() < EPSILON, "Zero mean should give PI=0");
    }

    #[test]
    fn test_pulsatility_index_equal_velocities() {
        let pi = pulsatility_index(1.0, 1.0, 1.0);
        assert!(pi.abs() < EPSILON, "Equal systolic/diastolic should give PI=0");
    }

    // -----------------------------------------------------------------------
    // resistive_index
    // -----------------------------------------------------------------------

    #[test]
    fn test_resistive_index_normal() {
        // Typical renal RI ~0.6: Vs=1.0, Vd=0.4
        let ri = resistive_index(1.0, 0.4);
        assert!(
            (ri - 0.6).abs() < EPSILON,
            "RI: expected 0.6, got {}",
            ri
        );
    }

    #[test]
    fn test_resistive_index_zero_systolic() {
        let ri = resistive_index(0.0, 0.5);
        assert!(ri.abs() < EPSILON, "Zero systolic should give RI=0");
    }

    #[test]
    fn test_resistive_index_no_diastolic() {
        // Complete diastolic flow reversal
        let ri = resistive_index(1.0, 0.0);
        assert!(
            (ri - 1.0).abs() < EPSILON,
            "Zero diastolic should give RI=1.0, got {}",
            ri
        );
    }

    // -----------------------------------------------------------------------
    // nyquist_velocity
    // -----------------------------------------------------------------------

    #[test]
    fn test_nyquist_velocity_basic() {
        // v_max = PRF * c / (4 * f0 * cos(theta))
        let v_max = nyquist_velocity(8000.0, 5.0e6, C_TISSUE, 60.0);
        let expected = 8000.0 * C_TISSUE / (4.0 * 5.0e6 * (60.0_f64 * PI / 180.0).cos());
        assert!(
            (v_max - expected).abs() < EPSILON,
            "Nyquist velocity: expected {}, got {}",
            expected,
            v_max
        );
    }

    #[test]
    fn test_nyquist_velocity_higher_prf() {
        let v1 = nyquist_velocity(8000.0, 5.0e6, C_TISSUE, 60.0);
        let v2 = nyquist_velocity(16000.0, 5.0e6, C_TISSUE, 60.0);
        assert!(
            v2 > v1,
            "Higher PRF should give higher Nyquist velocity"
        );
        assert!(
            (v2 / v1 - 2.0).abs() < EPSILON,
            "Doubling PRF should double Nyquist velocity"
        );
    }

    #[test]
    fn test_nyquist_velocity_90_deg() {
        let v = nyquist_velocity(8000.0, 5.0e6, C_TISSUE, 90.0);
        assert!(v.abs() < EPSILON, "90-degree angle should give zero Nyquist velocity");
    }

    // -----------------------------------------------------------------------
    // max_depth_m
    // -----------------------------------------------------------------------

    #[test]
    fn test_max_depth_basic() {
        // d = c / (2 * PRF) = 1540 / (2 * 8000) = 0.09625 m
        let d = max_depth_m(8000.0, C_TISSUE);
        let expected = C_TISSUE / (2.0 * 8000.0);
        assert!(
            (d - expected).abs() < EPSILON,
            "Max depth: expected {}, got {}",
            expected,
            d
        );
    }

    #[test]
    fn test_max_depth_inverse_prf() {
        let d1 = max_depth_m(4000.0, C_TISSUE);
        let d2 = max_depth_m(8000.0, C_TISSUE);
        assert!(
            (d1 / d2 - 2.0).abs() < EPSILON,
            "Halving PRF should double max depth"
        );
    }

    #[test]
    fn test_max_depth_zero_prf() {
        let d = max_depth_m(0.0, C_TISSUE);
        assert!(d.abs() < EPSILON, "Zero PRF should give zero depth");
    }

    // -----------------------------------------------------------------------
    // DopplerProcessor
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_empty_frame() {
        let config = DopplerConfig::default();
        let proc = DopplerProcessor::new(config);
        let result = proc.process_frame(&[]);
        assert!(result.mean_velocity_mps.abs() < EPSILON);
        assert!(result.spectrum.is_empty());
    }

    #[test]
    fn test_processor_dc_signal() {
        // A DC-only signal should produce near-zero velocity after wall filtering
        let config = DopplerConfig::default();
        let proc = DopplerProcessor::new(config);
        let dc: Vec<(f64, f64)> = vec![(1.0, 0.0); 256];
        let result = proc.process_frame(&dc);
        // After wall filter removes DC, velocity should be near zero
        assert!(
            result.mean_velocity_mps.abs() < 1.0,
            "DC signal should give low mean velocity after wall filter, got {}",
            result.mean_velocity_mps
        );
    }

    #[test]
    fn test_processor_returns_spectrum() {
        let config = DopplerConfig::default();
        let sample_rate = config.sample_rate_hz;
        let proc = DopplerProcessor::new(config);
        let n = 128;
        let data: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / sample_rate;
                let phase = 2.0 * PI * 500.0 * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let result = proc.process_frame(&data);
        assert!(
            !result.spectrum.is_empty(),
            "Spectrum should not be empty"
        );
        assert!(
            result.spectrum.len().is_power_of_two(),
            "Spectrum length should be power of 2"
        );
    }

    #[test]
    fn test_processor_positive_velocity_for_positive_doppler() {
        let config = DopplerConfig {
            transmit_freq_mhz: 5.0,
            prf_hz: 8000.0,
            sample_rate_hz: 16000.0,
            speed_of_sound_mps: C_TISSUE,
            beam_angle_deg: 60.0,
        };
        let proc = DopplerProcessor::new(config);

        // Generate a positive-frequency tone (blood flowing toward probe)
        let n = 256;
        let freq = 1000.0; // 1 kHz Doppler shift
        let data: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / 16000.0;
                let phase = 2.0 * PI * freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let result = proc.process_frame(&data);
        assert!(
            result.peak_velocity_mps.abs() > 0.01,
            "Should detect non-zero peak velocity for tone input"
        );
    }

    // -----------------------------------------------------------------------
    // FFT internal
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_impulse() {
        // FFT of an impulse should be flat (all bins equal magnitude)
        let mut re = vec![0.0; 8];
        let mut im = vec![0.0; 8];
        re[0] = 1.0;

        fft_inplace(&mut re, &mut im);

        for k in 0..8 {
            let mag = (re[k] * re[k] + im[k] * im[k]).sqrt();
            assert!(
                (mag - 1.0).abs() < EPSILON,
                "Impulse FFT bin {} magnitude should be 1.0, got {}",
                k,
                mag
            );
        }
    }

    #[test]
    fn test_fft_dc() {
        // FFT of constant signal should have energy only at bin 0
        let n = 16;
        let mut re = vec![1.0; n];
        let mut im = vec![0.0; n];

        fft_inplace(&mut re, &mut im);

        let dc_mag = (re[0] * re[0] + im[0] * im[0]).sqrt();
        assert!(
            (dc_mag - n as f64).abs() < EPSILON,
            "DC bin should have magnitude N, got {}",
            dc_mag
        );

        for k in 1..n {
            let mag = (re[k] * re[k] + im[k] * im[k]).sqrt();
            assert!(
                mag < EPSILON,
                "Non-DC bin {} should be zero, got {}",
                k,
                mag
            );
        }
    }

    // -----------------------------------------------------------------------
    // DopplerConfig default
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_default() {
        let config = DopplerConfig::default();
        assert!((config.speed_of_sound_mps - 1540.0).abs() < EPSILON);
        assert!((config.beam_angle_deg - 60.0).abs() < EPSILON);
        assert!((config.transmit_freq_mhz - 5.0).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Depth-velocity trade-off
    // -----------------------------------------------------------------------

    #[test]
    fn test_depth_velocity_tradeoff() {
        // Higher PRF -> higher Nyquist velocity but lower max depth
        let prf_low = 4000.0;
        let prf_high = 16000.0;

        let d_low = max_depth_m(prf_low, C_TISSUE);
        let d_high = max_depth_m(prf_high, C_TISSUE);
        assert!(d_low > d_high, "Lower PRF should give greater depth");

        let v_low = nyquist_velocity(prf_low, 5.0e6, C_TISSUE, 60.0);
        let v_high = nyquist_velocity(prf_high, 5.0e6, C_TISSUE, 60.0);
        assert!(v_high > v_low, "Higher PRF should give higher Nyquist velocity");
    }

    // -----------------------------------------------------------------------
    // Edge cases and numerical stability
    // -----------------------------------------------------------------------

    #[test]
    fn test_very_high_frequency_probe() {
        // 15 MHz probe (superficial imaging)
        let v = doppler_shift_to_velocity(3000.0, 15.0e6, 45.0, C_TISSUE);
        let expected = 3000.0 * C_TISSUE / (2.0 * 15.0e6 * (45.0_f64 * PI / 180.0).cos());
        assert!(
            (v - expected).abs() < EPSILON,
            "High-freq probe: expected {}, got {}",
            expected,
            v
        );
    }

    #[test]
    fn test_physiological_velocity_range() {
        // Normal carotid artery: peak systolic ~0.5-1.5 m/s
        let fd = velocity_to_doppler_shift(1.0, 5.0e6, 60.0, C_TISSUE);
        assert!(
            fd > 0.0 && fd < 10000.0,
            "Physiological Doppler shift should be in audible range, got {} Hz",
            fd
        );
    }

    #[test]
    fn test_wall_filter_single_sample() {
        let filtered = wall_filter(&[(1.0, 0.5)], 50.0, 8000.0);
        assert_eq!(filtered.len(), 1);
        // First sample is zeroed
        assert!((filtered[0].0).abs() < EPSILON);
        assert!((filtered[0].1).abs() < EPSILON);
    }
}
