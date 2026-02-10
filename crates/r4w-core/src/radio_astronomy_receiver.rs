//! Radio astronomy signal processing for amateur and professional radio telescopes.
//!
//! This module provides radiometer implementations, noise temperature calibration,
//! spectral line detection, pulsar processing, RFI mitigation, and related utilities
//! commonly used in radio astronomy observations.
//!
//! # Features
//!
//! - **Radiometers**: Total power and Dicke switching radiometer algorithms
//! - **Calibration**: Y-factor method for noise temperature and gain measurement
//! - **Spectral lines**: 21 cm hydrogen line detection and spectral line extraction
//! - **Solar**: Solar radio burst detection using sigma-threshold algorithms
//! - **Pulsar**: Epoch folding and incoherent dedispersion
//! - **RFI**: Radio frequency interference flagging
//! - **Antenna temperature**: Computation from raw power measurements
//!
//! # Example
//!
//! ```
//! use r4w_core::radio_astronomy_receiver::{
//!     RadiometerConfig, RadioAstronomyReceiver,
//!     total_power_radiometer, radiometer_sensitivity,
//! };
//!
//! // Configure a simple total-power radiometer
//! let config = RadiometerConfig {
//!     bandwidth_hz: 1.0e6,
//!     integration_time_s: 1.0,
//!     system_temp_k: 50.0,
//!     gain: 1e6,
//! };
//!
//! let receiver = RadioAstronomyReceiver::new(config);
//! let sensitivity = receiver.sensitivity();
//! assert!(sensitivity < 1.0, "Sensitivity should be sub-kelvin for these parameters");
//!
//! // Direct function usage
//! let delta_t = radiometer_sensitivity(50.0, 1.0e6, 1.0);
//! assert!((delta_t - 0.05).abs() < 0.001);
//! ```

// ---------------------------------------------------------------------------
// Configuration and result types
// ---------------------------------------------------------------------------

/// Configuration for a radiometer receiver.
#[derive(Debug, Clone)]
pub struct RadiometerConfig {
    /// Receiver bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Integration (averaging) time in seconds.
    pub integration_time_s: f64,
    /// Total system noise temperature in kelvin.
    pub system_temp_k: f64,
    /// Receiver gain (linear, dimensionless).
    pub gain: f64,
}

/// Result of a radiometer measurement.
#[derive(Debug, Clone)]
pub struct RadiometerResult {
    /// Estimated antenna temperature in kelvin.
    pub antenna_temp_k: f64,
    /// RMS sensitivity (delta-T rms) in kelvin.
    pub rms_sensitivity_k: f64,
    /// Signal-to-noise ratio in dB.
    pub snr_db: f64,
}

// ---------------------------------------------------------------------------
// Main receiver struct
// ---------------------------------------------------------------------------

/// A radio astronomy receiver combining radiometry, calibration, spectral
/// analysis, pulsar processing, and RFI mitigation.
#[derive(Debug, Clone)]
pub struct RadioAstronomyReceiver {
    config: RadiometerConfig,
}

impl RadioAstronomyReceiver {
    /// Create a new receiver with the given configuration.
    pub fn new(config: RadiometerConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &RadiometerConfig {
        &self.config
    }

    /// Compute the radiometer sensitivity (delta-T rms) for the current config.
    pub fn sensitivity(&self) -> f64 {
        radiometer_sensitivity(
            self.config.system_temp_k,
            self.config.bandwidth_hz,
            self.config.integration_time_s,
        )
    }

    /// Perform a total-power measurement on the given power samples and return
    /// a complete [`RadiometerResult`].
    pub fn measure(&self, power_samples: &[f64]) -> RadiometerResult {
        let antenna_temp_k = total_power_radiometer(
            power_samples,
            self.config.system_temp_k,
            self.config.gain,
        );
        let rms_sensitivity_k = self.sensitivity();
        let snr_db = if rms_sensitivity_k > 0.0 {
            10.0 * (antenna_temp_k / rms_sensitivity_k).abs().log10()
        } else {
            f64::INFINITY
        };
        RadiometerResult {
            antenna_temp_k,
            rms_sensitivity_k,
            snr_db,
        }
    }

    /// Run Dicke-switching measurement and return antenna temperature estimate.
    pub fn measure_dicke(&self, signal_power: f64, reference_power: f64) -> f64 {
        dicke_switch(signal_power, reference_power, self.config.system_temp_k)
    }

    /// Perform Y-factor calibration returning `(noise_temp, gain)`.
    pub fn calibrate(
        &self,
        hot_power: f64,
        cold_power: f64,
        hot_temp_k: f64,
        cold_temp_k: f64,
    ) -> (f64, f64) {
        y_factor_calibration(hot_power, cold_power, hot_temp_k, cold_temp_k)
    }
}

// ---------------------------------------------------------------------------
// Free functions -- radiometry
// ---------------------------------------------------------------------------

/// Estimate antenna temperature from total-power radiometer samples.
///
/// The antenna temperature is derived by dividing the mean measured power by
/// the system gain and subtracting the system noise contribution:
///
/// `T_a = P_mean / (k * G * B) - T_sys`
///
/// For simplicity this implementation uses `T_a = (mean_power / gain) - T_sys`
/// where power samples are assumed to already be in temperature-equivalent
/// units scaled by the gain.
///
/// # Arguments
///
/// * `power_samples` -- Raw power detector output samples.
/// * `system_temp_k` -- System noise temperature in kelvin.
/// * `gain` -- Receiver gain (linear).
///
/// # Returns
///
/// Estimated antenna temperature in kelvin. May be negative if the source is
/// weaker than the system noise.
pub fn total_power_radiometer(power_samples: &[f64], system_temp_k: f64, gain: f64) -> f64 {
    if power_samples.is_empty() || gain == 0.0 {
        return 0.0;
    }
    let mean_power: f64 = power_samples.iter().sum::<f64>() / power_samples.len() as f64;
    (mean_power / gain) - system_temp_k
}

/// Dicke switching radiometer.
///
/// Computes the antenna temperature from the difference between signal and
/// reference observations:
///
/// `T_a = T_sys * (P_sig - P_ref) / P_ref`
///
/// # Arguments
///
/// * `signal_power` -- Power measured while observing the source.
/// * `reference_power` -- Power measured on the reference load.
/// * `system_temp_k` -- System noise temperature in kelvin.
///
/// # Returns
///
/// Estimated antenna temperature in kelvin.
pub fn dicke_switch(signal_power: f64, reference_power: f64, system_temp_k: f64) -> f64 {
    if reference_power == 0.0 {
        return 0.0;
    }
    system_temp_k * (signal_power - reference_power) / reference_power
}

/// Radiometer sensitivity (delta-T rms) -- the minimum detectable temperature change.
///
/// `delta_T = T_sys / sqrt(B * tau)`
///
/// # Arguments
///
/// * `system_temp_k` -- System noise temperature in kelvin.
/// * `bandwidth_hz` -- Receiver bandwidth in Hz.
/// * `integration_time_s` -- Integration time in seconds.
///
/// # Returns
///
/// RMS sensitivity in kelvin.
pub fn radiometer_sensitivity(system_temp_k: f64, bandwidth_hz: f64, integration_time_s: f64) -> f64 {
    let bt = bandwidth_hz * integration_time_s;
    if bt <= 0.0 {
        return f64::INFINITY;
    }
    system_temp_k / bt.sqrt()
}

// ---------------------------------------------------------------------------
// Free functions -- calibration
// ---------------------------------------------------------------------------

/// Y-factor noise temperature calibration.
///
/// Given hot and cold load measurements, determines the receiver noise
/// temperature and gain:
///
/// ```text
/// Y = P_hot / P_cold
/// T_rx = (T_hot - Y * T_cold) / (Y - 1)
/// G = P_hot / (T_hot + T_rx)
/// ```
///
/// # Arguments
///
/// * `hot_power` -- Power measured on the hot load.
/// * `cold_power` -- Power measured on the cold load.
/// * `hot_temp_k` -- Physical temperature of the hot load in kelvin.
/// * `cold_temp_k` -- Physical temperature of the cold load in kelvin.
///
/// # Returns
///
/// A tuple `(noise_temperature_k, gain)`.
pub fn y_factor_calibration(
    hot_power: f64,
    cold_power: f64,
    hot_temp_k: f64,
    cold_temp_k: f64,
) -> (f64, f64) {
    if cold_power == 0.0 {
        return (0.0, 0.0);
    }
    let y = hot_power / cold_power;
    if (y - 1.0).abs() < 1e-15 {
        return (0.0, 0.0);
    }
    let noise_temp = (hot_temp_k - y * cold_temp_k) / (y - 1.0);
    let gain = hot_power / (hot_temp_k + noise_temp);
    (noise_temp, gain)
}

// ---------------------------------------------------------------------------
// Free functions -- spectral line detection
// ---------------------------------------------------------------------------

/// The rest frequency of the hydrogen 21 cm line in Hz.
pub const HYDROGEN_LINE_HZ: f64 = 1_420_405_751.768;

/// Detect the 21 cm hydrogen line in a power spectrum.
///
/// Searches for a peak near the expected H-I frequency (1420.405 MHz) within
/// the provided spectrum. Returns the frequency and peak power if a peak
/// exceeding 3 sigma above the mean is found in the hydrogen band.
///
/// # Arguments
///
/// * `spectrum` -- Power spectral density values (linear).
/// * `freq_start_hz` -- Frequency of the first bin.
/// * `freq_step_hz` -- Frequency spacing between bins.
///
/// # Returns
///
/// `Some((frequency_hz, peak_power))` if hydrogen line is detected, else `None`.
pub fn detect_hydrogen_line(
    spectrum: &[f64],
    freq_start_hz: f64,
    freq_step_hz: f64,
) -> Option<(f64, f64)> {
    if spectrum.is_empty() || freq_step_hz <= 0.0 {
        return None;
    }

    // Window around the hydrogen line: +/- 500 kHz
    let margin_hz = 500_000.0;
    let lo = HYDROGEN_LINE_HZ - margin_hz;
    let hi = HYDROGEN_LINE_HZ + margin_hz;

    let n = spectrum.len();
    let mean = spectrum.iter().sum::<f64>() / n as f64;
    let variance = spectrum.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n as f64;
    let sigma = variance.sqrt();
    let threshold = mean + 3.0 * sigma;

    let mut best_power = f64::NEG_INFINITY;
    let mut best_freq = 0.0;

    for (i, &psd) in spectrum.iter().enumerate() {
        let freq = freq_start_hz + i as f64 * freq_step_hz;
        if freq >= lo && freq <= hi && psd > threshold && psd > best_power {
            best_power = psd;
            best_freq = freq;
        }
    }

    if best_power > f64::NEG_INFINITY {
        Some((best_freq, best_power))
    } else {
        None
    }
}

/// Extract a spectral line from a power spectrum.
///
/// Computes the integrated area, power-weighted centroid, and full-width at
/// half-maximum (FWHM) of a spectral feature centred on `center_bin` with
/// a window of `width_bins` on each side.
///
/// # Arguments
///
/// * `spectrum` -- Power spectral density values.
/// * `center_bin` -- Bin index of the spectral line centre.
/// * `width_bins` -- Number of bins on each side of centre to include.
///
/// # Returns
///
/// A tuple `(area, centroid_bin, fwhm_bins)`.
pub fn spectral_line_extract(
    spectrum: &[f64],
    center_bin: usize,
    width_bins: usize,
) -> (f64, f64, f64) {
    if spectrum.is_empty() {
        return (0.0, 0.0, 0.0);
    }

    let start = center_bin.saturating_sub(width_bins);
    let end = (center_bin + width_bins + 1).min(spectrum.len());

    if start >= end {
        return (0.0, center_bin as f64, 0.0);
    }

    // Integrated area and centroid
    let mut area = 0.0;
    let mut weighted_sum = 0.0;
    let mut peak_val = f64::NEG_INFINITY;

    for i in start..end {
        area += spectrum[i];
        weighted_sum += spectrum[i] * i as f64;
        if spectrum[i] > peak_val {
            peak_val = spectrum[i];
        }
    }

    let centroid = if area > 0.0 {
        weighted_sum / area
    } else {
        center_bin as f64
    };

    // FWHM -- find the half-power points
    let half_max = peak_val / 2.0;
    let mut left = start;
    let mut right = end.saturating_sub(1);

    for i in start..end {
        if spectrum[i] >= half_max {
            left = i;
            break;
        }
    }
    for i in (start..end).rev() {
        if spectrum[i] >= half_max {
            right = i;
            break;
        }
    }

    let fwhm = if right > left {
        (right - left) as f64
    } else {
        1.0
    };

    (area, centroid, fwhm)
}

// ---------------------------------------------------------------------------
// Free functions -- solar burst detection
// ---------------------------------------------------------------------------

/// Detect solar radio bursts in a power time series.
///
/// Identifies samples exceeding the mean by more than `threshold_sigma`
/// standard deviations.
///
/// # Arguments
///
/// * `power_series` -- Time series of power measurements.
/// * `threshold_sigma` -- Detection threshold in standard deviations.
///
/// # Returns
///
/// A vector of `(sample_index, power_value)` for each detected burst event.
pub fn detect_solar_burst(power_series: &[f64], threshold_sigma: f64) -> Vec<(usize, f64)> {
    if power_series.is_empty() {
        return Vec::new();
    }

    let n = power_series.len() as f64;
    let mean = power_series.iter().sum::<f64>() / n;
    let variance = power_series.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
    let sigma = variance.sqrt();

    if sigma == 0.0 {
        return Vec::new();
    }

    let threshold = mean + threshold_sigma * sigma;

    power_series
        .iter()
        .enumerate()
        .filter(|(_, &p)| p > threshold)
        .map(|(i, &p)| (i, p))
        .collect()
}

// ---------------------------------------------------------------------------
// Free functions -- pulsar processing
// ---------------------------------------------------------------------------

/// Fold a time series at a given pulsar period (epoch folding).
///
/// Accumulates samples into phase bins according to the pulsar rotation
/// period, producing an integrated pulse profile.
///
/// # Arguments
///
/// * `samples` -- Input time series (power or voltage).
/// * `period_s` -- Pulsar rotation period in seconds.
/// * `sample_rate` -- Sample rate in Hz.
/// * `num_bins` -- Number of phase bins in the folded profile.
///
/// # Returns
///
/// A vector of `num_bins` values representing the folded pulse profile,
/// normalised by the number of samples accumulated in each bin.
pub fn pulsar_fold(
    samples: &[f64],
    period_s: f64,
    sample_rate: f64,
    num_bins: usize,
) -> Vec<f64> {
    if samples.is_empty() || period_s <= 0.0 || sample_rate <= 0.0 || num_bins == 0 {
        return vec![0.0; num_bins];
    }

    let mut bins = vec![0.0_f64; num_bins];
    let mut counts = vec![0u64; num_bins];

    let samples_per_period = period_s * sample_rate;

    for (i, &s) in samples.iter().enumerate() {
        let phase = (i as f64 % samples_per_period) / samples_per_period;
        let bin = (phase * num_bins as f64) as usize;
        let bin = bin.min(num_bins - 1);
        bins[bin] += s;
        counts[bin] += 1;
    }

    for (b, c) in bins.iter_mut().zip(counts.iter()) {
        if *c > 0 {
            *b /= *c as f64;
        }
    }

    bins
}

/// Incoherent dedispersion of a dynamic spectrum.
///
/// Shifts each frequency channel in time to compensate for the dispersive
/// delay caused by the interstellar medium. The delay for channel `f` relative
/// to the highest frequency is:
///
/// ```text
/// dt = 4.149 ms * DM * (1/f_lo^2 - 1/f_hi^2)    (f in GHz, DM in pc/cm^3)
/// ```
///
/// # Arguments
///
/// * `spectra` -- Dynamic spectrum: `spectra[time_index][freq_channel]`.
/// * `dm` -- Dispersion measure in pc/cm^3.
/// * `freq_channels_hz` -- Centre frequency of each channel in Hz.
/// * `sample_rate` -- Time sample rate in Hz.
///
/// # Returns
///
/// Dedispersed dynamic spectrum with the same dimensions as the input.
pub fn dedisperse(
    spectra: &[Vec<f64>],
    dm: f64,
    freq_channels_hz: &[f64],
    sample_rate: f64,
) -> Vec<Vec<f64>> {
    if spectra.is_empty() || freq_channels_hz.is_empty() || sample_rate <= 0.0 {
        return spectra.to_vec();
    }

    let n_time = spectra.len();
    let n_freq = freq_channels_hz.len();

    // Find the highest frequency channel (reference -- zero delay)
    let f_max_ghz = freq_channels_hz
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max)
        / 1e9;

    // Precompute delay in samples for each channel
    let delays: Vec<isize> = freq_channels_hz
        .iter()
        .map(|&f_hz| {
            let f_ghz = f_hz / 1e9;
            if f_ghz <= 0.0 {
                return 0;
            }
            // 4.149 ms factor
            let delay_s = 4.149e-3 * dm * (1.0 / (f_ghz * f_ghz) - 1.0 / (f_max_ghz * f_max_ghz));
            (delay_s * sample_rate).round() as isize
        })
        .collect();

    // Build dedispersed spectra
    let mut output = vec![vec![0.0; n_freq]; n_time];
    for t in 0..n_time {
        for ch in 0..n_freq {
            let src_t = t as isize - delays[ch];
            if src_t >= 0 && (src_t as usize) < n_time {
                let src_idx = src_t as usize;
                if ch < spectra[src_idx].len() {
                    output[t][ch] = spectra[src_idx][ch];
                }
            }
        }
    }

    output
}

// ---------------------------------------------------------------------------
// Free functions -- RFI flagging
// ---------------------------------------------------------------------------

/// Flag RFI-contaminated spectral bins.
///
/// Marks bins whose power exceeds the median by more than `threshold_sigma`
/// median-absolute-deviations (MAD). Using MAD makes the flagging robust to
/// outliers.
///
/// # Arguments
///
/// * `spectrum` -- Power spectral density values.
/// * `threshold_sigma` -- Detection threshold in MAD-scaled units.
///
/// # Returns
///
/// A boolean mask of the same length as `spectrum`, where `true` indicates
/// an RFI-contaminated bin.
pub fn flag_rfi(spectrum: &[f64], threshold_sigma: f64) -> Vec<bool> {
    if spectrum.is_empty() {
        return Vec::new();
    }

    let median = median_of(spectrum);
    let deviations: Vec<f64> = spectrum.iter().map(|&x| (x - median).abs()).collect();
    let mad = median_of(&deviations);

    // Scale MAD to equivalent sigma (for Gaussian: sigma ~ 1.4826 * MAD)
    let sigma_est = 1.4826 * mad;

    if sigma_est == 0.0 {
        // All values identical -- flag nothing
        return vec![false; spectrum.len()];
    }

    let threshold = median + threshold_sigma * sigma_est;

    spectrum.iter().map(|&x| x > threshold).collect()
}

/// Compute the antenna temperature from raw power, Boltzmann's constant,
/// system gain, and bandwidth.
///
/// `T_a = P / (k_B * G * B)`
///
/// # Arguments
///
/// * `power_watts` -- Measured power in watts.
/// * `gain` -- Receiver gain (linear).
/// * `bandwidth_hz` -- Receiver bandwidth in Hz.
///
/// # Returns
///
/// Antenna temperature in kelvin.
pub fn antenna_temperature(power_watts: f64, gain: f64, bandwidth_hz: f64) -> f64 {
    // Boltzmann constant in J/K
    const K_B: f64 = 1.380649e-23;
    let denom = K_B * gain * bandwidth_hz;
    if denom == 0.0 {
        return 0.0;
    }
    power_watts / denom
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the median of a slice (non-destructive -- allocates a sorted copy).
fn median_of(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f64> = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // --- RadiometerConfig / RadioAstronomyReceiver tests --------------------

    #[test]
    fn test_receiver_sensitivity() {
        let config = RadiometerConfig {
            bandwidth_hz: 1.0e6,
            integration_time_s: 1.0,
            system_temp_k: 50.0,
            gain: 1.0,
        };
        let rx = RadioAstronomyReceiver::new(config);
        let s = rx.sensitivity();
        // delta_T = 50 / sqrt(1e6) = 0.05 K
        assert!((s - 0.05).abs() < 1e-6);
    }

    #[test]
    fn test_receiver_config_access() {
        let config = RadiometerConfig {
            bandwidth_hz: 2.0e6,
            integration_time_s: 0.5,
            system_temp_k: 100.0,
            gain: 1e3,
        };
        let rx = RadioAstronomyReceiver::new(config);
        assert!((rx.config().bandwidth_hz - 2.0e6).abs() < EPSILON);
        assert!((rx.config().system_temp_k - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_receiver_measure() {
        let config = RadiometerConfig {
            bandwidth_hz: 1.0e6,
            integration_time_s: 1.0,
            system_temp_k: 50.0,
            gain: 10.0,
        };
        let rx = RadioAstronomyReceiver::new(config);
        // Power samples: mean = 600 -> T_a = 600/10 - 50 = 10 K
        let samples = vec![580.0, 600.0, 620.0];
        let result = rx.measure(&samples);
        assert!((result.antenna_temp_k - 10.0).abs() < 1e-6);
        assert!(result.snr_db > 0.0);
    }

    #[test]
    fn test_receiver_measure_dicke() {
        let config = RadiometerConfig {
            bandwidth_hz: 1e6,
            integration_time_s: 1.0,
            system_temp_k: 100.0,
            gain: 1.0,
        };
        let rx = RadioAstronomyReceiver::new(config);
        let t_a = rx.measure_dicke(1.1, 1.0);
        // T_a = 100 * (1.1 - 1.0) / 1.0 = 10 K
        assert!((t_a - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_receiver_calibrate() {
        let config = RadiometerConfig {
            bandwidth_hz: 1e6,
            integration_time_s: 1.0,
            system_temp_k: 50.0,
            gain: 1.0,
        };
        let rx = RadioAstronomyReceiver::new(config);
        let (noise_temp, gain) = rx.calibrate(10.0, 5.0, 300.0, 77.0);
        // Y = 2, T_rx = (300 - 2*77)/(2-1) = 146
        assert!((noise_temp - 146.0).abs() < 1e-6);
        assert!(gain > 0.0);
    }

    // --- total_power_radiometer tests ---------------------------------------

    #[test]
    fn test_total_power_radiometer_basic() {
        // mean = 100, gain = 10, sys_temp = 5 -> T_a = 100/10 - 5 = 5
        let samples = vec![90.0, 100.0, 110.0];
        let t_a = total_power_radiometer(&samples, 5.0, 10.0);
        assert!((t_a - 5.0).abs() < 1e-9);
    }

    #[test]
    fn test_total_power_radiometer_empty() {
        assert_eq!(total_power_radiometer(&[], 50.0, 10.0), 0.0);
    }

    #[test]
    fn test_total_power_radiometer_zero_gain() {
        let samples = vec![1.0, 2.0, 3.0];
        assert_eq!(total_power_radiometer(&samples, 50.0, 0.0), 0.0);
    }

    // --- dicke_switch tests -------------------------------------------------

    #[test]
    fn test_dicke_switch_basic() {
        // T_a = 200 * (1.05 - 1.0) / 1.0 = 10
        let t_a = dicke_switch(1.05, 1.0, 200.0);
        assert!((t_a - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_dicke_switch_zero_reference() {
        assert_eq!(dicke_switch(1.0, 0.0, 100.0), 0.0);
    }

    // --- radiometer_sensitivity tests ---------------------------------------

    #[test]
    fn test_radiometer_sensitivity_basic() {
        // delta_T = 100 / sqrt(1e6 * 1) = 0.1
        let dt = radiometer_sensitivity(100.0, 1.0e6, 1.0);
        assert!((dt - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_radiometer_sensitivity_zero_bandwidth() {
        let dt = radiometer_sensitivity(100.0, 0.0, 1.0);
        assert!(dt.is_infinite());
    }

    // --- y_factor_calibration tests -----------------------------------------

    #[test]
    fn test_y_factor_calibration_basic() {
        // Hot = 300 K, Cold = 77 K, P_hot = 10, P_cold = 5 -> Y = 2
        // T_rx = (300 - 2*77) / (2 - 1) = 146 K
        let (noise_temp, gain) = y_factor_calibration(10.0, 5.0, 300.0, 77.0);
        assert!((noise_temp - 146.0).abs() < 1e-6);
        // gain = 10 / (300 + 146) = 0.02242...
        assert!((gain - 10.0 / 446.0).abs() < 1e-9);
    }

    #[test]
    fn test_y_factor_calibration_zero_cold_power() {
        let (nt, g) = y_factor_calibration(10.0, 0.0, 300.0, 77.0);
        assert_eq!(nt, 0.0);
        assert_eq!(g, 0.0);
    }

    // --- detect_hydrogen_line tests -----------------------------------------

    #[test]
    fn test_detect_hydrogen_line_found() {
        // Build a spectrum covering the H-I line with a peak
        let n = 1000;
        let freq_start = 1_420_000_000.0; // 1420 MHz
        let freq_step = 1000.0; // 1 kHz steps
        let mut spectrum = vec![1.0; n];
        // Place a strong peak near H-I (1420.4058 MHz)
        // Bin for 1420.4058 MHz: (1420405800 - 1420000000) / 1000 = 405.8
        let peak_bin = 406;
        spectrum[peak_bin] = 100.0;

        let result = detect_hydrogen_line(&spectrum, freq_start, freq_step);
        assert!(result.is_some());
        let (freq, power) = result.unwrap();
        assert!((freq - (freq_start + peak_bin as f64 * freq_step)).abs() < freq_step);
        assert!((power - 100.0).abs() < EPSILON);
    }

    #[test]
    fn test_detect_hydrogen_line_not_found() {
        // Spectrum far from H-I
        let spectrum = vec![1.0; 100];
        let result = detect_hydrogen_line(&spectrum, 1.0e9, 1000.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_hydrogen_line_empty() {
        assert!(detect_hydrogen_line(&[], 1.42e9, 1000.0).is_none());
    }

    // --- detect_solar_burst tests -------------------------------------------

    #[test]
    fn test_detect_solar_burst_basic() {
        let mut series = vec![1.0; 100];
        series[50] = 20.0; // Huge spike
        let bursts = detect_solar_burst(&series, 3.0);
        assert!(!bursts.is_empty());
        assert_eq!(bursts[0].0, 50);
    }

    #[test]
    fn test_detect_solar_burst_no_burst() {
        let series = vec![1.0; 100]; // Flat -- no bursts
        let bursts = detect_solar_burst(&series, 3.0);
        assert!(bursts.is_empty());
    }

    // --- pulsar_fold tests --------------------------------------------------

    #[test]
    fn test_pulsar_fold_basic() {
        // Create a signal with a known period
        let sample_rate = 1000.0; // 1 kHz
        let period_s = 0.01; // 10 ms period -> 10 samples per period
        let num_bins = 10;
        let n_samples = 1000; // 100 periods

        let mut samples = vec![0.0; n_samples];
        // Put a pulse at phase 0 of each period (every 10 samples)
        for i in (0..n_samples).step_by(10) {
            samples[i] = 10.0;
        }

        let profile = pulsar_fold(&samples, period_s, sample_rate, num_bins);
        assert_eq!(profile.len(), num_bins);
        // Bin 0 should be much larger than others
        assert!(profile[0] > profile[1] * 5.0);
    }

    #[test]
    fn test_pulsar_fold_empty() {
        let profile = pulsar_fold(&[], 0.01, 1000.0, 10);
        assert_eq!(profile.len(), 10);
        assert!(profile.iter().all(|&x| x == 0.0));
    }

    // --- dedisperse tests ---------------------------------------------------

    #[test]
    fn test_dedisperse_basic() {
        // 4 time steps, 2 frequency channels
        let spectra = vec![
            vec![0.0, 1.0],
            vec![0.0, 0.0],
            vec![1.0, 0.0],
            vec![0.0, 0.0],
        ];
        // Low freq channel should be delayed relative to high freq
        let freq_channels = vec![1.0e9, 1.5e9]; // 1 GHz, 1.5 GHz
        let dm = 10.0;
        let sample_rate = 1000.0;

        let output = dedisperse(&spectra, dm, &freq_channels, sample_rate);
        assert_eq!(output.len(), 4);
        assert_eq!(output[0].len(), 2);
    }

    #[test]
    fn test_dedisperse_empty() {
        let empty: Vec<Vec<f64>> = Vec::new();
        let result = dedisperse(&empty, 10.0, &[1.0e9], 1000.0);
        assert!(result.is_empty());
    }

    // --- flag_rfi tests -----------------------------------------------------

    #[test]
    fn test_flag_rfi_basic() {
        // Create a spectrum with natural variation so MAD is nonzero
        let mut spectrum: Vec<f64> = (0..100)
            .map(|i| 1.0 + 0.1 * ((i as f64 * 0.7).sin()))
            .collect();
        spectrum[25] = 100.0; // Strong RFI
        spectrum[75] = 100.0;
        let flags = flag_rfi(&spectrum, 3.0);
        assert_eq!(flags.len(), 100);
        assert!(flags[25]);
        assert!(flags[75]);
        // Most bins should be clean
        let flagged_count = flags.iter().filter(|&&f| f).count();
        assert!(flagged_count <= 5); // Only the outliers
    }

    #[test]
    fn test_flag_rfi_empty() {
        let flags = flag_rfi(&[], 3.0);
        assert!(flags.is_empty());
    }

    #[test]
    fn test_flag_rfi_flat_spectrum() {
        let spectrum = vec![5.0; 50];
        let flags = flag_rfi(&spectrum, 3.0);
        assert!(flags.iter().all(|&f| !f)); // Nothing flagged
    }

    // --- spectral_line_extract tests ----------------------------------------

    #[test]
    fn test_spectral_line_extract_basic() {
        // Gaussian-like shape centred at bin 50
        let n = 100;
        let mut spectrum = vec![0.0; n];
        for i in 0..n {
            let x = (i as f64 - 50.0) / 5.0;
            spectrum[i] = (-0.5 * x * x).exp();
        }

        let (area, centroid, fwhm) = spectral_line_extract(&spectrum, 50, 20);
        assert!(area > 0.0);
        assert!((centroid - 50.0).abs() < 1.0); // Centroid near bin 50
        // FWHM of a Gaussian with sigma=5 is ~11.77 bins
        assert!(fwhm > 8.0 && fwhm < 16.0);
    }

    #[test]
    fn test_spectral_line_extract_empty() {
        let (area, centroid, fwhm) = spectral_line_extract(&[], 0, 5);
        assert_eq!(area, 0.0);
        assert_eq!(centroid, 0.0);
        assert_eq!(fwhm, 0.0);
    }

    // --- antenna_temperature tests ------------------------------------------

    #[test]
    fn test_antenna_temperature_basic() {
        let k_b = 1.380649e-23;
        let gain = 1e6;
        let bw = 1e6;
        let power = k_b * gain * bw * 300.0; // Should give 300 K
        let t = antenna_temperature(power, gain, bw);
        assert!((t - 300.0).abs() < 1e-6);
    }

    #[test]
    fn test_antenna_temperature_zero_gain() {
        assert_eq!(antenna_temperature(1.0, 0.0, 1e6), 0.0);
    }

    // --- median helper test -------------------------------------------------

    #[test]
    fn test_median_of_odd() {
        let data = vec![3.0, 1.0, 2.0];
        assert!((median_of(&data) - 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_median_of_even() {
        let data = vec![4.0, 1.0, 3.0, 2.0];
        assert!((median_of(&data) - 2.5).abs() < EPSILON);
    }
}
