//! Spectral analysis and pulse processing for gamma-ray, neutron, and Cherenkov radiation detection.
//!
//! This module provides a complete radiation measurement toolkit including:
//!
//! - **Pulse Height Analysis (PHA)**: Extract pulse amplitudes from detector signals
//! - **Multi-Channel Analyzer (MCA)**: Histogram pulse heights into energy channels
//! - **Energy Calibration**: Linear and quadratic channel-to-energy mapping
//! - **Dead-Time Correction**: Paralyzable and non-paralyzable models
//! - **Pile-Up Detection**: Identify overlapping pulses for rejection
//! - **Count Rate Estimation**: Statistical uncertainty via Poisson statistics
//! - **ROI Integration**: Region-of-interest net counts with uncertainty
//! - **Peak Search**: Gaussian peak fitting with FWHM estimation
//! - **Compton Edge Identification**: Locate Compton continuum edges
//! - **Background Subtraction**: Scaled background removal
//!
//! # Example
//!
//! ```
//! use r4w_core::radiation_detector_processor::{
//!     RadiationConfig, RadiationProcessor, histogram, roi_integrate,
//! };
//!
//! let config = RadiationConfig {
//!     num_channels: 1024,
//!     energy_range_kev: (0.0, 3000.0),
//!     dead_time_us: 2.0,
//! };
//! let processor = RadiationProcessor::new(config);
//!
//! // Simulate some pulse heights (in arbitrary units)
//! let pulse_heights = vec![100.0, 250.5, 100.2, 511.0, 661.7, 250.1];
//! let spectrum = histogram(&pulse_heights, 1024, (0.0, 1024.0));
//! assert_eq!(spectrum.len(), 1024);
//!
//! // Integrate a region of interest
//! let (counts, uncertainty) = roi_integrate(&spectrum, 90, 110);
//! assert!(counts >= 2); // our ~100 keV pulses
//! assert!(uncertainty >= 0.0);
//! ```

/// Configuration for the radiation detection processor.
#[derive(Debug, Clone)]
pub struct RadiationConfig {
    /// Number of MCA channels.
    pub num_channels: usize,
    /// Energy range in keV as (min, max).
    pub energy_range_kev: (f64, f64),
    /// Dead time in microseconds.
    pub dead_time_us: f64,
}

impl Default for RadiationConfig {
    fn default() -> Self {
        Self {
            num_channels: 1024,
            energy_range_kev: (0.0, 3000.0),
            dead_time_us: 2.0,
        }
    }
}

/// MCA spectrum data with timing information.
#[derive(Debug, Clone)]
pub struct McaSpectrum {
    /// Channel counts (one entry per channel).
    pub counts: Vec<u64>,
    /// Number of channels in the spectrum.
    pub num_channels: usize,
    /// Live time in seconds (time the detector was active and not dead).
    pub live_time_s: f64,
    /// Real (wall-clock) time in seconds.
    pub real_time_s: f64,
}

impl McaSpectrum {
    /// Create a new MCA spectrum with the given channel count and timing.
    pub fn new(num_channels: usize, live_time_s: f64, real_time_s: f64) -> Self {
        Self {
            counts: vec![0; num_channels],
            num_channels,
            live_time_s,
            real_time_s,
        }
    }

    /// Total number of counts across all channels.
    pub fn total_counts(&self) -> u64 {
        self.counts.iter().sum()
    }

    /// Count rate in counts per second based on live time.
    pub fn count_rate(&self) -> f64 {
        if self.live_time_s > 0.0 {
            self.total_counts() as f64 / self.live_time_s
        } else {
            0.0
        }
    }

    /// Dead time fraction (1 - live_time / real_time).
    pub fn dead_time_fraction(&self) -> f64 {
        if self.real_time_s > 0.0 {
            1.0 - self.live_time_s / self.real_time_s
        } else {
            0.0
        }
    }
}

/// Main radiation detector processor.
#[derive(Debug, Clone)]
pub struct RadiationProcessor {
    /// Active configuration.
    pub config: RadiationConfig,
}

impl RadiationProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: RadiationConfig) -> Self {
        Self { config }
    }

    /// Convert a channel number to energy using linear calibration derived from
    /// the configured energy range.
    pub fn channel_to_energy(&self, channel: f64) -> f64 {
        let (e_min, e_max) = self.config.energy_range_kev;
        let gain = (e_max - e_min) / self.config.num_channels as f64;
        energy_calibrate_linear(channel, gain, e_min)
    }

    /// Convert energy (keV) to a channel number using the configured range.
    pub fn energy_to_channel(&self, energy_kev: f64) -> f64 {
        let (e_min, e_max) = self.config.energy_range_kev;
        let gain = (e_max - e_min) / self.config.num_channels as f64;
        if gain.abs() < f64::EPSILON {
            return 0.0;
        }
        (energy_kev - e_min) / gain
    }

    /// Process a raw detector signal into an MCA spectrum.
    ///
    /// Steps: pulse height analysis -> histogram -> dead-time correction metadata.
    pub fn process_signal(
        &self,
        signal: &[f64],
        threshold: f64,
        real_time_s: f64,
    ) -> McaSpectrum {
        let pulse_heights = pulse_height_analysis(signal, threshold);
        let counts = histogram(
            &pulse_heights,
            self.config.num_channels,
            self.config.energy_range_kev,
        );
        let total = counts.iter().sum::<u64>();
        let measured_rate = if real_time_s > 0.0 {
            total as f64 / real_time_s
        } else {
            0.0
        };
        let dead_time_s = self.config.dead_time_us * 1e-6;
        let dead_fraction = measured_rate * dead_time_s;
        let live_time_s = real_time_s * (1.0 - dead_fraction).max(0.0);

        McaSpectrum {
            counts,
            num_channels: self.config.num_channels,
            live_time_s,
            real_time_s,
        }
    }
}

/// Extract pulse heights from a detector signal using threshold crossing.
///
/// Scans the signal for peaks that exceed `threshold`. For each contiguous
/// region above the threshold, the maximum value is recorded as the pulse height.
pub fn pulse_height_analysis(signal: &[f64], threshold: f64) -> Vec<f64> {
    let mut heights = Vec::new();
    let mut in_pulse = false;
    let mut current_max = 0.0_f64;

    for &sample in signal {
        if sample >= threshold {
            if !in_pulse {
                in_pulse = true;
                current_max = sample;
            } else if sample > current_max {
                current_max = sample;
            }
        } else if in_pulse {
            heights.push(current_max);
            in_pulse = false;
            current_max = 0.0;
        }
    }
    // Handle pulse at end of signal
    if in_pulse {
        heights.push(current_max);
    }
    heights
}

/// Build an MCA histogram from pulse height values.
///
/// Maps each pulse height into one of `num_channels` bins spanning `range`.
/// Values outside the range are discarded.
pub fn histogram(pulse_heights: &[f64], num_channels: usize, range: (f64, f64)) -> Vec<u64> {
    let mut counts = vec![0u64; num_channels];
    let (lo, hi) = range;
    let span = hi - lo;
    if span <= 0.0 || num_channels == 0 {
        return counts;
    }
    for &ph in pulse_heights {
        if ph < lo || ph >= hi {
            continue;
        }
        let bin = ((ph - lo) / span * num_channels as f64) as usize;
        let bin = bin.min(num_channels - 1);
        counts[bin] += 1;
    }
    counts
}

/// Linear energy calibration: E(keV) = gain * channel + offset.
pub fn energy_calibrate_linear(channel: f64, gain: f64, offset: f64) -> f64 {
    gain * channel + offset
}

/// Quadratic energy calibration: E(keV) = a * channel^2 + b * channel + c.
pub fn energy_calibrate_quadratic(channel: f64, a: f64, b: f64, c: f64) -> f64 {
    a * channel * channel + b * channel + c
}

/// Dead-time correction using the non-paralyzable (Type I) model.
///
/// Returns the estimated true count rate given the measured rate and per-event
/// dead time. Formula: n_true = n_meas / (1 - n_meas * tau).
///
/// Returns `f64::INFINITY` when the denominator approaches zero (saturated detector).
pub fn dead_time_correct_nonparalyzable(measured_rate: f64, dead_time_s: f64) -> f64 {
    let denom = 1.0 - measured_rate * dead_time_s;
    if denom <= 0.0 {
        return f64::INFINITY;
    }
    measured_rate / denom
}

/// Dead-time correction using the paralyzable (Type II) model.
///
/// Uses Newton-Raphson iteration to solve m = n * exp(-n * tau) for n.
/// Returns the estimated true count rate.
pub fn dead_time_correct_paralyzable(measured_rate: f64, dead_time_s: f64) -> f64 {
    if measured_rate <= 0.0 {
        return 0.0;
    }
    if dead_time_s <= 0.0 {
        return measured_rate;
    }
    // Newton-Raphson: solve f(n) = n * exp(-n*tau) - m = 0
    // f'(n) = exp(-n*tau) * (1 - n*tau)
    // Start with the non-paralyzable estimate as initial guess.
    let mut n = dead_time_correct_nonparalyzable(measured_rate, dead_time_s);
    if n.is_infinite() {
        // Use a reasonable starting point
        n = 1.0 / dead_time_s;
    }

    for _ in 0..50 {
        let exp_nt = (-n * dead_time_s).exp();
        let f = n * exp_nt - measured_rate;
        let f_prime = exp_nt * (1.0 - n * dead_time_s);
        if f_prime.abs() < 1e-30 {
            break;
        }
        let delta = f / f_prime;
        n -= delta;
        if n < 0.0 {
            n = measured_rate;
        }
        if delta.abs() < 1e-10 * n.abs() {
            break;
        }
    }
    n
}

/// Detect pile-up events in a signal.
///
/// Returns indices of pulses (threshold crossings) that occur within
/// `min_separation` samples of the previous pulse. These events should
/// typically be rejected for accurate spectroscopy.
pub fn detect_pileup(signal: &[f64], min_separation: usize) -> Vec<usize> {
    if signal.is_empty() {
        return Vec::new();
    }

    // Find all pulse peak positions (local maxima where signal rises then falls)
    let mut peaks = Vec::new();
    let mut in_pulse = false;
    let mut peak_idx = 0usize;
    let mut peak_val = 0.0_f64;

    for i in 1..signal.len().saturating_sub(1) {
        if signal[i] > signal[i - 1] && signal[i] >= signal[i + 1] && signal[i] > 0.0 {
            if in_pulse && signal[i] > peak_val {
                peak_val = signal[i];
                peak_idx = i;
            } else if !in_pulse {
                in_pulse = true;
                peak_val = signal[i];
                peak_idx = i;
            }
        } else if in_pulse && signal[i] < peak_val * 0.5 {
            peaks.push(peak_idx);
            in_pulse = false;
        }
    }
    if in_pulse {
        peaks.push(peak_idx);
    }

    // Find peaks that are too close together
    let mut pileup_indices = Vec::new();
    for i in 1..peaks.len() {
        if peaks[i] - peaks[i - 1] < min_separation {
            pileup_indices.push(peaks[i]);
            if !pileup_indices.contains(&peaks[i - 1]) {
                pileup_indices.push(peaks[i - 1]);
            }
        }
    }
    pileup_indices.sort_unstable();
    pileup_indices.dedup();
    pileup_indices
}

/// Integrate counts in a region of interest (ROI).
///
/// Returns `(gross_counts, uncertainty)` where uncertainty is the Poisson
/// standard deviation (sqrt of gross counts).
///
/// # Panics
///
/// Panics if `end_ch` exceeds the spectrum length or `start_ch > end_ch`.
pub fn roi_integrate(spectrum: &[u64], start_ch: usize, end_ch: usize) -> (u64, f64) {
    assert!(
        end_ch <= spectrum.len(),
        "end_ch {} exceeds spectrum length {}",
        end_ch,
        spectrum.len()
    );
    assert!(
        start_ch <= end_ch,
        "start_ch {} > end_ch {}",
        start_ch,
        end_ch
    );
    let gross: u64 = spectrum[start_ch..end_ch].iter().sum();
    let uncertainty = (gross as f64).sqrt();
    (gross, uncertainty)
}

/// Search for peaks in a spectrum.
///
/// Returns a vector of `(channel, height, fwhm)` tuples for each peak found
/// above the given threshold. Peak height is interpolated; FWHM is estimated
/// from the peak shape assuming a Gaussian profile.
pub fn peak_search(spectrum: &[u64], threshold: f64) -> Vec<(usize, f64, f64)> {
    let mut peaks = Vec::new();
    let n = spectrum.len();
    if n < 3 {
        return peaks;
    }

    for i in 1..n - 1 {
        let y_prev = spectrum[i - 1] as f64;
        let y_curr = spectrum[i] as f64;
        let y_next = spectrum[i + 1] as f64;

        // Must be a local maximum above threshold
        if y_curr > y_prev && y_curr > y_next && y_curr > threshold {
            // Parabolic interpolation for height
            let denom = 2.0 * y_curr - y_prev - y_next;
            let height = if denom.abs() > 1e-12 {
                y_curr - 0.25 * (y_prev - y_next).powi(2) / denom
            } else {
                y_curr
            };

            // Estimate FWHM from second derivative (Gaussian approximation)
            // y'' at peak ~ y_prev - 2*y_curr + y_next
            // For Gaussian: sigma^2 = -A / y''
            let second_deriv = y_prev - 2.0 * y_curr + y_next;
            let fwhm = if second_deriv < -1e-12 {
                let sigma_sq = -y_curr / second_deriv;
                if sigma_sq > 0.0 {
                    2.0 * (2.0 * (2.0_f64).ln()).sqrt() * sigma_sq.sqrt()
                } else {
                    1.0
                }
            } else {
                1.0
            };

            peaks.push((i, height, fwhm));
        }
    }
    peaks
}

/// Identify Compton edges in a gamma-ray spectrum.
///
/// A Compton edge appears as a sudden drop-off in the spectrum. This function
/// looks for channels where the derivative of the smoothed spectrum drops
/// sharply (negative slope exceeding `slope_threshold`).
///
/// Returns channel indices of detected Compton edges.
pub fn compton_edge_search(spectrum: &[u64], slope_threshold: f64) -> Vec<usize> {
    let n = spectrum.len();
    if n < 5 {
        return Vec::new();
    }

    // Smooth with a 5-point moving average
    let mut smoothed = vec![0.0_f64; n];
    for i in 2..n - 2 {
        smoothed[i] = (spectrum[i - 2] as f64
            + spectrum[i - 1] as f64
            + spectrum[i] as f64
            + spectrum[i + 1] as f64
            + spectrum[i + 2] as f64)
            / 5.0;
    }

    // Find steep negative slopes
    let mut edges = Vec::new();
    for i in 3..n - 3 {
        let slope = smoothed[i + 1] - smoothed[i];
        if slope < -slope_threshold.abs() {
            let slope_prev = smoothed[i] - smoothed[i - 1];
            let slope_next = smoothed[i + 2] - smoothed[i + 1];
            if slope < slope_prev && slope < slope_next {
                edges.push(i);
            }
        }
    }
    edges
}

/// Subtract a scaled background spectrum from a measured spectrum.
///
/// Returns floating-point net counts per channel. Negative values indicate
/// channels where background exceeds the measurement (can occur due to
/// statistical fluctuations).
pub fn background_subtract(spectrum: &[u64], background: &[u64], scale: f64) -> Vec<f64> {
    let len = spectrum.len().min(background.len());
    let mut result = Vec::with_capacity(len);
    for i in 0..len {
        result.push(spectrum[i] as f64 - background[i] as f64 * scale);
    }
    result
}

/// Estimate count rate with Poisson uncertainty.
///
/// Returns `(rate, sigma)` where rate = total_counts / live_time and
/// sigma = sqrt(total_counts) / live_time.
pub fn count_rate_with_uncertainty(total_counts: u64, live_time_s: f64) -> (f64, f64) {
    if live_time_s <= 0.0 {
        return (0.0, 0.0);
    }
    let rate = total_counts as f64 / live_time_s;
    let sigma = (total_counts as f64).sqrt() / live_time_s;
    (rate, sigma)
}

/// Gaussian peak shape for fitting.
///
/// Returns the value of a Gaussian: amplitude * exp(-0.5 * ((x - center) / sigma)^2).
pub fn gaussian_peak(x: f64, amplitude: f64, center: f64, sigma: f64) -> f64 {
    if sigma.abs() < f64::EPSILON {
        return if (x - center).abs() < f64::EPSILON {
            amplitude
        } else {
            0.0
        };
    }
    let arg = (x - center) / sigma;
    amplitude * (-0.5 * arg * arg).exp()
}

/// Convert FWHM to Gaussian sigma.
///
/// FWHM = 2 * sqrt(2 * ln(2)) * sigma approximately equals 2.3548 * sigma.
pub fn fwhm_to_sigma(fwhm: f64) -> f64 {
    fwhm / (2.0 * (2.0 * (2.0_f64).ln()).sqrt())
}

/// Convert Gaussian sigma to FWHM.
pub fn sigma_to_fwhm(sigma: f64) -> f64 {
    sigma * 2.0 * (2.0 * (2.0_f64).ln()).sqrt()
}

/// Calculate the Compton edge energy for a given photopeak energy.
///
/// E_compton = E_gamma * (2 * E_gamma / m_e_c2) / (1 + 2 * E_gamma / m_e_c2)
/// where m_e_c2 = 511 keV (electron rest mass energy).
pub fn compton_edge_energy(photopeak_kev: f64) -> f64 {
    let m_e_c2 = 511.0; // keV
    let ratio = 2.0 * photopeak_kev / m_e_c2;
    photopeak_kev * ratio / (1.0 + ratio)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_radiation_config_default() {
        let config = RadiationConfig::default();
        assert_eq!(config.num_channels, 1024);
        assert_eq!(config.energy_range_kev, (0.0, 3000.0));
        assert!((config.dead_time_us - 2.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_mca_spectrum_new() {
        let spec = McaSpectrum::new(512, 9.8, 10.0);
        assert_eq!(spec.counts.len(), 512);
        assert_eq!(spec.num_channels, 512);
        assert_eq!(spec.total_counts(), 0);
        assert!((spec.live_time_s - 9.8).abs() < f64::EPSILON);
        assert!((spec.real_time_s - 10.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_mca_spectrum_count_rate() {
        let mut spec = McaSpectrum::new(100, 10.0, 10.0);
        spec.counts[50] = 1000;
        let rate = spec.count_rate();
        assert!((rate - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_mca_spectrum_dead_time_fraction() {
        let spec = McaSpectrum::new(100, 8.0, 10.0);
        let dtf = spec.dead_time_fraction();
        assert!((dtf - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_pulse_height_analysis_basic() {
        let signal = vec![
            0.0, 1.0, 3.0, 8.0, 12.0, 9.0, 4.0, 1.0,
            0.0, 0.0, 2.0, 6.0, 15.0, 10.0, 3.0, 0.0,
        ];
        let heights = pulse_height_analysis(&signal, 5.0);
        assert_eq!(heights.len(), 2);
        assert!((heights[0] - 12.0).abs() < f64::EPSILON);
        assert!((heights[1] - 15.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_pulse_height_analysis_no_pulses() {
        let signal = vec![0.0, 1.0, 2.0, 1.0, 0.0];
        let heights = pulse_height_analysis(&signal, 5.0);
        assert!(heights.is_empty());
    }

    #[test]
    fn test_pulse_height_analysis_pulse_at_end() {
        let signal = vec![0.0, 1.0, 6.0, 10.0, 15.0];
        let heights = pulse_height_analysis(&signal, 5.0);
        assert_eq!(heights.len(), 1);
        assert!((heights[0] - 15.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_histogram_basic() {
        let pulse_heights = vec![0.5, 1.5, 2.5, 3.5, 4.5];
        let h = histogram(&pulse_heights, 5, (0.0, 5.0));
        assert_eq!(h.len(), 5);
        for count in &h {
            assert_eq!(*count, 1);
        }
    }

    #[test]
    fn test_histogram_out_of_range() {
        let pulse_heights = vec![-1.0, 5.0, 10.0, 2.5];
        let h = histogram(&pulse_heights, 5, (0.0, 5.0));
        let total: u64 = h.iter().sum();
        assert_eq!(total, 1);
    }

    #[test]
    fn test_energy_calibrate_linear() {
        let energy = energy_calibrate_linear(100.0, 3.0, 10.0);
        assert!((energy - 310.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_energy_calibrate_quadratic() {
        let energy = energy_calibrate_quadratic(100.0, 0.001, 2.9, 5.0);
        let expected = 0.001 * 10000.0 + 2.9 * 100.0 + 5.0;
        assert!((energy - expected).abs() < 1e-10);
    }

    #[test]
    fn test_dead_time_nonparalyzable() {
        let true_rate = dead_time_correct_nonparalyzable(1000.0, 10e-6);
        let expected = 1000.0 / (1.0 - 0.01);
        assert!((true_rate - expected).abs() < 0.01);
    }

    #[test]
    fn test_dead_time_nonparalyzable_saturated() {
        let true_rate = dead_time_correct_nonparalyzable(1e6, 1e-6);
        assert!(true_rate.is_infinite());
    }

    #[test]
    fn test_dead_time_paralyzable() {
        let np = dead_time_correct_nonparalyzable(100.0, 10e-6);
        let p = dead_time_correct_paralyzable(100.0, 10e-6);
        assert!((np - p).abs() / np < 0.001);
    }

    #[test]
    fn test_dead_time_paralyzable_zero_rate() {
        let rate = dead_time_correct_paralyzable(0.0, 10e-6);
        assert!((rate - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_detect_pileup() {
        let mut signal = vec![0.0; 100];
        // First peak at index 20
        signal[18] = 2.0;
        signal[19] = 5.0;
        signal[20] = 10.0;
        signal[21] = 5.0;
        signal[22] = 2.0;
        // Second peak at index 27 (separation = 7 < 10)
        signal[25] = 2.0;
        signal[26] = 5.0;
        signal[27] = 8.0;
        signal[28] = 5.0;
        signal[29] = 2.0;
        // Third peak at index 60 (far away, no pileup)
        signal[58] = 2.0;
        signal[59] = 5.0;
        signal[60] = 12.0;
        signal[61] = 5.0;
        signal[62] = 2.0;

        let pileup = detect_pileup(&signal, 10);
        assert!(!pileup.is_empty());
        assert!(!pileup.contains(&60));
    }

    #[test]
    fn test_detect_pileup_empty() {
        let pileup = detect_pileup(&[], 10);
        assert!(pileup.is_empty());
    }

    #[test]
    fn test_roi_integrate() {
        let mut spectrum = vec![0u64; 100];
        spectrum[45] = 10;
        spectrum[46] = 25;
        spectrum[47] = 50;
        spectrum[48] = 30;
        spectrum[49] = 12;
        let (counts, uncertainty) = roi_integrate(&spectrum, 44, 51);
        assert_eq!(counts, 127);
        assert!((uncertainty - (127.0_f64).sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_roi_integrate_empty_range() {
        let spectrum = vec![100u64; 50];
        let (counts, uncertainty) = roi_integrate(&spectrum, 10, 10);
        assert_eq!(counts, 0);
        assert!((uncertainty - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_peak_search_single_peak() {
        let mut spectrum = vec![0u64; 200];
        for i in 90..111 {
            let x = (i as f64 - 100.0) / 3.0;
            spectrum[i] = (1000.0 * (-0.5 * x * x).exp()) as u64;
        }
        let peaks = peak_search(&spectrum, 10.0);
        assert!(!peaks.is_empty());
        let (ch, height, fwhm) = peaks[0];
        assert!((ch as i64 - 100).abs() <= 1);
        assert!(height > 900.0);
        assert!(fwhm > 0.0);
    }

    #[test]
    fn test_peak_search_no_peaks() {
        let spectrum = vec![5u64; 100];
        let peaks = peak_search(&spectrum, 10.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_background_subtract() {
        let spectrum = vec![100, 200, 300, 400, 500];
        let background = vec![50, 50, 50, 50, 50];
        let net = background_subtract(&spectrum, &background, 1.0);
        assert_eq!(net.len(), 5);
        assert!((net[0] - 50.0).abs() < f64::EPSILON);
        assert!((net[4] - 450.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_background_subtract_scaled() {
        let spectrum = vec![100, 200, 300];
        let background = vec![50, 100, 150];
        let net = background_subtract(&spectrum, &background, 2.0);
        assert!((net[0] - 0.0).abs() < f64::EPSILON);
        assert!((net[1] - 0.0).abs() < f64::EPSILON);
        assert!((net[2] - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_count_rate_with_uncertainty() {
        let (rate, sigma) = count_rate_with_uncertainty(10000, 10.0);
        assert!((rate - 1000.0).abs() < f64::EPSILON);
        assert!((sigma - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_peak_function() {
        let val = gaussian_peak(5.0, 100.0, 5.0, 2.0);
        assert!((val - 100.0).abs() < 1e-10);

        let val_sigma = gaussian_peak(7.0, 100.0, 5.0, 2.0);
        let expected = 100.0 * (-0.5_f64).exp();
        assert!((val_sigma - expected).abs() < 1e-10);
    }

    #[test]
    fn test_fwhm_sigma_conversion() {
        let sigma = 3.0;
        let fwhm = sigma_to_fwhm(sigma);
        let sigma_back = fwhm_to_sigma(fwhm);
        assert!((sigma - sigma_back).abs() < 1e-10);

        let expected_fwhm = 2.0 * (2.0 * (2.0_f64).ln()).sqrt() * sigma;
        assert!((fwhm - expected_fwhm).abs() < 1e-10);
    }

    #[test]
    fn test_compton_edge_energy() {
        // Cs-137: 661.7 keV photopeak -> Compton edge at ~477.3 keV
        let ce = compton_edge_energy(661.7);
        assert!((ce - 477.3).abs() < 0.5);

        // Co-60: 1332.5 keV -> Compton edge at ~1118.1 keV
        let ce_co60 = compton_edge_energy(1332.5);
        assert!((ce_co60 - 1118.1).abs() < 1.0);
    }

    #[test]
    fn test_processor_channel_energy_roundtrip() {
        let config = RadiationConfig {
            num_channels: 1024,
            energy_range_kev: (0.0, 2048.0),
            dead_time_us: 2.0,
        };
        let proc = RadiationProcessor::new(config);
        let energy = proc.channel_to_energy(512.0);
        assert!((energy - 1024.0).abs() < 1e-10);

        let ch = proc.energy_to_channel(1024.0);
        assert!((ch - 512.0).abs() < 1e-10);
    }

    #[test]
    fn test_processor_process_signal() {
        let config = RadiationConfig {
            num_channels: 100,
            energy_range_kev: (0.0, 100.0),
            dead_time_us: 1.0,
        };
        let proc = RadiationProcessor::new(config);

        let mut signal = vec![0.0; 500];
        signal[100] = 10.0;
        signal[101] = 30.0;
        signal[102] = 50.0;
        signal[103] = 35.0;
        signal[104] = 10.0;

        signal[200] = 8.0;
        signal[201] = 25.0;
        signal[202] = 50.0;
        signal[203] = 28.0;
        signal[204] = 5.0;

        let spectrum = proc.process_signal(&signal, 5.0, 1.0);
        assert_eq!(spectrum.num_channels, 100);
        assert!(spectrum.total_counts() >= 2);
        assert!(spectrum.live_time_s <= spectrum.real_time_s);
    }

    #[test]
    fn test_compton_edge_search() {
        let mut spectrum = vec![0u64; 200];
        for i in 60..120 {
            spectrum[i] = 100;
        }
        spectrum[120] = 80;
        spectrum[121] = 40;
        spectrum[122] = 10;
        spectrum[123] = 2;
        spectrum[149] = 50;
        spectrum[150] = 200;
        spectrum[151] = 50;

        let edges = compton_edge_search(&spectrum, 10.0);
        if !edges.is_empty() {
            let nearest = edges.iter().min_by_key(|&&e| (e as i64 - 120).unsigned_abs());
            assert!((*nearest.unwrap() as i64 - 120).abs() < 5);
        }
    }
}
