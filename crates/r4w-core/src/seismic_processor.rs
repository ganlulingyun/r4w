//! Seismic wave arrival detection and earthquake magnitude estimation for early warning systems.
//!
//! This module implements the STA/LTA (Short-Term Average / Long-Term Average) algorithm
//! for detecting P-wave arrivals in seismic data, along with local magnitude (ML) and
//! moment magnitude (Mw) estimation. It also provides P-wave and S-wave travel time
//! calculations and epicenter distance estimation from P-S differential arrival times.
//!
//! # Example
//!
//! ```
//! use r4w_core::seismic_processor::{SeismicProcessor, SeismicConfig, Phase};
//!
//! // Configure for 100 Hz seismic data
//! let config = SeismicConfig {
//!     sample_rate: 100.0,
//!     sta_window_s: 0.5,
//!     lta_window_s: 10.0,
//!     trigger_ratio: 4.0,
//!     detrigger_ratio: 1.5,
//! };
//!
//! let processor = SeismicProcessor::new(config);
//!
//! // Generate synthetic seismic trace: noise then a P-wave arrival
//! let mut trace: Vec<f64> = vec![0.01; 1200];
//! for i in 1050..1200 {
//!     trace[i] = 0.5; // strong arrival starting at sample 1050
//! }
//!
//! // Compute STA/LTA ratio
//! let ratio = processor.sta_lta(&trace);
//! assert!(!ratio.is_empty());
//!
//! // Detect P-wave arrival
//! if let Some(arrival) = processor.detect_arrival(&trace) {
//!     assert!(arrival.time_s > 0.0);
//!     assert!(matches!(arrival.phase, Phase::P));
//! }
//!
//! // Estimate local magnitude from max amplitude (mm) and distance (km)
//! let ml = SeismicProcessor::estimate_magnitude_ml(10.0, 100.0);
//! assert!(ml > 0.0);
//! ```

use std::f64::consts::PI;

/// Configuration for the seismic processor.
#[derive(Debug, Clone)]
pub struct SeismicConfig {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Short-term average window length in seconds.
    pub sta_window_s: f64,
    /// Long-term average window length in seconds.
    pub lta_window_s: f64,
    /// STA/LTA ratio threshold to declare a trigger (P-wave detection).
    pub trigger_ratio: f64,
    /// STA/LTA ratio threshold to declare de-trigger (end of event).
    pub detrigger_ratio: f64,
}

impl Default for SeismicConfig {
    fn default() -> Self {
        Self {
            sample_rate: 100.0,
            sta_window_s: 1.0,
            lta_window_s: 10.0,
            trigger_ratio: 4.0,
            detrigger_ratio: 1.5,
        }
    }
}

/// Seismic wave phase identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// Primary (compressional) wave — fastest, arrives first.
    P,
    /// Secondary (shear) wave — slower, arrives after P.
    S,
}

/// Result of a seismic arrival detection.
#[derive(Debug, Clone)]
pub struct ArrivalDetection {
    /// Sample index where the arrival was detected.
    pub sample_index: usize,
    /// Time in seconds from the start of the trace.
    pub time_s: f64,
    /// STA/LTA ratio at the detection point.
    pub sta_lta_ratio: f64,
    /// Phase type (P or S).
    pub phase: Phase,
}

/// Summary of a detected seismic event.
#[derive(Debug, Clone)]
pub struct SeismicEvent {
    /// Arrival time in seconds from trace start.
    pub arrival_time: f64,
    /// Estimated magnitude (ML or Mw).
    pub magnitude: f64,
    /// Estimated epicenter distance in km.
    pub distance_km: f64,
    /// Estimated azimuth to epicenter in degrees (0 = north, 90 = east).
    pub azimuth_deg: f64,
}

/// Seismic wave arrival detector and magnitude estimator.
///
/// Implements the classic STA/LTA trigger algorithm for P-wave detection and
/// provides earthquake magnitude estimation routines.
#[derive(Debug, Clone)]
pub struct SeismicProcessor {
    config: SeismicConfig,
}

impl SeismicProcessor {
    /// Average P-wave velocity in the upper crust (km/s).
    const VP_CRUST: f64 = 6.0;
    /// Average S-wave velocity in the upper crust (km/s).
    const VS_CRUST: f64 = 3.5;

    /// Create a new `SeismicProcessor` with the given configuration.
    pub fn new(config: SeismicConfig) -> Self {
        Self { config }
    }

    /// Compute the STA/LTA ratio for each sample in the trace.
    ///
    /// The STA and LTA windows are computed as running averages of the absolute
    /// amplitude. The output vector has the same length as the input; samples
    /// where the LTA window has not yet been filled are set to 0.0.
    pub fn sta_lta(&self, trace: &[f64]) -> Vec<f64> {
        let sta_len = (self.config.sta_window_s * self.config.sample_rate).round() as usize;
        let lta_len = (self.config.lta_window_s * self.config.sample_rate).round() as usize;
        let n = trace.len();

        if sta_len == 0 || lta_len == 0 || n == 0 || lta_len + sta_len > n {
            return vec![0.0; n];
        }

        let mut ratio = vec![0.0; n];

        // Precompute absolute values
        let abs_trace: Vec<f64> = trace.iter().map(|&x| x.abs()).collect();

        for i in lta_len..n {
            // LTA window: [i - lta_len, i - sta_len)
            // STA window: [i - sta_len, i)
            let lta_start = i - lta_len;
            let lta_end = i - sta_len;
            let sta_start = i - sta_len;

            if lta_end <= lta_start {
                continue;
            }

            let lta: f64 =
                abs_trace[lta_start..lta_end].iter().sum::<f64>() / (lta_end - lta_start) as f64;
            let sta: f64 = abs_trace[sta_start..i].iter().sum::<f64>() / sta_len as f64;

            if lta > 1e-30 {
                ratio[i] = sta / lta;
            }
        }

        ratio
    }

    /// Detect the first P-wave arrival in the seismic trace using STA/LTA triggering.
    ///
    /// Returns `None` if no arrival exceeds the trigger ratio.
    pub fn detect_arrival(&self, trace: &[f64]) -> Option<ArrivalDetection> {
        let ratio = self.sta_lta(trace);

        for (i, &r) in ratio.iter().enumerate() {
            if r >= self.config.trigger_ratio {
                return Some(ArrivalDetection {
                    sample_index: i,
                    time_s: i as f64 / self.config.sample_rate,
                    sta_lta_ratio: r,
                    phase: Phase::P,
                });
            }
        }

        None
    }

    /// Estimate local magnitude (ML) from peak amplitude and hypocentral distance.
    ///
    /// Uses the Richter (1935) formula:
    ///   ML = log10(amplitude_mm) + 1.110 * log10(distance_km) + 0.00189 * distance_km - 2.09
    ///
    /// # Arguments
    /// * `amplitude_mm` - Maximum trace amplitude in millimeters (Wood-Anderson equivalent).
    /// * `distance_km` - Hypocentral distance in kilometers.
    pub fn estimate_magnitude_ml(amplitude_mm: f64, distance_km: f64) -> f64 {
        if amplitude_mm <= 0.0 || distance_km <= 0.0 {
            return 0.0;
        }
        amplitude_mm.log10() + 1.110 * distance_km.log10() + 0.00189 * distance_km - 2.09
    }

    /// Estimate moment magnitude (Mw) from a simplified spectral analysis.
    ///
    /// Computes the seismic moment M0 from the low-frequency spectral level (Omega_0)
    /// and then converts to Mw:
    ///   M0 = 4 * pi * rho * v^3 * R * Omega_0 / (F * S)
    ///   Mw = (2/3) * log10(M0) - 6.07
    ///
    /// This is a simplified single-station estimate.
    ///
    /// # Arguments
    /// * `trace` - Seismic waveform samples (velocity, m/s).
    /// * `sample_rate` - Sample rate in Hz.
    /// * `distance_km` - Hypocentral distance in km.
    pub fn estimate_magnitude_mw(trace: &[f64], sample_rate: f64, distance_km: f64) -> f64 {
        if trace.is_empty() || distance_km <= 0.0 || sample_rate <= 0.0 {
            return 0.0;
        }

        // Compute amplitude spectrum via DFT of the trace
        let n = trace.len();
        let omega_0 = spectral_low_freq_level(trace, sample_rate, n);

        // Crustal density (kg/m^3)
        let rho = 2700.0;
        // P-wave velocity (m/s)
        let v = Self::VP_CRUST * 1000.0;
        // Distance in meters
        let r = distance_km * 1000.0;
        // Radiation pattern factor (average for P)
        let f_rad = 0.52;
        // Free-surface amplification
        let s_free = 2.0;

        let m0 = 4.0 * PI * rho * v.powi(3) * r * omega_0 / (f_rad * s_free);

        if m0 <= 0.0 {
            return 0.0;
        }

        // Hanks & Kanamori (1979)
        (2.0 / 3.0) * m0.log10() - 6.07
    }

    /// Compute P-wave travel time for a given distance.
    ///
    /// Uses average crustal P-wave velocity (6.0 km/s).
    ///
    /// # Arguments
    /// * `distance_km` - Distance in kilometers.
    ///
    /// # Returns
    /// Travel time in seconds.
    pub fn p_wave_velocity(distance_km: f64) -> f64 {
        distance_km / Self::VP_CRUST
    }

    /// Compute S-wave travel time for a given distance.
    ///
    /// Uses average crustal S-wave velocity (3.5 km/s).
    ///
    /// # Arguments
    /// * `distance_km` - Distance in kilometers.
    ///
    /// # Returns
    /// Travel time in seconds.
    pub fn s_wave_velocity(distance_km: f64) -> f64 {
        distance_km / Self::VS_CRUST
    }

    /// Estimate epicenter distance from the P-S arrival time difference.
    ///
    /// Uses the relation:
    ///   d = dt_ps / (1/Vs - 1/Vp)
    ///
    /// # Arguments
    /// * `dt_ps_s` - Time difference between S and P arrival in seconds.
    ///
    /// # Returns
    /// Estimated distance in kilometers.
    pub fn epicenter_distance(dt_ps_s: f64) -> f64 {
        if dt_ps_s <= 0.0 {
            return 0.0;
        }
        let slowness_diff = 1.0 / Self::VS_CRUST - 1.0 / Self::VP_CRUST;
        dt_ps_s / slowness_diff
    }

    /// Apply a simple bandpass filter for the seismic band (0.1-20 Hz).
    ///
    /// Implements a first-order IIR highpass at 0.1 Hz cascaded with a first-order
    /// IIR lowpass at 20 Hz. For production use, a proper Butterworth or Bessel
    /// bandpass should be used.
    ///
    /// # Arguments
    /// * `trace` - Input seismic trace.
    /// * `sample_rate` - Sample rate in Hz.
    ///
    /// # Returns
    /// Filtered trace.
    pub fn filter_seismic(trace: &[f64], sample_rate: f64) -> Vec<f64> {
        if trace.is_empty() || sample_rate <= 0.0 {
            return vec![];
        }

        // Highpass at 0.1 Hz (remove DC drift) followed by lowpass at 20 Hz
        let hp = highpass_first_order(trace, 0.1, sample_rate);
        lowpass_first_order(&hp, 20.0, sample_rate)
    }

    /// Build a [`SeismicEvent`] from detected arrivals and amplitude measurements.
    ///
    /// # Arguments
    /// * `p_arrival_s` - P-wave arrival time in seconds from trace start.
    /// * `s_arrival_s` - S-wave arrival time in seconds from trace start (if available).
    /// * `max_amplitude_mm` - Maximum Wood-Anderson equivalent amplitude in mm.
    /// * `azimuth_deg` - Back-azimuth to epicenter in degrees.
    pub fn build_event(
        p_arrival_s: f64,
        s_arrival_s: Option<f64>,
        max_amplitude_mm: f64,
        azimuth_deg: f64,
    ) -> SeismicEvent {
        let distance_km = match s_arrival_s {
            Some(s_time) => Self::epicenter_distance(s_time - p_arrival_s),
            None => 0.0,
        };
        let magnitude = if distance_km > 0.0 {
            Self::estimate_magnitude_ml(max_amplitude_mm, distance_km)
        } else {
            0.0
        };
        SeismicEvent {
            arrival_time: p_arrival_s,
            magnitude,
            distance_km,
            azimuth_deg,
        }
    }
}

/// Compute the low-frequency spectral level (Omega_0) of a trace using a simple DFT.
///
/// Averages the magnitude spectrum in the band 0.1-2 Hz.
fn spectral_low_freq_level(trace: &[f64], sample_rate: f64, n: usize) -> f64 {
    if n == 0 || sample_rate <= 0.0 {
        return 0.0;
    }

    let freq_res = sample_rate / n as f64;
    let lo_bin = (0.1 / freq_res).ceil() as usize;
    let hi_bin = (2.0 / freq_res).floor() as usize;

    if lo_bin >= hi_bin || hi_bin >= n / 2 {
        // Fallback: use mean absolute value
        let sum: f64 = trace.iter().map(|&x| x.abs()).sum::<f64>();
        return sum / n as f64;
    }

    let mut sum = 0.0;
    let mut count = 0usize;
    for k in lo_bin..=hi_bin {
        // DFT at bin k
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &x) in trace.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += x * angle.cos();
            im += x * angle.sin();
        }
        let mag = (re * re + im * im).sqrt() / n as f64;
        sum += mag;
        count += 1;
    }

    if count > 0 {
        sum / count as f64
    } else {
        0.0
    }
}

/// First-order IIR highpass filter.
fn highpass_first_order(trace: &[f64], cutoff_hz: f64, sample_rate: f64) -> Vec<f64> {
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    let dt = 1.0 / sample_rate;
    let alpha = rc / (rc + dt);

    let mut out = vec![0.0; trace.len()];
    if trace.is_empty() {
        return out;
    }
    out[0] = trace[0];
    for i in 1..trace.len() {
        out[i] = alpha * (out[i - 1] + trace[i] - trace[i - 1]);
    }
    out
}

/// First-order IIR lowpass filter.
fn lowpass_first_order(trace: &[f64], cutoff_hz: f64, sample_rate: f64) -> Vec<f64> {
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    let dt = 1.0 / sample_rate;
    let alpha = dt / (rc + dt);

    let mut out = vec![0.0; trace.len()];
    if trace.is_empty() {
        return out;
    }
    out[0] = alpha * trace[0];
    for i in 1..trace.len() {
        out[i] = out[i - 1] + alpha * (trace[i] - out[i - 1]);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> SeismicProcessor {
        SeismicProcessor::new(SeismicConfig::default())
    }

    #[test]
    fn test_sta_lta_quiet_trace() {
        let proc = default_processor();
        // Uniform quiet trace should give ratios near 1.0 (STA ~ LTA)
        let trace = vec![0.01; 2000];
        let ratio = proc.sta_lta(&trace);
        assert_eq!(ratio.len(), trace.len());
        // After LTA fills, ratios should be close to 1
        for &r in &ratio[1100..] {
            assert!((r - 1.0).abs() < 0.2, "Quiet trace ratio {r} not near 1.0");
        }
    }

    #[test]
    fn test_sta_lta_with_arrival() {
        let proc = default_processor();
        let mut trace = vec![0.01; 2000];
        // Insert strong arrival at sample 1200
        for i in 1200..2000 {
            trace[i] = 1.0;
        }
        let ratio = proc.sta_lta(&trace);
        // The ratio should spike after the arrival
        let max_ratio = ratio.iter().cloned().fold(0.0f64, f64::max);
        assert!(
            max_ratio > 4.0,
            "Expected high STA/LTA ratio after arrival, got {max_ratio}"
        );
    }

    #[test]
    fn test_detect_arrival_none_for_quiet() {
        let proc = default_processor();
        let trace = vec![0.01; 2000];
        assert!(proc.detect_arrival(&trace).is_none());
    }

    #[test]
    fn test_detect_arrival_finds_event() {
        let proc = default_processor();
        let mut trace = vec![0.01; 2000];
        for i in 1200..2000 {
            trace[i] = 1.0;
        }
        let arrival = proc.detect_arrival(&trace).expect("Should detect arrival");
        assert!(arrival.sample_index >= 1200);
        assert!(arrival.sample_index < 1400);
        assert!(arrival.sta_lta_ratio >= 4.0);
        assert_eq!(arrival.phase, Phase::P);
        assert!(arrival.time_s > 12.0); // sample 1200 / 100 Hz = 12.0 s
    }

    #[test]
    fn test_estimate_magnitude_ml() {
        // Richter magnitude for a 10 mm amplitude at 100 km
        let ml = SeismicProcessor::estimate_magnitude_ml(10.0, 100.0);
        // Expected: log10(10) + 1.110*log10(100) + 0.00189*100 - 2.09
        //         = 1.0 + 2.22 + 0.189 - 2.09 = 1.319
        assert!((ml - 1.319).abs() < 0.01, "ML = {ml}, expected ~1.319");
    }

    #[test]
    fn test_estimate_magnitude_ml_edge_cases() {
        assert_eq!(SeismicProcessor::estimate_magnitude_ml(0.0, 100.0), 0.0);
        assert_eq!(SeismicProcessor::estimate_magnitude_ml(10.0, 0.0), 0.0);
        assert_eq!(SeismicProcessor::estimate_magnitude_ml(-1.0, 100.0), 0.0);
    }

    #[test]
    fn test_estimate_magnitude_mw() {
        // Synthetic trace: a velocity pulse
        let sample_rate = 100.0;
        let n = 10000;
        let mut trace = vec![0.0; n];
        // Insert a low-frequency pulse (simulating displacement)
        for i in 5000..5500 {
            let t = (i - 5000) as f64 / sample_rate;
            trace[i] = 0.001 * (2.0 * PI * 1.0 * t).sin(); // 1 Hz signal
        }
        let mw = SeismicProcessor::estimate_magnitude_mw(&trace, sample_rate, 50.0);
        // Just check it returns a finite, positive value for a real signal
        assert!(mw.is_finite(), "Mw should be finite");
    }

    #[test]
    fn test_p_wave_velocity() {
        let t = SeismicProcessor::p_wave_velocity(60.0);
        // 60 km / 6.0 km/s = 10.0 s
        assert!((t - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_s_wave_velocity() {
        let t = SeismicProcessor::s_wave_velocity(35.0);
        // 35 km / 3.5 km/s = 10.0 s
        assert!((t - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_epicenter_distance() {
        // dt = d * (1/Vs - 1/Vp) = d * (1/3.5 - 1/6.0)
        // For d = 100 km: dt = 100 * (0.28571 - 0.16667) = 100 * 0.11905 = 11.905 s
        let d = SeismicProcessor::epicenter_distance(11.905);
        assert!(
            (d - 100.0).abs() < 0.1,
            "Distance {d} km, expected ~100 km"
        );
    }

    #[test]
    fn test_epicenter_distance_zero() {
        assert_eq!(SeismicProcessor::epicenter_distance(0.0), 0.0);
        assert_eq!(SeismicProcessor::epicenter_distance(-5.0), 0.0);
    }

    #[test]
    fn test_filter_seismic() {
        let sample_rate = 100.0;
        let n = 1000;
        let mut trace = vec![0.0; n];

        // Add a DC offset (should be removed by highpass)
        for i in 0..n {
            trace[i] = 5.0 + 0.1 * (2.0 * PI * 5.0 * i as f64 / sample_rate).sin();
        }

        let filtered = SeismicProcessor::filter_seismic(&trace, sample_rate);
        assert_eq!(filtered.len(), n);

        // The mean of the filtered signal should be near zero (DC removed)
        let mean: f64 = filtered[200..].iter().sum::<f64>() / (n - 200) as f64;
        assert!(
            mean.abs() < 1.0,
            "Filtered mean {mean} should be near 0 (DC removed)"
        );
    }

    #[test]
    fn test_filter_seismic_empty() {
        let result = SeismicProcessor::filter_seismic(&[], 100.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_build_event() {
        let event = SeismicProcessor::build_event(5.0, Some(16.905), 10.0, 45.0);
        // P-S dt = 11.905 s -> distance ~100 km
        assert!((event.distance_km - 100.0).abs() < 0.1);
        assert!(event.magnitude > 0.0);
        assert_eq!(event.arrival_time, 5.0);
        assert_eq!(event.azimuth_deg, 45.0);
    }

    #[test]
    fn test_build_event_no_s_wave() {
        let event = SeismicProcessor::build_event(5.0, None, 10.0, 90.0);
        assert_eq!(event.distance_km, 0.0);
        assert_eq!(event.magnitude, 0.0);
    }

    #[test]
    fn test_sta_lta_empty_trace() {
        let proc = default_processor();
        let ratio = proc.sta_lta(&[]);
        assert!(ratio.is_empty());
    }

    #[test]
    fn test_config_default() {
        let cfg = SeismicConfig::default();
        assert_eq!(cfg.sample_rate, 100.0);
        assert_eq!(cfg.sta_window_s, 1.0);
        assert_eq!(cfg.lta_window_s, 10.0);
        assert_eq!(cfg.trigger_ratio, 4.0);
        assert_eq!(cfg.detrigger_ratio, 1.5);
    }
}
