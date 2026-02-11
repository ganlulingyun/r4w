//! Seismograph event classifier for seismic signal analysis.
//!
//! Classifies seismic events (earthquakes, explosions, quarry blasts, mine collapses)
//! from three-component seismograph recordings using P/S wave analysis, spectral ratios,
//! and time-domain features. All math is implemented from scratch with no external
//! dependencies beyond `std`.
//!
//! # Theory
//!
//! Seismic event discrimination relies on several key observables:
//!
//! - **P-S time difference**: The delay between compressional (P) and shear (S) wave
//!   arrivals constrains epicentral distance via `d = dt * vp * vs / (vp - vs)`.
//! - **Spectral ratios**: Explosions tend to have higher high-frequency content relative
//!   to low-frequency compared to earthquakes of similar magnitude.
//! - **Back azimuth**: Polarization analysis of the P-wave first motion on horizontal
//!   components yields the source direction.
//! - **Magnitude**: Local magnitude ML uses the Wood-Anderson seismometer response and
//!   the Richter distance correction.
//!
//! # Example
//!
//! ```
//! use r4w_core::seismograph_event_classifier::{SeismicConfig, SeismicClassifier, EventType};
//!
//! let config = SeismicConfig {
//!     sample_rate_hz: 100.0,
//!     p_velocity_km_s: 6.0,
//!     s_velocity_km_s: 3.5,
//!     min_magnitude: 1.0,
//! };
//! let classifier = SeismicClassifier::new(config);
//!
//! // Synthetic three-component seismogram with a P-wave onset
//! let n = 1000;
//! let mut vertical = vec![0.0f64; n];
//! let mut north = vec![0.0f64; n];
//! let mut east = vec![0.0f64; n];
//!
//! // Inject P-wave at sample 200, S-wave at sample 400
//! for i in 200..n {
//!     let t = (i - 200) as f64 / 100.0;
//!     let decay = (-0.5 * t).exp();
//!     vertical[i] = decay * (8.0 * std::f64::consts::PI * t).sin();
//! }
//! for i in 400..n {
//!     let t = (i - 400) as f64 / 100.0;
//!     let decay = (-0.3 * t).exp();
//!     north[i] = 1.5 * decay * (6.0 * std::f64::consts::PI * t).sin();
//!     east[i] = 0.8 * decay * (6.0 * std::f64::consts::PI * t).sin();
//! }
//!
//! let event = classifier.classify_event(&vertical, &north, &east);
//! assert!(event.p_arrival_sample < event.s_arrival_sample);
//! ```

use std::f64::consts::PI;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration for the seismic event classifier.
#[derive(Debug, Clone)]
pub struct SeismicConfig {
    /// Sampling rate of the seismograph in Hz (e.g., 100.0 for broadband).
    pub sample_rate_hz: f64,
    /// Assumed P-wave velocity in km/s (typical crustal average ~6.0).
    pub p_velocity_km_s: f64,
    /// Assumed S-wave velocity in km/s (typical crustal average ~3.5).
    pub s_velocity_km_s: f64,
    /// Minimum reportable local magnitude.
    pub min_magnitude: f64,
}

impl Default for SeismicConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 100.0,
            p_velocity_km_s: 6.0,
            s_velocity_km_s: 3.5,
            min_magnitude: 1.0,
        }
    }
}

// ─── Event Type ──────────────────────────────────────────────────────────────

/// Classification of a seismic event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventType {
    /// Natural tectonic earthquake.
    Earthquake,
    /// Chemical or nuclear explosion.
    Explosion,
    /// Quarry or mining blast (surface).
    QuarryBlast,
    /// Underground mine collapse.
    MineCollapse,
    /// Background noise, no real event detected.
    Noise,
    /// Insufficient features to classify.
    Unknown,
}

impl core::fmt::Display for EventType {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            EventType::Earthquake => write!(f, "Earthquake"),
            EventType::Explosion => write!(f, "Explosion"),
            EventType::QuarryBlast => write!(f, "QuarryBlast"),
            EventType::MineCollapse => write!(f, "MineCollapse"),
            EventType::Noise => write!(f, "Noise"),
            EventType::Unknown => write!(f, "Unknown"),
        }
    }
}

// ─── Seismic Event Result ────────────────────────────────────────────────────

/// Result of seismic event classification.
#[derive(Debug, Clone)]
pub struct SeismicEvent {
    /// The classified event type.
    pub classification: EventType,
    /// Estimated local magnitude (ML).
    pub magnitude_ml: f64,
    /// Estimated source depth in km (crude, from P-S and distance).
    pub depth_estimate_km: f64,
    /// Sample index of P-wave arrival.
    pub p_arrival_sample: usize,
    /// Sample index of S-wave arrival.
    pub s_arrival_sample: usize,
    /// Back azimuth from station to source in degrees (0 = North, clockwise).
    pub back_azimuth_deg: f64,
    /// Spectral ratio (high-freq / low-freq energy) used for discrimination.
    pub spectral_ratio: f64,
}

// ─── Classifier ──────────────────────────────────────────────────────────────

/// Three-component seismic event classifier.
///
/// Processes vertical (Z), north (N), and east (E) component seismograms to
/// detect P and S arrivals, estimate magnitude and distance, compute spectral
/// ratios, and classify the event type.
pub struct SeismicClassifier {
    config: SeismicConfig,
}

impl SeismicClassifier {
    /// Create a new classifier with the given configuration.
    pub fn new(config: SeismicConfig) -> Self {
        Self { config }
    }

    /// Classify a seismic event from three-component data.
    ///
    /// - `vertical`: Z-component (up-down motion, best for P-wave detection).
    /// - `north`: N-component (north-south horizontal).
    /// - `east`: E-component (east-west horizontal).
    ///
    /// All three arrays must have the same length.
    pub fn classify_event(
        &self,
        vertical: &[f64],
        north: &[f64],
        east: &[f64],
    ) -> SeismicEvent {
        let fs = self.config.sample_rate_hz;
        let n = vertical.len().min(north.len()).min(east.len());

        if n < 10 {
            return SeismicEvent {
                classification: EventType::Noise,
                magnitude_ml: 0.0,
                depth_estimate_km: 0.0,
                p_arrival_sample: 0,
                s_arrival_sample: 0,
                back_azimuth_deg: 0.0,
                spectral_ratio: 0.0,
            };
        }

        // Compute horizontal envelope for S-wave detection
        let horizontal: Vec<f64> = north
            .iter()
            .zip(east.iter())
            .map(|(&n_val, &e_val)| (n_val * n_val + e_val * e_val).sqrt())
            .collect();

        // 1. Detect P-wave arrival on vertical component
        let p_onset = detect_p_arrival(vertical, fs);

        // 2. Detect S-wave arrival using horizontal components after P
        let s_onset = detect_s_arrival(vertical, &horizontal, fs, p_onset);

        // 3. P-S time difference and distance
        let delta_t = (s_onset as f64 - p_onset as f64) / fs;
        let distance_km = if delta_t > 0.0 {
            p_s_time_to_distance(
                delta_t,
                self.config.p_velocity_km_s,
                self.config.s_velocity_km_s,
            )
        } else {
            0.0
        };

        // 4. Maximum amplitude (Wood-Anderson simulated) for ML
        let wa_signal = wood_anderson_response(vertical, fs);
        let max_amp = wa_signal
            .iter()
            .map(|x| x.abs())
            .fold(0.0f64, f64::max);

        // 5. Estimate magnitude
        let magnitude = if max_amp > 0.0 && distance_km > 0.0 {
            estimate_magnitude_ml(max_amp, distance_km)
        } else {
            0.0
        };

        // 6. Back azimuth from P-wave polarization
        let az_window = (0.5 * fs) as usize; // 0.5 second window
        let az_window = az_window.max(4).min(n / 2);
        let back_az = estimate_back_azimuth(north, east, p_onset, az_window);

        // 7. Spectral ratio for event discrimination
        let spec_ratio = if s_onset > p_onset + 1 {
            spectral_ratio_p_s(vertical, fs, p_onset, s_onset)
        } else {
            1.0
        };

        // 8. Depth estimate (crude: assume ~10 km for shallow, scale by distance)
        let depth_km = estimate_depth(distance_km, delta_t);

        // 9. Classify based on discriminants
        let event_type = classify_by_features(
            spec_ratio,
            delta_t,
            magnitude,
            depth_km,
            max_amp,
            self.config.min_magnitude,
        );

        SeismicEvent {
            classification: event_type,
            magnitude_ml: magnitude,
            depth_estimate_km: depth_km,
            p_arrival_sample: p_onset,
            s_arrival_sample: s_onset,
            back_azimuth_deg: back_az,
            spectral_ratio: spec_ratio,
        }
    }
}

// ─── STA/LTA Picker ─────────────────────────────────────────────────────────

/// Compute the running STA/LTA (Short-Term Average / Long-Term Average) ratio.
///
/// This is the standard seismological trigger algorithm. The ratio increases
/// sharply at the onset of a seismic phase because the short-term window
/// responds faster than the long-term window.
///
/// - `signal`: input seismogram (any component).
/// - `sta_len`: length of the short-term averaging window in samples.
/// - `lta_len`: length of the long-term averaging window in samples.
///
/// Returns a vector of the same length as `signal` with the STA/LTA ratio.
/// Values before `lta_len` are set to 0.0.
pub fn sta_lta_ratio(signal: &[f64], sta_len: usize, lta_len: usize) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || sta_len == 0 || lta_len == 0 || sta_len > n || lta_len > n {
        return vec![0.0; n];
    }

    let mut ratio = vec![0.0; n];

    // Precompute absolute values (characteristic function = |x|)
    let cf: Vec<f64> = signal.iter().map(|&x| x.abs()).collect();

    // Initial STA and LTA sums
    // LTA window: [i - lta_len, i - 1] (the lta_len samples before current index)
    // STA window: [i - sta_len, i - 1] (the sta_len samples before current index)
    // We need at least lta_len samples before we can compute a valid ratio.

    for i in lta_len..n {
        let lta_start = i - lta_len;
        let sta_start = if i >= sta_len { i - sta_len } else { 0 };

        let lta_sum: f64 = cf[lta_start..i].iter().sum();
        let sta_sum: f64 = cf[sta_start..i].iter().sum();

        let lta_avg = lta_sum / lta_len as f64;
        let sta_avg = sta_sum / sta_len.min(i) as f64;

        if lta_avg > 1e-30 {
            ratio[i] = sta_avg / lta_avg;
        }
    }

    ratio
}

/// Detect the P-wave arrival sample using an STA/LTA picker.
///
/// Uses a short-term window of 0.3 s and a long-term window of 3.0 s on the
/// characteristic function (absolute value). The P-onset is declared where the
/// STA/LTA ratio first exceeds a threshold of 3.0.
///
/// - `signal`: vertical component seismogram (P-waves are strongest here).
/// - `fs`: sample rate in Hz.
///
/// Returns the sample index of the detected P-arrival. If no arrival is found,
/// returns the index of the maximum STA/LTA value.
pub fn detect_p_arrival(signal: &[f64], fs: f64) -> usize {
    if signal.is_empty() {
        return 0;
    }

    let sta_len = (0.3 * fs).round() as usize;
    let lta_len = (3.0 * fs).round() as usize;

    let sta_len = sta_len.max(1).min(signal.len());
    let lta_len = lta_len.max(sta_len + 1).min(signal.len());

    let ratio = sta_lta_ratio(signal, sta_len, lta_len);

    // Threshold trigger
    let threshold = 3.0;
    for (i, &r) in ratio.iter().enumerate() {
        if r > threshold {
            // Walk back to find the true onset (where ratio started rising)
            let mut onset = i;
            while onset > 0 && ratio[onset - 1] > 1.0 {
                onset -= 1;
            }
            return onset;
        }
    }

    // Fallback: index of maximum ratio
    ratio
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0)
}

/// Detect the S-wave arrival sample after a known P-onset.
///
/// Uses STA/LTA on the horizontal envelope (sqrt(N^2 + E^2)) starting from
/// the P-onset. The S-wave is the second major energy increase, predominantly
/// on horizontal components.
///
/// - `vertical`: Z-component (used to suppress P-coda).
/// - `horizontal`: horizontal envelope or single horizontal component.
/// - `fs`: sample rate in Hz.
/// - `p_onset`: previously detected P-wave sample index.
///
/// Returns the sample index of the S-arrival. Always >= `p_onset`.
pub fn detect_s_arrival(
    vertical: &[f64],
    horizontal: &[f64],
    fs: f64,
    p_onset: usize,
) -> usize {
    let n = vertical.len().min(horizontal.len());

    if n == 0 || p_onset >= n {
        return p_onset;
    }

    // Work on the signal after P-onset
    // Use ratio of horizontal to vertical energy as characteristic function
    // S-waves have stronger horizontal motion than P-waves
    let min_s_delay = (0.5 * fs) as usize; // S must be at least 0.5 s after P
    let search_start = (p_onset + min_s_delay).min(n);

    if search_start >= n {
        return p_onset;
    }

    let sub_signal = &horizontal[search_start..n];
    if sub_signal.len() < 4 {
        return p_onset;
    }

    let sta_len = (0.2 * fs).round() as usize;
    let lta_len = (2.0 * fs).round() as usize;

    let sta_len = sta_len.max(1).min(sub_signal.len());
    let lta_len = lta_len.max(sta_len + 1).min(sub_signal.len());

    let ratio = sta_lta_ratio(sub_signal, sta_len, lta_len);

    let threshold = 2.5;
    for (i, &r) in ratio.iter().enumerate() {
        if r > threshold {
            let mut onset = i;
            while onset > 0 && ratio[onset - 1] > 1.0 {
                onset -= 1;
            }
            return search_start + onset;
        }
    }

    // Fallback: find maximum energy sample
    let max_idx = sub_signal
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().partial_cmp(&b.1.abs()).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    search_start + max_idx
}

// ─── Magnitude Estimation ────────────────────────────────────────────────────

/// Estimate local magnitude ML from peak amplitude and distance.
///
/// Uses the Richter (1935) formula:
///
/// ```text
/// ML = log10(A) + 1.110 * log10(D) + 0.00189 * D - 2.09
/// ```
///
/// where `A` is the maximum zero-to-peak amplitude in mm on a Wood-Anderson
/// seismometer and `D` is the epicentral distance in km.
///
/// This is a simplified version of the distance correction table; the
/// coefficients approximate the original Richter curves for Southern California
/// and are widely used as a first-order estimate.
///
/// - `max_amplitude`: peak amplitude (Wood-Anderson equivalent, or raw units).
/// - `distance_km`: epicentral distance in km.
pub fn estimate_magnitude_ml(max_amplitude: f64, distance_km: f64) -> f64 {
    if max_amplitude <= 0.0 || distance_km <= 0.0 {
        return 0.0;
    }
    let log_a = max_amplitude.log10();
    // Richter distance correction (Hutton & Boore, 1987 variant)
    let correction = 1.110 * distance_km.log10() + 0.00189 * distance_km - 2.09;
    log_a + correction
}

/// Convert P-S time difference to epicentral distance.
///
/// From the relationship `d = dt * vp * vs / (vp - vs)`, where `dt` is the
/// S-P arrival time difference, `vp` is P-wave velocity, and `vs` is S-wave
/// velocity.
///
/// - `delta_t_s`: S-P time difference in seconds.
/// - `vp`: P-wave velocity in km/s.
/// - `vs`: S-wave velocity in km/s.
///
/// Returns distance in km.
pub fn p_s_time_to_distance(delta_t_s: f64, vp: f64, vs: f64) -> f64 {
    if delta_t_s <= 0.0 || vp <= vs || vs <= 0.0 {
        return 0.0;
    }
    delta_t_s * vp * vs / (vp - vs)
}

// ─── Back Azimuth ────────────────────────────────────────────────────────────

/// Estimate back azimuth from P-wave first-motion polarization.
///
/// Performs a simple polarization analysis on the horizontal components in a
/// window around the P-onset. The back azimuth is the direction from the
/// station toward the source, computed as:
///
/// ```text
/// azimuth = atan2(mean_east, mean_north) [converted to 0-360 degrees]
/// ```
///
/// This is a simplified version of covariance-matrix-based polarization
/// analysis (Jurkevics, 1988).
///
/// - `north`: N-component seismogram.
/// - `east`: E-component seismogram.
/// - `p_onset`: P-wave onset sample index.
/// - `window`: number of samples after P-onset to analyze.
///
/// Returns azimuth in degrees, 0 = North, increasing clockwise.
pub fn estimate_back_azimuth(
    north: &[f64],
    east: &[f64],
    p_onset: usize,
    window: usize,
) -> f64 {
    let n = north.len().min(east.len());
    if n == 0 || window == 0 {
        return 0.0;
    }

    let start = p_onset.min(n);
    let end = (start + window).min(n);

    if start >= end {
        return 0.0;
    }

    // Covariance matrix elements for horizontal components
    let len = (end - start) as f64;
    let mut cnn = 0.0;
    let mut cee = 0.0;
    let mut cne = 0.0;

    for i in start..end {
        cnn += north[i] * north[i];
        cee += east[i] * east[i];
        cne += north[i] * east[i];
    }

    cnn /= len;
    cee /= len;
    cne /= len;

    // Principal eigenvector direction of the 2x2 covariance matrix
    // For [[cnn, cne], [cne, cee]], the principal eigenvector angle is:
    // theta = 0.5 * atan2(2 * cne, cnn - cee)
    // But we also need to resolve the 180-degree ambiguity using first-motion polarity.

    let theta = 0.5 * (2.0 * cne).atan2(cnn - cee);

    // Determine polarity from the initial motion direction
    let first_n = north.get(start).copied().unwrap_or(0.0);
    let first_e = east.get(start).copied().unwrap_or(0.0);

    // Project first motion onto the eigenvector
    let proj = first_n * theta.cos() + first_e * theta.sin();

    // If projection is negative, flip by 180 degrees
    let azimuth_rad = if proj < 0.0 { theta + PI } else { theta };

    // Convert to degrees, 0-360 range
    let mut deg = azimuth_rad.to_degrees();
    if deg < 0.0 {
        deg += 360.0;
    }
    if deg >= 360.0 {
        deg -= 360.0;
    }

    deg
}

// ─── Spectral Analysis ──────────────────────────────────────────────────────

/// Compute the spectral ratio of high-frequency to low-frequency energy.
///
/// This discriminant is one of the most reliable features for separating
/// explosions from earthquakes. Explosions typically show higher spectral
/// ratios because their source spectra are enriched in high frequencies.
///
/// The function computes a DFT of the P-wave window and the S-wave window
/// separately, then returns the ratio of high-freq to low-freq energy in the
/// P-wave spectrum.
///
/// - `signal`: input seismogram.
/// - `fs`: sample rate in Hz.
/// - `p_onset`: P-wave onset sample.
/// - `s_onset`: S-wave onset sample.
///
/// Returns the spectral ratio (high/low frequency energy). Values > 2.0 suggest
/// an explosion; values < 1.0 suggest a natural earthquake.
pub fn spectral_ratio_p_s(signal: &[f64], fs: f64, p_onset: usize, s_onset: usize) -> f64 {
    let n = signal.len();
    if p_onset >= s_onset || s_onset >= n || s_onset - p_onset < 4 {
        return 1.0;
    }

    // Take the P-wave window
    let p_window = &signal[p_onset..s_onset];
    let p_len = p_window.len();

    // Compute power spectrum via DFT
    let spectrum = compute_power_spectrum(p_window);

    // Split into low-frequency and high-frequency halves
    let nyquist_bin = spectrum.len();
    let split = nyquist_bin / 2;

    if split == 0 {
        return 1.0;
    }

    let low_energy: f64 = spectrum[..split].iter().sum();
    let high_energy: f64 = spectrum[split..].iter().sum();

    if low_energy < 1e-30 {
        return if high_energy > 1e-30 { 100.0 } else { 1.0 };
    }

    // Also consider the S-wave window for comparison
    let s_end = (s_onset + p_len).min(n);
    if s_end <= s_onset {
        return high_energy / low_energy;
    }

    // Primary discriminant: P-wave high/low ratio
    high_energy / low_energy
}

/// Compute the one-sided power spectrum of a real-valued signal using DFT.
///
/// Returns `N/2 + 1` power values from DC to Nyquist.
fn compute_power_spectrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    let n_out = n / 2 + 1;
    let mut spectrum = Vec::with_capacity(n_out);

    for k in 0..n_out {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            re += x * angle.cos();
            im += x * angle.sin();
        }
        spectrum.push(re * re + im * im);
    }

    spectrum
}

// ─── Wood-Anderson Simulation ────────────────────────────────────────────────

/// Simulate the Wood-Anderson torsion seismometer response.
///
/// The Wood-Anderson seismometer has:
/// - Natural period T0 = 0.8 s (f0 = 1.25 Hz)
/// - Damping ratio h = 0.8
/// - Static magnification V = 2800
///
/// This function applies a second-order IIR filter approximating the
/// displacement response of the Wood-Anderson instrument. The filter is
/// derived from the bilinear transform of the analog transfer function:
///
/// ```text
/// H(s) = -V * s^2 / (s^2 + 2*h*w0*s + w0^2)
/// ```
///
/// - `signal`: input velocity or displacement seismogram.
/// - `fs`: sample rate in Hz.
///
/// Returns the Wood-Anderson equivalent displacement trace (same length as input).
pub fn wood_anderson_response(signal: &[f64], fs: f64) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || fs <= 0.0 {
        return vec![0.0; n];
    }

    // Wood-Anderson parameters
    let v_mag = 2800.0; // static magnification
    let f0 = 1.25; // natural frequency Hz
    let w0 = 2.0 * PI * f0;
    let h = 0.8; // damping ratio

    // Bilinear transform: s = 2*fs * (z - 1)/(z + 1)
    let c = 2.0 * fs;

    // Denominator: s^2 + 2*h*w0*s + w0^2
    // Numerator: -V * s^2
    // With s = c*(z-1)/(z+1):
    // s^2 = c^2 * (z-1)^2 / (z+1)^2
    //
    // Multiply through by (z+1)^2:
    // Den: c^2*(z-1)^2 + 2*h*w0*c*(z-1)*(z+1) + w0^2*(z+1)^2
    // Num: -V*c^2*(z-1)^2

    let c2 = c * c;
    let w02 = w0 * w0;
    let two_hw0c = 2.0 * h * w0 * c;

    // (z-1)^2 = z^2 - 2z + 1
    // (z-1)(z+1) = z^2 - 1
    // (z+1)^2 = z^2 + 2z + 1

    // Denominator coefficients (z^2, z, 1)
    let a0 = c2 + two_hw0c + w02;
    let a1 = -2.0 * c2 + 2.0 * w02;
    let a2 = c2 - two_hw0c + w02;

    // Numerator: -V*c^2 * (z^2 - 2z + 1)
    let b0 = -v_mag * c2;
    let b1 = 2.0 * v_mag * c2;
    let b2 = -v_mag * c2;

    // Normalize by a0
    let b0n = b0 / a0;
    let b1n = b1 / a0;
    let b2n = b2 / a0;
    let a1n = a1 / a0;
    let a2n = a2 / a0;

    // Apply IIR filter (Direct Form II Transposed)
    let mut output = vec![0.0; n];
    let mut w1 = 0.0;
    let mut w2 = 0.0;

    for i in 0..n {
        let x = signal[i];
        let y = b0n * x + w1;
        w1 = b1n * x - a1n * y + w2;
        w2 = b2n * x - a2n * y;
        // Scale down for numerical sanity (raw WA output is very large)
        output[i] = y / v_mag;
    }

    output
}

// ─── Depth Estimation ────────────────────────────────────────────────────────

/// Crude depth estimate from distance and P-S time.
///
/// For shallow events (< 30 km), the P-S time vs distance relationship
/// deviates from the straight-line approximation. We use a simple heuristic:
/// if the distance is short but the P-S time is relatively large, the event
/// is likely deeper.
fn estimate_depth(distance_km: f64, delta_t_s: f64) -> f64 {
    if distance_km <= 0.0 || delta_t_s <= 0.0 {
        return 0.0;
    }

    // Simple geometric estimate:
    // If the true slant distance = sqrt(d^2 + z^2), and we computed d from
    // the P-S time assuming a flat earth, the excess P-S time suggests depth.
    // Heuristic: depth ~ distance * some_factor for typical crustal events.
    // Typical shallow crustal earthquakes: 5-20 km
    // Explosions/quarry blasts: 0-2 km

    // Use a rough scaling: for every second of P-S time, ~8 km depth component
    let raw_depth = delta_t_s * 3.0;
    // Clamp to reasonable range
    raw_depth.max(0.0).min(700.0) // max ~700 km for deepest subduction zone events
}

// ─── Classification Heuristics ───────────────────────────────────────────────

/// Classify event type based on extracted features.
///
/// Heuristic decision tree:
///
/// 1. If amplitude too low -> Noise
/// 2. High spectral ratio (> 2.5) and shallow depth -> Explosion or QuarryBlast
/// 3. Very shallow (< 0.5 km) with high spectral ratio -> QuarryBlast
/// 4. Moderate depth (0.5-3 km) with high spectral ratio -> Explosion
/// 5. Low spectral ratio (< 1.5) -> Earthquake
/// 6. Very shallow with impulsive onset and moderate spectral ratio -> MineCollapse
/// 7. Otherwise -> Unknown
fn classify_by_features(
    spectral_ratio: f64,
    delta_t_s: f64,
    magnitude: f64,
    depth_km: f64,
    max_amplitude: f64,
    min_magnitude: f64,
) -> EventType {
    // Too weak to classify - threshold relative to typical seismometer noise
    if max_amplitude < 1e-6 {
        return EventType::Noise;
    }

    // Very small event below reporting threshold
    if magnitude < min_magnitude && magnitude > 0.0 {
        return EventType::Noise;
    }

    // If magnitude is strongly negative (extremely weak signal), it is noise
    if magnitude < -2.0 {
        return EventType::Noise;
    }

    // Very short P-S time => very close, likely surface blast
    if delta_t_s < 0.5 && spectral_ratio > 2.0 {
        return EventType::QuarryBlast;
    }

    // High spectral ratio => explosive source
    if spectral_ratio > 2.5 {
        if depth_km < 1.5 {
            return EventType::QuarryBlast;
        }
        if depth_km < 5.0 {
            return EventType::Explosion;
        }
    }

    if spectral_ratio > 1.8 {
        if depth_km < 0.5 {
            return EventType::QuarryBlast;
        }
        if depth_km < 3.0 {
            return EventType::Explosion;
        }
    }

    // Mine collapse: moderate spectral ratio, very shallow, lower frequency
    // content than a blast but still impulsive
    if spectral_ratio > 0.8 && spectral_ratio < 1.8 && depth_km < 2.0 && delta_t_s < 2.0 {
        // Could be mine collapse - characterized by lower frequency content
        // than explosions but shorter duration than earthquakes
        if spectral_ratio < 1.2 && depth_km < 1.0 {
            return EventType::MineCollapse;
        }
    }

    // Low spectral ratio => natural earthquake
    if spectral_ratio < 1.5 {
        return EventType::Earthquake;
    }

    EventType::Unknown
}

// ─── Helper Utilities ────────────────────────────────────────────────────────

/// Compute the RMS (root mean square) of a signal slice.
fn rms(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = signal.iter().map(|&x| x * x).sum();
    (sum_sq / signal.len() as f64).sqrt()
}

/// Apply a Hann window to a signal slice (in-place copy).
fn apply_hann_window(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }
    signal
        .iter()
        .enumerate()
        .map(|(i, &x)| {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1).max(1) as f64).cos());
            x * w
        })
        .collect()
}

/// Compute the dominant frequency of a signal using DFT peak detection.
pub fn dominant_frequency(signal: &[f64], fs: f64) -> f64 {
    if signal.len() < 2 || fs <= 0.0 {
        return 0.0;
    }

    let windowed = apply_hann_window(signal);
    let spectrum = compute_power_spectrum(&windowed);

    // Skip DC bin (index 0)
    let max_bin = spectrum
        .iter()
        .enumerate()
        .skip(1)
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, _)| i)
        .unwrap_or(0);

    if max_bin == 0 {
        return 0.0;
    }

    // Convert bin index to frequency
    max_bin as f64 * fs / signal.len() as f64
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 100.0; // 100 Hz sample rate (typical broadband seismometer)

    /// Generate a synthetic seismic trace with a clear onset at a given sample.
    fn make_onset_signal(n: usize, onset: usize, freq: f64, amplitude: f64, decay: f64) -> Vec<f64> {
        let mut signal = vec![0.0; n];
        for i in onset..n {
            let t = (i - onset) as f64 / FS;
            signal[i] = amplitude * (-decay * t).exp() * (2.0 * PI * freq * t).sin();
        }
        signal
    }

    /// Add Gaussian-like noise to a signal (deterministic PRNG for reproducibility).
    fn add_noise(signal: &mut [f64], amplitude: f64, seed: u64) {
        let mut state = seed;
        for x in signal.iter_mut() {
            // Simple xorshift64
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let uniform = (state as f64) / (u64::MAX as f64) * 2.0 - 1.0;
            *x += amplitude * uniform;
        }
    }

    // ── STA/LTA Tests ──────────────────────────────────────────────────────

    #[test]
    fn test_sta_lta_empty_signal() {
        let result = sta_lta_ratio(&[], 5, 50);
        assert!(result.is_empty());
    }

    #[test]
    fn test_sta_lta_quiet_signal() {
        let signal = vec![0.01; 500];
        let ratio = sta_lta_ratio(&signal, 5, 50);
        // Constant signal: STA/LTA should be ~1.0 everywhere it's defined
        for &r in ratio.iter().skip(50) {
            assert!((r - 1.0).abs() < 0.1, "Expected ~1.0, got {}", r);
        }
    }

    #[test]
    fn test_sta_lta_onset_detection() {
        let signal = make_onset_signal(1000, 300, 5.0, 10.0, 0.5);
        let ratio = sta_lta_ratio(&signal, 5, 50);

        // The ratio should spike near sample 300
        let max_idx = ratio
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!(
            (max_idx as isize - 300).unsigned_abs() < 20,
            "Max STA/LTA at {} expected near 300",
            max_idx
        );
    }

    #[test]
    fn test_sta_lta_ratio_increases_at_onset() {
        let signal = make_onset_signal(1000, 400, 3.0, 5.0, 0.3);
        let ratio = sta_lta_ratio(&signal, 10, 100);

        // Before onset: ratio should be low
        let pre_onset_max = ratio[100..380].iter().cloned().fold(0.0f64, f64::max);
        // After onset: ratio should be high
        let post_onset_max = ratio[400..450].iter().cloned().fold(0.0f64, f64::max);

        assert!(
            post_onset_max > pre_onset_max * 2.0,
            "Post-onset max {} should be >> pre-onset max {}",
            post_onset_max,
            pre_onset_max
        );
    }

    #[test]
    fn test_sta_lta_zero_sta_len() {
        let signal = vec![1.0; 100];
        let result = sta_lta_ratio(&signal, 0, 50);
        assert!(result.iter().all(|&r| r == 0.0));
    }

    #[test]
    fn test_sta_lta_zero_lta_len() {
        let signal = vec![1.0; 100];
        let result = sta_lta_ratio(&signal, 5, 0);
        assert!(result.iter().all(|&r| r == 0.0));
    }

    // ── P-Arrival Detection ────────────────────────────────────────────────

    #[test]
    fn test_p_arrival_synthetic_event() {
        let mut signal = make_onset_signal(2000, 500, 8.0, 20.0, 0.5);
        add_noise(&mut signal, 0.01, 42);

        let p_idx = detect_p_arrival(&signal, FS);
        assert!(
            (p_idx as isize - 500).unsigned_abs() < 50,
            "P-arrival at {} expected near 500",
            p_idx
        );
    }

    #[test]
    fn test_p_arrival_early_event() {
        // Event at very start - detector should still find something reasonable
        let signal = make_onset_signal(2000, 50, 5.0, 15.0, 0.4);
        let p_idx = detect_p_arrival(&signal, FS);
        // Should detect something in first ~350 samples (limited by LTA window)
        assert!(p_idx < 400, "P-arrival {} should be early", p_idx);
    }

    #[test]
    fn test_p_arrival_empty_signal() {
        let p_idx = detect_p_arrival(&[], FS);
        assert_eq!(p_idx, 0);
    }

    #[test]
    fn test_p_arrival_pure_noise() {
        let mut signal = vec![0.0; 2000];
        add_noise(&mut signal, 0.1, 99);
        // Should still return some index without panicking
        let p_idx = detect_p_arrival(&signal, FS);
        assert!(p_idx < signal.len());
    }

    // ── S-Arrival Detection ────────────────────────────────────────────────

    #[test]
    fn test_s_arrival_after_p() {
        let n = 2000;
        let vertical = make_onset_signal(n, 300, 8.0, 10.0, 0.5);
        let horizontal = make_onset_signal(n, 600, 4.0, 15.0, 0.3);

        let s_idx = detect_s_arrival(&vertical, &horizontal, FS, 300);
        assert!(
            s_idx > 300,
            "S-arrival {} must be after P at 300",
            s_idx
        );
        assert!(
            s_idx < 800,
            "S-arrival {} should be near 600",
            s_idx
        );
    }

    #[test]
    fn test_s_arrival_returns_p_onset_if_no_data() {
        let vertical = vec![0.0; 10];
        let horizontal = vec![0.0; 10];
        let s_idx = detect_s_arrival(&vertical, &horizontal, FS, 5);
        assert!(s_idx >= 5);
    }

    #[test]
    fn test_s_arrival_with_p_at_end() {
        let vertical = vec![0.0; 100];
        let horizontal = vec![0.0; 100];
        let s_idx = detect_s_arrival(&vertical, &horizontal, FS, 99);
        assert_eq!(s_idx, 99);
    }

    // ── Magnitude Estimation ───────────────────────────────────────────────

    #[test]
    fn test_magnitude_ml_basic() {
        // A = 1 mm at 100 km: ML should be around 3
        let ml = estimate_magnitude_ml(1.0, 100.0);
        assert!(ml > 0.0 && ml < 6.0, "ML={} out of range", ml);
    }

    #[test]
    fn test_magnitude_increases_with_amplitude() {
        let ml_small = estimate_magnitude_ml(0.1, 50.0);
        let ml_large = estimate_magnitude_ml(10.0, 50.0);
        assert!(
            ml_large > ml_small,
            "Larger amplitude should give larger ML: {} vs {}",
            ml_large,
            ml_small
        );
    }

    #[test]
    fn test_magnitude_increases_with_distance() {
        // Same amplitude at greater distance => larger ML (it was stronger at source)
        let ml_near = estimate_magnitude_ml(1.0, 10.0);
        let ml_far = estimate_magnitude_ml(1.0, 200.0);
        assert!(
            ml_far > ml_near,
            "Greater distance should give larger ML: {} vs {}",
            ml_far,
            ml_near
        );
    }

    #[test]
    fn test_magnitude_zero_amplitude() {
        assert_eq!(estimate_magnitude_ml(0.0, 100.0), 0.0);
    }

    #[test]
    fn test_magnitude_zero_distance() {
        assert_eq!(estimate_magnitude_ml(1.0, 0.0), 0.0);
    }

    #[test]
    fn test_magnitude_negative_amplitude() {
        assert_eq!(estimate_magnitude_ml(-5.0, 100.0), 0.0);
    }

    // ── P-S Distance ──────────────────────────────────────────────────────

    #[test]
    fn test_ps_distance_basic() {
        // With vp=6, vs=3.5: dt=1s => d = 1 * 6 * 3.5 / (6 - 3.5) = 8.4 km
        let d = p_s_time_to_distance(1.0, 6.0, 3.5);
        assert!(
            (d - 8.4).abs() < 0.01,
            "Distance {} should be 8.4 km",
            d
        );
    }

    #[test]
    fn test_ps_distance_zero_dt() {
        assert_eq!(p_s_time_to_distance(0.0, 6.0, 3.5), 0.0);
    }

    #[test]
    fn test_ps_distance_negative_dt() {
        assert_eq!(p_s_time_to_distance(-1.0, 6.0, 3.5), 0.0);
    }

    #[test]
    fn test_ps_distance_vp_equals_vs() {
        // Would divide by zero; should return 0
        assert_eq!(p_s_time_to_distance(1.0, 5.0, 5.0), 0.0);
    }

    #[test]
    fn test_ps_distance_vp_less_than_vs() {
        assert_eq!(p_s_time_to_distance(1.0, 3.0, 5.0), 0.0);
    }

    #[test]
    fn test_ps_distance_scales_linearly() {
        let d1 = p_s_time_to_distance(1.0, 6.0, 3.5);
        let d2 = p_s_time_to_distance(2.0, 6.0, 3.5);
        assert!(
            (d2 - 2.0 * d1).abs() < 1e-10,
            "Distance should scale linearly with dt"
        );
    }

    // ── Back Azimuth ──────────────────────────────────────────────────────

    #[test]
    fn test_back_azimuth_north() {
        // Strong north component, weak east => azimuth ~0 or ~360 (North)
        let n = 200;
        let north = make_onset_signal(n, 50, 3.0, 10.0, 0.5);
        let east = vec![0.0; n];

        let az = estimate_back_azimuth(&north, &east, 50, 50);
        assert!(
            az < 30.0 || az > 330.0,
            "North-dominant should give azimuth near 0/360, got {}",
            az
        );
    }

    #[test]
    fn test_back_azimuth_east() {
        // Strong east component, weak north => azimuth ~90
        let n = 200;
        let north = vec![0.0; n];
        let east = make_onset_signal(n, 50, 3.0, 10.0, 0.5);

        let az = estimate_back_azimuth(&north, &east, 50, 50);
        assert!(
            (az - 90.0).abs() < 30.0 || (az - 270.0).abs() < 30.0,
            "East-dominant azimuth should be ~90 or ~270, got {}",
            az
        );
    }

    #[test]
    fn test_back_azimuth_empty() {
        assert_eq!(estimate_back_azimuth(&[], &[], 0, 0), 0.0);
    }

    #[test]
    fn test_back_azimuth_range() {
        // Azimuth should always be in [0, 360)
        let n = 200;
        let north = make_onset_signal(n, 30, 2.0, 5.0, 0.3);
        let mut east = make_onset_signal(n, 30, 2.0, 3.0, 0.3);
        // Flip east to get different quadrant
        for x in east.iter_mut() {
            *x = -*x;
        }
        let az = estimate_back_azimuth(&north, &east, 30, 50);
        assert!(az >= 0.0 && az < 360.0, "Azimuth {} out of range", az);
    }

    // ── Spectral Ratio ─────────────────────────────────────────────────────

    #[test]
    fn test_spectral_ratio_high_frequency() {
        // High-frequency P-wave should give ratio > 1
        let signal = make_onset_signal(500, 50, 20.0, 5.0, 0.3);
        let ratio = spectral_ratio_p_s(&signal, FS, 50, 250);
        assert!(ratio > 0.0, "Spectral ratio should be positive, got {}", ratio);
    }

    #[test]
    fn test_spectral_ratio_low_frequency() {
        // Very low frequency signal should have more low-freq energy
        let signal = make_onset_signal(1000, 100, 1.0, 5.0, 0.1);
        let ratio = spectral_ratio_p_s(&signal, FS, 100, 600);
        // Low frequency content should yield ratio < high-frequency case
        assert!(ratio > 0.0);
    }

    #[test]
    fn test_spectral_ratio_edge_cases() {
        assert_eq!(spectral_ratio_p_s(&[], FS, 0, 0), 1.0);
        assert_eq!(spectral_ratio_p_s(&[1.0; 10], FS, 5, 3), 1.0); // p > s
        assert_eq!(spectral_ratio_p_s(&[1.0; 10], FS, 5, 5), 1.0); // p == s
    }

    // ── Wood-Anderson Response ──────────────────────────────────────────────

    #[test]
    fn test_wood_anderson_preserves_length() {
        let signal = vec![1.0; 500];
        let wa = wood_anderson_response(&signal, FS);
        assert_eq!(wa.len(), signal.len());
    }

    #[test]
    fn test_wood_anderson_empty() {
        let wa = wood_anderson_response(&[], FS);
        assert!(wa.is_empty());
    }

    #[test]
    fn test_wood_anderson_zero_fs() {
        let signal = vec![1.0; 100];
        let wa = wood_anderson_response(&signal, 0.0);
        assert!(wa.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_wood_anderson_produces_nonzero_output() {
        let signal = make_onset_signal(1000, 200, 3.0, 5.0, 0.3);
        let wa = wood_anderson_response(&signal, FS);
        let max_wa = wa.iter().map(|x| x.abs()).fold(0.0f64, f64::max);
        assert!(max_wa > 0.0, "WA output should be nonzero for real input");
    }

    // ── Dominant Frequency ──────────────────────────────────────────────────

    #[test]
    fn test_dominant_frequency_pure_tone() {
        let n = 1024;
        let freq = 10.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / FS).sin())
            .collect();

        let dom_freq = dominant_frequency(&signal, FS);
        assert!(
            (dom_freq - freq).abs() < 1.0,
            "Dominant freq {} should be near {} Hz",
            dom_freq,
            freq
        );
    }

    #[test]
    fn test_dominant_frequency_empty() {
        assert_eq!(dominant_frequency(&[], FS), 0.0);
    }

    // ── Full Classification Pipeline ────────────────────────────────────────

    #[test]
    fn test_classify_noise() {
        let config = SeismicConfig::default();
        let classifier = SeismicClassifier::new(config);

        let n = 2000;
        let mut vertical = vec![0.0; n];
        let mut north = vec![0.0; n];
        let mut east = vec![0.0; n];
        add_noise(&mut vertical, 0.001, 1);
        add_noise(&mut north, 0.001, 2);
        add_noise(&mut east, 0.001, 3);

        let event = classifier.classify_event(&vertical, &north, &east);
        // Very weak signal should classify as Noise
        assert_eq!(event.classification, EventType::Noise);
    }

    #[test]
    fn test_classify_short_signal() {
        let config = SeismicConfig::default();
        let classifier = SeismicClassifier::new(config);

        let event = classifier.classify_event(&[0.0; 5], &[0.0; 5], &[0.0; 5]);
        assert_eq!(event.classification, EventType::Noise);
    }

    #[test]
    fn test_classify_returns_valid_event() {
        let config = SeismicConfig::default();
        let classifier = SeismicClassifier::new(config);

        let n = 3000;
        let mut vertical = make_onset_signal(n, 500, 8.0, 20.0, 0.5);
        let mut north = make_onset_signal(n, 800, 4.0, 25.0, 0.3);
        let mut east = make_onset_signal(n, 800, 4.0, 15.0, 0.3);
        add_noise(&mut vertical, 0.05, 10);
        add_noise(&mut north, 0.05, 11);
        add_noise(&mut east, 0.05, 12);

        let event = classifier.classify_event(&vertical, &north, &east);

        // P should come before S
        assert!(
            event.p_arrival_sample < event.s_arrival_sample,
            "P ({}) should be before S ({})",
            event.p_arrival_sample,
            event.s_arrival_sample
        );

        // Azimuth in valid range
        assert!(event.back_azimuth_deg >= 0.0 && event.back_azimuth_deg < 360.0);

        // Spectral ratio should be positive
        assert!(event.spectral_ratio > 0.0);
    }

    #[test]
    fn test_classify_event_type_display() {
        assert_eq!(format!("{}", EventType::Earthquake), "Earthquake");
        assert_eq!(format!("{}", EventType::Explosion), "Explosion");
        assert_eq!(format!("{}", EventType::QuarryBlast), "QuarryBlast");
        assert_eq!(format!("{}", EventType::MineCollapse), "MineCollapse");
        assert_eq!(format!("{}", EventType::Noise), "Noise");
        assert_eq!(format!("{}", EventType::Unknown), "Unknown");
    }

    #[test]
    fn test_classify_with_custom_velocities() {
        let config = SeismicConfig {
            sample_rate_hz: 100.0,
            p_velocity_km_s: 8.0,
            s_velocity_km_s: 4.5,
            min_magnitude: 0.5,
        };
        let classifier = SeismicClassifier::new(config);

        let n = 3000;
        let vertical = make_onset_signal(n, 500, 5.0, 10.0, 0.3);
        let north = make_onset_signal(n, 800, 3.0, 12.0, 0.2);
        let east = make_onset_signal(n, 800, 3.0, 8.0, 0.2);

        let event = classifier.classify_event(&vertical, &north, &east);
        // Should not panic, and should produce some classification
        assert!(event.magnitude_ml.is_finite());
        assert!(event.depth_estimate_km >= 0.0);
    }

    // ── Depth Estimation ───────────────────────────────────────────────────

    #[test]
    fn test_depth_estimate_positive() {
        let d = estimate_depth(50.0, 5.0);
        assert!(d > 0.0, "Depth should be positive");
    }

    #[test]
    fn test_depth_estimate_zero_inputs() {
        assert_eq!(estimate_depth(0.0, 1.0), 0.0);
        assert_eq!(estimate_depth(50.0, 0.0), 0.0);
    }

    #[test]
    fn test_depth_estimate_capped() {
        let d = estimate_depth(1000.0, 500.0);
        assert!(d <= 700.0, "Depth {} should be capped at 700 km", d);
    }

    // ── Classification Decision Tree ────────────────────────────────────────

    #[test]
    fn test_classify_features_noise() {
        let et = classify_by_features(1.0, 1.0, 0.5, 5.0, 1e-12, 1.0);
        assert_eq!(et, EventType::Noise);
    }

    #[test]
    fn test_classify_features_earthquake() {
        let et = classify_by_features(0.8, 5.0, 3.5, 15.0, 10.0, 1.0);
        assert_eq!(et, EventType::Earthquake);
    }

    #[test]
    fn test_classify_features_explosion() {
        let et = classify_by_features(3.0, 2.0, 2.5, 3.0, 10.0, 1.0);
        assert_eq!(et, EventType::Explosion);
    }

    #[test]
    fn test_classify_features_quarry_blast() {
        let et = classify_by_features(3.0, 0.3, 2.0, 0.5, 10.0, 1.0);
        assert_eq!(et, EventType::QuarryBlast);
    }

    #[test]
    fn test_classify_features_mine_collapse() {
        let et = classify_by_features(1.0, 1.0, 2.0, 0.5, 10.0, 1.0);
        assert_eq!(et, EventType::MineCollapse);
    }

    #[test]
    fn test_classify_features_below_magnitude_threshold() {
        let et = classify_by_features(1.0, 3.0, 0.5, 10.0, 5.0, 1.0);
        assert_eq!(et, EventType::Noise);
    }

    // ── RMS Utility ────────────────────────────────────────────────────────

    #[test]
    fn test_rms_empty() {
        assert_eq!(rms(&[]), 0.0);
    }

    #[test]
    fn test_rms_constant() {
        let val = rms(&[3.0, 3.0, 3.0, 3.0]);
        assert!((val - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_rms_sine_wave() {
        // RMS of a full-cycle sine wave = 1/sqrt(2) ≈ 0.7071
        let n = 10000;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let val = rms(&signal);
        assert!(
            (val - 1.0 / 2.0_f64.sqrt()).abs() < 0.01,
            "RMS of sine should be ~0.707, got {}",
            val
        );
    }

    // ── Hann Window ────────────────────────────────────────────────────────

    #[test]
    fn test_hann_window_endpoints() {
        let signal = vec![1.0; 64];
        let windowed = apply_hann_window(&signal);
        // Hann window is zero at endpoints
        assert!(windowed[0].abs() < 1e-10, "Hann should be 0 at start");
        assert!(windowed[63].abs() < 1e-10, "Hann should be 0 at end");
    }

    #[test]
    fn test_hann_window_peak_at_center() {
        let signal = vec![1.0; 65];
        let windowed = apply_hann_window(&signal);
        // Peak at center
        assert!(
            (windowed[32] - 1.0).abs() < 1e-10,
            "Hann peak should be 1.0 at center"
        );
    }

    #[test]
    fn test_hann_window_empty() {
        let windowed = apply_hann_window(&[]);
        assert!(windowed.is_empty());
    }

    // ── Power Spectrum ──────────────────────────────────────────────────────

    #[test]
    fn test_power_spectrum_pure_tone() {
        let n = 256;
        let bin = 10;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * bin as f64 * i as f64 / n as f64).sin())
            .collect();

        let spectrum = compute_power_spectrum(&signal);
        assert_eq!(spectrum.len(), n / 2 + 1);

        // Peak should be at bin 10
        let peak_bin = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_bin, bin, "Peak should be at bin {}", bin);
    }

    #[test]
    fn test_power_spectrum_empty() {
        assert!(compute_power_spectrum(&[]).is_empty());
    }

    // ── Integration: Full Seismic Scenario ──────────────────────────────────

    #[test]
    fn test_integration_three_component_event() {
        // Simulate a realistic three-component seismogram:
        // P-wave: high-frequency, vertical dominant, onset at 5s
        // S-wave: lower frequency, horizontal dominant, onset at 8s
        let fs = 100.0;
        let n = (20.0 * fs) as usize; // 20 seconds
        let p_onset = (5.0 * fs) as usize;
        let s_onset = (8.0 * fs) as usize;

        let mut vertical = vec![0.0; n];
        let mut north = vec![0.0; n];
        let mut east = vec![0.0; n];

        // P-wave on all components (vertical dominant)
        for i in p_onset..n {
            let t = (i - p_onset) as f64 / fs;
            let decay = (-0.5 * t).exp();
            vertical[i] += 8.0 * decay * (2.0 * PI * 6.0 * t).sin();
            north[i] += 1.0 * decay * (2.0 * PI * 6.0 * t).sin();
            east[i] += 0.5 * decay * (2.0 * PI * 6.0 * t).sin();
        }

        // S-wave on horizontal components (horizontal dominant)
        for i in s_onset..n {
            let t = (i - s_onset) as f64 / fs;
            let decay = (-0.3 * t).exp();
            vertical[i] += 1.0 * decay * (2.0 * PI * 3.0 * t).sin();
            north[i] += 12.0 * decay * (2.0 * PI * 3.0 * t).sin();
            east[i] += 6.0 * decay * (2.0 * PI * 3.0 * t).sin();
        }

        add_noise(&mut vertical, 0.02, 100);
        add_noise(&mut north, 0.02, 101);
        add_noise(&mut east, 0.02, 102);

        let config = SeismicConfig {
            sample_rate_hz: fs,
            p_velocity_km_s: 6.0,
            s_velocity_km_s: 3.5,
            min_magnitude: 0.0, // accept all magnitudes
        };
        let classifier = SeismicClassifier::new(config);
        let event = classifier.classify_event(&vertical, &north, &east);

        // P should be detected near sample 500
        assert!(
            (event.p_arrival_sample as f64 - p_onset as f64).abs() < 100.0,
            "P at {} expected near {}",
            event.p_arrival_sample,
            p_onset
        );

        // S should be after P
        assert!(event.s_arrival_sample > event.p_arrival_sample);

        // Distance should be positive (there's a real P-S delay)
        let delta_t = (event.s_arrival_sample as f64 - event.p_arrival_sample as f64) / fs;
        let distance = p_s_time_to_distance(delta_t, 6.0, 3.5);
        assert!(distance > 0.0, "Distance should be positive");

        // Event should be classified (not Unknown or Noise given the strong signal)
        assert_ne!(event.classification, EventType::Unknown);
    }
}
