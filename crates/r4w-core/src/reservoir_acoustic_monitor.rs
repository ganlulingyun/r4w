//! Distributed Acoustic Sensing (DAS) / Distributed Temperature Sensing (DTS) signal
//! processing for oil/gas reservoir monitoring using fiber optic cables in wellbores.
//!
//! This module implements the complete signal processing chain for fiber-optic
//! sensing in downhole applications:
//!
//! - **DAS processing**: Rayleigh backscatter phase-to-strain conversion with
//!   configurable gauge length and spatial resolution
//! - **DTS processing**: Raman backscatter ratio to temperature conversion
//! - **Event detection**: Fluid flow, sand production, leak detection,
//!   microseismic events, casing damage
//! - **VSP (Vertical Seismic Profile)**: Extracting seismic velocity from DAS data
//! - **F-K filtering**: Frequency-wavenumber domain velocity filtering for
//!   wavefield separation (up-going vs down-going)
//! - **Microseismic localization**: Source depth and offset estimation from
//!   arrival-time moveout
//! - **Optical budget**: Link-loss calculations for fiber installations
//!
//! # Example
//!
//! ```
//! use r4w_core::reservoir_acoustic_monitor::{DasConfig, DasProcessor};
//!
//! let config = DasConfig {
//!     fiber_length_m: 3000.0,
//!     spatial_resolution_m: 1.0,
//!     sample_rate_hz: 10000.0,
//!     gauge_length_m: 10.0,
//!     pulse_rate_hz: 5000.0,
//! };
//! let processor = DasProcessor::new(config);
//!
//! // Simulate backscatter from 3000 depth channels
//! let backscatter: Vec<f64> = (0..3000)
//!     .map(|i| 0.01 * (i as f64 * 0.1).sin())
//!     .collect();
//! let frame = processor.process_frame(&backscatter);
//! assert_eq!(frame.depth_axis_m.len(), frame.strain_rate.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the Distributed Acoustic Sensing processor.
#[derive(Debug, Clone)]
pub struct DasConfig {
    /// Total length of the sensing fiber in metres.
    pub fiber_length_m: f64,
    /// Spatial resolution along the fiber in metres (default 1.0 m).
    pub spatial_resolution_m: f64,
    /// Temporal sample rate of the interrogator in Hz.
    pub sample_rate_hz: f64,
    /// Gauge length used for phase differencing in metres (default 10.0 m).
    pub gauge_length_m: f64,
    /// Pulse repetition rate of the interrogator laser in Hz.
    pub pulse_rate_hz: f64,
}

impl Default for DasConfig {
    fn default() -> Self {
        Self {
            fiber_length_m: 3000.0,
            spatial_resolution_m: 1.0,
            sample_rate_hz: 10000.0,
            gauge_length_m: 10.0,
            pulse_rate_hz: 5000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Event types
// ---------------------------------------------------------------------------

/// Classification of detected acoustic events along the wellbore.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventType {
    /// Broadband noise from fluid moving through perforations or fractures.
    FluidFlow,
    /// High-frequency impulsive events caused by sand grains hitting casing.
    SandProduction,
    /// Persistent localised noise signature consistent with a casing or tubing leak.
    LeakDetection,
    /// Short-duration, high-amplitude event from rock fracture.
    MicroseismicEvent,
    /// Low-frequency resonance pattern indicative of casing deformation.
    CasingDamage,
    /// Background noise that does not match any known pattern.
    Noise,
}

/// A single detected acoustic event.
#[derive(Debug, Clone)]
pub struct AcousticEvent {
    /// Measured depth along the fiber in metres.
    pub depth_m: f64,
    /// Peak amplitude of the event (strain-rate units).
    pub amplitude: f64,
    /// Dominant frequency of the event in Hz.
    pub frequency_hz: f64,
    /// Classification of the event.
    pub event_type: EventType,
}

// ---------------------------------------------------------------------------
// DAS frame output
// ---------------------------------------------------------------------------

/// Output of a single DAS processing frame.
#[derive(Debug, Clone)]
pub struct DasFrame {
    /// Strain-rate values at each depth channel (units: strain/s or equivalent).
    pub strain_rate: Vec<f64>,
    /// Measured depth axis in metres corresponding to each strain-rate element.
    pub depth_axis_m: Vec<f64>,
    /// Acoustic events detected in this frame.
    pub events: Vec<AcousticEvent>,
}

// ---------------------------------------------------------------------------
// Optical budget
// ---------------------------------------------------------------------------

/// Result of an optical link-budget calculation.
#[derive(Debug, Clone)]
pub struct OpticalBudget {
    /// Total loss across the entire fiber link in dB.
    pub total_loss_db: f64,
    /// Loss attributable to the fiber itself (attenuation) in dB.
    pub fiber_loss_db: f64,
    /// Total loss from all fusion splices in dB.
    pub splice_loss_db: f64,
    /// Total loss from all connectors in dB.
    pub connector_loss_db: f64,
}

// ---------------------------------------------------------------------------
// DAS processor
// ---------------------------------------------------------------------------

/// Main processor that converts raw Rayleigh backscatter into strain-rate
/// profiles and detects acoustic events.
#[derive(Debug, Clone)]
pub struct DasProcessor {
    config: DasConfig,
    /// Number of spatial channels along the fiber.
    num_channels: usize,
}

impl DasProcessor {
    /// Create a new DAS processor from the given configuration.
    pub fn new(config: DasConfig) -> Self {
        let num_channels =
            (config.fiber_length_m / config.spatial_resolution_m).ceil() as usize;
        Self {
            config,
            num_channels,
        }
    }

    /// Process a single frame of raw backscatter data.
    ///
    /// `backscatter` contains one sample per spatial channel. Its length is
    /// clamped to `num_channels` if longer, or zero-padded conceptually if
    /// shorter.
    pub fn process_frame(&self, backscatter: &[f64]) -> DasFrame {
        let n = backscatter.len().min(self.num_channels);

        // Build depth axis
        let depth_axis_m: Vec<f64> = (0..n)
            .map(|i| i as f64 * self.config.spatial_resolution_m)
            .collect();

        // Convert phase-change proxy in backscatter to strain-rate.
        // We treat each backscatter sample as a phase difference and use the
        // standard DAS phase-to-strain conversion with the configured gauge
        // length. The laser wavelength is assumed 1550 nm (C-band telecom).
        let wavelength_nm = 1550.0;
        let strain_rate = rayleigh_backscatter_to_strain(
            &backscatter[..n],
            self.config.gauge_length_m,
            wavelength_nm,
        );

        // Detect events
        let events = self.detect_events(&strain_rate, &depth_axis_m);

        DasFrame {
            strain_rate,
            depth_axis_m,
            events,
        }
    }

    /// Simple event detection based on amplitude thresholding and frequency
    /// estimation. Events whose amplitude exceeds 3 sigma of the frame RMS
    /// are flagged and classified by their dominant frequency content.
    fn detect_events(
        &self,
        strain_rate: &[f64],
        depth_axis_m: &[f64],
    ) -> Vec<AcousticEvent> {
        if strain_rate.is_empty() {
            return Vec::new();
        }

        // Compute RMS
        let sum_sq: f64 = strain_rate.iter().map(|&v| v * v).sum();
        let rms = (sum_sq / strain_rate.len() as f64).sqrt();
        let threshold = 3.0 * rms;

        let mut events = Vec::new();

        for (i, (&val, &depth)) in
            strain_rate.iter().zip(depth_axis_m.iter()).enumerate()
        {
            if val.abs() > threshold && threshold > 0.0 {
                // Estimate dominant frequency from zero-crossing rate in a small window
                let freq = self.estimate_frequency(strain_rate, i);
                let event_type = classify_event(freq, val.abs());
                events.push(AcousticEvent {
                    depth_m: depth,
                    amplitude: val.abs(),
                    frequency_hz: freq,
                    event_type,
                });
            }
        }

        events
    }

    /// Estimate the dominant frequency around index `center` using zero-crossing
    /// rate within a small window.
    fn estimate_frequency(&self, data: &[f64], center: usize) -> f64 {
        let half_win = 16.min(data.len() / 2);
        let start = center.saturating_sub(half_win);
        let end = (center + half_win).min(data.len());
        if end - start < 3 {
            return 0.0;
        }

        let mut crossings: usize = 0;
        for i in (start + 1)..end {
            if (data[i] >= 0.0) != (data[i - 1] >= 0.0) {
                crossings += 1;
            }
        }
        let duration_s = (end - start) as f64 / self.config.sample_rate_hz;
        if duration_s <= 0.0 {
            return 0.0;
        }
        // Each full cycle has two zero crossings
        crossings as f64 / (2.0 * duration_s)
    }
}

// ---------------------------------------------------------------------------
// Event classification heuristic
// ---------------------------------------------------------------------------

/// Classify an acoustic event based on its dominant frequency and amplitude.
///
/// Rough heuristic boundaries (typical wellbore DAS):
/// - < 10 Hz  : CasingDamage (structural resonance)
/// - 10-100 Hz: MicroseismicEvent (rock fracture)
/// - 100-500 Hz: FluidFlow (turbulent flow noise)
/// - 500-2000 Hz: LeakDetection (high-velocity jet noise)
/// - > 2000 Hz: SandProduction (impulsive grain impacts)
/// - Low amplitude catch-all: Noise
pub fn classify_event(frequency_hz: f64, amplitude: f64) -> EventType {
    if amplitude < 1e-15 {
        return EventType::Noise;
    }
    if frequency_hz < 10.0 {
        EventType::CasingDamage
    } else if frequency_hz < 100.0 {
        EventType::MicroseismicEvent
    } else if frequency_hz < 500.0 {
        EventType::FluidFlow
    } else if frequency_hz < 2000.0 {
        EventType::LeakDetection
    } else {
        EventType::SandProduction
    }
}

// ---------------------------------------------------------------------------
// Rayleigh backscatter to strain
// ---------------------------------------------------------------------------

/// Convert an array of optical phase changes (radians) measured over a gauge
/// length into strain values.
///
/// The relationship is:
///
/// ```text
/// epsilon = (lambda / (4 * pi * n_eff * L_g)) * delta_phi
/// ```
///
/// where `lambda` is laser wavelength, `n_eff` is effective refractive index
/// (assumed 1.468 for standard SMF-28), `L_g` is gauge length, and
/// `delta_phi` is the phase change in radians.
pub fn rayleigh_backscatter_to_strain(
    phase_change: &[f64],
    gauge_length_m: f64,
    wavelength_nm: f64,
) -> Vec<f64> {
    if gauge_length_m <= 0.0 {
        return vec![0.0; phase_change.len()];
    }
    let n_eff = 1.468; // effective refractive index for SMF-28
    let lambda_m = wavelength_nm * 1e-9;
    let scale = lambda_m / (4.0 * PI * n_eff * gauge_length_m);
    phase_change.iter().map(|&phi| scale * phi).collect()
}

// ---------------------------------------------------------------------------
// DTS temperature conversion
// ---------------------------------------------------------------------------

/// Convert Raman anti-Stokes/Stokes backscatter intensity ratios to a
/// temperature profile along the fiber.
///
/// The Raman ratio `R` relates to temperature `T` (in kelvin) by:
///
/// ```text
/// T = (h * delta_nu) / (k_B * ln(C / R))
/// ```
///
/// where `h` is Planck's constant, `k_B` is Boltzmann's constant, and
/// `delta_nu` is the Raman frequency shift (~13.0 THz for silica).
/// `C` is a calibration constant determined from a reference temperature.
///
/// This function returns temperatures in degrees Celsius, adding
/// `calibration_offset` (in Celsius) to each value for field calibration.
///
/// Values with non-positive ratio are clamped to absolute zero.
pub fn depth_to_temperature(
    backscatter_ratio: &[f64],
    calibration_offset: f64,
) -> Vec<f64> {
    // Physical constants
    let h: f64 = 6.626_070_15e-34; // Planck's constant (J*s)
    let k_b: f64 = 1.380_649e-23; // Boltzmann's constant (J/K)
    let delta_nu: f64 = 13.0e12; // Raman shift for silica (Hz)

    // Calibration constant C: determined so that R=1.0 corresponds to ~300 K.
    // From T = h*dv / (kB * ln(C/R)):  C = R * exp(h*dv / (kB*T_ref))
    let t_ref: f64 = 300.0; // reference temperature (K)
    let c_cal = (h * delta_nu / (k_b * t_ref)).exp(); // C when R=1 at T_ref

    backscatter_ratio
        .iter()
        .map(|&r| {
            if r <= 0.0 {
                // Non-physical; return absolute zero in Celsius
                -273.15 + calibration_offset
            } else {
                let ln_arg = c_cal / r;
                if ln_arg <= 0.0 {
                    -273.15 + calibration_offset
                } else {
                    let t_kelvin = (h * delta_nu) / (k_b * ln_arg.ln());
                    (t_kelvin - 273.15) + calibration_offset
                }
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Flow event detection
// ---------------------------------------------------------------------------

/// Detect flow noise events in a strain-rate trace.
///
/// Returns a vector of `(index, amplitude)` pairs where the signal energy
/// in a sliding window exceeds the given `threshold`.
///
/// The detection uses a short-time energy measure: the RMS of a window of
/// length `window_samples = fs / 10` (100 ms window) is compared against
/// `threshold`.
pub fn detect_flow_events(
    strain_rate: &[f64],
    fs: f64,
    threshold: f64,
) -> Vec<(usize, f64)> {
    if strain_rate.is_empty() || fs <= 0.0 {
        return Vec::new();
    }

    let window_samples = ((fs / 10.0).round() as usize).max(1);
    let mut results = Vec::new();

    let mut i = 0;
    while i + window_samples <= strain_rate.len() {
        let sum_sq: f64 = strain_rate[i..i + window_samples]
            .iter()
            .map(|&v| v * v)
            .sum();
        let rms = (sum_sq / window_samples as f64).sqrt();

        if rms > threshold {
            // Record the peak sample within the window
            let (peak_idx, peak_val) = strain_rate[i..i + window_samples]
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| {
                    a.abs().partial_cmp(&b.abs()).unwrap_or(std::cmp::Ordering::Equal)
                })
                .map(|(j, &v)| (i + j, v.abs()))
                .unwrap();
            results.push((peak_idx, peak_val));
            // Skip past this window to avoid duplicate detections
            i += window_samples;
        } else {
            i += 1;
        }
    }

    results
}

// ---------------------------------------------------------------------------
// Vertical Seismic Profile (VSP)
// ---------------------------------------------------------------------------

/// Compute a Vertical Seismic Profile stack from DAS data.
///
/// `das_data` is a 2-D array where each inner `Vec<f64>` is a time trace
/// for one depth channel. `source_depth_m` is the depth of the seismic
/// source (used for direct-arrival alignment). `fs` is the temporal sample
/// rate.
///
/// The function:
/// 1. Estimates direct-arrival time per channel using first-break picking
///    (first sample exceeding a fraction of the trace maximum).
/// 2. Aligns traces to the picked first break.
/// 3. Stacks (sums) aligned traces to enhance signal-to-noise ratio.
///
/// Returns the stacked trace.
pub fn compute_vsp(
    das_data: &[Vec<f64>],
    _source_depth_m: f64,
    _fs: f64,
) -> Vec<f64> {
    if das_data.is_empty() {
        return Vec::new();
    }

    let max_len = das_data.iter().map(|t| t.len()).max().unwrap_or(0);
    if max_len == 0 {
        return Vec::new();
    }

    // First-break picking: find the first sample that exceeds 10% of the
    // trace peak amplitude.
    let pick_fraction = 0.10;
    let mut first_breaks: Vec<usize> = Vec::with_capacity(das_data.len());

    for trace in das_data {
        let peak = trace
            .iter()
            .map(|v| v.abs())
            .fold(0.0_f64, f64::max);
        let fb_threshold = pick_fraction * peak;
        let fb = trace
            .iter()
            .position(|&v| v.abs() >= fb_threshold)
            .unwrap_or(0);
        first_breaks.push(fb);
    }

    // Align to the minimum first-break time
    let min_fb = *first_breaks.iter().min().unwrap_or(&0);

    let mut stack = vec![0.0_f64; max_len];
    let mut count = vec![0u32; max_len];

    for (trace, &fb) in das_data.iter().zip(first_breaks.iter()) {
        let shift = fb - min_fb;
        for (j, &val) in trace.iter().enumerate() {
            if j >= shift {
                let out_idx = j - shift;
                if out_idx < max_len {
                    stack[out_idx] += val;
                    count[out_idx] += 1;
                }
            }
        }
    }

    // Normalize
    for (s, &c) in stack.iter_mut().zip(count.iter()) {
        if c > 0 {
            *s /= c as f64;
        }
    }

    stack
}

// ---------------------------------------------------------------------------
// F-K (frequency-wavenumber) filter
// ---------------------------------------------------------------------------

/// Apply frequency-wavenumber (F-K) velocity filtering to a 2-D DAS gather.
///
/// `das_data` is indexed as `[depth_channel][time_sample]`.
/// `fs` is the temporal sample rate (Hz), `spatial_res_m` is the distance
/// between depth channels (m), and `velocity_min` / `velocity_max` define the
/// apparent-velocity pass band (m/s).
///
/// The algorithm:
/// 1. Applies a 2-D DFT (time and space dimensions).
/// 2. Masks the F-K plane to zero out energy whose apparent velocity
///    `v = f / k` falls outside `[velocity_min, velocity_max]`.
/// 3. Applies the inverse 2-D DFT.
///
/// The DFT is computed from scratch using the Cooley-Tukey radix-2 FFT.
pub fn fk_filter(
    das_data: &[Vec<f64>],
    fs: f64,
    spatial_res_m: f64,
    velocity_min: f64,
    velocity_max: f64,
) -> Vec<Vec<f64>> {
    if das_data.is_empty() {
        return Vec::new();
    }

    let n_channels = das_data.len();
    let n_samples = das_data.iter().map(|t| t.len()).max().unwrap_or(0);
    if n_samples == 0 {
        return vec![Vec::new(); n_channels];
    }

    // Pad to next power of 2 for both dimensions
    let nt = next_power_of_2(n_samples);
    let nx = next_power_of_2(n_channels);

    // Build zero-padded complex 2-D array (row = space, col = time)
    let mut data: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); nt]; nx];
    for (i, trace) in das_data.iter().enumerate() {
        for (j, &val) in trace.iter().enumerate() {
            data[i][j] = (val, 0.0);
        }
    }

    // Forward 2-D FFT: FFT along time (columns), then along space (rows)
    // Time FFT (each row)
    for row in data.iter_mut() {
        fft_in_place(row, false);
    }
    // Space FFT (each column)
    for j in 0..nt {
        let mut col: Vec<(f64, f64)> = (0..nx).map(|i| data[i][j]).collect();
        fft_in_place(&mut col, false);
        for (i, &val) in col.iter().enumerate() {
            data[i][j] = val;
        }
    }

    // Apply velocity mask in the F-K domain
    let df = fs / nt as f64;
    let dk = 1.0 / (spatial_res_m * nx as f64);

    for i in 0..nx {
        // wavenumber for bin i
        let k_idx = if i <= nx / 2 { i as f64 } else { i as f64 - nx as f64 };
        let k = k_idx * dk; // cycles/m

        for j in 0..nt {
            // frequency for bin j
            let f_idx = if j <= nt / 2 { j as f64 } else { j as f64 - nt as f64 };
            let f = f_idx * df; // Hz

            // Apparent velocity v = f / k  (handle k == 0 separately)
            let pass = if k.abs() < 1e-30 {
                // DC wavenumber: pass only DC frequency
                f.abs() < df
            } else {
                let v = (f / k).abs();
                v >= velocity_min && v <= velocity_max
            };

            if !pass {
                data[i][j] = (0.0, 0.0);
            }
        }
    }

    // Inverse 2-D FFT: IFFT along space (columns), then along time (rows)
    for j in 0..nt {
        let mut col: Vec<(f64, f64)> = (0..nx).map(|i| data[i][j]).collect();
        fft_in_place(&mut col, true);
        for (i, &val) in col.iter().enumerate() {
            data[i][j] = val;
        }
    }
    for row in data.iter_mut() {
        fft_in_place(row, true);
    }

    // Extract real parts for the original dimensions
    let mut result = Vec::with_capacity(n_channels);
    for i in 0..n_channels {
        let row: Vec<f64> = data[i][..n_samples].iter().map(|&(re, _im)| re).collect();
        result.push(row);
    }

    result
}

// ---------------------------------------------------------------------------
// Microseismic locator
// ---------------------------------------------------------------------------

/// Locate a microseismic event from first-arrival times measured at receivers
/// along the wellbore.
///
/// Uses a simple moveout-based grid search. The model assumes a homogeneous
/// velocity medium with P-wave velocity `velocity_mps`.
///
/// `event_times` are the first-arrival times (seconds) at each receiver.
/// `receiver_depths` are the measured depths of the receivers (metres).
///
/// Returns `(source_depth_m, lateral_offset_m)` that best fits the observed
/// travel-time moveout.
pub fn microseismic_locator(
    event_times: &[f64],
    receiver_depths: &[f64],
    velocity_mps: f64,
) -> (f64, f64) {
    assert_eq!(
        event_times.len(),
        receiver_depths.len(),
        "event_times and receiver_depths must have the same length"
    );

    if event_times.is_empty() || velocity_mps <= 0.0 {
        return (0.0, 0.0);
    }

    // Reference time: the earliest arrival
    let t_min = event_times
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);

    let depth_min = receiver_depths
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);
    let depth_max = receiver_depths
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    let depth_range = (depth_max - depth_min).max(1.0);

    // Grid search over (source_depth, offset)
    let n_depth_steps = 100;
    let n_offset_steps = 50;
    let max_offset = depth_range; // reasonable lateral search range

    let mut best_depth = 0.0;
    let mut best_offset = 0.0;
    let mut best_misfit = f64::INFINITY;

    for di in 0..=n_depth_steps {
        let trial_depth =
            depth_min - depth_range * 0.1 + (depth_range * 1.2) * (di as f64 / n_depth_steps as f64);

        for oi in 0..=n_offset_steps {
            let trial_offset = max_offset * (oi as f64 / n_offset_steps as f64);

            // Compute predicted travel times
            let mut misfit = 0.0_f64;
            for ((&t_obs, &z_r)) in event_times.iter().zip(receiver_depths.iter()) {
                let dz = trial_depth - z_r;
                let dist = (dz * dz + trial_offset * trial_offset).sqrt();
                let t_pred = dist / velocity_mps;
                // Use differential times relative to earliest arrival
                let dt_obs = t_obs - t_min;
                // We also need to find predicted minimum, so compute all first
                // and compare. For simplicity, accumulate squared residuals of
                // (t_obs - t_min) vs (t_pred - t_pred_min_candidate). This is
                // handled below.
                let _ = dt_obs;
                let _ = t_pred;
                misfit += 0.0; // placeholder, overridden below
            }

            // Compute predicted times for all receivers at once
            let predicted: Vec<f64> = receiver_depths
                .iter()
                .map(|&z_r| {
                    let dz = trial_depth - z_r;
                    (dz * dz + trial_offset * trial_offset).sqrt() / velocity_mps
                })
                .collect();

            let t_pred_min = predicted
                .iter()
                .cloned()
                .fold(f64::INFINITY, f64::min);

            misfit = 0.0;
            for (&t_obs, &t_pred) in event_times.iter().zip(predicted.iter()) {
                let dt_obs = t_obs - t_min;
                let dt_pred = t_pred - t_pred_min;
                let residual = dt_obs - dt_pred;
                misfit += residual * residual;
            }

            if misfit < best_misfit {
                best_misfit = misfit;
                best_depth = trial_depth;
                best_offset = trial_offset;
            }
        }
    }

    (best_depth, best_offset)
}

// ---------------------------------------------------------------------------
// Optical budget
// ---------------------------------------------------------------------------

/// Compute the optical power budget for a fiber optic DAS installation.
///
/// Standard loss values used:
/// - Fiber attenuation: 0.2 dB/km (typical single-mode at 1550 nm)
/// - Fusion splice loss: 0.05 dB per splice
/// - Connector loss: 0.3 dB per connector (SC/APC or similar)
pub fn optical_budget(
    fiber_length_km: f64,
    splice_count: usize,
    connector_count: usize,
) -> OpticalBudget {
    let fiber_atten_db_per_km = 0.2;
    let splice_loss_db_each = 0.05;
    let connector_loss_db_each = 0.3;

    let fiber_loss_db = fiber_length_km * fiber_atten_db_per_km;
    let splice_loss_db = splice_count as f64 * splice_loss_db_each;
    let connector_loss_db = connector_count as f64 * connector_loss_db_each;
    let total_loss_db = fiber_loss_db + splice_loss_db + connector_loss_db;

    OpticalBudget {
        total_loss_db,
        fiber_loss_db,
        splice_loss_db,
        connector_loss_db,
    }
}

// ---------------------------------------------------------------------------
// Internal helpers: FFT from scratch (Cooley-Tukey radix-2)
// ---------------------------------------------------------------------------

fn next_power_of_2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// In-place Cooley-Tukey radix-2 FFT on complex data.
/// `inverse`: if true, computes the IFFT (including 1/N scaling).
fn fft_in_place(data: &mut [(f64, f64)], inverse: bool) {
    let n = data.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of 2");

    // Bit-reversal permutation
    let mut j: usize = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            data.swap(i, j);
        }
    }

    // Butterfly passes
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let w_base = (angle.cos(), angle.sin());

        let mut i = 0;
        while i < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = data[i + k];
                let t = complex_mul(w, data[i + k + half]);
                data[i + k] = (u.0 + t.0, u.1 + t.1);
                data[i + k + half] = (u.0 - t.0, u.1 - t.1);
                w = complex_mul(w, w_base);
            }
            i += len;
        }
        len <<= 1;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for d in data.iter_mut() {
            d.0 *= scale;
            d.1 *= scale;
        }
    }
}

#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    // -- DasConfig default --------------------------------------------------

    #[test]
    fn test_das_config_default() {
        let cfg = DasConfig::default();
        assert!((cfg.fiber_length_m - 3000.0).abs() < EPSILON);
        assert!((cfg.spatial_resolution_m - 1.0).abs() < EPSILON);
        assert!((cfg.sample_rate_hz - 10000.0).abs() < EPSILON);
        assert!((cfg.gauge_length_m - 10.0).abs() < EPSILON);
        assert!((cfg.pulse_rate_hz - 5000.0).abs() < EPSILON);
    }

    // -- DasProcessor basic frame -------------------------------------------

    #[test]
    fn test_das_processor_basic_frame() {
        let config = DasConfig {
            fiber_length_m: 100.0,
            spatial_resolution_m: 1.0,
            sample_rate_hz: 1000.0,
            gauge_length_m: 10.0,
            pulse_rate_hz: 500.0,
        };
        let proc = DasProcessor::new(config);
        let backscatter = vec![0.0; 100];
        let frame = proc.process_frame(&backscatter);
        assert_eq!(frame.strain_rate.len(), 100);
        assert_eq!(frame.depth_axis_m.len(), 100);
        // All zeros -> no strain
        for &v in &frame.strain_rate {
            assert!(v.abs() < EPSILON);
        }
    }

    #[test]
    fn test_das_processor_depth_axis() {
        let config = DasConfig {
            fiber_length_m: 50.0,
            spatial_resolution_m: 2.0,
            sample_rate_hz: 1000.0,
            gauge_length_m: 10.0,
            pulse_rate_hz: 500.0,
        };
        let proc = DasProcessor::new(config);
        // 50m / 2m = 25 channels
        let backscatter = vec![0.1; 25];
        let frame = proc.process_frame(&backscatter);
        assert_eq!(frame.depth_axis_m.len(), 25);
        assert!((frame.depth_axis_m[0] - 0.0).abs() < EPSILON);
        assert!((frame.depth_axis_m[1] - 2.0).abs() < EPSILON);
        assert!((frame.depth_axis_m[24] - 48.0).abs() < EPSILON);
    }

    // -- Strain conversion ---------------------------------------------------

    #[test]
    fn test_rayleigh_strain_zero_phase() {
        let phase = vec![0.0; 10];
        let strain = rayleigh_backscatter_to_strain(&phase, 10.0, 1550.0);
        assert_eq!(strain.len(), 10);
        for &s in &strain {
            assert!(s.abs() < EPSILON);
        }
    }

    #[test]
    fn test_rayleigh_strain_nonzero() {
        // Known conversion: epsilon = lambda / (4*pi*n_eff*Lg) * phi
        let lambda_m = 1550.0e-9;
        let n_eff = 1.468;
        let gauge = 10.0;
        let expected_scale = lambda_m / (4.0 * PI * n_eff * gauge);

        let phase = vec![1.0, -1.0, 2.0];
        let strain = rayleigh_backscatter_to_strain(&phase, gauge, 1550.0);
        assert!((strain[0] - expected_scale).abs() < 1e-20);
        assert!((strain[1] + expected_scale).abs() < 1e-20);
        assert!((strain[2] - 2.0 * expected_scale).abs() < 1e-20);
    }

    #[test]
    fn test_rayleigh_strain_zero_gauge() {
        // Zero gauge length should return zeros (guarded)
        let phase = vec![1.0, 2.0, 3.0];
        let strain = rayleigh_backscatter_to_strain(&phase, 0.0, 1550.0);
        for &s in &strain {
            assert!(s.abs() < EPSILON);
        }
    }

    #[test]
    fn test_rayleigh_strain_different_wavelength() {
        let phase = vec![1.0];
        let s1 = rayleigh_backscatter_to_strain(&phase, 10.0, 1550.0)[0];
        let s2 = rayleigh_backscatter_to_strain(&phase, 10.0, 1310.0)[0];
        // Shorter wavelength -> smaller strain for same phase change
        assert!(s2 < s1);
    }

    // -- Temperature conversion ----------------------------------------------

    #[test]
    fn test_temperature_reference_point() {
        // R = 1.0 should give ~300 K = ~26.85 C (with zero calibration offset)
        let temps = depth_to_temperature(&[1.0], 0.0);
        assert_eq!(temps.len(), 1);
        // Should be close to 26.85 C (300 K - 273.15)
        assert!((temps[0] - 26.85).abs() < 0.1);
    }

    #[test]
    fn test_temperature_calibration_offset() {
        let temps_no_offset = depth_to_temperature(&[1.0], 0.0);
        let temps_with_offset = depth_to_temperature(&[1.0], 5.0);
        assert!((temps_with_offset[0] - temps_no_offset[0] - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_temperature_negative_ratio() {
        // Non-physical ratio should return absolute zero + offset
        let temps = depth_to_temperature(&[-0.5], 0.0);
        assert!((temps[0] - (-273.15)).abs() < EPSILON);
    }

    #[test]
    fn test_temperature_monotonic_with_ratio() {
        // Higher Raman ratio -> higher temperature
        let ratios: Vec<f64> = (1..=10).map(|i| i as f64 * 0.5).collect();
        let temps = depth_to_temperature(&ratios, 0.0);
        for i in 1..temps.len() {
            assert!(
                temps[i] > temps[i - 1],
                "Temperature should increase with Raman ratio: T[{}]={} <= T[{}]={}",
                i, temps[i], i - 1, temps[i - 1]
            );
        }
    }

    // -- Flow event detection ------------------------------------------------

    #[test]
    fn test_detect_flow_events_empty() {
        let events = detect_flow_events(&[], 1000.0, 0.1);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_flow_events_below_threshold() {
        let data = vec![0.001; 200];
        let events = detect_flow_events(&data, 1000.0, 0.1);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_flow_events_above_threshold() {
        let mut data = vec![0.0; 1000];
        // Insert a burst of flow noise
        for i in 200..400 {
            data[i] = 1.0;
        }
        let events = detect_flow_events(&data, 1000.0, 0.5);
        assert!(
            !events.is_empty(),
            "Should detect at least one flow event"
        );
        // The detected event should be somewhere in the 200-400 range
        let (idx, amp) = events[0];
        assert!(idx >= 200 && idx < 400);
        assert!((amp - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_detect_flow_events_multiple_bursts() {
        let mut data = vec![0.0; 2000];
        // Two distinct bursts
        for i in 100..200 {
            data[i] = 2.0;
        }
        for i in 1500..1600 {
            data[i] = 3.0;
        }
        let events = detect_flow_events(&data, 1000.0, 1.0);
        assert!(
            events.len() >= 2,
            "Should detect at least 2 flow events, got {}",
            events.len()
        );
    }

    // -- VSP computation -----------------------------------------------------

    #[test]
    fn test_vsp_empty() {
        let result = compute_vsp(&[], 0.0, 1000.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_vsp_single_trace() {
        let trace = vec![0.0, 0.0, 1.0, 0.5, 0.2, 0.0];
        let result = compute_vsp(&[trace.clone()], 0.0, 1000.0);
        assert_eq!(result.len(), trace.len());
        // Single trace -> output should match input (aligned to itself)
        for (a, b) in result.iter().zip(trace.iter()) {
            assert!((a - b).abs() < 1e-6);
        }
    }

    #[test]
    fn test_vsp_alignment() {
        // Two traces with different first-break times should be aligned
        let trace1 = vec![0.0, 0.0, 1.0, 0.5, 0.0, 0.0];
        let trace2 = vec![0.0, 0.0, 0.0, 1.0, 0.5, 0.0];
        let result = compute_vsp(&[trace1, trace2], 0.0, 1000.0);
        // After alignment, the stack should have enhanced amplitude near the peak
        let peak = result.iter().cloned().fold(0.0_f64, f64::max);
        assert!(peak > 0.0, "Stacked VSP should have a positive peak");
    }

    // -- F-K filter ----------------------------------------------------------

    #[test]
    fn test_fk_filter_empty() {
        let result = fk_filter(&[], 1000.0, 1.0, 100.0, 5000.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_fk_filter_preserves_dimensions() {
        // 4 channels x 8 samples
        let data: Vec<Vec<f64>> = (0..4)
            .map(|_| vec![0.0; 8])
            .collect();
        let result = fk_filter(&data, 1000.0, 1.0, 100.0, 5000.0);
        assert_eq!(result.len(), 4);
        for row in &result {
            assert_eq!(row.len(), 8);
        }
    }

    #[test]
    fn test_fk_filter_passthrough_dc() {
        // A constant signal (DC) should be mostly preserved through the filter
        // when velocity range includes very high velocities (DC has infinite apparent velocity,
        // but our filter passes DC wavenumber for DC frequency).
        let data: Vec<Vec<f64>> = (0..4)
            .map(|_| vec![1.0; 8])
            .collect();
        let result = fk_filter(&data, 1000.0, 1.0, 100.0, 100000.0);
        // The DC component should survive
        let mean: f64 = result.iter().flat_map(|r| r.iter()).sum::<f64>()
            / (result.len() * result[0].len()) as f64;
        assert!(
            mean.abs() > 0.5,
            "DC component should be largely preserved, got mean={}",
            mean
        );
    }

    #[test]
    fn test_fk_filter_attenuates_slow_velocity() {
        // Create a signal moving at a very slow apparent velocity (steep moveout)
        // and filter for fast velocities only. Energy should be reduced.
        let n_channels = 8;
        let n_samples = 16;
        let mut data: Vec<Vec<f64>> = vec![vec![0.0; n_samples]; n_channels];

        // Impulse that moves 1 sample per channel -> slow apparent velocity
        for ch in 0..n_channels {
            let idx = ch.min(n_samples - 1);
            data[ch][idx] = 1.0;
        }

        let energy_before: f64 = data.iter()
            .flat_map(|r| r.iter())
            .map(|v| v * v)
            .sum();

        let result = fk_filter(&data, 1000.0, 1.0, 5000.0, 100000.0);

        let energy_after: f64 = result.iter()
            .flat_map(|r| r.iter())
            .map(|v| v * v)
            .sum();

        assert!(
            energy_after < energy_before * 0.8,
            "F-K filter should attenuate slow-velocity signal: before={}, after={}",
            energy_before, energy_after
        );
    }

    // -- Microseismic locator ------------------------------------------------

    #[test]
    fn test_microseismic_locator_centered() {
        // Source directly on the array (zero offset) at the center depth
        let velocity = 3000.0; // m/s
        let source_depth = 500.0;
        let offsets = [0.0_f64]; // true offset is 0

        let receiver_depths: Vec<f64> = (0..20).map(|i| 400.0 + i as f64 * 10.0).collect();
        let event_times: Vec<f64> = receiver_depths
            .iter()
            .map(|&z| {
                let dz = source_depth - z;
                (dz * dz + offsets[0] * offsets[0]).sqrt() / velocity
            })
            .collect();

        let (est_depth, est_offset) =
            microseismic_locator(&event_times, &receiver_depths, velocity);

        assert!(
            (est_depth - source_depth).abs() < 20.0,
            "Estimated depth {} should be near {} m",
            est_depth, source_depth
        );
        assert!(
            est_offset < 50.0,
            "Estimated offset {} should be near 0 m",
            est_offset
        );
    }

    #[test]
    fn test_microseismic_locator_with_offset() {
        let velocity = 3000.0;
        let source_depth = 600.0;
        let source_offset = 100.0;

        let receiver_depths: Vec<f64> = (0..30).map(|i| 400.0 + i as f64 * 10.0).collect();
        let event_times: Vec<f64> = receiver_depths
            .iter()
            .map(|&z| {
                let dz = source_depth - z;
                (dz * dz + source_offset * source_offset).sqrt() / velocity
            })
            .collect();

        let (est_depth, est_offset) =
            microseismic_locator(&event_times, &receiver_depths, velocity);

        assert!(
            (est_depth - source_depth).abs() < 30.0,
            "Estimated depth {} should be near {} m",
            est_depth, source_depth
        );
        assert!(
            (est_offset - source_offset).abs() < 40.0,
            "Estimated offset {} should be near {} m",
            est_offset, source_offset
        );
    }

    // -- Optical budget ------------------------------------------------------

    #[test]
    fn test_optical_budget_basic() {
        let budget = optical_budget(10.0, 5, 2);
        // 10 km * 0.2 dB/km = 2.0 dB fiber loss
        assert!((budget.fiber_loss_db - 2.0).abs() < EPSILON);
        // 5 splices * 0.05 dB = 0.25 dB
        assert!((budget.splice_loss_db - 0.25).abs() < EPSILON);
        // 2 connectors * 0.3 dB = 0.6 dB
        assert!((budget.connector_loss_db - 0.6).abs() < EPSILON);
        // Total = 2.0 + 0.25 + 0.6 = 2.85 dB
        assert!((budget.total_loss_db - 2.85).abs() < EPSILON);
    }

    #[test]
    fn test_optical_budget_zero_length() {
        let budget = optical_budget(0.0, 0, 0);
        assert!(budget.total_loss_db.abs() < EPSILON);
    }

    #[test]
    fn test_optical_budget_long_run() {
        // A 50 km run with 20 splices and 4 connectors (typical offshore)
        let budget = optical_budget(50.0, 20, 4);
        assert!((budget.fiber_loss_db - 10.0).abs() < EPSILON);
        assert!((budget.splice_loss_db - 1.0).abs() < EPSILON);
        assert!((budget.connector_loss_db - 1.2).abs() < EPSILON);
        assert!((budget.total_loss_db - 12.2).abs() < EPSILON);
    }

    // -- Event classification ------------------------------------------------

    #[test]
    fn test_classify_fluid_flow() {
        let et = classify_event(250.0, 1.0);
        assert_eq!(et, EventType::FluidFlow);
    }

    #[test]
    fn test_classify_sand_production() {
        let et = classify_event(3000.0, 1.0);
        assert_eq!(et, EventType::SandProduction);
    }

    #[test]
    fn test_classify_leak() {
        let et = classify_event(1000.0, 1.0);
        assert_eq!(et, EventType::LeakDetection);
    }

    #[test]
    fn test_classify_microseismic() {
        let et = classify_event(50.0, 1.0);
        assert_eq!(et, EventType::MicroseismicEvent);
    }

    #[test]
    fn test_classify_casing_damage() {
        let et = classify_event(5.0, 1.0);
        assert_eq!(et, EventType::CasingDamage);
    }

    #[test]
    fn test_classify_noise_low_amplitude() {
        let et = classify_event(250.0, 0.0);
        assert_eq!(et, EventType::Noise);
    }

    // -- FFT helper ----------------------------------------------------------

    #[test]
    fn test_fft_roundtrip() {
        let mut data: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (4.0, 0.0),
        ];
        let original = data.clone();
        fft_in_place(&mut data, false);
        fft_in_place(&mut data, true);
        for (a, b) in data.iter().zip(original.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10, "FFT roundtrip failed: {} vs {}", a.0, b.0);
            assert!((a.1 - b.1).abs() < 1e-10);
        }
    }

    // -- Edge cases -----------------------------------------------------------

    #[test]
    fn test_das_processor_oversized_input() {
        let config = DasConfig {
            fiber_length_m: 10.0,
            spatial_resolution_m: 1.0,
            sample_rate_hz: 1000.0,
            gauge_length_m: 10.0,
            pulse_rate_hz: 500.0,
        };
        let proc = DasProcessor::new(config);
        // Provide more samples than channels: should be clamped
        let backscatter = vec![0.1; 100];
        let frame = proc.process_frame(&backscatter);
        assert_eq!(frame.strain_rate.len(), 10);
        assert_eq!(frame.depth_axis_m.len(), 10);
    }

    #[test]
    fn test_das_processor_undersized_input() {
        let config = DasConfig {
            fiber_length_m: 100.0,
            spatial_resolution_m: 1.0,
            sample_rate_hz: 1000.0,
            gauge_length_m: 10.0,
            pulse_rate_hz: 500.0,
        };
        let proc = DasProcessor::new(config);
        // Provide fewer samples than channels
        let backscatter = vec![0.1; 5];
        let frame = proc.process_frame(&backscatter);
        assert_eq!(frame.strain_rate.len(), 5);
        assert_eq!(frame.depth_axis_m.len(), 5);
    }

    #[test]
    fn test_event_detection_with_spike() {
        let config = DasConfig {
            fiber_length_m: 100.0,
            spatial_resolution_m: 1.0,
            sample_rate_hz: 10000.0,
            gauge_length_m: 10.0,
            pulse_rate_hz: 5000.0,
        };
        let proc = DasProcessor::new(config);

        // Create data with a large spike at channel 50
        let mut backscatter = vec![0.001; 100];
        backscatter[50] = 100.0; // huge spike

        let frame = proc.process_frame(&backscatter);
        // Should detect at least one event at or near depth 50m
        assert!(
            !frame.events.is_empty(),
            "Should detect the spike as an event"
        );
        let has_event_near_50 = frame
            .events
            .iter()
            .any(|e| (e.depth_m - 50.0).abs() < 2.0);
        assert!(has_event_near_50, "Event should be near depth 50 m");
    }

    #[test]
    fn test_detect_flow_events_zero_fs() {
        // Zero sample rate should return empty gracefully
        let events = detect_flow_events(&[1.0, 2.0, 3.0], 0.0, 0.1);
        assert!(events.is_empty());
    }

    #[test]
    fn test_next_power_of_2() {
        assert_eq!(next_power_of_2(1), 1);
        assert_eq!(next_power_of_2(2), 2);
        assert_eq!(next_power_of_2(3), 4);
        assert_eq!(next_power_of_2(5), 8);
        assert_eq!(next_power_of_2(16), 16);
        assert_eq!(next_power_of_2(17), 32);
    }
}
