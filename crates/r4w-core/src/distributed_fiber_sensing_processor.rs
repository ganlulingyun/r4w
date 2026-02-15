//! Distributed fiber sensing processor for DAS and DTS applications.
//!
//! This module implements phase-modulated backscatter demodulation and spatial
//! correlation analysis for Distributed Acoustic Sensing (DAS) and Distributed
//! Temperature Sensing (DTS) along optical fiber. It covers Rayleigh backscatter
//! processing, phase demodulation from phase-OTDR data, spatial filtering with
//! configurable gauge lengths, event detection and classification, and Raman-based
//! distributed temperature extraction.
//!
//! # Physical Background
//!
//! Distributed fiber sensing exploits intrinsic scattering mechanisms in optical
//! fiber to measure physical quantities along the entire fiber length:
//!
//! - **Rayleigh backscatter**: Elastic scattering from refractive index
//!   inhomogeneities. Used in phase-OTDR for acoustic/vibration sensing (DAS).
//!   The phase of the backscattered light encodes strain perturbations.
//!
//! - **Raman scattering**: Inelastic scattering producing anti-Stokes and Stokes
//!   components. The ratio of anti-Stokes to Stokes intensities is temperature-
//!   dependent, enabling distributed temperature measurement (DTS).
//!
//! # Spatial Resolution
//!
//! The spatial resolution of a distributed sensor is determined by the optical
//! pulse width:
//!
//! ```text
//! Δz = c · τ / (2 · n)
//! ```
//!
//! where `c` is the speed of light, `τ` is the pulse width, and `n` is the
//! group refractive index of the fiber. The factor of 2 accounts for the
//! round-trip propagation.
//!
//! # Example
//!
//! ```
//! use r4w_core::distributed_fiber_sensing_processor::{
//!     FiberConfig, RayleighBackscatterProcessor, PhaseDemodulator,
//!     SpatialFilter, EventDetector, TemperatureProfiler,
//!     group_velocity, spatial_resolution, rayleigh_coefficient, phase_unwrap,
//! };
//!
//! // Configure a 10 km fiber
//! let config = FiberConfig {
//!     fiber_length_m: 10_000.0,
//!     spatial_resolution_m: 1.0,
//!     pulse_width_s: 10e-9,
//!     refractive_index: 1.4681,
//!     attenuation_db_per_km: 0.2,
//!     rayleigh_scatter_coeff: 1e-4,
//!     sample_rate_hz: 100e6,
//! };
//!
//! // Process a raw backscatter trace
//! let processor = RayleighBackscatterProcessor::new(config.clone());
//! let raw_trace: Vec<f64> = (0..1000).map(|i| {
//!     let d = i as f64 * 10.0;
//!     let atten = (-0.2 * d / (1000.0 * 10.0 * (10.0_f64).log10())).exp();
//!     atten * (1.0 + 0.05 * (i as f64 * 0.1).sin())
//! }).collect();
//! let processed = processor.process_trace(&raw_trace);
//! assert_eq!(processed.len(), raw_trace.len());
//!
//! // Phase demodulation
//! let demod = PhaseDemodulator::new(config.clone());
//! let iq_i: Vec<f64> = (0..500).map(|i| (i as f64 * 0.01).cos()).collect();
//! let iq_q: Vec<f64> = (0..500).map(|i| (i as f64 * 0.01).sin()).collect();
//! let phase = demod.demodulate(&iq_i, &iq_q);
//! assert_eq!(phase.len(), 500);
//! ```

/// Speed of light in vacuum (m/s).
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Boltzmann constant (J/K).
const BOLTZMANN_K: f64 = 1.380649e-23;

/// Planck constant (J*s).
const PLANCK_H: f64 = 6.62607015e-34;

/// Standard temperature reference for DTS (K).
const T_REF: f64 = 300.0;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the group velocity of light in fiber.
///
/// Returns `c / n` where `c` is the speed of light in vacuum and `n` is the
/// group refractive index.
///
/// # Arguments
///
/// * `refractive_index` - Group refractive index of the fiber (typically ~1.468).
///
/// # Returns
///
/// Group velocity in m/s.
pub fn group_velocity(refractive_index: f64) -> f64 {
    SPEED_OF_LIGHT / refractive_index
}

/// Compute the spatial resolution of a distributed fiber sensor.
///
/// The spatial resolution is half the pulse length in fiber:
///
/// ```text
/// Δz = c · τ / (2 · n)
/// ```
///
/// # Arguments
///
/// * `pulse_width_s` - Optical pulse width in seconds.
/// * `refractive_index` - Group refractive index of the fiber.
///
/// # Returns
///
/// Spatial resolution in meters.
pub fn spatial_resolution(pulse_width_s: f64, refractive_index: f64) -> f64 {
    SPEED_OF_LIGHT * pulse_width_s / (2.0 * refractive_index)
}

/// Compute the Rayleigh backscatter coefficient for a given fiber.
///
/// The Rayleigh scattering coefficient (per meter) at wavelength λ is
/// approximately:
///
/// ```text
/// α_R = C / λ^4
/// ```
///
/// where `C` is a material constant for silica fiber (approximately
/// 0.7 dB/(km · μm⁴) for standard single-mode fiber, here converted to
/// linear per-meter units).
///
/// # Arguments
///
/// * `wavelength_m` - Laser wavelength in meters.
///
/// # Returns
///
/// Rayleigh scattering coefficient in 1/m (linear units).
pub fn rayleigh_coefficient(wavelength_m: f64) -> f64 {
    // Typical Rayleigh scattering constant for silica glass.
    // C ~ 0.7 dB/(km·μm^4) converted to Neper/m·m^4.
    // 0.7 dB/km = 0.7 / (10 * log10(e)) Np/km = 0.7 / 4.3429 Np/km
    // = 0.1612 Np/km = 1.612e-4 Np/m
    // In μm^4 units: C = 1.612e-4 * (1e-6)^4 = 1.612e-4 * 1e-24 = 1.612e-28
    let c_rayleigh = 1.612e-28; // Np/m · m^4
    let lambda4 = wavelength_m * wavelength_m * wavelength_m * wavelength_m;
    c_rayleigh / lambda4
}

/// Unwrap a phase signal by removing 2π discontinuities.
///
/// Scans through the input phase array and adds or subtracts multiples of
/// 2π so that the difference between consecutive samples never exceeds π
/// in magnitude.
///
/// # Arguments
///
/// * `phase` - Wrapped phase values in radians.
///
/// # Returns
///
/// Unwrapped phase values in radians.
pub fn phase_unwrap(phase: &[f64]) -> Vec<f64> {
    if phase.is_empty() {
        return Vec::new();
    }
    let mut unwrapped = Vec::with_capacity(phase.len());
    unwrapped.push(phase[0]);
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut cumulative_offset = 0.0;
    for i in 1..phase.len() {
        let mut diff = phase[i] - phase[i - 1];
        // Wrap diff to (-π, π]
        while diff > std::f64::consts::PI {
            diff -= two_pi;
        }
        while diff <= -std::f64::consts::PI {
            diff += two_pi;
        }
        cumulative_offset += diff;
        unwrapped.push(phase[0] + cumulative_offset);
    }
    unwrapped
}

// ---------------------------------------------------------------------------
// FiberConfig
// ---------------------------------------------------------------------------

/// Configuration parameters for a distributed fiber sensor.
///
/// Describes the physical properties of the sensing fiber, the interrogation
/// pulse, and the acquisition parameters.
#[derive(Debug, Clone)]
pub struct FiberConfig {
    /// Total fiber length in meters.
    pub fiber_length_m: f64,
    /// Target spatial resolution in meters.
    pub spatial_resolution_m: f64,
    /// Optical pulse width in seconds.
    pub pulse_width_s: f64,
    /// Group refractive index of the fiber core (typically ~1.4681 for SMF-28).
    pub refractive_index: f64,
    /// Fiber attenuation in dB/km.
    pub attenuation_db_per_km: f64,
    /// Rayleigh backscatter coefficient (fraction of power scattered per meter).
    pub rayleigh_scatter_coeff: f64,
    /// Digitizer sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl FiberConfig {
    /// Convert a sample index to distance along the fiber in meters.
    ///
    /// Uses the two-way travel time: `d = c * t / (2 * n)`.
    pub fn sample_to_distance(&self, sample_index: usize) -> f64 {
        let t = sample_index as f64 / self.sample_rate_hz;
        SPEED_OF_LIGHT * t / (2.0 * self.refractive_index)
    }

    /// Number of samples corresponding to the full fiber length.
    pub fn total_samples(&self) -> usize {
        let round_trip_time = 2.0 * self.fiber_length_m * self.refractive_index / SPEED_OF_LIGHT;
        (round_trip_time * self.sample_rate_hz).ceil() as usize
    }

    /// Number of samples per spatial resolution cell.
    pub fn samples_per_resolution(&self) -> usize {
        let cell_time = 2.0 * self.spatial_resolution_m * self.refractive_index / SPEED_OF_LIGHT;
        (cell_time * self.sample_rate_hz).round().max(1.0) as usize
    }
}

// ---------------------------------------------------------------------------
// RayleighBackscatterProcessor
// ---------------------------------------------------------------------------

/// Processes raw Rayleigh backscatter traces from a phase-OTDR system.
///
/// Performs trace normalization, attenuation compensation, and noise floor
/// estimation. The processor converts raw photodetector output to calibrated
/// backscatter power along the fiber.
#[derive(Debug, Clone)]
pub struct RayleighBackscatterProcessor {
    config: FiberConfig,
}

impl RayleighBackscatterProcessor {
    /// Create a new processor with the given fiber configuration.
    pub fn new(config: FiberConfig) -> Self {
        Self { config }
    }

    /// Process a raw backscatter intensity trace.
    ///
    /// Applies the following steps:
    /// 1. Convert to log scale (dB)
    /// 2. Compensate for fiber attenuation (2-way)
    /// 3. Subtract estimated noise floor
    ///
    /// # Arguments
    ///
    /// * `raw` - Raw backscatter intensity values (linear power units).
    ///
    /// # Returns
    ///
    /// Processed trace in dB, attenuation-compensated.
    pub fn process_trace(&self, raw: &[f64]) -> Vec<f64> {
        let n = raw.len();
        let mut result = Vec::with_capacity(n);

        // Convert to dB and compensate for round-trip attenuation
        let atten_per_m_db = self.config.attenuation_db_per_km / 1000.0;
        for i in 0..n {
            let distance = self.config.sample_to_distance(i);
            let power_db = if raw[i] > 1e-30 {
                10.0 * raw[i].log10()
            } else {
                -300.0
            };
            // Compensate round-trip attenuation (factor of 2)
            let compensated = power_db + 2.0 * atten_per_m_db * distance;
            result.push(compensated);
        }
        result
    }

    /// Average multiple backscatter traces to improve SNR.
    ///
    /// The SNR improvement is proportional to √N where N is the number of
    /// traces averaged.
    ///
    /// # Arguments
    ///
    /// * `traces` - Slice of traces, each the same length.
    ///
    /// # Returns
    ///
    /// Averaged trace, or empty vector if input is empty.
    pub fn average_traces(&self, traces: &[Vec<f64>]) -> Vec<f64> {
        if traces.is_empty() {
            return Vec::new();
        }
        let n = traces[0].len();
        let count = traces.len() as f64;
        let mut avg = vec![0.0; n];
        for trace in traces {
            for (i, &val) in trace.iter().enumerate().take(n) {
                avg[i] += val;
            }
        }
        for v in &mut avg {
            *v /= count;
        }
        avg
    }

    /// Estimate the noise floor of a processed trace.
    ///
    /// Uses the median of the last 10% of samples (beyond fiber end) as the
    /// noise estimate.
    ///
    /// # Arguments
    ///
    /// * `trace_db` - Processed trace in dB.
    ///
    /// # Returns
    ///
    /// Estimated noise floor in dB.
    pub fn estimate_noise_floor(&self, trace_db: &[f64]) -> f64 {
        if trace_db.is_empty() {
            return f64::NEG_INFINITY;
        }
        // Use last 10% of samples
        let start = (trace_db.len() as f64 * 0.9) as usize;
        let tail = &trace_db[start..];
        median(tail)
    }

    /// Compute the backscatter power at a given distance.
    ///
    /// # Arguments
    ///
    /// * `distance_m` - Distance along fiber in meters.
    ///
    /// # Returns
    ///
    /// Expected relative backscatter power in dB (normalized to launch power).
    pub fn expected_backscatter_db(&self, distance_m: f64) -> f64 {
        let atten_db = self.config.attenuation_db_per_km * distance_m / 1000.0;
        let scatter_db = 10.0 * self.config.rayleigh_scatter_coeff.log10();
        scatter_db - 2.0 * atten_db
    }
}

// ---------------------------------------------------------------------------
// PhaseDemodulator
// ---------------------------------------------------------------------------

/// Extracts acoustic/strain information from phase-OTDR I/Q data.
///
/// In a coherent phase-OTDR system, the backscattered light is mixed with a
/// local oscillator to produce in-phase (I) and quadrature (Q) components.
/// The instantaneous phase `φ = atan2(Q, I)` encodes the strain perturbation
/// along the fiber. This demodulator extracts, unwraps, and differentiates
/// the phase to produce strain-rate or acoustic signals.
#[derive(Debug, Clone)]
pub struct PhaseDemodulator {
    config: FiberConfig,
}

impl PhaseDemodulator {
    /// Create a new phase demodulator.
    pub fn new(config: FiberConfig) -> Self {
        Self { config }
    }

    /// Extract phase from I/Q backscatter data.
    ///
    /// Computes `atan2(Q, I)` for each sample, then unwraps the result.
    ///
    /// # Arguments
    ///
    /// * `i_data` - In-phase component samples.
    /// * `q_data` - Quadrature component samples (must be same length as `i_data`).
    ///
    /// # Returns
    ///
    /// Unwrapped phase in radians.
    ///
    /// # Panics
    ///
    /// Panics if `i_data` and `q_data` have different lengths.
    pub fn demodulate(&self, i_data: &[f64], q_data: &[f64]) -> Vec<f64> {
        assert_eq!(i_data.len(), q_data.len(), "I and Q data must have same length");
        let wrapped: Vec<f64> = i_data
            .iter()
            .zip(q_data.iter())
            .map(|(&i, &q)| q.atan2(i))
            .collect();
        phase_unwrap(&wrapped)
    }

    /// Compute the differential phase between consecutive spatial samples.
    ///
    /// This represents the strain-rate along the fiber. The gauge length is
    /// one sample spacing.
    ///
    /// # Arguments
    ///
    /// * `phase` - Unwrapped phase array (output from `demodulate`).
    ///
    /// # Returns
    ///
    /// Differential phase (length = input length - 1).
    pub fn differential_phase(&self, phase: &[f64]) -> Vec<f64> {
        if phase.len() < 2 {
            return Vec::new();
        }
        let mut diff = Vec::with_capacity(phase.len() - 1);
        for i in 1..phase.len() {
            diff.push(phase[i] - phase[i - 1]);
        }
        diff
    }

    /// Convert phase difference to strain (micro-strain).
    ///
    /// The relationship between phase shift and strain is:
    ///
    /// ```text
    /// ε = Δφ · λ / (4π · n · ξ · L)
    /// ```
    ///
    /// where λ is the laser wavelength, n is the refractive index, ξ is the
    /// strain-optic coefficient (~0.78 for silica), and L is the gauge length.
    ///
    /// # Arguments
    ///
    /// * `diff_phase` - Differential phase values in radians.
    /// * `wavelength_m` - Laser wavelength in meters.
    /// * `gauge_length_m` - Gauge length (distance between measurement points) in meters.
    ///
    /// # Returns
    ///
    /// Strain values in micro-strain (με).
    pub fn phase_to_strain(
        &self,
        diff_phase: &[f64],
        wavelength_m: f64,
        gauge_length_m: f64,
    ) -> Vec<f64> {
        let strain_optic = 0.78; // typical for silica fiber
        let n = self.config.refractive_index;
        let factor = wavelength_m
            / (4.0 * std::f64::consts::PI * n * strain_optic * gauge_length_m);
        diff_phase.iter().map(|&dp| dp * factor * 1e6).collect()
    }

    /// Compute amplitude from I/Q data.
    ///
    /// Returns `sqrt(I^2 + Q^2)` for each sample.
    pub fn amplitude(&self, i_data: &[f64], q_data: &[f64]) -> Vec<f64> {
        assert_eq!(i_data.len(), q_data.len(), "I and Q data must have same length");
        i_data
            .iter()
            .zip(q_data.iter())
            .map(|(&i, &q)| (i * i + q * q).sqrt())
            .collect()
    }
}

// ---------------------------------------------------------------------------
// SpatialFilter
// ---------------------------------------------------------------------------

/// Spatial filtering and gauge-length processing for distributed fiber data.
///
/// In DAS systems, the raw phase measurement has a gauge length equal to the
/// pulse width. This module provides tools to apply custom gauge lengths via
/// differencing, spatial averaging for noise reduction, and bandpass filtering
/// along the fiber axis.
#[derive(Debug, Clone)]
pub struct SpatialFilter {
    config: FiberConfig,
}

impl SpatialFilter {
    /// Create a new spatial filter.
    pub fn new(config: FiberConfig) -> Self {
        Self { config }
    }

    /// Apply gauge-length processing by differencing samples separated by
    /// the gauge length.
    ///
    /// The effective gauge length in samples is:
    ///
    /// ```text
    /// N_gauge = gauge_length_m / (c / (2 · n · fs))
    /// ```
    ///
    /// # Arguments
    ///
    /// * `data` - Input spatial data (e.g., unwrapped phase along fiber).
    /// * `gauge_length_m` - Desired gauge length in meters.
    ///
    /// # Returns
    ///
    /// Gauge-length processed data (length = input length - gauge samples).
    pub fn gauge_length(&self, data: &[f64], gauge_length_m: f64) -> Vec<f64> {
        let sample_spacing =
            SPEED_OF_LIGHT / (2.0 * self.config.refractive_index * self.config.sample_rate_hz);
        let gauge_samples = (gauge_length_m / sample_spacing).round().max(1.0) as usize;

        if data.len() <= gauge_samples {
            return Vec::new();
        }

        let mut result = Vec::with_capacity(data.len() - gauge_samples);
        for i in gauge_samples..data.len() {
            result.push(data[i] - data[i - gauge_samples]);
        }
        result
    }

    /// Apply spatial moving-average smoothing.
    ///
    /// Reduces speckle noise at the cost of spatial resolution.
    ///
    /// # Arguments
    ///
    /// * `data` - Input data along the fiber.
    /// * `window_size` - Number of samples in the averaging window.
    ///
    /// # Returns
    ///
    /// Smoothed data (same length as input, with edge handling via truncation).
    pub fn spatial_average(&self, data: &[f64], window_size: usize) -> Vec<f64> {
        if data.is_empty() || window_size == 0 {
            return data.to_vec();
        }
        let w = window_size.min(data.len());
        let half = w / 2;
        let n = data.len();
        let mut result = Vec::with_capacity(n);

        // Use running sum for efficiency
        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = (i + w - half).min(n);
            let count = end - start;
            let sum: f64 = data[start..end].iter().sum();
            result.push(sum / count as f64);
        }
        result
    }

    /// Apply a spatial median filter for impulse noise removal.
    ///
    /// # Arguments
    ///
    /// * `data` - Input data along the fiber.
    /// * `window_size` - Number of samples in the median window (should be odd).
    ///
    /// # Returns
    ///
    /// Median-filtered data (same length as input).
    pub fn spatial_median(&self, data: &[f64], window_size: usize) -> Vec<f64> {
        if data.is_empty() || window_size == 0 {
            return data.to_vec();
        }
        let w = window_size.min(data.len());
        let half = w / 2;
        let n = data.len();
        let mut result = Vec::with_capacity(n);

        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = (i + w - half).min(n);
            let window_data = &data[start..end];
            result.push(median(window_data));
        }
        result
    }

    /// Compute the spatial derivative (gradient) of the data along the fiber.
    ///
    /// Uses central differences where possible, forward/backward differences
    /// at the edges.
    ///
    /// # Arguments
    ///
    /// * `data` - Input data along the fiber.
    ///
    /// # Returns
    ///
    /// Spatial derivative (same length as input), in units per sample.
    pub fn spatial_gradient(&self, data: &[f64]) -> Vec<f64> {
        let n = data.len();
        if n < 2 {
            return vec![0.0; n];
        }
        let mut grad = Vec::with_capacity(n);
        grad.push(data[1] - data[0]); // forward difference
        for i in 1..n - 1 {
            grad.push((data[i + 1] - data[i - 1]) / 2.0); // central difference
        }
        grad.push(data[n - 1] - data[n - 2]); // backward difference
        grad
    }
}

// ---------------------------------------------------------------------------
// EventDetector
// ---------------------------------------------------------------------------

/// Classification of detected fiber events.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FiberEventType {
    /// Acoustic/vibration event (e.g., footsteps, vehicle, digging).
    Intrusion,
    /// Fluid or gas leak producing acoustic emission.
    Leak,
    /// Static strain event (e.g., ground subsidence, structural load).
    Strain,
    /// Temperature anomaly (hot spot or cold spot).
    Temperature,
    /// Fiber break or severe disturbance.
    FiberBreak,
    /// Unknown or unclassified event.
    Unknown,
}

/// A detected event on the fiber.
#[derive(Debug, Clone)]
pub struct FiberEvent {
    /// Distance along the fiber in meters where the event was detected.
    pub distance_m: f64,
    /// Classification of the event.
    pub event_type: FiberEventType,
    /// Magnitude of the event (units depend on event type).
    pub magnitude: f64,
    /// Duration of the event in seconds (0.0 for instantaneous).
    pub duration_s: f64,
    /// Confidence level (0.0 to 1.0).
    pub confidence: f64,
}

/// Detects and classifies events along the fiber from processed DAS/DTS data.
///
/// Uses threshold-based detection with configurable sensitivity, temporal
/// persistence checking, and simple heuristic classification based on
/// signal characteristics (frequency content, duration, amplitude).
#[derive(Debug, Clone)]
pub struct EventDetector {
    config: FiberConfig,
    /// Threshold for event detection in standard deviations above noise.
    threshold_sigma: f64,
    /// Minimum event duration in samples to avoid false alarms.
    min_duration_samples: usize,
}

impl EventDetector {
    /// Create a new event detector.
    ///
    /// # Arguments
    ///
    /// * `config` - Fiber configuration.
    /// * `threshold_sigma` - Detection threshold in standard deviations.
    /// * `min_duration_samples` - Minimum consecutive samples above threshold.
    pub fn new(config: FiberConfig, threshold_sigma: f64, min_duration_samples: usize) -> Self {
        Self {
            config,
            threshold_sigma,
            min_duration_samples,
        }
    }

    /// Detect events in spatial data from a single time snapshot.
    ///
    /// Scans the data for regions where the absolute value exceeds
    /// `threshold_sigma` standard deviations of the data.
    ///
    /// # Arguments
    ///
    /// * `data` - Processed data along the fiber (e.g., strain, phase).
    ///
    /// # Returns
    ///
    /// List of detected events.
    pub fn detect(&self, data: &[f64]) -> Vec<FiberEvent> {
        if data.is_empty() {
            return Vec::new();
        }

        let mean = data.iter().sum::<f64>() / data.len() as f64;
        let variance =
            data.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / data.len() as f64;
        let sigma = variance.sqrt();

        if sigma < 1e-30 {
            return Vec::new();
        }

        let threshold = self.threshold_sigma * sigma;
        let mut events = Vec::new();
        let mut in_event = false;
        let mut event_start = 0usize;
        let mut event_peak = 0.0f64;
        let mut event_peak_idx = 0usize;

        for i in 0..data.len() {
            let abs_val = (data[i] - mean).abs();
            if abs_val > threshold {
                if !in_event {
                    in_event = true;
                    event_start = i;
                    event_peak = abs_val;
                    event_peak_idx = i;
                } else if abs_val > event_peak {
                    event_peak = abs_val;
                    event_peak_idx = i;
                }
            } else if in_event {
                let duration = i - event_start;
                if duration >= self.min_duration_samples {
                    let distance = self.config.sample_to_distance(event_peak_idx);
                    let event_type = Self::classify_event(event_peak / sigma, duration);
                    let confidence = (event_peak / sigma / 10.0).min(1.0);
                    events.push(FiberEvent {
                        distance_m: distance,
                        event_type,
                        magnitude: event_peak,
                        duration_s: duration as f64 / self.config.sample_rate_hz,
                        confidence,
                    });
                }
                in_event = false;
            }
        }

        // Handle event at end of data
        if in_event {
            let duration = data.len() - event_start;
            if duration >= self.min_duration_samples {
                let distance = self.config.sample_to_distance(event_peak_idx);
                let event_type = Self::classify_event(event_peak / sigma, duration);
                let confidence = (event_peak / sigma / 10.0).min(1.0);
                events.push(FiberEvent {
                    distance_m: distance,
                    event_type,
                    magnitude: event_peak,
                    duration_s: duration as f64 / self.config.sample_rate_hz,
                    confidence,
                });
            }
        }

        events
    }

    /// Classify an event based on its characteristics.
    ///
    /// Simple heuristic:
    /// - Very high magnitude + short duration -> FiberBreak
    /// - High magnitude + moderate duration -> Intrusion
    /// - Moderate magnitude + long duration -> Strain
    /// - Low magnitude + very long duration -> Leak
    fn classify_event(snr: f64, duration_samples: usize) -> FiberEventType {
        if snr > 20.0 && duration_samples < 10 {
            FiberEventType::FiberBreak
        } else if snr > 8.0 && duration_samples < 100 {
            FiberEventType::Intrusion
        } else if duration_samples >= 100 && snr > 5.0 {
            FiberEventType::Strain
        } else if duration_samples >= 50 && snr <= 5.0 {
            FiberEventType::Leak
        } else {
            FiberEventType::Unknown
        }
    }

    /// Detect events across multiple time frames, requiring temporal persistence.
    ///
    /// An event must be present in at least `min_persistence` consecutive frames
    /// at approximately the same location to be reported.
    ///
    /// # Arguments
    ///
    /// * `frames` - Multiple time frames of spatial data.
    /// * `min_persistence` - Minimum number of frames an event must persist.
    ///
    /// # Returns
    ///
    /// List of persistent events.
    pub fn detect_persistent(
        &self,
        frames: &[Vec<f64>],
        min_persistence: usize,
    ) -> Vec<FiberEvent> {
        if frames.is_empty() {
            return Vec::new();
        }

        // Detect events in each frame
        let frame_events: Vec<Vec<FiberEvent>> =
            frames.iter().map(|f| self.detect(f)).collect();

        // Find persistent events across frames using spatial proximity
        let spatial_tolerance = self.config.spatial_resolution_m * 2.0;
        let mut persistent = Vec::new();

        if let Some(first_events) = frame_events.first() {
            for candidate in first_events {
                let mut count = 1usize;
                let mut total_mag = candidate.magnitude;

                for frame_ev in frame_events.iter().skip(1) {
                    let found = frame_ev.iter().any(|e| {
                        (e.distance_m - candidate.distance_m).abs() < spatial_tolerance
                    });
                    if found {
                        count += 1;
                        if let Some(matched) = frame_ev.iter().find(|e| {
                            (e.distance_m - candidate.distance_m).abs() < spatial_tolerance
                        }) {
                            total_mag += matched.magnitude;
                        }
                    }
                }

                if count >= min_persistence {
                    persistent.push(FiberEvent {
                        distance_m: candidate.distance_m,
                        event_type: candidate.event_type,
                        magnitude: total_mag / count as f64,
                        duration_s: count as f64 / self.config.sample_rate_hz,
                        confidence: (count as f64 / frames.len() as f64).min(1.0),
                    });
                }
            }
        }

        persistent
    }
}

// ---------------------------------------------------------------------------
// TemperatureProfiler
// ---------------------------------------------------------------------------

/// Distributed Temperature Sensing (DTS) via Raman backscatter analysis.
///
/// In Raman-based DTS, the ratio of anti-Stokes to Stokes backscatter
/// intensities is related to local temperature by:
///
/// ```text
/// R(T) = I_as / I_s = (ν_as / ν_s)^4 · exp(-h·Δν / (k·T))
/// ```
///
/// where `Δν` is the Raman frequency shift (~13.2 THz for silica), `h` is
/// Planck's constant, `k` is Boltzmann's constant, and `T` is absolute
/// temperature.
///
/// The temperature is extracted by comparing the measured ratio to a reference
/// calibration.
#[derive(Debug, Clone)]
pub struct TemperatureProfiler {
    config: FiberConfig,
    /// Reference temperature in Kelvin used for calibration.
    reference_temp_k: f64,
    /// Raman frequency shift in Hz (typically ~13.2 THz for silica).
    raman_shift_hz: f64,
}

impl TemperatureProfiler {
    /// Create a new temperature profiler.
    ///
    /// # Arguments
    ///
    /// * `config` - Fiber configuration.
    /// * `reference_temp_k` - Reference (calibration) temperature in Kelvin.
    pub fn new(config: FiberConfig, reference_temp_k: f64) -> Self {
        Self {
            config,
            reference_temp_k,
            // Standard Raman shift for silica fiber
            raman_shift_hz: 13.2e12,
        }
    }

    /// Extract temperature profile from anti-Stokes and Stokes traces.
    ///
    /// # Arguments
    ///
    /// * `anti_stokes` - Anti-Stokes backscatter intensity (linear).
    /// * `stokes` - Stokes backscatter intensity (linear).
    ///
    /// # Returns
    ///
    /// Temperature along the fiber in Kelvin.
    ///
    /// # Panics
    ///
    /// Panics if the two traces have different lengths.
    pub fn extract_temperature(&self, anti_stokes: &[f64], stokes: &[f64]) -> Vec<f64> {
        assert_eq!(
            anti_stokes.len(),
            stokes.len(),
            "Anti-Stokes and Stokes traces must have same length"
        );

        let gamma = PLANCK_H * self.raman_shift_hz / BOLTZMANN_K;

        // Reference ratio at calibration temperature
        let r_ref = (-gamma / self.reference_temp_k).exp();

        anti_stokes
            .iter()
            .zip(stokes.iter())
            .map(|(&a_s, &s)| {
                if s.abs() < 1e-30 || a_s.abs() < 1e-30 {
                    return self.reference_temp_k;
                }
                let r_meas = a_s / s;
                // T = gamma / ln(R_ref / R_meas) + 1/T_ref
                // More precisely: 1/T = 1/T_ref - (1/gamma) * ln(R_meas / R_ref)
                let inv_t = 1.0 / self.reference_temp_k - (1.0 / gamma) * (r_meas / r_ref).ln();
                if inv_t > 1e-10 {
                    1.0 / inv_t
                } else {
                    // Temperature diverges - clamp to a large value
                    1e4
                }
            })
            .collect()
    }

    /// Convert temperature in Kelvin to Celsius.
    pub fn kelvin_to_celsius(temps_k: &[f64]) -> Vec<f64> {
        temps_k.iter().map(|&t| t - 273.15).collect()
    }

    /// Compute the expected Raman ratio at a given temperature.
    ///
    /// # Arguments
    ///
    /// * `temp_k` - Temperature in Kelvin.
    ///
    /// # Returns
    ///
    /// Anti-Stokes to Stokes ratio.
    pub fn raman_ratio(&self, temp_k: f64) -> f64 {
        let gamma = PLANCK_H * self.raman_shift_hz / BOLTZMANN_K;
        (-gamma / temp_k).exp()
    }

    /// Apply attenuation correction to Raman traces.
    ///
    /// Compensates for the differential attenuation between anti-Stokes and
    /// Stokes wavelengths.
    ///
    /// # Arguments
    ///
    /// * `trace` - Raw Raman trace (linear intensity).
    /// * `extra_atten_db_per_km` - Additional attenuation of this Raman band
    ///   relative to the pump (dB/km). Anti-Stokes is typically ~0.03 dB/km
    ///   more than Stokes at 1550 nm.
    ///
    /// # Returns
    ///
    /// Attenuation-corrected trace.
    pub fn correct_attenuation(&self, trace: &[f64], extra_atten_db_per_km: f64) -> Vec<f64> {
        trace
            .iter()
            .enumerate()
            .map(|(i, &val)| {
                let distance = self.config.sample_to_distance(i);
                let atten_db = extra_atten_db_per_km * distance / 1000.0 * 2.0;
                let correction = 10.0_f64.powf(atten_db / 10.0);
                val * correction
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the median of a slice of f64 values.
fn median(data: &[f64]) -> f64 {
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

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper to create a default test fiber config.
    fn test_config() -> FiberConfig {
        FiberConfig {
            fiber_length_m: 10_000.0,
            spatial_resolution_m: 1.0,
            pulse_width_s: 10e-9,
            refractive_index: 1.4681,
            attenuation_db_per_km: 0.2,
            rayleigh_scatter_coeff: 1e-4,
            sample_rate_hz: 100e6,
        }
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_group_velocity() {
        let v = group_velocity(1.4681);
        // c / 1.4681 ≈ 2.041e8 m/s
        assert!((v - 2.041e8).abs() < 1e6);
    }

    #[test]
    fn test_spatial_resolution() {
        // 10 ns pulse, n=1.4681: Δz = 3e8 * 10e-9 / (2 * 1.4681) ≈ 1.021 m
        let sr = spatial_resolution(10e-9, 1.4681);
        assert!((sr - 1.021).abs() < 0.01);
    }

    #[test]
    fn test_rayleigh_coefficient() {
        let coeff_1550 = rayleigh_coefficient(1550e-9);
        let coeff_1310 = rayleigh_coefficient(1310e-9);
        // Shorter wavelength should have higher scattering (λ^-4 dependence)
        assert!(coeff_1310 > coeff_1550);
        // Both should be positive
        assert!(coeff_1550 > 0.0);
        assert!(coeff_1310 > 0.0);
    }

    #[test]
    fn test_rayleigh_lambda4_dependence() {
        let c1 = rayleigh_coefficient(1.0e-6);
        let c2 = rayleigh_coefficient(2.0e-6);
        // Ratio should be (2/1)^4 = 16
        let ratio = c1 / c2;
        assert!((ratio - 16.0).abs() < 0.01);
    }

    #[test]
    fn test_phase_unwrap_empty() {
        let result = phase_unwrap(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_phase_unwrap_single() {
        let result = phase_unwrap(&[1.5]);
        assert_eq!(result.len(), 1);
        assert!((result[0] - 1.5).abs() < 1e-12);
    }

    #[test]
    fn test_phase_unwrap_no_wrapping() {
        let phases: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let unwrapped = phase_unwrap(&phases);
        for (a, b) in phases.iter().zip(unwrapped.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_phase_unwrap_with_wrapping() {
        // Create a linearly increasing phase that wraps at ±π
        let n = 200;
        let rate = 0.1; // rad/sample
        let expected: Vec<f64> = (0..n).map(|i| i as f64 * rate).collect();
        let wrapped: Vec<f64> = expected
            .iter()
            .map(|&p| {
                let mut w = p % (2.0 * PI);
                if w > PI {
                    w -= 2.0 * PI;
                }
                w
            })
            .collect();
        let unwrapped = phase_unwrap(&wrapped);
        // Should recover the linear trend (up to a constant offset)
        for i in 1..n {
            let diff_expected = expected[i] - expected[i - 1];
            let diff_actual = unwrapped[i] - unwrapped[i - 1];
            assert!(
                (diff_expected - diff_actual).abs() < 1e-10,
                "Phase unwrap failed at sample {}: expected diff {}, got {}",
                i,
                diff_expected,
                diff_actual,
            );
        }
    }

    // -----------------------------------------------------------------------
    // FiberConfig tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fiber_config_sample_to_distance() {
        let config = test_config();
        let d0 = config.sample_to_distance(0);
        assert!((d0 - 0.0).abs() < 1e-6);

        let d100 = config.sample_to_distance(100);
        // t = 100 / 100e6 = 1e-6 s, d = c * 1e-6 / (2 * 1.4681) ≈ 102.1 m
        assert!((d100 - 102.1).abs() < 1.0);
    }

    #[test]
    fn test_fiber_config_total_samples() {
        let config = test_config();
        let n = config.total_samples();
        // round_trip = 2 * 10000 * 1.4681 / 3e8 ≈ 9.787e-5 s
        // samples = 9.787e-5 * 100e6 ≈ 9787
        assert!(n > 9000 && n < 11000, "total_samples = {} not in expected range", n);
    }

    #[test]
    fn test_fiber_config_samples_per_resolution() {
        let config = test_config();
        let spr = config.samples_per_resolution();
        // cell_time = 2 * 1.0 * 1.4681 / 3e8 ≈ 9.787e-9 s
        // samples = 9.787e-9 * 100e6 ≈ 1
        assert!(spr >= 1 && spr <= 2, "samples_per_resolution = {}", spr);
    }

    // -----------------------------------------------------------------------
    // RayleighBackscatterProcessor tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rayleigh_process_trace_length() {
        let config = test_config();
        let proc = RayleighBackscatterProcessor::new(config);
        let raw = vec![1.0; 500];
        let result = proc.process_trace(&raw);
        assert_eq!(result.len(), 500);
    }

    #[test]
    fn test_rayleigh_process_trace_attenuation_compensation() {
        let config = test_config();
        let proc = RayleighBackscatterProcessor::new(config.clone());
        // Create trace with exponential decay matching fiber attenuation
        let atten_neper_per_m = 0.2 / (1000.0 * 10.0 * 10.0_f64.log10());
        let raw: Vec<f64> = (0..500)
            .map(|i| {
                let d = config.sample_to_distance(i);
                (-2.0 * atten_neper_per_m * d).exp()
            })
            .collect();
        let result = proc.process_trace(&raw);
        // After compensation, the trace should be approximately flat
        // Check that the variation across the trace is small
        let max_val = result.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_val = result.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(
            (max_val - min_val) < 1.0,
            "Compensated trace should be flat but range = {} dB",
            max_val - min_val
        );
    }

    #[test]
    fn test_rayleigh_average_traces() {
        let config = test_config();
        let proc = RayleighBackscatterProcessor::new(config);
        let t1 = vec![1.0, 2.0, 3.0];
        let t2 = vec![3.0, 4.0, 5.0];
        let avg = proc.average_traces(&[t1, t2]);
        assert_eq!(avg.len(), 3);
        assert!((avg[0] - 2.0).abs() < 1e-10);
        assert!((avg[1] - 3.0).abs() < 1e-10);
        assert!((avg[2] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_rayleigh_average_traces_empty() {
        let config = test_config();
        let proc = RayleighBackscatterProcessor::new(config);
        let avg = proc.average_traces(&[]);
        assert!(avg.is_empty());
    }

    #[test]
    fn test_rayleigh_noise_floor() {
        let config = test_config();
        let proc = RayleighBackscatterProcessor::new(config);
        // Trace that is high then drops to noise
        let mut trace = vec![0.0; 100];
        for (i, v) in trace.iter_mut().enumerate() {
            *v = if i < 90 { -10.0 } else { -50.0 };
        }
        let nf = proc.estimate_noise_floor(&trace);
        assert!((nf - (-50.0)).abs() < 1e-10, "Noise floor = {} expected -50", nf);
    }

    #[test]
    fn test_rayleigh_expected_backscatter() {
        let config = test_config();
        let proc = RayleighBackscatterProcessor::new(config);
        let bs0 = proc.expected_backscatter_db(0.0);
        let bs5k = proc.expected_backscatter_db(5000.0);
        // Should decrease with distance
        assert!(bs0 > bs5k, "Backscatter should decrease with distance");
    }

    // -----------------------------------------------------------------------
    // PhaseDemodulator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_phase_demod_constant_phase() {
        let config = test_config();
        let demod = PhaseDemodulator::new(config);
        let n = 100;
        let phase_val: f64 = 1.23;
        let i_data: Vec<f64> = (0..n).map(|_| phase_val.cos()).collect();
        let q_data: Vec<f64> = (0..n).map(|_| phase_val.sin()).collect();
        let phase = demod.demodulate(&i_data, &q_data);
        assert_eq!(phase.len(), n);
        for &p in &phase {
            assert!((p - phase_val).abs() < 1e-10, "Phase should be constant");
        }
    }

    #[test]
    fn test_phase_demod_linear_phase() {
        let config = test_config();
        let demod = PhaseDemodulator::new(config);
        let n = 200;
        let rate = 0.05;
        let i_data: Vec<f64> = (0..n).map(|k| (k as f64 * rate).cos()).collect();
        let q_data: Vec<f64> = (0..n).map(|k| (k as f64 * rate).sin()).collect();
        let phase = demod.demodulate(&i_data, &q_data);
        // Phase should increase linearly
        for i in 1..n {
            let diff = phase[i] - phase[i - 1];
            assert!(
                (diff - rate).abs() < 1e-6,
                "Phase rate at {} = {} expected {}",
                i,
                diff,
                rate
            );
        }
    }

    #[test]
    fn test_phase_demod_differential() {
        let config = test_config();
        let demod = PhaseDemodulator::new(config);
        let phase = vec![0.0, 0.1, 0.3, 0.6, 1.0];
        let diff = demod.differential_phase(&phase);
        assert_eq!(diff.len(), 4);
        assert!((diff[0] - 0.1).abs() < 1e-10);
        assert!((diff[1] - 0.2).abs() < 1e-10);
        assert!((diff[2] - 0.3).abs() < 1e-10);
        assert!((diff[3] - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_phase_to_strain() {
        let config = test_config();
        let demod = PhaseDemodulator::new(config);
        let diff_phase = vec![0.0, 0.001, -0.001];
        let strain = demod.phase_to_strain(&diff_phase, 1550e-9, 10.0);
        assert_eq!(strain.len(), 3);
        // First sample zero phase -> zero strain
        assert!(strain[0].abs() < 1e-10);
        // Positive and negative phase should give opposite strain
        assert!((strain[1] + strain[2]).abs() < 1e-10);
    }

    #[test]
    fn test_amplitude() {
        let config = test_config();
        let demod = PhaseDemodulator::new(config);
        let i_data = vec![3.0, 0.0, -4.0];
        let q_data = vec![4.0, 5.0, 3.0];
        let amp = demod.amplitude(&i_data, &q_data);
        assert!((amp[0] - 5.0).abs() < 1e-10);
        assert!((amp[1] - 5.0).abs() < 1e-10);
        assert!((amp[2] - 5.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // SpatialFilter tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gauge_length() {
        let config = test_config();
        let sf = SpatialFilter::new(config);
        // Simple ramp data
        let data: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let gauge_m = 10.0; // 10 m gauge length
        let result = sf.gauge_length(&data, gauge_m);
        // Result should be shorter than input
        assert!(result.len() < data.len());
        // For a linear ramp, gauge-length differencing gives a constant
        if result.len() > 2 {
            let first = result[0];
            for &v in &result[1..] {
                assert!(
                    (v - first).abs() < 1e-6,
                    "Gauge-length of ramp should be constant, got {} vs {}",
                    v,
                    first
                );
            }
        }
    }

    #[test]
    fn test_spatial_average_identity() {
        let config = test_config();
        let sf = SpatialFilter::new(config);
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let avg = sf.spatial_average(&data, 1);
        assert_eq!(avg.len(), 5);
        for (a, b) in data.iter().zip(avg.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_spatial_average_smoothing() {
        let config = test_config();
        let sf = SpatialFilter::new(config);
        let data = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let avg = sf.spatial_average(&data, 3);
        assert_eq!(avg.len(), 5);
        // The peak should be reduced by averaging
        assert!(avg[2] < 10.0);
        // Center sample: average of [0, 10, 0] = 3.33
        assert!((avg[2] - 10.0 / 3.0).abs() < 0.1);
    }

    #[test]
    fn test_spatial_median() {
        let config = test_config();
        let sf = SpatialFilter::new(config);
        // Data with an outlier
        let data = vec![1.0, 1.0, 100.0, 1.0, 1.0];
        let filtered = sf.spatial_median(&data, 3);
        assert_eq!(filtered.len(), 5);
        // Median filter should remove the outlier
        assert!(filtered[2] < 50.0, "Median should suppress outlier, got {}", filtered[2]);
    }

    #[test]
    fn test_spatial_gradient() {
        let config = test_config();
        let sf = SpatialFilter::new(config);
        // Linear ramp: gradient should be constant
        let data: Vec<f64> = (0..10).map(|i| 2.0 * i as f64).collect();
        let grad = sf.spatial_gradient(&data);
        assert_eq!(grad.len(), 10);
        // Interior points should have gradient = 2.0
        for &g in &grad[1..9] {
            assert!((g - 2.0).abs() < 1e-10);
        }
    }

    // -----------------------------------------------------------------------
    // EventDetector tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_event_detector_no_events() {
        let config = test_config();
        let det = EventDetector::new(config, 5.0, 2);
        // Flat data -> no events
        let data = vec![1.0; 200];
        let events = det.detect(&data);
        assert!(events.is_empty(), "Flat data should produce no events");
    }

    #[test]
    fn test_event_detector_single_event() {
        let config = test_config();
        let det = EventDetector::new(config, 3.0, 2);
        // Background noise with a clear event
        let mut data = vec![0.0; 200];
        // Insert a large event at samples 100-105
        for i in 100..106 {
            data[i] = 50.0;
        }
        let events = det.detect(&data);
        assert!(!events.is_empty(), "Should detect the event");
        // Check that the event is near the right location
        let ev = &events[0];
        assert!(ev.magnitude > 10.0, "Event magnitude should be significant");
    }

    #[test]
    fn test_event_classification_break() {
        // Very high SNR, very short duration -> FiberBreak
        let t = EventDetector::classify_event(25.0, 5);
        assert_eq!(t, FiberEventType::FiberBreak);
    }

    #[test]
    fn test_event_classification_intrusion() {
        let t = EventDetector::classify_event(10.0, 50);
        assert_eq!(t, FiberEventType::Intrusion);
    }

    #[test]
    fn test_event_classification_strain() {
        let t = EventDetector::classify_event(6.0, 150);
        assert_eq!(t, FiberEventType::Strain);
    }

    #[test]
    fn test_event_classification_leak() {
        let t = EventDetector::classify_event(4.0, 80);
        assert_eq!(t, FiberEventType::Leak);
    }

    // -----------------------------------------------------------------------
    // TemperatureProfiler tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_temperature_profiler_reference() {
        let config = test_config();
        let tp = TemperatureProfiler::new(config, T_REF);
        // At reference temperature, ratio should give back T_REF
        let r = tp.raman_ratio(T_REF);
        let as_trace = vec![r; 10];
        let s_trace = vec![1.0; 10];
        let temps = tp.extract_temperature(&as_trace, &s_trace);
        for &t in &temps {
            assert!(
                (t - T_REF).abs() < 0.1,
                "Temperature at reference ratio should be T_REF={}, got {}",
                T_REF,
                t
            );
        }
    }

    #[test]
    fn test_temperature_profiler_hot_spot() {
        let config = test_config();
        let tp = TemperatureProfiler::new(config, T_REF);
        let r_ref = tp.raman_ratio(T_REF);
        let r_hot = tp.raman_ratio(350.0);
        // Create a trace with a hot spot
        let mut as_trace = vec![r_ref; 20];
        let s_trace = vec![1.0; 20];
        as_trace[10] = r_hot;
        let temps = tp.extract_temperature(&as_trace, &s_trace);
        // Hot spot should be hotter than surroundings
        assert!(
            temps[10] > temps[0] + 10.0,
            "Hot spot temp {} should be much higher than background {}",
            temps[10],
            temps[0]
        );
    }

    #[test]
    fn test_raman_ratio_increases_with_temperature() {
        let config = test_config();
        let tp = TemperatureProfiler::new(config, T_REF);
        let r_cold = tp.raman_ratio(250.0);
        let r_hot = tp.raman_ratio(400.0);
        assert!(
            r_hot > r_cold,
            "Raman ratio should increase with temperature"
        );
    }

    #[test]
    fn test_kelvin_to_celsius() {
        let temps_k = vec![273.15, 373.15, 300.0];
        let temps_c = TemperatureProfiler::kelvin_to_celsius(&temps_k);
        assert!((temps_c[0] - 0.0).abs() < 0.01);
        assert!((temps_c[1] - 100.0).abs() < 0.01);
        assert!((temps_c[2] - 26.85).abs() < 0.01);
    }

    #[test]
    fn test_attenuation_correction() {
        let config = test_config();
        let tp = TemperatureProfiler::new(config, T_REF);
        let trace = vec![1.0; 100];
        let corrected = tp.correct_attenuation(&trace, 0.03);
        // Correction should increase with distance
        assert!(corrected[99] > corrected[0]);
        // First sample (distance=0) should be unchanged
        assert!((corrected[0] - 1.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Integration / edge case tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_median_helper() {
        assert!((median(&[5.0, 1.0, 3.0]) - 3.0).abs() < 1e-10);
        assert!((median(&[1.0, 2.0, 3.0, 4.0]) - 2.5).abs() < 1e-10);
        assert!((median(&[42.0]) - 42.0).abs() < 1e-10);
        assert!((median(&[]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_demod_amplitude_invariance() {
        // Phase demodulation should be independent of amplitude
        let config = test_config();
        let demod = PhaseDemodulator::new(config);
        let n = 50;
        let rate = 0.1;
        let i_large: Vec<f64> = (0..n).map(|k| 10.0 * (k as f64 * rate).cos()).collect();
        let q_large: Vec<f64> = (0..n).map(|k| 10.0 * (k as f64 * rate).sin()).collect();
        let i_small: Vec<f64> = (0..n).map(|k| 0.01 * (k as f64 * rate).cos()).collect();
        let q_small: Vec<f64> = (0..n).map(|k| 0.01 * (k as f64 * rate).sin()).collect();
        let phase_large = demod.demodulate(&i_large, &q_large);
        let phase_small = demod.demodulate(&i_small, &q_small);
        for i in 0..n {
            assert!(
                (phase_large[i] - phase_small[i]).abs() < 1e-10,
                "Phase should be amplitude-invariant"
            );
        }
    }
}
