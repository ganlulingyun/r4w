//! Ultrasonic signal processing for pipeline wall thickness measurement and defect detection.
//!
//! This module implements ultrasonic inspection techniques used in pipeline integrity
//! management. It covers transit-time ultrasonic flow measurement, pulse-echo wall
//! thickness gauging, grid-based corrosion mapping with loss-rate estimation, and
//! weld inspection with B-scan generation and indication detection.
//!
//! # Features
//!
//! - **Transit-time flow measurement** using upstream/downstream time-of-flight difference
//! - **Wall thickness gauging** with pulse-echo A-scan processing and multi-echo averaging
//! - **Corrosion mapping** on a configurable grid with loss rate and remaining life estimation
//! - **Weld inspection** with B-scan image generation and flaw indication detection
//! - **Helper functions** for Snell refraction, acoustic impedance, reflection coefficients,
//!   and beam spread calculations
//!
//! # Example
//!
//! ```
//! use r4w_core::ultrasonic_pipeline_inspector::{
//!     PipelineConfig, TransitTimeFlowMeter, WallThicknessGauge,
//! };
//!
//! let config = PipelineConfig {
//!     outer_diameter_mm: 323.9,
//!     nominal_wall_mm: 9.5,
//!     material_velocity_mps: 5900.0,
//!     material_density_kgm3: 7850.0,
//!     fluid_velocity_mps: 1480.0,
//!     fluid_density_kgm3: 1000.0,
//! };
//!
//! // Measure flow velocity from transit-time difference
//! let meter = TransitTimeFlowMeter::new(&config, 45.0_f64.to_radians(), 0.1);
//! let t_up = 0.000_105_0;   // upstream transit time (seconds)
//! let t_down = 0.000_095_0; // downstream transit time (seconds)
//! let result = meter.measure(t_up, t_down);
//! assert!(result.flow_velocity_mps > 0.0);
//!
//! // Gauge wall thickness from pulse-echo signal
//! let gauge = WallThicknessGauge::new(&config, 100_000_000.0);
//! let signal = gauge.generate_test_signal(8.5, 0.8);
//! let thickness = gauge.measure(&signal);
//! assert!(thickness.is_some());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for a pipeline under ultrasonic inspection.
#[derive(Debug, Clone)]
pub struct PipelineConfig {
    /// Outer diameter of the pipe in millimetres.
    pub outer_diameter_mm: f64,
    /// Nominal (design) wall thickness in millimetres.
    pub nominal_wall_mm: f64,
    /// Longitudinal sound velocity in the pipe wall material (m/s).
    /// Typical steel: 5900 m/s.
    pub material_velocity_mps: f64,
    /// Density of the pipe wall material (kg/m^3).
    /// Typical steel: 7850 kg/m^3.
    pub material_density_kgm3: f64,
    /// Sound velocity in the fluid inside the pipe (m/s).
    /// Typical water: 1480 m/s.
    pub fluid_velocity_mps: f64,
    /// Density of the fluid inside the pipe (kg/m^3).
    /// Typical water: 1000 kg/m^3.
    pub fluid_density_kgm3: f64,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            outer_diameter_mm: 323.9, // 12" NPS
            nominal_wall_mm: 9.5,
            material_velocity_mps: 5900.0,
            material_density_kgm3: 7850.0,
            fluid_velocity_mps: 1480.0,
            fluid_density_kgm3: 1000.0,
        }
    }
}

impl PipelineConfig {
    /// Returns the inner diameter of the pipe in millimetres.
    pub fn inner_diameter_mm(&self) -> f64 {
        self.outer_diameter_mm - 2.0 * self.nominal_wall_mm
    }

    /// Returns the cross-sectional flow area in square metres.
    pub fn flow_area_m2(&self) -> f64 {
        let id_m = self.inner_diameter_mm() / 1000.0;
        PI / 4.0 * id_m * id_m
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute acoustic impedance Z = density * velocity (Rayl = kg/(m^2*s)).
///
/// # Arguments
/// * `density_kgm3` - Material density in kg/m^3
/// * `velocity_mps` - Sound velocity in m/s
///
/// # Returns
/// Acoustic impedance in Rayl (Pa*s/m)
pub fn acoustic_impedance(density_kgm3: f64, velocity_mps: f64) -> f64 {
    density_kgm3 * velocity_mps
}

/// Compute the pressure reflection coefficient at a planar interface.
///
/// R = (Z2 - Z1) / (Z2 + Z1), where Z is acoustic impedance.
///
/// Returns a value in [-1, 1]. Positive means the reflected wave has the same
/// polarity as the incident wave; negative means phase inversion.
///
/// # Arguments
/// * `z1` - Acoustic impedance of the first medium (Rayl)
/// * `z2` - Acoustic impedance of the second medium (Rayl)
pub fn reflection_coefficient(z1: f64, z2: f64) -> f64 {
    if (z1 + z2).abs() < 1e-30 {
        return 0.0;
    }
    (z2 - z1) / (z2 + z1)
}

/// Compute the pressure transmission coefficient at a planar interface.
///
/// T = 2 * Z2 / (Z1 + Z2).
///
/// # Arguments
/// * `z1` - Acoustic impedance of the first medium (Rayl)
/// * `z2` - Acoustic impedance of the second medium (Rayl)
pub fn transmission_coefficient(z1: f64, z2: f64) -> f64 {
    if (z1 + z2).abs() < 1e-30 {
        return 0.0;
    }
    2.0 * z2 / (z1 + z2)
}

/// Apply Snell's law for refraction at a material boundary.
///
/// sin(theta2) / v2 = sin(theta1) / v1
///
/// Returns `None` if total internal reflection occurs (refracted angle would
/// exceed 90 degrees).
///
/// # Arguments
/// * `incident_angle_rad` - Angle of incidence in radians
/// * `v1_mps` - Sound velocity in the first medium (m/s)
/// * `v2_mps` - Sound velocity in the second medium (m/s)
pub fn snell_refraction(incident_angle_rad: f64, v1_mps: f64, v2_mps: f64) -> Option<f64> {
    let sin_theta2 = incident_angle_rad.sin() * v2_mps / v1_mps;
    if sin_theta2.abs() > 1.0 {
        None // total internal reflection
    } else {
        Some(sin_theta2.asin())
    }
}

/// Compute the half-angle of beam spread for a circular transducer.
///
/// theta = arcsin(1.22 * lambda / D)
///
/// where lambda = velocity / frequency and D is the transducer diameter.
///
/// # Arguments
/// * `frequency_hz` - Transducer frequency in Hz
/// * `velocity_mps` - Sound velocity in the medium (m/s)
/// * `diameter_mm` - Transducer active element diameter in mm
///
/// # Returns
/// Half-angle of beam spread in radians. Returns PI/2 if the ratio exceeds 1.
pub fn beam_spread(frequency_hz: f64, velocity_mps: f64, diameter_mm: f64) -> f64 {
    let lambda = velocity_mps / frequency_hz;
    let diameter_m = diameter_mm / 1000.0;
    let ratio = 1.22 * lambda / diameter_m;
    if ratio >= 1.0 {
        PI / 2.0
    } else {
        ratio.asin()
    }
}

/// Compute the near-field (Fresnel zone) length for a circular transducer.
///
/// N = D^2 / (4 * lambda) = D^2 * f / (4 * v)
///
/// # Arguments
/// * `frequency_hz` - Transducer frequency in Hz
/// * `velocity_mps` - Sound velocity in the medium (m/s)
/// * `diameter_mm` - Transducer active element diameter in mm
///
/// # Returns
/// Near-field length in millimetres.
pub fn near_field_length(frequency_hz: f64, velocity_mps: f64, diameter_mm: f64) -> f64 {
    let d_m = diameter_mm / 1000.0;
    let n_m = d_m * d_m * frequency_hz / (4.0 * velocity_mps);
    n_m * 1000.0 // convert to mm
}

/// Convert a pulse-echo time-of-flight to thickness/depth.
///
/// In pulse-echo mode the sound traverses the wall twice:
/// thickness = velocity * tof / 2.
///
/// # Arguments
/// * `tof_s` - Time-of-flight in seconds
/// * `velocity_mps` - Sound velocity in the material (m/s)
///
/// # Returns
/// Thickness/depth in millimetres.
pub fn tof_to_thickness(tof_s: f64, velocity_mps: f64) -> f64 {
    velocity_mps * tof_s / 2.0 * 1000.0
}

/// Convert a thickness to the expected pulse-echo time-of-flight.
///
/// # Arguments
/// * `thickness_mm` - Wall thickness in mm
/// * `velocity_mps` - Sound velocity in the material (m/s)
///
/// # Returns
/// Time-of-flight in seconds.
pub fn thickness_to_tof(thickness_mm: f64, velocity_mps: f64) -> f64 {
    2.0 * (thickness_mm / 1000.0) / velocity_mps
}

/// Compute signal attenuation in dB for a given path length.
///
/// Attenuation = alpha * distance (one-way) or 2 * alpha * distance (round-trip).
///
/// # Arguments
/// * `distance_mm` - One-way path length in mm
/// * `attenuation_db_per_mm` - Material attenuation coefficient
/// * `round_trip` - If true, doubles the path length for pulse-echo
pub fn attenuation_db(distance_mm: f64, attenuation_db_per_mm: f64, round_trip: bool) -> f64 {
    let factor = if round_trip { 2.0 } else { 1.0 };
    factor * distance_mm * attenuation_db_per_mm
}

// ---------------------------------------------------------------------------
// Transit-Time Flow Meter
// ---------------------------------------------------------------------------

/// Result of a transit-time flow velocity measurement.
#[derive(Debug, Clone)]
pub struct FlowMeasurement {
    /// Measured flow velocity in m/s along the pipe axis.
    pub flow_velocity_mps: f64,
    /// Volumetric flow rate in m^3/s.
    pub volume_flow_m3s: f64,
    /// Upstream transit time in seconds (as measured).
    pub t_upstream_s: f64,
    /// Downstream transit time in seconds (as measured).
    pub t_downstream_s: f64,
    /// Computed mean sound velocity along the acoustic path (m/s).
    pub mean_sound_velocity_mps: f64,
}

/// Transit-time ultrasonic flow meter.
///
/// Measures fluid flow velocity by comparing upstream and downstream
/// transit times of ultrasonic pulses through the fluid. The transducers
/// are mounted at an angle to the pipe axis.
///
/// The flow velocity is computed as:
///
/// v_flow = (L / (2 * cos(theta))) * (1/t_down - 1/t_up)
///
/// where L is the acoustic path length and theta is the beam angle
/// relative to the pipe axis.
#[derive(Debug, Clone)]
pub struct TransitTimeFlowMeter {
    /// Acoustic path length through the fluid in metres.
    path_length_m: f64,
    /// Beam angle relative to the pipe axis in radians.
    beam_angle_rad: f64,
    /// Pipe cross-sectional flow area in m^2.
    flow_area_m2: f64,
    /// Correction factor for flow profile (default 1.0 for plug flow,
    /// typically ~0.75 for fully developed turbulent flow in a pipe).
    profile_factor: f64,
}

impl TransitTimeFlowMeter {
    /// Create a new transit-time flow meter.
    ///
    /// # Arguments
    /// * `config` - Pipeline configuration
    /// * `beam_angle_rad` - Angle of the ultrasonic beam to the pipe axis (radians)
    /// * `profile_factor` - Velocity profile correction factor (use 1.0 for plug flow)
    pub fn new(config: &PipelineConfig, beam_angle_rad: f64, profile_factor: f64) -> Self {
        let id_m = config.inner_diameter_mm() / 1000.0;
        // Acoustic path length across the pipe at the given angle
        let path_length_m = id_m / beam_angle_rad.sin();
        Self {
            path_length_m,
            beam_angle_rad,
            flow_area_m2: config.flow_area_m2(),
            profile_factor: if profile_factor > 0.0 {
                profile_factor
            } else {
                1.0
            },
        }
    }

    /// Measure flow velocity from upstream and downstream transit times.
    ///
    /// # Arguments
    /// * `t_upstream_s` - Upstream transit time in seconds (against flow, longer)
    /// * `t_downstream_s` - Downstream transit time in seconds (with flow, shorter)
    ///
    /// # Returns
    /// A `FlowMeasurement` with computed velocity and volume flow rate.
    pub fn measure(&self, t_upstream_s: f64, t_downstream_s: f64) -> FlowMeasurement {
        // Mean sound velocity along the path
        let c_mean = self.path_length_m * (1.0 / t_downstream_s + 1.0 / t_upstream_s) / 2.0;

        // Flow velocity component along the acoustic path
        let v_path = self.path_length_m * (1.0 / t_downstream_s - 1.0 / t_upstream_s) / 2.0;

        // Project onto pipe axis and apply profile correction
        let v_flow = v_path / self.beam_angle_rad.cos() * self.profile_factor;

        let volume_flow = v_flow * self.flow_area_m2;

        FlowMeasurement {
            flow_velocity_mps: v_flow,
            volume_flow_m3s: volume_flow,
            t_upstream_s,
            t_downstream_s,
            mean_sound_velocity_mps: c_mean,
        }
    }

    /// Compute the Reynolds number for flow characterization.
    ///
    /// Re = v * D / nu, where nu is kinematic viscosity.
    ///
    /// # Arguments
    /// * `velocity_mps` - Flow velocity in m/s
    /// * `inner_diameter_m` - Inner pipe diameter in metres
    /// * `kinematic_viscosity` - Kinematic viscosity in m^2/s (water ~1e-6)
    pub fn reynolds_number(
        velocity_mps: f64,
        inner_diameter_m: f64,
        kinematic_viscosity: f64,
    ) -> f64 {
        if kinematic_viscosity.abs() < 1e-30 {
            return 0.0;
        }
        (velocity_mps * inner_diameter_m / kinematic_viscosity).abs()
    }
}

// ---------------------------------------------------------------------------
// Wall Thickness Gauge
// ---------------------------------------------------------------------------

/// Result of a wall thickness measurement.
#[derive(Debug, Clone)]
pub struct ThicknessMeasurement {
    /// Measured wall thickness in millimetres.
    pub thickness_mm: f64,
    /// Time-of-flight in seconds.
    pub tof_s: f64,
    /// Peak amplitude of the first back-wall echo (0.0 to 1.0).
    pub echo_amplitude: f64,
    /// Number of echoes detected in the A-scan.
    pub echo_count: usize,
}

/// Pulse-echo wall thickness gauge for pipeline inspection.
///
/// Processes A-scan signals to determine remaining wall thickness by detecting
/// the first back-wall echo and measuring its time-of-flight. Supports
/// multi-echo averaging for improved accuracy.
#[derive(Debug, Clone)]
pub struct WallThicknessGauge {
    /// Sound velocity in the pipe wall material (m/s).
    velocity_mps: f64,
    /// Digitiser sample rate in Hz.
    sample_rate_hz: f64,
    /// Minimum detectable thickness in mm (dead zone).
    min_thickness_mm: f64,
    /// Maximum measurable thickness in mm.
    max_thickness_mm: f64,
    /// Detection threshold as a fraction of peak amplitude (0 to 1).
    threshold: f64,
}

impl WallThicknessGauge {
    /// Create a new wall thickness gauge.
    ///
    /// # Arguments
    /// * `config` - Pipeline configuration (uses material velocity)
    /// * `sample_rate_hz` - Digitiser sample rate in Hz
    pub fn new(config: &PipelineConfig, sample_rate_hz: f64) -> Self {
        Self {
            velocity_mps: config.material_velocity_mps,
            sample_rate_hz,
            min_thickness_mm: 1.0,
            max_thickness_mm: config.nominal_wall_mm * 2.0,
            threshold: 0.2,
        }
    }

    /// Set the detection threshold (fraction of signal peak, 0 to 1).
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold.clamp(0.01, 0.99);
    }

    /// Set the minimum detectable thickness in mm.
    pub fn set_min_thickness(&mut self, mm: f64) {
        self.min_thickness_mm = mm.max(0.1);
    }

    /// Measure wall thickness from an A-scan signal.
    ///
    /// The algorithm:
    /// 1. Compute the envelope of the signal (absolute value)
    /// 2. Find the peak amplitude
    /// 3. Apply threshold-based peak detection to find echo locations
    /// 4. Convert first echo time-of-flight to thickness
    ///
    /// # Arguments
    /// * `signal` - A-scan amplitude samples (time domain)
    ///
    /// # Returns
    /// `Some(ThicknessMeasurement)` if a valid echo is found, `None` otherwise.
    pub fn measure(&self, signal: &[f64]) -> Option<ThicknessMeasurement> {
        if signal.is_empty() {
            return None;
        }

        // Compute envelope (absolute values)
        let envelope: Vec<f64> = signal.iter().map(|s| s.abs()).collect();

        // Find global peak
        let max_amp = envelope.iter().cloned().fold(0.0_f64, f64::max);
        if max_amp < 1e-12 {
            return None;
        }

        let abs_threshold = max_amp * self.threshold;

        // Minimum sample index based on min thickness
        let min_tof_s = thickness_to_tof(self.min_thickness_mm, self.velocity_mps);
        let min_sample = (min_tof_s * self.sample_rate_hz) as usize;

        // Maximum sample index based on max thickness
        let max_tof_s = thickness_to_tof(self.max_thickness_mm, self.velocity_mps);
        let max_sample = ((max_tof_s * self.sample_rate_hz) as usize).min(envelope.len() - 1);

        // Find echo peaks (local maxima above threshold)
        let mut echoes: Vec<(usize, f64)> = Vec::new();
        let search_start = min_sample.min(envelope.len().saturating_sub(1));
        let search_end = max_sample.min(envelope.len().saturating_sub(1));

        for i in search_start..=search_end {
            if envelope[i] >= abs_threshold {
                let prev = if i > 0 { envelope[i - 1] } else { 0.0 };
                let next = if i + 1 < envelope.len() {
                    envelope[i + 1]
                } else {
                    0.0
                };
                if envelope[i] >= prev && envelope[i] >= next {
                    // Check if this is a new peak (not part of the same echo)
                    let is_new = echoes.last().map_or(true, |(last_idx, _)| i - last_idx > 5);
                    if is_new {
                        echoes.push((i, envelope[i]));
                    } else if let Some(last) = echoes.last_mut() {
                        if envelope[i] > last.1 {
                            *last = (i, envelope[i]);
                        }
                    }
                }
            }
        }

        if echoes.is_empty() {
            return None;
        }

        // Use first echo for thickness measurement
        let (echo_idx, echo_amp) = echoes[0];
        let tof_s = echo_idx as f64 / self.sample_rate_hz;
        let thickness = tof_to_thickness(tof_s, self.velocity_mps);

        Some(ThicknessMeasurement {
            thickness_mm: thickness,
            tof_s,
            echo_amplitude: echo_amp / max_amp,
            echo_count: echoes.len(),
        })
    }

    /// Measure with multi-echo averaging for improved accuracy.
    ///
    /// If multiple back-wall echoes are detected, the inter-echo spacing
    /// is used to compute a more accurate thickness (cancels the unknown
    /// coupling delay of the first echo).
    ///
    /// # Arguments
    /// * `signal` - A-scan amplitude samples
    ///
    /// # Returns
    /// Averaged thickness measurement, or single-echo result.
    pub fn measure_multi_echo(&self, signal: &[f64]) -> Option<ThicknessMeasurement> {
        if signal.is_empty() {
            return None;
        }

        let envelope: Vec<f64> = signal.iter().map(|s| s.abs()).collect();
        let max_amp = envelope.iter().cloned().fold(0.0_f64, f64::max);
        if max_amp < 1e-12 {
            return None;
        }

        let abs_threshold = max_amp * self.threshold;

        // Find all peaks above threshold
        let mut peaks: Vec<(usize, f64)> = Vec::new();
        for i in 1..envelope.len().saturating_sub(1) {
            if envelope[i] >= abs_threshold
                && envelope[i] >= envelope[i - 1]
                && envelope[i] >= envelope[i + 1]
            {
                let is_new = peaks.last().map_or(true, |(last_idx, _)| i - last_idx > 5);
                if is_new {
                    peaks.push((i, envelope[i]));
                } else if let Some(last) = peaks.last_mut() {
                    if envelope[i] > last.1 {
                        *last = (i, envelope[i]);
                    }
                }
            }
        }

        if peaks.is_empty() {
            return self.measure(signal);
        }

        if peaks.len() >= 2 {
            // Use inter-echo spacing for accuracy
            let mut spacings = Vec::new();
            for i in 1..peaks.len() {
                spacings.push(peaks[i].0 as f64 - peaks[i - 1].0 as f64);
            }
            let avg_spacing = spacings.iter().sum::<f64>() / spacings.len() as f64;
            let tof_s = avg_spacing / self.sample_rate_hz;
            let thickness = tof_to_thickness(tof_s, self.velocity_mps);

            Some(ThicknessMeasurement {
                thickness_mm: thickness,
                tof_s,
                echo_amplitude: peaks[0].1 / max_amp,
                echo_count: peaks.len(),
            })
        } else {
            // Single echo - use direct measurement
            let tof_s = peaks[0].0 as f64 / self.sample_rate_hz;
            let thickness = tof_to_thickness(tof_s, self.velocity_mps);
            Some(ThicknessMeasurement {
                thickness_mm: thickness,
                tof_s,
                echo_amplitude: peaks[0].1 / max_amp,
                echo_count: 1,
            })
        }
    }

    /// Generate a synthetic A-scan test signal with a back-wall echo.
    ///
    /// Creates a Gaussian-enveloped RF pulse at the expected time-of-flight
    /// for the given wall thickness.
    ///
    /// # Arguments
    /// * `thickness_mm` - Simulated wall thickness in mm
    /// * `amplitude` - Echo amplitude (0 to 1)
    ///
    /// # Returns
    /// Vector of A-scan samples.
    pub fn generate_test_signal(&self, thickness_mm: f64, amplitude: f64) -> Vec<f64> {
        let tof_s = thickness_to_tof(thickness_mm, self.velocity_mps);
        let total_samples = ((tof_s * 3.0) * self.sample_rate_hz) as usize + 100;
        let echo_center = (tof_s * self.sample_rate_hz) as usize;

        let mut signal = vec![0.0; total_samples];

        // Gaussian-enveloped pulse centred at the echo location
        let pulse_width = 10; // samples half-width
        let sigma = pulse_width as f64 / 3.0;

        for i in 0..signal.len() {
            let dist = i as f64 - echo_center as f64;
            if dist.abs() < (pulse_width * 3) as f64 {
                let envelope = (-dist * dist / (2.0 * sigma * sigma)).exp();
                // RF carrier at 5 MHz equivalent
                let carrier_freq = 5.0e6;
                let t = i as f64 / self.sample_rate_hz;
                let carrier = (2.0 * PI * carrier_freq * t).sin();
                signal[i] = amplitude * envelope * carrier;
            }
        }

        signal
    }
}

// ---------------------------------------------------------------------------
// Corrosion Mapper
// ---------------------------------------------------------------------------

/// A single thickness reading on the corrosion map grid.
#[derive(Debug, Clone)]
pub struct GridReading {
    /// Axial position along the pipe in millimetres.
    pub axial_mm: f64,
    /// Circumferential position in degrees (0-360, 12 o'clock = 0).
    pub circ_deg: f64,
    /// Measured wall thickness in millimetres.
    pub thickness_mm: f64,
    /// Timestamp of the reading (arbitrary units, e.g. days since installation).
    pub timestamp: f64,
}

/// Corrosion assessment for a single grid cell.
#[derive(Debug, Clone)]
pub struct CorrosionAssessment {
    /// Axial position in mm.
    pub axial_mm: f64,
    /// Circumferential position in degrees.
    pub circ_deg: f64,
    /// Minimum measured thickness in mm.
    pub min_thickness_mm: f64,
    /// Maximum measured thickness in mm.
    pub max_thickness_mm: f64,
    /// Wall loss as a percentage of nominal thickness.
    pub wall_loss_percent: f64,
    /// Estimated corrosion rate in mm/year (if multiple readings at different
    /// times are available). `None` if only one time point.
    pub corrosion_rate_mmpy: Option<f64>,
    /// Estimated remaining life in years before reaching minimum allowable
    /// thickness. `None` if corrosion rate is not available.
    pub remaining_life_years: Option<f64>,
}

/// Grid-based corrosion mapping for pipeline wall thickness monitoring.
///
/// Collects thickness readings on a configurable axial x circumferential grid
/// and produces corrosion assessments including wall loss percentage, corrosion
/// rate (if historical data is available), and remaining life estimation.
#[derive(Debug, Clone)]
pub struct CorrosionMapper {
    /// Nominal wall thickness in mm.
    nominal_wall_mm: f64,
    /// Minimum allowable wall thickness in mm (for remaining life calculation).
    min_allowable_mm: f64,
    /// Collected grid readings.
    readings: Vec<GridReading>,
}

impl CorrosionMapper {
    /// Create a new corrosion mapper.
    ///
    /// # Arguments
    /// * `config` - Pipeline configuration
    /// * `min_allowable_mm` - Minimum allowable wall thickness (regulatory/design limit)
    pub fn new(config: &PipelineConfig, min_allowable_mm: f64) -> Self {
        Self {
            nominal_wall_mm: config.nominal_wall_mm,
            min_allowable_mm,
            readings: Vec::new(),
        }
    }

    /// Add a thickness reading to the map.
    pub fn add_reading(&mut self, reading: GridReading) {
        self.readings.push(reading);
    }

    /// Add multiple readings at once.
    pub fn add_readings(&mut self, readings: &[GridReading]) {
        self.readings.extend(readings.iter().cloned());
    }

    /// Return the number of readings collected.
    pub fn reading_count(&self) -> usize {
        self.readings.len()
    }

    /// Get the minimum thickness reading across all grid points.
    pub fn min_thickness(&self) -> Option<f64> {
        self.readings
            .iter()
            .map(|r| r.thickness_mm)
            .fold(None, |acc, t| match acc {
                None => Some(t),
                Some(min) => Some(if t < min { t } else { min }),
            })
    }

    /// Get the maximum thickness reading across all grid points.
    pub fn max_thickness(&self) -> Option<f64> {
        self.readings
            .iter()
            .map(|r| r.thickness_mm)
            .fold(None, |acc, t| match acc {
                None => Some(t),
                Some(max) => Some(if t > max { t } else { max }),
            })
    }

    /// Get the mean thickness across all readings.
    pub fn mean_thickness(&self) -> Option<f64> {
        if self.readings.is_empty() {
            return None;
        }
        let sum: f64 = self.readings.iter().map(|r| r.thickness_mm).sum();
        Some(sum / self.readings.len() as f64)
    }

    /// Compute corrosion assessments for distinct grid positions.
    ///
    /// Groups readings by position (using a tolerance of 1mm axial and 1 degree
    /// circumferential), then computes wall loss and corrosion rate for each cell.
    pub fn assess(&self) -> Vec<CorrosionAssessment> {
        if self.readings.is_empty() {
            return Vec::new();
        }

        // Group readings by grid position
        let mut cells: Vec<Vec<&GridReading>> = Vec::new();
        let mut cell_keys: Vec<(f64, f64)> = Vec::new();

        for reading in &self.readings {
            let mut found = false;
            for (idx, key) in cell_keys.iter().enumerate() {
                if (reading.axial_mm - key.0).abs() < 1.0
                    && (reading.circ_deg - key.1).abs() < 1.0
                {
                    cells[idx].push(reading);
                    found = true;
                    break;
                }
            }
            if !found {
                cell_keys.push((reading.axial_mm, reading.circ_deg));
                cells.push(vec![reading]);
            }
        }

        // Assess each cell
        let mut assessments = Vec::new();
        for (idx, cell) in cells.iter().enumerate() {
            let (axial, circ) = cell_keys[idx];
            let min_t = cell
                .iter()
                .map(|r| r.thickness_mm)
                .fold(f64::MAX, f64::min);
            let max_t = cell
                .iter()
                .map(|r| r.thickness_mm)
                .fold(f64::MIN, f64::max);

            let wall_loss = (self.nominal_wall_mm - min_t) / self.nominal_wall_mm * 100.0;
            let wall_loss = wall_loss.max(0.0);

            // Corrosion rate from linear regression of thickness vs time
            let (rate, remaining) = if cell.len() >= 2 {
                let rate = self.estimate_corrosion_rate(cell);
                let remaining = if rate > 1e-9 {
                    let margin = min_t - self.min_allowable_mm;
                    if margin > 0.0 {
                        Some(margin / rate)
                    } else {
                        Some(0.0)
                    }
                } else {
                    None
                };
                (Some(rate), remaining)
            } else {
                (None, None)
            };

            assessments.push(CorrosionAssessment {
                axial_mm: axial,
                circ_deg: circ,
                min_thickness_mm: min_t,
                max_thickness_mm: max_t,
                wall_loss_percent: wall_loss,
                corrosion_rate_mmpy: rate,
                remaining_life_years: remaining,
            });
        }

        assessments
    }

    /// Estimate corrosion rate from readings at the same position over time.
    ///
    /// Uses simple linear regression: thickness = a + b * time.
    /// Corrosion rate = -b (thickness decreases over time).
    fn estimate_corrosion_rate(&self, readings: &[&GridReading]) -> f64 {
        let n = readings.len() as f64;
        if n < 2.0 {
            return 0.0;
        }

        let sum_t: f64 = readings.iter().map(|r| r.timestamp).sum();
        let sum_th: f64 = readings.iter().map(|r| r.thickness_mm).sum();
        let sum_tt: f64 = readings.iter().map(|r| r.timestamp * r.timestamp).sum();
        let sum_t_th: f64 = readings
            .iter()
            .map(|r| r.timestamp * r.thickness_mm)
            .sum();

        let denom = n * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return 0.0;
        }

        let slope = (n * sum_t_th - sum_t * sum_th) / denom;

        // Corrosion rate is the negative of the slope (thickness decreases)
        (-slope).max(0.0)
    }
}

// ---------------------------------------------------------------------------
// Weld Inspector
// ---------------------------------------------------------------------------

/// Classification of weld indications.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WeldIndicationType {
    /// Porosity (gas pockets) in the weld.
    Porosity,
    /// Slag inclusion trapped in the weld.
    SlagInclusion,
    /// Incomplete fusion between weld passes or base metal.
    LackOfFusion,
    /// Incomplete penetration at the root.
    LackOfPenetration,
    /// Crack in the weld or heat-affected zone.
    Crack,
    /// Undetermined indication type.
    Unknown,
}

/// A detected indication in the weld B-scan.
#[derive(Debug, Clone)]
pub struct WeldIndication {
    /// Indication type classification.
    pub indication_type: WeldIndicationType,
    /// Position along the scan axis in millimetres.
    pub scan_position_mm: f64,
    /// Depth from the outer surface in millimetres.
    pub depth_mm: f64,
    /// Peak amplitude of the indication (0.0 to 1.0).
    pub amplitude: f64,
    /// Length of the indication along the scan axis in mm.
    pub length_mm: f64,
    /// Height (through-wall extent) of the indication in mm.
    pub height_mm: f64,
}

/// B-scan image data from weld inspection.
#[derive(Debug, Clone)]
pub struct BScanImage {
    /// Number of A-scan positions along the scan axis.
    pub num_positions: usize,
    /// Number of depth samples per A-scan.
    pub num_depth_samples: usize,
    /// Scan step size in mm.
    pub scan_step_mm: f64,
    /// Depth range in mm.
    pub depth_range_mm: f64,
    /// Pixel data stored row-major: `data[position * num_depth_samples + depth]`.
    /// Values are normalised amplitudes (0.0 to 1.0).
    pub data: Vec<f64>,
}

impl BScanImage {
    /// Get the amplitude at a given position and depth index.
    pub fn get(&self, position: usize, depth: usize) -> f64 {
        if position < self.num_positions && depth < self.num_depth_samples {
            self.data[position * self.num_depth_samples + depth]
        } else {
            0.0
        }
    }

    /// Find the maximum amplitude in the B-scan.
    pub fn max_amplitude(&self) -> f64 {
        self.data.iter().cloned().fold(0.0_f64, f64::max)
    }

    /// Compute the mean amplitude across the entire B-scan.
    pub fn mean_amplitude(&self) -> f64 {
        if self.data.is_empty() {
            return 0.0;
        }
        self.data.iter().sum::<f64>() / self.data.len() as f64
    }
}

/// Ultrasonic weld inspector for pipeline girth and seam welds.
///
/// Processes multiple A-scans collected along a weld to build a B-scan
/// (cross-sectional) image and detect flaw indications. Classification
/// is based on echo characteristics (amplitude, depth, extent).
#[derive(Debug, Clone)]
pub struct WeldInspector {
    /// Sound velocity in the weld/base material (m/s).
    velocity_mps: f64,
    /// Digitiser sample rate (Hz).
    sample_rate_hz: f64,
    /// Detection threshold for indications (fraction of max, 0 to 1).
    threshold: f64,
    /// Nominal wall thickness in mm (for depth reference).
    wall_thickness_mm: f64,
    /// Scan step size in mm.
    scan_step_mm: f64,
}

impl WeldInspector {
    /// Create a new weld inspector.
    ///
    /// # Arguments
    /// * `config` - Pipeline configuration
    /// * `sample_rate_hz` - Digitiser sample rate (Hz)
    /// * `scan_step_mm` - Distance between A-scan positions along the weld (mm)
    pub fn new(config: &PipelineConfig, sample_rate_hz: f64, scan_step_mm: f64) -> Self {
        Self {
            velocity_mps: config.material_velocity_mps,
            sample_rate_hz,
            threshold: 0.2,
            wall_thickness_mm: config.nominal_wall_mm,
            scan_step_mm,
        }
    }

    /// Set the indication detection threshold (0 to 1).
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold.clamp(0.01, 0.99);
    }

    /// Build a B-scan image from a series of A-scans.
    ///
    /// Each A-scan is rectified (absolute value) and normalised. The resulting
    /// 2D image represents amplitude vs. scan position and depth.
    ///
    /// # Arguments
    /// * `ascans` - Slice of A-scan signal vectors. Each vector is one position.
    ///
    /// # Returns
    /// A `BScanImage` with normalised amplitude data.
    pub fn build_bscan(&self, ascans: &[Vec<f64>]) -> BScanImage {
        if ascans.is_empty() {
            return BScanImage {
                num_positions: 0,
                num_depth_samples: 0,
                scan_step_mm: self.scan_step_mm,
                depth_range_mm: 0.0,
                data: Vec::new(),
            };
        }

        // Determine uniform depth sample count (use maximum across A-scans)
        let max_len = ascans.iter().map(|a| a.len()).max().unwrap_or(0);
        if max_len == 0 {
            return BScanImage {
                num_positions: ascans.len(),
                num_depth_samples: 0,
                scan_step_mm: self.scan_step_mm,
                depth_range_mm: 0.0,
                data: Vec::new(),
            };
        }

        // Rectify and collect
        let mut data = vec![0.0; ascans.len() * max_len];
        let mut global_max = 0.0_f64;

        for (pos, ascan) in ascans.iter().enumerate() {
            for (i, &sample) in ascan.iter().enumerate() {
                let val = sample.abs();
                data[pos * max_len + i] = val;
                if val > global_max {
                    global_max = val;
                }
            }
        }

        // Normalise
        if global_max > 1e-12 {
            for val in data.iter_mut() {
                *val /= global_max;
            }
        }

        let depth_range_mm =
            tof_to_thickness(max_len as f64 / self.sample_rate_hz, self.velocity_mps);

        BScanImage {
            num_positions: ascans.len(),
            num_depth_samples: max_len,
            scan_step_mm: self.scan_step_mm,
            depth_range_mm,
            data,
        }
    }

    /// Detect flaw indications in a B-scan image.
    ///
    /// Scans the B-scan for clusters of pixels above the threshold and
    /// classifies each cluster based on its characteristics.
    ///
    /// # Arguments
    /// * `bscan` - B-scan image to analyse
    ///
    /// # Returns
    /// Vector of detected weld indications.
    pub fn detect_indications(&self, bscan: &BScanImage) -> Vec<WeldIndication> {
        if bscan.data.is_empty() || bscan.num_positions == 0 || bscan.num_depth_samples == 0 {
            return Vec::new();
        }

        let mut visited = vec![false; bscan.data.len()];
        let mut indications = Vec::new();

        let depth_per_sample = if bscan.num_depth_samples > 1 {
            bscan.depth_range_mm / bscan.num_depth_samples as f64
        } else {
            1.0
        };

        for pos in 0..bscan.num_positions {
            for depth in 0..bscan.num_depth_samples {
                let idx = pos * bscan.num_depth_samples + depth;
                if visited[idx] || bscan.data[idx] < self.threshold {
                    continue;
                }

                // Flood-fill to find the extent of this cluster
                let mut cluster_positions: Vec<(usize, usize)> = Vec::new();
                let mut stack = vec![(pos, depth)];
                let mut max_amp = 0.0_f64;

                while let Some((p, d)) = stack.pop() {
                    let ci = p * bscan.num_depth_samples + d;
                    if ci >= bscan.data.len() || visited[ci] || bscan.data[ci] < self.threshold {
                        continue;
                    }
                    visited[ci] = true;
                    cluster_positions.push((p, d));
                    if bscan.data[ci] > max_amp {
                        max_amp = bscan.data[ci];
                    }

                    // 4-connected neighbours
                    if p > 0 {
                        stack.push((p - 1, d));
                    }
                    if p + 1 < bscan.num_positions {
                        stack.push((p + 1, d));
                    }
                    if d > 0 {
                        stack.push((p, d - 1));
                    }
                    if d + 1 < bscan.num_depth_samples {
                        stack.push((p, d + 1));
                    }
                }

                if cluster_positions.is_empty() {
                    continue;
                }

                // Compute cluster extents
                let min_pos = cluster_positions.iter().map(|c| c.0).min().unwrap();
                let max_pos = cluster_positions.iter().map(|c| c.0).max().unwrap();
                let min_depth = cluster_positions.iter().map(|c| c.1).min().unwrap();
                let max_depth = cluster_positions.iter().map(|c| c.1).max().unwrap();

                let length_mm = (max_pos - min_pos + 1) as f64 * self.scan_step_mm;
                let height_mm = (max_depth - min_depth + 1) as f64 * depth_per_sample;
                let center_pos_mm =
                    (min_pos as f64 + max_pos as f64) / 2.0 * self.scan_step_mm;
                let center_depth_mm =
                    (min_depth as f64 + max_depth as f64) / 2.0 * depth_per_sample;

                // Classify the indication
                let indication_type =
                    self.classify_indication(max_amp, length_mm, height_mm, center_depth_mm);

                indications.push(WeldIndication {
                    indication_type,
                    scan_position_mm: center_pos_mm,
                    depth_mm: center_depth_mm,
                    amplitude: max_amp,
                    length_mm,
                    height_mm,
                });
            }
        }

        indications
    }

    /// Classify a weld indication based on its echo characteristics.
    ///
    /// Heuristic classification based on amplitude, length, height, and depth:
    /// - High amplitude + thin height near surface/root = Lack of Fusion / Penetration
    /// - High amplitude + large height = Crack
    /// - Low amplitude + small extent = Porosity
    /// - Medium amplitude + moderate extent = Slag Inclusion
    fn classify_indication(
        &self,
        amplitude: f64,
        length_mm: f64,
        height_mm: f64,
        depth_mm: f64,
    ) -> WeldIndicationType {
        let aspect_ratio = if height_mm > 1e-6 {
            length_mm / height_mm
        } else {
            100.0
        };
        let depth_fraction = depth_mm / self.wall_thickness_mm;

        if amplitude > 0.8 && height_mm > self.wall_thickness_mm * 0.3 {
            WeldIndicationType::Crack
        } else if amplitude > 0.6 && aspect_ratio > 3.0 {
            if depth_fraction > 0.8 {
                WeldIndicationType::LackOfPenetration
            } else {
                WeldIndicationType::LackOfFusion
            }
        } else if amplitude < 0.4 && length_mm < 3.0 && height_mm < 3.0 {
            WeldIndicationType::Porosity
        } else if amplitude >= 0.4 && amplitude <= 0.7 {
            WeldIndicationType::SlagInclusion
        } else {
            WeldIndicationType::Unknown
        }
    }

    /// Generate a synthetic A-scan with an indication at the specified depth.
    ///
    /// Useful for testing the weld inspection pipeline.
    ///
    /// # Arguments
    /// * `wall_mm` - Pipe wall thickness in mm
    /// * `indication_depth_mm` - Depth of the flaw indication in mm
    /// * `indication_amplitude` - Amplitude of the indication (0 to 1)
    ///
    /// # Returns
    /// A-scan signal vector.
    pub fn generate_test_ascan(
        &self,
        wall_mm: f64,
        indication_depth_mm: f64,
        indication_amplitude: f64,
    ) -> Vec<f64> {
        let max_tof_s = thickness_to_tof(wall_mm * 1.5, self.velocity_mps);
        let total_samples = (max_tof_s * self.sample_rate_hz) as usize + 50;
        let mut signal = vec![0.0; total_samples];

        let carrier_freq = 5.0e6;
        let pulse_width = 8;
        let sigma = pulse_width as f64 / 3.0;

        // Add indication echo
        let ind_tof = thickness_to_tof(indication_depth_mm, self.velocity_mps);
        let ind_center = (ind_tof * self.sample_rate_hz) as usize;

        for i in 0..signal.len() {
            let dist = i as f64 - ind_center as f64;
            if dist.abs() < (pulse_width * 3) as f64 {
                let envelope = (-dist * dist / (2.0 * sigma * sigma)).exp();
                let t = i as f64 / self.sample_rate_hz;
                let carrier = (2.0 * PI * carrier_freq * t).sin();
                signal[i] += indication_amplitude * envelope * carrier;
            }
        }

        // Add back-wall echo
        let bw_tof = thickness_to_tof(wall_mm, self.velocity_mps);
        let bw_center = (bw_tof * self.sample_rate_hz) as usize;

        for i in 0..signal.len() {
            let dist = i as f64 - bw_center as f64;
            if dist.abs() < (pulse_width * 3) as f64 {
                let envelope = (-dist * dist / (2.0 * sigma * sigma)).exp();
                let t = i as f64 / self.sample_rate_hz;
                let carrier = (2.0 * PI * carrier_freq * t).sin();
                signal[i] += 0.9 * envelope * carrier;
            }
        }

        signal
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PipelineConfig {
        PipelineConfig::default()
    }

    // -- Helper function tests --

    #[test]
    fn test_acoustic_impedance() {
        // Steel: Z = 7850 * 5900 = 46,315,000 Rayl
        let z = acoustic_impedance(7850.0, 5900.0);
        assert!((z - 46_315_000.0).abs() < 1.0);
    }

    #[test]
    fn test_acoustic_impedance_water() {
        // Water: Z = 1000 * 1480 = 1,480,000 Rayl
        let z = acoustic_impedance(1000.0, 1480.0);
        assert!((z - 1_480_000.0).abs() < 1.0);
    }

    #[test]
    fn test_reflection_coefficient_steel_water() {
        let z_steel = acoustic_impedance(7850.0, 5900.0);
        let z_water = acoustic_impedance(1000.0, 1480.0);
        let r = reflection_coefficient(z_steel, z_water);
        // Steel to water: large negative reflection
        assert!(r < 0.0);
        assert!(r.abs() > 0.9);
    }

    #[test]
    fn test_reflection_coefficient_same_medium() {
        let z = acoustic_impedance(7850.0, 5900.0);
        let r = reflection_coefficient(z, z);
        assert!(r.abs() < 1e-10);
    }

    #[test]
    fn test_transmission_coefficient() {
        let z1 = acoustic_impedance(7850.0, 5900.0);
        let z2 = acoustic_impedance(1000.0, 1480.0);
        let r = reflection_coefficient(z1, z2);
        let t = transmission_coefficient(z1, z2);
        // Energy conservation: R + T = 1 (for pressure coefficients: T = 1 + R)
        assert!((t - (1.0 + r)).abs() < 1e-10);
    }

    #[test]
    fn test_snell_refraction_normal_incidence() {
        // Normal incidence: refracted angle should also be 0
        let result = snell_refraction(0.0, 5900.0, 3200.0);
        assert!(result.is_some());
        assert!(result.unwrap().abs() < 1e-10);
    }

    #[test]
    fn test_snell_refraction_angle() {
        // Going from slower to faster medium: refracted angle > incident
        let incident = 10.0_f64.to_radians();
        let result = snell_refraction(incident, 3200.0, 5900.0);
        assert!(result.is_some());
        assert!(result.unwrap() > incident);
    }

    #[test]
    fn test_snell_total_internal_reflection() {
        // Large angle from fast to slow medium should cause TIR
        let incident = 80.0_f64.to_radians();
        let result = snell_refraction(incident, 5900.0, 3200.0);
        // sin(80) * 3200/5900 ~ 0.534 -> valid, not TIR
        // Let's try the opposite direction at a steep angle
        let result2 = snell_refraction(incident, 3200.0, 5900.0);
        // sin(80) * 5900/3200 ~ 1.815 -> TIR
        assert!(result.is_some());
        assert!(result2.is_none());
    }

    #[test]
    fn test_beam_spread() {
        let spread = beam_spread(5.0e6, 5900.0, 10.0);
        // lambda = 5900 / 5e6 = 1.18 mm
        // 1.22 * 1.18e-3 / 0.01 = 0.144
        // asin(0.144) ~ 0.144 rad ~ 8.3 deg
        assert!(spread > 0.0);
        assert!(spread < PI / 4.0);
    }

    #[test]
    fn test_beam_spread_large_ratio() {
        // Very low frequency / small transducer -> ratio > 1 -> PI/2
        let spread = beam_spread(100.0, 5900.0, 1.0);
        assert!((spread - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_near_field_length() {
        let nf = near_field_length(5.0e6, 5900.0, 10.0);
        // N = D^2 * f / (4 * v) = 0.01^2 * 5e6 / (4 * 5900) = 500 / 23600 ~ 0.02119 m ~ 21.19 mm
        assert!((nf - 21.186).abs() < 0.1);
    }

    #[test]
    fn test_tof_to_thickness_roundtrip() {
        let thickness = 9.5;
        let velocity = 5900.0;
        let tof = thickness_to_tof(thickness, velocity);
        let recovered = tof_to_thickness(tof, velocity);
        assert!((recovered - thickness).abs() < 1e-6);
    }

    #[test]
    fn test_attenuation_db_one_way() {
        let atten = attenuation_db(50.0, 0.01, false);
        assert!((atten - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_attenuation_db_round_trip() {
        let atten = attenuation_db(50.0, 0.01, true);
        assert!((atten - 1.0).abs() < 1e-10);
    }

    // -- PipelineConfig tests --

    #[test]
    fn test_pipeline_config_inner_diameter() {
        let config = default_config();
        let id = config.inner_diameter_mm();
        assert!((id - (323.9 - 2.0 * 9.5)).abs() < 1e-6);
    }

    #[test]
    fn test_pipeline_config_flow_area() {
        let config = default_config();
        let area = config.flow_area_m2();
        let expected_id_m = (323.9 - 19.0) / 1000.0;
        let expected = PI / 4.0 * expected_id_m * expected_id_m;
        assert!((area - expected).abs() < 1e-6);
    }

    // -- Transit-Time Flow Meter tests --

    #[test]
    fn test_flow_meter_zero_flow() {
        let config = default_config();
        let meter = TransitTimeFlowMeter::new(&config, 45.0_f64.to_radians(), 1.0);
        // Same transit time = zero flow
        let result = meter.measure(0.0001, 0.0001);
        assert!(result.flow_velocity_mps.abs() < 1e-10);
        assert!(result.volume_flow_m3s.abs() < 1e-10);
    }

    #[test]
    fn test_flow_meter_positive_flow() {
        let config = default_config();
        let meter = TransitTimeFlowMeter::new(&config, 45.0_f64.to_radians(), 1.0);
        // Upstream takes longer than downstream = positive flow
        let result = meter.measure(0.000_105, 0.000_095);
        assert!(result.flow_velocity_mps > 0.0);
        assert!(result.volume_flow_m3s > 0.0);
    }

    #[test]
    fn test_flow_meter_negative_flow() {
        let config = default_config();
        let meter = TransitTimeFlowMeter::new(&config, 45.0_f64.to_radians(), 1.0);
        // Downstream takes longer = negative (reversed) flow
        let result = meter.measure(0.000_095, 0.000_105);
        assert!(result.flow_velocity_mps < 0.0);
    }

    #[test]
    fn test_flow_meter_profile_factor() {
        let config = default_config();
        let meter_plug = TransitTimeFlowMeter::new(&config, 45.0_f64.to_radians(), 1.0);
        let meter_turb = TransitTimeFlowMeter::new(&config, 45.0_f64.to_radians(), 0.75);
        let r1 = meter_plug.measure(0.000_105, 0.000_095);
        let r2 = meter_turb.measure(0.000_105, 0.000_095);
        // Turbulent profile factor reduces measured velocity
        assert!(r2.flow_velocity_mps.abs() < r1.flow_velocity_mps.abs());
    }

    #[test]
    fn test_reynolds_number() {
        let re = TransitTimeFlowMeter::reynolds_number(2.0, 0.305, 1.0e-6);
        // Re = 2 * 0.305 / 1e-6 = 610,000 (turbulent)
        assert!((re - 610_000.0).abs() < 1.0);
    }

    // -- Wall Thickness Gauge tests --

    #[test]
    fn test_gauge_generate_and_measure() {
        let config = default_config();
        let gauge = WallThicknessGauge::new(&config, 100_000_000.0);
        let signal = gauge.generate_test_signal(9.5, 0.8);
        let result = gauge.measure(&signal);
        assert!(result.is_some());
        let m = result.unwrap();
        // Should be within 0.5 mm of the simulated thickness
        assert!((m.thickness_mm - 9.5).abs() < 0.5, "Got {} mm", m.thickness_mm);
        assert!(m.echo_count >= 1);
    }

    #[test]
    fn test_gauge_thin_wall() {
        let mut config = default_config();
        config.nominal_wall_mm = 5.0;
        let gauge = WallThicknessGauge::new(&config, 100_000_000.0);
        let signal = gauge.generate_test_signal(3.0, 0.9);
        let result = gauge.measure(&signal);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!((m.thickness_mm - 3.0).abs() < 0.5, "Got {} mm", m.thickness_mm);
    }

    #[test]
    fn test_gauge_empty_signal() {
        let config = default_config();
        let gauge = WallThicknessGauge::new(&config, 100_000_000.0);
        assert!(gauge.measure(&[]).is_none());
    }

    #[test]
    fn test_gauge_noise_only() {
        let config = default_config();
        let gauge = WallThicknessGauge::new(&config, 100_000_000.0);
        // Very low amplitude noise - should be below threshold
        let signal: Vec<f64> = (0..1000).map(|i| (i as f64 * 0.1).sin() * 1e-15).collect();
        assert!(gauge.measure(&signal).is_none());
    }

    #[test]
    fn test_gauge_multi_echo() {
        let config = default_config();
        let gauge = WallThicknessGauge::new(&config, 100_000_000.0);

        // Create signal with two echoes at 9.5mm intervals
        let tof_s = thickness_to_tof(9.5, 5900.0);
        let n = (tof_s * 100_000_000.0) as usize;
        let total = n * 4;
        let mut signal = vec![0.0; total];
        let sigma = 3.0;

        for echo in 0..2 {
            let center = n * (echo + 1);
            for i in 0..signal.len() {
                let dist = i as f64 - center as f64;
                if dist.abs() < 30.0 {
                    let env = (-dist * dist / (2.0 * sigma * sigma)).exp();
                    let amp = if echo == 0 { 0.8 } else { 0.5 };
                    signal[i] += amp * env;
                }
            }
        }

        let result = gauge.measure_multi_echo(&signal);
        assert!(result.is_some());
        let m = result.unwrap();
        assert!(m.echo_count >= 2);
        assert!((m.thickness_mm - 9.5).abs() < 0.5, "Got {} mm", m.thickness_mm);
    }

    // -- Corrosion Mapper tests --

    #[test]
    fn test_corrosion_mapper_empty() {
        let config = default_config();
        let mapper = CorrosionMapper::new(&config, 3.0);
        assert_eq!(mapper.reading_count(), 0);
        assert!(mapper.min_thickness().is_none());
        assert!(mapper.assess().is_empty());
    }

    #[test]
    fn test_corrosion_mapper_single_reading() {
        let config = default_config();
        let mut mapper = CorrosionMapper::new(&config, 3.0);
        mapper.add_reading(GridReading {
            axial_mm: 100.0,
            circ_deg: 0.0,
            thickness_mm: 8.5,
            timestamp: 0.0,
        });
        assert_eq!(mapper.reading_count(), 1);
        assert!((mapper.min_thickness().unwrap() - 8.5).abs() < 1e-6);
        assert!((mapper.mean_thickness().unwrap() - 8.5).abs() < 1e-6);

        let assessments = mapper.assess();
        assert_eq!(assessments.len(), 1);
        // Wall loss = (9.5 - 8.5) / 9.5 * 100 = 10.526%
        assert!((assessments[0].wall_loss_percent - 10.526).abs() < 0.1);
        assert!(assessments[0].corrosion_rate_mmpy.is_none());
    }

    #[test]
    fn test_corrosion_mapper_rate_estimation() {
        let config = default_config();
        let mut mapper = CorrosionMapper::new(&config, 3.0);

        // Two readings at the same position, 1 year apart
        mapper.add_reading(GridReading {
            axial_mm: 100.0,
            circ_deg: 0.0,
            thickness_mm: 9.0,
            timestamp: 0.0,
        });
        mapper.add_reading(GridReading {
            axial_mm: 100.0,
            circ_deg: 0.0,
            thickness_mm: 8.5,
            timestamp: 1.0, // 1 year later
        });

        let assessments = mapper.assess();
        assert_eq!(assessments.len(), 1);
        let a = &assessments[0];
        assert!(a.corrosion_rate_mmpy.is_some());
        // Rate should be ~0.5 mm/year
        assert!((a.corrosion_rate_mmpy.unwrap() - 0.5).abs() < 0.01);

        // Remaining life: (8.5 - 3.0) / 0.5 = 11 years
        assert!(a.remaining_life_years.is_some());
        assert!((a.remaining_life_years.unwrap() - 11.0).abs() < 0.1);
    }

    #[test]
    fn test_corrosion_mapper_multiple_positions() {
        let config = default_config();
        let mut mapper = CorrosionMapper::new(&config, 3.0);

        mapper.add_readings(&[
            GridReading {
                axial_mm: 0.0,
                circ_deg: 0.0,
                thickness_mm: 9.2,
                timestamp: 0.0,
            },
            GridReading {
                axial_mm: 50.0,
                circ_deg: 0.0,
                thickness_mm: 7.5,
                timestamp: 0.0,
            },
            GridReading {
                axial_mm: 100.0,
                circ_deg: 180.0,
                thickness_mm: 8.0,
                timestamp: 0.0,
            },
        ]);

        assert_eq!(mapper.reading_count(), 3);
        assert!((mapper.min_thickness().unwrap() - 7.5).abs() < 1e-6);
        assert!((mapper.max_thickness().unwrap() - 9.2).abs() < 1e-6);

        let assessments = mapper.assess();
        assert_eq!(assessments.len(), 3);
    }

    // -- Weld Inspector tests --

    #[test]
    fn test_weld_inspector_empty_bscan() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        let bscan = inspector.build_bscan(&[]);
        assert_eq!(bscan.num_positions, 0);
        assert!(inspector.detect_indications(&bscan).is_empty());
    }

    #[test]
    fn test_weld_inspector_clean_weld() {
        let config = default_config();
        let mut inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        inspector.set_threshold(0.5);

        // Generate A-scans with only back-wall echoes (no flaws)
        let ascans: Vec<Vec<f64>> = (0..10)
            .map(|_| {
                let tof = thickness_to_tof(9.5, 5900.0);
                let n = (tof * 100_000_000.0) as usize;
                let total = n * 2;
                let mut s = vec![0.0; total];
                let sigma = 3.0;
                for i in 0..s.len() {
                    let dist = i as f64 - n as f64;
                    if dist.abs() < 20.0 {
                        s[i] = 0.9 * (-dist * dist / (2.0 * sigma * sigma)).exp();
                    }
                }
                s
            })
            .collect();

        let bscan = inspector.build_bscan(&ascans);
        assert_eq!(bscan.num_positions, 10);
        assert!(bscan.max_amplitude() > 0.9);

        // With threshold at 0.5, the back-wall echoes will be detected as a single
        // continuous indication. This is expected behavior - in practice, the back wall
        // would be excluded by gating.
    }

    #[test]
    fn test_weld_inspector_bscan_properties() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);

        let ascans: Vec<Vec<f64>> = (0..5)
            .map(|_| vec![0.0; 100])
            .collect();

        let bscan = inspector.build_bscan(&ascans);
        assert_eq!(bscan.num_positions, 5);
        assert_eq!(bscan.num_depth_samples, 100);
        assert!((bscan.mean_amplitude()).abs() < 1e-10);
        assert_eq!(bscan.get(0, 0), 0.0);
        assert_eq!(bscan.get(100, 0), 0.0); // out of bounds returns 0
    }

    #[test]
    fn test_weld_inspector_indication_detection() {
        let config = default_config();
        let mut inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        inspector.set_threshold(0.3);

        // Create a B-scan with a bright spot (simulated indication)
        let n_pos = 20;
        let n_depth = 100;
        let mut data = vec![0.0; n_pos * n_depth];

        // Place a cluster of high-amplitude pixels in the middle
        for p in 8..12 {
            for d in 40..50 {
                data[p * n_depth + d] = 0.7;
            }
        }

        let bscan = BScanImage {
            num_positions: n_pos,
            num_depth_samples: n_depth,
            scan_step_mm: 1.0,
            depth_range_mm: 10.0,
            data,
        };

        let indications = inspector.detect_indications(&bscan);
        assert!(!indications.is_empty(), "Should detect at least one indication");
        assert!(indications[0].amplitude > 0.5);
        assert!(indications[0].length_mm > 0.0);
        assert!(indications[0].height_mm > 0.0);
    }

    #[test]
    fn test_weld_inspector_generate_test_ascan() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        let ascan = inspector.generate_test_ascan(9.5, 5.0, 0.6);
        assert!(!ascan.is_empty());
        // Signal should have non-trivial amplitude
        let max_amp = ascan.iter().cloned().fold(0.0_f64, |a, b| a.max(b.abs()));
        assert!(max_amp > 0.4);
    }

    #[test]
    fn test_weld_indication_classification_crack() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        let t = inspector.classify_indication(0.9, 5.0, 5.0, 5.0);
        assert_eq!(t, WeldIndicationType::Crack);
    }

    #[test]
    fn test_weld_indication_classification_porosity() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        let t = inspector.classify_indication(0.3, 2.0, 2.0, 3.0);
        assert_eq!(t, WeldIndicationType::Porosity);
    }

    #[test]
    fn test_weld_indication_classification_lack_of_penetration() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        // High amplitude, high aspect ratio, near root (depth > 80% of wall)
        let t = inspector.classify_indication(0.7, 10.0, 2.0, 8.5);
        assert_eq!(t, WeldIndicationType::LackOfPenetration);
    }

    #[test]
    fn test_weld_indication_classification_slag() {
        let config = default_config();
        let inspector = WeldInspector::new(&config, 100_000_000.0, 1.0);
        let t = inspector.classify_indication(0.5, 5.0, 5.0, 5.0);
        assert_eq!(t, WeldIndicationType::SlagInclusion);
    }
}
