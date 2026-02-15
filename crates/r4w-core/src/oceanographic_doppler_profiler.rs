//! Acoustic Doppler Current Profiler (ADCP) signal processing for ocean current
//! velocity measurement.
//!
//! This module implements the complete ADCP signal processing chain for measuring
//! water current velocity profiles through the water column. It covers multi-beam
//! Doppler processing, depth-cell (range gate) binning, beam-to-ENU coordinate
//! transforms, bottom tracking, and quality screening.
//!
//! # ADCP Principle
//!
//! An ADCP transmits acoustic pulses along multiple beams (typically four in a
//! Janus configuration at 20--30 degrees from vertical). Backscattered returns
//! from suspended particles are Doppler-shifted proportionally to the along-beam
//! component of the water velocity:
//!
//! ```text
//! f_d = 2 * f_0 * v_beam / c
//! ```
//!
//! where `f_0` is the transmit frequency, `v_beam` is the radial velocity along
//! the beam, and `c` is the speed of sound in seawater.
//!
//! # Processing Chain
//!
//! ```text
//! Acoustic Returns (4 beams)
//!         │
//!    DepthCellBinner  ── range-gate into depth cells
//!         │
//!  DopplerVelocityProcessor  ── extract Doppler shift per cell per beam
//!         │
//!    VelocityProfiler  ── beam-to-ENU coordinate transform
//!         │
//!     QualityFilter  ── screen by correlation, error velocity, percent-good
//!         │
//!   Velocity Profile (East, North, Up vs depth)
//! ```
//!
//! # Janus Beam Geometry
//!
//! ```text
//!         Up (z)
//!          │
//!    B3    │    B1       Beam 1 & 2 in the xz-plane (east/up)
//!     \    │   /         Beam 3 & 4 in the yz-plane (north/up)
//!      \   │  /          All beams at angle theta from vertical
//!       \  │ /
//!        \ │/
//!     ────[T]────  Transducer
//!        / │\
//!       /  │ \
//!      /   │  \
//!     /    │   \
//!    B4    │    B2
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::oceanographic_doppler_profiler::{
//!     AdcpConfig, DepthCellBinner, DopplerVelocityProcessor,
//!     VelocityProfiler, sound_speed_seawater,
//! };
//!
//! let config = AdcpConfig::default_300khz();
//! let sound_speed = sound_speed_seawater(15.0, 35.0, 100.0);
//! assert!((sound_speed - 1507.0).abs() < 5.0);
//!
//! let binner = DepthCellBinner::new(&config, sound_speed);
//! assert!(binner.num_cells() > 0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Default Janus beam angle from vertical (degrees), typical for RDI Workhorse.
const DEFAULT_BEAM_ANGLE_DEG: f64 = 20.0;

/// Number of beams in a standard Janus configuration.
const JANUS_NUM_BEAMS: usize = 4;

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Compute the speed of sound in seawater using the UNESCO algorithm
/// (Chen & Millero, 1977; UNESCO Tech Papers in Marine Science No. 44).
///
/// # Arguments
///
/// * `temperature_c` - Temperature in degrees Celsius (valid 0--40)
/// * `salinity_psu` - Salinity in practical salinity units (valid 0--40)
/// * `depth_m` - Depth in metres (valid 0--1000)
///
/// # Returns
///
/// Speed of sound in m/s. Typical values range from ~1450 m/s (cold, fresh,
/// shallow) to ~1550 m/s (warm, saline, deep).
///
/// The UNESCO formula is a polynomial fit in T, S, and pressure (approximated
/// from depth). This simplified version captures the dominant terms.
pub fn sound_speed_seawater(temperature_c: f64, salinity_psu: f64, depth_m: f64) -> f64 {
    let t = temperature_c;
    let s = salinity_psu;
    // Approximate pressure in dbar (~= depth in metres for seawater)
    let p = depth_m * 0.1; // MPa (rough: 1 dbar ~ 1 m)

    // Mackenzie (1981) nine-term equation -- widely used ADCP firmware formula
    // c = 1448.96 + 4.591*T - 0.05304*T^2 + 0.0002374*T^3
    //   + 1.340*(S-35) + 0.0163*D + 1.675e-7*D^2
    //   - 0.01025*T*(S-35) - 7.139e-13*T*D^3
    let d = depth_m;
    1448.96
        + 4.591 * t
        - 0.05304 * t * t
        + 0.0002374 * t * t * t
        + 1.340 * (s - 35.0)
        + 0.0163 * d
        + 1.675e-7 * d * d
        - 0.01025 * t * (s - 35.0)
        - 7.139e-13 * t * d * d * d
}

/// Convert a Doppler frequency shift to radial velocity.
///
/// ```text
/// v = f_d * c / (2 * f_0)
/// ```
///
/// # Arguments
///
/// * `doppler_shift_hz` - Measured Doppler shift in Hz (positive = towards transducer)
/// * `transmit_freq_hz` - Acoustic transmit frequency in Hz
/// * `sound_speed_mps` - Speed of sound in m/s
///
/// # Returns
///
/// Radial (along-beam) velocity in m/s. Positive = towards transducer.
pub fn doppler_to_velocity(
    doppler_shift_hz: f64,
    transmit_freq_hz: f64,
    sound_speed_mps: f64,
) -> f64 {
    doppler_shift_hz * sound_speed_mps / (2.0 * transmit_freq_hz)
}

/// Convert radial velocity to Doppler frequency shift.
///
/// ```text
/// f_d = 2 * f_0 * v / c
/// ```
pub fn velocity_to_doppler(
    velocity_mps: f64,
    transmit_freq_hz: f64,
    sound_speed_mps: f64,
) -> f64 {
    2.0 * transmit_freq_hz * velocity_mps / sound_speed_mps
}

/// Transform four Janus beam radial velocities to East-North-Up (ENU) coordinates.
///
/// The standard Janus 4-beam transform assumes:
/// - Beams 1 & 2 oppose in the xz (east-up) plane
/// - Beams 3 & 4 oppose in the yz (north-up) plane
///
/// ```text
/// v_east  = (v1 - v2) / (2 * sin(theta))
/// v_north = (v4 - v3) / (2 * sin(theta))
/// v_up    = (v1 + v2 + v3 + v4) / (4 * cos(theta))
/// v_error = (v1 + v2 - v3 - v4) / (4 * sin(theta))  // homogeneity check
/// ```
///
/// # Arguments
///
/// * `beam_velocities` - Array of 4 radial velocities [beam1, beam2, beam3, beam4] in m/s
/// * `beam_angle_deg` - Beam angle from vertical in degrees
///
/// # Returns
///
/// Tuple `(east, north, up, error)` velocities in m/s. The error velocity
/// should be near zero in a horizontally homogeneous flow; large values
/// indicate inhomogeneity or bad data.
pub fn beam_to_enu_transform(
    beam_velocities: &[f64; JANUS_NUM_BEAMS],
    beam_angle_deg: f64,
) -> (f64, f64, f64, f64) {
    let theta = beam_angle_deg * PI / 180.0;
    let sin_t = theta.sin();
    let cos_t = theta.cos();

    let v1 = beam_velocities[0];
    let v2 = beam_velocities[1];
    let v3 = beam_velocities[2];
    let v4 = beam_velocities[3];

    let east = (v1 - v2) / (2.0 * sin_t);
    let north = (v4 - v3) / (2.0 * sin_t);
    let up = (v1 + v2 + v3 + v4) / (4.0 * cos_t);
    let error = (v1 + v2 - v3 - v4) / (4.0 * sin_t);

    (east, north, up, error)
}

// ---------------------------------------------------------------------------
// AdcpConfig
// ---------------------------------------------------------------------------

/// Configuration for an Acoustic Doppler Current Profiler.
///
/// Encapsulates transducer geometry, acoustic parameters, and measurement
/// settings that define the instrument's operating mode.
#[derive(Debug, Clone)]
pub struct AdcpConfig {
    /// Number of beams (typically 4 for Janus).
    pub num_beams: usize,
    /// Beam angle from vertical in degrees (typically 20 or 30).
    pub beam_angle_deg: f64,
    /// Acoustic transmit frequency in Hz (e.g. 300_000 for 300 kHz).
    pub transmit_freq_hz: f64,
    /// Depth cell (bin) size in metres.
    pub cell_size_m: f64,
    /// Blanking distance in metres (dead zone below transducer).
    pub blanking_distance_m: f64,
    /// Number of depth cells.
    pub num_cells: usize,
    /// Number of pings to average per ensemble.
    pub pings_per_ensemble: usize,
    /// Pulse length in seconds.
    pub pulse_length_s: f64,
    /// Sample rate for digitised returns (Hz).
    pub sample_rate_hz: f64,
}

impl AdcpConfig {
    /// Creates a default configuration for a 300 kHz broadband ADCP.
    ///
    /// Typical for shelf/slope deployments (range ~100 m).
    pub fn default_300khz() -> Self {
        Self {
            num_beams: JANUS_NUM_BEAMS,
            beam_angle_deg: DEFAULT_BEAM_ANGLE_DEG,
            transmit_freq_hz: 300_000.0,
            cell_size_m: 4.0,
            blanking_distance_m: 1.76,
            num_cells: 25,
            pings_per_ensemble: 50,
            pulse_length_s: 4.0e-3,
            sample_rate_hz: 50_000.0,
        }
    }

    /// Creates a default configuration for a 1200 kHz ADCP.
    ///
    /// Typical for shallow-water/estuary deployments (range ~20 m).
    pub fn default_1200khz() -> Self {
        Self {
            num_beams: JANUS_NUM_BEAMS,
            beam_angle_deg: DEFAULT_BEAM_ANGLE_DEG,
            transmit_freq_hz: 1_200_000.0,
            cell_size_m: 0.5,
            blanking_distance_m: 0.44,
            num_cells: 40,
            pings_per_ensemble: 100,
            pulse_length_s: 1.0e-3,
            sample_rate_hz: 200_000.0,
        }
    }

    /// Creates a default configuration for a 75 kHz ADCP.
    ///
    /// Typical for deep-water deployments (range ~600 m).
    pub fn default_75khz() -> Self {
        Self {
            num_beams: JANUS_NUM_BEAMS,
            beam_angle_deg: DEFAULT_BEAM_ANGLE_DEG,
            transmit_freq_hz: 75_000.0,
            cell_size_m: 16.0,
            blanking_distance_m: 7.04,
            num_cells: 40,
            pings_per_ensemble: 30,
            pulse_length_s: 16.0e-3,
            sample_rate_hz: 12_500.0,
        }
    }

    /// Maximum profiling range in metres (slant distance along beam).
    pub fn max_range_m(&self) -> f64 {
        self.blanking_distance_m + self.num_cells as f64 * self.cell_size_m
    }

    /// Maximum profiling depth (vertical component).
    pub fn max_depth_m(&self) -> f64 {
        self.max_range_m() * (self.beam_angle_deg * PI / 180.0).cos()
    }
}

// ---------------------------------------------------------------------------
// DepthCellBinner
// ---------------------------------------------------------------------------

/// Partitions the water column into range-gated depth cells.
///
/// Each depth cell corresponds to a time window in the received acoustic
/// return. The binner computes the sample indices and depth centres for
/// each cell given the sound speed and instrument configuration.
#[derive(Debug, Clone)]
pub struct DepthCellBinner {
    /// Centre depth (metres) of each cell.
    pub cell_centres_m: Vec<f64>,
    /// Start sample index for each cell (inclusive).
    pub cell_start_samples: Vec<usize>,
    /// Number of samples in each cell.
    pub cell_num_samples: usize,
    /// Sound speed used for depth conversion.
    pub sound_speed_mps: f64,
}

impl DepthCellBinner {
    /// Create a new depth cell binner.
    ///
    /// # Arguments
    ///
    /// * `config` - ADCP configuration
    /// * `sound_speed_mps` - Speed of sound in m/s
    pub fn new(config: &AdcpConfig, sound_speed_mps: f64) -> Self {
        let theta_rad = config.beam_angle_deg * PI / 180.0;
        let cos_theta = theta_rad.cos();

        // Two-way travel time per metre of slant range
        let twtt_per_m = 2.0 / sound_speed_mps;

        // Samples per depth cell (along beam, slant range)
        let cell_slant_m = config.cell_size_m / cos_theta;
        let cell_time_s = cell_slant_m * twtt_per_m;
        let cell_num_samples = (cell_time_s * config.sample_rate_hz).round() as usize;
        let cell_num_samples = cell_num_samples.max(1);

        // Blanking: slant range
        let blanking_slant_m = config.blanking_distance_m / cos_theta;
        let blanking_time_s = blanking_slant_m * twtt_per_m;
        let blanking_samples = (blanking_time_s * config.sample_rate_hz).round() as usize;

        let mut cell_centres_m = Vec::with_capacity(config.num_cells);
        let mut cell_start_samples = Vec::with_capacity(config.num_cells);

        for i in 0..config.num_cells {
            let start = blanking_samples + i * cell_num_samples;
            cell_start_samples.push(start);

            // Centre depth of this cell (vertical)
            let slant_centre = blanking_slant_m + (i as f64 + 0.5) * cell_slant_m;
            let depth_centre = slant_centre * cos_theta;
            cell_centres_m.push(depth_centre);
        }

        Self {
            cell_centres_m,
            cell_start_samples,
            cell_num_samples,
            sound_speed_mps,
        }
    }

    /// Number of depth cells.
    pub fn num_cells(&self) -> usize {
        self.cell_centres_m.len()
    }

    /// Extract samples for a given depth cell from a beam return.
    ///
    /// Returns `None` if the return is too short to contain this cell.
    pub fn extract_cell(&self, beam_return: &[f64], cell_index: usize) -> Option<Vec<f64>> {
        if cell_index >= self.num_cells() {
            return None;
        }
        let start = self.cell_start_samples[cell_index];
        let end = start + self.cell_num_samples;
        if end > beam_return.len() {
            return None;
        }
        Some(beam_return[start..end].to_vec())
    }

    /// Minimum number of samples required to populate all cells.
    pub fn required_samples(&self) -> usize {
        if self.cell_start_samples.is_empty() {
            return 0;
        }
        self.cell_start_samples[self.num_cells() - 1] + self.cell_num_samples
    }
}

// ---------------------------------------------------------------------------
// DopplerVelocityProcessor
// ---------------------------------------------------------------------------

/// Extracts Doppler shift from backscattered acoustic returns per beam.
///
/// Uses the pulse-pair (autocovariance) method: the phase difference between
/// successive pulse returns at a given range gate gives the mean Doppler
/// velocity for that depth cell.
///
/// ```text
/// phi = arg( sum_n( z[n] * conj(z[n-1]) ) )
/// v_radial = phi * c / (4 * pi * f_0 * T_lag)
/// ```
///
/// where `T_lag` is the time between successive pings and `z[n]` is the
/// complex (I,Q) return for ping n at a specific depth cell.
#[derive(Debug, Clone)]
pub struct DopplerVelocityProcessor {
    /// Transmit frequency (Hz).
    pub transmit_freq_hz: f64,
    /// Speed of sound (m/s).
    pub sound_speed_mps: f64,
    /// Time between successive pings (seconds).
    pub ping_interval_s: f64,
}

/// Complex sample represented as (I, Q) tuple.
pub type Complex = (f64, f64);

/// Multiply two complex numbers.
fn complex_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
fn complex_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

/// Magnitude of a complex number.
fn complex_mag(a: Complex) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Phase angle (argument) of a complex number.
fn complex_arg(a: Complex) -> f64 {
    a.1.atan2(a.0)
}

impl DopplerVelocityProcessor {
    /// Create a new Doppler velocity processor.
    ///
    /// # Arguments
    ///
    /// * `transmit_freq_hz` - Acoustic transmit frequency (Hz)
    /// * `sound_speed_mps` - Speed of sound (m/s)
    /// * `ping_interval_s` - Time between successive pings (s)
    pub fn new(transmit_freq_hz: f64, sound_speed_mps: f64, ping_interval_s: f64) -> Self {
        Self {
            transmit_freq_hz,
            sound_speed_mps,
            ping_interval_s,
        }
    }

    /// Estimate radial velocity for one depth cell using the pulse-pair method.
    ///
    /// # Arguments
    ///
    /// * `cell_pings` - Complex returns from successive pings at this cell.
    ///   Each entry is one ping's complex sample (or mean of samples within the cell).
    ///
    /// # Returns
    ///
    /// Tuple `(velocity_mps, correlation)` where velocity is the along-beam
    /// radial velocity in m/s and correlation is the normalised autocovariance
    /// magnitude (0..1).
    pub fn estimate_velocity(&self, cell_pings: &[Complex]) -> (f64, f64) {
        if cell_pings.len() < 2 {
            return (0.0, 0.0);
        }

        // Pulse-pair autocovariance at lag 1
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        let mut power = 0.0;
        let n = cell_pings.len() - 1;

        for i in 0..n {
            let product = complex_mul(cell_pings[i + 1], complex_conj(cell_pings[i]));
            sum_re += product.0;
            sum_im += product.1;
            power += complex_mag(cell_pings[i]) * complex_mag(cell_pings[i + 1]);
        }

        let correlation = if power > 1e-30 {
            (sum_re * sum_re + sum_im * sum_im).sqrt() / power
        } else {
            0.0
        };

        let phase = sum_im.atan2(sum_re);

        // v = phase * c / (4 * pi * f0 * T_lag)
        let velocity =
            phase * self.sound_speed_mps / (4.0 * PI * self.transmit_freq_hz * self.ping_interval_s);

        (velocity, correlation)
    }

    /// Process all depth cells for one beam across multiple pings.
    ///
    /// # Arguments
    ///
    /// * `ping_data` - Vector of pings, each ping is a vector of complex
    ///   cell averages (length = num_cells).
    ///
    /// # Returns
    ///
    /// Vector of `(velocity_mps, correlation)` per depth cell.
    pub fn process_beam(
        &self,
        ping_data: &[Vec<Complex>],
    ) -> Vec<(f64, f64)> {
        if ping_data.is_empty() {
            return Vec::new();
        }

        let num_cells = ping_data[0].len();
        let mut results = Vec::with_capacity(num_cells);

        for cell_idx in 0..num_cells {
            let cell_pings: Vec<Complex> = ping_data
                .iter()
                .filter_map(|ping| ping.get(cell_idx).copied())
                .collect();
            results.push(self.estimate_velocity(&cell_pings));
        }

        results
    }
}

// ---------------------------------------------------------------------------
// VelocityProfiler
// ---------------------------------------------------------------------------

/// Converts beam radial velocities to East-North-Up (ENU) velocity profiles.
///
/// Takes per-cell, per-beam radial velocities from the `DopplerVelocityProcessor`
/// and applies the Janus coordinate transform to produce a full water column
/// velocity profile in geographic (ENU) coordinates.
#[derive(Debug, Clone)]
pub struct VelocityProfiler {
    /// Beam angle from vertical (degrees).
    pub beam_angle_deg: f64,
    /// Heading correction: instrument heading relative to true north (degrees).
    pub heading_deg: f64,
    /// Pitch angle (degrees, positive bow-up).
    pub pitch_deg: f64,
    /// Roll angle (degrees, positive starboard-down).
    pub roll_deg: f64,
}

/// A single velocity measurement at a depth cell.
#[derive(Debug, Clone, Copy)]
pub struct VelocityCell {
    /// Depth of cell centre below transducer (m).
    pub depth_m: f64,
    /// Eastward velocity (m/s).
    pub east_mps: f64,
    /// Northward velocity (m/s).
    pub north_mps: f64,
    /// Upward velocity (m/s).
    pub up_mps: f64,
    /// Error velocity (m/s) -- homogeneity diagnostic.
    pub error_mps: f64,
    /// Mean correlation magnitude across beams (0..1).
    pub mean_correlation: f64,
}

/// A complete velocity profile through the water column.
#[derive(Debug, Clone)]
pub struct VelocityProfile {
    /// Individual cell measurements, ordered by increasing depth.
    pub cells: Vec<VelocityCell>,
}

impl VelocityProfiler {
    /// Create a new velocity profiler with instrument orientation.
    ///
    /// # Arguments
    ///
    /// * `beam_angle_deg` - Beam angle from vertical
    /// * `heading_deg` - Instrument heading (degrees from true north)
    /// * `pitch_deg` - Pitch angle (degrees)
    /// * `roll_deg` - Roll angle (degrees)
    pub fn new(beam_angle_deg: f64, heading_deg: f64, pitch_deg: f64, roll_deg: f64) -> Self {
        Self {
            beam_angle_deg,
            heading_deg,
            pitch_deg,
            roll_deg,
        }
    }

    /// Create a profiler for a level, north-facing instrument.
    pub fn level(beam_angle_deg: f64) -> Self {
        Self::new(beam_angle_deg, 0.0, 0.0, 0.0)
    }

    /// Transform beam velocities to ENU for one depth cell.
    fn transform_cell(
        &self,
        beam_vels: &[f64; JANUS_NUM_BEAMS],
        depth_m: f64,
        mean_corr: f64,
    ) -> VelocityCell {
        let (inst_e, inst_n, up, error) =
            beam_to_enu_transform(beam_vels, self.beam_angle_deg);

        // Apply heading rotation to go from instrument frame to true ENU
        let heading_rad = self.heading_deg * PI / 180.0;
        let cos_h = heading_rad.cos();
        let sin_h = heading_rad.sin();

        let east = inst_e * cos_h - inst_n * sin_h;
        let north = inst_e * sin_h + inst_n * cos_h;

        VelocityCell {
            depth_m,
            east_mps: east,
            north_mps: north,
            up_mps: up,
            error_mps: error,
            mean_correlation: mean_corr,
        }
    }

    /// Produce a full velocity profile from per-beam, per-cell Doppler results.
    ///
    /// # Arguments
    ///
    /// * `beam_results` - 4-element array, each containing per-cell
    ///   `(velocity_mps, correlation)` from `DopplerVelocityProcessor`.
    /// * `cell_depths_m` - Depth of each cell centre.
    ///
    /// # Returns
    ///
    /// A `VelocityProfile` with ENU velocities at each depth cell.
    pub fn compute_profile(
        &self,
        beam_results: &[Vec<(f64, f64)>; JANUS_NUM_BEAMS],
        cell_depths_m: &[f64],
    ) -> VelocityProfile {
        let num_cells = cell_depths_m.len();
        let mut cells = Vec::with_capacity(num_cells);

        for i in 0..num_cells {
            // Collect beam velocities for this cell
            let mut beam_vels = [0.0f64; JANUS_NUM_BEAMS];
            let mut mean_corr = 0.0;
            let mut count = 0;

            for b in 0..JANUS_NUM_BEAMS {
                if let Some(&(vel, corr)) = beam_results[b].get(i) {
                    beam_vels[b] = vel;
                    mean_corr += corr;
                    count += 1;
                }
            }

            if count > 0 {
                mean_corr /= count as f64;
            }

            cells.push(self.transform_cell(&beam_vels, cell_depths_m[i], mean_corr));
        }

        VelocityProfile { cells }
    }
}

impl VelocityProfile {
    /// Depth-averaged velocity (east, north) over the profile.
    pub fn depth_averaged_velocity(&self) -> (f64, f64) {
        if self.cells.is_empty() {
            return (0.0, 0.0);
        }
        let n = self.cells.len() as f64;
        let sum_e: f64 = self.cells.iter().map(|c| c.east_mps).sum();
        let sum_n: f64 = self.cells.iter().map(|c| c.north_mps).sum();
        (sum_e / n, sum_n / n)
    }

    /// Maximum current speed in the profile.
    pub fn max_speed(&self) -> f64 {
        self.cells
            .iter()
            .map(|c| (c.east_mps * c.east_mps + c.north_mps * c.north_mps).sqrt())
            .fold(0.0f64, f64::max)
    }

    /// Maximum depth with valid data.
    pub fn max_depth(&self) -> f64 {
        self.cells
            .iter()
            .map(|c| c.depth_m)
            .fold(0.0f64, f64::max)
    }
}

// ---------------------------------------------------------------------------
// BottomTracker
// ---------------------------------------------------------------------------

/// Bottom detection and ship speed estimation from ADCP bottom track data.
///
/// Bottom tracking uses a separate acoustic mode (wider bandwidth, higher power)
/// to detect the seabed echo. The round-trip travel time gives the depth, and the
/// Doppler shift of the bottom return gives the vessel's velocity over ground.
#[derive(Debug, Clone)]
pub struct BottomTracker {
    /// Transmit frequency (Hz).
    pub transmit_freq_hz: f64,
    /// Beam angle from vertical (degrees).
    pub beam_angle_deg: f64,
    /// Sound speed (m/s).
    pub sound_speed_mps: f64,
    /// Minimum signal-to-noise ratio (linear, not dB) to accept a bottom detection.
    pub min_snr_linear: f64,
}

/// Result of a bottom track measurement.
#[derive(Debug, Clone, Copy)]
pub struct BottomTrackResult {
    /// Depth to bottom below transducer (m).
    pub depth_m: f64,
    /// Vessel velocity east (m/s).
    pub velocity_east_mps: f64,
    /// Vessel velocity north (m/s).
    pub velocity_north_mps: f64,
    /// Vessel velocity up (m/s).
    pub velocity_up_mps: f64,
    /// Whether the bottom detection is valid.
    pub valid: bool,
}

impl BottomTracker {
    /// Create a new bottom tracker.
    pub fn new(
        transmit_freq_hz: f64,
        beam_angle_deg: f64,
        sound_speed_mps: f64,
        min_snr_linear: f64,
    ) -> Self {
        Self {
            transmit_freq_hz,
            beam_angle_deg,
            sound_speed_mps,
            min_snr_linear,
        }
    }

    /// Create a bottom tracker from an ADCP config.
    pub fn from_config(config: &AdcpConfig, sound_speed_mps: f64) -> Self {
        Self::new(
            config.transmit_freq_hz,
            config.beam_angle_deg,
            sound_speed_mps,
            3.0, // ~4.8 dB default minimum SNR
        )
    }

    /// Detect the bottom from a single beam return using peak detection.
    ///
    /// Finds the strongest echo beyond the profiling range (or at least beyond
    /// `min_range_m`) and estimates depth from the two-way travel time.
    ///
    /// # Arguments
    ///
    /// * `beam_power` - Power envelope of the beam return (linear scale)
    /// * `sample_rate_hz` - Sample rate of the return
    /// * `min_range_m` - Minimum slant range to search (metres)
    ///
    /// # Returns
    ///
    /// `Some((slant_range_m, snr))` if a peak is found, `None` otherwise.
    pub fn detect_bottom_peak(
        &self,
        beam_power: &[f64],
        sample_rate_hz: f64,
        min_range_m: f64,
    ) -> Option<(f64, f64)> {
        if beam_power.is_empty() {
            return None;
        }

        let twtt_per_sample = 1.0 / sample_rate_hz;
        let range_per_sample = self.sound_speed_mps * twtt_per_sample / 2.0;
        let min_sample = (min_range_m / range_per_sample).ceil() as usize;

        if min_sample >= beam_power.len() {
            return None;
        }

        // Compute noise floor from first few samples (before blanking usually)
        let noise_samples = (min_sample / 2).max(1).min(beam_power.len());
        let noise_floor: f64 =
            beam_power[..noise_samples].iter().sum::<f64>() / noise_samples as f64;
        let noise_floor = noise_floor.max(1e-30);

        // Find peak beyond min_range
        let mut max_power = 0.0f64;
        let mut max_idx = min_sample;
        for (i, &p) in beam_power.iter().enumerate().skip(min_sample) {
            if p > max_power {
                max_power = p;
                max_idx = i;
            }
        }

        let snr = max_power / noise_floor;
        if snr < self.min_snr_linear {
            return None;
        }

        let slant_range = max_idx as f64 * range_per_sample;
        Some((slant_range, snr))
    }

    /// Estimate depth and vessel velocity from 4-beam bottom track data.
    ///
    /// # Arguments
    ///
    /// * `beam_ranges_m` - Slant range to bottom for each of the 4 beams
    /// * `beam_doppler_hz` - Doppler shift of bottom echo for each beam
    /// * `heading_deg` - Instrument heading (degrees from north)
    ///
    /// # Returns
    ///
    /// `BottomTrackResult` with depth and vessel velocity in ENU coordinates.
    pub fn compute_bottom_track(
        &self,
        beam_ranges_m: &[f64; JANUS_NUM_BEAMS],
        beam_doppler_hz: &[f64; JANUS_NUM_BEAMS],
        heading_deg: f64,
    ) -> BottomTrackResult {
        let theta_rad = self.beam_angle_deg * PI / 180.0;
        let cos_theta = theta_rad.cos();

        // Depth = mean of vertical projections
        let depth: f64 = beam_ranges_m.iter().sum::<f64>() / JANUS_NUM_BEAMS as f64 * cos_theta;

        // Convert Doppler to beam velocities
        let mut beam_vels = [0.0f64; JANUS_NUM_BEAMS];
        for i in 0..JANUS_NUM_BEAMS {
            beam_vels[i] = doppler_to_velocity(
                beam_doppler_hz[i],
                self.transmit_freq_hz,
                self.sound_speed_mps,
            );
        }

        // Transform to instrument ENU
        let (inst_e, inst_n, up, _error) =
            beam_to_enu_transform(&beam_vels, self.beam_angle_deg);

        // Rotate to true ENU
        let heading_rad = heading_deg * PI / 180.0;
        let cos_h = heading_rad.cos();
        let sin_h = heading_rad.sin();

        // Bottom track velocity is opposite sign: positive Doppler = vessel
        // moving towards beam, so negate for vessel velocity
        let east = -(inst_e * cos_h - inst_n * sin_h);
        let north = -(inst_e * sin_h + inst_n * cos_h);

        BottomTrackResult {
            depth_m: depth,
            velocity_east_mps: east,
            velocity_north_mps: north,
            velocity_up_mps: -up,
            valid: depth > 0.0,
        }
    }
}

// ---------------------------------------------------------------------------
// QualityFilter
// ---------------------------------------------------------------------------

/// Quality screening criteria for ADCP velocity measurements.
///
/// Implements standard RDI-style quality filters including:
/// - Correlation magnitude threshold
/// - Error velocity threshold
/// - Percent-good minimum
/// - Maximum velocity magnitude
#[derive(Debug, Clone)]
pub struct QualityFilter {
    /// Minimum correlation magnitude (0..1). Typical: 0.64 (RDI default).
    pub min_correlation: f64,
    /// Maximum absolute error velocity (m/s). Typical: 0.15 -- 0.5 m/s.
    pub max_error_velocity_mps: f64,
    /// Minimum percent-good (fraction 0..1). Typical: 0.25.
    pub min_percent_good: f64,
    /// Maximum horizontal speed (m/s). Rejects obviously bad data.
    pub max_horizontal_speed_mps: f64,
    /// Maximum vertical speed (m/s).
    pub max_vertical_speed_mps: f64,
}

impl Default for QualityFilter {
    fn default() -> Self {
        Self {
            min_correlation: 0.64,
            max_error_velocity_mps: 0.5,
            min_percent_good: 0.25,
            max_horizontal_speed_mps: 5.0,
            max_vertical_speed_mps: 2.0,
        }
    }
}

/// Result of quality screening for a single depth cell.
#[derive(Debug, Clone, Copy)]
pub struct QualityResult {
    /// Whether the cell passes all quality criteria.
    pub pass: bool,
    /// Reason for failure, if any.
    pub fail_reason: QualityFailReason,
}

/// Reason a quality check failed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QualityFailReason {
    /// Cell passed all checks.
    None,
    /// Correlation below threshold.
    LowCorrelation,
    /// Error velocity exceeded threshold.
    HighErrorVelocity,
    /// Percent good below threshold.
    LowPercentGood,
    /// Horizontal speed exceeded maximum.
    ExcessiveHorizontalSpeed,
    /// Vertical speed exceeded maximum.
    ExcessiveVerticalSpeed,
}

impl QualityFilter {
    /// Create a new quality filter with default RDI-style thresholds.
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a strict quality filter for high-accuracy measurements.
    pub fn strict() -> Self {
        Self {
            min_correlation: 0.80,
            max_error_velocity_mps: 0.15,
            min_percent_good: 0.50,
            max_horizontal_speed_mps: 3.0,
            max_vertical_speed_mps: 1.0,
        }
    }

    /// Screen a single velocity cell.
    ///
    /// # Arguments
    ///
    /// * `cell` - Velocity measurement to screen
    /// * `percent_good` - Fraction of good pings (0..1) for this cell
    pub fn screen_cell(&self, cell: &VelocityCell, percent_good: f64) -> QualityResult {
        if cell.mean_correlation < self.min_correlation {
            return QualityResult {
                pass: false,
                fail_reason: QualityFailReason::LowCorrelation,
            };
        }

        if cell.error_mps.abs() > self.max_error_velocity_mps {
            return QualityResult {
                pass: false,
                fail_reason: QualityFailReason::HighErrorVelocity,
            };
        }

        if percent_good < self.min_percent_good {
            return QualityResult {
                pass: false,
                fail_reason: QualityFailReason::LowPercentGood,
            };
        }

        let horiz_speed =
            (cell.east_mps * cell.east_mps + cell.north_mps * cell.north_mps).sqrt();
        if horiz_speed > self.max_horizontal_speed_mps {
            return QualityResult {
                pass: false,
                fail_reason: QualityFailReason::ExcessiveHorizontalSpeed,
            };
        }

        if cell.up_mps.abs() > self.max_vertical_speed_mps {
            return QualityResult {
                pass: false,
                fail_reason: QualityFailReason::ExcessiveVerticalSpeed,
            };
        }

        QualityResult {
            pass: true,
            fail_reason: QualityFailReason::None,
        }
    }

    /// Screen an entire velocity profile, returning only cells that pass.
    ///
    /// # Arguments
    ///
    /// * `profile` - Velocity profile to screen
    /// * `percent_good` - Per-cell percent-good values
    ///
    /// # Returns
    ///
    /// Filtered profile containing only cells that pass quality screening.
    pub fn filter_profile(
        &self,
        profile: &VelocityProfile,
        percent_good: &[f64],
    ) -> VelocityProfile {
        let cells: Vec<VelocityCell> = profile
            .cells
            .iter()
            .enumerate()
            .filter(|(i, cell)| {
                let pg = percent_good.get(*i).copied().unwrap_or(0.0);
                self.screen_cell(cell, pg).pass
            })
            .map(|(_, cell)| *cell)
            .collect();

        VelocityProfile { cells }
    }

    /// Count the number of cells that pass quality screening.
    pub fn count_good_cells(
        &self,
        profile: &VelocityProfile,
        percent_good: &[f64],
    ) -> usize {
        profile
            .cells
            .iter()
            .enumerate()
            .filter(|(i, cell)| {
                let pg = percent_good.get(*i).copied().unwrap_or(0.0);
                self.screen_cell(cell, pg).pass
            })
            .count()
    }
}

// ---------------------------------------------------------------------------
// Ensemble averaging
// ---------------------------------------------------------------------------

/// Average multiple velocity profiles into a single ensemble profile.
///
/// Used for temporal averaging to reduce measurement noise. Profiles must
/// have the same number of cells and depth structure.
///
/// # Arguments
///
/// * `profiles` - Slice of velocity profiles to average
///
/// # Returns
///
/// Averaged profile, or `None` if input is empty or cell counts differ.
pub fn ensemble_average(profiles: &[VelocityProfile]) -> Option<VelocityProfile> {
    if profiles.is_empty() {
        return None;
    }

    let num_cells = profiles[0].cells.len();
    if profiles.iter().any(|p| p.cells.len() != num_cells) {
        return None;
    }

    let n = profiles.len() as f64;
    let mut cells = Vec::with_capacity(num_cells);

    for i in 0..num_cells {
        let sum_e: f64 = profiles.iter().map(|p| p.cells[i].east_mps).sum();
        let sum_n: f64 = profiles.iter().map(|p| p.cells[i].north_mps).sum();
        let sum_u: f64 = profiles.iter().map(|p| p.cells[i].up_mps).sum();
        let sum_err: f64 = profiles.iter().map(|p| p.cells[i].error_mps).sum();
        let sum_corr: f64 = profiles.iter().map(|p| p.cells[i].mean_correlation).sum();

        cells.push(VelocityCell {
            depth_m: profiles[0].cells[i].depth_m,
            east_mps: sum_e / n,
            north_mps: sum_n / n,
            up_mps: sum_u / n,
            error_mps: sum_err / n,
            mean_correlation: sum_corr / n,
        });
    }

    Some(VelocityProfile { cells })
}

/// Compute the standard deviation of velocity across an ensemble.
///
/// # Arguments
///
/// * `profiles` - Slice of velocity profiles
/// * `cell_index` - Which depth cell to analyse
///
/// # Returns
///
/// `(std_east, std_north, std_up)` in m/s, or `None` if insufficient data.
pub fn velocity_std_dev(profiles: &[VelocityProfile], cell_index: usize) -> Option<(f64, f64, f64)> {
    if profiles.len() < 2 {
        return None;
    }
    if profiles.iter().any(|p| cell_index >= p.cells.len()) {
        return None;
    }

    let n = profiles.len() as f64;

    let mean_e: f64 = profiles.iter().map(|p| p.cells[cell_index].east_mps).sum::<f64>() / n;
    let mean_n: f64 = profiles.iter().map(|p| p.cells[cell_index].north_mps).sum::<f64>() / n;
    let mean_u: f64 = profiles.iter().map(|p| p.cells[cell_index].up_mps).sum::<f64>() / n;

    let var_e: f64 = profiles
        .iter()
        .map(|p| (p.cells[cell_index].east_mps - mean_e).powi(2))
        .sum::<f64>()
        / (n - 1.0);
    let var_n: f64 = profiles
        .iter()
        .map(|p| (p.cells[cell_index].north_mps - mean_n).powi(2))
        .sum::<f64>()
        / (n - 1.0);
    let var_u: f64 = profiles
        .iter()
        .map(|p| (p.cells[cell_index].up_mps - mean_u).powi(2))
        .sum::<f64>()
        / (n - 1.0);

    Some((var_e.sqrt(), var_n.sqrt(), var_u.sqrt()))
}

/// Compute acoustic absorption coefficient in seawater (dB/m) using the
/// Francois-Garrison equation (simplified).
///
/// # Arguments
///
/// * `freq_khz` - Acoustic frequency in kHz
/// * `temperature_c` - Temperature in degrees Celsius
/// * `salinity_psu` - Salinity in PSU
/// * `depth_m` - Depth in metres
///
/// # Returns
///
/// Absorption coefficient in dB/m.
pub fn absorption_coefficient(
    freq_khz: f64,
    temperature_c: f64,
    salinity_psu: f64,
    depth_m: f64,
) -> f64 {
    // Simplified Francois-Garrison (1982) / Ainslie-McColm (1998)
    let t = temperature_c;
    let s = salinity_psu;
    let _d = depth_m;
    let f = freq_khz;

    // Boric acid relaxation frequency (kHz)
    let f1 = 0.78 * (s / 35.0).sqrt() * (t / 26.0).exp();
    // Magnesium sulphate relaxation frequency (kHz)
    let f2 = 42.0 * (t / 17.0).exp();

    // Boric acid contribution
    let a1 = 0.106 * f1 * f * f / (f1 * f1 + f * f);
    // MgSO4 contribution
    let a2 = 0.52 * (1.0 + t / 43.0) * (s / 35.0) * f2 * f * f / (f2 * f2 + f * f);
    // Pure water viscous absorption
    let a3 = 0.00049 * f * f * (-0.04 * (t - 10.0) + 0.003 * depth_m / 1000.0).exp();

    // Total absorption in dB/km, convert to dB/m
    (a1 + a2 + a3) / 1000.0
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // -- sound speed tests --

    #[test]
    fn test_sound_speed_typical_ocean() {
        // Typical ocean conditions: 15 C, 35 PSU, 100 m depth
        let c = sound_speed_seawater(15.0, 35.0, 100.0);
        // Expected ~1507 m/s from Mackenzie equation
        assert!(
            (c - 1507.0).abs() < 5.0,
            "Expected ~1507 m/s, got {c}"
        );
    }

    #[test]
    fn test_sound_speed_cold_fresh() {
        // Cold fresh water: 0 C, 0 PSU, 0 m
        let c = sound_speed_seawater(0.0, 0.0, 0.0);
        // Fresh water at 0C: ~1402 m/s
        assert!(
            c > 1380.0 && c < 1420.0,
            "Cold fresh water speed {c} out of range"
        );
    }

    #[test]
    fn test_sound_speed_warm_saline() {
        // Warm tropical: 30 C, 35 PSU, 0 m
        let c = sound_speed_seawater(30.0, 35.0, 0.0);
        // Expected ~1543 m/s
        assert!(
            c > 1530.0 && c < 1560.0,
            "Warm saline speed {c} out of range"
        );
    }

    #[test]
    fn test_sound_speed_increases_with_depth() {
        let c_shallow = sound_speed_seawater(10.0, 35.0, 0.0);
        let c_deep = sound_speed_seawater(10.0, 35.0, 1000.0);
        assert!(c_deep > c_shallow, "Sound speed should increase with depth");
    }

    // -- Doppler conversion tests --

    #[test]
    fn test_doppler_to_velocity_positive() {
        // 300 kHz transducer, 1500 m/s sound speed, 10 Hz shift
        let v = doppler_to_velocity(10.0, 300_000.0, 1500.0);
        // v = 10 * 1500 / (2 * 300000) = 0.025 m/s
        assert!((v - 0.025).abs() < EPSILON);
    }

    #[test]
    fn test_velocity_to_doppler_roundtrip() {
        let v_original = 1.5; // m/s
        let f0 = 300_000.0;
        let c = 1500.0;
        let fd = velocity_to_doppler(v_original, f0, c);
        let v_back = doppler_to_velocity(fd, f0, c);
        assert!(
            (v_back - v_original).abs() < EPSILON,
            "Roundtrip failed: {v_original} -> {fd} Hz -> {v_back}"
        );
    }

    #[test]
    fn test_doppler_zero() {
        let v = doppler_to_velocity(0.0, 300_000.0, 1500.0);
        assert!(v.abs() < EPSILON);
    }

    // -- beam_to_enu_transform tests --

    #[test]
    fn test_enu_pure_east_flow() {
        // Pure eastward flow: beam 1 (east-up) sees approach, beam 2 sees recede
        let theta = 20.0;
        let v_east = 1.0; // m/s
        let v_beam = v_east * (theta * PI / 180.0).sin();
        let beams = [v_beam, -v_beam, 0.0, 0.0];
        let (e, n, u, err) = beam_to_enu_transform(&beams, theta);
        assert!((e - v_east).abs() < 1e-10, "East: expected {v_east}, got {e}");
        assert!(n.abs() < 1e-10, "North should be ~0, got {n}");
        assert!(u.abs() < 1e-10, "Up should be ~0, got {u}");
        assert!(err.abs() < 1e-10, "Error should be ~0, got {err}");
    }

    #[test]
    fn test_enu_pure_north_flow() {
        let theta = 20.0;
        let v_north = 0.8;
        let v_beam = v_north * (theta * PI / 180.0).sin();
        let beams = [0.0, 0.0, -v_beam, v_beam];
        let (e, n, _u, err) = beam_to_enu_transform(&beams, theta);
        assert!(e.abs() < 1e-10);
        assert!((n - v_north).abs() < 1e-10, "North: expected {v_north}, got {n}");
        assert!(err.abs() < 1e-10);
    }

    #[test]
    fn test_enu_pure_vertical_flow() {
        let theta = 20.0;
        let v_up = 0.1;
        let cos_theta = (theta * PI / 180.0).cos();
        // All beams see the same along-beam component from vertical flow
        let v_beam = v_up * cos_theta;
        let beams = [v_beam, v_beam, v_beam, v_beam];
        let (_e, _n, u, _err) = beam_to_enu_transform(&beams, theta);
        assert!(
            (u - v_up).abs() < 1e-10,
            "Up: expected {v_up}, got {u}"
        );
    }

    #[test]
    fn test_enu_error_velocity_nonzero() {
        // Inhomogeneous flow: beams 1&2 see different magnitude than 3&4
        let beams = [0.5, -0.3, -0.1, 0.2];
        let (_, _, _, err) = beam_to_enu_transform(&beams, 20.0);
        assert!(err.abs() > 0.01, "Error velocity should be nonzero for inhomogeneous flow");
    }

    // -- AdcpConfig tests --

    #[test]
    fn test_config_300khz_defaults() {
        let config = AdcpConfig::default_300khz();
        assert_eq!(config.num_beams, 4);
        assert!((config.beam_angle_deg - 20.0).abs() < EPSILON);
        assert!((config.transmit_freq_hz - 300_000.0).abs() < EPSILON);
        assert!(config.num_cells == 25);
    }

    #[test]
    fn test_config_max_range() {
        let config = AdcpConfig::default_300khz();
        let max_range = config.max_range_m();
        // 1.76 + 25 * 4 = 101.76 m
        assert!(
            (max_range - 101.76).abs() < 0.01,
            "Max range: expected ~101.76, got {max_range}"
        );
    }

    #[test]
    fn test_config_max_depth_less_than_range() {
        let config = AdcpConfig::default_300khz();
        assert!(
            config.max_depth_m() < config.max_range_m(),
            "Vertical depth should be less than slant range"
        );
    }

    #[test]
    fn test_config_1200khz_shallow() {
        let config = AdcpConfig::default_1200khz();
        assert!(config.cell_size_m < 1.0, "1200 kHz cells should be small");
        assert!(config.max_range_m() < 30.0, "1200 kHz range should be short");
    }

    // -- DepthCellBinner tests --

    #[test]
    fn test_binner_cell_count() {
        let config = AdcpConfig::default_300khz();
        let binner = DepthCellBinner::new(&config, 1500.0);
        assert_eq!(binner.num_cells(), config.num_cells);
    }

    #[test]
    fn test_binner_cell_centres_increasing() {
        let config = AdcpConfig::default_300khz();
        let binner = DepthCellBinner::new(&config, 1500.0);
        for i in 1..binner.num_cells() {
            assert!(
                binner.cell_centres_m[i] > binner.cell_centres_m[i - 1],
                "Cell centres should be monotonically increasing"
            );
        }
    }

    #[test]
    fn test_binner_extract_cell() {
        let config = AdcpConfig::default_300khz();
        let binner = DepthCellBinner::new(&config, 1500.0);
        let total_samples = binner.required_samples();
        let beam_return: Vec<f64> = (0..total_samples).map(|i| i as f64).collect();

        let cell0 = binner.extract_cell(&beam_return, 0);
        assert!(cell0.is_some());
        assert_eq!(cell0.unwrap().len(), binner.cell_num_samples);

        // Last cell should also work
        let last = binner.extract_cell(&beam_return, config.num_cells - 1);
        assert!(last.is_some());
    }

    #[test]
    fn test_binner_extract_out_of_range() {
        let config = AdcpConfig::default_300khz();
        let binner = DepthCellBinner::new(&config, 1500.0);
        let result = binner.extract_cell(&[0.0; 10], 0); // too short
        assert!(result.is_none());
    }

    // -- DopplerVelocityProcessor tests --

    #[test]
    fn test_doppler_processor_zero_velocity() {
        let proc = DopplerVelocityProcessor::new(300_000.0, 1500.0, 1.0);
        // All pings have the same phase -> zero Doppler
        let pings: Vec<Complex> = (0..10).map(|_| (1.0, 0.0)).collect();
        let (vel, corr) = proc.estimate_velocity(&pings);
        assert!(vel.abs() < 1e-10, "Expected zero velocity, got {vel}");
        assert!(corr > 0.99, "Expected high correlation, got {corr}");
    }

    #[test]
    fn test_doppler_processor_known_velocity() {
        let f0 = 300_000.0;
        let c = 1500.0;
        // Use a very short ping interval so that v_max = c/(4*f0*T_lag) >> v_true.
        // T_lag = 50 us -> v_max = 1500/(4*300000*50e-6) = 25 m/s
        let t_lag = 50.0e-6;
        let proc = DopplerVelocityProcessor::new(f0, c, t_lag);

        // Simulate a known velocity by applying a phase ramp
        let v_true = 0.5; // m/s radial, well below v_max
        // Phase per ping: phi = 4*pi*f0*v*T_lag / c
        let phase_per_ping = 4.0 * PI * f0 * v_true * t_lag / c;

        let num_pings = 20;
        let pings: Vec<Complex> = (0..num_pings)
            .map(|i| {
                let phase = i as f64 * phase_per_ping;
                (phase.cos(), phase.sin())
            })
            .collect();

        let (vel, corr) = proc.estimate_velocity(&pings);
        assert!(
            (vel - v_true).abs() < 0.01,
            "Expected v={v_true}, got {vel}"
        );
        assert!(corr > 0.99, "Expected high correlation, got {corr}");
    }

    #[test]
    fn test_doppler_processor_single_ping() {
        let proc = DopplerVelocityProcessor::new(300_000.0, 1500.0, 1.0);
        let (vel, corr) = proc.estimate_velocity(&[(1.0, 0.0)]);
        assert!(vel.abs() < EPSILON);
        assert!(corr.abs() < EPSILON);
    }

    #[test]
    fn test_doppler_processor_process_beam() {
        let proc = DopplerVelocityProcessor::new(300_000.0, 1500.0, 1.0);
        let num_cells = 5;
        let num_pings = 8;
        let ping_data: Vec<Vec<Complex>> = (0..num_pings)
            .map(|_| vec![(1.0, 0.0); num_cells])
            .collect();

        let results = proc.process_beam(&ping_data);
        assert_eq!(results.len(), num_cells);
        for (vel, _corr) in &results {
            assert!(vel.abs() < 1e-10);
        }
    }

    // -- VelocityProfiler tests --

    #[test]
    fn test_profiler_uniform_east_flow() {
        let profiler = VelocityProfiler::level(20.0);
        let num_cells = 5;
        let theta_rad = 20.0 * PI / 180.0;
        let v_east = 1.0;
        let v_beam = v_east * theta_rad.sin();

        let beam1: Vec<(f64, f64)> = vec![(v_beam, 0.95); num_cells];
        let beam2: Vec<(f64, f64)> = vec![(-v_beam, 0.95); num_cells];
        let beam3: Vec<(f64, f64)> = vec![(0.0, 0.95); num_cells];
        let beam4: Vec<(f64, f64)> = vec![(0.0, 0.95); num_cells];

        let depths: Vec<f64> = (0..num_cells).map(|i| 5.0 + i as f64 * 4.0).collect();
        let profile = profiler.compute_profile(&[beam1, beam2, beam3, beam4], &depths);

        assert_eq!(profile.cells.len(), num_cells);
        for cell in &profile.cells {
            assert!(
                (cell.east_mps - v_east).abs() < 1e-8,
                "East: expected {v_east}, got {}",
                cell.east_mps
            );
            assert!(cell.north_mps.abs() < 1e-8);
        }
    }

    #[test]
    fn test_profiler_heading_rotation() {
        // Heading 90 degrees: instrument east -> true north
        let profiler = VelocityProfiler::new(20.0, 90.0, 0.0, 0.0);
        let theta_rad = 20.0 * PI / 180.0;
        let v_inst_east = 1.0;
        let v_beam = v_inst_east * theta_rad.sin();

        let beam1: Vec<(f64, f64)> = vec![(v_beam, 0.9)];
        let beam2: Vec<(f64, f64)> = vec![(-v_beam, 0.9)];
        let beam3: Vec<(f64, f64)> = vec![(0.0, 0.9)];
        let beam4: Vec<(f64, f64)> = vec![(0.0, 0.9)];

        let profile = profiler.compute_profile(&[beam1, beam2, beam3, beam4], &[10.0]);
        let cell = &profile.cells[0];
        // With 90-degree heading, instrument east becomes true north
        assert!(cell.east_mps.abs() < 1e-8, "East should be ~0 with 90 deg heading");
        assert!(
            (cell.north_mps - v_inst_east).abs() < 1e-8,
            "North should be {v_inst_east}, got {}",
            cell.north_mps
        );
    }

    #[test]
    fn test_profile_depth_averaged() {
        let profile = VelocityProfile {
            cells: vec![
                VelocityCell {
                    depth_m: 5.0,
                    east_mps: 0.5,
                    north_mps: 0.2,
                    up_mps: 0.0,
                    error_mps: 0.0,
                    mean_correlation: 0.9,
                },
                VelocityCell {
                    depth_m: 9.0,
                    east_mps: 1.5,
                    north_mps: 0.8,
                    up_mps: 0.0,
                    error_mps: 0.0,
                    mean_correlation: 0.9,
                },
            ],
        };
        let (avg_e, avg_n) = profile.depth_averaged_velocity();
        assert!((avg_e - 1.0).abs() < EPSILON);
        assert!((avg_n - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_profile_max_speed() {
        let profile = VelocityProfile {
            cells: vec![
                VelocityCell {
                    depth_m: 5.0,
                    east_mps: 0.3,
                    north_mps: 0.4,
                    up_mps: 0.0,
                    error_mps: 0.0,
                    mean_correlation: 0.9,
                },
                VelocityCell {
                    depth_m: 9.0,
                    east_mps: 0.6,
                    north_mps: 0.8,
                    up_mps: 0.0,
                    error_mps: 0.0,
                    mean_correlation: 0.9,
                },
            ],
        };
        let max_spd = profile.max_speed();
        assert!((max_spd - 1.0).abs() < EPSILON, "Expected 1.0, got {max_spd}");
    }

    // -- BottomTracker tests --

    #[test]
    fn test_bottom_tracker_detect_peak() {
        let bt = BottomTracker::new(300_000.0, 20.0, 1500.0, 3.0);
        let sr = 50_000.0;
        let range_per_sample = 1500.0 / (2.0 * sr);

        // Create a return with a strong bottom echo at ~50 m slant range
        let target_range_m = 50.0;
        let target_sample = (target_range_m / range_per_sample) as usize;
        let mut power = vec![0.01; target_sample + 100];
        power[target_sample] = 10.0; // strong bottom echo

        let result = bt.detect_bottom_peak(&power, sr, 5.0);
        assert!(result.is_some());
        let (range, snr) = result.unwrap();
        assert!(
            (range - target_range_m).abs() < range_per_sample * 2.0,
            "Range: expected ~{target_range_m}, got {range}"
        );
        assert!(snr > 3.0);
    }

    #[test]
    fn test_bottom_tracker_no_bottom() {
        let bt = BottomTracker::new(300_000.0, 20.0, 1500.0, 10.0);
        // Weak noisy return, no clear bottom
        let power = vec![0.01; 1000];
        let result = bt.detect_bottom_peak(&power, 50_000.0, 5.0);
        // SNR of uniform noise is 1.0, below threshold 10.0
        assert!(result.is_none());
    }

    #[test]
    fn test_bottom_track_velocity() {
        let bt = BottomTracker::new(300_000.0, 20.0, 1500.0, 3.0);
        // Stationary vessel: all Doppler shifts are zero
        let ranges = [100.0, 100.0, 100.0, 100.0];
        let doppler = [0.0, 0.0, 0.0, 0.0];
        let result = bt.compute_bottom_track(&ranges, &doppler, 0.0);
        assert!(result.valid);
        assert!(result.velocity_east_mps.abs() < EPSILON);
        assert!(result.velocity_north_mps.abs() < EPSILON);
    }

    // -- QualityFilter tests --

    #[test]
    fn test_quality_filter_pass() {
        let qf = QualityFilter::default();
        let cell = VelocityCell {
            depth_m: 10.0,
            east_mps: 0.5,
            north_mps: 0.3,
            up_mps: 0.01,
            error_mps: 0.05,
            mean_correlation: 0.85,
        };
        let result = qf.screen_cell(&cell, 0.80);
        assert!(result.pass);
        assert_eq!(result.fail_reason, QualityFailReason::None);
    }

    #[test]
    fn test_quality_filter_low_correlation() {
        let qf = QualityFilter::default();
        let cell = VelocityCell {
            depth_m: 10.0,
            east_mps: 0.5,
            north_mps: 0.3,
            up_mps: 0.01,
            error_mps: 0.05,
            mean_correlation: 0.30, // below 0.64 threshold
        };
        let result = qf.screen_cell(&cell, 0.80);
        assert!(!result.pass);
        assert_eq!(result.fail_reason, QualityFailReason::LowCorrelation);
    }

    #[test]
    fn test_quality_filter_high_error_velocity() {
        let qf = QualityFilter::default();
        let cell = VelocityCell {
            depth_m: 10.0,
            east_mps: 0.5,
            north_mps: 0.3,
            up_mps: 0.01,
            error_mps: 1.5, // exceeds 0.5 threshold
            mean_correlation: 0.85,
        };
        let result = qf.screen_cell(&cell, 0.80);
        assert!(!result.pass);
        assert_eq!(result.fail_reason, QualityFailReason::HighErrorVelocity);
    }

    #[test]
    fn test_quality_filter_profile() {
        let qf = QualityFilter::default();
        let profile = VelocityProfile {
            cells: vec![
                VelocityCell {
                    depth_m: 5.0,
                    east_mps: 0.5,
                    north_mps: 0.3,
                    up_mps: 0.01,
                    error_mps: 0.05,
                    mean_correlation: 0.90, // good
                },
                VelocityCell {
                    depth_m: 9.0,
                    east_mps: 0.5,
                    north_mps: 0.3,
                    up_mps: 0.01,
                    error_mps: 0.05,
                    mean_correlation: 0.20, // bad
                },
                VelocityCell {
                    depth_m: 13.0,
                    east_mps: 0.5,
                    north_mps: 0.3,
                    up_mps: 0.01,
                    error_mps: 0.05,
                    mean_correlation: 0.80, // good
                },
            ],
        };
        let pg = vec![0.80, 0.80, 0.80];
        let filtered = qf.filter_profile(&profile, &pg);
        assert_eq!(filtered.cells.len(), 2); // only 2 pass
    }

    // -- ensemble averaging tests --

    #[test]
    fn test_ensemble_average() {
        let p1 = VelocityProfile {
            cells: vec![VelocityCell {
                depth_m: 5.0,
                east_mps: 1.0,
                north_mps: 0.0,
                up_mps: 0.0,
                error_mps: 0.0,
                mean_correlation: 0.9,
            }],
        };
        let p2 = VelocityProfile {
            cells: vec![VelocityCell {
                depth_m: 5.0,
                east_mps: 2.0,
                north_mps: 0.0,
                up_mps: 0.0,
                error_mps: 0.0,
                mean_correlation: 0.8,
            }],
        };
        let avg = ensemble_average(&[p1, p2]).unwrap();
        assert_eq!(avg.cells.len(), 1);
        assert!((avg.cells[0].east_mps - 1.5).abs() < EPSILON);
        assert!((avg.cells[0].mean_correlation - 0.85).abs() < EPSILON);
    }

    #[test]
    fn test_ensemble_average_empty() {
        let result = ensemble_average(&[]);
        assert!(result.is_none());
    }

    #[test]
    fn test_velocity_std_dev() {
        let make_profile = |e: f64| VelocityProfile {
            cells: vec![VelocityCell {
                depth_m: 5.0,
                east_mps: e,
                north_mps: 0.0,
                up_mps: 0.0,
                error_mps: 0.0,
                mean_correlation: 0.9,
            }],
        };
        let profiles = vec![make_profile(1.0), make_profile(3.0)];
        let (std_e, std_n, std_u) = velocity_std_dev(&profiles, 0).unwrap();
        // std of [1, 3] = sqrt(2) ~= 1.4142
        assert!((std_e - std::f64::consts::SQRT_2).abs() < 0.01);
        assert!(std_n.abs() < EPSILON);
        assert!(std_u.abs() < EPSILON);
    }

    // -- absorption coefficient test --

    #[test]
    fn test_absorption_positive() {
        let alpha = absorption_coefficient(300.0, 15.0, 35.0, 100.0);
        assert!(alpha > 0.0, "Absorption must be positive, got {alpha}");
        // At 300 kHz, typical ~0.05-0.1 dB/m
        assert!(alpha < 1.0, "Absorption unexpectedly high: {alpha}");
    }

    #[test]
    fn test_absorption_increases_with_frequency() {
        let a_low = absorption_coefficient(75.0, 15.0, 35.0, 100.0);
        let a_high = absorption_coefficient(1200.0, 15.0, 35.0, 100.0);
        assert!(
            a_high > a_low,
            "Absorption should increase with frequency: {a_low} vs {a_high}"
        );
    }
}
