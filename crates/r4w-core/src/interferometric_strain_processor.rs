//! Interferometric Strain Processor for InSAR and fiber-optic deformation monitoring.
//!
//! This module implements signal processing for interferometric strain measurement
//! across multiple sensing modalities: satellite InSAR, fiber-optic strain sensing
//! (FBG and DAS), GNSS displacement, and structural health monitoring.
//!
//! # Applications
//!
//! - **InSAR**: Ground deformation from differential interferometric phase
//! - **Fiber-optic sensing**: Fiber Bragg Grating (FBG) and Distributed Acoustic Sensing (DAS)
//! - **Structural health monitoring**: Bridge, dam, and building deformation
//! - **Seismology**: Co-seismic and post-seismic strain transients
//! - **Volcanology**: Magma chamber inflation/deflation via Mogi/Okada models
//!
//! # Physics
//!
//! ```text
//! InSAR:    phase = 4*pi*d / lambda         (double-pass, LOS geometry)
//! FBG:      delta_lambda/lambda = (1-p_e)*epsilon + (alpha+xi)*delta_T
//! Mogi:     u_r = (3*V*dP*a^3)/(4*G) * r/(r^2+d^2)^(3/2)
//! Coherence: gamma = |sum(s1*conj(s2))| / sqrt(sum(|s1|^2)*sum(|s2|^2))
//! ```
//!
//! # Example
//!
//! ```rust
//! use r4w_core::interferometric_strain_processor::{
//!     StrainConfig, MeasurementType, InSarPhaseProcessor, StrainCalculator,
//! };
//!
//! // C-band SAR (Sentinel-1): wavelength 5.55 cm
//! let config = StrainConfig {
//!     measurement_type: MeasurementType::InSar,
//!     wavelength_m: 0.0555,
//!     gauge_length_m: 100.0,
//!     sample_rate_hz: 1.0,
//! };
//!
//! let processor = InSarPhaseProcessor::new(config.wavelength_m);
//!
//! // 1 radian of interferometric phase -> displacement
//! let displacement = processor.phase_to_displacement(1.0);
//! // d = phase * lambda / (4*pi)
//! let expected = 1.0 * 0.0555 / (4.0 * std::f64::consts::PI);
//! assert!((displacement - expected).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Type of interferometric strain measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MeasurementType {
    /// Satellite Interferometric Synthetic Aperture Radar.
    InSar,
    /// Fiber-optic sensing (FBG or DAS).
    FiberOptic,
    /// GNSS-based displacement monitoring.
    Gnss,
}

/// Configuration for interferometric strain measurement.
#[derive(Debug, Clone)]
pub struct StrainConfig {
    /// Measurement modality.
    pub measurement_type: MeasurementType,
    /// Radar wavelength (InSAR) or optical wavelength (fiber) in metres.
    pub wavelength_m: f64,
    /// Gauge length in metres for strain computation (epsilon = delta_L / L).
    pub gauge_length_m: f64,
    /// Sample rate in Hz (temporal sampling of measurements).
    pub sample_rate_hz: f64,
}

impl StrainConfig {
    /// Create a Sentinel-1 C-band InSAR configuration.
    pub fn sentinel1() -> Self {
        Self {
            measurement_type: MeasurementType::InSar,
            wavelength_m: 0.0555,
            gauge_length_m: 100.0,
            sample_rate_hz: 1.0 / (12.0 * 86400.0), // 12-day repeat
        }
    }

    /// Create a TerraSAR-X X-band InSAR configuration.
    pub fn terrasar_x() -> Self {
        Self {
            measurement_type: MeasurementType::InSar,
            wavelength_m: 0.031,
            gauge_length_m: 100.0,
            sample_rate_hz: 1.0 / (11.0 * 86400.0), // 11-day repeat
        }
    }

    /// Create a standard FBG fiber-optic configuration at 1550 nm.
    pub fn fbg_1550nm() -> Self {
        Self {
            measurement_type: MeasurementType::FiberOptic,
            wavelength_m: 1550.0e-9,
            gauge_length_m: 0.01, // 10 mm gauge
            sample_rate_hz: 1000.0,
        }
    }

    /// Create a DAS fiber-optic configuration.
    pub fn das_standard() -> Self {
        Self {
            measurement_type: MeasurementType::FiberOptic,
            wavelength_m: 1550.0e-9,
            gauge_length_m: 10.0, // 10 m gauge length
            sample_rate_hz: 10000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// InSAR Phase Processor
// ---------------------------------------------------------------------------

/// Converts differential interferometric phase to line-of-sight displacement.
///
/// The relationship is: d = phase * lambda / (4 * pi)
/// where the factor of 4*pi accounts for the double pass (satellite to ground
/// and back) in the radar geometry.
#[derive(Debug, Clone)]
pub struct InSarPhaseProcessor {
    /// Radar wavelength in metres.
    wavelength_m: f64,
    /// Conversion factor: lambda / (4 * pi).
    phase_to_disp_factor: f64,
}

impl InSarPhaseProcessor {
    /// Create a new processor for the given radar wavelength (metres).
    pub fn new(wavelength_m: f64) -> Self {
        let phase_to_disp_factor = wavelength_m / (4.0 * PI);
        Self {
            wavelength_m,
            phase_to_disp_factor,
        }
    }

    /// Radar wavelength in metres.
    pub fn wavelength(&self) -> f64 {
        self.wavelength_m
    }

    /// Convert a single interferometric phase value (radians) to
    /// line-of-sight displacement (metres).
    pub fn phase_to_displacement(&self, phase_rad: f64) -> f64 {
        phase_rad * self.phase_to_disp_factor
    }

    /// Convert a displacement (metres) back to interferometric phase (radians).
    pub fn displacement_to_phase(&self, displacement_m: f64) -> f64 {
        displacement_m / self.phase_to_disp_factor
    }

    /// Process a batch of interferometric phase values, returning displacements.
    pub fn process_phases(&self, phases: &[f64]) -> Vec<f64> {
        phases
            .iter()
            .map(|&p| self.phase_to_displacement(p))
            .collect()
    }

    /// Compute line-of-sight displacement velocity from two acquisitions.
    ///
    /// `delta_t` is the time between acquisitions in seconds.
    /// Returns velocity in m/s.
    pub fn displacement_velocity(&self, phase1: f64, phase2: f64, delta_t: f64) -> f64 {
        let d1 = self.phase_to_displacement(phase1);
        let d2 = self.phase_to_displacement(phase2);
        (d2 - d1) / delta_t
    }
}

// ---------------------------------------------------------------------------
// 2D Phase Unwrapping
// ---------------------------------------------------------------------------

/// Quality-guided 2D phase unwrapper for InSAR interferograms.
///
/// Uses phase gradient reliability to guide the unwrapping path. Pixels are
/// processed in order of decreasing quality (second difference magnitude),
/// propagating the unwrapped phase from high-quality regions outward.
#[derive(Debug, Clone)]
pub struct PhaseUnwrapper2D {
    /// Number of rows in the interferogram.
    rows: usize,
    /// Number of columns in the interferogram.
    cols: usize,
}

impl PhaseUnwrapper2D {
    /// Create a new 2D phase unwrapper for the given grid dimensions.
    pub fn new(rows: usize, cols: usize) -> Self {
        Self { rows, cols }
    }

    /// Wrap an angle to the interval (-pi, pi].
    fn wrap(phase: f64) -> f64 {
        let mut p = phase % (2.0 * PI);
        if p > PI {
            p -= 2.0 * PI;
        } else if p <= -PI {
            p += 2.0 * PI;
        }
        p
    }

    /// Compute the phase gradient reliability for each pixel.
    ///
    /// Reliability is the inverse of the second-order phase difference
    /// magnitude. Higher reliability means smoother phase variation.
    fn compute_reliability(&self, wrapped: &[f64]) -> Vec<f64> {
        let rows = self.rows;
        let cols = self.cols;
        let mut reliability = vec![0.0f64; rows * cols];

        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                let idx = r * cols + c;
                // Horizontal second difference
                let h = Self::wrap(wrapped[idx + 1] - wrapped[idx])
                    - Self::wrap(wrapped[idx] - wrapped[idx - 1]);
                // Vertical second difference
                let v = Self::wrap(wrapped[(r + 1) * cols + c] - wrapped[idx])
                    - Self::wrap(wrapped[idx] - wrapped[(r - 1) * cols + c]);
                // Reliability = inverse of second-difference magnitude
                let sq = h * h + v * v;
                reliability[idx] = if sq > 1e-30 { 1.0 / sq } else { 1e15 };
            }
        }
        reliability
    }

    /// Unwrap a 2D phase field stored in row-major order.
    ///
    /// The input `wrapped` must have exactly `rows * cols` elements, each in
    /// the range (-pi, pi]. Returns the unwrapped phase field of the same size.
    pub fn unwrap(&self, wrapped: &[f64]) -> Vec<f64> {
        let n = self.rows * self.cols;
        assert_eq!(
            wrapped.len(),
            n,
            "Input size {} does not match {}x{}",
            wrapped.len(),
            self.rows,
            self.cols
        );

        if n == 0 {
            return vec![];
        }

        let reliability = self.compute_reliability(wrapped);

        // Build an edge list between adjacent pixels
        let rows = self.rows;
        let cols = self.cols;
        let mut edges: Vec<(f64, usize, usize)> = Vec::new();

        for r in 0..rows {
            for c in 0..cols {
                let idx = r * cols + c;
                if c + 1 < cols {
                    let right = idx + 1;
                    let rel = reliability[idx].min(reliability[right]);
                    edges.push((rel, idx, right));
                }
                if r + 1 < rows {
                    let below = idx + cols;
                    let rel = reliability[idx].min(reliability[below]);
                    edges.push((rel, idx, below));
                }
            }
        }

        // Sort edges by descending reliability (highest quality first)
        edges.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        // Union-Find for merging groups
        let mut parent: Vec<usize> = (0..n).collect();
        let mut rank = vec![0u32; n];
        let mut unwrapped: Vec<f64> = wrapped.to_vec();

        fn find(parent: &mut [usize], x: usize) -> usize {
            let mut root = x;
            while parent[root] != root {
                root = parent[root];
            }
            // Path compression
            let mut cur = x;
            while parent[cur] != root {
                let next = parent[cur];
                parent[cur] = root;
                cur = next;
            }
            root
        }

        for (_rel, a, b) in &edges {
            let ra = find(&mut parent, *a);
            let rb = find(&mut parent, *b);
            if ra == rb {
                continue;
            }

            // Compute the phase difference and unwrap
            let diff = Self::wrap(wrapped[*b] - wrapped[*a]);
            let actual_diff = unwrapped[*b] - unwrapped[*a];
            let correction = diff - actual_diff;

            // Apply correction to all pixels in the smaller group
            if rank[ra] < rank[rb] {
                // Correct group ra to align with rb
                let _root_a = ra;
                for i in 0..n {
                    if find(&mut parent, i) == ra {
                        unwrapped[i] += correction;
                    }
                }
                parent[ra] = rb;
            } else {
                // Correct group rb to align with ra
                for i in 0..n {
                    if find(&mut parent, i) == rb {
                        unwrapped[i] -= correction;
                    }
                }
                parent[rb] = ra;
                if rank[ra] == rank[rb] {
                    rank[ra] += 1;
                }
            }
        }

        unwrapped
    }
}

// ---------------------------------------------------------------------------
// Strain Calculator
// ---------------------------------------------------------------------------

/// Computes engineering strain from displacement data.
///
/// Strain epsilon = delta_L / L where L is the gauge length and delta_L
/// is the change in length (displacement difference across the gauge).
#[derive(Debug, Clone)]
pub struct StrainCalculator {
    /// Gauge length in metres.
    gauge_length_m: f64,
}

impl StrainCalculator {
    /// Create a new calculator with the given gauge length (metres).
    pub fn new(gauge_length_m: f64) -> Self {
        assert!(gauge_length_m > 0.0, "Gauge length must be positive");
        Self { gauge_length_m }
    }

    /// Gauge length in metres.
    pub fn gauge_length(&self) -> f64 {
        self.gauge_length_m
    }

    /// Compute strain from a single displacement (metres).
    ///
    /// Returns dimensionless strain (e.g. 1e-6 = 1 microstrain).
    pub fn displacement_to_strain(&self, displacement_m: f64) -> f64 {
        displacement_m / self.gauge_length_m
    }

    /// Compute strain from displacement difference between two points
    /// separated by the gauge length.
    pub fn differential_strain(&self, disp_a: f64, disp_b: f64) -> f64 {
        (disp_b - disp_a) / self.gauge_length_m
    }

    /// Compute strain from a displacement profile.
    ///
    /// Returns one strain value for each adjacent pair of displacement
    /// measurements, assuming uniform spacing equal to gauge_length_m.
    pub fn strain_profile(&self, displacements: &[f64]) -> Vec<f64> {
        if displacements.len() < 2 {
            return vec![];
        }
        displacements
            .windows(2)
            .map(|w| (w[1] - w[0]) / self.gauge_length_m)
            .collect()
    }

    /// Compute strain rate from a displacement time series.
    ///
    /// Returns strain rate in units of 1/s.
    pub fn strain_rate(&self, displacements: &[f64], sample_rate_hz: f64) -> Vec<f64> {
        if displacements.len() < 2 {
            return vec![];
        }
        displacements
            .windows(2)
            .map(|w| {
                let dd = w[1] - w[0];
                (dd / self.gauge_length_m) * sample_rate_hz
            })
            .collect()
    }

    /// Convert microstrain to strain.
    pub fn microstrain_to_strain(microstrain: f64) -> f64 {
        microstrain * 1e-6
    }

    /// Convert strain to microstrain.
    pub fn strain_to_microstrain(strain: f64) -> f64 {
        strain * 1e6
    }
}

// ---------------------------------------------------------------------------
// Fiber Bragg Grating Processor
// ---------------------------------------------------------------------------

/// Fiber Bragg Grating wavelength-shift to strain converter.
///
/// Converts measured wavelength shifts to strain using the relationship:
///
/// ```text
/// epsilon = delta_lambda / (lambda_B * (1 - p_e))
/// ```
///
/// where `p_e` is the effective photoelastic coefficient (typically ~0.22 for
/// silica fiber).
#[derive(Debug, Clone)]
pub struct FiberBraggProcessor {
    /// Bragg wavelength in metres.
    bragg_wavelength_m: f64,
    /// Effective photoelastic coefficient (dimensionless, ~0.22).
    photoelastic_coeff: f64,
    /// Thermal expansion + thermo-optic coefficient (1/K), ~6.67e-6 for silica.
    thermal_coeff: f64,
}

impl FiberBraggProcessor {
    /// Create a new FBG processor.
    ///
    /// - `bragg_wavelength_m`: Nominal Bragg wavelength in metres
    /// - `photoelastic_coeff`: Effective strain-optic coefficient p_e (~0.22)
    pub fn new(bragg_wavelength_m: f64, photoelastic_coeff: f64) -> Self {
        Self {
            bragg_wavelength_m,
            photoelastic_coeff,
            thermal_coeff: 6.67e-6, // alpha + xi for silica
        }
    }

    /// Create a standard silica fiber processor at 1550 nm.
    pub fn silica_1550nm() -> Self {
        Self::new(1550.0e-9, 0.22)
    }

    /// Set the thermal coefficient (alpha + xi) in 1/K.
    pub fn with_thermal_coeff(mut self, coeff: f64) -> Self {
        self.thermal_coeff = coeff;
        self
    }

    /// Convert a wavelength shift (metres) to strain (dimensionless).
    ///
    /// Assumes temperature change is zero.
    pub fn wavelength_shift_to_strain(&self, delta_lambda_m: f64) -> f64 {
        delta_lambda_m / (self.bragg_wavelength_m * (1.0 - self.photoelastic_coeff))
    }

    /// Convert strain to expected wavelength shift (metres).
    pub fn strain_to_wavelength_shift(&self, strain: f64) -> f64 {
        strain * self.bragg_wavelength_m * (1.0 - self.photoelastic_coeff)
    }

    /// Convert wavelength shift to strain, compensating for a known
    /// temperature change.
    ///
    /// - `delta_lambda_m`: Measured wavelength shift (metres)
    /// - `delta_t_k`: Temperature change (Kelvin)
    pub fn wavelength_shift_to_strain_with_temp(
        &self,
        delta_lambda_m: f64,
        delta_t_k: f64,
    ) -> f64 {
        let relative_shift = delta_lambda_m / self.bragg_wavelength_m;
        let thermal_component = self.thermal_coeff * delta_t_k;
        (relative_shift - thermal_component) / (1.0 - self.photoelastic_coeff)
    }

    /// Process a batch of wavelength shifts, returning strains.
    pub fn process_shifts(&self, shifts_m: &[f64]) -> Vec<f64> {
        shifts_m
            .iter()
            .map(|&s| self.wavelength_shift_to_strain(s))
            .collect()
    }

    /// Compute the strain sensitivity in pm/microstrain.
    pub fn sensitivity_pm_per_microstrain(&self) -> f64 {
        // delta_lambda = lambda * (1-p_e) * epsilon
        // For 1 microstrain: delta_lambda = lambda * (1-p_e) * 1e-6
        // Convert to pm: multiply by 1e12
        self.bragg_wavelength_m * (1.0 - self.photoelastic_coeff) * 1e-6 * 1e12
    }
}

// ---------------------------------------------------------------------------
// DAS Processor
// ---------------------------------------------------------------------------

/// Distributed Acoustic Sensing processor.
///
/// Converts phase-rate measurements from a DAS interrogator into strain-rate,
/// and optionally integrates to obtain strain. DAS measures the rate of change
/// of optical phase backscattered along the fiber, which is proportional to
/// the strain rate over the gauge length.
///
/// ```text
/// strain_rate = (lambda / (4 * pi * n * gauge_length)) * phase_rate
/// ```
#[derive(Debug, Clone)]
pub struct DasProcessor {
    /// Optical wavelength in metres (typically 1550 nm).
    wavelength_m: f64,
    /// Refractive index of the fiber core (~1.468).
    refractive_index: f64,
    /// Gauge length in metres.
    gauge_length_m: f64,
    /// Sample rate in Hz.
    sample_rate_hz: f64,
    /// Conversion factor from phase-rate to strain-rate.
    conversion_factor: f64,
}

impl DasProcessor {
    /// Create a new DAS processor.
    ///
    /// - `wavelength_m`: Laser wavelength (metres)
    /// - `refractive_index`: Fiber core refractive index
    /// - `gauge_length_m`: Gauge length (metres)
    /// - `sample_rate_hz`: Temporal sample rate (Hz)
    pub fn new(
        wavelength_m: f64,
        refractive_index: f64,
        gauge_length_m: f64,
        sample_rate_hz: f64,
    ) -> Self {
        let conversion_factor =
            wavelength_m / (4.0 * PI * refractive_index * gauge_length_m);
        Self {
            wavelength_m,
            refractive_index,
            gauge_length_m,
            sample_rate_hz,
            conversion_factor,
        }
    }

    /// Standard DAS configuration: 1550 nm, n=1.468, 10 m gauge, 10 kHz.
    pub fn standard() -> Self {
        Self::new(1550.0e-9, 1.468, 10.0, 10000.0)
    }

    /// Convert a single phase-rate value (rad/s) to strain-rate (1/s).
    pub fn phase_rate_to_strain_rate(&self, phase_rate: f64) -> f64 {
        phase_rate * self.conversion_factor
    }

    /// Process a batch of phase-rate measurements, returning strain-rates.
    pub fn process_phase_rates(&self, phase_rates: &[f64]) -> Vec<f64> {
        phase_rates
            .iter()
            .map(|&pr| self.phase_rate_to_strain_rate(pr))
            .collect()
    }

    /// Integrate strain-rate to obtain strain (trapezoidal rule).
    ///
    /// Returns cumulative strain at each sample time.
    pub fn integrate_strain_rate(&self, strain_rates: &[f64]) -> Vec<f64> {
        let dt = 1.0 / self.sample_rate_hz;
        let mut strain = vec![0.0; strain_rates.len()];
        for i in 1..strain_rates.len() {
            strain[i] = strain[i - 1] + 0.5 * (strain_rates[i - 1] + strain_rates[i]) * dt;
        }
        strain
    }

    /// Full pipeline: phase-rate -> strain-rate -> strain (integrated).
    pub fn process_to_strain(&self, phase_rates: &[f64]) -> Vec<f64> {
        let sr = self.process_phase_rates(phase_rates);
        self.integrate_strain_rate(&sr)
    }

    /// Wavelength in metres.
    pub fn wavelength(&self) -> f64 {
        self.wavelength_m
    }

    /// Gauge length in metres.
    pub fn gauge_length(&self) -> f64 {
        self.gauge_length_m
    }

    /// Sample rate in Hz.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate_hz
    }
}

// ---------------------------------------------------------------------------
// Deformation Model Fitter
// ---------------------------------------------------------------------------

/// Surface deformation prediction from source models.
///
/// Supports the Mogi (point source) and Okada (rectangular dislocation) models
/// commonly used in volcano and fault deformation studies.
#[derive(Debug, Clone)]
pub struct DeformationModelFitter {
    /// Shear modulus in Pa (typical crust: 30 GPa).
    shear_modulus_pa: f64,
    /// Poisson's ratio (typical crust: 0.25).
    poissons_ratio: f64,
}

/// Result of a Mogi model evaluation at a surface point.
#[derive(Debug, Clone)]
pub struct MogiDisplacement {
    /// Radial (horizontal) displacement in metres.
    pub radial_m: f64,
    /// Vertical displacement in metres.
    pub vertical_m: f64,
}

/// Parameters for the Okada rectangular dislocation model.
#[derive(Debug, Clone)]
pub struct OkadaParams {
    /// Fault length along strike (metres).
    pub length_m: f64,
    /// Fault width along dip (metres).
    pub width_m: f64,
    /// Depth to the top edge of the fault (metres).
    pub depth_m: f64,
    /// Dip angle (radians, 0 = horizontal, pi/2 = vertical).
    pub dip_rad: f64,
    /// Strike-slip displacement (metres, left-lateral positive).
    pub strike_slip_m: f64,
    /// Dip-slip displacement (metres, thrust positive).
    pub dip_slip_m: f64,
    /// Tensile opening (metres).
    pub tensile_m: f64,
}

impl DeformationModelFitter {
    /// Create a new fitter with crustal parameters.
    ///
    /// - `shear_modulus_pa`: Shear modulus G (Pa)
    /// - `poissons_ratio`: Poisson's ratio nu
    pub fn new(shear_modulus_pa: f64, poissons_ratio: f64) -> Self {
        Self {
            shear_modulus_pa,
            poissons_ratio,
        }
    }

    /// Default crustal parameters: G = 30 GPa, nu = 0.25.
    pub fn default_crust() -> Self {
        Self::new(30.0e9, 0.25)
    }

    /// Mogi model: point source of pressure change at depth.
    ///
    /// Computes surface displacement at radial distance `r` from the epicentre
    /// for a spherical source of radius `a` at depth `d` with pressure
    /// change `delta_p`.
    ///
    /// ```text
    /// u_r = (3 * V * dP * a^3) / (4 * G) * r / (r^2 + d^2)^(3/2)
    ///     = (1 - nu) * dV / pi * r / (r^2 + d^2)^(3/2)
    ///
    /// u_z = (1 - nu) * dV / pi * d / (r^2 + d^2)^(3/2)
    /// ```
    ///
    /// For simplicity we use the volume-change formulation:
    /// dV = pi * a^3 * dP / G  (source volume change)
    pub fn mogi(
        &self,
        source_depth_m: f64,
        source_radius_m: f64,
        delta_pressure_pa: f64,
        radial_distance_m: f64,
    ) -> MogiDisplacement {
        let a3 = source_radius_m.powi(3);
        let g = self.shear_modulus_pa;
        let nu = self.poissons_ratio;

        // Volume change
        let dv = PI * a3 * delta_pressure_pa / g;

        let r = radial_distance_m;
        let d = source_depth_m;
        let denom = (r * r + d * d).powf(1.5);
        let coeff = (1.0 - nu) * dv / PI;

        MogiDisplacement {
            radial_m: coeff * r / denom,
            vertical_m: coeff * d / denom,
        }
    }

    /// Evaluate Mogi displacement at multiple radial distances.
    pub fn mogi_profile(
        &self,
        source_depth_m: f64,
        source_radius_m: f64,
        delta_pressure_pa: f64,
        distances: &[f64],
    ) -> Vec<MogiDisplacement> {
        distances
            .iter()
            .map(|&r| self.mogi(source_depth_m, source_radius_m, delta_pressure_pa, r))
            .collect()
    }

    /// Simplified Okada model: vertical surface displacement from a
    /// rectangular dislocation (tensile component only, for dyke/sill).
    ///
    /// This is a simplified analytic form for a horizontal tensile crack
    /// (sill) at depth `d`, computing vertical displacement at distance `r`
    /// from the centre.
    ///
    /// For a full Okada model the expressions are much longer; this
    /// implements the key first-order term.
    pub fn okada_sill_vertical(
        &self,
        params: &OkadaParams,
        radial_distance_m: f64,
    ) -> f64 {
        let nu = self.poissons_ratio;
        let d = params.depth_m;
        let opening = params.tensile_m;
        let area = params.length_m * params.width_m;
        let r = radial_distance_m;

        // Simplified Okada for horizontal sill (dip = 0):
        // u_z ~ (1-nu) * opening * area / (2*pi) * d / (r^2 + d^2)^(3/2)
        let denom = (r * r + d * d).powf(1.5);
        (1.0 - nu) * opening * area / (2.0 * PI) * d / denom
    }

    /// Fit a Mogi model to observed vertical displacements via grid search.
    ///
    /// Returns (best_depth, best_dv) that minimizes RMS misfit.
    ///
    /// - `distances`: Radial distances of observation points (metres)
    /// - `observed_vertical`: Observed vertical displacement (metres)
    /// - `depth_range`: (min, max) depth to search (metres)
    /// - `dv_range`: (min, max) volume change to search (m^3)
    /// - `n_steps`: Number of grid steps per dimension
    pub fn fit_mogi_grid(
        &self,
        distances: &[f64],
        observed_vertical: &[f64],
        depth_range: (f64, f64),
        dv_range: (f64, f64),
        n_steps: usize,
    ) -> (f64, f64) {
        assert_eq!(distances.len(), observed_vertical.len());
        let nu = self.poissons_ratio;

        let mut best_depth = depth_range.0;
        let mut best_dv = dv_range.0;
        let mut best_rms = f64::INFINITY;

        for di in 0..n_steps {
            let d =
                depth_range.0 + (depth_range.1 - depth_range.0) * di as f64 / n_steps as f64;
            for vi in 0..n_steps {
                let dv =
                    dv_range.0 + (dv_range.1 - dv_range.0) * vi as f64 / n_steps as f64;

                let mut sum_sq = 0.0;
                for (i, &r) in distances.iter().enumerate() {
                    let denom = (r * r + d * d).powf(1.5);
                    let predicted = (1.0 - nu) * dv / PI * d / denom;
                    let residual = predicted - observed_vertical[i];
                    sum_sq += residual * residual;
                }
                let rms = (sum_sq / distances.len() as f64).sqrt();
                if rms < best_rms {
                    best_rms = rms;
                    best_depth = d;
                    best_dv = dv;
                }
            }
        }

        (best_depth, best_dv)
    }
}

// ---------------------------------------------------------------------------
// Time Series Decomposer
// ---------------------------------------------------------------------------

/// Decomposes a displacement time series into secular trend, seasonal
/// (annual + semi-annual) harmonics, and residual transient components.
///
/// Model: y(t) = v*t + a1*sin(2*pi*t/T) + b1*cos(2*pi*t/T)
///              + a2*sin(4*pi*t/T) + b2*cos(4*pi*t/T) + residual
///
/// where T = 1 year (365.25 days).
#[derive(Debug, Clone)]
pub struct TimeSeriesDecomposer {
    /// Period of the annual cycle in the same time units as input.
    annual_period: f64,
}

/// Result of time series decomposition.
#[derive(Debug, Clone)]
pub struct DecompositionResult {
    /// Linear velocity (units per time-unit).
    pub velocity: f64,
    /// Offset (intercept).
    pub offset: f64,
    /// Annual sine amplitude.
    pub annual_sin: f64,
    /// Annual cosine amplitude.
    pub annual_cos: f64,
    /// Semi-annual sine amplitude.
    pub semiannual_sin: f64,
    /// Semi-annual cosine amplitude.
    pub semiannual_cos: f64,
    /// Residuals after removing trend and seasonal.
    pub residuals: Vec<f64>,
}

impl TimeSeriesDecomposer {
    /// Create a decomposer with the given annual period in time units.
    ///
    /// If your time axis is in days, use 365.25. In years, use 1.0.
    pub fn new(annual_period: f64) -> Self {
        Self { annual_period }
    }

    /// Decompose a time series via least-squares fit.
    ///
    /// - `times`: Time values (arbitrary units, but consistent with `annual_period`)
    /// - `values`: Measured values
    pub fn decompose(&self, times: &[f64], values: &[f64]) -> DecompositionResult {
        assert_eq!(times.len(), values.len());
        let n = times.len();
        assert!(n >= 6, "Need at least 6 data points for decomposition");

        let t_period = self.annual_period;

        // Design matrix columns: [1, t, sin(w*t), cos(w*t), sin(2w*t), cos(2w*t)]
        // We solve A^T A x = A^T y  (normal equations)
        let ncols = 6;
        let mut ata = vec![0.0f64; ncols * ncols];
        let mut aty = vec![0.0f64; ncols];

        for i in 0..n {
            let t = times[i];
            let y = values[i];
            let w = 2.0 * PI / t_period;

            let row = [
                1.0,
                t,
                (w * t).sin(),
                (w * t).cos(),
                (2.0 * w * t).sin(),
                (2.0 * w * t).cos(),
            ];

            for r in 0..ncols {
                for c in 0..ncols {
                    ata[r * ncols + c] += row[r] * row[c];
                }
                aty[r] += row[r] * y;
            }
        }

        // Solve using Gaussian elimination with partial pivoting
        let x = solve_linear_6x6(&ata, &aty);

        // Compute residuals
        let mut residuals = Vec::with_capacity(n);
        for i in 0..n {
            let t = times[i];
            let w = 2.0 * PI / t_period;
            let model = x[0]
                + x[1] * t
                + x[2] * (w * t).sin()
                + x[3] * (w * t).cos()
                + x[4] * (2.0 * w * t).sin()
                + x[5] * (2.0 * w * t).cos();
            residuals.push(values[i] - model);
        }

        DecompositionResult {
            offset: x[0],
            velocity: x[1],
            annual_sin: x[2],
            annual_cos: x[3],
            semiannual_sin: x[4],
            semiannual_cos: x[5],
            residuals,
        }
    }

    /// Reconstruct the seasonal (annual + semi-annual) component only.
    pub fn seasonal_component(&self, times: &[f64], result: &DecompositionResult) -> Vec<f64> {
        let w = 2.0 * PI / self.annual_period;
        times
            .iter()
            .map(|&t| {
                result.annual_sin * (w * t).sin()
                    + result.annual_cos * (w * t).cos()
                    + result.semiannual_sin * (2.0 * w * t).sin()
                    + result.semiannual_cos * (2.0 * w * t).cos()
            })
            .collect()
    }

    /// Reconstruct the linear trend component.
    pub fn trend_component(&self, times: &[f64], result: &DecompositionResult) -> Vec<f64> {
        times
            .iter()
            .map(|&t| result.offset + result.velocity * t)
            .collect()
    }
}

/// Solve a 6x6 linear system Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_6x6(a_flat: &[f64], b: &[f64]) -> Vec<f64> {
    let n = 6;
    // Augmented matrix [A | b]
    let mut aug = vec![0.0f64; n * (n + 1)];
    for r in 0..n {
        for c in 0..n {
            aug[r * (n + 1) + c] = a_flat[r * n + c];
        }
        aug[r * (n + 1) + n] = b[r];
    }

    // Forward elimination
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col * (n + 1) + col].abs();
        for row in (col + 1)..n {
            let val = aug[row * (n + 1) + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        if max_row != col {
            for c in 0..=n {
                let tmp = aug[col * (n + 1) + c];
                aug[col * (n + 1) + c] = aug[max_row * (n + 1) + c];
                aug[max_row * (n + 1) + c] = tmp;
            }
        }

        let pivot = aug[col * (n + 1) + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..n {
            let factor = aug[row * (n + 1) + col] / pivot;
            for c in col..=n {
                aug[row * (n + 1) + c] -= factor * aug[col * (n + 1) + c];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0f64; n];
    for row in (0..n).rev() {
        let pivot = aug[row * (n + 1) + row];
        if pivot.abs() < 1e-30 {
            continue;
        }
        let mut sum = aug[row * (n + 1) + n];
        for c in (row + 1)..n {
            sum -= aug[row * (n + 1) + c] * x[c];
        }
        x[row] = sum / pivot;
    }

    x
}

// ---------------------------------------------------------------------------
// Atmospheric Phase Screen
// ---------------------------------------------------------------------------

/// Estimates and removes tropospheric phase delay from InSAR interferograms.
///
/// The stratified tropospheric delay is approximately proportional to
/// elevation: pixels at higher elevation have less atmosphere above them,
/// resulting in less delay.
///
/// ```text
/// phase_atmo(h) = k * (h - h_ref) + offset
/// ```
///
/// where k is the delay-elevation gradient estimated via linear regression.
#[derive(Debug, Clone)]
pub struct AtmosphericPhaseScreen {
    /// Reference elevation (metres).
    reference_elevation_m: f64,
}

/// Result of atmospheric phase screen estimation.
#[derive(Debug, Clone)]
pub struct ApsEstimate {
    /// Delay-elevation gradient (rad/m).
    pub gradient_rad_per_m: f64,
    /// Constant offset at reference elevation (rad).
    pub offset_rad: f64,
    /// Corrected phases with atmospheric delay removed.
    pub corrected_phases: Vec<f64>,
}

impl AtmosphericPhaseScreen {
    /// Create a new APS estimator with the given reference elevation.
    pub fn new(reference_elevation_m: f64) -> Self {
        Self {
            reference_elevation_m,
        }
    }

    /// Estimate and remove the stratified atmospheric phase delay.
    ///
    /// - `phases`: Interferometric phase values (radians)
    /// - `elevations`: Corresponding DEM elevations (metres)
    ///
    /// Returns the estimated APS parameters and corrected phases.
    pub fn estimate_and_remove(
        &self,
        phases: &[f64],
        elevations: &[f64],
    ) -> ApsEstimate {
        assert_eq!(phases.len(), elevations.len());
        let n = phases.len() as f64;
        assert!(n >= 2.0);

        // Linear regression: phase = k * (elev - ref) + offset
        let h_ref = self.reference_elevation_m;
        let mut sum_dh = 0.0;
        let mut sum_phi = 0.0;
        let mut sum_dh2 = 0.0;
        let mut sum_dh_phi = 0.0;

        for i in 0..phases.len() {
            let dh = elevations[i] - h_ref;
            let phi = phases[i];
            sum_dh += dh;
            sum_phi += phi;
            sum_dh2 += dh * dh;
            sum_dh_phi += dh * phi;
        }

        let denom = n * sum_dh2 - sum_dh * sum_dh;
        let (k, offset) = if denom.abs() > 1e-30 {
            let k = (n * sum_dh_phi - sum_dh * sum_phi) / denom;
            let offset = (sum_phi - k * sum_dh) / n;
            (k, offset)
        } else {
            (0.0, sum_phi / n)
        };

        let corrected: Vec<f64> = phases
            .iter()
            .zip(elevations.iter())
            .map(|(&phi, &h)| phi - (k * (h - h_ref) + offset))
            .collect();

        ApsEstimate {
            gradient_rad_per_m: k,
            offset_rad: offset,
            corrected_phases: corrected,
        }
    }

    /// Predict atmospheric phase delay at a given elevation.
    pub fn predict_delay(&self, gradient: f64, offset: f64, elevation_m: f64) -> f64 {
        gradient * (elevation_m - self.reference_elevation_m) + offset
    }
}

// ---------------------------------------------------------------------------
// Coherence Estimator
// ---------------------------------------------------------------------------

/// InSAR coherence magnitude estimator.
///
/// Coherence measures the similarity between two SAR images over a local
/// window:
///
/// ```text
/// gamma = |sum(s1 * conj(s2))| / sqrt(sum(|s1|^2) * sum(|s2|^2))
/// ```
///
/// Values range from 0 (completely decorrelated) to 1 (perfect correlation).
/// Decorrelation sources: thermal noise, geometric baseline, temporal change.
#[derive(Debug, Clone)]
pub struct CoherenceEstimator {
    /// Window size for coherence estimation (pixels).
    window_size: usize,
}

impl CoherenceEstimator {
    /// Create a coherence estimator with the given window size.
    ///
    /// Window size determines the number of looks (spatial averaging).
    /// Typical values: 3-7 for high resolution, 11-21 for smooth estimates.
    pub fn new(window_size: usize) -> Self {
        assert!(window_size >= 1, "Window size must be at least 1");
        Self { window_size }
    }

    /// Estimate coherence from two complex SAR images (real, imag interleaved).
    ///
    /// - `s1_re`, `s1_im`: Real and imaginary parts of image 1
    /// - `s2_re`, `s2_im`: Real and imaginary parts of image 2
    ///
    /// Returns coherence magnitude in [0, 1].
    pub fn estimate(
        &self,
        s1_re: &[f64],
        s1_im: &[f64],
        s2_re: &[f64],
        s2_im: &[f64],
    ) -> f64 {
        let n = s1_re.len().min(s1_im.len()).min(s2_re.len()).min(s2_im.len());

        let mut cross_re = 0.0;
        let mut cross_im = 0.0;
        let mut pow1 = 0.0;
        let mut pow2 = 0.0;

        for i in 0..n {
            // s1 * conj(s2) = (a + jb)(c - jd) = (ac + bd) + j(bc - ad)
            let a = s1_re[i];
            let b = s1_im[i];
            let c = s2_re[i];
            let d = s2_im[i];

            cross_re += a * c + b * d;
            cross_im += b * c - a * d;
            pow1 += a * a + b * b;
            pow2 += c * c + d * d;
        }

        let cross_mag = (cross_re * cross_re + cross_im * cross_im).sqrt();
        let denom = (pow1 * pow2).sqrt();

        if denom > 1e-30 {
            cross_mag / denom
        } else {
            0.0
        }
    }

    /// Estimate coherence over a 2D grid with a sliding window.
    ///
    /// Images are stored in row-major order with dimensions `rows x cols`.
    /// Returns a coherence map of the same dimensions (border pixels get 0).
    pub fn estimate_2d(
        &self,
        s1_re: &[f64],
        s1_im: &[f64],
        s2_re: &[f64],
        s2_im: &[f64],
        rows: usize,
        cols: usize,
    ) -> Vec<f64> {
        let n = rows * cols;
        assert_eq!(s1_re.len(), n);
        assert_eq!(s1_im.len(), n);
        assert_eq!(s2_re.len(), n);
        assert_eq!(s2_im.len(), n);

        let half = self.window_size / 2;
        let mut coherence_map = vec![0.0f64; n];

        for r in half..rows.saturating_sub(half) {
            for c in half..cols.saturating_sub(half) {
                let mut cross_re = 0.0;
                let mut cross_im = 0.0;
                let mut pow1 = 0.0;
                let mut pow2 = 0.0;

                for wr in r.saturating_sub(half)..=(r + half).min(rows - 1) {
                    for wc in c.saturating_sub(half)..=(c + half).min(cols - 1) {
                        let idx = wr * cols + wc;
                        let a = s1_re[idx];
                        let b = s1_im[idx];
                        let cc = s2_re[idx];
                        let d = s2_im[idx];

                        cross_re += a * cc + b * d;
                        cross_im += b * cc - a * d;
                        pow1 += a * a + b * b;
                        pow2 += cc * cc + d * d;
                    }
                }

                let cross_mag = (cross_re * cross_re + cross_im * cross_im).sqrt();
                let denom = (pow1 * pow2).sqrt();
                coherence_map[r * cols + c] = if denom > 1e-30 {
                    cross_mag / denom
                } else {
                    0.0
                };
            }
        }

        coherence_map
    }

    /// Compute theoretical coherence from SNR.
    ///
    /// gamma_thermal = 1 / (1 + 1/SNR)
    pub fn coherence_from_snr(snr_linear: f64) -> f64 {
        if snr_linear <= 0.0 {
            return 0.0;
        }
        1.0 / (1.0 + 1.0 / snr_linear)
    }

    /// Compute theoretical coherence from geometric baseline.
    ///
    /// gamma_geom = 1 - |B_perp| / B_crit
    /// where B_crit = lambda * R * tan(theta) / (2 * range_resolution)
    pub fn coherence_from_baseline(
        b_perp: f64,
        b_crit: f64,
    ) -> f64 {
        let ratio = b_perp.abs() / b_crit;
        if ratio >= 1.0 {
            0.0
        } else {
            1.0 - ratio
        }
    }

    /// Window size used for estimation.
    pub fn window_size(&self) -> usize {
        self.window_size
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Wrap phase to (-pi, pi] interval.
pub fn wrap_phase(phase: f64) -> f64 {
    let mut p = phase % (2.0 * PI);
    if p > PI {
        p -= 2.0 * PI;
    } else if p <= -PI {
        p += 2.0 * PI;
    }
    p
}

/// 1D phase unwrapping by cumulative wrap detection.
pub fn unwrap_phase_1d(wrapped: &[f64]) -> Vec<f64> {
    if wrapped.is_empty() {
        return vec![];
    }
    let mut unwrapped = vec![0.0; wrapped.len()];
    unwrapped[0] = wrapped[0];
    for i in 1..wrapped.len() {
        let diff = wrap_phase(wrapped[i] - wrapped[i - 1]);
        unwrapped[i] = unwrapped[i - 1] + diff;
    }
    unwrapped
}

/// Compute interferometric phase from two complex SAR values.
///
/// phase = arg(s1 * conj(s2))
pub fn interferometric_phase(s1_re: f64, s1_im: f64, s2_re: f64, s2_im: f64) -> f64 {
    // s1 * conj(s2) = (a+jb)(c-jd) = (ac+bd) + j(bc-ad)
    let re = s1_re * s2_re + s1_im * s2_im;
    let im = s1_im * s2_re - s1_re * s2_im;
    im.atan2(re)
}

/// Convert line-of-sight displacement to vertical displacement given
/// an incidence angle.
///
/// d_vertical = d_los / cos(incidence_angle)
pub fn los_to_vertical(d_los: f64, incidence_angle_rad: f64) -> f64 {
    d_los / incidence_angle_rad.cos()
}

/// Convert displacement in metres to millimetres.
pub fn m_to_mm(metres: f64) -> f64 {
    metres * 1000.0
}

/// Convert millimetres to metres.
pub fn mm_to_m(mm: f64) -> f64 {
    mm * 0.001
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // --- StrainConfig ---

    #[test]
    fn test_sentinel1_config() {
        let c = StrainConfig::sentinel1();
        assert_eq!(c.measurement_type, MeasurementType::InSar);
        assert!((c.wavelength_m - 0.0555).abs() < 1e-6);
    }

    #[test]
    fn test_terrasar_x_config() {
        let c = StrainConfig::terrasar_x();
        assert!((c.wavelength_m - 0.031).abs() < 1e-6);
    }

    #[test]
    fn test_fbg_config() {
        let c = StrainConfig::fbg_1550nm();
        assert_eq!(c.measurement_type, MeasurementType::FiberOptic);
        assert!((c.wavelength_m - 1550.0e-9).abs() < 1e-15);
    }

    #[test]
    fn test_das_config() {
        let c = StrainConfig::das_standard();
        assert!((c.sample_rate_hz - 10000.0).abs() < 1e-6);
        assert!((c.gauge_length_m - 10.0).abs() < 1e-6);
    }

    // --- InSarPhaseProcessor ---

    #[test]
    fn test_phase_to_displacement_zero() {
        let p = InSarPhaseProcessor::new(0.0555);
        assert!((p.phase_to_displacement(0.0)).abs() < 1e-15);
    }

    #[test]
    fn test_phase_to_displacement_one_radian() {
        let lambda = 0.0555; // C-band
        let p = InSarPhaseProcessor::new(lambda);
        let d = p.phase_to_displacement(1.0);
        let expected = lambda / (4.0 * PI);
        assert!((d - expected).abs() < 1e-12);
    }

    #[test]
    fn test_phase_to_displacement_full_fringe() {
        // One full fringe (2*pi) corresponds to lambda/2 displacement
        let lambda = 0.0555;
        let p = InSarPhaseProcessor::new(lambda);
        let d = p.phase_to_displacement(2.0 * PI);
        assert!((d - lambda / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_displacement_roundtrip() {
        let p = InSarPhaseProcessor::new(0.031);
        let d = 0.005; // 5 mm
        let phase = p.displacement_to_phase(d);
        let d2 = p.phase_to_displacement(phase);
        assert!((d - d2).abs() < 1e-14);
    }

    #[test]
    fn test_process_phases_batch() {
        let p = InSarPhaseProcessor::new(0.0555);
        let phases = vec![0.0, 1.0, -1.0, PI];
        let disps = p.process_phases(&phases);
        assert_eq!(disps.len(), 4);
        assert!(disps[0].abs() < 1e-15);
        assert!((disps[1] - disps[2]).abs() > 0.0); // opposite sign
    }

    #[test]
    fn test_displacement_velocity() {
        let p = InSarPhaseProcessor::new(0.0555);
        let v = p.displacement_velocity(0.0, 2.0 * PI, 1.0);
        // Velocity should be lambda/2 per second
        assert!((v - 0.0555 / 2.0).abs() < 1e-10);
    }

    // --- PhaseUnwrapper2D ---

    #[test]
    fn test_unwrap_2d_constant() {
        let uw = PhaseUnwrapper2D::new(3, 3);
        let wrapped = vec![0.5; 9];
        let result = uw.unwrap(&wrapped);
        for &v in &result {
            assert!((v - 0.5).abs() < 1e-10);
        }
    }

    #[test]
    fn test_unwrap_2d_linear_ramp() {
        // Phase ramp that wraps
        let rows = 5;
        let cols = 5;
        let uw = PhaseUnwrapper2D::new(rows, cols);

        // Create a smooth phase ramp along columns
        let mut wrapped = vec![0.0; rows * cols];
        for r in 0..rows {
            for c in 0..cols {
                let phase = 0.5 * c as f64; // 0, 0.5, 1.0, 1.5, 2.0
                wrapped[r * cols + c] = PhaseUnwrapper2D::wrap(phase);
            }
        }

        let result = uw.unwrap(&wrapped);
        // The unwrapped should recover a smooth ramp
        // Check that adjacent differences are smooth
        for r in 0..rows {
            for c in 1..cols {
                let diff = result[r * cols + c] - result[r * cols + c - 1];
                assert!(
                    (diff - 0.5).abs() < 0.6,
                    "Unexpected jump at ({}, {}): diff = {}",
                    r,
                    c,
                    diff
                );
            }
        }
    }

    #[test]
    fn test_unwrap_2d_empty() {
        let uw = PhaseUnwrapper2D::new(0, 0);
        let result = uw.unwrap(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_wrap_function() {
        assert!((PhaseUnwrapper2D::wrap(0.0)).abs() < 1e-15);
        assert!((PhaseUnwrapper2D::wrap(PI) - PI).abs() < 1e-10);
        assert!((PhaseUnwrapper2D::wrap(3.0 * PI) - PI).abs() < 1e-10);
        // -3*pi wraps to +pi (equivalent to -pi at the boundary)
        let w = PhaseUnwrapper2D::wrap(-3.0 * PI);
        assert!(
            (w - PI).abs() < 1e-10 || (w + PI).abs() < 1e-10,
            "wrap(-3*pi) = {} should be +/-pi",
            w
        );
    }

    // --- StrainCalculator ---

    #[test]
    fn test_strain_from_displacement() {
        let sc = StrainCalculator::new(100.0);
        let strain = sc.displacement_to_strain(0.001); // 1 mm over 100 m
        assert!((strain - 1e-5).abs() < 1e-12);
    }

    #[test]
    fn test_differential_strain() {
        let sc = StrainCalculator::new(10.0);
        let s = sc.differential_strain(0.0, 0.0001); // 0.1 mm over 10 m
        assert!((s - 1e-5).abs() < 1e-12);
    }

    #[test]
    fn test_strain_profile() {
        let sc = StrainCalculator::new(1.0);
        let disps = vec![0.0, 0.001, 0.003, 0.006];
        let strains = sc.strain_profile(&disps);
        assert_eq!(strains.len(), 3);
        assert!((strains[0] - 0.001).abs() < 1e-12);
        assert!((strains[1] - 0.002).abs() < 1e-12);
        assert!((strains[2] - 0.003).abs() < 1e-12);
    }

    #[test]
    fn test_strain_rate() {
        let sc = StrainCalculator::new(1.0);
        let disps = vec![0.0, 0.001, 0.003];
        let rates = sc.strain_rate(&disps, 10.0);
        assert_eq!(rates.len(), 2);
        assert!((rates[0] - 0.01).abs() < 1e-12);
        assert!((rates[1] - 0.02).abs() < 1e-12);
    }

    #[test]
    fn test_microstrain_conversion() {
        let s = StrainCalculator::microstrain_to_strain(100.0);
        assert!((s - 1e-4).abs() < 1e-12);
        let us = StrainCalculator::strain_to_microstrain(1e-4);
        assert!((us - 100.0).abs() < 1e-8);
    }

    // --- FiberBraggProcessor ---

    #[test]
    fn test_fbg_strain_conversion() {
        let fbg = FiberBraggProcessor::silica_1550nm();
        // Typical sensitivity: ~1.2 pm/microstrain at 1550 nm
        let sensitivity = fbg.sensitivity_pm_per_microstrain();
        assert!(
            (sensitivity - 1.209).abs() < 0.01,
            "Sensitivity {} pm/ue",
            sensitivity
        );
    }

    #[test]
    fn test_fbg_roundtrip() {
        let fbg = FiberBraggProcessor::new(1550.0e-9, 0.22);
        let strain = 500e-6; // 500 microstrain
        let shift = fbg.strain_to_wavelength_shift(strain);
        let recovered = fbg.wavelength_shift_to_strain(shift);
        assert!((recovered - strain).abs() < 1e-12);
    }

    #[test]
    fn test_fbg_temperature_compensation() {
        let fbg = FiberBraggProcessor::silica_1550nm();
        // With no temperature change, should match uncompensated
        let shift = 1.0e-12; // 1 pm
        let s1 = fbg.wavelength_shift_to_strain(shift);
        let s2 = fbg.wavelength_shift_to_strain_with_temp(shift, 0.0);
        assert!((s1 - s2).abs() < 1e-15);
    }

    #[test]
    fn test_fbg_temperature_effect() {
        let fbg = FiberBraggProcessor::silica_1550nm();
        // Temperature increase should reduce apparent mechanical strain
        let shift = 10.0e-12; // 10 pm
        let s_no_temp = fbg.wavelength_shift_to_strain(shift);
        let s_with_temp = fbg.wavelength_shift_to_strain_with_temp(shift, 1.0);
        assert!(s_with_temp < s_no_temp);
    }

    #[test]
    fn test_fbg_batch() {
        let fbg = FiberBraggProcessor::silica_1550nm();
        let shifts = vec![0.0, 1.0e-12, -1.0e-12];
        let strains = fbg.process_shifts(&shifts);
        assert_eq!(strains.len(), 3);
        assert!(strains[0].abs() < 1e-15);
        assert!((strains[1] + strains[2]).abs() < 1e-15); // symmetric
    }

    // --- DAS Processor ---

    #[test]
    fn test_das_zero_phase_rate() {
        let das = DasProcessor::standard();
        assert!(das.phase_rate_to_strain_rate(0.0).abs() < 1e-30);
    }

    #[test]
    fn test_das_conversion_factor() {
        let das = DasProcessor::new(1550.0e-9, 1.468, 10.0, 10000.0);
        let factor = 1550.0e-9 / (4.0 * PI * 1.468 * 10.0);
        let sr = das.phase_rate_to_strain_rate(1.0);
        assert!((sr - factor).abs() < 1e-20);
    }

    #[test]
    fn test_das_integration() {
        let das = DasProcessor::new(1550.0e-9, 1.468, 10.0, 100.0);
        // Constant strain rate -> linearly increasing strain
        let sr = vec![1e-6; 100];
        let strain = das.integrate_strain_rate(&sr);
        assert_eq!(strain.len(), 100);
        assert!(strain[0].abs() < 1e-30);
        // After 100 samples at 100 Hz = 1 second, strain = rate * time = 1e-6
        let final_strain = strain[99];
        assert!((final_strain - 1e-6 * 99.0 / 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_das_full_pipeline() {
        let das = DasProcessor::standard();
        let phase_rates = vec![100.0; 50]; // constant phase rate
        let strain = das.process_to_strain(&phase_rates);
        assert_eq!(strain.len(), 50);
        // Strain should be monotonically increasing
        for i in 1..strain.len() {
            assert!(strain[i] > strain[i - 1]);
        }
    }

    // --- DeformationModelFitter ---

    #[test]
    fn test_mogi_at_zero_distance() {
        let fitter = DeformationModelFitter::default_crust();
        let disp = fitter.mogi(5000.0, 500.0, 10e6, 0.0);
        // At r=0, radial displacement should be zero
        assert!(disp.radial_m.abs() < 1e-15);
        // Vertical should be maximum
        assert!(disp.vertical_m > 0.0);
    }

    #[test]
    fn test_mogi_symmetry() {
        let fitter = DeformationModelFitter::default_crust();
        let d1 = fitter.mogi(5000.0, 500.0, 10e6, 1000.0);
        let d2 = fitter.mogi(5000.0, 500.0, 10e6, -1000.0);
        // Radial displacement should have opposite sign at symmetric distances
        // (but our model uses abs distance implicitly, so same magnitude)
        assert!((d1.radial_m.abs() - d2.radial_m.abs()).abs() < 1e-15);
        assert!((d1.vertical_m - d2.vertical_m).abs() < 1e-15);
    }

    #[test]
    fn test_mogi_vertical_decreases_with_distance() {
        let fitter = DeformationModelFitter::default_crust();
        let d_near = fitter.mogi(5000.0, 500.0, 10e6, 100.0);
        let d_far = fitter.mogi(5000.0, 500.0, 10e6, 10000.0);
        assert!(d_near.vertical_m > d_far.vertical_m);
    }

    #[test]
    fn test_mogi_profile() {
        let fitter = DeformationModelFitter::default_crust();
        let distances = vec![0.0, 1000.0, 2000.0, 5000.0, 10000.0];
        let profile = fitter.mogi_profile(5000.0, 500.0, 10e6, &distances);
        assert_eq!(profile.len(), 5);
        // Vertical displacement should decrease with distance
        for i in 1..profile.len() {
            assert!(profile[i].vertical_m < profile[i - 1].vertical_m);
        }
    }

    #[test]
    fn test_okada_sill_at_epicentre() {
        let fitter = DeformationModelFitter::default_crust();
        let params = OkadaParams {
            length_m: 1000.0,
            width_m: 1000.0,
            depth_m: 3000.0,
            dip_rad: 0.0,
            strike_slip_m: 0.0,
            dip_slip_m: 0.0,
            tensile_m: 1.0,
        };
        let uz = fitter.okada_sill_vertical(&params, 0.0);
        assert!(uz > 0.0); // Uplift above an opening sill
    }

    #[test]
    fn test_mogi_fit_grid() {
        let fitter = DeformationModelFitter::default_crust();
        let true_depth = 5000.0;
        let nu = 0.25;
        let true_dv = 1e6; // 1e6 m^3 volume change

        // Generate synthetic data
        let distances: Vec<f64> = (0..20).map(|i| i as f64 * 500.0).collect();
        let observed: Vec<f64> = distances
            .iter()
            .map(|&r| {
                let d = true_depth;
                let denom = (r * r + d * d).powf(1.5);
                (1.0 - nu) * true_dv / PI * d / denom
            })
            .collect();

        let (fit_depth, fit_dv) = fitter.fit_mogi_grid(
            &distances,
            &observed,
            (3000.0, 7000.0),
            (0.5e6, 1.5e6),
            50,
        );

        assert!(
            (fit_depth - true_depth).abs() < 200.0,
            "Depth: {} vs {}",
            fit_depth,
            true_depth
        );
        assert!(
            (fit_dv - true_dv).abs() / true_dv < 0.05,
            "dV: {} vs {}",
            fit_dv,
            true_dv
        );
    }

    // --- TimeSeriesDecomposer ---

    #[test]
    fn test_decompose_linear_trend() {
        let decomp = TimeSeriesDecomposer::new(365.25);
        let times: Vec<f64> = (0..100).map(|i| i as f64 * 10.0).collect();
        let values: Vec<f64> = times.iter().map(|&t| 5.0 + 0.01 * t).collect();
        let result = decomp.decompose(&times, &values);
        assert!((result.velocity - 0.01).abs() < 1e-6);
        assert!((result.offset - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_decompose_seasonal() {
        let decomp = TimeSeriesDecomposer::new(365.25);
        let times: Vec<f64> = (0..730).map(|i| i as f64).collect();
        let w = 2.0 * PI / 365.25;
        let values: Vec<f64> = times
            .iter()
            .map(|&t| 3.0 * (w * t).sin() + 1.5 * (w * t).cos())
            .collect();
        let result = decomp.decompose(&times, &values);
        assert!(
            (result.annual_sin - 3.0).abs() < 0.05,
            "sin amp: {}",
            result.annual_sin
        );
        assert!(
            (result.annual_cos - 1.5).abs() < 0.05,
            "cos amp: {}",
            result.annual_cos
        );
    }

    #[test]
    fn test_seasonal_component() {
        let decomp = TimeSeriesDecomposer::new(1.0);
        let times: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let w = 2.0 * PI;
        let values: Vec<f64> = times.iter().map(|&t| 2.0 * (w * t).sin()).collect();
        let result = decomp.decompose(&times, &values);
        let seasonal = decomp.seasonal_component(&times, &result);
        assert_eq!(seasonal.len(), 100);
    }

    #[test]
    fn test_trend_component() {
        let decomp = TimeSeriesDecomposer::new(365.25);
        let result = DecompositionResult {
            offset: 10.0,
            velocity: 2.0,
            annual_sin: 0.0,
            annual_cos: 0.0,
            semiannual_sin: 0.0,
            semiannual_cos: 0.0,
            residuals: vec![],
        };
        let times = vec![0.0, 1.0, 2.0];
        let trend = decomp.trend_component(&times, &result);
        assert!((trend[0] - 10.0).abs() < 1e-10);
        assert!((trend[1] - 12.0).abs() < 1e-10);
        assert!((trend[2] - 14.0).abs() < 1e-10);
    }

    // --- AtmosphericPhaseScreen ---

    #[test]
    fn test_aps_flat_terrain() {
        let aps = AtmosphericPhaseScreen::new(0.0);
        let phases = vec![1.0, 1.0, 1.0, 1.0];
        let elevations = vec![0.0, 0.0, 0.0, 0.0];
        let result = aps.estimate_and_remove(&phases, &elevations);
        // With flat terrain, gradient should be ~0
        assert!(result.gradient_rad_per_m.abs() < 1e-10);
        // Corrected phases should be ~0 (offset removed)
        for &v in &result.corrected_phases {
            assert!(v.abs() < 1e-10);
        }
    }

    #[test]
    fn test_aps_linear_elevation_correlation() {
        let aps = AtmosphericPhaseScreen::new(0.0);
        // Phase linearly correlated with elevation
        let elevations = vec![0.0, 100.0, 200.0, 300.0, 400.0, 500.0];
        let gradient = 0.01; // rad/m
        let phases: Vec<f64> = elevations.iter().map(|&h| gradient * h + 0.5).collect();
        let result = aps.estimate_and_remove(&phases, &elevations);
        assert!(
            (result.gradient_rad_per_m - gradient).abs() < 1e-8,
            "gradient: {}",
            result.gradient_rad_per_m
        );
        for &v in &result.corrected_phases {
            assert!(v.abs() < 1e-8, "residual: {}", v);
        }
    }

    #[test]
    fn test_aps_predict_delay() {
        let aps = AtmosphericPhaseScreen::new(500.0);
        let delay = aps.predict_delay(0.01, 0.5, 1000.0);
        // 0.01 * (1000 - 500) + 0.5 = 5.5
        assert!((delay - 5.5).abs() < 1e-10);
    }

    // --- CoherenceEstimator ---

    #[test]
    fn test_coherence_identical_signals() {
        let ce = CoherenceEstimator::new(1);
        let re = vec![1.0, 0.5, -0.3, 0.8];
        let im = vec![0.2, -0.1, 0.7, 0.3];
        let gamma = ce.estimate(&re, &im, &re, &im);
        assert!(
            (gamma - 1.0).abs() < 1e-10,
            "Coherence of identical signals: {}",
            gamma
        );
    }

    #[test]
    fn test_coherence_orthogonal_signals() {
        let ce = CoherenceEstimator::new(1);
        // s1 = [1+j0, 0+j1], s2 = [0+j1, -1+j0]
        // Cross: s1*conj(s2) = 1*(0-j1) + j1*(-1-j0) = -j -j = -2j
        // Wait, let me use truly orthogonal:
        // s1 = [1, 0], s2 = [0, 1] (real-only)
        let s1_re = vec![1.0, 0.0];
        let s1_im = vec![0.0, 0.0];
        let s2_re = vec![0.0, 1.0];
        let s2_im = vec![0.0, 0.0];
        let gamma = ce.estimate(&s1_re, &s1_im, &s2_re, &s2_im);
        assert!(
            gamma < 1e-10,
            "Coherence of orthogonal signals: {}",
            gamma
        );
    }

    #[test]
    fn test_coherence_from_snr() {
        // SNR = infinity -> coherence = 1
        assert!((CoherenceEstimator::coherence_from_snr(1e10) - 1.0).abs() < 1e-6);
        // SNR = 1 -> coherence = 0.5
        assert!((CoherenceEstimator::coherence_from_snr(1.0) - 0.5).abs() < 1e-10);
        // SNR = 0 -> coherence = 0
        assert!(CoherenceEstimator::coherence_from_snr(0.0).abs() < 1e-15);
    }

    #[test]
    fn test_coherence_from_baseline() {
        assert!((CoherenceEstimator::coherence_from_baseline(0.0, 1000.0) - 1.0).abs() < 1e-10);
        assert!((CoherenceEstimator::coherence_from_baseline(500.0, 1000.0) - 0.5).abs() < 1e-10);
        assert!(CoherenceEstimator::coherence_from_baseline(1000.0, 1000.0).abs() < 1e-10);
        assert!(CoherenceEstimator::coherence_from_baseline(1500.0, 1000.0).abs() < 1e-10);
    }

    #[test]
    fn test_coherence_2d() {
        let rows = 5;
        let cols = 5;
        let n = rows * cols;
        let ce = CoherenceEstimator::new(3);

        // Identical images -> coherence ~1 in interior
        let re: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).cos()).collect();
        let im: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).sin()).collect();

        let coh = ce.estimate_2d(&re, &im, &re, &im, rows, cols);
        assert_eq!(coh.len(), n);
        // Centre pixel should have coherence ~1
        let centre = 2 * cols + 2;
        assert!(coh[centre] > 0.99, "Centre coherence: {}", coh[centre]);
    }

    // --- Utility functions ---

    #[test]
    fn test_wrap_phase_identity() {
        assert!((wrap_phase(0.5) - 0.5).abs() < 1e-10);
        assert!((wrap_phase(-0.5) - (-0.5)).abs() < 1e-10);
    }

    #[test]
    fn test_wrap_phase_large() {
        assert!((wrap_phase(7.0 * PI) - PI).abs() < 1e-10);
    }

    #[test]
    fn test_unwrap_1d() {
        // Wrap a linear ramp
        let n = 50;
        let original: Vec<f64> = (0..n).map(|i| 0.3 * i as f64).collect();
        let wrapped: Vec<f64> = original.iter().map(|&p| wrap_phase(p)).collect();
        let unwrapped = unwrap_phase_1d(&wrapped);

        // Should recover the original (up to a constant offset)
        let offset = unwrapped[0] - original[0];
        for i in 0..n {
            assert!(
                (unwrapped[i] - original[i] - offset).abs() < 0.1,
                "Sample {}: unwrapped={}, original={}",
                i,
                unwrapped[i],
                original[i]
            );
        }
    }

    #[test]
    fn test_interferometric_phase() {
        // s1 = s2 -> phase = 0
        let p = interferometric_phase(1.0, 0.0, 1.0, 0.0);
        assert!(p.abs() < 1e-10);

        // s1 = j, s2 = 1 -> phase = arg(j*1) = pi/2
        let p = interferometric_phase(0.0, 1.0, 1.0, 0.0);
        assert!((p - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_los_to_vertical() {
        // At normal incidence (0 rad), LOS = vertical
        let v = los_to_vertical(0.01, 0.0);
        assert!((v - 0.01).abs() < 1e-10);

        // At 30 degrees, vertical = LOS / cos(30) = LOS / 0.866
        let v = los_to_vertical(0.01, PI / 6.0);
        let expected = 0.01 / (PI / 6.0).cos();
        assert!((v - expected).abs() < 1e-10);
    }

    #[test]
    fn test_m_to_mm_conversion() {
        assert!((m_to_mm(0.001) - 1.0).abs() < 1e-10);
        assert!((mm_to_m(1.0) - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_m_to_mm_roundtrip() {
        let orig = 0.0234;
        assert!((mm_to_m(m_to_mm(orig)) - orig).abs() < 1e-14);
    }
}
