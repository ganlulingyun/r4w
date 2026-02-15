//! Acoustic well log processing for petroleum geophysics.
//!
//! This module implements sonic log interpretation algorithms used in oil and gas
//! exploration and production. It covers the full workflow from raw waveform
//! processing to synthetic seismogram generation:
//!
//! - **Slowness extraction** via cross-correlation of receiver waveforms
//! - **Porosity estimation** using Wyllie time-average and Raymer-Hunt-Gardner transforms
//! - **Cement bond analysis** from CBL amplitude for casing integrity evaluation
//! - **Synthetic seismogram generation** by convolving reflection coefficients with a wavelet
//! - **Lithology classification** from Vp/Vs crossplot regions
//!
//! # Key Equations
//!
//! **Wyllie Time-Average:**
//! ```text
//! 1/V = phi/V_f + (1 - phi)/V_ma
//! ```
//! where `V` is measured velocity, `V_f` is fluid velocity, `V_ma` is matrix velocity,
//! and `phi` is porosity.
//!
//! **Raymer-Hunt-Gardner:**
//! ```text
//! V = (1 - phi)^2 * V_ma + phi * V_f
//! ```
//!
//! **Gardner's Equation:**
//! ```text
//! rho = a * V^b     (typically a=0.31, b=0.25 for V in m/s)
//! ```
//!
//! **Reflection Coefficient:**
//! ```text
//! RC = (Z2 - Z1) / (Z2 + Z1)
//! ```
//! where `Z = rho * V` is acoustic impedance.
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_well_log_processor::{
//!     SonicLogConfig, SlownessProcessor, PorosityEstimator, PorosityModel,
//!     acoustic_impedance, reflection_coefficient,
//! };
//!
//! // Configure a sonic logging tool
//! let config = SonicLogConfig {
//!     transmitter_receiver_spacing_m: 0.9144,  // 3 ft
//!     receiver_spacing_m: 0.3048,               // 1 ft
//!     tool_frequency_hz: 20_000.0,
//!     mud_velocity_m_s: 1500.0,
//!     sample_rate_hz: 200_000.0,
//! };
//!
//! // Estimate porosity from sonic slowness
//! let estimator = PorosityEstimator::new(PorosityModel::WyllieTimeAverage {
//!     matrix_velocity_m_s: 5500.0,   // limestone
//!     fluid_velocity_m_s: 1500.0,
//! });
//!
//! let dt_us_per_ft = 70.0; // measured interval transit time
//! let porosity = estimator.porosity_from_slowness(dt_us_per_ft);
//! assert!(porosity > 0.0 && porosity < 1.0);
//!
//! // Compute acoustic impedance and reflection coefficient
//! let z1 = acoustic_impedance(2650.0, 4500.0); // limestone
//! let z2 = acoustic_impedance(2200.0, 2500.0); // sandstone
//! let rc = reflection_coefficient(z1, z2);
//! assert!(rc.abs() < 1.0);
//! ```

use std::f64::consts::PI;

// ─────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────

/// Configuration for a sonic logging tool.
///
/// Defines the physical geometry and operating parameters of the
/// downhole acoustic measurement system.
#[derive(Debug, Clone)]
pub struct SonicLogConfig {
    /// Distance from the transmitter to the near receiver (metres).
    pub transmitter_receiver_spacing_m: f64,
    /// Distance between the near and far receivers (metres).
    pub receiver_spacing_m: f64,
    /// Dominant tool frequency in Hz (typically 10–30 kHz).
    pub tool_frequency_hz: f64,
    /// Compressional-wave velocity of the borehole mud (m/s).
    pub mud_velocity_m_s: f64,
    /// Waveform digitisation rate in Hz.
    pub sample_rate_hz: f64,
}

impl Default for SonicLogConfig {
    fn default() -> Self {
        Self {
            transmitter_receiver_spacing_m: 0.9144, // 3 ft
            receiver_spacing_m: 0.3048,              // 1 ft
            tool_frequency_hz: 20_000.0,
            mud_velocity_m_s: 1500.0,
            sample_rate_hz: 200_000.0,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Slowness Processor
// ─────────────────────────────────────────────────────────────────────

/// Extracts interval transit time (delta-T / slowness) from raw acoustic waveforms
/// using cross-correlation of near and far receiver signals.
///
/// The processor finds the time delay between the near and far receivers,
/// then converts to slowness in microseconds per foot.
#[derive(Debug, Clone)]
pub struct SlownessProcessor {
    config: SonicLogConfig,
}

/// Result of a slowness measurement at one depth station.
#[derive(Debug, Clone)]
pub struct SlownessResult {
    /// Interval transit time in microseconds per foot.
    pub delta_t_us_per_ft: f64,
    /// Formation compressional velocity in m/s.
    pub velocity_m_s: f64,
    /// Cross-correlation coefficient at the best lag (0.0–1.0).
    pub correlation_quality: f64,
    /// Best lag in samples.
    pub lag_samples: i32,
}

impl SlownessProcessor {
    /// Create a new processor from the given tool configuration.
    pub fn new(config: SonicLogConfig) -> Self {
        Self { config }
    }

    /// Compute cross-correlation between two waveforms over a range of lags.
    ///
    /// Returns a vector of (lag, correlation_value) pairs. The lag range is
    /// `[-max_lag, max_lag]` in samples.
    pub fn cross_correlate(a: &[f64], b: &[f64], max_lag: usize) -> Vec<(i32, f64)> {
        let n = a.len().min(b.len());
        if n == 0 {
            return Vec::new();
        }

        // Compute energies for normalisation
        let energy_a: f64 = a.iter().take(n).map(|x| x * x).sum();
        let energy_b: f64 = b.iter().take(n).map(|x| x * x).sum();
        let norm = (energy_a * energy_b).sqrt();
        if norm < 1e-30 {
            return Vec::new();
        }

        let mut results = Vec::with_capacity(2 * max_lag + 1);
        let max_lag_i = max_lag as i32;

        for lag in -max_lag_i..=max_lag_i {
            let mut sum = 0.0;
            for i in 0..n {
                let j = i as i32 + lag;
                if j >= 0 && (j as usize) < n {
                    sum += a[i] * b[j as usize];
                }
            }
            results.push((lag, sum / norm));
        }

        results
    }

    /// Extract slowness from near and far receiver waveforms.
    ///
    /// The waveforms should be time-aligned to the transmitter pulse. The
    /// processor cross-correlates them to find the differential travel time,
    /// then converts to interval transit time.
    pub fn extract_slowness(&self, near_waveform: &[f64], far_waveform: &[f64]) -> SlownessResult {
        // Maximum expected lag: receiver spacing / min formation velocity * sample rate
        // Use mud velocity as the slow bound
        let max_time_s = self.config.receiver_spacing_m / (self.config.mud_velocity_m_s * 0.5);
        let max_lag = (max_time_s * self.config.sample_rate_hz) as usize;
        let max_lag = max_lag.max(1);

        let corr = Self::cross_correlate(near_waveform, far_waveform, max_lag);

        // Find peak correlation (positive lags only — far receiver arrives later)
        let mut best_lag = 0i32;
        let mut best_val = f64::NEG_INFINITY;
        for &(lag, val) in &corr {
            if lag >= 0 && val > best_val {
                best_val = val;
                best_lag = lag;
            }
        }

        // Convert lag to time delay
        let dt_s = best_lag as f64 / self.config.sample_rate_hz;

        // Velocity = spacing / dt
        let velocity = if dt_s > 0.0 {
            self.config.receiver_spacing_m / dt_s
        } else {
            self.config.mud_velocity_m_s // fallback
        };

        // Convert to microseconds per foot: dt_us_per_ft = 1e6 / (velocity * 3.28084)
        let feet_per_m = 3.28084;
        let delta_t_us_per_ft = if velocity > 0.0 {
            1e6 / (velocity * feet_per_m)
        } else {
            0.0
        };

        SlownessResult {
            delta_t_us_per_ft,
            velocity_m_s: velocity,
            correlation_quality: best_val.max(0.0).min(1.0),
            lag_samples: best_lag,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Porosity Estimator
// ─────────────────────────────────────────────────────────────────────

/// Porosity transform model selection.
#[derive(Debug, Clone)]
pub enum PorosityModel {
    /// Wyllie time-average equation (1956).
    ///
    /// ```text
    /// phi = (dt - dt_ma) / (dt_f - dt_ma)
    /// ```
    WyllieTimeAverage {
        /// Matrix (grain) compressional velocity (m/s).
        matrix_velocity_m_s: f64,
        /// Pore-fluid compressional velocity (m/s).
        fluid_velocity_m_s: f64,
    },
    /// Raymer-Hunt-Gardner transform (1980).
    ///
    /// ```text
    /// V = (1 - phi)^2 * V_ma + phi * V_f
    /// ```
    /// Solved iteratively for porosity.
    RaymerHuntGardner {
        /// Matrix velocity (m/s).
        matrix_velocity_m_s: f64,
        /// Fluid velocity (m/s).
        fluid_velocity_m_s: f64,
    },
}

/// Estimates formation porosity from sonic log data.
#[derive(Debug, Clone)]
pub struct PorosityEstimator {
    model: PorosityModel,
}

impl PorosityEstimator {
    /// Create a new estimator with the specified transform model.
    pub fn new(model: PorosityModel) -> Self {
        Self { model }
    }

    /// Estimate porosity from interval transit time in microseconds per foot.
    ///
    /// Returns a value clamped to \[0.0, 1.0\].
    pub fn porosity_from_slowness(&self, dt_us_per_ft: f64) -> f64 {
        let phi = match &self.model {
            PorosityModel::WyllieTimeAverage {
                matrix_velocity_m_s,
                fluid_velocity_m_s,
            } => {
                let feet_per_m = 3.28084;
                let dt_ma = 1e6 / (matrix_velocity_m_s * feet_per_m);
                let dt_f = 1e6 / (fluid_velocity_m_s * feet_per_m);
                if (dt_f - dt_ma).abs() < 1e-12 {
                    0.0
                } else {
                    (dt_us_per_ft - dt_ma) / (dt_f - dt_ma)
                }
            }
            PorosityModel::RaymerHuntGardner {
                matrix_velocity_m_s,
                fluid_velocity_m_s,
            } => {
                let feet_per_m = 3.28084;
                let v_measured = 1e6 / (dt_us_per_ft * feet_per_m);
                // V = (1-phi)^2 * V_ma + phi * V_f
                // Solve quadratic: V_ma * phi^2 - (2*V_ma + V_f)*phi + (V_ma - V) = 0
                // But more robust: iterative Newton
                let v_ma = *matrix_velocity_m_s;
                let v_f = *fluid_velocity_m_s;
                Self::solve_raymer(v_measured, v_ma, v_f)
            }
        };

        phi.max(0.0).min(1.0)
    }

    /// Estimate porosity from velocity in m/s.
    pub fn porosity_from_velocity(&self, velocity_m_s: f64) -> f64 {
        let feet_per_m = 3.28084;
        let dt = 1e6 / (velocity_m_s * feet_per_m);
        self.porosity_from_slowness(dt)
    }

    /// Solve the Raymer-Hunt-Gardner equation for porosity using Newton's method.
    fn solve_raymer(v_measured: f64, v_ma: f64, v_f: f64) -> f64 {
        // V(phi) = (1-phi)^2 * V_ma + phi * V_f
        // f(phi) = (1-phi)^2 * V_ma + phi * V_f - V_measured = 0
        // f'(phi) = -2*(1-phi)*V_ma + V_f
        let mut phi: f64 = 0.1; // initial guess
        for _ in 0..50 {
            let f = (1.0 - phi).powi(2) * v_ma + phi * v_f - v_measured;
            let fp = -2.0 * (1.0 - phi) * v_ma + v_f;
            if fp.abs() < 1e-30 {
                break;
            }
            let dphi = f / fp;
            phi -= dphi;
            phi = phi.max(0.0).min(1.0);
            if dphi.abs() < 1e-12 {
                break;
            }
        }
        phi
    }
}

// ─────────────────────────────────────────────────────────────────────
// Cement Bond Analyzer
// ─────────────────────────────────────────────────────────────────────

/// Cement bond quality classification from CBL amplitude analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CementBondGrade {
    /// Good cement bond — amplitude below free-pipe threshold.
    Good,
    /// Partial bond — intermediate amplitude.
    Partial,
    /// Free pipe — high amplitude indicating no cement.
    FreePipe,
}

/// Result of cement bond analysis at one depth.
#[derive(Debug, Clone)]
pub struct CementBondResult {
    /// CBL amplitude in millivolts.
    pub amplitude_mv: f64,
    /// Bond index (0.0 = free pipe, 1.0 = perfect bond).
    pub bond_index: f64,
    /// Qualitative grade.
    pub grade: CementBondGrade,
}

/// Analyses Cement Bond Log (CBL) amplitude data for casing integrity evaluation.
///
/// The CBL measures the amplitude of the first compressional arrival through
/// the casing. High amplitudes indicate free pipe (no cement), while low
/// amplitudes indicate good cement bond.
#[derive(Debug, Clone)]
pub struct CementBondAnalyzer {
    /// Free-pipe amplitude reference in millivolts (100% free pipe).
    free_pipe_amplitude_mv: f64,
    /// Good-bond threshold in millivolts (below this = good bond).
    good_bond_threshold_mv: f64,
}

impl CementBondAnalyzer {
    /// Create a new analyser with calibration values.
    ///
    /// # Arguments
    /// * `free_pipe_amplitude_mv` - Amplitude in millivolts at a known free-pipe section.
    /// * `good_bond_threshold_mv` - Amplitude threshold below which bond is considered good.
    pub fn new(free_pipe_amplitude_mv: f64, good_bond_threshold_mv: f64) -> Self {
        Self {
            free_pipe_amplitude_mv,
            good_bond_threshold_mv,
        }
    }

    /// Create with typical values for a standard 7-inch casing.
    pub fn default_7_inch() -> Self {
        Self {
            free_pipe_amplitude_mv: 80.0,
            good_bond_threshold_mv: 10.0,
        }
    }

    /// Analyse a single CBL amplitude reading.
    pub fn analyze(&self, amplitude_mv: f64) -> CementBondResult {
        let bond_index = if self.free_pipe_amplitude_mv > 0.0 {
            (1.0 - amplitude_mv / self.free_pipe_amplitude_mv).max(0.0).min(1.0)
        } else {
            0.0
        };

        let grade = if amplitude_mv <= self.good_bond_threshold_mv {
            CementBondGrade::Good
        } else if amplitude_mv >= self.free_pipe_amplitude_mv * 0.8 {
            CementBondGrade::FreePipe
        } else {
            CementBondGrade::Partial
        };

        CementBondResult {
            amplitude_mv,
            bond_index,
            grade,
        }
    }

    /// Analyse a log of CBL amplitudes, returning results for each depth station.
    pub fn analyze_log(&self, amplitudes_mv: &[f64]) -> Vec<CementBondResult> {
        amplitudes_mv.iter().map(|&a| self.analyze(a)).collect()
    }

    /// Compute the fraction of the logged interval with good bond.
    pub fn good_bond_fraction(&self, amplitudes_mv: &[f64]) -> f64 {
        if amplitudes_mv.is_empty() {
            return 0.0;
        }
        let good_count = amplitudes_mv
            .iter()
            .filter(|&&a| a <= self.good_bond_threshold_mv)
            .count();
        good_count as f64 / amplitudes_mv.len() as f64
    }

    /// Extract an amplitude envelope from raw waveform data.
    ///
    /// Takes the maximum absolute value within the expected first-arrival
    /// window (defined by `start_sample` and `window_length`).
    pub fn extract_amplitude(
        waveform: &[f64],
        start_sample: usize,
        window_length: usize,
    ) -> f64 {
        waveform
            .iter()
            .skip(start_sample)
            .take(window_length)
            .map(|x| x.abs())
            .fold(0.0f64, f64::max)
    }
}

// ─────────────────────────────────────────────────────────────────────
// Synthetic Seismogram Generator
// ─────────────────────────────────────────────────────────────────────

/// Generates synthetic seismograms from acoustic impedance logs.
///
/// The process:
/// 1. Compute acoustic impedance `Z = rho * V` at each depth sample.
/// 2. Derive reflection coefficient series from impedance contrasts.
/// 3. Convert from depth to two-way time domain.
/// 4. Convolve with a seismic source wavelet (Ricker or user-supplied).
#[derive(Debug, Clone)]
pub struct SyntheticSeismogramGenerator {
    /// Sample interval in two-way time (seconds).
    pub time_sample_interval_s: f64,
}

/// A synthetic seismogram result.
#[derive(Debug, Clone)]
pub struct SyntheticSeismogram {
    /// Two-way time values in seconds.
    pub time_s: Vec<f64>,
    /// Seismogram amplitude values.
    pub amplitude: Vec<f64>,
    /// Reflection coefficient series (before convolution).
    pub reflection_coefficients: Vec<f64>,
}

impl SyntheticSeismogramGenerator {
    /// Create a new generator with the specified output time sample interval.
    pub fn new(time_sample_interval_s: f64) -> Self {
        Self {
            time_sample_interval_s,
        }
    }

    /// Compute reflection coefficients from a velocity and density log.
    ///
    /// Each element in the returned vector corresponds to the interface between
    /// consecutive depth samples.
    pub fn compute_reflection_coefficients(
        velocities_m_s: &[f64],
        densities_kg_m3: &[f64],
    ) -> Vec<f64> {
        let n = velocities_m_s.len().min(densities_kg_m3.len());
        if n < 2 {
            return Vec::new();
        }

        let mut rcs = Vec::with_capacity(n - 1);
        for i in 0..n - 1 {
            let z1 = velocities_m_s[i] * densities_kg_m3[i];
            let z2 = velocities_m_s[i + 1] * densities_kg_m3[i + 1];
            rcs.push(reflection_coefficient(z1, z2));
        }
        rcs
    }

    /// Generate a Ricker (Mexican hat) wavelet.
    ///
    /// # Arguments
    /// * `peak_frequency_hz` - Dominant frequency of the wavelet.
    /// * `sample_rate_hz` - Sample rate of the output wavelet.
    /// * `duration_s` - Total duration (centred at t=0).
    pub fn ricker_wavelet(peak_frequency_hz: f64, sample_rate_hz: f64, duration_s: f64) -> Vec<f64> {
        let n = (duration_s * sample_rate_hz) as usize;
        let n = if n % 2 == 0 { n + 1 } else { n }; // ensure odd length for symmetry
        let centre = n / 2;
        let mut wavelet = Vec::with_capacity(n);

        for i in 0..n {
            let t = (i as f64 - centre as f64) / sample_rate_hz;
            let u = (PI * peak_frequency_hz * t).powi(2);
            let w = (1.0 - 2.0 * u) * (-u).exp();
            wavelet.push(w);
        }
        wavelet
    }

    /// Generate a synthetic seismogram by convolving a reflection coefficient
    /// series with a source wavelet.
    ///
    /// # Arguments
    /// * `velocities_m_s` - Compressional velocity log (one per depth sample).
    /// * `densities_kg_m3` - Bulk density log (one per depth sample).
    /// * `depth_interval_m` - Spacing between depth samples in metres.
    /// * `wavelet` - Source wavelet to convolve with the RC series.
    pub fn generate(
        &self,
        velocities_m_s: &[f64],
        densities_kg_m3: &[f64],
        depth_interval_m: f64,
        wavelet: &[f64],
    ) -> SyntheticSeismogram {
        let rcs = Self::compute_reflection_coefficients(velocities_m_s, densities_kg_m3);

        // Compute two-way travel times for each interface
        let n = velocities_m_s.len().min(densities_kg_m3.len());
        let mut twt = Vec::with_capacity(n);
        let mut cumulative_time = 0.0;
        twt.push(0.0);
        for i in 0..n - 1 {
            // Two-way time through one depth interval
            let dt = 2.0 * depth_interval_m / velocities_m_s[i];
            cumulative_time += dt;
            twt.push(cumulative_time);
        }

        // Resample RC series to uniform time sampling
        let total_time = cumulative_time;
        let num_time_samples = if self.time_sample_interval_s > 0.0 {
            (total_time / self.time_sample_interval_s) as usize + 1
        } else {
            rcs.len()
        };

        let mut rc_resampled = vec![0.0; num_time_samples];
        let mut time_axis = Vec::with_capacity(num_time_samples);
        for i in 0..num_time_samples {
            let t = i as f64 * self.time_sample_interval_s;
            time_axis.push(t);

            // Find nearest RC
            for j in 0..rcs.len() {
                if (twt[j] - t).abs() < self.time_sample_interval_s * 0.5 {
                    rc_resampled[i] += rcs[j];
                }
            }
        }

        // Convolve with wavelet
        let trace = convolve(&rc_resampled, wavelet);

        // Trim to original length
        let half_w = wavelet.len() / 2;
        let output: Vec<f64> = trace
            .iter()
            .skip(half_w)
            .take(num_time_samples)
            .copied()
            .collect();

        SyntheticSeismogram {
            time_s: time_axis,
            amplitude: output,
            reflection_coefficients: rcs,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Lithology Classifier
// ─────────────────────────────────────────────────────────────────────

/// Lithology classification from Vp/Vs ratio and Vp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lithology {
    /// Sandstone (quartz-rich clastic).
    Sandstone,
    /// Limestone (calcite-dominated carbonate).
    Limestone,
    /// Dolomite (dolomite mineral carbonate).
    Dolomite,
    /// Shale (clay-rich, fine-grained).
    Shale,
    /// Anhydrite (evaporite mineral).
    Anhydrite,
    /// Salt (halite).
    Salt,
    /// Coal.
    Coal,
    /// Unknown / unclassified.
    Unknown,
}

/// Classification result with confidence.
#[derive(Debug, Clone)]
pub struct LithologyResult {
    /// Classified lithology.
    pub lithology: Lithology,
    /// Confidence (0.0–1.0) based on distance from crossplot centroid.
    pub confidence: f64,
    /// Input Vp in m/s.
    pub vp_m_s: f64,
    /// Input Vs in m/s.
    pub vs_m_s: f64,
    /// Computed Vp/Vs ratio.
    pub vp_vs_ratio: f64,
}

/// Classifies formation lithology from compressional (Vp) and shear (Vs) velocities.
///
/// Uses regions on the Vp/Vs crossplot, following Castagna et al. (1985)
/// and Pickett (1963) empirical relationships.
#[derive(Debug, Clone)]
pub struct LithologyClassifier {
    // Crossplot region centroids: (vp, vp_vs_ratio, lithology)
    regions: Vec<(f64, f64, Lithology)>,
}

impl Default for LithologyClassifier {
    fn default() -> Self {
        Self::new()
    }
}

impl LithologyClassifier {
    /// Create a classifier with standard crossplot regions.
    pub fn new() -> Self {
        // Typical centroids from Castagna et al. and other references.
        // (Vp in m/s, Vp/Vs ratio, lithology)
        let regions = vec![
            (4000.0, 1.65, Lithology::Sandstone),
            (5500.0, 1.90, Lithology::Limestone),
            (6000.0, 1.80, Lithology::Dolomite),
            (3200.0, 2.50, Lithology::Shale),
            (6100.0, 1.75, Lithology::Anhydrite),
            (4500.0, 1.73, Lithology::Salt),
            (2200.0, 3.00, Lithology::Coal),
        ];
        Self { regions }
    }

    /// Classify a formation from Vp and Vs measurements.
    pub fn classify(&self, vp_m_s: f64, vs_m_s: f64) -> LithologyResult {
        let vp_vs = if vs_m_s > 0.0 {
            vp_m_s / vs_m_s
        } else {
            f64::INFINITY
        };

        let mut best_dist = f64::INFINITY;
        let mut best_litho = Lithology::Unknown;

        for &(ref_vp, ref_ratio, litho) in &self.regions {
            // Normalised Euclidean distance in (Vp, Vp/Vs) space
            let dvp = (vp_m_s - ref_vp) / 1000.0; // scale Vp to km/s range
            let dr = vp_vs - ref_ratio;
            let dist = (dvp * dvp + dr * dr).sqrt();
            if dist < best_dist {
                best_dist = dist;
                best_litho = litho;
            }
        }

        // Confidence decreases with distance from centroid
        let confidence = (-best_dist / 2.0).exp().max(0.0).min(1.0);

        LithologyResult {
            lithology: best_litho,
            confidence,
            vp_m_s,
            vs_m_s,
            vp_vs_ratio: vp_vs,
        }
    }

    /// Classify a log of Vp/Vs measurements.
    pub fn classify_log(&self, vp: &[f64], vs: &[f64]) -> Vec<LithologyResult> {
        let n = vp.len().min(vs.len());
        (0..n).map(|i| self.classify(vp[i], vs[i])).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────
// Helper Functions
// ─────────────────────────────────────────────────────────────────────

/// Compute acoustic impedance Z = rho * V.
///
/// # Arguments
/// * `density_kg_m3` - Bulk density in kg/m^3.
/// * `velocity_m_s` - Compressional velocity in m/s.
///
/// # Returns
/// Acoustic impedance in kg/(m^2 * s) = rayls.
pub fn acoustic_impedance(density_kg_m3: f64, velocity_m_s: f64) -> f64 {
    density_kg_m3 * velocity_m_s
}

/// Compute the normal-incidence reflection coefficient at an interface.
///
/// ```text
/// RC = (Z2 - Z1) / (Z2 + Z1)
/// ```
///
/// Returns a value in \[-1.0, 1.0\]. Positive means impedance increases downward.
pub fn reflection_coefficient(z_upper: f64, z_lower: f64) -> f64 {
    let sum = z_upper + z_lower;
    if sum.abs() < 1e-30 {
        0.0
    } else {
        (z_lower - z_upper) / sum
    }
}

/// Convert a depth log to two-way travel time.
///
/// # Arguments
/// * `depths_m` - Depth values in metres (monotonically increasing).
/// * `velocities_m_s` - Interval velocity at each depth sample.
///
/// # Returns
/// Two-way travel times in seconds corresponding to each depth.
pub fn time_to_depth(depths_m: &[f64], velocities_m_s: &[f64]) -> Vec<f64> {
    let n = depths_m.len().min(velocities_m_s.len());
    if n == 0 {
        return Vec::new();
    }

    let mut twt = Vec::with_capacity(n);
    twt.push(0.0);

    for i in 1..n {
        let dz = depths_m[i] - depths_m[i - 1];
        let v = velocities_m_s[i - 1];
        let dt = if v > 0.0 { 2.0 * dz / v } else { 0.0 };
        twt.push(twt[i - 1] + dt);
    }

    twt
}

/// Estimate bulk density from compressional velocity using Gardner's equation.
///
/// ```text
/// rho = a * V^b
/// ```
///
/// Default coefficients (a=0.31, b=0.25) are for velocity in m/s and density
/// in g/cm^3. The result is returned in kg/m^3.
///
/// # Arguments
/// * `velocity_m_s` - Compressional velocity in m/s.
///
/// # Returns
/// Bulk density in kg/m^3.
pub fn gardner_density(velocity_m_s: f64) -> f64 {
    // Gardner's empirical relationship (Gardner et al., 1974)
    // rho (g/cm3) = 0.31 * V(m/s)^0.25
    let a = 0.31;
    let b = 0.25;
    let rho_gcc = a * velocity_m_s.powf(b);
    rho_gcc * 1000.0 // convert g/cm3 to kg/m3
}

/// Gardner's equation with custom coefficients.
///
/// # Arguments
/// * `velocity_m_s` - Compressional velocity in m/s.
/// * `a` - Multiplicative constant.
/// * `b` - Exponent.
///
/// # Returns
/// Bulk density in g/cm^3.
pub fn gardner_density_custom(velocity_m_s: f64, a: f64, b: f64) -> f64 {
    a * velocity_m_s.powf(b)
}

/// Convert slowness (microseconds per foot) to velocity (m/s).
pub fn slowness_to_velocity(dt_us_per_ft: f64) -> f64 {
    if dt_us_per_ft <= 0.0 {
        return 0.0;
    }
    let feet_per_m = 3.28084;
    1e6 / (dt_us_per_ft * feet_per_m)
}

/// Convert velocity (m/s) to slowness (microseconds per foot).
pub fn velocity_to_slowness(velocity_m_s: f64) -> f64 {
    if velocity_m_s <= 0.0 {
        return 0.0;
    }
    let feet_per_m = 3.28084;
    1e6 / (velocity_m_s * feet_per_m)
}

/// Simple linear convolution of two signals.
fn convolve(a: &[f64], b: &[f64]) -> Vec<f64> {
    if a.is_empty() || b.is_empty() {
        return Vec::new();
    }
    let out_len = a.len() + b.len() - 1;
    let mut result = vec![0.0; out_len];
    for (i, &av) in a.iter().enumerate() {
        for (j, &bv) in b.iter().enumerate() {
            result[i + j] += av * bv;
        }
    }
    result
}

// ─────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper ──

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── SonicLogConfig ──

    #[test]
    fn test_default_config() {
        let cfg = SonicLogConfig::default();
        assert!(approx_eq(cfg.transmitter_receiver_spacing_m, 0.9144, 0.001));
        assert!(approx_eq(cfg.receiver_spacing_m, 0.3048, 0.001));
        assert!(approx_eq(cfg.tool_frequency_hz, 20_000.0, 1.0));
    }

    // ── SlownessProcessor ──

    #[test]
    fn test_cross_correlate_identical() {
        let sig = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let corr = SlownessProcessor::cross_correlate(&sig, &sig, 2);
        // Peak should be at lag 0
        let peak = corr.iter().max_by(|a, b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
        assert_eq!(peak.0, 0);
        assert!(peak.1 > 0.99);
    }

    #[test]
    fn test_cross_correlate_shifted() {
        // Near receiver: impulse at sample 5
        let mut near = vec![0.0; 20];
        near[5] = 1.0;
        // Far receiver: impulse at sample 8 (3-sample delay)
        let mut far = vec![0.0; 20];
        far[8] = 1.0;

        let corr = SlownessProcessor::cross_correlate(&near, &far, 10);
        let peak = corr
            .iter()
            .filter(|(lag, _)| *lag >= 0)
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        assert_eq!(peak.0, 3);
    }

    #[test]
    fn test_cross_correlate_empty() {
        let result = SlownessProcessor::cross_correlate(&[], &[], 5);
        assert!(result.is_empty());
    }

    #[test]
    fn test_extract_slowness_known_delay() {
        let config = SonicLogConfig {
            receiver_spacing_m: 0.3048,
            sample_rate_hz: 100_000.0,
            transmitter_receiver_spacing_m: 0.9144,
            tool_frequency_hz: 20_000.0,
            mud_velocity_m_s: 1500.0,
        };
        let proc = SlownessProcessor::new(config);

        // Create waveforms with known delay (5 samples = 50 us)
        let mut near = vec![0.0; 100];
        let mut far = vec![0.0; 100];
        for i in 20..30 {
            near[i] = (2.0 * PI * (i as f64) / 10.0).sin();
        }
        for i in 25..35 {
            far[i] = (2.0 * PI * ((i - 5) as f64) / 10.0).sin();
        }

        let result = proc.extract_slowness(&near, &far);
        assert!(result.velocity_m_s > 0.0);
        assert!(result.delta_t_us_per_ft > 0.0);
        assert_eq!(result.lag_samples, 5);
    }

    // ── PorosityEstimator ──

    #[test]
    fn test_wyllie_porosity_zero() {
        // If dt equals matrix dt, porosity should be ~0
        let est = PorosityEstimator::new(PorosityModel::WyllieTimeAverage {
            matrix_velocity_m_s: 5500.0,
            fluid_velocity_m_s: 1500.0,
        });
        let dt_matrix = velocity_to_slowness(5500.0);
        let phi = est.porosity_from_slowness(dt_matrix);
        assert!(approx_eq(phi, 0.0, 0.01));
    }

    #[test]
    fn test_wyllie_porosity_full() {
        // If dt equals fluid dt, porosity should be ~1
        let est = PorosityEstimator::new(PorosityModel::WyllieTimeAverage {
            matrix_velocity_m_s: 5500.0,
            fluid_velocity_m_s: 1500.0,
        });
        let dt_fluid = velocity_to_slowness(1500.0);
        let phi = est.porosity_from_slowness(dt_fluid);
        assert!(approx_eq(phi, 1.0, 0.01));
    }

    #[test]
    fn test_wyllie_typical_sandstone() {
        // Typical sandstone: dt ~ 85 us/ft => moderate porosity
        let est = PorosityEstimator::new(PorosityModel::WyllieTimeAverage {
            matrix_velocity_m_s: 5486.0, // ~55.5 us/ft sandstone
            fluid_velocity_m_s: 1524.0,  // ~200 us/ft water
        });
        let phi = est.porosity_from_slowness(85.0);
        assert!(phi > 0.15 && phi < 0.35, "Sandstone porosity ~20-30%, got {}", phi);
    }

    #[test]
    fn test_raymer_porosity_zero() {
        let est = PorosityEstimator::new(PorosityModel::RaymerHuntGardner {
            matrix_velocity_m_s: 5500.0,
            fluid_velocity_m_s: 1500.0,
        });
        let dt_matrix = velocity_to_slowness(5500.0);
        let phi = est.porosity_from_slowness(dt_matrix);
        assert!(approx_eq(phi, 0.0, 0.02));
    }

    #[test]
    fn test_raymer_porosity_moderate() {
        let est = PorosityEstimator::new(PorosityModel::RaymerHuntGardner {
            matrix_velocity_m_s: 5500.0,
            fluid_velocity_m_s: 1500.0,
        });
        // For phi=0.2: V = 0.64*5500 + 0.2*1500 = 3520 + 300 = 3820
        let dt = velocity_to_slowness(3820.0);
        let phi = est.porosity_from_slowness(dt);
        assert!(approx_eq(phi, 0.2, 0.03), "Expected ~0.2, got {}", phi);
    }

    #[test]
    fn test_porosity_from_velocity() {
        let est = PorosityEstimator::new(PorosityModel::WyllieTimeAverage {
            matrix_velocity_m_s: 5500.0,
            fluid_velocity_m_s: 1500.0,
        });
        let phi = est.porosity_from_velocity(5500.0);
        assert!(approx_eq(phi, 0.0, 0.01));
    }

    // ── CementBondAnalyzer ──

    #[test]
    fn test_cbl_good_bond() {
        let cba = CementBondAnalyzer::new(80.0, 10.0);
        let result = cba.analyze(5.0);
        assert_eq!(result.grade, CementBondGrade::Good);
        assert!(result.bond_index > 0.9);
    }

    #[test]
    fn test_cbl_free_pipe() {
        let cba = CementBondAnalyzer::new(80.0, 10.0);
        let result = cba.analyze(78.0);
        assert_eq!(result.grade, CementBondGrade::FreePipe);
        assert!(result.bond_index < 0.1);
    }

    #[test]
    fn test_cbl_partial_bond() {
        let cba = CementBondAnalyzer::new(80.0, 10.0);
        let result = cba.analyze(40.0);
        assert_eq!(result.grade, CementBondGrade::Partial);
    }

    #[test]
    fn test_cbl_log_analysis() {
        let cba = CementBondAnalyzer::new(80.0, 10.0);
        let log = vec![5.0, 8.0, 40.0, 75.0, 3.0];
        let results = cba.analyze_log(&log);
        assert_eq!(results.len(), 5);
        assert_eq!(results[0].grade, CementBondGrade::Good);
        assert_eq!(results[3].grade, CementBondGrade::FreePipe);
    }

    #[test]
    fn test_good_bond_fraction() {
        let cba = CementBondAnalyzer::new(80.0, 10.0);
        let log = vec![5.0, 8.0, 40.0, 75.0, 3.0];
        let frac = cba.good_bond_fraction(&log);
        assert!(approx_eq(frac, 3.0 / 5.0, 0.01));
    }

    #[test]
    fn test_extract_amplitude() {
        let waveform = vec![0.1, -0.2, 0.8, -0.5, 0.3, 0.1];
        let amp = CementBondAnalyzer::extract_amplitude(&waveform, 1, 3);
        assert!(approx_eq(amp, 0.8, 0.001));
    }

    #[test]
    fn test_cbl_default_7_inch() {
        let cba = CementBondAnalyzer::default_7_inch();
        assert!(approx_eq(cba.free_pipe_amplitude_mv, 80.0, 0.1));
        assert!(approx_eq(cba.good_bond_threshold_mv, 10.0, 0.1));
    }

    // ── SyntheticSeismogramGenerator ──

    #[test]
    fn test_ricker_wavelet_peak() {
        let wavelet = SyntheticSeismogramGenerator::ricker_wavelet(30.0, 1000.0, 0.1);
        assert!(!wavelet.is_empty());
        // Peak should be at center and equal to 1.0
        let centre = wavelet.len() / 2;
        assert!(approx_eq(wavelet[centre], 1.0, 0.01));
    }

    #[test]
    fn test_ricker_wavelet_symmetry() {
        let wavelet = SyntheticSeismogramGenerator::ricker_wavelet(25.0, 1000.0, 0.08);
        let n = wavelet.len();
        for i in 0..n / 2 {
            assert!(
                approx_eq(wavelet[i], wavelet[n - 1 - i], 1e-10),
                "Asymmetry at index {}",
                i
            );
        }
    }

    #[test]
    fn test_reflection_coefficients_single_interface() {
        let v = vec![3000.0, 5000.0];
        let d = vec![2200.0, 2650.0];
        let rcs = SyntheticSeismogramGenerator::compute_reflection_coefficients(&v, &d);
        assert_eq!(rcs.len(), 1);
        let expected = reflection_coefficient(3000.0 * 2200.0, 5000.0 * 2650.0);
        assert!(approx_eq(rcs[0], expected, 1e-10));
    }

    #[test]
    fn test_synthetic_seismogram_generation() {
        // Simple two-layer model
        let v = vec![3000.0; 50];
        let mut d = vec![2200.0; 50];
        // Insert impedance contrast at depth 25
        for i in 25..50 {
            d[i] = 2650.0;
        }
        let depth_interval = 0.5; // metres

        let gen = SyntheticSeismogramGenerator::new(0.001); // 1 ms
        let wavelet = SyntheticSeismogramGenerator::ricker_wavelet(30.0, 1000.0, 0.1);
        let result = gen.generate(&v, &d, depth_interval, &wavelet);

        assert!(!result.amplitude.is_empty());
        assert!(!result.reflection_coefficients.is_empty());
        assert!(!result.time_s.is_empty());
    }

    // ── LithologyClassifier ──

    #[test]
    fn test_classify_sandstone() {
        let clf = LithologyClassifier::new();
        let result = clf.classify(4000.0, 2400.0); // Vp/Vs ~ 1.67
        assert_eq!(result.lithology, Lithology::Sandstone);
        assert!(result.confidence > 0.5);
    }

    #[test]
    fn test_classify_limestone() {
        let clf = LithologyClassifier::new();
        let result = clf.classify(5500.0, 2900.0); // Vp/Vs ~ 1.90
        assert_eq!(result.lithology, Lithology::Limestone);
    }

    #[test]
    fn test_classify_shale() {
        let clf = LithologyClassifier::new();
        let result = clf.classify(3200.0, 1280.0); // Vp/Vs ~ 2.5
        assert_eq!(result.lithology, Lithology::Shale);
    }

    #[test]
    fn test_classify_log() {
        let clf = LithologyClassifier::new();
        let vp = vec![4000.0, 5500.0, 3200.0];
        let vs = vec![2400.0, 2900.0, 1280.0];
        let results = clf.classify_log(&vp, &vs);
        assert_eq!(results.len(), 3);
        assert_eq!(results[0].lithology, Lithology::Sandstone);
        assert_eq!(results[1].lithology, Lithology::Limestone);
        assert_eq!(results[2].lithology, Lithology::Shale);
    }

    // ── Helper Functions ──

    #[test]
    fn test_acoustic_impedance() {
        let z = acoustic_impedance(2650.0, 5000.0);
        assert!(approx_eq(z, 13_250_000.0, 1.0));
    }

    #[test]
    fn test_reflection_coefficient_same() {
        let rc = reflection_coefficient(1000.0, 1000.0);
        assert!(approx_eq(rc, 0.0, 1e-10));
    }

    #[test]
    fn test_reflection_coefficient_hard_over_soft() {
        let rc = reflection_coefficient(5_000_000.0, 15_000_000.0);
        assert!(rc > 0.0); // impedance increase downward
        assert!(rc < 1.0);
    }

    #[test]
    fn test_reflection_coefficient_soft_over_hard() {
        let rc = reflection_coefficient(15_000_000.0, 5_000_000.0);
        assert!(rc < 0.0); // impedance decrease downward
    }

    #[test]
    fn test_time_to_depth_conversion() {
        let depths = vec![0.0, 100.0, 200.0, 300.0];
        let velocities = vec![2000.0, 3000.0, 4000.0, 5000.0];
        let twt = time_to_depth(&depths, &velocities);
        assert_eq!(twt.len(), 4);
        assert!(approx_eq(twt[0], 0.0, 1e-10));
        // First interval: 2 * 100m / 2000 m/s = 0.1 s
        assert!(approx_eq(twt[1], 0.1, 1e-10));
    }

    #[test]
    fn test_time_to_depth_empty() {
        let twt = time_to_depth(&[], &[]);
        assert!(twt.is_empty());
    }

    #[test]
    fn test_gardner_density_typical() {
        // For sandstone at 4000 m/s: rho ~ 2.46 g/cm3 = 2460 kg/m3
        let rho = gardner_density(4000.0);
        assert!(rho > 2000.0 && rho < 3000.0, "Gardner density = {}", rho);
    }

    #[test]
    fn test_gardner_density_custom() {
        let rho = gardner_density_custom(3000.0, 0.31, 0.25);
        assert!(rho > 2.0 && rho < 3.0); // result in g/cm3
    }

    #[test]
    fn test_slowness_velocity_roundtrip() {
        let v_original = 4500.0;
        let dt = velocity_to_slowness(v_original);
        let v_recovered = slowness_to_velocity(dt);
        assert!(approx_eq(v_original, v_recovered, 0.01));
    }

    #[test]
    fn test_slowness_to_velocity_zero() {
        assert!(approx_eq(slowness_to_velocity(0.0), 0.0, 1e-10));
        assert!(approx_eq(velocity_to_slowness(0.0), 0.0, 1e-10));
    }

    #[test]
    fn test_convolve_impulse() {
        let a = vec![0.0, 0.0, 1.0, 0.0, 0.0];
        let b = vec![1.0, 2.0, 3.0];
        let c = convolve(&a, &b);
        // Convolving with impulse at index 2 shifts b to start at index 2
        assert!(approx_eq(c[2], 1.0, 1e-10));
        assert!(approx_eq(c[3], 2.0, 1e-10));
        assert!(approx_eq(c[4], 3.0, 1e-10));
    }

    #[test]
    fn test_convolve_empty() {
        assert!(convolve(&[], &[1.0]).is_empty());
        assert!(convolve(&[1.0], &[]).is_empty());
    }
}
