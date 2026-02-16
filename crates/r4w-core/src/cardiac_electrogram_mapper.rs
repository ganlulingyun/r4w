//! Cardiac electrogram (EGM) signal processing for electrophysiology mapping.
//!
//! This module implements signal processing algorithms used in cardiac
//! electrophysiology studies for mapping atrial and ventricular electrical
//! activity. Applications include atrial fibrillation ablation guidance,
//! ventricular tachycardia substrate mapping, cardiac resynchronization
//! therapy planning, and arrhythmia research.
//!
//! # Components
//!
//! - [`ElectrogramConfig`] — Electrode array and acquisition parameters
//! - [`ActivationTimeDetector`] — Local activation time (LAT) detection
//! - [`VoltageMapper`] — Bipolar voltage substrate mapping with scar classification
//! - [`ConductionVelocityEstimator`] — Local conduction velocity from electrode arrays
//! - [`FractionationDetector`] — Complex fractionated atrial electrogram (CFAE) analysis
//! - [`DominantFrequencyAnalyzer`] — Spectral analysis for AF dominant frequency
//! - [`PhaseMapper`] — Hilbert transform phase mapping for rotor detection
//! - [`RotorDetector`] — Phase singularity detection (topological charge)
//! - [`IsochronalMapper`] — Isochronal activation map generation
//! - [`EntrainmentAnalyzer`] — Pacing entrainment analysis for tachycardia mechanism
//!
//! # Physiology Reference
//!
//! - Normal conduction velocity: ~0.5--1.0 m/s (atrium), ~1--4 m/s (ventricle/Purkinje)
//! - Bipolar voltage: normal >1.5 mV, borderzone 0.5--1.5 mV, scar <0.5 mV
//! - Dominant frequency in AF: typically 4--12 Hz
//! - Phase singularity: point where the line integral of phase gradient = +/-2*pi
//! - Activation time: dV/dt_min (unipolar), |dV/dt|_max or peak amplitude (bipolar)
//!
//! # Example
//!
//! ```
//! use r4w_core::cardiac_electrogram_mapper::{
//!     ElectrogramConfig, CatheterType, ActivationTimeDetector, DetectionMode,
//!     VoltageMapper, SubstrateClass,
//! };
//!
//! let config = ElectrogramConfig::new(16, 2000.0, 4.0, CatheterType::Grid);
//!
//! // Detect activation times from a bipolar electrogram
//! let detector = ActivationTimeDetector::new(config.sample_rate, DetectionMode::Bipolar);
//! let egm: Vec<f64> = (0..200).map(|i| {
//!     let t = i as f64 / 2000.0;
//!     if (t - 0.03).abs() < 0.005 { 2.0 } else { 0.05 * (t * 50.0).sin() }
//! }).collect();
//! let activations = detector.detect(&egm);
//!
//! // Classify substrate voltage
//! let mapper = VoltageMapper::default();
//! assert_eq!(mapper.classify(2.0), SubstrateClass::Normal);
//! assert_eq!(mapper.classify(1.0), SubstrateClass::Abnormal);
//! assert_eq!(mapper.classify(0.3), SubstrateClass::Scar);
//! ```

use std::f64::consts::PI;

// ============================================================================
// Configuration
// ============================================================================

/// Type of mapping catheter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CatheterType {
    /// Basket catheter (e.g., Constellation 64-pole).
    Basket,
    /// High-density grid catheter (e.g., Advisor HD Grid).
    Grid,
    /// HD multipolar catheter (e.g., PentaRay).
    HighDensity,
    /// Standard diagnostic catheter (e.g., decapolar).
    Standard,
}

/// Configuration for electrogram acquisition and electrode geometry.
#[derive(Debug, Clone)]
pub struct ElectrogramConfig {
    /// Number of electrodes.
    pub num_electrodes: usize,
    /// Sampling rate in Hz.
    pub sample_rate: f64,
    /// Inter-electrode spacing in millimeters.
    pub electrode_spacing_mm: f64,
    /// Catheter type.
    pub catheter_type: CatheterType,
}

impl ElectrogramConfig {
    /// Create a new configuration.
    ///
    /// # Panics
    ///
    /// Panics if `num_electrodes` is zero or `sample_rate` is not positive.
    pub fn new(
        num_electrodes: usize,
        sample_rate: f64,
        electrode_spacing_mm: f64,
        catheter_type: CatheterType,
    ) -> Self {
        assert!(num_electrodes > 0, "num_electrodes must be > 0");
        assert!(sample_rate > 0.0, "sample_rate must be positive");
        assert!(electrode_spacing_mm > 0.0, "electrode_spacing_mm must be positive");
        Self {
            num_electrodes,
            sample_rate,
            electrode_spacing_mm,
            catheter_type,
        }
    }

    /// Electrode spacing in meters.
    pub fn electrode_spacing_m(&self) -> f64 {
        self.electrode_spacing_mm / 1000.0
    }
}

// ============================================================================
// Activation Time Detection
// ============================================================================

/// Detection mode for local activation time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DetectionMode {
    /// Unipolar: activation at dV/dt minimum (steepest negative slope).
    Unipolar,
    /// Bipolar: activation at maximum absolute amplitude.
    Bipolar,
}

/// Result of activation time detection.
#[derive(Debug, Clone)]
pub struct ActivationResult {
    /// Sample indices of detected activations.
    pub activation_indices: Vec<usize>,
    /// Activation times in seconds.
    pub activation_times_s: Vec<f64>,
    /// Amplitudes at activation points.
    pub amplitudes: Vec<f64>,
}

/// Detects local activation time (LAT) from electrogram signals.
///
/// For unipolar EGMs, the activation corresponds to the minimum of dV/dt
/// (steepest negative intrinsic deflection). For bipolar EGMs, activation
/// corresponds to the maximum absolute amplitude of the signal.
#[derive(Debug, Clone)]
pub struct ActivationTimeDetector {
    sample_rate: f64,
    mode: DetectionMode,
    /// Minimum refractory interval between detections (seconds).
    refractory_s: f64,
    /// Amplitude threshold for detection (absolute value).
    threshold: f64,
}

impl ActivationTimeDetector {
    /// Create a new detector with default refractory period (150 ms) and threshold (0.1 mV).
    pub fn new(sample_rate: f64, mode: DetectionMode) -> Self {
        assert!(sample_rate > 0.0, "sample_rate must be positive");
        Self {
            sample_rate,
            mode,
            refractory_s: 0.150,
            threshold: 0.1,
        }
    }

    /// Set the refractory period in seconds.
    pub fn with_refractory(mut self, refractory_s: f64) -> Self {
        self.refractory_s = refractory_s;
        self
    }

    /// Set the detection threshold (in mV for bipolar, dV/dt units for unipolar).
    pub fn with_threshold(mut self, threshold: f64) -> Self {
        self.threshold = threshold;
        self
    }

    /// Detect activations in the given electrogram signal.
    pub fn detect(&self, egm: &[f64]) -> ActivationResult {
        if egm.len() < 3 {
            return ActivationResult {
                activation_indices: vec![],
                activation_times_s: vec![],
                amplitudes: vec![],
            };
        }

        let refractory_samples = (self.refractory_s * self.sample_rate).round() as usize;

        match self.mode {
            DetectionMode::Unipolar => self.detect_unipolar(egm, refractory_samples),
            DetectionMode::Bipolar => self.detect_bipolar(egm, refractory_samples),
        }
    }

    /// Unipolar detection: find dV/dt minimum (steepest negative slope).
    fn detect_unipolar(&self, egm: &[f64], refractory: usize) -> ActivationResult {
        let n = egm.len();
        // Compute derivative: dV/dt approximated by central differences
        let dt = 1.0 / self.sample_rate;
        let mut dvdt = vec![0.0_f64; n];
        for i in 1..n - 1 {
            dvdt[i] = (egm[i + 1] - egm[i - 1]) / (2.0 * dt);
        }

        // Find local minima of dV/dt below the negative threshold
        let mut indices = Vec::new();
        let mut amplitudes = Vec::new();
        let mut last_det: Option<usize> = None;

        for i in 1..n - 1 {
            if dvdt[i] < dvdt[i - 1] && dvdt[i] <= dvdt[i + 1] && dvdt[i] < -self.threshold {
                if let Some(last) = last_det {
                    if i - last < refractory {
                        // Within refractory, keep the one with more negative dV/dt
                        if dvdt[i] < dvdt[last] {
                            *indices.last_mut().unwrap() = i;
                            *amplitudes.last_mut().unwrap() = dvdt[i];
                            last_det = Some(i);
                        }
                        continue;
                    }
                }
                indices.push(i);
                amplitudes.push(dvdt[i]);
                last_det = Some(i);
            }
        }

        let times: Vec<f64> = indices.iter().map(|&i| i as f64 / self.sample_rate).collect();
        ActivationResult {
            activation_indices: indices,
            activation_times_s: times,
            amplitudes,
        }
    }

    /// Bipolar detection: find maximum absolute amplitude.
    fn detect_bipolar(&self, egm: &[f64], refractory: usize) -> ActivationResult {
        let n = egm.len();
        let abs_egm: Vec<f64> = egm.iter().map(|&x| x.abs()).collect();

        let mut indices = Vec::new();
        let mut amplitudes = Vec::new();
        let mut last_det: Option<usize> = None;

        for i in 1..n - 1 {
            if abs_egm[i] > abs_egm[i - 1]
                && abs_egm[i] >= abs_egm[i + 1]
                && abs_egm[i] > self.threshold
            {
                if let Some(last) = last_det {
                    if i - last < refractory {
                        if abs_egm[i] > abs_egm[last] {
                            *indices.last_mut().unwrap() = i;
                            *amplitudes.last_mut().unwrap() = egm[i];
                            last_det = Some(i);
                        }
                        continue;
                    }
                }
                indices.push(i);
                amplitudes.push(egm[i]);
                last_det = Some(i);
            }
        }

        let times: Vec<f64> = indices.iter().map(|&i| i as f64 / self.sample_rate).collect();
        ActivationResult {
            activation_indices: indices,
            activation_times_s: times,
            amplitudes,
        }
    }
}

// ============================================================================
// Voltage Mapping
// ============================================================================

/// Substrate classification based on bipolar voltage.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubstrateClass {
    /// Normal tissue: bipolar voltage > 1.5 mV.
    Normal,
    /// Abnormal / borderzone tissue: 0.5--1.5 mV.
    Abnormal,
    /// Scar tissue: < 0.5 mV.
    Scar,
}

/// Voltage mapping result for a single electrode site.
#[derive(Debug, Clone)]
pub struct VoltageMeasurement {
    /// Peak-to-peak bipolar voltage in mV.
    pub peak_to_peak_mv: f64,
    /// Substrate classification.
    pub classification: SubstrateClass,
}

/// Computes peak-to-peak bipolar voltage and classifies substrate.
///
/// Standard thresholds: normal >1.5 mV, abnormal 0.5--1.5 mV, scar <0.5 mV.
/// These thresholds can be customized for specific mapping studies.
#[derive(Debug, Clone)]
pub struct VoltageMapper {
    /// Scar threshold in mV (default 0.5).
    pub scar_threshold_mv: f64,
    /// Normal threshold in mV (default 1.5).
    pub normal_threshold_mv: f64,
}

impl Default for VoltageMapper {
    fn default() -> Self {
        Self {
            scar_threshold_mv: 0.5,
            normal_threshold_mv: 1.5,
        }
    }
}

impl VoltageMapper {
    /// Create a mapper with custom thresholds.
    pub fn new(scar_threshold_mv: f64, normal_threshold_mv: f64) -> Self {
        assert!(
            scar_threshold_mv < normal_threshold_mv,
            "scar threshold must be less than normal threshold"
        );
        Self {
            scar_threshold_mv,
            normal_threshold_mv,
        }
    }

    /// Classify a voltage value.
    pub fn classify(&self, voltage_mv: f64) -> SubstrateClass {
        if voltage_mv < self.scar_threshold_mv {
            SubstrateClass::Scar
        } else if voltage_mv < self.normal_threshold_mv {
            SubstrateClass::Abnormal
        } else {
            SubstrateClass::Normal
        }
    }

    /// Compute peak-to-peak voltage from a bipolar EGM segment and classify.
    pub fn measure(&self, egm: &[f64]) -> VoltageMeasurement {
        if egm.is_empty() {
            return VoltageMeasurement {
                peak_to_peak_mv: 0.0,
                classification: SubstrateClass::Scar,
            };
        }
        let max = egm.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = egm.iter().cloned().fold(f64::INFINITY, f64::min);
        let pp = max - min;
        VoltageMeasurement {
            peak_to_peak_mv: pp,
            classification: self.classify(pp),
        }
    }

    /// Batch measurement: compute voltage map from multiple EGM segments.
    pub fn map_electrodes(&self, egms: &[Vec<f64>]) -> Vec<VoltageMeasurement> {
        egms.iter().map(|egm| self.measure(egm)).collect()
    }
}

// ============================================================================
// Conduction Velocity Estimation
// ============================================================================

/// Result of conduction velocity estimation.
#[derive(Debug, Clone)]
pub struct ConductionVelocityResult {
    /// Estimated conduction velocity in m/s.
    pub velocity_m_s: f64,
    /// Distance between electrodes in meters.
    pub distance_m: f64,
    /// Activation time difference in seconds.
    pub delta_lat_s: f64,
}

/// Estimates local conduction velocity from activation times across electrode pairs.
///
/// CV = distance / delta_LAT, where distance is the electrode spacing and
/// delta_LAT is the difference in local activation times.
///
/// Normal values: atrium ~0.5--1.0 m/s, ventricle ~1--4 m/s, Purkinje up to 4 m/s.
#[derive(Debug, Clone)]
pub struct ConductionVelocityEstimator {
    /// Electrode spacing in meters.
    electrode_spacing_m: f64,
    /// Minimum delta-LAT to avoid division-by-near-zero (seconds).
    min_delta_lat_s: f64,
    /// Maximum plausible CV in m/s.
    max_cv_m_s: f64,
}

impl ConductionVelocityEstimator {
    /// Create a new estimator.
    pub fn new(electrode_spacing_mm: f64) -> Self {
        Self {
            electrode_spacing_m: electrode_spacing_mm / 1000.0,
            min_delta_lat_s: 0.0005, // 0.5 ms minimum
            max_cv_m_s: 10.0,
        }
    }

    /// Set the maximum plausible conduction velocity.
    pub fn with_max_cv(mut self, max_cv_m_s: f64) -> Self {
        self.max_cv_m_s = max_cv_m_s;
        self
    }

    /// Estimate CV from two activation times (seconds).
    /// Returns `None` if the delta-LAT is too small or CV exceeds the maximum.
    pub fn estimate(&self, lat1_s: f64, lat2_s: f64) -> Option<ConductionVelocityResult> {
        let delta = (lat2_s - lat1_s).abs();
        if delta < self.min_delta_lat_s {
            return None;
        }
        let cv = self.electrode_spacing_m / delta;
        if cv > self.max_cv_m_s {
            return None;
        }
        Some(ConductionVelocityResult {
            velocity_m_s: cv,
            distance_m: self.electrode_spacing_m,
            delta_lat_s: delta,
        })
    }

    /// Estimate CV from an array of activation times (seconds) at equally-spaced
    /// electrodes. Returns CV estimates for each consecutive pair.
    pub fn estimate_array(&self, activation_times_s: &[f64]) -> Vec<Option<ConductionVelocityResult>> {
        if activation_times_s.len() < 2 {
            return vec![];
        }
        activation_times_s
            .windows(2)
            .map(|w| self.estimate(w[0], w[1]))
            .collect()
    }

    /// Compute mean CV from an array, ignoring None results.
    pub fn mean_cv(&self, activation_times_s: &[f64]) -> Option<f64> {
        let estimates = self.estimate_array(activation_times_s);
        let valid: Vec<f64> = estimates
            .iter()
            .filter_map(|e| e.as_ref().map(|r| r.velocity_m_s))
            .collect();
        if valid.is_empty() {
            None
        } else {
            Some(valid.iter().sum::<f64>() / valid.len() as f64)
        }
    }
}

// ============================================================================
// Fractionation Detection (CFAE)
// ============================================================================

/// Result of fractionation analysis.
#[derive(Debug, Clone)]
pub struct FractionationResult {
    /// Number of deflections exceeding threshold within analysis window.
    pub deflection_count: usize,
    /// Mean fractionation interval in seconds (mean interval between deflections).
    pub mean_interval_s: f64,
    /// Whether the EGM is classified as complex fractionated.
    pub is_fractionated: bool,
    /// Intervals between consecutive deflections in seconds.
    pub intervals_s: Vec<f64>,
}

/// Detects complex fractionated atrial electrograms (CFAEs).
///
/// CFAEs are characterized by multiple deflections within a short time window.
/// They are thought to represent areas of slow conduction, pivot points of
/// re-entrant circuits, or sites of wave-break, and are potential ablation targets.
///
/// Detection criteria (Nademanee et al.):
/// - Fractionated signals with >= 2 deflections, or
/// - Continuous electrical activity with very short cycle length (< 120 ms)
#[derive(Debug, Clone)]
pub struct FractionationDetector {
    sample_rate: f64,
    /// Amplitude threshold for deflection detection (mV).
    amplitude_threshold: f64,
    /// Analysis window duration in seconds (default 2.5 s).
    window_duration_s: f64,
    /// Minimum fractionation interval for CFAE classification (seconds).
    cfae_interval_threshold_s: f64,
    /// Minimum deflection count for CFAE classification.
    min_deflection_count: usize,
}

impl FractionationDetector {
    /// Create a new detector with default settings.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            amplitude_threshold: 0.05,
            window_duration_s: 2.5,
            cfae_interval_threshold_s: 0.120, // 120 ms
            min_deflection_count: 4,
        }
    }

    /// Set amplitude threshold for deflection detection.
    pub fn with_amplitude_threshold(mut self, threshold: f64) -> Self {
        self.amplitude_threshold = threshold;
        self
    }

    /// Set the minimum deflection count for CFAE classification.
    pub fn with_min_deflection_count(mut self, count: usize) -> Self {
        self.min_deflection_count = count;
        self
    }

    /// Analyze an EGM segment for fractionation.
    pub fn analyze(&self, egm: &[f64]) -> FractionationResult {
        let window_samples =
            (self.window_duration_s * self.sample_rate).round() as usize;
        let segment = if egm.len() > window_samples {
            &egm[..window_samples]
        } else {
            egm
        };

        // Find deflection peaks: local maxima of |egm| above threshold
        let abs_signal: Vec<f64> = segment.iter().map(|&x| x.abs()).collect();
        let min_refractory = (0.010 * self.sample_rate).round() as usize; // 10 ms min between deflections

        let mut deflection_indices = Vec::new();
        let mut last_idx: Option<usize> = None;

        for i in 1..abs_signal.len().saturating_sub(1) {
            if abs_signal[i] > abs_signal[i - 1]
                && abs_signal[i] >= abs_signal[i + 1]
                && abs_signal[i] > self.amplitude_threshold
            {
                if let Some(last) = last_idx {
                    if i - last < min_refractory {
                        continue;
                    }
                }
                deflection_indices.push(i);
                last_idx = Some(i);
            }
        }

        let deflection_count = deflection_indices.len();

        // Compute intervals between consecutive deflections
        let intervals_s: Vec<f64> = deflection_indices
            .windows(2)
            .map(|w| (w[1] - w[0]) as f64 / self.sample_rate)
            .collect();

        let mean_interval_s = if intervals_s.is_empty() {
            0.0
        } else {
            intervals_s.iter().sum::<f64>() / intervals_s.len() as f64
        };

        let is_fractionated = deflection_count >= self.min_deflection_count
            && mean_interval_s > 0.0
            && mean_interval_s < self.cfae_interval_threshold_s;

        FractionationResult {
            deflection_count,
            mean_interval_s,
            is_fractionated,
            intervals_s,
        }
    }
}

// ============================================================================
// Dominant Frequency Analysis
// ============================================================================

/// Result of dominant frequency analysis.
#[derive(Debug, Clone)]
pub struct DominantFrequencyResult {
    /// Dominant frequency in Hz.
    pub dominant_freq_hz: f64,
    /// Power at the dominant frequency (arbitrary units).
    pub peak_power: f64,
    /// Organization index (ratio of peak power to total spectral power in band).
    pub organization_index: f64,
    /// Power spectrum (frequency, power) pairs in the analysis band.
    pub spectrum: Vec<(f64, f64)>,
}

/// Spectral analysis of electrograms for dominant frequency estimation.
///
/// In atrial fibrillation, the dominant frequency (DF) reflects the rate of
/// local activation and can guide ablation targeting high-frequency sources.
/// The analysis band is typically 3--15 Hz, with AF DFs usually 4--12 Hz.
#[derive(Debug, Clone)]
pub struct DominantFrequencyAnalyzer {
    sample_rate: f64,
    /// Lower frequency bound for analysis (Hz).
    freq_min_hz: f64,
    /// Upper frequency bound for analysis (Hz).
    freq_max_hz: f64,
}

impl DominantFrequencyAnalyzer {
    /// Create a new analyzer with default AF band (3--15 Hz).
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            freq_min_hz: 3.0,
            freq_max_hz: 15.0,
        }
    }

    /// Set the analysis frequency band.
    pub fn with_freq_range(mut self, min_hz: f64, max_hz: f64) -> Self {
        self.freq_min_hz = min_hz;
        self.freq_max_hz = max_hz;
        self
    }

    /// Analyze a signal segment for dominant frequency using DFT.
    pub fn analyze(&self, signal: &[f64]) -> DominantFrequencyResult {
        let n = signal.len();
        if n == 0 {
            return DominantFrequencyResult {
                dominant_freq_hz: 0.0,
                peak_power: 0.0,
                organization_index: 0.0,
                spectrum: vec![],
            };
        }

        // Apply Hann window
        let windowed: Vec<f64> = signal
            .iter()
            .enumerate()
            .map(|(i, &x)| {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n as f64 - 1.0)).cos());
                x * w
            })
            .collect();

        // Compute DFT magnitude spectrum for positive frequencies
        let n_bins = n / 2 + 1;
        let mut spectrum = Vec::with_capacity(n_bins);
        let mut band_spectrum = Vec::new();
        let mut peak_power = 0.0_f64;
        let mut peak_freq = 0.0_f64;
        let mut total_band_power = 0.0_f64;

        for k in 0..n_bins {
            let freq = k as f64 * self.sample_rate / n as f64;
            // DFT bin
            let mut re = 0.0_f64;
            let mut im = 0.0_f64;
            for (j, &x) in windowed.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                re += x * angle.cos();
                im += x * angle.sin();
            }
            let power = (re * re + im * im) / (n as f64 * n as f64);

            if freq >= self.freq_min_hz && freq <= self.freq_max_hz {
                band_spectrum.push((freq, power));
                total_band_power += power;
                if power > peak_power {
                    peak_power = power;
                    peak_freq = freq;
                }
            }
            spectrum.push((freq, power));
        }

        let organization_index = if total_band_power > 0.0 {
            peak_power / total_band_power
        } else {
            0.0
        };

        DominantFrequencyResult {
            dominant_freq_hz: peak_freq,
            peak_power,
            organization_index,
            spectrum: band_spectrum,
        }
    }
}

// ============================================================================
// Phase Mapping (Hilbert Transform)
// ============================================================================

/// Phase mapping result.
#[derive(Debug, Clone)]
pub struct PhaseMapResult {
    /// Instantaneous phase at each sample (radians, [-pi, pi]).
    pub phase: Vec<f64>,
    /// Analytic signal (real, imaginary) pairs.
    pub analytic_signal: Vec<(f64, f64)>,
}

/// Hilbert-transform-based phase mapping for rotor detection.
///
/// Computes the analytic signal via DFT-based Hilbert transform,
/// then extracts instantaneous phase as atan2(imag, real).
/// Phase singularities (rotors) appear where all phase values
/// converge at a single point.
#[derive(Debug, Clone)]
pub struct PhaseMapper {
    sample_rate: f64,
    /// Optional bandpass pre-filtering: (low_hz, high_hz).
    bandpass: Option<(f64, f64)>,
}

impl PhaseMapper {
    /// Create a new phase mapper.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            bandpass: None,
        }
    }

    /// Enable bandpass pre-filtering before Hilbert transform.
    pub fn with_bandpass(mut self, low_hz: f64, high_hz: f64) -> Self {
        self.bandpass = Some((low_hz, high_hz));
        self
    }

    /// Return the configured sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Compute instantaneous phase of the signal via Hilbert transform.
    pub fn compute_phase(&self, signal: &[f64]) -> PhaseMapResult {
        let signal = if let Some((lo, hi)) = self.bandpass {
            bandpass_filter(signal, self.sample_rate, lo, hi)
        } else {
            signal.to_vec()
        };

        let analytic = hilbert_transform(&signal);
        let phase: Vec<f64> = analytic.iter().map(|&(re, im)| im.atan2(re)).collect();

        PhaseMapResult {
            phase,
            analytic_signal: analytic,
        }
    }
}

/// Compute the DFT-based Hilbert transform, returning the analytic signal.
///
/// The analytic signal has the original signal as real part and the Hilbert
/// transform as imaginary part. Computed by zeroing negative-frequency DFT bins
/// and doubling positive-frequency bins.
fn hilbert_transform(signal: &[f64]) -> Vec<(f64, f64)> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // Forward DFT
    let mut spectrum_re = vec![0.0_f64; n];
    let mut spectrum_im = vec![0.0_f64; n];
    for k in 0..n {
        for (j, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            spectrum_re[k] += x * angle.cos();
            spectrum_im[k] += x * angle.sin();
        }
    }

    // Zero negative frequencies, double positive frequencies
    // Bin 0 (DC) and bin N/2 (Nyquist for even N) stay as-is
    for k in 1..n {
        let is_positive = if n % 2 == 0 {
            k < n / 2
        } else {
            k <= n / 2
        };
        let is_nyquist = n % 2 == 0 && k == n / 2;

        if is_positive && !is_nyquist {
            spectrum_re[k] *= 2.0;
            spectrum_im[k] *= 2.0;
        } else if !is_nyquist && k != 0 {
            spectrum_re[k] = 0.0;
            spectrum_im[k] = 0.0;
        }
    }

    // Inverse DFT
    let mut result = Vec::with_capacity(n);
    for j in 0..n {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
            re += spectrum_re[k] * angle.cos() - spectrum_im[k] * angle.sin();
            im += spectrum_re[k] * angle.sin() + spectrum_im[k] * angle.cos();
        }
        result.push((re / n as f64, im / n as f64));
    }

    result
}

/// Simple frequency-domain bandpass filter.
fn bandpass_filter(signal: &[f64], sample_rate: f64, low_hz: f64, high_hz: f64) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return vec![];
    }

    // Forward DFT
    let mut spec_re = vec![0.0_f64; n];
    let mut spec_im = vec![0.0_f64; n];
    for k in 0..n {
        for (j, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            spec_re[k] += x * angle.cos();
            spec_im[k] += x * angle.sin();
        }
    }

    // Zero bins outside passband
    for k in 0..n {
        let freq = if k <= n / 2 {
            k as f64 * sample_rate / n as f64
        } else {
            (n as f64 - k as f64) * sample_rate / n as f64
        };
        if freq < low_hz || freq > high_hz {
            spec_re[k] = 0.0;
            spec_im[k] = 0.0;
        }
    }

    // Inverse DFT (real part only)
    let mut result = vec![0.0_f64; n];
    for j in 0..n {
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
            result[j] += spec_re[k] * angle.cos() - spec_im[k] * angle.sin();
        }
        result[j] /= n as f64;
    }
    result
}

// ============================================================================
// Rotor / Phase Singularity Detection
// ============================================================================

/// A detected phase singularity (rotor core).
#[derive(Debug, Clone)]
pub struct PhaseSingularity {
    /// Grid row index.
    pub row: usize,
    /// Grid column index.
    pub col: usize,
    /// Topological charge: +1 (counterclockwise rotor) or -1 (clockwise rotor).
    pub charge: i32,
    /// Time sample index at which the singularity was detected.
    pub time_index: usize,
}

/// Detects phase singularities (rotors) in a 2D phase map.
///
/// A phase singularity is a point where the line integral of the phase
/// gradient around a closed loop equals +/-2*pi. These points represent
/// the organizing centers of re-entrant circuits and are key targets
/// for ablation in atrial fibrillation.
#[derive(Debug, Clone)]
pub struct RotorDetector {
    /// Number of rows in the electrode grid.
    rows: usize,
    /// Number of columns in the electrode grid.
    cols: usize,
}

impl RotorDetector {
    /// Create a new rotor detector for a grid of `rows x cols` electrodes.
    pub fn new(rows: usize, cols: usize) -> Self {
        assert!(rows >= 2 && cols >= 2, "grid must be at least 2x2");
        Self { rows, cols }
    }

    /// Detect phase singularities in a 2D phase map at a given time index.
    ///
    /// `phase_map` is a `rows x cols` slice of phase values in [-pi, pi].
    pub fn detect(&self, phase_map: &[f64], time_index: usize) -> Vec<PhaseSingularity> {
        assert_eq!(
            phase_map.len(),
            self.rows * self.cols,
            "phase_map length must equal rows * cols"
        );

        let mut singularities = Vec::new();

        // Check each interior point (loop around 4 neighbors)
        for r in 0..self.rows - 1 {
            for c in 0..self.cols - 1 {
                let charge = self.compute_topological_charge(phase_map, r, c);
                if charge != 0 {
                    singularities.push(PhaseSingularity {
                        row: r,
                        col: c,
                        charge,
                        time_index,
                    });
                }
            }
        }

        singularities
    }

    /// Compute topological charge around a unit cell at (r, c).
    /// Loop: (r,c) -> (r,c+1) -> (r+1,c+1) -> (r+1,c) -> (r,c).
    fn compute_topological_charge(&self, phase_map: &[f64], r: usize, c: usize) -> i32 {
        let idx = |row: usize, col: usize| row * self.cols + col;
        let corners = [
            phase_map[idx(r, c)],
            phase_map[idx(r, c + 1)],
            phase_map[idx(r + 1, c + 1)],
            phase_map[idx(r + 1, c)],
        ];

        // Sum of phase differences around the loop
        let mut total_phase_diff = 0.0_f64;
        for i in 0..4 {
            let next = (i + 1) % 4;
            let diff = wrap_phase(corners[next] - corners[i]);
            total_phase_diff += diff;
        }

        // Topological charge: total / 2*pi, rounded to nearest integer
        let charge = (total_phase_diff / (2.0 * PI)).round() as i32;
        charge
    }

    /// Detect singularities across a time series of phase maps.
    /// `phase_maps[t]` is the 2D phase map at time index `t`.
    pub fn detect_sequence(&self, phase_maps: &[Vec<f64>]) -> Vec<PhaseSingularity> {
        let mut all = Vec::new();
        for (t, map) in phase_maps.iter().enumerate() {
            all.extend(self.detect(map, t));
        }
        all
    }
}

/// Wrap a phase difference to [-pi, pi].
fn wrap_phase(mut phase: f64) -> f64 {
    while phase > PI {
        phase -= 2.0 * PI;
    }
    while phase < -PI {
        phase += 2.0 * PI;
    }
    phase
}

// ============================================================================
// Isochronal Mapping
// ============================================================================

/// An isochronal activation map on a regular grid.
#[derive(Debug, Clone)]
pub struct IsochronalMap {
    /// Grid rows.
    pub rows: usize,
    /// Grid columns.
    pub cols: usize,
    /// Interpolated activation times (ms) at each grid point (row-major).
    /// `f64::NAN` indicates no data.
    pub activation_times_ms: Vec<f64>,
    /// Isochrone contour levels in ms.
    pub isochrone_levels_ms: Vec<f64>,
}

/// Generates isochronal activation maps from scattered LAT measurements.
///
/// Isochronal maps display activation wavefronts as contour lines separated
/// by equal time intervals (e.g., 10 ms isochrones). They reveal conduction
/// patterns, slow conduction zones, and re-entry circuits.
///
/// Uses inverse-distance weighting (IDW) interpolation for scattered data.
#[derive(Debug, Clone)]
pub struct IsochronalMapper {
    /// Grid rows.
    rows: usize,
    /// Grid columns.
    cols: usize,
    /// Isochrone interval in milliseconds.
    isochrone_interval_ms: f64,
    /// IDW power parameter (typically 2).
    idw_power: f64,
}

impl IsochronalMapper {
    /// Create a new mapper for a grid of `rows x cols` points.
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            isochrone_interval_ms: 10.0,
            idw_power: 2.0,
        }
    }

    /// Set the isochrone interval.
    pub fn with_interval_ms(mut self, interval_ms: f64) -> Self {
        self.isochrone_interval_ms = interval_ms;
        self
    }

    /// Set the IDW power parameter.
    pub fn with_idw_power(mut self, power: f64) -> Self {
        self.idw_power = power;
        self
    }

    /// Generate an isochronal map from scattered measurements.
    ///
    /// `measurements` is a slice of (row, col, activation_time_ms) tuples.
    pub fn generate(
        &self,
        measurements: &[(f64, f64, f64)],
    ) -> IsochronalMap {
        if measurements.is_empty() {
            return IsochronalMap {
                rows: self.rows,
                cols: self.cols,
                activation_times_ms: vec![f64::NAN; self.rows * self.cols],
                isochrone_levels_ms: vec![],
            };
        }

        // IDW interpolation onto regular grid
        let mut grid = vec![f64::NAN; self.rows * self.cols];
        for r in 0..self.rows {
            for c in 0..self.cols {
                let gr = r as f64;
                let gc = c as f64;

                // Check if there's an exact measurement at this point
                let mut exact = None;
                for &(mr, mc, mt) in measurements {
                    let dr = gr - mr;
                    let dc = gc - mc;
                    let dist_sq = dr * dr + dc * dc;
                    if dist_sq < 1e-12 {
                        exact = Some(mt);
                        break;
                    }
                }

                if let Some(t) = exact {
                    grid[r * self.cols + c] = t;
                } else {
                    let mut numerator = 0.0_f64;
                    let mut denominator = 0.0_f64;
                    for &(mr, mc, mt) in measurements {
                        let dr = gr - mr;
                        let dc = gc - mc;
                        let dist = (dr * dr + dc * dc).sqrt();
                        let weight = 1.0 / dist.powf(self.idw_power);
                        numerator += weight * mt;
                        denominator += weight;
                    }
                    if denominator > 0.0 {
                        grid[r * self.cols + c] = numerator / denominator;
                    }
                }
            }
        }

        // Compute isochrone levels
        let valid_times: Vec<f64> = grid.iter().copied().filter(|t| !t.is_nan()).collect();
        let isochrone_levels = if valid_times.is_empty() {
            vec![]
        } else {
            let min_t = valid_times.iter().cloned().fold(f64::INFINITY, f64::min);
            let max_t = valid_times
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let mut levels = Vec::new();
            let mut t = (min_t / self.isochrone_interval_ms).ceil() * self.isochrone_interval_ms;
            while t <= max_t {
                levels.push(t);
                t += self.isochrone_interval_ms;
            }
            levels
        };

        IsochronalMap {
            rows: self.rows,
            cols: self.cols,
            activation_times_ms: grid,
            isochrone_levels_ms: isochrone_levels,
        }
    }
}

// ============================================================================
// Entrainment Analysis
// ============================================================================

/// Result of entrainment analysis for tachycardia mechanism determination.
#[derive(Debug, Clone)]
pub struct EntrainmentResult {
    /// Post-pacing interval in milliseconds.
    pub ppi_ms: f64,
    /// Tachycardia cycle length in milliseconds.
    pub tcl_ms: f64,
    /// PPI - TCL difference in milliseconds.
    pub ppi_tcl_diff_ms: f64,
    /// Stimulus to QRS interval in milliseconds (if available).
    pub stim_qrs_ms: Option<f64>,
    /// Classification of entrainment response.
    pub classification: EntrainmentClass,
}

/// Classification of entrainment response.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntrainmentClass {
    /// PPI - TCL <= 30 ms: pacing site is within the re-entrant circuit.
    WithinCircuit,
    /// PPI - TCL 30--60 ms: pacing site is proximal to the circuit.
    Proximal,
    /// PPI - TCL > 60 ms: pacing site is remote from the circuit.
    Remote,
    /// Tachycardia terminated during pacing.
    Terminated,
}

/// Analyzes entrainment pacing for tachycardia mechanism diagnosis.
///
/// Entrainment pacing involves overdrive pacing during tachycardia at a rate
/// slightly faster than the tachycardia cycle length (TCL). The post-pacing
/// interval (PPI) relative to the TCL indicates whether the pacing site is
/// within the re-entrant circuit.
///
/// - PPI - TCL <= 30 ms: within the circuit
/// - PPI - TCL 30--60 ms: proximal bystander or outer loop
/// - PPI - TCL > 60 ms: remote from the circuit
#[derive(Debug, Clone)]
pub struct EntrainmentAnalyzer {
    /// Threshold for "within circuit" classification (ms).
    within_threshold_ms: f64,
    /// Threshold for "proximal" classification (ms).
    proximal_threshold_ms: f64,
}

impl Default for EntrainmentAnalyzer {
    fn default() -> Self {
        Self {
            within_threshold_ms: 30.0,
            proximal_threshold_ms: 60.0,
        }
    }
}

impl EntrainmentAnalyzer {
    /// Create with custom thresholds.
    pub fn new(within_threshold_ms: f64, proximal_threshold_ms: f64) -> Self {
        Self {
            within_threshold_ms,
            proximal_threshold_ms,
        }
    }

    /// Analyze entrainment from the post-pacing interval and tachycardia cycle length.
    ///
    /// - `ppi_ms`: post-pacing interval in milliseconds
    /// - `tcl_ms`: tachycardia cycle length in milliseconds
    /// - `stim_qrs_ms`: optional stimulus-to-QRS interval in milliseconds
    pub fn analyze(
        &self,
        ppi_ms: f64,
        tcl_ms: f64,
        stim_qrs_ms: Option<f64>,
    ) -> EntrainmentResult {
        let diff = ppi_ms - tcl_ms;
        let classification = if diff < 0.0 {
            EntrainmentClass::Terminated
        } else if diff <= self.within_threshold_ms {
            EntrainmentClass::WithinCircuit
        } else if diff <= self.proximal_threshold_ms {
            EntrainmentClass::Proximal
        } else {
            EntrainmentClass::Remote
        };

        EntrainmentResult {
            ppi_ms,
            tcl_ms,
            ppi_tcl_diff_ms: diff,
            stim_qrs_ms,
            classification,
        }
    }

    /// Analyze from a series of activation times (ms) where the last few cycles
    /// represent pacing, and earlier cycles represent tachycardia.
    ///
    /// `activation_times_ms`: activation times in order.
    /// `num_pacing_beats`: number of paced beats at the end of the sequence.
    ///
    /// Returns `None` if there are not enough data points.
    pub fn analyze_from_activations(
        &self,
        activation_times_ms: &[f64],
        num_pacing_beats: usize,
    ) -> Option<EntrainmentResult> {
        // Need at least num_pacing_beats + 2 activations to compute TCL and PPI
        if activation_times_ms.len() < num_pacing_beats + 3 {
            return None;
        }

        let n = activation_times_ms.len();

        // TCL: average cycle length before pacing.
        // The last (num_pacing_beats + 1) activations are pacing + return beat,
        // so TCL beats are indices 0 through n - num_pacing_beats - 2.
        let num_tcl_beats = n - num_pacing_beats - 1;
        if num_tcl_beats < 2 {
            return None;
        }
        let tcl_intervals: Vec<f64> = (0..num_tcl_beats - 1)
            .map(|i| activation_times_ms[i + 1] - activation_times_ms[i])
            .collect();
        let tcl_ms = tcl_intervals.iter().sum::<f64>() / tcl_intervals.len() as f64;

        // PPI: interval from last pacing stimulus to first return beat
        let last_pace_idx = n - 2;
        let return_idx = n - 1;
        let ppi_ms = activation_times_ms[return_idx] - activation_times_ms[last_pace_idx];

        Some(self.analyze(ppi_ms, tcl_ms, None))
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // --- ElectrogramConfig tests ---

    #[test]
    fn test_config_creation() {
        let config = ElectrogramConfig::new(64, 2000.0, 4.0, CatheterType::Basket);
        assert_eq!(config.num_electrodes, 64);
        assert_eq!(config.sample_rate, 2000.0);
        assert_eq!(config.electrode_spacing_mm, 4.0);
        assert_eq!(config.catheter_type, CatheterType::Basket);
    }

    #[test]
    fn test_config_spacing_meters() {
        let config = ElectrogramConfig::new(16, 2000.0, 4.0, CatheterType::Grid);
        assert!((config.electrode_spacing_m() - 0.004).abs() < 1e-9);
    }

    #[test]
    #[should_panic]
    fn test_config_zero_electrodes() {
        ElectrogramConfig::new(0, 2000.0, 4.0, CatheterType::Standard);
    }

    #[test]
    #[should_panic]
    fn test_config_zero_sample_rate() {
        ElectrogramConfig::new(16, 0.0, 4.0, CatheterType::Standard);
    }

    // --- ActivationTimeDetector tests ---

    #[test]
    fn test_bipolar_activation_single_peak() {
        let fs = 2000.0;
        let detector = ActivationTimeDetector::new(fs, DetectionMode::Bipolar)
            .with_threshold(0.5);

        // Create a signal with one sharp bipolar deflection at 50 ms
        let n = 400; // 200 ms
        let peak_idx = 100; // 50 ms
        let mut egm = vec![0.0; n];
        egm[peak_idx] = 3.0;

        let result = detector.detect(&egm);
        assert_eq!(result.activation_indices.len(), 1);
        assert_eq!(result.activation_indices[0], peak_idx);
        assert!((result.activation_times_s[0] - 0.05).abs() < 0.001);
    }

    #[test]
    fn test_bipolar_activation_multiple_peaks() {
        let fs = 2000.0;
        let detector = ActivationTimeDetector::new(fs, DetectionMode::Bipolar)
            .with_threshold(0.5)
            .with_refractory(0.150);

        let n = 2000; // 1 second
        let mut egm = vec![0.0; n];
        // Place peaks at 200ms, 500ms, 800ms (>150ms apart)
        for &idx in &[400, 1000, 1600] {
            egm[idx] = 2.5;
        }

        let result = detector.detect(&egm);
        assert_eq!(result.activation_indices.len(), 3);
    }

    #[test]
    fn test_bipolar_refractory_period() {
        let fs = 2000.0;
        let detector = ActivationTimeDetector::new(fs, DetectionMode::Bipolar)
            .with_threshold(0.3)
            .with_refractory(0.200);

        let n = 1000;
        let mut egm = vec![0.0; n];
        // Two peaks 50 ms apart (within 200 ms refractory)
        egm[200] = 1.0;
        egm[300] = 2.0; // Larger, should replace the first

        let result = detector.detect(&egm);
        assert_eq!(result.activation_indices.len(), 1);
        assert_eq!(result.activation_indices[0], 300); // Kept the larger one
    }

    #[test]
    fn test_unipolar_activation_detection() {
        let fs = 2000.0;
        let detector = ActivationTimeDetector::new(fs, DetectionMode::Unipolar)
            .with_threshold(100.0);

        // Create unipolar EGM: sharp negative deflection (intrinsic deflection)
        let n = 400;
        let mut egm = vec![0.0; n];
        // Rising then sharply falling at sample 100
        for i in 80..100 {
            egm[i] = (i - 80) as f64 * 0.1;
        }
        for i in 100..120 {
            egm[i] = 2.0 - (i - 100) as f64 * 0.2;
        }
        for i in 120..140 {
            egm[i] = -2.0 + (i - 120) as f64 * 0.1;
        }

        let result = detector.detect(&egm);
        // Should detect activation near the steepest negative slope
        assert!(!result.activation_indices.is_empty());
    }

    #[test]
    fn test_activation_empty_signal() {
        let detector = ActivationTimeDetector::new(2000.0, DetectionMode::Bipolar);
        let result = detector.detect(&[]);
        assert!(result.activation_indices.is_empty());
    }

    #[test]
    fn test_activation_short_signal() {
        let detector = ActivationTimeDetector::new(2000.0, DetectionMode::Bipolar);
        let result = detector.detect(&[1.0, 2.0]);
        assert!(result.activation_indices.is_empty());
    }

    // --- VoltageMapper tests ---

    #[test]
    fn test_voltage_classify_normal() {
        let mapper = VoltageMapper::default();
        assert_eq!(mapper.classify(2.0), SubstrateClass::Normal);
        assert_eq!(mapper.classify(1.5), SubstrateClass::Normal);
    }

    #[test]
    fn test_voltage_classify_abnormal() {
        let mapper = VoltageMapper::default();
        assert_eq!(mapper.classify(1.0), SubstrateClass::Abnormal);
        assert_eq!(mapper.classify(0.5), SubstrateClass::Abnormal);
    }

    #[test]
    fn test_voltage_classify_scar() {
        let mapper = VoltageMapper::default();
        assert_eq!(mapper.classify(0.3), SubstrateClass::Scar);
        assert_eq!(mapper.classify(0.0), SubstrateClass::Scar);
    }

    #[test]
    fn test_voltage_measure_peak_to_peak() {
        let mapper = VoltageMapper::default();
        let egm = vec![-1.0, 0.5, 2.0, -0.5, 0.0];
        let result = mapper.measure(&egm);
        assert!((result.peak_to_peak_mv - 3.0).abs() < 1e-9);
        assert_eq!(result.classification, SubstrateClass::Normal);
    }

    #[test]
    fn test_voltage_measure_empty() {
        let mapper = VoltageMapper::default();
        let result = mapper.measure(&[]);
        assert_eq!(result.peak_to_peak_mv, 0.0);
        assert_eq!(result.classification, SubstrateClass::Scar);
    }

    #[test]
    fn test_voltage_map_electrodes() {
        let mapper = VoltageMapper::default();
        let egms = vec![
            vec![-1.0, 2.0],  // pp=3.0 -> Normal
            vec![-0.2, 0.5],  // pp=0.7 -> Abnormal
            vec![0.0, 0.1],   // pp=0.1 -> Scar
        ];
        let results = mapper.map_electrodes(&egms);
        assert_eq!(results.len(), 3);
        assert_eq!(results[0].classification, SubstrateClass::Normal);
        assert_eq!(results[1].classification, SubstrateClass::Abnormal);
        assert_eq!(results[2].classification, SubstrateClass::Scar);
    }

    #[test]
    fn test_voltage_custom_thresholds() {
        let mapper = VoltageMapper::new(0.3, 1.0);
        assert_eq!(mapper.classify(0.2), SubstrateClass::Scar);
        assert_eq!(mapper.classify(0.5), SubstrateClass::Abnormal);
        assert_eq!(mapper.classify(1.5), SubstrateClass::Normal);
    }

    // --- ConductionVelocityEstimator tests ---

    #[test]
    fn test_cv_basic_estimate() {
        let estimator = ConductionVelocityEstimator::new(4.0); // 4 mm spacing
        let result = estimator.estimate(0.000, 0.004).unwrap(); // 4 ms delay
        // CV = 4mm / 4ms = 1.0 m/s
        assert!((result.velocity_m_s - 1.0).abs() < 1e-6);
        assert!((result.distance_m - 0.004).abs() < 1e-9);
    }

    #[test]
    fn test_cv_too_small_delta() {
        let estimator = ConductionVelocityEstimator::new(4.0);
        // delta = 0.1 ms, below 0.5 ms minimum
        let result = estimator.estimate(0.0, 0.0001);
        assert!(result.is_none());
    }

    #[test]
    fn test_cv_exceeds_max() {
        let estimator = ConductionVelocityEstimator::new(4.0).with_max_cv(5.0);
        // Very short delta -> very high CV
        let result = estimator.estimate(0.0, 0.0005); // 0.5ms -> 8 m/s > 5 m/s max
        assert!(result.is_none());
    }

    #[test]
    fn test_cv_array_estimation() {
        let estimator = ConductionVelocityEstimator::new(4.0);
        // 4 electrodes, 4ms between each activation
        let lats = vec![0.000, 0.004, 0.008, 0.012];
        let results = estimator.estimate_array(&lats);
        assert_eq!(results.len(), 3);
        for r in &results {
            let cv = r.as_ref().unwrap().velocity_m_s;
            assert!((cv - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn test_cv_mean() {
        let estimator = ConductionVelocityEstimator::new(4.0);
        let lats = vec![0.000, 0.004, 0.008, 0.012];
        let mean = estimator.mean_cv(&lats).unwrap();
        assert!((mean - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_cv_empty_array() {
        let estimator = ConductionVelocityEstimator::new(4.0);
        let results = estimator.estimate_array(&[0.0]);
        assert!(results.is_empty());
        assert!(estimator.mean_cv(&[]).is_none());
    }

    // --- FractionationDetector tests ---

    #[test]
    fn test_fractionation_normal_egm() {
        let detector = FractionationDetector::new(2000.0)
            .with_amplitude_threshold(0.3);

        // Normal EGM: single clean deflection per beat
        let n = 5000; // 2.5 seconds
        let mut egm = vec![0.0; n];
        // One deflection per 300ms (200 BPM, definitely not fractionated)
        for i in (600..n).step_by(600) {
            egm[i] = 2.0;
        }

        let result = detector.analyze(&egm);
        // Should have few deflections with long intervals
        assert!(!result.is_fractionated);
    }

    #[test]
    fn test_fractionation_cfae() {
        let detector = FractionationDetector::new(2000.0)
            .with_amplitude_threshold(0.3)
            .with_min_deflection_count(4);

        // CFAE: many deflections with short intervals
        let n = 5000;
        let mut egm = vec![0.0; n];
        // Deflections every 50ms (20 Hz) — highly fractionated
        for i in (100..n).step_by(100) {
            egm[i] = 1.0;
        }

        let result = detector.analyze(&egm);
        assert!(result.deflection_count >= 4);
        assert!(result.mean_interval_s < 0.120);
        assert!(result.is_fractionated);
    }

    #[test]
    fn test_fractionation_intervals() {
        let detector = FractionationDetector::new(1000.0)
            .with_amplitude_threshold(0.1);

        let n = 2500;
        let mut egm = vec![0.0; n];
        // Three deflections at 100, 200, 300 ms (intervals of 100 ms)
        egm[100] = 1.0;
        egm[200] = 1.0;
        egm[300] = 1.0;

        let result = detector.analyze(&egm);
        assert_eq!(result.deflection_count, 3);
        assert_eq!(result.intervals_s.len(), 2);
        assert!((result.intervals_s[0] - 0.1).abs() < 0.01);
    }

    // --- DominantFrequencyAnalyzer tests ---

    #[test]
    fn test_df_pure_tone() {
        let fs = 200.0;
        let analyzer = DominantFrequencyAnalyzer::new(fs);

        // 7 Hz pure tone (typical AF frequency)
        let n = 400; // 2 seconds
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 7.0 * i as f64 / fs).sin())
            .collect();

        let result = analyzer.analyze(&signal);
        // DF should be close to 7 Hz
        assert!(
            (result.dominant_freq_hz - 7.0).abs() < 1.0,
            "Expected ~7 Hz, got {} Hz",
            result.dominant_freq_hz
        );
        assert!(result.peak_power > 0.0);
        assert!(result.organization_index > 0.3);
    }

    #[test]
    fn test_df_empty_signal() {
        let analyzer = DominantFrequencyAnalyzer::new(1000.0);
        let result = analyzer.analyze(&[]);
        assert_eq!(result.dominant_freq_hz, 0.0);
        assert_eq!(result.peak_power, 0.0);
    }

    #[test]
    fn test_df_custom_band() {
        let fs = 500.0;
        let analyzer = DominantFrequencyAnalyzer::new(fs).with_freq_range(5.0, 10.0);

        // Signal with two components: 3 Hz (outside band) and 8 Hz (inside band)
        let n = 500;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                5.0 * (2.0 * PI * 3.0 * t).sin() + 1.0 * (2.0 * PI * 8.0 * t).sin()
            })
            .collect();

        let result = analyzer.analyze(&signal);
        // DF within band should be near 8 Hz
        assert!(
            (result.dominant_freq_hz - 8.0).abs() < 2.0,
            "Expected ~8 Hz in band, got {} Hz",
            result.dominant_freq_hz
        );
    }

    #[test]
    fn test_df_organization_index() {
        let fs = 200.0;
        let analyzer = DominantFrequencyAnalyzer::new(fs);

        // Single strong tone -> high OI
        let n = 400;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 6.0 * i as f64 / fs).sin())
            .collect();
        let result = analyzer.analyze(&signal);
        assert!(result.organization_index > 0.2);
    }

    // --- PhaseMapper tests ---

    #[test]
    fn test_hilbert_transform_basic() {
        // Cosine should give analytic signal with sin as imaginary part
        let n = 64;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4.0 * i as f64 / n as f64).cos())
            .collect();

        let analytic = hilbert_transform(&signal);
        assert_eq!(analytic.len(), n);

        // Real part should match the original signal
        for i in 4..n - 4 {
            assert!(
                (analytic[i].0 - signal[i]).abs() < 0.2,
                "Real part mismatch at {}: {} vs {}",
                i,
                analytic[i].0,
                signal[i]
            );
        }

        // Imaginary part should be close to sin
        for i in 4..n - 4 {
            let expected_im = (2.0 * PI * 4.0 * i as f64 / n as f64).sin();
            assert!(
                (analytic[i].1 - expected_im).abs() < 0.3,
                "Imaginary part mismatch at {}: {} vs {}",
                i,
                analytic[i].1,
                expected_im
            );
        }
    }

    #[test]
    fn test_hilbert_empty() {
        let result = hilbert_transform(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_phase_mapper_monotonic_phase() {
        let fs = 200.0;
        let mapper = PhaseMapper::new(fs);

        // Pure tone should have monotonically changing phase (modulo 2*pi)
        let n = 100;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 5.0 * i as f64 / fs).cos())
            .collect();

        let result = mapper.compute_phase(&signal);
        assert_eq!(result.phase.len(), n);

        // Phase should be bounded in [-pi, pi]
        for &p in &result.phase {
            assert!(p >= -PI && p <= PI, "Phase {} out of range", p);
        }
    }

    #[test]
    fn test_phase_mapper_with_bandpass() {
        let fs = 200.0;
        let mapper = PhaseMapper::new(fs).with_bandpass(3.0, 15.0);
        assert_eq!(mapper.sample_rate(), fs);

        let n = 200;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 7.0 * i as f64 / fs).sin())
            .collect();

        let result = mapper.compute_phase(&signal);
        assert_eq!(result.phase.len(), n);
    }

    // --- RotorDetector tests ---

    #[test]
    fn test_rotor_no_singularity() {
        let detector = RotorDetector::new(3, 3);
        // Uniform phase -> no singularity
        let phase_map = vec![0.0; 9];
        let singularities = detector.detect(&phase_map, 0);
        assert!(singularities.is_empty());
    }

    #[test]
    fn test_rotor_positive_singularity() {
        let detector = RotorDetector::new(2, 2);
        // Create a phase pattern with +2*pi circulation
        // Clockwise traversal: 0 -> pi/2 -> pi -> -pi/2 -> 0 (total = +2*pi)
        let phase_map = vec![
            0.0,          PI / 2.0,
            -PI / 2.0,    PI,
        ];
        let singularities = detector.detect(&phase_map, 0);
        assert_eq!(singularities.len(), 1);
        assert_eq!(singularities[0].charge, 1);
    }

    #[test]
    fn test_rotor_negative_singularity() {
        let detector = RotorDetector::new(2, 2);
        // Phase pattern with -2*pi circulation
        let phase_map = vec![
            0.0,          -PI / 2.0,
            PI / 2.0,     PI,
        ];
        let singularities = detector.detect(&phase_map, 0);
        assert_eq!(singularities.len(), 1);
        assert_eq!(singularities[0].charge, -1);
    }

    #[test]
    fn test_rotor_sequence_detection() {
        let detector = RotorDetector::new(2, 2);
        let maps = vec![
            vec![0.0, PI / 2.0, -PI / 2.0, PI],
            vec![0.0, 0.0, 0.0, 0.0], // no singularity
        ];
        let all = detector.detect_sequence(&maps);
        assert_eq!(all.len(), 1);
        assert_eq!(all[0].time_index, 0);
    }

    #[test]
    fn test_rotor_larger_grid() {
        let detector = RotorDetector::new(4, 4);
        // Create a grid with a singularity in one cell
        let mut phase_map = vec![0.0; 16];
        // Place a singularity at cell (1,1)
        let idx = |r: usize, c: usize| r * 4 + c;
        phase_map[idx(1, 1)] = 0.0;
        phase_map[idx(1, 2)] = PI / 2.0;
        phase_map[idx(2, 2)] = PI;
        phase_map[idx(2, 1)] = -PI / 2.0;

        let singularities = detector.detect(&phase_map, 0);
        // Should find at least the singularity at (1,1)
        let has_singularity = singularities.iter().any(|s| s.row == 1 && s.col == 1);
        assert!(has_singularity, "Expected singularity at (1,1)");
    }

    // --- IsochronalMapper tests ---

    #[test]
    fn test_isochronal_empty_measurements() {
        let mapper = IsochronalMapper::new(4, 4);
        let map = mapper.generate(&[]);
        assert_eq!(map.rows, 4);
        assert_eq!(map.cols, 4);
        assert!(map.activation_times_ms.iter().all(|t| t.is_nan()));
        assert!(map.isochrone_levels_ms.is_empty());
    }

    #[test]
    fn test_isochronal_exact_points() {
        let mapper = IsochronalMapper::new(3, 3);
        // Measurements at grid points
        let measurements = vec![
            (0.0, 0.0, 0.0),
            (0.0, 2.0, 10.0),
            (2.0, 0.0, 20.0),
            (2.0, 2.0, 30.0),
        ];
        let map = mapper.generate(&measurements);
        // Exact corner points should have exact times
        assert!((map.activation_times_ms[0] - 0.0).abs() < 1e-6); // (0,0)
        assert!((map.activation_times_ms[2] - 10.0).abs() < 1e-6); // (0,2)
        assert!((map.activation_times_ms[6] - 20.0).abs() < 1e-6); // (2,0)
        assert!((map.activation_times_ms[8] - 30.0).abs() < 1e-6); // (2,2)
    }

    #[test]
    fn test_isochronal_interpolation() {
        let mapper = IsochronalMapper::new(3, 3);
        let measurements = vec![
            (0.0, 0.0, 0.0),
            (2.0, 2.0, 40.0),
        ];
        let map = mapper.generate(&measurements);
        // Center point (1,1) should be between 0 and 40
        let center = map.activation_times_ms[4]; // row 1, col 1
        assert!(center > 0.0 && center < 40.0, "Center should be interpolated, got {}", center);
    }

    #[test]
    fn test_isochronal_isochrone_levels() {
        let mapper = IsochronalMapper::new(3, 3).with_interval_ms(10.0);
        let measurements = vec![
            (0.0, 0.0, 5.0),
            (2.0, 2.0, 45.0),
        ];
        let map = mapper.generate(&measurements);
        // Levels should be: 10, 20, 30, 40
        assert!(map.isochrone_levels_ms.len() >= 3);
        assert!((map.isochrone_levels_ms[0] - 10.0).abs() < 1e-6);
    }

    // --- EntrainmentAnalyzer tests ---

    #[test]
    fn test_entrainment_within_circuit() {
        let analyzer = EntrainmentAnalyzer::default();
        let result = analyzer.analyze(310.0, 300.0, None);
        assert_eq!(result.classification, EntrainmentClass::WithinCircuit);
        assert!((result.ppi_tcl_diff_ms - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_entrainment_proximal() {
        let analyzer = EntrainmentAnalyzer::default();
        let result = analyzer.analyze(345.0, 300.0, None);
        assert_eq!(result.classification, EntrainmentClass::Proximal);
    }

    #[test]
    fn test_entrainment_remote() {
        let analyzer = EntrainmentAnalyzer::default();
        let result = analyzer.analyze(400.0, 300.0, None);
        assert_eq!(result.classification, EntrainmentClass::Remote);
    }

    #[test]
    fn test_entrainment_terminated() {
        let analyzer = EntrainmentAnalyzer::default();
        let result = analyzer.analyze(280.0, 300.0, None);
        assert_eq!(result.classification, EntrainmentClass::Terminated);
    }

    #[test]
    fn test_entrainment_with_stim_qrs() {
        let analyzer = EntrainmentAnalyzer::default();
        let result = analyzer.analyze(310.0, 300.0, Some(40.0));
        assert_eq!(result.stim_qrs_ms, Some(40.0));
        assert_eq!(result.classification, EntrainmentClass::WithinCircuit);
    }

    #[test]
    fn test_entrainment_from_activations() {
        let analyzer = EntrainmentAnalyzer::default();
        // TCL ~300ms, then 2 pacing beats, return at TCL+20ms (within circuit)
        let activations = vec![
            0.0, 300.0, 600.0, 900.0,  // 4 TCL beats (300ms each)
            1180.0, 1460.0,             // 2 pacing beats (280ms)
            1780.0,                     // return beat: PPI = 1780-1460 = 320ms
        ];
        let result = analyzer.analyze_from_activations(&activations, 2);
        assert!(result.is_some());
        let r = result.unwrap();
        assert!((r.tcl_ms - 300.0).abs() < 1.0);
        assert!((r.ppi_ms - 320.0).abs() < 1e-6);
    }

    #[test]
    fn test_entrainment_from_activations_insufficient() {
        let analyzer = EntrainmentAnalyzer::default();
        let result = analyzer.analyze_from_activations(&[0.0, 300.0], 2);
        assert!(result.is_none());
    }

    #[test]
    fn test_entrainment_custom_thresholds() {
        let analyzer = EntrainmentAnalyzer::new(20.0, 50.0);
        // 25ms diff: proximal with these thresholds
        let result = analyzer.analyze(325.0, 300.0, None);
        assert_eq!(result.classification, EntrainmentClass::Proximal);
    }

    // --- wrap_phase tests ---

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0)).abs() < 1e-10);
        assert!((wrap_phase(PI) - PI).abs() < 1e-10);
        assert!((wrap_phase(-PI) - (-PI)).abs() < 1e-10);
        assert!((wrap_phase(3.0 * PI) - PI).abs() < 1e-10);
        assert!((wrap_phase(-3.0 * PI) - (-PI)).abs() < 1e-10);
    }

    // --- bandpass_filter tests ---

    #[test]
    fn test_bandpass_filter_empty() {
        let result = bandpass_filter(&[], 1000.0, 3.0, 15.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_bandpass_filter_passes_inband() {
        let fs = 200.0;
        let n = 200;
        // 7 Hz tone (within 3-15 Hz band)
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 7.0 * i as f64 / fs).sin())
            .collect();

        let filtered = bandpass_filter(&signal, fs, 3.0, 15.0);
        assert_eq!(filtered.len(), n);

        // Should retain most of the energy
        let input_power: f64 = signal.iter().map(|x| x * x).sum::<f64>() / n as f64;
        let output_power: f64 = filtered.iter().map(|x| x * x).sum::<f64>() / n as f64;
        assert!(
            output_power > 0.3 * input_power,
            "In-band signal should pass: in={}, out={}",
            input_power,
            output_power
        );
    }

    // --- Integration / combined workflow test ---

    #[test]
    fn test_substrate_mapping_workflow() {
        // Simulate a multi-electrode substrate map
        let config = ElectrogramConfig::new(16, 2000.0, 4.0, CatheterType::Grid);
        let mapper = VoltageMapper::default();

        // Generate synthetic bipolar EGMs with varying voltage
        let voltages_mv = [2.5, 1.8, 1.2, 0.8, 0.3, 0.1, 1.6, 2.0];
        let egms: Vec<Vec<f64>> = voltages_mv
            .iter()
            .map(|&v| {
                (0..200)
                    .map(|i| {
                        if i == 100 {
                            v / 2.0
                        } else if i == 101 {
                            -v / 2.0
                        } else {
                            0.01
                        }
                    })
                    .collect()
            })
            .collect();

        let results = mapper.map_electrodes(&egms);
        assert_eq!(results.len(), 8);

        // Count classifications
        let normal = results
            .iter()
            .filter(|r| r.classification == SubstrateClass::Normal)
            .count();
        let scar = results
            .iter()
            .filter(|r| r.classification == SubstrateClass::Scar)
            .count();
        assert!(normal > 0, "Should have normal tissue: {:?}", config.catheter_type);
        assert!(scar > 0);
    }

    #[test]
    fn test_conduction_velocity_clinical_range() {
        // Test that typical clinical values produce reasonable CV
        let estimator = ConductionVelocityEstimator::new(4.0); // 4mm PentaRay spacing

        // Atrial conduction: ~0.5-1.0 m/s
        // 4mm / 0.005s = 0.8 m/s
        let result = estimator.estimate(0.000, 0.005).unwrap();
        assert!(
            result.velocity_m_s > 0.3 && result.velocity_m_s < 2.0,
            "Atrial CV should be 0.5-1.0 m/s, got {} m/s",
            result.velocity_m_s
        );
    }
}
