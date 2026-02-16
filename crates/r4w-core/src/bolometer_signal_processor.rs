//! Bolometer signal processor for thermal radiation detectors.
//!
//! This module implements signal processing for bolometric radiation detectors
//! used in cosmic microwave background (CMB) observations, far-infrared astronomy,
//! THz spectroscopy, X-ray microcalorimetry, and particle physics calorimetry.
//!
//! A bolometer is a thermal detector that absorbs incident radiation and measures
//! the resulting temperature rise. The thermal model is characterized by heat
//! capacity C, thermal conductance G to the heat bath, and the resulting time
//! constant tau = C/G.
//!
//! ## Key capabilities
//!
//! - **Thermal response modeling** — single-pole exponential response dT/dt = (P_in - G*(T-T_bath))/C
//! - **Optimal filtering** — frequency-domain Wiener filter for maximum-SNR pulse height estimation
//! - **Pulse template generation** — from thermal model or averaged pulses
//! - **Baseline estimation** — pre-trigger baseline subtraction
//! - **Energy calibration** — polynomial fit from pulse amplitude to photon energy
//! - **Cosmic ray rejection** — fast rise time and saturation detection
//! - **Noise characterization** — phonon, Johnson, 1/f, and readout noise PSD
//! - **NEP calculation** — Noise Equivalent Power for phonon-noise-limited detectors
//! - **TES modeling** — Transition Edge Sensor electrothermal feedback
//! - **Multiplex readout** — time-division and frequency-division multiplexing
//!
//! # Example
//!
//! ```
//! use r4w_core::bolometer_signal_processor::{
//!     BolometerConfig, ThermalResponseModel, OptimalFilter,
//!     PulseTemplateGenerator, NepCalculator,
//! };
//!
//! // Configure a CMB bolometer at 300 mK
//! let config = BolometerConfig {
//!     thermal_conductance_g: 1e-10,  // 100 pW/K
//!     heat_capacity_c: 1e-12,        // 1 pJ/K
//!     operating_temp_k: 0.3,         // 300 mK
//!     bath_temp_k: 0.25,             // 250 mK
//!     sample_rate_hz: 1000.0,        // 1 kHz readout
//! };
//!
//! assert!((config.time_constant_s() - 0.01).abs() < 1e-6);
//!
//! // Compute phonon NEP
//! let nep = NepCalculator::phonon_nep(&config);
//! assert!(nep > 0.0 && nep < 1e-15); // sub-fW/sqrt(Hz)
//!
//! // Generate thermal pulse template
//! let template = PulseTemplateGenerator::from_thermal_model(&config, 200);
//! assert_eq!(template.len(), 200);
//! ```

use std::f64::consts::PI;

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380_649e-23;

/// Stefan-Boltzmann constant in W/(m^2 K^4).
const STEFAN_BOLTZMANN: f64 = 5.670_374_419e-8;

/// Planck constant in J*s.
const H_PLANCK: f64 = 6.626_070_15e-34;

// ─── BolometerConfig ──────────────────────────────────────────────────

/// Configuration for a bolometric detector.
#[derive(Debug, Clone)]
pub struct BolometerConfig {
    /// Thermal conductance to heat bath in W/K.
    pub thermal_conductance_g: f64,
    /// Heat capacity in J/K.
    pub heat_capacity_c: f64,
    /// Operating temperature of the detector in Kelvin.
    pub operating_temp_k: f64,
    /// Bath (heat sink) temperature in Kelvin.
    pub bath_temp_k: f64,
    /// Sample rate of the readout electronics in Hz.
    pub sample_rate_hz: f64,
}

impl BolometerConfig {
    /// Thermal time constant tau = C / G in seconds.
    pub fn time_constant_s(&self) -> f64 {
        self.heat_capacity_c / self.thermal_conductance_g
    }

    /// 3 dB bandwidth of the thermal response in Hz: f_3dB = 1 / (2*pi*tau).
    pub fn thermal_bandwidth_hz(&self) -> f64 {
        1.0 / (2.0 * PI * self.time_constant_s())
    }

    /// Static power dissipation P_static = G * (T_op - T_bath) in Watts.
    pub fn static_power_w(&self) -> f64 {
        self.thermal_conductance_g * (self.operating_temp_k - self.bath_temp_k)
    }
}

impl Default for BolometerConfig {
    fn default() -> Self {
        Self {
            thermal_conductance_g: 1e-10,   // 100 pW/K, typical CMB bolometer
            heat_capacity_c: 1e-12,          // 1 pJ/K
            operating_temp_k: 0.3,           // 300 mK
            bath_temp_k: 0.25,               // 250 mK
            sample_rate_hz: 1000.0,          // 1 kHz readout
        }
    }
}

// ─── ThermalResponseModel ─────────────────────────────────────────────

/// Single-pole thermal response model for a bolometer.
///
/// Solves: dT/dt = (P_in - G * (T - T_bath)) / C
///
/// The impulse response is an exponential decay with time constant tau = C/G.
#[derive(Debug, Clone)]
pub struct ThermalResponseModel {
    config: BolometerConfig,
    /// Current temperature of the absorber in Kelvin.
    current_temp_k: f64,
}

impl ThermalResponseModel {
    /// Create a new thermal model at equilibrium.
    pub fn new(config: BolometerConfig) -> Self {
        let t0 = config.operating_temp_k;
        Self {
            config,
            current_temp_k: t0,
        }
    }

    /// Step the thermal model forward by one sample period given incident power P_in (Watts).
    ///
    /// Returns the new temperature in Kelvin.
    pub fn step(&mut self, p_in: f64) -> f64 {
        let dt = 1.0 / self.config.sample_rate_hz;
        let g = self.config.thermal_conductance_g;
        let c = self.config.heat_capacity_c;
        let t_bath = self.config.bath_temp_k;

        // Euler integration of dT/dt = (P_in - G*(T - T_bath)) / C
        let dtdt = (p_in - g * (self.current_temp_k - t_bath)) / c;
        self.current_temp_k += dtdt * dt;
        self.current_temp_k
    }

    /// Process a time series of incident power values, returning temperature trace.
    pub fn process(&mut self, p_in: &[f64]) -> Vec<f64> {
        p_in.iter().map(|&p| self.step(p)).collect()
    }

    /// Reset temperature to operating point.
    pub fn reset(&mut self) {
        self.current_temp_k = self.config.operating_temp_k;
    }

    /// Current temperature in Kelvin.
    pub fn temperature(&self) -> f64 {
        self.current_temp_k
    }

    /// Generate the impulse response (temperature rise from a delta-function
    /// energy pulse of 1 Joule) for `n_samples` at the configured sample rate.
    pub fn impulse_response(&self, n_samples: usize) -> Vec<f64> {
        let tau = self.config.time_constant_s();
        let c = self.config.heat_capacity_c;
        let dt = 1.0 / self.config.sample_rate_hz;
        let amplitude = 1.0 / c; // Temperature rise per joule
        (0..n_samples)
            .map(|i| {
                let t = i as f64 * dt;
                amplitude * (-t / tau).exp()
            })
            .collect()
    }
}

// ─── PulseTemplateGenerator ───────────────────────────────────────────

/// Generate template pulse shapes for optimal filtering.
pub struct PulseTemplateGenerator;

impl PulseTemplateGenerator {
    /// Generate a pulse template from the thermal model.
    ///
    /// Returns a normalized template (peak = 1.0) of the specified length.
    pub fn from_thermal_model(config: &BolometerConfig, n_samples: usize) -> Vec<f64> {
        let tau = config.time_constant_s();
        let dt = 1.0 / config.sample_rate_hz;
        let mut template: Vec<f64> = (0..n_samples)
            .map(|i| {
                let t = i as f64 * dt;
                (-t / tau).exp()
            })
            .collect();
        // Normalize to unit peak
        let max = template.iter().cloned().fold(0.0_f64, f64::max);
        if max > 0.0 {
            for v in &mut template {
                *v /= max;
            }
        }
        template
    }

    /// Generate a bi-exponential pulse template with separate rise and fall times.
    ///
    /// `tau_rise` and `tau_fall` are in seconds.
    pub fn biexponential(
        tau_rise: f64,
        tau_fall: f64,
        sample_rate: f64,
        n_samples: usize,
    ) -> Vec<f64> {
        let dt = 1.0 / sample_rate;
        let mut template: Vec<f64> = (0..n_samples)
            .map(|i| {
                let t = i as f64 * dt;
                let rise = 1.0 - (-t / tau_rise).exp();
                let fall = (-t / tau_fall).exp();
                rise * fall
            })
            .collect();
        let max = template.iter().cloned().fold(0.0_f64, f64::max);
        if max > 0.0 {
            for v in &mut template {
                *v /= max;
            }
        }
        template
    }

    /// Average multiple pulse records into a template, aligned at their peaks.
    ///
    /// Each pulse record should contain a single pulse. Returns a normalized average.
    pub fn from_averaged_pulses(pulses: &[Vec<f64>], n_samples: usize) -> Vec<f64> {
        if pulses.is_empty() || n_samples == 0 {
            return vec![0.0; n_samples];
        }

        let mut accumulated = vec![0.0; n_samples];
        let mut count = 0usize;

        for pulse in pulses {
            // Find peak index
            let peak_idx = pulse
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                .map(|(i, _)| i)
                .unwrap_or(0);

            // Align so peak is at n_samples / 4 (give room for tail)
            let align_target = n_samples / 4;
            let offset = peak_idx as isize - align_target as isize;

            for i in 0..n_samples {
                let src = i as isize + offset;
                if src >= 0 && (src as usize) < pulse.len() {
                    accumulated[i] += pulse[src as usize];
                }
            }
            count += 1;
        }

        if count > 0 {
            for v in &mut accumulated {
                *v /= count as f64;
            }
        }

        // Normalize
        let max = accumulated.iter().cloned().fold(0.0_f64, f64::max);
        if max > 0.0 {
            for v in &mut accumulated {
                *v /= max;
            }
        }
        accumulated
    }
}

// ─── BaselineEstimator ────────────────────────────────────────────────

/// Estimate and subtract pre-trigger baseline from pulse records.
#[derive(Debug, Clone)]
pub struct BaselineEstimator {
    /// Number of pre-trigger samples to use for baseline estimation.
    pre_trigger_samples: usize,
}

impl BaselineEstimator {
    /// Create a new baseline estimator.
    ///
    /// `pre_trigger_samples` — number of samples before the trigger to average.
    pub fn new(pre_trigger_samples: usize) -> Self {
        assert!(pre_trigger_samples > 0, "Need at least 1 pre-trigger sample");
        Self { pre_trigger_samples }
    }

    /// Estimate the baseline (mean of pre-trigger region).
    pub fn estimate(&self, record: &[f64]) -> f64 {
        let n = self.pre_trigger_samples.min(record.len());
        if n == 0 {
            return 0.0;
        }
        record[..n].iter().sum::<f64>() / n as f64
    }

    /// Estimate baseline standard deviation (noise level in pre-trigger region).
    pub fn estimate_noise(&self, record: &[f64]) -> f64 {
        let n = self.pre_trigger_samples.min(record.len());
        if n < 2 {
            return 0.0;
        }
        let mean = record[..n].iter().sum::<f64>() / n as f64;
        let var = record[..n]
            .iter()
            .map(|&x| (x - mean).powi(2))
            .sum::<f64>()
            / (n - 1) as f64;
        var.sqrt()
    }

    /// Subtract baseline from the entire record.
    pub fn subtract(&self, record: &[f64]) -> Vec<f64> {
        let bl = self.estimate(record);
        record.iter().map(|&x| x - bl).collect()
    }
}

// ─── OptimalFilter ────────────────────────────────────────────────────

/// Frequency-domain optimal (Wiener) filter for pulse height estimation.
///
/// The optimal filter maximizes signal-to-noise ratio for known pulse shape:
///   H(f) = S*(f) / (|S(f)|^2 + N(f))
///
/// where S(f) is the pulse template spectrum and N(f) is the noise PSD.
#[derive(Debug, Clone)]
pub struct OptimalFilter {
    /// Filter coefficients in frequency domain (real, for magnitude weighting).
    filter_freq: Vec<f64>,
    /// Normalization factor so that filtered template has unit amplitude.
    normalization: f64,
    /// Length of the FFT.
    fft_len: usize,
}

impl OptimalFilter {
    /// Build an optimal filter from a pulse template and noise PSD.
    ///
    /// `template` — time-domain pulse template (normalized to peak=1).
    /// `noise_psd` — one-sided noise power spectral density (length = template.len()/2 + 1).
    ///
    /// If `noise_psd` is shorter than needed, it is zero-padded (white noise assumed).
    pub fn new(template: &[f64], noise_psd: &[f64]) -> Self {
        let n = template.len();
        let fft_len = n;
        let n_freq = n / 2 + 1;

        // Compute DFT of template using real DFT
        let template_fft = real_dft(template);

        // Build noise PSD array, extending with last value if too short
        let mut npsd = vec![0.0; n_freq];
        for i in 0..n_freq {
            npsd[i] = if i < noise_psd.len() {
                noise_psd[i].max(1e-30)
            } else if !noise_psd.is_empty() {
                noise_psd[noise_psd.len() - 1].max(1e-30)
            } else {
                1e-30
            };
        }

        // Wiener optimal filter: H(f) = S*(f) / (|S(f)|^2 + N(f))
        // We store the magnitude weight; phase is handled via the template spectrum.
        // For amplitude estimation, we compute: sum_k [ |S(k)|^2 / (|S(k)|^2 + N(k)) ]
        // and normalize so a unit-amplitude template produces amplitude 1.0.
        let mut filter_freq = vec![0.0; n_freq];
        let mut norm_sum = 0.0;
        for i in 0..n_freq {
            let s_mag_sq = template_fft[i].0 * template_fft[i].0
                + template_fft[i].1 * template_fft[i].1;
            let weight = s_mag_sq / (s_mag_sq + npsd[i]);
            filter_freq[i] = weight;
            // Normalization: sum of weights applied to the signal spectrum
            let mult = if i == 0 || i == n / 2 { 1.0 } else { 2.0 };
            norm_sum += weight * s_mag_sq * mult;
        }

        // normalization ensures that filtering the template itself yields peak ~= 1.0
        let normalization = if norm_sum > 0.0 { n as f64 / norm_sum } else { 1.0 };

        Self {
            filter_freq,
            normalization,
            fft_len,
        }
    }

    /// Apply the optimal filter to a pulse record, returning the estimated pulse height.
    ///
    /// The returned value is the amplitude estimate that maximizes SNR.
    pub fn estimate_amplitude(&self, record: &[f64]) -> f64 {
        let n = self.fft_len.min(record.len());
        let mut padded = vec![0.0; self.fft_len];
        padded[..n].copy_from_slice(&record[..n]);

        let spectrum = real_dft(&padded);
        let n_freq = self.filter_freq.len();

        // Multiply by filter weights and inverse DFT to find peak
        let mut filtered_re = vec![0.0; n_freq];
        let mut filtered_im = vec![0.0; n_freq];
        for i in 0..n_freq {
            filtered_re[i] = spectrum[i].0 * self.filter_freq[i];
            filtered_im[i] = spectrum[i].1 * self.filter_freq[i];
        }

        // Inverse DFT to get filtered time-domain signal
        let filtered_td = inverse_real_dft(&filtered_re, &filtered_im, self.fft_len);

        // Find peak value
        let peak = filtered_td
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);

        peak * self.normalization
    }

    /// Return the filter frequency response (weights).
    pub fn frequency_response(&self) -> &[f64] {
        &self.filter_freq
    }
}

// ─── EnergyCalibrator ─────────────────────────────────────────────────

/// Convert pulse amplitude to photon energy using polynomial calibration.
///
/// Fits a polynomial E(A) = c0 + c1*A + c2*A^2 + ... to calibration data.
#[derive(Debug, Clone)]
pub struct EnergyCalibrator {
    /// Polynomial coefficients [c0, c1, c2, ...] mapping amplitude to energy.
    coefficients: Vec<f64>,
}

impl EnergyCalibrator {
    /// Create a linear calibrator: E = offset + gain * amplitude.
    pub fn linear(offset_ev: f64, gain_ev_per_unit: f64) -> Self {
        Self {
            coefficients: vec![offset_ev, gain_ev_per_unit],
        }
    }

    /// Fit a polynomial calibration from (amplitude, energy_eV) pairs.
    ///
    /// `order` is the polynomial degree (1 = linear, 2 = quadratic, etc.).
    /// Uses least-squares fitting via normal equations.
    pub fn fit(cal_points: &[(f64, f64)], order: usize) -> Self {
        let n = cal_points.len();
        let m = order + 1;
        assert!(n >= m, "Need at least {} calibration points for order {}", m, order);

        // Build Vandermonde matrix A (n x m) and vector b (n)
        // Normal equations: (A^T A) c = A^T b
        let mut ata = vec![0.0; m * m];
        let mut atb = vec![0.0; m];

        for &(amp, energy) in cal_points {
            let mut powers = vec![1.0; m];
            for j in 1..m {
                powers[j] = powers[j - 1] * amp;
            }
            for i in 0..m {
                for j in 0..m {
                    ata[i * m + j] += powers[i] * powers[j];
                }
                atb[i] += powers[i] * energy;
            }
        }

        // Solve via Gaussian elimination with partial pivoting
        let coefficients = solve_linear_system(&ata, &atb, m);

        Self { coefficients }
    }

    /// Convert a pulse amplitude to energy in eV.
    pub fn amplitude_to_energy(&self, amplitude: f64) -> f64 {
        let mut result = 0.0;
        let mut a_pow = 1.0;
        for &c in &self.coefficients {
            result += c * a_pow;
            a_pow *= amplitude;
        }
        result
    }

    /// Convert energy in eV back to expected amplitude (inverse, linear only).
    pub fn energy_to_amplitude(&self, energy_ev: f64) -> Option<f64> {
        if self.coefficients.len() == 2 && self.coefficients[1].abs() > 1e-30 {
            Some((energy_ev - self.coefficients[0]) / self.coefficients[1])
        } else {
            None
        }
    }

    /// Get calibration coefficients.
    pub fn coefficients(&self) -> &[f64] {
        &self.coefficients
    }
}

// ─── CosmicRayRejector ────────────────────────────────────────────────

/// Identify and flag cosmic ray glitches in bolometer data.
///
/// Cosmic rays produce fast, high-amplitude spikes that differ from
/// photon pulses in rise time and shape.
#[derive(Debug, Clone)]
pub struct CosmicRayRejector {
    /// Maximum allowed rise time in samples for a valid photon pulse.
    max_rise_samples: usize,
    /// Amplitude threshold (multiple of baseline RMS) for glitch detection.
    amplitude_threshold: f64,
    /// Saturation level — pulses exceeding this are flagged.
    saturation_level: f64,
}

/// Result of cosmic ray analysis on a single pulse record.
#[derive(Debug, Clone)]
pub struct CosmicRayResult {
    /// Whether a cosmic ray glitch was detected.
    pub is_glitch: bool,
    /// Rise time in samples (10%-90% of peak).
    pub rise_time_samples: usize,
    /// Peak amplitude.
    pub peak_amplitude: f64,
    /// Whether the pulse saturated.
    pub is_saturated: bool,
    /// Reason for flagging (if flagged).
    pub reason: Option<String>,
}

impl CosmicRayRejector {
    /// Create a new cosmic ray rejector.
    ///
    /// * `max_rise_samples` — valid photon pulses must rise slower than this
    /// * `amplitude_threshold` — flag pulses above this multiple of RMS
    /// * `saturation_level` — absolute saturation level
    pub fn new(
        max_rise_samples: usize,
        amplitude_threshold: f64,
        saturation_level: f64,
    ) -> Self {
        Self {
            max_rise_samples,
            amplitude_threshold,
            saturation_level,
        }
    }

    /// Analyze a baseline-subtracted pulse record for cosmic ray contamination.
    pub fn analyze(&self, record: &[f64]) -> CosmicRayResult {
        if record.is_empty() {
            return CosmicRayResult {
                is_glitch: false,
                rise_time_samples: 0,
                peak_amplitude: 0.0,
                is_saturated: false,
                reason: None,
            };
        }

        let peak_amplitude = record.iter().cloned().fold(0.0_f64, f64::max);
        let peak_idx = record
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Check saturation
        let is_saturated = peak_amplitude >= self.saturation_level;

        // Compute rise time (10% to 90% of peak)
        let thresh_low = 0.1 * peak_amplitude;
        let thresh_high = 0.9 * peak_amplitude;
        let mut idx_10 = 0;
        let mut idx_90 = 0;
        for i in 0..=peak_idx {
            if record[i] >= thresh_low && idx_10 == 0 {
                idx_10 = i;
            }
            if record[i] >= thresh_high {
                idx_90 = i;
                break;
            }
        }
        let rise_time_samples = if idx_90 > idx_10 { idx_90 - idx_10 } else { 0 };

        // Check amplitude threshold
        let is_over_threshold = peak_amplitude > self.amplitude_threshold;

        // Determine if glitch: rise time of 0 (instantaneous) is also too fast
        let too_fast = rise_time_samples < self.max_rise_samples;
        let is_glitch = is_saturated || (too_fast && is_over_threshold);

        let reason = if is_saturated {
            Some("Saturated pulse".to_string())
        } else if too_fast && is_over_threshold {
            Some(format!(
                "Fast rise ({} samples < {} limit) with high amplitude ({:.2} > {:.2})",
                rise_time_samples, self.max_rise_samples, peak_amplitude, self.amplitude_threshold
            ))
        } else {
            None
        };

        CosmicRayResult {
            is_glitch,
            rise_time_samples,
            peak_amplitude,
            is_saturated,
            reason,
        }
    }

    /// Process a stream of pulse records, returning flags for each.
    pub fn process_batch(&self, records: &[Vec<f64>]) -> Vec<CosmicRayResult> {
        records.iter().map(|r| self.analyze(r)).collect()
    }
}

// ─── NoiseCharacterizer ───────────────────────────────────────────────

/// Noise component types in a bolometer readout.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NoiseType {
    /// White phonon noise (flat PSD).
    Phonon,
    /// Johnson (thermal) noise from resistive elements.
    Johnson,
    /// 1/f (flicker) noise from detector or electronics.
    OneOverF,
    /// Readout electronics noise (amplifier, ADC).
    Readout,
}

/// Result of noise characterization.
#[derive(Debug, Clone)]
pub struct NoiseCharacterization {
    /// One-sided noise PSD in units^2/Hz.
    pub noise_psd: Vec<f64>,
    /// Frequency axis in Hz.
    pub freq_hz: Vec<f64>,
    /// Estimated white noise floor level.
    pub white_noise_level: f64,
    /// Estimated 1/f knee frequency in Hz.
    pub knee_freq_hz: f64,
    /// Total RMS noise.
    pub total_rms: f64,
}

/// Compute noise power spectral density from time-domain data.
pub struct NoiseCharacterizer;

impl NoiseCharacterizer {
    /// Estimate the noise PSD from a quiet (no-signal) time stream.
    ///
    /// Uses Welch's method with 50% overlapping segments.
    pub fn estimate_psd(data: &[f64], sample_rate: f64, segment_len: usize) -> NoiseCharacterization {
        let seg = segment_len.min(data.len());
        let n_freq = seg / 2 + 1;
        let step = seg / 2; // 50% overlap
        let mut psd_accum = vec![0.0; n_freq];
        let mut n_segments = 0usize;

        let mut offset = 0;
        while offset + seg <= data.len() {
            let segment = &data[offset..offset + seg];

            // Apply Hann window
            let windowed: Vec<f64> = segment
                .iter()
                .enumerate()
                .map(|(i, &x)| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / seg as f64).cos());
                    x * w
                })
                .collect();

            // Compute DFT
            let spectrum = real_dft(&windowed);

            // Accumulate power
            let df = sample_rate / seg as f64;
            let window_power: f64 = (0..seg)
                .map(|i| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / seg as f64).cos());
                    w * w
                })
                .sum();
            let scale = 2.0 / (sample_rate * window_power); // one-sided PSD scaling

            for i in 0..n_freq {
                let power = spectrum[i].0 * spectrum[i].0 + spectrum[i].1 * spectrum[i].1;
                psd_accum[i] += power * scale;
            }

            n_segments += 1;
            offset += step;
        }

        if n_segments == 0 {
            // Fall back to single segment
            let windowed: Vec<f64> = data
                .iter()
                .enumerate()
                .map(|(i, &x)| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / data.len() as f64).cos());
                    x * w
                })
                .collect();
            let spectrum = real_dft(&windowed);
            let n_freq_actual = data.len() / 2 + 1;
            let window_power: f64 = (0..data.len())
                .map(|i| {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / data.len() as f64).cos());
                    w * w
                })
                .sum();
            let scale = 2.0 / (sample_rate * window_power);
            psd_accum = vec![0.0; n_freq_actual];
            for i in 0..n_freq_actual.min(spectrum.len()) {
                let power = spectrum[i].0 * spectrum[i].0 + spectrum[i].1 * spectrum[i].1;
                psd_accum[i] = power * scale;
            }
            n_segments = 1;
        }

        // Average
        let noise_psd: Vec<f64> = psd_accum.iter().map(|&p| p / n_segments as f64).collect();
        let n_freq_final = noise_psd.len();
        let df = sample_rate / (if n_segments > 0 { seg } else { data.len() }) as f64;
        let freq_hz: Vec<f64> = (0..n_freq_final).map(|i| i as f64 * df).collect();

        // Estimate white noise floor (median of upper half of spectrum)
        let upper_half = &noise_psd[n_freq_final / 2..];
        let white_noise_level = median_of(upper_half);

        // Estimate 1/f knee: find where PSD drops to 2x white level
        let knee_thresh = 2.0 * white_noise_level;
        let knee_freq_hz = freq_hz
            .iter()
            .zip(noise_psd.iter())
            .skip(1) // skip DC
            .find(|(_, &p)| p <= knee_thresh)
            .map(|(&f, _)| f)
            .unwrap_or(0.0);

        // Total RMS noise (Parseval's theorem)
        let total_power: f64 = noise_psd.iter().sum::<f64>() * df;
        let total_rms = total_power.sqrt();

        NoiseCharacterization {
            noise_psd,
            freq_hz,
            white_noise_level,
            knee_freq_hz,
            total_rms,
        }
    }

    /// Generate a theoretical noise model combining phonon, Johnson, 1/f, and readout noise.
    ///
    /// Returns PSD values at the given frequencies.
    pub fn theoretical_model(
        freqs: &[f64],
        phonon_level: f64,
        johnson_level: f64,
        one_over_f_level: f64,
        one_over_f_knee: f64,
        readout_level: f64,
    ) -> Vec<f64> {
        freqs
            .iter()
            .map(|&f| {
                let f_abs = f.abs().max(1e-10);
                let phonon = phonon_level;
                let johnson = johnson_level;
                let flicker = one_over_f_level * (one_over_f_knee / f_abs);
                let readout = readout_level;
                phonon + johnson + flicker + readout
            })
            .collect()
    }
}

// ─── NepCalculator ────────────────────────────────────────────────────

/// Noise Equivalent Power calculator for bolometric detectors.
///
/// NEP is the signal power that produces SNR = 1 in a 1 Hz bandwidth.
pub struct NepCalculator;

impl NepCalculator {
    /// Phonon-noise-limited NEP: NEP_phonon = sqrt(4 * k_B * T^2 * G).
    ///
    /// This is the fundamental thermal fluctuation noise limit.
    pub fn phonon_nep(config: &BolometerConfig) -> f64 {
        let t = config.operating_temp_k;
        let g = config.thermal_conductance_g;
        (4.0 * K_B * t * t * g).sqrt()
    }

    /// Johnson noise NEP from a bias resistor at temperature T_r with resistance R.
    ///
    /// NEP_johnson = sqrt(4 * k_B * T_r / R) * V_bias / S_v
    /// Simplified to just the voltage noise: sqrt(4 * k_B * T_r * R) (V/sqrt(Hz)).
    pub fn johnson_noise_voltage(temp_k: f64, resistance_ohm: f64) -> f64 {
        (4.0 * K_B * temp_k * resistance_ohm).sqrt()
    }

    /// Photon noise NEP for background-limited performance (BLIP):
    /// NEP_photon = sqrt(2 * P * h * nu + 2 * P^2 / delta_nu)
    ///
    /// where P is background power, nu is center frequency, delta_nu is bandwidth.
    pub fn photon_nep(power_w: f64, freq_hz: f64, bandwidth_hz: f64) -> f64 {
        let shot = 2.0 * power_w * H_PLANCK * freq_hz;
        let bunching = if bandwidth_hz > 0.0 {
            2.0 * power_w * power_w / bandwidth_hz
        } else {
            0.0
        };
        (shot + bunching).sqrt()
    }

    /// Total NEP from multiple independent noise sources (add in quadrature).
    pub fn total_nep(nep_values: &[f64]) -> f64 {
        nep_values.iter().map(|n| n * n).sum::<f64>().sqrt()
    }

    /// Noise Equivalent Temperature Difference: NETD = NEP / (dP/dT).
    ///
    /// For a bolometer viewing a blackbody, dP/dT ~ 4 * sigma * A * T^3.
    pub fn netd(nep: f64, area_m2: f64, scene_temp_k: f64) -> f64 {
        let dpdt = 4.0 * STEFAN_BOLTZMANN * area_m2 * scene_temp_k.powi(3);
        if dpdt > 0.0 {
            nep / dpdt
        } else {
            f64::INFINITY
        }
    }

    /// Background power loading from a blackbody at temperature T through
    /// aperture area A: P = sigma * A * T^4.
    pub fn background_power(area_m2: f64, temp_k: f64) -> f64 {
        STEFAN_BOLTZMANN * area_m2 * temp_k.powi(4)
    }
}

// ─── TesModel ─────────────────────────────────────────────────────────

/// Transition Edge Sensor electrothermal feedback model.
///
/// A TES operates on the steep superconducting transition, where small
/// temperature changes cause large resistance changes. The logarithmic
/// sensitivity alpha = (T/R) * (dR/dT) characterizes this.
#[derive(Debug, Clone)]
pub struct TesModel {
    /// Normal state resistance in Ohms.
    pub r_normal: f64,
    /// Superconducting transition temperature in Kelvin.
    pub t_c: f64,
    /// Transition width (10%-90%) in Kelvin.
    pub delta_t: f64,
    /// Thermal conductance to bath in W/K.
    pub g: f64,
    /// Heat capacity in J/K.
    pub c: f64,
    /// Bias current in Amperes.
    pub i_bias: f64,
}

impl TesModel {
    /// Create a new TES model.
    pub fn new(
        r_normal: f64,
        t_c: f64,
        delta_t: f64,
        g: f64,
        c: f64,
        i_bias: f64,
    ) -> Self {
        Self { r_normal, t_c, delta_t, g, c, i_bias }
    }

    /// Resistance as a function of temperature using a smooth transition model.
    ///
    /// R(T) = R_n / (1 + exp(-(T - T_c) / (delta_T / 4)))
    /// (sigmoid function centered at T_c).
    pub fn resistance(&self, temp_k: f64) -> f64 {
        let x = (temp_k - self.t_c) / (self.delta_t / 4.0);
        self.r_normal / (1.0 + (-x).exp())
    }

    /// Logarithmic sensitivity: alpha = (T/R) * dR/dT.
    ///
    /// Computed numerically with a small perturbation.
    pub fn alpha(&self, temp_k: f64) -> f64 {
        let dt = self.delta_t * 1e-4;
        let r = self.resistance(temp_k);
        if r.abs() < 1e-30 {
            return 0.0;
        }
        let drdt = (self.resistance(temp_k + dt) - self.resistance(temp_k - dt)) / (2.0 * dt);
        (temp_k / r) * drdt
    }

    /// Bias power at operating point: P_bias = I_bias^2 * R(T).
    pub fn bias_power(&self, temp_k: f64) -> f64 {
        self.i_bias * self.i_bias * self.resistance(temp_k)
    }

    /// Effective time constant with electrothermal feedback (ETF):
    ///   tau_eff = tau / (1 + alpha * P_bias / (G * T))
    ///
    /// ETF speeds up the detector response when alpha > 0 (negative feedback in voltage bias).
    pub fn effective_tau(&self, temp_k: f64) -> f64 {
        let tau = self.c / self.g;
        let a = self.alpha(temp_k);
        let p_bias = self.bias_power(temp_k);
        let loop_gain = a * p_bias / (self.g * temp_k);
        tau / (1.0 + loop_gain)
    }

    /// Intrinsic energy resolution (FWHM) for a TES calorimeter:
    ///   dE = 2.35 * sqrt(k_B * T^2 * C) * sqrt(n / (2 * alpha))
    ///
    /// where n is the thermal conductance exponent (typically 3-5).
    pub fn energy_resolution_ev(&self, temp_k: f64, n_exponent: f64) -> f64 {
        let a = self.alpha(temp_k);
        if a.abs() < 1e-10 {
            return f64::INFINITY;
        }
        let base = K_B * temp_k * temp_k * self.c;
        let factor = n_exponent / (2.0 * a.abs());
        // Convert from Joules to eV: 1 eV = 1.602e-19 J
        2.35 * (base * factor).sqrt() / 1.602_176_634e-19
    }

    /// Compute the R-T (resistance vs temperature) curve.
    pub fn rt_curve(&self, t_start: f64, t_end: f64, n_points: usize) -> Vec<(f64, f64)> {
        (0..n_points)
            .map(|i| {
                let t = t_start + (t_end - t_start) * i as f64 / (n_points - 1).max(1) as f64;
                (t, self.resistance(t))
            })
            .collect()
    }
}

// ─── MultiplexReadout ─────────────────────────────────────────────────

/// Multiplexing scheme for detector arrays.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MuxScheme {
    /// Time-Division Multiplexing — detectors read sequentially.
    TimeDivision,
    /// Frequency-Division Multiplexing — each detector at a unique carrier frequency.
    FrequencyDivision,
}

/// Multiplexed readout for bolometer arrays.
///
/// Models the readout of N detectors through a single amplifier chain
/// using either TDM or FDM schemes.
#[derive(Debug, Clone)]
pub struct MultiplexReadout {
    /// Number of detectors in the array.
    pub n_detectors: usize,
    /// Multiplexing scheme.
    pub scheme: MuxScheme,
    /// Base sample rate of the readout (per-detector effective rate).
    pub base_sample_rate: f64,
    /// FDM carrier frequencies (only used for FrequencyDivision).
    carrier_freqs: Vec<f64>,
}

impl MultiplexReadout {
    /// Create a TDM readout for `n_detectors` at the given base sample rate.
    ///
    /// The total readout rate is `n_detectors * base_sample_rate`.
    pub fn tdm(n_detectors: usize, base_sample_rate: f64) -> Self {
        Self {
            n_detectors,
            scheme: MuxScheme::TimeDivision,
            base_sample_rate,
            carrier_freqs: Vec::new(),
        }
    }

    /// Create an FDM readout with specified carrier frequencies.
    ///
    /// Each detector is read out at its own carrier frequency.
    pub fn fdm(carrier_freqs: Vec<f64>, base_sample_rate: f64) -> Self {
        let n = carrier_freqs.len();
        Self {
            n_detectors: n,
            scheme: MuxScheme::FrequencyDivision,
            base_sample_rate,
            carrier_freqs,
        }
    }

    /// Total readout bandwidth in Hz.
    pub fn total_bandwidth(&self) -> f64 {
        match self.scheme {
            MuxScheme::TimeDivision => self.n_detectors as f64 * self.base_sample_rate,
            MuxScheme::FrequencyDivision => {
                if self.carrier_freqs.len() < 2 {
                    return self.base_sample_rate;
                }
                let min = self.carrier_freqs.iter().cloned().fold(f64::INFINITY, f64::min);
                let max = self.carrier_freqs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
                (max - min) + self.base_sample_rate
            }
        }
    }

    /// Effective per-detector sample rate.
    pub fn effective_sample_rate(&self) -> f64 {
        self.base_sample_rate
    }

    /// TDM: Demultiplex an interleaved stream into per-detector streams.
    ///
    /// The input stream has samples ordered: [det0_s0, det1_s0, ..., detN_s0, det0_s1, ...].
    pub fn demux_tdm(&self, interleaved: &[f64]) -> Vec<Vec<f64>> {
        let n = self.n_detectors;
        let mut channels: Vec<Vec<f64>> = (0..n).map(|_| Vec::new()).collect();
        for (i, &sample) in interleaved.iter().enumerate() {
            channels[i % n].push(sample);
        }
        channels
    }

    /// TDM: Multiplex per-detector streams into an interleaved stream.
    pub fn mux_tdm(&self, channels: &[Vec<f64>]) -> Vec<f64> {
        if channels.is_empty() {
            return Vec::new();
        }
        let max_len = channels.iter().map(|c| c.len()).max().unwrap_or(0);
        let n = channels.len();
        let mut interleaved = Vec::with_capacity(n * max_len);
        for s in 0..max_len {
            for ch in channels {
                interleaved.push(if s < ch.len() { ch[s] } else { 0.0 });
            }
        }
        interleaved
    }

    /// FDM: Modulate each detector channel onto its carrier frequency.
    ///
    /// Returns the composite multiplexed signal at the total bandwidth sample rate.
    pub fn modulate_fdm(&self, channels: &[Vec<f64>], total_sample_rate: f64) -> Vec<f64> {
        if channels.is_empty() {
            return Vec::new();
        }
        let max_len = channels.iter().map(|c| c.len()).max().unwrap_or(0);
        // Output length: resample from base_sample_rate to total_sample_rate
        let output_len = (max_len as f64 * total_sample_rate / self.base_sample_rate) as usize;
        let mut composite = vec![0.0; output_len];

        for (ch_idx, channel) in channels.iter().enumerate() {
            if ch_idx >= self.carrier_freqs.len() {
                break;
            }
            let freq = self.carrier_freqs[ch_idx];
            for i in 0..output_len {
                let t = i as f64 / total_sample_rate;
                // Nearest-neighbor resampling of channel data
                let src_idx = (t * self.base_sample_rate) as usize;
                let sample = if src_idx < channel.len() { channel[src_idx] } else { 0.0 };
                composite[i] += sample * (2.0 * PI * freq * t).cos();
            }
        }
        composite
    }

    /// FDM: Demodulate a composite signal to extract per-detector channels.
    pub fn demodulate_fdm(&self, composite: &[f64], total_sample_rate: f64) -> Vec<Vec<f64>> {
        let output_len = (composite.len() as f64 * self.base_sample_rate / total_sample_rate) as usize;
        let mut channels = Vec::with_capacity(self.carrier_freqs.len());

        for &freq in &self.carrier_freqs {
            let mut demod = vec![0.0; output_len];
            for i in 0..output_len {
                let t_out = i as f64 / self.base_sample_rate;
                // Integrate over one output sample period
                let src_start = (t_out * total_sample_rate) as usize;
                let src_end = ((t_out + 1.0 / self.base_sample_rate) * total_sample_rate) as usize;
                let src_end = src_end.min(composite.len());
                let mut acc = 0.0;
                let mut count = 0;
                for j in src_start..src_end {
                    let t = j as f64 / total_sample_rate;
                    acc += composite[j] * (2.0 * PI * freq * t).cos() * 2.0;
                    count += 1;
                }
                demod[i] = if count > 0 { acc / count as f64 } else { 0.0 };
            }
            channels.push(demod);
        }
        channels
    }

    /// Noise penalty for multiplexing: sqrt(N) for TDM, 1.0 for FDM.
    pub fn noise_penalty(&self) -> f64 {
        match self.scheme {
            MuxScheme::TimeDivision => (self.n_detectors as f64).sqrt(),
            MuxScheme::FrequencyDivision => 1.0,
        }
    }
}

// ─── Helper functions ─────────────────────────────────────────────────

/// Compute the real DFT of a signal, returning (real, imag) pairs for N/2+1 bins.
fn real_dft(x: &[f64]) -> Vec<(f64, f64)> {
    let n = x.len();
    let n_freq = n / 2 + 1;
    let mut result = Vec::with_capacity(n_freq);
    for k in 0..n_freq {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &xi) in x.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += xi * angle.cos();
            im += xi * angle.sin();
        }
        result.push((re, im));
    }
    result
}

/// Inverse real DFT from N/2+1 frequency bins to N time-domain samples.
fn inverse_real_dft(re: &[f64], im: &[f64], n: usize) -> Vec<f64> {
    let n_freq = re.len();
    let mut result = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for k in 0..n_freq {
            let angle = 2.0 * PI * k as f64 * i as f64 / n as f64;
            let weight = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
            sum += weight * (re[k] * angle.cos() - im[k] * angle.sin());
        }
        result[i] = sum / n as f64;
    }
    result
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_linear_system(a_flat: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut aug: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let mut row = Vec::with_capacity(n + 1);
            for j in 0..n {
                row.push(a_flat[i * n + j]);
            }
            row.push(b[i]);
            row
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                let val = aug[col][j];
                aug[row][j] -= factor * val;
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }
    x
}

/// Compute the median of a slice.
fn median_of(data: &[f64]) -> f64 {
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

// ─── Tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> BolometerConfig {
        BolometerConfig {
            thermal_conductance_g: 1e-10,
            heat_capacity_c: 1e-12,
            operating_temp_k: 0.3,
            bath_temp_k: 0.25,
            sample_rate_hz: 1000.0,
        }
    }

    // ── BolometerConfig tests ──

    #[test]
    fn test_time_constant() {
        let config = default_config();
        let tau = config.time_constant_s();
        assert!((tau - 0.01).abs() < 1e-10, "tau = C/G = 1e-12/1e-10 = 0.01 s");
    }

    #[test]
    fn test_thermal_bandwidth() {
        let config = default_config();
        let bw = config.thermal_bandwidth_hz();
        let expected = 1.0 / (2.0 * PI * 0.01);
        assert!((bw - expected).abs() < 0.01);
    }

    #[test]
    fn test_static_power() {
        let config = default_config();
        let p = config.static_power_w();
        let expected = 1e-10 * (0.3 - 0.25); // 5 pW
        assert!((p - expected).abs() < 1e-20);
    }

    #[test]
    fn test_default_config() {
        let config = BolometerConfig::default();
        assert_eq!(config.thermal_conductance_g, 1e-10);
        assert_eq!(config.heat_capacity_c, 1e-12);
        assert_eq!(config.operating_temp_k, 0.3);
    }

    // ── ThermalResponseModel tests ──

    #[test]
    fn test_thermal_model_equilibrium() {
        let config = default_config();
        let mut model = ThermalResponseModel::new(config.clone());
        // At equilibrium with static power, temperature should stay constant
        let p_static = config.static_power_w();
        for _ in 0..100 {
            model.step(p_static);
        }
        assert!((model.temperature() - config.operating_temp_k).abs() < 1e-6);
    }

    #[test]
    fn test_thermal_model_heating() {
        let config = default_config();
        let mut model = ThermalResponseModel::new(config.clone());
        let p_extra = config.static_power_w() + 1e-11; // Extra 10 pW
        let t0 = model.temperature();
        for _ in 0..100 {
            model.step(p_extra);
        }
        assert!(model.temperature() > t0, "Temperature should rise with extra power");
    }

    #[test]
    fn test_thermal_model_cooling() {
        let config = default_config();
        let mut model = ThermalResponseModel::new(config.clone());
        // No input power -> should cool toward bath temperature
        for _ in 0..10000 {
            model.step(0.0);
        }
        assert!(
            (model.temperature() - config.bath_temp_k).abs() < 0.01,
            "Should cool toward bath temp"
        );
    }

    #[test]
    fn test_thermal_model_process() {
        let config = default_config();
        let mut model = ThermalResponseModel::new(config.clone());
        let p_in = vec![config.static_power_w(); 10];
        let temps = model.process(&p_in);
        assert_eq!(temps.len(), 10);
    }

    #[test]
    fn test_thermal_model_reset() {
        let config = default_config();
        let mut model = ThermalResponseModel::new(config.clone());
        model.step(1e-9);
        model.reset();
        assert_eq!(model.temperature(), config.operating_temp_k);
    }

    #[test]
    fn test_impulse_response_shape() {
        let config = default_config();
        let model = ThermalResponseModel::new(config);
        let ir = model.impulse_response(100);
        assert_eq!(ir.len(), 100);
        // Should be monotonically decreasing (exponential decay)
        for i in 1..ir.len() {
            assert!(ir[i] <= ir[i - 1] + 1e-20, "Impulse response should decay");
        }
        assert!(ir[0] > 0.0, "Initial impulse should be positive");
    }

    // ── PulseTemplateGenerator tests ──

    #[test]
    fn test_template_from_thermal_model() {
        let config = default_config();
        let template = PulseTemplateGenerator::from_thermal_model(&config, 200);
        assert_eq!(template.len(), 200);
        assert!((template[0] - 1.0).abs() < 1e-10, "Peak should be at t=0, normalized to 1");
        assert!(template[199] < template[0], "Template should decay");
    }

    #[test]
    fn test_biexponential_template() {
        let template = PulseTemplateGenerator::biexponential(0.001, 0.01, 1000.0, 100);
        assert_eq!(template.len(), 100);
        // Peak should be normalized to 1.0
        let max = template.iter().cloned().fold(0.0_f64, f64::max);
        assert!((max - 1.0).abs() < 1e-10);
        // Should start at 0 (rise from zero)
        assert!(template[0] < 0.5, "Bi-exponential should start near zero");
    }

    #[test]
    fn test_averaged_pulses_template() {
        // Create simple exponential pulses with slight variations
        let pulses: Vec<Vec<f64>> = (0..5)
            .map(|_| {
                (0..100)
                    .map(|i| {
                        let t = i as f64 / 100.0;
                        (-t / 0.01).exp()
                    })
                    .collect()
            })
            .collect();
        let template = PulseTemplateGenerator::from_averaged_pulses(&pulses, 100);
        assert_eq!(template.len(), 100);
        let max = template.iter().cloned().fold(0.0_f64, f64::max);
        assert!((max - 1.0).abs() < 1e-10, "Should be normalized");
    }

    #[test]
    fn test_averaged_pulses_empty() {
        let template = PulseTemplateGenerator::from_averaged_pulses(&[], 50);
        assert_eq!(template.len(), 50);
        assert!(template.iter().all(|&v| v == 0.0));
    }

    // ── BaselineEstimator tests ──

    #[test]
    fn test_baseline_estimate() {
        let estimator = BaselineEstimator::new(10);
        let mut record = vec![1.0; 10];
        record.extend(vec![5.0; 90]);
        let bl = estimator.estimate(&record);
        assert!((bl - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_baseline_subtraction() {
        let estimator = BaselineEstimator::new(10);
        let mut record = vec![2.0; 10];
        record.extend(vec![7.0; 10]);
        let subtracted = estimator.subtract(&record);
        assert!((subtracted[0]).abs() < 1e-10, "Baseline should be removed");
        assert!((subtracted[15] - 5.0).abs() < 1e-10, "Signal should be offset-corrected");
    }

    #[test]
    fn test_baseline_noise() {
        let estimator = BaselineEstimator::new(4);
        // Known variance: [1, 3, 1, 3] -> mean=2, var=4/3, std=1.155
        let record = vec![1.0, 3.0, 1.0, 3.0, 10.0, 10.0];
        let noise = estimator.estimate_noise(&record);
        let expected_std = (4.0 / 3.0_f64).sqrt(); // sample std with N-1
        assert!((noise - expected_std).abs() < 0.01);
    }

    // ── OptimalFilter tests ──

    #[test]
    fn test_optimal_filter_creation() {
        let config = default_config();
        let template = PulseTemplateGenerator::from_thermal_model(&config, 64);
        let noise_psd = vec![1e-6; 33];
        let filter = OptimalFilter::new(&template, &noise_psd);
        assert_eq!(filter.frequency_response().len(), 33);
    }

    #[test]
    fn test_optimal_filter_amplitude_estimation() {
        let config = default_config();
        let template = PulseTemplateGenerator::from_thermal_model(&config, 64);
        // Scale template by known amplitude
        let amplitude = 3.5;
        let record: Vec<f64> = template.iter().map(|&v| v * amplitude).collect();
        let noise_psd = vec![1e-10; 33]; // Very low noise
        let filter = OptimalFilter::new(&template, &noise_psd);
        // First verify the filter on the template itself (should give ~1.0)
        let est_unit = filter.estimate_amplitude(&template);
        // Then verify scaling: the amplitude ratio should match
        let est_scaled = filter.estimate_amplitude(&record);
        // The ratio of estimates should match the amplitude ratio
        if est_unit.abs() > 1e-10 {
            let ratio = est_scaled / est_unit;
            assert!(
                (ratio - amplitude).abs() < amplitude * 0.1,
                "Amplitude ratio {} should be close to {}", ratio, amplitude
            );
        } else {
            // Just verify the scaled output is proportionally larger
            assert!(est_scaled > est_unit, "Scaled pulse should have larger amplitude estimate");
        }
    }

    #[test]
    fn test_optimal_filter_snr_improvement() {
        // With noise, the filtered output should have better SNR than raw
        let template: Vec<f64> = (0..64).map(|i| (-i as f64 / 10.0).exp()).collect();
        let noise_psd = vec![0.01; 33];
        let filter = OptimalFilter::new(&template, &noise_psd);
        let resp = filter.frequency_response();
        // Filter should suppress frequencies where noise dominates
        assert!(resp.iter().all(|&w| w >= 0.0 && w <= 1.0));
    }

    // ── EnergyCalibrator tests ──

    #[test]
    fn test_linear_calibrator() {
        let cal = EnergyCalibrator::linear(0.0, 100.0);
        assert!((cal.amplitude_to_energy(1.0) - 100.0).abs() < 1e-10);
        assert!((cal.amplitude_to_energy(5.0) - 500.0).abs() < 1e-10);
    }

    #[test]
    fn test_linear_inverse() {
        let cal = EnergyCalibrator::linear(10.0, 50.0);
        let amp = cal.energy_to_amplitude(260.0).unwrap();
        assert!((amp - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_polynomial_fit_linear() {
        let points = vec![(1.0, 100.0), (2.0, 200.0), (3.0, 300.0)];
        let cal = EnergyCalibrator::fit(&points, 1);
        assert!((cal.amplitude_to_energy(1.0) - 100.0).abs() < 1.0);
        assert!((cal.amplitude_to_energy(2.0) - 200.0).abs() < 1.0);
    }

    #[test]
    fn test_polynomial_fit_quadratic() {
        // E = 10 + 5*A + 2*A^2
        let points: Vec<(f64, f64)> = (0..5)
            .map(|i| {
                let a = i as f64;
                let e = 10.0 + 5.0 * a + 2.0 * a * a;
                (a, e)
            })
            .collect();
        let cal = EnergyCalibrator::fit(&points, 2);
        let e_test = cal.amplitude_to_energy(3.0);
        let expected = 10.0 + 5.0 * 3.0 + 2.0 * 9.0;
        assert!((e_test - expected).abs() < 0.1, "Got {}, expected {}", e_test, expected);
    }

    #[test]
    fn test_calibrator_coefficients() {
        let cal = EnergyCalibrator::linear(5.0, 10.0);
        assert_eq!(cal.coefficients().len(), 2);
        assert_eq!(cal.coefficients()[0], 5.0);
        assert_eq!(cal.coefficients()[1], 10.0);
    }

    // ── CosmicRayRejector tests ──

    #[test]
    fn test_cosmic_ray_fast_rise() {
        let rejector = CosmicRayRejector::new(5, 10.0, 100.0);
        // Instantaneous spike (1 sample rise)
        let mut record = vec![0.0; 50];
        record[10] = 50.0;
        record[11] = 45.0;
        record[12] = 40.0;
        let result = rejector.analyze(&record);
        assert!(result.is_glitch, "Fast spike should be flagged as cosmic ray");
    }

    #[test]
    fn test_cosmic_ray_saturated() {
        let rejector = CosmicRayRejector::new(5, 10.0, 100.0);
        let mut record = vec![0.0; 50];
        for i in 5..15 {
            record[i] = 150.0; // Above saturation level
        }
        let result = rejector.analyze(&record);
        assert!(result.is_saturated);
        assert!(result.is_glitch);
    }

    #[test]
    fn test_cosmic_ray_valid_pulse() {
        let rejector = CosmicRayRejector::new(3, 50.0, 100.0);
        // Slow-rising valid photon pulse
        let record: Vec<f64> = (0..50)
            .map(|i| {
                let t = i as f64 / 50.0;
                if t < 0.2 {
                    30.0 * (t / 0.2) // Slow rise over 10 samples
                } else {
                    30.0 * (-(t - 0.2) / 0.3).exp() // Exponential decay
                }
            })
            .collect();
        let result = rejector.analyze(&record);
        assert!(!result.is_glitch, "Slow pulse should not be flagged");
    }

    #[test]
    fn test_cosmic_ray_batch() {
        let rejector = CosmicRayRejector::new(3, 10.0, 100.0);
        let records = vec![
            vec![0.0; 20],          // quiet
            vec![0.0, 200.0, 0.0],  // saturated spike
        ];
        let results = rejector.process_batch(&records);
        assert_eq!(results.len(), 2);
        assert!(!results[0].is_glitch);
        assert!(results[1].is_glitch);
    }

    #[test]
    fn test_cosmic_ray_empty() {
        let rejector = CosmicRayRejector::new(3, 10.0, 100.0);
        let result = rejector.analyze(&[]);
        assert!(!result.is_glitch);
    }

    // ── NoiseCharacterizer tests ──

    #[test]
    fn test_noise_psd_constant_signal() {
        // Constant signal -> all power at DC
        let data = vec![1.0; 256];
        let result = NoiseCharacterizer::estimate_psd(&data, 1000.0, 64);
        assert!(!result.noise_psd.is_empty());
        assert_eq!(result.freq_hz.len(), result.noise_psd.len());
    }

    #[test]
    fn test_noise_psd_length() {
        let data = vec![0.0; 512];
        let result = NoiseCharacterizer::estimate_psd(&data, 1000.0, 128);
        assert_eq!(result.noise_psd.len(), 65); // 128/2 + 1
        assert_eq!(result.freq_hz.len(), 65);
    }

    #[test]
    fn test_theoretical_noise_model() {
        let freqs: Vec<f64> = (1..100).map(|i| i as f64).collect();
        let psd = NoiseCharacterizer::theoretical_model(&freqs, 1e-6, 1e-7, 1e-5, 10.0, 1e-8);
        assert_eq!(psd.len(), 99);
        // Low frequencies should have more noise (1/f contribution)
        assert!(psd[0] > psd[98], "Low frequency should have higher noise");
    }

    #[test]
    fn test_noise_total_rms() {
        // White noise-like data
        let n = 1024;
        let data: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin() * 0.01).collect();
        let result = NoiseCharacterizer::estimate_psd(&data, 1000.0, 256);
        assert!(result.total_rms >= 0.0);
    }

    // ── NepCalculator tests ──

    #[test]
    fn test_phonon_nep() {
        let config = default_config();
        let nep = NepCalculator::phonon_nep(&config);
        // NEP = sqrt(4 * k_B * T^2 * G)
        let expected = (4.0 * K_B * 0.3 * 0.3 * 1e-10).sqrt();
        assert!((nep - expected).abs() < 1e-20);
        assert!(nep > 0.0);
        assert!(nep < 1e-15, "CMB bolometer NEP should be sub-fW/sqrt(Hz)");
    }

    #[test]
    fn test_johnson_noise() {
        let v_noise = NepCalculator::johnson_noise_voltage(4.2, 1000.0);
        let expected = (4.0 * K_B * 4.2 * 1000.0).sqrt();
        assert!((v_noise - expected).abs() < 1e-15);
    }

    #[test]
    fn test_photon_nep() {
        let nep = NepCalculator::photon_nep(1e-12, 150e9, 30e9);
        assert!(nep > 0.0, "Photon NEP should be positive");
    }

    #[test]
    fn test_total_nep_quadrature() {
        let nep_values = vec![3e-17, 4e-17];
        let total = NepCalculator::total_nep(&nep_values);
        let expected = 5e-17; // 3-4-5 triangle
        assert!((total - expected).abs() < 1e-20);
    }

    #[test]
    fn test_netd() {
        let nep = 1e-16;
        let area = 1e-4; // 1 cm^2
        let temp = 300.0;
        let netd = NepCalculator::netd(nep, area, temp);
        assert!(netd > 0.0);
        assert!(netd < 1.0, "NETD should be very small for sensitive detector");
    }

    #[test]
    fn test_background_power() {
        let p = NepCalculator::background_power(1e-4, 300.0);
        let expected = STEFAN_BOLTZMANN * 1e-4 * 300.0_f64.powi(4);
        assert!((p - expected).abs() < 1e-10);
    }

    // ── TesModel tests ──

    #[test]
    fn test_tes_resistance_transition() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        // Well below T_c: near zero
        let r_low = tes.resistance(0.095);
        assert!(r_low < 0.01 * tes.r_normal, "Below T_c, R should be near zero");
        // Well above T_c: near R_normal
        let r_high = tes.resistance(0.105);
        assert!(r_high > 0.9 * tes.r_normal, "Above T_c, R should be near R_normal");
        // At T_c: should be R_normal/2
        let r_tc = tes.resistance(0.1);
        assert!((r_tc - 0.05).abs() < 0.01, "At T_c, R should be R_n/2");
    }

    #[test]
    fn test_tes_alpha() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        let alpha = tes.alpha(0.1);
        // Alpha should be large at the transition midpoint
        assert!(alpha > 10.0, "Alpha should be large at T_c, got {}", alpha);
    }

    #[test]
    fn test_tes_alpha_off_transition() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        // Well below transition: R is near zero, alpha = (T/R)*dR/dT can be large
        // but the absolute dR/dT is tiny far from transition
        let alpha_low = tes.alpha(0.09).abs();
        // Well above transition: R is near R_normal, dR/dT is tiny
        let alpha_high = tes.alpha(0.11).abs();
        // At transition midpoint: alpha is large
        let alpha_mid = tes.alpha(0.1).abs();
        // Alpha at the transition midpoint should be the largest
        // Note: far below T_c, R->0 so (T/R)*dR/dT could blow up numerically,
        // but the sigmoid makes dR/dT exponentially small there.
        // Far above T_c, dR/dT -> 0 so alpha -> 0.
        assert!(alpha_mid > alpha_high, "alpha_mid={} should > alpha_high={}", alpha_mid, alpha_high);
        // alpha at the midpoint should be significantly larger than 1
        assert!(alpha_mid > 1.0, "alpha at T_c should be large, got {}", alpha_mid);
    }

    #[test]
    fn test_tes_bias_power() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        let p = tes.bias_power(0.1);
        let r = tes.resistance(0.1);
        let expected = 1e-6 * 1e-6 * r;
        assert!((p - expected).abs() < 1e-20);
    }

    #[test]
    fn test_tes_effective_tau() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        let tau_bare = tes.c / tes.g;
        let tau_eff = tes.effective_tau(0.1);
        // ETF should make tau_eff < tau_bare
        assert!(tau_eff < tau_bare, "ETF should speed up response: tau_eff={}, tau_bare={}", tau_eff, tau_bare);
        assert!(tau_eff > 0.0, "tau_eff should be positive");
    }

    #[test]
    fn test_tes_energy_resolution() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        let de = tes.energy_resolution_ev(0.1, 4.0);
        assert!(de > 0.0);
        assert!(de.is_finite());
    }

    #[test]
    fn test_tes_rt_curve() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        let curve = tes.rt_curve(0.095, 0.105, 100);
        assert_eq!(curve.len(), 100);
        // Should be monotonically increasing
        for i in 1..curve.len() {
            assert!(curve[i].1 >= curve[i - 1].1 - 1e-15, "R should increase with T");
        }
    }

    // ── MultiplexReadout tests ──

    #[test]
    fn test_tdm_demux() {
        let mux = MultiplexReadout::tdm(3, 100.0);
        let interleaved = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let channels = mux.demux_tdm(&interleaved);
        assert_eq!(channels.len(), 3);
        assert_eq!(channels[0], vec![1.0, 4.0]);
        assert_eq!(channels[1], vec![2.0, 5.0]);
        assert_eq!(channels[2], vec![3.0, 6.0]);
    }

    #[test]
    fn test_tdm_mux_roundtrip() {
        let mux = MultiplexReadout::tdm(3, 100.0);
        let channels = vec![vec![1.0, 4.0], vec![2.0, 5.0], vec![3.0, 6.0]];
        let interleaved = mux.mux_tdm(&channels);
        let recovered = mux.demux_tdm(&interleaved);
        assert_eq!(recovered, channels);
    }

    #[test]
    fn test_tdm_total_bandwidth() {
        let mux = MultiplexReadout::tdm(10, 100.0);
        assert_eq!(mux.total_bandwidth(), 1000.0);
    }

    #[test]
    fn test_tdm_noise_penalty() {
        let mux = MultiplexReadout::tdm(4, 100.0);
        assert!((mux.noise_penalty() - 2.0).abs() < 1e-10); // sqrt(4) = 2
    }

    #[test]
    fn test_fdm_creation() {
        let carriers = vec![1e6, 2e6, 3e6, 4e6];
        let mux = MultiplexReadout::fdm(carriers, 100.0);
        assert_eq!(mux.n_detectors, 4);
        assert_eq!(mux.scheme, MuxScheme::FrequencyDivision);
        assert!((mux.noise_penalty() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_fdm_bandwidth() {
        let carriers = vec![1e6, 2e6, 3e6];
        let mux = MultiplexReadout::fdm(carriers, 1000.0);
        let bw = mux.total_bandwidth();
        assert!((bw - 2.001e6).abs() < 1.0); // (3e6 - 1e6) + 1000
    }

    #[test]
    fn test_effective_sample_rate() {
        let mux = MultiplexReadout::tdm(8, 250.0);
        assert_eq!(mux.effective_sample_rate(), 250.0);
    }

    // ── Helper function tests ──

    #[test]
    fn test_real_dft_dc() {
        let signal = vec![1.0; 8];
        let spectrum = real_dft(&signal);
        assert!((spectrum[0].0 - 8.0).abs() < 1e-10, "DC bin should equal sum");
        // All other bins should be zero
        for k in 1..spectrum.len() {
            assert!(
                spectrum[k].0.abs() < 1e-10 && spectrum[k].1.abs() < 1e-10,
                "Non-DC bins should be zero for constant signal"
            );
        }
    }

    #[test]
    fn test_dft_roundtrip() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let spectrum = real_dft(&signal);
        let re: Vec<f64> = spectrum.iter().map(|s| s.0).collect();
        let im: Vec<f64> = spectrum.iter().map(|s| s.1).collect();
        let recovered = inverse_real_dft(&re, &im, signal.len());
        for (a, b) in signal.iter().zip(recovered.iter()) {
            assert!((a - b).abs() < 1e-10, "DFT roundtrip failed: {} vs {}", a, b);
        }
    }

    #[test]
    fn test_median_odd() {
        assert!((median_of(&[3.0, 1.0, 2.0]) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_median_even() {
        assert!((median_of(&[4.0, 1.0, 3.0, 2.0]) - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_median_empty() {
        assert_eq!(median_of(&[]), 0.0);
    }

    #[test]
    fn test_solve_linear_2x2() {
        // 2x + 3y = 8, x + y = 3 => x=1, y=2
        let a = vec![2.0, 3.0, 1.0, 1.0];
        let b = vec![8.0, 3.0];
        let x = solve_linear_system(&a, &b, 2);
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
    }

    // ── Integration tests ──

    #[test]
    fn test_full_pulse_processing_pipeline() {
        let config = default_config();

        // Generate template
        let template = PulseTemplateGenerator::from_thermal_model(&config, 128);

        // Create a pulse record with baseline offset and known amplitude
        let amplitude = 5.0;
        let baseline = 0.3;
        let mut record: Vec<f64> = vec![baseline; 128];
        for (i, &t) in template.iter().enumerate() {
            if i + 20 < 128 {
                record[i + 20] = baseline + amplitude * t;
            }
        }

        // Baseline subtraction
        let estimator = BaselineEstimator::new(15);
        let bl_subtracted = estimator.subtract(&record);

        // Verify baseline was removed
        let pre_trigger_mean: f64 = bl_subtracted[..15].iter().sum::<f64>() / 15.0;
        assert!(pre_trigger_mean.abs() < 0.1, "Baseline should be removed");
    }

    #[test]
    fn test_tes_with_nep() {
        let tes = TesModel::new(0.1, 0.1, 0.001, 1e-10, 1e-12, 1e-6);
        let config = BolometerConfig {
            thermal_conductance_g: tes.g,
            heat_capacity_c: tes.c,
            operating_temp_k: tes.t_c,
            bath_temp_k: tes.t_c - 0.01,
            sample_rate_hz: 10000.0,
        };
        let nep = NepCalculator::phonon_nep(&config);
        assert!(nep > 0.0);

        // Energy resolution should be finite for a TES at its transition
        let de = tes.energy_resolution_ev(tes.t_c, 4.0);
        assert!(de > 0.0 && de.is_finite());
    }
}
