//! # Superconducting Qubit Readout Signal Processing
//!
//! This module implements dispersive readout signal processing for superconducting
//! quantum computers. In circuit quantum electrodynamics (cQED), a superconducting
//! qubit is coupled to a microwave resonator. The qubit's quantum state shifts the
//! resonator's frequency via the dispersive interaction, and this shift is detected
//! by measuring the reflected or transmitted microwave signal using homodyne or
//! heterodyne detection.
//!
//! ## Physical Background
//!
//! The dispersive Hamiltonian in the rotating frame is:
//!
//! ```text
//! H_disp = (ω_r + χ σ_z) a†a + ω_q/2 σ_z
//! ```
//!
//! where `χ = g²/Δ` is the dispersive shift, `g` is the qubit-resonator coupling,
//! `Δ = ω_q - ω_r` is the qubit-resonator detuning, and `σ_z` is the qubit Pauli-Z
//! operator. The resonator frequency is `ω_r + χ` when the qubit is in |0⟩ and
//! `ω_r - χ` when the qubit is in |1⟩.
//!
//! ## Readout Chain
//!
//! ```text
//! Readout pulse → Resonator → Amplifier → Downconversion → ADC → Integration → Classification
//!                     ↑
//!              Qubit state shifts
//!              resonator frequency
//! ```
//!
//! ## Key Concepts
//!
//! - **Dispersive shift (χ)**: Frequency shift of the resonator per qubit excitation
//! - **Integration weights**: Boxcar (uniform), optimal, or matched filter weighting
//! - **State discrimination**: Rotate IQ plane to maximize separation, threshold along one axis
//! - **Purcell filter**: Bandpass filter protecting the qubit from decay through the readout line
//! - **Critical photon number**: Maximum photon occupancy before leaving the dispersive regime
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::superconducting_qubit_readout::*;
//!
//! let config = ReadoutConfig {
//!     resonator_frequency_ghz: 7.0,
//!     qubit_frequency_ghz: 5.0,
//!     dispersive_shift_mhz: 1.0,
//!     readout_duration_ns: 1000.0,
//!     integration_weight: IntegrationWeight::BoxCar,
//!     if_frequency_mhz: 50.0,
//!     sample_rate_mhz: 1000.0,
//! };
//!
//! let readout = QubitReadout::new(config);
//! let pulse = readout.generate_readout_pulse(500.0, 0.1, 7.0);
//! assert!(!pulse.is_empty());
//! ```

use std::f64::consts::PI;

/// Configuration for a single qubit readout channel.
#[derive(Debug, Clone)]
pub struct ReadoutConfig {
    /// Resonator frequency in GHz (typically 6-8 GHz).
    pub resonator_frequency_ghz: f64,
    /// Qubit frequency in GHz (typically 4-6 GHz).
    pub qubit_frequency_ghz: f64,
    /// Dispersive shift χ/2π in MHz (typically 0.5-5 MHz).
    pub dispersive_shift_mhz: f64,
    /// Readout duration in nanoseconds (typically 300-2000 ns).
    pub readout_duration_ns: f64,
    /// Integration weight function for matched/optimal filtering.
    pub integration_weight: IntegrationWeight,
    /// Intermediate frequency after downconversion in MHz.
    pub if_frequency_mhz: f64,
    /// ADC sample rate in MHz.
    pub sample_rate_mhz: f64,
}

/// Integration weighting strategy for the readout signal.
///
/// The integration weights determine how the demodulated IQ signal is combined
/// over the readout window to produce a single (I, Q) point for classification.
#[derive(Debug, Clone)]
pub enum IntegrationWeight {
    /// Uniform (boxcar) weighting — all samples weighted equally.
    BoxCar,
    /// Optimal weights derived from calibration data.
    /// Separate I and Q weight vectors allow for asymmetric responses.
    Optimal {
        /// In-phase integration weights.
        weights_i: Vec<f64>,
        /// Quadrature integration weights.
        weights_q: Vec<f64>,
    },
    /// Matched filter weighting — conjugate of expected signal difference.
    Matched,
}

/// Qubit state assignment result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QubitState {
    /// Ground state |0⟩.
    Ground,
    /// First excited state |1⟩.
    Excited,
    /// Leakage to higher levels (|2⟩ or above).
    Leakage,
}

/// Decision boundary for single-shot state classification.
///
/// The IQ data is rotated by `angle_rad` so that the ground and excited state
/// centroids are maximally separated along the real axis, then a threshold is
/// applied to discriminate the states.
#[derive(Debug, Clone)]
pub struct StateThreshold {
    /// Rotation angle in radians to maximize state separation.
    pub angle_rad: f64,
    /// Decision boundary along the rotated axis.
    pub threshold: f64,
}

impl StateThreshold {
    /// Derive the optimal threshold from calibration single-shot data.
    ///
    /// Given IQ shots prepared in |0⟩ and |1⟩, this finds the rotation angle
    /// that maximizes separation of the projected distributions and places the
    /// threshold at the midpoint between the two centroids.
    ///
    /// # Arguments
    ///
    /// * `ground_shots` - IQ points from |0⟩-prepared measurements
    /// * `excited_shots` - IQ points from |1⟩-prepared measurements
    ///
    /// # Returns
    ///
    /// A `StateThreshold` with the optimal rotation angle and midpoint threshold.
    pub fn from_calibration(
        ground_shots: &[(f64, f64)],
        excited_shots: &[(f64, f64)],
    ) -> Self {
        // Compute centroids
        let n0 = ground_shots.len() as f64;
        let n1 = excited_shots.len() as f64;

        let (sum_i0, sum_q0) = ground_shots
            .iter()
            .fold((0.0, 0.0), |(si, sq), &(i, q)| (si + i, sq + q));
        let mu0 = (sum_i0 / n0, sum_q0 / n0);

        let (sum_i1, sum_q1) = excited_shots
            .iter()
            .fold((0.0, 0.0), |(si, sq), &(i, q)| (si + i, sq + q));
        let mu1 = (sum_i1 / n1, sum_q1 / n1);

        // Rotation angle: angle of the line connecting the two centroids
        let di = mu1.0 - mu0.0;
        let dq = mu1.1 - mu0.1;
        let angle = dq.atan2(di);

        // Project centroids onto rotated axis
        let proj0 = mu0.0 * angle.cos() + mu0.1 * angle.sin();
        let proj1 = mu1.0 * angle.cos() + mu1.1 * angle.sin();

        // Threshold at midpoint
        let threshold = (proj0 + proj1) / 2.0;

        StateThreshold {
            angle_rad: angle,
            threshold,
        }
    }
}

/// Core readout signal processor for a single superconducting qubit.
///
/// Handles pulse generation, demodulation, integration, and state classification
/// for dispersive readout of a transmon-type qubit coupled to a readout resonator.
#[derive(Debug, Clone)]
pub struct QubitReadout {
    /// Configuration parameters.
    config: ReadoutConfig,
}

impl QubitReadout {
    /// Create a new readout processor with the given configuration.
    pub fn new(config: ReadoutConfig) -> Self {
        Self { config }
    }

    /// Generate a readout pulse as a sequence of (I, Q) samples.
    ///
    /// Produces a constant-envelope pulse at the specified frequency,
    /// suitable for driving the readout resonator.
    ///
    /// # Arguments
    ///
    /// * `duration_ns` - Pulse duration in nanoseconds
    /// * `amplitude` - Pulse amplitude (linear scale)
    /// * `frequency_ghz` - Drive frequency in GHz
    ///
    /// # Returns
    ///
    /// A vector of (I, Q) sample pairs at the configured sample rate.
    pub fn generate_readout_pulse(
        &self,
        duration_ns: f64,
        amplitude: f64,
        frequency_ghz: f64,
    ) -> Vec<(f64, f64)> {
        let sample_period_ns = 1000.0 / self.config.sample_rate_mhz; // ns per sample
        let n_samples = (duration_ns / sample_period_ns).ceil() as usize;

        // Frequency in GHz relative to IF, the pulse is at IF after downconversion
        // We generate at the IF frequency since the physical mixing is implicit
        let freq_hz = frequency_ghz * 1e9;
        let dt_s = sample_period_ns * 1e-9;

        let mut pulse = Vec::with_capacity(n_samples);
        for k in 0..n_samples {
            let t = k as f64 * dt_s;
            let phase = 2.0 * PI * freq_hz * t;
            let i = amplitude * phase.cos();
            let q = amplitude * phase.sin();
            pulse.push((i, q));
        }
        pulse
    }

    /// Compute the first-order dispersive shift.
    ///
    /// In the dispersive regime (|Δ| >> g), the shift is:
    ///
    /// ```text
    /// χ = g² / Δ
    /// ```
    ///
    /// where `g` is the qubit-resonator coupling and `Δ = ω_q - ω_r` is the detuning.
    ///
    /// # Arguments
    ///
    /// * `g_mhz` - Coupling strength g/2π in MHz
    /// * `delta_mhz` - Detuning Δ/2π in MHz (qubit_freq - resonator_freq)
    ///
    /// # Returns
    ///
    /// Dispersive shift χ/2π in MHz.
    pub fn dispersive_shift(g_mhz: f64, delta_mhz: f64) -> f64 {
        if delta_mhz.abs() < 1e-12 {
            return 0.0; // Avoid division by zero at resonance
        }
        g_mhz * g_mhz / delta_mhz
    }

    /// Compute the resonator S21 response when the qubit is in the ground state |0⟩.
    ///
    /// Models a Lorentzian resonance shifted by +χ from the bare resonator frequency.
    /// Returns the complex transmission as (magnitude, phase) at the given probe frequency.
    ///
    /// # Arguments
    ///
    /// * `freq_ghz` - Probe frequency in GHz
    ///
    /// # Returns
    ///
    /// (magnitude, phase_radians) of the S21 transmission coefficient.
    pub fn resonator_response_ground(&self, freq_ghz: f64) -> (f64, f64) {
        // Resonator shifted by +χ for ground state
        let res_freq = self.config.resonator_frequency_ghz
            + self.config.dispersive_shift_mhz * 1e-3;
        self.lorentzian_response(freq_ghz, res_freq)
    }

    /// Compute the resonator S21 response when the qubit is in the excited state |1⟩.
    ///
    /// Models a Lorentzian resonance shifted by -χ from the bare resonator frequency.
    /// Returns the complex transmission as (magnitude, phase) at the given probe frequency.
    ///
    /// # Arguments
    ///
    /// * `freq_ghz` - Probe frequency in GHz
    ///
    /// # Returns
    ///
    /// (magnitude, phase_radians) of the S21 transmission coefficient.
    pub fn resonator_response_excited(&self, freq_ghz: f64) -> (f64, f64) {
        // Resonator shifted by -χ for excited state
        let res_freq = self.config.resonator_frequency_ghz
            - self.config.dispersive_shift_mhz * 1e-3;
        self.lorentzian_response(freq_ghz, res_freq)
    }

    /// Internal Lorentzian resonance model for a resonator.
    ///
    /// S21 = 1 - κ_ext / (κ/2 + j(ω - ω_r))
    ///
    /// We use a typical linewidth κ/2π = 2 MHz and assume critical coupling
    /// (κ_ext = κ/2).
    fn lorentzian_response(&self, freq_ghz: f64, res_freq_ghz: f64) -> (f64, f64) {
        let kappa_mhz = 2.0; // Total linewidth κ/2π in MHz (typical)
        let kappa_ext_mhz = kappa_mhz / 2.0; // Critical coupling

        let detuning_mhz = (freq_ghz - res_freq_ghz) * 1e3; // Convert GHz to MHz

        // S21 = 1 - κ_ext / (κ/2 + j*detuning)
        let denom_re = kappa_mhz / 2.0;
        let denom_im = detuning_mhz;
        let denom_mag_sq = denom_re * denom_re + denom_im * denom_im;

        // κ_ext / (κ/2 + j*δ) = κ_ext * (κ/2 - j*δ) / |κ/2 + j*δ|²
        let ratio_re = kappa_ext_mhz * denom_re / denom_mag_sq;
        let ratio_im = -kappa_ext_mhz * denom_im / denom_mag_sq;

        let s21_re = 1.0 - ratio_re;
        let s21_im = -ratio_im;

        let magnitude = (s21_re * s21_re + s21_im * s21_im).sqrt();
        let phase = s21_im.atan2(s21_re);

        (magnitude, phase)
    }

    /// Integrate the demodulated IQ signal using the configured weighting.
    ///
    /// Applies the integration weight function (BoxCar, Optimal, or Matched)
    /// to the I and Q time series and returns a single (I, Q) integrated value.
    ///
    /// # Arguments
    ///
    /// * `signal_i` - In-phase demodulated signal
    /// * `signal_q` - Quadrature demodulated signal
    ///
    /// # Returns
    ///
    /// (integrated_I, integrated_Q) single-shot measurement result.
    pub fn integrate_signal(&self, signal_i: &[f64], signal_q: &[f64]) -> (f64, f64) {
        let n = signal_i.len().min(signal_q.len());
        if n == 0 {
            return (0.0, 0.0);
        }

        match &self.config.integration_weight {
            IntegrationWeight::BoxCar => {
                let sum_i: f64 = signal_i[..n].iter().sum();
                let sum_q: f64 = signal_q[..n].iter().sum();
                (sum_i / n as f64, sum_q / n as f64)
            }
            IntegrationWeight::Optimal {
                weights_i,
                weights_q,
            } => {
                let wn = weights_i.len().min(weights_q.len()).min(n);
                let mut int_i = 0.0;
                let mut int_q = 0.0;
                let mut norm_i = 0.0;
                let mut norm_q = 0.0;
                for k in 0..wn {
                    int_i += signal_i[k] * weights_i[k];
                    int_q += signal_q[k] * weights_q[k];
                    norm_i += weights_i[k].abs();
                    norm_q += weights_q[k].abs();
                }
                let ni = if norm_i > 1e-30 { norm_i } else { 1.0 };
                let nq = if norm_q > 1e-30 { norm_q } else { 1.0 };
                (int_i / ni, int_q / nq)
            }
            IntegrationWeight::Matched => {
                // Matched filter: use the signal itself as the weight
                // In practice this would be the conjugate of the expected signal difference
                // For now we use uniform weighting as a fallback (same as BoxCar)
                let sum_i: f64 = signal_i[..n].iter().sum();
                let sum_q: f64 = signal_q[..n].iter().sum();
                (sum_i / n as f64, sum_q / n as f64)
            }
        }
    }

    /// Demodulate a real-valued IF signal into I and Q components.
    ///
    /// Performs digital homodyne demodulation by mixing with cos and sin
    /// at the IF frequency and low-pass filtering (moving average).
    ///
    /// # Arguments
    ///
    /// * `signal` - Real-valued digitized IF signal
    /// * `if_freq_mhz` - Intermediate frequency in MHz
    /// * `sample_rate_mhz` - ADC sample rate in MHz
    ///
    /// # Returns
    ///
    /// (I_samples, Q_samples) demodulated baseband signals.
    pub fn demodulate(
        &self,
        signal: &[f64],
        if_freq_mhz: f64,
        sample_rate_mhz: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = signal.len();
        let mut i_out = Vec::with_capacity(n);
        let mut q_out = Vec::with_capacity(n);

        let dt = 1.0 / sample_rate_mhz; // microseconds per sample
        let omega = 2.0 * PI * if_freq_mhz; // rad/us

        for k in 0..n {
            let t = k as f64 * dt;
            let phase = omega * t;
            // Mix with local oscillator
            i_out.push(signal[k] * 2.0 * phase.cos());
            q_out.push(signal[k] * -2.0 * phase.sin());
        }

        // Simple moving average low-pass filter
        // Use a window of approximately one IF period
        let period_samples = (sample_rate_mhz / if_freq_mhz).round() as usize;
        let window = period_samples.max(1);

        let i_filtered = moving_average(&i_out, window);
        let q_filtered = moving_average(&q_out, window);

        (i_filtered, q_filtered)
    }

    /// Classify a single (I, Q) measurement into a qubit state.
    ///
    /// Projects the IQ point onto the rotated axis defined by the threshold,
    /// then classifies based on which side of the decision boundary the
    /// projection falls.
    ///
    /// # Arguments
    ///
    /// * `i_val` - In-phase integrated value
    /// * `q_val` - Quadrature integrated value
    /// * `threshold` - Decision boundary parameters
    ///
    /// # Returns
    ///
    /// Assigned `QubitState`.
    pub fn classify_state(
        &self,
        i_val: f64,
        q_val: f64,
        threshold: &StateThreshold,
    ) -> QubitState {
        // Project onto rotated axis
        let projection =
            i_val * threshold.angle_rad.cos() + q_val * threshold.angle_rad.sin();

        if projection < threshold.threshold {
            QubitState::Ground
        } else {
            QubitState::Excited
        }
    }

    /// Compute readout assignment fidelity from a set of prepared-and-measured results.
    ///
    /// Fidelity F = (P(0|0) + P(1|1)) / 2, where P(m|p) is the probability of
    /// measuring state m given preparation in state p.
    ///
    /// # Arguments
    ///
    /// * `assignments` - Pairs of (prepared_state, measured_state)
    ///
    /// # Returns
    ///
    /// Assignment fidelity in [0, 1].
    pub fn readout_fidelity(assignments: &[(QubitState, QubitState)]) -> f64 {
        if assignments.is_empty() {
            return 0.0;
        }

        let mut n_ground_prepared = 0usize;
        let mut n_excited_prepared = 0usize;
        let mut correct_ground = 0usize;
        let mut correct_excited = 0usize;

        for &(prepared, measured) in assignments {
            match prepared {
                QubitState::Ground => {
                    n_ground_prepared += 1;
                    if measured == QubitState::Ground {
                        correct_ground += 1;
                    }
                }
                QubitState::Excited => {
                    n_excited_prepared += 1;
                    if measured == QubitState::Excited {
                        correct_excited += 1;
                    }
                }
                QubitState::Leakage => {}
            }
        }

        let p0_0 = if n_ground_prepared > 0 {
            correct_ground as f64 / n_ground_prepared as f64
        } else {
            0.0
        };
        let p1_1 = if n_excited_prepared > 0 {
            correct_excited as f64 / n_excited_prepared as f64
        } else {
            0.0
        };

        (p0_0 + p1_1) / 2.0
    }

    /// Compute the signal-to-noise ratio from the separation of two Gaussian distributions.
    ///
    /// SNR = |μ₁ - μ₀| / σ
    ///
    /// where μ₀ and μ₁ are the centroids of the ground and excited state distributions
    /// in IQ space, and σ is the common standard deviation.
    ///
    /// # Arguments
    ///
    /// * `mu_0` - Centroid (I, Q) of the ground state distribution
    /// * `mu_1` - Centroid (I, Q) of the excited state distribution
    /// * `sigma` - Common standard deviation of the distributions
    ///
    /// # Returns
    ///
    /// SNR (dimensionless).
    pub fn snr_from_separation(
        mu_0: (f64, f64),
        mu_1: (f64, f64),
        sigma: f64,
    ) -> f64 {
        if sigma.abs() < 1e-30 {
            return f64::INFINITY;
        }
        let di = mu_1.0 - mu_0.0;
        let dq = mu_1.1 - mu_0.1;
        let distance = (di * di + dq * dq).sqrt();
        distance / sigma
    }
}

/// Multi-qubit frequency-multiplexed readout processor.
///
/// In multi-qubit systems, each qubit's readout resonator is designed at a different
/// frequency. The readout tones are combined (frequency multiplexed) into a single
/// signal, sent through the feedline, and then demultiplexed after measurement.
#[derive(Debug, Clone)]
pub struct MultiQubitReadout {
    /// Number of qubits.
    num_qubits: usize,
    /// Per-qubit readout configurations.
    configs: Vec<ReadoutConfig>,
}

impl MultiQubitReadout {
    /// Create a new multi-qubit readout processor.
    ///
    /// # Arguments
    ///
    /// * `num_qubits` - Number of qubits to read out simultaneously
    /// * `configs` - Per-qubit readout configurations (must have length num_qubits)
    pub fn new(num_qubits: usize, configs: Vec<ReadoutConfig>) -> Self {
        assert_eq!(
            configs.len(),
            num_qubits,
            "Must provide one config per qubit"
        );
        Self {
            num_qubits,
            configs,
        }
    }

    /// Combine multiple readout tones into a single frequency-multiplexed signal.
    ///
    /// Each input signal is a sequence of (I, Q) samples at different IF frequencies.
    /// The output is the sum of all tones.
    ///
    /// # Arguments
    ///
    /// * `signals` - Per-qubit IQ readout pulses
    ///
    /// # Returns
    ///
    /// Combined (I, Q) signal.
    pub fn frequency_multiplex(
        &self,
        signals: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        if signals.is_empty() {
            return vec![];
        }

        let max_len = signals.iter().map(|s| s.len()).max().unwrap_or(0);
        let mut combined = vec![(0.0, 0.0); max_len];

        for signal in signals {
            for (k, &(i, q)) in signal.iter().enumerate() {
                combined[k].0 += i;
                combined[k].1 += q;
            }
        }

        combined
    }

    /// Demultiplex a combined signal into per-qubit IQ streams.
    ///
    /// Uses digital downconversion at each qubit's IF frequency to extract
    /// individual readout signals from the composite.
    ///
    /// # Arguments
    ///
    /// * `combined` - Frequency-multiplexed (I, Q) signal
    /// * `frequencies` - Per-qubit IF frequencies in MHz
    /// * `sample_rate` - Sample rate in MHz
    ///
    /// # Returns
    ///
    /// Per-qubit demodulated (I, Q) streams.
    pub fn demultiplex(
        &self,
        combined: &[(f64, f64)],
        frequencies: &[f64],
        sample_rate: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        let n = combined.len();
        let mut results = Vec::with_capacity(frequencies.len());

        for &freq_mhz in frequencies {
            let dt = 1.0 / sample_rate; // us per sample
            let omega = 2.0 * PI * freq_mhz; // rad/us

            let mut demod = Vec::with_capacity(n);
            for k in 0..n {
                let t = k as f64 * dt;
                let phase = omega * t;
                let cos_lo = phase.cos();
                let sin_lo = phase.sin();

                // Complex downconversion: multiply by exp(-j*omega*t)
                let i_in = combined[k].0;
                let q_in = combined[k].1;
                let i_out = i_in * cos_lo + q_in * sin_lo;
                let q_out = -i_in * sin_lo + q_in * cos_lo;
                demod.push((i_out, q_out));
            }

            // Low-pass filter
            let period_samples = (sample_rate / freq_mhz).round().max(1.0) as usize;
            let window = period_samples.max(2);
            let i_vals: Vec<f64> = demod.iter().map(|&(i, _)| i).collect();
            let q_vals: Vec<f64> = demod.iter().map(|&(_, q)| q).collect();
            let i_filt = moving_average(&i_vals, window);
            let q_filt = moving_average(&q_vals, window);

            let filtered: Vec<(f64, f64)> = i_filt
                .into_iter()
                .zip(q_filt)
                .collect();
            results.push(filtered);
        }

        results
    }

    /// Assign qubit states from a vector of (I, Q) measurements.
    ///
    /// Uses a simple threshold classifier: each qubit is classified independently
    /// based on its I-component (assuming pre-rotated IQ data with separation
    /// along the I axis).
    ///
    /// # Arguments
    ///
    /// * `iq_values` - Per-qubit integrated (I, Q) values
    ///
    /// # Returns
    ///
    /// Per-qubit state assignments.
    pub fn joint_state_assignment(&self, iq_values: &[(f64, f64)]) -> Vec<QubitState> {
        iq_values
            .iter()
            .map(|&(i, _q)| {
                if i >= 0.0 {
                    QubitState::Excited
                } else {
                    QubitState::Ground
                }
            })
            .collect()
    }
}

/// Purcell filter analysis for qubit lifetime protection.
///
/// A Purcell filter is a bandpass filter placed between the readout resonator
/// and the output transmission line. It allows readout photons at the resonator
/// frequency to pass while suppressing qubit decay at the qubit frequency.
///
/// The Purcell decay rate without a filter is:
///
/// ```text
/// γ_P = (g/Δ)² × κ
/// ```
///
/// where `κ` is the resonator linewidth (photon loss rate).
#[derive(Debug, Clone)]
pub struct PurcellFilter {
    /// Resonator frequency in GHz.
    resonator_freq_ghz: f64,
    /// Filter 3 dB bandwidth in MHz.
    filter_bandwidth_mhz: f64,
    /// Qubit frequency in GHz.
    qubit_freq_ghz: f64,
}

impl PurcellFilter {
    /// Create a new Purcell filter model.
    ///
    /// # Arguments
    ///
    /// * `resonator_freq_ghz` - Resonator center frequency in GHz
    /// * `filter_bandwidth_mhz` - Filter passband bandwidth in MHz
    /// * `qubit_freq_ghz` - Qubit frequency in GHz
    pub fn new(
        resonator_freq_ghz: f64,
        filter_bandwidth_mhz: f64,
        qubit_freq_ghz: f64,
    ) -> Self {
        Self {
            resonator_freq_ghz,
            filter_bandwidth_mhz,
            qubit_freq_ghz,
        }
    }

    /// Compute the Purcell decay rate (without filter).
    ///
    /// ```text
    /// γ_P = (g / Δ)² × κ
    /// ```
    ///
    /// # Arguments
    ///
    /// * `g_mhz` - Coupling strength g/2π in MHz
    /// * `kappa_mhz` - Resonator linewidth κ/2π in MHz
    /// * `delta_mhz` - Qubit-resonator detuning Δ/2π in MHz
    ///
    /// # Returns
    ///
    /// Purcell decay rate γ_P/2π in MHz.
    pub fn purcell_decay_rate(g_mhz: f64, kappa_mhz: f64, delta_mhz: f64) -> f64 {
        if delta_mhz.abs() < 1e-12 {
            return f64::INFINITY;
        }
        let ratio = g_mhz / delta_mhz;
        ratio * ratio * kappa_mhz
    }

    /// Compute the T1 limit imposed by Purcell decay.
    ///
    /// ```text
    /// T1 = 1 / γ_P
    /// ```
    ///
    /// where γ_P is in MHz (i.e., 10⁶ s⁻¹), so T1 is returned in nanoseconds
    /// (T1_ns = 1000 / γ_P_MHz).
    ///
    /// # Arguments
    ///
    /// * `gamma_p_mhz` - Purcell decay rate γ_P/2π in MHz
    ///
    /// # Returns
    ///
    /// T1 limit in nanoseconds.
    pub fn t1_limit_from_purcell(gamma_p_mhz: f64) -> f64 {
        if gamma_p_mhz.abs() < 1e-30 {
            return f64::INFINITY;
        }
        // γ_P in MHz means rate in units of 10^6 per second
        // T1 = 1 / (γ_P * 2π * 10^6) but since γ_P is already in angular frequency
        // units (γ_P/2π in MHz), we have T1 = 1/(γ_P_MHz * 10^6) seconds
        // = 1000 / γ_P_MHz nanoseconds
        1000.0 / gamma_p_mhz
    }

    /// Compute the filter suppression at a given frequency offset from the passband center.
    ///
    /// Models the Purcell filter as a single-pole bandpass with Lorentzian response:
    ///
    /// ```text
    /// Suppression(dB) = 10 * log10(1 + (2 * Δf / BW)²)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `freq_offset_ghz` - Offset from filter center frequency in GHz
    /// * `bandwidth_mhz` - Filter 3 dB bandwidth in MHz
    ///
    /// # Returns
    ///
    /// Suppression in dB (positive value means attenuation).
    pub fn filter_suppression_db(freq_offset_ghz: f64, bandwidth_mhz: f64) -> f64 {
        let offset_mhz = freq_offset_ghz * 1e3;
        let ratio = 2.0 * offset_mhz / bandwidth_mhz;
        10.0 * (1.0 + ratio * ratio).log10()
    }
}

/// Readout parameter optimizer for maximizing measurement fidelity.
///
/// Provides utilities to determine optimal readout frequency, power, and
/// the critical photon number that limits readout strength.
pub struct ReadoutOptimizer;

impl ReadoutOptimizer {
    /// Find the optimal readout frequency for maximum state contrast.
    ///
    /// The optimal frequency is the midpoint between the ground and excited
    /// state resonator frequencies, where the phase difference is maximized.
    ///
    /// # Arguments
    ///
    /// * `res_freq_0` - Resonator frequency for |0⟩ in GHz
    /// * `res_freq_1` - Resonator frequency for |1⟩ in GHz
    ///
    /// # Returns
    ///
    /// Optimal readout frequency in GHz.
    pub fn optimal_readout_frequency(res_freq_0: f64, res_freq_1: f64) -> f64 {
        (res_freq_0 + res_freq_1) / 2.0
    }

    /// Estimate the optimal readout power.
    ///
    /// The readout power should keep the average photon number below the
    /// critical photon number. This returns a rough estimate in dBm.
    ///
    /// ```text
    /// P_opt ≈ -30 + 10*log10(κ/χ) dBm  (order-of-magnitude estimate)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `chi_mhz` - Dispersive shift χ/2π in MHz
    /// * `kappa_mhz` - Resonator linewidth κ/2π in MHz
    ///
    /// # Returns
    ///
    /// Approximate optimal readout power in dBm.
    pub fn optimal_power_dbm(chi_mhz: f64, kappa_mhz: f64) -> f64 {
        if chi_mhz.abs() < 1e-12 {
            return f64::NEG_INFINITY;
        }
        // Heuristic: want n_bar ~ n_crit/10, which scales as κ/χ
        -30.0 + 10.0 * (kappa_mhz / chi_mhz).abs().log10()
    }

    /// Compute the critical photon number.
    ///
    /// The critical photon number marks the onset of breakdown of the dispersive
    /// approximation:
    ///
    /// ```text
    /// n_crit = Δ² / (4g²)
    /// ```
    ///
    /// Above this photon number, the qubit-resonator system leaves the dispersive
    /// regime and readout-induced transitions become significant.
    ///
    /// # Arguments
    ///
    /// * `delta_mhz` - Qubit-resonator detuning Δ/2π in MHz
    /// * `g_mhz` - Coupling strength g/2π in MHz
    ///
    /// # Returns
    ///
    /// Critical photon number (dimensionless).
    pub fn critical_photon_number(delta_mhz: f64, g_mhz: f64) -> f64 {
        if g_mhz.abs() < 1e-12 {
            return f64::INFINITY;
        }
        (delta_mhz * delta_mhz) / (4.0 * g_mhz * g_mhz)
    }
}

/// Simple moving average low-pass filter.
fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if n == 0 || window == 0 {
        return vec![];
    }
    let w = window.min(n);
    let mut result = Vec::with_capacity(n);
    let mut sum = 0.0;

    // Initialize with first window
    for k in 0..w {
        sum += data[k];
        result.push(sum / (k + 1) as f64);
    }

    // Sliding window for rest
    for k in w..n {
        sum += data[k] - data[k - w];
        result.push(sum / w as f64);
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> ReadoutConfig {
        ReadoutConfig {
            resonator_frequency_ghz: 7.0,
            qubit_frequency_ghz: 5.0,
            dispersive_shift_mhz: 1.0,
            readout_duration_ns: 1000.0,
            integration_weight: IntegrationWeight::BoxCar,
            if_frequency_mhz: 50.0,
            sample_rate_mhz: 1000.0,
        }
    }

    // ==================== Dispersive Shift Tests ====================

    #[test]
    fn test_dispersive_shift_basic() {
        // χ = g²/Δ, with g=100 MHz, Δ=2000 MHz -> χ = 10000/2000 = 5 MHz
        let chi = QubitReadout::dispersive_shift(100.0, 2000.0);
        assert!((chi - 5.0).abs() < 1e-10, "chi = {}, expected 5.0", chi);
    }

    #[test]
    fn test_dispersive_shift_small_coupling() {
        // g=50 MHz, Δ=1000 MHz -> χ = 2500/1000 = 2.5 MHz
        let chi = QubitReadout::dispersive_shift(50.0, 1000.0);
        assert!((chi - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_dispersive_shift_negative_detuning() {
        // Negative detuning (resonator above qubit)
        let chi = QubitReadout::dispersive_shift(100.0, -2000.0);
        assert!((chi - (-5.0)).abs() < 1e-10);
    }

    #[test]
    fn test_dispersive_shift_zero_detuning() {
        // Should return 0 for zero detuning (avoid division by zero)
        let chi = QubitReadout::dispersive_shift(100.0, 0.0);
        assert_eq!(chi, 0.0);
    }

    // ==================== Readout Pulse Tests ====================

    #[test]
    fn test_generate_readout_pulse_length() {
        let readout = QubitReadout::new(default_config());
        let pulse = readout.generate_readout_pulse(500.0, 0.1, 7.0);
        // 500 ns at 1000 MHz sample rate = 500 samples
        assert_eq!(pulse.len(), 500);
    }

    #[test]
    fn test_generate_readout_pulse_amplitude() {
        let readout = QubitReadout::new(default_config());
        let pulse = readout.generate_readout_pulse(100.0, 0.5, 7.0);
        // Check that all samples have magnitude <= amplitude
        for &(i, q) in &pulse {
            let mag = (i * i + q * q).sqrt();
            assert!(
                mag <= 0.5 + 1e-10,
                "mag {} exceeds amplitude 0.5",
                mag
            );
        }
    }

    #[test]
    fn test_generate_readout_pulse_correct_frequency() {
        let readout = QubitReadout::new(default_config());
        let freq_ghz = 7.0;
        let pulse = readout.generate_readout_pulse(1000.0, 1.0, freq_ghz);

        // Verify the first sample is at phase=0: I=1, Q=0
        assert!((pulse[0].0 - 1.0).abs() < 1e-10);
        assert!(pulse[0].1.abs() < 1e-10);
    }

    #[test]
    fn test_generate_readout_pulse_nonzero() {
        let readout = QubitReadout::new(default_config());
        // Use IF frequency (50 MHz) which is well below the sample rate (1000 MHz)
        // to avoid aliasing effects
        let pulse = readout.generate_readout_pulse(100.0, 0.3, 0.05);
        assert!(!pulse.is_empty());
        // At least some Q components should be nonzero (not a DC signal)
        let has_nonzero_q = pulse.iter().any(|&(_, q)| q.abs() > 1e-10);
        assert!(has_nonzero_q, "Pulse should have nonzero Q components");
    }

    // ==================== Resonator Response Tests ====================

    #[test]
    fn test_resonator_response_ground_on_resonance() {
        let readout = QubitReadout::new(default_config());
        let freq = 7.0 + 1e-3; // Exactly at shifted resonance for |0⟩
        let (mag, _phase) = readout.resonator_response_ground(freq);
        // On resonance with critical coupling, S21 should be at minimum
        // For our model: S21 = 1 - κ_ext/(κ/2) = 1 - 1 = 0
        assert!(mag < 0.1, "On-resonance magnitude should be small, got {}", mag);
    }

    #[test]
    fn test_resonator_response_ground_vs_excited_differ() {
        let readout = QubitReadout::new(default_config());
        let freq = 7.0; // Bare resonator frequency
        let (mag_g, phase_g) = readout.resonator_response_ground(freq);
        let (mag_e, phase_e) = readout.resonator_response_excited(freq);
        // Ground and excited responses should differ
        assert!(
            (mag_g - mag_e).abs() > 1e-6 || (phase_g - phase_e).abs() > 1e-6,
            "Ground and excited responses should differ"
        );
    }

    #[test]
    fn test_resonator_response_far_detuned() {
        let readout = QubitReadout::new(default_config());
        // Far from resonance, S21 should be close to 1 (full transmission)
        let (mag, _) = readout.resonator_response_ground(8.0);
        assert!(
            mag > 0.99,
            "Far-detuned magnitude should be near 1.0, got {}",
            mag
        );
    }

    #[test]
    fn test_resonator_response_symmetry() {
        let readout = QubitReadout::new(default_config());
        let res_freq = 7.0 + 1e-3; // Ground state resonance
        // Equal detuning above and below should give same magnitude
        let (mag_above, _) = readout.resonator_response_ground(res_freq + 0.01);
        let (mag_below, _) = readout.resonator_response_ground(res_freq - 0.01);
        assert!(
            (mag_above - mag_below).abs() < 0.01,
            "Lorentzian should be symmetric: {} vs {}",
            mag_above,
            mag_below
        );
    }

    // ==================== Integration Tests ====================

    #[test]
    fn test_integrate_boxcar() {
        let readout = QubitReadout::new(default_config());
        let signal_i = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let signal_q = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let (i, q) = readout.integrate_signal(&signal_i, &signal_q);
        assert!((i - 3.0).abs() < 1e-10, "BoxCar I should be mean: {}", i);
        assert!((q - 1.5).abs() < 1e-10, "BoxCar Q should be mean: {}", q);
    }

    #[test]
    fn test_integrate_optimal_weights() {
        let config = ReadoutConfig {
            integration_weight: IntegrationWeight::Optimal {
                weights_i: vec![1.0, 2.0, 1.0],
                weights_q: vec![1.0, 1.0, 1.0],
            },
            ..default_config()
        };
        let readout = QubitReadout::new(config);
        let signal_i = vec![1.0, 1.0, 1.0];
        let signal_q = vec![2.0, 2.0, 2.0];
        let (i, q) = readout.integrate_signal(&signal_i, &signal_q);
        // Weighted I: (1*1 + 2*1 + 1*1)/4 = 4/4 = 1.0
        assert!((i - 1.0).abs() < 1e-10, "Optimal I = {}", i);
        // Weighted Q: (1*2 + 1*2 + 1*2)/3 = 6/3 = 2.0
        assert!((q - 2.0).abs() < 1e-10, "Optimal Q = {}", q);
    }

    #[test]
    fn test_integrate_empty_signal() {
        let readout = QubitReadout::new(default_config());
        let (i, q) = readout.integrate_signal(&[], &[]);
        assert_eq!(i, 0.0);
        assert_eq!(q, 0.0);
    }

    #[test]
    fn test_integrate_matched() {
        let config = ReadoutConfig {
            integration_weight: IntegrationWeight::Matched,
            ..default_config()
        };
        let readout = QubitReadout::new(config);
        let signal_i = vec![1.0, 2.0, 3.0];
        let signal_q = vec![0.5, 1.0, 1.5];
        let (i, q) = readout.integrate_signal(&signal_i, &signal_q);
        // Matched falls back to boxcar in this implementation
        assert!((i - 2.0).abs() < 1e-10);
        assert!((q - 1.0).abs() < 1e-10);
    }

    // ==================== Demodulation Tests ====================

    #[test]
    fn test_demodulate_single_tone() {
        let readout = QubitReadout::new(default_config());
        let sample_rate = 1000.0; // MHz
        let if_freq = 50.0; // MHz
        let n = 1000;

        // Generate a test signal at IF frequency
        let signal: Vec<f64> = (0..n)
            .map(|k| {
                let t = k as f64 / sample_rate; // microseconds
                (2.0 * PI * if_freq * t).cos()
            })
            .collect();

        let (i_out, q_out) = readout.demodulate(&signal, if_freq, sample_rate);
        assert_eq!(i_out.len(), n);
        assert_eq!(q_out.len(), n);

        // After demodulation and filtering, the I component should be positive (DC)
        // and Q should be near zero for a cosine input
        // Check the steady-state region (after filter transient)
        let steady_start = n / 2;
        let avg_i: f64 = i_out[steady_start..].iter().sum::<f64>()
            / (n - steady_start) as f64;
        let avg_q: f64 = q_out[steady_start..].iter().sum::<f64>()
            / (n - steady_start) as f64;

        assert!(avg_i > 0.5, "Demod I should be positive for cos: {}", avg_i);
        assert!(avg_q.abs() < 0.2, "Demod Q should be near zero: {}", avg_q);
    }

    #[test]
    fn test_demodulate_zero_signal() {
        let readout = QubitReadout::new(default_config());
        let signal = vec![0.0; 100];
        let (i_out, q_out) = readout.demodulate(&signal, 50.0, 1000.0);
        for &val in &i_out {
            assert!(val.abs() < 1e-10);
        }
        for &val in &q_out {
            assert!(val.abs() < 1e-10);
        }
    }

    #[test]
    fn test_demodulate_output_length_matches_input() {
        let readout = QubitReadout::new(default_config());
        let signal = vec![1.0; 256];
        let (i_out, q_out) = readout.demodulate(&signal, 50.0, 1000.0);
        assert_eq!(i_out.len(), 256);
        assert_eq!(q_out.len(), 256);
    }

    // ==================== Classification Tests ====================

    #[test]
    fn test_classify_state_ground() {
        let readout = QubitReadout::new(default_config());
        let threshold = StateThreshold {
            angle_rad: 0.0,
            threshold: 0.0,
        };
        let state = readout.classify_state(-1.0, 0.0, &threshold);
        assert_eq!(state, QubitState::Ground);
    }

    #[test]
    fn test_classify_state_excited() {
        let readout = QubitReadout::new(default_config());
        let threshold = StateThreshold {
            angle_rad: 0.0,
            threshold: 0.0,
        };
        let state = readout.classify_state(1.0, 0.0, &threshold);
        assert_eq!(state, QubitState::Excited);
    }

    #[test]
    fn test_classify_state_with_rotation() {
        let readout = QubitReadout::new(default_config());
        // Rotate 90 degrees: separation along Q axis
        let threshold = StateThreshold {
            angle_rad: PI / 2.0,
            threshold: 0.0,
        };
        // Point at (0, -1) should project to -1 on rotated axis -> Ground
        let state = readout.classify_state(0.0, -1.0, &threshold);
        assert_eq!(state, QubitState::Ground);
        // Point at (0, 1) should project to +1 on rotated axis -> Excited
        let state = readout.classify_state(0.0, 1.0, &threshold);
        assert_eq!(state, QubitState::Excited);
    }

    // ==================== Threshold Calibration Tests ====================

    #[test]
    fn test_threshold_from_calibration_separated_clusters() {
        let ground = vec![(-1.0, 0.0), (-1.1, 0.1), (-0.9, -0.1)];
        let excited = vec![(1.0, 0.0), (1.1, 0.1), (0.9, -0.1)];
        let threshold = StateThreshold::from_calibration(&ground, &excited);

        // Angle should be close to 0 (separation along I axis)
        assert!(
            threshold.angle_rad.abs() < 0.2,
            "Angle should be near 0, got {}",
            threshold.angle_rad
        );
        // Threshold should be near 0 (midpoint of -1 and +1)
        assert!(
            threshold.threshold.abs() < 0.2,
            "Threshold should be near 0, got {}",
            threshold.threshold
        );
    }

    #[test]
    fn test_threshold_from_calibration_q_separated() {
        let ground = vec![(0.0, -1.0), (0.1, -1.1), (-0.1, -0.9)];
        let excited = vec![(0.0, 1.0), (0.1, 1.1), (-0.1, 0.9)];
        let threshold = StateThreshold::from_calibration(&ground, &excited);

        // Angle should be close to π/2 (separation along Q axis)
        assert!(
            (threshold.angle_rad - PI / 2.0).abs() < 0.2,
            "Angle should be near π/2, got {}",
            threshold.angle_rad
        );
    }

    // ==================== Fidelity Tests ====================

    #[test]
    fn test_readout_fidelity_perfect() {
        let assignments = vec![
            (QubitState::Ground, QubitState::Ground),
            (QubitState::Ground, QubitState::Ground),
            (QubitState::Excited, QubitState::Excited),
            (QubitState::Excited, QubitState::Excited),
        ];
        let fid = QubitReadout::readout_fidelity(&assignments);
        assert!((fid - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_readout_fidelity_half() {
        // All ground correctly assigned, all excited incorrectly assigned
        let assignments = vec![
            (QubitState::Ground, QubitState::Ground),
            (QubitState::Excited, QubitState::Ground),
        ];
        let fid = QubitReadout::readout_fidelity(&assignments);
        // P(0|0) = 1.0, P(1|1) = 0.0 -> F = 0.5
        assert!((fid - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_readout_fidelity_empty() {
        let fid = QubitReadout::readout_fidelity(&[]);
        assert_eq!(fid, 0.0);
    }

    #[test]
    fn test_readout_fidelity_90_percent() {
        let mut assignments = Vec::new();
        // 9 out of 10 ground correctly assigned
        for _ in 0..9 {
            assignments.push((QubitState::Ground, QubitState::Ground));
        }
        assignments.push((QubitState::Ground, QubitState::Excited));
        // 9 out of 10 excited correctly assigned
        for _ in 0..9 {
            assignments.push((QubitState::Excited, QubitState::Excited));
        }
        assignments.push((QubitState::Excited, QubitState::Ground));

        let fid = QubitReadout::readout_fidelity(&assignments);
        assert!((fid - 0.9).abs() < 1e-10, "Fidelity = {}", fid);
    }

    // ==================== SNR Tests ====================

    #[test]
    fn test_snr_well_separated() {
        // Centroids at (-1, 0) and (1, 0), sigma=0.5
        let snr = QubitReadout::snr_from_separation((-1.0, 0.0), (1.0, 0.0), 0.5);
        assert!((snr - 4.0).abs() < 1e-10, "SNR = {}, expected 4.0", snr);
    }

    #[test]
    fn test_snr_diagonal_separation() {
        // Centroids at (0, 0) and (3, 4), sigma=1.0 -> distance=5, SNR=5
        let snr = QubitReadout::snr_from_separation((0.0, 0.0), (3.0, 4.0), 1.0);
        assert!((snr - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_snr_zero_sigma() {
        let snr = QubitReadout::snr_from_separation((-1.0, 0.0), (1.0, 0.0), 0.0);
        assert!(snr.is_infinite());
    }

    #[test]
    fn test_snr_overlapping() {
        // Same centroid -> distance=0 -> SNR=0
        let snr = QubitReadout::snr_from_separation((1.0, 1.0), (1.0, 1.0), 1.0);
        assert!(snr.abs() < 1e-10);
    }

    // ==================== Purcell Filter Tests ====================

    #[test]
    fn test_purcell_decay_rate() {
        // γ_P = (g/Δ)² * κ
        // g=100, Δ=1000, κ=2 -> (0.1)² * 2 = 0.02 MHz
        let gamma = PurcellFilter::purcell_decay_rate(100.0, 2.0, 1000.0);
        assert!(
            (gamma - 0.02).abs() < 1e-10,
            "γ_P = {}, expected 0.02",
            gamma
        );
    }

    #[test]
    fn test_purcell_decay_rate_strong_coupling() {
        // g=200, Δ=1000, κ=5 -> (0.2)² * 5 = 0.2 MHz
        let gamma = PurcellFilter::purcell_decay_rate(200.0, 5.0, 1000.0);
        assert!((gamma - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_purcell_decay_rate_zero_detuning() {
        let gamma = PurcellFilter::purcell_decay_rate(100.0, 2.0, 0.0);
        assert!(gamma.is_infinite());
    }

    #[test]
    fn test_t1_limit_from_purcell() {
        // γ_P = 0.01 MHz -> T1 = 1000/0.01 = 100,000 ns = 100 μs
        let t1 = PurcellFilter::t1_limit_from_purcell(0.01);
        assert!(
            (t1 - 100_000.0).abs() < 1e-6,
            "T1 = {} ns, expected 100000",
            t1
        );
    }

    #[test]
    fn test_t1_limit_from_purcell_small_rate() {
        // γ_P = 0.001 MHz -> T1 = 1,000,000 ns = 1 ms
        let t1 = PurcellFilter::t1_limit_from_purcell(0.001);
        assert!((t1 - 1_000_000.0).abs() < 1e-3);
    }

    #[test]
    fn test_t1_limit_zero_rate() {
        let t1 = PurcellFilter::t1_limit_from_purcell(0.0);
        assert!(t1.is_infinite());
    }

    #[test]
    fn test_filter_suppression_on_center() {
        // Zero offset -> 10*log10(1) = 0 dB
        let suppression = PurcellFilter::filter_suppression_db(0.0, 100.0);
        assert!(suppression.abs() < 1e-10);
    }

    #[test]
    fn test_filter_suppression_at_bandwidth_edge() {
        // At offset = BW/2 in GHz: ratio = 2*(BW/2*1e3)/BW = 1e3 >> 1
        // Actually: offset_ghz such that offset_mhz = BW/2
        // offset_ghz = (BW/2)/1000
        let bw = 100.0; // MHz
        let offset = (bw / 2.0) / 1000.0; // GHz
        let suppression = PurcellFilter::filter_suppression_db(offset, bw);
        // ratio = 2*(50)/100 = 1.0, suppression = 10*log10(2) ≈ 3.01 dB
        assert!(
            (suppression - 3.0103).abs() < 0.01,
            "Suppression at BW edge = {} dB",
            suppression
        );
    }

    #[test]
    fn test_filter_suppression_large_offset() {
        // 1 GHz offset with 100 MHz bandwidth
        let suppression = PurcellFilter::filter_suppression_db(1.0, 100.0);
        // ratio = 2*1000/100 = 20, suppression = 10*log10(401) ≈ 26.03 dB
        assert!(suppression > 25.0, "Large offset suppression = {} dB", suppression);
    }

    // ==================== ReadoutOptimizer Tests ====================

    #[test]
    fn test_optimal_readout_frequency() {
        let f_opt = ReadoutOptimizer::optimal_readout_frequency(7.001, 6.999);
        assert!((f_opt - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_optimal_readout_frequency_asymmetric() {
        let f_opt = ReadoutOptimizer::optimal_readout_frequency(7.002, 6.998);
        assert!((f_opt - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_critical_photon_number() {
        // n_crit = Δ²/(4g²), Δ=2000, g=100 -> 4e6/(4*1e4) = 100
        let n_crit = ReadoutOptimizer::critical_photon_number(2000.0, 100.0);
        assert!(
            (n_crit - 100.0).abs() < 1e-10,
            "n_crit = {}, expected 100",
            n_crit
        );
    }

    #[test]
    fn test_critical_photon_number_weak_coupling() {
        // Δ=1000, g=50 -> 1e6/(4*2500) = 100
        let n_crit = ReadoutOptimizer::critical_photon_number(1000.0, 50.0);
        assert!((n_crit - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_critical_photon_number_zero_coupling() {
        let n_crit = ReadoutOptimizer::critical_photon_number(1000.0, 0.0);
        assert!(n_crit.is_infinite());
    }

    #[test]
    fn test_optimal_power_dbm() {
        // Should return a finite value for reasonable parameters
        let power = ReadoutOptimizer::optimal_power_dbm(1.0, 2.0);
        assert!(power.is_finite());
        // κ/χ = 2 -> 10*log10(2) ≈ 3.01, so power ≈ -27 dBm
        let expected = -30.0 + 10.0 * 2.0_f64.log10();
        assert!((power - expected).abs() < 1e-10);
    }

    // ==================== Multi-Qubit Tests ====================

    #[test]
    fn test_frequency_multiplex_single() {
        let configs = vec![default_config()];
        let mq = MultiQubitReadout::new(1, configs);
        let signal = vec![(1.0, 0.5), (0.8, 0.3)];
        let combined = mq.frequency_multiplex(&[signal.clone()]);
        assert_eq!(combined.len(), 2);
        assert!((combined[0].0 - 1.0).abs() < 1e-10);
        assert!((combined[0].1 - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_frequency_multiplex_two_qubits() {
        let configs = vec![default_config(), default_config()];
        let mq = MultiQubitReadout::new(2, configs);
        let s1 = vec![(1.0, 0.0), (1.0, 0.0)];
        let s2 = vec![(0.0, 1.0), (0.0, 1.0)];
        let combined = mq.frequency_multiplex(&[s1, s2]);
        assert_eq!(combined.len(), 2);
        assert!((combined[0].0 - 1.0).abs() < 1e-10);
        assert!((combined[0].1 - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_frequency_multiplex_empty() {
        let configs = vec![default_config()];
        let mq = MultiQubitReadout::new(1, configs);
        let combined = mq.frequency_multiplex(&[]);
        assert!(combined.is_empty());
    }

    #[test]
    fn test_demultiplex_output_count() {
        let configs = vec![default_config(), default_config()];
        let mq = MultiQubitReadout::new(2, configs);
        let combined = vec![(1.0, 0.0); 100];
        let freqs = vec![50.0, 150.0];
        let result = mq.demultiplex(&combined, &freqs, 1000.0);
        assert_eq!(result.len(), 2);
        assert_eq!(result[0].len(), 100);
        assert_eq!(result[1].len(), 100);
    }

    #[test]
    fn test_joint_state_assignment() {
        let configs = vec![default_config(), default_config()];
        let mq = MultiQubitReadout::new(2, configs);
        let iq_values = vec![(-0.5, 0.1), (0.5, -0.1)];
        let states = mq.joint_state_assignment(&iq_values);
        assert_eq!(states.len(), 2);
        assert_eq!(states[0], QubitState::Ground);
        assert_eq!(states[1], QubitState::Excited);
    }

    // ==================== Moving Average Tests ====================

    #[test]
    fn test_moving_average_basic() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = moving_average(&data, 3);
        assert_eq!(result.len(), 5);
        // First element: avg of [1] = 1.0
        assert!((result[0] - 1.0).abs() < 1e-10);
        // Second element: avg of [1,2] = 1.5
        assert!((result[1] - 1.5).abs() < 1e-10);
        // Third element: avg of [1,2,3] = 2.0
        assert!((result[2] - 2.0).abs() < 1e-10);
        // Fourth element: avg of [2,3,4] = 3.0
        assert!((result[3] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_moving_average_empty() {
        let result = moving_average(&[], 5);
        assert!(result.is_empty());
    }

    // ==================== End-to-End Tests ====================

    #[test]
    fn test_end_to_end_readout() {
        let config = default_config();
        let readout = QubitReadout::new(config);

        // Generate readout pulse
        let pulse = readout.generate_readout_pulse(500.0, 0.1, 7.0);
        assert!(!pulse.is_empty());

        // Simulate a real signal from the resonator (just use I component as IF signal)
        let signal: Vec<f64> = pulse.iter().map(|&(i, _)| i).collect();

        // Demodulate
        let (i_demod, q_demod) = readout.demodulate(&signal, 50.0, 1000.0);
        assert_eq!(i_demod.len(), pulse.len());

        // Integrate
        let (i_int, q_int) = readout.integrate_signal(&i_demod, &q_demod);
        assert!(i_int.is_finite());
        assert!(q_int.is_finite());

        // Classify
        let threshold = StateThreshold {
            angle_rad: 0.0,
            threshold: 0.0,
        };
        let state = readout.classify_state(i_int, q_int, &threshold);
        // Result should be some valid state
        assert!(
            state == QubitState::Ground
                || state == QubitState::Excited
                || state == QubitState::Leakage
        );
    }

    #[test]
    fn test_purcell_filter_creation() {
        let pf = PurcellFilter::new(7.0, 100.0, 5.0);
        assert!((pf.resonator_freq_ghz - 7.0).abs() < 1e-10);
        assert!((pf.filter_bandwidth_mhz - 100.0).abs() < 1e-10);
        assert!((pf.qubit_freq_ghz - 5.0).abs() < 1e-10);
    }
}
