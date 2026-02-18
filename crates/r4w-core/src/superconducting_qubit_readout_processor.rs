//! # Superconducting Qubit Readout Processor
//!
//! Signal processing for dispersive readout of superconducting qubits
//! (transmon, Xmon, fluxonium) in circuit quantum electrodynamics (cQED).
//!
//! ## Physical Background
//!
//! In a cQED architecture, a superconducting qubit is capacitively or inductively
//! coupled to a microwave readout resonator. The qubit-state-dependent dispersive
//! shift causes the resonator frequency to differ by 2*chi depending on whether
//! the qubit occupies |0> or |1>. A microwave tone at or near the resonator
//! frequency acquires a state-dependent phase and amplitude upon reflection or
//! transmission, enabling quantum non-demolition (QND) measurement.
//!
//! ## Dispersive Hamiltonian
//!
//! In the dispersive regime (|Delta| >> g), the Jaynes-Cummings Hamiltonian
//! reduces to:
//!
//! ```text
//! H_disp = (omega_r + chi * sigma_z) * a^dag a + omega_q/2 * sigma_z
//! ```
//!
//! where:
//! - `chi = -g^2 * alpha / (Delta * (Delta + alpha))` is the dispersive shift
//! - `g` is the qubit-resonator vacuum Rabi coupling
//! - `Delta = omega_q - omega_r` is the qubit-resonator detuning
//! - `alpha` is the qubit anharmonicity (negative for transmon)
//!
//! ## Readout Signal Chain
//!
//! ```text
//! Drive tone -> Resonator -> HEMT/TWPA -> IF mixing -> ADC -> DDC -> Integrate -> Threshold
//!                  |
//!            Qubit state encodes
//!            phase/amplitude shift
//! ```
//!
//! ## Key Parameters
//!
//! - **chi (dispersive shift)**: Typically 0.5-5 MHz for transmons
//! - **kappa (cavity linewidth)**: Sets readout bandwidth, typically 1-10 MHz
//! - **n_crit (critical photon number)**: Delta^2 / (4*g^2), max photons before
//!   leaving dispersive regime
//! - **Purcell rate**: gamma_P = kappa * (g/Delta)^2, enhanced qubit decay
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::superconducting_qubit_readout_processor::*;
//!
//! let config = QubitReadoutConfig {
//!     readout_freq_ghz: 7.0,
//!     readout_power_dbm: -30.0,
//!     integration_time_ns: 1000.0,
//!     qubit_freq_ghz: 5.0,
//!     anharmonicity_mhz: -200.0,
//!     coupling_strength_mhz: 100.0,
//!     kappa_mhz: 2.0,
//!     t1_us: 50.0,
//!     t2_us: 30.0,
//!     sample_rate_mhz: 1000.0,
//! };
//!
//! let processor = QubitReadoutProcessor::new(config);
//! let chi = processor.dispersive_shift();
//! assert!(chi.abs() > 0.0);
//! ```

use std::f64::consts::PI;

/// Configuration for superconducting qubit dispersive readout.
///
/// Contains all physical parameters needed to model the qubit-resonator
/// system and perform state discrimination from readout signals.
#[derive(Debug, Clone)]
pub struct QubitReadoutConfig {
    /// Readout resonator frequency in GHz (typically 6-8 GHz).
    pub readout_freq_ghz: f64,
    /// Readout drive power in dBm (typically -40 to -20 dBm).
    pub readout_power_dbm: f64,
    /// Integration time for readout signal in nanoseconds (typically 300-2000 ns).
    pub integration_time_ns: f64,
    /// Qubit transition frequency in GHz (typically 4-6 GHz for transmon).
    pub qubit_freq_ghz: f64,
    /// Qubit anharmonicity alpha/2pi in MHz (typically -200 to -300 MHz for transmon).
    pub anharmonicity_mhz: f64,
    /// Qubit-resonator coupling strength g/2pi in MHz (typically 50-200 MHz).
    pub coupling_strength_mhz: f64,
    /// Cavity linewidth kappa/2pi in MHz (typically 1-10 MHz).
    pub kappa_mhz: f64,
    /// Qubit energy relaxation time T1 in microseconds.
    pub t1_us: f64,
    /// Qubit dephasing time T2 in microseconds (T2 <= 2*T1).
    pub t2_us: f64,
    /// ADC sample rate in MHz.
    pub sample_rate_mhz: f64,
}

/// Result of IQ cloud analysis for qubit state distributions.
#[derive(Debug, Clone)]
pub struct IqCloudResult {
    /// Centroid (I, Q) of the |0> state distribution.
    pub center_0: (f64, f64),
    /// Centroid (I, Q) of the |1> state distribution.
    pub center_1: (f64, f64),
    /// Covariance matrix [[var_i, cov_iq], [cov_iq, var_q]] for |0> state.
    pub covariance_0: [[f64; 2]; 2],
    /// Covariance matrix [[var_i, cov_iq], [cov_iq, var_q]] for |1> state.
    pub covariance_1: [[f64; 2]; 2],
    /// Separation distance between centroids.
    pub separation: f64,
}

/// Confusion matrix for readout assignment.
#[derive(Debug, Clone)]
pub struct ConfusionMatrix {
    /// P(assign 0 | prepared 0).
    pub p00: f64,
    /// P(assign 1 | prepared 0).
    pub p01: f64,
    /// P(assign 0 | prepared 1).
    pub p10: f64,
    /// P(assign 1 | prepared 1).
    pub p11: f64,
}

/// Result of exponential decay fitting.
#[derive(Debug, Clone)]
pub struct DecayFitResult {
    /// Fitted amplitude (initial value).
    pub amplitude: f64,
    /// Fitted decay time constant (same units as input times).
    pub decay_time: f64,
    /// Fitted offset (asymptotic value).
    pub offset: f64,
    /// Sum of squared residuals.
    pub residual_sum_sq: f64,
}

/// Result of Ramsey / echo fitting (decaying oscillation).
#[derive(Debug, Clone)]
pub struct RamseyFitResult {
    /// Fitted amplitude.
    pub amplitude: f64,
    /// Fitted coherence time T2* or T2.
    pub coherence_time: f64,
    /// Fitted detuning frequency (for Ramsey) in same units as 1/time.
    pub detuning_freq: f64,
    /// Fitted phase offset in radians.
    pub phase_offset: f64,
    /// Fitted offset (baseline).
    pub offset: f64,
    /// Sum of squared residuals.
    pub residual_sum_sq: f64,
}

/// Superconducting qubit readout signal processor.
///
/// Provides methods for dispersive readout signal generation, demodulation,
/// state discrimination, and qubit characterization (T1, T2*, T2).
///
/// The processor models the qubit-resonator interaction in the dispersive
/// regime and implements the full readout signal chain from tone generation
/// through integration and thresholding.
#[derive(Debug, Clone)]
pub struct QubitReadoutProcessor {
    config: QubitReadoutConfig,
}

impl QubitReadoutProcessor {
    /// Create a new qubit readout processor with the given configuration.
    pub fn new(config: QubitReadoutConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &QubitReadoutConfig {
        &self.config
    }

    // -----------------------------------------------------------------------
    // Dispersive shift calculation
    // -----------------------------------------------------------------------

    /// Calculate the dispersive shift chi/2pi in MHz from the qubit-cavity
    /// coupling strength g, detuning Delta, and anharmonicity alpha.
    ///
    /// For a transmon qubit the dispersive shift including the second excited
    /// state (Straddling regime formula) is:
    ///
    /// ```text
    /// chi = -g^2 * alpha / (Delta * (Delta + alpha))
    /// ```
    ///
    /// where Delta = omega_q - omega_r and alpha is the anharmonicity
    /// (negative for transmon).
    ///
    /// # Returns
    ///
    /// Dispersive shift chi/2pi in MHz. Typically negative for transmon qubits.
    pub fn dispersive_shift(&self) -> f64 {
        let g = self.config.coupling_strength_mhz; // MHz
        let delta = (self.config.qubit_freq_ghz - self.config.readout_freq_ghz) * 1000.0; // MHz
        let alpha = self.config.anharmonicity_mhz; // MHz

        let denom = delta * (delta + alpha);
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        -g * g * alpha / denom
    }

    // -----------------------------------------------------------------------
    // Readout signal generation
    // -----------------------------------------------------------------------

    /// Generate IQ readout tone for qubit in |0> or |1> state.
    ///
    /// The readout signal is a CW tone at the resonator frequency shifted by
    /// +chi (for |0>) or -chi (for |1>). The amplitude envelope is flat
    /// (square pulse) over the integration window.
    ///
    /// # Arguments
    ///
    /// * `state` - 0 for ground state, 1 for excited state
    /// * `n_samples` - number of IQ samples to generate
    /// * `amplitude` - signal amplitude (linear scale)
    ///
    /// # Returns
    ///
    /// Vector of (I, Q) samples at the IF representing the readout response.
    pub fn readout_signal(&self, state: u8, n_samples: usize, amplitude: f64) -> Vec<(f64, f64)> {
        let chi = self.dispersive_shift(); // MHz
        // IF offset is state-dependent: |0> sees +chi, |1> sees -chi
        let freq_offset_mhz = if state == 0 { chi } else { -chi };

        let dt = 1.0 / self.config.sample_rate_mhz; // in microseconds
        let omega = 2.0 * PI * freq_offset_mhz; // rad/us

        (0..n_samples)
            .map(|n| {
                let t = n as f64 * dt;
                let phase = omega * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Optimal weight function (matched filter)
    // -----------------------------------------------------------------------

    /// Compute the optimal weight function for state discrimination.
    ///
    /// The optimal integration weights are the time-domain difference between
    /// the |0> and |1> readout signals, weighted by an exponential decay
    /// envelope accounting for cavity ring-up/down (kappa).
    ///
    /// ```text
    /// w(t) = (s_0(t) - s_1(t)) * exp(-kappa * t / 2)
    /// ```
    ///
    /// The weights are normalized so that sum(|w|^2) = 1.
    ///
    /// # Arguments
    ///
    /// * `n_samples` - number of weight samples
    ///
    /// # Returns
    ///
    /// Vector of (w_I, w_Q) weight pairs.
    pub fn optimal_weight_function(&self, n_samples: usize) -> Vec<(f64, f64)> {
        let s0 = self.readout_signal(0, n_samples, 1.0);
        let s1 = self.readout_signal(1, n_samples, 1.0);

        // kappa_mhz is the linewidth in MHz; half-linewidth for exponential decay envelope
        let kappa_half = PI * self.config.kappa_mhz; // rad/us
        let dt = 1.0 / self.config.sample_rate_mhz; // us

        let mut weights: Vec<(f64, f64)> = (0..n_samples)
            .map(|n| {
                let t = n as f64 * dt;
                let envelope = (-kappa_half * t).exp();
                let di = (s0[n].0 - s1[n].0) * envelope;
                let dq = (s0[n].1 - s1[n].1) * envelope;
                (di, dq)
            })
            .collect();

        // Normalize
        let norm_sq: f64 = weights.iter().map(|(wi, wq)| wi * wi + wq * wq).sum();
        if norm_sq > 1e-30 {
            let norm = norm_sq.sqrt();
            for w in &mut weights {
                w.0 /= norm;
                w.1 /= norm;
            }
        }

        weights
    }

    // -----------------------------------------------------------------------
    // State discrimination
    // -----------------------------------------------------------------------

    /// Perform single-shot state discrimination on integrated IQ data.
    ///
    /// Uses a linear discriminator: projects each IQ point onto the axis
    /// connecting the |0> and |1> centroids, then applies a threshold at the
    /// midpoint.
    ///
    /// # Arguments
    ///
    /// * `iq_point` - measured (I, Q) value after integration
    /// * `center_0` - calibrated |0> centroid
    /// * `center_1` - calibrated |1> centroid
    ///
    /// # Returns
    ///
    /// 0 for ground state, 1 for excited state.
    pub fn state_discrimination(
        &self,
        iq_point: (f64, f64),
        center_0: (f64, f64),
        center_1: (f64, f64),
    ) -> u8 {
        // Discrimination axis: unit vector from center_0 to center_1
        let dx = center_1.0 - center_0.0;
        let dy = center_1.1 - center_0.1;
        let norm = (dx * dx + dy * dy).sqrt();
        if norm < 1e-15 {
            return 0;
        }
        let ux = dx / norm;
        let uy = dy / norm;

        // Project the IQ point onto the discrimination axis
        let proj = (iq_point.0 - center_0.0) * ux + (iq_point.1 - center_0.1) * uy;

        // Threshold at the midpoint (norm / 2)
        if proj > norm / 2.0 {
            1
        } else {
            0
        }
    }

    // -----------------------------------------------------------------------
    // Assignment fidelity
    // -----------------------------------------------------------------------

    /// Calculate readout assignment fidelity from a confusion matrix.
    ///
    /// The assignment fidelity is defined as:
    ///
    /// ```text
    /// F = 1 - (P(1|0) + P(0|1)) / 2
    /// ```
    ///
    /// where P(1|0) is the probability of assigning |1> when |0> was prepared,
    /// and P(0|1) is the probability of assigning |0> when |1> was prepared.
    ///
    /// # Arguments
    ///
    /// * `confusion` - Confusion matrix from calibration measurements
    ///
    /// # Returns
    ///
    /// Assignment fidelity in [0, 1].
    pub fn assignment_fidelity(&self, confusion: &ConfusionMatrix) -> f64 {
        1.0 - (confusion.p01 + confusion.p10) / 2.0
    }

    /// Build a confusion matrix from calibration single-shot data.
    ///
    /// # Arguments
    ///
    /// * `ground_shots` - IQ shots prepared in |0>
    /// * `excited_shots` - IQ shots prepared in |1>
    /// * `center_0` - |0> centroid for discrimination
    /// * `center_1` - |1> centroid for discrimination
    ///
    /// # Returns
    ///
    /// Confusion matrix with assignment probabilities.
    pub fn build_confusion_matrix(
        &self,
        ground_shots: &[(f64, f64)],
        excited_shots: &[(f64, f64)],
        center_0: (f64, f64),
        center_1: (f64, f64),
    ) -> ConfusionMatrix {
        let n0 = ground_shots.len() as f64;
        let n1 = excited_shots.len() as f64;

        let mut count_00 = 0usize;
        for &shot in ground_shots {
            if self.state_discrimination(shot, center_0, center_1) == 0 {
                count_00 += 1;
            }
        }
        let count_10 = ground_shots.len() - count_00;

        let mut count_11 = 0usize;
        for &shot in excited_shots {
            if self.state_discrimination(shot, center_0, center_1) == 1 {
                count_11 += 1;
            }
        }
        let count_01 = excited_shots.len() - count_11;

        ConfusionMatrix {
            p00: count_00 as f64 / n0.max(1.0),
            p01: count_10 as f64 / n0.max(1.0), // P(assign 1 | prepared 0)
            p10: count_01 as f64 / n1.max(1.0), // P(assign 0 | prepared 1)
            p11: count_11 as f64 / n1.max(1.0),
        }
    }

    // -----------------------------------------------------------------------
    // IQ cloud analysis
    // -----------------------------------------------------------------------

    /// Compute centroids and covariance matrices of |0> and |1> IQ distributions.
    ///
    /// # Arguments
    ///
    /// * `ground_shots` - IQ points from |0>-prepared measurements
    /// * `excited_shots` - IQ points from |1>-prepared measurements
    ///
    /// # Returns
    ///
    /// `IqCloudResult` with centroids, covariance matrices, and separation distance.
    pub fn iq_cloud_centers(
        &self,
        ground_shots: &[(f64, f64)],
        excited_shots: &[(f64, f64)],
    ) -> IqCloudResult {
        let c0 = centroid(ground_shots);
        let c1 = centroid(excited_shots);
        let cov0 = covariance_matrix(ground_shots, c0);
        let cov1 = covariance_matrix(excited_shots, c1);
        let dx = c1.0 - c0.0;
        let dy = c1.1 - c0.1;
        let sep = (dx * dx + dy * dy).sqrt();

        IqCloudResult {
            center_0: c0,
            center_1: c1,
            covariance_0: cov0,
            covariance_1: cov1,
            separation: sep,
        }
    }

    // -----------------------------------------------------------------------
    // Readout SNR
    // -----------------------------------------------------------------------

    /// Compute readout signal-to-noise ratio.
    ///
    /// The SNR for dispersive readout is approximately:
    ///
    /// ```text
    /// SNR = 8 * eta * n_bar * chi^2 * T_int / kappa
    /// ```
    ///
    /// where eta is the quantum efficiency (assumed 1 here), n_bar is the
    /// mean photon number in the resonator, chi is the dispersive shift,
    /// kappa is the cavity linewidth, and T_int is the integration time.
    ///
    /// # Arguments
    ///
    /// * `n_photons` - mean intracavity photon number
    ///
    /// # Returns
    ///
    /// Linear SNR (not in dB).
    pub fn snr_readout(&self, n_photons: f64) -> f64 {
        let chi = self.dispersive_shift(); // MHz
        let kappa = self.config.kappa_mhz; // MHz
        let t_int = self.config.integration_time_ns / 1000.0; // convert ns to us

        if kappa.abs() < 1e-15 {
            return 0.0;
        }

        // SNR = 8 * n_bar * chi^2 * T_int / kappa
        // All in MHz and us: chi^2 (MHz^2) * t_int (us) / kappa (MHz)
        //   = MHz * us = dimensionless (since MHz * us = 1)
        8.0 * n_photons * chi * chi * t_int / kappa
    }

    // -----------------------------------------------------------------------
    // Purcell decay rate
    // -----------------------------------------------------------------------

    /// Calculate the Purcell-enhanced decay rate.
    ///
    /// The Purcell effect causes enhanced spontaneous emission of the qubit
    /// through the readout resonator. The Purcell decay rate is:
    ///
    /// ```text
    /// gamma_Purcell = kappa * (g / Delta)^2
    /// ```
    ///
    /// This sets a limit on qubit T1 from the readout channel alone:
    /// T1_Purcell = 1 / gamma_Purcell.
    ///
    /// # Returns
    ///
    /// Tuple of (gamma_purcell in MHz, T1_purcell in microseconds).
    pub fn purcell_decay_rate(&self) -> (f64, f64) {
        let g = self.config.coupling_strength_mhz;
        let delta = (self.config.qubit_freq_ghz - self.config.readout_freq_ghz) * 1000.0;
        let kappa = self.config.kappa_mhz;

        if delta.abs() < 1e-12 {
            return (f64::INFINITY, 0.0);
        }

        let gamma_purcell = kappa * (g / delta).powi(2); // MHz
        let t1_purcell = if gamma_purcell > 1e-15 {
            1.0 / (2.0 * PI * gamma_purcell) // us (since gamma is in MHz = 1/us * 2pi)
        } else {
            f64::INFINITY
        };

        (gamma_purcell, t1_purcell)
    }

    // -----------------------------------------------------------------------
    // Optimal readout power
    // -----------------------------------------------------------------------

    /// Estimate the optimal readout photon number balancing SNR against
    /// measurement-induced transitions.
    ///
    /// The critical photon number (above which the dispersive approximation
    /// breaks down) is:
    ///
    /// ```text
    /// n_crit = Delta^2 / (4 * g^2)
    /// ```
    ///
    /// The optimal readout typically uses n_bar ~ n_crit / 4 to n_crit / 2
    /// to stay safely in the dispersive regime while maximizing SNR.
    ///
    /// # Returns
    ///
    /// Tuple of (n_crit, recommended n_bar, corresponding SNR).
    pub fn optimal_readout_power(&self) -> (f64, f64, f64) {
        let g = self.config.coupling_strength_mhz;
        let delta = (self.config.qubit_freq_ghz - self.config.readout_freq_ghz) * 1000.0;

        if g.abs() < 1e-15 {
            return (f64::INFINITY, 0.0, 0.0);
        }

        let n_crit = delta * delta / (4.0 * g * g);
        // Recommend n_bar = n_crit / 4 as a safe operating point
        let n_recommended = n_crit / 4.0;
        let snr = self.snr_readout(n_recommended);

        (n_crit, n_recommended, snr)
    }

    // -----------------------------------------------------------------------
    // Demodulate readout signal (digital downconversion)
    // -----------------------------------------------------------------------

    /// Digitally downconvert a readout signal from IF to baseband IQ.
    ///
    /// Mixes the input signal with a local oscillator at the specified IF
    /// frequency and applies a simple averaging (lowpass) filter.
    ///
    /// # Arguments
    ///
    /// * `signal_i` - in-phase IF samples
    /// * `signal_q` - quadrature IF samples
    /// * `if_freq_mhz` - intermediate frequency in MHz
    ///
    /// # Returns
    ///
    /// Vector of baseband (I, Q) samples.
    pub fn demodulate_readout(
        &self,
        signal_i: &[f64],
        signal_q: &[f64],
        if_freq_mhz: f64,
    ) -> Vec<(f64, f64)> {
        let n = signal_i.len().min(signal_q.len());
        let dt = 1.0 / self.config.sample_rate_mhz; // us
        let omega = 2.0 * PI * if_freq_mhz;

        (0..n)
            .map(|k| {
                let t = k as f64 * dt;
                let lo_i = (omega * t).cos();
                let lo_q = (omega * t).sin();

                // Complex multiply: (si + j*sq) * e^{-j*w*t}
                // = (si + j*sq) * (cos(wt) - j*sin(wt))
                // bb_i = si*cos(wt) + sq*sin(wt)
                // bb_q = sq*cos(wt) - si*sin(wt)
                let bb_i = signal_i[k] * lo_i + signal_q[k] * lo_q;
                let bb_q = signal_q[k] * lo_i - signal_i[k] * lo_q;
                (bb_i, bb_q)
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // Integrate and threshold
    // -----------------------------------------------------------------------

    /// Perform boxcar or weighted integration of demodulated IQ data,
    /// then apply threshold-based state discrimination.
    ///
    /// # Arguments
    ///
    /// * `baseband_iq` - demodulated (I, Q) samples
    /// * `weights` - optional weight function; if None, uses boxcar (uniform)
    /// * `center_0` - calibrated |0> centroid
    /// * `center_1` - calibrated |1> centroid
    ///
    /// # Returns
    ///
    /// Tuple of (integrated IQ point, assigned state 0 or 1).
    pub fn integrate_and_threshold(
        &self,
        baseband_iq: &[(f64, f64)],
        weights: Option<&[(f64, f64)]>,
        center_0: (f64, f64),
        center_1: (f64, f64),
    ) -> ((f64, f64), u8) {
        let n = baseband_iq.len();
        if n == 0 {
            return ((0.0, 0.0), 0);
        }

        let (sum_i, sum_q) = match weights {
            Some(w) => {
                let wn = w.len().min(n);
                let mut si = 0.0;
                let mut sq = 0.0;
                for k in 0..wn {
                    // Weighted integration: Re(iq * conj(w))
                    si += baseband_iq[k].0 * w[k].0 + baseband_iq[k].1 * w[k].1;
                    sq += baseband_iq[k].1 * w[k].0 - baseband_iq[k].0 * w[k].1;
                }
                (si, sq)
            }
            None => {
                let mut si = 0.0;
                let mut sq = 0.0;
                for &(i, q) in baseband_iq {
                    si += i;
                    sq += q;
                }
                (si / n as f64, sq / n as f64)
            }
        };

        let integrated = (sum_i, sum_q);
        let state = self.state_discrimination(integrated, center_0, center_1);
        (integrated, state)
    }

    // -----------------------------------------------------------------------
    // T1 measurement (exponential decay fit)
    // -----------------------------------------------------------------------

    /// Fit T1 from exponential decay data.
    ///
    /// Given delay times and corresponding excited-state population
    /// measurements, fit the model:
    ///
    /// ```text
    /// P(t) = A * exp(-t / T1) + B
    /// ```
    ///
    /// Uses linearized least-squares on log(P - offset) with iterative
    /// offset estimation.
    ///
    /// # Arguments
    ///
    /// * `times` - delay times (arbitrary units, e.g. microseconds)
    /// * `populations` - measured excited-state probability at each delay
    ///
    /// # Returns
    ///
    /// `DecayFitResult` with amplitude, T1, offset, and residual.
    pub fn t1_measurement(&self, times: &[f64], populations: &[f64]) -> DecayFitResult {
        fit_exponential_decay(times, populations)
    }

    // -----------------------------------------------------------------------
    // T2 Ramsey (T2*)
    // -----------------------------------------------------------------------

    /// Fit T2* from Ramsey interference fringes.
    ///
    /// Given delay times and Ramsey fringe data, fit the model:
    ///
    /// ```text
    /// P(t) = A * exp(-t / T2*) * cos(2*pi*f_det*t + phi) + B
    /// ```
    ///
    /// Uses a coarse frequency search followed by linear regression on
    /// the envelope.
    ///
    /// # Arguments
    ///
    /// * `times` - free-precession delay times
    /// * `populations` - measured population at each delay
    ///
    /// # Returns
    ///
    /// `RamseyFitResult` with amplitude, T2*, detuning, phase, offset.
    pub fn t2_ramsey(&self, times: &[f64], populations: &[f64]) -> RamseyFitResult {
        fit_decaying_oscillation(times, populations)
    }

    // -----------------------------------------------------------------------
    // T2 Hahn echo
    // -----------------------------------------------------------------------

    /// Fit T2 from Hahn echo decay data.
    ///
    /// The echo removes low-frequency dephasing, so the decay is a simple
    /// exponential:
    ///
    /// ```text
    /// P(t) = A * exp(-t / T2) + B
    /// ```
    ///
    /// (No oscillation; the echo refocuses static detuning.)
    ///
    /// # Arguments
    ///
    /// * `times` - total echo delay (2*tau) times
    /// * `populations` - measured population at each delay
    ///
    /// # Returns
    ///
    /// `RamseyFitResult` with coherence_time = T2, detuning_freq = 0.
    pub fn t2_echo(&self, times: &[f64], populations: &[f64]) -> RamseyFitResult {
        let decay = fit_exponential_decay(times, populations);
        RamseyFitResult {
            amplitude: decay.amplitude,
            coherence_time: decay.decay_time,
            detuning_freq: 0.0,
            phase_offset: 0.0,
            offset: decay.offset,
            residual_sum_sq: decay.residual_sum_sq,
        }
    }

    // -----------------------------------------------------------------------
    // Readout crosstalk correction
    // -----------------------------------------------------------------------

    /// Correct multi-qubit readout crosstalk via matrix inversion.
    ///
    /// Given a calibration matrix M where M[i][j] = P(assign i | prepared j),
    /// the corrected probabilities are:
    ///
    /// ```text
    /// p_corrected = M^{-1} * p_measured
    /// ```
    ///
    /// For a 2-qubit system, M is 4x4 (states: 00, 01, 10, 11).
    /// This implementation handles NxN matrices via Gaussian elimination.
    ///
    /// # Arguments
    ///
    /// * `calibration_matrix` - NxN assignment matrix (row i, col j = P(assign i | state j))
    /// * `measured_probs` - measured probability vector (length N)
    ///
    /// # Returns
    ///
    /// Corrected probability vector, or None if the matrix is singular.
    pub fn readout_crosstalk_correction(
        &self,
        calibration_matrix: &[Vec<f64>],
        measured_probs: &[f64],
    ) -> Option<Vec<f64>> {
        let n = calibration_matrix.len();
        if n == 0 || measured_probs.len() != n {
            return None;
        }
        for row in calibration_matrix {
            if row.len() != n {
                return None;
            }
        }

        // Invert the calibration matrix using Gaussian elimination
        let inv = invert_matrix(calibration_matrix)?;

        // Multiply inv * measured_probs
        let mut result = vec![0.0; n];
        for i in 0..n {
            for j in 0..n {
                result[i] += inv[i][j] * measured_probs[j];
            }
        }

        Some(result)
    }
}

// ===========================================================================
// Helper functions
// ===========================================================================

/// Compute the centroid of a set of (I, Q) points.
fn centroid(points: &[(f64, f64)]) -> (f64, f64) {
    if points.is_empty() {
        return (0.0, 0.0);
    }
    let n = points.len() as f64;
    let (si, sq) = points.iter().fold((0.0, 0.0), |(ai, aq), &(i, q)| (ai + i, aq + q));
    (si / n, sq / n)
}

/// Compute the 2x2 covariance matrix of IQ data given the centroid.
fn covariance_matrix(points: &[(f64, f64)], center: (f64, f64)) -> [[f64; 2]; 2] {
    if points.len() < 2 {
        return [[0.0; 2]; 2];
    }
    let n = points.len() as f64;
    let mut var_i = 0.0;
    let mut var_q = 0.0;
    let mut cov_iq = 0.0;
    for &(i, q) in points {
        let di = i - center.0;
        let dq = q - center.1;
        var_i += di * di;
        var_q += dq * dq;
        cov_iq += di * dq;
    }
    let d = n - 1.0;
    [[var_i / d, cov_iq / d], [cov_iq / d, var_q / d]]
}

/// Invert an NxN matrix using Gaussian elimination with partial pivoting.
///
/// Returns None if the matrix is singular.
fn invert_matrix(matrix: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = matrix.len();
    // Augmented matrix [M | I]
    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(2 * n);
        row.extend_from_slice(&matrix[i]);
        for j in 0..n {
            row.push(if i == j { 1.0 } else { 0.0 });
        }
        aug.push(row);
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = aug[row][col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        if max_val < 1e-14 {
            return None; // Singular
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        let pivot = aug[col][col];
        for j in 0..(2 * n) {
            aug[col][j] /= pivot;
        }

        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Extract inverse from augmented matrix
    let inv: Vec<Vec<f64>> = aug
        .into_iter()
        .map(|row| row[n..].to_vec())
        .collect();

    Some(inv)
}

/// Fit y = A * exp(-t / tau) + B to data using iterative linearized least squares.
///
/// Strategy: estimate B from the tail, then do log-linear regression.
fn fit_exponential_decay(times: &[f64], values: &[f64]) -> DecayFitResult {
    let n = times.len().min(values.len());
    if n < 3 {
        return DecayFitResult {
            amplitude: 0.0,
            decay_time: 1.0,
            offset: 0.0,
            residual_sum_sq: 0.0,
        };
    }

    // Estimate offset B from the last quarter of the data
    let tail_start = (3 * n) / 4;
    let tail_count = n - tail_start;
    let b_est = if tail_count > 0 {
        values[tail_start..n].iter().sum::<f64>() / tail_count as f64
    } else {
        0.0
    };

    // Iterative fitting: try a few offset values
    let mut best_residual = f64::MAX;
    let mut best_a = 1.0;
    let mut best_tau = 1.0;
    let mut best_b = b_est;

    for b_trial_idx in 0..5 {
        let b = b_est + (b_trial_idx as f64 - 2.0) * 0.02 * (values[0] - b_est).abs().max(0.01);

        // Log-linear regression: log(y - B) = log(A) - t / tau
        let mut sum_t = 0.0;
        let mut sum_ly = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_tly = 0.0;
        let mut count = 0.0;

        for k in 0..n {
            let yk = values[k] - b;
            if yk > 1e-10 {
                let ly = yk.ln();
                let t = times[k];
                sum_t += t;
                sum_ly += ly;
                sum_t2 += t * t;
                sum_tly += t * ly;
                count += 1.0;
            }
        }

        if count < 2.0 {
            continue;
        }

        let det = count * sum_t2 - sum_t * sum_t;
        if det.abs() < 1e-30 {
            continue;
        }

        let log_a = (sum_ly * sum_t2 - sum_t * sum_tly) / det;
        let slope = (count * sum_tly - sum_t * sum_ly) / det;

        let a = log_a.exp();
        let tau = if slope.abs() > 1e-15 { -1.0 / slope } else { 1e6 };

        if tau <= 0.0 {
            continue;
        }

        // Compute residual
        let mut resid = 0.0;
        for k in 0..n {
            let pred = a * (-times[k] / tau).exp() + b;
            let diff = values[k] - pred;
            resid += diff * diff;
        }

        if resid < best_residual {
            best_residual = resid;
            best_a = a;
            best_tau = tau;
            best_b = b;
        }
    }

    DecayFitResult {
        amplitude: best_a,
        decay_time: best_tau,
        offset: best_b,
        residual_sum_sq: best_residual,
    }
}

/// Fit y = A * exp(-t / T2) * cos(2*pi*f*t + phi) + B to Ramsey data.
///
/// Strategy:
/// 1. Estimate offset from mean
/// 2. Coarse frequency search via peak of DFT
/// 3. Fit envelope via log of rectified signal
fn fit_decaying_oscillation(times: &[f64], values: &[f64]) -> RamseyFitResult {
    let n = times.len().min(values.len());
    if n < 4 {
        return RamseyFitResult {
            amplitude: 0.0,
            coherence_time: 1.0,
            detuning_freq: 0.0,
            phase_offset: 0.0,
            offset: 0.0,
            residual_sum_sq: 0.0,
        };
    }

    // Step 1: Estimate offset from mean
    let mean_val = values.iter().take(n).sum::<f64>() / n as f64;
    let centered: Vec<f64> = values.iter().take(n).map(|&v| v - mean_val).collect();

    // Step 2: Coarse frequency search using DFT
    let t_span = times[n - 1] - times[0];
    let dt_avg = t_span / (n - 1) as f64;
    let f_max = 0.5 / dt_avg; // Nyquist

    let n_freq = 200;
    let mut best_power = 0.0;
    let mut best_freq = 0.0;

    for fi in 1..n_freq {
        let f = (fi as f64 / n_freq as f64) * f_max;
        let mut re = 0.0;
        let mut im = 0.0;
        for k in 0..n {
            let phase = 2.0 * PI * f * times[k];
            re += centered[k] * phase.cos();
            im += centered[k] * phase.sin();
        }
        let power = re * re + im * im;
        if power > best_power {
            best_power = power;
            best_freq = f;
        }
    }

    // Step 3: Estimate phase from the DFT at the best frequency
    let mut re_sum = 0.0;
    let mut im_sum = 0.0;
    for k in 0..n {
        let phase = 2.0 * PI * best_freq * times[k];
        re_sum += centered[k] * phase.cos();
        im_sum += centered[k] * phase.sin();
    }
    let phase_est = (-im_sum).atan2(re_sum);

    // Step 4: Estimate amplitude and decay via envelope fitting
    // Demodulate: multiply by cos(2*pi*f*t + phi) and lowpass (average)
    let mut envelope_data: Vec<(f64, f64)> = Vec::new(); // (time, |envelope|)
    let window = (n / 10).max(3);

    for k in 0..n {
        // Smooth with a small window around sample k
        let start = if k >= window / 2 { k - window / 2 } else { 0 };
        let end = (k + window / 2 + 1).min(n);
        let mut avg = 0.0;
        for j in start..end {
            let c = (2.0 * PI * best_freq * times[j] + phase_est).cos();
            avg += centered[j] * c * 2.0;
        }
        avg /= (end - start) as f64;

        if avg.abs() > 1e-10 {
            envelope_data.push((times[k], avg.abs()));
        }
    }

    // Fit exponential to envelope
    let env_times: Vec<f64> = envelope_data.iter().map(|&(t, _)| t).collect();
    let env_vals: Vec<f64> = envelope_data.iter().map(|&(_, v)| v).collect();

    let (amplitude, coherence_time) = if env_times.len() >= 3 {
        let env_fit = fit_exponential_decay(&env_times, &env_vals);
        (env_fit.amplitude, env_fit.decay_time.max(dt_avg))
    } else {
        // Fallback: use peak value and rough estimate
        let peak = centered.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        (peak, t_span)
    };

    // Compute residual
    let mut resid_sum = 0.0;
    for k in 0..n {
        let pred =
            amplitude * (-times[k] / coherence_time).exp() * (2.0 * PI * best_freq * times[k] + phase_est).cos()
                + mean_val;
        let diff = values[k] - pred;
        resid_sum += diff * diff;
    }

    RamseyFitResult {
        amplitude,
        coherence_time,
        detuning_freq: best_freq,
        phase_offset: phase_est,
        offset: mean_val,
        residual_sum_sq: resid_sum,
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Standard test configuration representing a typical transmon qubit.
    fn test_config() -> QubitReadoutConfig {
        QubitReadoutConfig {
            readout_freq_ghz: 7.0,
            readout_power_dbm: -30.0,
            integration_time_ns: 1000.0,
            qubit_freq_ghz: 5.0,
            anharmonicity_mhz: -200.0,
            coupling_strength_mhz: 100.0,
            kappa_mhz: 2.0,
            t1_us: 50.0,
            t2_us: 30.0,
            sample_rate_mhz: 1000.0,
        }
    }

    fn make_processor() -> QubitReadoutProcessor {
        QubitReadoutProcessor::new(test_config())
    }

    // -----------------------------------------------------------------------
    // Dispersive shift tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_dispersive_shift_sign() {
        let p = make_processor();
        let chi = p.dispersive_shift();
        // For a transmon (alpha < 0) with qubit below resonator (Delta < 0),
        // chi = -g^2 * alpha / (Delta * (Delta + alpha))
        // Delta = 5.0 - 7.0 = -2.0 GHz = -2000 MHz
        // alpha = -200 MHz
        // chi = -100^2 * (-200) / (-2000 * (-2000 + (-200)))
        //     = -10000 * (-200) / (-2000 * -2200)
        //     = 2000000 / 4400000 = 0.4545... MHz
        assert!(chi > 0.0, "chi should be positive: got {}", chi);
    }

    #[test]
    fn test_dispersive_shift_value() {
        let p = make_processor();
        let chi = p.dispersive_shift();
        // Expected: -100^2 * (-200) / (-2000 * -2200)
        //         = 2_000_000 / 4_400_000 = ~0.4545 MHz
        let expected = 2_000_000.0 / 4_400_000.0;
        assert!(
            (chi - expected).abs() < 1e-6,
            "chi = {}, expected = {}",
            chi,
            expected
        );
    }

    #[test]
    fn test_dispersive_shift_increases_with_coupling() {
        let mut cfg = test_config();
        cfg.coupling_strength_mhz = 50.0;
        let chi_small = QubitReadoutProcessor::new(cfg.clone()).dispersive_shift();

        cfg.coupling_strength_mhz = 150.0;
        let chi_large = QubitReadoutProcessor::new(cfg).dispersive_shift();

        assert!(
            chi_large.abs() > chi_small.abs(),
            "Larger coupling should give larger |chi|"
        );
    }

    #[test]
    fn test_dispersive_shift_zero_detuning() {
        let mut cfg = test_config();
        cfg.qubit_freq_ghz = cfg.readout_freq_ghz; // Delta = 0
        let p = QubitReadoutProcessor::new(cfg);
        let chi = p.dispersive_shift();
        // Should handle gracefully (denom ~ alpha * 0 through Delta*(Delta+alpha))
        // Actually Delta = 0, so denom = 0 * (0 + alpha) = 0
        assert_eq!(chi, 0.0, "chi should be 0 when Delta = 0");
    }

    // -----------------------------------------------------------------------
    // Readout signal tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_readout_signal_length() {
        let p = make_processor();
        let sig = p.readout_signal(0, 100, 1.0);
        assert_eq!(sig.len(), 100);
    }

    #[test]
    fn test_readout_signal_amplitude() {
        let p = make_processor();
        let amp = 0.5;
        let sig = p.readout_signal(0, 1000, amp);
        for &(i, q) in &sig {
            let mag = (i * i + q * q).sqrt();
            assert!(
                (mag - amp).abs() < 1e-10,
                "Signal magnitude should equal amplitude"
            );
        }
    }

    #[test]
    fn test_readout_signal_different_states() {
        let p = make_processor();
        let sig0 = p.readout_signal(0, 100, 1.0);
        let sig1 = p.readout_signal(1, 100, 1.0);

        // Signals for |0> and |1> should differ (different frequencies)
        let mut diff_sum = 0.0;
        for k in 1..100 {
            let di = sig0[k].0 - sig1[k].0;
            let dq = sig0[k].1 - sig1[k].1;
            diff_sum += di * di + dq * dq;
        }
        assert!(diff_sum > 1e-6, "State 0 and 1 signals should differ");
    }

    #[test]
    fn test_readout_signal_first_sample() {
        let p = make_processor();
        let sig = p.readout_signal(0, 10, 1.0);
        // At t=0, phase=0, so I=1, Q=0
        assert!((sig[0].0 - 1.0).abs() < 1e-10);
        assert!(sig[0].1.abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Optimal weight function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_optimal_weights_normalized() {
        let p = make_processor();
        let w = p.optimal_weight_function(200);
        let norm_sq: f64 = w.iter().map(|(wi, wq)| wi * wi + wq * wq).sum();
        assert!(
            (norm_sq - 1.0).abs() < 1e-6,
            "Weights should be normalized: ||w||^2 = {}",
            norm_sq
        );
    }

    #[test]
    fn test_optimal_weights_length() {
        let p = make_processor();
        let w = p.optimal_weight_function(500);
        assert_eq!(w.len(), 500);
    }

    #[test]
    fn test_optimal_weights_decay() {
        let p = make_processor();
        let w = p.optimal_weight_function(500);
        // The envelope of the weights should decay over time
        let early_power: f64 = w[..50]
            .iter()
            .map(|(wi, wq)| wi * wi + wq * wq)
            .sum::<f64>()
            / 50.0;
        let late_power: f64 = w[450..]
            .iter()
            .map(|(wi, wq)| wi * wi + wq * wq)
            .sum::<f64>()
            / 50.0;
        assert!(
            early_power > late_power,
            "Early weights should have more power than late weights"
        );
    }

    // -----------------------------------------------------------------------
    // State discrimination tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_discrimination_ground_state() {
        let p = make_processor();
        let center_0 = (1.0, 0.0);
        let center_1 = (-1.0, 0.0);
        // Point near center_0
        assert_eq!(p.state_discrimination((0.8, 0.1), center_0, center_1), 0);
    }

    #[test]
    fn test_discrimination_excited_state() {
        let p = make_processor();
        let center_0 = (1.0, 0.0);
        let center_1 = (-1.0, 0.0);
        // Point near center_1
        assert_eq!(p.state_discrimination((-0.8, 0.1), center_0, center_1), 1);
    }

    #[test]
    fn test_discrimination_on_boundary() {
        let p = make_processor();
        let center_0 = (1.0, 0.0);
        let center_1 = (-1.0, 0.0);
        // Point exactly at midpoint -> should be 0 (threshold is >)
        assert_eq!(p.state_discrimination((0.0, 0.0), center_0, center_1), 0);
    }

    #[test]
    fn test_discrimination_rotated_axis() {
        let p = make_processor();
        let center_0 = (0.0, 1.0);
        let center_1 = (0.0, -1.0);
        // Point near center_0 in Q direction
        assert_eq!(p.state_discrimination((0.0, 0.7), center_0, center_1), 0);
        // Point near center_1 in Q direction
        assert_eq!(p.state_discrimination((0.0, -0.7), center_0, center_1), 1);
    }

    // -----------------------------------------------------------------------
    // Assignment fidelity tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_perfect_fidelity() {
        let p = make_processor();
        let cm = ConfusionMatrix {
            p00: 1.0,
            p01: 0.0,
            p10: 0.0,
            p11: 1.0,
        };
        assert!((p.assignment_fidelity(&cm) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_random_fidelity() {
        let p = make_processor();
        let cm = ConfusionMatrix {
            p00: 0.5,
            p01: 0.5,
            p10: 0.5,
            p11: 0.5,
        };
        assert!((p.assignment_fidelity(&cm) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_typical_fidelity() {
        let p = make_processor();
        let cm = ConfusionMatrix {
            p00: 0.98,
            p01: 0.02,
            p10: 0.03,
            p11: 0.97,
        };
        let f = p.assignment_fidelity(&cm);
        // F = 1 - (0.02 + 0.03) / 2 = 0.975
        assert!((f - 0.975).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // IQ cloud analysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_iq_cloud_centers_separation() {
        let p = make_processor();
        let ground = vec![(1.0, 0.0), (1.1, 0.1), (0.9, -0.1), (1.0, 0.05)];
        let excited = vec![(-1.0, 0.0), (-1.1, 0.1), (-0.9, -0.1), (-1.0, 0.05)];

        let result = p.iq_cloud_centers(&ground, &excited);
        assert!(result.separation > 1.5, "States should be well separated");
        assert!(result.center_0.0 > 0.0, "|0> should have positive I");
        assert!(result.center_1.0 < 0.0, "|1> should have negative I");
    }

    #[test]
    fn test_iq_cloud_covariance() {
        let p = make_processor();
        let ground = vec![
            (1.0, 0.0),
            (1.2, 0.0),
            (0.8, 0.0),
            (1.0, 0.2),
            (1.0, -0.2),
        ];
        let excited = vec![(-1.0, 0.0)];

        let result = p.iq_cloud_centers(&ground, &excited);
        // Variance in I should be non-zero
        assert!(result.covariance_0[0][0] > 0.0, "Var(I) for |0> should be > 0");
        assert!(result.covariance_0[1][1] > 0.0, "Var(Q) for |0> should be > 0");
    }

    // -----------------------------------------------------------------------
    // Readout SNR tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_snr_positive() {
        let p = make_processor();
        let snr = p.snr_readout(5.0);
        assert!(snr > 0.0, "SNR should be positive for n_photons > 0");
    }

    #[test]
    fn test_snr_zero_photons() {
        let p = make_processor();
        assert_eq!(p.snr_readout(0.0), 0.0, "SNR should be 0 for 0 photons");
    }

    #[test]
    fn test_snr_increases_with_photons() {
        let p = make_processor();
        let snr_1 = p.snr_readout(1.0);
        let snr_10 = p.snr_readout(10.0);
        assert!(snr_10 > snr_1, "SNR should increase with photon number");
    }

    #[test]
    fn test_snr_scales_with_chi_squared() {
        // SNR ~ chi^2, so doubling g (which ~doubles chi) should ~4x SNR
        let mut cfg1 = test_config();
        cfg1.coupling_strength_mhz = 50.0;
        let p1 = QubitReadoutProcessor::new(cfg1);

        let mut cfg2 = test_config();
        cfg2.coupling_strength_mhz = 100.0;
        let p2 = QubitReadoutProcessor::new(cfg2);

        let snr1 = p1.snr_readout(5.0);
        let snr2 = p2.snr_readout(5.0);

        let chi1 = p1.dispersive_shift();
        let chi2 = p2.dispersive_shift();
        let expected_ratio = (chi2 / chi1).powi(2);

        let actual_ratio = snr2 / snr1;
        assert!(
            (actual_ratio - expected_ratio).abs() / expected_ratio < 0.01,
            "SNR ratio {} should match chi^2 ratio {}",
            actual_ratio,
            expected_ratio
        );
    }

    // -----------------------------------------------------------------------
    // Purcell decay rate tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_purcell_positive() {
        let p = make_processor();
        let (gamma, t1_purcell) = p.purcell_decay_rate();
        assert!(gamma > 0.0, "Purcell rate should be positive");
        assert!(t1_purcell > 0.0, "Purcell T1 should be positive");
    }

    #[test]
    fn test_purcell_increases_with_coupling() {
        let mut cfg1 = test_config();
        cfg1.coupling_strength_mhz = 50.0;
        let (gamma1, _) = QubitReadoutProcessor::new(cfg1).purcell_decay_rate();

        let mut cfg2 = test_config();
        cfg2.coupling_strength_mhz = 200.0;
        let (gamma2, _) = QubitReadoutProcessor::new(cfg2).purcell_decay_rate();

        assert!(
            gamma2 > gamma1,
            "Purcell rate should increase with coupling: {} vs {}",
            gamma2,
            gamma1
        );
    }

    #[test]
    fn test_purcell_scales_as_g_over_delta_squared() {
        let p = make_processor();
        let (gamma, _) = p.purcell_decay_rate();

        let g = p.config().coupling_strength_mhz;
        let delta = (p.config().qubit_freq_ghz - p.config().readout_freq_ghz) * 1000.0;
        let kappa = p.config().kappa_mhz;

        let expected = kappa * (g / delta).powi(2);
        assert!(
            (gamma - expected).abs() < 1e-10,
            "gamma = {}, expected = {}",
            gamma,
            expected
        );
    }

    // -----------------------------------------------------------------------
    // Optimal readout power tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_optimal_power_n_crit() {
        let p = make_processor();
        let (n_crit, n_rec, _snr) = p.optimal_readout_power();

        let g = p.config().coupling_strength_mhz;
        let delta = (p.config().qubit_freq_ghz - p.config().readout_freq_ghz) * 1000.0;
        let expected_n_crit = delta * delta / (4.0 * g * g);

        assert!(
            (n_crit - expected_n_crit).abs() < 1e-6,
            "n_crit = {}, expected = {}",
            n_crit,
            expected_n_crit
        );
        assert!(
            n_rec < n_crit,
            "Recommended n_bar should be less than n_crit"
        );
    }

    // -----------------------------------------------------------------------
    // Demodulation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_demodulate_dc_signal() {
        let p = make_processor();
        let n = 100;
        // A DC signal (0 Hz IF) should pass through as-is
        let si: Vec<f64> = vec![1.0; n];
        let sq: Vec<f64> = vec![0.0; n];
        let bb = p.demodulate_readout(&si, &sq, 0.0);
        assert_eq!(bb.len(), n);
        for &(i, q) in &bb {
            assert!((i - 1.0).abs() < 1e-10);
            assert!(q.abs() < 1e-10);
        }
    }

    #[test]
    fn test_demodulate_tone() {
        let p = make_processor();
        let n = 1000;
        let if_freq = 50.0; // MHz
        let dt = 1.0 / p.config().sample_rate_mhz;

        // Generate a tone at the IF frequency
        let si: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * if_freq * k as f64 * dt).cos())
            .collect();
        let sq: Vec<f64> = (0..n)
            .map(|k| (2.0 * PI * if_freq * k as f64 * dt).sin())
            .collect();

        let bb = p.demodulate_readout(&si, &sq, if_freq);

        // After perfect downconversion, the baseband should be approximately DC
        // Average of the last half should be close to (1, 0)
        let avg_i: f64 = bb[n / 2..].iter().map(|&(i, _)| i).sum::<f64>() / (n / 2) as f64;
        let avg_q: f64 = bb[n / 2..].iter().map(|&(_, q)| q).sum::<f64>() / (n / 2) as f64;

        assert!(
            (avg_i - 1.0).abs() < 0.1,
            "Demodulated I should be ~1, got {}",
            avg_i
        );
        assert!(avg_q.abs() < 0.1, "Demodulated Q should be ~0, got {}", avg_q);
    }

    // -----------------------------------------------------------------------
    // Integrate and threshold tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_integrate_boxcar_ground() {
        let p = make_processor();
        let center_0 = (1.0, 0.0);
        let center_1 = (-1.0, 0.0);
        let iq: Vec<(f64, f64)> = vec![(0.9, 0.1); 100];

        let (integrated, state) = p.integrate_and_threshold(&iq, None, center_0, center_1);
        assert_eq!(state, 0, "Should classify as ground state");
        assert!((integrated.0 - 0.9).abs() < 1e-10);
    }

    #[test]
    fn test_integrate_boxcar_excited() {
        let p = make_processor();
        let center_0 = (1.0, 0.0);
        let center_1 = (-1.0, 0.0);
        let iq: Vec<(f64, f64)> = vec![(-0.8, 0.1); 100];

        let (_, state) = p.integrate_and_threshold(&iq, None, center_0, center_1);
        assert_eq!(state, 1, "Should classify as excited state");
    }

    #[test]
    fn test_integrate_empty() {
        let p = make_processor();
        let (integrated, state) = p.integrate_and_threshold(&[], None, (1.0, 0.0), (-1.0, 0.0));
        assert_eq!(state, 0);
        assert_eq!(integrated, (0.0, 0.0));
    }

    // -----------------------------------------------------------------------
    // T1 measurement tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_t1_fit_exponential() {
        let p = make_processor();
        let t1_true = 50.0; // us
        let amp = 0.95;
        let offset = 0.05;

        let times: Vec<f64> = (0..50).map(|k| k as f64 * 5.0).collect(); // 0, 5, 10, ...
        let pops: Vec<f64> = times
            .iter()
            .map(|&t| amp * (-t / t1_true).exp() + offset)
            .collect();

        let result = p.t1_measurement(&times, &pops);
        assert!(
            (result.decay_time - t1_true).abs() / t1_true < 0.1,
            "T1 fit: got {}, expected {}",
            result.decay_time,
            t1_true
        );
        assert!(
            (result.amplitude - amp).abs() / amp < 0.2,
            "Amplitude fit: got {}, expected {}",
            result.amplitude,
            amp
        );
    }

    // -----------------------------------------------------------------------
    // T2 Ramsey tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_t2_ramsey_fit() {
        let p = make_processor();
        let t2_star = 20.0; // us
        let detuning = 0.5; // MHz
        let amp = 0.45;
        let offset = 0.5;

        let times: Vec<f64> = (0..100).map(|k| k as f64 * 0.5).collect(); // 0 to 50 us
        let pops: Vec<f64> = times
            .iter()
            .map(|&t| {
                amp * (-t / t2_star).exp() * (2.0 * PI * detuning * t).cos() + offset
            })
            .collect();

        let result = p.t2_ramsey(&times, &pops);
        assert!(
            (result.detuning_freq - detuning).abs() / detuning < 0.15,
            "Detuning fit: got {}, expected {}",
            result.detuning_freq,
            detuning
        );
        assert!(
            result.coherence_time > 0.0,
            "T2* should be positive: got {}",
            result.coherence_time
        );
    }

    // -----------------------------------------------------------------------
    // T2 echo tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_t2_echo_fit() {
        let p = make_processor();
        let t2_true = 40.0;
        let amp = 0.9;
        let offset = 0.05;

        let times: Vec<f64> = (0..40).map(|k| k as f64 * 5.0).collect();
        let pops: Vec<f64> = times
            .iter()
            .map(|&t| amp * (-t / t2_true).exp() + offset)
            .collect();

        let result = p.t2_echo(&times, &pops);
        assert!(
            (result.coherence_time - t2_true).abs() / t2_true < 0.15,
            "T2 echo fit: got {}, expected {}",
            result.coherence_time,
            t2_true
        );
        assert!(
            (result.detuning_freq).abs() < 1e-10,
            "Echo should have zero detuning"
        );
    }

    // -----------------------------------------------------------------------
    // Crosstalk correction tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_crosstalk_identity() {
        let p = make_processor();
        // Identity calibration matrix -> no correction needed
        let cal = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let measured = vec![0.6, 0.4];
        let corrected = p.readout_crosstalk_correction(&cal, &measured).unwrap();
        assert!((corrected[0] - 0.6).abs() < 1e-10);
        assert!((corrected[1] - 0.4).abs() < 1e-10);
    }

    #[test]
    fn test_crosstalk_correction_2x2() {
        let p = make_processor();
        // Realistic 2-qubit calibration with crosstalk
        let cal = vec![vec![0.95, 0.05], vec![0.05, 0.95]];
        let measured = vec![0.55, 0.45];

        let corrected = p.readout_crosstalk_correction(&cal, &measured).unwrap();

        // Verify the correction is invertible: cal * corrected ≈ measured
        let check_0 = cal[0][0] * corrected[0] + cal[0][1] * corrected[1];
        let check_1 = cal[1][0] * corrected[0] + cal[1][1] * corrected[1];
        assert!(
            (check_0 - measured[0]).abs() < 1e-10,
            "Reconstruction check failed"
        );
        assert!(
            (check_1 - measured[1]).abs() < 1e-10,
            "Reconstruction check failed"
        );
    }

    #[test]
    fn test_crosstalk_singular_matrix() {
        let p = make_processor();
        let cal = vec![vec![1.0, 1.0], vec![1.0, 1.0]]; // Singular
        let measured = vec![0.5, 0.5];
        assert!(
            p.readout_crosstalk_correction(&cal, &measured).is_none(),
            "Should return None for singular matrix"
        );
    }

    #[test]
    fn test_crosstalk_size_mismatch() {
        let p = make_processor();
        let cal = vec![vec![1.0, 0.0], vec![0.0, 1.0]];
        let measured = vec![0.5, 0.3, 0.2]; // Wrong size
        assert!(
            p.readout_crosstalk_correction(&cal, &measured).is_none(),
            "Should return None for size mismatch"
        );
    }

    #[test]
    fn test_crosstalk_4x4() {
        let p = make_processor();
        // 4-state (2-qubit) calibration matrix
        let cal = vec![
            vec![0.90, 0.03, 0.04, 0.01],
            vec![0.03, 0.92, 0.01, 0.04],
            vec![0.04, 0.01, 0.91, 0.03],
            vec![0.03, 0.04, 0.04, 0.92],
        ];
        let measured = vec![0.25, 0.25, 0.25, 0.25];
        let corrected = p.readout_crosstalk_correction(&cal, &measured).unwrap();

        // Verify reconstruction
        for i in 0..4 {
            let check: f64 = (0..4).map(|j| cal[i][j] * corrected[j]).sum();
            assert!(
                (check - measured[i]).abs() < 1e-8,
                "4x4 reconstruction failed at index {}",
                i
            );
        }
    }

    // -----------------------------------------------------------------------
    // Confusion matrix and fidelity integration test
    // -----------------------------------------------------------------------

    #[test]
    fn test_build_confusion_matrix() {
        let p = make_processor();
        let center_0 = (1.0, 0.0);
        let center_1 = (-1.0, 0.0);

        // Ground state shots clustered near center_0
        let ground: Vec<(f64, f64)> = vec![
            (0.9, 0.05),
            (1.1, -0.05),
            (0.95, 0.0),
            (1.05, 0.1),
            (0.85, -0.1),
        ];
        // Excited state shots clustered near center_1
        let excited: Vec<(f64, f64)> = vec![
            (-0.9, 0.05),
            (-1.1, -0.05),
            (-0.95, 0.0),
            (-1.05, 0.1),
            (-0.85, -0.1),
        ];

        let cm = p.build_confusion_matrix(&ground, &excited, center_0, center_1);
        assert!(
            cm.p00 > 0.9,
            "P(0|0) should be high: got {}",
            cm.p00
        );
        assert!(
            cm.p11 > 0.9,
            "P(1|1) should be high: got {}",
            cm.p11
        );

        let fid = p.assignment_fidelity(&cm);
        assert!(fid > 0.9, "Fidelity should be high: got {}", fid);
    }

    // -----------------------------------------------------------------------
    // Config accessor test
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_accessor() {
        let p = make_processor();
        assert!((p.config().readout_freq_ghz - 7.0).abs() < 1e-10);
        assert!((p.config().qubit_freq_ghz - 5.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Helper function tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_centroid_single_point() {
        let c = centroid(&[(3.0, 4.0)]);
        assert!((c.0 - 3.0).abs() < 1e-10);
        assert!((c.1 - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_centroid_empty() {
        let c = centroid(&[]);
        assert_eq!(c, (0.0, 0.0));
    }

    #[test]
    fn test_covariance_diagonal() {
        // Points spread only along I
        let points = vec![(1.0, 0.0), (3.0, 0.0), (5.0, 0.0)];
        let c = centroid(&points);
        let cov = covariance_matrix(&points, c);
        assert!(cov[0][0] > 0.0, "Var(I) should be positive");
        assert!(cov[1][1].abs() < 1e-10, "Var(Q) should be zero");
        assert!(cov[0][1].abs() < 1e-10, "Cov(I,Q) should be zero");
    }

    #[test]
    fn test_invert_2x2() {
        let m = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let inv = invert_matrix(&m).unwrap();
        // M * M^-1 should be identity
        for i in 0..2 {
            for j in 0..2 {
                let val: f64 = (0..2).map(|k| m[i][k] * inv[k][j]).sum();
                let expected = if i == j { 1.0 } else { 0.0 };
                assert!(
                    (val - expected).abs() < 1e-10,
                    "M*M^-1[{}][{}] = {}, expected {}",
                    i,
                    j,
                    val,
                    expected
                );
            }
        }
    }
}
