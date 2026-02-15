//! # Quantum Illumination Radar Signal Processor
//!
//! This module implements quantum illumination radar signal processing. Quantum radar
//! uses entangled photon pairs (signal and idler) where the signal interrogates the
//! target while the idler is retained locally. Correlating the returned signal with
//! the idler provides a quantum advantage in target detection against bright thermal
//! noise backgrounds, even though entanglement is destroyed upon interaction with
//! the lossy, noisy channel.
//!
//! ## Background
//!
//! Quantum illumination (QI) was proposed by Seth Lloyd (2008) and rigorously analyzed
//! by Tan et al. (2008). The key insight is that even though entanglement between the
//! signal and idler is destroyed by the noisy channel, the correlations in a two-mode
//! squeezed vacuum (TMSV) state still provide a 6 dB advantage in the error exponent
//! for target detection compared to the best classical transmitter using the same
//! mean photon number per mode.
//!
//! ## Key Equations
//!
//! - **Quantum SNR**: `SNR_QI = 4 * M * kappa * N_S / N_B`
//! - **Classical SNR**: `SNR_CL = M * kappa^2 * N_S^2 / N_B`
//! - **Quantum Advantage**: `10 * log10(4 * N_B / (kappa * N_S))` dB
//! - **Error probability (QI Chernoff)**: `P_e = 0.5 * exp(-SNR/2)`
//!
//! where:
//! - `M` = number of signal-idler temporal modes
//! - `kappa` = target reflectivity (0 to 1)
//! - `N_S` = mean signal photon number per mode (typically << 1)
//! - `N_B` = mean background thermal noise photon number per mode (typically >> 1)
//!
//! ## References
//!
//! - Lloyd, S. "Enhanced sensitivity of photodetection via quantum illumination."
//!   Science 321, 1463-1465 (2008).
//! - Tan, S. H. et al. "Quantum illumination with Gaussian states."
//!   Phys. Rev. Lett. 101, 253601 (2008).
//! - Guha, S. & Erkmen, B. I. "Gaussian-state quantum-illumination receivers for
//!   target detection." Phys. Rev. A 80, 052310 (2009).

use std::f64::consts::PI;

/// Planck constant in joule-seconds (J·s).
pub const PLANCK_CONSTANT: f64 = 6.62607015e-34;

/// Boltzmann constant in joules per kelvin (J/K).
pub const BOLTZMANN_CONSTANT: f64 = 1.380649e-23;

/// Speed of light in vacuum in meters per second (m/s).
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Configuration for a quantum illumination radar system.
///
/// Specifies the operating parameters including signal frequency, bandwidth,
/// number of temporal modes, photon statistics, target properties, and
/// observation geometry.
#[derive(Debug, Clone)]
pub struct QuantumRadarConfig {
    /// Signal carrier frequency in GHz (microwave regime: 1-10 GHz typical).
    pub signal_frequency_ghz: f64,
    /// Signal bandwidth in MHz.
    pub bandwidth_mhz: f64,
    /// Number of signal-idler temporal modes (M). More modes improve SNR linearly.
    pub num_signal_modes: usize,
    /// Mean signal photon number per mode (N_S). Typically much less than 1
    /// for quantum advantage (e.g., 0.001 to 0.1).
    pub mean_photon_number: f64,
    /// Mean background thermal noise photon number per mode (N_B). Typically much
    /// greater than 1 for microwave frequencies at room temperature.
    pub noise_photon_number: f64,
    /// Target reflectivity (kappa), ranging from 0 (no reflection) to 1 (perfect mirror).
    pub target_reflectivity: f64,
    /// Integration time in seconds. Determines the number of available temporal modes.
    pub integration_time_s: f64,
    /// Target range in meters.
    pub range_m: f64,
}

impl QuantumRadarConfig {
    /// Creates a new configuration with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `signal_frequency_ghz` - Carrier frequency in GHz
    /// * `bandwidth_mhz` - Signal bandwidth in MHz
    /// * `num_signal_modes` - Number of temporal modes M
    /// * `mean_photon_number` - Mean signal photons per mode N_S
    /// * `noise_photon_number` - Mean noise photons per mode N_B
    /// * `target_reflectivity` - Target reflectivity kappa (0-1)
    /// * `integration_time_s` - Observation time in seconds
    /// * `range_m` - Target range in meters
    pub fn new(
        signal_frequency_ghz: f64,
        bandwidth_mhz: f64,
        num_signal_modes: usize,
        mean_photon_number: f64,
        noise_photon_number: f64,
        target_reflectivity: f64,
        integration_time_s: f64,
        range_m: f64,
    ) -> Self {
        Self {
            signal_frequency_ghz,
            bandwidth_mhz,
            num_signal_modes,
            mean_photon_number,
            noise_photon_number,
            target_reflectivity,
            integration_time_s,
            range_m,
        }
    }

    /// Returns a default configuration suitable for microwave quantum radar testing.
    ///
    /// Parameters: 3 GHz, 100 MHz bandwidth, 10000 modes, N_S=0.01, N_B=100,
    /// kappa=0.1, 1 ms integration, 100 m range.
    pub fn default_microwave() -> Self {
        Self {
            signal_frequency_ghz: 3.0,
            bandwidth_mhz: 100.0,
            num_signal_modes: 10_000,
            mean_photon_number: 0.01,
            noise_photon_number: 100.0,
            target_reflectivity: 0.1,
            integration_time_s: 1e-3,
            range_m: 100.0,
        }
    }
}

/// Quantum illumination signal processor.
///
/// Computes detection performance metrics for quantum illumination radar,
/// including SNR, error probabilities, detection thresholds, and receiver
/// operating characteristics. Compares quantum and classical performance
/// to quantify the quantum advantage.
pub struct QuantumIlluminationProcessor {
    config: QuantumRadarConfig,
}

impl QuantumIlluminationProcessor {
    /// Creates a new processor with the given configuration.
    pub fn new(config: QuantumRadarConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the current configuration.
    pub fn config(&self) -> &QuantumRadarConfig {
        &self.config
    }

    /// Computes the quantum illumination SNR.
    ///
    /// In the regime N_S << 1 and N_B >> 1, the quantum illumination SNR is:
    ///
    /// ```text
    /// SNR_QI = 4 * M * kappa * N_S / N_B
    /// ```
    ///
    /// This is the Chernoff-bound exponent for the quantum hypothesis test.
    ///
    /// # Arguments
    ///
    /// * `ns` - Mean signal photon number per mode
    /// * `nb` - Mean noise photon number per mode
    /// * `kappa` - Target reflectivity
    /// * `m` - Number of temporal modes
    pub fn quantum_snr(ns: f64, nb: f64, kappa: f64, m: usize) -> f64 {
        4.0 * (m as f64) * kappa * ns / nb
    }

    /// Computes the classical (coherent state) SNR.
    ///
    /// For a classical transmitter with the same mean photon number per mode:
    ///
    /// ```text
    /// SNR_CL = M * kappa^2 * N_S^2 / N_B
    /// ```
    ///
    /// Note the N_S^2 dependence, which is worse than the quantum N_S dependence
    /// when N_S << 1.
    ///
    /// # Arguments
    ///
    /// * `ns` - Mean signal photon number per mode
    /// * `nb` - Mean noise photon number per mode
    /// * `kappa` - Target reflectivity
    /// * `m` - Number of temporal modes
    pub fn classical_snr(ns: f64, nb: f64, kappa: f64, m: usize) -> f64 {
        (m as f64) * kappa * kappa * ns * ns / nb
    }

    /// Computes the quantum advantage in dB.
    ///
    /// The ratio of quantum to classical SNR gives:
    ///
    /// ```text
    /// Advantage = 10 * log10(SNR_QI / SNR_CL) = 10 * log10(4 * N_B / (kappa * N_S))
    /// ```
    ///
    /// This advantage grows with increasing N_B (noisier backgrounds) and
    /// decreasing N_S (fewer signal photons).
    ///
    /// # Arguments
    ///
    /// * `ns` - Mean signal photon number per mode
    /// * `nb` - Mean noise photon number per mode
    pub fn quantum_advantage_db(&self, ns: f64, nb: f64) -> f64 {
        let kappa = self.config.target_reflectivity;
        10.0 * (4.0 * nb / (kappa * ns)).log10()
    }

    /// Computes the quantum error probability using the Chernoff bound.
    ///
    /// For the quantum illumination receiver:
    ///
    /// ```text
    /// P_e = 0.5 * exp(-SNR / 2)
    /// ```
    ///
    /// This exponential decay represents the quantum Chernoff bound.
    ///
    /// # Arguments
    ///
    /// * `snr` - Quantum illumination SNR (linear)
    pub fn error_probability_quantum(snr: f64) -> f64 {
        0.5 * (-snr / 2.0).exp()
    }

    /// Computes the classical error probability using the complementary error function.
    ///
    /// For the classical (coherent state) receiver:
    ///
    /// ```text
    /// P_e = 0.5 * erfc(sqrt(SNR / 2))
    /// ```
    ///
    /// # Arguments
    ///
    /// * `snr` - Classical SNR (linear)
    pub fn error_probability_classical(snr: f64) -> f64 {
        0.5 * erfc((snr / 2.0).sqrt())
    }

    /// Computes the detection threshold from a desired false alarm probability.
    ///
    /// For a correlation-based detector with `num_modes` independent samples,
    /// the threshold is derived from the inverse complementary error function
    /// relationship. Under the null hypothesis (no target), the test statistic
    /// is approximately Gaussian by the CLT.
    ///
    /// # Arguments
    ///
    /// * `pfa` - Desired probability of false alarm (0 < pfa < 1)
    /// * `num_modes` - Number of independent temporal modes
    pub fn detection_threshold(pfa: f64, num_modes: usize) -> f64 {
        // Under H0, the correlation statistic ~ N(0, sigma^2/M)
        // Threshold = erfc_inv(2*pfa) * sqrt(2/M)
        // We use the approximation: erfc_inv(x) ≈ sqrt(-ln(x * sqrt(pi) * (1 - 0.147 * ln(x * sqrt(pi)))))
        // Simplified: threshold scales with sqrt(-2 * ln(pfa)) / sqrt(num_modes)
        let inv_erfc_2pfa = erfc_inv(2.0 * pfa);
        inv_erfc_2pfa * (2.0 / num_modes as f64).sqrt()
    }

    /// Generates a Receiver Operating Characteristic (ROC) curve.
    ///
    /// Returns a vector of `(P_FA, P_D)` pairs where P_FA is the probability
    /// of false alarm and P_D is the probability of detection. The curve is
    /// computed for the quantum illumination receiver.
    ///
    /// # Arguments
    ///
    /// * `snr` - Quantum SNR (linear)
    /// * `num_points` - Number of points on the ROC curve
    ///
    /// # Returns
    ///
    /// Vector of (PFA, PD) pairs, sorted by increasing PFA from 0 to 1.
    pub fn receiver_operating_characteristic(snr: f64, num_points: usize) -> Vec<(f64, f64)> {
        let mut roc = Vec::with_capacity(num_points);
        for i in 0..num_points {
            let pfa = (i as f64 + 0.5) / num_points as f64;
            // For the quantum receiver, detection probability:
            // PD = 1 - Q(Q_inv(PFA) - sqrt(SNR))
            // where Q is the Q-function. Using erfc:
            // PD = 0.5 * erfc((erfc_inv(2*PFA) - sqrt(SNR/2)) )
            let z_pfa = erfc_inv(2.0 * pfa) * std::f64::consts::SQRT_2;
            let z_shifted = z_pfa - snr.sqrt();
            let pd = 0.5 * erfc(z_shifted / std::f64::consts::SQRT_2);
            roc.push((pfa, pd));
        }
        roc
    }
}

/// Entanglement source for generating signal-idler photon pairs.
///
/// Models a two-mode squeezed vacuum (TMSV) source, which is the optimal
/// Gaussian entangled state for quantum illumination. The TMSV state has
/// correlations between signal and idler that exceed what is classically
/// possible.
pub struct EntanglementSource {
    /// Mean signal photon number per mode.
    mean_photon_number: f64,
    /// Source bandwidth in Hz.
    bandwidth_hz: f64,
}

impl EntanglementSource {
    /// Creates a new entanglement source.
    ///
    /// # Arguments
    ///
    /// * `mean_photon_number` - Mean photon number per mode (N_S)
    /// * `bandwidth_hz` - Source bandwidth in Hz
    pub fn new(mean_photon_number: f64, bandwidth_hz: f64) -> Self {
        Self {
            mean_photon_number,
            bandwidth_hz,
        }
    }

    /// Computes the two-mode squeezed vacuum state properties.
    ///
    /// For a TMSV state with mean photon number N_S per mode:
    /// - Signal variance: `<a_s^† a_s> = N_S`
    /// - Idler variance: `<a_i^† a_i> = N_S`
    /// - Cross-correlation: `|<a_s a_i>| = sqrt(N_S * (N_S + 1))`
    ///
    /// The cross-correlation exceeds the geometric mean of individual variances,
    /// which is the entanglement signature.
    ///
    /// # Arguments
    ///
    /// * `ns` - Mean photon number per mode
    ///
    /// # Returns
    ///
    /// Tuple of (signal_variance, idler_variance, cross_correlation)
    pub fn two_mode_squeezed_vacuum(ns: f64) -> (f64, f64, f64) {
        let signal_variance = ns;
        let idler_variance = ns;
        let cross_correlation = (ns * (ns + 1.0)).sqrt();
        (signal_variance, idler_variance, cross_correlation)
    }

    /// Computes the entangled photon pair generation rate.
    ///
    /// The pair rate depends on pump power and the parametric down-conversion
    /// or four-wave mixing efficiency.
    ///
    /// # Arguments
    ///
    /// * `pump_power_w` - Pump laser power in watts
    /// * `efficiency` - Down-conversion efficiency (pairs per joule)
    ///
    /// # Returns
    ///
    /// Pair generation rate in pairs per second.
    pub fn photon_pair_rate(pump_power_w: f64, efficiency: f64) -> f64 {
        pump_power_w * efficiency
    }

    /// Computes the von Neumann entropy of the entangled state.
    ///
    /// For a TMSV state with mean photon number N_S, the entropy of the
    /// reduced single-mode state is:
    ///
    /// ```text
    /// S = (N_S + 1) * ln(N_S + 1) - N_S * ln(N_S)
    /// ```
    ///
    /// This is the thermal state entropy, measuring the entanglement between
    /// signal and idler modes.
    ///
    /// # Arguments
    ///
    /// * `ns` - Mean photon number per mode
    ///
    /// # Returns
    ///
    /// Von Neumann entropy in nats. Returns 0 for ns = 0 (vacuum state).
    pub fn entanglement_entropy(ns: f64) -> f64 {
        if ns <= 0.0 {
            return 0.0;
        }
        (ns + 1.0) * (ns + 1.0).ln() - ns * ns.ln()
    }

    /// Computes the quantum discord for the signal-target-idler system.
    ///
    /// Quantum discord measures quantum correlations beyond entanglement.
    /// After the signal passes through the lossy, noisy channel (reflectivity
    /// kappa, noise N_B), the discord between the returned mode and the idler is:
    ///
    /// ```text
    /// D ≈ kappa * N_S / N_B  (for N_S << 1, N_B >> 1)
    /// ```
    ///
    /// This residual quantum discord enables the quantum advantage even though
    /// entanglement is destroyed.
    ///
    /// # Arguments
    ///
    /// * `ns` - Mean signal photon number per mode
    /// * `nb` - Mean noise photon number per mode
    /// * `kappa` - Target reflectivity
    ///
    /// # Returns
    ///
    /// Approximate quantum discord (dimensionless).
    pub fn quantum_discord(ns: f64, nb: f64, kappa: f64) -> f64 {
        kappa * ns / nb
    }

    /// Computes the coincidence time window.
    ///
    /// The temporal window for detecting correlated photon pairs is inversely
    /// proportional to the source bandwidth:
    ///
    /// ```text
    /// tau_c = 1 / bandwidth
    /// ```
    ///
    /// # Arguments
    ///
    /// * `bandwidth_hz` - Source bandwidth in Hz
    ///
    /// # Returns
    ///
    /// Coincidence window in seconds.
    pub fn coincidence_window_s(bandwidth_hz: f64) -> f64 {
        1.0 / bandwidth_hz
    }

    /// Returns the configured mean photon number.
    pub fn mean_photon_number(&self) -> f64 {
        self.mean_photon_number
    }

    /// Returns the configured bandwidth in Hz.
    pub fn bandwidth_hz(&self) -> f64 {
        self.bandwidth_hz
    }
}

/// Correlation receiver for quantum illumination radar.
///
/// Implements various receiver architectures for extracting target information
/// from the returned signal by correlating with the locally stored idler.
pub struct CorrelationReceiver {
    /// Number of temporal modes to process.
    num_modes: usize,
}

impl CorrelationReceiver {
    /// Creates a new correlation receiver.
    ///
    /// # Arguments
    ///
    /// * `num_modes` - Number of temporal modes to process
    pub fn new(num_modes: usize) -> Self {
        Self { num_modes }
    }

    /// Computes the cross-correlation between returned signal and stored idler.
    ///
    /// This is the simplest quantum measurement: element-wise multiplication
    /// and summation. The result is compared against a threshold for detection.
    ///
    /// # Arguments
    ///
    /// * `signal_return` - Returned signal samples (real-valued)
    /// * `idler_stored` - Locally stored idler samples (real-valued)
    ///
    /// # Returns
    ///
    /// Cross-correlation value (sum of element-wise products).
    pub fn cross_correlate_quantum(signal_return: &[f64], idler_stored: &[f64]) -> f64 {
        let n = signal_return.len().min(idler_stored.len());
        signal_return[..n]
            .iter()
            .zip(&idler_stored[..n])
            .map(|(s, i)| s * i)
            .sum()
    }

    /// Models an optical parametric amplifier (OPA) receiver.
    ///
    /// The OPA receiver amplifies the returned signal conditioned on the idler,
    /// then performs photon counting. This is the near-optimal receiver for
    /// quantum illumination proposed by Guha & Erkmen (2009).
    ///
    /// The OPA combines signal and idler quadratures:
    /// ```text
    /// output = gain * (signal_I * idler_I + signal_Q * idler_Q)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `signal` - Returned signal as (I, Q) pairs
    /// * `idler` - Stored idler as (I, Q) pairs
    /// * `gain` - OPA parametric gain
    ///
    /// # Returns
    ///
    /// OPA output samples (one per mode).
    pub fn opa_receiver(
        signal: &[(f64, f64)],
        idler: &[(f64, f64)],
        gain: f64,
    ) -> Vec<f64> {
        let n = signal.len().min(idler.len());
        (0..n)
            .map(|i| {
                let (si, sq) = signal[i];
                let (ii, iq) = idler[i];
                gain * (si * ii + sq * iq)
            })
            .collect()
    }

    /// Models a phase-conjugate (PC) receiver.
    ///
    /// The PC receiver applies phase conjugation to the idler before
    /// interfering with the returned signal. This extracts the quantum
    /// correlations optimally in the low-brightness regime.
    ///
    /// ```text
    /// output = signal_I * idler_I + signal_Q * (-idler_Q)  [conjugate Q]
    /// ```
    ///
    /// # Arguments
    ///
    /// * `signal` - Returned signal as (I, Q) pairs
    /// * `idler` - Stored idler as (I, Q) pairs
    ///
    /// # Returns
    ///
    /// PC receiver output samples.
    pub fn phase_conjugate_receiver(
        signal: &[(f64, f64)],
        idler: &[(f64, f64)],
    ) -> Vec<f64> {
        let n = signal.len().min(idler.len());
        (0..n)
            .map(|i| {
                let (si, sq) = signal[i];
                let (ii, iq) = idler[i];
                // Phase conjugation: conjugate the idler (negate Q component)
                si * ii + sq * (-iq)
            })
            .collect()
    }

    /// Models a sum-frequency generation (SFG) receiver.
    ///
    /// The SFG process converts correlated signal-idler photon pairs into
    /// higher-frequency photons that can be detected with single-photon
    /// counters. The output is the total converted photon count.
    ///
    /// # Arguments
    ///
    /// * `signal` - Returned signal samples (real-valued amplitudes)
    /// * `idler` - Stored idler samples (real-valued amplitudes)
    ///
    /// # Returns
    ///
    /// SFG output proportional to the correlation.
    pub fn sum_frequency_receiver(signal: &[f64], idler: &[f64]) -> f64 {
        let n = signal.len().min(idler.len());
        signal[..n]
            .iter()
            .zip(&idler[..n])
            .map(|(s, i)| s * i)
            .sum::<f64>()
            / n.max(1) as f64
    }

    /// Makes a binary detection decision based on threshold comparison.
    ///
    /// # Arguments
    ///
    /// * `correlation` - Measured correlation value
    /// * `threshold` - Detection threshold
    ///
    /// # Returns
    ///
    /// `true` if target is declared present (correlation >= threshold).
    pub fn threshold_decision(correlation: f64, threshold: f64) -> bool {
        correlation >= threshold
    }

    /// Returns the number of modes this receiver is configured for.
    pub fn num_modes(&self) -> usize {
        self.num_modes
    }
}

/// Range estimation using time-bin correlations.
///
/// Provides methods for estimating target range from round-trip delay
/// measurements and computing range resolution limits.
pub struct RangeEstimator;

impl RangeEstimator {
    /// Computes target range from round-trip delay.
    ///
    /// ```text
    /// R = c * t / 2
    /// ```
    ///
    /// # Arguments
    ///
    /// * `delay_ns` - Round-trip delay in nanoseconds
    ///
    /// # Returns
    ///
    /// Target range in meters.
    pub fn range_from_delay(delay_ns: f64) -> f64 {
        SPEED_OF_LIGHT * delay_ns * 1e-9 / 2.0
    }

    /// Computes the range resolution determined by bandwidth.
    ///
    /// ```text
    /// dR = c / (2 * B)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `bandwidth_hz` - Signal bandwidth in Hz
    ///
    /// # Returns
    ///
    /// Range resolution in meters.
    pub fn range_resolution(bandwidth_hz: f64) -> f64 {
        SPEED_OF_LIGHT / (2.0 * bandwidth_hz)
    }

    /// Computes the quantum-limited range precision (Cramer-Rao lower bound).
    ///
    /// The minimum achievable standard deviation of the range estimate is:
    ///
    /// ```text
    /// sigma_R = c / (2 * B * sqrt(2 * SNR))
    /// ```
    ///
    /// # Arguments
    ///
    /// * `snr` - Signal-to-noise ratio (linear)
    /// * `bandwidth_hz` - Signal bandwidth in Hz
    ///
    /// # Returns
    ///
    /// Cramer-Rao lower bound on range precision in meters.
    pub fn quantum_range_precision(snr: f64, bandwidth_hz: f64) -> f64 {
        SPEED_OF_LIGHT / (2.0 * bandwidth_hz * (2.0 * snr).sqrt())
    }

    /// Performs a delay scan to find the peak correlation.
    ///
    /// Cross-correlates the signal and idler at multiple delay offsets
    /// to estimate the round-trip time. The delay with the highest
    /// correlation indicates the target range.
    ///
    /// # Arguments
    ///
    /// * `signal` - Returned signal samples
    /// * `idler` - Stored idler samples
    /// * `max_delay_bins` - Maximum number of delay bins to search (in both directions)
    ///
    /// # Returns
    ///
    /// Vector of (delay_bin, correlation) pairs sorted by delay.
    pub fn time_bin_correlation(
        signal: &[f64],
        idler: &[f64],
        max_delay_bins: usize,
    ) -> Vec<(i64, f64)> {
        let n = signal.len().min(idler.len());
        if n == 0 {
            return vec![];
        }
        let max_delay = max_delay_bins.min(n - 1);
        let mut results = Vec::with_capacity(2 * max_delay + 1);

        for delay in -(max_delay as i64)..=(max_delay as i64) {
            let mut corr = 0.0;
            let mut count = 0usize;
            for j in 0..n {
                let k = j as i64 + delay;
                if k >= 0 && (k as usize) < n {
                    corr += signal[j] * idler[k as usize];
                    count += 1;
                }
            }
            if count > 0 {
                corr /= count as f64;
            }
            results.push((delay, corr));
        }
        results
    }
}

/// Performance analyzer for comparing quantum and classical radar.
///
/// Provides methods for computing performance metrics, required resources,
/// and operational limits for quantum illumination radar systems.
pub struct PerformanceAnalyzer;

impl PerformanceAnalyzer {
    /// Compares quantum and classical detection performance.
    ///
    /// # Arguments
    ///
    /// * `config` - Quantum radar configuration
    ///
    /// # Returns
    ///
    /// Tuple of (snr_qi, snr_cl, advantage_db):
    /// - `snr_qi`: Quantum illumination SNR
    /// - `snr_cl`: Classical (coherent state) SNR
    /// - `advantage_db`: Quantum advantage in dB
    pub fn compare_quantum_classical(config: &QuantumRadarConfig) -> (f64, f64, f64) {
        let ns = config.mean_photon_number;
        let nb = config.noise_photon_number;
        let kappa = config.target_reflectivity;
        let m = config.num_signal_modes;

        let snr_qi = QuantumIlluminationProcessor::quantum_snr(ns, nb, kappa, m);
        let snr_cl = QuantumIlluminationProcessor::classical_snr(ns, nb, kappa, m);

        let advantage_db = if snr_cl > 0.0 {
            10.0 * (snr_qi / snr_cl).log10()
        } else {
            f64::INFINITY
        };

        (snr_qi, snr_cl, advantage_db)
    }

    /// Computes the required integration time to achieve a target SNR.
    ///
    /// Since SNR_QI scales linearly with M, and M is proportional to
    /// integration time * bandwidth, we can solve:
    ///
    /// ```text
    /// T = target_snr * N_B / (4 * kappa * N_S * bandwidth)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `target_snr` - Desired quantum SNR
    /// * `config` - Quantum radar configuration
    ///
    /// # Returns
    ///
    /// Required integration time in seconds.
    pub fn required_integration_time(target_snr: f64, config: &QuantumRadarConfig) -> f64 {
        let ns = config.mean_photon_number;
        let nb = config.noise_photon_number;
        let kappa = config.target_reflectivity;
        let bandwidth_hz = config.bandwidth_mhz * 1e6;

        // SNR_QI = 4 * M * kappa * ns / nb, where M = T * B
        // T = target_snr * nb / (4 * kappa * ns * B)
        target_snr * nb / (4.0 * kappa * ns * bandwidth_hz)
    }

    /// Computes the maximum detectable range for a given minimum SNR.
    ///
    /// As range increases, the received signal power decreases (inversely
    /// with range^4 for radar). This method finds the range at which the
    /// quantum SNR equals the minimum required SNR.
    ///
    /// The free-space path loss factor is modeled as:
    ///
    /// ```text
    /// kappa_eff(R) = kappa_0 * (R_0 / R)^4
    /// ```
    ///
    /// # Arguments
    ///
    /// * `min_snr` - Minimum required SNR for detection
    /// * `config` - Quantum radar configuration
    ///
    /// # Returns
    ///
    /// Maximum range in meters.
    pub fn max_range(min_snr: f64, config: &QuantumRadarConfig) -> f64 {
        let ns = config.mean_photon_number;
        let nb = config.noise_photon_number;
        let kappa = config.target_reflectivity;
        let m = config.num_signal_modes;
        let r0 = config.range_m;

        // SNR at reference range: SNR_0 = 4*M*kappa*ns/nb
        let snr_0 = QuantumIlluminationProcessor::quantum_snr(ns, nb, kappa, m);

        // SNR scales as (R0/R)^4, so R_max = R0 * (SNR_0/min_snr)^(1/4)
        if min_snr <= 0.0 {
            return f64::INFINITY;
        }
        r0 * (snr_0 / min_snr).powf(0.25)
    }

    /// Computes the number of modes needed to achieve a target SNR.
    ///
    /// ```text
    /// M = target_snr * N_B / (4 * kappa * N_S)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `target_snr` - Desired quantum SNR
    /// * `ns` - Mean signal photon number per mode
    /// * `nb` - Mean noise photon number per mode
    /// * `kappa` - Target reflectivity
    ///
    /// # Returns
    ///
    /// Required number of temporal modes (rounded up).
    pub fn mode_count_for_snr(target_snr: f64, ns: f64, nb: f64, kappa: f64) -> usize {
        let m = target_snr * nb / (4.0 * kappa * ns);
        m.ceil() as usize
    }

    /// Computes the noise-equivalent reflectivity (minimum detectable reflectivity).
    ///
    /// This is the smallest kappa for which the quantum SNR reaches the threshold:
    ///
    /// ```text
    /// kappa_min = snr_threshold * N_B / (4 * M * N_S)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `snr_threshold` - Minimum SNR for detection
    /// * `ns` - Mean signal photon number per mode
    /// * `nb` - Mean noise photon number per mode
    /// * `m` - Number of temporal modes
    ///
    /// # Returns
    ///
    /// Minimum detectable reflectivity (0-1 range, may exceed 1 if infeasible).
    pub fn noise_equivalent_reflectivity(snr_threshold: f64, ns: f64, nb: f64, m: usize) -> f64 {
        snr_threshold * nb / (4.0 * (m as f64) * ns)
    }
}

// ---------- Helper math functions ----------

/// Complementary error function approximation.
///
/// Uses the Abramowitz and Stegun approximation (formula 7.1.26) with
/// maximum error |epsilon(x)| < 1.5e-7.
fn erfc(x: f64) -> f64 {
    if x >= 0.0 {
        erfc_positive(x)
    } else {
        2.0 - erfc_positive(-x)
    }
}

/// erfc for x >= 0 using rational approximation.
fn erfc_positive(x: f64) -> f64 {
    // Abramowitz & Stegun 7.1.26
    let p = 0.3275911;
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;

    let t = 1.0 / (1.0 + p * x);
    let poly = t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))));
    poly * (-x * x).exp()
}

/// Inverse complementary error function approximation.
///
/// Uses a rational approximation suitable for 0 < x < 2.
fn erfc_inv(x: f64) -> f64 {
    if x <= 0.0 {
        return f64::INFINITY;
    }
    if x >= 2.0 {
        return f64::NEG_INFINITY;
    }
    if (x - 1.0).abs() < 1e-15 {
        return 0.0;
    }

    // Use Newton's method on erfc(y) = x starting from an initial approximation
    // Initial guess from rational approximation
    let p = if x < 1.0 { x } else { 2.0 - x };
    let sign = if x < 1.0 { 1.0 } else { -1.0 };

    // Approximation: erfc_inv(p) ≈ sqrt(-ln(p * sqrt(pi)))  for small p
    let t = (-((p / 2.0).ln())).sqrt();

    // Rational approximation for the initial guess
    let c0 = 2.515517;
    let c1 = 0.802853;
    let c2 = 0.010328;
    let d1 = 1.432788;
    let d2 = 0.189269;
    let d3 = 0.001308;

    let mut y = t - (c0 + c1 * t + c2 * t * t) / (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);

    // Newton refinement: y_{n+1} = y_n + (erfc(y_n) - p) * sqrt(pi) * exp(y_n^2) / 2
    for _ in 0..3 {
        let err = erfc_positive(y) - p;
        let deriv = -2.0 / PI.sqrt() * (-y * y).exp();
        if deriv.abs() > 1e-300 {
            y -= err / deriv;
        }
    }

    sign * y
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;
    const APPROX_EPSILON: f64 = 1e-3;

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    fn relative_eq(a: f64, b: f64, rel_eps: f64) -> bool {
        if a == b {
            return true;
        }
        let denom = a.abs().max(b.abs());
        if denom < 1e-15 {
            return (a - b).abs() < 1e-15;
        }
        (a - b).abs() / denom < rel_eps
    }

    // --- QuantumRadarConfig tests ---

    #[test]
    fn test_config_new() {
        let config = QuantumRadarConfig::new(5.0, 200.0, 5000, 0.05, 50.0, 0.2, 0.001, 200.0);
        assert!((config.signal_frequency_ghz - 5.0).abs() < EPSILON);
        assert!((config.bandwidth_mhz - 200.0).abs() < EPSILON);
        assert_eq!(config.num_signal_modes, 5000);
        assert!((config.mean_photon_number - 0.05).abs() < EPSILON);
        assert!((config.noise_photon_number - 50.0).abs() < EPSILON);
        assert!((config.target_reflectivity - 0.2).abs() < EPSILON);
        assert!((config.integration_time_s - 0.001).abs() < EPSILON);
        assert!((config.range_m - 200.0).abs() < EPSILON);
    }

    #[test]
    fn test_config_default_microwave() {
        let config = QuantumRadarConfig::default_microwave();
        assert!((config.signal_frequency_ghz - 3.0).abs() < EPSILON);
        assert!((config.mean_photon_number - 0.01).abs() < EPSILON);
        assert!((config.noise_photon_number - 100.0).abs() < EPSILON);
    }

    // --- Quantum SNR tests ---

    #[test]
    fn test_quantum_snr_formula() {
        // SNR_QI = 4 * M * kappa * N_S / N_B
        let snr = QuantumIlluminationProcessor::quantum_snr(0.01, 100.0, 0.1, 10_000);
        let expected = 4.0 * 10_000.0 * 0.1 * 0.01 / 100.0;
        assert!((snr - expected).abs() < EPSILON);
    }

    #[test]
    fn test_quantum_snr_linear_with_modes() {
        // SNR should scale linearly with M
        let snr1 = QuantumIlluminationProcessor::quantum_snr(0.01, 100.0, 0.1, 1000);
        let snr2 = QuantumIlluminationProcessor::quantum_snr(0.01, 100.0, 0.1, 2000);
        assert!(relative_eq(snr2 / snr1, 2.0, 1e-10));
    }

    #[test]
    fn test_quantum_snr_linear_with_ns() {
        // SNR scales linearly with N_S
        let snr1 = QuantumIlluminationProcessor::quantum_snr(0.01, 100.0, 0.1, 10_000);
        let snr2 = QuantumIlluminationProcessor::quantum_snr(0.02, 100.0, 0.1, 10_000);
        assert!(relative_eq(snr2 / snr1, 2.0, 1e-10));
    }

    // --- Classical SNR tests ---

    #[test]
    fn test_classical_snr_formula() {
        // SNR_CL = M * kappa^2 * N_S^2 / N_B
        let snr = QuantumIlluminationProcessor::classical_snr(0.01, 100.0, 0.1, 10_000);
        let expected = 10_000.0 * 0.1 * 0.1 * 0.01 * 0.01 / 100.0;
        assert!((snr - expected).abs() < EPSILON);
    }

    #[test]
    fn test_classical_snr_quadratic_with_ns() {
        // Classical SNR scales with N_S^2 (worse for low N_S)
        let snr1 = QuantumIlluminationProcessor::classical_snr(0.01, 100.0, 0.1, 10_000);
        let snr2 = QuantumIlluminationProcessor::classical_snr(0.02, 100.0, 0.1, 10_000);
        assert!(relative_eq(snr2 / snr1, 4.0, 1e-10));
    }

    // --- Quantum advantage tests ---

    #[test]
    fn test_quantum_advantage_specific_case() {
        // When N_B / N_S = kappa, advantage = 10*log10(4) ≈ 6.02 dB
        // Set kappa=0.1, N_S=0.1, N_B=0.01 => N_B/(kappa*N_S) = 0.01/(0.1*0.1) = 1
        // Advantage = 10*log10(4*1) = 10*log10(4) ≈ 6.02
        let config = QuantumRadarConfig::new(3.0, 100.0, 10_000, 0.1, 0.01, 0.1, 1e-3, 100.0);
        let proc = QuantumIlluminationProcessor::new(config);
        let adv = proc.quantum_advantage_db(0.1, 0.01);
        assert!(approx_eq(adv, 10.0 * 4.0_f64.log10(), 0.01));
    }

    #[test]
    fn test_quantum_advantage_increases_with_nb() {
        // Noisier backgrounds give bigger quantum advantage
        let config = QuantumRadarConfig::new(3.0, 100.0, 10_000, 0.01, 100.0, 0.1, 1e-3, 100.0);
        let proc = QuantumIlluminationProcessor::new(config);
        let adv_low_nb = proc.quantum_advantage_db(0.01, 10.0);
        let adv_high_nb = proc.quantum_advantage_db(0.01, 100.0);
        assert!(adv_high_nb > adv_low_nb);
    }

    #[test]
    fn test_quantum_advantage_increases_with_lower_ns() {
        // Lower signal photon number gives bigger quantum advantage
        let config = QuantumRadarConfig::new(3.0, 100.0, 10_000, 0.01, 100.0, 0.1, 1e-3, 100.0);
        let proc = QuantumIlluminationProcessor::new(config);
        let adv_high_ns = proc.quantum_advantage_db(0.1, 100.0);
        let adv_low_ns = proc.quantum_advantage_db(0.01, 100.0);
        assert!(adv_low_ns > adv_high_ns);
    }

    // --- Error probability tests ---

    #[test]
    fn test_quantum_error_probability_decreases_with_snr() {
        let pe1 = QuantumIlluminationProcessor::error_probability_quantum(1.0);
        let pe2 = QuantumIlluminationProcessor::error_probability_quantum(5.0);
        assert!(pe2 < pe1);
    }

    #[test]
    fn test_quantum_error_probability_at_zero_snr() {
        let pe = QuantumIlluminationProcessor::error_probability_quantum(0.0);
        assert!(approx_eq(pe, 0.5, EPSILON));
    }

    #[test]
    fn test_classical_error_probability_decreases_with_snr() {
        let pe1 = QuantumIlluminationProcessor::error_probability_classical(1.0);
        let pe2 = QuantumIlluminationProcessor::error_probability_classical(5.0);
        assert!(pe2 < pe1);
    }

    #[test]
    fn test_classical_error_probability_at_zero_snr() {
        let pe = QuantumIlluminationProcessor::error_probability_classical(0.0);
        // erfc(0) = 1, so Pe = 0.5 * 1 = 0.5
        assert!(approx_eq(pe, 0.5, APPROX_EPSILON));
    }

    // --- TMSV state tests ---

    #[test]
    fn test_tmsv_cross_correlation_exceeds_individual() {
        // Entanglement signature: cross_corr > sqrt(signal_var * idler_var)
        // cross_corr = sqrt(N_S * (N_S + 1)) > N_S = sqrt(N_S * N_S)
        let ns = 0.5;
        let (sv, iv, cc) = EntanglementSource::two_mode_squeezed_vacuum(ns);
        assert!(cc > (sv * iv).sqrt());
    }

    #[test]
    fn test_tmsv_symmetric_variances() {
        let (sv, iv, _) = EntanglementSource::two_mode_squeezed_vacuum(0.3);
        assert!((sv - iv).abs() < EPSILON);
    }

    #[test]
    fn test_tmsv_vacuum_state() {
        // N_S = 0: vacuum state, no correlations
        let (sv, iv, cc) = EntanglementSource::two_mode_squeezed_vacuum(0.0);
        assert!(sv.abs() < EPSILON);
        assert!(iv.abs() < EPSILON);
        assert!(cc.abs() < EPSILON);
    }

    // --- Entanglement entropy tests ---

    #[test]
    fn test_entropy_zero_for_vacuum() {
        // No entanglement in vacuum state
        let s = EntanglementSource::entanglement_entropy(0.0);
        assert!(s.abs() < EPSILON);
    }

    #[test]
    fn test_entropy_positive_for_nonzero_ns() {
        let s = EntanglementSource::entanglement_entropy(1.0);
        assert!(s > 0.0);
    }

    #[test]
    fn test_entropy_increases_with_ns() {
        let s1 = EntanglementSource::entanglement_entropy(0.1);
        let s2 = EntanglementSource::entanglement_entropy(1.0);
        let s3 = EntanglementSource::entanglement_entropy(10.0);
        assert!(s2 > s1);
        assert!(s3 > s2);
    }

    // --- Photon pair rate test ---

    #[test]
    fn test_photon_pair_rate() {
        let rate = EntanglementSource::photon_pair_rate(0.1, 1e10);
        assert!(approx_eq(rate, 1e9, EPSILON));
    }

    // --- Quantum discord test ---

    #[test]
    fn test_quantum_discord() {
        let d = EntanglementSource::quantum_discord(0.01, 100.0, 0.5);
        assert!(approx_eq(d, 0.5 * 0.01 / 100.0, EPSILON));
    }

    // --- Coincidence window test ---

    #[test]
    fn test_coincidence_window_inversely_proportional() {
        let t1 = EntanglementSource::coincidence_window_s(1e9);
        let t2 = EntanglementSource::coincidence_window_s(2e9);
        assert!(relative_eq(t1 / t2, 2.0, 1e-10));
    }

    // --- Correlation receiver tests ---

    #[test]
    fn test_cross_correlate_quantum() {
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let idler = vec![1.0, 1.0, 1.0, 1.0];
        let corr = CorrelationReceiver::cross_correlate_quantum(&signal, &idler);
        assert!(approx_eq(corr, 10.0, EPSILON));
    }

    #[test]
    fn test_opa_receiver() {
        let receiver = CorrelationReceiver::new(3);
        let signal = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let idler = vec![(1.0, 0.0), (1.0, 0.0), (1.0, 1.0)];
        let gain = 2.0;
        let out = CorrelationReceiver::opa_receiver(&signal, &idler, gain);
        assert_eq!(out.len(), 3);
        // (1*1 + 0*0) * 2 = 2.0
        assert!(approx_eq(out[0], 2.0, EPSILON));
        // (0*1 + 1*0) * 2 = 0.0
        assert!(approx_eq(out[1], 0.0, EPSILON));
        // (1*1 + 1*1) * 2 = 4.0
        assert!(approx_eq(out[2], 4.0, EPSILON));
    }

    #[test]
    fn test_phase_conjugate_receiver_correlated() {
        // With correlated signal and idler, PC output should be non-zero
        let signal = vec![(1.0, 0.5), (0.5, 1.0)];
        let idler = vec![(1.0, 0.5), (0.5, 1.0)];
        let out = CorrelationReceiver::phase_conjugate_receiver(&signal, &idler);
        assert_eq!(out.len(), 2);
        // (1*1 + 0.5*(-0.5)) = 1.0 - 0.25 = 0.75
        assert!(approx_eq(out[0], 0.75, EPSILON));
    }

    #[test]
    fn test_phase_conjugate_receiver_depends_on_correlation() {
        // Uncorrelated: random-ish outputs
        let signal_corr = vec![(1.0, 0.0); 10];
        let idler_corr = vec![(1.0, 0.0); 10];
        let out_corr = CorrelationReceiver::phase_conjugate_receiver(&signal_corr, &idler_corr);

        let signal_uncorr = vec![(1.0, 0.0); 10];
        let idler_uncorr = vec![(0.0, 1.0); 10];
        let out_uncorr =
            CorrelationReceiver::phase_conjugate_receiver(&signal_uncorr, &idler_uncorr);

        let sum_corr: f64 = out_corr.iter().sum();
        let sum_uncorr: f64 = out_uncorr.iter().sum();
        // Correlated signals should give larger total output
        assert!(sum_corr > sum_uncorr);
    }

    #[test]
    fn test_sfg_receiver() {
        let signal = vec![1.0, 2.0, 3.0];
        let idler = vec![1.0, 1.0, 1.0];
        let out = CorrelationReceiver::sum_frequency_receiver(&signal, &idler);
        // (1+2+3)/3 = 2.0
        assert!(approx_eq(out, 2.0, EPSILON));
    }

    #[test]
    fn test_threshold_decision() {
        assert!(CorrelationReceiver::threshold_decision(1.5, 1.0));
        assert!(!CorrelationReceiver::threshold_decision(0.5, 1.0));
        assert!(CorrelationReceiver::threshold_decision(1.0, 1.0));
    }

    // --- Range estimator tests ---

    #[test]
    fn test_range_from_delay_roundtrip() {
        // 1 microsecond roundtrip = c * 1e-6 / 2 ≈ 149.896 m
        let range = RangeEstimator::range_from_delay(1000.0); // 1000 ns
        let expected = SPEED_OF_LIGHT * 1e-6 / 2.0;
        assert!(relative_eq(range, expected, 1e-6));
    }

    #[test]
    fn test_range_resolution() {
        // dR = c / (2 * B)
        let dr = RangeEstimator::range_resolution(100e6); // 100 MHz
        let expected = SPEED_OF_LIGHT / (2.0 * 100e6);
        assert!(relative_eq(dr, expected, 1e-10));
    }

    #[test]
    fn test_range_resolution_is_c_over_2b() {
        let bw = 200e6;
        let dr = RangeEstimator::range_resolution(bw);
        assert!(relative_eq(dr, SPEED_OF_LIGHT / (2.0 * bw), 1e-10));
    }

    #[test]
    fn test_quantum_range_precision() {
        let precision = RangeEstimator::quantum_range_precision(10.0, 100e6);
        assert!(precision > 0.0);
        assert!(precision < RangeEstimator::range_resolution(100e6));
    }

    #[test]
    fn test_time_bin_correlation_peak() {
        // Idler has a sharp peak at index 5, signal has the same peak at index 2
        // So the signal leads the idler by 3 bins.
        // Correlation: sum of signal[j] * idler[j + delay]
        // Peak at delay = 3 because signal[2] * idler[2+3=5] aligns the peaks.
        let signal = vec![0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let idler = vec![0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0];
        let result = RangeEstimator::time_bin_correlation(&signal, &idler, 6);

        // Find the peak
        let peak = result
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        assert_eq!(peak.0, 3);
    }

    // --- Detection threshold tests ---

    #[test]
    fn test_detection_threshold_increases_with_lower_pfa() {
        let t1 = QuantumIlluminationProcessor::detection_threshold(0.1, 1000);
        let t2 = QuantumIlluminationProcessor::detection_threshold(0.01, 1000);
        let t3 = QuantumIlluminationProcessor::detection_threshold(0.001, 1000);
        assert!(t2 > t1);
        assert!(t3 > t2);
    }

    #[test]
    fn test_detection_threshold_positive() {
        let t = QuantumIlluminationProcessor::detection_threshold(0.01, 1000);
        assert!(t > 0.0);
    }

    // --- ROC curve tests ---

    #[test]
    fn test_roc_curve_pd_increases_with_pfa() {
        let roc = QuantumIlluminationProcessor::receiver_operating_characteristic(5.0, 100);
        assert_eq!(roc.len(), 100);

        // PD should generally increase with PFA
        // Check first and last points
        let (pfa_first, pd_first) = roc[0];
        let (pfa_last, pd_last) = roc[roc.len() - 1];
        assert!(pfa_last > pfa_first);
        assert!(pd_last > pd_first);
    }

    #[test]
    fn test_roc_curve_higher_snr_better() {
        let roc_low = QuantumIlluminationProcessor::receiver_operating_characteristic(1.0, 50);
        let roc_high = QuantumIlluminationProcessor::receiver_operating_characteristic(10.0, 50);

        // At same PFA, higher SNR should give higher PD
        let pd_low_mid = roc_low[25].1;
        let pd_high_mid = roc_high[25].1;
        assert!(pd_high_mid > pd_low_mid);
    }

    // --- Performance analyzer tests ---

    #[test]
    fn test_compare_quantum_classical() {
        let config = QuantumRadarConfig::default_microwave();
        let (snr_qi, snr_cl, advantage) = PerformanceAnalyzer::compare_quantum_classical(&config);

        // Quantum should beat classical
        assert!(snr_qi > snr_cl);
        assert!(advantage > 0.0);
    }

    #[test]
    fn test_required_integration_time() {
        let config = QuantumRadarConfig::default_microwave();
        let t = PerformanceAnalyzer::required_integration_time(10.0, &config);
        assert!(t > 0.0);
    }

    #[test]
    fn test_max_range_decreases_with_higher_snr() {
        let config = QuantumRadarConfig::default_microwave();
        let r1 = PerformanceAnalyzer::max_range(1.0, &config);
        let r2 = PerformanceAnalyzer::max_range(10.0, &config);
        assert!(r1 > r2);
    }

    #[test]
    fn test_max_range_positive() {
        let config = QuantumRadarConfig::default_microwave();
        let r = PerformanceAnalyzer::max_range(1.0, &config);
        assert!(r > 0.0);
    }

    #[test]
    fn test_mode_count_for_snr() {
        let m = PerformanceAnalyzer::mode_count_for_snr(10.0, 0.01, 100.0, 0.1);
        // M = 10.0 * 100.0 / (4 * 0.1 * 0.01) = 250_000
        assert_eq!(m, 250_000);
    }

    #[test]
    fn test_noise_equivalent_reflectivity() {
        let kappa_min = PerformanceAnalyzer::noise_equivalent_reflectivity(1.0, 0.01, 100.0, 10_000);
        // kappa_min = 1.0 * 100.0 / (4 * 10000 * 0.01) = 0.25
        assert!(relative_eq(kappa_min, 0.25, 1e-10));
    }

    // --- Constants tests ---

    #[test]
    fn test_speed_of_light() {
        assert!(relative_eq(SPEED_OF_LIGHT, 299_792_458.0, 1e-10));
    }

    #[test]
    fn test_planck_constant() {
        assert!(PLANCK_CONSTANT > 6.6e-34 && PLANCK_CONSTANT < 6.7e-34);
    }

    #[test]
    fn test_boltzmann_constant() {
        assert!(BOLTZMANN_CONSTANT > 1.38e-23 && BOLTZMANN_CONSTANT < 1.39e-23);
    }

    // --- Helper function tests ---

    #[test]
    fn test_erfc_at_zero() {
        assert!(approx_eq(erfc(0.0), 1.0, 1e-6));
    }

    #[test]
    fn test_erfc_large_positive() {
        assert!(erfc(5.0) < 1e-10);
    }

    #[test]
    fn test_erfc_negative() {
        // erfc(-x) = 2 - erfc(x)
        let val = erfc(-1.0);
        assert!(val > 1.0 && val < 2.0);
    }

    #[test]
    fn test_erfc_inv_roundtrip() {
        for x in &[0.1, 0.5, 1.0, 1.5, 1.9] {
            let y = erfc_inv(*x);
            let x_back = erfc(y);
            assert!(
                approx_eq(x_back, *x, 1e-4),
                "erfc_inv roundtrip failed for x={}: got {}, expected {}",
                x,
                x_back,
                x
            );
        }
    }

    // --- EntanglementSource struct tests ---

    #[test]
    fn test_entanglement_source_new() {
        let src = EntanglementSource::new(0.05, 1e9);
        assert!(approx_eq(src.mean_photon_number(), 0.05, EPSILON));
        assert!(approx_eq(src.bandwidth_hz(), 1e9, EPSILON));
    }

    // --- Edge case tests ---

    #[test]
    fn test_quantum_snr_zero_modes() {
        let snr = QuantumIlluminationProcessor::quantum_snr(0.01, 100.0, 0.1, 0);
        assert!(snr.abs() < EPSILON);
    }

    #[test]
    fn test_classical_snr_zero_ns() {
        let snr = QuantumIlluminationProcessor::classical_snr(0.0, 100.0, 0.1, 10_000);
        assert!(snr.abs() < EPSILON);
    }

    #[test]
    fn test_cross_correlate_empty() {
        let corr = CorrelationReceiver::cross_correlate_quantum(&[], &[]);
        assert!(corr.abs() < EPSILON);
    }

    #[test]
    fn test_time_bin_correlation_empty() {
        let result = RangeEstimator::time_bin_correlation(&[], &[], 5);
        assert!(result.is_empty());
    }
}
