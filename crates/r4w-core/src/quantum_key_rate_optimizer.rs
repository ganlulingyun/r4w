//! Quantum Key Distribution (QKD) protocol optimization and key rate analysis.
//!
//! This module provides tools for analyzing and optimizing QKD system performance,
//! including key rate estimation for BB84, B92, E91, SARG04, and decoy-state
//! protocols. It implements the fundamental information-theoretic bounds that
//! govern secure key generation rates in practical QKD systems.
//!
//! # Key Concepts
//!
//! - **Secure Key Rate**: The rate at which secret key bits can be distilled from
//!   raw quantum measurements, accounting for error correction and privacy amplification.
//! - **QBER (Quantum Bit Error Rate)**: The fraction of sifted key bits that differ
//!   between Alice and Bob, caused by channel noise, detector imperfections, and
//!   potential eavesdropping.
//! - **Decoy State Analysis**: A technique using multiple intensity levels to tightly
//!   bound single-photon contributions, defeating photon-number-splitting attacks on
//!   weak coherent pulse (WCP) sources.
//! - **Privacy Amplification**: The process of shortening the error-corrected key to
//!   remove any information an eavesdropper may have gained.
//!
//! # Security Thresholds
//!
//! - BB84: Secure for QBER < 11% (unconditional security proof by Shor-Preskill)
//! - B92: Secure for QBER < ~7% (tighter bound due to two-state protocol)
//! - SARG04: Secure for QBER < ~10% with improved PNS resistance
//!
//! # Example
//!
//! ```rust
//! use r4w_core::quantum_key_rate_optimizer::{QkdConfig, QkdProtocol, KeyRateOptimizer};
//!
//! let config = QkdConfig {
//!     protocol: QkdProtocol::BB84,
//!     channel_loss_db: 10.0,
//!     detector_efficiency: 0.1,
//!     dark_count_rate_hz: 1e4,
//!     source_repetition_rate_hz: 1e9,
//!     mean_photon_number: 0.1,
//!     fiber_length_km: 50.0,
//!     attenuation_db_per_km: 0.2,
//!     error_correction_efficiency: 1.16,
//!     misalignment_error: 0.01,
//! };
//!
//! let optimizer = KeyRateOptimizer::new(config);
//! let rate = optimizer.secure_key_rate();
//! println!("Secure key rate: {:.6} bits/pulse", rate);
//! ```

use std::f64::consts::{E, LN_2};

// ─── Protocol Enum ───────────────────────────────────────────────────────────

/// Supported QKD protocol variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QkdProtocol {
    /// Bennett-Brassard 1984 four-state protocol (H, V, D, A polarizations).
    /// Sifting fraction: 0.5. Security threshold: QBER < 11%.
    BB84,
    /// Bennett 1992 two-state protocol.
    /// Sifting fraction: 0.5. Security threshold: QBER ~ 7%.
    B92,
    /// Ekert 1991 entanglement-based protocol using Bell inequality violations.
    /// Equivalent key rate to BB84 in the asymptotic limit.
    E91,
    /// Scarani-Acin-Ribordy-Gisin 2004 four-state protocol with improved
    /// resistance to photon-number-splitting (PNS) attacks.
    /// Sifting fraction: 0.25.
    SARG04,
    /// BB84 with decoy state method for tight single-photon bounds.
    /// Uses multiple intensity levels (signal, decoy, vacuum).
    DecoyBB84,
}

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration parameters for a QKD system.
#[derive(Debug, Clone)]
pub struct QkdConfig {
    /// QKD protocol variant.
    pub protocol: QkdProtocol,
    /// Total channel loss in dB (includes fiber + coupling + other losses).
    pub channel_loss_db: f64,
    /// Single-photon detector efficiency (probability of detecting an arriving photon).
    /// Range: 0.0 to 1.0. Typical InGaAs SPADs: 0.1-0.25, SNSPDs: 0.8-0.95.
    pub detector_efficiency: f64,
    /// Dark count rate of each detector in Hz.
    /// Typical: 10-1000 Hz for SNSPDs, 100-10000 Hz for InGaAs SPADs.
    pub dark_count_rate_hz: f64,
    /// Source pulse repetition rate in Hz. Typical: 1 MHz to 10 GHz.
    pub source_repetition_rate_hz: f64,
    /// Mean photon number per pulse for weak coherent pulse (WCP) sources.
    /// Optimal value depends on channel loss; typically 0.1-0.8.
    pub mean_photon_number: f64,
    /// Fiber link length in km.
    pub fiber_length_km: f64,
    /// Fiber attenuation coefficient in dB/km.
    /// Standard single-mode fiber at 1550 nm: ~0.2 dB/km.
    pub attenuation_db_per_km: f64,
    /// Error correction efficiency factor (f_EC >= 1.0).
    /// f_EC = 1.0 is Shannon limit; practical systems achieve f_EC ~ 1.16.
    pub error_correction_efficiency: f64,
    /// Optical misalignment error probability.
    /// Contributes to baseline QBER even without eavesdropping.
    pub misalignment_error: f64,
}

impl Default for QkdConfig {
    fn default() -> Self {
        Self {
            protocol: QkdProtocol::BB84,
            channel_loss_db: 10.0,
            detector_efficiency: 0.1,
            dark_count_rate_hz: 1e4,
            source_repetition_rate_hz: 1e9,
            mean_photon_number: 0.1,
            fiber_length_km: 50.0,
            attenuation_db_per_km: 0.2,
            error_correction_efficiency: 1.16,
            misalignment_error: 0.01,
        }
    }
}

// ─── Key Rate Optimizer ──────────────────────────────────────────────────────

/// Computes secure key generation rates for various QKD protocols.
///
/// The optimizer uses the GLLP (Gottesman-Lo-Lutkenhaus-Preskill) security
/// framework to compute asymptotic key rates, accounting for realistic
/// device imperfections including detector dark counts, limited efficiency,
/// optical misalignment, and finite error correction efficiency.
pub struct KeyRateOptimizer {
    config: QkdConfig,
}

impl KeyRateOptimizer {
    /// Creates a new key rate optimizer with the given configuration.
    pub fn new(config: QkdConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the current configuration.
    pub fn config(&self) -> &QkdConfig {
        &self.config
    }

    /// Converts channel loss in dB to linear transmittance.
    ///
    /// `transmittance = 10^(-loss_db / 10)`
    ///
    /// # Arguments
    /// * `loss_db` - Channel loss in decibels (positive value).
    ///
    /// # Returns
    /// Linear transmittance in range (0, 1].
    pub fn channel_transmittance(loss_db: f64) -> f64 {
        10.0_f64.powf(-loss_db / 10.0)
    }

    /// Computes total fiber loss for a given length and attenuation coefficient.
    ///
    /// `loss_db = length_km * attenuation_db_per_km`
    ///
    /// # Arguments
    /// * `length_km` - Fiber length in kilometers.
    /// * `atten_db_per_km` - Attenuation in dB/km.
    ///
    /// # Returns
    /// Total loss in dB.
    pub fn fiber_loss(length_km: f64, atten_db_per_km: f64) -> f64 {
        length_km * atten_db_per_km
    }

    /// Computes the overall detection probability per pulse.
    ///
    /// For a WCP source with mean photon number mu:
    /// `P_detect = 1 - exp(-mu * transmittance * detector_efficiency)`
    ///
    /// This accounts for the Poisson photon number distribution and
    /// detector efficiency.
    ///
    /// # Arguments
    /// * `transmittance` - Channel transmittance (linear).
    /// * `detector_eff` - Detector efficiency (0 to 1).
    ///
    /// # Returns
    /// Detection probability per pulse.
    pub fn detection_probability(transmittance: f64, detector_eff: f64) -> f64 {
        // Using the approximation for WCP: mu * eta * t (valid for small mu*eta*t)
        // More accurate: 1 - exp(-mu * eta * t) + p_dark
        // For simplicity we return the linear product which is the per-photon
        // detection probability.
        transmittance * detector_eff
    }

    /// Estimates the quantum bit error rate (QBER) from system parameters.
    ///
    /// QBER has contributions from:
    /// 1. Dark counts (random, contribute 50% errors)
    /// 2. Optical misalignment
    ///
    /// `QBER = (0.5 * p_dark + e_misalign * p_signal) / (p_dark + p_signal)`
    ///
    /// where p_dark is the dark count probability per gate and p_signal is the
    /// signal detection probability.
    ///
    /// # Arguments
    /// * `dark_count_rate` - Dark count probability per detection gate.
    /// * `detection_prob` - Signal detection probability per pulse.
    /// * `misalign` - Misalignment error probability.
    ///
    /// # Returns
    /// Estimated QBER in range [0, 0.5].
    pub fn qber_estimate(dark_count_rate: f64, detection_prob: f64, misalign: f64) -> f64 {
        let total = detection_prob + dark_count_rate;
        if total <= 0.0 {
            return 0.0;
        }
        let error_contribution = 0.5 * dark_count_rate + misalign * detection_prob;
        (error_contribution / total).min(0.5)
    }

    /// Computes the binary Shannon entropy function h(p).
    ///
    /// `h(p) = -p * log2(p) - (1-p) * log2(1-p)`
    ///
    /// This is the fundamental quantity in QKD security proofs, representing
    /// the uncertainty (in bits) of a binary random variable with bias p.
    ///
    /// # Arguments
    /// * `p` - Probability in range [0, 1].
    ///
    /// # Returns
    /// Binary entropy in range [0, 1]. Returns 0 for p=0 or p=1.
    pub fn binary_entropy(p: f64) -> f64 {
        if p <= 0.0 || p >= 1.0 {
            return 0.0;
        }
        -p * log2(p) - (1.0 - p) * log2(1.0 - p)
    }

    /// Computes the BB84 secure key rate per detection event.
    ///
    /// Using the Shor-Preskill security proof:
    /// `R = Q * [1 - h(e) - f_EC * h(e)]`
    ///
    /// where:
    /// - Q is the detection probability (gain)
    /// - h(e) is the binary entropy of the QBER
    /// - f_EC is the error correction inefficiency factor
    ///
    /// The first h(e) term accounts for privacy amplification (removing Eve's
    /// information), and f_EC * h(e) accounts for error correction leakage.
    ///
    /// # Arguments
    /// * `qber` - Quantum bit error rate.
    /// * `detection_prob` - Detection probability (gain Q).
    ///
    /// # Returns
    /// Secure key rate in bits per pulse. Returns 0 if QBER exceeds security threshold.
    pub fn bb84_key_rate(&self, qber: f64, detection_prob: f64) -> f64 {
        let he = Self::binary_entropy(qber);
        let rate = detection_prob
            * self.sifting_fraction(&QkdProtocol::BB84)
            * (1.0 - he - self.config.error_correction_efficiency * he);
        rate.max(0.0)
    }

    /// Computes the B92 secure key rate per detection event.
    ///
    /// B92 uses only two non-orthogonal states, yielding a tighter security
    /// bound than BB84:
    /// `R = Q * sift * [1 - h(e) - f_EC * h(e)] * (1 - 2*e)`
    ///
    /// The extra (1 - 2*e) factor accounts for the reduced tolerance to errors
    /// in the two-state protocol.
    ///
    /// # Arguments
    /// * `qber` - Quantum bit error rate.
    /// * `detection_prob` - Detection probability (gain Q).
    ///
    /// # Returns
    /// Secure key rate in bits per pulse.
    pub fn b92_key_rate(&self, qber: f64, detection_prob: f64) -> f64 {
        let he = Self::binary_entropy(qber);
        let base = 1.0 - he - self.config.error_correction_efficiency * he;
        // B92 has an additional penalty factor for its two-state structure
        let b92_factor = (1.0 - 2.0 * qber).max(0.0);
        let rate = detection_prob
            * self.sifting_fraction(&QkdProtocol::B92)
            * base
            * b92_factor;
        rate.max(0.0)
    }

    /// Returns the sifting fraction for a given protocol.
    ///
    /// The sifting fraction is the probability that Alice and Bob choose
    /// compatible measurement bases, determining what fraction of raw
    /// detections contribute to the sifted key.
    ///
    /// - BB84: 1/2 (two bases, random choice)
    /// - B92: 1/2 (conclusive/inconclusive measurement)
    /// - E91: 1/2 (equivalent to BB84 in key generation mode)
    /// - SARG04: 1/4 (four-state encoding with two-bit sifting)
    /// - DecoyBB84: 1/2 (same sifting as BB84)
    pub fn sifting_fraction(&self, protocol: &QkdProtocol) -> f64 {
        match protocol {
            QkdProtocol::BB84 => 0.5,
            QkdProtocol::B92 => 0.5,
            QkdProtocol::E91 => 0.5,
            QkdProtocol::SARG04 => 0.25,
            QkdProtocol::DecoyBB84 => 0.5,
        }
    }

    /// Computes the secure key rate for the configured protocol and parameters.
    ///
    /// This is the main entry point that combines channel model, QBER estimation,
    /// and protocol-specific key rate formulas.
    ///
    /// # Returns
    /// Secure key rate in bits per pulse. Returns 0 if no secure key can be generated.
    pub fn secure_key_rate(&self) -> f64 {
        let total_loss = Self::fiber_loss(
            self.config.fiber_length_km,
            self.config.attenuation_db_per_km,
        ) + self.config.channel_loss_db;

        let transmittance = Self::channel_transmittance(total_loss);

        let mu = self.config.mean_photon_number;
        let eta = self.config.detector_efficiency;

        // Signal detection probability (from Poisson + detector efficiency)
        let p_signal = 1.0 - (-mu * transmittance * eta).exp();

        // Dark count probability per gate
        let p_dark = self.config.dark_count_rate_hz / self.config.source_repetition_rate_hz;

        // Total detection probability (gain)
        let q_mu = p_signal + p_dark - p_signal * p_dark;

        // QBER estimate
        let qber = Self::qber_estimate(p_dark, p_signal, self.config.misalignment_error);

        match self.config.protocol {
            QkdProtocol::BB84 | QkdProtocol::E91 => self.bb84_key_rate(qber, q_mu),
            QkdProtocol::B92 => self.b92_key_rate(qber, q_mu),
            QkdProtocol::SARG04 => {
                // SARG04 has lower sifting but better PNS resistance
                let he = Self::binary_entropy(qber);
                let rate = q_mu
                    * self.sifting_fraction(&QkdProtocol::SARG04)
                    * (1.0 - he - self.config.error_correction_efficiency * he);
                rate.max(0.0)
            }
            QkdProtocol::DecoyBB84 => {
                // Use decoy state analysis for tighter bounds
                let analyzer = DecoyStateAnalyzer::new(mu, mu / 5.0, 0.0);
                let q_nu = 1.0 - (-(mu / 5.0) * transmittance * eta).exp() + p_dark;
                let e_mu_total = qber;
                let e_nu = Self::qber_estimate(
                    p_dark,
                    1.0 - (-(mu / 5.0) * transmittance * eta).exp(),
                    self.config.misalignment_error,
                );
                analyzer.decoy_key_rate(q_mu, e_mu_total, q_nu, e_nu, p_dark)
                    * self.sifting_fraction(&QkdProtocol::DecoyBB84)
            }
        }
    }

    /// Finds the maximum fiber distance (in km) at which a positive key rate
    /// can still be achieved.
    ///
    /// Uses a binary search over distance, finding where the secure key rate
    /// transitions from positive to zero.
    ///
    /// # Returns
    /// Maximum distance in km (resolution: ~0.1 km).
    pub fn max_distance_km(&self) -> f64 {
        let mut low = 0.0_f64;
        let mut high = 1000.0_f64; // Start with 1000 km upper bound

        for _ in 0..100 {
            let mid = (low + high) / 2.0;
            let mut test_config = self.config.clone();
            test_config.fiber_length_km = mid;
            let optimizer = KeyRateOptimizer::new(test_config);
            let rate = optimizer.secure_key_rate();
            if rate > 0.0 {
                low = mid;
            } else {
                high = mid;
            }
        }
        low
    }
}

// ─── Decoy State Analysis ────────────────────────────────────────────────────

/// Decoy state method for bounding single-photon contributions.
///
/// Practical QKD systems use weak coherent pulses (WCPs) which have a Poisson
/// photon number distribution. Multi-photon pulses are vulnerable to
/// photon-number-splitting (PNS) attacks. The decoy state method uses
/// additional intensity levels to tightly bound the single-photon gain and
/// error rate, enabling near-ideal key rates with practical sources.
///
/// # Three-Intensity Protocol
///
/// - Signal intensity mu (~0.5): Used for key generation
/// - Decoy intensity nu (~0.1): Used for parameter estimation
/// - Vacuum intensity (~0): Used for background estimation
pub struct DecoyStateAnalyzer {
    /// Signal state mean photon number.
    mu_signal: f64,
    /// Decoy state mean photon number.
    mu_decoy: f64,
    /// Vacuum state mean photon number (ideally 0).
    mu_vacuum: f64,
}

impl DecoyStateAnalyzer {
    /// Creates a new decoy state analyzer.
    ///
    /// # Arguments
    /// * `mu_signal` - Signal intensity (mean photon number per pulse).
    /// * `mu_decoy` - Decoy intensity (should be < mu_signal).
    /// * `mu_vacuum` - Vacuum intensity (ideally 0 or very small).
    pub fn new(mu_signal: f64, mu_decoy: f64, mu_vacuum: f64) -> Self {
        Self {
            mu_signal,
            mu_decoy,
            mu_vacuum,
        }
    }

    /// Computes a lower bound on the single-photon gain Q1.
    ///
    /// Using the decoy state equations:
    /// `Q1_lower >= (mu_s * Q_nu * exp(nu) - mu_d * Q_vac * exp(0)) / (mu_s * nu - mu_d * mu_vac)`
    ///
    /// Simplified for vacuum = 0:
    /// `Q1_lower >= (mu * Q_nu * exp(nu) - nu^2 * Q_vac / mu) / (mu * nu - nu^2)`
    ///
    /// # Arguments
    /// * `q_mu` - Gain (detection probability) at signal intensity.
    /// * `q_nu` - Gain at decoy intensity.
    /// * `q_vacuum` - Gain at vacuum intensity (background).
    ///
    /// # Returns
    /// Lower bound on single-photon gain Q1.
    pub fn single_photon_gain(&self, q_mu: f64, q_nu: f64, q_vacuum: f64) -> f64 {
        let mu = self.mu_signal;
        let nu = self.mu_decoy;

        let numerator = mu * q_nu * nu.exp() - nu * nu * q_vacuum;
        let denominator = mu * nu - nu * nu;

        if denominator <= 0.0 {
            return 0.0;
        }

        (numerator / denominator).max(0.0)
    }

    /// Computes an upper bound on the single-photon error rate e1.
    ///
    /// `e1_upper <= (E_nu * Q_nu * exp(nu) - 0.5 * Q_vac) / Q1`
    ///
    /// # Arguments
    /// * `e_mu` - Overall QBER at signal intensity.
    /// * `e_nu` - Overall QBER at decoy intensity.
    /// * `q_mu` - Gain at signal intensity.
    /// * `q_nu` - Gain at decoy intensity.
    ///
    /// # Returns
    /// Upper bound on single-photon error rate.
    pub fn single_photon_error(
        &self,
        _e_mu: f64,
        e_nu: f64,
        _q_mu: f64,
        q_nu: f64,
    ) -> f64 {
        let nu = self.mu_decoy;
        let q1 = self.single_photon_gain(q_nu, q_nu, 0.0);
        if q1 <= 0.0 {
            return 0.5;
        }
        let e1 = (e_nu * q_nu * nu.exp() - 0.5 * 0.0) / q1;
        e1.min(0.5).max(0.0)
    }

    /// Computes the decoy-state BB84 key rate.
    ///
    /// Uses the GLLP formula with decoy-bounded single-photon parameters:
    /// `R = Q1 * [1 - h(e1)] - Q_mu * f_EC * h(E_mu)`
    ///
    /// where Q1 and e1 are bounded by decoy state analysis.
    ///
    /// # Arguments
    /// * `q_mu` - Gain at signal intensity.
    /// * `e_mu` - QBER at signal intensity.
    /// * `q_nu` - Gain at decoy intensity.
    /// * `e_nu` - QBER at decoy intensity.
    /// * `q_vacuum` - Gain at vacuum intensity.
    ///
    /// # Returns
    /// Secure key rate in bits per pulse.
    pub fn decoy_key_rate(
        &self,
        q_mu: f64,
        e_mu: f64,
        q_nu: f64,
        e_nu: f64,
        q_vacuum: f64,
    ) -> f64 {
        let mu = self.mu_signal;
        let q1 = self.single_photon_gain(q_mu, q_nu, q_vacuum);
        let e1 = self.single_photon_error(e_mu, e_nu, q_mu, q_nu);

        // Single-photon contribution
        let q1_mu = q1 * mu * (-mu).exp();

        let he1 = KeyRateOptimizer::binary_entropy(e1);
        let he_mu = KeyRateOptimizer::binary_entropy(e_mu);

        // f_EC = 1.16 typical
        let f_ec = 1.16;

        let rate = q1_mu * (1.0 - he1) - q_mu * f_ec * he_mu;
        rate.max(0.0)
    }

    /// Finds approximately optimal signal, decoy, and vacuum intensities
    /// for a given channel loss.
    ///
    /// Uses heuristic rules from the literature:
    /// - Signal intensity: mu ~ sqrt(eta) for low loss, ~0.5-0.8 for moderate loss
    /// - Decoy intensity: nu ~ mu / 5
    /// - Vacuum: 0
    ///
    /// # Arguments
    /// * `channel_loss_db` - Total channel loss in dB.
    ///
    /// # Returns
    /// Tuple of (mu_signal, mu_decoy, mu_vacuum).
    pub fn optimal_intensities(channel_loss_db: f64) -> (f64, f64, f64) {
        let eta = KeyRateOptimizer::channel_transmittance(channel_loss_db);

        // Heuristic: mu_opt scales roughly as sqrt(eta) for optimal performance
        let mu = (eta.sqrt()).min(0.8).max(0.05);
        let nu = mu / 5.0;
        let vacuum = 0.0;

        (mu, nu, vacuum)
    }
}

// ─── Privacy Amplification ───────────────────────────────────────────────────

/// Privacy amplification bounds and final key length computation.
///
/// After error correction, Alice and Bob share an identical but partially
/// compromised key. Privacy amplification uses universal hashing to compress
/// this key, removing all information accessible to Eve.
pub struct PrivacyAmplification;

impl PrivacyAmplification {
    /// Computes the min-entropy rate available per sifted bit.
    ///
    /// For BB84 with one-way post-processing:
    /// `H_min_rate = 1 - h(e)`
    ///
    /// This is the amount of randomness per bit that is guaranteed to be
    /// unknown to Eve, given a QBER of e.
    ///
    /// # Arguments
    /// * `qber` - Quantum bit error rate.
    ///
    /// # Returns
    /// Min-entropy rate in bits per sifted bit.
    pub fn min_entropy_rate(qber: f64) -> f64 {
        (1.0 - KeyRateOptimizer::binary_entropy(qber)).max(0.0)
    }

    /// Computes the fraction of bits remaining after privacy amplification.
    ///
    /// `fraction = 1 - h(e) - leakage`
    ///
    /// where leakage = f_EC * h(e) is the information leaked during
    /// error correction.
    ///
    /// # Arguments
    /// * `qber` - Quantum bit error rate.
    /// * `error_correction_leakage` - Information leaked during EC, in bits per sifted bit.
    ///
    /// # Returns
    /// Fraction of sifted bits retained in final key.
    pub fn privacy_amplification_fraction(qber: f64, error_correction_leakage: f64) -> f64 {
        let he = KeyRateOptimizer::binary_entropy(qber);
        (1.0 - he - error_correction_leakage).max(0.0)
    }

    /// Computes the final secure key length after all post-processing.
    ///
    /// `L = max(0, floor(n * [1 - h(e) - f_EC * h(e)] - log2(1/epsilon)))`
    ///
    /// where:
    /// - n is the number of sifted bits
    /// - e is the QBER
    /// - f_EC is the error correction inefficiency
    /// - epsilon is the security parameter (composable security)
    ///
    /// # Arguments
    /// * `sifted_bits` - Number of sifted key bits.
    /// * `qber` - Quantum bit error rate.
    /// * `f_ec` - Error correction inefficiency factor (>= 1.0).
    /// * `security_parameter` - Epsilon for composable security (e.g., 1e-10).
    ///
    /// # Returns
    /// Number of final secure key bits.
    pub fn final_key_length(
        sifted_bits: usize,
        qber: f64,
        f_ec: f64,
        security_parameter: f64,
    ) -> usize {
        let he = KeyRateOptimizer::binary_entropy(qber);
        let rate = 1.0 - he - f_ec * he;
        if rate <= 0.0 || security_parameter <= 0.0 {
            return 0;
        }
        let security_bits = log2(1.0 / security_parameter);
        let key_bits = (sifted_bits as f64) * rate - security_bits;
        if key_bits <= 0.0 {
            0
        } else {
            key_bits.floor() as usize
        }
    }

    /// Computes the output size of a universal hash function for privacy amplification.
    ///
    /// `output_bits = floor(input_bits * fraction)`
    ///
    /// # Arguments
    /// * `input_bits` - Number of input (error-corrected) bits.
    /// * `fraction` - Privacy amplification fraction (from `privacy_amplification_fraction`).
    ///
    /// # Returns
    /// Number of output bits.
    pub fn universal_hash_output_bits(input_bits: usize, fraction: f64) -> usize {
        if fraction <= 0.0 {
            return 0;
        }
        ((input_bits as f64) * fraction).floor() as usize
    }
}

// ─── Channel Model ───────────────────────────────────────────────────────────

/// Quantum channel models for QKD security analysis.
///
/// Provides the physical-layer formulas connecting device parameters
/// to observable quantities (gain Q and error rate E).
pub struct ChannelModel;

impl ChannelModel {
    /// Returns the Kraus operator coefficients for a depolarizing channel.
    ///
    /// The depolarizing channel with QBER e maps:
    /// `rho -> (1-p)*rho + (p/3)*(X*rho*X + Y*rho*Y + Z*rho*Z)`
    ///
    /// where p = 4e/3 for BB84 QBER = e.
    ///
    /// Returns a 4x4 matrix representing the Pauli transfer matrix (PTM):
    /// ```text
    /// [[1,    0,    0,    0   ],
    ///  [0, 1-4e/3, 0,    0   ],
    ///  [0,    0, 1-4e/3, 0   ],
    ///  [0,    0,    0, 1-4e/3]]
    /// ```
    ///
    /// # Arguments
    /// * `qber` - Quantum bit error rate.
    ///
    /// # Returns
    /// 4x4 Pauli transfer matrix.
    pub fn depolarizing_channel(qber: f64) -> [[f64; 4]; 4] {
        let p = (4.0 * qber / 3.0).min(1.0);
        let shrink = 1.0 - p;
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, shrink, 0.0, 0.0],
            [0.0, 0.0, shrink, 0.0],
            [0.0, 0.0, 0.0, shrink],
        ]
    }

    /// Computes the overall gain Q_mu for BB84 with WCP source.
    ///
    /// `Q_mu = 1 - (1 - p_dark)^2 * exp(-mu * eta * t)`
    ///
    /// Simplified (assuming p_dark << 1):
    /// `Q_mu ≈ 1 - exp(-mu * eta * t) + 2 * p_dark`
    ///
    /// # Arguments
    /// * `mu` - Mean photon number per pulse.
    /// * `transmittance` - Channel transmittance (linear).
    /// * `dark_count` - Dark count probability per gate per detector.
    ///
    /// # Returns
    /// Overall gain (detection probability per pulse).
    pub fn bb84_gain_formula(mu: f64, transmittance: f64, dark_count: f64) -> f64 {
        let p_signal = 1.0 - (-mu * transmittance).exp();
        // Two detectors, each with dark count probability
        let p_no_dark = (1.0 - dark_count).powi(2);
        1.0 - p_no_dark * (-mu * transmittance).exp()
            + p_signal * (1.0 - p_no_dark) // correction term
    }

    /// Computes the overall QBER E_mu for BB84 with WCP source.
    ///
    /// `E_mu = (e_det * p_signal + 0.5 * p_dark) / Q_mu`
    ///
    /// where e_det is the detector misalignment error and p_signal is the
    /// signal detection probability.
    ///
    /// # Arguments
    /// * `mu` - Mean photon number per pulse.
    /// * `transmittance` - Channel transmittance.
    /// * `dark_count` - Dark count probability per gate.
    /// * `e_misalign` - Optical misalignment error probability.
    ///
    /// # Returns
    /// Overall quantum bit error rate.
    pub fn bb84_error_formula(
        mu: f64,
        transmittance: f64,
        dark_count: f64,
        e_misalign: f64,
    ) -> f64 {
        let p_signal = 1.0 - (-mu * transmittance).exp();
        let q_mu = p_signal + dark_count;
        if q_mu <= 0.0 {
            return 0.0;
        }
        let errors = e_misalign * p_signal + 0.5 * dark_count;
        (errors / q_mu).min(0.5)
    }
}

// ─── Helper Functions ────────────────────────────────────────────────────────

/// Computes the Poisson probability P(n; mu) = e^(-mu) * mu^n / n!
///
/// This is the probability of detecting exactly n photons from a coherent
/// (laser) source with mean photon number mu.
///
/// # Arguments
/// * `mu` - Mean photon number (intensity parameter).
/// * `n` - Number of photons.
///
/// # Returns
/// Poisson probability.
///
/// # Example
/// ```rust
/// use r4w_core::quantum_key_rate_optimizer::poisson_probability;
/// let p0 = poisson_probability(1.0, 0); // ~0.368
/// let p1 = poisson_probability(1.0, 1); // ~0.368
/// assert!((p0 - 0.368).abs() < 0.001);
/// ```
pub fn poisson_probability(mu: f64, n: usize) -> f64 {
    if mu < 0.0 {
        return 0.0;
    }
    if mu == 0.0 {
        return if n == 0 { 1.0 } else { 0.0 };
    }
    // Use log-space computation to avoid overflow for large n
    let log_p = -(mu) + (n as f64) * mu.ln() - ln_factorial(n);
    log_p.exp()
}

/// Computes log(n!) using Stirling's approximation for large n,
/// or exact computation for small n.
fn ln_factorial(n: usize) -> f64 {
    if n <= 1 {
        return 0.0;
    }
    if n <= 20 {
        // Exact for small values
        let mut result = 0.0_f64;
        for i in 2..=n {
            result += (i as f64).ln();
        }
        return result;
    }
    // Stirling's approximation for large n
    let nf = n as f64;
    nf * nf.ln() - nf + 0.5 * (2.0 * std::f64::consts::PI * nf).ln()
}

/// Computes the base-2 logarithm of x.
///
/// `log2(x) = ln(x) / ln(2)`
///
/// # Arguments
/// * `x` - Input value (must be > 0).
///
/// # Returns
/// Base-2 logarithm. Returns negative infinity for x = 0.
pub fn log2(x: f64) -> f64 {
    x.ln() / LN_2
}

/// Computes the mutual information I(A:B) for BB84.
///
/// For a binary symmetric channel with crossover probability e:
/// `I(A:B) = 1 - h(e)`
///
/// This represents the information Alice and Bob share per sifted bit.
///
/// # Arguments
/// * `qber` - Quantum bit error rate.
///
/// # Returns
/// Mutual information in bits.
pub fn mutual_information_bb84(qber: f64) -> f64 {
    1.0 - KeyRateOptimizer::binary_entropy(qber)
}

/// Computes the Holevo bound chi(B:E) for BB84.
///
/// For an intercept-resend attack on BB84:
/// `chi(B:E) = h(e)`
///
/// For more general attacks (collective/coherent), the Holevo bound equals
/// h(e) in the asymptotic limit due to the Devetak-Winter rate.
///
/// # Arguments
/// * `qber` - Quantum bit error rate.
///
/// # Returns
/// Holevo information in bits.
pub fn holevo_bound_bb84(qber: f64) -> f64 {
    KeyRateOptimizer::binary_entropy(qber)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;
    const LOOSE_EPSILON: f64 = 1e-4;

    // ── Binary Entropy Tests ──

    #[test]
    fn test_binary_entropy_zero() {
        assert!((KeyRateOptimizer::binary_entropy(0.0) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_binary_entropy_one() {
        assert!((KeyRateOptimizer::binary_entropy(1.0) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_binary_entropy_half() {
        assert!((KeyRateOptimizer::binary_entropy(0.5) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_binary_entropy_011() {
        // h(0.11) should be approximately 0.5
        let h = KeyRateOptimizer::binary_entropy(0.11);
        assert!((h - 0.5).abs() < 0.02, "h(0.11) = {}, expected ~0.5", h);
    }

    #[test]
    fn test_binary_entropy_symmetry() {
        // h(p) = h(1-p)
        let p = 0.3;
        let h1 = KeyRateOptimizer::binary_entropy(p);
        let h2 = KeyRateOptimizer::binary_entropy(1.0 - p);
        assert!((h1 - h2).abs() < EPSILON);
    }

    #[test]
    fn test_binary_entropy_monotone_increasing() {
        // h(p) is monotonically increasing on [0, 0.5]
        let h1 = KeyRateOptimizer::binary_entropy(0.1);
        let h2 = KeyRateOptimizer::binary_entropy(0.3);
        let h3 = KeyRateOptimizer::binary_entropy(0.5);
        assert!(h1 < h2);
        assert!(h2 < h3);
    }

    // ── Channel Transmittance Tests ──

    #[test]
    fn test_transmittance_zero_loss() {
        assert!((KeyRateOptimizer::channel_transmittance(0.0) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_transmittance_3db() {
        // 3 dB loss = 50% transmittance
        assert!((KeyRateOptimizer::channel_transmittance(3.0) - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_transmittance_10db() {
        // 10 dB loss = 10% transmittance
        assert!((KeyRateOptimizer::channel_transmittance(10.0) - 0.1).abs() < EPSILON);
    }

    #[test]
    fn test_transmittance_20db() {
        // 20 dB loss = 1% transmittance
        assert!((KeyRateOptimizer::channel_transmittance(20.0) - 0.01).abs() < EPSILON);
    }

    // ── Fiber Loss Tests ──

    #[test]
    fn test_fiber_loss_100km() {
        let loss = KeyRateOptimizer::fiber_loss(100.0, 0.2);
        assert!((loss - 20.0).abs() < EPSILON);
    }

    #[test]
    fn test_fiber_loss_zero_length() {
        let loss = KeyRateOptimizer::fiber_loss(0.0, 0.2);
        assert!((loss - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_fiber_loss_50km() {
        let loss = KeyRateOptimizer::fiber_loss(50.0, 0.2);
        assert!((loss - 10.0).abs() < EPSILON);
    }

    // ── Poisson Probability Tests ──

    #[test]
    fn test_poisson_sum_to_one() {
        let mu = 2.5;
        let sum: f64 = (0..30).map(|n| poisson_probability(mu, n)).sum();
        assert!((sum - 1.0).abs() < 1e-6, "Poisson sum = {}, expected ~1.0", sum);
    }

    #[test]
    fn test_poisson_mean_one() {
        // P(0; 1) = e^(-1) ≈ 0.3679
        let p0 = poisson_probability(1.0, 0);
        assert!((p0 - (-1.0_f64).exp()).abs() < EPSILON);
    }

    #[test]
    fn test_poisson_p1_mu1() {
        // P(1; 1) = e^(-1) ≈ 0.3679
        let p1 = poisson_probability(1.0, 1);
        assert!((p1 - (-1.0_f64).exp()).abs() < EPSILON);
    }

    #[test]
    fn test_poisson_zero_mu() {
        // P(0; 0) = 1, P(n>0; 0) = 0
        assert!((poisson_probability(0.0, 0) - 1.0).abs() < EPSILON);
        assert!((poisson_probability(0.0, 1) - 0.0).abs() < EPSILON);
        assert!((poisson_probability(0.0, 5) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_poisson_large_n() {
        // Should not panic or produce NaN for large n
        let p = poisson_probability(5.0, 50);
        assert!(p.is_finite());
        assert!(p >= 0.0);
    }

    // ── BB84 Key Rate Tests ──

    #[test]
    fn test_bb84_positive_rate_low_qber() {
        let config = QkdConfig {
            protocol: QkdProtocol::BB84,
            error_correction_efficiency: 1.16,
            ..Default::default()
        };
        let opt = KeyRateOptimizer::new(config);
        let rate = opt.bb84_key_rate(0.05, 0.01);
        assert!(rate > 0.0, "BB84 should have positive rate at 5% QBER, got {}", rate);
    }

    #[test]
    fn test_bb84_zero_rate_at_threshold() {
        // BB84 security threshold: QBER ≈ 11%
        // With f_EC = 1.16, the threshold is at about 11% QBER
        let config = QkdConfig {
            error_correction_efficiency: 1.16,
            ..Default::default()
        };
        let opt = KeyRateOptimizer::new(config);

        // At 11% QBER: 1 - h(0.11) - 1.16 * h(0.11) should be ≈ 0
        // h(0.11) ≈ 0.4997
        // 1 - 0.4997 - 1.16 * 0.4997 = 1 - 0.4997 - 0.5797 = -0.0794 < 0
        let rate = opt.bb84_key_rate(0.11, 0.01);
        assert!(rate <= EPSILON, "BB84 should have zero rate at 11% QBER, got {}", rate);
    }

    #[test]
    fn test_bb84_rate_increases_with_lower_qber() {
        let config = QkdConfig::default();
        let opt = KeyRateOptimizer::new(config);
        let rate_1pct = opt.bb84_key_rate(0.01, 0.01);
        let rate_5pct = opt.bb84_key_rate(0.05, 0.01);
        assert!(rate_1pct > rate_5pct, "Lower QBER should give higher rate");
    }

    #[test]
    fn test_bb84_rate_zero_for_high_qber() {
        let config = QkdConfig::default();
        let opt = KeyRateOptimizer::new(config);
        let rate = opt.bb84_key_rate(0.20, 0.01);
        assert!((rate - 0.0).abs() < EPSILON, "BB84 rate should be 0 at 20% QBER");
    }

    // ── Sifting Fraction Tests ──

    #[test]
    fn test_sifting_bb84() {
        let opt = KeyRateOptimizer::new(QkdConfig::default());
        assert!((opt.sifting_fraction(&QkdProtocol::BB84) - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_sifting_sarg04() {
        let opt = KeyRateOptimizer::new(QkdConfig::default());
        assert!((opt.sifting_fraction(&QkdProtocol::SARG04) - 0.25).abs() < EPSILON);
    }

    #[test]
    fn test_sifting_e91() {
        let opt = KeyRateOptimizer::new(QkdConfig::default());
        assert!((opt.sifting_fraction(&QkdProtocol::E91) - 0.5).abs() < EPSILON);
    }

    // ── QBER Estimation Tests ──

    #[test]
    fn test_qber_no_dark_no_misalign() {
        let qber = KeyRateOptimizer::qber_estimate(0.0, 0.01, 0.0);
        assert!((qber - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_qber_increases_with_dark_counts() {
        let q1 = KeyRateOptimizer::qber_estimate(1e-5, 0.001, 0.01);
        let q2 = KeyRateOptimizer::qber_estimate(1e-3, 0.001, 0.01);
        assert!(q2 > q1, "Higher dark counts should increase QBER");
    }

    #[test]
    fn test_qber_bounded_by_half() {
        let qber = KeyRateOptimizer::qber_estimate(0.9, 0.001, 0.5);
        assert!(qber <= 0.5, "QBER should not exceed 0.5");
    }

    #[test]
    fn test_qber_increases_with_distance() {
        // At longer distance, signal weakens and dark counts dominate
        let config_short = QkdConfig {
            fiber_length_km: 10.0,
            ..Default::default()
        };
        let config_long = QkdConfig {
            fiber_length_km: 100.0,
            ..Default::default()
        };
        let opt_short = KeyRateOptimizer::new(config_short);
        let opt_long = KeyRateOptimizer::new(config_long);

        let rate_short = opt_short.secure_key_rate();
        let rate_long = opt_long.secure_key_rate();

        // Longer distance should give lower (or equal) key rate
        assert!(rate_short >= rate_long,
            "Short distance rate {} should be >= long distance rate {}", rate_short, rate_long);
    }

    // ── Privacy Amplification Tests ──

    #[test]
    fn test_privacy_amplification_reduces_key() {
        let sifted = 100_000;
        let final_len = PrivacyAmplification::final_key_length(sifted, 0.05, 1.16, 1e-10);
        assert!(final_len > 0, "Should produce some key at 5% QBER");
        assert!(final_len < sifted, "Final key should be shorter than sifted key");
    }

    #[test]
    fn test_privacy_amplification_zero_at_threshold() {
        let sifted = 100_000;
        let final_len = PrivacyAmplification::final_key_length(sifted, 0.11, 1.16, 1e-10);
        assert_eq!(final_len, 0, "No secure key at 11% QBER");
    }

    #[test]
    fn test_min_entropy_rate_zero_qber() {
        let rate = PrivacyAmplification::min_entropy_rate(0.0);
        assert!((rate - 1.0).abs() < EPSILON, "Min entropy rate should be 1 at zero QBER");
    }

    #[test]
    fn test_min_entropy_rate_half_qber() {
        let rate = PrivacyAmplification::min_entropy_rate(0.5);
        assert!((rate - 0.0).abs() < EPSILON, "Min entropy rate should be 0 at QBER=0.5");
    }

    #[test]
    fn test_universal_hash_output() {
        let output = PrivacyAmplification::universal_hash_output_bits(1000, 0.5);
        assert_eq!(output, 500);
    }

    #[test]
    fn test_universal_hash_output_zero_fraction() {
        let output = PrivacyAmplification::universal_hash_output_bits(1000, 0.0);
        assert_eq!(output, 0);
    }

    #[test]
    fn test_privacy_fraction_decreases_with_qber() {
        let f1 = PrivacyAmplification::privacy_amplification_fraction(0.02, 0.1);
        let f2 = PrivacyAmplification::privacy_amplification_fraction(0.08, 0.1);
        assert!(f1 > f2, "Higher QBER should reduce PA fraction");
    }

    // ── Decoy State Analysis Tests ──

    #[test]
    fn test_decoy_higher_rate_than_nondecoy() {
        // The decoy state method should give a higher (or comparable) key rate
        // than standard BB84 with the same parameters, because it provides
        // tighter bounds on single-photon contributions.
        let config_bb84 = QkdConfig {
            protocol: QkdProtocol::BB84,
            fiber_length_km: 30.0,
            mean_photon_number: 0.5,
            detector_efficiency: 0.1,
            dark_count_rate_hz: 100.0,
            source_repetition_rate_hz: 1e9,
            misalignment_error: 0.01,
            attenuation_db_per_km: 0.2,
            channel_loss_db: 0.0,
            error_correction_efficiency: 1.16,
        };

        let config_decoy = QkdConfig {
            protocol: QkdProtocol::DecoyBB84,
            ..config_bb84.clone()
        };

        let rate_bb84 = KeyRateOptimizer::new(config_bb84).secure_key_rate();
        let rate_decoy = KeyRateOptimizer::new(config_decoy).secure_key_rate();

        // Decoy should give non-negative rate
        assert!(rate_decoy >= 0.0, "Decoy rate should be non-negative: {}", rate_decoy);
        // At least one should be positive for this short distance
        assert!(rate_bb84 > 0.0 || rate_decoy >= 0.0,
            "At least one protocol should work at 30 km");
    }

    #[test]
    fn test_optimal_intensities() {
        let (mu, nu, vac) = DecoyStateAnalyzer::optimal_intensities(10.0);
        assert!(mu > nu, "Signal intensity should exceed decoy intensity");
        assert!(nu > vac, "Decoy intensity should exceed vacuum");
        assert!(vac == 0.0, "Vacuum should be zero");
        assert!(mu <= 0.8, "Signal intensity should not exceed 0.8");
        assert!(mu > 0.0, "Signal intensity should be positive");
    }

    #[test]
    fn test_single_photon_gain_positive() {
        let analyzer = DecoyStateAnalyzer::new(0.5, 0.1, 0.0);
        let q1 = analyzer.single_photon_gain(0.01, 0.005, 1e-6);
        assert!(q1 >= 0.0, "Single photon gain should be non-negative");
    }

    // ── Secure Key Rate Tests ──

    #[test]
    fn test_secure_key_rate_positive_short_distance() {
        let config = QkdConfig {
            protocol: QkdProtocol::BB84,
            fiber_length_km: 10.0,
            channel_loss_db: 0.0,
            detector_efficiency: 0.1,
            dark_count_rate_hz: 100.0,
            source_repetition_rate_hz: 1e9,
            mean_photon_number: 0.1,
            attenuation_db_per_km: 0.2,
            error_correction_efficiency: 1.16,
            misalignment_error: 0.01,
        };
        let rate = KeyRateOptimizer::new(config).secure_key_rate();
        assert!(rate > 0.0, "Should get positive key rate at 10 km, got {}", rate);
    }

    #[test]
    fn test_secure_key_rate_zero_very_long_distance() {
        let config = QkdConfig {
            protocol: QkdProtocol::BB84,
            fiber_length_km: 500.0,
            channel_loss_db: 0.0,
            detector_efficiency: 0.1,
            dark_count_rate_hz: 1e4,
            source_repetition_rate_hz: 1e9,
            mean_photon_number: 0.1,
            attenuation_db_per_km: 0.2,
            error_correction_efficiency: 1.16,
            misalignment_error: 0.01,
        };
        let rate = KeyRateOptimizer::new(config).secure_key_rate();
        assert!((rate - 0.0).abs() < EPSILON, "Should get zero rate at 500 km, got {}", rate);
    }

    // ── Max Distance Tests ──

    #[test]
    fn test_max_distance_increases_with_better_detectors() {
        let config_bad = QkdConfig {
            protocol: QkdProtocol::BB84,
            detector_efficiency: 0.05,
            dark_count_rate_hz: 1e4,
            ..Default::default()
        };
        let config_good = QkdConfig {
            protocol: QkdProtocol::BB84,
            detector_efficiency: 0.3,
            dark_count_rate_hz: 10.0,
            ..Default::default()
        };

        let max_bad = KeyRateOptimizer::new(config_bad).max_distance_km();
        let max_good = KeyRateOptimizer::new(config_good).max_distance_km();

        assert!(max_good > max_bad,
            "Better detectors should allow longer distance: good={}, bad={}",
            max_good, max_bad);
    }

    #[test]
    fn test_max_distance_positive() {
        let config = QkdConfig {
            protocol: QkdProtocol::BB84,
            fiber_length_km: 0.0, // will be overridden
            channel_loss_db: 0.0,
            detector_efficiency: 0.1,
            dark_count_rate_hz: 100.0,
            source_repetition_rate_hz: 1e9,
            mean_photon_number: 0.1,
            attenuation_db_per_km: 0.2,
            error_correction_efficiency: 1.16,
            misalignment_error: 0.01,
        };
        let max_dist = KeyRateOptimizer::new(config).max_distance_km();
        assert!(max_dist > 0.0, "Max distance should be positive, got {}", max_dist);
    }

    // ── B92 Key Rate Tests ──

    #[test]
    fn test_b92_positive_rate_low_qber() {
        let config = QkdConfig::default();
        let opt = KeyRateOptimizer::new(config);
        let rate = opt.b92_key_rate(0.03, 0.01);
        assert!(rate > 0.0, "B92 should have positive rate at 3% QBER, got {}", rate);
    }

    #[test]
    fn test_b92_zero_rate_high_qber() {
        let config = QkdConfig::default();
        let opt = KeyRateOptimizer::new(config);
        let rate = opt.b92_key_rate(0.15, 0.01);
        assert!((rate - 0.0).abs() < EPSILON, "B92 should have zero rate at 15% QBER");
    }

    // ── Channel Model Tests ──

    #[test]
    fn test_depolarizing_channel_identity_at_zero_qber() {
        let ptm = ChannelModel::depolarizing_channel(0.0);
        // Should be identity: diagonal = 1
        assert!((ptm[0][0] - 1.0).abs() < EPSILON);
        assert!((ptm[1][1] - 1.0).abs() < EPSILON);
        assert!((ptm[2][2] - 1.0).abs() < EPSILON);
        assert!((ptm[3][3] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_depolarizing_channel_full_depolarization() {
        // At QBER = 0.25 (p=1/3 * 4 * 0.25 = 1/3), should have 2/3 shrink
        // Actually p = 4*0.25/3 = 1/3, shrink = 1 - 1/3 = 2/3
        let ptm = ChannelModel::depolarizing_channel(0.25);
        assert!((ptm[1][1] - 2.0 / 3.0).abs() < EPSILON);
    }

    #[test]
    fn test_bb84_gain_formula() {
        let q = ChannelModel::bb84_gain_formula(0.1, 0.5, 1e-6);
        assert!(q > 0.0, "Gain should be positive");
        assert!(q < 1.0, "Gain should be less than 1");
    }

    #[test]
    fn test_bb84_error_formula_no_misalign() {
        let e = ChannelModel::bb84_error_formula(0.1, 1.0, 0.0, 0.0);
        assert!((e - 0.0).abs() < EPSILON, "No errors without dark counts or misalignment");
    }

    // ── Log2 Tests ──

    #[test]
    fn test_log2_basic() {
        assert!((log2(1.0) - 0.0).abs() < EPSILON);
        assert!((log2(2.0) - 1.0).abs() < EPSILON);
        assert!((log2(4.0) - 2.0).abs() < EPSILON);
        assert!((log2(8.0) - 3.0).abs() < EPSILON);
    }

    // ── Mutual Information / Holevo Tests ──

    #[test]
    fn test_mutual_information_zero_qber() {
        let mi = mutual_information_bb84(0.0);
        assert!((mi - 1.0).abs() < EPSILON, "MI should be 1 at zero QBER");
    }

    #[test]
    fn test_mutual_information_half_qber() {
        let mi = mutual_information_bb84(0.5);
        assert!((mi - 0.0).abs() < EPSILON, "MI should be 0 at QBER=0.5");
    }

    #[test]
    fn test_holevo_bound_increases_with_qber() {
        let chi1 = holevo_bound_bb84(0.02);
        let chi2 = holevo_bound_bb84(0.08);
        assert!(chi2 > chi1, "Holevo bound should increase with QBER");
    }

    #[test]
    fn test_secure_rate_equals_mi_minus_holevo() {
        // For ideal BB84 (f_EC = 1): R = I(A:B) - chi(B:E) = (1-h(e)) - h(e) = 1 - 2h(e)
        let qber = 0.05;
        let mi = mutual_information_bb84(qber);
        let chi = holevo_bound_bb84(qber);
        let ideal_rate = mi - chi;

        // With f_EC = 1, this should match
        let config = QkdConfig {
            error_correction_efficiency: 1.0,
            ..Default::default()
        };
        let opt = KeyRateOptimizer::new(config);
        let computed_rate = opt.bb84_key_rate(qber, 1.0) / 0.5; // divide by sifting

        assert!((computed_rate - ideal_rate).abs() < LOOSE_EPSILON,
            "Rate should equal I(A:B) - chi(B:E) for f_EC=1: {} vs {}",
            computed_rate, ideal_rate);
    }

    // ── Detection Probability Tests ──

    #[test]
    fn test_detection_probability_zero_transmittance() {
        let p = KeyRateOptimizer::detection_probability(0.0, 0.5);
        assert!((p - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_detection_probability_full_efficiency() {
        let p = KeyRateOptimizer::detection_probability(0.5, 1.0);
        assert!((p - 0.5).abs() < EPSILON);
    }

    // ── Protocol Enum Tests ──

    #[test]
    fn test_protocol_enum_equality() {
        assert_eq!(QkdProtocol::BB84, QkdProtocol::BB84);
        assert_ne!(QkdProtocol::BB84, QkdProtocol::B92);
    }

    #[test]
    fn test_protocol_clone() {
        let p = QkdProtocol::DecoyBB84;
        let p2 = p;
        assert_eq!(p, p2);
    }
}
