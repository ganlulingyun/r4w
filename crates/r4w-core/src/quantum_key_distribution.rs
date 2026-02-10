//! BB84 quantum key distribution protocol simulation for secure key establishment.
//!
//! This module implements a simulation of the BB84 quantum key distribution (QKD)
//! protocol, which allows two parties (Alice and Bob) to establish a shared secret
//! key with information-theoretic security. The security is based on the no-cloning
//! theorem of quantum mechanics: any eavesdropper (Eve) attempting to intercept
//! photons will inevitably introduce detectable errors.
//!
//! # Protocol Overview
//!
//! 1. **Preparation**: Alice encodes random bits using random bases (Rectilinear or Diagonal)
//! 2. **Transmission**: Photons travel through a lossy quantum channel
//! 3. **Measurement**: Bob measures each photon in a randomly chosen basis
//! 4. **Sifting**: Alice and Bob publicly compare bases, keeping only matching-basis results
//! 5. **Error estimation**: A subset of sifted bits is sacrificed to estimate the QBER
//! 6. **Privacy amplification**: The remaining key is shortened to remove Eve's information
//!
//! # Example
//!
//! ```rust
//! use r4w_core::quantum_key_distribution::{Bb84Protocol, QkdConfig};
//!
//! let config = QkdConfig {
//!     num_photons: 10_000,
//!     channel_loss_db: 3.0,
//!     dark_count_rate: 1e-6,
//!     detector_efficiency: 0.85,
//!     eavesdrop_fraction: 0.0,
//! };
//! let protocol = Bb84Protocol::new(config);
//! let result = protocol.simulate(42);
//!
//! assert!(result.qber < 0.11, "QBER should be low without eavesdropping");
//! assert!(!result.eavesdropper_detected, "No eavesdropper should be detected");
//! assert!(!result.final_key_bits.is_empty(), "Should produce a final key");
//! println!("Final key length: {} bits, QBER: {:.4}", result.final_key_bits.len(), result.qber);
//! ```

/// Measurement/preparation basis for BB84 protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Basis {
    /// Rectilinear basis: encodes 0 as H (horizontal), 1 as V (vertical).
    Rectilinear,
    /// Diagonal basis: encodes 0 as D (diagonal, +45deg), 1 as A (anti-diagonal, -45deg).
    Diagonal,
}

/// Polarization state of a single photon.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolarizationState {
    /// Horizontal (0 degree linear polarization) -- bit 0 in Rectilinear basis.
    H,
    /// Vertical (90 degree linear polarization) -- bit 1 in Rectilinear basis.
    V,
    /// Diagonal (+45 degree linear polarization) -- bit 0 in Diagonal basis.
    D,
    /// Anti-diagonal (-45 degree linear polarization) -- bit 1 in Diagonal basis.
    A,
}

/// Configuration for a BB84 QKD simulation run.
#[derive(Debug, Clone)]
pub struct QkdConfig {
    /// Number of photons Alice transmits.
    pub num_photons: usize,
    /// Channel loss in dB (higher = more photons lost).
    pub channel_loss_db: f64,
    /// Dark count rate of Bob's detectors (probability per detection window).
    pub dark_count_rate: f64,
    /// Detector efficiency (probability that a photon triggers a detection, 0.0..=1.0).
    pub detector_efficiency: f64,
    /// Fraction of photons intercepted by an eavesdropper (0.0 = no Eve, 1.0 = full intercept).
    pub eavesdrop_fraction: f64,
}

impl Default for QkdConfig {
    fn default() -> Self {
        Self {
            num_photons: 10_000,
            channel_loss_db: 3.0,
            dark_count_rate: 1e-6,
            detector_efficiency: 0.85,
            eavesdrop_fraction: 0.0,
        }
    }
}

/// Result of a BB84 QKD simulation.
#[derive(Debug, Clone)]
pub struct QkdResult {
    /// Raw key bits before sifting (Alice's transmitted bits).
    pub raw_key_bits: Vec<bool>,
    /// Sifted key bits (only matching-basis measurements).
    pub sifted_key_bits: Vec<bool>,
    /// Final key after privacy amplification.
    pub final_key_bits: Vec<bool>,
    /// Quantum Bit Error Rate (fraction of errors in sifted key sample).
    pub qber: f64,
    /// Secret key rate: fraction of sifted bits that survive privacy amplification.
    pub secret_key_rate: f64,
    /// Whether the QBER exceeds the 11% security threshold.
    pub eavesdropper_detected: bool,
}

/// A simple linear congruential pseudo-random number generator.
/// Used to avoid external crate dependencies.
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        // Ensure non-zero state by mixing the seed
        Self {
            state: seed.wrapping_add(0x9E3779B97F4A7C15),
        }
    }

    /// Returns a pseudo-random u64 using the SplitMix64 algorithm.
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    /// Returns a pseudo-random f64 in [0.0, 1.0).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Returns a pseudo-random bool.
    fn next_bool(&mut self) -> bool {
        self.next_u64() & 1 == 1
    }
}

/// BB84 quantum key distribution protocol simulator.
///
/// Simulates the full BB84 protocol including photon preparation, lossy quantum
/// channel, measurement, basis sifting, QBER estimation, and privacy amplification.
#[derive(Debug, Clone)]
pub struct Bb84Protocol {
    config: QkdConfig,
}

impl Bb84Protocol {
    /// Create a new BB84 protocol simulator with the given configuration.
    pub fn new(config: QkdConfig) -> Self {
        Self { config }
    }

    /// Run a full BB84 simulation with the given random seed.
    ///
    /// Returns a [`QkdResult`] containing the raw, sifted, and final keys,
    /// along with the QBER and whether eavesdropping was detected.
    pub fn simulate(&self, seed: u64) -> QkdResult {
        let mut rng = SimpleRng::new(seed);

        // Step 1: Alice generates random bits and random bases
        let alice_bits: Vec<bool> = (0..self.config.num_photons)
            .map(|_| rng.next_bool())
            .collect();
        let alice_bases: Vec<Basis> = (0..self.config.num_photons)
            .map(|_| {
                if rng.next_bool() {
                    Basis::Rectilinear
                } else {
                    Basis::Diagonal
                }
            })
            .collect();

        // Step 2: Alice prepares photon polarization states
        let photon_states: Vec<PolarizationState> = alice_bits
            .iter()
            .zip(alice_bases.iter())
            .map(|(&bit, &basis)| self.prepare_photon(bit, basis))
            .collect();

        // Step 3: Eavesdropper (Eve) intercepts some photons.
        // Eve measures in a random basis and re-prepares. If she picks the wrong
        // basis she disturbs the state, introducing errors detectable by Alice/Bob.
        let mut states_after_eve: Vec<PolarizationState> = Vec::with_capacity(self.config.num_photons);
        for &state in &photon_states {
            if self.config.eavesdrop_fraction > 0.0 && rng.next_f64() < self.config.eavesdrop_fraction {
                let eve_basis = if rng.next_bool() {
                    Basis::Rectilinear
                } else {
                    Basis::Diagonal
                };
                let rand_val = rng.next_f64();
                let eve_result = self.measure_photon(state, eve_basis, &mut || rand_val);
                match eve_result {
                    Some(bit) => states_after_eve.push(self.prepare_photon(bit, eve_basis)),
                    None => states_after_eve.push(state),
                }
            } else {
                states_after_eve.push(state);
            }
        }

        // Step 4: Bob chooses random bases and measures
        let bob_bases: Vec<Basis> = (0..self.config.num_photons)
            .map(|_| {
                if rng.next_bool() {
                    Basis::Rectilinear
                } else {
                    Basis::Diagonal
                }
            })
            .collect();

        let channel_transmission = f64::powf(10.0, -self.config.channel_loss_db / 10.0);

        let bob_results: Vec<Option<bool>> = states_after_eve
            .iter()
            .zip(bob_bases.iter())
            .map(|(&state, &basis)| {
                // Channel loss: photon may not arrive
                let detection_prob = channel_transmission * self.config.detector_efficiency;
                if rng.next_f64() >= detection_prob {
                    // Photon lost; check for dark count
                    if rng.next_f64() < self.config.dark_count_rate {
                        Some(rng.next_bool()) // random dark count result
                    } else {
                        None
                    }
                } else {
                    let meas_rand = rng.next_f64();
                    self.measure_photon(state, basis, &mut || meas_rand)
                }
            })
            .collect();

        // Step 5: Basis sifting -- keep only positions where bases match and Bob detected
        let (_, bob_sifted) = self.sift_keys(&alice_bases, &bob_bases, &bob_results);

        // Build Alice's sifted key from matching-basis, detected positions
        let mut alice_sifted = Vec::new();
        for i in 0..self.config.num_photons {
            if alice_bases[i] == bob_bases[i] {
                if bob_results[i].is_some() {
                    alice_sifted.push(alice_bits[i]);
                }
            }
        }

        if alice_sifted.is_empty() {
            return QkdResult {
                raw_key_bits: alice_bits,
                sifted_key_bits: vec![],
                final_key_bits: vec![],
                qber: 1.0,
                secret_key_rate: 0.0,
                eavesdropper_detected: true,
            };
        }

        // Step 6: QBER estimation -- sacrifice first half for error checking
        let sample_size = (alice_sifted.len() / 2).max(1);
        let alice_sample = &alice_sifted[..sample_size];
        let bob_sample = &bob_sifted[..sample_size];
        let qber = self.estimate_qber(alice_sample, bob_sample);

        // Remaining bits form the raw key for privacy amplification
        let remaining_alice = alice_sifted[sample_size..].to_vec();

        // Step 7: Privacy amplification
        let skr = self.secret_key_rate(qber);
        let eavesdropper_detected = qber > 0.11;

        let final_key = if eavesdropper_detected || remaining_alice.is_empty() {
            vec![] // Abort: key is not secure
        } else {
            self.privacy_amplification(&remaining_alice, qber)
        };

        QkdResult {
            raw_key_bits: alice_bits,
            sifted_key_bits: alice_sifted,
            final_key_bits: final_key,
            qber,
            secret_key_rate: skr,
            eavesdropper_detected,
        }
    }

    /// Prepare a photon in the given polarization state for a bit value and basis.
    ///
    /// - Rectilinear basis: 0 -> H, 1 -> V
    /// - Diagonal basis: 0 -> D, 1 -> A
    pub fn prepare_photon(&self, bit: bool, basis: Basis) -> PolarizationState {
        match (basis, bit) {
            (Basis::Rectilinear, false) => PolarizationState::H,
            (Basis::Rectilinear, true) => PolarizationState::V,
            (Basis::Diagonal, false) => PolarizationState::D,
            (Basis::Diagonal, true) => PolarizationState::A,
        }
    }

    /// Measure a photon in a given basis.
    ///
    /// If the measurement basis matches the preparation basis, the correct bit is
    /// returned deterministically. If the bases are incompatible (e.g., measuring
    /// an H/V photon in the Diagonal basis), the result is random (50/50).
    ///
    /// The `rng` closure should return a value in `[0.0, 1.0)` for randomized outcomes.
    ///
    /// Returns `Some(bit)` with the measurement result. Detection loss is handled
    /// at the channel level in `simulate()`, not here.
    pub fn measure_photon(
        &self,
        state: PolarizationState,
        basis: Basis,
        rng: &mut impl FnMut() -> f64,
    ) -> Option<bool> {
        match (state, basis) {
            // Matching basis: deterministic outcome
            (PolarizationState::H, Basis::Rectilinear) => Some(false),
            (PolarizationState::V, Basis::Rectilinear) => Some(true),
            (PolarizationState::D, Basis::Diagonal) => Some(false),
            (PolarizationState::A, Basis::Diagonal) => Some(true),
            // Mismatched basis: random outcome (50/50)
            (PolarizationState::H | PolarizationState::V, Basis::Diagonal) => {
                Some(rng() < 0.5)
            }
            (PolarizationState::D | PolarizationState::A, Basis::Rectilinear) => {
                Some(rng() < 0.5)
            }
        }
    }

    /// Sift keys by keeping only positions where Alice and Bob used the same basis
    /// and Bob successfully detected a photon.
    ///
    /// Returns `(alice_sifted, bob_sifted)`. Note: without Alice's original bits,
    /// `alice_sifted` mirrors `bob_sifted` here. The `simulate()` method builds
    /// the true `alice_sifted` from Alice's original bit array separately.
    pub fn sift_keys(
        &self,
        alice_bases: &[Basis],
        bob_bases: &[Basis],
        bob_results: &[Option<bool>],
    ) -> (Vec<bool>, Vec<bool>) {
        let mut alice_sifted = Vec::new();
        let mut bob_sifted = Vec::new();

        let len = alice_bases.len().min(bob_bases.len()).min(bob_results.len());
        for i in 0..len {
            if alice_bases[i] == bob_bases[i] {
                if let Some(bit) = bob_results[i] {
                    alice_sifted.push(bit);
                    bob_sifted.push(bit);
                }
            }
        }

        (alice_sifted, bob_sifted)
    }

    /// Estimate the Quantum Bit Error Rate from sample bits.
    ///
    /// Compares corresponding bits from Alice's and Bob's sample subsets.
    /// Returns the fraction of mismatched bits.
    pub fn estimate_qber(&self, alice_sample: &[bool], bob_sample: &[bool]) -> f64 {
        if alice_sample.is_empty() || bob_sample.is_empty() {
            return 0.0;
        }
        let len = alice_sample.len().min(bob_sample.len());
        let errors = alice_sample[..len]
            .iter()
            .zip(bob_sample[..len].iter())
            .filter(|(&a, &b)| a != b)
            .count();
        errors as f64 / len as f64
    }

    /// Apply privacy amplification to reduce Eve's information about the key.
    ///
    /// Uses universal hashing (XOR-based) to compress the key. The output length
    /// is determined by the secret key rate: `output_len = key.len() * (1 - 2*h(qber))`.
    pub fn privacy_amplification(&self, key: &[bool], qber: f64) -> Vec<bool> {
        if key.is_empty() {
            return vec![];
        }
        let rate = self.secret_key_rate(qber);
        if rate <= 0.0 {
            return vec![];
        }
        let output_len = ((key.len() as f64) * rate).floor() as usize;
        let output_len = output_len.max(1).min(key.len());

        // XOR-based universal hashing: each output bit is XOR of a pseudo-random
        // subset of input bits, using a different hash function per output bit.
        let mut result = Vec::with_capacity(output_len);
        let n = key.len();

        for i in 0..output_len {
            let mut bit = false;
            let mut hash_state: u64 = (i as u64)
                .wrapping_mul(0x517CC1B727220A95)
                .wrapping_add(0x6C62272E07BB0142);
            for j in 0..n {
                hash_state = hash_state
                    .wrapping_mul(0x5851F42D4C957F2D)
                    .wrapping_add(0x14057B7EF767814F);
                if (hash_state >> 63) == 1 {
                    bit ^= key[j];
                }
            }
            result.push(bit);
        }

        result
    }

    /// Compute the secret key rate: `r = 1 - 2*h(qber)`
    /// where `h(x)` is the binary entropy function: `h(x) = -x*log2(x) - (1-x)*log2(1-x)`.
    ///
    /// Returns 0.0 if the rate would be negative (QBER too high for secure key extraction).
    pub fn secret_key_rate(&self, qber: f64) -> f64 {
        let h = binary_entropy(qber);
        let rate = 1.0 - 2.0 * h;
        if rate < 0.0 {
            0.0
        } else {
            rate
        }
    }
}

/// Binary entropy function: `h(x) = -x*log2(x) - (1-x)*log2(1-x)`.
///
/// Returns 0.0 for x = 0 or x = 1 (by convention, since lim x->0 of x*log(x) = 0).
fn binary_entropy(x: f64) -> f64 {
    if x <= 0.0 || x >= 1.0 {
        return 0.0;
    }
    -x * x.log2() - (1.0 - x) * (1.0 - x).log2()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_prepare_photon_rectilinear() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        assert_eq!(proto.prepare_photon(false, Basis::Rectilinear), PolarizationState::H);
        assert_eq!(proto.prepare_photon(true, Basis::Rectilinear), PolarizationState::V);
    }

    #[test]
    fn test_prepare_photon_diagonal() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        assert_eq!(proto.prepare_photon(false, Basis::Diagonal), PolarizationState::D);
        assert_eq!(proto.prepare_photon(true, Basis::Diagonal), PolarizationState::A);
    }

    #[test]
    fn test_measure_matching_basis_rectilinear() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let mut dummy_rng = || 0.5_f64;
        assert_eq!(
            proto.measure_photon(PolarizationState::H, Basis::Rectilinear, &mut dummy_rng),
            Some(false)
        );
        assert_eq!(
            proto.measure_photon(PolarizationState::V, Basis::Rectilinear, &mut dummy_rng),
            Some(true)
        );
    }

    #[test]
    fn test_measure_matching_basis_diagonal() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let mut dummy_rng = || 0.5_f64;
        assert_eq!(
            proto.measure_photon(PolarizationState::D, Basis::Diagonal, &mut dummy_rng),
            Some(false)
        );
        assert_eq!(
            proto.measure_photon(PolarizationState::A, Basis::Diagonal, &mut dummy_rng),
            Some(true)
        );
    }

    #[test]
    fn test_measure_mismatched_basis_is_random() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        // rng returns 0.3 < 0.5, so result should be true
        let mut rng_low = || 0.3_f64;
        assert_eq!(
            proto.measure_photon(PolarizationState::H, Basis::Diagonal, &mut rng_low),
            Some(true)
        );
        // rng returns 0.7 >= 0.5, so result should be false
        let mut rng_high = || 0.7_f64;
        assert_eq!(
            proto.measure_photon(PolarizationState::H, Basis::Diagonal, &mut rng_high),
            Some(false)
        );
    }

    #[test]
    fn test_measure_diagonal_in_rectilinear() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let mut rng_low = || 0.3_f64;
        assert_eq!(
            proto.measure_photon(PolarizationState::D, Basis::Rectilinear, &mut rng_low),
            Some(true)
        );
        let mut rng_high = || 0.7_f64;
        assert_eq!(
            proto.measure_photon(PolarizationState::A, Basis::Rectilinear, &mut rng_high),
            Some(false)
        );
    }

    #[test]
    fn test_estimate_qber_no_errors() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let alice = vec![true, false, true, true, false];
        let bob = vec![true, false, true, true, false];
        assert!((proto.estimate_qber(&alice, &bob) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_estimate_qber_all_errors() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let alice = vec![true, false, true, false];
        let bob = vec![false, true, false, true];
        assert!((proto.estimate_qber(&alice, &bob) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_estimate_qber_partial_errors() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let alice = vec![true, false, true, false];
        let bob = vec![true, true, true, false]; // 1 error out of 4
        assert!((proto.estimate_qber(&alice, &bob) - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_estimate_qber_empty() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        assert!((proto.estimate_qber(&[], &[]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_secret_key_rate_zero_qber() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        // h(0) = 0, so rate = 1 - 0 = 1.0
        assert!((proto.secret_key_rate(0.0) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_secret_key_rate_high_qber() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        // At QBER = 0.5, h(0.5) = 1.0, so rate = 1 - 2 = -1, clamped to 0.0
        assert!((proto.secret_key_rate(0.5) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_secret_key_rate_threshold() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        // BB84 threshold is ~11%. At exactly 0.11, rate should be very small but positive.
        let rate = proto.secret_key_rate(0.11);
        assert!(rate >= 0.0);
        assert!(rate < 0.2);
        // At 0.12, rate should be essentially zero or very small
        let rate_above = proto.secret_key_rate(0.12);
        assert!(rate_above < 0.05);
    }

    #[test]
    fn test_binary_entropy() {
        assert!((binary_entropy(0.0) - 0.0).abs() < 1e-10);
        assert!((binary_entropy(1.0) - 0.0).abs() < 1e-10);
        assert!((binary_entropy(0.5) - 1.0).abs() < 1e-10);
        // h(0.25) is approximately 0.8113
        assert!((binary_entropy(0.25) - 0.8113).abs() < 0.001);
    }

    #[test]
    fn test_privacy_amplification_reduces_key() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let key: Vec<bool> = (0..100).map(|i| i % 3 == 0).collect();
        let result = proto.privacy_amplification(&key, 0.05);
        // With QBER 0.05, rate is approx 1 - 2*h(0.05) which is approx 0.286
        assert!(!result.is_empty());
        assert!(result.len() < key.len());
    }

    #[test]
    fn test_privacy_amplification_empty_key() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let result = proto.privacy_amplification(&[], 0.05);
        assert!(result.is_empty());
    }

    #[test]
    fn test_simulate_no_eavesdropper() {
        let config = QkdConfig {
            num_photons: 20_000,
            channel_loss_db: 1.0,
            dark_count_rate: 0.0,
            detector_efficiency: 1.0,
            eavesdrop_fraction: 0.0,
        };
        let proto = Bb84Protocol::new(config);
        let result = proto.simulate(12345);

        // Without eavesdropping and ideal channel, QBER should be very low
        assert!(result.qber < 0.05, "QBER was {} (expected < 0.05)", result.qber);
        assert!(!result.eavesdropper_detected);
        assert!(!result.final_key_bits.is_empty());
        assert!(!result.sifted_key_bits.is_empty());
        // Sifted key should be roughly half of detected photons (basis matching ~50%)
        assert!(result.sifted_key_bits.len() > 1000);
    }

    #[test]
    fn test_simulate_with_eavesdropper() {
        let config = QkdConfig {
            num_photons: 50_000,
            channel_loss_db: 0.5,
            dark_count_rate: 0.0,
            detector_efficiency: 1.0,
            eavesdrop_fraction: 1.0, // Eve intercepts everything
        };
        let proto = Bb84Protocol::new(config);
        let result = proto.simulate(99999);

        // Full eavesdropping should cause ~25% QBER (Eve uses wrong basis 50% of the time,
        // and wrong basis gives random result 50% of the time, yielding 25% error rate)
        assert!(
            result.qber > 0.11,
            "QBER was {} (expected > 0.11 with full eavesdropping)",
            result.qber
        );
        assert!(result.eavesdropper_detected);
        assert!(
            result.final_key_bits.is_empty(),
            "Key should be empty when eavesdropper detected"
        );
    }

    #[test]
    fn test_simulate_deterministic_with_seed() {
        let config = QkdConfig {
            num_photons: 5_000,
            channel_loss_db: 2.0,
            dark_count_rate: 1e-7,
            detector_efficiency: 0.9,
            eavesdrop_fraction: 0.0,
        };
        let proto = Bb84Protocol::new(config);
        let result1 = proto.simulate(42);
        let result2 = proto.simulate(42);

        assert_eq!(result1.sifted_key_bits, result2.sifted_key_bits);
        assert_eq!(result1.final_key_bits, result2.final_key_bits);
        assert!((result1.qber - result2.qber).abs() < 1e-15);
    }

    #[test]
    fn test_sift_keys_matching_bases() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let alice_bases = vec![
            Basis::Rectilinear,
            Basis::Diagonal,
            Basis::Rectilinear,
            Basis::Diagonal,
        ];
        let bob_bases = vec![
            Basis::Rectilinear,
            Basis::Rectilinear,
            Basis::Rectilinear,
            Basis::Diagonal,
        ];
        let bob_results = vec![Some(false), Some(true), Some(true), Some(false)];

        let (_, bob_sifted) = proto.sift_keys(&alice_bases, &bob_bases, &bob_results);

        // Matching bases at indices 0, 2, 3
        assert_eq!(bob_sifted.len(), 3);
        assert_eq!(bob_sifted, vec![false, true, false]);
    }

    #[test]
    fn test_sift_keys_with_lost_photons() {
        let proto = Bb84Protocol::new(QkdConfig::default());
        let alice_bases = vec![Basis::Rectilinear, Basis::Diagonal, Basis::Rectilinear];
        let bob_bases = vec![Basis::Rectilinear, Basis::Diagonal, Basis::Rectilinear];
        let bob_results = vec![Some(false), None, Some(true)]; // middle photon lost

        let (_, bob_sifted) = proto.sift_keys(&alice_bases, &bob_bases, &bob_results);

        // All bases match, but index 1 is lost
        assert_eq!(bob_sifted.len(), 2);
        assert_eq!(bob_sifted, vec![false, true]);
    }

    #[test]
    fn test_default_config() {
        let config = QkdConfig::default();
        assert_eq!(config.num_photons, 10_000);
        assert!((config.channel_loss_db - 3.0).abs() < 1e-10);
        assert!((config.dark_count_rate - 1e-6).abs() < 1e-10);
        assert!((config.detector_efficiency - 0.85).abs() < 1e-10);
        assert!((config.eavesdrop_fraction - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_channel_loss_affects_sifted_key_length() {
        let config_low_loss = QkdConfig {
            num_photons: 10_000,
            channel_loss_db: 0.5,
            dark_count_rate: 0.0,
            detector_efficiency: 1.0,
            eavesdrop_fraction: 0.0,
        };
        let config_high_loss = QkdConfig {
            num_photons: 10_000,
            channel_loss_db: 10.0,
            dark_count_rate: 0.0,
            detector_efficiency: 1.0,
            eavesdrop_fraction: 0.0,
        };
        let proto_low = Bb84Protocol::new(config_low_loss);
        let proto_high = Bb84Protocol::new(config_high_loss);

        let result_low = proto_low.simulate(7777);
        let result_high = proto_high.simulate(7777);

        assert!(
            result_low.sifted_key_bits.len() > result_high.sifted_key_bits.len(),
            "Low loss ({}) should yield more sifted bits than high loss ({})",
            result_low.sifted_key_bits.len(),
            result_high.sifted_key_bits.len()
        );
    }
}
