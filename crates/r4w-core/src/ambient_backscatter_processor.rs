//! Ambient RF backscatter signal detection and decoding.
//!
//! This module detects and decodes ambient RF backscatter signals from passive
//! tags that modulate existing WiFi or cellular transmissions. Ambient
//! backscatter allows battery-free tags to communicate by reflecting and
//! modulating ambient RF energy, enabling ultra-low-power IoT communication.
//!
//! # Overview
//!
//! Ambient backscatter works by having a passive tag switch its antenna
//! impedance between reflecting and absorbing states, effectively creating
//! on-off keying (OOK) or frequency-shift keying (FSK) modulation on top of
//! existing ambient RF signals (WiFi, TV broadcasts, cellular, etc.).
//!
//! The receiver must:
//! 1. Separate the ambient carrier from the tag's modulation
//! 2. Detect the tag's presence via envelope averaging
//! 3. Decode the tag data from amplitude or frequency variations
//!
//! # Example
//!
//! ```
//! use r4w_core::ambient_backscatter_processor::{
//!     AmbientBackscatterProcessor, BackscatterConfig, BackscatterModulation,
//! };
//!
//! let config = BackscatterConfig {
//!     sample_rate: 100_000.0,
//!     tag_data_rate: 1_000.0,
//!     averaging_window: 50,
//!     detection_threshold: 0.05,
//!     modulation: BackscatterModulation::Ook,
//! };
//!
//! let mut processor = AmbientBackscatterProcessor::new(config);
//!
//! // Generate a synthetic backscatter signal: ambient + tag data
//! let tag_bits = vec![true, false, true, false, true, false, true, false];
//! let signal = processor.simulate_tag(&tag_bits, 0.5, 20.0);
//!
//! // Detect tag presence
//! let detected = processor.detect_tag(&signal);
//! assert!(detected);
//!
//! // Decode tag data
//! let message = processor.decode_tag(&signal);
//! assert!(!message.bits.is_empty());
//! ```

use std::f64::consts::PI;

/// Complex number as `(re, im)` tuple.
type Complex = (f64, f64);

/// Backscatter modulation scheme used by the tag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BackscatterModulation {
    /// On-off keying: tag switches between reflecting and absorbing.
    Ook,
    /// Frequency-shift keying: tag shifts reflected frequency.
    Fsk,
}

/// Configuration for the ambient backscatter processor.
#[derive(Debug, Clone)]
pub struct BackscatterConfig {
    /// Receiver sample rate in Hz.
    pub sample_rate: f64,
    /// Tag data rate in bits per second.
    pub tag_data_rate: f64,
    /// Number of samples for the envelope averaging window.
    pub averaging_window: usize,
    /// Detection threshold (0.0 to 1.0) for tag presence.
    pub detection_threshold: f64,
    /// Modulation type used by the tag.
    pub modulation: BackscatterModulation,
}

impl Default for BackscatterConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1_000_000.0,
            tag_data_rate: 1_000.0,
            averaging_window: 100,
            detection_threshold: 0.15,
            modulation: BackscatterModulation::Ook,
        }
    }
}

/// Decoded message from a backscatter tag.
#[derive(Debug, Clone)]
pub struct TagMessage {
    /// Decoded bit sequence.
    pub bits: Vec<bool>,
    /// Received signal strength indicator in dB.
    pub rssi: f64,
    /// Estimated bit error rate.
    pub ber_estimate: f64,
}

/// Runtime statistics for the processor.
#[derive(Debug, Clone)]
pub struct Statistics {
    /// Fraction of detection attempts that found a tag (0.0 to 1.0).
    pub detection_rate: f64,
    /// Average signal-to-noise ratio in dB across decoded messages.
    pub average_snr: f64,
    /// Total number of successfully decoded messages.
    pub decoded_messages: u64,
}

/// Ambient backscatter signal processor.
///
/// Detects and decodes backscatter tag modulation riding on ambient RF.
#[derive(Debug)]
pub struct AmbientBackscatterProcessor {
    config: BackscatterConfig,
    stats: Statistics,
    detection_attempts: u64,
    detection_successes: u64,
    snr_accumulator: f64,
}

impl AmbientBackscatterProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: BackscatterConfig) -> Self {
        Self {
            config,
            stats: Statistics {
                detection_rate: 0.0,
                average_snr: 0.0,
                decoded_messages: 0,
            },
            detection_attempts: 0,
            detection_successes: 0,
            snr_accumulator: 0.0,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &BackscatterConfig {
        &self.config
    }

    /// Return a snapshot of the current statistics.
    pub fn statistics(&self) -> &Statistics {
        &self.stats
    }

    /// Detect whether a backscatter tag is present in the received signal.
    ///
    /// Computes the envelope of the signal, averages it, then checks whether
    /// the variance of the averaged envelope exceeds the detection threshold.
    /// A modulating tag introduces amplitude fluctuations at the tag data rate
    /// that are absent when only ambient RF is present.
    pub fn detect_tag(&mut self, signal: &[Complex]) -> bool {
        if signal.len() < self.config.averaging_window * 2 {
            return false;
        }

        self.detection_attempts += 1;

        let envelope = self.envelope_average(signal);
        if envelope.is_empty() {
            return false;
        }

        // Compute mean of envelope
        let mean = envelope.iter().sum::<f64>() / envelope.len() as f64;
        if mean < 1e-12 {
            return false;
        }

        // Compute normalized variance (coefficient of variation squared)
        let variance =
            envelope.iter().map(|&v| (v - mean) * (v - mean)).sum::<f64>() / envelope.len() as f64;
        let normalized_var = variance / (mean * mean);

        let detected = normalized_var > self.config.detection_threshold;

        if detected {
            self.detection_successes += 1;
        }
        self.stats.detection_rate = if self.detection_attempts > 0 {
            self.detection_successes as f64 / self.detection_attempts as f64
        } else {
            0.0
        };

        detected
    }

    /// Decode tag data from the received signal.
    ///
    /// Computes the signal envelope, averages per-bit segments, and slices
    /// into bits based on amplitude thresholding. The tag modulation appears
    /// as amplitude variation on the ambient carrier.
    pub fn decode_tag(&mut self, signal: &[Complex]) -> TagMessage {
        let samples_per_bit = (self.config.sample_rate / self.config.tag_data_rate).round() as usize;
        if samples_per_bit == 0 || signal.len() < samples_per_bit {
            return TagMessage {
                bits: Vec::new(),
                rssi: f64::NEG_INFINITY,
                ber_estimate: 1.0,
            };
        }

        // Compute envelope of the raw signal (tag modulation = amplitude variation)
        let env: Vec<f64> = signal
            .iter()
            .map(|&(re, im)| (re * re + im * im).sqrt())
            .collect();

        // Compute overall RSSI
        let signal_power: f64 =
            signal.iter().map(|&(re, im)| re * re + im * im).sum::<f64>() / signal.len() as f64;
        let rssi = if signal_power > 0.0 {
            10.0 * signal_power.log10()
        } else {
            f64::NEG_INFINITY
        };

        // Slice envelope into bits
        let num_bits = env.len() / samples_per_bit;
        let mut bits = Vec::with_capacity(num_bits);
        let mut bit_energies = Vec::with_capacity(num_bits);

        for i in 0..num_bits {
            let start = i * samples_per_bit;
            let end = start + samples_per_bit;
            let avg: f64 = env[start..end].iter().sum::<f64>() / samples_per_bit as f64;
            bit_energies.push(avg);
        }

        // Threshold: midpoint between min and max energy
        if bit_energies.is_empty() {
            return TagMessage {
                bits: Vec::new(),
                rssi,
                ber_estimate: 1.0,
            };
        }

        let max_e = bit_energies
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let min_e = bit_energies
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        let threshold = (max_e + min_e) / 2.0;

        for &e in &bit_energies {
            bits.push(e > threshold);
        }

        let ber = self.ber_estimate(&bit_energies, threshold);

        // Update statistics
        let snr = if min_e > 1e-15 {
            10.0 * (max_e / min_e).log10()
        } else {
            0.0
        };
        self.snr_accumulator += snr;
        self.stats.decoded_messages += 1;
        self.stats.average_snr = self.snr_accumulator / self.stats.decoded_messages as f64;

        TagMessage {
            bits,
            rssi,
            ber_estimate: ber,
        }
    }

    /// Compute a running-average envelope of the signal.
    ///
    /// Returns one averaged magnitude value per window position.
    pub fn envelope_average(&self, signal: &[Complex]) -> Vec<f64> {
        let window = self.config.averaging_window;
        if window == 0 || signal.len() < window {
            return Vec::new();
        }

        let magnitudes: Vec<f64> = signal
            .iter()
            .map(|&(re, im)| (re * re + im * im).sqrt())
            .collect();

        let mut result = Vec::with_capacity(magnitudes.len() - window + 1);
        let mut running_sum: f64 = magnitudes[..window].iter().sum();
        result.push(running_sum / window as f64);

        for i in window..magnitudes.len() {
            running_sum += magnitudes[i] - magnitudes[i - window];
            result.push(running_sum / window as f64);
        }

        result
    }

    /// Estimate and subtract the ambient carrier from the received signal.
    ///
    /// Uses a low-pass averaging approach: the ambient signal is estimated as
    /// the slowly varying component, and the residual (tag modulation) is the
    /// difference.
    pub fn separate_ambient(&self, signal: &[Complex]) -> Vec<Complex> {
        let window = self.config.averaging_window;
        if window == 0 || signal.is_empty() {
            return signal.to_vec();
        }

        let half = window / 2;
        let len = signal.len();
        let mut result = Vec::with_capacity(len);

        for i in 0..len {
            let start = if i >= half { i - half } else { 0 };
            let end = if i + half < len { i + half + 1 } else { len };
            let count = (end - start) as f64;

            let avg_re: f64 = signal[start..end].iter().map(|&(re, _)| re).sum::<f64>() / count;
            let avg_im: f64 = signal[start..end].iter().map(|&(_, im)| im).sum::<f64>() / count;

            result.push((signal[i].0 - avg_re, signal[i].1 - avg_im));
        }

        result
    }

    /// Estimate the tag-to-reader link margin in dB.
    ///
    /// # Parameters
    /// - `tx_power_dbm`: Ambient transmitter power in dBm
    /// - `tx_to_tag_distance_m`: Distance from ambient transmitter to tag in meters
    /// - `tag_to_reader_distance_m`: Distance from tag to reader in meters
    /// - `frequency_hz`: Carrier frequency in Hz
    /// - `tag_reflection_coeff`: Tag reflection coefficient (0.0 to 1.0)
    /// - `reader_sensitivity_dbm`: Reader sensitivity in dBm
    pub fn tag_link_budget(
        tx_power_dbm: f64,
        tx_to_tag_distance_m: f64,
        tag_to_reader_distance_m: f64,
        frequency_hz: f64,
        tag_reflection_coeff: f64,
        reader_sensitivity_dbm: f64,
    ) -> f64 {
        let c = 299_792_458.0; // speed of light in m/s
        let wavelength = c / frequency_hz;

        // Free-space path loss (dB) from TX to tag
        let fspl_tx_tag = if tx_to_tag_distance_m > 0.0 {
            20.0 * (4.0 * PI * tx_to_tag_distance_m / wavelength).log10()
        } else {
            0.0
        };

        // Free-space path loss (dB) from tag to reader
        let fspl_tag_reader = if tag_to_reader_distance_m > 0.0 {
            20.0 * (4.0 * PI * tag_to_reader_distance_m / wavelength).log10()
        } else {
            0.0
        };

        // Reflection loss
        let reflection_loss_db = if tag_reflection_coeff > 0.0 {
            -20.0 * tag_reflection_coeff.log10()
        } else {
            f64::INFINITY
        };

        let received_power_dbm =
            tx_power_dbm - fspl_tx_tag - reflection_loss_db - fspl_tag_reader;

        // Link margin = received power - sensitivity
        received_power_dbm - reader_sensitivity_dbm
    }

    /// Generate a synthetic backscatter signal for testing.
    ///
    /// Creates an ambient carrier (sinusoid) and modulates it with the given
    /// tag bits using the configured modulation scheme.
    ///
    /// # Parameters
    /// - `tag_bits`: Bit sequence the tag transmits
    /// - `modulation_depth`: Depth of tag modulation (0.0 to 1.0)
    /// - `ambient_snr_db`: SNR of the ambient carrier in dB
    pub fn simulate_tag(
        &self,
        tag_bits: &[bool],
        modulation_depth: f64,
        ambient_snr_db: f64,
    ) -> Vec<Complex> {
        let samples_per_bit =
            (self.config.sample_rate / self.config.tag_data_rate).round() as usize;
        let total_samples = tag_bits.len() * samples_per_bit;

        // Deterministic PRNG for noise (simple LCG)
        let mut rng_state: u64 = 42;
        let mut next_rand = || -> f64 {
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            // Convert to uniform [0, 1)
            (rng_state >> 11) as f64 / (1u64 << 53) as f64
        };

        // Box-Muller for Gaussian noise
        let mut next_gaussian = || -> f64 {
            let u1 = next_rand().max(1e-15);
            let u2 = next_rand();
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        let noise_power = 10.0_f64.powf(-ambient_snr_db / 10.0);
        let noise_std = (noise_power / 2.0).sqrt();

        // Ambient carrier frequency (arbitrary, e.g., 100 kHz offset)
        let carrier_freq = 100_000.0;

        let mut signal = Vec::with_capacity(total_samples);

        for i in 0..total_samples {
            let t = i as f64 / self.config.sample_rate;

            // Ambient carrier
            let phase = 2.0 * PI * carrier_freq * t;
            let ambient_re = phase.cos();
            let ambient_im = phase.sin();

            // Tag modulation
            let bit_index = i / samples_per_bit;
            let tag_factor = match self.config.modulation {
                BackscatterModulation::Ook => {
                    if tag_bits[bit_index] {
                        1.0 + modulation_depth
                    } else {
                        1.0 - modulation_depth
                    }
                }
                BackscatterModulation::Fsk => {
                    let freq_offset = if tag_bits[bit_index] {
                        self.config.tag_data_rate
                    } else {
                        -self.config.tag_data_rate
                    };
                    let fsk_phase = 2.0 * PI * freq_offset * t;
                    // FSK modulation adds a small frequency-shifted component
                    1.0 + modulation_depth * fsk_phase.cos()
                }
            };

            // Apply tag modulation to ambient + noise
            let re = ambient_re * tag_factor + noise_std * next_gaussian();
            let im = ambient_im * tag_factor + noise_std * next_gaussian();

            signal.push((re, im));
        }

        signal
    }

    /// Estimate the bit error rate from decoded bit energy distribution.
    ///
    /// Uses the separation between high and low energy clusters relative to
    /// their spread to estimate BER via a Q-function approximation.
    pub fn ber_estimate(&self, bit_energies: &[f64], threshold: f64) -> f64 {
        if bit_energies.is_empty() {
            return 1.0;
        }

        let mut high_vals = Vec::new();
        let mut low_vals = Vec::new();

        for &e in bit_energies {
            if e > threshold {
                high_vals.push(e);
            } else {
                low_vals.push(e);
            }
        }

        if high_vals.is_empty() || low_vals.is_empty() {
            // All bits are the same; cannot estimate BER meaningfully
            return 0.5;
        }

        let mean_high = high_vals.iter().sum::<f64>() / high_vals.len() as f64;
        let mean_low = low_vals.iter().sum::<f64>() / low_vals.len() as f64;

        let var_high = high_vals
            .iter()
            .map(|&v| (v - mean_high) * (v - mean_high))
            .sum::<f64>()
            / high_vals.len() as f64;
        let var_low = low_vals
            .iter()
            .map(|&v| (v - mean_low) * (v - mean_low))
            .sum::<f64>()
            / low_vals.len() as f64;

        let std_avg = ((var_high + var_low) / 2.0).sqrt();
        if std_avg < 1e-15 {
            return 0.0; // Perfect separation
        }

        let separation = (mean_high - mean_low).abs();
        let q_arg = separation / (2.0 * std_avg);

        // Q-function approximation: Q(x) = 0.5 * erfc(x/sqrt(2))
        // For small x, use a simple bound
        if q_arg > 5.0 {
            0.0 // Essentially zero BER
        } else if q_arg < 0.1 {
            0.5 // Very poor separation
        } else {
            // Simple Q-function approximation
            let x = q_arg / std::f64::consts::SQRT_2;
            0.5 * erfc_approx(x)
        }
    }
}

/// Approximate complementary error function using Horner form of
/// Abramowitz and Stegun 7.1.26 (max error ~1.5e-7).
fn erfc_approx(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> AmbientBackscatterProcessor {
        AmbientBackscatterProcessor::new(BackscatterConfig::default())
    }

    fn make_processor(
        sample_rate: f64,
        tag_data_rate: f64,
        window: usize,
        threshold: f64,
    ) -> AmbientBackscatterProcessor {
        AmbientBackscatterProcessor::new(BackscatterConfig {
            sample_rate,
            tag_data_rate,
            averaging_window: window,
            detection_threshold: threshold,
            modulation: BackscatterModulation::Ook,
        })
    }

    #[test]
    fn test_envelope_average_basic() {
        let proc = make_processor(1000.0, 100.0, 3, 0.1);
        let signal: Vec<Complex> = vec![
            (1.0, 0.0),
            (2.0, 0.0),
            (3.0, 0.0),
            (4.0, 0.0),
            (5.0, 0.0),
        ];
        let env = proc.envelope_average(&signal);
        assert_eq!(env.len(), 3); // 5 - 3 + 1
        assert!((env[0] - 2.0).abs() < 1e-10); // avg(1,2,3)
        assert!((env[1] - 3.0).abs() < 1e-10); // avg(2,3,4)
        assert!((env[2] - 4.0).abs() < 1e-10); // avg(3,4,5)
    }

    #[test]
    fn test_envelope_average_empty() {
        let proc = make_processor(1000.0, 100.0, 10, 0.1);
        let signal: Vec<Complex> = vec![(1.0, 0.0), (2.0, 0.0)];
        let env = proc.envelope_average(&signal);
        assert!(env.is_empty());
    }

    #[test]
    fn test_detect_tag_with_modulated_signal() {
        let mut proc = make_processor(100_000.0, 1_000.0, 50, 0.05);
        let bits = vec![true, false, true, false, true, false, true, false];
        let signal = proc.simulate_tag(&bits, 0.4, 20.0);
        let detected = proc.detect_tag(&signal);
        assert!(detected, "Should detect tag with strong modulation");
    }

    #[test]
    fn test_detect_tag_pure_carrier_no_detection() {
        let mut proc = make_processor(100_000.0, 1_000.0, 50, 0.05);
        // Pure sinusoid with no tag modulation
        let n = 10_000;
        let signal: Vec<Complex> = (0..n)
            .map(|i| {
                let t = i as f64 / 100_000.0;
                let phase = 2.0 * PI * 1000.0 * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let detected = proc.detect_tag(&signal);
        assert!(!detected, "Should not detect tag on pure carrier");
    }

    #[test]
    fn test_decode_tag_recovers_bits() {
        let mut proc = make_processor(100_000.0, 1_000.0, 50, 0.05);
        let original_bits = vec![true, false, true, true, false, false, true, false];
        let signal = proc.simulate_tag(&original_bits, 0.5, 30.0);
        let message = proc.decode_tag(&signal);

        // With high SNR and strong modulation, most bits should be correct
        assert_eq!(message.bits.len(), original_bits.len());
        let correct = message
            .bits
            .iter()
            .zip(original_bits.iter())
            .filter(|(a, b)| a == b)
            .count();
        let accuracy = correct as f64 / original_bits.len() as f64;
        assert!(
            accuracy >= 0.75,
            "Expected >=75% accuracy, got {:.1}%",
            accuracy * 100.0
        );
    }

    #[test]
    fn test_separate_ambient_preserves_length() {
        let proc = default_processor();
        let signal: Vec<Complex> = (0..500)
            .map(|i| {
                let t = i as f64 / 1_000_000.0;
                (t.cos(), t.sin())
            })
            .collect();
        let residual = proc.separate_ambient(&signal);
        assert_eq!(residual.len(), signal.len());
    }

    #[test]
    fn test_separate_ambient_removes_dc() {
        let proc = make_processor(1000.0, 100.0, 5, 0.1);
        // Constant signal should yield near-zero residual
        let signal: Vec<Complex> = vec![(3.0, 2.0); 20];
        let residual = proc.separate_ambient(&signal);
        for &(re, im) in &residual[3..17] {
            // Interior points (away from boundary effects)
            assert!(
                re.abs() < 1e-10,
                "Residual real part should be ~0, got {}",
                re
            );
            assert!(
                im.abs() < 1e-10,
                "Residual imag part should be ~0, got {}",
                im
            );
        }
    }

    #[test]
    fn test_link_budget_positive_margin() {
        // Very close range backscatter (typical indoor tag scenario)
        let margin = AmbientBackscatterProcessor::tag_link_budget(
            30.0,  // 30 dBm TX power
            1.0,   // 1 m TX-to-tag
            1.0,   // 1 m tag-to-reader
            900e6, // 900 MHz cellular
            0.5,   // 50% reflection
            -80.0, // -80 dBm sensitivity
        );
        // Should have positive margin at very close range
        assert!(
            margin > 0.0,
            "Expected positive link margin at close range, got {:.1} dB",
            margin
        );
    }

    #[test]
    fn test_link_budget_decreases_with_distance() {
        let margin_close = AmbientBackscatterProcessor::tag_link_budget(
            20.0, 5.0, 2.0, 900e6, 0.3, -80.0,
        );
        let margin_far = AmbientBackscatterProcessor::tag_link_budget(
            20.0, 50.0, 20.0, 900e6, 0.3, -80.0,
        );
        assert!(
            margin_close > margin_far,
            "Close margin ({:.1}) should exceed far margin ({:.1})",
            margin_close,
            margin_far
        );
    }

    #[test]
    fn test_simulate_tag_output_length() {
        let proc = make_processor(10_000.0, 1_000.0, 10, 0.1);
        let bits = vec![true, false, true];
        let signal = proc.simulate_tag(&bits, 0.3, 10.0);
        let expected_len = 3 * 10; // 3 bits * (10000/1000) samples per bit
        assert_eq!(signal.len(), expected_len);
    }

    #[test]
    fn test_simulate_tag_fsk_mode() {
        let proc = AmbientBackscatterProcessor::new(BackscatterConfig {
            sample_rate: 10_000.0,
            tag_data_rate: 1_000.0,
            averaging_window: 5,
            detection_threshold: 0.1,
            modulation: BackscatterModulation::Fsk,
        });
        let bits = vec![true, false];
        let signal = proc.simulate_tag(&bits, 0.3, 20.0);
        assert_eq!(signal.len(), 20); // 2 bits * 10 samples/bit
        // All samples should be finite
        for &(re, im) in &signal {
            assert!(re.is_finite());
            assert!(im.is_finite());
        }
    }

    #[test]
    fn test_ber_estimate_good_separation() {
        let proc = default_processor();
        // Well-separated clusters
        let energies = vec![0.9, 0.1, 0.85, 0.15, 0.92, 0.08];
        let threshold = 0.5;
        let ber = proc.ber_estimate(&energies, threshold);
        assert!(
            ber < 0.01,
            "BER should be very low for well-separated clusters, got {}",
            ber
        );
    }

    #[test]
    fn test_ber_estimate_poor_separation() {
        let proc = default_processor();
        // Closely spaced clusters with noticeable variance
        let energies = vec![0.52, 0.48, 0.53, 0.47, 0.51, 0.49, 0.52, 0.48];
        let threshold = 0.50;
        let ber = proc.ber_estimate(&energies, threshold);
        assert!(
            ber > 0.001,
            "BER should be non-trivial for close clusters, got {}",
            ber
        );
    }

    #[test]
    fn test_statistics_update_after_decode() {
        let mut proc = make_processor(100_000.0, 1_000.0, 30, 0.05);
        assert_eq!(proc.statistics().decoded_messages, 0);

        let bits = vec![true, false, true, false];
        let signal = proc.simulate_tag(&bits, 0.4, 20.0);
        let _msg = proc.decode_tag(&signal);

        assert_eq!(proc.statistics().decoded_messages, 1);
        assert!(proc.statistics().average_snr.is_finite());
    }

    #[test]
    fn test_detection_rate_tracking() {
        let mut proc = make_processor(100_000.0, 1_000.0, 50, 0.05);

        // Modulated signal should be detected
        let bits = vec![true, false, true, false, true, false];
        let signal = proc.simulate_tag(&bits, 0.5, 20.0);
        proc.detect_tag(&signal);

        // Pure carrier should not
        let carrier: Vec<Complex> = (0..10_000)
            .map(|i| {
                let phase = 2.0 * PI * 1000.0 * i as f64 / 100_000.0;
                (phase.cos(), phase.sin())
            })
            .collect();
        proc.detect_tag(&carrier);

        // Detection rate should be between 0 and 1
        let rate = proc.statistics().detection_rate;
        assert!(
            (0.0..=1.0).contains(&rate),
            "Detection rate should be in [0, 1], got {}",
            rate
        );
    }

    #[test]
    fn test_erfc_approx_known_values() {
        // erfc(0) = 1.0
        assert!((erfc_approx(0.0) - 1.0).abs() < 1e-5);
        // erfc is decreasing
        assert!(erfc_approx(1.0) < erfc_approx(0.5));
        // erfc(large) ~ 0
        assert!(erfc_approx(5.0) < 1e-6);
        // erfc(-x) = 2 - erfc(x)
        let val = erfc_approx(1.0);
        let neg_val = erfc_approx(-1.0);
        assert!((neg_val - (2.0 - val)).abs() < 1e-5);
    }

    #[test]
    fn test_default_config() {
        let cfg = BackscatterConfig::default();
        assert_eq!(cfg.sample_rate, 1_000_000.0);
        assert_eq!(cfg.tag_data_rate, 1_000.0);
        assert_eq!(cfg.averaging_window, 100);
        assert_eq!(cfg.detection_threshold, 0.15);
        assert_eq!(cfg.modulation, BackscatterModulation::Ook);
    }
}
