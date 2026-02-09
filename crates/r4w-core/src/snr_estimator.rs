//! SNR / Noise Figure Estimator
//!
//! Estimates signal-to-noise ratio from received I/Q samples using
//! several blind estimation methods. Useful for adaptive modulation,
//! link quality monitoring, and receiver diagnostics.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::snr_estimator::{SnrEstimator, SnrMethod};
//! use num_complex::Complex64;
//!
//! let mut est = SnrEstimator::new(SnrMethod::M2M4);
//!
//! // Clean signal: SNR should be high
//! let clean: Vec<Complex64> = (0..1000)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * i as f64 / 10.0;
//!         Complex64::new(phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let snr_db = est.estimate(&clean);
//! assert!(snr_db > 20.0, "Clean signal should have high SNR");
//! ```

use num_complex::Complex64;

/// SNR estimation method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SnrMethod {
    /// M2M4 estimator: uses second and fourth moments.
    /// Works best for constant-modulus signals (PSK, FM).
    /// Ref: Pauluzzi & Beaulieu, "A comparison of SNR estimation techniques"
    M2M4,
    /// Split-spectrum estimator: divides spectrum into signal and noise regions.
    /// Works for narrowband signals in wideband captures.
    SplitSpectrum {
        /// Fraction of bandwidth occupied by signal (0.0-1.0).
        signal_bw_fraction: f64,
    },
    /// Signal-plus-noise / Noise-only (requires known noise-only segment).
    /// Most accurate when a noise-only reference is available.
    SignalPlusNoise,
    /// EVM-based estimator for known constellations.
    /// Measures scatter around ideal constellation points.
    Evm {
        /// Number of constellation points (2, 4, 8, 16, etc.)
        constellation_size: usize,
    },
}

/// SNR estimation result.
#[derive(Debug, Clone)]
pub struct SnrResult {
    /// Estimated SNR in dB.
    pub snr_db: f64,
    /// Estimated signal power (linear).
    pub signal_power: f64,
    /// Estimated noise power (linear).
    pub noise_power: f64,
    /// Estimation method used.
    pub method: SnrMethod,
}

/// SNR estimator.
#[derive(Debug, Clone)]
pub struct SnrEstimator {
    method: SnrMethod,
}

impl SnrEstimator {
    /// Create a new SNR estimator.
    pub fn new(method: SnrMethod) -> Self {
        Self { method }
    }

    /// Estimate SNR from received samples.
    pub fn estimate(&self, samples: &[Complex64]) -> f64 {
        self.estimate_detailed(samples).snr_db
    }

    /// Estimate SNR with detailed results.
    pub fn estimate_detailed(&self, samples: &[Complex64]) -> SnrResult {
        match self.method {
            SnrMethod::M2M4 => self.m2m4_estimate(samples),
            SnrMethod::SplitSpectrum { signal_bw_fraction } => {
                self.split_spectrum_estimate(samples, signal_bw_fraction)
            }
            SnrMethod::SignalPlusNoise => {
                // Use the first half as signal, second half as reference
                // (caller should provide signal-only for real use)
                let mid = samples.len() / 2;
                self.signal_plus_noise_estimate(&samples[..mid], &samples[mid..])
            }
            SnrMethod::Evm { constellation_size } => {
                self.evm_estimate(samples, constellation_size)
            }
        }
    }

    /// Estimate SNR for signal+noise vs noise-only reference.
    pub fn estimate_with_reference(
        &self,
        signal_plus_noise: &[Complex64],
        noise_only: &[Complex64],
    ) -> SnrResult {
        self.signal_plus_noise_estimate(signal_plus_noise, noise_only)
    }

    /// M2M4 estimator.
    ///
    /// Uses the ratio of second and fourth moments to separate signal and noise.
    /// For constant-modulus signals (PSK): kurtosis of signal = 1.
    ///
    /// ```text
    /// M2 = E[|x|^2] = S + N
    /// M4 = E[|x|^4] = S^2 * kappa_s + 2*S*N + N^2
    /// ```
    ///
    /// For constant-modulus: kappa_s = 1, solving gives:
    /// ```text
    /// N = M2 - sqrt(2*M2^2 - M4)
    /// S = M2 - N
    /// ```
    fn m2m4_estimate(&self, samples: &[Complex64]) -> SnrResult {
        let n = samples.len() as f64;
        if n < 2.0 {
            return SnrResult {
                snr_db: 0.0,
                signal_power: 0.0,
                noise_power: 0.0,
                method: self.method,
            };
        }

        let m2: f64 = samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / n;
        let m4: f64 = samples
            .iter()
            .map(|s| {
                let p = s.norm_sqr();
                p * p
            })
            .sum::<f64>()
            / n;

        // For constant-modulus (kappa_s = 1):
        // N = M2 - sqrt(2*M2^2 - M4)
        let discriminant = 2.0 * m2 * m2 - m4;
        let noise_power = if discriminant > 0.0 {
            (m2 - discriminant.sqrt()).max(0.0)
        } else {
            // Fallback: assume high SNR
            m2 * 0.01
        };

        let signal_power = (m2 - noise_power).max(1e-30);

        let snr_db = if noise_power > 1e-30 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0 // Effectively infinite SNR
        };

        SnrResult {
            snr_db,
            signal_power,
            noise_power,
            method: self.method,
        }
    }

    /// Split-spectrum estimator.
    ///
    /// Computes FFT, identifies signal region (center fraction), and noise
    /// region (edges). SNR = signal_power / noise_power.
    fn split_spectrum_estimate(&self, samples: &[Complex64], bw_fraction: f64) -> SnrResult {
        let n = samples.len();
        if n < 4 {
            return SnrResult {
                snr_db: 0.0,
                signal_power: 0.0,
                noise_power: 0.0,
                method: self.method,
            };
        }

        // Simple DFT (for small blocks; would use FFT for production)
        let mut spectrum = vec![0.0f64; n];
        for k in 0..n {
            let mut sum = Complex64::new(0.0, 0.0);
            for i in 0..n {
                let angle = -2.0 * std::f64::consts::PI * k as f64 * i as f64 / n as f64;
                let twiddle = Complex64::new(angle.cos(), angle.sin());
                sum += samples[i] * twiddle;
            }
            spectrum[k] = sum.norm_sqr() / n as f64;
        }

        // Signal bins: center fraction
        let signal_bins = (n as f64 * bw_fraction).max(1.0) as usize;
        let start = (n - signal_bins) / 2;
        let end = start + signal_bins;

        let signal_power: f64 = spectrum[start..end].iter().sum::<f64>() / signal_bins as f64;

        // Noise bins: edges
        let noise_bins = n - signal_bins;
        let noise_power: f64 = if noise_bins > 0 {
            let noise_sum: f64 = spectrum[..start].iter().sum::<f64>()
                + spectrum[end..].iter().sum::<f64>();
            noise_sum / noise_bins as f64
        } else {
            1e-30
        };

        let snr_db = if noise_power > 1e-30 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0
        };

        SnrResult {
            snr_db,
            signal_power,
            noise_power,
            method: self.method,
        }
    }

    /// Signal-plus-noise vs noise-only estimator.
    fn signal_plus_noise_estimate(
        &self,
        signal_plus_noise: &[Complex64],
        noise_only: &[Complex64],
    ) -> SnrResult {
        let spn_power: f64 = signal_plus_noise.iter().map(|s| s.norm_sqr()).sum::<f64>()
            / signal_plus_noise.len().max(1) as f64;
        let noise_power: f64 = noise_only.iter().map(|s| s.norm_sqr()).sum::<f64>()
            / noise_only.len().max(1) as f64;

        let signal_power = (spn_power - noise_power).max(1e-30);

        let snr_db = if noise_power > 1e-30 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0
        };

        SnrResult {
            snr_db,
            signal_power,
            noise_power,
            method: SnrMethod::SignalPlusNoise,
        }
    }

    /// EVM-based SNR estimator.
    ///
    /// Places received symbols on the nearest constellation point and measures
    /// the error vector magnitude. SNR ≈ 1/EVM^2.
    fn evm_estimate(&self, samples: &[Complex64], constellation_size: usize) -> SnrResult {
        if samples.is_empty() || constellation_size == 0 {
            return SnrResult {
                snr_db: 0.0,
                signal_power: 0.0,
                noise_power: 0.0,
                method: self.method,
            };
        }

        // Generate constellation points (M-PSK)
        let constellation: Vec<Complex64> = (0..constellation_size)
            .map(|i| {
                let angle =
                    2.0 * std::f64::consts::PI * i as f64 / constellation_size as f64;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();

        let mut total_signal_power = 0.0;
        let mut total_error_power = 0.0;

        for s in samples {
            // Normalize to unit circle
            let norm = s.norm();
            if norm < 1e-10 {
                continue;
            }
            let normalized = s / norm;

            // Find nearest constellation point
            let (_, nearest) = constellation
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    (normalized - *a)
                        .norm()
                        .partial_cmp(&(normalized - *b).norm())
                        .unwrap()
                })
                .unwrap();

            let ideal = *nearest * norm;
            let error = s - ideal;

            total_signal_power += ideal.norm_sqr();
            total_error_power += error.norm_sqr();
        }

        let n = samples.len() as f64;
        let signal_power = total_signal_power / n;
        let noise_power = (total_error_power / n).max(1e-30);

        let snr_db = if noise_power > 1e-30 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            100.0
        };

        SnrResult {
            snr_db,
            signal_power,
            noise_power,
            method: self.method,
        }
    }
}

/// Quick SNR estimation using M2M4 method.
pub fn estimate_snr(samples: &[Complex64]) -> f64 {
    SnrEstimator::new(SnrMethod::M2M4).estimate(samples)
}

/// Quick noise power estimation.
pub fn estimate_noise_power(samples: &[Complex64]) -> f64 {
    SnrEstimator::new(SnrMethod::M2M4)
        .estimate_detailed(samples)
        .noise_power
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_psk_signal(snr_db: f64, num_samples: usize) -> Vec<Complex64> {
        let signal_power = 1.0;
        let noise_power = signal_power * 10.0f64.powf(-snr_db / 10.0);
        let noise_sigma = (noise_power / 2.0).sqrt();

        // Simple LCG for reproducible noise
        let mut rng = 0x12345678u64;
        let next_rng = |state: &mut u64| -> f64 {
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (*state >> 11) as f64 / (1u64 << 53) as f64
        };

        (0..num_samples)
            .map(|i| {
                // QPSK symbol
                let phase = std::f64::consts::FRAC_PI_4
                    + std::f64::consts::FRAC_PI_2 * (i % 4) as f64;
                let signal = Complex64::new(phase.cos(), phase.sin());

                // Gaussian noise via Box-Muller
                let u1 = next_rng(&mut rng).max(1e-20);
                let u2 = next_rng(&mut rng);
                let r = (-2.0 * u1.ln()).sqrt();
                let theta = 2.0 * std::f64::consts::PI * u2;
                let noise = Complex64::new(
                    noise_sigma * r * theta.cos(),
                    noise_sigma * r * theta.sin(),
                );

                signal + noise
            })
            .collect()
    }

    #[test]
    fn test_m2m4_high_snr() {
        let samples = make_psk_signal(30.0, 10000);
        let est = SnrEstimator::new(SnrMethod::M2M4);
        let result = est.estimate_detailed(&samples);

        assert!(
            result.snr_db > 20.0,
            "High SNR signal should estimate > 20 dB: got {:.1}",
            result.snr_db
        );
    }

    #[test]
    fn test_m2m4_low_snr() {
        let samples = make_psk_signal(3.0, 10000);
        let est = SnrEstimator::new(SnrMethod::M2M4);
        let result = est.estimate_detailed(&samples);

        assert!(
            result.snr_db > 0.0 && result.snr_db < 15.0,
            "Low SNR signal (~3 dB) should estimate in range: got {:.1}",
            result.snr_db
        );
    }

    #[test]
    fn test_clean_signal_high_snr() {
        // Pure tone, no noise
        let samples: Vec<Complex64> = (0..1000)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 10.0;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let snr = estimate_snr(&samples);
        assert!(
            snr > 30.0,
            "Clean signal should have very high SNR: got {snr:.1}"
        );
    }

    #[test]
    fn test_signal_plus_noise_method() {
        let signal: Vec<Complex64> = (0..500)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 10.0;
                Complex64::new(phase.cos() + 0.1, phase.sin() + 0.1) // signal + some noise
            })
            .collect();

        let noise: Vec<Complex64> = (0..500)
            .map(|i| Complex64::new(0.1 * (i as f64 * 0.7).sin(), 0.1 * (i as f64 * 1.3).cos()))
            .collect();

        let est = SnrEstimator::new(SnrMethod::SignalPlusNoise);
        let result = est.estimate_with_reference(&signal, &noise);

        assert!(
            result.snr_db > 5.0,
            "Signal+noise method should estimate positive SNR: got {:.1}",
            result.snr_db
        );
    }

    #[test]
    fn test_evm_clean_qpsk() {
        // Perfect QPSK aligned with EVM reference constellation (0, pi/2, pi, 3pi/2)
        let samples: Vec<Complex64> = (0..400)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * (i % 4) as f64 / 4.0;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();

        let est = SnrEstimator::new(SnrMethod::Evm {
            constellation_size: 4,
        });
        let result = est.estimate_detailed(&samples);

        assert!(
            result.snr_db > 50.0,
            "Perfect QPSK should have very high SNR: got {:.1}",
            result.snr_db
        );
    }

    #[test]
    fn test_estimate_noise_power() {
        let samples = make_psk_signal(10.0, 5000);
        let noise_pwr = estimate_noise_power(&samples);

        // At 10 dB SNR with unit signal, noise power should be ~0.1
        assert!(
            noise_pwr > 0.01 && noise_pwr < 1.0,
            "Noise power estimate should be reasonable: got {noise_pwr:.4}"
        );
    }

    #[test]
    fn test_quick_estimate() {
        let samples = make_psk_signal(20.0, 5000);
        let snr = estimate_snr(&samples);
        assert!(snr > 10.0, "Quick SNR estimate should be positive: got {snr:.1}");
    }

    #[test]
    fn test_split_spectrum() {
        let n = 256;
        // Tone at bin n/2 (center of DFT output) with noise floor
        // freq = n/2 * sr/n = sr/2 ... but that's Nyquist. Let's use bin near center.
        let signal_bin = n / 2;
        let mut rng = 0x9876u64;
        let samples: Vec<Complex64> = (0..n)
            .map(|i| {
                // Tone at target bin
                let phase = 2.0 * std::f64::consts::PI * signal_bin as f64 * i as f64 / n as f64;
                let signal = Complex64::new(phase.cos(), phase.sin());
                // Add small noise
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let ni = (rng >> 11) as f64 / (1u64 << 53) as f64 * 0.02 - 0.01;
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let nq = (rng >> 11) as f64 / (1u64 << 53) as f64 * 0.02 - 0.01;
                signal + Complex64::new(ni, nq)
            })
            .collect();

        let est = SnrEstimator::new(SnrMethod::SplitSpectrum {
            signal_bw_fraction: 0.1,
        });
        let result = est.estimate_detailed(&samples);

        assert!(
            result.snr_db > 10.0,
            "Narrowband tone should have high split-spectrum SNR: got {:.1}",
            result.snr_db
        );
    }
}
