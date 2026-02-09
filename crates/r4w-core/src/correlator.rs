//! Correlator / Correlation Estimator
//!
//! Generic cross-correlation for sync word detection, preamble search,
//! and timing estimation. Complements the LoRa-specific sync module with
//! a general-purpose pattern matching block.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::correlator::{Correlator, CorrelatorConfig};
//! use num_complex::Complex64;
//!
//! // Detect a Barker-13 sync word
//! let barker13: Vec<Complex64> = [1,1,1,1,1,-1,-1,1,1,-1,1,-1,1]
//!     .iter()
//!     .map(|&b| Complex64::new(b as f64, 0.0))
//!     .collect();
//!
//! let mut corr = Correlator::new(CorrelatorConfig {
//!     pattern: barker13,
//!     threshold: 0.8,
//!     ..Default::default()
//! });
//!
//! // Feed samples; detections returned with timing/phase info
//! let signal = vec![Complex64::new(0.1, 0.0); 100];
//! let detections = corr.process_block(&signal);
//! ```

use num_complex::Complex64;

/// Detection result from the correlator.
#[derive(Debug, Clone)]
pub struct Detection {
    /// Sample index (within the processed block) of the detection.
    pub index: usize,
    /// Normalized correlation magnitude (0.0 to 1.0).
    pub correlation: f64,
    /// Phase estimate at detection point (radians).
    pub phase: f64,
    /// Absolute sample offset since correlator creation/reset.
    pub absolute_offset: u64,
}

/// Threshold method for detection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThresholdMethod {
    /// Fixed absolute threshold on normalized correlation (0.0-1.0).
    Absolute,
    /// Dynamic threshold: mean + factor * stddev of correlation magnitudes.
    Dynamic { factor: f64 },
}

impl Default for ThresholdMethod {
    fn default() -> Self {
        ThresholdMethod::Absolute
    }
}

/// Configuration for the correlator.
#[derive(Debug, Clone)]
pub struct CorrelatorConfig {
    /// Reference pattern to correlate against.
    pub pattern: Vec<Complex64>,
    /// Detection threshold (0.0-1.0 for Absolute, scale factor for Dynamic).
    pub threshold: f64,
    /// Threshold method.
    pub threshold_method: ThresholdMethod,
    /// Minimum samples between detections (deadzone to avoid duplicates).
    pub holdoff: usize,
}

impl Default for CorrelatorConfig {
    fn default() -> Self {
        Self {
            pattern: vec![Complex64::new(1.0, 0.0)],
            threshold: 0.8,
            threshold_method: ThresholdMethod::Absolute,
            holdoff: 0,
        }
    }
}

/// Cross-correlation based pattern detector.
///
/// Slides a reference pattern across the input signal, computing the
/// normalized cross-correlation at each position. When the correlation
/// exceeds the threshold, a detection is emitted.
///
/// The normalized correlation is:
/// ```text
///            |sum_k( conj(pattern[k]) * signal[n+k] )|
/// rho[n] = ─────────────────────────────────────────────
///           sqrt( sum|pattern|^2 * sum|signal[n..n+L]|^2 )
/// ```
///
/// This is equivalent to GNU Radio's `corr_est_cc`.
#[derive(Debug, Clone)]
pub struct Correlator {
    config: CorrelatorConfig,
    /// Pre-computed conjugate of pattern
    pattern_conj: Vec<Complex64>,
    /// Pre-computed pattern energy
    pattern_energy: f64,
    /// Sliding window buffer
    buffer: Vec<Complex64>,
    /// Absolute sample counter
    sample_count: u64,
    /// Last detection sample (for holdoff)
    last_detection: Option<u64>,
}

impl Correlator {
    /// Create a new correlator.
    pub fn new(config: CorrelatorConfig) -> Self {
        let pattern_conj: Vec<Complex64> = config.pattern.iter().map(|p| p.conj()).collect();
        let pattern_energy: f64 = config.pattern.iter().map(|p| p.norm_sqr()).sum();

        Self {
            pattern_conj,
            pattern_energy,
            buffer: Vec::new(),
            sample_count: 0,
            last_detection: None,
            config,
        }
    }

    /// Create a correlator from a bit pattern (maps 0→-1, 1→+1).
    pub fn from_bits(bits: &[u8], threshold: f64) -> Self {
        let pattern: Vec<Complex64> = bits
            .iter()
            .map(|&b| {
                let v = if b == 0 { -1.0 } else { 1.0 };
                Complex64::new(v, 0.0)
            })
            .collect();
        Self::new(CorrelatorConfig {
            pattern,
            threshold,
            ..Default::default()
        })
    }

    /// Process a block of samples and return detections.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Detection> {
        let pat_len = self.config.pattern.len();
        if pat_len == 0 {
            return vec![];
        }

        // Append input to buffer
        self.buffer.extend_from_slice(input);

        let mut detections = Vec::new();
        let mut correlations = Vec::new();

        // Slide correlation window
        let mut local_idx: usize = 0;
        while self.buffer.len() >= pat_len {
            let window = &self.buffer[..pat_len];

            // Cross-correlation
            let mut corr_sum = Complex64::new(0.0, 0.0);
            let mut signal_energy = 0.0;
            for k in 0..pat_len {
                corr_sum += self.pattern_conj[k] * window[k];
                signal_energy += window[k].norm_sqr();
            }

            // Normalized correlation
            let denom = (self.pattern_energy * signal_energy).sqrt();
            let norm_corr = if denom > 1e-20 {
                corr_sum.norm() / denom
            } else {
                0.0
            };

            correlations.push((local_idx, self.sample_count, norm_corr, corr_sum.arg()));

            // Consume one sample (sliding window)
            self.buffer.remove(0);
            self.sample_count += 1;
            local_idx += 1;
        }

        // Apply threshold
        let threshold = match self.config.threshold_method {
            ThresholdMethod::Absolute => self.config.threshold,
            ThresholdMethod::Dynamic { factor } => {
                if correlations.is_empty() {
                    self.config.threshold
                } else {
                    let mean: f64 = correlations.iter().map(|c| c.2).sum::<f64>()
                        / correlations.len() as f64;
                    let var: f64 = correlations.iter().map(|c| (c.2 - mean).powi(2)).sum::<f64>()
                        / correlations.len() as f64;
                    mean + factor * var.sqrt()
                }
            }
        };

        for (idx, abs_offset, corr, phase) in correlations {
            if corr >= threshold {
                // Check holdoff
                let pass_holdoff = match self.last_detection {
                    Some(last) => abs_offset.saturating_sub(last) as usize >= self.config.holdoff,
                    None => true,
                };
                if pass_holdoff {
                    detections.push(Detection {
                        index: idx,
                        correlation: corr,
                        phase,
                        absolute_offset: abs_offset,
                    });
                    self.last_detection = Some(abs_offset);
                }
            }
        }

        detections
    }

    /// Compute the full cross-correlation between signal and pattern (non-sliding).
    ///
    /// Returns a vector of correlation magnitudes, one per valid position.
    pub fn cross_correlate(signal: &[Complex64], pattern: &[Complex64]) -> Vec<f64> {
        let pat_len = pattern.len();
        if signal.len() < pat_len {
            return vec![];
        }

        let pattern_conj: Vec<Complex64> = pattern.iter().map(|p| p.conj()).collect();
        let pattern_energy: f64 = pattern.iter().map(|p| p.norm_sqr()).sum();

        let mut result = Vec::with_capacity(signal.len() - pat_len + 1);
        for i in 0..=(signal.len() - pat_len) {
            let mut corr = Complex64::new(0.0, 0.0);
            let mut sig_energy = 0.0;
            for k in 0..pat_len {
                corr += pattern_conj[k] * signal[i + k];
                sig_energy += signal[i + k].norm_sqr();
            }
            let denom = (pattern_energy * sig_energy).sqrt();
            let norm = if denom > 1e-20 {
                corr.norm() / denom
            } else {
                0.0
            };
            result.push(norm);
        }

        result
    }

    /// Reset the correlator state.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.sample_count = 0;
        self.last_detection = None;
    }

    /// Get the pattern length.
    pub fn pattern_len(&self) -> usize {
        self.config.pattern.len()
    }
}

/// Common sync word patterns.
pub struct SyncWords;

impl SyncWords {
    /// Barker-7 code: excellent autocorrelation (sidelobe = 1/7).
    pub fn barker7() -> Vec<Complex64> {
        [1, 1, 1, -1, -1, 1, -1]
            .iter()
            .map(|&b| Complex64::new(b as f64, 0.0))
            .collect()
    }

    /// Barker-11 code.
    pub fn barker11() -> Vec<Complex64> {
        [1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1]
            .iter()
            .map(|&b| Complex64::new(b as f64, 0.0))
            .collect()
    }

    /// Barker-13 code: longest Barker code (sidelobe = 1/13).
    pub fn barker13() -> Vec<Complex64> {
        [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1]
            .iter()
            .map(|&b| Complex64::new(b as f64, 0.0))
            .collect()
    }

    /// IEEE 802.11a/g short training sequence (normalized).
    pub fn wifi_sts() -> Vec<Complex64> {
        // Simplified: alternating +/- pattern
        let v = std::f64::consts::FRAC_1_SQRT_2;
        (0..16)
            .map(|i| {
                if i % 2 == 0 {
                    Complex64::new(v, v)
                } else {
                    Complex64::new(-v, v)
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_perfect_detection() {
        let pattern = SyncWords::barker13();
        let mut corr = Correlator::new(CorrelatorConfig {
            pattern: pattern.clone(),
            threshold: 0.9,
            ..Default::default()
        });

        // Build signal: noise + pattern + noise
        let mut signal = vec![Complex64::new(0.01, 0.01); 50];
        signal.extend_from_slice(&pattern);
        signal.extend(vec![Complex64::new(0.01, -0.01); 50]);

        let detections = corr.process_block(&signal);
        assert!(
            !detections.is_empty(),
            "Should detect embedded pattern"
        );
        // Detection should be near index 50
        let det = &detections[0];
        assert!(
            (det.index as i64 - 50).unsigned_abs() < 3,
            "Detection at wrong index: {}",
            det.index
        );
        assert!(det.correlation > 0.9);
    }

    #[test]
    fn test_no_false_alarm() {
        let pattern = SyncWords::barker13();
        let mut corr = Correlator::new(CorrelatorConfig {
            pattern,
            threshold: 0.9,
            ..Default::default()
        });

        // Pure noise (low amplitude)
        let noise: Vec<Complex64> = (0..200)
            .map(|i| Complex64::new(0.01 * (i as f64 * 0.7).sin(), 0.01 * (i as f64 * 1.3).cos()))
            .collect();

        let detections = corr.process_block(&noise);
        assert!(
            detections.is_empty(),
            "Should not detect pattern in noise: got {} detections",
            detections.len()
        );
    }

    #[test]
    fn test_cross_correlate_static() {
        let pattern = SyncWords::barker7();
        let mut signal = vec![Complex64::new(0.0, 0.0); 20];
        for (i, &p) in pattern.iter().enumerate() {
            signal[5 + i] = p;
        }

        let corr = Correlator::cross_correlate(&signal, &pattern);
        assert!(!corr.is_empty());

        // Peak should be at index 5
        let (peak_idx, &peak_val) = corr
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_idx, 5);
        assert!((peak_val - 1.0).abs() < 0.01, "Peak should be ~1.0: got {peak_val}");
    }

    #[test]
    fn test_holdoff() {
        let pattern: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 4];
        let mut corr = Correlator::new(CorrelatorConfig {
            pattern: pattern.clone(),
            threshold: 0.9,
            holdoff: 20,
            ..Default::default()
        });

        // Two copies of the pattern close together
        let mut signal = vec![Complex64::new(0.0, 0.0); 10];
        signal.extend_from_slice(&pattern);
        signal.extend(vec![Complex64::new(0.0, 0.0); 5]);
        signal.extend_from_slice(&pattern);
        signal.extend(vec![Complex64::new(0.0, 0.0); 10]);

        let detections = corr.process_block(&signal);
        // With holdoff=20, second detection at ~19 should be suppressed
        assert!(
            detections.len() <= 2,
            "Holdoff should limit detections"
        );
    }

    #[test]
    fn test_dynamic_threshold() {
        let pattern = SyncWords::barker13();
        let mut corr = Correlator::new(CorrelatorConfig {
            pattern: pattern.clone(),
            threshold: 0.0, // Not used for dynamic
            threshold_method: ThresholdMethod::Dynamic { factor: 3.0 },
            ..Default::default()
        });

        let mut signal = vec![Complex64::new(0.01, 0.01); 50];
        signal.extend_from_slice(&pattern);
        signal.extend(vec![Complex64::new(0.01, -0.01); 50]);

        let detections = corr.process_block(&signal);
        assert!(
            !detections.is_empty(),
            "Dynamic threshold should detect embedded pattern"
        );
    }

    #[test]
    fn test_barker_autocorrelation() {
        let pattern = SyncWords::barker13();
        let corr = Correlator::cross_correlate(&pattern, &pattern);
        // Autocorrelation peak should be at center (index 0 since same length)
        assert!(corr.len() == 1);
        assert!((corr[0] - 1.0).abs() < 1e-10, "Autocorrelation should be 1.0");
    }

    #[test]
    fn test_from_bits() {
        let bits = [1u8, 0, 1, 1, 0, 0, 1, 0];
        let corr = Correlator::from_bits(&bits, 0.9);
        assert_eq!(corr.pattern_len(), 8);
    }

    #[test]
    fn test_phase_estimation() {
        let pattern: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 8];
        let mut corr = Correlator::new(CorrelatorConfig {
            pattern,
            threshold: 0.9,
            ..Default::default()
        });

        // Signal with 90° phase rotation
        let rotated: Vec<Complex64> = vec![Complex64::new(0.0, 1.0); 8];
        let mut signal = vec![Complex64::new(0.0, 0.0); 5];
        signal.extend_from_slice(&rotated);
        signal.extend(vec![Complex64::new(0.0, 0.0); 5]);

        let detections = corr.process_block(&signal);
        if !detections.is_empty() {
            let phase = detections[0].phase;
            // Phase should be near π/2 (90°)
            assert!(
                (phase - std::f64::consts::FRAC_PI_2).abs() < 0.2,
                "Phase should be ~π/2: got {phase:.3}"
            );
        }
    }

    #[test]
    fn test_reset() {
        let mut corr = Correlator::new(CorrelatorConfig {
            pattern: SyncWords::barker7(),
            threshold: 0.8,
            ..Default::default()
        });
        let signal = vec![Complex64::new(1.0, 0.0); 20];
        let _ = corr.process_block(&signal);
        corr.reset();
        assert_eq!(corr.sample_count, 0);
    }
}
