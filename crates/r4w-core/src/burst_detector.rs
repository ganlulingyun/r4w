//! Burst Detector
//!
//! Power-based signal detector for bursty transmissions (TDMA, packet radio).
//! Detects start-of-burst (SOB) and end-of-burst (EOB) with configurable
//! hysteresis to prevent chatter on noisy signals.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::burst_detector::{BurstDetector, BurstConfig, BurstEvent};
//! use num_complex::Complex64;
//!
//! let mut detector = BurstDetector::new(BurstConfig {
//!     high_threshold_db: -20.0,
//!     low_threshold_db: -30.0,
//!     attack_alpha: 0.1,
//!     release_alpha: 0.01,
//!     ..Default::default()
//! });
//!
//! // Process a loud signal
//! let burst: Vec<Complex64> = vec![Complex64::new(0.1, 0.0); 100];
//! let events = detector.process_block(&burst);
//! // Should detect start of burst
//! ```

use num_complex::Complex64;

/// Burst detector configuration.
#[derive(Debug, Clone)]
pub struct BurstConfig {
    /// High threshold in dBFS — power must exceed this to trigger burst start.
    pub high_threshold_db: f64,
    /// Low threshold in dBFS — power must drop below this to trigger burst end.
    /// Should be lower than `high_threshold_db` for hysteresis.
    pub low_threshold_db: f64,
    /// Attack smoothing factor (0-1). Higher = faster onset detection.
    pub attack_alpha: f64,
    /// Release smoothing factor (0-1). Lower = slower release (prevents chatter).
    pub release_alpha: f64,
    /// Minimum burst length in samples. Bursts shorter than this are ignored.
    pub min_burst_samples: usize,
    /// Holdoff in samples after end-of-burst before a new burst can start.
    pub holdoff_samples: usize,
}

impl Default for BurstConfig {
    fn default() -> Self {
        Self {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 0.1,
            release_alpha: 0.01,
            min_burst_samples: 10,
            holdoff_samples: 0,
        }
    }
}

/// Event emitted by the burst detector.
#[derive(Debug, Clone, PartialEq)]
pub enum BurstEvent {
    /// Burst started at this sample offset.
    StartOfBurst {
        offset: usize,
        power_db: f64,
    },
    /// Burst ended at this sample offset.
    EndOfBurst {
        offset: usize,
        power_db: f64,
        length: usize,
    },
}

/// Burst detector state.
#[derive(Debug, Clone, Copy, PartialEq)]
enum BurstState {
    /// Waiting for signal above high threshold.
    Idle,
    /// Signal is above threshold, burst in progress.
    Active,
    /// Holdoff period after burst end.
    Holdoff,
}

/// Power-based burst detector with hysteresis.
#[derive(Debug, Clone)]
pub struct BurstDetector {
    config: BurstConfig,
    /// Smoothed power estimate (linear)
    power_est: f64,
    /// Current state
    state: BurstState,
    /// Sample counter since burst start
    burst_length: usize,
    /// Sample counter for holdoff
    holdoff_counter: usize,
    /// Total samples processed
    total_samples: usize,
}

impl BurstDetector {
    /// Create a new burst detector.
    pub fn new(config: BurstConfig) -> Self {
        Self {
            config,
            power_est: 0.0,
            state: BurstState::Idle,
            burst_length: 0,
            holdoff_counter: 0,
            total_samples: 0,
        }
    }

    /// Process a single sample. Returns an event if a state transition occurs.
    pub fn process_sample(&mut self, sample: Complex64) -> Option<BurstEvent> {
        let power = sample.norm_sqr();

        // Asymmetric smoothing: fast attack, slow release
        let alpha = if power > self.power_est {
            self.config.attack_alpha
        } else {
            self.config.release_alpha
        };
        self.power_est = (1.0 - alpha) * self.power_est + alpha * power;

        let power_db = if self.power_est > 1e-20 {
            10.0 * self.power_est.log10()
        } else {
            -200.0
        };

        let offset = self.total_samples;
        self.total_samples += 1;

        match self.state {
            BurstState::Idle => {
                if power_db >= self.config.high_threshold_db {
                    self.state = BurstState::Active;
                    self.burst_length = 1;
                    return Some(BurstEvent::StartOfBurst { offset, power_db });
                }
            }
            BurstState::Active => {
                self.burst_length += 1;
                if power_db < self.config.low_threshold_db {
                    let length = self.burst_length;
                    self.burst_length = 0;

                    if length >= self.config.min_burst_samples {
                        if self.config.holdoff_samples > 0 {
                            self.state = BurstState::Holdoff;
                            self.holdoff_counter = self.config.holdoff_samples;
                        } else {
                            self.state = BurstState::Idle;
                        }
                        return Some(BurstEvent::EndOfBurst {
                            offset,
                            power_db,
                            length,
                        });
                    } else {
                        // Burst too short — ignore it
                        self.state = BurstState::Idle;
                    }
                }
            }
            BurstState::Holdoff => {
                self.holdoff_counter = self.holdoff_counter.saturating_sub(1);
                if self.holdoff_counter == 0 {
                    self.state = BurstState::Idle;
                }
            }
        }

        None
    }

    /// Process a block of samples, returning all burst events.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<BurstEvent> {
        let mut events = Vec::new();
        for &s in input {
            if let Some(event) = self.process_sample(s) {
                events.push(event);
            }
        }
        events
    }

    /// Gate signal: pass through during bursts, zero otherwise.
    pub fn gate(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input
            .iter()
            .map(|&s| {
                let _ = self.process_sample(s);
                if self.state == BurstState::Active {
                    s
                } else {
                    Complex64::new(0.0, 0.0)
                }
            })
            .collect()
    }

    /// Check if currently in a burst.
    pub fn is_active(&self) -> bool {
        self.state == BurstState::Active
    }

    /// Get the current smoothed power estimate in dB.
    pub fn power_db(&self) -> f64 {
        if self.power_est > 1e-20 {
            10.0 * self.power_est.log10()
        } else {
            -200.0
        }
    }

    /// Get current burst length (0 if not in burst).
    pub fn burst_length(&self) -> usize {
        if self.state == BurstState::Active {
            self.burst_length
        } else {
            0
        }
    }

    /// Reset detector state.
    pub fn reset(&mut self) {
        self.power_est = 0.0;
        self.state = BurstState::Idle;
        self.burst_length = 0;
        self.holdoff_counter = 0;
        self.total_samples = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_burst_signal(
        noise_len: usize,
        burst_len: usize,
        noise_level: f64,
        burst_level: f64,
    ) -> Vec<Complex64> {
        let mut signal = Vec::new();
        // Noise floor
        for _ in 0..noise_len {
            signal.push(Complex64::new(noise_level, 0.0));
        }
        // Burst
        for _ in 0..burst_len {
            signal.push(Complex64::new(burst_level, 0.0));
        }
        // More noise
        for _ in 0..noise_len {
            signal.push(Complex64::new(noise_level, 0.0));
        }
        signal
    }

    #[test]
    fn test_burst_detection() {
        let mut det = BurstDetector::new(BurstConfig {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 0.5,
            release_alpha: 0.5,
            min_burst_samples: 5,
            holdoff_samples: 0,
        });

        // Noise at -60 dBFS, burst at -10 dBFS
        let signal = make_burst_signal(50, 100, 0.001, 0.3);
        let events = det.process_block(&signal);

        // Should have at least SOB and EOB
        let sob_count = events
            .iter()
            .filter(|e| matches!(e, BurstEvent::StartOfBurst { .. }))
            .count();
        let eob_count = events
            .iter()
            .filter(|e| matches!(e, BurstEvent::EndOfBurst { .. }))
            .count();

        assert!(sob_count >= 1, "Should detect start of burst: events={events:?}");
        assert!(eob_count >= 1, "Should detect end of burst: events={events:?}");
    }

    #[test]
    fn test_no_burst_in_noise() {
        let mut det = BurstDetector::new(BurstConfig {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 0.1,
            release_alpha: 0.01,
            min_burst_samples: 5,
            holdoff_samples: 0,
        });

        // Only noise
        let noise: Vec<Complex64> = vec![Complex64::new(0.001, 0.0); 200];
        let events = det.process_block(&noise);
        assert!(events.is_empty(), "Should not detect burst in noise: events={events:?}");
    }

    #[test]
    fn test_hysteresis() {
        let config = BurstConfig {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 1.0,  // Instantaneous
            release_alpha: 1.0, // Instantaneous
            min_burst_samples: 1,
            holdoff_samples: 0,
        };
        let mut det = BurstDetector::new(config);

        // Signal at -25 dBFS (between high and low thresholds)
        // Should NOT trigger start (below high)
        let mid: Vec<Complex64> = vec![Complex64::new(0.056, 0.0); 100]; // ~-25 dBFS
        let events = det.process_block(&mid);
        assert!(events.is_empty(), "Mid-level signal should not trigger burst");

        // Now go high to start burst
        let high: Vec<Complex64> = vec![Complex64::new(0.3, 0.0); 10]; // ~-10 dBFS
        let events = det.process_block(&high);
        assert!(
            events.iter().any(|e| matches!(e, BurstEvent::StartOfBurst { .. })),
            "High signal should start burst"
        );
        assert!(det.is_active());

        // Back to mid level — should stay active (above low threshold)
        let mid: Vec<Complex64> = vec![Complex64::new(0.056, 0.0); 50];
        let events = det.process_block(&mid);
        // With instantaneous alpha, power drops fast, but -25 dB > -30 dB low threshold
        // So burst should still be active
        assert!(
            !events.iter().any(|e| matches!(e, BurstEvent::EndOfBurst { .. })),
            "Mid-level should maintain burst (hysteresis)"
        );
    }

    #[test]
    fn test_min_burst_length() {
        let mut det = BurstDetector::new(BurstConfig {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 1.0,
            release_alpha: 1.0,
            min_burst_samples: 50,
            holdoff_samples: 0,
        });

        // Short burst (only 5 samples)
        let signal = make_burst_signal(50, 5, 0.001, 0.3);
        let events = det.process_block(&signal);

        // Should not report EOB because burst was too short
        let eob_count = events
            .iter()
            .filter(|e| matches!(e, BurstEvent::EndOfBurst { .. }))
            .count();
        assert_eq!(eob_count, 0, "Short burst should be ignored");
    }

    #[test]
    fn test_gate_function() {
        let mut det = BurstDetector::new(BurstConfig {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 0.5,
            release_alpha: 0.5,
            min_burst_samples: 1,
            holdoff_samples: 0,
        });

        // Loud signal
        let loud: Vec<Complex64> = vec![Complex64::new(0.5, 0.0); 50];
        let gated = det.gate(&loud);

        // After power ramps up, signal should pass through
        let passed: usize = gated.iter().filter(|s| s.norm() > 0.1).count();
        assert!(passed > 30, "Gate should pass most of loud signal: passed={passed}");
    }

    #[test]
    fn test_burst_detector_reset() {
        let mut det = BurstDetector::new(BurstConfig::default());

        // Process some signal
        let signal: Vec<Complex64> = vec![Complex64::new(0.5, 0.0); 100];
        let _ = det.process_block(&signal);

        det.reset();
        assert!(!det.is_active());
        assert_eq!(det.burst_length(), 0);
    }

    #[test]
    fn test_holdoff() {
        let mut det = BurstDetector::new(BurstConfig {
            high_threshold_db: -20.0,
            low_threshold_db: -30.0,
            attack_alpha: 1.0,
            release_alpha: 1.0,
            min_burst_samples: 1,
            holdoff_samples: 100,
        });

        // First burst
        let high: Vec<Complex64> = vec![Complex64::new(0.3, 0.0); 20];
        let _ = det.process_block(&high);
        assert!(det.is_active());

        // Drop to noise (triggers EOB + holdoff)
        let noise: Vec<Complex64> = vec![Complex64::new(0.001, 0.0); 10];
        let events = det.process_block(&noise);
        assert!(events.iter().any(|e| matches!(e, BurstEvent::EndOfBurst { .. })));

        // During holdoff, a new burst should NOT start
        let high: Vec<Complex64> = vec![Complex64::new(0.3, 0.0); 20];
        let events = det.process_block(&high);
        assert!(
            !events.iter().any(|e| matches!(e, BurstEvent::StartOfBurst { .. })),
            "Holdoff should prevent new burst"
        );
    }
}
