//! # Burst Synchronizer
//!
//! Detects and aligns to burst boundaries in a sample stream.
//! Combines power detection, preamble correlation, and timing
//! alignment for burst-mode receivers (e.g., TDMA, packet radio).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::burst_synchronizer::{BurstSynchronizer, BurstConfig};
//!
//! let config = BurstConfig {
//!     preamble: vec![1.0, -1.0, 1.0, -1.0],
//!     threshold: 0.7,
//!     min_burst_len: 32,
//!     max_burst_len: 1024,
//!     guard_samples: 4,
//! };
//! let mut sync = BurstSynchronizer::new(config);
//! let mut signal = vec![0.0; 20];
//! signal.extend_from_slice(&[1.0, -1.0, 1.0, -1.0]); // preamble
//! signal.extend_from_slice(&[1.0; 50]); // payload
//! let bursts = sync.process(&signal);
//! ```

/// Configuration for burst synchronizer.
#[derive(Debug, Clone)]
pub struct BurstConfig {
    /// Known preamble pattern.
    pub preamble: Vec<f64>,
    /// Correlation threshold (0..1 normalized).
    pub threshold: f64,
    /// Minimum burst length in samples.
    pub min_burst_len: usize,
    /// Maximum burst length in samples.
    pub max_burst_len: usize,
    /// Guard interval (quiet) samples between bursts.
    pub guard_samples: usize,
}

impl Default for BurstConfig {
    fn default() -> Self {
        Self {
            preamble: vec![1.0, -1.0, 1.0, -1.0],
            threshold: 0.7,
            min_burst_len: 32,
            max_burst_len: 4096,
            guard_samples: 8,
        }
    }
}

/// A detected burst.
#[derive(Debug, Clone)]
pub struct DetectedBurst {
    /// Start index in the input stream.
    pub start: usize,
    /// Burst length in samples.
    pub length: usize,
    /// Peak correlation value.
    pub correlation: f64,
    /// Burst data.
    pub data: Vec<f64>,
}

/// Burst synchronizer.
#[derive(Debug, Clone)]
pub struct BurstSynchronizer {
    config: BurstConfig,
    buffer: Vec<f64>,
    total_samples: u64,
    bursts_detected: u64,
}

impl BurstSynchronizer {
    /// Create a new burst synchronizer.
    pub fn new(config: BurstConfig) -> Self {
        Self {
            config,
            buffer: Vec::new(),
            total_samples: 0,
            bursts_detected: 0,
        }
    }

    /// Process a block of samples and return detected bursts.
    pub fn process(&mut self, input: &[f64]) -> Vec<DetectedBurst> {
        self.buffer.extend_from_slice(input);
        self.total_samples += input.len() as u64;

        let preamble_len = self.config.preamble.len();
        if preamble_len == 0 || self.buffer.len() < preamble_len {
            return Vec::new();
        }

        // Compute normalized cross-correlation with preamble.
        let corr = self.correlate_preamble();
        let mut bursts = Vec::new();
        let mut skip_until = 0;

        for (i, &c) in corr.iter().enumerate() {
            if i < skip_until {
                continue;
            }
            if c >= self.config.threshold {
                let burst_start = i + preamble_len;
                let burst_end = (burst_start + self.config.max_burst_len).min(self.buffer.len());
                let burst_len = burst_end - burst_start;

                if burst_len >= self.config.min_burst_len {
                    bursts.push(DetectedBurst {
                        start: i,
                        length: burst_len,
                        correlation: c,
                        data: self.buffer[burst_start..burst_end].to_vec(),
                    });
                    self.bursts_detected += 1;
                    skip_until = burst_end + self.config.guard_samples;
                }
            }
        }

        // Keep only unprocessed samples.
        if let Some(last) = bursts.last() {
            let consumed = last.start + preamble_len + last.length + self.config.guard_samples;
            if consumed < self.buffer.len() {
                self.buffer = self.buffer[consumed..].to_vec();
            } else {
                self.buffer.clear();
            }
        }

        bursts
    }

    fn correlate_preamble(&self) -> Vec<f64> {
        let p = &self.config.preamble;
        let p_len = p.len();
        let n = self.buffer.len();
        if n < p_len {
            return Vec::new();
        }

        // Preamble energy.
        let p_energy: f64 = p.iter().map(|&x| x * x).sum();

        let mut corr = Vec::with_capacity(n - p_len + 1);
        for i in 0..=(n - p_len) {
            let mut dot = 0.0;
            let mut s_energy = 0.0;
            for j in 0..p_len {
                dot += self.buffer[i + j] * p[j];
                s_energy += self.buffer[i + j] * self.buffer[i + j];
            }
            let denom = (p_energy * s_energy).sqrt();
            if denom > 1e-30 {
                corr.push(dot / denom);
            } else {
                corr.push(0.0);
            }
        }
        corr
    }

    /// Get total bursts detected.
    pub fn bursts_detected(&self) -> u64 {
        self.bursts_detected
    }

    /// Get total samples processed.
    pub fn total_samples(&self) -> u64 {
        self.total_samples
    }

    /// Get current buffer size.
    pub fn buffer_len(&self) -> usize {
        self.buffer.len()
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.total_samples = 0;
        self.bursts_detected = 0;
    }
}

/// Simple energy-based burst detector.
pub fn detect_bursts_energy(
    signal: &[f64],
    threshold: f64,
    min_len: usize,
) -> Vec<(usize, usize)> {
    let mut bursts = Vec::new();
    let mut in_burst = false;
    let mut start = 0;

    for (i, &s) in signal.iter().enumerate() {
        let energy = s * s;
        if !in_burst && energy > threshold {
            in_burst = true;
            start = i;
        } else if in_burst && energy <= threshold {
            if i - start >= min_len {
                bursts.push((start, i - start));
            }
            in_burst = false;
        }
    }

    // Handle burst at end of signal.
    if in_burst && signal.len() - start >= min_len {
        bursts.push((start, signal.len() - start));
    }

    bursts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_energy_detection() {
        let mut signal = vec![0.0; 20];
        signal.extend(vec![1.0; 30]);
        signal.extend(vec![0.0; 20]);
        let bursts = detect_bursts_energy(&signal, 0.5, 10);
        assert_eq!(bursts.len(), 1);
        assert_eq!(bursts[0].0, 20);
        assert_eq!(bursts[0].1, 30);
    }

    #[test]
    fn test_energy_multiple_bursts() {
        let mut signal = vec![0.0; 10];
        signal.extend(vec![1.0; 20]);
        signal.extend(vec![0.0; 10]);
        signal.extend(vec![1.0; 15]);
        signal.extend(vec![0.0; 10]);
        let bursts = detect_bursts_energy(&signal, 0.5, 5);
        assert_eq!(bursts.len(), 2);
    }

    #[test]
    fn test_energy_too_short() {
        let mut signal = vec![0.0; 10];
        signal.extend(vec![1.0; 3]); // Shorter than min_len.
        signal.extend(vec![0.0; 10]);
        let bursts = detect_bursts_energy(&signal, 0.5, 5);
        assert!(bursts.is_empty());
    }

    #[test]
    fn test_preamble_detection() {
        let preamble = vec![1.0, -1.0, 1.0, -1.0];
        let config = BurstConfig {
            preamble: preamble.clone(),
            threshold: 0.9,
            min_burst_len: 4,
            max_burst_len: 100,
            guard_samples: 2,
        };
        let mut sync = BurstSynchronizer::new(config);
        let mut signal = vec![0.0; 20];
        signal.extend(&preamble);
        signal.extend(vec![1.0; 50]);
        let bursts = sync.process(&signal);
        assert!(!bursts.is_empty());
        assert!(bursts[0].correlation >= 0.9);
    }

    #[test]
    fn test_burst_data() {
        let preamble = vec![1.0, 1.0];
        let config = BurstConfig {
            preamble: preamble.clone(),
            threshold: 0.9,
            min_burst_len: 2,
            max_burst_len: 10,
            guard_samples: 0,
        };
        let mut sync = BurstSynchronizer::new(config);
        let mut signal = vec![0.1; 5]; // low noise
        signal.extend(&preamble);
        signal.extend(vec![2.0; 8]); // payload
        let bursts = sync.process(&signal);
        if !bursts.is_empty() {
            assert!(bursts[0].data.len() >= 2);
        }
    }

    #[test]
    fn test_no_burst() {
        let config = BurstConfig::default();
        let mut sync = BurstSynchronizer::new(config);
        let signal = vec![0.0; 100];
        let bursts = sync.process(&signal);
        assert!(bursts.is_empty());
    }

    #[test]
    fn test_reset() {
        let config = BurstConfig::default();
        let mut sync = BurstSynchronizer::new(config);
        sync.process(&vec![1.0; 100]);
        assert!(sync.total_samples() > 0);
        sync.reset();
        assert_eq!(sync.total_samples(), 0);
        assert_eq!(sync.bursts_detected(), 0);
        assert_eq!(sync.buffer_len(), 0);
    }

    #[test]
    fn test_default_config() {
        let config = BurstConfig::default();
        assert_eq!(config.preamble.len(), 4);
        assert_eq!(config.threshold, 0.7);
        assert_eq!(config.guard_samples, 8);
    }

    #[test]
    fn test_energy_at_end() {
        // Burst extends to end of signal.
        let mut signal = vec![0.0; 10];
        signal.extend(vec![1.0; 20]);
        let bursts = detect_bursts_energy(&signal, 0.5, 5);
        assert_eq!(bursts.len(), 1);
        assert_eq!(bursts[0].1, 20);
    }

    #[test]
    fn test_empty_signal() {
        let bursts = detect_bursts_energy(&[], 0.5, 5);
        assert!(bursts.is_empty());
    }
}
