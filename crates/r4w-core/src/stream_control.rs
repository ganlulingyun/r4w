//! Stream Control Blocks
//!
//! Simple utility blocks for controlling sample streams:
//! - **Head**: Pass first N samples, then stop
//! - **SkipHead**: Drop first N samples, pass the rest
//! - **Throttle**: Rate-limit samples to match wall-clock time
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_control::{Head, SkipHead};
//! use num_complex::Complex64;
//!
//! let mut head = Head::new(10);
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let output = head.process_block(&input);
//! assert_eq!(output.len(), 10);
//!
//! let mut skip = SkipHead::new(90);
//! let output = skip.process_block(&input);
//! assert_eq!(output.len(), 10);
//! ```

use num_complex::Complex64;
use std::time::{Duration, Instant};

/// Pass only the first N samples, then output nothing.
#[derive(Debug, Clone)]
pub struct Head {
    /// Maximum samples to pass
    limit: usize,
    /// Samples passed so far
    count: usize,
}

impl Head {
    /// Create a new Head block that passes at most `n` samples.
    pub fn new(n: usize) -> Self {
        Self { limit: n, count: 0 }
    }

    /// Process a single sample. Returns `Some` if under limit.
    pub fn process(&mut self, input: Complex64) -> Option<Complex64> {
        if self.count < self.limit {
            self.count += 1;
            Some(input)
        } else {
            None
        }
    }

    /// Process a block. Returns only samples up to the limit.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let remaining = self.limit.saturating_sub(self.count);
        let take = input.len().min(remaining);
        self.count += take;
        input[..take].to_vec()
    }

    /// Process real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let remaining = self.limit.saturating_sub(self.count);
        let take = input.len().min(remaining);
        self.count += take;
        input[..take].to_vec()
    }

    /// Check if the head has been exhausted.
    pub fn is_done(&self) -> bool {
        self.count >= self.limit
    }

    /// Get number of samples passed.
    pub fn samples_passed(&self) -> usize {
        self.count
    }

    /// Reset counter.
    pub fn reset(&mut self) {
        self.count = 0;
    }
}

/// Drop the first N samples, then pass everything.
#[derive(Debug, Clone)]
pub struct SkipHead {
    /// Number of samples to skip
    skip: usize,
    /// Samples skipped so far
    count: usize,
}

impl SkipHead {
    /// Create a new SkipHead that drops the first `n` samples.
    pub fn new(n: usize) -> Self {
        Self { skip: n, count: 0 }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Option<Complex64> {
        if self.count < self.skip {
            self.count += 1;
            None
        } else {
            Some(input)
        }
    }

    /// Process a block.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let remaining_skip = self.skip.saturating_sub(self.count);
        let to_skip = input.len().min(remaining_skip);
        self.count += to_skip;
        input[to_skip..].to_vec()
    }

    /// Process real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let remaining_skip = self.skip.saturating_sub(self.count);
        let to_skip = input.len().min(remaining_skip);
        self.count += to_skip;
        input[to_skip..].to_vec()
    }

    /// Check if skip period is complete.
    pub fn is_skipping(&self) -> bool {
        self.count < self.skip
    }

    /// Get number of samples skipped.
    pub fn samples_skipped(&self) -> usize {
        self.count
    }

    /// Reset counter.
    pub fn reset(&mut self) {
        self.count = 0;
    }
}

/// Rate-limiting throttle block.
///
/// Ensures samples are processed at approximately the given sample rate,
/// sleeping as needed to match wall-clock time. Useful for file-based
/// sources and simulations to prevent CPU saturation.
#[derive(Debug)]
pub struct Throttle {
    /// Target sample rate (Hz)
    sample_rate: f64,
    /// Samples processed since last time reference
    samples_processed: u64,
    /// Time reference
    start_time: Instant,
    /// Whether to actually sleep (can be disabled for testing)
    enabled: bool,
}

impl Throttle {
    /// Create a new throttle at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            samples_processed: 0,
            start_time: Instant::now(),
            enabled: true,
        }
    }

    /// Create a throttle that doesn't actually sleep (for testing).
    pub fn new_passthrough(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            samples_processed: 0,
            start_time: Instant::now(),
            enabled: false,
        }
    }

    /// Process a block, sleeping if we're ahead of real-time.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        self.throttle(input.len());
        input.to_vec()
    }

    /// Process real samples with throttling.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        self.throttle(input.len());
        input.to_vec()
    }

    /// Throttle by sleeping if we're ahead of real time.
    pub fn throttle(&mut self, num_samples: usize) {
        self.samples_processed += num_samples as u64;

        if !self.enabled {
            return;
        }

        // How much time should have elapsed for this many samples?
        let expected_duration =
            Duration::from_secs_f64(self.samples_processed as f64 / self.sample_rate);
        let actual_duration = self.start_time.elapsed();

        if expected_duration > actual_duration {
            let sleep_time = expected_duration - actual_duration;
            std::thread::sleep(sleep_time);
        }
    }

    /// Get the current throughput in samples per second.
    pub fn throughput(&self) -> f64 {
        let elapsed = self.start_time.elapsed().as_secs_f64();
        if elapsed > 0.0 {
            self.samples_processed as f64 / elapsed
        } else {
            0.0
        }
    }

    /// Set the target sample rate.
    pub fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate;
        self.reset();
    }

    /// Reset timing reference.
    pub fn reset(&mut self) {
        self.samples_processed = 0;
        self.start_time = Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_head_limits_samples() {
        let mut head = Head::new(10);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = head.process_block(&input);
        assert_eq!(output.len(), 10);
        assert!(head.is_done());
    }

    #[test]
    fn test_head_multiple_blocks() {
        let mut head = Head::new(15);
        let block1 = vec![Complex64::new(1.0, 0.0); 10];
        let block2 = vec![Complex64::new(2.0, 0.0); 10];

        let out1 = head.process_block(&block1);
        assert_eq!(out1.len(), 10);
        assert!(!head.is_done());

        let out2 = head.process_block(&block2);
        assert_eq!(out2.len(), 5);
        assert!(head.is_done());
    }

    #[test]
    fn test_head_sample_by_sample() {
        let mut head = Head::new(3);
        assert!(head.process(Complex64::new(1.0, 0.0)).is_some());
        assert!(head.process(Complex64::new(2.0, 0.0)).is_some());
        assert!(head.process(Complex64::new(3.0, 0.0)).is_some());
        assert!(head.process(Complex64::new(4.0, 0.0)).is_none());
    }

    #[test]
    fn test_head_real() {
        let mut head = Head::new(5);
        let output = head.process_real(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]);
        assert_eq!(output.len(), 5);
    }

    #[test]
    fn test_head_reset() {
        let mut head = Head::new(5);
        let _ = head.process_block(&vec![Complex64::new(1.0, 0.0); 10]);
        assert!(head.is_done());
        head.reset();
        assert!(!head.is_done());
        assert_eq!(head.samples_passed(), 0);
    }

    #[test]
    fn test_skip_head_drops_samples() {
        let mut skip = SkipHead::new(90);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = skip.process_block(&input);
        assert_eq!(output.len(), 10);
        assert!(!skip.is_skipping());
    }

    #[test]
    fn test_skip_head_multiple_blocks() {
        let mut skip = SkipHead::new(15);
        let block1 = vec![Complex64::new(1.0, 0.0); 10];
        let block2 = vec![Complex64::new(2.0, 0.0); 10];

        let out1 = skip.process_block(&block1);
        assert_eq!(out1.len(), 0);
        assert!(skip.is_skipping());

        let out2 = skip.process_block(&block2);
        assert_eq!(out2.len(), 5);
        // Verify the correct samples are passed (from block2, after skip)
        assert!((out2[0].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_skip_head_sample_by_sample() {
        let mut skip = SkipHead::new(2);
        assert!(skip.process(Complex64::new(1.0, 0.0)).is_none());
        assert!(skip.process(Complex64::new(2.0, 0.0)).is_none());
        let out = skip.process(Complex64::new(3.0, 0.0));
        assert!(out.is_some());
        assert!((out.unwrap().re - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_skip_head_real() {
        let mut skip = SkipHead::new(3);
        let output = skip.process_real(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(output.len(), 2);
        assert!((output[0] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_throttle_passthrough() {
        let mut throttle = Throttle::new_passthrough(48000.0);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = throttle.process_block(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_throttle_real() {
        let mut throttle = Throttle::new_passthrough(48000.0);
        let output = throttle.process_real(&[1.0, 2.0, 3.0]);
        assert_eq!(output.len(), 3);
    }

    #[test]
    fn test_throttle_timing() {
        // Use a very high sample rate so we process instantly
        let mut throttle = Throttle::new(1_000_000.0); // 1 MHz
        let start = Instant::now();

        // Process 100 samples at 1 MHz = 0.1 ms
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let _ = throttle.process_block(&input);

        // Should complete very quickly
        assert!(start.elapsed() < Duration::from_millis(50));
    }
}
