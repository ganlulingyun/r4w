//! # Throttle Block
//!
//! Rate-limiting block for controlling sample throughput in
//! non-real-time pipelines. Ensures downstream blocks don't
//! process faster than the specified rate, useful for simulation
//! and file-based playback.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::throttle_blk::{Throttle, TokenBucket};
//!
//! let mut throttle = Throttle::new(48000.0); // 48 kHz
//! let input = vec![0.0; 4800];
//! let allowed = throttle.request(input.len());
//! assert!(allowed <= input.len());
//!
//! // Token bucket for burst-aware rate limiting.
//! let mut bucket = TokenBucket::new(1000.0, 2000);
//! bucket.refill(0.5); // 0.5 seconds elapsed
//! assert!(bucket.consume(500));
//! ```

/// Sample-rate throttle for pipelines.
#[derive(Debug, Clone)]
pub struct Throttle {
    /// Target sample rate (samples/second).
    sample_rate: f64,
    /// Accumulated samples allowed.
    budget: f64,
    /// Maximum burst size (samples).
    max_burst: usize,
    /// Total samples passed.
    total_passed: u64,
    /// Total samples blocked.
    total_blocked: u64,
}

impl Throttle {
    /// Create a new throttle at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate: sample_rate.max(1.0),
            budget: 0.0,
            max_burst: usize::MAX,
            total_passed: 0,
            total_blocked: 0,
        }
    }

    /// Create with maximum burst size.
    pub fn with_burst(sample_rate: f64, max_burst: usize) -> Self {
        Self {
            sample_rate: sample_rate.max(1.0),
            budget: 0.0,
            max_burst,
            total_passed: 0,
            total_blocked: 0,
        }
    }

    /// Add elapsed time, increasing the sample budget.
    pub fn tick(&mut self, elapsed_seconds: f64) {
        self.budget += elapsed_seconds * self.sample_rate;
        if self.budget > self.max_burst as f64 {
            self.budget = self.max_burst as f64;
        }
    }

    /// Request N samples. Returns how many are allowed.
    pub fn request(&mut self, n: usize) -> usize {
        let allowed = (n as f64).min(self.budget) as usize;
        self.budget -= allowed as f64;
        self.total_passed += allowed as u64;
        self.total_blocked += (n - allowed) as u64;
        allowed
    }

    /// Process real samples with throttling.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let n = self.request(input.len());
        input[..n].to_vec()
    }

    /// Process complex samples with throttling.
    pub fn process_complex(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.request(input.len());
        input[..n].to_vec()
    }

    /// Get current budget.
    pub fn budget(&self) -> f64 {
        self.budget
    }

    /// Get total samples passed.
    pub fn total_passed(&self) -> u64 {
        self.total_passed
    }

    /// Get total samples blocked.
    pub fn total_blocked(&self) -> u64 {
        self.total_blocked
    }

    /// Set sample rate.
    pub fn set_sample_rate(&mut self, rate: f64) {
        self.sample_rate = rate.max(1.0);
    }

    /// Get sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.budget = 0.0;
        self.total_passed = 0;
        self.total_blocked = 0;
    }
}

/// Token bucket rate limiter.
///
/// Allows bursts up to `capacity` tokens, refilling at `rate` tokens/second.
#[derive(Debug, Clone)]
pub struct TokenBucket {
    /// Refill rate (tokens per second).
    rate: f64,
    /// Maximum capacity.
    capacity: usize,
    /// Current tokens available.
    tokens: f64,
}

impl TokenBucket {
    /// Create a new token bucket.
    pub fn new(rate: f64, capacity: usize) -> Self {
        Self {
            rate: rate.max(0.0),
            capacity,
            tokens: capacity as f64, // Start full.
        }
    }

    /// Refill tokens based on elapsed time.
    pub fn refill(&mut self, elapsed_seconds: f64) {
        self.tokens += elapsed_seconds * self.rate;
        if self.tokens > self.capacity as f64 {
            self.tokens = self.capacity as f64;
        }
    }

    /// Try to consume N tokens. Returns true if successful.
    pub fn consume(&mut self, n: usize) -> bool {
        if self.tokens >= n as f64 {
            self.tokens -= n as f64;
            true
        } else {
            false
        }
    }

    /// Consume up to N tokens, returning how many were consumed.
    pub fn consume_partial(&mut self, n: usize) -> usize {
        let available = (self.tokens as usize).min(n);
        self.tokens -= available as f64;
        available
    }

    /// Get available tokens.
    pub fn available(&self) -> usize {
        self.tokens as usize
    }

    /// Check if N tokens are available.
    pub fn can_consume(&self, n: usize) -> bool {
        self.tokens >= n as f64
    }

    /// Get current token count (fractional).
    pub fn tokens(&self) -> f64 {
        self.tokens
    }

    /// Reset to full capacity.
    pub fn reset(&mut self) {
        self.tokens = self.capacity as f64;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_throttle_basic() {
        let mut t = Throttle::new(1000.0);
        t.tick(0.1); // 100 samples budget.
        let allowed = t.request(50);
        assert_eq!(allowed, 50);
        assert_eq!(t.total_passed(), 50);
    }

    #[test]
    fn test_throttle_limit() {
        let mut t = Throttle::new(1000.0);
        t.tick(0.1); // 100 samples budget.
        let allowed = t.request(200);
        assert_eq!(allowed, 100);
        assert_eq!(t.total_blocked(), 100);
    }

    #[test]
    fn test_throttle_process() {
        let mut t = Throttle::new(1000.0);
        t.tick(0.05); // 50 samples.
        let input = vec![1.0; 100];
        let output = t.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_throttle_burst_limit() {
        let mut t = Throttle::with_burst(1000.0, 50);
        t.tick(1.0); // Would give 1000 but burst limit is 50.
        let allowed = t.request(100);
        assert_eq!(allowed, 50);
    }

    #[test]
    fn test_throttle_reset() {
        let mut t = Throttle::new(1000.0);
        t.tick(1.0);
        t.request(500);
        t.reset();
        assert_eq!(t.total_passed(), 0);
        assert_eq!(t.total_blocked(), 0);
        assert_eq!(t.budget(), 0.0);
    }

    #[test]
    fn test_token_bucket_basic() {
        let mut bucket = TokenBucket::new(100.0, 1000);
        assert_eq!(bucket.available(), 1000); // Starts full.
        assert!(bucket.consume(500));
        assert_eq!(bucket.available(), 500);
    }

    #[test]
    fn test_token_bucket_refill() {
        let mut bucket = TokenBucket::new(100.0, 1000);
        bucket.consume(1000); // Empty.
        assert_eq!(bucket.available(), 0);
        bucket.refill(5.0); // +500 tokens.
        assert_eq!(bucket.available(), 500);
    }

    #[test]
    fn test_token_bucket_overflow() {
        let mut bucket = TokenBucket::new(100.0, 200);
        bucket.refill(100.0); // Would add 10000 but cap is 200.
        assert_eq!(bucket.available(), 200);
    }

    #[test]
    fn test_token_bucket_consume_fail() {
        let mut bucket = TokenBucket::new(100.0, 100);
        assert!(bucket.consume(100));
        assert!(!bucket.consume(1)); // Empty.
    }

    #[test]
    fn test_token_bucket_partial() {
        let mut bucket = TokenBucket::new(100.0, 100);
        bucket.consume(80); // 20 left.
        let consumed = bucket.consume_partial(50);
        assert_eq!(consumed, 20);
    }
}
