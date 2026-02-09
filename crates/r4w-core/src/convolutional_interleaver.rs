//! Convolutional Interleaver — Forney-type burst error dispersal
//!
//! Depth-based convolutional interleaver for continuous burst error protection.
//! Unlike block interleavers, convolutional interleavers operate continuously
//! with fixed latency. Used in DVB-S2 (I=12, M=17), GSM (I=4, M=19), and
//! other satellite/mobile standards. GNU Radio equivalent: `dvbt_convolutional_interleaver_bb`.
//!
//! ## Structure
//!
//! I branches with delays D_i = i × M:
//! ```text
//! Branch 0: delay = 0      (pass-through)
//! Branch 1: delay = M      (M-sample FIFO)
//! Branch 2: delay = 2M
//!   ...
//! Branch I-1: delay = (I-1)×M
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::convolutional_interleaver::{ConvolutionalInterleaver, ConvolutionalDeinterleaver};
//!
//! let mut interleaver = ConvolutionalInterleaver::new(4, 3);
//! let mut deinterleaver = ConvolutionalDeinterleaver::new(4, 3);
//!
//! let data: Vec<u8> = (0..40).collect();
//! let interleaved = interleaver.process(&data);
//! let recovered = deinterleaver.process(&interleaved);
//! // After initial latency, recovered data matches original
//! assert_eq!(interleaver.get_latency(), 36);
//! ```

use std::collections::VecDeque;

/// Convolutional interleaver (Forney type).
///
/// Input symbols are routed through I branches with linearly increasing delays.
/// Branch i has delay i × M samples.
#[derive(Debug, Clone)]
pub struct ConvolutionalInterleaver {
    /// Number of branches.
    branches: usize,
    /// Base delay depth.
    depth: usize,
    /// Delay lines for each branch (branch i has delay i*depth).
    delays: Vec<VecDeque<u8>>,
    /// Current input counter (selects branch).
    input_counter: usize,
}

/// Convolutional deinterleaver (complement of interleaver).
///
/// Uses reversed branch ordering: branch i has delay (I-1-i) × M.
#[derive(Debug, Clone)]
pub struct ConvolutionalDeinterleaver {
    /// Number of branches.
    branches: usize,
    /// Base delay depth.
    depth: usize,
    /// Delay lines for each branch (branch i has delay (I-1-i)*depth).
    delays: Vec<VecDeque<u8>>,
    /// Current input counter.
    input_counter: usize,
}

impl ConvolutionalInterleaver {
    /// Create a new convolutional interleaver.
    ///
    /// # Arguments
    /// * `branches` - Number of branches I (must be >= 1)
    /// * `depth` - Base delay M (branch i has delay i*M)
    pub fn new(branches: usize, depth: usize) -> Self {
        assert!(branches >= 1, "Need at least 1 branch");
        let mut delays = Vec::with_capacity(branches);
        for i in 0..branches {
            let len = i * depth;
            let mut dq = VecDeque::with_capacity(len);
            for _ in 0..len {
                dq.push_back(0);
            }
            delays.push(dq);
        }
        Self {
            branches,
            depth,
            delays,
            input_counter: 0,
        }
    }

    /// DVB-S2 standard interleaver (I=12, M=17).
    pub fn dvb_s2() -> Self {
        Self::new(12, 17)
    }

    /// GSM standard interleaver (I=4, M=19).
    pub fn gsm() -> Self {
        Self::new(4, 19)
    }

    /// Process a block of input bytes through the interleaver.
    pub fn process(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &byte in input {
            output.push(self.process_one(byte));
        }
        output
    }

    /// Process a single byte.
    fn process_one(&mut self, input: u8) -> u8 {
        let branch = self.input_counter % self.branches;
        self.input_counter += 1;

        if self.delays[branch].is_empty() {
            // Branch 0 (zero delay) — pass-through
            return input;
        }

        self.delays[branch].push_back(input);
        self.delays[branch].pop_front().unwrap()
    }

    /// Process a block of bits through the interleaver.
    pub fn process_bits(&mut self, input: &[bool]) -> Vec<bool> {
        let bytes: Vec<u8> = input.iter().map(|&b| b as u8).collect();
        let interleaved = self.process(&bytes);
        interleaved.iter().map(|&b| b != 0).collect()
    }

    /// Get total interleaver latency in samples.
    ///
    /// Latency = M × I × (I-1) for interleaver+deinterleaver pair.
    pub fn get_latency(&self) -> usize {
        self.depth * self.branches * (self.branches - 1)
    }

    /// Get total memory cells used.
    pub fn total_memory(&self) -> usize {
        self.depth * self.branches * (self.branches - 1) / 2
    }

    /// Reset all delay lines to zero.
    pub fn reset(&mut self) {
        self.input_counter = 0;
        for (i, dq) in self.delays.iter_mut().enumerate() {
            let len = i * self.depth;
            dq.clear();
            for _ in 0..len {
                dq.push_back(0);
            }
        }
    }

    /// Number of branches.
    pub fn branches(&self) -> usize {
        self.branches
    }

    /// Base delay depth.
    pub fn depth(&self) -> usize {
        self.depth
    }
}

impl ConvolutionalDeinterleaver {
    /// Create a new convolutional deinterleaver.
    ///
    /// Uses reversed branch delays: branch i has delay (I-1-i) × M.
    pub fn new(branches: usize, depth: usize) -> Self {
        assert!(branches >= 1, "Need at least 1 branch");
        let mut delays = Vec::with_capacity(branches);
        for i in 0..branches {
            let len = (branches - 1 - i) * depth;
            let mut dq = VecDeque::with_capacity(len);
            for _ in 0..len {
                dq.push_back(0);
            }
            delays.push(dq);
        }
        Self {
            branches,
            depth,
            delays,
            input_counter: 0,
        }
    }

    /// Process a block of input bytes through the deinterleaver.
    pub fn process(&mut self, input: &[u8]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len());
        for &byte in input {
            output.push(self.process_one(byte));
        }
        output
    }

    /// Process a single byte.
    fn process_one(&mut self, input: u8) -> u8 {
        let branch = self.input_counter % self.branches;
        self.input_counter += 1;

        if self.delays[branch].is_empty() {
            return input;
        }

        self.delays[branch].push_back(input);
        self.delays[branch].pop_front().unwrap()
    }

    /// Reset all delay lines.
    pub fn reset(&mut self) {
        self.input_counter = 0;
        for (i, dq) in self.delays.iter_mut().enumerate() {
            let len = (self.branches - 1 - i) * self.depth;
            dq.clear();
            for _ in 0..len {
                dq.push_back(0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity_zero_depth() {
        let mut interleaver = ConvolutionalInterleaver::new(4, 0);
        let input: Vec<u8> = (0..20).collect();
        let output = interleaver.process(&input);
        assert_eq!(input, output, "Zero depth should pass through unchanged");
    }

    #[test]
    fn test_single_branch() {
        let mut interleaver = ConvolutionalInterleaver::new(1, 5);
        let input: Vec<u8> = (0..10).collect();
        let output = interleaver.process(&input);
        assert_eq!(input, output, "Single branch should pass through unchanged");
    }

    #[test]
    fn test_small_pattern() {
        let mut interleaver = ConvolutionalInterleaver::new(2, 3);
        // Branch 0: delay=0, Branch 1: delay=3
        let input: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        let output = interleaver.process(&input);
        // Branch 0 samples (indices 0,2,4,6,8): pass through = 1,3,5,7,9
        // Branch 1 samples (indices 1,3,5,7,9): delayed by 3 = 0,0,0,2,4,6,8,10
        assert_eq!(output[0], 1); // Branch 0: pass-through
        assert_eq!(output[1], 0); // Branch 1: delay, output zero
        assert_eq!(output[2], 3); // Branch 0: pass-through
        assert_eq!(output[3], 0); // Branch 1: still in delay
    }

    #[test]
    fn test_dvb_s2_latency() {
        let interleaver = ConvolutionalInterleaver::dvb_s2();
        assert_eq!(interleaver.branches(), 12);
        assert_eq!(interleaver.depth(), 17);
        assert_eq!(interleaver.get_latency(), 17 * 12 * 11);
        assert_eq!(interleaver.get_latency(), 2244);
    }

    #[test]
    fn test_gsm_latency() {
        let interleaver = ConvolutionalInterleaver::gsm();
        assert_eq!(interleaver.branches(), 4);
        assert_eq!(interleaver.depth(), 19);
        assert_eq!(interleaver.get_latency(), 19 * 4 * 3);
        assert_eq!(interleaver.get_latency(), 228);
    }

    #[test]
    fn test_roundtrip() {
        let mut interleaver = ConvolutionalInterleaver::new(4, 3);
        let mut deinterleaver = ConvolutionalDeinterleaver::new(4, 3);

        // Send enough data to flush through latency
        let latency = interleaver.get_latency(); // 4*3*3 = 36
        let total = latency + 100;
        let input: Vec<u8> = (0..total as u8).map(|i| i.wrapping_add(1)).collect();

        let interleaved = interleaver.process(&input);
        let recovered = deinterleaver.process(&interleaved);

        // After latency samples, output should match input
        let mut matches = 0;
        for i in latency..total {
            if recovered[i] == input[i - latency] {
                matches += 1;
            }
        }
        assert!(matches > 90, "Expected most samples to match after latency, got {}/100", matches);
    }

    #[test]
    fn test_burst_error_dispersal() {
        let mut interleaver = ConvolutionalInterleaver::new(4, 5);
        let latency = interleaver.get_latency(); // 60
        let total = latency + 200;
        let input: Vec<u8> = vec![1; total];
        let mut interleaved = interleaver.process(&input);

        // Inject a burst error well past the transient
        let burst_start = latency + 50;
        let burst_len = 8;
        for i in burst_start..(burst_start + burst_len) {
            interleaved[i] = 0;
        }

        let mut deinterleaver = ConvolutionalDeinterleaver::new(4, 5);
        let recovered = deinterleaver.process(&interleaved);

        // Only count errors after the full transient (2x latency for interleaver+deinterleaver)
        let check_start = 2 * latency;
        let mut max_consecutive_errors = 0;
        let mut current_consecutive = 0;
        for &byte in &recovered[check_start..] {
            if byte == 0 {
                current_consecutive += 1;
                max_consecutive_errors = max_consecutive_errors.max(current_consecutive);
            } else {
                current_consecutive = 0;
            }
        }
        // After deinterleaving, a burst of 8 should be dispersed to at most
        // ceil(burst_len / branches) = 2 consecutive errors
        assert!(max_consecutive_errors <= 3,
            "Burst of {} should be dispersed, max consecutive after transient: {}",
            burst_len, max_consecutive_errors);
    }

    #[test]
    fn test_total_memory() {
        let interleaver = ConvolutionalInterleaver::new(4, 3);
        // Memory = M * I * (I-1) / 2 = 3 * 4 * 3 / 2 = 18
        assert_eq!(interleaver.total_memory(), 18);
    }

    #[test]
    fn test_reset() {
        let mut interleaver = ConvolutionalInterleaver::new(3, 4);
        let input: Vec<u8> = vec![10; 30];
        let _ = interleaver.process(&input);

        interleaver.reset();

        // After reset, should behave like fresh
        let output = interleaver.process(&[1, 2, 3]);
        assert_eq!(output[0], 1); // Branch 0 pass-through
    }

    #[test]
    fn test_process_bits() {
        let mut interleaver = ConvolutionalInterleaver::new(2, 2);
        let bits = vec![true, false, true, true, false, false, true, false];
        let interleaved = interleaver.process_bits(&bits);
        assert_eq!(interleaved.len(), bits.len());
    }
}
