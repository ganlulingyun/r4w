//! Patterned Interleaver — Custom-pattern stream interleaving
//!
//! Interleaves N input streams using a user-defined pattern instead of
//! simple round-robin. Each element of the pattern selects which input
//! stream provides the next output sample. Useful for LDPC/turbo code
//! interleavers, custom frame structures, and protocol-specific muxing.
//! GNU Radio equivalent: `patterned_interleaver`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::patterned_interleaver::PatternedInterleaver;
//!
//! // Pattern: take from stream 0, 0, 1, 0, 1, 1
//! let interleaver = PatternedInterleaver::new(&[0, 0, 1, 0, 1, 1]);
//! let streams: Vec<Vec<f64>> = vec![
//!     vec![1.0, 2.0, 3.0],  // stream 0
//!     vec![10.0, 20.0, 30.0], // stream 1
//! ];
//! let slices: Vec<&[f64]> = streams.iter().map(|v| v.as_slice()).collect();
//! let output = interleaver.process(&slices);
//! assert_eq!(output, vec![1.0, 2.0, 10.0, 3.0, 20.0, 30.0]);
//! ```

/// Patterned interleaver for N streams.
#[derive(Debug, Clone)]
pub struct PatternedInterleaver {
    /// Pattern: each element is a stream index.
    pattern: Vec<usize>,
    /// Number of input streams (max(pattern) + 1).
    num_streams: usize,
}

impl PatternedInterleaver {
    /// Create with a pattern. Each element selects a stream index.
    pub fn new(pattern: &[usize]) -> Self {
        let num_streams = pattern.iter().copied().max().map(|m| m + 1).unwrap_or(0);
        Self {
            pattern: pattern.to_vec(),
            num_streams,
        }
    }

    /// Process real streams. Reads from each stream sequentially per pattern.
    pub fn process(&self, streams: &[&[f64]]) -> Vec<f64> {
        if self.pattern.is_empty() || streams.len() < self.num_streams {
            return Vec::new();
        }
        let mut cursors = vec![0usize; self.num_streams];
        let mut output = Vec::new();

        // Determine how many full pattern cycles we can do
        let mut counts = vec![0usize; self.num_streams];
        for &idx in &self.pattern {
            counts[idx] += 1;
        }
        let max_cycles = (0..self.num_streams)
            .map(|i| {
                if counts[i] == 0 {
                    usize::MAX
                } else {
                    streams[i].len() / counts[i]
                }
            })
            .min()
            .unwrap_or(0);

        for _ in 0..max_cycles {
            for &idx in &self.pattern {
                output.push(streams[idx][cursors[idx]]);
                cursors[idx] += 1;
            }
        }
        output
    }

    /// Process byte streams.
    pub fn process_bytes(&self, streams: &[&[u8]]) -> Vec<u8> {
        if self.pattern.is_empty() || streams.len() < self.num_streams {
            return Vec::new();
        }
        let mut cursors = vec![0usize; self.num_streams];
        let mut output = Vec::new();

        let mut counts = vec![0usize; self.num_streams];
        for &idx in &self.pattern {
            counts[idx] += 1;
        }
        let max_cycles = (0..self.num_streams)
            .map(|i| {
                if counts[i] == 0 {
                    usize::MAX
                } else {
                    streams[i].len() / counts[i]
                }
            })
            .min()
            .unwrap_or(0);

        for _ in 0..max_cycles {
            for &idx in &self.pattern {
                output.push(streams[idx][cursors[idx]]);
                cursors[idx] += 1;
            }
        }
        output
    }

    /// Get the pattern.
    pub fn pattern(&self) -> &[usize] {
        &self.pattern
    }

    /// Get number of required input streams.
    pub fn num_streams(&self) -> usize {
        self.num_streams
    }

    /// Get pattern length.
    pub fn pattern_len(&self) -> usize {
        self.pattern.len()
    }
}

/// Patterned deinterleaver (inverse operation).
#[derive(Debug, Clone)]
pub struct PatternedDeinterleaver {
    pattern: Vec<usize>,
    num_streams: usize,
}

impl PatternedDeinterleaver {
    /// Create from the same pattern used for interleaving.
    pub fn new(pattern: &[usize]) -> Self {
        let num_streams = pattern.iter().copied().max().map(|m| m + 1).unwrap_or(0);
        Self {
            pattern: pattern.to_vec(),
            num_streams,
        }
    }

    /// Deinterleave a stream back into N separate streams.
    pub fn process(&self, input: &[f64]) -> Vec<Vec<f64>> {
        let mut outputs = vec![Vec::new(); self.num_streams];
        if self.pattern.is_empty() {
            return outputs;
        }
        let full_cycles = input.len() / self.pattern.len();
        for cycle in 0..full_cycles {
            for (j, &idx) in self.pattern.iter().enumerate() {
                outputs[idx].push(input[cycle * self.pattern.len() + j]);
            }
        }
        outputs
    }

    /// Deinterleave bytes.
    pub fn process_bytes(&self, input: &[u8]) -> Vec<Vec<u8>> {
        let mut outputs = vec![Vec::new(); self.num_streams];
        if self.pattern.is_empty() {
            return outputs;
        }
        let full_cycles = input.len() / self.pattern.len();
        for cycle in 0..full_cycles {
            for (j, &idx) in self.pattern.iter().enumerate() {
                outputs[idx].push(input[cycle * self.pattern.len() + j]);
            }
        }
        outputs
    }

    /// Number of output streams.
    pub fn num_streams(&self) -> usize {
        self.num_streams
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_interleave() {
        let il = PatternedInterleaver::new(&[0, 0, 1, 0, 1, 1]);
        let s0 = vec![1.0, 2.0, 3.0];
        let s1 = vec![10.0, 20.0, 30.0];
        let out = il.process(&[&s0, &s1]);
        assert_eq!(out, vec![1.0, 2.0, 10.0, 3.0, 20.0, 30.0]);
    }

    #[test]
    fn test_round_robin_equivalent() {
        let il = PatternedInterleaver::new(&[0, 1]);
        let s0 = vec![1.0, 3.0, 5.0];
        let s1 = vec![2.0, 4.0, 6.0];
        let out = il.process(&[&s0, &s1]);
        assert_eq!(out, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_three_streams() {
        let il = PatternedInterleaver::new(&[0, 1, 2]);
        let s0 = vec![1.0, 4.0];
        let s1 = vec![2.0, 5.0];
        let s2 = vec![3.0, 6.0];
        let out = il.process(&[&s0, &s1, &s2]);
        assert_eq!(out, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_unequal_streams() {
        let il = PatternedInterleaver::new(&[0, 1]);
        let s0 = vec![1.0, 3.0, 5.0];
        let s1 = vec![2.0, 4.0]; // shorter
        let out = il.process(&[&s0, &s1]);
        // Only 2 full cycles possible
        assert_eq!(out, vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_bytes() {
        let il = PatternedInterleaver::new(&[0, 1, 0]);
        let s0: Vec<u8> = vec![0xAA, 0xBB];
        let s1: Vec<u8> = vec![0x11];
        let out = il.process_bytes(&[&s0, &s1]);
        assert_eq!(out, vec![0xAA, 0x11, 0xBB]);
    }

    #[test]
    fn test_deinterleave() {
        let pattern = [0, 0, 1, 0, 1, 1];
        let il = PatternedInterleaver::new(&pattern);
        let s0 = vec![1.0, 2.0, 3.0];
        let s1 = vec![10.0, 20.0, 30.0];
        let interleaved = il.process(&[&s0, &s1]);

        let dil = PatternedDeinterleaver::new(&pattern);
        let recovered = dil.process(&interleaved);
        assert_eq!(recovered[0], s0);
        assert_eq!(recovered[1], s1);
    }

    #[test]
    fn test_deinterleave_bytes() {
        let pattern = [0, 1];
        let input: Vec<u8> = vec![1, 10, 2, 20, 3, 30];
        let dil = PatternedDeinterleaver::new(&pattern);
        let out = dil.process_bytes(&input);
        assert_eq!(out[0], vec![1, 2, 3]);
        assert_eq!(out[1], vec![10, 20, 30]);
    }

    #[test]
    fn test_empty_pattern() {
        let il = PatternedInterleaver::new(&[]);
        assert!(il.process(&[&[1.0]]).is_empty());
    }

    #[test]
    fn test_wrong_stream_count() {
        let il = PatternedInterleaver::new(&[0, 1, 2]);
        // Only 2 streams but pattern needs 3
        assert!(il.process(&[&[1.0], &[2.0]]).is_empty());
    }

    #[test]
    fn test_single_stream_pattern() {
        let il = PatternedInterleaver::new(&[0, 0, 0]);
        let s0 = vec![1.0, 2.0, 3.0];
        let out = il.process(&[&s0]);
        assert_eq!(out, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_accessors() {
        let il = PatternedInterleaver::new(&[0, 1, 0, 2]);
        assert_eq!(il.pattern(), &[0, 1, 0, 2]);
        assert_eq!(il.num_streams(), 3);
        assert_eq!(il.pattern_len(), 4);
    }

    #[test]
    fn test_roundtrip() {
        let pattern = [0, 1, 2, 0, 1];
        let s0 = vec![1.0, 4.0];
        let s1 = vec![2.0, 5.0];
        let s2 = vec![3.0];
        let il = PatternedInterleaver::new(&pattern);
        let interleaved = il.process(&[&s0, &s1, &s2]);
        let dil = PatternedDeinterleaver::new(&pattern);
        let recovered = dil.process(&interleaved);
        assert_eq!(recovered[0], s0);
        assert_eq!(recovered[1], s1);
        assert_eq!(recovered[2], s2);
    }
}
