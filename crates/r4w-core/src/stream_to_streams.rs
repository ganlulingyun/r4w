//! Stream to Streams — Round-robin demux/mux by index
//!
//! Splits a single stream into N output streams by distributing samples
//! in round-robin order, and vice versa. Essential for polyphase
//! decomposition, OFDM subcarrier demuxing, and I/Q deinterleaving.
//! GNU Radio equivalents: `stream_to_streams`, `streams_to_stream`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_to_streams::{StreamToStreams, StreamsToStream};
//!
//! // Demux: [1, 2, 3, 4, 5, 6] with 3 streams → [[1,4], [2,5], [3,6]]
//! let demux = StreamToStreams::new(3);
//! let outputs = demux.process(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
//! assert_eq!(outputs[0], vec![1.0, 4.0]);
//! assert_eq!(outputs[1], vec![2.0, 5.0]);
//! assert_eq!(outputs[2], vec![3.0, 6.0]);
//!
//! // Mux: [[1,4], [2,5], [3,6]] → [1, 2, 3, 4, 5, 6]
//! let mux = StreamsToStream::new(3);
//! let slices: Vec<&[f64]> = outputs.iter().map(|v| v.as_slice()).collect();
//! let output = mux.process(&slices);
//! assert_eq!(output, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
//! ```

use num_complex::Complex64;

/// Demultiplex a single stream into N streams (round-robin).
#[derive(Debug, Clone)]
pub struct StreamToStreams {
    num_streams: usize,
}

impl StreamToStreams {
    /// Create a demuxer with `n` output streams.
    pub fn new(n: usize) -> Self {
        Self {
            num_streams: n.max(1),
        }
    }

    /// Demux real samples. Truncates to multiple of `num_streams`.
    pub fn process(&self, input: &[f64]) -> Vec<Vec<f64>> {
        let n = self.num_streams;
        let usable = (input.len() / n) * n;
        let mut outputs = vec![Vec::with_capacity(usable / n); n];
        for (i, &x) in input[..usable].iter().enumerate() {
            outputs[i % n].push(x);
        }
        outputs
    }

    /// Demux complex samples.
    pub fn process_complex(&self, input: &[Complex64]) -> Vec<Vec<Complex64>> {
        let n = self.num_streams;
        let usable = (input.len() / n) * n;
        let mut outputs = vec![Vec::with_capacity(usable / n); n];
        for (i, &x) in input[..usable].iter().enumerate() {
            outputs[i % n].push(x);
        }
        outputs
    }

    /// Demux bytes.
    pub fn process_bytes(&self, input: &[u8]) -> Vec<Vec<u8>> {
        let n = self.num_streams;
        let usable = (input.len() / n) * n;
        let mut outputs = vec![Vec::with_capacity(usable / n); n];
        for (i, &x) in input[..usable].iter().enumerate() {
            outputs[i % n].push(x);
        }
        outputs
    }

    /// Get number of output streams.
    pub fn num_streams(&self) -> usize {
        self.num_streams
    }
}

/// Multiplex N streams back into a single stream (round-robin interleave).
#[derive(Debug, Clone)]
pub struct StreamsToStream {
    num_streams: usize,
}

impl StreamsToStream {
    /// Create a muxer for `n` input streams.
    pub fn new(n: usize) -> Self {
        Self {
            num_streams: n.max(1),
        }
    }

    /// Mux real samples. Uses the shortest stream length.
    pub fn process(&self, inputs: &[&[f64]]) -> Vec<f64> {
        if inputs.len() != self.num_streams || inputs.is_empty() {
            return Vec::new();
        }
        let min_len = inputs.iter().map(|s| s.len()).min().unwrap_or(0);
        let mut output = Vec::with_capacity(min_len * self.num_streams);
        for i in 0..min_len {
            for stream in inputs {
                output.push(stream[i]);
            }
        }
        output
    }

    /// Mux complex samples.
    pub fn process_complex(&self, inputs: &[&[Complex64]]) -> Vec<Complex64> {
        if inputs.len() != self.num_streams || inputs.is_empty() {
            return Vec::new();
        }
        let min_len = inputs.iter().map(|s| s.len()).min().unwrap_or(0);
        let mut output = Vec::with_capacity(min_len * self.num_streams);
        for i in 0..min_len {
            for stream in inputs {
                output.push(stream[i]);
            }
        }
        output
    }

    /// Mux bytes.
    pub fn process_bytes(&self, inputs: &[&[u8]]) -> Vec<u8> {
        if inputs.len() != self.num_streams || inputs.is_empty() {
            return Vec::new();
        }
        let min_len = inputs.iter().map(|s| s.len()).min().unwrap_or(0);
        let mut output = Vec::with_capacity(min_len * self.num_streams);
        for i in 0..min_len {
            for stream in inputs {
                output.push(stream[i]);
            }
        }
        output
    }

    /// Get number of input streams.
    pub fn num_streams(&self) -> usize {
        self.num_streams
    }
}

/// Convenience: Deinterleave I/Q from a single interleaved real stream.
///
/// Input: [I0, Q0, I1, Q1, ...] → Output: Vec<Complex64>
pub fn deinterleave_iq(input: &[f64]) -> Vec<Complex64> {
    let demux = StreamToStreams::new(2);
    let streams = demux.process(input);
    if streams[0].len() != streams[1].len() {
        return Vec::new();
    }
    streams[0]
        .iter()
        .zip(streams[1].iter())
        .map(|(&i, &q)| Complex64::new(i, q))
        .collect()
}

/// Convenience: Interleave Complex64 → [I0, Q0, I1, Q1, ...]
pub fn interleave_iq(input: &[Complex64]) -> Vec<f64> {
    let mut output = Vec::with_capacity(input.len() * 2);
    for &z in input {
        output.push(z.re);
        output.push(z.im);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_demux_basic() {
        let demux = StreamToStreams::new(3);
        let out = demux.process(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(out.len(), 3);
        assert_eq!(out[0], vec![1.0, 4.0]);
        assert_eq!(out[1], vec![2.0, 5.0]);
        assert_eq!(out[2], vec![3.0, 6.0]);
    }

    #[test]
    fn test_demux_truncates() {
        let demux = StreamToStreams::new(3);
        let out = demux.process(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        // Only uses first 3 (truncates to multiple of 3)
        assert_eq!(out[0], vec![1.0]);
        assert_eq!(out[1], vec![2.0]);
        assert_eq!(out[2], vec![3.0]);
    }

    #[test]
    fn test_demux_single_stream() {
        let demux = StreamToStreams::new(1);
        let out = demux.process(&[1.0, 2.0, 3.0]);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0], vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_mux_basic() {
        let mux = StreamsToStream::new(3);
        let s0 = [1.0, 4.0];
        let s1 = [2.0, 5.0];
        let s2 = [3.0, 6.0];
        let out = mux.process(&[&s0, &s1, &s2]);
        assert_eq!(out, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_mux_unequal_lengths() {
        let mux = StreamsToStream::new(2);
        let s0 = [1.0, 3.0, 5.0];
        let s1 = [2.0, 4.0]; // shorter
        let out = mux.process(&[&s0, &s1]);
        assert_eq!(out, vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_roundtrip() {
        let data: Vec<f64> = (0..12).map(|i| i as f64).collect();
        let demux = StreamToStreams::new(4);
        let streams = demux.process(&data);
        let mux = StreamsToStream::new(4);
        let slices: Vec<&[f64]> = streams.iter().map(|v| v.as_slice()).collect();
        let recovered = mux.process(&slices);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_complex_roundtrip() {
        let data: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, -(i as f64)))
            .collect();
        let demux = StreamToStreams::new(2);
        let streams = demux.process_complex(&data);
        assert_eq!(streams.len(), 2);
        assert_eq!(streams[0].len(), 4);
        let mux = StreamsToStream::new(2);
        let slices: Vec<&[Complex64]> = streams.iter().map(|v| v.as_slice()).collect();
        let recovered = mux.process_complex(&slices);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_bytes_demux() {
        let demux = StreamToStreams::new(2);
        let out = demux.process_bytes(&[0x01, 0x02, 0x03, 0x04]);
        assert_eq!(out[0], vec![0x01, 0x03]);
        assert_eq!(out[1], vec![0x02, 0x04]);
    }

    #[test]
    fn test_deinterleave_iq() {
        let interleaved = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let complex = deinterleave_iq(&interleaved);
        assert_eq!(complex.len(), 3);
        assert_eq!(complex[0], Complex64::new(1.0, 2.0));
        assert_eq!(complex[1], Complex64::new(3.0, 4.0));
        assert_eq!(complex[2], Complex64::new(5.0, 6.0));
    }

    #[test]
    fn test_interleave_iq() {
        let complex = vec![
            Complex64::new(1.0, 2.0),
            Complex64::new(3.0, 4.0),
        ];
        let flat = interleave_iq(&complex);
        assert_eq!(flat, vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_iq_roundtrip() {
        let original = vec![
            Complex64::new(1.0, -1.0),
            Complex64::new(2.0, -2.0),
            Complex64::new(3.0, -3.0),
        ];
        let interleaved = interleave_iq(&original);
        let recovered = deinterleave_iq(&interleaved);
        assert_eq!(recovered, original);
    }

    #[test]
    fn test_wrong_stream_count() {
        let mux = StreamsToStream::new(3);
        let s0 = [1.0];
        let s1 = [2.0];
        // Only 2 streams, but mux expects 3
        let out = mux.process(&[&s0, &s1]);
        assert!(out.is_empty());
    }

    #[test]
    fn test_empty_input() {
        let demux = StreamToStreams::new(3);
        let out = demux.process(&[]);
        assert_eq!(out.len(), 3);
        assert!(out[0].is_empty());
    }
}
