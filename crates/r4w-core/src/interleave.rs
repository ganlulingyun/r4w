//! # Interleave / De-interleave Streams
//!
//! Interleaves N input streams into a single output stream and
//! de-interleaves a single stream into N outputs. Supports configurable
//! block sizes per stream. Different from the `interleaved` module which
//! handles I/Q interleaving specifically.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::interleave::{interleave_streams, deinterleave_stream};
//!
//! let a = vec![1.0, 2.0];
//! let b = vec![3.0, 4.0];
//! let interleaved = interleave_streams(&[&a, &b], 1);
//! assert_eq!(interleaved, vec![1.0, 3.0, 2.0, 4.0]);
//!
//! let (outputs, _) = deinterleave_stream(&interleaved, 2, 1);
//! assert_eq!(outputs[0], vec![1.0, 2.0]);
//! assert_eq!(outputs[1], vec![3.0, 4.0]);
//! ```

/// Interleave N input streams into one output.
///
/// Takes `block_size` samples from each stream in round-robin order.
pub fn interleave_streams(streams: &[&[f64]], block_size: usize) -> Vec<f64> {
    if streams.is_empty() || block_size == 0 {
        return Vec::new();
    }

    let min_len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let num_complete_blocks = min_len / block_size;
    let n = streams.len();
    let mut output = Vec::with_capacity(num_complete_blocks * block_size * n);

    for block_idx in 0..num_complete_blocks {
        let start = block_idx * block_size;
        for stream in streams {
            output.extend_from_slice(&stream[start..start + block_size]);
        }
    }

    output
}

/// Interleave complex IQ streams.
pub fn interleave_complex(streams: &[&[(f64, f64)]], block_size: usize) -> Vec<(f64, f64)> {
    if streams.is_empty() || block_size == 0 {
        return Vec::new();
    }

    let min_len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let num_complete_blocks = min_len / block_size;
    let n = streams.len();
    let mut output = Vec::with_capacity(num_complete_blocks * block_size * n);

    for block_idx in 0..num_complete_blocks {
        let start = block_idx * block_size;
        for stream in streams {
            output.extend_from_slice(&stream[start..start + block_size]);
        }
    }

    output
}

/// De-interleave one stream into N output streams.
///
/// Returns (outputs, remainder).
pub fn deinterleave_stream(
    input: &[f64],
    num_outputs: usize,
    block_size: usize,
) -> (Vec<Vec<f64>>, Vec<f64>) {
    if num_outputs == 0 || block_size == 0 {
        return (Vec::new(), input.to_vec());
    }

    let cycle_len = num_outputs * block_size;
    let mut outputs: Vec<Vec<f64>> = vec![Vec::new(); num_outputs];

    let mut pos = 0;
    while pos + cycle_len <= input.len() {
        for (i, output) in outputs.iter_mut().enumerate() {
            let start = pos + i * block_size;
            output.extend_from_slice(&input[start..start + block_size]);
        }
        pos += cycle_len;
    }

    let remainder = input[pos..].to_vec();
    (outputs, remainder)
}

/// De-interleave complex IQ stream.
pub fn deinterleave_complex(
    input: &[(f64, f64)],
    num_outputs: usize,
    block_size: usize,
) -> (Vec<Vec<(f64, f64)>>, Vec<(f64, f64)>) {
    if num_outputs == 0 || block_size == 0 {
        return (Vec::new(), input.to_vec());
    }

    let cycle_len = num_outputs * block_size;
    let mut outputs: Vec<Vec<(f64, f64)>> = vec![Vec::new(); num_outputs];

    let mut pos = 0;
    while pos + cycle_len <= input.len() {
        for (i, output) in outputs.iter_mut().enumerate() {
            let start = pos + i * block_size;
            output.extend_from_slice(&input[start..start + block_size]);
        }
        pos += cycle_len;
    }

    let remainder = input[pos..].to_vec();
    (outputs, remainder)
}

/// Interleave bytes.
pub fn interleave_bytes(streams: &[&[u8]], block_size: usize) -> Vec<u8> {
    if streams.is_empty() || block_size == 0 {
        return Vec::new();
    }

    let min_len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let num_blocks = min_len / block_size;
    let n = streams.len();
    let mut output = Vec::with_capacity(num_blocks * block_size * n);

    for block_idx in 0..num_blocks {
        let start = block_idx * block_size;
        for stream in streams {
            output.extend_from_slice(&stream[start..start + block_size]);
        }
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interleave_sample_by_sample() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![4.0, 5.0, 6.0];
        let result = interleave_streams(&[&a, &b], 1);
        assert_eq!(result, vec![1.0, 4.0, 2.0, 5.0, 3.0, 6.0]);
    }

    #[test]
    fn test_interleave_blocks() {
        let a = vec![1.0, 2.0, 3.0, 4.0];
        let b = vec![5.0, 6.0, 7.0, 8.0];
        let result = interleave_streams(&[&a, &b], 2);
        assert_eq!(result, vec![1.0, 2.0, 5.0, 6.0, 3.0, 4.0, 7.0, 8.0]);
    }

    #[test]
    fn test_interleave_three() {
        let a = vec![1.0, 2.0];
        let b = vec![3.0, 4.0];
        let c = vec![5.0, 6.0];
        let result = interleave_streams(&[&a, &b, &c], 1);
        assert_eq!(result, vec![1.0, 3.0, 5.0, 2.0, 4.0, 6.0]);
    }

    #[test]
    fn test_deinterleave() {
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let (outputs, rem) = deinterleave_stream(&input, 2, 1);
        assert_eq!(outputs[0], vec![1.0, 3.0, 5.0]);
        assert_eq!(outputs[1], vec![2.0, 4.0, 6.0]);
        assert!(rem.is_empty());
    }

    #[test]
    fn test_deinterleave_blocks() {
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let (outputs, rem) = deinterleave_stream(&input, 2, 2);
        assert_eq!(outputs[0], vec![1.0, 2.0, 5.0, 6.0]);
        assert_eq!(outputs[1], vec![3.0, 4.0, 7.0, 8.0]);
        assert!(rem.is_empty());
    }

    #[test]
    fn test_roundtrip() {
        let a = vec![1.0, 2.0, 3.0, 4.0];
        let b = vec![5.0, 6.0, 7.0, 8.0];
        let interleaved = interleave_streams(&[&a, &b], 2);
        let (outputs, _) = deinterleave_stream(&interleaved, 2, 2);
        assert_eq!(outputs[0], a);
        assert_eq!(outputs[1], b);
    }

    #[test]
    fn test_remainder() {
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let (outputs, rem) = deinterleave_stream(&input, 2, 1);
        assert_eq!(outputs[0], vec![1.0, 3.0]);
        assert_eq!(outputs[1], vec![2.0, 4.0]);
        assert_eq!(rem, vec![5.0]);
    }

    #[test]
    fn test_complex_interleave() {
        let a = vec![(1.0, 0.0), (2.0, 0.0)];
        let b = vec![(3.0, 0.0), (4.0, 0.0)];
        let result = interleave_complex(&[&a, &b], 1);
        assert_eq!(result.len(), 4);
        assert_eq!(result[0], (1.0, 0.0));
        assert_eq!(result[1], (3.0, 0.0));
    }

    #[test]
    fn test_empty_input() {
        let result = interleave_streams(&[], 1);
        assert!(result.is_empty());
    }

    #[test]
    fn test_bytes() {
        let a = vec![0xAAu8, 0xBB];
        let b = vec![0xCC, 0xDD];
        let result = interleave_bytes(&[&a, &b], 1);
        assert_eq!(result, vec![0xAA, 0xCC, 0xBB, 0xDD]);
    }
}
