//! # Stream Demultiplexer
//!
//! Demultiplexes a single input stream into multiple output streams.
//! The inverse of `stream_mux`. Supports round-robin and length-tagged modes.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_demux::StreamDemux;
//!
//! // Round-robin demux into 3 outputs, 2 samples each
//! let demux = StreamDemux::new(vec![2, 2, 2]);
//! let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0];
//! let outputs = demux.process(&input);
//! assert_eq!(outputs[0], vec![1.0, 2.0, 7.0, 8.0]);
//! assert_eq!(outputs[1], vec![3.0, 4.0, 9.0, 10.0]);
//! assert_eq!(outputs[2], vec![5.0, 6.0, 11.0, 12.0]);
//! ```

/// Stream demultiplexer.
#[derive(Debug, Clone)]
pub struct StreamDemux {
    /// Number of samples per output per cycle.
    lengths: Vec<usize>,
    /// Total samples per cycle.
    cycle_len: usize,
}

impl StreamDemux {
    /// Create a new stream demux with the given output lengths per cycle.
    ///
    /// `lengths[i]` = number of samples routed to output i per cycle.
    pub fn new(lengths: Vec<usize>) -> Self {
        let cycle_len = lengths.iter().sum();
        Self { lengths, cycle_len }
    }

    /// Create a round-robin demux with equal chunks.
    pub fn round_robin(num_outputs: usize, samples_per_output: usize) -> Self {
        Self::new(vec![samples_per_output; num_outputs])
    }

    /// Demux input into N output streams.
    pub fn process(&self, input: &[f64]) -> Vec<Vec<f64>> {
        let n = self.lengths.len();
        let mut outputs: Vec<Vec<f64>> = vec![Vec::new(); n];

        let mut pos = 0;
        while pos + self.cycle_len <= input.len() {
            for (i, &len) in self.lengths.iter().enumerate() {
                outputs[i].extend_from_slice(&input[pos..pos + len]);
                pos += len;
            }
        }

        outputs
    }

    /// Demux complex IQ samples.
    pub fn process_complex(&self, input: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let n = self.lengths.len();
        let mut outputs: Vec<Vec<(f64, f64)>> = vec![Vec::new(); n];

        let mut pos = 0;
        while pos + self.cycle_len <= input.len() {
            for (i, &len) in self.lengths.iter().enumerate() {
                outputs[i].extend_from_slice(&input[pos..pos + len]);
                pos += len;
            }
        }

        outputs
    }

    /// Demux with remainder handling (returns leftover samples).
    pub fn process_with_remainder<'a>(&self, input: &'a [f64]) -> (Vec<Vec<f64>>, &'a [f64]) {
        let n = self.lengths.len();
        let mut outputs: Vec<Vec<f64>> = vec![Vec::new(); n];

        let mut pos = 0;
        while pos + self.cycle_len <= input.len() {
            for (i, &len) in self.lengths.iter().enumerate() {
                outputs[i].extend_from_slice(&input[pos..pos + len]);
                pos += len;
            }
        }

        (outputs, &input[pos..])
    }

    /// Get the number of outputs.
    pub fn num_outputs(&self) -> usize {
        self.lengths.len()
    }

    /// Get the cycle length (total samples consumed per cycle).
    pub fn cycle_len(&self) -> usize {
        self.cycle_len
    }

    /// Get the lengths per output.
    pub fn lengths(&self) -> &[usize] {
        &self.lengths
    }

    /// Set new output lengths.
    pub fn set_lengths(&mut self, lengths: Vec<usize>) {
        self.cycle_len = lengths.iter().sum();
        self.lengths = lengths;
    }
}

/// Demux a byte stream into N outputs.
pub fn demux_bytes(input: &[u8], lengths: &[usize]) -> Vec<Vec<u8>> {
    let cycle_len: usize = lengths.iter().sum();
    let n = lengths.len();
    let mut outputs: Vec<Vec<u8>> = vec![Vec::new(); n];

    let mut pos = 0;
    while pos + cycle_len <= input.len() {
        for (i, &len) in lengths.iter().enumerate() {
            outputs[i].extend_from_slice(&input[pos..pos + len]);
            pos += len;
        }
    }

    outputs
}

/// Demux a bool/bit stream into N outputs.
pub fn demux_bits(input: &[bool], lengths: &[usize]) -> Vec<Vec<bool>> {
    let cycle_len: usize = lengths.iter().sum();
    let n = lengths.len();
    let mut outputs: Vec<Vec<bool>> = vec![Vec::new(); n];

    let mut pos = 0;
    while pos + cycle_len <= input.len() {
        for (i, &len) in lengths.iter().enumerate() {
            outputs[i].extend_from_slice(&input[pos..pos + len]);
            pos += len;
        }
    }

    outputs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_demux() {
        let demux = StreamDemux::new(vec![2, 3]);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0];
        let outputs = demux.process(&input);
        assert_eq!(outputs[0], vec![1.0, 2.0, 6.0, 7.0]);
        assert_eq!(outputs[1], vec![3.0, 4.0, 5.0, 8.0, 9.0, 10.0]);
    }

    #[test]
    fn test_round_robin() {
        let demux = StreamDemux::round_robin(3, 1);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let outputs = demux.process(&input);
        assert_eq!(outputs[0], vec![1.0, 4.0]);
        assert_eq!(outputs[1], vec![2.0, 5.0]);
        assert_eq!(outputs[2], vec![3.0, 6.0]);
    }

    #[test]
    fn test_single_output() {
        let demux = StreamDemux::new(vec![5]);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let outputs = demux.process(&input);
        assert_eq!(outputs[0], input);
    }

    #[test]
    fn test_remainder() {
        let demux = StreamDemux::new(vec![2, 2]);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        let (outputs, remainder) = demux.process_with_remainder(&input);
        assert_eq!(outputs[0], vec![1.0, 2.0]);
        assert_eq!(outputs[1], vec![3.0, 4.0]);
        assert_eq!(remainder, &[5.0, 6.0, 7.0]);
    }

    #[test]
    fn test_complex_demux() {
        let demux = StreamDemux::round_robin(2, 1);
        let input = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0)];
        let outputs = demux.process_complex(&input);
        assert_eq!(outputs[0], vec![(1.0, 2.0), (5.0, 6.0)]);
        assert_eq!(outputs[1], vec![(3.0, 4.0), (7.0, 8.0)]);
    }

    #[test]
    fn test_empty_input() {
        let demux = StreamDemux::new(vec![2, 3]);
        let outputs = demux.process(&[]);
        assert_eq!(outputs.len(), 2);
        assert!(outputs[0].is_empty());
        assert!(outputs[1].is_empty());
    }

    #[test]
    fn test_demux_bytes() {
        let input = vec![0xAA, 0xBB, 0xCC, 0xDD];
        let outputs = demux_bytes(&input, &[1, 1]);
        assert_eq!(outputs[0], vec![0xAA, 0xCC]);
        assert_eq!(outputs[1], vec![0xBB, 0xDD]);
    }

    #[test]
    fn test_demux_bits() {
        let input = vec![true, false, true, false, true, false];
        let outputs = demux_bits(&input, &[1, 2]);
        assert_eq!(outputs[0], vec![true, false]);
        assert_eq!(outputs[1], vec![false, true, true, false]);
    }

    #[test]
    fn test_cycle_len() {
        let demux = StreamDemux::new(vec![3, 5, 2]);
        assert_eq!(demux.cycle_len(), 10);
        assert_eq!(demux.num_outputs(), 3);
    }

    #[test]
    fn test_set_lengths() {
        let mut demux = StreamDemux::new(vec![1, 1]);
        assert_eq!(demux.cycle_len(), 2);
        demux.set_lengths(vec![3, 2, 1]);
        assert_eq!(demux.cycle_len(), 6);
        assert_eq!(demux.num_outputs(), 3);
    }
}
