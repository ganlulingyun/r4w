//! Stream Multiplexer and Demultiplexer
//!
//! Interleave and de-interleave sample streams:
//!
//! - `StreamMux`: Combine N input streams into one by taking a specified
//!   number of samples from each in round-robin order.
//! - `StreamDemux`: Split one stream into N outputs by distributing
//!   samples round-robin.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stream_mux::{StreamMux, StreamDemux};
//! use num_complex::Complex64;
//!
//! // Mux: take 2 samples from each of 3 streams
//! let mux = StreamMux::new(&[2, 2, 2]);
//! let streams = vec![
//!     vec![Complex64::new(1.0, 0.0); 4],
//!     vec![Complex64::new(2.0, 0.0); 4],
//!     vec![Complex64::new(3.0, 0.0); 4],
//! ];
//! let output = mux.mux(&streams);
//! // Output: [1,1, 2,2, 3,3, 1,1, 2,2, 3,3]
//! assert_eq!(output.len(), 12);
//!
//! // Demux: split back into 3 streams
//! let demux = StreamDemux::new(&[2, 2, 2]);
//! let streams_out = demux.demux(&output);
//! assert_eq!(streams_out.len(), 3);
//! ```

use num_complex::Complex64;

/// Round-robin multiplexer for N streams.
///
/// Takes `lengths[i]` samples from stream `i` in each cycle.
#[derive(Debug, Clone)]
pub struct StreamMux {
    /// Number of samples to take from each stream per cycle
    lengths: Vec<usize>,
}

impl StreamMux {
    /// Create a mux. `lengths[i]` = samples per cycle from stream i.
    pub fn new(lengths: &[usize]) -> Self {
        assert!(!lengths.is_empty(), "Must have at least one stream");
        assert!(lengths.iter().all(|&l| l > 0), "All lengths must be positive");
        Self { lengths: lengths.to_vec() }
    }

    /// Create a uniform mux (same count from each stream).
    pub fn uniform(num_streams: usize, samples_per_stream: usize) -> Self {
        Self::new(&vec![samples_per_stream; num_streams])
    }

    /// Multiplex N streams into one output stream.
    pub fn mux(&self, streams: &[Vec<Complex64>]) -> Vec<Complex64> {
        assert_eq!(streams.len(), self.lengths.len(),
            "Expected {} streams, got {}", self.lengths.len(), streams.len());

        let cycle_len: usize = self.lengths.iter().sum();
        // Find max number of complete cycles
        let max_cycles = streams.iter().zip(self.lengths.iter())
            .map(|(s, &l)| if l > 0 { s.len() / l } else { 0 })
            .min()
            .unwrap_or(0);

        let mut output = Vec::with_capacity(max_cycles * cycle_len);
        let mut positions: Vec<usize> = vec![0; streams.len()];

        for _ in 0..max_cycles {
            for (i, &count) in self.lengths.iter().enumerate() {
                let start = positions[i];
                let end = start + count;
                if end <= streams[i].len() {
                    output.extend_from_slice(&streams[i][start..end]);
                    positions[i] = end;
                }
            }
        }

        output
    }

    /// Multiplex real streams.
    pub fn mux_real(&self, streams: &[Vec<f64>]) -> Vec<f64> {
        assert_eq!(streams.len(), self.lengths.len());
        let max_cycles = streams.iter().zip(self.lengths.iter())
            .map(|(s, &l)| if l > 0 { s.len() / l } else { 0 })
            .min()
            .unwrap_or(0);
        let cycle_len: usize = self.lengths.iter().sum();
        let mut output = Vec::with_capacity(max_cycles * cycle_len);
        let mut positions: Vec<usize> = vec![0; streams.len()];
        for _ in 0..max_cycles {
            for (i, &count) in self.lengths.iter().enumerate() {
                let start = positions[i];
                let end = start + count;
                if end <= streams[i].len() {
                    output.extend_from_slice(&streams[i][start..end]);
                    positions[i] = end;
                }
            }
        }
        output
    }

    /// Number of streams.
    pub fn num_streams(&self) -> usize {
        self.lengths.len()
    }
}

/// Round-robin demultiplexer — splits one stream into N.
///
/// Takes `lengths[i]` samples for output stream `i` in each cycle.
#[derive(Debug, Clone)]
pub struct StreamDemux {
    /// Number of samples to assign to each stream per cycle
    lengths: Vec<usize>,
}

impl StreamDemux {
    /// Create a demux. `lengths[i]` = samples per cycle for stream i.
    pub fn new(lengths: &[usize]) -> Self {
        assert!(!lengths.is_empty(), "Must have at least one stream");
        assert!(lengths.iter().all(|&l| l > 0), "All lengths must be positive");
        Self { lengths: lengths.to_vec() }
    }

    /// Create a uniform demux.
    pub fn uniform(num_streams: usize, samples_per_stream: usize) -> Self {
        Self::new(&vec![samples_per_stream; num_streams])
    }

    /// Demultiplex one stream into N output streams.
    pub fn demux(&self, input: &[Complex64]) -> Vec<Vec<Complex64>> {
        let cycle_len: usize = self.lengths.iter().sum();
        let num_cycles = if cycle_len > 0 { input.len() / cycle_len } else { 0 };

        let mut streams: Vec<Vec<Complex64>> = self.lengths.iter()
            .map(|&l| Vec::with_capacity(num_cycles * l))
            .collect();

        let mut pos = 0;
        for _ in 0..num_cycles {
            for (i, &count) in self.lengths.iter().enumerate() {
                let end = pos + count;
                if end <= input.len() {
                    streams[i].extend_from_slice(&input[pos..end]);
                    pos = end;
                }
            }
        }

        streams
    }

    /// Demultiplex real samples.
    pub fn demux_real(&self, input: &[f64]) -> Vec<Vec<f64>> {
        let cycle_len: usize = self.lengths.iter().sum();
        let num_cycles = if cycle_len > 0 { input.len() / cycle_len } else { 0 };
        let mut streams: Vec<Vec<f64>> = self.lengths.iter()
            .map(|&l| Vec::with_capacity(num_cycles * l))
            .collect();
        let mut pos = 0;
        for _ in 0..num_cycles {
            for (i, &count) in self.lengths.iter().enumerate() {
                let end = pos + count;
                if end <= input.len() {
                    streams[i].extend_from_slice(&input[pos..end]);
                    pos = end;
                }
            }
        }
        streams
    }

    /// Number of output streams.
    pub fn num_streams(&self) -> usize {
        self.lengths.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn c(re: f64) -> Complex64 {
        Complex64::new(re, 0.0)
    }

    #[test]
    fn test_mux_uniform() {
        let mux = StreamMux::uniform(2, 3);
        let streams = vec![
            vec![c(1.0), c(1.0), c(1.0), c(1.0), c(1.0), c(1.0)],
            vec![c(2.0), c(2.0), c(2.0), c(2.0), c(2.0), c(2.0)],
        ];
        let output = mux.mux(&streams);
        // 2 complete cycles of [3 from A, 3 from B]
        assert_eq!(output.len(), 12);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[3].re - 2.0).abs() < 1e-10);
        assert!((output[6].re - 1.0).abs() < 1e-10);
        assert!((output[9].re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_mux_non_uniform() {
        let mux = StreamMux::new(&[1, 2]);
        let streams = vec![
            vec![c(1.0), c(1.0)],
            vec![c(2.0), c(2.0), c(2.0), c(2.0)],
        ];
        let output = mux.mux(&streams);
        // 2 cycles of [1 from A, 2 from B] = 6
        assert_eq!(output.len(), 6);
        assert!((output[0].re - 1.0).abs() < 1e-10); // A
        assert!((output[1].re - 2.0).abs() < 1e-10); // B
        assert!((output[2].re - 2.0).abs() < 1e-10); // B
        assert!((output[3].re - 1.0).abs() < 1e-10); // A
    }

    #[test]
    fn test_demux_uniform() {
        let demux = StreamDemux::uniform(3, 2);
        let input: Vec<Complex64> = vec![c(1.0), c(1.0), c(2.0), c(2.0), c(3.0), c(3.0)];
        let streams = demux.demux(&input);
        assert_eq!(streams.len(), 3);
        assert_eq!(streams[0].len(), 2);
        assert!((streams[0][0].re - 1.0).abs() < 1e-10);
        assert!((streams[1][0].re - 2.0).abs() < 1e-10);
        assert!((streams[2][0].re - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_mux_demux_roundtrip() {
        let mux = StreamMux::uniform(2, 4);
        let demux = StreamDemux::uniform(2, 4);

        let original = vec![
            vec![c(1.0), c(2.0), c(3.0), c(4.0)],
            vec![c(5.0), c(6.0), c(7.0), c(8.0)],
        ];
        let muxed = mux.mux(&original);
        let recovered = demux.demux(&muxed);

        assert_eq!(recovered.len(), 2);
        for (orig, rec) in original.iter().zip(recovered.iter()) {
            for (o, r) in orig.iter().zip(rec.iter()) {
                assert!((o.re - r.re).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_demux_truncates() {
        let demux = StreamDemux::uniform(2, 3);
        // 7 samples, cycle=6, only 1 complete cycle
        let input = vec![c(1.0); 7];
        let streams = demux.demux(&input);
        assert_eq!(streams[0].len(), 3);
        assert_eq!(streams[1].len(), 3);
        // 7th sample is discarded (incomplete cycle)
    }

    #[test]
    fn test_mux_real() {
        let mux = StreamMux::uniform(2, 2);
        let streams = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![5.0, 6.0, 7.0, 8.0],
        ];
        let output = mux.mux_real(&streams);
        assert_eq!(output, vec![1.0, 2.0, 5.0, 6.0, 3.0, 4.0, 7.0, 8.0]);
    }

    #[test]
    fn test_demux_real() {
        let demux = StreamDemux::uniform(2, 2);
        let input = vec![1.0, 2.0, 5.0, 6.0, 3.0, 4.0, 7.0, 8.0];
        let streams = demux.demux_real(&input);
        assert_eq!(streams[0], vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(streams[1], vec![5.0, 6.0, 7.0, 8.0]);
    }

    #[test]
    fn test_three_stream_mux() {
        let mux = StreamMux::new(&[1, 1, 1]);
        let streams = vec![
            vec![c(1.0), c(1.0)],
            vec![c(2.0), c(2.0)],
            vec![c(3.0), c(3.0)],
        ];
        let output = mux.mux(&streams);
        assert_eq!(output.len(), 6);
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[1].re - 2.0).abs() < 1e-10);
        assert!((output[2].re - 3.0).abs() < 1e-10);
    }
}
