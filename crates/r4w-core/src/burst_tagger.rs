//! Burst Tagger and Tagged Stream Utilities
//!
//! Power-based burst detection and tagged stream processing for packet-mode
//! signal processing pipelines.
//!
//! ## Blocks
//!
//! - **BurstTagger**: Detect bursts by power level, output tagged bursts
//! - **TaggedStreamMux**: Multiplex multiple tagged streams
//! - **TaggedStreamMultiplyLength**: Scale length tags after rate changes
//! - **TaggedStreamAlign**: Align bursts to fixed boundaries
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::burst_tagger::BurstTagger;
//! use num_complex::Complex64;
//!
//! let mut tagger = BurstTagger::new(-20.0, 10);
//! let silence = vec![Complex64::new(0.001, 0.0); 50];
//! let burst = vec![Complex64::new(1.0, 0.0); 100];
//! let more_silence = vec![Complex64::new(0.001, 0.0); 50];
//!
//! let mut input = Vec::new();
//! input.extend_from_slice(&silence);
//! input.extend_from_slice(&burst);
//! input.extend_from_slice(&more_silence);
//!
//! let bursts = tagger.process(&input);
//! assert!(!bursts.is_empty());
//! ```

use num_complex::Complex64;

/// A tagged burst — a contiguous block of samples with metadata.
#[derive(Debug, Clone)]
pub struct TaggedBurst {
    /// Burst samples.
    pub data: Vec<Complex64>,
    /// Start offset in the original stream.
    pub start_offset: usize,
    /// Length in samples.
    pub length: usize,
    /// Peak power in dB.
    pub peak_power_db: f64,
    /// Average power in dB.
    pub avg_power_db: f64,
}

/// Burst detection state.
#[derive(Debug, Clone, Copy, PartialEq)]
enum BurstState {
    Idle,
    InBurst,
}

/// BurstTagger — detect bursts by power level and tag them.
#[derive(Debug, Clone)]
pub struct BurstTagger {
    /// Power threshold in dB.
    threshold_db: f64,
    /// Minimum burst length in samples.
    min_burst_len: usize,
    /// Holdoff in samples (keep burst alive after power drops).
    holdoff_samples: usize,
    /// Current state.
    state: BurstState,
    /// Current burst accumulator.
    current_burst: Vec<Complex64>,
    /// Start offset of current burst.
    burst_start: usize,
    /// Holdoff counter.
    holdoff_counter: usize,
    /// Global sample counter.
    sample_count: usize,
}

impl BurstTagger {
    pub fn new(threshold_db: f64, min_burst_len: usize) -> Self {
        Self {
            threshold_db,
            min_burst_len,
            holdoff_samples: 10,
            state: BurstState::Idle,
            current_burst: Vec::new(),
            burst_start: 0,
            holdoff_counter: 0,
            sample_count: 0,
        }
    }

    /// Set holdoff in samples.
    pub fn set_holdoff(&mut self, samples: usize) {
        self.holdoff_samples = samples;
    }

    /// Process a block of samples, returning detected bursts.
    pub fn process(&mut self, samples: &[Complex64]) -> Vec<TaggedBurst> {
        let mut bursts = Vec::new();

        for &s in samples {
            let power_db = 10.0 * s.norm_sqr().max(1e-20).log10();
            let above_threshold = power_db > self.threshold_db;

            match self.state {
                BurstState::Idle => {
                    if above_threshold {
                        self.state = BurstState::InBurst;
                        self.burst_start = self.sample_count;
                        self.current_burst.clear();
                        self.current_burst.push(s);
                        self.holdoff_counter = self.holdoff_samples;
                    }
                }
                BurstState::InBurst => {
                    self.current_burst.push(s);
                    if above_threshold {
                        self.holdoff_counter = self.holdoff_samples;
                    } else {
                        if self.holdoff_counter > 0 {
                            self.holdoff_counter -= 1;
                        } else {
                            // End of burst
                            if self.current_burst.len() >= self.min_burst_len {
                                let burst = self.finish_burst();
                                bursts.push(burst);
                            }
                            self.state = BurstState::Idle;
                            self.current_burst.clear();
                        }
                    }
                }
            }

            self.sample_count += 1;
        }

        bursts
    }

    /// Flush any pending burst.
    pub fn flush(&mut self) -> Option<TaggedBurst> {
        if self.state == BurstState::InBurst && self.current_burst.len() >= self.min_burst_len {
            let burst = self.finish_burst();
            self.state = BurstState::Idle;
            self.current_burst.clear();
            Some(burst)
        } else {
            self.state = BurstState::Idle;
            self.current_burst.clear();
            None
        }
    }

    fn finish_burst(&self) -> TaggedBurst {
        let data = self.current_burst.clone();
        let length = data.len();
        let peak_power_db = data
            .iter()
            .map(|s| 10.0 * s.norm_sqr().max(1e-20).log10())
            .fold(f64::NEG_INFINITY, f64::max);
        let avg_power = data.iter().map(|s| s.norm_sqr()).sum::<f64>() / length as f64;
        let avg_power_db = 10.0 * avg_power.max(1e-20).log10();

        TaggedBurst {
            data,
            start_offset: self.burst_start,
            length,
            peak_power_db,
            avg_power_db,
        }
    }

    pub fn reset(&mut self) {
        self.state = BurstState::Idle;
        self.current_burst.clear();
        self.burst_start = 0;
        self.holdoff_counter = 0;
        self.sample_count = 0;
    }
}

/// TaggedStreamMux — multiplex multiple tagged bursts into one stream.
#[derive(Debug, Clone)]
pub struct TaggedStreamMux {
    num_inputs: usize,
}

impl TaggedStreamMux {
    pub fn new(num_inputs: usize) -> Self {
        Self { num_inputs }
    }

    /// Concatenate bursts from multiple inputs into a single output.
    pub fn mux(&self, inputs: &[TaggedBurst]) -> TaggedBurst {
        let mut combined_data = Vec::new();
        let mut peak_power = f64::NEG_INFINITY;
        let start_offset = inputs.first().map_or(0, |b| b.start_offset);

        for burst in inputs {
            combined_data.extend_from_slice(&burst.data);
            if burst.peak_power_db > peak_power {
                peak_power = burst.peak_power_db;
            }
        }

        let length = combined_data.len();
        let avg_power = if length > 0 {
            let total: f64 = combined_data.iter().map(|s| s.norm_sqr()).sum();
            10.0 * (total / length as f64).max(1e-20).log10()
        } else {
            f64::NEG_INFINITY
        };

        TaggedBurst {
            data: combined_data,
            start_offset,
            length,
            peak_power_db: peak_power,
            avg_power_db: avg_power,
        }
    }
}

/// TaggedStreamMultiplyLength — scale burst length after rate changes.
#[derive(Debug, Clone)]
pub struct TaggedStreamMultiplyLength {
    coefficient: f64,
}

impl TaggedStreamMultiplyLength {
    pub fn new(coefficient: f64) -> Self {
        Self { coefficient }
    }

    /// Apply coefficient to burst length field.
    pub fn process(&self, burst: &mut TaggedBurst) {
        burst.length = (burst.length as f64 * self.coefficient).round() as usize;
    }
}

/// TaggedStreamAlign — align burst to fixed boundary.
#[derive(Debug, Clone)]
pub struct TaggedStreamAlign {
    alignment: usize,
}

impl TaggedStreamAlign {
    pub fn new(alignment: usize) -> Self {
        Self { alignment: alignment.max(1) }
    }

    /// Pad burst data to alignment boundary.
    pub fn align(&self, burst: &mut TaggedBurst) {
        let remainder = burst.data.len() % self.alignment;
        if remainder != 0 {
            let padding = self.alignment - remainder;
            burst.data.extend(std::iter::repeat(Complex64::new(0.0, 0.0)).take(padding));
            burst.length = burst.data.len();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_burst_signal(silence_before: usize, burst_len: usize, silence_after: usize, amplitude: f64) -> Vec<Complex64> {
        let mut signal = Vec::new();
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(silence_before));
        signal.extend(std::iter::repeat(Complex64::new(amplitude, 0.0)).take(burst_len));
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(silence_after));
        signal
    }

    #[test]
    fn test_burst_tagger_single_burst() {
        let mut tagger = BurstTagger::new(-20.0, 5);
        tagger.set_holdoff(3);
        let signal = make_burst_signal(50, 100, 50, 1.0);
        let mut bursts = tagger.process(&signal);
        if let Some(pending) = tagger.flush() {
            bursts.push(pending);
        }
        assert!(!bursts.is_empty(), "Should detect at least one burst");
        assert!(bursts[0].length >= 5, "Burst should meet minimum length");
    }

    #[test]
    fn test_burst_tagger_no_burst() {
        let mut tagger = BurstTagger::new(-10.0, 5);
        let signal = vec![Complex64::new(0.001, 0.0); 200];
        let bursts = tagger.process(&signal);
        assert!(bursts.is_empty(), "No burst should be detected in silence");
    }

    #[test]
    fn test_burst_tagger_two_bursts() {
        let mut tagger = BurstTagger::new(-20.0, 5);
        tagger.set_holdoff(2);
        let mut signal = Vec::new();
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(20));
        signal.extend(std::iter::repeat(Complex64::new(1.0, 0.0)).take(30));
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(20));
        signal.extend(std::iter::repeat(Complex64::new(0.5, 0.0)).take(40));
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(20));

        let mut bursts = tagger.process(&signal);
        if let Some(pending) = tagger.flush() {
            bursts.push(pending);
        }
        assert!(bursts.len() >= 2, "Should detect two bursts, got {}", bursts.len());
    }

    #[test]
    fn test_burst_tagger_min_length() {
        let mut tagger = BurstTagger::new(-20.0, 50);
        tagger.set_holdoff(0);
        // Short burst should be discarded
        let signal = make_burst_signal(20, 10, 20, 1.0);
        let bursts = tagger.process(&signal);
        assert!(bursts.is_empty(), "Short burst should be filtered out");
    }

    #[test]
    fn test_burst_power_measurement() {
        let mut tagger = BurstTagger::new(-30.0, 1);
        tagger.set_holdoff(0);
        let signal = make_burst_signal(10, 50, 20, 0.5);
        let mut bursts = tagger.process(&signal);
        if let Some(pending) = tagger.flush() {
            bursts.push(pending);
        }
        assert!(!bursts.is_empty());
        // 0.5^2 = 0.25, 10*log10(0.25) ≈ -6.02 dB
        let burst = &bursts[0];
        assert!(burst.peak_power_db > -10.0, "Peak power should be reasonable");
    }

    #[test]
    fn test_burst_tagger_flush() {
        let mut tagger = BurstTagger::new(-20.0, 5);
        // Start a burst but don't end it
        let signal = vec![Complex64::new(1.0, 0.0); 50];
        let bursts = tagger.process(&signal);
        assert!(bursts.is_empty(), "Burst still in progress");

        let flushed = tagger.flush();
        assert!(flushed.is_some(), "Flush should return pending burst");
        assert_eq!(flushed.unwrap().length, 50);
    }

    #[test]
    fn test_burst_tagger_reset() {
        let mut tagger = BurstTagger::new(-20.0, 5);
        let signal = vec![Complex64::new(1.0, 0.0); 50];
        tagger.process(&signal);
        tagger.reset();
        assert_eq!(tagger.sample_count, 0);
    }

    #[test]
    fn test_tagged_stream_mux() {
        let mux = TaggedStreamMux::new(2);
        let burst1 = TaggedBurst {
            data: vec![Complex64::new(1.0, 0.0); 10],
            start_offset: 0,
            length: 10,
            peak_power_db: 0.0,
            avg_power_db: 0.0,
        };
        let burst2 = TaggedBurst {
            data: vec![Complex64::new(0.5, 0.0); 20],
            start_offset: 100,
            length: 20,
            peak_power_db: -6.0,
            avg_power_db: -6.0,
        };
        let combined = mux.mux(&[burst1, burst2]);
        assert_eq!(combined.length, 30);
        assert_eq!(combined.data.len(), 30);
    }

    #[test]
    fn test_tagged_stream_multiply_length() {
        let scaler = TaggedStreamMultiplyLength::new(4.0);
        let mut burst = TaggedBurst {
            data: vec![Complex64::new(1.0, 0.0); 100],
            start_offset: 0,
            length: 100,
            peak_power_db: 0.0,
            avg_power_db: 0.0,
        };
        scaler.process(&mut burst);
        assert_eq!(burst.length, 400);
    }

    #[test]
    fn test_tagged_stream_align() {
        let aligner = TaggedStreamAlign::new(8);
        let mut burst = TaggedBurst {
            data: vec![Complex64::new(1.0, 0.0); 13],
            start_offset: 0,
            length: 13,
            peak_power_db: 0.0,
            avg_power_db: 0.0,
        };
        aligner.align(&mut burst);
        assert_eq!(burst.data.len() % 8, 0);
        assert_eq!(burst.data.len(), 16); // 13 → 16
        assert_eq!(burst.length, 16);
    }

    #[test]
    fn test_tagged_stream_align_already_aligned() {
        let aligner = TaggedStreamAlign::new(4);
        let mut burst = TaggedBurst {
            data: vec![Complex64::new(1.0, 0.0); 8],
            start_offset: 0,
            length: 8,
            peak_power_db: 0.0,
            avg_power_db: 0.0,
        };
        aligner.align(&mut burst);
        assert_eq!(burst.data.len(), 8); // already aligned
    }

    #[test]
    fn test_tagged_stream_mux_empty() {
        let mux = TaggedStreamMux::new(0);
        let combined = mux.mux(&[]);
        assert_eq!(combined.length, 0);
    }

    #[test]
    fn test_holdoff_extends_burst() {
        let mut tagger = BurstTagger::new(-20.0, 5);
        tagger.set_holdoff(10);
        // Burst with a brief dip
        let mut signal = Vec::new();
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(20));
        signal.extend(std::iter::repeat(Complex64::new(1.0, 0.0)).take(30));
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(5)); // brief gap (< holdoff)
        signal.extend(std::iter::repeat(Complex64::new(1.0, 0.0)).take(30));
        signal.extend(std::iter::repeat(Complex64::new(0.001, 0.0)).take(30));

        let mut bursts = tagger.process(&signal);
        if let Some(pending) = tagger.flush() {
            bursts.push(pending);
        }
        // With holdoff=10, the 5-sample gap should be bridged into one burst
        assert_eq!(bursts.len(), 1, "Holdoff should bridge short gap into one burst");
    }
}
