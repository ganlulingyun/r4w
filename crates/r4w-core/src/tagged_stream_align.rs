//! # Tagged Stream Aligner
//!
//! Aligns tagged stream blocks to fixed-size boundaries by padding or
//! truncating. Useful for ensuring uniform block sizes in processing
//! chains that require fixed-length inputs (FFT, OFDM, etc.).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tagged_stream_align::{TaggedStreamAlign, AlignMode};
//!
//! let aligner = TaggedStreamAlign::new(8, AlignMode::ZeroPad);
//! let short_block = vec![1.0, 2.0, 3.0];
//! let aligned = aligner.align(&short_block);
//! assert_eq!(aligned.len(), 8); // Padded with zeros
//! ```

/// Alignment mode for undersized blocks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlignMode {
    /// Pad with zeros.
    ZeroPad,
    /// Repeat the block cyclically to fill.
    CyclicRepeat,
    /// Pad with the last sample value.
    RepeatLast,
    /// Discard undersized blocks entirely.
    Discard,
}

/// Tagged stream block aligner.
#[derive(Debug, Clone)]
pub struct TaggedStreamAlign {
    /// Target block size.
    block_size: usize,
    /// How to handle undersized blocks.
    mode: AlignMode,
    /// Whether to truncate oversized blocks.
    truncate: bool,
    /// Total blocks processed.
    blocks_processed: u64,
    /// Total blocks discarded.
    blocks_discarded: u64,
}

impl TaggedStreamAlign {
    /// Create a new aligner with the given block size and mode.
    pub fn new(block_size: usize, mode: AlignMode) -> Self {
        Self {
            block_size,
            mode,
            truncate: true,
            blocks_processed: 0,
            blocks_discarded: 0,
        }
    }

    /// Set whether to truncate oversized blocks (default: true).
    pub fn set_truncate(&mut self, truncate: bool) {
        self.truncate = truncate;
    }

    /// Align a single block of real samples.
    pub fn align(&self, block: &[f64]) -> Vec<f64> {
        if block.len() == self.block_size {
            return block.to_vec();
        }

        if block.len() > self.block_size {
            if self.truncate {
                return block[..self.block_size].to_vec();
            } else {
                return block.to_vec();
            }
        }

        // Undersized block.
        match self.mode {
            AlignMode::ZeroPad => {
                let mut out = block.to_vec();
                out.resize(self.block_size, 0.0);
                out
            }
            AlignMode::CyclicRepeat => {
                let mut out = Vec::with_capacity(self.block_size);
                for i in 0..self.block_size {
                    out.push(block[i % block.len()]);
                }
                out
            }
            AlignMode::RepeatLast => {
                let mut out = block.to_vec();
                let last = *block.last().unwrap_or(&0.0);
                out.resize(self.block_size, last);
                out
            }
            AlignMode::Discard => {
                Vec::new()
            }
        }
    }

    /// Align a single block of complex samples.
    pub fn align_complex(&self, block: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if block.len() == self.block_size {
            return block.to_vec();
        }

        if block.len() > self.block_size {
            if self.truncate {
                return block[..self.block_size].to_vec();
            } else {
                return block.to_vec();
            }
        }

        match self.mode {
            AlignMode::ZeroPad => {
                let mut out = block.to_vec();
                out.resize(self.block_size, (0.0, 0.0));
                out
            }
            AlignMode::CyclicRepeat => {
                let mut out = Vec::with_capacity(self.block_size);
                for i in 0..self.block_size {
                    out.push(block[i % block.len()]);
                }
                out
            }
            AlignMode::RepeatLast => {
                let mut out = block.to_vec();
                let last = *block.last().unwrap_or(&(0.0, 0.0));
                out.resize(self.block_size, last);
                out
            }
            AlignMode::Discard => {
                Vec::new()
            }
        }
    }

    /// Align multiple blocks, filtering discarded ones.
    pub fn align_batch(&mut self, blocks: &[Vec<f64>]) -> Vec<Vec<f64>> {
        let mut result = Vec::new();
        for block in blocks {
            let aligned = self.align(block);
            self.blocks_processed += 1;
            if aligned.is_empty() && self.mode == AlignMode::Discard {
                self.blocks_discarded += 1;
            } else if !aligned.is_empty() {
                result.push(aligned);
            }
        }
        result
    }

    /// Get the target block size.
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Set a new block size.
    pub fn set_block_size(&mut self, size: usize) {
        self.block_size = size;
    }

    /// Get the alignment mode.
    pub fn mode(&self) -> AlignMode {
        self.mode
    }

    /// Get total blocks processed.
    pub fn blocks_processed(&self) -> u64 {
        self.blocks_processed
    }

    /// Get total blocks discarded.
    pub fn blocks_discarded(&self) -> u64 {
        self.blocks_discarded
    }

    /// Reset counters.
    pub fn reset(&mut self) {
        self.blocks_processed = 0;
        self.blocks_discarded = 0;
    }
}

/// Segment a continuous stream into aligned blocks.
pub fn segment_and_align(data: &[f64], block_size: usize, mode: AlignMode) -> Vec<Vec<f64>> {
    let aligner = TaggedStreamAlign::new(block_size, mode);
    let mut result = Vec::new();

    for chunk in data.chunks(block_size) {
        let aligned = aligner.align(chunk);
        if !aligned.is_empty() {
            result.push(aligned);
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_exact_size() {
        let aligner = TaggedStreamAlign::new(4, AlignMode::ZeroPad);
        let block = vec![1.0, 2.0, 3.0, 4.0];
        assert_eq!(aligner.align(&block), vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_zero_pad() {
        let aligner = TaggedStreamAlign::new(6, AlignMode::ZeroPad);
        let block = vec![1.0, 2.0, 3.0];
        let aligned = aligner.align(&block);
        assert_eq!(aligned, vec![1.0, 2.0, 3.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn test_cyclic_repeat() {
        let aligner = TaggedStreamAlign::new(7, AlignMode::CyclicRepeat);
        let block = vec![1.0, 2.0, 3.0];
        let aligned = aligner.align(&block);
        assert_eq!(aligned, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0]);
    }

    #[test]
    fn test_repeat_last() {
        let aligner = TaggedStreamAlign::new(5, AlignMode::RepeatLast);
        let block = vec![1.0, 2.0, 3.0];
        let aligned = aligner.align(&block);
        assert_eq!(aligned, vec![1.0, 2.0, 3.0, 3.0, 3.0]);
    }

    #[test]
    fn test_discard() {
        let aligner = TaggedStreamAlign::new(5, AlignMode::Discard);
        let block = vec![1.0, 2.0, 3.0];
        let aligned = aligner.align(&block);
        assert!(aligned.is_empty());
    }

    #[test]
    fn test_truncate() {
        let aligner = TaggedStreamAlign::new(3, AlignMode::ZeroPad);
        let block = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let aligned = aligner.align(&block);
        assert_eq!(aligned, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn test_no_truncate() {
        let mut aligner = TaggedStreamAlign::new(3, AlignMode::ZeroPad);
        aligner.set_truncate(false);
        let block = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let aligned = aligner.align(&block);
        assert_eq!(aligned.len(), 5);
    }

    #[test]
    fn test_complex_align() {
        let aligner = TaggedStreamAlign::new(4, AlignMode::ZeroPad);
        let block = vec![(1.0, 2.0), (3.0, 4.0)];
        let aligned = aligner.align_complex(&block);
        assert_eq!(aligned.len(), 4);
        assert_eq!(aligned[2], (0.0, 0.0));
    }

    #[test]
    fn test_batch_align() {
        let mut aligner = TaggedStreamAlign::new(4, AlignMode::Discard);
        let blocks = vec![
            vec![1.0, 2.0, 3.0, 4.0], // exact → kept
            vec![1.0, 2.0],             // short → discarded
            vec![1.0, 2.0, 3.0, 4.0, 5.0], // long → truncated to 4
        ];
        let result = aligner.align_batch(&blocks);
        assert_eq!(result.len(), 2);
        assert_eq!(aligner.blocks_processed(), 3);
        assert_eq!(aligner.blocks_discarded(), 1);
    }

    #[test]
    fn test_segment_and_align() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0];
        let blocks = segment_and_align(&data, 3, AlignMode::ZeroPad);
        assert_eq!(blocks.len(), 3);
        assert_eq!(blocks[0], vec![1.0, 2.0, 3.0]);
        assert_eq!(blocks[1], vec![4.0, 5.0, 6.0]);
        assert_eq!(blocks[2], vec![7.0, 0.0, 0.0]); // Padded
    }
}
