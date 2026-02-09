//! Tagged Stream Multiply Length — Scale length tags for rate changes
//!
//! Adjusts the packet-length tag value in a tagged stream when the
//! sample rate or item size changes. After interpolation by L, a
//! packet of N items becomes L*N items; after decimation by M, it
//! becomes N/M. This block updates the length tag accordingly.
//! GNU Radio equivalent: `tagged_stream_multiply_length`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::tagged_stream_multiply_length::{
//!     multiply_length_tag, scale_length_tags,
//! };
//!
//! // After 4x interpolation, packet of 100 samples → 400
//! let new_len = multiply_length_tag(100, 4.0);
//! assert_eq!(new_len, 400);
//!
//! // After 2x decimation, packet of 100 → 50
//! let new_len = multiply_length_tag(100, 0.5);
//! assert_eq!(new_len, 50);
//! ```

/// A length tag with an offset and value.
#[derive(Debug, Clone, PartialEq)]
pub struct LengthTag {
    /// Sample offset where the tag applies.
    pub offset: u64,
    /// Tag key name.
    pub key: String,
    /// Packet length value.
    pub length: usize,
}

impl LengthTag {
    /// Create a new length tag.
    pub fn new(offset: u64, key: &str, length: usize) -> Self {
        Self {
            offset,
            key: key.to_string(),
            length,
        }
    }

    /// Create a scaled copy of this tag.
    pub fn scaled(&self, factor: f64) -> Self {
        Self {
            offset: (self.offset as f64 * factor) as u64,
            key: self.key.clone(),
            length: (self.length as f64 * factor).round() as usize,
        }
    }
}

/// Scale a packet length by a rate factor.
///
/// `original_len * factor`, rounded to nearest integer.
pub fn multiply_length_tag(original_len: usize, factor: f64) -> usize {
    (original_len as f64 * factor).round() as usize
}

/// Scale a vector of length tags by a rate factor.
///
/// Both offsets and lengths are adjusted.
pub fn scale_length_tags(tags: &[LengthTag], factor: f64) -> Vec<LengthTag> {
    tags.iter().map(|t| t.scaled(factor)).collect()
}

/// Tagged stream length scaler (stateful block).
#[derive(Debug, Clone)]
pub struct TaggedStreamMultiplyLength {
    /// Scale factor (output_rate / input_rate).
    factor: f64,
    /// Tag key to look for.
    length_tag_key: String,
}

impl TaggedStreamMultiplyLength {
    /// Create with a scale factor and tag key.
    ///
    /// - `factor`: output_items / input_items (e.g., 4.0 for 4x interpolation)
    /// - `length_tag_key`: the tag key name to scale (e.g., "packet_len")
    pub fn new(factor: f64, length_tag_key: &str) -> Self {
        Self {
            factor,
            length_tag_key: length_tag_key.to_string(),
        }
    }

    /// Process a vector of tags, scaling those matching the length tag key.
    pub fn process(&self, tags: &[LengthTag]) -> Vec<LengthTag> {
        tags.iter()
            .map(|t| {
                if t.key == self.length_tag_key {
                    t.scaled(self.factor)
                } else {
                    // Non-length tags: scale offset only, keep value
                    LengthTag {
                        offset: (t.offset as f64 * self.factor) as u64,
                        key: t.key.clone(),
                        length: t.length,
                    }
                }
            })
            .collect()
    }

    /// Get the scale factor.
    pub fn factor(&self) -> f64 {
        self.factor
    }

    /// Set a new scale factor.
    pub fn set_factor(&mut self, factor: f64) {
        self.factor = factor;
    }

    /// Get the length tag key.
    pub fn length_tag_key(&self) -> &str {
        &self.length_tag_key
    }
}

/// Compute the number of output samples for a given input packet.
pub fn output_packet_len(input_len: usize, interp: usize, decim: usize) -> usize {
    if decim == 0 {
        return 0;
    }
    (input_len * interp + decim - 1) / decim
}

/// Verify that a tagged stream's length tags are consistent with data length.
///
/// Returns `true` if the sum of all packet lengths equals `total_samples`.
pub fn verify_length_tags(tags: &[LengthTag], total_samples: usize) -> bool {
    let sum: usize = tags.iter().map(|t| t.length).sum();
    sum == total_samples
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_multiply_length_interp() {
        assert_eq!(multiply_length_tag(100, 4.0), 400);
    }

    #[test]
    fn test_multiply_length_decim() {
        assert_eq!(multiply_length_tag(100, 0.5), 50);
    }

    #[test]
    fn test_multiply_length_identity() {
        assert_eq!(multiply_length_tag(100, 1.0), 100);
    }

    #[test]
    fn test_multiply_fractional() {
        // 100 * 1.5 = 150
        assert_eq!(multiply_length_tag(100, 1.5), 150);
    }

    #[test]
    fn test_length_tag_scaled() {
        let tag = LengthTag::new(10, "pkt_len", 50);
        let scaled = tag.scaled(2.0);
        assert_eq!(scaled.offset, 20);
        assert_eq!(scaled.length, 100);
        assert_eq!(scaled.key, "pkt_len");
    }

    #[test]
    fn test_scale_length_tags() {
        let tags = vec![
            LengthTag::new(0, "len", 100),
            LengthTag::new(100, "len", 100),
        ];
        let scaled = scale_length_tags(&tags, 0.5);
        assert_eq!(scaled[0].offset, 0);
        assert_eq!(scaled[0].length, 50);
        assert_eq!(scaled[1].offset, 50);
        assert_eq!(scaled[1].length, 50);
    }

    #[test]
    fn test_stateful_block() {
        let block = TaggedStreamMultiplyLength::new(3.0, "pkt_len");
        let tags = vec![
            LengthTag::new(0, "pkt_len", 10),
            LengthTag::new(10, "other_tag", 99),
        ];
        let out = block.process(&tags);
        assert_eq!(out[0].length, 30); // Scaled
        assert_eq!(out[0].offset, 0);
        assert_eq!(out[1].length, 99); // Not scaled (different key)
        assert_eq!(out[1].offset, 30); // Offset still scaled
    }

    #[test]
    fn test_set_factor() {
        let mut block = TaggedStreamMultiplyLength::new(2.0, "len");
        assert_eq!(block.factor(), 2.0);
        block.set_factor(0.25);
        assert_eq!(block.factor(), 0.25);
    }

    #[test]
    fn test_output_packet_len() {
        // 100 input, 4x interp, no decim
        assert_eq!(output_packet_len(100, 4, 1), 400);
        // 100 input, no interp, 2x decim
        assert_eq!(output_packet_len(100, 1, 2), 50);
        // Rational: 48000→44100 is 147/160
        assert_eq!(output_packet_len(160, 147, 160), 147);
    }

    #[test]
    fn test_output_packet_len_zero_decim() {
        assert_eq!(output_packet_len(100, 4, 0), 0);
    }

    #[test]
    fn test_verify_length_tags() {
        let tags = vec![
            LengthTag::new(0, "len", 50),
            LengthTag::new(50, "len", 50),
        ];
        assert!(verify_length_tags(&tags, 100));
        assert!(!verify_length_tags(&tags, 99));
    }

    #[test]
    fn test_verify_empty() {
        assert!(verify_length_tags(&[], 0));
        assert!(!verify_length_tags(&[], 10));
    }

    #[test]
    fn test_accessors() {
        let block = TaggedStreamMultiplyLength::new(2.0, "my_key");
        assert_eq!(block.length_tag_key(), "my_key");
        assert_eq!(block.factor(), 2.0);
    }
}
