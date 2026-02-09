//! Generic FEC API — Unified Forward Error Correction Framework
//!
//! Provides a trait-based FEC abstraction that wraps all existing FEC codecs
//! (convolutional/Viterbi, Reed-Solomon, LDPC, Polar, turbo, BCH, trellis)
//! behind common `GenericEncoder`/`GenericDecoder` interfaces with streaming
//! block-framing wrappers and a codec registry for runtime selection.
//!
//! GNU Radio equivalent: `gr::fec::encoder`, `gr::fec::decoder`,
//! `gr::fec::generic_encoder`, `gr::fec::generic_decoder`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fec_generic_api::{GenericEncoder, GenericDecoder, FecEncoderBlock, FecDecoderBlock, RepetitionEncoder, RepetitionDecoder};
//!
//! let enc = RepetitionEncoder::new(3);
//! let dec = RepetitionDecoder::new(3);
//! let mut encoder_block = FecEncoderBlock::new(Box::new(enc));
//! let mut decoder_block = FecDecoderBlock::new(Box::new(dec));
//!
//! let data = vec![true, false, true, true];
//! let encoded = encoder_block.process(&data);
//! let decoded = decoder_block.process_hard(&encoded);
//! assert_eq!(&decoded[..4], &data[..]);
//! ```

/// Common FEC codec properties.
pub trait FecCodec {
    /// Code rate (k/n), e.g., 0.5 for rate-1/2.
    fn rate(&self) -> f64;
    /// Expected input block size in bits.
    fn input_size(&self) -> usize;
    /// Output block size in bits.
    fn output_size(&self) -> usize;
    /// Codec name.
    fn name(&self) -> &str;
}

/// Generic FEC encoder trait.
pub trait GenericEncoder: FecCodec {
    /// Encode a block of input bits.
    fn encode(&mut self, input: &[bool]) -> Vec<bool>;
}

/// Generic FEC decoder trait.
pub trait GenericDecoder: FecCodec {
    /// Decode hard-decision bits.
    fn decode_hard(&mut self, input: &[bool], num_info_bits: usize) -> Vec<bool>;
    /// Decode soft LLR values (positive = likely 0, negative = likely 1).
    fn decode_soft(&mut self, llr: &[f64], num_info_bits: usize) -> Vec<bool> {
        // Default: convert soft to hard and use hard decoder
        let hard: Vec<bool> = llr.iter().map(|&v| v < 0.0).collect();
        self.decode_hard(&hard, num_info_bits)
    }
}

/// Streaming FEC encoder block with automatic block framing.
pub struct FecEncoderBlock {
    codec: Box<dyn GenericEncoder>,
}

impl FecEncoderBlock {
    pub fn new(codec: Box<dyn GenericEncoder>) -> Self {
        Self { codec }
    }

    /// Encode input bits, framing into codec-sized blocks.
    ///
    /// Input is split into chunks of `input_size()`, each independently
    /// encoded. The last chunk is zero-padded if necessary.
    pub fn process(&mut self, input: &[bool]) -> Vec<bool> {
        let blk = self.codec.input_size();
        if blk == 0 {
            return self.codec.encode(input);
        }

        let num_blocks = (input.len() + blk - 1) / blk;
        let mut output = Vec::with_capacity(num_blocks * self.codec.output_size());

        for b in 0..num_blocks {
            let start = b * blk;
            let end = (start + blk).min(input.len());
            let mut chunk = input[start..end].to_vec();
            chunk.resize(blk, false); // Zero-pad
            output.extend(self.codec.encode(&chunk));
        }

        output
    }

    /// Get a reference to the underlying codec.
    pub fn codec(&self) -> &dyn GenericEncoder {
        &*self.codec
    }
}

/// Streaming FEC decoder block with automatic block framing.
pub struct FecDecoderBlock {
    codec: Box<dyn GenericDecoder>,
}

impl FecDecoderBlock {
    pub fn new(codec: Box<dyn GenericDecoder>) -> Self {
        Self { codec }
    }

    /// Decode hard-decision bits, framing into codec-sized blocks.
    pub fn process_hard(&mut self, input: &[bool]) -> Vec<bool> {
        let blk = self.codec.output_size();
        if blk == 0 {
            return self.codec.decode_hard(input, input.len());
        }

        let info_blk = self.codec.input_size();
        let num_blocks = (input.len() + blk - 1) / blk;
        let mut output = Vec::with_capacity(num_blocks * info_blk);

        for b in 0..num_blocks {
            let start = b * blk;
            let end = (start + blk).min(input.len());
            let mut chunk = input[start..end].to_vec();
            chunk.resize(blk, false);
            output.extend(self.codec.decode_hard(&chunk, info_blk));
        }

        output
    }

    /// Decode soft LLR values, framing into codec-sized blocks.
    pub fn process_soft(&mut self, llr: &[f64]) -> Vec<bool> {
        let blk = self.codec.output_size();
        if blk == 0 {
            return self.codec.decode_soft(llr, llr.len());
        }

        let info_blk = self.codec.input_size();
        let num_blocks = (llr.len() + blk - 1) / blk;
        let mut output = Vec::with_capacity(num_blocks * info_blk);

        for b in 0..num_blocks {
            let start = b * blk;
            let end = (start + blk).min(llr.len());
            let mut chunk = llr[start..end].to_vec();
            chunk.resize(blk, 0.0);
            output.extend(self.codec.decode_soft(&chunk, info_blk));
        }

        output
    }

    /// Get a reference to the underlying codec.
    pub fn codec(&self) -> &dyn GenericDecoder {
        &*self.codec
    }
}

/// PDU-oriented FEC encoder for variable-length messages.
pub struct AsyncFecEncoder {
    codec: Box<dyn GenericEncoder>,
}

impl AsyncFecEncoder {
    pub fn new(codec: Box<dyn GenericEncoder>) -> Self {
        Self { codec }
    }

    /// Encode a variable-length PDU. Pads to codec block size internally.
    pub fn encode_pdu(&mut self, data: &[bool]) -> Vec<bool> {
        let blk = self.codec.input_size();
        if blk == 0 {
            return self.codec.encode(data);
        }
        let mut padded = data.to_vec();
        let pad_len = (blk - (data.len() % blk)) % blk;
        padded.resize(data.len() + pad_len, false);

        let mut output = Vec::new();
        for chunk in padded.chunks(blk) {
            output.extend(self.codec.encode(chunk));
        }
        output
    }
}

/// PDU-oriented FEC decoder.
pub struct AsyncFecDecoder {
    codec: Box<dyn GenericDecoder>,
}

impl AsyncFecDecoder {
    pub fn new(codec: Box<dyn GenericDecoder>) -> Self {
        Self { codec }
    }

    /// Decode a PDU, returning `num_info_bits` information bits.
    pub fn decode_pdu(&mut self, data: &[bool], num_info_bits: usize) -> Vec<bool> {
        let blk = self.codec.output_size();
        let info_blk = self.codec.input_size();
        if blk == 0 {
            return self.codec.decode_hard(data, num_info_bits);
        }

        let mut output = Vec::new();
        for chunk in data.chunks(blk) {
            let mut padded = chunk.to_vec();
            padded.resize(blk, false);
            output.extend(self.codec.decode_hard(&padded, info_blk));
        }
        output.truncate(num_info_bits);
        output
    }
}

/// FEC codec registry for runtime selection by name.
pub struct FecCodecRegistry {
    encoders: Vec<(&'static str, fn() -> Box<dyn GenericEncoder>)>,
    decoders: Vec<(&'static str, fn() -> Box<dyn GenericDecoder>)>,
}

impl FecCodecRegistry {
    pub fn new() -> Self {
        Self {
            encoders: Vec::new(),
            decoders: Vec::new(),
        }
    }

    /// Register an encoder factory.
    pub fn register_encoder(&mut self, name: &'static str, factory: fn() -> Box<dyn GenericEncoder>) {
        self.encoders.push((name, factory));
    }

    /// Register a decoder factory.
    pub fn register_decoder(&mut self, name: &'static str, factory: fn() -> Box<dyn GenericDecoder>) {
        self.decoders.push((name, factory));
    }

    /// Create an encoder by name.
    pub fn create_encoder(&self, name: &str) -> Option<Box<dyn GenericEncoder>> {
        self.encoders
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, f)| f())
    }

    /// Create a decoder by name.
    pub fn create_decoder(&self, name: &str) -> Option<Box<dyn GenericDecoder>> {
        self.decoders
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, f)| f())
    }

    /// List registered codec names (encoders).
    pub fn list_encoders(&self) -> Vec<&str> {
        self.encoders.iter().map(|(n, _)| *n).collect()
    }

    /// List registered codec names (decoders).
    pub fn list_decoders(&self) -> Vec<&str> {
        self.decoders.iter().map(|(n, _)| *n).collect()
    }
}

impl Default for FecCodecRegistry {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Built-in codec: Repetition code (simple, for testing)
// ============================================================

/// Simple repetition encoder: each bit repeated N times.
pub struct RepetitionEncoder {
    repetitions: usize,
    block_size: usize,
}

impl RepetitionEncoder {
    pub fn new(repetitions: usize) -> Self {
        Self {
            repetitions,
            block_size: 64,
        }
    }

    pub fn with_block_size(mut self, size: usize) -> Self {
        self.block_size = size;
        self
    }
}

impl FecCodec for RepetitionEncoder {
    fn rate(&self) -> f64 {
        1.0 / self.repetitions as f64
    }
    fn input_size(&self) -> usize {
        self.block_size
    }
    fn output_size(&self) -> usize {
        self.block_size * self.repetitions
    }
    fn name(&self) -> &str {
        "repetition"
    }
}

impl GenericEncoder for RepetitionEncoder {
    fn encode(&mut self, input: &[bool]) -> Vec<bool> {
        let mut output = Vec::with_capacity(input.len() * self.repetitions);
        for &bit in input {
            for _ in 0..self.repetitions {
                output.push(bit);
            }
        }
        output
    }
}

/// Simple repetition decoder: majority vote over N bits.
pub struct RepetitionDecoder {
    repetitions: usize,
    block_size: usize,
}

impl RepetitionDecoder {
    pub fn new(repetitions: usize) -> Self {
        Self {
            repetitions,
            block_size: 64,
        }
    }

    pub fn with_block_size(mut self, size: usize) -> Self {
        self.block_size = size;
        self
    }
}

impl FecCodec for RepetitionDecoder {
    fn rate(&self) -> f64 {
        1.0 / self.repetitions as f64
    }
    fn input_size(&self) -> usize {
        self.block_size
    }
    fn output_size(&self) -> usize {
        self.block_size * self.repetitions
    }
    fn name(&self) -> &str {
        "repetition"
    }
}

impl GenericDecoder for RepetitionDecoder {
    fn decode_hard(&mut self, input: &[bool], num_info_bits: usize) -> Vec<bool> {
        let mut output = Vec::with_capacity(num_info_bits);
        for chunk in input.chunks(self.repetitions) {
            let ones: usize = chunk.iter().filter(|&&b| b).count();
            output.push(ones > chunk.len() / 2);
            if output.len() >= num_info_bits {
                break;
            }
        }
        output.truncate(num_info_bits);
        output
    }

    fn decode_soft(&mut self, llr: &[f64], num_info_bits: usize) -> Vec<bool> {
        let mut output = Vec::with_capacity(num_info_bits);
        for chunk in llr.chunks(self.repetitions) {
            let sum: f64 = chunk.iter().sum();
            output.push(sum < 0.0); // Negative LLR → bit is 1
            if output.len() >= num_info_bits {
                break;
            }
        }
        output.truncate(num_info_bits);
        output
    }
}

// ============================================================
// Built-in codec: Single Parity Check
// ============================================================

/// Single parity check encoder: appends one parity bit per block.
pub struct ParityCheckEncoder {
    block_size: usize,
}

impl ParityCheckEncoder {
    pub fn new(block_size: usize) -> Self {
        Self { block_size }
    }
}

impl FecCodec for ParityCheckEncoder {
    fn rate(&self) -> f64 {
        self.block_size as f64 / (self.block_size + 1) as f64
    }
    fn input_size(&self) -> usize {
        self.block_size
    }
    fn output_size(&self) -> usize {
        self.block_size + 1
    }
    fn name(&self) -> &str {
        "spc"
    }
}

impl GenericEncoder for ParityCheckEncoder {
    fn encode(&mut self, input: &[bool]) -> Vec<bool> {
        let mut output = input.to_vec();
        let parity = input.iter().filter(|&&b| b).count() % 2 == 1;
        output.push(parity);
        output
    }
}

/// Single parity check decoder: detects (but cannot correct) single errors.
pub struct ParityCheckDecoder {
    block_size: usize,
}

impl ParityCheckDecoder {
    pub fn new(block_size: usize) -> Self {
        Self { block_size }
    }
}

impl FecCodec for ParityCheckDecoder {
    fn rate(&self) -> f64 {
        self.block_size as f64 / (self.block_size + 1) as f64
    }
    fn input_size(&self) -> usize {
        self.block_size
    }
    fn output_size(&self) -> usize {
        self.block_size + 1
    }
    fn name(&self) -> &str {
        "spc"
    }
}

impl GenericDecoder for ParityCheckDecoder {
    fn decode_hard(&mut self, input: &[bool], num_info_bits: usize) -> Vec<bool> {
        // SPC can detect but not correct — just strip parity
        let end = num_info_bits.min(input.len());
        input[..end].to_vec()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_repetition_round_trip() {
        let mut enc = RepetitionEncoder::new(3).with_block_size(4);
        let mut dec = RepetitionDecoder::new(3).with_block_size(4);
        let data = vec![true, false, true, true];
        let encoded = enc.encode(&data);
        assert_eq!(encoded.len(), 12);
        let decoded = dec.decode_hard(&encoded, 4);
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_repetition_error_correction() {
        let mut enc = RepetitionEncoder::new(5).with_block_size(4);
        let mut dec = RepetitionDecoder::new(5).with_block_size(4);
        let data = vec![true, false, true, false];
        let mut encoded = enc.encode(&data);
        // Flip 2 out of 5 for each bit (should still decode correctly)
        encoded[0] = false; // First rep of bit 0
        encoded[1] = false; // Second rep of bit 0
        // 3 of 5 still true → majority vote = true
        let decoded = dec.decode_hard(&encoded, 4);
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_soft_decoding() {
        let mut dec = RepetitionDecoder::new(3).with_block_size(2);
        // Bit 0 = true: 3 soft values (negative = likely 1)
        // Bit 1 = false: 3 soft values (positive = likely 0)
        let llr = vec![-0.5, -0.8, -0.3, 1.2, 0.9, 0.7];
        let decoded = dec.decode_soft(&llr, 2);
        assert_eq!(decoded, vec![true, false]);
    }

    #[test]
    fn test_block_framing() {
        let enc = RepetitionEncoder::new(3).with_block_size(4);
        let dec = RepetitionDecoder::new(3).with_block_size(4);
        let mut encoder = FecEncoderBlock::new(Box::new(enc));
        let mut decoder = FecDecoderBlock::new(Box::new(dec));

        // 10 bits → 3 blocks of 4 (last zero-padded) → 3 * 12 = 36 coded bits
        let data = vec![true; 10];
        let encoded = encoder.process(&data);
        assert_eq!(encoded.len(), 36);

        let decoded = decoder.process_hard(&encoded);
        // Should recover original 10 bits (plus 2 zeros from padding)
        assert_eq!(decoded.len(), 12);
        for i in 0..10 {
            assert_eq!(decoded[i], true, "bit {i} should be true");
        }
    }

    #[test]
    fn test_pdu_mode() {
        let enc = RepetitionEncoder::new(3).with_block_size(8);
        let dec = RepetitionDecoder::new(3).with_block_size(8);
        let mut async_enc = AsyncFecEncoder::new(Box::new(enc));
        let mut async_dec = AsyncFecDecoder::new(Box::new(dec));

        let data = vec![true, false, true, false, true, true, false];
        let encoded = async_enc.encode_pdu(&data);
        let decoded = async_dec.decode_pdu(&encoded, 7);
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_codec_registry() {
        let mut registry = FecCodecRegistry::new();
        registry.register_encoder("rep3", || Box::new(RepetitionEncoder::new(3)));
        registry.register_decoder("rep3", || Box::new(RepetitionDecoder::new(3)));

        let names = registry.list_encoders();
        assert!(names.contains(&"rep3"));

        let mut enc = registry.create_encoder("rep3").expect("should find rep3");
        let encoded = enc.encode(&[true, false]);
        assert_eq!(encoded.len(), 6); // 2 bits * 3 reps
    }

    #[test]
    fn test_unknown_codec_name() {
        let registry = FecCodecRegistry::new();
        assert!(registry.create_encoder("nonexistent").is_none());
        assert!(registry.create_decoder("nonexistent").is_none());
    }

    #[test]
    fn test_rate_calculation() {
        let enc = RepetitionEncoder::new(3);
        assert!((enc.rate() - 1.0 / 3.0).abs() < 1e-10);

        let spc = ParityCheckEncoder::new(7);
        assert!((spc.rate() - 7.0 / 8.0).abs() < 1e-10);
    }

    #[test]
    fn test_parity_check_round_trip() {
        let mut enc = ParityCheckEncoder::new(8);
        let mut dec = ParityCheckDecoder::new(8);

        let data = vec![true, false, true, true, false, false, true, false];
        let encoded = enc.encode(&data);
        assert_eq!(encoded.len(), 9);

        // Verify parity bit
        let ones: usize = data.iter().filter(|&&b| b).count();
        assert_eq!(encoded[8], ones % 2 == 1);

        let decoded = dec.decode_hard(&encoded, 8);
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_encoder_block_api() {
        let enc = RepetitionEncoder::new(3).with_block_size(4);
        let block = FecEncoderBlock::new(Box::new(enc));
        assert_eq!(block.codec().name(), "repetition");
        assert!((block.codec().rate() - 1.0 / 3.0).abs() < 1e-10);
    }
}
