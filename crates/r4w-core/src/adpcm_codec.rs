//! Adaptive Differential Pulse Code Modulation (ADPCM) Codec
//!
//! Implements IMA/DVI ADPCM for bandwidth-efficient audio compression. ADPCM
//! encodes the difference between successive PCM samples using an adaptive
//! quantizer, achieving compression ratios of 2:1 to 8:1 depending on the
//! bits-per-sample setting. The IMA standard uses 4-bit codes with an 88-entry
//! step-size adaptation table.
//!
//! Supported bit depths:
//! - **4-bit** (IMA/DVI standard): 4:1 compression, good quality (~30 dB SNR)
//! - **3-bit**: ~5.3:1 compression, moderate quality
//! - **2-bit**: 8:1 compression, low bitrate voice
//!
//! # Example
//!
//! ```rust
//! use r4w_core::adpcm_codec::{AdpcmEncoder, AdpcmDecoder};
//!
//! let mut encoder = AdpcmEncoder::new(4);
//! let mut decoder = AdpcmDecoder::new(4);
//!
//! // Encode a simple ramp signal
//! let samples: Vec<i16> = (0..100).map(|i| (i * 100) as i16).collect();
//! let encoded = encoder.encode(&samples);
//! let decoded = decoder.decode(&encoded);
//!
//! // Decoded length matches original
//! assert_eq!(decoded.len(), samples.len());
//!
//! // Roundtrip is approximate (lossy compression)
//! let max_err: i16 = samples.iter().zip(decoded.iter())
//!     .map(|(a, b)| (a - b).abs())
//!     .max().unwrap_or(0);
//! assert!(max_err < 500, "max error too large: {}", max_err);
//! ```

/// IMA ADPCM step-size table (88 entries).
///
/// Each entry is the quantizer step size for the corresponding step index.
/// The table spans from 7 (finest quantization) to 32767 (coarsest).
const IMA_STEP_TABLE: [i32; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37,
    41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173,
    190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658,
    724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894,
    6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899, 15289,
    16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
];

/// IMA ADPCM index adjustment table for 4-bit codes.
///
/// Indexed by the ADPCM code (0-15). Codes 0-3 decrease the step index
/// (signal is getting quieter), codes 4-7 increase it (signal is getting louder).
const IMA_INDEX_TABLE_4BIT: [i32; 16] = [
    -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8,
];

/// Index adjustment table for 3-bit codes.
const IMA_INDEX_TABLE_3BIT: [i32; 8] = [-1, -1, 2, 4, -1, -1, 2, 4];

/// Index adjustment table for 2-bit codes.
const IMA_INDEX_TABLE_2BIT: [i32; 4] = [-1, 2, -1, 2];

/// Returns the index adjustment table for the given bits-per-sample.
fn index_table(bits_per_sample: usize) -> &'static [i32] {
    match bits_per_sample {
        2 => &IMA_INDEX_TABLE_2BIT,
        3 => &IMA_INDEX_TABLE_3BIT,
        4 => &IMA_INDEX_TABLE_4BIT,
        _ => &IMA_INDEX_TABLE_4BIT,
    }
}

/// Clamp the step index to valid range [0, 88].
fn clamp_index(index: i32) -> i32 {
    index.clamp(0, 88)
}

/// Clamp a predicted sample to i16 range.
fn clamp_sample(sample: i32) -> i32 {
    sample.clamp(i16::MIN as i32, i16::MAX as i32)
}

/// Reconstruct the quantized difference from an ADPCM code.
///
/// This is the shared reconstruction formula used by both encoder and decoder
/// to ensure they stay in sync. The formula is:
///   diff = step * (b2/1 + b1/2 + b0/4) + step/8  (for 4-bit, ignoring sign)
///
/// More generally, for N magnitude bits, the reconstruction sums step >> shift
/// for each set bit, plus a half-step at the end.
fn reconstruct_diff(code: u8, step: i32, bits_per_sample: usize) -> i32 {
    let mag_bits = bits_per_sample - 1;
    let mut diff: i32 = 0;
    let mut step_val = step;
    for i in (0..mag_bits).rev() {
        if code & (1 << i) != 0 {
            diff += step_val;
        }
        step_val >>= 1;
    }
    diff += step_val; // half-step rounding
    diff
}

/// ADPCM encoder that compresses 16-bit PCM samples to ADPCM nibbles.
#[derive(Debug, Clone)]
pub struct AdpcmEncoder {
    /// Bits per ADPCM sample (2, 3, or 4).
    bits_per_sample: usize,
    /// Current predicted sample value.
    predicted: i32,
    /// Current step index into `IMA_STEP_TABLE`.
    step_index: i32,
}

impl AdpcmEncoder {
    /// Create a new ADPCM encoder.
    ///
    /// - `bits_per_sample`: Number of bits per encoded sample (2, 3, or 4).
    ///   IMA/DVI ADPCM standard uses 4.
    pub fn new(bits_per_sample: usize) -> Self {
        assert!(
            (2..=4).contains(&bits_per_sample),
            "bits_per_sample must be 2, 3, or 4"
        );
        Self {
            bits_per_sample,
            predicted: 0,
            step_index: 0,
        }
    }

    /// Encode a slice of 16-bit PCM samples to packed ADPCM bytes.
    ///
    /// For 4-bit ADPCM, two nibbles are packed per byte (low nibble first).
    /// For 3-bit and 2-bit, codes are packed sequentially into bytes from LSB.
    pub fn encode(&mut self, samples: &[i16]) -> Vec<u8> {
        let codes: Vec<u8> = samples.iter().map(|&s| self.encode_sample(s)).collect();
        pack_codes(&codes, self.bits_per_sample, samples.len())
    }

    /// Reset the encoder to its initial state.
    pub fn reset(&mut self) {
        self.predicted = 0;
        self.step_index = 0;
    }

    /// Returns the current step index (useful for diagnostics).
    pub fn step_index(&self) -> i32 {
        self.step_index
    }

    /// Encode a single sample, returning the ADPCM code.
    fn encode_sample(&mut self, sample: i16) -> u8 {
        let step = IMA_STEP_TABLE[self.step_index as usize];
        let diff = sample as i32 - self.predicted;
        let sign_bit = 1u8 << (self.bits_per_sample - 1);
        let mag_bits = self.bits_per_sample - 1;

        // Determine sign
        let (sign, mut remaining) = if diff < 0 {
            (sign_bit, -diff)
        } else {
            (0u8, diff)
        };

        // Quantize the magnitude using successive approximation:
        // each bit tests whether the remaining difference exceeds the
        // current threshold, then subtracts it if so.
        let mut code: u8 = 0;
        let mut threshold = step;
        for i in (0..mag_bits).rev() {
            if remaining >= threshold {
                code |= 1 << i;
                remaining -= threshold;
            }
            threshold >>= 1;
        }

        let code = sign | code;

        // Reconstruct (must match decoder exactly)
        let diff_q = reconstruct_diff(code, step, self.bits_per_sample);

        if code & sign_bit != 0 {
            self.predicted -= diff_q;
        } else {
            self.predicted += diff_q;
        }
        self.predicted = clamp_sample(self.predicted);

        // Update step index
        let idx_table = index_table(self.bits_per_sample);
        self.step_index += idx_table[code as usize];
        self.step_index = clamp_index(self.step_index);

        code
    }
}

/// ADPCM decoder that decompresses ADPCM nibbles back to 16-bit PCM.
#[derive(Debug, Clone)]
pub struct AdpcmDecoder {
    /// Bits per ADPCM sample (2, 3, or 4).
    bits_per_sample: usize,
    /// Current predicted sample value.
    predicted: i32,
    /// Current step index into `IMA_STEP_TABLE`.
    step_index: i32,
}

impl AdpcmDecoder {
    /// Create a new ADPCM decoder.
    ///
    /// - `bits_per_sample`: Must match the encoder setting.
    pub fn new(bits_per_sample: usize) -> Self {
        assert!(
            (2..=4).contains(&bits_per_sample),
            "bits_per_sample must be 2, 3, or 4"
        );
        Self {
            bits_per_sample,
            predicted: 0,
            step_index: 0,
        }
    }

    /// Decode packed ADPCM bytes back to 16-bit PCM samples.
    ///
    /// - `data`: Packed ADPCM data (as produced by `AdpcmEncoder::encode`).
    ///
    /// The number of output samples is inferred from the data length and
    /// bits-per-sample. For 4-bit: 2 samples per byte. For 2-bit: 4 samples
    /// per byte.
    pub fn decode(&mut self, data: &[u8]) -> Vec<i16> {
        let codes = unpack_codes(data, self.bits_per_sample);
        codes.iter().map(|&c| self.decode_sample(c)).collect()
    }

    /// Decode packed ADPCM bytes, returning exactly `num_samples` PCM samples.
    pub fn decode_exact(&mut self, data: &[u8], num_samples: usize) -> Vec<i16> {
        let codes = unpack_codes(data, self.bits_per_sample);
        codes
            .iter()
            .take(num_samples)
            .map(|&c| self.decode_sample(c))
            .collect()
    }

    /// Reset the decoder to its initial state.
    pub fn reset(&mut self) {
        self.predicted = 0;
        self.step_index = 0;
    }

    /// Returns the current step index (useful for diagnostics).
    pub fn step_index(&self) -> i32 {
        self.step_index
    }

    /// Decode a single ADPCM code to a PCM sample.
    fn decode_sample(&mut self, code: u8) -> i16 {
        let step = IMA_STEP_TABLE[self.step_index as usize];
        let sign_bit = 1u8 << (self.bits_per_sample - 1);

        let diff_q = reconstruct_diff(code, step, self.bits_per_sample);

        if code & sign_bit != 0 {
            self.predicted -= diff_q;
        } else {
            self.predicted += diff_q;
        }
        self.predicted = clamp_sample(self.predicted);

        // Update step index
        let idx_table = index_table(self.bits_per_sample);
        self.step_index += idx_table[code as usize];
        self.step_index = clamp_index(self.step_index);

        self.predicted as i16
    }
}

// ---------------------------------------------------------------------------
// Bit packing helpers
// ---------------------------------------------------------------------------

/// Pack ADPCM codes into bytes.
///
/// For 4-bit: two nibbles per byte, low nibble first.
/// For 2-bit: four codes per byte, low bits first.
/// For 3-bit: packed sequentially from LSB across byte boundaries.
fn pack_codes(codes: &[u8], bits_per_sample: usize, _num_samples: usize) -> Vec<u8> {
    match bits_per_sample {
        4 => {
            let mut out = Vec::with_capacity((codes.len() + 1) / 2);
            for chunk in codes.chunks(2) {
                let lo = chunk[0] & 0x0F;
                let hi = if chunk.len() > 1 {
                    chunk[1] & 0x0F
                } else {
                    0
                };
                out.push(lo | (hi << 4));
            }
            out
        }
        2 => {
            let mut out = Vec::with_capacity((codes.len() + 3) / 4);
            for chunk in codes.chunks(4) {
                let mut byte = 0u8;
                for (i, &c) in chunk.iter().enumerate() {
                    byte |= (c & 0x03) << (i * 2);
                }
                out.push(byte);
            }
            out
        }
        3 => {
            // Pack 3-bit codes sequentially
            let total_bits = codes.len() * 3;
            let num_bytes = (total_bits + 7) / 8;
            let mut out = vec![0u8; num_bytes];
            let mut bit_pos = 0usize;
            for &code in codes {
                let c = code & 0x07;
                for b in 0..3 {
                    if c & (1 << b) != 0 {
                        let byte_idx = bit_pos / 8;
                        let bit_idx = bit_pos % 8;
                        out[byte_idx] |= 1 << bit_idx;
                    }
                    bit_pos += 1;
                }
            }
            out
        }
        _ => {
            // Fallback: treat as 4-bit
            pack_codes(codes, 4, _num_samples)
        }
    }
}

/// Unpack ADPCM codes from bytes.
fn unpack_codes(data: &[u8], bits_per_sample: usize) -> Vec<u8> {
    match bits_per_sample {
        4 => {
            let mut codes = Vec::with_capacity(data.len() * 2);
            for &byte in data {
                codes.push(byte & 0x0F);
                codes.push((byte >> 4) & 0x0F);
            }
            codes
        }
        2 => {
            let mut codes = Vec::with_capacity(data.len() * 4);
            for &byte in data {
                codes.push(byte & 0x03);
                codes.push((byte >> 2) & 0x03);
                codes.push((byte >> 4) & 0x03);
                codes.push((byte >> 6) & 0x03);
            }
            codes
        }
        3 => {
            let total_codes = (data.len() * 8) / 3;
            let mut codes = Vec::with_capacity(total_codes);
            let mut bit_pos = 0usize;
            for _ in 0..total_codes {
                let mut code = 0u8;
                for b in 0..3 {
                    let byte_idx = bit_pos / 8;
                    let bit_idx = bit_pos % 8;
                    if byte_idx < data.len() && data[byte_idx] & (1 << bit_idx) != 0 {
                        code |= 1 << b;
                    }
                    bit_pos += 1;
                }
                codes.push(code);
            }
            codes
        }
        _ => unpack_codes(data, 4),
    }
}

// ---------------------------------------------------------------------------
// Block-based encoding
// ---------------------------------------------------------------------------

/// Block-based ADPCM encoding for random-access and streaming.
///
/// Each block contains a 4-byte header (initial predictor as i16, initial step
/// index as u8, reserved byte) followed by packed ADPCM data.
pub struct AdpcmBlock;

impl AdpcmBlock {
    /// Block header size in bytes.
    pub const HEADER_SIZE: usize = 4;

    /// Encode a block of PCM samples.
    ///
    /// The block header stores the first sample value and the initial step
    /// index, allowing independent decoding of each block.
    ///
    /// - `samples`: PCM samples for this block.
    /// - `block_size`: Number of PCM samples per block. If `samples.len()` is
    ///   less than `block_size`, the remaining space is zero-padded.
    pub fn encode_block(samples: &[i16], block_size: usize) -> Vec<u8> {
        let bits_per_sample = 4usize;
        let actual_len = samples.len().min(block_size);

        // Header: first sample (i16 LE) + step_index (u8) + reserved (u8)
        let first_sample = if actual_len > 0 { samples[0] } else { 0i16 };
        let mut out = Vec::new();
        out.extend_from_slice(&first_sample.to_le_bytes());
        out.push(0u8); // initial step index
        out.push(0u8); // reserved

        // Encode remaining samples (skip first, which is stored verbatim)
        if actual_len > 1 {
            let mut encoder = AdpcmEncoder::new(bits_per_sample);
            encoder.predicted = first_sample as i32;
            let codes: Vec<u8> = samples[1..actual_len]
                .iter()
                .map(|&s| encoder.encode_sample(s))
                .collect();
            let packed = pack_codes(&codes, bits_per_sample, codes.len());
            out.extend_from_slice(&packed);
        }

        out
    }

    /// Decode a single ADPCM block back to PCM samples.
    ///
    /// Returns an empty vector if the block is too short to contain a header.
    pub fn decode_block(data: &[u8]) -> Vec<i16> {
        if data.len() < Self::HEADER_SIZE {
            return Vec::new();
        }

        let bits_per_sample = 4usize;

        // Parse header
        let first_sample = i16::from_le_bytes([data[0], data[1]]);
        let initial_step_index = data[2] as i32;

        let mut result = vec![first_sample];

        if data.len() > Self::HEADER_SIZE {
            let mut decoder = AdpcmDecoder::new(bits_per_sample);
            decoder.predicted = first_sample as i32;
            decoder.step_index = clamp_index(initial_step_index);

            let adpcm_data = &data[Self::HEADER_SIZE..];
            let codes = unpack_codes(adpcm_data, bits_per_sample);
            for &code in &codes {
                result.push(decoder.decode_sample(code));
            }
        }

        result
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute the compression ratio of ADPCM relative to 16-bit PCM.
///
/// For example, 4-bit ADPCM achieves 16/4 = 4.0 compression ratio.
///
/// - `bits_per_sample`: Bits per ADPCM code (2, 3, or 4).
pub fn compression_ratio(bits_per_sample: usize) -> f64 {
    16.0 / bits_per_sample as f64
}

/// Compute the Signal-to-Noise Ratio (SNR) in dB between original and
/// reconstructed audio.
///
/// SNR = 10 * log10(signal_power / noise_power)
///
/// where noise = original - decoded.
///
/// Returns `f64::INFINITY` if the signals are identical, or `f64::NEG_INFINITY`
/// if the original signal has zero power.
pub fn snr_estimate(original: &[i16], decoded: &[i16]) -> f64 {
    let n = original.len().min(decoded.len());
    if n == 0 {
        return 0.0;
    }

    let mut signal_power: f64 = 0.0;
    let mut noise_power: f64 = 0.0;

    for i in 0..n {
        let s = original[i] as f64;
        let d = decoded[i] as f64;
        signal_power += s * s;
        noise_power += (s - d) * (s - d);
    }

    if noise_power == 0.0 {
        return f64::INFINITY;
    }
    if signal_power == 0.0 {
        return f64::NEG_INFINITY;
    }

    10.0 * (signal_power / noise_power).log10()
}

// ---------------------------------------------------------------------------
// Factory functions
// ---------------------------------------------------------------------------

/// Create a matched IMA/DVI ADPCM encoder-decoder pair (4-bit, standard).
///
/// This is the most common ADPCM variant, used in WAV files (codec ID 0x0011),
/// telephony, and many embedded systems.
pub fn ima_adpcm() -> (AdpcmEncoder, AdpcmDecoder) {
    (AdpcmEncoder::new(4), AdpcmDecoder::new(4))
}

/// Create a matched 2-bit ADPCM encoder-decoder pair for very low bitrate.
///
/// Provides 8:1 compression (16-bit PCM to 2-bit ADPCM) at the cost of
/// lower audio quality. Suitable for voice-only applications where bandwidth
/// is extremely constrained.
pub fn adpcm_2bit() -> (AdpcmEncoder, AdpcmDecoder) {
    (AdpcmEncoder::new(2), AdpcmDecoder::new(2))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a sine-wave test signal at the given frequency and sample rate.
    fn sine_wave(freq_hz: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<i16> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (amplitude * (2.0 * std::f64::consts::PI * freq_hz * t).sin()) as i16
            })
            .collect()
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let mut encoder = AdpcmEncoder::new(4);
        let mut decoder = AdpcmDecoder::new(4);

        // 400 Hz tone at 8 kHz sample rate -- moderate frequency so the
        // predictor can track the waveform after a brief startup transient.
        let original = sine_wave(400.0, 8000.0, 500, 8000.0);
        let encoded = encoder.encode(&original);
        let decoded = decoder.decode_exact(&encoded, original.len());

        assert_eq!(decoded.len(), original.len());

        // Skip the first 20 samples (startup transient) and verify the
        // steady-state error is within a reasonable bound for 4-bit ADPCM.
        let skip = 20;
        let max_err: i32 = original[skip..]
            .iter()
            .zip(decoded[skip..].iter())
            .map(|(&a, &b)| (a as i32 - b as i32).abs())
            .max()
            .unwrap();
        assert!(
            max_err < 2000,
            "max steady-state roundtrip error too large: {}",
            max_err
        );
    }

    #[test]
    fn test_compression_ratio() {
        assert!((compression_ratio(4) - 4.0).abs() < f64::EPSILON);
        assert!((compression_ratio(2) - 8.0).abs() < f64::EPSILON);
        assert!((compression_ratio(3) - 16.0 / 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_snr_roundtrip() {
        let mut encoder = AdpcmEncoder::new(4);
        let mut decoder = AdpcmDecoder::new(4);

        // Single 400 Hz tone at 8 kHz for 2000 samples -- long enough to
        // amortize the startup transient and get steady-state SNR.
        let original = sine_wave(400.0, 8000.0, 2000, 8000.0);

        let encoded = encoder.encode(&original);
        let decoded = decoder.decode_exact(&encoded, original.len());

        // Skip the first 50 samples (startup transient) for the SNR measurement
        let skip = 50;
        let snr = snr_estimate(&original[skip..], &decoded[skip..]);

        assert!(
            snr > 20.0,
            "SNR should be > 20 dB for 4-bit ADPCM, got {:.1} dB",
            snr
        );
    }

    #[test]
    fn test_silence_encodes_small() {
        let mut encoder = AdpcmEncoder::new(4);

        let silence = vec![0i16; 100];
        let encoded = encoder.encode(&silence);

        // Silence should encode to all-zero codes (packed as 0x00 bytes)
        for &byte in &encoded {
            assert_eq!(
                byte, 0x00,
                "silence should encode to zero codes, got 0x{:02x}",
                byte
            );
        }
    }

    #[test]
    fn test_step_size_adapts() {
        let mut encoder = AdpcmEncoder::new(4);

        // Encode silence first -- step index should stay low
        let silence = vec![0i16; 20];
        encoder.encode(&silence);
        let index_after_silence = encoder.step_index();

        // Now encode a loud signal -- step index should increase
        encoder.reset();
        let loud: Vec<i16> = (0..20)
            .map(|i| if i % 2 == 0 { 20000 } else { -20000 })
            .collect();
        encoder.encode(&loud);
        let index_after_loud = encoder.step_index();

        assert!(
            index_after_loud > index_after_silence,
            "step index should adapt to larger signals: silence={}, loud={}",
            index_after_silence,
            index_after_loud
        );
    }

    #[test]
    fn test_reset_restores_initial_state() {
        let mut encoder = AdpcmEncoder::new(4);
        let mut decoder = AdpcmDecoder::new(4);

        // Process some data
        let signal = sine_wave(500.0, 8000.0, 100, 5000.0);
        encoder.encode(&signal);
        let enc_data = encoder.encode(&signal);
        decoder.decode(&enc_data);

        // Reset both
        encoder.reset();
        decoder.reset();

        assert_eq!(encoder.step_index(), 0);
        assert_eq!(decoder.step_index(), 0);

        // After reset, encoding the same signal should produce the same output
        // as a fresh encoder
        let mut fresh_encoder = AdpcmEncoder::new(4);
        let out_reset = encoder.encode(&signal);
        let out_fresh = fresh_encoder.encode(&signal);
        assert_eq!(out_reset, out_fresh, "reset encoder should match fresh encoder");
    }

    #[test]
    fn test_ima_factory_produces_4bit() {
        let (mut enc, mut dec) = ima_adpcm();

        let signal = sine_wave(1000.0, 8000.0, 100, 8000.0);
        let encoded = enc.encode(&signal);
        let decoded = dec.decode_exact(&encoded, signal.len());

        // 4-bit: 2 samples per byte
        assert_eq!(encoded.len(), (signal.len() + 1) / 2);
        assert_eq!(decoded.len(), signal.len());

        // Verify it's 4-bit by checking compression ratio
        assert_eq!(enc.bits_per_sample, 4);
        assert_eq!(dec.bits_per_sample, 4);
    }

    #[test]
    fn test_2bit_factory_produces_2bit() {
        let (mut enc, mut dec) = adpcm_2bit();

        let signal = sine_wave(1000.0, 8000.0, 100, 8000.0);
        let encoded = enc.encode(&signal);
        let decoded = dec.decode_exact(&encoded, signal.len());

        // 2-bit: 4 samples per byte
        assert_eq!(encoded.len(), (signal.len() + 3) / 4);
        assert_eq!(decoded.len(), signal.len());

        assert_eq!(enc.bits_per_sample, 2);
        assert_eq!(dec.bits_per_sample, 2);
    }

    #[test]
    fn test_block_encode_decode() {
        // Use a moderate-frequency tone so the predictor can adapt quickly.
        let signal = sine_wave(400.0, 8000.0, 256, 8000.0);
        let block_size = 256;

        let block_data = AdpcmBlock::encode_block(&signal, block_size);
        let decoded = AdpcmBlock::decode_block(&block_data);

        // First sample should be stored verbatim in the header
        assert_eq!(decoded[0], signal[0]);

        // Compare up to original length, skipping first 20 samples for
        // startup transient (the predictor must adapt from 0 step index).
        let compare_len = signal.len().min(decoded.len());
        let skip = 20;
        let snr = snr_estimate(&signal[skip..compare_len], &decoded[skip..compare_len]);
        assert!(
            snr > 15.0,
            "block SNR should be > 15 dB, got {:.1} dB",
            snr
        );
    }

    #[test]
    fn test_different_bit_depths_different_quality() {
        let signal = sine_wave(1000.0, 8000.0, 500, 10000.0);

        // 4-bit encode/decode
        let (mut enc4, mut dec4) = ima_adpcm();
        let encoded4 = enc4.encode(&signal);
        let decoded4 = dec4.decode_exact(&encoded4, signal.len());
        let snr4 = snr_estimate(&signal, &decoded4);

        // 2-bit encode/decode
        let (mut enc2, mut dec2) = adpcm_2bit();
        let encoded2 = enc2.encode(&signal);
        let decoded2 = dec2.decode_exact(&encoded2, signal.len());
        let snr2 = snr_estimate(&signal, &decoded2);

        // 4-bit should have better quality (higher SNR) than 2-bit
        assert!(
            snr4 > snr2,
            "4-bit SNR ({:.1} dB) should be higher than 2-bit SNR ({:.1} dB)",
            snr4,
            snr2
        );

        // 4-bit should use more bytes than 2-bit
        assert!(
            encoded4.len() > encoded2.len(),
            "4-bit should produce more bytes ({}) than 2-bit ({})",
            encoded4.len(),
            encoded2.len()
        );
    }
}
