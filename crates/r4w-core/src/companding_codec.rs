//! Non-uniform quantization codec for dynamic range compression.
//!
//! Implements A-law and µ-law companding per ITU-T G.711, used in telephony
//! and audio systems to compress the dynamic range of speech signals into 8-bit
//! encoded samples. Both laws provide approximately uniform SNR across a wide
//! range of input amplitudes, unlike linear quantization which favors loud signals.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::companding_codec::{CompandingCodec, CompandingLaw};
//!
//! // Create a µ-law codec (North America / Japan standard)
//! let mu_codec = CompandingCodec::new(CompandingLaw::MuLaw);
//!
//! // Encode and decode a PCM sample
//! let original: i16 = 1000;
//! let compressed: u8 = mu_codec.encode_sample(original);
//! let restored: i16 = mu_codec.decode_sample(compressed);
//!
//! // Roundtrip introduces small quantization error
//! assert!((original - restored).abs() < 50);
//!
//! // Batch encode/decode
//! let pcm = vec![0i16, 100, -100, 1000, -8000, 32767];
//! let encoded = mu_codec.encode(&pcm);
//! let decoded = mu_codec.decode(&encoded);
//! assert_eq!(pcm.len(), decoded.len());
//!
//! // A-law codec (European standard)
//! let a_codec = CompandingCodec::new(CompandingLaw::ALaw);
//! let compressed_a = a_codec.encode_sample(1000);
//! let restored_a = a_codec.decode_sample(compressed_a);
//! assert!((1000 - restored_a).abs() < 80);
//! ```

/// Selects the companding law to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompandingLaw {
    /// ITU-T G.711 A-law (used in Europe and most of the world).
    ALaw,
    /// ITU-T G.711 µ-law (used in North America and Japan).
    MuLaw,
}

/// Configuration parameters for the companding codec.
#[derive(Debug, Clone)]
pub struct CompandingConfig {
    /// Which companding law to use.
    pub law: CompandingLaw,
    /// µ parameter for µ-law companding (default: 255.0).
    pub mu_value: f64,
    /// A parameter for A-law companding (default: 87.6).
    pub a_value: f64,
}

impl Default for CompandingConfig {
    fn default() -> Self {
        Self {
            law: CompandingLaw::MuLaw,
            mu_value: 255.0,
            a_value: 87.6,
        }
    }
}

/// Non-uniform quantization codec implementing G.711 A-law and µ-law.
///
/// Companding (compressing + expanding) maps a wide dynamic range into a
/// smaller number of bits while maintaining approximately constant
/// signal-to-quantization-noise ratio across input levels.
#[derive(Debug, Clone)]
pub struct CompandingCodec {
    config: CompandingConfig,
}

// ---------- µ-law constants (ITU-T G.711) ----------

/// Bias added before µ-law encoding.
const MULAW_BIAS: i32 = 0x84;

/// Maximum µ-law input magnitude (clips above this).
const MULAW_CLIP: i32 = 32635;

impl CompandingCodec {
    /// Create a new codec with default parameters for the given law.
    pub fn new(law: CompandingLaw) -> Self {
        Self {
            config: CompandingConfig {
                law,
                ..Default::default()
            },
        }
    }

    /// Create a codec from a full configuration.
    pub fn from_config(config: CompandingConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &CompandingConfig {
        &self.config
    }

    // ----------------------------------------------------------------
    // µ-law encode / decode  (ITU-T G.711)
    // ----------------------------------------------------------------

    /// Encode a 16-bit linear PCM sample to 8-bit µ-law.
    fn mulaw_encode(&self, pcm: i16) -> u8 {
        let mut pcm_val = pcm as i32;

        // Get the sign bit. Positive = 0, negative = 0x80 (after inversion).
        let sign = if pcm_val < 0 {
            pcm_val = -pcm_val;
            0x80
        } else {
            0x00
        };

        // Clip
        if pcm_val > MULAW_CLIP {
            pcm_val = MULAW_CLIP;
        }

        // Add bias
        pcm_val += MULAW_BIAS;

        // Find the segment (exponent). The segment is determined by
        // finding the position of the highest set bit.
        let mut exponent: u8 = 7;
        let mut mask: i32 = 0x4000;
        while exponent > 0 {
            if pcm_val & mask != 0 {
                break;
            }
            exponent -= 1;
            mask >>= 1;
        }

        // Extract the 4 mantissa bits
        let mantissa = ((pcm_val >> (exponent + 3)) & 0x0F) as u8;

        // Combine sign, exponent, mantissa and complement
        let mulaw_byte = sign | (exponent << 4) | mantissa;
        !mulaw_byte
    }

    /// Decode an 8-bit µ-law sample back to 16-bit linear PCM.
    fn mulaw_decode(&self, mulaw: u8) -> i16 {
        // Complement to undo encoding
        let mulaw_val = !mulaw;

        let sign = mulaw_val & 0x80;
        let exponent = ((mulaw_val >> 4) & 0x07) as i32;
        let mantissa = (mulaw_val & 0x0F) as i32;

        // Reconstruct the magnitude
        let mut mag = ((2 * mantissa + 33) << (exponent + 2)) - MULAW_BIAS;

        if mag < 0 {
            mag = 0;
        }

        if sign != 0 {
            -(mag as i16)
        } else {
            mag as i16
        }
    }

    // ----------------------------------------------------------------
    // A-law encode / decode  (ITU-T G.711)
    // ----------------------------------------------------------------

    /// Encode a 16-bit linear PCM sample to 8-bit A-law.
    ///
    /// Per G.711, A-law uses an inverted sign bit convention: bit 7 = 1 for
    /// positive, 0 for negative. Even bits are also toggled for transmission
    /// (to avoid long runs of zeros on idle channels).
    fn alaw_encode(&self, pcm: i16) -> u8 {
        let mut pcm_val = pcm as i32;

        // G.711 A-law sign convention is inverted: bit 7 = 1 for positive
        let sign = if pcm_val >= 0 {
            0x80u8 // positive: sign bit = 1
        } else {
            pcm_val = -pcm_val;
            0x00u8 // negative: sign bit = 0
        };

        // A-law works on 13-bit magnitude (bits 12..0 of linear)
        // Input is 16-bit, so shift right by 3 to get 13-bit
        pcm_val >>= 3;

        if pcm_val > 0xFFF {
            pcm_val = 0xFFF;
        }

        // Find segment (exponent) and mantissa
        let (exponent, mantissa) = if pcm_val < 0x20 {
            // Segment 0: linear region (no leading 1 bit above position 4)
            (0u8, ((pcm_val >> 1) & 0x0F) as u8)
        } else {
            // Segments 1-7: find the position of the leading 1 bit
            let mut exp = 1u8;
            let mut val = pcm_val >> 5;
            while val > 1 && exp < 7 {
                val >>= 1;
                exp += 1;
            }
            let mant = ((pcm_val >> exp) & 0x0F) as u8;
            (exp, mant)
        };

        // Assemble: sign | exponent | mantissa, then toggle even bits
        let encoded = sign | (exponent << 4) | mantissa;
        encoded ^ 0x55 // toggle even bits (D5 for idle positive zero)
    }

    /// Decode an 8-bit A-law sample back to 16-bit linear PCM.
    fn alaw_decode(&self, alaw: u8) -> i16 {
        // Toggle even bits to undo transmission encoding
        let inv = alaw ^ 0x55;

        // Sign bit: 1 = positive, 0 = negative (inverted convention)
        let is_negative = (inv & 0x80) == 0;
        let exponent = ((inv >> 4) & 0x07) as i32;
        let mantissa = (inv & 0x0F) as i32;

        // Reconstruct the 13-bit magnitude
        let mag13 = if exponent == 0 {
            (mantissa << 1) | 1
        } else {
            ((mantissa << 1) | 1 | 0x20) << (exponent - 1)
        };

        // Scale back to 16-bit (13-bit -> 16-bit: shift left 3)
        let mag16 = mag13 << 3;

        if is_negative {
            -(mag16 as i16)
        } else {
            mag16 as i16
        }
    }

    // ----------------------------------------------------------------
    // Public API
    // ----------------------------------------------------------------

    /// Encode a single 16-bit linear PCM sample to 8-bit compressed form.
    pub fn encode_sample(&self, pcm: i16) -> u8 {
        match self.config.law {
            CompandingLaw::MuLaw => self.mulaw_encode(pcm),
            CompandingLaw::ALaw => self.alaw_encode(pcm),
        }
    }

    /// Decode a single 8-bit compressed sample back to 16-bit linear PCM.
    pub fn decode_sample(&self, compressed: u8) -> i16 {
        match self.config.law {
            CompandingLaw::MuLaw => self.mulaw_decode(compressed),
            CompandingLaw::ALaw => self.alaw_decode(compressed),
        }
    }

    /// Encode a slice of 16-bit PCM samples to 8-bit compressed form.
    pub fn encode(&self, pcm_samples: &[i16]) -> Vec<u8> {
        pcm_samples.iter().map(|&s| self.encode_sample(s)).collect()
    }

    /// Decode a slice of 8-bit compressed samples to 16-bit linear PCM.
    pub fn decode(&self, compressed: &[u8]) -> Vec<i16> {
        compressed.iter().map(|&c| self.decode_sample(c)).collect()
    }

    /// Continuous companding curve: compress a normalized value x in [-1, 1].
    ///
    /// For µ-law: F(x) = sgn(x) * ln(1 + µ|x|) / ln(1 + µ)
    /// For A-law: piecewise linear/logarithmic per ITU-T G.711.
    pub fn compress_f64(&self, x: f64) -> f64 {
        let sign = x.signum();
        let abs_x = x.abs().min(1.0);

        match self.config.law {
            CompandingLaw::MuLaw => {
                let mu = self.config.mu_value;
                sign * (1.0 + mu * abs_x).ln() / (1.0 + mu).ln()
            }
            CompandingLaw::ALaw => {
                let a = self.config.a_value;
                let ln_a = a.ln();
                if abs_x < 1.0 / a {
                    sign * (a * abs_x) / (1.0 + ln_a)
                } else {
                    sign * (1.0 + (a * abs_x).ln()) / (1.0 + ln_a)
                }
            }
        }
    }

    /// Expand a compressed value y in [-1, 1] back to linear.
    ///
    /// Inverse of `compress_f64`.
    pub fn expand_f64(&self, y: f64) -> f64 {
        let sign = y.signum();
        let abs_y = y.abs().min(1.0);

        match self.config.law {
            CompandingLaw::MuLaw => {
                let mu = self.config.mu_value;
                sign * ((1.0 + mu).powf(abs_y) - 1.0) / mu
            }
            CompandingLaw::ALaw => {
                let a = self.config.a_value;
                let ln_a = a.ln();
                let threshold = 1.0 / (1.0 + ln_a);
                if abs_y < threshold {
                    sign * abs_y * (1.0 + ln_a) / a
                } else {
                    sign * ((abs_y * (1.0 + ln_a) - 1.0).exp()) / a
                }
            }
        }
    }

    /// Theoretical signal-to-noise-plus-distortion ratio (SNDR) in dB
    /// for a given number of encoding bits.
    ///
    /// For µ-law: SNDR ~ 6.02*n + 1.76 - 10*log10(3*[ln(1+µ)]^2)  dB
    /// For A-law: SNDR ~ 6.02*n + 1.76 - 10*log10(3*[1+ln(A)]^2)  dB
    ///
    /// These are approximations valid for signals above the minimum
    /// quantization level.
    pub fn sndr_db(&self, bits: usize) -> f64 {
        let n = bits as f64;
        let base = 6.02 * n + 1.76;

        match self.config.law {
            CompandingLaw::MuLaw => {
                let mu = self.config.mu_value;
                let ln_mu1 = (1.0 + mu).ln();
                base - 10.0 * (3.0 * ln_mu1 * ln_mu1).log10()
            }
            CompandingLaw::ALaw => {
                let a = self.config.a_value;
                let ln_a1 = 1.0 + a.ln();
                // A-law SNDR penalty from companding noise
                base - 10.0 * (3.0 * ln_a1 * ln_a1).log10()
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ----------------------------------------------------------------
    // µ-law tests
    // ----------------------------------------------------------------

    #[test]
    fn mulaw_silence_is_0xff() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        assert_eq!(codec.encode_sample(0), 0xFF, "µ-law silence must be 0xFF");
    }

    #[test]
    fn mulaw_roundtrip_zero() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        let decoded = codec.decode_sample(codec.encode_sample(0));
        // µ-law zero doesn't roundtrip to exactly 0 due to bias, but should be very small
        assert!(decoded.abs() < 8, "decoded zero should be near zero, got {decoded}");
    }

    #[test]
    fn mulaw_roundtrip_positive() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        for &val in &[100i16, 500, 1000, 5000, 10000, 30000] {
            let compressed = codec.encode_sample(val);
            let restored = codec.decode_sample(compressed);
            let error = (val as i32 - restored as i32).unsigned_abs();
            let threshold = (val as u32 / 8).max(50);
            assert!(
                error < threshold,
                "µ-law roundtrip error too large for {val}: restored={restored}, error={error}, threshold={threshold}"
            );
        }
    }

    #[test]
    fn mulaw_roundtrip_negative() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        for &val in &[-100i16, -1000, -10000, -32000] {
            let compressed = codec.encode_sample(val);
            let restored = codec.decode_sample(compressed);
            assert!(
                restored < 0,
                "negative input {val} should decode to negative, got {restored}"
            );
            let error = (val as i32 - restored as i32).unsigned_abs();
            let threshold = ((-val) as u32 / 8).max(50);
            assert!(
                error < threshold,
                "µ-law roundtrip error too large for {val}: restored={restored}, error={error}"
            );
        }
    }

    #[test]
    fn mulaw_symmetry() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        for &val in &[100i16, 1000, 10000, 32000] {
            let enc_pos = codec.encode_sample(val);
            let enc_neg = codec.encode_sample(-val);
            assert_eq!(
                enc_pos ^ enc_neg,
                0x80,
                "µ-law encoding of {val} and {} should differ only in sign bit",
                -val
            );
        }
    }

    #[test]
    fn mulaw_monotonic() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        let mut prev_decoded: i16 = 0;
        for val in (0i16..32000).step_by(500) {
            let decoded = codec.decode_sample(codec.encode_sample(val));
            assert!(
                decoded >= prev_decoded,
                "µ-law should be monotonic: val={val}, decoded={decoded}, prev={prev_decoded}"
            );
            prev_decoded = decoded;
        }
    }

    // ----------------------------------------------------------------
    // A-law tests
    // ----------------------------------------------------------------

    #[test]
    fn alaw_silence_is_0xd5() {
        let codec = CompandingCodec::new(CompandingLaw::ALaw);
        assert_eq!(codec.encode_sample(0), 0xD5, "A-law silence must be 0xD5");
    }

    #[test]
    fn alaw_roundtrip_positive() {
        let codec = CompandingCodec::new(CompandingLaw::ALaw);
        for &val in &[100i16, 500, 1000, 5000, 10000] {
            let compressed = codec.encode_sample(val);
            let restored = codec.decode_sample(compressed);
            let error = (val as i32 - restored as i32).unsigned_abs();
            let threshold = (val as u32 / 4).max(80);
            assert!(
                error < threshold,
                "A-law roundtrip error too large for {val}: restored={restored}, error={error}"
            );
        }
    }

    #[test]
    fn alaw_roundtrip_negative() {
        let codec = CompandingCodec::new(CompandingLaw::ALaw);
        for &val in &[-100i16, -1000, -10000] {
            let compressed = codec.encode_sample(val);
            let restored = codec.decode_sample(compressed);
            assert!(
                restored < 0,
                "negative input {val} should decode to negative, got {restored}"
            );
        }
    }

    // ----------------------------------------------------------------
    // Batch encode/decode tests
    // ----------------------------------------------------------------

    #[test]
    fn batch_encode_decode_mulaw() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        let pcm: Vec<i16> = (-5000..5000).step_by(100).collect();
        let encoded = codec.encode(&pcm);
        let decoded = codec.decode(&encoded);
        assert_eq!(pcm.len(), encoded.len());
        assert_eq!(pcm.len(), decoded.len());
        for (&orig, &dec) in pcm.iter().zip(decoded.iter()) {
            if orig.abs() > 200 {
                assert_eq!(
                    orig.signum(),
                    dec.signum(),
                    "sign mismatch: orig={orig}, decoded={dec}"
                );
            }
        }
    }

    #[test]
    fn batch_encode_decode_alaw() {
        let codec = CompandingCodec::new(CompandingLaw::ALaw);
        let pcm: Vec<i16> = vec![0, 1, -1, 100, -100, 1000, -1000, i16::MAX, i16::MIN + 1];
        let encoded = codec.encode(&pcm);
        let decoded = codec.decode(&encoded);
        assert_eq!(pcm.len(), decoded.len());
    }

    // ----------------------------------------------------------------
    // Continuous companding curve tests
    // ----------------------------------------------------------------

    #[test]
    fn compress_expand_roundtrip_mulaw() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        for &x in &[0.0, 0.1, 0.5, 0.99, -0.1, -0.5, -0.99] {
            let y = codec.compress_f64(x);
            let x_restored = codec.expand_f64(y);
            assert!(
                (x - x_restored).abs() < 1e-12,
                "µ-law compress/expand roundtrip failed for {x}: got {x_restored}"
            );
        }
    }

    #[test]
    fn compress_expand_roundtrip_alaw() {
        let codec = CompandingCodec::new(CompandingLaw::ALaw);
        for &x in &[0.0, 0.001, 0.1, 0.5, 0.99, -0.1, -0.5, -0.99] {
            let y = codec.compress_f64(x);
            let x_restored = codec.expand_f64(y);
            assert!(
                (x - x_restored).abs() < 1e-10,
                "A-law compress/expand roundtrip failed for {x}: got {x_restored}"
            );
        }
    }

    #[test]
    fn compress_f64_output_range() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        for i in 0..=100 {
            let x = (i as f64) / 100.0 * 2.0 - 1.0;
            let y = codec.compress_f64(x);
            assert!(
                y.abs() <= 1.0 + 1e-12,
                "compressed value out of range for x={x}: y={y}"
            );
        }
    }

    // ----------------------------------------------------------------
    // SNDR tests
    // ----------------------------------------------------------------

    #[test]
    fn sndr_8bit_reasonable() {
        let mu_codec = CompandingCodec::new(CompandingLaw::MuLaw);
        let a_codec = CompandingCodec::new(CompandingLaw::ALaw);

        let mu_sndr = mu_codec.sndr_db(8);
        let a_sndr = a_codec.sndr_db(8);

        // Both µ-law and A-law 8-bit give approximately 38 dB SNDR
        // (exact depends on formula variant, accept 28-45 range)
        assert!(
            mu_sndr > 28.0 && mu_sndr < 45.0,
            "µ-law 8-bit SNDR out of expected range: {mu_sndr} dB"
        );
        assert!(
            a_sndr > 28.0 && a_sndr < 45.0,
            "A-law 8-bit SNDR out of expected range: {a_sndr} dB"
        );
    }

    #[test]
    fn sndr_increases_with_bits() {
        let codec = CompandingCodec::new(CompandingLaw::MuLaw);
        let sndr_8 = codec.sndr_db(8);
        let sndr_12 = codec.sndr_db(12);
        let sndr_16 = codec.sndr_db(16);
        assert!(
            sndr_12 > sndr_8,
            "SNDR should increase with more bits: 8-bit={sndr_8}, 12-bit={sndr_12}"
        );
        assert!(
            sndr_16 > sndr_12,
            "SNDR should increase with more bits: 12-bit={sndr_12}, 16-bit={sndr_16}"
        );
    }

    // ----------------------------------------------------------------
    // Edge case and config tests
    // ----------------------------------------------------------------

    #[test]
    fn codec_handles_i16_extremes() {
        let mu_codec = CompandingCodec::new(CompandingLaw::MuLaw);
        let a_codec = CompandingCodec::new(CompandingLaw::ALaw);

        let _ = mu_codec.encode_sample(i16::MAX);
        let _ = mu_codec.encode_sample(i16::MIN);
        let _ = a_codec.encode_sample(i16::MAX);
        let _ = a_codec.encode_sample(i16::MIN);

        for byte in 0u8..=255 {
            let _ = mu_codec.decode_sample(byte);
            let _ = a_codec.decode_sample(byte);
        }
    }

    #[test]
    fn custom_config() {
        let config = CompandingConfig {
            law: CompandingLaw::MuLaw,
            mu_value: 100.0,
            a_value: 87.6,
        };
        let codec = CompandingCodec::from_config(config);
        assert_eq!(codec.config().law, CompandingLaw::MuLaw);
        assert!((codec.config().mu_value - 100.0).abs() < f64::EPSILON);

        let y = codec.compress_f64(0.5);
        assert!(y.abs() <= 1.0);
    }
}
