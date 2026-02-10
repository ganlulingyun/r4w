//! # Sigfox Ultra-Narrowband IoT Decoder
//!
//! Decodes and encodes Sigfox ultra-narrowband (UNB) IoT messages using
//! Differential Binary Phase-Shift Keying (DBPSK) at 100 bps. Sigfox operates
//! in sub-GHz ISM bands (868 MHz in Europe, 902 MHz in the US) with an
//! extremely narrow 100 Hz channel bandwidth, enabling long-range, low-power
//! communication for IoT devices.
//!
//! ## Features
//!
//! - **DBPSK modulation/demodulation** at 100 bps
//! - **Preamble detection** with correlation-based synchronization
//! - **Frame encoding/decoding** with header, payload (up to 12 bytes), and CRC-16
//! - **Narrowband filtering** (100 Hz channel bandwidth)
//! - **SNR estimation** from preamble symbols
//! - **Uplink message generation** via `SigfoxEncoder`
//!
//! ## Example
//!
//! ```
//! use r4w_core::sigfox_decoder::{SigfoxConfig, SigfoxDecoder, SigfoxEncoder, SigfoxFrame};
//!
//! // Configure for default Sigfox parameters
//! let config = SigfoxConfig::default();
//!
//! // Encode a message
//! let encoder = SigfoxEncoder::new(config.clone());
//! let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
//! let frame = SigfoxFrame::new(0x001A2B3C, 5, &payload);
//! let bits = encoder.encode_frame(&frame);
//! let waveform = encoder.modulate_dbpsk(&bits);
//!
//! // Decode it back
//! let decoder = SigfoxDecoder::new(config);
//! let demod_bits = decoder.demodulate_dbpsk(&waveform);
//! let preamble_bits = 5 * 8; // 5 preamble bytes
//! let decoded_frame = SigfoxDecoder::decode_frame(&demod_bits[preamble_bits..]);
//! assert!(decoded_frame.is_some());
//! let decoded = decoded_frame.unwrap();
//! assert_eq!(decoded.payload, payload);
//! assert!(SigfoxDecoder::verify_crc(&decoded));
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Sigfox radio configuration parameters.
#[derive(Debug, Clone)]
pub struct SigfoxConfig {
    /// ADC / DAC sample rate in Hz.
    pub sample_rate: f64,
    /// Center frequency in Hz (e.g. 868_000_000.0 for EU).
    pub center_freq: f64,
    /// Channel bandwidth in Hz (Sigfox standard: 100 Hz).
    pub channel_bandwidth: f64,
}

impl Default for SigfoxConfig {
    fn default() -> Self {
        Self {
            sample_rate: 1000.0, // 1 kHz -- convenient default for tests
            center_freq: 868_000_000.0,
            channel_bandwidth: 100.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Frame / Message types
// ---------------------------------------------------------------------------

/// A decoded Sigfox application-layer message.
#[derive(Debug, Clone, PartialEq)]
pub struct SigfoxMessage {
    /// 32-bit device identifier.
    pub device_id: u32,
    /// Uplink sequence number (12 bits).
    pub sequence_number: u16,
    /// Application payload (0-12 bytes).
    pub payload: Vec<u8>,
    /// Received signal strength indicator in dBm.
    pub rssi_dbm: f64,
}

/// Raw Sigfox PHY frame (uplink).
///
/// ```text
/// | Preamble (5 x 0xAA) | Sync 0xB227 | Header (32-bit dev_id, 12-bit seq, 4-bit len) | Payload | CRC-16 |
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct SigfoxFrame {
    /// Preamble pattern bytes (alternating 1/0).
    pub preamble: Vec<u8>,
    /// 16-bit sync word (default `0xB227`).
    pub sync_word: u16,
    /// 32-bit device identifier stored in the header.
    pub device_id: u32,
    /// 12-bit sequence number stored in the header.
    pub sequence_number: u16,
    /// Payload length nibble (0..=12).
    pub payload_length: u8,
    /// Application payload bytes.
    pub payload: Vec<u8>,
    /// CRC-16 over header + payload.
    pub crc: u16,
}

/// Default sync word used by Sigfox uplink frames.
pub const SIGFOX_SYNC_WORD: u16 = 0xB227;

/// Preamble byte (alternating bits).
const PREAMBLE_BYTE: u8 = 0xAA;

/// Number of preamble repetitions.
const PREAMBLE_REPS: usize = 5;

impl SigfoxFrame {
    /// Build a new frame with auto-computed CRC.
    pub fn new(device_id: u32, sequence_number: u16, payload: &[u8]) -> Self {
        assert!(payload.len() <= 12, "Sigfox payload max 12 bytes");
        let preamble = vec![PREAMBLE_BYTE; PREAMBLE_REPS];
        let payload = payload.to_vec();
        let payload_length = payload.len() as u8;
        let mut frame = Self {
            preamble,
            sync_word: SIGFOX_SYNC_WORD,
            device_id,
            sequence_number: sequence_number & 0x0FFF,
            payload_length,
            payload,
            crc: 0,
        };
        frame.crc = frame.compute_crc();
        frame
    }

    /// Serialise header + payload into a byte vector (for CRC computation).
    fn header_payload_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // Device ID -- big-endian
        out.extend_from_slice(&self.device_id.to_be_bytes());
        // Seq (12 bits) | Length (4 bits) -> 2 bytes big-endian
        let seq_len: u16 =
            ((self.sequence_number & 0x0FFF) << 4) | (self.payload_length as u16 & 0x0F);
        out.extend_from_slice(&seq_len.to_be_bytes());
        // Payload
        out.extend_from_slice(&self.payload);
        out
    }

    /// Compute CRC-16/CCITT-FALSE over the header+payload bytes.
    fn compute_crc(&self) -> u16 {
        crc16_ccitt(&self.header_payload_bytes())
    }
}

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
// ---------------------------------------------------------------------------

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// Sigfox DBPSK decoder / receiver.
#[derive(Debug, Clone)]
pub struct SigfoxDecoder {
    config: SigfoxConfig,
    /// Samples per DBPSK symbol (= sample_rate / 100).
    samples_per_symbol: usize,
}

impl SigfoxDecoder {
    /// Create a new decoder from the given configuration.
    pub fn new(config: SigfoxConfig) -> Self {
        let samples_per_symbol = (config.sample_rate / 100.0).round() as usize;
        assert!(samples_per_symbol >= 1, "sample_rate must be >= 100 Hz");
        Self {
            config,
            samples_per_symbol,
        }
    }

    /// Return the underlying configuration.
    pub fn config(&self) -> &SigfoxConfig {
        &self.config
    }

    // -- Preamble detection --------------------------------------------------

    /// Detect the Sigfox preamble in a baseband IQ signal.
    ///
    /// Returns the sample index where the preamble *ends* (i.e. just before the
    /// sync word) if detected, or `None`.
    pub fn detect_preamble(&self, samples: &[(f64, f64)]) -> Option<usize> {
        // Build a reference preamble waveform (alternating +1 / -1 at symbol rate)
        let preamble_bits: Vec<bool> = (0..PREAMBLE_REPS * 8).map(|i| i % 2 == 0).collect();
        let ref_syms = Self::dbpsk_encode_bits(&preamble_bits);
        let ref_len = ref_syms.len() * self.samples_per_symbol;
        if samples.len() < ref_len {
            return None;
        }

        // Build reference waveform (real only, unit amplitude)
        let mut reference = Vec::with_capacity(ref_len);
        for &sym in &ref_syms {
            let val = if sym { 1.0 } else { -1.0 };
            for _ in 0..self.samples_per_symbol {
                reference.push(val);
            }
        }

        // Slide and correlate (normalised)
        let ref_energy: f64 = reference.iter().map(|r| r * r).sum();
        let mut best_idx = 0usize;
        let mut best_corr = 0.0f64;
        let end = samples.len() - ref_len;
        for start in 0..=end {
            let mut corr = 0.0f64;
            let mut sig_energy = 0.0f64;
            for (k, r) in reference.iter().enumerate() {
                let s = samples[start + k].0; // use real part
                corr += s * r;
                sig_energy += s * s;
            }
            let norm = (sig_energy * ref_energy).sqrt();
            let nc = if norm > 1e-12 { corr / norm } else { 0.0 };
            if nc > best_corr {
                best_corr = nc;
                best_idx = start + ref_len;
            }
        }

        // Threshold
        if best_corr > 0.5 {
            Some(best_idx)
        } else {
            None
        }
    }

    // -- DBPSK demodulation --------------------------------------------------

    /// Demodulate a DBPSK waveform into a bit vector.
    ///
    /// Each symbol spans `samples_per_symbol` IQ samples. The first symbol is
    /// the reference phase and is consumed (not output).
    pub fn demodulate_dbpsk(&self, samples: &[(f64, f64)]) -> Vec<bool> {
        let sps = self.samples_per_symbol;
        let n_symbols = samples.len() / sps;
        if n_symbols < 2 {
            return Vec::new();
        }

        // Average each symbol interval to a single complex value
        let mut symbols: Vec<(f64, f64)> = Vec::with_capacity(n_symbols);
        for i in 0..n_symbols {
            let start = i * sps;
            let end = start + sps;
            let mut sum = (0.0, 0.0);
            for s in &samples[start..end] {
                sum.0 += s.0;
                sum.1 += s.1;
            }
            symbols.push((sum.0 / sps as f64, sum.1 / sps as f64));
        }

        // Differential decode: bit = 0 if phase changed ~0, bit = 1 if ~pi
        let mut bits = Vec::with_capacity(n_symbols - 1);
        for i in 1..symbols.len() {
            let (pr, pi) = symbols[i - 1];
            let (cr, ci) = symbols[i];
            // Multiply current by conjugate of previous
            let dr = cr * pr + ci * pi;
            let _di = ci * pr - cr * pi;
            bits.push(dr < 0.0); // phase flip -> bit 1
        }
        bits
    }

    // -- Frame decoding ------------------------------------------------------

    /// Decode a raw bit stream (after DBPSK demodulation) into a `SigfoxFrame`.
    ///
    /// The bit stream should start at the sync-word boundary.
    /// Returns `None` if the stream is too short or the sync word does not match.
    pub fn decode_frame(bits: &[bool]) -> Option<SigfoxFrame> {
        // Sync word: 16 bits
        if bits.len() < 16 {
            return None;
        }
        let sync = bits_to_u16(&bits[0..16]);
        if sync != SIGFOX_SYNC_WORD {
            return None;
        }

        // Header: device_id (32) + seq_len (16) = 48 bits -> total so far 64
        if bits.len() < 64 {
            return None;
        }
        let device_id = bits_to_u32(&bits[16..48]);
        let seq_len = bits_to_u16(&bits[48..64]);
        let sequence_number = seq_len >> 4;
        let payload_length = (seq_len & 0x0F) as u8;
        if payload_length > 12 {
            return None;
        }

        // Payload: payload_length * 8 bits
        let payload_bits_end = 64 + (payload_length as usize) * 8;
        if bits.len() < payload_bits_end + 16 {
            return None;
        }
        let mut payload = Vec::with_capacity(payload_length as usize);
        for i in 0..payload_length as usize {
            let start = 64 + i * 8;
            payload.push(bits_to_u8(&bits[start..start + 8]));
        }

        // CRC-16
        let crc = bits_to_u16(&bits[payload_bits_end..payload_bits_end + 16]);

        Some(SigfoxFrame {
            preamble: vec![PREAMBLE_BYTE; PREAMBLE_REPS],
            sync_word: sync,
            device_id,
            sequence_number,
            payload_length,
            payload,
            crc,
        })
    }

    // -- CRC verification ----------------------------------------------------

    /// Verify the CRC of a decoded frame. Returns `true` when valid.
    pub fn verify_crc(frame: &SigfoxFrame) -> bool {
        frame.crc == frame.compute_crc()
    }

    // -- SNR estimation ------------------------------------------------------

    /// Estimate SNR (dB) from a preamble section of the received signal.
    ///
    /// Assumes the preamble consists of alternating +1/-1 DBPSK symbols.
    /// Computes signal power from the mean symbol magnitude and noise power
    /// from the variance around the expected levels.
    pub fn estimate_snr(&self, preamble_samples: &[(f64, f64)]) -> f64 {
        let sps = self.samples_per_symbol;
        let n_symbols = preamble_samples.len() / sps;
        if n_symbols < 2 {
            return 0.0;
        }

        let mut magnitudes = Vec::with_capacity(n_symbols);
        for i in 0..n_symbols {
            let start = i * sps;
            let end = start + sps;
            let mut sum = (0.0, 0.0);
            for s in &preamble_samples[start..end] {
                sum.0 += s.0;
                sum.1 += s.1;
            }
            let avg_re = sum.0 / sps as f64;
            let avg_im = sum.1 / sps as f64;
            magnitudes.push((avg_re * avg_re + avg_im * avg_im).sqrt());
        }

        let mean_mag: f64 = magnitudes.iter().sum::<f64>() / magnitudes.len() as f64;
        let signal_power = mean_mag * mean_mag;

        let noise_power: f64 = magnitudes
            .iter()
            .map(|m| {
                let diff = m - mean_mag;
                diff * diff
            })
            .sum::<f64>()
            / magnitudes.len() as f64;

        if noise_power < 1e-30 {
            return 60.0; // essentially no noise
        }
        10.0 * (signal_power / noise_power).log10()
    }

    // -- Narrowband filtering ------------------------------------------------

    /// Apply a simple narrowband FIR low-pass filter centred at DC with a
    /// bandwidth of `channel_bandwidth` Hz.
    ///
    /// This uses a windowed-sinc design with a Hamming window.
    pub fn narrowband_filter(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let cutoff = self.config.channel_bandwidth / 2.0; // one-sided
        let fc = cutoff / self.config.sample_rate; // normalised
        let n = 63usize; // filter order (odd for type-I FIR)
        let m = n / 2;

        // Design windowed-sinc coefficients
        let mut h: Vec<f64> = Vec::with_capacity(n);
        for i in 0..n {
            let k = i as f64 - m as f64;
            let sinc = if k.abs() < 1e-12 {
                2.0 * fc
            } else {
                (2.0 * PI * fc * k).sin() / (PI * k)
            };
            let hamming = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1) as f64).cos();
            h.push(sinc * hamming);
        }

        // Normalise
        let sum: f64 = h.iter().sum();
        for v in &mut h {
            *v /= sum;
        }

        // Apply (direct convolution)
        let mut out = Vec::with_capacity(samples.len());
        for i in 0..samples.len() {
            let mut re = 0.0;
            let mut im = 0.0;
            for (j, &coeff) in h.iter().enumerate() {
                let idx = i as isize - j as isize + m as isize;
                if idx >= 0 && (idx as usize) < samples.len() {
                    let s = samples[idx as usize];
                    re += s.0 * coeff;
                    im += s.1 * coeff;
                }
            }
            out.push((re, im));
        }
        out
    }

    // -- helpers (static) ---------------------------------------------------

    /// DBPSK-encode a bit sequence: output a vector of differential phase
    /// symbols (bool, where `true` = phase-inverted).
    ///
    /// The output has length `bits.len() + 1` (first element is the reference).
    fn dbpsk_encode_bits(bits: &[bool]) -> Vec<bool> {
        let mut syms = Vec::with_capacity(bits.len() + 1);
        syms.push(false); // reference symbol (phase = 0)
        let mut phase = false;
        for &b in bits {
            if b {
                phase = !phase;
            }
            syms.push(phase);
        }
        syms
    }
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

/// Sigfox DBPSK encoder / transmitter.
#[derive(Debug, Clone)]
pub struct SigfoxEncoder {
    config: SigfoxConfig,
    /// Samples per symbol.
    samples_per_symbol: usize,
}

impl SigfoxEncoder {
    /// Create a new encoder from the given configuration.
    pub fn new(config: SigfoxConfig) -> Self {
        let samples_per_symbol = (config.sample_rate / 100.0).round() as usize;
        assert!(samples_per_symbol >= 1, "sample_rate must be >= 100 Hz");
        Self {
            config,
            samples_per_symbol,
        }
    }

    /// Return the underlying configuration.
    pub fn config(&self) -> &SigfoxConfig {
        &self.config
    }

    /// Serialise a `SigfoxFrame` into a bit stream (preamble + sync + header +
    /// payload + CRC).
    pub fn encode_frame(&self, frame: &SigfoxFrame) -> Vec<bool> {
        let mut bits = Vec::new();

        // Preamble bytes -> bits
        for &b in &frame.preamble {
            push_byte(&mut bits, b);
        }

        // Sync word (16 bits, MSB first)
        push_u16(&mut bits, frame.sync_word);

        // Header: device_id (32 bits) + seq_len (16 bits)
        push_u32(&mut bits, frame.device_id);
        let seq_len: u16 =
            ((frame.sequence_number & 0x0FFF) << 4) | (frame.payload_length as u16 & 0x0F);
        push_u16(&mut bits, seq_len);

        // Payload
        for &b in &frame.payload {
            push_byte(&mut bits, b);
        }

        // CRC-16
        push_u16(&mut bits, frame.crc);

        bits
    }

    /// Generate a DBPSK baseband IQ waveform from a bit vector at 100 bps.
    ///
    /// Each symbol is held for `samples_per_symbol` samples. The carrier is at
    /// DC (zero IF).
    pub fn modulate_dbpsk(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let syms = SigfoxDecoder::dbpsk_encode_bits(bits);
        let sps = self.samples_per_symbol;
        let mut waveform = Vec::with_capacity(syms.len() * sps);
        for &s in &syms {
            let val = if s { -1.0 } else { 1.0 };
            for _ in 0..sps {
                waveform.push((val, 0.0));
            }
        }
        waveform
    }
}

// ---------------------------------------------------------------------------
// Bit-packing helpers
// ---------------------------------------------------------------------------

fn push_byte(bits: &mut Vec<bool>, byte: u8) {
    for i in (0..8).rev() {
        bits.push((byte >> i) & 1 == 1);
    }
}

fn push_u16(bits: &mut Vec<bool>, val: u16) {
    for i in (0..16).rev() {
        bits.push((val >> i) & 1 == 1);
    }
}

fn push_u32(bits: &mut Vec<bool>, val: u32) {
    for i in (0..32).rev() {
        bits.push((val >> i) & 1 == 1);
    }
}

fn bits_to_u8(bits: &[bool]) -> u8 {
    assert!(bits.len() == 8);
    let mut val = 0u8;
    for &b in bits {
        val = (val << 1) | (b as u8);
    }
    val
}

fn bits_to_u16(bits: &[bool]) -> u16 {
    assert!(bits.len() == 16);
    let mut val = 0u16;
    for &b in bits {
        val = (val << 1) | (b as u16);
    }
    val
}

fn bits_to_u32(bits: &[bool]) -> u32 {
    assert!(bits.len() == 32);
    let mut val = 0u32;
    for &b in bits {
        val = (val << 1) | (b as u32);
    }
    val
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> SigfoxConfig {
        SigfoxConfig::default()
    }

    // 1. Round-trip encode -> modulate -> demodulate -> decode
    #[test]
    fn test_roundtrip_encode_decode() {
        let config = default_config();
        let encoder = SigfoxEncoder::new(config.clone());
        let decoder = SigfoxDecoder::new(config);

        let payload = vec![0x01, 0x02, 0x03, 0x04];
        let frame = SigfoxFrame::new(0xAABBCCDD, 42, &payload);
        let bits = encoder.encode_frame(&frame);
        let waveform = encoder.modulate_dbpsk(&bits);

        let preamble_bits = PREAMBLE_REPS * 8;
        let demod_bits = decoder.demodulate_dbpsk(&waveform);

        // Decode from sync-word offset
        let decoded = SigfoxDecoder::decode_frame(&demod_bits[preamble_bits..]);
        assert!(decoded.is_some(), "frame decode failed");
        let decoded = decoded.unwrap();
        assert_eq!(decoded.device_id, 0xAABBCCDD);
        assert_eq!(decoded.sequence_number, 42);
        assert_eq!(decoded.payload, payload);
        assert!(SigfoxDecoder::verify_crc(&decoded));
    }

    // 2. CRC verification on valid frame
    #[test]
    fn test_crc_valid() {
        let frame = SigfoxFrame::new(0x12345678, 100, &[0xFF; 12]);
        assert!(SigfoxDecoder::verify_crc(&frame));
    }

    // 3. CRC verification fails on corrupted frame
    #[test]
    fn test_crc_invalid() {
        let mut frame = SigfoxFrame::new(0x12345678, 100, &[0xFF; 12]);
        frame.crc ^= 0x0001; // flip one bit
        assert!(!SigfoxDecoder::verify_crc(&frame));
    }

    // 4. Empty payload round-trip
    #[test]
    fn test_empty_payload() {
        let config = default_config();
        let encoder = SigfoxEncoder::new(config.clone());
        let decoder = SigfoxDecoder::new(config);

        let frame = SigfoxFrame::new(0x00000001, 0, &[]);
        let bits = encoder.encode_frame(&frame);
        let waveform = encoder.modulate_dbpsk(&bits);
        let demod = decoder.demodulate_dbpsk(&waveform);

        let preamble_bits = PREAMBLE_REPS * 8;
        let decoded = SigfoxDecoder::decode_frame(&demod[preamble_bits..]).unwrap();
        assert_eq!(decoded.payload.len(), 0);
        assert!(SigfoxDecoder::verify_crc(&decoded));
    }

    // 5. Maximum payload (12 bytes)
    #[test]
    fn test_max_payload() {
        let payload: Vec<u8> = (0..12).collect();
        let frame = SigfoxFrame::new(0xDEADBEEF, 4095, &payload);
        assert_eq!(frame.payload_length, 12);
        assert_eq!(frame.sequence_number, 4095);
        assert!(SigfoxDecoder::verify_crc(&frame));
    }

    // 6. Sequence number masking to 12 bits
    #[test]
    fn test_sequence_number_masking() {
        let frame = SigfoxFrame::new(0, 0xFFFF, &[]);
        assert_eq!(frame.sequence_number, 0x0FFF);
    }

    // 7. Preamble detection on clean signal
    #[test]
    fn test_preamble_detection_clean() {
        let config = default_config();
        let encoder = SigfoxEncoder::new(config.clone());
        let decoder = SigfoxDecoder::new(config);

        let frame = SigfoxFrame::new(0x11223344, 1, &[0xAA]);
        let bits = encoder.encode_frame(&frame);
        let waveform = encoder.modulate_dbpsk(&bits);

        let result = decoder.detect_preamble(&waveform);
        assert!(result.is_some(), "preamble should be detected");
    }

    // 8. Preamble detection fails on noise
    #[test]
    fn test_preamble_detection_noise() {
        let config = default_config();
        let decoder = SigfoxDecoder::new(config);

        // Deterministic pseudo-noise that does not correlate with preamble
        let noise: Vec<(f64, f64)> = (0..500)
            .map(|i| {
                let v = ((i as f64) * 1.7).sin() * 0.1;
                (v, 0.0)
            })
            .collect();
        let result = decoder.detect_preamble(&noise);
        assert!(result.is_none(), "should not detect preamble in noise");
    }

    // 9. SNR estimation on clean signal gives high SNR
    #[test]
    fn test_snr_estimation_clean() {
        let config = default_config();
        let decoder = SigfoxDecoder::new(config.clone());
        let sps = decoder.samples_per_symbol;

        // Build a clean alternating preamble
        let n_symbols = 40;
        let mut samples = Vec::new();
        for i in 0..n_symbols {
            let val = if i % 2 == 0 { 1.0 } else { -1.0 };
            for _ in 0..sps {
                samples.push((val, 0.0));
            }
        }

        let snr = decoder.estimate_snr(&samples);
        assert!(snr > 20.0, "clean signal should have high SNR, got {snr}");
    }

    // 10. DBPSK modulation produces correct number of samples
    #[test]
    fn test_modulation_length() {
        let config = default_config();
        let encoder = SigfoxEncoder::new(config.clone());
        let sps = (config.sample_rate / 100.0).round() as usize;

        let bits = vec![true, false, true, true, false];
        let waveform = encoder.modulate_dbpsk(&bits);
        // bits.len() + 1 symbols (reference symbol), each sps samples
        assert_eq!(waveform.len(), (bits.len() + 1) * sps);
    }

    // 11. Narrowband filter preserves DC signal
    #[test]
    fn test_narrowband_filter_dc() {
        let config = default_config();
        let decoder = SigfoxDecoder::new(config);

        // DC signal at amplitude 1.0
        let samples: Vec<(f64, f64)> = vec![(1.0, 0.0); 200];
        let filtered = decoder.narrowband_filter(&samples);

        // Middle of the output should be close to 1.0
        let mid = filtered.len() / 2;
        assert!(
            (filtered[mid].0 - 1.0).abs() < 0.05,
            "DC should pass through, got {}",
            filtered[mid].0
        );
    }

    // 12. Decode frame returns None for too-short bitstream
    #[test]
    fn test_decode_frame_too_short() {
        let bits = vec![false; 10];
        assert!(SigfoxDecoder::decode_frame(&bits).is_none());
    }

    // 13. Decode frame returns None for wrong sync word
    #[test]
    fn test_decode_frame_wrong_sync() {
        // 16 bits that do not match SIGFOX_SYNC_WORD, plus padding
        let bits = vec![false; 128];
        assert!(SigfoxDecoder::decode_frame(&bits).is_none());
    }

    // 14. CRC-16 known test vector
    #[test]
    fn test_crc16_known_vector() {
        // "123456789" -> CRC-16/CCITT-FALSE = 0x29B1
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0x29B1, "CRC mismatch: got {crc:#06X}");
    }

    // 15. Bit packing round-trip
    #[test]
    fn test_bit_packing_roundtrip() {
        let val: u32 = 0xCAFEBABE;
        let mut bits = Vec::new();
        push_u32(&mut bits, val);
        assert_eq!(bits_to_u32(&bits), val);

        let val16: u16 = 0xB227;
        let mut bits16 = Vec::new();
        push_u16(&mut bits16, val16);
        assert_eq!(bits_to_u16(&bits16), val16);
    }
}
