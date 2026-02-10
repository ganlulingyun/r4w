//! UHF RFID backscatter signal detection and demodulation per EPC Gen2 / ISO 18000-6C.
//!
//! This module implements the physical-layer signal processing needed to communicate
//! with passive UHF RFID tags. It covers both directions of the air interface:
//!
//! - **Reader-to-Tag (R→T)**: Pulse-Interval Encoding (PIE) for commands
//! - **Tag-to-Reader (T→R)**: FM0 baseband and Miller subcarrier (M=2,4,8) encoding
//!
//! Additionally it provides carrier removal for backscatter extraction, RSSI
//! measurement, EPC memory parsing with CRC-16 verification, and the adaptive
//! Q-algorithm used for tag anti-collision.
//!
//! # Example
//!
//! ```
//! use r4w_core::rfid_backscatter_receiver::{encode_pie, decode_pie};
//!
//! let bits = vec![true, false, true, true, false];
//! let tari_us = 12.5;
//! let sample_rate = 1_000_000.0;
//!
//! let signal = encode_pie(&bits, tari_us, sample_rate);
//! let decoded = decode_pie(&signal, tari_us, sample_rate);
//! assert_eq!(decoded, bits);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for an RFID reader/receiver.
///
/// These values govern timing, link frequencies, and encoding choices as
/// defined by EPC Gen2 / ISO 18000-6C.
#[derive(Debug, Clone, PartialEq)]
pub struct RfidConfig {
    /// Tari – the reference time interval for PIE encoding, in microseconds.
    /// Typical values: 6.25, 12.5, or 25.0 µs.
    pub tari_us: f64,
    /// Backscatter Link Frequency in kHz (tag reply clock rate).
    /// Common value: 640 kHz for DR=64/3, Tari=6.25 µs.
    pub blf_khz: f64,
    /// Miller subcarrier index: 1 (FM0), 2, 4, or 8.
    pub miller_m: u8,
    /// ADC / simulation sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl Default for RfidConfig {
    fn default() -> Self {
        Self {
            tari_us: 12.5,
            blf_khz: 640.0,
            miller_m: 1,
            sample_rate_hz: 2_000_000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Tag response
// ---------------------------------------------------------------------------

/// Decoded response from a single RFID tag.
#[derive(Debug, Clone, PartialEq)]
pub struct TagResponse {
    /// Electronic Product Code bytes extracted from the reply.
    pub epc: Vec<u8>,
    /// Received signal strength indicator in dBm.
    pub rssi_dbm: f64,
    /// Carrier phase of the backscatter in radians.
    pub phase_rad: f64,
}

// ---------------------------------------------------------------------------
// Main receiver
// ---------------------------------------------------------------------------

/// Stateful RFID backscatter receiver.
///
/// Wraps an [`RfidConfig`] and provides high-level decode helpers that chain
/// carrier removal, FM0/Miller decoding, CRC verification, and EPC extraction.
#[derive(Debug, Clone)]
pub struct RfidReceiver {
    /// Active configuration.
    pub config: RfidConfig,
}

impl RfidReceiver {
    /// Create a new receiver with the given configuration.
    pub fn new(config: RfidConfig) -> Self {
        Self { config }
    }

    /// Attempt to decode a tag reply from raw IQ (real-valued envelope).
    ///
    /// Performs carrier removal, decoding (FM0 or Miller depending on
    /// `config.miller_m`), CRC check, and EPC extraction.  Returns `None`
    /// when the CRC fails or the bit stream is too short.
    pub fn decode_tag_reply(
        &self,
        signal: &[f64],
        carrier_freq: f64,
    ) -> Option<TagResponse> {
        let baseband = extract_backscatter(signal, carrier_freq, self.config.sample_rate_hz);
        let bits = if self.config.miller_m <= 1 {
            decode_fm0(&baseband, self.config.blf_khz, self.config.sample_rate_hz)
        } else {
            decode_miller(
                &baseband,
                self.config.miller_m,
                self.config.blf_khz,
                self.config.sample_rate_hz,
            )
        };
        let epc = parse_epc(&bits)?;
        let rssi_dbm = measure_rssi(signal);
        // Estimate carrier phase from first sample pair (simple approximation).
        let phase_rad = if signal.len() >= 2 {
            signal[1].atan2(signal[0])
        } else {
            0.0
        };
        Some(TagResponse {
            epc,
            rssi_dbm,
            phase_rad,
        })
    }
}

// ---------------------------------------------------------------------------
// PIE encoding / decoding  (Reader → Tag)
// ---------------------------------------------------------------------------

/// Encode a bit sequence using Pulse-Interval Encoding (PIE).
///
/// In PIE a data-0 symbol has a high period of half Tari followed by a low
/// period of half Tari (total = 1 Tari).  A data-1 symbol has a high period
/// of 1.5 Tari followed by a low period of 0.5 Tari (total = 2 Tari).
///
/// The output is a baseband signal sampled at `sample_rate` Hz.
pub fn encode_pie(bits: &[bool], tari_us: f64, sample_rate: f64) -> Vec<f64> {
    let tari_samples = (tari_us * 1e-6 * sample_rate).round() as usize;
    let half = tari_samples / 2;
    // Ensure at least 1 sample per half-interval.
    let half = half.max(1);
    let mut out = Vec::new();
    for &bit in bits {
        if bit {
            // data-1: high for 1.5*Tari, low for 0.5*Tari
            let high_len = half * 3; // 1.5 * tari
            let low_len = half;      // 0.5 * tari
            out.extend(std::iter::repeat(1.0).take(high_len));
            out.extend(std::iter::repeat(-1.0).take(low_len));
        } else {
            // data-0: high for 0.5*Tari, low for 0.5*Tari
            out.extend(std::iter::repeat(1.0).take(half));
            out.extend(std::iter::repeat(-1.0).take(half));
        }
    }
    out
}

/// Decode a PIE-encoded baseband signal back to bits.
///
/// Works by scanning for high→low transitions and measuring each symbol
/// length. Symbols shorter than 1.5 Tari are decoded as `false` (data-0),
/// longer ones as `true` (data-1).
pub fn decode_pie(signal: &[f64], tari_us: f64, sample_rate: f64) -> Vec<bool> {
    let tari_samples = (tari_us * 1e-6 * sample_rate).round() as usize;
    let half = tari_samples.max(2) / 2;
    let half = half.max(1);

    // Threshold at 0
    let binary: Vec<bool> = signal.iter().map(|&s| s >= 0.0).collect();

    let mut bits = Vec::new();
    let mut idx = 0;

    // Skip any leading low
    while idx < binary.len() && !binary[idx] {
        idx += 1;
    }

    while idx < binary.len() {
        // Count the high portion
        let start = idx;
        while idx < binary.len() && binary[idx] {
            idx += 1;
        }
        let high_len = idx - start;
        if high_len == 0 {
            idx += 1;
            continue;
        }

        // Count the low portion
        let low_start = idx;
        while idx < binary.len() && !binary[idx] {
            idx += 1;
        }
        let low_len = idx - low_start;

        // Total symbol length determines bit value.
        // data-0 total = tari_samples (2*half), data-1 total = 2*tari_samples (4*half)
        // Threshold at 1.5*tari
        let symbol_len = high_len + low_len;
        let threshold = half * 3; // 1.5 * tari_samples
        bits.push(symbol_len > threshold);
    }

    bits
}

// ---------------------------------------------------------------------------
// FM0 encoding / decoding  (Tag → Reader, M=1)
// ---------------------------------------------------------------------------

/// Encode bits using FM0 baseband encoding.
///
/// FM0 inverts the level at every bit boundary. A data-0 has an additional
/// mid-bit transition; a data-1 does not.
pub fn encode_fm0(bits: &[bool], blf_khz: f64, sample_rate: f64) -> Vec<f64> {
    let samples_per_bit = ((sample_rate / (blf_khz * 1e3)).round() as usize).max(2);
    let half = samples_per_bit / 2;
    let mut level: f64 = 1.0;
    let mut out = Vec::with_capacity(bits.len() * samples_per_bit);

    for &bit in bits {
        // Transition at start of every bit
        level = -level;

        if !bit {
            // data-0: transition at mid-bit
            out.extend(std::iter::repeat(level).take(half));
            level = -level;
            out.extend(std::iter::repeat(level).take(samples_per_bit - half));
        } else {
            // data-1: no mid-bit transition
            out.extend(std::iter::repeat(level).take(samples_per_bit));
        }
    }
    out
}

/// Decode an FM0-encoded signal back to bits.
///
/// Uses the mid-bit transition (or lack thereof) to distinguish data-0 from
/// data-1.
pub fn decode_fm0(signal: &[f64], blf_khz: f64, sample_rate: f64) -> Vec<bool> {
    let samples_per_bit = ((sample_rate / (blf_khz * 1e3)).round() as usize).max(2);
    let half = samples_per_bit / 2;

    let mut bits = Vec::new();
    let mut idx = 0;

    while idx + samples_per_bit <= signal.len() {
        // Sample the first-half and second-half levels
        let first_quarter = idx + half / 2;
        let third_quarter = idx + half + (samples_per_bit - half) / 2;

        if first_quarter >= signal.len() || third_quarter >= signal.len() {
            break;
        }

        let first_level = signal[first_quarter];
        let second_level = signal[third_quarter];

        // If signs differ there was a mid-bit transition → data-0
        let transition = (first_level >= 0.0) != (second_level >= 0.0);
        bits.push(!transition); // data-0 has transition, data-1 does not

        idx += samples_per_bit;
    }
    bits
}

// ---------------------------------------------------------------------------
// Miller subcarrier encoding / decoding  (Tag → Reader, M=2,4,8)
// ---------------------------------------------------------------------------

/// Encode bits using Miller-M subcarrier modulation.
///
/// Each bit occupies `M` subcarrier cycles at the backscatter link frequency.
/// Phase inversions encode the data: a data-1 has a phase inversion at the
/// bit boundary and no mid-bit inversion; data-0 has a mid-bit inversion and
/// the same phase at the boundary as the previous bit's end.
pub fn encode_miller(bits: &[bool], m: u8, blf_khz: f64, sample_rate: f64) -> Vec<f64> {
    let m = m.max(1) as usize;
    let samples_per_cycle = ((sample_rate / (blf_khz * 1e3)).round() as usize).max(2);
    let samples_per_bit = samples_per_cycle * m;
    let half_cycle = samples_per_cycle / 2;

    let mut out = Vec::with_capacity(bits.len() * samples_per_bit);
    let mut phase: f64 = 1.0;

    for &bit in bits {
        if bit {
            // data-1: phase inversion at bit boundary, no mid-bit change
            phase = -phase;
            for _c in 0..m {
                out.extend(std::iter::repeat(phase).take(half_cycle));
                out.extend(std::iter::repeat(-phase).take(samples_per_cycle - half_cycle));
            }
        } else {
            // data-0: mid-bit phase inversion (after M/2 cycles keep same
            // starting phase, then invert for second half of bit)
            let first_half_cycles = m / 2;
            let second_half_cycles = m - first_half_cycles;

            for _c in 0..first_half_cycles {
                out.extend(std::iter::repeat(phase).take(half_cycle));
                out.extend(std::iter::repeat(-phase).take(samples_per_cycle - half_cycle));
            }
            phase = -phase;
            for _c in 0..second_half_cycles {
                out.extend(std::iter::repeat(phase).take(half_cycle));
                out.extend(std::iter::repeat(-phase).take(samples_per_cycle - half_cycle));
            }
        }
    }
    out
}

/// Decode a Miller-M subcarrier encoded signal back to bits.
///
/// Correlates the first and second halves of each bit period against the
/// subcarrier to detect mid-bit phase inversions.
pub fn decode_miller(signal: &[f64], m: u8, blf_khz: f64, sample_rate: f64) -> Vec<bool> {
    let m_val = m.max(1) as usize;
    let samples_per_cycle = ((sample_rate / (blf_khz * 1e3)).round() as usize).max(2);
    let samples_per_bit = samples_per_cycle * m_val;
    let half_bit = samples_per_bit / 2;

    let mut bits = Vec::new();
    let mut idx = 0;

    // Build one cycle of the subcarrier reference
    let half_cycle = samples_per_cycle / 2;
    let mut ref_cycle = vec![1.0_f64; samples_per_cycle];
    for s in &mut ref_cycle[half_cycle..] {
        *s = -1.0;
    }

    while idx + samples_per_bit <= signal.len() {
        // Correlate first half against subcarrier reference
        let mut corr_first: f64 = 0.0;
        for i in 0..half_bit {
            let ref_val = ref_cycle[i % samples_per_cycle];
            corr_first += signal[idx + i] * ref_val;
        }
        // Correlate second half
        let mut corr_second: f64 = 0.0;
        for i in 0..half_bit {
            let ref_val = ref_cycle[i % samples_per_cycle];
            corr_second += signal[idx + half_bit + i] * ref_val;
        }

        // If the two halves have opposite correlation sign → mid-bit
        // inversion → data-0.  Same sign → data-1.
        let same_sign = (corr_first >= 0.0) == (corr_second >= 0.0);
        bits.push(same_sign); // same sign → data-1

        idx += samples_per_bit;
    }
    bits
}

// ---------------------------------------------------------------------------
// Backscatter extraction
// ---------------------------------------------------------------------------

/// Remove the CW carrier from a received signal to obtain the backscatter
/// baseband envelope.
///
/// Multiplies by a complex exponential at the carrier frequency and low-pass
/// filters (simple moving-average) to strip the carrier, returning the
/// real-valued envelope.
pub fn extract_backscatter(signal: &[f64], carrier_freq: f64, sample_rate: f64) -> Vec<f64> {
    // Down-convert by multiplying with cos(2π f t)
    let mixed: Vec<f64> = signal
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let t = i as f64 / sample_rate;
            s * 2.0 * (2.0 * PI * carrier_freq * t).cos()
        })
        .collect();

    // Simple moving-average low-pass filter
    let filter_len = (sample_rate / carrier_freq).round() as usize;
    let filter_len = filter_len.max(1);

    if mixed.len() < filter_len {
        return mixed;
    }

    let mut out = Vec::with_capacity(mixed.len());
    let mut sum: f64 = mixed[..filter_len].iter().sum();
    // Pad leading samples
    for _ in 0..filter_len / 2 {
        out.push(sum / filter_len as f64);
    }
    out.push(sum / filter_len as f64);
    for i in 1..=(mixed.len() - filter_len) {
        sum += mixed[i + filter_len - 1] - mixed[i - 1];
        out.push(sum / filter_len as f64);
    }
    // Pad trailing to original length
    while out.len() < signal.len() {
        out.push(*out.last().unwrap_or(&0.0));
    }
    out.truncate(signal.len());
    out
}

// ---------------------------------------------------------------------------
// RSSI
// ---------------------------------------------------------------------------

/// Measure the received signal strength as dBm (assuming 50 Ω impedance).
///
/// Computes 10 · log10(P_rms / 1 mW) where P_rms = V_rms² / 50.
pub fn measure_rssi(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return f64::NEG_INFINITY;
    }
    let mean_sq: f64 = signal.iter().map(|s| s * s).sum::<f64>() / signal.len() as f64;
    // P in watts (50 Ω), convert to dBm
    let power_w = mean_sq / 50.0;
    if power_w <= 0.0 {
        return f64::NEG_INFINITY;
    }
    10.0 * (power_w / 1e-3).log10()
}

// ---------------------------------------------------------------------------
// Anti-collision Q-algorithm
// ---------------------------------------------------------------------------

/// Compute the optimal Q value for an EPC Gen2 Query round given an
/// estimated tag population.
///
/// The Q value sets the number of slots to 2^Q. The ideal Q minimises
/// collision probability; a good approximation is Q = ceil(log2(num_tags)).
/// The returned Q is clamped to 0..=15 per the specification.
pub fn q_algorithm_round(num_tags_estimate: usize) -> usize {
    if num_tags_estimate <= 1 {
        return 0;
    }
    let q = (num_tags_estimate as f64).log2().ceil() as usize;
    q.min(15)
}

// ---------------------------------------------------------------------------
// EPC parsing
// ---------------------------------------------------------------------------

/// Parse an EPC from a decoded bit stream.
///
/// Expects a simplified frame layout:
/// - bits\[0..N-16\]: EPC data bits (must be a multiple of 8)
/// - bits\[N-16..N\]: CRC-16 (MSB first)
///
/// Returns the EPC bytes on CRC success, or `None` otherwise.
pub fn parse_epc(bits: &[bool]) -> Option<Vec<u8>> {
    // Need at least 16 bits of CRC + 8 bits of data
    if bits.len() < 24 {
        return None;
    }

    let crc_start = bits.len() - 16;
    let data_bits = &bits[..crc_start];
    let crc_bits = &bits[crc_start..];

    // Convert CRC bits to u16
    let mut received_crc: u16 = 0;
    for &b in crc_bits {
        received_crc = (received_crc << 1) | (b as u16);
    }

    let computed_crc = crc16_epc(data_bits);
    if received_crc != computed_crc {
        return None;
    }

    // Pack data bits into bytes
    let byte_count = data_bits.len() / 8;
    if byte_count == 0 {
        return None;
    }
    let mut epc = Vec::with_capacity(byte_count);
    for i in 0..byte_count {
        let mut byte: u8 = 0;
        for j in 0..8 {
            byte = (byte << 1) | (data_bits[i * 8 + j] as u8);
        }
        epc.push(byte);
    }

    Some(epc)
}

// ---------------------------------------------------------------------------
// CRC-16
// ---------------------------------------------------------------------------

/// Compute the CRC-16 used in EPC Gen2 tag responses.
///
/// Uses the CCITT polynomial 0x1021 with initial value 0xFFFF and final XOR
/// of 0xFFFF, operating on a bit stream.
pub fn crc16_epc(data: &[bool]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &bit in data {
        let xor_flag = ((crc >> 15) & 1) ^ (bit as u16);
        crc <<= 1;
        if xor_flag == 1 {
            crc ^= 0x1021;
        }
    }
    crc ^= 0xFFFF;
    crc
}

// ---------------------------------------------------------------------------
// Link timing analysis helpers
// ---------------------------------------------------------------------------

/// Measure the T1 timing (reader-to-tag turn-around) from a composite
/// signal containing both the reader command and the tag preamble.
///
/// Scans for the end of the last reader high pulse and the start of the
/// first tag modulation. Returns the delay in microseconds, or `None` if
/// the boundaries cannot be found.
pub fn measure_link_timing_t1(signal: &[f64], sample_rate: f64) -> Option<f64> {
    let threshold = 0.3
        * signal
            .iter()
            .map(|s| s.abs())
            .fold(0.0_f64, f64::max);
    if threshold <= 0.0 {
        return None;
    }

    // Find last sample above threshold (end of reader transmission)
    let mut reader_end: Option<usize> = None;
    let mut tag_start: Option<usize> = None;

    // Walk forward: find a gap (below threshold) after initial high region
    let mut in_high = false;
    let mut gap_start = None;
    for (i, &s) in signal.iter().enumerate() {
        if s.abs() > threshold {
            if !in_high {
                if gap_start.is_some() && reader_end.is_some() {
                    tag_start = Some(i);
                    break;
                }
                in_high = true;
            }
            reader_end = Some(i);
        } else {
            if in_high {
                gap_start = Some(i);
                in_high = false;
            }
        }
    }

    match (reader_end, tag_start) {
        (Some(re), Some(ts)) if ts > re => {
            let samples_delay = (ts - re) as f64;
            Some(samples_delay / sample_rate * 1e6)
        }
        _ => None,
    }
}

// =========================================================================
// Tests
// =========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ----- PIE -----------------------------------------------------------

    #[test]
    fn test_pie_encode_decode_roundtrip() {
        let bits = vec![true, false, true, true, false, false, true];
        let signal = encode_pie(&bits, 12.5, 1_000_000.0);
        let decoded = decode_pie(&signal, 12.5, 1_000_000.0);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_pie_encode_single_zero() {
        let signal = encode_pie(&[false], 12.5, 1_000_000.0);
        // data-0 should be shorter than data-1
        let signal1 = encode_pie(&[true], 12.5, 1_000_000.0);
        assert!(signal.len() < signal1.len());
    }

    #[test]
    fn test_pie_encode_empty() {
        let signal = encode_pie(&[], 12.5, 1_000_000.0);
        assert!(signal.is_empty());
    }

    #[test]
    fn test_pie_decode_empty() {
        let decoded = decode_pie(&[], 12.5, 1_000_000.0);
        assert!(decoded.is_empty());
    }

    // ----- FM0 -----------------------------------------------------------

    #[test]
    fn test_fm0_encode_decode_roundtrip() {
        let bits = vec![false, true, false, false, true, true];
        let signal = encode_fm0(&bits, 640.0, 6_400_000.0);
        let decoded = decode_fm0(&signal, 640.0, 6_400_000.0);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_fm0_encode_single_bit() {
        let signal = encode_fm0(&[true], 640.0, 6_400_000.0);
        let spb = (6_400_000.0_f64 / 640_000.0).round() as usize;
        assert_eq!(signal.len(), spb);
    }

    #[test]
    fn test_fm0_all_zeros() {
        let bits = vec![false; 8];
        let signal = encode_fm0(&bits, 640.0, 6_400_000.0);
        let decoded = decode_fm0(&signal, 640.0, 6_400_000.0);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_fm0_all_ones() {
        let bits = vec![true; 8];
        let signal = encode_fm0(&bits, 640.0, 6_400_000.0);
        let decoded = decode_fm0(&signal, 640.0, 6_400_000.0);
        assert_eq!(decoded, bits);
    }

    // ----- Miller --------------------------------------------------------

    #[test]
    fn test_miller_m2_roundtrip() {
        let bits = vec![true, false, true, false, false, true];
        let signal = encode_miller(&bits, 2, 640.0, 6_400_000.0);
        let decoded = decode_miller(&signal, 2, 640.0, 6_400_000.0);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_miller_m4_roundtrip() {
        let bits = vec![false, true, true, false];
        let signal = encode_miller(&bits, 4, 640.0, 6_400_000.0);
        let decoded = decode_miller(&signal, 4, 640.0, 6_400_000.0);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_miller_m8_roundtrip() {
        let bits = vec![true, true, false, false];
        let signal = encode_miller(&bits, 8, 320.0, 6_400_000.0);
        let decoded = decode_miller(&signal, 8, 320.0, 6_400_000.0);
        assert_eq!(decoded, bits);
    }

    #[test]
    fn test_miller_empty() {
        let signal = encode_miller(&[], 2, 640.0, 6_400_000.0);
        assert!(signal.is_empty());
        let decoded = decode_miller(&[], 2, 640.0, 6_400_000.0);
        assert!(decoded.is_empty());
    }

    // ----- Backscatter extraction ----------------------------------------

    #[test]
    fn test_extract_backscatter_removes_carrier() {
        let sample_rate = 10_000_000.0;
        let carrier = 915e6;
        // Create a pure carrier signal
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * carrier * t).cos()
            })
            .collect();
        let baseband = extract_backscatter(&signal, carrier, sample_rate);
        assert_eq!(baseband.len(), signal.len());
        // Baseband should be approximately DC (non-zero mean)
        let mean: f64 = baseband.iter().sum::<f64>() / baseband.len() as f64;
        // The envelope of a cos^2 downconversion has a DC component
        assert!(mean.abs() > 0.01, "baseband mean should be significant");
    }

    #[test]
    fn test_extract_backscatter_short_signal() {
        let baseband = extract_backscatter(&[1.0, -1.0], 1000.0, 10000.0);
        assert_eq!(baseband.len(), 2);
    }

    // ----- RSSI ----------------------------------------------------------

    #[test]
    fn test_measure_rssi_known_signal() {
        // 1V RMS into 50 Ω = 20 mW → ~13 dBm
        let signal = vec![1.0; 100];
        let rssi = measure_rssi(&signal);
        // 10*log10(1.0/50.0 / 1e-3) = 10*log10(20) ≈ 13.01 dBm
        assert!((rssi - 13.01).abs() < 0.1, "rssi={rssi}");
    }

    #[test]
    fn test_measure_rssi_empty() {
        assert!(measure_rssi(&[]).is_infinite());
        assert!(measure_rssi(&[]) < 0.0);
    }

    #[test]
    fn test_measure_rssi_zero_signal() {
        assert!(measure_rssi(&[0.0; 100]).is_infinite());
    }

    // ----- Q-algorithm ---------------------------------------------------

    #[test]
    fn test_q_algorithm_single_tag() {
        assert_eq!(q_algorithm_round(1), 0);
    }

    #[test]
    fn test_q_algorithm_many_tags() {
        // 100 tags → Q = ceil(log2(100)) = 7
        assert_eq!(q_algorithm_round(100), 7);
    }

    #[test]
    fn test_q_algorithm_power_of_two() {
        // 16 tags → Q = 4
        assert_eq!(q_algorithm_round(16), 4);
    }

    #[test]
    fn test_q_algorithm_clamp_max() {
        // Very large → clamped to 15
        assert_eq!(q_algorithm_round(1_000_000), 15);
    }

    #[test]
    fn test_q_algorithm_zero_tags() {
        assert_eq!(q_algorithm_round(0), 0);
    }

    // ----- CRC-16 --------------------------------------------------------

    #[test]
    fn test_crc16_epc_known_vector() {
        // All zeros (8 bits) → known CRC
        let data = vec![false; 8];
        let crc = crc16_epc(&data);
        // Verify it is a valid 16-bit value and deterministic
        let crc2 = crc16_epc(&data);
        assert_eq!(crc, crc2);
        assert_ne!(crc, 0); // CRC of zeros with 0xFFFF init and final XOR is nonzero
    }

    #[test]
    fn test_crc16_epc_different_data() {
        let zeros = vec![false; 16];
        let ones = vec![true; 16];
        assert_ne!(crc16_epc(&zeros), crc16_epc(&ones));
    }

    // ----- EPC parsing ---------------------------------------------------

    #[test]
    fn test_parse_epc_valid() {
        // Create a 96-bit EPC (12 bytes) + 16-bit CRC
        let epc_bytes: Vec<u8> = (0..12).collect(); // 0x00..0x0B
        let mut data_bits: Vec<bool> = Vec::new();
        for &byte in &epc_bytes {
            for j in (0..8).rev() {
                data_bits.push((byte >> j) & 1 == 1);
            }
        }

        let crc = crc16_epc(&data_bits);
        // Append CRC as 16 bits MSB first
        for j in (0..16).rev() {
            data_bits.push((crc >> j) & 1 == 1);
        }

        let parsed = parse_epc(&data_bits);
        assert!(parsed.is_some());
        assert_eq!(parsed.unwrap(), epc_bytes);
    }

    #[test]
    fn test_parse_epc_bad_crc() {
        let epc_bytes: Vec<u8> = vec![0xAB; 4];
        let mut data_bits: Vec<bool> = Vec::new();
        for &byte in &epc_bytes {
            for j in (0..8).rev() {
                data_bits.push((byte >> j) & 1 == 1);
            }
        }
        // Append wrong CRC
        for _ in 0..16 {
            data_bits.push(false);
        }
        assert!(parse_epc(&data_bits).is_none());
    }

    #[test]
    fn test_parse_epc_too_short() {
        assert!(parse_epc(&[false; 10]).is_none());
    }

    // ----- RfidConfig / RfidReceiver -------------------------------------

    #[test]
    fn test_rfid_config_default() {
        let cfg = RfidConfig::default();
        assert_eq!(cfg.tari_us, 12.5);
        assert_eq!(cfg.blf_khz, 640.0);
        assert_eq!(cfg.miller_m, 1);
        assert_eq!(cfg.sample_rate_hz, 2_000_000.0);
    }

    #[test]
    fn test_rfid_receiver_new() {
        let cfg = RfidConfig::default();
        let rx = RfidReceiver::new(cfg.clone());
        assert_eq!(rx.config, cfg);
    }

    // ----- Link timing ---------------------------------------------------

    #[test]
    fn test_link_timing_t1_gap() {
        let sample_rate = 1_000_000.0;
        // Reader pulse, gap, tag pulse
        let mut signal = vec![1.0; 100]; // reader
        signal.extend(vec![0.0; 50]); // gap = 50 µs
        signal.extend(vec![0.8; 100]); // tag

        let t1 = measure_link_timing_t1(&signal, sample_rate);
        assert!(t1.is_some());
        // Should be roughly 50 µs (gap portion)
        let val = t1.unwrap();
        assert!(val > 10.0 && val < 200.0, "t1={val}");
    }

    #[test]
    fn test_link_timing_t1_no_gap() {
        // Flat signal, no gap
        let signal = vec![1.0; 200];
        assert!(measure_link_timing_t1(&signal, 1e6).is_none());
    }
}
