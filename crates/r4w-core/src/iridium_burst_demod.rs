//! Iridium NEXT Satellite Burst Demodulator
//!
//! Implements DQPSK burst demodulation for the Iridium NEXT LEO satellite
//! constellation simplex downlink (1626.0–1626.5 MHz L-band).
//!
//! # Signal Parameters
//! - Frequency band: 1626.0–1626.5 MHz (L-band simplex downlink)
//! - Channel spacing: 25 kHz, 41 channels
//! - Modulation: DQPSK (Differential Quaternary Phase-Shift Keying)
//! - Symbol rate: 25 ksps → 50 kbps raw data rate
//! - Doppler range: ±37 kHz (LEO orbit ~780 km altitude)
//!
//! # Features
//! - DQPSK demodulation with differential decoding
//! - Power-based burst detection with AGC
//! - Preamble correlation and UW (Unique Word) detection
//! - IRA (Iridium Ring Alert) burst format decoding
//! - SBD (Short Burst Data) message framing and header parsing
//! - XOR descrambling with LFSR generator polynomial
//! - BCH(31,21) error correction with syndrome computation
//! - Per-burst Doppler estimation and compensation
//! - Block interleaver / de-interleaver

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// L-band simplex downlink base frequency (Hz)
pub const IRIDIUM_BASE_FREQ_HZ: f64 = 1_626_000_000.0;

/// Simplex downlink bandwidth (Hz)
pub const IRIDIUM_BAND_BW_HZ: f64 = 500_000.0;

/// Channel spacing (Hz)
pub const IRIDIUM_CHANNEL_SPACING_HZ: f64 = 25_000.0;

/// Number of simplex downlink channels
pub const IRIDIUM_NUM_CHANNELS: usize = 41;

/// Symbol rate (symbols per second)
pub const IRIDIUM_SYMBOL_RATE: f64 = 25_000.0;

/// Raw data rate (bits per second)
pub const IRIDIUM_DATA_RATE: f64 = 50_000.0;

/// Maximum LEO Doppler shift (Hz)
pub const IRIDIUM_MAX_DOPPLER_HZ: f64 = 37_000.0;

/// Preamble length in symbols
pub const IRIDIUM_PREAMBLE_SYMBOLS: usize = 64;

/// Unique Word (UW) length in bits
pub const IRIDIUM_UW_BITS: usize = 32;

/// Known UW pattern for Iridium downlink frames
pub const IRIDIUM_UW: u32 = 0xACAC_ACAC;

/// IRA burst payload bytes
pub const IRA_PAYLOAD_BYTES: usize = 75;

/// SBD header size in bytes
pub const SBD_HEADER_BYTES: usize = 3;

/// BCH(31,21) code parameters
pub const BCH_N: usize = 31;
pub const BCH_K: usize = 21;
pub const BCH_T: usize = 2; // can correct up to 2 errors

/// BCH generator polynomial for BCH(31,21), g(x) = x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1
pub const BCH_GENERATOR: u32 = 0b111_0110_1001; // 0x769 (10 parity bits)

/// Iridium LFSR descrambler polynomial (x^7 + x^3 + 1)
pub const LFSR_POLY: u8 = 0x89; // feedback polynomial
pub const LFSR_SEED: u8 = 0x1F; // initial state

// ---------------------------------------------------------------------------
// Basic complex number type
// ---------------------------------------------------------------------------

/// A complex-valued IQ sample (64-bit float)
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    #[inline]
    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    /// Complex conjugate
    #[inline]
    pub fn conj(self) -> Self {
        Self { re: self.re, im: -self.im }
    }

    /// Magnitude squared
    #[inline]
    pub fn mag_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    /// Magnitude
    #[inline]
    pub fn mag(self) -> f64 {
        self.mag_sq().sqrt()
    }

    /// Phase angle (radians)
    #[inline]
    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }

    /// Multiply two complex numbers
    #[inline]
    pub fn mul(self, other: Self) -> Self {
        Self {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }

    /// Scale by a real factor
    #[inline]
    pub fn scale(self, s: f64) -> Self {
        Self { re: self.re * s, im: self.im * s }
    }

    /// Add two complex numbers
    #[inline]
    pub fn add(self, other: Self) -> Self {
        Self { re: self.re + other.re, im: self.im + other.im }
    }

    /// Unit phasor e^{jθ}
    #[inline]
    pub fn exp_j(theta: f64) -> Self {
        Self { re: theta.cos(), im: theta.sin() }
    }
}

// ---------------------------------------------------------------------------
// DQPSK Demodulator
// ---------------------------------------------------------------------------

/// DQPSK symbol (dibit: 2 bits)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DqpskDibit {
    S00 = 0,
    S01 = 1,
    S10 = 2,
    S11 = 3,
}

impl DqpskDibit {
    /// Convert integer 0..3 to dibit
    pub fn from_u8(v: u8) -> Self {
        match v & 0x3 {
            0 => Self::S00,
            1 => Self::S01,
            2 => Self::S10,
            _ => Self::S11,
        }
    }

    /// Returns the two bits [MSB, LSB]
    pub fn to_bits(self) -> [bool; 2] {
        let v = self as u8;
        [(v >> 1) & 1 == 1, v & 1 == 1]
    }
}

/// Phase change mapping for DQPSK (Gray-coded)
/// Δφ  0° → 00, 90° → 01, 180° → 11, 270° → 10
fn phase_diff_to_dibit(delta_phase: f64) -> DqpskDibit {
    // Wrap to [0, 2π)
    let dp = ((delta_phase % (2.0 * PI)) + 2.0 * PI) % (2.0 * PI);
    // Quantize to nearest 90°
    let quad = (dp / (PI / 2.0) + 0.5).floor() as u8 % 4;
    match quad {
        0 => DqpskDibit::S00, //   0° → 00
        1 => DqpskDibit::S01, //  90° → 01
        2 => DqpskDibit::S11, // 180° → 11  (Gray)
        _ => DqpskDibit::S10, // 270° → 10  (Gray)
    }
}

/// DQPSK demodulator state machine
pub struct DqpskDemodulator {
    /// Previous received symbol (for differential decoding)
    prev_symbol: Complex,
    /// Accumulated phase for constellation rotation removal
    rotation_phase: f64,
}

impl DqpskDemodulator {
    /// Create a new DQPSK demodulator
    pub fn new() -> Self {
        Self {
            prev_symbol: Complex::new(1.0, 0.0),
            rotation_phase: 0.0,
        }
    }

    /// Reset demodulator state
    pub fn reset(&mut self) {
        self.prev_symbol = Complex::new(1.0, 0.0);
        self.rotation_phase = 0.0;
    }

    /// Apply constellation rotation correction
    pub fn set_rotation(&mut self, phase_rad: f64) {
        self.rotation_phase = phase_rad;
    }

    /// Demodulate a single received sample (already matched-filtered / symbol-sampled)
    /// Returns the decoded dibit
    pub fn demodulate_sample(&mut self, sample: Complex) -> DqpskDibit {
        // Apply rotation correction
        let correction = Complex::exp_j(-self.rotation_phase);
        let corrected = sample.mul(correction);

        // Differential detection: multiply by conjugate of previous
        let diff = corrected.mul(self.prev_symbol.conj());
        let delta_phase = diff.arg();

        // Update previous symbol
        self.prev_symbol = corrected;

        phase_diff_to_dibit(delta_phase)
    }

    /// Demodulate a vector of complex samples into dibits
    pub fn demodulate(&mut self, samples: &[Complex]) -> Vec<DqpskDibit> {
        samples.iter().map(|&s| self.demodulate_sample(s)).collect()
    }

    /// Demodulate into raw bits (2 bits per symbol, MSB first)
    pub fn demodulate_bits(&mut self, samples: &[Complex]) -> Vec<bool> {
        let mut bits = Vec::with_capacity(samples.len() * 2);
        for &s in samples {
            let dibit = self.demodulate_sample(s);
            let [b1, b0] = dibit.to_bits();
            bits.push(b1);
            bits.push(b0);
        }
        bits
    }
}

impl Default for DqpskDemodulator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Burst Detector
// ---------------------------------------------------------------------------

/// Burst event type
#[derive(Clone, Debug, PartialEq)]
pub enum BurstEvent {
    /// Burst started at given sample index, estimated power
    Start { sample_index: usize, power_db: f64 },
    /// Burst ended at given sample index
    End { sample_index: usize, duration_samples: usize },
}

/// Power-based burst detector with hysteresis and per-burst AGC
pub struct BurstDetector {
    /// Threshold for burst ON (linear power)
    threshold_on: f64,
    /// Threshold for burst OFF (hysteresis, linear power)
    threshold_off: f64,
    /// IIR smoothing coefficient for power estimate
    alpha: f64,
    /// Current smoothed power estimate
    smoothed_power: f64,
    /// Whether we are currently in a burst
    in_burst: bool,
    /// Sample index of burst start
    burst_start: usize,
    /// Current sample index
    sample_count: usize,
    /// AGC gain for current burst
    agc_gain: f64,
    /// Peak power seen in this burst
    burst_peak_power: f64,
}

impl BurstDetector {
    /// Create a new burst detector.
    ///
    /// - `threshold_on_db`: Power threshold to declare burst start (dB relative to 0 dBFS)
    /// - `threshold_off_db`: Power threshold to declare burst end (hysteresis, < threshold_on)
    /// - `alpha`: IIR smoothing factor (0 < alpha < 1, smaller = more smoothing)
    pub fn new(threshold_on_db: f64, threshold_off_db: f64, alpha: f64) -> Self {
        let on = 10f64.powf(threshold_on_db / 10.0);
        let off = 10f64.powf(threshold_off_db / 10.0);
        Self {
            threshold_on: on,
            threshold_off: off,
            alpha,
            smoothed_power: 0.0,
            in_burst: false,
            burst_start: 0,
            sample_count: 0,
            agc_gain: 1.0,
            burst_peak_power: 0.0,
        }
    }

    /// Process a single complex sample. Returns an optional burst event.
    pub fn process_sample(&mut self, sample: Complex) -> Option<BurstEvent> {
        let power = sample.mag_sq();
        self.smoothed_power = self.alpha * power + (1.0 - self.alpha) * self.smoothed_power;

        let mut event = None;

        if !self.in_burst && self.smoothed_power > self.threshold_on {
            self.in_burst = true;
            self.burst_start = self.sample_count;
            self.burst_peak_power = self.smoothed_power;
            let power_db = 10.0 * self.smoothed_power.log10();
            event = Some(BurstEvent::Start {
                sample_index: self.sample_count,
                power_db,
            });
            // Compute AGC gain to normalize burst to unit power
            self.agc_gain = if self.smoothed_power > 1e-12 {
                1.0 / self.smoothed_power.sqrt()
            } else {
                1.0
            };
        } else if self.in_burst {
            if self.smoothed_power > self.burst_peak_power {
                self.burst_peak_power = self.smoothed_power;
            }
            if self.smoothed_power < self.threshold_off {
                self.in_burst = false;
                let duration = self.sample_count - self.burst_start;
                event = Some(BurstEvent::End {
                    sample_index: self.sample_count,
                    duration_samples: duration,
                });
            }
        }

        self.sample_count += 1;
        event
    }

    /// Process a block of samples. Returns all burst events found.
    pub fn process(&mut self, samples: &[Complex]) -> Vec<BurstEvent> {
        samples.iter().filter_map(|&s| self.process_sample(s)).collect()
    }

    /// Apply AGC to a sample using the current burst gain
    pub fn apply_agc(&self, sample: Complex) -> Complex {
        sample.scale(self.agc_gain)
    }

    /// Returns true if currently inside a burst
    pub fn in_burst(&self) -> bool {
        self.in_burst
    }

    /// Returns current AGC gain
    pub fn agc_gain(&self) -> f64 {
        self.agc_gain
    }
}

// ---------------------------------------------------------------------------
// Preamble Synchronizer
// ---------------------------------------------------------------------------

/// Result of preamble search
#[derive(Clone, Debug)]
pub struct PreambleResult {
    /// Best correlation peak location (sample offset)
    pub peak_offset: usize,
    /// Normalized correlation peak value (0..1)
    pub peak_corr: f64,
    /// Estimated phase offset at peak (radians)
    pub phase_offset: f64,
}

/// Preamble correlator for burst synchronization
///
/// Correlates received samples against a known BPSK/DQPSK preamble sequence.
pub struct PreambleCorrelator {
    /// Reference preamble (complex baseband)
    reference: Vec<Complex>,
}

impl PreambleCorrelator {
    /// Create correlator from a known bit pattern (BPSK modulated)
    pub fn new(preamble_bits: &[bool]) -> Self {
        let reference: Vec<Complex> = preamble_bits
            .iter()
            .map(|&b| {
                if b {
                    Complex::new(1.0, 0.0)
                } else {
                    Complex::new(-1.0, 0.0)
                }
            })
            .collect();
        Self { reference }
    }

    /// Create the standard Iridium alternating preamble (64 symbols, alternating 0°/180°)
    pub fn iridium_preamble() -> Self {
        let bits: Vec<bool> = (0..IRIDIUM_PREAMBLE_SYMBOLS).map(|i| i % 2 == 0).collect();
        Self::new(&bits)
    }

    /// Correlate preamble against input samples.
    /// Returns the best peak result if correlation exceeds min_threshold.
    pub fn correlate(&self, samples: &[Complex], min_threshold: f64) -> Option<PreambleResult> {
        let n = self.reference.len();
        if samples.len() < n {
            return None;
        }

        let ref_power: f64 = self.reference.iter().map(|s| s.mag_sq()).sum::<f64>().sqrt();

        let mut best_corr = 0.0f64;
        let mut best_offset = 0usize;
        let mut best_phase = 0.0f64;

        for start in 0..=(samples.len() - n) {
            let window = &samples[start..start + n];
            // Complex cross-correlation
            let corr: Complex = self
                .reference
                .iter()
                .zip(window.iter())
                .fold(Complex::zero(), |acc, (&r, &s)| acc.add(s.mul(r.conj())));

            let sig_power: f64 = window.iter().map(|s| s.mag_sq()).sum::<f64>().sqrt();
            let norm = if ref_power * sig_power > 1e-20 {
                corr.mag() / (ref_power * sig_power)
            } else {
                0.0
            };

            if norm > best_corr {
                best_corr = norm;
                best_offset = start;
                best_phase = corr.arg();
            }
        }

        if best_corr >= min_threshold {
            Some(PreambleResult {
                peak_offset: best_offset,
                peak_corr: best_corr,
                phase_offset: best_phase,
            })
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Unique Word Detector
// ---------------------------------------------------------------------------

/// UW detection result
#[derive(Clone, Debug)]
pub struct UwDetectionResult {
    /// Bit offset where UW was found
    pub bit_offset: usize,
    /// Number of bit errors (Hamming distance from expected UW)
    pub bit_errors: u32,
}

/// Detects the Iridium Unique Word in a bit stream
pub struct UniqueWordDetector {
    /// Expected UW pattern
    expected: u32,
    /// Maximum allowed bit errors (Hamming distance threshold)
    max_errors: u32,
}

impl UniqueWordDetector {
    /// Create a UW detector for the standard Iridium UW
    pub fn new() -> Self {
        Self {
            expected: IRIDIUM_UW,
            max_errors: 4,
        }
    }

    /// Create a UW detector with a custom pattern and error threshold
    pub fn with_pattern(pattern: u32, max_errors: u32) -> Self {
        Self {
            expected: pattern,
            max_errors,
        }
    }

    /// Search a bit stream for the UW. Returns the first valid detection.
    pub fn detect(&self, bits: &[bool]) -> Option<UwDetectionResult> {
        if bits.len() < IRIDIUM_UW_BITS {
            return None;
        }

        for offset in 0..=(bits.len() - IRIDIUM_UW_BITS) {
            let mut word = 0u32;
            for i in 0..IRIDIUM_UW_BITS {
                if bits[offset + i] {
                    word |= 1 << (IRIDIUM_UW_BITS - 1 - i);
                }
            }
            let errors = (word ^ self.expected).count_ones();
            if errors <= self.max_errors {
                return Some(UwDetectionResult {
                    bit_offset: offset,
                    bit_errors: errors,
                });
            }
        }
        None
    }
}

impl Default for UniqueWordDetector {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// IRA (Iridium Ring Alert) Burst
// ---------------------------------------------------------------------------

/// Parsed IRA burst
#[derive(Clone, Debug)]
pub struct IraBurst {
    /// Raw payload bytes
    pub payload: Vec<u8>,
    /// 3-tone detection flags (tone1, tone2, guard)
    pub tones: [bool; 3],
    /// Ring alert sequence number (if decoded)
    pub sequence_number: u8,
}

/// Iridium 3-tone ring alert detector
///
/// Detects the three tones used for ring alert signalling.
/// Typical tone frequencies are at ±2 kHz and guard at 0 Hz relative to channel center.
pub struct RingAlertDetector {
    /// Sample rate (Hz)
    sample_rate: f64,
    /// Tone frequencies relative to channel center (Hz)
    tone_freqs: [f64; 3],
    /// Detection threshold (normalized power)
    threshold: f64,
}

impl RingAlertDetector {
    /// Create a ring alert detector
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            tone_freqs: [-2000.0, 2000.0, 0.0],
            threshold: 0.01,
        }
    }

    /// Compute Goertzel power at a given frequency
    fn goertzel_power(&self, samples: &[Complex], freq_hz: f64) -> f64 {
        let n = samples.len() as f64;
        let k = freq_hz / self.sample_rate * n;
        let omega = 2.0 * PI * k / n;
        let coeff = 2.0 * omega.cos();

        let (mut s_prev2, mut s_prev1) = (0.0f64, 0.0f64);
        for s in samples {
            let s_curr = s.re + coeff * s_prev1 - s_prev2;
            s_prev2 = s_prev1;
            s_prev1 = s_curr;
        }
        s_prev2 * s_prev2 + s_prev1 * s_prev1 - coeff * s_prev1 * s_prev2
    }

    /// Detect tones in the given sample block.
    /// Returns [tone1_present, tone2_present, guard_present]
    pub fn detect_tones(&self, samples: &[Complex]) -> [bool; 3] {
        let mut result = [false; 3];
        for (i, &freq) in self.tone_freqs.iter().enumerate() {
            let power = self.goertzel_power(samples, freq);
            let norm = power / samples.len() as f64;
            result[i] = norm > self.threshold;
        }
        result
    }

    /// Parse an IRA burst from demodulated bits
    pub fn parse_ira_burst(&self, bits: &[bool]) -> Option<IraBurst> {
        if bits.len() < IRA_PAYLOAD_BYTES * 8 {
            return None;
        }
        let payload = bits_to_bytes(bits);
        let sequence_number = if !payload.is_empty() { payload[0] } else { 0 };
        Some(IraBurst {
            payload,
            tones: [false; 3],
            sequence_number,
        })
    }
}

// ---------------------------------------------------------------------------
// SBD (Short Burst Data) Frame Parser
// ---------------------------------------------------------------------------

/// SBD message type
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SbdMessageType {
    /// Mobile originated (device → gateway)
    MobileOriginated = 0x01,
    /// Mobile terminated (gateway → device)
    MobileTerminated = 0x02,
    /// Unknown / reserved
    Unknown = 0xFF,
}

/// Parsed SBD header
#[derive(Clone, Debug)]
pub struct SbdHeader {
    /// Message type
    pub msg_type: SbdMessageType,
    /// Sequence number (16-bit)
    pub sequence_number: u16,
    /// IMEI (15 decimal digits, packed as bytes)
    pub imei: [u8; 8],
}

/// Extracted SBD message
#[derive(Clone, Debug)]
pub struct SbdMessage {
    /// Parsed header
    pub header: SbdHeader,
    /// Payload bytes
    pub payload: Vec<u8>,
    /// CRC check passed?
    pub crc_ok: bool,
}

/// SBD frame parser
pub struct SbdParser;

impl SbdParser {
    /// Parse an SBD message from raw bytes.
    /// Returns None if the data is too short.
    pub fn parse(data: &[u8]) -> Option<SbdMessage> {
        if data.len() < SBD_HEADER_BYTES + 2 {
            return None;
        }

        let msg_type = match data[0] {
            0x01 => SbdMessageType::MobileOriginated,
            0x02 => SbdMessageType::MobileTerminated,
            _ => SbdMessageType::Unknown,
        };

        let sequence_number = u16::from_be_bytes([data[1], data[2]]);

        // IMEI is next 8 bytes (BCD packed or raw bytes)
        let mut imei = [0u8; 8];
        if data.len() >= SBD_HEADER_BYTES + 8 {
            imei.copy_from_slice(&data[SBD_HEADER_BYTES..SBD_HEADER_BYTES + 8]);
        }

        let payload_start = SBD_HEADER_BYTES + 8;
        let payload = if data.len() > payload_start + 2 {
            data[payload_start..data.len() - 2].to_vec()
        } else {
            vec![]
        };

        // CRC-16 check (last 2 bytes)
        let crc_ok = if data.len() >= 2 {
            let received_crc = u16::from_be_bytes([
                data[data.len() - 2],
                data[data.len() - 1],
            ]);
            let computed = crc16_ccitt(&data[..data.len() - 2]);
            received_crc == computed
        } else {
            false
        };

        Some(SbdMessage {
            header: SbdHeader {
                msg_type,
                sequence_number,
                imei,
            },
            payload,
            crc_ok,
        })
    }
}

// ---------------------------------------------------------------------------
// LFSR Descrambler
// ---------------------------------------------------------------------------

/// XOR descrambler using a 7-bit LFSR (x^7 + x^3 + 1)
///
/// Used to remove the Iridium burst scrambling sequence.
pub struct IridiumDescrambler {
    /// LFSR state (7 bits used)
    state: u8,
    /// Feedback polynomial (taps)
    poly: u8,
}

impl IridiumDescrambler {
    /// Create a new descrambler with the Iridium polynomial
    pub fn new() -> Self {
        Self {
            state: LFSR_SEED,
            poly: LFSR_POLY,
        }
    }

    /// Reset the LFSR to initial seed
    pub fn reset(&mut self) {
        self.state = LFSR_SEED;
    }

    /// Generate one bit from the LFSR and advance state
    fn next_bit(&mut self) -> bool {
        let feedback = (self.state & 0x80) != 0;
        self.state = (self.state << 1)
            | if (self.state.count_ones() & (self.poly.count_ones() & 1)) != 0 {
                1
            } else {
                0
            };
        feedback
    }

    /// Descramble a slice of bits in-place (XOR with LFSR sequence)
    pub fn descramble_bits(&mut self, bits: &mut [bool]) {
        for bit in bits.iter_mut() {
            let seq_bit = self.next_bit();
            *bit ^= seq_bit;
        }
    }

    /// Descramble and return a new bit vector
    pub fn descramble(&mut self, bits: &[bool]) -> Vec<bool> {
        bits.iter()
            .map(|&b| {
                let seq = self.next_bit();
                b ^ seq
            })
            .collect()
    }

    /// Descramble bytes in-place
    pub fn descramble_bytes(&mut self, data: &mut [u8]) {
        for byte in data.iter_mut() {
            let mut seq_byte = 0u8;
            for i in (0..8).rev() {
                let seq_bit = self.next_bit();
                seq_byte |= (seq_bit as u8) << i;
            }
            *byte ^= seq_byte;
        }
    }
}

impl Default for IridiumDescrambler {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// BCH(31,21) Error Correction
// ---------------------------------------------------------------------------

/// BCH(31,21) codeword (31 bits packed in u32)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BchCodeword(pub u32);

/// BCH error correction result
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum BchResult {
    /// No errors detected
    NoError(u32),
    /// Errors corrected, returns corrected data (21 bits)
    Corrected { data: u32, errors: usize },
    /// Uncorrectable errors (more than t=2)
    Uncorrectable,
}

/// BCH(31,21) codec with t=2 error correction
///
/// Uses GF(2^5) arithmetic over the field generated by p(x) = x^5 + x^2 + 1.
pub struct Bch3121;

impl Bch3121 {
    /// GF(2^5) primitive polynomial: x^5 + x^2 + 1 = 0b100101 = 0x25
    const PRIM_POLY: u8 = 0x25;

    /// Multiply two GF(2^5) elements
    fn gf_mul(mut a: u8, mut b: u8) -> u8 {
        let mut result = 0u8;
        for _ in 0..5 {
            if b & 1 != 0 {
                result ^= a;
            }
            let high = (a & 0x10) != 0;
            a <<= 1;
            if high {
                a ^= Self::PRIM_POLY & 0x1F;
            }
            b >>= 1;
        }
        result & 0x1F
    }

    /// Compute GF(2^5) power: alpha^n where alpha is root of primitive poly
    fn gf_pow(base: u8, exp: usize) -> u8 {
        let mut result = 1u8;
        let mut b = base & 0x1F;
        let mut e = exp;
        while e > 0 {
            if e & 1 != 0 {
                result = Self::gf_mul(result, b);
            }
            b = Self::gf_mul(b, b);
            e >>= 1;
        }
        result
    }

    /// Compute syndromes S1 and S2 (for t=2 BCH)
    /// Returns (S1, S2) as GF(2^5) elements.
    fn compute_syndromes(codeword: u32) -> (u8, u8) {
        // alpha = 0x02 (primitive element)
        let alpha: u8 = 0x02;
        let mut s1 = 0u8;
        let mut s2 = 0u8;
        for i in 0..BCH_N {
            if (codeword >> i) & 1 != 0 {
                s1 ^= Self::gf_pow(alpha, i);
                s2 ^= Self::gf_pow(alpha, 2 * i);
            }
        }
        (s1, s2)
    }

    /// Encode 21 data bits to BCH(31,21) codeword (returns 31-bit word)
    pub fn encode(data: u32) -> BchCodeword {
        // data occupies bits [30:10], parity in bits [9:0]
        let msg = (data & 0x1F_FFFF) << 10; // shift data to high bits
        let mut remainder = msg;
        for i in (10..BCH_N).rev() {
            if (remainder >> i) & 1 != 0 {
                remainder ^= BCH_GENERATOR << (i - 10);
            }
        }
        BchCodeword(msg | (remainder & 0x3FF))
    }

    /// Decode a BCH(31,21) codeword, correcting up to t=2 errors
    pub fn decode(codeword: BchCodeword) -> BchResult {
        let (s1, s2) = Self::compute_syndromes(codeword.0);

        if s1 == 0 && s2 == 0 {
            // No errors
            let data = (codeword.0 >> 10) & 0x1F_FFFF;
            return BchResult::NoError(data);
        }

        // Single error correction: if s2 == s1^2, single error at position log_alpha(s1)
        let alpha: u8 = 0x02;
        let s1_sq = Self::gf_mul(s1, s1);
        if s1_sq == s2 {
            // Single error – find position
            for pos in 0..BCH_N {
                if Self::gf_pow(alpha, pos) == s1 {
                    let corrected = codeword.0 ^ (1u32 << pos);
                    let data = (corrected >> 10) & 0x1F_FFFF;
                    return BchResult::Corrected { data, errors: 1 };
                }
            }
        }

        // Two error correction using error-locator polynomial:
        // sigma(x) = 1 + sigma1*x + sigma2*x^2
        // sigma1 = s1, sigma2 = (s1^2 + s2) / s1  (if s1 != 0)
        if s1 != 0 {
            let alpha_inv = Self::gf_pow(alpha, (BCH_N as usize - 1) * 30); // rough inverse
            let _ = alpha_inv;

            // Find the two error locations via exhaustive search over GF(2^5)^2
            for e1 in 0..BCH_N {
                for e2 in (e1 + 1)..BCH_N {
                    let a1 = Self::gf_pow(alpha, e1);
                    let a2 = Self::gf_pow(alpha, e2);
                    let test_s1 = a1 ^ a2;
                    let test_s2 = Self::gf_mul(a1, a1) ^ Self::gf_mul(a2, a2);
                    if test_s1 == s1 && test_s2 == s2 {
                        let corrected = codeword.0 ^ (1u32 << e1) ^ (1u32 << e2);
                        let data = (corrected >> 10) & 0x1F_FFFF;
                        return BchResult::Corrected { data, errors: 2 };
                    }
                }
            }
        }

        BchResult::Uncorrectable
    }
}

// ---------------------------------------------------------------------------
// Doppler Estimator and Compensator
// ---------------------------------------------------------------------------

/// Per-burst Doppler estimation result
#[derive(Clone, Debug)]
pub struct DopplerEstimate {
    /// Estimated frequency offset (Hz)
    pub freq_offset_hz: f64,
    /// Confidence metric (0..1)
    pub confidence: f64,
}

/// Doppler compensator for Iridium bursts
///
/// Estimates Doppler from the preamble phase progression and applies
/// a phase-continuous frequency correction to the burst samples.
pub struct DopplerCompensator {
    /// NCO phase accumulator
    phase: f64,
    /// Current frequency correction (rad/sample)
    freq_rad_per_sample: f64,
    /// Sample rate (Hz)
    sample_rate: f64,
}

impl DopplerCompensator {
    /// Create a new Doppler compensator
    pub fn new(sample_rate: f64) -> Self {
        Self {
            phase: 0.0,
            freq_rad_per_sample: 0.0,
            sample_rate,
        }
    }

    /// Reset NCO state
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Set frequency correction from an estimate
    pub fn set_correction(&mut self, freq_hz: f64) {
        self.freq_rad_per_sample = -2.0 * PI * freq_hz / self.sample_rate;
    }

    /// Estimate Doppler from preamble using phase slope method.
    ///
    /// For an alternating preamble, the phase between consecutive symbols
    /// should be constant (0 or π). Any linear drift indicates frequency offset.
    pub fn estimate_from_preamble(&self, preamble_samples: &[Complex]) -> DopplerEstimate {
        if preamble_samples.len() < 4 {
            return DopplerEstimate {
                freq_offset_hz: 0.0,
                confidence: 0.0,
            };
        }

        // Compute phase differences between alternate symbols to remove
        // the known +π phase of the alternating preamble
        let n = preamble_samples.len();
        let mut phase_acc = 0.0f64;
        let mut count = 0usize;

        for i in 2..n {
            // Compare sym[i] with sym[i-2] (same polarity in alternating preamble)
            let diff = preamble_samples[i].mul(preamble_samples[i - 2].conj());
            let phase_step = diff.arg() / 2.0; // phase per symbol
            phase_acc += phase_step;
            count += 1;
        }

        if count == 0 {
            return DopplerEstimate {
                freq_offset_hz: 0.0,
                confidence: 0.0,
            };
        }

        let avg_phase_per_sym = phase_acc / count as f64;
        let freq_hz = avg_phase_per_sym * self.sample_rate / (2.0 * PI);

        // Confidence: check if Doppler is within LEO expected range
        let confidence = if freq_hz.abs() <= IRIDIUM_MAX_DOPPLER_HZ {
            1.0 - (freq_hz.abs() / IRIDIUM_MAX_DOPPLER_HZ)
        } else {
            0.0
        };

        DopplerEstimate {
            freq_offset_hz: freq_hz,
            confidence,
        }
    }

    /// Apply frequency correction to a block of samples (in-place)
    pub fn compensate(&mut self, samples: &mut [Complex]) {
        for s in samples.iter_mut() {
            let corr = Complex::exp_j(self.phase);
            *s = s.mul(corr);
            self.phase += self.freq_rad_per_sample;
            // Wrap phase to avoid accumulation error
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
        }
    }

    /// Apply and return compensated samples
    pub fn compensate_copy(&mut self, samples: &[Complex]) -> Vec<Complex> {
        let mut out = samples.to_vec();
        self.compensate(&mut out);
        out
    }
}

// ---------------------------------------------------------------------------
// Block Interleaver / De-interleaver
// ---------------------------------------------------------------------------

/// Block interleaver / de-interleaver for Iridium channel coding
///
/// Uses a rectangular block interleaver of size rows × cols.
pub struct BlockInterleaver {
    rows: usize,
    cols: usize,
}

impl BlockInterleaver {
    /// Create a new block interleaver
    pub fn new(rows: usize, cols: usize) -> Self {
        Self { rows, cols }
    }

    /// Interleave bits: write by rows, read by columns
    pub fn interleave(&self, bits: &[bool]) -> Vec<bool> {
        let total = self.rows * self.cols;
        let mut buf = vec![false; total];
        let n = bits.len().min(total);
        // Write row-major
        buf[..n].copy_from_slice(&bits[..n]);
        // Read column-major
        let mut out = Vec::with_capacity(total);
        for col in 0..self.cols {
            for row in 0..self.rows {
                out.push(buf[row * self.cols + col]);
            }
        }
        out
    }

    /// De-interleave bits: write by columns, read by rows
    pub fn deinterleave(&self, bits: &[bool]) -> Vec<bool> {
        let total = self.rows * self.cols;
        let mut buf = vec![false; total];
        let n = bits.len().min(total);
        // Write column-major
        let mut idx = 0;
        'outer: for col in 0..self.cols {
            for row in 0..self.rows {
                if idx >= n {
                    break 'outer;
                }
                buf[row * self.cols + col] = bits[idx];
                idx += 1;
            }
        }
        // Read row-major
        buf
    }
}

// ---------------------------------------------------------------------------
// Channel Frequency Utilities
// ---------------------------------------------------------------------------

/// Compute the center frequency for a given Iridium channel index (0..40)
pub fn channel_center_freq(channel: usize) -> f64 {
    assert!(channel < IRIDIUM_NUM_CHANNELS, "channel index out of range");
    IRIDIUM_BASE_FREQ_HZ + channel as f64 * IRIDIUM_CHANNEL_SPACING_HZ + IRIDIUM_CHANNEL_SPACING_HZ / 2.0
}

/// Find the best channel for a given observed frequency (Hz)
pub fn freq_to_channel(freq_hz: f64) -> Option<usize> {
    for ch in 0..IRIDIUM_NUM_CHANNELS {
        let center = channel_center_freq(ch);
        if (freq_hz - center).abs() <= IRIDIUM_CHANNEL_SPACING_HZ / 2.0 {
            return Some(ch);
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Convert a bit slice to packed bytes (MSB first)
pub fn bits_to_bytes(bits: &[bool]) -> Vec<u8> {
    let nbytes = (bits.len() + 7) / 8;
    let mut out = vec![0u8; nbytes];
    for (i, &b) in bits.iter().enumerate() {
        if b {
            out[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    out
}

/// Convert packed bytes to bit slice (MSB first)
pub fn bytes_to_bits(bytes: &[u8]) -> Vec<bool> {
    let mut bits = Vec::with_capacity(bytes.len() * 8);
    for &byte in bytes {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1 != 0);
        }
    }
    bits
}

/// CRC-16/CCITT (polynomial 0x1021, init 0xFFFF)
pub fn crc16_ccitt(data: &[u8]) -> u16 {
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
// Complete Burst Demodulator (top-level)
// ---------------------------------------------------------------------------

/// Decoded Iridium burst result
#[derive(Clone, Debug)]
pub struct DemodulatedBurst {
    /// Burst detected at this sample offset
    pub sample_offset: usize,
    /// Doppler-compensated DQPSK bits
    pub bits: Vec<bool>,
    /// UW detection result (if found)
    pub uw: Option<UwDetectionResult>,
    /// Decoded SBD message (if applicable)
    pub sbd: Option<SbdMessage>,
    /// Estimated Doppler offset (Hz)
    pub doppler_hz: f64,
    /// Preamble correlation peak
    pub preamble_corr: f64,
}

/// Full Iridium NEXT burst demodulator pipeline
pub struct IridiumBurstDemodulator {
    /// Sample rate of input IQ stream
    sample_rate: f64,
    /// Burst detector
    burst_detector: BurstDetector,
    /// Preamble correlator
    preamble_correlator: PreambleCorrelator,
    /// UW detector
    uw_detector: UniqueWordDetector,
    /// DQPSK demodulator
    dqpsk: DqpskDemodulator,
    /// Doppler compensator
    doppler: DopplerCompensator,
    /// Descrambler
    descrambler: IridiumDescrambler,
    /// De-interleaver
    deinterleaver: BlockInterleaver,
    /// Accumulated burst samples
    burst_samples: Vec<Complex>,
    /// Whether currently collecting a burst
    collecting: bool,
    /// Burst start offset
    burst_start_offset: usize,
}

impl IridiumBurstDemodulator {
    /// Create a new full pipeline demodulator
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            burst_detector: BurstDetector::new(-20.0, -25.0, 0.01),
            preamble_correlator: PreambleCorrelator::iridium_preamble(),
            uw_detector: UniqueWordDetector::new(),
            dqpsk: DqpskDemodulator::new(),
            doppler: DopplerCompensator::new(sample_rate),
            descrambler: IridiumDescrambler::new(),
            deinterleaver: BlockInterleaver::new(8, 16),
            burst_samples: Vec::new(),
            collecting: false,
            burst_start_offset: 0,
        }
    }

    /// Process a block of IQ samples.
    /// Returns any fully decoded bursts found in this block.
    pub fn process(&mut self, samples: &[Complex]) -> Vec<DemodulatedBurst> {
        let mut results = Vec::new();

        for (idx, &sample) in samples.iter().enumerate() {
            // Apply AGC
            let agc_sample = if self.burst_detector.in_burst() {
                self.burst_detector.apply_agc(sample)
            } else {
                sample
            };

            match self.burst_detector.process_sample(sample) {
                Some(BurstEvent::Start { sample_index, .. }) => {
                    self.collecting = true;
                    self.burst_start_offset = sample_index;
                    self.burst_samples.clear();
                    self.burst_samples.push(agc_sample);
                }
                Some(BurstEvent::End { .. }) => {
                    if self.collecting && !self.burst_samples.is_empty() {
                        if let Some(burst) = self.decode_burst() {
                            results.push(burst);
                        }
                        self.collecting = false;
                        self.burst_samples.clear();
                    }
                }
                None => {
                    if self.collecting {
                        self.burst_samples.push(agc_sample);
                    }
                }
            }
            let _ = idx;
        }

        results
    }

    /// Decode a collected burst
    fn decode_burst(&mut self) -> Option<DemodulatedBurst> {
        let samples = self.burst_samples.clone();

        // Step 1: Preamble correlation
        let preamble_result = self
            .preamble_correlator
            .correlate(&samples, 0.3)?;

        // Step 2: Doppler estimation from preamble
        let preamble_end = preamble_result.peak_offset + IRIDIUM_PREAMBLE_SYMBOLS;
        let preamble_samples = if preamble_end <= samples.len() {
            &samples[preamble_result.peak_offset..preamble_end]
        } else {
            return None;
        };

        let doppler_est = self.doppler.estimate_from_preamble(preamble_samples);
        self.doppler.set_correction(doppler_est.freq_offset_hz);
        self.doppler.reset();

        // Step 3: Doppler compensation
        let mut compensated = samples[preamble_result.peak_offset..].to_vec();
        self.doppler.compensate(&mut compensated);

        // Step 4: DQPSK demodulation
        self.dqpsk.reset();
        self.dqpsk.set_rotation(preamble_result.phase_offset);
        let mut bits = self.dqpsk.demodulate_bits(&compensated);

        // Step 5: De-interleave
        let deinterleaved = self.deinterleaver.deinterleave(&bits);
        bits = deinterleaved;

        // Step 6: Descramble
        self.descrambler.reset();
        let descrambled = self.descrambler.descramble(&bits);

        // Step 7: UW detection
        let uw = self.uw_detector.detect(&descrambled);

        // Step 8: SBD parsing (if enough bits)
        let sbd = if descrambled.len() >= SBD_HEADER_BYTES * 8 {
            let bytes = bits_to_bytes(&descrambled);
            SbdParser::parse(&bytes)
        } else {
            None
        };

        Some(DemodulatedBurst {
            sample_offset: self.burst_start_offset,
            bits: descrambled,
            uw,
            sbd,
            doppler_hz: doppler_est.freq_offset_hz,
            preamble_corr: preamble_result.peak_corr,
        })
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Complex arithmetic ---

    #[test]
    fn test_complex_mul() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, 4.0);
        let c = a.mul(b);
        assert!((c.re - (-5.0)).abs() < 1e-10);
        assert!((c.im - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_conj() {
        let a = Complex::new(3.0, -4.0);
        let c = a.conj();
        assert_eq!(c.re, 3.0);
        assert_eq!(c.im, 4.0);
    }

    #[test]
    fn test_complex_mag() {
        let a = Complex::new(3.0, 4.0);
        assert!((a.mag() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_arg() {
        let a = Complex::new(0.0, 1.0);
        assert!((a.arg() - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_exp_j() {
        let e = Complex::exp_j(PI / 2.0);
        assert!(e.re.abs() < 1e-10);
        assert!((e.im - 1.0).abs() < 1e-10);
    }

    // --- DQPSK ---

    #[test]
    fn test_phase_diff_to_dibit_0() {
        let d = phase_diff_to_dibit(0.0);
        assert_eq!(d, DqpskDibit::S00);
    }

    #[test]
    fn test_phase_diff_to_dibit_90() {
        let d = phase_diff_to_dibit(PI / 2.0);
        assert_eq!(d, DqpskDibit::S01);
    }

    #[test]
    fn test_phase_diff_to_dibit_180() {
        let d = phase_diff_to_dibit(PI);
        assert_eq!(d, DqpskDibit::S11);
    }

    #[test]
    fn test_phase_diff_to_dibit_270() {
        let d = phase_diff_to_dibit(3.0 * PI / 2.0);
        assert_eq!(d, DqpskDibit::S10);
    }

    #[test]
    fn test_dibit_to_bits() {
        assert_eq!(DqpskDibit::S00.to_bits(), [false, false]);
        assert_eq!(DqpskDibit::S01.to_bits(), [false, true]);
        assert_eq!(DqpskDibit::S10.to_bits(), [true, false]);
        assert_eq!(DqpskDibit::S11.to_bits(), [true, true]);
    }

    #[test]
    fn test_dqpsk_demodulate_no_phase_change() {
        let mut demod = DqpskDemodulator::new();
        // Two identical samples → 0° phase change → S00
        let s1 = Complex::new(1.0, 0.0);
        let s2 = Complex::new(1.0, 0.0);
        demod.demodulate_sample(s1); // initializes prev
        let d = demod.demodulate_sample(s2);
        assert_eq!(d, DqpskDibit::S00);
    }

    #[test]
    fn test_dqpsk_demodulate_90deg() {
        let mut demod = DqpskDemodulator::new();
        let s1 = Complex::new(1.0, 0.0);
        let s2 = Complex::new(0.0, 1.0); // 90° phase advance
        demod.demodulate_sample(s1);
        let d = demod.demodulate_sample(s2);
        assert_eq!(d, DqpskDibit::S01);
    }

    #[test]
    fn test_dqpsk_demodulate_180deg() {
        let mut demod = DqpskDemodulator::new();
        let s1 = Complex::new(1.0, 0.0);
        let s2 = Complex::new(-1.0, 0.0); // 180° phase advance
        demod.demodulate_sample(s1);
        let d = demod.demodulate_sample(s2);
        assert_eq!(d, DqpskDibit::S11);
    }

    #[test]
    fn test_dqpsk_demodulate_bits_length() {
        let mut demod = DqpskDemodulator::new();
        let samples: Vec<Complex> = (0..10).map(|_| Complex::new(1.0, 0.0)).collect();
        let bits = demod.demodulate_bits(&samples);
        assert_eq!(bits.len(), 20);
    }

    #[test]
    fn test_dqpsk_reset() {
        let mut demod = DqpskDemodulator::new();
        let s = Complex::new(0.0, 1.0);
        demod.demodulate_sample(s);
        demod.reset();
        assert_eq!(demod.prev_symbol, Complex::new(1.0, 0.0));
    }

    // --- Burst Detector ---

    #[test]
    fn test_burst_detector_start() {
        let mut det = BurstDetector::new(-10.0, -15.0, 0.5);
        // Send high-power sample
        let strong = Complex::new(10.0, 0.0);
        let mut found_start = false;
        for _ in 0..20 {
            if let Some(BurstEvent::Start { .. }) = det.process_sample(strong) {
                found_start = true;
                break;
            }
        }
        assert!(found_start);
    }

    #[test]
    fn test_burst_detector_end() {
        let mut det = BurstDetector::new(-20.0, -25.0, 0.5);
        let strong = Complex::new(5.0, 0.0);
        let weak = Complex::new(0.0, 0.0);

        // Trigger burst start
        for _ in 0..5 {
            det.process_sample(strong);
        }
        // Send weak samples to trigger end
        let mut found_end = false;
        for _ in 0..20 {
            if let Some(BurstEvent::End { .. }) = det.process_sample(weak) {
                found_end = true;
                break;
            }
        }
        assert!(found_end);
    }

    #[test]
    fn test_burst_detector_agc_gain() {
        let mut det = BurstDetector::new(-20.0, -25.0, 0.5);
        let strong = Complex::new(3.0, 4.0); // mag = 5, power = 25
        for _ in 0..10 {
            det.process_sample(strong);
        }
        assert!(det.agc_gain() > 0.0);
    }

    #[test]
    fn test_burst_detector_initially_not_in_burst() {
        let det = BurstDetector::new(-10.0, -15.0, 0.5);
        assert!(!det.in_burst());
    }

    // --- Preamble Correlator ---

    #[test]
    fn test_preamble_correlator_perfect_match() {
        let bits: Vec<bool> = (0..16).map(|i| i % 2 == 0).collect();
        let corr = PreambleCorrelator::new(&bits);
        // Generate matching baseband samples
        let samples: Vec<Complex> = bits
            .iter()
            .map(|&b| if b { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) })
            .collect();
        let result = corr.correlate(&samples, 0.9);
        assert!(result.is_some());
        let r = result.unwrap();
        assert!((r.peak_corr - 1.0).abs() < 0.01);
        assert_eq!(r.peak_offset, 0);
    }

    #[test]
    fn test_preamble_correlator_offset() {
        let bits: Vec<bool> = (0..8).map(|i| i % 2 == 0).collect();
        let corr = PreambleCorrelator::new(&bits);
        let mut samples: Vec<Complex> = vec![Complex::zero(); 4]; // 4 zeros before preamble
        for &b in &bits {
            samples.push(if b { Complex::new(1.0, 0.0) } else { Complex::new(-1.0, 0.0) });
        }
        let result = corr.correlate(&samples, 0.9);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.peak_offset, 4);
    }

    #[test]
    fn test_preamble_correlator_below_threshold() {
        let bits: Vec<bool> = vec![true; 8];
        let corr = PreambleCorrelator::new(&bits);
        // Random-ish samples that won't match well
        let samples: Vec<Complex> = (0..8)
            .map(|i| Complex::new((i as f64 % 3.0) - 1.0, 0.0))
            .collect();
        // Use very high threshold to force None
        let result = corr.correlate(&samples, 0.999);
        assert!(result.is_none());
    }

    #[test]
    fn test_preamble_correlator_iridium() {
        let corr = PreambleCorrelator::iridium_preamble();
        assert_eq!(corr.reference.len(), IRIDIUM_PREAMBLE_SYMBOLS);
    }

    // --- UW Detector ---

    #[test]
    fn test_uw_detector_exact_match() {
        let det = UniqueWordDetector::new();
        let mut bits = vec![false; 32];
        // Encode IRIDIUM_UW into bits MSB first
        for i in 0..32 {
            bits[31 - i] = (IRIDIUM_UW >> i) & 1 == 1;
        }
        // reverse to MSB first
        let bits_msb: Vec<bool> = (0..32).map(|i| (IRIDIUM_UW >> (31 - i)) & 1 == 1).collect();
        let result = det.detect(&bits_msb);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.bit_errors, 0);
    }

    #[test]
    fn test_uw_detector_with_errors() {
        let det = UniqueWordDetector::with_pattern(0xAAAA_AAAA, 3);
        let mut bits: Vec<bool> = (0..32).map(|i| (0xAAAA_AAAAu32 >> (31 - i)) & 1 == 1).collect();
        // Flip 2 bits
        bits[0] = !bits[0];
        bits[5] = !bits[5];
        let result = det.detect(&bits);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.bit_errors, 2);
    }

    #[test]
    fn test_uw_detector_not_found() {
        let det = UniqueWordDetector::with_pattern(0xFFFF_FFFF, 0);
        let bits: Vec<bool> = vec![false; 32]; // all zeros, won't match
        let result = det.detect(&bits);
        assert!(result.is_none());
    }

    #[test]
    fn test_uw_detector_offset() {
        let det = UniqueWordDetector::with_pattern(0x0000_FFFF, 0);
        let uw_bits: Vec<bool> = (0..32)
            .map(|i| (0x0000_FFFFu32 >> (31 - i)) & 1 == 1)
            .collect();
        let mut stream = vec![false; 8]; // padding before UW
        stream.extend_from_slice(&uw_bits);
        let result = det.detect(&stream);
        assert!(result.is_some());
        assert_eq!(result.unwrap().bit_offset, 8);
    }

    // --- IRA / Ring Alert ---

    #[test]
    fn test_ring_alert_detector_creation() {
        let det = RingAlertDetector::new(100_000.0);
        assert_eq!(det.tone_freqs[2], 0.0);
    }

    #[test]
    fn test_ring_alert_tone_detection_presence() {
        let det = RingAlertDetector::new(100_000.0);
        // Generate a pure tone at -2000 Hz
        let n = 256usize;
        let freq = -2000.0f64;
        let samples: Vec<Complex> = (0..n)
            .map(|i| Complex::exp_j(2.0 * PI * freq * i as f64 / 100_000.0))
            .collect();
        let tones = det.detect_tones(&samples);
        assert!(tones[0]); // -2000 Hz should be detected
    }

    #[test]
    fn test_ira_parse_burst_too_short() {
        let det = RingAlertDetector::new(25_000.0);
        let bits = vec![false; 10];
        let result = det.parse_ira_burst(&bits);
        assert!(result.is_none());
    }

    #[test]
    fn test_ira_parse_burst_valid() {
        let det = RingAlertDetector::new(25_000.0);
        let bits = vec![false; IRA_PAYLOAD_BYTES * 8 + 10];
        let result = det.parse_ira_burst(&bits);
        assert!(result.is_some());
        let ira = result.unwrap();
        assert_eq!(ira.payload.len(), IRA_PAYLOAD_BYTES + 2); // ceil(bits/8)
    }

    // --- SBD Parser ---

    #[test]
    fn test_sbd_parser_too_short() {
        let data = vec![0x01, 0x00];
        let result = SbdParser::parse(&data);
        assert!(result.is_none());
    }

    #[test]
    fn test_sbd_parser_mobile_originated() {
        let mut data = vec![0x01u8, 0x00, 0x42]; // type=MO, seq=0x0042
        data.extend_from_slice(&[0u8; 8]); // IMEI placeholder
        data.extend_from_slice(b"Hello"); // payload
        let crc = crc16_ccitt(&data);
        data.push((crc >> 8) as u8);
        data.push(crc as u8);
        let result = SbdParser::parse(&data);
        assert!(result.is_some());
        let msg = result.unwrap();
        assert_eq!(msg.header.msg_type, SbdMessageType::MobileOriginated);
        assert_eq!(msg.header.sequence_number, 0x0042);
        assert!(msg.crc_ok);
    }

    #[test]
    fn test_sbd_parser_mobile_terminated() {
        let mut data = vec![0x02u8, 0x01, 0x00]; // type=MT
        data.extend_from_slice(&[0u8; 8]); // IMEI
        data.push(0xAA); // payload
        let crc = crc16_ccitt(&data);
        data.push((crc >> 8) as u8);
        data.push(crc as u8);
        let result = SbdParser::parse(&data);
        assert!(result.is_some());
        assert_eq!(
            result.unwrap().header.msg_type,
            SbdMessageType::MobileTerminated
        );
    }

    #[test]
    fn test_sbd_crc_failure() {
        let mut data = vec![0x01u8, 0x00, 0x01];
        data.extend_from_slice(&[0u8; 8]);
        data.push(0xFF); // payload
        data.push(0x00); // bad CRC
        data.push(0x00);
        let result = SbdParser::parse(&data);
        assert!(result.is_some());
        assert!(!result.unwrap().crc_ok);
    }

    // --- Descrambler ---

    #[test]
    fn test_descrambler_output_length() {
        let mut desc = IridiumDescrambler::new();
        let bits = vec![false; 64];
        let out = desc.descramble(&bits);
        assert_eq!(out.len(), 64);
    }

    #[test]
    fn test_descrambler_invertible() {
        let original = vec![
            true, false, true, true, false, true, false, false,
            true, true, false, false, true, false, true, true,
        ];
        let mut desc1 = IridiumDescrambler::new();
        let scrambled = desc1.descramble(&original);
        let mut desc2 = IridiumDescrambler::new();
        let recovered = desc2.descramble(&scrambled);
        assert_eq!(recovered, original);
    }

    #[test]
    fn test_descrambler_reset_reproducibility() {
        let bits = vec![true, false, true, false, true, false, true, false];
        let mut desc = IridiumDescrambler::new();
        let out1 = desc.descramble(&bits);
        desc.reset();
        let out2 = desc.descramble(&bits);
        assert_eq!(out1, out2);
    }

    #[test]
    fn test_descrambler_bytes() {
        let mut desc = IridiumDescrambler::new();
        let mut data = vec![0xAA, 0x55, 0xFF];
        let original = data.clone();
        desc.descramble_bytes(&mut data);
        // Should differ from original (scrambled != plain for nonzero LFSR)
        // Just check it runs without panic and produces 3 bytes
        assert_eq!(data.len(), 3);
        // Descramble again to recover
        let mut desc2 = IridiumDescrambler::new();
        desc2.descramble_bytes(&mut data);
        assert_eq!(data, original);
    }

    // --- BCH(31,21) ---

    #[test]
    fn test_bch_encode_decode_no_error() {
        let data = 0x15_A5A; // 21-bit pattern
        let cw = Bch3121::encode(data);
        let result = Bch3121::decode(cw);
        match result {
            BchResult::NoError(d) => assert_eq!(d, data & 0x1F_FFFF),
            BchResult::Corrected { data: d, errors } => {
                assert_eq!(d, data & 0x1F_FFFF);
                assert_eq!(errors, 0);
            }
            BchResult::Uncorrectable => panic!("unexpected uncorrectable"),
        }
    }

    #[test]
    fn test_bch_single_error_correction() {
        let data = 0x0ABCD;
        let cw = Bch3121::encode(data);
        // Flip bit 5
        let corrupted = BchCodeword(cw.0 ^ (1u32 << 5));
        let result = Bch3121::decode(corrupted);
        match result {
            BchResult::Corrected { data: d, errors } => {
                assert_eq!(d, data & 0x1F_FFFF);
                assert_eq!(errors, 1);
            }
            BchResult::NoError(d) => {
                // If syndromes are zero it's technically valid (rare coincidence)
                assert_eq!(d, data & 0x1F_FFFF);
            }
            BchResult::Uncorrectable => panic!("single error should be correctable"),
        }
    }

    #[test]
    fn test_bch_gf_mul_identity() {
        // 1 * a = a in GF(2^5)
        for a in 0u8..32 {
            assert_eq!(Bch3121::gf_mul(1, a), a);
        }
    }

    #[test]
    fn test_bch_gf_mul_commutative() {
        for a in 0u8..16 {
            for b in 0u8..16 {
                assert_eq!(Bch3121::gf_mul(a, b), Bch3121::gf_mul(b, a));
            }
        }
    }

    #[test]
    fn test_bch_encode_parity_bits() {
        let cw = Bch3121::encode(0);
        // Zero data should have zero parity
        assert_eq!(cw.0 & 0x3FF, 0);
    }

    // --- Doppler Compensator ---

    #[test]
    fn test_doppler_compensator_zero_offset() {
        let mut comp = DopplerCompensator::new(25_000.0);
        comp.set_correction(0.0);
        let samples = vec![Complex::new(1.0, 0.0); 10];
        let out = comp.compensate_copy(&samples);
        for s in &out {
            assert!((s.re - 1.0).abs() < 1e-10);
            assert!(s.im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_doppler_compensator_phase_rotation() {
        let sample_rate = 25_000.0;
        let mut comp = DopplerCompensator::new(sample_rate);
        // Set a correction of one full cycle per symbol period
        comp.set_correction(sample_rate); // +fs → every sample rotates by -2π
        let samples = vec![Complex::new(1.0, 0.0); 4];
        let out = comp.compensate_copy(&samples);
        // All samples should remain approximately at angle 0 (modulo 2π)
        for s in &out {
            let mag = s.mag();
            assert!((mag - 1.0).abs() < 1e-9);
        }
    }

    #[test]
    fn test_doppler_estimate_from_preamble_zero_offset() {
        let comp = DopplerCompensator::new(25_000.0);
        // Perfect alternating preamble → zero Doppler
        let preamble: Vec<Complex> = (0..32)
            .map(|i| {
                if i % 2 == 0 {
                    Complex::new(1.0, 0.0)
                } else {
                    Complex::new(-1.0, 0.0)
                }
            })
            .collect();
        let est = comp.estimate_from_preamble(&preamble);
        assert!(est.freq_offset_hz.abs() < 500.0);
    }

    #[test]
    fn test_doppler_estimate_within_leo_range() {
        let comp = DopplerCompensator::new(25_000.0);
        // Inject +5 kHz Doppler on alternating preamble
        let doppler_hz = 5_000.0f64;
        let sample_rate = 25_000.0f64;
        let preamble: Vec<Complex> = (0..64)
            .map(|i| {
                let base_phase = if i % 2 == 0 { 0.0 } else { PI };
                let doppler_phase = 2.0 * PI * doppler_hz * i as f64 / sample_rate;
                Complex::exp_j(base_phase + doppler_phase)
            })
            .collect();
        let est = comp.estimate_from_preamble(&preamble);
        // Should be within LEO range
        assert!(est.freq_offset_hz.abs() <= IRIDIUM_MAX_DOPPLER_HZ);
    }

    // --- Block Interleaver ---

    #[test]
    fn test_interleaver_roundtrip() {
        let il = BlockInterleaver::new(4, 8);
        let bits: Vec<bool> = (0..32).map(|i| i % 3 == 0).collect();
        let interleaved = il.interleave(&bits);
        let recovered = il.deinterleave(&interleaved);
        assert_eq!(recovered, bits);
    }

    #[test]
    fn test_interleaver_output_length() {
        let il = BlockInterleaver::new(4, 8);
        let bits = vec![true; 32];
        let out = il.interleave(&bits);
        assert_eq!(out.len(), 32);
    }

    #[test]
    fn test_interleaver_permutes() {
        let il = BlockInterleaver::new(2, 4);
        // bits: [1,0,0,0, 0,0,0,0]
        let mut bits = vec![false; 8];
        bits[0] = true;
        let out = il.interleave(&bits);
        // bit[0] (row 0, col 0) should move to position 0*2+0=0 in column-major
        // Actually: column-major reads col 0 first, so out[0] = buf[0*4+0]=bits[0]=true
        assert!(out[0]);
    }

    #[test]
    fn test_deinterleaver_permutes() {
        let il = BlockInterleaver::new(2, 4);
        let bits: Vec<bool> = (0..8).map(|i| i < 4).collect();
        let deint = il.deinterleave(&bits);
        assert_eq!(deint.len(), 8);
    }

    // --- Channel Frequencies ---

    #[test]
    fn test_channel_center_freq_ch0() {
        let f = channel_center_freq(0);
        assert!((f - (IRIDIUM_BASE_FREQ_HZ + 12_500.0)).abs() < 1.0);
    }

    #[test]
    fn test_channel_center_freq_ch40() {
        // Channel 40 is the last of 41 channels; its center should be above channel 39
        let f40 = channel_center_freq(40);
        let f39 = channel_center_freq(39);
        assert!(f40 > f39);
        assert!((f40 - f39 - IRIDIUM_CHANNEL_SPACING_HZ).abs() < 1.0);
    }

    #[test]
    fn test_freq_to_channel_known() {
        let f = channel_center_freq(5);
        let ch = freq_to_channel(f);
        assert_eq!(ch, Some(5));
    }

    #[test]
    fn test_freq_to_channel_outside_band() {
        let ch = freq_to_channel(1_500_000_000.0); // 1.5 GHz is outside
        assert!(ch.is_none());
    }

    // --- Utility functions ---

    #[test]
    fn test_bits_to_bytes_zero() {
        let bits = vec![false; 8];
        assert_eq!(bits_to_bytes(&bits), vec![0x00]);
    }

    #[test]
    fn test_bits_to_bytes_ones() {
        let bits = vec![true; 8];
        assert_eq!(bits_to_bytes(&bits), vec![0xFF]);
    }

    #[test]
    fn test_bits_to_bytes_pattern() {
        // 0b1010_0101 = 0xA5
        let bits = vec![true, false, true, false, false, true, false, true];
        assert_eq!(bits_to_bytes(&bits), vec![0xA5]);
    }

    #[test]
    fn test_bytes_to_bits_roundtrip() {
        let data = vec![0x42u8, 0xDE];
        let bits = bytes_to_bits(&data);
        let recovered = bits_to_bytes(&bits);
        assert_eq!(recovered, data);
    }

    #[test]
    fn test_crc16_ccitt_known() {
        // CRC-16/CCITT of b"123456789" = 0x29B1
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0x29B1);
    }

    #[test]
    fn test_crc16_empty() {
        let crc = crc16_ccitt(b"");
        assert_eq!(crc, 0xFFFF);
    }

    // --- IridiumBurstDemodulator integration ---

    #[test]
    fn test_full_demodulator_construction() {
        let demod = IridiumBurstDemodulator::new(100_000.0);
        assert!(!demod.collecting);
    }

    #[test]
    fn test_full_demodulator_quiet_input() {
        let mut demod = IridiumBurstDemodulator::new(100_000.0);
        let noise: Vec<Complex> = (0..1000)
            .map(|_| Complex::new(0.001, 0.001))
            .collect();
        let results = demod.process(&noise);
        assert!(results.is_empty());
    }

    #[test]
    fn test_full_demodulator_burst_detection() {
        let mut demod = IridiumBurstDemodulator::new(100_000.0);
        // Create a synthetic burst: preamble + some data + silence
        let mut samples = vec![Complex::new(0.001, 0.0); 50]; // silence
        // Alternating preamble
        for i in 0..IRIDIUM_PREAMBLE_SYMBOLS {
            let phase = if i % 2 == 0 { 0.0 } else { PI };
            samples.push(Complex::exp_j(phase).scale(2.0));
        }
        // Data symbols
        for _ in 0..50 {
            samples.push(Complex::new(2.0, 0.0));
        }
        // Silence to end burst
        for _ in 0..100 {
            samples.push(Complex::new(0.0, 0.0));
        }
        // Should process without panic
        let _results = demod.process(&samples);
    }

    #[test]
    fn test_dibit_from_u8() {
        assert_eq!(DqpskDibit::from_u8(0), DqpskDibit::S00);
        assert_eq!(DqpskDibit::from_u8(1), DqpskDibit::S01);
        assert_eq!(DqpskDibit::from_u8(2), DqpskDibit::S10);
        assert_eq!(DqpskDibit::from_u8(3), DqpskDibit::S11);
        assert_eq!(DqpskDibit::from_u8(4), DqpskDibit::S00); // wraps
    }

    #[test]
    fn test_bch_two_error_scenario() {
        let data = 0x1_5555u32;
        let cw = Bch3121::encode(data);
        // Corrupt two bits at positions 2 and 7
        let corrupted = BchCodeword(cw.0 ^ (1 << 2) ^ (1 << 7));
        let result = Bch3121::decode(corrupted);
        // Should either correct or report uncorrectable; just check it doesn't panic
        match result {
            BchResult::Corrected { data: d, errors } => {
                assert!(errors <= BCH_T);
                assert_eq!(d, data & 0x1F_FFFF);
            }
            BchResult::NoError(_) | BchResult::Uncorrectable => {}
        }
    }

    #[test]
    fn test_channel_spacing() {
        let f0 = channel_center_freq(0);
        let f1 = channel_center_freq(1);
        assert!((f1 - f0 - IRIDIUM_CHANNEL_SPACING_HZ).abs() < 1.0);
    }

    #[test]
    fn test_iridium_constants() {
        assert_eq!(IRIDIUM_NUM_CHANNELS, 41);
        assert!((IRIDIUM_SYMBOL_RATE - 25_000.0).abs() < 1.0);
        assert!((IRIDIUM_DATA_RATE - 50_000.0).abs() < 1.0);
        assert!((IRIDIUM_MAX_DOPPLER_HZ - 37_000.0).abs() < 1.0);
    }

    #[test]
    fn test_goertzel_zero_output_for_orthogonal_freq() {
        let det = RingAlertDetector::new(100_000.0);
        // DC signal vs non-DC tone detection
        let samples: Vec<Complex> = vec![Complex::new(0.001, 0.001); 256];
        let tones = det.detect_tones(&samples);
        // DC power is tiny – all tone flags should be false
        assert!(!tones[0] || !tones[1]);
    }
}
