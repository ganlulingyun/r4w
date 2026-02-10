//! IRIG-B Time Code Decoder — Precision timing for telemetry and test ranges
//!
//! Decodes IRIG-B time codes from AM-modulated audio or baseband samples.
//! IRIG-B (Inter-Range Instrumentation Group, Format B) encodes BCD time
//! into a 100-bit-per-second serial frame with a 1 kHz AM carrier. Pulse
//! widths encode binary 0 (2 ms), binary 1 (5 ms), and position markers
//! (8 ms). Ten position markers per one-second frame provide framing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::irig_b_decoder::{IrigBEncoder, IrigBDecoder, IrigBFrame};
//!
//! let frame = IrigBFrame {
//!     seconds: 30,
//!     minutes: 15,
//!     hours: 12,
//!     day_of_year: 45,
//!     year: 26,
//! };
//! let encoder = IrigBEncoder::new(10000.0);
//! let samples = encoder.encode(&frame);
//!
//! // Feed two identical frames: the first establishes sync, the second is decoded.
//! let mut two_frames = samples.clone();
//! two_frames.extend_from_slice(&samples);
//!
//! let mut decoder = IrigBDecoder::new(10000.0);
//! let decoded = decoder.process(&two_frames);
//! assert_eq!(decoded.len(), 1);
//! assert_eq!(decoded[0].hours, 12);
//! assert_eq!(decoded[0].minutes, 15);
//! assert_eq!(decoded[0].seconds, 30);
//! ```

use std::f64::consts::PI;

/// IRIG-B carrier frequency in Hz.
const CARRIER_FREQ: f64 = 1000.0;

/// Bits per IRIG-B frame (one frame = one second).
const BITS_PER_FRAME: usize = 100;

/// Bit period in seconds.
const BIT_PERIOD: f64 = 0.01;

/// Pulse width thresholds in seconds.
const ZERO_WIDTH: f64 = 0.002;
const ONE_WIDTH: f64 = 0.005;
const MARKER_WIDTH: f64 = 0.008;

/// Pulse type decoded from envelope width.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PulseType {
    /// Binary 0 — 2 ms high pulse.
    Zero,
    /// Binary 1 — 5 ms high pulse.
    One,
    /// Position marker — 8 ms high pulse.
    Marker,
}

/// A decoded IRIG-B time frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IrigBFrame {
    /// Seconds (0-59).
    pub seconds: u8,
    /// Minutes (0-59).
    pub minutes: u8,
    /// Hours (0-23).
    pub hours: u8,
    /// Day of year (1-366).
    pub day_of_year: u16,
    /// Two-digit year (0-99).
    pub year: u16,
}

impl IrigBFrame {
    /// Check whether the frame contains valid time values.
    pub fn is_valid(&self) -> bool {
        self.seconds < 60
            && self.minutes < 60
            && self.hours < 24
            && self.day_of_year >= 1
            && self.day_of_year <= 366
            && self.year < 100
    }

    /// Format the frame as a human-readable time string.
    pub fn to_string(&self) -> String {
        format!(
            "Day {:03} {:02}:{:02}:{:02} (year {:02})",
            self.day_of_year, self.hours, self.minutes, self.seconds, self.year
        )
    }
}

/// Decode a BCD-encoded bit slice to an integer.
///
/// Each group of 4 bits represents one decimal digit (LSB first within each
/// nibble). Groups are ordered least-significant digit first.
pub fn decode_bcd(bits: &[bool]) -> u16 {
    let mut value: u16 = 0;
    let mut multiplier: u16 = 1;
    for chunk in bits.chunks(4) {
        let mut digit: u16 = 0;
        for (i, &b) in chunk.iter().enumerate() {
            if b {
                digit += 1 << i;
            }
        }
        value += digit * multiplier;
        multiplier *= 10;
    }
    value
}

/// Encode an integer as BCD bits.
///
/// Returns `num_digits * 4` bits, LSB first within each nibble, least
/// significant digit first.
pub fn encode_bcd(value: u16, num_digits: usize) -> Vec<bool> {
    let mut bits = Vec::with_capacity(num_digits * 4);
    let mut remaining = value;
    for _ in 0..num_digits {
        let digit = remaining % 10;
        for bit_idx in 0..4 {
            bits.push((digit >> bit_idx) & 1 == 1);
        }
        remaining /= 10;
    }
    bits
}

/// Classify a pulse from its envelope samples.
///
/// Measures the width of the high-amplitude region within one bit period
/// and returns the corresponding pulse type, or `None` if the pulse is
/// ambiguous.
pub fn detect_pulse_width(
    samples: &[f64],
    sample_rate: f64,
    threshold: f64,
) -> Option<PulseType> {
    let high_count = samples.iter().filter(|&&s| s > threshold).count();
    let high_duration = high_count as f64 / sample_rate;

    // Decision boundaries halfway between nominal widths.
    let boundary_01 = (ZERO_WIDTH + ONE_WIDTH) / 2.0; // 3.5 ms
    let boundary_1m = (ONE_WIDTH + MARKER_WIDTH) / 2.0; // 6.5 ms

    if high_duration < ZERO_WIDTH * 0.3 {
        None // too short to be a valid pulse
    } else if high_duration < boundary_01 {
        Some(PulseType::Zero)
    } else if high_duration < boundary_1m {
        Some(PulseType::One)
    } else {
        Some(PulseType::Marker)
    }
}

/// IRIG-B time code decoder.
///
/// Processes AM-modulated IRIG-B audio samples and produces decoded time
/// frames. Operates as a streaming decoder — feed successive sample buffers
/// to [`process`](IrigBDecoder::process) and collect completed frames.
#[derive(Debug, Clone)]
pub struct IrigBDecoder {
    sample_rate: f64,
    samples_per_bit: usize,
    /// Envelope detector state (single-pole lowpass).
    env_state: f64,
    /// Envelope alpha coefficient.
    env_alpha: f64,
    /// Running peak for adaptive threshold.
    peak: f64,
    /// Accumulated envelope samples for current bit.
    bit_buffer: Vec<f64>,
    /// Decoded pulse types for the current frame.
    pulse_buffer: Vec<PulseType>,
    /// Whether we have frame sync (found two consecutive markers).
    synced: bool,
}

impl IrigBDecoder {
    /// Create a new decoder for the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        let samples_per_bit = (sample_rate * BIT_PERIOD).round() as usize;
        // Envelope lowpass cutoff ~200 Hz — well below carrier, above bit rate.
        let cutoff = 200.0;
        let env_alpha = 1.0 - (-2.0 * PI * cutoff / sample_rate).exp();
        Self {
            sample_rate,
            samples_per_bit,
            env_state: 0.0,
            env_alpha,
            peak: 0.0,
            bit_buffer: Vec::with_capacity(samples_per_bit),
            pulse_buffer: Vec::new(),
            synced: false,
        }
    }

    /// Reset decoder state, discarding any partial frame.
    pub fn reset(&mut self) {
        self.env_state = 0.0;
        self.peak = 0.0;
        self.bit_buffer.clear();
        self.pulse_buffer.clear();
        self.synced = false;
    }

    /// Process a buffer of audio samples and return any completed frames.
    pub fn process(&mut self, samples: &[f64]) -> Vec<IrigBFrame> {
        let mut frames = Vec::new();

        for &s in samples {
            // Envelope detection: rectify then lowpass.
            let rectified = s.abs();
            self.env_state += self.env_alpha * (rectified - self.env_state);
            let envelope = self.env_state;

            // Track peak with slow decay.
            if envelope > self.peak {
                self.peak = envelope;
            } else {
                self.peak *= 0.9999;
            }

            self.bit_buffer.push(envelope);

            if self.bit_buffer.len() >= self.samples_per_bit {
                // Classify this bit period.
                let threshold = self.peak * 0.5;
                if let Some(pulse) =
                    detect_pulse_width(&self.bit_buffer, self.sample_rate, threshold)
                {
                    self.handle_pulse(pulse, &mut frames);
                }
                self.bit_buffer.clear();
            }
        }

        frames
    }

    /// Handle a decoded pulse, maintaining frame sync and accumulating bits.
    fn handle_pulse(&mut self, pulse: PulseType, frames: &mut Vec<IrigBFrame>) {
        if !self.synced {
            // Look for two consecutive markers to establish frame sync.
            // In IRIG-B the reference marker (P0) is followed by the first
            // bit of the seconds field; we detect sync when we see the pair
            // Pr (end of previous frame) + P0 (start of new frame).
            if pulse == PulseType::Marker {
                if let Some(&PulseType::Marker) = self.pulse_buffer.last() {
                    // Two consecutive markers: the second one is P0.
                    self.pulse_buffer.clear();
                    self.pulse_buffer.push(PulseType::Marker); // P0
                    self.synced = true;
                    return;
                }
            }
            self.pulse_buffer.clear();
            self.pulse_buffer.push(pulse);
            return;
        }

        self.pulse_buffer.push(pulse);

        // A complete frame is 100 pulses (P0 at index 0 through Pr at index 99).
        if self.pulse_buffer.len() == BITS_PER_FRAME {
            if let Some(frame) = Self::decode_frame(&self.pulse_buffer) {
                frames.push(frame);
            }
            self.pulse_buffer.clear();
            // Stay synced — next pulse should be P0 of the following frame.
            // We require P0 marker to maintain sync.
            self.synced = true;
        }
    }

    /// Attempt to decode a 100-pulse buffer into an IrigBFrame.
    ///
    /// IRIG-B frame layout (bit indices 0-99):
    ///
    /// ```text
    /// [0]      P0  — position marker (reference)
    /// [1..5]   seconds units (BCD, 4 bits)
    /// [5]      unused (0)
    /// [6..9]   seconds tens (BCD, 3 bits used)
    /// [9]      P1  — position marker
    /// [10..14] minutes units (BCD, 4 bits)
    /// [14]     unused (0)
    /// [15..18] minutes tens (BCD, 3 bits used)
    /// [18]     unused (0)
    /// [19]     P2  — position marker
    /// [20..24] hours units (BCD, 4 bits)
    /// [24]     unused (0)
    /// [25..28] hours tens (BCD, 2 bits used)
    /// [28]     unused (0)
    /// [29]     P3  — position marker
    /// [30..34] day-of-year units (BCD, 4 bits)
    /// [34]     unused (0)
    /// [35..38] day-of-year tens (BCD, 4 bits)
    /// [39]     P4  — position marker
    /// [40..43] day-of-year hundreds (BCD, 2 bits used)
    /// [43..49] unused (0s)
    /// [49]     P5  — position marker
    /// [50..54] year units (BCD, 4 bits)
    /// [54]     unused (0)
    /// [55..58] year tens (BCD, 4 bits)
    /// [58]     unused (0)
    /// [59]     P6  — position marker
    /// [60..69] control functions (unused here)
    /// [69]     P7  — position marker
    /// [70..79] control functions (unused here)
    /// [79]     P8  — position marker
    /// [80..89] straight binary seconds (unused here)
    /// [89]     P9  — position marker
    /// [90..98] straight binary seconds cont. (unused here)
    /// [99]     Pr  — reference marker
    /// ```
    fn decode_frame(pulses: &[PulseType]) -> Option<IrigBFrame> {
        if pulses.len() != BITS_PER_FRAME {
            return None;
        }

        // Verify position markers at expected locations.
        let marker_positions = [0, 9, 19, 29, 39, 49, 59, 69, 79, 89, 99];
        // We only have 100 elements (indices 0..99), so Pr at index 99 is present.
        for &pos in &marker_positions {
            if pos < pulses.len() && pulses[pos] != PulseType::Marker {
                return None;
            }
        }

        // Helper to convert pulses to bool bits (Marker treated as 0).
        let to_bits = |range: std::ops::Range<usize>| -> Vec<bool> {
            range
                .map(|i| pulses[i] == PulseType::One)
                .collect()
        };

        // Seconds: units [1..5], tens [6..9]
        let sec_units = decode_bcd(&to_bits(1..5));
        let sec_tens = decode_bcd(&to_bits(6..9));
        let seconds = (sec_tens * 10 + sec_units) as u8;

        // Minutes: units [10..14], tens [15..18]
        let min_units = decode_bcd(&to_bits(10..14));
        let min_tens = decode_bcd(&to_bits(15..18));
        let minutes = (min_tens * 10 + min_units) as u8;

        // Hours: units [20..24], tens [25..28]
        let hr_units = decode_bcd(&to_bits(20..24));
        let hr_tens = decode_bcd(&to_bits(25..28));
        let hours = (hr_tens * 10 + hr_units) as u8;

        // Day of year: units [30..34], tens [35..39], hundreds [40..43]
        let day_units = decode_bcd(&to_bits(30..34));
        let day_tens = decode_bcd(&to_bits(35..39));
        let day_hundreds = decode_bcd(&to_bits(40..43));
        let day_of_year = day_hundreds * 100 + day_tens * 10 + day_units;

        // Year: units [50..54], tens [55..59]
        let yr_units = decode_bcd(&to_bits(50..54));
        let yr_tens = decode_bcd(&to_bits(55..59));
        let year = yr_tens * 10 + yr_units;

        Some(IrigBFrame {
            seconds,
            minutes,
            hours,
            day_of_year,
            year,
        })
    }
}

/// IRIG-B time code encoder.
///
/// Generates AM-modulated 1 kHz audio carrying IRIG-B time data.
/// Useful for testing the decoder and generating reference signals.
#[derive(Debug, Clone)]
pub struct IrigBEncoder {
    sample_rate: f64,
}

impl IrigBEncoder {
    /// Create an encoder at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        Self { sample_rate }
    }

    /// Encode a time frame into AM-modulated audio samples.
    ///
    /// Returns exactly `sample_rate` samples (one second of audio).
    pub fn encode(&self, frame: &IrigBFrame) -> Vec<f64> {
        let pulses = self.frame_to_pulses(frame);
        let total_samples = (self.sample_rate * 1.0).round() as usize; // 1 second
        let samples_per_bit = (self.sample_rate * BIT_PERIOD).round() as usize;
        let mut output = Vec::with_capacity(total_samples);

        for (bit_idx, pulse) in pulses.iter().enumerate() {
            let high_samples = match pulse {
                PulseType::Zero => (self.sample_rate * ZERO_WIDTH).round() as usize,
                PulseType::One => (self.sample_rate * ONE_WIDTH).round() as usize,
                PulseType::Marker => (self.sample_rate * MARKER_WIDTH).round() as usize,
            };

            for sample_idx in 0..samples_per_bit {
                let t = (bit_idx * samples_per_bit + sample_idx) as f64 / self.sample_rate;
                let carrier = (2.0 * PI * CARRIER_FREQ * t).sin();
                // AM modulation: high amplitude during pulse, low otherwise.
                let amplitude = if sample_idx < high_samples { 1.0 } else { 0.3 };
                output.push(amplitude * carrier);
            }
        }

        // Pad or trim to exact sample count.
        output.resize(total_samples, 0.0);
        output
    }

    /// Convert a frame to the 100-pulse IRIG-B sequence.
    fn frame_to_pulses(&self, frame: &IrigBFrame) -> Vec<PulseType> {
        let mut pulses = vec![PulseType::Zero; BITS_PER_FRAME];

        // Position markers.
        let marker_positions = [0, 9, 19, 29, 39, 49, 59, 69, 79, 89, 99];
        for &pos in &marker_positions {
            if pos < BITS_PER_FRAME {
                pulses[pos] = PulseType::Marker;
            }
        }

        // Helper: write BCD bits into the pulse array.
        let write_bits = |pulses: &mut Vec<PulseType>, start: usize, bits: &[bool]| {
            for (i, &b) in bits.iter().enumerate() {
                pulses[start + i] = if b { PulseType::One } else { PulseType::Zero };
            }
        };

        // Seconds: units [1..5], tens [6..9]
        let sec_units_bits = encode_bcd(frame.seconds as u16 % 10, 1);
        let sec_tens_bits = encode_bcd(frame.seconds as u16 / 10, 1);
        write_bits(&mut pulses, 1, &sec_units_bits);
        // Tens field only uses 3 bits (0-5).
        write_bits(&mut pulses, 6, &sec_tens_bits[..3]);

        // Minutes: units [10..14], tens [15..18]
        let min_units_bits = encode_bcd(frame.minutes as u16 % 10, 1);
        let min_tens_bits = encode_bcd(frame.minutes as u16 / 10, 1);
        write_bits(&mut pulses, 10, &min_units_bits);
        write_bits(&mut pulses, 15, &min_tens_bits[..3]);

        // Hours: units [20..24], tens [25..28]
        let hr_units_bits = encode_bcd(frame.hours as u16 % 10, 1);
        let hr_tens_bits = encode_bcd(frame.hours as u16 / 10, 1);
        write_bits(&mut pulses, 20, &hr_units_bits);
        write_bits(&mut pulses, 25, &hr_tens_bits[..3]);

        // Day of year: units [30..34], tens [35..39], hundreds [40..43]
        let day = frame.day_of_year;
        let day_units_bits = encode_bcd(day % 10, 1);
        let day_tens_bits = encode_bcd((day / 10) % 10, 1);
        let day_hundreds_bits = encode_bcd((day / 100) % 10, 1);
        write_bits(&mut pulses, 30, &day_units_bits);
        write_bits(&mut pulses, 35, &day_tens_bits);
        write_bits(&mut pulses, 40, &day_hundreds_bits[..3]);

        // Year: units [50..54], tens [55..59]
        let yr_units_bits = encode_bcd(frame.year % 10, 1);
        let yr_tens_bits = encode_bcd((frame.year / 10) % 10, 1);
        write_bits(&mut pulses, 50, &yr_units_bits);
        write_bits(&mut pulses, 55, &yr_tens_bits);

        pulses
    }
}

/// Create a decoder configured for 44.1 kHz audio input.
pub fn irig_b_audio() -> IrigBDecoder {
    IrigBDecoder::new(44100.0)
}

/// Create a decoder configured for 10 kHz baseband input.
pub fn irig_b_baseband() -> IrigBDecoder {
    IrigBDecoder::new(10000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_frame() -> IrigBFrame {
        IrigBFrame {
            seconds: 45,
            minutes: 30,
            hours: 14,
            day_of_year: 123,
            year: 26,
        }
    }

    #[test]
    fn test_bcd_round_trip() {
        // Single-digit values.
        for v in 0..10u16 {
            let bits = encode_bcd(v, 1);
            assert_eq!(bits.len(), 4);
            assert_eq!(decode_bcd(&bits), v);
        }
        // Multi-digit values.
        for v in [0, 7, 42, 99, 365, 999] {
            let digits = if v < 10 {
                1
            } else if v < 100 {
                2
            } else {
                3
            };
            let bits = encode_bcd(v, digits);
            assert_eq!(bits.len(), digits * 4);
            assert_eq!(decode_bcd(&bits), v, "BCD round-trip failed for {}", v);
        }
    }

    #[test]
    fn test_pulse_type_classification() {
        let sample_rate = 10000.0;
        let samples_per_bit = (sample_rate * BIT_PERIOD) as usize; // 100

        // Generate pulse with known high duration.
        let make_pulse = |high_samples: usize| -> Vec<f64> {
            let mut v = vec![0.1; samples_per_bit];
            for s in v.iter_mut().take(high_samples) {
                *s = 1.0;
            }
            v
        };

        // 2 ms = 20 samples at 10 kHz -> Zero
        assert_eq!(
            detect_pulse_width(&make_pulse(20), sample_rate, 0.5),
            Some(PulseType::Zero)
        );
        // 5 ms = 50 samples at 10 kHz -> One
        assert_eq!(
            detect_pulse_width(&make_pulse(50), sample_rate, 0.5),
            Some(PulseType::One)
        );
        // 8 ms = 80 samples at 10 kHz -> Marker
        assert_eq!(
            detect_pulse_width(&make_pulse(80), sample_rate, 0.5),
            Some(PulseType::Marker)
        );
    }

    #[test]
    fn test_encoder_output_length() {
        let encoder = IrigBEncoder::new(10000.0);
        let frame = test_frame();
        let samples = encoder.encode(&frame);
        assert_eq!(samples.len(), 10000, "Encoder should produce exactly sample_rate samples");

        let encoder_44k = IrigBEncoder::new(44100.0);
        let samples_44k = encoder_44k.encode(&frame);
        assert_eq!(samples_44k.len(), 44100);
    }

    #[test]
    fn test_encode_then_decode() {
        let frame = test_frame();
        let encoder = IrigBEncoder::new(10000.0);
        let samples = encoder.encode(&frame);

        let mut decoder = IrigBDecoder::new(10000.0);
        // Feed a sync preamble: two marker pulses so the decoder can sync.
        // The easiest way is to send two frames — the first establishes sync.
        let mut two_frames = samples.clone();
        two_frames.extend_from_slice(&samples);

        let decoded = decoder.process(&two_frames);
        assert!(
            !decoded.is_empty(),
            "Decoder should produce at least one frame"
        );
        let d = &decoded[0];
        assert_eq!(d.seconds, frame.seconds);
        assert_eq!(d.minutes, frame.minutes);
        assert_eq!(d.hours, frame.hours);
        assert_eq!(d.day_of_year, frame.day_of_year);
        assert_eq!(d.year, frame.year);
    }

    #[test]
    fn test_valid_frame() {
        let good = test_frame();
        assert!(good.is_valid());

        let bad_sec = IrigBFrame { seconds: 60, ..good.clone() };
        assert!(!bad_sec.is_valid());

        let bad_min = IrigBFrame { minutes: 60, ..good.clone() };
        assert!(!bad_min.is_valid());

        let bad_hr = IrigBFrame { hours: 24, ..good.clone() };
        assert!(!bad_hr.is_valid());

        let bad_day_zero = IrigBFrame { day_of_year: 0, ..good.clone() };
        assert!(!bad_day_zero.is_valid());

        let bad_day_high = IrigBFrame { day_of_year: 367, ..good.clone() };
        assert!(!bad_day_high.is_valid());

        let bad_year = IrigBFrame { year: 100, ..good.clone() };
        assert!(!bad_year.is_valid());
    }

    #[test]
    fn test_frame_to_string() {
        let frame = IrigBFrame {
            seconds: 5,
            minutes: 9,
            hours: 3,
            day_of_year: 7,
            year: 26,
        };
        let s = frame.to_string();
        assert_eq!(s, "Day 007 03:09:05 (year 26)");
    }

    #[test]
    fn test_position_marker_detection() {
        let encoder = IrigBEncoder::new(10000.0);
        let frame = test_frame();
        let pulses = encoder.frame_to_pulses(&frame);

        // Verify all 10 in-frame markers plus Pr.
        let marker_positions = [0, 9, 19, 29, 39, 49, 59, 69, 79, 89, 99];
        for &pos in &marker_positions {
            assert_eq!(
                pulses[pos],
                PulseType::Marker,
                "Expected marker at position {}",
                pos
            );
        }
    }

    #[test]
    fn test_audio_factory() {
        let decoder = irig_b_audio();
        assert_eq!(decoder.sample_rate, 44100.0);
        // 44100 * 0.01 = 441 samples per bit.
        assert_eq!(decoder.samples_per_bit, 441);
    }

    #[test]
    fn test_baseband_factory() {
        let decoder = irig_b_baseband();
        assert_eq!(decoder.sample_rate, 10000.0);
        assert_eq!(decoder.samples_per_bit, 100);
    }

    #[test]
    fn test_multiple_frames_in_sequence() {
        let frames = vec![
            IrigBFrame {
                seconds: 10,
                minutes: 20,
                hours: 8,
                day_of_year: 200,
                year: 25,
            },
            IrigBFrame {
                seconds: 11,
                minutes: 20,
                hours: 8,
                day_of_year: 200,
                year: 25,
            },
            IrigBFrame {
                seconds: 12,
                minutes: 20,
                hours: 8,
                day_of_year: 200,
                year: 25,
            },
        ];

        let encoder = IrigBEncoder::new(10000.0);
        let mut all_samples = Vec::new();
        for f in &frames {
            all_samples.extend_from_slice(&encoder.encode(f));
        }

        let mut decoder = IrigBDecoder::new(10000.0);
        let decoded = decoder.process(&all_samples);

        // First frame is used for sync, so we expect frames[1] and frames[2].
        assert!(
            decoded.len() >= 2,
            "Expected at least 2 decoded frames, got {}",
            decoded.len()
        );
        assert_eq!(decoded[0].seconds, 11);
        assert_eq!(decoded[1].seconds, 12);
        assert_eq!(decoded[0].day_of_year, 200);
        assert_eq!(decoded[1].year, 25);
    }
}
