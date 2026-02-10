//! DTMF (Dual-Tone Multi-Frequency) tone detection and generation per ITU-T Q.23/Q.24.
//!
//! This module provides a Goertzel-algorithm-based DTMF detector and a tone generator
//! for the standard 16-digit DTMF keypad (0-9, *, #, A-D).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::dtmf_detector::{DtmfConfig, DtmfDetector, DtmfGenerator, DtmfDigit};
//!
//! let sample_rate = 8000.0;
//! let generator = DtmfGenerator::new(sample_rate);
//! let tone = generator.generate(DtmfDigit::D5, 100.0);
//!
//! let config = DtmfConfig {
//!     sample_rate,
//!     min_duration_ms: 40.0,
//!     twist_db_max: 8.0,
//!     snr_threshold_db: 10.0,
//! };
//! let detector = DtmfDetector::new(config);
//! let events = detector.detect(&tone);
//! assert_eq!(events.len(), 1);
//! assert_eq!(events[0].digit, DtmfDigit::D5);
//! ```

use std::f64::consts::PI;

/// The 16 standard DTMF digits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DtmfDigit {
    D0,
    D1,
    D2,
    D3,
    D4,
    D5,
    D6,
    D7,
    D8,
    D9,
    Star,
    Hash,
    A,
    B,
    C,
    D,
}

impl DtmfDigit {
    /// Return the (low_freq, high_freq) pair for this digit.
    pub fn frequencies(self) -> (f64, f64) {
        match self {
            DtmfDigit::D1 => (697.0, 1209.0),
            DtmfDigit::D2 => (697.0, 1336.0),
            DtmfDigit::D3 => (697.0, 1477.0),
            DtmfDigit::A => (697.0, 1633.0),
            DtmfDigit::D4 => (770.0, 1209.0),
            DtmfDigit::D5 => (770.0, 1336.0),
            DtmfDigit::D6 => (770.0, 1477.0),
            DtmfDigit::B => (770.0, 1633.0),
            DtmfDigit::D7 => (852.0, 1209.0),
            DtmfDigit::D8 => (852.0, 1336.0),
            DtmfDigit::D9 => (852.0, 1477.0),
            DtmfDigit::C => (852.0, 1633.0),
            DtmfDigit::Star => (941.0, 1209.0),
            DtmfDigit::D0 => (941.0, 1336.0),
            DtmfDigit::Hash => (941.0, 1477.0),
            DtmfDigit::D => (941.0, 1633.0),
        }
    }

    /// Return the character representation of this digit.
    pub fn to_char(self) -> char {
        match self {
            DtmfDigit::D0 => '0',
            DtmfDigit::D1 => '1',
            DtmfDigit::D2 => '2',
            DtmfDigit::D3 => '3',
            DtmfDigit::D4 => '4',
            DtmfDigit::D5 => '5',
            DtmfDigit::D6 => '6',
            DtmfDigit::D7 => '7',
            DtmfDigit::D8 => '8',
            DtmfDigit::D9 => '9',
            DtmfDigit::Star => '*',
            DtmfDigit::Hash => '#',
            DtmfDigit::A => 'A',
            DtmfDigit::B => 'B',
            DtmfDigit::C => 'C',
            DtmfDigit::D => 'D',
        }
    }

    /// Parse a character into a DTMF digit.
    pub fn from_char(c: char) -> Option<DtmfDigit> {
        match c {
            '0' => Some(DtmfDigit::D0),
            '1' => Some(DtmfDigit::D1),
            '2' => Some(DtmfDigit::D2),
            '3' => Some(DtmfDigit::D3),
            '4' => Some(DtmfDigit::D4),
            '5' => Some(DtmfDigit::D5),
            '6' => Some(DtmfDigit::D6),
            '7' => Some(DtmfDigit::D7),
            '8' => Some(DtmfDigit::D8),
            '9' => Some(DtmfDigit::D9),
            '*' => Some(DtmfDigit::Star),
            '#' => Some(DtmfDigit::Hash),
            'A' | 'a' => Some(DtmfDigit::A),
            'B' | 'b' => Some(DtmfDigit::B),
            'C' | 'c' => Some(DtmfDigit::C),
            'D' | 'd' => Some(DtmfDigit::D),
            _ => None,
        }
    }

    /// All 16 DTMF digits.
    pub fn all() -> [DtmfDigit; 16] {
        [
            DtmfDigit::D0,
            DtmfDigit::D1,
            DtmfDigit::D2,
            DtmfDigit::D3,
            DtmfDigit::D4,
            DtmfDigit::D5,
            DtmfDigit::D6,
            DtmfDigit::D7,
            DtmfDigit::D8,
            DtmfDigit::D9,
            DtmfDigit::Star,
            DtmfDigit::Hash,
            DtmfDigit::A,
            DtmfDigit::B,
            DtmfDigit::C,
            DtmfDigit::D,
        ]
    }
}

impl std::fmt::Display for DtmfDigit {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_char())
    }
}

/// A detected DTMF event.
#[derive(Debug, Clone, PartialEq)]
pub struct DtmfEvent {
    /// The detected digit.
    pub digit: DtmfDigit,
    /// Sample index where the tone starts.
    pub start_sample: usize,
    /// Duration of the tone in samples.
    pub duration_samples: usize,
    /// Detected low-group frequency (Hz).
    pub low_freq_hz: f64,
    /// Detected high-group frequency (Hz).
    pub high_freq_hz: f64,
    /// Combined power in dB.
    pub power_db: f64,
}

/// Configuration for the DTMF detector.
#[derive(Debug, Clone)]
pub struct DtmfConfig {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Minimum tone duration in milliseconds (default: 40).
    pub min_duration_ms: f64,
    /// Maximum allowed twist (|P_high - P_low|) in dB (default: 8.0).
    pub twist_db_max: f64,
    /// Minimum SNR threshold in dB above noise floor (default: 10.0).
    pub snr_threshold_db: f64,
}

impl Default for DtmfConfig {
    fn default() -> Self {
        Self {
            sample_rate: 8000.0,
            min_duration_ms: 40.0,
            twist_db_max: 8.0,
            snr_threshold_db: 10.0,
        }
    }
}

/// Low-group DTMF frequencies (Hz).
const LOW_FREQS: [f64; 4] = [697.0, 770.0, 852.0, 941.0];

/// High-group DTMF frequencies (Hz).
const HIGH_FREQS: [f64; 4] = [1209.0, 1336.0, 1477.0, 1633.0];

/// DTMF digit lookup table indexed by [low_index][high_index].
const DIGIT_TABLE: [[DtmfDigit; 4]; 4] = [
    [DtmfDigit::D1, DtmfDigit::D2, DtmfDigit::D3, DtmfDigit::A],
    [DtmfDigit::D4, DtmfDigit::D5, DtmfDigit::D6, DtmfDigit::B],
    [DtmfDigit::D7, DtmfDigit::D8, DtmfDigit::D9, DtmfDigit::C],
    [DtmfDigit::Star, DtmfDigit::D0, DtmfDigit::Hash, DtmfDigit::D],
];

/// Compute the Goertzel power for a specific frequency on a block of samples.
///
/// Returns the power (magnitude squared) at the target frequency.
fn goertzel_power(samples: &[f64], target_freq: f64, sample_rate: f64) -> f64 {
    let n = samples.len();
    if n == 0 {
        return 0.0;
    }
    let k = (0.5 + (n as f64 * target_freq / sample_rate)).floor();
    let w = 2.0 * PI * k / n as f64;
    let coeff = 2.0 * w.cos();

    let mut s0 = 0.0;
    let mut s1 = 0.0;
    let mut s2;

    for &sample in samples {
        s2 = s1;
        s1 = s0;
        s0 = sample + coeff * s1 - s2;
    }

    // Power = s0^2 + s1^2 - coeff*s0*s1, normalized by N^2
    let power = s0 * s0 + s1 * s1 - coeff * s0 * s1;
    power / (n as f64 * n as f64)
}

/// Convert linear power to dB.
fn power_to_db(power: f64) -> f64 {
    if power <= 0.0 {
        -120.0
    } else {
        10.0 * power.log10()
    }
}

/// Goertzel-based DTMF tone detector.
pub struct DtmfDetector {
    config: DtmfConfig,
    block_size: usize,
}

impl DtmfDetector {
    /// Create a new DTMF detector with the given configuration.
    pub fn new(config: DtmfConfig) -> Self {
        // Block size based on minimum duration
        let block_size =
            (config.sample_rate * config.min_duration_ms / 1000.0).ceil() as usize;
        Self { config, block_size }
    }

    /// Detect all DTMF events in an audio buffer.
    ///
    /// The audio is processed in non-overlapping blocks of `min_duration_ms` length.
    /// Adjacent blocks detecting the same digit are merged into a single event.
    pub fn detect(&self, audio: &[f64]) -> Vec<DtmfEvent> {
        if audio.len() < self.block_size {
            // If the entire buffer is shorter than one block, try detecting on what we have
            if let Some(digit) = self.detect_digit(audio) {
                let (low, high) = digit.frequencies();
                let low_power = goertzel_power(audio, low, self.config.sample_rate);
                let high_power = goertzel_power(audio, high, self.config.sample_rate);
                let combined = power_to_db(low_power + high_power);
                return vec![DtmfEvent {
                    digit,
                    start_sample: 0,
                    duration_samples: audio.len(),
                    low_freq_hz: low,
                    high_freq_hz: high,
                    power_db: combined,
                }];
            }
            return Vec::new();
        }

        let mut events: Vec<DtmfEvent> = Vec::new();
        let mut offset = 0;

        while offset + self.block_size <= audio.len() {
            let block = &audio[offset..offset + self.block_size];
            if let Some(digit) = self.detect_digit(block) {
                let (low, high) = digit.frequencies();
                let low_power = goertzel_power(block, low, self.config.sample_rate);
                let high_power = goertzel_power(block, high, self.config.sample_rate);
                let combined = power_to_db(low_power + high_power);

                // Merge with previous event if same digit and contiguous
                if let Some(last) = events.last_mut() {
                    if last.digit == digit
                        && last.start_sample + last.duration_samples == offset
                    {
                        last.duration_samples += self.block_size;
                        // Update power to max seen
                        if combined > last.power_db {
                            last.power_db = combined;
                        }
                        offset += self.block_size;
                        continue;
                    }
                }

                events.push(DtmfEvent {
                    digit,
                    start_sample: offset,
                    duration_samples: self.block_size,
                    low_freq_hz: low,
                    high_freq_hz: high,
                    power_db: combined,
                });
            }
            offset += self.block_size;
        }

        events
    }

    /// Detect a single DTMF digit in a short audio block.
    ///
    /// Returns `None` if no valid DTMF digit is found (failed twist check,
    /// below SNR threshold, or no clear tone pair).
    pub fn detect_digit(&self, audio: &[f64]) -> Option<DtmfDigit> {
        if audio.is_empty() {
            return None;
        }

        let sr = self.config.sample_rate;

        // Compute Goertzel power for all 8 DTMF frequencies
        let low_powers: Vec<f64> = LOW_FREQS.iter().map(|&f| goertzel_power(audio, f, sr)).collect();
        let high_powers: Vec<f64> = HIGH_FREQS.iter().map(|&f| goertzel_power(audio, f, sr)).collect();

        // Find strongest low and high frequencies
        let (low_idx, &low_max) = low_powers
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())?;
        let (high_idx, &high_max) = high_powers
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())?;

        if low_max <= 0.0 || high_max <= 0.0 {
            return None;
        }

        // Twist check: difference between high and low powers
        let low_db = power_to_db(low_max);
        let high_db = power_to_db(high_max);
        let twist = (high_db - low_db).abs();
        if twist > self.config.twist_db_max {
            return None;
        }

        // SNR check: dominant tones should be significantly above the other tones
        let total_power: f64 = low_powers.iter().sum::<f64>() + high_powers.iter().sum::<f64>();
        let signal_power = low_max + high_max;
        let noise_power = total_power - signal_power;

        if noise_power > 0.0 {
            let snr_db = power_to_db(signal_power) - power_to_db(noise_power);
            if snr_db < self.config.snr_threshold_db {
                return None;
            }
        }

        // Ensure the strongest low tone is dominant among low tones
        let second_low = low_powers
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != low_idx)
            .map(|(_, &v)| v)
            .fold(0.0_f64, f64::max);
        if second_low > 0.0 && power_to_db(low_max) - power_to_db(second_low) < 6.0 {
            return None;
        }

        // Ensure the strongest high tone is dominant among high tones
        let second_high = high_powers
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != high_idx)
            .map(|(_, &v)| v)
            .fold(0.0_f64, f64::max);
        if second_high > 0.0 && power_to_db(high_max) - power_to_db(second_high) < 6.0 {
            return None;
        }

        Some(DIGIT_TABLE[low_idx][high_idx])
    }
}

/// DTMF tone generator.
pub struct DtmfGenerator {
    sample_rate: f64,
}

impl DtmfGenerator {
    /// Create a new DTMF generator at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        Self { sample_rate }
    }

    /// Generate a DTMF tone for a single digit.
    ///
    /// The output is the sum of two sinusoids at the low and high group frequencies,
    /// each at amplitude 0.5 (so the peak amplitude is 1.0).
    pub fn generate(&self, digit: DtmfDigit, duration_ms: f64) -> Vec<f64> {
        let (low_freq, high_freq) = digit.frequencies();
        let num_samples = (self.sample_rate * duration_ms / 1000.0).round() as usize;
        let mut output = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let t = i as f64 / self.sample_rate;
            let sample =
                0.5 * (2.0 * PI * low_freq * t).sin() + 0.5 * (2.0 * PI * high_freq * t).sin();
            output.push(sample);
        }

        output
    }

    /// Generate a sequence of DTMF tones with gaps between them.
    ///
    /// Each tone lasts `tone_ms` milliseconds, followed by `gap_ms` milliseconds of silence.
    pub fn generate_sequence(
        &self,
        digits: &[DtmfDigit],
        tone_ms: f64,
        gap_ms: f64,
    ) -> Vec<f64> {
        let gap_samples = (self.sample_rate * gap_ms / 1000.0).round() as usize;
        let mut output = Vec::new();

        for (i, &digit) in digits.iter().enumerate() {
            output.extend(self.generate(digit, tone_ms));
            if i < digits.len() - 1 {
                output.extend(std::iter::repeat(0.0).take(gap_samples));
            }
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> DtmfConfig {
        DtmfConfig {
            sample_rate: 8000.0,
            min_duration_ms: 40.0,
            twist_db_max: 8.0,
            snr_threshold_db: 10.0,
        }
    }

    #[test]
    fn test_digit_frequencies() {
        assert_eq!(DtmfDigit::D1.frequencies(), (697.0, 1209.0));
        assert_eq!(DtmfDigit::D0.frequencies(), (941.0, 1336.0));
        assert_eq!(DtmfDigit::Star.frequencies(), (941.0, 1209.0));
        assert_eq!(DtmfDigit::Hash.frequencies(), (941.0, 1477.0));
        assert_eq!(DtmfDigit::A.frequencies(), (697.0, 1633.0));
        assert_eq!(DtmfDigit::D.frequencies(), (941.0, 1633.0));
    }

    #[test]
    fn test_digit_to_char() {
        assert_eq!(DtmfDigit::D0.to_char(), '0');
        assert_eq!(DtmfDigit::D9.to_char(), '9');
        assert_eq!(DtmfDigit::Star.to_char(), '*');
        assert_eq!(DtmfDigit::Hash.to_char(), '#');
        assert_eq!(DtmfDigit::A.to_char(), 'A');
    }

    #[test]
    fn test_digit_from_char() {
        assert_eq!(DtmfDigit::from_char('5'), Some(DtmfDigit::D5));
        assert_eq!(DtmfDigit::from_char('*'), Some(DtmfDigit::Star));
        assert_eq!(DtmfDigit::from_char('#'), Some(DtmfDigit::Hash));
        assert_eq!(DtmfDigit::from_char('a'), Some(DtmfDigit::A));
        assert_eq!(DtmfDigit::from_char('D'), Some(DtmfDigit::D));
        assert_eq!(DtmfDigit::from_char('x'), None);
    }

    #[test]
    fn test_digit_display() {
        assert_eq!(format!("{}", DtmfDigit::D7), "7");
        assert_eq!(format!("{}", DtmfDigit::Star), "*");
    }

    #[test]
    fn test_all_digits_count() {
        assert_eq!(DtmfDigit::all().len(), 16);
    }

    #[test]
    fn test_generator_basic() {
        let gen = DtmfGenerator::new(8000.0);
        let tone = gen.generate(DtmfDigit::D5, 100.0);
        // 8000 * 0.1 = 800 samples
        assert_eq!(tone.len(), 800);
    }

    #[test]
    fn test_generator_amplitude() {
        let gen = DtmfGenerator::new(8000.0);
        let tone = gen.generate(DtmfDigit::D1, 50.0);
        // Peak amplitude should not exceed 1.0 (0.5 + 0.5)
        let max_amp = tone.iter().map(|s| s.abs()).fold(0.0_f64, f64::max);
        assert!(max_amp <= 1.01, "Max amplitude {} exceeds 1.0", max_amp);
        assert!(max_amp > 0.5, "Max amplitude {} too low", max_amp);
    }

    #[test]
    fn test_generator_sequence() {
        let gen = DtmfGenerator::new(8000.0);
        let digits = [DtmfDigit::D1, DtmfDigit::D2, DtmfDigit::D3];
        let seq = gen.generate_sequence(&digits, 100.0, 50.0);
        // 3 tones * 800 samples + 2 gaps * 400 samples = 3200
        assert_eq!(seq.len(), 3200);
    }

    #[test]
    fn test_detect_single_digit() {
        let gen = DtmfGenerator::new(8000.0);
        let config = default_config();
        let detector = DtmfDetector::new(config);

        let tone = gen.generate(DtmfDigit::D5, 100.0);
        let digit = detector.detect_digit(&tone);
        assert_eq!(digit, Some(DtmfDigit::D5));
    }

    #[test]
    fn test_detect_all_16_digits() {
        let gen = DtmfGenerator::new(8000.0);
        let config = default_config();
        let detector = DtmfDetector::new(config);

        for digit in DtmfDigit::all() {
            let tone = gen.generate(digit, 80.0);
            let detected = detector.detect_digit(&tone);
            assert_eq!(
                detected,
                Some(digit),
                "Failed to detect digit {}",
                digit.to_char()
            );
        }
    }

    #[test]
    fn test_detect_events() {
        let gen = DtmfGenerator::new(8000.0);
        let config = default_config();
        let detector = DtmfDetector::new(config);

        let tone = gen.generate(DtmfDigit::D9, 120.0);
        let events = detector.detect(&tone);
        assert!(!events.is_empty(), "Should detect at least one event");
        assert_eq!(events[0].digit, DtmfDigit::D9);
        assert_eq!(events[0].low_freq_hz, 852.0);
        assert_eq!(events[0].high_freq_hz, 1477.0);
    }

    #[test]
    fn test_detect_sequence() {
        let gen = DtmfGenerator::new(8000.0);
        let config = default_config();
        let detector = DtmfDetector::new(config);

        let digits = [DtmfDigit::D1, DtmfDigit::D2, DtmfDigit::D3];
        let seq = gen.generate_sequence(&digits, 100.0, 100.0);
        let events = detector.detect(&seq);

        assert_eq!(events.len(), 3, "Expected 3 events, got {}", events.len());
        assert_eq!(events[0].digit, DtmfDigit::D1);
        assert_eq!(events[1].digit, DtmfDigit::D2);
        assert_eq!(events[2].digit, DtmfDigit::D3);
    }

    #[test]
    fn test_silence_no_detection() {
        let config = default_config();
        let detector = DtmfDetector::new(config);

        let silence = vec![0.0; 800];
        let digit = detector.detect_digit(&silence);
        assert_eq!(digit, None, "Should not detect a digit in silence");
    }

    #[test]
    fn test_noise_no_detection() {
        let config = default_config();
        let detector = DtmfDetector::new(config);

        // Pseudo-random noise using a simple LCG
        let mut rng_state: u64 = 12345;
        let noise: Vec<f64> = (0..800)
            .map(|_| {
                rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
                // Map to [-0.1, 0.1] (low amplitude noise)
                ((rng_state >> 33) as f64 / u32::MAX as f64 - 0.5) * 0.2
            })
            .collect();

        let digit = detector.detect_digit(&noise);
        assert_eq!(digit, None, "Should not detect a digit in noise");
    }

    #[test]
    fn test_goertzel_power_basic() {
        let sample_rate = 8000.0;
        let freq = 1000.0;
        let n = 320;
        let samples: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect();

        let power_at_freq = goertzel_power(&samples, freq, sample_rate);
        let power_off_freq = goertzel_power(&samples, 2000.0, sample_rate);

        assert!(
            power_at_freq > power_off_freq * 100.0,
            "On-frequency power ({}) should dominate off-frequency ({})",
            power_at_freq,
            power_off_freq
        );
    }

    #[test]
    fn test_twist_rejection() {
        // Generate a tone with very unbalanced power (one tone 20 dB louder)
        let sample_rate = 8000.0;
        let n = 320;
        let low_freq = 697.0;
        let high_freq = 1209.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                // Low tone at full amplitude, high tone at -20 dB (0.1x)
                0.5 * (2.0 * PI * low_freq * t).sin()
                    + 0.05 * (2.0 * PI * high_freq * t).sin()
            })
            .collect();

        let config = DtmfConfig {
            sample_rate,
            twist_db_max: 8.0,
            snr_threshold_db: 3.0,
            ..default_config()
        };
        let detector = DtmfDetector::new(config);
        let digit = detector.detect_digit(&samples);
        assert_eq!(digit, None, "Should reject due to twist");
    }

    #[test]
    fn test_event_start_sample() {
        let gen = DtmfGenerator::new(8000.0);
        let config = default_config();
        let detector = DtmfDetector::new(config);

        let tone = gen.generate(DtmfDigit::D7, 200.0);
        let events = detector.detect(&tone);
        assert!(!events.is_empty());
        assert_eq!(events[0].start_sample, 0);
    }

    #[test]
    fn test_config_default() {
        let config = DtmfConfig::default();
        assert_eq!(config.sample_rate, 8000.0);
        assert_eq!(config.min_duration_ms, 40.0);
        assert_eq!(config.twist_db_max, 8.0);
        assert_eq!(config.snr_threshold_db, 10.0);
    }

    #[test]
    fn test_empty_audio() {
        let config = default_config();
        let detector = DtmfDetector::new(config);
        let events = detector.detect(&[]);
        assert!(events.is_empty());
        assert_eq!(detector.detect_digit(&[]), None);
    }

    #[test]
    fn test_higher_sample_rate() {
        let sr = 44100.0;
        let gen = DtmfGenerator::new(sr);
        let config = DtmfConfig {
            sample_rate: sr,
            min_duration_ms: 40.0,
            twist_db_max: 8.0,
            snr_threshold_db: 10.0,
        };
        let detector = DtmfDetector::new(config);

        for digit in DtmfDigit::all() {
            let tone = gen.generate(digit, 80.0);
            let detected = detector.detect_digit(&tone);
            assert_eq!(
                detected,
                Some(digit),
                "Failed at 44100 Hz for digit {}",
                digit.to_char()
            );
        }
    }
}
