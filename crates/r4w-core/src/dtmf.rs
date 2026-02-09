//! DTMF Decoder — Dual-Tone Multi-Frequency detection
//!
//! Detects DTMF tones used in telephone signaling by analyzing pairs of
//! frequencies from the 4x4 tone grid. Uses a bank of Goertzel filters
//! for efficient single-frequency detection.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dtmf::DtmfDecoder;
//!
//! let mut decoder = DtmfDecoder::new(8000.0, 205);
//! // Generate DTMF '5' (770 Hz + 1336 Hz)
//! let samples: Vec<f64> = (0..205)
//!     .map(|i| {
//!         let t = i as f64 / 8000.0;
//!         (2.0 * std::f64::consts::PI * 770.0 * t).sin()
//!             + (2.0 * std::f64::consts::PI * 1336.0 * t).sin()
//!     })
//!     .collect();
//! let result = decoder.detect(&samples);
//! assert_eq!(result, Some('5'));
//! ```

use std::f64::consts::PI;

/// DTMF frequency rows (low group).
const DTMF_ROWS: [f64; 4] = [697.0, 770.0, 852.0, 941.0];

/// DTMF frequency columns (high group).
const DTMF_COLS: [f64; 4] = [1209.0, 1336.0, 1477.0, 1633.0];

/// DTMF key mapping: rows × columns.
const DTMF_KEYS: [[char; 4]; 4] = [
    ['1', '2', '3', 'A'],
    ['4', '5', '6', 'B'],
    ['7', '8', '9', 'C'],
    ['*', '0', '#', 'D'],
];

/// Goertzel filter state for a single frequency.
#[derive(Debug, Clone)]
struct GoertzelState {
    coeff: f64,
    s1: f64,
    s2: f64,
    freq_hz: f64,
}

impl GoertzelState {
    fn new(freq_hz: f64, sample_rate: f64, block_size: usize) -> Self {
        let k = (0.5 + block_size as f64 * freq_hz / sample_rate).floor();
        let omega = 2.0 * PI * k / block_size as f64;
        Self {
            coeff: 2.0 * omega.cos(),
            s1: 0.0,
            s2: 0.0,
            freq_hz,
        }
    }

    fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }

    fn process_sample(&mut self, sample: f64) {
        let s0 = sample + self.coeff * self.s1 - self.s2;
        self.s2 = self.s1;
        self.s1 = s0;
    }

    fn magnitude_squared(&self) -> f64 {
        self.s1 * self.s1 + self.s2 * self.s2 - self.coeff * self.s1 * self.s2
    }
}

/// DTMF decoder using Goertzel filters.
#[derive(Debug, Clone)]
pub struct DtmfDecoder {
    sample_rate: f64,
    block_size: usize,
    /// Goertzel filters for 4 row + 4 column frequencies.
    row_filters: Vec<GoertzelState>,
    col_filters: Vec<GoertzelState>,
    /// Minimum relative threshold for tone detection.
    threshold: f64,
    /// Twist limit: max difference between row and col magnitudes (dB).
    max_twist_db: f64,
}

impl DtmfDecoder {
    /// Create a DTMF decoder.
    ///
    /// `sample_rate`: Input sample rate in Hz (typically 8000).
    /// `block_size`: Number of samples per detection frame (typically 205 for 8kHz = ~25ms).
    pub fn new(sample_rate: f64, block_size: usize) -> Self {
        let row_filters = DTMF_ROWS
            .iter()
            .map(|&f| GoertzelState::new(f, sample_rate, block_size))
            .collect();
        let col_filters = DTMF_COLS
            .iter()
            .map(|&f| GoertzelState::new(f, sample_rate, block_size))
            .collect();
        Self {
            sample_rate,
            block_size,
            row_filters,
            col_filters,
            threshold: 0.1,
            max_twist_db: 8.0,
        }
    }

    /// Set the detection threshold (relative to total energy).
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }

    /// Detect a DTMF character from a block of samples.
    ///
    /// Returns `Some(char)` if a valid DTMF tone pair is detected.
    pub fn detect(&mut self, samples: &[f64]) -> Option<char> {
        // Reset all filters
        for f in &mut self.row_filters {
            f.reset();
        }
        for f in &mut self.col_filters {
            f.reset();
        }

        // Process samples through all Goertzel filters
        let len = samples.len().min(self.block_size);
        for &s in &samples[..len] {
            for f in &mut self.row_filters {
                f.process_sample(s);
            }
            for f in &mut self.col_filters {
                f.process_sample(s);
            }
        }

        // Get magnitudes
        let row_mags: Vec<f64> = self.row_filters.iter().map(|f| f.magnitude_squared()).collect();
        let col_mags: Vec<f64> = self.col_filters.iter().map(|f| f.magnitude_squared()).collect();

        // Find strongest row and column
        let (row_idx, &row_max) = row_mags
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())?;
        let (col_idx, &col_max) = col_mags
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())?;

        // Check threshold
        let total_energy: f64 = row_mags.iter().sum::<f64>() + col_mags.iter().sum::<f64>();
        if total_energy < 1e-20 {
            return None;
        }
        let tone_energy = row_max + col_max;
        if tone_energy / total_energy < self.threshold {
            return None;
        }

        // Check twist (row/col magnitude balance)
        if row_max > 1e-20 && col_max > 1e-20 {
            let twist = 10.0 * (row_max / col_max).abs().log10();
            if twist.abs() > self.max_twist_db {
                return None;
            }
        }

        // Check that no other row/col is close to the max (at least 6dB below)
        for (i, &m) in row_mags.iter().enumerate() {
            if i != row_idx && m > row_max * 0.25 {
                return None; // Ambiguous row
            }
        }
        for (i, &m) in col_mags.iter().enumerate() {
            if i != col_idx && m > col_max * 0.25 {
                return None; // Ambiguous column
            }
        }

        Some(DTMF_KEYS[row_idx][col_idx])
    }

    /// Decode a continuous stream of samples into DTMF characters.
    pub fn decode_stream(&mut self, samples: &[f64]) -> Vec<char> {
        let mut result = Vec::new();
        let mut prev: Option<char> = None;

        for chunk in samples.chunks(self.block_size) {
            if chunk.len() < self.block_size / 2 {
                break; // Too short to detect
            }
            let detected = self.detect(chunk);
            // Only emit on new detection (debounce)
            if detected != prev && detected.is_some() {
                result.push(detected.unwrap());
            }
            prev = detected;
        }

        result
    }

    /// Generate DTMF tone for a character.
    pub fn generate(key: char, sample_rate: f64, duration_samples: usize) -> Option<Vec<f64>> {
        let (row, col) = Self::key_to_freqs(key)?;
        let samples: Vec<f64> = (0..duration_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                0.5 * (2.0 * PI * row * t).sin() + 0.5 * (2.0 * PI * col * t).sin()
            })
            .collect();
        Some(samples)
    }

    /// Map a DTMF key to its (row_freq, col_freq) pair.
    fn key_to_freqs(key: char) -> Option<(f64, f64)> {
        for (r, row) in DTMF_KEYS.iter().enumerate() {
            for (c, &k) in row.iter().enumerate() {
                if k == key {
                    return Some((DTMF_ROWS[r], DTMF_COLS[c]));
                }
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generate_tone(freq: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    fn generate_dtmf(row_freq: f64, col_freq: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * row_freq * t).sin() + (2.0 * PI * col_freq * t).sin()
            })
            .collect()
    }

    #[test]
    fn test_detect_digit_5() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = generate_dtmf(770.0, 1336.0, 8000.0, 205);
        assert_eq!(dec.detect(&samples), Some('5'));
    }

    #[test]
    fn test_detect_digit_1() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = generate_dtmf(697.0, 1209.0, 8000.0, 205);
        assert_eq!(dec.detect(&samples), Some('1'));
    }

    #[test]
    fn test_detect_star() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = generate_dtmf(941.0, 1209.0, 8000.0, 205);
        assert_eq!(dec.detect(&samples), Some('*'));
    }

    #[test]
    fn test_detect_hash() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = generate_dtmf(941.0, 1477.0, 8000.0, 205);
        assert_eq!(dec.detect(&samples), Some('#'));
    }

    #[test]
    fn test_detect_a() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = generate_dtmf(697.0, 1633.0, 8000.0, 205);
        assert_eq!(dec.detect(&samples), Some('A'));
    }

    #[test]
    fn test_no_dtmf_in_silence() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = vec![0.0; 205];
        assert_eq!(dec.detect(&samples), None);
    }

    #[test]
    fn test_no_dtmf_single_tone() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let samples = generate_tone(770.0, 8000.0, 205);
        // Single tone should not be detected as DTMF (threshold check)
        // The energy is concentrated in one group only
        assert_eq!(dec.detect(&samples), None);
    }

    #[test]
    fn test_generate_and_detect() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        for key in "0123456789*#ABCD".chars() {
            let samples = DtmfDecoder::generate(key, 8000.0, 205).unwrap();
            let detected = dec.detect(&samples);
            assert_eq!(detected, Some(key), "Failed to detect '{}'", key);
        }
    }

    #[test]
    fn test_decode_stream() {
        let mut dec = DtmfDecoder::new(8000.0, 205);
        let mut stream = Vec::new();
        // Generate "42"
        stream.extend(DtmfDecoder::generate('4', 8000.0, 205).unwrap());
        stream.extend(vec![0.0; 205]); // silence gap
        stream.extend(DtmfDecoder::generate('2', 8000.0, 205).unwrap());
        let decoded = dec.decode_stream(&stream);
        assert_eq!(decoded, vec!['4', '2']);
    }

    #[test]
    fn test_key_to_freqs() {
        assert_eq!(DtmfDecoder::key_to_freqs('5'), Some((770.0, 1336.0)));
        assert_eq!(DtmfDecoder::key_to_freqs('0'), Some((941.0, 1336.0)));
        assert_eq!(DtmfDecoder::key_to_freqs('Z'), None);
    }
}
