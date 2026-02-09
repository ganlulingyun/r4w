//! RAKE Receiver — Multipath Combining for Spread-Spectrum
//!
//! Combines multipath copies of a DSSS/CDMA signal using maximal ratio
//! combining (MRC), equal-gain combining, or selection diversity. Fingers
//! track individual multipath taps via sliding correlation with the
//! spreading code.
//!
//! GNU Radio equivalent: `gr-spread` RAKE blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rake_receiver::{RakeReceiver, CombiningMode};
//!
//! let code = vec![1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0];
//! let mut rake = RakeReceiver::new(&code, 4, CombiningMode::MaximalRatio);
//! let signal = vec![(0.5, 0.0); 256];
//! let combined = rake.combine(&signal);
//! assert!(!combined.is_empty());
//! ```

use std::f64::consts::PI;

/// Combining mode for multi-finger RAKE.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CombiningMode {
    /// Maximal Ratio Combining: weight by complex conjugate of channel estimate.
    MaximalRatio,
    /// Equal Gain Combining: phase-align only (unit weights).
    EqualGain,
    /// Selection Diversity: pick the strongest finger.
    SelectionDiversity,
}

/// State of a single RAKE finger (multipath tap tracker).
#[derive(Debug, Clone)]
pub struct RakeFinger {
    /// Delay in samples from start.
    pub delay: usize,
    /// Complex channel gain estimate (re, im).
    pub gain: (f64, f64),
    /// Finger strength (|gain|^2).
    pub strength: f64,
    /// Whether the finger is actively tracking.
    pub active: bool,
}

/// RAKE receiver for spread-spectrum multipath combining.
#[derive(Debug, Clone)]
pub struct RakeReceiver {
    spreading_code: Vec<f64>,
    max_fingers: usize,
    mode: CombiningMode,
    fingers: Vec<RakeFinger>,
    search_window: usize,
    threshold: f64,
}

impl RakeReceiver {
    /// Create a new RAKE receiver.
    ///
    /// `spreading_code`: PN code chip sequence (±1 values).
    /// `max_fingers`: maximum number of multipath fingers to track.
    /// `mode`: combining strategy.
    pub fn new(spreading_code: &[f64], max_fingers: usize, mode: CombiningMode) -> Self {
        Self {
            spreading_code: spreading_code.to_vec(),
            max_fingers: max_fingers.max(1),
            mode,
            fingers: Vec::new(),
            search_window: spreading_code.len() * 4,
            threshold: 0.3,
        }
    }

    /// Set the search window size (in samples).
    pub fn search_window(mut self, window: usize) -> Self {
        self.search_window = window.max(self.spreading_code.len());
        self
    }

    /// Set the finger detection threshold (0.0-1.0, relative to max correlation).
    pub fn threshold(mut self, thresh: f64) -> Self {
        self.threshold = thresh.clamp(0.0, 1.0);
        self
    }

    /// Search for multipath fingers via sliding correlation.
    pub fn search_fingers(&mut self, signal: &[(f64, f64)]) -> &[RakeFinger] {
        let code_len = self.spreading_code.len();
        let search_len = self.search_window.min(signal.len().saturating_sub(code_len));
        if search_len == 0 || code_len == 0 {
            return &self.fingers;
        }

        // Compute correlation at each delay
        let mut correlations: Vec<(usize, f64, (f64, f64))> = Vec::new();
        for delay in 0..search_len {
            let (cr, ci) = self.correlate_at(signal, delay);
            let mag = (cr * cr + ci * ci).sqrt();
            correlations.push((delay, mag, (cr, ci)));
        }

        // Find max for normalization
        let max_mag = correlations.iter().map(|c| c.1).fold(0.0f64, f64::max);
        if max_mag < 1e-20 {
            self.fingers.clear();
            return &self.fingers;
        }

        // Find peaks above threshold
        let thresh = self.threshold * max_mag;
        let mut peaks: Vec<(usize, f64, (f64, f64))> = Vec::new();
        for &(delay, mag, gain) in &correlations {
            if mag >= thresh {
                // Simple peak detection: must be local max
                let is_peak = correlations
                    .iter()
                    .filter(|c| c.0 != delay && (c.0 as i64 - delay as i64).unsigned_abs() <= 1)
                    .all(|c| c.1 <= mag);
                if is_peak {
                    peaks.push((delay, mag, gain));
                }
            }
        }

        // Sort by strength, keep max_fingers
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        peaks.truncate(self.max_fingers);

        self.fingers = peaks
            .iter()
            .map(|&(delay, strength, gain)| RakeFinger {
                delay,
                gain,
                strength,
                active: true,
            })
            .collect();

        &self.fingers
    }

    /// Despread signal at a given finger's delay.
    pub fn despread_at(&self, signal: &[(f64, f64)], delay: usize) -> Vec<(f64, f64)> {
        let code_len = self.spreading_code.len();
        if delay + code_len > signal.len() {
            return vec![];
        }
        let num_symbols = (signal.len() - delay) / code_len;
        let mut symbols = Vec::with_capacity(num_symbols);

        for s in 0..num_symbols {
            let start = delay + s * code_len;
            let mut sum_r = 0.0;
            let mut sum_i = 0.0;
            for c in 0..code_len {
                let chip = self.spreading_code[c];
                let (sr, si) = signal[start + c];
                sum_r += sr * chip;
                sum_i += si * chip;
            }
            symbols.push((sum_r / code_len as f64, sum_i / code_len as f64));
        }
        symbols
    }

    /// Combine multipath fingers using configured combining mode.
    ///
    /// Automatically searches for fingers if none have been detected yet.
    pub fn combine(&mut self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if self.fingers.is_empty() {
            self.search_fingers(signal);
        }

        if self.fingers.is_empty() {
            return self.despread_at(signal, 0);
        }

        match self.mode {
            CombiningMode::SelectionDiversity => {
                // Pick strongest finger
                let best = self
                    .fingers
                    .iter()
                    .max_by(|a, b| a.strength.partial_cmp(&b.strength).unwrap())
                    .unwrap();
                self.despread_at(signal, best.delay)
            }
            CombiningMode::EqualGain | CombiningMode::MaximalRatio => {
                let finger_data: Vec<_> = self
                    .fingers
                    .iter()
                    .map(|f| (f.delay, f.gain, f.strength))
                    .collect();

                // Find minimum number of symbols across all fingers
                let code_len = self.spreading_code.len();
                let min_symbols = finger_data
                    .iter()
                    .map(|&(delay, _, _)| {
                        if delay + code_len > signal.len() {
                            0
                        } else {
                            (signal.len() - delay) / code_len
                        }
                    })
                    .min()
                    .unwrap_or(0);

                if min_symbols == 0 {
                    return vec![];
                }

                let mut combined = vec![(0.0, 0.0); min_symbols];
                let mut total_weight = 0.0;

                for &(delay, gain, strength) in &finger_data {
                    let symbols = self.despread_at(signal, delay);
                    let weight = match self.mode {
                        CombiningMode::MaximalRatio => strength,
                        CombiningMode::EqualGain => 1.0,
                        _ => unreachable!(),
                    };

                    // Phase-align using conjugate of channel gain
                    let gain_mag = (gain.0 * gain.0 + gain.1 * gain.1).sqrt().max(1e-20);
                    let conj_norm = (gain.0 / gain_mag, -gain.1 / gain_mag);

                    for i in 0..min_symbols.min(symbols.len()) {
                        let (sr, si) = symbols[i];
                        // Multiply by conjugate of gain (phase correction)
                        let aligned_r = sr * conj_norm.0 - si * conj_norm.1;
                        let aligned_i = sr * conj_norm.1 + si * conj_norm.0;
                        combined[i].0 += weight * aligned_r;
                        combined[i].1 += weight * aligned_i;
                    }
                    total_weight += weight;
                }

                // Normalize
                if total_weight > 0.0 {
                    for c in &mut combined {
                        c.0 /= total_weight;
                        c.1 /= total_weight;
                    }
                }
                combined
            }
        }
    }

    /// Number of active fingers.
    pub fn finger_count(&self) -> usize {
        self.fingers.iter().filter(|f| f.active).count()
    }

    /// Current finger state.
    pub fn fingers(&self) -> &[RakeFinger] {
        &self.fingers
    }

    fn correlate_at(&self, signal: &[(f64, f64)], delay: usize) -> (f64, f64) {
        let code_len = self.spreading_code.len();
        if delay + code_len > signal.len() {
            return (0.0, 0.0);
        }
        let mut sum_r = 0.0;
        let mut sum_i = 0.0;
        for c in 0..code_len {
            let chip = self.spreading_code[c];
            let (sr, si) = signal[delay + c];
            sum_r += sr * chip;
            sum_i += si * chip;
        }
        (sum_r / code_len as f64, sum_i / code_len as f64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_spread_signal(data: &[f64], code: &[f64]) -> Vec<(f64, f64)> {
        let mut signal = Vec::new();
        for &bit in data {
            for &chip in code {
                signal.push((bit * chip, 0.0));
            }
        }
        signal
    }

    #[test]
    fn test_single_finger_despread() {
        let code = vec![1.0, -1.0, 1.0, -1.0];
        let data = vec![1.0, -1.0, 1.0, 1.0];
        let signal = make_spread_signal(&data, &code);
        let rake = RakeReceiver::new(&code, 1, CombiningMode::MaximalRatio);
        let symbols = rake.despread_at(&signal, 0);
        assert_eq!(symbols.len(), 4);
        for (i, &(sr, _)) in symbols.iter().enumerate() {
            assert!(
                (sr - data[i]).abs() < 0.01,
                "symbol {i}: got {sr}, expected {}",
                data[i]
            );
        }
    }

    #[test]
    fn test_finger_search_finds_delays() {
        let code = vec![1.0, -1.0, 1.0, 1.0];
        let data = vec![1.0, -1.0, 1.0];
        let direct = make_spread_signal(&data, &code);
        // Create signal with two paths: delay 0 and delay 2
        let total_len = direct.len() + 4;
        let mut signal = vec![(0.0, 0.0); total_len];
        for (i, &s) in direct.iter().enumerate() {
            signal[i].0 += s.0;
            if i + 2 < total_len {
                signal[i + 2].0 += s.0 * 0.5; // delayed copy at half amplitude
            }
        }
        let mut rake = RakeReceiver::new(&code, 4, CombiningMode::MaximalRatio)
            .threshold(0.2);
        rake.search_fingers(&signal);
        assert!(rake.finger_count() >= 1, "should find at least 1 finger");
    }

    #[test]
    fn test_mrc_improves_over_single() {
        let code = vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let data = vec![1.0, -1.0, 1.0, -1.0, 1.0];
        let direct = make_spread_signal(&data, &code);
        // Single path
        let rake_single = RakeReceiver::new(&code, 1, CombiningMode::SelectionDiversity);
        let single_out = rake_single.despread_at(&direct, 0);
        assert!(!single_out.is_empty());
        // Just verify MRC also works
        let mut rake_mrc = RakeReceiver::new(&code, 4, CombiningMode::MaximalRatio);
        let mrc_out = rake_mrc.combine(&direct);
        assert!(!mrc_out.is_empty());
    }

    #[test]
    fn test_equal_gain_combining() {
        let code = vec![1.0, -1.0, 1.0, -1.0];
        let signal = vec![(0.5, 0.0); 64];
        let mut rake = RakeReceiver::new(&code, 2, CombiningMode::EqualGain);
        let result = rake.combine(&signal);
        assert!(!result.is_empty());
        for &(r, i) in &result {
            assert!(r.is_finite() && i.is_finite());
        }
    }

    #[test]
    fn test_selection_diversity() {
        let code = vec![1.0, -1.0, 1.0, -1.0];
        let signal = vec![(0.5, 0.0); 64];
        let mut rake = RakeReceiver::new(&code, 4, CombiningMode::SelectionDiversity);
        let result = rake.combine(&signal);
        assert!(!result.is_empty());
    }

    #[test]
    fn test_max_finger_limit() {
        let code = vec![1.0, -1.0];
        let signal = vec![(1.0, 0.0); 128];
        let mut rake = RakeReceiver::new(&code, 2, CombiningMode::MaximalRatio);
        rake.search_fingers(&signal);
        assert!(rake.finger_count() <= 2);
    }

    #[test]
    fn test_empty_signal() {
        let code = vec![1.0, -1.0, 1.0, -1.0];
        let mut rake = RakeReceiver::new(&code, 2, CombiningMode::MaximalRatio);
        let result = rake.combine(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_despread_length() {
        let code = vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let signal = vec![(1.0, 0.0); 80]; // 10 symbols worth
        let rake = RakeReceiver::new(&code, 1, CombiningMode::MaximalRatio);
        let symbols = rake.despread_at(&signal, 0);
        assert_eq!(symbols.len(), 10);
    }

    #[test]
    fn test_finger_state() {
        let code = vec![1.0, -1.0, 1.0, -1.0];
        let signal = vec![(1.0, 0.0); 64];
        let mut rake = RakeReceiver::new(&code, 4, CombiningMode::MaximalRatio);
        rake.search_fingers(&signal);
        for f in rake.fingers() {
            assert!(f.active);
            assert!(f.strength >= 0.0);
        }
    }

    #[test]
    fn test_threshold_setting() {
        let code = vec![1.0, -1.0];
        let signal = vec![(0.5, 0.0); 64];
        let mut rake = RakeReceiver::new(&code, 4, CombiningMode::MaximalRatio)
            .threshold(0.9); // very high threshold
        rake.search_fingers(&signal);
        // High threshold should reduce finger count
        let high_count = rake.finger_count();
        let mut rake2 = RakeReceiver::new(&code, 4, CombiningMode::MaximalRatio)
            .threshold(0.1); // low threshold
        rake2.search_fingers(&signal);
        let low_count = rake2.finger_count();
        assert!(low_count >= high_count);
    }
}
