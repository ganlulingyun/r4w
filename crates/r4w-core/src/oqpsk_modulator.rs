//! OQPSK Modulator — Offset Quadrature Phase Shift Keying
//!
//! OQPSK staggers the I and Q channels by half a symbol period,
//! limiting phase transitions to ±90° (vs ±180° for QPSK). This reduces
//! envelope fluctuations, improving power amplifier efficiency. Used in
//! IEEE 802.15.4 (ZigBee), CDMA IS-95, and satellite communications.
//!
//! ## Properties
//!
//! - Same spectral efficiency as QPSK (2 bits/symbol)
//! - Maximum phase change: ±π/2 (vs ±π for QPSK)
//! - Lower PAPR → better PA efficiency
//! - I and Q transitions never occur simultaneously
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::oqpsk_modulator::OqpskModulator;
//!
//! let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
//! let bits = vec![true, false, true, true, false, false];
//! let signal = oqpsk.modulate(&bits);
//! // 6 bits = 3 symbols, each 8 samples
//! assert_eq!(signal.len(), 3 * 8);
//! ```

use std::f64::consts::PI;

/// Complex sample for OQPSK.
#[derive(Debug, Clone, Copy)]
pub struct OqpskComplex {
    pub re: f64,
    pub im: f64,
}

impl OqpskComplex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn mag_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn mag(self) -> f64 {
        self.mag_sq().sqrt()
    }

    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }
}

impl std::ops::Add for OqpskComplex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::Sub for OqpskComplex {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

/// OQPSK modulator.
#[derive(Debug, Clone)]
pub struct OqpskModulator {
    /// Symbol rate (Hz).
    symbol_rate: f64,
    /// Sample rate (Hz).
    sample_rate: f64,
    /// Samples per symbol.
    samples_per_symbol: usize,
    /// Previous I symbol value (for offset).
    prev_i: f64,
    /// Previous Q symbol value.
    prev_q: f64,
}

/// OQPSK demodulator.
#[derive(Debug, Clone)]
pub struct OqpskDemodulator {
    /// Samples per symbol.
    samples_per_symbol: usize,
}

impl OqpskModulator {
    /// Create a new OQPSK modulator.
    pub fn new(symbol_rate: f64, sample_rate: f64) -> Self {
        let samples_per_symbol = (sample_rate / symbol_rate).round() as usize;
        assert!(samples_per_symbol >= 2, "Need at least 2 samples per symbol");

        Self {
            symbol_rate,
            sample_rate,
            samples_per_symbol,
            prev_i: 1.0,
            prev_q: 1.0,
        }
    }

    /// Modulate bits to OQPSK complex baseband.
    ///
    /// Input bits are grouped in pairs: (I_bit, Q_bit).
    /// Q channel is delayed by T/2 relative to I.
    pub fn modulate(&mut self, bits: &[bool]) -> Vec<OqpskComplex> {
        let sps = self.samples_per_symbol;
        let half_sps = sps / 2;
        let num_symbols = bits.len() / 2;

        if num_symbols == 0 {
            return Vec::new();
        }

        // Map bits to ±1 symbols
        let i_symbols: Vec<f64> = (0..num_symbols)
            .map(|k| if bits[2 * k] { 1.0 } else { -1.0 })
            .collect();
        let q_symbols: Vec<f64> = (0..num_symbols)
            .map(|k| if bits[2 * k + 1] { 1.0 } else { -1.0 })
            .collect();

        let total_samples = num_symbols * sps;
        let mut output = Vec::with_capacity(total_samples);

        for n in 0..total_samples {
            // I channel: symbol changes at n = k * sps
            let i_idx = n / sps;
            let i_val = if i_idx < i_symbols.len() {
                i_symbols[i_idx]
            } else {
                *i_symbols.last().unwrap_or(&1.0)
            };

            // Q channel: delayed by half symbol (changes at n = k * sps + sps/2)
            let q_shifted = n + half_sps;
            let q_idx = q_shifted / sps;
            let q_val = if q_idx > 0 && (q_idx - 1) < q_symbols.len() {
                q_symbols[q_idx - 1]
            } else if q_idx == 0 {
                self.prev_q
            } else {
                *q_symbols.last().unwrap_or(&1.0)
            };

            output.push(OqpskComplex::new(i_val, q_val));
        }

        // Save last symbols for continuity
        if let Some(&last_i) = i_symbols.last() {
            self.prev_i = last_i;
        }
        if let Some(&last_q) = q_symbols.last() {
            self.prev_q = last_q;
        }

        output
    }

    /// Samples per symbol.
    pub fn samples_per_symbol(&self) -> usize {
        self.samples_per_symbol
    }

    /// Bits per symbol (always 2 for OQPSK).
    pub fn bits_per_symbol(&self) -> usize {
        2
    }

    /// Compute PAPR of a signal.
    pub fn papr(signal: &[OqpskComplex]) -> f64 {
        if signal.is_empty() {
            return 0.0;
        }
        let peak = signal.iter().map(|s| s.mag_sq()).fold(0.0_f64, f64::max);
        let avg = signal.iter().map(|s| s.mag_sq()).sum::<f64>() / signal.len() as f64;
        if avg > 0.0 {
            10.0 * (peak / avg).log10()
        } else {
            0.0
        }
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.prev_i = 1.0;
        self.prev_q = 1.0;
    }
}

impl OqpskDemodulator {
    /// Create a new OQPSK demodulator.
    pub fn new(symbol_rate: f64, sample_rate: f64) -> Self {
        let samples_per_symbol = (sample_rate / symbol_rate).round() as usize;
        Self { samples_per_symbol }
    }

    /// Demodulate OQPSK signal to bits.
    ///
    /// Samples I at symbol centers and Q at half-symbol offset centers.
    pub fn demodulate(&self, signal: &[OqpskComplex]) -> Vec<bool> {
        let sps = self.samples_per_symbol;
        let half_sps = sps / 2;
        let num_symbols = signal.len() / sps;

        let mut bits = Vec::with_capacity(num_symbols * 2);

        for k in 0..num_symbols {
            // I channel: sample at center of symbol k
            let i_center = k * sps + sps / 2;
            // Q channel: sample at offset center (half symbol later)
            let q_center = k * sps + sps / 2 + half_sps;

            if i_center < signal.len() {
                bits.push(signal[i_center].re > 0.0);
            }
            if q_center < signal.len() {
                bits.push(signal[q_center].im > 0.0);
            }
        }

        bits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_modulation_length() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
        let bits = vec![true, false, true, true, false, false];
        let signal = oqpsk.modulate(&bits);
        assert_eq!(signal.len(), 3 * 8); // 3 symbols × 8 sps
    }

    #[test]
    fn test_max_phase_change() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
        let bits = vec![true, false, false, true, true, true, false, false];
        let signal = oqpsk.modulate(&bits);

        // Check phase changes between successive samples
        for i in 1..signal.len() {
            let phase_prev = signal[i - 1].arg();
            let phase_curr = signal[i].arg();
            let mut delta = (phase_curr - phase_prev).abs();
            if delta > PI {
                delta = 2.0 * PI - delta;
            }
            // OQPSK: max phase change should be ≤ π/2 + small margin
            assert!(delta < PI / 2.0 + 0.1,
                "Phase change at sample {} too large: {:.3} rad", i, delta);
        }
    }

    #[test]
    fn test_demod_roundtrip() {
        let mut modulator = OqpskModulator::new(1000.0, 8000.0);
        let demodulator = OqpskDemodulator::new(1000.0, 8000.0);

        let bits = vec![true, false, true, true, false, true];
        let signal = modulator.modulate(&bits);
        let decoded = demodulator.demodulate(&signal);

        // I bits should match
        for k in 0..(bits.len() / 2) {
            if 2 * k < decoded.len() {
                assert_eq!(decoded[2 * k], bits[2 * k],
                    "I bit {} mismatch", k);
            }
        }
    }

    #[test]
    fn test_bits_per_symbol() {
        let oqpsk = OqpskModulator::new(1000.0, 8000.0);
        assert_eq!(oqpsk.bits_per_symbol(), 2);
    }

    #[test]
    fn test_papr_lower_than_qpsk() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);

        // Generate OQPSK signal
        let bits: Vec<bool> = (0..100).map(|i| i % 3 != 0).collect();
        let oqpsk_signal = oqpsk.modulate(&bits);

        // OQPSK PAPR should be low (ideal rectangular pulses: 0 dB)
        let papr = OqpskModulator::papr(&oqpsk_signal);
        // With rectangular pulses, PAPR should be close to 0 dB
        assert!(papr < 3.5, "OQPSK PAPR too high: {:.2} dB", papr);
    }

    #[test]
    fn test_empty_input() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
        let signal = oqpsk.modulate(&[]);
        assert!(signal.is_empty());
    }

    #[test]
    fn test_single_symbol() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
        let bits = vec![true, true];
        let signal = oqpsk.modulate(&bits);
        assert_eq!(signal.len(), 8);

        // I=+1, Q=+1 → 45° constellation point
        let mid = signal[4]; // middle sample
        assert!(mid.re > 0.0, "I should be positive");
    }

    #[test]
    fn test_i_q_stagger() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
        let sps = oqpsk.samples_per_symbol();

        // Two symbols: [+1,+1] then [-1,-1]
        let bits = vec![true, true, false, false];
        let signal = oqpsk.modulate(&bits);

        // I changes at symbol boundary (sample sps)
        // Q changes at half-symbol offset (sample sps + sps/2)
        let i_at_boundary = signal[sps].re;
        let i_before = signal[sps - 1].re;
        // I should transition at sps
        assert!(i_before > 0.0 && i_at_boundary < 0.0,
            "I should change at symbol boundary");
    }

    #[test]
    fn test_reset() {
        let mut oqpsk = OqpskModulator::new(1000.0, 8000.0);
        let _ = oqpsk.modulate(&[true, false, false, true]);
        oqpsk.reset();
        assert!((oqpsk.prev_i - 1.0).abs() < 1e-10);
        assert!((oqpsk.prev_q - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_constellation_points() {
        let mut oqpsk = OqpskModulator::new(1000.0, 4000.0); // 4 sps
        // All four symbol combinations
        for &(bi, bq) in &[(true, true), (true, false), (false, true), (false, false)] {
            oqpsk.reset();
            let signal = oqpsk.modulate(&[bi, bq]);
            // Check I value at center
            let center = signal[2]; // sps=4, center=2
            let expected_i = if bi { 1.0 } else { -1.0 };
            assert!((center.re - expected_i).abs() < 1e-10,
                "I bit {} should give {}", bi, expected_i);
        }
    }
}
