//! PN Sequence Sync — Synchronize and despread using known PN sequences
//!
//! Correlates incoming symbols against a known PN (Pseudo-Noise) sequence
//! to achieve synchronization, then despreads. Useful for CDMA, GPS C/A
//! code synchronization, and spread-spectrum communications.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pn_sync::{PnGenerator, PnCorrelator};
//!
//! let pn = PnGenerator::gold(5, 0x1F, 0x1D);
//! let sequence = pn.generate(31);
//!
//! let mut correlator = PnCorrelator::new(sequence.clone());
//! // Feed the PN sequence — should get a strong peak
//! let bipolar: Vec<f64> = sequence.iter().map(|&b| if b { 1.0 } else { -1.0 }).collect();
//! let (peak_val, peak_idx) = correlator.correlate(&bipolar);
//! assert!(peak_val > 0.9); // Normalized peak near 1.0
//! ```

/// Linear Feedback Shift Register (LFSR) PN sequence generator.
#[derive(Debug, Clone)]
pub struct PnGenerator {
    /// Feedback polynomial (bitmask).
    polynomial: u32,
    /// Shift register state.
    state: u32,
    /// Register length (number of stages).
    length: usize,
    /// Initial state.
    initial_state: u32,
}

impl PnGenerator {
    /// Create a new PN generator with given polynomial and initial state.
    ///
    /// Polynomial is specified as a bitmask of feedback taps.
    /// Length is the number of shift register stages.
    pub fn new(length: usize, polynomial: u32, initial_state: u32) -> Self {
        let mask = (1u32 << length) - 1;
        Self {
            polynomial: polynomial & mask,
            state: initial_state & mask,
            length,
            initial_state: initial_state & mask,
        }
    }

    /// Gold code generator: XOR of two m-sequences.
    pub fn gold(length: usize, poly1: u32, poly2: u32) -> GoldCodeGenerator {
        GoldCodeGenerator {
            gen1: PnGenerator::new(length, poly1, 1),
            gen2: PnGenerator::new(length, poly2, 1),
        }
    }

    /// Maximum-length sequence (m-sequence).
    ///
    /// Common polynomials:
    /// - Length 5: 0x25 (x^5 + x^2 + 1)
    /// - Length 7: 0x89 (x^7 + x^3 + 1)
    /// - Length 10: 0x409 (x^10 + x^3 + 1)
    pub fn m_sequence(length: usize, polynomial: u32) -> Self {
        Self::new(length, polynomial, 1)
    }

    /// Generate next bit.
    pub fn next_bit(&mut self) -> bool {
        let feedback = (self.state & self.polynomial).count_ones() % 2;
        let output = self.state & 1 != 0;
        self.state >>= 1;
        self.state |= feedback << (self.length - 1);
        output
    }

    /// Generate a sequence of N bits.
    pub fn generate(&self, n: usize) -> Vec<bool> {
        let mut gen = self.clone();
        (0..n).map(|_| gen.next_bit()).collect()
    }

    /// Period of the m-sequence: 2^length - 1.
    pub fn period(&self) -> usize {
        (1 << self.length) - 1
    }

    pub fn reset(&mut self) {
        self.state = self.initial_state;
    }
}

/// Gold code generator — XOR of two m-sequences.
#[derive(Debug, Clone)]
pub struct GoldCodeGenerator {
    gen1: PnGenerator,
    gen2: PnGenerator,
}

impl GoldCodeGenerator {
    /// Generate next Gold code bit.
    pub fn next_bit(&mut self) -> bool {
        self.gen1.next_bit() ^ self.gen2.next_bit()
    }

    /// Generate N bits.
    pub fn generate(&self, n: usize) -> Vec<bool> {
        let mut gen = self.clone();
        (0..n).map(|_| gen.next_bit()).collect()
    }

    /// Set code phase offset (shift gen2 by offset chips).
    pub fn set_phase(&mut self, offset: usize) {
        self.gen2.reset();
        for _ in 0..offset {
            self.gen2.next_bit();
        }
    }

    pub fn reset(&mut self) {
        self.gen1.reset();
        self.gen2.reset();
    }
}

/// PN sequence correlator — finds PN alignment in received signal.
#[derive(Debug, Clone)]
pub struct PnCorrelator {
    /// Reference PN sequence (bipolar: +1/-1).
    reference: Vec<f64>,
    /// Normalized flag.
    normalized: bool,
}

impl PnCorrelator {
    /// Create correlator from boolean PN sequence.
    pub fn new(pn_sequence: Vec<bool>) -> Self {
        let reference: Vec<f64> = pn_sequence
            .iter()
            .map(|&b| if b { 1.0 } else { -1.0 })
            .collect();
        Self {
            reference,
            normalized: true,
        }
    }

    /// Create correlator from bipolar sequence.
    pub fn from_bipolar(reference: Vec<f64>) -> Self {
        Self {
            reference,
            normalized: true,
        }
    }

    /// Correlate received signal against the PN reference.
    ///
    /// Returns (normalized_peak_value, peak_offset).
    pub fn correlate(&self, received: &[f64]) -> (f64, usize) {
        if received.len() < self.reference.len() {
            return (0.0, 0);
        }

        let n = self.reference.len();
        let num_offsets = received.len() - n + 1;
        let ref_energy: f64 = self.reference.iter().map(|x| x * x).sum();

        let mut max_val = 0.0_f64;
        let mut max_idx = 0;

        for offset in 0..num_offsets {
            let mut corr = 0.0;
            let mut sig_energy = 0.0;
            for i in 0..n {
                corr += received[offset + i] * self.reference[i];
                sig_energy += received[offset + i] * received[offset + i];
            }

            let normalized = if self.normalized && ref_energy > 1e-20 && sig_energy > 1e-20 {
                corr / (ref_energy.sqrt() * sig_energy.sqrt())
            } else {
                corr
            };

            if normalized.abs() > max_val.abs() {
                max_val = normalized;
                max_idx = offset;
            }
        }

        (max_val, max_idx)
    }

    /// Correlate complex IQ signal against the PN reference.
    pub fn correlate_complex(&self, received: &[num_complex::Complex64]) -> (f64, usize) {
        // Correlate against real part
        let real: Vec<f64> = received.iter().map(|c| c.re).collect();
        self.correlate(&real)
    }

    /// Despread: multiply received signal by PN sequence at given offset.
    pub fn despread(&self, received: &[f64], offset: usize) -> Vec<f64> {
        let n = self.reference.len();
        if offset + n > received.len() {
            return Vec::new();
        }

        received[offset..offset + n]
            .iter()
            .zip(self.reference.iter())
            .map(|(&r, &p)| r * p)
            .collect()
    }

    pub fn sequence_length(&self) -> usize {
        self.reference.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_m_sequence_period() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        assert_eq!(gen.period(), 31);
    }

    #[test]
    fn test_m_sequence_generation() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        let seq = gen.generate(31);
        assert_eq!(seq.len(), 31);
        // m-sequence should have (2^(n-1)) ones and (2^(n-1)-1) zeros
        let ones: usize = seq.iter().filter(|&&b| b).count();
        let zeros: usize = seq.iter().filter(|&&b| !b).count();
        assert_eq!(ones + zeros, 31);
        assert!((ones as i32 - zeros as i32).abs() <= 1);
    }

    #[test]
    fn test_m_sequence_periodicity() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        let seq1 = gen.generate(31);
        let seq2 = gen.generate(62);
        // Second period should equal the first
        for i in 0..31 {
            assert_eq!(seq1[i], seq2[31 + i], "Periodicity check at {}", i);
        }
    }

    #[test]
    fn test_gold_code_generation() {
        let gold = PnGenerator::gold(5, 0x25, 0x37);
        let seq = gold.generate(31);
        assert_eq!(seq.len(), 31);
    }

    #[test]
    fn test_gold_code_phase() {
        let mut gold = PnGenerator::gold(5, 0x25, 0x37);
        let seq1 = gold.generate(31);
        gold.set_phase(5);
        let seq2 = gold.generate(31);
        // Different phases should give different sequences
        assert_ne!(seq1, seq2);
    }

    #[test]
    fn test_correlator_autocorrelation() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        let seq = gen.generate(31);
        let correlator = PnCorrelator::new(seq.clone());

        let bipolar: Vec<f64> = seq.iter().map(|&b| if b { 1.0 } else { -1.0 }).collect();
        let (peak, idx) = correlator.correlate(&bipolar);
        assert!(peak > 0.9, "Autocorrelation peak should be ~1.0, got {}", peak);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_correlator_offset_detection() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        let seq = gen.generate(31);
        let correlator = PnCorrelator::new(seq.clone());

        // Embed PN sequence at offset 10
        let mut signal = vec![0.0; 50];
        for (i, &b) in seq.iter().enumerate() {
            signal[10 + i] = if b { 1.0 } else { -1.0 };
        }

        let (peak, idx) = correlator.correlate(&signal);
        assert!(peak > 0.9, "Should find embedded sequence");
        assert_eq!(idx, 10, "Should detect correct offset");
    }

    #[test]
    fn test_despread() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        let seq = gen.generate(31);
        let correlator = PnCorrelator::new(seq.clone());

        // Spread: data bit * PN sequence
        let data_bit = 1.0;
        let spread: Vec<f64> = seq.iter().map(|&b| data_bit * if b { 1.0 } else { -1.0 }).collect();

        let despread = correlator.despread(&spread, 0);
        // After despreading, all values should be +1.0 (data * pn * pn = data)
        for &v in &despread {
            assert!((v - 1.0).abs() < 1e-10, "Despread should give all 1.0");
        }
    }

    #[test]
    fn test_pn_reset() {
        let mut gen = PnGenerator::m_sequence(5, 0x25);
        let seq1 = gen.generate(10);
        gen.reset();
        let seq2 = gen.generate(10);
        assert_eq!(seq1, seq2, "Reset should produce same sequence");
    }

    #[test]
    fn test_gold_reset() {
        let mut gold = PnGenerator::gold(5, 0x25, 0x37);
        let seq1 = gold.generate(10);
        gold.reset();
        let seq2 = gold.generate(10);
        assert_eq!(seq1, seq2);
    }

    #[test]
    fn test_correlator_noise() {
        let gen = PnGenerator::m_sequence(7, 0x89);
        let seq = gen.generate(127);
        let correlator = PnCorrelator::new(seq.clone());

        // Signal + noise
        let mut signal: Vec<f64> = seq.iter().map(|&b| if b { 1.0 } else { -1.0 }).collect();
        // Add some noise (deterministic pseudo-noise)
        for (i, s) in signal.iter_mut().enumerate() {
            *s += (i as f64 * 0.7).sin() * 0.3;
        }

        let (peak, idx) = correlator.correlate(&signal);
        assert!(peak > 0.8, "Should detect through noise, got {}", peak);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_sequence_length() {
        let gen = PnGenerator::m_sequence(5, 0x25);
        let seq = gen.generate(31);
        let correlator = PnCorrelator::new(seq);
        assert_eq!(correlator.sequence_length(), 31);
    }
}
