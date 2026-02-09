//! Probe Density — Bit and transition density measurement
//!
//! Measures the density of 1-bits and symbol transitions in a
//! digital stream. Running estimates for clock recovery assessment,
//! scrambler verification, and DC balance monitoring. Includes
//! run-length analysis for line coding evaluation.
//! GNU Radio equivalent: `probe_density_b`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::probe_density::ProbeDensity;
//!
//! let mut probe = ProbeDensity::new(0.01);
//! let bits = vec![true, false, true, false, true, true, false, true];
//! probe.process_block(&bits);
//! let d = probe.density();
//! assert!((d - 0.625).abs() < 0.2); // Roughly 5/8
//! ```

/// Bit density probe with exponential averaging.
#[derive(Debug, Clone)]
pub struct ProbeDensity {
    /// Smoothing factor (0 = no update, 1 = instant).
    alpha: f64,
    /// Current density estimate (0.0 to 1.0).
    density: f64,
    /// Total bits seen.
    total_bits: usize,
    /// Total ones seen (for exact density).
    total_ones: usize,
}

impl ProbeDensity {
    /// Create a new density probe.
    ///
    /// `alpha` is the exponential smoothing factor (e.g., 0.01 for slow tracking).
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            density: 0.5, // Start at balanced
            total_bits: 0,
            total_ones: 0,
        }
    }

    /// Process a single bit.
    pub fn process_bit(&mut self, bit: bool) {
        let val = if bit { 1.0 } else { 0.0 };
        self.density = self.alpha * val + (1.0 - self.alpha) * self.density;
        self.total_bits += 1;
        if bit {
            self.total_ones += 1;
        }
    }

    /// Process a block of bits.
    pub fn process_block(&mut self, bits: &[bool]) {
        for &bit in bits {
            self.process_bit(bit);
        }
    }

    /// Get the smoothed density estimate (0.0 to 1.0).
    pub fn density(&self) -> f64 {
        self.density
    }

    /// Get the exact density (total ones / total bits).
    pub fn exact_density(&self) -> f64 {
        if self.total_bits == 0 {
            return 0.5;
        }
        self.total_ones as f64 / self.total_bits as f64
    }

    /// Total bits processed.
    pub fn total_bits(&self) -> usize {
        self.total_bits
    }

    /// Reset the probe.
    pub fn reset(&mut self) {
        self.density = 0.5;
        self.total_bits = 0;
        self.total_ones = 0;
    }
}

/// Transition density probe.
///
/// Measures how often consecutive bits differ (useful for clock recovery).
#[derive(Debug, Clone)]
pub struct TransitionDensity {
    /// Smoothing factor.
    alpha: f64,
    /// Transition density estimate (0.0 to 1.0).
    density: f64,
    /// Previous bit value.
    prev_bit: Option<bool>,
    /// Total transitions counted.
    total_transitions: usize,
    /// Total bit pairs examined.
    total_pairs: usize,
}

impl TransitionDensity {
    /// Create a new transition density probe.
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.0, 1.0),
            density: 0.5,
            prev_bit: None,
            total_transitions: 0,
            total_pairs: 0,
        }
    }

    /// Process a single bit.
    pub fn process_bit(&mut self, bit: bool) {
        if let Some(prev) = self.prev_bit {
            let transition = if bit != prev { 1.0 } else { 0.0 };
            self.density = self.alpha * transition + (1.0 - self.alpha) * self.density;
            self.total_pairs += 1;
            if bit != prev {
                self.total_transitions += 1;
            }
        }
        self.prev_bit = Some(bit);
    }

    /// Process a block of bits.
    pub fn process_block(&mut self, bits: &[bool]) {
        for &bit in bits {
            self.process_bit(bit);
        }
    }

    /// Get the smoothed transition density estimate.
    pub fn density(&self) -> f64 {
        self.density
    }

    /// Get the exact transition density.
    pub fn exact_density(&self) -> f64 {
        if self.total_pairs == 0 {
            return 0.5;
        }
        self.total_transitions as f64 / self.total_pairs as f64
    }

    /// Total transitions detected.
    pub fn total_transitions(&self) -> usize {
        self.total_transitions
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.density = 0.5;
        self.prev_bit = None;
        self.total_transitions = 0;
        self.total_pairs = 0;
    }
}

/// Run-length analyzer — measures distribution of consecutive same-value runs.
#[derive(Debug, Clone)]
pub struct RunLengthAnalyzer {
    /// Run length histogram (index = run length, value = count).
    histogram: Vec<usize>,
    /// Current run value.
    current_value: Option<bool>,
    /// Current run length.
    current_length: usize,
    /// Total runs counted.
    total_runs: usize,
}

impl RunLengthAnalyzer {
    /// Create a new run-length analyzer.
    ///
    /// `max_run` is the maximum run length to track (longer runs are clamped).
    pub fn new(max_run: usize) -> Self {
        Self {
            histogram: vec![0; max_run.max(1) + 1],
            current_value: None,
            current_length: 0,
            total_runs: 0,
        }
    }

    /// Process a single bit.
    pub fn process_bit(&mut self, bit: bool) {
        match self.current_value {
            Some(v) if v == bit => {
                self.current_length += 1;
            }
            _ => {
                // End current run
                if self.current_value.is_some() {
                    self.record_run(self.current_length);
                }
                self.current_value = Some(bit);
                self.current_length = 1;
            }
        }
    }

    /// Process a block of bits.
    pub fn process_block(&mut self, bits: &[bool]) {
        for &bit in bits {
            self.process_bit(bit);
        }
    }

    /// Flush the current run (call when stream ends).
    pub fn flush(&mut self) {
        if self.current_value.is_some() && self.current_length > 0 {
            self.record_run(self.current_length);
            self.current_length = 0;
        }
    }

    fn record_run(&mut self, length: usize) {
        let idx = length.min(self.histogram.len() - 1);
        self.histogram[idx] += 1;
        self.total_runs += 1;
    }

    /// Get the run-length histogram.
    pub fn histogram(&self) -> &[usize] {
        &self.histogram
    }

    /// Average run length.
    pub fn average_run_length(&self) -> f64 {
        if self.total_runs == 0 {
            return 0.0;
        }
        let sum: usize = self
            .histogram
            .iter()
            .enumerate()
            .map(|(len, &count)| len * count)
            .sum();
        sum as f64 / self.total_runs as f64
    }

    /// Maximum observed run length.
    pub fn max_run_length(&self) -> usize {
        for i in (0..self.histogram.len()).rev() {
            if self.histogram[i] > 0 {
                return i;
            }
        }
        0
    }

    /// Total runs counted.
    pub fn total_runs(&self) -> usize {
        self.total_runs
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.histogram.fill(0);
        self.current_value = None;
        self.current_length = 0;
        self.total_runs = 0;
    }
}

/// Compute exact bit density of a slice.
pub fn bit_density(bits: &[bool]) -> f64 {
    if bits.is_empty() {
        return 0.5;
    }
    let ones = bits.iter().filter(|&&b| b).count();
    ones as f64 / bits.len() as f64
}

/// Compute exact transition density of a slice.
pub fn transition_density(bits: &[bool]) -> f64 {
    if bits.len() < 2 {
        return 0.0;
    }
    let transitions = bits.windows(2).filter(|w| w[0] != w[1]).count();
    transitions as f64 / (bits.len() - 1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bit_density_all_ones() {
        assert!((bit_density(&[true, true, true]) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_bit_density_all_zeros() {
        assert!((bit_density(&[false, false, false]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_bit_density_balanced() {
        assert!((bit_density(&[true, false, true, false]) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_transition_density_alternating() {
        // Alternating bits: every pair is a transition
        assert!(
            (transition_density(&[true, false, true, false]) - 1.0).abs() < 1e-10
        );
    }

    #[test]
    fn test_transition_density_constant() {
        assert!(
            (transition_density(&[true, true, true, true]) - 0.0).abs() < 1e-10
        );
    }

    #[test]
    fn test_probe_density() {
        let mut probe = ProbeDensity::new(1.0); // alpha=1 for instant tracking
        probe.process_bit(true);
        assert!((probe.density() - 1.0).abs() < 1e-10);
        probe.process_bit(false);
        assert!((probe.density() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_probe_density_exact() {
        let mut probe = ProbeDensity::new(0.1);
        let bits = vec![true, false, true, true, false];
        probe.process_block(&bits);
        assert!((probe.exact_density() - 0.6).abs() < 1e-10);
        assert_eq!(probe.total_bits(), 5);
    }

    #[test]
    fn test_transition_density_probe() {
        let mut td = TransitionDensity::new(1.0);
        td.process_bit(true);
        td.process_bit(false); // Transition
        assert!((td.density() - 1.0).abs() < 1e-10);
        td.process_bit(false); // No transition
        assert!((td.density() - 0.0).abs() < 1e-10);
        assert_eq!(td.total_transitions(), 1);
    }

    #[test]
    fn test_transition_density_exact() {
        let mut td = TransitionDensity::new(0.1);
        td.process_block(&[true, false, true, false]); // 3 transitions out of 3 pairs
        assert!((td.exact_density() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_run_length_analyzer() {
        let mut rla = RunLengthAnalyzer::new(10);
        // 1,1,1,0,0,1 → runs: 3, 2, 1
        rla.process_block(&[true, true, true, false, false, true]);
        rla.flush();
        assert_eq!(rla.histogram()[3], 1); // Run of 3
        assert_eq!(rla.histogram()[2], 1); // Run of 2
        assert_eq!(rla.histogram()[1], 1); // Run of 1
        assert_eq!(rla.total_runs(), 3);
    }

    #[test]
    fn test_run_length_average() {
        let mut rla = RunLengthAnalyzer::new(10);
        rla.process_block(&[true, true, false, false]);
        rla.flush();
        // Runs: 2, 2 → avg = 2.0
        assert!((rla.average_run_length() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_run_length_max() {
        let mut rla = RunLengthAnalyzer::new(20);
        rla.process_block(&[true; 10]);
        rla.flush();
        assert_eq!(rla.max_run_length(), 10);
    }

    #[test]
    fn test_reset() {
        let mut probe = ProbeDensity::new(0.1);
        probe.process_block(&[true; 100]);
        probe.reset();
        assert_eq!(probe.total_bits(), 0);
        assert!((probe.density() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_empty() {
        assert!((bit_density(&[]) - 0.5).abs() < 1e-10);
        assert!((transition_density(&[]) - 0.0).abs() < 1e-10);
    }
}
