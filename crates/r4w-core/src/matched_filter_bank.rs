//! Multi-template matched filter bank for parallel signal/target detection.
//!
//! Provides [`MatchedFilter`] for single-template cross-correlation,
//! [`MatchedFilterBank`] for running multiple templates in parallel, and
//! [`ComplexMatchedFilter`] for IQ (complex) signals. Utility functions
//! generate common radar/communications templates such as LFM chirps and
//! Barker codes.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::matched_filter_bank::{MatchedFilter, MatchedFilterBank, barker_template};
//!
//! // Build a bank with a Barker-13 template
//! let mut bank = MatchedFilterBank::new();
//! bank.add_template("barker13", barker_template(13));
//!
//! // Create a test signal: zeros, then the Barker-13 pattern, then zeros
//! let mut signal = vec![0.0; 10];
//! signal.extend_from_slice(&barker_template(13));
//! signal.extend_from_slice(&[0.0; 10]);
//!
//! let results = bank.process(&signal);
//! assert_eq!(results.len(), 1);
//! assert!(results[0].peak_value > 12.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// BankResult
// ---------------------------------------------------------------------------

/// Result produced by [`MatchedFilterBank::process`] for a single template.
#[derive(Debug, Clone)]
pub struct BankResult {
    /// Name of the template that produced this result.
    pub template_name: String,
    /// Peak value of the cross-correlation output.
    pub peak_value: f64,
    /// Sample index at which the peak occurs.
    pub peak_index: usize,
    /// Estimated SNR improvement in dB (matched filter processing gain).
    pub snr_db: f64,
}

// ---------------------------------------------------------------------------
// MatchedFilter
// ---------------------------------------------------------------------------

/// Single-template matched filter.
///
/// Computes the cross-correlation of an input signal with the time-reversed
/// (conjugate) template, which maximises output SNR for signals matching the
/// template shape.
pub struct MatchedFilter {
    /// Template used for cross-correlation (time-reversed conjugate is
    /// applied internally via the correlation sum order).
    template: Vec<f64>,
    /// Template length.
    length: usize,
}

impl MatchedFilter {
    /// Create a new matched filter from the given `template`.
    pub fn new(template: Vec<f64>) -> Self {
        let length = template.len();
        Self { template, length }
    }

    /// Cross-correlate `input` with the time-reversed conjugate template.
    ///
    /// For real-valued signals this is equivalent to the standard
    /// cross-correlation: `output[i] = sum_j input[i+j] * template[j]`.
    ///
    /// The output length is `input.len() - template.len() + 1` (valid
    /// correlation — no zero-padding). Returns an empty vector when the input
    /// is shorter than the template.
    pub fn filter(&self, input: &[f64]) -> Vec<f64> {
        if input.len() < self.length {
            return Vec::new();
        }
        let out_len = input.len() - self.length + 1;
        let mut output = Vec::with_capacity(out_len);
        for i in 0..out_len {
            let mut sum = 0.0_f64;
            for (j, &t) in self.template.iter().enumerate() {
                sum += input[i + j] * t;
            }
            output.push(sum);
        }
        output
    }

    /// Matched-filter processing gain in dB: `10 * log10(N)`.
    pub fn snr_gain_db(&self) -> f64 {
        10.0 * (self.length as f64).log10()
    }
}

// ---------------------------------------------------------------------------
// MatchedFilterBank
// ---------------------------------------------------------------------------

/// A collection of named matched filters that are evaluated in parallel.
pub struct MatchedFilterBank {
    filters: Vec<(String, MatchedFilter)>,
}

impl MatchedFilterBank {
    /// Create an empty filter bank.
    pub fn new() -> Self {
        Self {
            filters: Vec::new(),
        }
    }

    /// Add a named template to the bank.
    pub fn add_template(&mut self, name: &str, template: Vec<f64>) {
        let mf = MatchedFilter::new(template);
        self.filters.push((name.to_string(), mf));
    }

    /// Run every template against `input` and return results **sorted by peak
    /// value in descending order** (strongest detection first).
    pub fn process(&self, input: &[f64]) -> Vec<BankResult> {
        let mut results: Vec<BankResult> = self
            .filters
            .iter()
            .filter_map(|(name, mf)| {
                let corr = mf.filter(input);
                if corr.is_empty() {
                    return None;
                }
                let (peak_index, peak_value) = corr
                    .iter()
                    .enumerate()
                    .fold((0, f64::NEG_INFINITY), |(bi, bv), (i, &v)| {
                        if v > bv {
                            (i, v)
                        } else {
                            (bi, bv)
                        }
                    });
                Some(BankResult {
                    template_name: name.clone(),
                    peak_value,
                    peak_index,
                    snr_db: mf.snr_gain_db(),
                })
            })
            .collect();
        results.sort_by(|a, b| b.peak_value.partial_cmp(&a.peak_value).unwrap());
        results
    }

    /// Number of templates currently in the bank.
    pub fn num_templates(&self) -> usize {
        self.filters.len()
    }
}

impl Default for MatchedFilterBank {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// ComplexMatchedFilter
// ---------------------------------------------------------------------------

/// Matched filter for complex (IQ) signals.
///
/// Each sample is represented as `(re, im)`.  The filter computes the
/// magnitude of the complex cross-correlation with the conjugate
/// time-reversed template.
pub struct ComplexMatchedFilter {
    /// Conjugate of the template (for cross-correlation).
    conj_template: Vec<(f64, f64)>,
    length: usize,
}

impl ComplexMatchedFilter {
    /// Create a complex matched filter from an IQ template.
    pub fn new(template: Vec<(f64, f64)>) -> Self {
        let length = template.len();
        let conj_template: Vec<(f64, f64)> = template
            .iter()
            .map(|&(re, im)| (re, -im)) // conjugate
            .collect();
        Self {
            conj_template,
            length,
        }
    }

    /// Cross-correlate `input` with the conjugate template and return the
    /// **magnitude** at each lag (valid correlation).
    ///
    /// `output[i] = |sum_j input[i+j] * conj(template[j])|`
    pub fn filter(&self, input: &[(f64, f64)]) -> Vec<f64> {
        if input.len() < self.length {
            return Vec::new();
        }
        let out_len = input.len() - self.length + 1;
        let mut output = Vec::with_capacity(out_len);
        for i in 0..out_len {
            let mut sum_re = 0.0_f64;
            let mut sum_im = 0.0_f64;
            for (j, &(cr, ci)) in self.conj_template.iter().enumerate() {
                let (sr, si) = input[i + j];
                // complex multiply: (sr + j*si) * (cr + j*ci)
                sum_re += sr * cr - si * ci;
                sum_im += sr * ci + si * cr;
            }
            output.push((sum_re * sum_re + sum_im * sum_im).sqrt());
        }
        output
    }
}

// ---------------------------------------------------------------------------
// Template generators
// ---------------------------------------------------------------------------

/// Generate a linear frequency-modulated (LFM) chirp template.
///
/// * `bandwidth` — chirp sweep in Hz
/// * `duration_s` — pulse duration in seconds
/// * `sample_rate` — samples per second
pub fn chirp_template(bandwidth: f64, duration_s: f64, sample_rate: f64) -> Vec<f64> {
    let n = (duration_s * sample_rate).round() as usize;
    let chirp_rate = bandwidth / duration_s;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            (2.0 * PI * ((-bandwidth / 2.0) * t + 0.5 * chirp_rate * t * t)).cos()
        })
        .collect()
}

/// Return a Barker code of the given `length` as `+1 / -1` values.
///
/// Valid Barker lengths: 2, 3, 4, 5, 7, 11, 13.
///
/// # Panics
///
/// Panics if `length` is not a valid Barker code length.
pub fn barker_template(length: usize) -> Vec<f64> {
    let code: &[f64] = match length {
        2 => &[1.0, -1.0],
        3 => &[1.0, 1.0, -1.0],
        4 => &[1.0, 1.0, -1.0, 1.0],
        5 => &[1.0, 1.0, 1.0, -1.0, 1.0],
        7 => &[1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
        11 => &[1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
        13 => &[1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0],
        _ => panic!(
            "Invalid Barker code length {}. Valid lengths: 2, 3, 4, 5, 7, 11, 13",
            length
        ),
    };
    code.to_vec()
}

/// Pulse compression ratio: ratio of autocorrelation peak to the largest
/// sidelobe magnitude.
pub fn pulse_compression_ratio(template: &[f64]) -> f64 {
    let n = template.len();
    // full autocorrelation length = 2*n - 1
    let auto_len = 2 * n - 1;
    let mut auto = vec![0.0_f64; auto_len];
    for lag in 0..auto_len {
        let mut sum = 0.0;
        for i in 0..n {
            let j = lag as isize - (n as isize - 1) + i as isize;
            if j >= 0 && (j as usize) < n {
                sum += template[i] * template[j as usize];
            }
        }
        auto[lag] = sum;
    }
    let center = n - 1; // index of zero-lag (peak)
    let peak = auto[center].abs();
    let max_sidelobe = auto
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != center)
        .map(|(_, &v)| v.abs())
        .fold(0.0_f64, f64::max);
    if max_sidelobe == 0.0 {
        f64::INFINITY
    } else {
        peak / max_sidelobe
    }
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

/// Create a pre-populated [`MatchedFilterBank`] with common radar templates:
///
/// * `chirp_short` — 10 us LFM chirp, bandwidth = sample_rate / 4
/// * `chirp_long`  — 50 us LFM chirp, bandwidth = sample_rate / 2
/// * `barker5`     — 5-chip Barker code
/// * `barker13`    — 13-chip Barker code
pub fn radar_filter_bank(sample_rate: f64) -> MatchedFilterBank {
    let mut bank = MatchedFilterBank::new();
    bank.add_template(
        "chirp_short",
        chirp_template(sample_rate / 4.0, 10e-6, sample_rate),
    );
    bank.add_template(
        "chirp_long",
        chirp_template(sample_rate / 2.0, 50e-6, sample_rate),
    );
    bank.add_template("barker5", barker_template(5));
    bank.add_template("barker13", barker_template(13));
    bank
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_matched_filter_output_length() {
        let template = vec![1.0, 1.0, 1.0];
        let mf = MatchedFilter::new(template);
        let input = vec![0.0; 10];
        let output = mf.filter(&input);
        // valid correlation: 10 - 3 + 1 = 8
        assert_eq!(output.len(), 8);
    }

    #[test]
    fn test_snr_gain_computation() {
        let template = vec![0.0; 100];
        let mf = MatchedFilter::new(template);
        let gain = mf.snr_gain_db();
        // 10 * log10(100) = 20 dB
        assert!((gain - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_peak_at_correct_delay() {
        // Use Barker-5 code whose autocorrelation has an unambiguous
        // positive peak at zero lag.
        let template = barker_template(5); // [1, 1, 1, -1, 1]
        let mf = MatchedFilter::new(template.clone());

        // Embed the template at offset 7 inside a longer signal.
        let mut signal = vec![0.0; 30];
        for (i, &v) in template.iter().enumerate() {
            signal[7 + i] = v;
        }

        let output = mf.filter(&signal);
        // Peak of autocorrelation should appear at index 7.
        let (peak_idx, &peak_val) = output
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_idx, 7);
        // Autocorrelation peak = sum of squares = 5.0.
        assert!((peak_val - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_filter_bank_multiple_templates() {
        let mut bank = MatchedFilterBank::new();
        bank.add_template("short", vec![1.0, 1.0]);
        bank.add_template("medium", vec![1.0, 1.0, 1.0]);
        bank.add_template("long", vec![1.0, 1.0, 1.0, 1.0]);
        assert_eq!(bank.num_templates(), 3);

        let input = vec![1.0; 10];
        let results = bank.process(&input);
        assert_eq!(results.len(), 3);
    }

    #[test]
    fn test_results_sorted_by_peak_value() {
        let mut bank = MatchedFilterBank::new();
        // Templates of different lengths of all-ones against an all-ones input:
        // longer template yields a bigger correlation peak.
        bank.add_template("len2", vec![1.0; 2]);
        bank.add_template("len5", vec![1.0; 5]);
        bank.add_template("len3", vec![1.0; 3]);

        let input = vec![1.0; 20];
        let results = bank.process(&input);

        // Results must be descending by peak_value.
        for w in results.windows(2) {
            assert!(
                w[0].peak_value >= w[1].peak_value,
                "Results not sorted: {} < {}",
                w[0].peak_value,
                w[1].peak_value,
            );
        }
        // The 5-element template should be first (peak = 5).
        assert_eq!(results[0].template_name, "len5");
    }

    #[test]
    fn test_complex_matched_filter() {
        // Pure real template (im=0) should behave like real matched filter.
        let template: Vec<(f64, f64)> = vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0)];
        let cmf = ComplexMatchedFilter::new(template.clone());

        let mut input: Vec<(f64, f64)> = vec![(0.0, 0.0); 10];
        for (i, &(re, im)) in template.iter().enumerate() {
            input[3 + i] = (re, im);
        }

        let output = cmf.filter(&input);
        assert!(!output.is_empty());

        // Peak should be at index 3.
        let (peak_idx, &peak_val) = output
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_idx, 3);
        // Autocorrelation peak for [1,-1,1] is 3.0.
        assert!((peak_val - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_chirp_template_generation() {
        let sample_rate = 1e6; // 1 MHz
        let bandwidth = 100e3; // 100 kHz
        let duration = 10e-6; // 10 us
        let tmpl = chirp_template(bandwidth, duration, sample_rate);

        // Expected number of samples: 10e-6 * 1e6 = 10
        assert_eq!(tmpl.len(), 10);

        // All values should be in [-1, 1] since it's a cosine.
        for &v in &tmpl {
            assert!(v >= -1.0 && v <= 1.0, "chirp sample out of range: {}", v);
        }
    }

    #[test]
    fn test_barker_template_known_values() {
        let b13 = barker_template(13);
        assert_eq!(b13.len(), 13);
        assert_eq!(
            b13,
            vec![1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0]
        );

        let b5 = barker_template(5);
        assert_eq!(b5, vec![1.0, 1.0, 1.0, -1.0, 1.0]);

        let b7 = barker_template(7);
        assert_eq!(b7.len(), 7);
    }

    #[test]
    fn test_pulse_compression_ratio_barker13() {
        let b13 = barker_template(13);
        let pcr = pulse_compression_ratio(&b13);
        // Barker-13 autocorrelation peak = 13, max sidelobe = 1 => ratio = 13.
        assert!(
            (pcr - 13.0).abs() < 1e-10,
            "Expected PCR of 13.0 for Barker-13, got {}",
            pcr
        );
    }

    #[test]
    fn test_radar_factory_bank() {
        let sample_rate = 1e6;
        let bank = radar_filter_bank(sample_rate);
        assert_eq!(bank.num_templates(), 4);

        // Smoke-test: process a short signal without panic.
        let input = vec![0.0; 200];
        let results = bank.process(&input);
        // Some templates may produce empty output if input is too short,
        // but at least the barker templates (len 5 and 13) should produce results.
        assert!(
            results.len() >= 2,
            "Expected at least 2 results, got {}",
            results.len()
        );
    }
}
