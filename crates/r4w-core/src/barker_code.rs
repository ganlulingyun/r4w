//! Barker Code Generation for Synchronization
//!
//! Barker codes are finite binary sequences with optimal autocorrelation
//! properties: all off-peak (sidelobe) autocorrelation values have absolute
//! magnitude at most 1. This makes them ideal for frame synchronization,
//! radar pulse compression, and spread-spectrum communications.
//!
//! Only seven Barker codes are known to exist (lengths 2, 3, 4, 5, 7, 11, 13).
//! It has been proven that no Barker codes of odd length greater than 13 exist,
//! and extensive searches have found none of even length greater than 4.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::barker_code::{BarkerCode, autocorrelation, is_barker};
//!
//! let code = BarkerCode::Barker13;
//! let seq = code.sequence();
//! assert_eq!(seq.len(), 13);
//!
//! // Verify the Barker sidelobe property
//! let ac = autocorrelation(seq);
//! assert_eq!(ac[0], 13); // main lobe equals code length
//! for &val in &ac[1..] {
//!     assert!(val.abs() <= 1); // all sidelobes <= 1
//! }
//!
//! assert!(is_barker(seq));
//! ```

/// Known Barker code sequences.
///
/// Each variant corresponds to one of the seven known Barker codes,
/// represented as sequences of +1 and -1 values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BarkerCode {
    /// Length-2 Barker code: [+1, -1]
    Barker2,
    /// Length-3 Barker code: [+1, +1, -1]
    Barker3,
    /// Length-4 Barker code: [+1, +1, -1, +1]
    Barker4,
    /// Length-5 Barker code: [+1, +1, +1, -1, +1]
    Barker5,
    /// Length-7 Barker code: [+1, +1, +1, -1, -1, +1, -1]
    Barker7,
    /// Length-11 Barker code: [+1, +1, +1, -1, -1, -1, +1, -1, -1, +1, -1]
    Barker11,
    /// Length-13 Barker code: [+1, +1, +1, +1, +1, -1, -1, +1, +1, -1, +1, -1, +1]
    Barker13,
}

// Static slices for each Barker code sequence.
static BARKER2: [i8; 2] = [1, -1];
static BARKER3: [i8; 3] = [1, 1, -1];
static BARKER4: [i8; 4] = [1, 1, -1, 1];
static BARKER5: [i8; 5] = [1, 1, 1, -1, 1];
static BARKER7: [i8; 7] = [1, 1, 1, -1, -1, 1, -1];
static BARKER11: [i8; 11] = [1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1];
static BARKER13: [i8; 13] = [1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1];

impl BarkerCode {
    /// Returns the Barker sequence as a static slice of +1/-1 values.
    pub fn sequence(&self) -> &'static [i8] {
        match self {
            BarkerCode::Barker2 => &BARKER2,
            BarkerCode::Barker3 => &BARKER3,
            BarkerCode::Barker4 => &BARKER4,
            BarkerCode::Barker5 => &BARKER5,
            BarkerCode::Barker7 => &BARKER7,
            BarkerCode::Barker11 => &BARKER11,
            BarkerCode::Barker13 => &BARKER13,
        }
    }

    /// Returns the length of the Barker code.
    pub fn length(&self) -> usize {
        self.sequence().len()
    }

    /// Returns all seven known Barker codes, ordered by length.
    pub fn all() -> Vec<BarkerCode> {
        vec![
            BarkerCode::Barker2,
            BarkerCode::Barker3,
            BarkerCode::Barker4,
            BarkerCode::Barker5,
            BarkerCode::Barker7,
            BarkerCode::Barker11,
            BarkerCode::Barker13,
        ]
    }
}

/// Returns the Barker code of the given length, if one exists.
///
/// Only lengths 2, 3, 4, 5, 7, 11, and 13 have known Barker codes.
/// Returns `None` for any other length.
///
/// # Example
///
/// ```rust
/// use r4w_core::barker_code::generate_barker;
///
/// let code = generate_barker(7).unwrap();
/// assert_eq!(code, vec![1, 1, 1, -1, -1, 1, -1]);
/// assert!(generate_barker(6).is_none());
/// ```
pub fn generate_barker(length: usize) -> Option<Vec<i8>> {
    match length {
        2 => Some(BARKER2.to_vec()),
        3 => Some(BARKER3.to_vec()),
        4 => Some(BARKER4.to_vec()),
        5 => Some(BARKER5.to_vec()),
        7 => Some(BARKER7.to_vec()),
        11 => Some(BARKER11.to_vec()),
        13 => Some(BARKER13.to_vec()),
        _ => None,
    }
}

/// Computes the aperiodic autocorrelation of a bipolar code sequence.
///
/// Returns a vector of length `code.len()` where index 0 is the zero-lag
/// (main lobe) value and indices 1..N-1 are the sidelobe values for
/// increasing lag.
///
/// For a Barker code of length N, the zero-lag value equals N and all
/// sidelobe values have absolute magnitude at most 1.
pub fn autocorrelation(code: &[i8]) -> Vec<i64> {
    let n = code.len();
    let mut result = Vec::with_capacity(n);
    for lag in 0..n {
        let mut sum: i64 = 0;
        for i in 0..(n - lag) {
            sum += (code[i] as i64) * (code[i + lag] as i64);
        }
        result.push(sum);
    }
    result
}

/// Computes the peak sidelobe ratio (PSR) in decibels.
///
/// PSR = 20 * log10(main_lobe / max_sidelobe)
///
/// The main lobe is the zero-lag autocorrelation value, and the max
/// sidelobe is the largest absolute value among all non-zero lags.
/// Returns `f64::INFINITY` if all sidelobes are zero.
pub fn peak_sidelobe_ratio(code: &[i8]) -> f64 {
    let ac = autocorrelation(code);
    if ac.len() <= 1 {
        return f64::INFINITY;
    }
    let main_lobe = ac[0].abs() as f64;
    let max_sidelobe = ac[1..]
        .iter()
        .map(|v| v.abs())
        .max()
        .unwrap_or(0) as f64;
    if max_sidelobe == 0.0 {
        return f64::INFINITY;
    }
    20.0 * (main_lobe / max_sidelobe).log10()
}

/// Checks whether a bipolar (+1/-1) code has the Barker property:
/// all aperiodic autocorrelation sidelobes have absolute magnitude at most 1.
///
/// Returns `true` if the code satisfies the sidelobe constraint, `false` otherwise.
/// An empty or single-element code trivially satisfies the property.
pub fn is_barker(code: &[i8]) -> bool {
    let ac = autocorrelation(code);
    ac[1..].iter().all(|&v| v.abs() <= 1)
}

/// Generates a BPSK-modulated Barker waveform.
///
/// Each chip of the Barker code is mapped to `carrier_samples_per_chip`
/// samples at amplitude +1.0 or -1.0. The resulting waveform has length
/// `code.length() * carrier_samples_per_chip`.
///
/// # Arguments
///
/// * `code` - The Barker code to modulate
/// * `carrier_samples_per_chip` - Number of output samples per code chip
///
/// # Example
///
/// ```rust
/// use r4w_core::barker_code::{BarkerCode, barker_modulate};
///
/// let waveform = barker_modulate(&BarkerCode::Barker5, 4);
/// assert_eq!(waveform.len(), 20); // 5 chips * 4 samples/chip
/// ```
pub fn barker_modulate(code: &BarkerCode, carrier_samples_per_chip: usize) -> Vec<f64> {
    let seq = code.sequence();
    let mut output = Vec::with_capacity(seq.len() * carrier_samples_per_chip);
    for &chip in seq {
        let val = chip as f64;
        for _ in 0..carrier_samples_per_chip {
            output.push(val);
        }
    }
    output
}

/// Performs matched filter correlation of a signal against a Barker code.
///
/// Correlates the input signal with a BPSK-modulated reference waveform
/// built from the given Barker code. The output has the same length as
/// the input signal, with each element representing the correlation value
/// at that sample offset. The correlation peak occurs where the Barker
/// code aligns with a matching pattern in the signal.
///
/// # Arguments
///
/// * `signal` - Input signal samples
/// * `code` - The Barker code to correlate against
/// * `samples_per_chip` - Number of signal samples per code chip
///
/// # Example
///
/// ```rust
/// use r4w_core::barker_code::{BarkerCode, barker_modulate, barker_correlate};
///
/// let code = BarkerCode::Barker7;
/// let waveform = barker_modulate(&code, 4);
///
/// // Embed waveform in noise-free signal with leading zeros
/// let mut signal = vec![0.0; 20];
/// signal.extend_from_slice(&waveform);
/// signal.extend_from_slice(&[0.0; 20]);
///
/// let corr = barker_correlate(&signal, &code, 4);
/// // Peak should be at the end of the embedded waveform
/// let peak_idx = corr.iter()
///     .enumerate()
///     .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
///     .unwrap().0;
/// assert_eq!(peak_idx, 20 + waveform.len() - 1);
/// ```
pub fn barker_correlate(signal: &[f64], code: &BarkerCode, samples_per_chip: usize) -> Vec<f64> {
    let reference = barker_modulate(code, samples_per_chip);
    let ref_len = reference.len();
    let sig_len = signal.len();
    let mut output = vec![0.0; sig_len];

    for i in 0..sig_len {
        let mut sum = 0.0;
        // Matched filter: correlate reference ending at position i
        // output[i] = sum_{j} signal[i - ref_len + 1 + j] * reference[j]
        for j in 0..ref_len {
            let sig_idx = (i + 1) as isize - ref_len as isize + j as isize;
            if sig_idx >= 0 && (sig_idx as usize) < sig_len {
                sum += signal[sig_idx as usize] * reference[j];
            }
        }
        output[i] = sum;
    }

    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_barker_sequences() {
        // Verify each known Barker code sequence against published values
        assert_eq!(BarkerCode::Barker2.sequence(), &[1, -1]);
        assert_eq!(BarkerCode::Barker3.sequence(), &[1, 1, -1]);
        assert_eq!(BarkerCode::Barker4.sequence(), &[1, 1, -1, 1]);
        assert_eq!(BarkerCode::Barker5.sequence(), &[1, 1, 1, -1, 1]);
        assert_eq!(BarkerCode::Barker7.sequence(), &[1, 1, 1, -1, -1, 1, -1]);
        assert_eq!(
            BarkerCode::Barker11.sequence(),
            &[1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1]
        );
        assert_eq!(
            BarkerCode::Barker13.sequence(),
            &[1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1]
        );
    }

    #[test]
    fn test_barker_lengths() {
        assert_eq!(BarkerCode::Barker2.length(), 2);
        assert_eq!(BarkerCode::Barker3.length(), 3);
        assert_eq!(BarkerCode::Barker4.length(), 4);
        assert_eq!(BarkerCode::Barker5.length(), 5);
        assert_eq!(BarkerCode::Barker7.length(), 7);
        assert_eq!(BarkerCode::Barker11.length(), 11);
        assert_eq!(BarkerCode::Barker13.length(), 13);
    }

    #[test]
    fn test_autocorrelation_barker13() {
        let seq = BarkerCode::Barker13.sequence();
        let ac = autocorrelation(seq);

        // Zero-lag autocorrelation must equal code length
        assert_eq!(ac[0], 13);

        // All sidelobes must have absolute value <= 1
        for (lag, &val) in ac.iter().enumerate().skip(1) {
            assert!(
                val.abs() <= 1,
                "Barker-13 sidelobe at lag {} is {}, expected |val| <= 1",
                lag,
                val
            );
        }

        // Barker-13 specific sidelobe pattern: alternating 0 and 1
        let expected_sidelobes: [i64; 12] = [0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1];
        for (i, &expected) in expected_sidelobes.iter().enumerate() {
            assert_eq!(
                ac[i + 1], expected,
                "Barker-13 autocorrelation at lag {} is {}, expected {}",
                i + 1, ac[i + 1], expected
            );
        }
    }

    #[test]
    fn test_sidelobe_property() {
        // All known Barker codes must satisfy |sidelobe| <= 1
        for code in BarkerCode::all() {
            let seq = code.sequence();
            let ac = autocorrelation(seq);

            // Main lobe equals code length
            assert_eq!(ac[0], code.length() as i64);

            // Sidelobes bounded by 1
            for (lag, &val) in ac.iter().enumerate().skip(1) {
                assert!(
                    val.abs() <= 1,
                    "Barker-{} sidelobe at lag {} is {}, expected |val| <= 1",
                    code.length(),
                    lag,
                    val
                );
            }
        }
    }

    #[test]
    fn test_psr() {
        // PSR for Barker-13: 20*log10(13/1) ~ 22.28 dB
        let psr = peak_sidelobe_ratio(BarkerCode::Barker13.sequence());
        let expected = 20.0 * (13.0_f64).log10(); // 22.279 dB
        assert!(
            (psr - expected).abs() < 0.01,
            "Barker-13 PSR = {:.3} dB, expected {:.3} dB",
            psr,
            expected
        );

        // PSR for Barker-7: 20*log10(7/1) ~ 16.90 dB
        let psr7 = peak_sidelobe_ratio(BarkerCode::Barker7.sequence());
        let expected7 = 20.0 * (7.0_f64).log10();
        assert!(
            (psr7 - expected7).abs() < 0.01,
            "Barker-7 PSR = {:.3} dB, expected {:.3} dB",
            psr7,
            expected7
        );
    }

    #[test]
    fn test_is_barker() {
        // All known Barker codes must pass
        for code in BarkerCode::all() {
            assert!(
                is_barker(code.sequence()),
                "Barker-{} should be recognized as a Barker code",
                code.length()
            );
        }

        // A non-Barker sequence should fail
        let not_barker = [1i8, 1, 1, 1, 1, 1]; // length 6, all +1
        assert!(
            !is_barker(&not_barker),
            "All-ones sequence should not be a Barker code"
        );
    }

    #[test]
    fn test_generate_known() {
        // All known lengths should return the correct sequence
        let known_lengths = [2, 3, 4, 5, 7, 11, 13];
        for &len in &known_lengths {
            let code = generate_barker(len);
            assert!(code.is_some(), "Barker code of length {} should exist", len);
            let code = code.unwrap();
            assert_eq!(code.len(), len);
            assert!(
                is_barker(&code),
                "Generated length-{} code should have Barker property",
                len
            );
        }
    }

    #[test]
    fn test_generate_unknown() {
        // Lengths without known Barker codes should return None
        let invalid_lengths = [0, 1, 6, 8, 9, 10, 12, 14, 15, 100];
        for &len in &invalid_lengths {
            assert!(
                generate_barker(len).is_none(),
                "No Barker code should exist for length {}",
                len
            );
        }
    }

    #[test]
    fn test_modulate_correlate() {
        let code = BarkerCode::Barker13;
        let spc = 8; // samples per chip
        let waveform = barker_modulate(&code, spc);

        // Waveform length check
        assert_eq!(waveform.len(), 13 * spc);

        // Embed waveform with leading and trailing silence
        let lead = 50;
        let trail = 50;
        let mut signal = vec![0.0; lead];
        signal.extend_from_slice(&waveform);
        signal.extend_from_slice(&vec![0.0; trail]);

        let corr = barker_correlate(&signal, &code, spc);
        assert_eq!(corr.len(), signal.len());

        // Find the peak
        let (peak_idx, peak_val) = corr
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();

        // Peak should be at lead + waveform_len - 1 (matched filter aligns at end)
        assert_eq!(
            peak_idx,
            lead + waveform.len() - 1,
            "Peak at index {}, expected {}",
            peak_idx,
            lead + waveform.len() - 1
        );

        // Peak value should equal code_length * samples_per_chip
        let expected_peak = (code.length() * spc) as f64;
        assert!(
            (*peak_val - expected_peak).abs() < 1e-9,
            "Peak value = {}, expected {}",
            peak_val,
            expected_peak
        );

        // All non-peak values should be significantly smaller
        for (i, &val) in corr.iter().enumerate() {
            if i != peak_idx {
                assert!(
                    val.abs() < expected_peak,
                    "Non-peak value at {} = {} should be < {}",
                    i,
                    val,
                    expected_peak
                );
            }
        }
    }

    #[test]
    fn test_all_codes() {
        let all = BarkerCode::all();
        assert_eq!(all.len(), 7);

        // Verify ordering by length
        let lengths: Vec<usize> = all.iter().map(|c| c.length()).collect();
        assert_eq!(lengths, vec![2, 3, 4, 5, 7, 11, 13]);

        // Verify each code is unique
        for i in 0..all.len() {
            for j in (i + 1)..all.len() {
                assert_ne!(
                    all[i].sequence(),
                    all[j].sequence(),
                    "Codes at indices {} and {} should be different",
                    i,
                    j
                );
            }
        }

        // Verify all sequences contain only +1 and -1
        for code in &all {
            for &chip in code.sequence() {
                assert!(
                    chip == 1 || chip == -1,
                    "Barker-{} contains invalid chip value {}",
                    code.length(),
                    chip
                );
            }
        }
    }
}
