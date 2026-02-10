//! Crest Factor Reduction (CFR)
//!
//! Reduces the Peak-to-Average Power Ratio (PAPR) of OFDM and multi-carrier
//! signals so that power amplifiers can operate closer to saturation without
//! excessive distortion. High PAPR forces large back-off, wasting transmit
//! power; CFR trades a small amount of Error Vector Magnitude (EVM) for
//! significantly improved amplifier efficiency.
//!
//! Three complementary algorithms are provided:
//!
//! - [`CfrClipping`] -- simple hard clipping of peaks that exceed the target
//! - [`CfrPeakWindowing`] -- smooth peak cancellation using a raised-cosine
//!   window, which preserves spectral shape better than hard clipping
//! - [`CfrIterative`] -- iterative clip-and-filter that re-grows the spectrum
//!   after each clip pass to maintain occupied bandwidth
//!
//! The helper functions [`crest_factor`] and [`papr_db`] compute raw metrics,
//! and [`measure_cfr_stats`] compares an input/output pair to report PAPR
//! reduction and EVM degradation.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::crest_factor_reduction::{papr_db, CfrClipping};
//!
//! // Build a multi-tone signal with high PAPR
//! let n = 512;
//! let freqs = [1.0_f64, 3.0, 7.0, 13.0, 19.0, 29.0, 37.0, 43.0];
//! let signal: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / n as f64;
//!         let re: f64 = freqs.iter()
//!             .map(|&f| (2.0 * std::f64::consts::PI * f * t).cos())
//!             .sum();
//!         let im: f64 = freqs.iter()
//!             .map(|&f| (2.0 * std::f64::consts::PI * f * t).sin())
//!             .sum();
//!         (re, im)
//!     })
//!     .collect();
//!
//! let before = papr_db(&signal);
//! let mut cfr = CfrClipping::new(3.0);
//! let reduced = cfr.process(&signal);
//! let after = papr_db(&reduced);
//! assert!(after < before, "PAPR should decrease after clipping");
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

/// Magnitude of a complex sample represented as `(re, im)`.
#[inline]
fn mag(s: &(f64, f64)) -> f64 {
    (s.0 * s.0 + s.1 * s.1).sqrt()
}

/// Squared magnitude.
#[inline]
fn mag_sq(s: &(f64, f64)) -> f64 {
    s.0 * s.0 + s.1 * s.1
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

/// Compute the crest factor of a signal.
///
/// CF = peak\_magnitude / rms\_magnitude
///
/// Returns 0.0 for empty signals or signals with zero RMS.
pub fn crest_factor(signal: &[(f64, f64)]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }

    let peak = signal.iter().map(|s| mag(s)).fold(0.0_f64, f64::max);
    let mean_sq = signal.iter().map(|s| mag_sq(s)).sum::<f64>() / signal.len() as f64;
    let rms = mean_sq.sqrt();

    if rms == 0.0 {
        return 0.0;
    }

    peak / rms
}

/// Compute PAPR in decibels.
///
/// PAPR\_dB = 20 * log10(CF)
///
/// Returns 0.0 for empty signals or signals with zero RMS.
pub fn papr_db(signal: &[(f64, f64)]) -> f64 {
    let cf = crest_factor(signal);
    if cf <= 0.0 {
        return 0.0;
    }
    20.0 * cf.log10()
}

// ---------------------------------------------------------------------------
// CfrClipping
// ---------------------------------------------------------------------------

/// Hard-clipping crest factor reduction.
///
/// Any sample whose magnitude exceeds the computed threshold is scaled down
/// to exactly the threshold level while preserving its phase.  This is the
/// simplest and lowest-latency CFR method but introduces out-of-band
/// spectral regrowth.
#[derive(Debug, Clone)]
pub struct CfrClipping {
    target_papr_db: f64,
}

impl CfrClipping {
    /// Create a new hard-clipping CFR with the given target PAPR in dB.
    pub fn new(target_papr_db: f64) -> Self {
        Self { target_papr_db }
    }

    /// Clip peaks exceeding the target PAPR.
    ///
    /// The clipping threshold is derived from the signal's RMS level and the
    /// configured `target_papr_db`.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if input.is_empty() {
            return Vec::new();
        }

        let mean_sq = input.iter().map(|s| mag_sq(s)).sum::<f64>() / input.len() as f64;
        let rms = mean_sq.sqrt();

        if rms == 0.0 {
            return input.to_vec();
        }

        // threshold = rms * 10^(target_papr_db / 20)
        let threshold = rms * 10.0_f64.powf(self.target_papr_db / 20.0);

        input
            .iter()
            .map(|s| {
                let m = mag(s);
                if m > threshold {
                    let scale = threshold / m;
                    (s.0 * scale, s.1 * scale)
                } else {
                    *s
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// CfrPeakWindowing
// ---------------------------------------------------------------------------

/// Peak windowing / cancellation CFR.
///
/// Identifies peaks that exceed the threshold, generates a raised-cosine
/// cancellation pulse centred on each peak, and subtracts it from the
/// signal.  The smooth window limits spectral regrowth compared to hard
/// clipping.
#[derive(Debug, Clone)]
pub struct CfrPeakWindowing {
    target_papr_db: f64,
    window_len: usize,
    window: Vec<f64>,
}

impl CfrPeakWindowing {
    /// Create a peak-windowing CFR.
    ///
    /// * `target_papr_db` -- desired PAPR in dB
    /// * `window_len` -- length of the raised-cosine cancellation window
    ///   (must be odd; will be forced odd by adding 1 if even)
    pub fn new(target_papr_db: f64, window_len: usize) -> Self {
        let wl = if window_len % 2 == 0 {
            window_len + 1
        } else {
            window_len
        };

        // Raised-cosine (Hann) window
        let window: Vec<f64> = (0..wl)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (wl - 1) as f64).cos()))
            .collect();

        Self {
            target_papr_db,
            window_len: wl,
            window,
        }
    }

    /// Process the signal by subtracting windowed peak-cancellation pulses.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if input.is_empty() {
            return Vec::new();
        }

        let mean_sq = input.iter().map(|s| mag_sq(s)).sum::<f64>() / input.len() as f64;
        let rms = mean_sq.sqrt();

        if rms == 0.0 {
            return input.to_vec();
        }

        let threshold = rms * 10.0_f64.powf(self.target_papr_db / 20.0);
        let half = self.window_len / 2;

        let mut output: Vec<(f64, f64)> = input.to_vec();

        // Find and cancel peaks
        for i in 0..input.len() {
            let m = mag(&output[i]);
            if m > threshold {
                // Amount by which the sample exceeds the threshold
                let excess = m - threshold;
                // Unit-vector in the direction of the sample
                let dir_re = output[i].0 / m;
                let dir_im = output[i].1 / m;

                // Subtract a windowed cancellation pulse
                for j in 0..self.window_len {
                    let idx = i as isize - half as isize + j as isize;
                    if idx >= 0 && (idx as usize) < output.len() {
                        let w = self.window[j] * excess;
                        let k = idx as usize;
                        output[k].0 -= w * dir_re;
                        output[k].1 -= w * dir_im;
                    }
                }
            }
        }

        output
    }
}

// ---------------------------------------------------------------------------
// CfrIterative
// ---------------------------------------------------------------------------

/// Iterative clipping and filtering CFR.
///
/// Each iteration hard-clips the signal then applies a simple low-pass FIR
/// filter (sinc-windowed) to suppress spectral regrowth.  Multiple passes
/// progressively reduce PAPR while keeping the spectrum contained.
#[derive(Debug, Clone)]
pub struct CfrIterative {
    target_papr_db: f64,
    num_iterations: usize,
    filter_coeffs: Vec<f64>,
}

impl CfrIterative {
    /// Create an iterative clip-and-filter CFR.
    ///
    /// * `target_papr_db` -- desired PAPR in dB
    /// * `num_iterations` -- number of clip+filter passes
    /// * `filter_len` -- length of the lowpass FIR used after each clip
    ///   (must be odd; will be forced odd if even)
    pub fn new(target_papr_db: f64, num_iterations: usize, filter_len: usize) -> Self {
        let fl = if filter_len % 2 == 0 {
            filter_len + 1
        } else {
            filter_len
        };

        // Design a simple windowed-sinc lowpass filter (cutoff at 0.8 * Nyquist
        // to preserve the main signal while suppressing clip-induced regrowth).
        let cutoff = 0.8;
        let half = fl / 2;
        let mut coeffs: Vec<f64> = (0..fl)
            .map(|i| {
                let n = i as f64 - half as f64;
                let sinc = if n.abs() < 1e-12 {
                    cutoff
                } else {
                    (PI * cutoff * n).sin() / (PI * n)
                };
                // Blackman window
                let w = 0.42 - 0.5 * (2.0 * PI * i as f64 / (fl - 1) as f64).cos()
                    + 0.08 * (4.0 * PI * i as f64 / (fl - 1) as f64).cos();
                sinc * w
            })
            .collect();

        // Normalise to unity gain at DC
        let sum: f64 = coeffs.iter().sum();
        if sum.abs() > 1e-12 {
            for c in &mut coeffs {
                *c /= sum;
            }
        }

        Self {
            target_papr_db,
            num_iterations,
            filter_coeffs: coeffs,
        }
    }

    /// Process the signal through repeated clip-and-filter passes.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if input.is_empty() {
            return Vec::new();
        }

        let mut signal = input.to_vec();

        for _ in 0..self.num_iterations {
            // --- Clip ---
            let mean_sq =
                signal.iter().map(|s| mag_sq(s)).sum::<f64>() / signal.len() as f64;
            let rms = mean_sq.sqrt();
            if rms == 0.0 {
                break;
            }
            let threshold = rms * 10.0_f64.powf(self.target_papr_db / 20.0);

            for s in &mut signal {
                let m = mag(s);
                if m > threshold {
                    let scale = threshold / m;
                    s.0 *= scale;
                    s.1 *= scale;
                }
            }

            // --- Filter ---
            signal = self.apply_filter(&signal);
        }

        signal
    }

    /// Apply the lowpass FIR filter (linear convolution, same-length output).
    fn apply_filter(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = signal.len();
        let half = self.filter_coeffs.len() / 2;
        let mut out = vec![(0.0, 0.0); n];

        for i in 0..n {
            let mut re = 0.0;
            let mut im = 0.0;
            for (j, &c) in self.filter_coeffs.iter().enumerate() {
                let idx = i as isize - half as isize + j as isize;
                if idx >= 0 && (idx as usize) < n {
                    let k = idx as usize;
                    re += c * signal[k].0;
                    im += c * signal[k].1;
                }
            }
            out[i] = (re, im);
        }

        out
    }
}

// ---------------------------------------------------------------------------
// CfrStats
// ---------------------------------------------------------------------------

/// Statistics comparing a signal before and after CFR processing.
#[derive(Debug, Clone)]
pub struct CfrStats {
    /// PAPR of the input signal in dB.
    pub input_papr_db: f64,
    /// PAPR of the output signal in dB.
    pub output_papr_db: f64,
    /// Error Vector Magnitude as a percentage.
    pub evm_percent: f64,
    /// Number of samples whose magnitude was reduced (clipped or cancelled).
    pub clipped_samples: usize,
}

/// Measure CFR statistics by comparing input and output signals.
///
/// The two slices must have the same length.  EVM is computed as:
///
/// EVM% = 100 * sqrt( sum(|error|^2) / sum(|input|^2) )
///
/// `clipped_samples` counts the number of samples where the output magnitude
/// is strictly less than the input magnitude (within floating-point tolerance).
pub fn measure_cfr_stats(input: &[(f64, f64)], output: &[(f64, f64)]) -> CfrStats {
    assert_eq!(
        input.len(),
        output.len(),
        "input and output must have the same length"
    );

    let input_papr = papr_db(input);
    let output_papr = papr_db(output);

    let mut error_power = 0.0;
    let mut signal_power = 0.0;
    let mut clipped = 0usize;

    for (inp, out) in input.iter().zip(output.iter()) {
        let dr = out.0 - inp.0;
        let di = out.1 - inp.1;
        error_power += dr * dr + di * di;
        signal_power += mag_sq(inp);

        if mag(out) < mag(inp) - 1e-12 {
            clipped += 1;
        }
    }

    let evm = if signal_power > 0.0 {
        100.0 * (error_power / signal_power).sqrt()
    } else {
        0.0
    };

    CfrStats {
        input_papr_db: input_papr,
        output_papr_db: output_papr,
        evm_percent: evm,
        clipped_samples: clipped,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Generate a complex sinusoid of length `n` with frequency index `k`.
    fn tone(n: usize, k: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * k * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Generate a multi-tone signal (sum of several complex sinusoids).
    fn multi_tone(n: usize, freqs: &[f64]) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let re: f64 = freqs.iter().map(|&f| (2.0 * PI * f * t).cos()).sum();
                let im: f64 = freqs.iter().map(|&f| (2.0 * PI * f * t).sin()).sum();
                (re, im)
            })
            .collect()
    }

    #[test]
    fn test_crest_factor_sine() {
        // A single complex sinusoid has a crest factor of 1.0 (constant
        // envelope) -- every sample has the same magnitude.
        let sig = tone(256, 3.0);
        let cf = crest_factor(&sig);
        assert!(
            (cf - 1.0).abs() < 1e-6,
            "Single tone CF should be ~1.0, got {}",
            cf
        );
    }

    #[test]
    fn test_papr_db() {
        // CF == 1.0 => PAPR == 0 dB
        let sig = tone(256, 5.0);
        let p = papr_db(&sig);
        assert!(
            p.abs() < 0.01,
            "Single tone PAPR should be ~0 dB, got {}",
            p
        );

        // Multi-tone signal should have PAPR > 0 dB
        let mt = multi_tone(256, &[1.0, 5.0, 11.0, 17.0]);
        let p2 = papr_db(&mt);
        assert!(p2 > 1.0, "Multi-tone PAPR should be > 1 dB, got {}", p2);
    }

    #[test]
    fn test_clipping_reduces_papr() {
        let sig = multi_tone(512, &[1.0, 3.0, 7.0, 13.0, 19.0]);
        let before = papr_db(&sig);

        let mut cfr = CfrClipping::new(3.0);
        let out = cfr.process(&sig);
        let after = papr_db(&out);

        assert!(
            after < before,
            "Clipping should reduce PAPR: before={:.2}, after={:.2}",
            before,
            after
        );
        // After clipping, the RMS drops slightly so the measured PAPR can
        // exceed the target by a small margin.  Allow up to 1.5 dB overshoot.
        assert!(
            after <= 3.0 + 1.5,
            "Clipped PAPR should be near target 3 dB, got {:.2}",
            after
        );
    }

    #[test]
    fn test_peak_windowing() {
        let sig = multi_tone(512, &[2.0, 5.0, 11.0, 17.0]);
        let before = papr_db(&sig);

        let mut cfr = CfrPeakWindowing::new(4.0, 31);
        let out = cfr.process(&sig);
        let after = papr_db(&out);

        assert!(
            after < before,
            "Peak windowing should reduce PAPR: before={:.2}, after={:.2}",
            before,
            after
        );
    }

    #[test]
    fn test_iterative_cfr() {
        let sig = multi_tone(512, &[1.0, 4.0, 9.0, 16.0, 25.0]);
        let before = papr_db(&sig);

        let mut cfr = CfrIterative::new(4.0, 3, 15);
        let out = cfr.process(&sig);
        let after = papr_db(&out);

        assert!(
            after < before,
            "Iterative CFR should reduce PAPR: before={:.2}, after={:.2}",
            before,
            after
        );
    }

    #[test]
    fn test_cfr_stats() {
        let sig = multi_tone(256, &[1.0, 5.0, 13.0]);
        let mut cfr = CfrClipping::new(3.0);
        let out = cfr.process(&sig);
        let stats = measure_cfr_stats(&sig, &out);

        assert!(
            stats.output_papr_db < stats.input_papr_db,
            "output PAPR ({:.2}) should be less than input ({:.2})",
            stats.output_papr_db,
            stats.input_papr_db
        );
        assert!(stats.evm_percent > 0.0, "EVM should be > 0 after clipping");
        assert!(
            stats.clipped_samples > 0,
            "Some samples should have been clipped"
        );
    }

    #[test]
    fn test_constant_signal() {
        // All samples have the same magnitude => CF = 1, PAPR = 0 dB.
        let sig: Vec<(f64, f64)> = (0..128).map(|_| (1.0, 0.0)).collect();
        let cf = crest_factor(&sig);
        assert!(
            (cf - 1.0).abs() < 1e-10,
            "Constant signal CF should be 1.0, got {}",
            cf
        );

        let p = papr_db(&sig);
        assert!(
            p.abs() < 1e-6,
            "Constant signal PAPR should be 0 dB, got {}",
            p
        );
    }

    #[test]
    fn test_already_below_target() {
        // Single tone has 0 dB PAPR, targeting 6 dB should leave it unchanged.
        let sig = tone(256, 4.0);

        let mut cfr = CfrClipping::new(6.0);
        let out = cfr.process(&sig);

        for (a, b) in sig.iter().zip(out.iter()) {
            assert!(
                (a.0 - b.0).abs() < 1e-12 && (a.1 - b.1).abs() < 1e-12,
                "Signal already below target should not be modified"
            );
        }
    }

    #[test]
    fn test_evm_measurement() {
        // Artificially construct input and output with known error.
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        // Each sample shifted by (0.1, 0.0) => error magnitude = 0.1
        let output: Vec<(f64, f64)> = vec![(0.9, 0.0); 100];

        let stats = measure_cfr_stats(&input, &output);

        // EVM = 100 * sqrt( sum(0.1^2 * 100) / sum(1.0^2 * 100) )
        //     = 100 * sqrt(0.01) = 100 * 0.1 = 10%
        assert!(
            (stats.evm_percent - 10.0).abs() < 0.01,
            "EVM should be 10%, got {:.4}",
            stats.evm_percent
        );
    }

    #[test]
    fn test_empty_input() {
        assert_eq!(crest_factor(&[]), 0.0);
        assert_eq!(papr_db(&[]), 0.0);

        let mut clip = CfrClipping::new(6.0);
        assert!(clip.process(&[]).is_empty());

        let mut pw = CfrPeakWindowing::new(6.0, 15);
        assert!(pw.process(&[]).is_empty());

        let mut iter = CfrIterative::new(6.0, 3, 15);
        assert!(iter.process(&[]).is_empty());
    }
}
