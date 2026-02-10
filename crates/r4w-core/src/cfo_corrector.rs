//! CFO Corrector — Carrier Frequency Offset correction
//!
//! Rotates IQ samples to remove a known or estimated frequency offset.
//! Provides both fixed-frequency correction (when offset is known from
//! a prior estimation stage) and adaptive decision-directed correction
//! (which jointly estimates and removes the offset). A standalone
//! autocorrelation-based fine CFO estimator is also included.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cfo_corrector::CfoCorrector;
//! use std::f64::consts::PI;
//!
//! let sample_rate = 1000.0;
//! let cfo_hz = 50.0;
//!
//! // Generate a tone with 50 Hz offset
//! let signal: Vec<(f64, f64)> = (0..256)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let phase = 2.0 * PI * cfo_hz * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! // Correct the offset — output should be near DC
//! let mut corrector = CfoCorrector::new(cfo_hz, sample_rate);
//! let corrected = corrector.correct(&signal);
//! assert!((corrected[100].1).abs() < 0.05);
//! ```

use std::f64::consts::PI;

/// Fixed-frequency CFO corrector.
///
/// Multiplies each IQ sample by `exp(-j * 2 * pi * freq_offset * n / fs)`,
/// maintaining continuous phase across successive calls to [`correct`].
#[derive(Debug, Clone)]
pub struct CfoCorrector {
    /// Frequency offset to remove, in Hz.
    freq_offset: f64,
    /// Current phase accumulator (radians).
    phase: f64,
    /// Sample rate in Hz.
    sample_rate: f64,
}

impl CfoCorrector {
    /// Create a new CFO corrector.
    ///
    /// - `freq_offset_hz`: frequency offset to remove (positive = signal is
    ///   above nominal, negative = below).
    /// - `sample_rate`: sample rate of the input signal in Hz.
    pub fn new(freq_offset_hz: f64, sample_rate: f64) -> Self {
        Self {
            freq_offset: freq_offset_hz,
            phase: 0.0,
            sample_rate,
        }
    }

    /// Correct a block of IQ samples by removing the configured frequency offset.
    ///
    /// Each sample `(i, q)` is multiplied by `exp(-j * phase)` where the phase
    /// advances by `2 * pi * freq_offset / sample_rate` per sample. Phase is
    /// continuous across successive calls.
    pub fn correct(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let phase_step = -2.0 * PI * self.freq_offset / self.sample_rate;
        let mut output = Vec::with_capacity(input.len());

        for &(i, q) in input {
            let cos_p = self.phase.cos();
            let sin_p = self.phase.sin();
            // Complex multiply: (i + jq) * (cos_p + j*sin_p)
            output.push((i * cos_p - q * sin_p, i * sin_p + q * cos_p));

            self.phase += phase_step;
            // Keep phase in [-pi, pi) to avoid precision loss
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
        }

        output
    }

    /// Update the frequency offset without resetting the phase accumulator.
    ///
    /// Useful when a new CFO estimate arrives mid-stream.
    pub fn set_frequency(&mut self, freq_hz: f64) {
        self.freq_offset = freq_hz;
    }

    /// Return the current frequency offset in Hz.
    pub fn frequency(&self) -> f64 {
        self.freq_offset
    }

    /// Return the current phase accumulator value in radians.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Reset the phase accumulator to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// One-shot CFO correction without maintaining state.
    ///
    /// Convenience function when you have a complete block and do not need
    /// phase continuity across calls.
    pub fn correct_static(
        input: &[(f64, f64)],
        freq_offset_hz: f64,
        sample_rate: f64,
    ) -> Vec<(f64, f64)> {
        let mut corrector = CfoCorrector::new(freq_offset_hz, sample_rate);
        corrector.correct(input)
    }
}

/// Adaptive (decision-directed) CFO corrector.
///
/// Uses a second-order phase-locked loop to jointly estimate and remove
/// carrier frequency offset. The loop drives the residual phase error
/// toward zero using QPSK-style decision-directed feedback (i.e., it
/// assumes the signal lies on the unit circle in one of four quadrants).
#[derive(Debug, Clone)]
pub struct AdaptiveCfoCorrector {
    /// Current phase accumulator (radians).
    phase: f64,
    /// Current frequency estimate (radians per sample).
    freq_est: f64,
    /// Proportional (phase) gain.
    alpha: f64,
    /// Integral (frequency) gain.
    beta: f64,
    /// Sample rate in Hz.
    sample_rate: f64,
}

impl AdaptiveCfoCorrector {
    /// Create a new adaptive CFO corrector.
    ///
    /// `loop_bandwidth` controls the trade-off between tracking speed and
    /// noise rejection, expressed as a fraction of the sample rate.
    /// Typical values: 0.001 to 0.05.
    ///
    /// `sample_rate` is the sample rate in Hz (used for converting the
    /// internal estimate to Hz via [`estimated_offset`]).
    pub fn new(loop_bandwidth: f64, sample_rate: f64) -> Self {
        // Second-order loop gains derived from loop bandwidth.
        // Damping factor = 1/sqrt(2) (critically damped).
        let bw_norm = loop_bandwidth;
        let denom = 1.0 + 2.0 * 0.707 * bw_norm + bw_norm * bw_norm;
        let alpha = (4.0 * 0.707 * bw_norm) / denom;
        let beta = (4.0 * bw_norm * bw_norm) / denom;

        Self {
            phase: 0.0,
            freq_est: 0.0,
            alpha,
            beta,
            sample_rate,
        }
    }

    /// Process a block of IQ samples, correcting CFO adaptively.
    ///
    /// Returns corrected samples. After processing, [`estimated_offset`]
    /// gives the current CFO estimate.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());

        for &(i, q) in input {
            // Apply current correction
            let cos_p = (-self.phase).cos();
            let sin_p = (-self.phase).sin();
            let ci = i * cos_p - q * sin_p;
            let cq = i * sin_p + q * cos_p;

            output.push((ci, cq));

            // Decision-directed phase error: use sign of I and Q as decision
            // (equivalent to nearest QPSK point), then compute cross-product
            // phase detector.
            let di = if ci >= 0.0 { 1.0 } else { -1.0 };
            let dq = if cq >= 0.0 { 1.0 } else { -1.0 };
            let phase_error = ci * dq - cq * di;

            // Update loop
            self.freq_est += self.beta * phase_error;
            self.phase += self.alpha * phase_error + self.freq_est;

            // Wrap phase
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
        }

        output
    }

    /// Return the current estimated CFO in Hz.
    ///
    /// Converts from internal radians-per-sample to Hz using the sample rate.
    pub fn estimated_offset(&self) -> f64 {
        self.freq_est * self.sample_rate / (2.0 * PI)
    }
}

/// Autocorrelation-based fine CFO estimation.
///
/// Computes the lag-1 autocorrelation of `samples` and extracts the
/// frequency offset from the phase angle:
///
///   `f_cfo = atan2(Im(R), Re(R)) * fs / (2 * pi)`
///
/// where `R = sum( s[n] * conj(s[n-1]) )`.
///
/// Best used on a known preamble or pilot section where the signal
/// energy is roughly constant.
///
/// Returns the estimated CFO in Hz.
pub fn fine_cfo_estimate(samples: &[(f64, f64)], sample_rate: f64) -> f64 {
    if samples.len() < 2 {
        return 0.0;
    }

    let mut re_sum = 0.0;
    let mut im_sum = 0.0;

    for n in 1..samples.len() {
        let (i1, q1) = samples[n];
        let (i0, q0) = samples[n - 1];
        // s[n] * conj(s[n-1]) = (i1 + jq1)(i0 - jq0)
        re_sum += i1 * i0 + q1 * q0;
        im_sum += q1 * i0 - i1 * q0;
    }

    let angle = im_sum.atan2(re_sum);
    angle * sample_rate / (2.0 * PI)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    /// Generate a complex tone at the given frequency.
    fn tone(freq_hz: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq_hz * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_zero_offset_passthrough() {
        let input = tone(0.0, 1000.0, 128);
        let mut corrector = CfoCorrector::new(0.0, 1000.0);
        let output = corrector.correct(&input);

        for (i, (oi, oq)) in output.iter().enumerate() {
            assert!(
                (oi - input[i].0).abs() < EPSILON,
                "I mismatch at sample {i}"
            );
            assert!(
                (oq - input[i].1).abs() < EPSILON,
                "Q mismatch at sample {i}"
            );
        }
    }

    #[test]
    fn test_known_offset_correction() {
        let fs = 1000.0;
        let cfo = 50.0;
        let signal = tone(cfo, fs, 256);

        let mut corrector = CfoCorrector::new(cfo, fs);
        let corrected = corrector.correct(&signal);

        // After correction the signal should be near DC (I ~1, Q ~0)
        for (n, &(ci, cq)) in corrected.iter().enumerate() {
            assert!(
                (ci - 1.0).abs() < 1e-6,
                "I not near 1.0 at sample {n}: {ci}"
            );
            assert!(cq.abs() < 1e-6, "Q not near 0.0 at sample {n}: {cq}");
        }
    }

    #[test]
    fn test_continuous_phase() {
        let fs = 1000.0;
        let cfo = 100.0;
        let signal = tone(cfo, fs, 512);

        let mut corrector = CfoCorrector::new(cfo, fs);

        // Process in two halves
        let out1 = corrector.correct(&signal[..256]);
        let out2 = corrector.correct(&signal[256..]);

        // Combine and check continuity — all samples should be near DC
        let combined: Vec<(f64, f64)> = out1.into_iter().chain(out2).collect();
        for (n, &(ci, cq)) in combined.iter().enumerate() {
            assert!(
                (ci - 1.0).abs() < 1e-5,
                "I discontinuity at sample {n}: {ci}"
            );
            assert!(
                cq.abs() < 1e-5,
                "Q discontinuity at sample {n}: {cq}"
            );
        }
    }

    #[test]
    fn test_set_frequency() {
        let fs = 1000.0;
        let mut corrector = CfoCorrector::new(50.0, fs);
        assert!((corrector.frequency() - 50.0).abs() < EPSILON);

        corrector.set_frequency(100.0);
        assert!((corrector.frequency() - 100.0).abs() < EPSILON);

        // Phase should not have been reset
        // Process a few samples first to accumulate phase
        let mut c2 = CfoCorrector::new(50.0, fs);
        let _ = c2.correct(&tone(50.0, fs, 10));
        let phase_before = c2.phase();
        c2.set_frequency(100.0);
        assert!((c2.phase() - phase_before).abs() < EPSILON);
    }

    #[test]
    fn test_reset() {
        let fs = 1000.0;
        let mut corrector = CfoCorrector::new(50.0, fs);

        // Accumulate some phase (use 73 samples so phase does not wrap to zero)
        let _ = corrector.correct(&tone(50.0, fs, 73));
        assert!(corrector.phase().abs() > EPSILON);

        corrector.reset();
        assert!(corrector.phase().abs() < EPSILON);
    }

    #[test]
    fn test_static_correction() {
        let fs = 1000.0;
        let cfo = 25.0;
        let signal = tone(cfo, fs, 256);

        let corrected = CfoCorrector::correct_static(&signal, cfo, fs);

        for (n, &(ci, cq)) in corrected.iter().enumerate() {
            assert!(
                (ci - 1.0).abs() < 1e-6,
                "Static I not near 1.0 at sample {n}: {ci}"
            );
            assert!(
                cq.abs() < 1e-6,
                "Static Q not near 0.0 at sample {n}: {cq}"
            );
        }
    }

    #[test]
    fn test_adaptive_tracks_cfo() {
        let fs = 10000.0;
        let cfo = 50.0;

        // Generate a QPSK-like signal with CFO: each symbol at (1,1)/sqrt(2)
        // rotated by the CFO.
        let n = 4000;
        let base_i = 1.0 / 2.0_f64.sqrt();
        let base_q = 1.0 / 2.0_f64.sqrt();
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let t = k as f64 / fs;
                let rot = 2.0 * PI * cfo * t;
                let cos_r = rot.cos();
                let sin_r = rot.sin();
                (
                    base_i * cos_r - base_q * sin_r,
                    base_i * sin_r + base_q * cos_r,
                )
            })
            .collect();

        let mut adaptive = AdaptiveCfoCorrector::new(0.01, fs);
        let _corrected = adaptive.process(&signal);

        // After convergence the estimate should be close to the true CFO
        let est = adaptive.estimated_offset();
        assert!(
            (est - cfo).abs() < 5.0,
            "Adaptive estimate {est} Hz not close to true {cfo} Hz"
        );
    }

    #[test]
    fn test_fine_estimate() {
        let fs = 10000.0;
        let cfo = 123.0;
        let signal = tone(cfo, fs, 1024);

        let est = fine_cfo_estimate(&signal, fs);
        assert!(
            (est - cfo).abs() < 1.0,
            "Fine estimate {est} Hz not close to true {cfo} Hz"
        );
    }

    #[test]
    fn test_roundtrip_apply_remove() {
        let fs = 1000.0;
        let cfo = 75.0;

        // Start with a DC signal
        let dc: Vec<(f64, f64)> = vec![(1.0, 0.0); 256];

        // Apply CFO (positive rotation)
        let with_cfo: Vec<(f64, f64)> = dc
            .iter()
            .enumerate()
            .map(|(n, &(i, q))| {
                let phase = 2.0 * PI * cfo * n as f64 / fs;
                let cos_p = phase.cos();
                let sin_p = phase.sin();
                (i * cos_p - q * sin_p, i * sin_p + q * cos_p)
            })
            .collect();

        // Remove CFO
        let mut corrector = CfoCorrector::new(cfo, fs);
        let recovered = corrector.correct(&with_cfo);

        // Should be back to DC
        for (n, &(ri, rq)) in recovered.iter().enumerate() {
            assert!(
                (ri - 1.0).abs() < 1e-6,
                "Roundtrip I at {n}: {ri}"
            );
            assert!(rq.abs() < 1e-6, "Roundtrip Q at {n}: {rq}");
        }
    }

    #[test]
    fn test_empty_input() {
        let mut corrector = CfoCorrector::new(100.0, 1000.0);
        let output = corrector.correct(&[]);
        assert!(output.is_empty());

        let static_out = CfoCorrector::correct_static(&[], 100.0, 1000.0);
        assert!(static_out.is_empty());

        let mut adaptive = AdaptiveCfoCorrector::new(0.01, 1000.0);
        let adaptive_out = adaptive.process(&[]);
        assert!(adaptive_out.is_empty());

        let est = fine_cfo_estimate(&[], 1000.0);
        assert!(est.abs() < EPSILON);
    }
}
