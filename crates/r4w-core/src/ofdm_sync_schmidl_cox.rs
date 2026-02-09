//! OFDM Schmidl-Cox Synchronization
//!
//! Performs OFDM symbol timing detection and coarse/fine carrier frequency
//! offset (CFO) estimation using the Schmidl & Cox algorithm. Exploits the
//! repetition structure in a specially designed preamble where the first
//! symbol uses only even-indexed subcarriers (creating a half-symbol repeat).
//!
//! Reference: T. M. Schmidl and D. C. Cox, "Robust Frequency and Timing
//! Synchronization for OFDM," IEEE Trans. Commun., vol. 45, no. 12, 1997.
//!
//! No direct GNU Radio standalone equivalent — functionality split across
//! `gr::digital::ofdm_sync_sc_cfb` + timing blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm_sync_schmidl_cox::{SchmidlCoxSync, generate_schmidl_cox_preamble};
//! use num_complex::Complex64;
//!
//! let fft_size = 64;
//! let cp_len = 16;
//! let (s1, s2) = generate_schmidl_cox_preamble(fft_size, 42);
//! let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.5);
//!
//! // Build a signal: some noise, then CP+S1, CP+S2
//! let mut signal = vec![Complex64::new(0.0, 0.0); 32];
//! // Add CP for S1 (last cp_len samples of S1)
//! signal.extend_from_slice(&s1[fft_size - cp_len..]);
//! signal.extend_from_slice(&s1);
//! // Add CP for S2
//! signal.extend_from_slice(&s2[fft_size - cp_len..]);
//! signal.extend_from_slice(&s2);
//! signal.extend(vec![Complex64::new(0.0, 0.0); 32]);
//!
//! let results = sync.process(&signal);
//! assert!(!results.is_empty());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Synchronization detection result.
#[derive(Debug, Clone)]
pub struct SyncResult {
    /// Sample index of detected symbol boundary.
    pub timing_offset: usize,
    /// Coarse (fractional) CFO estimate in subcarrier spacings.
    pub coarse_cfo: f64,
    /// Fine CFO estimate in subcarrier spacings (if known preamble provided).
    pub fine_cfo: f64,
    /// Peak value of the timing metric at detection.
    pub metric_peak: f64,
}

/// Schmidl-Cox OFDM synchronizer.
#[derive(Debug, Clone)]
pub struct SchmidlCoxSync {
    fft_size: usize,
    cp_len: usize,
    threshold: f64,
    /// Optional known second preamble symbol for fine CFO.
    known_s2: Option<Vec<Complex64>>,
}

impl SchmidlCoxSync {
    /// Create a new Schmidl-Cox synchronizer.
    ///
    /// * `fft_size` - OFDM FFT size (N)
    /// * `cp_len` - Cyclic prefix length in samples
    /// * `threshold` - Detection threshold for timing metric M(d) (0.0 to 1.0)
    pub fn new(fft_size: usize, cp_len: usize, threshold: f64) -> Self {
        Self {
            fft_size,
            cp_len,
            threshold,
            known_s2: None,
        }
    }

    /// Set known second preamble symbol for fine CFO estimation.
    pub fn set_known_preamble_s2(&mut self, s2: Vec<Complex64>) {
        self.known_s2 = Some(s2);
    }

    /// Compute the timing metric M(d) for each sample offset.
    ///
    /// M(d) = |P(d)|^2 / R(d)^2
    /// where P(d) = sum_{m=0}^{N/2-1} conj(r[d+m]) * r[d+m+N/2]
    ///       R(d) = sum_{m=0}^{N/2-1} |r[d+m+N/2]|^2
    pub fn timing_metric(&self, samples: &[Complex64]) -> Vec<f64> {
        let half = self.fft_size / 2;
        let len = samples.len();
        if len < self.fft_size {
            return vec![];
        }

        let num_metrics = len - self.fft_size + 1;
        let mut metrics = vec![0.0; num_metrics];

        for d in 0..num_metrics {
            let mut p = Complex64::new(0.0, 0.0);
            let mut r = 0.0;
            for m in 0..half {
                p += samples[d + m].conj() * samples[d + m + half];
                r += samples[d + m + half].norm_sqr();
            }
            let r_sq = r * r;
            if r_sq > 1e-30 {
                metrics[d] = p.norm_sqr() / r_sq;
            }
        }

        metrics
    }

    /// Estimate fractional CFO from the phase of P(d) at the given timing offset.
    ///
    /// epsilon_hat = (1/pi) * angle(P(d_opt))
    pub fn estimate_cfo(&self, samples: &[Complex64], timing_offset: usize) -> f64 {
        let half = self.fft_size / 2;
        if timing_offset + self.fft_size > samples.len() {
            return 0.0;
        }

        let mut p = Complex64::new(0.0, 0.0);
        for m in 0..half {
            p += samples[timing_offset + m].conj() * samples[timing_offset + m + half];
        }

        p.arg() / PI
    }

    /// Fine CFO estimation using cross-correlation with known second preamble.
    pub fn fine_cfo(
        &self,
        samples: &[Complex64],
        timing_offset: usize,
        known_preamble: &[Complex64],
    ) -> f64 {
        if known_preamble.len() != self.fft_size {
            return 0.0;
        }
        // The second preamble symbol starts after first symbol + CP
        let s2_start = timing_offset + self.fft_size + self.cp_len;
        if s2_start + self.fft_size > samples.len() {
            return 0.0;
        }

        let mut corr = Complex64::new(0.0, 0.0);
        for m in 0..self.fft_size {
            corr += samples[s2_start + m] * known_preamble[m].conj();
        }

        // Fine CFO in subcarrier spacings
        corr.arg() / (2.0 * PI)
    }

    /// Process a buffer of samples and detect preamble locations.
    pub fn process(&mut self, samples: &[Complex64]) -> Vec<SyncResult> {
        let metrics = self.timing_metric(samples);
        if metrics.is_empty() {
            return vec![];
        }

        let mut results = Vec::new();

        // Find peaks above threshold with plateau detection
        let mut in_plateau = false;
        let mut plateau_start = 0;
        let mut plateau_max = 0.0;
        let mut plateau_max_idx = 0;

        for (d, &m) in metrics.iter().enumerate() {
            if m > self.threshold {
                if !in_plateau {
                    in_plateau = true;
                    plateau_start = d;
                    plateau_max = m;
                    plateau_max_idx = d;
                } else if m > plateau_max {
                    plateau_max = m;
                    plateau_max_idx = d;
                }
            } else if in_plateau {
                in_plateau = false;
                // The timing point is at the end of the plateau (CP absorbed)
                let timing = plateau_max_idx;

                let coarse_cfo = self.estimate_cfo(samples, timing);

                let fine_cfo = if let Some(ref s2) = self.known_s2 {
                    self.fine_cfo(samples, timing, s2)
                } else {
                    0.0
                };

                results.push(SyncResult {
                    timing_offset: timing,
                    coarse_cfo,
                    fine_cfo,
                    metric_peak: plateau_max,
                });
            }
        }

        // Handle plateau that extends to end of buffer
        if in_plateau {
            let timing = plateau_max_idx;
            let coarse_cfo = self.estimate_cfo(samples, timing);
            let fine_cfo = if let Some(ref s2) = self.known_s2 {
                self.fine_cfo(samples, timing, s2)
            } else {
                0.0
            };
            results.push(SyncResult {
                timing_offset: timing,
                coarse_cfo,
                fine_cfo,
                metric_peak: plateau_max,
            });
        }

        results
    }
}

/// Generate a Schmidl-Cox two-symbol preamble.
///
/// S1: only even-indexed subcarriers carry PN symbols (creates half-symbol repetition).
/// S2: all subcarriers carry PN symbols (for fine CFO / channel estimation).
///
/// Returns (s1_time, s2_time) as time-domain OFDM symbols (no CP).
pub fn generate_schmidl_cox_preamble(
    fft_size: usize,
    seed: u64,
) -> (Vec<Complex64>, Vec<Complex64>) {
    // Simple PRNG for PN sequence (BPSK: +1/-1)
    let mut rng_state = seed;
    let mut next_pn = || -> f64 {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        if (rng_state >> 33) & 1 == 0 {
            1.0
        } else {
            -1.0
        }
    };

    // S1: even subcarriers only
    let mut s1_freq = vec![Complex64::new(0.0, 0.0); fft_size];
    for k in (0..fft_size).step_by(2) {
        if k == 0 {
            continue; // Skip DC
        }
        s1_freq[k] = Complex64::new(next_pn(), 0.0);
    }

    // S2: all subcarriers
    let mut s2_freq = vec![Complex64::new(0.0, 0.0); fft_size];
    for k in 1..fft_size {
        s2_freq[k] = Complex64::new(next_pn(), 0.0);
    }

    // Simple IDFT (no external FFT dependency)
    let s1_time = idft(&s1_freq);
    let s2_time = idft(&s2_freq);

    (s1_time, s2_time)
}

/// Simple inverse DFT for preamble generation.
fn idft(freq: &[Complex64]) -> Vec<Complex64> {
    let n = freq.len();
    let mut time = vec![Complex64::new(0.0, 0.0); n];
    let scale = 1.0 / (n as f64).sqrt();
    for t in 0..n {
        let mut sum = Complex64::new(0.0, 0.0);
        for k in 0..n {
            let phase = 2.0 * PI * (k * t) as f64 / n as f64;
            sum += freq[k] * Complex64::new(phase.cos(), phase.sin());
        }
        time[t] = sum * scale;
    }
    time
}

/// Simple forward DFT.
fn dft(time: &[Complex64]) -> Vec<Complex64> {
    let n = time.len();
    let mut freq = vec![Complex64::new(0.0, 0.0); n];
    let scale = 1.0 / (n as f64).sqrt();
    for k in 0..n {
        let mut sum = Complex64::new(0.0, 0.0);
        for t in 0..n {
            let phase = -2.0 * PI * (k * t) as f64 / n as f64;
            sum += time[t] * Complex64::new(phase.cos(), phase.sin());
        }
        freq[k] = sum * scale;
    }
    freq
}

#[cfg(test)]
mod tests {
    use super::*;

    fn add_cp(symbol: &[Complex64], cp_len: usize) -> Vec<Complex64> {
        let n = symbol.len();
        let mut out = Vec::with_capacity(n + cp_len);
        out.extend_from_slice(&symbol[n - cp_len..]);
        out.extend_from_slice(symbol);
        out
    }

    fn build_preamble_signal(fft_size: usize, cp_len: usize, seed: u64) -> Vec<Complex64> {
        let (s1, s2) = generate_schmidl_cox_preamble(fft_size, seed);
        let mut signal = vec![Complex64::new(0.0, 0.0); 50];
        signal.extend(add_cp(&s1, cp_len));
        signal.extend(add_cp(&s2, cp_len));
        signal.extend(vec![Complex64::new(0.0, 0.0); 50]);
        signal
    }

    #[test]
    fn test_perfect_channel_detection() {
        let fft_size = 64;
        let cp_len = 16;
        let signal = build_preamble_signal(fft_size, cp_len, 42);
        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.3);
        let results = sync.process(&signal);
        assert!(!results.is_empty(), "should detect preamble");
        // Timing should be near sample 50 (start of noise padding) + cp_len
        let timing = results[0].timing_offset;
        assert!(
            (timing as i64 - 50).unsigned_abs() < (cp_len + 5) as u64,
            "timing {timing} should be near 50..{}",
            50 + cp_len
        );
    }

    #[test]
    fn test_awgn_detection() {
        let fft_size = 64;
        let cp_len = 16;
        let mut signal = build_preamble_signal(fft_size, cp_len, 42);
        // Add noise at ~10 dB SNR
        let mut rng = 12345u64;
        for s in signal.iter_mut() {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            let n_i = ((rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.2;
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
            let n_q = ((rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 0.2;
            *s += Complex64::new(n_i, n_q);
        }
        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.2);
        let results = sync.process(&signal);
        assert!(!results.is_empty(), "should detect preamble in noise");
    }

    #[test]
    fn test_coarse_cfo_estimation() {
        let fft_size = 64;
        let cp_len = 16;
        let cfo_frac = 0.25; // 0.25 subcarrier spacings
        let mut signal = build_preamble_signal(fft_size, cp_len, 42);
        // Apply CFO
        let delta_f = cfo_frac / fft_size as f64;
        for (i, s) in signal.iter_mut().enumerate() {
            let phase = 2.0 * PI * delta_f * i as f64;
            *s *= Complex64::new(phase.cos(), phase.sin());
        }
        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.3);
        let results = sync.process(&signal);
        assert!(!results.is_empty(), "should detect preamble with CFO");
        let est_cfo = results[0].coarse_cfo;
        assert!(
            (est_cfo - cfo_frac).abs() < 0.1,
            "CFO estimate {est_cfo} should be near {cfo_frac}"
        );
    }

    #[test]
    fn test_no_preamble_no_detection() {
        let fft_size = 64;
        let cp_len = 16;
        // Random data, no preamble structure
        let mut rng = 99999u64;
        let signal: Vec<Complex64> = (0..256)
            .map(|_| {
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let r = (rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                let i = (rng >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
                Complex64::new(r * 0.1, i * 0.1)
            })
            .collect();
        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.8);
        let results = sync.process(&signal);
        assert!(results.is_empty(), "should not detect preamble in noise");
    }

    #[test]
    fn test_preamble_generation_structure() {
        let fft_size = 64;
        let (s1, s2) = generate_schmidl_cox_preamble(fft_size, 42);
        assert_eq!(s1.len(), fft_size);
        assert_eq!(s2.len(), fft_size);

        // S1 should have half-symbol repetition (due to even-only subcarriers)
        let half = fft_size / 2;
        let mut err = 0.0;
        for i in 0..half {
            let diff = s1[i] - s1[i + half];
            err += diff.norm_sqr();
        }
        assert!(
            err < 1e-10,
            "S1 should have perfect half-symbol repetition, error={err}"
        );
    }

    #[test]
    fn test_round_trip_generate_detect() {
        let fft_size = 64;
        let cp_len = 16;
        let (s1, s2) = generate_schmidl_cox_preamble(fft_size, 77);
        let mut signal = vec![Complex64::new(0.0, 0.0); 30];
        signal.extend(add_cp(&s1, cp_len));
        signal.extend(add_cp(&s2, cp_len));
        signal.extend(vec![Complex64::new(0.0, 0.0); 30]);

        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.3);
        sync.set_known_preamble_s2(s2);
        let results = sync.process(&signal);
        assert!(!results.is_empty(), "should detect generated preamble");
    }

    #[test]
    fn test_timing_metric_shape() {
        let fft_size = 64;
        let cp_len = 16;
        let signal = build_preamble_signal(fft_size, cp_len, 42);
        let sync = SchmidlCoxSync::new(fft_size, cp_len, 0.5);
        let metrics = sync.timing_metric(&signal);
        assert!(!metrics.is_empty());
        // Should have a clear peak region
        let max_val = metrics.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_val > 0.5, "peak metric should be high, got {max_val}");
    }

    #[test]
    fn test_different_cp_lengths() {
        for &cp_len in &[4, 8, 16] {
            let fft_size = 64;
            let signal = build_preamble_signal(fft_size, cp_len, 42);
            let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.3);
            let results = sync.process(&signal);
            assert!(
                !results.is_empty(),
                "should detect with cp_len={cp_len}"
            );
        }
    }

    #[test]
    fn test_zero_cfo_estimate() {
        let fft_size = 64;
        let cp_len = 16;
        let signal = build_preamble_signal(fft_size, cp_len, 42);
        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.3);
        let results = sync.process(&signal);
        assert!(!results.is_empty());
        assert!(
            results[0].coarse_cfo.abs() < 0.05,
            "zero CFO expected, got {}",
            results[0].coarse_cfo
        );
    }

    #[test]
    fn test_multiple_preambles() {
        let fft_size = 64;
        let cp_len = 16;
        let (s1, s2) = generate_schmidl_cox_preamble(fft_size, 42);

        let mut signal = vec![Complex64::new(0.0, 0.0); 30];
        // First preamble
        signal.extend(add_cp(&s1, cp_len));
        signal.extend(add_cp(&s2, cp_len));
        // Gap
        signal.extend(vec![Complex64::new(0.0, 0.0); 100]);
        // Second preamble
        signal.extend(add_cp(&s1, cp_len));
        signal.extend(add_cp(&s2, cp_len));
        signal.extend(vec![Complex64::new(0.0, 0.0); 30]);

        let mut sync = SchmidlCoxSync::new(fft_size, cp_len, 0.3);
        let results = sync.process(&signal);
        assert!(
            results.len() >= 2,
            "should detect 2 preambles, got {}",
            results.len()
        );
    }
}
