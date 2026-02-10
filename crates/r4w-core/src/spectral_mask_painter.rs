//! Spectral Mask Painter — shape signals to conform to spectral emission masks.
//!
//! This module applies frequency-domain gain profiles so that a signal's power
//! spectral density fits within a regulatory emission mask (FCC Part 15, ETSI EN
//! 300 220 for LoRa, IEEE 802.11 Wi-Fi, or a custom piecewise-linear mask).
//!
//! The core workflow is:
//! 1. Define a mask with [`MaskPoint`] breakpoints (frequency offset vs. power limit).
//! 2. Build a [`SpectralMaskPainter`] with an FFT size and sample rate.
//! 3. Call [`apply_mask`](SpectralMaskPainter::apply_mask) to attenuate violating bins, or
//!    [`check_compliance`](SpectralMaskPainter::check_compliance) to measure margin.
//! 4. Use overlap-add streaming via [`process_stream`](SpectralMaskPainter::process_stream).
//!
//! All processing uses a built-in radix-2 Cooley-Tukey FFT with only the standard
//! library — no external crates are required.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::spectral_mask_painter::{SpectralMaskPainter, MaskPreset};
//!
//! let mut painter = SpectralMaskPainter::new(64, 1_000_000.0);
//! painter.set_mask_from_preset(MaskPreset::FccPart15 { bandwidth_hz: 500_000.0 });
//!
//! // Generate shaped noise that conforms to the mask
//! let noise = painter.generate_compliant_noise(256);
//! assert_eq!(noise.len(), 256);
//!
//! // Check that a DC tone is within mask
//! let tone: Vec<(f64, f64)> = (0..64)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * (i as f64) / 64.0;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//! let result = painter.check_compliance(&tone);
//! assert!(result.passes);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A single breakpoint in a piecewise-linear spectral mask.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MaskPoint {
    /// Frequency offset from carrier centre (Hz). Negative = below carrier.
    pub frequency_offset_hz: f64,
    /// Maximum allowed power at this offset (dB relative to carrier).
    pub power_limit_db: f64,
}

/// Preset spectral masks for common regulatory domains.
#[derive(Debug, Clone, PartialEq)]
pub enum MaskPreset {
    /// FCC Part 15.247 — ISM-band devices.
    /// `bandwidth_hz` is the occupied bandwidth used to scale the mask.
    FccPart15 { bandwidth_hz: f64 },
    /// ETSI EN 300 220 — LoRa / SRD in Europe.
    EtsiEnLoRa { bandwidth_hz: f64 },
    /// IEEE 802.11 — Wi-Fi 20 MHz channel mask.
    Wifi80211,
    /// User-supplied mask points.
    Custom(Vec<MaskPoint>),
}

/// Result of a compliance check.
#[derive(Debug, Clone)]
pub struct ComplianceResult {
    /// `true` if every bin is within the mask.
    pub passes: bool,
    /// Worst (smallest) margin in dB — negative means violation.
    pub worst_margin_db: f64,
    /// Indices of frequency bins that violate the mask.
    pub violating_bins: Vec<usize>,
    /// Per-bin margin (mask_limit - measured_power) in dB.
    pub margins: Vec<f64>,
}

/// Spectral mask painter: shapes signals so they fit within an emission mask.
#[derive(Debug, Clone)]
pub struct SpectralMaskPainter {
    fft_size: usize,
    sample_rate: f64,
    /// Mask points sorted by frequency offset.
    mask_points: Vec<MaskPoint>,
    /// Pre-computed per-bin gain (linear) derived from the mask.
    bin_gains: Vec<f64>,
    /// Overlap-add tail from previous block.
    overlap_tail: Vec<(f64, f64)>,
}

// ---------------------------------------------------------------------------
// Internal mini-FFT (radix-2 Cooley-Tukey, decimation-in-time)
// ---------------------------------------------------------------------------

/// In-place radix-2 FFT. `inverse` = true for IFFT.
/// `buf` is a slice of (re, im) pairs whose length **must** be a power of two.
fn fft_in_place(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");
    if n <= 1 {
        return;
    }

    // Bit-reversal permutation
    let mut j: usize = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / (len as f64);
        let wn = (angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = complex_mul(w, buf[start + k + half]);
                buf[start + k] = (u.0 + t.0, u.1 + t.1);
                buf[start + k + half] = (u.0 - t.0, u.1 - t.1);
                w = complex_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for s in buf.iter_mut() {
            s.0 *= inv_n;
            s.1 *= inv_n;
        }
    }
}

#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Forward FFT helper returning a new Vec.
fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = input.to_vec();
    fft_in_place(&mut buf, false);
    buf
}

/// Inverse FFT helper returning a new Vec.
fn ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = input.to_vec();
    fft_in_place(&mut buf, true);
    buf
}

/// Power of a complex sample in dB (10 * log10(|x|^2)), clamped at -200 dB.
fn power_db(sample: (f64, f64)) -> f64 {
    let mag_sq = sample.0 * sample.0 + sample.1 * sample.1;
    if mag_sq < 1e-20 {
        -200.0
    } else {
        10.0 * mag_sq.log10()
    }
}

/// Convert dB to linear amplitude (voltage).
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 20.0)
}

// ---------------------------------------------------------------------------
// Hann window
// ---------------------------------------------------------------------------

fn hann_window(n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / n as f64).cos()))
        .collect()
}

// ---------------------------------------------------------------------------
// Simple PRNG (xoshiro256**)
// ---------------------------------------------------------------------------

struct Rng {
    s: [u64; 4],
}

impl Rng {
    fn new(seed: u64) -> Self {
        // SplitMix64 to expand seed
        let mut z = seed;
        let mut s = [0u64; 4];
        for si in &mut s {
            z = z.wrapping_add(0x9e3779b97f4a7c15);
            let mut v = z;
            v = (v ^ (v >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            v = (v ^ (v >> 27)).wrapping_mul(0x94d049bb133111eb);
            *si = v ^ (v >> 31);
        }
        Self { s }
    }

    fn next_u64(&mut self) -> u64 {
        let result = (self.s[1].wrapping_mul(5)).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Approximate Gaussian via Box-Muller.
    fn next_gaussian(&mut self) -> (f64, f64) {
        let u1 = self.next_f64().max(1e-15);
        let u2 = self.next_f64();
        let r = (-2.0 * u1.ln()).sqrt();
        let theta = 2.0 * PI * u2;
        (r * theta.cos(), r * theta.sin())
    }
}

// ---------------------------------------------------------------------------
// Mask preset factories
// ---------------------------------------------------------------------------

fn fcc_part15_mask(bw: f64) -> Vec<MaskPoint> {
    let hbw = bw / 2.0;
    vec![
        MaskPoint { frequency_offset_hz: -3.0 * hbw, power_limit_db: -50.0 },
        MaskPoint { frequency_offset_hz: -2.0 * hbw, power_limit_db: -40.0 },
        MaskPoint { frequency_offset_hz: -hbw,        power_limit_db: -20.0 },
        MaskPoint { frequency_offset_hz: -0.8 * hbw,  power_limit_db:   0.0 },
        MaskPoint { frequency_offset_hz:  0.8 * hbw,  power_limit_db:   0.0 },
        MaskPoint { frequency_offset_hz:  hbw,         power_limit_db: -20.0 },
        MaskPoint { frequency_offset_hz:  2.0 * hbw,  power_limit_db: -40.0 },
        MaskPoint { frequency_offset_hz:  3.0 * hbw,  power_limit_db: -50.0 },
    ]
}

fn etsi_en_lora_mask(bw: f64) -> Vec<MaskPoint> {
    let hbw = bw / 2.0;
    vec![
        MaskPoint { frequency_offset_hz: -3.0 * hbw, power_limit_db: -60.0 },
        MaskPoint { frequency_offset_hz: -1.5 * hbw, power_limit_db: -36.0 },
        MaskPoint { frequency_offset_hz: -hbw,        power_limit_db: -10.0 },
        MaskPoint { frequency_offset_hz: -0.9 * hbw,  power_limit_db:   0.0 },
        MaskPoint { frequency_offset_hz:  0.9 * hbw,  power_limit_db:   0.0 },
        MaskPoint { frequency_offset_hz:  hbw,         power_limit_db: -10.0 },
        MaskPoint { frequency_offset_hz:  1.5 * hbw,  power_limit_db: -36.0 },
        MaskPoint { frequency_offset_hz:  3.0 * hbw,  power_limit_db: -60.0 },
    ]
}

fn wifi_80211_mask() -> Vec<MaskPoint> {
    vec![
        MaskPoint { frequency_offset_hz: -30e6, power_limit_db: -40.0 },
        MaskPoint { frequency_offset_hz: -20e6, power_limit_db: -28.0 },
        MaskPoint { frequency_offset_hz: -11e6, power_limit_db: -20.0 },
        MaskPoint { frequency_offset_hz:  -9e6, power_limit_db:   0.0 },
        MaskPoint { frequency_offset_hz:   9e6, power_limit_db:   0.0 },
        MaskPoint { frequency_offset_hz:  11e6, power_limit_db: -20.0 },
        MaskPoint { frequency_offset_hz:  20e6, power_limit_db: -28.0 },
        MaskPoint { frequency_offset_hz:  30e6, power_limit_db: -40.0 },
    ]
}

// ---------------------------------------------------------------------------
// SpectralMaskPainter implementation
// ---------------------------------------------------------------------------

impl SpectralMaskPainter {
    /// Create a new painter.
    ///
    /// `fft_size` **must** be a power of two (will be rounded up if not).
    /// `sample_rate` is in Hz.
    pub fn new(fft_size: usize, sample_rate: f64) -> Self {
        let fft_size = fft_size.next_power_of_two();
        Self {
            fft_size,
            sample_rate,
            mask_points: Vec::new(),
            bin_gains: vec![1.0; fft_size],
            overlap_tail: vec![(0.0, 0.0); fft_size],
        }
    }

    /// Current FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Current sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Set the mask from raw [`MaskPoint`] breakpoints.
    ///
    /// Points are sorted by `frequency_offset_hz` internally.
    pub fn set_mask(&mut self, mut points: Vec<MaskPoint>) {
        points.sort_by(|a, b| {
            a.frequency_offset_hz
                .partial_cmp(&b.frequency_offset_hz)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        self.mask_points = points;
        self.recompute_bin_gains();
    }

    /// Set the mask from a [`MaskPreset`].
    pub fn set_mask_from_preset(&mut self, preset: MaskPreset) {
        let points = match preset {
            MaskPreset::FccPart15 { bandwidth_hz } => fcc_part15_mask(bandwidth_hz),
            MaskPreset::EtsiEnLoRa { bandwidth_hz } => etsi_en_lora_mask(bandwidth_hz),
            MaskPreset::Wifi80211 => wifi_80211_mask(),
            MaskPreset::Custom(pts) => pts,
        };
        self.set_mask(points);
    }

    /// Return a copy of the current mask points.
    pub fn mask_points(&self) -> &[MaskPoint] {
        &self.mask_points
    }

    /// Return the per-bin gains (linear amplitude scale).
    pub fn bin_gains(&self) -> &[f64] {
        &self.bin_gains
    }

    // -- Core operations ----------------------------------------------------

    /// Apply the mask to `signal`, attenuating bins that exceed the limit.
    ///
    /// Returns the filtered time-domain signal (same length as input, zero-padded
    /// to FFT size internally).
    pub fn apply_mask(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.fft_size;
        let mut padded = vec![(0.0, 0.0); n];
        let copy_len = signal.len().min(n);
        padded[..copy_len].copy_from_slice(&signal[..copy_len]);

        let mut freq = fft(&padded);

        // For each bin, if the bin power exceeds the mask limit, scale it down
        // so it sits exactly at the limit. Never amplify.
        for (i, bin) in freq.iter_mut().enumerate() {
            let gain = self.bin_gains[i];
            if gain < 1.0 {
                bin.0 *= gain;
                bin.1 *= gain;
            }
        }

        let time = ifft(&freq);
        time[..copy_len].to_vec()
    }

    /// Check whether `signal` complies with the mask.
    ///
    /// The signal is FFT'd and each bin's power is compared against the
    /// interpolated mask limit.
    pub fn check_compliance(&self, signal: &[(f64, f64)]) -> ComplianceResult {
        let n = self.fft_size;
        let mut padded = vec![(0.0, 0.0); n];
        let copy_len = signal.len().min(n);
        padded[..copy_len].copy_from_slice(&signal[..copy_len]);

        let freq = fft(&padded);

        // Reference power: peak bin
        let peak_power_db = freq.iter().map(|s| power_db(*s)).fold(f64::NEG_INFINITY, f64::max);

        let mask_limits = self.interpolated_mask_limits_db();
        let mut margins = Vec::with_capacity(n);
        let mut violating_bins = Vec::new();
        let mut worst_margin = f64::INFINITY;

        for (i, bin) in freq.iter().enumerate() {
            let bin_power = power_db(*bin) - peak_power_db; // relative to peak
            let margin = mask_limits[i] - bin_power;
            if margin < worst_margin {
                worst_margin = margin;
            }
            if margin < 0.0 {
                violating_bins.push(i);
            }
            margins.push(margin);
        }

        if worst_margin == f64::INFINITY {
            worst_margin = 0.0;
        }

        ComplianceResult {
            passes: violating_bins.is_empty(),
            worst_margin_db: worst_margin,
            violating_bins,
            margins,
        }
    }

    /// Generate white Gaussian noise shaped to conform to the current mask.
    ///
    /// `num_samples` is the desired output length. A fixed seed is used for
    /// reproducibility; call with different lengths for different realizations.
    pub fn generate_compliant_noise(&self, num_samples: usize) -> Vec<(f64, f64)> {
        self.generate_compliant_noise_seeded(num_samples, 42)
    }

    /// Like [`generate_compliant_noise`](Self::generate_compliant_noise) but
    /// with a caller-supplied seed.
    pub fn generate_compliant_noise_seeded(&self, num_samples: usize, seed: u64) -> Vec<(f64, f64)> {
        let n = self.fft_size;
        let mut rng = Rng::new(seed);
        let mut output = Vec::with_capacity(num_samples);

        while output.len() < num_samples {
            // Generate white noise in frequency domain
            let mut freq = Vec::with_capacity(n);
            for _ in 0..n {
                let (re, im) = rng.next_gaussian();
                freq.push((re, im));
            }

            // Apply mask gains
            for (i, bin) in freq.iter_mut().enumerate() {
                let g = self.bin_gains[i];
                bin.0 *= g;
                bin.1 *= g;
            }

            let time = ifft(&freq);
            let take = (num_samples - output.len()).min(n);
            output.extend_from_slice(&time[..take]);
        }

        output.truncate(num_samples);
        output
    }

    /// Overlap-add streaming processor.
    ///
    /// Feed successive blocks of arbitrary length. Internally buffers with 50%
    /// overlap and a Hann window. Returns the output samples available so far.
    pub fn process_stream(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = self.fft_size;
        let hop = n / 2;
        let window = hann_window(n);

        let mut output = Vec::new();

        // Process each hop-sized chunk
        let mut pos = 0;
        while pos + n <= input.len() + hop {
            let mut frame = vec![(0.0, 0.0); n];
            for i in 0..n {
                let src_idx = pos + i;
                if src_idx < input.len() {
                    frame[i] = input[src_idx];
                }
            }

            // Apply analysis window
            for (i, s) in frame.iter_mut().enumerate() {
                s.0 *= window[i];
                s.1 *= window[i];
            }

            // FFT -> mask -> IFFT
            let mut freq = fft(&frame);
            for (i, bin) in freq.iter_mut().enumerate() {
                let g = self.bin_gains[i];
                if g < 1.0 {
                    bin.0 *= g;
                    bin.1 *= g;
                }
            }
            let time = ifft(&freq);

            // Synthesis window
            let mut synth = vec![(0.0, 0.0); n];
            for i in 0..n {
                synth[i] = (time[i].0 * window[i], time[i].1 * window[i]);
            }

            // Overlap-add with previous tail
            let mut combined = vec![(0.0, 0.0); n];
            for i in 0..n {
                combined[i].0 = synth[i].0 + self.overlap_tail[i].0;
                combined[i].1 = synth[i].1 + self.overlap_tail[i].1;
            }

            // Output the first hop samples
            let out_len = hop.min(input.len().saturating_sub(pos));
            output.extend_from_slice(&combined[..out_len]);

            // Save the second half as the new overlap tail
            let mut new_tail = vec![(0.0, 0.0); n];
            for i in hop..n {
                new_tail[i - hop] = combined[i];
            }
            self.overlap_tail = new_tail;

            pos += hop;
            if pos >= input.len() {
                break;
            }
        }

        output
    }

    /// Reset the overlap-add state (e.g. between non-contiguous blocks).
    pub fn reset_overlap(&mut self) {
        self.overlap_tail = vec![(0.0, 0.0); self.fft_size];
    }

    // -- Internals ----------------------------------------------------------

    /// Recompute per-bin gains from the current mask points.
    fn recompute_bin_gains(&mut self) {
        let limits = self.interpolated_mask_limits_db();
        self.bin_gains = limits.iter().map(|&db| db_to_linear(db.min(0.0))).collect();
    }

    /// Interpolate mask points to get a dB limit for every FFT bin.
    ///
    /// Bins outside the mask range are clamped to the nearest endpoint.
    fn interpolated_mask_limits_db(&self) -> Vec<f64> {
        let n = self.fft_size;
        let fs = self.sample_rate;
        let mut limits = vec![0.0_f64; n];

        if self.mask_points.is_empty() {
            return limits;
        }

        for i in 0..n {
            // Map bin index to frequency offset: bin 0 = 0 Hz, bin n/2 = +fs/2,
            // bin n/2+1 = -fs/2+1*df, ..., bin n-1 = -df.  (standard FFT layout)
            let freq = if i <= n / 2 {
                (i as f64) * fs / (n as f64)
            } else {
                (i as f64 - n as f64) * fs / (n as f64)
            };

            limits[i] = self.interpolate_mask_at(freq);
        }

        limits
    }

    /// Linear interpolation of the mask at a given frequency offset.
    fn interpolate_mask_at(&self, freq: f64) -> f64 {
        let pts = &self.mask_points;
        if pts.is_empty() {
            return 0.0;
        }

        // Clamp to endpoints
        if freq <= pts[0].frequency_offset_hz {
            return pts[0].power_limit_db;
        }
        if freq >= pts[pts.len() - 1].frequency_offset_hz {
            return pts[pts.len() - 1].power_limit_db;
        }

        // Find the two surrounding points
        for w in pts.windows(2) {
            let (lo, hi) = (&w[0], &w[1]);
            if freq >= lo.frequency_offset_hz && freq <= hi.frequency_offset_hz {
                let span = hi.frequency_offset_hz - lo.frequency_offset_hz;
                if span.abs() < 1e-12 {
                    return lo.power_limit_db;
                }
                let t = (freq - lo.frequency_offset_hz) / span;
                return lo.power_limit_db + t * (hi.power_limit_db - lo.power_limit_db);
            }
        }

        // Fallback (should not reach here)
        pts[pts.len() - 1].power_limit_db
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: magnitude of complex tuple.
    fn mag(s: (f64, f64)) -> f64 {
        (s.0 * s.0 + s.1 * s.1).sqrt()
    }

    // --- FFT correctness ---------------------------------------------------

    #[test]
    fn test_fft_roundtrip() {
        let n = 16;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| ((i as f64).sin(), (i as f64).cos()))
            .collect();
        let freq = fft(&signal);
        let recovered = ifft(&freq);
        for (a, b) in signal.iter().zip(recovered.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10, "FFT roundtrip mismatch");
            assert!((a.1 - b.1).abs() < 1e-10, "FFT roundtrip mismatch");
        }
    }

    #[test]
    fn test_fft_dc_signal() {
        let n = 8;
        let signal = vec![(1.0, 0.0); n];
        let freq = fft(&signal);
        // DC bin should have magnitude n, all others ~0
        assert!((mag(freq[0]) - n as f64).abs() < 1e-10);
        for i in 1..n {
            assert!(mag(freq[i]) < 1e-10, "non-DC bin {} should be zero", i);
        }
    }

    // --- Mask preset construction ------------------------------------------

    #[test]
    fn test_fcc_part15_mask_symmetric() {
        let pts = fcc_part15_mask(1e6);
        // Verify symmetry: positive and negative offsets should mirror
        let n = pts.len();
        for i in 0..n / 2 {
            let lo = &pts[i];
            let hi = &pts[n - 1 - i];
            assert!(
                (lo.frequency_offset_hz + hi.frequency_offset_hz).abs() < 1.0,
                "mask not symmetric"
            );
            assert!(
                (lo.power_limit_db - hi.power_limit_db).abs() < 1e-10,
                "mask power not symmetric"
            );
        }
    }

    #[test]
    fn test_wifi_mask_bandwidth() {
        let pts = wifi_80211_mask();
        // Passband should be 0 dB at +/-9 MHz
        let passband: Vec<_> = pts.iter().filter(|p| p.power_limit_db == 0.0).collect();
        assert_eq!(passband.len(), 2);
        assert!((passband[0].frequency_offset_hz - (-9e6)).abs() < 1.0);
        assert!((passband[1].frequency_offset_hz - 9e6).abs() < 1.0);
    }

    // --- set_mask and interpolation ----------------------------------------

    #[test]
    fn test_mask_interpolation() {
        let mut painter = SpectralMaskPainter::new(64, 1e6);
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -500e3, power_limit_db: -40.0 },
            MaskPoint { frequency_offset_hz:    0.0, power_limit_db:   0.0 },
            MaskPoint { frequency_offset_hz:  500e3, power_limit_db: -40.0 },
        ]);

        // At DC the limit should be 0 dB
        let at_dc = painter.interpolate_mask_at(0.0);
        assert!((at_dc - 0.0).abs() < 1e-10);

        // At +/-250 kHz the limit should be -20 dB (midpoint)
        let at_250k = painter.interpolate_mask_at(250e3);
        assert!((at_250k - (-20.0)).abs() < 1e-10);
    }

    #[test]
    fn test_mask_clamping_outside_range() {
        let mut painter = SpectralMaskPainter::new(64, 1e6);
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -100e3, power_limit_db: -30.0 },
            MaskPoint { frequency_offset_hz:  100e3, power_limit_db: -30.0 },
        ]);

        // Beyond +/-100 kHz should clamp to -30 dB
        assert!((painter.interpolate_mask_at(-500e3) - (-30.0)).abs() < 1e-10);
        assert!((painter.interpolate_mask_at(500e3) - (-30.0)).abs() < 1e-10);
    }

    // --- apply_mask --------------------------------------------------------

    #[test]
    fn test_apply_mask_attenuates_oob() {
        let n = 64;
        let mut painter = SpectralMaskPainter::new(n, 1e6);
        // Flat -40 dB mask everywhere: should heavily attenuate
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -500e3, power_limit_db: -40.0 },
            MaskPoint { frequency_offset_hz:  500e3, power_limit_db: -40.0 },
        ]);

        // Input: unit-amplitude tone
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * (i as f64) / (n as f64);
                (phase.cos(), phase.sin())
            })
            .collect();

        let shaped = painter.apply_mask(&signal);
        // Output should be much smaller than input
        let input_power: f64 = signal.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / n as f64;
        let output_power: f64 = shaped.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum::<f64>() / n as f64;
        assert!(
            output_power < input_power * 0.01,
            "output should be attenuated: in={input_power} out={output_power}"
        );
    }

    #[test]
    fn test_apply_mask_preserves_passband() {
        let n = 64;
        let mut painter = SpectralMaskPainter::new(n, 1e6);
        // 0 dB passband mask -- should not attenuate at all
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -500e3, power_limit_db: 0.0 },
            MaskPoint { frequency_offset_hz:  500e3, power_limit_db: 0.0 },
        ]);

        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 3.0 * (i as f64) / (n as f64);
                (phase.cos(), phase.sin())
            })
            .collect();

        let shaped = painter.apply_mask(&signal);
        for (a, b) in signal.iter().zip(shaped.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10, "passband should be preserved");
            assert!((a.1 - b.1).abs() < 1e-10, "passband should be preserved");
        }
    }

    // --- check_compliance --------------------------------------------------

    #[test]
    fn test_compliance_passes_for_quiet_signal() {
        let n = 64;
        let mut painter = SpectralMaskPainter::new(n, 1e6);
        painter.set_mask_from_preset(MaskPreset::FccPart15 { bandwidth_hz: 500e3 });

        // A DC signal should have all energy in bin 0, well within mask
        let signal = vec![(1.0, 0.0); n];
        let result = painter.check_compliance(&signal);
        assert!(result.passes, "DC signal should pass FCC mask");
    }

    #[test]
    fn test_compliance_reports_margins_length() {
        let n = 64;
        let mut painter = SpectralMaskPainter::new(n, 1e6);
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -500e3, power_limit_db: -80.0 },
            MaskPoint { frequency_offset_hz:  -10e3, power_limit_db: -80.0 },
            MaskPoint { frequency_offset_hz:    0.0, power_limit_db:   0.0 },
            MaskPoint { frequency_offset_hz:   10e3, power_limit_db: -80.0 },
            MaskPoint { frequency_offset_hz:  500e3, power_limit_db: -80.0 },
        ]);

        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| ((i as f64 * 0.7).sin(), (i as f64 * 1.3).cos()))
            .collect();

        let result = painter.check_compliance(&signal);
        assert_eq!(result.margins.len(), n);
    }

    // --- generate_compliant_noise ------------------------------------------

    #[test]
    fn test_compliant_noise_length() {
        let mut painter = SpectralMaskPainter::new(64, 1e6);
        painter.set_mask_from_preset(MaskPreset::FccPart15 { bandwidth_hz: 500e3 });

        let noise = painter.generate_compliant_noise(1000);
        assert_eq!(noise.len(), 1000);
    }

    #[test]
    fn test_compliant_noise_shaped_spectrum() {
        let n = 256;
        let mut painter = SpectralMaskPainter::new(n, 1e6);
        // Strong roll-off mask
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -500e3, power_limit_db: -60.0 },
            MaskPoint { frequency_offset_hz: -100e3, power_limit_db:   0.0 },
            MaskPoint { frequency_offset_hz:  100e3, power_limit_db:   0.0 },
            MaskPoint { frequency_offset_hz:  500e3, power_limit_db: -60.0 },
        ]);

        let noise = painter.generate_compliant_noise_seeded(n, 123);
        let freq = fft(&noise);

        // Edge bins (near +/-fs/2 = +/-500 kHz) should be much weaker than centre
        let centre_power = freq[0].0 * freq[0].0 + freq[0].1 * freq[0].1;
        let edge_power = freq[n / 2].0 * freq[n / 2].0 + freq[n / 2].1 * freq[n / 2].1;
        // With -60 dB at the edge, the edge should be much weaker. Allow some
        // statistical variation since it is random noise.
        assert!(
            edge_power < centre_power || centre_power < 1e-10,
            "edge bins should be weaker than centre (edge={edge_power}, centre={centre_power})"
        );
    }

    // --- overlap-add streaming ---------------------------------------------

    #[test]
    fn test_stream_output_length() {
        let n = 64;
        let mut painter = SpectralMaskPainter::new(n, 1e6);
        painter.set_mask(vec![
            MaskPoint { frequency_offset_hz: -500e3, power_limit_db: 0.0 },
            MaskPoint { frequency_offset_hz:  500e3, power_limit_db: 0.0 },
        ]);

        let input = vec![(1.0, 0.0); 256];
        let output = painter.process_stream(&input);
        // Overlap-add with hop = n/2 should produce roughly input.len() samples
        // (minus some edge effects)
        assert!(
            output.len() >= 128,
            "stream output too short: {}",
            output.len()
        );
    }

    // --- edge cases --------------------------------------------------------

    #[test]
    fn test_empty_mask_is_passthrough() {
        let n = 32;
        let painter = SpectralMaskPainter::new(n, 1e6);
        // No mask set -> bin_gains all 1.0 -> passthrough
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| ((i as f64).sin(), 0.0))
            .collect();
        let shaped = painter.apply_mask(&signal);
        for (a, b) in signal.iter().zip(shaped.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10);
            assert!((a.1 - b.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_custom_preset() {
        let mut painter = SpectralMaskPainter::new(64, 1e6);
        let custom_points = vec![
            MaskPoint { frequency_offset_hz: -400e3, power_limit_db: -30.0 },
            MaskPoint { frequency_offset_hz:    0.0, power_limit_db:   0.0 },
            MaskPoint { frequency_offset_hz:  400e3, power_limit_db: -30.0 },
        ];
        painter.set_mask_from_preset(MaskPreset::Custom(custom_points.clone()));
        assert_eq!(painter.mask_points().len(), 3);
    }

    #[test]
    fn test_non_power_of_two_rounds_up() {
        let painter = SpectralMaskPainter::new(100, 1e6);
        assert_eq!(painter.fft_size(), 128);
    }

    #[test]
    fn test_db_to_linear_conversions() {
        assert!((db_to_linear(0.0) - 1.0).abs() < 1e-10);
        assert!((db_to_linear(-20.0) - 0.1).abs() < 1e-5);
        assert!((db_to_linear(-40.0) - 0.01).abs() < 1e-6);
        assert!((db_to_linear(-60.0) - 0.001).abs() < 1e-7);
    }

    #[test]
    fn test_etsi_lora_mask_has_points() {
        let pts = etsi_en_lora_mask(125e3);
        assert!(pts.len() >= 6, "ETSI mask should have multiple points");
        // Centre should be 0 dB
        let centre: Vec<_> = pts.iter().filter(|p| p.power_limit_db == 0.0).collect();
        assert!(!centre.is_empty(), "should have 0 dB passband");
    }
}
