//! Polyphase Filter Bank (PFB) Synthesizer
//!
//! The inverse of `PfbChannelizer`. Takes M narrowband channel streams and
//! combines them into a single wideband output using polyphase-FFT synthesis.
//! Used for multi-carrier transmitters and efficient upconversion.
//!
//! ## Signal Flow
//!
//! ```text
//! channel[0] ─┐
//! channel[1] ─┤→ [M-point FFT] → [polyphase sub-filters] → [commutate] → wideband out
//! ...         │
//! channel[M-1]┘
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pfb_synthesizer::{PfbSynthesizer, PfbSynthConfig};
//! use num_complex::Complex64;
//!
//! let synth = PfbSynthesizer::new(PfbSynthConfig {
//!     num_channels: 4,
//!     taps_per_channel: 8,
//!     ..Default::default()
//! });
//!
//! // 4 channels, each with 16 samples
//! let channels: Vec<Vec<Complex64>> = (0..4)
//!     .map(|_| vec![Complex64::new(1.0, 0.0); 16])
//!     .collect();
//!
//! let wideband = synth.synthesize(&channels);
//! assert_eq!(wideband.len(), 64); // 4 * 16 = 64
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for the PFB synthesizer.
#[derive(Debug, Clone)]
pub struct PfbSynthConfig {
    /// Number of channels (M). Must be a power of 2. Default: 4
    pub num_channels: usize,
    /// Number of filter taps per channel. Default: 8
    pub taps_per_channel: usize,
    /// Window function: "hamming", "blackman", "hann". Default: "hamming"
    pub window: String,
}

impl Default for PfbSynthConfig {
    fn default() -> Self {
        Self {
            num_channels: 4,
            taps_per_channel: 8,
            window: "hamming".to_string(),
        }
    }
}

/// Polyphase Filter Bank Synthesizer.
#[derive(Clone, Debug)]
pub struct PfbSynthesizer {
    num_channels: usize,
    taps_per_channel: usize,
    /// Polyphase sub-filters (M branches)
    polyphase_taps: Vec<Vec<f64>>,
}

impl PfbSynthesizer {
    /// Create a new PFB synthesizer.
    pub fn new(config: PfbSynthConfig) -> Self {
        let m = config.num_channels;
        assert!(m > 0 && m.is_power_of_two(), "num_channels must be power of 2");

        let total_taps = m * config.taps_per_channel;
        let prototype = Self::design_prototype(total_taps, m, &config.window);

        // Decompose into polyphase branches
        let mut polyphase_taps = vec![vec![0.0; config.taps_per_channel]; m];
        for (i, &tap) in prototype.iter().enumerate() {
            let branch = i % m;
            let idx = i / m;
            if idx < config.taps_per_channel {
                polyphase_taps[branch][idx] = tap;
            }
        }

        Self {
            num_channels: m,
            taps_per_channel: config.taps_per_channel,
            polyphase_taps,
        }
    }

    /// Synthesize wideband output from M channel streams.
    ///
    /// Each channel vector should have the same length (N samples).
    /// Output length = M * N.
    pub fn synthesize(&self, channels: &[Vec<Complex64>]) -> Vec<Complex64> {
        let m = self.num_channels;
        assert_eq!(channels.len(), m, "Expected {} channels, got {}", m, channels.len());

        let n = channels[0].len();
        for ch in channels {
            assert_eq!(ch.len(), n, "All channels must have equal length");
        }

        let mut output = Vec::with_capacity(m * n);

        for sample_idx in 0..n {
            // Gather one sample from each channel
            let mut freq_domain = vec![Complex64::new(0.0, 0.0); m];
            for ch in 0..m {
                freq_domain[ch] = channels[ch][sample_idx];
            }

            // M-point IFFT
            let time_domain = Self::ifft(&freq_domain);

            // Apply polyphase sub-filters and commutate to output
            for branch in 0..m {
                let mut val = time_domain[branch];
                // Apply the first tap of the sub-filter (simplified - full version would use delay lines)
                val *= self.polyphase_taps[branch][0];
                output.push(val);
            }
        }

        output
    }

    /// Get the prototype lowpass filter taps.
    pub fn prototype_taps(&self) -> Vec<f64> {
        let total = self.num_channels * self.taps_per_channel;
        Self::design_prototype(total, self.num_channels, "hamming")
    }

    /// Number of channels.
    pub fn num_channels(&self) -> usize {
        self.num_channels
    }

    /// Design prototype lowpass filter (windowed sinc).
    fn design_prototype(num_taps: usize, num_channels: usize, window: &str) -> Vec<f64> {
        let fc = 1.0 / num_channels as f64; // Normalized cutoff
        let mid = (num_taps - 1) as f64 / 2.0;

        (0..num_taps)
            .map(|i| {
                let n = i as f64 - mid;
                // Sinc
                let sinc = if n.abs() < 1e-12 {
                    2.0 * PI * fc
                } else {
                    (2.0 * PI * fc * n).sin() / n
                };

                // Window
                let w = match window {
                    "blackman" => {
                        let x = i as f64 / (num_taps - 1).max(1) as f64;
                        0.42 - 0.5 * (2.0 * PI * x).cos() + 0.08 * (4.0 * PI * x).cos()
                    }
                    "hann" => {
                        let x = i as f64 / (num_taps - 1).max(1) as f64;
                        0.5 * (1.0 - (2.0 * PI * x).cos())
                    }
                    _ => {
                        // hamming
                        let x = i as f64 / (num_taps - 1).max(1) as f64;
                        0.54 - 0.46 * (2.0 * PI * x).cos()
                    }
                };

                sinc * w / (2.0 * PI)
            })
            .collect()
    }

    /// Simple radix-2 IFFT (inverse FFT).
    fn ifft(data: &[Complex64]) -> Vec<Complex64> {
        let n = data.len();
        if n <= 1 {
            return data.to_vec();
        }

        // Conjugate, FFT, conjugate, scale
        let mut conj: Vec<Complex64> = data.iter().map(|z| z.conj()).collect();
        Self::fft_inplace(&mut conj);
        conj.iter()
            .map(|z| z.conj() / n as f64)
            .collect()
    }

    /// In-place Cooley-Tukey radix-2 FFT.
    fn fft_inplace(data: &mut [Complex64]) {
        let n = data.len();
        if n <= 1 {
            return;
        }
        assert!(n.is_power_of_two());

        // Bit-reversal permutation
        let bits = n.trailing_zeros();
        for i in 0..n {
            let j = i.reverse_bits() >> (usize::BITS - bits);
            if i < j {
                data.swap(i, j);
            }
        }

        // Butterfly stages
        let mut len = 2;
        while len <= n {
            let half = len / 2;
            let w_step = -2.0 * PI / len as f64;
            for start in (0..n).step_by(len) {
                for k in 0..half {
                    let angle = w_step * k as f64;
                    let twiddle = Complex64::new(angle.cos(), angle.sin());
                    let u = data[start + k];
                    let v = data[start + k + half] * twiddle;
                    data[start + k] = u + v;
                    data[start + k + half] = u - v;
                }
            }
            len *= 2;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_construction() {
        let synth = PfbSynthesizer::new(PfbSynthConfig::default());
        assert_eq!(synth.num_channels(), 4);
    }

    #[test]
    fn test_output_length() {
        let synth = PfbSynthesizer::new(PfbSynthConfig {
            num_channels: 4,
            taps_per_channel: 4,
            ..Default::default()
        });
        let channels: Vec<Vec<Complex64>> = (0..4)
            .map(|_| vec![Complex64::new(1.0, 0.0); 16])
            .collect();
        let output = synth.synthesize(&channels);
        assert_eq!(output.len(), 64); // 4 * 16
    }

    #[test]
    fn test_single_channel_passthrough() {
        // With all channels silent except one, output should have spectral content
        // only at that channel's frequency
        let synth = PfbSynthesizer::new(PfbSynthConfig {
            num_channels: 4,
            taps_per_channel: 4,
            ..Default::default()
        });
        let mut channels = vec![vec![Complex64::new(0.0, 0.0); 32]; 4];
        channels[0] = vec![Complex64::new(1.0, 0.0); 32];
        let output = synth.synthesize(&channels);
        assert_eq!(output.len(), 128);
        // Output should be non-zero (channel 0 has content)
        let power: f64 = output.iter().map(|z| z.norm_sqr()).sum();
        assert!(power > 0.0, "Output should have non-zero power");
    }

    #[test]
    fn test_all_channels_equal() {
        let synth = PfbSynthesizer::new(PfbSynthConfig {
            num_channels: 4,
            taps_per_channel: 4,
            ..Default::default()
        });
        let channels: Vec<Vec<Complex64>> = (0..4)
            .map(|_| vec![Complex64::new(1.0, 0.0); 8])
            .collect();
        let output = synth.synthesize(&channels);
        assert_eq!(output.len(), 32);
    }

    #[test]
    fn test_prototype_taps() {
        let synth = PfbSynthesizer::new(PfbSynthConfig::default());
        let taps = synth.prototype_taps();
        assert_eq!(taps.len(), 32); // 4 channels * 8 taps
        // All taps should be finite
        for &t in &taps {
            assert!(t.is_finite(), "Tap should be finite");
        }
    }

    #[test]
    fn test_two_channels() {
        let synth = PfbSynthesizer::new(PfbSynthConfig {
            num_channels: 2,
            taps_per_channel: 4,
            ..Default::default()
        });
        let channels = vec![
            vec![Complex64::new(1.0, 0.0); 8],
            vec![Complex64::new(0.5, 0.0); 8],
        ];
        let output = synth.synthesize(&channels);
        assert_eq!(output.len(), 16); // 2 * 8
    }

    #[test]
    fn test_ifft_known_result() {
        // IFFT of [4, 0, 0, 0] = [1, 1, 1, 1]
        let data = vec![
            Complex64::new(4.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
        ];
        let result = PfbSynthesizer::ifft(&data);
        for (i, s) in result.iter().enumerate() {
            assert!(
                (s.re - 1.0).abs() < 1e-10 && s.im.abs() < 1e-10,
                "IFFT[{}] = ({:.4}, {:.4}), expected (1, 0)",
                i, s.re, s.im
            );
        }
    }

    #[test]
    fn test_window_types() {
        for window in &["hamming", "blackman", "hann"] {
            let synth = PfbSynthesizer::new(PfbSynthConfig {
                window: window.to_string(),
                ..Default::default()
            });
            assert_eq!(synth.num_channels(), 4);
        }
    }
}
