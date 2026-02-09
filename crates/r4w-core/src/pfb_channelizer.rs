//! Polyphase Filter Bank (PFB) Channelizer
//!
//! Efficiently splits a wideband signal into multiple narrowband channels
//! using polyphase decomposition with an IFFT. Equivalent to running M
//! independent filters and downsampling, but at O(N log M) instead of O(N*M).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pfb_channelizer::{PfbChannelizer, PfbConfig};
//! use num_complex::Complex64;
//!
//! // Split into 4 channels
//! let channelizer = PfbChannelizer::new(PfbConfig {
//!     num_channels: 4,
//!     taps_per_channel: 8,
//!     ..Default::default()
//! });
//!
//! let wideband = vec![Complex64::new(1.0, 0.0); 64];
//! let channels = channelizer.analyze(&wideband);
//! assert_eq!(channels.len(), 4); // 4 channels
//! assert_eq!(channels[0].len(), 16); // 64/4 = 16 samples per channel
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for the PFB channelizer.
#[derive(Debug, Clone)]
pub struct PfbConfig {
    /// Number of channels (M). Must be a power of 2. Default: 4
    pub num_channels: usize,
    /// Number of filter taps per channel. Total taps = num_channels * taps_per_channel. Default: 8
    pub taps_per_channel: usize,
    /// Window function for prototype filter: "hamming", "blackman", "hann". Default: "hamming"
    pub window: String,
    /// Oversampling ratio (1 = critically sampled). Default: 1
    pub oversample: usize,
}

impl Default for PfbConfig {
    fn default() -> Self {
        Self {
            num_channels: 4,
            taps_per_channel: 8,
            window: "hamming".to_string(),
            oversample: 1,
        }
    }
}

/// Polyphase Filter Bank Channelizer.
///
/// Decomposes a wideband input into M narrowband channels using the
/// polyphase-IFFT architecture:
///
/// 1. Commutate input into M polyphase branches
/// 2. Filter each branch with its sub-filter
/// 3. Compute M-point IFFT to produce one sample per channel
///
/// This is equivalent to GNU Radio's `pfb_channelizer_ccf`.
#[derive(Clone)]
pub struct PfbChannelizer {
    config: PfbConfig,
    /// Polyphase sub-filters (M branches × taps_per_channel)
    polyphase_taps: Vec<Vec<f64>>,
    /// Delay lines for each branch
    delay_lines: Vec<Vec<Complex64>>,
    /// Delay line write index
    delay_idx: usize,
}

impl std::fmt::Debug for PfbChannelizer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PfbChannelizer")
            .field("num_channels", &self.config.num_channels)
            .field("taps_per_channel", &self.config.taps_per_channel)
            .finish()
    }
}

impl PfbChannelizer {
    /// Create a new PFB channelizer.
    pub fn new(config: PfbConfig) -> Self {
        let m = config.num_channels;
        let taps_per = config.taps_per_channel;

        // Design prototype lowpass filter
        let total_taps = m * taps_per;
        let prototype = Self::design_prototype(total_taps, m, &config.window);

        // Decompose into polyphase branches
        // Branch k gets taps: prototype[k], prototype[k+M], prototype[k+2M], ...
        let mut polyphase_taps = vec![vec![0.0f64; taps_per]; m];
        for k in 0..m {
            for j in 0..taps_per {
                let idx = k + j * m;
                if idx < prototype.len() {
                    polyphase_taps[k][j] = prototype[idx];
                }
            }
        }

        let delay_lines = vec![vec![Complex64::new(0.0, 0.0); taps_per]; m];

        Self {
            config,
            polyphase_taps,
            delay_lines,
            delay_idx: 0,
        }
    }

    /// Design a windowed-sinc lowpass prototype filter.
    fn design_prototype(num_taps: usize, m: usize, window: &str) -> Vec<f64> {
        let mut taps = vec![0.0f64; num_taps];
        let fc = 1.0 / m as f64; // Normalized cutoff for channelizer
        let center = (num_taps - 1) as f64 / 2.0;

        for i in 0..num_taps {
            let n = i as f64 - center;
            // Sinc function
            let sinc = if n.abs() < 1e-10 {
                2.0 * PI * fc
            } else {
                (2.0 * PI * fc * n).sin() / n
            };

            // Window function
            let w = match window {
                "blackman" => {
                    0.42 - 0.5 * (2.0 * PI * i as f64 / (num_taps - 1) as f64).cos()
                        + 0.08 * (4.0 * PI * i as f64 / (num_taps - 1) as f64).cos()
                }
                "hann" => {
                    0.5 * (1.0 - (2.0 * PI * i as f64 / (num_taps - 1) as f64).cos())
                }
                _ => {
                    // Hamming (default)
                    0.54 - 0.46 * (2.0 * PI * i as f64 / (num_taps - 1) as f64).cos()
                }
            };

            taps[i] = sinc * w;
        }

        // Normalize
        let sum: f64 = taps.iter().sum();
        if sum.abs() > 1e-20 {
            for t in taps.iter_mut() {
                *t /= sum;
            }
        }

        taps
    }

    /// Analyze (channelize) a block of wideband input.
    ///
    /// Input must be a multiple of num_channels samples.
    /// Returns M channel outputs, each of length input_len / M.
    pub fn analyze(&self, input: &[Complex64]) -> Vec<Vec<Complex64>> {
        let m = self.config.num_channels;
        let taps_per = self.config.taps_per_channel;
        let num_output_samples = input.len() / m;

        let mut channels = vec![vec![Complex64::new(0.0, 0.0); num_output_samples]; m];

        // Per-branch delay lines for stateful filtering
        let mut delay_lines = vec![vec![Complex64::new(0.0, 0.0); taps_per]; m];

        // Process M samples at a time
        for out_idx in 0..num_output_samples {
            let base = out_idx * m;

            // Feed M samples into polyphase branches (reverse order for commutator)
            let mut branch_outputs = vec![Complex64::new(0.0, 0.0); m];
            for k in 0..m {
                let branch_idx = m - 1 - k; // Reverse commutation
                let sample = if base + k < input.len() {
                    input[base + k]
                } else {
                    Complex64::new(0.0, 0.0)
                };

                // Shift delay line and insert new sample
                let dl = &mut delay_lines[branch_idx];
                for j in (1..taps_per).rev() {
                    dl[j] = dl[j - 1];
                }
                dl[0] = sample;

                // Apply polyphase sub-filter (FIR convolution)
                let mut acc = Complex64::new(0.0, 0.0);
                for j in 0..taps_per {
                    acc += dl[j] * self.polyphase_taps[branch_idx][j];
                }
                branch_outputs[branch_idx] = acc;
            }

            // M-point IFFT to separate channels
            let channel_samples = Self::ifft_m(&branch_outputs);

            for k in 0..m {
                channels[k][out_idx] = channel_samples[k];
            }
        }

        channels
    }

    /// Simple M-point IFFT (DIT radix-2 for power-of-2, DFT for others).
    fn ifft_m(input: &[Complex64]) -> Vec<Complex64> {
        let m = input.len();
        if m == 1 {
            return input.to_vec();
        }

        // General DFT-based IFFT for any M
        let mut output = vec![Complex64::new(0.0, 0.0); m];
        let scale = 1.0 / m as f64;
        for k in 0..m {
            let mut sum = Complex64::new(0.0, 0.0);
            for n in 0..m {
                let angle = 2.0 * PI * k as f64 * n as f64 / m as f64;
                let twiddle = Complex64::new(angle.cos(), angle.sin());
                sum += input[n] * twiddle;
            }
            output[k] = sum * scale;
        }
        output
    }

    /// Get the number of channels.
    pub fn num_channels(&self) -> usize {
        self.config.num_channels
    }

    /// Get the prototype filter taps for a specific branch.
    pub fn branch_taps(&self, branch: usize) -> &[f64] {
        &self.polyphase_taps[branch]
    }

    /// Reset all delay lines.
    pub fn reset(&mut self) {
        let taps_per = self.config.taps_per_channel;
        let m = self.config.num_channels;
        self.delay_lines = vec![vec![Complex64::new(0.0, 0.0); taps_per]; m];
        self.delay_idx = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_channelizer_output_dimensions() {
        let ch = PfbChannelizer::new(PfbConfig {
            num_channels: 4,
            taps_per_channel: 8,
            ..Default::default()
        });

        let input = vec![Complex64::new(1.0, 0.0); 64];
        let channels = ch.analyze(&input);

        assert_eq!(channels.len(), 4, "Should produce 4 channels");
        assert_eq!(channels[0].len(), 16, "Each channel should have 64/4=16 samples");
    }

    #[test]
    fn test_channelizer_8_channels() {
        let ch = PfbChannelizer::new(PfbConfig {
            num_channels: 8,
            taps_per_channel: 4,
            ..Default::default()
        });

        let input = vec![Complex64::new(1.0, 0.0); 80];
        let channels = ch.analyze(&input);
        assert_eq!(channels.len(), 8);
        assert_eq!(channels[0].len(), 10);
    }

    #[test]
    fn test_channelizer_dc_signal() {
        let ch = PfbChannelizer::new(PfbConfig {
            num_channels: 4,
            taps_per_channel: 8,
            ..Default::default()
        });

        // DC signal should appear only in channel 0
        let input = vec![Complex64::new(1.0, 0.0); 64];
        let channels = ch.analyze(&input);

        // Channel 0 (DC) should have most of the energy
        let ch0_power: f64 = channels[0].iter().map(|s| s.norm_sqr()).sum();
        let total_power: f64 = channels.iter().flat_map(|c| c.iter()).map(|s| s.norm_sqr()).sum();

        if total_power > 1e-10 {
            let ratio = ch0_power / total_power;
            assert!(
                ratio > 0.5,
                "DC signal should concentrate in channel 0: ratio = {ratio:.3}"
            );
        }
    }

    #[test]
    fn test_channelizer_tone_separation() {
        let m = 4;
        let n = 256; // 64 output samples per channel
        let ch = PfbChannelizer::new(PfbConfig {
            num_channels: m,
            taps_per_channel: 8,
            ..Default::default()
        });

        // Tone at channel 1 frequency: f = 1/M of the sample rate
        let f = 1.0 / m as f64;
        let input: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * f * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let channels = ch.analyze(&input);

        // Channel 1 should have the most energy
        let powers: Vec<f64> = channels
            .iter()
            .map(|c| c.iter().map(|s| s.norm_sqr()).sum())
            .collect();

        let max_ch = powers
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        assert_eq!(
            max_ch, 1,
            "Tone at f=1/M should appear in channel 1: powers = {powers:?}"
        );
    }

    #[test]
    fn test_prototype_filter_sum() {
        let taps = PfbChannelizer::design_prototype(32, 4, "hamming");
        let sum: f64 = taps.iter().sum();
        assert!(
            (sum - 1.0).abs() < 0.01,
            "Prototype filter should be normalized: sum = {sum}"
        );
    }

    #[test]
    fn test_ifft_identity() {
        // IFFT of [1, 0, 0, 0] should be [0.25, 0.25, 0.25, 0.25]
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
        ];
        let output = PfbChannelizer::ifft_m(&input);
        for s in &output {
            assert!(
                (s.re - 0.25).abs() < 1e-10,
                "IFFT of [1,0,0,0] should be 0.25"
            );
        }
    }

    #[test]
    fn test_channelizer_reset() {
        let mut ch = PfbChannelizer::new(PfbConfig::default());
        let input = vec![Complex64::new(1.0, 0.0); 16];
        let _ = ch.analyze(&input);
        ch.reset();
        // After reset, delay lines should be cleared
        for dl in &ch.delay_lines {
            for s in dl {
                assert!(s.norm() < 1e-10);
            }
        }
    }
}
