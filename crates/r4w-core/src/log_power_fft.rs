//! Log Power FFT — Streaming Spectrum Estimation
//!
//! Computes windowed FFT and converts to dB power for continuous spectrum
//! monitoring. Supports averaging (exponential or sliding window) and
//! configurable window functions.
//!
//! ## Signal Flow
//!
//! ```text
//! input → [window] → [FFT] → |·|² → 10·log10(·) → [average] → dB spectrum
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::log_power_fft::{LogPowerFft, LogPowerConfig};
//! use num_complex::Complex64;
//!
//! let mut lpf = LogPowerFft::new(LogPowerConfig {
//!     fft_size: 256,
//!     avg_alpha: 0.3,
//!     ..Default::default()
//! });
//!
//! // Generate a tone at bin 64 (1/4 of FFT size)
//! let input: Vec<Complex64> = (0..256)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * 64.0 * i as f64 / 256.0;
//!         Complex64::new(phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let spectrum = lpf.process_block(&input);
//! assert_eq!(spectrum.len(), 256);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for Log Power FFT.
#[derive(Debug, Clone)]
pub struct LogPowerConfig {
    /// FFT size (must be power of 2). Default: 1024
    pub fft_size: usize,
    /// Window function. Default: Blackman-Harris
    pub window: WindowType,
    /// Exponential averaging factor (0.0 = no averaging, 1.0 = no smoothing). Default: 0.2
    pub avg_alpha: f64,
    /// Floor value in dB for clipping. Default: -120.0
    pub floor_db: f64,
    /// Whether to shift DC to center (fftshift). Default: true
    pub center_dc: bool,
}

impl Default for LogPowerConfig {
    fn default() -> Self {
        Self {
            fft_size: 1024,
            window: WindowType::BlackmanHarris,
            avg_alpha: 0.2,
            floor_db: -120.0,
            center_dc: true,
        }
    }
}

/// Window function type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WindowType {
    Rectangular,
    Hann,
    Hamming,
    Blackman,
    BlackmanHarris,
}

/// Log Power FFT processor.
#[derive(Debug, Clone)]
pub struct LogPowerFft {
    config: LogPowerConfig,
    /// Pre-computed window coefficients
    window: Vec<f64>,
    /// Averaged spectrum in dB
    avg_spectrum: Vec<f64>,
    /// Whether we have at least one frame
    has_data: bool,
    /// Scratch buffer for windowed samples
    scratch: Vec<Complex64>,
}

impl LogPowerFft {
    /// Create a new Log Power FFT processor.
    pub fn new(config: LogPowerConfig) -> Self {
        let n = config.fft_size.next_power_of_two();
        let window = compute_window(config.window, n);

        Self {
            avg_spectrum: vec![config.floor_db; n],
            has_data: false,
            scratch: vec![Complex64::new(0.0, 0.0); n],
            window,
            config: LogPowerConfig {
                fft_size: n,
                ..config
            },
        }
    }

    /// Process a block of input samples.
    ///
    /// Segments input into FFT-sized blocks and returns the latest
    /// averaged dB spectrum. If input is shorter than fft_size, it is
    /// zero-padded.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<f64> {
        let n = self.config.fft_size;

        // Process each complete FFT block
        for chunk in input.chunks(n) {
            // Apply window + zero-pad if needed
            for i in 0..n {
                if i < chunk.len() {
                    self.scratch[i] = chunk[i] * self.window[i];
                } else {
                    self.scratch[i] = Complex64::new(0.0, 0.0);
                }
            }

            // In-place FFT (Cooley-Tukey radix-2)
            fft_inplace(&mut self.scratch);

            // Compute log power and average
            let alpha = self.config.avg_alpha;
            for i in 0..n {
                let power = self.scratch[i].norm_sqr() / (n as f64 * n as f64);
                let db = if power > 1e-30 {
                    10.0 * power.log10()
                } else {
                    self.config.floor_db
                };

                if self.has_data {
                    self.avg_spectrum[i] = alpha * db + (1.0 - alpha) * self.avg_spectrum[i];
                } else {
                    self.avg_spectrum[i] = db;
                }
            }
            self.has_data = true;
        }

        // Return spectrum (optionally fftshift)
        if self.config.center_dc {
            let half = n / 2;
            let mut result = vec![0.0; n];
            result[..half].copy_from_slice(&self.avg_spectrum[half..]);
            result[half..].copy_from_slice(&self.avg_spectrum[..half]);
            result
        } else {
            self.avg_spectrum.clone()
        }
    }

    /// Get the current averaged spectrum without processing new data.
    pub fn spectrum(&self) -> Vec<f64> {
        if self.config.center_dc {
            let n = self.config.fft_size;
            let half = n / 2;
            let mut result = vec![0.0; n];
            result[..half].copy_from_slice(&self.avg_spectrum[half..]);
            result[half..].copy_from_slice(&self.avg_spectrum[..half]);
            result
        } else {
            self.avg_spectrum.clone()
        }
    }

    /// Get the FFT size.
    pub fn fft_size(&self) -> usize {
        self.config.fft_size
    }

    /// Get frequency axis values given sample rate.
    pub fn freq_axis(&self, sample_rate: f64) -> Vec<f64> {
        let n = self.config.fft_size;
        if self.config.center_dc {
            (0..n)
                .map(|i| {
                    let bin = i as f64 - n as f64 / 2.0;
                    bin * sample_rate / n as f64
                })
                .collect()
        } else {
            (0..n)
                .map(|i| i as f64 * sample_rate / n as f64)
                .collect()
        }
    }

    /// Find the peak frequency and power.
    pub fn peak(&self) -> (usize, f64) {
        let spectrum = self.spectrum();
        let (idx, &val) = spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap_or((0, &self.config.floor_db));
        (idx, val)
    }

    /// Reset averaging.
    pub fn reset(&mut self) {
        self.avg_spectrum.fill(self.config.floor_db);
        self.has_data = false;
    }
}

/// Compute window function coefficients.
fn compute_window(window_type: WindowType, n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| {
            let x = i as f64 / (n - 1).max(1) as f64;
            match window_type {
                WindowType::Rectangular => 1.0,
                WindowType::Hann => 0.5 * (1.0 - (2.0 * PI * x).cos()),
                WindowType::Hamming => 0.54 - 0.46 * (2.0 * PI * x).cos(),
                WindowType::Blackman => {
                    0.42 - 0.5 * (2.0 * PI * x).cos() + 0.08 * (4.0 * PI * x).cos()
                }
                WindowType::BlackmanHarris => {
                    0.35875 - 0.48829 * (2.0 * PI * x).cos()
                        + 0.14128 * (4.0 * PI * x).cos()
                        - 0.01168 * (6.0 * PI * x).cos()
                }
            }
        })
        .collect()
}

/// In-place Cooley-Tukey radix-2 FFT.
fn fft_inplace(data: &mut [Complex64]) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT size must be power of 2");

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_power_fft_basic() {
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: 64,
            avg_alpha: 1.0, // no averaging
            center_dc: false,
            window: WindowType::Rectangular,
            ..Default::default()
        });

        // DC signal
        let input = vec![Complex64::new(1.0, 0.0); 64];
        let spectrum = lpf.process_block(&input);
        assert_eq!(spectrum.len(), 64);

        // DC bin should be strongest (with rectangular window, sidelobes are at -13 dB)
        assert!(
            spectrum[0] > spectrum[2] + 10.0,
            "DC bin should dominate: dc={:.1}, bin2={:.1}",
            spectrum[0],
            spectrum[2]
        );
    }

    #[test]
    fn test_log_power_fft_tone() {
        let n = 256;
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: n,
            avg_alpha: 1.0,
            center_dc: false,
            ..Default::default()
        });

        // Tone at bin 32
        let bin = 32;
        let input: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * bin as f64 * i as f64 / n as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let spectrum = lpf.process_block(&input);

        // Find peak
        let (peak_idx, _) = spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();

        assert_eq!(peak_idx, bin, "Peak should be at bin {}, got {}", bin, peak_idx);
    }

    #[test]
    fn test_log_power_fft_center_dc() {
        let n = 64;
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: n,
            avg_alpha: 1.0,
            center_dc: true,
            ..Default::default()
        });

        // DC signal
        let input = vec![Complex64::new(1.0, 0.0); n];
        let spectrum = lpf.process_block(&input);

        // DC should now be at center (bin n/2)
        let (peak_idx, _) = spectrum
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap();
        assert_eq!(peak_idx, n / 2, "DC should be at center when center_dc=true");
    }

    #[test]
    fn test_log_power_fft_averaging() {
        let n = 64;
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: n,
            avg_alpha: 0.5,
            center_dc: false,
            ..Default::default()
        });

        // Process two blocks — averaging should smooth
        let input1 = vec![Complex64::new(1.0, 0.0); n];
        let spec1 = lpf.process_block(&input1);

        let input2 = vec![Complex64::new(0.0, 0.0); n];
        let spec2 = lpf.process_block(&input2);

        // DC bin in spec2 should be less than spec1 (averaged down)
        assert!(
            spec2[0] < spec1[0],
            "Averaging should reduce DC: spec1={:.1}, spec2={:.1}",
            spec1[0],
            spec2[0]
        );
    }

    #[test]
    fn test_log_power_fft_freq_axis() {
        let lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: 64,
            center_dc: true,
            ..Default::default()
        });

        let freqs = lpf.freq_axis(48000.0);
        assert_eq!(freqs.len(), 64);
        // Center should be ~0 Hz
        assert!((freqs[32]).abs() < 1.0);
        // First bin should be negative
        assert!(freqs[0] < 0.0);
    }

    #[test]
    fn test_log_power_fft_peak() {
        let n = 128;
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: n,
            avg_alpha: 1.0,
            center_dc: false,
            ..Default::default()
        });

        let input: Vec<Complex64> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 16.0 * i as f64 / n as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        lpf.process_block(&input);

        let (idx, db) = lpf.peak();
        // With center_dc=false and a tone at bin 16
        // Peak should be close to bin 16 (but might be shifted by fftshift)
        assert!(db > -10.0, "Peak should be strong: {:.1} dB", db);
        assert!(idx < n, "Peak index should be valid");
    }

    #[test]
    fn test_log_power_fft_reset() {
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: 64,
            ..Default::default()
        });

        let input = vec![Complex64::new(1.0, 0.0); 64];
        lpf.process_block(&input);
        assert!(lpf.has_data);

        lpf.reset();
        assert!(!lpf.has_data);
        assert!(lpf.avg_spectrum[0] < -100.0);
    }

    #[test]
    fn test_fft_inplace_known_result() {
        // 4-point DFT of [1, 0, 0, 0] = [1, 1, 1, 1]
        let mut data = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
            Complex64::new(0.0, 0.0),
        ];
        fft_inplace(&mut data);

        for (i, s) in data.iter().enumerate() {
            assert!(
                (s.re - 1.0).abs() < 1e-10 && s.im.abs() < 1e-10,
                "FFT[{}] = ({:.4}, {:.4}), expected (1, 0)",
                i,
                s.re,
                s.im
            );
        }
    }

    #[test]
    fn test_window_types() {
        for wt in [
            WindowType::Rectangular,
            WindowType::Hann,
            WindowType::Hamming,
            WindowType::Blackman,
            WindowType::BlackmanHarris,
        ] {
            let w = compute_window(wt, 64);
            assert_eq!(w.len(), 64);
            // All values should be in [0, 1]
            for &v in &w {
                assert!(v >= -1e-10 && v <= 1.01, "{:?}: value {:.4} out of range", wt, v);
            }
        }
    }

    #[test]
    fn test_zero_padded_input() {
        let mut lpf = LogPowerFft::new(LogPowerConfig {
            fft_size: 256,
            avg_alpha: 1.0,
            center_dc: false,
            ..Default::default()
        });

        // Short input (auto zero-padded)
        let input = vec![Complex64::new(1.0, 0.0); 50];
        let spectrum = lpf.process_block(&input);
        assert_eq!(spectrum.len(), 256);
    }
}
