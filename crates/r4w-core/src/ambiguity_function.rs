//! Ambiguity Function — Radar Waveform Delay-Doppler Analysis
//!
//! Computes the narrowband ambiguity function |χ(τ, ν)|² for radar
//! waveform design and analysis. The ambiguity function characterizes
//! a waveform's ability to resolve targets in range (delay τ) and
//! velocity (Doppler ν). Includes zero-delay (Doppler) cuts,
//! zero-Doppler (range) cuts, and full 2D surface computation.
//!
//! GNU Radio equivalent: no direct equivalent (radar analysis tool).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ambiguity_function::{ambiguity_surface, zero_doppler_cut};
//!
//! // Simple rectangular pulse
//! let pulse: Vec<f64> = vec![1.0; 64];
//! let range_cut = zero_doppler_cut(&pulse);
//! assert_eq!(range_cut.len(), 2 * pulse.len() - 1);
//! // Peak at zero lag
//! let peak_idx = range_cut.iter().enumerate()
//!     .max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
//! assert_eq!(peak_idx, pulse.len() - 1);
//! ```

use std::f64::consts::PI;
use num_complex::Complex64;

/// Compute the full 2D ambiguity function |χ(τ, ν)|².
///
/// Returns a 2D grid of `num_doppler` rows × `(2*N-1)` delay columns,
/// where N = signal length. Doppler bins span [-doppler_max, +doppler_max].
///
/// `fs` is the sampling frequency in Hz.
pub fn ambiguity_surface(
    signal: &[f64],
    fs: f64,
    doppler_max: f64,
    num_doppler: usize,
) -> AmbiguitySurface {
    let n = signal.len();
    let num_delay = 2 * n - 1;
    let num_doppler = num_doppler.max(1) | 1; // Ensure odd for zero-Doppler bin

    let doppler_step = 2.0 * doppler_max / (num_doppler - 1).max(1) as f64;
    let mut surface = vec![0.0; num_doppler * num_delay];

    for d_idx in 0..num_doppler {
        let doppler = -doppler_max + d_idx as f64 * doppler_step;
        let row_offset = d_idx * num_delay;

        for tau_idx in 0..num_delay {
            let tau = tau_idx as isize - (n as isize - 1);
            let mut sum = Complex64::new(0.0, 0.0);

            for k in 0..n {
                let shifted_k = k as isize + tau;
                if shifted_k >= 0 && (shifted_k as usize) < n {
                    let x1 = signal[k];
                    let x2 = signal[shifted_k as usize];
                    let phase = 2.0 * PI * doppler * k as f64 / fs;
                    sum += Complex64::new(x1 * x2, 0.0)
                        * Complex64::new(phase.cos(), phase.sin());
                }
            }
            surface[row_offset + tau_idx] = sum.norm_sqr();
        }
    }

    // Normalize to peak = 1.0
    let peak = surface.iter().cloned().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in &mut surface {
            *v /= peak;
        }
    }

    AmbiguitySurface {
        surface,
        num_delay,
        num_doppler,
        delay_min: -(n as isize - 1),
        delay_max: (n as isize - 1),
        doppler_min: -doppler_max,
        doppler_max,
        fs,
    }
}

/// Compute the zero-Doppler cut (autocorrelation): |χ(τ, 0)|².
///
/// This is the squared magnitude of the signal's autocorrelation.
pub fn zero_doppler_cut(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let num_lags = 2 * n - 1;
    let mut result = vec![0.0; num_lags];

    for lag_idx in 0..num_lags {
        let tau = lag_idx as isize - (n as isize - 1);
        let mut sum = 0.0;
        for k in 0..n {
            let j = k as isize + tau;
            if j >= 0 && (j as usize) < n {
                sum += signal[k] * signal[j as usize];
            }
        }
        result[lag_idx] = sum * sum; // |χ|²
    }

    // Normalize
    let peak = result.iter().cloned().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in &mut result {
            *v /= peak;
        }
    }
    result
}

/// Compute the zero-delay cut (Doppler resolution): |χ(0, ν)|².
///
/// `doppler_bins` Doppler frequencies spanning [-doppler_max, +doppler_max].
pub fn zero_delay_cut(signal: &[f64], fs: f64, doppler_max: f64, num_bins: usize) -> Vec<f64> {
    let n = signal.len();
    let num_bins = num_bins.max(1);
    let doppler_step = 2.0 * doppler_max / (num_bins - 1).max(1) as f64;
    let mut result = vec![0.0; num_bins];

    for d_idx in 0..num_bins {
        let doppler = -doppler_max + d_idx as f64 * doppler_step;
        let mut sum = Complex64::new(0.0, 0.0);
        for k in 0..n {
            let phase = 2.0 * PI * doppler * k as f64 / fs;
            let x = signal[k] * signal[k]; // τ=0
            sum += Complex64::new(x * phase.cos(), x * phase.sin());
        }
        result[d_idx] = sum.norm_sqr();
    }

    // Normalize
    let peak = result.iter().cloned().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in &mut result {
            *v /= peak;
        }
    }
    result
}

/// Compute the ambiguity function for a complex baseband signal.
pub fn ambiguity_surface_complex(
    signal: &[Complex64],
    fs: f64,
    doppler_max: f64,
    num_doppler: usize,
) -> AmbiguitySurface {
    let n = signal.len();
    let num_delay = 2 * n - 1;
    let num_doppler = num_doppler.max(1) | 1;

    let doppler_step = 2.0 * doppler_max / (num_doppler - 1).max(1) as f64;
    let mut surface = vec![0.0; num_doppler * num_delay];

    for d_idx in 0..num_doppler {
        let doppler = -doppler_max + d_idx as f64 * doppler_step;
        let row_offset = d_idx * num_delay;

        for tau_idx in 0..num_delay {
            let tau = tau_idx as isize - (n as isize - 1);
            let mut sum = Complex64::new(0.0, 0.0);

            for k in 0..n {
                let shifted_k = k as isize + tau;
                if shifted_k >= 0 && (shifted_k as usize) < n {
                    let x1 = signal[k];
                    let x2 = signal[shifted_k as usize].conj();
                    let phase = 2.0 * PI * doppler * k as f64 / fs;
                    sum += x1 * x2 * Complex64::new(phase.cos(), phase.sin());
                }
            }
            surface[row_offset + tau_idx] = sum.norm_sqr();
        }
    }

    let peak = surface.iter().cloned().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in &mut surface {
            *v /= peak;
        }
    }

    AmbiguitySurface {
        surface,
        num_delay,
        num_doppler,
        delay_min: -(n as isize - 1),
        delay_max: (n as isize - 1),
        doppler_min: -doppler_max,
        doppler_max,
        fs,
    }
}

/// Result of a 2D ambiguity function computation.
#[derive(Debug, Clone)]
pub struct AmbiguitySurface {
    /// Flattened 2D surface [doppler_idx * num_delay + delay_idx].
    pub surface: Vec<f64>,
    /// Number of delay bins.
    pub num_delay: usize,
    /// Number of Doppler bins.
    pub num_doppler: usize,
    /// Minimum delay in samples.
    pub delay_min: isize,
    /// Maximum delay in samples.
    pub delay_max: isize,
    /// Minimum Doppler in Hz.
    pub doppler_min: f64,
    /// Maximum Doppler in Hz.
    pub doppler_max: f64,
    /// Sampling frequency.
    pub fs: f64,
}

impl AmbiguitySurface {
    /// Get the value at a specific (delay_idx, doppler_idx).
    pub fn get(&self, delay_idx: usize, doppler_idx: usize) -> f64 {
        if delay_idx < self.num_delay && doppler_idx < self.num_doppler {
            self.surface[doppler_idx * self.num_delay + delay_idx]
        } else {
            0.0
        }
    }

    /// Extract the zero-Doppler row (range cut).
    pub fn range_cut(&self) -> Vec<f64> {
        let mid_doppler = self.num_doppler / 2;
        let offset = mid_doppler * self.num_delay;
        self.surface[offset..offset + self.num_delay].to_vec()
    }

    /// Extract the zero-delay column (Doppler cut).
    pub fn doppler_cut(&self) -> Vec<f64> {
        let mid_delay = self.num_delay / 2;
        (0..self.num_doppler)
            .map(|d| self.surface[d * self.num_delay + mid_delay])
            .collect()
    }

    /// Get the delay axis in samples.
    pub fn delay_axis_samples(&self) -> Vec<isize> {
        (self.delay_min..=self.delay_max).collect()
    }

    /// Get the delay axis in seconds.
    pub fn delay_axis_seconds(&self) -> Vec<f64> {
        (self.delay_min..=self.delay_max)
            .map(|d| d as f64 / self.fs)
            .collect()
    }

    /// Get the Doppler axis in Hz.
    pub fn doppler_axis_hz(&self) -> Vec<f64> {
        let step = (self.doppler_max - self.doppler_min) / (self.num_doppler - 1).max(1) as f64;
        (0..self.num_doppler)
            .map(|i| self.doppler_min + i as f64 * step)
            .collect()
    }

    /// Find the peak location (delay_idx, doppler_idx, value).
    pub fn find_peak(&self) -> (usize, usize, f64) {
        let mut max_val = 0.0;
        let mut max_delay = 0;
        let mut max_doppler = 0;
        for d in 0..self.num_doppler {
            for t in 0..self.num_delay {
                let v = self.surface[d * self.num_delay + t];
                if v > max_val {
                    max_val = v;
                    max_delay = t;
                    max_doppler = d;
                }
            }
        }
        (max_delay, max_doppler, max_val)
    }

    /// Compute the -3 dB mainlobe width in delay bins.
    pub fn range_resolution_bins(&self) -> f64 {
        let cut = self.range_cut();
        mainlobe_width(&cut, 0.5)
    }

    /// Compute the -3 dB mainlobe width in Doppler bins.
    pub fn doppler_resolution_bins(&self) -> f64 {
        let cut = self.doppler_cut();
        mainlobe_width(&cut, 0.5)
    }
}

/// Compute mainlobe width at a given threshold (0..1 of peak).
fn mainlobe_width(cut: &[f64], threshold: f64) -> f64 {
    let peak_idx = cut.iter().enumerate()
        .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    // Search left
    let mut left = peak_idx;
    while left > 0 && cut[left] > threshold {
        left -= 1;
    }

    // Search right
    let mut right = peak_idx;
    while right < cut.len() - 1 && cut[right] > threshold {
        right += 1;
    }

    (right - left) as f64
}

/// Generate a Linear Frequency Modulated (LFM) chirp for testing.
///
/// Returns a real-valued chirp sweeping from 0 to `bandwidth` Hz over `n` samples.
pub fn lfm_chirp(n: usize, bandwidth: f64, fs: f64) -> Vec<f64> {
    let chirp_rate = bandwidth / (n as f64 / fs);
    (0..n)
        .map(|i| {
            let t = i as f64 / fs;
            (2.0 * PI * (0.5 * chirp_rate * t * t)).cos()
        })
        .collect()
}

/// Generate a Barker code pulse (length 13).
pub fn barker_13() -> Vec<f64> {
    vec![1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0]
}

/// Generate a Barker code of given length.
///
/// Valid lengths: 2, 3, 4, 5, 7, 11, 13.
pub fn barker_code(len: usize) -> Option<Vec<f64>> {
    match len {
        2 => Some(vec![1.0, -1.0]),
        3 => Some(vec![1.0, 1.0, -1.0]),
        4 => Some(vec![1.0, 1.0, -1.0, 1.0]),
        5 => Some(vec![1.0, 1.0, 1.0, -1.0, 1.0]),
        7 => Some(vec![1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0]),
        11 => Some(vec![1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0]),
        13 => Some(barker_13()),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_doppler_cut_rect_pulse() {
        let pulse = vec![1.0; 32];
        let cut = zero_doppler_cut(&pulse);
        assert_eq!(cut.len(), 63);
        // Peak at center
        let peak_idx = cut.iter().enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap().0;
        assert_eq!(peak_idx, 31);
        assert!((cut[31] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_zero_doppler_cut_symmetry() {
        let pulse = vec![1.0; 16];
        let cut = zero_doppler_cut(&pulse);
        // Autocorrelation of real symmetric pulse is symmetric
        for i in 0..cut.len() / 2 {
            assert!((cut[i] - cut[cut.len() - 1 - i]).abs() < 1e-10,
                "asymmetry at i={i}");
        }
    }

    #[test]
    fn test_zero_delay_cut_peak() {
        let pulse = vec![1.0; 64];
        let cut = zero_delay_cut(&pulse, 1000.0, 500.0, 101);
        // Peak should be at zero Doppler (center bin)
        let peak_idx = cut.iter().enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap().0;
        assert_eq!(peak_idx, 50);
    }

    #[test]
    fn test_ambiguity_surface_peak_at_origin() {
        let pulse = vec![1.0; 16];
        let surface = ambiguity_surface(&pulse, 1000.0, 100.0, 11);
        let (delay_idx, doppler_idx, val) = surface.find_peak();
        // Peak at zero delay, zero Doppler
        assert_eq!(delay_idx, 15); // Center of delay axis
        assert_eq!(doppler_idx, 5); // Center of Doppler axis
        assert!((val - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_ambiguity_surface_dimensions() {
        let pulse = vec![1.0; 20];
        let surface = ambiguity_surface(&pulse, 1000.0, 500.0, 21);
        assert_eq!(surface.num_delay, 39);
        assert_eq!(surface.num_doppler, 21);
        assert_eq!(surface.surface.len(), 21 * 39);
    }

    #[test]
    fn test_range_cut_extraction() {
        let pulse = vec![1.0; 16];
        let surface = ambiguity_surface(&pulse, 1000.0, 100.0, 11);
        let range_cut = surface.range_cut();
        assert_eq!(range_cut.len(), 31);
        // Should match zero_doppler_cut approximately
        assert!((range_cut[15] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_doppler_cut_extraction() {
        let pulse = vec![1.0; 16];
        let surface = ambiguity_surface(&pulse, 1000.0, 100.0, 11);
        let doppler_cut = surface.doppler_cut();
        assert_eq!(doppler_cut.len(), 11);
    }

    #[test]
    fn test_lfm_chirp_generation() {
        let chirp = lfm_chirp(128, 500.0, 1000.0);
        assert_eq!(chirp.len(), 128);
        // Values should be in [-1, 1]
        assert!(chirp.iter().all(|&v| v >= -1.0 && v <= 1.0));
    }

    #[test]
    fn test_lfm_narrower_range_cut() {
        // LFM chirp should have narrower range cut than rectangular pulse of same length
        let n = 64;
        let rect_cut = zero_doppler_cut(&vec![1.0; n]);
        let lfm = lfm_chirp(n, 500.0, 1000.0);
        let lfm_cut = zero_doppler_cut(&lfm);
        // LFM -3dB width should be narrower
        let rect_width = mainlobe_width(&rect_cut, 0.5);
        let lfm_width = mainlobe_width(&lfm_cut, 0.5);
        assert!(lfm_width < rect_width,
            "LFM width ({lfm_width}) should be < rect width ({rect_width})");
    }

    #[test]
    fn test_barker_codes() {
        assert!(barker_code(13).is_some());
        assert!(barker_code(7).is_some());
        assert!(barker_code(6).is_none());
        let b13 = barker_13();
        assert_eq!(b13.len(), 13);
        // Barker code values are ±1
        assert!(b13.iter().all(|&v| v == 1.0 || v == -1.0));
    }

    #[test]
    fn test_barker_low_sidelobes() {
        // Barker-13 autocorrelation sidelobes should be ≤ 1/13 of peak
        let b13 = barker_13();
        let cut = zero_doppler_cut(&b13);
        let peak = cut[12]; // Center
        for (i, &v) in cut.iter().enumerate() {
            if i != 12 {
                // Sidelobes of |R(τ)|² ≤ 1/N² relative to peak
                assert!(v <= peak * 0.1 + 0.01,
                    "sidelobe at {i} = {v} too high (peak={peak})");
            }
        }
    }

    #[test]
    fn test_complex_ambiguity() {
        let pulse: Vec<Complex64> = (0..32)
            .map(|i| Complex64::new((i as f64 * 0.2).cos(), (i as f64 * 0.2).sin()))
            .collect();
        let surface = ambiguity_surface_complex(&pulse, 1000.0, 100.0, 11);
        let (_, _, peak) = surface.find_peak();
        assert!((peak - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_axis_generation() {
        let pulse = vec![1.0; 10];
        let surface = ambiguity_surface(&pulse, 1000.0, 500.0, 5);
        let delays = surface.delay_axis_samples();
        assert_eq!(delays.len(), 19);
        assert_eq!(delays[0], -9);
        assert_eq!(delays[18], 9);
        let times = surface.delay_axis_seconds();
        assert!((times[9] - 0.0).abs() < 1e-10); // Zero delay
        let dopplers = surface.doppler_axis_hz();
        assert_eq!(dopplers.len(), 5);
    }
}
