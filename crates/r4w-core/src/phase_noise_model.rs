//! Phase Noise Model — Oscillator Impairment Synthesis
//!
//! Generates realistic phase noise from L(f) PSD masks for oscillator
//! simulation. Supports Leeson's model, crystal oscillator profiles,
//! and custom PSD masks. Applies phase noise to IQ signals for
//! system-level fidelity testing.
//!
//! GNU Radio equivalent: `channels::phase_noise_gen`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::phase_noise_model::{PhaseNoiseModel, PhaseNoiseProfile};
//!
//! let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1e3, 1e4], &[-80.0, -110.0, -130.0]);
//! let mut model = PhaseNoiseModel::new(1e6, profile, Some(42));
//! let noise = model.generate(1000);
//! assert_eq!(noise.len(), 1000);
//! ```

use std::f64::consts::PI;

/// Phase noise spectral profile.
#[derive(Debug, Clone)]
pub enum PhaseNoiseProfile {
    /// Custom PSD mask: (offset_hz, level_dBc_Hz) pairs.
    PsdMask {
        offsets: Vec<f64>,
        levels: Vec<f64>,
    },
    /// Leeson's oscillator model.
    Leeson {
        center_freq: f64,
        q_loaded: f64,
        flicker_corner: f64,
        noise_floor_dbchz: f64,
    },
}

impl PhaseNoiseProfile {
    /// Create a PSD mask profile.
    pub fn psd_mask(offsets: &[f64], levels: &[f64]) -> Self {
        Self::PsdMask {
            offsets: offsets.to_vec(),
            levels: levels.to_vec(),
        }
    }

    /// Create a Leeson model profile.
    pub fn leeson(center_freq: f64, q_loaded: f64, flicker_corner: f64, noise_floor: f64) -> Self {
        Self::Leeson {
            center_freq,
            q_loaded,
            flicker_corner,
            noise_floor_dbchz: noise_floor,
        }
    }

    /// Evaluate L(f) in dBc/Hz at a given offset frequency.
    pub fn evaluate(&self, offset_hz: f64) -> f64 {
        let offset = offset_hz.abs().max(1e-3);
        match self {
            Self::PsdMask { offsets, levels } => {
                if offsets.is_empty() {
                    return -150.0;
                }
                // Log-linear interpolation
                if offset <= offsets[0] {
                    return levels[0];
                }
                if offset >= offsets[offsets.len() - 1] {
                    return levels[levels.len() - 1];
                }
                for i in 0..offsets.len() - 1 {
                    if offset >= offsets[i] && offset <= offsets[i + 1] {
                        let log_ratio = (offset / offsets[i]).ln()
                            / (offsets[i + 1] / offsets[i]).ln();
                        return levels[i] + (levels[i + 1] - levels[i]) * log_ratio;
                    }
                }
                levels[levels.len() - 1]
            }
            Self::Leeson {
                center_freq,
                q_loaded,
                flicker_corner,
                noise_floor_dbchz,
            } => {
                let f_l = center_freq / (2.0 * q_loaded);
                let mut l_f = *noise_floor_dbchz;
                // 1/f^2 region (Leeson)
                if offset < f_l {
                    l_f += 20.0 * (f_l / offset).log10();
                }
                // 1/f^3 region (flicker)
                if offset < *flicker_corner {
                    l_f += 10.0 * (flicker_corner / offset).log10();
                }
                l_f
            }
        }
    }
}

/// Phase noise generator.
#[derive(Debug, Clone)]
pub struct PhaseNoiseModel {
    sample_rate: f64,
    profile: PhaseNoiseProfile,
    rng_state: u64,
    filter_state: Vec<f64>,
}

impl PhaseNoiseModel {
    /// Create a new phase noise model.
    ///
    /// `sample_rate`: signal sample rate (Hz).
    /// `profile`: phase noise spectral profile.
    /// `seed`: optional RNG seed for reproducibility.
    pub fn new(sample_rate: f64, profile: PhaseNoiseProfile, seed: Option<u64>) -> Self {
        Self {
            sample_rate,
            profile,
            rng_state: seed.unwrap_or(123456789),
            filter_state: vec![0.0; 8],
        }
    }

    /// Generate phase noise time series (radians).
    pub fn generate(&mut self, num_samples: usize) -> Vec<f64> {
        // Generate shaped noise via spectral shaping
        let fft_size = num_samples.next_power_of_two();

        // Generate white Gaussian noise in frequency domain
        let mut freq_domain: Vec<(f64, f64)> = Vec::with_capacity(fft_size);
        for k in 0..fft_size {
            let n1 = self.randn();
            let n2 = self.randn();

            // Compute PSD at this frequency bin
            let f_offset = if k <= fft_size / 2 {
                k as f64 * self.sample_rate / fft_size as f64
            } else {
                (fft_size - k) as f64 * self.sample_rate / fft_size as f64
            };

            let f_offset = f_offset.max(1.0); // avoid DC
            let l_f_db = self.profile.evaluate(f_offset);
            let l_f_linear = 10.0_f64.powf(l_f_db / 10.0);
            let amplitude = l_f_linear.sqrt();

            freq_domain.push((n1 * amplitude, n2 * amplitude));
        }

        // Force DC to zero (no phase offset, just noise)
        freq_domain[0] = (0.0, 0.0);

        // IFFT to get time-domain phase noise
        let time_domain = ifft(&freq_domain);
        let scale = (self.sample_rate / fft_size as f64).sqrt();

        time_domain
            .iter()
            .take(num_samples)
            .map(|(r, _)| r * scale)
            .collect()
    }

    /// Apply phase noise to an IQ signal in-place.
    ///
    /// Multiplies each sample by exp(j * phi_noise(n)).
    pub fn apply_to_signal(&mut self, signal: &mut [(f64, f64)]) {
        let noise = self.generate(signal.len());
        for (i, s) in signal.iter_mut().enumerate() {
            let phi = noise[i];
            let (cos_phi, sin_phi) = (phi.cos(), phi.sin());
            let (sr, si) = *s;
            s.0 = sr * cos_phi - si * sin_phi;
            s.1 = sr * sin_phi + si * cos_phi;
        }
    }

    /// Query L(f) at a given offset frequency (dBc/Hz).
    pub fn psd_at_offset(&self, offset_hz: f64) -> f64 {
        self.profile.evaluate(offset_hz)
    }

    /// Compute integrated phase noise (radians RMS) over frequency range.
    pub fn integrated_phase_noise(&self, f_low: f64, f_high: f64) -> f64 {
        let num_points = 1000;
        let log_low = f_low.max(1.0).ln();
        let log_high = f_high.max(f_low + 1.0).ln();
        let mut integral = 0.0;

        for i in 0..num_points {
            let f = ((log_low + (log_high - log_low) * i as f64 / num_points as f64).exp());
            let l_f = 10.0_f64.powf(self.profile.evaluate(f) / 10.0);
            let df = f * (log_high - log_low) / num_points as f64;
            integral += l_f * df;
        }

        (2.0 * integral).sqrt() // radians RMS
    }

    fn randn(&mut self) -> f64 {
        // Box-Muller from LCG
        self.rng_state = self.rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u1 = (self.rng_state >> 33) as f64 / (1u64 << 31) as f64;
        self.rng_state = self.rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let u2 = (self.rng_state >> 33) as f64 / (1u64 << 31) as f64;
        let u1 = u1.max(1e-10);
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

fn ifft(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    let result = fft_core(x, true);
    result
        .iter()
        .map(|(r, i)| (r / n as f64, i / n as f64))
        .collect()
}

fn fft_core(x: &[(f64, f64)], inverse: bool) -> Vec<(f64, f64)> {
    let n = x.len();
    if n <= 1 {
        return x.to_vec();
    }
    let even: Vec<(f64, f64)> = x.iter().step_by(2).cloned().collect();
    let odd: Vec<(f64, f64)> = x.iter().skip(1).step_by(2).cloned().collect();
    let even_fft = fft_core(&even, inverse);
    let odd_fft = fft_core(&odd, inverse);
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n / 2 {
        let angle = sign * 2.0 * PI * k as f64 / n as f64;
        let tw = (angle.cos(), angle.sin());
        let (or, oi) = odd_fft[k];
        let prod = (tw.0 * or - tw.1 * oi, tw.0 * oi + tw.1 * or);
        result[k] = (even_fft[k].0 + prod.0, even_fft[k].1 + prod.1);
        result[k + n / 2] = (even_fft[k].0 - prod.0, even_fft[k].1 - prod.1);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_length() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1e3], &[-80.0, -110.0]);
        let mut model = PhaseNoiseModel::new(1e6, profile, Some(42));
        let noise = model.generate(1000);
        assert_eq!(noise.len(), 1000);
    }

    #[test]
    fn test_zero_mean() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1e3, 1e4], &[-80.0, -110.0, -130.0]);
        let mut model = PhaseNoiseModel::new(1e6, profile, Some(42));
        let noise = model.generate(10000);
        let mean = noise.iter().sum::<f64>() / noise.len() as f64;
        assert!(mean.abs() < 0.1, "mean={mean}, expected ~0");
    }

    #[test]
    fn test_psd_mask_evaluation() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1000.0], &[-80.0, -110.0]);
        let l_100 = profile.evaluate(100.0);
        let l_1k = profile.evaluate(1000.0);
        assert!((l_100 - (-80.0)).abs() < 0.01);
        assert!((l_1k - (-110.0)).abs() < 0.01);
    }

    #[test]
    fn test_psd_interpolation() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 10000.0], &[-80.0, -120.0]);
        let l_mid = profile.evaluate(1000.0);
        // Should be between -80 and -120 (log-linear interpolation)
        assert!(l_mid > -120.0 && l_mid < -80.0, "l_mid={l_mid}");
    }

    #[test]
    fn test_leeson_model() {
        let profile = PhaseNoiseProfile::leeson(1e9, 100.0, 1e3, -150.0);
        let l_close = profile.evaluate(10.0);
        let l_far = profile.evaluate(1e6);
        // Close-in should be much higher than far-out
        assert!(l_close > l_far, "close={l_close}, far={l_far}");
    }

    #[test]
    fn test_apply_preserves_amplitude() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1e4], &[-80.0, -120.0]);
        let mut model = PhaseNoiseModel::new(1e6, profile, Some(42));
        let mut signal: Vec<(f64, f64)> = (0..100).map(|_| (1.0, 0.0)).collect();
        model.apply_to_signal(&mut signal);
        for &(r, i) in &signal {
            let mag = (r * r + i * i).sqrt();
            assert!(
                (mag - 1.0).abs() < 0.01,
                "amplitude changed: {mag}"
            );
        }
    }

    #[test]
    fn test_seed_reproducibility() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0], &[-80.0]);
        let mut m1 = PhaseNoiseModel::new(1e6, profile.clone(), Some(42));
        let mut m2 = PhaseNoiseModel::new(1e6, profile, Some(42));
        let n1 = m1.generate(100);
        let n2 = m2.generate(100);
        for i in 0..100 {
            assert!((n1[i] - n2[i]).abs() < 1e-15, "seed mismatch at {i}");
        }
    }

    #[test]
    fn test_different_seeds() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0], &[-80.0]);
        let mut m1 = PhaseNoiseModel::new(1e6, profile.clone(), Some(42));
        let mut m2 = PhaseNoiseModel::new(1e6, profile, Some(99));
        let n1 = m1.generate(100);
        let n2 = m2.generate(100);
        let diff: f64 = n1.iter().zip(n2.iter()).map(|(a, b)| (a - b).abs()).sum();
        assert!(diff > 0.01, "different seeds should produce different noise");
    }

    #[test]
    fn test_integrated_phase_noise() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1e4], &[-80.0, -120.0]);
        let model = PhaseNoiseModel::new(1e6, profile, None);
        let jitter = model.integrated_phase_noise(100.0, 10000.0);
        assert!(jitter > 0.0 && jitter.is_finite(), "jitter={jitter}");
    }

    #[test]
    fn test_psd_at_offset() {
        let profile = PhaseNoiseProfile::psd_mask(&[100.0, 1e3], &[-80.0, -110.0]);
        let model = PhaseNoiseModel::new(1e6, profile, None);
        let l = model.psd_at_offset(100.0);
        assert!((l - (-80.0)).abs() < 0.01);
    }

    #[test]
    fn test_all_finite() {
        let profile = PhaseNoiseProfile::psd_mask(&[1.0, 1e6], &[-60.0, -160.0]);
        let mut model = PhaseNoiseModel::new(1e6, profile, Some(7));
        let noise = model.generate(1024);
        assert!(noise.iter().all(|x| x.is_finite()));
    }
}
