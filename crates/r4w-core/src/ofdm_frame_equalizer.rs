//! OFDM Frame Equalizer — Per-Subcarrier Channel Estimation and Equalization
//!
//! Performs channel estimation from pilot subcarriers and equalizes
//! data subcarriers in OFDM frames. Supports zero-forcing and MMSE
//! equalization with linear/nearest-neighbor pilot interpolation.
//! GNU Radio equivalent: `gr::digital::ofdm_frame_equalizer_vcvc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm_frame_equalizer::{OfdmFrameEqualizer, EqualizerType, PilotPattern};
//! use num_complex::Complex64;
//!
//! let pilots = PilotPattern::uniform(64, 8, Complex64::new(1.0, 0.0));
//! let mut eq = OfdmFrameEqualizer::new(64, EqualizerType::ZeroForcing, pilots);
//! let rx_symbol: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 64];
//! let equalized = eq.equalize_symbol(&rx_symbol);
//! assert_eq!(equalized.len(), 64);
//! ```

use num_complex::Complex64;

/// Equalization algorithm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EqualizerType {
    /// Zero-forcing: Y/H.
    ZeroForcing,
    /// MMSE: H* / (|H|^2 + sigma^2).
    Mmse { noise_var: f64 },
}

/// Pilot subcarrier pattern.
#[derive(Debug, Clone)]
pub struct PilotPattern {
    /// Pilot subcarrier indices.
    pub pilot_indices: Vec<usize>,
    /// Known pilot values.
    pub pilot_values: Vec<Complex64>,
    /// Data subcarrier indices (derived).
    pub data_indices: Vec<usize>,
}

impl PilotPattern {
    /// Create a uniform pilot pattern.
    ///
    /// Places pilots every `spacing` subcarriers.
    pub fn uniform(fft_size: usize, spacing: usize, pilot_value: Complex64) -> Self {
        let mut pilot_indices = Vec::new();
        let mut pilot_values = Vec::new();
        let mut data_indices = Vec::new();

        for i in 0..fft_size {
            if i % spacing == 0 {
                pilot_indices.push(i);
                pilot_values.push(pilot_value);
            } else {
                data_indices.push(i);
            }
        }

        Self {
            pilot_indices,
            pilot_values,
            data_indices,
        }
    }

    /// Create from explicit pilot positions.
    pub fn from_indices(
        fft_size: usize,
        pilot_indices: Vec<usize>,
        pilot_values: Vec<Complex64>,
    ) -> Self {
        let mut data_indices = Vec::new();
        for i in 0..fft_size {
            if !pilot_indices.contains(&i) {
                data_indices.push(i);
            }
        }
        Self {
            pilot_indices,
            pilot_values,
            data_indices,
        }
    }

    /// Number of pilots.
    pub fn num_pilots(&self) -> usize {
        self.pilot_indices.len()
    }

    /// Number of data subcarriers.
    pub fn num_data(&self) -> usize {
        self.data_indices.len()
    }
}

/// Channel estimate for one OFDM symbol.
#[derive(Debug, Clone)]
pub struct ChannelEstimate {
    /// Per-subcarrier channel response.
    pub h: Vec<Complex64>,
    /// Average SNR estimate (dB).
    pub avg_snr_db: f64,
}

/// OFDM frame equalizer.
#[derive(Debug, Clone)]
pub struct OfdmFrameEqualizer {
    /// FFT size.
    fft_size: usize,
    /// Equalization type.
    eq_type: EqualizerType,
    /// Pilot pattern.
    pilots: PilotPattern,
    /// Current channel estimate.
    channel_est: Vec<Complex64>,
    /// Whether channel has been estimated.
    has_estimate: bool,
}

impl OfdmFrameEqualizer {
    /// Create a new OFDM frame equalizer.
    pub fn new(fft_size: usize, eq_type: EqualizerType, pilots: PilotPattern) -> Self {
        Self {
            fft_size,
            eq_type,
            pilots,
            channel_est: vec![Complex64::new(1.0, 0.0); fft_size],
            has_estimate: false,
        }
    }

    /// Estimate channel from received OFDM symbol using pilots.
    pub fn estimate_channel(&mut self, rx_symbol: &[Complex64]) -> ChannelEstimate {
        assert!(rx_symbol.len() >= self.fft_size);

        // Estimate H at pilot positions: H_pilot = rx / pilot_value
        let mut pilot_h: Vec<(usize, Complex64)> = Vec::new();
        for (_, (&idx, &pval)) in self
            .pilots
            .pilot_indices
            .iter()
            .zip(self.pilots.pilot_values.iter())
            .enumerate()
        {
            if pval.norm() > 1e-10 {
                let h = rx_symbol[idx] / pval;
                pilot_h.push((idx, h));
            }
        }

        // Interpolate channel estimate to all subcarriers
        self.channel_est = interpolate_channel(&pilot_h, self.fft_size);
        self.has_estimate = true;

        // Compute average SNR
        let avg_h_mag_sq: f64 = self
            .channel_est
            .iter()
            .map(|h| h.norm_sqr())
            .sum::<f64>()
            / self.fft_size as f64;

        let avg_snr_db = if avg_h_mag_sq > 1e-12 {
            10.0 * avg_h_mag_sq.log10()
        } else {
            -100.0
        };

        ChannelEstimate {
            h: self.channel_est.clone(),
            avg_snr_db,
        }
    }

    /// Equalize one OFDM symbol using current channel estimate.
    pub fn equalize_symbol(&mut self, rx_symbol: &[Complex64]) -> Vec<Complex64> {
        if !self.has_estimate {
            self.estimate_channel(rx_symbol);
        }

        let mut equalized = vec![Complex64::new(0.0, 0.0); self.fft_size];

        for i in 0..self.fft_size {
            let h = self.channel_est[i];
            equalized[i] = match self.eq_type {
                EqualizerType::ZeroForcing => {
                    if h.norm_sqr() > 1e-12 {
                        rx_symbol[i] / h
                    } else {
                        Complex64::new(0.0, 0.0)
                    }
                }
                EqualizerType::Mmse { noise_var } => {
                    let h_conj = h.conj();
                    let denom = h.norm_sqr() + noise_var;
                    if denom > 1e-12 {
                        rx_symbol[i] * h_conj / denom
                    } else {
                        Complex64::new(0.0, 0.0)
                    }
                }
            };
        }

        equalized
    }

    /// Extract only data subcarriers from equalized symbol.
    pub fn extract_data(&self, equalized: &[Complex64]) -> Vec<Complex64> {
        self.pilots
            .data_indices
            .iter()
            .map(|&i| equalized[i])
            .collect()
    }

    /// Process a complete OFDM frame (multiple symbols).
    ///
    /// First symbol is used for channel estimation, rest are equalized.
    pub fn process_frame(&mut self, symbols: &[Vec<Complex64>]) -> Vec<Vec<Complex64>> {
        if symbols.is_empty() {
            return Vec::new();
        }

        // Estimate channel from first symbol (preamble/pilot symbol)
        self.estimate_channel(&symbols[0]);

        // Equalize all symbols
        symbols
            .iter()
            .map(|sym| self.equalize_symbol(sym))
            .collect()
    }

    /// Get current channel estimate.
    pub fn channel_estimate(&self) -> &[Complex64] {
        &self.channel_est
    }

    /// Reset channel estimate.
    pub fn reset(&mut self) {
        self.channel_est = vec![Complex64::new(1.0, 0.0); self.fft_size];
        self.has_estimate = false;
    }

    /// Get FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Get pilot pattern.
    pub fn pilot_pattern(&self) -> &PilotPattern {
        &self.pilots
    }
}

/// Interpolate channel estimates from pilot positions to all subcarriers.
fn interpolate_channel(
    pilot_h: &[(usize, Complex64)],
    fft_size: usize,
) -> Vec<Complex64> {
    if pilot_h.is_empty() {
        return vec![Complex64::new(1.0, 0.0); fft_size];
    }

    if pilot_h.len() == 1 {
        return vec![pilot_h[0].1; fft_size];
    }

    let mut result = vec![Complex64::new(0.0, 0.0); fft_size];

    for i in 0..fft_size {
        // Find surrounding pilot positions
        let mut left = 0;
        let mut right = pilot_h.len() - 1;

        for (j, &(idx, _)) in pilot_h.iter().enumerate() {
            if idx <= i {
                left = j;
            }
            if idx >= i && j < right {
                right = j;
                break;
            }
        }

        if left == right || pilot_h[left].0 == pilot_h[right].0 {
            result[i] = pilot_h[left].1;
        } else {
            // Linear interpolation
            let x0 = pilot_h[left].0 as f64;
            let x1 = pilot_h[right].0 as f64;
            let h0 = pilot_h[left].1;
            let h1 = pilot_h[right].1;
            let t = (i as f64 - x0) / (x1 - x0);
            result[i] = h0 * (1.0 - t) + h1 * t;
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pilot_pattern_uniform() {
        let pilots = PilotPattern::uniform(64, 8, Complex64::new(1.0, 0.0));
        assert_eq!(pilots.num_pilots(), 8); // 0, 8, 16, 24, 32, 40, 48, 56
        assert_eq!(pilots.num_data(), 56);
    }

    #[test]
    fn test_pilot_pattern_explicit() {
        let indices = vec![0, 16, 32, 48];
        let values = vec![Complex64::new(1.0, 0.0); 4];
        let pilots = PilotPattern::from_indices(64, indices, values);
        assert_eq!(pilots.num_pilots(), 4);
        assert_eq!(pilots.num_data(), 60);
    }

    #[test]
    fn test_zf_equalization() {
        let pilots = PilotPattern::uniform(8, 2, Complex64::new(1.0, 0.0));
        let mut eq = OfdmFrameEqualizer::new(8, EqualizerType::ZeroForcing, pilots);

        // Known channel: H = 2+j for all subcarriers
        let h = Complex64::new(2.0, 1.0);
        let tx: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 8];
        let rx: Vec<Complex64> = tx.iter().map(|&x| x * h).collect();

        let equalized = eq.equalize_symbol(&rx);
        // Should recover ~tx
        for &e in &equalized {
            assert!(
                (e.re - 1.0).abs() < 0.3,
                "ZF equalization failed: got {:.3}",
                e.re
            );
        }
    }

    #[test]
    fn test_mmse_equalization() {
        let pilots = PilotPattern::uniform(8, 2, Complex64::new(1.0, 0.0));
        let mut eq = OfdmFrameEqualizer::new(
            8,
            EqualizerType::Mmse { noise_var: 0.01 },
            pilots,
        );

        let h = Complex64::new(1.5, 0.5);
        let tx: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 8];
        let rx: Vec<Complex64> = tx.iter().map(|&x| x * h).collect();

        let equalized = eq.equalize_symbol(&rx);
        assert_eq!(equalized.len(), 8);
    }

    #[test]
    fn test_channel_estimation() {
        let pilots = PilotPattern::uniform(16, 4, Complex64::new(1.0, 0.0));
        let mut eq = OfdmFrameEqualizer::new(16, EqualizerType::ZeroForcing, pilots);

        // Channel: all ones (flat)
        let rx = vec![Complex64::new(1.0, 0.0); 16];
        let est = eq.estimate_channel(&rx);
        assert_eq!(est.h.len(), 16);
    }

    #[test]
    fn test_extract_data() {
        let pilots = PilotPattern::uniform(8, 4, Complex64::new(1.0, 0.0));
        let eq = OfdmFrameEqualizer::new(8, EqualizerType::ZeroForcing, pilots);

        let equalized: Vec<Complex64> = (0..8)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let data = eq.extract_data(&equalized);
        assert_eq!(data.len(), 6); // 8 - 2 pilots
    }

    #[test]
    fn test_process_frame() {
        let pilots = PilotPattern::uniform(16, 4, Complex64::new(1.0, 0.0));
        let mut eq = OfdmFrameEqualizer::new(16, EqualizerType::ZeroForcing, pilots);

        let symbols = vec![vec![Complex64::new(1.0, 0.0); 16]; 5];
        let equalized = eq.process_frame(&symbols);
        assert_eq!(equalized.len(), 5);
    }

    #[test]
    fn test_reset() {
        let pilots = PilotPattern::uniform(8, 2, Complex64::new(1.0, 0.0));
        let mut eq = OfdmFrameEqualizer::new(8, EqualizerType::ZeroForcing, pilots);

        let rx = vec![Complex64::new(2.0, 0.0); 8];
        eq.estimate_channel(&rx);
        eq.reset();

        // After reset, estimate should be all ones
        for &h in eq.channel_estimate() {
            assert!((h.re - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_identity_channel() {
        let pilots = PilotPattern::uniform(16, 4, Complex64::new(1.0, 0.0));
        let mut eq = OfdmFrameEqualizer::new(16, EqualizerType::ZeroForcing, pilots);

        // Identity channel: H = 1 for all
        let tx: Vec<Complex64> = (0..16)
            .map(|i| Complex64::new((i as f64 * 0.5).cos(), (i as f64 * 0.5).sin()))
            .collect();
        let equalized = eq.equalize_symbol(&tx);

        // Should recover tx approximately
        // With pilot-based estimation, most subcarriers should be close
        let errors: f64 = equalized
            .iter()
            .zip(tx.iter())
            .map(|(e, t)| (e - t).norm_sqr())
            .sum::<f64>()
            / equalized.len() as f64;
        assert!(
            errors < 5.0,
            "Identity channel avg error should be small: {}",
            errors
        );
    }

    #[test]
    fn test_equalizer_type() {
        let eq_type = EqualizerType::ZeroForcing;
        assert_eq!(eq_type, EqualizerType::ZeroForcing);
        let mmse = EqualizerType::Mmse { noise_var: 0.1 };
        assert_ne!(eq_type, mmse);
    }

    #[test]
    fn test_fft_size_accessor() {
        let pilots = PilotPattern::uniform(64, 8, Complex64::new(1.0, 0.0));
        let eq = OfdmFrameEqualizer::new(64, EqualizerType::ZeroForcing, pilots);
        assert_eq!(eq.fft_size(), 64);
    }
}
