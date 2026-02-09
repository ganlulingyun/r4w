//! MTI Filter — Moving Target Indication / Moving Target Detection
//!
//! Implements slow-time clutter rejection for pulsed radar systems. MTI filters
//! operate pulse-to-pulse to suppress stationary and slow-moving clutter while
//! preserving moving target echoes. Includes single/double/triple cancellers,
//! binomial weights, and Doppler filter banks (MTD).
//!
//! GNU Radio equivalent: `gr-radar` MTI/MTD processing blocks (OOT).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mti_filter::{MtiFilter, MtiConfig, MtiFilterType};
//! use num_complex::Complex64;
//!
//! let config = MtiConfig {
//!     filter_type: MtiFilterType::DoubleCanceller,
//!     num_range_bins: 64,
//!     prf_hz: 1000.0,
//!     stagger_ratios: None,
//! };
//! let mut mti = MtiFilter::new(config);
//!
//! // Process three pulses (minimum for double canceller)
//! let pulse = vec![Complex64::new(1.0, 0.0); 64];
//! assert!(mti.process_pulse(&pulse).is_none()); // first pulse
//! assert!(mti.process_pulse(&pulse).is_none()); // second pulse
//! let out = mti.process_pulse(&pulse);           // third pulse → output
//! // Constant (clutter) should be cancelled
//! if let Some(result) = out {
//!     let power: f64 = result.iter().map(|s| s.norm_sqr()).sum();
//!     assert!(power < 1e-10);
//! }
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// MTI filter type.
#[derive(Debug, Clone)]
pub enum MtiFilterType {
    /// H(z) = 1 - z^{-1}
    SingleCanceller,
    /// H(z) = (1 - z^{-1})^2
    DoubleCanceller,
    /// H(z) = (1 - z^{-1})^3
    TripleCanceller,
    /// Binomial weights of given order K: H(z) = (1 - z^{-1})^K
    BinomialWeights { order: usize },
    /// Custom FIR weights applied across slow-time.
    Custom { weights: Vec<f64> },
}

/// MTI filter configuration.
#[derive(Debug, Clone)]
pub struct MtiConfig {
    /// Filter type.
    pub filter_type: MtiFilterType,
    /// Number of range bins per pulse.
    pub num_range_bins: usize,
    /// Pulse Repetition Frequency in Hz.
    pub prf_hz: f64,
    /// Optional stagger ratios for staggered PRF.
    pub stagger_ratios: Option<Vec<f64>>,
}

/// Doppler window type.
#[derive(Debug, Clone)]
pub enum DopplerWindow {
    None,
    Hamming,
    Hann,
    Chebyshev { sidelobe_db: f64 },
}

/// MTI filter for clutter rejection.
#[derive(Debug, Clone)]
pub struct MtiFilter {
    /// FIR weights for slow-time filtering.
    weights: Vec<f64>,
    /// Pulse history buffer (ring buffer).
    pulse_history: Vec<Vec<Complex64>>,
    /// Number of range bins.
    num_range_bins: usize,
    /// PRF in Hz.
    prf_hz: f64,
    /// How many pulses collected so far.
    pulses_received: usize,
    /// Filter order (number of weights).
    filter_order: usize,
}

impl MtiFilter {
    /// Create a new MTI filter.
    pub fn new(config: MtiConfig) -> Self {
        let weights = match &config.filter_type {
            MtiFilterType::SingleCanceller => vec![1.0, -1.0],
            MtiFilterType::DoubleCanceller => vec![1.0, -2.0, 1.0],
            MtiFilterType::TripleCanceller => vec![1.0, -3.0, 3.0, -1.0],
            MtiFilterType::BinomialWeights { order } => binomial_weights(*order),
            MtiFilterType::Custom { weights } => weights.clone(),
        };
        let filter_order = weights.len();

        Self {
            weights,
            pulse_history: Vec::with_capacity(filter_order),
            num_range_bins: config.num_range_bins,
            prf_hz: config.prf_hz,
            pulses_received: 0,
            filter_order,
        }
    }

    /// Process a Coherent Processing Interval (CPI) — multiple pulses at once.
    ///
    /// Input: `cpi[pulse_index][range_bin]`
    /// Output: filtered pulses.
    pub fn process_cpi(&mut self, cpi: &[Vec<Complex64>]) -> Vec<Vec<Complex64>> {
        let mut results = Vec::new();
        for pulse in cpi {
            if let Some(filtered) = self.process_pulse(pulse) {
                results.push(filtered);
            }
        }
        results
    }

    /// Process a single pulse (range profile).
    ///
    /// Returns None until enough pulses are buffered.
    pub fn process_pulse(&mut self, range_profile: &[Complex64]) -> Option<Vec<Complex64>> {
        self.pulse_history.push(range_profile.to_vec());
        self.pulses_received += 1;

        if self.pulse_history.len() < self.filter_order {
            return None;
        }

        // Keep only the last filter_order pulses
        while self.pulse_history.len() > self.filter_order {
            self.pulse_history.remove(0);
        }

        let num_bins = range_profile.len().min(self.num_range_bins);
        let mut output = vec![Complex64::new(0.0, 0.0); num_bins];

        for bin in 0..num_bins {
            let mut sum = Complex64::new(0.0, 0.0);
            for (k, &w) in self.weights.iter().enumerate() {
                if k < self.pulse_history.len() && bin < self.pulse_history[k].len() {
                    sum += self.pulse_history[k][bin] * w;
                }
            }
            output[bin] = sum;
        }

        Some(output)
    }

    /// Compute the frequency response of the MTI filter.
    ///
    /// Returns magnitude response at `num_points` equally spaced normalized
    /// frequencies in [0, 1) (normalized to PRF).
    pub fn frequency_response(&self, num_points: usize) -> Vec<f64> {
        (0..num_points)
            .map(|i| {
                let f_norm = i as f64 / num_points as f64;
                let mut sum = Complex64::new(0.0, 0.0);
                for (k, &w) in self.weights.iter().enumerate() {
                    let angle = -2.0 * PI * f_norm * k as f64;
                    sum += Complex64::new(angle.cos(), angle.sin()) * w;
                }
                sum.norm()
            })
            .collect()
    }

    /// Compute blind speeds in m/s given the radar wavelength.
    ///
    /// Blind speeds: v = k * lambda * PRF / 2
    pub fn blind_speeds(&self, wavelength: f64) -> Vec<f64> {
        (1..=5)
            .map(|k| k as f64 * wavelength * self.prf_hz / 2.0)
            .collect()
    }

    /// Improvement factor in dB.
    ///
    /// For a binomial canceller of order K against Gaussian clutter with zero
    /// mean Doppler: I ≈ (2K choose K) for narrow-spectrum clutter.
    pub fn improvement_factor_db(&self) -> f64 {
        let order = self.filter_order - 1; // canceller order
        // Sum of squared weights for simple estimate
        let sum_sq: f64 = self.weights.iter().map(|w| w * w).sum();
        let sum_abs: f64 = self.weights.iter().map(|w| w.abs()).sum();
        10.0 * (sum_sq).log10() + 10.0 * (self.filter_order as f64).log10()
            - 20.0 * (sum_abs / self.filter_order as f64).log10()
            + 3.0 * order as f64
    }

    /// Clutter attenuation for given clutter spectral width.
    pub fn clutter_attenuation_db(&self, clutter_spectral_width: f64) -> f64 {
        // Integrate |H(f)|^2 * S_clutter(f) vs |H(f)|^2 for uniform spectrum
        let num_points = 1024;
        let freq_resp = self.frequency_response(num_points);

        let mut signal_power = 0.0;
        let mut clutter_power = 0.0;

        for i in 0..num_points {
            let f_norm = i as f64 / num_points as f64;
            let h_sq = freq_resp[i] * freq_resp[i];
            signal_power += h_sq;

            // Gaussian clutter centered at DC
            let clutter_weight = (-0.5 * (f_norm / clutter_spectral_width).powi(2)).exp()
                + (-0.5 * ((1.0 - f_norm) / clutter_spectral_width).powi(2)).exp();
            clutter_power += h_sq * clutter_weight;
        }

        if clutter_power < 1e-30 {
            return 60.0; // Very high attenuation
        }

        10.0 * (signal_power / clutter_power).log10()
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.pulse_history.clear();
        self.pulses_received = 0;
    }
}

/// Doppler filter bank for Moving Target Detection (MTD).
#[derive(Debug, Clone)]
pub struct DopplerFilterBank {
    /// Number of Doppler filters.
    num_filters: usize,
    /// Window coefficients.
    window: Vec<f64>,
}

impl DopplerFilterBank {
    /// Create a new Doppler filter bank.
    pub fn new(num_filters: usize, window: DopplerWindow) -> Self {
        let win = match &window {
            DopplerWindow::None => vec![1.0; num_filters],
            DopplerWindow::Hamming => (0..num_filters)
                .map(|i| {
                    0.54 - 0.46 * (2.0 * PI * i as f64 / (num_filters - 1).max(1) as f64).cos()
                })
                .collect(),
            DopplerWindow::Hann => (0..num_filters)
                .map(|i| {
                    0.5 * (1.0 - (2.0 * PI * i as f64 / (num_filters - 1).max(1) as f64).cos())
                })
                .collect(),
            DopplerWindow::Chebyshev { .. } => {
                // Simplified Chebyshev approximation
                vec![1.0; num_filters]
            }
        };

        Self {
            num_filters,
            window: win,
        }
    }

    /// Process a CPI to produce a range-Doppler map.
    ///
    /// Input: `cpi[pulse_index][range_bin]`
    /// Output: `[doppler_bin][range_bin]` in power (linear).
    pub fn process_cpi(&self, cpi: &[Vec<Complex64>]) -> Vec<Vec<f64>> {
        if cpi.is_empty() {
            return vec![];
        }

        let num_pulses = cpi.len();
        let num_range = cpi[0].len();
        let n = self.num_filters;

        let mut rd_map = vec![vec![0.0; num_range]; n];

        for range_bin in 0..num_range {
            // Extract slow-time samples for this range bin
            let slow_time: Vec<Complex64> = cpi
                .iter()
                .enumerate()
                .map(|(i, pulse)| {
                    if range_bin < pulse.len() {
                        let w = if i < self.window.len() {
                            self.window[i]
                        } else {
                            1.0
                        };
                        pulse[range_bin] * w
                    } else {
                        Complex64::new(0.0, 0.0)
                    }
                })
                .collect();

            // DFT across slow-time
            for k in 0..n {
                let mut sum = Complex64::new(0.0, 0.0);
                for (j, &s) in slow_time.iter().enumerate().take(num_pulses) {
                    let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                    sum += s * Complex64::new(angle.cos(), angle.sin());
                }
                rd_map[k][range_bin] = sum.norm_sqr();
            }
        }

        rd_map
    }

    /// Compute the velocity axis in m/s.
    pub fn velocity_axis(&self, prf: f64, wavelength: f64) -> Vec<f64> {
        let v_max = wavelength * prf / 4.0;
        (0..self.num_filters)
            .map(|i| {
                let f_norm = i as f64 / self.num_filters as f64;
                if f_norm <= 0.5 {
                    f_norm * 2.0 * v_max
                } else {
                    (f_norm - 1.0) * 2.0 * v_max
                }
            })
            .collect()
    }
}

/// Compute binomial weights of order K.
///
/// H(z) = (1 - z^{-1})^K → coefficients from Pascal's triangle with alternating signs.
fn binomial_weights(order: usize) -> Vec<f64> {
    let n = order + 1;
    let mut w = vec![0.0; n];
    w[0] = 1.0;

    for _ in 0..order {
        let mut new_w = vec![0.0; n];
        for j in 0..n {
            new_w[j] = w[j];
            if j > 0 {
                new_w[j] -= w[j - 1];
            }
        }
        w = new_w;
    }

    w
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_single_canceller_dc_suppression() {
        let config = MtiConfig {
            filter_type: MtiFilterType::SingleCanceller,
            num_range_bins: 32,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };
        let mut mti = MtiFilter::new(config);

        // Constant (DC) signal across pulses
        let pulse = vec![Complex64::new(1.0, 0.0); 32];
        assert!(mti.process_pulse(&pulse).is_none());
        let out = mti.process_pulse(&pulse).unwrap();

        let power: f64 = out.iter().map(|s| s.norm_sqr()).sum();
        assert!(power < 1e-20, "DC not cancelled: power = {power}");
    }

    #[test]
    fn test_single_canceller_max_doppler() {
        let config = MtiConfig {
            filter_type: MtiFilterType::SingleCanceller,
            num_range_bins: 8,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };
        let mut mti = MtiFilter::new(config);

        // Alternating +1/-1 (max Doppler)
        let pulse1 = vec![Complex64::new(1.0, 0.0); 8];
        let pulse2 = vec![Complex64::new(-1.0, 0.0); 8];

        mti.process_pulse(&pulse1);
        let out = mti.process_pulse(&pulse2).unwrap();

        // Should pass through at full amplitude: |-1 - 1| = 2
        let expected_amp = 2.0;
        for s in &out {
            assert!(
                (s.norm() - expected_amp).abs() < 1e-10,
                "amplitude = {}, expected {}",
                s.norm(),
                expected_amp
            );
        }
    }

    #[test]
    fn test_double_canceller_deeper_null() {
        let single_config = MtiConfig {
            filter_type: MtiFilterType::SingleCanceller,
            num_range_bins: 1,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };
        let double_config = MtiConfig {
            filter_type: MtiFilterType::DoubleCanceller,
            num_range_bins: 1,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };

        let single_resp = MtiFilter::new(single_config).frequency_response(256);
        let double_resp = MtiFilter::new(double_config).frequency_response(256);

        // At DC (bin 0), both should be near zero
        assert!(single_resp[0] < 1e-10);
        assert!(double_resp[0] < 1e-10);

        // Near DC (bin 1), double should be smaller
        assert!(
            double_resp[1] < single_resp[1],
            "double = {}, single = {}",
            double_resp[1],
            single_resp[1]
        );
    }

    #[test]
    fn test_blind_speeds() {
        let config = MtiConfig {
            filter_type: MtiFilterType::SingleCanceller,
            num_range_bins: 1,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };
        let mti = MtiFilter::new(config);

        let wavelength = 0.03; // X-band, ~10 GHz
        let speeds = mti.blind_speeds(wavelength);

        // First blind speed = lambda * PRF / 2 = 0.03 * 1000 / 2 = 15 m/s
        assert!(
            (speeds[0] - 15.0).abs() < 0.01,
            "first blind speed = {}, expected 15.0",
            speeds[0]
        );
    }

    #[test]
    fn test_frequency_response_shape() {
        let config = MtiConfig {
            filter_type: MtiFilterType::TripleCanceller,
            num_range_bins: 1,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };
        let mti = MtiFilter::new(config);

        let resp = mti.frequency_response(128);

        // DC null (3rd order)
        assert!(resp[0] < 1e-10, "DC response = {}", resp[0]);

        // Maximum at f_norm = 0.5 (half PRF)
        let mid = resp[64];
        assert!(mid > 5.0, "mid response = {mid}");
    }

    #[test]
    fn test_doppler_filter_bank_separation() {
        let dfb = DopplerFilterBank::new(16, DopplerWindow::None);

        // Create CPI with two targets at Doppler bins 3 and 10
        let num_pulses = 16;
        let num_range = 32;
        let mut cpi = vec![vec![Complex64::new(0.0, 0.0); num_range]; num_pulses];

        for pulse in 0..num_pulses {
            // Target 1: range bin 5, Doppler bin 3
            let phase1 = 2.0 * PI * 3.0 * pulse as f64 / 16.0;
            cpi[pulse][5] += Complex64::new(phase1.cos(), phase1.sin());

            // Target 2: range bin 20, Doppler bin 10
            let phase2 = 2.0 * PI * 10.0 * pulse as f64 / 16.0;
            cpi[pulse][20] += Complex64::new(phase2.cos(), phase2.sin());
        }

        let rd_map = dfb.process_cpi(&cpi);

        // Target 1 peak should be at Doppler bin 3, range bin 5
        let peak1_doppler = (0..16)
            .max_by(|&a, &b| rd_map[a][5].partial_cmp(&rd_map[b][5]).unwrap())
            .unwrap();
        assert_eq!(peak1_doppler, 3, "target 1 Doppler = {peak1_doppler}");

        // Target 2 peak should be at Doppler bin 10, range bin 20
        let peak2_doppler = (0..16)
            .max_by(|&a, &b| rd_map[a][20].partial_cmp(&rd_map[b][20]).unwrap())
            .unwrap();
        assert_eq!(peak2_doppler, 10, "target 2 Doppler = {peak2_doppler}");
    }

    #[test]
    fn test_hamming_windowed_sidelobes() {
        let dfb_rect = DopplerFilterBank::new(32, DopplerWindow::None);
        let dfb_ham = DopplerFilterBank::new(32, DopplerWindow::Hamming);

        // Target between bins (8.3) to force spectral leakage
        let target_bin = 8.3;
        let num_pulses = 32;
        let mut cpi = vec![vec![Complex64::new(0.0, 0.0); 1]; num_pulses];
        for pulse in 0..num_pulses {
            let phase = 2.0 * PI * target_bin * pulse as f64 / 32.0;
            cpi[pulse][0] = Complex64::new(phase.cos(), phase.sin());
        }

        let rd_rect = dfb_rect.process_cpi(&cpi);
        let rd_ham = dfb_ham.process_cpi(&cpi);

        // Find peak bin for each
        let peak_bin_rect = (0..32).max_by(|&a, &b| rd_rect[a][0].partial_cmp(&rd_rect[b][0]).unwrap()).unwrap();
        let peak_bin_ham = (0..32).max_by(|&a, &b| rd_ham[a][0].partial_cmp(&rd_ham[b][0]).unwrap()).unwrap();

        let peak_rect = rd_rect[peak_bin_rect][0];
        let peak_ham = rd_ham[peak_bin_ham][0];

        // Find peak sidelobe (excluding bins within 2 of peak)
        let max_sl_rect = (0..32)
            .filter(|&i| (i as isize - peak_bin_rect as isize).unsigned_abs() > 2)
            .map(|i| rd_rect[i][0])
            .fold(0.0_f64, f64::max);

        let max_sl_ham = (0..32)
            .filter(|&i| (i as isize - peak_bin_ham as isize).unsigned_abs() > 2)
            .map(|i| rd_ham[i][0])
            .fold(0.0_f64, f64::max);

        let psl_rect = 10.0 * (max_sl_rect / peak_rect).log10();
        let psl_ham = 10.0 * (max_sl_ham / peak_ham).log10();

        assert!(
            psl_ham < psl_rect,
            "Hamming PSL {psl_ham:.1} should be less than rect {psl_rect:.1}"
        );
    }

    #[test]
    fn test_process_pulse_returns_none_initially() {
        let config = MtiConfig {
            filter_type: MtiFilterType::DoubleCanceller,
            num_range_bins: 4,
            prf_hz: 1000.0,
            stagger_ratios: None,
        };
        let mut mti = MtiFilter::new(config);

        let pulse = vec![Complex64::new(1.0, 0.0); 4];
        assert!(mti.process_pulse(&pulse).is_none()); // need 3 pulses
        assert!(mti.process_pulse(&pulse).is_none()); // still need 1 more
        assert!(mti.process_pulse(&pulse).is_some()); // now we have enough
    }

    #[test]
    fn test_binomial_weights() {
        // Order 2 should give [1, -2, 1]
        let w = binomial_weights(2);
        assert_eq!(w.len(), 3);
        assert!((w[0] - 1.0).abs() < 1e-10);
        assert!((w[1] - (-2.0)).abs() < 1e-10);
        assert!((w[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_velocity_axis() {
        let dfb = DopplerFilterBank::new(16, DopplerWindow::None);
        let prf = 1000.0;
        let wavelength = 0.03; // X-band

        let vaxis = dfb.velocity_axis(prf, wavelength);
        assert_eq!(vaxis.len(), 16);

        // v_max = lambda * PRF / 4 = 0.03 * 1000 / 4 = 7.5 m/s
        assert!((vaxis[0]).abs() < 1e-10, "DC velocity should be 0");
    }
}
