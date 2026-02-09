//! Cross-Ambiguity Function — Passive Bistatic Radar Processing
//!
//! Computes the cross-ambiguity function (CAF) between a reference signal and
//! a surveillance signal for passive coherent location (PCL) applications.
//! Detects targets illuminated by transmitters of opportunity (FM radio, DVB-T,
//! DAB, LTE, Wi-Fi). Includes direct-path interference (DPI) cancellation.
//!
//! GNU Radio equivalent: `gr-passive_radar` / `gr-plasma` cross-ambiguity
//! surface (OOT modules).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cross_ambiguity_function::{
//!     CrossAmbiguityFunction, CafConfig, CafAlgorithm,
//! };
//! use num_complex::Complex64;
//!
//! let config = CafConfig {
//!     max_delay_samples: 32,
//!     num_doppler_bins: 16,
//!     doppler_max_hz: 500.0,
//!     sample_rate: 10000.0,
//!     algorithm: CafAlgorithm::DirectCorrelation,
//! };
//! let mut caf = CrossAmbiguityFunction::new(config);
//! let reference = vec![Complex64::new(1.0, 0.0); 128];
//! let surveillance = vec![Complex64::new(0.5, 0.0); 128];
//! let surface = caf.compute(&reference, &surveillance);
//! assert_eq!(surface.data.len(), 16);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// CAF processing algorithm.
#[derive(Debug, Clone)]
pub enum CafAlgorithm {
    /// Direct correlation (brute force).
    DirectCorrelation,
    /// FFT-based processing.
    FftBased,
    /// Batched FFT with incoherent averaging.
    BatchedFft { cpi_samples: usize },
}

/// DPI cancellation algorithm.
#[derive(Debug, Clone)]
pub enum DpiAlgorithm {
    /// Least Mean Squares.
    Lms { step_size: f64 },
    /// Normalized LMS.
    Nlms { step_size: f64 },
    /// Extensible Cancellation Algorithm - Batched.
    ExtensibleCancellation { batches: usize, order: usize },
}

/// CAF configuration.
#[derive(Debug, Clone)]
pub struct CafConfig {
    /// Maximum delay in samples to search.
    pub max_delay_samples: usize,
    /// Number of Doppler frequency bins.
    pub num_doppler_bins: usize,
    /// Maximum Doppler shift in Hz.
    pub doppler_max_hz: f64,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Processing algorithm.
    pub algorithm: CafAlgorithm,
}

/// CAF surface result.
#[derive(Debug, Clone)]
pub struct CafSurface {
    /// 2D surface data: `data[doppler_bin][delay_bin]` in linear power.
    pub data: Vec<Vec<f64>>,
    /// Delay axis in seconds.
    pub delay_axis_s: Vec<f64>,
    /// Doppler axis in Hz.
    pub doppler_axis_hz: Vec<f64>,
}

/// Detected target from CAF.
#[derive(Debug, Clone)]
pub struct CafTarget {
    /// Bistatic delay in seconds.
    pub delay_s: f64,
    /// Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Target power in dB.
    pub power_db: f64,
    /// Target SNR in dB.
    pub snr_db: f64,
}

impl CafSurface {
    /// Find targets above a threshold.
    pub fn find_targets(&self, threshold_db: f64) -> Vec<CafTarget> {
        let noise_floor = self.estimate_noise_floor();
        let threshold_linear = noise_floor * 10.0_f64.powf(threshold_db / 10.0);

        let mut targets = Vec::new();
        for (d_idx, doppler_row) in self.data.iter().enumerate() {
            for (t_idx, &power) in doppler_row.iter().enumerate() {
                if power > threshold_linear {
                    // Check if local maximum
                    let is_peak = self.is_local_peak(d_idx, t_idx);
                    if is_peak {
                        let power_db = 10.0 * power.log10();
                        let snr_db = 10.0 * (power / noise_floor.max(1e-30)).log10();
                        targets.push(CafTarget {
                            delay_s: self.delay_axis_s[t_idx],
                            doppler_hz: self.doppler_axis_hz[d_idx],
                            power_db,
                            snr_db,
                        });
                    }
                }
            }
        }
        targets
    }

    /// Convert surface to dB scale.
    pub fn to_db(&self) -> Vec<Vec<f64>> {
        self.data
            .iter()
            .map(|row| row.iter().map(|&v| 10.0 * v.max(1e-30).log10()).collect())
            .collect()
    }

    /// Peak-to-noise ratio in dB.
    pub fn peak_to_noise_db(&self) -> f64 {
        let peak = self
            .data
            .iter()
            .flat_map(|row| row.iter())
            .cloned()
            .fold(0.0_f64, f64::max);

        let noise_floor = self.estimate_noise_floor();
        if noise_floor < 1e-30 {
            return 0.0;
        }

        10.0 * (peak / noise_floor).log10()
    }

    /// Convert delay index to bistatic range in meters.
    pub fn delay_to_bistatic_range(&self, delay_idx: usize) -> f64 {
        if delay_idx < self.delay_axis_s.len() {
            self.delay_axis_s[delay_idx] * 299_792_458.0
        } else {
            0.0
        }
    }

    fn estimate_noise_floor(&self) -> f64 {
        // Use median of all values as noise floor estimate
        let mut all_values: Vec<f64> = self
            .data
            .iter()
            .flat_map(|row| row.iter())
            .cloned()
            .filter(|&v| v > 0.0)
            .collect();

        if all_values.is_empty() {
            return 1e-30;
        }

        all_values.sort_by(|a, b| a.partial_cmp(b).unwrap());
        all_values[all_values.len() / 2]
    }

    fn is_local_peak(&self, d_idx: usize, t_idx: usize) -> bool {
        let val = self.data[d_idx][t_idx];
        let nd = self.data.len();
        let nt = if nd > 0 { self.data[0].len() } else { 0 };

        for dd in [-1i32, 0, 1] {
            for dt in [-1i32, 0, 1] {
                if dd == 0 && dt == 0 {
                    continue;
                }
                let ni = d_idx as i32 + dd;
                let nj = t_idx as i32 + dt;
                if ni >= 0 && (ni as usize) < nd && nj >= 0 && (nj as usize) < nt {
                    if self.data[ni as usize][nj as usize] > val {
                        return false;
                    }
                }
            }
        }
        true
    }
}

/// Cross-ambiguity function processor.
#[derive(Debug, Clone)]
pub struct CrossAmbiguityFunction {
    config: CafConfig,
}

impl CrossAmbiguityFunction {
    /// Create a new CAF processor.
    pub fn new(config: CafConfig) -> Self {
        Self { config }
    }

    /// Compute the cross-ambiguity function.
    pub fn compute(&mut self, reference: &[Complex64], surveillance: &[Complex64]) -> CafSurface {
        match &self.config.algorithm {
            CafAlgorithm::DirectCorrelation => self.compute_direct(reference, surveillance),
            CafAlgorithm::FftBased => self.compute_direct(reference, surveillance), // Fallback
            CafAlgorithm::BatchedFft { cpi_samples } => {
                self.compute_batched(reference, surveillance, *cpi_samples)
            }
        }
    }

    /// Compute CAF with direct-path interference cancellation.
    pub fn compute_with_cancellation(
        &mut self,
        reference: &[Complex64],
        surveillance: &[Complex64],
        canceller: &mut DirectPathCanceller,
    ) -> CafSurface {
        let cleaned = canceller.cancel(reference, surveillance);
        self.compute(reference, &cleaned)
    }

    fn compute_direct(&self, reference: &[Complex64], surveillance: &[Complex64]) -> CafSurface {
        let n = reference.len().min(surveillance.len());
        let max_delay = self.config.max_delay_samples.min(n);
        let num_doppler = self.config.num_doppler_bins;

        let doppler_axis: Vec<f64> = (0..num_doppler)
            .map(|i| {
                let frac = if num_doppler > 1 {
                    i as f64 / (num_doppler - 1) as f64
                } else {
                    0.0
                };
                -self.config.doppler_max_hz + 2.0 * self.config.doppler_max_hz * frac
            })
            .collect();

        let delay_axis: Vec<f64> = (0..max_delay)
            .map(|tau| tau as f64 / self.config.sample_rate)
            .collect();

        let mut data = vec![vec![0.0; max_delay]; num_doppler];

        for (d_idx, &fd) in doppler_axis.iter().enumerate() {
            for tau in 0..max_delay {
                let mut sum = Complex64::new(0.0, 0.0);
                for i in 0..(n - tau) {
                    let doppler_phase = -2.0 * PI * fd * i as f64 / self.config.sample_rate;
                    let steering = Complex64::new(doppler_phase.cos(), doppler_phase.sin());
                    sum += surveillance[i] * reference[i + tau].conj() * steering;
                }
                data[d_idx][tau] = sum.norm_sqr();
            }
        }

        CafSurface {
            data,
            delay_axis_s: delay_axis,
            doppler_axis_hz: doppler_axis,
        }
    }

    fn compute_batched(
        &self,
        reference: &[Complex64],
        surveillance: &[Complex64],
        batch_size: usize,
    ) -> CafSurface {
        let n = reference.len().min(surveillance.len());
        let max_delay = self.config.max_delay_samples.min(n);
        let num_doppler = self.config.num_doppler_bins;

        let doppler_axis: Vec<f64> = (0..num_doppler)
            .map(|i| {
                let frac = if num_doppler > 1 {
                    i as f64 / (num_doppler - 1) as f64
                } else {
                    0.0
                };
                -self.config.doppler_max_hz + 2.0 * self.config.doppler_max_hz * frac
            })
            .collect();

        let delay_axis: Vec<f64> = (0..max_delay)
            .map(|tau| tau as f64 / self.config.sample_rate)
            .collect();

        let mut data = vec![vec![0.0; max_delay]; num_doppler];
        let num_batches = ((n - max_delay) / batch_size).max(1);

        for batch in 0..num_batches {
            let start = batch * batch_size;
            let end = (start + batch_size).min(n - max_delay);

            for (d_idx, &fd) in doppler_axis.iter().enumerate() {
                for tau in 0..max_delay {
                    let mut sum = Complex64::new(0.0, 0.0);
                    for i in start..end {
                        if i + tau < n {
                            let doppler_phase =
                                -2.0 * PI * fd * i as f64 / self.config.sample_rate;
                            let steering =
                                Complex64::new(doppler_phase.cos(), doppler_phase.sin());
                            sum += surveillance[i] * reference[i + tau].conj() * steering;
                        }
                    }
                    data[d_idx][tau] += sum.norm_sqr(); // Incoherent average
                }
            }
        }

        CafSurface {
            data,
            delay_axis_s: delay_axis,
            doppler_axis_hz: doppler_axis,
        }
    }
}

/// Direct-path interference canceller.
#[derive(Debug, Clone)]
pub struct DirectPathCanceller {
    /// Adaptive filter taps.
    taps: Vec<Complex64>,
    /// Number of taps.
    num_taps: usize,
    /// Algorithm.
    algorithm: DpiAlgorithm,
    /// Cancellation ratio achieved.
    cancellation_ratio: f64,
}

impl DirectPathCanceller {
    /// Create a new DPI canceller.
    pub fn new(num_taps: usize, algorithm: DpiAlgorithm) -> Self {
        Self {
            taps: vec![Complex64::new(0.0, 0.0); num_taps],
            num_taps,
            algorithm,
            cancellation_ratio: 0.0,
        }
    }

    /// Cancel direct-path interference.
    ///
    /// Returns cleaned surveillance signal.
    pub fn cancel(&mut self, reference: &[Complex64], surveillance: &[Complex64]) -> Vec<Complex64> {
        let n = surveillance.len().min(reference.len());

        match &self.algorithm {
            DpiAlgorithm::Lms { step_size } => {
                self.cancel_lms(reference, surveillance, n, *step_size)
            }
            DpiAlgorithm::Nlms { step_size } => {
                self.cancel_nlms(reference, surveillance, n, *step_size)
            }
            DpiAlgorithm::ExtensibleCancellation { batches, order } => {
                self.cancel_eca(reference, surveillance, n, *batches, *order)
            }
        }
    }

    /// Cancellation ratio in dB.
    pub fn cancellation_ratio_db(&self) -> f64 {
        self.cancellation_ratio
    }

    fn cancel_lms(
        &mut self,
        reference: &[Complex64],
        surveillance: &[Complex64],
        n: usize,
        step_size: f64,
    ) -> Vec<Complex64> {
        let mut output = vec![Complex64::new(0.0, 0.0); n];
        let mut input_power_before = 0.0;
        let mut output_power = 0.0;

        for i in 0..n {
            input_power_before += surveillance[i].norm_sqr();

            // Filter output
            let mut y = Complex64::new(0.0, 0.0);
            for k in 0..self.num_taps {
                if i >= k {
                    y += self.taps[k] * reference[i - k];
                }
            }

            let error = surveillance[i] - y;
            output[i] = error;
            output_power += error.norm_sqr();

            // Update taps
            for k in 0..self.num_taps {
                if i >= k {
                    self.taps[k] += Complex64::new(step_size, 0.0) * error * reference[i - k].conj();
                }
            }
        }

        self.cancellation_ratio = if output_power > 0.0 {
            10.0 * (input_power_before / output_power).log10()
        } else {
            60.0
        };

        output
    }

    fn cancel_nlms(
        &mut self,
        reference: &[Complex64],
        surveillance: &[Complex64],
        n: usize,
        step_size: f64,
    ) -> Vec<Complex64> {
        let mut output = vec![Complex64::new(0.0, 0.0); n];
        let mut input_power_before = 0.0;
        let mut output_power = 0.0;

        for i in 0..n {
            input_power_before += surveillance[i].norm_sqr();

            let mut y = Complex64::new(0.0, 0.0);
            let mut ref_power = 0.0;
            for k in 0..self.num_taps {
                if i >= k {
                    y += self.taps[k] * reference[i - k];
                    ref_power += reference[i - k].norm_sqr();
                }
            }

            let error = surveillance[i] - y;
            output[i] = error;
            output_power += error.norm_sqr();

            let mu = if ref_power > 1e-30 {
                step_size / ref_power
            } else {
                0.0
            };

            for k in 0..self.num_taps {
                if i >= k {
                    self.taps[k] += Complex64::new(mu, 0.0) * error * reference[i - k].conj();
                }
            }
        }

        self.cancellation_ratio = if output_power > 0.0 {
            10.0 * (input_power_before / output_power).log10()
        } else {
            60.0
        };

        output
    }

    fn cancel_eca(
        &mut self,
        reference: &[Complex64],
        surveillance: &[Complex64],
        n: usize,
        _batches: usize,
        order: usize,
    ) -> Vec<Complex64> {
        // Build reference matrix and solve least-squares
        let num_taps = order.min(self.num_taps);

        // Compute reference autocorrelation and cross-correlation
        let mut r_xx = vec![vec![Complex64::new(0.0, 0.0); num_taps]; num_taps];
        let mut r_xs = vec![Complex64::new(0.0, 0.0); num_taps];

        for i in num_taps..n {
            for j in 0..num_taps {
                r_xs[j] += reference[i - j].conj() * surveillance[i];
                for k in 0..num_taps {
                    r_xx[j][k] += reference[i - j].conj() * reference[i - k];
                }
            }
        }

        // Simple diagonal loading and solve
        let eps = 1e-6;
        for j in 0..num_taps {
            r_xx[j][j] += Complex64::new(eps, 0.0);
        }

        // Gauss-Seidel iterative solve
        let mut weights = vec![Complex64::new(0.0, 0.0); num_taps];
        for _ in 0..50 {
            for j in 0..num_taps {
                let mut sum = r_xs[j];
                for k in 0..num_taps {
                    if k != j {
                        sum -= r_xx[j][k] * weights[k];
                    }
                }
                if r_xx[j][j].norm() > 1e-30 {
                    weights[j] = sum / r_xx[j][j];
                }
            }
        }

        // Apply cancellation
        let mut output = surveillance[..n].to_vec();
        let mut input_power = 0.0;
        let mut output_power = 0.0;

        for i in 0..n {
            input_power += surveillance[i].norm_sqr();
            let mut cancel = Complex64::new(0.0, 0.0);
            for k in 0..num_taps {
                if i >= k {
                    cancel += weights[k] * reference[i - k];
                }
            }
            output[i] = surveillance[i] - cancel;
            output_power += output[i].norm_sqr();
        }

        self.cancellation_ratio = if output_power > 0.0 {
            10.0 * (input_power / output_power).log10()
        } else {
            60.0
        };

        for (k, &w) in weights.iter().enumerate() {
            if k < self.taps.len() {
                self.taps[k] = w;
            }
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a wideband pseudorandom reference signal (good range resolution).
    fn generate_wideband_signal(n: usize, seed: u64) -> Vec<Complex64> {
        let mut state = seed;
        (0..n)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                let phase = (state as f64 / u64::MAX as f64) * 2.0 * PI;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_synthetic_target_detection() {
        let fs = 10000.0;
        let n = 256;
        let target_delay = 20;

        // Wideband reference signal (pseudorandom for range resolution)
        let reference = generate_wideband_signal(n, 12345);

        // Surveillance = delayed reference (zero Doppler for simplicity)
        let mut surveillance = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..(n - target_delay) {
            surveillance[i] = reference[i + target_delay] * 0.5;
        }

        let config = CafConfig {
            max_delay_samples: 40,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let mut caf = CrossAmbiguityFunction::new(config);
        let surface = caf.compute(&reference, &surveillance);

        // Find peak delay
        let row = &surface.data[0];
        let max_t = row
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        assert!(
            (max_t as isize - target_delay as isize).unsigned_abs() < 3,
            "delay peak at {max_t}, expected ~{target_delay}"
        );
    }

    #[test]
    fn test_dpi_cancellation() {
        let fs = 10000.0;
        let n = 512;

        let reference = generate_wideband_signal(n, 66666);

        // Surveillance = strong direct path (no delay, no Doppler)
        let surveillance: Vec<Complex64> = reference.iter().map(|&s| s * 10.0).collect();

        let mut canceller = DirectPathCanceller::new(16, DpiAlgorithm::Nlms { step_size: 0.5 });
        let cleaned = canceller.cancel(&reference, &surveillance);

        // Direct path should be significantly reduced
        let ratio = canceller.cancellation_ratio_db();
        assert!(
            ratio > 3.0,
            "cancellation ratio = {ratio:.1} dB, expected > 3"
        );

        // Cleaned signal should have lower power than surveillance
        let surv_power: f64 = surveillance.iter().map(|s| s.norm_sqr()).sum();
        let clean_power: f64 = cleaned.iter().map(|s| s.norm_sqr()).sum();
        assert!(clean_power < surv_power, "cleaned should have less power");
    }

    #[test]
    fn test_two_targets_resolved() {
        let fs = 10000.0;
        let n = 256;

        let reference = generate_wideband_signal(n, 54321);

        let mut surveillance = vec![Complex64::new(0.0, 0.0); n];
        // Target 1: delay 10
        for i in 0..(n - 10) {
            surveillance[i] += reference[i + 10] * 0.1;
        }
        // Target 2: delay 30
        for i in 0..(n - 30) {
            surveillance[i] += reference[i + 30] * 0.05;
        }

        let config = CafConfig {
            max_delay_samples: 50,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let mut caf = CrossAmbiguityFunction::new(config);
        let surface = caf.compute(&reference, &surveillance);

        let row = &surface.data[0];
        // Should have peaks near delays 10 and 30
        let peak1_region: f64 = row[8..13].iter().cloned().fold(0.0_f64, f64::max);
        let peak2_region: f64 = row[28..33].iter().cloned().fold(0.0_f64, f64::max);
        let noise_region: f64 = row[18..24].iter().cloned().fold(0.0_f64, f64::max);

        assert!(peak1_region > noise_region * 1.5, "target 1 not detected");
        assert!(peak2_region > noise_region * 1.5, "target 2 not detected");
    }

    #[test]
    fn test_doppler_resolution() {
        let fs = 10000.0;
        let n = 256;

        let reference = generate_wideband_signal(n, 77777);

        // Target at Doppler 100 Hz, delay 5
        let mut surveillance = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..(n - 5) {
            let dp = 2.0 * PI * 100.0 * i as f64 / fs;
            surveillance[i] = reference[i + 5] * Complex64::new(dp.cos(), dp.sin()) * 0.1;
        }

        let config = CafConfig {
            max_delay_samples: 15,
            num_doppler_bins: 21,
            doppler_max_hz: 200.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let mut caf = CrossAmbiguityFunction::new(config);
        let surface = caf.compute(&reference, &surveillance);

        // Find peak Doppler
        let mut max_val = 0.0;
        let mut max_d = 0;
        for (d, row) in surface.data.iter().enumerate() {
            for &v in row.iter() {
                if v > max_val {
                    max_val = v;
                    max_d = d;
                }
            }
        }

        let detected = surface.doppler_axis_hz[max_d];
        assert!(
            (detected - 100.0).abs() < 25.0,
            "detected Doppler = {detected}, expected ~100"
        );
    }

    #[test]
    fn test_coherent_integration_gain() {
        let fs = 10000.0;

        // Test with two different lengths using wideband reference
        let n_short = 64;
        let n_long = 256;

        let ref_short = generate_wideband_signal(n_short, 11111);
        let ref_long = generate_wideband_signal(n_long, 11111); // same seed, longer

        let mut surv_short = vec![Complex64::new(0.0, 0.0); n_short];
        let mut surv_long = vec![Complex64::new(0.0, 0.0); n_long];

        for i in 0..(n_short - 5) {
            surv_short[i] = ref_short[i + 5] * 0.1;
        }
        for i in 0..(n_long - 5) {
            surv_long[i] = ref_long[i + 5] * 0.1;
        }

        let config_short = CafConfig {
            max_delay_samples: 10,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };
        let config_long = CafConfig {
            max_delay_samples: 10,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let mut caf_short = CrossAmbiguityFunction::new(config_short);
        let mut caf_long = CrossAmbiguityFunction::new(config_long);

        let surf_short = caf_short.compute(&ref_short, &surv_short);
        let surf_long = caf_long.compute(&ref_long, &surv_long);

        let peak_short = surf_short.data[0].iter().cloned().fold(0.0_f64, f64::max);
        let peak_long = surf_long.data[0].iter().cloned().fold(0.0_f64, f64::max);

        // Longer integration should give higher peak
        assert!(
            peak_long > peak_short * 2.0,
            "long peak {peak_long:.1} not much larger than short {peak_short:.1}"
        );
    }

    #[test]
    fn test_noise_only_no_false_peaks() {
        let fs = 10000.0;
        let n = 128;

        let reference = generate_wideband_signal(n, 99999);

        // Independent noise surveillance (different seed)
        let surveillance = generate_wideband_signal(n, 88888);

        let config = CafConfig {
            max_delay_samples: 32,
            num_doppler_bins: 11,
            doppler_max_hz: 100.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let mut caf = CrossAmbiguityFunction::new(config);
        let surface = caf.compute(&reference, &surveillance);

        // With independent noise, the CAF surface should be relatively flat
        let pnr = surface.peak_to_noise_db();
        // Should not have very high peaks
        assert!(
            pnr < 15.0,
            "noise-only PNR = {pnr:.1} dB, expected < 15"
        );
    }

    #[test]
    fn test_delay_to_bistatic_range() {
        let config = CafConfig {
            max_delay_samples: 100,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: 1e6,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let caf = CrossAmbiguityFunction::new(config);
        let reference = vec![Complex64::new(1.0, 0.0); 200];
        let surveillance = vec![Complex64::new(0.0, 0.0); 200];
        let mut caf_mut = caf;
        let surface = caf_mut.compute(&reference, &surveillance);

        // Delay of 10 samples at 1 MHz = 10 us = 3 km
        let range = surface.delay_to_bistatic_range(10);
        let expected = 10.0 / 1e6 * 299_792_458.0; // ~2998 m
        assert!(
            (range - expected).abs() < 1.0,
            "range = {range:.0} m, expected {expected:.0}"
        );
    }

    #[test]
    fn test_batched_processing() {
        let fs = 10000.0;
        let n = 256;

        let reference = generate_wideband_signal(n, 55555);

        let mut surveillance = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..(n - 15) {
            surveillance[i] = reference[i + 15] * 0.1;
        }

        let config = CafConfig {
            max_delay_samples: 30,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::BatchedFft { cpi_samples: 64 },
        };

        let mut caf = CrossAmbiguityFunction::new(config);
        let surface = caf.compute(&reference, &surveillance);

        // Should still detect target near delay 15
        let row = &surface.data[0];
        let peak_idx = row
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        assert!(
            (peak_idx as isize - 15).unsigned_abs() < 5,
            "batched peak at {peak_idx}, expected ~15"
        );
    }

    #[test]
    fn test_peak_to_noise_db() {
        let fs = 10000.0;
        let n = 256;

        let reference = generate_wideband_signal(n, 33333);

        // Target at delay 10 plus some noise
        let noise = generate_wideband_signal(n, 44444);
        let mut surveillance = vec![Complex64::new(0.0, 0.0); n];
        for i in 0..n {
            surveillance[i] = noise[i] * 0.01; // weak noise floor
        }
        for i in 0..(n - 10) {
            surveillance[i] += reference[i + 10] * 0.5; // strong target
        }

        let config = CafConfig {
            max_delay_samples: 30,
            num_doppler_bins: 1,
            doppler_max_hz: 0.0,
            sample_rate: fs,
            algorithm: CafAlgorithm::DirectCorrelation,
        };

        let mut caf = CrossAmbiguityFunction::new(config);
        let surface = caf.compute(&reference, &surveillance);

        let pnr = surface.peak_to_noise_db();
        assert!(
            pnr > 3.0,
            "peak-to-noise = {pnr:.1} dB, expected > 3"
        );
    }

    #[test]
    fn test_eca_cancellation() {
        let fs = 10000.0;
        let n = 512;

        let reference = generate_wideband_signal(n, 77777);
        let surveillance: Vec<Complex64> = reference.iter().map(|&s| s * 5.0).collect();

        let mut canceller = DirectPathCanceller::new(
            32,
            DpiAlgorithm::ExtensibleCancellation {
                batches: 4,
                order: 32,
            },
        );
        let _cleaned = canceller.cancel(&reference, &surveillance);

        let ratio = canceller.cancellation_ratio_db();
        assert!(
            ratio > 3.0,
            "ECA cancellation ratio = {ratio:.1} dB, expected > 3"
        );
    }
}
