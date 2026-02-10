//! Pulse-Doppler processing for pulsed radar systems with MTI and clutter rejection.
//!
//! This module implements range-Doppler processing by applying FFTs across the
//! slow-time (pulse) dimension of a radar data cube, producing a 2D range-Doppler
//! map. It includes Moving Target Indicator (MTI) filters for clutter suppression
//! and 2D CFAR detection for automatic target extraction.
//!
//! # Example
//!
//! ```
//! use r4w_core::pulse_doppler_processor::{PdConfig, PulseDopplerProcessor, MtiFilter};
//!
//! let config = PdConfig {
//!     num_range_bins: 64,
//!     num_pulses: 16,
//!     prf_hz: 1000.0,
//!     sample_rate: 1e6,
//!     wavelength_m: 0.03,
//! };
//!
//! let processor = PulseDopplerProcessor::new(config);
//!
//! // Create a data cube: num_pulses rows, each with num_range_bins IQ samples.
//! // Inject a target at range bin 10 with a constant-phase tone across pulses.
//! let mut data_cube: Vec<Vec<(f64, f64)>> = Vec::new();
//! let doppler_freq = 200.0; // Hz
//! for pulse_idx in 0..16 {
//!     let mut row = vec![(0.0, 0.0); 64];
//!     let t = pulse_idx as f64 / 1000.0; // time = pulse_idx / PRF
//!     let phase = 2.0 * std::f64::consts::PI * doppler_freq * t;
//!     row[10] = (phase.cos(), phase.sin());
//!     data_cube.push(row);
//! }
//!
//! // Apply 2-pulse MTI to suppress zero-Doppler clutter
//! let mti = MtiFilter::new(2);
//! let filtered = mti.apply_mti(&data_cube);
//!
//! // Process into range-Doppler map
//! let rdm = processor.process(&data_cube);
//! assert_eq!(rdm.map.len(), 64);          // num_range_bins rows
//! assert_eq!(rdm.map[0].len(), 16);       // num_pulses Doppler bins
//!
//! // Detect targets above threshold
//! let detections = processor.detect_targets(&rdm, 6.0);
//! assert!(!detections.is_empty());
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
const C: f64 = 299_792_458.0;

/// Configuration for the pulse-Doppler processor.
#[derive(Debug, Clone)]
pub struct PdConfig {
    /// Number of range bins (fast-time samples per pulse).
    pub num_range_bins: usize,
    /// Number of pulses in the CPI (coherent processing interval).
    pub num_pulses: usize,
    /// Pulse repetition frequency in Hz.
    pub prf_hz: f64,
    /// ADC sample rate in Hz.
    pub sample_rate: f64,
    /// Radar wavelength in metres (lambda = c / f_carrier).
    pub wavelength_m: f64,
}

/// A 2-D range-Doppler map with associated axes.
#[derive(Debug, Clone)]
pub struct RangeDopplerMap {
    /// 2-D power map in dB, indexed as `map[range_bin][doppler_bin]`.
    pub map: Vec<Vec<f64>>,
    /// Range axis values in metres, length = `num_range_bins`.
    pub range_axis_m: Vec<f64>,
    /// Doppler axis values in Hz, length = `num_pulses`.
    pub doppler_axis_hz: Vec<f64>,
    /// Velocity axis values in m/s, length = `num_pulses`.
    pub velocity_axis_mps: Vec<f64>,
}

/// A single detected target from the range-Doppler map.
#[derive(Debug, Clone)]
pub struct PdDetection {
    /// Estimated range in metres.
    pub range_m: f64,
    /// Estimated radial velocity in m/s.
    pub velocity_mps: f64,
    /// Estimated Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Signal-to-noise ratio estimate in dB.
    pub snr_db: f64,
}

/// Moving Target Indicator filter for clutter suppression.
///
/// An MTI canceller of order *N* applies an FIR filter of length *N* across the
/// slow-time (pulse) dimension so that zero-Doppler (stationary) clutter is
/// attenuated.
///
/// * Order 2 (2-pulse canceller): h = \[1, −1\]
/// * Order 3 (3-pulse canceller): h = \[1, −2, 1\]
#[derive(Debug, Clone)]
pub struct MtiFilter {
    /// Filter coefficients (binomial high-pass).
    coeffs: Vec<f64>,
    /// Filter order (number of pulses consumed per output sample).
    order: usize,
}

impl MtiFilter {
    /// Create a new MTI filter of the given order.
    ///
    /// * `order = 2` produces the 2-pulse canceller `[1, -1]`.
    /// * `order = 3` produces the 3-pulse canceller `[1, -2, 1]`.
    /// * Higher orders follow the binomial pattern (alternating-sign Pascal's row).
    ///
    /// # Panics
    ///
    /// Panics if `order < 2`.
    pub fn new(order: usize) -> Self {
        assert!(order >= 2, "MTI order must be >= 2");
        // Binomial coefficients with alternating signs: C(n-1,k) * (-1)^(n-1-k)
        let n = order - 1; // number of cancellations
        let mut coeffs = Vec::with_capacity(order);
        for k in 0..order {
            let binom = binomial(n, k) as f64;
            let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
            coeffs.push(sign * binom);
        }
        Self { coeffs, order }
    }

    /// Apply the MTI filter across the slow-time (pulse) dimension.
    ///
    /// Input: `pulses[pulse_idx][range_bin]` with shape `(num_pulses, num_range_bins)`.
    /// Output has `num_pulses - order + 1` rows.
    pub fn apply_mti(&self, pulses: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        let num_pulses = pulses.len();
        if num_pulses < self.order {
            return Vec::new();
        }
        let num_range_bins = pulses[0].len();
        let out_len = num_pulses - self.order + 1;
        let mut result = Vec::with_capacity(out_len);

        for i in 0..out_len {
            let mut row = vec![(0.0, 0.0); num_range_bins];
            for (k, &coeff) in self.coeffs.iter().enumerate() {
                for rb in 0..num_range_bins {
                    row[rb].0 += coeff * pulses[i + k][rb].0;
                    row[rb].1 += coeff * pulses[i + k][rb].1;
                }
            }
            result.push(row);
        }
        result
    }

    /// Return the filter coefficients.
    pub fn coefficients(&self) -> &[f64] {
        &self.coeffs
    }
}

/// Compute binomial coefficient C(n, k).
fn binomial(n: usize, k: usize) -> usize {
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut result: usize = 1;
    for i in 0..k {
        result = result * (n - i) / (i + 1);
    }
    result
}

/// Main pulse-Doppler processor.
#[derive(Debug, Clone)]
pub struct PulseDopplerProcessor {
    config: PdConfig,
}

impl PulseDopplerProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: PdConfig) -> Self {
        Self { config }
    }

    /// Convert a Doppler frequency (Hz) to radial velocity (m/s).
    ///
    /// `v = lambda * f_d / 2`
    pub fn doppler_to_velocity(&self, doppler_hz: f64) -> f64 {
        self.config.wavelength_m * doppler_hz / 2.0
    }

    /// Maximum unambiguous range: `R_max = c / (2 * PRF)`.
    pub fn max_unambiguous_range(&self) -> f64 {
        C / (2.0 * self.config.prf_hz)
    }

    /// Maximum unambiguous velocity: `v_max = lambda * PRF / 4`.
    pub fn max_unambiguous_velocity(&self) -> f64 {
        self.config.wavelength_m * self.config.prf_hz / 4.0
    }

    /// Process a data cube into a range-Doppler map.
    ///
    /// `data_cube[pulse_idx][range_bin]` — each entry is an `(I, Q)` pair.
    ///
    /// The Doppler FFT is applied across the pulse (slow-time) dimension for
    /// every range bin, producing a 2-D power map in dB.
    pub fn process(&self, data_cube: &[Vec<(f64, f64)>]) -> RangeDopplerMap {
        let num_pulses = data_cube.len();
        let num_range_bins = if num_pulses > 0 { data_cube[0].len() } else { 0 };

        // Build axes
        let range_axis_m = self.build_range_axis(num_range_bins);
        let doppler_axis_hz = self.build_doppler_axis(num_pulses);
        let velocity_axis_mps: Vec<f64> = doppler_axis_hz
            .iter()
            .map(|&fd| self.doppler_to_velocity(fd))
            .collect();

        // For each range bin, collect the slow-time vector and compute the DFT
        let mut map = vec![vec![0.0f64; num_pulses]; num_range_bins];

        for rb in 0..num_range_bins {
            // Collect slow-time samples for this range bin
            let slow_time: Vec<(f64, f64)> =
                data_cube.iter().map(|pulse| pulse[rb]).collect();

            // Apply Hamming window
            let windowed = apply_hamming_window(&slow_time);

            // Compute DFT (Doppler FFT)
            let spectrum = dft(&windowed);

            // FFT-shift: move DC to centre
            let shifted = fft_shift(&spectrum);

            // Convert to power in dB
            for (di, &(re, im)) in shifted.iter().enumerate() {
                let power = re * re + im * im;
                map[rb][di] = 10.0 * (power + 1e-30).log10();
            }
        }

        RangeDopplerMap {
            map,
            range_axis_m,
            doppler_axis_hz,
            velocity_axis_mps,
        }
    }

    /// Detect targets in the range-Doppler map using 2-D Cell-Averaging CFAR.
    ///
    /// `threshold_db` is the CFAR offset above the estimated noise floor.
    pub fn detect_targets(
        &self,
        rdm: &RangeDopplerMap,
        threshold_db: f64,
    ) -> Vec<PdDetection> {
        let num_range = rdm.map.len();
        if num_range == 0 {
            return Vec::new();
        }
        let num_doppler = rdm.map[0].len();

        // 2D CA-CFAR parameters
        let guard_range = 2;
        let guard_doppler = 2;
        let train_range = 4;
        let train_doppler = 4;

        let mut detections = Vec::new();

        for ri in 0..num_range {
            for di in 0..num_doppler {
                let cell_under_test = rdm.map[ri][di];

                // Collect training cell values (power in dB, excluding guard cells)
                let mut sum = 0.0;
                let mut count = 0usize;

                let r_lo = if ri >= guard_range + train_range {
                    ri - guard_range - train_range
                } else {
                    0
                };
                let r_hi = (ri + guard_range + train_range + 1).min(num_range);
                let d_lo = if di >= guard_doppler + train_doppler {
                    di - guard_doppler - train_doppler
                } else {
                    0
                };
                let d_hi = (di + guard_doppler + train_doppler + 1).min(num_doppler);

                for r in r_lo..r_hi {
                    for d in d_lo..d_hi {
                        // Skip guard region and CUT
                        let in_guard_r =
                            r >= ri.saturating_sub(guard_range) && r <= ri + guard_range;
                        let in_guard_d =
                            d >= di.saturating_sub(guard_doppler) && d <= di + guard_doppler;
                        if in_guard_r && in_guard_d {
                            continue;
                        }
                        sum += rdm.map[r][d];
                        count += 1;
                    }
                }

                if count == 0 {
                    continue;
                }

                let noise_estimate = sum / count as f64;
                let snr_db = cell_under_test - noise_estimate;

                if snr_db > threshold_db {
                    let range_m = if ri < rdm.range_axis_m.len() {
                        rdm.range_axis_m[ri]
                    } else {
                        0.0
                    };
                    let doppler_hz = if di < rdm.doppler_axis_hz.len() {
                        rdm.doppler_axis_hz[di]
                    } else {
                        0.0
                    };
                    let velocity_mps = self.doppler_to_velocity(doppler_hz);

                    detections.push(PdDetection {
                        range_m,
                        velocity_mps,
                        doppler_hz,
                        snr_db,
                    });
                }
            }
        }

        detections
    }

    /// Build the range axis in metres for the given number of range bins.
    fn build_range_axis(&self, num_bins: usize) -> Vec<f64> {
        let range_res = C / (2.0 * self.config.sample_rate);
        (0..num_bins).map(|i| i as f64 * range_res).collect()
    }

    /// Build the Doppler axis in Hz (FFT-shifted so DC is in the centre).
    fn build_doppler_axis(&self, n: usize) -> Vec<f64> {
        if n == 0 {
            return Vec::new();
        }
        let df = self.config.prf_hz / n as f64;
        (0..n)
            .map(|i| {
                let k = if i < (n + 1) / 2 {
                    // negative half first after fft_shift: -(n/2) .. -1
                    i as f64 - (n / 2) as f64
                } else {
                    i as f64 - (n / 2) as f64
                };
                k * df
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Apply a Hamming window to a sequence of complex samples.
fn apply_hamming_window(samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = samples.len();
    if n == 0 {
        return Vec::new();
    }
    samples
        .iter()
        .enumerate()
        .map(|(i, &(re, im))| {
            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1).max(1) as f64).cos();
            (re * w, im * w)
        })
        .collect()
}

/// Compute the Discrete Fourier Transform (radix-2 when possible, otherwise DFT).
fn dft(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return x.to_vec();
    }
    // If n is a power of two, use Cooley-Tukey FFT; otherwise brute-force DFT
    if n.is_power_of_two() {
        fft_radix2(x)
    } else {
        dft_brute(x)
    }
}

/// Brute-force O(N^2) DFT.
fn dft_brute(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    let mut out = vec![(0.0, 0.0); n];
    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (i, &(xr, xi)) in x.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            let (s, c) = angle.sin_cos();
            sum_re += xr * c - xi * s;
            sum_im += xr * s + xi * c;
        }
        out[k] = (sum_re, sum_im);
    }
    out
}

/// Radix-2 decimation-in-time FFT (iterative, in-place).
fn fft_radix2(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    debug_assert!(n.is_power_of_two());

    // Bit-reversal permutation
    let mut a: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
    let bits = n.trailing_zeros();
    for i in 0..n {
        let j = i.reverse_bits() >> (usize::BITS - bits);
        a[j] = x[i];
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = -2.0 * PI / len as f64;
        let mut i = 0;
        while i < n {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let (s, c) = angle.sin_cos();
                let t_re = a[i + k + half].0 * c - a[i + k + half].1 * s;
                let t_im = a[i + k + half].0 * s + a[i + k + half].1 * c;
                let u = a[i + k];
                a[i + k] = (u.0 + t_re, u.1 + t_im);
                a[i + k + half] = (u.0 - t_re, u.1 - t_im);
            }
            i += len;
        }
        len *= 2;
    }
    a
}

/// FFT-shift: swap left and right halves so that DC is centred.
fn fft_shift(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    let mid = n / 2;
    let mut out = Vec::with_capacity(n);
    out.extend_from_slice(&x[mid..]);
    out.extend_from_slice(&x[..mid]);
    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PdConfig {
        PdConfig {
            num_range_bins: 64,
            num_pulses: 16,
            prf_hz: 1000.0,
            sample_rate: 1e6,
            wavelength_m: 0.03,
        }
    }

    fn make_empty_cube(num_pulses: usize, num_range_bins: usize) -> Vec<Vec<(f64, f64)>> {
        vec![vec![(0.0, 0.0); num_range_bins]; num_pulses]
    }

    #[test]
    fn test_max_unambiguous_range() {
        let proc = PulseDopplerProcessor::new(default_config());
        // R_max = c / (2 * PRF) = 299792458 / 2000 ~ 149896.229 m
        let r_max = proc.max_unambiguous_range();
        assert!((r_max - 149_896.229).abs() < 1.0);
    }

    #[test]
    fn test_max_unambiguous_velocity() {
        let proc = PulseDopplerProcessor::new(default_config());
        // v_max = lambda * PRF / 4 = 0.03 * 1000 / 4 = 7.5 m/s
        let v_max = proc.max_unambiguous_velocity();
        assert!((v_max - 7.5).abs() < 1e-10);
    }

    #[test]
    fn test_doppler_to_velocity() {
        let proc = PulseDopplerProcessor::new(default_config());
        // v = lambda * fd / 2 = 0.03 * 500 / 2 = 7.5
        assert!((proc.doppler_to_velocity(500.0) - 7.5).abs() < 1e-10);
        assert!((proc.doppler_to_velocity(0.0)).abs() < 1e-10);
        assert!((proc.doppler_to_velocity(-500.0) + 7.5).abs() < 1e-10);
    }

    #[test]
    fn test_mti_order2_coefficients() {
        let mti = MtiFilter::new(2);
        let c = mti.coefficients();
        assert_eq!(c.len(), 2);
        assert!((c[0] - 1.0).abs() < 1e-12);
        assert!((c[1] + 1.0).abs() < 1e-12); // -1
    }

    #[test]
    fn test_mti_order3_coefficients() {
        let mti = MtiFilter::new(3);
        let c = mti.coefficients();
        assert_eq!(c.len(), 3);
        assert!((c[0] - 1.0).abs() < 1e-12);
        assert!((c[1] + 2.0).abs() < 1e-12); // -2
        assert!((c[2] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_mti_suppresses_dc() {
        // A constant (DC) signal across pulses should be cancelled by 2-pulse MTI.
        let mti = MtiFilter::new(2);
        let pulses: Vec<Vec<(f64, f64)>> = vec![
            vec![(1.0, 0.0); 4],
            vec![(1.0, 0.0); 4],
            vec![(1.0, 0.0); 4],
        ];
        let result = mti.apply_mti(&pulses);
        assert_eq!(result.len(), 2); // 3 - 2 + 1
        for row in &result {
            for &(re, im) in row {
                assert!(re.abs() < 1e-12);
                assert!(im.abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_mti_passes_alternating() {
        // An alternating ±1 signal (max Doppler) should pass through the 2-pulse canceller.
        let mti = MtiFilter::new(2);
        let pulses: Vec<Vec<(f64, f64)>> = vec![
            vec![(1.0, 0.0)],
            vec![(-1.0, 0.0)],
            vec![(1.0, 0.0)],
        ];
        let result = mti.apply_mti(&pulses);
        // h=[1,-1] convolved with [1,-1,1]: output [1-(-1), -1-1] = [2, -2]
        assert_eq!(result.len(), 2);
        assert!((result[0][0].0 - 2.0).abs() < 1e-12);
        assert!((result[1][0].0 + 2.0).abs() < 1e-12);
    }

    #[test]
    fn test_mti_empty_input() {
        let mti = MtiFilter::new(2);
        let result = mti.apply_mti(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_mti_insufficient_pulses() {
        let mti = MtiFilter::new(3);
        let pulses = vec![vec![(1.0, 0.0); 4]; 2]; // only 2 pulses, need 3
        let result = mti.apply_mti(&pulses);
        assert!(result.is_empty());
    }

    #[test]
    fn test_process_shape() {
        let config = default_config();
        let proc = PulseDopplerProcessor::new(config.clone());
        let cube = make_empty_cube(config.num_pulses, config.num_range_bins);
        let rdm = proc.process(&cube);
        assert_eq!(rdm.map.len(), config.num_range_bins);
        assert_eq!(rdm.map[0].len(), config.num_pulses);
        assert_eq!(rdm.range_axis_m.len(), config.num_range_bins);
        assert_eq!(rdm.doppler_axis_hz.len(), config.num_pulses);
        assert_eq!(rdm.velocity_axis_mps.len(), config.num_pulses);
    }

    #[test]
    fn test_process_single_target_peak() {
        // Inject a constant-amplitude target at range bin 5 with a known Doppler.
        let config = PdConfig {
            num_range_bins: 32,
            num_pulses: 16,
            prf_hz: 1000.0,
            sample_rate: 1e6,
            wavelength_m: 0.03,
        };
        let proc = PulseDopplerProcessor::new(config.clone());
        let target_rb = 5;
        let doppler_bin = 4; // corresponds to a positive Doppler shift
        let mut cube = make_empty_cube(config.num_pulses, config.num_range_bins);
        for p in 0..config.num_pulses {
            let phase = 2.0 * PI * doppler_bin as f64 * p as f64 / config.num_pulses as f64;
            cube[p][target_rb] = (phase.cos(), phase.sin());
        }
        let rdm = proc.process(&cube);

        // The peak should be at range bin 5. Find the Doppler bin with max power there.
        let row = &rdm.map[target_rb];
        let max_di = row
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // After fft_shift the bin mapping is: shifted_bin = (original_bin + N/2) % N.
        // DFT bin `doppler_bin` maps to shifted index `(doppler_bin + N/2) % N`.
        let expected_shifted = (doppler_bin + config.num_pulses / 2) % config.num_pulses;
        assert_eq!(max_di, expected_shifted);
    }

    #[test]
    fn test_detect_targets_finds_strong_signal() {
        let config = PdConfig {
            num_range_bins: 32,
            num_pulses: 16,
            prf_hz: 1000.0,
            sample_rate: 1e6,
            wavelength_m: 0.03,
        };
        let proc = PulseDopplerProcessor::new(config.clone());
        let target_rb = 15;
        let mut cube = make_empty_cube(config.num_pulses, config.num_range_bins);
        // Place a strong tone at a known Doppler
        for p in 0..config.num_pulses {
            let phase = 2.0 * PI * 3.0 * p as f64 / config.num_pulses as f64;
            cube[p][target_rb] = (10.0 * phase.cos(), 10.0 * phase.sin());
        }
        let rdm = proc.process(&cube);
        let dets = proc.detect_targets(&rdm, 10.0);
        assert!(!dets.is_empty());
        // At least one detection should be at the target range bin
        let range_res = C / (2.0 * config.sample_rate);
        let expected_range = target_rb as f64 * range_res;
        let found = dets.iter().any(|d| (d.range_m - expected_range).abs() < range_res);
        assert!(found, "Expected detection near range bin {}", target_rb);
    }

    #[test]
    fn test_detect_no_targets_in_noise_floor() {
        let config = default_config();
        let proc = PulseDopplerProcessor::new(config.clone());
        let cube = make_empty_cube(config.num_pulses, config.num_range_bins);
        let rdm = proc.process(&cube);
        // With a very high threshold, no detections should occur on an all-zero cube
        let dets = proc.detect_targets(&rdm, 100.0);
        assert!(dets.is_empty());
    }

    #[test]
    fn test_range_axis_spacing() {
        let config = default_config();
        let proc = PulseDopplerProcessor::new(config.clone());
        let cube = make_empty_cube(config.num_pulses, config.num_range_bins);
        let rdm = proc.process(&cube);
        let range_res = C / (2.0 * config.sample_rate);
        // First range bin at 0
        assert!((rdm.range_axis_m[0]).abs() < 1e-10);
        // Second at range_res
        assert!((rdm.range_axis_m[1] - range_res).abs() < 1e-6);
    }

    #[test]
    fn test_dft_single_tone() {
        // DFT of a single complex exponential at bin k=2 should peak at bin 2
        let n = 8;
        let k0 = 2;
        let x: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * k0 as f64 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let spectrum = dft(&x);
        let mags: Vec<f64> = spectrum.iter().map(|&(r, i)| (r * r + i * i).sqrt()).collect();
        let peak_bin = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_bin, k0);
        // Peak magnitude should be close to N
        assert!((mags[k0] - n as f64).abs() < 1e-6);
    }

    #[test]
    fn test_fft_shift_even() {
        let input = vec![(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)];
        let shifted = fft_shift(&input);
        // Should swap halves: [2,3,0,1]
        assert!((shifted[0].0 - 2.0).abs() < 1e-12);
        assert!((shifted[1].0 - 3.0).abs() < 1e-12);
        assert!((shifted[2].0 - 0.0).abs() < 1e-12);
        assert!((shifted[3].0 - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_binomial_coefficients() {
        assert_eq!(binomial(0, 0), 1);
        assert_eq!(binomial(4, 2), 6);
        assert_eq!(binomial(5, 0), 1);
        assert_eq!(binomial(5, 5), 1);
        assert_eq!(binomial(3, 2), 3);
    }

    #[test]
    #[should_panic(expected = "MTI order must be >= 2")]
    fn test_mti_order_too_small() {
        MtiFilter::new(1);
    }

    #[test]
    fn test_velocity_axis_matches_doppler_axis() {
        let config = default_config();
        let proc = PulseDopplerProcessor::new(config.clone());
        let cube = make_empty_cube(config.num_pulses, config.num_range_bins);
        let rdm = proc.process(&cube);
        for (i, (&fd, &v)) in rdm.doppler_axis_hz.iter().zip(rdm.velocity_axis_mps.iter()).enumerate() {
            let expected_v = config.wavelength_m * fd / 2.0;
            assert!(
                (v - expected_v).abs() < 1e-10,
                "Mismatch at index {}: fd={}, v={}, expected_v={}",
                i, fd, v, expected_v
            );
        }
    }
}
