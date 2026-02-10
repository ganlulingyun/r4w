//! Range-Doppler Map — Pulse-Doppler Radar Processing
//!
//! Generates a two-dimensional range-Doppler map from a coherent processing
//! interval (CPI) of radar pulses. Range compression is performed via matched
//! filtering (cross-correlation with the transmitted reference waveform), and
//! Doppler processing is performed via DFT across the slow-time dimension.
//!
//! The resulting map shows target returns as peaks in the (range bin, Doppler
//! bin) plane, enabling simultaneous measurement of target range and radial
//! velocity. This is the core processing step in pulse-Doppler radar systems
//! used for airborne, ground-based, and maritime surveillance.
//!
//! ## Processing Chain
//!
//! ```text
//! Pulse 0: ──[matched filter]──► range profile 0 ─┐
//! Pulse 1: ──[matched filter]──► range profile 1 ─┤
//!   ...                                            ├──[DFT across pulses]──► Range-Doppler Map
//! Pulse N-1: ──[matched filter]──► range profile N-1─┘
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::range_doppler_map::RangeDopplerMap;
//!
//! // Create a simple reference pulse (real-valued tone represented as IQ)
//! let reference: Vec<(f64, f64)> = (0..16)
//!     .map(|i| ((i as f64 * 0.3).cos(), (i as f64 * 0.3).sin()))
//!     .collect();
//!
//! // Simulate 8 received pulses (reference + noise)
//! let pulses: Vec<Vec<(f64, f64)>> = (0..8)
//!     .map(|p| {
//!         reference.iter().enumerate().map(|(i, &(re, im))| {
//!             let phase = p as f64 * 0.1; // slight Doppler shift per pulse
//!             let c = phase.cos();
//!             let s = phase.sin();
//!             (re * c - im * s, re * s + im * c)
//!         }).collect()
//!     })
//!     .collect();
//!
//! let rdm = RangeDopplerMap::generate(&pulses, &reference);
//! let (rb, db, power) = rdm.peak();
//! assert!(power > 0.0);
//! ```

use std::f64::consts::PI;

/// Speed of light in vacuum (m/s).
const C: f64 = 299_792_458.0;

/// A two-dimensional range-Doppler map produced by pulse-Doppler processing.
///
/// Rows correspond to range bins (fast-time, from matched filtering) and
/// columns correspond to Doppler bins (slow-time, from DFT across pulses).
/// Values represent power (magnitude squared).
#[derive(Debug, Clone)]
pub struct RangeDopplerMap {
    /// Number of range bins (fast-time dimension).
    pub num_range_bins: usize,
    /// Number of Doppler bins (slow-time dimension, equals number of pulses).
    pub num_doppler_bins: usize,
    /// 2D power data indexed as `data[range_bin][doppler_bin]`.
    pub data: Vec<Vec<f64>>,
}

impl RangeDopplerMap {
    /// Create an empty range-Doppler map with the given dimensions.
    ///
    /// All power values are initialized to zero.
    pub fn new(num_range_bins: usize, num_doppler_bins: usize) -> Self {
        Self {
            num_range_bins,
            num_doppler_bins,
            data: vec![vec![0.0; num_doppler_bins]; num_range_bins],
        }
    }

    /// Generate a range-Doppler map from a coherent pulse train.
    ///
    /// Each element of `pulses` is one received pulse represented as IQ samples
    /// `(re, im)`. The `reference` is the transmitted waveform used for matched
    /// filtering (pulse compression).
    ///
    /// Processing steps:
    /// 1. **Range compression**: cross-correlate each pulse with `reference` to
    ///    produce a range profile (matched filter output magnitude squared).
    /// 2. **Doppler processing**: for each range bin, compute the DFT across
    ///    pulses (slow-time) and take magnitude squared.
    ///
    /// Returns a map with `num_range_bins` equal to the cross-correlation length
    /// (pulse length + reference length - 1) and `num_doppler_bins` equal to the
    /// number of pulses.
    ///
    /// # Panics
    ///
    /// Returns an empty map if `pulses` is empty or `reference` is empty.
    pub fn generate(pulses: &[Vec<(f64, f64)>], reference: &[(f64, f64)]) -> Self {
        let num_pulses = pulses.len();
        if num_pulses == 0 || reference.is_empty() {
            return Self::new(0, 0);
        }

        // Step 1: Range compression via cross-correlation with reference.
        // The matched filter is the time-reversed conjugate of the reference.
        // Cross-correlation length = pulse_len + ref_len - 1.
        let ref_len = reference.len();
        let pulse_len = pulses[0].len();
        let corr_len = pulse_len + ref_len - 1;

        // Compute range-compressed profiles for all pulses.
        // Store as complex values (before magnitude) so Doppler DFT operates
        // on the complex range profiles, preserving phase information.
        let mut range_profiles: Vec<Vec<(f64, f64)>> = Vec::with_capacity(num_pulses);

        for pulse in pulses {
            let profile = cross_correlate(pulse, reference, corr_len);
            range_profiles.push(profile);
        }

        // Step 2: Doppler processing — DFT across slow-time for each range bin.
        let num_range_bins = corr_len;
        let num_doppler_bins = num_pulses;
        let mut data = vec![vec![0.0; num_doppler_bins]; num_range_bins];

        for r in 0..num_range_bins {
            // Collect slow-time samples at this range bin across all pulses.
            let slow_time: Vec<(f64, f64)> = range_profiles
                .iter()
                .map(|profile| profile[r])
                .collect();

            // DFT across slow-time dimension.
            let spectrum = dft(&slow_time);

            // Store magnitude squared (power).
            for (d, &(re, im)) in spectrum.iter().enumerate() {
                data[r][d] = re * re + im * im;
            }
        }

        Self {
            num_range_bins,
            num_doppler_bins,
            data,
        }
    }

    /// Get the power value at a specific (range_bin, doppler_bin).
    ///
    /// Returns 0.0 if the indices are out of bounds.
    pub fn get(&self, range_bin: usize, doppler_bin: usize) -> f64 {
        if range_bin < self.num_range_bins && doppler_bin < self.num_doppler_bins {
            self.data[range_bin][doppler_bin]
        } else {
            0.0
        }
    }

    /// Extract a 1D range profile (range cut) at a fixed Doppler bin.
    ///
    /// Returns the power values across all range bins for the specified
    /// Doppler bin.
    pub fn range_profile(&self, doppler_bin: usize) -> Vec<f64> {
        if doppler_bin >= self.num_doppler_bins {
            return vec![];
        }
        self.data.iter().map(|row| row[doppler_bin]).collect()
    }

    /// Extract a 1D Doppler profile (Doppler cut) at a fixed range bin.
    ///
    /// Returns the power values across all Doppler bins for the specified
    /// range bin.
    pub fn doppler_profile(&self, range_bin: usize) -> Vec<f64> {
        if range_bin >= self.num_range_bins {
            return vec![];
        }
        self.data[range_bin].clone()
    }

    /// Find the peak power location in the map.
    ///
    /// Returns `(range_bin, doppler_bin, power)` of the maximum value.
    /// For an empty map, returns `(0, 0, 0.0)`.
    pub fn peak(&self) -> (usize, usize, f64) {
        let mut max_power = 0.0;
        let mut max_r = 0;
        let mut max_d = 0;

        for r in 0..self.num_range_bins {
            for d in 0..self.num_doppler_bins {
                let v = self.data[r][d];
                if v > max_power {
                    max_power = v;
                    max_r = r;
                    max_d = d;
                }
            }
        }

        (max_r, max_d, max_power)
    }

    /// Convert the entire map to decibel (dB) scale.
    ///
    /// Computes `10 * log10(power)` for each cell. Zero or negative values
    /// are clamped to -300 dB to avoid -infinity.
    pub fn as_db(&self) -> Vec<Vec<f64>> {
        self.data
            .iter()
            .map(|row| {
                row.iter()
                    .map(|&v| {
                        if v > 0.0 {
                            10.0 * v.log10()
                        } else {
                            -300.0
                        }
                    })
                    .collect()
            })
            .collect()
    }

    /// Normalize all power values to the range [0, 1].
    ///
    /// Divides every cell by the peak power value. If the map is entirely
    /// zero, no changes are made.
    pub fn normalize(&mut self) {
        let (_, _, peak) = self.peak();
        if peak > 0.0 {
            for row in &mut self.data {
                for v in row.iter_mut() {
                    *v /= peak;
                }
            }
        }
    }

    /// Convert a range bin index to physical distance in meters.
    ///
    /// Uses the two-way propagation delay: `distance = bin * c / (2 * sample_rate)`.
    ///
    /// # Arguments
    ///
    /// * `range_bin` - The range bin index.
    /// * `sample_rate` - The fast-time sample rate in Hz.
    pub fn range_to_meters(range_bin: usize, sample_rate: f64) -> f64 {
        range_bin as f64 * C / (2.0 * sample_rate)
    }

    /// Convert a Doppler bin index to radial velocity in m/s.
    ///
    /// The Doppler axis spans [0, PRF) in frequency. Bins above `num_bins / 2`
    /// represent negative Doppler (approaching targets, depending on convention).
    /// The velocity is computed as:
    ///
    /// ```text
    /// f_doppler = (bin / num_bins) * PRF        if bin <= num_bins / 2
    /// f_doppler = ((bin - num_bins) / num_bins) * PRF   otherwise
    /// velocity  = f_doppler * wavelength / 2
    /// ```
    ///
    /// # Arguments
    ///
    /// * `doppler_bin` - The Doppler bin index.
    /// * `num_bins` - Total number of Doppler bins.
    /// * `prf` - Pulse repetition frequency in Hz.
    /// * `wavelength` - Radar wavelength in meters.
    pub fn doppler_to_velocity(
        doppler_bin: usize,
        num_bins: usize,
        prf: f64,
        wavelength: f64,
    ) -> f64 {
        if num_bins == 0 {
            return 0.0;
        }
        let f_doppler = if doppler_bin <= num_bins / 2 {
            doppler_bin as f64 / num_bins as f64 * prf
        } else {
            (doppler_bin as f64 - num_bins as f64) / num_bins as f64 * prf
        };
        f_doppler * wavelength / 2.0
    }
}

/// Cross-correlate `signal` with `reference`, returning complex output.
///
/// Computes: `y[n] = sum_k signal[k] * conj(reference[k - n])`
/// for n in 0..corr_len where corr_len = signal.len() + reference.len() - 1.
fn cross_correlate(
    signal: &[(f64, f64)],
    reference: &[(f64, f64)],
    corr_len: usize,
) -> Vec<(f64, f64)> {
    let sig_len = signal.len();
    let ref_len = reference.len();
    let mut result = vec![(0.0, 0.0); corr_len];

    for n in 0..corr_len {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;

        for k in 0..sig_len {
            // Index into reference: j = k - (n - (ref_len - 1))
            let j = k as isize - n as isize + ref_len as isize - 1;
            if j >= 0 && (j as usize) < ref_len {
                let (s_re, s_im) = signal[k];
                let (r_re, r_im) = reference[j as usize];
                // Multiply signal[k] by conj(reference[j])
                // (a + bi)(c - di) = (ac + bd) + (bc - ad)i
                sum_re += s_re * r_re + s_im * r_im;
                sum_im += s_im * r_re - s_re * r_im;
            }
        }

        result[n] = (sum_re, sum_im);
    }

    result
}

/// Simple O(N^2) Discrete Fourier Transform for IQ samples.
///
/// Computes X[k] = sum_{n=0}^{N-1} x[n] * exp(-j 2 pi k n / N).
fn dft(x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = x.len();
    if n == 0 {
        return vec![];
    }

    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (j, &(x_re, x_im)) in x.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            // (x_re + j*x_im) * (cos_a + j*sin_a)
            // = (x_re*cos_a - x_im*sin_a) + j*(x_re*sin_a + x_im*cos_a)
            sum_re += x_re * cos_a - x_im * sin_a;
            sum_im += x_re * sin_a + x_im * cos_a;
        }
        result[k] = (sum_re, sum_im);
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_generation() {
        // Empty pulses should produce an empty map.
        let rdm = RangeDopplerMap::generate(&[], &[(1.0, 0.0)]);
        assert_eq!(rdm.num_range_bins, 0);
        assert_eq!(rdm.num_doppler_bins, 0);
        assert!(rdm.data.is_empty());

        // Empty reference should also produce an empty map.
        let rdm2 = RangeDopplerMap::generate(&[vec![(1.0, 0.0)]], &[]);
        assert_eq!(rdm2.num_range_bins, 0);
        assert_eq!(rdm2.num_doppler_bins, 0);
    }

    #[test]
    fn test_single_pulse() {
        // A single pulse correlated with itself should produce a triangular
        // range profile with one Doppler bin.
        let reference: Vec<(f64, f64)> = vec![(1.0, 0.0); 8];
        let pulses = vec![reference.clone()];

        let rdm = RangeDopplerMap::generate(&pulses, &reference);
        assert_eq!(rdm.num_range_bins, 15); // 8 + 8 - 1
        assert_eq!(rdm.num_doppler_bins, 1);

        // Peak should be at the center range bin (bin 7).
        let (peak_r, peak_d, power) = rdm.peak();
        assert_eq!(peak_r, 7);
        assert_eq!(peak_d, 0);
        assert!(power > 0.0);

        // The cross-correlation peak of a length-N all-ones pulse with itself
        // is N (real). After DFT of 1 sample, the power is |N|^2 = N^2.
        let expected = 64.0; // N=8, |xcorr_peak|^2 = 8^2 = 64
        assert!(
            (power - expected).abs() / expected < 0.01,
            "power = {power}, expected = {expected}"
        );
    }

    #[test]
    fn test_peak_detection() {
        // Create pulses with a target at a known range bin.
        let ref_len = 8;
        let pulse_len = 20;
        let reference: Vec<(f64, f64)> = (0..ref_len)
            .map(|i| ((i as f64 * 0.5).cos(), (i as f64 * 0.5).sin()))
            .collect();

        // Place the reference signal starting at sample 6 in each pulse.
        let num_pulses = 4;
        let pulses: Vec<Vec<(f64, f64)>> = (0..num_pulses)
            .map(|_| {
                let mut pulse = vec![(0.0, 0.0); pulse_len];
                for (i, &s) in reference.iter().enumerate() {
                    pulse[6 + i] = s;
                }
                pulse
            })
            .collect();

        let rdm = RangeDopplerMap::generate(&pulses, &reference);
        let (peak_r, _peak_d, power) = rdm.peak();

        // The cross-correlation peak should appear near range bin
        // corresponding to the delay offset.
        assert!(power > 0.0, "peak power should be positive");
        // The correlation peak for delay=6 with ref_len=8 in a signal of
        // length 20 appears at bin = delay + ref_len - 1 = 13.
        assert_eq!(peak_r, 13, "peak range bin = {peak_r}, expected 13");
    }

    #[test]
    fn test_range_profile() {
        let reference: Vec<(f64, f64)> = vec![(1.0, 0.0); 4];
        let pulses = vec![reference.clone(); 3];

        let rdm = RangeDopplerMap::generate(&pulses, &reference);
        let profile = rdm.range_profile(0);

        assert_eq!(profile.len(), rdm.num_range_bins);

        // Peak of the range profile at Doppler bin 0 should be at the
        // correlation center.
        let peak_idx = profile
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 3); // ref_len - 1

        // Out-of-bounds Doppler bin returns empty.
        let empty = rdm.range_profile(100);
        assert!(empty.is_empty());
    }

    #[test]
    fn test_doppler_profile() {
        let reference: Vec<(f64, f64)> = vec![(1.0, 0.0); 4];
        let pulses = vec![reference.clone(); 6];

        let rdm = RangeDopplerMap::generate(&pulses, &reference);
        let profile = rdm.doppler_profile(3); // at peak range bin

        assert_eq!(profile.len(), rdm.num_doppler_bins);
        assert_eq!(profile.len(), 6);

        // For identical pulses (zero Doppler), the DFT should peak at bin 0.
        let peak_idx = profile
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 0, "zero Doppler should peak at bin 0");

        // Out-of-bounds range bin returns empty.
        let empty = rdm.doppler_profile(1000);
        assert!(empty.is_empty());
    }

    #[test]
    fn test_db_conversion() {
        let mut rdm = RangeDopplerMap::new(3, 2);
        rdm.data[0][0] = 1.0;
        rdm.data[0][1] = 10.0;
        rdm.data[1][0] = 100.0;
        rdm.data[1][1] = 0.001;
        rdm.data[2][0] = 0.0; // should map to -300 dB
        rdm.data[2][1] = 1000.0;

        let db = rdm.as_db();
        assert_eq!(db.len(), 3);
        assert_eq!(db[0].len(), 2);

        assert!((db[0][0] - 0.0).abs() < 1e-10, "10*log10(1) = 0");
        assert!((db[0][1] - 10.0).abs() < 1e-10, "10*log10(10) = 10");
        assert!((db[1][0] - 20.0).abs() < 1e-10, "10*log10(100) = 20");
        assert!((db[1][1] - (-30.0)).abs() < 1e-10, "10*log10(0.001) = -30");
        assert!((db[2][0] - (-300.0)).abs() < 1e-10, "zero maps to -300");
        assert!((db[2][1] - 30.0).abs() < 1e-10, "10*log10(1000) = 30");
    }

    #[test]
    fn test_normalize() {
        let mut rdm = RangeDopplerMap::new(2, 2);
        rdm.data[0][0] = 4.0;
        rdm.data[0][1] = 8.0;
        rdm.data[1][0] = 2.0;
        rdm.data[1][1] = 16.0;

        rdm.normalize();

        assert!((rdm.data[1][1] - 1.0).abs() < 1e-10, "peak should be 1.0");
        assert!(
            (rdm.data[0][0] - 0.25).abs() < 1e-10,
            "4/16 = 0.25, got {}",
            rdm.data[0][0]
        );
        assert!(
            (rdm.data[0][1] - 0.5).abs() < 1e-10,
            "8/16 = 0.5, got {}",
            rdm.data[0][1]
        );
        assert!(
            (rdm.data[1][0] - 0.125).abs() < 1e-10,
            "2/16 = 0.125, got {}",
            rdm.data[1][0]
        );

        // Normalizing an all-zero map should be a no-op.
        let mut zero_rdm = RangeDopplerMap::new(3, 3);
        zero_rdm.normalize();
        assert!((zero_rdm.data[0][0]).abs() < 1e-10);
    }

    #[test]
    fn test_range_to_meters() {
        // At 10 MHz sample rate, each bin corresponds to c / (2 * 10e6) = 14.99 m.
        let sample_rate = 10e6;
        let d0 = RangeDopplerMap::range_to_meters(0, sample_rate);
        assert!((d0 - 0.0).abs() < 1e-10);

        let d1 = RangeDopplerMap::range_to_meters(1, sample_rate);
        let expected = C / (2.0 * sample_rate);
        assert!(
            (d1 - expected).abs() < 0.01,
            "d1 = {d1:.4}, expected = {expected:.4}"
        );

        let d10 = RangeDopplerMap::range_to_meters(10, sample_rate);
        assert!(
            (d10 - 10.0 * expected).abs() < 0.1,
            "d10 = {d10:.4}, expected = {:.4}",
            10.0 * expected
        );
    }

    #[test]
    fn test_doppler_to_velocity() {
        let prf = 1000.0; // 1 kHz PRF
        let wavelength = 0.03; // 10 GHz => ~3 cm wavelength
        let num_bins = 16;

        // Bin 0 => zero velocity.
        let v0 = RangeDopplerMap::doppler_to_velocity(0, num_bins, prf, wavelength);
        assert!((v0 - 0.0).abs() < 1e-10, "bin 0 should be 0 m/s");

        // Bin 1 => f_doppler = 1/16 * 1000 = 62.5 Hz => v = 62.5 * 0.03 / 2 = 0.9375 m/s.
        let v1 = RangeDopplerMap::doppler_to_velocity(1, num_bins, prf, wavelength);
        let expected_v1 = 62.5 * wavelength / 2.0;
        assert!(
            (v1 - expected_v1).abs() < 1e-10,
            "v1 = {v1}, expected = {expected_v1}"
        );

        // Bin num_bins/2 + 1 (bin 9) => negative Doppler (approaching).
        // f_doppler = (9 - 16) / 16 * 1000 = -437.5 Hz => v = -437.5 * 0.03 / 2 = -6.5625 m/s.
        let v9 = RangeDopplerMap::doppler_to_velocity(9, num_bins, prf, wavelength);
        let expected_v9 = (-437.5) * wavelength / 2.0;
        assert!(
            (v9 - expected_v9).abs() < 1e-10,
            "v9 = {v9}, expected = {expected_v9}"
        );

        // Zero bins edge case.
        let v_zero = RangeDopplerMap::doppler_to_velocity(0, 0, prf, wavelength);
        assert!((v_zero - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_multi_pulse() {
        // Generate pulses with a progressive phase shift to simulate Doppler.
        // A constant phase ramp across pulses should produce a Doppler peak
        // at a non-zero bin.
        let ref_len = 8;
        let num_pulses = 16;
        let reference: Vec<(f64, f64)> = vec![(1.0, 0.0); ref_len];

        let doppler_phase_per_pulse = 2.0 * PI / num_pulses as f64; // 1 cycle over CPI

        let pulses: Vec<Vec<(f64, f64)>> = (0..num_pulses)
            .map(|p| {
                let phase = p as f64 * doppler_phase_per_pulse;
                let c = phase.cos();
                let s = phase.sin();
                // Apply phase rotation to each sample in the pulse.
                reference
                    .iter()
                    .map(|&(re, im)| (re * c - im * s, re * s + im * c))
                    .collect()
            })
            .collect();

        let rdm = RangeDopplerMap::generate(&pulses, &reference);
        assert_eq!(rdm.num_range_bins, 2 * ref_len - 1);
        assert_eq!(rdm.num_doppler_bins, num_pulses);

        let (peak_r, peak_d, power) = rdm.peak();

        // The peak should be at the correlation center (range bin = ref_len - 1 = 7).
        assert_eq!(peak_r, 7, "peak range bin = {peak_r}, expected 7");

        // With 1 cycle of phase rotation over 16 pulses, the DFT peak should
        // be at Doppler bin 1.
        assert_eq!(peak_d, 1, "peak Doppler bin = {peak_d}, expected 1");

        assert!(power > 0.0, "peak power should be positive");

        // Verify that the Doppler bin 0 at the same range bin has much less
        // power than the peak.
        let power_dc = rdm.get(peak_r, 0);
        assert!(
            power_dc < power * 0.01,
            "DC Doppler power ({power_dc:.2}) should be much less than peak ({power:.2})"
        );
    }
}
