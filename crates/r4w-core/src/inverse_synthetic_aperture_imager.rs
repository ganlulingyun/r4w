//! Inverse Synthetic Aperture Radar (ISAR) image formation module.
//!
//! Performs ISAR imaging via motion compensation and range-Doppler processing.
//! Uses `(f64, f64)` tuples as complex numbers `(re, im)` with no external dependencies.
//!
//! # Example
//!
//! ```
//! use r4w_core::inverse_synthetic_aperture_imager::{IsarParams, InverseSyntheticApertureImager};
//!
//! let params = IsarParams {
//!     num_range_bins: 64,
//!     num_pulses: 32,
//!     bandwidth_hz: 500e6,
//!     center_freq_hz: 10e9,
//!     prf_hz: 1000.0,
//!     speed_of_light: 3e8,
//! };
//! let imager = InverseSyntheticApertureImager::new(params);
//!
//! // Create trivial test data (num_pulses x num_range_bins)
//! let data: Vec<Vec<(f64, f64)>> = vec![vec![(1.0, 0.0); 64]; 32];
//! let image = imager.range_doppler_image(&data);
//! assert_eq!(image.len(), 32);
//! assert_eq!(image[0].len(), 64);
//! ```

use std::f64::consts::PI;

// ─── Complex arithmetic helpers ────────────────────────────────────────────

type C64 = (f64, f64);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_mag_sq(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_mag(a: C64) -> f64 {
    c_mag_sq(a).sqrt()
}

#[inline]
fn c_phase(a: C64) -> f64 {
    a.1.atan2(a.0)
}

#[inline]
fn c_scale(s: f64, a: C64) -> C64 {
    (s * a.0, s * a.1)
}

#[inline]
fn c_exp(phase: f64) -> C64 {
    (phase.cos(), phase.sin())
}

// ─── Radix-2 DIT FFT (power-of-two) ───────────────────────────────────────

fn bit_reverse(x: usize, log2n: u32) -> usize {
    x.reverse_bits() >> (usize::BITS - log2n)
}

/// In-place radix-2 DIT FFT. `inverse` = true for IFFT.
fn fft_in_place(buf: &mut [C64], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");
    let log2n = n.trailing_zeros();

    // Bit-reversal permutation
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            buf.swap(i, j);
        }
    }

    let sign = if inverse { 1.0 } else { -1.0 };

    let mut half_size = 1;
    while half_size < n {
        let size = half_size * 2;
        let angle_step = sign * PI / half_size as f64;
        for k in (0..n).step_by(size) {
            for j in 0..half_size {
                let w = c_exp(angle_step * j as f64);
                let u = buf[k + j];
                let t = c_mul(w, buf[k + j + half_size]);
                buf[k + j] = c_add(u, t);
                buf[k + j + half_size] = c_sub(u, t);
            }
        }
        half_size = size;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for x in buf.iter_mut() {
            *x = c_scale(inv_n, *x);
        }
    }
}

fn fft(input: &[C64]) -> Vec<C64> {
    let mut buf = input.to_vec();
    fft_in_place(&mut buf, false);
    buf
}

fn ifft(input: &[C64]) -> Vec<C64> {
    let mut buf = input.to_vec();
    fft_in_place(&mut buf, true);
    buf
}

/// Zero-pad or truncate `data` to the next power of two >= `min_len`.
fn pad_to_pow2(data: &[C64], min_len: usize) -> Vec<C64> {
    let n = min_len.next_power_of_two();
    let mut out = vec![(0.0, 0.0); n];
    let copy_len = data.len().min(n);
    out[..copy_len].copy_from_slice(&data[..copy_len]);
    out
}

// ─── Public API ────────────────────────────────────────────────────────────

/// Radar parameters for ISAR imaging.
#[derive(Debug, Clone)]
pub struct IsarParams {
    /// Number of range bins (fast-time samples per pulse).
    pub num_range_bins: usize,
    /// Number of pulses in the coherent processing interval.
    pub num_pulses: usize,
    /// Waveform bandwidth in Hz (determines range resolution).
    pub bandwidth_hz: f64,
    /// Carrier / center frequency in Hz.
    pub center_freq_hz: f64,
    /// Pulse repetition frequency in Hz.
    pub prf_hz: f64,
    /// Speed of light in m/s (default 3e8).
    pub speed_of_light: f64,
}

impl Default for IsarParams {
    fn default() -> Self {
        Self {
            num_range_bins: 128,
            num_pulses: 64,
            bandwidth_hz: 500e6,
            center_freq_hz: 10e9,
            prf_hz: 1000.0,
            speed_of_light: 3e8,
        }
    }
}

/// ISAR image formed by range-Doppler processing.
#[derive(Debug, Clone)]
pub struct IsarImage {
    /// Complex image data, indexed `[doppler_bin][range_bin]`.
    pub data: Vec<Vec<C64>>,
    /// Number of Doppler (cross-range) bins.
    pub num_doppler_bins: usize,
    /// Number of range bins.
    pub num_range_bins: usize,
}

impl IsarImage {
    /// Extract magnitude image.
    pub fn magnitude(&self) -> Vec<Vec<f64>> {
        self.data
            .iter()
            .map(|row| row.iter().map(|&s| c_mag(s)).collect())
            .collect()
    }

    /// Extract phase image in radians.
    pub fn phase(&self) -> Vec<Vec<f64>> {
        self.data
            .iter()
            .map(|row| row.iter().map(|&s| c_phase(s)).collect())
            .collect()
    }

    /// Image contrast metric (std / mean of magnitude image).
    /// Higher contrast generally indicates better focus.
    pub fn contrast(&self) -> f64 {
        let mag = self.magnitude();
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        let mut count = 0usize;
        for row in &mag {
            for &v in row {
                sum += v;
                sum_sq += v * v;
                count += 1;
            }
        }
        if count == 0 {
            return 0.0;
        }
        let mean = sum / count as f64;
        if mean < 1e-30 {
            return 0.0;
        }
        let variance = (sum_sq / count as f64) - mean * mean;
        let std = if variance > 0.0 { variance.sqrt() } else { 0.0 };
        std / mean
    }

    /// Extract the `k` strongest scatterers as `(doppler_idx, range_idx, magnitude)`.
    pub fn dominant_scatterers(&self, k: usize) -> Vec<(usize, usize, f64)> {
        let mut all: Vec<(usize, usize, f64)> = Vec::new();
        for (di, row) in self.data.iter().enumerate() {
            for (ri, &s) in row.iter().enumerate() {
                all.push((di, ri, c_mag(s)));
            }
        }
        all.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));
        all.truncate(k);
        all
    }
}

/// Inverse Synthetic Aperture Radar imager.
///
/// Performs range compression, translational motion compensation, and
/// range-Doppler image formation on radar pulse data.
#[derive(Debug, Clone)]
pub struct InverseSyntheticApertureImager {
    params: IsarParams,
}

impl InverseSyntheticApertureImager {
    /// Create a new ISAR imager with the given parameters.
    pub fn new(params: IsarParams) -> Self {
        Self { params }
    }

    /// Access the radar parameters.
    pub fn params(&self) -> &IsarParams {
        &self.params
    }

    /// Range resolution in metres: `c / (2 * B)`.
    pub fn range_resolution(&self) -> f64 {
        self.params.speed_of_light / (2.0 * self.params.bandwidth_hz)
    }

    /// Cross-range resolution in metres given a target rotation rate (rad/s).
    ///
    /// `delta_cr = lambda / (2 * omega * T_cpi)`
    /// where `T_cpi = num_pulses / prf`.
    pub fn cross_range_resolution(&self, rotation_rate_rad_s: f64) -> f64 {
        let lambda = self.params.speed_of_light / self.params.center_freq_hz;
        let t_cpi = self.params.num_pulses as f64 / self.params.prf_hz;
        if rotation_rate_rad_s.abs() < 1e-30 || t_cpi < 1e-30 {
            return f64::INFINITY;
        }
        lambda / (2.0 * rotation_rate_rad_s.abs() * t_cpi)
    }

    /// Wavelength in metres.
    pub fn wavelength(&self) -> f64 {
        self.params.speed_of_light / self.params.center_freq_hz
    }

    /// Coherent processing interval duration in seconds.
    pub fn cpi_duration(&self) -> f64 {
        self.params.num_pulses as f64 / self.params.prf_hz
    }

    /// Range compression via matched filtering in the frequency domain.
    ///
    /// `pulse_data` is one range profile (fast-time samples).
    /// `reference` is the transmitted waveform (same length or zero-padded).
    ///
    /// Returns the compressed range profile.
    pub fn range_compress(&self, pulse_data: &[C64], reference: &[C64]) -> Vec<C64> {
        let n = pulse_data.len().max(reference.len());
        let data_p = pad_to_pow2(pulse_data, n);
        let ref_p = pad_to_pow2(reference, n);
        let data_f = fft(&data_p);
        let ref_f = fft(&ref_p);

        // Matched filter: multiply by conjugate of reference spectrum
        let product: Vec<C64> = data_f
            .iter()
            .zip(ref_f.iter())
            .map(|(&d, &r)| c_mul(d, c_conj(r)))
            .collect();

        let result = ifft(&product);
        result[..pulse_data.len().max(reference.len()).next_power_of_two()]
            .to_vec()
    }

    /// Range compress all pulses in a 2D data matrix.
    ///
    /// `data[pulse][range_sample]`, `reference` is the transmit waveform.
    pub fn range_compress_all(
        &self,
        data: &[Vec<C64>],
        reference: &[C64],
    ) -> Vec<Vec<C64>> {
        data.iter()
            .map(|pulse| self.range_compress(pulse, reference))
            .collect()
    }

    /// Translational motion compensation: envelope alignment.
    ///
    /// Aligns each pulse to the reference pulse (first pulse) by
    /// finding the peak of the cross-correlation and circularly shifting.
    ///
    /// `data[pulse][range_bin]` — range-compressed data.
    pub fn envelope_alignment(&self, data: &[Vec<C64>]) -> Vec<Vec<C64>> {
        if data.is_empty() {
            return vec![];
        }
        let ref_pulse = &data[0];
        let n = ref_pulse.len();

        data.iter()
            .map(|pulse| {
                // Cross-correlation via FFT
                let n_fft = (n * 2).next_power_of_two();
                let ref_padded = pad_to_pow2(ref_pulse, n_fft);
                let pulse_padded = pad_to_pow2(pulse, n_fft);

                let ref_f = fft(&ref_padded);
                let pulse_f = fft(&pulse_padded);

                let xcorr_f: Vec<C64> = pulse_f
                    .iter()
                    .zip(ref_f.iter())
                    .map(|(&p, &r)| c_mul(p, c_conj(r)))
                    .collect();

                let xcorr = ifft(&xcorr_f);

                // Find peak
                let (peak_idx, _) = xcorr
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| {
                        c_mag_sq(**a)
                            .partial_cmp(&c_mag_sq(**b))
                            .unwrap_or(std::cmp::Ordering::Equal)
                    })
                    .unwrap();

                // Determine shift: if peak > n_fft/2, shift is negative
                let shift = if peak_idx <= n_fft / 2 {
                    peak_idx
                } else {
                    peak_idx // wraps around via modular indexing below
                };

                // Circular shift original pulse
                let mut aligned = vec![(0.0, 0.0); n];
                for i in 0..n {
                    let src = (i + shift) % n;
                    aligned[i] = pulse[src];
                }
                aligned
            })
            .collect()
    }

    /// Phase correction for translational motion compensation.
    ///
    /// Estimates and removes the residual phase from each pulse
    /// relative to the first pulse, using the dominant range bin.
    ///
    /// `data[pulse][range_bin]` — envelope-aligned data.
    pub fn phase_correction(&self, data: &[Vec<C64>]) -> Vec<Vec<C64>> {
        if data.is_empty() || data[0].is_empty() {
            return data.to_vec();
        }

        // Find dominant range bin (highest total energy)
        let num_range = data[0].len();
        let mut energy_per_bin = vec![0.0; num_range];
        for pulse in data {
            for (i, &s) in pulse.iter().enumerate() {
                energy_per_bin[i] += c_mag_sq(s);
            }
        }
        let dominant_bin = energy_per_bin
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Reference phase from first pulse
        let ref_phase = c_phase(data[0][dominant_bin]);

        data.iter()
            .map(|pulse| {
                let pulse_phase = c_phase(pulse[dominant_bin]);
                let correction = ref_phase - pulse_phase;
                let corr_phasor = c_exp(correction);
                pulse.iter().map(|&s| c_mul(s, corr_phasor)).collect()
            })
            .collect()
    }

    /// Full translational motion compensation (envelope alignment + phase correction).
    pub fn motion_compensate(&self, data: &[Vec<C64>]) -> Vec<Vec<C64>> {
        let aligned = self.envelope_alignment(data);
        self.phase_correction(&aligned)
    }

    /// Form a range-Doppler image using 2D FFT.
    ///
    /// `data[pulse][range_bin]` — motion-compensated, range-compressed data.
    ///
    /// Returns the complex image indexed `[doppler_bin][range_bin]`.
    pub fn range_doppler_image(&self, data: &[Vec<C64>]) -> Vec<Vec<C64>> {
        if data.is_empty() {
            return vec![];
        }
        let num_pulses = data.len();
        let num_range = data[0].len();

        let range_fft_size = num_range.next_power_of_two();
        let doppler_fft_size = num_pulses.next_power_of_two();

        // Range FFT on each pulse
        let range_profiles: Vec<Vec<C64>> = data
            .iter()
            .map(|pulse| {
                let padded = pad_to_pow2(pulse, range_fft_size);
                fft(&padded)
            })
            .collect();

        // FFT along Doppler (slow-time) for each range bin
        let mut image = vec![vec![(0.0, 0.0); range_fft_size]; doppler_fft_size];

        for ri in 0..range_fft_size {
            // Extract slow-time column for this range bin
            let mut column = vec![(0.0, 0.0); doppler_fft_size];
            for (pi, profile) in range_profiles.iter().enumerate() {
                column[pi] = profile[ri];
            }

            // Apply Hamming window before Doppler FFT
            let n = doppler_fft_size;
            for i in 0..n {
                let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1).max(1) as f64).cos();
                column[i] = c_scale(w, column[i]);
            }

            fft_in_place(&mut column, false);

            // Place into image
            for di in 0..doppler_fft_size {
                image[di][ri] = column[di];
            }
        }

        image
    }

    /// Form an ISAR image and return it as an `IsarImage` struct.
    pub fn form_image(&self, data: &[Vec<C64>]) -> IsarImage {
        let image_data = self.range_doppler_image(data);
        let num_doppler = image_data.len();
        let num_range = if num_doppler > 0 {
            image_data[0].len()
        } else {
            0
        };
        IsarImage {
            data: image_data,
            num_doppler_bins: num_doppler,
            num_range_bins: num_range,
        }
    }

    /// Phase Gradient Autofocus (PGA) — simplified algorithm.
    ///
    /// Iteratively estimates and corrects phase errors across pulses
    /// by examining the phase gradient of dominant scatterers.
    ///
    /// `data[pulse][range_bin]` — input data (range-compressed).
    /// `iterations` — number of PGA iterations.
    ///
    /// Returns phase-corrected data.
    pub fn phase_gradient_autofocus(
        &self,
        data: &[Vec<C64>],
        iterations: usize,
    ) -> Vec<Vec<C64>> {
        if data.is_empty() || data[0].is_empty() {
            return data.to_vec();
        }

        let num_pulses = data.len();
        let num_range = data[0].len();
        let mut corrected: Vec<Vec<C64>> = data.to_vec();

        for _iter in 0..iterations {
            // Step 1: Form image and find dominant range bins
            let image = self.range_doppler_image(&corrected);
            let isar_img = IsarImage {
                data: image.clone(),
                num_doppler_bins: image.len(),
                num_range_bins: if image.is_empty() { 0 } else { image[0].len() },
            };
            let scatterers = isar_img.dominant_scatterers(num_range.min(5).max(1));

            if scatterers.is_empty() {
                break;
            }

            // Step 2: Use the strongest range bin for phase estimation
            let strongest_range = scatterers[0].1;

            // Step 3: Extract slow-time phase history at strongest range bin
            let mut phase_errors = vec![0.0; num_pulses];
            let ref_phase = c_phase(corrected[0][strongest_range]);
            for pi in 0..num_pulses {
                let p = c_phase(corrected[pi][strongest_range]);
                phase_errors[pi] = p - ref_phase;
            }

            // Step 4: Compute phase gradient (difference of successive phases)
            // and integrate to get smooth phase error estimate
            let mut gradient = vec![0.0; num_pulses];
            for i in 1..num_pulses {
                let mut diff = phase_errors[i] - phase_errors[i - 1];
                // Wrap to [-pi, pi]
                while diff > PI {
                    diff -= 2.0 * PI;
                }
                while diff < -PI {
                    diff += 2.0 * PI;
                }
                gradient[i] = diff;
            }

            // Integrate gradient to get smoothed phase errors
            let mut integrated = vec![0.0; num_pulses];
            for i in 1..num_pulses {
                integrated[i] = integrated[i - 1] + gradient[i];
            }

            // Step 5: Apply phase correction
            for pi in 0..num_pulses {
                let correction = c_exp(-integrated[pi]);
                for ri in 0..corrected[pi].len() {
                    corrected[pi][ri] = c_mul(corrected[pi][ri], correction);
                }
            }
        }

        corrected
    }

    /// Image contrast-based autofocus.
    ///
    /// Searches over a range of constant phase-rate corrections to maximize
    /// image contrast. Returns the corrected data and the optimal phase rate.
    pub fn contrast_autofocus(
        &self,
        data: &[Vec<C64>],
        search_steps: usize,
        max_phase_rate: f64,
    ) -> (Vec<Vec<C64>>, f64) {
        if data.is_empty() {
            return (data.to_vec(), 0.0);
        }

        let mut best_contrast = f64::NEG_INFINITY;
        let mut best_rate = 0.0;

        let step = if search_steps > 1 {
            2.0 * max_phase_rate / (search_steps - 1) as f64
        } else {
            0.0
        };

        for si in 0..search_steps {
            let rate = -max_phase_rate + step * si as f64;

            // Apply trial phase correction
            let trial: Vec<Vec<C64>> = data
                .iter()
                .enumerate()
                .map(|(pi, pulse)| {
                    let correction = c_exp(-rate * pi as f64);
                    pulse.iter().map(|&s| c_mul(s, correction)).collect()
                })
                .collect();

            let image = self.form_image(&trial);
            let contrast = image.contrast();

            if contrast > best_contrast {
                best_contrast = contrast;
                best_rate = rate;
            }
        }

        // Apply best correction
        let result: Vec<Vec<C64>> = data
            .iter()
            .enumerate()
            .map(|(pi, pulse)| {
                let correction = c_exp(-best_rate * pi as f64);
                pulse.iter().map(|&s| c_mul(s, correction)).collect()
            })
            .collect();

        (result, best_rate)
    }

    /// Generate a Linear Frequency Modulated (LFM) chirp reference waveform.
    ///
    /// Returns `num_samples` complex samples of an LFM chirp spanning
    /// the configured bandwidth.
    pub fn generate_lfm_reference(&self, num_samples: usize) -> Vec<C64> {
        let t_pulse = num_samples as f64 / (2.0 * self.params.bandwidth_hz);
        let chirp_rate = self.params.bandwidth_hz / t_pulse;

        (0..num_samples)
            .map(|i| {
                let t = (i as f64 / num_samples as f64 - 0.5) * t_pulse;
                let phase = PI * chirp_rate * t * t;
                c_exp(phase)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_imager() -> InverseSyntheticApertureImager {
        InverseSyntheticApertureImager::new(IsarParams {
            num_range_bins: 64,
            num_pulses: 32,
            bandwidth_hz: 500e6,
            center_freq_hz: 10e9,
            prf_hz: 1000.0,
            speed_of_light: 3e8,
        })
    }

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── Test 1: IsarParams default ────────────────────────────────────

    #[test]
    fn test_params_default() {
        let p = IsarParams::default();
        assert_eq!(p.num_range_bins, 128);
        assert_eq!(p.num_pulses, 64);
        assert!(approx_eq(p.bandwidth_hz, 500e6, 1.0));
        assert!(approx_eq(p.center_freq_hz, 10e9, 1.0));
        assert!(approx_eq(p.prf_hz, 1000.0, 0.01));
    }

    // ── Test 2: Range resolution ──────────────────────────────────────

    #[test]
    fn test_range_resolution() {
        let imager = default_imager();
        // c / (2*B) = 3e8 / (2 * 500e6) = 0.3 m
        let r = imager.range_resolution();
        assert!(approx_eq(r, 0.3, 1e-10));
    }

    // ── Test 3: Cross-range resolution ────────────────────────────────

    #[test]
    fn test_cross_range_resolution() {
        let imager = default_imager();
        // lambda = 3e8/10e9 = 0.03 m, T_cpi = 32/1000 = 0.032 s
        // delta_cr = 0.03 / (2 * 0.01 * 0.032) = 0.03 / 0.00064 = 46.875 m
        let cr = imager.cross_range_resolution(0.01);
        assert!(approx_eq(cr, 46.875, 1e-6));
    }

    // ── Test 4: Cross-range resolution zero rotation ──────────────────

    #[test]
    fn test_cross_range_resolution_zero_rotation() {
        let imager = default_imager();
        assert_eq!(imager.cross_range_resolution(0.0), f64::INFINITY);
    }

    // ── Test 5: Wavelength ────────────────────────────────────────────

    #[test]
    fn test_wavelength() {
        let imager = default_imager();
        assert!(approx_eq(imager.wavelength(), 0.03, 1e-10));
    }

    // ── Test 6: CPI duration ──────────────────────────────────────────

    #[test]
    fn test_cpi_duration() {
        let imager = default_imager();
        assert!(approx_eq(imager.cpi_duration(), 0.032, 1e-10));
    }

    // ── Test 7: FFT of delta function ─────────────────────────────────

    #[test]
    fn test_fft_delta() {
        let mut input = vec![(0.0, 0.0); 8];
        input[0] = (1.0, 0.0);
        let output = fft(&input);
        // FFT of delta should be all ones
        for s in &output {
            assert!(approx_eq(s.0, 1.0, 1e-12));
            assert!(approx_eq(s.1, 0.0, 1e-12));
        }
    }

    // ── Test 8: FFT/IFFT round-trip ───────────────────────────────────

    #[test]
    fn test_fft_ifft_roundtrip() {
        let input: Vec<C64> = (0..16).map(|i| (i as f64, -(i as f64) * 0.5)).collect();
        let output = ifft(&fft(&input));
        for (a, b) in input.iter().zip(output.iter()) {
            assert!(approx_eq(a.0, b.0, 1e-10));
            assert!(approx_eq(a.1, b.1, 1e-10));
        }
    }

    // ── Test 9: Range compression produces a peak ─────────────────────

    #[test]
    fn test_range_compression_peak() {
        let imager = default_imager();
        let n = 64;
        // Simple LFM chirp as reference
        let reference: Vec<C64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                c_exp(PI * 100.0 * t * t)
            })
            .collect();

        // Delayed echo = shifted version of reference
        let mut echo = vec![(0.0, 0.0); n];
        let delay = 10;
        for i in 0..(n - delay) {
            echo[i + delay] = reference[i];
        }

        let compressed = imager.range_compress(&echo, &reference);
        // Find peak
        let (peak_idx, _) = compressed
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_mag_sq(**a).partial_cmp(&c_mag_sq(**b)).unwrap())
            .unwrap();

        // Peak should be near the delay
        assert!(compressed.len() >= n);
        // Just verify a clear peak exists (peak magnitude > mean magnitude)
        let peak_mag = c_mag(compressed[peak_idx]);
        let mean_mag: f64 =
            compressed.iter().map(|s| c_mag(*s)).sum::<f64>() / compressed.len() as f64;
        assert!(peak_mag > mean_mag * 2.0, "No clear peak after range compression");
    }

    // ── Test 10: Range compress all pulses ────────────────────────────

    #[test]
    fn test_range_compress_all() {
        let imager = default_imager();
        let n = 32;
        let reference: Vec<C64> = (0..n).map(|i| c_exp(PI * 50.0 * (i as f64 / n as f64).powi(2))).collect();
        let data = vec![reference.clone(); 4];
        let compressed = imager.range_compress_all(&data, &reference);
        assert_eq!(compressed.len(), 4);
        // Each compressed pulse should have a peak at bin 0 (autocorrelation)
        for pulse in &compressed {
            let peak_mag = c_mag(pulse[0]);
            let other_max: f64 = pulse[1..]
                .iter()
                .map(|s| c_mag(*s))
                .fold(0.0, f64::max);
            assert!(peak_mag >= other_max, "Autocorrelation peak should be at bin 0");
        }
    }

    // ── Test 11: Envelope alignment on shifted data ───────────────────

    #[test]
    fn test_envelope_alignment() {
        let imager = default_imager();
        let n = 32;
        // Reference pulse: single strong scatterer at bin 5
        let mut ref_pulse = vec![(0.0, 0.0); n];
        ref_pulse[5] = (10.0, 0.0);

        // Second pulse: same scatterer shifted to bin 7
        let mut shifted = vec![(0.0, 0.0); n];
        shifted[7] = (10.0, 0.0);

        let data = vec![ref_pulse.clone(), shifted];
        let aligned = imager.envelope_alignment(&data);

        // After alignment, the peak in pulse 1 should be near bin 5
        let (peak_idx, _) = aligned[1]
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_mag_sq(**a).partial_cmp(&c_mag_sq(**b)).unwrap())
            .unwrap();

        // Allow some tolerance due to circular correlation
        let distance = ((peak_idx as i64 - 5i64).abs()).min(
            n as i64 - (peak_idx as i64 - 5i64).abs()
        );
        assert!(distance <= 1, "Envelope alignment failed: peak at {} expected near 5", peak_idx);
    }

    // ── Test 12: Phase correction removes linear phase drift ──────────

    #[test]
    fn test_phase_correction() {
        let imager = default_imager();
        let n = 16;
        let num_pulses = 8;

        // Create data with a strong scatterer and linear phase drift
        let mut data = Vec::new();
        for pi in 0..num_pulses {
            let mut pulse = vec![(0.1, 0.0); n];
            let phase_drift = 0.3 * pi as f64;
            pulse[4] = c_mul((10.0, 0.0), c_exp(phase_drift));
            data.push(pulse);
        }

        let corrected = imager.phase_correction(&data);

        // After correction, the phase at bin 4 should be approximately constant
        let ref_phase = c_phase(corrected[0][4]);
        for pi in 1..num_pulses {
            let p = c_phase(corrected[pi][4]);
            let diff = (p - ref_phase).abs();
            let wrapped = if diff > PI { 2.0 * PI - diff } else { diff };
            assert!(wrapped < 0.1, "Phase correction failed: pulse {} phase diff = {}", pi, wrapped);
        }
    }

    // ── Test 13: Range-Doppler image dimensions ───────────────────────

    #[test]
    fn test_range_doppler_image_dimensions() {
        let imager = default_imager();
        let data: Vec<Vec<C64>> = vec![vec![(1.0, 0.0); 64]; 32];
        let image = imager.range_doppler_image(&data);
        // Doppler dimension = next_power_of_two(32) = 32
        assert_eq!(image.len(), 32);
        // Range dimension = next_power_of_two(64) = 64
        assert_eq!(image[0].len(), 64);
    }

    // ── Test 14: Range-Doppler image empty input ──────────────────────

    #[test]
    fn test_range_doppler_image_empty() {
        let imager = default_imager();
        let image = imager.range_doppler_image(&[]);
        assert!(image.is_empty());
    }

    // ── Test 15: Form image struct ────────────────────────────────────

    #[test]
    fn test_form_image() {
        let imager = default_imager();
        let data: Vec<Vec<C64>> = vec![vec![(1.0, 0.0); 16]; 8];
        let img = imager.form_image(&data);
        assert_eq!(img.num_doppler_bins, 8);
        assert_eq!(img.num_range_bins, 16);
        assert_eq!(img.data.len(), 8);
        assert_eq!(img.data[0].len(), 16);
    }

    // ── Test 16: Image magnitude and phase extraction ─────────────────

    #[test]
    fn test_image_magnitude_and_phase() {
        let img = IsarImage {
            data: vec![vec![(3.0, 4.0), (0.0, 1.0)], vec![(-1.0, 0.0), (1.0, 1.0)]],
            num_doppler_bins: 2,
            num_range_bins: 2,
        };

        let mag = img.magnitude();
        assert!(approx_eq(mag[0][0], 5.0, 1e-10)); // |3+4i| = 5
        assert!(approx_eq(mag[0][1], 1.0, 1e-10)); // |0+1i| = 1
        assert!(approx_eq(mag[1][0], 1.0, 1e-10)); // |-1+0i| = 1

        let phase = img.phase();
        assert!(approx_eq(phase[0][1], PI / 2.0, 1e-10)); // arg(0+1i) = pi/2
        assert!(approx_eq(phase[1][0], PI, 1e-10)); // arg(-1+0i) = pi
    }

    // ── Test 17: Image contrast metric ────────────────────────────────

    #[test]
    fn test_image_contrast() {
        // Uniform image should have zero contrast (std=0)
        let uniform = IsarImage {
            data: vec![vec![(1.0, 0.0); 4]; 4],
            num_doppler_bins: 4,
            num_range_bins: 4,
        };
        assert!(approx_eq(uniform.contrast(), 0.0, 1e-10));

        // Image with one bright pixel should have higher contrast
        let mut focused_data = vec![vec![(0.0, 0.0); 4]; 4];
        focused_data[1][2] = (100.0, 0.0);
        let focused = IsarImage {
            data: focused_data,
            num_doppler_bins: 4,
            num_range_bins: 4,
        };
        assert!(focused.contrast() > 1.0);
    }

    // ── Test 18: Dominant scatterers extraction ───────────────────────

    #[test]
    fn test_dominant_scatterers() {
        let mut data = vec![vec![(0.0, 0.0); 8]; 8];
        data[2][3] = (10.0, 0.0);
        data[5][7] = (0.0, 8.0);
        data[1][1] = (3.0, 4.0); // mag = 5

        let img = IsarImage {
            data,
            num_doppler_bins: 8,
            num_range_bins: 8,
        };

        let top3 = img.dominant_scatterers(3);
        assert_eq!(top3.len(), 3);

        // Strongest: (2,3) with mag 10
        assert_eq!(top3[0].0, 2);
        assert_eq!(top3[0].1, 3);
        assert!(approx_eq(top3[0].2, 10.0, 1e-10));

        // Second: (5,7) with mag 8
        assert_eq!(top3[1].0, 5);
        assert_eq!(top3[1].1, 7);
        assert!(approx_eq(top3[1].2, 8.0, 1e-10));

        // Third: (1,1) with mag 5
        assert_eq!(top3[2].0, 1);
        assert_eq!(top3[2].1, 1);
        assert!(approx_eq(top3[2].2, 5.0, 1e-10));
    }

    // ── Test 19: Phase gradient autofocus does not crash ──────────────

    #[test]
    fn test_phase_gradient_autofocus() {
        let imager = default_imager();
        let n = 32;
        let num_pulses = 16;

        // Create data with a scatterer and quadratic phase error
        let mut data = Vec::new();
        for pi in 0..num_pulses {
            let mut pulse = vec![(0.01, 0.0); n];
            let phase_err = 0.05 * (pi as f64).powi(2);
            pulse[8] = c_mul((5.0, 0.0), c_exp(phase_err));
            data.push(pulse);
        }

        let corrected = imager.phase_gradient_autofocus(&data, 3);
        assert_eq!(corrected.len(), num_pulses);
        assert_eq!(corrected[0].len(), n);

        // The corrected image should have better or equal contrast
        let img_before = imager.form_image(&data);
        let img_after = imager.form_image(&corrected);
        // PGA should not degrade contrast significantly
        assert!(
            img_after.contrast() >= img_before.contrast() * 0.5,
            "PGA degraded contrast excessively"
        );
    }

    // ── Test 20: Contrast autofocus finds optimal rate ────────────────

    #[test]
    fn test_contrast_autofocus() {
        let imager = default_imager();
        let n = 32;
        let num_pulses = 16;

        // Create focused data with a single scatterer
        let mut data = Vec::new();
        for _pi in 0..num_pulses {
            let mut pulse = vec![(0.01, 0.0); n];
            pulse[10] = (5.0, 0.0);
            data.push(pulse);
        }

        let (corrected, rate) = imager.contrast_autofocus(&data, 21, 1.0);
        assert_eq!(corrected.len(), num_pulses);
        // For already-focused data, optimal rate should be near zero
        assert!(rate.abs() < 0.5, "Contrast autofocus rate should be near 0 for focused data, got {}", rate);
    }

    // ── Test 21: LFM reference generation ─────────────────────────────

    #[test]
    fn test_generate_lfm_reference() {
        let imager = default_imager();
        let ref_wfm = imager.generate_lfm_reference(64);
        assert_eq!(ref_wfm.len(), 64);
        // All samples should have unit magnitude (pure phase modulation)
        for s in &ref_wfm {
            assert!(approx_eq(c_mag(*s), 1.0, 1e-12));
        }
    }

    // ── Test 22: Motion compensate full pipeline ──────────────────────

    #[test]
    fn test_motion_compensate() {
        let imager = default_imager();
        let n = 32;
        let num_pulses = 8;

        let mut data = Vec::new();
        for pi in 0..num_pulses {
            let mut pulse = vec![(0.1, 0.0); n];
            // Scatterer at bin 10 with phase drift
            pulse[10] = c_mul((10.0, 0.0), c_exp(0.2 * pi as f64));
            data.push(pulse);
        }

        let compensated = imager.motion_compensate(&data);
        assert_eq!(compensated.len(), num_pulses);
        assert_eq!(compensated[0].len(), n);

        // After compensation, phase at bin 10 should be approximately constant
        let ref_phase = c_phase(compensated[0][10]);
        for pi in 1..num_pulses {
            let p = c_phase(compensated[pi][10]);
            let diff = (p - ref_phase).abs();
            let wrapped = if diff > PI { 2.0 * PI - diff } else { diff };
            assert!(wrapped < 0.2, "Motion compensation left residual phase drift");
        }
    }

    // ── Test 23: Complex arithmetic helpers ───────────────────────────

    #[test]
    fn test_complex_helpers() {
        let a: C64 = (3.0, 4.0);
        let b: C64 = (1.0, -2.0);

        assert_eq!(c_add(a, b), (4.0, 2.0));
        assert_eq!(c_sub(a, b), (2.0, 6.0));
        // (3+4i)(1-2i) = 3-6i+4i-8i^2 = 3-6i+4i+8 = 11-2i
        assert_eq!(c_mul(a, b), (11.0, -2.0));
        assert_eq!(c_conj(a), (3.0, -4.0));
        assert!(approx_eq(c_mag(a), 5.0, 1e-12));
        assert!(approx_eq(c_mag_sq(a), 25.0, 1e-12));
        assert!(approx_eq(c_phase((0.0, 1.0)), PI / 2.0, 1e-12));

        let e = c_exp(PI / 4.0);
        assert!(approx_eq(e.0, (PI / 4.0).cos(), 1e-12));
        assert!(approx_eq(e.1, (PI / 4.0).sin(), 1e-12));

        assert_eq!(c_scale(2.0, (3.0, 4.0)), (6.0, 8.0));
    }

    // ── Test 24: Contrast of empty image ──────────────────────────────

    #[test]
    fn test_contrast_empty() {
        let img = IsarImage {
            data: vec![],
            num_doppler_bins: 0,
            num_range_bins: 0,
        };
        assert!(approx_eq(img.contrast(), 0.0, 1e-10));
    }
}
