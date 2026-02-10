//! Range Migration Correction (RMC) — Doppler-Dependent Range Cell Migration
//!
//! Implements correction for range cell migration (RCM) that occurs when a
//! target's range changes appreciably during a coherent processing interval
//! (CPI). In stepped-frequency radar and synthetic aperture radar (SAR), a
//! moving target's echo migrates across range cells as a function of Doppler,
//! smearing the range-Doppler response. This module provides several
//! correction techniques:
//!
//! - **Range migration trajectory computation** for a given target velocity
//! - **Keystone transform** for correcting linear (first-order) RCM
//! - **Chirp Z-transform interpolation** for non-uniform resampling
//! - **Doppler-range coupling compensation**
//! - **Range compression** via matched filtering in fast-time
//! - **Migration curve estimation** from a range-Doppler map
//! - **Integration gain computation** before and after correction
//!
//! ## Algorithm Overview
//!
//! Range cell migration arises because a target at range R(t) = R₀ + v·t
//! experiences a round-trip delay τ(t) = 2R(t)/c that shifts in fast-time
//! across slow-time pulses. The keystone transform rescales the slow-time
//! axis so that the migration becomes range-independent, converting the
//! curved trajectory into a straight line in the range-Doppler domain.
//!
//! For higher-order migration (acceleration), chirp Z-transform based
//! warping provides the required non-uniform resampling.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::range_migration_correction::{RangeMigrationCorrector, RadarParams};
//!
//! let params = RadarParams {
//!     carrier_freq_hz: 10.0e9,
//!     bandwidth_hz: 50.0e6,
//!     prf_hz: 1000.0,
//!     num_pulses: 64,
//!     num_range_bins: 128,
//!     sample_rate_hz: 100.0e6,
//! };
//!
//! let corrector = RangeMigrationCorrector::new(params);
//!
//! // Compute migration trajectory for a target moving at 300 m/s
//! let trajectory = corrector.migration_trajectory(300.0);
//! assert_eq!(trajectory.len(), 64);
//!
//! // Each entry is the fractional range-bin shift for that pulse
//! assert!((trajectory[0]).abs() < 1e-12); // First pulse has zero shift
//! ```

use std::f64::consts::PI;

// ── Complex arithmetic helpers using (f64, f64) tuples ────────────────────

/// Complex addition.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate.
#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex magnitude squared.
#[inline]
fn c_mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex magnitude.
#[inline]
fn c_mag(a: (f64, f64)) -> f64 {
    c_mag_sq(a).sqrt()
}

/// Complex exponential: e^{j·theta}.
#[inline]
fn c_exp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// Scale a complex number by a real scalar.
#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Complex division: a / b.
#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = c_mag_sq(b);
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

// ── FFT (radix-2 Cooley-Tukey) ───────────────────────────────────────────

fn next_pow2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// In-place radix-2 DIT FFT. `inverse` selects IFFT (with 1/N scaling).
fn fft_inplace(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT size must be power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let w_base = c_exp_j(angle);
        let mut i = 0;
        while i < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = buf[i + k];
                let t = c_mul(w, buf[i + k + half]);
                buf[i + k] = c_add(u, t);
                buf[i + k + half] = c_sub(u, t);
                w = c_mul(w, w_base);
            }
            i += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for sample in buf.iter_mut() {
            *sample = c_scale(*sample, inv_n);
        }
    }
}

/// Forward FFT returning a new vector (zero-padded to power of 2).
fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = next_pow2(input.len());
    let mut buf = vec![(0.0, 0.0); n];
    buf[..input.len()].copy_from_slice(input);
    fft_inplace(&mut buf, false);
    buf
}

/// Inverse FFT returning a new vector.
#[allow(dead_code)]
fn ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = next_pow2(input.len());
    let mut buf = vec![(0.0, 0.0); n];
    buf[..input.len()].copy_from_slice(input);
    fft_inplace(&mut buf, true);
    buf
}

// ── Speed of light ───────────────────────────────────────────────────────

const C_LIGHT: f64 = 299_792_458.0;

// ── Public types ─────────────────────────────────────────────────────────

/// Radar system parameters for range migration correction.
#[derive(Debug, Clone)]
pub struct RadarParams {
    /// Carrier (center) frequency in Hz.
    pub carrier_freq_hz: f64,
    /// Transmitted bandwidth in Hz (for range resolution).
    pub bandwidth_hz: f64,
    /// Pulse repetition frequency in Hz.
    pub prf_hz: f64,
    /// Number of pulses in the coherent processing interval.
    pub num_pulses: usize,
    /// Number of range bins (fast-time samples per pulse).
    pub num_range_bins: usize,
    /// ADC sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl RadarParams {
    /// Wavelength in metres: λ = c / f_c.
    pub fn wavelength(&self) -> f64 {
        C_LIGHT / self.carrier_freq_hz
    }

    /// Range resolution: δR = c / (2·B).
    pub fn range_resolution(&self) -> f64 {
        C_LIGHT / (2.0 * self.bandwidth_hz)
    }

    /// Unambiguous Doppler: ±PRF/2.
    pub fn max_unambiguous_doppler(&self) -> f64 {
        self.prf_hz / 2.0
    }

    /// Coherent processing interval duration in seconds.
    pub fn cpi_duration(&self) -> f64 {
        self.num_pulses as f64 / self.prf_hz
    }

    /// Maximum unambiguous velocity: v_max = λ·PRF/4.
    pub fn max_unambiguous_velocity(&self) -> f64 {
        self.wavelength() * self.prf_hz / 4.0
    }
}

/// Result of migration curve estimation.
#[derive(Debug, Clone)]
pub struct MigrationCurve {
    /// Estimated range-bin shift per pulse index.
    pub shifts: Vec<f64>,
    /// Estimated radial velocity in m/s.
    pub estimated_velocity: f64,
    /// Goodness-of-fit (0.0–1.0); higher is better.
    pub fit_quality: f64,
}

/// Integration gain measurement.
#[derive(Debug, Clone)]
pub struct IntegrationGain {
    /// Peak signal power before correction.
    pub peak_before: f64,
    /// Peak signal power after correction.
    pub peak_after: f64,
    /// Gain in dB: 10·log10(peak_after / peak_before).
    pub gain_db: f64,
}

/// The main corrector struct that performs range cell migration correction.
#[derive(Debug, Clone)]
pub struct RangeMigrationCorrector {
    params: RadarParams,
}

impl RangeMigrationCorrector {
    /// Create a new corrector with the given radar parameters.
    pub fn new(params: RadarParams) -> Self {
        Self { params }
    }

    /// Return a reference to the radar parameters.
    pub fn params(&self) -> &RadarParams {
        &self.params
    }

    // ── Migration trajectory ──────────────────────────────────────────

    /// Compute the range-bin migration trajectory for a target with the
    /// given radial velocity (m/s).
    ///
    /// Returns a vector of length `num_pulses`, where each element is the
    /// fractional range-bin shift Δr(m) at slow-time index m relative to
    /// the first pulse.
    ///
    /// The shift in samples is: Δn(m) = 2·v·(m/PRF) / c × fs
    pub fn migration_trajectory(&self, velocity_mps: f64) -> Vec<f64> {
        let pri = 1.0 / self.params.prf_hz;
        // Range change per PRI in samples: 2·v·PRI / c · fs
        let shift_per_pulse = 2.0 * velocity_mps * pri / C_LIGHT * self.params.sample_rate_hz;
        (0..self.params.num_pulses)
            .map(|m| m as f64 * shift_per_pulse)
            .collect()
    }

    /// Total migration in range bins across the full CPI for a given velocity.
    pub fn total_migration_bins(&self, velocity_mps: f64) -> f64 {
        let traj = self.migration_trajectory(velocity_mps);
        traj.last().copied().unwrap_or(0.0)
    }

    // ── Keystone transform ────────────────────────────────────────────

    /// Apply the keystone transform to a 2D data matrix (slow-time × fast-time)
    /// stored in row-major order.
    ///
    /// The keystone transform rescales the slow-time axis on a per-range-frequency
    /// basis so that linear RCM is removed. It works by:
    /// 1. FFT along fast-time (range) for each pulse → range-frequency domain
    /// 2. For each range-frequency bin f_r, resample slow-time by factor
    ///    (f_c + f_r) / f_c using sinc interpolation
    /// 3. IFFT along fast-time to return to range-time domain
    ///
    /// `data` is `num_pulses` rows × `num_range_bins` columns (row-major).
    /// Returns a corrected matrix of the same dimensions.
    pub fn keystone_transform(&self, data: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let np = self.params.num_pulses;
        let nr = self.params.num_range_bins;
        assert_eq!(
            data.len(),
            np * nr,
            "data length must be num_pulses × num_range_bins"
        );

        let nr_fft = next_pow2(nr);

        // Step 1: FFT along fast-time for each pulse
        let mut freq_domain = vec![(0.0, 0.0); np * nr_fft];
        for pulse in 0..np {
            let mut row = vec![(0.0, 0.0); nr_fft];
            for bin in 0..nr {
                row[bin] = data[pulse * nr + bin];
            }
            fft_inplace(&mut row, false);
            for bin in 0..nr_fft {
                freq_domain[pulse * nr_fft + bin] = row[bin];
            }
        }

        // Step 2: For each range-frequency bin, resample slow-time
        let fc = self.params.carrier_freq_hz;
        let df = self.params.sample_rate_hz / nr_fft as f64;

        let mut corrected_freq = vec![(0.0, 0.0); np * nr_fft];
        for bin in 0..nr_fft {
            // Range frequency for this bin (centered)
            let f_r = if bin <= nr_fft / 2 {
                bin as f64 * df
            } else {
                (bin as f64 - nr_fft as f64) * df
            };

            // Keystone rescaling factor
            let scale = (fc + f_r) / fc;

            // Extract slow-time column
            let column: Vec<(f64, f64)> =
                (0..np).map(|p| freq_domain[p * nr_fft + bin]).collect();

            // Resample: for output index m, interpolate at input position m * scale
            for m in 0..np {
                let t = m as f64 * scale;
                corrected_freq[m * nr_fft + bin] = sinc_interp(&column, t);
            }
        }

        // Step 3: IFFT along fast-time for each pulse
        let mut result = vec![(0.0, 0.0); np * nr];
        for pulse in 0..np {
            let mut row: Vec<(f64, f64)> = (0..nr_fft)
                .map(|bin| corrected_freq[pulse * nr_fft + bin])
                .collect();
            fft_inplace(&mut row, true);
            for bin in 0..nr {
                result[pulse * nr + bin] = row[bin];
            }
        }

        result
    }

    // ── Chirp Z-transform interpolation ──────────────────────────────

    /// Perform chirp Z-transform based interpolation for non-uniform resampling
    /// of a 1D signal.
    ///
    /// Given `input` samples, evaluate the signal at `output_len` points starting
    /// at normalized frequency `f_start` with spacing `f_step` (both in cycles/sample).
    ///
    /// This uses Bluestein's algorithm: CZT evaluates X(z_k) for
    /// z_k = A · W^{-k}, k = 0..M-1.
    pub fn czt_interpolate(
        &self,
        input: &[(f64, f64)],
        output_len: usize,
        f_start: f64,
        f_step: f64,
    ) -> Vec<(f64, f64)> {
        czt_eval(input, output_len, f_start, f_step)
    }

    // ── Doppler-range coupling compensation ──────────────────────────

    /// Compensate Doppler-range coupling by applying a phase correction to each
    /// pulse in the range-frequency domain.
    ///
    /// In linear FM (chirp) waveforms, a Doppler shift f_d causes an apparent
    /// range shift of f_d / chirp_rate. This method removes that coupling.
    ///
    /// `data`: num_pulses × num_range_bins, row-major.
    /// `doppler_freqs`: Doppler frequency (Hz) estimate for each pulse.
    /// `chirp_rate`: Chirp rate in Hz/s (bandwidth / pulse_duration).
    pub fn compensate_doppler_range_coupling(
        &self,
        data: &[(f64, f64)],
        doppler_freqs: &[f64],
        chirp_rate: f64,
    ) -> Vec<(f64, f64)> {
        let np = self.params.num_pulses;
        let nr = self.params.num_range_bins;
        assert_eq!(data.len(), np * nr);
        assert_eq!(doppler_freqs.len(), np);

        let nr_fft = next_pow2(nr);
        let mut result = vec![(0.0, 0.0); np * nr];

        for pulse in 0..np {
            // FFT along fast-time
            let mut row = vec![(0.0, 0.0); nr_fft];
            for bin in 0..nr {
                row[bin] = data[pulse * nr + bin];
            }
            fft_inplace(&mut row, false);

            // Apply phase correction in range-frequency domain
            let f_d = doppler_freqs[pulse];
            let df = self.params.sample_rate_hz / nr_fft as f64;
            for bin in 0..nr_fft {
                let f_r = if bin <= nr_fft / 2 {
                    bin as f64 * df
                } else {
                    (bin as f64 - nr_fft as f64) * df
                };
                // Coupling shift: Δτ = f_d / chirp_rate → phase = -2π f_r Δτ
                let phase_correction = -2.0 * PI * f_r * f_d / chirp_rate;
                row[bin] = c_mul(row[bin], c_exp_j(phase_correction));
            }

            // IFFT back
            fft_inplace(&mut row, true);
            for bin in 0..nr {
                result[pulse * nr + bin] = row[bin];
            }
        }

        result
    }

    // ── Range compression ────────────────────────────────────────────

    /// Perform range compression (matched filtering in fast-time) on each pulse.
    ///
    /// `data`: num_pulses × num_range_bins, row-major.
    /// `reference`: Reference waveform (transmitted pulse replica).
    ///
    /// Returns compressed data of same dimensions.
    pub fn range_compress(
        &self,
        data: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let np = self.params.num_pulses;
        let nr = self.params.num_range_bins;
        assert_eq!(data.len(), np * nr);

        let n_fft = next_pow2(nr.max(reference.len()));

        // Pre-compute reference spectrum (conjugate for matched filter)
        let mut ref_fft = vec![(0.0, 0.0); n_fft];
        for (i, &s) in reference.iter().enumerate() {
            if i < n_fft {
                ref_fft[i] = s;
            }
        }
        fft_inplace(&mut ref_fft, false);
        for s in ref_fft.iter_mut() {
            *s = c_conj(*s);
        }

        let mut result = vec![(0.0, 0.0); np * nr];

        for pulse in 0..np {
            let mut row = vec![(0.0, 0.0); n_fft];
            for bin in 0..nr {
                row[bin] = data[pulse * nr + bin];
            }
            fft_inplace(&mut row, false);

            // Multiply by conjugate of reference spectrum
            for bin in 0..n_fft {
                row[bin] = c_mul(row[bin], ref_fft[bin]);
            }

            fft_inplace(&mut row, true);
            for bin in 0..nr {
                result[pulse * nr + bin] = row[bin];
            }
        }

        result
    }

    // ── Migration curve estimation ───────────────────────────────────

    /// Estimate the migration curve from a range-Doppler map by finding the
    /// peak range bin at each Doppler frequency.
    ///
    /// `rd_map`: num_pulses × num_range_bins, row-major. Typically the output
    /// of range compression followed by slow-time DFT.
    ///
    /// Returns a `MigrationCurve` with the estimated shifts, velocity, and
    /// fit quality.
    pub fn estimate_migration_curve(&self, rd_map: &[(f64, f64)]) -> MigrationCurve {
        let np = self.params.num_pulses;
        let nr = self.params.num_range_bins;
        assert_eq!(rd_map.len(), np * nr);

        // Find peak range bin for each Doppler (slow-time) index
        let mut peak_bins = Vec::with_capacity(np);
        for pulse in 0..np {
            let mut max_mag = 0.0f64;
            let mut max_bin = 0usize;
            for bin in 0..nr {
                let mag = c_mag_sq(rd_map[pulse * nr + bin]);
                if mag > max_mag {
                    max_mag = mag;
                    max_bin = bin;
                }
            }
            peak_bins.push(max_bin as f64);
        }

        // Linear fit: shift[m] = a + b·m
        let n = np as f64;
        let sum_m: f64 = (0..np).map(|m| m as f64).sum();
        let sum_r: f64 = peak_bins.iter().sum();
        let sum_mr: f64 = (0..np).map(|m| m as f64 * peak_bins[m]).sum();
        let sum_mm: f64 = (0..np).map(|m| (m as f64) * (m as f64)).sum();

        let denom = n * sum_mm - sum_m * sum_m;
        let (a, b) = if denom.abs() > 1e-30 {
            let b_val = (n * sum_mr - sum_m * sum_r) / denom;
            let a_val = (sum_r - b_val * sum_m) / n;
            (a_val, b_val)
        } else {
            (peak_bins[0], 0.0)
        };

        // Shifts relative to first pulse
        let shifts: Vec<f64> = (0..np).map(|m| a + b * m as f64 - peak_bins[0]).collect();

        // Velocity estimate from slope b (bins/pulse)
        // b = 2·v / (c · PRF) · fs → v = b · c · PRF / (2 · fs)
        let velocity =
            b * C_LIGHT * self.params.prf_hz / (2.0 * self.params.sample_rate_hz);

        // R² goodness of fit
        let mean_r = sum_r / n;
        let ss_tot: f64 = peak_bins.iter().map(|r| (r - mean_r).powi(2)).sum();
        let ss_res: f64 = (0..np)
            .map(|m| {
                let predicted = a + b * m as f64;
                (peak_bins[m] - predicted).powi(2)
            })
            .sum();
        let fit_quality = if ss_tot > 1e-30 {
            (1.0 - ss_res / ss_tot).clamp(0.0, 1.0)
        } else {
            1.0 // All points identical → perfect fit
        };

        MigrationCurve {
            shifts,
            estimated_velocity: velocity,
            fit_quality,
        }
    }

    // ── Integration gain ─────────────────────────────────────────────

    /// Compute the integration gain by comparing peak power before and after
    /// range migration correction.
    ///
    /// `before`: data matrix before correction (num_pulses × num_range_bins).
    /// `after`: data matrix after correction (same dimensions).
    pub fn integration_gain(
        &self,
        before: &[(f64, f64)],
        after: &[(f64, f64)],
    ) -> IntegrationGain {
        let peak_before = before
            .iter()
            .map(|s| c_mag_sq(*s))
            .fold(0.0f64, f64::max);
        let peak_after = after
            .iter()
            .map(|s| c_mag_sq(*s))
            .fold(0.0f64, f64::max);

        let gain_db = if peak_before > 1e-30 {
            10.0 * (peak_after / peak_before).log10()
        } else {
            0.0
        };

        IntegrationGain {
            peak_before,
            peak_after,
            gain_db,
        }
    }

    // ── Full correction pipeline ─────────────────────────────────────

    /// Apply the full RMC pipeline to raw pulse data:
    /// 1. Range compression (if reference provided)
    /// 2. Keystone transform for linear RCM removal
    ///
    /// `data`: num_pulses × num_range_bins, row-major.
    /// `reference`: Optional reference waveform for range compression.
    pub fn correct(
        &self,
        data: &[(f64, f64)],
        reference: Option<&[(f64, f64)]>,
    ) -> Vec<(f64, f64)> {
        let compressed = if let Some(refwav) = reference {
            self.range_compress(data, refwav)
        } else {
            data.to_vec()
        };

        self.keystone_transform(&compressed)
    }
}

// ── Sinc interpolation helper ────────────────────────────────────────────

/// Windowed sinc interpolation at fractional position `t` into `data`.
/// Uses a Hann-windowed sinc kernel with a half-width of 4 samples.
fn sinc_interp(data: &[(f64, f64)], t: f64) -> (f64, f64) {
    let n = data.len();
    if n == 0 {
        return (0.0, 0.0);
    }

    let half_width = 4i64;
    let t_floor = t.floor() as i64;
    let frac = t - t_floor as f64;

    let mut result = (0.0, 0.0);
    let mut weight_sum = 0.0;

    for k in (t_floor - half_width + 1)..=(t_floor + half_width) {
        let idx = k.rem_euclid(n as i64) as usize;
        let delta = frac - (k - t_floor) as f64;
        let sinc_val = if delta.abs() < 1e-12 {
            1.0
        } else {
            (PI * delta).sin() / (PI * delta)
        };
        // Hann window
        let window_arg = (k - t_floor) as f64 - frac;
        let window = 0.5 + 0.5 * (PI * window_arg / half_width as f64).cos();
        let w = sinc_val * window;
        result = c_add(result, c_scale(data[idx], w));
        weight_sum += w;
    }

    if weight_sum.abs() > 1e-30 {
        c_scale(result, 1.0 / weight_sum)
    } else {
        (0.0, 0.0)
    }
}

// ── Chirp Z-transform (Bluestein's algorithm) ───────────────────────────

/// Evaluate the Z-transform at `m` points starting at normalized frequency
/// `f_start` with spacing `f_step`, using the chirp Z-transform.
fn czt_eval(input: &[(f64, f64)], m: usize, f_start: f64, f_step: f64) -> Vec<(f64, f64)> {
    let n = input.len();
    if n == 0 || m == 0 {
        return vec![(0.0, 0.0); m];
    }

    // A = exp(j·2π·f_start), W = exp(-j·2π·f_step)
    let a = c_exp_j(2.0 * PI * f_start);
    let _w = c_exp_j(-2.0 * PI * f_step);

    let l = next_pow2(n + m - 1);

    // Chirp sequence: w^{k²/2} = exp(-j·π·f_step·k²)
    let max_k = n.max(m);
    let chirp: Vec<(f64, f64)> = (0..=max_k)
        .map(|k| {
            let k2 = (k * k) as f64;
            c_exp_j(-PI * f_step * k2)
        })
        .collect();

    // Conjugate chirp for the convolution kernel
    let chirp_neg: Vec<(f64, f64)> = (0..=max_k)
        .map(|k| {
            let k2 = (k * k) as f64;
            c_exp_j(PI * f_step * k2)
        })
        .collect();

    // y[n] = x[n] · A^{-n} · chirp[n]
    let mut y = vec![(0.0, 0.0); l];
    let mut a_neg_n = (1.0, 0.0);
    let a_inv = c_conj(a); // For |A|=1, A^{-1} = conj(A)
    for k in 0..n {
        y[k] = c_mul(c_mul(input[k], a_neg_n), chirp[k]);
        a_neg_n = c_mul(a_neg_n, a_inv);
    }

    // h[n]: convolution kernel = chirp_neg values
    let mut h = vec![(0.0, 0.0); l];
    for k in 0..m {
        h[k] = chirp_neg[k];
    }
    // Negative time indices wrap around
    for k in 1..n {
        if l - k < l {
            h[l - k] = chirp_neg[k];
        }
    }

    // Convolve via FFT
    fft_inplace(&mut y, false);
    fft_inplace(&mut h, false);
    for i in 0..l {
        y[i] = c_mul(y[i], h[i]);
    }
    fft_inplace(&mut y, true);

    // Multiply output by chirp
    let mut result = Vec::with_capacity(m);
    for k in 0..m {
        result.push(c_mul(y[k], chirp[k]));
    }

    result
}

// ── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_params() -> RadarParams {
        RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 50.0e6,
            prf_hz: 1000.0,
            num_pulses: 64,
            num_range_bins: 128,
            sample_rate_hz: 100.0e6,
        }
    }

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn c_approx_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    // ── Complex arithmetic tests ─────────────────────────────────────

    #[test]
    fn test_complex_add() {
        let a = (1.0, 2.0);
        let b = (3.0, 4.0);
        let r = c_add(a, b);
        assert!(c_approx_eq(r, (4.0, 6.0), 1e-12));
    }

    #[test]
    fn test_complex_mul() {
        // (1+2i)(3+4i) = 3+4i+6i+8i² = -5+10i
        let r = c_mul((1.0, 2.0), (3.0, 4.0));
        assert!(c_approx_eq(r, (-5.0, 10.0), 1e-12));
    }

    #[test]
    fn test_complex_mag_sq() {
        assert!(approx_eq(c_mag_sq((3.0, 4.0)), 25.0, 1e-12));
    }

    #[test]
    fn test_complex_exp_j() {
        // e^{j·0} = 1+0i
        let r = c_exp_j(0.0);
        assert!(c_approx_eq(r, (1.0, 0.0), 1e-12));
        // e^{j·π/2} = 0+1i
        let r2 = c_exp_j(PI / 2.0);
        assert!(c_approx_eq(r2, (0.0, 1.0), 1e-12));
    }

    #[test]
    fn test_complex_div() {
        // (4+2i) / (2+0i) = 2+1i
        let r = c_div((4.0, 2.0), (2.0, 0.0));
        assert!(c_approx_eq(r, (2.0, 1.0), 1e-12));
    }

    // ── FFT tests ────────────────────────────────────────────────────

    #[test]
    fn test_fft_single_sample() {
        let mut buf = vec![(5.0, 3.0)];
        fft_inplace(&mut buf, false);
        assert!(c_approx_eq(buf[0], (5.0, 3.0), 1e-12));
    }

    #[test]
    fn test_fft_roundtrip() {
        let original = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let mut buf = original.clone();
        fft_inplace(&mut buf, false);
        fft_inplace(&mut buf, true);
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!(c_approx_eq(*a, *b, 1e-10));
        }
    }

    #[test]
    fn test_fft_known_values() {
        // DFT of [1, 1, 1, 1] = [4, 0, 0, 0]
        let mut buf = vec![(1.0, 0.0); 4];
        fft_inplace(&mut buf, false);
        assert!(c_approx_eq(buf[0], (4.0, 0.0), 1e-10));
        for k in 1..4 {
            assert!(c_mag(buf[k]) < 1e-10);
        }
    }

    // ── RadarParams tests ────────────────────────────────────────────

    #[test]
    fn test_wavelength() {
        let p = default_params();
        let expected = C_LIGHT / 10.0e9;
        assert!(approx_eq(p.wavelength(), expected, 1e-10));
    }

    #[test]
    fn test_range_resolution() {
        let p = default_params();
        // c / (2 * 50e6) = 2.998 m
        let expected = C_LIGHT / (2.0 * 50.0e6);
        assert!(approx_eq(p.range_resolution(), expected, 1e-6));
    }

    #[test]
    fn test_max_unambiguous_doppler() {
        let p = default_params();
        assert!(approx_eq(p.max_unambiguous_doppler(), 500.0, 1e-10));
    }

    #[test]
    fn test_cpi_duration() {
        let p = default_params();
        assert!(approx_eq(p.cpi_duration(), 0.064, 1e-10));
    }

    // ── Migration trajectory tests ───────────────────────────────────

    #[test]
    fn test_trajectory_zero_velocity() {
        let c = RangeMigrationCorrector::new(default_params());
        let traj = c.migration_trajectory(0.0);
        assert_eq!(traj.len(), 64);
        for &s in &traj {
            assert!(approx_eq(s, 0.0, 1e-15));
        }
    }

    #[test]
    fn test_trajectory_first_pulse_zero() {
        let c = RangeMigrationCorrector::new(default_params());
        let traj = c.migration_trajectory(300.0);
        assert!(approx_eq(traj[0], 0.0, 1e-15));
    }

    #[test]
    fn test_trajectory_positive_velocity() {
        let c = RangeMigrationCorrector::new(default_params());
        let traj = c.migration_trajectory(300.0);
        // Shift should increase linearly
        for m in 1..traj.len() {
            assert!(traj[m] > traj[m - 1]);
        }
    }

    #[test]
    fn test_total_migration_bins() {
        let c = RangeMigrationCorrector::new(default_params());
        let total = c.total_migration_bins(300.0);
        // 2 * 300 * (63/1000) / C * 100e6
        let expected = 2.0 * 300.0 * 63.0 / 1000.0 / C_LIGHT * 100.0e6;
        assert!(approx_eq(total, expected, 1e-6));
    }

    // ── Keystone transform test ──────────────────────────────────────

    #[test]
    fn test_keystone_preserves_dimensions() {
        let params = RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 10.0e6,
            prf_hz: 1000.0,
            num_pulses: 8,
            num_range_bins: 16,
            sample_rate_hz: 20.0e6,
        };
        let c = RangeMigrationCorrector::new(params);
        let data = vec![(1.0, 0.0); 8 * 16];
        let result = c.keystone_transform(&data);
        assert_eq!(result.len(), 8 * 16);
    }

    // ── CZT interpolation test ───────────────────────────────────────

    #[test]
    fn test_czt_matches_dft_for_uniform_spacing() {
        // CZT with f_start=0, f_step=1/N should match DFT
        let n = 8;
        let input: Vec<(f64, f64)> = (0..n).map(|i| ((i as f64).sin(), 0.0)).collect();

        let czt_result = czt_eval(&input, n, 0.0, 1.0 / n as f64);
        let dft_result = fft(&input);

        for k in 0..n {
            assert!(
                c_approx_eq(czt_result[k], dft_result[k], 1e-6),
                "Mismatch at bin {}: CZT={:?}, DFT={:?}",
                k,
                czt_result[k],
                dft_result[k]
            );
        }
    }

    // ── Range compression test ───────────────────────────────────────

    #[test]
    fn test_range_compression_peak() {
        let params = RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 10.0e6,
            prf_hz: 1000.0,
            num_pulses: 4,
            num_range_bins: 32,
            sample_rate_hz: 20.0e6,
        };
        let c = RangeMigrationCorrector::new(params);

        // Reference: chirp
        let reference: Vec<(f64, f64)> = (0..16)
            .map(|i| {
                let t = i as f64 / 20.0e6;
                let phase = 2.0 * PI * (5.0e6 * t + 5.0e6 * t * t);
                c_exp_j(phase)
            })
            .collect();

        // Data: place a copy of the reference at range bin 10 in pulse 0
        let mut data = vec![(0.0, 0.0); 4 * 32];
        for (i, &s) in reference.iter().enumerate() {
            if 10 + i < 32 {
                data[10 + i] = s;
            }
        }

        let compressed = c.range_compress(&data, &reference);

        // Find peak in pulse 0
        let mut max_mag = 0.0f64;
        for bin in 0..32 {
            let mag = c_mag_sq(compressed[bin]);
            if mag > max_mag {
                max_mag = mag;
            }
        }

        // Peak should be significantly above zero
        assert!(
            max_mag > 1.0,
            "Range compression should produce a detectable peak, got {}",
            max_mag
        );
    }

    // ── Doppler-range coupling compensation test ─────────────────────

    #[test]
    fn test_doppler_range_coupling_zero_doppler() {
        let params = RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 10.0e6,
            prf_hz: 1000.0,
            num_pulses: 4,
            num_range_bins: 16,
            sample_rate_hz: 20.0e6,
        };
        let c = RangeMigrationCorrector::new(params);

        let data: Vec<(f64, f64)> =
            (0..4 * 16).map(|i| ((i as f64 * 0.1).sin(), 0.0)).collect();
        let doppler_freqs = vec![0.0; 4];
        let chirp_rate = 10.0e6 / 1.0e-6; // 10 MHz / 1 μs

        let result =
            c.compensate_doppler_range_coupling(&data, &doppler_freqs, chirp_rate);

        // With zero Doppler, result should match input
        for i in 0..data.len() {
            assert!(
                c_approx_eq(data[i], result[i], 1e-6),
                "Zero Doppler should not change data at index {}",
                i
            );
        }
    }

    // ── Migration curve estimation test ──────────────────────────────

    #[test]
    fn test_estimate_migration_curve_stationary() {
        let params = RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 10.0e6,
            prf_hz: 1000.0,
            num_pulses: 8,
            num_range_bins: 32,
            sample_rate_hz: 20.0e6,
        };
        let c = RangeMigrationCorrector::new(params);

        // Stationary target: peak always at bin 15
        let mut data = vec![(0.0, 0.0); 8 * 32];
        for pulse in 0..8 {
            data[pulse * 32 + 15] = (10.0, 0.0);
        }

        let curve = c.estimate_migration_curve(&data);
        assert!(
            approx_eq(curve.estimated_velocity, 0.0, 1.0),
            "Stationary target should have near-zero velocity, got {}",
            curve.estimated_velocity
        );
        assert!(
            curve.fit_quality > 0.99,
            "Fit quality should be high for stationary target, got {}",
            curve.fit_quality
        );
    }

    // ── Integration gain test ────────────────────────────────────────

    #[test]
    fn test_integration_gain_positive() {
        let c = RangeMigrationCorrector::new(default_params());

        // Before: spread energy
        let before = vec![(1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];

        // After: concentrated energy
        let after = vec![(4.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];

        let gain = c.integration_gain(&before, &after);
        assert!(
            gain.gain_db > 0.0,
            "Gain should be positive when energy is concentrated"
        );
        assert!(approx_eq(gain.peak_before, 1.0, 1e-12));
        assert!(approx_eq(gain.peak_after, 16.0, 1e-12));
    }

    #[test]
    fn test_integration_gain_equal() {
        let c = RangeMigrationCorrector::new(default_params());
        let data = vec![(1.0, 0.0); 4];
        let gain = c.integration_gain(&data, &data);
        assert!(approx_eq(gain.gain_db, 0.0, 1e-10));
    }

    // ── Full pipeline test ───────────────────────────────────────────

    #[test]
    fn test_correct_without_reference() {
        let params = RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 10.0e6,
            prf_hz: 1000.0,
            num_pulses: 8,
            num_range_bins: 16,
            sample_rate_hz: 20.0e6,
        };
        let c = RangeMigrationCorrector::new(params);

        let data = vec![(1.0, 0.0); 8 * 16];
        let result = c.correct(&data, None);
        assert_eq!(result.len(), 8 * 16);
    }

    #[test]
    fn test_correct_with_reference() {
        let params = RadarParams {
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 10.0e6,
            prf_hz: 1000.0,
            num_pulses: 4,
            num_range_bins: 32,
            sample_rate_hz: 20.0e6,
        };
        let c = RangeMigrationCorrector::new(params);

        let reference: Vec<(f64, f64)> =
            (0..8).map(|i| c_exp_j(2.0 * PI * i as f64 / 8.0)).collect();

        let mut data = vec![(0.01, 0.0); 4 * 32];
        // Insert target echo
        for (i, &s) in reference.iter().enumerate() {
            if i < 32 {
                data[i] = c_add(data[i], s);
            }
        }

        let result = c.correct(&data, Some(&reference));
        assert_eq!(result.len(), 4 * 32);
    }
}
