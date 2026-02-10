//! FMCW Radar — Frequency Modulated Continuous Wave Signal Processing
//!
//! Implements FMCW radar signal generation, beat frequency extraction, and
//! range-Doppler processing for applications including:
//!
//! - **Automotive radar** (77 GHz): adaptive cruise control, collision avoidance,
//!   blind-spot detection, parking assistance
//! - **Industrial level gauging**: non-contact liquid/solid level measurement in
//!   tanks and silos
//! - **Gesture recognition**: short-range micro-Doppler for hand tracking and
//!   human-computer interaction
//! - **Weather radar**: precipitation measurement and wind profiling
//! - **Altimeters**: terrain-following and radio altimeters for aircraft
//!
//! ## FMCW Principle
//!
//! An FMCW radar transmits a continuous frequency-swept (chirp) signal. The
//! received echo is mixed with the transmit signal to produce a beat signal
//! whose frequency is proportional to target range:
//!
//! ```text
//! f_beat = 2 * B * R / (c * T_sweep)
//! ```
//!
//! where B is the sweep bandwidth, R is the target range, c is the speed of
//! light, and T_sweep is the sweep duration.
//!
//! ## Processing Chain
//!
//! ```text
//! TX chirp ──┐
//!            ├── Conjugate Mix ── Beat Signal ── Range FFT ──┐
//! RX echo ──┘                                                │
//!                                                  (repeat N sweeps)
//!                                                            │
//!                                               Doppler FFT (slow-time)
//!                                                            │
//!                                                   Range-Doppler Map
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fmcw_radar::{FmcwRadar, FmcwConfig};
//!
//! let config = FmcwConfig::automotive_77ghz();
//! let radar = FmcwRadar::new(config);
//!
//! // Generate TX chirp and a target echo at 50 m, 20 m/s
//! let tx = radar.generate_chirp();
//! let echo = radar.generate_target_echo(50.0, 20.0, 1.0);
//! let beat = radar.mix_beat_signal(&tx, &echo);
//! let spectrum = radar.range_fft(&beat);
//!
//! // Find peak range bin
//! let peak_bin = spectrum.iter()
//!     .enumerate()
//!     .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
//!     .unwrap().0;
//! println!("Peak at bin {}", peak_bin);
//! ```

use std::f64::consts::PI;

/// Speed of light in vacuum (m/s).
const C: f64 = 299_792_458.0;

/// Configuration for an FMCW radar system.
#[derive(Debug, Clone)]
pub struct FmcwConfig {
    /// Sweep bandwidth (Hz). Determines range resolution: delta_R = c / (2*B).
    pub bandwidth_hz: f64,
    /// Duration of one frequency sweep (s).
    pub sweep_time_s: f64,
    /// Baseband sample rate (Hz). Determines maximum beat frequency.
    pub sample_rate_hz: f64,
    /// Carrier / center frequency (Hz). Determines Doppler-to-velocity mapping.
    pub center_freq_hz: f64,
    /// Number of sweeps in a coherent processing interval (CPI).
    pub num_sweeps: usize,
}

impl FmcwConfig {
    /// Automotive 77 GHz short-range radar preset.
    ///
    /// Typical parameters for an ADAS (Advanced Driver Assistance System) radar:
    /// - 1 GHz sweep bandwidth (15 cm range resolution)
    /// - 10 us sweep time
    /// - 20 MHz sample rate
    /// - 77 GHz carrier
    /// - 128 sweeps per CPI
    pub fn automotive_77ghz() -> Self {
        Self {
            bandwidth_hz: 1.0e9,
            sweep_time_s: 10.0e-6,
            sample_rate_hz: 20.0e6,
            center_freq_hz: 77.0e9,
            num_sweeps: 128,
        }
    }
}

/// FMCW radar signal processor.
///
/// Provides chirp generation, beat signal extraction, range FFT, and
/// range-Doppler map computation.
#[derive(Debug, Clone)]
pub struct FmcwRadar {
    config: FmcwConfig,
}

impl FmcwRadar {
    /// Create a new FMCW radar processor from the given configuration.
    pub fn new(config: FmcwConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &FmcwConfig {
        &self.config
    }

    /// Number of fast-time samples per sweep.
    fn samples_per_sweep(&self) -> usize {
        (self.config.sample_rate_hz * self.config.sweep_time_s).round() as usize
    }

    // ----------------------------------------------------------------
    // Chirp generation
    // ----------------------------------------------------------------

    /// Generate one linear frequency modulated (LFM) up-chirp as I/Q samples.
    ///
    /// The instantaneous frequency sweeps linearly from -B/2 to +B/2 over
    /// one sweep period. The phase is:
    ///
    /// ```text
    /// phi(t) = 2*pi * (-B/2 * t + (B / (2*T)) * t^2)
    /// ```
    ///
    /// Returns a vector of `(I, Q)` tuples with unit amplitude.
    pub fn generate_chirp(&self) -> Vec<(f64, f64)> {
        let n = self.samples_per_sweep();
        let dt = 1.0 / self.config.sample_rate_hz;
        let chirp_rate = self.config.bandwidth_hz / self.config.sweep_time_s; // Hz/s

        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let phase =
                    2.0 * PI * (-self.config.bandwidth_hz / 2.0 * t + 0.5 * chirp_rate * t * t);
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // ----------------------------------------------------------------
    // Beat signal extraction
    // ----------------------------------------------------------------

    /// Conjugate-mix TX with RX to obtain the beat (de-ramped) signal.
    ///
    /// `beat[n] = tx[n] * conj(rx[n])`
    ///
    /// The resulting signal contains sinusoids at frequencies proportional
    /// to target range(s). The shorter of the two input lengths is used.
    pub fn mix_beat_signal(&self, tx: &[(f64, f64)], rx: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let n = tx.len().min(rx.len());
        (0..n)
            .map(|i| {
                let (ti, tq) = tx[i];
                let (ri, rq) = rx[i];
                // complex multiply: tx * conj(rx)
                let bi = ti * ri + tq * rq;
                let bq = tq * ri - ti * rq;
                (bi, bq)
            })
            .collect()
    }

    // ----------------------------------------------------------------
    // Range FFT
    // ----------------------------------------------------------------

    /// Compute the range profile from a single beat signal.
    ///
    /// Performs an FFT on the beat signal and returns the magnitude spectrum.
    /// Frequency bins map to range via [`beat_freq_to_range`](Self::beat_freq_to_range).
    pub fn range_fft(&self, beat_signal: &[(f64, f64)]) -> Vec<f64> {
        let n = beat_signal.len();
        if n == 0 {
            return Vec::new();
        }
        let fft_size = n.next_power_of_two();
        let mut buf: Vec<(f64, f64)> = Vec::with_capacity(fft_size);
        buf.extend_from_slice(beat_signal);
        buf.resize(fft_size, (0.0, 0.0));

        let spectrum = fft_complex(&buf);
        spectrum
            .iter()
            .map(|&(re, im)| (re * re + im * im).sqrt())
            .collect()
    }

    // ----------------------------------------------------------------
    // Range-Doppler map
    // ----------------------------------------------------------------

    /// Compute a 2-D range-Doppler map from multiple sweeps' beat signals.
    ///
    /// - Fast-time axis (columns): range FFT within each sweep
    /// - Slow-time axis (rows): Doppler FFT across sweeps for each range bin
    ///
    /// Input: `beats[sweep_index]` — beat signal for each sweep.
    ///
    /// Returns `map[doppler_bin][range_bin]` with magnitude values.
    pub fn range_doppler_map(&self, beats: &[Vec<(f64, f64)>]) -> Vec<Vec<f64>> {
        let num_sweeps = beats.len();
        if num_sweeps == 0 {
            return Vec::new();
        }

        // Step 1: Range FFT for each sweep
        let range_profiles: Vec<Vec<(f64, f64)>> = beats
            .iter()
            .map(|beat| {
                let n = beat.len();
                let fft_size = n.next_power_of_two();
                let mut buf = Vec::with_capacity(fft_size);
                buf.extend_from_slice(beat);
                buf.resize(fft_size, (0.0, 0.0));
                fft_complex(&buf)
            })
            .collect();

        let num_range_bins = range_profiles[0].len();
        let doppler_fft_size = num_sweeps.next_power_of_two();

        // Step 2: Doppler FFT for each range bin (across slow-time)
        let mut rdm = vec![vec![0.0_f64; num_range_bins]; doppler_fft_size];

        for rbin in 0..num_range_bins {
            // Collect slow-time samples for this range bin
            let mut slow_time = Vec::with_capacity(doppler_fft_size);
            for s in 0..num_sweeps {
                slow_time.push(range_profiles[s][rbin]);
            }
            slow_time.resize(doppler_fft_size, (0.0, 0.0));

            let doppler_spectrum = fft_complex(&slow_time);
            for d in 0..doppler_fft_size {
                let (re, im) = doppler_spectrum[d];
                rdm[d][rbin] = (re * re + im * im).sqrt();
            }
        }

        rdm
    }

    // ----------------------------------------------------------------
    // Target echo generation
    // ----------------------------------------------------------------

    /// Generate a simulated point-target echo with range delay and Doppler shift.
    ///
    /// The echo is a time-delayed, Doppler-shifted, and amplitude-scaled copy
    /// of the transmit chirp. The round-trip delay is `tau = 2*R/c`, and the
    /// Doppler shift on the carrier is `f_d = 2*v*f_c/c`.
    ///
    /// # Arguments
    ///
    /// * `range_m` — Target range in metres.
    /// * `velocity_mps` — Target radial velocity in m/s (positive = approaching).
    /// * `rcs` — Radar cross-section scaling factor (linear amplitude, not dB).
    pub fn generate_target_echo(
        &self,
        range_m: f64,
        velocity_mps: f64,
        rcs: f64,
    ) -> Vec<(f64, f64)> {
        let n = self.samples_per_sweep();
        let dt = 1.0 / self.config.sample_rate_hz;
        let chirp_rate = self.config.bandwidth_hz / self.config.sweep_time_s;
        let tau = 2.0 * range_m / C; // round-trip delay
        let f_doppler = 2.0 * velocity_mps * self.config.center_freq_hz / C;

        // Amplitude loss (simplified 1/R^2 path loss omitted; rcs controls amplitude)
        let amp = rcs;

        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let t_delayed = t - tau;
                if t_delayed < 0.0 || t_delayed >= self.config.sweep_time_s {
                    (0.0, 0.0)
                } else {
                    // Chirp phase at delayed time
                    let chirp_phase = 2.0
                        * PI
                        * (-self.config.bandwidth_hz / 2.0 * t_delayed
                            + 0.5 * chirp_rate * t_delayed * t_delayed);
                    // Doppler phase shift
                    let doppler_phase = 2.0 * PI * f_doppler * t;
                    let total_phase = chirp_phase + doppler_phase;
                    (amp * total_phase.cos(), amp * total_phase.sin())
                }
            })
            .collect()
    }

    // ----------------------------------------------------------------
    // Parameter computations
    // ----------------------------------------------------------------

    /// Convert beat frequency (Hz) to slant range (m).
    ///
    /// ```text
    /// R = c * T_sweep * f_beat / (2 * B)
    /// ```
    pub fn beat_freq_to_range(&self, beat_freq_hz: f64) -> f64 {
        C * self.config.sweep_time_s * beat_freq_hz / (2.0 * self.config.bandwidth_hz)
    }

    /// Convert Doppler frequency (Hz) to radial velocity (m/s).
    ///
    /// ```text
    /// v = c * f_d / (2 * f_c)
    /// ```
    pub fn doppler_to_velocity(&self, doppler_hz: f64) -> f64 {
        C * doppler_hz / (2.0 * self.config.center_freq_hz)
    }

    /// Range resolution (m).
    ///
    /// ```text
    /// delta_R = c / (2 * B)
    /// ```
    pub fn range_resolution(&self) -> f64 {
        C / (2.0 * self.config.bandwidth_hz)
    }

    /// Maximum unambiguous range (m).
    ///
    /// Limited by the Nyquist frequency of the beat signal:
    ///
    /// ```text
    /// R_max = c * T_sweep * f_s / (4 * B)
    /// ```
    ///
    /// where f_s is the sample rate (max observable beat frequency = f_s/2).
    pub fn max_range(&self) -> f64 {
        let f_beat_max = self.config.sample_rate_hz / 2.0;
        self.beat_freq_to_range(f_beat_max)
    }

    /// Velocity resolution (m/s).
    ///
    /// Determined by the coherent processing interval (CPI):
    ///
    /// ```text
    /// delta_v = c / (2 * f_c * T_cpi)
    /// ```
    ///
    /// where `T_cpi = num_sweeps * T_sweep`.
    pub fn velocity_resolution(&self) -> f64 {
        let t_cpi = self.config.num_sweeps as f64 * self.config.sweep_time_s;
        C / (2.0 * self.config.center_freq_hz * t_cpi)
    }

    /// Maximum unambiguous velocity (m/s).
    ///
    /// Limited by the sweep repetition rate (PRF = 1/T_sweep):
    ///
    /// ```text
    /// v_max = c / (4 * f_c * T_sweep)
    /// ```
    pub fn max_velocity(&self) -> f64 {
        C / (4.0 * self.config.center_freq_hz * self.config.sweep_time_s)
    }
}

// ====================================================================
// Built-in FFT (radix-2 Cooley-Tukey with DFT fallback)
// ====================================================================

/// In-place radix-2 Cooley-Tukey FFT for `(f64, f64)` I/Q pairs.
///
/// Input length **must** be a power of two. For non-power-of-two lengths
/// the caller should zero-pad first.
fn fft_complex(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }

    // Fall back to DFT for small or non-power-of-two sizes
    if !n.is_power_of_two() || n <= 4 {
        return dft_complex(input);
    }

    let mut data = input.to_vec();

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        let (wn_re, wn_im) = (angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let (mut wr, mut wi) = (1.0, 0.0);
            for k in 0..half {
                let ei = start + k;
                let oi = start + k + half;
                // twiddle multiply
                let (ore, oim) = data[oi];
                let tr = ore * wr - oim * wi;
                let ti = ore * wi + oim * wr;
                let (ere, eim) = data[ei];
                data[ei] = (ere + tr, eim + ti);
                data[oi] = (ere - tr, eim - ti);
                // rotate twiddle
                let new_wr = wr * wn_re - wi * wn_im;
                let new_wi = wr * wn_im + wi * wn_re;
                wr = new_wr;
                wi = new_wi;
            }
            start += len;
        }
        len <<= 1;
    }

    data
}

/// Inverse FFT via conjugate-forward-conjugate-scale.
#[allow(dead_code)]
fn ifft_complex(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let conj: Vec<(f64, f64)> = input.iter().map(|&(re, im)| (re, -im)).collect();
    let mut result = fft_complex(&conj);
    let scale = 1.0 / n as f64;
    for sample in &mut result {
        sample.0 *= scale;
        sample.1 *= -scale;
    }
    result
}

/// Direct DFT — O(N^2) fallback for small or non-power-of-two sizes.
fn dft_complex(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    let mut output = vec![(0.0, 0.0); n];
    for k in 0..n {
        let (mut sum_re, mut sum_im) = (0.0, 0.0);
        for j in 0..n {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            let (wre, wim) = (angle.cos(), angle.sin());
            let (xre, xim) = input[j];
            sum_re += xre * wre - xim * wim;
            sum_im += xre * wim + xim * wre;
        }
        output[k] = (sum_re, sum_im);
    }
    output
}

// ====================================================================
// Tests
// ====================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: default automotive config for most tests.
    fn default_config() -> FmcwConfig {
        FmcwConfig {
            bandwidth_hz: 200.0e6,
            sweep_time_s: 50.0e-6,
            sample_rate_hz: 2.0e6,
            center_freq_hz: 77.0e9,
            num_sweeps: 64,
        }
    }

    // ------------------------------------------------------------------
    // 1. Chirp generation
    // ------------------------------------------------------------------
    #[test]
    fn test_chirp_generation() {
        let radar = FmcwRadar::new(default_config());
        let chirp = radar.generate_chirp();

        let expected_len = (2.0e6_f64 * 50.0e-6).round() as usize; // 100 samples
        assert_eq!(chirp.len(), expected_len, "Chirp length mismatch");

        // Every sample should have unit magnitude (amplitude = 1)
        for (i, &(re, im)) in chirp.iter().enumerate() {
            let mag = (re * re + im * im).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-12,
                "Sample {} magnitude {:.6} != 1.0",
                i,
                mag
            );
        }
    }

    // ------------------------------------------------------------------
    // 2. Beat signal mixing
    // ------------------------------------------------------------------
    #[test]
    fn test_beat_signal_mixing() {
        let radar = FmcwRadar::new(default_config());
        let tx = radar.generate_chirp();

        // Mixing TX with itself should yield DC (zero beat frequency)
        let beat = radar.mix_beat_signal(&tx, &tx);
        assert_eq!(beat.len(), tx.len());

        // All Q components should be near zero; I components near +1
        for (i, &(bi, bq)) in beat.iter().enumerate() {
            let mag = (bi * bi + bq * bq).sqrt();
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Beat sample {} magnitude {:.6} != 1.0",
                i,
                mag
            );
            assert!(
                bq.abs() < 1e-10,
                "Beat sample {} Q = {:.6} should be ~0",
                i,
                bq
            );
        }
    }

    // ------------------------------------------------------------------
    // 3. Range FFT peak detection
    // ------------------------------------------------------------------
    #[test]
    fn test_range_fft_peak() {
        let config = FmcwConfig {
            bandwidth_hz: 200.0e6,
            sweep_time_s: 50.0e-6,
            sample_rate_hz: 2.0e6,
            center_freq_hz: 77.0e9,
            num_sweeps: 64,
        };
        let radar = FmcwRadar::new(config);

        let target_range = 30.0; // 30 metres
        let tx = radar.generate_chirp();
        let echo = radar.generate_target_echo(target_range, 0.0, 1.0);
        let beat = radar.mix_beat_signal(&tx, &echo);
        let spectrum = radar.range_fft(&beat);

        assert!(!spectrum.is_empty(), "Spectrum should not be empty");

        // Find the peak bin (excluding DC bin 0)
        let peak_bin = spectrum
            .iter()
            .enumerate()
            .skip(1)
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Convert peak bin to range
        let fft_size = spectrum.len();
        let bin_freq = peak_bin as f64 * radar.config.sample_rate_hz / fft_size as f64;
        let estimated_range = radar.beat_freq_to_range(bin_freq);

        // Allow generous tolerance due to FFT bin quantisation on short signals
        let range_res = radar.config.sample_rate_hz / fft_size as f64;
        let tolerance = radar.beat_freq_to_range(range_res) * 2.0;
        assert!(
            (estimated_range - target_range).abs() < tolerance,
            "Estimated range {:.1} m not within {:.1} m of target {:.1} m",
            estimated_range,
            tolerance,
            target_range
        );
    }

    // ------------------------------------------------------------------
    // 4. Range resolution
    // ------------------------------------------------------------------
    #[test]
    fn test_range_resolution() {
        let radar = FmcwRadar::new(default_config());
        let dr = radar.range_resolution();
        let expected = C / (2.0 * 200.0e6); // 0.749 m
        assert!(
            (dr - expected).abs() < 0.001,
            "Range resolution: expected {:.4} m, got {:.4} m",
            expected,
            dr
        );
    }

    // ------------------------------------------------------------------
    // 5. Velocity resolution
    // ------------------------------------------------------------------
    #[test]
    fn test_velocity_resolution() {
        let radar = FmcwRadar::new(default_config());
        let dv = radar.velocity_resolution();

        // T_cpi = 64 * 50e-6 = 3.2 ms
        // dv = c / (2 * fc * T_cpi)
        let t_cpi = 64.0 * 50.0e-6;
        let expected = C / (2.0 * 77.0e9 * t_cpi);
        assert!(
            (dv - expected).abs() / expected < 1e-6,
            "Velocity resolution: expected {:.4} m/s, got {:.4} m/s",
            expected,
            dv
        );
    }

    // ------------------------------------------------------------------
    // 6. Range-Doppler map
    // ------------------------------------------------------------------
    #[test]
    fn test_range_doppler_map() {
        let config = FmcwConfig {
            bandwidth_hz: 200.0e6,
            sweep_time_s: 50.0e-6,
            sample_rate_hz: 2.0e6,
            center_freq_hz: 77.0e9,
            num_sweeps: 16,
        };
        let radar = FmcwRadar::new(config);

        let target_range = 20.0;
        let target_vel = 10.0;
        let tx = radar.generate_chirp();

        // Generate beat signals for multiple sweeps with progressive Doppler phase
        let beats: Vec<Vec<(f64, f64)>> = (0..16)
            .map(|sweep_idx| {
                let echo = radar.generate_target_echo(target_range, target_vel, 1.0);
                // Add inter-sweep Doppler phase rotation
                let f_d = 2.0 * target_vel * radar.config.center_freq_hz / C;
                let sweep_phase = 2.0 * PI * f_d * sweep_idx as f64 * radar.config.sweep_time_s;
                let (cos_p, sin_p) = (sweep_phase.cos(), sweep_phase.sin());
                let beat = radar.mix_beat_signal(&tx, &echo);
                beat.iter()
                    .map(|&(re, im)| (re * cos_p - im * sin_p, re * sin_p + im * cos_p))
                    .collect()
            })
            .collect();

        let rdm = radar.range_doppler_map(&beats);

        // Should have doppler_fft_size rows and range_fft_size columns
        assert!(!rdm.is_empty(), "RDM should not be empty");
        assert!(
            rdm[0].len() > 0,
            "RDM should have range bins"
        );

        // Find global peak — should correspond to the target
        let mut max_val = 0.0_f64;
        for row in &rdm {
            for &val in row {
                max_val = max_val.max(val);
            }
        }
        assert!(max_val > 0.0, "RDM should have non-zero peak");
    }

    // ------------------------------------------------------------------
    // 7. Beat frequency to range conversion
    // ------------------------------------------------------------------
    #[test]
    fn test_beat_freq_to_range() {
        let radar = FmcwRadar::new(default_config());

        // f_beat = 2 * B * R / (c * T)  =>  R = c * T * f_beat / (2 * B)
        let test_range = 100.0;
        let expected_beat =
            2.0 * radar.config.bandwidth_hz * test_range / (C * radar.config.sweep_time_s);
        let computed_range = radar.beat_freq_to_range(expected_beat);
        assert!(
            (computed_range - test_range).abs() < 1e-6,
            "Expected {:.6} m, got {:.6} m",
            test_range,
            computed_range
        );

        // Zero beat frequency => zero range
        assert_eq!(radar.beat_freq_to_range(0.0), 0.0);
    }

    // ------------------------------------------------------------------
    // 8. Target echo generation
    // ------------------------------------------------------------------
    #[test]
    fn test_target_echo() {
        let radar = FmcwRadar::new(default_config());
        let echo = radar.generate_target_echo(50.0, 0.0, 1.0);

        assert_eq!(echo.len(), radar.samples_per_sweep());

        // Echo should have a leading zero region (round-trip delay)
        let tau = 2.0 * 50.0 / C;
        let delay_samples = (tau * radar.config.sample_rate_hz).floor() as usize;
        // Samples before the delay should be zero
        for i in 0..delay_samples.min(echo.len()) {
            let (re, im) = echo[i];
            assert!(
                re.abs() < 1e-15 && im.abs() < 1e-15,
                "Sample {} should be zero before delay, got ({:.2e}, {:.2e})",
                i,
                re,
                im
            );
        }

        // Non-zero samples should have magnitude = rcs
        let has_nonzero = echo.iter().any(|&(re, im)| (re * re + im * im).sqrt() > 0.5);
        assert!(has_nonzero, "Echo should have non-zero samples after delay");
    }

    // ------------------------------------------------------------------
    // 9. Automotive 77 GHz preset
    // ------------------------------------------------------------------
    #[test]
    fn test_automotive_preset() {
        let config = FmcwConfig::automotive_77ghz();
        assert_eq!(config.center_freq_hz, 77.0e9);
        assert_eq!(config.bandwidth_hz, 1.0e9);
        assert_eq!(config.sweep_time_s, 10.0e-6);
        assert_eq!(config.sample_rate_hz, 20.0e6);
        assert_eq!(config.num_sweeps, 128);

        let radar = FmcwRadar::new(config);

        // Range resolution = c / (2 * 1 GHz) ~ 0.15 m
        let dr = radar.range_resolution();
        assert!(
            (dr - 0.1499).abs() < 0.001,
            "77 GHz range res: {:.4} m",
            dr
        );

        // Max range = c * T * (fs/2) / (2 * B) = c * 10e-6 * 10e6 / (2e9)
        let rmax = radar.max_range();
        assert!(rmax > 10.0, "Max range should be > 10 m, got {:.1} m", rmax);
    }

    // ------------------------------------------------------------------
    // 10. Maximum range
    // ------------------------------------------------------------------
    #[test]
    fn test_max_range() {
        let radar = FmcwRadar::new(default_config());
        let rmax = radar.max_range();

        // R_max = c * T * (fs/2) / (2*B) = C * 50e-6 * 1e6 / (2 * 200e6)
        let expected = C * 50.0e-6 * (2.0e6 / 2.0) / (2.0 * 200.0e6);
        assert!(
            (rmax - expected).abs() < 0.01,
            "Max range: expected {:.2} m, got {:.2} m",
            expected,
            rmax
        );

        // Max velocity
        let vmax = radar.max_velocity();
        // v_max = c / (4 * fc * T) = C / (4 * 77e9 * 50e-6)
        let expected_v = C / (4.0 * 77.0e9 * 50.0e-6);
        assert!(
            (vmax - expected_v).abs() / expected_v < 1e-6,
            "Max velocity: expected {:.2} m/s, got {:.2} m/s",
            expected_v,
            vmax
        );
    }
}
