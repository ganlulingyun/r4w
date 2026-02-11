//! FX correlator for radio interferometry arrays (VLBI / connected-element).
//!
//! Implements aperture-synthesis cross-correlation: for every antenna pair
//! (baseline) the signals are channelised with an FFT (**F** step) and then
//! cross-multiplied (**X** step) to form complex *visibilities*.  These
//! visibilities are the Fourier-domain samples of the sky brightness
//! distribution and are the fundamental observable for synthesis imaging.
//!
//! # Features
//!
//! - **FX correlation**: FFT-then-multiply architecture for all baselines
//! - **UV coverage**: geometric projection of baseline vectors to the UV plane
//! - **Delay tracking**: fractional-sample geometric delay compensation
//! - **Fringe rate**: rate of change of the interferometric phase
//! - **Radiometer equation helpers**: SEFD, synthesized beam size, image sensitivity
//!
//! # Example
//!
//! ```
//! use r4w_core::radio_telescope_correlator::{
//!     CorrelatorConfig, FxCorrelator, num_baselines,
//! };
//!
//! let config = CorrelatorConfig {
//!     num_antennas: 3,
//!     num_channels: 16,
//!     integration_time_s: 1.0,
//!     bandwidth_hz: 1.0e6,
//!     center_freq_hz: 1.42e9,
//! };
//!
//! let correlator = FxCorrelator::new(config);
//! assert_eq!(correlator.num_baselines(), 3);
//!
//! // Three antennas, each with num_channels*2 IQ samples (zero-padded FFT)
//! let n = 32;
//! let ant_a = vec![(1.0, 0.0); n];
//! let ant_b = vec![(0.0, 1.0); n];
//! let ant_c = vec![(1.0, 1.0); n];
//!
//! let vis = correlator.correlate(&[ant_a, ant_b, ant_c]);
//! assert_eq!(vis.num_baselines, 3);
//! assert_eq!(vis.baselines.len(), 3);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in m/s.
const C: f64 = 299_792_458.0;

/// Boltzmann constant in J/K.
const K_B: f64 = 1.380_649e-23;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for an FX correlator.
#[derive(Debug, Clone)]
pub struct CorrelatorConfig {
    /// Number of antennas in the array.
    pub num_antennas: usize,
    /// Number of spectral channels produced by the F (FFT) stage.
    pub num_channels: usize,
    /// Integration (accumulation) time in seconds.
    pub integration_time_s: f64,
    /// Total processed bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Centre observing frequency in Hz.
    pub center_freq_hz: f64,
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// A single baseline (antenna pair) with its cross-correlation products.
#[derive(Debug, Clone)]
pub struct Baseline {
    /// First antenna index (i < j).
    pub antenna_i: usize,
    /// Second antenna index.
    pub antenna_j: usize,
    /// Complex visibility spectrum: one (re, im) pair per spectral channel.
    pub visibility: Vec<(f64, f64)>,
    /// UV coordinate of this baseline in wavelengths (u, v).
    pub uv_coords: (f64, f64),
    /// Per-channel visibility amplitude.
    pub amplitude: Vec<f64>,
    /// Per-channel visibility phase in degrees.
    pub phase_deg: Vec<f64>,
}

/// Complete set of visibilities produced by the correlator.
#[derive(Debug, Clone)]
pub struct VisibilitySet {
    /// One entry per baseline (antenna pair).
    pub baselines: Vec<Baseline>,
    /// Total number of baselines = N*(N-1)/2.
    pub num_baselines: usize,
}

// ---------------------------------------------------------------------------
// FX Correlator
// ---------------------------------------------------------------------------

/// An FX correlator that cross-correlates antenna signals to form visibilities.
///
/// The "FX" name reflects the processing order: **F**FT each antenna's time
/// series into spectral channels, then cross-multiply (the **X** step) every
/// pair of antennas channel-by-channel.
#[derive(Debug, Clone)]
pub struct FxCorrelator {
    config: CorrelatorConfig,
}

impl FxCorrelator {
    /// Create a new correlator from the given configuration.
    pub fn new(config: CorrelatorConfig) -> Self {
        Self { config }
    }

    /// Return the number of baselines (antenna pairs).
    pub fn num_baselines(&self) -> usize {
        num_baselines(self.config.num_antennas)
    }

    /// Return a reference to the underlying configuration.
    pub fn config(&self) -> &CorrelatorConfig {
        &self.config
    }

    /// Correlate signals from all antennas and return a [`VisibilitySet`].
    ///
    /// `antenna_data` must contain one entry per antenna.  Each entry is a
    /// slice of `(re, im)` IQ samples.  The number of samples per antenna
    /// should be at least `num_channels`; if longer the FFT uses only the
    /// first `num_channels` samples.
    ///
    /// # Panics
    ///
    /// Panics if `antenna_data.len() != config.num_antennas` or if any
    /// antenna has fewer than `num_channels` samples.
    pub fn correlate(&self, antenna_data: &[Vec<(f64, f64)>]) -> VisibilitySet {
        assert_eq!(
            antenna_data.len(),
            self.config.num_antennas,
            "Expected {} antennas, got {}",
            self.config.num_antennas,
            antenna_data.len()
        );
        for (idx, data) in antenna_data.iter().enumerate() {
            assert!(
                data.len() >= self.config.num_channels,
                "Antenna {} has {} samples but num_channels is {}",
                idx,
                data.len(),
                self.config.num_channels
            );
        }

        // F step: FFT each antenna
        let spectra: Vec<Vec<(f64, f64)>> = antenna_data
            .iter()
            .map(|d| fft_forward(&d[..self.config.num_channels]))
            .collect();

        // X step: cross-multiply every pair
        let n = self.config.num_antennas;
        let mut baselines = Vec::with_capacity(num_baselines(n));

        for i in 0..n {
            for j in (i + 1)..n {
                let vis = cross_multiply(&spectra[i], &spectra[j]);
                let amplitude: Vec<f64> = vis.iter().map(|&(r, im)| (r * r + im * im).sqrt()).collect();
                let phase_deg: Vec<f64> = vis.iter().map(|&(r, im)| im.atan2(r).to_degrees()).collect();

                baselines.push(Baseline {
                    antenna_i: i,
                    antenna_j: j,
                    visibility: vis,
                    uv_coords: (0.0, 0.0), // Caller can set via compute_uv_coverage
                    amplitude,
                    phase_deg,
                });
            }
        }

        let nb = baselines.len();
        VisibilitySet {
            baselines,
            num_baselines: nb,
        }
    }
}

// ---------------------------------------------------------------------------
// Core DSP helpers (pure Rust, no external deps)
// ---------------------------------------------------------------------------

/// Compute the number of baselines for `n` antennas: N*(N-1)/2.
pub fn num_baselines(n: usize) -> usize {
    n * n.saturating_sub(1) / 2
}

/// Radix-2 DIT FFT (in-place, Cooley-Tukey).
///
/// `data` is modified in place.  Length must be a power of two.
fn fft_inplace(data: &mut [(f64, f64)]) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT length must be a power of two");

    // Bit-reversal permutation
    let mut j: usize = 0;
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
        let angle_step = -2.0 * PI / len as f64;
        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let w = (angle.cos(), angle.sin());
                let a = data[start + k];
                let b = data[start + k + half];
                // Complex multiply: w * b
                let wb = (
                    w.0 * b.0 - w.1 * b.1,
                    w.0 * b.1 + w.1 * b.0,
                );
                data[start + k] = (a.0 + wb.0, a.1 + wb.1);
                data[start + k + half] = (a.0 - wb.0, a.1 - wb.1);
            }
        }
        len <<= 1;
    }
}

/// Forward FFT returning a new vector.  Pads/truncates to `data.len()`
/// (which must be a power of two).
fn fft_forward(data: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut buf = data.to_vec();
    fft_inplace(&mut buf);
    buf
}

/// Inverse FFT (via conjugate trick): conjugate, forward FFT, conjugate, scale.
fn ifft(data: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = data.len() as f64;
    let mut buf: Vec<(f64, f64)> = data.iter().map(|&(r, i)| (r, -i)).collect();
    fft_inplace(&mut buf);
    buf.iter().map(|&(r, i)| (r / n, -i / n)).collect()
}

/// Cross-multiply two spectra element-wise: `conj(A) * B`.
fn cross_multiply(spec_a: &[(f64, f64)], spec_b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    spec_a
        .iter()
        .zip(spec_b.iter())
        .map(|(&(ar, ai), &(br, bi))| {
            // conj(A) * B = (ar - j*ai) * (br + j*bi)
            (ar * br + ai * bi, ar * bi - ai * br)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Public stand-alone functions
// ---------------------------------------------------------------------------

/// Perform FX cross-correlation of two signals.
///
/// 1. FFT each signal into `num_channels` spectral channels.
/// 2. Cross-multiply: `conj(FFT(signal_a)) * FFT(signal_b)`.
///
/// Both signals must have at least `num_channels` samples.
/// `num_channels` must be a power of two.
pub fn cross_correlate_fx(
    signal_a: &[(f64, f64)],
    signal_b: &[(f64, f64)],
    num_channels: usize,
) -> Vec<(f64, f64)> {
    assert!(
        num_channels.is_power_of_two(),
        "num_channels must be a power of two"
    );
    assert!(
        signal_a.len() >= num_channels,
        "signal_a too short: {} < {}",
        signal_a.len(),
        num_channels
    );
    assert!(
        signal_b.len() >= num_channels,
        "signal_b too short: {} < {}",
        signal_b.len(),
        num_channels
    );

    let spec_a = fft_forward(&signal_a[..num_channels]);
    let spec_b = fft_forward(&signal_b[..num_channels]);
    cross_multiply(&spec_a, &spec_b)
}

/// Compute UV coverage from antenna positions and observation geometry.
///
/// Each antenna position is `(x, y, z)` in metres (typically a local East-North-Up
/// frame aligned to the array centre).  The function projects each baseline
/// vector onto the UV plane using the given hour angle and declination.
///
/// Returns one `(u, v)` pair per baseline (N*(N-1)/2 total), in units of
/// wavelengths at `freq_hz`.
pub fn compute_uv_coverage(
    antenna_positions: &[(f64, f64, f64)],
    freq_hz: f64,
    hour_angle_rad: f64,
    declination_rad: f64,
) -> Vec<(f64, f64)> {
    let lambda = C / freq_hz;
    let n = antenna_positions.len();
    let sin_h = hour_angle_rad.sin();
    let cos_h = hour_angle_rad.cos();
    let sin_d = declination_rad.sin();
    let cos_d = declination_rad.cos();

    let mut uv = Vec::with_capacity(num_baselines(n));
    for i in 0..n {
        for j in (i + 1)..n {
            let dx = antenna_positions[j].0 - antenna_positions[i].0;
            let dy = antenna_positions[j].1 - antenna_positions[i].1;
            let dz = antenna_positions[j].2 - antenna_positions[i].2;

            // Standard interferometry rotation from (x,y,z) baseline to (u,v,w):
            //   u =  sin(H)*dx + cos(H)*dy
            //   v = -sin(dec)*cos(H)*dx + sin(dec)*sin(H)*dy + cos(dec)*dz
            // (w is the geometric delay direction, not needed for UV coverage)
            let u = (sin_h * dx + cos_h * dy) / lambda;
            let v = (-sin_d * cos_h * dx + sin_d * sin_h * dy + cos_d * dz) / lambda;
            uv.push((u, v));
        }
    }
    uv
}

/// Compute the fringe rate for a baseline.
///
/// The fringe rate is the time derivative of the interferometric phase and
/// determines how fast fringes rotate.
///
/// `baseline_m` is the *projected* East-West baseline length in metres.
/// `freq_hz` is the observing frequency.
/// `hour_angle_rate` is the sidereal rotation rate in rad/s
///   (Earth: ~7.2921e-5 rad/s).
/// `declination_rad` is the source declination.
///
/// Returns the fringe rate in Hz (cycles per second).
pub fn fringe_rate(
    baseline_m: f64,
    freq_hz: f64,
    hour_angle_rate: f64,
    declination_rad: f64,
) -> f64 {
    let lambda = C / freq_hz;
    // d(phase)/dt = (2*pi * B_ew * cos(dec) * omega_e) / lambda
    // fringe rate in Hz = 1/(2*pi) * d(phase)/dt
    baseline_m * declination_rad.cos() * hour_angle_rate / lambda
}

/// Apply a fractional-sample delay to a complex signal using sinc interpolation.
///
/// This is the *delay tracking* step: compensating for the geometric delay
/// between antennas so that a source at the phase centre produces zero
/// residual delay across all baselines.
///
/// Uses a windowed-sinc (Hann window) interpolator with a kernel half-width
/// of 4 samples.
pub fn delay_tracking(signal: &[(f64, f64)], delay_samples: f64) -> Vec<(f64, f64)> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }

    // Integer and fractional parts
    let int_delay = delay_samples.floor() as isize;
    let frac = delay_samples - delay_samples.floor();

    // Sinc interpolation kernel half-width
    let half_width: isize = 4;
    let mut output = Vec::with_capacity(n);

    for i in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for k in -half_width..=half_width {
            let src_idx = i as isize - int_delay + k;
            if src_idx < 0 || src_idx >= n as isize {
                continue;
            }
            let x = k as f64 - frac;
            // Windowed sinc: sinc(x) * hann_window(x)
            let sinc_val = if x.abs() < 1e-12 {
                1.0
            } else {
                (PI * x).sin() / (PI * x)
            };
            // Hann window over the kernel span
            let w = 0.5 * (1.0 + (PI * x / half_width as f64).cos());
            let coeff = sinc_val * w;
            let s = signal[src_idx as usize];
            re += s.0 * coeff;
            im += s.1 * coeff;
        }
        output.push((re, im));
    }
    output
}

/// Compute the System Equivalent Flux Density (SEFD) in Jy.
///
/// SEFD = 2 * k_B * T_sys / (eta * A)
///
/// where k_B is Boltzmann's constant, T_sys is the system temperature in K,
/// eta is the aperture efficiency (0..1), and A is the geometric collecting
/// area in m^2.
///
/// Returns SEFD in Jy (1 Jy = 10^-26 W m^-2 Hz^-1).
pub fn system_equivalent_flux_density(t_sys_k: f64, efficiency: f64, area_m2: f64) -> f64 {
    // 1 Jy = 1e-26 W/m^2/Hz, so divide by 1e-26 to convert W/m^2/Hz -> Jy
    2.0 * K_B * t_sys_k / (efficiency * area_m2) / 1e-26
}

/// Compute the synthesized beam size (angular resolution) in arcseconds.
///
/// theta = lambda / B_max  (radians), converted to arcseconds.
///
/// `max_baseline_m` is the longest baseline in metres.
/// `freq_hz` is the observing frequency.
pub fn synthesized_beam_size(max_baseline_m: f64, freq_hz: f64) -> f64 {
    let lambda = C / freq_hz;
    let theta_rad = lambda / max_baseline_m;
    // Convert radians to arcseconds: 1 rad = 206264.806... arcsec
    theta_rad * 206_264.806_247_096_36
}

/// Compute the theoretical image RMS sensitivity (noise floor) in Jy.
///
/// sigma = SEFD / sqrt(2 * bandwidth_hz * integration_s * num_baselines)
///
/// This is the thermal noise limit for a naturally-weighted image.
pub fn image_sensitivity(
    sefd: f64,
    bandwidth_hz: f64,
    integration_s: f64,
    num_baselines: usize,
) -> f64 {
    sefd / (2.0 * bandwidth_hz * integration_s * num_baselines as f64).sqrt()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // -----------------------------------------------------------------------
    // Baseline count
    // -----------------------------------------------------------------------

    #[test]
    fn test_num_baselines_zero() {
        assert_eq!(num_baselines(0), 0);
    }

    #[test]
    fn test_num_baselines_one() {
        assert_eq!(num_baselines(1), 0);
    }

    #[test]
    fn test_num_baselines_two() {
        assert_eq!(num_baselines(2), 1);
    }

    #[test]
    fn test_num_baselines_three() {
        assert_eq!(num_baselines(3), 3);
    }

    #[test]
    fn test_num_baselines_vla() {
        // VLA has 27 antennas -> 351 baselines
        assert_eq!(num_baselines(27), 351);
    }

    #[test]
    fn test_num_baselines_formula() {
        for n in 0..=20 {
            assert_eq!(num_baselines(n), n * n.saturating_sub(1) / 2);
        }
    }

    // -----------------------------------------------------------------------
    // FFT internals
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_single_sample() {
        let data = vec![(1.0, 0.0)];
        let result = fft_forward(&data);
        assert_eq!(result.len(), 1);
        assert!((result[0].0 - 1.0).abs() < 1e-12);
        assert!(result[0].1.abs() < 1e-12);
    }

    #[test]
    fn test_fft_dc() {
        // All ones -> DC bin = N, rest zero
        let n = 8;
        let data = vec![(1.0, 0.0); n];
        let result = fft_forward(&data);
        assert!((result[0].0 - n as f64).abs() < 1e-10);
        for k in 1..n {
            let mag = (result[k].0 * result[k].0 + result[k].1 * result[k].1).sqrt();
            assert!(mag < 1e-10, "bin {} should be zero, got {}", k, mag);
        }
    }

    #[test]
    fn test_fft_ifft_roundtrip() {
        let data: Vec<(f64, f64)> = (0..16)
            .map(|i| ((i as f64).sin(), (i as f64).cos()))
            .collect();
        let spec = fft_forward(&data);
        let reconstructed = ifft(&spec);
        for (orig, rec) in data.iter().zip(reconstructed.iter()) {
            assert!(
                (orig.0 - rec.0).abs() < 1e-10,
                "re mismatch: {} vs {}",
                orig.0,
                rec.0
            );
            assert!(
                (orig.1 - rec.1).abs() < 1e-10,
                "im mismatch: {} vs {}",
                orig.1,
                rec.1
            );
        }
    }

    // -----------------------------------------------------------------------
    // Cross-correlation (FX)
    // -----------------------------------------------------------------------

    #[test]
    fn test_cross_correlate_fx_identity() {
        // Correlating a signal with itself should give real-valued (positive)
        // auto-power spectrum
        let n = 16;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                ((2.0 * PI * t).cos(), (2.0 * PI * t).sin())
            })
            .collect();
        let vis = cross_correlate_fx(&signal, &signal, n);
        assert_eq!(vis.len(), n);
        // conj(A)*A = |A|^2 which is real and non-negative
        for (re, im) in &vis {
            assert!(*re >= -1e-10, "auto-correlation should be non-negative");
            assert!(im.abs() < 1e-10, "auto-correlation should be real");
        }
    }

    #[test]
    fn test_cross_correlate_fx_length() {
        let a = vec![(1.0, 0.0); 32];
        let b = vec![(0.0, 1.0); 32];
        let vis = cross_correlate_fx(&a, &b, 16);
        assert_eq!(vis.len(), 16);
    }

    #[test]
    fn test_cross_correlate_fx_zeros() {
        let n = 8;
        let a = vec![(0.0, 0.0); n];
        let b = vec![(1.0, 0.5); n];
        let vis = cross_correlate_fx(&a, &b, n);
        for (re, im) in &vis {
            assert!(re.abs() < 1e-15);
            assert!(im.abs() < 1e-15);
        }
    }

    #[test]
    fn test_cross_correlate_fx_conjugate_symmetry() {
        // If we swap A and B, the result should be the complex conjugate
        let n = 16;
        let a: Vec<(f64, f64)> = (0..n).map(|i| ((i as f64).sin(), (i as f64).cos())).collect();
        let b: Vec<(f64, f64)> = (0..n)
            .map(|i| ((i as f64 * 1.3).cos(), (i as f64 * 0.7).sin()))
            .collect();
        let vis_ab = cross_correlate_fx(&a, &b, n);
        let vis_ba = cross_correlate_fx(&b, &a, n);
        for (ab, ba) in vis_ab.iter().zip(vis_ba.iter()) {
            // conj(A)*B should be conjugate of conj(B)*A
            assert!((ab.0 - ba.0).abs() < 1e-10, "real parts should match");
            assert!((ab.1 + ba.1).abs() < 1e-10, "imag parts should negate");
        }
    }

    // -----------------------------------------------------------------------
    // FxCorrelator struct
    // -----------------------------------------------------------------------

    #[test]
    fn test_correlator_basic() {
        let config = CorrelatorConfig {
            num_antennas: 3,
            num_channels: 8,
            integration_time_s: 1.0,
            bandwidth_hz: 1.0e6,
            center_freq_hz: 1.42e9,
        };
        let corr = FxCorrelator::new(config);
        assert_eq!(corr.num_baselines(), 3);

        let data = vec![vec![(1.0, 0.0); 8]; 3];
        let vis = corr.correlate(&data);
        assert_eq!(vis.num_baselines, 3);
        assert_eq!(vis.baselines.len(), 3);
    }

    #[test]
    fn test_correlator_baseline_indices() {
        let config = CorrelatorConfig {
            num_antennas: 4,
            num_channels: 4,
            integration_time_s: 1.0,
            bandwidth_hz: 1.0e6,
            center_freq_hz: 1.0e9,
        };
        let corr = FxCorrelator::new(config);
        let data = vec![vec![(1.0, 0.0); 4]; 4];
        let vis = corr.correlate(&data);

        // Expected baseline pairs: (0,1),(0,2),(0,3),(1,2),(1,3),(2,3)
        let expected_pairs = vec![(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        assert_eq!(vis.baselines.len(), expected_pairs.len());
        for (bl, &(ei, ej)) in vis.baselines.iter().zip(expected_pairs.iter()) {
            assert_eq!(bl.antenna_i, ei);
            assert_eq!(bl.antenna_j, ej);
        }
    }

    #[test]
    fn test_correlator_amplitude_phase() {
        let config = CorrelatorConfig {
            num_antennas: 2,
            num_channels: 4,
            integration_time_s: 1.0,
            bandwidth_hz: 1.0e6,
            center_freq_hz: 1.0e9,
        };
        let corr = FxCorrelator::new(config);
        let a = vec![(1.0, 0.0); 4];
        let b = vec![(1.0, 0.0); 4];
        let vis = corr.correlate(&[a, b]);
        assert_eq!(vis.baselines.len(), 1);

        let bl = &vis.baselines[0];
        // Amplitude and phase vectors should have num_channels entries
        assert_eq!(bl.amplitude.len(), 4);
        assert_eq!(bl.phase_deg.len(), 4);
        // Amplitude should be non-negative
        for &a in &bl.amplitude {
            assert!(a >= 0.0);
        }
    }

    #[test]
    #[should_panic(expected = "Expected 3 antennas")]
    fn test_correlator_wrong_antenna_count() {
        let config = CorrelatorConfig {
            num_antennas: 3,
            num_channels: 4,
            integration_time_s: 1.0,
            bandwidth_hz: 1.0e6,
            center_freq_hz: 1.0e9,
        };
        let corr = FxCorrelator::new(config);
        let data = vec![vec![(1.0, 0.0); 4]; 2]; // Only 2 antennas
        corr.correlate(&data);
    }

    #[test]
    #[should_panic(expected = "has 2 samples but num_channels is 4")]
    fn test_correlator_insufficient_samples() {
        let config = CorrelatorConfig {
            num_antennas: 2,
            num_channels: 4,
            integration_time_s: 1.0,
            bandwidth_hz: 1.0e6,
            center_freq_hz: 1.0e9,
        };
        let corr = FxCorrelator::new(config);
        let data = vec![vec![(1.0, 0.0); 2]; 2]; // Too few samples
        corr.correlate(&data);
    }

    // -----------------------------------------------------------------------
    // UV coverage
    // -----------------------------------------------------------------------

    #[test]
    fn test_uv_coverage_two_antennas_east_west() {
        // Two antennas on the East-West axis, 100 m apart.
        // At hour angle = 0 (transit) the u coordinate should be ~100/lambda.
        let positions = vec![(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)];
        let freq = 1.0e9;
        let lambda = C / freq;
        let ha = 0.0;  // transit
        let dec = PI / 4.0; // 45 deg declination

        let uv = compute_uv_coverage(&positions, freq, ha, dec);
        assert_eq!(uv.len(), 1);

        // At h=0: u = sin(0)*100 + cos(0)*0 = 0
        // v = -sin(dec)*cos(0)*100 + sin(dec)*sin(0)*0 + cos(dec)*0
        //   = -sin(dec)*100 / lambda
        // So u ~ 0, v ~ -100*sin(dec)/lambda
        assert!(uv[0].0.abs() < 1e-6, "u should be ~0 at transit for E-W baseline");
        let expected_v = -dec.sin() * 100.0 / lambda;
        assert!(
            (uv[0].1 - expected_v).abs() < 1e-6,
            "v mismatch: {} vs {}",
            uv[0].1,
            expected_v
        );
    }

    #[test]
    fn test_uv_coverage_baseline_count() {
        let positions = vec![
            (0.0, 0.0, 0.0),
            (100.0, 0.0, 0.0),
            (0.0, 200.0, 0.0),
            (100.0, 200.0, 0.0),
        ];
        let uv = compute_uv_coverage(&positions, 1.0e9, 0.0, 0.0);
        assert_eq!(uv.len(), num_baselines(4));
        assert_eq!(uv.len(), 6);
    }

    #[test]
    fn test_uv_coverage_zero_baseline() {
        // Same position -> (0, 0) in UV
        let positions = vec![(50.0, 50.0, 0.0), (50.0, 50.0, 0.0)];
        let uv = compute_uv_coverage(&positions, 1.0e9, 0.5, 0.3);
        assert_eq!(uv.len(), 1);
        assert!(uv[0].0.abs() < 1e-10);
        assert!(uv[0].1.abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Fringe rate
    // -----------------------------------------------------------------------

    #[test]
    fn test_fringe_rate_zero_baseline() {
        let fr = fringe_rate(0.0, 1.0e9, 7.2921e-5, 0.0);
        assert!(fr.abs() < 1e-15);
    }

    #[test]
    fn test_fringe_rate_positive() {
        // 1 km E-W baseline at 1.4 GHz, declination = 0 (equator)
        let baseline = 1000.0;
        let freq = 1.4e9;
        let omega = 7.2921e-5; // Earth rotation rate
        let dec = 0.0;
        let fr = fringe_rate(baseline, freq, omega, dec);
        // f_r = B * cos(dec) * omega / lambda
        let lambda = C / freq;
        let expected = baseline * dec.cos() * omega / lambda;
        assert!((fr - expected).abs() < 1e-10);
        assert!(fr > 0.0, "fringe rate should be positive for this geometry");
    }

    #[test]
    fn test_fringe_rate_pole() {
        // Source at the pole (dec = pi/2): cos(pi/2) = 0, so fringe rate = 0
        let fr = fringe_rate(1000.0, 1.0e9, 7.2921e-5, PI / 2.0);
        assert!(fr.abs() < 1e-10, "fringe rate should vanish at the pole");
    }

    // -----------------------------------------------------------------------
    // Delay tracking
    // -----------------------------------------------------------------------

    #[test]
    fn test_delay_tracking_zero_delay() {
        let signal: Vec<(f64, f64)> = (0..32)
            .map(|i| ((i as f64 * 0.1).sin(), (i as f64 * 0.1).cos()))
            .collect();
        let delayed = delay_tracking(&signal, 0.0);
        assert_eq!(delayed.len(), signal.len());
        // With zero delay the output should closely match the input
        // (not exact due to windowed-sinc reconstruction but very close)
        for (s, d) in signal.iter().zip(delayed.iter()) {
            assert!(
                (s.0 - d.0).abs() < 1e-6,
                "re mismatch at zero delay: {} vs {}",
                s.0,
                d.0
            );
            assert!(
                (s.1 - d.1).abs() < 1e-6,
                "im mismatch at zero delay: {} vs {}",
                s.1,
                d.1
            );
        }
    }

    #[test]
    fn test_delay_tracking_integer_delay() {
        // An integer delay of 2 samples: output[i] ~ input[i-2]
        let n = 32;
        let signal: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                ((2.0 * PI * t).cos(), (2.0 * PI * t).sin())
            })
            .collect();
        let delayed = delay_tracking(&signal, 2.0);
        assert_eq!(delayed.len(), n);
        // Check interior samples (edges have boundary effects)
        for i in 6..(n - 6) {
            let expected = signal[i - 2];
            assert!(
                (delayed[i].0 - expected.0).abs() < 0.05,
                "re mismatch at i={}: {} vs {}",
                i,
                delayed[i].0,
                expected.0
            );
            assert!(
                (delayed[i].1 - expected.1).abs() < 0.05,
                "im mismatch at i={}: {} vs {}",
                i,
                delayed[i].1,
                expected.1
            );
        }
    }

    #[test]
    fn test_delay_tracking_empty_signal() {
        let result = delay_tracking(&[], 1.5);
        assert!(result.is_empty());
    }

    #[test]
    fn test_delay_tracking_output_length() {
        let signal = vec![(1.0, 0.0); 64];
        let delayed = delay_tracking(&signal, 3.7);
        assert_eq!(delayed.len(), 64);
    }

    // -----------------------------------------------------------------------
    // SEFD
    // -----------------------------------------------------------------------

    #[test]
    fn test_sefd_basic() {
        // VLA-like dish: T_sys = 25 K, efficiency = 0.45, area = 490 m^2
        let sefd = system_equivalent_flux_density(25.0, 0.45, 490.0);
        // Expected: 2 * 1.38e-23 * 25 / (0.45 * 490) / 1e-26
        //         = 6.9e-22 / 220.5 / 1e-26
        //         = 3.129e-24 / 1e-26
        //         = 312.9 Jy
        assert!(sefd > 200.0 && sefd < 500.0, "SEFD = {} Jy unexpected", sefd);
    }

    #[test]
    fn test_sefd_proportional_to_tsys() {
        let sefd1 = system_equivalent_flux_density(25.0, 0.5, 100.0);
        let sefd2 = system_equivalent_flux_density(50.0, 0.5, 100.0);
        assert!(
            (sefd2 / sefd1 - 2.0).abs() < 1e-10,
            "SEFD should double when T_sys doubles"
        );
    }

    #[test]
    fn test_sefd_inversely_proportional_to_area() {
        let sefd1 = system_equivalent_flux_density(25.0, 0.5, 100.0);
        let sefd2 = system_equivalent_flux_density(25.0, 0.5, 200.0);
        assert!(
            (sefd1 / sefd2 - 2.0).abs() < 1e-10,
            "SEFD should halve when area doubles"
        );
    }

    // -----------------------------------------------------------------------
    // Synthesized beam size
    // -----------------------------------------------------------------------

    #[test]
    fn test_beam_size_vla_a() {
        // VLA A-config: max baseline ~ 36 km, freq = 1.4 GHz
        let theta = synthesized_beam_size(36_000.0, 1.4e9);
        // lambda = 0.214 m, theta = 0.214/36000 rad ~ 5.95e-6 rad ~ 1.23 arcsec
        assert!(
            theta > 1.0 && theta < 2.0,
            "VLA A-config beam at L-band should be ~1.2 arcsec, got {}",
            theta
        );
    }

    #[test]
    fn test_beam_size_longer_baseline() {
        let theta_short = synthesized_beam_size(1000.0, 1.0e9);
        let theta_long = synthesized_beam_size(2000.0, 1.0e9);
        assert!(
            (theta_short / theta_long - 2.0).abs() < 1e-10,
            "Beam should halve with double baseline"
        );
    }

    #[test]
    fn test_beam_size_higher_freq() {
        let theta_low = synthesized_beam_size(1000.0, 1.0e9);
        let theta_high = synthesized_beam_size(1000.0, 2.0e9);
        assert!(
            (theta_low / theta_high - 2.0).abs() < 1e-10,
            "Beam should halve at double frequency"
        );
    }

    // -----------------------------------------------------------------------
    // Image sensitivity
    // -----------------------------------------------------------------------

    #[test]
    fn test_image_sensitivity_basic() {
        // SEFD = 300 Jy, BW = 100 MHz, t = 3600 s, 351 baselines (VLA)
        let sigma = image_sensitivity(300.0, 100.0e6, 3600.0, 351);
        // sigma = 300 / sqrt(2 * 1e8 * 3600 * 351) ~ 300 / sqrt(2.527e14)
        //       ~ 300 / 1.59e7 ~ 1.89e-5 Jy ~ 18.9 uJy
        assert!(sigma > 1e-6 && sigma < 1e-4, "sigma = {} Jy unexpected", sigma);
    }

    #[test]
    fn test_sensitivity_improves_with_integration() {
        let sigma1 = image_sensitivity(300.0, 1.0e8, 100.0, 10);
        let sigma2 = image_sensitivity(300.0, 1.0e8, 400.0, 10);
        // 4x integration -> 2x improvement (sqrt)
        assert!(
            (sigma1 / sigma2 - 2.0).abs() < 1e-10,
            "sensitivity should improve as sqrt(t)"
        );
    }

    #[test]
    fn test_sensitivity_improves_with_baselines() {
        let sigma1 = image_sensitivity(300.0, 1.0e8, 100.0, 10);
        let sigma2 = image_sensitivity(300.0, 1.0e8, 100.0, 40);
        // 4x baselines -> 2x improvement
        assert!(
            (sigma1 / sigma2 - 2.0).abs() < 1e-10,
            "sensitivity should improve as sqrt(N_bl)"
        );
    }

    #[test]
    fn test_sensitivity_improves_with_bandwidth() {
        let sigma1 = image_sensitivity(300.0, 1.0e8, 100.0, 10);
        let sigma2 = image_sensitivity(300.0, 4.0e8, 100.0, 10);
        assert!(
            (sigma1 / sigma2 - 2.0).abs() < 1e-10,
            "sensitivity should improve as sqrt(BW)"
        );
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_correlator_two_antennas_min() {
        let config = CorrelatorConfig {
            num_antennas: 2,
            num_channels: 4,
            integration_time_s: 0.001,
            bandwidth_hz: 1.0e3,
            center_freq_hz: 100.0e6,
        };
        let corr = FxCorrelator::new(config);
        assert_eq!(corr.num_baselines(), 1);
        let data = vec![vec![(0.5, -0.5); 4]; 2];
        let vis = corr.correlate(&data);
        assert_eq!(vis.num_baselines, 1);
        assert_eq!(vis.baselines[0].visibility.len(), 4);
    }

    #[test]
    fn test_uv_coverage_single_antenna() {
        // Single antenna -> no baselines
        let positions = vec![(0.0, 0.0, 0.0)];
        let uv = compute_uv_coverage(&positions, 1.0e9, 0.0, 0.0);
        assert!(uv.is_empty());
    }

    #[test]
    fn test_fringe_rate_units() {
        // Sanity: for a ~100 m baseline at 1 GHz, fringe rate should be
        // on the order of mHz to Hz
        let fr = fringe_rate(100.0, 1.0e9, 7.2921e-5, 0.5);
        assert!(
            fr.abs() < 100.0,
            "fringe rate {} Hz seems unreasonably large for 100 m baseline",
            fr
        );
        assert!(
            fr.abs() > 1e-6,
            "fringe rate {} Hz seems unreasonably small",
            fr
        );
    }

    #[test]
    fn test_delay_tracking_negative_delay() {
        let signal: Vec<(f64, f64)> = (0..32)
            .map(|i| {
                let t = i as f64 / 32.0;
                ((2.0 * PI * t).cos(), (2.0 * PI * t).sin())
            })
            .collect();
        let delayed = delay_tracking(&signal, -1.0);
        assert_eq!(delayed.len(), 32);
        // Negative delay shifts forward: output[i] ~ input[i+1]
        for i in 6..26 {
            let expected = signal[i + 1];
            assert!(
                (delayed[i].0 - expected.0).abs() < 0.1,
                "negative delay re mismatch at i={}",
                i
            );
        }
    }

    #[test]
    fn test_config_accessible() {
        let config = CorrelatorConfig {
            num_antennas: 5,
            num_channels: 32,
            integration_time_s: 10.0,
            bandwidth_hz: 16.0e6,
            center_freq_hz: 5.0e9,
        };
        let corr = FxCorrelator::new(config);
        assert_eq!(corr.config().num_antennas, 5);
        assert_eq!(corr.config().num_channels, 32);
        assert!((corr.config().integration_time_s - 10.0).abs() < 1e-15);
    }
}
