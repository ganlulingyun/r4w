//! Wind Profiler Radar Signal Processing
//!
//! Implements Doppler beam swinging (DBS) wind profiler radar signal processing
//! for measuring atmospheric wind velocity profiles. Wind profiler radars
//! transmit vertically and at tilted beam positions, then use Doppler analysis
//! to recover the three-dimensional wind vector at each range gate.
//!
//! ## Doppler Beam Swinging (DBS) Technique
//!
//! The DBS method uses three or more beam directions:
//! - **Vertical beam**: measures vertical velocity (w)
//! - **North-tilted beam**: measures projection of u, v, w onto north beam
//! - **East-tilted beam**: measures projection of u, v, w onto east beam
//!
//! From the radial velocities measured in each beam, the full 3D wind vector
//! (u, v, w) is reconstructed using geometric relationships.
//!
//! ## Typical Operating Frequencies
//!
//! - **449 MHz**: Boundary-layer profilers (NOAA/NWS)
//! - **915 MHz**: Low-troposphere profilers
//! - **50 MHz**: Stratosphere-troposphere profilers
//!
//! ## Signal Processing Pipeline
//!
//! ```text
//! Raw IQ pulses
//!   -> Coherent integration (pulse summation for SNR improvement)
//!   -> Doppler spectrum (FFT of time-series at each range gate)
//!   -> Moment estimation (velocity, width, SNR from spectrum)
//!   -> DBS wind decomposition (radial -> u, v, w)
//!   -> Consensus averaging (outlier rejection)
//!   -> Wind profile output
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
const C: f64 = 299_792_458.0;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration for a wind profiler radar system.
#[derive(Debug, Clone)]
pub struct WindProfilerConfig {
    /// Transmit frequency in MHz (typically 449 or 915).
    pub frequency_mhz: f64,
    /// Number of range gates.
    pub num_range_gates: usize,
    /// Range gate spacing in metres.
    pub range_resolution_m: f64,
    /// Inter-pulse period in microseconds.
    pub ipp_us: f64,
    /// Number of pulses summed coherently (improves SNR by N).
    pub num_coherent_integrations: usize,
    /// Number of spectra averaged incoherently (reduces variance).
    pub num_incoherent_integrations: usize,
    /// Beam tilt angle from zenith in degrees (typically 15).
    pub beam_tilt_deg: f64,
}

impl WindProfilerConfig {
    /// Radar wavelength in metres.
    pub fn wavelength_m(&self) -> f64 {
        C / (self.frequency_mhz * 1e6)
    }

    /// Pulse repetition time in seconds (after coherent integration).
    pub fn prt_s(&self) -> f64 {
        self.ipp_us * 1e-6 * self.num_coherent_integrations as f64
    }

    /// Nyquist (maximum unambiguous) velocity in m/s.
    pub fn nyquist_velocity(&self) -> f64 {
        self.wavelength_m() / (4.0 * self.prt_s())
    }

    /// Maximum unambiguous range in metres.
    pub fn max_unambiguous_range(&self) -> f64 {
        C * self.ipp_us * 1e-6 / 2.0
    }

    /// Heights of the centre of each range gate (metres AGL).
    pub fn gate_heights(&self) -> Vec<f64> {
        (0..self.num_range_gates)
            .map(|i| (i as f64 + 0.5) * self.range_resolution_m)
            .collect()
    }
}

impl Default for WindProfilerConfig {
    fn default() -> Self {
        Self {
            frequency_mhz: 449.0,
            num_range_gates: 40,
            range_resolution_m: 100.0,
            ipp_us: 40.0,
            num_coherent_integrations: 64,
            num_incoherent_integrations: 50,
            beam_tilt_deg: 15.0,
        }
    }
}

// ─── Output ──────────────────────────────────────────────────────────────────

/// Wind profile output – one value per range gate.
#[derive(Debug, Clone)]
pub struct WindProfile {
    /// Height of each range gate in metres AGL.
    pub heights_m: Vec<f64>,
    /// East-west wind component (positive = eastward) in m/s.
    pub u_wind_mps: Vec<f64>,
    /// North-south wind component (positive = northward) in m/s.
    pub v_wind_mps: Vec<f64>,
    /// Vertical wind component (positive = upward) in m/s.
    pub w_wind_mps: Vec<f64>,
    /// Horizontal wind speed in m/s.
    pub speed_mps: Vec<f64>,
    /// Meteorological wind direction in degrees (direction the wind is *from*,
    /// 0 = north, 90 = east, clockwise).
    pub direction_deg: Vec<f64>,
    /// Signal-to-noise ratio in dB at each gate.
    pub snr_db: Vec<f64>,
}

// ─── Wind Profiler ───────────────────────────────────────────────────────────

/// Wind profiler radar processor.
///
/// After construction, call [`WindProfiler::process_dbs`] with IQ time-series
/// from the three beam directions to produce a [`WindProfile`].
#[derive(Debug, Clone)]
pub struct WindProfiler {
    config: WindProfilerConfig,
}

impl WindProfiler {
    /// Create a new wind profiler processor from the given configuration.
    pub fn new(config: WindProfilerConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &WindProfilerConfig {
        &self.config
    }

    /// Process DBS data to produce a wind profile.
    ///
    /// Each argument is a slice of range-gate time-series: `&[Vec<(f64, f64)>]`
    /// where the outer slice is indexed by range gate and each inner `Vec`
    /// contains the IQ samples (I, Q) over successive (coherent-integrated)
    /// pulses for that gate.
    ///
    /// `vertical` – zenith beam
    /// `north` – north-tilted beam
    /// `east` – east-tilted beam
    pub fn process_dbs(
        &self,
        vertical: &[Vec<(f64, f64)>],
        north: &[Vec<(f64, f64)>],
        east: &[Vec<(f64, f64)>],
    ) -> WindProfile {
        let n_gates = self.config.num_range_gates.min(vertical.len())
            .min(north.len())
            .min(east.len());
        let wavelength = self.config.wavelength_m();
        let prt = self.config.prt_s();
        let tilt = self.config.beam_tilt_deg;
        let heights = self.config.gate_heights();

        let mut u_wind = Vec::with_capacity(n_gates);
        let mut v_wind = Vec::with_capacity(n_gates);
        let mut w_wind = Vec::with_capacity(n_gates);
        let mut speed = Vec::with_capacity(n_gates);
        let mut direction = Vec::with_capacity(n_gates);
        let mut snr = Vec::with_capacity(n_gates);

        for g in 0..n_gates {
            let nfft = vertical[g].len().max(16);

            let spec_v = doppler_spectrum(&vertical[g], nfft);
            let spec_n = doppler_spectrum(&north[g], nfft);
            let spec_e = doppler_spectrum(&east[g], nfft);

            let vr_v = estimate_doppler_velocity(&spec_v, wavelength, prt);
            let vr_n = estimate_doppler_velocity(&spec_n, wavelength, prt);
            let vr_e = estimate_doppler_velocity(&spec_e, wavelength, prt);

            let (u, v, w) = dbs_wind_components(vr_n, vr_e, vr_v, tilt);
            let (spd, dir) = wind_speed_direction(u, v);

            // SNR estimate from the vertical beam spectrum
            let gate_snr = estimate_snr_db(&spec_v);

            u_wind.push(u);
            v_wind.push(v);
            w_wind.push(w);
            speed.push(spd);
            direction.push(dir);
            snr.push(gate_snr);
        }

        WindProfile {
            heights_m: heights[..n_gates].to_vec(),
            u_wind_mps: u_wind,
            v_wind_mps: v_wind,
            w_wind_mps: w_wind,
            speed_mps: speed,
            direction_deg: direction,
            snr_db: snr,
        }
    }
}

// ─── Spectral Analysis ───────────────────────────────────────────────────────

/// Compute the power spectral density of complex IQ data using a radix-2 FFT.
///
/// `iq_data` – time-ordered IQ samples (I, Q) for one range gate.
/// `num_points` – FFT length (will be rounded up to the next power of two;
///   zero-padding is applied if necessary).
///
/// Returns a `Vec<f64>` of length `num_points` containing power values in
/// linear scale, with zero-Doppler (DC) at index 0.
pub fn doppler_spectrum(iq_data: &[(f64, f64)], num_points: usize) -> Vec<f64> {
    let n = num_points.next_power_of_two().max(2);
    // Copy into a zero-padded complex buffer.
    let mut re = vec![0.0_f64; n];
    let mut im = vec![0.0_f64; n];
    for (i, &(r, q)) in iq_data.iter().enumerate().take(n) {
        re[i] = r;
        im[i] = q;
    }

    fft_in_place(&mut re, &mut im);

    // Power spectral density: |X[k]|^2 / N^2
    let scale = 1.0 / (n as f64 * n as f64);
    (0..n).map(|k| (re[k] * re[k] + im[k] * im[k]) * scale).collect()
}

/// First-moment Doppler velocity estimate from a power spectrum.
///
/// ```text
///        sum_k( v_k * S[k] )
/// v  =  ────────────────────
///           sum_k( S[k] )
/// ```
///
/// where `v_k = (k / N - 0.5) * wavelength / PRT` maps FFT bin `k` to
/// velocity (negative = toward, positive = away in the standard radar
/// convention), after FFT-shift.
pub fn estimate_doppler_velocity(spectrum: &[f64], wavelength_m: f64, prt_s: f64) -> f64 {
    let n = spectrum.len();
    if n == 0 {
        return 0.0;
    }
    let v_nyq = wavelength_m / (4.0 * prt_s);
    let mut sum_sv = 0.0_f64;
    let mut sum_s = 0.0_f64;
    for k in 0..n {
        // Map bin to velocity: bin 0 -> -v_nyq, bin N/2 -> 0, bin N-1 -> +v_nyq
        let v_k = ((k as f64 / n as f64) - 0.5) * 2.0 * v_nyq;
        sum_sv += spectrum[k] * v_k;
        sum_s += spectrum[k];
    }
    if sum_s.abs() < 1e-30 {
        0.0
    } else {
        sum_sv / sum_s
    }
}

/// Second-moment spectral width (m/s) – an indicator of turbulence and shear.
///
/// ```text
///             sum_k( (v_k - v_mean)^2 * S[k] )
/// sigma^2 =  ──────────────────────────────────
///                    sum_k( S[k] )
/// ```
pub fn spectral_width(
    spectrum: &[f64],
    mean_velocity: f64,
    wavelength_m: f64,
    prt_s: f64,
) -> f64 {
    let n = spectrum.len();
    if n == 0 {
        return 0.0;
    }
    let v_nyq = wavelength_m / (4.0 * prt_s);
    let mut sum_s2v = 0.0_f64;
    let mut sum_s = 0.0_f64;
    for k in 0..n {
        let v_k = ((k as f64 / n as f64) - 0.5) * 2.0 * v_nyq;
        let dv = v_k - mean_velocity;
        sum_s2v += spectrum[k] * dv * dv;
        sum_s += spectrum[k];
    }
    if sum_s.abs() < 1e-30 {
        0.0
    } else {
        (sum_s2v / sum_s).sqrt()
    }
}

/// Estimate SNR in dB from a Doppler spectrum.
///
/// Uses the edges of the spectrum (outermost 1/8 on each side) as a noise
/// estimate and the peak as the signal estimate.
fn estimate_snr_db(spectrum: &[f64]) -> f64 {
    let n = spectrum.len();
    if n < 8 {
        return 0.0;
    }
    let edge = n / 8;
    let noise: f64 = spectrum[..edge].iter().chain(spectrum[n - edge..].iter()).sum::<f64>()
        / (2 * edge) as f64;
    let peak = spectrum.iter().cloned().fold(0.0_f64, f64::max);
    if noise < 1e-30 {
        return 60.0; // cap
    }
    10.0 * (peak / noise).log10()
}

// ─── DBS Wind Decomposition ─────────────────────────────────────────────────

/// Convert radial velocities from the three DBS beams to u, v, w wind
/// components.
///
/// The tilted beams measure:
/// ```text
///   V_north  = w * cos(theta) + v * sin(theta)
///   V_east   = w * cos(theta) + u * sin(theta)
///   V_zenith = w
/// ```
///
/// Solving for u, v, w:
/// ```text
///   w = V_zenith
///   v = (V_north - w * cos(theta)) / sin(theta)
///   u = (V_east  - w * cos(theta)) / sin(theta)
/// ```
///
/// Returns `(u, v, w)` in m/s.
pub fn dbs_wind_components(
    v_radial_north: f64,
    v_radial_east: f64,
    v_radial_vertical: f64,
    tilt_deg: f64,
) -> (f64, f64, f64) {
    let theta = tilt_deg * PI / 180.0;
    let sin_t = theta.sin();
    let cos_t = theta.cos();

    let w = v_radial_vertical;
    let v = if sin_t.abs() < 1e-12 {
        0.0
    } else {
        (v_radial_north - w * cos_t) / sin_t
    };
    let u = if sin_t.abs() < 1e-12 {
        0.0
    } else {
        (v_radial_east - w * cos_t) / sin_t
    };

    (u, v, w)
}

/// Compute horizontal wind speed and meteorological direction from u, v.
///
/// Returns `(speed_mps, direction_deg)` where direction is the compass
/// bearing the wind is blowing **from** (0 = N, 90 = E, clockwise).
pub fn wind_speed_direction(u: f64, v: f64) -> (f64, f64) {
    let speed = (u * u + v * v).sqrt();
    if speed < 1e-12 {
        return (0.0, 0.0);
    }
    // Meteorological convention: direction wind is coming FROM.
    // atan2(-u, -v) gives the "from" direction in radians measured clockwise
    // from north.
    let dir_rad = (-u).atan2(-v);
    let mut dir_deg = dir_rad * 180.0 / PI;
    if dir_deg < 0.0 {
        dir_deg += 360.0;
    }
    (speed, dir_deg)
}

// ─── Pulse Processing ────────────────────────────────────────────────────────

/// Coherent integration – sum `n` consecutive pulses for each range gate.
///
/// `pulses` – outer index is pulse number, inner `Vec` is the IQ samples at
///   each range gate. All inner vectors must have the same length (number of
///   range gates).
///
/// `n` – number of pulses to integrate (must be >= 1).
///
/// Returns a vector of integrated groups. Each group has one entry per range
/// gate containing the complex sum of the `n` constituent pulses.
pub fn coherent_integration(
    pulses: &[Vec<(f64, f64)>],
    n: usize,
) -> Vec<Vec<(f64, f64)>> {
    if n == 0 || pulses.is_empty() {
        return Vec::new();
    }
    let n_groups = pulses.len() / n;
    let n_gates = pulses.first().map_or(0, |p| p.len());

    let mut result = Vec::with_capacity(n_groups);
    for g in 0..n_groups {
        let mut integrated = vec![(0.0_f64, 0.0_f64); n_gates];
        for p in 0..n {
            let pulse = &pulses[g * n + p];
            for (gate, &(ri, qi)) in pulse.iter().enumerate().take(n_gates) {
                integrated[gate].0 += ri;
                integrated[gate].1 += qi;
            }
        }
        result.push(integrated);
    }
    result
}

/// Incoherent integration – average power spectra.
///
/// Each input spectrum is a `Vec<f64>` of the same length. Returns the
/// element-wise average.
pub fn incoherent_integration(spectra: &[Vec<f64>]) -> Vec<f64> {
    if spectra.is_empty() {
        return Vec::new();
    }
    let n = spectra[0].len();
    let mut avg = vec![0.0_f64; n];
    for s in spectra {
        for (i, &v) in s.iter().enumerate().take(n) {
            avg[i] += v;
        }
    }
    let scale = 1.0 / spectra.len() as f64;
    for v in &mut avg {
        *v *= scale;
    }
    avg
}

// ─── Consensus Averaging ─────────────────────────────────────────────────────

/// Robust consensus average – iteratively rejects values farther than
/// `threshold` (in m/s) from the running mean until convergence.
///
/// This is the standard wind profiler quality-control algorithm used by NOAA.
pub fn consensus_average(values: &[f64], threshold: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    if values.len() == 1 {
        return values[0];
    }

    let mut accepted: Vec<f64> = values.to_vec();
    loop {
        let mean = accepted.iter().sum::<f64>() / accepted.len() as f64;
        let next: Vec<f64> = accepted
            .iter()
            .copied()
            .filter(|&v| (v - mean).abs() <= threshold)
            .collect();
        if next.len() == accepted.len() || next.is_empty() {
            let m = if next.is_empty() { mean } else { next.iter().sum::<f64>() / next.len() as f64 };
            return m;
        }
        accepted = next;
    }
}

// ─── Atmospheric Refractive Index ────────────────────────────────────────────

/// Compute the radio refractivity N from atmospheric state.
///
/// ```text
///   N = 77.6 * P / T  +  3.73e5 * e / T^2
/// ```
///
/// where
/// - `P` = total pressure in hPa
/// - `T` = temperature in Kelvin
/// - `e` = partial pressure of water vapour in hPa
///
/// The water-vapour pressure is computed from temperature and relative
/// humidity using the Magnus-Tetens approximation:
///
/// ```text
///   e_sat = 6.1078 * exp(17.27 * (T - 273.15) / (T - 273.15 + 237.3))
///   e     = (humidity / 100) * e_sat
/// ```
///
/// Returns `N` (dimensionless, typically 250-400 at sea level).
pub fn atmospheric_refractive_index(
    temp_k: f64,
    pressure_hpa: f64,
    humidity_percent: f64,
) -> f64 {
    let tc = temp_k - 273.15;
    let e_sat = 6.1078 * (17.27 * tc / (tc + 237.3)).exp();
    let e = (humidity_percent / 100.0) * e_sat;
    77.6 * pressure_hpa / temp_k + 3.73e5 * e / (temp_k * temp_k)
}

// ─── In-place radix-2 Cooley-Tukey FFT ──────────────────────────────────────

/// Bit-reversal permutation.
fn bit_reverse(x: usize, log2n: u32) -> usize {
    let mut result = 0usize;
    let mut val = x;
    for _ in 0..log2n {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place radix-2 DIT FFT.  `re` and `im` must have length that is a power
/// of two.
fn fft_in_place(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of two");
    let log2n = n.trailing_zeros();

    // Bit-reversal permutation.
    for i in 0..n {
        let j = bit_reverse(i, log2n);
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly passes.
    let mut size = 2usize;
    while size <= n {
        let half = size / 2;
        let angle_step = -2.0 * PI / size as f64;
        for start in (0..n).step_by(size) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let wr = angle.cos();
                let wi = angle.sin();
                let i1 = start + k;
                let i2 = start + k + half;
                let tr = wr * re[i2] - wi * im[i2];
                let ti = wr * im[i2] + wi * re[i2];
                re[i2] = re[i1] - tr;
                im[i2] = im[i1] - ti;
                re[i1] += tr;
                im[i1] += ti;
            }
        }
        size <<= 1;
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── DBS wind decomposition ──────────────────────────────────────────

    #[test]
    fn test_dbs_pure_vertical() {
        // Only vertical wind, no horizontal
        let (u, v, w) = dbs_wind_components(
            2.0_f64 * (15.0_f64 * PI / 180.0).cos(), // north beam
            2.0_f64 * (15.0_f64 * PI / 180.0).cos(), // east beam
            2.0,                                        // vertical
            15.0,
        );
        assert!(approx_eq(w, 2.0, TOL));
        assert!(approx_eq(u, 0.0, 0.01));
        assert!(approx_eq(v, 0.0, 0.01));
    }

    #[test]
    fn test_dbs_pure_east_wind() {
        // 10 m/s east wind, no vertical
        let tilt = 15.0_f64;
        let theta = tilt * PI / 180.0;
        let vr_north = 0.0 * theta.sin(); // no north component in north beam
        let vr_east = 10.0 * theta.sin(); // east wind projects onto east beam
        let vr_vert = 0.0;
        let (u, v, w) = dbs_wind_components(vr_north, vr_east, vr_vert, tilt);
        assert!(approx_eq(u, 10.0, TOL));
        assert!(approx_eq(v, 0.0, TOL));
        assert!(approx_eq(w, 0.0, TOL));
    }

    #[test]
    fn test_dbs_pure_north_wind() {
        let tilt = 15.0_f64;
        let theta = tilt * PI / 180.0;
        let vr_north = 5.0 * theta.sin();
        let vr_east = 0.0;
        let vr_vert = 0.0;
        let (u, v, w) = dbs_wind_components(vr_north, vr_east, vr_vert, tilt);
        assert!(approx_eq(v, 5.0, TOL));
        assert!(approx_eq(u, 0.0, TOL));
        assert!(approx_eq(w, 0.0, TOL));
    }

    #[test]
    fn test_dbs_combined_wind() {
        let tilt = 15.0_f64;
        let theta = tilt * PI / 180.0;
        let u_true = 3.0;
        let v_true = 4.0;
        let w_true = 0.5;
        let vr_north = w_true * theta.cos() + v_true * theta.sin();
        let vr_east = w_true * theta.cos() + u_true * theta.sin();
        let (u, v, w) = dbs_wind_components(vr_north, vr_east, w_true, tilt);
        assert!(approx_eq(u, u_true, TOL));
        assert!(approx_eq(v, v_true, TOL));
        assert!(approx_eq(w, w_true, TOL));
    }

    #[test]
    fn test_dbs_zero_tilt_returns_zero_horizontal() {
        // With zero tilt the tilted beams are both vertical, cannot resolve horizontal
        let (u, v, w) = dbs_wind_components(5.0, 5.0, 5.0, 0.0);
        assert!(approx_eq(u, 0.0, TOL));
        assert!(approx_eq(v, 0.0, TOL));
        assert!(approx_eq(w, 5.0, TOL));
    }

    // ── Wind speed and direction ────────────────────────────────────────

    #[test]
    fn test_wind_speed_direction_north() {
        // Wind blowing FROM the north (v = -10 means wind toward south, from north)
        let (spd, dir) = wind_speed_direction(0.0, -10.0);
        assert!(approx_eq(spd, 10.0, TOL));
        assert!(approx_eq(dir, 0.0, 0.1)); // from north = 0 deg
    }

    #[test]
    fn test_wind_speed_direction_south() {
        let (spd, dir) = wind_speed_direction(0.0, 10.0);
        assert!(approx_eq(spd, 10.0, TOL));
        assert!(approx_eq(dir, 180.0, 0.1));
    }

    #[test]
    fn test_wind_speed_direction_east() {
        // Wind blowing FROM the east means u = -10
        let (spd, dir) = wind_speed_direction(-10.0, 0.0);
        assert!(approx_eq(spd, 10.0, TOL));
        assert!(approx_eq(dir, 90.0, 0.1));
    }

    #[test]
    fn test_wind_speed_direction_west() {
        let (spd, dir) = wind_speed_direction(10.0, 0.0);
        assert!(approx_eq(spd, 10.0, TOL));
        assert!(approx_eq(dir, 270.0, 0.1));
    }

    #[test]
    fn test_wind_speed_direction_zero() {
        let (spd, dir) = wind_speed_direction(0.0, 0.0);
        assert!(approx_eq(spd, 0.0, TOL));
        assert!(approx_eq(dir, 0.0, TOL));
    }

    #[test]
    fn test_wind_speed_direction_southwest() {
        // u=5 (toward east), v=5 (toward north) => wind FROM 225 (SW)
        let (spd, dir) = wind_speed_direction(5.0, 5.0);
        let expected_speed = (50.0_f64).sqrt();
        assert!(approx_eq(spd, expected_speed, TOL));
        assert!(approx_eq(dir, 225.0, 0.1));
    }

    // ── Doppler spectrum and velocity ───────────────────────────────────

    #[test]
    fn test_doppler_spectrum_dc() {
        // Constant signal -> all energy at DC (bin 0)
        let iq: Vec<(f64, f64)> = (0..64).map(|_| (1.0, 0.0)).collect();
        let spec = doppler_spectrum(&iq, 64);
        assert_eq!(spec.len(), 64);
        // DC bin should be the strongest
        let peak_bin = spec.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(peak_bin, 0);
    }

    #[test]
    fn test_doppler_spectrum_tone() {
        // Single-frequency tone should produce a peak at the corresponding bin
        let n = 64usize;
        let bin_target = 8;
        let iq: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * bin_target as f64 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let spec = doppler_spectrum(&iq, n);
        let peak_bin = spec.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(peak_bin, bin_target);
    }

    #[test]
    fn test_doppler_spectrum_empty() {
        let spec = doppler_spectrum(&[], 4);
        assert_eq!(spec.len(), 4);
        // All zeros
        assert!(spec.iter().all(|&v| v.abs() < 1e-30));
    }

    #[test]
    fn test_estimate_doppler_velocity_zero() {
        // Symmetric spectrum -> velocity ~ 0
        // Use even-length spectrum; the bin mapping k/N - 0.5 has a slight
        // negative bias for even N, so we check within a reasonable bound.
        let n = 64;
        let spec = vec![1.0; n];
        let wavelength = 0.667;
        let prt = 1e-4;
        let v = estimate_doppler_velocity(&spec, wavelength, prt);
        let v_nyq = wavelength / (4.0 * prt);
        // For uniform spectrum the mean bin offset is -1/(2N), giving
        // velocity bias of -v_nyq / N.
        let expected_bias = -v_nyq / n as f64;
        assert!((v - expected_bias).abs() < 1.0,
            "v={v}, expected near {expected_bias}");
    }

    #[test]
    fn test_estimate_doppler_velocity_positive() {
        // Spectrum peaked in upper half -> positive velocity
        let n = 64;
        let mut spec = vec![0.001; n];
        spec[48] = 100.0; // upper half -> positive Doppler
        let v = estimate_doppler_velocity(&spec, 0.667, 1e-4);
        assert!(v > 0.0);
    }

    #[test]
    fn test_estimate_doppler_velocity_empty() {
        let v = estimate_doppler_velocity(&[], 0.667, 1e-4);
        assert!(approx_eq(v, 0.0, TOL));
    }

    // ── Spectral width ──────────────────────────────────────────────────

    #[test]
    fn test_spectral_width_narrow() {
        // Delta-like spectrum -> near-zero width
        let n = 64;
        let mut spec = vec![0.0; n];
        spec[32] = 1.0;
        let w = spectral_width(&spec, 0.0, 0.667, 1e-4);
        // Width should be small but not exactly zero due to discrete bins
        assert!(w < 10.0);
    }

    #[test]
    fn test_spectral_width_broad() {
        // Uniform spectrum -> broad width
        let spec = vec![1.0; 64];
        let w = spectral_width(&spec, 0.0, 0.667, 1e-4);
        assert!(w > 100.0); // should be wide
    }

    #[test]
    fn test_spectral_width_empty() {
        let w = spectral_width(&[], 0.0, 0.667, 1e-4);
        assert!(approx_eq(w, 0.0, TOL));
    }

    // ── Coherent integration ────────────────────────────────────────────

    #[test]
    fn test_coherent_integration_basic() {
        let pulses = vec![
            vec![(1.0, 2.0), (3.0, 4.0)],
            vec![(0.5, 0.5), (1.0, 1.0)],
            vec![(2.0, 1.0), (0.0, 0.0)],
            vec![(1.0, 1.0), (1.0, 1.0)],
        ];
        let result = coherent_integration(&pulses, 2);
        assert_eq!(result.len(), 2); // 4 pulses / 2 = 2 groups
        // First group: sum of pulses 0 and 1
        assert!(approx_eq(result[0][0].0, 1.5, TOL));
        assert!(approx_eq(result[0][0].1, 2.5, TOL));
        assert!(approx_eq(result[0][1].0, 4.0, TOL));
        assert!(approx_eq(result[0][1].1, 5.0, TOL));
        // Second group: sum of pulses 2 and 3
        assert!(approx_eq(result[1][0].0, 3.0, TOL));
        assert!(approx_eq(result[1][0].1, 2.0, TOL));
    }

    #[test]
    fn test_coherent_integration_single() {
        let pulses = vec![
            vec![(1.0, 2.0)],
            vec![(3.0, 4.0)],
        ];
        let result = coherent_integration(&pulses, 1);
        assert_eq!(result.len(), 2);
        assert!(approx_eq(result[0][0].0, 1.0, TOL));
        assert!(approx_eq(result[1][0].0, 3.0, TOL));
    }

    #[test]
    fn test_coherent_integration_empty() {
        let result = coherent_integration(&[], 4);
        assert!(result.is_empty());
    }

    #[test]
    fn test_coherent_integration_zero_n() {
        let pulses = vec![vec![(1.0, 2.0)]];
        let result = coherent_integration(&pulses, 0);
        assert!(result.is_empty());
    }

    // ── Incoherent integration ──────────────────────────────────────────

    #[test]
    fn test_incoherent_integration() {
        let spectra = vec![
            vec![1.0, 2.0, 3.0],
            vec![3.0, 2.0, 1.0],
        ];
        let avg = incoherent_integration(&spectra);
        assert_eq!(avg.len(), 3);
        assert!(approx_eq(avg[0], 2.0, TOL));
        assert!(approx_eq(avg[1], 2.0, TOL));
        assert!(approx_eq(avg[2], 2.0, TOL));
    }

    #[test]
    fn test_incoherent_integration_empty() {
        let avg = incoherent_integration(&[]);
        assert!(avg.is_empty());
    }

    // ── Consensus averaging ─────────────────────────────────────────────

    #[test]
    fn test_consensus_average_no_outliers() {
        let vals = vec![10.0, 10.1, 9.9, 10.0, 10.05];
        let avg = consensus_average(&vals, 1.0);
        assert!(approx_eq(avg, 10.01, 0.01));
    }

    #[test]
    fn test_consensus_average_with_outlier() {
        let vals = vec![10.0, 10.1, 9.9, 50.0, 10.0];
        // Threshold of 20 rejects 50.0 (mean ~ 18, 50 is 32 away) on
        // first pass, then the remaining values converge around 10.
        let avg = consensus_average(&vals, 20.0);
        assert!(avg < 15.0, "avg={avg}, expected < 15");
        assert!(avg > 9.0, "avg={avg}, expected > 9");
    }

    #[test]
    fn test_consensus_average_single() {
        assert!(approx_eq(consensus_average(&[42.0], 1.0), 42.0, TOL));
    }

    #[test]
    fn test_consensus_average_empty() {
        assert!(approx_eq(consensus_average(&[], 1.0), 0.0, TOL));
    }

    #[test]
    fn test_consensus_average_all_outliers() {
        // Values spread far apart, threshold very small
        let vals = vec![0.0, 100.0, 200.0];
        let avg = consensus_average(&vals, 1.0);
        // Should still return something (the last mean before empty)
        assert!(avg.is_finite());
    }

    // ── Atmospheric refractive index ────────────────────────────────────

    #[test]
    fn test_refractive_index_dry() {
        // Standard atmosphere, 0% humidity
        let n = atmospheric_refractive_index(288.15, 1013.25, 0.0);
        // Dry term: 77.6 * 1013.25 / 288.15 ≈ 272.8
        assert!(approx_eq(n, 272.8, 1.0));
    }

    #[test]
    fn test_refractive_index_humid() {
        // 50% humidity adds wet term
        let n_dry = atmospheric_refractive_index(288.15, 1013.25, 0.0);
        let n_humid = atmospheric_refractive_index(288.15, 1013.25, 50.0);
        // Humid should be larger than dry
        assert!(n_humid > n_dry);
    }

    #[test]
    fn test_refractive_index_high_altitude() {
        // Low pressure, low temp -> lower N
        let n = atmospheric_refractive_index(220.0, 200.0, 5.0);
        assert!(n < 100.0);
        assert!(n > 0.0);
    }

    // ── Config and profiler ─────────────────────────────────────────────

    #[test]
    fn test_config_defaults() {
        let cfg = WindProfilerConfig::default();
        assert!(approx_eq(cfg.frequency_mhz, 449.0, TOL));
        assert_eq!(cfg.num_range_gates, 40);
        assert!(approx_eq(cfg.beam_tilt_deg, 15.0, TOL));
    }

    #[test]
    fn test_wavelength_449mhz() {
        let cfg = WindProfilerConfig { frequency_mhz: 449.0, ..Default::default() };
        let wl = cfg.wavelength_m();
        // 299792458 / 449e6 ≈ 0.6677
        assert!(approx_eq(wl, 0.6677, 0.001));
    }

    #[test]
    fn test_wavelength_915mhz() {
        let cfg = WindProfilerConfig { frequency_mhz: 915.0, ..Default::default() };
        let wl = cfg.wavelength_m();
        // 299792458 / 915e6 ≈ 0.3276
        assert!(approx_eq(wl, 0.3276, 0.001));
    }

    #[test]
    fn test_prt() {
        let cfg = WindProfilerConfig {
            ipp_us: 40.0,
            num_coherent_integrations: 64,
            ..Default::default()
        };
        let prt = cfg.prt_s();
        // 40e-6 * 64 = 2.56e-3 s
        assert!(approx_eq(prt, 2.56e-3, 1e-9));
    }

    #[test]
    fn test_nyquist_velocity() {
        let cfg = WindProfilerConfig {
            frequency_mhz: 449.0,
            ipp_us: 40.0,
            num_coherent_integrations: 64,
            ..Default::default()
        };
        let v_nyq = cfg.nyquist_velocity();
        // lambda / (4 * PRT) = 0.6677 / (4 * 2.56e-3) ≈ 65.2 m/s
        assert!(v_nyq > 60.0 && v_nyq < 70.0);
    }

    #[test]
    fn test_max_unambiguous_range() {
        let cfg = WindProfilerConfig { ipp_us: 40.0, ..Default::default() };
        let r = cfg.max_unambiguous_range();
        // c * 40e-6 / 2 = 299792458 * 40e-6 / 2 = 5995.849 m
        assert!(approx_eq(r, 5995.849, 0.01));
    }

    #[test]
    fn test_gate_heights() {
        let cfg = WindProfilerConfig {
            num_range_gates: 5,
            range_resolution_m: 100.0,
            ..Default::default()
        };
        let h = cfg.gate_heights();
        assert_eq!(h.len(), 5);
        assert!(approx_eq(h[0], 50.0, TOL));
        assert!(approx_eq(h[1], 150.0, TOL));
        assert!(approx_eq(h[4], 450.0, TOL));
    }

    // ── End-to-end profiler ─────────────────────────────────────────────

    #[test]
    fn test_profiler_basic_process() {
        let cfg = WindProfilerConfig {
            frequency_mhz: 449.0,
            num_range_gates: 2,
            range_resolution_m: 100.0,
            ipp_us: 40.0,
            num_coherent_integrations: 1,
            num_incoherent_integrations: 1,
            beam_tilt_deg: 15.0,
        };
        let profiler = WindProfiler::new(cfg);

        // 16-sample IQ time-series at each gate (all zeros -> zero wind)
        let zeros: Vec<(f64, f64)> = vec![(0.0, 0.0); 16];
        let vertical = vec![zeros.clone(), zeros.clone()];
        let north = vec![zeros.clone(), zeros.clone()];
        let east = vec![zeros.clone(), zeros.clone()];

        let profile = profiler.process_dbs(&vertical, &north, &east);
        assert_eq!(profile.heights_m.len(), 2);
        assert_eq!(profile.u_wind_mps.len(), 2);
    }

    #[test]
    fn test_profiler_mismatched_gates() {
        let cfg = WindProfilerConfig {
            num_range_gates: 10,
            ..Default::default()
        };
        let profiler = WindProfiler::new(cfg);
        // Provide only 3 gates even though config says 10
        let data: Vec<Vec<(f64, f64)>> = (0..3)
            .map(|_| vec![(1.0, 0.0); 32])
            .collect();
        let profile = profiler.process_dbs(&data, &data, &data);
        assert_eq!(profile.heights_m.len(), 3);
    }

    // ── FFT sanity checks ───────────────────────────────────────────────

    #[test]
    fn test_fft_parseval() {
        // Parseval's theorem: sum(|x|^2) == sum(|X|^2) / N
        let n = 16;
        let iq: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 3.0 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let time_power: f64 = iq.iter().map(|&(r, i)| r * r + i * i).sum();

        let mut re: Vec<f64> = iq.iter().map(|&(r, _)| r).collect();
        let mut im: Vec<f64> = iq.iter().map(|&(_, i)| i).collect();
        fft_in_place(&mut re, &mut im);
        let freq_power: f64 = re.iter().zip(im.iter()).map(|(&r, &i)| r * r + i * i).sum::<f64>() / n as f64;

        assert!(approx_eq(time_power, freq_power, 1e-10));
    }

    #[test]
    fn test_fft_single_bin() {
        let n = 8;
        let bin = 3;
        let iq: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * bin as f64 * i as f64 / n as f64;
                (phase.cos(), phase.sin())
            })
            .collect();
        let mut re: Vec<f64> = iq.iter().map(|&(r, _)| r).collect();
        let mut im: Vec<f64> = iq.iter().map(|&(_, i)| i).collect();
        fft_in_place(&mut re, &mut im);

        // Only bin 3 should have significant energy
        for k in 0..n {
            let mag = (re[k] * re[k] + im[k] * im[k]).sqrt();
            if k == bin {
                assert!(mag > n as f64 * 0.9);
            } else {
                assert!(mag < 0.1);
            }
        }
    }

    // ── Roundtrip DBS ───────────────────────────────────────────────────

    #[test]
    fn test_roundtrip_dbs_to_speed_direction() {
        // 10 m/s from the northwest (315 deg): u = 10*sin(315) ≈ -7.07, v = 10*cos(315) ≈ 7.07
        // But u, v are wind *toward* direction, so from NW means toward SE:
        // u ≈ 7.07, v ≈ -7.07? No:
        // "from NW" means wind blows toward SE. In meteorological vector:
        // u = speed * sin(from_dir_rad) = 10 * sin(315 deg) = -7.07
        // v = speed * cos(from_dir_rad) = 10 * cos(315 deg) = 7.07
        // But u,v in wind_speed_direction are positive = toward east/north.
        // From=315 means blowing toward 135 (SE). So u should be positive (~7.07), v negative (~-7.07)
        // Actually, with our convention: from=315 => (-u).atan2(-v) should give 315 degrees.
        // Let's just pick u = -7.071, v = 7.071 and see what direction we get.
        let u = -7.071;
        let v = 7.071;
        let (spd, dir) = wind_speed_direction(u, v);
        assert!(approx_eq(spd, 10.0, 0.01));
        // from = atan2(-(-7.071), -(7.071)) = atan2(7.071, -7.071) = 135 deg
        assert!(approx_eq(dir, 135.0, 0.5));
    }
}
