//! Seismic ambient noise tomography for subsurface velocity imaging.
//!
//! Implements surface wave velocity imaging using cross-correlation of continuous
//! ambient seismic noise between station pairs. The methodology follows Shapiro &
//! Campillo (2004) and Bensen et al. (2007) for extracting empirical Green's
//! functions from noise cross-correlations.
//!
//! # Theory
//!
//! Ambient noise tomography (ANT) exploits the fact that the cross-correlation of
//! a diffuse noise wavefield recorded at two stations approximates the Green's
//! function between them. Surface wave group and phase velocities extracted from
//! these empirical Green's functions are then inverted for subsurface velocity
//! structure.
//!
//! Key steps:
//! 1. **Preprocessing**: instrument response removal, bandpass filtering, temporal
//!    normalization (one-bit or running absolute mean), spectral whitening
//! 2. **Cross-correlation**: segment-wise correlation with stacking for SNR
//! 3. **Green's function extraction**: symmetric component from causal + acausal
//! 4. **Dispersion measurement**: FTAN for group velocity, zero-crossing for phase
//! 5. **Inversion**: 1D depth inversion via sensitivity kernels, 2D tomography
//!    via linearized ray-based inversion with regularization
//!
//! # Example
//!
//! ```
//! use r4w_core::seismic_ambient_noise_tomographer::{
//!     AmbientNoiseProcessor, FrequencyBand, StationPair,
//! };
//!
//! let processor = AmbientNoiseProcessor::new(
//!     100.0,   // sample rate Hz
//!     60.0,    // segment length seconds
//!     0.5,     // overlap fraction
//!     FrequencyBand { min_hz: 0.05, max_hz: 0.5 },
//!     30.0,    // max lag seconds
//! );
//!
//! let pair = StationPair::new(0.0, 0.0, 100_000.0, 0.0);
//! assert!((pair.inter_station_distance() - 100_000.0).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

// ─── Station Pair ────────────────────────────────────────────────────────────

/// A pair of seismic stations with 2D positions.
#[derive(Debug, Clone)]
pub struct StationPair {
    /// Station A x-coordinate (meters).
    pub station_a_x: f64,
    /// Station A y-coordinate (meters).
    pub station_a_y: f64,
    /// Station B x-coordinate (meters).
    pub station_b_x: f64,
    /// Station B y-coordinate (meters).
    pub station_b_y: f64,
}

impl StationPair {
    /// Create a new station pair from (x, y) positions in meters.
    pub fn new(ax: f64, ay: f64, bx: f64, by: f64) -> Self {
        Self {
            station_a_x: ax,
            station_a_y: ay,
            station_b_x: bx,
            station_b_y: by,
        }
    }

    /// Euclidean inter-station distance in meters.
    pub fn inter_station_distance(&self) -> f64 {
        let dx = self.station_b_x - self.station_a_x;
        let dy = self.station_b_y - self.station_a_y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Azimuth from station A to station B in radians (0 = +x, pi/2 = +y).
    pub fn azimuth(&self) -> f64 {
        let dx = self.station_b_x - self.station_a_x;
        let dy = self.station_b_y - self.station_a_y;
        dy.atan2(dx)
    }

    /// Midpoint between the two stations.
    pub fn midpoint(&self) -> (f64, f64) {
        (
            (self.station_a_x + self.station_b_x) / 2.0,
            (self.station_a_y + self.station_b_y) / 2.0,
        )
    }
}

// ─── Frequency Band ──────────────────────────────────────────────────────────

/// Frequency band for filtering.
#[derive(Debug, Clone, Copy)]
pub struct FrequencyBand {
    /// Minimum frequency in Hz.
    pub min_hz: f64,
    /// Maximum frequency in Hz.
    pub max_hz: f64,
}

impl FrequencyBand {
    /// Center frequency.
    pub fn center(&self) -> f64 {
        (self.min_hz + self.max_hz) / 2.0
    }

    /// Bandwidth.
    pub fn bandwidth(&self) -> f64 {
        self.max_hz - self.min_hz
    }
}

// ─── Dispersion Curve ────────────────────────────────────────────────────────

/// Dispersion curve: velocity as a function of period.
#[derive(Debug, Clone)]
pub struct DispersionCurve {
    /// Periods in seconds (sorted ascending).
    pub period_s: Vec<f64>,
    /// Velocities in m/s at each period.
    pub velocity_ms: Vec<f64>,
    /// SNR at each period (optional quality metric).
    pub snr: Vec<f64>,
}

impl DispersionCurve {
    /// Create a new dispersion curve.
    pub fn new(period_s: Vec<f64>, velocity_ms: Vec<f64>, snr: Vec<f64>) -> Self {
        Self {
            period_s,
            velocity_ms,
            snr,
        }
    }

    /// Number of measurements.
    pub fn len(&self) -> usize {
        self.period_s.len()
    }

    /// Whether curve is empty.
    pub fn is_empty(&self) -> bool {
        self.period_s.is_empty()
    }

    /// Interpolate velocity at a given period using linear interpolation.
    pub fn interpolate(&self, period: f64) -> Option<f64> {
        if self.period_s.len() < 2 {
            return None;
        }
        if period < self.period_s[0] || period > *self.period_s.last().unwrap() {
            return None;
        }
        for i in 0..self.period_s.len() - 1 {
            if period >= self.period_s[i] && period <= self.period_s[i + 1] {
                let t =
                    (period - self.period_s[i]) / (self.period_s[i + 1] - self.period_s[i]);
                let v = self.velocity_ms[i]
                    + t * (self.velocity_ms[i + 1] - self.velocity_ms[i]);
                return Some(v);
            }
        }
        None
    }

    /// Smooth the dispersion curve using a moving average of given window size.
    pub fn smooth(&self, window: usize) -> DispersionCurve {
        let n = self.velocity_ms.len();
        if window < 2 || n < window {
            return self.clone();
        }
        let half = window / 2;
        let mut smoothed = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = if i + half < n { i + half } else { n - 1 };
            let count = (hi - lo + 1) as f64;
            let sum: f64 = self.velocity_ms[lo..=hi].iter().sum();
            smoothed[i] = sum / count;
        }
        DispersionCurve {
            period_s: self.period_s.clone(),
            velocity_ms: smoothed,
            snr: self.snr.clone(),
        }
    }

    /// Fit a polynomial of given degree and return fitted velocities.
    pub fn polynomial_fit(&self, degree: usize) -> Vec<f64> {
        polynomial_fit(&self.period_s, &self.velocity_ms, degree)
    }
}

// ─── Layer Model ─────────────────────────────────────────────────────────────

/// 1D layered velocity model.
#[derive(Debug, Clone)]
pub struct LayerModel {
    /// Depth to top of each layer (meters), starting at 0.
    pub depth_m: Vec<f64>,
    /// Shear-wave velocity in each layer (m/s).
    pub vs_ms: Vec<f64>,
}

impl LayerModel {
    /// Create a new layer model.
    pub fn new(depth_m: Vec<f64>, vs_ms: Vec<f64>) -> Self {
        Self { depth_m, vs_ms }
    }

    /// Get velocity at a given depth by finding enclosing layer.
    pub fn velocity_at_depth(&self, depth: f64) -> f64 {
        for i in (0..self.depth_m.len()).rev() {
            if depth >= self.depth_m[i] {
                return self.vs_ms[i];
            }
        }
        self.vs_ms[0]
    }

    /// Number of layers.
    pub fn num_layers(&self) -> usize {
        self.depth_m.len()
    }
}

// ─── Velocity Grid ───────────────────────────────────────────────────────────

/// 2D velocity perturbation grid for tomographic inversion.
#[derive(Debug, Clone)]
pub struct VelocityGrid {
    /// Number of cells in x direction.
    pub nx: usize,
    /// Number of cells in y direction.
    pub ny: usize,
    /// Cell size in x (meters).
    pub dx: f64,
    /// Cell size in y (meters).
    pub dy: f64,
    /// Origin x coordinate (meters).
    pub origin_x: f64,
    /// Origin y coordinate (meters).
    pub origin_y: f64,
    /// Velocity values (m/s), row-major [ny][nx].
    pub velocity: Vec<f64>,
}

impl VelocityGrid {
    /// Create a new grid with uniform velocity.
    pub fn new_uniform(
        nx: usize,
        ny: usize,
        dx: f64,
        dy: f64,
        origin_x: f64,
        origin_y: f64,
        v0: f64,
    ) -> Self {
        Self {
            nx,
            ny,
            dx,
            dy,
            origin_x,
            origin_y,
            velocity: vec![v0; nx * ny],
        }
    }

    /// Get velocity at grid index (ix, iy).
    pub fn get(&self, ix: usize, iy: usize) -> f64 {
        self.velocity[iy * self.nx + ix]
    }

    /// Set velocity at grid index (ix, iy).
    pub fn set(&mut self, ix: usize, iy: usize, val: f64) {
        self.velocity[iy * self.nx + ix] = val;
    }

    /// Get grid cell center coordinates.
    pub fn cell_center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (
            self.origin_x + (ix as f64 + 0.5) * self.dx,
            self.origin_y + (iy as f64 + 0.5) * self.dy,
        )
    }

    /// Find grid cell indices for a given (x, y) coordinate.
    pub fn find_cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        let ix = ((x - self.origin_x) / self.dx).floor() as isize;
        let iy = ((y - self.origin_y) / self.dy).floor() as isize;
        if ix >= 0 && ix < self.nx as isize && iy >= 0 && iy < self.ny as isize {
            Some((ix as usize, iy as usize))
        } else {
            None
        }
    }

    /// Total number of cells.
    pub fn num_cells(&self) -> usize {
        self.nx * self.ny
    }
}

// ─── Cross-Correlation Result ────────────────────────────────────────────────

/// Result of noise cross-correlation.
#[derive(Debug, Clone)]
pub struct CrossCorrelationResult {
    /// Lag times in seconds (symmetric around 0).
    pub lag_times: Vec<f64>,
    /// Cross-correlation amplitudes.
    pub correlation: Vec<f64>,
    /// Number of segments stacked.
    pub num_stacks: usize,
    /// SNR of the cross-correlation.
    pub snr: f64,
}

impl CrossCorrelationResult {
    /// Get the symmetric (Green's function) component: G(t) = [C(t) + C(-t)] / 2.
    pub fn symmetric_component(&self) -> Vec<f64> {
        let n = self.correlation.len();
        let mid = n / 2;
        let half_len = mid + 1;
        let mut sym = vec![0.0; half_len];
        for i in 0..half_len {
            let pos = mid + i;
            let neg = if mid >= i { mid - i } else { 0 };
            if pos < n {
                sym[i] = (self.correlation[pos] + self.correlation[neg]) / 2.0;
            }
        }
        sym
    }

    /// Get the causal part (positive lags).
    pub fn causal(&self) -> Vec<f64> {
        let mid = self.correlation.len() / 2;
        self.correlation[mid..].to_vec()
    }

    /// Get the acausal part (negative lags, reversed to positive time).
    pub fn acausal(&self) -> Vec<f64> {
        let mid = self.correlation.len() / 2;
        let mut ac = self.correlation[..=mid].to_vec();
        ac.reverse();
        ac
    }

    /// Peak amplitude in the cross-correlation.
    pub fn peak_amplitude(&self) -> f64 {
        self.correlation
            .iter()
            .fold(0.0_f64, |a, &b| a.max(b.abs()))
    }
}

// ─── Green's Function ────────────────────────────────────────────────────────

/// Extracted empirical Green's function.
#[derive(Debug, Clone)]
pub struct GreenFunction {
    /// Time samples (seconds, starting from 0).
    pub time_s: Vec<f64>,
    /// Green's function amplitude.
    pub amplitude: Vec<f64>,
    /// Convergence metric (normalized difference of running stacks).
    pub convergence: f64,
}

// ─── Tomography Result ───────────────────────────────────────────────────────

/// Result of 2D tomographic inversion.
#[derive(Debug, Clone)]
pub struct TomographyResult {
    /// Inverted velocity grid.
    pub velocity_grid: VelocityGrid,
    /// Residual norm after inversion.
    pub residual_norm: f64,
    /// Number of iterations used.
    pub iterations: usize,
}

// ─── Ambient Noise Processor ─────────────────────────────────────────────────

/// Main processor for ambient noise cross-correlation and tomography.
#[derive(Debug, Clone)]
pub struct AmbientNoiseProcessor {
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Segment length in seconds for windowed processing.
    pub segment_length_s: f64,
    /// Overlap fraction between segments (0.0 to 1.0).
    pub overlap_fraction: f64,
    /// Frequency band for analysis.
    pub frequency_band: FrequencyBand,
    /// Maximum lag time in seconds for cross-correlation.
    pub max_lag_s: f64,
}

impl AmbientNoiseProcessor {
    /// Create a new ambient noise processor.
    pub fn new(
        sample_rate_hz: f64,
        segment_length_s: f64,
        overlap_fraction: f64,
        frequency_band: FrequencyBand,
        max_lag_s: f64,
    ) -> Self {
        Self {
            sample_rate_hz,
            segment_length_s,
            overlap_fraction,
            frequency_band,
            max_lag_s,
        }
    }

    /// Segment length in samples.
    pub fn segment_samples(&self) -> usize {
        (self.segment_length_s * self.sample_rate_hz) as usize
    }

    /// Maximum lag in samples.
    pub fn max_lag_samples(&self) -> usize {
        (self.max_lag_s * self.sample_rate_hz) as usize
    }

    /// Segment data into overlapping windows.
    pub fn segment_data(&self, data: &[f64]) -> Vec<Vec<f64>> {
        let seg_len = self.segment_samples();
        let hop = ((1.0 - self.overlap_fraction) * seg_len as f64) as usize;
        let hop = hop.max(1);
        let mut segments = Vec::new();
        let mut start = 0;
        while start + seg_len <= data.len() {
            segments.push(data[start..start + seg_len].to_vec());
            start += hop;
        }
        segments
    }

    /// Cross-correlate two continuous noise traces with stacking.
    pub fn cross_correlate(
        &self,
        trace_a: &[f64],
        trace_b: &[f64],
    ) -> CrossCorrelationResult {
        let segs_a = self.segment_data(trace_a);
        let segs_b = self.segment_data(trace_b);
        let num_segs = segs_a.len().min(segs_b.len());
        let max_lag = self.max_lag_samples();
        let corr_len = 2 * max_lag + 1;
        let mut stack = vec![0.0; corr_len];
        let mut count = 0usize;

        for i in 0..num_segs {
            // Preprocess each segment
            let a = self.preprocess_segment(&segs_a[i]);
            let b = self.preprocess_segment(&segs_b[i]);

            // Compute cross-correlation via direct method
            let cc = cross_correlate_direct(&a, &b, max_lag);
            for j in 0..corr_len.min(cc.len()) {
                stack[j] += cc[j];
            }
            count += 1;
        }

        // Average the stack
        if count > 0 {
            let inv = 1.0 / count as f64;
            for v in stack.iter_mut() {
                *v *= inv;
            }
        }

        // Build lag times
        let dt = 1.0 / self.sample_rate_hz;
        let lag_times: Vec<f64> = (0..corr_len)
            .map(|i| (i as f64 - max_lag as f64) * dt)
            .collect();

        let snr = compute_snr(&stack, max_lag);

        CrossCorrelationResult {
            lag_times,
            correlation: stack,
            num_stacks: count,
            snr,
        }
    }

    /// Preprocess a segment: bandpass, one-bit normalization, spectral whitening.
    pub fn preprocess_segment(&self, data: &[f64]) -> Vec<f64> {
        let mut result = data.to_vec();

        // Bandpass filter
        result = bandpass_filter(
            &result,
            self.sample_rate_hz,
            self.frequency_band.min_hz,
            self.frequency_band.max_hz,
        );

        // Temporal normalization: one-bit
        result = one_bit_normalize(&result);

        // Spectral whitening
        result = spectral_whiten(&result);

        result
    }

    /// Extract empirical Green's function from cross-correlation.
    pub fn extract_green_function(
        &self,
        xcorr: &CrossCorrelationResult,
    ) -> GreenFunction {
        let sym = xcorr.symmetric_component();
        let dt = 1.0 / self.sample_rate_hz;
        let time_s: Vec<f64> = (0..sym.len()).map(|i| i as f64 * dt).collect();

        // Convergence: compare first half stack to full (simplified)
        let convergence = if sym.len() > 2 {
            let half_energy: f64 =
                sym[..sym.len() / 2].iter().map(|x| x * x).sum();
            let full_energy: f64 = sym.iter().map(|x| x * x).sum();
            if full_energy > 0.0 {
                (half_energy / full_energy - 0.5).abs()
            } else {
                1.0
            }
        } else {
            1.0
        };

        GreenFunction {
            time_s,
            amplitude: sym,
            convergence,
        }
    }

    /// Measure group velocity dispersion curve using Frequency-Time Analysis (FTAN).
    pub fn measure_group_velocity(
        &self,
        green: &GreenFunction,
        distance_m: f64,
        center_periods: &[f64],
    ) -> DispersionCurve {
        let mut periods = Vec::new();
        let mut velocities = Vec::new();
        let mut snrs = Vec::new();

        for &period in center_periods {
            let freq = 1.0 / period;
            if freq < self.frequency_band.min_hz || freq > self.frequency_band.max_hz {
                continue;
            }

            // Gaussian narrow-band filter
            let alpha = 4.0; // Gaussian filter width parameter
            let filtered =
                gaussian_narrowband_filter(&green.amplitude, self.sample_rate_hz, freq, alpha);

            // Compute envelope (analytic signal via Hilbert)
            let envelope = compute_envelope(&filtered);

            // Find peak of envelope (group arrival)
            let min_sample = (distance_m / 5000.0 * self.sample_rate_hz) as usize; // max 5 km/s
            let max_sample =
                (distance_m / 500.0 * self.sample_rate_hz) as usize; // min 0.5 km/s

            let search_start = min_sample.min(envelope.len().saturating_sub(1));
            let search_end = max_sample.min(envelope.len());

            if search_start >= search_end {
                continue;
            }

            let mut peak_idx = search_start;
            let mut peak_val = envelope[search_start];
            for i in search_start..search_end {
                if envelope[i] > peak_val {
                    peak_val = envelope[i];
                    peak_idx = i;
                }
            }

            let t_group = peak_idx as f64 / self.sample_rate_hz;
            if t_group > 0.0 {
                let u = distance_m / t_group;
                // SNR: peak / rms of tails
                let noise_rms = rms_excluding_peak(&envelope, peak_idx, 10);
                let snr = if noise_rms > 0.0 {
                    peak_val / noise_rms
                } else {
                    0.0
                };

                periods.push(period);
                velocities.push(u);
                snrs.push(snr);
            }
        }

        DispersionCurve::new(periods, velocities, snrs)
    }

    /// Measure phase velocity using zero-crossing method on filtered waveform.
    pub fn measure_phase_velocity(
        &self,
        green: &GreenFunction,
        distance_m: f64,
        center_periods: &[f64],
    ) -> DispersionCurve {
        let mut periods = Vec::new();
        let mut velocities = Vec::new();
        let mut snrs = Vec::new();

        for &period in center_periods {
            let freq = 1.0 / period;
            if freq < self.frequency_band.min_hz || freq > self.frequency_band.max_hz {
                continue;
            }

            let alpha = 4.0;
            let filtered =
                gaussian_narrowband_filter(&green.amplitude, self.sample_rate_hz, freq, alpha);

            // Count zero crossings in the expected travel-time window
            let expected_time = distance_m / 3000.0; // rough estimate at 3 km/s
            let window_start =
                ((expected_time * 0.5) * self.sample_rate_hz) as usize;
            let window_end =
                ((expected_time * 2.0) * self.sample_rate_hz) as usize;

            let ws = window_start.min(filtered.len().saturating_sub(2));
            let we = window_end.min(filtered.len().saturating_sub(1));

            if ws >= we {
                continue;
            }

            // Count positive-going zero crossings
            let mut crossings = Vec::new();
            for i in ws..we {
                if filtered[i] <= 0.0 && filtered[i + 1] > 0.0 {
                    // Linear interpolation for fractional crossing
                    let frac = -filtered[i] / (filtered[i + 1] - filtered[i]);
                    crossings.push(i as f64 + frac);
                }
            }

            if crossings.len() >= 2 {
                // Average crossing interval gives the period
                let total_crossings = crossings.len() as f64 - 1.0;
                let total_time =
                    (crossings.last().unwrap() - crossings[0]) / self.sample_rate_hz;
                let measured_period = total_time / total_crossings;

                // Phase velocity: c = distance * freq / (n + phi/(2pi))
                // Simplified: use the center of the crossing pattern
                let center_time = (crossings[0]
                    + (crossings.last().unwrap() - crossings[0]) / 2.0)
                    / self.sample_rate_hz;
                if center_time > 0.0 {
                    let c = distance_m / (center_time - measured_period / 2.0).max(1e-10);
                    // Only keep reasonable velocities
                    if c > 200.0 && c < 10000.0 {
                        let envelope = compute_envelope(&filtered);
                        let peak = envelope.iter().fold(0.0_f64, |a, &b| a.max(b));
                        let noise = rms_excluding_peak(&envelope, 0, 10);
                        let snr = if noise > 0.0 { peak / noise } else { 0.0 };

                        periods.push(period);
                        velocities.push(c);
                        snrs.push(snr);
                    }
                }
            }
        }

        DispersionCurve::new(periods, velocities, snrs)
    }

    /// 1D velocity inversion from dispersion curve using sensitivity kernels.
    pub fn invert_1d(
        &self,
        dispersion: &DispersionCurve,
        num_layers: usize,
        max_depth_m: f64,
        ref_velocity_ms: f64,
    ) -> LayerModel {
        let layer_thickness = max_depth_m / num_layers as f64;
        let depths: Vec<f64> = (0..num_layers)
            .map(|i| i as f64 * layer_thickness)
            .collect();
        let mut velocities = vec![ref_velocity_ms; num_layers];

        // Build sensitivity kernel matrix G and data vector d
        // d = (c_measured - c_ref) / c_ref for each period
        // G[i][j] = partial(c_i) / partial(v_j) * v_j / c_i (normalized kernel)
        let n_data = dispersion.period_s.len();
        if n_data == 0 {
            return LayerModel::new(depths, velocities);
        }

        let mut g_matrix = vec![vec![0.0; num_layers]; n_data];
        let mut d_vec = vec![0.0; n_data];

        for (i, &period) in dispersion.period_s.iter().enumerate() {
            let wavelength = ref_velocity_ms * period;
            // Rayleigh wave sensitivity peaks at ~lambda/3 depth
            let peak_depth = wavelength / 3.0;

            for j in 0..num_layers {
                let z = depths[j] + layer_thickness / 2.0;
                // Simplified Rayleigh wave depth sensitivity kernel
                // Gaussian-like sensitivity centered at lambda/3
                let sigma = wavelength / 4.0;
                if sigma > 0.0 {
                    let kernel =
                        (-(z - peak_depth).powi(2) / (2.0 * sigma * sigma)).exp();
                    g_matrix[i][j] = kernel;
                }
            }

            // Normalize kernel rows
            let row_sum: f64 = g_matrix[i].iter().sum();
            if row_sum > 0.0 {
                for j in 0..num_layers {
                    g_matrix[i][j] /= row_sum;
                }
            }

            d_vec[i] = (dispersion.velocity_ms[i] - ref_velocity_ms) / ref_velocity_ms;
        }

        // Solve via damped least squares: (G^T G + lambda I) m = G^T d
        let damping = 0.1;
        let mut gtg = vec![vec![0.0; num_layers]; num_layers];
        let mut gtd = vec![0.0; num_layers];

        for j in 0..num_layers {
            for k in 0..num_layers {
                for i in 0..n_data {
                    gtg[j][k] += g_matrix[i][j] * g_matrix[i][k];
                }
                if j == k {
                    gtg[j][k] += damping;
                }
            }
            for i in 0..n_data {
                gtd[j] += g_matrix[i][j] * d_vec[i];
            }
        }

        // Solve with simple Gauss elimination
        let m = solve_linear_system(&gtg, &gtd);

        // Apply perturbations
        for j in 0..num_layers {
            velocities[j] = ref_velocity_ms * (1.0 + m[j]);
        }

        LayerModel::new(depths, velocities)
    }

    /// 2D surface wave tomography using straight-ray inversion.
    pub fn invert_2d(
        &self,
        station_pairs: &[StationPair],
        travel_times: &[f64],
        grid: &mut VelocityGrid,
        damping: f64,
        smoothing: f64,
        max_iterations: usize,
    ) -> TomographyResult {
        let n_data = station_pairs.len();
        let n_cells = grid.num_cells();

        // Reference slowness (1/velocity)
        let ref_slowness: Vec<f64> = grid.velocity.iter().map(|&v| 1.0 / v).collect();

        // Build ray-path matrix: G[i][j] = path length of ray i through cell j
        let mut g_matrix = vec![vec![0.0; n_cells]; n_data];
        for (i, pair) in station_pairs.iter().enumerate() {
            trace_ray(pair, grid, &mut g_matrix[i]);
        }

        // Data vector: travel time residuals
        let mut d_vec = vec![0.0; n_data];
        for i in 0..n_data {
            // Predicted travel time from current model
            let predicted: f64 = g_matrix[i]
                .iter()
                .zip(ref_slowness.iter())
                .map(|(&g, &s)| g * s)
                .sum();
            d_vec[i] = travel_times[i] - predicted;
        }

        // LSQR-style iterative solver (simplified conjugate gradient)
        let mut model_update = vec![0.0; n_cells];
        let residual_norm =
            conjugate_gradient_solve(&g_matrix, &d_vec, &mut model_update, damping, smoothing, grid.nx, grid.ny, max_iterations);

        // Apply slowness perturbations
        for j in 0..n_cells {
            let new_slowness = ref_slowness[j] + model_update[j];
            if new_slowness > 0.0 {
                grid.velocity[j] = 1.0 / new_slowness;
            }
        }

        TomographyResult {
            velocity_grid: grid.clone(),
            residual_norm,
            iterations: max_iterations,
        }
    }

    /// Perform checkerboard resolution test.
    pub fn checkerboard_test(
        &self,
        station_pairs: &[StationPair],
        grid: &VelocityGrid,
        perturbation_pct: f64,
        checker_size: usize,
        damping: f64,
        smoothing: f64,
        max_iterations: usize,
    ) -> VelocityGrid {
        // Create checkerboard pattern
        let mut true_grid = grid.clone();
        for iy in 0..grid.ny {
            for ix in 0..grid.nx {
                let checker = ((ix / checker_size) + (iy / checker_size)) % 2;
                let sign = if checker == 0 { 1.0 } else { -1.0 };
                let v0 = grid.get(ix, iy);
                true_grid.set(ix, iy, v0 * (1.0 + sign * perturbation_pct / 100.0));
            }
        }

        // Compute synthetic travel times
        let mut travel_times = vec![0.0; station_pairs.len()];
        for (i, pair) in station_pairs.iter().enumerate() {
            let mut path = vec![0.0; grid.num_cells()];
            trace_ray(pair, &true_grid, &mut path);
            travel_times[i] = path
                .iter()
                .zip(true_grid.velocity.iter())
                .map(|(&g, &v)| if v > 0.0 { g / v } else { 0.0 })
                .sum();
        }

        // Invert starting from uniform model
        let mut result_grid = grid.clone();
        self.invert_2d(
            station_pairs,
            &travel_times,
            &mut result_grid,
            damping,
            smoothing,
            max_iterations,
        );

        result_grid
    }

    /// Estimate SNR of a cross-correlation.
    pub fn estimate_snr(&self, correlation: &[f64]) -> f64 {
        if correlation.is_empty() {
            return 0.0;
        }
        let mid = correlation.len() / 2;
        compute_snr(correlation, mid)
    }
}

// ─── Preprocessing Functions ─────────────────────────────────────────────────

/// One-bit normalization: sign(x(t)).
pub fn one_bit_normalize(data: &[f64]) -> Vec<f64> {
    data.iter()
        .map(|&x| {
            if x > 0.0 {
                1.0
            } else if x < 0.0 {
                -1.0
            } else {
                0.0
            }
        })
        .collect()
}

/// Running absolute mean normalization.
pub fn running_absolute_mean_normalize(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if n == 0 || window == 0 {
        return data.to_vec();
    }
    let half = window / 2;
    let mut result = vec![0.0; n];
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = if i + half < n { i + half } else { n - 1 };
        let count = (hi - lo + 1) as f64;
        let mean_abs: f64 = data[lo..=hi].iter().map(|x| x.abs()).sum::<f64>() / count;
        if mean_abs > 1e-30 {
            result[i] = data[i] / mean_abs;
        }
    }
    result
}

/// Spectral whitening: divide FFT by its amplitude spectrum (phase-only).
pub fn spectral_whiten(data: &[f64]) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }
    // Compute FFT
    let (mut re, mut im) = fft_real(data);
    let nf = re.len();

    // Divide by amplitude
    for i in 0..nf {
        let amp = (re[i] * re[i] + im[i] * im[i]).sqrt();
        if amp > 1e-30 {
            re[i] /= amp;
            im[i] /= amp;
        } else {
            re[i] = 0.0;
            im[i] = 0.0;
        }
    }

    // Inverse FFT
    ifft_to_real(&re, &im, n)
}

/// Bandpass filter using windowed-sinc FIR.
pub fn bandpass_filter(data: &[f64], fs: f64, f_low: f64, f_high: f64) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }

    // Design bandpass FIR with Hamming window
    let order = 63; // filter order (odd for type I FIR)
    let half = order / 2;
    let mut h = vec![0.0; order + 1];

    let fc_low = f_low / fs;
    let fc_high = f_high / fs;

    for i in 0..=order {
        let m = i as f64 - half as f64;
        // Bandpass = highpass - lowpass (highpass at f_low, lowpass at f_high)
        let sinc_high = if m.abs() < 1e-10 {
            2.0 * fc_high
        } else {
            (2.0 * PI * fc_high * m).sin() / (PI * m)
        };
        let sinc_low = if m.abs() < 1e-10 {
            2.0 * fc_low
        } else {
            (2.0 * PI * fc_low * m).sin() / (PI * m)
        };
        // Hamming window
        let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / order as f64).cos();
        h[i] = (sinc_high - sinc_low) * w;
    }

    // Convolve
    convolve(data, &h)
}

/// Instrument response removal via water-level deconvolution.
pub fn deconvolve_instrument(
    data: &[f64],
    response_re: &[f64],
    response_im: &[f64],
    water_level_db: f64,
) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }

    let (sig_re, sig_im) = fft_real(data);
    let nf = sig_re.len();

    let water_level = 10.0_f64.powf(water_level_db / 20.0);

    let mut out_re = vec![0.0; nf];
    let mut out_im = vec![0.0; nf];

    for i in 0..nf {
        let hr = if i < response_re.len() {
            response_re[i]
        } else {
            1.0
        };
        let hi = if i < response_im.len() {
            response_im[i]
        } else {
            0.0
        };
        let amp_sq = hr * hr + hi * hi;
        let amp = amp_sq.sqrt();

        // Water-level stabilization
        let denom = if amp > water_level {
            amp_sq
        } else {
            water_level * water_level
        };

        // Complex division: S / H = S * conj(H) / |H|^2
        out_re[i] = (sig_re[i] * hr + sig_im[i] * hi) / denom;
        out_im[i] = (sig_im[i] * hr - sig_re[i] * hi) / denom;
    }

    ifft_to_real(&out_re, &out_im, n)
}

// ─── Signal Processing Helpers ───────────────────────────────────────────────

/// Gaussian narrow-band filter centered at `center_freq` Hz.
fn gaussian_narrowband_filter(
    data: &[f64],
    sample_rate: f64,
    center_freq: f64,
    alpha: f64,
) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }

    let (mut re, mut im) = fft_real(data);
    let nf = re.len();

    for i in 0..nf {
        let freq = i as f64 * sample_rate / (nf as f64 * 2.0);
        let exponent = -alpha * ((freq - center_freq) / center_freq).powi(2);
        let gain = exponent.exp();
        re[i] *= gain;
        im[i] *= gain;
    }

    ifft_to_real(&re, &im, n)
}

/// Compute the envelope (magnitude of analytic signal) via Hilbert transform.
pub fn compute_envelope(data: &[f64]) -> Vec<f64> {
    let n = data.len();
    if n == 0 {
        return vec![];
    }

    let (re, im) = fft_real(data);
    let nf = re.len();

    // Build analytic signal: zero negative frequencies, double positive
    let mut a_re = vec![0.0; nf];
    let mut a_im = vec![0.0; nf];

    // DC component unchanged
    a_re[0] = re[0];
    a_im[0] = im[0];

    // Positive frequencies doubled
    for i in 1..nf {
        a_re[i] = 2.0 * re[i];
        a_im[i] = 2.0 * im[i];
    }

    // Nyquist bin (if even length)
    if n % 2 == 0 && nf > 1 {
        a_re[nf - 1] = re[nf - 1];
        a_im[nf - 1] = im[nf - 1];
    }

    let real_part = ifft_to_real(&a_re, &a_im, n);
    let imag_part = ifft_to_real_imag(&a_re, &a_im, n);

    real_part
        .iter()
        .zip(imag_part.iter())
        .map(|(&r, &i)| (r * r + i * i).sqrt())
        .collect()
}

/// Direct cross-correlation with maximum lag.
fn cross_correlate_direct(a: &[f64], b: &[f64], max_lag: usize) -> Vec<f64> {
    let n = a.len().min(b.len());
    let corr_len = 2 * max_lag + 1;
    let mut result = vec![0.0; corr_len];

    for (lag_idx, lag_signed) in (0..corr_len)
        .map(|i| (i, i as isize - max_lag as isize))
    {
        let mut sum = 0.0;
        let mut count = 0usize;
        for j in 0..n {
            let k = j as isize + lag_signed;
            if k >= 0 && (k as usize) < n {
                sum += a[j] * b[k as usize];
                count += 1;
            }
        }
        if count > 0 {
            result[lag_idx] = sum / count as f64;
        }
    }

    result
}

/// Compute SNR: peak amplitude / RMS of noise region.
fn compute_snr(correlation: &[f64], mid: usize) -> f64 {
    let peak = correlation.iter().fold(0.0_f64, |a, &b| a.max(b.abs()));
    let n = correlation.len();
    // Noise region: exclude central 20% around zero lag
    let exclude_half = n / 10;
    let noise_start = 0;
    let noise_end_left = if mid > exclude_half {
        mid - exclude_half
    } else {
        0
    };
    let noise_start_right = (mid + exclude_half).min(n);

    let mut noise_sum_sq = 0.0;
    let mut noise_count = 0;
    for i in noise_start..noise_end_left {
        noise_sum_sq += correlation[i] * correlation[i];
        noise_count += 1;
    }
    for i in noise_start_right..n {
        noise_sum_sq += correlation[i] * correlation[i];
        noise_count += 1;
    }

    if noise_count > 0 {
        let noise_rms = (noise_sum_sq / noise_count as f64).sqrt();
        if noise_rms > 1e-30 {
            return peak / noise_rms;
        }
    }
    0.0
}

/// RMS of envelope excluding a region around the peak.
fn rms_excluding_peak(envelope: &[f64], peak_idx: usize, exclude_half: usize) -> f64 {
    let n = envelope.len();
    let lo = peak_idx.saturating_sub(exclude_half);
    let hi = (peak_idx + exclude_half + 1).min(n);

    let mut sum_sq = 0.0;
    let mut count = 0;
    for i in 0..n {
        if i < lo || i >= hi {
            sum_sq += envelope[i] * envelope[i];
            count += 1;
        }
    }
    if count > 0 {
        (sum_sq / count as f64).sqrt()
    } else {
        0.0
    }
}

/// Convolution of two signals (full output, truncated to input length).
fn convolve(data: &[f64], kernel: &[f64]) -> Vec<f64> {
    let n = data.len();
    let m = kernel.len();
    let half = m / 2;
    let mut result = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..m {
            let idx = i as isize + j as isize - half as isize;
            if idx >= 0 && (idx as usize) < n {
                sum += data[idx as usize] * kernel[j];
            }
        }
        result[i] = sum;
    }
    result
}

// ─── FFT (radix-2 Cooley-Tukey) ─────────────────────────────────────────────

/// Find next power of 2 >= n.
fn next_pow2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Bit-reverse permutation.
fn bit_reverse(x: usize, bits: u32) -> usize {
    let mut result = 0;
    let mut val = x;
    for _ in 0..bits {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

/// In-place radix-2 FFT. Input must be power-of-2 length.
fn fft_inplace(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    let bits = (n as f64).log2() as u32;

    // Bit-reverse permutation
    for i in 0..n {
        let j = bit_reverse(i, bits);
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let sign = if inverse { 1.0 } else { -1.0 };
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();

        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for j in 0..half {
                let a = start + j;
                let b = start + j + half;
                let t_re = w_re * re[b] - w_im * im[b];
                let t_im = w_re * im[b] + w_im * re[b];
                re[b] = re[a] - t_re;
                im[b] = im[a] - t_im;
                re[a] += t_re;
                im[a] += t_im;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for i in 0..n {
            re[i] *= inv_n;
            im[i] *= inv_n;
        }
    }
}

/// FFT of a real signal, returns (real, imag) of non-negative frequencies.
fn fft_real(data: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = next_pow2(data.len());
    let mut re = vec![0.0; n];
    let mut im = vec![0.0; n];
    for (i, &v) in data.iter().enumerate() {
        re[i] = v;
    }
    fft_inplace(&mut re, &mut im, false);
    let nf = n / 2 + 1;
    (re[..nf].to_vec(), im[..nf].to_vec())
}

/// Inverse FFT from non-negative frequency bins back to real signal.
fn ifft_to_real(re: &[f64], im: &[f64], orig_len: usize) -> Vec<f64> {
    let nf = re.len();
    let n = (nf - 1) * 2;
    let mut full_re = vec![0.0; n];
    let mut full_im = vec![0.0; n];

    for i in 0..nf {
        full_re[i] = re[i];
        full_im[i] = im[i];
    }
    // Mirror for negative frequencies
    for i in 1..nf - 1 {
        full_re[n - i] = re[i];
        full_im[n - i] = -im[i];
    }

    fft_inplace(&mut full_re, &mut full_im, true);
    full_re.truncate(orig_len);
    full_re
}

/// Inverse FFT imaginary part (for Hilbert transform).
fn ifft_to_real_imag(re: &[f64], im: &[f64], orig_len: usize) -> Vec<f64> {
    let nf = re.len();
    let n = (nf - 1) * 2;
    let mut full_re = vec![0.0; n];
    let mut full_im = vec![0.0; n];

    for i in 0..nf {
        full_re[i] = re[i];
        full_im[i] = im[i];
    }
    for i in 1..nf - 1 {
        full_re[n - i] = re[i];
        full_im[n - i] = -im[i];
    }

    fft_inplace(&mut full_re, &mut full_im, true);
    full_im.truncate(orig_len);
    full_im
}

// ─── Linear Algebra Helpers ──────────────────────────────────────────────────

/// Solve linear system Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    // Augmented matrix
    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = a[i].clone();
        row.push(b[i]);
        aug.push(row);
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in col + 1..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in col + 1..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in i + 1..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() > 1e-30 {
            x[i] = sum / aug[i][i];
        }
    }
    x
}

/// Polynomial fit of degree `deg` to (x, y) data.
fn polynomial_fit(x: &[f64], y: &[f64], deg: usize) -> Vec<f64> {
    let n = x.len();
    if n == 0 {
        return vec![];
    }
    let m = deg + 1;

    // Build normal equations: A^T A c = A^T y
    let mut ata = vec![vec![0.0; m]; m];
    let mut aty = vec![0.0; m];

    for i in 0..n {
        let mut xi_pow = vec![1.0; m];
        for j in 1..m {
            xi_pow[j] = xi_pow[j - 1] * x[i];
        }
        for j in 0..m {
            for k in 0..m {
                ata[j][k] += xi_pow[j] * xi_pow[k];
            }
            aty[j] += xi_pow[j] * y[i];
        }
    }

    let coeffs = solve_linear_system(&ata, &aty);

    // Evaluate fitted values
    let mut fitted = vec![0.0; n];
    for i in 0..n {
        let mut val = 0.0;
        let mut xi_pow = 1.0;
        for c in &coeffs {
            val += c * xi_pow;
            xi_pow *= x[i];
        }
        fitted[i] = val;
    }
    fitted
}

// ─── Ray Tracing for 2D Tomography ──────────────────────────────────────────

/// Trace a straight ray through the velocity grid, computing path length in each cell.
fn trace_ray(pair: &StationPair, grid: &VelocityGrid, path_lengths: &mut [f64]) {
    // Simple straight-ray approximation using uniform sampling along the ray
    let n_cells = grid.num_cells();
    for v in path_lengths.iter_mut().take(n_cells) {
        *v = 0.0;
    }

    let dist = pair.inter_station_distance();
    if dist < 1e-6 {
        return;
    }

    let n_samples = ((dist / grid.dx.min(grid.dy)) * 2.0) as usize;
    let n_samples = n_samples.max(100);
    let ds = dist / n_samples as f64;

    let dx_norm = (pair.station_b_x - pair.station_a_x) / dist;
    let dy_norm = (pair.station_b_y - pair.station_a_y) / dist;

    for i in 0..=n_samples {
        let x = pair.station_a_x + dx_norm * ds * i as f64;
        let y = pair.station_a_y + dy_norm * ds * i as f64;

        if let Some((ix, iy)) = grid.find_cell(x, y) {
            let idx = iy * grid.nx + ix;
            if idx < n_cells {
                path_lengths[idx] += ds;
            }
        }
    }
}

/// Simplified conjugate gradient solver for the tomographic inverse problem.
/// Solves (G^T G + damping*I + smoothing*L) m = G^T d
fn conjugate_gradient_solve(
    g: &[Vec<f64>],
    d: &[f64],
    m: &mut [f64],
    damping: f64,
    smoothing: f64,
    nx: usize,
    ny: usize,
    max_iter: usize,
) -> f64 {
    let n_data = d.len();
    let n_model = m.len();

    // Compute G^T d
    let mut gtd = vec![0.0; n_model];
    for j in 0..n_model {
        for i in 0..n_data {
            gtd[j] += g[i][j] * d[i];
        }
    }

    // CG iteration
    // r = gtd - A*m where A = G^T G + damping*I + smoothing*L
    let apply_a = |x: &[f64]| -> Vec<f64> {
        let mut result = vec![0.0; n_model];
        // G^T (G x)
        let mut gx = vec![0.0; n_data];
        for i in 0..n_data {
            for j in 0..n_model {
                gx[i] += g[i][j] * x[j];
            }
        }
        for j in 0..n_model {
            for i in 0..n_data {
                result[j] += g[i][j] * gx[i];
            }
            // Damping
            result[j] += damping * x[j];
        }
        // Smoothing Laplacian
        if smoothing > 0.0 {
            for iy in 0..ny {
                for ix in 0..nx {
                    let idx = iy * nx + ix;
                    let mut laplacian = -4.0 * x[idx];
                    if ix > 0 {
                        laplacian += x[idx - 1];
                    } else {
                        laplacian += x[idx];
                    }
                    if ix + 1 < nx {
                        laplacian += x[idx + 1];
                    } else {
                        laplacian += x[idx];
                    }
                    if iy > 0 {
                        laplacian += x[idx - nx];
                    } else {
                        laplacian += x[idx];
                    }
                    if iy + 1 < ny {
                        laplacian += x[idx + nx];
                    } else {
                        laplacian += x[idx];
                    }
                    result[idx] -= smoothing * laplacian;
                }
            }
        }
        result
    };

    // Initial residual
    let am = apply_a(m);
    let mut r: Vec<f64> = gtd.iter().zip(am.iter()).map(|(&g, &a)| g - a).collect();
    let mut p = r.clone();

    let mut r_dot_r: f64 = r.iter().map(|x| x * x).sum();

    for _iter in 0..max_iter {
        if r_dot_r < 1e-20 {
            break;
        }

        let ap = apply_a(&p);
        let p_dot_ap: f64 = p.iter().zip(ap.iter()).map(|(&pi, &api)| pi * api).sum();
        if p_dot_ap.abs() < 1e-30 {
            break;
        }

        let alpha = r_dot_r / p_dot_ap;

        for j in 0..n_model {
            m[j] += alpha * p[j];
            r[j] -= alpha * ap[j];
        }

        let new_r_dot_r: f64 = r.iter().map(|x| x * x).sum();
        let beta = new_r_dot_r / r_dot_r;
        r_dot_r = new_r_dot_r;

        for j in 0..n_model {
            p[j] = r[j] + beta * p[j];
        }
    }

    r_dot_r.sqrt()
}

// ─── Rayleigh Wave Sensitivity Kernel ────────────────────────────────────────

/// Compute simplified Rayleigh wave depth sensitivity kernel.
/// The kernel peaks at approximately lambda/3 depth.
pub fn rayleigh_sensitivity_kernel(period_s: f64, ref_velocity: f64, depths: &[f64]) -> Vec<f64> {
    let wavelength = ref_velocity * period_s;
    let peak_depth = wavelength / 3.0;
    let sigma = wavelength / 4.0;

    let mut kernel: Vec<f64> = depths
        .iter()
        .map(|&z| {
            if sigma > 0.0 {
                (-(z - peak_depth).powi(2) / (2.0 * sigma * sigma)).exp()
            } else {
                0.0
            }
        })
        .collect();

    // Normalize
    let sum: f64 = kernel.iter().sum();
    if sum > 0.0 {
        for k in kernel.iter_mut() {
            *k /= sum;
        }
    }
    kernel
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- StationPair tests ---

    #[test]
    fn test_station_pair_distance() {
        let pair = StationPair::new(0.0, 0.0, 3000.0, 4000.0);
        assert!(approx_eq(pair.inter_station_distance(), 5000.0, 1e-6));
    }

    #[test]
    fn test_station_pair_zero_distance() {
        let pair = StationPair::new(100.0, 200.0, 100.0, 200.0);
        assert!(approx_eq(pair.inter_station_distance(), 0.0, 1e-10));
    }

    #[test]
    fn test_station_pair_azimuth() {
        let pair = StationPair::new(0.0, 0.0, 1000.0, 0.0);
        assert!(approx_eq(pair.azimuth(), 0.0, 1e-10)); // +x direction
    }

    #[test]
    fn test_station_pair_azimuth_45deg() {
        let pair = StationPair::new(0.0, 0.0, 1000.0, 1000.0);
        assert!(approx_eq(pair.azimuth(), PI / 4.0, 1e-10));
    }

    #[test]
    fn test_station_pair_midpoint() {
        let pair = StationPair::new(100.0, 200.0, 300.0, 400.0);
        let (mx, my) = pair.midpoint();
        assert!(approx_eq(mx, 200.0, 1e-10));
        assert!(approx_eq(my, 300.0, 1e-10));
    }

    // --- FrequencyBand tests ---

    #[test]
    fn test_frequency_band_center() {
        let band = FrequencyBand {
            min_hz: 0.1,
            max_hz: 0.5,
        };
        assert!(approx_eq(band.center(), 0.3, 1e-10));
    }

    #[test]
    fn test_frequency_band_bandwidth() {
        let band = FrequencyBand {
            min_hz: 0.1,
            max_hz: 0.5,
        };
        assert!(approx_eq(band.bandwidth(), 0.4, 1e-10));
    }

    // --- DispersionCurve tests ---

    #[test]
    fn test_dispersion_curve_length() {
        let dc = DispersionCurve::new(
            vec![1.0, 2.0, 3.0],
            vec![3000.0, 3200.0, 3400.0],
            vec![10.0, 12.0, 8.0],
        );
        assert_eq!(dc.len(), 3);
        assert!(!dc.is_empty());
    }

    #[test]
    fn test_dispersion_curve_interpolate() {
        let dc = DispersionCurve::new(
            vec![1.0, 2.0, 3.0],
            vec![3000.0, 3200.0, 3400.0],
            vec![10.0, 12.0, 8.0],
        );
        let v = dc.interpolate(1.5).unwrap();
        assert!(approx_eq(v, 3100.0, 1e-6));
    }

    #[test]
    fn test_dispersion_curve_interpolate_out_of_range() {
        let dc = DispersionCurve::new(
            vec![1.0, 2.0],
            vec![3000.0, 3200.0],
            vec![10.0, 12.0],
        );
        assert!(dc.interpolate(0.5).is_none());
        assert!(dc.interpolate(2.5).is_none());
    }

    #[test]
    fn test_dispersion_curve_smooth() {
        let dc = DispersionCurve::new(
            vec![1.0, 2.0, 3.0, 4.0, 5.0],
            vec![3000.0, 3500.0, 3100.0, 3400.0, 3200.0],
            vec![10.0; 5],
        );
        let smoothed = dc.smooth(3);
        // Smoothed values should be closer to the mean
        assert!(smoothed.velocity_ms[2] > 3000.0);
        assert!(smoothed.velocity_ms[2] < 3500.0);
    }

    #[test]
    fn test_dispersion_polynomial_fit() {
        let dc = DispersionCurve::new(
            vec![1.0, 2.0, 3.0, 4.0],
            vec![3000.0, 3100.0, 3200.0, 3300.0],
            vec![10.0; 4],
        );
        let fitted = dc.polynomial_fit(1); // Linear fit
        // Should be close to original for linear data
        for i in 0..4 {
            assert!(
                approx_eq(fitted[i], dc.velocity_ms[i], 1.0),
                "fitted[{}]={} vs expected={}",
                i,
                fitted[i],
                dc.velocity_ms[i]
            );
        }
    }

    // --- LayerModel tests ---

    #[test]
    fn test_layer_model_velocity_at_depth() {
        let model = LayerModel::new(
            vec![0.0, 1000.0, 5000.0],
            vec![2000.0, 3000.0, 4000.0],
        );
        assert!(approx_eq(model.velocity_at_depth(500.0), 2000.0, 1e-6));
        assert!(approx_eq(model.velocity_at_depth(2000.0), 3000.0, 1e-6));
        assert!(approx_eq(model.velocity_at_depth(8000.0), 4000.0, 1e-6));
    }

    #[test]
    fn test_layer_model_num_layers() {
        let model = LayerModel::new(vec![0.0, 100.0], vec![2500.0, 3500.0]);
        assert_eq!(model.num_layers(), 2);
    }

    // --- VelocityGrid tests ---

    #[test]
    fn test_velocity_grid_uniform() {
        let grid = VelocityGrid::new_uniform(4, 3, 1000.0, 1000.0, 0.0, 0.0, 3000.0);
        assert_eq!(grid.num_cells(), 12);
        assert!(approx_eq(grid.get(0, 0), 3000.0, 1e-6));
    }

    #[test]
    fn test_velocity_grid_set_get() {
        let mut grid = VelocityGrid::new_uniform(3, 3, 1000.0, 1000.0, 0.0, 0.0, 3000.0);
        grid.set(1, 1, 3500.0);
        assert!(approx_eq(grid.get(1, 1), 3500.0, 1e-6));
        assert!(approx_eq(grid.get(0, 0), 3000.0, 1e-6));
    }

    #[test]
    fn test_velocity_grid_find_cell() {
        let grid = VelocityGrid::new_uniform(4, 4, 1000.0, 1000.0, 0.0, 0.0, 3000.0);
        let (ix, iy) = grid.find_cell(1500.0, 2500.0).unwrap();
        assert_eq!(ix, 1);
        assert_eq!(iy, 2);
    }

    #[test]
    fn test_velocity_grid_find_cell_out_of_bounds() {
        let grid = VelocityGrid::new_uniform(4, 4, 1000.0, 1000.0, 0.0, 0.0, 3000.0);
        assert!(grid.find_cell(-100.0, 0.0).is_none());
        assert!(grid.find_cell(5000.0, 0.0).is_none());
    }

    #[test]
    fn test_velocity_grid_cell_center() {
        let grid = VelocityGrid::new_uniform(4, 4, 1000.0, 1000.0, 0.0, 0.0, 3000.0);
        let (cx, cy) = grid.cell_center(0, 0);
        assert!(approx_eq(cx, 500.0, 1e-6));
        assert!(approx_eq(cy, 500.0, 1e-6));
    }

    // --- Preprocessing tests ---

    #[test]
    fn test_one_bit_normalize() {
        let data = vec![-3.0, -0.1, 0.0, 0.5, 2.0];
        let result = one_bit_normalize(&data);
        assert!(approx_eq(result[0], -1.0, 1e-10));
        assert!(approx_eq(result[1], -1.0, 1e-10));
        assert!(approx_eq(result[2], 0.0, 1e-10));
        assert!(approx_eq(result[3], 1.0, 1e-10));
        assert!(approx_eq(result[4], 1.0, 1e-10));
    }

    #[test]
    fn test_running_absolute_mean_normalize() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let result = running_absolute_mean_normalize(&data, 3);
        assert_eq!(result.len(), 5);
        // Center element: mean_abs of [2,3,4] = 3, so result = 3/3 = 1
        assert!(approx_eq(result[2], 1.0, 1e-10));
    }

    #[test]
    fn test_spectral_whiten() {
        // Whitened signal should have roughly flat amplitude spectrum
        let n = 64;
        let data: Vec<f64> = (0..n)
            .map(|i| {
                (2.0 * PI * 5.0 * i as f64 / n as f64).sin()
                    + 0.5 * (2.0 * PI * 15.0 * i as f64 / n as f64).sin()
            })
            .collect();
        let whitened = spectral_whiten(&data);
        assert_eq!(whitened.len(), n);
        // After whitening, energy should be more evenly distributed
        let energy: f64 = whitened.iter().map(|x| x * x).sum();
        assert!(energy > 0.0);
    }

    #[test]
    fn test_bandpass_filter_passes_in_band() {
        let fs = 100.0;
        let n = 512;
        let f_signal = 10.0; // In-band
        let data: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_signal * i as f64 / fs).sin())
            .collect();
        let filtered = bandpass_filter(&data, fs, 5.0, 20.0);
        // Signal should be largely preserved
        let energy_in: f64 = data.iter().map(|x| x * x).sum();
        let energy_out: f64 = filtered.iter().map(|x| x * x).sum();
        assert!(energy_out > energy_in * 0.3); // At least 30% preserved
    }

    #[test]
    fn test_bandpass_filter_rejects_out_of_band() {
        let fs = 100.0;
        let n = 512;
        let f_signal = 40.0; // Out-of-band
        let data: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_signal * i as f64 / fs).sin())
            .collect();
        let filtered = bandpass_filter(&data, fs, 5.0, 20.0);
        let energy_in: f64 = data.iter().map(|x| x * x).sum();
        let energy_out: f64 = filtered.iter().map(|x| x * x).sum();
        assert!(energy_out < energy_in * 0.3); // Heavily attenuated
    }

    // --- Cross-correlation tests ---

    #[test]
    fn test_cross_correlate_direct_self() {
        let data = vec![1.0, 0.0, -1.0, 0.0, 1.0, 0.0, -1.0, 0.0];
        let cc = cross_correlate_direct(&data, &data, 3);
        // Auto-correlation peak at zero lag
        let mid = cc.len() / 2;
        let peak = cc[mid];
        assert!(peak > 0.0);
        // Peak should be the maximum
        for (i, &v) in cc.iter().enumerate() {
            if i != mid {
                assert!(v <= peak + 1e-10);
            }
        }
    }

    #[test]
    fn test_cross_correlate_delayed() {
        let n = 64;
        let delay = 5;
        let a: Vec<f64> = (0..n).map(|i| (2.0 * PI * 3.0 * i as f64 / n as f64).sin()).collect();
        let mut b = vec![0.0; n];
        for i in 0..n {
            if i + delay < n {
                b[i + delay] = a[i];
            }
        }
        let cc = cross_correlate_direct(&a, &b, 10);
        // Peak should be near lag = delay
        let mid = cc.len() / 2;
        let mut peak_lag = 0;
        let mut peak_val = cc[0].abs();
        for (i, &v) in cc.iter().enumerate() {
            if v.abs() > peak_val {
                peak_val = v.abs();
                peak_lag = i as isize - mid as isize;
            }
        }
        assert!((peak_lag - delay as isize).abs() <= 1);
    }

    // --- CrossCorrelationResult tests ---

    #[test]
    fn test_symmetric_component() {
        let xcorr = CrossCorrelationResult {
            lag_times: vec![-2.0, -1.0, 0.0, 1.0, 2.0],
            correlation: vec![0.1, 0.3, 1.0, 0.3, 0.1],
            num_stacks: 10,
            snr: 5.0,
        };
        let sym = xcorr.symmetric_component();
        // For symmetric input, symmetric component should match causal part
        assert!(sym.len() >= 2);
        assert!(approx_eq(sym[0], 1.0, 1e-10)); // Zero lag
    }

    #[test]
    fn test_causal_acausal_separation() {
        let xcorr = CrossCorrelationResult {
            lag_times: vec![-2.0, -1.0, 0.0, 1.0, 2.0],
            correlation: vec![0.1, 0.2, 1.0, 0.3, 0.4],
            num_stacks: 5,
            snr: 3.0,
        };
        let causal = xcorr.causal();
        let acausal = xcorr.acausal();
        assert_eq!(causal[0], 1.0); // Zero lag
        assert_eq!(acausal[0], 1.0); // Zero lag
        assert!(approx_eq(causal[1], 0.3, 1e-10));
        assert!(approx_eq(acausal[1], 0.2, 1e-10));
    }

    #[test]
    fn test_peak_amplitude() {
        let xcorr = CrossCorrelationResult {
            lag_times: vec![-1.0, 0.0, 1.0],
            correlation: vec![-0.5, 0.8, 0.3],
            num_stacks: 1,
            snr: 1.0,
        };
        assert!(approx_eq(xcorr.peak_amplitude(), 0.8, 1e-10));
    }

    // --- FFT tests ---

    #[test]
    fn test_fft_roundtrip() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let (re, im) = fft_real(&data);
        let recovered = ifft_to_real(&re, &im, data.len());
        for i in 0..data.len() {
            assert!(
                approx_eq(recovered[i], data[i], 1e-10),
                "FFT roundtrip failed at {}: {} vs {}",
                i,
                recovered[i],
                data[i]
            );
        }
    }

    #[test]
    fn test_fft_dc_signal() {
        let data = vec![5.0; 8];
        let (re, _im) = fft_real(&data);
        // DC bin should be N * amplitude
        assert!(approx_eq(re[0], 40.0, 1e-6));
    }

    // --- Envelope / Hilbert tests ---

    #[test]
    fn test_compute_envelope_sine() {
        let n = 128;
        let data: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4.0 * i as f64 / n as f64).sin())
            .collect();
        let env = compute_envelope(&data);
        assert_eq!(env.len(), n);
        // Envelope of a pure sinusoid should be roughly constant ~ 1.0
        // (with some edge effects)
        let mid_env: f64 = env[n / 4..3 * n / 4].iter().sum::<f64>() / (n / 2) as f64;
        assert!(mid_env > 0.8 && mid_env < 1.3, "mid_env = {}", mid_env);
    }

    // --- AmbientNoiseProcessor tests ---

    #[test]
    fn test_processor_segment_data() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 0.1,
                max_hz: 10.0,
            },
            5.0,
        );
        let data = vec![0.0; 500]; // 5 seconds
        let segments = proc.segment_data(&data);
        // Segment = 100 samples, hop = 50, data = 500 -> ~9 segments
        assert!(segments.len() >= 8);
        assert_eq!(segments[0].len(), 100);
    }

    #[test]
    fn test_processor_segment_samples() {
        let proc = AmbientNoiseProcessor::new(
            200.0,
            0.5,
            0.5,
            FrequencyBand {
                min_hz: 0.1,
                max_hz: 10.0,
            },
            5.0,
        );
        assert_eq!(proc.segment_samples(), 100);
    }

    #[test]
    fn test_processor_max_lag_samples() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 0.1,
                max_hz: 10.0,
            },
            5.0,
        );
        assert_eq!(proc.max_lag_samples(), 500);
    }

    #[test]
    fn test_cross_correlation_stacking() {
        let fs = 100.0;
        let proc = AmbientNoiseProcessor::new(
            fs,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 1.0,
                max_hz: 40.0,
            },
            0.5,
        );
        // Generate identical signals with some noise
        let n = 500;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 10.0 * i as f64 / fs).sin())
            .collect();
        let result = proc.cross_correlate(&signal, &signal);
        assert!(result.num_stacks > 0);
        assert!(!result.correlation.is_empty());
    }

    // --- Green's function extraction ---

    #[test]
    fn test_green_function_extraction() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 1.0,
                max_hz: 40.0,
            },
            0.5,
        );
        let xcorr = CrossCorrelationResult {
            lag_times: (-50..=50).map(|i| i as f64 * 0.01).collect(),
            correlation: (-50..=50)
                .map(|i| (-(i as f64).powi(2) / 200.0).exp())
                .collect(),
            num_stacks: 10,
            snr: 5.0,
        };
        let green = proc.extract_green_function(&xcorr);
        assert!(!green.amplitude.is_empty());
        assert!(!green.time_s.is_empty());
        assert!(green.time_s[0] >= 0.0);
    }

    // --- SNR estimation ---

    #[test]
    fn test_snr_estimation() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 0.1,
                max_hz: 10.0,
            },
            5.0,
        );
        // Strong peak at center, low noise elsewhere
        let n = 101;
        let mid = n / 2;
        let mut corr = vec![0.01; n];
        corr[mid] = 10.0;
        let snr = proc.estimate_snr(&corr);
        assert!(snr > 1.0, "SNR should be > 1 for peaked signal, got {}", snr);
    }

    // --- Rayleigh wave sensitivity kernel ---

    #[test]
    fn test_sensitivity_kernel_peaks_at_lambda_third() {
        let period = 10.0; // 10 seconds
        let velocity = 3000.0; // 3 km/s
        let wavelength = velocity * period; // 30000 m
        let expected_peak_depth = wavelength / 3.0; // 10000 m

        let depths: Vec<f64> = (0..100).map(|i| i as f64 * 200.0).collect(); // 0-20000 m
        let kernel = rayleigh_sensitivity_kernel(period, velocity, &depths);

        // Find peak of kernel
        let mut peak_idx = 0;
        let mut peak_val = kernel[0];
        for (i, &k) in kernel.iter().enumerate() {
            if k > peak_val {
                peak_val = k;
                peak_idx = i;
            }
        }
        let peak_depth = depths[peak_idx];
        // Peak should be near lambda/3
        assert!(
            (peak_depth - expected_peak_depth).abs() < 1000.0,
            "Kernel peak at {} m, expected near {} m",
            peak_depth,
            expected_peak_depth,
        );
    }

    #[test]
    fn test_sensitivity_kernel_normalized() {
        let depths: Vec<f64> = (0..50).map(|i| i as f64 * 100.0).collect();
        let kernel = rayleigh_sensitivity_kernel(5.0, 3000.0, &depths);
        let sum: f64 = kernel.iter().sum();
        assert!(approx_eq(sum, 1.0, 0.01), "Kernel sum = {}", sum);
    }

    // --- 1D Inversion test ---

    #[test]
    fn test_invert_1d_basic() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 0.05,
                max_hz: 0.5,
            },
            30.0,
        );

        // Create a dispersion curve with increasing velocity at longer periods
        let dispersion = DispersionCurve::new(
            vec![2.0, 5.0, 10.0, 20.0],
            vec![2500.0, 2800.0, 3200.0, 3600.0],
            vec![10.0; 4],
        );

        let model = proc.invert_1d(&dispersion, 10, 50000.0, 3000.0);
        assert_eq!(model.num_layers(), 10);
        // Velocity should generally increase with depth
        // (shallow layers sense shorter periods = slower velocities)
        assert!(model.vs_ms[0] < model.vs_ms[9] + 500.0);
    }

    // --- 2D Tomography test ---

    #[test]
    fn test_velocity_grid_ray_tracing() {
        let grid = VelocityGrid::new_uniform(5, 5, 1000.0, 1000.0, 0.0, 0.0, 3000.0);
        let pair = StationPair::new(500.0, 500.0, 4500.0, 4500.0);
        let mut path = vec![0.0; grid.num_cells()];
        trace_ray(&pair, &grid, &mut path);
        // Some cells along the diagonal should have non-zero path length
        let total: f64 = path.iter().sum();
        let expected_dist = pair.inter_station_distance();
        assert!(
            (total - expected_dist).abs() < expected_dist * 0.1,
            "Total path {} vs expected {}",
            total,
            expected_dist
        );
    }

    #[test]
    fn test_invert_2d_basic() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 0.05,
                max_hz: 0.5,
            },
            30.0,
        );

        let mut grid = VelocityGrid::new_uniform(3, 3, 1000.0, 1000.0, 0.0, 0.0, 3000.0);

        // Create some station pairs
        let pairs = vec![
            StationPair::new(500.0, 500.0, 2500.0, 500.0),
            StationPair::new(500.0, 500.0, 2500.0, 2500.0),
            StationPair::new(500.0, 2500.0, 2500.0, 500.0),
            StationPair::new(500.0, 1500.0, 2500.0, 1500.0),
        ];

        // Synthetic travel times (slightly perturbed from uniform model)
        let travel_times: Vec<f64> = pairs
            .iter()
            .map(|p| p.inter_station_distance() / 3000.0 * 1.02)
            .collect();

        let result = proc.invert_2d(&pairs, &travel_times, &mut grid, 1.0, 0.1, 20);
        assert!(result.iterations > 0);
        assert!(result.velocity_grid.num_cells() == 9);
    }

    // --- Checkerboard test ---

    #[test]
    fn test_checkerboard_resolution() {
        let proc = AmbientNoiseProcessor::new(
            100.0,
            1.0,
            0.5,
            FrequencyBand {
                min_hz: 0.05,
                max_hz: 0.5,
            },
            30.0,
        );

        let grid = VelocityGrid::new_uniform(4, 4, 1000.0, 1000.0, 0.0, 0.0, 3000.0);

        let pairs = vec![
            StationPair::new(500.0, 500.0, 3500.0, 500.0),
            StationPair::new(500.0, 500.0, 3500.0, 3500.0),
            StationPair::new(500.0, 3500.0, 3500.0, 500.0),
            StationPair::new(500.0, 1500.0, 3500.0, 1500.0),
            StationPair::new(1500.0, 500.0, 1500.0, 3500.0),
            StationPair::new(2500.0, 500.0, 2500.0, 3500.0),
        ];

        let result = proc.checkerboard_test(&pairs, &grid, 5.0, 2, 1.0, 0.1, 20);
        assert_eq!(result.num_cells(), 16);
    }

    // --- Deconvolution test ---

    #[test]
    fn test_deconvolve_instrument_identity() {
        // Identity response: all 1+0j
        let data = vec![1.0, 0.0, -1.0, 0.0, 1.0, 0.0, -1.0, 0.0];
        let nf = data.len() / 2 + 1;
        let response_re = vec![1.0; nf];
        let response_im = vec![0.0; nf];
        let result = deconvolve_instrument(&data, &response_re, &response_im, -60.0);
        assert_eq!(result.len(), data.len());
        // Should approximately recover original
        for i in 0..data.len() {
            assert!(
                approx_eq(result[i], data[i], 0.2),
                "Deconv at {}: {} vs {}",
                i,
                result[i],
                data[i]
            );
        }
    }

    // --- Linear algebra tests ---

    #[test]
    fn test_solve_linear_system() {
        // 2x + y = 5, x + 3y = 7 => x = 1.6, y = 1.8
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let b = vec![5.0, 7.0];
        let x = solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 1.6, 1e-10));
        assert!(approx_eq(x[1], 1.8, 1e-10));
    }

    #[test]
    fn test_next_pow2() {
        assert_eq!(next_pow2(1), 1);
        assert_eq!(next_pow2(3), 4);
        assert_eq!(next_pow2(8), 8);
        assert_eq!(next_pow2(100), 128);
    }
}
