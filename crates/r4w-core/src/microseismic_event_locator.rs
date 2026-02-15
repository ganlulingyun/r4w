//! # Microseismic Event Locator
//!
//! Downhole microseismic monitoring signal processing for hydraulic fracturing
//! operations. Microseismic events (magnitude -3 to +1) are located using arrival
//! time picks from downhole geophones to map fracture growth in real-time.
//!
//! ## Overview
//!
//! During hydraulic fracturing ("fracking"), high-pressure fluid injection causes
//! rock to fracture, generating small seismic events. Arrays of downhole geophones
//! (typically 8-24 receivers) detect P-wave and S-wave arrivals from these events.
//! By picking arrival times and solving the inverse problem, event locations can be
//! determined to map fracture geometry.
//!
//! ## Processing Pipeline
//!
//! ```text
//! Raw Waveforms → STA/LTA Detection → AIC P-Pick → Polarization S-Pick
//!     → Wadati Vp/Vs → Grid Search Initial → Geiger Refinement → Location
//! ```
//!
//! ## Key Algorithms
//!
//! - **STA/LTA**: Short-Term Average / Long-Term Average ratio for event detection
//! - **AIC Picker**: Akaike Information Criterion for precise onset picking
//! - **Wadati Diagram**: Linear regression of Ts-Tp vs Tp for Vp/Vs ratio
//! - **Grid Search**: Exhaustive spatial search for initial location estimate
//! - **Geiger's Method**: Iterative linearized inversion for refined hypocenter
//! - **Hodogram Analysis**: 3-component polarization for back-azimuth and incidence
//! - **Brune Source Model**: Corner frequency and source radius estimation

/// Configuration for the microseismic monitoring system.
#[derive(Debug, Clone)]
pub struct MicroseismicConfig {
    /// Sample rate in Hz (typically 2000-4000 Hz for downhole geophones)
    pub sample_rate_hz: f64,
    /// Number of receivers in the monitoring array (typically 8-24)
    pub num_receivers: usize,
    /// Receiver positions in meters [x, y, z] (z positive downward)
    pub receiver_positions: Vec<[f64; 3]>,
    /// P-wave velocity in m/s (typically 4000-6000 m/s in shale)
    pub vp_m_s: f64,
    /// S-wave velocity in m/s (typically 2300-3500 m/s in shale)
    pub vs_m_s: f64,
    /// Short-term average window in seconds (typically 0.02-0.05 s)
    pub sta_window_s: f64,
    /// Long-term average window in seconds (typically 0.5-2.0 s)
    pub lta_window_s: f64,
    /// STA/LTA detection threshold ratio (typically 3-8)
    pub detection_threshold: f64,
}

impl MicroseismicConfig {
    /// Creates a default configuration for a typical monitoring scenario.
    ///
    /// Uses 12 receivers at 10m spacing along a vertical monitoring well,
    /// with velocities typical of shale formations.
    pub fn default_config() -> Self {
        let num_receivers = 12;
        let mut receiver_positions = Vec::with_capacity(num_receivers);
        for i in 0..num_receivers {
            // Vertical monitoring well at x=0, y=0, depths 2000-2110 m
            receiver_positions.push([0.0, 0.0, 2000.0 + i as f64 * 10.0]);
        }
        Self {
            sample_rate_hz: 4000.0,
            num_receivers,
            receiver_positions,
            vp_m_s: 5000.0,
            vs_m_s: 2900.0,
            sta_window_s: 0.03,
            lta_window_s: 1.0,
            detection_threshold: 5.0,
        }
    }
}

/// A located microseismic event with associated metadata.
#[derive(Debug, Clone)]
pub struct SeismicEvent {
    /// Event location in meters [x, y, z]
    pub location: [f64; 3],
    /// Origin time in seconds relative to recording start
    pub origin_time_s: f64,
    /// Event magnitude (moment magnitude Mw)
    pub magnitude: f64,
    /// P-wave arrival times at each receiver (seconds)
    pub p_arrival_times: Vec<f64>,
    /// S-wave arrival times at each receiver (seconds)
    pub s_arrival_times: Vec<f64>,
    /// Back-azimuth to the source in degrees (0=North, 90=East)
    pub azimuth_deg: f64,
    /// Location error estimate in meters (RMS residual mapped to distance)
    pub location_error_m: f64,
}

/// Result of 3-component polarization analysis from a hodogram.
#[derive(Debug, Clone)]
pub struct PolarizationResult {
    /// Back-azimuth to the source in degrees (0=North, 90=East)
    pub back_azimuth_deg: f64,
    /// Incidence angle from vertical in degrees (0=vertical, 90=horizontal)
    pub incidence_angle_deg: f64,
    /// Rectilinearity: 0 = circular/random, 1 = perfectly linear polarization
    pub rectilinearity: f64,
    /// Planarity: 1 - lambda3/lambda2, measures planar vs 3D motion
    pub planarity: f64,
}

/// Microseismic event detector and arrival time picker.
///
/// Implements STA/LTA event detection and AIC-based arrival time picking
/// for P-wave and S-wave onsets.
///
/// # Example
///
/// ```
/// use r4w_core::microseismic_event_locator::{MicroseismicConfig, MicroseismicLocator};
///
/// let config = MicroseismicConfig::default_config();
/// let locator = MicroseismicLocator::new(config);
///
/// // Generate a test signal with a sudden onset at sample 1000
/// let mut signal = vec![0.01; 2000];
/// for i in 1000..2000 {
///     signal[i] = 1.0;
/// }
///
/// let ratio = locator.sta_lta(&signal, 40, 400);
/// let triggers = locator.detect_events(&ratio, 3.0, 100);
/// assert!(!triggers.is_empty());
/// ```
pub struct MicroseismicLocator {
    config: MicroseismicConfig,
}

impl MicroseismicLocator {
    /// Creates a new locator with the given configuration.
    pub fn new(config: MicroseismicConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the configuration.
    pub fn config(&self) -> &MicroseismicConfig {
        &self.config
    }

    /// Computes the STA/LTA (Short-Term Average / Long-Term Average) ratio.
    ///
    /// The STA/LTA algorithm compares the average absolute amplitude in a short
    /// window to that in a long window. When a seismic event arrives, the STA
    /// increases sharply while the LTA remains stable, producing a ratio spike.
    ///
    /// # Arguments
    /// * `signal` - Input seismic trace (single component)
    /// * `sta_samples` - Short-term average window length in samples
    /// * `lta_samples` - Long-term average window length in samples
    ///
    /// # Returns
    /// Vector of STA/LTA ratios (same length as input, padded with 1.0 for
    /// initial samples where windows are incomplete)
    pub fn sta_lta(
        &self,
        signal: &[f64],
        sta_samples: usize,
        lta_samples: usize,
    ) -> Vec<f64> {
        let n = signal.len();
        let mut ratio = vec![1.0; n];

        if n < lta_samples + sta_samples || sta_samples == 0 || lta_samples == 0 {
            return ratio;
        }

        for i in lta_samples..n {
            // STA: average absolute amplitude over short window ending at i
            let sta_start = if i >= sta_samples { i - sta_samples } else { 0 };
            let sta: f64 = signal[sta_start..i]
                .iter()
                .map(|x| x.abs())
                .sum::<f64>()
                / sta_samples as f64;

            // LTA: average absolute amplitude over long window ending at sta_start
            let lta_end = sta_start;
            let lta_start = if lta_end >= lta_samples {
                lta_end - lta_samples
            } else {
                0
            };
            let lta_len = lta_end - lta_start;
            if lta_len == 0 {
                continue;
            }
            let lta: f64 = signal[lta_start..lta_end]
                .iter()
                .map(|x| x.abs())
                .sum::<f64>()
                / lta_len as f64;

            if lta > 1e-30 {
                ratio[i] = sta / lta;
            }
        }

        ratio
    }

    /// Detects event trigger indices from an STA/LTA ratio trace.
    ///
    /// Finds samples where the ratio exceeds the threshold and enforces a
    /// holdoff period to prevent retriggering on the same event.
    ///
    /// # Arguments
    /// * `sta_lta_ratio` - STA/LTA ratio trace
    /// * `threshold` - Trigger threshold (e.g. 3.0-8.0)
    /// * `holdoff_samples` - Minimum samples between successive triggers
    ///
    /// # Returns
    /// Vector of sample indices where events are detected
    pub fn detect_events(
        &self,
        sta_lta_ratio: &[f64],
        threshold: f64,
        holdoff_samples: usize,
    ) -> Vec<usize> {
        let mut triggers = Vec::new();
        let mut last_trigger: Option<usize> = None;

        for (i, &r) in sta_lta_ratio.iter().enumerate() {
            if r >= threshold {
                let can_trigger = match last_trigger {
                    Some(lt) => i >= lt + holdoff_samples,
                    None => true,
                };
                if can_trigger {
                    triggers.push(i);
                    last_trigger = Some(i);
                }
            }
        }

        triggers
    }

    /// Picks the P-wave arrival time using the AIC picker within a window.
    ///
    /// Applies the Akaike Information Criterion picker to a windowed segment
    /// centered on the approximate event time. Returns the picked onset time
    /// in seconds.
    ///
    /// # Arguments
    /// * `signal` - Full seismic trace
    /// * `event_idx` - Approximate event sample index (e.g. from STA/LTA)
    /// * `window` - Half-window size in samples to search around event_idx
    ///
    /// # Returns
    /// P-wave onset time in seconds (sample index / sample_rate)
    pub fn pick_p_arrival(&self, signal: &[f64], event_idx: usize, window: usize) -> f64 {
        let start = if event_idx > window {
            event_idx - window
        } else {
            0
        };
        let end = (event_idx + window).min(signal.len());

        if end <= start + 2 {
            return event_idx as f64 / self.config.sample_rate_hz;
        }

        let segment = &signal[start..end];
        let aic_idx = Self::aic_picker(segment);

        (start + aic_idx) as f64 / self.config.sample_rate_hz
    }

    /// Picks the S-wave arrival from 3-component data using polarization change.
    ///
    /// S-wave arrivals are characterized by a change in particle motion direction.
    /// This method detects the onset of transverse motion by analyzing the
    /// eigenvalue ratio of the covariance matrix in sliding windows.
    ///
    /// # Arguments
    /// * `signal_3c` - Three-component seismic data [north, east, vertical]
    /// * `event_idx` - Approximate event sample index
    /// * `window` - Half-window size in samples
    ///
    /// # Returns
    /// S-wave onset time in seconds
    pub fn pick_s_arrival(
        &self,
        signal_3c: &[[f64; 3]],
        event_idx: usize,
        window: usize,
    ) -> f64 {
        let start = if event_idx > window {
            event_idx - window
        } else {
            0
        };
        let end = (event_idx + window * 2).min(signal_3c.len());

        if end <= start + 4 {
            return event_idx as f64 / self.config.sample_rate_hz;
        }

        // Compute transverse energy ratio in sliding windows
        // P-wave: predominantly longitudinal, S-wave: transverse
        let analysis_window = 10.min((end - start) / 4).max(2);
        let mut max_change = 0.0f64;
        let mut best_idx = event_idx;

        for i in (start + analysis_window)..(end - analysis_window) {
            // Compute rectilinearity before and after this point
            let rect_before = self.local_rectilinearity(signal_3c, i - analysis_window, i);
            let rect_after = self.local_rectilinearity(signal_3c, i, i + analysis_window);

            // S-wave onset: rectilinearity drops (motion becomes less linear)
            let change = rect_before - rect_after;
            if change > max_change {
                max_change = change;
                best_idx = i;
            }
        }

        best_idx as f64 / self.config.sample_rate_hz
    }

    /// Computes local rectilinearity for a segment of 3C data.
    fn local_rectilinearity(&self, signal_3c: &[[f64; 3]], start: usize, end: usize) -> f64 {
        if end <= start + 1 {
            return 0.0;
        }

        let n = end - start;
        let segment = &signal_3c[start..end];

        // Compute means
        let mut mean = [0.0; 3];
        for s in segment.iter() {
            for j in 0..3 {
                mean[j] += s[j];
            }
        }
        for j in 0..3 {
            mean[j] /= n as f64;
        }

        // Compute 3x3 covariance matrix
        let mut cov = [[0.0f64; 3]; 3];
        for s in segment.iter() {
            for j in 0..3 {
                for k in 0..3 {
                    cov[j][k] += (s[j] - mean[j]) * (s[k] - mean[k]);
                }
            }
        }
        for j in 0..3 {
            for k in 0..3 {
                cov[j][k] /= n as f64;
            }
        }

        let eigenvalues = eigenvalues_3x3_symmetric(&cov);
        let sum = eigenvalues[0] + eigenvalues[1] + eigenvalues[2];
        if sum < 1e-30 {
            return 0.0;
        }
        1.0 - (eigenvalues[1] + eigenvalues[2]) / (2.0 * eigenvalues[0])
    }

    /// Akaike Information Criterion (AIC) picker for seismic onset detection.
    ///
    /// The AIC function is minimized at the transition point between noise
    /// and signal. For a windowed signal segment, it computes:
    ///
    /// AIC(k) = k * ln(var(x[0..k])) + (N-k-1) * ln(var(x[k..N]))
    ///
    /// The minimum of this function indicates the most likely onset time.
    ///
    /// # Arguments
    /// * `signal` - Windowed signal segment containing the onset
    ///
    /// # Returns
    /// Index within the segment where the onset is detected
    pub fn aic_picker(signal: &[f64]) -> usize {
        let n = signal.len();
        if n < 4 {
            return 0;
        }

        let mut aic = vec![0.0f64; n];
        let mut best_idx = 0;
        let mut best_aic = f64::MAX;

        for k in 1..(n - 1) {
            // Variance of first segment [0..k]
            let var1 = variance(&signal[0..k]);
            // Variance of second segment [k..n]
            let var2 = variance(&signal[k..n]);

            // AIC formula
            let v1 = if var1 > 1e-30 { var1.ln() } else { -60.0 };
            let v2 = if var2 > 1e-30 { var2.ln() } else { -60.0 };

            aic[k] = k as f64 * v1 + (n - k - 1) as f64 * v2;

            if aic[k] < best_aic {
                best_aic = aic[k];
                best_idx = k;
            }
        }

        best_idx
    }
}

/// Event location solver using travel time inversion.
///
/// Implements grid search and Geiger's iterative linearized inversion
/// to determine microseismic event hypocenters from P-wave and S-wave
/// arrival times.
pub struct EventLocator {
    /// Receiver positions in meters [x, y, z]
    receiver_positions: Vec<[f64; 3]>,
    /// P-wave velocity in m/s
    vp: f64,
    /// S-wave velocity in m/s
    vs: f64,
}

impl EventLocator {
    /// Creates a new event locator.
    ///
    /// # Arguments
    /// * `receiver_positions` - Array of receiver locations [x, y, z] in meters
    /// * `vp` - P-wave velocity in m/s
    /// * `vs` - S-wave velocity in m/s
    pub fn new(receiver_positions: Vec<[f64; 3]>, vp: f64, vs: f64) -> Self {
        Self {
            receiver_positions,
            vp,
            vs,
        }
    }

    /// Computes travel time from source to receiver through a homogeneous medium.
    ///
    /// travel_time = distance / velocity
    ///
    /// # Arguments
    /// * `source` - Source position [x, y, z] in meters
    /// * `receiver` - Receiver position [x, y, z] in meters
    /// * `velocity` - Wave velocity in m/s
    ///
    /// # Returns
    /// Travel time in seconds
    pub fn travel_time(source: [f64; 3], receiver: [f64; 3], velocity: f64) -> f64 {
        distance_3d(source, receiver) / velocity
    }

    /// Locates an event using exhaustive grid search over P-wave arrival times.
    ///
    /// Evaluates the RMS travel time residual at every grid point and returns
    /// the position with the minimum residual.
    ///
    /// # Arguments
    /// * `p_times` - Observed P-wave arrival times at each receiver (seconds)
    /// * `receiver_positions` - Receiver positions [x, y, z] in meters
    /// * `vp` - P-wave velocity in m/s
    /// * `grid_bounds` - (min_corner, max_corner) of the search volume
    /// * `grid_step` - Grid spacing in meters
    ///
    /// # Returns
    /// Best-fit source location [x, y, z] in meters
    pub fn locate_grid_search(
        p_times: &[f64],
        receiver_positions: &[[f64; 3]],
        vp: f64,
        grid_bounds: ([f64; 3], [f64; 3]),
        grid_step: f64,
    ) -> [f64; 3] {
        let (min_corner, max_corner) = grid_bounds;
        let mut best_location = [0.0; 3];
        let mut best_residual = f64::MAX;

        let nx = ((max_corner[0] - min_corner[0]) / grid_step).ceil() as usize + 1;
        let ny = ((max_corner[1] - min_corner[1]) / grid_step).ceil() as usize + 1;
        let nz = ((max_corner[2] - min_corner[2]) / grid_step).ceil() as usize + 1;

        for ix in 0..nx {
            let x = min_corner[0] + ix as f64 * grid_step;
            for iy in 0..ny {
                let y = min_corner[1] + iy as f64 * grid_step;
                for iz in 0..nz {
                    let z = min_corner[2] + iz as f64 * grid_step;
                    let source = [x, y, z];

                    // Compute predicted travel times and estimate origin time
                    let predicted: Vec<f64> = receiver_positions
                        .iter()
                        .map(|r| Self::travel_time(source, *r, vp))
                        .collect();

                    // Origin time estimate: minimize residual by subtracting mean difference
                    let n = p_times.len().min(predicted.len());
                    if n == 0 {
                        continue;
                    }

                    let origin_time: f64 = (0..n)
                        .map(|i| p_times[i] - predicted[i])
                        .sum::<f64>()
                        / n as f64;

                    // RMS residual
                    let residual: f64 = (0..n)
                        .map(|i| {
                            let r = p_times[i] - (origin_time + predicted[i]);
                            r * r
                        })
                        .sum::<f64>()
                        / n as f64;
                    let rms = residual.sqrt();

                    if rms < best_residual {
                        best_residual = rms;
                        best_location = source;
                    }
                }
            }
        }

        best_location
    }

    /// Locates an event using Geiger's iterative linearized inversion.
    ///
    /// Starting from an initial guess, iteratively linearizes the travel time
    /// equations and solves the overdetermined system using least-squares to
    /// update the hypocenter location and origin time.
    ///
    /// # Arguments
    /// * `p_times` - Observed P-wave arrival times (seconds)
    /// * `s_times` - Observed S-wave arrival times (seconds), can be empty
    /// * `initial_guess` - Starting location [x, y, z] in meters
    ///
    /// # Returns
    /// Located SeismicEvent with refined position, origin time, and error estimate
    pub fn locate_geiger(
        &self,
        p_times: &[f64],
        s_times: &[f64],
        initial_guess: [f64; 3],
    ) -> SeismicEvent {
        let max_iterations = 20;
        let convergence_threshold = 0.1; // meters

        let mut location = initial_guess;
        let n_receivers = self.receiver_positions.len().min(p_times.len());

        // Estimate initial origin time
        let mut origin_time = if n_receivers > 0 {
            let sum: f64 = (0..n_receivers)
                .map(|i| {
                    p_times[i]
                        - Self::travel_time(location, self.receiver_positions[i], self.vp)
                })
                .sum();
            sum / n_receivers as f64
        } else {
            0.0
        };

        for _iter in 0..max_iterations {
            // Build the Jacobian matrix and residual vector
            // For each P observation: t_obs = t0 + dist(src, rec) / vp
            // Partial derivatives: dt/dx = (x_src - x_rec) / (dist * vp), etc.
            let mut num_obs = n_receivers;
            let n_s = s_times.len().min(n_receivers);
            if n_s > 0 {
                num_obs += n_s;
            }

            if num_obs < 4 {
                break;
            }

            // Jacobian [num_obs x 4] and residual [num_obs]
            let mut jtj = [[0.0f64; 4]; 4]; // J^T * J
            let mut jtr = [0.0f64; 4]; // J^T * residual

            for i in 0..n_receivers {
                let dist = distance_3d(location, self.receiver_positions[i]);
                if dist < 1e-6 {
                    continue;
                }
                let predicted = origin_time + dist / self.vp;
                let residual = p_times[i] - predicted;

                // Partial derivatives
                let dx = (location[0] - self.receiver_positions[i][0]) / (dist * self.vp);
                let dy = (location[1] - self.receiver_positions[i][1]) / (dist * self.vp);
                let dz = (location[2] - self.receiver_positions[i][2]) / (dist * self.vp);
                let dt0 = 1.0;

                let row = [dx, dy, dz, dt0];

                for j in 0..4 {
                    for k in 0..4 {
                        jtj[j][k] += row[j] * row[k];
                    }
                    jtr[j] += row[j] * residual;
                }
            }

            // Add S-wave observations
            for i in 0..n_s {
                let dist = distance_3d(location, self.receiver_positions[i]);
                if dist < 1e-6 {
                    continue;
                }
                let predicted = origin_time + dist / self.vs;
                let residual = s_times[i] - predicted;

                let dx = (location[0] - self.receiver_positions[i][0]) / (dist * self.vs);
                let dy = (location[1] - self.receiver_positions[i][1]) / (dist * self.vs);
                let dz = (location[2] - self.receiver_positions[i][2]) / (dist * self.vs);
                let dt0 = 1.0;

                let row = [dx, dy, dz, dt0];

                for j in 0..4 {
                    for k in 0..4 {
                        jtj[j][k] += row[j] * row[k];
                    }
                    jtr[j] += row[j] * residual;
                }
            }

            // Solve 4x4 system: jtj * delta = jtr using Gaussian elimination
            let delta = match solve_4x4(&jtj, &jtr) {
                Some(d) => d,
                None => break,
            };

            location[0] += delta[0];
            location[1] += delta[1];
            location[2] += delta[2];
            origin_time += delta[3];

            let shift = (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
            if shift < convergence_threshold {
                break;
            }
        }

        // Compute final residuals
        let mut p_residuals = Vec::new();
        for i in 0..n_receivers {
            let predicted =
                origin_time + Self::travel_time(location, self.receiver_positions[i], self.vp);
            p_residuals.push(p_times[i] - predicted);
        }
        let rms_residual = residual_rms(&p_residuals, &vec![0.0; p_residuals.len()]);
        let location_error = rms_residual * self.vp; // approximate mapping to distance

        SeismicEvent {
            location,
            origin_time_s: origin_time,
            magnitude: 0.0, // to be estimated separately
            p_arrival_times: p_times.to_vec(),
            s_arrival_times: s_times.to_vec(),
            azimuth_deg: 0.0,
            location_error_m: location_error,
        }
    }

    /// Performs Wadati diagram analysis to estimate Vp/Vs ratio and origin time.
    ///
    /// The Wadati method plots (Ts - Tp) vs Tp and fits a line. The slope
    /// equals (Vp/Vs - 1), and the x-intercept gives the origin time.
    ///
    /// # Arguments
    /// * `p_times` - P-wave arrival times at each receiver (seconds)
    /// * `s_times` - S-wave arrival times at each receiver (seconds)
    ///
    /// # Returns
    /// (vp_vs_ratio, origin_time_s) - the Vp/Vs ratio and estimated origin time
    pub fn wadati_diagram(p_times: &[f64], s_times: &[f64]) -> (f64, f64) {
        let n = p_times.len().min(s_times.len());
        if n < 2 {
            return (1.73, 0.0); // default Vp/Vs for Poisson solid
        }

        // x = Tp, y = Ts - Tp
        let x: Vec<f64> = p_times[..n].to_vec();
        let y: Vec<f64> = (0..n).map(|i| s_times[i] - p_times[i]).collect();

        // Linear regression: y = slope * x + intercept
        let (slope, intercept) = linear_regression(&x, &y);

        // slope = Vp/Vs - 1, so Vp/Vs = slope + 1
        let vp_vs = slope + 1.0;

        // Origin time: where Ts - Tp = 0, i.e., x = -intercept / slope
        let origin_time = if slope.abs() > 1e-10 {
            -intercept / slope
        } else {
            0.0
        };

        (vp_vs, origin_time)
    }
}

/// Computes the RMS of residuals between predicted and observed values.
///
/// # Arguments
/// * `predicted` - Predicted values
/// * `observed` - Observed values
///
/// # Returns
/// Root mean square of (predicted - observed) differences
pub fn residual_rms(predicted: &[f64], observed: &[f64]) -> f64 {
    let n = predicted.len().min(observed.len());
    if n == 0 {
        return 0.0;
    }
    let sum_sq: f64 = (0..n)
        .map(|i| {
            let r = predicted[i] - observed[i];
            r * r
        })
        .sum();
    (sum_sq / n as f64).sqrt()
}

/// Three-component hodogram analyzer for polarization and back-azimuth.
///
/// Analyzes the particle motion recorded on 3-component geophones (North,
/// East, Vertical) to determine the polarization characteristics and
/// back-azimuth to the seismic source.
pub struct HodogramAnalyzer;

impl HodogramAnalyzer {
    /// Creates a new hodogram analyzer.
    pub fn new() -> Self {
        Self
    }

    /// Performs full polarization analysis on 3-component data.
    ///
    /// Computes the covariance matrix of the 3-component motion, extracts
    /// eigenvalues and eigenvectors, and derives polarization attributes.
    ///
    /// # Arguments
    /// * `north` - North component (positive = North)
    /// * `east` - East component (positive = East)
    /// * `vertical` - Vertical component (positive = Up)
    ///
    /// # Returns
    /// PolarizationResult with back-azimuth, incidence angle, rectilinearity, planarity
    pub fn polarization_analysis(
        &self,
        north: &[f64],
        east: &[f64],
        vertical: &[f64],
    ) -> PolarizationResult {
        let n = north.len().min(east.len()).min(vertical.len());
        if n == 0 {
            return PolarizationResult {
                back_azimuth_deg: 0.0,
                incidence_angle_deg: 0.0,
                rectilinearity: 0.0,
                planarity: 0.0,
            };
        }

        // Compute means
        let mean_n: f64 = north[..n].iter().sum::<f64>() / n as f64;
        let mean_e: f64 = east[..n].iter().sum::<f64>() / n as f64;
        let mean_v: f64 = vertical[..n].iter().sum::<f64>() / n as f64;

        // Build 3x3 covariance matrix [N, E, V]
        let mut cov = [[0.0f64; 3]; 3];
        for i in 0..n {
            let dn = north[i] - mean_n;
            let de = east[i] - mean_e;
            let dv = vertical[i] - mean_v;
            let d = [dn, de, dv];
            for j in 0..3 {
                for k in 0..3 {
                    cov[j][k] += d[j] * d[k];
                }
            }
        }
        for j in 0..3 {
            for k in 0..3 {
                cov[j][k] /= n as f64;
            }
        }

        let eigenvalues = eigenvalues_3x3_symmetric(&cov);
        let eigenvector = dominant_eigenvector_3x3(&cov, eigenvalues[0]);

        // Back-azimuth from dominant eigenvector (N, E components)
        let baz = Self::back_azimuth(north, east);

        // Incidence angle from dominant eigenvector
        let horiz_mag = (eigenvector[0] * eigenvector[0] + eigenvector[1] * eigenvector[1]).sqrt();
        let vert_mag = eigenvector[2].abs();
        let incidence = horiz_mag.atan2(vert_mag).to_degrees();

        // Rectilinearity: 1 - (λ2 + λ3) / (2 * λ1)
        let rect = Self::rectilinearity(eigenvalues);

        // Planarity: 1 - λ3 / λ2
        let plan = if eigenvalues[1] > 1e-30 {
            1.0 - eigenvalues[2] / eigenvalues[1]
        } else {
            0.0
        };

        PolarizationResult {
            back_azimuth_deg: baz,
            incidence_angle_deg: incidence,
            rectilinearity: rect,
            planarity: plan,
        }
    }

    /// Computes back-azimuth from horizontal component data.
    ///
    /// The back-azimuth is the direction from the receiver toward the source,
    /// measured clockwise from North. Computed from the eigenvector of the
    /// N-E covariance matrix corresponding to the largest eigenvalue.
    ///
    /// # Arguments
    /// * `north` - North component signal
    /// * `east` - East component signal
    ///
    /// # Returns
    /// Back-azimuth in degrees (0-360, 0=North, 90=East)
    pub fn back_azimuth(north: &[f64], east: &[f64]) -> f64 {
        let n = north.len().min(east.len());
        if n == 0 {
            return 0.0;
        }

        let mean_n: f64 = north[..n].iter().sum::<f64>() / n as f64;
        let mean_e: f64 = east[..n].iter().sum::<f64>() / n as f64;

        // 2x2 covariance matrix
        let mut cnn = 0.0f64;
        let mut cne = 0.0f64;
        let mut cee = 0.0f64;

        for i in 0..n {
            let dn = north[i] - mean_n;
            let de = east[i] - mean_e;
            cnn += dn * dn;
            cne += dn * de;
            cee += de * de;
        }
        cnn /= n as f64;
        cne /= n as f64;
        cee /= n as f64;

        // Dominant eigenvector of 2x2 symmetric matrix
        // eigenvalues: 0.5*((cnn+cee) +/- sqrt((cnn-cee)^2 + 4*cne^2))
        let trace = cnn + cee;
        let det = cnn * cee - cne * cne;
        let disc = (trace * trace - 4.0 * det).max(0.0).sqrt();
        let lambda1 = 0.5 * (trace + disc);

        // Eigenvector for lambda1: (cne, lambda1 - cnn) or (lambda1 - cee, cne)
        let (vn, ve) = if cne.abs() > 1e-30 {
            (cne, lambda1 - cnn)
        } else if cnn >= cee {
            (1.0, 0.0)
        } else {
            (0.0, 1.0)
        };

        // atan2(east, north) gives angle from north
        let mut azimuth = ve.atan2(vn).to_degrees();
        if azimuth < 0.0 {
            azimuth += 360.0;
        }
        // Ensure we pick the direction (not 180 deg ambiguity) toward the source
        // Convention: back-azimuth is 0-360
        azimuth % 360.0
    }

    /// Computes incidence angle from horizontal and vertical components.
    ///
    /// The incidence angle is measured from the vertical (0 = straight down,
    /// 90 = horizontal).
    ///
    /// # Arguments
    /// * `horizontal` - Horizontal component amplitude (sqrt(N^2 + E^2))
    /// * `vertical` - Vertical component amplitude
    ///
    /// # Returns
    /// Incidence angle in degrees (0-90)
    pub fn incidence_angle(horizontal: &[f64], vertical: &[f64]) -> f64 {
        let n = horizontal.len().min(vertical.len());
        if n == 0 {
            return 0.0;
        }

        let h_energy: f64 = horizontal[..n].iter().map(|x| x * x).sum();
        let v_energy: f64 = vertical[..n].iter().map(|x| x * x).sum();

        let h_rms = (h_energy / n as f64).sqrt();
        let v_rms = (v_energy / n as f64).sqrt();

        h_rms.atan2(v_rms).to_degrees()
    }

    /// Computes rectilinearity from eigenvalues of the covariance matrix.
    ///
    /// Rectilinearity = 1 - (lambda2 + lambda3) / (2 * lambda1)
    ///
    /// - 0: perfectly circular/random polarization
    /// - 1: perfectly linear polarization (pure body wave)
    ///
    /// # Arguments
    /// * `eigenvalues` - Sorted eigenvalues [lambda1, lambda2, lambda3] (descending)
    ///
    /// # Returns
    /// Rectilinearity value in [0, 1]
    pub fn rectilinearity(eigenvalues: [f64; 3]) -> f64 {
        if eigenvalues[0] < 1e-30 {
            return 0.0;
        }
        let r = 1.0 - (eigenvalues[1] + eigenvalues[2]) / (2.0 * eigenvalues[0]);
        r.clamp(0.0, 1.0)
    }
}

/// Microseismic event magnitude estimator.
///
/// Provides methods for estimating event magnitude, source parameters, and
/// radiated energy using Brune's source model and standard magnitude scales.
pub struct MagnitudeEstimator;

impl MagnitudeEstimator {
    /// Computes moment magnitude from seismic moment.
    ///
    /// Mw = (2/3) * log10(M0) - 6.07
    ///
    /// where M0 is in Newton-meters (N*m).
    ///
    /// # Arguments
    /// * `seismic_moment_nm` - Seismic moment in N*m
    ///
    /// # Returns
    /// Moment magnitude Mw
    pub fn moment_magnitude(seismic_moment_nm: f64) -> f64 {
        (2.0 / 3.0) * seismic_moment_nm.log10() - 6.07
    }

    /// Estimates local magnitude with distance correction.
    ///
    /// ML = log10(amplitude) + 1.11 * log10(distance_km) + 0.00189 * distance_km - 2.09
    ///
    /// This is a simplified Richter-type local magnitude formula adapted
    /// for microseismic monitoring at short distances.
    ///
    /// # Arguments
    /// * `amplitude` - Peak displacement amplitude in micrometers
    /// * `distance_m` - Source-receiver distance in meters
    ///
    /// # Returns
    /// Local magnitude ML
    pub fn local_magnitude(amplitude: f64, distance_m: f64) -> f64 {
        let distance_km = distance_m / 1000.0;
        if amplitude <= 0.0 || distance_km <= 0.0 {
            return f64::NEG_INFINITY;
        }
        amplitude.log10() + 1.11 * distance_km.log10() + 0.00189 * distance_km - 2.09
    }

    /// Computes corner frequency from seismic moment using Brune's model.
    ///
    /// fc = 0.4906 * (stress_drop / M0)^(1/3) * vs
    ///
    /// where the constant comes from the Brune (1970) circular crack model:
    /// fc = (16 * Δσ / (7 * M0))^(1/3) * (2.34 * vs / (2π))^... simplified:
    /// r = 2.34 * vs / (2 * π * fc), so fc = 2.34 * vs / (2 * π * r)
    /// and M0 = (16/7) * Δσ * r^3
    ///
    /// # Arguments
    /// * `moment_nm` - Seismic moment in N*m
    /// * `stress_drop_pa` - Stress drop in Pascals (typically 0.1-10 MPa)
    ///
    /// # Returns
    /// Corner frequency in Hz
    pub fn corner_frequency(moment_nm: f64, stress_drop_pa: f64) -> f64 {
        if moment_nm <= 0.0 || stress_drop_pa <= 0.0 {
            return 0.0;
        }
        // From Brune model: r = (7*M0/(16*Δσ))^(1/3)
        // fc = 2.34*vs/(2*π*r), but vs cancels in the combined formula
        // Use the standard form: fc = 0.4906 * (Δσ/M0)^(1/3) * vs
        // Actually we need vs, but the typical form is:
        // r = (7*M0/(16*Δσ))^(1/3)
        // Return this as the "Brune radius" intermediary
        // For corner_frequency, we need vs. Use a typical value or return parameterized.

        // Standard Brune relation in terms of source radius:
        // M0 = (16/7) * Δσ * r^3  →  r = (7*M0/(16*Δσ))^(1/3)
        // fc = kc * Vs / r  where kc ≈ 0.3724 (Brune, 1970)
        //
        // Without Vs, return the radius-independent formula:
        // fc ∝ (Δσ/M0)^(1/3)
        // We use a reference Vs of 3000 m/s if not provided
        let vs = 3000.0;
        let r = (7.0 * moment_nm / (16.0 * stress_drop_pa)).powf(1.0 / 3.0);
        2.34 * vs / (2.0 * std::f64::consts::PI * r)
    }

    /// Computes source radius from corner frequency using Brune's model.
    ///
    /// r = 2.34 * Vs / (2 * π * fc)
    ///
    /// # Arguments
    /// * `corner_freq` - Corner frequency in Hz
    /// * `vs` - S-wave velocity in m/s
    ///
    /// # Returns
    /// Source radius in meters
    pub fn source_radius(corner_freq: f64, vs: f64) -> f64 {
        if corner_freq <= 0.0 {
            return 0.0;
        }
        2.34 * vs / (2.0 * std::f64::consts::PI * corner_freq)
    }

    /// Estimates radiated seismic energy from moment and stress drop.
    ///
    /// Er = (Δσ / (2 * μ)) * M0
    ///
    /// where μ is the shear modulus. For typical crustal rocks, μ ≈ 30 GPa.
    /// Simplified: Er ≈ Δσ * M0 / (2 * μ)
    ///
    /// # Arguments
    /// * `moment_nm` - Seismic moment in N*m
    /// * `stress_drop_pa` - Stress drop in Pascals
    ///
    /// # Returns
    /// Radiated energy in Joules
    pub fn radiated_energy(moment_nm: f64, stress_drop_pa: f64) -> f64 {
        let shear_modulus = 30.0e9; // 30 GPa typical for shale
        stress_drop_pa * moment_nm / (2.0 * shear_modulus)
    }
}

// ============================================================================
// Helper functions
// ============================================================================

/// Computes 3D Euclidean distance between two points.
///
/// # Arguments
/// * `a` - First point [x, y, z]
/// * `b` - Second point [x, y, z]
///
/// # Returns
/// Distance in the same units as the input coordinates
pub fn distance_3d(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Computes Vp/Vs ratio from Poisson's ratio.
///
/// Vp/Vs = sqrt((1 - ν) / (0.5 - ν))
///
/// For a Poisson solid (ν = 0.25), Vp/Vs = sqrt(3) ≈ 1.732
///
/// # Arguments
/// * `poisson_ratio` - Poisson's ratio (typically 0.15-0.35 for rocks)
///
/// # Returns
/// Vp/Vs velocity ratio
pub fn vp_vs_ratio_from_poisson(poisson_ratio: f64) -> f64 {
    if poisson_ratio >= 0.5 || poisson_ratio < 0.0 {
        return f64::INFINITY;
    }
    ((1.0 - poisson_ratio) / (0.5 - poisson_ratio)).sqrt()
}

// ============================================================================
// Internal helper functions
// ============================================================================

/// Computes variance of a slice.
fn variance(data: &[f64]) -> f64 {
    let n = data.len();
    if n < 2 {
        return 0.0;
    }
    let mean = data.iter().sum::<f64>() / n as f64;
    data.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / n as f64
}

/// Simple linear regression: y = slope * x + intercept.
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len().min(y.len());
    if n < 2 {
        return (0.0, 0.0);
    }

    let nf = n as f64;
    let sum_x: f64 = x[..n].iter().sum();
    let sum_y: f64 = y[..n].iter().sum();
    let sum_xy: f64 = (0..n).map(|i| x[i] * y[i]).sum();
    let sum_xx: f64 = x[..n].iter().map(|xi| xi * xi).sum();

    let denom = nf * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / nf);
    }

    let slope = (nf * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / nf;

    (slope, intercept)
}

/// Solves a 4x4 linear system using Gaussian elimination with partial pivoting.
fn solve_4x4(a: &[[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    let mut aug = [[0.0f64; 5]; 4];
    for i in 0..4 {
        for j in 0..4 {
            aug[i][j] = a[i][j];
        }
        aug[i][4] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..4 {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..4 {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }

        if max_val < 1e-20 {
            return None;
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate below
        for row in (col + 1)..4 {
            let factor = aug[row][col] / aug[col][col];
            for j in col..5 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 4];
    for i in (0..4).rev() {
        x[i] = aug[i][4];
        for j in (i + 1)..4 {
            x[i] -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-20 {
            return None;
        }
        x[i] /= aug[i][i];
    }

    Some(x)
}

/// Computes eigenvalues of a 3x3 symmetric matrix using the analytical formula.
/// Returns sorted eigenvalues in descending order [λ1 >= λ2 >= λ3].
fn eigenvalues_3x3_symmetric(m: &[[f64; 3]; 3]) -> [f64; 3] {
    // For a 3x3 symmetric matrix, use Cardano's formula
    let a = m[0][0];
    let b = m[1][1];
    let c = m[2][2];
    let d = m[0][1]; // = m[1][0]
    let e = m[0][2]; // = m[2][0]
    let f = m[1][2]; // = m[2][1]

    // Characteristic equation: λ^3 - p*λ^2 + q*λ - r = 0
    let p = a + b + c; // trace
    let q = a * b + a * c + b * c - d * d - e * e - f * f;
    let r = a * b * c + 2.0 * d * e * f - a * f * f - b * e * e - c * d * d; // determinant

    // Substitution: λ = t + p/3
    let p3 = p / 3.0;
    let q2 = (p * p - 3.0 * q) / 9.0;
    let r2 = (2.0 * p * p * p - 9.0 * p * q + 27.0 * r) / 54.0;

    let q3 = q2 * q2 * q2;

    let mut eigenvalues = if q2 < 1e-30 {
        // Nearly degenerate: all eigenvalues equal to trace/3
        [p3, p3, p3]
    } else if r2 * r2 <= q3 * (1.0 + 1e-10) {
        // Three real roots (symmetric matrix always has real eigenvalues)
        // Standard trigonometric solution for symmetric 3x3 eigenvalues
        let theta = (r2 / q3.sqrt()).clamp(-1.0, 1.0).acos();
        let sqrt_q2 = q2.sqrt();
        let e1 = p3 + 2.0 * sqrt_q2 * (theta / 3.0).cos();
        let e2 = p3 + 2.0 * sqrt_q2 * ((theta - 2.0 * std::f64::consts::PI) / 3.0).cos();
        let e3 = p3 + 2.0 * sqrt_q2 * ((theta + 2.0 * std::f64::consts::PI) / 3.0).cos();
        [e1, e2, e3]
    } else {
        // Fallback for numerical edge cases
        let theta = (r2 / q3.sqrt().max(1e-30)).clamp(-1.0, 1.0).acos();
        let sqrt_q2 = q2.sqrt();
        let e1 = p3 + 2.0 * sqrt_q2 * (theta / 3.0).cos();
        let e2 = p3 + 2.0 * sqrt_q2 * ((theta - 2.0 * std::f64::consts::PI) / 3.0).cos();
        let e3 = p3 + 2.0 * sqrt_q2 * ((theta + 2.0 * std::f64::consts::PI) / 3.0).cos();
        [e1, e2, e3]
    };

    // Sort descending
    eigenvalues.sort_by(|a, b| b.partial_cmp(a).unwrap_or(std::cmp::Ordering::Equal));

    // Clamp to non-negative (covariance eigenvalues should be >= 0)
    for ev in eigenvalues.iter_mut() {
        if *ev < 0.0 {
            *ev = 0.0;
        }
    }

    eigenvalues
}

/// Computes the dominant eigenvector of a 3x3 symmetric matrix using power iteration.
fn dominant_eigenvector_3x3(m: &[[f64; 3]; 3], _eigenvalue: f64) -> [f64; 3] {
    let mut v = [1.0f64, 1.0, 1.0];
    let norm = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    for c in v.iter_mut() {
        *c /= norm;
    }

    for _ in 0..50 {
        // Matrix-vector multiply
        let mut w = [0.0f64; 3];
        for i in 0..3 {
            for j in 0..3 {
                w[i] += m[i][j] * v[j];
            }
        }

        let norm = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        if norm < 1e-30 {
            return [1.0, 0.0, 0.0];
        }
        for i in 0..3 {
            v[i] = w[i] / norm;
        }
    }

    v
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // ---- Distance helper ----

    #[test]
    fn test_distance_3d_zero() {
        let a = [1.0, 2.0, 3.0];
        assert!((distance_3d(a, a)).abs() < EPSILON);
    }

    #[test]
    fn test_distance_3d_known() {
        let a = [0.0, 0.0, 0.0];
        let b = [3.0, 4.0, 0.0];
        assert!((distance_3d(a, b) - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_distance_3d_full() {
        let a = [1.0, 2.0, 3.0];
        let b = [4.0, 6.0, 3.0];
        let expected = (9.0 + 16.0 + 0.0_f64).sqrt(); // 5.0
        assert!((distance_3d(a, b) - expected).abs() < EPSILON);
    }

    // ---- Vp/Vs from Poisson's ratio ----

    #[test]
    fn test_vp_vs_from_poisson_025() {
        // Poisson solid: ν = 0.25 → Vp/Vs = sqrt(3) ≈ 1.7321
        let ratio = vp_vs_ratio_from_poisson(0.25);
        assert!((ratio - 3.0_f64.sqrt()).abs() < 1e-4);
    }

    #[test]
    fn test_vp_vs_from_poisson_zero() {
        // ν = 0 → Vp/Vs = sqrt(2) ≈ 1.4142
        let ratio = vp_vs_ratio_from_poisson(0.0);
        assert!((ratio - 2.0_f64.sqrt()).abs() < 1e-4);
    }

    #[test]
    fn test_vp_vs_from_poisson_invalid() {
        // ν = 0.5 → infinite (incompressible)
        let ratio = vp_vs_ratio_from_poisson(0.5);
        assert!(ratio.is_infinite());
    }

    #[test]
    fn test_vp_vs_from_poisson_negative() {
        let ratio = vp_vs_ratio_from_poisson(-0.1);
        assert!(ratio.is_infinite());
    }

    // ---- STA/LTA ----

    #[test]
    fn test_sta_lta_spike_at_onset() {
        let config = MicroseismicConfig::default_config();
        let locator = MicroseismicLocator::new(config);

        // Quiet background then sudden large signal
        let mut signal = vec![0.01; 4000];
        for s in signal.iter_mut().skip(2000) {
            *s = 1.0;
        }

        let ratio = locator.sta_lta(&signal, 40, 400);

        // Find max ratio - should be near the onset at sample 2000
        let max_idx = ratio
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;

        // Max should be within 100 samples of the onset
        assert!(
            (max_idx as i64 - 2040).unsigned_abs() < 100,
            "STA/LTA peak at {} should be near onset ~2040",
            max_idx
        );

        // The ratio at the peak should be >> 1
        assert!(ratio[max_idx] > 5.0, "Peak ratio {} should exceed 5", ratio[max_idx]);
    }

    #[test]
    fn test_sta_lta_flat_signal() {
        let config = MicroseismicConfig::default_config();
        let locator = MicroseismicLocator::new(config);

        let signal = vec![1.0; 2000];
        let ratio = locator.sta_lta(&signal, 40, 400);

        // For a constant signal, STA/LTA should be ~1.0 everywhere
        for (i, &r) in ratio.iter().enumerate().skip(500) {
            assert!(
                (r - 1.0).abs() < 0.5,
                "Flat signal STA/LTA at {} = {} should be ~1.0",
                i,
                r
            );
        }
    }

    #[test]
    fn test_sta_lta_short_signal() {
        let config = MicroseismicConfig::default_config();
        let locator = MicroseismicLocator::new(config);

        let signal = vec![1.0; 10];
        let ratio = locator.sta_lta(&signal, 40, 400);
        // Should return all 1.0 for too-short signal
        assert_eq!(ratio.len(), 10);
        for &r in &ratio {
            assert!((r - 1.0).abs() < EPSILON);
        }
    }

    // ---- Event detection ----

    #[test]
    fn test_detect_events_single() {
        let config = MicroseismicConfig::default_config();
        let locator = MicroseismicLocator::new(config);

        let mut ratio = vec![1.0; 1000];
        ratio[500] = 10.0;

        let triggers = locator.detect_events(&ratio, 5.0, 100);
        assert_eq!(triggers.len(), 1);
        assert_eq!(triggers[0], 500);
    }

    #[test]
    fn test_detect_events_holdoff() {
        let config = MicroseismicConfig::default_config();
        let locator = MicroseismicLocator::new(config);

        let mut ratio = vec![1.0; 1000];
        ratio[100] = 10.0;
        ratio[110] = 10.0; // Within holdoff
        ratio[500] = 10.0; // Outside holdoff

        let triggers = locator.detect_events(&ratio, 5.0, 200);
        assert_eq!(triggers.len(), 2);
        assert_eq!(triggers[0], 100);
        assert_eq!(triggers[1], 500);
    }

    #[test]
    fn test_detect_events_none_below_threshold() {
        let config = MicroseismicConfig::default_config();
        let locator = MicroseismicLocator::new(config);

        let ratio = vec![2.0; 1000];
        let triggers = locator.detect_events(&ratio, 5.0, 100);
        assert!(triggers.is_empty());
    }

    // ---- AIC picker ----

    #[test]
    fn test_aic_picker_known_onset() {
        // Create signal: noise then step
        let mut signal = vec![0.0; 200];
        // Low-amplitude noise
        for i in 0..100 {
            signal[i] = 0.01 * ((i as f64 * 0.1).sin());
        }
        // Higher-amplitude signal
        for i in 100..200 {
            signal[i] = 1.0 * ((i as f64 * 0.5).sin());
        }

        let onset = MicroseismicLocator::aic_picker(&signal);
        // AIC minimum should be near sample 100
        assert!(
            (onset as i64 - 100).unsigned_abs() < 20,
            "AIC onset {} should be near 100",
            onset
        );
    }

    #[test]
    fn test_aic_picker_short_signal() {
        let signal = vec![1.0, 2.0];
        let onset = MicroseismicLocator::aic_picker(&signal);
        assert_eq!(onset, 0); // Too short to pick
    }

    #[test]
    fn test_pick_p_arrival_returns_seconds() {
        let mut config = MicroseismicConfig::default_config();
        config.sample_rate_hz = 1000.0;
        let locator = MicroseismicLocator::new(config);

        let mut signal = vec![0.001; 2000];
        for s in signal.iter_mut().skip(1000) {
            *s = 1.0;
        }

        let p_time = locator.pick_p_arrival(&signal, 1000, 200);
        // Should return time near 1.0 seconds (sample 1000 / 1000 Hz)
        assert!(
            (p_time - 1.0).abs() < 0.3,
            "P arrival {} should be near 1.0 s",
            p_time
        );
    }

    // ---- Travel time ----

    #[test]
    fn test_travel_time_simple() {
        let source = [0.0, 0.0, 0.0];
        let receiver = [5000.0, 0.0, 0.0];
        let velocity = 5000.0;

        let tt = EventLocator::travel_time(source, receiver, velocity);
        assert!((tt - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_travel_time_3d() {
        let source = [0.0, 0.0, 0.0];
        let receiver = [3000.0, 4000.0, 0.0];
        let velocity = 5000.0;

        let tt = EventLocator::travel_time(source, receiver, velocity);
        assert!((tt - 1.0).abs() < EPSILON); // distance=5000, velocity=5000
    }

    // ---- Grid search location ----

    #[test]
    fn test_grid_search_locates_known_source() {
        // Place receivers in a 3D array for good geometry
        let receivers = vec![
            [0.0, 0.0, 2000.0],
            [0.0, 0.0, 2030.0],
            [0.0, 0.0, 2060.0],
            [0.0, 0.0, 2090.0],
            [0.0, 0.0, 2120.0],
            [0.0, 0.0, 2150.0],
            [0.0, 0.0, 2180.0],
            [0.0, 0.0, 2210.0],
        ];

        // Source at a moderate distance with clear depth differentiation
        let true_source = [100.0, 0.0, 2100.0];
        let vp = 5000.0;

        // Generate synthetic P arrival times
        let origin_time = 0.0;
        let p_times: Vec<f64> = receivers
            .iter()
            .map(|r| origin_time + EventLocator::travel_time(true_source, *r, vp))
            .collect();

        let bounds = ([-50.0, -50.0, 1950.0], [250.0, 50.0, 2250.0]);
        let location =
            EventLocator::locate_grid_search(&p_times, &receivers, vp, bounds, 10.0);

        let error = distance_3d(location, true_source);
        assert!(
            error < 50.0,
            "Grid search error {} m should be < 50 m",
            error
        );
    }

    // ---- Geiger inversion ----

    #[test]
    fn test_geiger_refines_location() {
        // Use wider vertical aperture (240 m) for better geometry
        let mut receivers = Vec::new();
        for i in 0..12 {
            receivers.push([0.0, 0.0, 2000.0 + i as f64 * 20.0]);
        }

        let true_source = [100.0, 0.0, 2100.0];
        let vp = 5000.0;
        let vs = 2900.0;
        let origin_time = 0.5;

        let p_times: Vec<f64> = receivers
            .iter()
            .map(|r| origin_time + EventLocator::travel_time(true_source, *r, vp))
            .collect();
        let s_times: Vec<f64> = receivers
            .iter()
            .map(|r| origin_time + EventLocator::travel_time(true_source, *r, vs))
            .collect();

        let locator = EventLocator::new(receivers, vp, vs);

        // Start with a close initial guess
        let guess = [80.0, 0.0, 2080.0];
        let event = locator.locate_geiger(&p_times, &s_times, guess);

        let error = distance_3d(event.location, true_source);
        assert!(
            error < 50.0,
            "Geiger location error {} m should be < 50 m",
            error
        );
        assert!(
            (event.origin_time_s - origin_time).abs() < 0.05,
            "Origin time error {} should be < 0.05 s",
            (event.origin_time_s - origin_time).abs()
        );
    }

    // ---- Wadati diagram ----

    #[test]
    fn test_wadati_vp_vs_ratio() {
        let vp = 5000.0;
        let vs = 2900.0;
        let true_ratio = vp / vs;
        let origin_time = 0.1;

        // Generate consistent P and S times
        let distances = [500.0, 800.0, 1200.0, 1500.0, 2000.0];
        let p_times: Vec<f64> = distances.iter().map(|d| origin_time + d / vp).collect();
        let s_times: Vec<f64> = distances.iter().map(|d| origin_time + d / vs).collect();

        let (ratio, t0) = EventLocator::wadati_diagram(&p_times, &s_times);

        assert!(
            (ratio - true_ratio).abs() < 0.05,
            "Wadati Vp/Vs {} should be near {}",
            ratio,
            true_ratio
        );
        assert!(
            (t0 - origin_time).abs() < 0.02,
            "Wadati origin time {} should be near {}",
            t0,
            origin_time
        );
    }

    #[test]
    fn test_wadati_insufficient_data() {
        let (ratio, _t0) = EventLocator::wadati_diagram(&[1.0], &[2.0]);
        assert!((ratio - 1.73).abs() < 0.01); // default value
    }

    // ---- Residual RMS ----

    #[test]
    fn test_residual_rms_perfect_fit() {
        let a = vec![1.0, 2.0, 3.0, 4.0];
        let b = vec![1.0, 2.0, 3.0, 4.0];
        assert!(residual_rms(&a, &b) < EPSILON);
    }

    #[test]
    fn test_residual_rms_known_value() {
        let predicted = vec![1.0, 2.0, 3.0];
        let observed = vec![1.1, 2.1, 3.1];
        let rms = residual_rms(&predicted, &observed);
        // All residuals are -0.1, RMS = 0.1
        assert!((rms - 0.1).abs() < EPSILON);
    }

    #[test]
    fn test_residual_rms_empty() {
        assert!(residual_rms(&[], &[]).abs() < EPSILON);
    }

    // ---- Hodogram / Polarization ----

    #[test]
    fn test_back_azimuth_pure_east() {
        let n = 100;
        let north = vec![0.0; n];
        let east: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();

        let baz = HodogramAnalyzer::back_azimuth(&north, &east);
        // Pure East motion → azimuth should be 90° (or 270° due to ambiguity)
        let min_diff = (baz - 90.0).abs().min((baz - 270.0).abs());
        assert!(
            min_diff < 5.0,
            "Pure East back-azimuth {} should be near 90 or 270 deg",
            baz
        );
    }

    #[test]
    fn test_back_azimuth_pure_north() {
        let n = 100;
        let north: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
        let east = vec![0.0; n];

        let baz = HodogramAnalyzer::back_azimuth(&north, &east);
        // Pure North motion → azimuth should be 0° (or 180°)
        let min_diff = baz.min(360.0 - baz).min((baz - 180.0).abs());
        assert!(
            min_diff < 5.0,
            "Pure North back-azimuth {} should be near 0 or 180 deg",
            baz
        );
    }

    #[test]
    fn test_rectilinearity_linear_polarization() {
        // Lambda1 >> lambda2, lambda3 → rectilinearity ~ 1
        let eigenvalues = [10.0, 0.0, 0.0];
        let rect = HodogramAnalyzer::rectilinearity(eigenvalues);
        assert!(
            (rect - 1.0).abs() < EPSILON,
            "Linear polarization rectilinearity {} should be 1.0",
            rect
        );
    }

    #[test]
    fn test_rectilinearity_circular_polarization() {
        // All eigenvalues equal → rectilinearity = 0
        let eigenvalues = [1.0, 1.0, 1.0];
        let rect = HodogramAnalyzer::rectilinearity(eigenvalues);
        assert!(
            rect.abs() < EPSILON,
            "Circular polarization rectilinearity {} should be 0",
            rect
        );
    }

    #[test]
    fn test_polarization_analysis_linear() {
        let analyzer = HodogramAnalyzer::new();
        let n = 200;

        // Purely vertical motion (linear P-wave)
        let north = vec![0.0; n];
        let east = vec![0.0; n];
        let vertical: Vec<f64> = (0..n).map(|i| (i as f64 * 0.3).sin()).collect();

        let result = analyzer.polarization_analysis(&north, &east, &vertical);
        assert!(
            result.rectilinearity > 0.9,
            "Vertical linear motion rectilinearity {} should be > 0.9",
            result.rectilinearity
        );
    }

    #[test]
    fn test_incidence_angle_vertical() {
        let n = 100;
        let horizontal = vec![0.0; n]; // No horizontal
        let vertical: Vec<f64> = (0..n).map(|i| (i as f64 * 0.2).sin()).collect();

        let angle = HodogramAnalyzer::incidence_angle(&horizontal, &vertical);
        assert!(
            angle < 5.0,
            "Vertical incidence angle {} should be near 0",
            angle
        );
    }

    #[test]
    fn test_incidence_angle_horizontal() {
        let n = 100;
        let horizontal: Vec<f64> = (0..n).map(|i| (i as f64 * 0.2).sin()).collect();
        let vertical = vec![0.0; n];

        let angle = HodogramAnalyzer::incidence_angle(&horizontal, &vertical);
        assert!(
            (angle - 90.0).abs() < 5.0,
            "Horizontal incidence angle {} should be near 90",
            angle
        );
    }

    // ---- Magnitude estimator ----

    #[test]
    fn test_moment_magnitude_m0_1e10() {
        // M0 = 1e10 N*m → Mw = (2/3)*10 - 6.07 = 6.667 - 6.07 = 0.597
        let mw = MagnitudeEstimator::moment_magnitude(1e10);
        assert!(
            (mw - 0.597).abs() < 0.01,
            "Mw for M0=1e10 should be ~0.597, got {}",
            mw
        );
    }

    #[test]
    fn test_moment_magnitude_m0_1e15() {
        // M0 = 1e15 → Mw = (2/3)*15 - 6.07 = 10.0 - 6.07 = 3.93
        let mw = MagnitudeEstimator::moment_magnitude(1e15);
        assert!(
            (mw - 3.93).abs() < 0.01,
            "Mw for M0=1e15 should be ~3.93, got {}",
            mw
        );
    }

    #[test]
    fn test_local_magnitude_positive() {
        let ml = MagnitudeEstimator::local_magnitude(100.0, 500.0);
        // Should return a finite value
        assert!(ml.is_finite());
    }

    #[test]
    fn test_local_magnitude_zero_amplitude() {
        let ml = MagnitudeEstimator::local_magnitude(0.0, 500.0);
        assert!(ml.is_infinite() && ml < 0.0);
    }

    #[test]
    fn test_source_radius_from_corner_freq() {
        let vs = 3000.0;
        let fc = 100.0; // 100 Hz corner frequency
        let r = MagnitudeEstimator::source_radius(fc, vs);
        // r = 2.34 * 3000 / (2π * 100) = 7020 / 628.318 ≈ 11.17 m
        let expected = 2.34 * vs / (2.0 * std::f64::consts::PI * fc);
        assert!(
            (r - expected).abs() < 0.01,
            "Source radius {} should be ~{} m",
            r,
            expected
        );
    }

    #[test]
    fn test_source_radius_zero_freq() {
        assert_eq!(MagnitudeEstimator::source_radius(0.0, 3000.0), 0.0);
    }

    #[test]
    fn test_corner_frequency_positive() {
        let fc = MagnitudeEstimator::corner_frequency(1e8, 1e6);
        assert!(fc > 0.0, "Corner frequency should be positive: {}", fc);
    }

    #[test]
    fn test_corner_frequency_zero_moment() {
        assert_eq!(MagnitudeEstimator::corner_frequency(0.0, 1e6), 0.0);
    }

    #[test]
    fn test_radiated_energy_positive() {
        let energy = MagnitudeEstimator::radiated_energy(1e10, 1e6);
        assert!(energy > 0.0, "Radiated energy should be positive: {}", energy);
    }

    #[test]
    fn test_radiated_energy_formula() {
        let m0 = 1e10;
        let stress = 1e6;
        let mu = 30.0e9;
        let expected = stress * m0 / (2.0 * mu);
        let energy = MagnitudeEstimator::radiated_energy(m0, stress);
        assert!(
            (energy - expected).abs() / expected < EPSILON,
            "Energy {} should match formula {}",
            energy,
            expected
        );
    }

    // ---- Variance helper ----

    #[test]
    fn test_variance_constant() {
        let data = vec![5.0; 100];
        assert!(variance(&data) < EPSILON);
    }

    #[test]
    fn test_variance_known() {
        // Variance of [1, 2, 3, 4, 5]: mean=3, var = (4+1+0+1+4)/5 = 2.0
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert!((variance(&data) - 2.0).abs() < EPSILON);
    }

    // ---- Linear regression helper ----

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // y = 2x
        let (slope, intercept) = linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < EPSILON);
        assert!(intercept.abs() < EPSILON);
    }

    #[test]
    fn test_linear_regression_with_offset() {
        let x = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let y = vec![1.0, 3.0, 5.0, 7.0, 9.0]; // y = 2x + 1
        let (slope, intercept) = linear_regression(&x, &y);
        assert!((slope - 2.0).abs() < EPSILON);
        assert!((intercept - 1.0).abs() < EPSILON);
    }

    // ---- Eigenvalue computation ----

    #[test]
    fn test_eigenvalues_diagonal() {
        let m = [[3.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 1.0]];
        let ev = eigenvalues_3x3_symmetric(&m);
        assert!((ev[0] - 3.0).abs() < EPSILON);
        assert!((ev[1] - 2.0).abs() < EPSILON);
        assert!((ev[2] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_eigenvalues_identity() {
        let m = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let ev = eigenvalues_3x3_symmetric(&m);
        for &e in &ev {
            assert!((e - 1.0).abs() < EPSILON);
        }
    }

    // ---- Config default ----

    #[test]
    fn test_default_config() {
        let config = MicroseismicConfig::default_config();
        assert_eq!(config.num_receivers, 12);
        assert_eq!(config.receiver_positions.len(), 12);
        assert!(config.vp_m_s > config.vs_m_s);
        assert!(config.sample_rate_hz > 0.0);
    }

    // ---- S-wave picking ----

    #[test]
    fn test_pick_s_arrival_returns_seconds() {
        let mut config = MicroseismicConfig::default_config();
        config.sample_rate_hz = 1000.0;
        let locator = MicroseismicLocator::new(config);

        let n = 2000;
        let mut signal_3c = vec![[0.001, 0.001, 0.01]; n];
        // P-wave: strong vertical, weak horizontal (linear)
        for i in 500..800 {
            signal_3c[i] = [0.001, 0.001, 1.0 * ((i as f64 * 0.5).sin())];
        }
        // S-wave: strong horizontal, weak vertical (onset at ~800)
        for i in 800..n {
            signal_3c[i] = [
                1.0 * ((i as f64 * 0.3).sin()),
                1.0 * ((i as f64 * 0.3 + 1.0).sin()),
                0.01,
            ];
        }

        let s_time = locator.pick_s_arrival(&signal_3c, 500, 400);
        // Should be a positive time value
        assert!(s_time > 0.0, "S-wave time should be positive: {}", s_time);
        assert!(s_time < 2.0, "S-wave time should be < 2s: {}", s_time);
    }

    // ---- 4x4 solver ----

    #[test]
    fn test_solve_4x4_identity() {
        let a = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let b = [1.0, 2.0, 3.0, 4.0];
        let x = solve_4x4(&a, &b).unwrap();
        for i in 0..4 {
            assert!((x[i] - b[i]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_solve_4x4_singular() {
        let a = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let b = [1.0, 2.0, 3.0, 4.0];
        assert!(solve_4x4(&a, &b).is_none());
    }
}
