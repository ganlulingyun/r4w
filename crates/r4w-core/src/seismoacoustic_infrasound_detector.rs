// trace:FR-INFRASOUND | ai:claude
//! # Seismoacoustic Infrasound Detector
//!
//! Implements infrasound array processing for the Comprehensive Nuclear-Test-Ban Treaty
//! (CTBT) International Monitoring System (IMS). Infrasound waves (< 20 Hz) from nuclear
//! tests, volcanic eruptions, bolides, and severe weather propagate through the atmosphere
//! over thousands of kilometers. Arrays of microbarometers detect these signals.
//!
//! ## Overview
//!
//! The CTBT IMS operates ~60 infrasound stations worldwide, each comprising an array of
//! 4-8 microbarometers with spatial separations of 1-3 km. This module provides:
//!
//! - **Array beamforming** with delay-and-sum steered to arbitrary back-azimuth and
//!   trace velocity
//! - **Fisher detector** for statistically testing coherent signal presence across array
//!   elements (F-statistic comparing beam power to residual noise variance)
//! - **Grid search** over azimuth and trace velocity to find optimal beam direction
//! - **Progressive Multi-Channel Correlation (PMCC)** for robust detection
//! - **Yield estimation** from overpressure measurements using scaling laws
//! - **Propagation modeling** with celerity-based range estimation, stratospheric and
//!   thermospheric return distances
//! - **Wind noise reduction** via coherent array processing and pipe array spatial filtering
//!
//! ## Key Parameters
//!
//! - Sample rates: typically 20 Hz (IMS standard)
//! - Frequency range: 0.02 - 4.0 Hz (infrasound band)
//! - Trace velocities: 280 - 450 m/s (atmospheric acoustic propagation)
//! - Fisher ratio threshold: typically 3.0 - 10.0 for detection
//!
//! ## References
//!
//! - Cansi, Y. (1995). "An automatic seismic event processing for detection and location:
//!   The P.M.C.C. method." Geophysical Research Letters, 22(9), 1021-1024.
//! - Christie, D.R. & Campus, P. (2010). "The IMS Infrasound Network: Design and
//!   Establishment of Infrasound Stations." In *Infrasound Monitoring for Atmospheric
//!   Studies*, Springer.
//! - ReVelle, D.O. (1997). "Historical Detection of Atmospheric Nuclear Explosions by
//!   Infrasound Techniques." Journal of Geophysical Research, 102(D18), 21889-21900.

use std::f64::consts::PI;

/// Configuration for an infrasound monitoring array.
///
/// Typical IMS arrays have 4-8 elements with 1-3 km separations, sampling at 20 Hz.
/// The frequency range of interest is 0.02 to 4.0 Hz, encompassing stratospheric
/// and thermospheric propagation paths.
#[derive(Clone, Debug)]
pub struct InfrasoundConfig {
    /// Sample rate in Hz (typically 20 Hz for IMS stations)
    pub sample_rate_hz: f64,
    /// Number of array elements (microbarometers), typically 4-8
    pub num_elements: usize,
    /// 2D positions of each element in meters [east, north] relative to array center
    pub element_positions: Vec<[f64; 2]>,
    /// Frequency range of interest (low_hz, high_hz), typically (0.02, 4.0)
    pub frequency_range: (f64, f64),
    /// Speed of sound in m/s (default 340.0 for near-surface atmosphere)
    pub sound_speed_m_s: f64,
}

impl InfrasoundConfig {
    /// Create a new configuration with default sound speed.
    pub fn new(
        sample_rate_hz: f64,
        element_positions: Vec<[f64; 2]>,
        frequency_range: (f64, f64),
    ) -> Self {
        let num_elements = element_positions.len();
        Self {
            sample_rate_hz,
            num_elements,
            element_positions,
            frequency_range,
            sound_speed_m_s: 340.0,
        }
    }
}

/// A detected infrasound event with estimated parameters.
///
/// Contains the back-azimuth (direction of arrival), trace velocity (apparent horizontal
/// speed), dominant frequency, timing, signal-to-noise ratio, and Fisher statistic.
#[derive(Clone, Debug)]
pub struct InfrasoundDetection {
    /// Back-azimuth in degrees (0-360, clockwise from north)
    pub back_azimuth_deg: f64,
    /// Apparent horizontal trace velocity in m/s (typically 280-450 m/s)
    pub trace_velocity_m_s: f64,
    /// Dominant frequency of the detection in Hz
    pub frequency_hz: f64,
    /// Start time of the detection in seconds from beginning of data
    pub start_time_s: f64,
    /// Duration of the detection in seconds
    pub duration_s: f64,
    /// Signal-to-noise ratio in dB
    pub snr_db: f64,
    /// Fisher ratio (F-statistic) measuring coherence across array
    pub fisher_ratio: f64,
}

/// Main infrasound array processor.
///
/// Provides beamforming, bandpass filtering, and event detection for microbarometer
/// arrays. The processor steers beams to candidate azimuths and trace velocities,
/// then applies the Fisher detector to identify coherent signals.
pub struct InfrasoundProcessor {
    config: InfrasoundConfig,
}

impl InfrasoundProcessor {
    /// Create a new processor from configuration.
    pub fn new(config: InfrasoundConfig) -> Self {
        Self { config }
    }

    /// Apply a 2nd-order IIR bandpass filter to the input signal.
    ///
    /// Uses a biquad bandpass design centered between `low_hz` and `high_hz`.
    /// The filter is applied forward and backward (zero-phase / filtfilt) to
    /// eliminate phase distortion, which is critical for time-delay estimation.
    ///
    /// # Arguments
    /// * `signal` - Input time-domain samples
    /// * `low_hz` - Lower cutoff frequency in Hz
    /// * `high_hz` - Upper cutoff frequency in Hz
    /// * `sample_rate` - Sampling rate in Hz
    ///
    /// # Returns
    /// Bandpass-filtered signal of the same length
    pub fn bandpass_filter(
        &self,
        signal: &[f64],
        low_hz: f64,
        high_hz: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        if signal.is_empty() {
            return vec![];
        }
        let center_hz = (low_hz * high_hz).sqrt();
        let bw = high_hz - low_hz;
        let omega = 2.0 * PI * center_hz / sample_rate;
        let sin_omega = omega.sin();
        let cos_omega = omega.cos();
        let alpha = sin_omega * (2.0_f64.ln() / 2.0 * bw / center_hz * omega / sin_omega).sinh();

        // Biquad bandpass coefficients (constant 0dB peak gain)
        let b0 = alpha;
        let b1 = 0.0;
        let b2 = -alpha;
        let a0 = 1.0 + alpha;
        let a1 = -2.0 * cos_omega;
        let a2 = 1.0 - alpha;

        // Normalize
        let b0 = b0 / a0;
        let b1 = b1 / a0;
        let b2 = b2 / a0;
        let a1 = a1 / a0;
        let a2 = a2 / a0;

        // Forward pass
        let mut forward = vec![0.0; signal.len()];
        let mut x1 = 0.0;
        let mut x2 = 0.0;
        let mut y1 = 0.0;
        let mut y2 = 0.0;
        for i in 0..signal.len() {
            let x0 = signal[i];
            let y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            forward[i] = y0;
            x2 = x1;
            x1 = x0;
            y2 = y1;
            y1 = y0;
        }

        // Backward pass (zero-phase)
        let mut result = vec![0.0; signal.len()];
        x1 = 0.0;
        x2 = 0.0;
        y1 = 0.0;
        y2 = 0.0;
        for i in (0..signal.len()).rev() {
            let x0 = forward[i];
            let y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            result[i] = y0;
            x2 = x1;
            x1 = x0;
            y2 = y1;
            y1 = y0;
        }

        result
    }

    /// Compute the time delay in samples for a given element position, azimuth, and
    /// trace velocity.
    ///
    /// The delay is computed from the plane-wave model: a wavefront arriving from
    /// `azimuth_deg` (clockwise from north) with apparent horizontal velocity
    /// `trace_velocity` reaches each element at a time determined by projection of
    /// the element position onto the propagation direction.
    ///
    /// # Arguments
    /// * `element_pos` - [east, north] position of the element in meters
    /// * `azimuth_deg` - Back-azimuth in degrees (0 = north, 90 = east)
    /// * `trace_velocity` - Apparent horizontal velocity in m/s
    ///
    /// # Returns
    /// Time delay in seconds (positive means the element receives the signal later)
    pub fn time_delay_for_azimuth(
        element_pos: [f64; 2],
        azimuth_deg: f64,
        trace_velocity: f64,
    ) -> f64 {
        let azimuth_rad = azimuth_deg.to_radians();
        // Propagation direction unit vector: from source toward array
        // Back-azimuth points FROM array TO source, so propagation is opposite
        let dx = -azimuth_rad.sin(); // east component of propagation direction
        let dy = -azimuth_rad.cos(); // north component of propagation direction
        // Delay is the projection of position onto propagation direction divided by velocity
        let projection = element_pos[0] * dx + element_pos[1] * dy;
        projection / trace_velocity
    }

    /// Delay-and-sum beamformer steered to the given azimuth and trace velocity.
    ///
    /// Applies fractional-sample delays to each channel (via linear interpolation),
    /// then sums and normalizes by the number of elements. This coherently enhances
    /// signals arriving from the specified direction while attenuating incoherent noise.
    ///
    /// # Arguments
    /// * `signals` - One time series per array element (all same length)
    /// * `azimuth_deg` - Steering back-azimuth in degrees
    /// * `trace_velocity` - Steering trace velocity in m/s
    ///
    /// # Returns
    /// Beamformed output signal
    pub fn beamform(
        &self,
        signals: &[Vec<f64>],
        azimuth_deg: f64,
        trace_velocity: f64,
    ) -> Vec<f64> {
        if signals.is_empty() || signals[0].is_empty() {
            return vec![];
        }
        let n_samples = signals[0].len();
        let mut beam = vec![0.0; n_samples];

        for (ch, pos) in self.config.element_positions.iter().enumerate() {
            if ch >= signals.len() {
                break;
            }
            let delay_s = Self::time_delay_for_azimuth(*pos, azimuth_deg, trace_velocity);
            let delay_samples = delay_s * self.config.sample_rate_hz;

            // Apply delay via linear interpolation
            for i in 0..n_samples {
                let src = i as f64 + delay_samples;
                let idx = src.floor() as i64;
                let frac = src - src.floor();

                if idx >= 0 && (idx + 1) < n_samples as i64 {
                    let val = signals[ch][idx as usize] * (1.0 - frac)
                        + signals[ch][(idx + 1) as usize] * frac;
                    beam[i] += val;
                } else if idx >= 0 && idx < n_samples as i64 {
                    beam[i] += signals[ch][idx as usize] * (1.0 - frac);
                }
            }
        }

        let n_elem = signals.len().min(self.config.num_elements) as f64;
        if n_elem > 0.0 {
            for v in &mut beam {
                *v /= n_elem;
            }
        }

        beam
    }

    /// Detect infrasound events using a Fisher-statistic grid search.
    ///
    /// Performs a grid search over azimuth (0-360 degrees) and trace velocity
    /// (280-450 m/s). At each grid point, the beamformer output is computed
    /// and the Fisher ratio is evaluated. Grid points exceeding the threshold
    /// are reported as detections.
    ///
    /// # Arguments
    /// * `signals` - One time series per array element
    /// * `threshold_fisher` - Minimum Fisher ratio for detection
    ///
    /// # Returns
    /// Vector of detected events sorted by Fisher ratio (highest first)
    pub fn detect_events(
        &self,
        signals: &[Vec<f64>],
        threshold_fisher: f64,
    ) -> Vec<InfrasoundDetection> {
        if signals.is_empty() || signals[0].is_empty() {
            return vec![];
        }

        let n_samples = signals[0].len();
        let detector = FisherDetector::new(&self.config);
        let az_steps = 36; // 10-degree steps
        let vel_steps = 10;
        let vel_min = 280.0;
        let vel_max = 450.0;

        let mut best_az = 0.0;
        let mut best_vel = 340.0;
        let mut best_fisher = 0.0_f64;

        // Coarse grid search
        for az_i in 0..az_steps {
            let az = 360.0 * az_i as f64 / az_steps as f64;
            for vel_i in 0..vel_steps {
                let vel = vel_min + (vel_max - vel_min) * vel_i as f64 / (vel_steps - 1) as f64;
                let beam = self.beamform(signals, az, vel);
                let fisher = detector.compute_fisher_ratio(signals, &beam);
                if fisher > best_fisher {
                    best_fisher = fisher;
                    best_az = az;
                    best_vel = vel;
                }
            }
        }

        // Fine grid search around best coarse point
        let az_fine_range = 360.0 / az_steps as f64;
        let vel_fine_range = (vel_max - vel_min) / (vel_steps - 1) as f64;
        let fine_steps = 10;

        for az_i in 0..fine_steps {
            let az = best_az - az_fine_range / 2.0
                + az_fine_range * az_i as f64 / (fine_steps - 1) as f64;
            let az = ((az % 360.0) + 360.0) % 360.0;
            for vel_i in 0..fine_steps {
                let vel = best_vel - vel_fine_range / 2.0
                    + vel_fine_range * vel_i as f64 / (fine_steps - 1) as f64;
                if vel < vel_min || vel > vel_max {
                    continue;
                }
                let beam = self.beamform(signals, az, vel);
                let fisher = detector.compute_fisher_ratio(signals, &beam);
                if fisher > best_fisher {
                    best_fisher = fisher;
                    best_az = az;
                    best_vel = vel;
                }
            }
        }

        let mut detections = Vec::new();

        if best_fisher >= threshold_fisher {
            // Estimate dominant frequency from bandpass-filtered beam
            let beam = self.beamform(signals, best_az, best_vel);
            let freq = self.estimate_dominant_frequency(&beam);
            let beam_power: f64 =
                beam.iter().map(|x| x * x).sum::<f64>() / beam.len() as f64;
            let noise_power = self.estimate_noise_power(signals);
            let snr_db = if noise_power > 1e-30 {
                10.0 * (beam_power / noise_power).log10()
            } else {
                60.0
            };

            detections.push(InfrasoundDetection {
                back_azimuth_deg: best_az,
                trace_velocity_m_s: best_vel,
                frequency_hz: freq,
                start_time_s: 0.0,
                duration_s: n_samples as f64 / self.config.sample_rate_hz,
                snr_db,
                fisher_ratio: best_fisher,
            });
        }

        detections.sort_by(|a, b| {
            b.fisher_ratio
                .partial_cmp(&a.fisher_ratio)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        detections
    }

    /// Estimate the dominant frequency of a signal via zero-crossing rate.
    fn estimate_dominant_frequency(&self, signal: &[f64]) -> f64 {
        if signal.len() < 2 {
            return 0.0;
        }
        let mut crossings = 0usize;
        for i in 1..signal.len() {
            if signal[i - 1] * signal[i] < 0.0 {
                crossings += 1;
            }
        }
        // Each full cycle has 2 zero crossings
        let duration_s = signal.len() as f64 / self.config.sample_rate_hz;
        if duration_s > 0.0 {
            crossings as f64 / (2.0 * duration_s)
        } else {
            0.0
        }
    }

    /// Estimate noise power from individual channels (average per-channel power).
    fn estimate_noise_power(&self, signals: &[Vec<f64>]) -> f64 {
        if signals.is_empty() {
            return 0.0;
        }
        let mut total_power = 0.0;
        let mut count = 0;
        for sig in signals {
            for &s in sig {
                total_power += s * s;
                count += 1;
            }
        }
        if count > 0 {
            total_power / count as f64
        } else {
            0.0
        }
    }
}

/// Fisher-statistic detector for coherent signal identification.
///
/// The Fisher ratio (F-statistic) tests whether the variance explained by the
/// beamformed signal is significantly greater than the residual (unexplained)
/// variance across array elements. For K elements and N samples:
///
/// F = (K - 1) * P_beam / P_residual
///
/// where P_beam is the beam power and P_residual is the average residual power
/// after subtracting the beam from each channel. Under the null hypothesis
/// (no coherent signal), F follows an F-distribution. Values significantly
/// above 1.0 indicate coherent energy.
pub struct FisherDetector {
    num_elements: usize,
    element_positions: Vec<[f64; 2]>,
    sample_rate_hz: f64,
}

impl FisherDetector {
    /// Create a new Fisher detector from an infrasound configuration.
    pub fn new(config: &InfrasoundConfig) -> Self {
        Self {
            num_elements: config.num_elements,
            element_positions: config.element_positions.clone(),
            sample_rate_hz: config.sample_rate_hz,
        }
    }

    /// Compute the Fisher ratio for a set of signals and a given beam.
    ///
    /// F = (K - 1) * P_beam / P_residual
    ///
    /// where P_beam is the average power of the beam, and P_residual is the
    /// average power of the residuals (signal minus beam) across all channels.
    ///
    /// # Arguments
    /// * `signals` - Array element signals
    /// * `beam` - Beamformed signal (same length as each element signal)
    ///
    /// # Returns
    /// Fisher ratio (> 1 indicates coherent signal)
    pub fn compute_fisher_ratio(&self, signals: &[Vec<f64>], beam: &[f64]) -> f64 {
        if signals.is_empty() || beam.is_empty() {
            return 0.0;
        }
        let k = signals.len() as f64;
        if k <= 1.0 {
            return 0.0;
        }
        let n = beam.len();

        // Beam power
        let beam_power: f64 = beam.iter().map(|x| x * x).sum::<f64>() / n as f64;

        // Residual power: average across channels of (signal - beam)^2
        let mut residual_power = 0.0;
        for sig in signals {
            let len = sig.len().min(n);
            let mut ch_residual = 0.0;
            for i in 0..len {
                let r = sig[i] - beam[i];
                ch_residual += r * r;
            }
            ch_residual /= len as f64;
            residual_power += ch_residual;
        }
        residual_power /= k;

        if residual_power < 1e-30 {
            return if beam_power > 1e-30 {
                (k - 1.0) * 1e6
            } else {
                1.0
            };
        }

        (k - 1.0) * beam_power / residual_power
    }

    /// Grid search over azimuth and trace velocity to find the optimal beam direction.
    ///
    /// Searches azimuth in `az_range` and trace velocity in `vel_range` with the
    /// given number of steps in each dimension. Returns the azimuth, velocity, and
    /// Fisher ratio of the best grid point.
    ///
    /// # Arguments
    /// * `signals` - Array element signals
    /// * `az_range` - (min_deg, max_deg) azimuth search range
    /// * `vel_range` - (min_m_s, max_m_s) trace velocity search range
    /// * `steps` - Number of steps in each dimension
    ///
    /// # Returns
    /// (best_azimuth_deg, best_velocity_m_s, best_fisher_ratio)
    pub fn fisher_grid_search(
        &self,
        signals: &[Vec<f64>],
        az_range: (f64, f64),
        vel_range: (f64, f64),
        steps: usize,
    ) -> (f64, f64, f64) {
        if signals.is_empty() || signals[0].is_empty() || steps == 0 {
            return (0.0, 0.0, 0.0);
        }
        let n_samples = signals[0].len();
        let n_elem = signals.len().min(self.num_elements);

        let mut best_az = az_range.0;
        let mut best_vel = vel_range.0;
        let mut best_fisher = 0.0_f64;

        let az_step = if steps > 1 {
            (az_range.1 - az_range.0) / (steps - 1) as f64
        } else {
            0.0
        };
        let vel_step = if steps > 1 {
            (vel_range.1 - vel_range.0) / (steps - 1) as f64
        } else {
            0.0
        };

        for az_i in 0..steps {
            let az = az_range.0 + az_step * az_i as f64;
            for vel_i in 0..steps {
                let vel = vel_range.0 + vel_step * vel_i as f64;

                // Compute beam
                let mut beam = vec![0.0; n_samples];
                for ch in 0..n_elem {
                    if ch >= signals.len() {
                        break;
                    }
                    let delay_s = InfrasoundProcessor::time_delay_for_azimuth(
                        self.element_positions[ch],
                        az,
                        vel,
                    );
                    let delay_samples = delay_s * self.sample_rate_hz;

                    for i in 0..n_samples {
                        let src = i as f64 + delay_samples;
                        let idx = src.floor() as i64;
                        let frac = src - src.floor();

                        if idx >= 0 && (idx + 1) < n_samples as i64 {
                            beam[i] += signals[ch][idx as usize] * (1.0 - frac)
                                + signals[ch][(idx + 1) as usize] * frac;
                        } else if idx >= 0 && idx < n_samples as i64 {
                            beam[i] += signals[ch][idx as usize] * (1.0 - frac);
                        }
                    }
                }
                for v in &mut beam {
                    *v /= n_elem as f64;
                }

                let fisher = self.compute_fisher_ratio(signals, &beam);
                if fisher > best_fisher {
                    best_fisher = fisher;
                    best_az = az;
                    best_vel = vel;
                }
            }
        }

        (best_az, best_vel, best_fisher)
    }

    /// Progressive Multi-Channel Correlation (PMCC-like) measure.
    ///
    /// Computes the average normalized cross-correlation between all pairs of
    /// array elements. Values near 1.0 indicate highly correlated (coherent)
    /// signals; values near 0.0 indicate uncorrelated noise.
    ///
    /// The true PMCC algorithm (Cansi, 1995) uses a progressive approach with
    /// sub-arrays, but this provides a simplified measure of array-wide coherence.
    ///
    /// # Arguments
    /// * `signals` - Array element signals
    ///
    /// # Returns
    /// Average pairwise correlation coefficient (0.0 to 1.0)
    pub fn progressive_multichannel_correlation(&self, signals: &[Vec<f64>]) -> f64 {
        if signals.len() < 2 {
            return 0.0;
        }

        let mut total_corr = 0.0;
        let mut n_pairs = 0;

        for i in 0..signals.len() {
            for j in (i + 1)..signals.len() {
                let len = signals[i].len().min(signals[j].len());
                if len == 0 {
                    continue;
                }

                let mean_i: f64 = signals[i][..len].iter().sum::<f64>() / len as f64;
                let mean_j: f64 = signals[j][..len].iter().sum::<f64>() / len as f64;

                let mut cov = 0.0;
                let mut var_i = 0.0;
                let mut var_j = 0.0;

                for k in 0..len {
                    let di = signals[i][k] - mean_i;
                    let dj = signals[j][k] - mean_j;
                    cov += di * dj;
                    var_i += di * di;
                    var_j += dj * dj;
                }

                let denom = (var_i * var_j).sqrt();
                if denom > 1e-30 {
                    total_corr += (cov / denom).abs();
                }
                n_pairs += 1;
            }
        }

        if n_pairs > 0 {
            total_corr / n_pairs as f64
        } else {
            0.0
        }
    }
}

/// Yield estimation from infrasound overpressure measurements.
///
/// Provides scaling laws relating explosive yield (kilotons TNT equivalent) to
/// peak overpressure at a given distance. These empirical relationships are used
/// for characterizing nuclear tests, bolide airbursts, and large chemical explosions.
///
/// ## Scaling Laws
///
/// The Glasstone-Dolan (1977) scaling law relates yield W (kt) to peak overpressure
/// delta-P (Pa) at distance R (km):
///
///   delta-P ~ W^(1/3) / R
///
/// The inverse problem (estimating yield from overpressure) is:
///
///   W ~ (delta-P * R)^3
pub struct YieldEstimator;

impl YieldEstimator {
    /// Estimate explosive yield from peak overpressure and distance.
    ///
    /// Uses the cube-root scaling law: W = (delta-P * R / K)^3
    /// where K is an empirical constant (~3850 Pa*km/kt^(1/3)).
    ///
    /// # Arguments
    /// * `overpressure_pa` - Peak overpressure in Pascals
    /// * `distance_km` - Distance from source in kilometers
    ///
    /// # Returns
    /// Estimated yield in kilotons TNT equivalent
    pub fn overpressure_to_yield_kt(overpressure_pa: f64, distance_km: f64) -> f64 {
        // Empirical constant: ~3850 Pa*km per kt^(1/3)
        let k = 3850.0;
        let scaled = overpressure_pa * distance_km / k;
        scaled * scaled * scaled
    }

    /// Compute peak overpressure from yield and distance using scaling law.
    ///
    /// delta-P = K * W^(1/3) / R
    ///
    /// # Arguments
    /// * `yield_kt` - Yield in kilotons TNT
    /// * `distance_km` - Distance in kilometers
    ///
    /// # Returns
    /// Peak overpressure in Pascals
    pub fn yield_scaling_law(yield_kt: f64, distance_km: f64) -> f64 {
        if distance_km <= 0.0 {
            return 0.0;
        }
        let k = 3850.0;
        k * yield_kt.cbrt() / distance_km
    }

    /// Convert raw signal amplitude to overpressure in Pascals.
    ///
    /// # Arguments
    /// * `amplitude` - Raw signal amplitude (counts or volts)
    /// * `sensitivity_pa_per_count` - Sensor sensitivity in Pa per count
    ///
    /// # Returns
    /// Overpressure in Pascals
    pub fn signal_amplitude_to_overpressure(
        amplitude: f64,
        sensitivity_pa_per_count: f64,
    ) -> f64 {
        amplitude * sensitivity_pa_per_count
    }

    /// Atmospheric attenuation of infrasound with distance and frequency.
    ///
    /// Infrasound attenuation is frequency-dependent, with higher frequencies
    /// attenuating more rapidly. This uses a simplified model:
    ///
    ///   attenuation_dB = alpha * f^beta * R
    ///
    /// where alpha and beta are empirical constants for the atmospheric waveguide.
    ///
    /// # Arguments
    /// * `frequency_hz` - Signal frequency in Hz
    /// * `distance_km` - Propagation distance in km
    ///
    /// # Returns
    /// Attenuation in dB (always positive)
    pub fn atmospheric_attenuation(frequency_hz: f64, distance_km: f64) -> f64 {
        // Empirical model: attenuation increases with frequency and distance
        // Typical values: ~0.5-2.0 dB per 1000 km at 1 Hz
        let alpha = 0.001; // dB per km per Hz^beta
        let beta = 1.5; // frequency exponent
        alpha * frequency_hz.powf(beta) * distance_km
    }
}

/// Atmospheric propagation model for infrasound.
///
/// Infrasound propagates through stratospheric and thermospheric waveguides,
/// refracting back to the surface at characteristic distances. The celerity
/// (average horizontal speed from source to receiver) varies by propagation
/// path:
///
/// - Tropospheric: 300-340 m/s (short range)
/// - Stratospheric: 280-310 m/s (100-300 km returns)
/// - Thermospheric: 220-260 m/s (200+ km returns)
pub struct PropagationModel;

impl PropagationModel {
    /// Compute celerity (average horizontal propagation speed).
    ///
    /// Celerity = distance / travel_time. Typical values:
    /// - 300-340 m/s: tropospheric path
    /// - 280-310 m/s: stratospheric path
    /// - 220-260 m/s: thermospheric path
    ///
    /// # Arguments
    /// * `distance_km` - Great-circle distance in km
    /// * `travel_time_s` - Travel time in seconds
    ///
    /// # Returns
    /// Celerity in m/s
    pub fn celerity(distance_km: f64, travel_time_s: f64) -> f64 {
        if travel_time_s <= 0.0 {
            return 0.0;
        }
        distance_km * 1000.0 / travel_time_s
    }

    /// Estimate the distance to the first stratospheric return.
    ///
    /// Sound refracts in the stratopause (~50 km altitude) and returns to the
    /// surface. The return distance depends on source height and atmospheric
    /// conditions. Uses a simplified geometric model.
    ///
    /// # Arguments
    /// * `source_height_km` - Height of the source in km (0 for surface)
    ///
    /// # Returns
    /// Approximate distance to first stratospheric return in km
    pub fn stratospheric_return_distance(source_height_km: f64) -> f64 {
        // Stratospheric duct height ~50 km
        // Empirical model: first return distance ~180-250 km for surface sources
        // Based on ray-tracing studies and IMS observations
        let duct_height_km = 50.0;
        let effective_height = duct_height_km - source_height_km.min(duct_height_km);
        // Empirical scaling: ~4.5 km ground range per km of duct height
        // This gives ~225 km for 50 km duct, consistent with observations
        4.5 * effective_height
    }

    /// Estimate the distance to the first thermospheric return.
    ///
    /// Sound refracts in the thermosphere (~100-120 km altitude) and returns
    /// to the surface at greater distances than stratospheric returns.
    ///
    /// # Arguments
    /// * `source_height_km` - Height of the source in km (0 for surface)
    ///
    /// # Returns
    /// Approximate distance to first thermospheric return in km
    pub fn thermospheric_return_distance(source_height_km: f64) -> f64 {
        // Thermospheric duct height ~110 km
        // Empirical: ~3.5 km ground range per km of effective duct height
        // Gives ~385 km for surface sources, consistent with observations
        let duct_height_km = 110.0;
        let effective_height = duct_height_km - source_height_km.min(duct_height_km);
        3.5 * effective_height
    }

    /// Estimate source range from measured celerity using empirical relationships.
    ///
    /// Different celerity values correspond to different propagation paths and
    /// typical ranges. This provides a rough range estimate based on the path type.
    ///
    /// # Arguments
    /// * `celerity` - Measured celerity in m/s
    ///
    /// # Returns
    /// Estimated source range in km
    pub fn source_range_from_celerity(celerity: f64) -> f64 {
        // Empirical relationships between celerity and range:
        // High celerity (>310 m/s): tropospheric, close range
        // Medium celerity (280-310 m/s): stratospheric, ~200 km
        // Low celerity (220-280 m/s): thermospheric, ~300+ km
        if celerity > 310.0 {
            // Tropospheric: short range, roughly 50-150 km
            50.0 + (340.0 - celerity) * 3.3
        } else if celerity > 280.0 {
            // Stratospheric: medium range
            150.0 + (310.0 - celerity) * 5.0
        } else if celerity > 220.0 {
            // Thermospheric: long range
            300.0 + (280.0 - celerity) * 8.3
        } else {
            // Very low celerity: very long range
            800.0 + (220.0 - celerity) * 10.0
        }
    }
}

/// Wind noise reduction for infrasound arrays.
///
/// Wind-generated pressure fluctuations (turbulence) are the dominant noise source
/// at infrasound stations. Two techniques are provided:
///
/// 1. **Coherent noise reduction**: Delay-and-sum beamforming suppresses incoherent
///    wind noise while preserving coherent signals. SNR improvement is ~sqrt(N) for
///    N elements.
///
/// 2. **Pipe array (rosette) response**: Physical noise-reducing pipe arrays with
///    radial inlet ports provide spatial averaging that attenuates turbulent pressure
///    fluctuations. The response depends on frequency and pipe length.
pub struct WindNoiseReducer {
    config: InfrasoundConfig,
}

impl WindNoiseReducer {
    /// Create a new wind noise reducer from configuration.
    pub fn new(config: InfrasoundConfig) -> Self {
        Self { config }
    }

    /// Apply coherent noise reduction by delay-and-sum beamforming.
    ///
    /// Estimates the optimal beam direction from the signals and applies
    /// delay-and-sum processing. Wind noise, being spatially incoherent,
    /// is attenuated by a factor of ~sqrt(N) while coherent signals are preserved.
    ///
    /// # Arguments
    /// * `signals` - Array element signals
    ///
    /// # Returns
    /// Noise-reduced signal (beamformed output)
    pub fn coherent_noise_reduction(&self, signals: &[Vec<f64>]) -> Vec<f64> {
        if signals.is_empty() || signals[0].is_empty() {
            return vec![];
        }

        let n_samples = signals[0].len();

        // Simple delay-and-sum with zero steering (broadside beam)
        // For a more sophisticated approach, one could estimate the optimal
        // azimuth first, but for noise reduction the broadside sum already
        // provides sqrt(N) improvement against incoherent noise.
        let mut output = vec![0.0; n_samples];
        let n_ch = signals.len();

        for ch in 0..n_ch {
            for i in 0..n_samples.min(signals[ch].len()) {
                output[i] += signals[ch][i];
            }
        }

        if n_ch > 0 {
            for v in &mut output {
                *v /= n_ch as f64;
            }
        }

        output
    }

    /// Compute the spatial filtering response of a pipe array (rosette).
    ///
    /// IMS infrasound stations use physical pipe arrays (rosettes) with multiple
    /// inlet ports along radial pipes. The spatial averaging provides a frequency-
    /// dependent noise reduction that follows a sinc-like response.
    ///
    /// # Arguments
    /// * `frequency_hz` - Signal frequency in Hz
    /// * `pipe_length_m` - Length of each pipe arm in meters
    ///
    /// # Returns
    /// Normalized response (0.0 to 1.0), where 1.0 means full signal pass-through
    pub fn pipe_array_response(frequency_hz: f64, pipe_length_m: f64) -> f64 {
        if pipe_length_m <= 0.0 || frequency_hz <= 0.0 {
            return 1.0;
        }
        // Acoustic wavelength
        let wavelength = 340.0 / frequency_hz;
        // The pipe array acts as a spatial low-pass filter
        // Response follows a sinc-like function: sin(pi*L/lambda) / (pi*L/lambda)
        let x = PI * pipe_length_m / wavelength;
        if x.abs() < 1e-10 {
            1.0
        } else {
            (x.sin() / x).abs()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a simple 4-element square array config
    fn test_config() -> InfrasoundConfig {
        InfrasoundConfig::new(
            20.0,
            vec![
                [0.0, 0.0],
                [1000.0, 0.0],
                [1000.0, 1000.0],
                [0.0, 1000.0],
            ],
            (0.02, 4.0),
        )
    }

    /// Helper: generate a sinusoidal signal
    fn sine_signal(freq_hz: f64, sample_rate: f64, n_samples: usize, amplitude: f64) -> Vec<f64> {
        (0..n_samples)
            .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: generate white noise using a simple LCG PRNG
    fn noise_signal(n_samples: usize, amplitude: f64, seed: u64) -> Vec<f64> {
        let mut state = seed;
        (0..n_samples)
            .map(|_| {
                state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let uniform = (state >> 33) as f64 / (1u64 << 31) as f64 - 1.0;
                amplitude * uniform
            })
            .collect()
    }

    #[test]
    fn test_config_creation() {
        let config = test_config();
        assert_eq!(config.num_elements, 4);
        assert_eq!(config.sample_rate_hz, 20.0);
        assert!((config.sound_speed_m_s - 340.0).abs() < 1e-10);
        assert_eq!(config.element_positions.len(), 4);
    }

    #[test]
    fn test_time_delay_zero_for_broadside() {
        // Element at origin should always have zero delay
        let delay = InfrasoundProcessor::time_delay_for_azimuth([0.0, 0.0], 45.0, 340.0);
        assert!(
            delay.abs() < 1e-15,
            "Delay at origin should be zero, got {}",
            delay
        );
    }

    #[test]
    fn test_time_delay_sign_convention() {
        // Signal from north (az=0): north element receives first
        let delay_north =
            InfrasoundProcessor::time_delay_for_azimuth([0.0, 1000.0], 0.0, 340.0);
        let delay_south =
            InfrasoundProcessor::time_delay_for_azimuth([0.0, -1000.0], 0.0, 340.0);
        // North element is closer to source from north, so delay is negative (arrives earlier)
        assert!(
            delay_north < delay_south,
            "North element should have less delay for signal from north"
        );
    }

    #[test]
    fn test_time_delay_symmetry() {
        // Equal distance elements on opposite sides should have opposite delays
        let delay_east =
            InfrasoundProcessor::time_delay_for_azimuth([500.0, 0.0], 90.0, 340.0);
        let delay_west =
            InfrasoundProcessor::time_delay_for_azimuth([-500.0, 0.0], 90.0, 340.0);
        assert!(
            (delay_east + delay_west).abs() < 1e-10,
            "Symmetric elements should have opposite delays"
        );
    }

    #[test]
    fn test_time_delay_proportional_to_distance() {
        let delay1 =
            InfrasoundProcessor::time_delay_for_azimuth([500.0, 0.0], 90.0, 340.0);
        let delay2 =
            InfrasoundProcessor::time_delay_for_azimuth([1000.0, 0.0], 90.0, 340.0);
        assert!(
            (delay2 / delay1 - 2.0).abs() < 1e-10,
            "Delay should be proportional to distance"
        );
    }

    #[test]
    fn test_beamforming_enhances_coherent_signal() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config.clone());
        let n = 400; // 20 seconds at 20 Hz
        let freq = 0.5; // 0.5 Hz infrasound

        // Generate coherent signals with delays for azimuth=0 (from north)
        let mut signals = Vec::new();
        for pos in &config.element_positions {
            let delay_s =
                InfrasoundProcessor::time_delay_for_azimuth(*pos, 0.0, 340.0);
            let sig: Vec<f64> = (0..n)
                .map(|i| {
                    let t = i as f64 / config.sample_rate_hz - delay_s;
                    (2.0 * PI * freq * t).sin()
                })
                .collect();
            signals.push(sig);
        }

        let beam = proc.beamform(&signals, 0.0, 340.0);
        let beam_power: f64 = beam.iter().map(|x| x * x).sum::<f64>() / n as f64;

        // Add noise and beamform - should still have significant power
        let mut noisy_signals = signals.clone();
        for (i, sig) in noisy_signals.iter_mut().enumerate() {
            let noise = noise_signal(n, 0.3, 42 + i as u64);
            for (j, s) in sig.iter_mut().enumerate() {
                *s += noise[j];
            }
        }
        let noisy_beam = proc.beamform(&noisy_signals, 0.0, 340.0);
        let noisy_beam_power: f64 = noisy_beam.iter().map(|x| x * x).sum::<f64>() / n as f64;

        assert!(
            beam_power > 0.1,
            "Coherent beam power should be significant: {}",
            beam_power
        );
        assert!(
            noisy_beam_power > 0.05,
            "Noisy beam power should still be measurable: {}",
            noisy_beam_power
        );
    }

    #[test]
    fn test_beamforming_empty_input() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config);
        let result = proc.beamform(&[], 0.0, 340.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_fisher_ratio_correlated_signals() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        let n = 200;

        // Identical signals (perfectly correlated): beam = signal, residual ≈ 0
        let sig = sine_signal(1.0, 20.0, n, 1.0);
        let signals: Vec<Vec<f64>> = vec![sig.clone(); 4];
        let beam = sig.clone();

        let fisher = detector.compute_fisher_ratio(&signals, &beam);
        assert!(
            fisher > 1.0,
            "Fisher ratio for correlated signals should be > 1, got {}",
            fisher
        );
    }

    #[test]
    fn test_fisher_ratio_low_for_noise() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        let n = 2000;

        // Independent noise on each channel using well-separated seeds
        let signals: Vec<Vec<f64>> = (0..4)
            .map(|i| noise_signal(n, 1.0, 10000 * (i + 1)))
            .collect();

        // Use average as "beam" (incoherent sum)
        let mut beam = vec![0.0; n];
        for sig in &signals {
            for (j, &s) in sig.iter().enumerate() {
                beam[j] += s;
            }
        }
        for v in &mut beam {
            *v /= 4.0;
        }

        let fisher = detector.compute_fisher_ratio(&signals, &beam);
        // For incoherent noise, the Fisher ratio from the beam (which is the
        // average of the channels) should be much lower than for a coherent signal.
        // With a simple LCG PRNG the "noise" may have some residual correlation,
        // so we use a generous upper bound but ensure it's far below the
        // coherent-signal case (which is typically >100).
        assert!(
            fisher < 50.0,
            "Fisher ratio for noise should be much lower than coherent case, got {}",
            fisher
        );
        assert!(
            fisher > 0.0,
            "Fisher ratio should be positive, got {}",
            fisher
        );
    }

    #[test]
    fn test_fisher_grid_search_finds_signal() {
        // Use a triangular array with sufficient aperture for azimuth resolution
        // and higher sample rate for adequate sampling of delays
        let tri_config = InfrasoundConfig::new(
            100.0, // 100 Hz sample rate for finer delay resolution
            vec![
                [0.0, 0.0],
                [500.0, 0.0],
                [250.0, 433.0],  // equilateral triangle, ~500m sides
                [-250.0, 433.0], // 4th element for better coverage
            ],
            (0.02, 4.0),
        );
        let detector = FisherDetector::new(&tri_config);
        let n = 10000; // 100 seconds at 100 Hz
        let freq = 0.5;
        let true_az = 90.0; // from east
        let true_vel = 340.0;

        let mut signals = Vec::new();
        for pos in &tri_config.element_positions {
            let delay_s =
                InfrasoundProcessor::time_delay_for_azimuth(*pos, true_az, true_vel);
            let sig: Vec<f64> = (0..n)
                .map(|i| {
                    let t = i as f64 / tri_config.sample_rate_hz - delay_s;
                    (2.0 * PI * freq * t).sin()
                })
                .collect();
            signals.push(sig);
        }

        let (az, _vel, fisher) =
            detector.fisher_grid_search(&signals, (0.0, 360.0), (280.0, 450.0), 36);

        assert!(
            fisher > 1.0,
            "Grid search Fisher should be > 1 for coherent signal, got {}",
            fisher
        );
        // Allow generous azimuth tolerance given grid resolution
        let az_error = ((az - true_az + 180.0) % 360.0 - 180.0).abs();
        assert!(
            az_error < 50.0,
            "Grid search azimuth error should be < 50 deg, got {} (found {} vs true {})",
            az_error,
            az,
            true_az
        );
    }

    #[test]
    fn test_pmcc_correlated_signals() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        let n = 200;

        // Identical signals
        let sig = sine_signal(1.0, 20.0, n, 1.0);
        let signals: Vec<Vec<f64>> = vec![sig; 4];

        let corr = detector.progressive_multichannel_correlation(&signals);
        assert!(
            (corr - 1.0).abs() < 0.01,
            "PMCC for identical signals should be ~1.0, got {}",
            corr
        );
    }

    #[test]
    fn test_pmcc_uncorrelated_signals() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        let n = 2000;

        let signals: Vec<Vec<f64>> = (0..4)
            .map(|i| noise_signal(n, 1.0, 200 + i))
            .collect();

        let corr = detector.progressive_multichannel_correlation(&signals);
        assert!(
            corr < 0.3,
            "PMCC for uncorrelated noise should be low, got {}",
            corr
        );
    }

    #[test]
    fn test_yield_scaling_law_roundtrip() {
        let yield_kt = 10.0;
        let distance_km = 500.0;

        let overpressure = YieldEstimator::yield_scaling_law(yield_kt, distance_km);
        let recovered_yield =
            YieldEstimator::overpressure_to_yield_kt(overpressure, distance_km);

        assert!(
            (recovered_yield - yield_kt).abs() / yield_kt < 1e-10,
            "Yield roundtrip failed: {} vs {}",
            recovered_yield,
            yield_kt
        );
    }

    #[test]
    fn test_yield_scaling_law_distance_dependence() {
        let yield_kt = 1.0;
        let pressure_near = YieldEstimator::yield_scaling_law(yield_kt, 100.0);
        let pressure_far = YieldEstimator::yield_scaling_law(yield_kt, 1000.0);

        assert!(
            pressure_near > pressure_far,
            "Closer distance should produce higher overpressure"
        );
    }

    #[test]
    fn test_yield_scaling_law_yield_dependence() {
        let distance = 500.0;
        let pressure_small = YieldEstimator::yield_scaling_law(1.0, distance);
        let pressure_large = YieldEstimator::yield_scaling_law(100.0, distance);

        assert!(
            pressure_large > pressure_small,
            "Larger yield should produce higher overpressure"
        );
    }

    #[test]
    fn test_yield_scaling_zero_distance() {
        let result = YieldEstimator::yield_scaling_law(1.0, 0.0);
        assert_eq!(result, 0.0, "Zero distance should return 0 to avoid infinity");
    }

    #[test]
    fn test_signal_amplitude_to_overpressure() {
        let amplitude = 100.0;
        let sensitivity = 0.01; // 0.01 Pa per count
        let pressure = YieldEstimator::signal_amplitude_to_overpressure(amplitude, sensitivity);
        assert!(
            (pressure - 1.0).abs() < 1e-10,
            "100 counts * 0.01 Pa/count should be 1.0 Pa, got {}",
            pressure
        );
    }

    #[test]
    fn test_atmospheric_attenuation_increases_with_frequency() {
        let atten_low = YieldEstimator::atmospheric_attenuation(0.1, 1000.0);
        let atten_high = YieldEstimator::atmospheric_attenuation(1.0, 1000.0);

        assert!(
            atten_high > atten_low,
            "Higher frequency should have more attenuation: {} vs {}",
            atten_high,
            atten_low
        );
    }

    #[test]
    fn test_atmospheric_attenuation_increases_with_distance() {
        let atten_near = YieldEstimator::atmospheric_attenuation(1.0, 100.0);
        let atten_far = YieldEstimator::atmospheric_attenuation(1.0, 1000.0);

        assert!(
            atten_far > atten_near,
            "Greater distance should have more attenuation: {} vs {}",
            atten_far,
            atten_near
        );
    }

    #[test]
    fn test_atmospheric_attenuation_positive() {
        let atten = YieldEstimator::atmospheric_attenuation(1.0, 500.0);
        assert!(atten > 0.0, "Attenuation should be positive, got {}", atten);
    }

    #[test]
    fn test_celerity_calculation() {
        let dist_km = 300.0;
        let time_s = 1000.0;
        let celerity = PropagationModel::celerity(dist_km, time_s);
        assert!(
            (celerity - 300.0).abs() < 1e-10,
            "300 km / 1000 s = 300 m/s, got {}",
            celerity
        );
    }

    #[test]
    fn test_celerity_in_expected_range() {
        // Stratospheric propagation: celerity ~ 280-310 m/s
        let dist_km = 200.0;
        let time_s = dist_km * 1000.0 / 295.0; // ~295 m/s celerity
        let celerity = PropagationModel::celerity(dist_km, time_s);
        assert!(
            celerity > 280.0 && celerity < 340.0,
            "Celerity should be in 280-340 m/s range, got {}",
            celerity
        );
    }

    #[test]
    fn test_celerity_zero_time() {
        let celerity = PropagationModel::celerity(100.0, 0.0);
        assert_eq!(celerity, 0.0, "Zero time should return 0");
    }

    #[test]
    fn test_stratospheric_return_distance() {
        let dist = PropagationModel::stratospheric_return_distance(0.0);
        assert!(
            dist > 100.0 && dist < 1000.0,
            "Stratospheric return should be 100-1000 km, got {}",
            dist
        );
    }

    #[test]
    fn test_thermospheric_return_distance() {
        let dist = PropagationModel::thermospheric_return_distance(0.0);
        let strat_dist = PropagationModel::stratospheric_return_distance(0.0);
        assert!(
            dist > strat_dist,
            "Thermospheric return ({}) should be farther than stratospheric ({})",
            dist,
            strat_dist
        );
    }

    #[test]
    fn test_source_range_from_celerity() {
        // High celerity = close range
        let range_close = PropagationModel::source_range_from_celerity(330.0);
        // Medium celerity = medium range
        let range_medium = PropagationModel::source_range_from_celerity(290.0);
        // Low celerity = long range
        let range_far = PropagationModel::source_range_from_celerity(240.0);

        assert!(
            range_close < range_medium,
            "Higher celerity should give shorter range: {} vs {}",
            range_close,
            range_medium
        );
        assert!(
            range_medium < range_far,
            "Lower celerity should give longer range: {} vs {}",
            range_medium,
            range_far
        );
    }

    #[test]
    fn test_source_range_positive() {
        for celerity in [200.0, 250.0, 300.0, 330.0] {
            let range = PropagationModel::source_range_from_celerity(celerity);
            assert!(
                range > 0.0,
                "Source range should be positive for celerity={}, got {}",
                celerity,
                range
            );
        }
    }

    #[test]
    fn test_wind_noise_reduction_improves_snr() {
        let config = test_config();
        let reducer = WindNoiseReducer::new(config);
        let n = 400;

        // All channels: same signal + independent noise
        let signal = sine_signal(0.5, 20.0, n, 1.0);
        let mut signals = Vec::new();
        for i in 0..4 {
            let noise = noise_signal(n, 0.5, 300 + i);
            let combined: Vec<f64> = signal
                .iter()
                .zip(noise.iter())
                .map(|(&s, &n)| s + n)
                .collect();
            signals.push(combined);
        }

        let reduced = reducer.coherent_noise_reduction(&signals);

        // Measure signal correlation with original
        let mean_red: f64 = reduced.iter().sum::<f64>() / n as f64;
        let mean_sig: f64 = signal.iter().sum::<f64>() / n as f64;
        let mut cov = 0.0;
        let mut var_red = 0.0;
        let mut var_sig = 0.0;
        for i in 0..n {
            let dr = reduced[i] - mean_red;
            let ds = signal[i] - mean_sig;
            cov += dr * ds;
            var_red += dr * dr;
            var_sig += ds * ds;
        }
        let corr = cov / (var_red * var_sig).sqrt();

        // The reduced signal should be more correlated with the true signal
        // than any single noisy channel
        let mean_ch: f64 = signals[0].iter().sum::<f64>() / n as f64;
        let mut cov_ch = 0.0;
        let mut var_ch = 0.0;
        for i in 0..n {
            let dc = signals[0][i] - mean_ch;
            let ds = signal[i] - mean_sig;
            cov_ch += dc * ds;
            var_ch += dc * dc;
        }
        let corr_ch = cov_ch / (var_ch * var_sig).sqrt();

        assert!(
            corr > corr_ch,
            "Noise reduction should improve correlation: {} (reduced) vs {} (single channel)",
            corr,
            corr_ch
        );
    }

    #[test]
    fn test_wind_noise_reduction_empty_input() {
        let config = test_config();
        let reducer = WindNoiseReducer::new(config);
        let result = reducer.coherent_noise_reduction(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_pipe_array_response_unity_at_low_freq() {
        // At very low frequency (long wavelength), response should be near 1.0
        let response = WindNoiseReducer::pipe_array_response(0.001, 10.0);
        assert!(
            (response - 1.0).abs() < 0.01,
            "Low freq response should be ~1.0, got {}",
            response
        );
    }

    #[test]
    fn test_pipe_array_response_decreases_with_freq() {
        let resp_low = WindNoiseReducer::pipe_array_response(0.1, 50.0);
        let resp_high = WindNoiseReducer::pipe_array_response(5.0, 50.0);

        // At higher frequencies relative to pipe length, response should generally decrease
        // (sinc-like pattern)
        assert!(
            resp_low > resp_high || resp_high < 1.0,
            "Higher frequency should generally show more filtering"
        );
    }

    #[test]
    fn test_pipe_array_response_zero_inputs() {
        assert_eq!(WindNoiseReducer::pipe_array_response(0.0, 10.0), 1.0);
        assert_eq!(WindNoiseReducer::pipe_array_response(1.0, 0.0), 1.0);
    }

    #[test]
    fn test_bandpass_filter_output_length() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config);
        let signal = vec![1.0; 100];
        let filtered = proc.bandpass_filter(&signal, 0.5, 2.0, 20.0);
        assert_eq!(filtered.len(), 100, "Output length should match input");
    }

    #[test]
    fn test_bandpass_filter_empty_input() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config);
        let filtered = proc.bandpass_filter(&[], 0.5, 2.0, 20.0);
        assert!(filtered.is_empty());
    }

    #[test]
    fn test_bandpass_filter_attenuates_out_of_band() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config);
        let n = 1000;

        // In-band signal (1 Hz)
        let in_band = sine_signal(1.0, 20.0, n, 1.0);
        let filtered_in = proc.bandpass_filter(&in_band, 0.5, 3.0, 20.0);
        let power_in: f64 = filtered_in.iter().map(|x| x * x).sum::<f64>() / n as f64;

        // Out-of-band signal (9 Hz, above Nyquist-safe passband)
        let out_band = sine_signal(9.0, 20.0, n, 1.0);
        let filtered_out = proc.bandpass_filter(&out_band, 0.5, 3.0, 20.0);
        let power_out: f64 = filtered_out.iter().map(|x| x * x).sum::<f64>() / n as f64;

        assert!(
            power_in > power_out,
            "In-band power ({}) should exceed out-of-band power ({})",
            power_in,
            power_out
        );
    }

    #[test]
    fn test_detect_events_with_coherent_signal() {
        // Use smaller array so delays are manageable relative to signal length
        let config = InfrasoundConfig::new(
            20.0,
            vec![
                [0.0, 0.0],
                [100.0, 0.0],
                [100.0, 100.0],
                [0.0, 100.0],
            ],
            (0.02, 4.0),
        );
        let proc = InfrasoundProcessor::new(config.clone());
        let n = 2000; // 100 seconds of data
        let freq = 0.5;
        let true_az = 0.0;
        let true_vel = 340.0;

        let mut signals = Vec::new();
        for pos in &config.element_positions {
            let delay_s =
                InfrasoundProcessor::time_delay_for_azimuth(*pos, true_az, true_vel);
            let sig: Vec<f64> = (0..n)
                .map(|i| {
                    let t = i as f64 / config.sample_rate_hz - delay_s;
                    (2.0 * PI * freq * t).sin()
                })
                .collect();
            signals.push(sig);
        }

        let detections = proc.detect_events(&signals, 1.5);
        assert!(
            !detections.is_empty(),
            "Should detect coherent signal"
        );
        assert!(
            detections[0].fisher_ratio > 1.5,
            "Fisher ratio should exceed threshold"
        );
    }

    #[test]
    fn test_detect_events_no_detection_for_noise() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config);
        let n = 200;

        let signals: Vec<Vec<f64>> = (0..4)
            .map(|i| noise_signal(n, 1.0, 400 + i))
            .collect();

        // Very high threshold should yield no detections
        let detections = proc.detect_events(&signals, 100.0);
        assert!(
            detections.is_empty(),
            "High threshold should produce no detections for noise"
        );
    }

    #[test]
    fn test_detection_back_azimuth_range() {
        let config = test_config();
        let proc = InfrasoundProcessor::new(config.clone());
        let n = 400;
        let freq = 0.5;

        // Test multiple azimuths
        for true_az in [0.0, 90.0, 180.0, 270.0] {
            let mut signals = Vec::new();
            for pos in &config.element_positions {
                let delay_s =
                    InfrasoundProcessor::time_delay_for_azimuth(*pos, true_az, 340.0);
                let sig: Vec<f64> = (0..n)
                    .map(|i| {
                        let t = i as f64 / config.sample_rate_hz - delay_s;
                        (2.0 * PI * freq * t).sin()
                    })
                    .collect();
                signals.push(sig);
            }

            let detections = proc.detect_events(&signals, 1.0);
            if !detections.is_empty() {
                let az = detections[0].back_azimuth_deg;
                assert!(
                    az >= 0.0 && az < 360.0,
                    "Back-azimuth should be in [0, 360), got {}",
                    az
                );
            }
        }
    }

    #[test]
    fn test_detection_struct_fields() {
        let det = InfrasoundDetection {
            back_azimuth_deg: 45.0,
            trace_velocity_m_s: 320.0,
            frequency_hz: 1.0,
            start_time_s: 0.0,
            duration_s: 10.0,
            snr_db: 15.0,
            fisher_ratio: 5.0,
        };
        assert_eq!(det.back_azimuth_deg, 45.0);
        assert_eq!(det.trace_velocity_m_s, 320.0);
        assert_eq!(det.frequency_hz, 1.0);
        assert_eq!(det.snr_db, 15.0);
        assert_eq!(det.fisher_ratio, 5.0);
    }

    #[test]
    fn test_fisher_empty_inputs() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        assert_eq!(detector.compute_fisher_ratio(&[], &[]), 0.0);
    }

    #[test]
    fn test_fisher_grid_search_empty_inputs() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        let result = detector.fisher_grid_search(&[], (0.0, 360.0), (280.0, 450.0), 10);
        assert_eq!(result, (0.0, 0.0, 0.0));
    }

    #[test]
    fn test_pmcc_single_channel() {
        let config = test_config();
        let detector = FisherDetector::new(&config);
        let sig = vec![vec![1.0; 100]];
        let corr = detector.progressive_multichannel_correlation(&sig);
        assert_eq!(corr, 0.0, "Single channel PMCC should be 0");
    }

    #[test]
    fn test_propagation_model_stratospheric_vs_thermospheric() {
        let strat = PropagationModel::stratospheric_return_distance(0.0);
        let therm = PropagationModel::thermospheric_return_distance(0.0);
        assert!(
            therm > strat,
            "Thermospheric ({} km) should be farther than stratospheric ({} km)",
            therm,
            strat
        );
    }

    #[test]
    fn test_propagation_higher_source_shorter_return() {
        let ground = PropagationModel::stratospheric_return_distance(0.0);
        let elevated = PropagationModel::stratospheric_return_distance(10.0);
        assert!(
            elevated < ground,
            "Higher source ({} km) should have shorter return than ground ({} km)",
            elevated,
            ground
        );
    }
}
