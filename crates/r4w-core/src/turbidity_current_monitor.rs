// trace:FR-TURBIDITY | ai:claude
//! Turbidity current monitoring and analysis for submarine sediment flow detection.
//!
//! Implements detection, characterization, and analysis of turbidity currents
//! in oceanographic and geohazard applications. Supports sensor arrays along
//! submarine cables for front velocity estimation and the historical cable-break
//! detection method (1929 Grand Banks).

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Gravitational acceleration (m/s^2)
const G: f64 = 9.81;

/// Dynamic viscosity of seawater at ~4 deg C (Pa*s)
const MU_SEAWATER: f64 = 1.88e-3;

/// Density of seawater (kg/m^3)
const RHO_WATER: f64 = 1025.0;

/// Typical sediment grain density (quartz, kg/m^3)
const RHO_SEDIMENT: f64 = 2650.0;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// A single turbidity sensor reading.
#[derive(Debug, Clone, Copy)]
pub struct TurbidityReading {
    /// Time stamp in seconds since epoch or start of record.
    pub timestamp_s: f64,
    /// Nephelometric Turbidity Units.
    pub ntu: f64,
    /// Water temperature in degrees Celsius.
    pub temperature_c: f64,
    /// Pressure in decibars (approx depth in metres).
    pub pressure_dbar: f64,
    /// Current speed in m/s.
    pub current_speed_ms: f64,
    /// Current direction in degrees (0-360, north = 0).
    pub current_direction_deg: f64,
}

/// Configuration for a single turbidity sensor in an array.
#[derive(Debug, Clone, Copy)]
pub struct SensorConfig {
    /// Deployment depth (m).
    pub depth_m: f64,
    /// Distance along the cable from the reference point (m).
    pub cable_distance_m: f64,
    /// Sampling interval (s).
    pub sampling_interval_s: f64,
    /// NTU threshold for simple detection.
    pub ntu_threshold: f64,
}

/// Parameters for the turbidity current monitor.
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    /// NTU above background to declare detection.
    pub detection_threshold_ntu: f64,
    /// Minimum event duration (s) to avoid transient spikes.
    pub min_event_duration_s: f64,
    /// Maximum gap (s) within an event before splitting.
    pub max_gap_s: f64,
    /// Number of sensors in the array.
    pub num_sensors: usize,
}

impl Default for MonitorConfig {
    fn default() -> Self {
        Self {
            detection_threshold_ntu: 5.0,
            min_event_duration_s: 60.0,
            max_gap_s: 300.0,
            num_sensors: 4,
        }
    }
}

/// A detected turbidity current event.
#[derive(Debug, Clone)]
pub struct TurbidityEvent {
    /// Onset time (s).
    pub onset_s: f64,
    /// End time (s).
    pub end_s: f64,
    /// Peak NTU value.
    pub peak_ntu: f64,
    /// Time of peak NTU (s).
    pub peak_time_s: f64,
    /// Duration above threshold (s).
    pub duration_s: f64,
    /// Integrated turbidity (NTU*s) -- sediment load proxy.
    pub integrated_ntu_s: f64,
    /// Rise time: onset to peak (s).
    pub rise_time_s: f64,
    /// Decay time constant from exponential fit of falling limb (s).
    pub decay_tau_s: f64,
}

/// Result of front velocity estimation between two sensors.
#[derive(Debug, Clone, Copy)]
pub struct FrontVelocity {
    /// Time delay between sensors (s).
    pub delay_s: f64,
    /// Distance between sensors (m).
    pub distance_m: f64,
    /// Front speed (m/s).
    pub speed_ms: f64,
}

/// Froude number classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlowRegime {
    Subcritical,
    Critical,
    Supercritical,
}

/// Cable break record for historical detection method.
#[derive(Debug, Clone, Copy)]
pub struct CableBreak {
    /// Time of communication loss (s).
    pub time_s: f64,
    /// Distance along cable (m).
    pub cable_distance_m: f64,
}

/// Event statistics over a catalogue.
#[derive(Debug, Clone)]
pub struct EventStatistics {
    /// Number of events.
    pub count: usize,
    /// Mean peak NTU.
    pub mean_peak_ntu: f64,
    /// Max peak NTU.
    pub max_peak_ntu: f64,
    /// Mean duration (s).
    pub mean_duration_s: f64,
    /// Mean recurrence interval (s).
    pub mean_recurrence_s: f64,
    /// Seasonal counts (index 0..11 for Jan..Dec) — only populated
    /// when timestamps map to months. We use a simple heuristic:
    /// month = floor(timestamp % (365.25*86400) / (30.44*86400)).
    pub seasonal_counts: [usize; 12],
}

// ---------------------------------------------------------------------------
// Core monitor
// ---------------------------------------------------------------------------

/// Turbidity current monitor with detection, characterisation, and analysis.
pub struct TurbidityCurrentMonitor {
    config: MonitorConfig,
}

impl TurbidityCurrentMonitor {
    /// Create a new monitor with the given configuration.
    pub fn new(config: MonitorConfig) -> Self {
        Self { config }
    }

    // ----- Background & threshold estimation -----

    /// Compute the rolling median of a slice (exact, O(n*w log w)).
    /// Returns a vector of the same length with edge values repeated.
    pub fn rolling_median(data: &[f64], window: usize) -> Vec<f64> {
        let n = data.len();
        if n == 0 {
            return vec![];
        }
        let w = window.max(1).min(n);
        let half = w / 2;
        let mut out = Vec::with_capacity(n);
        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = (i + w - half).min(n);
            let mut buf: Vec<f64> = data[lo..hi].to_vec();
            buf.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let m = buf.len();
            let med = if m % 2 == 1 {
                buf[m / 2]
            } else {
                (buf[m / 2 - 1] + buf[m / 2]) / 2.0
            };
            out.push(med);
        }
        out
    }

    /// Compute a given percentile (0–100) of a slice.
    pub fn percentile(data: &[f64], pct: f64) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        let mut sorted: Vec<f64> = data.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let k = (pct / 100.0) * (sorted.len() as f64 - 1.0);
        let lo = k.floor() as usize;
        let hi = k.ceil() as usize;
        if lo == hi || hi >= sorted.len() {
            sorted[lo.min(sorted.len() - 1)]
        } else {
            let frac = k - lo as f64;
            sorted[lo] * (1.0 - frac) + sorted[hi] * frac
        }
    }

    /// Estimate background NTU as the 10th percentile over a window of readings.
    pub fn background_estimate(ntu_values: &[f64]) -> f64 {
        Self::percentile(ntu_values, 10.0)
    }

    /// Compute mean and standard deviation of a slice.
    pub fn mean_std(data: &[f64]) -> (f64, f64) {
        let n = data.len() as f64;
        if n < 1.0 {
            return (0.0, 0.0);
        }
        let mean = data.iter().sum::<f64>() / n;
        if n < 2.0 {
            return (mean, 0.0);
        }
        let var = data.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / (n - 1.0);
        (mean, var.sqrt())
    }

    /// Adaptive threshold: background + n_sigma * sigma, with seasonal baseline.
    /// `seasonal_baseline` is a 12-element array of monthly background NTU.
    /// `month` is 0-based (0 = January).
    pub fn adaptive_threshold(
        ntu_values: &[f64],
        n_sigma: f64,
        seasonal_baseline: &[f64; 12],
        month: usize,
    ) -> f64 {
        let (_mean, sigma) = Self::mean_std(ntu_values);
        let baseline = seasonal_baseline[month % 12];
        baseline + n_sigma * sigma
    }

    // ----- STA/LTA -----

    /// Short-Term Average / Long-Term Average ratio on NTU time series.
    /// Returns a vector of STA/LTA ratios (length = data.len()).
    /// `sta_len` and `lta_len` are in number of samples.
    pub fn sta_lta(data: &[f64], sta_len: usize, lta_len: usize) -> Vec<f64> {
        let n = data.len();
        if n == 0 || sta_len == 0 || lta_len == 0 {
            return vec![1.0; n];
        }
        let mut result = vec![1.0; n];
        for i in 0..n {
            // LTA window: [i-lta_len+1 .. i-sta_len] (before STA)
            // STA window: [i-sta_len+1 .. i]
            let sta_start = if i + 1 >= sta_len { i + 1 - sta_len } else { 0 };
            let sta_count = i + 1 - sta_start;
            let sta_sum: f64 = data[sta_start..=i].iter().sum();
            let sta_avg = sta_sum / sta_count as f64;

            let lta_end = if i >= sta_len { i - sta_len } else { 0 };
            let lta_start = if lta_end + 1 >= lta_len {
                lta_end + 1 - lta_len
            } else {
                0
            };
            if lta_end < lta_start {
                result[i] = 1.0;
                continue;
            }
            let lta_count = lta_end + 1 - lta_start;
            let lta_sum: f64 = data[lta_start..=lta_end].iter().sum();
            let lta_avg = lta_sum / lta_count as f64;

            result[i] = if lta_avg > 1e-12 {
                sta_avg / lta_avg
            } else {
                1.0
            };
        }
        result
    }

    // ----- Event detection -----

    /// Detect events by threshold crossing on NTU readings.
    /// `background` is the estimated background NTU.
    /// Returns a list of events.
    pub fn detect_events(
        &self,
        readings: &[TurbidityReading],
        background: f64,
    ) -> Vec<TurbidityEvent> {
        let threshold = background + self.config.detection_threshold_ntu;
        let mut events = Vec::new();
        let mut in_event = false;
        let mut onset_idx = 0usize;
        let mut last_above_idx = 0usize;

        for (i, r) in readings.iter().enumerate() {
            if r.ntu >= threshold {
                if !in_event {
                    in_event = true;
                    onset_idx = i;
                }
                last_above_idx = i;
            } else if in_event {
                // Check gap
                let gap = r.timestamp_s - readings[last_above_idx].timestamp_s;
                if gap > self.config.max_gap_s {
                    // Close the event
                    let ev = self.characterize_event(readings, onset_idx, last_above_idx, background);
                    if ev.duration_s >= self.config.min_event_duration_s {
                        events.push(ev);
                    }
                    in_event = false;
                }
            }
        }
        // Close any trailing event
        if in_event {
            let ev = self.characterize_event(readings, onset_idx, last_above_idx, background);
            if ev.duration_s >= self.config.min_event_duration_s {
                events.push(ev);
            }
        }
        events
    }

    /// Characterize a single event from onset to end indices.
    fn characterize_event(
        &self,
        readings: &[TurbidityReading],
        start: usize,
        end: usize,
        _background: f64,
    ) -> TurbidityEvent {
        let onset_s = readings[start].timestamp_s;
        let end_s = readings[end].timestamp_s;

        // Find peak
        let mut peak_ntu = f64::NEG_INFINITY;
        let mut peak_idx = start;
        for i in start..=end {
            if readings[i].ntu > peak_ntu {
                peak_ntu = readings[i].ntu;
                peak_idx = i;
            }
        }
        let peak_time_s = readings[peak_idx].timestamp_s;

        // Integrated turbidity (trapezoidal rule)
        let mut integrated = 0.0;
        for i in start..end {
            let dt = readings[i + 1].timestamp_s - readings[i].timestamp_s;
            integrated += 0.5 * (readings[i].ntu + readings[i + 1].ntu) * dt;
        }

        let rise_time_s = peak_time_s - onset_s;

        // Decay time constant: fit ln(NTU) = ln(A) - t/tau on falling limb
        let decay_tau_s = Self::fit_decay_tau(readings, peak_idx, end);

        TurbidityEvent {
            onset_s,
            end_s,
            peak_ntu,
            peak_time_s,
            duration_s: end_s - onset_s,
            integrated_ntu_s: integrated,
            rise_time_s,
            decay_tau_s,
        }
    }

    /// Fit an exponential decay constant tau from the falling limb.
    /// Uses linear regression on ln(NTU) vs time.
    fn fit_decay_tau(readings: &[TurbidityReading], peak_idx: usize, end_idx: usize) -> f64 {
        if end_idx <= peak_idx {
            return 0.0;
        }
        let t0 = readings[peak_idx].timestamp_s;
        let mut sum_t = 0.0;
        let mut sum_ln = 0.0;
        let mut sum_t2 = 0.0;
        let mut sum_t_ln = 0.0;
        let mut count = 0.0;

        for i in peak_idx..=end_idx {
            let ntu = readings[i].ntu;
            if ntu <= 0.0 {
                continue;
            }
            let t = readings[i].timestamp_s - t0;
            let ln_ntu = ntu.ln();
            sum_t += t;
            sum_ln += ln_ntu;
            sum_t2 += t * t;
            sum_t_ln += t * ln_ntu;
            count += 1.0;
        }
        if count < 2.0 {
            return 0.0;
        }
        // slope = (n*sum(t*ln) - sum(t)*sum(ln)) / (n*sum(t^2) - sum(t)^2)
        let denom = count * sum_t2 - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        let slope = (count * sum_t_ln - sum_t * sum_ln) / denom;
        // ln(NTU) = ln(A) + slope*t → NTU = A*exp(slope*t)
        // slope = -1/tau → tau = -1/slope
        if slope.abs() < 1e-30 {
            return 0.0;
        }
        (-1.0 / slope).abs()
    }

    // ----- Front velocity estimation -----

    /// Estimate front velocity from two sensor NTU time series.
    /// Uses cross-correlation to find the time delay.
    pub fn front_velocity(
        sensor_a: &[f64],
        sensor_b: &[f64],
        sampling_interval_s: f64,
        distance_m: f64,
    ) -> FrontVelocity {
        let delay_samples = Self::cross_correlation_delay(sensor_a, sensor_b);
        let delay_s = delay_samples as f64 * sampling_interval_s;
        let speed = if delay_s.abs() > 1e-12 {
            distance_m / delay_s.abs()
        } else {
            0.0
        };
        FrontVelocity {
            delay_s,
            distance_m,
            speed_ms: speed,
        }
    }

    /// Cross-correlation to find sample delay (positive = b lags a).
    fn cross_correlation_delay(a: &[f64], b: &[f64]) -> i64 {
        let n = a.len().min(b.len());
        if n == 0 {
            return 0;
        }
        // Mean-subtract
        let mean_a = a[..n].iter().sum::<f64>() / n as f64;
        let mean_b = b[..n].iter().sum::<f64>() / n as f64;
        let a_c: Vec<f64> = a[..n].iter().map(|x| x - mean_a).collect();
        let b_c: Vec<f64> = b[..n].iter().map(|x| x - mean_b).collect();

        let max_lag = (n / 2) as i64;
        let mut best_corr = f64::NEG_INFINITY;
        let mut best_lag: i64 = 0;

        for lag in -max_lag..=max_lag {
            let mut corr = 0.0;
            let mut count = 0usize;
            for i in 0..n {
                let j = i as i64 + lag;
                if j >= 0 && (j as usize) < n {
                    corr += a_c[i] * b_c[j as usize];
                    count += 1;
                }
            }
            if count > 0 {
                corr /= count as f64;
            }
            if corr > best_corr {
                best_corr = corr;
                best_lag = lag;
            }
        }
        best_lag
    }

    /// Estimate front acceleration from an array of sensors.
    /// Returns speeds between consecutive pairs and acceleration between them.
    pub fn front_acceleration(
        sensors_ntu: &[&[f64]],
        configs: &[SensorConfig],
    ) -> (Vec<FrontVelocity>, Vec<f64>) {
        let n = sensors_ntu.len().min(configs.len());
        let mut velocities = Vec::new();
        let mut accelerations = Vec::new();

        for i in 0..n.saturating_sub(1) {
            let dist = (configs[i + 1].cable_distance_m - configs[i].cable_distance_m).abs();
            let dt_s = configs[i].sampling_interval_s;
            let v = Self::front_velocity(sensors_ntu[i], sensors_ntu[i + 1], dt_s, dist);
            velocities.push(v);
        }

        for i in 0..velocities.len().saturating_sub(1) {
            let dv = velocities[i + 1].speed_ms - velocities[i].speed_ms;
            let dt = velocities[i + 1].delay_s.abs() - velocities[i].delay_s.abs();
            let acc = if dt.abs() > 1e-12 { dv / dt } else { 0.0 };
            accelerations.push(acc);
        }

        (velocities, accelerations)
    }

    // ----- Densiometric Froude number -----

    /// Compute the densiometric Froude number.
    /// Fr = U / sqrt(g' * h)
    pub fn froude_number(speed_ms: f64, reduced_gravity: f64, flow_height_m: f64) -> f64 {
        let denom = (reduced_gravity * flow_height_m).abs().sqrt();
        if denom < 1e-30 {
            return 0.0;
        }
        speed_ms / denom
    }

    /// Classify the flow regime from Froude number.
    pub fn flow_regime(fr: f64) -> FlowRegime {
        if (fr - 1.0).abs() < 0.05 {
            FlowRegime::Critical
        } else if fr > 1.0 {
            FlowRegime::Supercritical
        } else {
            FlowRegime::Subcritical
        }
    }

    // ----- Sediment concentration -----

    /// NTU to sediment concentration (volumetric fraction) using power-law calibration.
    /// C = a * NTU^b
    pub fn ntu_to_concentration(ntu: f64, a: f64, b: f64) -> f64 {
        if ntu <= 0.0 {
            return 0.0;
        }
        a * ntu.powf(b)
    }

    /// Reduced gravity g' from volumetric sediment concentration C.
    /// g' = g * C * (rho_s - rho_w) / (rho_w * (1 + C*(rho_s/rho_w - 1)))
    pub fn reduced_gravity(
        concentration: f64,
        rho_sediment: f64,
        rho_water: f64,
    ) -> f64 {
        let numerator = G * concentration * (rho_sediment - rho_water);
        let denominator = rho_water * (1.0 + concentration * (rho_sediment / rho_water - 1.0));
        if denominator.abs() < 1e-30 {
            return 0.0;
        }
        numerator / denominator
    }

    /// Simplified reduced gravity: g' = g*(rho_current - rho_ambient)/rho_ambient.
    pub fn reduced_gravity_simple(rho_current: f64, rho_ambient: f64) -> f64 {
        if rho_ambient.abs() < 1e-30 {
            return 0.0;
        }
        G * (rho_current - rho_ambient) / rho_ambient
    }

    // ----- Chezy equation -----

    /// Steady-state velocity from Chezy equation.
    /// U = sqrt(g' * h * sin(theta) / Cd)
    pub fn chezy_velocity(
        reduced_gravity: f64,
        flow_height_m: f64,
        slope_rad: f64,
        drag_coefficient: f64,
    ) -> f64 {
        if drag_coefficient <= 0.0 {
            return 0.0;
        }
        let val = reduced_gravity * flow_height_m * slope_rad.sin() / drag_coefficient;
        if val <= 0.0 {
            return 0.0;
        }
        val.sqrt()
    }

    // ----- Depositional rate estimation -----

    /// Stokes settling velocity for fine sediment.
    /// w_s = (rho_s - rho_w) * g * d^2 / (18 * mu)
    pub fn stokes_settling_velocity(
        grain_diameter_m: f64,
        rho_sediment: f64,
        rho_water: f64,
        viscosity: f64,
    ) -> f64 {
        (rho_sediment - rho_water) * G * grain_diameter_m * grain_diameter_m / (18.0 * viscosity)
    }

    /// Deposition rate R_dep = w_s * C_near_bed (mass per area per time).
    pub fn deposition_rate(settling_velocity: f64, concentration_near_bed: f64) -> f64 {
        settling_velocity * concentration_near_bed
    }

    // ----- Cable break detection -----

    /// Estimate front velocity from sequential cable breaks (Grand Banks method).
    /// Returns speeds between consecutive breaks.
    pub fn cable_break_velocity(breaks: &[CableBreak]) -> Vec<FrontVelocity> {
        let mut sorted: Vec<CableBreak> = breaks.to_vec();
        sorted.sort_by(|a, b| a.time_s.partial_cmp(&b.time_s).unwrap());

        let mut velocities = Vec::new();
        for i in 0..sorted.len().saturating_sub(1) {
            let dt = sorted[i + 1].time_s - sorted[i].time_s;
            let dd = (sorted[i + 1].cable_distance_m - sorted[i].cable_distance_m).abs();
            let speed = if dt.abs() > 1e-12 { dd / dt } else { 0.0 };
            velocities.push(FrontVelocity {
                delay_s: dt,
                distance_m: dd,
                speed_ms: speed,
            });
        }
        velocities
    }

    // ----- Settling column model -----

    /// Exponential concentration profile: C(z) = C0 * exp(-z * w_s / kappa).
    /// `z` is height above bed (m), `w_s` settling velocity (m/s), `kappa` eddy
    /// diffusivity (m^2/s).
    pub fn concentration_profile_exp(z: f64, c0: f64, settling_vel: f64, kappa: f64) -> f64 {
        if kappa.abs() < 1e-30 {
            return if z.abs() < 1e-12 { c0 } else { 0.0 };
        }
        c0 * (-z * settling_vel / kappa).exp()
    }

    /// Simplified 1-D advection-diffusion: C(z,t) for an initial concentration
    /// c0 at z=0 settling with diffusion.
    /// C(z,t) = (c0 / sqrt(4*pi*kappa*t)) * exp(-(z - w_s*t)^2 / (4*kappa*t))
    pub fn concentration_advection_diffusion(
        z: f64,
        t: f64,
        c0: f64,
        settling_vel: f64,
        kappa: f64,
    ) -> f64 {
        if t <= 0.0 || kappa <= 0.0 {
            return 0.0;
        }
        let denom = (4.0 * PI * kappa * t).sqrt();
        let exponent = -(z - settling_vel * t).powi(2) / (4.0 * kappa * t);
        c0 / denom * exponent.exp()
    }

    // ----- Statistical analysis -----

    /// Compute event statistics over a catalogue of events.
    pub fn event_statistics(events: &[TurbidityEvent]) -> EventStatistics {
        let count = events.len();
        if count == 0 {
            return EventStatistics {
                count: 0,
                mean_peak_ntu: 0.0,
                max_peak_ntu: 0.0,
                mean_duration_s: 0.0,
                mean_recurrence_s: 0.0,
                seasonal_counts: [0; 12],
            };
        }

        let mean_peak = events.iter().map(|e| e.peak_ntu).sum::<f64>() / count as f64;
        let max_peak = events
            .iter()
            .map(|e| e.peak_ntu)
            .fold(f64::NEG_INFINITY, f64::max);
        let mean_dur = events.iter().map(|e| e.duration_s).sum::<f64>() / count as f64;

        let mean_rec = if count > 1 {
            let mut intervals = Vec::new();
            for i in 1..count {
                intervals.push(events[i].onset_s - events[i - 1].onset_s);
            }
            intervals.iter().sum::<f64>() / intervals.len() as f64
        } else {
            0.0
        };

        // Seasonal counts (heuristic month from timestamp)
        let mut seasonal = [0usize; 12];
        let year_s = 365.25 * 86400.0;
        let month_s = year_s / 12.0;
        for e in events {
            let month = ((e.onset_s % year_s) / month_s).floor() as usize % 12;
            seasonal[month] += 1;
        }

        EventStatistics {
            count,
            mean_peak_ntu: mean_peak,
            max_peak_ntu: max_peak,
            mean_duration_s: mean_dur,
            mean_recurrence_s: mean_rec,
            seasonal_counts: seasonal,
        }
    }

    /// Magnitude-frequency relationship: log10(N) = a - b*log10(NTU_peak).
    /// Returns (a, b) from linear regression on log-log axes.
    pub fn magnitude_frequency(events: &[TurbidityEvent], ntu_bins: &[f64]) -> (f64, f64) {
        if events.is_empty() || ntu_bins.is_empty() {
            return (0.0, 0.0);
        }
        // Count events exceeding each bin threshold
        let mut log_n = Vec::new();
        let mut log_m = Vec::new();
        for &threshold in ntu_bins {
            let count = events.iter().filter(|e| e.peak_ntu >= threshold).count();
            if count > 0 && threshold > 0.0 {
                log_n.push((count as f64).log10());
                log_m.push(threshold.log10());
            }
        }
        if log_n.len() < 2 {
            return (0.0, 0.0);
        }
        // Linear regression: log_n = a + b*log_m  (we want a - b*log_m, so b is negative slope)
        let n = log_n.len() as f64;
        let sx: f64 = log_m.iter().sum();
        let sy: f64 = log_n.iter().sum();
        let sxy: f64 = log_m.iter().zip(log_n.iter()).map(|(x, y)| x * y).sum();
        let sxx: f64 = log_m.iter().map(|x| x * x).sum();
        let denom = n * sxx - sx * sx;
        if denom.abs() < 1e-30 {
            return (0.0, 0.0);
        }
        let slope = (n * sxy - sx * sy) / denom;
        let intercept = (sy - slope * sx) / n;
        // Return (a, b) where log10(N) = a - b*log10(M), so b = -slope
        (intercept, -slope)
    }

    /// Recurrence interval for a given NTU threshold.
    /// T_r = total_observation_time / N_exceedances.
    pub fn recurrence_interval(
        events: &[TurbidityEvent],
        ntu_threshold: f64,
        total_observation_s: f64,
    ) -> f64 {
        let n = events
            .iter()
            .filter(|e| e.peak_ntu >= ntu_threshold)
            .count();
        if n == 0 {
            return f64::INFINITY;
        }
        total_observation_s / n as f64
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_reading(t: f64, ntu: f64) -> TurbidityReading {
        TurbidityReading {
            timestamp_s: t,
            ntu,
            temperature_c: 4.0,
            pressure_dbar: 3000.0,
            current_speed_ms: 0.0,
            current_direction_deg: 0.0,
        }
    }

    // ---- Rolling median ----

    #[test]
    fn test_rolling_median_basic() {
        let data = vec![1.0, 3.0, 2.0, 5.0, 4.0];
        let med = TurbidityCurrentMonitor::rolling_median(&data, 3);
        assert_eq!(med.len(), 5);
        // Centre element of [1,3,2] => 2.0
        assert!((med[1] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_rolling_median_single() {
        let data = vec![7.0];
        let med = TurbidityCurrentMonitor::rolling_median(&data, 3);
        assert_eq!(med.len(), 1);
        assert!((med[0] - 7.0).abs() < 1e-9);
    }

    #[test]
    fn test_rolling_median_empty() {
        let med = TurbidityCurrentMonitor::rolling_median(&[], 5);
        assert!(med.is_empty());
    }

    // ---- Percentile ----

    #[test]
    fn test_percentile_50() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let p = TurbidityCurrentMonitor::percentile(&data, 50.0);
        assert!((p - 3.0).abs() < 1e-9);
    }

    #[test]
    fn test_percentile_10() {
        let data: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let p = TurbidityCurrentMonitor::percentile(&data, 10.0);
        assert!((p - 9.9).abs() < 0.1);
    }

    #[test]
    fn test_percentile_empty() {
        let p = TurbidityCurrentMonitor::percentile(&[], 50.0);
        assert!((p - 0.0).abs() < 1e-9);
    }

    // ---- Background estimate ----

    #[test]
    fn test_background_estimate() {
        // Background should be low for mostly-quiet data with a few spikes
        let mut data: Vec<f64> = vec![1.0; 90];
        data.extend(vec![100.0; 10]);
        let bg = TurbidityCurrentMonitor::background_estimate(&data);
        assert!((bg - 1.0).abs() < 0.5);
    }

    // ---- Mean / Std ----

    #[test]
    fn test_mean_std() {
        let data = vec![2.0, 4.0, 6.0, 8.0, 10.0];
        let (mean, std) = TurbidityCurrentMonitor::mean_std(&data);
        assert!((mean - 6.0).abs() < 1e-9);
        // std of [2,4,6,8,10] sample std = sqrt(10) ≈ 3.162
        assert!((std - 10.0_f64.sqrt()).abs() < 1e-9);
    }

    #[test]
    fn test_mean_std_empty() {
        let (m, s) = TurbidityCurrentMonitor::mean_std(&[]);
        assert!((m - 0.0).abs() < 1e-9);
        assert!((s - 0.0).abs() < 1e-9);
    }

    // ---- Adaptive threshold ----

    #[test]
    fn test_adaptive_threshold() {
        let data = vec![1.0, 1.1, 0.9, 1.0, 1.2];
        let baseline = [2.0; 12];
        let thr = TurbidityCurrentMonitor::adaptive_threshold(&data, 3.0, &baseline, 0);
        // baseline=2.0, sigma ≈ 0.11, threshold ≈ 2.0 + 0.33 ≈ 2.33
        assert!(thr > 2.0);
    }

    // ---- STA/LTA ----

    #[test]
    fn test_sta_lta_flat() {
        let data = vec![1.0; 100];
        let ratio = TurbidityCurrentMonitor::sta_lta(&data, 5, 50);
        // Flat signal → ratio ≈ 1.0
        for r in &ratio[50..] {
            assert!((r - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_sta_lta_spike() {
        let mut data = vec![1.0; 100];
        data.extend(vec![50.0; 10]);
        data.extend(vec![1.0; 50]);
        let ratio = TurbidityCurrentMonitor::sta_lta(&data, 5, 50);
        // At the spike, STA >> LTA
        assert!(ratio[102] > 5.0);
    }

    #[test]
    fn test_sta_lta_empty() {
        let ratio = TurbidityCurrentMonitor::sta_lta(&[], 5, 50);
        assert!(ratio.is_empty());
    }

    // ---- Event detection ----

    #[test]
    fn test_detect_event_simple() {
        let config = MonitorConfig {
            detection_threshold_ntu: 5.0,
            min_event_duration_s: 10.0,
            max_gap_s: 5.0,
            num_sensors: 1,
        };
        let mon = TurbidityCurrentMonitor::new(config);

        // Background at 1 NTU, event from t=100..200 s, peak at t=130
        let mut readings = Vec::new();
        for i in 0..300 {
            let t = i as f64;
            let ntu = if (100.0..=200.0).contains(&t) {
                let peak = 50.0;
                let rising = if t < 130.0 {
                    1.0 + (peak - 1.0) * (t - 100.0) / 30.0
                } else {
                    peak * (-(t - 130.0) / 40.0).exp()
                };
                rising
            } else {
                1.0
            };
            readings.push(default_reading(t, ntu));
        }

        let events = mon.detect_events(&readings, 1.0);
        assert!(!events.is_empty(), "should detect at least one event");
        assert!(events[0].peak_ntu > 40.0);
        assert!(events[0].onset_s >= 99.0);
        assert!(events[0].rise_time_s > 0.0);
    }

    #[test]
    fn test_detect_event_too_short() {
        let config = MonitorConfig {
            detection_threshold_ntu: 5.0,
            min_event_duration_s: 100.0, // very long minimum
            max_gap_s: 5.0,
            num_sensors: 1,
        };
        let mon = TurbidityCurrentMonitor::new(config);
        let mut readings = Vec::new();
        for i in 0..50 {
            let ntu = if i >= 20 && i <= 30 { 20.0 } else { 1.0 };
            readings.push(default_reading(i as f64, ntu));
        }
        let events = mon.detect_events(&readings, 1.0);
        assert!(events.is_empty(), "event shorter than min should be rejected");
    }

    #[test]
    fn test_detect_no_event() {
        let config = MonitorConfig::default();
        let mon = TurbidityCurrentMonitor::new(config);
        let readings: Vec<TurbidityReading> = (0..100)
            .map(|i| default_reading(i as f64, 1.0))
            .collect();
        let events = mon.detect_events(&readings, 1.0);
        assert!(events.is_empty());
    }

    // ---- Event characterisation ----

    #[test]
    fn test_event_integrated_turbidity() {
        let config = MonitorConfig {
            detection_threshold_ntu: 2.0,
            min_event_duration_s: 1.0,
            max_gap_s: 1.0,
            num_sensors: 1,
        };
        let mon = TurbidityCurrentMonitor::new(config);
        // Constant 10 NTU for 10 seconds → integral ≈ 100 NTU*s
        let readings: Vec<TurbidityReading> = (0..=10)
            .map(|i| default_reading(i as f64, 10.0))
            .collect();
        let events = mon.detect_events(&readings, 0.0);
        assert!(!events.is_empty());
        assert!((events[0].integrated_ntu_s - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_event_decay_tau() {
        // Exponential decay: NTU = 100 * exp(-t/50)
        let config = MonitorConfig {
            detection_threshold_ntu: 1.0,
            min_event_duration_s: 1.0,
            max_gap_s: 500.0,
            num_sensors: 1,
        };
        let mon = TurbidityCurrentMonitor::new(config);
        let readings: Vec<TurbidityReading> = (0..200)
            .map(|i| {
                let t = i as f64;
                let ntu = 100.0 * (-t / 50.0).exp();
                default_reading(t, ntu)
            })
            .collect();
        let events = mon.detect_events(&readings, 0.0);
        assert!(!events.is_empty());
        // Tau should be near 50
        assert!(
            (events[0].decay_tau_s - 50.0).abs() < 5.0,
            "tau={} expected ~50",
            events[0].decay_tau_s
        );
    }

    // ---- Front velocity ----

    #[test]
    fn test_front_velocity_known_delay() {
        // Sensor A has a pulse at sample 50, sensor B at sample 60
        let n = 200;
        let mut a = vec![0.0; n];
        let mut b = vec![0.0; n];
        for i in 50..70 {
            a[i] = 100.0;
        }
        for i in 60..80 {
            b[i] = 100.0;
        }
        let v = TurbidityCurrentMonitor::front_velocity(&a, &b, 1.0, 1000.0);
        // delay ≈ 10 samples → speed = 1000/10 = 100 m/s
        assert!((v.delay_s - 10.0).abs() < 2.0, "delay_s={}", v.delay_s);
        assert!(v.speed_ms > 50.0);
    }

    #[test]
    fn test_front_velocity_zero_distance() {
        let a = vec![1.0; 100];
        let b = vec![1.0; 100];
        let v = TurbidityCurrentMonitor::front_velocity(&a, &b, 1.0, 0.0);
        assert!((v.distance_m - 0.0).abs() < 1e-9);
    }

    // ---- Froude number ----

    #[test]
    fn test_froude_subcritical() {
        let gprime = 0.1; // m/s^2
        let h = 10.0;
        let u = 0.5; // m/s
        let fr = TurbidityCurrentMonitor::froude_number(u, gprime, h);
        assert!(fr < 1.0);
        assert_eq!(
            TurbidityCurrentMonitor::flow_regime(fr),
            FlowRegime::Subcritical
        );
    }

    #[test]
    fn test_froude_supercritical() {
        let fr = TurbidityCurrentMonitor::froude_number(10.0, 0.1, 5.0);
        assert!(fr > 1.0);
        assert_eq!(
            TurbidityCurrentMonitor::flow_regime(fr),
            FlowRegime::Supercritical
        );
    }

    #[test]
    fn test_froude_critical() {
        // Fr ≈ 1.0
        let gprime = 1.0;
        let h = 1.0;
        let u = 1.0; // sqrt(1*1)=1, Fr=1
        let fr = TurbidityCurrentMonitor::froude_number(u, gprime, h);
        assert_eq!(
            TurbidityCurrentMonitor::flow_regime(fr),
            FlowRegime::Critical
        );
    }

    // ---- Sediment concentration ----

    #[test]
    fn test_ntu_to_concentration() {
        // Linear calibration: C = 0.001 * NTU^1.0
        let c = TurbidityCurrentMonitor::ntu_to_concentration(100.0, 0.001, 1.0);
        assert!((c - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_ntu_to_concentration_zero() {
        let c = TurbidityCurrentMonitor::ntu_to_concentration(0.0, 0.001, 1.0);
        assert!((c - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_reduced_gravity() {
        let c = 0.01; // 1% concentration
        let gp = TurbidityCurrentMonitor::reduced_gravity(c, RHO_SEDIMENT, RHO_WATER);
        // g' = 9.81 * 0.01 * (2650-1025) / (1025 * (1 + 0.01*(2650/1025 - 1)))
        // ≈ 9.81 * 0.01 * 1625 / (1025 * 1.01585) ≈ 0.153
        assert!(gp > 0.1 && gp < 0.2, "g'={}", gp);
    }

    #[test]
    fn test_reduced_gravity_simple() {
        let gp =
            TurbidityCurrentMonitor::reduced_gravity_simple(1030.0, 1025.0);
        // g' = 9.81 * 5/1025 ≈ 0.0479
        assert!((gp - 9.81 * 5.0 / 1025.0).abs() < 1e-6);
    }

    // ---- Chezy equation ----

    #[test]
    fn test_chezy_velocity() {
        let gp = 0.1;
        let h = 10.0;
        let theta = 0.05_f64; // ~2.9 degrees
        let cd = 0.003;
        let u = TurbidityCurrentMonitor::chezy_velocity(gp, h, theta, cd);
        // U = sqrt(0.1 * 10 * sin(0.05) / 0.003) = sqrt(0.1665) ≈ 0.408
        let expected = (gp * h * theta.sin() / cd).sqrt();
        assert!((u - expected).abs() < 1e-6);
    }

    #[test]
    fn test_chezy_zero_drag() {
        let u = TurbidityCurrentMonitor::chezy_velocity(0.1, 10.0, 0.05, 0.0);
        assert!((u - 0.0).abs() < 1e-9);
    }

    // ---- Stokes settling ----

    #[test]
    fn test_stokes_settling_velocity() {
        let d = 50e-6; // 50 microns (fine silt)
        let ws = TurbidityCurrentMonitor::stokes_settling_velocity(
            d,
            RHO_SEDIMENT,
            RHO_WATER,
            MU_SEAWATER,
        );
        // ws = (2650-1025)*9.81*(50e-6)^2 / (18*1.88e-3)
        // = 1625 * 9.81 * 2.5e-9 / 0.03384 ≈ 1.178e-3 m/s
        assert!(ws > 1e-4 && ws < 5e-3, "ws={}", ws);
    }

    // ---- Deposition rate ----

    #[test]
    fn test_deposition_rate() {
        let ws = 0.001; // 1 mm/s
        let c = 0.01; // kg/m^3 as volumetric * density
        let r = TurbidityCurrentMonitor::deposition_rate(ws, c);
        assert!((r - 1e-5).abs() < 1e-9);
    }

    // ---- Cable break detection ----

    #[test]
    fn test_cable_break_velocity() {
        // Grand Banks-style: cables at 0, 100, 300 km broken at 0, 1000, 2000 s
        let breaks = vec![
            CableBreak {
                time_s: 0.0,
                cable_distance_m: 0.0,
            },
            CableBreak {
                time_s: 1000.0,
                cable_distance_m: 100_000.0,
            },
            CableBreak {
                time_s: 2000.0,
                cable_distance_m: 300_000.0,
            },
        ];
        let vels = TurbidityCurrentMonitor::cable_break_velocity(&breaks);
        assert_eq!(vels.len(), 2);
        // First segment: 100 km in 1000 s = 100 m/s
        assert!((vels[0].speed_ms - 100.0).abs() < 1e-6);
        // Second segment: 200 km in 1000 s = 200 m/s
        assert!((vels[1].speed_ms - 200.0).abs() < 1e-6);
    }

    #[test]
    fn test_cable_break_single() {
        let breaks = vec![CableBreak {
            time_s: 0.0,
            cable_distance_m: 0.0,
        }];
        let vels = TurbidityCurrentMonitor::cable_break_velocity(&breaks);
        assert!(vels.is_empty());
    }

    // ---- Settling column model ----

    #[test]
    fn test_concentration_profile_exp() {
        let c0 = 100.0;
        let ws = 0.001;
        let kappa = 0.01;
        // At z=0, C = C0
        let c_at_0 = TurbidityCurrentMonitor::concentration_profile_exp(0.0, c0, ws, kappa);
        assert!((c_at_0 - c0).abs() < 1e-9);

        // At z=10*kappa/ws = 100 m, C = C0*exp(-10) ≈ 0.00454 * C0
        let z = 10.0 * kappa / ws;
        let c_at_z =
            TurbidityCurrentMonitor::concentration_profile_exp(z, c0, ws, kappa);
        assert!((c_at_z - c0 * (-10.0_f64).exp()).abs() < 1e-6);
    }

    #[test]
    fn test_concentration_advdiff() {
        let c0 = 1.0;
        let ws = 0.001;
        let kappa = 0.01;
        let t = 100.0;
        let c = TurbidityCurrentMonitor::concentration_advection_diffusion(0.1, t, c0, ws, kappa);
        assert!(c > 0.0, "concentration should be positive");
        assert!(c.is_finite());
    }

    #[test]
    fn test_concentration_advdiff_zero_time() {
        let c =
            TurbidityCurrentMonitor::concentration_advection_diffusion(1.0, 0.0, 1.0, 0.001, 0.01);
        assert!((c - 0.0).abs() < 1e-9);
    }

    // ---- Event statistics ----

    #[test]
    fn test_event_statistics_empty() {
        let stats = TurbidityCurrentMonitor::event_statistics(&[]);
        assert_eq!(stats.count, 0);
        assert!((stats.mean_peak_ntu - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_event_statistics_multiple() {
        let events = vec![
            TurbidityEvent {
                onset_s: 0.0,
                end_s: 100.0,
                peak_ntu: 50.0,
                peak_time_s: 30.0,
                duration_s: 100.0,
                integrated_ntu_s: 2500.0,
                rise_time_s: 30.0,
                decay_tau_s: 40.0,
            },
            TurbidityEvent {
                onset_s: 86400.0,
                end_s: 86600.0,
                peak_ntu: 100.0,
                peak_time_s: 86450.0,
                duration_s: 200.0,
                integrated_ntu_s: 10000.0,
                rise_time_s: 50.0,
                decay_tau_s: 80.0,
            },
        ];
        let stats = TurbidityCurrentMonitor::event_statistics(&events);
        assert_eq!(stats.count, 2);
        assert!((stats.mean_peak_ntu - 75.0).abs() < 1e-9);
        assert!((stats.max_peak_ntu - 100.0).abs() < 1e-9);
        assert!((stats.mean_duration_s - 150.0).abs() < 1e-9);
        assert!(stats.mean_recurrence_s > 0.0);
    }

    // ---- Magnitude-frequency ----

    #[test]
    fn test_magnitude_frequency() {
        // Create events with decreasing counts at higher magnitudes
        let events = vec![
            TurbidityEvent {
                onset_s: 0.0,
                end_s: 10.0,
                peak_ntu: 10.0,
                peak_time_s: 5.0,
                duration_s: 10.0,
                integrated_ntu_s: 50.0,
                rise_time_s: 5.0,
                decay_tau_s: 5.0,
            },
            TurbidityEvent {
                onset_s: 100.0,
                end_s: 110.0,
                peak_ntu: 20.0,
                peak_time_s: 105.0,
                duration_s: 10.0,
                integrated_ntu_s: 100.0,
                rise_time_s: 5.0,
                decay_tau_s: 5.0,
            },
            TurbidityEvent {
                onset_s: 200.0,
                end_s: 210.0,
                peak_ntu: 50.0,
                peak_time_s: 205.0,
                duration_s: 10.0,
                integrated_ntu_s: 250.0,
                rise_time_s: 5.0,
                decay_tau_s: 5.0,
            },
        ];
        let bins = vec![5.0, 10.0, 20.0, 50.0];
        let (a, b) = TurbidityCurrentMonitor::magnitude_frequency(&events, &bins);
        // b should be positive (fewer large events)
        assert!(b > 0.0, "b={}", b);
        assert!(a.is_finite());
    }

    #[test]
    fn test_magnitude_frequency_empty() {
        let (a, b) = TurbidityCurrentMonitor::magnitude_frequency(&[], &[1.0, 10.0]);
        assert!((a - 0.0).abs() < 1e-9);
        assert!((b - 0.0).abs() < 1e-9);
    }

    // ---- Recurrence interval ----

    #[test]
    fn test_recurrence_interval() {
        let events = vec![
            TurbidityEvent {
                onset_s: 0.0,
                end_s: 10.0,
                peak_ntu: 30.0,
                peak_time_s: 5.0,
                duration_s: 10.0,
                integrated_ntu_s: 150.0,
                rise_time_s: 5.0,
                decay_tau_s: 5.0,
            },
            TurbidityEvent {
                onset_s: 100.0,
                end_s: 110.0,
                peak_ntu: 50.0,
                peak_time_s: 105.0,
                duration_s: 10.0,
                integrated_ntu_s: 250.0,
                rise_time_s: 5.0,
                decay_tau_s: 5.0,
            },
        ];
        let total = 1000.0;
        let ri = TurbidityCurrentMonitor::recurrence_interval(&events, 20.0, total);
        // 2 events above 20 NTU in 1000 s → ri = 500
        assert!((ri - 500.0).abs() < 1e-9);

        let ri_high = TurbidityCurrentMonitor::recurrence_interval(&events, 100.0, total);
        assert!(ri_high.is_infinite());
    }

    // ---- Front acceleration ----

    #[test]
    fn test_front_acceleration() {
        // 3 sensors, pulse arrives at different times
        let n = 200;
        let mut s0 = vec![0.0; n];
        let mut s1 = vec![0.0; n];
        let mut s2 = vec![0.0; n];
        for i in 50..70 {
            s0[i] = 100.0;
        }
        for i in 60..80 {
            s1[i] = 100.0;
        }
        for i in 75..95 {
            s2[i] = 100.0;
        }
        let configs = vec![
            SensorConfig {
                depth_m: 1000.0,
                cable_distance_m: 0.0,
                sampling_interval_s: 1.0,
                ntu_threshold: 5.0,
            },
            SensorConfig {
                depth_m: 1000.0,
                cable_distance_m: 1000.0,
                sampling_interval_s: 1.0,
                ntu_threshold: 5.0,
            },
            SensorConfig {
                depth_m: 1000.0,
                cable_distance_m: 2000.0,
                sampling_interval_s: 1.0,
                ntu_threshold: 5.0,
            },
        ];
        let sensors: Vec<&[f64]> = vec![&s0, &s1, &s2];
        let (vels, _accel) = TurbidityCurrentMonitor::front_acceleration(&sensors, &configs);
        assert_eq!(vels.len(), 2);
        assert!(vels[0].speed_ms > 0.0);
        assert!(vels[1].speed_ms > 0.0);
    }

    // ---- MonitorConfig default ----

    #[test]
    fn test_monitor_config_default() {
        let cfg = MonitorConfig::default();
        assert!((cfg.detection_threshold_ntu - 5.0).abs() < 1e-9);
        assert!((cfg.min_event_duration_s - 60.0).abs() < 1e-9);
        assert!((cfg.max_gap_s - 300.0).abs() < 1e-9);
        assert_eq!(cfg.num_sensors, 4);
    }

    // ---- Concentration profile monotone decrease ----

    #[test]
    fn test_concentration_profile_decreasing() {
        let c0 = 100.0;
        let ws = 0.001;
        let kappa = 0.01;
        let c1 = TurbidityCurrentMonitor::concentration_profile_exp(1.0, c0, ws, kappa);
        let c2 = TurbidityCurrentMonitor::concentration_profile_exp(2.0, c0, ws, kappa);
        let c3 = TurbidityCurrentMonitor::concentration_profile_exp(5.0, c0, ws, kappa);
        assert!(c1 > c2);
        assert!(c2 > c3);
        assert!(c3 > 0.0);
    }

    // ---- Froude zero height ----

    #[test]
    fn test_froude_zero_height() {
        let fr = TurbidityCurrentMonitor::froude_number(5.0, 0.1, 0.0);
        assert!((fr - 0.0).abs() < 1e-9);
    }

    // ---- NTU power-law calibration ----

    #[test]
    fn test_ntu_power_law() {
        // Non-linear calibration: C = 0.0005 * NTU^1.2
        let c = TurbidityCurrentMonitor::ntu_to_concentration(100.0, 0.0005, 1.2);
        let expected = 0.0005 * 100.0_f64.powf(1.2);
        assert!((c - expected).abs() < 1e-9);
    }
}
