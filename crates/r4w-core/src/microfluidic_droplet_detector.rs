//! # Microfluidic Droplet Detector
//!
//! Microfluidic droplet detection and characterization for lab-on-chip applications.
//! Supports optical (absorbance/fluorescence), impedance, and capacitive detection
//! modes for monitoring droplet generation, transport, and sorting in microchannels.
//!
//! ## Overview
//!
//! In droplet-based microfluidics, immiscible fluids are combined at a junction
//! (T-junction, flow-focusing, or co-flow) to generate picoliter-to-nanoliter
//! droplets at rates of 100 Hz to 10+ kHz. Downstream detection characterizes
//! each droplet's volume, velocity, contents, and generation regularity.
//!
//! ## Processing Pipeline
//!
//! ```text
//! Raw Signal → Baseline Estimation → Threshold Detection → Event Segmentation
//!     → Peak/Area Measurement → Volume Estimation → Statistics → Sorting Decision
//! ```
//!
//! ## Key Algorithms
//!
//! - **Hysteresis threshold**: Dual high/low thresholds prevent false triggers on noise
//! - **Baseline tracking**: Exponential moving average of inter-droplet signal
//! - **Volume estimation**: Plug flow (V = Q*dt), spherical, or slug geometry
//! - **Periodicity analysis**: FFT of droplet train for generation frequency
//! - **Polydispersity index**: PDI = (sigma/mean)^2 for size uniformity
//! - **Dimensionless numbers**: Ca, We, Re for flow regime characterization
//! - **Coalescence/fission detection**: Anomalous event width or amplitude changes

use std::f64::consts::PI;

// ---------- Enums & Configuration ----------

/// Detection modality for droplet sensing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionMode {
    /// Optical absorbance or fluorescence measurement.
    Optical,
    /// Impedance measurement between microelectrodes.
    Impedance,
    /// Capacitive sensing of dielectric change.
    Capacitive,
}

/// Geometry model used for volume estimation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VolumeModel {
    /// Plug flow: V = flow_rate * duration.
    PlugFlow,
    /// Spherical droplet: V = (4/3)*pi*r^3, diameter from cross-section.
    Spherical,
    /// Slug/plug filling channel cross-section: V = A_channel * L_droplet.
    Slug,
}

/// Sorting decision for a detected droplet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SortDecision {
    /// Droplet meets criteria — route to collection.
    Sort,
    /// Droplet does not meet criteria — route to waste.
    Waste,
}

/// A detected coalescence or fission event.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MergeEvent {
    /// Two droplets merged into one (coalescence).
    Coalescence {
        /// Index of the merged droplet in the event list.
        merged_index: usize,
        /// Time of the coalescence event in seconds.
        time_s: f64,
    },
    /// One droplet split into two (fission).
    Fission {
        /// Index of the parent droplet.
        parent_index: usize,
        /// Time of the fission event in seconds.
        time_s: f64,
    },
}

/// Configuration for the microchannel geometry.
#[derive(Debug, Clone)]
pub struct ChannelConfig {
    /// Channel width in micrometers.
    pub channel_width_um: f64,
    /// Channel height in micrometers.
    pub channel_height_um: f64,
    /// Distance from droplet generation junction to the detection point, in micrometers.
    pub detection_point_um: f64,
}

impl ChannelConfig {
    /// Creates a new channel configuration.
    pub fn new(width_um: f64, height_um: f64, detection_um: f64) -> Self {
        Self {
            channel_width_um: width_um,
            channel_height_um: height_um,
            detection_point_um: detection_um,
        }
    }

    /// Cross-sectional area in square meters.
    pub fn cross_section_m2(&self) -> f64 {
        (self.channel_width_um * 1e-6) * (self.channel_height_um * 1e-6)
    }

    /// Hydraulic diameter D_h = 2*w*h / (w+h) for a rectangular channel, in meters.
    pub fn hydraulic_diameter_m(&self) -> f64 {
        let w = self.channel_width_um * 1e-6;
        let h = self.channel_height_um * 1e-6;
        2.0 * w * h / (w + h)
    }
}

/// A single detected droplet event.
#[derive(Debug, Clone)]
pub struct DropletEvent {
    /// Time of the rising edge crossing the high threshold (seconds).
    pub start_time_s: f64,
    /// Time of the falling edge crossing the low threshold (seconds).
    pub end_time_s: f64,
    /// Peak signal amplitude within the droplet window.
    pub peak_amplitude: f64,
    /// Integrated signal area (trapezoidal rule) above baseline.
    pub area: f64,
    /// Estimated volume in picoliters.
    pub volume_pl: f64,
    /// Estimated velocity in m/s (only set when two-detector data is available).
    pub velocity_ms: f64,
}

impl DropletEvent {
    /// Duration of the droplet event in seconds.
    pub fn duration_s(&self) -> f64 {
        self.end_time_s - self.start_time_s
    }
}

/// Fluorescence characterization of a single droplet.
#[derive(Debug, Clone)]
pub struct FluorescenceResult {
    /// Peak fluorescence intensity (arbitrary units).
    pub peak_intensity: f64,
    /// Integrated fluorescence over the droplet duration.
    pub integrated_intensity: f64,
    /// Sorting decision based on threshold.
    pub decision: SortDecision,
}

/// Impedance measurement for a droplet passing between electrodes.
#[derive(Debug, Clone)]
pub struct ImpedanceResult {
    /// Baseline impedance magnitude (ohms).
    pub baseline_impedance_ohm: f64,
    /// Droplet impedance magnitude (ohms).
    pub droplet_impedance_ohm: f64,
    /// Differential impedance change (ohms).
    pub delta_impedance_ohm: f64,
    /// Estimated conductivity of the droplet contents (S/m).
    pub conductivity_s_m: f64,
}

/// Size-distribution statistics for a population of droplets.
#[derive(Debug, Clone)]
pub struct SizeDistribution {
    /// Number of droplets.
    pub count: usize,
    /// Mean volume in picoliters.
    pub mean_volume_pl: f64,
    /// Median volume in picoliters.
    pub median_volume_pl: f64,
    /// Standard deviation of volume in picoliters.
    pub std_volume_pl: f64,
    /// Coefficient of variation (std/mean).
    pub cv: f64,
    /// Polydispersity index: (sigma/mean)^2.
    pub pdi: f64,
    /// Histogram bin edges in picoliters.
    pub histogram_edges: Vec<f64>,
    /// Histogram counts per bin.
    pub histogram_counts: Vec<usize>,
}

/// Generation-frequency analysis result.
#[derive(Debug, Clone)]
pub struct GenerationAnalysis {
    /// Droplet generation rate in Hz.
    pub rate_hz: f64,
    /// Mean inter-droplet gap time in seconds.
    pub mean_gap_s: f64,
    /// Standard deviation of gap time in seconds.
    pub std_gap_s: f64,
    /// Coefficient of variation of gap times (regularity metric).
    pub gap_cv: f64,
    /// Dominant frequency from FFT of the droplet train (Hz).
    pub dominant_freq_hz: f64,
}

/// Dimensionless flow numbers characterizing the droplet generation regime.
#[derive(Debug, Clone)]
pub struct FlowNumbers {
    /// Capillary number Ca = mu*v / gamma.
    pub capillary: f64,
    /// Weber number We = rho*v^2*D / gamma.
    pub weber: f64,
    /// Reynolds number Re = rho*v*D_h / mu.
    pub reynolds: f64,
}

// ---------- Detector ----------

/// Configurable microfluidic droplet detector.
///
/// Processes a one-dimensional signal stream (optical, impedance, or capacitive)
/// and extracts individual [`DropletEvent`]s together with population statistics.
#[derive(Debug, Clone)]
pub struct DropletDetector {
    /// Sample rate of the input signal in Hz.
    pub sample_rate_hz: f64,
    /// High threshold for triggering a droplet event (rising edge).
    pub threshold_high: f64,
    /// Low threshold for releasing a droplet event (falling edge, hysteresis).
    pub threshold_low: f64,
    /// Minimum allowed droplet duration in seconds (reject shorter).
    pub min_droplet_width_s: f64,
    /// Maximum allowed droplet duration in seconds (reject longer).
    pub max_droplet_width_s: f64,
    /// Detection modality.
    pub detection_mode: DetectionMode,
    /// Channel geometry (optional; needed for volume estimation).
    pub channel: Option<ChannelConfig>,
    /// Volume estimation model.
    pub volume_model: VolumeModel,
    /// Flow rate in m^3/s (needed for plug-flow volume).
    pub flow_rate_m3s: f64,
    /// Baseline EMA smoothing factor (0..1, smaller = slower adaptation).
    pub baseline_alpha: f64,
}

impl DropletDetector {
    /// Creates a detector with sensible defaults for optical detection at 100 kHz.
    pub fn new(sample_rate_hz: f64, threshold_high: f64, threshold_low: f64) -> Self {
        Self {
            sample_rate_hz,
            threshold_high,
            threshold_low,
            min_droplet_width_s: 1e-5,
            max_droplet_width_s: 0.1,
            detection_mode: DetectionMode::Optical,
            channel: None,
            volume_model: VolumeModel::PlugFlow,
            flow_rate_m3s: 1e-11, // 10 nL/s default
            baseline_alpha: 0.001,
        }
    }

    // ---- Baseline estimation ----

    /// Estimate baseline with exponential moving average.
    /// Only updates when the signal is *below* the low threshold
    /// to avoid biasing the baseline with droplet signal.
    pub fn estimate_baseline(&self, signal: &[f64]) -> Vec<f64> {
        let mut baseline = vec![0.0; signal.len()];
        if signal.is_empty() {
            return baseline;
        }
        let mut ema = signal[0];
        for (i, &s) in signal.iter().enumerate() {
            if s < self.threshold_low {
                ema += self.baseline_alpha * (s - ema);
            }
            baseline[i] = ema;
        }
        baseline
    }

    // ---- Detection ----

    /// Detect droplet events from a signal using hysteresis thresholding.
    ///
    /// Returns a list of `DropletEvent`s with timing, amplitude, area, and volume.
    pub fn detect(&self, signal: &[f64]) -> Vec<DropletEvent> {
        let baseline = self.estimate_baseline(signal);
        let dt = 1.0 / self.sample_rate_hz;

        let mut events = Vec::new();
        let mut in_droplet = false;
        let mut start_idx: usize = 0;
        let mut peak: f64 = f64::NEG_INFINITY;
        let mut area: f64 = 0.0;

        for (i, &s) in signal.iter().enumerate() {
            let corrected = s - baseline[i];
            if !in_droplet {
                if corrected >= self.threshold_high {
                    in_droplet = true;
                    start_idx = i;
                    peak = corrected;
                    area = corrected * dt;
                }
            } else {
                if corrected > peak {
                    peak = corrected;
                }
                area += corrected * dt;
                if corrected < self.threshold_low {
                    in_droplet = false;
                    let start_t = start_idx as f64 * dt;
                    let end_t = i as f64 * dt;
                    let duration = end_t - start_t;
                    if duration >= self.min_droplet_width_s
                        && duration <= self.max_droplet_width_s
                    {
                        let volume_pl = self.estimate_volume(duration);
                        events.push(DropletEvent {
                            start_time_s: start_t,
                            end_time_s: end_t,
                            peak_amplitude: peak,
                            area,
                            volume_pl,
                            velocity_ms: 0.0,
                        });
                    }
                    peak = f64::NEG_INFINITY;
                    area = 0.0;
                }
            }
        }
        events
    }

    // ---- Volume estimation ----

    /// Estimate droplet volume in picoliters from its transit duration.
    pub fn estimate_volume(&self, duration_s: f64) -> f64 {
        match self.volume_model {
            VolumeModel::PlugFlow => {
                // V = Q * dt  (m^3 → pL: 1 m^3 = 1e15 pL)
                self.flow_rate_m3s * duration_s * 1e15
            }
            VolumeModel::Spherical => {
                // Assume diameter = channel smaller dimension * fill factor ~1
                if let Some(ref ch) = self.channel {
                    let d_m = ch.channel_height_um.min(ch.channel_width_um) * 1e-6;
                    let r = d_m / 2.0;
                    (4.0 / 3.0) * PI * r * r * r * 1e15
                } else {
                    0.0
                }
            }
            VolumeModel::Slug => {
                // V = A_channel * L_droplet;  L = v * dt
                if let Some(ref ch) = self.channel {
                    let a = ch.cross_section_m2();
                    // Estimate length from flow_rate/cross_section * duration
                    let v = self.flow_rate_m3s / a;
                    let l = v * duration_s;
                    a * l * 1e15
                } else {
                    0.0
                }
            }
        }
    }

    /// Compute the spherical-equivalent diameter in micrometers from a volume in pL.
    pub fn diameter_from_volume_um(volume_pl: f64) -> f64 {
        // V_m3 = volume_pl * 1e-15
        let v_m3 = volume_pl * 1e-15;
        let r = (3.0 * v_m3 / (4.0 * PI)).cbrt();
        r * 2.0 * 1e6 // metres to micrometers
    }

    // ---- Velocity from two detectors ----

    /// Compute droplet velocity from transit time between two detection points.
    ///
    /// `distance_um` is the separation between detectors in micrometers,
    /// `dt_s` is the measured transit time in seconds.
    pub fn velocity_from_transit(distance_um: f64, dt_s: f64) -> f64 {
        if dt_s <= 0.0 {
            return 0.0;
        }
        (distance_um * 1e-6) / dt_s
    }

    // ---- Generation frequency analysis ----

    /// Analyse the generation regularity of a sequence of detected droplets.
    pub fn generation_analysis(&self, events: &[DropletEvent]) -> Option<GenerationAnalysis> {
        if events.len() < 2 {
            return None;
        }
        let n = events.len();
        let total_time = events[n - 1].start_time_s - events[0].start_time_s;
        let rate = if total_time > 0.0 {
            (n - 1) as f64 / total_time
        } else {
            0.0
        };

        // Inter-droplet gap times
        let gaps: Vec<f64> = events
            .windows(2)
            .map(|w| w[1].start_time_s - w[0].end_time_s)
            .collect();

        let mean_gap = gaps.iter().sum::<f64>() / gaps.len() as f64;
        let var_gap = gaps
            .iter()
            .map(|g| (g - mean_gap) * (g - mean_gap))
            .sum::<f64>()
            / gaps.len() as f64;
        let std_gap = var_gap.sqrt();
        let gap_cv = if mean_gap > 0.0 {
            std_gap / mean_gap
        } else {
            0.0
        };

        // FFT of droplet occurrence times for periodicity
        let dominant = self.droplet_train_fft(events);

        Some(GenerationAnalysis {
            rate_hz: rate,
            mean_gap_s: mean_gap,
            std_gap_s: std_gap,
            gap_cv,
            dominant_freq_hz: dominant,
        })
    }

    /// Simple FFT of a droplet-train binary signal to find dominant generation frequency.
    ///
    /// Constructs a binary pulse train (1 at droplet centres, 0 elsewhere) and
    /// computes a radix-2 FFT to find the dominant spectral peak.
    fn droplet_train_fft(&self, events: &[DropletEvent]) -> f64 {
        if events.len() < 2 {
            return 0.0;
        }
        let last_t = events.last().unwrap().end_time_s;
        // Use up to 1024 samples for the FFT (power-of-2)
        let n = 1024usize;
        let dt = last_t / n as f64;
        if dt <= 0.0 {
            return 0.0;
        }
        let mut train = vec![0.0f64; n];
        for ev in events {
            let centre = (ev.start_time_s + ev.end_time_s) / 2.0;
            let idx = ((centre / dt) as usize).min(n - 1);
            train[idx] = 1.0;
        }

        // In-place radix-2 DFT (Cooley-Tukey)
        let (mag, _) = fft_magnitude(&train);

        // Find peak in positive-frequency half (skip DC bin 0)
        let half = n / 2;
        let mut best_bin = 1;
        let mut best_val = 0.0f64;
        for i in 1..half {
            if mag[i] > best_val {
                best_val = mag[i];
                best_bin = i;
            }
        }
        best_bin as f64 / (n as f64 * dt)
    }

    // ---- Size distribution ----

    /// Compute size-distribution statistics from a population of droplet events.
    pub fn size_distribution(&self, events: &[DropletEvent], num_bins: usize) -> SizeDistribution {
        let count = events.len();
        if count == 0 {
            return SizeDistribution {
                count: 0,
                mean_volume_pl: 0.0,
                median_volume_pl: 0.0,
                std_volume_pl: 0.0,
                cv: 0.0,
                pdi: 0.0,
                histogram_edges: vec![],
                histogram_counts: vec![],
            };
        }
        let mut vols: Vec<f64> = events.iter().map(|e| e.volume_pl).collect();
        let mean = vols.iter().sum::<f64>() / count as f64;
        let var = vols
            .iter()
            .map(|v| (v - mean) * (v - mean))
            .sum::<f64>()
            / count as f64;
        let std = var.sqrt();
        let cv = if mean > 0.0 { std / mean } else { 0.0 };
        let pdi = cv * cv;

        vols.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = if count % 2 == 0 {
            (vols[count / 2 - 1] + vols[count / 2]) / 2.0
        } else {
            vols[count / 2]
        };

        // Histogram
        let min_v = vols[0];
        let max_v = vols[count - 1];
        let bins = num_bins.max(1);
        let range = max_v - min_v;
        let bin_width = if range > 0.0 {
            range / bins as f64
        } else {
            1.0
        };
        let mut edges = Vec::with_capacity(bins + 1);
        for i in 0..=bins {
            edges.push(min_v + i as f64 * bin_width);
        }
        let mut counts = vec![0usize; bins];
        for &v in &vols {
            let mut b = ((v - min_v) / bin_width) as usize;
            if b >= bins {
                b = bins - 1;
            }
            counts[b] += 1;
        }

        SizeDistribution {
            count,
            mean_volume_pl: mean,
            median_volume_pl: median,
            std_volume_pl: std,
            cv,
            pdi,
            histogram_edges: edges,
            histogram_counts: counts,
        }
    }

    // ---- Flow-rate estimation ----

    /// Estimate volumetric flow rate from droplet frequency and mean volume.
    ///
    /// Q = f * V_mean  (returns m^3/s).
    pub fn estimate_flow_rate(freq_hz: f64, mean_volume_pl: f64) -> f64 {
        freq_hz * mean_volume_pl * 1e-15
    }

    // ---- Dimensionless numbers ----

    /// Compute capillary, Weber, and Reynolds numbers.
    ///
    /// * `mu_pa_s`   — dynamic viscosity of continuous phase (Pa·s)
    /// * `rho_kg_m3` — density of dispersed phase (kg/m^3)
    /// * `gamma_n_m` — interfacial tension (N/m)
    /// * `velocity_ms` — droplet velocity (m/s)
    /// * `channel`   — channel configuration for hydraulic diameter
    pub fn flow_numbers(
        mu_pa_s: f64,
        rho_kg_m3: f64,
        gamma_n_m: f64,
        velocity_ms: f64,
        channel: &ChannelConfig,
    ) -> FlowNumbers {
        let d_h = channel.hydraulic_diameter_m();
        let ca = mu_pa_s * velocity_ms / gamma_n_m;
        let we = rho_kg_m3 * velocity_ms * velocity_ms * d_h / gamma_n_m;
        let re = rho_kg_m3 * velocity_ms * d_h / mu_pa_s;
        FlowNumbers {
            capillary: ca,
            weber: we,
            reynolds: re,
        }
    }

    // ---- Impedance-based detection ----

    /// Analyse impedance change as a droplet passes between electrodes.
    ///
    /// Models the sensing region as a parallel R-C circuit.
    /// * `baseline_r_ohm`  — resistance without droplet
    /// * `baseline_c_f`    — capacitance without droplet
    /// * `droplet_r_ohm`   — resistance with droplet present
    /// * `droplet_c_f`     — capacitance with droplet present
    /// * `freq_hz`         — excitation frequency
    /// * `cell_constant_m` — electrode cell constant k (1/m)
    pub fn impedance_analysis(
        baseline_r_ohm: f64,
        baseline_c_f: f64,
        droplet_r_ohm: f64,
        droplet_c_f: f64,
        freq_hz: f64,
        cell_constant_m: f64,
    ) -> ImpedanceResult {
        let omega = 2.0 * PI * freq_hz;
        let z_baseline = complex_impedance_mag(baseline_r_ohm, baseline_c_f, omega);
        let z_droplet = complex_impedance_mag(droplet_r_ohm, droplet_c_f, omega);
        let delta = z_droplet - z_baseline;
        // Conductivity sigma = cell_constant / R
        let conductivity = if droplet_r_ohm > 0.0 {
            cell_constant_m / droplet_r_ohm
        } else {
            0.0
        };
        ImpedanceResult {
            baseline_impedance_ohm: z_baseline,
            droplet_impedance_ohm: z_droplet,
            delta_impedance_ohm: delta,
            conductivity_s_m: conductivity,
        }
    }

    // ---- Fluorescence sorting ----

    /// Evaluate fluorescence of a droplet event and make a sorting decision.
    ///
    /// `fluorescence_signal` is the raw fluorescence channel aligned with the
    /// event indices.  `sort_threshold` is the minimum integrated intensity to sort.
    pub fn fluorescence_analysis(
        &self,
        fluorescence_signal: &[f64],
        event: &DropletEvent,
        sort_threshold: f64,
    ) -> FluorescenceResult {
        let dt = 1.0 / self.sample_rate_hz;
        let i_start = (event.start_time_s * self.sample_rate_hz) as usize;
        let i_end = ((event.end_time_s * self.sample_rate_hz) as usize)
            .min(fluorescence_signal.len());

        let mut peak = f64::NEG_INFINITY;
        let mut integrated = 0.0;
        for i in i_start..i_end {
            let v = fluorescence_signal[i];
            if v > peak {
                peak = v;
            }
            integrated += v * dt;
        }
        if peak == f64::NEG_INFINITY {
            peak = 0.0;
        }
        let decision = if integrated >= sort_threshold {
            SortDecision::Sort
        } else {
            SortDecision::Waste
        };
        FluorescenceResult {
            peak_intensity: peak,
            integrated_intensity: integrated,
            decision,
        }
    }

    // ---- Merging / splitting detection ----

    /// Detect coalescence and fission events in a droplet train.
    ///
    /// Uses statistical outlier detection on droplet duration and area:
    /// * A droplet significantly wider/larger than the median is flagged as **coalescence**.
    /// * A droplet significantly narrower/smaller is flagged as **fission**.
    ///
    /// `sigma_factor` is the number of standard deviations for the outlier threshold
    /// (typically 2.0–3.0).
    pub fn detect_merge_split(
        events: &[DropletEvent],
        sigma_factor: f64,
    ) -> Vec<MergeEvent> {
        if events.len() < 3 {
            return vec![];
        }
        let areas: Vec<f64> = events.iter().map(|e| e.area).collect();
        let mean = areas.iter().sum::<f64>() / areas.len() as f64;
        let var = areas
            .iter()
            .map(|a| (a - mean) * (a - mean))
            .sum::<f64>()
            / areas.len() as f64;
        let std = var.sqrt();

        let hi = mean + sigma_factor * std;
        let lo = mean - sigma_factor * std;

        let mut results = Vec::new();
        for (i, ev) in events.iter().enumerate() {
            if ev.area > hi {
                results.push(MergeEvent::Coalescence {
                    merged_index: i,
                    time_s: ev.start_time_s,
                });
            } else if ev.area < lo && ev.area > 0.0 {
                results.push(MergeEvent::Fission {
                    parent_index: i,
                    time_s: ev.start_time_s,
                });
            }
        }
        results
    }

    // ---- Dual-detector velocity ----

    /// Given events detected at two detectors separated by `distance_um`,
    /// match droplets by order and compute per-droplet velocities.
    ///
    /// Returns a vector of velocities (m/s) for matched pairs.
    pub fn dual_detector_velocity(
        events_a: &[DropletEvent],
        events_b: &[DropletEvent],
        distance_um: f64,
    ) -> Vec<f64> {
        let n = events_a.len().min(events_b.len());
        let mut velocities = Vec::with_capacity(n);
        for i in 0..n {
            let dt = events_b[i].start_time_s - events_a[i].start_time_s;
            velocities.push(Self::velocity_from_transit(distance_um, dt));
        }
        velocities
    }
}

// ---------- Helper functions ----------

/// Complex impedance magnitude for a parallel R-C circuit.
///
/// Z = R / sqrt(1 + (omega*R*C)^2)
fn complex_impedance_mag(r: f64, c: f64, omega: f64) -> f64 {
    let x = omega * r * c;
    r / (1.0 + x * x).sqrt()
}

/// Radix-2 Cooley-Tukey FFT returning (magnitude, phase) vectors.
///
/// Input is zero-padded or truncated to length `n` (must be power of 2).
fn fft_magnitude(input: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = input.len();
    assert!(n > 0 && n.is_power_of_two(), "FFT length must be power of 2");

    // Bit-reversal permutation
    let mut re = input.to_vec();
    let mut im = vec![0.0f64; n];
    let bits = n.trailing_zeros();
    for i in 0..n {
        let j = i.reverse_bits() >> (usize::BITS - bits);
        if i < j {
            re.swap(i, j);
        }
    }

    // Butterfly stages
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = -2.0 * PI / size as f64;
        for k in (0..n).step_by(size) {
            for j in 0..half {
                let angle = angle_step * j as f64;
                let wr = angle.cos();
                let wi = angle.sin();
                let a = k + j;
                let b = k + j + half;
                let tr = wr * re[b] - wi * im[b];
                let ti = wr * im[b] + wi * re[b];
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
            }
        }
        size <<= 1;
    }

    let mag: Vec<f64> = re.iter().zip(im.iter()).map(|(r, i)| (r * r + i * i).sqrt()).collect();
    let phase: Vec<f64> = re.iter().zip(im.iter()).map(|(r, i)| i.atan2(*r)).collect();
    (mag, phase)
}

// ---------- Tests ----------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_detector() -> DropletDetector {
        DropletDetector::new(100_000.0, 0.5, 0.2)
    }

    fn make_channel() -> ChannelConfig {
        ChannelConfig::new(100.0, 50.0, 5000.0)
    }

    // Synthesise a signal with `n_droplets` identical Gaussian-like pulses.
    fn synth_droplet_train(
        sample_rate: f64,
        n_droplets: usize,
        period_s: f64,
        pulse_width_s: f64,
        amplitude: f64,
    ) -> Vec<f64> {
        let total = (period_s * n_droplets as f64 * sample_rate) as usize + 1;
        let mut sig = vec![0.0f64; total];
        for d in 0..n_droplets {
            let centre = (d as f64 + 0.5) * period_s;
            let sigma = pulse_width_s / 4.0; // ~95% energy within pulse_width
            let i_start = ((centre - pulse_width_s) * sample_rate) as usize;
            let i_end = (((centre + pulse_width_s) * sample_rate) as usize).min(total);
            for i in i_start..i_end {
                let t = i as f64 / sample_rate - centre;
                sig[i] += amplitude * (-0.5 * (t / sigma).powi(2)).exp();
            }
        }
        sig
    }

    // --- ChannelConfig tests ---

    #[test]
    fn test_channel_cross_section() {
        let ch = ChannelConfig::new(100.0, 50.0, 5000.0);
        // 100 um * 50 um = 5000 um^2 = 5e-9 m^2
        assert!((ch.cross_section_m2() - 5e-9).abs() < 1e-15);
    }

    #[test]
    fn test_hydraulic_diameter() {
        let ch = ChannelConfig::new(100.0, 50.0, 5000.0);
        // D_h = 2*100*50/(100+50) um = 66.667 um = 6.6667e-5 m
        let expected = 2.0 * 100.0 * 50.0 / 150.0 * 1e-6;
        assert!((ch.hydraulic_diameter_m() - expected).abs() < 1e-12);
    }

    #[test]
    fn test_channel_square() {
        let ch = ChannelConfig::new(100.0, 100.0, 1000.0);
        // Square: D_h = 2*100*100/200 = 100 um
        assert!((ch.hydraulic_diameter_m() - 100e-6).abs() < 1e-12);
    }

    // --- DropletEvent tests ---

    #[test]
    fn test_droplet_event_duration() {
        let ev = DropletEvent {
            start_time_s: 0.001,
            end_time_s: 0.003,
            peak_amplitude: 1.0,
            area: 0.5,
            volume_pl: 100.0,
            velocity_ms: 0.0,
        };
        assert!((ev.duration_s() - 0.002).abs() < 1e-12);
    }

    // --- Baseline estimation ---

    #[test]
    fn test_baseline_constant() {
        let det = make_detector();
        let sig = vec![0.1; 1000];
        let bl = det.estimate_baseline(&sig);
        // Should converge towards 0.1
        let last = bl[999];
        assert!((last - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_baseline_ignores_droplet_peaks() {
        let mut det = make_detector();
        det.baseline_alpha = 0.01;
        let mut sig = vec![0.1; 2000];
        // Insert a big pulse
        for i in 500..600 {
            sig[i] = 2.0;
        }
        let bl = det.estimate_baseline(&sig);
        // After the pulse, baseline should still be close to 0.1
        assert!((bl[1999] - 0.1).abs() < 0.05);
    }

    #[test]
    fn test_baseline_empty_signal() {
        let det = make_detector();
        let bl = det.estimate_baseline(&[]);
        assert!(bl.is_empty());
    }

    // --- Detection tests ---

    #[test]
    fn test_detect_single_droplet() {
        let mut det = make_detector();
        det.min_droplet_width_s = 0.0;
        let sig = synth_droplet_train(100_000.0, 1, 0.005, 0.001, 1.0);
        let events = det.detect(&sig);
        assert_eq!(events.len(), 1);
        assert!(events[0].peak_amplitude > 0.5);
    }

    #[test]
    fn test_detect_multiple_droplets() {
        let mut det = make_detector();
        det.min_droplet_width_s = 0.0;
        let sig = synth_droplet_train(100_000.0, 5, 0.005, 0.001, 1.0);
        let events = det.detect(&sig);
        assert_eq!(events.len(), 5);
    }

    #[test]
    fn test_no_detection_below_threshold() {
        let det = make_detector();
        let sig = vec![0.1; 10000]; // Well below threshold
        let events = det.detect(&sig);
        assert!(events.is_empty());
    }

    #[test]
    fn test_min_width_filter() {
        let mut det = make_detector();
        det.min_droplet_width_s = 1.0; // Impossibly wide
        let sig = synth_droplet_train(100_000.0, 3, 0.005, 0.001, 1.0);
        let events = det.detect(&sig);
        assert!(events.is_empty());
    }

    #[test]
    fn test_max_width_filter() {
        let mut det = make_detector();
        det.min_droplet_width_s = 0.0;
        det.max_droplet_width_s = 1e-6; // Impossibly narrow
        let sig = synth_droplet_train(100_000.0, 3, 0.005, 0.001, 1.0);
        let events = det.detect(&sig);
        assert!(events.is_empty());
    }

    #[test]
    fn test_hysteresis_prevents_chatter() {
        let mut det = make_detector();
        det.threshold_high = 0.6;
        det.threshold_low = 0.3;
        det.min_droplet_width_s = 0.0;
        // Signal that hovers near threshold but only fully crosses once
        let mut sig = vec![0.0; 2000];
        for i in 400..600 {
            sig[i] = 0.8;
        }
        // Small bumps that don't reach high threshold
        for i in 200..220 {
            sig[i] = 0.4;
        }
        let events = det.detect(&sig);
        assert_eq!(events.len(), 1);
    }

    // --- Volume estimation ---

    #[test]
    fn test_volume_plug_flow() {
        let mut det = make_detector();
        det.volume_model = VolumeModel::PlugFlow;
        det.flow_rate_m3s = 1e-12; // 1 pL/s
        let v = det.estimate_volume(1.0);
        // V = 1e-12 * 1.0 * 1e15 = 1000 pL
        assert!((v - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_volume_spherical() {
        let mut det = make_detector();
        det.volume_model = VolumeModel::Spherical;
        det.channel = Some(ChannelConfig::new(100.0, 50.0, 5000.0));
        let v = det.estimate_volume(0.001); // duration unused for spherical
        // diameter = min(100,50) = 50 um; r = 25 um = 25e-6 m
        let expected = (4.0 / 3.0) * PI * (25e-6_f64).powi(3) * 1e15;
        assert!((v - expected).abs() < 1e-3);
    }

    #[test]
    fn test_volume_slug() {
        let mut det = make_detector();
        det.volume_model = VolumeModel::Slug;
        det.flow_rate_m3s = 1e-11;
        det.channel = Some(ChannelConfig::new(100.0, 50.0, 5000.0));
        let dur = 0.001;
        let v = det.estimate_volume(dur);
        let a = 5e-9; // m^2
        let vel = 1e-11 / a; // 2e-3 m/s
        let l = vel * dur;   // 2e-6 m
        let expected = a * l * 1e15;
        assert!((v - expected).abs() < 1e-6);
    }

    #[test]
    fn test_diameter_from_volume() {
        // 1 pL sphere: V = 1e-15 m^3
        let d = DropletDetector::diameter_from_volume_um(1.0);
        // r = (3*1e-15/(4*pi))^(1/3), d = 2*r in um
        let r = (3.0 * 1e-15 / (4.0 * PI)).cbrt();
        let expected = 2.0 * r * 1e6;
        assert!((d - expected).abs() < 1e-6);
    }

    // --- Velocity ---

    #[test]
    fn test_velocity_from_transit() {
        let v = DropletDetector::velocity_from_transit(1000.0, 0.001);
        // 1000 um / 1 ms = 1e-3 m / 1e-3 s = 1 m/s
        assert!((v - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_velocity_zero_dt() {
        let v = DropletDetector::velocity_from_transit(1000.0, 0.0);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn test_dual_detector_velocity() {
        let a = vec![
            DropletEvent { start_time_s: 0.001, end_time_s: 0.002, peak_amplitude: 1.0, area: 0.5, volume_pl: 10.0, velocity_ms: 0.0 },
            DropletEvent { start_time_s: 0.011, end_time_s: 0.012, peak_amplitude: 1.0, area: 0.5, volume_pl: 10.0, velocity_ms: 0.0 },
        ];
        let b = vec![
            DropletEvent { start_time_s: 0.002, end_time_s: 0.003, peak_amplitude: 1.0, area: 0.5, volume_pl: 10.0, velocity_ms: 0.0 },
            DropletEvent { start_time_s: 0.012, end_time_s: 0.013, peak_amplitude: 1.0, area: 0.5, volume_pl: 10.0, velocity_ms: 0.0 },
        ];
        let vels = DropletDetector::dual_detector_velocity(&a, &b, 1000.0);
        assert_eq!(vels.len(), 2);
        // 1000 um / 1 ms = 1 m/s
        assert!((vels[0] - 1.0).abs() < 1e-9);
        assert!((vels[1] - 1.0).abs() < 1e-9);
    }

    // --- Generation analysis ---

    #[test]
    fn test_generation_rate() {
        let mut det = make_detector();
        det.min_droplet_width_s = 0.0;
        let sig = synth_droplet_train(100_000.0, 10, 0.01, 0.002, 1.0);
        let events = det.detect(&sig);
        assert!(events.len() >= 8); // May miss edges
        if let Some(ga) = det.generation_analysis(&events) {
            // ~100 Hz rate for 10 ms period
            assert!(ga.rate_hz > 50.0 && ga.rate_hz < 200.0);
            assert!(ga.mean_gap_s > 0.0);
        }
    }

    #[test]
    fn test_generation_analysis_too_few() {
        let det = make_detector();
        let events = vec![DropletEvent {
            start_time_s: 0.0,
            end_time_s: 0.001,
            peak_amplitude: 1.0,
            area: 0.5,
            volume_pl: 10.0,
            velocity_ms: 0.0,
        }];
        assert!(det.generation_analysis(&events).is_none());
    }

    #[test]
    fn test_gap_cv_perfect_train() {
        // Perfectly periodic → CV ~ 0
        let events: Vec<DropletEvent> = (0..20)
            .map(|i| DropletEvent {
                start_time_s: i as f64 * 0.01,
                end_time_s: i as f64 * 0.01 + 0.002,
                peak_amplitude: 1.0,
                area: 0.5,
                volume_pl: 10.0,
                velocity_ms: 0.0,
            })
            .collect();
        let det = make_detector();
        let ga = det.generation_analysis(&events).unwrap();
        assert!(ga.gap_cv < 0.01);
    }

    // --- Size distribution ---

    #[test]
    fn test_size_distribution_basic() {
        let events: Vec<DropletEvent> = (0..100)
            .map(|i| DropletEvent {
                start_time_s: i as f64 * 0.01,
                end_time_s: i as f64 * 0.01 + 0.001,
                peak_amplitude: 1.0,
                area: 0.5,
                volume_pl: 100.0 + (i % 10) as f64,
                velocity_ms: 0.0,
            })
            .collect();
        let det = make_detector();
        let sd = det.size_distribution(&events, 5);
        assert_eq!(sd.count, 100);
        assert!(sd.mean_volume_pl > 99.0 && sd.mean_volume_pl < 110.0);
        assert!(sd.pdi < 0.01); // Low dispersion
        assert_eq!(sd.histogram_counts.iter().sum::<usize>(), 100);
    }

    #[test]
    fn test_size_distribution_empty() {
        let det = make_detector();
        let sd = det.size_distribution(&[], 10);
        assert_eq!(sd.count, 0);
    }

    #[test]
    fn test_pdi_monodisperse() {
        // All same volume → PDI = 0
        let events: Vec<DropletEvent> = (0..50)
            .map(|i| DropletEvent {
                start_time_s: i as f64 * 0.01,
                end_time_s: i as f64 * 0.01 + 0.001,
                peak_amplitude: 1.0,
                area: 0.5,
                volume_pl: 42.0,
                velocity_ms: 0.0,
            })
            .collect();
        let det = make_detector();
        let sd = det.size_distribution(&events, 5);
        assert!(sd.pdi < 1e-12);
    }

    // --- Flow rate estimation ---

    #[test]
    fn test_estimate_flow_rate() {
        // 100 Hz, 100 pL each → Q = 100 * 100e-15 = 1e-11 m^3/s = 10 nL/s
        let q = DropletDetector::estimate_flow_rate(100.0, 100.0);
        assert!((q - 1e-11).abs() < 1e-20);
    }

    // --- Dimensionless numbers ---

    #[test]
    fn test_flow_numbers_ca() {
        let ch = make_channel();
        // mu = 1e-3 Pa·s (water), v = 0.01 m/s, gamma = 0.072 N/m
        let fn_ = DropletDetector::flow_numbers(1e-3, 1000.0, 0.072, 0.01, &ch);
        // Ca = 1e-3 * 0.01 / 0.072 = 1.389e-4
        let expected_ca = 1e-3 * 0.01 / 0.072;
        assert!((fn_.capillary - expected_ca).abs() < 1e-10);
    }

    #[test]
    fn test_flow_numbers_re() {
        let ch = make_channel();
        let fn_ = DropletDetector::flow_numbers(1e-3, 1000.0, 0.072, 0.01, &ch);
        let d_h = ch.hydraulic_diameter_m();
        let expected_re = 1000.0 * 0.01 * d_h / 1e-3;
        assert!((fn_.reynolds - expected_re).abs() < 1e-10);
    }

    #[test]
    fn test_flow_numbers_we() {
        let ch = make_channel();
        let fn_ = DropletDetector::flow_numbers(1e-3, 1000.0, 0.072, 0.01, &ch);
        let d_h = ch.hydraulic_diameter_m();
        let expected_we = 1000.0 * 0.01 * 0.01 * d_h / 0.072;
        assert!((fn_.weber - expected_we).abs() < 1e-12);
    }

    // --- Impedance analysis ---

    #[test]
    fn test_impedance_dc() {
        // At DC (freq=0) impedance = R
        let res = DropletDetector::impedance_analysis(1000.0, 1e-12, 2000.0, 1e-12, 0.0, 100.0);
        assert!((res.baseline_impedance_ohm - 1000.0).abs() < 1e-6);
        assert!((res.droplet_impedance_ohm - 2000.0).abs() < 1e-6);
        assert!((res.delta_impedance_ohm - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_impedance_conductivity() {
        let res = DropletDetector::impedance_analysis(1000.0, 1e-12, 500.0, 1e-12, 0.0, 100.0);
        // sigma = k / R = 100 / 500 = 0.2 S/m
        assert!((res.conductivity_s_m - 0.2).abs() < 1e-12);
    }

    #[test]
    fn test_impedance_high_frequency() {
        // At very high freq, capacitance dominates → |Z| → 0
        let res = DropletDetector::impedance_analysis(1000.0, 1e-9, 1000.0, 1e-9, 1e9, 100.0);
        assert!(res.baseline_impedance_ohm < 1.0);
    }

    // --- Fluorescence ---

    #[test]
    fn test_fluorescence_sort() {
        let det = DropletDetector::new(1000.0, 0.5, 0.2);
        let event = DropletEvent {
            start_time_s: 0.01,
            end_time_s: 0.02,
            peak_amplitude: 1.0,
            area: 0.5,
            volume_pl: 10.0,
            velocity_ms: 0.0,
        };
        let fluo = vec![0.0; 100];
        // All zeros → integrated = 0, below any positive threshold
        let res = det.fluorescence_analysis(&fluo, &event, 0.001);
        assert_eq!(res.decision, SortDecision::Waste);
    }

    #[test]
    fn test_fluorescence_above_threshold() {
        let det = DropletDetector::new(1000.0, 0.5, 0.2);
        let event = DropletEvent {
            start_time_s: 0.01,
            end_time_s: 0.02,
            peak_amplitude: 1.0,
            area: 0.5,
            volume_pl: 10.0,
            velocity_ms: 0.0,
        };
        let mut fluo = vec![0.0; 100];
        for i in 10..20 {
            fluo[i] = 5.0;
        }
        let res = det.fluorescence_analysis(&fluo, &event, 0.01);
        assert_eq!(res.decision, SortDecision::Sort);
        assert!((res.peak_intensity - 5.0).abs() < 1e-12);
    }

    // --- Merge/split detection ---

    #[test]
    fn test_coalescence_detection() {
        let mut events: Vec<DropletEvent> = (0..20)
            .map(|i| DropletEvent {
                start_time_s: i as f64 * 0.01,
                end_time_s: i as f64 * 0.01 + 0.002,
                peak_amplitude: 1.0,
                area: 1.0,
                volume_pl: 100.0,
                velocity_ms: 0.0,
            })
            .collect();
        // Make event 10 a coalescence (double area)
        events[10].area = 5.0;
        let merges = DropletDetector::detect_merge_split(&events, 2.0);
        assert!(!merges.is_empty());
        assert!(merges.iter().any(|m| matches!(m, MergeEvent::Coalescence { merged_index: 10, .. })));
    }

    #[test]
    fn test_fission_detection() {
        let mut events: Vec<DropletEvent> = (0..20)
            .map(|i| DropletEvent {
                start_time_s: i as f64 * 0.01,
                end_time_s: i as f64 * 0.01 + 0.002,
                peak_amplitude: 1.0,
                area: 1.0,
                volume_pl: 100.0,
                velocity_ms: 0.0,
            })
            .collect();
        // Make event 5 a fission (very small area)
        events[5].area = 0.01;
        let merges = DropletDetector::detect_merge_split(&events, 2.0);
        assert!(merges.iter().any(|m| matches!(m, MergeEvent::Fission { parent_index: 5, .. })));
    }

    #[test]
    fn test_no_merge_split_uniform() {
        let events: Vec<DropletEvent> = (0..20)
            .map(|i| DropletEvent {
                start_time_s: i as f64 * 0.01,
                end_time_s: i as f64 * 0.01 + 0.002,
                peak_amplitude: 1.0,
                area: 1.0,
                volume_pl: 100.0,
                velocity_ms: 0.0,
            })
            .collect();
        let merges = DropletDetector::detect_merge_split(&events, 3.0);
        assert!(merges.is_empty());
    }

    // --- FFT helper ---

    #[test]
    fn test_fft_single_tone() {
        let n = 64;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4.0 * i as f64 / n as f64).sin())
            .collect();
        let (mag, _) = fft_magnitude(&signal);
        // Bin 4 should be the peak
        let peak_bin = mag[1..n / 2]
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0
            + 1;
        assert_eq!(peak_bin, 4);
    }

    #[test]
    fn test_fft_dc() {
        let signal = vec![1.0; 16];
        let (mag, _) = fft_magnitude(&signal);
        // DC bin should dominate
        assert!(mag[0] > mag[1] * 10.0);
    }

    // --- Complex impedance helper ---

    #[test]
    fn test_complex_impedance_pure_r() {
        let z = complex_impedance_mag(1000.0, 0.0, 1000.0);
        assert!((z - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn test_complex_impedance_decreases_with_freq() {
        let z_low = complex_impedance_mag(1000.0, 1e-9, 1e3);
        let z_high = complex_impedance_mag(1000.0, 1e-9, 1e9);
        assert!(z_high < z_low);
    }

    // --- Detection mode enum ---

    #[test]
    fn test_detection_modes() {
        let mut det = make_detector();
        det.detection_mode = DetectionMode::Impedance;
        assert_eq!(det.detection_mode, DetectionMode::Impedance);
        det.detection_mode = DetectionMode::Capacitive;
        assert_eq!(det.detection_mode, DetectionMode::Capacitive);
    }

    // --- Integration: detect + characterise ---

    #[test]
    fn test_full_pipeline() {
        let mut det = make_detector();
        det.min_droplet_width_s = 0.0;
        det.flow_rate_m3s = 1e-12;
        det.channel = Some(make_channel());

        let sig = synth_droplet_train(100_000.0, 15, 0.008, 0.0015, 1.2);
        let events = det.detect(&sig);
        assert!(events.len() >= 10);

        let sd = det.size_distribution(&events, 5);
        assert!(sd.count >= 10);
        assert!(sd.mean_volume_pl > 0.0);

        if let Some(ga) = det.generation_analysis(&events) {
            assert!(ga.rate_hz > 50.0);
        }
    }
}
