//! # Microfluidic Droplet Sorter
//!
//! Signal processing for microfluidic droplet detection, characterization, and
//! sorting in lab-on-a-chip systems. Covers the full pipeline from raw sensor
//! signal acquisition through droplet classification and actuator trigger timing.
//!
//! ## Overview
//!
//! Droplet-based microfluidics generates picoliter-to-nanoliter aqueous droplets
//! inside immiscible carrier oil. Downstream sensors (optical, impedance, or
//! fluorescence) produce transient signals as each droplet transits the detection
//! zone. This module processes those signals to:
//!
//! 1. Detect droplet arrival (hysteresis threshold on rising/falling edges)
//! 2. Characterize droplet size, velocity, and spacing
//! 3. Classify droplets based on fluorescence or impedance signatures
//! 4. Compute actuator trigger timing for sorting junctions
//! 5. Track throughput, purity, and recovery statistics
//!
//! ## Processing Pipeline
//!
//! ```text
//! Raw Sensor → Baseline Correction → Hysteresis Detection → Transit Timing
//!     → Size/Velocity → Fluorescence/Impedance → Sort Decision → Actuator Delay
//!     → Throughput Statistics
//! ```
//!
//! ## Dimensionless Numbers
//!
//! - **Capillary number** Ca = μU/γ — viscous vs. surface tension forces
//! - **Weber number** We = ρU²L/γ — inertial vs. surface tension forces
//! - **Reynolds number** Re = ρUL/μ — inertial vs. viscous forces
//! - **Dean flow number** De = Re·√(D_h/2R) — secondary flows in curved channels
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::microfluidic_droplet_sorter::*;
//!
//! let config = DropletConfig::new(100.0, 50.0, 1e-12, 1000.0);
//! let mut proc = DropletSorterProcessor::new(config, 100_000.0);
//!
//! // Generate a synthetic sensor signal with droplet pulses
//! let signal: Vec<f64> = (0..10000)
//!     .map(|i| {
//!         let t = i as f64 / 100_000.0;
//!         if (t * 1000.0) as u64 % 10 < 2 { 0.8 } else { 0.05 }
//!     })
//!     .collect();
//!
//! let droplets = proc.detect_droplets(&signal, 0.5, 0.2);
//! assert!(droplets.len() > 0);
//! ```

use std::f64::consts::PI;

// ============================================================
// Enums
// ============================================================

/// Detection method used in the microfluidic chip.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionMethod {
    /// Optical absorbance / transmission measurement.
    Optical,
    /// Electrical impedance between coplanar electrodes.
    Impedance,
    /// Laser-induced fluorescence (LIF).
    Fluorescence,
}

/// Sorting actuator type at the downstream junction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActuatorType {
    /// Dielectrophoretic (DEP) electrode.
    Dielectrophoretic,
    /// Pneumatic valve.
    Pneumatic,
    /// Piezoelectric surface acoustic wave (SAW).
    Piezoelectric,
    /// Optical force (laser-based).
    OpticalForce,
}

/// Sorting decision for a detected droplet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SortDecision {
    /// Droplet meets criteria — route to collection.
    Collect,
    /// Droplet does not meet criteria — route to waste.
    Waste,
    /// Insufficient data to decide.
    Undetermined,
}

// ============================================================
// Configuration
// ============================================================

/// Configuration for the microfluidic channel and droplet generation.
#[derive(Debug, Clone)]
pub struct DropletConfig {
    /// Channel width in micrometers.
    pub channel_width_um: f64,
    /// Channel height in micrometers.
    pub channel_height_um: f64,
    /// Continuous-phase flow rate in m^3/s.
    pub flow_rate_m3s: f64,
    /// Droplet generation frequency in Hz.
    pub generation_freq_hz: f64,
    /// Detection method.
    pub detection_method: DetectionMethod,
    /// Sorting actuator type.
    pub actuator_type: ActuatorType,
    /// Distance from detector to sorting junction in micrometers.
    pub detector_to_sort_um: f64,
    /// Spacing between two detectors (if dual-detector) in micrometers.
    pub detector_spacing_um: f64,
    /// Continuous-phase dynamic viscosity in Pa·s.
    pub viscosity_pa_s: f64,
    /// Continuous-phase density in kg/m^3.
    pub density_kg_m3: f64,
    /// Interfacial tension in N/m.
    pub surface_tension_n_m: f64,
    /// Radius of curvature for curved channels in micrometers (0 = straight).
    pub curve_radius_um: f64,
}

impl DropletConfig {
    /// Create a new configuration with essential parameters.
    ///
    /// Uses water-in-oil defaults: viscosity 1e-3 Pa·s, density 1000 kg/m^3,
    /// surface tension 5e-3 N/m (water-oil), DEP actuator.
    pub fn new(
        channel_width_um: f64,
        channel_height_um: f64,
        flow_rate_m3s: f64,
        generation_freq_hz: f64,
    ) -> Self {
        Self {
            channel_width_um,
            channel_height_um,
            flow_rate_m3s,
            generation_freq_hz,
            detection_method: DetectionMethod::Fluorescence,
            actuator_type: ActuatorType::Dielectrophoretic,
            detector_to_sort_um: 500.0,
            detector_spacing_um: 200.0,
            viscosity_pa_s: 1e-3,
            density_kg_m3: 1000.0,
            surface_tension_n_m: 5e-3,
            curve_radius_um: 0.0,
        }
    }

    /// Channel cross-sectional area in m^2.
    pub fn cross_section_m2(&self) -> f64 {
        self.channel_width_um * 1e-6 * self.channel_height_um * 1e-6
    }

    /// Hydraulic diameter D_h = 2wh/(w+h) in meters.
    pub fn hydraulic_diameter_m(&self) -> f64 {
        let w = self.channel_width_um;
        let h = self.channel_height_um;
        2.0 * w * h / (w + h) * 1e-6
    }

    /// Mean flow velocity U = Q / A in m/s.
    pub fn mean_velocity_ms(&self) -> f64 {
        let a = self.cross_section_m2();
        if a > 0.0 {
            self.flow_rate_m3s / a
        } else {
            0.0
        }
    }
}

// ============================================================
// Detected droplet event
// ============================================================

/// A single detected droplet event.
#[derive(Debug, Clone)]
pub struct DropletEvent {
    /// Start time of the droplet transit in seconds.
    pub start_time_s: f64,
    /// End time of the droplet transit in seconds.
    pub end_time_s: f64,
    /// Peak amplitude of the sensor signal during transit.
    pub peak_amplitude: f64,
    /// Integrated signal area (signal × time).
    pub area: f64,
}

impl DropletEvent {
    /// Transit duration in seconds.
    pub fn transit_time_s(&self) -> f64 {
        self.end_time_s - self.start_time_s
    }
}

// ============================================================
// Throughput statistics
// ============================================================

/// Throughput and sorting performance statistics.
#[derive(Debug, Clone)]
pub struct ThroughputStats {
    /// Total droplets processed.
    pub total_droplets: usize,
    /// Droplets sorted to collection.
    pub sorted_count: usize,
    /// Droplets routed to waste.
    pub waste_count: usize,
    /// Processing rate in droplets per second.
    pub droplets_per_second: f64,
    /// Sort purity: fraction of collected droplets that truly meet criteria.
    pub sort_purity: f64,
    /// Recovery rate: fraction of target droplets that were collected.
    pub recovery_rate: f64,
}

// ============================================================
// Encapsulation statistics
// ============================================================

/// Poisson encapsulation statistics for cell/bead loading.
#[derive(Debug, Clone)]
pub struct EncapsulationStats {
    /// Mean occupancy λ (average entities per droplet).
    pub lambda: f64,
    /// P(k=0): probability of empty droplet.
    pub p_empty: f64,
    /// P(k=1): probability of single occupancy.
    pub p_single: f64,
    /// P(k≥2): probability of multiple occupancy.
    pub p_multiple: f64,
}

// ============================================================
// Main processor
// ============================================================

/// Microfluidic droplet sorter processor.
///
/// Processes continuous sensor signals to detect, characterize, and sort
/// droplets in real time. Supports optical, impedance, and fluorescence
/// detection modalities.
#[derive(Debug, Clone)]
pub struct DropletSorterProcessor {
    /// Channel and flow configuration.
    pub config: DropletConfig,
    /// Sensor sample rate in Hz.
    pub sample_rate_hz: f64,
}

impl DropletSorterProcessor {
    /// Create a new droplet sorter processor.
    pub fn new(config: DropletConfig, sample_rate_hz: f64) -> Self {
        Self {
            config,
            sample_rate_hz,
        }
    }

    // ----------------------------------------------------------------
    // Detection
    // ----------------------------------------------------------------

    /// Detect droplets from a continuous sensor signal using hysteresis thresholding.
    ///
    /// Rising edge crosses `threshold_high` to enter a droplet; falling edge
    /// crosses `threshold_low` to exit. This prevents false triggers from noise
    /// near the threshold.
    ///
    /// Returns a vector of `DropletEvent` structs with timing and amplitude info.
    pub fn detect_droplets(
        &self,
        signal: &[f64],
        threshold_high: f64,
        threshold_low: f64,
    ) -> Vec<DropletEvent> {
        if signal.is_empty() {
            return Vec::new();
        }

        let dt = 1.0 / self.sample_rate_hz;
        let mut events = Vec::new();
        let mut in_droplet = false;
        let mut start_idx: usize = 0;
        let mut peak = f64::NEG_INFINITY;
        let mut area = 0.0;

        for (i, &s) in signal.iter().enumerate() {
            if !in_droplet {
                if s >= threshold_high {
                    in_droplet = true;
                    start_idx = i;
                    peak = s;
                    area = s * dt;
                }
            } else {
                if s > peak {
                    peak = s;
                }
                area += s * dt;
                if s < threshold_low {
                    in_droplet = false;
                    events.push(DropletEvent {
                        start_time_s: start_idx as f64 * dt,
                        end_time_s: i as f64 * dt,
                        peak_amplitude: peak,
                        area,
                    });
                    peak = f64::NEG_INFINITY;
                    area = 0.0;
                }
            }
        }

        // Close any open event at end of signal
        if in_droplet {
            let end_idx = signal.len() - 1;
            events.push(DropletEvent {
                start_time_s: start_idx as f64 * dt,
                end_time_s: end_idx as f64 * dt,
                peak_amplitude: peak,
                area,
            });
        }

        events
    }

    // ----------------------------------------------------------------
    // Size, velocity, spacing
    // ----------------------------------------------------------------

    /// Calculate droplet volume from transit time and flow velocity.
    ///
    /// Uses plug-flow approximation: V = A_channel × U × t_transit.
    /// Returns volume in picoliters (pL).
    pub fn droplet_size_from_transit_time(&self, transit_time_s: f64) -> f64 {
        let a = self.config.cross_section_m2();
        let u = self.config.mean_velocity_ms();
        let volume_m3 = a * u * transit_time_s;
        volume_m3 * 1e15 // convert m^3 to pL
    }

    /// Calculate droplet velocity from dual-detector time-of-flight.
    ///
    /// v = detector_spacing / dt. Returns velocity in m/s.
    pub fn droplet_velocity(&self, tof_s: f64) -> f64 {
        if tof_s <= 0.0 {
            return 0.0;
        }
        self.config.detector_spacing_um * 1e-6 / tof_s
    }

    /// Calculate inter-droplet spacing from consecutive event times.
    ///
    /// Returns a vector of spacings in micrometers, computed as
    /// velocity × time_gap for each consecutive pair.
    pub fn droplet_spacing(&self, events: &[DropletEvent]) -> Vec<f64> {
        if events.len() < 2 {
            return Vec::new();
        }
        let u = self.config.mean_velocity_ms();
        let mut spacings = Vec::with_capacity(events.len() - 1);
        for i in 1..events.len() {
            let gap_s = events[i].start_time_s - events[i - 1].end_time_s;
            let spacing_m = u * gap_s;
            spacings.push(spacing_m * 1e6); // convert to micrometers
        }
        spacings
    }

    // ----------------------------------------------------------------
    // Generation frequency models
    // ----------------------------------------------------------------

    /// Estimate droplet generation frequency from flow rates.
    ///
    /// T-junction model: f = Q_d / V_drop, where V_drop is estimated from
    /// channel geometry. For flow-focusing, the same scaling applies but with
    /// a geometric correction factor.
    ///
    /// - `dispersed_flow_rate_m3s`: dispersed phase flow rate
    /// - `t_junction`: true for T-junction, false for flow-focusing
    ///
    /// Returns frequency in Hz.
    pub fn generation_frequency(
        &self,
        dispersed_flow_rate_m3s: f64,
        t_junction: bool,
    ) -> f64 {
        let w = self.config.channel_width_um * 1e-6;
        let h = self.config.channel_height_um * 1e-6;

        // Estimated droplet volume: sphere fitting in channel (slug model)
        let d_min = w.min(h);
        let v_drop = if t_junction {
            // T-junction: slug volume ~ w × h × w (one channel width long)
            w * h * w
        } else {
            // Flow-focusing: smaller droplets, ~sphere of diameter = min(w,h)
            (PI / 6.0) * d_min * d_min * d_min
        };

        if v_drop > 0.0 {
            dispersed_flow_rate_m3s / v_drop
        } else {
            0.0
        }
    }

    // ----------------------------------------------------------------
    // Dimensionless numbers
    // ----------------------------------------------------------------

    /// Capillary number: Ca = μU/γ.
    ///
    /// Ratio of viscous forces to surface tension. Governs droplet formation
    /// regime (dripping Ca < 0.1, jetting Ca > 0.1).
    pub fn capillary_number(&self) -> f64 {
        let u = self.config.mean_velocity_ms();
        self.config.viscosity_pa_s * u / self.config.surface_tension_n_m
    }

    /// Weber number: We = ρU²L/γ.
    ///
    /// Ratio of inertial to surface tension forces. Uses hydraulic diameter
    /// as the characteristic length.
    pub fn weber_number(&self) -> f64 {
        let u = self.config.mean_velocity_ms();
        let d_h = self.config.hydraulic_diameter_m();
        self.config.density_kg_m3 * u * u * d_h / self.config.surface_tension_n_m
    }

    /// Reynolds number: Re = ρUL/μ.
    ///
    /// Ratio of inertial to viscous forces. Microfluidic flows are typically
    /// Re << 1 (Stokes flow) to Re ~ 100.
    pub fn reynolds_number(&self) -> f64 {
        let u = self.config.mean_velocity_ms();
        let d_h = self.config.hydraulic_diameter_m();
        self.config.density_kg_m3 * u * d_h / self.config.viscosity_pa_s
    }

    /// Dean flow number: De = Re × √(D_h / 2R).
    ///
    /// Quantifies secondary flow effects in curved/serpentine channels.
    /// Returns 0 if the channel is straight (curve_radius = 0).
    pub fn dean_flow_number(&self) -> f64 {
        if self.config.curve_radius_um <= 0.0 {
            return 0.0;
        }
        let re = self.reynolds_number();
        let d_h = self.config.hydraulic_diameter_m();
        let r = self.config.curve_radius_um * 1e-6;
        re * (d_h / (2.0 * r)).sqrt()
    }

    // ----------------------------------------------------------------
    // Fluorescence
    // ----------------------------------------------------------------

    /// Integrate fluorescence signal over a droplet transit window.
    ///
    /// Returns the total integrated fluorescence intensity (signal × time)
    /// for the given droplet event.
    pub fn fluorescence_intensity(
        &self,
        signal: &[f64],
        event: &DropletEvent,
    ) -> f64 {
        let dt = 1.0 / self.sample_rate_hz;
        let i_start = (event.start_time_s * self.sample_rate_hz) as usize;
        let i_end = ((event.end_time_s * self.sample_rate_hz) as usize).min(signal.len());

        let mut integrated = 0.0;
        for i in i_start..i_end {
            integrated += signal[i] * dt;
        }
        integrated
    }

    // ----------------------------------------------------------------
    // Sort decision
    // ----------------------------------------------------------------

    /// Classify a droplet based on a measured value against a threshold.
    ///
    /// For fluorescence sorting, `measured_value` is the integrated intensity.
    /// For impedance sorting, it is the impedance delta. The `latency_s`
    /// parameter accounts for processing delay — if the droplet has already
    /// passed the sorting junction by the time the decision is made, the
    /// result is `Undetermined`.
    pub fn sort_decision(
        &self,
        measured_value: f64,
        threshold: f64,
        event: &DropletEvent,
        current_time_s: f64,
        latency_s: f64,
    ) -> SortDecision {
        // Check if the sorting window has expired
        let sort_delay = self.sorting_delay();
        let sort_time = event.start_time_s + sort_delay;
        let decision_ready_time = current_time_s + latency_s;

        if decision_ready_time > sort_time + event.transit_time_s() {
            return SortDecision::Undetermined;
        }

        if measured_value >= threshold {
            SortDecision::Collect
        } else {
            SortDecision::Waste
        }
    }

    // ----------------------------------------------------------------
    // Sorting delay
    // ----------------------------------------------------------------

    /// Calculate actuator trigger delay from detection to sorting junction.
    ///
    /// delay = distance_to_sort / flow_velocity. Returns delay in seconds.
    pub fn sorting_delay(&self) -> f64 {
        let u = self.config.mean_velocity_ms();
        if u <= 0.0 {
            return 0.0;
        }
        let d = self.config.detector_to_sort_um * 1e-6;
        d / u
    }

    // ----------------------------------------------------------------
    // Impedance model
    // ----------------------------------------------------------------

    /// Model electrical impedance change as a droplet passes between electrodes.
    ///
    /// Uses Maxwell mixture theory: the effective permittivity of a dilute
    /// suspension of spheres in a medium is
    ///
    ///   ε_eff = ε_m × (1 + 3φf_CM)
    ///
    /// where f_CM = (ε_p - ε_m) / (ε_p + 2ε_m) is the Clausius-Mossotti
    /// factor and φ is the volume fraction.
    ///
    /// Returns the impedance change ratio ΔZ/Z_baseline.
    pub fn impedance_signal(
        &self,
        permittivity_medium: f64,
        permittivity_droplet: f64,
        volume_fraction: f64,
    ) -> f64 {
        if permittivity_medium <= 0.0 {
            return 0.0;
        }
        // Clausius-Mossotti factor
        let f_cm = (permittivity_droplet - permittivity_medium)
            / (permittivity_droplet + 2.0 * permittivity_medium);
        // Effective permittivity change
        let delta_eps_ratio = 3.0 * volume_fraction * f_cm;
        // For a capacitive sensor, impedance ~ 1/ε, so ΔZ/Z ≈ -Δε/ε
        -delta_eps_ratio
    }

    // ----------------------------------------------------------------
    // Encapsulation statistics
    // ----------------------------------------------------------------

    /// Poisson distribution model for cell/bead encapsulation.
    ///
    /// Given the mean occupancy λ (average number of entities per droplet),
    /// computes the probability of empty, single, and multiple occupancy.
    ///
    /// P(k) = λ^k × e^(-λ) / k!
    pub fn encapsulation_statistics(&self, lambda: f64) -> EncapsulationStats {
        let exp_neg_lambda = (-lambda).exp();
        let p_empty = exp_neg_lambda;                        // P(k=0)
        let p_single = lambda * exp_neg_lambda;              // P(k=1)
        let p_multiple = 1.0 - p_empty - p_single;          // P(k>=2)

        EncapsulationStats {
            lambda,
            p_empty,
            p_single,
            p_multiple,
        }
    }

    // ----------------------------------------------------------------
    // Pressure drop
    // ----------------------------------------------------------------

    /// Hagen-Poiseuille pressure drop for a rectangular microchannel.
    ///
    /// For a rectangular cross-section with width w and height h (w > h),
    /// the pressure drop per unit length is approximately:
    ///
    ///   ΔP/L = 12μQ / (wh³) × (1 - 0.630 h/w)^(-1)
    ///
    /// `channel_length_um` is the channel length in micrometers.
    /// Returns pressure drop in Pascals.
    pub fn pressure_drop(&self, channel_length_um: f64) -> f64 {
        let w = self.config.channel_width_um * 1e-6;
        let h = self.config.channel_height_um * 1e-6;
        let l = channel_length_um * 1e-6;
        let mu = self.config.viscosity_pa_s;
        let q = self.config.flow_rate_m3s;

        // Ensure w >= h for the approximation
        let (ww, hh) = if w >= h { (w, h) } else { (h, w) };

        if ww <= 0.0 || hh <= 0.0 {
            return 0.0;
        }

        let correction = 1.0 - 0.630 * hh / ww;
        if correction <= 0.0 {
            return 0.0;
        }

        12.0 * mu * q * l / (ww * hh.powi(3) * correction)
    }

    // ----------------------------------------------------------------
    // Mixing time
    // ----------------------------------------------------------------

    /// Estimate mixing time inside a droplet.
    ///
    /// For chaotic advection in serpentine channels:
    ///   t_mix ≈ (w²/D) × (1/Pe^(2/3))
    ///
    /// For diffusion-limited mixing in straight channels:
    ///   t_mix ≈ w² / D
    ///
    /// - `diffusion_coeff_m2s`: molecular diffusion coefficient (e.g. 1e-9 for small molecules)
    /// - `chaotic`: true for serpentine/grooved channels, false for straight
    ///
    /// Returns mixing time in seconds.
    pub fn mixing_time(&self, diffusion_coeff_m2s: f64, chaotic: bool) -> f64 {
        let w = self.config.channel_width_um * 1e-6;
        if diffusion_coeff_m2s <= 0.0 || w <= 0.0 {
            return 0.0;
        }

        let t_diff = w * w / diffusion_coeff_m2s;

        if chaotic {
            let u = self.config.mean_velocity_ms();
            let pe = u * w / diffusion_coeff_m2s; // Peclet number
            if pe > 0.0 {
                t_diff / pe.powf(2.0 / 3.0)
            } else {
                t_diff
            }
        } else {
            t_diff
        }
    }

    // ----------------------------------------------------------------
    // Throughput analysis
    // ----------------------------------------------------------------

    /// Compute throughput and sorting performance statistics.
    ///
    /// - `decisions`: sort decisions for each droplet
    /// - `true_positives`: count of correctly sorted droplets (for purity)
    /// - `total_targets`: total number of target droplets in the population
    /// - `elapsed_time_s`: total elapsed time of the experiment
    pub fn throughput_analysis(
        &self,
        decisions: &[SortDecision],
        true_positives: usize,
        total_targets: usize,
        elapsed_time_s: f64,
    ) -> ThroughputStats {
        let total = decisions.len();
        let sorted = decisions.iter().filter(|&&d| d == SortDecision::Collect).count();
        let waste = decisions.iter().filter(|&&d| d == SortDecision::Waste).count();

        let rate = if elapsed_time_s > 0.0 {
            total as f64 / elapsed_time_s
        } else {
            0.0
        };

        let purity = if sorted > 0 {
            true_positives as f64 / sorted as f64
        } else {
            0.0
        };

        let recovery = if total_targets > 0 {
            true_positives as f64 / total_targets as f64
        } else {
            0.0
        };

        ThroughputStats {
            total_droplets: total,
            sorted_count: sorted,
            waste_count: waste,
            droplets_per_second: rate,
            sort_purity: purity,
            recovery_rate: recovery,
        }
    }

    // ----------------------------------------------------------------
    // Signal baseline correction
    // ----------------------------------------------------------------

    /// Running median baseline subtraction for sensor signal correction.
    ///
    /// Computes a running median over a window of `window_size` samples
    /// and subtracts it from the signal to remove slow drift and offset.
    ///
    /// Falls back to polynomial (linear) baseline if `polynomial_order` is
    /// Some(1) or higher (currently supports order 0 and 1).
    pub fn signal_baseline_correction(
        &self,
        signal: &[f64],
        window_size: usize,
        polynomial_order: Option<usize>,
    ) -> Vec<f64> {
        if signal.is_empty() {
            return Vec::new();
        }

        match polynomial_order {
            Some(order) if order <= 1 => {
                // Polynomial baseline subtraction
                self.polynomial_baseline(signal, order)
            }
            _ => {
                // Running median baseline
                self.running_median_baseline(signal, window_size)
            }
        }
    }

    /// Running median baseline estimation.
    fn running_median_baseline(&self, signal: &[f64], window_size: usize) -> Vec<f64> {
        let n = signal.len();
        let ws = window_size.max(1).min(n);
        let half = ws / 2;
        let mut corrected = Vec::with_capacity(n);
        let mut window_buf = Vec::with_capacity(ws);

        for i in 0..n {
            window_buf.clear();
            let start = if i >= half { i - half } else { 0 };
            let end = (i + half + 1).min(n);
            for j in start..end {
                window_buf.push(signal[j]);
            }
            // Sort for median
            window_buf.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = if window_buf.len() % 2 == 0 {
                let mid = window_buf.len() / 2;
                (window_buf[mid - 1] + window_buf[mid]) / 2.0
            } else {
                window_buf[window_buf.len() / 2]
            };
            corrected.push(signal[i] - median);
        }

        corrected
    }

    /// Polynomial baseline subtraction (order 0 = constant, order 1 = linear).
    fn polynomial_baseline(&self, signal: &[f64], order: usize) -> Vec<f64> {
        let n = signal.len();
        if n == 0 {
            return Vec::new();
        }

        if order == 0 {
            // Constant baseline = mean
            let mean = signal.iter().sum::<f64>() / n as f64;
            signal.iter().map(|&s| s - mean).collect()
        } else {
            // Linear baseline: y = a + b*x via least squares
            let n_f = n as f64;
            let sum_x: f64 = (0..n).map(|i| i as f64).sum();
            let sum_y: f64 = signal.iter().sum();
            let sum_xy: f64 = signal.iter().enumerate().map(|(i, &y)| i as f64 * y).sum();
            let sum_x2: f64 = (0..n).map(|i| (i as f64) * (i as f64)).sum();

            let denom = n_f * sum_x2 - sum_x * sum_x;
            if denom.abs() < 1e-30 {
                return signal.to_vec();
            }

            let b = (n_f * sum_xy - sum_x * sum_y) / denom;
            let a = (sum_y - b * sum_x) / n_f;

            signal
                .iter()
                .enumerate()
                .map(|(i, &s)| s - (a + b * i as f64))
                .collect()
        }
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_config() -> DropletConfig {
        DropletConfig::new(100.0, 50.0, 1e-12, 1000.0)
    }

    fn make_processor() -> DropletSorterProcessor {
        DropletSorterProcessor::new(make_config(), 100_000.0)
    }

    /// Synthesize a droplet signal with `n` Gaussian-like pulses at period `period_s`.
    fn synth_signal(
        sample_rate: f64,
        n: usize,
        period_s: f64,
        pulse_width_s: f64,
        amplitude: f64,
    ) -> Vec<f64> {
        let total = ((period_s * n as f64 + pulse_width_s * 2.0) * sample_rate) as usize + 1;
        let mut sig = vec![0.0f64; total];
        for d in 0..n {
            let centre = (d as f64 + 0.5) * period_s;
            let sigma = pulse_width_s / 4.0;
            let i_start = ((centre - pulse_width_s) * sample_rate).max(0.0) as usize;
            let i_end = ((centre + pulse_width_s) * sample_rate) as usize;
            let i_end = i_end.min(total);
            for i in i_start..i_end {
                let t = i as f64 / sample_rate - centre;
                sig[i] += amplitude * (-0.5 * (t / sigma).powi(2)).exp();
            }
        }
        sig
    }

    // --- DropletConfig tests ---

    #[test]
    fn test_config_cross_section() {
        let cfg = make_config();
        // 100 um × 50 um = 5000 um² = 5e-9 m²
        assert!((cfg.cross_section_m2() - 5e-9).abs() < 1e-15);
    }

    #[test]
    fn test_config_hydraulic_diameter() {
        let cfg = make_config();
        // D_h = 2×100×50/(100+50) × 1e-6 = 66.667 um = 6.6667e-5 m
        let expected = 2.0 * 100.0 * 50.0 / 150.0 * 1e-6;
        assert!((cfg.hydraulic_diameter_m() - expected).abs() < 1e-12);
    }

    #[test]
    fn test_config_square_channel() {
        let cfg = DropletConfig::new(100.0, 100.0, 1e-12, 1000.0);
        assert!((cfg.hydraulic_diameter_m() - 100e-6).abs() < 1e-12);
    }

    #[test]
    fn test_config_mean_velocity() {
        let cfg = make_config();
        // U = Q/A = 1e-12 / 5e-9 = 2e-4 m/s
        let expected = 1e-12 / 5e-9;
        assert!((cfg.mean_velocity_ms() - expected).abs() < 1e-12);
    }

    // --- Detection tests ---

    #[test]
    fn test_detect_empty_signal() {
        let proc = make_processor();
        let events = proc.detect_droplets(&[], 0.5, 0.2);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_no_droplets_below_threshold() {
        let proc = make_processor();
        let sig = vec![0.1; 10000];
        let events = proc.detect_droplets(&sig, 0.5, 0.2);
        assert!(events.is_empty());
    }

    #[test]
    fn test_detect_single_droplet() {
        let proc = make_processor();
        let sig = synth_signal(100_000.0, 1, 0.005, 0.001, 1.0);
        let events = proc.detect_droplets(&sig, 0.5, 0.2);
        assert_eq!(events.len(), 1);
        assert!(events[0].peak_amplitude > 0.8);
    }

    #[test]
    fn test_detect_multiple_droplets() {
        let proc = make_processor();
        let sig = synth_signal(100_000.0, 5, 0.005, 0.001, 1.0);
        let events = proc.detect_droplets(&sig, 0.5, 0.2);
        assert_eq!(events.len(), 5);
    }

    #[test]
    fn test_hysteresis_prevents_chatter() {
        let proc = make_processor();
        let mut sig = vec![0.0; 5000];
        // One big pulse
        for i in 400..600 {
            sig[i] = 0.8;
        }
        // Small bump that doesn't reach high threshold
        for i in 200..220 {
            sig[i] = 0.4;
        }
        let events = proc.detect_droplets(&sig, 0.6, 0.3);
        assert_eq!(events.len(), 1);
    }

    #[test]
    fn test_droplet_event_transit_time() {
        let event = DropletEvent {
            start_time_s: 0.001,
            end_time_s: 0.003,
            peak_amplitude: 1.0,
            area: 0.5,
        };
        assert!((event.transit_time_s() - 0.002).abs() < 1e-12);
    }

    // --- Size, velocity, spacing ---

    #[test]
    fn test_size_from_transit_time() {
        let proc = make_processor();
        let u = proc.config.mean_velocity_ms(); // 2e-4 m/s
        let a = proc.config.cross_section_m2(); // 5e-9 m^2
        let transit = 0.001; // 1 ms
        let vol = proc.droplet_size_from_transit_time(transit);
        let expected = a * u * transit * 1e15;
        assert!((vol - expected).abs() < 1e-6);
    }

    #[test]
    fn test_droplet_velocity() {
        let proc = make_processor();
        // 200 um spacing, 0.001 s TOF → v = 200e-6 / 1e-3 = 0.2 m/s
        let v = proc.droplet_velocity(0.001);
        assert!((v - 0.2).abs() < 1e-9);
    }

    #[test]
    fn test_droplet_velocity_zero_tof() {
        let proc = make_processor();
        assert_eq!(proc.droplet_velocity(0.0), 0.0);
    }

    #[test]
    fn test_droplet_spacing() {
        let proc = make_processor();
        let events = vec![
            DropletEvent { start_time_s: 0.001, end_time_s: 0.002, peak_amplitude: 1.0, area: 0.5 },
            DropletEvent { start_time_s: 0.004, end_time_s: 0.005, peak_amplitude: 1.0, area: 0.5 },
            DropletEvent { start_time_s: 0.007, end_time_s: 0.008, peak_amplitude: 1.0, area: 0.5 },
        ];
        let spacings = proc.droplet_spacing(&events);
        assert_eq!(spacings.len(), 2);
        // gap = 0.002 s, velocity = 2e-4 m/s → spacing = 4e-7 m = 0.4 um
        let expected = proc.config.mean_velocity_ms() * 0.002 * 1e6;
        assert!((spacings[0] - expected).abs() < 1e-6);
    }

    #[test]
    fn test_droplet_spacing_single() {
        let proc = make_processor();
        let events = vec![
            DropletEvent { start_time_s: 0.001, end_time_s: 0.002, peak_amplitude: 1.0, area: 0.5 },
        ];
        assert!(proc.droplet_spacing(&events).is_empty());
    }

    // --- Generation frequency ---

    #[test]
    fn test_generation_frequency_t_junction() {
        let proc = make_processor();
        let q_d = 1e-13; // dispersed flow rate
        let freq = proc.generation_frequency(q_d, true);
        // V_drop = w*h*w = (100e-6)^2 * 50e-6 = 5e-16 m^3
        let expected = q_d / (100e-6 * 50e-6 * 100e-6);
        assert!((freq - expected).abs() < 1e-3);
    }

    #[test]
    fn test_generation_frequency_flow_focusing() {
        let proc = make_processor();
        let q_d = 1e-13;
        let freq = proc.generation_frequency(q_d, false);
        // V_drop = (pi/6)*d_min^3, d_min = 50e-6 m
        let v_drop = (PI / 6.0) * (50e-6_f64).powi(3);
        let expected = q_d / v_drop;
        assert!((freq - expected).abs() / expected < 1e-6);
    }

    // --- Dimensionless numbers ---

    #[test]
    fn test_capillary_number() {
        let proc = make_processor();
        let u = proc.config.mean_velocity_ms();
        let expected = proc.config.viscosity_pa_s * u / proc.config.surface_tension_n_m;
        assert!((proc.capillary_number() - expected).abs() < 1e-15);
    }

    #[test]
    fn test_weber_number() {
        let proc = make_processor();
        let u = proc.config.mean_velocity_ms();
        let d_h = proc.config.hydraulic_diameter_m();
        let expected = proc.config.density_kg_m3 * u * u * d_h / proc.config.surface_tension_n_m;
        assert!((proc.weber_number() - expected).abs() < 1e-20);
    }

    #[test]
    fn test_reynolds_number() {
        let proc = make_processor();
        let u = proc.config.mean_velocity_ms();
        let d_h = proc.config.hydraulic_diameter_m();
        let expected = proc.config.density_kg_m3 * u * d_h / proc.config.viscosity_pa_s;
        assert!((proc.reynolds_number() - expected).abs() < 1e-12);
    }

    #[test]
    fn test_reynolds_low_for_microfluidics() {
        let proc = make_processor();
        // Typical microfluidic Re << 1
        assert!(proc.reynolds_number() < 1.0);
    }

    #[test]
    fn test_dean_flow_straight_channel() {
        let proc = make_processor();
        // curve_radius_um = 0 → Dean number = 0
        assert_eq!(proc.dean_flow_number(), 0.0);
    }

    #[test]
    fn test_dean_flow_curved_channel() {
        let mut config = make_config();
        config.curve_radius_um = 500.0; // 500 um radius
        let proc = DropletSorterProcessor::new(config, 100_000.0);
        let de = proc.dean_flow_number();
        let re = proc.reynolds_number();
        let d_h = proc.config.hydraulic_diameter_m();
        let r = 500.0 * 1e-6;
        let expected = re * (d_h / (2.0 * r)).sqrt();
        assert!((de - expected).abs() < 1e-15);
        assert!(de > 0.0);
    }

    // --- Fluorescence ---

    #[test]
    fn test_fluorescence_intensity_zero() {
        let proc = make_processor();
        let signal = vec![0.0; 1000];
        let event = DropletEvent {
            start_time_s: 0.001,
            end_time_s: 0.005,
            peak_amplitude: 1.0,
            area: 0.5,
        };
        let intensity = proc.fluorescence_intensity(&signal, &event);
        assert!(intensity.abs() < 1e-12);
    }

    #[test]
    fn test_fluorescence_intensity_nonzero() {
        let proc = make_processor();
        let mut signal = vec![0.0; 1000];
        // Set samples in the event window to 1.0
        for i in 100..500 {
            signal[i] = 1.0;
        }
        let event = DropletEvent {
            start_time_s: 0.001,  // sample 100
            end_time_s: 0.005,    // sample 500
            peak_amplitude: 1.0,
            area: 0.5,
        };
        let intensity = proc.fluorescence_intensity(&signal, &event);
        // 400 samples × 1.0 × dt(1e-5) = 4e-3
        assert!((intensity - 0.004).abs() < 1e-6);
    }

    // --- Sort decision ---

    #[test]
    fn test_sort_decision_collect() {
        let proc = make_processor();
        let event = DropletEvent {
            start_time_s: 0.0,
            end_time_s: 0.001,
            peak_amplitude: 1.0,
            area: 0.5,
        };
        let decision = proc.sort_decision(1.0, 0.5, &event, 0.0, 0.0);
        assert_eq!(decision, SortDecision::Collect);
    }

    #[test]
    fn test_sort_decision_waste() {
        let proc = make_processor();
        let event = DropletEvent {
            start_time_s: 0.0,
            end_time_s: 0.001,
            peak_amplitude: 1.0,
            area: 0.5,
        };
        let decision = proc.sort_decision(0.1, 0.5, &event, 0.0, 0.0);
        assert_eq!(decision, SortDecision::Waste);
    }

    #[test]
    fn test_sort_decision_undetermined_late() {
        let proc = make_processor();
        let event = DropletEvent {
            start_time_s: 0.0,
            end_time_s: 0.001,
            peak_amplitude: 1.0,
            area: 0.5,
        };
        // Very late decision
        let decision = proc.sort_decision(1.0, 0.5, &event, 100.0, 0.0);
        assert_eq!(decision, SortDecision::Undetermined);
    }

    // --- Sorting delay ---

    #[test]
    fn test_sorting_delay() {
        let proc = make_processor();
        let u = proc.config.mean_velocity_ms();
        let d = proc.config.detector_to_sort_um * 1e-6;
        let expected = d / u;
        assert!((proc.sorting_delay() - expected).abs() < 1e-9);
    }

    // --- Impedance signal ---

    #[test]
    fn test_impedance_same_permittivity() {
        let proc = make_processor();
        // Same permittivity → no change
        let dz = proc.impedance_signal(80.0, 80.0, 0.1);
        assert!(dz.abs() < 1e-12);
    }

    #[test]
    fn test_impedance_higher_permittivity_droplet() {
        let proc = make_processor();
        // Droplet with higher permittivity → ΔZ/Z < 0 (impedance decreases)
        let dz = proc.impedance_signal(2.0, 80.0, 0.1);
        assert!(dz < 0.0); // Negative means impedance decreases
    }

    #[test]
    fn test_impedance_lower_permittivity_droplet() {
        let proc = make_processor();
        // Oil droplet in water → positive change
        let dz = proc.impedance_signal(80.0, 2.0, 0.1);
        assert!(dz > 0.0);
    }

    // --- Encapsulation statistics ---

    #[test]
    fn test_encapsulation_lambda_zero() {
        let proc = make_processor();
        let stats = proc.encapsulation_statistics(0.0);
        assert!((stats.p_empty - 1.0).abs() < 1e-12);
        assert!(stats.p_single.abs() < 1e-12);
        assert!(stats.p_multiple.abs() < 1e-12);
    }

    #[test]
    fn test_encapsulation_lambda_one() {
        let proc = make_processor();
        let stats = proc.encapsulation_statistics(1.0);
        let e_inv = (-1.0_f64).exp();
        assert!((stats.p_empty - e_inv).abs() < 1e-12);
        assert!((stats.p_single - e_inv).abs() < 1e-12);
        let expected_multi = 1.0 - 2.0 * e_inv;
        assert!((stats.p_multiple - expected_multi).abs() < 1e-12);
    }

    #[test]
    fn test_encapsulation_probabilities_sum_to_one() {
        let proc = make_processor();
        for lambda in &[0.1, 0.5, 1.0, 2.0, 5.0, 10.0] {
            let stats = proc.encapsulation_statistics(*lambda);
            let sum = stats.p_empty + stats.p_single + stats.p_multiple;
            assert!((sum - 1.0).abs() < 1e-12, "Sum = {} for lambda = {}", sum, lambda);
        }
    }

    // --- Pressure drop ---

    #[test]
    fn test_pressure_drop_positive() {
        let proc = make_processor();
        let dp = proc.pressure_drop(10_000.0); // 10 mm channel
        assert!(dp > 0.0);
    }

    #[test]
    fn test_pressure_drop_scales_with_length() {
        let proc = make_processor();
        let dp1 = proc.pressure_drop(10_000.0);
        let dp2 = proc.pressure_drop(20_000.0);
        assert!((dp2 / dp1 - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_pressure_drop_zero_length() {
        let proc = make_processor();
        let dp = proc.pressure_drop(0.0);
        assert!(dp.abs() < 1e-30);
    }

    // --- Mixing time ---

    #[test]
    fn test_mixing_time_diffusion() {
        let proc = make_processor();
        let d = 1e-9; // 1e-9 m^2/s (small molecule in water)
        let t = proc.mixing_time(d, false);
        // t = w^2 / D = (100e-6)^2 / 1e-9 = 10 s
        let expected = (100e-6_f64).powi(2) / d;
        assert!((t - expected).abs() < 1e-6);
    }

    #[test]
    fn test_mixing_time_chaotic_faster() {
        let proc = make_processor();
        let d = 1e-9;
        let t_diff = proc.mixing_time(d, false);
        let t_chaotic = proc.mixing_time(d, true);
        // Chaotic mixing should be faster (smaller time) if Pe > 1
        // For very low velocity Pe might be < 1, so just check it's finite
        assert!(t_chaotic > 0.0);
        assert!(t_chaotic.is_finite());
        // With Pe > 1, chaotic should be faster
        let u = proc.config.mean_velocity_ms();
        let w = proc.config.channel_width_um * 1e-6;
        let pe = u * w / d;
        if pe > 1.0 {
            assert!(t_chaotic < t_diff);
        }
    }

    #[test]
    fn test_mixing_time_zero_diffusion() {
        let proc = make_processor();
        assert_eq!(proc.mixing_time(0.0, false), 0.0);
    }

    // --- Throughput analysis ---

    #[test]
    fn test_throughput_basic() {
        let proc = make_processor();
        let decisions = vec![
            SortDecision::Collect,
            SortDecision::Waste,
            SortDecision::Collect,
            SortDecision::Waste,
            SortDecision::Collect,
        ];
        let stats = proc.throughput_analysis(&decisions, 3, 4, 1.0);
        assert_eq!(stats.total_droplets, 5);
        assert_eq!(stats.sorted_count, 3);
        assert_eq!(stats.waste_count, 2);
        assert!((stats.droplets_per_second - 5.0).abs() < 1e-12);
        assert!((stats.sort_purity - 1.0).abs() < 1e-12);
        assert!((stats.recovery_rate - 0.75).abs() < 1e-12);
    }

    #[test]
    fn test_throughput_no_sort() {
        let proc = make_processor();
        let decisions = vec![SortDecision::Waste; 10];
        let stats = proc.throughput_analysis(&decisions, 0, 5, 2.0);
        assert_eq!(stats.sorted_count, 0);
        assert_eq!(stats.sort_purity, 0.0);
        assert_eq!(stats.recovery_rate, 0.0);
    }

    #[test]
    fn test_throughput_empty() {
        let proc = make_processor();
        let stats = proc.throughput_analysis(&[], 0, 0, 1.0);
        assert_eq!(stats.total_droplets, 0);
        assert_eq!(stats.droplets_per_second, 0.0);
    }

    // --- Baseline correction ---

    #[test]
    fn test_baseline_median_removes_dc() {
        let proc = make_processor();
        let signal: Vec<f64> = vec![5.0; 100];
        let corrected = proc.signal_baseline_correction(&signal, 11, None);
        // All values should be ~0 after removing constant baseline
        for &v in &corrected {
            assert!(v.abs() < 1e-12);
        }
    }

    #[test]
    fn test_baseline_polynomial_order_zero() {
        let proc = make_processor();
        let signal: Vec<f64> = (0..100).map(|i| 3.0 + 0.01 * (i as f64)).collect();
        let corrected = proc.signal_baseline_correction(&signal, 11, Some(0));
        // After removing mean, sum should be ~0
        let sum: f64 = corrected.iter().sum();
        assert!(sum.abs() < 1e-6);
    }

    #[test]
    fn test_baseline_polynomial_order_one() {
        let proc = make_processor();
        // Linear signal
        let signal: Vec<f64> = (0..100).map(|i| 2.0 + 0.05 * i as f64).collect();
        let corrected = proc.signal_baseline_correction(&signal, 11, Some(1));
        // After removing linear trend, all values should be near 0
        for &v in &corrected {
            assert!(v.abs() < 1e-6, "Residual {} too large", v);
        }
    }

    #[test]
    fn test_baseline_empty_signal() {
        let proc = make_processor();
        let corrected = proc.signal_baseline_correction(&[], 11, None);
        assert!(corrected.is_empty());
    }

    // --- Integration tests ---

    #[test]
    fn test_full_pipeline() {
        let mut config = make_config();
        config.flow_rate_m3s = 1e-11;
        let proc = DropletSorterProcessor::new(config, 100_000.0);

        // Generate signal with 10 droplets
        let sig = synth_signal(100_000.0, 10, 0.008, 0.0015, 1.2);
        let events = proc.detect_droplets(&sig, 0.5, 0.2);
        assert!(events.len() >= 8, "Detected {} droplets", events.len());

        // Check spacing
        let spacings = proc.droplet_spacing(&events);
        assert!(!spacings.is_empty());

        // Check dimensionless numbers
        let ca = proc.capillary_number();
        let we = proc.weber_number();
        let re = proc.reynolds_number();
        assert!(ca > 0.0);
        assert!(we > 0.0);
        assert!(re > 0.0);
    }

    #[test]
    fn test_detect_with_baseline_correction() {
        let proc = make_processor();

        // Signal with drift + droplets
        let mut sig = synth_signal(100_000.0, 5, 0.005, 0.001, 1.0);
        // Add linear drift
        for i in 0..sig.len() {
            sig[i] += 0.3 + 0.0001 * i as f64;
        }

        // Correct baseline first
        let corrected = proc.signal_baseline_correction(&sig, 201, None);
        let events = proc.detect_droplets(&corrected, 0.5, 0.2);
        // Should still detect droplets after baseline removal
        assert!(events.len() >= 3, "Detected {} droplets after correction", events.len());
    }
}
