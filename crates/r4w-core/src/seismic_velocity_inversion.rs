//! Seismic velocity model estimation from traveltime and waveform data.
//!
//! Applications: oil/gas exploration, earthquake seismology, crustal structure studies,
//! engineering site characterization.
//!
//! # Components
//!
//! - [`VelocityModel`] -- 1D layered velocity model with Vp, Vs, density, thickness
//! - [`RayTracer`] -- Snell's law ray tracing through layered media
//! - [`TraveltimeCalculator`] -- P-wave and S-wave traveltime computation
//! - [`NmoCorrection`] -- Normal Move-Out correction for CMP gathers
//! - [`SemblanceAnalysis`] -- Velocity spectrum via semblance coherence measure
//! - [`DixInversion`] -- Convert stacking (RMS) velocities to interval velocities
//! - [`TomographicInversion`] -- Linearized traveltime tomography (SIRT)
//! - [`DispersionCurveInversion`] -- Surface wave dispersion inversion for Vs profile
//! - [`VpVsRatioEstimator`] -- Wadati diagram for Vp/Vs ratio
//! - [`RefractedWaveInterpreter`] -- Head wave interpretation for layer velocities and depths
//!
//! # Physics
//!
//! Snell's law: `sin(theta_i) / V_i = sin(theta_j) / V_j = p` (ray parameter)
//!
//! NMO equation: `t^2 = t0^2 + x^2 / V_rms^2` (hyperbolic moveout)
//!
//! Dix equation: `V_int^2 = (V_rms_n^2 * t_n - V_rms_(n-1)^2 * t_(n-1)) / (t_n - t_(n-1))`
//!
//! # Example
//!
//! ```
//! use r4w_core::seismic_velocity_inversion::{VelocityModel, Layer, TraveltimeCalculator};
//!
//! // Build a simple 3-layer model
//! let model = VelocityModel::new(vec![
//!     Layer { vp: 1500.0, vs: 800.0, density: 1800.0, thickness: 100.0 },
//!     Layer { vp: 2500.0, vs: 1400.0, density: 2200.0, thickness: 200.0 },
//!     Layer { vp: 4000.0, vs: 2300.0, density: 2600.0, thickness: f64::INFINITY },
//! ]);
//!
//! // Compute vertical two-way traveltime through the first two layers
//! let twt = model.two_way_time(1); // to bottom of layer index 1
//! assert!(twt > 0.0);
//!
//! // Calculate P-wave traveltime at offset
//! let calc = TraveltimeCalculator::new(&model);
//! let tt = calc.direct_p_traveltime(0.0, 500.0); // source at 0, receiver at 500 m
//! assert!(tt > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Layer and VelocityModel
// ---------------------------------------------------------------------------

/// A single layer in a 1D velocity model.
#[derive(Debug, Clone)]
pub struct Layer {
    /// P-wave velocity in m/s.
    pub vp: f64,
    /// S-wave velocity in m/s.
    pub vs: f64,
    /// Density in kg/m^3.
    pub density: f64,
    /// Layer thickness in meters (use `f64::INFINITY` for the half-space).
    pub thickness: f64,
}

impl Layer {
    /// Acoustic impedance for P-wave: Z = density * Vp.
    pub fn impedance_p(&self) -> f64 {
        self.density * self.vp
    }

    /// Acoustic impedance for S-wave: Z = density * Vs.
    pub fn impedance_s(&self) -> f64 {
        self.density * self.vs
    }

    /// Vp/Vs ratio (Poisson's ratio related).
    pub fn vp_vs_ratio(&self) -> f64 {
        self.vp / self.vs
    }

    /// Poisson's ratio from Vp/Vs: nu = (r^2 - 2) / (2*(r^2 - 1)) where r = Vp/Vs.
    pub fn poissons_ratio(&self) -> f64 {
        let r = self.vp / self.vs;
        let r2 = r * r;
        (r2 - 2.0) / (2.0 * (r2 - 1.0))
    }

    /// P-wave reflection coefficient at normal incidence between this layer (top)
    /// and the layer below.
    pub fn reflection_coefficient_p(&self, below: &Layer) -> f64 {
        let z1 = self.impedance_p();
        let z2 = below.impedance_p();
        (z2 - z1) / (z2 + z1)
    }
}

/// 1D horizontally layered velocity model.
#[derive(Debug, Clone)]
pub struct VelocityModel {
    /// Layers from top to bottom. The last layer is the half-space.
    pub layers: Vec<Layer>,
}

impl VelocityModel {
    /// Create a new velocity model from layers.
    pub fn new(layers: Vec<Layer>) -> Self {
        assert!(!layers.is_empty(), "Model must have at least one layer");
        Self { layers }
    }

    /// Number of layers.
    pub fn num_layers(&self) -> usize {
        self.layers.len()
    }

    /// Depth to the top of the given layer index.
    pub fn depth_to_top(&self, layer_idx: usize) -> f64 {
        self.layers[..layer_idx]
            .iter()
            .map(|l| if l.thickness.is_finite() { l.thickness } else { 0.0 })
            .sum()
    }

    /// Depth to the bottom of the given layer index.
    pub fn depth_to_bottom(&self, layer_idx: usize) -> f64 {
        self.depth_to_top(layer_idx) + if self.layers[layer_idx].thickness.is_finite() {
            self.layers[layer_idx].thickness
        } else {
            0.0
        }
    }

    /// One-way vertical P-wave traveltime through layers 0..=layer_idx.
    pub fn one_way_time(&self, layer_idx: usize) -> f64 {
        self.layers[..=layer_idx]
            .iter()
            .filter(|l| l.thickness.is_finite())
            .map(|l| l.thickness / l.vp)
            .sum()
    }

    /// Two-way vertical P-wave traveltime to the bottom of layer_idx.
    pub fn two_way_time(&self, layer_idx: usize) -> f64 {
        2.0 * self.one_way_time(layer_idx)
    }

    /// RMS velocity down to and including layer_idx (P-wave).
    /// V_rms^2 = sum(V_i^2 * dt_i) / sum(dt_i)
    pub fn rms_velocity(&self, layer_idx: usize) -> f64 {
        let mut sum_v2dt = 0.0;
        let mut sum_dt = 0.0;
        for layer in &self.layers[..=layer_idx] {
            if layer.thickness.is_finite() {
                let dt = layer.thickness / layer.vp;
                sum_v2dt += layer.vp * layer.vp * dt;
                sum_dt += dt;
            }
        }
        if sum_dt > 0.0 {
            (sum_v2dt / sum_dt).sqrt()
        } else {
            self.layers[layer_idx].vp
        }
    }

    /// Average velocity down to and including layer_idx (P-wave).
    pub fn average_velocity(&self, layer_idx: usize) -> f64 {
        let total_thickness: f64 = self.layers[..=layer_idx]
            .iter()
            .filter(|l| l.thickness.is_finite())
            .map(|l| l.thickness)
            .sum();
        let total_time: f64 = self.layers[..=layer_idx]
            .iter()
            .filter(|l| l.thickness.is_finite())
            .map(|l| l.thickness / l.vp)
            .sum();
        if total_time > 0.0 {
            total_thickness / total_time
        } else {
            self.layers[layer_idx].vp
        }
    }

    /// Create a simple gradient model: velocity increases linearly with depth.
    pub fn gradient_model(v_top: f64, gradient: f64, num_layers: usize, layer_thickness: f64) -> Self {
        let layers: Vec<Layer> = (0..num_layers)
            .map(|i| {
                let depth = i as f64 * layer_thickness + layer_thickness / 2.0;
                let vp = v_top + gradient * depth;
                Layer {
                    vp,
                    vs: vp / 1.73,
                    density: 1600.0 + 0.3 * vp, // Gardner's relation approximation
                    thickness: if i < num_layers - 1 { layer_thickness } else { f64::INFINITY },
                }
            })
            .collect();
        Self::new(layers)
    }
}

// ---------------------------------------------------------------------------
// RayTracer
// ---------------------------------------------------------------------------

/// Result of ray tracing through the model.
#[derive(Debug, Clone)]
pub struct RayResult {
    /// Ray parameter p = sin(theta)/V in s/m.
    pub ray_parameter: f64,
    /// Total traveltime in seconds.
    pub traveltime: f64,
    /// Horizontal offset (distance) in meters.
    pub offset: f64,
    /// Incidence angles at each layer boundary (radians).
    pub angles: Vec<f64>,
    /// Deepest layer the ray penetrates.
    pub turning_layer: usize,
}

/// Snell's law ray tracing through a 1D layered velocity model.
pub struct RayTracer<'a> {
    model: &'a VelocityModel,
}

impl<'a> RayTracer<'a> {
    /// Create a new ray tracer for the given velocity model.
    pub fn new(model: &'a VelocityModel) -> Self {
        Self { model }
    }

    /// Trace a ray from the surface with a given ray parameter p (s/m).
    /// Returns the total traveltime and horizontal offset for a reflected ray
    /// from the bottom of `target_layer`.
    pub fn trace_reflected(&self, p: f64, target_layer: usize) -> Option<RayResult> {
        let mut traveltime = 0.0;
        let mut offset = 0.0;
        let mut angles = Vec::new();

        for i in 0..=target_layer {
            let layer = &self.model.layers[i];
            let sin_theta = p * layer.vp;
            if sin_theta.abs() > 1.0 {
                return None; // Total internal reflection / post-critical
            }
            let theta = sin_theta.asin();
            angles.push(theta);
            let cos_theta = theta.cos();

            if !layer.thickness.is_finite() {
                // Half-space: ray reflects at the top interface, no traversal
                break;
            }

            // Contribution from this layer (one-way down)
            let dt = layer.thickness / (layer.vp * cos_theta);
            let dx = layer.thickness * sin_theta / cos_theta;
            traveltime += dt;
            offset += dx;
        }

        // Two-way path
        traveltime *= 2.0;
        offset *= 2.0;

        Some(RayResult {
            ray_parameter: p,
            traveltime,
            offset,
            angles,
            turning_layer: target_layer,
        })
    }

    /// Find the ray parameter that produces a given offset for a reflection
    /// from `target_layer`, using bisection.
    pub fn find_ray_parameter_for_offset(
        &self,
        target_offset: f64,
        target_layer: usize,
        max_iter: usize,
    ) -> Option<f64> {
        // Ray parameter bounds: p=0 (vertical) to p = 1/Vmin (critical)
        let mut p_lo = 0.0;
        let mut p_hi = 0.999 / self.model.layers[0..=target_layer]
            .iter()
            .map(|l| l.vp)
            .fold(f64::INFINITY, f64::min);

        for _ in 0..max_iter {
            let p_mid = (p_lo + p_hi) / 2.0;
            if let Some(result) = self.trace_reflected(p_mid, target_layer) {
                if (result.offset - target_offset).abs() < 0.01 {
                    return Some(p_mid);
                }
                if result.offset < target_offset {
                    p_lo = p_mid;
                } else {
                    p_hi = p_mid;
                }
            } else {
                p_hi = p_mid;
            }
        }

        let p_mid = (p_lo + p_hi) / 2.0;
        self.trace_reflected(p_mid, target_layer).map(|_| p_mid)
    }

    /// Compute traveltime vs offset curve for a reflection from `target_layer`.
    pub fn traveltime_curve(
        &self,
        target_layer: usize,
        offsets: &[f64],
    ) -> Vec<(f64, f64)> {
        offsets
            .iter()
            .filter_map(|&x| {
                self.find_ray_parameter_for_offset(x, target_layer, 100)
                    .and_then(|p| self.trace_reflected(p, target_layer))
                    .map(|r| (x, r.traveltime))
            })
            .collect()
    }

    /// Compute the critical angle for a ray going from layer_above into layer_below.
    /// Returns None if layer_below is slower (no critical angle).
    pub fn critical_angle(&self, layer_above: usize, layer_below: usize) -> Option<f64> {
        let v1 = self.model.layers[layer_above].vp;
        let v2 = self.model.layers[layer_below].vp;
        if v2 > v1 {
            Some((v1 / v2).asin())
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// TraveltimeCalculator
// ---------------------------------------------------------------------------

/// Computes P-wave and S-wave traveltimes for source-receiver geometry.
pub struct TraveltimeCalculator<'a> {
    model: &'a VelocityModel,
}

impl<'a> TraveltimeCalculator<'a> {
    /// Create a new traveltime calculator.
    pub fn new(model: &'a VelocityModel) -> Self {
        Self { model }
    }

    /// Direct P-wave traveltime through the top layer (straight line).
    /// Assumes source and receiver at the surface, direct wave through the first layer.
    pub fn direct_p_traveltime(&self, source_x: f64, receiver_x: f64) -> f64 {
        let offset = (receiver_x - source_x).abs();
        offset / self.model.layers[0].vp
    }

    /// Direct S-wave traveltime through the top layer.
    pub fn direct_s_traveltime(&self, source_x: f64, receiver_x: f64) -> f64 {
        let offset = (receiver_x - source_x).abs();
        offset / self.model.layers[0].vs
    }

    /// NMO-approximated P-wave reflection traveltime from the bottom of `reflector_idx`.
    /// Uses the hyperbolic approximation: t^2 = t0^2 + x^2/V_rms^2.
    pub fn nmo_traveltime(&self, offset: f64, reflector_idx: usize) -> f64 {
        let t0 = self.model.two_way_time(reflector_idx);
        let v_rms = self.model.rms_velocity(reflector_idx);
        (t0 * t0 + offset * offset / (v_rms * v_rms)).sqrt()
    }

    /// Refracted (head wave) P-wave traveltime along the top of `refractor_layer`.
    /// The head wave travels down through layers above, along the refractor, then back up.
    /// Valid only when V_refractor > V_layers_above.
    pub fn refracted_p_traveltime(&self, offset: f64, refractor_layer: usize) -> Option<f64> {
        if refractor_layer == 0 {
            return None;
        }
        let v_refract = self.model.layers[refractor_layer].vp;

        // Check that refractor is faster than all layers above
        for i in 0..refractor_layer {
            if self.model.layers[i].vp >= v_refract {
                return None;
            }
        }

        // Time along the refractor
        let mut t_along = offset / v_refract;

        // Time in each layer above (down and up legs)
        for i in 0..refractor_layer {
            let v_i = self.model.layers[i].vp;
            let h_i = self.model.layers[i].thickness;
            if !h_i.is_finite() {
                return None;
            }
            let cos_ic = (1.0 - (v_i / v_refract).powi(2)).sqrt();
            // Two-way through layer i
            t_along += 2.0 * h_i * cos_ic / v_i;
            // Subtract the horizontal distance traveled in each layer
            // (already accounted for in offset/v_refract, so we subtract that portion)
            let sin_ic = v_i / v_refract;
            t_along -= 2.0 * h_i * sin_ic / v_refract;
        }

        if t_along > 0.0 {
            Some(t_along)
        } else {
            None
        }
    }

    /// Crossover distance where refracted wave arrives before the direct wave.
    /// For a two-layer model: x_cross = 2*h * sqrt((V2+V1)/(V2-V1))
    pub fn crossover_distance(&self, refractor_layer: usize) -> Option<f64> {
        if refractor_layer != 1 || self.model.layers.len() < 2 {
            // Only implement the simple two-layer case
            return None;
        }
        let v1 = self.model.layers[0].vp;
        let v2 = self.model.layers[1].vp;
        let h = self.model.layers[0].thickness;
        if !h.is_finite() || v2 <= v1 {
            return None;
        }
        Some(2.0 * h * ((v2 + v1) / (v2 - v1)).sqrt())
    }
}

// ---------------------------------------------------------------------------
// NmoCorrection
// ---------------------------------------------------------------------------

/// Normal Move-Out correction for CMP (Common Mid-Point) gathers.
///
/// NMO flattens hyperbolic reflection events so they can be stacked.
/// t_nmo(x) = sqrt(t0^2 + x^2 / Vnmo^2)
pub struct NmoCorrection;

impl NmoCorrection {
    /// Compute NMO-corrected time for a given offset and velocity.
    /// Returns t0 (zero-offset traveltime) given observed t(x).
    pub fn correct(t_observed: f64, offset: f64, v_nmo: f64) -> f64 {
        let t0_sq = t_observed * t_observed - offset * offset / (v_nmo * v_nmo);
        if t0_sq > 0.0 {
            t0_sq.sqrt()
        } else {
            0.0
        }
    }

    /// Compute the NMO moveout: Delta_t = t(x) - t0.
    pub fn moveout(t0: f64, offset: f64, v_nmo: f64) -> f64 {
        let t_x = (t0 * t0 + offset * offset / (v_nmo * v_nmo)).sqrt();
        t_x - t0
    }

    /// Apply NMO correction to a CMP gather.
    /// `gather`: 2D array [trace_idx][sample_idx] of amplitudes.
    /// `offsets`: source-receiver offsets for each trace.
    /// `v_nmo`: NMO velocity in m/s.
    /// `dt`: sample interval in seconds.
    /// Returns corrected gather with flattened reflections.
    pub fn apply_correction(
        gather: &[Vec<f64>],
        offsets: &[f64],
        v_nmo: f64,
        dt: f64,
    ) -> Vec<Vec<f64>> {
        let num_traces = gather.len();
        if num_traces == 0 {
            return vec![];
        }
        let num_samples = gather[0].len();
        let mut corrected = vec![vec![0.0; num_samples]; num_traces];

        for (i, trace) in gather.iter().enumerate() {
            let x = offsets[i];
            for j in 0..num_samples {
                let t0 = j as f64 * dt;
                let t_x = (t0 * t0 + x * x / (v_nmo * v_nmo)).sqrt();
                let sample_idx = t_x / dt;
                let idx_lo = sample_idx.floor() as usize;
                let idx_hi = idx_lo + 1;
                let frac = sample_idx - idx_lo as f64;

                if idx_hi < trace.len() {
                    // Linear interpolation
                    corrected[i][j] = trace[idx_lo] * (1.0 - frac) + trace[idx_hi] * frac;
                }
            }
        }

        corrected
    }

    /// NMO stretch factor: beta = t(x)/t0. Large stretch at shallow times
    /// causes frequency distortion (NMO stretch). Beta > threshold should be muted.
    pub fn stretch_factor(t0: f64, offset: f64, v_nmo: f64) -> f64 {
        if t0 <= 0.0 {
            return f64::INFINITY;
        }
        let t_x = (t0 * t0 + offset * offset / (v_nmo * v_nmo)).sqrt();
        t_x / t0
    }

    /// Apply stretch muting: set samples to zero where stretch exceeds threshold.
    pub fn apply_stretch_mute(
        corrected: &mut [Vec<f64>],
        offsets: &[f64],
        v_nmo: f64,
        dt: f64,
        max_stretch: f64,
    ) {
        for (i, trace) in corrected.iter_mut().enumerate() {
            let x = offsets[i];
            for j in 0..trace.len() {
                let t0 = j as f64 * dt;
                if t0 > 0.0 {
                    let stretch = Self::stretch_factor(t0, x, v_nmo);
                    if stretch > max_stretch {
                        trace[j] = 0.0;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// SemblanceAnalysis
// ---------------------------------------------------------------------------

/// Velocity analysis via semblance coherence measure.
///
/// Semblance S(t0, Vnmo) measures how well NMO-corrected traces stack
/// at a given (t0, velocity) point. S in [0, 1], where 1 = perfect coherence.
pub struct SemblanceAnalysis;

impl SemblanceAnalysis {
    /// Compute semblance for a single (t0, v_nmo) pair over a CMP gather.
    ///
    /// S = [sum_j(sum_i f_nmo(i,j))^2] / [M * sum_j(sum_i f_nmo(i,j)^2)]
    ///
    /// where i = trace index, j = sample in window around t0,
    /// f_nmo = NMO-corrected sample value.
    pub fn semblance_point(
        gather: &[Vec<f64>],
        offsets: &[f64],
        t0: f64,
        v_nmo: f64,
        dt: f64,
        half_window: usize,
    ) -> f64 {
        let m = gather.len() as f64;
        if m < 1.0 {
            return 0.0;
        }
        let num_samples = if gather.is_empty() { 0 } else { gather[0].len() };
        let center = (t0 / dt).round() as isize;

        let mut numerator = 0.0;
        let mut denominator = 0.0;

        for dj in -(half_window as isize)..=(half_window as isize) {
            let j = center + dj;
            if j < 0 || j as usize >= num_samples {
                continue;
            }
            let t0_j = j as f64 * dt;

            let mut sum_f = 0.0;
            let mut sum_f2 = 0.0;

            for (i, trace) in gather.iter().enumerate() {
                let x = offsets[i];
                let t_x = (t0_j * t0_j + x * x / (v_nmo * v_nmo)).sqrt();
                let sample_idx = t_x / dt;
                let idx_lo = sample_idx.floor() as usize;
                let idx_hi = idx_lo + 1;
                let frac = sample_idx - idx_lo as f64;

                let val = if idx_hi < trace.len() {
                    trace[idx_lo] * (1.0 - frac) + trace[idx_hi] * frac
                } else {
                    0.0
                };

                sum_f += val;
                sum_f2 += val * val;
            }

            numerator += sum_f * sum_f;
            denominator += m * sum_f2;
        }

        if denominator > 1e-30 {
            (numerator / denominator).clamp(0.0, 1.0)
        } else {
            0.0
        }
    }

    /// Compute full velocity spectrum: semblance over a grid of (t0, v_nmo).
    /// Returns a 2D grid [t0_idx][v_idx] of semblance values.
    pub fn velocity_spectrum(
        gather: &[Vec<f64>],
        offsets: &[f64],
        dt: f64,
        t0_values: &[f64],
        velocity_values: &[f64],
        half_window: usize,
    ) -> Vec<Vec<f64>> {
        t0_values
            .iter()
            .map(|&t0| {
                velocity_values
                    .iter()
                    .map(|&v| Self::semblance_point(gather, offsets, t0, v, dt, half_window))
                    .collect()
            })
            .collect()
    }

    /// Pick the best velocity at each t0 from the velocity spectrum.
    /// Returns pairs of (t0, best_velocity).
    pub fn pick_velocities(
        spectrum: &[Vec<f64>],
        t0_values: &[f64],
        velocity_values: &[f64],
    ) -> Vec<(f64, f64)> {
        spectrum
            .iter()
            .enumerate()
            .map(|(i, row)| {
                let best_v_idx = row
                    .iter()
                    .enumerate()
                    .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
                    .map(|(idx, _)| idx)
                    .unwrap_or(0);
                (t0_values[i], velocity_values[best_v_idx])
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// DixInversion
// ---------------------------------------------------------------------------

/// Dix equation: convert stacking (RMS) velocities to interval velocities.
///
/// V_int_n^2 = (V_rms_n^2 * t_n - V_rms_(n-1)^2 * t_(n-1)) / (t_n - t_(n-1))
pub struct DixInversion;

impl DixInversion {
    /// Convert a series of (t0, V_rms) picks to interval velocities.
    /// Input: pairs of (two_way_time, rms_velocity) sorted by increasing time.
    /// Returns interval velocities for each interval between picks.
    pub fn compute(picks: &[(f64, f64)]) -> Vec<f64> {
        if picks.is_empty() {
            return vec![];
        }

        let mut interval_velocities = Vec::with_capacity(picks.len());
        // First interval: V_int = V_rms (since there's nothing above)
        interval_velocities.push(picks[0].1);

        for i in 1..picks.len() {
            let (t_n, v_rms_n) = picks[i];
            let (t_prev, v_rms_prev) = picks[i - 1];
            let dt = t_n - t_prev;
            if dt > 1e-12 {
                let v_int_sq = (v_rms_n * v_rms_n * t_n - v_rms_prev * v_rms_prev * t_prev) / dt;
                if v_int_sq > 0.0 {
                    interval_velocities.push(v_int_sq.sqrt());
                } else {
                    // Physically unrealistic (velocity inversion artifact); clamp to previous
                    interval_velocities.push(*interval_velocities.last().unwrap_or(&0.0));
                }
            }
        }

        interval_velocities
    }

    /// Convert interval velocities back to RMS velocities.
    /// Input: pairs of (interval_velocity, layer_time) for each layer.
    /// Returns cumulative RMS velocity at the bottom of each layer.
    pub fn interval_to_rms(intervals: &[(f64, f64)]) -> Vec<f64> {
        let mut rms_vels = Vec::with_capacity(intervals.len());
        let mut sum_v2t = 0.0;
        let mut sum_t = 0.0;

        for &(v_int, dt) in intervals {
            sum_v2t += v_int * v_int * dt;
            sum_t += dt;
            if sum_t > 0.0 {
                rms_vels.push((sum_v2t / sum_t).sqrt());
            }
        }

        rms_vels
    }

    /// Compute layer thicknesses from interval velocities and two-way times.
    pub fn layer_thicknesses(picks: &[(f64, f64)]) -> Vec<f64> {
        let v_ints = Self::compute(picks);
        let mut thicknesses = Vec::with_capacity(picks.len());

        // First layer
        if !picks.is_empty() {
            thicknesses.push(v_ints[0] * picks[0].0 / 2.0);
        }

        for i in 1..picks.len() {
            let dt = picks[i].0 - picks[i - 1].0;
            thicknesses.push(v_ints[i] * dt / 2.0);
        }

        thicknesses
    }
}

// ---------------------------------------------------------------------------
// TomographicInversion
// ---------------------------------------------------------------------------

/// Linearized traveltime tomography using SIRT (Simultaneous Iterative
/// Reconstruction Technique).
///
/// Given observed traveltimes and ray paths through a gridded model,
/// iteratively updates slowness (1/velocity) to minimize traveltime residuals.
pub struct TomographicInversion {
    /// Number of cells in the model (nx * nz).
    pub num_cells: usize,
    /// Current slowness model (s/m) for each cell.
    pub slowness: Vec<f64>,
    /// Number of cells in x direction.
    pub nx: usize,
    /// Number of cells in z direction.
    pub nz: usize,
    /// Cell size in meters.
    pub cell_size: f64,
}

/// A ray path through the model, represented as (cell_index, path_length) pairs.
#[derive(Debug, Clone)]
pub struct RayPath {
    /// Pairs of (cell_index, segment_length_in_cell).
    pub segments: Vec<(usize, f64)>,
    /// Observed traveltime for this ray.
    pub observed_time: f64,
}

impl TomographicInversion {
    /// Create a new tomographic model with uniform initial velocity.
    pub fn new(nx: usize, nz: usize, cell_size: f64, initial_velocity: f64) -> Self {
        let num_cells = nx * nz;
        let slowness = vec![1.0 / initial_velocity; num_cells];
        Self {
            num_cells,
            slowness,
            nx,
            nz,
            cell_size,
        }
    }

    /// Compute the predicted traveltime for a given ray path.
    pub fn predicted_traveltime(&self, ray: &RayPath) -> f64 {
        ray.segments
            .iter()
            .map(|&(cell, dl)| self.slowness[cell] * dl)
            .sum()
    }

    /// Run SIRT iterations on the given ray paths.
    ///
    /// SIRT update: delta_s_j = (1/N_j) * sum_i [ (t_obs_i - t_pred_i) * dl_ij / L_i^2 ]
    /// where N_j = number of rays through cell j, L_i = total path length of ray i.
    pub fn sirt_iterate(&mut self, rays: &[RayPath], num_iterations: usize, damping: f64) {
        for _ in 0..num_iterations {
            let mut delta_s = vec![0.0; self.num_cells];
            let mut hit_count = vec![0u32; self.num_cells];

            for ray in rays {
                let t_pred = self.predicted_traveltime(ray);
                let residual = ray.observed_time - t_pred;

                // Total path length squared
                let l_sq: f64 = ray.segments.iter().map(|&(_, dl)| dl * dl).sum();
                if l_sq < 1e-30 {
                    continue;
                }

                for &(cell, dl) in &ray.segments {
                    delta_s[cell] += residual * dl / l_sq;
                    hit_count[cell] += 1;
                }
            }

            // Apply SIRT averaging
            for j in 0..self.num_cells {
                if hit_count[j] > 0 {
                    let update = damping * delta_s[j] / hit_count[j] as f64;
                    self.slowness[j] += update;
                    // Enforce positivity
                    if self.slowness[j] < 1e-10 {
                        self.slowness[j] = 1e-10;
                    }
                }
            }
        }
    }

    /// Get the velocity model (m/s) from the current slowness.
    pub fn velocity_model(&self) -> Vec<f64> {
        self.slowness.iter().map(|&s| 1.0 / s).collect()
    }

    /// Get the 2D velocity model as a grid [iz][ix].
    pub fn velocity_grid(&self) -> Vec<Vec<f64>> {
        (0..self.nz)
            .map(|iz| {
                (0..self.nx)
                    .map(|ix| 1.0 / self.slowness[iz * self.nx + ix])
                    .collect()
            })
            .collect()
    }

    /// RMS traveltime residual for all rays.
    pub fn rms_residual(&self, rays: &[RayPath]) -> f64 {
        if rays.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = rays
            .iter()
            .map(|ray| {
                let r = ray.observed_time - self.predicted_traveltime(ray);
                r * r
            })
            .sum();
        (sum_sq / rays.len() as f64).sqrt()
    }

    /// Create a simple straight-ray path from (x1, z1) to (x2, z2).
    /// Uses DDA-like line traversal through the cell grid.
    pub fn straight_ray(&self, x1: f64, z1: f64, x2: f64, z2: f64) -> Vec<(usize, f64)> {
        let dx = x2 - x1;
        let dz = z2 - z1;
        let total_length = (dx * dx + dz * dz).sqrt();
        if total_length < 1e-10 {
            return vec![];
        }

        let num_steps = (total_length / (self.cell_size * 0.1)).ceil() as usize;
        let num_steps = num_steps.max(10);
        let step = total_length / num_steps as f64;
        let ux = dx / total_length;
        let uz = dz / total_length;

        let mut segments: Vec<(usize, f64)> = Vec::new();

        for s in 0..num_steps {
            let x = x1 + ux * (s as f64 + 0.5) * step;
            let z = z1 + uz * (s as f64 + 0.5) * step;

            let ix = (x / self.cell_size).floor() as isize;
            let iz = (z / self.cell_size).floor() as isize;

            if ix >= 0 && ix < self.nx as isize && iz >= 0 && iz < self.nz as isize {
                let cell = iz as usize * self.nx + ix as usize;
                if let Some(last) = segments.last_mut() {
                    if last.0 == cell {
                        last.1 += step;
                        continue;
                    }
                }
                segments.push((cell, step));
            }
        }

        segments
    }
}

// ---------------------------------------------------------------------------
// DispersionCurveInversion
// ---------------------------------------------------------------------------

/// Surface wave dispersion inversion for 1D shear-wave velocity (Vs) profile.
///
/// Uses iterative linearized inversion of Rayleigh wave phase velocity
/// dispersion data to recover Vs vs depth.
pub struct DispersionCurveInversion {
    /// Layer thicknesses (m).
    pub thicknesses: Vec<f64>,
    /// Current Vs profile (m/s).
    pub vs_profile: Vec<f64>,
    /// Vp/Vs ratio per layer (default ~1.73).
    pub vp_vs_ratio: Vec<f64>,
    /// Densities per layer (kg/m^3).
    pub densities: Vec<f64>,
}

impl DispersionCurveInversion {
    /// Create a new inversion with uniform initial Vs.
    pub fn new(thicknesses: Vec<f64>, initial_vs: f64) -> Self {
        let n = thicknesses.len();
        Self {
            vs_profile: vec![initial_vs; n],
            vp_vs_ratio: vec![1.73; n],
            densities: vec![2000.0; n],
            thicknesses,
        }
    }

    /// Simplified Rayleigh wave phase velocity approximation for a layered model.
    /// Uses a depth-weighted average of Vs for a given wavelength.
    /// Real implementations use the Thomson-Haskell matrix method.
    pub fn forward_model(&self, frequencies: &[f64]) -> Vec<f64> {
        frequencies
            .iter()
            .map(|&f| {
                // Approximate penetration depth ~ 0.4 * wavelength
                // For initial Vs estimate, use average
                let avg_vs: f64 = self.vs_profile.iter().sum::<f64>() / self.vs_profile.len() as f64;
                let wavelength = avg_vs / f;
                let pen_depth = 0.4 * wavelength;

                // Depth-weighted average Vs
                let mut depth = 0.0;
                let mut weighted_vs = 0.0;
                let mut total_weight = 0.0;

                for (i, &h) in self.thicknesses.iter().enumerate() {
                    let h_eff = if h.is_finite() { h } else { pen_depth };
                    let layer_top = depth;
                    let layer_bot = depth + h_eff;

                    // Weight decreases with depth (sensitivity kernel)
                    let z_mid = (layer_top + layer_bot.min(pen_depth)) / 2.0;
                    if z_mid <= pen_depth {
                        let w = 1.0 - z_mid / pen_depth;
                        let dz = h_eff.min(pen_depth - layer_top).max(0.0);
                        weighted_vs += w * self.vs_profile[i] * dz;
                        total_weight += w * dz;
                    }
                    depth += h_eff;
                    if depth >= pen_depth {
                        break;
                    }
                }

                if total_weight > 0.0 {
                    // Rayleigh wave is ~0.92 * Vs for Poisson solid
                    0.92 * weighted_vs / total_weight
                } else {
                    0.92 * self.vs_profile[0]
                }
            })
            .collect()
    }

    /// Invert observed dispersion curve using iterative least-squares.
    /// `observed`: pairs of (frequency_hz, phase_velocity_m_s).
    /// Returns final RMS misfit.
    pub fn invert(
        &mut self,
        observed: &[(f64, f64)],
        num_iterations: usize,
        damping: f64,
    ) -> f64 {
        let freqs: Vec<f64> = observed.iter().map(|&(f, _)| f).collect();
        let obs_vel: Vec<f64> = observed.iter().map(|&(_, v)| v).collect();
        let n_layers = self.vs_profile.len();

        for _ in 0..num_iterations {
            let pred = self.forward_model(&freqs);

            // Compute Jacobian by finite differences
            let dv = 1.0; // 1 m/s perturbation
            let mut jacobian = vec![vec![0.0; n_layers]; freqs.len()];

            for j in 0..n_layers {
                self.vs_profile[j] += dv;
                let pred_pert = self.forward_model(&freqs);
                self.vs_profile[j] -= dv;

                for i in 0..freqs.len() {
                    jacobian[i][j] = (pred_pert[i] - pred[i]) / dv;
                }
            }

            // Residual
            let residual: Vec<f64> = obs_vel
                .iter()
                .zip(pred.iter())
                .map(|(&o, &p)| o - p)
                .collect();

            // Damped least-squares: delta_m = (J^T J + lambda*I)^-1 J^T r
            // Simplified: use gradient descent with J^T r
            let mut gradient = vec![0.0; n_layers];
            for j in 0..n_layers {
                for i in 0..freqs.len() {
                    gradient[j] += jacobian[i][j] * residual[i];
                }
            }

            // Normalize and apply update
            let grad_norm: f64 = gradient.iter().map(|g| g * g).sum::<f64>().sqrt();
            if grad_norm > 1e-10 {
                for j in 0..n_layers {
                    self.vs_profile[j] += damping * gradient[j] / grad_norm * 10.0;
                    // Enforce positivity
                    if self.vs_profile[j] < 50.0 {
                        self.vs_profile[j] = 50.0;
                    }
                }
            }
        }

        // Compute final misfit
        let pred = self.forward_model(&freqs);
        let rms: f64 = obs_vel
            .iter()
            .zip(pred.iter())
            .map(|(&o, &p)| (o - p) * (o - p))
            .sum::<f64>();
        (rms / obs_vel.len() as f64).sqrt()
    }
}

// ---------------------------------------------------------------------------
// VpVsRatioEstimator (Wadati diagram)
// ---------------------------------------------------------------------------

/// Wadati diagram for Vp/Vs ratio estimation.
///
/// Plots (Tp-Ts) vs Tp; slope m gives Vp/Vs = 1 + 1/m or equivalently Vp/Vs = m + 1
/// depending on convention. Here: Ts - Tp = (Vp/Vs - 1) * Tp, so slope = Vp/Vs - 1.
pub struct VpVsRatioEstimator;

impl VpVsRatioEstimator {
    /// Estimate Vp/Vs ratio from P-wave and S-wave arrival times.
    /// Input: pairs of (tp, ts) arrival times at multiple stations.
    /// Uses linear regression of (Ts - Tp) vs Tp.
    /// Returns (vp_vs_ratio, r_squared) or None if insufficient data.
    pub fn estimate(arrivals: &[(f64, f64)]) -> Option<(f64, f64)> {
        if arrivals.len() < 2 {
            return None;
        }

        // x = Tp, y = Ts - Tp
        let n = arrivals.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;
        let mut sum_yy = 0.0;

        for &(tp, ts) in arrivals {
            let x = tp;
            let y = ts - tp;
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
            sum_yy += y * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        // Vp/Vs = slope + 1
        let vp_vs = slope + 1.0;

        // R-squared
        let mean_y = sum_y / n;
        let ss_tot = sum_yy - n * mean_y * mean_y;
        let intercept = (sum_y - slope * sum_x) / n;
        let ss_res: f64 = arrivals
            .iter()
            .map(|&(tp, ts)| {
                let predicted = slope * tp + intercept;
                let r = (ts - tp) - predicted;
                r * r
            })
            .sum();

        let r_sq = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

        Some((vp_vs, r_sq))
    }

    /// Compute Poisson's ratio from Vp/Vs ratio.
    /// nu = (r^2 - 2) / (2*(r^2 - 1))
    pub fn poissons_ratio(vp_vs: f64) -> f64 {
        let r2 = vp_vs * vp_vs;
        (r2 - 2.0) / (2.0 * (r2 - 1.0))
    }

    /// Origin time estimation from Wadati diagram.
    /// The x-intercept of the (Ts-Tp) vs Tp line gives the origin time.
    pub fn origin_time(arrivals: &[(f64, f64)]) -> Option<f64> {
        if arrivals.len() < 2 {
            return None;
        }

        let n = arrivals.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &(tp, ts) in arrivals {
            let x = tp;
            let y = ts - tp;
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        if slope.abs() < 1e-30 {
            return None;
        }

        // x-intercept: 0 = slope * x + intercept => x = -intercept / slope
        Some(-intercept / slope)
    }
}

// ---------------------------------------------------------------------------
// RefractedWaveInterpreter
// ---------------------------------------------------------------------------

/// Refraction seismology: interpret head wave arrivals for layer velocities and depths.
///
/// For a two-layer model:
/// - Direct wave: t = x / V1
/// - Head wave: t = x / V2 + 2*h*cos(ic)/V1
///   where ic = arcsin(V1/V2) is the critical angle.
pub struct RefractedWaveInterpreter;

/// Result of refraction interpretation.
#[derive(Debug, Clone)]
pub struct RefractionResult {
    /// Layer velocities from top to bottom (m/s).
    pub velocities: Vec<f64>,
    /// Layer thicknesses (m). Last entry is the half-space (no thickness).
    pub thicknesses: Vec<f64>,
    /// Depths to layer interfaces (m).
    pub interface_depths: Vec<f64>,
    /// Critical angles at each interface (radians).
    pub critical_angles: Vec<f64>,
}

impl RefractedWaveInterpreter {
    /// Interpret a two-layer refraction model from first-break picks.
    ///
    /// `picks`: pairs of (offset_m, traveltime_s) sorted by offset.
    ///
    /// Identifies the crossover distance and fits:
    /// - V1 from the slope of the direct wave segment
    /// - V2 from the slope of the refracted wave segment
    /// - h from the time intercept
    pub fn interpret_two_layer(picks: &[(f64, f64)]) -> Option<RefractionResult> {
        if picks.len() < 4 {
            return None;
        }

        // Find the crossover point by detecting slope change.
        // Compute apparent velocity (slope^-1) for successive pairs,
        // then find where it changes significantly.
        let mut crossover_idx = picks.len() / 2;

        // Compute incremental velocities between successive points
        let mut inc_vels = Vec::new();
        for i in 1..picks.len() {
            let dx = picks[i].0 - picks[i - 1].0;
            let dt = picks[i].1 - picks[i - 1].1;
            if dt > 1e-12 && dx > 1e-12 {
                inc_vels.push((i, dx / dt));
            }
        }

        if inc_vels.len() >= 2 {
            // Find the biggest jump in incremental velocity
            let mut max_jump = 0.0;
            for i in 1..inc_vels.len() {
                let jump = inc_vels[i].1 - inc_vels[i - 1].1;
                if jump > max_jump {
                    max_jump = jump;
                    crossover_idx = inc_vels[i].0;
                }
            }
        }

        // Ensure at least 2 points in each segment
        crossover_idx = crossover_idx.max(2).min(picks.len() - 2);

        // Linear regression on first segment for V1
        let v1 = Self::fit_velocity(&picks[..crossover_idx])?;

        // Linear regression on last segment for V2
        let v2 = Self::fit_velocity(&picks[crossover_idx..])?;

        if v2 <= v1 {
            return None; // Refraction requires V2 > V1
        }

        // Time intercept: fit refracted segment => t = x/V2 + t_intercept
        let (_, t_int) = Self::fit_line(&picks[crossover_idx..])?;

        // h = t_intercept * V1 * V2 / (2 * sqrt(V2^2 - V1^2))
        let v_diff = (v2 * v2 - v1 * v1).sqrt();
        let h = t_int * v1 * v2 / (2.0 * v_diff);

        let ic = (v1 / v2).asin();

        Some(RefractionResult {
            velocities: vec![v1, v2],
            thicknesses: vec![h],
            interface_depths: vec![h],
            critical_angles: vec![ic],
        })
    }

    /// Interpret a three-layer refraction model.
    ///
    /// `picks_forward`: first-break picks from a forward shot.
    /// `v1`, `v2_apparent`: velocities from two-layer interpretation.
    /// `second_crossover_picks`: picks beyond the second crossover.
    pub fn interpret_three_layer(
        v1: f64,
        h1: f64,
        v2: f64,
        second_refraction_picks: &[(f64, f64)],
    ) -> Option<RefractionResult> {
        if second_refraction_picks.len() < 2 {
            return None;
        }

        // Fit V3 from the second refraction segment
        let v3 = Self::fit_velocity(second_refraction_picks)?;
        if v3 <= v2 {
            return None;
        }

        let (_, t_int2) = Self::fit_line(second_refraction_picks)?;

        // Remove the effect of the first layer
        let ic1_3 = (v1 / v3).asin();
        let t_layer1 = 2.0 * h1 * ic1_3.cos() / v1;
        let t_int_corrected = t_int2 - t_layer1;

        let ic2_3 = (v2 / v3).asin();
        let h2 = t_int_corrected * v2 / (2.0 * ic2_3.cos());

        Some(RefractionResult {
            velocities: vec![v1, v2, v3],
            thicknesses: vec![h1, h2],
            interface_depths: vec![h1, h1 + h2],
            critical_angles: vec![(v1 / v2).asin(), (v2 / v3).asin()],
        })
    }

    /// Fit velocity (1/slope) from (offset, time) picks via linear regression.
    fn fit_velocity(picks: &[(f64, f64)]) -> Option<f64> {
        if picks.len() < 2 {
            return None;
        }
        let n = picks.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &(x, t) in picks {
            sum_x += x;
            sum_y += t;
            sum_xx += x * x;
            sum_xy += x * t;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        if slope > 1e-10 {
            Some(1.0 / slope) // velocity = 1/slowness
        } else {
            None
        }
    }

    /// Fit a line t = slope * x + intercept. Returns (slope, intercept).
    fn fit_line(picks: &[(f64, f64)]) -> Option<(f64, f64)> {
        if picks.len() < 2 {
            return None;
        }
        let n = picks.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &(x, t) in picks {
            sum_x += x;
            sum_y += t;
            sum_xx += x * x;
            sum_xy += x * t;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;
        Some((slope, intercept))
    }

    /// Generate synthetic first-break picks for a two-layer model.
    pub fn synthetic_picks_two_layer(
        v1: f64,
        v2: f64,
        h: f64,
        offsets: &[f64],
    ) -> Vec<(f64, f64)> {
        let ic = (v1 / v2).asin();
        let t_intercept = 2.0 * h * ic.cos() / v1;

        offsets
            .iter()
            .map(|&x| {
                let t_direct = x / v1;
                let t_refracted = x / v2 + t_intercept;
                (x, t_direct.min(t_refracted))
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn simple_two_layer() -> VelocityModel {
        VelocityModel::new(vec![
            Layer { vp: 2000.0, vs: 1000.0, density: 2000.0, thickness: 500.0 },
            Layer { vp: 4000.0, vs: 2300.0, density: 2500.0, thickness: f64::INFINITY },
        ])
    }

    fn three_layer_model() -> VelocityModel {
        VelocityModel::new(vec![
            Layer { vp: 1500.0, vs: 800.0, density: 1800.0, thickness: 100.0 },
            Layer { vp: 2500.0, vs: 1400.0, density: 2200.0, thickness: 200.0 },
            Layer { vp: 4000.0, vs: 2300.0, density: 2600.0, thickness: f64::INFINITY },
        ])
    }

    // ---- Layer tests ----

    #[test]
    fn test_layer_impedance() {
        let layer = Layer { vp: 2000.0, vs: 1000.0, density: 2200.0, thickness: 100.0 };
        assert!((layer.impedance_p() - 4_400_000.0).abs() < 1.0);
        assert!((layer.impedance_s() - 2_200_000.0).abs() < 1.0);
    }

    #[test]
    fn test_layer_vp_vs_ratio() {
        let layer = Layer { vp: 3000.0, vs: 1732.0, density: 2200.0, thickness: 100.0 };
        assert!((layer.vp_vs_ratio() - 1.732).abs() < 0.01);
    }

    #[test]
    fn test_layer_poissons_ratio() {
        // For Vp/Vs = sqrt(3) ~ 1.732, Poisson's ratio = 0.25 (Poisson solid)
        let layer = Layer { vp: 1732.0, vs: 1000.0, density: 2200.0, thickness: 100.0 };
        let nu = layer.poissons_ratio();
        assert!((nu - 0.25).abs() < 0.01);
    }

    #[test]
    fn test_reflection_coefficient() {
        let l1 = Layer { vp: 2000.0, vs: 1000.0, density: 2000.0, thickness: 100.0 };
        let l2 = Layer { vp: 3000.0, vs: 1700.0, density: 2500.0, thickness: 100.0 };
        let rc = l1.reflection_coefficient_p(&l2);
        // Z1 = 4e6, Z2 = 7.5e6, RC = (7.5-4)/(7.5+4) = 3.5/11.5 ~ 0.304
        assert!((rc - 0.304).abs() < 0.01);
    }

    // ---- VelocityModel tests ----

    #[test]
    fn test_model_depth() {
        let model = three_layer_model();
        assert!((model.depth_to_top(0) - 0.0).abs() < 1e-6);
        assert!((model.depth_to_top(1) - 100.0).abs() < 1e-6);
        assert!((model.depth_to_top(2) - 300.0).abs() < 1e-6);
        assert!((model.depth_to_bottom(0) - 100.0).abs() < 1e-6);
        assert!((model.depth_to_bottom(1) - 300.0).abs() < 1e-6);
    }

    #[test]
    fn test_model_one_way_time() {
        let model = simple_two_layer();
        // Layer 0: 500m at 2000 m/s = 0.25s
        let owt = model.one_way_time(0);
        assert!((owt - 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_model_two_way_time() {
        let model = simple_two_layer();
        let twt = model.two_way_time(0);
        assert!((twt - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_rms_velocity_single_layer() {
        let model = simple_two_layer();
        // For a single layer, V_rms = Vp
        let v_rms = model.rms_velocity(0);
        assert!((v_rms - 2000.0).abs() < 1e-3);
    }

    #[test]
    fn test_rms_velocity_multi_layer() {
        let model = three_layer_model();
        // V_rms^2 = (1500^2 * 100/1500 + 2500^2 * 200/2500) / (100/1500 + 200/2500)
        // = (150000 + 500000) / (0.06667 + 0.08) = 650000 / 0.14667 ~ 4431818
        // V_rms ~ 2105
        let v_rms = model.rms_velocity(1);
        assert!(v_rms > 1500.0 && v_rms < 2500.0);
    }

    #[test]
    fn test_average_velocity() {
        let model = three_layer_model();
        // Average = total_distance / total_time = 300 / (100/1500 + 200/2500)
        let v_avg = model.average_velocity(1);
        assert!(v_avg > 1500.0 && v_avg < 2500.0);
    }

    #[test]
    fn test_gradient_model() {
        let model = VelocityModel::gradient_model(1500.0, 1.0, 5, 100.0);
        assert_eq!(model.num_layers(), 5);
        // Velocity should increase with depth
        for i in 1..5 {
            assert!(model.layers[i].vp > model.layers[i - 1].vp);
        }
        // Last layer is half-space
        assert!(model.layers[4].thickness.is_infinite());
    }

    // ---- RayTracer tests ----

    #[test]
    fn test_ray_trace_vertical() {
        let model = simple_two_layer();
        let tracer = RayTracer::new(&model);
        // Vertical ray: p=0
        let result = tracer.trace_reflected(0.0, 0).unwrap();
        assert!((result.traveltime - 0.5).abs() < 1e-6); // 2 * 500/2000
        assert!(result.offset.abs() < 1e-6);
    }

    #[test]
    fn test_ray_trace_nonzero_parameter() {
        let model = simple_two_layer();
        let tracer = RayTracer::new(&model);
        let p = 0.0001; // small ray parameter
        let result = tracer.trace_reflected(p, 0).unwrap();
        assert!(result.traveltime > 0.5); // longer than vertical
        assert!(result.offset > 0.0);
    }

    #[test]
    fn test_ray_postcritical_returns_none() {
        let model = simple_two_layer();
        let tracer = RayTracer::new(&model);
        // p > 1/2000 = 0.0005 causes post-critical in layer 0
        let result = tracer.trace_reflected(0.0006, 0);
        assert!(result.is_none());
    }

    #[test]
    fn test_find_ray_parameter() {
        let model = simple_two_layer();
        let tracer = RayTracer::new(&model);
        let p = tracer.find_ray_parameter_for_offset(200.0, 0, 200);
        assert!(p.is_some());
        let result = tracer.trace_reflected(p.unwrap(), 0).unwrap();
        assert!((result.offset - 200.0).abs() < 1.0);
    }

    #[test]
    fn test_traveltime_curve() {
        let model = simple_two_layer();
        let tracer = RayTracer::new(&model);
        let offsets = vec![0.0, 100.0, 200.0, 500.0];
        let curve = tracer.traveltime_curve(0, &offsets);
        // Traveltime should increase with offset
        for i in 1..curve.len() {
            assert!(curve[i].1 >= curve[i - 1].1);
        }
    }

    #[test]
    fn test_critical_angle() {
        let model = simple_two_layer();
        let tracer = RayTracer::new(&model);
        let ic = tracer.critical_angle(0, 1).unwrap();
        // sin(ic) = V1/V2 = 2000/4000 = 0.5 => ic = 30 degrees
        assert!((ic - PI / 6.0).abs() < 1e-6);
    }

    // ---- TraveltimeCalculator tests ----

    #[test]
    fn test_direct_p_traveltime() {
        let model = simple_two_layer();
        let calc = TraveltimeCalculator::new(&model);
        let tt = calc.direct_p_traveltime(0.0, 1000.0);
        assert!((tt - 0.5).abs() < 1e-6); // 1000/2000
    }

    #[test]
    fn test_direct_s_traveltime() {
        let model = simple_two_layer();
        let calc = TraveltimeCalculator::new(&model);
        let tt = calc.direct_s_traveltime(0.0, 1000.0);
        assert!((tt - 1.0).abs() < 1e-6); // 1000/1000
    }

    #[test]
    fn test_nmo_traveltime_zero_offset() {
        let model = simple_two_layer();
        let calc = TraveltimeCalculator::new(&model);
        let tt = calc.nmo_traveltime(0.0, 0);
        let twt = model.two_way_time(0);
        assert!((tt - twt).abs() < 1e-6);
    }

    #[test]
    fn test_nmo_traveltime_nonzero_offset() {
        let model = simple_two_layer();
        let calc = TraveltimeCalculator::new(&model);
        let tt = calc.nmo_traveltime(1000.0, 0);
        let twt = model.two_way_time(0);
        assert!(tt > twt);
    }

    #[test]
    fn test_refracted_traveltime() {
        let model = simple_two_layer();
        let calc = TraveltimeCalculator::new(&model);
        let tt = calc.refracted_p_traveltime(2000.0, 1);
        assert!(tt.is_some());
        assert!(tt.unwrap() > 0.0);
    }

    #[test]
    fn test_crossover_distance() {
        let model = simple_two_layer();
        let calc = TraveltimeCalculator::new(&model);
        let x_cross = calc.crossover_distance(1).unwrap();
        // x_cross = 2*500*sqrt((4000+2000)/(4000-2000)) = 1000*sqrt(3) ~ 1732 m
        assert!((x_cross - 1000.0 * 3.0_f64.sqrt()).abs() < 1.0);
    }

    // ---- NmoCorrection tests ----

    #[test]
    fn test_nmo_correct() {
        // t_observed = sqrt(0.5^2 + 1000^2/2000^2) = sqrt(0.25+0.25) = sqrt(0.5)
        let t_obs = (0.25 + 0.25_f64).sqrt();
        let t0 = NmoCorrection::correct(t_obs, 1000.0, 2000.0);
        assert!((t0 - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_nmo_moveout() {
        let dm = NmoCorrection::moveout(0.5, 1000.0, 2000.0);
        let expected = (0.25 + 0.25_f64).sqrt() - 0.5;
        assert!((dm - expected).abs() < 1e-6);
    }

    #[test]
    fn test_nmo_apply_correction() {
        // Create a simple gather with a hyperbolic event
        let dt = 0.004; // 4ms sample interval
        let num_samples = 250; // 1 second
        let offsets = vec![0.0, 500.0, 1000.0, 1500.0];
        let v_nmo = 2000.0;
        let t0_event: f64 = 0.5; // event at 0.5s zero-offset

        let mut gather = Vec::new();
        for &x in &offsets {
            let mut trace = vec![0.0; num_samples];
            let t_x = (t0_event * t0_event + x * x / (v_nmo * v_nmo)).sqrt();
            let sample = (t_x / dt).round() as usize;
            if sample < num_samples {
                trace[sample] = 1.0;
            }
            gather.push(trace);
        }

        let corrected = NmoCorrection::apply_correction(&gather, &offsets, v_nmo, dt);
        // After correction, all traces should have energy near the same sample
        let target_sample = (t0_event / dt).round() as usize;
        for trace in &corrected {
            // The maximum should be near the target
            let max_idx = trace
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                .map(|(i, _)| i)
                .unwrap();
            assert!((max_idx as isize - target_sample as isize).unsigned_abs() <= 2);
        }
    }

    #[test]
    fn test_nmo_stretch_factor() {
        let sf = NmoCorrection::stretch_factor(0.5, 0.0, 2000.0);
        assert!((sf - 1.0).abs() < 1e-6); // No stretch at zero offset
        let sf2 = NmoCorrection::stretch_factor(0.5, 1000.0, 2000.0);
        assert!(sf2 > 1.0); // Stretch at nonzero offset
    }

    #[test]
    fn test_nmo_stretch_mute() {
        let dt = 0.004;
        let mut corrected = vec![vec![1.0; 250]; 4];
        let offsets = vec![0.0, 500.0, 1000.0, 2000.0];
        NmoCorrection::apply_stretch_mute(&mut corrected, &offsets, 2000.0, dt, 1.5);
        // Far offset at shallow times should be muted
        // Sample 1 at offset 2000m: t0=0.004s, stretch = sqrt(0.004^2 + 2000^2/2000^2) / 0.004
        // = sqrt(0.000016+1)/0.004 ~ 1.0/0.004 = 250 >> 1.5 => muted
        assert_eq!(corrected[3][1], 0.0);
        // Near offset (0m) should not be muted
        assert_eq!(corrected[0][50], 1.0);
    }

    // ---- SemblanceAnalysis tests ----

    #[test]
    fn test_semblance_perfect_coherence() {
        // All traces identical => semblance = 1.0 at correct velocity
        let dt = 0.004;
        let num_samples = 100;
        let trace: Vec<f64> = (0..num_samples)
            .map(|i| if i == 50 { 1.0 } else { 0.0 })
            .collect();
        let gather = vec![trace.clone(); 4];
        let offsets = vec![0.0; 4]; // zero offset => no NMO needed

        let s = SemblanceAnalysis::semblance_point(&gather, &offsets, 50.0 * dt, 2000.0, dt, 1);
        assert!(s > 0.9);
    }

    #[test]
    fn test_semblance_incoherent() {
        // Random traces => low semblance
        let dt = 0.004;
        let num_samples = 100;
        let gather: Vec<Vec<f64>> = (0..8)
            .map(|seed| {
                (0..num_samples)
                    .map(|i| {
                        let x = (i as f64 * 0.73 + seed as f64 * 1.37).sin();
                        x
                    })
                    .collect()
            })
            .collect();
        let offsets: Vec<f64> = (0..8).map(|i| i as f64 * 100.0).collect();

        let s = SemblanceAnalysis::semblance_point(&gather, &offsets, 0.2, 2000.0, dt, 2);
        // Should be low but not necessarily zero
        assert!(s < 0.8);
    }

    #[test]
    fn test_velocity_spectrum() {
        let dt = 0.004;
        let num_samples = 100;
        let trace: Vec<f64> = (0..num_samples)
            .map(|i| if i == 50 { 1.0 } else { 0.0 })
            .collect();
        let gather = vec![trace.clone(); 4];
        let offsets = vec![0.0; 4];

        let t0_values = vec![0.2];
        let v_values = vec![1500.0, 2000.0, 2500.0];

        let spectrum =
            SemblanceAnalysis::velocity_spectrum(&gather, &offsets, dt, &t0_values, &v_values, 1);
        assert_eq!(spectrum.len(), 1);
        assert_eq!(spectrum[0].len(), 3);
    }

    #[test]
    fn test_pick_velocities() {
        let spectrum = vec![
            vec![0.3, 0.9, 0.5],
            vec![0.2, 0.4, 0.8],
        ];
        let t0_values = vec![0.2, 0.4];
        let v_values = vec![1500.0, 2000.0, 2500.0];

        let picks = SemblanceAnalysis::pick_velocities(&spectrum, &t0_values, &v_values);
        assert_eq!(picks.len(), 2);
        assert!((picks[0].1 - 2000.0).abs() < 1e-3); // Best at v=2000
        assert!((picks[1].1 - 2500.0).abs() < 1e-3); // Best at v=2500
    }

    // ---- DixInversion tests ----

    #[test]
    fn test_dix_single_layer() {
        let picks = vec![(0.5, 2000.0)];
        let v_ints = DixInversion::compute(&picks);
        assert_eq!(v_ints.len(), 1);
        assert!((v_ints[0] - 2000.0).abs() < 1e-3);
    }

    #[test]
    fn test_dix_two_layers() {
        // Layer 1: V=2000 m/s, thickness=500m, t=0.5s (TWT)
        // Layer 2: V=3000 m/s, thickness=600m, dt=0.4s (TWT)
        // V_rms at bottom of layer 2:
        // V_rms^2 = (2000^2 * 0.5 + 3000^2 * 0.4) / (0.5 + 0.4)
        //         = (2000000 + 3600000) / 0.9 = 6222222
        // V_rms = 2494.4
        let v_rms_2 = ((2000.0_f64.powi(2) * 0.5 + 3000.0_f64.powi(2) * 0.4) / 0.9).sqrt();
        let picks = vec![(0.5, 2000.0), (0.9, v_rms_2)];
        let v_ints = DixInversion::compute(&picks);
        assert_eq!(v_ints.len(), 2);
        assert!((v_ints[0] - 2000.0).abs() < 1.0);
        assert!((v_ints[1] - 3000.0).abs() < 1.0);
    }

    #[test]
    fn test_dix_interval_to_rms_roundtrip() {
        let intervals = vec![(2000.0, 0.5), (3000.0, 0.4), (4000.0, 0.3)];
        let rms = DixInversion::interval_to_rms(&intervals);
        assert_eq!(rms.len(), 3);
        // First RMS should equal first interval velocity
        assert!((rms[0] - 2000.0).abs() < 1e-3);
        // RMS should increase
        assert!(rms[1] > rms[0]);
        assert!(rms[2] > rms[1]);
    }

    #[test]
    fn test_dix_layer_thicknesses() {
        let picks = vec![(0.5, 2000.0)]; // TWT=0.5s, V_rms=2000 m/s
        let thicknesses = DixInversion::layer_thicknesses(&picks);
        assert_eq!(thicknesses.len(), 1);
        // h = V * TWT/2 = 2000 * 0.5/2 = 500m
        assert!((thicknesses[0] - 500.0).abs() < 1.0);
    }

    // ---- TomographicInversion tests ----

    #[test]
    fn test_tomographic_init() {
        let tomo = TomographicInversion::new(10, 5, 100.0, 3000.0);
        assert_eq!(tomo.num_cells, 50);
        assert_eq!(tomo.slowness.len(), 50);
        let v = tomo.velocity_model();
        assert!((v[0] - 3000.0).abs() < 1e-3);
    }

    #[test]
    fn test_tomographic_predicted_traveltime() {
        let tomo = TomographicInversion::new(10, 5, 100.0, 2000.0);
        let ray = RayPath {
            segments: vec![(0, 100.0), (1, 100.0)],
            observed_time: 0.1,
        };
        let t_pred = tomo.predicted_traveltime(&ray);
        // 100/2000 + 100/2000 = 0.1
        assert!((t_pred - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_tomographic_sirt() {
        // Create a model with a known anomaly and try to recover it
        let mut tomo = TomographicInversion::new(5, 5, 100.0, 2000.0);

        // Create rays that pass through cells with known traveltimes
        let rays = vec![
            RayPath {
                segments: vec![(0, 100.0), (1, 100.0), (2, 100.0)],
                observed_time: 0.15, // consistent with 2000 m/s
            },
            RayPath {
                segments: vec![(5, 100.0), (6, 100.0), (7, 100.0)],
                observed_time: 0.12, // slightly faster => higher velocity
            },
        ];

        let rms_before = tomo.rms_residual(&rays);
        tomo.sirt_iterate(&rays, 50, 0.5);
        let rms_after = tomo.rms_residual(&rays);
        assert!(rms_after <= rms_before + 1e-10); // Should not get worse
    }

    #[test]
    fn test_tomographic_velocity_grid() {
        let tomo = TomographicInversion::new(3, 2, 100.0, 2500.0);
        let grid = tomo.velocity_grid();
        assert_eq!(grid.len(), 2);
        assert_eq!(grid[0].len(), 3);
        assert!((grid[0][0] - 2500.0).abs() < 1e-3);
    }

    #[test]
    fn test_straight_ray() {
        let tomo = TomographicInversion::new(10, 10, 100.0, 2000.0);
        let segments = tomo.straight_ray(50.0, 50.0, 950.0, 950.0);
        assert!(!segments.is_empty());
        // Total length should approximate sqrt(900^2 + 900^2) ~ 1272.8 m
        let total: f64 = segments.iter().map(|&(_, dl)| dl).sum();
        assert!((total - 1272.8).abs() < 20.0);
    }

    // ---- DispersionCurveInversion tests ----

    #[test]
    fn test_dispersion_forward_model() {
        let inv = DispersionCurveInversion::new(vec![50.0, 100.0, f64::INFINITY], 300.0);
        let freqs = vec![5.0, 10.0, 20.0];
        let phase_vel = inv.forward_model(&freqs);
        assert_eq!(phase_vel.len(), 3);
        // All should be positive and roughly 0.92 * Vs
        for &v in &phase_vel {
            assert!(v > 0.0);
            assert!(v < 500.0); // Should be less than Vs for Rayleigh
        }
    }

    #[test]
    fn test_dispersion_inversion() {
        let mut inv = DispersionCurveInversion::new(vec![50.0, 100.0, f64::INFINITY], 200.0);
        // Target Vs ~ 300 m/s, Rayleigh ~ 0.92*300 = 276
        let observed = vec![(5.0, 270.0), (10.0, 275.0), (20.0, 278.0)];
        let misfit = inv.invert(&observed, 100, 0.5);
        // Check that Vs profile has been updated
        assert!(inv.vs_profile[0] > 200.0); // Should have increased from initial
    }

    // ---- VpVsRatioEstimator tests ----

    #[test]
    fn test_vp_vs_ratio_estimate() {
        // Vp/Vs = 1.73, so Ts = 1.73 * Tp
        // Ts - Tp = 0.73 * Tp => slope = 0.73 => Vp/Vs = 1.73
        let arrivals = vec![
            (1.0, 1.73),
            (2.0, 3.46),
            (3.0, 5.19),
            (4.0, 6.92),
        ];
        let (ratio, r_sq) = VpVsRatioEstimator::estimate(&arrivals).unwrap();
        assert!((ratio - 1.73).abs() < 0.01);
        assert!(r_sq > 0.99);
    }

    #[test]
    fn test_vp_vs_poissons() {
        let nu = VpVsRatioEstimator::poissons_ratio(1.732);
        assert!((nu - 0.25).abs() < 0.01);
    }

    #[test]
    fn test_wadati_origin_time() {
        // Origin at t=0, Vp=5000, Vs=2887 (Vp/Vs=1.732)
        // Station at distance d: Tp = d/Vp, Ts = d/Vs
        let arrivals: Vec<(f64, f64)> = (1..=5)
            .map(|i| {
                let d = i as f64 * 10000.0;
                (d / 5000.0, d / 2887.0)
            })
            .collect();
        let t0 = VpVsRatioEstimator::origin_time(&arrivals);
        assert!(t0.is_some());
        assert!(t0.unwrap().abs() < 0.1); // Should be near zero
    }

    #[test]
    fn test_vp_vs_insufficient_data() {
        let arrivals = vec![(1.0, 1.73)];
        assert!(VpVsRatioEstimator::estimate(&arrivals).is_none());
    }

    // ---- RefractedWaveInterpreter tests ----

    #[test]
    fn test_synthetic_picks_two_layer() {
        let picks = RefractedWaveInterpreter::synthetic_picks_two_layer(
            2000.0, 4000.0, 500.0,
            &[0.0, 500.0, 1000.0, 1500.0, 2000.0, 3000.0, 4000.0],
        );
        assert_eq!(picks.len(), 7);
        // At zero offset, traveltime = 0
        assert!(picks[0].1.abs() < 1e-6);
        // Traveltimes should increase with offset
        for i in 1..picks.len() {
            assert!(picks[i].1 > picks[i - 1].1);
        }
    }

    #[test]
    fn test_refraction_two_layer_interpretation() {
        let v1 = 2000.0;
        let v2 = 5000.0;
        let h = 300.0;

        // Generate picks with clear crossover
        let offsets: Vec<f64> = (1..=20).map(|i| i as f64 * 200.0).collect();
        let picks = RefractedWaveInterpreter::synthetic_picks_two_layer(v1, v2, h, &offsets);

        let result = RefractedWaveInterpreter::interpret_two_layer(&picks);
        assert!(result.is_some());
        let r = result.unwrap();
        // V1 should be close to 2000
        assert!((r.velocities[0] - v1).abs() / v1 < 0.15);
        // V2 should be close to 5000
        assert!((r.velocities[1] - v2).abs() / v2 < 0.15);
    }

    #[test]
    fn test_refraction_three_layer() {
        let result = RefractedWaveInterpreter::interpret_three_layer(
            2000.0,
            300.0,
            4000.0,
            &[(5000.0, 0.85), (6000.0, 0.95), (7000.0, 1.05), (8000.0, 1.15)],
        );
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.velocities.len(), 3);
        assert!(r.velocities[2] > r.velocities[1]);
    }

    #[test]
    fn test_refraction_insufficient_picks() {
        let picks = vec![(100.0, 0.05)];
        assert!(RefractedWaveInterpreter::interpret_two_layer(&picks).is_none());
    }

    // ---- Integration / roundtrip tests ----

    #[test]
    fn test_model_to_rms_to_dix_roundtrip() {
        let model = three_layer_model();

        // Get RMS velocities from the model
        let picks: Vec<(f64, f64)> = (0..2)
            .map(|i| (model.two_way_time(i), model.rms_velocity(i)))
            .collect();

        // Dix inversion to get interval velocities
        let v_ints = DixInversion::compute(&picks);
        assert_eq!(v_ints.len(), 2);
        // First interval velocity should match layer 0 Vp
        assert!((v_ints[0] - 1500.0).abs() < 1.0);
        // Second interval velocity should match layer 1 Vp
        assert!((v_ints[1] - 2500.0).abs() < 5.0);
    }

    #[test]
    fn test_nmo_with_known_model() {
        let model = simple_two_layer();
        let v_rms = model.rms_velocity(0);
        let t0 = model.two_way_time(0);

        // NMO at 1000m offset
        let t_x = (t0 * t0 + 1000.0 * 1000.0 / (v_rms * v_rms)).sqrt();
        let t0_recovered = NmoCorrection::correct(t_x, 1000.0, v_rms);
        assert!((t0_recovered - t0).abs() < 1e-6);
    }

    #[test]
    fn test_dix_empty() {
        assert!(DixInversion::compute(&[]).is_empty());
    }

    #[test]
    fn test_dix_thicknesses_two_layers() {
        // V_rms_1 = 2000, t_1 = 0.5 => h1 = 2000*0.5/2 = 500
        // V_rms_2 for layers (2000, 500m) and (3000, 600m):
        let v_rms_2 = ((2000.0_f64.powi(2) * 0.5 + 3000.0_f64.powi(2) * 0.4) / 0.9).sqrt();
        let picks = vec![(0.5, 2000.0), (0.9, v_rms_2)];
        let thicknesses = DixInversion::layer_thicknesses(&picks);
        assert_eq!(thicknesses.len(), 2);
        assert!((thicknesses[0] - 500.0).abs() < 1.0);
        assert!((thicknesses[1] - 600.0).abs() < 1.0);
    }
}
