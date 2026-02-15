//! Acoustic emission (AE) source localization for structural health monitoring and NDT.
//!
//! This module implements algorithms for determining the location of AE sources
//! in structures using arrays of piezoelectric sensors. Unlike the companion
//! `acoustic_emission_sensor` module (which focuses on hit detection and parametric
//! analysis), this module addresses the inverse problem: given arrival times at
//! multiple sensors, where did the emission originate?
//!
//! # Localization Methods
//!
//! - **1D linear**: Two sensors on a plate/beam, source position from TDOA
//! - **2D triangulation**: Three or more sensors, TDOA-based hyperbolic intersection
//! - **Least-squares iterative**: Gauss-Newton minimization of arrival time residuals
//! - **Grid search**: Coarse-to-fine spatial search for robustness
//!
//! # Arrival Time Picking
//!
//! - Threshold crossing (first break)
//! - AIC (Akaike Information Criterion) picker
//! - STA/LTA ratio picker
//! - Cross-correlation for precise time difference
//!
//! # Supporting Analysis
//!
//! - Wave velocity estimation (pencil lead break calibration, autocalibration)
//! - Attenuation modeling (geometric spreading + material absorption)
//! - Location uncertainty (residual error, confidence ellipse)
//! - Zonal location and event clustering
//! - Source characterization (b-value, Felicity ratio, Kaiser effect)
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_emission_localizer::{AeSensor, SensorArray, AeEvent};
//!
//! // Two sensors on a 1D beam, 1.0 m apart
//! let sensors = vec![
//!     AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
//!     AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
//! ];
//! let array = SensorArray::new(sensors, 5000.0); // 5000 m/s wave velocity
//!
//! // Source at x=0.3 → dt = (1.0 - 2*0.3) / 5000 = 0.00008 s
//! let loc = array.locate_1d(0.00008);
//! assert!((loc - 0.3).abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Core data structures
// ---------------------------------------------------------------------------

/// A single acoustic emission sensor with position and characteristics.
#[derive(Debug, Clone)]
pub struct AeSensor {
    /// X position in metres.
    pub x: f64,
    /// Y position in metres.
    pub y: f64,
    /// Z position in metres.
    pub z: f64,
    /// Sensitivity in dB_AE (typically negative, e.g. -26 dB ref 1 V/ubar).
    pub sensitivity_dbae: f64,
    /// Lower bound of usable frequency range in Hz.
    pub freq_low_hz: f64,
    /// Upper bound of usable frequency range in Hz.
    pub freq_high_hz: f64,
}

impl AeSensor {
    /// Create a new sensor at the given position with sensitivity and frequency range.
    pub fn new(x: f64, y: f64, z: f64, sensitivity_dbae: f64, freq_low_hz: f64, freq_high_hz: f64) -> Self {
        Self { x, y, z, sensitivity_dbae, freq_low_hz, freq_high_hz }
    }

    /// Distance to another sensor.
    pub fn distance_to(&self, other: &AeSensor) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Distance from sensor to a 3D point.
    pub fn distance_to_point(&self, px: f64, py: f64, pz: f64) -> f64 {
        let dx = self.x - px;
        let dy = self.y - py;
        let dz = self.z - pz;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// An acoustic emission event with parametric data and arrival times.
#[derive(Debug, Clone)]
pub struct AeEvent {
    /// Arrival time at each sensor (seconds). Index matches sensor index in the array.
    pub arrival_times: Vec<f64>,
    /// Peak amplitude in volts.
    pub amplitude: f64,
    /// Rise time from threshold crossing to peak (seconds).
    pub rise_time: f64,
    /// Total event duration (seconds).
    pub duration: f64,
    /// Number of threshold crossings.
    pub counts: u32,
    /// Signal energy (integral of squared voltage, V^2 * s).
    pub energy: f64,
}

impl AeEvent {
    /// Create a new AE event.
    pub fn new(
        arrival_times: Vec<f64>,
        amplitude: f64,
        rise_time: f64,
        duration: f64,
        counts: u32,
        energy: f64,
    ) -> Self {
        Self { arrival_times, amplitude, rise_time, duration, counts, energy }
    }

    /// Magnitude in dB_AE (20 * log10(amplitude / 1e-6)).
    pub fn magnitude_dbae(&self) -> f64 {
        if self.amplitude <= 0.0 {
            return f64::NEG_INFINITY;
        }
        20.0 * (self.amplitude / 1e-6).log10()
    }
}

/// A located AE source with position and quality metrics.
#[derive(Debug, Clone)]
pub struct LocatedSource {
    /// Estimated X position (metres).
    pub x: f64,
    /// Estimated Y position (metres).
    pub y: f64,
    /// Estimated Z position (metres).
    pub z: f64,
    /// RMS residual of arrival time fit (seconds).
    pub residual_rms: f64,
    /// Number of iterations used (for iterative methods).
    pub iterations: u32,
}

/// Confidence ellipse parameters (2D).
#[derive(Debug, Clone)]
pub struct ConfidenceEllipse {
    /// Centre X.
    pub cx: f64,
    /// Centre Y.
    pub cy: f64,
    /// Semi-major axis length (metres).
    pub semi_major: f64,
    /// Semi-minor axis length (metres).
    pub semi_minor: f64,
    /// Orientation angle of the major axis (radians from X axis).
    pub angle_rad: f64,
}

/// A predefined zone for zonal location.
#[derive(Debug, Clone)]
pub struct Zone {
    /// Zone identifier / name.
    pub name: String,
    /// Centre X.
    pub cx: f64,
    /// Centre Y.
    pub cy: f64,
    /// Radius (metres).
    pub radius: f64,
}

/// A cluster of spatially grouped AE events.
#[derive(Debug, Clone)]
pub struct EventCluster {
    /// Indices of events belonging to this cluster.
    pub event_indices: Vec<usize>,
    /// Centroid X.
    pub centroid_x: f64,
    /// Centroid Y.
    pub centroid_y: f64,
    /// Centroid Z.
    pub centroid_z: f64,
}

/// Attenuation model: A(r) = A0 * r^(-n) * exp(-alpha * r).
#[derive(Debug, Clone)]
pub struct AttenuationModel {
    /// Reference amplitude at unit distance.
    pub a0: f64,
    /// Geometric spreading exponent (0.5 for plate waves, 1.0 for bulk).
    pub n: f64,
    /// Material absorption coefficient (Np/m).
    pub alpha: f64,
}

impl AttenuationModel {
    /// Create a new attenuation model.
    pub fn new(a0: f64, n: f64, alpha: f64) -> Self {
        Self { a0, n, alpha }
    }

    /// Plate wave default: n=0.5, alpha from material.
    pub fn plate_wave(a0: f64, alpha: f64) -> Self {
        Self { a0, n: 0.5, alpha }
    }

    /// Predicted amplitude at distance r.
    pub fn amplitude_at(&self, r: f64) -> f64 {
        if r <= 0.0 {
            return self.a0;
        }
        self.a0 * r.powf(-self.n) * (-self.alpha * r).exp()
    }

    /// Fit attenuation model from (distance, amplitude) data pairs.
    /// Uses linearised least-squares: ln(A) = ln(A0) - n*ln(r) - alpha*r.
    pub fn fit(data: &[(f64, f64)]) -> Option<Self> {
        // Filter valid data points (positive r and A).
        let valid: Vec<(f64, f64)> = data
            .iter()
            .filter(|(r, a)| *r > 0.0 && *a > 0.0)
            .copied()
            .collect();
        if valid.len() < 3 {
            return None;
        }
        let n_pts = valid.len() as f64;

        // Solve: ln(A_i) = c0 - n*ln(r_i) - alpha*r_i
        // Let y = ln(A), x1 = ln(r), x2 = r
        // y = c0 - n*x1 - alpha*x2
        // Normal equations for [c0, n, alpha]:
        let mut sum_y = 0.0;
        let mut sum_x1 = 0.0;
        let mut sum_x2 = 0.0;
        let mut sum_x1x1 = 0.0;
        let mut sum_x2x2 = 0.0;
        let mut sum_x1x2 = 0.0;
        let mut sum_x1y = 0.0;
        let mut sum_x2y = 0.0;

        for &(r, a) in &valid {
            let y = a.ln();
            let x1 = r.ln();
            let x2 = r;
            sum_y += y;
            sum_x1 += x1;
            sum_x2 += x2;
            sum_x1x1 += x1 * x1;
            sum_x2x2 += x2 * x2;
            sum_x1x2 += x1 * x2;
            sum_x1y += x1 * y;
            sum_x2y += x2 * y;
        }

        // 3x3 system: A * [c0, n, alpha]^T = b
        // Row 0: n_pts*c0 - sum_x1*n - sum_x2*alpha = sum_y
        // Row 1: sum_x1*c0 - sum_x1x1*n - sum_x1x2*alpha = sum_x1y
        // Row 2: sum_x2*c0 - sum_x1x2*n - sum_x2x2*alpha = sum_x2y
        let a_mat = [
            [n_pts, -sum_x1, -sum_x2],
            [sum_x1, -sum_x1x1, -sum_x1x2],
            [sum_x2, -sum_x1x2, -sum_x2x2],
        ];
        let b_vec = [sum_y, sum_x1y, sum_x2y];

        let sol = solve_3x3(&a_mat, &b_vec)?;
        let c0 = sol[0];
        let n = sol[1];
        let alpha = sol[2];

        Some(Self {
            a0: c0.exp(),
            n,
            alpha,
        })
    }
}

// ---------------------------------------------------------------------------
// Sensor array and localization
// ---------------------------------------------------------------------------

/// An array of AE sensors for source localization.
#[derive(Debug, Clone)]
pub struct SensorArray {
    /// Sensors in the array.
    pub sensors: Vec<AeSensor>,
    /// Wave propagation velocity in the test medium (m/s).
    pub wave_velocity_ms: f64,
}

impl SensorArray {
    /// Create a new sensor array.
    pub fn new(sensors: Vec<AeSensor>, wave_velocity_ms: f64) -> Self {
        Self { sensors, wave_velocity_ms }
    }

    /// 1D linear location from two sensors along X axis.
    /// `dt` = t1 - t0 (arrival time difference in seconds).
    /// Returns source X coordinate.
    /// Formula: x = d/2 + v*dt/2 where d is sensor spacing.
    pub fn locate_1d(&self, dt: f64) -> f64 {
        assert!(self.sensors.len() >= 2, "Need at least 2 sensors for 1D location");
        let s0 = &self.sensors[0];
        let s1 = &self.sensors[1];
        let d = ((s1.x - s0.x).powi(2) + (s1.y - s0.y).powi(2) + (s1.z - s0.z).powi(2)).sqrt();
        let x_mid = (s0.x + s1.x) / 2.0;
        // Source is offset from midpoint along the line joining the two sensors.
        // Positive dt means wave arrived at s0 first → source closer to s0.
        // x = midpoint - v*dt/2 (referenced to s0 side).
        // Using the standard formula: x = d/2 + v*dt/2 (from s0).
        s0.x + d / 2.0 + self.wave_velocity_ms * dt / 2.0
    }

    /// 2D triangulation using TDOA from 3+ sensors.
    /// Uses the hyperbolic linearisation method.
    /// `arrival_times`: time of arrival at each sensor (seconds).
    pub fn locate_2d_tdoa(&self, arrival_times: &[f64]) -> Option<LocatedSource> {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 3 {
            return None;
        }

        // Use sensor 0 as reference. Form TDOA equations.
        // |r - r_i| - |r - r_0| = v * (t_i - t_0)
        // Linearise: difference of squared distances.
        // For sensors i=1..n-1 relative to sensor 0:
        // 2*(x_i - x_0)*x + 2*(y_i - y_0)*y = (x_i^2 + y_i^2) - (x_0^2 + y_0^2) - d_i^2 + d_0^2
        // where d_i = v * t_i (pseudorange).
        let v = self.wave_velocity_ms;
        let x0 = self.sensors[0].x;
        let y0 = self.sensors[0].y;
        let t0 = arrival_times[0];

        // Iterative approach: start with linearised TDOA, then refine.
        // Linearised TDOA: form Ax = b from TDOA pairs.
        let m = n - 1;
        let mut a_mat = vec![vec![0.0; 2]; m];
        let mut b_vec = vec![0.0; m];

        for i in 0..m {
            let si = &self.sensors[i + 1];
            let dt_i = arrival_times[i + 1] - t0;
            let range_diff = v * dt_i;

            a_mat[i][0] = 2.0 * (si.x - x0);
            a_mat[i][1] = 2.0 * (si.y - y0);
            b_vec[i] = (si.x * si.x + si.y * si.y) - (x0 * x0 + y0 * y0)
                - range_diff * range_diff;
            // Add the cross term with d0: need iterative refinement.
        }

        // Solve overdetermined system via normal equations: (A^T A) x = A^T b.
        let solution = solve_least_squares_2d(&a_mat, &b_vec)?;
        let (sx, sy) = (solution[0], solution[1]);

        // Compute residual.
        let residual = self.compute_residual_2d(sx, sy, arrival_times);

        // Refine with Gauss-Newton.
        let refined = self.gauss_newton_2d(sx, sy, arrival_times, 50);
        Some(refined)
    }

    /// Least-squares iterative localization (Gauss-Newton) in 2D.
    fn gauss_newton_2d(&self, x0: f64, y0: f64, arrival_times: &[f64], max_iter: u32) -> LocatedSource {
        let n = arrival_times.len().min(self.sensors.len());
        let v = self.wave_velocity_ms;
        let mut sx = x0;
        let mut sy = y0;
        let mut iterations = 0u32;

        for _iter in 0..max_iter {
            iterations = _iter + 1;

            // Estimate t_offset as the minimum (t_i - d_i/v).
            let mut t_offset = f64::MAX;
            for i in 0..n {
                let di = self.sensors[i].distance_to_point(sx, sy, 0.0);
                let ti = arrival_times[i] - di / v;
                if ti < t_offset {
                    t_offset = ti;
                }
            }

            // Jacobian J[i] = [d(r_i)/dx, d(r_i)/dy] / v
            // Residual: (d_i / v + t_offset) - t_i
            let mut jtj = [[0.0; 2]; 2];
            let mut jtr = [0.0; 2];
            let mut rms = 0.0;

            for i in 0..n {
                let dx = sx - self.sensors[i].x;
                let dy = sy - self.sensors[i].y;
                let di = (dx * dx + dy * dy).sqrt().max(1e-12);
                let predicted_t = di / v + t_offset;
                let ri = predicted_t - arrival_times[i];
                rms += ri * ri;

                let jx = dx / (di * v);
                let jy = dy / (di * v);

                jtj[0][0] += jx * jx;
                jtj[0][1] += jx * jy;
                jtj[1][0] += jy * jx;
                jtj[1][1] += jy * jy;
                jtr[0] += jx * ri;
                jtr[1] += jy * ri;
            }

            // Solve 2x2: (J^T J) delta = -J^T r
            let det = jtj[0][0] * jtj[1][1] - jtj[0][1] * jtj[1][0];
            if det.abs() < 1e-30 {
                break;
            }
            let delta_x = -(jtj[1][1] * jtr[0] - jtj[0][1] * jtr[1]) / det;
            let delta_y = -(jtj[0][0] * jtr[1] - jtj[1][0] * jtr[0]) / det;

            sx += delta_x;
            sy += delta_y;

            if delta_x.abs() < 1e-9 && delta_y.abs() < 1e-9 {
                break;
            }
        }

        let residual = self.compute_residual_2d(sx, sy, arrival_times);
        LocatedSource {
            x: sx,
            y: sy,
            z: 0.0,
            residual_rms: residual,
            iterations,
        }
    }

    /// 3D least-squares localization using Gauss-Newton on TDOA residuals.
    /// Uses differences relative to sensor 0 to eliminate the unknown origin time.
    pub fn locate_3d_iterative(
        &self,
        arrival_times: &[f64],
        x0: f64,
        y0: f64,
        z0: f64,
        max_iter: u32,
    ) -> Option<LocatedSource> {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 4 {
            return None;
        }
        let v = self.wave_velocity_ms;
        let mut sx = x0;
        let mut sy = y0;
        let mut sz = z0;
        let mut iterations = 0u32;

        for _iter in 0..max_iter {
            iterations = _iter + 1;

            // TDOA residuals: r_i = (d_i - d_0)/v - (t_i - t_0), for i=1..n-1
            let d0 = self.sensors[0].distance_to_point(sx, sy, sz).max(1e-12);
            let dx0 = sx - self.sensors[0].x;
            let dy0 = sy - self.sensors[0].y;
            let dz0 = sz - self.sensors[0].z;

            let m = n - 1;
            let mut jtj = [[0.0; 3]; 3];
            let mut jtr = [0.0; 3];

            for i in 0..m {
                let si = &self.sensors[i + 1];
                let dxi = sx - si.x;
                let dyi = sy - si.y;
                let dzi = sz - si.z;
                let di = (dxi * dxi + dyi * dyi + dzi * dzi).sqrt().max(1e-12);

                let predicted_dt = (di - d0) / v;
                let measured_dt = arrival_times[i + 1] - arrival_times[0];
                let ri = predicted_dt - measured_dt;

                // Jacobian: d(residual_i)/d(sx) = (dxi/di - dx0/d0) / v
                let jx = (dxi / di - dx0 / d0) / v;
                let jy = (dyi / di - dy0 / d0) / v;
                let jz = (dzi / di - dz0 / d0) / v;

                let jvec = [jx, jy, jz];
                for a in 0..3 {
                    for b in 0..3 {
                        jtj[a][b] += jvec[a] * jvec[b];
                    }
                    jtr[a] += jvec[a] * ri;
                }
            }

            let sol = solve_3x3(&jtj, &[-jtr[0], -jtr[1], -jtr[2]]);
            match sol {
                Some(delta) => {
                    sx += delta[0];
                    sy += delta[1];
                    sz += delta[2];
                    if delta[0].abs() < 1e-9 && delta[1].abs() < 1e-9 && delta[2].abs() < 1e-9 {
                        break;
                    }
                }
                None => break,
            }
        }

        let residual = self.compute_residual_3d(sx, sy, sz, arrival_times);
        Some(LocatedSource {
            x: sx,
            y: sy,
            z: sz,
            residual_rms: residual,
            iterations,
        })
    }

    /// Grid search localization in 2D: coarse grid followed by fine refinement.
    /// `bounds`: (x_min, x_max, y_min, y_max).
    /// `coarse_steps`: number of grid divisions per axis for coarse search.
    /// `fine_steps`: number of divisions for fine refinement.
    pub fn locate_grid_search(
        &self,
        arrival_times: &[f64],
        bounds: (f64, f64, f64, f64),
        coarse_steps: usize,
        fine_steps: usize,
    ) -> Option<LocatedSource> {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 2 {
            return None;
        }

        let (x_min, x_max, y_min, y_max) = bounds;

        // Coarse search.
        let (best_x, best_y) =
            self.grid_search_pass(arrival_times, x_min, x_max, y_min, y_max, coarse_steps);

        // Fine search around best coarse point.
        let dx = (x_max - x_min) / coarse_steps as f64;
        let dy = (y_max - y_min) / coarse_steps as f64;
        let (fine_x, fine_y) = self.grid_search_pass(
            arrival_times,
            best_x - dx,
            best_x + dx,
            best_y - dy,
            best_y + dy,
            fine_steps,
        );

        let residual = self.compute_residual_2d(fine_x, fine_y, arrival_times);
        Some(LocatedSource {
            x: fine_x,
            y: fine_y,
            z: 0.0,
            residual_rms: residual,
            iterations: (coarse_steps * coarse_steps + fine_steps * fine_steps) as u32,
        })
    }

    fn grid_search_pass(
        &self,
        arrival_times: &[f64],
        x_min: f64,
        x_max: f64,
        y_min: f64,
        y_max: f64,
        steps: usize,
    ) -> (f64, f64) {
        let n = arrival_times.len().min(self.sensors.len());
        let v = self.wave_velocity_ms;
        let mut best_cost = f64::MAX;
        let mut best_x = (x_min + x_max) / 2.0;
        let mut best_y = (y_min + y_max) / 2.0;

        for ix in 0..=steps {
            let x = x_min + (x_max - x_min) * ix as f64 / steps as f64;
            for iy in 0..=steps {
                let y = y_min + (y_max - y_min) * iy as f64 / steps as f64;

                // For TDOA: use differences relative to sensor 0.
                let mut cost = 0.0;
                let d0 = self.sensors[0].distance_to_point(x, y, 0.0);
                for i in 1..n {
                    let di = self.sensors[i].distance_to_point(x, y, 0.0);
                    let predicted_dt = (di - d0) / v;
                    let measured_dt = arrival_times[i] - arrival_times[0];
                    let err = predicted_dt - measured_dt;
                    cost += err * err;
                }

                if cost < best_cost {
                    best_cost = cost;
                    best_x = x;
                    best_y = y;
                }
            }
        }
        (best_x, best_y)
    }

    /// Compute RMS residual for a 2D source position.
    pub fn compute_residual_2d(&self, sx: f64, sy: f64, arrival_times: &[f64]) -> f64 {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 2 {
            return 0.0;
        }
        let v = self.wave_velocity_ms;

        // TDOA residuals relative to sensor 0.
        let d0 = self.sensors[0].distance_to_point(sx, sy, 0.0);
        let mut sum_sq = 0.0;
        let count = n - 1;
        for i in 1..n {
            let di = self.sensors[i].distance_to_point(sx, sy, 0.0);
            let predicted_dt = (di - d0) / v;
            let measured_dt = arrival_times[i] - arrival_times[0];
            let err = predicted_dt - measured_dt;
            sum_sq += err * err;
        }
        (sum_sq / count as f64).sqrt()
    }

    /// Compute RMS residual for a 3D source position.
    pub fn compute_residual_3d(&self, sx: f64, sy: f64, sz: f64, arrival_times: &[f64]) -> f64 {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 2 {
            return 0.0;
        }
        let v = self.wave_velocity_ms;
        let d0 = self.sensors[0].distance_to_point(sx, sy, sz);
        let mut sum_sq = 0.0;
        let count = n - 1;
        for i in 1..n {
            let di = self.sensors[i].distance_to_point(sx, sy, sz);
            let predicted_dt = (di - d0) / v;
            let measured_dt = arrival_times[i] - arrival_times[0];
            let err = predicted_dt - measured_dt;
            sum_sq += err * err;
        }
        (sum_sq / count as f64).sqrt()
    }

    /// Estimate wave velocity from a pencil lead break (Hsu-Nielsen source) at known position.
    /// Given known source position and measured arrival times, solve for velocity.
    pub fn estimate_velocity_plb(
        &self,
        source_x: f64,
        source_y: f64,
        source_z: f64,
        arrival_times: &[f64],
    ) -> f64 {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 2 {
            return self.wave_velocity_ms;
        }

        // Use TDOA pairs to estimate velocity: v = (d_i - d_j) / (t_i - t_j).
        let mut sum_v = 0.0;
        let mut count = 0;
        for i in 0..n {
            let di = self.sensors[i].distance_to_point(source_x, source_y, source_z);
            for j in (i + 1)..n {
                let dj = self.sensors[j].distance_to_point(source_x, source_y, source_z);
                let dt = arrival_times[j] - arrival_times[i];
                if dt.abs() > 1e-12 {
                    let v_est = (dj - di) / dt;
                    if v_est > 0.0 {
                        sum_v += v_est;
                        count += 1;
                    }
                }
            }
        }
        if count > 0 {
            sum_v / count as f64
        } else {
            self.wave_velocity_ms
        }
    }

    /// Autocalibrate wave velocity from multiple events with known locations.
    /// Returns the mean velocity estimate.
    pub fn autocalibrate_velocity(
        &self,
        events: &[(f64, f64, f64, Vec<f64>)], // (x, y, z, arrival_times)
    ) -> f64 {
        let mut sum_v = 0.0;
        let mut count = 0;
        for (sx, sy, sz, times) in events {
            let v = self.estimate_velocity_plb(*sx, *sy, *sz, times);
            if v.is_finite() && v > 0.0 {
                sum_v += v;
                count += 1;
            }
        }
        if count > 0 {
            sum_v / count as f64
        } else {
            self.wave_velocity_ms
        }
    }

    /// Confidence ellipse for a 2D source location estimate.
    /// Uses linearised error propagation from arrival time uncertainties.
    /// `sigma_t`: standard deviation of arrival time measurement (seconds).
    pub fn confidence_ellipse(
        &self,
        sx: f64,
        sy: f64,
        arrival_times: &[f64],
        sigma_t: f64,
    ) -> Option<ConfidenceEllipse> {
        let n = arrival_times.len().min(self.sensors.len());
        if n < 3 {
            return None;
        }
        let v = self.wave_velocity_ms;

        // Build Jacobian of TDOA residuals w.r.t. source position.
        let d0 = self.sensors[0].distance_to_point(sx, sy, 0.0);
        let m = n - 1;
        let mut jx = vec![0.0; m];
        let mut jy = vec![0.0; m];

        for i in 0..m {
            let si = &self.sensors[i + 1];
            let di = si.distance_to_point(sx, sy, 0.0).max(1e-12);
            // d(TDOA_i)/dx = (sx - si.x)/(di*v) - (sx - s0.x)/(d0*v)
            jx[i] = (sx - si.x) / (di * v) - (sx - self.sensors[0].x) / (d0.max(1e-12) * v);
            jy[i] = (sy - si.y) / (di * v) - (sy - self.sensors[0].y) / (d0.max(1e-12) * v);
        }

        // J^T J
        let mut jtj = [[0.0; 2]; 2];
        for i in 0..m {
            jtj[0][0] += jx[i] * jx[i];
            jtj[0][1] += jx[i] * jy[i];
            jtj[1][0] += jy[i] * jx[i];
            jtj[1][1] += jy[i] * jy[i];
        }

        // Covariance = sigma_t^2 * (J^T J)^{-1}
        let det = jtj[0][0] * jtj[1][1] - jtj[0][1] * jtj[1][0];
        if det.abs() < 1e-30 {
            return None;
        }
        let cov_xx = sigma_t * sigma_t * jtj[1][1] / det;
        let cov_yy = sigma_t * sigma_t * jtj[0][0] / det;
        let cov_xy = -sigma_t * sigma_t * jtj[0][1] / det;

        // Eigenvalues of covariance matrix → ellipse axes.
        let trace = cov_xx + cov_yy;
        let det_cov = cov_xx * cov_yy - cov_xy * cov_xy;
        let discriminant = (trace * trace / 4.0 - det_cov).max(0.0);
        let lambda1 = trace / 2.0 + discriminant.sqrt();
        let lambda2 = trace / 2.0 - discriminant.sqrt();

        // 95% confidence: chi-squared(2, 0.95) ≈ 5.991
        let scale = 5.991_f64.sqrt();
        let semi_major = scale * lambda1.abs().sqrt();
        let semi_minor = scale * lambda2.abs().sqrt();

        // Angle of major axis.
        let angle = if cov_xy.abs() < 1e-30 {
            if cov_xx >= cov_yy { 0.0 } else { PI / 2.0 }
        } else {
            0.5 * (2.0 * cov_xy).atan2(cov_xx - cov_yy)
        };

        Some(ConfidenceEllipse {
            cx: sx,
            cy: sy,
            semi_major,
            semi_minor,
            angle_rad: angle,
        })
    }

    /// Zonal location: assign source to the zone of the first-arriving sensor.
    pub fn zonal_locate(&self, arrival_times: &[f64], zones: &[Zone]) -> Option<String> {
        let n = arrival_times.len().min(self.sensors.len());
        if n == 0 || zones.is_empty() {
            return None;
        }

        // Find first-hit sensor.
        let mut min_idx = 0;
        let mut min_t = arrival_times[0];
        for i in 1..n {
            if arrival_times[i] < min_t {
                min_t = arrival_times[i];
                min_idx = i;
            }
        }

        let sensor = &self.sensors[min_idx];
        // Find closest zone to that sensor.
        let mut best_zone: Option<&Zone> = None;
        let mut best_dist = f64::MAX;
        for zone in zones {
            let dx = sensor.x - zone.cx;
            let dy = sensor.y - zone.cy;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist <= zone.radius && dist < best_dist {
                best_dist = dist;
                best_zone = Some(zone);
            }
        }

        best_zone.map(|z| z.name.clone())
    }
}

// ---------------------------------------------------------------------------
// Arrival time picking
// ---------------------------------------------------------------------------

/// Pick arrival time using threshold crossing (first break).
/// Returns the sample index where signal first exceeds threshold.
pub fn pick_threshold(signal: &[f64], threshold: f64) -> Option<usize> {
    signal.iter().position(|&s| s.abs() >= threshold)
}

/// AIC (Akaike Information Criterion) picker.
/// AIC(k) = k * log(var(x[0..k])) + (N-k) * log(var(x[k..N])).
/// Returns the sample index that minimizes AIC.
pub fn pick_aic(signal: &[f64]) -> Option<usize> {
    let n = signal.len();
    if n < 4 {
        return None;
    }

    // Precompute cumulative sum and sum-of-squares.
    let mut cum_sum = vec![0.0; n + 1];
    let mut cum_sq = vec![0.0; n + 1];
    for i in 0..n {
        cum_sum[i + 1] = cum_sum[i] + signal[i];
        cum_sq[i + 1] = cum_sq[i] + signal[i] * signal[i];
    }

    let variance = |start: usize, end: usize| -> f64 {
        let count = (end - start) as f64;
        if count < 2.0 {
            return 1e-30;
        }
        let s = cum_sum[end] - cum_sum[start];
        let sq = cum_sq[end] - cum_sq[start];
        let mean = s / count;
        (sq / count - mean * mean).max(1e-30)
    };

    let mut min_aic = f64::MAX;
    let mut min_idx = 0;

    // Avoid edge effects: evaluate from index 2 to n-2.
    for k in 2..(n - 2) {
        let var_left = variance(0, k);
        let var_right = variance(k, n);
        let aic = k as f64 * var_left.ln() + (n - k) as f64 * var_right.ln();
        if aic < min_aic {
            min_aic = aic;
            min_idx = k;
        }
    }
    Some(min_idx)
}

/// STA/LTA ratio picker.
/// `sta_len`: short-term average window length (samples).
/// `lta_len`: long-term average window length (samples).
/// `threshold`: STA/LTA ratio threshold for detection.
/// Returns the sample index where the ratio first exceeds the threshold.
pub fn pick_sta_lta(signal: &[f64], sta_len: usize, lta_len: usize, threshold: f64) -> Option<usize> {
    let n = signal.len();
    if n < sta_len + lta_len || sta_len == 0 || lta_len == 0 {
        return None;
    }

    // Compute absolute values / energy.
    let energy: Vec<f64> = signal.iter().map(|s| s * s).collect();

    for i in lta_len..n.saturating_sub(sta_len) {
        let lta: f64 = energy[i.saturating_sub(lta_len)..i].iter().sum::<f64>() / lta_len as f64;
        let sta: f64 = energy[i..((i + sta_len).min(n))].iter().sum::<f64>() / sta_len as f64;

        if lta > 1e-30 && sta / lta >= threshold {
            return Some(i);
        }
    }
    None
}

/// Cross-correlation between two sensor waveforms to find precise time difference.
/// Returns the lag (in samples) that maximizes the normalized cross-correlation.
/// Positive lag means signal_b is delayed relative to signal_a.
pub fn cross_correlate_dt(signal_a: &[f64], signal_b: &[f64], max_lag: usize) -> (i64, f64) {
    let n = signal_a.len().min(signal_b.len());
    if n == 0 {
        return (0, 0.0);
    }

    // Energy of both signals for normalization.
    let energy_a: f64 = signal_a[..n].iter().map(|s| s * s).sum();
    let energy_b: f64 = signal_b[..n].iter().map(|s| s * s).sum();
    let norm = (energy_a * energy_b).sqrt().max(1e-30);

    let mut best_lag: i64 = 0;
    let mut best_corr = f64::NEG_INFINITY;

    let max_l = max_lag.min(n - 1);
    for lag_i in 0..=(2 * max_l) {
        let lag = lag_i as i64 - max_l as i64;
        let mut sum = 0.0;
        for j in 0..n {
            let idx_b = j as i64 + lag;
            if idx_b >= 0 && (idx_b as usize) < n {
                sum += signal_a[j] * signal_b[idx_b as usize];
            }
        }
        let corr = sum / norm;
        if corr > best_corr {
            best_corr = corr;
            best_lag = lag;
        }
    }
    (best_lag, best_corr)
}

// ---------------------------------------------------------------------------
// Event clustering
// ---------------------------------------------------------------------------

/// Cluster located AE events by spatial proximity.
/// `locations`: (x, y, z) positions of events.
/// `distance_threshold`: maximum distance between events in the same cluster.
/// Returns vector of clusters.
pub fn cluster_events(locations: &[(f64, f64, f64)], distance_threshold: f64) -> Vec<EventCluster> {
    let n = locations.len();
    if n == 0 {
        return vec![];
    }

    let mut assigned = vec![false; n];
    let mut clusters = Vec::new();

    for i in 0..n {
        if assigned[i] {
            continue;
        }
        assigned[i] = true;
        let mut indices = vec![i];

        // Expand cluster: find all events within threshold distance.
        let mut k = 0;
        while k < indices.len() {
            let ci = indices[k];
            let (cx, cy, cz) = locations[ci];
            for j in 0..n {
                if assigned[j] {
                    continue;
                }
                let (jx, jy, jz) = locations[j];
                let dx = cx - jx;
                let dy = cy - jy;
                let dz = cz - jz;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                if dist <= distance_threshold {
                    assigned[j] = true;
                    indices.push(j);
                }
            }
            k += 1;
        }

        // Compute centroid.
        let count = indices.len() as f64;
        let (mut sx, mut sy, mut sz) = (0.0, 0.0, 0.0);
        for &idx in &indices {
            sx += locations[idx].0;
            sy += locations[idx].1;
            sz += locations[idx].2;
        }

        clusters.push(EventCluster {
            event_indices: indices,
            centroid_x: sx / count,
            centroid_y: sy / count,
            centroid_z: sz / count,
        });
    }
    clusters
}

// ---------------------------------------------------------------------------
// Source characterization
// ---------------------------------------------------------------------------

/// Compute b-value from AE amplitude distribution (Gutenberg-Richter law).
/// log10(N) = a - b * M, where M = magnitude (dB_AE / 20).
/// `amplitudes_dbae`: event amplitudes in dB_AE.
/// Returns (b_value, a_value, r_squared).
pub fn b_value_analysis(amplitudes_dbae: &[f64]) -> Option<(f64, f64, f64)> {
    if amplitudes_dbae.len() < 5 {
        return None;
    }

    // Sort amplitudes and compute cumulative distribution.
    let mut sorted: Vec<f64> = amplitudes_dbae.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

    let n = sorted.len();
    // Bin into magnitude classes.
    let m_min = sorted[0];
    let m_max = sorted[n - 1];
    let range = m_max - m_min;
    if range < 1.0 {
        return None;
    }

    let n_bins = 20usize;
    let bin_width = range / n_bins as f64;
    let mut bins: Vec<(f64, f64)> = Vec::new(); // (M, log10(N_cumulative))

    for b in 0..n_bins {
        let m_threshold = m_min + b as f64 * bin_width;
        let count = sorted.iter().filter(|&&a| a >= m_threshold).count();
        if count > 0 {
            bins.push((m_threshold / 20.0, (count as f64).log10()));
        }
    }

    if bins.len() < 3 {
        return None;
    }

    // Linear regression: log10(N) = a - b*M.
    let nb = bins.len() as f64;
    let sum_m: f64 = bins.iter().map(|(m, _)| m).sum();
    let sum_ln: f64 = bins.iter().map(|(_, ln)| ln).sum();
    let sum_mm: f64 = bins.iter().map(|(m, _)| m * m).sum();
    let sum_mln: f64 = bins.iter().map(|(m, ln)| m * ln).sum();

    let denom = nb * sum_mm - sum_m * sum_m;
    if denom.abs() < 1e-30 {
        return None;
    }

    let b_val = -(nb * sum_mln - sum_m * sum_ln) / denom;
    let a_val = (sum_ln + b_val * sum_m) / nb;

    // R-squared.
    let mean_ln = sum_ln / nb;
    let ss_tot: f64 = bins.iter().map(|(_, ln)| (ln - mean_ln).powi(2)).sum();
    let ss_res: f64 = bins
        .iter()
        .map(|(m, ln)| {
            let predicted = a_val - b_val * m;
            (ln - predicted).powi(2)
        })
        .sum();
    let r_sq = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

    Some((b_val, a_val, r_sq))
}

/// Compute Felicity ratio: load at AE onset / previous maximum load.
/// `onset_load`: load level when AE activity resumes on reloading.
/// `previous_max_load`: maximum load from the previous loading cycle.
pub fn felicity_ratio(onset_load: f64, previous_max_load: f64) -> f64 {
    if previous_max_load.abs() < 1e-30 {
        return 1.0;
    }
    onset_load / previous_max_load
}

/// Detect Kaiser effect: AE resumes only when previous max load is exceeded.
/// Returns true if Kaiser effect is present (felicity ratio >= threshold, typically ~1.0).
/// `load_history`: sequence of (load, ae_count) measurements.
/// `threshold`: Felicity ratio threshold (typically 0.95 to 1.05).
pub fn detect_kaiser_effect(load_history: &[(f64, u32)], threshold: f64) -> bool {
    if load_history.len() < 3 {
        return false;
    }

    // Find previous max load (where AE was active).
    let mut prev_max_load = 0.0f64;
    let mut onset_load = None;
    let mut passed_peak = false;

    for &(load, ae_count) in load_history {
        if !passed_peak {
            if load > prev_max_load && ae_count > 0 {
                prev_max_load = load;
            }
            if load < prev_max_load * 0.8 {
                passed_peak = true;
            }
        } else if onset_load.is_none() && ae_count > 0 {
            onset_load = Some(load);
        }
    }

    match onset_load {
        Some(ol) => felicity_ratio(ol, prev_max_load) >= threshold,
        None => true, // No AE on reload → Kaiser effect present.
    }
}

// ---------------------------------------------------------------------------
// Helper: solve linear systems
// ---------------------------------------------------------------------------

/// Solve a 3x3 linear system Ax = b via Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-30 {
        return None;
    }

    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        / det;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        / det;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        / det;

    Some([x0, x1, x2])
}

/// Solve 2D overdetermined least-squares: (A^T A) x = A^T b.
fn solve_least_squares_2d(a: &[Vec<f64>], b: &[f64]) -> Option<[f64; 2]> {
    let m = a.len();
    if m < 2 {
        return None;
    }

    let mut ata = [[0.0; 2]; 2];
    let mut atb = [0.0; 2];

    for i in 0..m {
        for j in 0..2 {
            for k in 0..2 {
                ata[j][k] += a[i][j] * a[i][k];
            }
            atb[j] += a[i][j] * b[i];
        }
    }

    let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
    if det.abs() < 1e-30 {
        return None;
    }

    let x0 = (ata[1][1] * atb[0] - ata[0][1] * atb[1]) / det;
    let x1 = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;
    Some([x0, x1])
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- AeSensor tests --

    #[test]
    fn test_sensor_creation() {
        let s = AeSensor::new(1.0, 2.0, 3.0, -26.0, 100e3, 1e6);
        assert_eq!(s.x, 1.0);
        assert_eq!(s.y, 2.0);
        assert_eq!(s.z, 3.0);
        assert_eq!(s.sensitivity_dbae, -26.0);
        assert_eq!(s.freq_low_hz, 100e3);
        assert_eq!(s.freq_high_hz, 1e6);
    }

    #[test]
    fn test_sensor_distance() {
        let s1 = AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6);
        let s2 = AeSensor::new(3.0, 4.0, 0.0, -20.0, 100e3, 1e6);
        assert!(approx_eq(s1.distance_to(&s2), 5.0, 1e-10));
    }

    #[test]
    fn test_sensor_distance_to_point() {
        let s = AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6);
        assert!(approx_eq(s.distance_to_point(1.0, 0.0, 0.0), 1.0, 1e-10));
        assert!(approx_eq(s.distance_to_point(3.0, 4.0, 0.0), 5.0, 1e-10));
    }

    #[test]
    fn test_sensor_distance_3d() {
        let s1 = AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6);
        let s2 = AeSensor::new(1.0, 2.0, 2.0, -20.0, 100e3, 1e6);
        assert!(approx_eq(s1.distance_to(&s2), 3.0, 1e-10));
    }

    // -- AeEvent tests --

    #[test]
    fn test_event_magnitude() {
        // 1 V peak → 20*log10(1V / 1uV) = 120 dB_AE
        let event = AeEvent::new(vec![0.0], 1.0, 1e-5, 1e-4, 10, 1e-6);
        assert!(approx_eq(event.magnitude_dbae(), 120.0, 0.01));
    }

    #[test]
    fn test_event_magnitude_low() {
        // 0.001 V → 20*log10(0.001/1e-6) = 20*log10(1000) = 60 dB_AE
        let event = AeEvent::new(vec![0.0], 0.001, 1e-5, 1e-4, 5, 1e-8);
        assert!(approx_eq(event.magnitude_dbae(), 60.0, 0.01));
    }

    #[test]
    fn test_event_magnitude_zero() {
        let event = AeEvent::new(vec![0.0], 0.0, 0.0, 0.0, 0, 0.0);
        assert!(event.magnitude_dbae().is_infinite());
    }

    // -- 1D location --

    #[test]
    fn test_1d_location_midpoint() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let array = SensorArray::new(sensors, 5000.0);
        // dt = 0 → source at midpoint (0.5)
        let loc = array.locate_1d(0.0);
        assert!(approx_eq(loc, 0.5, 1e-6));
    }

    #[test]
    fn test_1d_location_near_s0() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let array = SensorArray::new(sensors, 5000.0);
        // Source at x=0.2: dt = (2*0.2 - 1.0)/5000 = -0.6/5000 = -0.00012
        let loc = array.locate_1d(-0.00012);
        assert!(approx_eq(loc, 0.2, 0.01));
    }

    #[test]
    fn test_1d_location_near_s1() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let array = SensorArray::new(sensors, 5000.0);
        // Source at x=0.8: dt = (2*0.8 - 1.0)/5000 = 0.6/5000 = 0.00012
        let loc = array.locate_1d(0.00012);
        assert!(approx_eq(loc, 0.8, 0.01));
    }

    // -- Grid search --

    #[test]
    fn test_grid_search_known_source() {
        // 3 sensors in a triangle, source at (0.3, 0.4).
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 0.866, 0.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let array = SensorArray::new(sensors.clone(), v);

        let sx = 0.3;
        let sy = 0.4;
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(sx, sy, 0.0) / v)
            .collect();

        let loc = array
            .locate_grid_search(&times, (0.0, 1.0, 0.0, 1.0), 50, 50)
            .unwrap();
        assert!(approx_eq(loc.x, sx, 0.05));
        assert!(approx_eq(loc.y, sy, 0.05));
    }

    // -- 2D TDOA --

    #[test]
    fn test_2d_tdoa_triangulation() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 0.866, 0.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let array = SensorArray::new(sensors.clone(), v);

        let sx = 0.4;
        let sy = 0.3;
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(sx, sy, 0.0) / v)
            .collect();

        let loc = array.locate_2d_tdoa(&times).unwrap();
        assert!(approx_eq(loc.x, sx, 0.05));
        assert!(approx_eq(loc.y, sy, 0.05));
    }

    #[test]
    fn test_2d_tdoa_centre() {
        // Source at array centroid.
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 0.866, 0.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let cx = 0.5;
        let cy = 0.289;
        let array = SensorArray::new(sensors.clone(), v);
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(cx, cy, 0.0) / v)
            .collect();
        let loc = array.locate_2d_tdoa(&times).unwrap();
        assert!(approx_eq(loc.x, cx, 0.05));
        assert!(approx_eq(loc.y, cy, 0.05));
    }

    // -- Gauss-Newton 2D --

    #[test]
    fn test_gauss_newton_convergence() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 1.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.0, 1.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let sx = 0.6;
        let sy = 0.5;
        let array = SensorArray::new(sensors.clone(), v);
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(sx, sy, 0.0) / v)
            .collect();

        let loc = array.gauss_newton_2d(0.5, 0.5, &times, 100);
        assert!(approx_eq(loc.x, sx, 0.02));
        assert!(approx_eq(loc.y, sy, 0.02));
        assert!(loc.residual_rms < 1e-6);
    }

    // -- 3D iterative --

    #[test]
    fn test_3d_iterative() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.0, 1.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.0, 0.0, 1.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let sx = 0.4;
        let sy = 0.3;
        let sz = 0.5;
        let array = SensorArray::new(sensors.clone(), v);
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(sx, sy, sz) / v)
            .collect();

        let loc = array
            .locate_3d_iterative(&times, 0.5, 0.5, 0.5, 100)
            .unwrap();
        assert!(approx_eq(loc.x, sx, 0.05));
        assert!(approx_eq(loc.y, sy, 0.05));
        assert!(approx_eq(loc.z, sz, 0.05));
    }

    // -- Velocity estimation --

    #[test]
    fn test_velocity_estimation_plb() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let v_true = 5200.0;
        let array = SensorArray::new(sensors.clone(), 5000.0);

        let source_x = 0.3;
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(source_x, 0.0, 0.0) / v_true)
            .collect();

        let v_est = array.estimate_velocity_plb(source_x, 0.0, 0.0, &times);
        assert!(approx_eq(v_est, v_true, 10.0));
    }

    #[test]
    fn test_autocalibrate_velocity() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 0.866, 0.0, -20.0, 100e3, 1e6),
        ];
        let v_true = 5100.0;
        let array = SensorArray::new(sensors.clone(), 5000.0);

        let events: Vec<(f64, f64, f64, Vec<f64>)> = vec![
            (0.3, 0.2, 0.0, sensors.iter().map(|s| s.distance_to_point(0.3, 0.2, 0.0) / v_true).collect()),
            (0.7, 0.5, 0.0, sensors.iter().map(|s| s.distance_to_point(0.7, 0.5, 0.0) / v_true).collect()),
        ];

        let v_est = array.autocalibrate_velocity(&events);
        assert!(approx_eq(v_est, v_true, 100.0));
    }

    // -- Arrival time pickers --

    #[test]
    fn test_pick_threshold() {
        let signal = vec![0.0, 0.01, 0.02, 0.5, 0.8, 0.3, 0.0];
        assert_eq!(pick_threshold(&signal, 0.1), Some(3));
    }

    #[test]
    fn test_pick_threshold_none() {
        let signal = vec![0.0, 0.01, 0.02, 0.03];
        assert_eq!(pick_threshold(&signal, 0.1), None);
    }

    #[test]
    fn test_pick_aic() {
        // Noise then a step function → AIC should pick near the transition.
        let mut signal = vec![0.0; 200];
        for i in 100..200 {
            signal[i] = 1.0 + 0.01 * ((i * 7) as f64 % 3.0 - 1.0);
        }
        // Add small noise to the quiet part.
        for i in 0..100 {
            signal[i] = 0.001 * ((i * 13) as f64 % 5.0 - 2.0);
        }
        let idx = pick_aic(&signal).unwrap();
        // Should be near index 100.
        assert!(idx >= 90 && idx <= 115, "AIC pick was at {}, expected near 100", idx);
    }

    #[test]
    fn test_pick_sta_lta() {
        let mut signal = vec![0.001; 200];
        // Burst starting at sample 100.
        for i in 100..150 {
            signal[i] = 1.0;
        }
        let idx = pick_sta_lta(&signal, 5, 50, 5.0);
        assert!(idx.is_some());
        let idx = idx.unwrap();
        assert!(idx >= 90 && idx <= 110, "STA/LTA pick was at {}, expected near 100", idx);
    }

    #[test]
    fn test_sta_lta_no_event() {
        let signal = vec![0.001; 200];
        assert!(pick_sta_lta(&signal, 5, 50, 5.0).is_none());
    }

    // -- Cross-correlation --

    #[test]
    fn test_cross_correlate_zero_lag() {
        let a: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let b = a.clone();
        let (lag, corr) = cross_correlate_dt(&a, &b, 20);
        assert_eq!(lag, 0);
        assert!(corr > 0.99);
    }

    #[test]
    fn test_cross_correlate_known_lag() {
        let n = 100;
        let a: Vec<f64> = (0..n).map(|i| (i as f64 * 0.2).sin()).collect();
        // Shift b by 3 samples.
        let mut b = vec![0.0; n];
        for i in 3..n {
            b[i] = a[i - 3];
        }
        let (lag, _corr) = cross_correlate_dt(&a, &b, 10);
        assert_eq!(lag, 3);
    }

    // -- Attenuation model --

    #[test]
    fn test_attenuation_at_distance() {
        let model = AttenuationModel::new(1.0, 0.5, 0.01);
        let a1 = model.amplitude_at(1.0);
        // At r=1: A0 * 1^(-0.5) * exp(-0.01) ≈ 0.99005
        assert!(approx_eq(a1, (-0.01_f64).exp(), 0.001));

        let a4 = model.amplitude_at(4.0);
        // At r=4: 1.0 * 4^(-0.5) * exp(-0.04) = 0.5 * 0.9608 ≈ 0.4804
        assert!(approx_eq(a4, 0.5 * (-0.04_f64).exp(), 0.001));
    }

    #[test]
    fn test_attenuation_zero_distance() {
        let model = AttenuationModel::new(2.0, 0.5, 0.01);
        assert_eq!(model.amplitude_at(0.0), 2.0);
    }

    #[test]
    fn test_attenuation_plate_wave() {
        let model = AttenuationModel::plate_wave(1.0, 0.02);
        assert_eq!(model.n, 0.5);
        assert_eq!(model.alpha, 0.02);
    }

    #[test]
    fn test_attenuation_fit() {
        // Generate synthetic data from known model.
        let true_model = AttenuationModel::new(2.0, 0.5, 0.01);
        let distances: Vec<f64> = (1..=20).map(|i| i as f64 * 0.5).collect();
        let data: Vec<(f64, f64)> = distances
            .iter()
            .map(|&r| (r, true_model.amplitude_at(r)))
            .collect();

        let fitted = AttenuationModel::fit(&data).unwrap();
        assert!(approx_eq(fitted.n, 0.5, 0.1));
        assert!(approx_eq(fitted.alpha, 0.01, 0.01));
    }

    // -- Confidence ellipse --

    #[test]
    fn test_confidence_ellipse() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 1.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let sx = 0.5;
        let sy = 0.4;
        let array = SensorArray::new(sensors.clone(), v);
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(sx, sy, 0.0) / v)
            .collect();

        let ellipse = array.confidence_ellipse(sx, sy, &times, 1e-6).unwrap();
        assert!(ellipse.semi_major > 0.0);
        assert!(ellipse.semi_minor > 0.0);
        assert!(ellipse.semi_major >= ellipse.semi_minor);
        assert!(approx_eq(ellipse.cx, sx, 1e-10));
        assert!(approx_eq(ellipse.cy, sy, 1e-10));
    }

    // -- Zonal location --

    #[test]
    fn test_zonal_locate() {
        let sensors = vec![
            AeSensor::new(0.1, 0.1, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.9, 0.1, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 0.9, 0.0, -20.0, 100e3, 1e6),
        ];
        let array = SensorArray::new(sensors, 5000.0);

        let zones = vec![
            Zone { name: "Zone_A".to_string(), cx: 0.1, cy: 0.1, radius: 0.3 },
            Zone { name: "Zone_B".to_string(), cx: 0.9, cy: 0.1, radius: 0.3 },
            Zone { name: "Zone_C".to_string(), cx: 0.5, cy: 0.9, radius: 0.3 },
        ];

        // Earliest arrival at sensor 0 → Zone_A
        let times = vec![0.001, 0.002, 0.003];
        let zone = array.zonal_locate(&times, &zones);
        assert_eq!(zone, Some("Zone_A".to_string()));
    }

    #[test]
    fn test_zonal_locate_zone_b() {
        let sensors = vec![
            AeSensor::new(0.1, 0.1, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.9, 0.1, 0.0, -20.0, 100e3, 1e6),
        ];
        let array = SensorArray::new(sensors, 5000.0);

        let zones = vec![
            Zone { name: "Left".to_string(), cx: 0.1, cy: 0.1, radius: 0.3 },
            Zone { name: "Right".to_string(), cx: 0.9, cy: 0.1, radius: 0.3 },
        ];

        // Earliest at sensor 1 → Right
        let times = vec![0.003, 0.001];
        let zone = array.zonal_locate(&times, &zones);
        assert_eq!(zone, Some("Right".to_string()));
    }

    // -- Event clustering --

    #[test]
    fn test_cluster_events_single() {
        let locs = vec![(0.0, 0.0, 0.0), (0.01, 0.01, 0.0), (0.02, 0.0, 0.0)];
        let clusters = cluster_events(&locs, 0.1);
        assert_eq!(clusters.len(), 1);
        assert_eq!(clusters[0].event_indices.len(), 3);
    }

    #[test]
    fn test_cluster_events_two_clusters() {
        let locs = vec![
            (0.0, 0.0, 0.0),
            (0.01, 0.01, 0.0),
            (1.0, 1.0, 0.0),
            (1.01, 1.01, 0.0),
        ];
        let clusters = cluster_events(&locs, 0.1);
        assert_eq!(clusters.len(), 2);
    }

    #[test]
    fn test_cluster_events_empty() {
        let locs: Vec<(f64, f64, f64)> = vec![];
        let clusters = cluster_events(&locs, 0.1);
        assert!(clusters.is_empty());
    }

    #[test]
    fn test_cluster_centroid() {
        let locs = vec![(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.5, 0.5, 0.0)];
        let clusters = cluster_events(&locs, 2.0);
        assert_eq!(clusters.len(), 1);
        assert!(approx_eq(clusters[0].centroid_x, 0.5, 1e-10));
    }

    // -- b-value analysis --

    #[test]
    fn test_b_value_analysis() {
        // Generate amplitudes following Gutenberg-Richter with b~1.0
        let mut amps = Vec::new();
        for i in 0..200 {
            // Rough inverse CDF: many small events, few large ones.
            let m = 40.0 + (200 - i) as f64 * 0.3;
            amps.push(m);
        }
        let result = b_value_analysis(&amps);
        assert!(result.is_some());
        let (b, _a, r_sq) = result.unwrap();
        assert!(b > 0.0, "b-value should be positive, got {}", b);
        assert!(r_sq > 0.5, "R-squared should be reasonable, got {}", r_sq);
    }

    #[test]
    fn test_b_value_too_few() {
        let amps = vec![40.0, 50.0, 60.0];
        assert!(b_value_analysis(&amps).is_none());
    }

    // -- Felicity ratio --

    #[test]
    fn test_felicity_ratio() {
        assert!(approx_eq(felicity_ratio(95.0, 100.0), 0.95, 1e-10));
        assert!(approx_eq(felicity_ratio(100.0, 100.0), 1.0, 1e-10));
        assert!(approx_eq(felicity_ratio(80.0, 100.0), 0.8, 1e-10));
    }

    #[test]
    fn test_felicity_ratio_zero_prev() {
        assert!(approx_eq(felicity_ratio(10.0, 0.0), 1.0, 1e-10));
    }

    // -- Kaiser effect --

    #[test]
    fn test_kaiser_effect_present() {
        // Load → AE → unload → reload past previous max → AE resumes.
        let history = vec![
            (50.0, 5),
            (80.0, 10),
            (100.0, 20),
            (50.0, 0),  // unload
            (80.0, 0),  // reload, no AE yet
            (99.0, 0),  // still no AE
            (101.0, 3), // AE resumes past previous max → Kaiser effect
        ];
        assert!(detect_kaiser_effect(&history, 0.95));
    }

    #[test]
    fn test_kaiser_effect_absent() {
        // AE resumes well below previous max → Felicity ratio low.
        let history = vec![
            (50.0, 5),
            (100.0, 20),
            (50.0, 0),
            (60.0, 5), // AE at 60% of previous max → no Kaiser effect
        ];
        assert!(!detect_kaiser_effect(&history, 0.95));
    }

    // -- Residual computation --

    #[test]
    fn test_residual_perfect_data() {
        let sensors = vec![
            AeSensor::new(0.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(1.0, 0.0, 0.0, -20.0, 100e3, 1e6),
            AeSensor::new(0.5, 1.0, 0.0, -20.0, 100e3, 1e6),
        ];
        let v = 5000.0;
        let sx = 0.3;
        let sy = 0.4;
        let array = SensorArray::new(sensors.clone(), v);
        let times: Vec<f64> = sensors
            .iter()
            .map(|s| s.distance_to_point(sx, sy, 0.0) / v)
            .collect();

        let res = array.compute_residual_2d(sx, sy, &times);
        assert!(res < 1e-10, "Residual should be near zero for perfect data, got {}", res);
    }

    // -- Solve 3x3 --

    #[test]
    fn test_solve_3x3_identity() {
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [1.0, 2.0, 3.0];
        let sol = solve_3x3(&a, &b).unwrap();
        assert!(approx_eq(sol[0], 1.0, 1e-10));
        assert!(approx_eq(sol[1], 2.0, 1e-10));
        assert!(approx_eq(sol[2], 3.0, 1e-10));
    }

    #[test]
    fn test_solve_3x3_singular() {
        let a = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [1.0, 1.0, 1.0]];
        let b = [1.0, 2.0, 3.0];
        assert!(solve_3x3(&a, &b).is_none());
    }

    // -- Least squares 2D --

    #[test]
    fn test_least_squares_2d() {
        // Simple overdetermined system.
        let a = vec![vec![1.0, 0.0], vec![0.0, 1.0], vec![1.0, 1.0]];
        let b = vec![3.0, 4.0, 7.0];
        let sol = solve_least_squares_2d(&a, &b).unwrap();
        assert!(approx_eq(sol[0], 3.0, 0.01));
        assert!(approx_eq(sol[1], 4.0, 0.01));
    }
}
