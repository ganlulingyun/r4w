//! Passive RF emitter geolocation using TDOA and AoA from distributed sensor networks.
//!
//! This module implements hyperbolic trilateration (TDOA), bearing triangulation (AoA),
//! and hybrid fusion for locating RF emitters using measurements from spatially
//! distributed sensors.
//!
//! # Example
//!
//! ```
//! use r4w_core::emitter_localization::{EmitterLocalizer, SensorPosition, TdoaMeasurement};
//!
//! // Place four sensors in a square pattern
//! let sensors = vec![
//!     SensorPosition { x: 0.0, y: 0.0, z: 0.0 },
//!     SensorPosition { x: 1000.0, y: 0.0, z: 0.0 },
//!     SensorPosition { x: 1000.0, y: 1000.0, z: 0.0 },
//!     SensorPosition { x: 0.0, y: 1000.0, z: 0.0 },
//! ];
//!
//! let localizer = EmitterLocalizer::new(sensors.clone());
//!
//! // Emitter is near the center at (500, 500, 0)
//! // Generate synthetic TDOA measurements from known position
//! let true_pos = SensorPosition { x: 500.0, y: 500.0, z: 0.0 };
//! let tdoas: Vec<TdoaMeasurement> = vec![
//!     TdoaMeasurement::new(0, 1, r4w_core::emitter_localization::tdoa_from_range(&sensors[0], &sensors[1], &true_pos), 0.95),
//!     TdoaMeasurement::new(0, 2, r4w_core::emitter_localization::tdoa_from_range(&sensors[0], &sensors[2], &true_pos), 0.95),
//!     TdoaMeasurement::new(0, 3, r4w_core::emitter_localization::tdoa_from_range(&sensors[0], &sensors[3], &true_pos), 0.95),
//!     TdoaMeasurement::new(1, 2, r4w_core::emitter_localization::tdoa_from_range(&sensors[1], &sensors[2], &true_pos), 0.95),
//!     TdoaMeasurement::new(1, 3, r4w_core::emitter_localization::tdoa_from_range(&sensors[1], &sensors[3], &true_pos), 0.95),
//! ];
//!
//! let result = localizer.localize_tdoa(&tdoas).unwrap();
//! assert!((result.position.x - 500.0).abs() < 50.0);
//! assert!((result.position.y - 500.0).abs() < 50.0);
//! ```

use std::f64::consts::PI;

/// Speed of light in meters per second.
const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// A 3D position in Cartesian coordinates (meters).
#[derive(Debug, Clone, PartialEq)]
pub struct SensorPosition {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl SensorPosition {
    /// Create a new sensor position.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Euclidean distance to another position.
    pub fn distance_to(&self, other: &SensorPosition) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

/// A TDOA measurement between a pair of sensors.
#[derive(Debug, Clone)]
pub struct TdoaMeasurement {
    /// Index of the first sensor in the sensor array.
    pub sensor_i: usize,
    /// Index of the second sensor in the sensor array.
    pub sensor_j: usize,
    /// Time difference of arrival in seconds (positive means signal arrived at i first).
    pub tdoa_seconds: f64,
    /// Measurement confidence in [0, 1].
    pub confidence: f64,
}

impl TdoaMeasurement {
    /// Create a new TDOA measurement.
    pub fn new(sensor_i: usize, sensor_j: usize, tdoa_seconds: f64, confidence: f64) -> Self {
        Self {
            sensor_i,
            sensor_j,
            tdoa_seconds,
            confidence: confidence.clamp(0.0, 1.0),
        }
    }

    /// Compute the expected TDOA given an emitter position and two sensor positions.
    ///
    /// This is useful for generating synthetic test data and for computing residuals.
    pub fn from_range_difference(
        sensor_i: &SensorPosition,
        sensor_j: &SensorPosition,
        emitter: &SensorPosition,
        confidence: f64,
    ) -> Self {
        let range_i = sensor_i.distance_to(emitter);
        let range_j = sensor_j.distance_to(emitter);
        let tdoa = (range_i - range_j) / SPEED_OF_LIGHT;
        Self {
            sensor_i: 0,
            sensor_j: 1,
            tdoa_seconds: tdoa,
            confidence: confidence.clamp(0.0, 1.0),
        }
    }
}

/// An angle-of-arrival measurement from a single sensor.
#[derive(Debug, Clone)]
pub struct AoaMeasurement {
    /// Index of the sensor in the sensor array.
    pub sensor_id: usize,
    /// Azimuth angle in radians (0 = +x axis, pi/2 = +y axis).
    pub azimuth_rad: f64,
    /// Elevation angle in radians (0 = horizon, pi/2 = zenith).
    pub elevation_rad: f64,
    /// Measurement confidence in [0, 1].
    pub confidence: f64,
}

impl AoaMeasurement {
    /// Create a new AoA measurement.
    pub fn new(sensor_id: usize, azimuth_rad: f64, elevation_rad: f64, confidence: f64) -> Self {
        Self {
            sensor_id,
            azimuth_rad,
            elevation_rad,
            confidence: confidence.clamp(0.0, 1.0),
        }
    }

    /// Compute AoA from sensor position to emitter position.
    pub fn from_positions(
        sensor_id: usize,
        sensor: &SensorPosition,
        emitter: &SensorPosition,
        confidence: f64,
    ) -> Self {
        let dx = emitter.x - sensor.x;
        let dy = emitter.y - sensor.y;
        let dz = emitter.z - sensor.z;
        let azimuth = dy.atan2(dx);
        let horiz_range = (dx * dx + dy * dy).sqrt();
        let elevation = dz.atan2(horiz_range);
        Self {
            sensor_id,
            azimuth_rad: azimuth,
            elevation_rad: elevation,
            confidence: confidence.clamp(0.0, 1.0),
        }
    }
}

/// Result of a localization computation.
#[derive(Debug, Clone)]
pub struct LocalizationResult {
    /// Estimated emitter position in Cartesian coordinates (meters).
    pub position: SensorPosition,
    /// Circular error probable (50th percentile) in meters.
    pub cep_meters: f64,
    /// Geometric dilution of precision.
    pub gdop: f64,
}

/// Passive RF emitter localizer using distributed sensors.
///
/// Supports TDOA (hyperbolic trilateration), AoA (bearing triangulation),
/// and hybrid fusion of both measurement types.
#[derive(Debug, Clone)]
pub struct EmitterLocalizer {
    sensors: Vec<SensorPosition>,
}

impl EmitterLocalizer {
    /// Create a new localizer with the given sensor positions.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 sensors are provided.
    pub fn new(sensors: Vec<SensorPosition>) -> Self {
        assert!(
            sensors.len() >= 2,
            "At least 2 sensors are required for localization"
        );
        Self { sensors }
    }

    /// Return the number of sensors.
    pub fn num_sensors(&self) -> usize {
        self.sensors.len()
    }

    /// Get reference to sensor positions.
    pub fn sensors(&self) -> &[SensorPosition] {
        &self.sensors
    }

    /// Compute geometric dilution of precision from sensor geometry relative to a position.
    ///
    /// GDOP measures how sensor geometry amplifies measurement errors. Lower is better.
    /// A value of 1.0 is ideal; values above 6 indicate poor geometry.
    ///
    /// Automatically detects 2D (coplanar) sensor layouts and uses the appropriate
    /// dimensionality for the computation.
    pub fn compute_gdop(&self, position: &SensorPosition) -> f64 {
        let n = self.sensors.len();
        if n < 3 {
            return f64::INFINITY;
        }

        // Build direction cosine matrix H (n x 3 for 3D)
        let mut h = vec![0.0; n * 3];
        for (i, s) in self.sensors.iter().enumerate() {
            let r = position.distance_to(s);
            if r < 1e-10 {
                return f64::INFINITY;
            }
            h[i * 3] = (s.x - position.x) / r;
            h[i * 3 + 1] = (s.y - position.y) / r;
            h[i * 3 + 2] = (s.z - position.z) / r;
        }

        // Check if geometry is essentially 2D (all z direction cosines near zero)
        let z_energy: f64 = (0..n).map(|k| h[k * 3 + 2] * h[k * 3 + 2]).sum();
        let is_2d = z_energy < 1e-10;

        if is_2d {
            // 2D case: use 2x2 H^T*H from x,y components only
            let mut hth = [0.0f64; 4];
            for i in 0..2 {
                for j in 0..2 {
                    let mut sum = 0.0;
                    for k in 0..n {
                        sum += h[k * 3 + i] * h[k * 3 + j];
                    }
                    hth[i * 2 + j] = sum;
                }
            }
            if let Some(inv) = invert_2x2(&hth) {
                let trace = inv[0] + inv[3];
                if trace > 0.0 {
                    trace.sqrt()
                } else {
                    f64::INFINITY
                }
            } else {
                f64::INFINITY
            }
        } else {
            // 3D case: use full 3x3
            let mut hth = [0.0f64; 9];
            for i in 0..3 {
                for j in 0..3 {
                    let mut sum = 0.0;
                    for k in 0..n {
                        sum += h[k * 3 + i] * h[k * 3 + j];
                    }
                    hth[i * 3 + j] = sum;
                }
            }
            if let Some(inv) = invert_3x3(&hth) {
                let trace = inv[0] + inv[4] + inv[8];
                if trace > 0.0 {
                    trace.sqrt()
                } else {
                    f64::INFINITY
                }
            } else {
                f64::INFINITY
            }
        }
    }

    /// Localize an emitter using TDOA measurements via iterative least-squares.
    ///
    /// Uses Gauss-Newton iteration with sensor centroid as initial estimate.
    ///
    /// Requires at least 2 TDOA measurements.
    pub fn localize_tdoa(
        &self,
        measurements: &[TdoaMeasurement],
    ) -> Result<LocalizationResult, LocalizationError> {
        if measurements.len() < 2 {
            return Err(LocalizationError::InsufficientMeasurements {
                required: 2,
                provided: measurements.len(),
            });
        }

        // Convert TDOA to range differences
        let range_diffs: Vec<RangeDiffEntry> = measurements
            .iter()
            .map(|m| {
                RangeDiffEntry {
                    si: m.sensor_i,
                    sj: m.sensor_j,
                    range_diff: m.tdoa_seconds * SPEED_OF_LIGHT,
                    weight: m.confidence,
                }
            })
            .collect();

        // Validate sensor indices
        for rd in &range_diffs {
            if rd.si >= self.sensors.len() || rd.sj >= self.sensors.len() {
                return Err(LocalizationError::SingularGeometry);
            }
        }

        // Initial estimate: centroid of sensors
        let mut estimate = self.sensor_centroid();

        // Gauss-Newton iteration
        let max_iter = 50;
        let tolerance = 1e-6;

        for _ in 0..max_iter {
            let n = range_diffs.len();

            let mut jacobian = vec![0.0; n * 3];
            let mut residuals = vec![0.0; n];

            for (k, rd) in range_diffs.iter().enumerate() {
                let si = &self.sensors[rd.si];
                let sj = &self.sensors[rd.sj];
                let ri = estimate.distance_to(si);
                let rj = estimate.distance_to(sj);

                if ri < 1e-10 || rj < 1e-10 {
                    continue;
                }

                let computed_diff = ri - rj;
                residuals[k] = (rd.range_diff - computed_diff) * rd.weight;

                let dri_dx = (estimate.x - si.x) / ri;
                let dri_dy = (estimate.y - si.y) / ri;
                let dri_dz = (estimate.z - si.z) / ri;
                let drj_dx = (estimate.x - sj.x) / rj;
                let drj_dy = (estimate.y - sj.y) / rj;
                let drj_dz = (estimate.z - sj.z) / rj;

                jacobian[k * 3] = (dri_dx - drj_dx) * rd.weight;
                jacobian[k * 3 + 1] = (dri_dy - drj_dy) * rd.weight;
                jacobian[k * 3 + 2] = (dri_dz - drj_dz) * rd.weight;
            }

            // Solve J^T J delta = J^T r
            let mut jtj = [0.0f64; 9];
            let mut jtr = [0.0f64; 3];

            for i in 0..3 {
                for j in 0..3 {
                    let mut sum = 0.0;
                    for k in 0..n {
                        sum += jacobian[k * 3 + i] * jacobian[k * 3 + j];
                    }
                    jtj[i * 3 + j] = sum;
                }
                let mut sum = 0.0;
                for k in 0..n {
                    sum += jacobian[k * 3 + i] * residuals[k];
                }
                jtr[i] = sum;
            }

            // Regularization for numerical stability
            jtj[0] += 1e-10;
            jtj[4] += 1e-10;
            jtj[8] += 1e-10;

            let inv = match invert_3x3(&jtj) {
                Some(inv) => inv,
                None => break,
            };

            let dx = inv[0] * jtr[0] + inv[1] * jtr[1] + inv[2] * jtr[2];
            let dy = inv[3] * jtr[0] + inv[4] * jtr[1] + inv[5] * jtr[2];
            let dz = inv[6] * jtr[0] + inv[7] * jtr[1] + inv[8] * jtr[2];

            estimate.x += dx;
            estimate.y += dy;
            estimate.z += dz;

            let step = (dx * dx + dy * dy + dz * dz).sqrt();
            if step < tolerance {
                break;
            }
        }

        let gdop = self.compute_gdop(&estimate);

        let rms_residual = self.compute_tdoa_rms(&estimate, measurements);
        let cep = rms_residual * SPEED_OF_LIGHT * 0.675;

        Ok(LocalizationResult {
            position: estimate,
            cep_meters: cep,
            gdop,
        })
    }

    /// Localize an emitter using AoA (angle of arrival) measurements via bearing triangulation.
    ///
    /// Uses weighted least-squares intersection of bearing lines.
    /// Requires at least 2 AoA measurements.
    pub fn localize_aoa(
        &self,
        measurements: &[AoaMeasurement],
    ) -> Result<LocalizationResult, LocalizationError> {
        if measurements.len() < 2 {
            return Err(LocalizationError::InsufficientMeasurements {
                required: 2,
                provided: measurements.len(),
            });
        }

        // Weighted least-squares bearing intersection
        // For each bearing line from sensor s in direction d:
        //   (I - d*d^T) * (p - s) = 0
        // => sum_i w_i (I - di*di^T) * p = sum_i w_i (I - di*di^T) * si

        let mut a = [0.0f64; 9]; // 3x3
        let mut b = [0.0f64; 3];

        for m in measurements {
            if m.sensor_id >= self.sensors.len() {
                continue;
            }
            let s = &self.sensors[m.sensor_id];
            let w = m.confidence;

            let cos_el = m.elevation_rad.cos();
            let dx = m.azimuth_rad.cos() * cos_el;
            let dy = m.azimuth_rad.sin() * cos_el;
            let dz = m.elevation_rad.sin();

            // P = I - d*d^T
            let p = [
                1.0 - dx * dx, -dx * dy,       -dx * dz,
                -dy * dx,       1.0 - dy * dy,  -dy * dz,
                -dz * dx,       -dz * dy,       1.0 - dz * dz,
            ];

            let sv = [s.x, s.y, s.z];

            for i in 0..3 {
                for j in 0..3 {
                    a[i * 3 + j] += w * p[i * 3 + j];
                }
                let mut ps_i = 0.0;
                for j in 0..3 {
                    ps_i += p[i * 3 + j] * sv[j];
                }
                b[i] += w * ps_i;
            }
        }

        // Regularization
        a[0] += 1e-10;
        a[4] += 1e-10;
        a[8] += 1e-10;

        let inv = invert_3x3(&a).ok_or(LocalizationError::SingularGeometry)?;

        let px = inv[0] * b[0] + inv[1] * b[1] + inv[2] * b[2];
        let py = inv[3] * b[0] + inv[4] * b[1] + inv[5] * b[2];
        let pz = inv[6] * b[0] + inv[7] * b[1] + inv[8] * b[2];

        let position = SensorPosition::new(px, py, pz);
        let gdop = self.compute_gdop(&position);
        let cep = self.compute_aoa_cep(&position, measurements);

        Ok(LocalizationResult {
            position,
            cep_meters: cep,
            gdop,
        })
    }

    /// Localize using a hybrid combination of TDOA and AoA measurements.
    ///
    /// Performs TDOA and AoA localization independently, then fuses results
    /// with inverse-CEP weighting.
    pub fn localize_hybrid(
        &self,
        tdoa_measurements: &[TdoaMeasurement],
        aoa_measurements: &[AoaMeasurement],
    ) -> Result<LocalizationResult, LocalizationError> {
        if tdoa_measurements.is_empty() && aoa_measurements.is_empty() {
            return Err(LocalizationError::InsufficientMeasurements {
                required: 2,
                provided: 0,
            });
        }

        if tdoa_measurements.is_empty() {
            return self.localize_aoa(aoa_measurements);
        }
        if aoa_measurements.is_empty() {
            return self.localize_tdoa(tdoa_measurements);
        }

        let tdoa_result = self.localize_tdoa(tdoa_measurements)?;
        let aoa_result = self.localize_aoa(aoa_measurements)?;

        // Weighted fusion based on inverse CEP
        let w_tdoa = if tdoa_result.cep_meters > 1e-10 {
            1.0 / tdoa_result.cep_meters
        } else {
            1e10
        };
        let w_aoa = if aoa_result.cep_meters > 1e-10 {
            1.0 / aoa_result.cep_meters
        } else {
            1e10
        };
        let w_total = w_tdoa + w_aoa;

        let fused_position = SensorPosition::new(
            (w_tdoa * tdoa_result.position.x + w_aoa * aoa_result.position.x) / w_total,
            (w_tdoa * tdoa_result.position.y + w_aoa * aoa_result.position.y) / w_total,
            (w_tdoa * tdoa_result.position.z + w_aoa * aoa_result.position.z) / w_total,
        );

        let gdop = self.compute_gdop(&fused_position);
        let cep = (w_tdoa * tdoa_result.cep_meters + w_aoa * aoa_result.cep_meters) / w_total;

        Ok(LocalizationResult {
            position: fused_position,
            cep_meters: cep,
            gdop,
        })
    }

    // --- private helpers ---

    fn sensor_centroid(&self) -> SensorPosition {
        let n = self.sensors.len() as f64;
        let mut cx = 0.0;
        let mut cy = 0.0;
        let mut cz = 0.0;
        for s in &self.sensors {
            cx += s.x;
            cy += s.y;
            cz += s.z;
        }
        SensorPosition::new(cx / n, cy / n, cz / n)
    }

    fn compute_tdoa_rms(
        &self,
        position: &SensorPosition,
        measurements: &[TdoaMeasurement],
    ) -> f64 {
        if measurements.is_empty() {
            return 0.0;
        }
        let mut sum_sq = 0.0;
        let mut count = 0;
        for m in measurements {
            if m.sensor_i >= self.sensors.len() || m.sensor_j >= self.sensors.len() {
                continue;
            }
            let ri = position.distance_to(&self.sensors[m.sensor_i]);
            let rj = position.distance_to(&self.sensors[m.sensor_j]);
            let computed_tdoa = (ri - rj) / SPEED_OF_LIGHT;
            let residual = m.tdoa_seconds - computed_tdoa;
            sum_sq += residual * residual;
            count += 1;
        }
        if count == 0 {
            return 0.0;
        }
        (sum_sq / count as f64).sqrt()
    }

    fn compute_aoa_cep(
        &self,
        position: &SensorPosition,
        measurements: &[AoaMeasurement],
    ) -> f64 {
        if measurements.is_empty() {
            return 0.0;
        }
        let mut sum_sq = 0.0;
        let mut count = 0;
        for m in measurements {
            if m.sensor_id >= self.sensors.len() {
                continue;
            }
            let s = &self.sensors[m.sensor_id];
            let expected = AoaMeasurement::from_positions(m.sensor_id, s, position, 1.0);
            let az_err = angle_diff(m.azimuth_rad, expected.azimuth_rad);
            let el_err = angle_diff(m.elevation_rad, expected.elevation_rad);
            let r = s.distance_to(position);
            sum_sq += (az_err * az_err + el_err * el_err) * r * r;
            count += 1;
        }
        if count == 0 {
            return f64::INFINITY;
        }
        (sum_sq / count as f64).sqrt() * 0.675
    }
}

/// Compute the expected TDOA given an emitter position and two sensor positions.
///
/// Returns time difference in seconds. Positive means signal arrived at sensor_i first.
pub fn tdoa_from_range(
    sensor_i: &SensorPosition,
    sensor_j: &SensorPosition,
    emitter: &SensorPosition,
) -> f64 {
    let range_i = sensor_i.distance_to(emitter);
    let range_j = sensor_j.distance_to(emitter);
    (range_i - range_j) / SPEED_OF_LIGHT
}

/// Errors that can occur during localization.
#[derive(Debug, Clone, PartialEq)]
pub enum LocalizationError {
    /// Not enough measurements for the requested localization.
    InsufficientMeasurements { required: usize, provided: usize },
    /// Sensor geometry is degenerate (all collinear, etc.).
    SingularGeometry,
}

impl std::fmt::Display for LocalizationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LocalizationError::InsufficientMeasurements { required, provided } => {
                write!(
                    f,
                    "insufficient measurements: need {}, got {}",
                    required, provided
                )
            }
            LocalizationError::SingularGeometry => {
                write!(f, "sensor geometry is degenerate (singular matrix)")
            }
        }
    }
}

impl std::error::Error for LocalizationError {}

// --- internal helpers ---

struct RangeDiffEntry {
    si: usize,
    sj: usize,
    range_diff: f64,
    weight: f64,
}

/// Invert a 2x2 matrix stored in row-major order. Returns None if singular.
fn invert_2x2(m: &[f64; 4]) -> Option<[f64; 4]> {
    let a = m[0]; let b = m[1];
    let c = m[2]; let d = m[3];
    let det = a * d - b * c;
    if det.abs() < 1e-30 {
        return None;
    }
    let inv_det = 1.0 / det;
    Some([d * inv_det, -b * inv_det, -c * inv_det, a * inv_det])
}

/// Invert a 3x3 matrix stored in row-major order. Returns None if singular.
fn invert_3x3(m: &[f64; 9]) -> Option<[f64; 9]> {
    let a = m[0]; let b = m[1]; let c = m[2];
    let d = m[3]; let e = m[4]; let f = m[5];
    let g = m[6]; let h = m[7]; let i = m[8];

    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);

    if det.abs() < 1e-30 {
        return None;
    }

    let inv_det = 1.0 / det;

    Some([
        (e * i - f * h) * inv_det,
        (c * h - b * i) * inv_det,
        (b * f - c * e) * inv_det,
        (f * g - d * i) * inv_det,
        (a * i - c * g) * inv_det,
        (c * d - a * f) * inv_det,
        (d * h - e * g) * inv_det,
        (b * g - a * h) * inv_det,
        (a * e - b * d) * inv_det,
    ])
}

/// Compute the shortest angular difference between two angles in radians.
fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = a - b;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: create a square sensor layout at z=0.
    fn square_sensors(size: f64) -> Vec<SensorPosition> {
        vec![
            SensorPosition::new(0.0, 0.0, 0.0),
            SensorPosition::new(size, 0.0, 0.0),
            SensorPosition::new(size, size, 0.0),
            SensorPosition::new(0.0, size, 0.0),
        ]
    }

    /// Helper: generate TDOA measurements for all sensor pairs from a known position.
    fn generate_tdoa_measurements(
        sensors: &[SensorPosition],
        emitter: &SensorPosition,
    ) -> Vec<TdoaMeasurement> {
        let mut measurements = Vec::new();
        for i in 0..sensors.len() {
            for j in (i + 1)..sensors.len() {
                let tdoa = tdoa_from_range(&sensors[i], &sensors[j], emitter);
                measurements.push(TdoaMeasurement::new(i, j, tdoa, 1.0));
            }
        }
        measurements
    }

    /// Helper: generate AoA measurements from known position.
    fn generate_aoa_measurements(
        sensors: &[SensorPosition],
        emitter: &SensorPosition,
    ) -> Vec<AoaMeasurement> {
        sensors
            .iter()
            .enumerate()
            .map(|(id, s)| AoaMeasurement::from_positions(id, s, emitter, 1.0))
            .collect()
    }

    #[test]
    fn test_sensor_distance() {
        let a = SensorPosition::new(0.0, 0.0, 0.0);
        let b = SensorPosition::new(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_sensor_distance_3d() {
        let a = SensorPosition::new(1.0, 2.0, 3.0);
        let b = SensorPosition::new(4.0, 6.0, 3.0);
        let expected = 5.0;
        assert!((a.distance_to(&b) - expected).abs() < 1e-10);
    }

    #[test]
    fn test_tdoa_from_range_equidistant() {
        let s1 = SensorPosition::new(-100.0, 0.0, 0.0);
        let s2 = SensorPosition::new(100.0, 0.0, 0.0);
        let emitter = SensorPosition::new(0.0, 500.0, 0.0);
        let tdoa = tdoa_from_range(&s1, &s2, &emitter);
        assert!(tdoa.abs() < 1e-15);
    }

    #[test]
    fn test_tdoa_from_range_known_value() {
        let s1 = SensorPosition::new(0.0, 0.0, 0.0);
        let s2 = SensorPosition::new(1000.0, 0.0, 0.0);
        let emitter = SensorPosition::new(0.0, 0.0, 0.0);
        let tdoa = tdoa_from_range(&s1, &s2, &emitter);
        let expected = -1000.0 / SPEED_OF_LIGHT;
        assert!((tdoa - expected).abs() < 1e-20);
    }

    #[test]
    fn test_aoa_from_positions() {
        let sensor = SensorPosition::new(0.0, 0.0, 0.0);
        let emitter = SensorPosition::new(100.0, 0.0, 0.0);
        let aoa = AoaMeasurement::from_positions(0, &sensor, &emitter, 1.0);
        assert!((aoa.azimuth_rad - 0.0).abs() < 1e-10);
        assert!((aoa.elevation_rad - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_aoa_from_positions_45deg() {
        let sensor = SensorPosition::new(0.0, 0.0, 0.0);
        let emitter = SensorPosition::new(100.0, 100.0, 0.0);
        let aoa = AoaMeasurement::from_positions(0, &sensor, &emitter, 1.0);
        assert!((aoa.azimuth_rad - PI / 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_localize_tdoa_center_of_square() {
        let sensors = square_sensors(1000.0);
        let emitter = SensorPosition::new(500.0, 500.0, 0.0);
        let measurements = generate_tdoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_tdoa(&measurements).unwrap();

        assert!(
            (result.position.x - 500.0).abs() < 1.0,
            "x error: {}",
            (result.position.x - 500.0).abs()
        );
        assert!(
            (result.position.y - 500.0).abs() < 1.0,
            "y error: {}",
            (result.position.y - 500.0).abs()
        );
    }

    #[test]
    fn test_localize_tdoa_off_center() {
        let sensors = square_sensors(2000.0);
        let emitter = SensorPosition::new(300.0, 700.0, 0.0);
        let measurements = generate_tdoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_tdoa(&measurements).unwrap();

        assert!(
            (result.position.x - 300.0).abs() < 5.0,
            "x error: {}",
            (result.position.x - 300.0).abs()
        );
        assert!(
            (result.position.y - 700.0).abs() < 5.0,
            "y error: {}",
            (result.position.y - 700.0).abs()
        );
    }

    #[test]
    fn test_localize_aoa_2d() {
        let sensors = square_sensors(1000.0);
        let emitter = SensorPosition::new(500.0, 500.0, 0.0);
        let measurements = generate_aoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_aoa(&measurements).unwrap();

        assert!(
            (result.position.x - 500.0).abs() < 1.0,
            "x error: {}",
            (result.position.x - 500.0).abs()
        );
        assert!(
            (result.position.y - 500.0).abs() < 1.0,
            "y error: {}",
            (result.position.y - 500.0).abs()
        );
    }

    #[test]
    fn test_localize_aoa_off_center() {
        let sensors = square_sensors(1000.0);
        let emitter = SensorPosition::new(200.0, 800.0, 0.0);
        let measurements = generate_aoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_aoa(&measurements).unwrap();

        assert!(
            (result.position.x - 200.0).abs() < 1.0,
            "x error: {}",
            (result.position.x - 200.0).abs()
        );
        assert!(
            (result.position.y - 800.0).abs() < 1.0,
            "y error: {}",
            (result.position.y - 800.0).abs()
        );
    }

    #[test]
    fn test_localize_hybrid() {
        let sensors = square_sensors(1000.0);
        let emitter = SensorPosition::new(400.0, 600.0, 0.0);
        let tdoa_meas = generate_tdoa_measurements(&sensors, &emitter);
        let aoa_meas = generate_aoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer
            .localize_hybrid(&tdoa_meas, &aoa_meas)
            .unwrap();

        assert!(
            (result.position.x - 400.0).abs() < 5.0,
            "x error: {}",
            (result.position.x - 400.0).abs()
        );
        assert!(
            (result.position.y - 600.0).abs() < 5.0,
            "y error: {}",
            (result.position.y - 600.0).abs()
        );
    }

    #[test]
    fn test_gdop_symmetric_geometry() {
        let sensors = square_sensors(1000.0);
        let localizer = EmitterLocalizer::new(sensors);
        let center = SensorPosition::new(500.0, 500.0, 0.0);
        let gdop = localizer.compute_gdop(&center);
        assert!(gdop > 0.0 && gdop < 10.0, "GDOP = {}", gdop);
    }

    #[test]
    fn test_gdop_poor_geometry() {
        let sensors = vec![
            SensorPosition::new(0.0, 0.0, 0.0),
            SensorPosition::new(100.0, 0.1, 0.0),
            SensorPosition::new(200.0, 0.2, 0.0),
        ];
        let localizer = EmitterLocalizer::new(sensors);
        let pos = SensorPosition::new(100.0, 500.0, 0.0);
        let gdop = localizer.compute_gdop(&pos);
        assert!(gdop > 3.0, "GDOP should be large for near-collinear sensors, got {}", gdop);
    }

    #[test]
    fn test_insufficient_measurements_error() {
        let sensors = square_sensors(1000.0);
        let localizer = EmitterLocalizer::new(sensors);

        let result = localizer.localize_tdoa(&[]);
        assert!(result.is_err());
        match result.unwrap_err() {
            LocalizationError::InsufficientMeasurements { required, provided } => {
                assert_eq!(required, 2);
                assert_eq!(provided, 0);
            }
            _ => panic!("expected InsufficientMeasurements"),
        }
    }

    #[test]
    fn test_aoa_insufficient_measurements() {
        let sensors = square_sensors(1000.0);
        let localizer = EmitterLocalizer::new(sensors);
        let single = vec![AoaMeasurement::new(0, 0.5, 0.0, 1.0)];
        let result = localizer.localize_aoa(&single);
        assert!(result.is_err());
    }

    #[test]
    fn test_3d_localization_tdoa() {
        let sensors = vec![
            SensorPosition::new(0.0, 0.0, 0.0),
            SensorPosition::new(1000.0, 0.0, 0.0),
            SensorPosition::new(0.0, 1000.0, 0.0),
            SensorPosition::new(0.0, 0.0, 500.0),
            SensorPosition::new(1000.0, 1000.0, 500.0),
        ];
        let emitter = SensorPosition::new(500.0, 500.0, 200.0);
        let measurements = generate_tdoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_tdoa(&measurements).unwrap();

        assert!(
            (result.position.x - 500.0).abs() < 10.0,
            "3D x error: {}",
            (result.position.x - 500.0).abs()
        );
        assert!(
            (result.position.y - 500.0).abs() < 10.0,
            "3D y error: {}",
            (result.position.y - 500.0).abs()
        );
        assert!(
            (result.position.z - 200.0).abs() < 10.0,
            "3D z error: {}",
            (result.position.z - 200.0).abs()
        );
    }

    #[test]
    fn test_3d_localization_aoa() {
        let sensors = vec![
            SensorPosition::new(0.0, 0.0, 0.0),
            SensorPosition::new(1000.0, 0.0, 0.0),
            SensorPosition::new(0.0, 1000.0, 0.0),
            SensorPosition::new(500.0, 500.0, 500.0),
        ];
        let emitter = SensorPosition::new(400.0, 300.0, 100.0);
        let measurements = generate_aoa_measurements(&sensors, &emitter);

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_aoa(&measurements).unwrap();

        assert!(
            (result.position.x - 400.0).abs() < 2.0,
            "3D AoA x error: {}",
            (result.position.x - 400.0).abs()
        );
        assert!(
            (result.position.y - 300.0).abs() < 2.0,
            "3D AoA y error: {}",
            (result.position.y - 300.0).abs()
        );
        assert!(
            (result.position.z - 100.0).abs() < 2.0,
            "3D AoA z error: {}",
            (result.position.z - 100.0).abs()
        );
    }

    #[test]
    fn test_confidence_weighting() {
        let sensors = square_sensors(1000.0);
        let emitter = SensorPosition::new(500.0, 500.0, 0.0);
        let mut measurements = generate_tdoa_measurements(&sensors, &emitter);

        for (i, m) in measurements.iter_mut().enumerate() {
            m.confidence = if i == 0 { 1.0 } else { 0.5 };
        }

        let localizer = EmitterLocalizer::new(sensors);
        let result = localizer.localize_tdoa(&measurements).unwrap();

        assert!(
            (result.position.x - 500.0).abs() < 5.0,
            "Weighted x error: {}",
            (result.position.x - 500.0).abs()
        );
    }

    #[test]
    fn test_localization_error_display() {
        let err = LocalizationError::InsufficientMeasurements {
            required: 3,
            provided: 1,
        };
        let msg = format!("{}", err);
        assert!(msg.contains("insufficient measurements"));
        assert!(msg.contains("3"));
        assert!(msg.contains("1"));

        let err2 = LocalizationError::SingularGeometry;
        let msg2 = format!("{}", err2);
        assert!(msg2.contains("singular"));
    }

    #[test]
    fn test_angle_diff_wrapping() {
        assert!((angle_diff(0.1, 2.0 * PI - 0.1) - 0.2).abs() < 1e-10);
        assert!((angle_diff(2.0 * PI - 0.1, 0.1) + 0.2).abs() < 1e-10);
        assert!((angle_diff(PI, -PI) - 0.0).abs() < 1e-10);
    }
}
