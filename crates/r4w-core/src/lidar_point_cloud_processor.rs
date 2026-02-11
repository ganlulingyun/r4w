//! LiDAR point cloud processing for terrain classification, ground filtering,
//! and feature extraction from time-of-flight laser scanning systems.
//!
//! Provides [`LidarProcessor`] for progressive morphological ground filtering,
//! digital surface/terrain model generation, surface normal estimation via PCA,
//! statistical outlier removal, voxel downsampling, and canopy height
//! computation. All math is implemented from scratch with no external
//! dependencies beyond `std`.
//!
//! # Example
//!
//! ```
//! use r4w_core::lidar_point_cloud_processor::{
//!     LidarConfig, LidarPoint, LidarProcessor, PointClass, PointCloud,
//!     compute_dsm, compute_dtm, compute_canopy_height, range_from_tof,
//! };
//!
//! let config = LidarConfig::default();
//! let processor = LidarProcessor::new(config);
//!
//! let mut cloud = PointCloud::new(vec![
//!     LidarPoint::new(0.0, 0.0, 1.0, 100.0, 1),
//!     LidarPoint::new(1.0, 1.0, 1.2, 90.0, 1),
//!     LidarPoint::new(2.0, 2.0, 10.0, 80.0, 1),
//! ]);
//!
//! processor.classify_ground(&mut cloud, 5.0, 1.0);
//!
//! let dsm = compute_dsm(&cloud, 5.0);
//! let dtm = compute_dtm(&cloud, 5.0);
//! let chm = compute_canopy_height(&dsm, &dtm);
//!
//! let tof_ns = 100.0;
//! let range = range_from_tof(tof_ns);
//! assert!((range - 14.9896229).abs() < 0.001);
//! ```

use std::collections::HashMap;

// ── constants ──────────────────────────────────────────────────────────

/// Speed of light in vacuum (m/s).
const C: f64 = 299_792_458.0;

/// Mathematical constant pi.
const PI: f64 = std::f64::consts::PI;

// ── enums ──────────────────────────────────────────────────────────────

/// ASPRS-style point classification codes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PointClass {
    Unclassified,
    Ground,
    Vegetation,
    Building,
    Water,
    Noise,
    Bridge,
}

impl Default for PointClass {
    fn default() -> Self {
        PointClass::Unclassified
    }
}

// ── core structs ───────────────────────────────────────────────────────

/// Configuration for a LiDAR sensor.
#[derive(Debug, Clone)]
pub struct LidarConfig {
    /// Maximum measurable range in metres.
    pub max_range_m: f64,
    /// Angular resolution in degrees.
    pub angular_resolution_deg: f64,
    /// Pulse repetition rate in Hz.
    pub pulse_rate_hz: f64,
    /// Laser wavelength in nanometres (default 905 nm).
    pub wavelength_nm: f64,
}

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            max_range_m: 200.0,
            angular_resolution_deg: 0.1,
            pulse_rate_hz: 300_000.0,
            wavelength_nm: 905.0,
        }
    }
}

/// A single LiDAR point with 3-D coordinates, intensity, return number, and
/// classification.
#[derive(Debug, Clone)]
pub struct LidarPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub intensity: f64,
    pub return_number: u8,
    pub classification: PointClass,
}

impl LidarPoint {
    /// Create a new unclassified point.
    pub fn new(x: f64, y: f64, z: f64, intensity: f64, return_number: u8) -> Self {
        Self {
            x,
            y,
            z,
            intensity,
            return_number,
            classification: PointClass::Unclassified,
        }
    }

    /// Create a point with an explicit classification.
    pub fn with_class(
        x: f64,
        y: f64,
        z: f64,
        intensity: f64,
        return_number: u8,
        classification: PointClass,
    ) -> Self {
        Self {
            x,
            y,
            z,
            intensity,
            return_number,
            classification,
        }
    }
}

/// Axis-aligned bounding box.
#[derive(Debug, Clone)]
pub struct BoundingBox {
    pub min_x: f64,
    pub min_y: f64,
    pub min_z: f64,
    pub max_x: f64,
    pub max_y: f64,
    pub max_z: f64,
}

/// A collection of LiDAR points.
#[derive(Debug, Clone)]
pub struct PointCloud {
    pub points: Vec<LidarPoint>,
}

impl PointCloud {
    pub fn new(points: Vec<LidarPoint>) -> Self {
        Self { points }
    }

    /// Number of points in the cloud.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the cloud is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Compute the axis-aligned bounding box. Returns `None` if empty.
    pub fn bounding_box(&self) -> Option<BoundingBox> {
        if self.points.is_empty() {
            return None;
        }
        let mut bb = BoundingBox {
            min_x: f64::INFINITY,
            min_y: f64::INFINITY,
            min_z: f64::INFINITY,
            max_x: f64::NEG_INFINITY,
            max_y: f64::NEG_INFINITY,
            max_z: f64::NEG_INFINITY,
        };
        for p in &self.points {
            if p.x < bb.min_x {
                bb.min_x = p.x;
            }
            if p.y < bb.min_y {
                bb.min_y = p.y;
            }
            if p.z < bb.min_z {
                bb.min_z = p.z;
            }
            if p.x > bb.max_x {
                bb.max_x = p.x;
            }
            if p.y > bb.max_y {
                bb.max_y = p.y;
            }
            if p.z > bb.max_z {
                bb.max_z = p.z;
            }
        }
        Some(bb)
    }

    /// Compute the centroid (mean x, y, z). Returns `None` if empty.
    pub fn centroid(&self) -> Option<(f64, f64, f64)> {
        if self.points.is_empty() {
            return None;
        }
        let n = self.points.len() as f64;
        let (sx, sy, sz) = self
            .points
            .iter()
            .fold((0.0, 0.0, 0.0), |(ax, ay, az), p| {
                (ax + p.x, ay + p.y, az + p.z)
            });
        Some((sx / n, sy / n, sz / n))
    }

    /// Return a new cloud containing only points of the given class.
    pub fn filter_by_class(&self, class: PointClass) -> PointCloud {
        PointCloud {
            points: self
                .points
                .iter()
                .filter(|p| p.classification == class)
                .cloned()
                .collect(),
        }
    }
}

// ── digital elevation models ───────────────────────────────────────────

/// Grid of highest-point elevations (Digital Surface Model).
#[derive(Debug, Clone)]
pub struct DigitalSurfaceModel {
    /// Row-major grid; `grid[row][col]`. `f64::NEG_INFINITY` means no data.
    pub grid: Vec<Vec<f64>>,
    pub cell_size_m: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

/// Grid of ground-point elevations (Digital Terrain Model).
#[derive(Debug, Clone)]
pub struct DigitalTerrainModel {
    /// Row-major grid; `grid[row][col]`. `f64::NEG_INFINITY` means no data.
    pub grid: Vec<Vec<f64>>,
    pub cell_size_m: f64,
    pub origin_x: f64,
    pub origin_y: f64,
}

// ── free functions ─────────────────────────────────────────────────────

/// Convert a round-trip time-of-flight (in nanoseconds) to one-way range (m).
///
/// range = c * tof / 2, with tof converted from ns to s.
pub fn range_from_tof(tof_ns: f64) -> f64 {
    C * tof_ns * 1e-9 / 2.0
}

/// Euclidean distance between two 3-D points.
pub fn euclidean_distance_3d(a: &LidarPoint, b: &LidarPoint) -> f64 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    let dz = a.z - b.z;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Compute a Digital Surface Model (highest z per cell).
pub fn compute_dsm(cloud: &PointCloud, cell_size_m: f64) -> DigitalSurfaceModel {
    let bb = match cloud.bounding_box() {
        Some(bb) => bb,
        None => {
            return DigitalSurfaceModel {
                grid: Vec::new(),
                cell_size_m,
                origin_x: 0.0,
                origin_y: 0.0,
            }
        }
    };

    let cols = ((bb.max_x - bb.min_x) / cell_size_m).ceil() as usize + 1;
    let rows = ((bb.max_y - bb.min_y) / cell_size_m).ceil() as usize + 1;
    let mut grid = vec![vec![f64::NEG_INFINITY; cols]; rows];

    for p in &cloud.points {
        let c = ((p.x - bb.min_x) / cell_size_m) as usize;
        let r = ((p.y - bb.min_y) / cell_size_m) as usize;
        let c = c.min(cols - 1);
        let r = r.min(rows - 1);
        if p.z > grid[r][c] {
            grid[r][c] = p.z;
        }
    }

    DigitalSurfaceModel {
        grid,
        cell_size_m,
        origin_x: bb.min_x,
        origin_y: bb.min_y,
    }
}

/// Compute a Digital Terrain Model (highest *ground-classified* z per cell).
pub fn compute_dtm(cloud: &PointCloud, cell_size_m: f64) -> DigitalTerrainModel {
    let bb = match cloud.bounding_box() {
        Some(bb) => bb,
        None => {
            return DigitalTerrainModel {
                grid: Vec::new(),
                cell_size_m,
                origin_x: 0.0,
                origin_y: 0.0,
            }
        }
    };

    let cols = ((bb.max_x - bb.min_x) / cell_size_m).ceil() as usize + 1;
    let rows = ((bb.max_y - bb.min_y) / cell_size_m).ceil() as usize + 1;
    let mut grid = vec![vec![f64::NEG_INFINITY; cols]; rows];

    for p in &cloud.points {
        if p.classification != PointClass::Ground {
            continue;
        }
        let c = ((p.x - bb.min_x) / cell_size_m) as usize;
        let r = ((p.y - bb.min_y) / cell_size_m) as usize;
        let c = c.min(cols - 1);
        let r = r.min(rows - 1);
        if p.z > grid[r][c] {
            grid[r][c] = p.z;
        }
    }

    DigitalTerrainModel {
        grid,
        cell_size_m,
        origin_x: bb.min_x,
        origin_y: bb.min_y,
    }
}

/// Canopy Height Model: CHM = DSM - DTM.
///
/// Both grids must have the same dimensions. Cells where either model has no
/// data yield 0.0.
pub fn compute_canopy_height(
    dsm: &DigitalSurfaceModel,
    dtm: &DigitalTerrainModel,
) -> Vec<Vec<f64>> {
    let rows = dsm.grid.len().min(dtm.grid.len());
    let mut chm = Vec::with_capacity(rows);
    for r in 0..rows {
        let cols = dsm.grid[r].len().min(dtm.grid[r].len());
        let mut row = Vec::with_capacity(cols);
        for c in 0..cols {
            let s = dsm.grid[r][c];
            let t = dtm.grid[r][c];
            if s == f64::NEG_INFINITY || t == f64::NEG_INFINITY {
                row.push(0.0);
            } else {
                row.push((s - t).max(0.0));
            }
        }
        chm.push(row);
    }
    chm
}

/// Estimate surface normals via PCA of `k_neighbors` nearest neighbours.
///
/// For each point, finds the k closest points, computes the 3x3 covariance
/// matrix of the neighbourhood, and extracts the eigenvector corresponding to
/// the smallest eigenvalue (the surface normal).
pub fn compute_normals(cloud: &PointCloud, k_neighbors: usize) -> Vec<(f64, f64, f64)> {
    let n = cloud.points.len();
    if n == 0 || k_neighbors == 0 {
        return Vec::new();
    }

    let mut normals = Vec::with_capacity(n);
    for i in 0..n {
        // Find k nearest neighbours (brute force).
        let mut dists: Vec<(usize, f64)> = (0..n)
            .filter(|&j| j != i)
            .map(|j| {
                let d = euclidean_distance_3d(&cloud.points[i], &cloud.points[j]);
                (j, d)
            })
            .collect();
        dists.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        let k = k_neighbors.min(dists.len());
        let neighbours: Vec<usize> = dists[..k].iter().map(|&(j, _)| j).collect();

        if neighbours.is_empty() {
            normals.push((0.0, 0.0, 1.0));
            continue;
        }

        // Compute centroid of neighbourhood (including the query point).
        let all: Vec<&LidarPoint> = std::iter::once(&cloud.points[i])
            .chain(neighbours.iter().map(|&j| &cloud.points[j]))
            .collect();
        let cnt = all.len() as f64;
        let cx: f64 = all.iter().map(|p| p.x).sum::<f64>() / cnt;
        let cy: f64 = all.iter().map(|p| p.y).sum::<f64>() / cnt;
        let cz: f64 = all.iter().map(|p| p.z).sum::<f64>() / cnt;

        // 3x3 covariance matrix (symmetric).
        let mut cov = [[0.0f64; 3]; 3];
        for p in &all {
            let dx = p.x - cx;
            let dy = p.y - cy;
            let dz = p.z - cz;
            cov[0][0] += dx * dx;
            cov[0][1] += dx * dy;
            cov[0][2] += dx * dz;
            cov[1][1] += dy * dy;
            cov[1][2] += dy * dz;
            cov[2][2] += dz * dz;
        }
        cov[1][0] = cov[0][1];
        cov[2][0] = cov[0][2];
        cov[2][1] = cov[1][2];

        // Find smallest eigenvector via analytical 3x3 symmetric eigendecomposition.
        let normal = smallest_eigenvector_3x3(&cov);
        normals.push(normal);
    }

    normals
}

/// Statistical outlier removal.
///
/// For each point, compute the mean distance to its k nearest neighbours. Then
/// compute the global mean and standard deviation of these mean distances.
/// Points whose mean-k-distance exceeds `global_mean + std_multiplier * std`
/// are classified as [`PointClass::Noise`].
pub fn statistical_outlier_removal(cloud: &mut PointCloud, k: usize, std_multiplier: f64) {
    let n = cloud.points.len();
    if n <= 1 || k == 0 {
        return;
    }

    // Compute mean k-distance for every point.
    let mean_dists: Vec<f64> = (0..n)
        .map(|i| {
            let mut dists: Vec<f64> = (0..n)
                .filter(|&j| j != i)
                .map(|j| euclidean_distance_3d(&cloud.points[i], &cloud.points[j]))
                .collect();
            dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let k_actual = k.min(dists.len());
            dists[..k_actual].iter().sum::<f64>() / k_actual as f64
        })
        .collect();

    let global_mean = mean_dists.iter().sum::<f64>() / n as f64;
    let variance = mean_dists
        .iter()
        .map(|d| (d - global_mean) * (d - global_mean))
        .sum::<f64>()
        / n as f64;
    let std_dev = variance.sqrt();
    let threshold = global_mean + std_multiplier * std_dev;

    for (i, &md) in mean_dists.iter().enumerate() {
        if md > threshold {
            cloud.points[i].classification = PointClass::Noise;
        }
    }
}

/// Voxel-grid downsampling.
///
/// Divides space into cubic voxels and keeps the centroid of all points within
/// each voxel.
pub fn voxel_downsample(cloud: &PointCloud, voxel_size_m: f64) -> PointCloud {
    if cloud.is_empty() || voxel_size_m <= 0.0 {
        return PointCloud::new(Vec::new());
    }

    // Key: (ix, iy, iz) voxel index.
    let mut voxels: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();

    for (idx, p) in cloud.points.iter().enumerate() {
        let ix = (p.x / voxel_size_m).floor() as i64;
        let iy = (p.y / voxel_size_m).floor() as i64;
        let iz = (p.z / voxel_size_m).floor() as i64;
        voxels.entry((ix, iy, iz)).or_default().push(idx);
    }

    let mut out = Vec::with_capacity(voxels.len());
    for (_key, indices) in &voxels {
        let cnt = indices.len() as f64;
        let mut sx = 0.0;
        let mut sy = 0.0;
        let mut sz = 0.0;
        let mut si = 0.0;
        for &i in indices {
            let p = &cloud.points[i];
            sx += p.x;
            sy += p.y;
            sz += p.z;
            si += p.intensity;
        }
        // Inherit return_number and classification from the first point in the
        // voxel.
        let first = &cloud.points[indices[0]];
        out.push(LidarPoint {
            x: sx / cnt,
            y: sy / cnt,
            z: sz / cnt,
            intensity: si / cnt,
            return_number: first.return_number,
            classification: first.classification,
        });
    }

    PointCloud::new(out)
}

// ── LidarProcessor ─────────────────────────────────────────────────────

/// High-level point-cloud processor tied to a sensor configuration.
pub struct LidarProcessor {
    pub config: LidarConfig,
}

impl LidarProcessor {
    pub fn new(config: LidarConfig) -> Self {
        Self { config }
    }

    /// Progressive morphological filter for ground classification.
    ///
    /// 1. Partition the cloud into a grid of cells.
    /// 2. Seed each cell's minimum-z point as ground.
    /// 3. Iteratively grow the ground set: a non-ground point whose height
    ///    above the nearest ground point (within the cell and its 8 neighbours)
    ///    is below `height_threshold_m` is reclassified as ground.
    /// 4. Repeat until no new ground points are added (or a fixed iteration
    ///    limit is reached).
    pub fn classify_ground(
        &self,
        cloud: &mut PointCloud,
        cell_size_m: f64,
        height_threshold_m: f64,
    ) {
        if cloud.is_empty() || cell_size_m <= 0.0 {
            return;
        }

        let bb = match cloud.bounding_box() {
            Some(bb) => bb,
            None => return,
        };

        let cols = ((bb.max_x - bb.min_x) / cell_size_m).ceil() as usize + 1;
        let rows = ((bb.max_y - bb.min_y) / cell_size_m).ceil() as usize + 1;

        // Assign each point to a cell.
        let cell_of: Vec<(usize, usize)> = cloud
            .points
            .iter()
            .map(|p| {
                let c = ((p.x - bb.min_x) / cell_size_m) as usize;
                let r = ((p.y - bb.min_y) / cell_size_m) as usize;
                (r.min(rows - 1), c.min(cols - 1))
            })
            .collect();

        // Build cell -> point-index map.
        let mut cell_points: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
        for (idx, &(r, c)) in cell_of.iter().enumerate() {
            cell_points.entry((r, c)).or_default().push(idx);
        }

        // Seed: minimum-z point in each cell.
        for indices in cell_points.values() {
            let min_idx = *indices
                .iter()
                .min_by(|&&a, &&b| {
                    cloud.points[a]
                        .z
                        .partial_cmp(&cloud.points[b].z)
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap();
            cloud.points[min_idx].classification = PointClass::Ground;
        }

        // Iterative growth.
        let max_iterations = 50;
        for _ in 0..max_iterations {
            let mut changed = false;

            for idx in 0..cloud.points.len() {
                if cloud.points[idx].classification == PointClass::Ground {
                    continue;
                }

                let (r, c) = cell_of[idx];
                let z = cloud.points[idx].z;

                // Search this cell and its 8 neighbours for the closest ground
                // point (in z).
                let mut min_ground_z = f64::INFINITY;
                for dr in -1i32..=1 {
                    for dc in -1i32..=1 {
                        let nr = r as i32 + dr;
                        let nc = c as i32 + dc;
                        if nr < 0 || nc < 0 || nr >= rows as i32 || nc >= cols as i32 {
                            continue;
                        }
                        if let Some(neighbours) =
                            cell_points.get(&(nr as usize, nc as usize))
                        {
                            for &ni in neighbours {
                                if cloud.points[ni].classification == PointClass::Ground
                                    && cloud.points[ni].z < min_ground_z
                                {
                                    min_ground_z = cloud.points[ni].z;
                                }
                            }
                        }
                    }
                }

                if min_ground_z < f64::INFINITY
                    && (z - min_ground_z).abs() <= height_threshold_m
                {
                    cloud.points[idx].classification = PointClass::Ground;
                    changed = true;
                }
            }

            if !changed {
                break;
            }
        }
    }

    /// Maximum unambiguous range (m) for the configured pulse rate.
    pub fn max_unambiguous_range(&self) -> f64 {
        C / (2.0 * self.config.pulse_rate_hz)
    }

    /// Angular resolution in radians.
    pub fn angular_resolution_rad(&self) -> f64 {
        self.config.angular_resolution_deg * PI / 180.0
    }
}

// ── internal helpers ───────────────────────────────────────────────────

/// Compute the eigenvector of a 3x3 real symmetric matrix corresponding to the
/// smallest eigenvalue using the closed-form cubic solution for 3x3 symmetric
/// matrices.
fn smallest_eigenvector_3x3(m: &[[f64; 3]; 3]) -> (f64, f64, f64) {
    // Eigenvalues via Cardano's formula for 3x3 symmetric matrix.
    let a = m[0][0];
    let b = m[1][1];
    let c = m[2][2];
    let d = m[0][1];
    let e = m[0][2];
    let f = m[1][2];

    // Characteristic polynomial: lambda^3 - p*lambda^2 + q*lambda - r = 0
    let p = a + b + c; // trace
    let q = a * b + a * c + b * c - d * d - e * e - f * f;
    let r = a * b * c + 2.0 * d * e * f - a * f * f - b * e * e - c * d * d; // determinant

    // Shift: let x = lambda - p/3
    let p3 = p / 3.0;
    let q2 = (p * p - 3.0 * q) / 9.0;
    let r2 = (2.0 * p * p * p - 9.0 * p * q + 27.0 * r) / 54.0;

    let q2_cubed = q2 * q2 * q2;
    let discriminant = r2 * r2 - q2_cubed;

    let eigenvalues = if discriminant < 0.0 {
        // Three real roots.
        let theta = (r2 / q2_cubed.sqrt()).acos();
        let sqrt_q2 = q2.sqrt();
        let e1 = -2.0 * sqrt_q2 * (theta / 3.0).cos() + p3;
        let e2 = -2.0 * sqrt_q2 * ((theta + 2.0 * PI) / 3.0).cos() + p3;
        let e3 = -2.0 * sqrt_q2 * ((theta - 2.0 * PI) / 3.0).cos() + p3;
        [e1, e2, e3]
    } else {
        // Degenerate or near-degenerate case. Fall back to default normal.
        return (0.0, 0.0, 1.0);
    };

    // Find the smallest eigenvalue.
    let mut min_ev = eigenvalues[0];
    for &ev in &eigenvalues[1..] {
        if ev < min_ev {
            min_ev = ev;
        }
    }

    // Compute eigenvector for min_ev via (M - lambda*I) null space using
    // cross-product of two rows.
    let shifted = [
        [m[0][0] - min_ev, m[0][1], m[0][2]],
        [m[1][0], m[1][1] - min_ev, m[1][2]],
        [m[2][0], m[2][1], m[2][2] - min_ev],
    ];

    // Cross product of row 0 and row 1.
    let mut nx = shifted[0][1] * shifted[1][2] - shifted[0][2] * shifted[1][1];
    let mut ny = shifted[0][2] * shifted[1][0] - shifted[0][0] * shifted[1][2];
    let mut nz = shifted[0][0] * shifted[1][1] - shifted[0][1] * shifted[1][0];

    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len < 1e-15 {
        // Try rows 0 and 2.
        nx = shifted[0][1] * shifted[2][2] - shifted[0][2] * shifted[2][1];
        ny = shifted[0][2] * shifted[2][0] - shifted[0][0] * shifted[2][2];
        nz = shifted[0][0] * shifted[2][1] - shifted[0][1] * shifted[2][0];
        let len2 = (nx * nx + ny * ny + nz * nz).sqrt();
        if len2 < 1e-15 {
            // Try rows 1 and 2.
            nx = shifted[1][1] * shifted[2][2] - shifted[1][2] * shifted[2][1];
            ny = shifted[1][2] * shifted[2][0] - shifted[1][0] * shifted[2][2];
            nz = shifted[1][0] * shifted[2][1] - shifted[1][1] * shifted[2][0];
            let len3 = (nx * nx + ny * ny + nz * nz).sqrt();
            if len3 < 1e-15 {
                return (0.0, 0.0, 1.0);
            }
            return (nx / len3, ny / len3, nz / len3);
        }
        return (nx / len2, ny / len2, nz / len2);
    }

    (nx / len, ny / len, nz / len)
}

// ── tests ──────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: create a flat-grid point cloud on z=0 for ground tests.
    fn flat_cloud(n: usize, spacing: f64) -> PointCloud {
        let mut pts = Vec::new();
        for i in 0..n {
            for j in 0..n {
                pts.push(LidarPoint::new(
                    i as f64 * spacing,
                    j as f64 * spacing,
                    0.0,
                    100.0,
                    1,
                ));
            }
        }
        PointCloud::new(pts)
    }

    // ── range_from_tof ────────────────────────────────────────────

    #[test]
    fn test_range_from_tof_basic() {
        // 100 ns round-trip -> ~14.99 m one-way
        let r = range_from_tof(100.0);
        assert!((r - 14.9896229).abs() < 0.001, "got {r}");
    }

    #[test]
    fn test_range_from_tof_zero() {
        assert_eq!(range_from_tof(0.0), 0.0);
    }

    #[test]
    fn test_range_from_tof_large() {
        // 1 microsecond -> ~149.9 m
        let r = range_from_tof(1000.0);
        assert!((r - 149.896229).abs() < 0.01, "got {r}");
    }

    // ── euclidean_distance_3d ─────────────────────────────────────

    #[test]
    fn test_distance_zero() {
        let a = LidarPoint::new(1.0, 2.0, 3.0, 0.0, 1);
        assert!((euclidean_distance_3d(&a, &a) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn test_distance_unit() {
        let a = LidarPoint::new(0.0, 0.0, 0.0, 0.0, 1);
        let b = LidarPoint::new(1.0, 0.0, 0.0, 0.0, 1);
        assert!((euclidean_distance_3d(&a, &b) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_distance_3d() {
        let a = LidarPoint::new(0.0, 0.0, 0.0, 0.0, 1);
        let b = LidarPoint::new(1.0, 2.0, 2.0, 0.0, 1);
        assert!((euclidean_distance_3d(&a, &b) - 3.0).abs() < 1e-12);
    }

    // ── bounding_box ──────────────────────────────────────────────

    #[test]
    fn test_bounding_box_empty() {
        let cloud = PointCloud::new(Vec::new());
        assert!(cloud.bounding_box().is_none());
    }

    #[test]
    fn test_bounding_box_single() {
        let cloud = PointCloud::new(vec![LidarPoint::new(5.0, 10.0, 15.0, 0.0, 1)]);
        let bb = cloud.bounding_box().unwrap();
        assert_eq!(bb.min_x, 5.0);
        assert_eq!(bb.max_x, 5.0);
        assert_eq!(bb.min_z, 15.0);
        assert_eq!(bb.max_z, 15.0);
    }

    #[test]
    fn test_bounding_box_multiple() {
        let cloud = PointCloud::new(vec![
            LidarPoint::new(-1.0, -2.0, -3.0, 0.0, 1),
            LidarPoint::new(4.0, 5.0, 6.0, 0.0, 1),
        ]);
        let bb = cloud.bounding_box().unwrap();
        assert_eq!(bb.min_x, -1.0);
        assert_eq!(bb.max_x, 4.0);
        assert_eq!(bb.min_y, -2.0);
        assert_eq!(bb.max_y, 5.0);
        assert_eq!(bb.min_z, -3.0);
        assert_eq!(bb.max_z, 6.0);
    }

    // ── centroid ──────────────────────────────────────────────────

    #[test]
    fn test_centroid_empty() {
        let cloud = PointCloud::new(Vec::new());
        assert!(cloud.centroid().is_none());
    }

    #[test]
    fn test_centroid_single() {
        let cloud = PointCloud::new(vec![LidarPoint::new(3.0, 4.0, 5.0, 0.0, 1)]);
        let (cx, cy, cz) = cloud.centroid().unwrap();
        assert!((cx - 3.0).abs() < 1e-12);
        assert!((cy - 4.0).abs() < 1e-12);
        assert!((cz - 5.0).abs() < 1e-12);
    }

    #[test]
    fn test_centroid_symmetric() {
        let cloud = PointCloud::new(vec![
            LidarPoint::new(-1.0, -1.0, 0.0, 0.0, 1),
            LidarPoint::new(1.0, 1.0, 0.0, 0.0, 1),
        ]);
        let (cx, cy, cz) = cloud.centroid().unwrap();
        assert!(cx.abs() < 1e-12);
        assert!(cy.abs() < 1e-12);
        assert!(cz.abs() < 1e-12);
    }

    // ── filter_by_class ───────────────────────────────────────────

    #[test]
    fn test_filter_by_class() {
        let cloud = PointCloud::new(vec![
            LidarPoint::with_class(0.0, 0.0, 0.0, 0.0, 1, PointClass::Ground),
            LidarPoint::with_class(1.0, 1.0, 5.0, 0.0, 1, PointClass::Vegetation),
            LidarPoint::with_class(2.0, 2.0, 0.1, 0.0, 1, PointClass::Ground),
        ]);
        let gnd = cloud.filter_by_class(PointClass::Ground);
        assert_eq!(gnd.len(), 2);
        let veg = cloud.filter_by_class(PointClass::Vegetation);
        assert_eq!(veg.len(), 1);
    }

    #[test]
    fn test_filter_by_class_empty_result() {
        let cloud = PointCloud::new(vec![
            LidarPoint::with_class(0.0, 0.0, 0.0, 0.0, 1, PointClass::Ground),
        ]);
        let water = cloud.filter_by_class(PointClass::Water);
        assert!(water.is_empty());
    }

    // ── DSM ───────────────────────────────────────────────────────

    #[test]
    fn test_dsm_empty() {
        let cloud = PointCloud::new(Vec::new());
        let dsm = compute_dsm(&cloud, 1.0);
        assert!(dsm.grid.is_empty());
    }

    #[test]
    fn test_dsm_single_cell() {
        let cloud = PointCloud::new(vec![
            LidarPoint::new(0.0, 0.0, 5.0, 0.0, 1),
            LidarPoint::new(0.1, 0.1, 10.0, 0.0, 1),
        ]);
        let dsm = compute_dsm(&cloud, 1.0);
        // Both points fall in the same cell; DSM should have the max z.
        assert!(!dsm.grid.is_empty());
        assert!((dsm.grid[0][0] - 10.0).abs() < 1e-9);
    }

    #[test]
    fn test_dsm_multiple_cells() {
        let cloud = PointCloud::new(vec![
            LidarPoint::new(0.0, 0.0, 1.0, 0.0, 1),
            LidarPoint::new(5.0, 0.0, 3.0, 0.0, 1),
            LidarPoint::new(5.0, 5.0, 7.0, 0.0, 1),
        ]);
        let dsm = compute_dsm(&cloud, 5.0);
        assert!(dsm.grid.len() >= 2);
        assert!(dsm.grid[0].len() >= 2);
    }

    // ── DTM ───────────────────────────────────────────────────────

    #[test]
    fn test_dtm_only_ground() {
        let cloud = PointCloud::new(vec![
            LidarPoint::with_class(0.0, 0.0, 2.0, 0.0, 1, PointClass::Ground),
            LidarPoint::with_class(0.1, 0.1, 10.0, 0.0, 1, PointClass::Vegetation),
        ]);
        let dtm = compute_dtm(&cloud, 1.0);
        // Only the ground point should contribute.
        assert!((dtm.grid[0][0] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_dtm_no_ground() {
        let cloud = PointCloud::new(vec![
            LidarPoint::with_class(0.0, 0.0, 5.0, 0.0, 1, PointClass::Vegetation),
        ]);
        let dtm = compute_dtm(&cloud, 1.0);
        // No ground points -> NEG_INFINITY.
        assert_eq!(dtm.grid[0][0], f64::NEG_INFINITY);
    }

    // ── canopy height ─────────────────────────────────────────────

    #[test]
    fn test_canopy_height_basic() {
        let cloud_gnd = PointCloud::new(vec![
            LidarPoint::with_class(0.0, 0.0, 1.0, 0.0, 1, PointClass::Ground),
            LidarPoint::with_class(0.0, 0.0, 8.0, 0.0, 1, PointClass::Vegetation),
        ]);
        let dsm = compute_dsm(&cloud_gnd, 1.0);
        let dtm = compute_dtm(&cloud_gnd, 1.0);
        let chm = compute_canopy_height(&dsm, &dtm);
        // DSM = 8, DTM = 1 -> CHM = 7
        assert!((chm[0][0] - 7.0).abs() < 1e-9);
    }

    #[test]
    fn test_canopy_height_no_data() {
        let dsm = DigitalSurfaceModel {
            grid: vec![vec![f64::NEG_INFINITY]],
            cell_size_m: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
        };
        let dtm = DigitalTerrainModel {
            grid: vec![vec![1.0]],
            cell_size_m: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
        };
        let chm = compute_canopy_height(&dsm, &dtm);
        assert_eq!(chm[0][0], 0.0);
    }

    // ── statistical outlier removal ───────────────────────────────

    #[test]
    fn test_sor_removes_outlier() {
        let mut pts = vec![
            LidarPoint::new(0.0, 0.0, 0.0, 100.0, 1),
            LidarPoint::new(1.0, 0.0, 0.0, 100.0, 1),
            LidarPoint::new(0.0, 1.0, 0.0, 100.0, 1),
            LidarPoint::new(1.0, 1.0, 0.0, 100.0, 1),
            // Outlier far away.
            LidarPoint::new(100.0, 100.0, 100.0, 100.0, 1),
        ];
        let mut cloud = PointCloud::new(pts);
        statistical_outlier_removal(&mut cloud, 2, 1.0);
        let noise = cloud.filter_by_class(PointClass::Noise);
        assert_eq!(noise.len(), 1, "outlier should be classified as noise");
        assert!((noise.points[0].x - 100.0).abs() < 1e-9);
    }

    #[test]
    fn test_sor_no_outliers() {
        let mut cloud = flat_cloud(3, 1.0);
        statistical_outlier_removal(&mut cloud, 3, 3.0);
        let noise = cloud.filter_by_class(PointClass::Noise);
        assert_eq!(noise.len(), 0);
    }

    #[test]
    fn test_sor_empty_cloud() {
        let mut cloud = PointCloud::new(Vec::new());
        statistical_outlier_removal(&mut cloud, 3, 1.0);
        assert!(cloud.is_empty());
    }

    // ── voxel downsampling ────────────────────────────────────────

    #[test]
    fn test_voxel_downsample_reduces_points() {
        // 9 points in a 3x3 grid with spacing 0.1 -> all in one 1m voxel.
        let cloud = flat_cloud(3, 0.1);
        let ds = voxel_downsample(&cloud, 1.0);
        assert_eq!(ds.len(), 1, "all points should merge into one voxel");
    }

    #[test]
    fn test_voxel_downsample_preserves_separated() {
        // Points separated by more than voxel size.
        let cloud = PointCloud::new(vec![
            LidarPoint::new(0.0, 0.0, 0.0, 100.0, 1),
            LidarPoint::new(10.0, 10.0, 10.0, 100.0, 1),
        ]);
        let ds = voxel_downsample(&cloud, 1.0);
        assert_eq!(ds.len(), 2);
    }

    #[test]
    fn test_voxel_downsample_centroid() {
        let cloud = PointCloud::new(vec![
            LidarPoint::new(0.0, 0.0, 0.0, 100.0, 1),
            LidarPoint::new(0.2, 0.2, 0.2, 100.0, 1),
        ]);
        let ds = voxel_downsample(&cloud, 1.0);
        assert_eq!(ds.len(), 1);
        assert!((ds.points[0].x - 0.1).abs() < 1e-9);
        assert!((ds.points[0].y - 0.1).abs() < 1e-9);
        assert!((ds.points[0].z - 0.1).abs() < 1e-9);
    }

    #[test]
    fn test_voxel_downsample_empty() {
        let cloud = PointCloud::new(Vec::new());
        let ds = voxel_downsample(&cloud, 1.0);
        assert!(ds.is_empty());
    }

    // ── ground classification ─────────────────────────────────────

    #[test]
    fn test_classify_ground_flat() {
        let config = LidarConfig::default();
        let proc = LidarProcessor::new(config);

        let mut cloud = flat_cloud(3, 1.0);
        proc.classify_ground(&mut cloud, 5.0, 0.5);
        // All points at z=0 should be ground.
        for p in &cloud.points {
            assert_eq!(p.classification, PointClass::Ground);
        }
    }

    #[test]
    fn test_classify_ground_with_elevated() {
        let config = LidarConfig::default();
        let proc = LidarProcessor::new(config);

        let mut cloud = PointCloud::new(vec![
            LidarPoint::new(0.0, 0.0, 0.0, 100.0, 1),
            LidarPoint::new(1.0, 0.0, 0.3, 100.0, 1), // slightly above -> ground
            LidarPoint::new(2.0, 0.0, 15.0, 100.0, 1), // far above -> not ground
        ]);
        proc.classify_ground(&mut cloud, 5.0, 1.0);

        assert_eq!(cloud.points[0].classification, PointClass::Ground);
        assert_eq!(cloud.points[1].classification, PointClass::Ground);
        assert_ne!(cloud.points[2].classification, PointClass::Ground);
    }

    #[test]
    fn test_classify_ground_empty() {
        let config = LidarConfig::default();
        let proc = LidarProcessor::new(config);
        let mut cloud = PointCloud::new(Vec::new());
        proc.classify_ground(&mut cloud, 1.0, 1.0);
        assert!(cloud.is_empty());
    }

    // ── normal estimation ─────────────────────────────────────────

    #[test]
    fn test_normals_flat_plane() {
        // Points on z=0 plane -> normal should be close to (0,0,+/-1).
        let cloud = flat_cloud(4, 1.0);
        let normals = compute_normals(&cloud, 4);
        assert_eq!(normals.len(), cloud.len());
        for (nx, ny, nz) in &normals {
            // The magnitude of z-component should dominate.
            assert!(
                nz.abs() > 0.9,
                "expected z-dominant normal, got ({nx}, {ny}, {nz})"
            );
        }
    }

    #[test]
    fn test_normals_empty() {
        let cloud = PointCloud::new(Vec::new());
        let normals = compute_normals(&cloud, 3);
        assert!(normals.is_empty());
    }

    #[test]
    fn test_normals_unit_length() {
        let cloud = flat_cloud(3, 1.0);
        let normals = compute_normals(&cloud, 3);
        for (nx, ny, nz) in &normals {
            let len = (nx * nx + ny * ny + nz * nz).sqrt();
            assert!(
                (len - 1.0).abs() < 1e-6,
                "normal not unit length: {len}"
            );
        }
    }

    // ── LidarProcessor misc ──────────────────────────────────────

    #[test]
    fn test_max_unambiguous_range() {
        let config = LidarConfig {
            pulse_rate_hz: 300_000.0,
            ..Default::default()
        };
        let proc = LidarProcessor::new(config);
        let r = proc.max_unambiguous_range();
        // c / (2 * 300e3) ~ 499.65 m
        assert!((r - 499.654).abs() < 1.0, "got {r}");
    }

    #[test]
    fn test_angular_resolution_rad() {
        let config = LidarConfig {
            angular_resolution_deg: 0.1,
            ..Default::default()
        };
        let proc = LidarProcessor::new(config);
        let rad = proc.angular_resolution_rad();
        assert!((rad - 0.001745).abs() < 0.0001, "got {rad}");
    }

    // ── LidarConfig default ──────────────────────────────────────

    #[test]
    fn test_config_default_wavelength() {
        let c = LidarConfig::default();
        assert_eq!(c.wavelength_nm, 905.0);
    }

    // ── PointClass default ───────────────────────────────────────

    #[test]
    fn test_point_class_default() {
        let cls: PointClass = Default::default();
        assert_eq!(cls, PointClass::Unclassified);
    }

    // ── PointCloud::len / is_empty ───────────────────────────────

    #[test]
    fn test_cloud_len() {
        let cloud = PointCloud::new(vec![
            LidarPoint::new(0.0, 0.0, 0.0, 0.0, 1),
            LidarPoint::new(1.0, 1.0, 1.0, 0.0, 1),
        ]);
        assert_eq!(cloud.len(), 2);
        assert!(!cloud.is_empty());
    }

    #[test]
    fn test_cloud_is_empty() {
        let cloud = PointCloud::new(Vec::new());
        assert!(cloud.is_empty());
        assert_eq!(cloud.len(), 0);
    }

    // ── edge cases ───────────────────────────────────────────────

    #[test]
    fn test_voxel_downsample_zero_size() {
        let cloud = flat_cloud(2, 1.0);
        let ds = voxel_downsample(&cloud, 0.0);
        assert!(ds.is_empty(), "zero voxel size should return empty");
    }

    #[test]
    fn test_dsm_dtm_same_dimensions() {
        let mut cloud = PointCloud::new(vec![
            LidarPoint::with_class(0.0, 0.0, 1.0, 0.0, 1, PointClass::Ground),
            LidarPoint::with_class(0.0, 0.0, 5.0, 0.0, 1, PointClass::Vegetation),
            LidarPoint::with_class(2.0, 2.0, 0.5, 0.0, 1, PointClass::Ground),
            LidarPoint::with_class(2.0, 2.0, 8.0, 0.0, 1, PointClass::Building),
        ]);
        let dsm = compute_dsm(&cloud, 1.0);
        let dtm = compute_dtm(&cloud, 1.0);
        assert_eq!(dsm.grid.len(), dtm.grid.len());
        assert_eq!(dsm.grid[0].len(), dtm.grid[0].len());
    }

    #[test]
    fn test_canopy_height_nonnegative() {
        // Even if DTM > DSM for some reason, CHM clamps to 0.
        let dsm = DigitalSurfaceModel {
            grid: vec![vec![2.0]],
            cell_size_m: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
        };
        let dtm = DigitalTerrainModel {
            grid: vec![vec![5.0]],
            cell_size_m: 1.0,
            origin_x: 0.0,
            origin_y: 0.0,
        };
        let chm = compute_canopy_height(&dsm, &dtm);
        assert_eq!(chm[0][0], 0.0);
    }

    #[test]
    fn test_with_class_constructor() {
        let p = LidarPoint::with_class(1.0, 2.0, 3.0, 50.0, 2, PointClass::Bridge);
        assert_eq!(p.classification, PointClass::Bridge);
        assert_eq!(p.return_number, 2);
        assert!((p.intensity - 50.0).abs() < 1e-12);
    }
}
