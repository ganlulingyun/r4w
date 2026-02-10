//! Multi-station Radio Direction Finding (RDF) network fusion for emitter geolocation.
//!
//! This module implements bearing-only triangulation from multiple RDF stations,
//! including least-squares fixes, Stansfield maximum-likelihood estimation,
//! line-of-bearing (LOB) intersection, weighted bearing fusion, confidence
//! ellipse computation, circular error probable (CEP) calculation, multi-target
//! resolution, and bearing error modeling.
//!
//! Unlike single-station bearing estimation (e.g., Watson-Watt), this module
//! focuses on fusing bearings from a geographically distributed network of
//! stations to produce a geolocation fix.
//!
//! # Example
//!
//! ```
//! use r4w_core::rdf_network_triangulator::{RdfStation, triangulate_bearings};
//!
//! let stations = vec![
//!     RdfStation {
//!         latitude_deg: 38.0,
//!         longitude_deg: -77.0,
//!         bearing_deg: 45.0,
//!         bearing_error_deg: 2.0,
//!     },
//!     RdfStation {
//!         latitude_deg: 39.0,
//!         longitude_deg: -76.0,
//!         bearing_deg: 225.0,
//!         bearing_error_deg: 2.0,
//!     },
//!     RdfStation {
//!         latitude_deg: 38.5,
//!         longitude_deg: -78.0,
//!         bearing_deg: 90.0,
//!         bearing_error_deg: 3.0,
//!     },
//! ];
//!
//! let fix = triangulate_bearings(&stations).unwrap();
//! assert!(fix.latitude_deg > 37.0 && fix.latitude_deg < 40.0);
//! assert!(fix.longitude_deg > -79.0 && fix.longitude_deg < -75.0);
//! assert!(fix.num_bearings == 3);
//! ```

use std::f64::consts::PI;

/// Earth's mean radius in kilometers (WGS-84 volumetric mean).
const EARTH_RADIUS_KM: f64 = 6371.0;

/// An RDF station observation: a geographic position and the measured bearing
/// toward an emitter, along with the estimated bearing error.
#[derive(Debug, Clone, PartialEq)]
pub struct RdfStation {
    /// Station latitude in degrees (positive north).
    pub latitude_deg: f64,
    /// Station longitude in degrees (positive east).
    pub longitude_deg: f64,
    /// Measured bearing from station toward the emitter, in degrees true north
    /// (0 = north, 90 = east, clockwise).
    pub bearing_deg: f64,
    /// One-sigma bearing measurement error in degrees.
    pub bearing_error_deg: f64,
}

/// A geolocation fix produced by bearing fusion.
#[derive(Debug, Clone, PartialEq)]
pub struct GeoFix {
    /// Estimated emitter latitude in degrees.
    pub latitude_deg: f64,
    /// Estimated emitter longitude in degrees.
    pub longitude_deg: f64,
    /// Circular Error Probable in kilometers (50% containment radius).
    pub cep_km: f64,
    /// Confidence percentage associated with the fix (0.0..100.0).
    pub confidence_pct: f64,
    /// Number of bearings used to compute this fix.
    pub num_bearings: usize,
}

/// A network of RDF stations with fixed positions.
///
/// `RdfNetwork` stores station configurations (positions) separately from
/// per-observation bearings so that the same network can process many
/// observation sets.
#[derive(Debug, Clone)]
pub struct RdfNetwork {
    /// Fixed station positions (latitude/longitude).
    stations: Vec<(f64, f64)>,
}

impl RdfNetwork {
    /// Create a new RDF network from station positions.
    ///
    /// Each tuple is `(latitude_deg, longitude_deg)`.
    pub fn new(stations: &[(f64, f64)]) -> Self {
        Self {
            stations: stations.to_vec(),
        }
    }

    /// Return the number of stations in the network.
    pub fn num_stations(&self) -> usize {
        self.stations.len()
    }

    /// Build `RdfStation` observations from a set of measured bearings and
    /// uniform error.
    ///
    /// `bearings_deg` must have the same length as the number of stations.
    /// Returns `None` if lengths mismatch.
    pub fn observe(
        &self,
        bearings_deg: &[f64],
        bearing_error_deg: f64,
    ) -> Option<Vec<RdfStation>> {
        if bearings_deg.len() != self.stations.len() {
            return None;
        }
        Some(
            self.stations
                .iter()
                .zip(bearings_deg.iter())
                .map(|(&(lat, lon), &brg)| RdfStation {
                    latitude_deg: lat,
                    longitude_deg: lon,
                    bearing_deg: brg,
                    bearing_error_deg,
                })
                .collect(),
        )
    }
}

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

/// Convert degrees to radians.
fn deg2rad(d: f64) -> f64 {
    d * PI / 180.0
}

/// Convert radians to degrees.
fn rad2deg(r: f64) -> f64 {
    r * 180.0 / PI
}

/// Normalize an angle to the range [0, 360).
fn normalize_deg(a: f64) -> f64 {
    ((a % 360.0) + 360.0) % 360.0
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Compute the great-circle initial bearing from one point to another.
///
/// All inputs in degrees; returns bearing in degrees [0, 360).
pub fn bearing_from_coords(from_lat: f64, from_lon: f64, to_lat: f64, to_lon: f64) -> f64 {
    let lat1 = deg2rad(from_lat);
    let lat2 = deg2rad(to_lat);
    let dlon = deg2rad(to_lon - from_lon);

    let x = dlon.sin() * lat2.cos();
    let y = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();

    let bearing_rad = x.atan2(y);
    normalize_deg(rad2deg(bearing_rad))
}

/// Compute haversine distance between two points in kilometers.
///
/// All inputs in degrees.
pub fn distance_km(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let dlat = deg2rad(lat2 - lat1);
    let dlon = deg2rad(lon2 - lon1);
    let lat1r = deg2rad(lat1);
    let lat2r = deg2rad(lat2);

    let a = (dlat / 2.0).sin().powi(2) + lat1r.cos() * lat2r.cos() * (dlon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
    EARTH_RADIUS_KM * c
}

/// Compute the intersection of two lines of bearing (LOBs).
///
/// Returns `Some((latitude_deg, longitude_deg))` of the intersection, or
/// `None` if the bearings are nearly parallel (angular separation < ~0.006°).
pub fn lob_intersection(s1: &RdfStation, s2: &RdfStation) -> Option<(f64, f64)> {
    // Use a flat-earth Cartesian approximation for the intersection, then
    // convert back. This is accurate for station separations up to a few
    // hundred km at mid-latitudes.

    let lat1 = deg2rad(s1.latitude_deg);
    let lon1 = deg2rad(s1.longitude_deg);
    let lat2 = deg2rad(s2.latitude_deg);
    let lon2 = deg2rad(s2.longitude_deg);

    let brg1 = deg2rad(s1.bearing_deg);
    let brg2 = deg2rad(s2.bearing_deg);

    // Direction unit vectors in a local tangent plane (north = +y, east = +x)
    let dx1 = brg1.sin();
    let dy1 = brg1.cos();
    let dx2 = brg2.sin();
    let dy2 = brg2.cos();

    // Check for near-parallel bearings
    let cross = dx1 * dy2 - dy1 * dx2;
    if cross.abs() < 1e-4 {
        return None;
    }

    // Station positions in a local Cartesian frame centred on s1
    let mean_lat = (lat1 + lat2) / 2.0;
    let cos_lat = mean_lat.cos();

    let x2_km = (lon2 - lon1) * cos_lat * EARTH_RADIUS_KM;
    let y2_km = (lat2 - lat1) * EARTH_RADIUS_KM;

    // Intersection: P1 + t1*d1 = P2 + t2*d2
    // Solving for t1: t1 = (x2*dy2 - y2*dx2) / (dx1*dy2 - dy1*dx2)
    let t1 = (x2_km * dy2 - y2_km * dx2) / cross;

    // Intersection point in km from s1
    let ix_km = t1 * dx1;
    let iy_km = t1 * dy1;

    // Convert back to lat/lon
    let int_lat = s1.latitude_deg + rad2deg(iy_km / EARTH_RADIUS_KM);
    let int_lon = s1.longitude_deg + rad2deg(ix_km / (EARTH_RADIUS_KM * cos_lat));

    Some((int_lat, int_lon))
}

/// Compute a weighted circular mean of bearings.
///
/// `bearings_deg` and `weights` must have the same length (and at least one
/// element). Returns the fused bearing in degrees [0, 360).
///
/// Uses the vector-sum method on the unit circle to properly handle the
/// 0 deg / 360 deg wraparound.
pub fn weighted_bearing_fusion(bearings_deg: &[f64], weights: &[f64]) -> f64 {
    assert_eq!(
        bearings_deg.len(),
        weights.len(),
        "bearings and weights must have equal length"
    );
    assert!(!bearings_deg.is_empty(), "need at least one bearing");

    let mut sx = 0.0_f64;
    let mut sy = 0.0_f64;
    for (&b, &w) in bearings_deg.iter().zip(weights.iter()) {
        let r = deg2rad(b);
        sx += w * r.sin();
        sy += w * r.cos();
    }

    normalize_deg(rad2deg(sx.atan2(sy)))
}

/// Least-squares bearing-only triangulation (iterative weighted LS).
///
/// Given two or more station observations, computes the emitter position by
/// iterating a weighted least-squares solution over bearing residuals.
///
/// Returns `None` if fewer than two stations are provided or the geometry is
/// degenerate (e.g., all bearings parallel).
pub fn triangulate_bearings(stations: &[RdfStation]) -> Option<GeoFix> {
    if stations.len() < 2 {
        return None;
    }

    // Collect all pair-wise LOB intersections as initial candidates.
    let mut lats = Vec::new();
    let mut lons = Vec::new();
    for i in 0..stations.len() {
        for j in (i + 1)..stations.len() {
            if let Some((lat, lon)) = lob_intersection(&stations[i], &stations[j]) {
                lats.push(lat);
                lons.push(lon);
            }
        }
    }

    if lats.is_empty() {
        return None;
    }

    // Initial guess: centroid of intersections
    let n = lats.len() as f64;
    let mut est_lat = lats.iter().sum::<f64>() / n;
    let mut est_lon = lons.iter().sum::<f64>() / n;

    // Iterative weighted least-squares refinement
    for _iter in 0..20 {
        let mut ata = [[0.0f64; 2]; 2];
        let mut atb = [0.0f64; 2];

        let cos_lat = deg2rad(est_lat).cos();

        for s in stations {
            let dx_km =
                (deg2rad(est_lon) - deg2rad(s.longitude_deg)) * cos_lat * EARTH_RADIUS_KM;
            let dy_km = (deg2rad(est_lat) - deg2rad(s.latitude_deg)) * EARTH_RADIUS_KM;

            let pred_bearing = dx_km.atan2(dy_km); // radians, 0=north
            let meas_bearing = deg2rad(s.bearing_deg);

            // Angular residual (wrapped to [-pi, pi])
            let mut residual = meas_bearing - pred_bearing;
            while residual > PI {
                residual -= 2.0 * PI;
            }
            while residual < -PI {
                residual += 2.0 * PI;
            }

            let range_km = (dx_km * dx_km + dy_km * dy_km).sqrt().max(0.01);

            // Jacobian of bearing w.r.t. (east_km, north_km)
            let j_east = dy_km / (range_km * range_km);
            let j_north = -dx_km / (range_km * range_km);

            // Weight = 1/sigma^2
            let sigma = deg2rad(s.bearing_error_deg).max(1e-6);
            let w = 1.0 / (sigma * sigma);

            ata[0][0] += w * j_east * j_east;
            ata[0][1] += w * j_east * j_north;
            ata[1][0] += w * j_north * j_east;
            ata[1][1] += w * j_north * j_north;

            atb[0] += w * j_east * residual;
            atb[1] += w * j_north * residual;
        }

        // Solve 2x2 system
        let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
        if det.abs() < 1e-30 {
            break;
        }

        let d_east = (ata[1][1] * atb[0] - ata[0][1] * atb[1]) / det;
        let d_north = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;

        est_lon += rad2deg(d_east / (EARTH_RADIUS_KM * cos_lat));
        est_lat += rad2deg(d_north / EARTH_RADIUS_KM);

        if d_east.abs() < 1e-6 && d_north.abs() < 1e-6 {
            break;
        }
    }

    // Compute CEP from residual scatter of LOB intersections
    let cep = if lats.len() >= 2 {
        let mut dists: Vec<f64> = lats
            .iter()
            .zip(lons.iter())
            .map(|(&la, &lo)| distance_km(est_lat, est_lon, la, lo))
            .collect();
        dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let idx = (dists.len() as f64 * 0.5).ceil() as usize;
        dists[idx.min(dists.len() - 1)]
    } else {
        // Fallback: use mean bearing error and average range
        let avg_err: f64 = stations.iter().map(|s| s.bearing_error_deg).sum::<f64>()
            / stations.len() as f64;
        let avg_range: f64 = stations
            .iter()
            .map(|s| distance_km(s.latitude_deg, s.longitude_deg, est_lat, est_lon))
            .sum::<f64>()
            / stations.len() as f64;
        avg_range * deg2rad(avg_err).tan()
    };

    // Confidence: based on number of bearings and geometry
    let confidence = ((stations.len() as f64 - 1.0) / stations.len() as f64 * 100.0).min(99.0);

    Some(GeoFix {
        latitude_deg: est_lat,
        longitude_deg: est_lon,
        cep_km: cep,
        confidence_pct: confidence,
        num_bearings: stations.len(),
    })
}

/// Stansfield maximum-likelihood bearing-only estimator.
///
/// Uses an explicit Stansfield formulation that directly minimises the sum of
/// weighted squared bearing residuals. The Stansfield weighting is
/// `r_i^2 / sigma_i^2` which accounts for the geometric dilution of precision
/// at longer ranges.
///
/// Returns `None` if fewer than two stations are provided.
pub fn stansfield_estimate(stations: &[RdfStation]) -> Option<GeoFix> {
    if stations.len() < 2 {
        return None;
    }

    // Get initial estimate from LOB intersections
    let mut init_lat;
    let mut init_lon;
    {
        let mut lats = Vec::new();
        let mut lons = Vec::new();
        for i in 0..stations.len() {
            for j in (i + 1)..stations.len() {
                if let Some((la, lo)) = lob_intersection(&stations[i], &stations[j]) {
                    lats.push(la);
                    lons.push(lo);
                }
            }
        }
        if lats.is_empty() {
            return None;
        }
        let n = lats.len() as f64;
        init_lat = lats.iter().sum::<f64>() / n;
        init_lon = lons.iter().sum::<f64>() / n;
    }

    // Stansfield iteration
    for _iter in 0..30 {
        let cos_lat = deg2rad(init_lat).cos();

        let mut h_sum = [[0.0f64; 2]; 2]; // H^T W H
        let mut g_sum = [0.0f64; 2]; // H^T W r

        for s in stations {
            let dx = (deg2rad(init_lon) - deg2rad(s.longitude_deg)) * cos_lat * EARTH_RADIUS_KM;
            let dy = (deg2rad(init_lat) - deg2rad(s.latitude_deg)) * EARTH_RADIUS_KM;
            let r2 = dx * dx + dy * dy;
            let r = r2.sqrt().max(0.01);
            let _ = r; // used implicitly via r2

            let pred = dx.atan2(dy);
            let meas = deg2rad(s.bearing_deg);
            let mut res = meas - pred;
            while res > PI {
                res -= 2.0 * PI;
            }
            while res < -PI {
                res += 2.0 * PI;
            }

            // Stansfield weight: w_i = r_i^2 / sigma_i^2
            let sigma = deg2rad(s.bearing_error_deg).max(1e-6);
            let w = r2.max(1e-4) / (sigma * sigma);

            // Partial derivatives of bearing w.r.t. (x, y)
            let r2_safe = r2.max(1e-4);
            let hx = dy / r2_safe;
            let hy = -dx / r2_safe;

            h_sum[0][0] += w * hx * hx;
            h_sum[0][1] += w * hx * hy;
            h_sum[1][0] += w * hy * hx;
            h_sum[1][1] += w * hy * hy;

            g_sum[0] += w * hx * res;
            g_sum[1] += w * hy * res;
        }

        let det = h_sum[0][0] * h_sum[1][1] - h_sum[0][1] * h_sum[1][0];
        if det.abs() < 1e-30 {
            break;
        }

        let d_east = (h_sum[1][1] * g_sum[0] - h_sum[0][1] * g_sum[1]) / det;
        let d_north = (h_sum[0][0] * g_sum[1] - h_sum[1][0] * g_sum[0]) / det;

        init_lon += rad2deg(d_east / (EARTH_RADIUS_KM * cos_lat));
        init_lat += rad2deg(d_north / EARTH_RADIUS_KM);

        if d_east.abs() < 1e-6 && d_north.abs() < 1e-6 {
            break;
        }
    }

    // CEP estimate from Stansfield covariance
    let cos_lat = deg2rad(init_lat).cos();
    let mut cov = [[0.0f64; 2]; 2];
    for s in stations {
        let dx = (deg2rad(init_lon) - deg2rad(s.longitude_deg)) * cos_lat * EARTH_RADIUS_KM;
        let dy = (deg2rad(init_lat) - deg2rad(s.latitude_deg)) * EARTH_RADIUS_KM;
        let r2 = (dx * dx + dy * dy).max(1e-4);
        let sigma = deg2rad(s.bearing_error_deg).max(1e-6);
        let w = r2 / (sigma * sigma);

        let hx = dy / r2;
        let hy = -dx / r2;

        cov[0][0] += w * hx * hx;
        cov[0][1] += w * hx * hy;
        cov[1][0] += w * hy * hx;
        cov[1][1] += w * hy * hy;
    }

    let det = cov[0][0] * cov[1][1] - cov[0][1] * cov[1][0];
    let cep = if det.abs() > 1e-30 {
        // Inverse covariance -> covariance
        let var_x = cov[1][1] / det;
        let var_y = cov[0][0] / det;
        // CEP ~ 0.6745 * sqrt(average variance) for circular-normal
        0.6745 * ((var_x + var_y) / 2.0).sqrt()
    } else {
        f64::INFINITY
    };

    let confidence = ((stations.len() as f64 - 1.0) / stations.len() as f64 * 100.0).min(99.0);

    Some(GeoFix {
        latitude_deg: init_lat,
        longitude_deg: init_lon,
        cep_km: cep,
        confidence_pct: confidence,
        num_bearings: stations.len(),
    })
}

/// Compute the confidence ellipse for a bearing fix.
///
/// Returns `(semi_major_km, semi_minor_km, orientation_deg)` where
/// `orientation_deg` is the clockwise angle from north to the semi-major axis.
/// The ellipse corresponds to a 95% confidence region (chi-squared with 2 DOF).
pub fn confidence_ellipse(stations: &[RdfStation], fix: &GeoFix) -> (f64, f64, f64) {
    if stations.is_empty() {
        return (f64::INFINITY, f64::INFINITY, 0.0);
    }

    let cos_lat = deg2rad(fix.latitude_deg).cos();

    // Build the Fisher information matrix (bearing-only)
    let mut fim = [[0.0f64; 2]; 2];
    for s in stations {
        let dx = (deg2rad(fix.longitude_deg) - deg2rad(s.longitude_deg))
            * cos_lat
            * EARTH_RADIUS_KM;
        let dy =
            (deg2rad(fix.latitude_deg) - deg2rad(s.latitude_deg)) * EARTH_RADIUS_KM;
        let r2 = (dx * dx + dy * dy).max(1e-4);
        let sigma = deg2rad(s.bearing_error_deg).max(1e-6);

        let hx = dy / r2;
        let hy = -dx / r2;

        let w = 1.0 / (sigma * sigma);
        fim[0][0] += w * hx * hx;
        fim[0][1] += w * hx * hy;
        fim[1][0] += w * hy * hx;
        fim[1][1] += w * hy * hy;
    }

    let det = fim[0][0] * fim[1][1] - fim[0][1] * fim[1][0];
    if det.abs() < 1e-30 {
        return (f64::INFINITY, f64::INFINITY, 0.0);
    }

    // Covariance = FIM^{-1}
    let cxx = fim[1][1] / det;
    let cyy = fim[0][0] / det;
    let cxy = -fim[0][1] / det;

    // Eigenvalues of 2x2 covariance
    let trace = cxx + cyy;
    let disc = ((cxx - cyy).powi(2) + 4.0 * cxy * cxy).sqrt();
    let lambda1 = (trace + disc) / 2.0;
    let lambda2 = (trace - disc) / 2.0;

    // 95% confidence chi-squared with 2 dof = 5.991
    let chi2_95 = 5.991_f64;
    let semi_major = (chi2_95 * lambda1.abs()).sqrt();
    let semi_minor = (chi2_95 * lambda2.abs()).sqrt();

    // Orientation: angle of eigenvector for lambda1
    let angle_rad = (2.0 * cxy).atan2(cxx - cyy) / 2.0;
    let orientation = normalize_deg(rad2deg(angle_rad));

    (semi_major, semi_minor, orientation)
}

/// Calculate the Circular Error Probable (CEP) from a set of fixes against a
/// known true position.
///
/// CEP is the radius of a circle centered on `true_pos` that contains 50% of
/// the fix positions. Returns the CEP in kilometers.
pub fn circular_error_probable(fixes: &[(f64, f64)], true_pos: (f64, f64)) -> f64 {
    if fixes.is_empty() {
        return 0.0;
    }

    let mut dists: Vec<f64> = fixes
        .iter()
        .map(|&(lat, lon)| distance_km(lat, lon, true_pos.0, true_pos.1))
        .collect();
    dists.sort_by(|a, b| a.partial_cmp(b).unwrap());

    // Median (50th percentile)
    let n = dists.len();
    if n % 2 == 0 {
        (dists[n / 2 - 1] + dists[n / 2]) / 2.0
    } else {
        dists[n / 2]
    }
}

/// Apply systematic and random bearing errors to a nominal bearing.
///
/// - `bearing_deg`: nominal true bearing in degrees.
/// - `systematic_deg`: fixed bias in degrees (added directly).
/// - `random_std_deg`: standard deviation of random error in degrees. Since
///   this module uses only `std`, a simple deterministic model is applied:
///   the random component is modelled as `random_std_deg * sin(bearing * 7.3)`
///   to create a bearing-dependent repeatable error. For Monte-Carlo
///   simulation, use an external RNG.
///
/// Returns the bearing with errors applied, normalized to [0, 360).
pub fn add_bearing_error(bearing_deg: f64, systematic_deg: f64, random_std_deg: f64) -> f64 {
    // Deterministic pseudo-random component based on bearing (no external RNG)
    let pseudo_random = random_std_deg * deg2rad(bearing_deg * 7.3).sin();
    normalize_deg(bearing_deg + systematic_deg + pseudo_random)
}

/// Resolve multiple targets from a set of LOB intersections.
///
/// Given a set of stations, computes all pairwise LOB intersections and
/// clusters them spatially. Returns a `GeoFix` for each cluster (each
/// potential emitter).
///
/// `cluster_radius_km` is the maximum distance between intersections in the
/// same cluster.
pub fn resolve_multi_target(
    stations: &[RdfStation],
    cluster_radius_km: f64,
) -> Vec<GeoFix> {
    if stations.len() < 2 {
        return Vec::new();
    }

    // Collect all pairwise intersections
    let mut points: Vec<(f64, f64)> = Vec::new();
    for i in 0..stations.len() {
        for j in (i + 1)..stations.len() {
            if let Some(pt) = lob_intersection(&stations[i], &stations[j]) {
                points.push(pt);
            }
        }
    }

    if points.is_empty() {
        return Vec::new();
    }

    // Simple greedy clustering
    let mut assigned = vec![false; points.len()];
    let mut clusters: Vec<Vec<(f64, f64)>> = Vec::new();

    for i in 0..points.len() {
        if assigned[i] {
            continue;
        }
        assigned[i] = true;
        let mut cluster = vec![points[i]];

        for j in (i + 1)..points.len() {
            if assigned[j] {
                continue;
            }
            // Check distance to cluster centroid
            let cx: f64 = cluster.iter().map(|p| p.0).sum::<f64>() / cluster.len() as f64;
            let cy: f64 = cluster.iter().map(|p| p.1).sum::<f64>() / cluster.len() as f64;
            if distance_km(cx, cy, points[j].0, points[j].1) <= cluster_radius_km {
                assigned[j] = true;
                cluster.push(points[j]);
            }
        }

        clusters.push(cluster);
    }

    // Build a GeoFix for each cluster
    clusters
        .iter()
        .map(|c| {
            let n = c.len() as f64;
            let lat = c.iter().map(|p| p.0).sum::<f64>() / n;
            let lon = c.iter().map(|p| p.1).sum::<f64>() / n;

            // CEP from scatter within cluster
            let cep = if c.len() >= 2 {
                let mut dists: Vec<f64> = c
                    .iter()
                    .map(|&(la, lo)| distance_km(lat, lon, la, lo))
                    .collect();
                dists.sort_by(|a, b| a.partial_cmp(b).unwrap());
                let idx = (dists.len() as f64 * 0.5).ceil() as usize;
                dists[idx.min(dists.len() - 1)]
            } else {
                0.0
            };

            GeoFix {
                latitude_deg: lat,
                longitude_deg: lon,
                cep_km: cep,
                confidence_pct: (c.len() as f64 / (stations.len() as f64)).min(1.0) * 100.0,
                num_bearings: c.len(),
            }
        })
        .collect()
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: approximate equality for f64.
    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // bearing_from_coords
    // -----------------------------------------------------------------------

    #[test]
    fn test_bearing_due_north() {
        let b = bearing_from_coords(0.0, 0.0, 1.0, 0.0);
        assert!(approx_eq(b, 0.0, 1.0), "bearing due north = {b}");
    }

    #[test]
    fn test_bearing_due_east() {
        let b = bearing_from_coords(0.0, 0.0, 0.0, 1.0);
        assert!(approx_eq(b, 90.0, 1.0), "bearing due east = {b}");
    }

    #[test]
    fn test_bearing_due_south() {
        let b = bearing_from_coords(1.0, 0.0, 0.0, 0.0);
        assert!(approx_eq(b, 180.0, 1.0), "bearing due south = {b}");
    }

    #[test]
    fn test_bearing_due_west() {
        let b = bearing_from_coords(0.0, 0.0, 0.0, -1.0);
        assert!(approx_eq(b, 270.0, 1.0), "bearing due west = {b}");
    }

    // -----------------------------------------------------------------------
    // distance_km
    // -----------------------------------------------------------------------

    #[test]
    fn test_distance_zero() {
        let d = distance_km(40.0, -74.0, 40.0, -74.0);
        assert!(approx_eq(d, 0.0, 0.001), "same point distance = {d}");
    }

    #[test]
    fn test_distance_known() {
        // New York (40.7128, -74.0060) to London (51.5074, -0.1278) ~ 5570 km
        let d = distance_km(40.7128, -74.0060, 51.5074, -0.1278);
        assert!(d > 5500.0 && d < 5650.0, "NYC-London distance = {d} km");
    }

    #[test]
    fn test_distance_short() {
        // ~111 km per degree of latitude at equator
        let d = distance_km(0.0, 0.0, 1.0, 0.0);
        assert!(
            approx_eq(d, 111.19, 1.0),
            "one degree latitude at equator = {d} km"
        );
    }

    // -----------------------------------------------------------------------
    // lob_intersection
    // -----------------------------------------------------------------------

    #[test]
    fn test_lob_intersection_basic() {
        // Two stations looking at roughly the same point
        let s1 = RdfStation {
            latitude_deg: 38.0,
            longitude_deg: -77.0,
            bearing_deg: 45.0,
            bearing_error_deg: 1.0,
        };
        let s2 = RdfStation {
            latitude_deg: 38.0,
            longitude_deg: -76.0,
            bearing_deg: 315.0,
            bearing_error_deg: 1.0,
        };
        let result = lob_intersection(&s1, &s2);
        assert!(result.is_some(), "intersection should exist");
        let (lat, _lon) = result.unwrap();
        // Intersection should be north of both stations
        assert!(lat > 38.0, "intersection lat={lat} should be > 38");
    }

    #[test]
    fn test_lob_intersection_parallel_returns_none() {
        // Two stations with parallel bearings
        let s1 = RdfStation {
            latitude_deg: 38.0,
            longitude_deg: -77.0,
            bearing_deg: 90.0,
            bearing_error_deg: 1.0,
        };
        let s2 = RdfStation {
            latitude_deg: 39.0,
            longitude_deg: -77.0,
            bearing_deg: 90.0,
            bearing_error_deg: 1.0,
        };
        let result = lob_intersection(&s1, &s2);
        assert!(result.is_none(), "parallel bearings should return None");
    }

    // -----------------------------------------------------------------------
    // weighted_bearing_fusion
    // -----------------------------------------------------------------------

    #[test]
    fn test_weighted_fusion_equal_weights() {
        // Two bearings at 10 deg and 350 deg should average to ~0 deg (north)
        let fused = weighted_bearing_fusion(&[10.0, 350.0], &[1.0, 1.0]);
        assert!(
            approx_eq(fused, 0.0, 2.0) || approx_eq(fused, 360.0, 2.0),
            "fused bearing = {fused}, expected ~0/360"
        );
    }

    #[test]
    fn test_weighted_fusion_single_bearing() {
        let fused = weighted_bearing_fusion(&[123.0], &[1.0]);
        assert!(approx_eq(fused, 123.0, 0.01), "single bearing = {fused}");
    }

    #[test]
    fn test_weighted_fusion_unequal_weights() {
        // Heavy weight on 90 deg, light on 180 deg
        let fused = weighted_bearing_fusion(&[90.0, 180.0], &[10.0, 1.0]);
        // Should be closer to 90 than to 180
        assert!(
            fused > 85.0 && fused < 135.0,
            "weighted fusion = {fused}, expected near 90"
        );
    }

    // -----------------------------------------------------------------------
    // triangulate_bearings
    // -----------------------------------------------------------------------

    #[test]
    fn test_triangulate_two_stations() {
        let stations = vec![
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -77.0,
                bearing_deg: 45.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -76.0,
                bearing_deg: 315.0,
                bearing_error_deg: 2.0,
            },
        ];
        let fix = triangulate_bearings(&stations);
        assert!(fix.is_some(), "should produce a fix with 2 stations");
        let f = fix.unwrap();
        assert_eq!(f.num_bearings, 2);
        assert!(f.latitude_deg > 37.0 && f.latitude_deg < 40.0);
    }

    #[test]
    fn test_triangulate_three_stations() {
        let stations = vec![
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -77.5,
                bearing_deg: 60.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 39.0,
                longitude_deg: -76.5,
                bearing_deg: 210.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 37.5,
                longitude_deg: -76.0,
                bearing_deg: 300.0,
                bearing_error_deg: 2.0,
            },
        ];
        let fix = triangulate_bearings(&stations).unwrap();
        assert_eq!(fix.num_bearings, 3);
        assert!(fix.confidence_pct > 0.0);
    }

    #[test]
    fn test_triangulate_insufficient_stations() {
        let stations = vec![RdfStation {
            latitude_deg: 38.0,
            longitude_deg: -77.0,
            bearing_deg: 45.0,
            bearing_error_deg: 2.0,
        }];
        assert!(triangulate_bearings(&stations).is_none());
    }

    #[test]
    fn test_triangulate_empty() {
        assert!(triangulate_bearings(&[]).is_none());
    }

    // -----------------------------------------------------------------------
    // stansfield_estimate
    // -----------------------------------------------------------------------

    #[test]
    fn test_stansfield_two_stations() {
        let stations = vec![
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -77.0,
                bearing_deg: 45.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -76.0,
                bearing_deg: 315.0,
                bearing_error_deg: 2.0,
            },
        ];
        let fix = stansfield_estimate(&stations);
        assert!(fix.is_some());
        let f = fix.unwrap();
        assert_eq!(f.num_bearings, 2);
        // Should agree roughly with triangulate_bearings
        let f2 = triangulate_bearings(&stations).unwrap();
        let diff = distance_km(
            f.latitude_deg,
            f.longitude_deg,
            f2.latitude_deg,
            f2.longitude_deg,
        );
        assert!(
            diff < 50.0,
            "Stansfield vs LS differ by {diff} km, expected < 50"
        );
    }

    #[test]
    fn test_stansfield_insufficient() {
        assert!(stansfield_estimate(&[]).is_none());
        assert!(stansfield_estimate(&[RdfStation {
            latitude_deg: 0.0,
            longitude_deg: 0.0,
            bearing_deg: 0.0,
            bearing_error_deg: 1.0,
        }])
        .is_none());
    }

    // -----------------------------------------------------------------------
    // confidence_ellipse
    // -----------------------------------------------------------------------

    #[test]
    fn test_confidence_ellipse_basic() {
        let stations = vec![
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -77.0,
                bearing_deg: 45.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -76.0,
                bearing_deg: 315.0,
                bearing_error_deg: 2.0,
            },
        ];
        let fix = triangulate_bearings(&stations).unwrap();
        let (semi_major, semi_minor, orientation) = confidence_ellipse(&stations, &fix);
        assert!(semi_major >= semi_minor, "major >= minor");
        assert!(semi_major > 0.0, "semi_major > 0");
        assert!(
            orientation >= 0.0 && orientation < 360.0,
            "orientation in [0,360)"
        );
    }

    #[test]
    fn test_confidence_ellipse_empty() {
        let fix = GeoFix {
            latitude_deg: 38.5,
            longitude_deg: -76.5,
            cep_km: 1.0,
            confidence_pct: 50.0,
            num_bearings: 0,
        };
        let (a, b, _) = confidence_ellipse(&[], &fix);
        assert!(a.is_infinite());
        assert!(b.is_infinite());
    }

    // -----------------------------------------------------------------------
    // circular_error_probable
    // -----------------------------------------------------------------------

    #[test]
    fn test_cep_single_fix() {
        let fixes = vec![(38.5, -76.5)];
        let cep = circular_error_probable(&fixes, (38.5, -76.5));
        assert!(approx_eq(cep, 0.0, 0.01), "CEP at true pos = {cep}");
    }

    #[test]
    fn test_cep_multiple_fixes() {
        let fixes = vec![
            (38.5, -76.5),
            (38.51, -76.51),
            (38.49, -76.49),
            (38.52, -76.52),
        ];
        let cep = circular_error_probable(&fixes, (38.5, -76.5));
        assert!(cep > 0.0, "CEP should be > 0 for scattered fixes");
        assert!(cep < 10.0, "CEP = {cep} km, expected < 10 km");
    }

    #[test]
    fn test_cep_empty() {
        let cep = circular_error_probable(&[], (0.0, 0.0));
        assert!(approx_eq(cep, 0.0, 0.001));
    }

    // -----------------------------------------------------------------------
    // add_bearing_error
    // -----------------------------------------------------------------------

    #[test]
    fn test_add_bearing_error_systematic_only() {
        let result = add_bearing_error(90.0, 5.0, 0.0);
        assert!(approx_eq(result, 95.0, 0.01), "systematic only = {result}");
    }

    #[test]
    fn test_add_bearing_error_wraparound() {
        let result = add_bearing_error(358.0, 5.0, 0.0);
        assert!(approx_eq(result, 3.0, 0.01), "wraparound = {result}");
    }

    #[test]
    fn test_add_bearing_error_with_random() {
        let result = add_bearing_error(45.0, 0.0, 3.0);
        // The pseudo-random component should be deterministic and bounded
        let diff = (result - 45.0).abs();
        assert!(
            diff <= 3.5,
            "random component diff = {diff}, should be <= ~3"
        );
    }

    // -----------------------------------------------------------------------
    // RdfNetwork
    // -----------------------------------------------------------------------

    #[test]
    fn test_rdf_network_observe() {
        let net = RdfNetwork::new(&[(38.0, -77.0), (39.0, -76.0)]);
        assert_eq!(net.num_stations(), 2);

        let obs = net.observe(&[45.0, 225.0], 2.0);
        assert!(obs.is_some());
        let stations = obs.unwrap();
        assert_eq!(stations.len(), 2);
        assert!(approx_eq(stations[0].bearing_deg, 45.0, 0.001));
        assert!(approx_eq(stations[1].bearing_error_deg, 2.0, 0.001));
    }

    #[test]
    fn test_rdf_network_observe_mismatch() {
        let net = RdfNetwork::new(&[(38.0, -77.0), (39.0, -76.0)]);
        // Wrong number of bearings
        let obs = net.observe(&[45.0], 2.0);
        assert!(obs.is_none());
    }

    // -----------------------------------------------------------------------
    // resolve_multi_target
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_target_single_emitter() {
        let stations = vec![
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -77.0,
                bearing_deg: 45.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 38.0,
                longitude_deg: -76.0,
                bearing_deg: 315.0,
                bearing_error_deg: 2.0,
            },
            RdfStation {
                latitude_deg: 39.0,
                longitude_deg: -76.5,
                bearing_deg: 200.0,
                bearing_error_deg: 2.0,
            },
        ];
        let targets = resolve_multi_target(&stations, 50.0);
        // All intersections should cluster into one target
        assert!(
            !targets.is_empty(),
            "should find at least one target cluster"
        );
    }

    #[test]
    fn test_multi_target_insufficient() {
        let targets = resolve_multi_target(&[], 10.0);
        assert!(targets.is_empty());
    }

    // -----------------------------------------------------------------------
    // normalize_deg (internal, but tested for coverage)
    // -----------------------------------------------------------------------

    #[test]
    fn test_normalize_deg() {
        assert!(approx_eq(normalize_deg(0.0), 0.0, 1e-10));
        assert!(approx_eq(normalize_deg(360.0), 0.0, 1e-10));
        assert!(approx_eq(normalize_deg(-90.0), 270.0, 1e-10));
        assert!(approx_eq(normalize_deg(450.0), 90.0, 1e-10));
        assert!(approx_eq(normalize_deg(-360.0), 0.0, 1e-10));
    }
}
