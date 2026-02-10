//! Emitter geolocation from TDOA/range measurements using trilateration and
//! hyperbolic position solving.
//!
//! This module provides two complementary approaches to passive emitter
//! localisation:
//!
//! - **[`TrilaterationSolver`]** — range-based (TOA) positioning from 3+ anchors
//!   in 2-D or 4+ anchors in 3-D via iterative least-squares.
//! - **[`HyperbolicSolver`]** — time-difference-of-arrival (TDOA) positioning
//!   that works with range *differences* rather than absolute ranges.
//!
//! Both solvers support weighted measurements, Newton-Raphson refinement,
//! residual error reporting, and Geometric Dilution of Precision (GDOP)
//! computation.
//!
//! # Quick start
//!
//! ```
//! use r4w_core::trilateration_solver::{Anchor2D, TrilaterationSolver};
//!
//! // Three anchors at known positions with measured ranges.
//! let anchors = vec![
//!     Anchor2D::new(0.0, 0.0),
//!     Anchor2D::new(100.0, 0.0),
//!     Anchor2D::new(50.0, 86.6),
//! ];
//! let ranges = vec![50.0, 50.0, 50.0];
//!
//! let solver = TrilaterationSolver::new(anchors);
//! let result = solver.solve_2d(&ranges).unwrap();
//!
//! // The target is near (50, 28.87).
//! assert!((result.x - 50.0).abs() < 1.0);
//! assert!((result.y - 28.87).abs() < 1.0);
//! ```

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// A 2-D anchor / sensor at a known position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Anchor2D {
    pub x: f64,
    pub y: f64,
}

impl Anchor2D {
    /// Create a new 2-D anchor.
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
}

/// A 3-D anchor / sensor at a known position.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Anchor3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Anchor3D {
    /// Create a new 3-D anchor.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// A 2-D position estimate produced by a solver.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position2D {
    pub x: f64,
    pub y: f64,
    /// RMS residual error (metres) after the final iteration.
    pub residual: f64,
    /// Number of Newton-Raphson iterations used.
    pub iterations: usize,
}

/// A 3-D position estimate produced by a solver.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    /// RMS residual error (metres) after the final iteration.
    pub residual: f64,
    /// Number of Newton-Raphson iterations used.
    pub iterations: usize,
}

/// GDOP components.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GdopResult {
    /// Geometric DOP (overall).
    pub gdop: f64,
    /// Horizontal DOP (2-D: sqrt of x-var + y-var; 3-D: same).
    pub hdop: f64,
    /// Vertical DOP (3-D only; `0.0` for 2-D).
    pub vdop: f64,
    /// Position DOP (3-D sqrt(x-var+y-var+z-var); equals GDOP for 2-D).
    pub pdop: f64,
}

/// Error type for solver failures.
#[derive(Debug, Clone, PartialEq)]
pub enum SolverError {
    /// Too few anchors for the requested dimensionality.
    InsufficientAnchors { need: usize, have: usize },
    /// Range / weight vector length does not match anchor count.
    DimensionMismatch { anchors: usize, measurements: usize },
    /// The iterative solver did not converge within the allowed iterations.
    DidNotConverge { iterations: usize, residual: f64 },
    /// The geometry is degenerate (e.g., all anchors collinear).
    SingularGeometry,
}

impl std::fmt::Display for SolverError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SolverError::InsufficientAnchors { need, have } => {
                write!(f, "need at least {} anchors, got {}", need, have)
            }
            SolverError::DimensionMismatch {
                anchors,
                measurements,
            } => write!(
                f,
                "anchor count ({}) != measurement count ({})",
                anchors, measurements
            ),
            SolverError::DidNotConverge {
                iterations,
                residual,
            } => write!(
                f,
                "did not converge after {} iterations (residual={:.6})",
                iterations, residual
            ),
            SolverError::SingularGeometry => write!(f, "singular / degenerate geometry"),
        }
    }
}

// ---------------------------------------------------------------------------
// TrilaterationSolver (TOA / range-based)
// ---------------------------------------------------------------------------

/// Range-based (TOA) position solver using iterative Newton-Raphson least
/// squares.
///
/// Add anchors at construction time, then call [`solve_2d`](Self::solve_2d) or
/// [`solve_3d`](Self::solve_3d) with a vector of measured ranges.
#[derive(Debug, Clone)]
pub struct TrilaterationSolver {
    anchors_2d: Vec<Anchor2D>,
    anchors_3d: Vec<Anchor3D>,
    /// Maximum Newton-Raphson iterations (default 50).
    pub max_iterations: usize,
    /// Convergence threshold on position delta norm (default 1e-9).
    pub tolerance: f64,
}

impl TrilaterationSolver {
    /// Create a solver with 2-D anchors.
    pub fn new(anchors: Vec<Anchor2D>) -> Self {
        Self {
            anchors_2d: anchors,
            anchors_3d: Vec::new(),
            max_iterations: 50,
            tolerance: 1e-9,
        }
    }

    /// Create a solver with 3-D anchors.
    pub fn new_3d(anchors: Vec<Anchor3D>) -> Self {
        Self {
            anchors_2d: Vec::new(),
            anchors_3d: anchors,
            max_iterations: 50,
            tolerance: 1e-9,
        }
    }

    // -- 2-D -----------------------------------------------------------

    /// Solve for a 2-D position given `ranges.len()` range measurements
    /// (one per anchor). Requires at least 3 anchors.
    pub fn solve_2d(&self, ranges: &[f64]) -> Result<Position2D, SolverError> {
        let n = self.anchors_2d.len();
        if n < 3 {
            return Err(SolverError::InsufficientAnchors { need: 3, have: n });
        }
        if ranges.len() != n {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: ranges.len(),
            });
        }
        let weights: Vec<f64> = vec![1.0; n];
        self.solve_2d_weighted(ranges, &weights)
    }

    /// Weighted 2-D solve. `weights[i]` expresses confidence in anchor `i`.
    pub fn solve_2d_weighted(
        &self,
        ranges: &[f64],
        weights: &[f64],
    ) -> Result<Position2D, SolverError> {
        let n = self.anchors_2d.len();
        if n < 3 {
            return Err(SolverError::InsufficientAnchors { need: 3, have: n });
        }
        if ranges.len() != n || weights.len() != n {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: ranges.len().min(weights.len()),
            });
        }

        // Initial guess: weighted centroid
        let (mut x, mut y) = centroid_2d(&self.anchors_2d);

        for iter in 0..self.max_iterations {
            // Jacobian (n x 2) and residual vector (n x 1)
            let mut jtw_j = [[0.0f64; 2]; 2]; // 2x2
            let mut jtw_r = [0.0f64; 2]; // 2x1
            let mut rms = 0.0f64;

            for i in 0..n {
                let dx = x - self.anchors_2d[i].x;
                let dy = y - self.anchors_2d[i].y;
                let d = (dx * dx + dy * dy).sqrt().max(1e-15);
                let ri = d - ranges[i];
                let jx = dx / d;
                let jy = dy / d;
                let w = weights[i];

                jtw_j[0][0] += w * jx * jx;
                jtw_j[0][1] += w * jx * jy;
                jtw_j[1][0] += w * jy * jx;
                jtw_j[1][1] += w * jy * jy;

                jtw_r[0] += w * jx * ri;
                jtw_r[1] += w * jy * ri;

                rms += ri * ri;
            }
            rms = (rms / n as f64).sqrt();

            // Solve 2x2: delta = (JtWJ)^-1 * JtWr
            let det = jtw_j[0][0] * jtw_j[1][1] - jtw_j[0][1] * jtw_j[1][0];
            if det.abs() < 1e-30 {
                return Err(SolverError::SingularGeometry);
            }
            let inv_det = 1.0 / det;
            let delta_x =
                inv_det * (jtw_j[1][1] * jtw_r[0] - jtw_j[0][1] * jtw_r[1]);
            let delta_y =
                inv_det * (-jtw_j[1][0] * jtw_r[0] + jtw_j[0][0] * jtw_r[1]);

            x -= delta_x;
            y -= delta_y;

            let step = (delta_x * delta_x + delta_y * delta_y).sqrt();
            if step < self.tolerance {
                return Ok(Position2D {
                    x,
                    y,
                    residual: rms,
                    iterations: iter + 1,
                });
            }
        }

        // Compute final residual
        let rms = residual_rms_2d(x, y, &self.anchors_2d, ranges);
        Err(SolverError::DidNotConverge {
            iterations: self.max_iterations,
            residual: rms,
        })
    }

    // -- 3-D -----------------------------------------------------------

    /// Solve for a 3-D position given range measurements. Requires at least 4 anchors.
    pub fn solve_3d(&self, ranges: &[f64]) -> Result<Position3D, SolverError> {
        let n = self.anchors_3d.len();
        if n < 4 {
            return Err(SolverError::InsufficientAnchors { need: 4, have: n });
        }
        if ranges.len() != n {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: ranges.len(),
            });
        }
        let weights = vec![1.0; n];
        self.solve_3d_weighted(ranges, &weights)
    }

    /// Weighted 3-D solve.
    pub fn solve_3d_weighted(
        &self,
        ranges: &[f64],
        weights: &[f64],
    ) -> Result<Position3D, SolverError> {
        let n = self.anchors_3d.len();
        if n < 4 {
            return Err(SolverError::InsufficientAnchors { need: 4, have: n });
        }
        if ranges.len() != n || weights.len() != n {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: ranges.len().min(weights.len()),
            });
        }

        let (mut x, mut y, mut z) = centroid_3d(&self.anchors_3d);

        for iter in 0..self.max_iterations {
            let mut jtw_j = [[0.0f64; 3]; 3];
            let mut jtw_r = [0.0f64; 3];
            let mut rms = 0.0f64;

            for i in 0..n {
                let dx = x - self.anchors_3d[i].x;
                let dy = y - self.anchors_3d[i].y;
                let dz = z - self.anchors_3d[i].z;
                let d = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-15);
                let ri = d - ranges[i];
                let j = [dx / d, dy / d, dz / d];
                let w = weights[i];

                for a in 0..3 {
                    for b in 0..3 {
                        jtw_j[a][b] += w * j[a] * j[b];
                    }
                    jtw_r[a] += w * j[a] * ri;
                }
                rms += ri * ri;
            }
            rms = (rms / n as f64).sqrt();

            // Solve 3x3 via Cramer's rule
            let delta = solve_3x3(jtw_j, jtw_r)?;

            x -= delta[0];
            y -= delta[1];
            z -= delta[2];

            let step =
                (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
            if step < self.tolerance {
                return Ok(Position3D {
                    x,
                    y,
                    z,
                    residual: rms,
                    iterations: iter + 1,
                });
            }
        }

        let rms = residual_rms_3d(x, y, z, &self.anchors_3d, ranges);
        Err(SolverError::DidNotConverge {
            iterations: self.max_iterations,
            residual: rms,
        })
    }

    // -- GDOP ----------------------------------------------------------

    /// Compute GDOP from the 2-D anchor geometry relative to `(px, py)`.
    pub fn gdop_2d(&self, px: f64, py: f64) -> Result<GdopResult, SolverError> {
        let n = self.anchors_2d.len();
        if n < 3 {
            return Err(SolverError::InsufficientAnchors { need: 3, have: n });
        }
        // Build H matrix (n x 2) of unit direction cosines
        let mut hth = [[0.0f64; 2]; 2];
        for a in &self.anchors_2d {
            let dx = px - a.x;
            let dy = py - a.y;
            let d = (dx * dx + dy * dy).sqrt().max(1e-15);
            let ux = dx / d;
            let uy = dy / d;
            hth[0][0] += ux * ux;
            hth[0][1] += ux * uy;
            hth[1][0] += uy * ux;
            hth[1][1] += uy * uy;
        }
        let det = hth[0][0] * hth[1][1] - hth[0][1] * hth[1][0];
        if det.abs() < 1e-30 {
            return Err(SolverError::SingularGeometry);
        }
        let inv_det = 1.0 / det;
        let qxx = hth[1][1] * inv_det;
        let qyy = hth[0][0] * inv_det;

        let hdop = (qxx + qyy).sqrt();
        Ok(GdopResult {
            gdop: hdop,
            hdop,
            vdop: 0.0,
            pdop: hdop,
        })
    }

    /// Compute GDOP from the 3-D anchor geometry relative to `(px, py, pz)`.
    pub fn gdop_3d(
        &self,
        px: f64,
        py: f64,
        pz: f64,
    ) -> Result<GdopResult, SolverError> {
        let n = self.anchors_3d.len();
        if n < 4 {
            return Err(SolverError::InsufficientAnchors { need: 4, have: n });
        }
        let mut hth = [[0.0f64; 3]; 3];
        for a in &self.anchors_3d {
            let dx = px - a.x;
            let dy = py - a.y;
            let dz = pz - a.z;
            let d = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-15);
            let u = [dx / d, dy / d, dz / d];
            for i in 0..3 {
                for j in 0..3 {
                    hth[i][j] += u[i] * u[j];
                }
            }
        }

        let inv = invert_3x3(hth)?;
        let qxx = inv[0][0];
        let qyy = inv[1][1];
        let qzz = inv[2][2];

        let hdop = (qxx + qyy).sqrt();
        let vdop = qzz.sqrt();
        let pdop = (qxx + qyy + qzz).sqrt();
        Ok(GdopResult {
            gdop: pdop,
            hdop,
            vdop,
            pdop,
        })
    }

    /// Compute the residual error vector for a 2-D position.
    pub fn residuals_2d(&self, px: f64, py: f64, ranges: &[f64]) -> Vec<f64> {
        self.anchors_2d
            .iter()
            .zip(ranges.iter())
            .map(|(a, &r)| {
                let d = ((px - a.x).powi(2) + (py - a.y).powi(2)).sqrt();
                d - r
            })
            .collect()
    }

    /// Compute the residual error vector for a 3-D position.
    pub fn residuals_3d(&self, px: f64, py: f64, pz: f64, ranges: &[f64]) -> Vec<f64> {
        self.anchors_3d
            .iter()
            .zip(ranges.iter())
            .map(|(a, &r)| {
                let d =
                    ((px - a.x).powi(2) + (py - a.y).powi(2) + (pz - a.z).powi(2)).sqrt();
                d - r
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// HyperbolicSolver (TDOA / range-difference based)
// ---------------------------------------------------------------------------

/// TDOA-based (hyperbolic) position solver.
///
/// Given `n` anchors and `n-1` range *differences* (all referenced to anchor 0),
/// this solver finds the emitter position at the intersection of hyperbolae.
#[derive(Debug, Clone)]
pub struct HyperbolicSolver {
    anchors_2d: Vec<Anchor2D>,
    anchors_3d: Vec<Anchor3D>,
    /// Maximum Newton-Raphson iterations (default 50).
    pub max_iterations: usize,
    /// Convergence threshold (default 1e-9).
    pub tolerance: f64,
}

impl HyperbolicSolver {
    /// Create a 2-D TDOA solver.
    pub fn new(anchors: Vec<Anchor2D>) -> Self {
        Self {
            anchors_2d: anchors,
            anchors_3d: Vec::new(),
            max_iterations: 50,
            tolerance: 1e-9,
        }
    }

    /// Create a 3-D TDOA solver.
    pub fn new_3d(anchors: Vec<Anchor3D>) -> Self {
        Self {
            anchors_2d: Vec::new(),
            anchors_3d: anchors,
            max_iterations: 50,
            tolerance: 1e-9,
        }
    }

    /// Solve for a 2-D position from `n-1` range differences relative to
    /// anchor 0. `range_diffs[i]` = range(target, anchor[i+1]) - range(target, anchor[0]).
    /// Requires at least 3 anchors (at least 2 differences).
    pub fn solve_2d(&self, range_diffs: &[f64]) -> Result<Position2D, SolverError> {
        let n = self.anchors_2d.len();
        if n < 3 {
            return Err(SolverError::InsufficientAnchors { need: 3, have: n });
        }
        if range_diffs.len() != n - 1 {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: range_diffs.len(),
            });
        }
        let weights = vec![1.0; n - 1];
        self.solve_2d_weighted(range_diffs, &weights)
    }

    /// Weighted 2-D TDOA solve.
    pub fn solve_2d_weighted(
        &self,
        range_diffs: &[f64],
        weights: &[f64],
    ) -> Result<Position2D, SolverError> {
        let n = self.anchors_2d.len();
        let m = n - 1;
        if n < 3 {
            return Err(SolverError::InsufficientAnchors { need: 3, have: n });
        }
        if range_diffs.len() != m || weights.len() != m {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: range_diffs.len().min(weights.len()),
            });
        }

        let (mut x, mut y) = centroid_2d(&self.anchors_2d);

        for iter in 0..self.max_iterations {
            let mut jtw_j = [[0.0f64; 2]; 2];
            let mut jtw_r = [0.0f64; 2];
            let mut rms = 0.0f64;

            let d0 = dist_2d(x, y, &self.anchors_2d[0]);

            for i in 0..m {
                let di = dist_2d(x, y, &self.anchors_2d[i + 1]);
                let ri = (di - d0) - range_diffs[i];

                // Jacobian row: d(di-d0)/dx, d(di-d0)/dy
                let jx = (x - self.anchors_2d[i + 1].x) / di.max(1e-15)
                    - (x - self.anchors_2d[0].x) / d0.max(1e-15);
                let jy = (y - self.anchors_2d[i + 1].y) / di.max(1e-15)
                    - (y - self.anchors_2d[0].y) / d0.max(1e-15);

                let w = weights[i];
                jtw_j[0][0] += w * jx * jx;
                jtw_j[0][1] += w * jx * jy;
                jtw_j[1][0] += w * jy * jx;
                jtw_j[1][1] += w * jy * jy;
                jtw_r[0] += w * jx * ri;
                jtw_r[1] += w * jy * ri;
                rms += ri * ri;
            }
            rms = (rms / m as f64).sqrt();

            let det = jtw_j[0][0] * jtw_j[1][1] - jtw_j[0][1] * jtw_j[1][0];
            if det.abs() < 1e-30 {
                return Err(SolverError::SingularGeometry);
            }
            let inv_det = 1.0 / det;
            let delta_x =
                inv_det * (jtw_j[1][1] * jtw_r[0] - jtw_j[0][1] * jtw_r[1]);
            let delta_y =
                inv_det * (-jtw_j[1][0] * jtw_r[0] + jtw_j[0][0] * jtw_r[1]);

            x -= delta_x;
            y -= delta_y;

            let step = (delta_x * delta_x + delta_y * delta_y).sqrt();
            if step < self.tolerance {
                return Ok(Position2D {
                    x,
                    y,
                    residual: rms,
                    iterations: iter + 1,
                });
            }
        }

        Err(SolverError::DidNotConverge {
            iterations: self.max_iterations,
            residual: 0.0,
        })
    }

    /// Solve for a 3-D position from `n-1` range differences. Requires at least 4 anchors.
    pub fn solve_3d(&self, range_diffs: &[f64]) -> Result<Position3D, SolverError> {
        let n = self.anchors_3d.len();
        if n < 4 {
            return Err(SolverError::InsufficientAnchors { need: 4, have: n });
        }
        if range_diffs.len() != n - 1 {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: range_diffs.len(),
            });
        }
        let weights = vec![1.0; n - 1];
        self.solve_3d_weighted(range_diffs, &weights)
    }

    /// Weighted 3-D TDOA solve.
    pub fn solve_3d_weighted(
        &self,
        range_diffs: &[f64],
        weights: &[f64],
    ) -> Result<Position3D, SolverError> {
        let n = self.anchors_3d.len();
        let m = n - 1;
        if n < 4 {
            return Err(SolverError::InsufficientAnchors { need: 4, have: n });
        }
        if range_diffs.len() != m || weights.len() != m {
            return Err(SolverError::DimensionMismatch {
                anchors: n,
                measurements: range_diffs.len().min(weights.len()),
            });
        }

        let (mut x, mut y, mut z) = centroid_3d(&self.anchors_3d);

        for iter in 0..self.max_iterations {
            let mut jtw_j = [[0.0f64; 3]; 3];
            let mut jtw_r = [0.0f64; 3];
            let mut rms = 0.0f64;

            let d0 = dist_3d(x, y, z, &self.anchors_3d[0]);

            for i in 0..m {
                let di = dist_3d(x, y, z, &self.anchors_3d[i + 1]);
                let ri = (di - d0) - range_diffs[i];

                let jx = (x - self.anchors_3d[i + 1].x) / di.max(1e-15)
                    - (x - self.anchors_3d[0].x) / d0.max(1e-15);
                let jy = (y - self.anchors_3d[i + 1].y) / di.max(1e-15)
                    - (y - self.anchors_3d[0].y) / d0.max(1e-15);
                let jz = (z - self.anchors_3d[i + 1].z) / di.max(1e-15)
                    - (z - self.anchors_3d[0].z) / d0.max(1e-15);

                let w = weights[i];
                let j = [jx, jy, jz];
                for a in 0..3 {
                    for b in 0..3 {
                        jtw_j[a][b] += w * j[a] * j[b];
                    }
                    jtw_r[a] += w * j[a] * ri;
                }
                rms += ri * ri;
            }
            rms = (rms / m as f64).sqrt();

            let delta = solve_3x3(jtw_j, jtw_r)?;

            x -= delta[0];
            y -= delta[1];
            z -= delta[2];

            let step =
                (delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]).sqrt();
            if step < self.tolerance {
                return Ok(Position3D {
                    x,
                    y,
                    z,
                    residual: rms,
                    iterations: iter + 1,
                });
            }
        }

        Err(SolverError::DidNotConverge {
            iterations: self.max_iterations,
            residual: 0.0,
        })
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

fn centroid_2d(anchors: &[Anchor2D]) -> (f64, f64) {
    let n = anchors.len() as f64;
    let sx: f64 = anchors.iter().map(|a| a.x).sum();
    let sy: f64 = anchors.iter().map(|a| a.y).sum();
    (sx / n, sy / n)
}

fn centroid_3d(anchors: &[Anchor3D]) -> (f64, f64, f64) {
    let n = anchors.len() as f64;
    let sx: f64 = anchors.iter().map(|a| a.x).sum();
    let sy: f64 = anchors.iter().map(|a| a.y).sum();
    let sz: f64 = anchors.iter().map(|a| a.z).sum();
    (sx / n, sy / n, sz / n)
}

fn dist_2d(x: f64, y: f64, a: &Anchor2D) -> f64 {
    ((x - a.x).powi(2) + (y - a.y).powi(2)).sqrt()
}

fn dist_3d(x: f64, y: f64, z: f64, a: &Anchor3D) -> f64 {
    ((x - a.x).powi(2) + (y - a.y).powi(2) + (z - a.z).powi(2)).sqrt()
}

fn residual_rms_2d(x: f64, y: f64, anchors: &[Anchor2D], ranges: &[f64]) -> f64 {
    let sum: f64 = anchors
        .iter()
        .zip(ranges.iter())
        .map(|(a, &r)| {
            let d = ((x - a.x).powi(2) + (y - a.y).powi(2)).sqrt();
            (d - r).powi(2)
        })
        .sum();
    (sum / anchors.len() as f64).sqrt()
}

fn residual_rms_3d(x: f64, y: f64, z: f64, anchors: &[Anchor3D], ranges: &[f64]) -> f64 {
    let sum: f64 = anchors
        .iter()
        .zip(ranges.iter())
        .map(|(a, &r)| {
            let d = ((x - a.x).powi(2) + (y - a.y).powi(2) + (z - a.z).powi(2)).sqrt();
            (d - r).powi(2)
        })
        .sum();
    (sum / anchors.len() as f64).sqrt()
}

/// Solve a 3x3 linear system `A * x = b` via Cramer's rule.
fn solve_3x3(a: [[f64; 3]; 3], b: [f64; 3]) -> Result<[f64; 3], SolverError> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-30 {
        return Err(SolverError::SingularGeometry);
    }
    let inv = 1.0 / det;

    let x = inv
        * (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
            + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]));
    let y = inv
        * (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
            - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]));
    let z = inv
        * (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
            - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
            + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]));

    Ok([x, y, z])
}

/// Invert a 3x3 matrix.
fn invert_3x3(a: [[f64; 3]; 3]) -> Result<[[f64; 3]; 3], SolverError> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if det.abs() < 1e-30 {
        return Err(SolverError::SingularGeometry);
    }
    let inv = 1.0 / det;
    let mut out = [[0.0f64; 3]; 3];
    out[0][0] = inv * (a[1][1] * a[2][2] - a[1][2] * a[2][1]);
    out[0][1] = inv * (a[0][2] * a[2][1] - a[0][1] * a[2][2]);
    out[0][2] = inv * (a[0][1] * a[1][2] - a[0][2] * a[1][1]);
    out[1][0] = inv * (a[1][2] * a[2][0] - a[1][0] * a[2][2]);
    out[1][1] = inv * (a[0][0] * a[2][2] - a[0][2] * a[2][0]);
    out[1][2] = inv * (a[0][2] * a[1][0] - a[0][0] * a[1][2]);
    out[2][0] = inv * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    out[2][1] = inv * (a[0][1] * a[2][0] - a[0][0] * a[2][1]);
    out[2][2] = inv * (a[0][0] * a[1][1] - a[0][1] * a[1][0]);
    Ok(out)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: Euclidean distance 2-D
    fn range2(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
        ((ax - bx).powi(2) + (ay - by).powi(2)).sqrt()
    }

    // Helper: Euclidean distance 3-D
    fn range3(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> f64 {
        ((ax - bx).powi(2) + (ay - by).powi(2) + (az - bz).powi(2)).sqrt()
    }

    // ---- TrilaterationSolver 2-D ----------------------------------------

    #[test]
    fn test_trilateration_2d_equilateral() {
        let tx = 50.0;
        let ty = 28.8675;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(100.0, 0.0),
            Anchor2D::new(50.0, 86.6),
        ];
        let ranges: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let solver = TrilaterationSolver::new(anchors);
        let pos = solver.solve_2d(&ranges).unwrap();
        assert!((pos.x - tx).abs() < 0.01, "x: {} vs {}", pos.x, tx);
        assert!((pos.y - ty).abs() < 0.01, "y: {} vs {}", pos.y, ty);
        assert!(pos.residual < 1e-6);
    }

    #[test]
    fn test_trilateration_2d_origin() {
        let anchors = vec![
            Anchor2D::new(10.0, 0.0),
            Anchor2D::new(0.0, 10.0),
            Anchor2D::new(-10.0, 0.0),
            Anchor2D::new(0.0, -10.0),
        ];
        let ranges: Vec<f64> = anchors.iter().map(|a| range2(0.0, 0.0, a.x, a.y)).collect();
        let solver = TrilaterationSolver::new(anchors);
        let pos = solver.solve_2d(&ranges).unwrap();
        assert!((pos.x).abs() < 0.01);
        assert!((pos.y).abs() < 0.01);
    }

    #[test]
    fn test_trilateration_2d_off_center() {
        let tx = 73.5;
        let ty = -22.1;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(200.0, 0.0),
            Anchor2D::new(100.0, 150.0),
        ];
        let ranges: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let solver = TrilaterationSolver::new(anchors);
        let pos = solver.solve_2d(&ranges).unwrap();
        assert!((pos.x - tx).abs() < 0.1, "x: {}", pos.x);
        assert!((pos.y - ty).abs() < 0.1, "y: {}", pos.y);
    }

    #[test]
    fn test_trilateration_2d_insufficient_anchors() {
        let solver = TrilaterationSolver::new(vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(1.0, 0.0),
        ]);
        let err = solver.solve_2d(&[1.0, 1.0]).unwrap_err();
        assert_eq!(err, SolverError::InsufficientAnchors { need: 3, have: 2 });
    }

    #[test]
    fn test_trilateration_2d_dimension_mismatch() {
        let solver = TrilaterationSolver::new(vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(1.0, 0.0),
            Anchor2D::new(0.0, 1.0),
        ]);
        let err = solver.solve_2d(&[1.0, 1.0]).unwrap_err();
        assert!(matches!(err, SolverError::DimensionMismatch { .. }));
    }

    #[test]
    fn test_trilateration_2d_weighted() {
        let tx = 30.0;
        let ty = 40.0;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(100.0, 0.0),
            Anchor2D::new(50.0, 100.0),
        ];
        let ranges: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let weights = vec![1.0, 1.0, 1.0];
        let solver = TrilaterationSolver::new(anchors);
        let pos = solver.solve_2d_weighted(&ranges, &weights).unwrap();
        assert!((pos.x - tx).abs() < 0.1);
        assert!((pos.y - ty).abs() < 0.1);
    }

    // ---- TrilaterationSolver 3-D ----------------------------------------

    #[test]
    fn test_trilateration_3d_basic() {
        let tx = 5.0;
        let ty = 5.0;
        let tz = 5.0;
        let anchors = vec![
            Anchor3D::new(0.0, 0.0, 0.0),
            Anchor3D::new(10.0, 0.0, 0.0),
            Anchor3D::new(0.0, 10.0, 0.0),
            Anchor3D::new(0.0, 0.0, 10.0),
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| range3(tx, ty, tz, a.x, a.y, a.z))
            .collect();
        let solver = TrilaterationSolver::new_3d(anchors);
        let pos = solver.solve_3d(&ranges).unwrap();
        assert!((pos.x - tx).abs() < 0.1, "x: {}", pos.x);
        assert!((pos.y - ty).abs() < 0.1, "y: {}", pos.y);
        assert!((pos.z - tz).abs() < 0.1, "z: {}", pos.z);
    }

    #[test]
    fn test_trilateration_3d_off_center() {
        let tx = 3.0;
        let ty = -7.0;
        let tz = 12.0;
        let anchors = vec![
            Anchor3D::new(0.0, 0.0, 0.0),
            Anchor3D::new(20.0, 0.0, 0.0),
            Anchor3D::new(0.0, 20.0, 0.0),
            Anchor3D::new(0.0, 0.0, 20.0),
            Anchor3D::new(10.0, 10.0, 10.0),
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| range3(tx, ty, tz, a.x, a.y, a.z))
            .collect();
        let solver = TrilaterationSolver::new_3d(anchors);
        let pos = solver.solve_3d(&ranges).unwrap();
        assert!((pos.x - tx).abs() < 0.1);
        assert!((pos.y - ty).abs() < 0.1);
        assert!((pos.z - tz).abs() < 0.1);
    }

    #[test]
    fn test_trilateration_3d_insufficient_anchors() {
        let solver = TrilaterationSolver::new_3d(vec![
            Anchor3D::new(0.0, 0.0, 0.0),
            Anchor3D::new(1.0, 0.0, 0.0),
            Anchor3D::new(0.0, 1.0, 0.0),
        ]);
        let err = solver.solve_3d(&[1.0, 1.0, 1.0]).unwrap_err();
        assert_eq!(err, SolverError::InsufficientAnchors { need: 4, have: 3 });
    }

    #[test]
    fn test_trilateration_3d_weighted() {
        let tx = 5.0;
        let ty = 5.0;
        let tz = 5.0;
        let anchors = vec![
            Anchor3D::new(0.0, 0.0, 0.0),
            Anchor3D::new(10.0, 0.0, 0.0),
            Anchor3D::new(0.0, 10.0, 0.0),
            Anchor3D::new(0.0, 0.0, 10.0),
        ];
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| range3(tx, ty, tz, a.x, a.y, a.z))
            .collect();
        let weights = vec![1.0, 2.0, 1.0, 2.0];
        let solver = TrilaterationSolver::new_3d(anchors);
        let pos = solver.solve_3d_weighted(&ranges, &weights).unwrap();
        assert!((pos.x - tx).abs() < 0.1);
        assert!((pos.y - ty).abs() < 0.1);
        assert!((pos.z - tz).abs() < 0.1);
    }

    // ---- HyperbolicSolver 2-D -------------------------------------------

    #[test]
    fn test_tdoa_2d_basic() {
        let tx = 50.0;
        let ty = 30.0;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(100.0, 0.0),
            Anchor2D::new(50.0, 100.0),
        ];
        let d: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let range_diffs: Vec<f64> = (1..anchors.len()).map(|i| d[i] - d[0]).collect();
        let solver = HyperbolicSolver::new(anchors);
        let pos = solver.solve_2d(&range_diffs).unwrap();
        assert!((pos.x - tx).abs() < 1.0, "x: {}", pos.x);
        assert!((pos.y - ty).abs() < 1.0, "y: {}", pos.y);
    }

    #[test]
    fn test_tdoa_2d_four_anchors() {
        let tx = 25.0;
        let ty = 75.0;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(100.0, 0.0),
            Anchor2D::new(100.0, 100.0),
            Anchor2D::new(0.0, 100.0),
        ];
        let d: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let range_diffs: Vec<f64> = (1..anchors.len()).map(|i| d[i] - d[0]).collect();
        let solver = HyperbolicSolver::new(anchors);
        let pos = solver.solve_2d(&range_diffs).unwrap();
        assert!((pos.x - tx).abs() < 1.0, "x: {}", pos.x);
        assert!((pos.y - ty).abs() < 1.0, "y: {}", pos.y);
    }

    #[test]
    fn test_tdoa_2d_insufficient_anchors() {
        let solver = HyperbolicSolver::new(vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(1.0, 0.0),
        ]);
        let err = solver.solve_2d(&[0.5]).unwrap_err();
        assert_eq!(err, SolverError::InsufficientAnchors { need: 3, have: 2 });
    }

    // ---- HyperbolicSolver 3-D -------------------------------------------

    #[test]
    fn test_tdoa_3d_basic() {
        let tx = 5.0;
        let ty = 5.0;
        let tz = 5.0;
        let anchors = vec![
            Anchor3D::new(0.0, 0.0, 0.0),
            Anchor3D::new(20.0, 0.0, 0.0),
            Anchor3D::new(0.0, 20.0, 0.0),
            Anchor3D::new(0.0, 0.0, 20.0),
        ];
        let d: Vec<f64> = anchors
            .iter()
            .map(|a| range3(tx, ty, tz, a.x, a.y, a.z))
            .collect();
        let range_diffs: Vec<f64> = (1..anchors.len()).map(|i| d[i] - d[0]).collect();
        let solver = HyperbolicSolver::new_3d(anchors);
        let pos = solver.solve_3d(&range_diffs).unwrap();
        assert!((pos.x - tx).abs() < 1.0, "x: {}", pos.x);
        assert!((pos.y - ty).abs() < 1.0, "y: {}", pos.y);
        assert!((pos.z - tz).abs() < 1.0, "z: {}", pos.z);
    }

    // ---- GDOP -----------------------------------------------------------

    #[test]
    fn test_gdop_2d_symmetric() {
        let anchors = vec![
            Anchor2D::new(100.0, 0.0),
            Anchor2D::new(0.0, 100.0),
            Anchor2D::new(-100.0, 0.0),
            Anchor2D::new(0.0, -100.0),
        ];
        let solver = TrilaterationSolver::new(anchors);
        let gdop = solver.gdop_2d(0.0, 0.0).unwrap();
        assert!(gdop.hdop < 2.0, "HDOP should be small: {}", gdop.hdop);
        assert!(gdop.vdop == 0.0); // 2-D
    }

    #[test]
    fn test_gdop_3d() {
        let anchors = vec![
            Anchor3D::new(100.0, 0.0, 0.0),
            Anchor3D::new(0.0, 100.0, 0.0),
            Anchor3D::new(-100.0, 0.0, 0.0),
            Anchor3D::new(0.0, -100.0, 0.0),
            Anchor3D::new(0.0, 0.0, 100.0),
        ];
        let solver = TrilaterationSolver::new_3d(anchors);
        let gdop = solver.gdop_3d(0.0, 0.0, 0.0).unwrap();
        assert!(gdop.pdop > 0.0);
        assert!(gdop.hdop > 0.0);
        assert!(gdop.vdop > 0.0);
    }

    // ---- Residuals ------------------------------------------------------

    #[test]
    fn test_residuals_2d() {
        let tx = 5.0;
        let ty = 2.887;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(10.0, 0.0),
            Anchor2D::new(5.0, 8.66),
        ];
        let ranges: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let solver = TrilaterationSolver::new(anchors);
        let pos = solver.solve_2d(&ranges).unwrap();
        let residuals = solver.residuals_2d(pos.x, pos.y, &ranges);
        let rms: f64 =
            (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64).sqrt();
        assert!(rms < 0.01, "RMS residual: {}", rms);
    }

    #[test]
    fn test_residuals_3d() {
        let anchors = vec![
            Anchor3D::new(0.0, 0.0, 0.0),
            Anchor3D::new(10.0, 0.0, 0.0),
            Anchor3D::new(0.0, 10.0, 0.0),
            Anchor3D::new(0.0, 0.0, 10.0),
        ];
        let tx = 5.0;
        let ty = 5.0;
        let tz = 5.0;
        let ranges: Vec<f64> = anchors
            .iter()
            .map(|a| range3(tx, ty, tz, a.x, a.y, a.z))
            .collect();
        let solver = TrilaterationSolver::new_3d(anchors);
        let pos = solver.solve_3d(&ranges).unwrap();
        let residuals = solver.residuals_3d(pos.x, pos.y, pos.z, &ranges);
        let rms: f64 =
            (residuals.iter().map(|r| r * r).sum::<f64>() / residuals.len() as f64).sqrt();
        assert!(rms < 0.01, "RMS residual: {}", rms);
    }

    // ---- Edge cases / error paths ---------------------------------------

    #[test]
    fn test_solver_error_display() {
        let e = SolverError::InsufficientAnchors { need: 3, have: 2 };
        let s = format!("{}", e);
        assert!(s.contains("3"));
        assert!(s.contains("2"));
    }

    #[test]
    fn test_custom_tolerance_and_iterations() {
        let tx = 5.0;
        let ty = 2.887;
        let anchors = vec![
            Anchor2D::new(0.0, 0.0),
            Anchor2D::new(10.0, 0.0),
            Anchor2D::new(5.0, 8.66),
        ];
        let ranges: Vec<f64> = anchors.iter().map(|a| range2(tx, ty, a.x, a.y)).collect();
        let mut solver = TrilaterationSolver::new(anchors);
        solver.tolerance = 1e-12;
        solver.max_iterations = 200;
        let pos = solver.solve_2d(&ranges).unwrap();
        // With tighter tolerance it should still converge
        assert!(pos.residual < 1e-6);
    }
}
