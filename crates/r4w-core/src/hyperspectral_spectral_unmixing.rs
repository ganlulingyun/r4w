//! Hyperspectral spectral unmixing module.
//!
//! Decomposes hyperspectral imaging pixels into constituent material abundances.
//! Includes endmember extraction (N-FINDR, VCA), non-negative matrix factorization (NMF),
//! fully constrained least squares (FCLS) unmixing, spectral angle mapper (SAM),
//! Euclidean distance classification, abundance sum-to-one constraint, spectral library
//! matching, anomaly detection via RX detector, and dimensionality reduction via PCA.
//!
//! # Example
//!
//! ```
//! use r4w_core::hyperspectral_spectral_unmixing::{
//!     HsiImage, spectral_angle, euclidean_distance,
//! };
//!
//! let a = vec![1.0, 0.0, 0.0];
//! let b = vec![0.0, 1.0, 0.0];
//! let angle = spectral_angle(&a, &b);
//! assert!((angle - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
//!
//! let dist = euclidean_distance(&a, &b);
//! assert!((dist - std::f64::consts::SQRT_2).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for hyperspectral image processing.
#[derive(Debug, Clone)]
pub struct HsiConfig {
    /// Number of spectral bands per pixel.
    pub num_bands: usize,
    /// Number of endmembers to extract.
    pub num_endmembers: usize,
    /// Maximum iterations for iterative algorithms.
    pub max_iterations: usize,
}

impl HsiConfig {
    /// Create a new configuration.
    pub fn new(num_bands: usize, num_endmembers: usize, max_iterations: usize) -> Self {
        Self {
            num_bands,
            num_endmembers,
            max_iterations,
        }
    }
}

// ---------------------------------------------------------------------------
// Image type
// ---------------------------------------------------------------------------

/// A hyperspectral image stored as a collection of spectral pixel vectors.
#[derive(Debug, Clone)]
pub struct HsiImage {
    /// Pixel data -- each inner `Vec<f64>` is one spectral vector of length `num_bands`.
    pub pixels: Vec<Vec<f64>>,
    /// Number of rows in the image.
    pub rows: usize,
    /// Number of columns in the image.
    pub cols: usize,
    /// Number of spectral bands.
    pub num_bands: usize,
}

impl HsiImage {
    /// Create a new hyperspectral image.
    ///
    /// # Panics
    ///
    /// Panics if `rows * cols` does not equal `pixels.len()`, or if any pixel
    /// does not have exactly `num_bands` elements.
    pub fn new(pixels: Vec<Vec<f64>>, rows: usize, cols: usize, num_bands: usize) -> Self {
        assert_eq!(
            rows * cols,
            pixels.len(),
            "rows * cols must equal number of pixels"
        );
        for (i, p) in pixels.iter().enumerate() {
            assert_eq!(
                p.len(),
                num_bands,
                "pixel {} has {} bands, expected {}",
                i,
                p.len(),
                num_bands
            );
        }
        Self {
            pixels,
            rows,
            cols,
            num_bands,
        }
    }

    /// Return the pixel at the given (row, col) position.
    pub fn pixel(&self, row: usize, col: usize) -> &[f64] {
        &self.pixels[row * self.cols + col]
    }

    /// Total number of pixels.
    pub fn num_pixels(&self) -> usize {
        self.pixels.len()
    }
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// Main processor for hyperspectral spectral unmixing operations.
#[derive(Debug, Clone)]
pub struct HsiProcessor {
    /// Processor configuration.
    pub config: HsiConfig,
}

impl HsiProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: HsiConfig) -> Self {
        Self { config }
    }

    /// Extract endmembers using N-FINDR algorithm (delegates to free function).
    pub fn extract_endmembers_nfindr(&self, image: &HsiImage) -> Vec<Vec<f64>> {
        extract_endmembers_nfindr(image, self.config.num_endmembers)
    }

    /// Extract endmembers using VCA algorithm (delegates to free function).
    pub fn extract_endmembers_vca(&self, image: &HsiImage) -> Vec<Vec<f64>> {
        extract_endmembers_vca(image, self.config.num_endmembers)
    }

    /// Unmix a single pixel using FCLS (delegates to free function).
    pub fn unmix_pixel_fcls(&self, pixel: &[f64], endmembers: &[Vec<f64>]) -> Vec<f64> {
        unmix_fcls(pixel, endmembers)
    }

    /// Unmix a single pixel using NNLS (delegates to free function).
    pub fn unmix_pixel_nnls(&self, pixel: &[f64], endmembers: &[Vec<f64>]) -> Vec<f64> {
        unmix_nnls(pixel, endmembers)
    }
}

// ---------------------------------------------------------------------------
// Linear algebra helpers (no external crates)
// ---------------------------------------------------------------------------

/// Compute the dot product of two vectors.
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

/// Compute the L2 norm of a vector.
fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

/// Subtract two vectors: a - b.
fn vec_sub(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b.iter()).map(|(x, y)| x - y).collect()
}

/// Add two vectors: a + b.
#[allow(dead_code)]
fn vec_add(a: &[f64], b: &[f64]) -> Vec<f64> {
    a.iter().zip(b.iter()).map(|(x, y)| x + y).collect()
}

/// Scale a vector by a scalar.
fn vec_scale(a: &[f64], s: f64) -> Vec<f64> {
    a.iter().map(|x| x * s).collect()
}

/// Compute the mean vector of a set of vectors.
fn mean_vector(data: &[Vec<f64>]) -> Vec<f64> {
    let n = data.len() as f64;
    let dim = data[0].len();
    let mut mean = vec![0.0; dim];
    for v in data {
        for (i, val) in v.iter().enumerate() {
            mean[i] += val;
        }
    }
    for m in &mut mean {
        *m /= n;
    }
    mean
}

/// Compute the covariance matrix of data (each row is an observation).
/// Returns a dim x dim matrix stored as Vec<Vec<f64>>.
fn covariance_matrix(data: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let n = data.len() as f64;
    let dim = data[0].len();
    let mean = mean_vector(data);
    let mut cov = vec![vec![0.0; dim]; dim];
    for v in data {
        let centered: Vec<f64> = v.iter().zip(mean.iter()).map(|(a, b)| a - b).collect();
        for i in 0..dim {
            for j in i..dim {
                cov[i][j] += centered[i] * centered[j];
            }
        }
    }
    // Normalize and make symmetric
    for i in 0..dim {
        for j in i..dim {
            cov[i][j] /= n;
            if j != i {
                cov[j][i] = cov[i][j];
            }
        }
    }
    cov
}

/// Matrix-vector multiply: y = A * x where A is stored row-major.
fn mat_vec_mul(a: &[Vec<f64>], x: &[f64]) -> Vec<f64> {
    a.iter().map(|row| dot(row, x)).collect()
}

/// Transpose a matrix.
fn transpose(a: &[Vec<f64>]) -> Vec<Vec<f64>> {
    if a.is_empty() {
        return vec![];
    }
    let rows = a.len();
    let cols = a[0].len();
    let mut t = vec![vec![0.0; rows]; cols];
    for i in 0..rows {
        for j in 0..cols {
            t[j][i] = a[i][j];
        }
    }
    t
}

/// Matrix multiply: C = A * B.
fn mat_mul(a: &[Vec<f64>], b: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let rows_a = a.len();
    let cols_b = b[0].len();
    let cols_a = a[0].len();
    let mut c = vec![vec![0.0; cols_b]; rows_a];
    for i in 0..rows_a {
        for k in 0..cols_a {
            let a_ik = a[i][k];
            for j in 0..cols_b {
                c[i][j] += a_ik * b[k][j];
            }
        }
    }
    c
}

/// Invert a small square matrix using Gauss-Jordan elimination.
/// Returns `None` if singular.
fn mat_inverse(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let n = a.len();
    // Augmented matrix [A | I]
    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = a[i].clone();
        row.resize(2 * n, 0.0);
        row[n + i] = 1.0;
        aug.push(row);
    }

    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            let v = aug[row][col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-15 {
            return None; // singular
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        for j in 0..(2 * n) {
            aug[col][j] /= pivot;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Extract inverse
    let inv: Vec<Vec<f64>> = aug
        .into_iter()
        .map(|row| row[n..].to_vec())
        .collect();
    Some(inv)
}

/// Power iteration to find the dominant eigenvector of a symmetric matrix.
/// Returns (eigenvalue, eigenvector).
fn power_iteration(matrix: &[Vec<f64>], iterations: usize) -> (f64, Vec<f64>) {
    let n = matrix.len();
    let mut v = vec![0.0; n];
    // Initial guess -- alternate signs for better convergence
    for i in 0..n {
        v[i] = if i % 2 == 0 { 1.0 } else { -1.0 };
    }
    let n_v = norm(&v);
    if n_v > 0.0 {
        for x in &mut v {
            *x /= n_v;
        }
    }

    let mut eigenvalue = 0.0;
    for _ in 0..iterations {
        let mv = mat_vec_mul(matrix, &v);
        eigenvalue = dot(&v, &mv);
        let n_mv = norm(&mv);
        if n_mv < 1e-15 {
            break;
        }
        v = vec_scale(&mv, 1.0 / n_mv);
    }
    (eigenvalue, v)
}

/// Deflate a symmetric matrix by removing the component corresponding to an eigenvector.
fn deflate_matrix(matrix: &mut [Vec<f64>], eigenvalue: f64, eigenvector: &[f64]) {
    let n = matrix.len();
    for i in 0..n {
        for j in 0..n {
            matrix[i][j] -= eigenvalue * eigenvector[i] * eigenvector[j];
        }
    }
}

/// Compute the volume of the simplex formed by a set of points.
/// Uses the determinant of the matrix formed by augmented coordinates.
fn simplex_volume(points: &[Vec<f64>]) -> f64 {
    let k = points.len(); // number of vertices
    if k <= 1 {
        return 0.0;
    }
    // Build matrix with rows = (point_i - point_0) for i = 1..k-1
    let dim = points[0].len().min(k - 1);
    let mut mat = vec![vec![0.0; dim]; k - 1];
    for i in 1..k {
        for j in 0..dim {
            mat[i - 1][j] = points[i][j.min(points[i].len() - 1)]
                - points[0][j.min(points[0].len() - 1)];
        }
    }
    // For a square matrix, compute determinant
    if mat.len() == mat[0].len() {
        det(&mat).abs()
    } else {
        // Non-square: use det(M * M^T)^0.5
        let mt = transpose(&mat);
        let mmt = mat_mul(&mat, &mt);
        det(&mmt).abs().sqrt()
    }
}

/// Compute determinant of a square matrix via LU-style elimination.
fn det(matrix: &[Vec<f64>]) -> f64 {
    let n = matrix.len();
    if n == 0 {
        return 1.0;
    }
    let mut m: Vec<Vec<f64>> = matrix.to_vec();
    let mut sign = 1.0_f64;

    for col in 0..n {
        // Partial pivot
        let mut max_row = col;
        let mut max_val = m[col][col].abs();
        for row in (col + 1)..n {
            let v = m[row][col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-15 {
            return 0.0;
        }
        if max_row != col {
            m.swap(col, max_row);
            sign = -sign;
        }
        let pivot = m[col][col];
        for row in (col + 1)..n {
            let factor = m[row][col] / pivot;
            for j in col..n {
                m[row][j] -= factor * m[col][j];
            }
        }
    }
    let mut d = sign;
    for i in 0..n {
        d *= m[i][i];
    }
    d
}

/// Simple pseudo-random number generator (LCG).
/// Returns values in [0, 1).
struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9e3779b97f4a7c15),
        }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self
            .state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Random index in [0, max).
    #[allow(dead_code)]
    fn next_usize(&mut self, max: usize) -> usize {
        (self.next_f64() * max as f64) as usize % max
    }
}

// ---------------------------------------------------------------------------
// Public API -- spectral metrics
// ---------------------------------------------------------------------------

/// Compute the Spectral Angle Mapper (SAM) between two spectral vectors.
///
/// Returns the angle in radians in the range [0, pi].
pub fn spectral_angle(a: &[f64], b: &[f64]) -> f64 {
    let na = norm(a);
    let nb = norm(b);
    if na < 1e-15 || nb < 1e-15 {
        return PI / 2.0; // undefined -- treat as orthogonal
    }
    let cos_theta = (dot(a, b) / (na * nb)).clamp(-1.0, 1.0);
    cos_theta.acos()
}

/// Compute the Euclidean distance between two spectral vectors.
pub fn euclidean_distance(a: &[f64], b: &[f64]) -> f64 {
    let d = vec_sub(a, b);
    norm(&d)
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

/// Classify a pixel against a spectral library using Spectral Angle Mapper.
///
/// Returns `(best_index, best_angle)` where `best_index` is the index of
/// the closest library spectrum and `best_angle` is the angle in radians.
pub fn classify_sam(pixel: &[f64], library: &[Vec<f64>]) -> (usize, f64) {
    assert!(!library.is_empty(), "library must not be empty");
    let mut best_idx = 0;
    let mut best_angle = f64::MAX;
    for (i, spec) in library.iter().enumerate() {
        let angle = spectral_angle(pixel, spec);
        if angle < best_angle {
            best_angle = angle;
            best_idx = i;
        }
    }
    (best_idx, best_angle)
}

// ---------------------------------------------------------------------------
// Endmember extraction -- N-FINDR
// ---------------------------------------------------------------------------

/// Extract endmembers from a hyperspectral image using the N-FINDR algorithm.
///
/// The N-FINDR algorithm iteratively searches for the set of pixels that maximize
/// the volume of the simplex they define in spectral space.
///
/// Returns `num_endmembers` spectral vectors.
pub fn extract_endmembers_nfindr(image: &HsiImage, num_endmembers: usize) -> Vec<Vec<f64>> {
    assert!(num_endmembers >= 2, "need at least 2 endmembers");
    assert!(
        image.num_pixels() >= num_endmembers,
        "not enough pixels for the requested number of endmembers"
    );

    // Reduce dimensionality to (num_endmembers - 1) via PCA for volume computation
    let reduced = pca_reduce(&image.pixels, num_endmembers.saturating_sub(1).max(1));

    // Initialize with evenly spaced pixels
    let step = image.num_pixels() / num_endmembers;
    let mut indices: Vec<usize> = (0..num_endmembers)
        .map(|i| (i * step).min(image.num_pixels() - 1))
        .collect();

    let mut endmembers_reduced: Vec<Vec<f64>> =
        indices.iter().map(|&i| reduced[i].clone()).collect();
    let mut volume = simplex_volume(&endmembers_reduced);

    let max_iter = image.num_pixels() * 3;
    let mut changed = true;
    let mut iter_count = 0;

    while changed && iter_count < max_iter {
        changed = false;
        iter_count += 1;
        for e in 0..num_endmembers {
            for p in 0..image.num_pixels() {
                if indices.contains(&p) {
                    continue;
                }
                let old = endmembers_reduced[e].clone();
                endmembers_reduced[e] = reduced[p].clone();
                let new_volume = simplex_volume(&endmembers_reduced);
                if new_volume > volume * (1.0 + 1e-10) {
                    volume = new_volume;
                    indices[e] = p;
                    changed = true;
                } else {
                    endmembers_reduced[e] = old;
                }
            }
        }
    }

    indices.iter().map(|&i| image.pixels[i].clone()).collect()
}

// ---------------------------------------------------------------------------
// Endmember extraction -- Vertex Component Analysis (VCA)
// ---------------------------------------------------------------------------

/// Extract endmembers from a hyperspectral image using Vertex Component Analysis.
///
/// VCA projects data onto directions orthogonal to the subspace spanned by
/// already-selected endmembers and selects the pixel with maximum projection.
///
/// Returns `num_endmembers` spectral vectors.
pub fn extract_endmembers_vca(image: &HsiImage, num_endmembers: usize) -> Vec<Vec<f64>> {
    assert!(num_endmembers >= 1, "need at least 1 endmember");
    assert!(
        image.num_pixels() >= num_endmembers,
        "not enough pixels for the requested number of endmembers"
    );

    // Project data to num_endmembers dimensions via PCA
    let reduced = pca_reduce(&image.pixels, num_endmembers);

    let mut rng = SimpleRng::new(42);
    let mut selected_indices: Vec<usize> = Vec::with_capacity(num_endmembers);
    let mut endmembers_reduced: Vec<Vec<f64>> = Vec::with_capacity(num_endmembers);

    for iter in 0..num_endmembers {
        if iter == 0 {
            // First endmember: random direction projection, pick the extreme pixel
            let dim = reduced[0].len();
            let mut direction: Vec<f64> = (0..dim).map(|_| rng.next_f64() - 0.5).collect();
            let n_d = norm(&direction);
            if n_d > 1e-15 {
                direction = vec_scale(&direction, 1.0 / n_d);
            }
            let mut best_idx = 0;
            let mut best_proj = f64::NEG_INFINITY;
            for (i, px) in reduced.iter().enumerate() {
                let proj = dot(px, &direction).abs();
                if proj > best_proj {
                    best_proj = proj;
                    best_idx = i;
                }
            }
            selected_indices.push(best_idx);
            endmembers_reduced.push(reduced[best_idx].clone());
        } else {
            // Project each pixel onto the subspace orthogonal to already-chosen endmembers
            // using Gram-Schmidt residuals
            let mut best_idx = 0;
            let mut best_residual_norm = f64::NEG_INFINITY;
            for (i, px) in reduced.iter().enumerate() {
                if selected_indices.contains(&i) {
                    continue;
                }
                let mut residual = px.clone();
                for em in &endmembers_reduced {
                    let n_em = dot(em, em);
                    if n_em > 1e-15 {
                        let proj = dot(&residual, em) / n_em;
                        let scaled = vec_scale(em, proj);
                        residual = vec_sub(&residual, &scaled);
                    }
                }
                let rn = norm(&residual);
                if rn > best_residual_norm {
                    best_residual_norm = rn;
                    best_idx = i;
                }
            }
            selected_indices.push(best_idx);
            endmembers_reduced.push(reduced[best_idx].clone());
        }
    }

    selected_indices
        .iter()
        .map(|&i| image.pixels[i].clone())
        .collect()
}

// ---------------------------------------------------------------------------
// Unmixing -- Fully Constrained Least Squares (FCLS)
// ---------------------------------------------------------------------------

/// Unmix a pixel into endmember abundances using Fully Constrained Least Squares.
///
/// Constraints:
/// - Abundances are non-negative (abundance >= 0).
/// - Abundances sum to one (sum-to-one constraint).
///
/// Uses an iterative active-set approach.
pub fn unmix_fcls(pixel: &[f64], endmembers: &[Vec<f64>]) -> Vec<f64> {
    let num_em = endmembers.len();
    assert!(num_em > 0, "need at least one endmember");

    // First solve unconstrained via least squares, then project
    let mut abundances = solve_least_squares(pixel, endmembers);

    // Iterative projection to satisfy non-negativity and sum-to-one
    for _ in 0..200 {
        // Clamp negatives to zero
        for a in abundances.iter_mut() {
            if *a < 0.0 {
                *a = 0.0;
            }
        }
        // Normalize to sum to one
        let sum: f64 = abundances.iter().sum();
        if sum > 1e-15 {
            for a in abundances.iter_mut() {
                *a /= sum;
            }
        } else {
            // Fallback: uniform
            let uniform = 1.0 / num_em as f64;
            for a in abundances.iter_mut() {
                *a = uniform;
            }
        }
        // Check if unconstrained solution satisfies constraints
        let recon = reconstruct(endmembers, &abundances);
        let residual = vec_sub(pixel, &recon);
        let res_norm = norm(&residual);
        if res_norm < 1e-10 {
            break;
        }

        // Re-solve with current active set
        let active: Vec<usize> = abundances
            .iter()
            .enumerate()
            .filter(|(_, &a)| a > 1e-12)
            .map(|(i, _)| i)
            .collect();
        if active.is_empty() {
            break;
        }
        let active_em: Vec<Vec<f64>> = active.iter().map(|&i| endmembers[i].clone()).collect();
        let active_ab = solve_least_squares(pixel, &active_em);
        let mut new_ab = vec![0.0; num_em];
        for (ai, &idx) in active.iter().enumerate() {
            new_ab[idx] = active_ab[ai];
        }
        abundances = new_ab;
    }

    // Final projection
    for a in abundances.iter_mut() {
        if *a < 0.0 {
            *a = 0.0;
        }
    }
    let sum: f64 = abundances.iter().sum();
    if sum > 1e-15 {
        for a in abundances.iter_mut() {
            *a /= sum;
        }
    }

    abundances
}

/// Reconstruct a pixel from endmembers and abundances.
fn reconstruct(endmembers: &[Vec<f64>], abundances: &[f64]) -> Vec<f64> {
    let bands = endmembers[0].len();
    let mut result = vec![0.0; bands];
    for (em, &ab) in endmembers.iter().zip(abundances.iter()) {
        for (r, &e) in result.iter_mut().zip(em.iter()) {
            *r += ab * e;
        }
    }
    result
}

/// Solve the unconstrained least squares problem: min || pixel - E * a ||^2.
/// E is num_bands x num_endmembers, pixel is num_bands x 1.
/// Normal equations: (E^T E) a = E^T pixel.
fn solve_least_squares(pixel: &[f64], endmembers: &[Vec<f64>]) -> Vec<f64> {
    let num_em = endmembers.len();
    // E^T E
    let mut ete = vec![vec![0.0; num_em]; num_em];
    for i in 0..num_em {
        for j in i..num_em {
            let val = dot(&endmembers[i], &endmembers[j]);
            ete[i][j] = val;
            ete[j][i] = val;
        }
    }
    // E^T pixel
    let etp: Vec<f64> = endmembers.iter().map(|em| dot(em, pixel)).collect();

    // Solve via inverse (add small regularization for stability)
    for i in 0..num_em {
        ete[i][i] += 1e-10;
    }
    match mat_inverse(&ete) {
        Some(inv) => mat_vec_mul(&inv, &etp),
        None => vec![1.0 / num_em as f64; num_em],
    }
}

// ---------------------------------------------------------------------------
// Unmixing -- Non-Negative Least Squares (NNLS)
// ---------------------------------------------------------------------------

/// Unmix a pixel into endmember abundances using non-negative least squares.
///
/// Uses a simplified active-set NNLS approach (Lawson-Hanson style).
/// Abundances are non-negative but do NOT necessarily sum to one.
pub fn unmix_nnls(pixel: &[f64], endmembers: &[Vec<f64>]) -> Vec<f64> {
    let num_em = endmembers.len();
    assert!(num_em > 0, "need at least one endmember");

    let mut abundances = vec![0.0; num_em];
    let mut passive_set: Vec<bool> = vec![false; num_em];

    for _ in 0..500 {
        // Compute gradient: E^T (pixel - E * abundances)
        let recon = reconstruct(endmembers, &abundances);
        let residual = vec_sub(pixel, &recon);
        let gradient: Vec<f64> = endmembers.iter().map(|em| dot(em, &residual)).collect();

        // Find the most violating variable in the zero set
        let mut max_grad = 0.0;
        let mut max_idx = None;
        for i in 0..num_em {
            if !passive_set[i] && gradient[i] > max_grad {
                max_grad = gradient[i];
                max_idx = Some(i);
            }
        }
        if max_idx.is_none() || max_grad < 1e-12 {
            break;
        }
        let t = max_idx.unwrap();
        passive_set[t] = true;

        // Inner loop: solve LS on passive set, fix negatives
        loop {
            let active: Vec<usize> = (0..num_em).filter(|&i| passive_set[i]).collect();
            if active.is_empty() {
                break;
            }
            let active_em: Vec<Vec<f64>> =
                active.iter().map(|&i| endmembers[i].clone()).collect();
            let z = solve_least_squares(pixel, &active_em);

            if z.iter().all(|&v| v >= 0.0) {
                for (ai, &idx) in active.iter().enumerate() {
                    abundances[idx] = z[ai];
                }
                break;
            }

            // Find the most constraining negative and interpolate
            let mut alpha = f64::MAX;
            let mut alpha_idx = 0;
            for (ai, &idx) in active.iter().enumerate() {
                if z[ai] <= 0.0 {
                    let denom = abundances[idx] - z[ai];
                    if denom.abs() > 1e-15 {
                        let ratio = abundances[idx] / denom;
                        if ratio < alpha {
                            alpha = ratio;
                            alpha_idx = idx;
                        }
                    }
                }
            }
            // Interpolate
            for (ai, &idx) in active.iter().enumerate() {
                abundances[idx] += alpha * (z[ai] - abundances[idx]);
            }
            // Remove the constraining variable from passive set
            passive_set[alpha_idx] = false;
            abundances[alpha_idx] = 0.0;
        }
    }

    abundances
}

// ---------------------------------------------------------------------------
// Non-Negative Matrix Factorization (NMF)
// ---------------------------------------------------------------------------

/// Decompose a data matrix into non-negative factors W and H such that
/// data approximately equals W * H, using multiplicative update rules.
///
/// - `data`: N x M matrix (N observations of M features), stored row-major.
/// - `rank`: number of components (inner dimension).
/// - `iterations`: number of update iterations.
///
/// Returns `(W, H)` where W is N x rank and H is rank x M.
pub fn nmf_decompose(
    data: &[Vec<f64>],
    rank: usize,
    iterations: usize,
) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
    let n = data.len();
    assert!(n > 0, "data must not be empty");
    let m = data[0].len();
    assert!(rank > 0 && rank <= n.min(m), "invalid rank");

    let mut rng = SimpleRng::new(12345);

    // Initialize W and H with small positive random values
    let mut w: Vec<Vec<f64>> = (0..n)
        .map(|_| (0..rank).map(|_| rng.next_f64() * 0.1 + 0.01).collect())
        .collect();
    let mut h: Vec<Vec<f64>> = (0..rank)
        .map(|_| (0..m).map(|_| rng.next_f64() * 0.1 + 0.01).collect())
        .collect();

    let eps = 1e-15;

    for _ in 0..iterations {
        // Update H: H <- H .* (W^T * V) ./ (W^T * W * H)
        let wt = transpose(&w);
        let wt_v = mat_mul(&wt, data);
        let wt_w = mat_mul(&wt, &w);
        let wt_w_h = mat_mul(&wt_w, &h);
        for i in 0..rank {
            for j in 0..m {
                h[i][j] *= wt_v[i][j] / (wt_w_h[i][j] + eps);
            }
        }

        // Update W: W <- W .* (V * H^T) ./ (W * H * H^T)
        let ht = transpose(&h);
        let v_ht = mat_mul(data, &ht);
        let w_h = mat_mul(&w, &h);
        let w_h_ht = mat_mul(&w_h, &ht);
        for i in 0..n {
            for j in 0..rank {
                w[i][j] *= v_ht[i][j] / (w_h_ht[i][j] + eps);
            }
        }
    }

    (w, h)
}

// ---------------------------------------------------------------------------
// RX Anomaly Detection
// ---------------------------------------------------------------------------

/// Compute the Reed-Xiaoli (RX) anomaly detection score for each pixel.
///
/// The RX detector models the background as a multivariate Gaussian and
/// computes the Mahalanobis distance for each pixel. Higher scores indicate
/// more anomalous pixels.
///
/// Returns a vector of anomaly scores, one per pixel.
pub fn rx_anomaly_detect(image: &HsiImage) -> Vec<f64> {
    let mean = mean_vector(&image.pixels);
    let cov = covariance_matrix(&image.pixels);

    // Add regularization to covariance matrix
    let mut reg_cov = cov;
    for i in 0..reg_cov.len() {
        reg_cov[i][i] += 1e-8;
    }

    let inv_cov = mat_inverse(&reg_cov).unwrap_or_else(|| {
        // Fallback: use diagonal inverse
        let n = reg_cov.len();
        let mut diag = vec![vec![0.0; n]; n];
        for i in 0..n {
            diag[i][i] = 1.0 / reg_cov[i][i].max(1e-15);
        }
        diag
    });

    image
        .pixels
        .iter()
        .map(|px| {
            let centered = vec_sub(px, &mean);
            let tmp = mat_vec_mul(&inv_cov, &centered);
            dot(&centered, &tmp)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// PCA Dimensionality Reduction
// ---------------------------------------------------------------------------

/// Reduce dimensionality of spectral data using Principal Component Analysis.
///
/// - `data`: N observations, each of dimension D (stored as `Vec<Vec<f64>>`).
/// - `components`: number of principal components to keep.
///
/// Returns the projected data: N observations of dimension `components`.
pub fn pca_reduce(data: &[Vec<f64>], components: usize) -> Vec<Vec<f64>> {
    assert!(!data.is_empty(), "data must not be empty");
    let dim = data[0].len();
    let components = components.min(dim);
    if components == 0 {
        return data.iter().map(|_| vec![]).collect();
    }

    let mean = mean_vector(data);
    let centered: Vec<Vec<f64>> = data.iter().map(|v| vec_sub(v, &mean)).collect();

    let mut cov = covariance_matrix(&centered);

    // Extract top eigenvectors via power iteration + deflation
    let mut eigenvectors: Vec<Vec<f64>> = Vec::with_capacity(components);
    for _ in 0..components {
        let (eigenvalue, eigenvector) = power_iteration(&cov, 300);
        deflate_matrix(&mut cov, eigenvalue, &eigenvector);
        eigenvectors.push(eigenvector);
    }

    // Project data onto eigenvectors
    centered
        .iter()
        .map(|v| eigenvectors.iter().map(|ev| dot(v, ev)).collect())
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a simple 3-band test image.
    fn make_test_image() -> HsiImage {
        // 4x4 image, 3 bands
        // Endmembers: pure red [1,0,0], pure green [0,1,0], pure blue [0,0,1]
        let pixels = vec![
            vec![1.0, 0.0, 0.0],  // pure red
            vec![0.0, 1.0, 0.0],  // pure green
            vec![0.0, 0.0, 1.0],  // pure blue
            vec![0.5, 0.5, 0.0],  // red+green mix
            vec![0.5, 0.0, 0.5],  // red+blue mix
            vec![0.0, 0.5, 0.5],  // green+blue mix
            vec![0.33, 0.33, 0.34], // roughly equal mix
            vec![0.8, 0.1, 0.1],  // mostly red
            vec![0.1, 0.8, 0.1],  // mostly green
            vec![0.1, 0.1, 0.8],  // mostly blue
            vec![0.6, 0.3, 0.1],  // red dominant mix
            vec![0.2, 0.6, 0.2],  // green dominant mix
            vec![0.1, 0.3, 0.6],  // blue dominant mix
            vec![0.4, 0.4, 0.2],  // red+green dominant
            vec![0.3, 0.3, 0.4],  // blue-ish mix
            vec![0.9, 0.05, 0.05], // almost pure red
        ];
        HsiImage::new(pixels, 4, 4, 3)
    }

    #[test]
    fn test_hsi_config_creation() {
        let config = HsiConfig::new(100, 5, 50);
        assert_eq!(config.num_bands, 100);
        assert_eq!(config.num_endmembers, 5);
        assert_eq!(config.max_iterations, 50);
    }

    #[test]
    fn test_hsi_image_creation() {
        let image = make_test_image();
        assert_eq!(image.rows, 4);
        assert_eq!(image.cols, 4);
        assert_eq!(image.num_bands, 3);
        assert_eq!(image.num_pixels(), 16);
    }

    #[test]
    fn test_hsi_image_pixel_access() {
        let image = make_test_image();
        let p00 = image.pixel(0, 0);
        assert_eq!(p00, &[1.0, 0.0, 0.0]);
        let p01 = image.pixel(0, 1);
        assert_eq!(p01, &[0.0, 1.0, 0.0]);
    }

    #[test]
    fn test_spectral_angle_identical() {
        let a = vec![1.0, 2.0, 3.0];
        let angle = spectral_angle(&a, &a);
        assert!(angle.abs() < 1e-10, "angle of identical vectors should be 0");
    }

    #[test]
    fn test_spectral_angle_orthogonal() {
        let a = vec![1.0, 0.0, 0.0];
        let b = vec![0.0, 1.0, 0.0];
        let angle = spectral_angle(&a, &b);
        assert!(
            (angle - std::f64::consts::FRAC_PI_2).abs() < 1e-10,
            "orthogonal vectors should have angle pi/2, got {}",
            angle
        );
    }

    #[test]
    fn test_spectral_angle_opposite() {
        let a = vec![1.0, 0.0, 0.0];
        let b = vec![-1.0, 0.0, 0.0];
        let angle = spectral_angle(&a, &b);
        assert!(
            (angle - PI).abs() < 1e-10,
            "opposite vectors should have angle pi, got {}",
            angle
        );
    }

    #[test]
    fn test_euclidean_distance_same() {
        let a = vec![3.0, 4.0, 5.0];
        let d = euclidean_distance(&a, &a);
        assert!(d.abs() < 1e-15, "distance to self should be 0");
    }

    #[test]
    fn test_euclidean_distance_known() {
        let a = vec![0.0, 0.0];
        let b = vec![3.0, 4.0];
        let d = euclidean_distance(&a, &b);
        assert!(
            (d - 5.0).abs() < 1e-10,
            "3-4-5 triangle distance should be 5, got {}",
            d
        );
    }

    #[test]
    fn test_classify_sam_exact_match() {
        let library = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let pixel = vec![0.0, 0.0, 1.0];
        let (idx, angle) = classify_sam(&pixel, &library);
        assert_eq!(idx, 2);
        assert!(angle.abs() < 1e-10);
    }

    #[test]
    fn test_classify_sam_closest() {
        let library = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        // Pixel mostly aligned with red
        let pixel = vec![0.9, 0.1, 0.0];
        let (idx, _angle) = classify_sam(&pixel, &library);
        assert_eq!(idx, 0, "should classify as closest to red");
    }

    #[test]
    fn test_unmix_fcls_pure_endmember() {
        let endmembers = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let pixel = vec![1.0, 0.0, 0.0]; // pure first endmember
        let abundances = unmix_fcls(&pixel, &endmembers);
        assert_eq!(abundances.len(), 3);
        assert!(
            abundances[0] > 0.9,
            "first abundance should be ~1.0, got {}",
            abundances[0]
        );
        assert!(
            abundances[1] < 0.1,
            "second abundance should be ~0.0, got {}",
            abundances[1]
        );
        assert!(
            abundances[2] < 0.1,
            "third abundance should be ~0.0, got {}",
            abundances[2]
        );
    }

    #[test]
    fn test_unmix_fcls_sum_to_one() {
        let endmembers = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let pixel = vec![0.3, 0.5, 0.2];
        let abundances = unmix_fcls(&pixel, &endmembers);
        let sum: f64 = abundances.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-6,
            "FCLS abundances should sum to 1, got {}",
            sum
        );
    }

    #[test]
    fn test_unmix_fcls_non_negative() {
        let endmembers = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let pixel = vec![0.5, 0.3, 0.2];
        let abundances = unmix_fcls(&pixel, &endmembers);
        for (i, &a) in abundances.iter().enumerate() {
            assert!(a >= -1e-10, "abundance {} should be non-negative, got {}", i, a);
        }
    }

    #[test]
    fn test_unmix_nnls_pure_endmember() {
        let endmembers = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        let pixel = vec![0.0, 1.0, 0.0]; // pure second endmember
        let abundances = unmix_nnls(&pixel, &endmembers);
        assert_eq!(abundances.len(), 3);
        assert!(
            abundances[1] > 0.9,
            "second abundance should be ~1.0, got {}",
            abundances[1]
        );
    }

    #[test]
    fn test_unmix_nnls_non_negative() {
        let endmembers = vec![
            vec![1.0, 0.2, 0.0],
            vec![0.0, 1.0, 0.3],
            vec![0.2, 0.0, 1.0],
        ];
        let pixel = vec![0.5, 0.5, 0.5];
        let abundances = unmix_nnls(&pixel, &endmembers);
        for (i, &a) in abundances.iter().enumerate() {
            assert!(
                a >= -1e-10,
                "NNLS abundance {} should be non-negative, got {}",
                i,
                a
            );
        }
    }

    #[test]
    fn test_nmf_decompose_shapes() {
        let data = vec![
            vec![1.0, 0.5, 0.0],
            vec![0.5, 1.0, 0.5],
            vec![0.0, 0.5, 1.0],
            vec![0.8, 0.2, 0.1],
        ];
        let (w, h) = nmf_decompose(&data, 2, 50);
        assert_eq!(w.len(), 4, "W should have 4 rows");
        assert_eq!(w[0].len(), 2, "W should have rank columns");
        assert_eq!(h.len(), 2, "H should have rank rows");
        assert_eq!(h[0].len(), 3, "H should have 3 columns");
    }

    #[test]
    fn test_nmf_decompose_non_negative() {
        let data = vec![
            vec![1.0, 0.5, 0.0],
            vec![0.5, 1.0, 0.5],
            vec![0.0, 0.5, 1.0],
        ];
        let (w, h) = nmf_decompose(&data, 2, 100);
        for row in &w {
            for &v in row {
                assert!(v >= 0.0, "W should be non-negative, got {}", v);
            }
        }
        for row in &h {
            for &v in row {
                assert!(v >= 0.0, "H should be non-negative, got {}", v);
            }
        }
    }

    #[test]
    fn test_nmf_decompose_reconstruction() {
        let data = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.5, 0.5, 0.0],
        ];
        let (w, h) = nmf_decompose(&data, 2, 200);
        let recon = mat_mul(&w, &h);
        // Check that reconstruction is reasonably close
        for i in 0..data.len() {
            for j in 0..data[0].len() {
                assert!(
                    (recon[i][j] - data[i][j]).abs() < 0.5,
                    "reconstruction error too large at ({},{}): {} vs {}",
                    i,
                    j,
                    recon[i][j],
                    data[i][j]
                );
            }
        }
    }

    #[test]
    fn test_rx_anomaly_detect_basic() {
        // Create image with one obvious anomaly
        let mut pixels = vec![vec![0.5, 0.5, 0.5]; 15];
        pixels.push(vec![10.0, 10.0, 10.0]); // anomaly
        let image = HsiImage::new(pixels, 4, 4, 3);
        let scores = rx_anomaly_detect(&image);
        assert_eq!(scores.len(), 16);
        // The last pixel (anomaly) should have the highest score
        let max_idx = scores
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        assert_eq!(max_idx, 15, "anomaly pixel should have highest RX score");
    }

    #[test]
    fn test_rx_anomaly_detect_uniform() {
        // Uniform image: all scores should be similar
        let pixels = vec![vec![1.0, 2.0, 3.0]; 9];
        let image = HsiImage::new(pixels, 3, 3, 3);
        let scores = rx_anomaly_detect(&image);
        // All identical pixels should have very low (near-zero) scores
        for &s in &scores {
            assert!(
                s < 1e-4,
                "uniform pixels should have near-zero RX score, got {}",
                s
            );
        }
    }

    #[test]
    fn test_pca_reduce_dimensions() {
        let data = vec![
            vec![1.0, 2.0, 3.0, 4.0],
            vec![2.0, 3.0, 4.0, 5.0],
            vec![3.0, 4.0, 5.0, 6.0],
            vec![4.0, 5.0, 6.0, 7.0],
        ];
        let reduced = pca_reduce(&data, 2);
        assert_eq!(reduced.len(), 4);
        assert_eq!(reduced[0].len(), 2);
    }

    #[test]
    fn test_pca_reduce_preserves_structure() {
        // Two clusters in 4D
        let data = vec![
            vec![1.0, 1.0, 0.0, 0.0],
            vec![1.1, 0.9, 0.1, 0.0],
            vec![0.9, 1.1, 0.0, 0.1],
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.1, 0.0, 1.1, 0.9],
            vec![0.0, 0.1, 0.9, 1.1],
        ];
        let reduced = pca_reduce(&data, 1);
        assert_eq!(reduced[0].len(), 1);
        // First cluster should have similar projections, different from second cluster
        let cluster1_mean = (reduced[0][0] + reduced[1][0] + reduced[2][0]) / 3.0;
        let cluster2_mean = (reduced[3][0] + reduced[4][0] + reduced[5][0]) / 3.0;
        assert!(
            (cluster1_mean - cluster2_mean).abs() > 0.1,
            "PCA should separate the two clusters"
        );
    }

    #[test]
    fn test_extract_endmembers_nfindr_count() {
        let image = make_test_image();
        let endmembers = extract_endmembers_nfindr(&image, 3);
        assert_eq!(endmembers.len(), 3);
        for em in &endmembers {
            assert_eq!(em.len(), 3);
        }
    }

    #[test]
    fn test_extract_endmembers_vca_count() {
        let image = make_test_image();
        let endmembers = extract_endmembers_vca(&image, 3);
        assert_eq!(endmembers.len(), 3);
        for em in &endmembers {
            assert_eq!(em.len(), 3);
        }
    }

    #[test]
    fn test_extract_endmembers_nfindr_are_image_pixels() {
        let image = make_test_image();
        let endmembers = extract_endmembers_nfindr(&image, 3);
        for em in &endmembers {
            assert!(
                image.pixels.contains(em),
                "N-FINDR endmember should be an actual image pixel"
            );
        }
    }

    #[test]
    fn test_extract_endmembers_vca_are_image_pixels() {
        let image = make_test_image();
        let endmembers = extract_endmembers_vca(&image, 3);
        for em in &endmembers {
            assert!(
                image.pixels.contains(em),
                "VCA endmember should be an actual image pixel"
            );
        }
    }

    #[test]
    fn test_hsi_processor_workflow() {
        let config = HsiConfig::new(3, 3, 100);
        let processor = HsiProcessor::new(config);
        let image = make_test_image();

        let endmembers = processor.extract_endmembers_vca(&image);
        assert_eq!(endmembers.len(), 3);

        let pixel = vec![0.5, 0.3, 0.2];
        let abundances = processor.unmix_pixel_fcls(&pixel, &endmembers);
        assert_eq!(abundances.len(), 3);
        let sum: f64 = abundances.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-6,
            "FCLS abundances from processor should sum to 1"
        );
    }

    #[test]
    fn test_spectral_angle_zero_vector() {
        let a = vec![0.0, 0.0, 0.0];
        let b = vec![1.0, 2.0, 3.0];
        let angle = spectral_angle(&a, &b);
        assert!(
            (angle - PI / 2.0).abs() < 1e-10,
            "zero vector should give pi/2 angle"
        );
    }

    #[test]
    fn test_unmix_fcls_mixed_pixel() {
        let endmembers = vec![
            vec![1.0, 0.0, 0.0],
            vec![0.0, 1.0, 0.0],
            vec![0.0, 0.0, 1.0],
        ];
        // 50% red, 30% green, 20% blue
        let pixel = vec![0.5, 0.3, 0.2];
        let abundances = unmix_fcls(&pixel, &endmembers);
        // Should approximately recover the mixing fractions
        assert!(
            (abundances[0] - 0.5).abs() < 0.15,
            "red abundance should be ~0.5, got {}",
            abundances[0]
        );
        assert!(
            (abundances[1] - 0.3).abs() < 0.15,
            "green abundance should be ~0.3, got {}",
            abundances[1]
        );
        assert!(
            (abundances[2] - 0.2).abs() < 0.15,
            "blue abundance should be ~0.2, got {}",
            abundances[2]
        );
    }
}
