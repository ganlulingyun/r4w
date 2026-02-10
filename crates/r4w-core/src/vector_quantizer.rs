//! # Vector Quantizer
//!
//! Codebook-based vector quantization for signal compression, clustering,
//! and classification. Supports encoding (find nearest codebook entry),
//! decoding (reconstruct from index), and codebook training via k-means.
//!
//! ## Applications
//! - Speech/audio codec codebooks (CELP, ACELP)
//! - Blind signal clustering and modulation classification
//! - Lossy IQ compression for storage/transmission
//! - Feature vector classification
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vector_quantizer::{VectorQuantizer, DistanceMetric};
//!
//! // Create a codebook with 4 entries of dimension 2
//! let codebook = vec![
//!     vec![1.0, 0.0],
//!     vec![0.0, 1.0],
//!     vec![-1.0, 0.0],
//!     vec![0.0, -1.0],
//! ];
//! let vq = VectorQuantizer::new(codebook, DistanceMetric::Euclidean);
//!
//! // Quantize a vector
//! let (index, distortion) = vq.quantize(&[0.9, 0.1]);
//! assert_eq!(index, 0); // nearest to [1, 0]
//! ```

/// Distance metric for codebook search.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DistanceMetric {
    /// Euclidean (L2) distance.
    Euclidean,
    /// Squared Euclidean distance (faster, avoids sqrt).
    EuclideanSquared,
    /// Manhattan (L1) distance.
    Manhattan,
}

/// Vector quantizer with configurable codebook and distance metric.
#[derive(Debug, Clone)]
pub struct VectorQuantizer {
    /// Codebook: M vectors of dimension N.
    codebook: Vec<Vec<f64>>,
    /// Vector dimension.
    dimension: usize,
    /// Distance metric.
    metric: DistanceMetric,
}

impl VectorQuantizer {
    /// Create a new vector quantizer from a codebook.
    ///
    /// # Panics
    /// Panics if the codebook is empty or vectors have inconsistent dimensions.
    pub fn new(codebook: Vec<Vec<f64>>, metric: DistanceMetric) -> Self {
        assert!(!codebook.is_empty(), "Codebook must not be empty");
        let dimension = codebook[0].len();
        assert!(dimension > 0, "Codebook vectors must have dimension > 0");
        for (i, v) in codebook.iter().enumerate() {
            assert_eq!(
                v.len(),
                dimension,
                "Codebook entry {} has dimension {} (expected {})",
                i,
                v.len(),
                dimension
            );
        }
        Self {
            codebook,
            dimension,
            metric,
        }
    }

    /// Quantize a vector: find the nearest codebook entry.
    /// Returns (index, distortion).
    pub fn quantize(&self, input: &[f64]) -> (usize, f64) {
        assert_eq!(input.len(), self.dimension);
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;

        for (i, entry) in self.codebook.iter().enumerate() {
            let dist = self.distance(input, entry);
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }

        (best_idx, best_dist)
    }

    /// Decode an index back to the codebook vector.
    pub fn decode(&self, index: usize) -> &[f64] {
        &self.codebook[index]
    }

    /// Quantize a batch of vectors.
    pub fn quantize_batch(&self, inputs: &[Vec<f64>]) -> Vec<(usize, f64)> {
        inputs.iter().map(|v| self.quantize(v)).collect()
    }

    /// Decode a batch of indices.
    pub fn decode_batch(&self, indices: &[usize]) -> Vec<Vec<f64>> {
        indices.iter().map(|&i| self.codebook[i].clone()).collect()
    }

    /// Compute average distortion for a dataset.
    pub fn average_distortion(&self, data: &[Vec<f64>]) -> f64 {
        if data.is_empty() {
            return 0.0;
        }
        let total: f64 = data.iter().map(|v| self.quantize(v).1).sum();
        total / data.len() as f64
    }

    /// Get the codebook size (number of entries).
    pub fn codebook_size(&self) -> usize {
        self.codebook.len()
    }

    /// Get the vector dimension.
    pub fn dimension(&self) -> usize {
        self.dimension
    }

    /// Get a reference to the codebook.
    pub fn codebook(&self) -> &[Vec<f64>] {
        &self.codebook
    }

    /// Get the bits per vector (log2 of codebook size).
    pub fn bits_per_vector(&self) -> f64 {
        (self.codebook.len() as f64).log2()
    }

    fn distance(&self, a: &[f64], b: &[f64]) -> f64 {
        match self.metric {
            DistanceMetric::Euclidean => {
                let sq: f64 = a.iter().zip(b).map(|(x, y)| (x - y) * (x - y)).sum();
                sq.sqrt()
            }
            DistanceMetric::EuclideanSquared => {
                a.iter().zip(b).map(|(x, y)| (x - y) * (x - y)).sum()
            }
            DistanceMetric::Manhattan => {
                a.iter().zip(b).map(|(x, y)| (x - y).abs()).sum()
            }
        }
    }
}

/// Train a codebook using k-means clustering.
///
/// # Arguments
/// * `data` - Training vectors
/// * `num_clusters` - Number of codebook entries (M)
/// * `max_iterations` - Maximum k-means iterations
/// * `seed` - Random seed for initialization
///
/// # Returns
/// A trained codebook (Vec of centroid vectors).
pub fn train_codebook_kmeans(
    data: &[Vec<f64>],
    num_clusters: usize,
    max_iterations: usize,
    seed: u64,
) -> Vec<Vec<f64>> {
    assert!(!data.is_empty(), "Training data must not be empty");
    assert!(num_clusters > 0, "num_clusters must be > 0");
    let dim = data[0].len();
    let k = num_clusters.min(data.len());

    // Initialize centroids by selecting evenly-spaced data points.
    let mut centroids: Vec<Vec<f64>> = Vec::with_capacity(k);
    let mut rng_state = seed;
    for i in 0..k {
        // Simple deterministic selection with shuffle.
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let idx = ((rng_state >> 33) as usize + i * data.len() / k) % data.len();
        centroids.push(data[idx].clone());
    }

    let mut assignments = vec![0usize; data.len()];

    for _iter in 0..max_iterations {
        // Assign each point to nearest centroid.
        let mut changed = false;
        for (i, point) in data.iter().enumerate() {
            let mut best = 0;
            let mut best_dist = f64::MAX;
            for (j, centroid) in centroids.iter().enumerate() {
                let dist: f64 = point
                    .iter()
                    .zip(centroid)
                    .map(|(a, b)| (a - b) * (a - b))
                    .sum();
                if dist < best_dist {
                    best_dist = dist;
                    best = j;
                }
            }
            if assignments[i] != best {
                assignments[i] = best;
                changed = true;
            }
        }

        if !changed {
            break;
        }

        // Update centroids.
        let mut sums = vec![vec![0.0; dim]; k];
        let mut counts = vec![0usize; k];
        for (i, point) in data.iter().enumerate() {
            let c = assignments[i];
            counts[c] += 1;
            for (d, val) in point.iter().enumerate() {
                sums[c][d] += val;
            }
        }
        for j in 0..k {
            if counts[j] > 0 {
                for d in 0..dim {
                    centroids[j][d] = sums[j][d] / counts[j] as f64;
                }
            }
        }
    }

    centroids
}

/// Generate a uniform scalar codebook for 1D quantization.
pub fn uniform_codebook(min_val: f64, max_val: f64, levels: usize) -> Vec<Vec<f64>> {
    assert!(levels >= 2, "Need at least 2 levels");
    let step = (max_val - min_val) / (levels - 1) as f64;
    (0..levels)
        .map(|i| vec![min_val + i as f64 * step])
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn qpsk_codebook() -> Vec<Vec<f64>> {
        vec![
            vec![1.0, 0.0],
            vec![0.0, 1.0],
            vec![-1.0, 0.0],
            vec![0.0, -1.0],
        ]
    }

    #[test]
    fn test_exact_match() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::Euclidean);
        let (idx, dist) = vq.quantize(&[1.0, 0.0]);
        assert_eq!(idx, 0);
        assert!(dist < 1e-10);
    }

    #[test]
    fn test_nearest_neighbor() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::Euclidean);
        let (idx, _) = vq.quantize(&[0.8, 0.2]);
        assert_eq!(idx, 0); // closest to [1, 0]

        let (idx, _) = vq.quantize(&[-0.5, -0.9]);
        assert_eq!(idx, 3); // closest to [0, -1]
    }

    #[test]
    fn test_decode() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::Euclidean);
        assert_eq!(vq.decode(2), &[-1.0, 0.0]);
    }

    #[test]
    fn test_batch_quantize() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::EuclideanSquared);
        let inputs = vec![
            vec![0.9, 0.1],
            vec![-0.1, 0.9],
            vec![-0.8, -0.1],
        ];
        let results = vq.quantize_batch(&inputs);
        assert_eq!(results[0].0, 0);
        assert_eq!(results[1].0, 1);
        assert_eq!(results[2].0, 2);
    }

    #[test]
    fn test_manhattan_metric() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::Manhattan);
        let (idx, dist) = vq.quantize(&[1.0, 0.0]);
        assert_eq!(idx, 0);
        assert!(dist < 1e-10);
    }

    #[test]
    fn test_average_distortion() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::EuclideanSquared);
        let data = qpsk_codebook();
        let avg = vq.average_distortion(&data);
        assert!(avg < 1e-10, "Exact matches should have zero distortion");
    }

    #[test]
    fn test_codebook_info() {
        let vq = VectorQuantizer::new(qpsk_codebook(), DistanceMetric::Euclidean);
        assert_eq!(vq.codebook_size(), 4);
        assert_eq!(vq.dimension(), 2);
        assert!((vq.bits_per_vector() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_kmeans_training() {
        // Generate clustered data around 4 points.
        let mut data = Vec::new();
        let centers = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        for (cx, cy) in &centers {
            for i in 0..10 {
                let offset = (i as f64) * 0.01;
                data.push(vec![cx + offset, cy + offset]);
                data.push(vec![cx - offset, cy - offset]);
            }
        }

        let codebook = train_codebook_kmeans(&data, 4, 100, 42);
        assert_eq!(codebook.len(), 4);

        // Each centroid should be near one of the cluster centers.
        let vq = VectorQuantizer::new(codebook, DistanceMetric::EuclideanSquared);
        let avg = vq.average_distortion(&data);
        assert!(avg < 1.0, "K-means codebook should have low distortion, got {}", avg);
    }

    #[test]
    fn test_uniform_codebook() {
        let cb = uniform_codebook(-1.0, 1.0, 5);
        assert_eq!(cb.len(), 5);
        assert!((cb[0][0] - (-1.0)).abs() < 1e-10);
        assert!((cb[4][0] - 1.0).abs() < 1e-10);
        assert!((cb[2][0] - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_1d_scalar_quantization() {
        let cb = uniform_codebook(-1.0, 1.0, 5); // [-1, -0.5, 0, 0.5, 1]
        let vq = VectorQuantizer::new(cb, DistanceMetric::Euclidean);
        let (idx, _) = vq.quantize(&[0.3]);
        assert_eq!(idx, 3); // closest to 0.5
    }
}
