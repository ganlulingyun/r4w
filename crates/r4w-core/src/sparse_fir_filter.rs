//! Sparse FIR filter optimized for coefficient sets where most taps are zero.
//!
//! A standard FIR filter computes `y[n] = sum(h[k] * x[n-k])` for all `k` in `0..N`.
//! When many of the coefficients `h[k]` are zero, this wastes multiplications.
//! [`SparseFirFilter`] stores only the non-zero `(index, coefficient)` pairs and
//! skips zero taps entirely, reducing the number of multiply-accumulate operations
//! from `N` to the number of non-zero taps.
//!
//! # Example
//!
//! ```
//! use r4w_core::sparse_fir_filter::SparseFirFilter;
//!
//! // Create from a full tap vector (zeros are automatically removed)
//! let taps = vec![0.0, 0.5, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.125];
//! let mut filter = SparseFirFilter::new(&taps);
//!
//! assert_eq!(filter.dense_tap_count(), 9);
//! assert_eq!(filter.nonzero_tap_count(), 3);
//! assert!((filter.sparsity_ratio() - 6.0 / 9.0).abs() < 1e-10);
//!
//! // Process samples
//! let output: Vec<f64> = (0..5).map(|_| filter.filter_sample(1.0)).collect();
//! assert!((output[0] - 0.0).abs() < 1e-10); // tap at index 0 is zero
//! assert!((output[1] - 0.5).abs() < 1e-10); // tap at index 1 fires
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// SparseFirFilter (real-valued)
// ---------------------------------------------------------------------------

/// A real-valued FIR filter that stores only non-zero taps.
#[derive(Debug, Clone)]
pub struct SparseFirFilter {
    /// Non-zero taps as (delay-line index, coefficient).
    taps: Vec<(usize, f64)>,
    /// Total length of the equivalent dense filter.
    length: usize,
    /// Circular delay line.
    delay_line: Vec<f64>,
    /// Current write position in the delay line.
    write_pos: usize,
}

impl SparseFirFilter {
    /// Create a sparse filter from a full coefficient vector.
    ///
    /// Taps whose absolute value is exactly `0.0` are discarded.
    pub fn new(dense_taps: &[f64]) -> Self {
        let length = dense_taps.len();
        let taps: Vec<(usize, f64)> = dense_taps
            .iter()
            .enumerate()
            .filter(|(_, &c)| c != 0.0)
            .map(|(i, &c)| (i, c))
            .collect();
        Self {
            taps,
            length,
            delay_line: vec![0.0; length.max(1)],
            write_pos: 0,
        }
    }

    /// Create a sparse filter directly from `(index, coefficient)` pairs.
    ///
    /// `total_length` is the equivalent dense filter length (must be greater
    /// than the largest index).  Pairs with zero coefficients are discarded.
    ///
    /// # Panics
    ///
    /// Panics if any index is `>= total_length`.
    pub fn from_sparse(pairs: &[(usize, f64)], total_length: usize) -> Self {
        let taps: Vec<(usize, f64)> = pairs
            .iter()
            .filter(|(_, c)| *c != 0.0)
            .copied()
            .collect();
        for &(idx, _) in &taps {
            assert!(
                idx < total_length,
                "tap index {} exceeds total_length {}",
                idx,
                total_length
            );
        }
        Self {
            taps,
            length: total_length,
            delay_line: vec![0.0; total_length.max(1)],
            write_pos: 0,
        }
    }

    /// Process a single input sample and return one output sample.
    pub fn filter_sample(&mut self, sample: f64) -> f64 {
        // Write sample into the delay line at the current position.
        self.delay_line[self.write_pos] = sample;

        let mut acc = 0.0;
        for &(idx, coeff) in &self.taps {
            // The sample at delay `idx` is at position (write_pos - idx) mod length.
            let pos = (self.write_pos + self.length - idx) % self.length;
            acc += coeff * self.delay_line[pos];
        }

        // Advance write pointer.
        self.write_pos = (self.write_pos + 1) % self.length;
        acc
    }

    /// Process a block of input samples and return the output vector.
    pub fn filter_block(&mut self, samples: &[f64]) -> Vec<f64> {
        samples.iter().map(|&s| self.filter_sample(s)).collect()
    }

    /// The total number of taps in the equivalent dense filter.
    pub fn dense_tap_count(&self) -> usize {
        self.length
    }

    /// The number of non-zero taps actually stored.
    pub fn nonzero_tap_count(&self) -> usize {
        self.taps.len()
    }

    /// Fraction of taps that are zero: `(dense - nonzero) / dense`.
    ///
    /// Returns `0.0` for a zero-length filter.
    pub fn sparsity_ratio(&self) -> f64 {
        if self.length == 0 {
            return 0.0;
        }
        (self.length - self.taps.len()) as f64 / self.length as f64
    }

    /// Get the coefficient at a given tap index (returns `0.0` for zero taps).
    pub fn get_tap(&self, index: usize) -> f64 {
        self.taps
            .iter()
            .find(|(i, _)| *i == index)
            .map(|(_, c)| *c)
            .unwrap_or(0.0)
    }

    /// Set (or add) the coefficient at a given tap index.
    ///
    /// If the value is `0.0` the tap is removed. If `index >= dense_tap_count()`
    /// the filter length is extended and the delay line is resized.
    pub fn set_tap(&mut self, index: usize, coeff: f64) {
        // Grow filter if needed.
        if index >= self.length {
            self.length = index + 1;
            self.delay_line.resize(self.length, 0.0);
        }

        // Remove existing entry at this index.
        self.taps.retain(|(i, _)| *i != index);

        // Insert new non-zero tap.
        if coeff != 0.0 {
            self.taps.push((index, coeff));
        }
    }

    /// Add a new non-zero tap. If a tap at this index already exists, the
    /// coefficient is replaced.
    pub fn add_tap(&mut self, index: usize, coeff: f64) {
        self.set_tap(index, coeff);
    }

    /// Remove the tap at the given index (equivalent to setting it to 0).
    pub fn remove_tap(&mut self, index: usize) {
        self.taps.retain(|(i, _)| *i != index);
    }

    /// Reset the delay line to all zeros without changing the tap configuration.
    pub fn reset(&mut self) {
        self.delay_line.fill(0.0);
        self.write_pos = 0;
    }

    /// Compute the frequency response at `num_points` evenly-spaced
    /// frequencies in `[0, pi)`.
    ///
    /// Returns a vector of `(magnitude, phase_radians)` pairs.
    pub fn frequency_response(&self, num_points: usize) -> Vec<(f64, f64)> {
        (0..num_points)
            .map(|k| {
                let omega = PI * k as f64 / num_points as f64;
                self.frequency_response_at(omega)
            })
            .collect()
    }

    /// Evaluate the frequency response at a single normalised angular
    /// frequency `omega` (radians/sample, `0..pi`).
    ///
    /// Returns `(magnitude, phase_radians)`.
    pub fn frequency_response_at(&self, omega: f64) -> (f64, f64) {
        let mut re = 0.0;
        let mut im = 0.0;
        for &(idx, coeff) in &self.taps {
            let angle = omega * idx as f64;
            re += coeff * angle.cos();
            im -= coeff * angle.sin();
        }
        let mag = (re * re + im * im).sqrt();
        let phase = im.atan2(re);
        (mag, phase)
    }

    /// Return a dense coefficient vector (useful for comparison/export).
    pub fn to_dense(&self) -> Vec<f64> {
        let mut dense = vec![0.0; self.length];
        for &(idx, coeff) in &self.taps {
            dense[idx] = coeff;
        }
        dense
    }
}

// ---------------------------------------------------------------------------
// SparseFirFilterComplex
// ---------------------------------------------------------------------------

/// A complex-valued FIR filter that stores only non-zero taps.
///
/// Complex samples are represented as `(f64, f64)` — `(real, imag)`.
#[derive(Debug, Clone)]
pub struct SparseFirFilterComplex {
    taps: Vec<(usize, (f64, f64))>,
    length: usize,
    delay_line: Vec<(f64, f64)>,
    write_pos: usize,
}

impl SparseFirFilterComplex {
    /// Create from a full complex coefficient vector.
    pub fn new(dense_taps: &[(f64, f64)]) -> Self {
        let length = dense_taps.len();
        let taps: Vec<(usize, (f64, f64))> = dense_taps
            .iter()
            .enumerate()
            .filter(|(_, c)| c.0 != 0.0 || c.1 != 0.0)
            .map(|(i, &c)| (i, c))
            .collect();
        Self {
            taps,
            length,
            delay_line: vec![(0.0, 0.0); length.max(1)],
            write_pos: 0,
        }
    }

    /// Create directly from sparse `(index, (re, im))` pairs.
    ///
    /// # Panics
    ///
    /// Panics if any index is `>= total_length`.
    pub fn from_sparse(pairs: &[(usize, (f64, f64))], total_length: usize) -> Self {
        let taps: Vec<(usize, (f64, f64))> = pairs
            .iter()
            .filter(|(_, c)| c.0 != 0.0 || c.1 != 0.0)
            .copied()
            .collect();
        for &(idx, _) in &taps {
            assert!(
                idx < total_length,
                "tap index {} exceeds total_length {}",
                idx,
                total_length
            );
        }
        Self {
            taps,
            length: total_length,
            delay_line: vec![(0.0, 0.0); total_length.max(1)],
            write_pos: 0,
        }
    }

    /// Process a single complex sample.
    pub fn filter_sample(&mut self, sample: (f64, f64)) -> (f64, f64) {
        self.delay_line[self.write_pos] = sample;

        let mut acc_re = 0.0;
        let mut acc_im = 0.0;
        for &(idx, (cr, ci)) in &self.taps {
            let pos = (self.write_pos + self.length - idx) % self.length;
            let (sr, si) = self.delay_line[pos];
            // complex multiply: (cr + j*ci) * (sr + j*si)
            acc_re += cr * sr - ci * si;
            acc_im += cr * si + ci * sr;
        }

        self.write_pos = (self.write_pos + 1) % self.length;
        (acc_re, acc_im)
    }

    /// Process a block of complex samples.
    pub fn filter_block(&mut self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        samples.iter().map(|&s| self.filter_sample(s)).collect()
    }

    /// Total length of the equivalent dense filter.
    pub fn dense_tap_count(&self) -> usize {
        self.length
    }

    /// Number of non-zero complex taps.
    pub fn nonzero_tap_count(&self) -> usize {
        self.taps.len()
    }

    /// Fraction of taps that are zero.
    pub fn sparsity_ratio(&self) -> f64 {
        if self.length == 0 {
            return 0.0;
        }
        (self.length - self.taps.len()) as f64 / self.length as f64
    }

    /// Reset the delay line to all zeros.
    pub fn reset(&mut self) {
        self.delay_line.fill((0.0, 0.0));
        self.write_pos = 0;
    }

    /// Get the complex coefficient at a given tap index.
    pub fn get_tap(&self, index: usize) -> (f64, f64) {
        self.taps
            .iter()
            .find(|(i, _)| *i == index)
            .map(|(_, c)| *c)
            .unwrap_or((0.0, 0.0))
    }

    /// Set (or add) the complex coefficient at a given tap index.
    pub fn set_tap(&mut self, index: usize, coeff: (f64, f64)) {
        if index >= self.length {
            self.length = index + 1;
            self.delay_line.resize(self.length, (0.0, 0.0));
        }
        self.taps.retain(|(i, _)| *i != index);
        if coeff.0 != 0.0 || coeff.1 != 0.0 {
            self.taps.push((index, coeff));
        }
    }

    /// Remove the tap at the given index.
    pub fn remove_tap(&mut self, index: usize) {
        self.taps.retain(|(i, _)| *i != index);
    }

    /// Compute frequency response at `num_points` evenly-spaced frequencies.
    pub fn frequency_response(&self, num_points: usize) -> Vec<(f64, f64)> {
        (0..num_points)
            .map(|k| {
                let omega = PI * k as f64 / num_points as f64;
                let mut re = 0.0;
                let mut im = 0.0;
                for &(idx, (cr, ci)) in &self.taps {
                    let angle = omega * idx as f64;
                    let cos_a = angle.cos();
                    let sin_a = angle.sin();
                    // H(w) = sum h[k] * e^{-jwk}
                    // h[k] = cr + j*ci, e^{-jwk} = cos(wk) - j*sin(wk)
                    re += cr * cos_a + ci * sin_a;
                    im += -cr * sin_a + ci * cos_a;
                }
                let mag = (re * re + im * im).sqrt();
                let phase = im.atan2(re);
                (mag, phase)
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

    const EPS: f64 = 1e-10;

    #[test]
    fn test_new_extracts_nonzero_taps() {
        let taps = vec![0.0, 0.5, 0.0, 0.0, 0.25];
        let f = SparseFirFilter::new(&taps);
        assert_eq!(f.dense_tap_count(), 5);
        assert_eq!(f.nonzero_tap_count(), 2);
    }

    #[test]
    fn test_sparsity_ratio() {
        let taps = vec![0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let f = SparseFirFilter::new(&taps);
        assert!((f.sparsity_ratio() - 0.8).abs() < EPS);
    }

    #[test]
    fn test_sparsity_ratio_zero_length() {
        let f = SparseFirFilter::new(&[]);
        assert!((f.sparsity_ratio() - 0.0).abs() < EPS);
    }

    #[test]
    fn test_from_sparse() {
        let f = SparseFirFilter::from_sparse(&[(0, 1.0), (4, 0.5)], 8);
        assert_eq!(f.dense_tap_count(), 8);
        assert_eq!(f.nonzero_tap_count(), 2);
        assert!((f.get_tap(0) - 1.0).abs() < EPS);
        assert!((f.get_tap(4) - 0.5).abs() < EPS);
        assert!((f.get_tap(2) - 0.0).abs() < EPS);
    }

    #[test]
    #[should_panic(expected = "tap index 10 exceeds total_length 8")]
    fn test_from_sparse_panics_on_oob() {
        SparseFirFilter::from_sparse(&[(10, 1.0)], 8);
    }

    #[test]
    fn test_filter_sample_impulse_response() {
        // Impulse response of a sparse filter should match the taps.
        let taps = vec![0.0, 0.0, 1.0, 0.0, 0.5];
        let mut f = SparseFirFilter::new(&taps);

        let mut output = Vec::new();
        // Feed impulse then zeros.
        output.push(f.filter_sample(1.0));
        for _ in 0..4 {
            output.push(f.filter_sample(0.0));
        }
        // output should equal the taps vector.
        for (o, &t) in output.iter().zip(taps.iter()) {
            assert!((o - t).abs() < EPS, "expected {}, got {}", t, o);
        }
    }

    #[test]
    fn test_filter_block() {
        let taps = vec![1.0, 0.0, 0.5];
        let mut f = SparseFirFilter::new(&taps);
        let input = vec![1.0, 0.0, 0.0, 0.0];
        let output = f.filter_block(&input);
        assert!((output[0] - 1.0).abs() < EPS);
        assert!((output[1] - 0.0).abs() < EPS);
        assert!((output[2] - 0.5).abs() < EPS);
        assert!((output[3] - 0.0).abs() < EPS);
    }

    #[test]
    fn test_matches_dense_convolution() {
        // Compare sparse filter output against naive dense convolution.
        let dense_taps = vec![0.3, 0.0, 0.0, -0.5, 0.0, 0.2];
        let input = vec![1.0, -0.5, 0.3, 0.7, -1.0, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0];

        // Dense convolution reference
        let mut dense_out = vec![0.0; input.len()];
        for n in 0..input.len() {
            for (k, &h) in dense_taps.iter().enumerate() {
                if k <= n {
                    dense_out[n] += h * input[n - k];
                }
            }
        }

        let mut sf = SparseFirFilter::new(&dense_taps);
        let sparse_out = sf.filter_block(&input);

        for (i, (&d, &s)) in dense_out.iter().zip(sparse_out.iter()).enumerate() {
            assert!(
                (d - s).abs() < 1e-12,
                "mismatch at sample {}: dense={}, sparse={}",
                i,
                d,
                s
            );
        }
    }

    #[test]
    fn test_set_get_tap() {
        let mut f = SparseFirFilter::new(&[0.0, 0.0, 0.0, 0.0]);
        assert!((f.get_tap(1) - 0.0).abs() < EPS);
        f.set_tap(1, 0.75);
        assert!((f.get_tap(1) - 0.75).abs() < EPS);
        assert_eq!(f.nonzero_tap_count(), 1);
        // Set to zero removes it.
        f.set_tap(1, 0.0);
        assert_eq!(f.nonzero_tap_count(), 0);
    }

    #[test]
    fn test_add_remove_tap() {
        let mut f = SparseFirFilter::new(&[1.0, 0.0, 0.0]);
        assert_eq!(f.nonzero_tap_count(), 1);
        f.add_tap(2, 0.5);
        assert_eq!(f.nonzero_tap_count(), 2);
        assert!((f.get_tap(2) - 0.5).abs() < EPS);
        f.remove_tap(0);
        assert_eq!(f.nonzero_tap_count(), 1);
        assert!((f.get_tap(0) - 0.0).abs() < EPS);
    }

    #[test]
    fn test_add_tap_extends_filter() {
        let mut f = SparseFirFilter::new(&[1.0]);
        assert_eq!(f.dense_tap_count(), 1);
        f.add_tap(5, 0.3);
        assert_eq!(f.dense_tap_count(), 6);
        assert_eq!(f.nonzero_tap_count(), 2);
    }

    #[test]
    fn test_reset() {
        let mut f = SparseFirFilter::new(&[1.0, 0.0, 0.5]);
        f.filter_sample(1.0);
        f.filter_sample(2.0);
        f.reset();
        // After reset, feeding 0.0 should produce 0.0
        let out = f.filter_sample(0.0);
        assert!((out - 0.0).abs() < EPS);
    }

    #[test]
    fn test_frequency_response_dc() {
        // At DC (omega=0), H(0) = sum of all taps
        let taps = vec![0.0, 0.25, 0.0, 0.5, 0.0, 0.0, 0.25];
        let f = SparseFirFilter::new(&taps);
        let (mag, _) = f.frequency_response_at(0.0);
        let expected: f64 = taps.iter().sum();
        assert!(
            (mag - expected).abs() < 1e-12,
            "DC mag {} != expected {}",
            mag,
            expected
        );
    }

    #[test]
    fn test_frequency_response_length() {
        let f = SparseFirFilter::new(&[1.0, 0.0, 0.5]);
        let resp = f.frequency_response(64);
        assert_eq!(resp.len(), 64);
    }

    #[test]
    fn test_to_dense_roundtrip() {
        let original = vec![0.0, 0.5, 0.0, 0.0, 0.25, 0.0, 0.125];
        let f = SparseFirFilter::new(&original);
        let recovered = f.to_dense();
        assert_eq!(recovered.len(), original.len());
        for (a, b) in original.iter().zip(recovered.iter()) {
            assert!((a - b).abs() < EPS);
        }
    }

    // -----------------------------------------------------------------------
    // Complex filter tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_complex_impulse_response() {
        let taps = vec![(0.0, 0.0), (1.0, 0.5), (0.0, 0.0), (0.0, -0.3)];
        let mut f = SparseFirFilterComplex::new(&taps);

        let mut output = Vec::new();
        output.push(f.filter_sample((1.0, 0.0)));
        for _ in 0..3 {
            output.push(f.filter_sample((0.0, 0.0)));
        }
        for (o, t) in output.iter().zip(taps.iter()) {
            assert!((o.0 - t.0).abs() < EPS, "re mismatch");
            assert!((o.1 - t.1).abs() < EPS, "im mismatch");
        }
    }

    #[test]
    fn test_complex_sparsity() {
        let taps = vec![
            (0.0, 0.0),
            (0.0, 0.0),
            (1.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.5),
        ];
        let f = SparseFirFilterComplex::new(&taps);
        assert_eq!(f.dense_tap_count(), 5);
        assert_eq!(f.nonzero_tap_count(), 2);
        assert!((f.sparsity_ratio() - 0.6).abs() < EPS);
    }

    #[test]
    fn test_complex_filter_block() {
        let taps = vec![(1.0, 0.0), (0.0, 0.0), (0.5, 0.0)];
        let mut f = SparseFirFilterComplex::new(&taps);
        let input = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
        let output = f.filter_block(&input);
        assert!((output[0].0 - 1.0).abs() < EPS);
        assert!((output[2].0 - 0.5).abs() < EPS);
    }

    #[test]
    fn test_complex_reset() {
        let mut f = SparseFirFilterComplex::new(&[(1.0, 1.0), (0.0, 0.0), (0.5, -0.5)]);
        f.filter_sample((1.0, 1.0));
        f.reset();
        let out = f.filter_sample((0.0, 0.0));
        assert!((out.0).abs() < EPS);
        assert!((out.1).abs() < EPS);
    }

    #[test]
    fn test_complex_set_get_tap() {
        let mut f = SparseFirFilterComplex::new(&[(0.0, 0.0); 4]);
        f.set_tap(2, (0.5, -0.3));
        assert!((f.get_tap(2).0 - 0.5).abs() < EPS);
        assert!((f.get_tap(2).1 - (-0.3)).abs() < EPS);
        assert_eq!(f.nonzero_tap_count(), 1);
    }

    #[test]
    fn test_complex_frequency_response_dc() {
        let taps = vec![(0.0, 0.0), (0.5, 0.1), (0.0, 0.0), (0.3, -0.2)];
        let f = SparseFirFilterComplex::new(&taps);
        let resp = f.frequency_response(128);
        // At DC (index 0), magnitude should be |sum(taps)|
        let sum_re: f64 = taps.iter().map(|t| t.0).sum();
        let sum_im: f64 = taps.iter().map(|t| t.1).sum();
        let expected_mag = (sum_re * sum_re + sum_im * sum_im).sqrt();
        assert!(
            (resp[0].0 - expected_mag).abs() < 1e-12,
            "DC mag {} != {}",
            resp[0].0,
            expected_mag
        );
    }
}
