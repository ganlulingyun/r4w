//! Multiply Matrix — Matrix multiplication for MIMO and beamforming
//!
//! Applies a complex matrix to a vector of input samples, producing
//! a vector of output samples. Essential for MIMO precoding, beamforming
//! weight application, and spatial filtering.
//! GNU Radio equivalent: `multiply_matrix_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::multiply_matrix::MultiplyMatrix;
//! use num_complex::Complex64;
//!
//! // 2x2 identity matrix: output = input
//! let eye = MultiplyMatrix::identity(2);
//! let input = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
//! let output = eye.apply(&input);
//! assert_eq!(output[0], Complex64::new(1.0, 0.0));
//! assert_eq!(output[1], Complex64::new(0.0, 1.0));
//! ```

use num_complex::Complex64;

/// Complex matrix for vector multiplication.
#[derive(Debug, Clone)]
pub struct MultiplyMatrix {
    /// Matrix stored in row-major order.
    data: Vec<Complex64>,
    /// Number of rows (output size).
    rows: usize,
    /// Number of columns (input size).
    cols: usize,
}

impl MultiplyMatrix {
    /// Create from a flat row-major array.
    pub fn new(rows: usize, cols: usize, data: Vec<Complex64>) -> Self {
        assert_eq!(data.len(), rows * cols, "data length must equal rows * cols");
        Self { data, rows, cols }
    }

    /// Create from a 2D array (Vec of rows).
    pub fn from_rows(matrix: &[Vec<Complex64>]) -> Self {
        if matrix.is_empty() {
            return Self {
                data: Vec::new(),
                rows: 0,
                cols: 0,
            };
        }
        let rows = matrix.len();
        let cols = matrix[0].len();
        let mut data = Vec::with_capacity(rows * cols);
        for row in matrix {
            assert_eq!(row.len(), cols, "all rows must have same length");
            data.extend_from_slice(row);
        }
        Self { data, rows, cols }
    }

    /// Create an identity matrix.
    pub fn identity(n: usize) -> Self {
        let mut data = vec![Complex64::new(0.0, 0.0); n * n];
        for i in 0..n {
            data[i * n + i] = Complex64::new(1.0, 0.0);
        }
        Self {
            data,
            rows: n,
            cols: n,
        }
    }

    /// Create a scalar matrix (diagonal with same value).
    pub fn scalar(n: usize, value: Complex64) -> Self {
        let mut data = vec![Complex64::new(0.0, 0.0); n * n];
        for i in 0..n {
            data[i * n + i] = value;
        }
        Self {
            data,
            rows: n,
            cols: n,
        }
    }

    /// Create a diagonal matrix from a vector.
    pub fn diagonal(diag: &[Complex64]) -> Self {
        let n = diag.len();
        let mut data = vec![Complex64::new(0.0, 0.0); n * n];
        for (i, &v) in diag.iter().enumerate() {
            data[i * n + i] = v;
        }
        Self {
            data,
            rows: n,
            cols: n,
        }
    }

    /// Apply matrix to a vector: y = M * x.
    pub fn apply(&self, input: &[Complex64]) -> Vec<Complex64> {
        assert!(
            input.len() >= self.cols,
            "input vector too short: {} < {}",
            input.len(),
            self.cols
        );
        let mut output = Vec::with_capacity(self.rows);
        for r in 0..self.rows {
            let mut sum = Complex64::new(0.0, 0.0);
            for c in 0..self.cols {
                sum += self.data[r * self.cols + c] * input[c];
            }
            output.push(sum);
        }
        output
    }

    /// Process a block of vectors (each `cols` samples → `rows` output samples).
    pub fn process(&self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.cols * self.rows);
        for chunk in input.chunks_exact(self.cols) {
            output.extend(self.apply(chunk));
        }
        output
    }

    /// Get element at (row, col).
    pub fn get(&self, row: usize, col: usize) -> Complex64 {
        self.data[row * self.cols + col]
    }

    /// Set element at (row, col).
    pub fn set(&mut self, row: usize, col: usize, value: Complex64) {
        self.data[row * self.cols + col] = value;
    }

    /// Number of rows (output dimension).
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Number of columns (input dimension).
    pub fn cols(&self) -> usize {
        self.cols
    }

    /// Set matrix from a new set of data.
    pub fn set_data(&mut self, data: Vec<Complex64>) {
        assert_eq!(data.len(), self.rows * self.cols);
        self.data = data;
    }
}

/// Real-valued matrix multiply.
#[derive(Debug, Clone)]
pub struct MultiplyMatrixReal {
    data: Vec<f64>,
    rows: usize,
    cols: usize,
}

impl MultiplyMatrixReal {
    /// Create from flat row-major data.
    pub fn new(rows: usize, cols: usize, data: Vec<f64>) -> Self {
        assert_eq!(data.len(), rows * cols);
        Self { data, rows, cols }
    }

    /// Identity matrix.
    pub fn identity(n: usize) -> Self {
        let mut data = vec![0.0; n * n];
        for i in 0..n {
            data[i * n + i] = 1.0;
        }
        Self {
            data,
            rows: n,
            cols: n,
        }
    }

    /// Apply: y = M * x.
    pub fn apply(&self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(self.rows);
        for r in 0..self.rows {
            let mut sum = 0.0;
            for c in 0..self.cols {
                sum += self.data[r * self.cols + c] * input[c];
            }
            output.push(sum);
        }
        output
    }

    /// Process block.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::new();
        for chunk in input.chunks_exact(self.cols) {
            output.extend(self.apply(chunk));
        }
        output
    }

    /// Rows.
    pub fn rows(&self) -> usize {
        self.rows
    }

    /// Cols.
    pub fn cols(&self) -> usize {
        self.cols
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_identity() {
        let eye = MultiplyMatrix::identity(3);
        let input = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
        ];
        let output = eye.apply(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_scalar() {
        let m = MultiplyMatrix::scalar(2, Complex64::new(2.0, 0.0));
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(3.0, 0.0)];
        let output = m.apply(&input);
        assert_eq!(output[0], Complex64::new(2.0, 0.0));
        assert_eq!(output[1], Complex64::new(6.0, 0.0));
    }

    #[test]
    fn test_2x2_rotation() {
        // 90° rotation matrix
        let m = MultiplyMatrix::new(
            2,
            2,
            vec![
                Complex64::new(0.0, 0.0),
                Complex64::new(-1.0, 0.0),
                Complex64::new(1.0, 0.0),
                Complex64::new(0.0, 0.0),
            ],
        );
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 0.0)];
        let output = m.apply(&input);
        assert!((output[0].re - 0.0).abs() < 1e-10);
        assert!((output[1].re - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_from_rows() {
        let m = MultiplyMatrix::from_rows(&[
            vec![Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0)],
            vec![Complex64::new(3.0, 0.0), Complex64::new(4.0, 0.0)],
        ]);
        assert_eq!(m.rows(), 2);
        assert_eq!(m.cols(), 2);
        assert_eq!(m.get(0, 1), Complex64::new(2.0, 0.0));
    }

    #[test]
    fn test_diagonal() {
        let m = MultiplyMatrix::diagonal(&[
            Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0),
        ]);
        let input = vec![Complex64::new(1.0, 0.0), Complex64::new(1.0, 0.0)];
        let output = m.apply(&input);
        assert_eq!(output[0], Complex64::new(2.0, 0.0));
        assert_eq!(output[1], Complex64::new(3.0, 0.0));
    }

    #[test]
    fn test_nonsquare() {
        // 2x3 matrix: 3 inputs → 2 outputs
        let m = MultiplyMatrix::new(
            2,
            3,
            vec![
                Complex64::new(1.0, 0.0), Complex64::new(0.0, 0.0), Complex64::new(0.0, 0.0),
                Complex64::new(0.0, 0.0), Complex64::new(1.0, 0.0), Complex64::new(0.0, 0.0),
            ],
        );
        let input = vec![
            Complex64::new(5.0, 0.0),
            Complex64::new(3.0, 0.0),
            Complex64::new(7.0, 0.0),
        ];
        let output = m.apply(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0], Complex64::new(5.0, 0.0));
        assert_eq!(output[1], Complex64::new(3.0, 0.0));
    }

    #[test]
    fn test_complex_multiply() {
        let m = MultiplyMatrix::new(
            1,
            1,
            vec![Complex64::new(0.0, 1.0)], // multiply by j
        );
        let output = m.apply(&[Complex64::new(1.0, 0.0)]);
        assert!(output[0].re.abs() < 1e-10);
        assert!((output[0].im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_process_block() {
        let eye = MultiplyMatrix::identity(2);
        let input = vec![
            Complex64::new(1.0, 0.0), Complex64::new(2.0, 0.0),
            Complex64::new(3.0, 0.0), Complex64::new(4.0, 0.0),
        ];
        let output = eye.process(&input);
        assert_eq!(output.len(), 4);
        assert_eq!(output, input);
    }

    #[test]
    fn test_set_element() {
        let mut m = MultiplyMatrix::identity(2);
        m.set(0, 1, Complex64::new(5.0, 0.0));
        assert_eq!(m.get(0, 1), Complex64::new(5.0, 0.0));
    }

    #[test]
    fn test_real_identity() {
        let eye = MultiplyMatrixReal::identity(2);
        let output = eye.apply(&[3.0, 7.0]);
        assert_eq!(output, vec![3.0, 7.0]);
    }

    #[test]
    fn test_real_multiply() {
        let m = MultiplyMatrixReal::new(2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        let output = m.apply(&[1.0, 0.0]);
        assert_eq!(output, vec![1.0, 3.0]); // [1*1+2*0, 3*1+4*0]
    }

    #[test]
    fn test_real_process_block() {
        let m = MultiplyMatrixReal::new(1, 2, vec![1.0, 1.0]); // sum
        let output = m.process(&[1.0, 2.0, 3.0, 4.0]);
        assert_eq!(output, vec![3.0, 7.0]); // 1+2=3, 3+4=7
    }

    #[test]
    fn test_empty_matrix() {
        let m = MultiplyMatrix::from_rows(&[]);
        assert_eq!(m.rows(), 0);
        assert_eq!(m.cols(), 0);
    }
}
