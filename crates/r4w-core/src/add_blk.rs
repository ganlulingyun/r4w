//! # Add / Subtract Streams Block
//!
//! Element-wise addition and subtraction of multiple input streams.
//! Supports real and complex samples. GNU Radio equivalent of
//! `gr::blocks::add` and `gr::blocks::sub`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::add_blk::{add_streams, subtract_streams};
//!
//! let a = vec![1.0, 2.0, 3.0];
//! let b = vec![4.0, 5.0, 6.0];
//! let sum = add_streams(&[&a, &b]);
//! assert_eq!(sum, vec![5.0, 7.0, 9.0]);
//!
//! let diff = subtract_streams(&a, &b);
//! assert_eq!(diff, vec![-3.0, -3.0, -3.0]);
//! ```

/// Element-wise add N real streams.
pub fn add_streams(streams: &[&[f64]]) -> Vec<f64> {
    if streams.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let mut result = vec![0.0; len];
    for stream in streams {
        for (i, &val) in stream[..len].iter().enumerate() {
            result[i] += val;
        }
    }
    result
}

/// Element-wise add N complex streams.
pub fn add_complex(streams: &[&[(f64, f64)]]) -> Vec<(f64, f64)> {
    if streams.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let mut result = vec![(0.0, 0.0); len];
    for stream in streams {
        for (i, &(re, im)) in stream[..len].iter().enumerate() {
            result[i].0 += re;
            result[i].1 += im;
        }
    }
    result
}

/// Element-wise subtract: a - b.
pub fn subtract_streams(a: &[f64], b: &[f64]) -> Vec<f64> {
    let len = a.len().min(b.len());
    (0..len).map(|i| a[i] - b[i]).collect()
}

/// Element-wise subtract complex: a - b.
pub fn subtract_complex(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let len = a.len().min(b.len());
    (0..len).map(|i| (a[i].0 - b[i].0, a[i].1 - b[i].1)).collect()
}

/// Add a constant to every sample.
pub fn add_const(input: &[f64], constant: f64) -> Vec<f64> {
    input.iter().map(|&x| x + constant).collect()
}

/// Add a complex constant to every sample.
pub fn add_const_complex(input: &[(f64, f64)], constant: (f64, f64)) -> Vec<(f64, f64)> {
    input
        .iter()
        .map(|&(re, im)| (re + constant.0, im + constant.1))
        .collect()
}

/// Weighted sum of streams: result[i] = sum_k(weights[k] * streams[k][i]).
pub fn weighted_add(streams: &[&[f64]], weights: &[f64]) -> Vec<f64> {
    if streams.is_empty() || weights.is_empty() {
        return Vec::new();
    }
    let len = streams.iter().map(|s| s.len()).min().unwrap_or(0);
    let mut result = vec![0.0; len];
    for (stream, &weight) in streams.iter().zip(weights.iter()) {
        for (i, &val) in stream[..len].iter().enumerate() {
            result[i] += val * weight;
        }
    }
    result
}

/// In-place add: a += b.
pub fn add_inplace(a: &mut [f64], b: &[f64]) {
    let len = a.len().min(b.len());
    for i in 0..len {
        a[i] += b[i];
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_two() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![4.0, 5.0, 6.0];
        assert_eq!(add_streams(&[&a, &b]), vec![5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_add_three() {
        let a = vec![1.0, 1.0];
        let b = vec![2.0, 2.0];
        let c = vec![3.0, 3.0];
        assert_eq!(add_streams(&[&a, &b, &c]), vec![6.0, 6.0]);
    }

    #[test]
    fn test_add_complex() {
        let a = vec![(1.0, 2.0), (3.0, 4.0)];
        let b = vec![(5.0, 6.0), (7.0, 8.0)];
        let result = add_complex(&[&a, &b]);
        assert_eq!(result, vec![(6.0, 8.0), (10.0, 12.0)]);
    }

    #[test]
    fn test_subtract() {
        let a = vec![5.0, 3.0, 1.0];
        let b = vec![1.0, 2.0, 3.0];
        assert_eq!(subtract_streams(&a, &b), vec![4.0, 1.0, -2.0]);
    }

    #[test]
    fn test_subtract_complex() {
        let a = vec![(5.0, 6.0)];
        let b = vec![(1.0, 2.0)];
        assert_eq!(subtract_complex(&a, &b), vec![(4.0, 4.0)]);
    }

    #[test]
    fn test_add_const() {
        let input = vec![1.0, 2.0, 3.0];
        assert_eq!(add_const(&input, 10.0), vec![11.0, 12.0, 13.0]);
    }

    #[test]
    fn test_add_const_complex() {
        let input = vec![(1.0, 2.0)];
        let result = add_const_complex(&input, (3.0, 4.0));
        assert_eq!(result, vec![(4.0, 6.0)]);
    }

    #[test]
    fn test_weighted_add() {
        let a = vec![1.0, 2.0];
        let b = vec![3.0, 4.0];
        let result = weighted_add(&[&a, &b], &[2.0, 0.5]);
        assert_eq!(result, vec![3.5, 6.0]); // 1*2+3*0.5=3.5, 2*2+4*0.5=6
    }

    #[test]
    fn test_add_inplace() {
        let mut a = vec![1.0, 2.0, 3.0];
        let b = vec![4.0, 5.0, 6.0];
        add_inplace(&mut a, &b);
        assert_eq!(a, vec![5.0, 7.0, 9.0]);
    }

    #[test]
    fn test_empty() {
        assert!(add_streams(&[]).is_empty());
        assert!(subtract_streams(&[], &[]).is_empty());
    }
}
