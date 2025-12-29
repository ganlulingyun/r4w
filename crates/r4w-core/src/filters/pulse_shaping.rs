//! Pulse Shaping Filters
//!
//! Filters for controlling the spectral characteristics of digital signals.

use std::f64::consts::PI;

/// Trait for pulse shaping filters
pub trait PulseShapingFilter: std::fmt::Debug + Send + Sync {
    /// Get the filter coefficients (impulse response)
    fn coefficients(&self) -> &[f64];

    /// Get the filter length in samples
    fn length(&self) -> usize {
        self.coefficients().len()
    }

    /// Get the filter delay in samples
    fn delay(&self) -> usize {
        self.length() / 2
    }

    /// Apply the filter to input samples (convolution)
    fn filter(&self, input: &[f64]) -> Vec<f64> {
        let coeffs = self.coefficients();
        let n = coeffs.len();

        if input.is_empty() || n == 0 {
            return vec![];
        }

        let output_len = input.len() + n - 1;
        let mut output = vec![0.0; output_len];

        for (i, &x) in input.iter().enumerate() {
            for (j, &h) in coeffs.iter().enumerate() {
                output[i + j] += x * h;
            }
        }

        output
    }

    /// Filter complex samples
    fn filter_complex(&self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let coeffs = self.coefficients();
        let n = coeffs.len();

        if input.is_empty() || n == 0 {
            return vec![];
        }

        let output_len = input.len() + n - 1;
        let mut output = vec![(0.0, 0.0); output_len];

        for (i, &(re, im)) in input.iter().enumerate() {
            for (j, &h) in coeffs.iter().enumerate() {
                output[i + j].0 += re * h;
                output[i + j].1 += im * h;
            }
        }

        output
    }
}

/// Raised Cosine Filter
///
/// Nyquist filter that achieves zero ISI at optimal sampling points.
/// Characterized by the roll-off factor α (0 to 1).
///
/// - α = 0: Sinc (minimum bandwidth, maximum ISI sensitivity)
/// - α = 1: Maximum bandwidth, best ISI tolerance
/// - Typical: α = 0.25 to 0.35
#[derive(Debug, Clone)]
pub struct RaisedCosineFilter {
    /// Filter coefficients
    coefficients: Vec<f64>,
    /// Roll-off factor (0 to 1)
    rolloff: f64,
    /// Filter span in symbols
    #[allow(dead_code)]
    span_symbols: usize,
    /// Samples per symbol
    #[allow(dead_code)]
    samples_per_symbol: usize,
}

impl RaisedCosineFilter {
    /// Create a new Raised Cosine filter
    ///
    /// # Arguments
    /// - `rolloff`: Roll-off factor α (0 to 1)
    /// - `span_symbols`: Filter length in symbols (typically 6-10)
    /// - `samples_per_symbol`: Oversampling factor
    pub fn new(rolloff: f64, span_symbols: usize, samples_per_symbol: usize) -> Self {
        assert!(rolloff >= 0.0 && rolloff <= 1.0, "Roll-off must be 0 to 1");
        assert!(span_symbols > 0, "Span must be positive");
        assert!(samples_per_symbol > 0, "Samples per symbol must be positive");

        let length = span_symbols * samples_per_symbol + 1;
        let delay = (length - 1) / 2;
        let ts = 1.0 / samples_per_symbol as f64; // Normalized symbol period

        let mut coefficients = vec![0.0; length];

        for i in 0..length {
            let t = (i as f64 - delay as f64) * ts;

            if t.abs() < 1e-10 {
                // t = 0
                coefficients[i] = 1.0;
            } else if rolloff > 0.0 && (1.0 - (2.0 * rolloff * t).abs()).abs() < 1e-10 {
                // t = ±1/(2α)
                coefficients[i] = (PI / 4.0) * sinc(1.0 / (2.0 * rolloff));
            } else {
                // General case
                let sinc_term = sinc(t);
                let cos_term = (PI * rolloff * t).cos();
                let denom = 1.0 - (2.0 * rolloff * t).powi(2);
                coefficients[i] = sinc_term * cos_term / denom;
            }
        }

        // Normalize
        let sum: f64 = coefficients.iter().sum();
        if sum.abs() > 1e-10 {
            for c in &mut coefficients {
                *c /= sum;
            }
        }

        Self {
            coefficients,
            rolloff,
            span_symbols,
            samples_per_symbol,
        }
    }

    /// Get the roll-off factor
    pub fn rolloff(&self) -> f64 {
        self.rolloff
    }

    /// Get the bandwidth efficiency
    pub fn bandwidth_efficiency(&self) -> f64 {
        1.0 / (1.0 + self.rolloff)
    }
}

impl PulseShapingFilter for RaisedCosineFilter {
    fn coefficients(&self) -> &[f64] {
        &self.coefficients
    }
}

/// Root Raised Cosine Filter
///
/// The square root of the Raised Cosine response. When used at both
/// TX and RX, the cascade gives Raised Cosine (zero ISI).
///
/// This is the most common pulse shaping filter in modern communications.
#[derive(Debug, Clone)]
pub struct RootRaisedCosineFilter {
    /// Filter coefficients
    coefficients: Vec<f64>,
    /// Roll-off factor (0 to 1)
    rolloff: f64,
    /// Filter span in symbols
    #[allow(dead_code)]
    span_symbols: usize,
    /// Samples per symbol
    #[allow(dead_code)]
    samples_per_symbol: usize,
}

impl RootRaisedCosineFilter {
    /// Create a new Root Raised Cosine filter
    ///
    /// # Arguments
    /// - `rolloff`: Roll-off factor α (0 to 1)
    /// - `span_symbols`: Filter length in symbols (typically 6-10)
    /// - `samples_per_symbol`: Oversampling factor
    pub fn new(rolloff: f64, span_symbols: usize, samples_per_symbol: usize) -> Self {
        assert!(rolloff >= 0.0 && rolloff <= 1.0, "Roll-off must be 0 to 1");
        assert!(span_symbols > 0, "Span must be positive");
        assert!(samples_per_symbol > 0, "Samples per symbol must be positive");

        let length = span_symbols * samples_per_symbol + 1;
        let delay = (length - 1) / 2;
        let ts = 1.0 / samples_per_symbol as f64;

        let mut coefficients = vec![0.0; length];

        for i in 0..length {
            let t = (i as f64 - delay as f64) * ts;

            if t.abs() < 1e-10 {
                // t = 0
                coefficients[i] = 1.0 - rolloff + 4.0 * rolloff / PI;
            } else if rolloff > 0.0 && ((4.0 * rolloff * t).abs() - 1.0).abs() < 1e-10 {
                // t = ±1/(4α)
                let term1 = (1.0 + 2.0 / PI) * (PI / (4.0 * rolloff)).sin();
                let term2 = (1.0 - 2.0 / PI) * (PI / (4.0 * rolloff)).cos();
                coefficients[i] = rolloff / 2.0_f64.sqrt() * (term1 + term2);
            } else {
                // General case
                let pi_t = PI * t;
                let four_alpha_t = 4.0 * rolloff * t;

                let num = (pi_t * (1.0 - rolloff)).sin()
                    + four_alpha_t * (pi_t * (1.0 + rolloff)).cos();
                let denom = pi_t * (1.0 - four_alpha_t.powi(2));

                if denom.abs() > 1e-10 {
                    coefficients[i] = num / denom;
                }
            }
        }

        // Normalize for unity gain
        let energy: f64 = coefficients.iter().map(|&x| x * x).sum();
        if energy > 1e-10 {
            let norm = energy.sqrt();
            for c in &mut coefficients {
                *c /= norm;
            }
        }

        Self {
            coefficients,
            rolloff,
            span_symbols,
            samples_per_symbol,
        }
    }

    /// Get the roll-off factor
    pub fn rolloff(&self) -> f64 {
        self.rolloff
    }
}

impl PulseShapingFilter for RootRaisedCosineFilter {
    fn coefficients(&self) -> &[f64] {
        &self.coefficients
    }
}

/// Gaussian Filter
///
/// Used in GMSK (Gaussian Minimum Shift Keying) for smooth phase transitions.
/// The BT product controls the trade-off between spectral efficiency and ISI.
///
/// - BT = 0.3: GSM standard (good spectral containment, some ISI)
/// - BT = 0.5: Bluetooth (less spectral containment, less ISI)
#[derive(Debug, Clone)]
pub struct GaussianFilter {
    /// Filter coefficients
    coefficients: Vec<f64>,
    /// BT product (bandwidth-time product)
    bt_product: f64,
    /// Filter span in symbols
    #[allow(dead_code)]
    span_symbols: usize,
    /// Samples per symbol
    #[allow(dead_code)]
    samples_per_symbol: usize,
}

impl GaussianFilter {
    /// Create a new Gaussian filter
    ///
    /// # Arguments
    /// - `bt_product`: Bandwidth-time product (typically 0.3 to 0.5)
    /// - `span_symbols`: Filter length in symbols
    /// - `samples_per_symbol`: Oversampling factor
    pub fn new(bt_product: f64, span_symbols: usize, samples_per_symbol: usize) -> Self {
        assert!(bt_product > 0.0, "BT product must be positive");
        assert!(span_symbols > 0, "Span must be positive");
        assert!(samples_per_symbol > 0, "Samples per symbol must be positive");

        let length = span_symbols * samples_per_symbol + 1;
        let delay = (length - 1) / 2;
        let ts = 1.0 / samples_per_symbol as f64;

        // Gaussian parameter
        let alpha = (2.0 * PI * bt_product) / (2.0_f64.ln().sqrt());

        let mut coefficients = vec![0.0; length];

        for i in 0..length {
            let t = (i as f64 - delay as f64) * ts;
            coefficients[i] = alpha / PI.sqrt() * (-alpha.powi(2) * t.powi(2)).exp();
        }

        // Normalize
        let sum: f64 = coefficients.iter().sum();
        if sum.abs() > 1e-10 {
            for c in &mut coefficients {
                *c /= sum;
            }
        }

        Self {
            coefficients,
            bt_product,
            span_symbols,
            samples_per_symbol,
        }
    }

    /// Create a GSM-standard Gaussian filter (BT = 0.3)
    pub fn gsm(samples_per_symbol: usize) -> Self {
        Self::new(0.3, 4, samples_per_symbol)
    }

    /// Create a Bluetooth-standard Gaussian filter (BT = 0.5)
    pub fn bluetooth(samples_per_symbol: usize) -> Self {
        Self::new(0.5, 4, samples_per_symbol)
    }

    /// Get the BT product
    pub fn bt_product(&self) -> f64 {
        self.bt_product
    }
}

impl PulseShapingFilter for GaussianFilter {
    fn coefficients(&self) -> &[f64] {
        &self.coefficients
    }
}

/// Sinc function: sin(πx)/(πx), with sinc(0) = 1
fn sinc(x: f64) -> f64 {
    if x.abs() < 1e-10 {
        1.0
    } else {
        let px = PI * x;
        px.sin() / px
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_raised_cosine_creation() {
        let rc = RaisedCosineFilter::new(0.35, 8, 4);

        assert_eq!(rc.rolloff(), 0.35);
        assert!(!rc.coefficients().is_empty());
        assert_eq!(rc.length(), 8 * 4 + 1);
    }

    #[test]
    fn test_root_raised_cosine_creation() {
        let rrc = RootRaisedCosineFilter::new(0.35, 8, 4);

        assert_eq!(rrc.rolloff(), 0.35);
        assert!(!rrc.coefficients().is_empty());
    }

    #[test]
    fn test_gaussian_creation() {
        let gauss = GaussianFilter::new(0.3, 4, 4);

        assert_eq!(gauss.bt_product(), 0.3);
        assert!(!gauss.coefficients().is_empty());
    }

    #[test]
    fn test_gaussian_presets() {
        let gsm = GaussianFilter::gsm(4);
        assert_eq!(gsm.bt_product(), 0.3);

        let bt = GaussianFilter::bluetooth(4);
        assert_eq!(bt.bt_product(), 0.5);
    }

    #[test]
    fn test_filter_convolution() {
        let rrc = RootRaisedCosineFilter::new(0.35, 4, 4);

        // Impulse response test
        let impulse = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let response = rrc.filter(&impulse);

        // Response should match coefficients (approximately, shifted)
        assert!(!response.is_empty());
    }

    #[test]
    fn test_rrc_symmetry() {
        let rrc = RootRaisedCosineFilter::new(0.35, 8, 4);
        let coeffs = rrc.coefficients();
        let n = coeffs.len();

        // RRC should be symmetric
        for i in 0..n / 2 {
            assert!(
                (coeffs[i] - coeffs[n - 1 - i]).abs() < 1e-10,
                "RRC should be symmetric"
            );
        }
    }

    #[test]
    fn test_sinc() {
        assert!((sinc(0.0) - 1.0).abs() < 1e-10);
        assert!((sinc(1.0) - 0.0).abs() < 1e-10);
        assert!((sinc(0.5)).abs() < 1.0);
    }

    #[test]
    fn test_bandwidth_efficiency() {
        let rc_25 = RaisedCosineFilter::new(0.25, 8, 4);
        let rc_50 = RaisedCosineFilter::new(0.50, 8, 4);

        // Lower roll-off = higher efficiency
        assert!(rc_25.bandwidth_efficiency() > rc_50.bandwidth_efficiency());
    }
}
