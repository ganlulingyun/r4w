//! IIR Filter implementations
//!
//! Provides Infinite Impulse Response filters using cascaded biquad sections
//! for numerical stability. Supports Butterworth, Chebyshev Type I/II, and Bessel designs.
//!
//! ## Design Methods
//!
//! - **Butterworth**: Maximally flat passband, monotonic rolloff
//! - **Chebyshev Type I**: Equiripple passband, steeper rolloff
//! - **Chebyshev Type II**: Equiripple stopband, flat passband
//! - **Bessel**: Maximally flat group delay (linear phase approximation)
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::filters::{IirFilter, Filter};
//! use num_complex::Complex64;
//!
//! // Create a 4th-order Butterworth lowpass at 1 kHz, 8 kHz sample rate
//! let mut filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);
//!
//! // Process samples
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let output = filter.process_block(&input);
//! ```

use super::traits::{Filter, FilterType, RealFilter};
use num_complex::Complex64;
use std::f64::consts::PI;

/// A single biquad (second-order section) filter.
///
/// Transfer function: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
///
/// Using Direct Form II Transposed for better numerical properties.
#[derive(Debug, Clone)]
pub struct Biquad {
    /// Numerator coefficients [b0, b1, b2]
    b: [f64; 3],
    /// Denominator coefficients [a1, a2] (a0 is normalized to 1)
    a: [f64; 2],
    /// State variables for Direct Form II Transposed
    state: [f64; 2],
    /// Complex state for complex processing
    state_complex: [Complex64; 2],
}

impl Biquad {
    /// Create a new biquad section with given coefficients.
    ///
    /// # Arguments
    /// * `b` - Numerator coefficients [b0, b1, b2]
    /// * `a` - Denominator coefficients [a1, a2] (a0 assumed to be 1)
    pub fn new(b: [f64; 3], a: [f64; 2]) -> Self {
        Self {
            b,
            a,
            state: [0.0; 2],
            state_complex: [Complex64::new(0.0, 0.0); 2],
        }
    }

    /// Create a pass-through (unity gain) biquad.
    pub fn unity() -> Self {
        Self::new([1.0, 0.0, 0.0], [0.0, 0.0])
    }

    /// Process a single real sample using Direct Form II Transposed.
    pub fn process_real(&mut self, input: f64) -> f64 {
        let output = self.b[0] * input + self.state[0];
        self.state[0] = self.b[1] * input - self.a[0] * output + self.state[1];
        self.state[1] = self.b[2] * input - self.a[1] * output;
        output
    }

    /// Process a single complex sample.
    pub fn process_complex(&mut self, input: Complex64) -> Complex64 {
        let output = self.b[0] * input + self.state_complex[0];
        self.state_complex[0] = self.b[1] * input - self.a[0] * output + self.state_complex[1];
        self.state_complex[1] = self.b[2] * input - self.a[1] * output;
        output
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.state = [0.0; 2];
        self.state_complex = [Complex64::new(0.0, 0.0); 2];
    }

    /// Get the numerator coefficients.
    pub fn numerator(&self) -> &[f64; 3] {
        &self.b
    }

    /// Get the denominator coefficients.
    pub fn denominator(&self) -> &[f64; 2] {
        &self.a
    }

    /// Check if this biquad is stable (poles inside unit circle).
    pub fn is_stable(&self) -> bool {
        // For a second-order section with denominator 1 + a1*z^-1 + a2*z^-2,
        // stability requires:
        // |a2| < 1
        // |a1| < 1 + a2
        self.a[1].abs() < 1.0 && self.a[0].abs() < 1.0 + self.a[1]
    }
}

/// IIR filter implemented as a cascade of biquad sections.
///
/// Using cascaded biquads provides better numerical stability than
/// direct implementation of high-order transfer functions.
#[derive(Debug, Clone)]
pub struct IirFilter {
    /// Cascade of biquad sections
    sections: Vec<Biquad>,
    /// Overall gain factor
    gain: f64,
    /// Filter type for reference
    filter_type: FilterType,
    /// Filter order
    order: usize,
}

impl IirFilter {
    /// Create a new IIR filter from biquad sections.
    pub fn new(sections: Vec<Biquad>, gain: f64) -> Self {
        let order = sections.len() * 2;
        Self {
            sections,
            gain,
            filter_type: FilterType::Lowpass,
            order,
        }
    }

    /// Design a Butterworth lowpass filter.
    ///
    /// Butterworth filters have maximally flat passband response.
    ///
    /// # Arguments
    /// * `order` - Filter order (1-10 recommended)
    /// * `cutoff_hz` - Cutoff frequency in Hz (-3 dB point)
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Example
    /// ```rust,ignore
    /// let lpf = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);
    /// ```
    pub fn butterworth_lowpass(order: usize, cutoff_hz: f64, sample_rate: f64) -> Self {
        assert!(order > 0 && order <= 20, "Order must be 1-20");
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0);

        let sections = design_butterworth(order, cutoff_hz, sample_rate, FilterType::Lowpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Lowpass;
        filter.order = order;
        filter
    }

    /// Design a Butterworth highpass filter.
    ///
    /// # Arguments
    /// * `order` - Filter order (1-10 recommended)
    /// * `cutoff_hz` - Cutoff frequency in Hz (-3 dB point)
    /// * `sample_rate` - Sample rate in Hz
    pub fn butterworth_highpass(order: usize, cutoff_hz: f64, sample_rate: f64) -> Self {
        assert!(order > 0 && order <= 20, "Order must be 1-20");
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0);

        let sections = design_butterworth(order, cutoff_hz, sample_rate, FilterType::Highpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Highpass;
        filter.order = order;
        filter
    }

    /// Design a Butterworth bandpass filter.
    ///
    /// # Arguments
    /// * `order` - Filter order per edge (total order = 2*order)
    /// * `low_hz` - Lower cutoff frequency in Hz
    /// * `high_hz` - Upper cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    pub fn butterworth_bandpass(
        order: usize,
        low_hz: f64,
        high_hz: f64,
        sample_rate: f64,
    ) -> Self {
        assert!(order > 0 && order <= 10, "Order must be 1-10");
        assert!(low_hz > 0.0 && low_hz < high_hz && high_hz < sample_rate / 2.0);

        let sections = design_butterworth_bandpass(order, low_hz, high_hz, sample_rate);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Bandpass;
        filter.order = order * 2;
        filter
    }

    /// Design a Chebyshev Type I lowpass filter.
    ///
    /// Chebyshev Type I has equiripple passband and monotonic stopband.
    /// Provides steeper rolloff than Butterworth for the same order.
    ///
    /// # Arguments
    /// * `order` - Filter order (1-10 recommended)
    /// * `ripple_db` - Passband ripple in dB (typically 0.5 to 3.0)
    /// * `cutoff_hz` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    pub fn chebyshev1_lowpass(
        order: usize,
        ripple_db: f64,
        cutoff_hz: f64,
        sample_rate: f64,
    ) -> Self {
        assert!(order > 0 && order <= 20, "Order must be 1-20");
        assert!(ripple_db > 0.0, "Ripple must be positive");
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0);

        let sections = design_chebyshev1(order, ripple_db, cutoff_hz, sample_rate, FilterType::Lowpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Lowpass;
        filter.order = order;
        filter
    }

    /// Design a Chebyshev Type I highpass filter.
    pub fn chebyshev1_highpass(
        order: usize,
        ripple_db: f64,
        cutoff_hz: f64,
        sample_rate: f64,
    ) -> Self {
        assert!(order > 0 && order <= 20, "Order must be 1-20");
        assert!(ripple_db > 0.0, "Ripple must be positive");
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0);

        let sections = design_chebyshev1(order, ripple_db, cutoff_hz, sample_rate, FilterType::Highpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Highpass;
        filter.order = order;
        filter
    }

    /// Design a Chebyshev Type II lowpass filter.
    ///
    /// Chebyshev Type II has flat passband and equiripple stopband.
    ///
    /// # Arguments
    /// * `order` - Filter order (1-10 recommended)
    /// * `stopband_db` - Stopband attenuation in dB (typically 40 to 80)
    /// * `stopband_hz` - Stopband edge frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    pub fn chebyshev2_lowpass(
        order: usize,
        stopband_db: f64,
        stopband_hz: f64,
        sample_rate: f64,
    ) -> Self {
        assert!(order > 0 && order <= 20, "Order must be 1-20");
        assert!(stopband_db > 0.0, "Stopband attenuation must be positive");
        assert!(stopband_hz > 0.0 && stopband_hz < sample_rate / 2.0);

        let sections = design_chebyshev2(order, stopband_db, stopband_hz, sample_rate, FilterType::Lowpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Lowpass;
        filter.order = order;
        filter
    }

    /// Design a Bessel lowpass filter.
    ///
    /// Bessel filters have maximally flat group delay, providing
    /// the best phase linearity among classical IIR designs.
    ///
    /// # Arguments
    /// * `order` - Filter order (1-10 recommended)
    /// * `cutoff_hz` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    pub fn bessel_lowpass(order: usize, cutoff_hz: f64, sample_rate: f64) -> Self {
        assert!(order > 0 && order <= 10, "Order must be 1-10");
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0);

        let sections = design_bessel(order, cutoff_hz, sample_rate, FilterType::Lowpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Lowpass;
        filter.order = order;
        filter
    }

    /// Design a Bessel highpass filter.
    pub fn bessel_highpass(order: usize, cutoff_hz: f64, sample_rate: f64) -> Self {
        assert!(order > 0 && order <= 10, "Order must be 1-10");
        assert!(cutoff_hz > 0.0 && cutoff_hz < sample_rate / 2.0);

        let sections = design_bessel(order, cutoff_hz, sample_rate, FilterType::Highpass);
        let mut filter = Self::new(sections, 1.0);
        filter.filter_type = FilterType::Highpass;
        filter.order = order;
        filter
    }

    /// Get the number of biquad sections.
    pub fn num_sections(&self) -> usize {
        self.sections.len()
    }

    /// Get the filter type.
    pub fn filter_type(&self) -> FilterType {
        self.filter_type
    }

    /// Check if the filter is stable (all poles inside unit circle).
    pub fn is_stable(&self) -> bool {
        self.sections.iter().all(|s| s.is_stable())
    }

    /// Get the frequency response at a given frequency.
    ///
    /// # Arguments
    /// * `freq_hz` - Frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    ///
    /// # Returns
    /// Complex frequency response H(e^jω)
    pub fn frequency_response(&self, freq_hz: f64, sample_rate: f64) -> Complex64 {
        let omega = 2.0 * PI * freq_hz / sample_rate;
        let z = Complex64::new(omega.cos(), omega.sin());
        let z_inv = Complex64::new(omega.cos(), -omega.sin());
        let z_inv2 = z_inv * z_inv;

        let mut response = Complex64::new(self.gain, 0.0);

        for section in &self.sections {
            let num = Complex64::new(section.b[0], 0.0)
                + Complex64::new(section.b[1], 0.0) * z_inv
                + Complex64::new(section.b[2], 0.0) * z_inv2;
            let den = Complex64::new(1.0, 0.0)
                + Complex64::new(section.a[0], 0.0) * z_inv
                + Complex64::new(section.a[1], 0.0) * z_inv2;
            response *= num / den;
        }

        response
    }

    /// Get the magnitude response in dB at a given frequency.
    pub fn magnitude_response_db(&self, freq_hz: f64, sample_rate: f64) -> f64 {
        20.0 * self.frequency_response(freq_hz, sample_rate).norm().log10()
    }

    /// Get the phase response in radians at a given frequency.
    pub fn phase_response(&self, freq_hz: f64, sample_rate: f64) -> f64 {
        let response = self.frequency_response(freq_hz, sample_rate);
        response.im.atan2(response.re)
    }

    /// Get the group delay at a given frequency (in samples).
    ///
    /// Uses numerical differentiation of phase response.
    pub fn group_delay_at(&self, freq_hz: f64, sample_rate: f64) -> f64 {
        let delta = 1.0; // 1 Hz step
        let phase1 = self.phase_response(freq_hz - delta, sample_rate);
        let phase2 = self.phase_response(freq_hz + delta, sample_rate);

        // Unwrap phase difference
        let mut d_phase = phase2 - phase1;
        if d_phase > PI {
            d_phase -= 2.0 * PI;
        } else if d_phase < -PI {
            d_phase += 2.0 * PI;
        }

        // Group delay = -d(phase)/d(omega)
        let d_omega = 2.0 * PI * 2.0 * delta / sample_rate;
        -d_phase / d_omega
    }

    /// Get access to biquad sections for analysis.
    pub fn sections(&self) -> &[Biquad] {
        &self.sections
    }
}

impl Filter for IirFilter {
    fn process(&mut self, input: Complex64) -> Complex64 {
        let mut output = input * self.gain;
        for section in &mut self.sections {
            output = section.process_complex(output);
        }
        output
    }

    fn reset(&mut self) {
        for section in &mut self.sections {
            section.reset();
        }
    }

    fn group_delay(&self) -> f64 {
        // IIR filters have frequency-dependent group delay
        // Return the approximate delay at DC
        self.group_delay_at(0.0, 1.0)
    }

    fn order(&self) -> usize {
        self.order
    }
}

impl RealFilter for IirFilter {
    fn process_real(&mut self, input: f64) -> f64 {
        let mut output = input * self.gain;
        for section in &mut self.sections {
            output = section.process_real(output);
        }
        output
    }
}

// ============================================================================
// Design Functions
// ============================================================================

/// Design Butterworth filter sections using bilinear transform.
fn design_butterworth(
    order: usize,
    cutoff_hz: f64,
    sample_rate: f64,
    filter_type: FilterType,
) -> Vec<Biquad> {
    // Pre-warp the cutoff frequency
    let wc = prewarp(cutoff_hz, sample_rate);

    // Calculate analog prototype poles (on the s-plane unit circle)
    let poles = butterworth_poles(order);

    // Convert to biquad sections
    poles_to_biquads(&poles, wc, sample_rate, filter_type)
}

/// Design Butterworth bandpass filter.
fn design_butterworth_bandpass(
    order: usize,
    low_hz: f64,
    high_hz: f64,
    sample_rate: f64,
) -> Vec<Biquad> {
    // For bandpass, cascade lowpass and highpass
    let mut sections = design_butterworth(order, high_hz, sample_rate, FilterType::Lowpass);
    sections.extend(design_butterworth(order, low_hz, sample_rate, FilterType::Highpass));
    sections
}

/// Design Chebyshev Type I filter sections.
fn design_chebyshev1(
    order: usize,
    ripple_db: f64,
    cutoff_hz: f64,
    sample_rate: f64,
    filter_type: FilterType,
) -> Vec<Biquad> {
    let wc = prewarp(cutoff_hz, sample_rate);
    let poles = chebyshev1_poles(order, ripple_db);
    poles_to_biquads(&poles, wc, sample_rate, filter_type)
}

/// Design Chebyshev Type II filter sections.
fn design_chebyshev2(
    order: usize,
    stopband_db: f64,
    stopband_hz: f64,
    sample_rate: f64,
    filter_type: FilterType,
) -> Vec<Biquad> {
    let wc = prewarp(stopband_hz, sample_rate);
    let (poles, zeros) = chebyshev2_poles_zeros(order, stopband_db);
    poles_zeros_to_biquads(&poles, &zeros, wc, sample_rate, filter_type)
}

/// Design Bessel filter sections.
fn design_bessel(
    order: usize,
    cutoff_hz: f64,
    sample_rate: f64,
    filter_type: FilterType,
) -> Vec<Biquad> {
    let wc = prewarp(cutoff_hz, sample_rate);
    let poles = bessel_poles(order);
    poles_to_biquads(&poles, wc, sample_rate, filter_type)
}

/// Pre-warp frequency for bilinear transform.
fn prewarp(freq_hz: f64, sample_rate: f64) -> f64 {
    2.0 * sample_rate * (PI * freq_hz / sample_rate).tan()
}

/// Calculate Butterworth analog prototype poles.
fn butterworth_poles(order: usize) -> Vec<Complex64> {
    let mut poles = Vec::with_capacity(order);
    for k in 0..order {
        let theta = PI * (2 * k + order + 1) as f64 / (2 * order) as f64;
        poles.push(Complex64::new(theta.cos(), theta.sin()));
    }
    poles
}

/// Calculate Chebyshev Type I analog prototype poles.
fn chebyshev1_poles(order: usize, ripple_db: f64) -> Vec<Complex64> {
    let epsilon = (10.0_f64.powf(ripple_db / 10.0) - 1.0).sqrt();
    let a = (1.0 / epsilon + (1.0 / (epsilon * epsilon) + 1.0).sqrt()).ln() / order as f64;

    let mut poles = Vec::with_capacity(order);
    for k in 0..order {
        let theta = PI * (2 * k + 1) as f64 / (2 * order) as f64;
        let real = -a.sinh() * theta.sin();
        let imag = a.cosh() * theta.cos();
        poles.push(Complex64::new(real, imag));
    }
    poles
}

/// Calculate Chebyshev Type II analog prototype poles and zeros.
fn chebyshev2_poles_zeros(order: usize, stopband_db: f64) -> (Vec<Complex64>, Vec<Complex64>) {
    let epsilon = 1.0 / (10.0_f64.powf(stopband_db / 10.0) - 1.0).sqrt();
    let a = (1.0 / epsilon + (1.0 / (epsilon * epsilon) + 1.0).sqrt()).ln() / order as f64;

    let mut poles = Vec::with_capacity(order);
    let mut zeros = Vec::with_capacity(order);

    for k in 0..order {
        let theta = PI * (2 * k + 1) as f64 / (2 * order) as f64;

        // Chebyshev I poles (before inversion)
        let real = -a.sinh() * theta.sin();
        let imag = a.cosh() * theta.cos();
        let p = Complex64::new(real, imag);

        // Invert to get Chebyshev II poles
        poles.push(1.0 / p);

        // Zeros on imaginary axis
        if theta.sin().abs() > 1e-10 {
            zeros.push(Complex64::new(0.0, 1.0 / theta.sin()));
        }
    }

    (poles, zeros)
}

/// Calculate Bessel analog prototype poles using polynomial coefficients.
fn bessel_poles(order: usize) -> Vec<Complex64> {
    // Bessel polynomial roots (normalized)
    // These are pre-computed for orders 1-10
    match order {
        1 => vec![Complex64::new(-1.0, 0.0)],
        2 => vec![
            Complex64::new(-1.1030, 0.6368),
            Complex64::new(-1.1030, -0.6368),
        ],
        3 => vec![
            Complex64::new(-1.0509, 0.9991),
            Complex64::new(-1.0509, -0.9991),
            Complex64::new(-1.3270, 0.0),
        ],
        4 => vec![
            Complex64::new(-0.9953, 1.2571),
            Complex64::new(-0.9953, -1.2571),
            Complex64::new(-1.3707, 0.4103),
            Complex64::new(-1.3707, -0.4103),
        ],
        5 => vec![
            Complex64::new(-0.9576, 1.4711),
            Complex64::new(-0.9576, -1.4711),
            Complex64::new(-1.3806, 0.7179),
            Complex64::new(-1.3806, -0.7179),
            Complex64::new(-1.5025, 0.0),
        ],
        6 => vec![
            Complex64::new(-0.9307, 1.6618),
            Complex64::new(-0.9307, -1.6618),
            Complex64::new(-1.3819, 0.9715),
            Complex64::new(-1.3819, -0.9715),
            Complex64::new(-1.5714, 0.3213),
            Complex64::new(-1.5714, -0.3213),
        ],
        7 => vec![
            Complex64::new(-0.9104, 1.8364),
            Complex64::new(-0.9104, -1.8364),
            Complex64::new(-1.3790, 1.1915),
            Complex64::new(-1.3790, -1.1915),
            Complex64::new(-1.6130, 0.5896),
            Complex64::new(-1.6130, -0.5896),
            Complex64::new(-1.6853, 0.0),
        ],
        8 => vec![
            Complex64::new(-0.8955, 1.9983),
            Complex64::new(-0.8955, -1.9983),
            Complex64::new(-1.3738, 1.3884),
            Complex64::new(-1.3738, -1.3884),
            Complex64::new(-1.6370, 0.8224),
            Complex64::new(-1.6370, -0.8224),
            Complex64::new(-1.7574, 0.2728),
            Complex64::new(-1.7574, -0.2728),
        ],
        9 => vec![
            Complex64::new(-0.8843, 2.1509),
            Complex64::new(-0.8843, -2.1509),
            Complex64::new(-1.3675, 1.5677),
            Complex64::new(-1.3675, -1.5677),
            Complex64::new(-1.6523, 1.0313),
            Complex64::new(-1.6523, -1.0313),
            Complex64::new(-1.8071, 0.5126),
            Complex64::new(-1.8071, -0.5126),
            Complex64::new(-1.8566, 0.0),
        ],
        10 => vec![
            Complex64::new(-0.8758, 2.2962),
            Complex64::new(-0.8758, -2.2962),
            Complex64::new(-1.3607, 1.7335),
            Complex64::new(-1.3607, -1.7335),
            Complex64::new(-1.6618, 1.2211),
            Complex64::new(-1.6618, -1.2211),
            Complex64::new(-1.8431, 0.7273),
            Complex64::new(-1.8431, -0.7273),
            Complex64::new(-1.9302, 0.2413),
            Complex64::new(-1.9302, -0.2413),
        ],
        _ => {
            // Fall back to Butterworth for unsupported orders
            butterworth_poles(order)
        }
    }
}

/// Convert analog prototype poles to digital biquad sections via bilinear transform.
fn poles_to_biquads(
    poles: &[Complex64],
    wc: f64,
    sample_rate: f64,
    filter_type: FilterType,
) -> Vec<Biquad> {
    let k = 2.0 * sample_rate;
    let mut sections = Vec::new();

    // Process poles in conjugate pairs
    let mut i = 0;
    while i < poles.len() {
        if poles[i].im.abs() < 1e-10 {
            // Real pole - create first-order section
            let p = poles[i].re * wc;
            let (b, a) = bilinear_1pole(p, k, filter_type);
            sections.push(Biquad::new(b, a));
            i += 1;
        } else {
            // Complex conjugate pair - create second-order section
            let p = poles[i] * wc;
            let (b, a) = bilinear_2pole(p, k, filter_type);
            sections.push(Biquad::new(b, a));
            i += 2; // Skip conjugate
        }
    }

    sections
}

/// Convert poles and zeros to biquads (for Chebyshev II).
fn poles_zeros_to_biquads(
    poles: &[Complex64],
    zeros: &[Complex64],
    wc: f64,
    sample_rate: f64,
    filter_type: FilterType,
) -> Vec<Biquad> {
    // For simplicity, use poles-only design for now
    // Full implementation would pair poles with zeros
    poles_to_biquads(poles, wc, sample_rate, filter_type)
}

/// Bilinear transform for a single real pole.
fn bilinear_1pole(p: f64, k: f64, filter_type: FilterType) -> ([f64; 3], [f64; 2]) {
    let alpha = k - p;
    let beta = k + p;

    match filter_type {
        FilterType::Lowpass => {
            // H(s) = 1/(s-p) -> H(z)
            let b0 = -p / alpha;
            let b1 = -p / alpha;
            let a1 = -beta / alpha;
            ([b0, b1, 0.0], [a1, 0.0])
        }
        FilterType::Highpass => {
            // s -> 1/s transform then bilinear
            let b0 = k / alpha;
            let b1 = -k / alpha;
            let a1 = -beta / alpha;
            ([b0, b1, 0.0], [a1, 0.0])
        }
        _ => ([1.0, 0.0, 0.0], [0.0, 0.0]),
    }
}

/// Bilinear transform for a complex conjugate pole pair.
fn bilinear_2pole(p: Complex64, k: f64, filter_type: FilterType) -> ([f64; 3], [f64; 2]) {
    let p_re = p.re;
    let p_im = p.im;
    let p_mag_sq = p_re * p_re + p_im * p_im;

    match filter_type {
        FilterType::Lowpass => {
            // Analog: H(s) = omega0^2 / (s^2 - 2*sigma*s + |p|^2)
            // where p = sigma + j*omega
            let k2 = k * k;
            let d = k2 - 2.0 * k * p_re + p_mag_sq;

            let b0 = p_mag_sq / d;
            let b1 = 2.0 * p_mag_sq / d;
            let b2 = p_mag_sq / d;

            let a1 = 2.0 * (p_mag_sq - k2) / d;
            let a2 = (k2 + 2.0 * k * p_re + p_mag_sq) / d;

            ([b0, b1, b2], [a1, a2])
        }
        FilterType::Highpass => {
            // Highpass: substitute s -> omega0^2/s
            let k2 = k * k;
            let d = k2 - 2.0 * k * p_re + p_mag_sq;

            let b0 = k2 / d;
            let b1 = -2.0 * k2 / d;
            let b2 = k2 / d;

            let a1 = 2.0 * (p_mag_sq - k2) / d;
            let a2 = (k2 + 2.0 * k * p_re + p_mag_sq) / d;

            ([b0, b1, b2], [a1, a2])
        }
        _ => ([1.0, 0.0, 0.0], [0.0, 0.0]),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_biquad_creation() {
        let bq = Biquad::new([1.0, 0.0, 0.0], [0.0, 0.0]);
        assert_eq!(bq.numerator(), &[1.0, 0.0, 0.0]);
        assert_eq!(bq.denominator(), &[0.0, 0.0]);
    }

    #[test]
    fn test_biquad_unity() {
        let mut bq = Biquad::unity();
        let output = bq.process_real(1.0);
        assert!((output - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_biquad_stability() {
        let stable = Biquad::new([1.0, 0.0, 0.0], [0.5, 0.2]);
        assert!(stable.is_stable());

        let unstable = Biquad::new([1.0, 0.0, 0.0], [2.0, 0.5]);
        assert!(!unstable.is_stable());
    }

    #[test]
    fn test_butterworth_lowpass() {
        let filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        assert_eq!(filter.order(), 4);
        assert!(filter.is_stable());
        assert_eq!(filter.num_sections(), 2); // 4th order = 2 biquads
    }

    #[test]
    fn test_butterworth_highpass() {
        let filter = IirFilter::butterworth_highpass(4, 1000.0, 8000.0);

        assert_eq!(filter.order(), 4);
        assert!(filter.is_stable());
    }

    #[test]
    fn test_butterworth_bandpass() {
        let filter = IirFilter::butterworth_bandpass(2, 500.0, 1500.0, 8000.0);

        assert!(filter.is_stable());
    }

    #[test]
    fn test_chebyshev1_lowpass() {
        let filter = IirFilter::chebyshev1_lowpass(4, 1.0, 1000.0, 8000.0);

        assert_eq!(filter.order(), 4);
        assert!(filter.is_stable());
    }

    #[test]
    fn test_chebyshev2_lowpass() {
        let filter = IirFilter::chebyshev2_lowpass(4, 40.0, 1500.0, 8000.0);

        assert_eq!(filter.order(), 4);
        assert!(filter.is_stable());
    }

    #[test]
    fn test_bessel_lowpass() {
        let filter = IirFilter::bessel_lowpass(4, 1000.0, 8000.0);

        assert_eq!(filter.order(), 4);
        assert!(filter.is_stable());
    }

    #[test]
    fn test_bessel_highpass() {
        let filter = IirFilter::bessel_highpass(4, 1000.0, 8000.0);

        assert_eq!(filter.order(), 4);
        assert!(filter.is_stable());
    }

    #[test]
    fn test_butterworth_dc_passthrough() {
        let mut filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        // Feed DC signal
        for _ in 0..100 {
            filter.process_real(1.0);
        }
        let output = filter.process_real(1.0);

        // Butterworth LPF should pass DC with unity gain
        assert!(
            (output - 1.0).abs() < 0.01,
            "DC passthrough failed: {}",
            output
        );
    }

    #[test]
    fn test_highpass_blocks_dc() {
        let mut filter = IirFilter::butterworth_highpass(4, 1000.0, 8000.0);

        // Feed DC signal
        for _ in 0..100 {
            filter.process_real(1.0);
        }
        let output = filter.process_real(1.0);

        // HPF should block DC
        assert!(output.abs() < 0.01, "HPF should block DC, got {}", output);
    }

    #[test]
    fn test_filter_reset() {
        let mut filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        // Process some samples
        for _ in 0..50 {
            filter.process_real(1.0);
        }

        // Reset
        filter.reset();

        // First output should be small (no history)
        let output = filter.process_real(1.0);
        assert!(output.abs() < 0.5, "After reset output should be small");
    }

    #[test]
    fn test_frequency_response() {
        let filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        // DC should have ~0 dB
        let dc_db = filter.magnitude_response_db(0.0, 8000.0);
        assert!(dc_db.abs() < 1.0, "DC gain should be ~0 dB, got {}", dc_db);

        // Cutoff should be ~-3 dB
        let cutoff_db = filter.magnitude_response_db(1000.0, 8000.0);
        assert!(
            (cutoff_db + 3.0).abs() < 1.0,
            "Cutoff should be ~-3 dB, got {}",
            cutoff_db
        );

        // High frequency should be attenuated
        let hf_db = filter.magnitude_response_db(3000.0, 8000.0);
        assert!(hf_db < -20.0, "High freq should be attenuated, got {}", hf_db);
    }

    #[test]
    fn test_complex_processing() {
        let mut filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        // Process complex samples
        let input = Complex64::new(1.0, 0.5);
        for _ in 0..100 {
            filter.process(input);
        }
        let output = filter.process(input);

        // Should converge to input (DC passthrough)
        assert!((output.re - 1.0).abs() < 0.01);
        assert!((output.im - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_process_block() {
        let mut filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = filter.process_block(&input);

        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_chebyshev_steeper_rolloff() {
        let butter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);
        let cheby = IirFilter::chebyshev1_lowpass(4, 1.0, 1000.0, 8000.0);

        // At 2x cutoff, Chebyshev should have more attenuation
        let butter_atten = butter.magnitude_response_db(2000.0, 8000.0);
        let cheby_atten = cheby.magnitude_response_db(2000.0, 8000.0);

        assert!(
            cheby_atten < butter_atten,
            "Chebyshev should be steeper: {} vs {}",
            cheby_atten,
            butter_atten
        );
    }

    #[test]
    fn test_various_orders() {
        for order in 1..=8 {
            let filter = IirFilter::butterworth_lowpass(order, 1000.0, 8000.0);
            assert_eq!(filter.order(), order);
            assert!(filter.is_stable());
        }
    }

    #[test]
    fn test_real_filter_trait() {
        let mut filter = IirFilter::butterworth_lowpass(4, 1000.0, 8000.0);

        let input: Vec<f64> = vec![1.0; 100];
        let output = filter.process_real_block(&input);

        assert_eq!(output.len(), 100);
        // Last sample should be close to 1.0 (DC)
        assert!((output[99] - 1.0).abs() < 0.02);
    }
}
