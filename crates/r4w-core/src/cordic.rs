//! CORDIC — COordinate Rotation DIgital Computer
//!
//! Hardware-efficient iterative algorithm for computing trigonometric
//! functions, vector rotations, magnitude/phase conversions, and
//! atan2 using only shifts and additions. Essential building block
//! for FPGA-targeted and fixed-point DSP implementations.
//! GNU Radio equivalent: `gr::blocks::rotator_cc` (uses CORDIC internally).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cordic::{Cordic, cordic_sincos};
//!
//! let (sin_val, cos_val) = cordic_sincos(std::f64::consts::FRAC_PI_4, 20);
//! assert!((sin_val - 0.7071).abs() < 0.001);
//! assert!((cos_val - 0.7071).abs() < 0.001);
//!
//! let cordic = Cordic::new(20);
//! let (mag, phase) = cordic.to_polar(3.0, 4.0);
//! assert!((mag - 5.0).abs() < 0.01);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Precomputed CORDIC arctangent table for up to 32 iterations.
const CORDIC_ATAN_TABLE: [f64; 32] = [
    0.7853981633974483,  // atan(2^0)
    0.4636476090008061,  // atan(2^-1)
    0.24497866312686414, // atan(2^-2)
    0.12435499454676144, // atan(2^-3)
    0.06241880999595735, // atan(2^-4)
    0.031239833430268277,
    0.015623728620476831,
    0.007812341060101111,
    0.0039062301319669718,
    0.0019531225164788188,
    0.0009765621895593195,
    0.0004882812111948983,
    0.00024414062014936177,
    0.00012207031189367021,
    0.00006103515617420877,
    0.000030517578115526096,
    0.000015258789061315762,
    7.62939453110197e-6,
    3.8146972656064963e-6,
    1.9073486328101870e-6,
    9.536743164059608e-7,
    4.768371582030888e-7,
    2.384185791015580e-7,
    1.192092895507806e-7,
    5.960464477539055e-8,
    2.980232238769530e-8,
    1.490116119384766e-8,
    7.450580596923828e-9,
    3.725290298461914e-9,
    1.862645149230957e-9,
    9.313225746154785e-10,
    4.656612873077393e-10,
];

/// Precomputed CORDIC gain for n iterations: product of 1/sqrt(1 + 2^(-2i)).
fn cordic_gain(iterations: usize) -> f64 {
    let mut gain = 1.0;
    for i in 0..iterations.min(32) {
        gain *= 1.0 / (1.0 + (2.0_f64).powi(-(2 * i as i32))).sqrt();
    }
    gain
}

/// Compute sin and cos using CORDIC rotation mode.
///
/// `angle`: angle in radians.
/// `iterations`: number of CORDIC iterations (more = higher precision, typ 16-24).
pub fn cordic_sincos(angle: f64, iterations: usize) -> (f64, f64) {
    let n = iterations.min(32);

    // Reduce angle to [-pi, pi]
    let mut theta = angle % (2.0 * PI);
    if theta > PI {
        theta -= 2.0 * PI;
    } else if theta < -PI {
        theta += 2.0 * PI;
    }

    // Handle quadrants: reduce to [-pi/2, pi/2]
    let mut x;
    let mut y;
    if theta > PI / 2.0 {
        theta -= PI;
        x = -cordic_gain(n);
        y = 0.0;
    } else if theta < -PI / 2.0 {
        theta += PI;
        x = -cordic_gain(n);
        y = 0.0;
    } else {
        x = cordic_gain(n);
        y = 0.0;
    }

    // Rotation mode: drive angle to zero
    let mut z = theta;
    for i in 0..n {
        let d = if z >= 0.0 { 1.0 } else { -1.0 };
        let shift = 2.0_f64.powi(-(i as i32));
        let x_new = x - d * y * shift;
        let y_new = y + d * x * shift;
        z -= d * CORDIC_ATAN_TABLE[i];
        x = x_new;
        y = y_new;
    }

    (y, x) // (sin, cos)
}

/// Compute atan2(y, x) using CORDIC vectoring mode.
pub fn cordic_atan2(y: f64, x: f64, iterations: usize) -> f64 {
    let n = iterations.min(32);

    // Handle special cases
    if x == 0.0 && y == 0.0 {
        return 0.0;
    }

    let mut xv = x;
    let mut yv = y;
    let mut z = 0.0;

    // Pre-rotate to first/fourth quadrant
    let mut angle_offset = 0.0;
    if xv < 0.0 {
        if yv >= 0.0 {
            angle_offset = PI;
            xv = -xv;
            yv = -yv;
        } else {
            angle_offset = -PI;
            xv = -xv;
            yv = -yv;
        }
    }

    // Vectoring mode: drive y to zero
    for i in 0..n {
        let d = if yv < 0.0 { 1.0 } else { -1.0 };
        let shift = 2.0_f64.powi(-(i as i32));
        let x_new = xv - d * yv * shift;
        let y_new = yv + d * xv * shift;
        z -= d * CORDIC_ATAN_TABLE[i];
        xv = x_new;
        yv = y_new;
    }

    z + angle_offset
}

/// CORDIC processor with cached gain.
#[derive(Debug, Clone)]
pub struct Cordic {
    iterations: usize,
    gain: f64,
}

impl Cordic {
    /// Create a new CORDIC processor.
    pub fn new(iterations: usize) -> Self {
        let n = iterations.min(32);
        Self {
            iterations: n,
            gain: cordic_gain(n),
        }
    }

    /// Compute (sin, cos) of an angle.
    pub fn sincos(&self, angle: f64) -> (f64, f64) {
        cordic_sincos(angle, self.iterations)
    }

    /// Convert rectangular to polar coordinates.
    pub fn to_polar(&self, x: f64, y: f64) -> (f64, f64) {
        let phase = cordic_atan2(y, x, self.iterations);
        let mag = (x * x + y * y).sqrt(); // Use exact for magnitude
        (mag, phase)
    }

    /// Convert polar to rectangular coordinates.
    pub fn to_rect(&self, mag: f64, phase: f64) -> (f64, f64) {
        let (sin, cos) = self.sincos(phase);
        (mag * cos, mag * sin)
    }

    /// Rotate a complex number by an angle.
    pub fn rotate(&self, z: Complex64, angle: f64) -> Complex64 {
        let (sin, cos) = self.sincos(angle);
        Complex64::new(
            z.re * cos - z.im * sin,
            z.re * sin + z.im * cos,
        )
    }

    /// Rotate a block of complex samples by linearly incrementing phase.
    pub fn rotate_block(
        &self,
        input: &[Complex64],
        start_phase: f64,
        phase_inc: f64,
    ) -> Vec<Complex64> {
        input
            .iter()
            .enumerate()
            .map(|(i, &z)| {
                let angle = start_phase + i as f64 * phase_inc;
                self.rotate(z, angle)
            })
            .collect()
    }

    /// Get the CORDIC gain factor.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Get number of iterations.
    pub fn iterations(&self) -> usize {
        self.iterations
    }
}

/// NCO (Numerically Controlled Oscillator) using CORDIC.
#[derive(Debug, Clone)]
pub struct CordicNco {
    cordic: Cordic,
    phase: f64,
    phase_inc: f64,
}

impl CordicNco {
    /// Create a CORDIC-based NCO.
    pub fn new(frequency: f64, sample_rate: f64, iterations: usize) -> Self {
        Self {
            cordic: Cordic::new(iterations),
            phase: 0.0,
            phase_inc: 2.0 * PI * frequency / sample_rate,
        }
    }

    /// Generate next complex sample.
    pub fn next_sample(&mut self) -> Complex64 {
        let (sin, cos) = self.cordic.sincos(self.phase);
        self.phase += self.phase_inc;
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        }
        Complex64::new(cos, sin)
    }

    /// Generate a block of samples.
    pub fn generate(&mut self, num_samples: usize) -> Vec<Complex64> {
        (0..num_samples).map(|_| self.next_sample()).collect()
    }

    /// Set frequency.
    pub fn set_frequency(&mut self, frequency: f64, sample_rate: f64) {
        self.phase_inc = 2.0 * PI * frequency / sample_rate;
    }

    /// Reset phase.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    #[test]
    fn test_sincos_zero() {
        let (sin, cos) = cordic_sincos(0.0, 20);
        assert!(sin.abs() < 0.001, "sin(0) = {}", sin);
        assert!((cos - 1.0).abs() < 0.001, "cos(0) = {}", cos);
    }

    #[test]
    fn test_sincos_pi_over_4() {
        let (sin, cos) = cordic_sincos(FRAC_PI_4, 20);
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        assert!((sin - expected).abs() < 0.001, "sin(pi/4) = {}", sin);
        assert!((cos - expected).abs() < 0.001, "cos(pi/4) = {}", cos);
    }

    #[test]
    fn test_sincos_pi_over_2() {
        let (sin, cos) = cordic_sincos(FRAC_PI_2, 20);
        assert!((sin - 1.0).abs() < 0.001, "sin(pi/2) = {}", sin);
        assert!(cos.abs() < 0.001, "cos(pi/2) = {}", cos);
    }

    #[test]
    fn test_sincos_negative() {
        let (sin, cos) = cordic_sincos(-FRAC_PI_4, 20);
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        assert!((sin + expected).abs() < 0.001);
        assert!((cos - expected).abs() < 0.001);
    }

    #[test]
    fn test_atan2_basic() {
        let angle = cordic_atan2(1.0, 1.0, 20);
        assert!((angle - FRAC_PI_4).abs() < 0.001);

        let angle2 = cordic_atan2(1.0, 0.0, 20);
        assert!((angle2 - FRAC_PI_2).abs() < 0.001);
    }

    #[test]
    fn test_atan2_negative() {
        let angle = cordic_atan2(-1.0, -1.0, 20);
        assert!((angle + 3.0 * FRAC_PI_4).abs() < 0.01);
    }

    #[test]
    fn test_to_polar() {
        let cordic = Cordic::new(20);
        let (mag, phase) = cordic.to_polar(3.0, 4.0);
        assert!((mag - 5.0).abs() < 0.01);
        assert!((phase - (4.0_f64).atan2(3.0)).abs() < 0.01);
    }

    #[test]
    fn test_to_rect() {
        let cordic = Cordic::new(20);
        let (x, y) = cordic.to_rect(5.0, FRAC_PI_4);
        let expected = 5.0 * std::f64::consts::FRAC_1_SQRT_2;
        assert!((x - expected).abs() < 0.01);
        assert!((y - expected).abs() < 0.01);
    }

    #[test]
    fn test_rotate_complex() {
        let cordic = Cordic::new(20);
        let z = Complex64::new(1.0, 0.0);
        let rotated = cordic.rotate(z, FRAC_PI_2);
        assert!(rotated.re.abs() < 0.001);
        assert!((rotated.im - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_nco_frequency() {
        let mut nco = CordicNco::new(1000.0, 8000.0, 20);
        let samples = nco.generate(8);
        assert_eq!(samples.len(), 8);
        // At 1000 Hz / 8000 Hz = 1/8 cycle per sample
        // After 8 samples, should complete one full cycle
        // First sample: cos(0) + j*sin(0) = 1 + j*0
        assert!((samples[0].re - 1.0).abs() < 0.01);
        assert!(samples[0].im.abs() < 0.01);
    }

    #[test]
    fn test_cordic_precision_increases_with_iterations() {
        let exact_sin = (1.0_f64).sin();
        let (sin_10, _) = cordic_sincos(1.0, 10);
        let (sin_20, _) = cordic_sincos(1.0, 20);
        assert!(
            (sin_20 - exact_sin).abs() <= (sin_10 - exact_sin).abs() + 1e-10,
            "More iterations should give better precision"
        );
    }

    #[test]
    fn test_gain_factor() {
        let cordic = Cordic::new(20);
        // CORDIC gain converges to ~0.6073
        assert!((cordic.gain() - 0.6073).abs() < 0.001);
    }
}
