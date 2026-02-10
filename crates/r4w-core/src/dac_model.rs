//! # Digital-to-Analog Converter Behavioral Model
//!
//! Simulates the behavior of a real DAC including quantization, differential
//! and integral nonlinearity (DNL/INL), clock jitter, and zero-order hold
//! reconstruction. Useful for modeling converter impairments in SDR transmit
//! chains before RF up-conversion.
//!
//! ## Features
//!
//! - Configurable resolution (8 to 16 bits)
//! - DNL and INL impairment injection
//! - Clock jitter model (RMS picoseconds)
//! - Zero-order hold (ZOH) output reconstruction
//! - Ideal SNR and SFDR estimation
//! - Builder-pattern configuration via [`DacConfig`]
//! - Factory constructors for common DAC resolutions
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dac_model::{DacModel, ZeroOrderHold};
//!
//! // Create a 14-bit DAC
//! let dac = DacModel::new(14);
//! assert_eq!(dac.resolution_bits, 14);
//!
//! // Quantize a mid-scale sample
//! let out = dac.quantize(0.5);
//! assert!((out - 0.5).abs() < 0.001);
//!
//! // Ideal SNR for 14 bits: 6.02*14 + 1.76 = 86.04 dB
//! let snr = dac.snr_ideal();
//! assert!((snr - 86.04).abs() < 0.01);
//!
//! // Zero-order hold reconstruction
//! let zoh = ZeroOrderHold::new(4);
//! let held = zoh.process(&[1.0, -1.0]);
//! assert_eq!(held.len(), 8);
//! ```

/// Digital-to-Analog Converter behavioral model.
///
/// Models quantization to `2^resolution_bits` levels within the range
/// `[-full_scale_voltage, +full_scale_voltage]`, with optional DNL, INL,
/// and clock jitter impairments.
#[derive(Debug, Clone)]
pub struct DacModel {
    /// Number of quantization bits (e.g. 8, 12, 14, 16).
    pub resolution_bits: u8,
    /// Peak output voltage (bipolar: -Vfs to +Vfs).
    pub full_scale_voltage: f64,
    /// RMS clock jitter in picoseconds.
    pub clock_jitter_rms_ps: f64,
    /// Differential nonlinearity in LSBs.
    pub dnl_lsb: f64,
    /// Integral nonlinearity in LSBs.
    pub inl_lsb: f64,
}

impl DacModel {
    /// Create a DAC model with the given resolution and sensible defaults.
    ///
    /// Defaults: full_scale_voltage = 1.0, no jitter, DNL = 0.0, INL = 0.0.
    pub fn new(resolution_bits: u8) -> Self {
        Self {
            resolution_bits,
            full_scale_voltage: 1.0,
            clock_jitter_rms_ps: 0.0,
            dnl_lsb: 0.0,
            inl_lsb: 0.0,
        }
    }

    /// Pure quantization without jitter or nonlinearity impairments.
    ///
    /// Clips the input to `[-full_scale_voltage, +full_scale_voltage]` then
    /// rounds to the nearest DAC level.
    pub fn quantize(&self, sample: f64) -> f64 {
        let levels = (1u64 << self.resolution_bits) as f64;
        let vfs = self.full_scale_voltage;

        // Clip to full-scale range
        let clamped = sample.clamp(-vfs, vfs);

        // Normalize to [0, 1]
        let normalized = (clamped + vfs) / (2.0 * vfs);

        // Quantize to integer level and convert back
        let level = (normalized * (levels - 1.0)).round();
        let quantized_normalized = level / (levels - 1.0);

        // Map back to [-vfs, +vfs]
        quantized_normalized * 2.0 * vfs - vfs
    }

    /// Convert a single sample through the DAC model.
    ///
    /// Applies quantization, then adds INL and DNL impairments.
    /// Clock jitter is modeled as a deterministic worst-case amplitude
    /// perturbation proportional to `clock_jitter_rms_ps`.
    pub fn convert(&self, digital: f64) -> f64 {
        let quantized = self.quantize(digital);

        let levels = (1u64 << self.resolution_bits) as f64;
        let lsb_voltage = (2.0 * self.full_scale_voltage) / levels;

        // Apply INL as a static offset proportional to the code position.
        // Model INL as a sinusoidal bow across the transfer function.
        let vfs = self.full_scale_voltage;
        let normalized_pos = (quantized + vfs) / (2.0 * vfs);
        let inl_contribution = self.inl_lsb
            * lsb_voltage
            * (std::f64::consts::PI * normalized_pos).sin();

        // DNL modeled as a small-scale perturbation at each code transition.
        // Use a simple deterministic model based on the quantized level.
        let level_index = ((quantized + vfs) / lsb_voltage).round();
        let dnl_contribution = self.dnl_lsb
            * lsb_voltage
            * ((level_index * 2.0 * std::f64::consts::PI / 7.0).sin() * 0.5);

        // Clock jitter modeled as amplitude uncertainty.
        // For a sinusoidal signal at frequency f, jitter-induced SNR limit is
        // -20*log10(2*pi*f*tj). Here we add a deterministic perturbation
        // proportional to the jitter magnitude for reproducibility.
        let jitter_contribution = if self.clock_jitter_rms_ps > 0.0 {
            let tj_seconds = self.clock_jitter_rms_ps * 1e-12;
            // Scale by signal slope estimate (use quantized value as proxy)
            tj_seconds * quantized * 1e6 // Tiny contribution for ps-level jitter
        } else {
            0.0
        };

        quantized + inl_contribution + dnl_contribution + jitter_contribution
    }

    /// Convert a batch of samples through the DAC model.
    pub fn convert_batch(&self, samples: &[f64]) -> Vec<f64> {
        samples.iter().map(|&s| self.convert(s)).collect()
    }

    /// Set the RMS clock jitter in picoseconds.
    pub fn set_jitter(&mut self, rms_ps: f64) {
        self.clock_jitter_rms_ps = rms_ps;
    }

    /// Theoretical ideal SNR for an N-bit DAC.
    ///
    /// Formula: `SNR = 6.02 * N + 1.76` dB, assuming a full-scale sinusoidal
    /// input and uniformly distributed quantization noise.
    pub fn snr_ideal(&self) -> f64 {
        6.02 * self.resolution_bits as f64 + 1.76
    }

    /// Estimate spurious-free dynamic range (SFDR) from INL.
    ///
    /// A common approximation is that the worst-case spur level due to
    /// integral nonlinearity is roughly `INL / 2^(N-1)` of full scale,
    /// giving SFDR in dB as approximately `20 * log10(2^(N-1) / INL)`.
    ///
    /// When INL is zero, returns the ideal SNR as a conservative upper bound.
    pub fn sfdr_estimate(&self) -> f64 {
        if self.inl_lsb <= 0.0 {
            self.snr_ideal()
        } else {
            let half_scale = (1u64 << (self.resolution_bits - 1)) as f64;
            20.0 * (half_scale / self.inl_lsb).log10()
        }
    }
}

/// Zero-order hold (ZOH) reconstruction filter.
///
/// Repeats each input sample for `oversampling` output samples, modeling
/// the sample-and-hold behavior of a real DAC output stage.
#[derive(Debug, Clone)]
pub struct ZeroOrderHold {
    /// Number of times each sample is repeated.
    pub oversampling: usize,
}

impl ZeroOrderHold {
    /// Create a new ZOH with the given oversampling factor.
    ///
    /// # Panics
    ///
    /// Panics if `oversampling` is 0.
    pub fn new(oversampling: usize) -> Self {
        assert!(oversampling > 0, "Oversampling factor must be at least 1");
        Self { oversampling }
    }

    /// Apply zero-order hold to a slice of samples.
    ///
    /// Each input sample is held (repeated) for `oversampling` output samples.
    /// Output length is `samples.len() * oversampling`.
    pub fn process(&self, samples: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(samples.len() * self.oversampling);
        for &s in samples {
            for _ in 0..self.oversampling {
                output.push(s);
            }
        }
        output
    }
}

/// Builder for constructing a [`DacModel`] with custom parameters.
///
/// # Example
///
/// ```rust
/// use r4w_core::dac_model::DacConfig;
///
/// let dac = DacConfig::new(14)
///     .full_scale_voltage(0.7)
///     .dnl(0.3)
///     .inl(0.8)
///     .jitter_rms_ps(1.5)
///     .build();
///
/// assert_eq!(dac.resolution_bits, 14);
/// assert!((dac.full_scale_voltage - 0.7).abs() < 1e-12);
/// ```
#[derive(Debug, Clone)]
pub struct DacConfig {
    resolution_bits: u8,
    full_scale_voltage: f64,
    clock_jitter_rms_ps: f64,
    dnl_lsb: f64,
    inl_lsb: f64,
}

impl DacConfig {
    /// Start building a DAC configuration with the given resolution.
    pub fn new(resolution_bits: u8) -> Self {
        Self {
            resolution_bits,
            full_scale_voltage: 1.0,
            clock_jitter_rms_ps: 0.0,
            dnl_lsb: 0.0,
            inl_lsb: 0.0,
        }
    }

    /// Set the full-scale output voltage.
    pub fn full_scale_voltage(mut self, v: f64) -> Self {
        self.full_scale_voltage = v;
        self
    }

    /// Set the RMS clock jitter in picoseconds.
    pub fn jitter_rms_ps(mut self, ps: f64) -> Self {
        self.clock_jitter_rms_ps = ps;
        self
    }

    /// Set the differential nonlinearity in LSBs.
    pub fn dnl(mut self, lsb: f64) -> Self {
        self.dnl_lsb = lsb;
        self
    }

    /// Set the integral nonlinearity in LSBs.
    pub fn inl(mut self, lsb: f64) -> Self {
        self.inl_lsb = lsb;
        self
    }

    /// Build the [`DacModel`] from this configuration.
    pub fn build(self) -> DacModel {
        DacModel {
            resolution_bits: self.resolution_bits,
            full_scale_voltage: self.full_scale_voltage,
            clock_jitter_rms_ps: self.clock_jitter_rms_ps,
            dnl_lsb: self.dnl_lsb,
            inl_lsb: self.inl_lsb,
        }
    }
}

/// Create a 14-bit DAC model with typical impairments.
///
/// - Resolution: 14 bits
/// - Full-scale voltage: 1.0 V
/// - DNL: 0.5 LSB
/// - INL: 1.0 LSB
/// - Clock jitter: 0.5 ps RMS
pub fn dac_14bit() -> DacModel {
    DacConfig::new(14)
        .full_scale_voltage(1.0)
        .dnl(0.5)
        .inl(1.0)
        .jitter_rms_ps(0.5)
        .build()
}

/// Create a 16-bit DAC model with typical impairments.
///
/// - Resolution: 16 bits
/// - Full-scale voltage: 1.0 V
/// - DNL: 0.5 LSB
/// - INL: 2.0 LSB
/// - Clock jitter: 0.3 ps RMS
pub fn dac_16bit() -> DacModel {
    DacConfig::new(16)
        .full_scale_voltage(1.0)
        .dnl(0.5)
        .inl(2.0)
        .jitter_rms_ps(0.3)
        .build()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction_defaults() {
        let dac = DacModel::new(12);
        assert_eq!(dac.resolution_bits, 12);
        assert!((dac.full_scale_voltage - 1.0).abs() < f64::EPSILON);
        assert!((dac.clock_jitter_rms_ps - 0.0).abs() < f64::EPSILON);
        assert!((dac.dnl_lsb - 0.0).abs() < f64::EPSILON);
        assert!((dac.inl_lsb - 0.0).abs() < f64::EPSILON);
    }

    #[test]
    fn test_quantization_rounds_correctly() {
        let dac = DacModel::new(8);
        // 8-bit DAC: 256 levels over [-1, +1], LSB = 2/256 ≈ 0.00784
        let lsb = 2.0 / 255.0;

        // Mid-scale (0.0) should quantize exactly to 0.0
        // Level 127.5 rounds to 128, which maps to 128/255 * 2 - 1 = 0.00392...
        // Actually 0.0 normalized = 0.5, level = 0.5 * 255 = 127.5 rounds to 128
        let q = dac.quantize(0.0);
        // 128/255 * 2 - 1 = 256/255 - 1 = 1/255
        assert!((q - 1.0 / 255.0).abs() < lsb);

        // Full-scale positive
        let q_pos = dac.quantize(1.0);
        assert!((q_pos - 1.0).abs() < f64::EPSILON);

        // Full-scale negative
        let q_neg = dac.quantize(-1.0);
        assert!((q_neg - (-1.0)).abs() < f64::EPSILON);
    }

    #[test]
    fn test_full_scale_clipping() {
        let dac = DacModel::new(12);
        // Values beyond full scale should clip
        let above = dac.quantize(2.0);
        assert!((above - 1.0).abs() < f64::EPSILON, "Above full scale should clip to +Vfs");

        let below = dac.quantize(-3.0);
        assert!((below - (-1.0)).abs() < f64::EPSILON, "Below full scale should clip to -Vfs");
    }

    #[test]
    fn test_snr_ideal_formula() {
        // SNR = 6.02*N + 1.76
        let dac8 = DacModel::new(8);
        assert!((dac8.snr_ideal() - 49.92).abs() < 0.01);

        let dac12 = DacModel::new(12);
        assert!((dac12.snr_ideal() - 74.0).abs() < 0.01);

        let dac16 = DacModel::new(16);
        assert!((dac16.snr_ideal() - 98.08).abs() < 0.01);
    }

    #[test]
    fn test_sfdr_estimate() {
        // With zero INL, SFDR should equal ideal SNR
        let dac = DacModel::new(14);
        assert!((dac.sfdr_estimate() - dac.snr_ideal()).abs() < f64::EPSILON);

        // With INL = 1.0 LSB, SFDR ≈ 20*log10(2^13 / 1.0) = 20*log10(8192) ≈ 78.26 dB
        let mut dac_inl = DacModel::new(14);
        dac_inl.inl_lsb = 1.0;
        let sfdr = dac_inl.sfdr_estimate();
        let expected = 20.0 * (8192.0_f64).log10();
        assert!((sfdr - expected).abs() < 0.01);
    }

    #[test]
    fn test_zoh_output_length() {
        let zoh = ZeroOrderHold::new(4);
        let input = vec![1.0, 2.0, 3.0];
        let output = zoh.process(&input);
        assert_eq!(output.len(), 12); // 3 * 4

        let zoh1 = ZeroOrderHold::new(1);
        let out1 = zoh1.process(&input);
        assert_eq!(out1.len(), 3); // 3 * 1

        // Empty input
        let empty = zoh.process(&[]);
        assert_eq!(empty.len(), 0);
    }

    #[test]
    fn test_zoh_waveform_shape() {
        let zoh = ZeroOrderHold::new(3);
        let input = vec![1.0, -1.0, 0.5];
        let output = zoh.process(&input);

        // First sample held 3 times
        assert!((output[0] - 1.0).abs() < f64::EPSILON);
        assert!((output[1] - 1.0).abs() < f64::EPSILON);
        assert!((output[2] - 1.0).abs() < f64::EPSILON);

        // Second sample held 3 times
        assert!((output[3] - (-1.0)).abs() < f64::EPSILON);
        assert!((output[4] - (-1.0)).abs() < f64::EPSILON);
        assert!((output[5] - (-1.0)).abs() < f64::EPSILON);

        // Third sample held 3 times
        assert!((output[6] - 0.5).abs() < f64::EPSILON);
        assert!((output[7] - 0.5).abs() < f64::EPSILON);
        assert!((output[8] - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn test_factories_14bit_and_16bit() {
        // 14-bit factory
        let dac14 = dac_14bit();
        assert_eq!(dac14.resolution_bits, 14);
        assert!((dac14.full_scale_voltage - 1.0).abs() < f64::EPSILON);
        assert!((dac14.dnl_lsb - 0.5).abs() < f64::EPSILON);
        assert!((dac14.inl_lsb - 1.0).abs() < f64::EPSILON);
        assert!((dac14.clock_jitter_rms_ps - 0.5).abs() < f64::EPSILON);

        // 16-bit factory
        let dac16 = dac_16bit();
        assert_eq!(dac16.resolution_bits, 16);
        assert!((dac16.full_scale_voltage - 1.0).abs() < f64::EPSILON);
        assert!((dac16.dnl_lsb - 0.5).abs() < f64::EPSILON);
        assert!((dac16.inl_lsb - 2.0).abs() < f64::EPSILON);
        assert!((dac16.clock_jitter_rms_ps - 0.3).abs() < f64::EPSILON);
    }

    #[test]
    fn test_batch_conversion_length() {
        let dac = DacModel::new(12);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 / 50.0) - 1.0).collect();
        let output = dac.convert_batch(&input);
        assert_eq!(output.len(), input.len());

        // Each output should be within full-scale range (with small impairment additions)
        for &v in &output {
            assert!(
                v >= -1.1 && v <= 1.1,
                "Output {} outside expected range",
                v
            );
        }
    }

    #[test]
    fn test_config_builder() {
        let dac = DacConfig::new(12)
            .full_scale_voltage(0.5)
            .dnl(0.25)
            .inl(0.75)
            .jitter_rms_ps(2.0)
            .build();

        assert_eq!(dac.resolution_bits, 12);
        assert!((dac.full_scale_voltage - 0.5).abs() < f64::EPSILON);
        assert!((dac.dnl_lsb - 0.25).abs() < f64::EPSILON);
        assert!((dac.inl_lsb - 0.75).abs() < f64::EPSILON);
        assert!((dac.clock_jitter_rms_ps - 2.0).abs() < f64::EPSILON);

        // Verify it produces valid output
        let q = dac.quantize(0.25);
        assert!(q >= -0.5 && q <= 0.5);
    }
}
