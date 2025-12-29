//! CVSD (Continuously Variable Slope Delta modulation) voice codec
//!
//! SINCGARS uses CVSD for voice digitization at 16 kbps.
//! This is unclassified and publicly documented.

/// CVSD codec parameters
#[derive(Debug, Clone)]
pub struct CvsdConfig {
    /// Sample rate in Hz (typically 8000)
    pub sample_rate: u32,
    /// Bit rate (16000 for SINCGARS)
    pub bit_rate: u32,
    /// Step size adaptation algorithm
    pub adaptation: CvsdAdaptation,
}

impl Default for CvsdConfig {
    fn default() -> Self {
        Self {
            sample_rate: 8000,
            bit_rate: 16000,
            adaptation: CvsdAdaptation::MilStd188_113,
        }
    }
}

/// CVSD step size adaptation algorithm
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CvsdAdaptation {
    /// MIL-STD-188-113 algorithm
    MilStd188_113,
    /// Simple syllabic companding
    Syllabic,
}

/// CVSD encoder/decoder
#[derive(Debug)]
pub struct CvsdCodec {
    config: CvsdConfig,
    /// Integrator value (reconstructed signal)
    integrator: f64,
    /// Current step size
    step_size: f64,
    /// Minimum step size
    step_min: f64,
    /// Maximum step size
    step_max: f64,
    /// Step size increase factor
    step_up: f64,
    /// Step size decrease factor
    step_down: f64,
    /// History for run detection
    history: u8,
    /// Number of consecutive same bits for adaptation
    run_length: u8,
}

impl CvsdCodec {
    /// Create new CVSD codec with given configuration
    pub fn new(config: CvsdConfig) -> Self {
        Self {
            config,
            integrator: 0.0,
            step_size: 0.01,
            step_min: 0.001,
            step_max: 0.5,
            step_up: 1.5,
            step_down: 0.9,
            history: 0,
            run_length: 3, // 3 consecutive same bits triggers adaptation
        }
    }

    /// Create codec with SINCGARS-compatible settings
    pub fn sincgars() -> Self {
        Self::new(CvsdConfig::default())
    }

    /// Reset codec state
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.step_size = 0.01;
        self.history = 0;
    }

    /// Encode audio samples to CVSD bits
    ///
    /// # Arguments
    ///
    /// * `samples` - Audio samples normalized to [-1.0, 1.0]
    ///
    /// # Returns
    ///
    /// CVSD encoded bits (packed into bytes, MSB first)
    pub fn encode(&mut self, samples: &[f32]) -> Vec<u8> {
        let samples_per_bit = self.config.sample_rate as usize / self.config.bit_rate as usize;
        let samples_per_bit = samples_per_bit.max(1);
        let num_bits = samples.len() / samples_per_bit;
        let num_bytes = (num_bits + 7) / 8;

        let mut output = vec![0u8; num_bytes];
        let mut bit_index = 0;

        for chunk in samples.chunks(samples_per_bit) {
            // Average the samples in this bit period
            let avg: f32 = chunk.iter().sum::<f32>() / chunk.len() as f32;
            let avg = avg as f64;

            // Compare to integrator - if input > integrator, output 1
            let bit = if avg > self.integrator { 1u8 } else { 0u8 };

            // Update integrator based on bit
            if bit == 1 {
                self.integrator += self.step_size;
            } else {
                self.integrator -= self.step_size;
            }

            // Limit integrator to prevent overflow
            self.integrator = self.integrator.clamp(-1.0, 1.0);

            // Adapt step size based on run detection
            self.adapt_step_size(bit);

            // Pack bit into output byte
            let byte_idx = bit_index / 8;
            let bit_pos = 7 - (bit_index % 8);
            if byte_idx < output.len() {
                output[byte_idx] |= bit << bit_pos;
            }

            bit_index += 1;
        }

        output
    }

    /// Decode CVSD bits to audio samples
    ///
    /// # Arguments
    ///
    /// * `bits` - CVSD encoded bits (packed in bytes, MSB first)
    ///
    /// # Returns
    ///
    /// Decoded audio samples normalized to [-1.0, 1.0]
    pub fn decode(&mut self, bits: &[u8]) -> Vec<f32> {
        let samples_per_bit = self.config.sample_rate as usize / self.config.bit_rate as usize;
        let samples_per_bit = samples_per_bit.max(1);
        let num_bits = bits.len() * 8;

        let mut output = Vec::with_capacity(num_bits * samples_per_bit);

        for byte_idx in 0..bits.len() {
            for bit_pos in (0..8).rev() {
                let bit = (bits[byte_idx] >> bit_pos) & 1;

                // Update integrator based on bit
                if bit == 1 {
                    self.integrator += self.step_size;
                } else {
                    self.integrator -= self.step_size;
                }

                // Limit integrator
                self.integrator = self.integrator.clamp(-1.0, 1.0);

                // Adapt step size
                self.adapt_step_size(bit);

                // Output samples for this bit period
                for _ in 0..samples_per_bit {
                    output.push(self.integrator as f32);
                }
            }
        }

        output
    }

    /// Adapt step size based on run detection
    fn adapt_step_size(&mut self, bit: u8) {
        // Shift history and add new bit
        self.history = (self.history << 1) | bit;

        // Check for runs (all 1s or all 0s in last N bits)
        let mask = (1u8 << self.run_length) - 1;
        let recent = self.history & mask;

        if recent == 0 || recent == mask {
            // Run detected - increase step size (slope overload)
            self.step_size = (self.step_size * self.step_up).min(self.step_max);
        } else {
            // No run - decrease step size (granular noise region)
            self.step_size = (self.step_size * self.step_down).max(self.step_min);
        }
    }

    /// Get current configuration
    pub fn config(&self) -> &CvsdConfig {
        &self.config
    }
}

/// LPC-10e vocoder stub
///
/// SINCGARS ICOM mode uses LPC-10e for 2.4 kbps voice.
/// This is a placeholder for future implementation.
#[allow(dead_code)]
#[derive(Debug)]
pub struct Lpc10eCodec {
    // LPC-10e parameters would go here
}

#[allow(dead_code)]
impl Lpc10eCodec {
    pub fn new() -> Self {
        Self {}
    }

    /// Encode voice frame (placeholder)
    pub fn encode_frame(&mut self, _samples: &[f32]) -> Vec<u8> {
        // LPC-10e encoding not yet implemented
        vec![0; 7] // 54 bits per 22.5ms frame ≈ 7 bytes
    }

    /// Decode voice frame (placeholder)
    pub fn decode_frame(&mut self, _bits: &[u8]) -> Vec<f32> {
        // LPC-10e decoding not yet implemented
        vec![0.0; 180] // 180 samples at 8kHz for 22.5ms
    }
}

impl Default for Lpc10eCodec {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cvsd_roundtrip() {
        let mut encoder = CvsdCodec::sincgars();
        let mut decoder = CvsdCodec::sincgars();

        // Generate a simple sine wave
        let freq = 440.0;
        let sample_rate = 8000.0;
        let samples: Vec<f32> = (0..800)
            .map(|i| {
                let t = i as f32 / sample_rate;
                (2.0 * std::f32::consts::PI * freq * t).sin() * 0.5
            })
            .collect();

        let encoded = encoder.encode(&samples);
        let decoded = decoder.decode(&encoded);

        // Check that we got approximately the same number of samples
        // (may differ slightly due to bit packing)
        assert!(decoded.len() >= samples.len() - 100);

        // CVSD is lossy, but the decoded signal should have
        // the same general characteristics
        let decoded_avg: f32 = decoded.iter().map(|x| x.abs()).sum::<f32>() / decoded.len() as f32;
        let input_avg: f32 = samples.iter().map(|x| x.abs()).sum::<f32>() / samples.len() as f32;

        // Average magnitude should be in the same ballpark
        assert!((decoded_avg - input_avg).abs() < 0.2);
    }

    #[test]
    fn test_cvsd_silent_input() {
        let mut codec = CvsdCodec::sincgars();

        let silence = vec![0.0f32; 160];
        let encoded = codec.encode(&silence);
        codec.reset();
        let decoded = codec.decode(&encoded);

        // Silent input should produce near-silent output
        let max_amplitude = decoded.iter().map(|x| x.abs()).fold(0.0f32, f32::max);
        assert!(max_amplitude < 0.1);
    }
}
