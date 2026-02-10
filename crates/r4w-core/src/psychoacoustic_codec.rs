//! Perceptual audio coding with critical band analysis and masking for voice/music over RF.
//!
//! This module implements a psychoacoustic codec that exploits human auditory masking
//! to efficiently compress audio. It uses the Bark critical band scale, simultaneous
//! masking models, and perceptual bit allocation to achieve low bitrate encoding
//! suitable for transmission over RF channels.
//!
//! # Example
//!
//! ```
//! use r4w_core::psychoacoustic_codec::{PsychoacousticCodec, CodecConfig};
//!
//! let config = CodecConfig {
//!     sample_rate: 8000.0,
//!     num_bands: 24,
//!     target_bitrate_bps: 16000,
//!     frame_size: 256,
//! };
//!
//! let codec = PsychoacousticCodec::new(config);
//!
//! // Generate a simple test tone at 440 Hz
//! let duration_samples = 256;
//! let audio: Vec<f64> = (0..duration_samples)
//!     .map(|i| 0.8 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 8000.0).sin())
//!     .collect();
//!
//! // Encode and decode
//! let encoded = codec.encode(&audio);
//! let decoded = codec.decode(&encoded);
//!
//! // Decoded audio should have the same length as the original frame
//! assert_eq!(decoded.len(), duration_samples);
//!
//! // Check that the reconstructed signal has reasonable energy
//! let energy: f64 = decoded.iter().map(|x| x * x).sum::<f64>() / decoded.len() as f64;
//! assert!(energy > 0.01, "Decoded signal should have non-trivial energy");
//! ```

use std::f64::consts::PI;

/// Configuration for the psychoacoustic codec.
#[derive(Debug, Clone)]
pub struct CodecConfig {
    /// Audio sample rate in Hz.
    pub sample_rate: f64,
    /// Number of Bark critical bands to use (typically 24).
    pub num_bands: usize,
    /// Target bitrate in bits per second.
    pub target_bitrate_bps: usize,
    /// Number of samples per frame.
    pub frame_size: usize,
}

impl Default for CodecConfig {
    fn default() -> Self {
        Self {
            sample_rate: 8000.0,
            num_bands: 24,
            target_bitrate_bps: 16000,
            frame_size: 256,
        }
    }
}

/// A Bark critical band with frequency boundaries.
#[derive(Debug, Clone)]
pub struct BarkBand {
    /// Center frequency in Hz.
    pub center_freq_hz: f64,
    /// Bandwidth of this critical band in Hz.
    pub bandwidth_hz: f64,
    /// Lower edge frequency in Hz.
    pub lower_hz: f64,
    /// Upper edge frequency in Hz.
    pub upper_hz: f64,
}

/// Computes simultaneous masking thresholds from a spectral representation.
#[derive(Debug, Clone)]
pub struct MaskingModel {
    /// The Bark bands used for masking computation.
    bands: Vec<BarkBand>,
}

impl MaskingModel {
    /// Create a new masking model for the given Bark bands.
    pub fn new(bands: Vec<BarkBand>) -> Self {
        Self { bands }
    }

    /// Compute the spreading function value for a given Bark distance.
    ///
    /// Uses a simplified version of the Schroeder spreading function:
    ///   SF(dz) = 15.81 + 7.5*(dz + 0.474) - 17.5*sqrt(1 + (dz + 0.474)^2)
    pub fn spreading_function(bark_distance: f64) -> f64 {
        let dz = bark_distance + 0.474;
        15.81 + 7.5 * dz - 17.5 * (1.0 + dz * dz).sqrt()
    }

    /// Compute masking thresholds for each Bark band given spectral power in dB.
    ///
    /// For each band, the masking threshold is the sum of masking contributions
    /// from all other bands, weighted by the spreading function.
    pub fn compute_thresholds(&self, spectrum_db: &[f64]) -> Vec<f64> {
        let n = self.bands.len().min(spectrum_db.len());
        let mut thresholds = vec![f64::NEG_INFINITY; n];

        for i in 0..n {
            let bark_i = bark_scale(self.bands[i].center_freq_hz);
            let mut max_mask = f64::NEG_INFINITY;

            for j in 0..n {
                let bark_j = bark_scale(self.bands[j].center_freq_hz);
                let dz = bark_i - bark_j;
                let spread = Self::spreading_function(dz);
                // Masking contribution: power of masker + spreading attenuation
                let mask_contribution = spectrum_db[j] + spread;
                if mask_contribution > max_mask {
                    max_mask = mask_contribution;
                }
            }
            thresholds[i] = max_mask;
        }

        thresholds
    }
}

/// Perceptual bit allocation across critical bands.
///
/// Iteratively assigns bits to the bands with the highest mask-to-noise ratio (MNR).
#[derive(Debug, Clone)]
pub struct BitAllocator;

impl BitAllocator {
    /// Create a new bit allocator.
    pub fn new() -> Self {
        Self
    }

    /// Allocate bits across bands based on mask-to-noise ratio.
    ///
    /// Bands with higher MNR receive more bits. Uses a greedy iterative
    /// approach: each iteration assigns one bit to the band with the highest
    /// remaining MNR, then reduces that band's MNR by ~6 dB (one bit of
    /// quantization improvement).
    pub fn allocate(&self, mask_to_noise: &[f64], total_bits: usize) -> Vec<usize> {
        let n = mask_to_noise.len();
        if n == 0 {
            return vec![];
        }

        let mut bits = vec![0usize; n];
        let mut remaining_mnr: Vec<f64> = mask_to_noise.to_vec();
        let max_bits_per_band = 16;

        for _ in 0..total_bits {
            // Find band with highest remaining MNR
            let mut best_band = 0;
            let mut best_mnr = f64::NEG_INFINITY;
            for (i, &mnr) in remaining_mnr.iter().enumerate() {
                if mnr > best_mnr && bits[i] < max_bits_per_band {
                    best_mnr = mnr;
                    best_band = i;
                }
            }

            if best_mnr <= f64::NEG_INFINITY {
                break;
            }

            bits[best_band] += 1;
            // Each additional bit reduces quantization noise by ~6.02 dB
            remaining_mnr[best_band] -= 6.02;
        }

        bits
    }
}

impl Default for BitAllocator {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert a frequency in Hz to the Bark scale.
///
/// Uses the formula: z = 13*atan(0.00076*f) + 3.5*atan((f/7500)^2)
pub fn bark_scale(freq_hz: f64) -> f64 {
    13.0 * (0.00076 * freq_hz).atan() + 3.5 * ((freq_hz / 7500.0).powi(2)).atan()
}

/// Convert a Bark value back to frequency in Hz.
///
/// Uses Newton's method to invert the Bark scale function.
pub fn bark_to_hz(bark: f64) -> f64 {
    // Newton's method to invert bark_scale
    let mut freq = bark * 100.0; // Initial guess
    for _ in 0..50 {
        let current_bark = bark_scale(freq);
        let error = current_bark - bark;
        if error.abs() < 1e-6 {
            break;
        }
        // Numerical derivative
        let h = 1.0;
        let deriv = (bark_scale(freq + h) - bark_scale(freq - h)) / (2.0 * h);
        if deriv.abs() < 1e-15 {
            break;
        }
        freq -= error / deriv;
        if freq < 0.0 {
            freq = 0.0;
        }
    }
    freq
}

/// Perceptual audio codec using critical band analysis and masking.
///
/// Encodes audio frames by:
/// 1. Computing the DFT of the audio frame
/// 2. Grouping spectral energy into Bark critical bands
/// 3. Computing masking thresholds via the spreading function
/// 4. Allocating bits proportional to mask-to-noise ratio
/// 5. Quantizing each band with the allocated number of bits
pub struct PsychoacousticCodec {
    config: CodecConfig,
    bands: Vec<BarkBand>,
    masking_model: MaskingModel,
    bit_allocator: BitAllocator,
}

impl PsychoacousticCodec {
    /// Create a new psychoacoustic codec with the given configuration.
    pub fn new(config: CodecConfig) -> Self {
        let bands = Self::compute_bark_bands(&config);
        let masking_model = MaskingModel::new(bands.clone());
        let bit_allocator = BitAllocator::new();
        Self {
            config,
            bands,
            masking_model,
            bit_allocator,
        }
    }

    /// Compute Bark critical bands for the given configuration.
    fn compute_bark_bands(config: &CodecConfig) -> Vec<BarkBand> {
        let nyquist = config.sample_rate / 2.0;
        let max_bark = bark_scale(nyquist);
        let bark_step = max_bark / config.num_bands as f64;
        let mut bands = Vec::with_capacity(config.num_bands);

        for i in 0..config.num_bands {
            let lower_bark = i as f64 * bark_step;
            let upper_bark = (i as f64 + 1.0) * bark_step;
            let lower_hz = bark_to_hz(lower_bark);
            let upper_hz = bark_to_hz(upper_bark).min(nyquist);
            let center_freq_hz = (lower_hz + upper_hz) / 2.0;
            let bandwidth_hz = upper_hz - lower_hz;

            bands.push(BarkBand {
                center_freq_hz,
                bandwidth_hz,
                lower_hz,
                upper_hz,
            });
        }

        bands
    }

    /// Return a reference to the Bark bands.
    pub fn bands(&self) -> &[BarkBand] {
        &self.bands
    }

    /// Return a reference to the codec configuration.
    pub fn config(&self) -> &CodecConfig {
        &self.config
    }

    /// Compute the DFT of a real-valued signal using (f64, f64) complex tuples.
    fn dft(signal: &[f64]) -> Vec<(f64, f64)> {
        let n = signal.len();
        let mut spectrum = Vec::with_capacity(n);
        for k in 0..n {
            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &x) in signal.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                re += x * angle.cos();
                im += x * angle.sin();
            }
            spectrum.push((re, im));
        }
        spectrum
    }

    /// Compute the inverse DFT, returning real-valued samples.
    fn idft(spectrum: &[(f64, f64)]) -> Vec<f64> {
        let n = spectrum.len();
        let mut signal = Vec::with_capacity(n);
        for i in 0..n {
            let mut re = 0.0;
            for (k, &(sk_re, sk_im)) in spectrum.iter().enumerate() {
                let angle = 2.0 * PI * k as f64 * i as f64 / n as f64;
                re += sk_re * angle.cos() - sk_im * angle.sin();
            }
            signal.push(re / n as f64);
        }
        signal
    }

    /// Compute the power spectrum in dB from a DFT.
    fn power_spectrum_db(spectrum: &[(f64, f64)]) -> Vec<f64> {
        spectrum
            .iter()
            .map(|&(re, im)| {
                let power = re * re + im * im;
                if power > 1e-30 {
                    10.0 * power.log10()
                } else {
                    -300.0
                }
            })
            .collect()
    }

    /// Group spectral bins into Bark band energies (in dB).
    fn band_energies_db(&self, power_db: &[f64]) -> Vec<f64> {
        let n = power_db.len();
        let freq_resolution = self.config.sample_rate / n as f64;

        self.bands
            .iter()
            .map(|band| {
                let bin_low = (band.lower_hz / freq_resolution).floor() as usize;
                let bin_high = ((band.upper_hz / freq_resolution).ceil() as usize).min(n / 2);

                if bin_low >= bin_high || bin_low >= n {
                    return -300.0;
                }

                // Sum linear power then convert to dB
                let mut total_power = 0.0;
                for bin in bin_low..bin_high {
                    if bin < power_db.len() {
                        total_power += 10.0_f64.powf(power_db[bin] / 10.0);
                    }
                }

                if total_power > 1e-30 {
                    10.0 * total_power.log10()
                } else {
                    -300.0
                }
            })
            .collect()
    }

    /// Find which Bark band a given FFT bin belongs to.
    fn bin_to_band(&self, bin_index: usize, fft_size: usize) -> Option<usize> {
        let freq = bin_index as f64 * self.config.sample_rate / fft_size as f64;
        for (i, band) in self.bands.iter().enumerate() {
            if freq >= band.lower_hz && freq < band.upper_hz {
                return Some(i);
            }
        }
        None
    }

    /// Compute the masking threshold for each Bark band.
    pub fn masking_threshold(&self, spectrum_db: &[f64]) -> Vec<f64> {
        self.masking_model.compute_thresholds(spectrum_db)
    }

    /// Allocate bits across critical bands based on mask-to-noise ratio.
    pub fn allocate_bits(&self, mask_to_noise: &[f64], total_bits: usize) -> Vec<usize> {
        self.bit_allocator.allocate(mask_to_noise, total_bits)
    }

    /// Compute the signal-to-mask ratio for each Bark band.
    ///
    /// Returns the difference (in dB) between each band's energy and its masking threshold.
    pub fn signal_to_mask_ratio(&self, audio: &[f64]) -> Vec<f64> {
        let spectrum = Self::dft(audio);
        let power_db = Self::power_spectrum_db(&spectrum);
        let band_energy = self.band_energies_db(&power_db);
        let thresholds = self.masking_model.compute_thresholds(&band_energy);

        band_energy
            .iter()
            .zip(thresholds.iter())
            .map(|(&energy, &threshold)| energy - threshold)
            .collect()
    }

    /// Encode an audio frame into a byte vector.
    ///
    /// The encoded format is:
    /// - 2 bytes: frame size (u16 big-endian)
    /// - 1 byte: number of bands
    /// - For each band: 1 byte (bits allocated)
    /// - For each band with >0 bits: quantized coefficients packed into bytes
    /// - 8 bytes: max magnitude (f64 big-endian) at the end for reconstruction
    pub fn encode(&self, audio: &[f64]) -> Vec<u8> {
        let frame_size = audio.len().min(self.config.frame_size);
        let frame: Vec<f64> = if audio.len() >= frame_size {
            audio[..frame_size].to_vec()
        } else {
            let mut padded = audio.to_vec();
            padded.resize(frame_size, 0.0);
            padded
        };

        // Compute DFT and band energies
        let spectrum = Self::dft(&frame);
        let power_db = Self::power_spectrum_db(&spectrum);
        let band_energy = self.band_energies_db(&power_db);

        // Compute masking thresholds and MNR
        let thresholds = self.masking_model.compute_thresholds(&band_energy);
        let mnr: Vec<f64> = band_energy
            .iter()
            .zip(thresholds.iter())
            .map(|(&e, &t)| (e - t).max(0.0))
            .collect();

        // Compute total bits per frame from bitrate
        let bits_per_frame =
            (self.config.target_bitrate_bps as f64 * frame_size as f64 / self.config.sample_rate)
                as usize;

        // Reserve bits for header
        let header_bits = (3 + self.bands.len()) * 8;
        let payload_bits = if bits_per_frame > header_bits {
            bits_per_frame - header_bits
        } else {
            self.bands.len() * 2 // minimum: 2 bits per band
        };

        // Allocate bits
        let bit_alloc = self.bit_allocator.allocate(&mnr, payload_bits);

        // Compute per-band average magnitude for quantization
        let half = frame_size / 2 + 1;
        let mut band_values: Vec<Vec<f64>> = vec![vec![]; self.bands.len()];
        for bin in 0..half {
            if let Some(band_idx) = self.bin_to_band(bin, frame_size) {
                let mag = (spectrum[bin].0 * spectrum[bin].0 + spectrum[bin].1 * spectrum[bin].1)
                    .sqrt();
                band_values[band_idx].push(mag);
            }
        }

        let band_mags: Vec<f64> = band_values
            .iter()
            .map(|vals| {
                if vals.is_empty() {
                    0.0
                } else {
                    vals.iter().sum::<f64>() / vals.len() as f64
                }
            })
            .collect();

        // Find global max magnitude for normalization
        let max_mag = band_mags
            .iter()
            .cloned()
            .fold(1e-30, f64::max);

        // Build output bytes
        let mut output = Vec::new();

        // Header: frame_size (u16), num_bands (u8)
        output.push((frame_size >> 8) as u8);
        output.push((frame_size & 0xFF) as u8);
        output.push(self.bands.len() as u8);

        // Bit allocation table
        for &bits in &bit_alloc {
            output.push(bits.min(255) as u8);
        }

        // Quantized band magnitudes
        for (i, &bits) in bit_alloc.iter().enumerate() {
            if bits == 0 || i >= band_mags.len() {
                continue;
            }
            let normalized = band_mags[i] / max_mag; // 0.0 to 1.0
            let levels = (1u32 << bits) - 1;
            let quantized = (normalized * levels as f64).round() as u32;

            // Pack quantized value into bytes (big-endian)
            let num_bytes = (bits + 7) / 8;
            for b in (0..num_bytes).rev() {
                output.push(((quantized >> (b * 8)) & 0xFF) as u8);
            }
        }

        // Store max_mag as 8 bytes at the end for reconstruction
        let mag_bytes = max_mag.to_be_bytes();
        output.extend_from_slice(&mag_bytes);

        output
    }

    /// Decode a byte vector back into an audio frame.
    pub fn decode(&self, data: &[u8]) -> Vec<f64> {
        if data.len() < 3 {
            return vec![];
        }

        // Parse header
        let frame_size = ((data[0] as usize) << 8) | (data[1] as usize);
        let num_bands = data[2] as usize;

        if frame_size == 0 || data.len() < 3 + num_bands + 8 {
            return vec![0.0; self.config.frame_size];
        }

        // Read bit allocation
        let mut bit_alloc = Vec::with_capacity(num_bands);
        for i in 0..num_bands {
            if 3 + i < data.len() {
                bit_alloc.push(data[3 + i] as usize);
            } else {
                bit_alloc.push(0);
            }
        }

        // Read max_mag from end
        let mag_start = data.len() - 8;
        let mut mag_bytes = [0u8; 8];
        mag_bytes.copy_from_slice(&data[mag_start..mag_start + 8]);
        let max_mag = f64::from_be_bytes(mag_bytes);

        // Read quantized band magnitudes
        let mut offset = 3 + num_bands;
        let mut band_mags = vec![0.0_f64; num_bands];

        for (i, &bits) in bit_alloc.iter().enumerate() {
            if bits == 0 {
                continue;
            }
            let num_bytes = (bits + 7) / 8;
            if offset + num_bytes > mag_start {
                break;
            }
            let mut quantized: u32 = 0;
            for b in (0..num_bytes).rev() {
                if offset < mag_start {
                    quantized |= (data[offset] as u32) << (b * 8);
                    offset += 1;
                }
            }
            let levels = (1u32 << bits) - 1;
            if levels > 0 {
                band_mags[i] = (quantized as f64 / levels as f64) * max_mag;
            }
        }

        // Reconstruct spectrum: assign magnitude to bins within each band,
        // using zero phase (simplified reconstruction).
        let half = frame_size / 2 + 1;
        let mut spectrum: Vec<(f64, f64)> = vec![(0.0, 0.0); frame_size];
        let freq_resolution = self.config.sample_rate / frame_size as f64;

        for bin in 0..half {
            let freq = bin as f64 * freq_resolution;
            // Find which band this bin belongs to
            let mut band_idx = None;
            for (i, band) in self.bands.iter().enumerate() {
                if i < num_bands && freq >= band.lower_hz && freq < band.upper_hz {
                    band_idx = Some(i);
                    break;
                }
            }

            if let Some(bi) = band_idx {
                // Count bins in this band for normalization
                let band = &self.bands[bi];
                let bin_low = (band.lower_hz / freq_resolution).floor() as usize;
                let bin_high = ((band.upper_hz / freq_resolution).ceil() as usize).min(half);
                let band_bin_count = if bin_high > bin_low {
                    bin_high - bin_low
                } else {
                    1
                };

                // Distribute band magnitude across its bins
                let bin_mag = band_mags[bi] / (band_bin_count as f64).sqrt();
                spectrum[bin] = (bin_mag, 0.0);

                // Mirror for negative frequencies (conjugate symmetry)
                if bin > 0 && bin < frame_size / 2 {
                    spectrum[frame_size - bin] = (bin_mag, 0.0);
                }
            }
        }

        // Inverse DFT
        Self::idft(&spectrum)
    }

    /// Compute the Bark scale for a frequency (convenience wrapper).
    pub fn bark_scale(freq_hz: f64) -> f64 {
        bark_scale(freq_hz)
    }

    /// Convert Bark to Hz (convenience wrapper).
    pub fn bark_to_hz(bark: f64) -> f64 {
        bark_to_hz(bark)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> CodecConfig {
        CodecConfig {
            sample_rate: 8000.0,
            num_bands: 24,
            target_bitrate_bps: 16000,
            frame_size: 256,
        }
    }

    fn sine_wave(freq: f64, sample_rate: f64, num_samples: usize, amplitude: f64) -> Vec<f64> {
        (0..num_samples)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    #[test]
    fn test_bark_scale_zero() {
        let b = bark_scale(0.0);
        assert!((b - 0.0).abs() < 1e-6, "Bark of 0 Hz should be ~0, got {}", b);
    }

    #[test]
    fn test_bark_scale_1000hz() {
        let b = bark_scale(1000.0);
        // Expected ~8.5 Bark for 1000 Hz
        assert!(b > 7.0 && b < 10.0, "Bark of 1000 Hz should be ~8.5, got {}", b);
    }

    #[test]
    fn test_bark_scale_monotonic() {
        let freqs = [100.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0];
        for w in freqs.windows(2) {
            let b0 = bark_scale(w[0]);
            let b1 = bark_scale(w[1]);
            assert!(b1 > b0, "Bark scale should be monotonically increasing: f={} bark={} >= f={} bark={}", w[0], b0, w[1], b1);
        }
    }

    #[test]
    fn test_bark_to_hz_roundtrip() {
        let test_freqs = [100.0, 500.0, 1000.0, 2000.0, 3500.0];
        for &freq in &test_freqs {
            let bark = bark_scale(freq);
            let recovered = bark_to_hz(bark);
            let error = (recovered - freq).abs();
            assert!(
                error < 5.0,
                "bark_to_hz roundtrip failed for {} Hz: got {} Hz (error {})",
                freq, recovered, error
            );
        }
    }

    #[test]
    fn test_codec_config_default() {
        let config = CodecConfig::default();
        assert_eq!(config.sample_rate, 8000.0);
        assert_eq!(config.num_bands, 24);
        assert_eq!(config.target_bitrate_bps, 16000);
        assert_eq!(config.frame_size, 256);
    }

    #[test]
    fn test_bark_bands_count() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);
        assert_eq!(codec.bands().len(), 24);
    }

    #[test]
    fn test_bark_bands_cover_spectrum() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config.clone());
        let bands = codec.bands();

        // First band should start near 0 Hz
        assert!(bands[0].lower_hz < 50.0, "First band lower edge should be near 0 Hz");

        // Last band should reach near Nyquist
        let nyquist = config.sample_rate / 2.0;
        let last = &bands[bands.len() - 1];
        assert!(
            last.upper_hz >= nyquist * 0.9,
            "Last band upper edge ({}) should be near Nyquist ({})",
            last.upper_hz,
            nyquist
        );
    }

    #[test]
    fn test_bark_bands_non_overlapping() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);
        let bands = codec.bands();
        for i in 1..bands.len() {
            assert!(
                bands[i].lower_hz >= bands[i - 1].lower_hz,
                "Bands should be in ascending frequency order"
            );
        }
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);
        let audio = sine_wave(440.0, 8000.0, 256, 0.8);

        let encoded = codec.encode(&audio);
        assert!(!encoded.is_empty(), "Encoded data should not be empty");

        let decoded = codec.decode(&encoded);
        assert_eq!(decoded.len(), 256, "Decoded frame should match original size");

        // Check that decoded signal has energy
        let energy: f64 = decoded.iter().map(|x| x * x).sum::<f64>() / decoded.len() as f64;
        assert!(energy > 0.001, "Decoded signal should have energy, got {}", energy);
    }

    #[test]
    fn test_encode_silence() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);
        let audio = vec![0.0; 256];

        let encoded = codec.encode(&audio);
        let decoded = codec.decode(&encoded);

        // Decoded silence should have near-zero energy
        let energy: f64 = decoded.iter().map(|x| x * x).sum::<f64>() / decoded.len() as f64;
        assert!(energy < 0.01, "Decoded silence should have near-zero energy, got {}", energy);
    }

    #[test]
    fn test_masking_model_spreading_function() {
        // At zero Bark distance, spreading should be near its peak
        let sf0 = MaskingModel::spreading_function(0.0);
        // At large distances, spreading should be much lower
        let sf5 = MaskingModel::spreading_function(5.0);
        let sf_neg5 = MaskingModel::spreading_function(-5.0);

        assert!(sf0 > sf5, "Spreading should decrease with positive distance");
        assert!(sf0 > sf_neg5, "Spreading should decrease with negative distance");
    }

    #[test]
    fn test_masking_threshold_shape() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);

        // Create a spectrum with one strong band
        let mut spectrum_db = vec![-60.0; 24];
        spectrum_db[10] = 0.0; // Strong signal in band 10

        let thresholds = codec.masking_threshold(&spectrum_db);
        assert_eq!(thresholds.len(), 24);

        // Threshold near the strong band should be higher than far away
        let near_threshold = thresholds[10];
        let far_threshold = thresholds[0];
        assert!(
            near_threshold > far_threshold,
            "Masking threshold near strong signal ({}) should exceed far ({}) ",
            near_threshold, far_threshold
        );
    }

    #[test]
    fn test_bit_allocator_total() {
        let allocator = BitAllocator::new();
        let mnr = vec![10.0, 20.0, 5.0, 15.0];
        let total_bits = 16;
        let alloc = allocator.allocate(&mnr, total_bits);

        let sum: usize = alloc.iter().sum();
        assert_eq!(sum, total_bits, "Total allocated bits should equal requested total");
    }

    #[test]
    fn test_bit_allocator_favors_high_mnr() {
        let allocator = BitAllocator::new();
        let mnr = vec![0.0, 50.0, 0.0, 0.0];
        let total_bits = 4;
        let alloc = allocator.allocate(&mnr, total_bits);

        // Band 1 has highest MNR by far, so it should get most or all bits
        assert!(
            alloc[1] >= 3,
            "Band with highest MNR should get most bits, got {:?}",
            alloc
        );
    }

    #[test]
    fn test_bit_allocator_empty() {
        let allocator = BitAllocator::new();
        let alloc = allocator.allocate(&[], 10);
        assert!(alloc.is_empty());
    }

    #[test]
    fn test_signal_to_mask_ratio() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);

        let audio = sine_wave(1000.0, 8000.0, 256, 0.9);
        let smr = codec.signal_to_mask_ratio(&audio);

        assert_eq!(smr.len(), 24, "SMR should have one value per band");

        // At least one band should have a non-trivially negative or positive SMR
        let has_nonzero = smr.iter().any(|&v| v.abs() > 0.001);
        assert!(has_nonzero, "SMR should have at least one non-zero value: {:?}", smr);
    }

    #[test]
    fn test_encode_short_frame() {
        let config = default_config();
        let codec = PsychoacousticCodec::new(config);

        // Shorter than frame_size: should still work (gets padded)
        let audio = sine_wave(440.0, 8000.0, 64, 0.5);
        let encoded = codec.encode(&audio);
        assert!(!encoded.is_empty());

        let decoded = codec.decode(&encoded);
        assert!(!decoded.is_empty());
    }

    #[test]
    fn test_dft_idft_roundtrip() {
        let signal: Vec<f64> = (0..64)
            .map(|i| 0.5 * (2.0 * PI * 3.0 * i as f64 / 64.0).sin())
            .collect();

        let spectrum = PsychoacousticCodec::dft(&signal);
        let recovered = PsychoacousticCodec::idft(&spectrum);

        assert_eq!(recovered.len(), signal.len());
        for (i, (&orig, &rec)) in signal.iter().zip(recovered.iter()).enumerate() {
            assert!(
                (orig - rec).abs() < 1e-8,
                "DFT/IDFT roundtrip mismatch at index {}: {} vs {}",
                i, orig, rec
            );
        }
    }

    #[test]
    fn test_convenience_bark_methods() {
        let b1 = PsychoacousticCodec::bark_scale(1000.0);
        let b2 = bark_scale(1000.0);
        assert!((b1 - b2).abs() < 1e-12, "Convenience method should match free function");

        let h1 = PsychoacousticCodec::bark_to_hz(5.0);
        let h2 = bark_to_hz(5.0);
        assert!((h1 - h2).abs() < 1e-12, "Convenience method should match free function");
    }
}
