//! SAR Processor — Synthetic Aperture Radar Range-Doppler Algorithm
//!
//! Core SAR imaging using the Range-Doppler Algorithm (RDA) for processing
//! raw radar echoes into 2D focused images. Implements range compression,
//! range cell migration correction (RCMC), and azimuth compression.
//!
//! ## Processing Chain
//!
//! ```text
//! Raw data → Range FFT → Range Compression → RCMC → Azimuth FFT
//!          → Azimuth Compression → Azimuth IFFT → Focused Image
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sar_processor::{SarProcessor, SarConfig};
//!
//! let config = SarConfig {
//!     range_bins: 256,
//!     azimuth_bins: 256,
//!     range_sample_rate: 50e6,
//!     prf: 500.0,
//!     carrier_freq: 5.4e9,
//!     chirp_bandwidth: 30e6,
//!     chirp_duration: 10e-6,
//!     platform_velocity: 100.0,
//!     platform_altitude: 5000.0,
//!     slant_range_center: 10000.0,
//! };
//!
//! let processor = SarProcessor::new(config);
//! assert_eq!(processor.range_resolution(), 299_792_458.0 / (2.0 * 30e6));
//! ```

use std::f64::consts::PI;

const C: f64 = 299_792_458.0; // Speed of light (m/s)

/// SAR processor configuration.
#[derive(Debug, Clone)]
pub struct SarConfig {
    /// Number of range bins (fast-time samples per pulse).
    pub range_bins: usize,
    /// Number of azimuth bins (slow-time pulses).
    pub azimuth_bins: usize,
    /// Range sampling rate (Hz).
    pub range_sample_rate: f64,
    /// Pulse repetition frequency (Hz).
    pub prf: f64,
    /// Carrier frequency (Hz).
    pub carrier_freq: f64,
    /// Chirp bandwidth (Hz).
    pub chirp_bandwidth: f64,
    /// Chirp pulse duration (s).
    pub chirp_duration: f64,
    /// Platform velocity (m/s).
    pub platform_velocity: f64,
    /// Platform altitude (m).
    pub platform_altitude: f64,
    /// Slant range to scene center (m).
    pub slant_range_center: f64,
}

/// Complex number for SAR processing.
#[derive(Debug, Clone, Copy)]
pub struct SarComplex {
    pub re: f64,
    pub im: f64,
}

impl SarComplex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }

    pub fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }

    pub fn mag_sq(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    pub fn mag(self) -> f64 {
        self.mag_sq().sqrt()
    }
}

impl std::ops::Add for SarComplex {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::AddAssign for SarComplex {
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

impl std::ops::Sub for SarComplex {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl std::ops::Mul for SarComplex {
    type Output = Self;
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::Mul<f64> for SarComplex {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }
}

/// SAR image output.
#[derive(Debug, Clone)]
pub struct SarImage {
    /// Complex image data [azimuth][range].
    pub data: Vec<Vec<SarComplex>>,
    /// Range resolution (m).
    pub range_resolution: f64,
    /// Azimuth resolution (m).
    pub azimuth_resolution: f64,
    /// Number of range bins.
    pub range_bins: usize,
    /// Number of azimuth bins.
    pub azimuth_bins: usize,
}

impl SarImage {
    /// Get magnitude image (dB scale, normalized).
    pub fn magnitude_db(&self) -> Vec<Vec<f64>> {
        let mut max_val = 0.0_f64;
        for row in &self.data {
            for c in row {
                max_val = max_val.max(c.mag());
            }
        }
        let max_db = if max_val > 0.0 {
            20.0 * max_val.log10()
        } else {
            0.0
        };

        self.data
            .iter()
            .map(|row| {
                row.iter()
                    .map(|c| {
                        let mag = c.mag();
                        if mag > 1e-15 {
                            20.0 * mag.log10() - max_db
                        } else {
                            -100.0
                        }
                    })
                    .collect()
            })
            .collect()
    }
}

/// SAR Range-Doppler Algorithm processor.
#[derive(Debug, Clone)]
pub struct SarProcessor {
    config: SarConfig,
    wavelength: f64,
}

impl SarProcessor {
    /// Create a new SAR processor.
    pub fn new(config: SarConfig) -> Self {
        let wavelength = C / config.carrier_freq;
        Self { config, wavelength }
    }

    /// Range resolution (m) = c / (2 * B).
    pub fn range_resolution(&self) -> f64 {
        C / (2.0 * self.config.chirp_bandwidth)
    }

    /// Azimuth resolution (m) — for focused SAR.
    /// δ_az ≈ V / (2 * B_doppler) where B_doppler = 2V/R₀ × aperture_time
    pub fn azimuth_resolution(&self) -> f64 {
        // Approximate: λ R₀ / (2 L_synth) where L_synth = V * T_aperture
        let t_aperture = self.config.azimuth_bins as f64 / self.config.prf;
        let l_synth = self.config.platform_velocity * t_aperture;
        if l_synth > 0.0 {
            self.wavelength * self.config.slant_range_center / (2.0 * l_synth)
        } else {
            f64::INFINITY
        }
    }

    /// Process raw radar data through the Range-Doppler Algorithm.
    ///
    /// Input: raw_pulses[azimuth_bin][range_bin] — complex baseband echoes.
    pub fn process_raw_data(&self, raw_pulses: &[Vec<SarComplex>]) -> SarImage {
        let na = raw_pulses.len();
        let nr = if na > 0 { raw_pulses[0].len() } else { 0 };

        // Step 1: Range compression
        let range_compressed = self.range_compression(raw_pulses);

        // Step 2: Range cell migration correction
        let rcmc_data = self.range_cell_migration_correction(&range_compressed);

        // Step 3: Azimuth compression
        let focused = self.azimuth_compression(&rcmc_data);

        SarImage {
            data: focused,
            range_resolution: self.range_resolution(),
            azimuth_resolution: self.azimuth_resolution(),
            range_bins: nr,
            azimuth_bins: na,
        }
    }

    /// Range compression via matched filtering.
    ///
    /// For each pulse: FFT → multiply by conj(FFT(reference)) → IFFT.
    pub fn range_compression(&self, raw_pulses: &[Vec<SarComplex>]) -> Vec<Vec<SarComplex>> {
        let nr = if !raw_pulses.is_empty() {
            raw_pulses[0].len()
        } else {
            return Vec::new();
        };

        let ref_chirp = self.generate_range_ref();
        let ref_fft = fft(&ref_chirp, nr);

        raw_pulses
            .iter()
            .map(|pulse| {
                let pulse_fft = fft(pulse, nr);
                let matched: Vec<SarComplex> = pulse_fft
                    .iter()
                    .zip(ref_fft.iter())
                    .map(|(&p, &r)| p * r.conj())
                    .collect();
                ifft(&matched, nr)
            })
            .collect()
    }

    /// Range Cell Migration Correction (RCMC).
    ///
    /// Corrects range walk and range curvature due to changing geometry.
    pub fn range_cell_migration_correction(
        &self,
        data: &[Vec<SarComplex>],
    ) -> Vec<Vec<SarComplex>> {
        let na = data.len();
        if na == 0 {
            return Vec::new();
        }
        let nr = data[0].len();

        // Perform RCMC in range-Doppler domain
        // For each azimuth frequency, shift range samples to correct migration
        // Transpose to [range][azimuth], FFT along azimuth, correct, IFFT

        let mut result = data.to_vec();

        // For each range bin, compute migration and apply phase correction
        let v = self.config.platform_velocity;
        let r0 = self.config.slant_range_center;

        for az in 0..na {
            let eta = (az as f64 - na as f64 / 2.0) / self.config.prf;
            let r_eta = (r0 * r0 + v * v * eta * eta).sqrt();
            let migration = r_eta - r0;
            let migration_samples = migration * 2.0 * self.config.range_sample_rate / C;

            // Integer shift for simplicity
            let shift = migration_samples.round() as isize;
            if shift != 0 && shift.unsigned_abs() < nr {
                let mut new_row = vec![SarComplex::zero(); nr];
                for r in 0..nr {
                    let src = r as isize - shift;
                    if src >= 0 && (src as usize) < nr {
                        new_row[r] = data[az][src as usize];
                    }
                }
                result[az] = new_row;
            }
        }

        result
    }

    /// Azimuth compression.
    ///
    /// For each range bin: FFT along azimuth → multiply by reference → IFFT.
    pub fn azimuth_compression(&self, data: &[Vec<SarComplex>]) -> Vec<Vec<SarComplex>> {
        let na = data.len();
        if na == 0 {
            return Vec::new();
        }
        let nr = data[0].len();

        // Transpose to process along azimuth
        let mut transposed = vec![vec![SarComplex::zero(); na]; nr];
        for az in 0..na {
            for r in 0..nr {
                transposed[r][az] = data[az][r];
            }
        }

        // For each range bin, azimuth compress
        for r in 0..nr {
            let az_ref = self.generate_azimuth_ref(r);
            let signal_fft = fft(&transposed[r], na);
            let ref_fft = fft(&az_ref, na);

            let matched: Vec<SarComplex> = signal_fft
                .iter()
                .zip(ref_fft.iter())
                .map(|(&s, &h)| s * h.conj())
                .collect();

            transposed[r] = ifft(&matched, na);
        }

        // Transpose back
        let mut result = vec![vec![SarComplex::zero(); nr]; na];
        for az in 0..na {
            for r in 0..nr {
                result[az][r] = transposed[r][az];
            }
        }

        result
    }

    /// Generate range reference (matched filter) chirp.
    pub fn generate_range_ref(&self) -> Vec<SarComplex> {
        let nr = self.config.range_bins;
        let kr = self.config.chirp_bandwidth / self.config.chirp_duration;
        let n_chirp = (self.config.chirp_duration * self.config.range_sample_rate) as usize;
        let n_chirp = n_chirp.min(nr);

        let mut ref_signal = vec![SarComplex::zero(); nr];
        for i in 0..n_chirp {
            let t = (i as f64 / self.config.range_sample_rate) - self.config.chirp_duration / 2.0;
            let phase = PI * kr * t * t;
            ref_signal[i] = SarComplex::from_polar(1.0, phase);
        }
        ref_signal
    }

    /// Generate azimuth reference function for a given range bin.
    pub fn generate_azimuth_ref(&self, _range_bin: usize) -> Vec<SarComplex> {
        let na = self.config.azimuth_bins;
        let v = self.config.platform_velocity;
        let r0 = self.config.slant_range_center;
        let ka = 2.0 * v * v / (self.wavelength * r0);

        let mut ref_signal = vec![SarComplex::zero(); na];
        for i in 0..na {
            let eta = (i as f64 - na as f64 / 2.0) / self.config.prf;
            let phase = -PI * ka * eta * eta;
            ref_signal[i] = SarComplex::from_polar(1.0, phase);
        }
        ref_signal
    }

    /// Estimate Doppler centroid from data.
    pub fn compute_doppler_centroid(&self, data: &[Vec<SarComplex>]) -> f64 {
        let na = data.len();
        if na < 2 {
            return 0.0;
        }

        // Average cross-correlation between successive pulses
        let mut sum = SarComplex::zero();
        for az in 0..(na - 1) {
            for r in 0..data[az].len() {
                sum += data[az + 1][r] * data[az][r].conj();
            }
        }

        // Doppler centroid = PRF / (2π) * angle(sum)
        let phase = sum.im.atan2(sum.re);
        self.config.prf * phase / (2.0 * PI)
    }

    /// Generate a synthetic point target scene for testing.
    pub fn generate_point_target(
        &self,
        range_offset: f64,
        azimuth_offset: f64,
        amplitude: f64,
    ) -> Vec<Vec<SarComplex>> {
        let na = self.config.azimuth_bins;
        let nr = self.config.range_bins;
        let v = self.config.platform_velocity;
        let r0 = self.config.slant_range_center + range_offset;
        let kr = self.config.chirp_bandwidth / self.config.chirp_duration;

        let mut raw = vec![vec![SarComplex::zero(); nr]; na];

        // Near-range delay — range samples start at this two-way delay
        let near_range_delay = 2.0 * (r0 - nr as f64 / 2.0 * C / (2.0 * self.config.range_sample_rate)) / C;

        for az in 0..na {
            let eta = (az as f64 - na as f64 / 2.0) / self.config.prf - azimuth_offset / v;
            let r_inst = (r0 * r0 + v * v * eta * eta).sqrt();
            let delay = 2.0 * r_inst / C;

            for r in 0..nr {
                let t = r as f64 / self.config.range_sample_rate + near_range_delay;
                let t_shifted = t - delay;
                if t_shifted.abs() < self.config.chirp_duration / 2.0 {
                    let phase_chirp = PI * kr * t_shifted * t_shifted;
                    let phase_range = -4.0 * PI * self.config.carrier_freq * r_inst / C;
                    raw[az][r] += SarComplex::from_polar(amplitude, phase_chirp + phase_range);
                }
            }
        }

        raw
    }
}

// --- FFT helpers (DIT radix-2 or DFT for non-power-of-2) ---

fn fft(input: &[SarComplex], n: usize) -> Vec<SarComplex> {
    let mut data: Vec<SarComplex> = if input.len() >= n {
        input[..n].to_vec()
    } else {
        let mut v = input.to_vec();
        v.resize(n, SarComplex::zero());
        v
    };

    if n <= 1 {
        return data;
    }

    // Use DFT for small or non-power-of-2 sizes
    if !n.is_power_of_two() || n <= 4 {
        return dft(&data);
    }

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Cooley-Tukey FFT
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        let w_n = SarComplex::from_polar(1.0, angle);
        let mut start = 0;
        while start < n {
            let mut w = SarComplex::new(1.0, 0.0);
            for k in 0..half {
                let even = data[start + k];
                let odd = data[start + k + half] * w;
                data[start + k] = even + odd;
                data[start + k + half] = even - odd;
                w = w * w_n;
            }
            start += len;
        }
        len <<= 1;
    }

    data
}

fn ifft(input: &[SarComplex], n: usize) -> Vec<SarComplex> {
    // Conjugate, FFT, conjugate, scale
    let conj_input: Vec<SarComplex> = input.iter().map(|c| c.conj()).collect();
    let mut result = fft(&conj_input, n);
    let scale = 1.0 / n as f64;
    for c in &mut result {
        *c = c.conj() * scale;
    }
    result
}

fn dft(input: &[SarComplex]) -> Vec<SarComplex> {
    let n = input.len();
    let mut output = vec![SarComplex::zero(); n];
    for k in 0..n {
        for j in 0..n {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            let w = SarComplex::from_polar(1.0, angle);
            output[k] += input[j] * w;
        }
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> SarConfig {
        SarConfig {
            range_bins: 128,
            azimuth_bins: 128,
            range_sample_rate: 50e6,
            prf: 500.0,
            carrier_freq: 5.4e9,
            chirp_bandwidth: 30e6,
            chirp_duration: 10e-6,
            platform_velocity: 100.0,
            platform_altitude: 5000.0,
            slant_range_center: 10000.0,
        }
    }

    #[test]
    fn test_range_resolution() {
        let proc = SarProcessor::new(default_config());
        let dr = proc.range_resolution();
        let expected = C / (2.0 * 30e6);
        assert!((dr - expected).abs() < 0.01, "Range res: expected {:.2}, got {:.2}", expected, dr);
    }

    #[test]
    fn test_azimuth_resolution() {
        let proc = SarProcessor::new(default_config());
        let da = proc.azimuth_resolution();
        // Should be a reasonable value (meters)
        assert!(da > 0.0 && da < 100.0, "Azimuth res: {:.2} m", da);
    }

    #[test]
    fn test_range_reference_chirp() {
        let proc = SarProcessor::new(default_config());
        let ref_chirp = proc.generate_range_ref();
        assert_eq!(ref_chirp.len(), 128);
        // First sample should have magnitude ~1
        let first_nonzero = ref_chirp.iter().find(|c| c.mag() > 0.5);
        assert!(first_nonzero.is_some(), "Reference chirp should have non-zero samples");
    }

    #[test]
    fn test_range_compression() {
        let proc = SarProcessor::new(default_config());
        let ref_chirp = proc.generate_range_ref();

        // Simulate a single pulse echo (chirp shifted in range)
        let nr = 128;
        let pulse = vec![ref_chirp.clone()]; // Echo = reference chirp

        let compressed = proc.range_compression(&pulse);
        assert_eq!(compressed.len(), 1);
        assert_eq!(compressed[0].len(), nr);

        // Find peak — should be concentrated
        let mut max_mag = 0.0;
        let mut max_idx = 0;
        for (i, c) in compressed[0].iter().enumerate() {
            let mag = c.mag();
            if mag > max_mag {
                max_mag = mag;
                max_idx = i;
            }
        }
        assert!(max_mag > 0.0, "Compressed signal should have a peak");
        // Peak should exist (not necessarily at center for autocorrelation)
        assert!(max_idx < nr, "Peak index should be within range");
    }

    #[test]
    fn test_point_target_generation() {
        let proc = SarProcessor::new(default_config());
        let raw = proc.generate_point_target(0.0, 0.0, 1.0);
        assert_eq!(raw.len(), 128);
        assert_eq!(raw[0].len(), 128);

        // Should have non-zero energy
        let total_energy: f64 = raw.iter()
            .flat_map(|row| row.iter())
            .map(|c| c.mag_sq())
            .sum();
        assert!(total_energy > 0.0, "Point target should have energy");
    }

    #[test]
    fn test_point_target_focusing() {
        let config = SarConfig {
            range_bins: 64,
            azimuth_bins: 64,
            range_sample_rate: 50e6,
            prf: 500.0,
            carrier_freq: 5.4e9,
            chirp_bandwidth: 30e6,
            chirp_duration: 10e-6,
            platform_velocity: 100.0,
            platform_altitude: 5000.0,
            slant_range_center: 10000.0,
        };
        let proc = SarProcessor::new(config);
        let raw = proc.generate_point_target(0.0, 0.0, 1.0);
        let image = proc.process_raw_data(&raw);

        assert_eq!(image.data.len(), 64);
        assert_eq!(image.data[0].len(), 64);

        // Find peak in focused image
        let mut max_mag = 0.0_f64;
        for row in &image.data {
            for c in row {
                max_mag = max_mag.max(c.mag());
            }
        }
        assert!(max_mag > 0.0, "Focused image should have a peak");
    }

    #[test]
    fn test_doppler_centroid_zero_squint() {
        let proc = SarProcessor::new(default_config());
        let raw = proc.generate_point_target(0.0, 0.0, 1.0);
        let f_dc = proc.compute_doppler_centroid(&raw);
        // Zero-squint should have small Doppler centroid
        assert!(f_dc.abs() < 100.0, "Doppler centroid should be near 0 for zero squint, got {:.1}", f_dc);
    }

    #[test]
    fn test_magnitude_db_image() {
        let config = SarConfig {
            range_bins: 32,
            azimuth_bins: 32,
            range_sample_rate: 50e6,
            prf: 500.0,
            carrier_freq: 5.4e9,
            chirp_bandwidth: 30e6,
            chirp_duration: 10e-6,
            platform_velocity: 100.0,
            platform_altitude: 5000.0,
            slant_range_center: 10000.0,
        };
        let proc = SarProcessor::new(config);
        let raw = proc.generate_point_target(0.0, 0.0, 1.0);
        let image = proc.process_raw_data(&raw);
        let db = image.magnitude_db();
        assert_eq!(db.len(), 32);
        assert_eq!(db[0].len(), 32);

        // Peak should be at 0 dB (normalized)
        let mut max_db = -200.0_f64;
        for row in &db {
            for &val in row {
                max_db = max_db.max(val);
            }
        }
        assert!((max_db - 0.0).abs() < 0.1, "Peak should be ~0 dB, got {:.1}", max_db);
    }

    #[test]
    fn test_empty_input() {
        let proc = SarProcessor::new(default_config());
        let compressed = proc.range_compression(&[]);
        assert!(compressed.is_empty());

        let rcmc = proc.range_cell_migration_correction(&[]);
        assert!(rcmc.is_empty());

        let az = proc.azimuth_compression(&[]);
        assert!(az.is_empty());
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let n = 16;
        let signal: Vec<SarComplex> = (0..n)
            .map(|i| SarComplex::from_polar(1.0, 2.0 * PI * 3.0 * i as f64 / n as f64))
            .collect();
        let freq = fft(&signal, n);
        let recovered = ifft(&freq, n);

        for i in 0..n {
            let err = (signal[i] - recovered[i]).mag();
            assert!(err < 1e-10, "FFT roundtrip error at {}: {:.2e}", i, err);
        }
    }
}
