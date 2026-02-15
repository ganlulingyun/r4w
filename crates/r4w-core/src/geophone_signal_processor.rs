//! Single geophone signal processing for seismic data acquisition.
//!
//! This module implements signal-level processing for individual geophone sensors,
//! including instrument response correction, ground motion conversion, spectral
//! analysis, seismic event characterization, coupling assessment, and signal
//! quality metrics.
//!
//! This is distinct from the [`geophone_array_processor`](crate::geophone_array_processor)
//! which handles multi-channel array processing for reflection seismology.
//!
//! # Geophone Transfer Function
//!
//! A moving-coil geophone has the transfer function:
//!
//! ```text
//! H(s) = S * s^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
//! ```
//!
//! where `S` is the sensitivity (V/(m/s)), `zeta` is the damping ratio,
//! and `omega_n = 2*pi*f_n` is the natural angular frequency.
//!
//! # Example
//!
//! ```
//! use r4w_core::geophone_signal_processor::{
//!     GeophoneModel, GeophoneProcessor, sta_lta_trigger,
//! };
//!
//! let model = GeophoneModel::sm24();
//! let processor = GeophoneProcessor::new(model, 1000.0);
//!
//! // Generate a simple test signal
//! let signal: Vec<f64> = (0..1000).map(|i| {
//!     let t = i as f64 / 1000.0;
//!     0.01 * (2.0 * std::f64::consts::PI * 15.0 * t).sin()
//! }).collect();
//!
//! let pgv = processor.peak_ground_velocity(&signal);
//! assert!(pgv > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Geophone model
// ---------------------------------------------------------------------------

/// Physical model of a moving-coil geophone.
#[derive(Debug, Clone)]
pub struct GeophoneModel {
    /// Natural (resonant) frequency in Hz.
    pub natural_frequency_hz: f64,
    /// Damping ratio (dimensionless, typically 0.3-0.7).
    pub damping_ratio: f64,
    /// Open-circuit sensitivity in V/(m/s).
    pub sensitivity_v_per_ms: f64,
    /// Coil resistance in ohms.
    pub coil_resistance_ohm: f64,
}

impl GeophoneModel {
    /// Sercel SM-24: 10 Hz natural frequency, 28.8 V/(m/s).
    pub fn sm24() -> Self {
        Self {
            natural_frequency_hz: 10.0,
            damping_ratio: 0.56,
            sensitivity_v_per_ms: 28.8,
            coil_resistance_ohm: 395.0,
        }
    }

    /// Geospace GS-11D: 4.5 Hz natural frequency, 32.0 V/(m/s).
    pub fn gs11d() -> Self {
        Self {
            natural_frequency_hz: 4.5,
            damping_ratio: 0.34,
            sensitivity_v_per_ms: 32.0,
            coil_resistance_ohm: 380.0,
        }
    }

    /// Mark Products L-28: 4.5 Hz natural frequency, 31.0 V/(m/s).
    pub fn l28() -> Self {
        Self {
            natural_frequency_hz: 4.5,
            damping_ratio: 0.50,
            sensitivity_v_per_ms: 31.0,
            coil_resistance_ohm: 395.0,
        }
    }

    /// Natural angular frequency omega_n = 2*pi*f_n.
    pub fn omega_n(&self) -> f64 {
        2.0 * PI * self.natural_frequency_hz
    }

    /// Compute the complex transfer function H(j*omega) at a given frequency (Hz).
    ///
    /// H(s) = S * s^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
    ///
    /// Returns (real, imaginary) parts.
    pub fn transfer_function(&self, freq_hz: f64) -> (f64, f64) {
        let omega = 2.0 * PI * freq_hz;
        let omega_n = self.omega_n();
        let s = self.sensitivity_v_per_ms;
        let zeta = self.damping_ratio;

        // s = j*omega, s^2 = -omega^2
        // Numerator: S * (-omega^2) = -S*omega^2 (real)
        let num_re = -s * omega * omega;
        let num_im = 0.0;

        // Denominator: -omega^2 + j*2*zeta*omega_n*omega + omega_n^2
        let den_re = omega_n * omega_n - omega * omega;
        let den_im = 2.0 * zeta * omega_n * omega;

        // Complex division: (a+bi)/(c+di) = ((ac+bd) + (bc-ad)i) / (c^2+d^2)
        let den_mag_sq = den_re * den_re + den_im * den_im;
        if den_mag_sq < 1e-30 {
            return (0.0, 0.0);
        }

        let re = (num_re * den_re + num_im * den_im) / den_mag_sq;
        let im = (num_im * den_re - num_re * den_im) / den_mag_sq;
        (re, im)
    }

    /// Magnitude of the transfer function |H(f)| at a given frequency.
    pub fn frequency_response_magnitude(&self, freq_hz: f64) -> f64 {
        let (re, im) = self.transfer_function(freq_hz);
        (re * re + im * im).sqrt()
    }

    /// Phase of the transfer function arg(H(f)) in radians at a given frequency.
    pub fn frequency_response_phase(&self, freq_hz: f64) -> f64 {
        let (re, im) = self.transfer_function(freq_hz);
        im.atan2(re)
    }
}

// ---------------------------------------------------------------------------
// Geophone processor
// ---------------------------------------------------------------------------

/// Configurable processor for single geophone signal analysis.
#[derive(Debug, Clone)]
pub struct GeophoneProcessor {
    /// Geophone instrument model.
    pub model: GeophoneModel,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Gain applied to raw voltage (amplifier gain).
    pub gain: f64,
}

impl GeophoneProcessor {
    /// Create a new processor with default gain of 1.0.
    pub fn new(model: GeophoneModel, sample_rate_hz: f64) -> Self {
        Self {
            model,
            sample_rate_hz,
            gain: 1.0,
        }
    }

    /// Create a new processor with specified gain.
    pub fn with_gain(model: GeophoneModel, sample_rate_hz: f64, gain: f64) -> Self {
        Self {
            model,
            sample_rate_hz,
            gain,
        }
    }

    // -------------------------------------------------------------------
    // Instrument response correction
    // -------------------------------------------------------------------

    /// Deconvolve the geophone instrument response in the frequency domain.
    ///
    /// Uses water-level regularization to avoid division by near-zero values.
    /// `water_level_db` is the regularization threshold below peak response (e.g., 60.0 dB).
    pub fn deconvolve_instrument_response(
        &self,
        voltage: &[f64],
        water_level_db: f64,
    ) -> Vec<f64> {
        let n = voltage.len();
        if n == 0 {
            return vec![];
        }

        // Pad to next power of 2
        let nfft = n.next_power_of_two();

        // FFT of input
        let (mut spec_re, mut spec_im) = real_fft(voltage, nfft);

        // Find max instrument response for water level
        let df = self.sample_rate_hz / nfft as f64;
        let mut max_h = 0.0f64;
        for k in 0..=nfft / 2 {
            let freq = k as f64 * df;
            let mag = self.model.frequency_response_magnitude(freq);
            if mag > max_h {
                max_h = mag;
            }
        }
        let water_level = max_h * 10.0f64.powf(-water_level_db / 20.0);

        // Divide by transfer function with water-level regularization
        for k in 0..=nfft / 2 {
            let freq = k as f64 * df;
            let (h_re, h_im) = self.model.transfer_function(freq);
            let h_mag_sq = h_re * h_re + h_im * h_im;
            let wl_sq = water_level * water_level;

            // Regularized inverse: H* / max(|H|^2, wl^2)
            let denom = if h_mag_sq > wl_sq { h_mag_sq } else { wl_sq };

            // (a+bi) / (c+di) where H = c+di
            // Multiply spectrum by H*/|H|^2 (regularized)
            let inv_re = h_re / denom;
            let inv_im = -h_im / denom;

            let s_re = spec_re[k];
            let s_im = spec_im[k];

            spec_re[k] = s_re * inv_re - s_im * inv_im;
            spec_im[k] = s_re * inv_im + s_im * inv_re;

            // Mirror for negative frequencies
            if k > 0 && k < nfft / 2 {
                spec_re[nfft - k] = spec_re[k];
                spec_im[nfft - k] = -spec_im[k];
            }
        }

        // Inverse FFT
        let result = real_ifft(&spec_re, &spec_im, nfft);
        result[..n].to_vec()
    }

    /// Design a time-domain FIR inverse filter for instrument response correction.
    ///
    /// Returns filter taps of the specified length.
    pub fn design_inverse_filter(&self, num_taps: usize) -> Vec<f64> {
        if num_taps == 0 {
            return vec![];
        }

        let nfft = num_taps.next_power_of_two().max(64);
        let df = self.sample_rate_hz / nfft as f64;

        let mut h_inv_re = vec![0.0; nfft];
        let mut h_inv_im = vec![0.0; nfft];

        // Find max for water level
        let mut max_h = 0.0f64;
        for k in 0..=nfft / 2 {
            let freq = k as f64 * df;
            let mag = self.model.frequency_response_magnitude(freq);
            if mag > max_h {
                max_h = mag;
            }
        }
        let water_level = max_h * 0.01; // -40 dB water level

        for k in 0..=nfft / 2 {
            let freq = k as f64 * df;
            let (h_re, h_im) = self.model.transfer_function(freq);
            let h_mag_sq = h_re * h_re + h_im * h_im;
            let wl_sq = water_level * water_level;
            let denom = if h_mag_sq > wl_sq { h_mag_sq } else { wl_sq };

            h_inv_re[k] = h_re / denom;
            h_inv_im[k] = -h_im / denom;

            if k > 0 && k < nfft / 2 {
                h_inv_re[nfft - k] = h_inv_re[k];
                h_inv_im[nfft - k] = -h_inv_im[k];
            }
        }

        let taps_full = real_ifft(&h_inv_re, &h_inv_im, nfft);

        // Window and truncate to requested length
        let mut taps = vec![0.0; num_taps];
        let half = num_taps / 2;
        for i in 0..num_taps {
            // Circular shift so filter is centered
            let idx = if i < half {
                nfft - half + i
            } else {
                i - half
            };
            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (num_taps - 1).max(1) as f64).cos();
            taps[i] = taps_full[idx % nfft] * w;
        }

        taps
    }

    // -------------------------------------------------------------------
    // Tilt correction
    // -------------------------------------------------------------------

    /// Estimate tilt angle (radians) from DC offset in the voltage signal.
    ///
    /// V_dc = sensitivity * g * sin(tilt_angle)
    pub fn estimate_tilt_angle(&self, voltage: &[f64]) -> f64 {
        if voltage.is_empty() {
            return 0.0;
        }
        let dc = voltage.iter().copied().sum::<f64>() / voltage.len() as f64;
        let g = 9.80665; // m/s^2
        let max_val = self.model.sensitivity_v_per_ms * g;
        let ratio = (dc / (max_val * self.gain)).clamp(-1.0, 1.0);
        ratio.asin()
    }

    /// Remove tilt-induced DC offset from the voltage signal.
    pub fn remove_tilt_offset(&self, voltage: &[f64]) -> Vec<f64> {
        if voltage.is_empty() {
            return vec![];
        }
        let dc = voltage.iter().copied().sum::<f64>() / voltage.len() as f64;
        voltage.iter().map(|&v| v - dc).collect()
    }

    // -------------------------------------------------------------------
    // Ground motion conversion
    // -------------------------------------------------------------------

    /// Convert voltage to ground velocity (m/s).
    ///
    /// Simple flat-band approximation: v = V / (sensitivity * gain).
    /// For accurate results at low frequencies, use `deconvolve_instrument_response`.
    pub fn voltage_to_velocity(&self, voltage: &[f64]) -> Vec<f64> {
        let scale = 1.0 / (self.model.sensitivity_v_per_ms * self.gain);
        voltage.iter().map(|&v| v * scale).collect()
    }

    /// Integrate velocity to displacement using cumulative trapezoidal rule,
    /// followed by a highpass filter to remove drift.
    ///
    /// `highpass_freq_hz` is the corner frequency of the drift-removal filter.
    pub fn velocity_to_displacement(
        &self,
        velocity: &[f64],
        highpass_freq_hz: f64,
    ) -> Vec<f64> {
        if velocity.is_empty() {
            return vec![];
        }

        let dt = 1.0 / self.sample_rate_hz;
        let n = velocity.len();

        // Cumulative trapezoidal integration
        let mut disp = vec![0.0; n];
        for i in 1..n {
            disp[i] = disp[i - 1] + 0.5 * (velocity[i] + velocity[i - 1]) * dt;
        }

        // Single-pole highpass to remove drift
        highpass_filter(&mut disp, self.sample_rate_hz, highpass_freq_hz);

        disp
    }

    /// Differentiate velocity to acceleration using central differences.
    pub fn velocity_to_acceleration(&self, velocity: &[f64]) -> Vec<f64> {
        let n = velocity.len();
        if n < 2 {
            return vec![0.0; n];
        }

        let dt = 1.0 / self.sample_rate_hz;
        let mut acc = vec![0.0; n];

        // Forward difference at start
        acc[0] = (velocity[1] - velocity[0]) / dt;
        // Central differences
        for i in 1..n - 1 {
            acc[i] = (velocity[i + 1] - velocity[i - 1]) / (2.0 * dt);
        }
        // Backward difference at end
        acc[n - 1] = (velocity[n - 1] - velocity[n - 2]) / dt;

        acc
    }

    /// Peak Ground Velocity (PGV) in m/s.
    pub fn peak_ground_velocity(&self, velocity: &[f64]) -> f64 {
        velocity.iter().map(|v| v.abs()).fold(0.0, f64::max)
    }

    /// Peak Ground Acceleration (PGA) in m/s^2.
    pub fn peak_ground_acceleration(&self, acceleration: &[f64]) -> f64 {
        acceleration.iter().map(|a| a.abs()).fold(0.0, f64::max)
    }

    // -------------------------------------------------------------------
    // Spectral analysis
    // -------------------------------------------------------------------

    /// Power Spectral Density via Welch's method.
    ///
    /// Returns (frequencies_hz, psd) vectors. The PSD is in units^2/Hz.
    /// `segment_len` is the FFT size per segment, `overlap` is the fractional overlap (0..1).
    pub fn welch_psd(
        &self,
        signal: &[f64],
        segment_len: usize,
        overlap: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = signal.len();
        if n == 0 || segment_len == 0 {
            return (vec![], vec![]);
        }

        let seg_len = segment_len.min(n);
        let nfft = seg_len.next_power_of_two();
        let step = ((seg_len as f64 * (1.0 - overlap)).round() as usize).max(1);

        // Hann window
        let window: Vec<f64> = (0..seg_len)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (seg_len - 1).max(1) as f64).cos()))
            .collect();
        let win_power: f64 = window.iter().map(|w| w * w).sum::<f64>();

        let num_bins = nfft / 2 + 1;
        let mut psd = vec![0.0; num_bins];
        let mut num_segments = 0;

        let mut pos = 0;
        while pos + seg_len <= n {
            // Apply window
            let windowed: Vec<f64> = (0..seg_len)
                .map(|i| signal[pos + i] * window[i])
                .collect();

            let (fft_re, fft_im) = real_fft(&windowed, nfft);

            for k in 0..num_bins {
                let power = fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k];
                psd[k] += power;
            }

            num_segments += 1;
            pos += step;
        }

        if num_segments > 0 {
            let scale = 2.0 / (self.sample_rate_hz * win_power * num_segments as f64);
            for k in 0..num_bins {
                psd[k] *= scale;
            }
            // DC and Nyquist bins are not doubled
            psd[0] /= 2.0;
            if num_bins > 1 {
                psd[num_bins - 1] /= 2.0;
            }
        }

        let df = self.sample_rate_hz / nfft as f64;
        let freqs: Vec<f64> = (0..num_bins).map(|k| k as f64 * df).collect();

        (freqs, psd)
    }

    /// Spectrogram (Short-Time FFT with overlap).
    ///
    /// Returns (times_s, frequencies_hz, magnitude_matrix).
    /// Each row of the matrix is one time slice.
    pub fn spectrogram(
        &self,
        signal: &[f64],
        segment_len: usize,
        overlap: f64,
    ) -> (Vec<f64>, Vec<f64>, Vec<Vec<f64>>) {
        let n = signal.len();
        if n == 0 || segment_len == 0 {
            return (vec![], vec![], vec![]);
        }

        let seg_len = segment_len.min(n);
        let nfft = seg_len.next_power_of_two();
        let step = ((seg_len as f64 * (1.0 - overlap)).round() as usize).max(1);
        let num_bins = nfft / 2 + 1;

        let window: Vec<f64> = (0..seg_len)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (seg_len - 1).max(1) as f64).cos()))
            .collect();

        let mut times = vec![];
        let mut matrix = vec![];

        let mut pos = 0;
        while pos + seg_len <= n {
            let windowed: Vec<f64> = (0..seg_len)
                .map(|i| signal[pos + i] * window[i])
                .collect();

            let (fft_re, fft_im) = real_fft(&windowed, nfft);

            let magnitudes: Vec<f64> = (0..num_bins)
                .map(|k| (fft_re[k] * fft_re[k] + fft_im[k] * fft_im[k]).sqrt())
                .collect();

            times.push((pos as f64 + seg_len as f64 / 2.0) / self.sample_rate_hz);
            matrix.push(magnitudes);

            pos += step;
        }

        let df = self.sample_rate_hz / nfft as f64;
        let freqs: Vec<f64> = (0..num_bins).map(|k| k as f64 * df).collect();

        (times, freqs, matrix)
    }

    /// Estimate dominant frequency from PSD peak.
    pub fn dominant_frequency(&self, signal: &[f64], segment_len: usize) -> f64 {
        let (freqs, psd) = self.welch_psd(signal, segment_len, 0.5);
        if freqs.is_empty() {
            return 0.0;
        }

        // Skip DC bin
        let mut max_idx = 1;
        let mut max_val = psd[1];
        for i in 2..psd.len() {
            if psd[i] > max_val {
                max_val = psd[i];
                max_idx = i;
            }
        }
        freqs[max_idx]
    }

    /// RMS velocity in a frequency band [f_low, f_high] Hz.
    pub fn rms_velocity_in_band(
        &self,
        velocity: &[f64],
        segment_len: usize,
        f_low: f64,
        f_high: f64,
    ) -> f64 {
        let (freqs, psd) = self.welch_psd(velocity, segment_len, 0.5);
        if freqs.len() < 2 {
            return 0.0;
        }
        let df = freqs[1] - freqs[0];
        let mut power = 0.0;
        for (i, &f) in freqs.iter().enumerate() {
            if f >= f_low && f <= f_high {
                power += psd[i] * df;
            }
        }
        power.sqrt()
    }

    // -------------------------------------------------------------------
    // Seismic event characterization
    // -------------------------------------------------------------------

    /// Arias intensity: I_a = (pi / (2*g)) * integral(a(t)^2 dt).
    pub fn arias_intensity(&self, acceleration: &[f64]) -> f64 {
        if acceleration.is_empty() {
            return 0.0;
        }
        let dt = 1.0 / self.sample_rate_hz;
        let g = 9.80665;
        let integral: f64 = acceleration.iter().map(|&a| a * a * dt).sum();
        PI / (2.0 * g) * integral
    }

    /// Duration based on 5%-95% of cumulative Arias intensity.
    pub fn significant_duration(&self, acceleration: &[f64]) -> f64 {
        if acceleration.len() < 2 {
            return 0.0;
        }
        let dt = 1.0 / self.sample_rate_hz;

        // Cumulative sum of a^2
        let mut cumulative = vec![0.0; acceleration.len()];
        cumulative[0] = acceleration[0] * acceleration[0] * dt;
        for i in 1..acceleration.len() {
            cumulative[i] = cumulative[i - 1] + acceleration[i] * acceleration[i] * dt;
        }

        let total = *cumulative.last().unwrap();
        if total <= 0.0 {
            return 0.0;
        }

        let threshold_5 = 0.05 * total;
        let threshold_95 = 0.95 * total;

        let mut t5 = 0.0;
        let mut t95 = 0.0;

        for (i, &c) in cumulative.iter().enumerate() {
            if c >= threshold_5 {
                t5 = i as f64 * dt;
                break;
            }
        }
        for (i, &c) in cumulative.iter().enumerate() {
            if c >= threshold_95 {
                t95 = i as f64 * dt;
                break;
            }
        }

        t95 - t5
    }

    // -------------------------------------------------------------------
    // Geophone coupling assessment
    // -------------------------------------------------------------------

    /// Detect resonance frequency shift from expected natural frequency.
    ///
    /// Returns the detected resonance frequency. A shift from the model's
    /// natural frequency indicates poor coupling.
    pub fn detect_resonance_frequency(&self, signal: &[f64], segment_len: usize) -> f64 {
        let (freqs, psd) = self.welch_psd(signal, segment_len, 0.5);
        if freqs.len() < 3 {
            return 0.0;
        }

        // Search around expected natural frequency (+/- 50%)
        let f_n = self.model.natural_frequency_hz;
        let f_low = f_n * 0.5;
        let f_high = f_n * 1.5;

        let mut max_idx = 0;
        let mut max_val = -1.0f64;
        for (i, &f) in freqs.iter().enumerate() {
            if f >= f_low && f <= f_high && psd[i] > max_val {
                max_val = psd[i];
                max_idx = i;
            }
        }

        if max_val > 0.0 {
            freqs[max_idx]
        } else {
            f_n
        }
    }

    /// Coherence between two signals (e.g., horizontal components).
    ///
    /// Returns (frequencies, coherence) where coherence is in [0, 1].
    pub fn coherence(
        &self,
        signal_a: &[f64],
        signal_b: &[f64],
        segment_len: usize,
        overlap: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let n = signal_a.len().min(signal_b.len());
        if n == 0 || segment_len == 0 {
            return (vec![], vec![]);
        }

        let seg_len = segment_len.min(n);
        let nfft = seg_len.next_power_of_two();
        let step = ((seg_len as f64 * (1.0 - overlap)).round() as usize).max(1);
        let num_bins = nfft / 2 + 1;

        let window: Vec<f64> = (0..seg_len)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (seg_len - 1).max(1) as f64).cos()))
            .collect();

        let mut paa = vec![0.0; num_bins]; // |A|^2
        let mut pbb = vec![0.0; num_bins]; // |B|^2
        let mut pab_re = vec![0.0; num_bins]; // Re(A* B)
        let mut pab_im = vec![0.0; num_bins]; // Im(A* B)

        let mut pos = 0;
        while pos + seg_len <= n {
            let wa: Vec<f64> = (0..seg_len).map(|i| signal_a[pos + i] * window[i]).collect();
            let wb: Vec<f64> = (0..seg_len).map(|i| signal_b[pos + i] * window[i]).collect();

            let (a_re, a_im) = real_fft(&wa, nfft);
            let (b_re, b_im) = real_fft(&wb, nfft);

            for k in 0..num_bins {
                paa[k] += a_re[k] * a_re[k] + a_im[k] * a_im[k];
                pbb[k] += b_re[k] * b_re[k] + b_im[k] * b_im[k];
                // Cross-spectrum: A* * B
                pab_re[k] += a_re[k] * b_re[k] + a_im[k] * b_im[k];
                pab_im[k] += -a_im[k] * b_re[k] + a_re[k] * b_im[k];
            }

            pos += step;
        }

        let mut coh = vec![0.0; num_bins];
        for k in 0..num_bins {
            let denom = paa[k] * pbb[k];
            if denom > 1e-30 {
                let cross_mag_sq = pab_re[k] * pab_re[k] + pab_im[k] * pab_im[k];
                coh[k] = cross_mag_sq / denom;
            }
        }

        let df = self.sample_rate_hz / nfft as f64;
        let freqs: Vec<f64> = (0..num_bins).map(|k| k as f64 * df).collect();

        (freqs, coh)
    }

    /// Spectral ratio between two signals (H/V spectral ratio method).
    ///
    /// Returns (frequencies, ratio).
    pub fn spectral_ratio(
        &self,
        signal_h: &[f64],
        signal_v: &[f64],
        segment_len: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let (freqs_h, psd_h) = self.welch_psd(signal_h, segment_len, 0.5);
        let (_, psd_v) = self.welch_psd(signal_v, segment_len, 0.5);

        let n = psd_h.len().min(psd_v.len());
        let ratio: Vec<f64> = (0..n)
            .map(|i| {
                if psd_v[i] > 1e-30 {
                    (psd_h[i] / psd_v[i]).sqrt()
                } else {
                    0.0
                }
            })
            .collect();

        (freqs_h[..n].to_vec(), ratio)
    }

    // -------------------------------------------------------------------
    // Signal quality metrics
    // -------------------------------------------------------------------

    /// SNR estimation from pre-event noise window and signal window.
    ///
    /// `noise_end` is the sample index marking end of noise window.
    /// `signal_start` and `signal_end` define the signal window.
    pub fn estimate_snr(
        &self,
        signal: &[f64],
        noise_end: usize,
        signal_start: usize,
        signal_end: usize,
    ) -> f64 {
        let noise_end = noise_end.min(signal.len());
        let signal_start = signal_start.min(signal.len());
        let signal_end = signal_end.min(signal.len());

        if noise_end == 0 || signal_start >= signal_end {
            return 0.0;
        }

        let noise_power: f64 = signal[..noise_end].iter().map(|&x| x * x).sum::<f64>()
            / noise_end as f64;
        let signal_power: f64 = signal[signal_start..signal_end]
            .iter()
            .map(|&x| x * x)
            .sum::<f64>()
            / (signal_end - signal_start) as f64;

        if noise_power > 1e-30 {
            10.0 * (signal_power / noise_power).log10()
        } else {
            f64::INFINITY
        }
    }

    /// Detect clipping (flat-top saturation) in the signal.
    ///
    /// Returns the fraction of samples that appear clipped (consecutive
    /// identical values at extremes).
    pub fn detect_clipping(&self, signal: &[f64], threshold_fraction: f64) -> f64 {
        if signal.len() < 3 {
            return 0.0;
        }

        let max_abs = signal.iter().map(|v| v.abs()).fold(0.0f64, f64::max);
        if max_abs < 1e-30 {
            return 0.0;
        }

        let clip_threshold = max_abs * threshold_fraction;
        let mut clipped_count = 0usize;

        for i in 1..signal.len() - 1 {
            let val = signal[i].abs();
            if val >= clip_threshold {
                // Check if value is sustained (flat-top)
                let diff_prev = (signal[i] - signal[i - 1]).abs();
                let diff_next = (signal[i + 1] - signal[i]).abs();
                if diff_prev < max_abs * 0.001 || diff_next < max_abs * 0.001 {
                    clipped_count += 1;
                }
            }
        }

        clipped_count as f64 / signal.len() as f64
    }

    /// Characterize the noise floor as RMS of a quiet segment.
    pub fn noise_floor_rms(&self, signal: &[f64], quiet_end: usize) -> f64 {
        let n = quiet_end.min(signal.len());
        if n == 0 {
            return 0.0;
        }
        let power: f64 = signal[..n].iter().map(|&x| x * x).sum::<f64>() / n as f64;
        power.sqrt()
    }
}

// ---------------------------------------------------------------------------
// STA/LTA trigger (standalone function)
// ---------------------------------------------------------------------------

/// Short-Term Average / Long-Term Average trigger for seismic event detection.
///
/// Returns a vector of booleans indicating triggered samples,
/// and the STA/LTA ratio at each sample.
///
/// `sta_len` and `lta_len` are in samples. `threshold` is the trigger ratio.
pub fn sta_lta_trigger(
    signal: &[f64],
    sta_len: usize,
    lta_len: usize,
    threshold: f64,
) -> (Vec<bool>, Vec<f64>) {
    let n = signal.len();
    if n == 0 || sta_len == 0 || lta_len == 0 || lta_len <= sta_len {
        return (vec![false; n], vec![0.0; n]);
    }

    let mut triggers = vec![false; n];
    let mut ratios = vec![0.0; n];

    // Compute squared amplitudes
    let sq: Vec<f64> = signal.iter().map(|&x| x * x).collect();

    for i in lta_len..n {
        let sta: f64 = sq[i - sta_len..i].iter().sum::<f64>() / sta_len as f64;
        let lta: f64 = sq[i - lta_len..i].iter().sum::<f64>() / lta_len as f64;

        if lta > 1e-30 {
            let ratio = sta / lta;
            ratios[i] = ratio;
            triggers[i] = ratio >= threshold;
        }
    }

    (triggers, ratios)
}

/// Pick P-wave onset using STA/LTA with refinement.
///
/// Returns the sample index of the estimated first arrival, or None.
pub fn pick_p_wave_onset(
    signal: &[f64],
    sta_len: usize,
    lta_len: usize,
    threshold: f64,
) -> Option<usize> {
    let (triggers, _) = sta_lta_trigger(signal, sta_len, lta_len, threshold);

    // Find first triggered sample
    for (i, &t) in triggers.iter().enumerate() {
        if t {
            // Refine: walk back to find where energy first rises
            let noise_rms = if i > lta_len {
                let noise_pow: f64 =
                    signal[..i - sta_len].iter().map(|&x| x * x).sum::<f64>()
                        / (i - sta_len) as f64;
                noise_pow.sqrt()
            } else {
                0.0
            };

            let onset_threshold = noise_rms * 3.0;
            for j in (0..i).rev() {
                if signal[j].abs() < onset_threshold {
                    return Some(j + 1);
                }
            }
            return Some(i);
        }
    }

    None
}

// ---------------------------------------------------------------------------
// Internal helper functions
// ---------------------------------------------------------------------------

/// Single-pole highpass filter (in-place).
fn highpass_filter(signal: &mut [f64], sample_rate: f64, cutoff_hz: f64) {
    if signal.is_empty() || cutoff_hz <= 0.0 {
        return;
    }
    let rc = 1.0 / (2.0 * PI * cutoff_hz);
    let dt = 1.0 / sample_rate;
    let alpha = rc / (rc + dt);

    let mut prev_in = signal[0];
    let mut prev_out = signal[0];
    signal[0] = 0.0; // DC removed at start

    for i in 1..signal.len() {
        let cur_in = signal[i];
        let out = alpha * (prev_out + cur_in - prev_in);
        prev_in = cur_in;
        prev_out = out;
        signal[i] = out;
    }
}

/// Real-valued FFT using DIT radix-2 Cooley-Tukey.
///
/// Input is zero-padded to `nfft` (must be power of 2).
/// Returns (real, imag) arrays of length `nfft`.
fn real_fft(signal: &[f64], nfft: usize) -> (Vec<f64>, Vec<f64>) {
    let mut re = vec![0.0; nfft];
    let mut im = vec![0.0; nfft];

    let copy_len = signal.len().min(nfft);
    re[..copy_len].copy_from_slice(&signal[..copy_len]);

    fft_in_place(&mut re, &mut im, false);

    (re, im)
}

/// Inverse FFT, returns real part scaled by 1/N.
fn real_ifft(re: &[f64], im: &[f64], nfft: usize) -> Vec<f64> {
    let mut re_c = re.to_vec();
    let mut im_c = im.to_vec();

    fft_in_place(&mut re_c, &mut im_c, true);

    let scale = 1.0 / nfft as f64;
    re_c.iter().map(|&x| x * scale).collect()
}

/// In-place radix-2 FFT (Cooley-Tukey). `inverse` flag for IFFT.
fn fft_in_place(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT size must be power of 2");
    assert_eq!(re.len(), im.len());

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle = sign * 2.0 * PI / size as f64;
        let w_re = angle.cos();
        let w_im = angle.sin();

        let mut k = 0;
        while k < n {
            let mut tw_re = 1.0;
            let mut tw_im = 0.0;

            for j2 in 0..half {
                let i1 = k + j2;
                let i2 = i1 + half;

                let t_re = tw_re * re[i2] - tw_im * im[i2];
                let t_im = tw_re * im[i2] + tw_im * re[i2];

                re[i2] = re[i1] - t_re;
                im[i2] = im[i1] - t_im;
                re[i1] += t_re;
                im[i1] += t_im;

                let new_tw_re = tw_re * w_re - tw_im * w_im;
                let new_tw_im = tw_re * w_im + tw_im * w_re;
                tw_re = new_tw_re;
                tw_im = new_tw_im;
            }

            k += size;
        }
        size *= 2;
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- GeophoneModel tests ---

    #[test]
    fn test_sm24_model() {
        let m = GeophoneModel::sm24();
        assert!((m.natural_frequency_hz - 10.0).abs() < TOLERANCE);
        assert!((m.damping_ratio - 0.56).abs() < TOLERANCE);
        assert!((m.sensitivity_v_per_ms - 28.8).abs() < TOLERANCE);
        assert!((m.coil_resistance_ohm - 395.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_gs11d_model() {
        let m = GeophoneModel::gs11d();
        assert!((m.natural_frequency_hz - 4.5).abs() < TOLERANCE);
        assert!((m.sensitivity_v_per_ms - 32.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_l28_model() {
        let m = GeophoneModel::l28();
        assert!((m.natural_frequency_hz - 4.5).abs() < TOLERANCE);
        assert!((m.damping_ratio - 0.50).abs() < TOLERANCE);
    }

    #[test]
    fn test_omega_n() {
        let m = GeophoneModel::sm24();
        let expected = 2.0 * PI * 10.0;
        assert!(approx_eq(m.omega_n(), expected, 1e-10));
    }

    #[test]
    fn test_transfer_function_dc() {
        // At DC (0 Hz), numerator is zero -> H = 0
        let m = GeophoneModel::sm24();
        let (re, im) = m.transfer_function(0.0);
        assert!(re.abs() < TOLERANCE);
        assert!(im.abs() < TOLERANCE);
    }

    #[test]
    fn test_transfer_function_high_freq() {
        // Well above resonance, H(f) -> sensitivity
        let m = GeophoneModel::sm24();
        let mag = m.frequency_response_magnitude(1000.0);
        // Should be close to sensitivity
        assert!((mag - m.sensitivity_v_per_ms).abs() < 1.0);
    }

    #[test]
    fn test_transfer_function_at_resonance() {
        // At natural frequency, response peaks for underdamped systems
        let m = GeophoneModel::sm24();
        let mag_at_fn = m.frequency_response_magnitude(m.natural_frequency_hz);
        let mag_below = m.frequency_response_magnitude(m.natural_frequency_hz * 0.5);
        // At resonance should be higher than below
        assert!(mag_at_fn > mag_below);
    }

    #[test]
    fn test_frequency_response_phase() {
        let m = GeophoneModel::sm24();
        let phase = m.frequency_response_phase(100.0);
        // Phase should be a finite number
        assert!(phase.is_finite());
    }

    #[test]
    fn test_frequency_response_magnitude_positive() {
        let m = GeophoneModel::sm24();
        for freq in &[1.0, 5.0, 10.0, 50.0, 100.0, 500.0] {
            let mag = m.frequency_response_magnitude(*freq);
            assert!(mag >= 0.0, "Magnitude should be non-negative at {} Hz", freq);
        }
    }

    // --- GeophoneProcessor basic tests ---

    #[test]
    fn test_processor_creation() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        assert!((p.sample_rate_hz - 1000.0).abs() < TOLERANCE);
        assert!((p.gain - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_processor_with_gain() {
        let p = GeophoneProcessor::with_gain(GeophoneModel::sm24(), 500.0, 10.0);
        assert!((p.gain - 10.0).abs() < TOLERANCE);
    }

    // --- Tilt correction tests ---

    #[test]
    fn test_estimate_tilt_zero() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let signal = vec![0.0; 1000];
        let tilt = p.estimate_tilt_angle(&signal);
        assert!(tilt.abs() < TOLERANCE);
    }

    #[test]
    fn test_estimate_tilt_with_offset() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        // Small DC offset representing tilt
        let signal = vec![1.0; 1000];
        let tilt = p.estimate_tilt_angle(&signal);
        assert!(tilt > 0.0, "Positive offset should give positive tilt");
        assert!(tilt < PI / 2.0, "Tilt should be less than 90 degrees");
    }

    #[test]
    fn test_remove_tilt_offset() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let signal = vec![5.0; 100];
        let corrected = p.remove_tilt_offset(&signal);
        let mean: f64 = corrected.iter().sum::<f64>() / corrected.len() as f64;
        assert!(mean.abs() < TOLERANCE);
    }

    #[test]
    fn test_remove_tilt_preserves_ac() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| 5.0 + (2.0 * PI * 10.0 * i as f64 / 1000.0).sin())
            .collect();
        let corrected = p.remove_tilt_offset(&signal);
        // AC amplitude should be preserved (roughly)
        let max_val = corrected.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_val > 0.5, "AC component should be preserved");
    }

    // --- Ground motion conversion tests ---

    #[test]
    fn test_voltage_to_velocity() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let voltage = vec![28.8; 10]; // sensitivity V/(m/s)
        let vel = p.voltage_to_velocity(&voltage);
        // 28.8 / 28.8 = 1.0 m/s
        for v in &vel {
            assert!(approx_eq(*v, 1.0, 1e-10));
        }
    }

    #[test]
    fn test_voltage_to_velocity_with_gain() {
        let p = GeophoneProcessor::with_gain(GeophoneModel::sm24(), 1000.0, 10.0);
        let voltage = vec![288.0; 5];
        let vel = p.voltage_to_velocity(&voltage);
        for v in &vel {
            assert!(approx_eq(*v, 1.0, 1e-10));
        }
    }

    #[test]
    fn test_velocity_to_displacement() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        // Constant velocity should give linearly increasing displacement (before highpass)
        let velocity = vec![1.0; 100];
        let disp = p.velocity_to_displacement(&velocity, 0.1);
        // After highpass the DC growth is removed, but there should be output
        assert_eq!(disp.len(), 100);
    }

    #[test]
    fn test_velocity_to_acceleration() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        // Linear velocity ramp -> constant acceleration
        let velocity: Vec<f64> = (0..100).map(|i| i as f64 * 0.01).collect();
        let acc = p.velocity_to_acceleration(&velocity);
        assert_eq!(acc.len(), 100);
        // Central differences for linear ramp should give constant
        for i in 2..98 {
            assert!(
                approx_eq(acc[i], 10.0, 0.5),
                "acc[{}] = {}, expected ~10.0",
                i,
                acc[i]
            );
        }
    }

    #[test]
    fn test_velocity_to_acceleration_sine() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 10000.0);
        let freq = 10.0;
        let n = 10000;
        let velocity: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / 10000.0;
                (2.0 * PI * freq * t).sin()
            })
            .collect();
        let acc = p.velocity_to_acceleration(&velocity);
        // Expected: d/dt sin(2*pi*f*t) = 2*pi*f * cos(2*pi*f*t)
        let expected_peak = 2.0 * PI * freq;
        let peak_acc = acc.iter().map(|a| a.abs()).fold(0.0, f64::max);
        assert!(
            (peak_acc - expected_peak).abs() < 1.0,
            "Peak acc {} vs expected {}",
            peak_acc,
            expected_peak
        );
    }

    #[test]
    fn test_peak_ground_velocity() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let vel = vec![0.1, -0.5, 0.3, -0.2, 0.4];
        let pgv = p.peak_ground_velocity(&vel);
        assert!(approx_eq(pgv, 0.5, TOLERANCE));
    }

    #[test]
    fn test_peak_ground_acceleration() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let acc = vec![1.0, -3.0, 2.0, -0.5];
        let pga = p.peak_ground_acceleration(&acc);
        assert!(approx_eq(pga, 3.0, TOLERANCE));
    }

    // --- Spectral analysis tests ---

    #[test]
    fn test_welch_psd_tone() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let freq = 50.0;
        let n = 4096;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / 1000.0).sin())
            .collect();

        let (freqs, psd) = p.welch_psd(&signal, 256, 0.5);
        assert!(!freqs.is_empty());
        assert_eq!(freqs.len(), psd.len());

        // Peak should be near 50 Hz
        let peak_idx = psd
            .iter()
            .enumerate()
            .skip(1) // skip DC
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_freq = freqs[peak_idx];
        assert!(
            (peak_freq - 50.0).abs() < 10.0,
            "Peak at {} Hz, expected ~50 Hz",
            peak_freq
        );
    }

    #[test]
    fn test_welch_psd_empty() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let (f, psd) = p.welch_psd(&[], 256, 0.5);
        assert!(f.is_empty());
        assert!(psd.is_empty());
    }

    #[test]
    fn test_spectrogram() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 2048;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 30.0 * i as f64 / 1000.0).sin())
            .collect();

        let (times, freqs, matrix) = p.spectrogram(&signal, 128, 0.5);
        assert!(!times.is_empty());
        assert!(!freqs.is_empty());
        assert_eq!(matrix.len(), times.len());
        for row in &matrix {
            assert_eq!(row.len(), freqs.len());
        }
    }

    #[test]
    fn test_dominant_frequency() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let freq = 75.0;
        let n = 4096;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / 1000.0).sin())
            .collect();

        let dom_freq = p.dominant_frequency(&signal, 256);
        assert!(
            (dom_freq - freq).abs() < 10.0,
            "Dominant freq {} Hz, expected ~{} Hz",
            dom_freq,
            freq
        );
    }

    #[test]
    fn test_rms_velocity_in_band() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 4096;
        // Signal with energy at 50 Hz
        let signal: Vec<f64> = (0..n)
            .map(|i| 0.5 * (2.0 * PI * 50.0 * i as f64 / 1000.0).sin())
            .collect();

        let rms = p.rms_velocity_in_band(&signal, 256, 40.0, 60.0);
        assert!(rms > 0.0, "RMS in band containing the signal should be > 0");

        // RMS outside the band should be much smaller
        let rms_out = p.rms_velocity_in_band(&signal, 256, 200.0, 300.0);
        assert!(
            rms_out < rms * 0.1,
            "RMS outside band should be much smaller"
        );
    }

    // --- Seismic event characterization tests ---

    #[test]
    fn test_arias_intensity() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 100.0);
        // Constant acceleration of 1 m/s^2 for 1 second
        let acc = vec![1.0; 100];
        let ia = p.arias_intensity(&acc);
        // I_a = pi/(2*g) * integral(1^2 dt) = pi/(2*9.80665) * 1.0
        let expected = PI / (2.0 * 9.80665) * 1.0;
        assert!(
            approx_eq(ia, expected, 0.01),
            "Arias intensity {} vs expected {}",
            ia,
            expected
        );
    }

    #[test]
    fn test_arias_intensity_empty() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 100.0);
        assert!(p.arias_intensity(&[]).abs() < TOLERANCE);
    }

    #[test]
    fn test_significant_duration() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        // Create a burst: silence - strong - silence
        let mut acc = vec![0.001; 500]; // low noise
        acc.extend(vec![1.0; 200]); // strong motion
        acc.extend(vec![0.001; 300]); // low noise

        let dur = p.significant_duration(&acc);
        // Duration should be roughly 0.2s (200 samples at 1000 Hz)
        assert!(
            dur > 0.05 && dur < 0.5,
            "Duration {} should be near 0.2s",
            dur
        );
    }

    // --- STA/LTA trigger tests ---

    #[test]
    fn test_sta_lta_no_event() {
        let signal = vec![0.01; 1000];
        let (triggers, _) = sta_lta_trigger(&signal, 20, 200, 3.0);
        assert_eq!(triggers.len(), 1000);
        // No triggers for constant signal
        assert!(!triggers.iter().any(|&t| t));
    }

    #[test]
    fn test_sta_lta_with_event() {
        let mut signal = vec![0.01; 500];
        // Insert a strong event
        for i in 500..600 {
            signal.push(1.0 * (2.0 * PI * 10.0 * (i - 500) as f64 / 100.0).sin());
        }
        signal.extend(vec![0.01; 400]);

        let (triggers, ratios) = sta_lta_trigger(&signal, 20, 200, 3.0);
        assert_eq!(triggers.len(), signal.len());

        // Should trigger somewhere in the event window
        let has_trigger = triggers[500..700].iter().any(|&t| t);
        assert!(has_trigger, "STA/LTA should trigger during the event");

        // Ratios should be positive
        let max_ratio = ratios.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_ratio > 3.0, "Peak ratio should exceed threshold");
    }

    #[test]
    fn test_sta_lta_empty() {
        let (triggers, ratios) = sta_lta_trigger(&[], 20, 200, 3.0);
        assert!(triggers.is_empty());
        assert!(ratios.is_empty());
    }

    #[test]
    fn test_sta_lta_invalid_lengths() {
        let signal = vec![1.0; 100];
        // sta_len >= lta_len -> no triggers
        let (triggers, _) = sta_lta_trigger(&signal, 200, 20, 3.0);
        assert!(!triggers.iter().any(|&t| t));
    }

    // --- P-wave onset picking ---

    #[test]
    fn test_p_wave_onset_picking() {
        let mut signal = vec![0.001; 500];
        for i in 0..500 {
            signal.push(0.5 * (2.0 * PI * 20.0 * i as f64 / 1000.0).sin());
        }

        let onset = pick_p_wave_onset(&signal, 20, 200, 3.0);
        assert!(onset.is_some(), "Should find P-wave onset");
        let idx = onset.unwrap();
        // Onset should be near sample 500
        assert!(
            idx > 400 && idx < 600,
            "Onset at {}, expected near 500",
            idx
        );
    }

    #[test]
    fn test_p_wave_onset_no_event() {
        let signal = vec![0.001; 1000];
        let onset = pick_p_wave_onset(&signal, 20, 200, 3.0);
        assert!(onset.is_none(), "Should not find onset in quiet signal");
    }

    // --- Instrument response correction tests ---

    #[test]
    fn test_deconvolve_identity() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 512;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 50.0 * i as f64 / 1000.0).sin())
            .collect();

        let corrected = p.deconvolve_instrument_response(&signal, 60.0);
        assert_eq!(corrected.len(), n);
        // Output should be finite
        assert!(corrected.iter().all(|&x| x.is_finite()));
    }

    #[test]
    fn test_deconvolve_empty() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let result = p.deconvolve_instrument_response(&[], 60.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_inverse_filter_design() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let taps = p.design_inverse_filter(64);
        assert_eq!(taps.len(), 64);
        assert!(taps.iter().all(|&t| t.is_finite()));
        // Filter should have non-zero energy
        let energy: f64 = taps.iter().map(|t| t * t).sum();
        assert!(energy > 0.0);
    }

    // --- Coupling assessment tests ---

    #[test]
    fn test_detect_resonance_frequency() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 4096;
        // Generate a signal with energy near the natural frequency
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / 1000.0;
                (2.0 * PI * 10.0 * t).sin() + 0.1 * (2.0 * PI * 50.0 * t).sin()
            })
            .collect();

        let detected = p.detect_resonance_frequency(&signal, 256);
        assert!(
            detected > 5.0 && detected < 15.0,
            "Resonance detected at {} Hz, expected near 10 Hz",
            detected
        );
    }

    #[test]
    fn test_coherence_identical_signals() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 2048;
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 25.0 * i as f64 / 1000.0).sin())
            .collect();

        let (freqs, coh) = p.coherence(&signal, &signal, 256, 0.5);
        assert!(!freqs.is_empty());

        // Coherence of identical signals should be 1.0 at signal frequency
        let max_coh = coh.iter().cloned().fold(0.0f64, f64::max);
        assert!(
            max_coh > 0.99,
            "Max coherence {} should be ~1.0 for identical signals",
            max_coh
        );
    }

    #[test]
    fn test_coherence_uncorrelated() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 4096;
        // Two unrelated tones
        let a: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 30.0 * i as f64 / 1000.0).sin())
            .collect();
        let b: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 170.0 * i as f64 / 1000.0).sin())
            .collect();

        let (_, coh) = p.coherence(&a, &b, 256, 0.5);
        // Average coherence should be low for unrelated signals
        let mean_coh = coh.iter().sum::<f64>() / coh.len() as f64;
        assert!(
            mean_coh < 0.5,
            "Mean coherence {} should be low for unrelated signals",
            mean_coh
        );
    }

    #[test]
    fn test_spectral_ratio() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let n = 2048;
        let h: Vec<f64> = (0..n)
            .map(|i| 2.0 * (2.0 * PI * 25.0 * i as f64 / 1000.0).sin())
            .collect();
        let v: Vec<f64> = (0..n)
            .map(|i| 1.0 * (2.0 * PI * 25.0 * i as f64 / 1000.0).sin())
            .collect();

        let (freqs, ratio) = p.spectral_ratio(&h, &v, 256);
        assert!(!freqs.is_empty());
        // Ratio should be ~2.0 at the signal frequency
        let max_ratio = ratio.iter().cloned().fold(0.0f64, f64::max);
        assert!(
            max_ratio > 1.5,
            "Max spectral ratio {} should reflect amplitude difference",
            max_ratio
        );
    }

    // --- Signal quality metrics tests ---

    #[test]
    fn test_snr_estimation() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let mut signal = vec![0.001; 200]; // noise
        signal.extend(vec![1.0; 200]); // signal

        let snr = p.estimate_snr(&signal, 200, 200, 400);
        // SNR should be high: 10*log10(1.0 / 0.000001) = 60 dB
        assert!(snr > 30.0, "SNR {} should be high", snr);
    }

    #[test]
    fn test_snr_equal_power() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let signal = vec![1.0; 400];
        let snr = p.estimate_snr(&signal, 200, 200, 400);
        assert!(
            snr.abs() < 1.0,
            "SNR {} should be ~0 dB for equal power",
            snr
        );
    }

    #[test]
    fn test_detect_clipping_none() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let signal: Vec<f64> = (0..1000)
            .map(|i| 0.5 * (2.0 * PI * 10.0 * i as f64 / 1000.0).sin())
            .collect();
        let clip_frac = p.detect_clipping(&signal, 0.99);
        assert!(
            clip_frac < 0.01,
            "Clean signal should have minimal clipping: {}",
            clip_frac
        );
    }

    #[test]
    fn test_detect_clipping_present() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        // Create clipped signal
        let mut signal: Vec<f64> = (0..1000)
            .map(|i| 2.0 * (2.0 * PI * 10.0 * i as f64 / 1000.0).sin())
            .collect();
        for s in &mut signal {
            if *s > 1.0 {
                *s = 1.0;
            }
            if *s < -1.0 {
                *s = -1.0;
            }
        }
        let clip_frac = p.detect_clipping(&signal, 0.99);
        assert!(
            clip_frac > 0.01,
            "Clipped signal should show clipping: {}",
            clip_frac
        );
    }

    #[test]
    fn test_noise_floor_rms() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let signal = vec![0.1; 100];
        let nf = p.noise_floor_rms(&signal, 100);
        assert!(approx_eq(nf, 0.1, 0.001));
    }

    #[test]
    fn test_noise_floor_rms_empty() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let nf = p.noise_floor_rms(&[], 0);
        assert!(nf.abs() < TOLERANCE);
    }

    // --- FFT helper tests ---

    #[test]
    fn test_fft_roundtrip() {
        let signal = vec![1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.0];
        let nfft = 8;
        let (re, im) = real_fft(&signal, nfft);
        let recovered = real_ifft(&re, &im, nfft);

        for i in 0..signal.len() {
            assert!(
                approx_eq(recovered[i], signal[i], 1e-10),
                "FFT roundtrip mismatch at {}: {} vs {}",
                i,
                recovered[i],
                signal[i]
            );
        }
    }

    #[test]
    fn test_fft_dc_signal() {
        let signal = vec![5.0; 8];
        let (re, im) = real_fft(&signal, 8);
        // DC bin should be 5*8 = 40
        assert!(approx_eq(re[0], 40.0, 1e-10));
        assert!(im[0].abs() < 1e-10);
        // All other bins should be zero
        for i in 1..8 {
            assert!(re[i].abs() < 1e-10, "re[{}] = {}", i, re[i]);
            assert!(im[i].abs() < 1e-10, "im[{}] = {}", i, im[i]);
        }
    }

    #[test]
    fn test_highpass_filter_removes_dc() {
        let mut signal = vec![10.0; 1000];
        highpass_filter(&mut signal, 1000.0, 1.0);
        // After highpass, DC should be mostly removed
        let tail_mean: f64 =
            signal[500..].iter().sum::<f64>() / 500.0;
        assert!(
            tail_mean.abs() < 1.0,
            "Highpass should remove DC, tail mean = {}",
            tail_mean
        );
    }

    // --- Edge case tests ---

    #[test]
    fn test_empty_velocity_to_displacement() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let result = p.velocity_to_displacement(&[], 1.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_single_sample_acceleration() {
        let p = GeophoneProcessor::new(GeophoneModel::sm24(), 1000.0);
        let acc = p.velocity_to_acceleration(&[1.0]);
        assert_eq!(acc.len(), 1);
        assert!(acc[0].abs() < TOLERANCE);
    }
}
