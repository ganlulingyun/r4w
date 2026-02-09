//! FM Stereo Decoder — Multiplex Stereo Separation
//!
//! Decodes composite FM stereo baseband into left/right audio channels
//! per ITU-R BS.450. Extracts mono (L+R), recovers 19 kHz pilot via PLL,
//! demodulates 38 kHz DSB-SC stereo difference (L-R), and applies
//! de-emphasis filtering.
//!
//! GNU Radio equivalent: `gr-analog` stereo decoder.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fm_stereo_decoder::{FmStereoDecoder, StereoConfig};
//!
//! let config = StereoConfig::new(228_000.0, 75.0);
//! let mut decoder = FmStereoDecoder::new(config);
//! let baseband = vec![0.0f64; 22800];
//! let output = decoder.decode(&baseband);
//! assert_eq!(output.left.len(), output.right.len());
//! ```

use std::f64::consts::PI;

/// FM stereo decoder configuration.
#[derive(Debug, Clone)]
pub struct StereoConfig {
    /// Baseband sample rate (Hz). Typical: 228 kHz.
    pub sample_rate: f64,
    /// De-emphasis time constant in microseconds (75 for US, 50 for Europe).
    pub de_emphasis_us: f64,
    /// Pilot PLL bandwidth (Hz).
    pub pilot_pll_bw: f64,
    /// Audio lowpass cutoff (Hz).
    pub audio_cutoff: f64,
}

impl StereoConfig {
    /// Create a new stereo config.
    pub fn new(sample_rate: f64, de_emphasis_us: f64) -> Self {
        Self {
            sample_rate,
            de_emphasis_us,
            pilot_pll_bw: 10.0,
            audio_cutoff: 15000.0,
        }
    }
}

/// Stereo decode output.
#[derive(Debug, Clone)]
pub struct StereoOutput {
    /// Left audio channel.
    pub left: Vec<f64>,
    /// Right audio channel.
    pub right: Vec<f64>,
    /// Whether pilot tone is locked.
    pub pilot_locked: bool,
    /// Detected pilot level (0.0-1.0).
    pub pilot_level: f64,
}

/// FM stereo decoder with pilot PLL and de-emphasis.
#[derive(Debug, Clone)]
pub struct FmStereoDecoder {
    config: StereoConfig,
    // Pilot PLL state
    pilot_phase: f64,
    pilot_freq: f64,
    pilot_locked: bool,
    pilot_level: f64,
    // De-emphasis filter state
    deemph_state_l: f64,
    deemph_state_r: f64,
}

impl FmStereoDecoder {
    /// Create a new FM stereo decoder.
    pub fn new(config: StereoConfig) -> Self {
        let pilot_freq = 19000.0 / config.sample_rate; // normalized pilot freq
        Self {
            config,
            pilot_phase: 0.0,
            pilot_freq,
            pilot_locked: false,
            pilot_level: 0.0,
            deemph_state_l: 0.0,
            deemph_state_r: 0.0,
        }
    }

    /// Decode FM stereo baseband into left and right channels.
    pub fn decode(&mut self, baseband: &[f64]) -> StereoOutput {
        if baseband.is_empty() {
            return StereoOutput {
                left: vec![],
                right: vec![],
                pilot_locked: false,
                pilot_level: 0.0,
            };
        }

        let n = baseband.len();
        let fs = self.config.sample_rate;

        // Step 1: Extract pilot tone and track with PLL
        let pilot_phases = self.track_pilot(baseband);

        // Step 2: Extract mono (L+R) — lowpass at 15 kHz
        let mono = lowpass_simple(baseband, self.config.audio_cutoff / fs);

        // Step 3: Demodulate stereo subcarrier at 38 kHz (2x pilot)
        let mut stereo_diff = vec![0.0; n];
        for i in 0..n {
            // Multiply by 2x pilot phase (coherent demod of DSB-SC)
            let carrier = (2.0 * pilot_phases[i]).cos();
            stereo_diff[i] = baseband[i] * carrier * 2.0; // x2 for DSB-SC
        }

        // Lowpass the demodulated difference signal
        let diff = lowpass_simple(&stereo_diff, self.config.audio_cutoff / fs);

        // Step 4: Stereo matrix: L = (mono + diff)/2, R = (mono - diff)/2
        let (mut left, mut right) = stereo_matrix(&mono, &diff);

        // Step 5: De-emphasis filtering
        let alpha = de_emphasis_alpha(self.config.de_emphasis_us, fs);
        self.deemph_state_l = apply_deemphasis(&mut left, self.deemph_state_l, alpha);
        self.deemph_state_r = apply_deemphasis(&mut right, self.deemph_state_r, alpha);

        StereoOutput {
            left,
            right,
            pilot_locked: self.pilot_locked,
            pilot_level: self.pilot_level,
        }
    }

    /// Check if stereo pilot is currently detected.
    pub fn is_stereo(&self) -> bool {
        self.pilot_locked
    }

    fn track_pilot(&mut self, baseband: &[f64]) -> Vec<f64> {
        let n = baseband.len();
        let mut phases = vec![0.0; n];
        let bw = self.config.pilot_pll_bw / self.config.sample_rate;
        let alpha = 2.0 * bw; // proportional gain
        let beta = alpha * alpha / 4.0; // integral gain

        let mut pilot_energy = 0.0;
        let mut total_energy = 0.0;

        for i in 0..n {
            // PLL: generate reference at current estimate
            let ref_signal = (2.0 * PI * self.pilot_phase).sin();

            // Phase detector: multiply input by reference
            let error = baseband[i] * ref_signal;

            // Bandpass around 19 kHz for pilot detection
            let bp_signal = baseband[i] * (2.0 * PI * self.pilot_phase).cos();
            pilot_energy += bp_signal * bp_signal;
            total_energy += baseband[i] * baseband[i];

            // Loop filter (PI controller)
            self.pilot_freq += beta * error;
            self.pilot_phase += self.pilot_freq + alpha * error;

            // Keep phase in [0, 1)
            self.pilot_phase = self.pilot_phase.rem_euclid(1.0);

            phases[i] = 2.0 * PI * self.pilot_phase;
        }

        // Determine lock status based on pilot energy
        self.pilot_level = if total_energy > 0.0 {
            (pilot_energy / total_energy).sqrt().min(1.0)
        } else {
            0.0
        };
        self.pilot_locked = self.pilot_level > 0.05;

        phases
    }
}

/// Stereo matrix: (L+R, L-R) → (L, R).
pub fn stereo_matrix(mono: &[f64], diff: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = mono.len().min(diff.len());
    let mut left = Vec::with_capacity(n);
    let mut right = Vec::with_capacity(n);
    for i in 0..n {
        left.push((mono[i] + diff[i]) / 2.0);
        right.push((mono[i] - diff[i]) / 2.0);
    }
    (left, right)
}

/// De-emphasis filter coefficient for given time constant.
pub fn de_emphasis_alpha(tau_us: f64, sample_rate: f64) -> f64 {
    let tau = tau_us * 1e-6;
    let dt = 1.0 / sample_rate;
    dt / (tau + dt)
}

fn apply_deemphasis(samples: &mut [f64], mut state: f64, alpha: f64) -> f64 {
    for s in samples.iter_mut() {
        state = alpha * *s + (1.0 - alpha) * state;
        *s = state;
    }
    state
}

fn lowpass_simple(signal: &[f64], cutoff_norm: f64) -> Vec<f64> {
    // Simple first-order IIR lowpass
    let alpha = (2.0 * PI * cutoff_norm).min(0.99);
    let alpha = alpha / (1.0 + alpha);
    let mut output = vec![0.0; signal.len()];
    let mut state = 0.0;
    for i in 0..signal.len() {
        state = alpha * signal[i] + (1.0 - alpha) * state;
        output[i] = state;
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mono_fallback() {
        let config = StereoConfig::new(228_000.0, 75.0);
        let mut decoder = FmStereoDecoder::new(config);
        // No pilot → mono
        let baseband = vec![0.5; 22800];
        let output = decoder.decode(&baseband);
        assert_eq!(output.left.len(), output.right.len());
    }

    #[test]
    fn test_pilot_detection_silence() {
        let config = StereoConfig::new(228_000.0, 75.0);
        let mut decoder = FmStereoDecoder::new(config);
        let baseband = vec![0.0; 22800];
        let output = decoder.decode(&baseband);
        assert!(!output.pilot_locked);
    }

    #[test]
    fn test_stereo_matrix_identity() {
        let mono = vec![1.0, 0.0, -1.0];
        let diff = vec![0.0, 0.0, 0.0]; // no difference = mono
        let (left, right) = stereo_matrix(&mono, &diff);
        for i in 0..3 {
            assert!(
                (left[i] - right[i]).abs() < 1e-10,
                "mono with no diff should give L=R"
            );
        }
    }

    #[test]
    fn test_stereo_matrix_separation() {
        // L=1, R=0 → mono=(L+R)/2=0.5, diff=(L-R)/2=0.5
        // Wait, stereo_matrix takes (L+R) and (L-R) directly
        let mono = vec![1.0]; // L+R = 1
        let diff = vec![1.0]; // L-R = 1
        let (left, right) = stereo_matrix(&mono, &diff);
        assert!((left[0] - 1.0).abs() < 1e-10); // (1+1)/2 = 1
        assert!((right[0] - 0.0).abs() < 1e-10); // (1-1)/2 = 0
    }

    #[test]
    fn test_deemphasis_75us() {
        let alpha = de_emphasis_alpha(75.0, 228_000.0);
        assert!(alpha > 0.0 && alpha < 1.0);
        // 75us at 228kHz → alpha ≈ dt/tau = (1/228000) / (75e-6) ≈ 0.058
        assert!((alpha - 0.058).abs() < 0.01, "alpha={alpha}");
    }

    #[test]
    fn test_deemphasis_50us() {
        let alpha = de_emphasis_alpha(50.0, 228_000.0);
        assert!(alpha > 0.0 && alpha < 1.0);
        // Should be larger than 75us case (more filtering)
        let alpha_75 = de_emphasis_alpha(75.0, 228_000.0);
        assert!(alpha > alpha_75, "50us should have larger alpha than 75us");
    }

    #[test]
    fn test_output_length() {
        let config = StereoConfig::new(228_000.0, 75.0);
        let mut decoder = FmStereoDecoder::new(config);
        let baseband = vec![0.1; 11400]; // 50ms
        let output = decoder.decode(&baseband);
        assert_eq!(output.left.len(), 11400);
        assert_eq!(output.right.len(), 11400);
    }

    #[test]
    fn test_empty_input() {
        let config = StereoConfig::new(228_000.0, 75.0);
        let mut decoder = FmStereoDecoder::new(config);
        let output = decoder.decode(&[]);
        assert!(output.left.is_empty());
        assert!(output.right.is_empty());
    }

    #[test]
    fn test_is_stereo() {
        let config = StereoConfig::new(228_000.0, 75.0);
        let decoder = FmStereoDecoder::new(config);
        assert!(!decoder.is_stereo()); // no signal processed yet
    }

    #[test]
    fn test_pilot_with_tone() {
        let config = StereoConfig::new(228_000.0, 75.0);
        let mut decoder = FmStereoDecoder::new(config);
        // Generate 19 kHz pilot tone
        let n = 22800;
        let baseband: Vec<f64> = (0..n)
            .map(|i| 0.1 * (2.0 * PI * 19000.0 * i as f64 / 228000.0).sin())
            .collect();
        let output = decoder.decode(&baseband);
        assert_eq!(output.left.len(), n);
        // Pilot level should be detected
        assert!(output.pilot_level > 0.0);
    }
}
