//! Silence Detector — Voice Activity Detection with energy thresholding
//!
//! Detects silence vs. speech/signal activity based on short-term
//! energy. Provides frame-level decisions, holdover (hangover) to
//! prevent clipping trailing syllables, and adaptive threshold.
//! Used for VOX, DTX, and audio gating.
//! GNU Radio equivalent: `simple_squelch` (audio-optimized).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::silence_detector::SilenceDetector;
//!
//! let mut vad = SilenceDetector::new(-30.0, 5);
//! let silence = vec![0.001; 160];
//! let speech = vec![0.5; 160];
//! assert!(!vad.is_active(&silence));
//! assert!(vad.is_active(&speech));
//! ```

/// Voice Activity / Silence Detector.
#[derive(Debug, Clone)]
pub struct SilenceDetector {
    /// Threshold in dB (relative to full scale).
    threshold_db: f64,
    /// Linear threshold (computed from dB).
    threshold_linear: f64,
    /// Holdover frames (keep active after signal drops).
    holdover: usize,
    /// Current holdover counter.
    holdover_count: usize,
    /// Whether we're currently in active (speech) state.
    active: bool,
    /// Smoothed energy for adaptive tracking.
    smoothed_energy: f64,
    /// Smoothing alpha.
    alpha: f64,
}

impl SilenceDetector {
    /// Create a silence detector.
    ///
    /// - `threshold_db`: energy threshold in dB (e.g., -30.0)
    /// - `holdover_frames`: keep active for this many frames after energy drops
    pub fn new(threshold_db: f64, holdover_frames: usize) -> Self {
        Self {
            threshold_db,
            threshold_linear: 10.0f64.powf(threshold_db / 10.0),
            holdover: holdover_frames,
            holdover_count: 0,
            active: false,
            smoothed_energy: 0.0,
            alpha: 0.1,
        }
    }

    /// Check if a frame is active (has signal above threshold).
    pub fn is_active(&mut self, frame: &[f64]) -> bool {
        let energy = frame_energy(frame);
        self.update(energy)
    }

    /// Update with a pre-computed energy value.
    pub fn update(&mut self, energy: f64) -> bool {
        self.smoothed_energy = self.alpha * energy + (1.0 - self.alpha) * self.smoothed_energy;

        if energy > self.threshold_linear {
            self.active = true;
            self.holdover_count = self.holdover;
        } else if self.holdover_count > 0 {
            self.holdover_count -= 1;
            // Still active during holdover
        } else {
            self.active = false;
        }

        self.active
    }

    /// Process a block of audio, returning per-frame activity decisions.
    ///
    /// Splits the input into frames of `frame_size` and returns a bool for each.
    pub fn process_frames(&mut self, data: &[f64], frame_size: usize) -> Vec<bool> {
        let frame_size = frame_size.max(1);
        data.chunks(frame_size)
            .map(|frame| self.is_active(frame))
            .collect()
    }

    /// Gate a signal: zero out silence portions.
    pub fn gate(&mut self, data: &[f64], frame_size: usize) -> Vec<f64> {
        let frame_size = frame_size.max(1);
        let mut output = Vec::with_capacity(data.len());
        for frame in data.chunks(frame_size) {
            if self.is_active(frame) {
                output.extend_from_slice(frame);
            } else {
                output.extend(std::iter::repeat(0.0).take(frame.len()));
            }
        }
        output
    }

    /// Get current state.
    pub fn is_currently_active(&self) -> bool {
        self.active
    }

    /// Get smoothed energy level.
    pub fn smoothed_energy(&self) -> f64 {
        self.smoothed_energy
    }

    /// Get smoothed energy in dB.
    pub fn smoothed_energy_db(&self) -> f64 {
        10.0 * self.smoothed_energy.max(1e-30).log10()
    }

    /// Set threshold in dB.
    pub fn set_threshold_db(&mut self, db: f64) {
        self.threshold_db = db;
        self.threshold_linear = 10.0f64.powf(db / 10.0);
    }

    /// Get threshold in dB.
    pub fn threshold_db(&self) -> f64 {
        self.threshold_db
    }

    /// Set holdover frames.
    pub fn set_holdover(&mut self, frames: usize) {
        self.holdover = frames;
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.active = false;
        self.holdover_count = 0;
        self.smoothed_energy = 0.0;
    }
}

/// Compute frame energy: mean(x²).
pub fn frame_energy(frame: &[f64]) -> f64 {
    if frame.is_empty() {
        return 0.0;
    }
    frame.iter().map(|&x| x * x).sum::<f64>() / frame.len() as f64
}

/// Compute frame energy in dB.
pub fn frame_energy_db(frame: &[f64]) -> f64 {
    10.0 * frame_energy(frame).max(1e-30).log10()
}

/// Simple threshold-based activity detection (one-shot).
pub fn detect_activity(data: &[f64], frame_size: usize, threshold_db: f64) -> Vec<bool> {
    let threshold = 10.0f64.powf(threshold_db / 10.0);
    let frame_size = frame_size.max(1);
    data.chunks(frame_size)
        .map(|frame| frame_energy(frame) > threshold)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_silence() {
        let mut vad = SilenceDetector::new(-30.0, 0);
        let silence = vec![0.0001; 160];
        assert!(!vad.is_active(&silence));
    }

    #[test]
    fn test_speech() {
        let mut vad = SilenceDetector::new(-30.0, 0);
        let speech = vec![0.5; 160];
        assert!(vad.is_active(&speech));
    }

    #[test]
    fn test_holdover() {
        let mut vad = SilenceDetector::new(-30.0, 3);
        let speech = vec![0.5; 160];
        let silence = vec![0.0001; 160];

        vad.is_active(&speech); // Active
        assert!(vad.is_active(&silence)); // Still active (holdover 3→2)
        assert!(vad.is_active(&silence)); // Still active (2→1)
        assert!(vad.is_active(&silence)); // Still active (1→0)
        assert!(!vad.is_active(&silence)); // Now inactive
    }

    #[test]
    fn test_process_frames() {
        let mut vad = SilenceDetector::new(-20.0, 0);
        let mut data = vec![0.0001; 320]; // 2 frames of silence
        data.extend_from_slice(&vec![0.5; 160]); // 1 frame of speech
        let decisions = vad.process_frames(&data, 160);
        assert_eq!(decisions.len(), 3);
        assert!(!decisions[0]);
        assert!(!decisions[1]);
        assert!(decisions[2]);
    }

    #[test]
    fn test_gate() {
        let mut vad = SilenceDetector::new(-20.0, 0);
        let silence = vec![0.0001; 4];
        let speech = vec![0.5; 4];
        let mut data = silence.clone();
        data.extend(&speech);
        let gated = vad.gate(&data, 4);
        assert_eq!(gated.len(), 8);
        // Silence portion zeroed
        assert!(gated[0..4].iter().all(|&v| v == 0.0));
        // Speech portion passed
        assert!(gated[4..8].iter().all(|&v| (v - 0.5).abs() < 1e-10));
    }

    #[test]
    fn test_frame_energy() {
        let frame = vec![1.0, -1.0, 1.0, -1.0];
        assert!((frame_energy(&frame) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_frame_energy_db() {
        let frame = vec![1.0; 10]; // Energy = 1.0 → 0 dB
        assert!((frame_energy_db(&frame) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_detect_activity() {
        let mut data = vec![0.0001; 100];
        data.extend_from_slice(&vec![0.5; 100]);
        let activity = detect_activity(&data, 100, -20.0);
        assert_eq!(activity.len(), 2);
        assert!(!activity[0]);
        assert!(activity[1]);
    }

    #[test]
    fn test_set_threshold() {
        let mut vad = SilenceDetector::new(-30.0, 0);
        vad.set_threshold_db(-10.0);
        assert!((vad.threshold_db() - (-10.0)).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut vad = SilenceDetector::new(-30.0, 5);
        vad.is_active(&vec![0.5; 160]);
        assert!(vad.is_currently_active());
        vad.reset();
        assert!(!vad.is_currently_active());
    }

    #[test]
    fn test_smoothed_energy() {
        let mut vad = SilenceDetector::new(-30.0, 0);
        // Process multiple frames to build up smoothed energy
        for _ in 0..20 {
            vad.is_active(&vec![1.0; 100]);
        }
        assert!(vad.smoothed_energy() > 0.5);
        assert!(vad.smoothed_energy_db() > -5.0);
    }

    #[test]
    fn test_empty() {
        assert_eq!(frame_energy(&[]), 0.0);
    }
}
