//! Waterfall spectrogram state management

use r4w_core::fft_utils::FftProcessor;
use r4w_core::types::IQSample;

/// Waterfall spectrogram state.
/// Stores FFT history for 2D color-mapped display.
#[derive(Debug)]
pub struct WaterfallState {
    /// FFT processor (reused to avoid allocations)
    fft: FftProcessor,
    /// Power spectrum history [newest..oldest], each row is FFT-shifted
    history: Vec<Vec<f32>>,
    /// Maximum history depth (number of rows)
    max_depth: usize,
    /// FFT size
    fft_size: usize,
    /// Minimum dB for color mapping
    pub min_db: f32,
    /// Maximum dB for color mapping
    pub max_db: f32,
    /// Sample accumulator for FFT
    sample_buffer: Vec<IQSample>,
}

impl WaterfallState {
    /// Create a new waterfall state with specified FFT size and history depth
    pub fn new(fft_size: usize, max_depth: usize) -> Self {
        Self {
            fft: FftProcessor::new(fft_size),
            history: Vec::with_capacity(max_depth),
            max_depth,
            fft_size,
            min_db: -60.0,
            max_db: 0.0,
            sample_buffer: Vec::with_capacity(fft_size),
        }
    }

    /// Add new samples and compute FFT when we have enough.
    /// Automatically manages sample accumulation.
    pub fn push_samples(&mut self, samples: &[IQSample]) {
        // Add samples to buffer
        self.sample_buffer.extend_from_slice(samples);

        // Process complete FFT blocks
        while self.sample_buffer.len() >= self.fft_size {
            // Take fft_size samples for FFT
            let fft_samples: Vec<IQSample> = self.sample_buffer.drain(..self.fft_size).collect();
            self.compute_and_add_fft(&fft_samples);
        }
    }

    /// Compute FFT for a block of samples and add to history
    fn compute_and_add_fft(&mut self, samples: &[IQSample]) {
        if samples.len() < self.fft_size {
            return;
        }

        // Apply Hann window for better frequency resolution
        let windowed: Vec<IQSample> = samples
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let window = 0.5
                    * (1.0
                        - (2.0 * std::f64::consts::PI * i as f64 / (self.fft_size - 1) as f64)
                            .cos());
                IQSample::new(s.re * window, s.im * window)
            })
            .collect();

        // Compute FFT
        let spectrum = self.fft.fft(&windowed);

        // Convert to power dB (f32 for memory efficiency)
        let power_db: Vec<f32> = FftProcessor::power_spectrum_db(&spectrum)
            .into_iter()
            .map(|v| v as f32)
            .collect();

        // FFT shift for display (center DC)
        let shifted = FftProcessor::fft_shift(&power_db);

        // Add to history (newest first)
        self.history.insert(0, shifted);

        // Trim to max depth
        if self.history.len() > self.max_depth {
            self.history.pop();
        }
    }

    /// Get history for rendering [row][freq_bin].
    /// Row 0 is newest, last row is oldest.
    pub fn get_history(&self) -> &[Vec<f32>] {
        &self.history
    }

    /// Get the current (newest) spectrum for real-time display.
    /// Returns None if no FFT has been computed yet.
    pub fn get_current_spectrum(&self) -> Option<&[f32]> {
        self.history.first().map(|v| v.as_slice())
    }

    /// Get number of rows in history
    pub fn depth(&self) -> usize {
        self.history.len()
    }

    /// Get FFT size (number of frequency bins)
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Clear all history
    pub fn clear(&mut self) {
        self.history.clear();
        self.sample_buffer.clear();
    }

    /// Resize FFT size and clear history
    pub fn resize(&mut self, new_fft_size: usize, new_max_depth: usize) {
        self.fft_size = new_fft_size;
        self.max_depth = new_max_depth;
        self.fft = FftProcessor::new(new_fft_size);
        self.sample_buffer = Vec::with_capacity(new_fft_size);
        self.history.clear();
    }

    /// Map a power value in dB to a normalized 0-1 range
    pub fn normalize_power(&self, power_db: f32) -> f32 {
        ((power_db - self.min_db) / (self.max_db - self.min_db)).clamp(0.0, 1.0)
    }
}

/// Viridis-like colormap for waterfall display
pub fn viridis_color(t: f32) -> egui::Color32 {
    // t in range [0, 1], approximation of viridis colormap
    let t = t.clamp(0.0, 1.0);

    // Polynomial approximation of viridis
    let r = (0.267 + 0.003 * t + 0.993 * t * t - 0.263 * t * t * t).clamp(0.0, 1.0);
    let g = (0.004 + 0.874 * t - 0.523 * t * t + 0.645 * t * t * t).clamp(0.0, 1.0);
    let b = (0.329 + 0.899 * t - 2.179 * t * t + 1.952 * t * t * t).clamp(0.0, 1.0);

    egui::Color32::from_rgb(
        (r * 255.0) as u8,
        (g * 255.0) as u8,
        (b * 255.0) as u8,
    )
}

/// Jet colormap (traditional waterfall colors)
pub fn jet_color(t: f32) -> egui::Color32 {
    let t = t.clamp(0.0, 1.0);

    let r = if t < 0.35 {
        0.0
    } else if t < 0.66 {
        (t - 0.35) / 0.31
    } else if t < 0.89 {
        1.0
    } else {
        1.0 - (t - 0.89) / 0.11 * 0.5
    };

    let g = if t < 0.125 {
        0.0
    } else if t < 0.375 {
        (t - 0.125) / 0.25
    } else if t < 0.64 {
        1.0
    } else if t < 0.91 {
        1.0 - (t - 0.64) / 0.27
    } else {
        0.0
    };

    let b = if t < 0.11 {
        0.5 + t / 0.11 * 0.5
    } else if t < 0.34 {
        1.0
    } else if t < 0.65 {
        1.0 - (t - 0.34) / 0.31
    } else {
        0.0
    };

    egui::Color32::from_rgb(
        (r * 255.0) as u8,
        (g * 255.0) as u8,
        (b * 255.0) as u8,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_waterfall_new() {
        let wf = WaterfallState::new(256, 100);
        assert_eq!(wf.fft_size(), 256);
        assert_eq!(wf.depth(), 0);
    }

    #[test]
    fn test_waterfall_push_samples() {
        let mut wf = WaterfallState::new(64, 10);

        // Push enough samples for one FFT
        let samples: Vec<IQSample> = (0..64)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * i as f64 / 64.0;
                IQSample::new(phase.cos(), phase.sin())
            })
            .collect();

        wf.push_samples(&samples);
        assert_eq!(wf.depth(), 1);

        // Push more for second FFT
        wf.push_samples(&samples);
        assert_eq!(wf.depth(), 2);
    }

    #[test]
    fn test_waterfall_max_depth() {
        let mut wf = WaterfallState::new(32, 3);

        let samples: Vec<IQSample> = (0..32).map(|_| IQSample::new(1.0, 0.0)).collect();

        // Push 5 FFT blocks
        for _ in 0..5 {
            wf.push_samples(&samples);
        }

        // Should be capped at max_depth
        assert_eq!(wf.depth(), 3);
    }

    #[test]
    fn test_normalize_power() {
        let wf = WaterfallState::new(64, 10);
        assert_eq!(wf.normalize_power(-60.0), 0.0);
        assert_eq!(wf.normalize_power(0.0), 1.0);
        assert_eq!(wf.normalize_power(-30.0), 0.5);
    }

    #[test]
    fn test_viridis_color() {
        let c0 = viridis_color(0.0);
        let c1 = viridis_color(1.0);

        // Just check it produces valid colors
        assert!(c0.r() < c1.r()); // Viridis goes dark to bright
    }
}
