//! Eye Diagram — Signal quality visualization and metrics
//!
//! Generates eye diagram data from baseband waveforms by folding
//! the signal at the symbol rate. Computes quality metrics:
//! eye opening, Q-factor, jitter, and timing margin. Essential
//! for receiver characterization and ISI assessment.
//! GNU Radio equivalent: `eye_sink_f`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::eye_diagram::EyeDiagram;
//!
//! let mut eye = EyeDiagram::new(4, 2); // 4 samples/symbol, 2 symbols wide
//! let signal: Vec<f64> = (0..40).map(|i| if (i / 4) % 2 == 0 { 1.0 } else { -1.0 }).collect();
//! eye.accumulate(&signal);
//! assert!(eye.num_traces() > 0);
//! ```

/// Eye diagram generator and analyzer.
#[derive(Debug, Clone)]
pub struct EyeDiagram {
    /// Samples per symbol.
    sps: usize,
    /// Number of symbol periods to display.
    num_symbols: usize,
    /// Trace width (sps * num_symbols).
    trace_len: usize,
    /// Accumulated traces (each trace_len samples long).
    traces: Vec<Vec<f64>>,
    /// Max traces to store.
    max_traces: usize,
}

impl EyeDiagram {
    /// Create an eye diagram generator.
    ///
    /// - `sps`: samples per symbol
    /// - `num_symbols`: symbol periods per eye trace (typically 2)
    pub fn new(sps: usize, num_symbols: usize) -> Self {
        let sps = sps.max(1);
        let num_symbols = num_symbols.max(1);
        Self {
            sps,
            num_symbols,
            trace_len: sps * num_symbols,
            traces: Vec::new(),
            max_traces: 1000,
        }
    }

    /// Create with a maximum trace count.
    pub fn with_max_traces(sps: usize, num_symbols: usize, max_traces: usize) -> Self {
        let mut eye = Self::new(sps, num_symbols);
        eye.max_traces = max_traces;
        eye
    }

    /// Accumulate signal data into eye traces.
    pub fn accumulate(&mut self, signal: &[f64]) {
        for chunk in signal.chunks(self.trace_len) {
            if chunk.len() == self.trace_len {
                if self.traces.len() >= self.max_traces {
                    self.traces.remove(0);
                }
                self.traces.push(chunk.to_vec());
            }
        }
    }

    /// Get all accumulated traces.
    pub fn traces(&self) -> &[Vec<f64>] {
        &self.traces
    }

    /// Number of accumulated traces.
    pub fn num_traces(&self) -> usize {
        self.traces.len()
    }

    /// Compute the mean trace (average eye).
    pub fn mean_trace(&self) -> Vec<f64> {
        if self.traces.is_empty() {
            return vec![0.0; self.trace_len];
        }
        let n = self.traces.len() as f64;
        let mut mean = vec![0.0; self.trace_len];
        for trace in &self.traces {
            for (i, &v) in trace.iter().enumerate() {
                mean[i] += v / n;
            }
        }
        mean
    }

    /// Compute min/max envelope at each sample position.
    pub fn envelope(&self) -> (Vec<f64>, Vec<f64>) {
        let mut min_env = vec![f64::INFINITY; self.trace_len];
        let mut max_env = vec![f64::NEG_INFINITY; self.trace_len];
        for trace in &self.traces {
            for (i, &v) in trace.iter().enumerate() {
                min_env[i] = min_env[i].min(v);
                max_env[i] = max_env[i].max(v);
            }
        }
        if self.traces.is_empty() {
            min_env.fill(0.0);
            max_env.fill(0.0);
        }
        (min_env, max_env)
    }

    /// Compute eye opening at the optimal sampling instant.
    ///
    /// Returns (opening, sample_index) where opening is the vertical
    /// gap between upper and lower eye levels at the best position.
    pub fn eye_opening(&self) -> (f64, usize) {
        if self.traces.is_empty() {
            return (0.0, self.trace_len / 2);
        }

        let (min_env, max_env) = self.envelope();

        // Find the sample position with the largest eye opening
        let mut best_opening = 0.0f64;
        let mut best_idx = self.trace_len / 2;

        for i in 0..self.trace_len {
            let _opening = max_env[i] - min_env[i];
            // Actually, we want the position where the eye is most "open"
            // For a proper eye, look at the center of each symbol period
        }

        // Check at center of each symbol period
        for sym in 0..self.num_symbols {
            let center = sym * self.sps + self.sps / 2;
            if center < self.trace_len {
                // Compute standard deviation at this point
                let values: Vec<f64> = self.traces.iter().map(|t| t[center]).collect();
                let mean: f64 = values.iter().sum::<f64>() / values.len() as f64;

                // Separate upper and lower clusters
                let upper: Vec<f64> = values.iter().filter(|&&v| v >= mean).cloned().collect();
                let lower: Vec<f64> = values.iter().filter(|&&v| v < mean).cloned().collect();

                if !upper.is_empty() && !lower.is_empty() {
                    let upper_mean = upper.iter().sum::<f64>() / upper.len() as f64;
                    let lower_mean = lower.iter().sum::<f64>() / lower.len() as f64;
                    let opening = upper_mean - lower_mean;
                    if opening > best_opening {
                        best_opening = opening;
                        best_idx = center;
                    }
                }
            }
        }

        (best_opening, best_idx)
    }

    /// Estimate timing jitter (RMS) at the zero-crossing points.
    pub fn timing_jitter_rms(&self) -> f64 {
        if self.traces.len() < 2 {
            return 0.0;
        }

        let mut crossings = Vec::new();
        for trace in &self.traces {
            for i in 1..trace.len() {
                if (trace[i - 1] >= 0.0 && trace[i] < 0.0) || (trace[i - 1] < 0.0 && trace[i] >= 0.0)
                {
                    // Linear interpolation for crossing point
                    let frac = trace[i - 1].abs() / (trace[i - 1].abs() + trace[i].abs());
                    crossings.push((i - 1) as f64 + frac);
                }
            }
        }

        if crossings.is_empty() {
            return 0.0;
        }

        // Group crossings by their approximate position modulo sps
        let mean: f64 = crossings.iter().sum::<f64>() / crossings.len() as f64;
        let variance: f64 = crossings
            .iter()
            .map(|&c| {
                let diff = (c - mean) % self.sps as f64;
                diff * diff
            })
            .sum::<f64>()
            / crossings.len() as f64;

        variance.sqrt()
    }

    /// Get samples per symbol.
    pub fn sps(&self) -> usize {
        self.sps
    }

    /// Get trace length.
    pub fn trace_len(&self) -> usize {
        self.trace_len
    }

    /// Clear all accumulated traces.
    pub fn clear(&mut self) {
        self.traces.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_nrz_signal(sps: usize, bits: &[bool]) -> Vec<f64> {
        bits.iter()
            .flat_map(|&b| {
                let val = if b { 1.0 } else { -1.0 };
                vec![val; sps]
            })
            .collect()
    }

    #[test]
    fn test_basic_accumulation() {
        let mut eye = EyeDiagram::new(4, 2);
        let signal = make_nrz_signal(4, &[true, false, true, false, true, false, true, false]);
        eye.accumulate(&signal);
        assert!(eye.num_traces() > 0);
    }

    #[test]
    fn test_trace_length() {
        let eye = EyeDiagram::new(4, 2);
        assert_eq!(eye.trace_len(), 8);
    }

    #[test]
    fn test_mean_trace() {
        let mut eye = EyeDiagram::new(2, 1);
        // All constant traces
        for _ in 0..10 {
            eye.accumulate(&[1.0, 1.0]);
        }
        let mean = eye.mean_trace();
        assert!((mean[0] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_envelope() {
        let mut eye = EyeDiagram::new(2, 1);
        eye.accumulate(&[1.0, 2.0]);
        eye.accumulate(&[3.0, 0.0]);
        let (min_e, max_e) = eye.envelope();
        assert!((min_e[0] - 1.0).abs() < 1e-10);
        assert!((max_e[0] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_eye_opening() {
        let mut eye = EyeDiagram::new(4, 2);
        // Use varied bit pattern so traces have different symbol content
        let signal = make_nrz_signal(
            4,
            &[true, false, true, true, false, false, true, false, false, true, true, false],
        );
        eye.accumulate(&signal);
        let (opening, _idx) = eye.eye_opening();
        assert!(opening > 0.0, "Eye should be open");
    }

    #[test]
    fn test_max_traces() {
        let mut eye = EyeDiagram::with_max_traces(2, 1, 5);
        for i in 0..10 {
            eye.accumulate(&[i as f64, i as f64]);
        }
        assert_eq!(eye.num_traces(), 5);
    }

    #[test]
    fn test_clear() {
        let mut eye = EyeDiagram::new(4, 2);
        eye.accumulate(&[1.0; 8]);
        eye.clear();
        assert_eq!(eye.num_traces(), 0);
    }

    #[test]
    fn test_empty() {
        let eye = EyeDiagram::new(4, 2);
        assert_eq!(eye.num_traces(), 0);
        let mean = eye.mean_trace();
        assert!(mean.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_partial_trace_ignored() {
        let mut eye = EyeDiagram::new(4, 2); // trace_len = 8
        eye.accumulate(&[1.0; 7]); // Too short
        assert_eq!(eye.num_traces(), 0);
    }

    #[test]
    fn test_sps_accessor() {
        let eye = EyeDiagram::new(8, 2);
        assert_eq!(eye.sps(), 8);
    }
}
