//! Digital Down Converter (DDC) — NCO + Mixer + Decimation
//!
//! A DDC translates a signal from an intermediate frequency (IF) to baseband
//! and decimates it. The standard DDC chain is: NCO → Complex Mixer → CIC
//! Decimation → FIR Compensation. Used in virtually every SDR receiver front-end.
//! GNU Radio equivalent: combination of `sig_source_c` + `multiply_cc` + `fir_filter_ccf`.
//!
//! ## Processing Chain
//!
//! ```text
//! IF Input → NCO × → CIC Decimate → FIR Compensate → Baseband Output
//!            ↑ exp(-j2πf_c·n/fs)
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::digital_down_converter::{DigitalDownConverter, DdcConfig};
//!
//! let config = DdcConfig {
//!     center_freq_hz: 100_000.0,
//!     sample_rate_hz: 1_000_000.0,
//!     decimation: 10,
//!     output_bandwidth_hz: 50_000.0,
//! };
//! let mut ddc = DigitalDownConverter::new(config);
//! let input = vec![(1.0, 0.0); 100]; // 100 I/Q samples
//! let output = ddc.process(&input);
//! assert_eq!(output.len(), 10); // Decimated by 10
//! ```

use std::f64::consts::PI;

/// DDC configuration.
#[derive(Debug, Clone)]
pub struct DdcConfig {
    /// Center frequency to downconvert (Hz).
    pub center_freq_hz: f64,
    /// Input sample rate (Hz).
    pub sample_rate_hz: f64,
    /// Decimation factor.
    pub decimation: usize,
    /// Desired output bandwidth (Hz) — used for FIR filter design.
    pub output_bandwidth_hz: f64,
}

/// Digital Down Converter.
#[derive(Debug, Clone)]
pub struct DigitalDownConverter {
    config: DdcConfig,
    /// NCO phase accumulator.
    nco_phase: f64,
    /// NCO phase increment per sample.
    nco_phase_inc: f64,
    /// CIC integrator states (N stages).
    cic_integrators_re: Vec<f64>,
    cic_integrators_im: Vec<f64>,
    /// CIC comb states (N stages).
    cic_combs_re: Vec<f64>,
    cic_combs_im: Vec<f64>,
    /// CIC order (number of stages).
    cic_order: usize,
    /// Decimation counter.
    decim_counter: usize,
    /// FIR compensation filter taps.
    fir_taps: Vec<f64>,
    /// FIR state buffer (I).
    fir_state_re: Vec<f64>,
    /// FIR state buffer (Q).
    fir_state_im: Vec<f64>,
}

impl DigitalDownConverter {
    /// Create a new DDC.
    pub fn new(config: DdcConfig) -> Self {
        assert!(config.decimation >= 1, "Decimation must be >= 1");
        assert!(config.sample_rate_hz > 0.0, "Sample rate must be positive");

        let nco_phase_inc = -2.0 * PI * config.center_freq_hz / config.sample_rate_hz;

        // CIC parameters
        let cic_order = 3; // 3-stage CIC is typical

        // Design FIR compensation filter
        let output_rate = config.sample_rate_hz / config.decimation as f64;
        let cutoff_normalized = config.output_bandwidth_hz / output_rate;
        let fir_taps = design_lowpass_fir(cutoff_normalized.min(0.45), 31);

        Self {
            config,
            nco_phase: 0.0,
            nco_phase_inc,
            cic_integrators_re: vec![0.0; cic_order],
            cic_integrators_im: vec![0.0; cic_order],
            cic_combs_re: vec![0.0; cic_order],
            cic_combs_im: vec![0.0; cic_order],
            cic_order,
            decim_counter: 0,
            fir_taps,
            fir_state_re: vec![0.0; 31],
            fir_state_im: vec![0.0; 31],
        }
    }

    /// Process input I/Q samples through the DDC.
    ///
    /// Input: slice of (I, Q) tuples at the input sample rate.
    /// Output: Vec of (I, Q) at the decimated rate.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::new();

        for &(in_re, in_im) in input {
            // Step 1: NCO mixing (frequency shift to baseband)
            let nco_re = self.nco_phase.cos();
            let nco_im = self.nco_phase.sin();
            let mixed_re = in_re * nco_re - in_im * nco_im;
            let mixed_im = in_re * nco_im + in_im * nco_re;

            self.nco_phase += self.nco_phase_inc;
            if self.nco_phase > PI {
                self.nco_phase -= 2.0 * PI;
            } else if self.nco_phase < -PI {
                self.nco_phase += 2.0 * PI;
            }

            // Step 2: CIC integrator stages (run at input rate)
            let mut val_re = mixed_re;
            let mut val_im = mixed_im;
            for stage in 0..self.cic_order {
                self.cic_integrators_re[stage] += val_re;
                self.cic_integrators_im[stage] += val_im;
                val_re = self.cic_integrators_re[stage];
                val_im = self.cic_integrators_im[stage];
            }

            // Step 3: Decimation
            self.decim_counter += 1;
            if self.decim_counter >= self.config.decimation {
                self.decim_counter = 0;

                // CIC comb stages (run at output rate)
                let mut out_re = val_re;
                let mut out_im = val_im;
                for stage in 0..self.cic_order {
                    let prev_re = self.cic_combs_re[stage];
                    let prev_im = self.cic_combs_im[stage];
                    self.cic_combs_re[stage] = out_re;
                    self.cic_combs_im[stage] = out_im;
                    out_re -= prev_re;
                    out_im -= prev_im;
                }

                // Normalize CIC gain
                let cic_gain = (self.config.decimation as f64).powi(self.cic_order as i32);
                out_re /= cic_gain;
                out_im /= cic_gain;

                // Step 4: FIR compensation filter
                let (fir_re, fir_im) = self.apply_fir(out_re, out_im);

                output.push((fir_re, fir_im));
            }
        }

        output
    }

    /// Apply FIR compensation filter.
    fn apply_fir(&mut self, in_re: f64, in_im: f64) -> (f64, f64) {
        let n = self.fir_taps.len();

        // Shift state
        for i in (1..n).rev() {
            self.fir_state_re[i] = self.fir_state_re[i - 1];
            self.fir_state_im[i] = self.fir_state_im[i - 1];
        }
        self.fir_state_re[0] = in_re;
        self.fir_state_im[0] = in_im;

        // Convolve
        let mut out_re = 0.0;
        let mut out_im = 0.0;
        for i in 0..n {
            out_re += self.fir_taps[i] * self.fir_state_re[i];
            out_im += self.fir_taps[i] * self.fir_state_im[i];
        }

        (out_re, out_im)
    }

    /// Set center frequency (retune).
    pub fn set_center_freq(&mut self, freq_hz: f64) {
        self.nco_phase_inc = -2.0 * PI * freq_hz / self.config.sample_rate_hz;
    }

    /// Get current center frequency.
    pub fn center_freq(&self) -> f64 {
        self.config.center_freq_hz
    }

    /// Get output sample rate.
    pub fn output_sample_rate(&self) -> f64 {
        self.config.sample_rate_hz / self.config.decimation as f64
    }

    /// Get decimation factor.
    pub fn decimation(&self) -> usize {
        self.config.decimation
    }

    /// Reset all internal state.
    pub fn reset(&mut self) {
        self.nco_phase = 0.0;
        self.decim_counter = 0;
        self.cic_integrators_re.fill(0.0);
        self.cic_integrators_im.fill(0.0);
        self.cic_combs_re.fill(0.0);
        self.cic_combs_im.fill(0.0);
        self.fir_state_re.fill(0.0);
        self.fir_state_im.fill(0.0);
    }
}

/// Design a lowpass FIR filter using windowed-sinc method.
fn design_lowpass_fir(cutoff_normalized: f64, num_taps: usize) -> Vec<f64> {
    let mid = (num_taps - 1) as f64 / 2.0;
    let fc = cutoff_normalized.clamp(0.001, 0.499);

    let mut taps = Vec::with_capacity(num_taps);
    for i in 0..num_taps {
        let n = i as f64 - mid;
        let sinc = if n.abs() < 1e-10 {
            2.0 * fc
        } else {
            (2.0 * PI * fc * n).sin() / (PI * n)
        };

        // Hamming window
        let window = 0.54 - 0.46 * (2.0 * PI * i as f64 / (num_taps - 1) as f64).cos();
        taps.push(sinc * window);
    }

    // Normalize for unity gain at DC
    let sum: f64 = taps.iter().sum();
    if sum.abs() > 1e-15 {
        for tap in &mut taps {
            *tap /= sum;
        }
    }

    taps
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> DdcConfig {
        DdcConfig {
            center_freq_hz: 100_000.0,
            sample_rate_hz: 1_000_000.0,
            decimation: 10,
            output_bandwidth_hz: 50_000.0,
        }
    }

    #[test]
    fn test_decimation_factor() {
        let ddc = DigitalDownConverter::new(default_config());
        assert_eq!(ddc.decimation(), 10);
    }

    #[test]
    fn test_output_rate() {
        let ddc = DigitalDownConverter::new(default_config());
        assert!((ddc.output_sample_rate() - 100_000.0).abs() < 0.1);
    }

    #[test]
    fn test_output_length() {
        let mut ddc = DigitalDownConverter::new(default_config());
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let output = ddc.process(&input);
        assert_eq!(output.len(), 10, "100 samples / decim 10 = 10 output samples");
    }

    #[test]
    fn test_dc_signal_passthrough() {
        let config = DdcConfig {
            center_freq_hz: 0.0, // No frequency shift
            sample_rate_hz: 100_000.0,
            decimation: 5,
            output_bandwidth_hz: 10_000.0,
        };
        let mut ddc = DigitalDownConverter::new(config);

        // DC signal (all ones)
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 500];
        let output = ddc.process(&input);

        // After settling, output should converge to ~(1, 0)
        if output.len() > 10 {
            let last = output[output.len() - 1];
            assert!(last.0.abs() > 0.1,
                "DC should pass through DDC, got ({:.4}, {:.4})", last.0, last.1);
        }
    }

    #[test]
    fn test_tone_at_center_freq() {
        let config = DdcConfig {
            center_freq_hz: 10_000.0,
            sample_rate_hz: 100_000.0,
            decimation: 5,
            output_bandwidth_hz: 5_000.0,
        };
        let mut ddc = DigitalDownConverter::new(config);

        // Generate tone at 10 kHz (matches center freq)
        let n = 1000;
        let input: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let phase = 2.0 * PI * 10_000.0 * i as f64 / 100_000.0;
                (phase.cos(), phase.sin())
            })
            .collect();

        let output = ddc.process(&input);
        assert_eq!(output.len(), 200);

        // After mixing with -10kHz NCO, the tone should be at DC
        // Check that output has energy
        let energy: f64 = output.iter().map(|&(r, i)| r * r + i * i).sum();
        assert!(energy > 0.0, "Output should have energy");
    }

    #[test]
    fn test_retune() {
        let mut ddc = DigitalDownConverter::new(default_config());
        assert!((ddc.center_freq() - 100_000.0).abs() < 0.1);

        ddc.set_center_freq(200_000.0);
        // NCO phase increment should change
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let output = ddc.process(&input);
        assert_eq!(output.len(), 10);
    }

    #[test]
    fn test_reset() {
        let mut ddc = DigitalDownConverter::new(default_config());
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let _ = ddc.process(&input);

        ddc.reset();
        assert!((ddc.nco_phase).abs() < 1e-10);
    }

    #[test]
    fn test_lowpass_fir_design() {
        let taps = design_lowpass_fir(0.2, 31);
        assert_eq!(taps.len(), 31);

        // DC gain should be ~1
        let dc_gain: f64 = taps.iter().sum();
        assert!((dc_gain - 1.0).abs() < 0.01,
            "DC gain should be ~1, got {:.4}", dc_gain);
    }

    #[test]
    fn test_no_decimation() {
        let config = DdcConfig {
            center_freq_hz: 0.0,
            sample_rate_hz: 100_000.0,
            decimation: 1,
            output_bandwidth_hz: 40_000.0,
        };
        let mut ddc = DigitalDownConverter::new(config);
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 50];
        let output = ddc.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_large_decimation() {
        let config = DdcConfig {
            center_freq_hz: 50_000.0,
            sample_rate_hz: 10_000_000.0,
            decimation: 100,
            output_bandwidth_hz: 25_000.0,
        };
        let mut ddc = DigitalDownConverter::new(config);
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 1000];
        let output = ddc.process(&input);
        assert_eq!(output.len(), 10);
    }
}
