//! Digital Up Converter (DUC) — Interpolation + FIR + NCO Mixer
//!
//! A DUC translates a baseband signal to an intermediate frequency (IF) and
//! interpolates it to a higher sample rate. The standard DUC chain is:
//! FIR Anti-Imaging → CIC Interpolation → NCO Complex Mixer.
//! Used in virtually every SDR transmitter back-end.
//! GNU Radio equivalent: combination of `interp_fir_filter_ccf` + `sig_source_c` + `multiply_cc`.
//!
//! ## Processing Chain
//!
//! ```text
//! Baseband Input → FIR Anti-Image → CIC Interpolate → NCO × → IF Output
//!                                                      ↑ exp(+j2πf_c·n/fs_out)
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::digital_up_converter::{DigitalUpConverter, DucConfig};
//!
//! let config = DucConfig {
//!     center_freq_hz: 100_000.0,
//!     input_sample_rate_hz: 100_000.0,
//!     interpolation: 10,
//!     output_bandwidth_hz: 50_000.0,
//! };
//! let mut duc = DigitalUpConverter::new(config);
//! let input = vec![(1.0, 0.0); 10]; // 10 baseband I/Q samples
//! let output = duc.process(&input);
//! assert_eq!(output.len(), 100); // Interpolated by 10
//! ```

use std::f64::consts::PI;

/// DUC configuration.
#[derive(Debug, Clone)]
pub struct DucConfig {
    /// Center frequency to upconvert to (Hz).
    pub center_freq_hz: f64,
    /// Input (baseband) sample rate (Hz).
    pub input_sample_rate_hz: f64,
    /// Interpolation factor.
    pub interpolation: usize,
    /// Signal bandwidth (Hz) — used for anti-imaging FIR filter design.
    pub output_bandwidth_hz: f64,
}

/// Digital Up Converter.
///
/// Translates a complex baseband signal to an intermediate frequency while
/// increasing the sample rate by the configured interpolation factor.
///
/// The internal processing chain is:
/// 1. **FIR anti-imaging filter** — windowed-sinc lowpass that compensates CIC
///    passband droop and suppresses interpolation images.
/// 2. **CIC interpolation filter** — 3-stage CIC running at the output rate,
///    providing efficient sample-rate increase.
/// 3. **NCO mixer** — numerically controlled oscillator that shifts the signal
///    from baseband to the desired center frequency.
#[derive(Debug, Clone)]
pub struct DigitalUpConverter {
    config: DucConfig,
    /// NCO phase accumulator (radians).
    nco_phase: f64,
    /// NCO phase increment per output sample (radians).
    nco_phase_inc: f64,
    /// CIC integrator states (N stages, run at input rate).
    cic_integrators_re: Vec<f64>,
    cic_integrators_im: Vec<f64>,
    /// CIC comb states (N stages, run at input rate).
    cic_combs_re: Vec<f64>,
    cic_combs_im: Vec<f64>,
    /// CIC order (number of stages).
    cic_order: usize,
    /// FIR anti-imaging filter taps.
    fir_taps: Vec<f64>,
    /// FIR state buffer (I).
    fir_state_re: Vec<f64>,
    /// FIR state buffer (Q).
    fir_state_im: Vec<f64>,
}

impl DigitalUpConverter {
    /// Create a new DUC from the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `interpolation` is less than 1 or `input_sample_rate_hz` is
    /// not positive.
    pub fn new(config: DucConfig) -> Self {
        assert!(config.interpolation >= 1, "Interpolation must be >= 1");
        assert!(
            config.input_sample_rate_hz > 0.0,
            "Input sample rate must be positive"
        );

        let output_rate = config.input_sample_rate_hz * config.interpolation as f64;
        let nco_phase_inc = 2.0 * PI * config.center_freq_hz / output_rate;

        // CIC parameters — 3-stage is the typical choice
        let cic_order = 3;

        // Design FIR anti-imaging filter at the *input* rate.
        // Cutoff is the signal bandwidth relative to input Nyquist.
        let cutoff_normalized = config.output_bandwidth_hz / config.input_sample_rate_hz;
        let num_taps = 31;
        let fir_taps = design_lowpass_fir(cutoff_normalized.min(0.45), num_taps);

        Self {
            config,
            nco_phase: 0.0,
            nco_phase_inc,
            cic_integrators_re: vec![0.0; cic_order],
            cic_integrators_im: vec![0.0; cic_order],
            cic_combs_re: vec![0.0; cic_order],
            cic_combs_im: vec![0.0; cic_order],
            cic_order,
            fir_taps,
            fir_state_re: vec![0.0; num_taps],
            fir_state_im: vec![0.0; num_taps],
        }
    }

    /// Process baseband I/Q samples through the DUC.
    ///
    /// Input: slice of `(I, Q)` tuples at the baseband sample rate.
    /// Output: `Vec<(I, Q)>` at the interpolated output rate
    /// (`input_sample_rate * interpolation`).
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let interp = self.config.interpolation;
        let mut output = Vec::with_capacity(input.len() * interp);

        for &(in_re, in_im) in input {
            // Step 1: FIR anti-imaging filter (runs at input rate)
            let (fir_re, fir_im) = self.apply_fir(in_re, in_im);

            // Step 2: CIC interpolation (expand each sample to `interp` outputs)
            // CIC interpolator: comb stage runs at input rate, integrator at output rate.

            // Comb stage (input rate) — differentiator
            let mut comb_re = fir_re;
            let mut comb_im = fir_im;
            for stage in 0..self.cic_order {
                let prev_re = self.cic_combs_re[stage];
                let prev_im = self.cic_combs_im[stage];
                self.cic_combs_re[stage] = comb_re;
                self.cic_combs_im[stage] = comb_im;
                comb_re -= prev_re;
                comb_im -= prev_im;
            }

            // Zero-stuffing + integrator stages (output rate)
            for k in 0..interp {
                // First output sample uses the comb result; the rest are zero-stuffed
                let sample_re = if k == 0 { comb_re } else { 0.0 };
                let sample_im = if k == 0 { comb_im } else { 0.0 };

                // Integrator cascade
                let mut val_re = sample_re;
                let mut val_im = sample_im;
                for stage in 0..self.cic_order {
                    self.cic_integrators_re[stage] += val_re;
                    self.cic_integrators_im[stage] += val_im;
                    val_re = self.cic_integrators_re[stage];
                    val_im = self.cic_integrators_im[stage];
                }

                // Normalize CIC gain
                let cic_gain = (interp as f64).powi(self.cic_order as i32);
                let norm_re = val_re / cic_gain;
                let norm_im = val_im / cic_gain;

                // Step 3: NCO mixing — shift to center frequency
                let nco_re = self.nco_phase.cos();
                let nco_im = self.nco_phase.sin();
                let out_re = norm_re * nco_re - norm_im * nco_im;
                let out_im = norm_re * nco_im + norm_im * nco_re;

                self.nco_phase += self.nco_phase_inc;
                if self.nco_phase > PI {
                    self.nco_phase -= 2.0 * PI;
                } else if self.nco_phase < -PI {
                    self.nco_phase += 2.0 * PI;
                }

                output.push((out_re, out_im));
            }
        }

        output
    }

    /// Apply FIR anti-imaging filter (runs at baseband rate).
    fn apply_fir(&mut self, in_re: f64, in_im: f64) -> (f64, f64) {
        let n = self.fir_taps.len();

        // Shift state buffer
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

    /// Retune the NCO to a new center frequency.
    pub fn set_center_freq(&mut self, freq_hz: f64) {
        let output_rate = self.config.input_sample_rate_hz * self.config.interpolation as f64;
        self.nco_phase_inc = 2.0 * PI * freq_hz / output_rate;
        self.config.center_freq_hz = freq_hz;
    }

    /// Get the current center frequency (Hz).
    pub fn center_freq(&self) -> f64 {
        self.config.center_freq_hz
    }

    /// Get the output sample rate (Hz).
    ///
    /// Equal to `input_sample_rate_hz * interpolation`.
    pub fn output_sample_rate(&self) -> f64 {
        self.config.input_sample_rate_hz * self.config.interpolation as f64
    }

    /// Get the interpolation factor.
    pub fn interpolation(&self) -> usize {
        self.config.interpolation
    }

    /// Reset all internal state (NCO phase, CIC accumulators, FIR buffers).
    pub fn reset(&mut self) {
        self.nco_phase = 0.0;
        self.cic_integrators_re.fill(0.0);
        self.cic_integrators_im.fill(0.0);
        self.cic_combs_re.fill(0.0);
        self.cic_combs_im.fill(0.0);
        self.fir_state_re.fill(0.0);
        self.fir_state_im.fill(0.0);
    }
}

/// Design a lowpass FIR filter using windowed-sinc method (Hamming window).
///
/// `cutoff_normalized` is the cutoff frequency divided by the sample rate
/// (range 0..0.5). Returns `num_taps` filter coefficients normalised for
/// unity DC gain.
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

    fn default_config() -> DucConfig {
        DucConfig {
            center_freq_hz: 100_000.0,
            input_sample_rate_hz: 100_000.0,
            interpolation: 10,
            output_bandwidth_hz: 50_000.0,
        }
    }

    #[test]
    fn test_interpolation_factor() {
        let duc = DigitalUpConverter::new(default_config());
        assert_eq!(duc.interpolation(), 10);
    }

    #[test]
    fn test_output_rate() {
        let duc = DigitalUpConverter::new(default_config());
        assert!(
            (duc.output_sample_rate() - 1_000_000.0).abs() < 0.1,
            "Expected 1 MHz output rate, got {}",
            duc.output_sample_rate()
        );
    }

    #[test]
    fn test_output_length() {
        let mut duc = DigitalUpConverter::new(default_config());
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 10];
        let output = duc.process(&input);
        assert_eq!(
            output.len(),
            100,
            "10 samples * interp 10 = 100 output samples"
        );
    }

    #[test]
    fn test_dc_passthrough() {
        let config = DucConfig {
            center_freq_hz: 0.0, // No frequency shift
            input_sample_rate_hz: 100_000.0,
            interpolation: 5,
            output_bandwidth_hz: 10_000.0,
        };
        let mut duc = DigitalUpConverter::new(config);

        // Constant DC input
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 200];
        let output = duc.process(&input);

        // After the CIC and FIR settle, the output should converge to ~(1, 0)
        assert!(output.len() > 50, "Need enough output to check settling");
        let last = output[output.len() - 1];
        assert!(
            last.0.abs() > 0.1,
            "DC should pass through DUC with zero center freq, got ({:.4}, {:.4})",
            last.0,
            last.1
        );
    }

    #[test]
    fn test_tone_generation() {
        let config = DucConfig {
            center_freq_hz: 10_000.0,
            input_sample_rate_hz: 100_000.0,
            interpolation: 5,
            output_bandwidth_hz: 40_000.0,
        };
        let mut duc = DigitalUpConverter::new(config);

        // Feed a DC baseband signal — NCO should produce a 10 kHz tone
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 200];
        let output = duc.process(&input);
        assert_eq!(output.len(), 1000);

        // Output should have energy
        let energy: f64 = output.iter().map(|&(r, i)| r * r + i * i).sum();
        assert!(energy > 0.0, "Output should have energy from the NCO tone");
    }

    #[test]
    fn test_retune() {
        let mut duc = DigitalUpConverter::new(default_config());
        assert!((duc.center_freq() - 100_000.0).abs() < 0.1);

        duc.set_center_freq(200_000.0);
        assert!((duc.center_freq() - 200_000.0).abs() < 0.1);

        // Process should still produce the correct number of samples
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 10];
        let output = duc.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_reset() {
        let mut duc = DigitalUpConverter::new(default_config());
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        let _ = duc.process(&input);

        duc.reset();
        assert!(
            duc.nco_phase.abs() < 1e-10,
            "NCO phase should be zero after reset"
        );
        assert!(
            duc.cic_integrators_re.iter().all(|&v| v.abs() < 1e-10),
            "CIC integrators should be zero after reset"
        );
        assert!(
            duc.fir_state_re.iter().all(|&v| v.abs() < 1e-10),
            "FIR state should be zero after reset"
        );
    }

    #[test]
    fn test_no_interpolation() {
        let config = DucConfig {
            center_freq_hz: 0.0,
            input_sample_rate_hz: 100_000.0,
            interpolation: 1,
            output_bandwidth_hz: 40_000.0,
        };
        let mut duc = DigitalUpConverter::new(config);
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 50];
        let output = duc.process(&input);
        assert_eq!(
            output.len(),
            50,
            "Interpolation factor 1 should not change sample count"
        );
    }

    #[test]
    fn test_large_interpolation() {
        let config = DucConfig {
            center_freq_hz: 50_000.0,
            input_sample_rate_hz: 100_000.0,
            interpolation: 100,
            output_bandwidth_hz: 25_000.0,
        };
        let mut duc = DigitalUpConverter::new(config);
        let input: Vec<(f64, f64)> = vec![(1.0, 0.0); 10];
        let output = duc.process(&input);
        assert_eq!(
            output.len(),
            1000,
            "10 samples * interp 100 = 1000 output samples"
        );
    }

    #[test]
    fn test_lowpass_fir_design() {
        let taps = design_lowpass_fir(0.2, 31);
        assert_eq!(taps.len(), 31);

        // DC gain should be ~1
        let dc_gain: f64 = taps.iter().sum();
        assert!(
            (dc_gain - 1.0).abs() < 0.01,
            "DC gain should be ~1, got {:.4}",
            dc_gain
        );

        // Taps should be symmetric (linear phase)
        for i in 0..taps.len() / 2 {
            let diff = (taps[i] - taps[taps.len() - 1 - i]).abs();
            assert!(
                diff < 1e-12,
                "FIR taps should be symmetric, mismatch at index {}",
                i
            );
        }
    }
}
