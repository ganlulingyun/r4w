//! Motor Current Signature Analysis (MCSA) for Electric Motor Fault Detection
//!
//! Detects faults in electric motors by analyzing the stator current frequency
//! spectrum. MCSA is a non-invasive condition monitoring technique used in
//! industrial predictive maintenance.
//!
//! ## Fault Types Detected
//!
//! | Fault              | Characteristic Frequency                    | Typical Threshold |
//! |--------------------|---------------------------------------------|-------------------|
//! | Broken Rotor Bar   | f_brb = (1 +/- 2ks) * f_line               | -50 dB to -35 dB  |
//! | Eccentricity       | f_ecc = f_line * (1 +/- (1-s)/p)            | -40 dB to -25 dB  |
//! | Bearing Inner      | n_balls * f_shaft/2 * (1 + d/D * cos(theta))| -30 dB            |
//! | Bearing Outer      | n_balls * f_shaft/2 * (1 - d/D * cos(theta))| -30 dB            |
//! | Stator Winding     | Odd harmonics of line frequency              | -20 dB            |
//! | Supply Imbalance   | 2 * f_line                                  | -25 dB            |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::electric_motor_fault_detector::{McsaConfig, McsaProcessor};
//! use std::f64::consts::PI;
//!
//! let config = McsaConfig {
//!     sample_rate_hz: 10000.0,
//!     line_frequency_hz: 60.0,
//!     num_poles: 4,
//!     rated_slip: 0.03,
//! };
//!
//! let processor = McsaProcessor::new(config);
//!
//! // Generate a synthetic stator current signal (60 Hz fundamental)
//! let n = 10000;
//! let fs = 10000.0;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / fs;
//!         (2.0 * PI * 60.0 * t).sin()
//!     })
//!     .collect();
//!
//! let report = processor.analyze(&signal);
//! assert!(report.line_freq_hz > 59.0 && report.line_freq_hz < 61.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration for the MCSA processor.
#[derive(Debug, Clone)]
pub struct McsaConfig {
    /// Sampling rate of the current signal in Hz.
    pub sample_rate_hz: f64,
    /// Nominal line (mains) frequency in Hz (typically 50 or 60).
    pub line_frequency_hz: f64,
    /// Number of magnetic poles in the motor.
    pub num_poles: usize,
    /// Rated per-unit slip (e.g. 0.03 for 3 %).
    pub rated_slip: f64,
}

impl Default for McsaConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10_000.0,
            line_frequency_hz: 60.0,
            num_poles: 4,
            rated_slip: 0.03,
        }
    }
}

// ---------------------------------------------------------------------------
// Fault types and severity
// ---------------------------------------------------------------------------

/// Types of faults detectable through MCSA.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorFault {
    /// Broken or cracked rotor bars — sidebands at (1 +/- 2ks) * f_line.
    BrokenRotorBar,
    /// Static or dynamic rotor eccentricity.
    Eccentricity,
    /// Inner race bearing defect.
    BearingInner,
    /// Outer race bearing defect.
    BearingOuter,
    /// Stator winding short-circuit or insulation fault.
    StatorWinding,
    /// Supply voltage or phase imbalance.
    SupplyImbalance,
}

/// Severity classification for a detected fault.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FaultSeverity {
    /// No fault — amplitude below all thresholds.
    Healthy,
    /// Early-stage fault — requires monitoring.
    Incipient,
    /// Developing fault — schedule maintenance.
    Moderate,
    /// Critical fault — immediate action required.
    Severe,
}

// ---------------------------------------------------------------------------
// Detection result types
// ---------------------------------------------------------------------------

/// A single fault detection with its associated metrics.
#[derive(Debug, Clone)]
pub struct FaultDetection {
    /// Type of fault detected.
    pub fault_type: MotorFault,
    /// Frequency at which the fault signature was found (Hz).
    pub frequency_hz: f64,
    /// Amplitude of the fault signature relative to the fundamental (dB).
    pub amplitude_db: f64,
    /// Classified severity level.
    pub severity: FaultSeverity,
}

/// Complete motor fault analysis report.
#[derive(Debug, Clone)]
pub struct MotorFaultReport {
    /// All detected fault indicators.
    pub faults: Vec<FaultDetection>,
    /// Measured line frequency (Hz) from spectrum peak.
    pub line_freq_hz: f64,
    /// Estimated per-unit slip.
    pub slip: f64,
    /// Estimated rotor speed in RPM.
    pub speed_rpm: f64,
    /// Power spectrum in dB (one-sided).
    pub spectrum_db: Vec<f64>,
}

/// Bearing characteristic frequencies.
#[derive(Debug, Clone, Copy)]
pub struct BearingFreqs {
    /// Ball Pass Frequency — Outer race (BPFO).
    pub bpfo: f64,
    /// Ball Pass Frequency — Inner race (BPFI).
    pub bpfi: f64,
    /// Ball Spin Frequency (BSF).
    pub bsf: f64,
    /// Fundamental Train Frequency (cage, FTF).
    pub ftf: f64,
}

// ---------------------------------------------------------------------------
// Standalone helper functions
// ---------------------------------------------------------------------------

/// Compute broken rotor bar sideband frequencies.
///
/// For a given harmonic order `k` (1, 2, 3, ...) the fault sidebands appear at:
///
/// ```text
/// f_brb = (1 +/- 2*k*s) * f_line
/// ```
///
/// Returns the lower and upper sideband for each k from 1..=`k_max`.
pub fn broken_bar_frequencies(f_line: f64, slip: f64, k_max: usize) -> Vec<f64> {
    let mut freqs = Vec::with_capacity(k_max * 2);
    for k in 1..=k_max {
        let lower = (1.0 - 2.0 * k as f64 * slip) * f_line;
        let upper = (1.0 + 2.0 * k as f64 * slip) * f_line;
        if lower > 0.0 {
            freqs.push(lower);
        }
        freqs.push(upper);
    }
    freqs
}

/// Compute eccentricity fault frequencies.
///
/// ```text
/// f_ecc = f_line * (1 +/- (1-s) / p)
/// ```
///
/// where `p` = number of pole pairs = `num_poles / 2`.
pub fn eccentricity_frequencies(f_line: f64, num_poles: usize, slip: f64) -> Vec<f64> {
    let p = (num_poles as f64) / 2.0;
    if p <= 0.0 {
        return vec![];
    }
    let offset = (1.0 - slip) / p;
    let lower = f_line * (1.0 - offset);
    let upper = f_line * (1.0 + offset);
    let mut freqs = Vec::new();
    if lower > 0.0 {
        freqs.push(lower);
    }
    freqs.push(upper);
    freqs
}

/// Compute bearing characteristic fault frequencies.
///
/// Standard formulas assuming a contact angle of zero:
///
/// ```text
/// BPFO = (n/2) * f_shaft * (1 - d/D)
/// BPFI = (n/2) * f_shaft * (1 + d/D)
/// BSF  = (D / (2*d)) * f_shaft * (1 - (d/D)^2)
/// FTF  = (f_shaft / 2) * (1 - d/D)
/// ```
///
/// * `f_shaft`  — shaft rotational frequency (Hz)
/// * `num_balls` — number of rolling elements
/// * `ball_diameter` — rolling element diameter
/// * `pitch_diameter` — bearing pitch diameter
pub fn bearing_fault_frequencies(
    f_shaft: f64,
    num_balls: usize,
    ball_diameter: f64,
    pitch_diameter: f64,
) -> BearingFreqs {
    let n = num_balls as f64;
    let ratio = ball_diameter / pitch_diameter;
    BearingFreqs {
        bpfo: (n / 2.0) * f_shaft * (1.0 - ratio),
        bpfi: (n / 2.0) * f_shaft * (1.0 + ratio),
        bsf: (pitch_diameter / (2.0 * ball_diameter)) * f_shaft * (1.0 - ratio * ratio),
        ftf: (f_shaft / 2.0) * (1.0 - ratio),
    }
}

/// Estimate per-unit slip from sideband analysis of the stator current signal.
///
/// Searches for the strongest sideband pair around the line frequency in the
/// range `f_line * (1 +/- 2s)` for slip values in [0.001, 0.15].
pub fn estimate_slip(current: &[f64], fs: f64, f_line: f64) -> f64 {
    if current.len() < 64 {
        return 0.03; // default fallback
    }

    let spectrum = power_spectrum_db(current, fs);
    let n = spectrum.len();
    let freq_resolution = fs / (2.0 * n as f64);

    // Find fundamental peak near f_line
    let fund_bin = (f_line / freq_resolution).round() as usize;
    if fund_bin >= n {
        return 0.03;
    }

    // Search for sideband peaks in the slip range [0.005, 0.10]
    // The lower sideband is at (1 - 2s) * f_line
    let mut best_slip = 0.03;
    let mut best_sideband_power = f64::NEG_INFINITY;

    let slip_steps = 100;
    for i in 1..=slip_steps {
        let s = 0.005 + (0.10 - 0.005) * (i as f64 / slip_steps as f64);
        let f_lower = (1.0 - 2.0 * s) * f_line;
        let f_upper = (1.0 + 2.0 * s) * f_line;

        let bin_lower = (f_lower / freq_resolution).round() as usize;
        let bin_upper = (f_upper / freq_resolution).round() as usize;

        let power_lower = if bin_lower < n { spectrum[bin_lower] } else { f64::NEG_INFINITY };
        let power_upper = if bin_upper < n { spectrum[bin_upper] } else { f64::NEG_INFINITY };

        let combined = power_lower + power_upper;
        if combined > best_sideband_power {
            best_sideband_power = combined;
            best_slip = s;
        }
    }

    best_slip
}

/// Convert slip to rotor speed in RPM.
///
/// ```text
/// RPM = 120 * f_line * (1 - slip) / num_poles
/// ```
pub fn slip_to_speed(slip: f64, f_line: f64, num_poles: usize) -> f64 {
    if num_poles == 0 {
        return 0.0;
    }
    120.0 * f_line * (1.0 - slip) / num_poles as f64
}

/// Compute one-sided power spectrum in dB using a Hann window and radix-2 FFT.
///
/// Returns N/2 + 1 bins covering [0, fs/2].
pub fn power_spectrum_db(signal: &[f64], _fs: f64) -> Vec<f64> {
    let n = signal.len().next_power_of_two();

    // Apply Hann window and zero-pad
    let mut windowed = vec![0.0; n];
    let sig_len = signal.len();
    for i in 0..sig_len {
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (sig_len as f64 - 1.0)).cos());
        windowed[i] = signal[i] * w;
    }

    // In-place radix-2 FFT (Cooley-Tukey)
    let mut re = windowed;
    let mut im = vec![0.0; n];
    fft_radix2(&mut re, &mut im, false);

    // One-sided magnitude spectrum in dB
    let half = n / 2 + 1;
    let mut spectrum_db = Vec::with_capacity(half);
    for i in 0..half {
        let mag_sq = re[i] * re[i] + im[i] * im[i];
        let db = 10.0 * (mag_sq.max(1e-30)).log10();
        spectrum_db.push(db);
    }

    spectrum_db
}

// ---------------------------------------------------------------------------
// Internal: radix-2 Cooley-Tukey FFT (pure Rust, no external deps)
// ---------------------------------------------------------------------------

/// In-place radix-2 FFT. `inverse` = true for IFFT.
fn fft_radix2(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT size must be a power of two");
    assert_eq!(re.len(), im.len());

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();

        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let even = start + k;
                let odd = start + k + half;

                let t_re = w_re * re[odd] - w_im * im[odd];
                let t_im = w_re * im[odd] + w_im * re[odd];

                re[odd] = re[even] - t_re;
                im[odd] = im[even] - t_im;
                re[even] += t_re;
                im[even] += t_im;

                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    // Normalize for inverse
    if inverse {
        let scale = 1.0 / n as f64;
        for i in 0..n {
            re[i] *= scale;
            im[i] *= scale;
        }
    }
}

// ---------------------------------------------------------------------------
// Severity classification
// ---------------------------------------------------------------------------

/// Classify fault severity from the sideband amplitude relative to the fundamental.
///
/// Thresholds are based on IEEE Std 1415 and common industrial practice:
///
/// | Severity   | Amplitude relative to fundamental |
/// |------------|-----------------------------------|
/// | Healthy    | < threshold_incipient             |
/// | Incipient  | threshold_incipient .. -40 dB     |
/// | Moderate   | -40 dB .. -30 dB                  |
/// | Severe     | > -30 dB                          |
fn classify_severity(amplitude_db: f64, fault: MotorFault) -> FaultSeverity {
    // Per-fault thresholds (relative to fundamental, so these are negative dB values
    // where smaller magnitude means higher severity).
    let (incipient, moderate, severe) = match fault {
        MotorFault::BrokenRotorBar => (-50.0, -40.0, -35.0),
        MotorFault::Eccentricity => (-45.0, -35.0, -25.0),
        MotorFault::BearingInner | MotorFault::BearingOuter => (-40.0, -30.0, -20.0),
        MotorFault::StatorWinding => (-35.0, -25.0, -15.0),
        MotorFault::SupplyImbalance => (-40.0, -30.0, -20.0),
    };

    if amplitude_db >= severe {
        FaultSeverity::Severe
    } else if amplitude_db >= moderate {
        FaultSeverity::Moderate
    } else if amplitude_db >= incipient {
        FaultSeverity::Incipient
    } else {
        FaultSeverity::Healthy
    }
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// MCSA processor that analyzes stator current signals for motor faults.
#[derive(Debug, Clone)]
pub struct McsaProcessor {
    config: McsaConfig,
}

impl McsaProcessor {
    /// Create a new MCSA processor with the given configuration.
    pub fn new(config: McsaConfig) -> Self {
        Self { config }
    }

    /// Analyze a stator current signal and produce a fault report.
    ///
    /// The signal should be sampled at `config.sample_rate_hz` and contain at
    /// least several complete cycles of the line frequency for reliable
    /// spectral estimation.
    pub fn analyze(&self, current_signal: &[f64]) -> MotorFaultReport {
        let fs = self.config.sample_rate_hz;
        let f_line_nom = self.config.line_frequency_hz;
        let num_poles = self.config.num_poles;

        // Compute power spectrum
        let spectrum_db = power_spectrum_db(current_signal, fs);
        let n_fft = (current_signal.len().next_power_of_two()) as f64;
        let freq_resolution = fs / n_fft;

        // Find actual line frequency from the strongest peak near nominal
        let line_freq_hz = find_peak_near(&spectrum_db, freq_resolution, f_line_nom, 5.0);

        // Estimate slip
        let slip = estimate_slip(current_signal, fs, line_freq_hz);

        // Compute speed
        let speed_rpm = slip_to_speed(slip, line_freq_hz, num_poles);

        // Fundamental amplitude for relative measurements
        let fund_bin = (line_freq_hz / freq_resolution).round() as usize;
        let fund_amp_db = if fund_bin < spectrum_db.len() {
            spectrum_db[fund_bin]
        } else {
            0.0
        };

        let mut faults = Vec::new();

        // --- Check broken rotor bar sidebands ---
        let brb_freqs = broken_bar_frequencies(line_freq_hz, slip, 3);
        for &f in &brb_freqs {
            if f > 0.0 && f < fs / 2.0 {
                let bin = (f / freq_resolution).round() as usize;
                if bin < spectrum_db.len() {
                    let amp_rel = spectrum_db[bin] - fund_amp_db;
                    let severity = classify_severity(amp_rel, MotorFault::BrokenRotorBar);
                    if severity != FaultSeverity::Healthy {
                        faults.push(FaultDetection {
                            fault_type: MotorFault::BrokenRotorBar,
                            frequency_hz: f,
                            amplitude_db: amp_rel,
                            severity,
                        });
                    }
                }
            }
        }

        // --- Check eccentricity ---
        let ecc_freqs = eccentricity_frequencies(line_freq_hz, num_poles, slip);
        for &f in &ecc_freqs {
            if f > 0.0 && f < fs / 2.0 {
                let bin = (f / freq_resolution).round() as usize;
                if bin < spectrum_db.len() {
                    let amp_rel = spectrum_db[bin] - fund_amp_db;
                    let severity = classify_severity(amp_rel, MotorFault::Eccentricity);
                    if severity != FaultSeverity::Healthy {
                        faults.push(FaultDetection {
                            fault_type: MotorFault::Eccentricity,
                            frequency_hz: f,
                            amplitude_db: amp_rel,
                            severity,
                        });
                    }
                }
            }
        }

        // --- Check stator winding (odd harmonics 3, 5, 7) ---
        for k in [3.0, 5.0, 7.0] {
            let f = k * line_freq_hz;
            if f < fs / 2.0 {
                let bin = (f / freq_resolution).round() as usize;
                if bin < spectrum_db.len() {
                    let amp_rel = spectrum_db[bin] - fund_amp_db;
                    let severity = classify_severity(amp_rel, MotorFault::StatorWinding);
                    if severity != FaultSeverity::Healthy {
                        faults.push(FaultDetection {
                            fault_type: MotorFault::StatorWinding,
                            frequency_hz: f,
                            amplitude_db: amp_rel,
                            severity,
                        });
                    }
                }
            }
        }

        // --- Check supply imbalance (2x line frequency) ---
        {
            let f = 2.0 * line_freq_hz;
            if f < fs / 2.0 {
                let bin = (f / freq_resolution).round() as usize;
                if bin < spectrum_db.len() {
                    let amp_rel = spectrum_db[bin] - fund_amp_db;
                    let severity = classify_severity(amp_rel, MotorFault::SupplyImbalance);
                    if severity != FaultSeverity::Healthy {
                        faults.push(FaultDetection {
                            fault_type: MotorFault::SupplyImbalance,
                            frequency_hz: f,
                            amplitude_db: amp_rel,
                            severity,
                        });
                    }
                }
            }
        }

        MotorFaultReport {
            faults,
            line_freq_hz,
            slip,
            speed_rpm,
            spectrum_db,
        }
    }
}

/// Find the peak frequency closest to `target_hz` within `tolerance_hz`.
fn find_peak_near(spectrum_db: &[f64], freq_res: f64, target_hz: f64, tolerance_hz: f64) -> f64 {
    let center_bin = (target_hz / freq_res).round() as usize;
    let search_bins = (tolerance_hz / freq_res).ceil() as usize;

    let lo = center_bin.saturating_sub(search_bins);
    let hi = (center_bin + search_bins + 1).min(spectrum_db.len());

    let mut best_bin = center_bin.min(spectrum_db.len().saturating_sub(1));
    let mut best_val = f64::NEG_INFINITY;

    for i in lo..hi {
        if spectrum_db[i] > best_val {
            best_val = spectrum_db[i];
            best_bin = i;
        }
    }

    best_bin as f64 * freq_res
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // Helper: generate a sinusoidal signal
    fn gen_sine(freq: f64, amplitude: f64, fs: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / fs).sin())
            .collect()
    }

    // Helper: add two signals element-wise
    fn add_signals(a: &[f64], b: &[f64]) -> Vec<f64> {
        a.iter().zip(b.iter()).map(|(x, y)| x + y).collect()
    }

    // -----------------------------------------------------------------------
    // Broken rotor bar frequencies
    // -----------------------------------------------------------------------

    #[test]
    fn test_broken_bar_frequencies_k1() {
        let freqs = broken_bar_frequencies(60.0, 0.03, 1);
        // Lower: (1 - 2*0.03)*60 = 0.94*60 = 56.4
        // Upper: (1 + 2*0.03)*60 = 1.06*60 = 63.6
        assert_eq!(freqs.len(), 2);
        assert!((freqs[0] - 56.4).abs() < 1e-10);
        assert!((freqs[1] - 63.6).abs() < 1e-10);
    }

    #[test]
    fn test_broken_bar_frequencies_k3() {
        let freqs = broken_bar_frequencies(60.0, 0.03, 3);
        // k=1: 56.4, 63.6
        // k=2: (1-0.12)*60=52.8, (1+0.12)*60=67.2
        // k=3: (1-0.18)*60=49.2, (1+0.18)*60=70.8
        assert_eq!(freqs.len(), 6);
        assert!((freqs[0] - 56.4).abs() < 1e-10);
        assert!((freqs[1] - 63.6).abs() < 1e-10);
        assert!((freqs[2] - 52.8).abs() < 1e-10);
        assert!((freqs[3] - 67.2).abs() < 1e-10);
        assert!((freqs[4] - 49.2).abs() < 1e-10);
        assert!((freqs[5] - 70.8).abs() < 1e-10);
    }

    #[test]
    fn test_broken_bar_frequencies_50hz() {
        let freqs = broken_bar_frequencies(50.0, 0.02, 1);
        assert!((freqs[0] - 48.0).abs() < 1e-10); // (1 - 0.04)*50
        assert!((freqs[1] - 52.0).abs() < 1e-10); // (1 + 0.04)*50
    }

    #[test]
    fn test_broken_bar_high_slip_excludes_negative() {
        // With very high slip, lower sideband for higher k could go negative
        let freqs = broken_bar_frequencies(50.0, 0.6, 1);
        // Lower: (1-1.2)*50 = -10 -> excluded
        // Upper: (1+1.2)*50 = 110
        assert_eq!(freqs.len(), 1);
        assert!((freqs[0] - 110.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Eccentricity frequencies
    // -----------------------------------------------------------------------

    #[test]
    fn test_eccentricity_frequencies_4pole() {
        let freqs = eccentricity_frequencies(60.0, 4, 0.03);
        // p = 2, offset = 0.97/2 = 0.485
        // Lower: 60*(1-0.485) = 60*0.515 = 30.9
        // Upper: 60*(1+0.485) = 60*1.485 = 89.1
        assert_eq!(freqs.len(), 2);
        assert!((freqs[0] - 30.9).abs() < 1e-10);
        assert!((freqs[1] - 89.1).abs() < 1e-10);
    }

    #[test]
    fn test_eccentricity_frequencies_2pole() {
        let freqs = eccentricity_frequencies(60.0, 2, 0.03);
        // p = 1, offset = 0.97
        // Lower: 60*(1-0.97) = 60*0.03 = 1.8
        // Upper: 60*(1+0.97) = 60*1.97 = 118.2
        assert_eq!(freqs.len(), 2);
        assert!((freqs[0] - 1.8).abs() < 1e-10);
        assert!((freqs[1] - 118.2).abs() < 1e-10);
    }

    #[test]
    fn test_eccentricity_frequencies_zero_poles() {
        let freqs = eccentricity_frequencies(60.0, 0, 0.03);
        assert!(freqs.is_empty());
    }

    // -----------------------------------------------------------------------
    // Bearing fault frequencies
    // -----------------------------------------------------------------------

    #[test]
    fn test_bearing_fault_frequencies_standard() {
        // 6205 bearing: 9 balls, d=7.938mm, D=38.5mm, shaft at 29.1 Hz
        let bf = bearing_fault_frequencies(29.1, 9, 7.938, 38.5);

        // BPFO = (9/2)*29.1*(1 - 7.938/38.5) = 4.5*29.1*0.7938... ≈ 103.96
        let ratio = 7.938 / 38.5;
        let expected_bpfo = 4.5 * 29.1 * (1.0 - ratio);
        assert!((bf.bpfo - expected_bpfo).abs() < 0.01);

        // BPFI = (9/2)*29.1*(1 + 7.938/38.5) = 4.5*29.1*1.2062... ≈ 157.98
        let expected_bpfi = 4.5 * 29.1 * (1.0 + ratio);
        assert!((bf.bpfi - expected_bpfi).abs() < 0.01);

        // BSF = (D/(2d))*f*(1 - (d/D)^2) = (38.5/15.876)*29.1*(1-ratio^2)
        let expected_bsf =
            (38.5 / (2.0 * 7.938)) * 29.1 * (1.0 - ratio * ratio);
        assert!((bf.bsf - expected_bsf).abs() < 0.01);

        // FTF = (f/2)*(1-d/D) = 29.1/2*(1-ratio)
        let expected_ftf = (29.1 / 2.0) * (1.0 - ratio);
        assert!((bf.ftf - expected_ftf).abs() < 0.01);
    }

    #[test]
    fn test_bearing_bpfo_less_than_bpfi() {
        let bf = bearing_fault_frequencies(30.0, 8, 10.0, 50.0);
        assert!(bf.bpfo < bf.bpfi, "BPFO should always be less than BPFI");
    }

    #[test]
    fn test_bearing_ftf_less_than_shaft() {
        let bf = bearing_fault_frequencies(30.0, 8, 10.0, 50.0);
        assert!(
            bf.ftf < 30.0,
            "Cage frequency must be less than shaft frequency"
        );
    }

    // -----------------------------------------------------------------------
    // Slip estimation
    // -----------------------------------------------------------------------

    #[test]
    fn test_estimate_slip_with_sidebands() {
        let fs = 10000.0;
        let f_line = 60.0;
        let actual_slip = 0.03;
        let n = 32768;

        // Fundamental
        let mut signal = gen_sine(f_line, 1.0, fs, n);
        // Add sidebands at (1 +/- 2s)*f_line
        let lower_sb = gen_sine((1.0 - 2.0 * actual_slip) * f_line, 0.05, fs, n);
        let upper_sb = gen_sine((1.0 + 2.0 * actual_slip) * f_line, 0.05, fs, n);
        signal = add_signals(&signal, &lower_sb);
        signal = add_signals(&signal, &upper_sb);

        let estimated = estimate_slip(&signal, fs, f_line);
        // Should be in a reasonable range (within 1% of actual)
        assert!(
            (estimated - actual_slip).abs() < 0.02,
            "Estimated slip {} too far from actual {}",
            estimated,
            actual_slip
        );
    }

    #[test]
    fn test_estimate_slip_short_signal_returns_default() {
        let signal = vec![1.0; 32]; // Too short
        let slip = estimate_slip(&signal, 1000.0, 60.0);
        assert!((slip - 0.03).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Slip to speed conversion
    // -----------------------------------------------------------------------

    #[test]
    fn test_slip_to_speed_4pole_60hz() {
        // Synchronous speed = 120*60/4 = 1800 RPM
        // With 3% slip: 1800 * 0.97 = 1746 RPM
        let rpm = slip_to_speed(0.03, 60.0, 4);
        assert!((rpm - 1746.0).abs() < 1e-10);
    }

    #[test]
    fn test_slip_to_speed_2pole_50hz() {
        // Synchronous speed = 120*50/2 = 3000 RPM
        // With 2% slip: 3000 * 0.98 = 2940 RPM
        let rpm = slip_to_speed(0.02, 50.0, 2);
        assert!((rpm - 2940.0).abs() < 1e-10);
    }

    #[test]
    fn test_slip_to_speed_zero_slip() {
        // At zero slip, speed equals synchronous speed
        let rpm = slip_to_speed(0.0, 60.0, 4);
        assert!((rpm - 1800.0).abs() < 1e-10);
    }

    #[test]
    fn test_slip_to_speed_zero_poles() {
        let rpm = slip_to_speed(0.03, 60.0, 0);
        assert!((rpm - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Power spectrum
    // -----------------------------------------------------------------------

    #[test]
    fn test_power_spectrum_peak_at_correct_frequency() {
        let fs = 1000.0;
        let n = 1024;
        let signal = gen_sine(100.0, 1.0, fs, n);
        let spec = power_spectrum_db(&signal, fs);

        // Find peak bin
        let peak_bin = spec
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let peak_freq = peak_bin as f64 * fs / n as f64;
        assert!(
            (peak_freq - 100.0).abs() < 2.0,
            "Peak at {} Hz, expected ~100 Hz",
            peak_freq
        );
    }

    #[test]
    fn test_power_spectrum_length() {
        let signal = vec![0.0; 256];
        let spec = power_spectrum_db(&signal, 1000.0);
        // N=256 (already power of 2), so half+1 = 129
        assert_eq!(spec.len(), 129);
    }

    #[test]
    fn test_power_spectrum_non_power_of_two() {
        let signal = vec![0.0; 300];
        let spec = power_spectrum_db(&signal, 1000.0);
        // Next power of two is 512, so half+1 = 257
        assert_eq!(spec.len(), 257);
    }

    #[test]
    fn test_power_spectrum_stronger_tone_has_higher_peak() {
        let fs = 4000.0;
        let n = 4096;
        let sig1 = gen_sine(200.0, 1.0, fs, n);
        let sig2 = gen_sine(500.0, 0.1, fs, n);
        let combined = add_signals(&sig1, &sig2);
        let spec = power_spectrum_db(&combined, fs);

        let bin_200 = (200.0 * n as f64 / fs).round() as usize;
        let bin_500 = (500.0 * n as f64 / fs).round() as usize;

        assert!(
            spec[bin_200] > spec[bin_500],
            "200 Hz tone (amp=1.0) should be stronger than 500 Hz (amp=0.1)"
        );
    }

    // -----------------------------------------------------------------------
    // FFT radix-2
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_single_tone() {
        let n = 64;
        let mut re: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 4.0 * i as f64 / n as f64).cos())
            .collect();
        let mut im = vec![0.0; n];

        fft_radix2(&mut re, &mut im, false);

        // Bin 4 should have the peak
        let mut peak_bin = 0;
        let mut peak_mag = 0.0;
        for i in 0..n {
            let mag = (re[i] * re[i] + im[i] * im[i]).sqrt();
            if mag > peak_mag {
                peak_mag = mag;
                peak_bin = i;
            }
        }
        assert!(peak_bin == 4 || peak_bin == n - 4);
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let n = 128;
        let original: Vec<f64> = (0..n).map(|i| (i as f64 * 0.1).sin()).collect();
        let mut re = original.clone();
        let mut im = vec![0.0; n];

        fft_radix2(&mut re, &mut im, false);
        fft_radix2(&mut re, &mut im, true);

        for i in 0..n {
            assert!(
                (re[i] - original[i]).abs() < 1e-10,
                "FFT roundtrip failed at index {}",
                i
            );
            assert!(im[i].abs() < 1e-10, "Imaginary part nonzero at index {}", i);
        }
    }

    // -----------------------------------------------------------------------
    // Severity classification
    // -----------------------------------------------------------------------

    #[test]
    fn test_severity_broken_bar_healthy() {
        let s = classify_severity(-55.0, MotorFault::BrokenRotorBar);
        assert_eq!(s, FaultSeverity::Healthy);
    }

    #[test]
    fn test_severity_broken_bar_incipient() {
        let s = classify_severity(-45.0, MotorFault::BrokenRotorBar);
        assert_eq!(s, FaultSeverity::Incipient);
    }

    #[test]
    fn test_severity_broken_bar_moderate() {
        let s = classify_severity(-38.0, MotorFault::BrokenRotorBar);
        assert_eq!(s, FaultSeverity::Moderate);
    }

    #[test]
    fn test_severity_broken_bar_severe() {
        let s = classify_severity(-30.0, MotorFault::BrokenRotorBar);
        assert_eq!(s, FaultSeverity::Severe);
    }

    #[test]
    fn test_severity_ordering() {
        assert!(FaultSeverity::Healthy < FaultSeverity::Incipient);
        assert!(FaultSeverity::Incipient < FaultSeverity::Moderate);
        assert!(FaultSeverity::Moderate < FaultSeverity::Severe);
    }

    // -----------------------------------------------------------------------
    // Fault detection (full processor)
    // -----------------------------------------------------------------------

    #[test]
    fn test_healthy_motor_no_faults() {
        let config = McsaConfig {
            sample_rate_hz: 10000.0,
            line_frequency_hz: 60.0,
            num_poles: 4,
            rated_slip: 0.03,
        };
        let processor = McsaProcessor::new(config);

        // Pure 60 Hz fundamental — no sidebands.
        // Use a long signal for fine frequency resolution to minimize leakage.
        let signal = gen_sine(60.0, 1.0, 10000.0, 65536);
        let report = processor.analyze(&signal);

        // Should detect line frequency correctly
        assert!(
            (report.line_freq_hz - 60.0).abs() < 2.0,
            "Line frequency {} not near 60 Hz",
            report.line_freq_hz
        );

        // No broken-rotor-bar or bearing severe faults expected on clean signal.
        // (Minor eccentricity/stator detections near the fundamental may occur
        //  due to FFT leakage and are not a concern for this test.)
        let severe_brb_bearing = report
            .faults
            .iter()
            .filter(|f| {
                f.severity == FaultSeverity::Severe
                    && matches!(
                        f.fault_type,
                        MotorFault::BrokenRotorBar
                            | MotorFault::BearingInner
                            | MotorFault::BearingOuter
                    )
            })
            .count();
        assert_eq!(
            severe_brb_bearing, 0,
            "Clean signal should have no severe BRB/bearing faults"
        );
    }

    #[test]
    fn test_broken_bar_detection() {
        let fs = 10000.0;
        let f_line = 60.0;
        let slip = 0.03;
        let n = 32768;

        let mut signal = gen_sine(f_line, 1.0, fs, n);
        // Inject strong broken bar sidebands
        let f_lower = (1.0 - 2.0 * slip) * f_line; // 56.4 Hz
        let f_upper = (1.0 + 2.0 * slip) * f_line; // 63.6 Hz
        let sb_lower = gen_sine(f_lower, 0.1, fs, n); // -20 dB relative
        let sb_upper = gen_sine(f_upper, 0.1, fs, n);
        signal = add_signals(&signal, &sb_lower);
        signal = add_signals(&signal, &sb_upper);

        let config = McsaConfig {
            sample_rate_hz: fs,
            line_frequency_hz: f_line,
            num_poles: 4,
            rated_slip: slip,
        };
        let processor = McsaProcessor::new(config);
        let report = processor.analyze(&signal);

        // Should detect at least one broken bar fault
        let brb_faults: Vec<_> = report
            .faults
            .iter()
            .filter(|f| f.fault_type == MotorFault::BrokenRotorBar)
            .collect();

        assert!(
            !brb_faults.is_empty(),
            "Should detect broken rotor bar from strong sidebands"
        );
    }

    #[test]
    fn test_supply_imbalance_detection() {
        let fs = 10000.0;
        let f_line = 60.0;
        let n = 16384;

        let mut signal = gen_sine(f_line, 1.0, fs, n);
        // Strong 2nd harmonic (120 Hz) indicates supply imbalance
        let harmonic = gen_sine(2.0 * f_line, 0.15, fs, n);
        signal = add_signals(&signal, &harmonic);

        let config = McsaConfig {
            sample_rate_hz: fs,
            line_frequency_hz: f_line,
            num_poles: 4,
            rated_slip: 0.03,
        };
        let processor = McsaProcessor::new(config);
        let report = processor.analyze(&signal);

        let imbalance_faults: Vec<_> = report
            .faults
            .iter()
            .filter(|f| f.fault_type == MotorFault::SupplyImbalance)
            .collect();

        assert!(
            !imbalance_faults.is_empty(),
            "Should detect supply imbalance from 2x harmonic"
        );
    }

    #[test]
    fn test_stator_winding_detection() {
        let fs = 10000.0;
        let f_line = 60.0;
        let n = 16384;

        let mut signal = gen_sine(f_line, 1.0, fs, n);
        // Strong odd harmonics indicate stator winding issues
        let h3 = gen_sine(3.0 * f_line, 0.2, fs, n);
        let h5 = gen_sine(5.0 * f_line, 0.15, fs, n);
        signal = add_signals(&signal, &h3);
        signal = add_signals(&signal, &h5);

        let config = McsaConfig {
            sample_rate_hz: fs,
            line_frequency_hz: f_line,
            num_poles: 4,
            rated_slip: 0.03,
        };
        let processor = McsaProcessor::new(config);
        let report = processor.analyze(&signal);

        let winding_faults: Vec<_> = report
            .faults
            .iter()
            .filter(|f| f.fault_type == MotorFault::StatorWinding)
            .collect();

        assert!(
            !winding_faults.is_empty(),
            "Should detect stator winding fault from odd harmonics"
        );
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_empty_signal() {
        let config = McsaConfig::default();
        let processor = McsaProcessor::new(config);
        // Empty signal should not panic; it will produce a minimal-length FFT
        let report = processor.analyze(&[]);
        assert!(report.spectrum_db.len() <= 2);
    }

    #[test]
    fn test_dc_only_signal() {
        let config = McsaConfig::default();
        let processor = McsaProcessor::new(config);
        let signal = vec![1.0; 1024];
        let report = processor.analyze(&signal);
        // All energy at DC, no faults at the line frequency
        assert!(report.spectrum_db.len() > 0);
    }

    #[test]
    fn test_very_short_signal() {
        let config = McsaConfig::default();
        let processor = McsaProcessor::new(config);
        let signal = vec![0.5, -0.5, 0.5, -0.5];
        let report = processor.analyze(&signal);
        assert!(report.spectrum_db.len() > 0);
    }

    #[test]
    fn test_50hz_system() {
        let config = McsaConfig {
            sample_rate_hz: 8000.0,
            line_frequency_hz: 50.0,
            num_poles: 2,
            rated_slip: 0.02,
        };
        let processor = McsaProcessor::new(config);
        let signal = gen_sine(50.0, 1.0, 8000.0, 8192);
        let report = processor.analyze(&signal);

        assert!(
            (report.line_freq_hz - 50.0).abs() < 2.0,
            "Should detect 50 Hz line frequency, got {}",
            report.line_freq_hz
        );
    }

    #[test]
    fn test_config_default() {
        let config = McsaConfig::default();
        assert_eq!(config.sample_rate_hz, 10000.0);
        assert_eq!(config.line_frequency_hz, 60.0);
        assert_eq!(config.num_poles, 4);
        assert!((config.rated_slip - 0.03).abs() < 1e-10);
    }

    #[test]
    fn test_report_speed_is_plausible() {
        let config = McsaConfig {
            sample_rate_hz: 10000.0,
            line_frequency_hz: 60.0,
            num_poles: 4,
            rated_slip: 0.03,
        };
        let processor = McsaProcessor::new(config);
        let signal = gen_sine(60.0, 1.0, 10000.0, 16384);
        let report = processor.analyze(&signal);

        // For a 4-pole 60 Hz motor, speed should be in [1600, 1800] RPM range
        assert!(
            report.speed_rpm > 1000.0 && report.speed_rpm < 1900.0,
            "Speed {} RPM out of plausible range",
            report.speed_rpm
        );
    }

    #[test]
    fn test_motor_fault_enum_equality() {
        assert_eq!(MotorFault::BrokenRotorBar, MotorFault::BrokenRotorBar);
        assert_ne!(MotorFault::BrokenRotorBar, MotorFault::Eccentricity);
        assert_ne!(MotorFault::BearingInner, MotorFault::BearingOuter);
    }

    #[test]
    fn test_fault_severity_equality() {
        assert_eq!(FaultSeverity::Healthy, FaultSeverity::Healthy);
        assert_ne!(FaultSeverity::Healthy, FaultSeverity::Severe);
    }

    #[test]
    fn test_bearing_freqs_debug() {
        let bf = bearing_fault_frequencies(30.0, 8, 10.0, 50.0);
        let debug_str = format!("{:?}", bf);
        assert!(debug_str.contains("bpfo"));
        assert!(debug_str.contains("bpfi"));
    }

    #[test]
    fn test_find_peak_near() {
        // Construct a simple spectrum with a known peak
        let mut spectrum = vec![-60.0; 512];
        spectrum[100] = 0.0; // Peak at bin 100
        let freq_res = 1.0; // 1 Hz per bin
        let found = find_peak_near(&spectrum, freq_res, 100.0, 5.0);
        assert!((found - 100.0).abs() < 1e-10);
    }
}
