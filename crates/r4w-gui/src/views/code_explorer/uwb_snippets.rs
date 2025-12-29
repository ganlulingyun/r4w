//! UWB (Ultra-Wideband) code snippets
//!
//! UWB uses very short pulses to achieve extremely wide bandwidth,
//! enabling precise ranging, high data rates, and low power operation.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete UWB waveform code documentation
pub static UWB_CODE: WaveformCode = WaveformCode {
    waveform_id: "UWB",
    display_name: "Ultra-Wideband (UWB)",
    introduction: "UWB transmits information using extremely short pulses (nanoseconds), \
        spreading energy across several GHz of bandwidth. This enables cm-level ranging precision, \
        high data rates, and operation below the noise floor for other systems. Used in Apple AirTags, \
        automotive keyless entry, indoor positioning, and through-wall radar. FCC defines UWB as >500 MHz \
        bandwidth or >20% fractional bandwidth.",
    complexity: 4,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &PULSE_SHAPES_CATEGORY,
        &MODULATION_CATEGORY,
        &RANGING_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "UWB Fundamentals",
    description: "Ultra-wideband principles and configuration",
    snippets: &[
        CodeSnippet {
            name: "UwbConfig",
            brief: "Configure UWB pulse and modulation parameters",
            code: r#"pub struct UwbConfig {
    /// Pulse shape
    pub pulse_shape: PulseShape,
    /// Pulse duration in nanoseconds (typically 1-2 ns)
    pub pulse_duration_ns: f64,
    /// Pulse Repetition Interval in nanoseconds
    pub pulse_interval_ns: f64,
    /// Modulation scheme
    pub modulation: UwbModulation,
    /// Pulses per bit (for integration gain)
    pub pulses_per_bit: usize,
    /// Center frequency (typically 3.5-10 GHz)
    pub center_freq: f64,
}

// Key UWB Properties:
// - Bandwidth: ~500 MHz to several GHz
// - Pulse duration: 0.5-2 ns
// - Range resolution: c * pulse_width / 2 ≈ 15 cm per ns
// - Power: -41.3 dBm/MHz (FCC limit)"#,
            explanation: "**UWB Fundamentals** center on ultra-short pulses.\n\n\
                **What Makes UWB Special:**\n\
                - Extremely wide bandwidth (>500 MHz)\n\
                - Very short pulses (nanoseconds)\n\
                - Low power spectral density (coexists with everything)\n\n\
                **Range Resolution:**\n\
                - Δr = c × pulse_width / 2\n\
                - 1 ns pulse → 15 cm resolution\n\
                - GPS: ~10m, UWB: ~10cm!\n\n\
                **FCC Regulations:**\n\
                - Max PSD: -41.3 dBm/MHz\n\
                - Below noise floor for narrowband systems\n\
                - Allowed in 3.1-10.6 GHz band",
            concepts: &["Pulse duration", "Bandwidth", "Range resolution", "FCC regulations"],
        },
        CodeSnippet {
            name: "PulseShape",
            brief: "Different UWB pulse shapes",
            code: r#"pub enum PulseShape {
    /// Gaussian monocycle (first derivative of Gaussian)
    GaussianMonocycle,
    /// Gaussian doublet (second derivative)
    GaussianDoublet,
    /// Raised cosine pulse
    RaisedCosine,
    /// Simple rectangular pulse
    Rectangular,
}

impl PulseShape {
    pub fn bandwidth(&self, duration_ns: f64) -> f64 {
        // Approximate -10dB bandwidth
        match self {
            Self::GaussianMonocycle => 0.8 / duration_ns * 1e9,
            Self::GaussianDoublet => 1.0 / duration_ns * 1e9,
            Self::RaisedCosine => 1.0 / duration_ns * 1e9,
            Self::Rectangular => 0.88 / duration_ns * 1e9,
        }
    }
}"#,
            explanation: "**Pulse Shapes** affect bandwidth and spectrum.\n\n\
                **Gaussian Monocycle:**\n\
                - First derivative of Gaussian\n\
                - Zero DC content (no low-frequency energy)\n\
                - Good spectral efficiency\n\n\
                **Gaussian Doublet:**\n\
                - Second derivative\n\
                - Even better spectral containment\n\
                - Used in IEEE 802.15.4a\n\n\
                **Why No DC?**\n\
                - FCC restricts low frequencies\n\
                - Derivative pulses naturally have no DC\n\
                - Better antenna efficiency",
            concepts: &["Gaussian pulse", "Monocycle", "Spectral shaping", "DC-free"],
        },
    ],
};

static PULSE_SHAPES_CATEGORY: CodeCategory = CodeCategory {
    name: "Pulse Generation",
    description: "Generating ultra-short UWB pulses",
    snippets: &[
        CodeSnippet {
            name: "gaussian_monocycle()",
            brief: "Generate Gaussian monocycle pulse",
            code: r#"fn gaussian_monocycle(&self, t: f64) -> f64 {
    // First derivative of Gaussian
    // p(t) = -t/σ² × exp(-t²/2σ²)
    let sigma = self.pulse_duration_ns * 1e-9 / 2.5;  // ~2.5σ = pulse width
    let t_norm = t / sigma;

    -t_norm * (-0.5 * t_norm * t_norm).exp()
}

// Properties:
// - Zero at t=0 (no DC)
// - Positive peak at t = -σ
// - Negative peak at t = +σ
// - ~90% energy in pulse_duration"#,
            explanation: "**Gaussian Monocycle** is the classic UWB pulse.\n\n\
                **Math:**\n\
                p(t) = -t/σ² × exp(-t²/2σ²)\n\n\
                **Properties:**\n\
                - Zero at t=0 (passes through zero)\n\
                - One positive lobe, one negative lobe\n\
                - Bandwidth ≈ 0.8/τ (τ = pulse duration)\n\n\
                **Why \"Monocycle\"?**\n\
                - One complete cycle (pos → neg or neg → pos)\n\
                - Efficient use of bandwidth\n\
                - Natural DC rejection\n\n\
                **Generation:**\n\
                - Can be created with simple circuits\n\
                - Or digitally synthesized and DAC'd",
            concepts: &["Monocycle", "Gaussian derivative", "DC rejection"],
        },
        CodeSnippet {
            name: "generate_pulse()",
            brief: "Generate one UWB pulse at specified time",
            code: r#"fn generate_pulse(&self, pulse_center: f64, samples: &mut [IQSample]) {
    let dt = 1.0 / self.sample_rate;
    let half_width = self.pulse_duration_ns * 1e-9 / 2.0;

    for (i, sample) in samples.iter_mut().enumerate() {
        let t = i as f64 * dt - pulse_center;

        // Only generate pulse near center (optimization)
        if t.abs() < half_width * 3.0 {
            let pulse_val = match self.pulse_shape {
                PulseShape::GaussianMonocycle => self.gaussian_monocycle(t),
                PulseShape::GaussianDoublet => self.gaussian_doublet(t),
                PulseShape::RaisedCosine => self.raised_cosine(t),
                PulseShape::Rectangular => self.rectangular(t),
            };
            sample.re += pulse_val * self.amplitude;
        }
    }
}"#,
            explanation: "**Pulse Generation** creates the UWB waveform.\n\n\
                **Efficiency Note:**\n\
                - Only compute pulse near its center\n\
                - Pulse energy is concentrated in ~3× duration\n\
                - Saves computation for sparse pulse trains\n\n\
                **Timing Precision:**\n\
                - Pulse center determines ranging\n\
                - Sub-nanosecond timing needed\n\
                - 3 cm per 100 ps timing error!\n\n\
                **Amplitude Control:**\n\
                - Meet FCC power limits\n\
                - Adjust for range/link budget",
            concepts: &["Pulse timing", "Sparse signals", "Power control"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "UWB Modulation",
    description: "Encoding data using ultra-short pulses",
    snippets: &[
        CodeSnippet {
            name: "UwbModulation",
            brief: "Different UWB data modulation schemes",
            code: r#"pub enum UwbModulation {
    /// On-Off Keying: pulse present (1) or absent (0)
    Ook,
    /// Binary PSK: pulse polarity encodes bit
    Bpsk,
    /// Pulse Position Modulation: early/late position
    Ppm { time_shift_ns: f64 },
}

impl UwbModulation {
    fn modulate_bit(&self, bit: u8, pulse: &[f64]) -> Vec<f64> {
        match self {
            Self::Ook => {
                if bit == 1 { pulse.to_vec() }
                else { vec![0.0; pulse.len()] }
            }
            Self::Bpsk => {
                let sign = if bit == 0 { 1.0 } else { -1.0 };
                pulse.iter().map(|&p| p * sign).collect()
            }
            Self::Ppm { time_shift_ns } => {
                // Shift pulse position based on bit
                let shift = if bit == 1 { *time_shift_ns } else { 0.0 };
                apply_time_shift(pulse, shift)
            }
        }
    }
}"#,
            explanation: "**UWB Modulation** schemes encode data in pulses.\n\n\
                **OOK (On-Off Keying):**\n\
                - Simplest: pulse = 1, no pulse = 0\n\
                - Half the energy efficiency of BPSK\n\n\
                **BPSK (Binary PSK):**\n\
                - Pulse polarity encodes bit\n\
                - Better noise performance than OOK\n\n\
                **PPM (Pulse Position Modulation):**\n\
                - Time position encodes bit\n\
                - Robust to amplitude variations\n\
                - Common in UWB systems\n\n\
                **Multi-Pulse:**\n\
                - Multiple pulses per bit for integration gain\n\
                - Improves SNR at cost of data rate",
            concepts: &["OOK", "BPSK", "PPM", "Integration gain"],
        },
        CodeSnippet {
            name: "modulate()",
            brief: "Generate UWB signal from data bits",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let samples_per_bit = (self.pulse_interval_ns * 1e-9 * self.sample_rate)
        as usize * self.pulses_per_bit;
    let mut signal = vec![IQSample::new(0.0, 0.0); data.len() * 8 * samples_per_bit];

    for (bit_idx, byte) in data.iter().enumerate() {
        for i in 0..8 {
            let bit = (byte >> (7 - i)) & 1;
            let bit_start = (bit_idx * 8 + i) * samples_per_bit;

            // Generate pulses_per_bit pulses for this bit
            for p in 0..self.pulses_per_bit {
                let pulse_center = bit_start as f64 / self.sample_rate
                    + p as f64 * self.pulse_interval_ns * 1e-9;

                self.generate_modulated_pulse(bit, pulse_center, &mut signal);
            }
        }
    }
    signal
}"#,
            explanation: "**UWB Modulation** generates the pulse train.\n\n\
                **Pulse Train Structure:**\n\
                - Each bit → pulses_per_bit pulses\n\
                - Pulses separated by pulse_interval\n\
                - Modulation applied to each pulse\n\n\
                **Integration Gain:**\n\
                - Multiple pulses per bit\n\
                - Receiver sums pulse energies\n\
                - Gain = 10×log₁₀(pulses_per_bit) dB\n\n\
                **Duty Cycle:**\n\
                - UWB has very low duty cycle\n\
                - ~1 ns pulse every ~1 µs = 0.1%\n\
                - Keeps average power low (FCC compliant)",
            concepts: &["Pulse train", "Integration gain", "Duty cycle"],
        },
    ],
};

static RANGING_CATEGORY: CodeCategory = CodeCategory {
    name: "UWB Ranging",
    description: "Precise distance measurement using UWB pulses",
    snippets: &[
        CodeSnippet {
            name: "calculate_range()",
            brief: "Calculate distance from time-of-flight",
            code: r#"fn calculate_range(&self, tof_ns: f64) -> f64 {
    // Speed of light: ~0.3 m/ns
    const C: f64 = 299_792_458.0;  // m/s

    // Range = c × ToF / 2 (round-trip)
    // Or just c × ToF for one-way with synchronized clocks
    let one_way_tof = tof_ns * 1e-9 / 2.0;  // Assuming round-trip
    C * one_way_tof
}

// Example:
// ToF = 10 ns round-trip
// Range = 3e8 × 5e-9 = 1.5 m

// Precision depends on:
// - Pulse bandwidth (wider = better time resolution)
// - SNR (affects leading edge detection)
// - Clock synchronization
// - Multipath (early reflections confuse)"#,
            explanation: "**UWB Ranging** achieves cm-level precision.\n\n\
                **Time-of-Flight:**\n\
                - Measure pulse round-trip time\n\
                - Range = c × ToF / 2\n\
                - 1 ns = 15 cm range resolution\n\n\
                **Why UWB Excels:**\n\
                - Wide bandwidth → sharp pulse\n\
                - Sharp pulse → precise timing\n\
                - Can resolve multipath (separate arrivals)\n\n\
                **Applications:**\n\
                - Indoor positioning (~10 cm accuracy)\n\
                - Secure car access (relay attack prevention)\n\
                - Apple AirTags, Samsung SmartTags",
            concepts: &["Time-of-flight", "Range resolution", "Indoor positioning"],
        },
        CodeSnippet {
            name: "detect_first_path()",
            brief: "Find the first (direct path) pulse arrival",
            code: r#"fn detect_first_path(&self, signal: &[IQSample]) -> Option<f64> {
    // Leading-edge detection for ToF
    let threshold = self.noise_floor * 6.0;  // SNR threshold
    let mut max_corr = 0.0;
    let mut first_above_threshold = None;

    // Correlate with reference pulse
    for (i, window) in signal.windows(self.pulse_samples).enumerate() {
        let corr = self.correlate_pulse(window);

        if corr > threshold && first_above_threshold.is_none() {
            first_above_threshold = Some(i);
        }
        max_corr = max_corr.max(corr);
    }

    // Return time of first path (not strongest!)
    first_above_threshold.map(|i| i as f64 / self.sample_rate)
}

// IMPORTANT: Use FIRST path, not strongest!
// Multipath reflections may be stronger but arrive later"#,
            explanation: "**First-Path Detection** is crucial for accurate ranging.\n\n\
                **The Multipath Problem:**\n\
                - Direct path might not be strongest\n\
                - Reflections arrive later but may be stronger\n\
                - Using strongest = overestimate distance\n\n\
                **Leading-Edge Detection:**\n\
                - Find first arrival above threshold\n\
                - That's the direct path distance\n\
                - Later arrivals are reflections\n\n\
                **UWB Advantage:**\n\
                - Narrow pulses resolve multipath\n\
                - Can see direct + reflected separately\n\
                - GPS/WiFi can't do this (too narrowband)",
            concepts: &["First-path detection", "Multipath resolution", "Leading edge"],
        },
    ],
};
