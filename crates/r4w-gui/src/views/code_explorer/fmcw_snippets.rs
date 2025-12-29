//! FMCW (Frequency Modulated Continuous Wave) radar code snippets
//!
//! FMCW radar uses frequency-swept chirps to measure distance and velocity.
//! Used in automotive radar, level sensing, and weather radar.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete FMCW waveform code documentation
pub static FMCW_CODE: WaveformCode = WaveformCode {
    waveform_id: "FMCW",
    display_name: "FMCW Radar (Frequency Modulated Continuous Wave)",
    introduction: "FMCW radar continuously transmits frequency-swept chirps and mixes the return signal \
        with the transmit signal. The beat frequency reveals target range, while Doppler shift reveals velocity. \
        Unlike pulsed radar, FMCW transmits continuously at low power, making it ideal for short-range applications. \
        Used in automotive radar (77 GHz), aircraft altimeters, industrial level sensors, and gesture recognition.",
    complexity: 4,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &CHIRP_GENERATION_CATEGORY,
        &RANGE_DETECTION_CATEGORY,
        &VELOCITY_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "FMCW Fundamentals",
    description: "Core concepts of FMCW radar operation",
    snippets: &[
        CodeSnippet {
            name: "FmcwConfig",
            brief: "Configure FMCW radar parameters",
            code: r#"pub struct FmcwConfig {
    /// Chirp bandwidth in Hz
    pub bandwidth: f64,        // e.g., 4 GHz for automotive
    /// Chirp duration in seconds
    pub chirp_duration: f64,   // e.g., 10-100 µs
    /// Number of chirps per frame
    pub num_chirps: usize,     // e.g., 64-256
    /// Chirp sweep direction
    pub direction: ChirpDirection,
    /// Center frequency
    pub center_freq: f64,      // e.g., 77 GHz automotive
    /// Sample rate for IF signal
    pub sample_rate: f64,
}

// Key FMCW equations:
// Range resolution: Δr = c / (2 × bandwidth)
// Max range: R_max = sample_rate × c × chirp_duration / (2 × bandwidth)
// Velocity resolution: Δv = λ / (2 × num_chirps × chirp_duration)"#,
            explanation: "**FMCW Parameters** determine radar performance.\n\n\
                **Range Resolution:**\n\
                Δr = c / (2 × BW)\n\
                - 4 GHz BW → 3.75 cm resolution\n\
                - More bandwidth = finer resolution\n\n\
                **Maximum Range:**\n\
                - Limited by IF bandwidth and chirp duration\n\
                - Trade-off with range resolution\n\n\
                **Automotive Radar (77 GHz):**\n\
                - Short range: 0.2-30 m, high resolution\n\
                - Long range: up to 250 m, lower resolution\n\
                - Typical BW: 1-4 GHz",
            concepts: &["Range resolution", "Maximum range", "Chirp bandwidth"],
        },
        CodeSnippet {
            name: "ChirpDirection",
            brief: "Chirp sweep patterns",
            code: r#"pub enum ChirpDirection {
    /// Frequency increases with time (up-chirp)
    Up,
    /// Frequency decreases with time (down-chirp)
    Down,
    /// Alternating up and down (triangle)
    Triangle,
    /// Up then idle, repeat (sawtooth)
    Sawtooth,
}

// Why different patterns?
// - Up-chirp only: simplest, ambiguous Doppler+range
// - Triangle: separates range and Doppler
// - Sawtooth: common in automotive, efficient duty cycle

// Beat frequency interpretation:
// Up-chirp:   f_b = f_range - f_doppler
// Down-chirp: f_b = f_range + f_doppler
// Average: f_range = (f_up + f_down) / 2
// Difference: f_doppler = (f_down - f_up) / 2"#,
            explanation: "**Chirp Direction** affects measurement capability.\n\n\
                **Up-Chirp Only:**\n\
                - Simple but range-Doppler ambiguous\n\
                - Beat freq = f_range - f_doppler\n\
                - Can't separate static from moving targets\n\n\
                **Triangle Chirp:**\n\
                - Up-chirp then down-chirp\n\
                - Solves ambiguity:\n\
                  - f_range = (f_up + f_down) / 2\n\
                  - f_doppler = (f_down - f_up) / 2\n\n\
                **Sawtooth:**\n\
                - Most common in automotive\n\
                - Uses multiple chirps + Doppler FFT",
            concepts: &["Up-chirp", "Down-chirp", "Triangle modulation", "Sawtooth"],
        },
    ],
};

static CHIRP_GENERATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Chirp Generation",
    description: "Creating FMCW transmit signals",
    snippets: &[
        CodeSnippet {
            name: "generate_chirp()",
            brief: "Generate a single FMCW chirp",
            code: r#"fn generate_chirp(&self) -> Vec<IQSample> {
    let num_samples = (self.chirp_duration * self.sample_rate) as usize;
    let mut samples = Vec::with_capacity(num_samples);

    // Chirp rate: Hz per second
    let k = self.bandwidth / self.chirp_duration;

    for i in 0..num_samples {
        let t = i as f64 / self.sample_rate;

        // Instantaneous frequency: f(t) = f0 + k*t
        // Instantaneous phase: φ(t) = 2π(f0*t + k*t²/2)
        let phase = 2.0 * PI * (self.start_freq * t + 0.5 * k * t * t);

        samples.push(IQSample::new(phase.cos(), phase.sin()));
    }
    samples
}

// Key insight: phase is INTEGRAL of frequency
// f(t) = f0 + k*t
// φ(t) = ∫f(t)dt = f0*t + k*t²/2"#,
            explanation: "**Chirp Generation** creates the frequency-swept waveform.\n\n\
                **Linear FM:**\n\
                - Frequency increases linearly: f(t) = f₀ + kt\n\
                - k = bandwidth / chirp_duration (chirp rate)\n\n\
                **Phase Calculation:**\n\
                - Phase is integral of frequency\n\
                - φ(t) = 2π(f₀t + kt²/2)\n\
                - Quadratic phase = linear frequency!\n\n\
                **I/Q Generation:**\n\
                - I = cos(φ), Q = sin(φ)\n\
                - Complex exponential: e^(jφ)",
            concepts: &["Linear FM", "Chirp rate", "Phase calculation"],
        },
        CodeSnippet {
            name: "generate_frame()",
            brief: "Generate a complete frame of chirps",
            code: r#"fn generate_frame(&self) -> Vec<Vec<IQSample>> {
    let mut frame = Vec::with_capacity(self.num_chirps);

    for chirp_idx in 0..self.num_chirps {
        let chirp = match self.direction {
            ChirpDirection::Up => self.generate_up_chirp(),
            ChirpDirection::Down => self.generate_down_chirp(),
            ChirpDirection::Triangle => {
                if chirp_idx % 2 == 0 {
                    self.generate_up_chirp()
                } else {
                    self.generate_down_chirp()
                }
            }
            ChirpDirection::Sawtooth => self.generate_up_chirp(),
        };
        frame.push(chirp);
    }
    frame
}

// Frame structure (typical automotive):
// - 64-256 chirps per frame
// - ~10-50 ms frame duration
// - Frame rate: 20-100 fps"#,
            explanation: "**Frame Structure** organizes chirps for processing.\n\n\
                **Why Multiple Chirps?**\n\
                - Doppler resolution needs multiple samples\n\
                - Each chirp → one range profile\n\
                - Stack chirps → Doppler FFT across chirps\n\n\
                **Frame Timing:**\n\
                - Frame = all chirps for one measurement\n\
                - Frame rate = radar update rate\n\
                - More chirps = better velocity resolution\n\n\
                **2D Processing:**\n\
                - Fast-time: samples within chirp (range)\n\
                - Slow-time: across chirps (velocity)",
            concepts: &["Frame structure", "Fast-time", "Slow-time", "Radar cube"],
        },
    ],
};

static RANGE_DETECTION_CATEGORY: CodeCategory = CodeCategory {
    name: "Range Detection",
    description: "Extracting target distance from beat signal",
    snippets: &[
        CodeSnippet {
            name: "dechirp()",
            brief: "Mix received signal with transmit to get beat signal",
            code: r#"fn dechirp(&self, tx: &[IQSample], rx: &[IQSample]) -> Vec<IQSample> {
    // Multiply received signal by conjugate of transmit
    // This is the "dechirp" or "stretch processing"
    tx.iter()
        .zip(rx.iter())
        .map(|(t, r)| {
            // Complex multiplication: rx × tx*
            IQSample::new(
                r.re * t.re + r.im * t.im,  // Real part
                r.im * t.re - r.re * t.im,  // Imag part
            )
        })
        .collect()
}

// The magic: chirp × chirp* = tone at beat frequency
// f_beat = f_rx - f_tx = (2 × R × k) / c
// where k = chirp_rate, R = range"#,
            explanation: "**Dechirping** converts range to frequency.\n\n\
                **The Key Insight:**\n\
                - TX sends chirp: f_tx(t) = f₀ + kt\n\
                - RX gets delayed chirp: f_rx(t) = f₀ + k(t - τ)\n\
                - Mix them: f_beat = k × τ = constant!\n\n\
                **Why This Works:**\n\
                - Delay τ = 2R/c (round-trip)\n\
                - Beat frequency f_b = k × 2R/c\n\
                - Range → frequency conversion!\n\n\
                **Benefits:**\n\
                - Converts wideband chirp to narrowband tone\n\
                - Can use low-speed ADC\n\
                - FFT gives range profile instantly",
            concepts: &["Dechirp", "Beat frequency", "Stretch processing"],
        },
        CodeSnippet {
            name: "range_fft()",
            brief: "FFT of beat signal to find target ranges",
            code: r#"fn range_fft(&self, beat_signal: &[IQSample]) -> Vec<(f64, f64)> {
    // FFT of beat signal
    let spectrum = fft(beat_signal);
    let mut targets = Vec::new();

    // Convert FFT bins to range
    for (bin, &magnitude) in spectrum.iter().enumerate() {
        if magnitude > self.threshold {
            // Beat frequency for this bin
            let f_beat = bin as f64 * self.sample_rate / spectrum.len() as f64;

            // Convert to range: R = f_beat × c × T_chirp / (2 × BW)
            let range = f_beat * C * self.chirp_duration / (2.0 * self.bandwidth);

            targets.push((range, magnitude));
        }
    }
    targets
}

// Example:
// BW = 4 GHz, T = 50 µs
// f_beat = 1 MHz → R = 1e6 × 3e8 × 50e-6 / (2 × 4e9) = 1.875 m"#,
            explanation: "**Range FFT** reveals target distances.\n\n\
                **Frequency to Range:**\n\
                R = f_beat × c × T_chirp / (2 × BW)\n\n\
                **Example Calculation:**\n\
                - f_beat = 1 MHz\n\
                - BW = 4 GHz, T = 50 µs\n\
                - R = 1.875 meters\n\n\
                **Resolution:**\n\
                - Range resolution = c / (2 × BW)\n\
                - 4 GHz BW → 3.75 cm resolution\n\n\
                **FFT Bins:**\n\
                - Each bin = different range\n\
                - Multiple targets = multiple peaks",
            concepts: &["Range FFT", "Bin-to-range conversion", "Multiple targets"],
        },
    ],
};

static VELOCITY_CATEGORY: CodeCategory = CodeCategory {
    name: "Velocity Detection",
    description: "Measuring target velocity via Doppler",
    snippets: &[
        CodeSnippet {
            name: "doppler_fft()",
            brief: "FFT across chirps to find target velocity",
            code: r#"fn doppler_fft(&self, range_profiles: &[Vec<f64>]) -> Vec<Vec<f64>> {
    let num_chirps = range_profiles.len();
    let num_range_bins = range_profiles[0].len();
    let mut range_doppler = vec![vec![0.0; num_chirps]; num_range_bins];

    // For each range bin, FFT across chirps
    for range_bin in 0..num_range_bins {
        // Extract this range bin across all chirps
        let samples: Vec<_> = range_profiles.iter()
            .map(|rp| rp[range_bin])
            .collect();

        // Doppler FFT
        let doppler_spectrum = fft(&samples);
        range_doppler[range_bin] = doppler_spectrum;
    }
    range_doppler
}

// Velocity from Doppler bin:
// v = (doppler_bin × λ × PRF) / (2 × num_chirps)
// where PRF = 1/chirp_duration"#,
            explanation: "**Doppler FFT** measures target velocity.\n\n\
                **How It Works:**\n\
                - Moving target → phase shift between chirps\n\
                - Phase shift rate = Doppler frequency\n\
                - FFT across chirps extracts Doppler\n\n\
                **Velocity Calculation:**\n\
                v = f_doppler × λ / 2\n\n\
                **Range-Doppler Map:**\n\
                - 2D matrix: range × velocity\n\
                - Each target appears as a peak\n\
                - Can resolve targets by range AND velocity\n\n\
                **Automotive Example:**\n\
                - Detect car at 50m, moving at 30 m/s\n\
                - Separate from car at 50m, stationary",
            concepts: &["Doppler FFT", "Range-Doppler map", "Velocity resolution"],
        },
        CodeSnippet {
            name: "process_frame()",
            brief: "Complete 2D-FFT processing of radar frame",
            code: r#"fn process_frame(&self, frame: &[Vec<IQSample>]) -> RangeDopplerMap {
    // Step 1: Dechirp each chirp with reference
    let beat_signals: Vec<_> = frame.iter()
        .map(|rx| self.dechirp(&self.tx_chirp, rx))
        .collect();

    // Step 2: Range FFT on each chirp (fast-time)
    let range_profiles: Vec<_> = beat_signals.iter()
        .map(|b| self.range_fft(b))
        .collect();

    // Step 3: Doppler FFT across chirps (slow-time)
    let range_doppler = self.doppler_fft(&range_profiles);

    // Step 4: CFAR detection to find targets
    let targets = self.cfar_detect(&range_doppler);

    RangeDopplerMap { data: range_doppler, targets }
}

// This is the standard FMCW radar processing chain:
// Raw data → Dechirp → Range FFT → Doppler FFT → Detection"#,
            explanation: "**2D-FFT Processing** is the FMCW radar pipeline.\n\n\
                **Step 1: Dechirp**\n\
                - Mix RX with TX reference\n\
                - Converts chirp to beat tones\n\n\
                **Step 2: Range FFT**\n\
                - FFT within each chirp\n\
                - Separates targets by range\n\n\
                **Step 3: Doppler FFT**\n\
                - FFT across chirps at each range\n\
                - Separates targets by velocity\n\n\
                **Step 4: Detection (CFAR)**\n\
                - Constant False Alarm Rate detection\n\
                - Adapts threshold to local noise\n\
                - Outputs range, velocity, amplitude",
            concepts: &["2D-FFT", "CFAR", "Radar processing chain"],
        },
    ],
};
