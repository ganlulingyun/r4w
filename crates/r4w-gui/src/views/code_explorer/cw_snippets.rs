//! CW (Continuous Wave) code snippets
//!
//! CW is the simplest possible waveform - a pure sinusoidal tone.
//! It's the foundation for understanding all other modulation schemes.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete CW waveform code documentation
pub static CW_CODE: WaveformCode = WaveformCode {
    waveform_id: "CW",
    display_name: "Continuous Wave (CW)",
    introduction: "CW is a pure sinusoidal tone - the simplest possible signal. \
        It doesn't carry data, but understanding how it's generated is fundamental \
        to all digital signal processing. Every modulation scheme builds on these concepts.",
    complexity: 1,
    categories: &[
        &GENERATION_CATEGORY,
        &DEMODULATION_CATEGORY,
        &VISUALIZATION_CATEGORY,
    ],
};

static GENERATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Signal Generation",
    description: "How to create I/Q samples representing a pure tone",
    snippets: &[
        CodeSnippet {
            name: "generate_samples()",
            brief: "Core I/Q sample generation using phase accumulation",
            code: r#"pub fn generate_samples(&self, num_samples: usize) -> Vec<IQSample> {
    // Angular frequency: how much phase advances per sample
    // omega = 2 * pi * frequency / sample_rate
    let omega = 2.0 * PI * self.frequency / self.common.sample_rate;
    let amp = self.common.amplitude;

    (0..num_samples)
        .map(|n| {
            // Phase accumulates linearly with each sample
            let phase = self.initial_phase + omega * n as f64;

            // Euler's formula: e^(j*phase) = cos(phase) + j*sin(phase)
            // I = Real part = cos(phase)
            // Q = Imaginary part = sin(phase)
            IQSample::new(amp * phase.cos(), amp * phase.sin())
        })
        .collect()
}"#,
            explanation: "This is the heart of all digital signal generation.\n\n\
                \
                **Phase Accumulation:**\n\
                The key insight is that a sinusoid's phase increases linearly with time. \
                At each sample n, the phase is: φ(n) = φ₀ + ω·n\n\n\
                \
                **Angular Frequency (ω):**\n\
                ω = 2πf/Fs normalizes frequency to radians per sample. \
                If f=1000 Hz and Fs=10000 Hz, then ω = 2π/10 ≈ 0.628 rad/sample.\n\n\
                \
                **Euler's Formula:**\n\
                The complex exponential e^(jφ) = cos(φ) + j·sin(φ) gives us both I and Q components. \
                This traces a circle in the I/Q plane as phase advances.\n\n\
                \
                **Why I/Q?**\n\
                Using complex (I/Q) representation lets us represent both amplitude AND phase \
                in a single sample, enabling frequency shifts and all modulation types.",
            concepts: &[
                "Phase accumulation",
                "Angular frequency (ω)",
                "Euler's formula",
                "I/Q representation",
                "Complex exponential",
            ],
        },
        CodeSnippet {
            name: "generate()",
            brief: "Generate a specific duration of CW signal",
            code: r#"pub fn generate(&self, duration_seconds: f64) -> Vec<IQSample> {
    let num_samples = (duration_seconds * self.common.sample_rate) as usize;
    self.generate_samples(num_samples)
}"#,
            explanation: "A convenience wrapper that converts time duration to sample count.\n\n\
                \
                **Time to Samples:**\n\
                samples = duration × sample_rate\n\n\
                For example, 1ms at 10kHz sample rate = 0.001 × 10000 = 10 samples.",
            concepts: &["Sample rate", "Time-to-samples conversion"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Demodulation / Analysis",
    description: "Extracting information from received CW signals",
    snippets: &[
        CodeSnippet {
            name: "demodulate() - Power Estimation",
            brief: "Measure signal power from I/Q samples",
            code: r#"// Estimate signal power (average magnitude squared)
let power: f64 = samples.iter()
    .map(|s| s.norm_sqr())  // |I + jQ|² = I² + Q²
    .sum::<f64>() / samples.len() as f64;

result.metadata.insert("power".to_string(), power);"#,
            explanation: "Power measurement is fundamental to all signal analysis.\n\n\
                \
                **Complex Magnitude:**\n\
                For a complex sample s = I + jQ, the magnitude squared is:\n\
                |s|² = I² + Q²\n\n\
                \
                **Why Squared?**\n\
                Power is proportional to amplitude squared (P ∝ A²). \
                Using norm_sqr() avoids the expensive square root operation.\n\n\
                \
                **Averaging:**\n\
                We average over all samples to get a stable power estimate \
                that's less affected by instantaneous noise.",
            concepts: &["Signal power", "Complex magnitude", "Averaging"],
        },
        CodeSnippet {
            name: "demodulate() - Frequency Estimation",
            brief: "Estimate frequency from phase rotation between samples",
            code: r#"// Estimate frequency from phase rotation
if samples.len() >= 2 {
    let mut phase_diffs = Vec::new();
    for i in 1..samples.len() {
        // Multiply current sample by conjugate of previous
        // This gives the phase difference between them
        let phase_diff = (samples[i] * samples[i - 1].conj()).arg();
        phase_diffs.push(phase_diff);
    }

    // Average phase difference per sample
    let avg_phase_diff: f64 = phase_diffs.iter().sum::<f64>()
        / phase_diffs.len() as f64;

    // Convert phase-per-sample to frequency in Hz
    // f = (Δφ / 2π) × sample_rate
    let estimated_freq = avg_phase_diff * self.common.sample_rate / (2.0 * PI);

    result.metadata.insert("estimated_freq".to_string(), estimated_freq);
}"#,
            explanation: "This elegant technique estimates frequency without an FFT.\n\n\
                \
                **The Conjugate Trick:**\n\
                When you multiply a complex number by the conjugate of another:\n\
                s[n] × conj(s[n-1]) = |s|² × e^(j·Δφ)\n\
                The result's phase IS the phase difference between samples.\n\n\
                \
                **Phase to Frequency:**\n\
                If phase advances by Δφ radians per sample, the frequency is:\n\
                f = (Δφ × Fs) / (2π)\n\n\
                \
                **Why This Works:**\n\
                A sinusoid at frequency f has phase that increases by 2πf/Fs per sample. \
                By measuring this phase change, we recover the frequency.\n\n\
                \
                **Averaging:**\n\
                Noise causes random phase errors, but averaging many measurements \
                gives a reliable estimate.",
            concepts: &[
                "Phase differentiation",
                "Complex conjugate",
                "Instantaneous frequency",
                "Phase-to-frequency conversion",
            ],
        },
    ],
};

static VISUALIZATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Visualization",
    description: "Creating visual representations for understanding",
    snippets: &[
        CodeSnippet {
            name: "get_visualization() - Constellation",
            brief: "Generate constellation diagram points",
            code: r#"// CW traces a circle in the I/Q plane
let constellation: Vec<IQSample> = (0..32)
    .map(|i| {
        let angle = 2.0 * PI * i as f64 / 32.0;
        IQSample::new(
            self.common.amplitude * angle.cos(),
            self.common.amplitude * angle.sin(),
        )
    })
    .collect();"#,
            explanation: "The constellation diagram shows where samples fall in the I/Q plane.\n\n\
                \
                **CW = Circle:**\n\
                A CW signal traces a perfect circle because:\n\
                - Amplitude is constant (radius)\n\
                - Phase rotates continuously (angle)\n\n\
                \
                **32 Points:**\n\
                We generate 32 evenly-spaced points around the circle to represent \
                all possible phases the signal passes through.\n\n\
                \
                **Contrast with Digital Modulation:**\n\
                Unlike CW's continuous circle, digital modulations use discrete points:\n\
                - BPSK: 2 points (0° and 180°)\n\
                - QPSK: 4 points (0°, 90°, 180°, 270°)\n\
                - 16QAM: 16 points in a grid",
            concepts: &[
                "Constellation diagram",
                "I/Q plane",
                "Constant envelope",
                "Phase rotation",
            ],
        },
    ],
};
