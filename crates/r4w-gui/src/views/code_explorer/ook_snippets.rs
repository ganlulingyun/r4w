//! OOK (On-Off Keying) code snippets
//!
//! OOK is the simplest digital modulation - carrier on for '1', off for '0'.
//! It introduces key concepts like samples-per-symbol and envelope detection.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete OOK waveform code documentation
pub static OOK_CODE: WaveformCode = WaveformCode {
    waveform_id: "OOK",
    display_name: "On-Off Keying (OOK)",
    introduction: "OOK is the simplest way to encode digital data: transmit a carrier for '1', \
        silence for '0'. Think of it like Morse code - the signal is either present or absent. \
        Despite its simplicity, OOK introduces fundamental concepts used in all digital modulations.",
    complexity: 2,
    categories: &[
        &GENERATION_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static GENERATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Symbol Generation",
    description: "Creating the carrier for ON and silence for OFF states",
    snippets: &[
        CodeSnippet {
            name: "sps() - Samples Per Symbol",
            brief: "Calculate how many samples represent one bit",
            code: r#"fn sps(&self) -> usize {
    // samples_per_symbol = sample_rate / symbol_rate
    (self.common.sample_rate / self.symbol_rate) as usize
}"#,
            explanation: "**Samples Per Symbol (SPS)** is a fundamental concept.\n\n\
                \
                **The Relationship:**\n\
                SPS = Fs / Rs (sample rate / symbol rate)\n\n\
                \
                **Example:**\n\
                If Fs = 10,000 Hz and Rs = 1,000 symbols/sec:\n\
                SPS = 10,000 / 1,000 = 10 samples per symbol\n\n\
                \
                **Why It Matters:**\n\
                - More samples = better signal quality but more data\n\
                - Fewer samples = higher symbol rate but harder to detect\n\
                - Minimum ~4-8 samples per symbol for reliable detection",
            concepts: &["Samples per symbol", "Symbol rate", "Sample rate"],
        },
        CodeSnippet {
            name: "generate_on()",
            brief: "Generate carrier samples for a '1' bit with phase continuity",
            code: r#"fn generate_on(&self, start_phase: f64) -> (Vec<IQSample>, f64) {
    let sps = self.sps();
    let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
    let amp = self.common.amplitude;

    let samples: Vec<IQSample> = (0..sps)
        .map(|n| {
            let phase = start_phase + omega * n as f64;
            IQSample::new(amp * phase.cos(), amp * phase.sin())
        })
        .collect();

    // Return ending phase for continuity
    let end_phase = start_phase + omega * sps as f64;
    (samples, end_phase)
}"#,
            explanation: "This generates a carrier burst for one symbol period.\n\n\
                \
                **Phase Continuity:**\n\
                Notice we track the ending phase and return it. This is crucial!\n\
                If we didn't, each '1' bit would start at phase 0, causing:\n\
                - Clicks/pops in the signal\n\
                - Wider bandwidth (spectral splatter)\n\
                - Harder detection\n\n\
                \
                **Returning a Tuple:**\n\
                (samples, end_phase) lets the caller maintain phase state \
                across multiple symbols.",
            concepts: &["Phase continuity", "Carrier generation", "Symbol period"],
        },
        CodeSnippet {
            name: "generate_off()",
            brief: "Generate silence for a '0' bit",
            code: r#"fn generate_off(&self) -> Vec<IQSample> {
    // Simply return zeros for the symbol duration
    vec![IQSample::new(0.0, 0.0); self.sps()]
}"#,
            explanation: "OFF is just silence - zero I and zero Q.\n\n\
                \
                **Why So Simple?**\n\
                Unlike ON, we don't need phase tracking for silence. \
                When the carrier comes back, we resume from where we left off.\n\n\
                \
                **The Power of Zeros:**\n\
                In the I/Q plane, (0,0) is the origin - no signal present. \
                This creates maximum contrast with the ON state.",
            concepts: &["Zero signal", "Symbol silence"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Modulation",
    description: "Converting a bit stream into I/Q samples",
    snippets: &[
        CodeSnippet {
            name: "modulate()",
            brief: "The complete modulation pipeline for OOK",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let mut samples = Vec::with_capacity(data.len() * self.sps());
    let mut phase = 0.0;

    for &bit in data {
        if bit != 0 {
            // Bit = 1: Generate carrier, maintaining phase
            let (on_samples, new_phase) = self.generate_on(phase);
            samples.extend(on_samples);
            phase = new_phase;
        } else {
            // Bit = 0: Generate silence
            samples.extend(self.generate_off());
            // Phase preserved for when carrier returns
        }
    }

    samples
}"#,
            explanation: "This is the complete OOK transmit chain.\n\n\
                \
                **The Algorithm:**\n\
                1. Pre-allocate output buffer (optimization)\n\
                2. Start with phase = 0\n\
                3. For each bit:\n\
                   - If 1: add carrier burst, update phase\n\
                   - If 0: add silence, keep phase\n\n\
                \
                **Phase State Machine:**\n\
                The `phase` variable is our state. It accumulates through ON periods \
                and pauses during OFF periods. This creates a coherent signal.\n\n\
                \
                **Memory Efficiency:**\n\
                `Vec::with_capacity()` pre-allocates memory to avoid repeated reallocations \
                as we append samples.",
            concepts: &[
                "Bit-to-symbol mapping",
                "Phase state machine",
                "Memory pre-allocation",
            ],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Demodulation",
    description: "Recovering bits from received I/Q samples",
    snippets: &[
        CodeSnippet {
            name: "demodulate() - Power Detection",
            brief: "Measure power per symbol for envelope detection",
            code: r#"let sps = self.sps();

// Calculate power in each symbol period
let powers: Vec<f64> = samples.chunks(sps)
    .map(|chunk| {
        // Average power = mean of |sample|²
        chunk.iter()
            .map(|s| s.norm_sqr())
            .sum::<f64>() / chunk.len() as f64
    })
    .collect();"#,
            explanation: "**Envelope Detection** - the simplest demodulation method.\n\n\
                \
                **How It Works:**\n\
                1. Divide samples into symbol-sized chunks\n\
                2. Calculate average power in each chunk\n\
                3. High power = carrier present = '1'\n\
                4. Low power = no carrier = '0'\n\n\
                \
                **Why Power, Not Amplitude?**\n\
                Power (amplitude²) is more stable because:\n\
                - Always positive (no sign issues)\n\
                - Integrates noise better\n\
                - Matches receiver hardware behavior\n\n\
                \
                **The chunks() Method:**\n\
                Rust's iterator method splits the slice into non-overlapping windows \
                of size `sps` - perfect for symbol-by-symbol processing.",
            concepts: &[
                "Envelope detection",
                "Power measurement",
                "Symbol-rate processing",
                "chunks() iterator",
            ],
        },
        CodeSnippet {
            name: "demodulate() - Threshold Decision",
            brief: "Decide bits using adaptive thresholding",
            code: r#"// Find min and max power levels
let max_power = powers.iter().cloned().fold(0.0_f64, f64::max);
let min_power = powers.iter().cloned().fold(f64::MAX, f64::min);

// Threshold = midpoint between ON and OFF power levels
let threshold = (max_power + min_power) / 2.0;

// Decide each bit
for &power in &powers {
    let bit = if power > threshold { 1 } else { 0 };
    result.bits.push(bit);
    result.symbols.push(bit as u16);
}"#,
            explanation: "**Adaptive Thresholding** - automatic level detection.\n\n\
                \
                **The Algorithm:**\n\
                1. Find the maximum power (strongest '1')\n\
                2. Find the minimum power (quietest '0')\n\
                3. Set threshold at the midpoint\n\
                4. Compare each symbol's power to threshold\n\n\
                \
                **Why Adaptive?**\n\
                Signal strength varies due to:\n\
                - Distance from transmitter\n\
                - Antenna orientation\n\
                - Propagation conditions\n\n\
                By computing threshold from the data itself, \
                we automatically adjust to current conditions.\n\n\
                \
                **Limitation:**\n\
                Requires both 0s and 1s in the data. A message of all 1s \
                would have min=max, giving a bad threshold.",
            concepts: &[
                "Adaptive threshold",
                "Decision boundary",
                "Automatic gain control concept",
            ],
        },
        CodeSnippet {
            name: "demodulate() - SNR Estimation",
            brief: "Estimate signal-to-noise ratio",
            code: r#"// Estimate SNR from power ratio
if max_power > 0.0 && min_power < max_power {
    let snr_linear = max_power / min_power.max(1e-10);
    // Convert to decibels: SNR_dB = 10 * log10(SNR_linear)
    result.snr_estimate = Some(10.0 * snr_linear.log10());
}"#,
            explanation: "**SNR** (Signal-to-Noise Ratio) measures signal quality.\n\n\
                \
                **The Calculation:**\n\
                SNR = Power_on / Power_off\n\n\
                In a perfect world, Power_off = 0, giving infinite SNR. \
                In reality, noise fills the 'off' periods.\n\n\
                \
                **Decibel Conversion:**\n\
                SNR_dB = 10 × log₁₀(SNR_linear)\n\n\
                Examples:\n\
                - SNR = 10 → 10 dB\n\
                - SNR = 100 → 20 dB\n\
                - SNR = 1000 → 30 dB\n\n\
                \
                **The 1e-10 Guard:**\n\
                Prevents division by zero if min_power is exactly 0.",
            concepts: &[
                "Signal-to-noise ratio",
                "Decibel conversion",
                "Power ratio",
            ],
        },
    ],
};
