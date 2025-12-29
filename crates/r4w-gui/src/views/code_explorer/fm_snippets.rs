//! FM (Frequency Modulation) code snippets
//!
//! FM encodes information by varying the instantaneous frequency of a carrier.
//! Known for its noise immunity and constant envelope.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete FM waveform code documentation
pub static FM_CODE: WaveformCode = WaveformCode {
    waveform_id: "FM",
    display_name: "Frequency Modulation (FM)",
    introduction: "FM encodes information by varying the carrier's frequency while keeping \
        amplitude constant. Invented by Edwin Armstrong in 1933, FM offers superior noise \
        immunity compared to AM. The constant envelope makes it ideal for non-linear \
        amplifiers. Key parameters are frequency deviation (delta-f) and modulation index (beta).",
    complexity: 3,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "FM Fundamentals",
    description: "Core concepts of frequency modulation",
    snippets: &[
        CodeSnippet {
            name: "FM struct",
            brief: "FM modulator/demodulator parameters",
            code: r#"pub struct FM {
    common: CommonParams,
    symbol_rate: f64,
    carrier_freq: f64,
    /// Frequency deviation in Hz (maximum frequency shift)
    deviation: f64,
    /// Number of frequency levels (2 for binary, 4 for multi-level)
    num_levels: usize,
}"#,
            explanation: "**Key Parameters:**\n\n\
                **Frequency Deviation (delta-f):**\n\
                The maximum frequency shift from the carrier.\n\
                - Binary FM: carrier +/- deviation\n\
                - Higher deviation = better noise immunity but more bandwidth\n\n\
                **Modulation Index (beta):**\n\
                beta = delta-f / f_message\n\
                - beta < 1: Narrowband FM (NBFM) - used in two-way radio\n\
                - beta > 1: Wideband FM (WBFM) - used in FM broadcast\n\n\
                **Carson's Rule:**\n\
                FM Bandwidth >= 2 * (delta-f + f_message)\n\
                Wider deviation = wider bandwidth required.",
            concepts: &["Frequency deviation", "Modulation index", "Carson's rule", "NBFM/WBFM"],
        },
        CodeSnippet {
            name: "modulation_index()",
            brief: "Calculate FM modulation index (beta)",
            code: r#"/// Get modulation index (beta = deviation / symbol_rate)
pub fn modulation_index(&self) -> f64 {
    self.deviation / self.symbol_rate
}

// Example: 2500 Hz deviation, 1000 sym/s
// beta = 2500 / 1000 = 2.5 (Wideband FM)"#,
            explanation: "**Modulation Index (beta):**\n\n\
                beta = delta-f / f_m\n\n\
                **NBFM (beta < 1):**\n\
                - Bandwidth ~= 2 * f_m (similar to AM)\n\
                - Used in VHF/UHF voice radio\n\
                - Example: 2.5 kHz deviation, 3 kHz voice = 0.83\n\n\
                **WBFM (beta > 1):**\n\
                - Much wider bandwidth\n\
                - Superior audio quality\n\
                - FM broadcast: 75 kHz deviation, 15 kHz audio = 5.0\n\n\
                **Trade-off:**\n\
                Higher beta = better SNR but more bandwidth.",
            concepts: &["Modulation index", "Bandwidth trade-off"],
        },
        CodeSnippet {
            name: "symbol_to_freq_offset()",
            brief: "Map symbols to frequency offsets",
            code: r#"fn symbol_to_freq_offset(&self, symbol: u8) -> f64 {
    if self.num_levels == 2 {
        // Binary: 0 -> -deviation, 1 -> +deviation
        if symbol == 0 { -self.deviation } else { self.deviation }
    } else {
        // Multi-level: evenly spaced frequencies
        let normalized = symbol as f64 / (self.num_levels - 1) as f64;
        let scaled = 2.0 * normalized - 1.0; // -1 to 1
        scaled * self.deviation
    }
}"#,
            explanation: "**Frequency Mapping:**\n\n\
                **Binary FM:**\n\
                - Symbol 0: f_carrier - deviation (mark)\n\
                - Symbol 1: f_carrier + deviation (space)\n\n\
                **4-level FM:**\n\
                With 1000 Hz deviation:\n\
                - Symbol 0: -1000 Hz\n\
                - Symbol 1: -333 Hz\n\
                - Symbol 2: +333 Hz\n\
                - Symbol 3: +1000 Hz\n\n\
                **Note:** Binary FM is identical to FSK (Frequency Shift Keying).\n\
                Multi-level FM is the same as M-FSK.",
            concepts: &["Frequency mapping", "FSK equivalence"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "FM Modulation",
    description: "Generating FM signals",
    snippets: &[
        CodeSnippet {
            name: "generate_symbol()",
            brief: "Generate I/Q samples with continuous phase",
            code: r#"fn generate_symbol(&self, symbol: u8, start_phase: f64) -> (Vec<IQSample>, f64) {
    let sps = self.sps();
    let freq_offset = self.symbol_to_freq_offset(symbol);
    let total_freq = self.carrier_freq + freq_offset;
    let omega = 2.0 * PI * total_freq / self.common.sample_rate;
    let amp = self.common.amplitude;

    let samples: Vec<IQSample> = (0..sps)
        .map(|n| {
            let phase = start_phase + omega * n as f64;
            IQSample::new(amp * phase.cos(), amp * phase.sin())
        })
        .collect();

    // Maintain phase continuity
    let end_phase = start_phase + omega * sps as f64;
    (samples, end_phase)
}"#,
            explanation: "**FM Signal Generation:**\n\n\
                s(t) = A * cos(2*pi*f(t)*t + phi)\n\n\
                **Constant Envelope:**\n\
                Unlike AM, the amplitude never changes. Only frequency varies.\n\
                This is crucial for efficiency - allows Class C amplifiers.\n\n\
                **Phase Continuity:**\n\
                Critical! Without continuous phase:\n\
                - Clicking sounds at symbol boundaries\n\
                - Spectral splatter (wideband interference)\n\
                - Poor BER performance\n\n\
                **Implementation:**\n\
                Track phase across symbols, using end phase as next start.",
            concepts: &["Constant envelope", "Phase continuity", "Spectral efficiency"],
        },
        CodeSnippet {
            name: "Continuous Phase FSK",
            brief: "Why phase continuity matters",
            code: r#"// WITHOUT phase continuity (bad):
for &bit in data {
    let freq = if bit == 0 { f_low } else { f_high };
    // Phase resets to 0 each symbol - creates discontinuities!
    for n in 0..sps {
        let phase = 2.0 * PI * freq * n as f64 / sample_rate;
        // ... generates clicks at transitions
    }
}

// WITH phase continuity (good - CPFSK):
let mut phase = 0.0;
for &bit in data {
    let freq = if bit == 0 { f_low } else { f_high };
    let omega = 2.0 * PI * freq / sample_rate;
    for _ in 0..sps {
        samples.push(IQSample::new(phase.cos(), phase.sin()));
        phase += omega;  // Accumulate, never reset
    }
}"#,
            explanation: "**Continuous Phase FSK (CPFSK):**\n\n\
                Phase must flow continuously across symbol boundaries.\n\n\
                **Why It Matters:**\n\
                Phase discontinuities are like sudden jumps:\n\
                - In time domain: clicks/pops\n\
                - In frequency domain: wideband splatter\n\n\
                **The Fix:**\n\
                Accumulate phase continuously. The frequency determines\n\
                the rate of phase change, not the absolute phase.\n\n\
                **Related Schemes:**\n\
                - MSK: Minimum Shift Keying (deviation = symbol_rate/4)\n\
                - GMSK: Gaussian MSK (used in GSM cellular)",
            concepts: &["CPFSK", "Phase continuity", "MSK", "Spectral containment"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "FM Demodulation",
    description: "Recovering data from FM signals",
    snippets: &[
        CodeSnippet {
            name: "Phase Differentiation",
            brief: "Estimate frequency from phase changes",
            code: r#"fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
    // FM demodulation using phase differentiation
    for chunk in samples.chunks(sps) {
        // Estimate instantaneous frequency from phase differences
        let mut freq_estimates = Vec::new();
        for i in 1..chunk.len() {
            // Multiply by conjugate of previous sample
            let phase_diff = (chunk[i] * chunk[i - 1].conj()).arg();
            let inst_freq = phase_diff * sample_rate / (2.0 * PI);
            freq_estimates.push(inst_freq);
        }

        // Average frequency estimate for this symbol
        let avg_freq = freq_estimates.iter().sum::<f64>()
            / freq_estimates.len() as f64;

        // Decision: find closest frequency level
        let freq_offset = avg_freq - carrier_freq;
        // ... compare to expected levels
    }
}"#,
            explanation: "**FM Discriminator (Phase Differentiation):**\n\n\
                The key insight: frequency is the rate of phase change.\n\
                f = (1/2pi) * d(phase)/dt\n\n\
                **Algorithm:**\n\
                1. Multiply sample by conjugate of previous\n\
                2. Take argument (angle) of result = phase difference\n\
                3. Scale by sample rate to get frequency\n\n\
                **Why Conjugate Multiply?**\n\
                s[n] * conj(s[n-1]) = |s|^2 * exp(j*delta_phase)\n\
                The angle of this IS the phase change.\n\n\
                **Averaging:**\n\
                Average over the symbol period to reduce noise.",
            concepts: &["FM discriminator", "Phase differentiation", "Instantaneous frequency"],
        },
        CodeSnippet {
            name: "Frequency Decision",
            brief: "Decide which symbol was sent",
            code: r#"// Remove carrier to get offset
let freq_offset = avg_freq - self.carrier_freq;

// Decision: find closest frequency level
let mut best_symbol = 0u8;
let mut best_error = f64::MAX;

for sym in 0..self.num_levels as u8 {
    let expected_offset = self.symbol_to_freq_offset(sym);
    let error = (freq_offset - expected_offset).abs();
    if error < best_error {
        best_error = error;
        best_symbol = sym;
    }
}

result.symbols.push(best_symbol as u16);"#,
            explanation: "**Minimum Distance Decision:**\n\n\
                Compare measured frequency to all expected values.\n\
                Choose the closest one.\n\n\
                **Binary Decision:**\n\
                Simple threshold at carrier frequency:\n\
                - freq < carrier: symbol 0\n\
                - freq >= carrier: symbol 1\n\n\
                **Multi-level Decision:**\n\
                Must find closest of N possible frequencies.\n\n\
                **Noise Tolerance:**\n\
                FM's constant envelope means amplitude noise doesn't\n\
                affect decisions. Only frequency/phase noise matters.\n\
                This is why FM is more noise-immune than AM.",
            concepts: &["Minimum distance", "Decision threshold", "Noise immunity"],
        },
        CodeSnippet {
            name: "FM Capture Effect",
            brief: "FM's unique interference behavior",
            code: r#"// FM Capture Effect: Stronger signal "captures" the receiver
//
// If two FM signals are present at different powers:
// - AM: both signals interfere, creating distortion
// - FM: stronger signal dominates completely
//
// This happens because FM limiters clip amplitude variations.
// The frequency of the stronger signal dominates.
//
// Rule of thumb: ~6dB difference = full capture
//
// Practical effect:
// - Good: Nearby stations don't interfere as much
// - Bad: Weak station can be completely masked"#,
            explanation: "**FM Capture Effect:**\n\n\
                A unique property where the stronger of two co-channel\n\
                signals completely dominates the weaker one.\n\n\
                **Why It Happens:**\n\
                FM receivers use limiters that clip amplitude.\n\
                The resulting phase/frequency is dominated by\n\
                the stronger signal's frequency.\n\n\
                **Threshold:**\n\
                ~6 dB power difference = full capture.\n\
                Below this, both signals may be heard.\n\n\
                **Implications:**\n\
                - FM broadcast: less co-channel interference\n\
                - Aviation: AM used to hear multiple aircraft\n\
                - Digital: improves BER in interference",
            concepts: &["Capture effect", "FM limiting", "Co-channel interference"],
        },
    ],
};
