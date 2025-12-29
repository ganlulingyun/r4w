//! AM (Amplitude Modulation) code snippets
//!
//! AM is one of the oldest modulation techniques, encoding information
//! in the amplitude of a carrier wave.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete AM waveform code documentation
pub static AM_CODE: WaveformCode = WaveformCode {
    waveform_id: "AM",
    display_name: "Amplitude Modulation (AM)",
    introduction: "AM encodes information by varying the amplitude of a carrier wave while \
        keeping frequency constant. Invented for voice transmission in the early 1900s, \
        it remains one of the most intuitive modulation schemes. The key parameter is \
        the modulation index (m), which determines how much the amplitude varies.",
    complexity: 2,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "AM Fundamentals",
    description: "Core concepts of amplitude modulation",
    snippets: &[
        CodeSnippet {
            name: "AM struct",
            brief: "AM modulator/demodulator parameters",
            code: r#"pub struct AM {
    common: CommonParams,
    symbol_rate: f64,
    carrier_freq: f64,
    /// Modulation index (0.0 to 1.0 typical, >1 for over-modulation)
    modulation_index: f64,
    /// Number of amplitude levels (2 for binary, 4 for 4-AM)
    num_levels: usize,
    /// Whether to suppress carrier (DSB-SC mode)
    suppress_carrier: bool,
}"#,
            explanation: "**Key Parameters:**\n\n\
                **Modulation Index (m):**\n\
                - m = 0: No modulation (pure carrier)\n\
                - m = 1: 100% modulation (envelope touches zero)\n\
                - m > 1: Over-modulation (causes distortion)\n\n\
                **AM Variants:**\n\
                - **DSB-AM**: Double Sideband with carrier (standard AM radio)\n\
                - **DSB-SC**: Suppressed Carrier (more power efficient)\n\
                - **SSB**: Single Sideband (most bandwidth efficient)\n\n\
                **Digital AM:**\n\
                For digital data, bits map to amplitude levels:\n\
                - Binary AM: 2 levels (1 bit/symbol)\n\
                - 4-AM (PAM-4): 4 levels (2 bits/symbol)",
            concepts: &["Modulation index", "DSB-AM", "DSB-SC", "PAM"],
        },
        CodeSnippet {
            name: "symbol_to_amplitude()",
            brief: "Map digital symbols to amplitude levels",
            code: r#"fn symbol_to_amplitude(&self, symbol: u8) -> f64 {
    if self.num_levels == 2 {
        // Binary: 0 -> low amplitude, 1 -> high amplitude
        if symbol == 0 {
            1.0 - self.modulation_index
        } else {
            1.0 + self.modulation_index
        }
    } else {
        // Multi-level: map symbol to amplitude
        let normalized = symbol as f64 / (self.num_levels - 1) as f64;
        let modulated = 2.0 * normalized - 1.0; // -1 to 1
        1.0 + self.modulation_index * modulated
    }
}"#,
            explanation: "**Amplitude Mapping:**\n\n\
                The amplitude varies between (1-m) and (1+m) times the carrier.\n\n\
                **Binary AM:**\n\
                - Bit 0: amplitude = 1 - m (low)\n\
                - Bit 1: amplitude = 1 + m (high)\n\n\
                **Example (m = 0.5):**\n\
                - Bit 0: amplitude = 0.5\n\
                - Bit 1: amplitude = 1.5\n\n\
                **Multi-level AM:**\n\
                Symbols 0,1,2,3 map to evenly-spaced amplitudes.\n\
                This is also called Pulse Amplitude Modulation (PAM).",
            concepts: &["Amplitude levels", "PAM", "Symbol mapping"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "AM Modulation",
    description: "Generating AM signals",
    snippets: &[
        CodeSnippet {
            name: "generate_symbol()",
            brief: "Generate I/Q samples for one symbol",
            code: r#"fn generate_symbol(&self, symbol: u8, start_phase: f64) -> (Vec<IQSample>, f64) {
    let sps = self.sps();
    let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
    let base_amp = self.common.amplitude;

    let envelope = if self.suppress_carrier {
        // DSB-SC: amplitude varies from -m to +m
        let normalized = if symbol == 0 { -1.0 } else { 1.0 };
        normalized * self.modulation_index
    } else {
        // Standard AM: DC offset + modulation
        self.symbol_to_amplitude(symbol)
    };

    let samples: Vec<IQSample> = (0..sps)
        .map(|n| {
            let phase = start_phase + omega * n as f64;
            let amp = base_amp * envelope;
            IQSample::new(amp * phase.cos(), amp * phase.sin())
        })
        .collect();

    let end_phase = start_phase + omega * sps as f64;
    (samples, end_phase)
}"#,
            explanation: "**AM Signal Equation:**\n\n\
                s(t) = [1 + m·x(t)] · A · cos(2πfct)\n\n\
                **Standard AM (DSB-AM):**\n\
                The carrier is always present, with amplitude varying around it.\n\
                Envelope = 1 ± m (always positive).\n\n\
                **Suppressed Carrier (DSB-SC):**\n\
                No DC component - amplitude goes negative.\n\
                More power efficient but requires coherent detection.\n\n\
                **Phase Continuity:**\n\
                We track phase across symbols to avoid discontinuities,\n\
                which would create spectral splatter.",
            concepts: &["Envelope modulation", "Phase continuity", "DSB-SC"],
        },
        CodeSnippet {
            name: "modulate()",
            brief: "Modulate a sequence of bits",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let bits_per_symbol = (self.num_levels as f64).log2() as usize;
    let mut samples = Vec::new();
    let mut phase = 0.0;

    if bits_per_symbol == 1 {
        // Binary AM
        for &bit in data {
            let symbol = bit & 1;
            let (sym_samples, new_phase) = self.generate_symbol(symbol, phase);
            samples.extend(sym_samples);
            phase = new_phase;
        }
    } else {
        // Multi-level AM: group bits into symbols
        for chunk in data.chunks(bits_per_symbol) {
            let mut symbol = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                symbol |= (bit & 1) << (bits_per_symbol - 1 - i);
            }
            let (sym_samples, new_phase) = self.generate_symbol(symbol, phase);
            samples.extend(sym_samples);
            phase = new_phase;
        }
    }
    samples
}"#,
            explanation: "**Bit-to-Symbol Mapping:**\n\n\
                **Binary AM:** Each bit becomes one symbol directly.\n\n\
                **4-AM:** Two bits combine into one symbol:\n\
                - 00 → symbol 0 (lowest amplitude)\n\
                - 01 → symbol 1\n\
                - 10 → symbol 2\n\
                - 11 → symbol 3 (highest amplitude)\n\n\
                **Throughput:**\n\
                Multi-level AM increases bit rate without increasing symbol rate.\n\
                4-AM carries 2× the data of binary AM at the same bandwidth.",
            concepts: &["Bit packing", "Symbol rate", "Spectral efficiency"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "AM Demodulation",
    description: "Recovering data from AM signals",
    snippets: &[
        CodeSnippet {
            name: "demodulate() - Envelope Detection",
            brief: "Simple envelope detection for AM",
            code: r#"fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
    let sps = self.sps();

    // Envelope detection: measure amplitude in each symbol period
    let envelopes: Vec<f64> = samples.chunks(sps)
        .map(|chunk| {
            // RMS amplitude
            let power: f64 = chunk.iter()
                .map(|s| s.norm_sqr())
                .sum::<f64>() / chunk.len() as f64;
            power.sqrt()
        })
        .collect();

    // Decision: threshold between amplitude levels
    let expected_levels: Vec<f64> = (0..self.num_levels)
        .map(|i| self.common.amplitude * self.symbol_to_amplitude(i as u8))
        .collect();

    let threshold = (expected_levels[0] + expected_levels[1]) / 2.0;

    for &env in &envelopes {
        let bit = if env > threshold { 1u8 } else { 0u8 };
        result.bits.push(bit);
    }
    result
}"#,
            explanation: "**Envelope Detection:**\n\n\
                The simplest AM demodulator extracts the envelope (amplitude).\n\n\
                **Steps:**\n\
                1. **Measure amplitude** in each symbol period (RMS)\n\
                2. **Compare to threshold** between expected levels\n\
                3. **Decide symbol** based on which level is closest\n\n\
                **Why RMS?**\n\
                Root-Mean-Square gives average amplitude over the period,\n\
                smoothing out carrier oscillations.\n\n\
                **Practical Envelope Detectors:**\n\
                - Analog: diode + RC filter (AM radio)\n\
                - Digital: magnitude calculation |I + jQ|",
            concepts: &["Envelope detection", "RMS", "Threshold decision"],
        },
        CodeSnippet {
            name: "Multi-level Decision",
            brief: "Decide between multiple amplitude levels",
            code: r#"// Multi-level: find closest expected level
let range = max_level - min_level;
let step = range / (self.num_levels - 1) as f64;

for &env in &envelopes {
    // Normalize and find closest level
    let normalized = (env - min_level) / step;
    let symbol = normalized.round()
        .min((self.num_levels - 1) as f64)
        .max(0.0) as u8;

    result.symbols.push(symbol as u16);

    // Convert symbol back to bits
    let bits_per_sym = (self.num_levels as f64).log2() as usize;
    for i in (0..bits_per_sym).rev() {
        result.bits.push((symbol >> i) & 1);
    }
}"#,
            explanation: "**Multi-level Decision:**\n\n\
                With more than 2 levels, we find the closest expected amplitude.\n\n\
                **Decision Regions:**\n\
                For 4-AM with levels [0.5, 0.83, 1.17, 1.5]:\n\
                - 0.00-0.67: symbol 0\n\
                - 0.67-1.00: symbol 1\n\
                - 1.00-1.33: symbol 2\n\
                - 1.33+: symbol 3\n\n\
                **SNR Requirements:**\n\
                More levels require higher SNR to distinguish them.\n\
                4-AM needs ~6dB more SNR than binary AM for same BER.",
            concepts: &["Decision regions", "Quantization", "SNR requirements"],
        },
    ],
};
