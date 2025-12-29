//! FSK (Frequency Shift Keying) code snippets
//!
//! FSK encodes data by shifting the carrier frequency. Binary FSK uses two
//! frequencies, while M-FSK uses M frequencies for higher data rates.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete FSK waveform code documentation
pub static FSK_CODE: WaveformCode = WaveformCode {
    waveform_id: "FSK",
    display_name: "Frequency Shift Keying (FSK)",
    introduction: "FSK encodes digital data by shifting the carrier frequency. \
        A '1' might use frequency f+Δf while '0' uses f-Δf. This is more robust than OOK \
        because we're detecting frequency differences rather than presence/absence of signal. \
        FSK is used in everything from old modems to modern LoRa.",
    complexity: 3,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &GENERATION_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "FSK Fundamentals",
    description: "Core concepts and parameters for FSK modulation",
    snippets: &[
        CodeSnippet {
            name: "symbol_to_freq()",
            brief: "Map a symbol (0 to M-1) to its frequency offset",
            code: r#"fn symbol_to_freq(&self, symbol: u16) -> f64 {
    // Map symbol 0..M-1 to frequencies centered around carrier
    // Symbol 0 -> -deviation * (M-1)/2
    // Symbol M-1 -> +deviation * (M-1)/2
    let m = self.bits_per_symbol.pow(2) as f64;
    let normalized = (symbol as f64 - (m - 1.0) / 2.0) / ((m - 1.0) / 2.0);
    self.deviation * normalized
}"#,
            explanation: "**Symbol-to-Frequency Mapping** centers the frequencies around the carrier.\n\n\
                \
                **For Binary FSK (M=2):**\n\
                - Symbol 0 → carrier - deviation\n\
                - Symbol 1 → carrier + deviation\n\n\
                \
                **For 4-FSK (M=4):**\n\
                - Symbol 0 → carrier - 1.5×deviation\n\
                - Symbol 1 → carrier - 0.5×deviation\n\
                - Symbol 2 → carrier + 0.5×deviation\n\
                - Symbol 3 → carrier + 1.5×deviation\n\n\
                \
                **Why Center Around Zero?**\n\
                Centering keeps the average frequency at the carrier, which:\n\
                - Minimizes DC offset issues\n\
                - Simplifies filtering\n\
                - Balances spectrum usage",
            concepts: &["Symbol mapping", "Frequency deviation", "M-ary FSK"],
        },
        CodeSnippet {
            name: "modulation_index()",
            brief: "Calculate the modulation index h = 2Δf/Rs",
            code: r#"pub fn modulation_index(&self) -> f64 {
    // h = 2 * deviation / symbol_rate
    // h = 0.5 is MSK (Minimum Shift Keying)
    // h = 1.0 gives orthogonal signaling
    2.0 * self.deviation / self.symbol_rate
}"#,
            explanation: "**Modulation Index (h)** is the key FSK parameter.\n\n\
                \
                **Definition:**\n\
                h = 2 × Δf / Rs\n\n\
                Where Δf is frequency deviation and Rs is symbol rate.\n\n\
                \
                **Special Values:**\n\
                - h = 0.5: MSK (Minimum Shift Keying) - most bandwidth efficient\n\
                - h = 1.0: Orthogonal FSK - symbols don't interfere\n\
                - h > 1.0: Wideband FSK - easier detection but uses more spectrum\n\n\
                \
                **The Carson Rule:**\n\
                FSK bandwidth ≈ 2×(Δf + Rs) = Rs×(h + 1)\n\
                Lower h = narrower bandwidth but harder detection",
            concepts: &["Modulation index", "MSK", "Carson bandwidth rule", "Orthogonality"],
        },
    ],
};

static GENERATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Symbol Generation",
    description: "Creating I/Q samples for each FSK symbol",
    snippets: &[
        CodeSnippet {
            name: "generate_symbol()",
            brief: "Generate one symbol's worth of I/Q samples with phase continuity",
            code: r#"fn generate_symbol(&self, symbol: u16, start_phase: f64) -> (Vec<IQSample>, f64) {
    let sps = self.sps();
    let freq_offset = self.symbol_to_freq(symbol);

    // Total frequency = carrier + offset for this symbol
    let omega = 2.0 * PI * (self.carrier_freq + freq_offset)
        / self.common.sample_rate;
    let amp = self.common.amplitude;

    let samples: Vec<IQSample> = (0..sps)
        .map(|n| {
            let phase = start_phase + omega * n as f64;
            IQSample::new(amp * phase.cos(), amp * phase.sin())
        })
        .collect();

    // Return ending phase for seamless transitions
    let end_phase = start_phase + omega * sps as f64;
    (samples, end_phase)
}"#,
            explanation: "**Continuous-Phase FSK (CPFSK)** is crucial for spectral efficiency.\n\n\
                \
                **Why Phase Continuity Matters:**\n\
                If we abruptly changed frequency at symbol boundaries:\n\
                - Phase discontinuities create 'clicks'\n\
                - Wideband spectral splatter\n\
                - Interference with adjacent channels\n\n\
                \
                **The Algorithm:**\n\
                1. Compute frequency for this symbol (carrier + offset)\n\
                2. Convert to angular frequency ω = 2πf/Fs\n\
                3. Generate samples continuing from start_phase\n\
                4. Return ending phase for next symbol\n\n\
                \
                **Phase Accumulation:**\n\
                Each sample advances phase by ω radians. Different symbols have \
                different ω values, causing the phase to 'rotate' faster or slower.",
            concepts: &["CPFSK", "Phase continuity", "Angular frequency", "Symbol generation"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Modulation",
    description: "Converting bits/symbols to I/Q samples",
    snippets: &[
        CodeSnippet {
            name: "modulate()",
            brief: "The complete FSK modulation pipeline",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let symbols = self.bits_to_symbols(data);
    let mut samples = Vec::with_capacity(symbols.len() * self.sps());
    let mut phase = 0.0;

    for symbol in symbols {
        let (symbol_samples, new_phase) = self.generate_symbol(symbol, phase);
        samples.extend(symbol_samples);
        phase = new_phase;  // Maintain phase continuity
    }

    samples
}"#,
            explanation: "The modulation pipeline converts bits to RF samples.\n\n\
                \
                **Pipeline Steps:**\n\
                1. **Bit Packing:** Group bits into symbols (1 bit for BFSK, 2 for 4-FSK)\n\
                2. **Symbol Generation:** Each symbol → frequency → I/Q samples\n\
                3. **Phase Threading:** Pass phase from symbol to symbol\n\n\
                \
                **Memory Optimization:**\n\
                `Vec::with_capacity()` pre-allocates the exact size needed:\n\
                num_symbols × samples_per_symbol\n\n\
                \
                **The Phase Variable:**\n\
                This single `phase` variable is the 'state' of our modulator. \
                It accumulates continuously, creating CPFSK rather than \
                discontinuous FSK.",
            concepts: &["Bit-to-symbol mapping", "CPFSK modulation", "Pipeline architecture"],
        },
        CodeSnippet {
            name: "bits_to_symbols()",
            brief: "Pack input bits into M-ary symbols",
            code: r#"fn bits_to_symbols(&self, bits: &[u8]) -> Vec<u16> {
    bits.chunks(self.bits_per_symbol as usize)
        .map(|chunk| {
            // Pack bits MSB-first into symbol value
            chunk.iter().fold(0u16, |acc, &bit| (acc << 1) | (bit as u16))
        })
        .collect()
}"#,
            explanation: "**Bit Packing** groups multiple bits into one symbol.\n\n\
                \
                **For 4-FSK (2 bits per symbol):**\n\
                - Bits [0,0] → Symbol 0\n\
                - Bits [0,1] → Symbol 1\n\
                - Bits [1,0] → Symbol 2\n\
                - Bits [1,1] → Symbol 3\n\n\
                \
                **MSB-First Packing:**\n\
                The fold accumulates bits left-to-right:\n\
                (acc << 1) | bit shifts existing bits left and adds new bit\n\n\
                \
                **Why M-ary?**\n\
                4-FSK transmits 2 bits per symbol, doubling throughput \
                compared to binary FSK at the same symbol rate. \
                Trade-off: requires more bandwidth and SNR.",
            concepts: &["Bit packing", "M-ary symbols", "MSB-first", "Throughput vs. bandwidth"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Demodulation",
    description: "Recovering symbols and bits from received I/Q samples",
    snippets: &[
        CodeSnippet {
            name: "demodulate() - Frequency Estimation",
            brief: "Estimate instantaneous frequency using phase differentiation",
            code: r#"// Process each symbol period
for chunk in samples.chunks(sps) {
    // Estimate frequency using phase differentiation
    let mut freq_sum = 0.0;
    let mut count = 0;

    for i in 1..chunk.len() {
        // Phase difference = arg(s[n] * conj(s[n-1]))
        let phase_diff = (chunk[i] * chunk[i - 1].conj()).arg();
        // Convert to frequency: f = (Δφ × Fs) / (2π)
        let inst_freq = phase_diff * self.common.sample_rate / (2.0 * PI);
        freq_sum += inst_freq;
        count += 1;
    }

    let avg_freq = freq_sum / count as f64;
    // ... symbol decision follows
}"#,
            explanation: "**Phase Differentiation** - the elegant FSK demodulator.\n\n\
                \
                **The Math:**\n\
                When you multiply sample s[n] by the conjugate of s[n-1]:\n\
                s[n] × conj(s[n-1]) = |s|² × e^(j×Δφ)\n\n\
                The argument (angle) of this product IS the phase change.\n\n\
                \
                **Phase to Frequency:**\n\
                f = (Δφ × Fs) / (2π)\n\n\
                If phase advances by 0.1 radians/sample at 10kHz sample rate:\n\
                f = (0.1 × 10000) / (2π) ≈ 159 Hz\n\n\
                \
                **Why Average?**\n\
                Noise causes random phase errors. Averaging over the symbol \
                period reduces variance and gives a reliable frequency estimate.\n\n\
                \
                **No FFT Required!**\n\
                This works sample-by-sample, making it computationally efficient \
                and suitable for real-time processing.",
            concepts: &[
                "Phase differentiation",
                "Instantaneous frequency",
                "Complex conjugate",
                "Noise averaging",
            ],
        },
        CodeSnippet {
            name: "demodulate() - Symbol Decision",
            brief: "Map estimated frequency to nearest valid symbol",
            code: r#"// Find which symbol frequency is closest
let mut best_symbol = 0u16;
let mut best_diff = f64::MAX;

for sym in 0..self.bits_per_symbol.pow(2) {
    let expected_freq = self.carrier_freq + self.symbol_to_freq(sym);
    let diff = (avg_freq - expected_freq).abs();
    if diff < best_diff {
        best_diff = diff;
        best_symbol = sym;
    }
}

result.symbols.push(best_symbol);"#,
            explanation: "**Nearest-Neighbor Detection** - simple but effective.\n\n\
                \
                **The Algorithm:**\n\
                1. For each valid symbol (0 to M-1)\n\
                2. Calculate its expected frequency\n\
                3. Compare to measured frequency\n\
                4. Choose the symbol with smallest difference\n\n\
                \
                **Decision Regions:**\n\
                For binary FSK, the decision boundary is at the carrier frequency:\n\
                - f < carrier → symbol 0\n\
                - f > carrier → symbol 1\n\n\
                \
                **For 4-FSK:**\n\
                Four decision regions divide the frequency axis.\n\
                Symbol boundaries are midpoints between adjacent frequencies.\n\n\
                \
                **Why It Works:**\n\
                AWGN noise is symmetric, so the closest frequency is the \
                most likely transmitted symbol (maximum likelihood detection).",
            concepts: &[
                "Nearest-neighbor detection",
                "Decision regions",
                "Maximum likelihood",
                "Symbol mapping",
            ],
        },
        CodeSnippet {
            name: "symbols_to_bits()",
            brief: "Unpack M-ary symbols back to bits",
            code: r#"fn symbols_to_bits(&self, symbols: &[u16]) -> Vec<u8> {
    let bps = self.bits_per_symbol as usize;
    let mut bits = Vec::with_capacity(symbols.len() * bps);

    for &symbol in symbols {
        // Extract bits MSB-first
        for i in (0..bps).rev() {
            bits.push(((symbol >> i) & 1) as u8);
        }
    }

    bits
}"#,
            explanation: "**Bit Unpacking** reverses the modulator's packing.\n\n\
                \
                **For 4-FSK (2 bits per symbol):**\n\
                - Symbol 0 → [0, 0]\n\
                - Symbol 1 → [0, 1]\n\
                - Symbol 2 → [1, 0]\n\
                - Symbol 3 → [1, 1]\n\n\
                \
                **The Algorithm:**\n\
                For each bit position (MSB to LSB):\n\
                1. Shift symbol right by bit position\n\
                2. Mask with &1 to get that bit\n\
                3. Append to output\n\n\
                \
                **MSB-First Order:**\n\
                `(0..bps).rev()` iterates from highest bit to lowest, \
                matching how bits were packed in the modulator.",
            concepts: &["Bit unpacking", "MSB-first extraction", "Symbol-to-bits"],
        },
    ],
};
