//! PSK (Phase Shift Keying) code snippets
//!
//! PSK encodes data in the phase of the carrier. BPSK uses 2 phases (0° and 180°),
//! QPSK uses 4 phases, and higher orders pack more bits per symbol.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete PSK waveform code documentation
pub static PSK_CODE: WaveformCode = WaveformCode {
    waveform_id: "PSK",
    display_name: "Phase Shift Keying (PSK)",
    introduction: "PSK encodes data in the phase of the carrier signal. Unlike FSK which changes \
        frequency, PSK keeps frequency constant but shifts the phase. BPSK (2 phases) is the most \
        robust, QPSK (4 phases) doubles the data rate, and higher orders trade robustness for throughput. \
        PSK is used in Wi-Fi, satellite communications, and many modern protocols.",
    complexity: 3,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &CONSTELLATION_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "PSK Fundamentals",
    description: "Core concepts: Gray coding, constellation points, and phase mapping",
    snippets: &[
        CodeSnippet {
            name: "Gray Coding Concept",
            brief: "Why adjacent symbols differ by only one bit",
            code: r#"// Gray code ensures adjacent constellation points
// differ by exactly 1 bit.
//
// Binary:  00  01  10  11
// Gray:    00  01  11  10
//
// QPSK constellation with Gray coding:
//        01 (90°)
//           |
//  00 ------+------ 11
// (180°)    |      (0°)
//           |
//        10 (270°)

fn gray_encode(n: u16) -> u16 {
    n ^ (n >> 1)
}

fn gray_decode(g: u16) -> u16 {
    let mut n = g;
    let mut mask = n >> 1;
    while mask != 0 {
        n ^= mask;
        mask >>= 1;
    }
    n
}"#,
            explanation: "**Gray Coding** is crucial for PSK performance.\n\n\
                \
                **The Problem:**\n\
                Without Gray coding, if the receiver mistakes symbol 01 for 10, \
                that's 2 bit errors! Phase errors are most likely between \
                adjacent constellation points.\n\n\
                \
                **The Solution:**\n\
                Gray code ensures adjacent symbols differ by only 1 bit:\n\
                - A small phase error causes at most 1 bit error\n\
                - Reduces BER by ~50% compared to natural binary\n\n\
                \
                **The XOR Trick:**\n\
                Gray encode: g = n XOR (n >> 1)\n\
                This simple operation creates the Gray code pattern.\n\n\
                \
                **Example (QPSK):**\n\
                - 0° = 11 (Gray), adjacent to 01 and 10 (each differs by 1 bit)\n\
                - 90° = 01, adjacent to 11 and 00\n\
                - 180° = 00, adjacent to 01 and 10\n\
                - 270° = 10, adjacent to 00 and 11",
            concepts: &["Gray coding", "Adjacent symbols", "Bit error reduction", "XOR encoding"],
        },
        CodeSnippet {
            name: "bits_per_symbol()",
            brief: "Relationship between constellation size and bits",
            code: r#"fn bits_per_symbol(&self) -> usize {
    // M-PSK: M = 2^bits_per_symbol
    // BPSK:  M=2,  bits=1
    // QPSK:  M=4,  bits=2
    // 8-PSK: M=8,  bits=3
    // 16-PSK: M=16, bits=4
    match self.order {
        2 => 1,    // BPSK
        4 => 2,    // QPSK
        8 => 3,    // 8-PSK
        16 => 4,   // 16-PSK
        _ => (self.order as f64).log2() as usize,
    }
}"#,
            explanation: "**Order vs. Bits** - the fundamental PSK trade-off.\n\n\
                \
                **The Relationship:**\n\
                M = 2^(bits_per_symbol)\n\
                bits_per_symbol = log₂(M)\n\n\
                \
                **Common PSK Orders:**\n\
                | Order | Bits/Symbol | Phase Spacing |\n\
                |-------|-------------|---------------|\n\
                | BPSK  | 1           | 180°          |\n\
                | QPSK  | 2           | 90°           |\n\
                | 8-PSK | 3           | 45°           |\n\
                | 16-PSK| 4           | 22.5°         |\n\n\
                \
                **The Trade-off:**\n\
                Higher order = more bits per symbol = higher data rate\n\
                BUT phases are closer together = more sensitive to noise\n\n\
                \
                BPSK is most robust (180° separation), while 16-PSK is \
                very sensitive (only 22.5° between phases).",
            concepts: &["M-ary modulation", "Bits per symbol", "SNR requirements", "Throughput trade-off"],
        },
    ],
};

static CONSTELLATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Constellation Points",
    description: "Computing the constellation diagram for PSK",
    snippets: &[
        CodeSnippet {
            name: "compute_constellation()",
            brief: "Pre-compute constellation points for all symbols",
            code: r#"fn compute_constellation(&self) -> Vec<IQSample> {
    let order = self.order as usize;
    (0..order)
        .map(|i| {
            // Phase for symbol i: 2πi/M + offset
            let phase = 2.0 * PI * i as f64 / order as f64
                + self.phase_offset;
            IQSample::new(
                self.common.amplitude * phase.cos(),
                self.common.amplitude * phase.sin(),
            )
        })
        .collect()
}"#,
            explanation: "**Constellation Points** are the valid I/Q values for each symbol.\n\n\
                \
                **The Geometry:**\n\
                For M-PSK, symbols are evenly spaced around a circle:\n\
                phase[i] = 2π × i / M + offset\n\n\
                \
                **QPSK Example (no offset):**\n\
                - Symbol 0: phase=0° → I=1, Q=0\n\
                - Symbol 1: phase=90° → I=0, Q=1\n\
                - Symbol 2: phase=180° → I=-1, Q=0\n\
                - Symbol 3: phase=270° → I=0, Q=-1\n\n\
                \
                **Phase Offset:**\n\
                Some systems use π/4 offset for QPSK (45°, 135°, 225°, 315°)\n\
                to avoid transitions through zero (helpful for amplitude-sensitive systems).\n\n\
                \
                **Pre-computation:**\n\
                We compute all M points once, then just look them up during modulation. \
                This trades memory for speed.",
            concepts: &["Constellation diagram", "Phase spacing", "I/Q coordinates", "Pre-computation"],
        },
        CodeSnippet {
            name: "bits_to_symbol()",
            brief: "Convert bits to constellation index using Gray code",
            code: r#"fn bits_to_symbol(&self, bits: &[u8]) -> u16 {
    // Pack bits into binary value
    let binary: u16 = bits.iter()
        .fold(0, |acc, &b| (acc << 1) | (b as u16));

    // Apply Gray coding for optimal bit-error performance
    Self::gray_encode(binary)
}"#,
            explanation: "**Bit-to-Symbol Mapping** with Gray coding.\n\n\
                \
                **Step 1: Pack Bits**\n\
                [0,1,1] → binary 011 = 3\n\
                Using fold with shift-and-OR pattern.\n\n\
                \
                **Step 2: Gray Encode**\n\
                binary 3 (011) → Gray 2 (010)\n\n\
                \
                **Why Gray Code Here?**\n\
                The Gray-coded value becomes the constellation index. \
                Adjacent indices (like 1 and 2) map to adjacent phases. \
                Since Gray-adjacent values differ by 1 bit, a phase error \
                to an adjacent symbol causes only 1 bit error.\n\n\
                \
                **The Full Chain:**\n\
                bits → binary → Gray code → constellation index → I/Q point",
            concepts: &["Bit packing", "Gray encoding", "Constellation mapping"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Modulation",
    description: "Converting symbols to I/Q samples",
    snippets: &[
        CodeSnippet {
            name: "modulate()",
            brief: "The complete PSK modulation pipeline",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let constellation = self.compute_constellation();
    let bps = self.bits_per_symbol();
    let sps = self.sps();

    let mut samples = Vec::with_capacity(
        (data.len() / bps) * sps
    );

    // Process input bits in groups
    for bit_chunk in data.chunks(bps) {
        // Map bits to constellation point
        let symbol = self.bits_to_symbol(bit_chunk);
        let point = constellation[symbol as usize];

        // Hold the constellation point for one symbol period
        // (PSK uses constant phase per symbol)
        for _ in 0..sps {
            samples.push(point);
        }
    }

    samples
}"#,
            explanation: "**PSK Modulation** is elegant in its simplicity.\n\n\
                \
                **The Pipeline:**\n\
                1. Compute constellation points once\n\
                2. Group input bits (e.g., 2 bits for QPSK)\n\
                3. Map each group to a constellation point\n\
                4. Repeat that point for the symbol duration\n\n\
                \
                **Symbol Holding:**\n\
                Unlike FSK which generates a changing waveform per symbol, \
                PSK simply repeats the same I/Q point for `sps` samples. \
                The phase stays constant during each symbol.\n\n\
                \
                **No Phase Accumulation?**\n\
                This is baseband PSK. The constellation points ARE the signal. \
                In a real radio, these would be mixed with a carrier, but \
                for simulation and SDR, baseband is sufficient.\n\n\
                \
                **Memory Efficiency:**\n\
                We pre-compute the constellation and just look up points. \
                No trig functions during the main loop.",
            concepts: &["Symbol holding", "Constellation lookup", "Baseband modulation", "No phase tracking"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Demodulation",
    description: "Recovering symbols and bits from received samples",
    snippets: &[
        CodeSnippet {
            name: "demodulate() - Sample Averaging",
            brief: "Average samples within each symbol period",
            code: r#"let sps = self.sps();

// Average samples in each symbol period
let symbol_averages: Vec<IQSample> = samples.chunks(sps)
    .map(|chunk| {
        let sum_i: f64 = chunk.iter().map(|s| s.i()).sum();
        let sum_q: f64 = chunk.iter().map(|s| s.q()).sum();
        let n = chunk.len() as f64;
        IQSample::new(sum_i / n, sum_q / n)
    })
    .collect();"#,
            explanation: "**Sample Averaging** reduces noise before symbol detection.\n\n\
                \
                **Why Average?**\n\
                Noise is random - averaging multiple samples:\n\
                - Signal adds constructively (same value each time)\n\
                - Noise partially cancels (random + random → smaller)\n\n\
                \
                **SNR Improvement:**\n\
                Averaging N samples improves SNR by √N.\n\
                With 10 samples/symbol: √10 ≈ 3.2× improvement (5 dB)\n\n\
                \
                **Matched Filter Concept:**\n\
                For constant-amplitude PSK, averaging is actually a simple \
                matched filter - optimal detection for AWGN noise.\n\n\
                \
                **Output:**\n\
                One averaged I/Q point per symbol, ready for constellation detection.",
            concepts: &["Noise averaging", "SNR improvement", "Matched filter", "Symbol integration"],
        },
        CodeSnippet {
            name: "demodulate() - Nearest Constellation Point",
            brief: "Find the closest constellation point to each received sample",
            code: r#"let constellation = self.compute_constellation();

for avg in &symbol_averages {
    // Find nearest constellation point
    let (symbol, min_dist) = constellation.iter()
        .enumerate()
        .map(|(i, &point)| {
            // Euclidean distance in I/Q plane
            let di = avg.i() - point.i();
            let dq = avg.q() - point.q();
            let dist = (di * di + dq * dq).sqrt();
            (i as u16, dist)
        })
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap();

    result.symbols.push(symbol);
    distances.push(min_dist);  // For EVM calculation
}"#,
            explanation: "**Nearest-Point Detection** - maximum likelihood for PSK.\n\n\
                \
                **The Algorithm:**\n\
                For each received point:\n\
                1. Calculate distance to each constellation point\n\
                2. Choose the closest one\n\
                3. That's our detected symbol\n\n\
                \
                **Euclidean Distance:**\n\
                d = √[(Ir-Ic)² + (Qr-Qc)²]\n\
                Where (Ir,Qr) is received and (Ic,Qc) is constellation.\n\n\
                \
                **Decision Regions:**\n\
                Each constellation point 'owns' the region of the I/Q plane \
                closest to it. For QPSK, these are 90° pie slices.\n\n\
                \
                **Error Distance:**\n\
                We save the minimum distance for EVM calculation - \
                it tells us how far each received point was from ideal.",
            concepts: &[
                "Euclidean distance",
                "Maximum likelihood",
                "Decision regions",
                "Error vector",
            ],
        },
        CodeSnippet {
            name: "symbol_to_bits()",
            brief: "Convert detected symbol back to bits using Gray decode",
            code: r#"fn symbol_to_bits(&self, symbol: u16) -> Vec<u8> {
    let bps = self.bits_per_symbol();

    // Gray decode to get original binary
    let binary = Self::gray_decode(symbol);

    // Extract bits MSB-first
    (0..bps)
        .rev()
        .map(|i| ((binary >> i) & 1) as u8)
        .collect()
}"#,
            explanation: "**Symbol-to-Bits** reverses the modulator's encoding.\n\n\
                \
                **The Steps:**\n\
                1. Gray decode the constellation index\n\
                2. Extract individual bits\n\n\
                \
                **Gray Decode:**\n\
                Constellation index → Gray decode → original binary → bits\n\n\
                \
                **Bit Extraction:**\n\
                For 3 bits: (binary >> 2) & 1, (binary >> 1) & 1, (binary >> 0) & 1\n\
                Gives MSB to LSB order.\n\n\
                \
                **Symmetry:**\n\
                This exactly reverses bits_to_symbol():\n\
                bits → binary → Gray → symbol → Gray⁻¹ → binary → bits",
            concepts: &["Gray decoding", "Bit extraction", "Inverse mapping"],
        },
        CodeSnippet {
            name: "EVM Calculation",
            brief: "Estimate signal quality from error vectors",
            code: r#"// EVM = RMS of error distances / reference amplitude
let avg_amplitude = self.common.amplitude;
let sum_sq_dist: f64 = distances.iter().map(|d| d * d).sum();
let rms_error = (sum_sq_dist / distances.len() as f64).sqrt();

// EVM as percentage
let evm_percent = (rms_error / avg_amplitude) * 100.0;
result.metadata.insert("evm_percent".to_string(), evm_percent);"#,
            explanation: "**EVM (Error Vector Magnitude)** measures signal quality.\n\n\
                \
                **What It Measures:**\n\
                How far received points are from ideal constellation points, \
                normalized to signal amplitude.\n\n\
                \
                **The Formula:**\n\
                EVM = RMS(error distances) / reference amplitude × 100%\n\n\
                \
                **Interpretation:**\n\
                - EVM < 5%: Excellent signal quality\n\
                - EVM 5-10%: Good\n\
                - EVM 10-20%: Marginal\n\
                - EVM > 20%: Poor, errors likely\n\n\
                \
                **Why RMS?**\n\
                Root-Mean-Square averages the squared errors, then takes \
                the square root. This gives more weight to larger errors \
                and matches how power adds.\n\n\
                \
                **Industry Standard:**\n\
                EVM is the standard quality metric for digital modulations. \
                Wi-Fi, LTE, and 5G all specify maximum EVM requirements.",
            concepts: &["Error Vector Magnitude", "RMS calculation", "Signal quality", "Industry metric"],
        },
    ],
};
