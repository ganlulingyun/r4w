//! QAM (Quadrature Amplitude Modulation) code snippets
//!
//! QAM combines amplitude AND phase modulation for maximum spectral efficiency.
//! It's used in WiFi, LTE, cable modems, and digital television.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete QAM waveform code documentation
pub static QAM_CODE: WaveformCode = WaveformCode {
    waveform_id: "QAM",
    display_name: "Quadrature Amplitude Modulation (QAM)",
    introduction: "QAM is the culmination of digital modulation - it varies BOTH amplitude AND phase \
        to pack maximum data into each symbol. While PSK only uses phase (points on a circle), \
        QAM uses a grid of points, dramatically increasing throughput. 16-QAM sends 4 bits per symbol, \
        64-QAM sends 6, and 256-QAM sends 8. The trade-off: higher orders require cleaner signals (more SNR). \
        QAM powers WiFi, 4G/5G, cable modems, and most high-speed digital communications.",
    complexity: 4,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &CONSTELLATION_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "QAM Fundamentals",
    description: "Core concepts: order, bits per symbol, and SNR requirements",
    snippets: &[
        CodeSnippet {
            name: "bits_per_symbol()",
            brief: "Relationship between QAM order and data rate",
            code: r#"fn bits_per_symbol(&self) -> u8 {
    // QAM order M = 2^bits
    // 16-QAM: log2(16) = 4 bits/symbol
    // 64-QAM: log2(64) = 6 bits/symbol
    // 256-QAM: log2(256) = 8 bits/symbol
    (self.order as f64).log2() as u8
}"#,
            explanation: "**Bits per Symbol** determines QAM's data rate advantage.\n\n\
                \
                **The Relationship:**\n\
                bits_per_symbol = log2(order)\n\n\
                \
                **Common QAM Orders:**\n\
                | Order   | Bits | Grid   | Relative Throughput |\n\
                |---------|------|--------|---------------------|\n\
                | 4-QAM   | 2    | 2x2    | 1x (same as QPSK)   |\n\
                | 16-QAM  | 4    | 4x4    | 2x                  |\n\
                | 64-QAM  | 6    | 8x8    | 3x                  |\n\
                | 256-QAM | 8    | 16x16  | 4x                  |\n\n\
                \
                **Why Square Grids?**\n\
                QAM uses square constellations (4, 16, 64, 256 points) because \
                they're easy to implement: the I and Q dimensions are independent, \
                each carrying half the bits.",
            concepts: &["Bits per symbol", "QAM order", "Square constellation", "Throughput scaling"],
        },
        CodeSnippet {
            name: "required_snr_db()",
            brief: "Approximate SNR needed for reliable decoding",
            code: r#"pub fn required_snr_db(&self) -> f64 {
    match self.order {
        4 => 10.0,    // Same as QPSK
        16 => 17.0,   // Points are closer together
        64 => 23.0,   // Even closer
        256 => 30.0,  // Very tight spacing
        _ => 10.0 * (self.order as f64).log2(),
    }
}"#,
            explanation: "**SNR Requirements** - the fundamental QAM trade-off.\n\n\
                \
                **Why Higher Orders Need More SNR:**\n\
                More points in the same I/Q space = smaller distances between points.\n\
                Noise can push a received point into the wrong decision region.\n\n\
                \
                **Approximate Requirements:**\n\
                - 4-QAM/QPSK: ~10 dB\n\
                - 16-QAM: ~17 dB\n\
                - 64-QAM: ~23 dB\n\
                - 256-QAM: ~30 dB\n\n\
                \
                **The 6 dB Rule:**\n\
                Each doubling of order (quadrupling of points) needs ~6 dB more SNR.\n\
                This is because point spacing halves, requiring 4x power for same BER.\n\n\
                \
                **Adaptive Modulation:**\n\
                Real systems (WiFi, LTE) dynamically switch QAM orders based on \
                channel quality - high SNR uses 256-QAM, poor conditions drop to QPSK.",
            concepts: &["SNR requirements", "Adaptive modulation", "6 dB rule", "Decision regions"],
        },
    ],
};

static CONSTELLATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Constellation Design",
    description: "Building the QAM grid with power normalization and Gray coding",
    snippets: &[
        CodeSnippet {
            name: "compute_constellation()",
            brief: "Generate the square QAM grid with normalized power",
            code: r#"fn compute_constellation(&mut self) {
    let side = (self.order as f64).sqrt() as usize;
    let amp = self.common.amplitude;

    // Calculate points and sum power for normalization
    let mut sum_power = 0.0;
    let mut points = Vec::new();

    for i in 0..side {
        for q in 0..side {
            // Map to symmetric grid: -3, -1, +1, +3 for 16-QAM
            let i_val = 2.0 * i as f64 - (side - 1) as f64;
            let q_val = 2.0 * q as f64 - (side - 1) as f64;
            points.push((i_val, q_val));
            sum_power += i_val * i_val + q_val * q_val;
        }
    }

    // Normalize so average power = amplitude^2
    let norm = (sum_power / self.order as f64).sqrt();
    self.constellation = points.iter()
        .map(|&(i, q)| IQSample::new(amp * i / norm, amp * q / norm))
        .collect();
}"#,
            explanation: "**Constellation Generation** creates the QAM grid.\n\n\
                \
                **Grid Construction (16-QAM Example):**\n\
                side = sqrt(16) = 4\n\
                Raw coordinates: -3, -1, +1, +3 on each axis\n\
                ```\n\
                  Q\n\
                  |\n\
                  +3  ●  ●  ●  ●\n\
                  +1  ●  ●  ●  ●\n\
                  ----+--------- I\n\
                  -1  ●  ●  ●  ●\n\
                  -3  ●  ●  ●  ●\n\
                     -3 -1 +1 +3\n\
                ```\n\n\
                \
                **Power Normalization:**\n\
                Corner points (like -3,-3) have more power than center points.\n\
                We normalize so AVERAGE power equals the target amplitude².\n\
                This ensures fair comparison with PSK and consistent SNR metrics.\n\n\
                \
                **Why Symmetric?**\n\
                Centering around (0,0) minimizes DC offset and keeps \
                average power consistent regardless of transmitted data.",
            concepts: &["Square grid", "Power normalization", "Symmetric constellation", "DC balance"],
        },
        CodeSnippet {
            name: "compute_gray_map()",
            brief: "2D Gray coding for minimal bit errors",
            code: r#"fn compute_gray_map(&self, side: usize) -> Vec<usize> {
    // Standard 1D Gray sequences
    let gray_1d: Vec<usize> = match side {
        2 => vec![0, 1],
        4 => vec![0, 1, 3, 2],           // 00, 01, 11, 10
        8 => vec![0, 1, 3, 2, 6, 7, 5, 4],
        _ => (0..side).collect(),
    };

    // Combine I and Q Gray codes for 2D mapping
    let mut map = vec![0; self.order];
    for (idx, &gi) in gray_1d.iter().enumerate() {
        for (jdx, &gq) in gray_1d.iter().enumerate() {
            let symbol = idx * side + jdx;
            let gray_symbol = gi * side + gq;
            map[gray_symbol] = symbol;
        }
    }

    map
}"#,
            explanation: "**2D Gray Coding** extends the 1D concept to QAM grids.\n\n\
                \
                **The 1D Gray Sequence:**\n\
                For 4 levels: 0→1→3→2 (binary: 00→01→11→10)\n\
                Adjacent values differ by exactly 1 bit.\n\n\
                \
                **Extending to 2D:**\n\
                Apply Gray coding independently to I and Q axes.\n\
                For 16-QAM (4×4 grid):\n\
                - I axis uses Gray sequence for 2 bits\n\
                - Q axis uses Gray sequence for 2 bits\n\
                - Combined: 4 bits with adjacent neighbors differing by 1 bit\n\n\
                \
                **Why It Matters:**\n\
                Errors typically move to adjacent constellation points.\n\
                Gray coding ensures these errors cause only 1 bit error instead of multiple.\n\n\
                \
                **16-QAM Example:**\n\
                Point at (+1,+1) has bits 0111\n\
                All 4 neighbors differ by exactly 1 bit:\n\
                (+3,+1)=0101, (-1,+1)=0110, (+1,+3)=0011, (+1,-1)=1111",
            concepts: &["2D Gray code", "Bit error minimization", "Adjacent symbols", "QAM labeling"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Modulation",
    description: "Converting bits to QAM symbols and I/Q samples",
    snippets: &[
        CodeSnippet {
            name: "bits_to_symbol()",
            brief: "Map input bits to constellation index using Gray code",
            code: r#"fn bits_to_symbol(&self, bits: &[u8]) -> usize {
    // Pack bits into integer (MSB first)
    let mut value = 0usize;
    for &bit in bits {
        value = (value << 1) | (bit as usize & 1);
    }

    // Apply Gray mapping to get constellation index
    self.gray_map.get(value).copied().unwrap_or(0)
}"#,
            explanation: "**Bit-to-Symbol Mapping** with Gray encoding.\n\n\
                \
                **The Process:**\n\
                1. Pack bits into binary integer (MSB first)\n\
                2. Use Gray map to find constellation index\n\n\
                \
                **16-QAM Example (4 bits → 1 symbol):**\n\
                Input bits: [1, 0, 1, 1]\n\
                Packed value: 1011 binary = 11 decimal\n\
                Gray map[11] → constellation index (e.g., 7)\n\n\
                \
                **Why the Lookup Table?**\n\
                Pre-computing the Gray map avoids runtime bit manipulation.\n\
                The gray_map array directly converts bit patterns to indices.\n\n\
                \
                **Bit Order:**\n\
                MSB-first means the first bit controls the most significant \
                decision (left vs right half of constellation).",
            concepts: &["Bit packing", "Gray mapping", "Lookup table", "MSB-first"],
        },
        CodeSnippet {
            name: "modulate()",
            brief: "The complete QAM modulation pipeline",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let bps = self.bits_per_symbol() as usize;
    let mut samples = Vec::new();

    for chunk in data.chunks(bps) {
        // Pad if necessary
        let mut bits = chunk.to_vec();
        while bits.len() < bps {
            bits.push(0);
        }

        // Map bits to constellation point
        let symbol = self.bits_to_symbol(&bits);

        // Hold constellation point for symbol duration
        samples.extend(self.generate_symbol(symbol));
    }

    samples
}"#,
            explanation: "**QAM Modulation** - packing multiple bits per symbol.\n\n\
                \
                **The Pipeline:**\n\
                1. Group input bits (4 for 16-QAM, 6 for 64-QAM, etc.)\n\
                2. Pad last group if needed\n\
                3. Map each group to constellation point via Gray code\n\
                4. Hold that I/Q point for `sps` samples\n\n\
                \
                **Padding:**\n\
                If data length isn't divisible by bits_per_symbol, we pad with zeros.\n\
                This ensures complete symbols but may waste some capacity.\n\n\
                \
                **Symbol Holding:**\n\
                Like PSK, QAM holds a constant I/Q value for the symbol duration.\n\
                The constellation point IS the signal - no carrier multiplication needed \
                for baseband processing.\n\n\
                \
                **Throughput:**\n\
                For 64-QAM at 1000 symbols/sec:\n\
                6 bits/symbol × 1000 = 6000 bits/sec",
            concepts: &["Bit grouping", "Padding", "Symbol rate", "Baseband modulation"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Demodulation",
    description: "Recovering bits from received QAM samples",
    snippets: &[
        CodeSnippet {
            name: "demodulate() - Symbol Detection",
            brief: "Average samples and find nearest constellation point",
            code: r#"fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
    let sps = self.sps();
    let mut result = DemodResult::default();

    for chunk in samples.chunks(sps) {
        // Average samples in symbol period
        let avg: IQSample = chunk.iter()
            .fold(IQSample::new(0.0, 0.0), |acc, &s| acc + s)
            / chunk.len() as f64;

        // Find nearest constellation point
        let mut best_symbol = 0;
        let mut best_dist = f64::MAX;

        for (i, &point) in self.constellation.iter().enumerate() {
            let dist = (avg - point).norm_sqr();
            if dist < best_dist {
                best_dist = dist;
                best_symbol = i;
            }
        }

        result.symbols.push(best_symbol as u16);
        result.bits.extend(self.symbol_to_bits(best_symbol));
    }
    result
}"#,
            explanation: "**QAM Demodulation** - the decision process.\n\n\
                \
                **Step 1: Sample Averaging**\n\
                Average all samples in the symbol period to reduce noise.\n\
                This is a simple matched filter for constant-amplitude signals.\n\n\
                \
                **Step 2: Nearest Point Search**\n\
                Compare averaged sample to ALL constellation points.\n\
                Choose the one with minimum Euclidean distance.\n\n\
                \
                **Complexity:**\n\
                For 256-QAM, we compare against 256 points per symbol.\n\
                Optimization: Use slicing (separate I and Q decisions) for large QAM.\n\n\
                \
                **Decision Regions:**\n\
                Each constellation point 'owns' a square region around it.\n\
                For 16-QAM with normalized power, region boundaries are at \
                ±0.5 normalized units between adjacent points.\n\n\
                \
                **Error Probability:**\n\
                Corner points have 2 neighbors → lower error probability.\n\
                Edge points have 3 neighbors.\n\
                Inner points have 4 neighbors → highest error probability.",
            concepts: &[
                "Sample averaging",
                "Nearest neighbor",
                "Decision regions",
                "Corner vs inner points",
            ],
        },
        CodeSnippet {
            name: "symbol_to_bits()",
            brief: "Convert detected symbol back to bits via Gray decode",
            code: r#"fn symbol_to_bits(&self, symbol: usize) -> Vec<u8> {
    // Find the Gray code value for this constellation index
    let gray_value = self.gray_map
        .iter()
        .position(|&s| s == symbol)
        .unwrap_or(0);

    // Extract bits MSB-first
    let bps = self.bits_per_symbol() as usize;
    (0..bps)
        .rev()
        .map(|i| ((gray_value >> i) & 1) as u8)
        .collect()
}"#,
            explanation: "**Symbol-to-Bits Decoding** reverses the encoding.\n\n\
                \
                **The Process:**\n\
                1. Find which Gray code value maps to this constellation index\n\
                2. Extract individual bits from that value\n\n\
                \
                **Inverse Lookup:**\n\
                gray_map[gray_value] = symbol, so we search for the position.\n\
                This is O(n) but only done once per symbol.\n\n\
                \
                **Bit Extraction:**\n\
                For 16-QAM (4 bits):\n\
                gray_value = 13 (binary 1101)\n\
                Bits: [1, 1, 0, 1] (MSB first)\n\n\
                \
                **Alternative:**\n\
                Could pre-compute inverse map for O(1) lookup, but the \
                constellation is small enough that linear search is fine.",
            concepts: &["Gray decoding", "Inverse mapping", "Bit extraction"],
        },
        CodeSnippet {
            name: "EVM Calculation",
            brief: "Measure signal quality from error vectors",
            code: r#"// Calculate EVM (Error Vector Magnitude)
let mut evm_sum = 0.0;
let mut count = 0;

for (chunk, &symbol) in samples.chunks(sps).zip(result.symbols.iter()) {
    let reference = self.constellation[symbol as usize];
    let avg: IQSample = chunk.iter()
        .fold(IQSample::new(0.0, 0.0), |acc, &s| acc + s)
        / chunk.len() as f64;

    // Error = distance from ideal point
    let error = (avg - reference).norm_sqr();
    evm_sum += error;
    count += 1;
}

let rms_evm = (evm_sum / count as f64).sqrt();
// Estimate SNR from EVM
result.snr_estimate = Some(-20.0 * rms_evm.log10());"#,
            explanation: "**EVM for QAM** - the key quality metric.\n\n\
                \
                **What EVM Measures:**\n\
                How far received points deviate from ideal constellation positions.\n\
                It captures all impairments: noise, phase error, amplitude distortion.\n\n\
                \
                **The Calculation:**\n\
                1. For each symbol, compute error = |received - ideal|²\n\
                2. Average all squared errors\n\
                3. Take square root for RMS (Root Mean Square)\n\n\
                \
                **EVM to SNR:**\n\
                SNR_dB ≈ -20 × log10(EVM_rms)\n\
                If EVM = 0.1 (10%), SNR ≈ 20 dB\n\
                If EVM = 0.01 (1%), SNR ≈ 40 dB\n\n\
                \
                **QAM EVM Requirements:**\n\
                | QAM Order | Max EVM for 10⁻⁴ BER |\n\
                |-----------|----------------------|\n\
                | 16-QAM    | ~12%                 |\n\
                | 64-QAM    | ~8%                  |\n\
                | 256-QAM   | ~3.5%                |\n\n\
                \
                **Industry Standard:**\n\
                EVM is THE standard quality metric for QAM systems. \
                WiFi 6 (802.11ax) specifies EVM limits for each MCS rate.",
            concepts: &["Error Vector Magnitude", "RMS calculation", "SNR estimation", "WiFi MCS"],
        },
    ],
};
