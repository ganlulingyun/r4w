//! Zigbee (IEEE 802.15.4) code snippets
//!
//! Zigbee uses O-QPSK modulation with half-sine pulse shaping and
//! direct sequence spreading for low-power wireless sensor networks.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete Zigbee waveform code documentation
pub static ZIGBEE_CODE: WaveformCode = WaveformCode {
    waveform_id: "Zigbee",
    display_name: "Zigbee (IEEE 802.15.4)",
    introduction: "Zigbee is a low-power, low-data-rate wireless standard for IoT and sensor networks. \
        It uses O-QPSK (Offset Quadrature PSK) with half-sine pulse shaping to achieve constant envelope, \
        combined with DSSS for robustness. The 2.4 GHz PHY achieves 250 kbps using a 32-chip spreading code \
        per 4-bit symbol. Used in smart home devices, industrial sensors, and mesh networks.",
    complexity: 4,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &OQPSK_CATEGORY,
        &SPREADING_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "Zigbee PHY Fundamentals",
    description: "IEEE 802.15.4 physical layer parameters",
    snippets: &[
        CodeSnippet {
            name: "ZigbeeConfig",
            brief: "Configure Zigbee PHY parameters",
            code: r#"pub struct ZigbeeConfig {
    /// Chip rate: 2 Mchip/s for 2.4 GHz
    pub chip_rate: f64,           // 2_000_000.0
    /// Chips per symbol: 32 (each 4-bit symbol → 32 chips)
    pub chips_per_symbol: usize,  // 32
    /// Symbol rate = chip_rate / chips_per_symbol
    pub symbol_rate: f64,         // 62_500 symbols/sec
    /// Bits per symbol (4 bits select one of 16 sequences)
    pub bits_per_symbol: usize,   // 4
    /// Data rate = symbol_rate * bits_per_symbol
    pub data_rate: f64,           // 250_000 bps
    /// Half-sine pulse shaping enabled
    pub half_sine: bool,
}

// 2.4 GHz PHY Summary:
// - 2 Mchip/s chip rate
// - 32 chips/symbol (8 dB processing gain)
// - 62.5 ksym/s symbol rate
// - 250 kbps data rate
// - 16 channels (2405-2480 MHz, 5 MHz spacing)"#,
            explanation: "**Zigbee 2.4 GHz PHY** uses DSSS with O-QPSK.\n\n\
                **Key Parameters:**\n\
                - Chip rate: 2 Mchip/s\n\
                - 32 chips per 4-bit symbol\n\
                - 250 kbps data rate\n\
                - 16 channels in 2.4 GHz band\n\n\
                **Why DSSS?**\n\
                - Processing gain (~15 dB) improves range\n\
                - Interference rejection\n\
                - Coexists with WiFi and Bluetooth\n\n\
                **Power Efficiency:**\n\
                - Low data rate = long battery life\n\
                - Typical: years on coin cell battery",
            concepts: &["IEEE 802.15.4", "DSSS", "O-QPSK", "Low power"],
        },
        CodeSnippet {
            name: "CHIP_SEQUENCES",
            brief: "The 16 spreading sequences (4 bits → 32 chips)",
            code: r#"// Each 4-bit symbol maps to a unique 32-chip sequence
// These are the IEEE 802.15.4 standard sequences
static CHIP_SEQUENCES: [[i8; 32]; 16] = [
    // Symbol 0 (0000)
    [1,1,0,1,1,0,0,1,1,1,0,0,0,0,1,1,0,1,0,1,0,0,1,0,0,0,1,0,1,1,1,0],
    // Symbol 1 (0001)
    [1,1,1,0,1,1,0,1,1,0,0,1,1,1,0,0,0,0,1,1,0,1,0,1,0,0,1,0,0,0,1,0],
    // ... (14 more sequences)
    // Symbol 15 (1111)
    [0,1,0,0,1,0,0,0,1,0,1,1,1,0,1,1,0,1,1,0,0,1,1,1,0,0,0,0,1,1,0,1],
];

// Properties:
// - Each sequence is unique
// - Low cross-correlation between sequences
// - Balanced (roughly equal 0s and 1s)"#,
            explanation: "**Chip Sequences** provide the spreading codes.\n\n\
                **Symbol-to-Sequence Mapping:**\n\
                - 4 bits → 16 possible symbols (0-15)\n\
                - Each symbol has a unique 32-chip sequence\n\
                - Sequences designed for low cross-correlation\n\n\
                **Processing Gain:**\n\
                - 32 chips per symbol\n\
                - PG = 10 × log₁₀(32) ≈ 15 dB\n\n\
                **Chip Values:**\n\
                - Stored as 0/1, transmitted as -1/+1\n\
                - Half-sine shaping smooths transitions",
            concepts: &["Spreading sequences", "Symbol mapping", "Cross-correlation"],
        },
    ],
};

static OQPSK_CATEGORY: CodeCategory = CodeCategory {
    name: "O-QPSK Modulation",
    description: "Offset Quadrature PSK with half-sine pulse shaping",
    snippets: &[
        CodeSnippet {
            name: "oqpsk_modulate()",
            brief: "O-QPSK modulation with timing offset",
            code: r#"fn oqpsk_modulate(&self, chips: &[i8]) -> Vec<IQSample> {
    let mut samples = Vec::new();
    let spc = self.samples_per_chip;

    // Split chips into I and Q streams (even/odd)
    for (i, &chip) in chips.iter().enumerate() {
        let chip_val = if chip == 0 { -1.0 } else { 1.0 };

        // Generate half-sine shaped samples for this chip
        for j in 0..spc {
            let t = j as f64 / spc as f64;
            let shape = (PI * t).sin();  // Half-sine shape

            // I and Q are offset by half a chip period
            if i % 2 == 0 {
                // Even chips go to I
                samples.push(IQSample::new(chip_val * shape, 0.0));
            } else {
                // Odd chips go to Q (offset)
                samples.push(IQSample::new(0.0, chip_val * shape));
            }
        }
    }
    samples
}"#,
            explanation: "**O-QPSK (Offset QPSK)** prevents 180° phase jumps.\n\n\
                **The Problem with QPSK:**\n\
                - I and Q can both change at once\n\
                - Creates 180° phase transitions\n\
                - Causes envelope to go through zero\n\
                - Bad for non-linear amplifiers\n\n\
                **O-QPSK Solution:**\n\
                - Offset Q by half a symbol (half chip in Zigbee)\n\
                - Maximum phase change is now 90°\n\
                - Combined with half-sine → constant envelope\n\n\
                **Half-Sine Pulse Shaping:**\n\
                - Smooth transitions between chips\n\
                - Reduces spectral splatter\n\
                - Constant envelope for PA efficiency",
            concepts: &["O-QPSK", "Half-sine shaping", "Constant envelope", "Phase transitions"],
        },
        CodeSnippet {
            name: "half_sine_shape()",
            brief: "Generate half-sine pulse shape",
            code: r#"fn half_sine_shape(&self, t: f64) -> f64 {
    // Half-sine: sin(πt) for t ∈ [0, 1]
    // Peak at t=0.5, zero at t=0 and t=1
    (PI * t).sin()
}

// Why half-sine?
// 1. Smooth transitions (zero at edges)
// 2. Constant envelope when I/Q combined
// 3. Good spectral containment
// 4. MSK-equivalent signaling

// The I and Q components:
// I(t) = data_I × sin(πt/T)
// Q(t) = data_Q × sin(π(t-T/2)/T)  // Offset by T/2
// Envelope: sqrt(I² + Q²) = constant!"#,
            explanation: "**Half-Sine Pulse Shaping** creates MSK-like signals.\n\n\
                **Why It Works:**\n\
                - I transitions at t=0, T, 2T, ...\n\
                - Q transitions at t=T/2, 3T/2, ...\n\
                - Never both transitioning at once\n\n\
                **Constant Envelope:**\n\
                - |signal|² = I² + Q² = constant\n\
                - Perfect for Class-C amplifiers\n\
                - Maximum power efficiency\n\n\
                **Spectrum:**\n\
                - Main lobe: 2/T wide\n\
                - Sidelobes fall off as 1/f⁴\n\
                - Good adjacent channel rejection",
            concepts: &["Pulse shaping", "MSK", "Spectral efficiency", "Amplifier efficiency"],
        },
    ],
};

static SPREADING_CATEGORY: CodeCategory = CodeCategory {
    name: "Direct Sequence Spreading",
    description: "Spreading 4-bit symbols to 32 chips",
    snippets: &[
        CodeSnippet {
            name: "spread_symbol()",
            brief: "Map 4-bit symbol to 32-chip spreading sequence",
            code: r#"fn spread_symbol(&self, symbol: u8) -> &[i8; 32] {
    // 4-bit symbol (0-15) selects one of 16 sequences
    &CHIP_SEQUENCES[symbol as usize]
}

fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let mut samples = Vec::new();

    // Process 4 bits at a time
    for byte in data {
        // High nibble first (MSB)
        let sym_hi = (byte >> 4) & 0x0F;
        let chips_hi = self.spread_symbol(sym_hi);
        samples.extend(self.oqpsk_modulate(chips_hi));

        // Low nibble
        let sym_lo = byte & 0x0F;
        let chips_lo = self.spread_symbol(sym_lo);
        samples.extend(self.oqpsk_modulate(chips_lo));
    }
    samples
}"#,
            explanation: "**Symbol Spreading** expands data for robustness.\n\n\
                **Process:**\n\
                1. Split each byte into two 4-bit symbols\n\
                2. Each symbol selects one of 16 sequences\n\
                3. 32 chips transmitted per 4-bit symbol\n\
                4. Chips modulated with O-QPSK\n\n\
                **Spreading Factor:**\n\
                - 32 chips / 4 bits = 8 chips/bit\n\
                - Bandwidth expansion: 8×\n\
                - Processing gain: ~9 dB\n\n\
                **Why Not More Spreading?**\n\
                - Battery life matters more than range for IoT\n\
                - 250 kbps is enough for sensor data\n\
                - Mesh networking extends range instead",
            concepts: &["Spreading factor", "Nibble processing", "Sequence selection"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Zigbee Demodulation",
    description: "Recovering data from O-QPSK spread signal",
    snippets: &[
        CodeSnippet {
            name: "correlate_sequences()",
            brief: "Correlate with all 16 sequences to find best match",
            code: r#"fn correlate_sequences(&self, chips: &[i8]) -> u8 {
    let mut best_symbol = 0;
    let mut best_corr = i32::MIN;

    for (symbol, sequence) in CHIP_SEQUENCES.iter().enumerate() {
        // Correlate received chips with this sequence
        let corr: i32 = chips.iter()
            .zip(sequence.iter())
            .map(|(&rx, &ref_chip)| {
                (rx as i32) * (ref_chip as i32 * 2 - 1)
            })
            .sum();

        if corr > best_corr {
            best_corr = corr;
            best_symbol = symbol as u8;
        }
    }
    best_symbol  // 4-bit symbol (0-15)
}

// Perfect correlation: 32 (all chips match)
// Orthogonal: 0 (half match, half don't)
// Anti-correlation: -32 (all chips opposite)"#,
            explanation: "**Sequence Correlation** determines the transmitted symbol.\n\n\
                **Process:**\n\
                1. Receive 32 chips (despread O-QPSK first)\n\
                2. Correlate with all 16 reference sequences\n\
                3. Pick sequence with highest correlation\n\
                4. That sequence's index is the 4-bit symbol\n\n\
                **Correlation Values:**\n\
                - Perfect match: +32\n\
                - Wrong sequence: near 0\n\
                - Inverted: -32\n\n\
                **Error Handling:**\n\
                - Noise corrupts some chips\n\
                - Correct sequence still has highest correlation\n\
                - Processing gain provides margin",
            concepts: &["Correlation", "Symbol detection", "Processing gain"],
        },
        CodeSnippet {
            name: "demodulate()",
            brief: "Complete Zigbee demodulation pipeline",
            code: r#"fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
    let mut bits = Vec::new();
    let samples_per_symbol = self.samples_per_chip * self.chips_per_symbol;

    for chunk in samples.chunks(samples_per_symbol) {
        // 1. O-QPSK demodulation to get chips
        let chips = self.oqpsk_demodulate(chunk);

        // 2. Correlate with all 16 sequences
        let symbol = self.correlate_sequences(&chips);

        // 3. Extract 4 bits from symbol
        bits.push((symbol >> 3) & 1);
        bits.push((symbol >> 2) & 1);
        bits.push((symbol >> 1) & 1);
        bits.push(symbol & 1);
    }

    DemodResult {
        bits,
        symbols: vec![],
        metadata: HashMap::new(),
    }
}"#,
            explanation: "**Zigbee Demodulation Pipeline:**\n\n\
                **Step 1: O-QPSK Demodulation**\n\
                - Separate I and Q with timing offset\n\
                - Half-sine matched filter\n\
                - Slice to get chip decisions\n\n\
                **Step 2: Sequence Correlation**\n\
                - Try all 16 sequences\n\
                - Best match wins\n\n\
                **Step 3: Bit Extraction**\n\
                - Symbol index → 4 bits (MSB first)\n\n\
                **Performance:**\n\
                - Range: ~10-100m depending on power\n\
                - Battery: years on coin cell\n\
                - Coexistence: designed for 2.4 GHz crowd",
            concepts: &["Demodulation pipeline", "Matched filter", "Bit extraction"],
        },
    ],
};
