//! DSSS (Direct Sequence Spread Spectrum) code snippets
//!
//! DSSS spreads the signal by multiplying each data symbol by a high-rate
//! pseudo-noise (PN) sequence, providing processing gain and interference rejection.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete DSSS waveform code documentation
pub static DSSS_CODE: WaveformCode = WaveformCode {
    waveform_id: "DSSS",
    display_name: "Direct Sequence Spread Spectrum (DSSS)",
    introduction: "DSSS is a spread spectrum technique that spreads the signal by multiplying \
        each data symbol by a high-rate pseudo-noise (PN) sequence. This provides processing gain \
        (can operate below noise floor), interference rejection, multi-user access (CDMA), and \
        Low Probability of Detection/Intercept (LPD/LPI). Used in GPS, CDMA cellular, WiFi, and military comms.",
    complexity: 4,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &PN_SEQUENCE_CATEGORY,
        &SPREADING_CATEGORY,
        &DESPREADING_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "DSSS Fundamentals",
    description: "Core concepts and configuration for spread spectrum",
    snippets: &[
        CodeSnippet {
            name: "DsssModulation",
            brief: "Underlying modulation type (BPSK or QPSK)",
            code: r#"pub enum DsssModulation {
    /// Binary Phase Shift Keying (1 bit/symbol)
    Bpsk,
    /// Quadrature Phase Shift Keying (2 bits/symbol)
    Qpsk,
}

impl DsssModulation {
    pub fn modulate(&self, bits: &[u8]) -> IQSample {
        match self {
            Self::Bpsk => {
                let bit = bits[0] & 1;
                if bit == 0 { IQSample::new(1.0, 0.0) }
                else { IQSample::new(-1.0, 0.0) }
            }
            Self::Qpsk => {
                let scale = 1.0 / 2.0_f64.sqrt();
                let i = if bits[0] == 0 { scale } else { -scale };
                let q = if bits[1] == 0 { scale } else { -scale };
                IQSample::new(i, q)
            }
        }
    }
}"#,
            explanation: "**DSSS Modulation Types**\n\n\
                DSSS can use different underlying modulations:\n\n\
                **BPSK (Binary PSK):**\n\
                - 1 bit per symbol\n\
                - Constellation: ±1 on real axis\n\
                - Most robust to noise\n\n\
                **QPSK (Quadrature PSK):**\n\
                - 2 bits per symbol\n\
                - 4 constellation points\n\
                - Double data rate vs BPSK\n\n\
                The modulation happens BEFORE spreading, so each modulated symbol \
                gets multiplied by the entire PN sequence.",
            concepts: &["BPSK", "QPSK", "Symbol modulation", "Constellation"],
        },
        CodeSnippet {
            name: "processing_gain()",
            brief: "Calculate the processing gain in dB",
            code: r#"pub fn processing_gain_db(&self) -> f64 {
    // Processing Gain = 10 * log10(chips_per_symbol)
    // | Chips | PG (dB) |
    // |-------|---------|
    // | 31    | 15 dB   |
    // | 127   | 21 dB   |
    // | 1023  | 30 dB   |
    10.0 * (self.chips_per_symbol() as f64).log10()
}"#,
            explanation: "**Processing Gain (PG)** is the key DSSS advantage.\n\n\
                **Formula:** PG = 10 × log₁₀(chips_per_symbol)\n\n\
                **What It Means:**\n\
                - Signal can be transmitted PG dB below the noise floor\n\
                - Narrowband interference is suppressed by PG dB\n\
                - More chips = more gain but lower data rate\n\n\
                **Example:** With 1023 chips/symbol:\n\
                - Processing Gain = 30 dB\n\
                - Signal can be 1000× weaker than noise and still be decoded!\n\
                - This is why GPS signals work even though they're very weak",
            concepts: &["Processing gain", "Spread spectrum", "Noise floor", "LPD/LPI"],
        },
    ],
};

static PN_SEQUENCE_CATEGORY: CodeCategory = CodeCategory {
    name: "PN Sequence Generation",
    description: "Pseudo-noise sequences for spreading",
    snippets: &[
        CodeSnippet {
            name: "GoldCodeGenerator",
            brief: "Generate Gold codes using two LFSRs",
            code: r#"pub struct GoldCodeGenerator {
    lfsr1: u32,  // First m-sequence LFSR
    lfsr2: u32,  // Second m-sequence LFSR
    mask1: u32,  // Feedback taps for LFSR1
    mask2: u32,  // Feedback taps for LFSR2
    degree: u8,  // LFSR degree (5-10)
}

impl GoldCodeGenerator {
    pub fn next_chip(&mut self) -> i8 {
        // XOR the outputs of both LFSRs
        let out1 = (self.lfsr1 & 1) as i8;
        let out2 = (self.lfsr2 & 1) as i8;

        // Advance both LFSRs
        self.lfsr1 = self.advance_lfsr(self.lfsr1, self.mask1);
        self.lfsr2 = self.advance_lfsr(self.lfsr2, self.mask2);

        // Gold code: XOR of two m-sequences, map to ±1
        if (out1 ^ out2) == 0 { 1 } else { -1 }
    }
}"#,
            explanation: "**Gold Codes** are ideal for multi-user systems (CDMA).\n\n\
                **How They Work:**\n\
                - Two maximal-length sequences (m-sequences) are XORed together\n\
                - Each LFSR has length n bits, producing 2ⁿ-1 chips\n\
                - Different initial states create different codes\n\n\
                **Properties:**\n\
                - Cross-correlation between codes is bounded (3-valued)\n\
                - 2ⁿ+1 codes available per degree\n\
                - GPS uses Gold codes for satellite identification\n\n\
                **Why Not Just m-sequences?**\n\
                - m-sequences have perfect autocorrelation\n\
                - But cross-correlation between different m-sequences is poor\n\
                - Gold codes trade some autocorrelation for better cross-correlation",
            concepts: &["Gold codes", "LFSR", "m-sequence", "Cross-correlation", "CDMA"],
        },
        CodeSnippet {
            name: "generate_pn_sequence()",
            brief: "Generate complete PN sequence for spreading",
            code: r#"fn generate_pn_sequence(&self) -> Vec<i8> {
    let length = self.chips_per_symbol();
    let mut chips = Vec::with_capacity(length);
    let mut gen = GoldCodeGenerator::new(self.config.pn_degree);

    for _ in 0..length {
        chips.push(gen.next_chip());  // +1 or -1
    }
    chips
}

// Typical sequence lengths based on LFSR degree:
// Degree 5:  31 chips   (15 dB processing gain)
// Degree 6:  63 chips   (18 dB)
// Degree 7:  127 chips  (21 dB)
// Degree 10: 1023 chips (30 dB, used in GPS)"#,
            explanation: "**PN Sequence Generation** creates the spreading code.\n\n\
                **Sequence Length:** 2^n - 1 chips (for LFSR degree n)\n\n\
                **Properties of Good PN Sequences:**\n\
                - Balance: Equal number of +1 and -1 (off by 1)\n\
                - Run length: Maximum run of same chip is n\n\
                - Autocorrelation: High peak at zero lag, low elsewhere\n\n\
                **The Chips:**\n\
                - Each chip is +1 or -1\n\
                - Chip rate >> bit rate (spreading)\n\
                - Receiver must know the exact same sequence to despread",
            concepts: &["PN sequence", "Chip rate", "LFSR degree", "Autocorrelation"],
        },
    ],
};

static SPREADING_CATEGORY: CodeCategory = CodeCategory {
    name: "Signal Spreading",
    description: "Spreading data symbols with PN sequence",
    snippets: &[
        CodeSnippet {
            name: "spread_symbol()",
            brief: "Spread a single data symbol with the PN sequence",
            code: r#"fn spread_symbol(&self, symbol: IQSample) -> Vec<IQSample> {
    let pn = &self.pn_sequence;  // Pre-generated PN chips
    let spc = self.samples_per_chip;

    let mut samples = Vec::with_capacity(pn.len() * spc);

    for &chip in pn {
        // Multiply symbol by chip (+1 or -1)
        let spread = IQSample::new(
            symbol.re * chip as f64,
            symbol.im * chip as f64,
        );

        // Oversample: repeat for samples_per_chip
        for _ in 0..spc {
            samples.push(spread);
        }
    }
    samples
}"#,
            explanation: "**Spreading** multiplies the data symbol by the PN sequence.\n\n\
                **Process:**\n\
                1. Take modulated symbol (e.g., BPSK: +1 or -1)\n\
                2. Multiply by each chip of PN sequence\n\
                3. Symbol energy is now spread across all chips\n\n\
                **Example with 31-chip sequence:**\n\
                - Data: +1 (bit 0)\n\
                - PN: [+1 -1 +1 +1 -1 ...] (31 chips)\n\
                - Spread: [+1 -1 +1 +1 -1 ...]\n\n\
                **Bandwidth Expansion:**\n\
                - Original bandwidth ≈ symbol rate\n\
                - Spread bandwidth ≈ chip rate = symbol_rate × chips_per_symbol\n\
                - Spread signal looks like wideband noise!",
            concepts: &["Spreading", "Chip multiplication", "Bandwidth expansion", "Processing gain"],
        },
        CodeSnippet {
            name: "modulate()",
            brief: "Complete DSSS modulation: bits → spread signal",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let bits = self.bytes_to_bits(data);
    let mut samples = Vec::new();

    // Process bits in groups for underlying modulation
    let bits_per_sym = self.modulation.bits_per_symbol();

    for chunk in bits.chunks(bits_per_sym) {
        // 1. Modulate bits to constellation point
        let symbol = self.modulation.modulate(chunk);

        // 2. Spread with PN sequence
        let spread = self.spread_symbol(symbol);

        samples.extend(spread);
    }
    samples
}"#,
            explanation: "**DSSS Modulation Pipeline:**\n\n\
                **Step 1: Bit-to-Symbol**\n\
                - Group bits (1 for BPSK, 2 for QPSK)\n\
                - Map to constellation point\n\n\
                **Step 2: Spreading**\n\
                - Multiply symbol by entire PN sequence\n\
                - Each symbol becomes many chips\n\n\
                **Result:**\n\
                - Signal occupies much more bandwidth\n\
                - Power spectral density is very low\n\
                - Signal appears as noise to unauthorized receivers\n\n\
                **Data Rate:**\n\
                Data rate = Chip rate / chips_per_symbol\n\
                e.g., 1 Mchip/s with 1023 chips = 977 bps",
            concepts: &["Modulation pipeline", "Data rate", "Chip rate", "PSD"],
        },
    ],
};

static DESPREADING_CATEGORY: CodeCategory = CodeCategory {
    name: "Signal Despreading",
    description: "Recovering data by correlating with PN sequence",
    snippets: &[
        CodeSnippet {
            name: "despread_symbol()",
            brief: "Correlate received signal with PN sequence",
            code: r#"fn despread_symbol(&self, chips: &[IQSample]) -> IQSample {
    let pn = &self.pn_sequence;
    let spc = self.samples_per_chip;

    // Accumulate correlation
    let mut sum_i = 0.0;
    let mut sum_q = 0.0;

    for (chip_idx, &chip) in pn.iter().enumerate() {
        // Average samples for this chip
        let start = chip_idx * spc;
        let end = start + spc;

        for sample in &chips[start..end] {
            // Multiply by PN chip (despread)
            sum_i += sample.re * chip as f64;
            sum_q += sample.im * chip as f64;
        }
    }

    // Normalize
    let n = (pn.len() * spc) as f64;
    IQSample::new(sum_i / n, sum_q / n)
}"#,
            explanation: "**Despreading** recovers the original symbol via correlation.\n\n\
                **How It Works:**\n\
                - Multiply received signal by the SAME PN sequence\n\
                - All chips coherently combine (they all had same sign)\n\
                - Interference doesn't align → averages to near zero\n\n\
                **Correlation Peak:**\n\
                - Correct PN alignment: correlation = chips_per_symbol\n\
                - Wrong alignment: correlation ≈ 0\n\
                - This is why receivers need synchronization!\n\n\
                **The Magic:**\n\
                - Signal was spread over N chips\n\
                - Despreading concentrates it back into one symbol\n\
                - Processing gain emerges from this concentration",
            concepts: &["Despreading", "Correlation", "PN synchronization", "Processing gain"],
        },
        CodeSnippet {
            name: "demodulate()",
            brief: "Complete DSSS demodulation: samples → bits",
            code: r#"fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
    let mut bits = Vec::new();
    let chips_per_sym = self.chips_per_symbol();
    let spc = self.samples_per_chip;
    let samples_per_symbol = chips_per_sym * spc;

    for chunk in samples.chunks(samples_per_symbol) {
        // 1. Despread: correlate with PN sequence
        let symbol = self.despread_symbol(chunk);

        // 2. Demodulate: symbol to bits
        let sym_bits = self.modulation.demodulate(symbol);
        bits.extend(sym_bits);
    }

    DemodResult {
        bits,
        symbols: vec![],
        metadata: HashMap::new(),
    }
}"#,
            explanation: "**DSSS Demodulation Pipeline:**\n\n\
                **Step 1: Despreading**\n\
                - Correlate with local PN replica\n\
                - Processing gain pulls signal out of noise\n\
                - Requires precise timing synchronization\n\n\
                **Step 2: Symbol Detection**\n\
                - Despread output is a clean constellation point\n\
                - Standard BPSK/QPSK demodulation\n\n\
                **Synchronization Challenge:**\n\
                - Must align PN sequence to within 1 chip\n\
                - Search over all possible alignments (acquisition)\n\
                - Track timing drift (tracking)\n\n\
                GPS receivers do this for each satellite!",
            concepts: &["Demodulation", "Timing sync", "Acquisition", "Tracking"],
        },
    ],
};
