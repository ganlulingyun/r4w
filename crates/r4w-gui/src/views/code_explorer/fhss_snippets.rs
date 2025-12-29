//! FHSS (Frequency Hopping Spread Spectrum) code snippets
//!
//! FHSS spreads the signal by rapidly hopping between frequencies according
//! to a pseudo-random sequence, providing interference avoidance and security.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete FHSS waveform code documentation
pub static FHSS_CODE: WaveformCode = WaveformCode {
    waveform_id: "FHSS",
    display_name: "Frequency Hopping Spread Spectrum (FHSS)",
    introduction: "FHSS rapidly switches the carrier frequency according to a pseudo-random \
        hopping pattern. This provides anti-jamming capability (jammers can't follow the hops), \
        reduced interference (different systems use different patterns), and security \
        (unauthorized receivers don't know the pattern). Used in Bluetooth, military radios (SINCGARS), \
        and some WiFi standards.",
    complexity: 3,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &HOP_PATTERN_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "FHSS Fundamentals",
    description: "Core concepts and configuration for frequency hopping",
    snippets: &[
        CodeSnippet {
            name: "FhssConfig",
            brief: "Configure hop channels, rate, and pattern",
            code: r#"pub struct FhssConfig {
    /// Number of frequency channels to hop across
    pub num_channels: usize,      // e.g., 79 for Bluetooth
    /// Spacing between channels in Hz
    pub channel_spacing: f64,     // e.g., 1 MHz
    /// Hops per second
    pub hop_rate: f64,            // e.g., 1600 for Bluetooth
    /// Symbols transmitted per hop (dwell time)
    pub symbols_per_hop: usize,   // e.g., 1-10
    /// Base/center frequency
    pub center_freq: f64,
    /// Modulation per hop (FSK, PSK, etc.)
    pub modulation: HopModulation,
    /// Hopping pattern type
    pub pattern: HopPattern,
}

// Processing gain from FHSS:
// PG = 10 * log10(num_channels)
// 79 channels → ~19 dB of jamming resistance"#,
            explanation: "**FHSS Configuration** defines the hopping behavior.\n\n\
                **Key Parameters:**\n\
                - **Num Channels:** More = harder to jam (79 for Bluetooth)\n\
                - **Channel Spacing:** Frequency gap between hops\n\
                - **Hop Rate:** How fast we switch frequencies\n\
                - **Dwell Time:** Time spent on each frequency\n\n\
                **Processing Gain:**\n\
                PG = 10 × log₁₀(num_channels)\n\
                - 79 channels = ~19 dB jamming resistance\n\
                - Jammer must spread power across all channels\n\n\
                **Bandwidth:**\n\
                Total BW = num_channels × channel_spacing\n\
                e.g., 79 × 1 MHz = 79 MHz for Bluetooth",
            concepts: &["Hop rate", "Dwell time", "Processing gain", "Channel spacing"],
        },
        CodeSnippet {
            name: "HopModulation",
            brief: "Modulation type used within each hop",
            code: r#"pub enum HopModulation {
    /// Binary FSK (most common in FHSS)
    Bfsk { deviation: f64 },
    /// Binary PSK
    Bpsk,
    /// Quadrature PSK (2 bits/symbol)
    Qpsk,
}

impl HopModulation {
    pub fn modulate_symbol(&self, bits: &[u8], freq: f64) -> Vec<IQSample> {
        match self {
            Self::Bfsk { deviation } => {
                // FSK: shift frequency based on bit
                let f = if bits[0] == 0 {
                    freq - deviation
                } else {
                    freq + deviation
                };
                generate_tone(f, samples_per_symbol)
            }
            Self::Bpsk => { /* phase modulation */ }
            Self::Qpsk => { /* quadrature modulation */ }
        }
    }
}"#,
            explanation: "**Hop Modulation** is the signaling within each hop.\n\n\
                **Why FSK is Popular:**\n\
                - Constant envelope (good for non-linear amplifiers)\n\
                - Simple to implement\n\
                - Robust to amplitude variations from hopping\n\n\
                **Data Rate Calculation:**\n\
                Data rate = hop_rate × symbols_per_hop × bits_per_symbol\n\n\
                **Example (Bluetooth):**\n\
                - 1600 hops/sec × 1 symbol/hop × 1 bit/symbol\n\
                - = 1600 bps (basic rate)\n\
                - Enhanced Data Rate uses PSK for higher rates",
            concepts: &["FSK modulation", "Constant envelope", "Data rate"],
        },
    ],
};

static HOP_PATTERN_CATEGORY: CodeCategory = CodeCategory {
    name: "Hop Pattern Generation",
    description: "Creating pseudo-random frequency sequences",
    snippets: &[
        CodeSnippet {
            name: "HopPattern",
            brief: "Different hop pattern generation methods",
            code: r#"pub enum HopPattern {
    /// Pseudo-random using LFSR
    PseudoRandom { seed: u32 },
    /// Sequential (for testing)
    Sequential,
    /// Adaptive (avoid bad channels)
    Adaptive { blacklist: Vec<usize> },
}

impl HopPattern {
    pub fn generate_sequence(&self, length: usize, num_channels: usize) -> Vec<usize> {
        match self {
            Self::PseudoRandom { seed } => {
                let mut lfsr = *seed;
                (0..length)
                    .map(|_| {
                        lfsr = advance_lfsr(lfsr);
                        (lfsr as usize) % num_channels
                    })
                    .collect()
            }
            Self::Sequential => {
                (0..length).map(|i| i % num_channels).collect()
            }
            Self::Adaptive { blacklist } => {
                // Skip known-bad channels
                generate_avoiding(blacklist, num_channels)
            }
        }
    }
}"#,
            explanation: "**Hop Patterns** determine which frequency is used when.\n\n\
                **Pattern Types:**\n\
                - **Pseudo-Random:** LFSR-based, looks random but repeatable\n\
                - **Sequential:** Simple but predictable (testing only)\n\
                - **Adaptive:** Avoid interfered/jammed channels\n\n\
                **Security Note:**\n\
                - TX and RX must know the same pattern + timing\n\
                - Pattern derived from shared secret (key)\n\
                - Eavesdropper sees random noise without key\n\n\
                **Bluetooth Example:**\n\
                - 79 channels in 2.4 GHz band\n\
                - Pattern derived from master's address\n\
                - Both devices hop in sync",
            concepts: &["LFSR", "Pseudo-random", "Adaptive hopping", "Synchronization"],
        },
        CodeSnippet {
            name: "get_hop_frequency()",
            brief: "Calculate the frequency for a given hop index",
            code: r#"fn get_hop_frequency(&self, hop_index: usize) -> f64 {
    // Get channel from hop pattern
    let channel = self.hop_pattern[hop_index % self.hop_pattern.len()];

    // Calculate actual frequency
    // Channels centered around center_freq
    let half_bw = (self.num_channels as f64 - 1.0) / 2.0 * self.channel_spacing;
    let base_freq = self.center_freq - half_bw;

    base_freq + channel as f64 * self.channel_spacing
}

// Example with 79 channels, 1 MHz spacing, center at 2.441 GHz:
// Channel 0:  2.402 GHz
// Channel 39: 2.441 GHz (center)
// Channel 78: 2.480 GHz"#,
            explanation: "**Frequency Calculation** maps channel index to RF frequency.\n\n\
                **Centering the Band:**\n\
                - Channels distributed symmetrically around center\n\
                - Channel spacing determines resolution\n\n\
                **Bluetooth Example:**\n\
                - 79 channels from 2.402 to 2.480 GHz\n\
                - 1 MHz spacing\n\
                - Avoids interference with other 2.4 GHz devices\n\n\
                **SINCGARS (Military):**\n\
                - 2320 channels in 30-88 MHz VHF band\n\
                - 25 kHz spacing\n\
                - Much larger hop set for security",
            concepts: &["Channel mapping", "Frequency band", "Channel spacing"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "FHSS Modulation",
    description: "Generating hopped signals",
    snippets: &[
        CodeSnippet {
            name: "modulate()",
            brief: "Complete FHSS modulation pipeline",
            code: r#"fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
    let mut samples = Vec::new();
    let mut bit_idx = 0;
    let mut hop_idx = 0;

    while bit_idx < data.len() * 8 {
        // 1. Get frequency for this hop
        let freq = self.get_hop_frequency(hop_idx);

        // 2. Modulate symbols_per_hop worth of data
        for _ in 0..self.symbols_per_hop {
            if bit_idx >= data.len() * 8 { break; }

            let bit = (data[bit_idx / 8] >> (7 - bit_idx % 8)) & 1;
            let symbol_samples = self.modulation.modulate_symbol(&[bit], freq);
            samples.extend(symbol_samples);
            bit_idx += 1;
        }

        // 3. Move to next hop
        hop_idx += 1;
    }
    samples
}"#,
            explanation: "**FHSS Modulation** combines hopping with per-hop modulation.\n\n\
                **For Each Hop:**\n\
                1. Look up frequency from hop pattern\n\
                2. Transmit symbols_per_hop symbols at that frequency\n\
                3. Advance to next hop\n\n\
                **Dwell Time:**\n\
                - Time spent on each frequency\n\
                - = symbols_per_hop × symbol_duration\n\
                - Must be long enough for receiver to lock\n\n\
                **Hop Timing:**\n\
                - TX and RX must hop at exact same times\n\
                - Typically uses GPS time or shared clock",
            concepts: &["Hop timing", "Dwell time", "Symbol transmission"],
        },
        CodeSnippet {
            name: "anti_jam_demo()",
            brief: "Demonstrate jamming resistance",
            code: r#"// Jammer tries to block communication
// but can only jam a few channels at once

fn simulate_with_jammer(signal: &[IQSample], jammed_channels: &[usize]) {
    let mut errors = 0;
    let mut total = 0;

    for (hop_idx, chunk) in signal.chunks(samples_per_hop).enumerate() {
        let channel = hop_pattern[hop_idx];
        total += 1;

        if jammed_channels.contains(&channel) {
            // This hop is jammed - might have errors
            errors += 1;
        }
        // Most hops are clean!
    }

    // With 79 channels and 5 jammed:
    // Only 5/79 ≈ 6% of hops affected
    // Error correction can fix this!
    println!("Jammed hops: {}/{} ({:.1}%)", errors, total, 100.0 * errors / total);
}"#,
            explanation: "**Anti-Jamming** is FHSS's superpower.\n\n\
                **Why Jamming is Hard:**\n\
                - Jammer doesn't know the hop pattern\n\
                - Can't jam all channels simultaneously (too much power)\n\
                - Partial-band jamming only affects few hops\n\n\
                **The Math:**\n\
                - 79 channels, jammer covers 5\n\
                - Only 6% of hops affected\n\
                - Forward Error Correction handles errors\n\n\
                **Follower Jamming:**\n\
                - Jammer tries to track hops\n\
                - But hop is complete before jammer can react\n\
                - Requires very fast hardware + hop prediction",
            concepts: &["Anti-jamming", "Partial-band jamming", "FEC", "LPI"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "FHSS Demodulation",
    description: "Synchronizing and receiving hopped signals",
    snippets: &[
        CodeSnippet {
            name: "synchronize()",
            brief: "Acquire hop timing and pattern synchronization",
            code: r#"fn synchronize(&mut self, samples: &[IQSample]) -> Option<usize> {
    // Scan for preamble on each possible channel
    for channel in 0..self.num_channels {
        let freq = self.channel_to_freq(channel);
        let power = self.measure_power_at_freq(samples, freq);

        if power > self.threshold {
            // Found signal, now find hop timing
            if let Some(sync_point) = self.detect_preamble(samples, channel) {
                self.current_hop = self.pattern_from_sync(sync_point);
                return Some(sync_point);
            }
        }
    }
    None  // No signal found
}"#,
            explanation: "**FHSS Synchronization** is the hardest part!\n\n\
                **The Challenge:**\n\
                - Must find which frequency and when\n\
                - Signal only on one channel at any instant\n\
                - Pattern is pseudo-random (unknown to start)\n\n\
                **Acquisition Process:**\n\
                1. Scan all channels for energy\n\
                2. Detect known preamble pattern\n\
                3. Derive hop timing from preamble\n\
                4. Start hopping in sync with TX\n\n\
                **Bluetooth Solution:**\n\
                - Master broadcasts on known schedule\n\
                - Slave listens, finds master's pattern\n\
                - Both derive pattern from master's address",
            concepts: &["Synchronization", "Acquisition", "Preamble detection"],
        },
        CodeSnippet {
            name: "demodulate()",
            brief: "Demodulate received FHSS signal",
            code: r#"fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
    let mut bits = Vec::new();
    let samples_per_hop = self.samples_per_symbol * self.symbols_per_hop;

    for (hop_idx, chunk) in samples.chunks(samples_per_hop).enumerate() {
        // 1. Get expected frequency for this hop
        let freq = self.get_hop_frequency(hop_idx);

        // 2. Mix down to baseband
        let baseband = self.mix_to_baseband(chunk, freq);

        // 3. Demodulate symbols at baseband
        for symbol_chunk in baseband.chunks(self.samples_per_symbol) {
            let bit = self.modulation.demodulate_symbol(symbol_chunk);
            bits.push(bit);
        }
    }

    DemodResult { bits, symbols: vec![], metadata: HashMap::new() }
}"#,
            explanation: "**FHSS Demodulation** follows the hops to recover data.\n\n\
                **For Each Hop:**\n\
                1. Calculate expected frequency from pattern\n\
                2. Mix signal down to baseband\n\
                3. Demodulate using per-hop scheme (FSK, PSK)\n\n\
                **Timing is Critical:**\n\
                - Must hop at exactly the same time as transmitter\n\
                - Clock drift causes \"hop slip\" - losing sync\n\
                - Guard time between hops helps\n\n\
                **Mix to Baseband:**\n\
                - Multiply by e^(-j2πft) at hop frequency\n\
                - Shifts signal to DC for demodulation",
            concepts: &["Hop synchronization", "Baseband mixing", "Symbol recovery"],
        },
    ],
};
