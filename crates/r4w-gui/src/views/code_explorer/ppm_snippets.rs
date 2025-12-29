//! PPM (Pulse Position Modulation) and ADS-B code snippets
//!
//! PPM encodes data in the position of pulses. ADS-B uses a Manchester-like
//! PPM scheme for aviation surveillance.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete PPM/ADS-B waveform code documentation
pub static PPM_CODE: WaveformCode = WaveformCode {
    waveform_id: "PPM",
    display_name: "Pulse Position Modulation / ADS-B",
    introduction: "PPM encodes information in the position of pulses within a symbol period. \
        ADS-B (Automatic Dependent Surveillance-Broadcast) uses a Manchester-like PPM scheme \
        where each bit is represented by two chips: '1' = high-then-low, '0' = low-then-high. \
        This enables aircraft to broadcast position and identification at 1090 MHz.",
    complexity: 3,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &ADSB_MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "PPM Fundamentals",
    description: "Basic concepts of pulse position modulation",
    snippets: &[
        CodeSnippet {
            name: "PpmVariant enum",
            brief: "Define PPM modulation variants",
            code: r#"pub enum PpmVariant {
    /// Standard PPM - pulse position within slot
    Standard,
    /// ADS-B Mode S - Manchester-like (two chips per bit)
    AdsB,
}"#,
            explanation: "**PPM has different variants:**\n\n\
                **Standard PPM:**\n\
                - Pulse position within slot encodes data\n\
                - Earlier position = '0', later = '1'\n\
                - Used in IR remotes, optical links\n\n\
                **ADS-B PPM:**\n\
                - Two chips per bit (Manchester encoding)\n\
                - Bit 1: high chip, then low chip\n\
                - Bit 0: low chip, then high chip\n\
                - 1 Mbps at 1090 MHz",
            concepts: &["Manchester encoding", "Chip rate", "Pulse position"],
        },
        CodeSnippet {
            name: "ADS-B parameters",
            brief: "Create ADS-B PPM modulator at 1 Mbps",
            code: r#"pub fn adsb(sample_rate: f64) -> Self {
    Self::new(sample_rate, 1_000_000.0, PpmVariant::AdsB)
}

// At 2 MHz sample rate:
// - 2 samples per bit (1 Mbps)
// - 1 sample per chip (0.5 us)
// - 8-bit preamble + 112-bit message"#,
            explanation: "**ADS-B Signal Parameters:**\n\n\
                **Frequency:** 1090 MHz\n\
                **Data Rate:** 1 Mbps\n\
                **Chip Duration:** 0.5 microseconds\n\
                **Bit Duration:** 1.0 microsecond\n\n\
                **Message Structure:**\n\
                - 8 us preamble (fixed pattern)\n\
                - 56 or 112 bits of data\n\
                - Total: 64 or 120 microseconds\n\n\
                **Extended Squitter (DF17):**\n\
                - 112 bits total\n\
                - Contains aircraft ID, position, velocity",
            concepts: &["ADS-B", "Mode S", "Extended squitter", "1090 MHz"],
        },
    ],
};

static ADSB_MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "ADS-B Modulation",
    description: "Generating ADS-B PPM signals",
    snippets: &[
        CodeSnippet {
            name: "generate_adsb_preamble()",
            brief: "Generate the 8-microsecond ADS-B preamble",
            code: r#"fn generate_adsb_preamble(&self) -> Vec<IQSample> {
    let sps = self.sps();
    let half_sps = sps / 2;
    let amp = self.common.amplitude;

    // 8us preamble with pulses at specific positions
    let mut samples = vec![IQSample::new(0.0, 0.0); sps * 8];

    // Pulse at 0-0.5us
    for i in 0..half_sps {
        samples[i] = IQSample::new(amp, 0.0);
    }
    // Pulse at 1-1.5us
    for i in sps..sps + half_sps {
        samples[i] = IQSample::new(amp, 0.0);
    }
    // Pulse at 3.5-4us and 4.5-5us...
    samples
}"#,
            explanation: "**ADS-B Preamble:**\n\n\
                The preamble is 8 microseconds long with 4 pulses:\n\
                - Pulses at 0us, 1us, 3.5us, and 4.5us\n\
                - Each pulse is 0.5us wide\n\n\
                **Purpose:**\n\
                - Synchronization: receiver locks to signal timing\n\
                - Detection: unique pattern identifies ADS-B\n\
                - AGC settling: receiver adjusts gain\n\n\
                **Binary Pattern:** 1010000101000000",
            concepts: &["Preamble", "Synchronization", "Mode S"],
        },
        CodeSnippet {
            name: "generate_adsb_bit()",
            brief: "Generate Manchester-encoded bit for ADS-B",
            code: r#"fn generate_adsb_bit(&self, bit: u8) -> Vec<IQSample> {
    let sps = self.sps();
    let half_sps = sps / 2;
    let amp = self.common.amplitude;
    let mut samples = vec![IQSample::new(0.0, 0.0); sps];

    if bit == 1 {
        // High then low (Manchester '1')
        for i in 0..half_sps {
            samples[i] = IQSample::new(amp, 0.0);
        }
    } else {
        // Low then high (Manchester '0')
        for i in half_sps..sps {
            samples[i] = IQSample::new(amp, 0.0);
        }
    }
    samples
}"#,
            explanation: "**Manchester Encoding for ADS-B:**\n\n\
                Each bit is 1 microsecond, divided into 2 chips:\n\n\
                **Bit '1':**\n\
                - First chip (0-0.5us): HIGH (pulse)\n\
                - Second chip (0.5-1us): LOW (no pulse)\n\n\
                **Bit '0':**\n\
                - First chip (0-0.5us): LOW (no pulse)\n\
                - Second chip (0.5-1us): HIGH (pulse)\n\n\
                **Benefits:**\n\
                - DC balanced (equal highs and lows)\n\
                - Self-clocking (transitions in every bit)\n\
                - Error detection (invalid if no transition)",
            concepts: &["Manchester encoding", "Chip", "DC balance"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "ADS-B Demodulation",
    description: "Decoding ADS-B PPM signals",
    snippets: &[
        CodeSnippet {
            name: "demod_adsb()",
            brief: "Demodulate ADS-B signal to recover bits",
            code: r#"fn demod_adsb(&self, samples: &[IQSample]) -> Vec<u8> {
    let sps = self.sps();
    let half_sps = sps / 2;
    let mut bits = Vec::new();

    // Skip 8-bit preamble
    let data_start = sps * 8;

    for bit_idx in 0..(samples.len() - data_start) / sps {
        let start = data_start + bit_idx * sps;

        // Measure energy in first half vs second half
        let first_half_energy: f64 = samples[start..start + half_sps]
            .iter()
            .map(|s| s.re * s.re + s.im * s.im)
            .sum();
        let second_half_energy: f64 = samples[start + half_sps..start + sps]
            .iter()
            .map(|s| s.re * s.re + s.im * s.im)
            .sum();

        // Bit 1 has energy in first half
        bits.push(if first_half_energy > second_half_energy { 1 } else { 0 });
    }
    bits
}"#,
            explanation: "**ADS-B Demodulation Algorithm:**\n\n\
                **Step 1: Skip Preamble**\n\
                The 8-microsecond preamble is used for sync but not decoded.\n\n\
                **Step 2: Energy Detection**\n\
                For each bit period:\n\
                - Measure signal energy in first half (0-0.5us)\n\
                - Measure signal energy in second half (0.5-1us)\n\
                - Compare: more energy in first = '1', second = '0'\n\n\
                **Why Energy Detection Works:**\n\
                Manchester encoding guarantees a pulse in one half.\n\
                Simple threshold comparison recovers the bit.\n\n\
                **Real-world Considerations:**\n\
                - Automatic gain control (AGC)\n\
                - Noise rejection thresholds\n\
                - Preamble correlation for detection",
            concepts: &["Energy detection", "Correlation", "Threshold decision"],
        },
    ],
};
