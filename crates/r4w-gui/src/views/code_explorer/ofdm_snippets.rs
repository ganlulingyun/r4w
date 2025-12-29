//! OFDM (Orthogonal Frequency Division Multiplexing) code snippets
//!
//! OFDM is the foundation of modern wireless: WiFi, LTE, 5G, DVB-T.
//! It uses FFT/IFFT for efficient multi-carrier modulation.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete OFDM waveform code documentation
pub static OFDM_CODE: WaveformCode = WaveformCode {
    waveform_id: "OFDM",
    display_name: "Orthogonal Frequency Division Multiplexing (OFDM)",
    introduction: "OFDM divides the available bandwidth into many narrow subcarriers, each \
        carrying a low-rate data stream. The key insight is using FFT/IFFT for efficient \
        implementation. A cyclic prefix provides immunity to multipath. OFDM is the basis \
        of WiFi (802.11a/g/n/ac/ax), LTE, 5G-NR, DVB-T, DAB, and DSL.",
    complexity: 5,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
        &PRACTICAL_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "OFDM Fundamentals",
    description: "Core concepts of multi-carrier modulation",
    snippets: &[
        CodeSnippet {
            name: "OFDM struct",
            brief: "OFDM modulator/demodulator parameters",
            code: r#"pub struct OFDM {
    common: CommonParams,
    /// Number of FFT points (total subcarriers)
    fft_size: usize,
    /// Number of data subcarriers (active)
    num_data_subcarriers: usize,
    /// Cyclic prefix length (samples)
    cyclic_prefix_len: usize,
    /// Subcarrier modulation (BPSK, QPSK, 16-QAM, 64-QAM)
    subcarrier_mod: SubcarrierModulation,
    /// Subcarrier spacing in Hz
    subcarrier_spacing: f64,
}"#,
            explanation: "**Key OFDM Parameters:**\n\n\
                **FFT Size:**\n\
                - Total number of subcarriers (typically 64, 256, 1024, 2048)\n\
                - WiFi uses 64, LTE uses up to 2048\n\n\
                **Data Subcarriers:**\n\
                - Active carriers that carry data\n\
                - WiFi: 48 data + 4 pilot out of 64\n\
                - Guard bands and DC null don't carry data\n\n\
                **Cyclic Prefix:**\n\
                - Guard interval to absorb multipath\n\
                - Typically 1/4 or 1/8 of symbol duration\n\n\
                **Subcarrier Spacing:**\n\
                - spacing = sample_rate / fft_size\n\
                - WiFi: 20 MHz / 64 = 312.5 kHz",
            concepts: &["FFT size", "Subcarrier spacing", "Cyclic prefix", "Guard bands"],
        },
        CodeSnippet {
            name: "Orthogonality",
            brief: "Why subcarriers don't interfere",
            code: r#"// Subcarrier frequencies are multiples of 1/T_symbol
// f_k = k / T_symbol = k * subcarrier_spacing
//
// Key property: ∫ exp(j2πf_k*t) * exp(-j2πf_m*t) dt = 0 for k≠m
//
// This means subcarriers are orthogonal over one symbol period.
// Even though they overlap in frequency, they don't interfere!
//
// The FFT naturally creates this orthogonal spacing."#,
            explanation: "**Orthogonality Explained:**\n\n\
                OFDM subcarriers overlap in frequency but don't interfere.\n\n\
                **Why It Works:**\n\
                At the center of each subcarrier, all other subcarriers\n\
                have a zero crossing. The FFT samples exactly at these points.\n\n\
                **Frequency Spacing:**\n\
                Δf = 1/T_symbol ensures orthogonality.\n\
                Closer spacing would cause interference.\n\n\
                **The FFT Connection:**\n\
                The DFT basis functions e^(j2πkn/N) are exactly\n\
                the subcarrier frequencies. FFT implements this efficiently.",
            concepts: &["Orthogonality", "Subcarrier spacing", "FFT basis functions"],
        },
        CodeSnippet {
            name: "Cyclic Prefix",
            brief: "Guard interval for multipath immunity",
            code: r#"// Add cyclic prefix (copy end to beginning)
fn add_cyclic_prefix(symbol: &[IQSample], cp_len: usize) -> Vec<IQSample> {
    let mut output = Vec::with_capacity(symbol.len() + cp_len);

    // Copy last cp_len samples to the front
    output.extend_from_slice(&symbol[symbol.len() - cp_len..]);
    // Then append the full symbol
    output.extend_from_slice(symbol);

    output
}

// WiFi example: 64 samples + 16 CP = 80 samples per symbol"#,
            explanation: "**Cyclic Prefix Purpose:**\n\n\
                Multipath causes copies of the signal to arrive at different times.\n\
                Without CP, these delayed copies cause inter-symbol interference.\n\n\
                **How It Works:**\n\
                1. Copy the end of the symbol to the beginning\n\
                2. Delayed copies 'wrap around' into the CP region\n\
                3. Receiver discards CP, leaving clean symbol\n\n\
                **Trade-off:**\n\
                - Longer CP = more multipath tolerance\n\
                - But wastes bandwidth (overhead)\n\
                - WiFi: 1/4 CP = 20% overhead\n\n\
                **Why Cyclic?**\n\
                The circular property of the DFT means copying the end\n\
                to the beginning makes convolution appear circular.",
            concepts: &["Cyclic prefix", "Multipath", "ISI immunity", "Guard interval"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "OFDM Modulation",
    description: "Generating OFDM signals using IFFT",
    snippets: &[
        CodeSnippet {
            name: "modulate_symbol()",
            brief: "IFFT-based OFDM symbol generation",
            code: r#"fn modulate_symbol(&self, subcarriers: &[IQSample]) -> Vec<IQSample> {
    // Create frequency domain representation
    let mut freq_domain = vec![Complex::new(0.0, 0.0); self.fft_size];

    // Place data on subcarriers (with guard bands and DC null)
    for (i, &sample) in subcarriers.iter().enumerate() {
        let fft_idx = self.data_to_fft_index(i);
        freq_domain[fft_idx] = Complex::new(sample.re, sample.im);
    }

    // IFFT: frequency domain → time domain
    let mut planner = FftPlanner::new();
    let ifft = planner.plan_fft_inverse(self.fft_size);
    ifft.process(&mut freq_domain);

    // Normalize and add cyclic prefix
    let scale = 1.0 / (self.fft_size as f64).sqrt();
    // ... convert to IQSample and add CP
}"#,
            explanation: "**IFFT for OFDM Modulation:**\n\n\
                The genius of OFDM is using IFFT for modulation.\n\n\
                **Concept:**\n\
                - Input: N complex symbols (one per subcarrier)\n\
                - IFFT: Convert to time domain\n\
                - Output: N time samples that contain all subcarriers\n\n\
                **Why IFFT?**\n\
                IFFT creates a sum of sinusoids at frequencies k/N.\n\
                Each input bin controls one sinusoid's amplitude/phase.\n\n\
                **Efficiency:**\n\
                - Direct: O(N²) multiplications\n\
                - FFT: O(N log N) multiplications\n\
                - For N=2048: 2000x speedup!\n\n\
                **Normalization:**\n\
                IFFT output is scaled by 1/sqrt(N) to maintain power.",
            concepts: &["IFFT", "Frequency to time", "Computational efficiency"],
        },
        CodeSnippet {
            name: "Subcarrier Mapping",
            brief: "How data maps to FFT bins",
            code: r#"/// Map data subcarrier index to FFT bin index
/// Uses centered allocation: DC null, guard bands at edges
fn data_to_fft_index(&self, data_idx: usize) -> usize {
    let half = self.num_data_subcarriers / 2;
    if data_idx < half {
        // Negative frequencies (upper half of FFT)
        self.fft_size - half + data_idx
    } else {
        // Positive frequencies (lower half of FFT, skip DC)
        data_idx - half + 1
    }
}

// Example: 64 FFT, 52 data subcarriers
// Bins 0 (DC), 27-37 (guard) are unused
// Data goes in bins 1-26 and 38-63"#,
            explanation: "**Subcarrier Allocation:**\n\n\
                Not all FFT bins carry data:\n\n\
                **DC Subcarrier (bin 0):**\n\
                - Always null - DC offset causes receiver problems\n\n\
                **Guard Bands:**\n\
                - Edge subcarriers left empty\n\
                - Provides transition to adjacent channels\n\
                - Makes filtering easier\n\n\
                **Pilot Subcarriers:**\n\
                - Known symbols for channel estimation\n\
                - WiFi uses 4 pilots at fixed positions\n\n\
                **FFT Bin Ordering:**\n\
                - Bins 0 to N/2-1: DC and positive frequencies\n\
                - Bins N/2 to N-1: Negative frequencies",
            concepts: &["Subcarrier mapping", "DC null", "Guard bands", "Pilots"],
        },
        CodeSnippet {
            name: "SubcarrierModulation",
            brief: "Per-subcarrier constellation mapping",
            code: r#"pub enum SubcarrierModulation {
    Bpsk,   // 1 bit/subcarrier
    Qpsk,   // 2 bits/subcarrier
    Qam16,  // 4 bits/subcarrier
    Qam64,  // 6 bits/subcarrier
}

impl SubcarrierModulation {
    pub fn modulate(&self, bits: &[u8]) -> IQSample {
        match self {
            Self::Qpsk => {
                let scale = 1.0 / 2.0_f64.sqrt();
                let i = if bits[0] == 0 { scale } else { -scale };
                let q = if bits[1] == 0 { scale } else { -scale };
                IQSample::new(i, q)
            }
            // Similar for other modulations
        }
    }
}"#,
            explanation: "**Adaptive Modulation:**\n\n\
                Each subcarrier can use different modulation.\n\n\
                **Standard Choices:**\n\
                - BPSK: 1 bit, most robust, -1 dB SNR\n\
                - QPSK: 2 bits, 3 dB more SNR needed\n\
                - 16-QAM: 4 bits, 10 dB SNR\n\
                - 64-QAM: 6 bits, 16 dB SNR\n\
                - 256-QAM: 8 bits (WiFi 6), 22+ dB SNR\n\n\
                **Rate Adaptation:**\n\
                Real systems switch modulation based on SNR:\n\
                - Poor signal: Use QPSK (slower but reliable)\n\
                - Good signal: Use 64-QAM (faster)\n\n\
                **Bits per OFDM Symbol:**\n\
                Total bits = num_data_subcarriers × bits_per_subcarrier",
            concepts: &["Adaptive modulation", "Rate adaptation", "Spectral efficiency"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "OFDM Demodulation",
    description: "Recovering data using FFT",
    snippets: &[
        CodeSnippet {
            name: "demodulate_symbol()",
            brief: "FFT-based OFDM demodulation",
            code: r#"fn demodulate_symbol(&self, samples: &[IQSample]) -> Vec<IQSample> {
    // Remove cyclic prefix
    let time_samples = &samples[self.cyclic_prefix_len..];

    // Convert to Complex for FFT
    let mut freq_domain: Vec<Complex<f64>> = time_samples
        .iter()
        .map(|s| Complex::new(s.re, s.im))
        .collect();

    // FFT: time domain → frequency domain
    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(self.fft_size);
    fft.process(&mut freq_domain);

    // Extract data subcarriers
    let mut subcarriers = Vec::new();
    for i in 0..self.num_data_subcarriers {
        let fft_idx = self.data_to_fft_index(i);
        subcarriers.push(freq_domain[fft_idx]);
    }
    subcarriers
}"#,
            explanation: "**FFT for OFDM Demodulation:**\n\n\
                Demodulation is the reverse of modulation.\n\n\
                **Steps:**\n\
                1. Remove cyclic prefix (discard first CP samples)\n\
                2. Apply FFT to remaining N samples\n\
                3. Extract data subcarriers from FFT output\n\
                4. Demodulate each subcarrier (QPSK → bits)\n\n\
                **The Magic:**\n\
                FFT separates all subcarriers simultaneously.\n\
                Each FFT bin gives one subcarrier's symbol.\n\n\
                **Channel Effects:**\n\
                In real systems, need channel equalization:\n\
                - Estimate channel from pilots\n\
                - Divide each subcarrier by channel estimate",
            concepts: &["FFT demodulation", "CP removal", "Subcarrier extraction"],
        },
        CodeSnippet {
            name: "Channel Estimation",
            brief: "Using pilots for equalization",
            code: r#"// Pilot-based channel estimation (simplified)
fn estimate_channel(rx_pilots: &[IQSample], tx_pilots: &[IQSample]) -> Vec<IQSample> {
    // H = Rx / Tx for each pilot
    rx_pilots.iter().zip(tx_pilots.iter())
        .map(|(rx, tx)| {
            let h = rx / tx;  // Complex division
            h
        })
        .collect()
}

// Equalization: divide received symbol by channel estimate
fn equalize(rx_symbol: IQSample, channel_est: IQSample) -> IQSample {
    rx_symbol / channel_est
}"#,
            explanation: "**Channel Estimation:**\n\n\
                The wireless channel distorts each subcarrier differently.\n\n\
                **Pilot Symbols:**\n\
                - Known symbols at fixed positions\n\
                - Receiver knows what was sent\n\
                - Compare received vs expected\n\n\
                **Channel Response:**\n\
                H[k] = Rx[k] / Tx[k] for each pilot k\n\
                Interpolate between pilots for data subcarriers.\n\n\
                **Equalization:**\n\
                Corrected[k] = Rx[k] / H[k]\n\
                This removes channel distortion.\n\n\
                **Why Per-Subcarrier?**\n\
                OFDM converts frequency-selective fading into\n\
                flat fading on each narrow subcarrier.",
            concepts: &["Channel estimation", "Pilots", "Equalization", "Frequency-selective fading"],
        },
    ],
};

static PRACTICAL_CATEGORY: CodeCategory = CodeCategory {
    name: "OFDM in Practice",
    description: "Real-world OFDM systems",
    snippets: &[
        CodeSnippet {
            name: "WiFi Parameters",
            brief: "802.11a/g OFDM configuration",
            code: r#"// WiFi 802.11a/g OFDM parameters
let wifi = OFDM {
    fft_size: 64,
    num_data_subcarriers: 48,  // + 4 pilots
    cyclic_prefix_len: 16,      // 1/4 of symbol
    sample_rate: 20_000_000.0,  // 20 MHz
    // ...
};

// Derived parameters:
// Subcarrier spacing: 20 MHz / 64 = 312.5 kHz
// Symbol duration: 64 / 20 MHz = 3.2 μs
// With CP: 80 / 20 MHz = 4.0 μs
// Data rate (64-QAM, 3/4 code): 54 Mbps"#,
            explanation: "**WiFi OFDM:**\n\n\
                **Key Numbers:**\n\
                - 64 subcarriers total\n\
                - 48 data + 4 pilot + 12 null (guard + DC)\n\
                - 312.5 kHz subcarrier spacing\n\
                - 4 μs symbol duration (with CP)\n\n\
                **Data Rates:**\n\
                - BPSK 1/2: 6 Mbps\n\
                - QPSK 3/4: 18 Mbps\n\
                - 16-QAM 3/4: 36 Mbps\n\
                - 64-QAM 3/4: 54 Mbps\n\n\
                **WiFi 6 (802.11ax):**\n\
                - Up to 1024 subcarriers\n\
                - 1024-QAM option\n\
                - OFDMA for multi-user",
            concepts: &["WiFi", "802.11", "Data rates", "Channel bandwidth"],
        },
        CodeSnippet {
            name: "PAPR Problem",
            brief: "Peak-to-Average Power Ratio issue",
            code: r#"// OFDM's Achilles heel: high PAPR
//
// When many subcarriers add in phase, the peak can be
// much higher than the average power.
//
// Worst case: all N subcarriers align
// PAPR_max = 10 * log10(N) dB
// For N=64: 18 dB!
//
// This requires linear (inefficient) amplifiers
// or clipping (causes distortion).
//
// Solutions: SC-FDMA (LTE uplink), clipping, tone reservation"#,
            explanation: "**PAPR Problem:**\n\n\
                OFDM signals can have very high peaks.\n\n\
                **Why It Happens:**\n\
                Adding N sinusoids can constructively interfere.\n\
                Peak amplitude can be N× average.\n\n\
                **Impact:**\n\
                - Amplifiers must handle peaks without clipping\n\
                - But average power determines range\n\
                - High PAPR = inefficient, expensive amplifiers\n\n\
                **Solutions:**\n\
                - Clipping: Clip peaks, accept distortion\n\
                - SC-FDMA: Single-carrier variant (LTE uplink)\n\
                - Tone reservation: Reserve subcarriers for PAPR reduction\n\
                - Coding: Choose sequences with low PAPR",
            concepts: &["PAPR", "Amplifier efficiency", "SC-FDMA", "Peak clipping"],
        },
        CodeSnippet {
            name: "OFDMA",
            brief: "Multi-user OFDM access",
            code: r#"// OFDMA: Assign different subcarriers to different users
//
// LTE Resource Block: 12 subcarriers × 7 symbols
//
// Example: 50 RBs (10 MHz), 3 users
// User A: RBs 0-19 (40% of bandwidth)
// User B: RBs 20-39 (40% of bandwidth)
// User C: RBs 40-49 (20% of bandwidth)
//
// Each user gets their own slice of subcarriers.
// Scheduler assigns based on channel quality and demand."#,
            explanation: "**OFDMA (OFDM Access):**\n\n\
                Share bandwidth among multiple users.\n\n\
                **How It Works:**\n\
                - Divide subcarriers into groups (resource blocks)\n\
                - Assign RBs to different users\n\
                - Each user's data on their subcarriers\n\n\
                **Advantages:**\n\
                - Flexible bandwidth allocation\n\
                - Frequency diversity for each user\n\
                - Efficient for bursty traffic\n\n\
                **Used In:**\n\
                - LTE/5G downlink and uplink\n\
                - WiFi 6 (802.11ax) - first WiFi with OFDMA\n\n\
                **Scheduling:**\n\
                Assign good subcarriers to each user\n\
                (frequency-selective scheduling gain).",
            concepts: &["OFDMA", "Resource blocks", "Multi-user", "LTE/5G"],
        },
    ],
};
