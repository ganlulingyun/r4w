//! CSS (Chirp Spread Spectrum) / LoRa code snippets
//!
//! CSS is LoRa's modulation scheme - it encodes data by cyclically shifting
//! chirp signals. This provides exceptional range and noise immunity.

use super::snippets::{CodeCategory, CodeSnippet, WaveformCode};

/// Complete CSS/LoRa waveform code documentation
pub static CSS_CODE: WaveformCode = WaveformCode {
    waveform_id: "CSS",
    display_name: "Chirp Spread Spectrum (LoRa)",
    introduction: "CSS is LoRa's secret weapon for long-range, low-power communication. Instead of \
        fixed frequencies (FSK) or phases (PSK), CSS uses chirps - signals that sweep across the \
        entire bandwidth. Data is encoded by cyclically shifting when the chirp wraps around. \
        The genius: multiply a received chirp by a reference downchirp, and you get a single tone \
        whose frequency reveals the symbol. An FFT finds this peak instantly. CSS provides ~20dB \
        processing gain, enabling communication below the noise floor!",
    complexity: 5,
    categories: &[
        &FUNDAMENTALS_CATEGORY,
        &CHIRP_GENERATION_CATEGORY,
        &MODULATION_CATEGORY,
        &DEMODULATION_CATEGORY,
    ],
};

static FUNDAMENTALS_CATEGORY: CodeCategory = CodeCategory {
    name: "CSS Fundamentals",
    description: "What makes chirps special and why LoRa uses them",
    snippets: &[
        CodeSnippet {
            name: "Chirp Concept",
            brief: "A signal whose frequency changes linearly over time",
            code: r#"// A chirp sweeps frequency linearly:
//
// Frequency
//     ^
// f_max|        ___/     (upchirp)
//     |     __/
//     |  __/
// f_min|_/
//     +----------> Time
//
// Instantaneous frequency: f(t) = f_init + (BW/T) * t
// Phase (integral): φ(t) = 2π * (f_init*t + (BW/2T)*t²)
// Signal: s(t) = e^(j*φ(t)) = cos(φ) + j*sin(φ)

let df = bandwidth / symbol_duration;  // Chirp rate (Hz/s)
let phase = 2.0 * PI * (f_init * t + df / 2.0 * t * t);"#,
            explanation: "**Chirps** are the foundation of CSS modulation.\n\n\
                \
                **The Math:**\n\
                A chirp's frequency increases (or decreases) linearly:\n\
                f(t) = f₀ + (Δf/T)×t\n\n\
                Phase is the integral of frequency:\n\
                φ(t) = 2π×∫f(t)dt = 2π×(f₀t + (Δf/2T)t²)\n\n\
                The t² term is what makes it a chirp - quadratic phase!\n\n\
                \
                **Why Chirps?**\n\
                1. **Spread spectrum**: Energy spreads across bandwidth\n\
                2. **Noise immunity**: Interference at one frequency only affects part of signal\n\
                3. **Multipath resistance**: Delayed copies are distinguishable\n\
                4. **Simple detection**: One FFT reveals the symbol\n\n\
                \
                **Processing Gain:**\n\
                For SF7: 2^7 = 128 chips → 10×log10(128) ≈ 21 dB gain!",
            concepts: &["Linear frequency sweep", "Quadratic phase", "Spread spectrum", "Processing gain"],
        },
        CodeSnippet {
            name: "Spreading Factor",
            brief: "Trade-off between range and data rate",
            code: r#"// Spreading Factor (SF) determines:
// - Chips per symbol: 2^SF (e.g., SF7 = 128 chips)
// - Symbol duration: 2^SF / BW seconds
// - Data rate: BW * SF / 2^SF bits/second
// - Sensitivity: ~3dB improvement per SF increase

// Example: SF7, BW=125kHz
let sf = 7;
let chips_per_symbol = 1 << sf;        // 128
let symbol_duration = 128.0 / 125000.0; // 1.024 ms
let data_rate = 125000.0 * 7.0 / 128.0; // ~6836 bps

// SF12 comparison:
// chips = 4096, duration = 32.77ms, rate = ~293 bps
// But 15dB more sensitive than SF7!"#,
            explanation: "**Spreading Factor** is LoRa's key trade-off parameter.\n\n\
                \
                **What SF Controls:**\n\
                | SF | Chips | Duration (BW=125k) | Data Rate | Relative Range |\n\
                |----|-------|-------------------|-----------|----------------|\n\
                | 7  | 128   | 1.02 ms           | 5.5 kbps  | 1x             |\n\
                | 8  | 256   | 2.05 ms           | 3.1 kbps  | 1.4x           |\n\
                | 9  | 512   | 4.10 ms           | 1.8 kbps  | 2x             |\n\
                | 10 | 1024  | 8.19 ms           | 1.0 kbps  | 2.8x           |\n\
                | 11 | 2048  | 16.38 ms          | 0.5 kbps  | 4x             |\n\
                | 12 | 4096  | 32.77 ms          | 0.3 kbps  | 5.6x           |\n\n\
                \
                **The Trade-off:**\n\
                Higher SF = more chips = longer symbols = more energy collected\n\
                → Better sensitivity but slower data rate\n\n\
                \
                **Orthogonality:**\n\
                Different SFs are orthogonal - multiple devices can transmit \
                simultaneously at different SFs without interference!",
            concepts: &["Spreading factor", "Chips per symbol", "Sensitivity trade-off", "SF orthogonality"],
        },
    ],
};

static CHIRP_GENERATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Chirp Generation",
    description: "Creating base chirps and symbol-modulated chirps",
    snippets: &[
        CodeSnippet {
            name: "generate_base_chirp()",
            brief: "Generate an unshifted reference chirp",
            code: r#"fn generate_base_chirp(params: &LoRaParams, chirp_type: ChirpType) -> Vec<IQSample> {
    let n = params.samples_per_symbol();
    let bw = params.bw.hz();
    let ts = params.sample_duration();
    let t_symbol = params.symbol_duration();

    // Chirp rate: how fast frequency changes
    let df = bw / t_symbol;  // Hz per second

    // Starting frequency
    let f_init = match chirp_type {
        ChirpType::Up => -bw / 2.0,    // Start at -BW/2
        ChirpType::Down => bw / 2.0,   // Start at +BW/2
    };

    let sign = match chirp_type {
        ChirpType::Up => 1.0,
        ChirpType::Down => -1.0,
    };

    (0..n).map(|i| {
        let t = i as f64 * ts;
        // Phase = 2π × (f_init×t + sign×df/2×t²)
        let phase = 2.0 * PI * (f_init * t + sign * df / 2.0 * t * t);
        Complex::new(phase.cos(), phase.sin())
    }).collect()
}"#,
            explanation: "**Base Chirp Generation** creates the reference signals.\n\n\
                \
                **Upchirp vs Downchirp:**\n\
                - Upchirp: frequency sweeps -BW/2 → +BW/2\n\
                - Downchirp: frequency sweeps +BW/2 → -BW/2\n\n\
                \
                **The Phase Formula:**\n\
                φ(t) = 2π×(f_init×t + df/2×t²)\n\n\
                The t² term creates the frequency sweep:\n\
                - dφ/dt = 2π×(f_init + df×t) = 2π×f(t)\n\
                - Frequency increases linearly with time!\n\n\
                \
                **Pre-computation:**\n\
                We generate base chirps once and store them. Symbol chirps \
                are created by cyclically rotating the base chirp.\n\n\
                \
                **Constant Envelope:**\n\
                |e^(jφ)| = 1 always. Chirps have constant amplitude, \
                making them robust to amplitude distortion.",
            concepts: &["Base chirp", "Upchirp/downchirp", "Quadratic phase", "Pre-computation"],
        },
        CodeSnippet {
            name: "generate_symbol_chirp_fast()",
            brief: "Create a symbol-modulated chirp via cyclic rotation",
            code: r#"pub fn generate_symbol_chirp_fast(&self, symbol: Symbol) -> Vec<IQSample> {
    let n = self.params.samples_per_symbol();
    let k = self.params.chips_per_symbol();
    let osf = self.params.oversample;

    // Symbol determines cyclic shift amount
    let shift_samples = (symbol as usize * osf) % n;

    if shift_samples == 0 {
        return self.base_upchirp.clone();  // Symbol 0 = no shift
    }

    // Phase correction for wrap-around discontinuity
    let bw = self.params.bw.hz();
    let t_symbol = self.params.symbol_duration();
    let phase_correction = 2.0 * PI * bw * (symbol as f64 / k as f64) * t_symbol;
    let correction = Complex::new(phase_correction.cos(), phase_correction.sin());

    // Cyclically rotate the base chirp
    let mut chirp = Vec::with_capacity(n);
    for i in 0..n {
        let src_idx = (i + shift_samples) % n;
        let sample = if i + shift_samples >= n {
            // After wrap: apply phase correction
            self.base_upchirp[src_idx] * correction
        } else {
            self.base_upchirp[src_idx]
        };
        chirp.push(sample);
    }
    chirp
}"#,
            explanation: "**Symbol Encoding via Cyclic Rotation**\n\n\
                \
                **The Key Insight:**\n\
                Instead of regenerating the chirp for each symbol, we rotate \
                the base chirp cyclically. Symbol N shifts the chirp by N chips.\n\n\
                \
                **Visual Example (SF7 = 128 chips):**\n\
                ```\n\
                Symbol 0:       Symbol 32:\n\
                freq↑  /        freq↑    /\n\
                    __/              ___/\n\
                  _/              __/\n\
                _/             __/\n\
                              /  ← wrap point\n\
                ```\n\n\
                \
                **Phase Correction:**\n\
                When the chirp wraps around, there's a phase discontinuity.\n\
                We must correct this to maintain phase continuity.\n\n\
                \
                **Why Cyclic Rotation Works:**\n\
                Dechirping (multiply by downchirp) converts the shift into a \
                frequency offset. FFT finds this offset = symbol value!\n\n\
                \
                **Efficiency:**\n\
                O(n) rotation vs O(n×log(n)) regeneration saves computation.",
            concepts: &["Cyclic rotation", "Symbol encoding", "Phase correction", "Wrap-around"],
        },
    ],
};

static MODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Modulation Pipeline",
    description: "The complete LoRa transmit chain",
    snippets: &[
        CodeSnippet {
            name: "LoRa TX Pipeline",
            brief: "Full modulation chain: data → I/Q samples",
            code: r#"// The complete LoRa modulation pipeline:
//
// Raw Data
//    ↓
// [Whitening]     XOR with LFSR sequence
//    ↓
// [Hamming FEC]   4 data bits → 5-8 coded bits
//    ↓
// [Interleave]    Spread bits across symbols
//    ↓
// [Gray Coding]   Minimize bit errors from symbol errors
//    ↓
// [CSS Modulation] Generate frequency-shifted chirps
//    ↓
// I/Q Samples

pub fn modulate(&mut self, payload: &[u8]) -> Vec<IQSample> {
    // 1. Whiten data
    let mut whitened = payload.to_vec();
    self.whitening.process(&mut whitened);

    // 2. Convert bytes to nibbles (4-bit)
    let nibbles = Self::bytes_to_nibbles(&whitened);

    // 3. Hamming encode each nibble
    let codewords: Vec<u8> = nibbles.iter()
        .map(|&n| self.hamming.encode(n))
        .collect();

    // 4. Interleave and Gray encode → symbols
    let symbols = self.encode_to_symbols(&codewords);

    // 5. Generate preamble + payload chirps
    let mut samples = self.chirp_gen.generate_preamble();
    for &symbol in &symbols {
        samples.extend(self.chirp_gen.generate_symbol_chirp_fast(symbol));
    }

    samples
}"#,
            explanation: "**The Full LoRa TX Pipeline**\n\n\
                \
                **Stage 1: Whitening**\n\
                XOR with pseudo-random sequence to ensure balanced 0s/1s.\n\
                Prevents long runs that could affect synchronization.\n\n\
                \
                **Stage 2: Hamming FEC**\n\
                Each 4-bit nibble gets 1-4 parity bits (CR 4/5 to 4/8).\n\
                Enables error correction at the receiver.\n\n\
                \
                **Stage 3: Interleaving**\n\
                Spreads bits across multiple symbols so burst errors \
                affect different codewords → correctable errors.\n\n\
                \
                **Stage 4: Gray Coding**\n\
                Adjacent symbol values differ by 1 bit, so a ±1 symbol \
                error causes only 1 bit error.\n\n\
                \
                **Stage 5: CSS Modulation**\n\
                Each symbol becomes a cyclically-shifted chirp.\n\
                Preamble added for synchronization.\n\n\
                \
                **Output:**\n\
                Complex I/Q samples ready for SDR transmission.",
            concepts: &["TX pipeline", "Whitening", "FEC", "Interleaving", "Gray coding"],
        },
        CodeSnippet {
            name: "generate_preamble()",
            brief: "Create the synchronization preamble",
            code: r#"pub fn generate_preamble(&self) -> Vec<IQSample> {
    let n = self.params.samples_per_symbol();
    let k = self.params.chips_per_symbol() as u16;
    let mut preamble = Vec::new();

    // 1. Add N base upchirps (typically 8)
    for _ in 0..self.params.preamble_length {
        preamble.extend_from_slice(&self.base_upchirp);
    }

    // 2. Add sync word chirps (shifted upchirps)
    // Standard sync word 0x12 → symbols (K-8) and (K-16)
    let sync1 = k.saturating_sub(8);
    let sync2 = k.saturating_sub(16);
    preamble.extend(self.generate_symbol_chirp_fast(sync1));
    preamble.extend(self.generate_symbol_chirp_fast(sync2));

    // 3. Add 2.25 downchirps (marks end of preamble)
    preamble.extend_from_slice(&self.base_downchirp);
    preamble.extend_from_slice(&self.base_downchirp);
    preamble.extend_from_slice(&self.base_downchirp[..n / 4]);

    preamble
}"#,
            explanation: "**The LoRa Preamble** enables receiver synchronization.\n\n\
                \
                **Preamble Structure:**\n\
                1. **8+ Upchirps**: Receiver detects repeated pattern\n\
                2. **2 Sync Word Chirps**: Network identification (default 0x12)\n\
                3. **2.25 Downchirps**: Marks preamble end, enables timing sync\n\n\
                \
                **Why This Pattern?**\n\
                - Repeated upchirps: easy energy detection\n\
                - Sync word: distinguishes networks (public vs private)\n\
                - Downchirps: phase reversal is unmistakable\n\n\
                \
                **Timing Recovery:**\n\
                The downchirps provide precise timing. When dechirped, \
                they produce a peak at a known position.\n\n\
                \
                **CFO Estimation:**\n\
                Carrier frequency offset shifts all FFT peaks. The preamble's \
                known pattern allows measuring and correcting this offset.",
            concepts: &["Preamble", "Sync word", "Timing recovery", "CFO estimation"],
        },
    ],
};

static DEMODULATION_CATEGORY: CodeCategory = CodeCategory {
    name: "Demodulation",
    description: "FFT-based symbol detection - the CSS magic trick",
    snippets: &[
        CodeSnippet {
            name: "The Dechirping Trick",
            brief: "Multiply by downchirp → single tone → FFT peak = symbol",
            code: r#"// THE KEY INSIGHT OF CSS DEMODULATION:
//
// received_chirp × conjugate(downchirp) = single_tone
//
// Why? Upchirp has phase: φ_up(t) = 2π(f₀t + kt²/2)
//      Downchirp has phase: φ_down(t) = 2π(f₀t - kt²/2)
//      Product phase: φ_up - φ_down = 2πkt² - ... wait, that's not right
//
// Actually, for symbol s with cyclic shift:
// The dechirped signal is e^(j2π·s/N·n) - a pure tone at bin s!
//
// So: FFT of (received × downchirp) has peak at bin = symbol value

let downchirp = self.chirp_gen.base_downchirp();

// Dechirp: multiply element-wise
let mixed: Vec<Complex> = received[..n]
    .iter()
    .zip(downchirp.iter())
    .map(|(&rx, &dc)| rx * dc)
    .collect();

// FFT finds the tone frequency
let spectrum = self.fft.fft(&mixed);

// Peak bin = symbol value!
let (symbol, magnitude, _) = FftProcessor::find_peak(&spectrum);"#,
            explanation: "**The CSS Dechirping Magic**\n\n\
                \
                **The Math:**\n\
                Received symbol s is an upchirp shifted by s chips.\n\
                Multiply by conjugate of downchirp (= upchirp):\n\
                \n\
                shifted_upchirp × conj(base_upchirp) = e^(j2π·s·n/N)\n\n\
                This is a pure sinusoid at frequency s/N!\n\n\
                \
                **Why It Works:**\n\
                The chirp's frequency sweep cancels out, leaving only \
                the frequency offset caused by the cyclic shift.\n\n\
                \
                **The FFT:**\n\
                FFT converts the tone to a spectral peak.\n\
                Peak bin index = symbol value (0 to 2^SF - 1)\n\n\
                \
                **Noise Immunity:**\n\
                FFT integrates energy across N samples → processing gain.\n\
                Noise averages out; signal coherently accumulates.\n\n\
                \
                **One FFT, One Symbol:**\n\
                No correlation bank, no matched filters - just one FFT!",
            concepts: &["Dechirping", "Conjugate multiplication", "FFT peak detection", "Processing gain"],
        },
        CodeSnippet {
            name: "demodulate_symbol()",
            brief: "Extract one symbol from samples using FFT",
            code: r#"pub fn demodulate_symbol(&mut self, samples: &[IQSample]) -> SymbolResult {
    let n = self.params.samples_per_symbol();
    let downchirp = self.chirp_gen.base_downchirp();

    // Step 1: Dechirp (multiply by downchirp)
    let mixed: Vec<Complex> = samples[..n]
        .iter()
        .zip(downchirp.iter())
        .map(|(&s, &d)| s * d)
        .collect();

    // Step 2: FFT
    let spectrum = self.fft.fft(&mixed);

    // Step 3: Find peak (interpolated for sub-bin precision)
    let (peak_idx, magnitude) = FftProcessor::find_peak_interpolated(&spectrum);
    let (peak_bin, _, phase) = FftProcessor::find_peak(&spectrum);

    // Symbol is the peak bin index
    let symbol = peak_bin as u16;

    // Step 4: Estimate SNR from peak-to-average ratio
    let avg_power: f64 = spectrum.iter()
        .map(|c| c.norm_sqr())
        .sum::<f64>() / spectrum.len() as f64;
    let peak_power = magnitude * magnitude;
    let snr_estimate = 10.0 * (peak_power / avg_power).log10();

    SymbolResult { symbol, magnitude, phase, snr_estimate: Some(snr_estimate) }
}"#,
            explanation: "**Single Symbol Demodulation**\n\n\
                \
                **The Algorithm:**\n\
                1. **Dechirp**: Multiply received by downchirp conjugate\n\
                2. **FFT**: Convert to frequency domain\n\
                3. **Peak Find**: Highest bin = symbol value\n\
                4. **SNR Estimate**: Peak power / average power\n\n\
                \
                **Peak Interpolation:**\n\
                For better accuracy, we interpolate between bins using \
                neighboring magnitudes. This helps with CFO.\n\n\
                \
                **SNR Estimation:**\n\
                Peak-to-average ratio in dB tells us signal quality.\n\
                SNR > 0 dB means detectable; SNR > 10 dB is comfortable.\n\n\
                \
                **Complexity:**\n\
                O(N log N) for FFT where N = 2^SF samples.\n\
                SF12: 4096-point FFT - still fast!\n\n\
                \
                **Phase Information:**\n\
                We also extract phase, useful for fine timing and \
                coherent combining of multiple symbols.",
            concepts: &["Symbol demodulation", "FFT peak detection", "SNR estimation", "Peak interpolation"],
        },
        CodeSnippet {
            name: "Full RX Pipeline",
            brief: "Complete demodulation: I/Q samples → payload bytes",
            code: r#"// The complete LoRa receive pipeline:
//
// I/Q Samples (from SDR)
//    ↓
// [Preamble Detect]  Find packet start
//    ↓
// [Synchronization]  CFO & timing estimation
//    ↓
// [CSS Demod]        FFT-based symbol extraction
//    ↓
// [Gray Decode]      Convert Gray to binary
//    ↓
// [De-interleave]    Reverse bit spreading
//    ↓
// [Hamming Decode]   Error correction
//    ↓
// [De-whitening]     XOR to recover original
//    ↓
// Raw Data

pub fn demodulate(&mut self, samples: &[IQSample]) -> DspResult<DemodulationResult> {
    let n = self.params.samples_per_symbol();
    let num_symbols = samples.len() / n;

    // 1. Demodulate all symbols (FFT-based)
    let symbol_results = self.demodulate_symbols(samples, num_symbols);
    let symbols: Vec<Symbol> = symbol_results.iter().map(|r| r.symbol).collect();

    // 2. Gray decode
    let gray_decoded: Vec<Symbol> = symbols.iter()
        .map(|&s| self.gray.decode(s))
        .collect();

    // 3. De-interleave and Hamming decode
    let payload = self.decode_symbols(&gray_decoded);

    // 4. De-whiten
    let mut dewhitened = payload.clone();
    self.whitening.process(&mut dewhitened);

    Ok(DemodulationResult {
        payload: dewhitened,
        symbols,
        rssi: calculate_rssi(&symbol_results),
        cfo: 0.0,  // From sync stage
        ...
    })
}"#,
            explanation: "**The Full LoRa RX Pipeline**\n\n\
                \
                **Inverse of TX:**\n\
                Each stage reverses its TX counterpart.\n\n\
                \
                **Stage 1: CSS Demodulation**\n\
                FFT-based dechirping extracts symbol values.\n\n\
                \
                **Stage 2: Gray Decode**\n\
                Convert Gray-coded symbols back to binary.\n\n\
                \
                **Stage 3: De-interleave**\n\
                Reverse the bit spreading across symbols.\n\n\
                \
                **Stage 4: Hamming Decode**\n\
                Correct errors using parity bits.\n\
                Can fix 1 error, detect 2 errors per codeword.\n\n\
                \
                **Stage 5: De-whitening**\n\
                XOR with same LFSR sequence recovers original data.\n\n\
                \
                **Quality Metrics:**\n\
                - RSSI: Received signal strength\n\
                - SNR: Signal-to-noise ratio per symbol\n\
                - Errors corrected: FEC effectiveness",
            concepts: &["RX pipeline", "De-interleaving", "Error correction", "De-whitening"],
        },
    ],
};
