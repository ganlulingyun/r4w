//! Block Metadata Registry
//!
//! Provides comprehensive metadata for pipeline blocks including:
//! - Rust implementation locations (file:line)
//! - Mathematical formulas and equations
//! - Detailed technical descriptions
//! - Standards and references
//! - Unit tests that can be executed
//! - Performance characteristics
//!
//! This enables users to:
//! - Click on a block to view its implementing code
//! - See the mathematical basis of DSP operations
//! - Run unit tests on individual blocks or pipelines
//! - Understand standards compliance

use std::collections::HashMap;
use std::sync::OnceLock;

/// Location of Rust implementation
#[derive(Clone, Debug)]
pub struct CodeLocation {
    /// Crate name (e.g., "r4w-core")
    pub crate_name: &'static str,
    /// Module path (e.g., "src/waveform/bpsk.rs")
    pub file_path: &'static str,
    /// Line number in file
    pub line: u32,
    /// Function or struct name
    pub symbol: &'static str,
    /// Brief description of what this code does
    pub description: &'static str,
}

/// Mathematical formula for a block
#[derive(Clone, Debug)]
pub struct BlockFormula {
    /// Name of the formula (e.g., "BPSK Symbol Mapping")
    pub name: &'static str,
    /// Plain text formula (e.g., "s(t) = A * cos(2πfct + φ)")
    pub plaintext: &'static str,
    /// LaTeX formula for rendering (optional)
    pub latex: Option<&'static str>,
    /// Explanation of variables
    pub variables: &'static [(&'static str, &'static str)],
}

/// Unit test information
#[derive(Clone, Debug)]
pub struct BlockTest {
    /// Test name
    pub name: &'static str,
    /// Test module path (e.g., "r4w_core::waveform::bpsk::tests")
    pub module: &'static str,
    /// Brief description
    pub description: &'static str,
    /// Expected runtime in ms (approximate)
    pub expected_runtime_ms: u32,
}

/// Performance characteristics
#[derive(Clone, Debug)]
pub struct PerformanceInfo {
    /// Computational complexity (e.g., "O(n)", "O(n log n)")
    pub complexity: &'static str,
    /// Memory usage description
    pub memory: &'static str,
    /// Whether SIMD optimized
    pub simd_optimized: bool,
    /// Whether GPU accelerable
    pub gpu_accelerable: bool,
}

/// Standards and references
#[derive(Clone, Debug)]
pub struct StandardReference {
    /// Standard name (e.g., "IEEE 802.11")
    pub name: &'static str,
    /// Section or clause reference
    pub section: Option<&'static str>,
    /// URL to specification (if public)
    pub url: Option<&'static str>,
}

/// Complete metadata for a block type
#[derive(Clone, Debug)]
pub struct BlockMetadata {
    /// Block type name (matches BlockType variant name)
    pub block_type: &'static str,
    /// Friendly display name
    pub display_name: &'static str,
    /// Category (e.g., "Modulation", "Recovery")
    pub category: &'static str,
    /// Detailed technical description
    pub description: &'static str,
    /// Primary implementation location
    pub implementation: Option<CodeLocation>,
    /// Additional code locations (helpers, dependencies)
    pub related_code: &'static [CodeLocation],
    /// Mathematical formulas
    pub formulas: &'static [BlockFormula],
    /// Unit tests
    pub tests: &'static [BlockTest],
    /// Performance characteristics
    pub performance: Option<PerformanceInfo>,
    /// Standards references
    pub standards: &'static [StandardReference],
    /// Related block types
    pub related_blocks: &'static [&'static str],
    /// Input signal type (e.g., "bits", "symbols", "IQ samples")
    pub input_type: &'static str,
    /// Output signal type
    pub output_type: &'static str,
    /// Key parameters that affect behavior
    pub key_parameters: &'static [&'static str],
}

// Static metadata for all block types
static BLOCK_METADATA: OnceLock<HashMap<&'static str, BlockMetadata>> = OnceLock::new();

fn init_block_metadata() -> HashMap<&'static str, BlockMetadata> {
    let mut m = HashMap::new();

        // ===== MODULATION BLOCKS =====

        m.insert("PskModulator", BlockMetadata {
            block_type: "PskModulator",
            display_name: "PSK Modulator",
            category: "Modulation",
            description: "Phase Shift Keying modulator. Maps groups of bits to constellation \
                points with different phase angles. BPSK uses 2 phases (0°, 180°), QPSK uses \
                4 phases (45°, 135°, 225°, 315°), and higher orders use more phases evenly \
                distributed around the unit circle.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/psk.rs",
                line: 45,
                symbol: "PskModulator::modulate",
                description: "Core PSK modulation function",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/waveform/mod.rs",
                    line: 1,
                    symbol: "Waveform trait",
                    description: "Base waveform trait",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "PSK Symbol",
                    plaintext: "s_k = exp(j * 2π * k / M) where k ∈ {0, 1, ..., M-1}",
                    latex: Some(r"s_k = e^{j\frac{2\pi k}{M}}"),
                    variables: &[
                        ("M", "Constellation size (2 for BPSK, 4 for QPSK, etc.)"),
                        ("k", "Symbol index from bit mapping"),
                        ("s_k", "Complex constellation point"),
                    ],
                },
                BlockFormula {
                    name: "Gray Coding",
                    plaintext: "gray(n) = n XOR (n >> 1)",
                    latex: Some(r"\text{gray}(n) = n \oplus (n \gg 1)"),
                    variables: &[
                        ("n", "Binary input"),
                        ("gray(n)", "Gray-coded output"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_bpsk_modulation",
                    module: "r4w_core::waveform::psk::tests",
                    description: "Verify BPSK maps bits to ±1",
                    expected_runtime_ms: 10,
                },
                BlockTest {
                    name: "test_qpsk_constellation",
                    module: "r4w_core::waveform::psk::tests",
                    description: "Verify QPSK constellation points",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "Constant (lookup table)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "IEEE 802.11",
                    section: Some("Section 17.3.5.8 - BPSK/QPSK"),
                    url: None,
                },
                StandardReference {
                    name: "DVB-S2",
                    section: Some("ETSI EN 302 307"),
                    url: Some("https://www.etsi.org/deliver/etsi_en/302300_302399/302307/"),
                },
            ],
            related_blocks: &["PskDemodulator", "GrayMapper", "ConstellationMapper"],
            input_type: "bits",
            output_type: "complex symbols",
            key_parameters: &["order (2=BPSK, 4=QPSK, 8=8PSK, ...)"],
        });

        m.insert("QamModulator", BlockMetadata {
            block_type: "QamModulator",
            display_name: "QAM Modulator",
            category: "Modulation",
            description: "Quadrature Amplitude Modulation. Maps bits to a rectangular grid \
                of constellation points varying in both amplitude and phase. Higher orders \
                (64-QAM, 256-QAM) increase spectral efficiency but require better SNR.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/qam.rs",
                line: 52,
                symbol: "QamModulator::modulate",
                description: "Core QAM modulation function",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "QAM Symbol",
                    plaintext: "s = (2i - √M + 1) + j(2q - √M + 1) normalized",
                    latex: Some(r"s = \frac{(2i - \sqrt{M} + 1) + j(2q - \sqrt{M} + 1)}{\sqrt{E_{avg}}}"),
                    variables: &[
                        ("M", "Constellation size (16, 64, 256, ...)"),
                        ("i, q", "Row and column indices"),
                        ("E_avg", "Average symbol energy for normalization"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_16qam_constellation",
                    module: "r4w_core::waveform::qam::tests",
                    description: "Verify 16-QAM constellation geometry",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "Constant (lookup table)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "DVB-S2X",
                    section: Some("ETSI EN 302 307-2"),
                    url: None,
                },
            ],
            related_blocks: &["QamDemodulator", "GrayMapper"],
            input_type: "bits",
            output_type: "complex symbols",
            key_parameters: &["order (16, 64, 256, 1024)"],
        });

        m.insert("FskModulator", BlockMetadata {
            block_type: "FskModulator",
            display_name: "FSK Modulator",
            category: "Modulation",
            description: "Frequency Shift Keying modulator. Represents data by shifting the \
                carrier frequency. Constant envelope makes it power-efficient and robust to \
                non-linear amplifiers. GFSK (Gaussian FSK) uses a Gaussian filter for \
                smoother frequency transitions.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/fsk.rs",
                line: 38,
                symbol: "FskModulator::modulate",
                description: "FSK modulation with phase continuity",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "FSK Signal",
                    plaintext: "s(t) = A * cos(2π(fc ± fd)t + φ)",
                    latex: Some(r"s(t) = A \cos(2\pi (f_c \pm f_d) t + \phi)"),
                    variables: &[
                        ("fc", "Carrier frequency"),
                        ("fd", "Frequency deviation"),
                        ("φ", "Phase (continuous for CPFSK)"),
                    ],
                },
                BlockFormula {
                    name: "Modulation Index",
                    plaintext: "h = 2 * fd / symbol_rate",
                    latex: Some(r"h = \frac{2 f_d}{R_s}"),
                    variables: &[
                        ("h", "Modulation index"),
                        ("fd", "Frequency deviation (Hz)"),
                        ("Rs", "Symbol rate (symbols/s)"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_fsk_frequency_deviation",
                    module: "r4w_core::waveform::fsk::tests",
                    description: "Verify frequency deviation accuracy",
                    expected_runtime_ms: 15,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1) (phase accumulator)",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "Bluetooth BR",
                    section: Some("Core Spec v5.3, Vol 2, Part H"),
                    url: None,
                },
            ],
            related_blocks: &["FskDemodulator", "PulseShaper"],
            input_type: "bits",
            output_type: "complex samples",
            key_parameters: &["deviation_hz", "order (2, 4, 8)"],
        });

        m.insert("CssModulator", BlockMetadata {
            block_type: "CssModulator",
            display_name: "CSS/LoRa Modulator",
            category: "Modulation",
            description: "Chirp Spread Spectrum modulator as used in LoRa. Symbols are \
                represented by cyclically shifted chirps. The spreading factor (SF) \
                determines bits per symbol and processing gain. Higher SF = longer range \
                but slower data rate.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/lora.rs",
                line: 85,
                symbol: "LoRa::modulate_symbol",
                description: "LoRa chirp generation with cyclic shift",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/waveform/lora.rs",
                    line: 42,
                    symbol: "generate_base_chirp",
                    description: "Base upchirp generation",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "LoRa Chirp",
                    plaintext: "s(t) = exp(j * π * (BW/T) * (t + 2*symbol*T/2^SF)²)",
                    latex: Some(r"s(t) = e^{j\pi \frac{BW}{T_s}\left(t + \frac{2 \cdot \text{sym}}{2^{SF}}\right)^2}"),
                    variables: &[
                        ("BW", "Bandwidth (Hz)"),
                        ("SF", "Spreading factor (7-12)"),
                        ("T", "Symbol duration = 2^SF / BW"),
                        ("sym", "Symbol value (0 to 2^SF - 1)"),
                    ],
                },
                BlockFormula {
                    name: "Processing Gain",
                    plaintext: "G_p = 10 * log10(2^SF) dB",
                    latex: Some(r"G_p = 10 \log_{10}(2^{SF}) \text{ dB}"),
                    variables: &[
                        ("Gp", "Processing gain"),
                        ("SF", "Spreading factor"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_lora_chirp_generation",
                    module: "r4w_core::waveform::lora::tests",
                    description: "Verify chirp frequency sweep",
                    expected_runtime_ms: 20,
                },
                BlockTest {
                    name: "test_lora_symbol_roundtrip",
                    module: "r4w_core::waveform::lora::tests",
                    description: "Modulate and demodulate symbols",
                    expected_runtime_ms: 50,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(2^SF) per symbol",
                memory: "O(2^SF) for chirp buffer",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "Semtech LoRa",
                    section: Some("AN1200.22 LoRa Modulation Basics"),
                    url: Some("https://www.semtech.com/products/wireless-rf/lora-connect"),
                },
            ],
            related_blocks: &["GrayMapper", "Interleaver", "Scrambler"],
            input_type: "symbols",
            output_type: "complex samples (chirps)",
            key_parameters: &["sf (spreading factor 5-12)", "bw_hz (125k, 250k, 500k)"],
        });

        // ===== RECOVERY BLOCKS =====

        m.insert("Agc", BlockMetadata {
            block_type: "Agc",
            display_name: "AGC",
            category: "Recovery",
            description: "Automatic Gain Control. Adjusts signal amplitude to maintain \
                consistent power level. Essential for handling varying channel conditions \
                and receiver front-end dynamics. Modes: Fast (quick convergence), Slow \
                (stable, less tracking), Adaptive (hybrid).",
            implementation: None, // AGC not yet implemented as standalone block
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Gain Update",
                    plaintext: "g[n+1] = g[n] + μ * (P_target - P_measured)",
                    latex: Some(r"g_{n+1} = g_n + \mu (P_{target} - P_{measured})"),
                    variables: &[
                        ("g", "Current gain"),
                        ("μ", "Step size / loop bandwidth"),
                        ("P_target", "Desired power level"),
                        ("P_measured", "Measured input power"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_agc_convergence",
                    module: "r4w_core::dsp::agc::tests",
                    description: "Verify AGC converges to target power",
                    expected_runtime_ms: 20,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["TimingRecovery", "CarrierRecovery"],
            input_type: "complex samples",
            output_type: "complex samples (normalized)",
            key_parameters: &["mode (Fast/Slow/Adaptive)", "target_db"],
        });

        m.insert("TimingRecovery", BlockMetadata {
            block_type: "TimingRecovery",
            display_name: "Timing Recovery",
            category: "Recovery",
            description: "Symbol timing recovery. Estimates and corrects sample timing \
                offset to sample at optimal points. Gardner algorithm uses zero crossings, \
                Mueller-Muller uses decision feedback, Early-Late uses correlation.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/sync.rs",
                line: 1,
                symbol: "TimingRecovery",
                description: "Symbol timing recovery with multiple algorithms",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Gardner TED",
                    plaintext: "e[n] = Re{(y[n] - y[n-2]) * conj(y[n-1])}",
                    latex: Some(r"e_n = \Re\{(y_n - y_{n-2}) \cdot y_{n-1}^*\}"),
                    variables: &[
                        ("e", "Timing error"),
                        ("y[n]", "Current sample"),
                        ("y[n-1]", "Mid-sample (strobe)"),
                        ("y[n-2]", "Previous sample"),
                    ],
                },
                BlockFormula {
                    name: "Loop Filter",
                    plaintext: "τ[n+1] = τ[n] + K1*e[n] + K2*∫e",
                    latex: Some(r"\tau_{n+1} = \tau_n + K_1 e_n + K_2 \sum e"),
                    variables: &[
                        ("τ", "Timing offset"),
                        ("K1", "Proportional gain"),
                        ("K2", "Integral gain"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_timing_recovery_offset",
                    module: "r4w_core::dsp::timing::tests",
                    description: "Recover from known timing offset",
                    expected_runtime_ms: 50,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(interpolator taps)",
                simd_optimized: true,
                gpu_accelerable: false,
            }),
            standards: &[],
            related_blocks: &["Agc", "CarrierRecovery", "PulseShaper"],
            input_type: "complex samples (oversampled)",
            output_type: "complex samples (1 samp/sym)",
            key_parameters: &["algorithm (Gardner/EarlyLate/MuellerMuller)", "loop_bw"],
        });

        m.insert("CarrierRecovery", BlockMetadata {
            block_type: "CarrierRecovery",
            display_name: "Carrier Recovery",
            category: "Recovery",
            description: "Carrier frequency and phase recovery. Compensates for frequency \
                offset and phase rotation. Costas loop works for BPSK/QPSK, decision-directed \
                for higher-order modulations, pilot-aided uses known reference symbols.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/sync.rs",
                line: 1,
                symbol: "CarrierRecovery",
                description: "Carrier frequency/phase tracking",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Costas Loop (QPSK)",
                    plaintext: "e[n] = Im{y[n]⁴} or sign(I)*Q - sign(Q)*I",
                    latex: Some(r"e_n = \Im\{y_n^4\}"),
                    variables: &[
                        ("e", "Phase error"),
                        ("y[n]", "Input symbol"),
                        ("I, Q", "In-phase and quadrature components"),
                    ],
                },
                BlockFormula {
                    name: "Phase Correction",
                    plaintext: "y_out[n] = y[n] * exp(-j*θ[n])",
                    latex: Some(r"y_{out,n} = y_n \cdot e^{-j\theta_n}"),
                    variables: &[
                        ("θ[n]", "Estimated phase offset"),
                        ("y[n]", "Input sample"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_costas_loop_lock",
                    module: "r4w_core::dsp::carrier::tests",
                    description: "Verify lock acquisition",
                    expected_runtime_ms: 100,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1)",
                simd_optimized: true,
                gpu_accelerable: false,
            }),
            standards: &[],
            related_blocks: &["Agc", "TimingRecovery", "PskDemodulator"],
            input_type: "complex symbols",
            output_type: "complex symbols (phase-corrected)",
            key_parameters: &["algorithm (CostasLoop/DecisionDirected/PilotAided)", "loop_bw"],
        });

        // ===== DEMODULATION BLOCKS =====

        m.insert("PskDemodulator", BlockMetadata {
            block_type: "PskDemodulator",
            display_name: "PSK Demodulator",
            category: "Recovery",
            description: "Phase Shift Keying demodulator. Maps received symbols back to bits \
                using minimum distance decision. Outputs hard or soft decisions.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/psk.rs",
                line: 120,
                symbol: "PskModulator::demodulate",
                description: "PSK symbol slicing",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Hard Decision",
                    plaintext: "k = argmin_i |r - s_i|²",
                    latex: Some(r"\hat{k} = \arg\min_i |r - s_i|^2"),
                    variables: &[
                        ("r", "Received symbol"),
                        ("s_i", "Reference constellation point"),
                        ("k", "Decoded symbol index"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_psk_demodulation",
                    module: "r4w_core::waveform::psk::tests",
                    description: "Verify correct bit recovery",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "Constant",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["PskModulator", "CarrierRecovery"],
            input_type: "complex symbols",
            output_type: "bits",
            key_parameters: &["order"],
        });

        // ===== FILTERING BLOCKS =====

        m.insert("PulseShaper", BlockMetadata {
            block_type: "PulseShaper",
            display_name: "Pulse Shaper",
            category: "Filtering",
            description: "Shapes transmitted pulses for spectrum efficiency and ISI control. \
                Root Raised Cosine (RRC) is common for matched filtering. Gaussian for FSK. \
                Rolloff factor controls bandwidth vs. ISI tradeoff.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/filters/pulse_shaping.rs",
                line: 48,
                symbol: "RrcFilter::new",
                description: "RRC filter coefficient generation",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/filters/fir.rs",
                    line: 52,
                    symbol: "FirFilter",
                    description: "FIR filter implementation",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "RRC Impulse Response",
                    plaintext: "h(t) = [sin(πt/T(1-α)) + 4αt/T·cos(πt/T(1+α))] / [πt/T(1-(4αt/T)²)]",
                    latex: Some(r"h(t) = \frac{\sin(\pi t/T(1-\alpha)) + 4\alpha t/T \cos(\pi t/T(1+\alpha))}{\pi t/T (1-(4\alpha t/T)^2)}"),
                    variables: &[
                        ("T", "Symbol period"),
                        ("α", "Rolloff factor (0 to 1)"),
                        ("h(t)", "Impulse response"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_rrc_filter",
                    module: "r4w_core::filters::pulse::tests",
                    description: "Verify ISI-free sampling points",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n*taps)",
                memory: "O(taps)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["MatchedFilter", "Upsampler", "TimingRecovery"],
            input_type: "complex symbols/samples",
            output_type: "complex samples",
            key_parameters: &["shape (RRC/Gaussian/Rect)", "rolloff", "span_symbols"],
        });

        // ===== CHANNEL BLOCKS =====

        m.insert("AwgnChannel", BlockMetadata {
            block_type: "AwgnChannel",
            display_name: "AWGN Channel",
            category: "Impairments",
            description: "Additive White Gaussian Noise channel model. Adds thermal noise \
                with specified SNR. Fundamental model for analyzing BER performance.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-sim",
                file_path: "src/channel.rs",
                line: 158,
                symbol: "Channel::apply",
                description: "Channel effects application including AWGN",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "AWGN Addition",
                    plaintext: "r(t) = s(t) + n(t) where n ~ N(0, σ²)",
                    latex: Some(r"r(t) = s(t) + n(t), \quad n \sim \mathcal{N}(0, \sigma^2)"),
                    variables: &[
                        ("s(t)", "Transmitted signal"),
                        ("n(t)", "Noise (complex Gaussian)"),
                        ("σ²", "Noise variance = N0/2"),
                    ],
                },
                BlockFormula {
                    name: "SNR to Noise Power",
                    plaintext: "σ² = P_signal / 10^(SNR_dB/10)",
                    latex: Some(r"\sigma^2 = \frac{P_s}{10^{SNR_{dB}/10}}"),
                    variables: &[
                        ("P_s", "Signal power"),
                        ("SNR_dB", "Signal-to-noise ratio in dB"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_awgn_power",
                    module: "r4w_sim::channel::tests",
                    description: "Verify noise power matches SNR",
                    expected_runtime_ms: 20,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["FadingChannel", "FrequencyOffset"],
            input_type: "complex samples",
            output_type: "complex samples",
            key_parameters: &["snr_db"],
        });

        m.insert("FadingChannel", BlockMetadata {
            block_type: "FadingChannel",
            display_name: "Fading Channel",
            category: "Impairments",
            description: "Multipath fading channel with Doppler spread. Rayleigh for \
                NLOS conditions, Rician for LOS with K-factor. Simulates mobile channel \
                dynamics.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-sim",
                file_path: "src/channel.rs",
                line: 158,
                symbol: "Channel::apply",
                description: "Rayleigh/Rician fading implementation",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-sim",
                    file_path: "src/doppler.rs",
                    line: 45,
                    symbol: "JakesDoppler",
                    description: "Jake's Doppler spectrum model",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "Jake's Doppler Spectrum",
                    plaintext: "S(f) = 1 / (π*fd*√(1 - (f/fd)²)) for |f| < fd",
                    latex: Some(r"S(f) = \frac{1}{\pi f_d \sqrt{1 - (f/f_d)^2}}"),
                    variables: &[
                        ("fd", "Maximum Doppler shift"),
                        ("f", "Frequency offset"),
                        ("S(f)", "Power spectral density"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_rayleigh_statistics",
                    module: "r4w_sim::channel::tests",
                    description: "Verify Rayleigh amplitude distribution",
                    expected_runtime_ms: 100,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n * taps)",
                memory: "O(taps)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "3GPP TR 38.901",
                    section: Some("Channel models for frequencies from 0.5 to 100 GHz"),
                    url: Some("https://www.3gpp.org/ftp/Specs/archive/38_series/38.901/"),
                },
            ],
            related_blocks: &["AwgnChannel", "Equalizer"],
            input_type: "complex samples",
            output_type: "complex samples",
            key_parameters: &["model (Rayleigh/Rician)", "doppler_hz"],
        });

        // ===== SOURCE BLOCKS =====

        m.insert("BitSource", BlockMetadata {
            block_type: "BitSource",
            display_name: "Bit Source",
            category: "Source",
            description: "Generates bit stream for transmission. Supports random data, \
                fixed patterns (0101..., 1111...), or file input.",
            implementation: None,
            related_code: &[],
            formulas: &[],
            tests: &[],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1)",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[],
            related_blocks: &["Scrambler", "FecEncoder"],
            input_type: "none",
            output_type: "bits",
            key_parameters: &["pattern (random/zeros/ones/alternating)"],
        });

        // ===== CODING BLOCKS =====

        m.insert("Scrambler", BlockMetadata {
            block_type: "Scrambler",
            display_name: "Scrambler/Whitener",
            category: "Coding",
            description: "Randomizes bit stream to ensure spectral flatness and aid \
                synchronization. Uses LFSR (Linear Feedback Shift Register) with \
                configurable polynomial.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/scrambler.rs",
                line: 1,
                symbol: "Scrambler",
                description: "LFSR-based additive/multiplicative scrambler",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Additive Scrambler",
                    plaintext: "out[n] = data[n] XOR LFSR_output[n]",
                    latex: Some(r"\text{out}_n = \text{data}_n \oplus \text{LFSR}_n"),
                    variables: &[
                        ("data[n]", "Input bit"),
                        ("LFSR_output", "Pseudo-random sequence from LFSR"),
                        ("mask", "Feedback polynomial taps"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_additive_roundtrip",
                    module: "r4w_core::scrambler::tests",
                    description: "Verify scramble/descramble roundtrip",
                    expected_runtime_ms: 10,
                },
                BlockTest {
                    name: "test_byte_roundtrip",
                    module: "r4w_core::scrambler::tests",
                    description: "Byte-level scramble/descramble",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1)",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[],
            related_blocks: &["FecEncoder", "Interleaver"],
            input_type: "bits",
            output_type: "bits (whitened)",
            key_parameters: &["polynomial", "seed"],
        });

        m.insert("FecEncoder", BlockMetadata {
            block_type: "FecEncoder",
            display_name: "FEC Encoder",
            category: "Coding",
            description: "Forward Error Correction encoder. Adds redundancy for error \
                detection and correction. Convolutional codes for streaming, block codes \
                (Hamming, Reed-Solomon) for fixed-length data.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/coding.rs",
                line: 126,
                symbol: "HammingCode",
                description: "Hamming FEC encoding",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Code Rate",
                    plaintext: "R = k/n where k=input bits, n=output bits",
                    latex: Some(r"R = \frac{k}{n}"),
                    variables: &[
                        ("R", "Code rate"),
                        ("k", "Information bits per block"),
                        ("n", "Coded bits per block"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_conv_encode_decode",
                    module: "r4w_core::coding::fec::tests",
                    description: "Encode and decode with Viterbi",
                    expected_runtime_ms: 50,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n) for convolutional, O(n²) for RS",
                memory: "O(constraint_length)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["Interleaver", "Scrambler"],
            input_type: "bits",
            output_type: "bits (coded)",
            key_parameters: &["code_type (Convolutional/Hamming/RS)", "rate"],
        });

        m.insert("Interleaver", BlockMetadata {
            block_type: "Interleaver",
            display_name: "Interleaver",
            category: "Coding",
            description: "Spreads burst errors across multiple codewords to improve FEC \
                effectiveness. Block interleaver writes row-by-row, reads column-by-column.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/coding.rs",
                line: 244,
                symbol: "Interleaver::interleave",
                description: "LoRa-style diagonal interleaving",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Block Interleaver Index",
                    plaintext: "out_idx = (in_idx % rows) * cols + (in_idx / rows)",
                    latex: Some(r"\text{out} = (\text{in} \mod R) \cdot C + \lfloor\text{in}/R\rfloor"),
                    variables: &[
                        ("rows", "Number of rows"),
                        ("cols", "Number of columns"),
                        ("in_idx", "Input index"),
                        ("out_idx", "Output index"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_interleave_deinterleave",
                    module: "r4w_core::coding::interleaver::tests",
                    description: "Verify interleaver round-trip",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(rows * cols)",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["FecEncoder", "GrayMapper"],
            input_type: "bits/symbols",
            output_type: "bits/symbols (reordered)",
            key_parameters: &["rows", "cols"],
        });

        m.insert("GrayMapper", BlockMetadata {
            block_type: "GrayMapper",
            display_name: "Gray Mapper",
            category: "Mapping",
            description: "Maps bits to symbols using Gray coding where adjacent symbols \
                differ by only one bit. Minimizes bit errors when symbol errors occur.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/coding.rs",
                line: 52,
                symbol: "GrayCode",
                description: "Gray code encoding/decoding",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Gray Encoding",
                    plaintext: "g = n XOR (n >> 1)",
                    latex: Some(r"g = n \oplus (n \gg 1)"),
                    variables: &[
                        ("n", "Binary input"),
                        ("g", "Gray-coded output"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_gray_code",
                    module: "r4w_core::coding::gray::tests",
                    description: "Verify Gray code properties",
                    expected_runtime_ms: 5,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n)",
                memory: "O(1)",
                simd_optimized: true,
                gpu_accelerable: true,
            }),
            standards: &[],
            related_blocks: &["PskModulator", "QamModulator", "CssModulator"],
            input_type: "bit groups",
            output_type: "symbol indices",
            key_parameters: &["bits_per_symbol"],
        });

        // ===== GNSS BLOCKS =====

        m.insert("GnssScenarioSource", BlockMetadata {
            block_type: "GnssScenarioSource",
            display_name: "GNSS Scenario Source",
            category: "GNSS",
            description: "Multi-satellite GNSS IQ scenario generator. Produces composite baseband \
                IQ samples from multiple GNSS satellites with realistic channel effects including \
                Keplerian orbital geometry, ionospheric/tropospheric delays, multipath, antenna \
                patterns, free-space path loss, and Doppler shifts. Supports GPS L1 C/A, GPS L5, \
                Galileo E1, and GLONASS L1OF signals.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/gnss/scenario.rs",
                line: 37,
                symbol: "GnssScenario",
                description: "GNSS scenario IQ generator",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/waveform/gnss/scenario_config.rs",
                    line: 430,
                    symbol: "GnssScenarioConfig",
                    description: "Scenario configuration struct",
                },
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/waveform/gnss/environment/mod.rs",
                    line: 1,
                    symbol: "environment",
                    description: "Ionosphere, troposphere, multipath models",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "Composite IQ Signal",
                    plaintext: "y[n] = sum_k( A_k * PRN_k(n) * exp(j*2*pi*(f_IF + f_doppler_k)*n/fs) )",
                    latex: Some(r"y[n] = \sum_k A_k \cdot \text{PRN}_k(n) \cdot e^{j2\pi(f_{IF} + f_{d,k})n/f_s}"),
                    variables: &[
                        ("A_k", "Signal amplitude for satellite k (from FSPL, antenna gain)"),
                        ("PRN_k(n)", "PRN code chips for satellite k"),
                        ("f_IF", "Intermediate frequency (0 for baseband)"),
                        ("f_doppler_k", "Doppler shift for satellite k"),
                        ("f_s", "Sample rate in Hz"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_gnss_scenario_opensky",
                    module: "r4w_core::waveform::gnss::scenario::tests",
                    description: "Verify open sky scenario generates valid IQ",
                    expected_runtime_ms: 500,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n * k) where k = number of satellites",
                memory: "O(n) for output buffer",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "IS-GPS-200",
                    section: Some("GPS L1 C/A signal specification"),
                    url: Some("https://www.gps.gov/technical/icwg/IS-GPS-200N.pdf"),
                },
                StandardReference {
                    name: "Galileo OS SIS ICD v2.1",
                    section: Some("Galileo E1 signal specification"),
                    url: Some("https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.1.pdf"),
                },
            ],
            related_blocks: &["GnssAcquisition"],
            input_type: "(none - source block)",
            output_type: "IQ samples",
            key_parameters: &["preset", "sample_rate_hz", "duration_s", "lat/lon/alt", "noise_figure_db"],
        });

        m.insert("GnssAcquisition", BlockMetadata {
            block_type: "GnssAcquisition",
            display_name: "GNSS Acquisition",
            category: "GNSS",
            description: "Parallel Code Phase Search (PCPS) acquisition engine. Performs FFT-based \
                circular correlation across a 2D search grid of code phase and Doppler frequency \
                to detect and estimate the parameters of a GNSS satellite signal. Reports detection \
                status, code phase, Doppler estimate, and peak metric.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/waveform/gnss/acquisition.rs",
                line: 38,
                symbol: "PcpsAcquisition",
                description: "PCPS acquisition engine",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/waveform/gnss/types.rs",
                    line: 1,
                    symbol: "AcquisitionResult",
                    description: "Acquisition result struct",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "PCPS Correlation",
                    plaintext: "R(tau, fd) = |IFFT{ FFT{x * exp(-j*2*pi*fd*t)} * conj(FFT{c(tau)}) }|",
                    latex: Some(r"R(\tau, f_d) = \left|\text{IFFT}\left\{\text{FFT}\left\{x \cdot e^{-j2\pi f_d t}\right\} \cdot \text{FFT}\{c(\tau)\}^*\right\}\right|"),
                    variables: &[
                        ("x", "Input IQ samples"),
                        ("c(tau)", "Local replica PRN code at delay tau"),
                        ("f_d", "Doppler frequency hypothesis"),
                        ("R", "Correlation power (2D surface)"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_pcps_acquisition",
                    module: "r4w_core::waveform::gnss::acquisition::tests",
                    description: "Verify PCPS acquisition detects GPS signal",
                    expected_runtime_ms: 200,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(N_d * N * log N) where N_d = Doppler bins, N = code length",
                memory: "O(N) for FFT buffers",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "Kaplan & Hegarty",
                    section: Some("Ch. 8 - GNSS Receivers"),
                    url: None,
                },
            ],
            related_blocks: &["GnssScenarioSource"],
            input_type: "IQ samples",
            output_type: "Real (acquisition results)",
            key_parameters: &["signal", "prn", "doppler_max_hz", "doppler_step_hz", "threshold"],
        });

        // ==================== Recovery Blocks ====================

        m.insert("Agc", BlockMetadata {
            block_type: "Agc",
            display_name: "Automatic Gain Control",
            category: "Recovery",
            description: "Normalizes signal amplitude to a target level. Essential for any receiver \
                chain where input power varies due to distance, fading, or interference. Three \
                modes: Fast (AGC3 - linear acquisition then log tracking), Adaptive (AGC2 - \
                separate attack/decay rates), Slow (AGC - single exponential rate).",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/agc.rs",
                line: 1,
                symbol: "Agc",
                description: "AGC implementations (Agc, Agc2, Agc3)",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Gain Update (AGC)",
                    plaintext: "gain += rate * (target - |output|)",
                    latex: Some(r"g[n+1] = g[n] + \mu \cdot (A_{target} - |y[n]|)"),
                    variables: &[
                        ("g[n]", "Current gain"),
                        ("mu", "Update rate"),
                        ("A_target", "Target output amplitude"),
                        ("y[n]", "Output sample = g[n] * x[n]"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_agc_basic_convergence",
                    module: "r4w_core::agc::tests",
                    description: "Verify AGC converges to target amplitude",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(1) per sample",
                memory: "O(1) state",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[],
            related_blocks: &["CarrierRecovery", "TimingRecovery"],
            input_type: "IQ samples",
            output_type: "IQ samples (amplitude-normalized)",
            key_parameters: &["mode (Fast/Adaptive/Slow)", "target_db"],
        });

        m.insert("CarrierRecovery", BlockMetadata {
            block_type: "CarrierRecovery",
            display_name: "Carrier Recovery (Costas Loop)",
            category: "Recovery",
            description: "Costas loop for carrier frequency and phase recovery in PSK signals. \
                Uses a modulation-order-aware phase error detector with a 2nd-order \
                proportional-integral loop filter. Supports BPSK, QPSK, and 8PSK modes.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/carrier_recovery.rs",
                line: 1,
                symbol: "CostasLoop",
                description: "Costas loop carrier recovery",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "BPSK Phase Error",
                    plaintext: "e[n] = I[n] * Q[n]",
                    latex: Some(r"e[n] = \text{Re}\{y[n]\} \cdot \text{Im}\{y[n]\}"),
                    variables: &[
                        ("e[n]", "Phase error signal"),
                        ("I[n]", "In-phase component of derotated sample"),
                        ("Q[n]", "Quadrature component of derotated sample"),
                    ],
                },
                BlockFormula {
                    name: "Loop Filter Update",
                    plaintext: "freq += beta * error; phase += freq + alpha * error",
                    latex: Some(r"\omega[n+1] = \omega[n] + \beta \cdot e[n]; \quad \phi[n+1] = \phi[n] + \omega[n] + \alpha \cdot e[n]"),
                    variables: &[
                        ("omega", "Frequency estimate (rad/sample)"),
                        ("phi", "Phase estimate (rad)"),
                        ("alpha", "Proportional gain"),
                        ("beta", "Integral gain"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_bpsk_frequency_offset_tracking",
                    module: "r4w_core::carrier_recovery::tests",
                    description: "Verify Costas loop tracks frequency offset",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(1) per sample",
                memory: "O(1) state (phase, freq, gains)",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[
                StandardReference {
                    name: "Proakis & Salehi",
                    section: Some("Digital Communications, Ch. 6 - Carrier Recovery"),
                    url: None,
                },
            ],
            related_blocks: &["Agc", "TimingRecovery"],
            input_type: "IQ samples",
            output_type: "IQ samples (phase-corrected)",
            key_parameters: &["algorithm (CostasLoop/PilotAided)", "loop_bw"],
        });

        m.insert("TimingRecovery", BlockMetadata {
            block_type: "TimingRecovery",
            display_name: "Symbol Timing Recovery",
            category: "Recovery",
            description: "Mueller & Muller symbol timing recovery. Takes oversampled IQ input and \
                outputs symbol-rate samples at the optimal sampling instants. Uses a timing error \
                detector with proportional-integral loop filter for clock offset tracking.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/clock_recovery.rs",
                line: 1,
                symbol: "MuellerMuller",
                description: "Mueller & Muller clock recovery",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "M&M Timing Error",
                    plaintext: "e[n] = Re{ conj(x[n]) * d[n-1] - conj(x[n-1]) * d[n] }",
                    latex: Some(r"e[n] = \text{Re}\{ x^*[n] \cdot d[n-1] - x^*[n-1] \cdot d[n] \}"),
                    variables: &[
                        ("e[n]", "Timing error signal"),
                        ("x[n]", "Interpolated sample at estimated optimal point"),
                        ("d[n]", "Decision (nearest constellation point)"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_mm_basic_recovery",
                    module: "r4w_core::clock_recovery::tests",
                    description: "Verify M&M recovers symbols from oversampled BPSK",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(1) per input sample",
                memory: "O(1) state",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[
                StandardReference {
                    name: "Mueller & Muller (1976)",
                    section: Some("Timing Recovery in Digital Synchronous Data Receivers"),
                    url: None,
                },
            ],
            related_blocks: &["Agc", "CarrierRecovery"],
            input_type: "IQ samples (oversampled)",
            output_type: "IQ samples (symbol-rate)",
            key_parameters: &["algorithm", "loop_bw"],
        });

        m.insert("FecEncoder", BlockMetadata {
            block_type: "FecEncoder",
            display_name: "Convolutional Encoder",
            category: "Coding",
            description: "Convolutional encoder with configurable constraint length and generator \
                polynomials. Appends tail bits to terminate the trellis. Supports NASA K=7 rate 1/2 \
                (CCSDS standard), GSM K=5 rate 1/2, and rate 1/3 K=9 codes.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/fec/convolutional.rs",
                line: 1,
                symbol: "ConvolutionalEncoder",
                description: "Convolutional FEC encoder",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Output Bit Computation",
                    plaintext: "y_i[n] = parity( state AND g_i )",
                    latex: Some(r"y_i[n] = \bigoplus_{k=0}^{K-1} s_k \cdot g_{i,k}"),
                    variables: &[
                        ("y_i[n]", "i-th output bit for input n"),
                        ("state", "K-bit shift register"),
                        ("g_i", "i-th generator polynomial"),
                        ("K", "Constraint length"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_nasa_k7_encode_decode",
                    module: "r4w_core::fec::convolutional::tests",
                    description: "Verify NASA K=7 rate 1/2 roundtrip encode/decode",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n) encoding, O(n * 2^K) Viterbi decoding",
                memory: "O(2^K * n) for traceback",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "CCSDS 131.0-B-3",
                    section: Some("TM Synchronization and Channel Coding"),
                    url: None,
                },
            ],
            related_blocks: &["CrcGenerator", "Interleaver"],
            input_type: "Bits",
            output_type: "Bits (coded, rate 1/n)",
            key_parameters: &["code_type", "rate"],
        });

        m.insert("CrcGenerator", BlockMetadata {
            block_type: "CrcGenerator",
            display_name: "CRC Generator",
            category: "Coding",
            description: "Appends a CRC (Cyclic Redundancy Check) to the input bit stream for \
                error detection. Supports CRC-8, CRC-16 CCITT, CRC-16 IBM, and CRC-32. \
                Uses table-based lookup for efficient computation.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/crc.rs",
                line: 1,
                symbol: "Crc32",
                description: "CRC computation engine",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "CRC Division",
                    plaintext: "CRC = data(x) mod g(x) over GF(2)",
                    latex: Some(r"\text{CRC} = M(x) \cdot x^r \mod G(x)"),
                    variables: &[
                        ("M(x)", "Message polynomial"),
                        ("G(x)", "Generator polynomial"),
                        ("r", "CRC width in bits"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_crc32_known_values",
                    module: "r4w_core::crc::tests",
                    description: "Verify CRC-32 against known test vectors",
                    expected_runtime_ms: 1,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(n) with table lookup",
                memory: "O(256) for CRC table",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[
                StandardReference {
                    name: "ISO 3309",
                    section: Some("CRC-32 for HDLC"),
                    url: None,
                },
            ],
            related_blocks: &["FecEncoder", "Scrambler"],
            input_type: "Bits",
            output_type: "Bits (with CRC appended)",
            key_parameters: &["crc_type (CRC-8/CRC-16/CRC-32)"],
        });

        m.insert("Equalizer", BlockMetadata {
            block_type: "Equalizer",
            display_name: "Adaptive Equalizer",
            category: "Recovery",
            description: "Compensates for inter-symbol interference (ISI), multipath distortion, \
                and channel fading using adaptive filtering. Supports LMS (trained), CMA (blind for PSK), \
                and Decision-Directed (hybrid) algorithms.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/equalizer.rs",
                line: 1,
                symbol: "CmaEqualizer",
                description: "Adaptive equalization algorithms (LMS, CMA, DD)",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "LMS Weight Update",
                    plaintext: "w[n+1] = w[n] + mu * conj(e[n]) * x[n], e = d - y",
                    latex: Some(r"\mathbf{w}[n+1] = \mathbf{w}[n] + \mu \cdot e^*[n] \cdot \mathbf{x}[n]"),
                    variables: &[
                        ("w", "Filter tap weights"),
                        ("mu", "Step size (convergence rate)"),
                        ("e", "Error signal (reference - output)"),
                        ("x", "Input vector (delay line)"),
                    ],
                },
                BlockFormula {
                    name: "CMA Cost Function",
                    plaintext: "J = E[(|y|^2 - R)^2], R = target modulus",
                    latex: Some(r"J_\text{CMA} = E\left[\left(|y[n]|^2 - R\right)^2\right]"),
                    variables: &[
                        ("J", "Cost function to minimize"),
                        ("y", "Equalizer output"),
                        ("R", "Target modulus (1.0 for unit-amplitude PSK)"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_lms_simple_isi_channel",
                    module: "r4w_core::equalizer::tests",
                    description: "LMS equalizer corrects ISI from h=[1, 0.5] channel",
                    expected_runtime_ms: 1,
                },
                BlockTest {
                    name: "test_cma_unit_modulus",
                    module: "r4w_core::equalizer::tests",
                    description: "CMA blind equalizer converges to unit modulus",
                    expected_runtime_ms: 1,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(N) per sample, N = number of taps",
                memory: "O(N) for tap weights and delay line",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "Haykin, Adaptive Filter Theory",
                    section: Some("Ch. 9: LMS Algorithm, Ch. 16: Blind Equalization"),
                    url: None,
                },
            ],
            related_blocks: &["CarrierRecovery", "TimingRecovery", "MatchedFilter"],
            input_type: "IQ",
            output_type: "IQ (equalized)",
            key_parameters: &["eq_type (LMS/CMA/DD)", "taps", "mu (step size)"],
        });

        m.insert("OfdmModulator", BlockMetadata {
            block_type: "OfdmModulator",
            display_name: "OFDM Modulator",
            category: "Modulation",
            description: "Orthogonal Frequency Division Multiplexing modulator. Maps data symbols \
                to subcarriers via IFFT, adds cyclic prefix and pilot tones. Used in WiFi, \
                LTE, DVB-T, and DAB.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/ofdm.rs",
                line: 120,
                symbol: "OfdmModulator",
                description: "OFDM IFFT-based modulator with CP insertion",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "OFDM Symbol",
                    plaintext: "x[n] = (1/sqrt(N)) * sum_k(X[k] * exp(j*2*pi*k*n/N)), n = 0..N-1",
                    latex: Some(r"x[n] = \frac{1}{\sqrt{N}} \sum_{k=0}^{N-1} X[k] e^{j2\pi kn/N}"),
                    variables: &[
                        ("X[k]", "Frequency-domain subcarrier values"),
                        ("N", "FFT size (number of subcarriers)"),
                        ("x[n]", "Time-domain OFDM symbol samples"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_ofdm_wifi_roundtrip",
                    module: "r4w_core::ofdm::tests",
                    description: "WiFi-like OFDM modulate/demodulate roundtrip",
                    expected_runtime_ms: 50,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(N^2) naive DFT, O(N log N) with FFT",
                memory: "O(N) for FFT buffer",
                simd_optimized: false,
                gpu_accelerable: true,
            }),
            standards: &[
                StandardReference {
                    name: "IEEE 802.11a",
                    section: Some("Section 17: OFDM PHY"),
                    url: None,
                },
                StandardReference {
                    name: "ETSI EN 300 744",
                    section: Some("DVB-T OFDM system"),
                    url: None,
                },
            ],
            related_blocks: &["PskModulator", "QamModulator", "FirFilter"],
            input_type: "IQ (subcarrier symbols)",
            output_type: "IQ (time-domain OFDM)",
            key_parameters: &["fft_size", "cp_len", "data_carriers"],
        });

        m.insert("DifferentialEncoder", BlockMetadata {
            block_type: "DifferentialEncoder",
            display_name: "Differential Encoder",
            category: "Mapping",
            description: "Encodes information in phase transitions rather than absolute phase. \
                Resolves phase ambiguity in PSK systems without needing absolute phase reference.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/differential.rs",
                line: 1,
                symbol: "DiffEncoder",
                description: "Differential encoder/decoder for DBPSK/DQPSK/D8PSK",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Differential Encoding",
                    plaintext: "y[n] = (x[n] + y[n-1]) mod M",
                    latex: Some(r"y[n] = (x[n] + y[n-1]) \bmod M"),
                    variables: &[
                        ("x[n]", "Input symbol (0 to M-1)"),
                        ("y[n]", "Encoded output symbol"),
                        ("M", "Constellation size (modulus)"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_dqpsk_roundtrip",
                    module: "r4w_core::differential::tests",
                    description: "DQPSK encode/decode roundtrip",
                    expected_runtime_ms: 10,
                },
                BlockTest {
                    name: "test_phase_ambiguity_resolution",
                    module: "r4w_core::differential::tests",
                    description: "Phase ambiguity resolution after rotation",
                    expected_runtime_ms: 10,
                },
            ],
            performance: Some(PerformanceInfo {
                complexity: "O(1) per symbol",
                memory: "O(1)",
                simd_optimized: false,
                gpu_accelerable: false,
            }),
            standards: &[],
            related_blocks: &["PskModulator", "ConstellationMapper", "CarrierRecovery"],
            input_type: "Symbols or Bits",
            output_type: "Symbols or Bits (differentially encoded)",
            key_parameters: &["modulus"],
        });

        // ===== NEW DSP BLOCKS (Batch 7-10) =====

        m.insert("NoiseSource", BlockMetadata {
            block_type: "NoiseSource",
            display_name: "Noise Source",
            category: "Source",
            description: "Generates colored noise signals (white, pink, brown, blue, violet). \
                Uses Voss-McCartney algorithm for pink noise and integration/differentiation \
                for other colors. Useful for testing, SNR calibration, and dithering.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/noise.rs",
                line: 77,
                symbol: "NoiseGenerator::new",
                description: "Colored noise generator with configurable spectral shape",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Power Spectral Density",
                    plaintext: "S(f) ∝ 1/f^α where α=0 (white), α=1 (pink), α=2 (brown)",
                    latex: Some(r"S(f) \propto \frac{1}{f^\alpha}"),
                    variables: &[
                        ("α", "Spectral slope: 0=white, 1=pink, 2=brown, -1=blue, -2=violet"),
                        ("f", "Frequency"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_white_noise_stats", module: "r4w_core::noise::tests", description: "White noise mean/variance", expected_runtime_ms: 5 },
                BlockTest { name: "test_pink_noise_spectrum", module: "r4w_core::noise::tests", description: "Pink noise 1/f slope", expected_runtime_ms: 10 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) state per generator", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["AwgnChannel", "BitSource"],
            input_type: "None (source)",
            output_type: "IQ samples",
            key_parameters: &["color", "amplitude"],
        });

        m.insert("DcBlocker", BlockMetadata {
            block_type: "DcBlocker",
            display_name: "DC Blocker",
            category: "Filtering",
            description: "High-pass IIR filter that removes DC offset from signals. \
                Uses a simple first-order IIR: y[n] = x[n] - x[n-1] + α*y[n-1]. \
                Higher alpha values provide narrower notch at DC but slower convergence.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/pll.rs",
                line: 228,
                symbol: "DcBlocker",
                description: "First-order IIR DC removal filter",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "DC Blocker Transfer",
                    plaintext: "y[n] = x[n] - x[n-1] + α·y[n-1]",
                    latex: Some(r"H(z) = \frac{1 - z^{-1}}{1 - \alpha z^{-1}}"),
                    variables: &[
                        ("α", "Pole location, typically 0.99-0.999"),
                        ("x[n]", "Input sample"),
                        ("y[n]", "Output sample (DC removed)"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_dc_blocker_removes_dc", module: "r4w_core::pll::tests", description: "DC removal convergence", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) (2 state variables)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["FirFilter", "IirFilter"],
            input_type: "IQ samples",
            output_type: "IQ samples",
            key_parameters: &["alpha"],
        });

        m.insert("CicDecimator", BlockMetadata {
            block_type: "CicDecimator",
            display_name: "CIC Decimator",
            category: "Rate Conversion",
            description: "Cascaded Integrator-Comb decimation filter. Multiplier-free architecture \
                using only additions and subtractions. Ideal for high decimation ratios in \
                front-end processing (e.g., SDR receivers). Passband droop can be compensated \
                with a short FIR filter.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/filters/cic.rs",
                line: 1,
                symbol: "CicDecimator",
                description: "CIC decimation filter with integer arithmetic",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "CIC Transfer Function",
                    plaintext: "H(z) = [(1 - z^(-RM)) / (1 - z^(-1))]^N",
                    latex: Some(r"H(z) = \left[\frac{1 - z^{-RM}}{1 - z^{-1}}\right]^N"),
                    variables: &[
                        ("R", "Decimation rate"),
                        ("M", "Differential delay (typically 1)"),
                        ("N", "Number of stages"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_cic_decimation_basic", module: "r4w_core::filters::cic::tests", description: "Basic CIC decimation", expected_runtime_ms: 5 },
                BlockTest { name: "test_cic_passband_droop", module: "r4w_core::filters::cic::tests", description: "Passband droop calculation", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n·N)", memory: "O(N) integrator state", simd_optimized: false, gpu_accelerable: true }),
            standards: &[
                StandardReference { name: "Hogenauer, 1981", section: Some("An economical class of digital filters for decimation and interpolation"), url: None },
            ],
            related_blocks: &["Downsampler", "PolyphaseResampler"],
            input_type: "IQ samples",
            output_type: "IQ samples (decimated)",
            key_parameters: &["rate", "stages"],
        });

        m.insert("BurstDetector", BlockMetadata {
            block_type: "BurstDetector",
            display_name: "Burst Detector",
            category: "Recovery",
            description: "Power-based burst detection with dual-threshold hysteresis. \
                Detects signal bursts by comparing smoothed power against high/low thresholds. \
                Supports minimum burst length filtering and holdoff between bursts. \
                Can gate input signals (zero non-burst samples).",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/burst_detector.rs",
                line: 1,
                symbol: "BurstDetector",
                description: "Power-based burst detection with hysteresis",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Power Estimate",
                    plaintext: "P[n] = (1-α)·P[n-1] + α·|x[n]|², α_attack > α_release",
                    latex: Some(r"P[n] = (1-\alpha)P[n-1] + \alpha|x[n]|^2"),
                    variables: &[
                        ("P[n]", "Smoothed power estimate (dB)"),
                        ("α", "Smoothing factor (asymmetric attack/release)"),
                        ("x[n]", "Input sample"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_burst_detection_basic", module: "r4w_core::burst_detector::tests", description: "Basic burst detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_burst_gating", module: "r4w_core::burst_detector::tests", description: "Signal gating", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) state", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Agc", "AwgnChannel"],
            input_type: "IQ samples",
            output_type: "IQ samples (gated)",
            key_parameters: &["high_threshold_db", "low_threshold_db"],
        });

        m.insert("GoertzelDetector", BlockMetadata {
            block_type: "GoertzelDetector",
            display_name: "Goertzel Detector",
            category: "Filtering",
            description: "Efficient single-frequency DFT using the Goertzel algorithm. \
                Computes the magnitude at one specific frequency bin with O(N) operations, \
                much faster than a full FFT when only one frequency is needed. \
                Includes a built-in DTMF detector for telephony applications.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/goertzel.rs",
                line: 1,
                symbol: "Goertzel",
                description: "Single-frequency DFT using second-order IIR",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Goertzel Recurrence",
                    plaintext: "s[n] = x[n] + 2·cos(2πk/N)·s[n-1] - s[n-2]",
                    latex: Some(r"s[n] = x[n] + 2\cos\left(\frac{2\pi k}{N}\right)s[n-1] - s[n-2]"),
                    variables: &[
                        ("k", "Target frequency bin index = f_target/f_s × N"),
                        ("N", "Block size (DFT length)"),
                        ("s[n]", "IIR state variable"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_goertzel_single_tone", module: "r4w_core::goertzel::tests", description: "Single tone detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_dtmf_detection", module: "r4w_core::goertzel::tests", description: "DTMF digit detection", expected_runtime_ms: 10 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(N) per frequency", memory: "O(1) per frequency", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "ITU-T Q.23/Q.24", section: Some("DTMF signaling"), url: None },
            ],
            related_blocks: &["FirFilter"],
            input_type: "IQ samples",
            output_type: "Real (magnitude per block)",
            key_parameters: &["target_freq_hz", "sample_rate_hz", "block_size"],
        });

        m.insert("CostasLoop", BlockMetadata {
            block_type: "CostasLoop",
            display_name: "Costas Loop",
            category: "Recovery",
            description: "Decision-directed carrier recovery loop for PSK signals. \
                Uses modulation-specific phase detectors: BPSK (re·im), QPSK (cross-product), \
                8PSK (8th-power). Tracks both phase and frequency offset with a second-order \
                PI loop filter.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/costas_loop.rs",
                line: 96,
                symbol: "CostasLoop",
                description: "Decision-directed carrier recovery (BPSK/QPSK/8PSK)",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "QPSK Phase Detector",
                    plaintext: "e = Re(y)·sgn(Im(y)) - Im(y)·sgn(Re(y))",
                    latex: Some(r"e = \text{Re}(y)\text{sgn}(\text{Im}(y)) - \text{Im}(y)\text{sgn}(\text{Re}(y))"),
                    variables: &[
                        ("y", "De-rotated input symbol"),
                        ("e", "Phase error"),
                        ("sgn", "Sign function"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_bpsk_costas_tracks_offset", module: "r4w_core::costas_loop::tests", description: "BPSK carrier tracking", expected_runtime_ms: 5 },
                BlockTest { name: "test_qpsk_costas_reduces_error", module: "r4w_core::costas_loop::tests", description: "QPSK convergence", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) loop state", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "Costas, 1956", section: Some("Synchronous communications"), url: None },
            ],
            related_blocks: &["CarrierRecovery", "ConstellationRx"],
            input_type: "IQ samples",
            output_type: "IQ samples (carrier-corrected)",
            key_parameters: &["order", "loop_bw"],
        });

        m.insert("ConstellationRx", BlockMetadata {
            block_type: "ConstellationRx",
            display_name: "Constellation Receiver",
            category: "Recovery",
            description: "Combined receiver block: AGC → Costas Loop → Symbol Demapper. \
                Performs amplitude normalization, carrier recovery, and symbol decisions \
                in a single processing chain. Supports BPSK/QPSK/8PSK/16QAM with hard \
                or soft (LLR) output.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/constellation_receiver.rs",
                line: 129,
                symbol: "ConstellationReceiver",
                description: "Combined AGC + carrier recovery + demapper receiver",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/costas_loop.rs",
                    line: 96,
                    symbol: "CostasLoop",
                    description: "Carrier recovery component",
                },
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/symbol_mapping.rs",
                    line: 1,
                    symbol: "SymbolMapper",
                    description: "Symbol demapping component",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "Signal Flow",
                    plaintext: "input → AGC → Costas Loop → Symbol Decision → bits/LLRs",
                    latex: None,
                    variables: &[],
                },
            ],
            tests: &[
                BlockTest { name: "test_bpsk_receiver_basic", module: "r4w_core::constellation_receiver::tests", description: "BPSK reception", expected_runtime_ms: 5 },
                BlockTest { name: "test_qpsk_receiver", module: "r4w_core::constellation_receiver::tests", description: "QPSK reception", expected_runtime_ms: 5 },
                BlockTest { name: "test_soft_output", module: "r4w_core::constellation_receiver::tests", description: "LLR soft decisions", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) receiver state", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["CostasLoop", "Agc", "PskDemodulator"],
            input_type: "IQ samples",
            output_type: "IQ samples (corrected symbols)",
            key_parameters: &["modulation", "loop_bw"],
        });

    // --- Batch 11: FreqXlatingFir, FM Emphasis, CTCSS, Stream Control, LogPowerFft ---

    m.insert("FreqXlatingFir", BlockMetadata {
            block_type: "FreqXlatingFir",
            display_name: "Freq Xlating FIR Filter",
            category: "Filtering",
            description: "Combines frequency translation (mixing), FIR filtering, and decimation in a single efficient block. Shifts a narrowband signal to baseband, filters, and decimates.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/freq_xlating_fir.rs",
                line: 43,
                symbol: "FreqXlatingFirFilter",
                description: "Frequency-translating FIR filter with decimation",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Frequency Translation + FIR + Decimation",
                    plaintext: "y[m] = sum_k( h[k] * x[mD+k] * exp(-j*2*pi*fc*(mD+k)/fs) )",
                    latex: Some("y[m] = \\sum_k h[k] \\cdot x[mD+k] \\cdot e^{-j2\\pi f_c (mD+k)/f_s}"),
                    variables: &[
                        ("h[k]", "FIR taps"),
                        ("D", "decimation factor"),
                        ("fc", "center frequency"),
                        ("fs", "sample rate"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_passthrough_no_shift", module: "r4w_core::freq_xlating_fir::tests", description: "No shift, no decimation", expected_runtime_ms: 5 },
                BlockTest { name: "test_decimation", module: "r4w_core::freq_xlating_fir::tests", description: "Decimation output length", expected_runtime_ms: 5 },
                BlockTest { name: "test_frequency_shift", module: "r4w_core::freq_xlating_fir::tests", description: "DC offset removal", expected_runtime_ms: 5 },
                BlockTest { name: "test_tone_shift_to_dc", module: "r4w_core::freq_xlating_fir::tests", description: "Shift tone to baseband", expected_runtime_ms: 10 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n * T / D)", memory: "O(T) delay line", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["FirFilter", "Nco", "PolyphaseDecimator"],
            input_type: "IQ samples",
            output_type: "IQ samples (baseband, decimated)",
            key_parameters: &["center_freq_hz", "sample_rate_hz", "decimation", "num_taps"],
        });

    m.insert("DeEmphasis", BlockMetadata {
            block_type: "DeEmphasis",
            display_name: "FM De-Emphasis Filter",
            category: "Filtering",
            description: "1-pole IIR lowpass filter for FM audio de-emphasis. Reverses the pre-emphasis applied at the transmitter to flatten the audio spectrum and reduce high-frequency noise.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/fm_emphasis.rs",
                line: 38,
                symbol: "DeEmphasisFilter",
                description: "FM de-emphasis 1-pole IIR lowpass",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "De-Emphasis IIR",
                    plaintext: "y[n] = (1-a)*x[n] + a*y[n-1], where a = exp(-1/(tau*fs))",
                    latex: Some("y[n] = (1-\\alpha)x[n] + \\alpha y[n-1], \\quad \\alpha = e^{-1/(\\tau f_s)}"),
                    variables: &[
                        ("tau", "time constant (75us US, 50us EU)"),
                        ("fs", "sample rate"),
                        ("alpha", "filter coefficient"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_deemphasis_lowpass", module: "r4w_core::fm_emphasis::tests", description: "High freq attenuation", expected_runtime_ms: 5 },
                BlockTest { name: "test_deemphasis_dc_pass", module: "r4w_core::fm_emphasis::tests", description: "DC passthrough", expected_runtime_ms: 5 },
                BlockTest { name: "test_us_vs_eu_standards", module: "r4w_core::fm_emphasis::tests", description: "Standard comparison", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) state", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "ITU-R BS.450", section: Some("75us/50us pre-emphasis"), url: Some("https://www.itu.int/rec/R-REC-BS.450") },
            ],
            related_blocks: &["PreEmphasis", "FirFilter", "IirFilter"],
            input_type: "Real audio samples",
            output_type: "Real audio samples (de-emphasized)",
            key_parameters: &["standard", "sample_rate_hz"],
        });

    m.insert("PreEmphasis", BlockMetadata {
            block_type: "PreEmphasis",
            display_name: "FM Pre-Emphasis Filter",
            category: "Filtering",
            description: "1-pole IIR highpass filter for FM audio pre-emphasis. Boosts high-frequency content before transmission to improve SNR after de-emphasis at the receiver.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/fm_emphasis.rs",
                line: 79,
                symbol: "PreEmphasisFilter",
                description: "FM pre-emphasis 1-pole IIR highpass",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Pre-Emphasis IIR",
                    plaintext: "y[n] = (x[n] - a*x[n-1]) / (1-a)",
                    latex: Some("y[n] = \\frac{x[n] - \\alpha x[n-1]}{1-\\alpha}"),
                    variables: &[
                        ("alpha", "exp(-1/(tau*fs))"),
                        ("tau", "time constant"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_preemphasis_highpass", module: "r4w_core::fm_emphasis::tests", description: "High freq boost", expected_runtime_ms: 5 },
                BlockTest { name: "test_emphasis_roundtrip", module: "r4w_core::fm_emphasis::tests", description: "Pre->De roundtrip", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) state", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "ITU-R BS.450", section: Some("FM pre-emphasis time constants"), url: Some("https://www.itu.int/rec/R-REC-BS.450") },
            ],
            related_blocks: &["DeEmphasis", "FirFilter"],
            input_type: "Real audio samples",
            output_type: "Real audio samples (pre-emphasized)",
            key_parameters: &["standard", "sample_rate_hz"],
        });

    m.insert("CtcssSquelch", BlockMetadata {
            block_type: "CtcssSquelch",
            display_name: "CTCSS Tone Squelch",
            category: "Synchronization",
            description: "Continuous Tone-Coded Squelch System. Detects sub-audible tones (67-250.3 Hz) using the Goertzel algorithm to gate audio. Standard in FM two-way radio.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/squelch.rs",
                line: 1,
                symbol: "CtcssSquelch",
                description: "CTCSS tone detection and squelch gating",
            }),
            related_code: &[
                CodeLocation {
                    crate_name: "r4w-core",
                    file_path: "src/goertzel.rs",
                    line: 1,
                    symbol: "Goertzel",
                    description: "Single-bin DFT detector",
                },
            ],
            formulas: &[
                BlockFormula {
                    name: "Goertzel Detection",
                    plaintext: "coeff = 2*cos(2*pi*k/N), power = s1^2+s2^2-coeff*s1*s2",
                    latex: Some("\\text{coeff} = 2\\cos(2\\pi k/N), \\quad P = s_1^2 + s_2^2 - \\text{coeff} \\cdot s_1 \\cdot s_2"),
                    variables: &[
                        ("k", "tone bin index"),
                        ("N", "block size"),
                        ("s1,s2", "Goertzel state"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_ctcss_opens_with_correct_tone", module: "r4w_core::squelch::tests", description: "Squelch opens on tone", expected_runtime_ms: 20 },
                BlockTest { name: "test_ctcss_stays_closed_wrong_tone", module: "r4w_core::squelch::tests", description: "Rejects wrong tone", expected_runtime_ms: 20 },
                BlockTest { name: "test_ctcss_tone_table", module: "r4w_core::squelch::tests", description: "EIA tone validation", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(N) block buffer", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "EIA/TIA-603", section: Some("38 standard CTCSS tones"), url: Some("https://www.tiaonline.org/") },
            ],
            related_blocks: &["GoertzelDetector", "PowerSquelch", "Squelch"],
            input_type: "Real audio samples",
            output_type: "Real audio samples (gated)",
            key_parameters: &["tone_freq", "sample_rate_hz", "threshold"],
        });

    m.insert("HeadBlock", BlockMetadata {
            block_type: "HeadBlock",
            display_name: "Head (First N Samples)",
            category: "Stream Control",
            description: "Passes only the first N samples, then outputs nothing. Useful for limiting recording duration or taking a fixed-size snapshot of a signal.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/stream_control.rs",
                line: 29,
                symbol: "Head",
                description: "Pass first N samples, then stop",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Head",
                    plaintext: "y[n] = x[n] if n < N, else nothing",
                    latex: None,
                    variables: &[
                        ("N", "number of samples to pass"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_head_limits_samples", module: "r4w_core::stream_control::tests", description: "Limits output count", expected_runtime_ms: 5 },
                BlockTest { name: "test_head_multiple_blocks", module: "r4w_core::stream_control::tests", description: "Multi-block accumulation", expected_runtime_ms: 5 },
                BlockTest { name: "test_head_sample_by_sample", module: "r4w_core::stream_control::tests", description: "Per-sample processing", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(1) per sample", memory: "O(1) counter", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["SkipHeadBlock", "Throttle"],
            input_type: "IQ samples",
            output_type: "IQ samples (first N)",
            key_parameters: &["num_samples"],
        });

    m.insert("SkipHeadBlock", BlockMetadata {
            block_type: "SkipHeadBlock",
            display_name: "Skip Head (Drop First N)",
            category: "Stream Control",
            description: "Drops the first N samples, then passes everything through. Useful for skipping transient startup effects or initial settling time.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/stream_control.rs",
                line: 86,
                symbol: "SkipHead",
                description: "Drop first N samples, pass the rest",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Skip Head",
                    plaintext: "y[n] = x[n+N] (output starts after N samples dropped)",
                    latex: None,
                    variables: &[
                        ("N", "number of samples to skip"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_skip_head_drops_samples", module: "r4w_core::stream_control::tests", description: "Drops correct count", expected_runtime_ms: 5 },
                BlockTest { name: "test_skip_head_multiple_blocks", module: "r4w_core::stream_control::tests", description: "Multi-block skipping", expected_runtime_ms: 5 },
                BlockTest { name: "test_skip_head_sample_by_sample", module: "r4w_core::stream_control::tests", description: "Per-sample processing", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(1) per sample", memory: "O(1) counter", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["HeadBlock", "Throttle"],
            input_type: "IQ samples",
            output_type: "IQ samples (after skip)",
            key_parameters: &["num_samples"],
        });

    m.insert("LogPowerFft", BlockMetadata {
            block_type: "LogPowerFft",
            display_name: "Log Power FFT",
            category: "Analysis",
            description: "Computes windowed FFT, converts to dB power, and applies exponential averaging for continuous spectrum monitoring. Supports multiple window functions and optional DC-centered output.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/log_power_fft.rs",
                line: 79,
                symbol: "LogPowerFft",
                description: "Streaming spectrum estimation in dB",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Log Power Spectrum",
                    plaintext: "P[k] = 10*log10(|FFT{w*x}[k]|^2 / N^2), avg[k] = a*P[k] + (1-a)*avg_prev[k]",
                    latex: Some("P[k] = 10\\log_{10}\\frac{|\\text{FFT}\\{w \\cdot x\\}[k]|^2}{N^2}, \\quad \\bar{P}[k] = \\alpha P[k] + (1-\\alpha)\\bar{P}_{\\text{prev}}[k]"),
                    variables: &[
                        ("w", "window function"),
                        ("N", "FFT size"),
                        ("alpha", "averaging factor (0=full avg, 1=no avg)"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_log_power_fft_basic", module: "r4w_core::log_power_fft::tests", description: "DC signal detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_log_power_fft_tone", module: "r4w_core::log_power_fft::tests", description: "Tone peak detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_log_power_fft_center_dc", module: "r4w_core::log_power_fft::tests", description: "FFT shift to center DC", expected_runtime_ms: 5 },
                BlockTest { name: "test_fft_inplace_known_result", module: "r4w_core::log_power_fft::tests", description: "FFT correctness", expected_runtime_ms: 5 },
                BlockTest { name: "test_window_types", module: "r4w_core::log_power_fft::tests", description: "Window function validation", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(N*log N) per FFT", memory: "O(N) spectrum + scratch", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["SpectrumAnalyzer", "WaterfallGenerator"],
            input_type: "IQ samples",
            output_type: "Real dB spectrum",
            key_parameters: &["fft_size", "avg_alpha", "window"],
        });

    // --- Batch 12: Type conversions, Quad Demod, Access Code, Fractional Resampler, FLL Band-Edge ---

    m.insert("QuadratureDemod", BlockMetadata {
            block_type: "QuadratureDemod",
            display_name: "Quadrature Demodulator",
            category: "Recovery",
            description: "FM discriminator. Computes instantaneous frequency by measuring phase difference between consecutive samples. Fundamental block for FM, NBFM, WBFM, FSK, GFSK, GMSK reception.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/quadrature_demod.rs",
                line: 49,
                symbol: "QuadratureDemod",
                description: "FM discriminator: y[n] = gain * arg(x[n] * conj(x[n-1]))",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Quadrature Demodulation",
                    plaintext: "y[n] = gain * arg(x[n] * conj(x[n-1]))",
                    latex: Some("y[n] = g \\cdot \\arg(x[n] \\cdot x^*[n-1])"),
                    variables: &[
                        ("g", "gain = fs / (2*pi*max_deviation) for FM"),
                        ("arg", "complex argument (atan2)"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_dc_input_zero_output", module: "r4w_core::quadrature_demod::tests", description: "DC gives zero", expected_runtime_ms: 5 },
                BlockTest { name: "test_positive_frequency", module: "r4w_core::quadrature_demod::tests", description: "Positive tone detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_fm_gain_normalization", module: "r4w_core::quadrature_demod::tests", description: "FM gain normalization", expected_runtime_ms: 5 },
                BlockTest { name: "test_fsk_binary", module: "r4w_core::quadrature_demod::tests", description: "FSK mark/space", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) prev sample", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["DeEmphasis", "FskDemodulator", "PLL"],
            input_type: "IQ samples",
            output_type: "Real (instantaneous frequency)",
            key_parameters: &["gain", "mode"],
        });

    m.insert("AccessCodeDetector", BlockMetadata {
            block_type: "AccessCodeDetector",
            display_name: "Access Code Detector",
            category: "Synchronization",
            description: "Bit-level sync word detector. Searches a binary stream for a specified access code using a shift register and Hamming distance threshold for bit error tolerance.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/access_code_detector.rs",
                line: 56,
                symbol: "AccessCodeDetector",
                description: "Shift register correlator with Hamming distance",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Hamming Distance",
                    plaintext: "d(a,b) = popcount(a XOR b) <= threshold",
                    latex: Some("d(a,b) = \\text{popcount}(a \\oplus b) \\leq T"),
                    variables: &[
                        ("a", "shift register contents"),
                        ("b", "access code pattern"),
                        ("T", "maximum allowed bit errors"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_exact_match", module: "r4w_core::access_code_detector::tests", description: "Exact code detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_threshold_tolerance", module: "r4w_core::access_code_detector::tests", description: "Bit error tolerance", expected_runtime_ms: 5 },
                BlockTest { name: "test_multiple_detections", module: "r4w_core::access_code_detector::tests", description: "Multiple matches", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n) per bit", memory: "O(1) shift register", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Correlator", "SyncWordInsert", "PreambleInsert"],
            input_type: "Bits",
            output_type: "Bits (with detection tags)",
            key_parameters: &["access_code_hex", "threshold"],
        });

    m.insert("FractionalResampler", BlockMetadata {
            block_type: "FractionalResampler",
            display_name: "Fractional Resampler",
            category: "Rate Conversion",
            description: "Arbitrary (non-rational) sample rate conversion using linear interpolation. Supports irrational ratios (e.g., 48kHz to 44.1kHz) and variable-rate operation.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/filters/fractional_resampler.rs",
                line: 30,
                symbol: "FractionalResampler",
                description: "MMSE interpolating resampler",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Linear Interpolation",
                    plaintext: "y[m] = (1-mu)*x[n] + mu*x[n+1], mu += step per output",
                    latex: Some("y[m] = (1-\\mu)x[n] + \\mu x[n+1]"),
                    variables: &[
                        ("mu", "fractional delay (0 to 1)"),
                        ("step", "1/ratio = input samples per output"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_unity_ratio_passthrough", module: "r4w_core::filters::fractional_resampler::tests", description: "Unity ratio", expected_runtime_ms: 5 },
                BlockTest { name: "test_downsample_2x", module: "r4w_core::filters::fractional_resampler::tests", description: "2x downsample", expected_runtime_ms: 5 },
                BlockTest { name: "test_irrational_ratio", module: "r4w_core::filters::fractional_resampler::tests", description: "48k to 44.1k", expected_runtime_ms: 5 },
                BlockTest { name: "test_tone_preservation", module: "r4w_core::filters::fractional_resampler::tests", description: "Tone frequency preserved", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1) history", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["Upsampler", "Downsampler", "RationalResampler", "PolyphaseResampler"],
            input_type: "IQ samples",
            output_type: "IQ samples (resampled)",
            key_parameters: &["ratio"],
        });

    m.insert("FllBandEdge", BlockMetadata {
            block_type: "FllBandEdge",
            display_name: "FLL Band-Edge",
            category: "Recovery",
            description: "Coarse frequency synchronization using band-edge FIR filters. Power difference between upper/lower band-edge filter outputs generates frequency error for NCO correction.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/fll_band_edge.rs",
                line: 66,
                symbol: "FllBandEdge",
                description: "Band-edge FLL with 2nd-order loop filter",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Band-Edge Frequency Error",
                    plaintext: "error = |x_lower|^2 - |x_upper|^2",
                    latex: Some("e = |x_l|^2 - |x_u|^2"),
                    variables: &[
                        ("x_l", "lower band-edge filter output"),
                        ("x_u", "upper band-edge filter output"),
                    ],
                },
            ],
            tests: &[
                BlockTest { name: "test_dc_no_correction", module: "r4w_core::fll_band_edge::tests", description: "DC stability", expected_runtime_ms: 5 },
                BlockTest { name: "test_frequency_correction_convergence", module: "r4w_core::fll_band_edge::tests", description: "Freq offset correction", expected_runtime_ms: 10 },
                BlockTest { name: "test_band_edge_filters_symmetric", module: "r4w_core::fll_band_edge::tests", description: "Filter symmetry", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n * T) per sample", memory: "O(T) filter taps + delay line", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["CostasLoop", "CarrierRecovery", "PLL"],
            input_type: "IQ samples",
            output_type: "IQ samples (frequency-corrected)",
            key_parameters: &["samples_per_symbol", "rolloff", "filter_size", "loop_bandwidth"],
        });

    m.insert("ComplexToMag", BlockMetadata {
            block_type: "ComplexToMag",
            display_name: "Complex to Magnitude",
            category: "Type Conversion",
            description: "Extracts magnitude (envelope) from complex samples: |z| = sqrt(re^2 + im^2). Use ComplexToMagSq for power without sqrt.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/type_conversions.rs",
                line: 39,
                symbol: "ComplexToMag",
                description: "Complex to magnitude conversion",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Magnitude",
                    plaintext: "|z| = sqrt(re^2 + im^2)",
                    latex: Some("|z| = \\sqrt{\\text{re}^2 + \\text{im}^2}"),
                    variables: &[],
                },
            ],
            tests: &[
                BlockTest { name: "test_complex_to_mag", module: "r4w_core::type_conversions::tests", description: "3+4j -> 5", expected_runtime_ms: 5 },
                BlockTest { name: "test_mag_phase_roundtrip", module: "r4w_core::type_conversions::tests", description: "Mag/phase roundtrip", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["ComplexToArg", "ComplexToReal", "IqSplit"],
            input_type: "IQ samples",
            output_type: "Real (magnitude)",
            key_parameters: &[],
        });

    m.insert("ComplexToArg", BlockMetadata {
            block_type: "ComplexToArg",
            display_name: "Complex to Phase",
            category: "Type Conversion",
            description: "Extracts phase angle from complex samples: arg(z) = atan2(im, re). Range: (-pi, pi].",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/type_conversions.rs",
                line: 56,
                symbol: "ComplexToArg",
                description: "Complex to phase angle",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Phase Angle",
                    plaintext: "arg(z) = atan2(im, re)",
                    latex: Some("\\arg(z) = \\text{atan2}(\\text{im}, \\text{re})"),
                    variables: &[],
                },
            ],
            tests: &[
                BlockTest { name: "test_complex_to_arg", module: "r4w_core::type_conversions::tests", description: "Phase extraction", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["ComplexToMag", "ComplexToReal", "QuadratureDemod"],
            input_type: "IQ samples",
            output_type: "Real (phase in radians)",
            key_parameters: &[],
        });

    m.insert("ComplexToReal", BlockMetadata {
            block_type: "ComplexToReal",
            display_name: "Complex to Real",
            category: "Type Conversion",
            description: "Extracts the real (in-phase) component from complex samples, discarding the imaginary (quadrature) part.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/type_conversions.rs",
                line: 66,
                symbol: "ComplexToReal",
                description: "Extract real part",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Real Part",
                    plaintext: "y = re(z)",
                    latex: Some("y = \\text{Re}(z)"),
                    variables: &[],
                },
            ],
            tests: &[
                BlockTest { name: "test_complex_to_real", module: "r4w_core::type_conversions::tests", description: "Real extraction", expected_runtime_ms: 5 },
                BlockTest { name: "test_real_imag_roundtrip", module: "r4w_core::type_conversions::tests", description: "Re/Im roundtrip", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["ComplexToImag", "ComplexToMag", "IqSplit"],
            input_type: "IQ samples",
            output_type: "Real",
            key_parameters: &[],
        });

    m.insert("RealToComplex", BlockMetadata {
            block_type: "RealToComplex",
            display_name: "Real to Complex",
            category: "Type Conversion",
            description: "Converts real samples to complex by setting imaginary part to zero: re -> re + j*0.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/type_conversions.rs",
                line: 76,
                symbol: "RealToComplex",
                description: "Real to complex conversion",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Real to Complex",
                    plaintext: "z = re + j*0",
                    latex: Some("z = x + j \\cdot 0"),
                    variables: &[],
                },
            ],
            tests: &[
                BlockTest { name: "test_real_to_complex", module: "r4w_core::type_conversions::tests", description: "Real to complex", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["ComplexToReal", "IqMerge"],
            input_type: "Real",
            output_type: "IQ samples",
            key_parameters: &[],
        });

    // Batch 13: PFB Synthesizer, Moving Average, KeepOneInN, SampleRepeat
    m.insert("PfbSynthesizer", BlockMetadata {
            block_type: "PfbSynthesizer",
            display_name: "PFB Synthesizer",
            category: "Source",
            description: "Polyphase Filter Bank Synthesizer. Combines M narrowband channel streams into a single wideband output using polyphase-FFT synthesis. Inverse of PFB Channelizer.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/pfb_synthesizer.rs",
                line: 70,
                symbol: "PfbSynthesizer",
                description: "PFB synthesis with M-point IFFT and polyphase sub-filters",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "PFB Synthesis",
                    plaintext: "y[n] = commutate( polyphase_filter( IFFT( X[0..M-1] ) ) )",
                    latex: Some("y[n] = \\sum_{k=0}^{M-1} h_k[n] \\cdot \\text{IFFT}\\{X_k\\}"),
                    variables: &[("M", "Number of channels"), ("h_k", "Polyphase sub-filter for branch k"), ("X_k", "Channel k input")],
                },
            ],
            tests: &[
                BlockTest { name: "test_basic_construction", module: "r4w_core::pfb_synthesizer::tests", description: "Construction with defaults", expected_runtime_ms: 5 },
                BlockTest { name: "test_output_length", module: "r4w_core::pfb_synthesizer::tests", description: "Output = M * N samples", expected_runtime_ms: 5 },
                BlockTest { name: "test_ifft_known_result", module: "r4w_core::pfb_synthesizer::tests", description: "IFFT [4,0,0,0] = [1,1,1,1]", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(M*log(M) + M*T) per output block", memory: "O(M*T)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[
                StandardReference { name: "Fredric J. Harris, Multirate Signal Processing", section: Some("Ch. 7: Polyphase Filter Banks"), url: None },
            ],
            related_blocks: &["PfbChannelizer", "Upsampler"],
            input_type: "M IQ channel streams",
            output_type: "Wideband IQ",
            key_parameters: &["num_channels", "taps_per_channel", "window"],
        });

    m.insert("MovingAverage", BlockMetadata {
            block_type: "MovingAverage",
            display_name: "Moving Average",
            category: "Filtering",
            description: "Efficient O(1)-per-sample moving average filter using a circular buffer and running sum. Smooths signal by averaging the last N samples.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/filters/moving_average.rs",
                line: 25,
                symbol: "MovingAverage",
                description: "O(1) moving average with circular buffer",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Moving Average",
                    plaintext: "y[n] = (1/N) * sum(x[n-k], k=0..N-1)",
                    latex: Some("y[n] = \\frac{1}{N} \\sum_{k=0}^{N-1} x[n-k]"),
                    variables: &[("N", "Window length"), ("x[n]", "Input sample at time n")],
                },
            ],
            tests: &[
                BlockTest { name: "test_dc_convergence", module: "r4w_core::filters::moving_average::tests", description: "DC input converges to DC", expected_runtime_ms: 5 },
                BlockTest { name: "test_ramp_averaging", module: "r4w_core::filters::moving_average::tests", description: "Ramp signal averaging", expected_runtime_ms: 5 },
                BlockTest { name: "test_complex_averaging", module: "r4w_core::filters::moving_average::tests", description: "Complex-valued averaging", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(1) per sample", memory: "O(N)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["FirFilter", "DcBlocker"],
            input_type: "IQ samples",
            output_type: "IQ samples",
            key_parameters: &["length"],
        });

    m.insert("KeepOneInN", BlockMetadata {
            block_type: "KeepOneInN",
            display_name: "Keep 1 in N",
            category: "Rate Conversion",
            description: "Decimation without anti-alias filtering. Keeps every Nth sample and discards the rest. Use after a lowpass filter to prevent aliasing.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/sample_ops.rs",
                line: 32,
                symbol: "KeepOneInN",
                description: "Simple decimation by N",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Decimation",
                    plaintext: "y[m] = x[m*N]",
                    latex: Some("y[m] = x[mN]"),
                    variables: &[("N", "Decimation factor"), ("m", "Output sample index")],
                },
            ],
            tests: &[
                BlockTest { name: "test_keep_one_in_n_basic", module: "r4w_core::sample_ops::tests", description: "Keep every 3rd sample", expected_runtime_ms: 5 },
                BlockTest { name: "test_keep_repeat_inverse", module: "r4w_core::sample_ops::tests", description: "Repeat then Keep recovers original", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n/N)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Downsampler", "SampleRepeat", "CicDecimator"],
            input_type: "IQ samples",
            output_type: "IQ samples (decimated)",
            key_parameters: &["n"],
        });

    m.insert("SampleRepeat", BlockMetadata {
            block_type: "SampleRepeat",
            display_name: "Repeat",
            category: "Rate Conversion",
            description: "Sample repetition without anti-image filtering. Replicates each input sample N times. Use before a pulse-shaping or interpolation filter.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/sample_ops.rs",
                line: 94,
                symbol: "Repeat",
                description: "Simple sample repetition by N",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Repetition",
                    plaintext: "y[n] = x[floor(n/N)]",
                    latex: Some("y[n] = x[\\lfloor n/N \\rfloor]"),
                    variables: &[("N", "Repetition factor"), ("n", "Output sample index")],
                },
            ],
            tests: &[
                BlockTest { name: "test_repeat_basic", module: "r4w_core::sample_ops::tests", description: "Repeat each sample 3 times", expected_runtime_ms: 5 },
                BlockTest { name: "test_keep_repeat_inverse", module: "r4w_core::sample_ops::tests", description: "Repeat then Keep recovers original", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*N)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Upsampler", "KeepOneInN", "PolyphaseResampler"],
            input_type: "IQ samples",
            output_type: "IQ samples (interpolated)",
            key_parameters: &["n"],
        });

    // Batch 14: Delay, MultiplyConst, PackKBits, UnpackKBits, PowerSquelch, PlateauDetector
    m.insert("SampleDelay", BlockMetadata {
            block_type: "SampleDelay",
            display_name: "Delay",
            category: "Stream Operations",
            description: "Delays an input sample stream by a configurable number of samples, inserting zeros at the beginning. Used for path alignment, filter group delay compensation, and feedback loops.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/delay.rs",
                line: 34,
                symbol: "Delay",
                description: "Configurable sample delay using VecDeque",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Delay",
                    plaintext: "y[n] = x[n - D]",
                    latex: Some("y[n] = x[n - D]"),
                    variables: &[("D", "Delay in samples")],
                },
            ],
            tests: &[
                BlockTest { name: "test_delay_inserts_zeros", module: "r4w_core::delay::tests", description: "Verifies zeros inserted", expected_runtime_ms: 5 },
                BlockTest { name: "test_delay_continuity", module: "r4w_core::delay::tests", description: "State across blocks", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(1) per sample", memory: "O(D)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["HeadBlock", "SkipHeadBlock"],
            input_type: "IQ samples",
            output_type: "IQ samples (delayed)",
            key_parameters: &["delay_samples"],
        });

    m.insert("MultiplyConst", BlockMetadata {
            block_type: "MultiplyConst",
            display_name: "Multiply Const",
            category: "Filtering",
            description: "Multiplies every sample by a fixed complex constant. Used for gain control, phase rotation, and frequency shifting (when combined with NCO).",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/multiply.rs",
                line: 54,
                symbol: "MultiplyConst",
                description: "Constant complex multiplication",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Multiply Const",
                    plaintext: "y[n] = c * x[n]",
                    latex: Some("y[n] = c \\cdot x[n]"),
                    variables: &[("c", "Complex constant (gain_re + j*gain_im)"), ("x[n]", "Input sample")],
                },
            ],
            tests: &[
                BlockTest { name: "test_multiply_const_gain", module: "r4w_core::multiply::tests", description: "Scalar gain", expected_runtime_ms: 5 },
                BlockTest { name: "test_multiply_const_rotation", module: "r4w_core::multiply::tests", description: "90 degree rotation by j", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["Agc", "FirFilter"],
            input_type: "IQ samples",
            output_type: "IQ samples",
            key_parameters: &["gain_re", "gain_im"],
        });

    m.insert("PackKBits", BlockMetadata {
            block_type: "PackKBits",
            display_name: "Pack K Bits",
            category: "Coding",
            description: "Packs K individual bits (one bit per byte, in LSB) into bytes with K bits each (MSB first). Essential glue between FEC decoders and modulators.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/bit_packing.rs",
                line: 29,
                symbol: "PackKBits",
                description: "Bit packing with configurable K",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Bit Packing",
                    plaintext: "output_byte = b[0]<<(K-1) | b[1]<<(K-2) | ... | b[K-1]",
                    latex: Some("y = \\sum_{i=0}^{K-1} b_i \\cdot 2^{K-1-i}"),
                    variables: &[("K", "Bits per output byte"), ("b_i", "Input bit i")],
                },
            ],
            tests: &[
                BlockTest { name: "test_pack_8_bits", module: "r4w_core::bit_packing::tests", description: "Pack 8 bits into byte", expected_runtime_ms: 5 },
                BlockTest { name: "test_pack_unpack_roundtrip", module: "r4w_core::bit_packing::tests", description: "Pack/unpack roundtrip", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["UnpackKBits", "Scrambler", "FecEncoder"],
            input_type: "Bits (1 per byte)",
            output_type: "Bits (K per byte)",
            key_parameters: &["k"],
        });

    m.insert("UnpackKBits", BlockMetadata {
            block_type: "UnpackKBits",
            display_name: "Unpack K Bits",
            category: "Coding",
            description: "Unpacks bytes with K bits each into individual bits (one per byte, MSB first). Inverse of Pack K Bits.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/bit_packing.rs",
                line: 65,
                symbol: "UnpackKBits",
                description: "Bit unpacking with configurable K",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Bit Unpacking",
                    plaintext: "b[i] = (input_byte >> (K-1-i)) & 1",
                    latex: Some("b_i = \\lfloor y / 2^{K-1-i} \\rfloor \\mod 2"),
                    variables: &[("K", "Bits per input byte"), ("b_i", "Output bit i")],
                },
            ],
            tests: &[
                BlockTest { name: "test_unpack_8_bits", module: "r4w_core::bit_packing::tests", description: "Unpack byte to 8 bits", expected_runtime_ms: 5 },
                BlockTest { name: "test_pack_unpack_roundtrip", module: "r4w_core::bit_packing::tests", description: "Pack/unpack roundtrip", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*K)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["PackKBits", "Scrambler"],
            input_type: "Bits (K per byte)",
            output_type: "Bits (1 per byte)",
            key_parameters: &["k"],
        });

    m.insert("PowerSquelch", BlockMetadata {
            block_type: "PowerSquelch",
            display_name: "Power Squelch",
            category: "Recovery",
            description: "Gates signal based on power level. Passes samples when power exceeds threshold, outputs zeros otherwise. Includes hysteresis and attack/release timing.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/power_squelch.rs",
                line: 26,
                symbol: "PowerSquelch",
                description: "Power-based signal gating with hysteresis",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Power Squelch",
                    plaintext: "y[n] = x[n] if P_smooth > threshold, else 0",
                    latex: Some("y[n] = \\begin{cases} x[n] & P_{\\text{smooth}} > T \\\\ 0 & \\text{otherwise} \\end{cases}"),
                    variables: &[("T", "Threshold in dBFS"), ("P_smooth", "Smoothed power estimate")],
                },
            ],
            tests: &[
                BlockTest { name: "test_quiet_signal_gated", module: "r4w_core::power_squelch::tests", description: "Quiet signal is gated", expected_runtime_ms: 5 },
                BlockTest { name: "test_loud_signal_passes", module: "r4w_core::power_squelch::tests", description: "Loud signal passes through", expected_runtime_ms: 5 },
                BlockTest { name: "test_hysteresis", module: "r4w_core::power_squelch::tests", description: "Hysteresis prevents toggling", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(1) per sample", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["CtcssSquelch", "BurstDetector", "Agc"],
            input_type: "IQ samples",
            output_type: "IQ samples (gated)",
            key_parameters: &["threshold_db", "hysteresis_db"],
        });

    m.insert("PlateauDetector", BlockMetadata {
            block_type: "PlateauDetector",
            display_name: "Plateau Detector",
            category: "Synchronization",
            description: "Detects flat regions (plateaus) in a signal and marks their midpoints. Used in OFDM Schmidl-Cox synchronization where the timing metric produces a plateau rather than a peak.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/plateau_detector.rs",
                line: 39,
                symbol: "PlateauDetector",
                description: "Plateau detection with configurable threshold and minimum width",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Plateau Detection",
                    plaintext: "detect: x[n] >= threshold for >= min_width consecutive samples",
                    latex: Some("\\text{plateau} = \\{n : x[n] \\geq T, \\text{width} \\geq W_{\\min}\\}"),
                    variables: &[("T", "Detection threshold"), ("W_min", "Minimum plateau width in samples")],
                },
            ],
            tests: &[
                BlockTest { name: "test_basic_plateau", module: "r4w_core::plateau_detector::tests", description: "Detect single plateau", expected_runtime_ms: 5 },
                BlockTest { name: "test_multiple_plateaus", module: "r4w_core::plateau_detector::tests", description: "Multiple plateau detection", expected_runtime_ms: 5 },
                BlockTest { name: "test_too_narrow", module: "r4w_core::plateau_detector::tests", description: "Reject narrow peaks", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "Schmidl & Cox, IEEE Trans. Comm., 1997", section: Some("Robust Frequency and Timing Synchronization for OFDM"), url: None },
            ],
            related_blocks: &["BurstDetector", "AccessCodeDetector", "PreambleInsert"],
            input_type: "Real (timing metric)",
            output_type: "Real (binary: 1 at midpoints)",
            key_parameters: &["threshold", "min_width"],
        });

    // Batch 15: BinarySlicer, HdlcDeframer, ClockRecoveryMm, NbfmReceiver, SymbolSync
    m.insert("BinarySlicer", BlockMetadata {
            block_type: "BinarySlicer",
            display_name: "Binary Slicer",
            category: "Recovery",
            description: "Converts soft floating-point symbols to hard binary decisions using a configurable threshold. Essential for packet radio, APRS, and AX.25.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/binary_slicer.rs",
                line: 22,
                symbol: "BinarySlicer",
                description: "Threshold-based soft-to-hard decision",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Binary Decision",
                    plaintext: "bit = 1 if x >= threshold, else 0",
                    latex: Some("b = \\begin{cases} 1 & x \\geq T \\\\ 0 & x < T \\end{cases}"),
                    variables: &[("T", "Decision threshold"), ("x", "Soft input sample")],
                },
            ],
            tests: &[
                BlockTest { name: "test_bipolar_basic", module: "r4w_core::binary_slicer::tests", description: "Bipolar decision", expected_runtime_ms: 5 },
                BlockTest { name: "test_noisy_signal", module: "r4w_core::binary_slicer::tests", description: "Noisy signal decisions", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["QuadratureDemod", "ClockRecoveryMm", "HdlcDeframer"],
            input_type: "Real (soft symbols)",
            output_type: "Bits",
            key_parameters: &["threshold"],
        });

    m.insert("HdlcDeframer", BlockMetadata {
            block_type: "HdlcDeframer",
            display_name: "HDLC Deframer",
            category: "Coding",
            description: "Decodes HDLC frames from a bit stream. Performs flag detection (0x7E), bit-unstuffing, and CRC-16/CCITT verification. Used by AX.25, APRS, and satellite protocols.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/hdlc.rs",
                line: 150,
                symbol: "HdlcDeframer",
                description: "HDLC frame decoder with bit-unstuffing and FCS",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "HDLC Frame",
                    plaintext: "Flag(7E) | Data (bit-stuffed) | FCS(CRC-16) | Flag(7E)",
                    latex: None,
                    variables: &[("7E", "Flag byte 01111110"), ("FCS", "Frame Check Sequence CRC-16/CCITT")],
                },
            ],
            tests: &[
                BlockTest { name: "test_frame_deframe_roundtrip", module: "r4w_core::hdlc::tests", description: "Encode/decode roundtrip", expected_runtime_ms: 5 },
                BlockTest { name: "test_ax25_like_payload", module: "r4w_core::hdlc::tests", description: "AX.25 packet decode", expected_runtime_ms: 5 },
                BlockTest { name: "test_corrupted_fcs", module: "r4w_core::hdlc::tests", description: "Reject bad FCS", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "ISO/IEC 13239:2002 HDLC", section: Some("Frame structure, bit-stuffing, FCS"), url: None },
                StandardReference { name: "AX.25 Link Access Protocol", section: Some("v2.2 §3"), url: None },
            ],
            related_blocks: &["BinarySlicer", "AccessCodeDetector", "CrcGenerator"],
            input_type: "Bits",
            output_type: "Bits (frame data)",
            key_parameters: &[],
        });

    m.insert("ClockRecoveryMm", BlockMetadata {
            block_type: "ClockRecoveryMm",
            display_name: "Clock Recovery (M&M)",
            category: "Recovery",
            description: "Mueller & Müller symbol timing recovery. Recovers symbol timing from oversampled signals using a decision-directed TED and PI loop filter.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/clock_recovery_mm.rs",
                line: 42,
                symbol: "ClockRecoveryMM",
                description: "M&M timing recovery with PI loop filter",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "M&M TED",
                    plaintext: "e[n] = y[n-1] * (y[n] - y[n-2])",
                    latex: Some("e[n] = y[n-1] \\cdot (y[n] - y[n-2])"),
                    variables: &[("e[n]", "Timing error"), ("y[n]", "Symbol sample at time n")],
                },
            ],
            tests: &[
                BlockTest { name: "test_alternating_symbols", module: "r4w_core::clock_recovery_mm::tests", description: "Recover alternating BPSK", expected_runtime_ms: 5 },
                BlockTest { name: "test_output_rate", module: "r4w_core::clock_recovery_mm::tests", description: "Correct decimation rate", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n/sps)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "Mueller & Müller, IEEE Trans. Comm., 1976", section: Some("Timing Recovery in Digital Synchronous Data Receivers"), url: None },
            ],
            related_blocks: &["TimingRecovery", "SymbolSync", "BinarySlicer"],
            input_type: "Real (oversampled)",
            output_type: "Real (symbol-rate)",
            key_parameters: &["sps", "loop_bw"],
        });

    m.insert("NbfmReceiver", BlockMetadata {
            block_type: "NbfmReceiver",
            display_name: "NBFM Receiver",
            category: "Recovery",
            description: "Narrowband FM receiver combining quadrature demodulation and de-emphasis filtering. Outputs demodulated audio/data from I/Q input.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/fm_receiver.rs",
                line: 37,
                symbol: "NbfmReceiver",
                description: "Composite NBFM receiver (discriminator + de-emphasis)",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "FM Discriminator",
                    plaintext: "y[n] = gain * arg(x[n] * conj(x[n-1]))",
                    latex: Some("y[n] = \\frac{f_s}{2\\pi \\Delta f} \\cdot \\arg(x[n] \\cdot x^*[n-1])"),
                    variables: &[("f_s", "Sample rate"), ("Delta_f", "Max FM deviation")],
                },
            ],
            tests: &[
                BlockTest { name: "test_nbfm_dc_input", module: "r4w_core::fm_receiver::tests", description: "DC input → zero output", expected_runtime_ms: 5 },
                BlockTest { name: "test_output_length_matches", module: "r4w_core::fm_receiver::tests", description: "1:1 output rate", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["QuadratureDemod", "DeEmphasis", "PowerSquelch"],
            input_type: "IQ samples",
            output_type: "Real (audio/data)",
            key_parameters: &["max_deviation", "sample_rate"],
        });

    m.insert("SymbolSync", BlockMetadata {
            block_type: "SymbolSync",
            display_name: "Symbol Sync",
            category: "Recovery",
            description: "Polyphase filterbank-based symbol synchronizer with configurable TED (Gardner, Zero-Crossing, or Mueller & Müller). Recovers optimal sampling instants from oversampled I/Q.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/symbol_sync.rs",
                line: 56,
                symbol: "SymbolSync",
                description: "PFB-based symbol sync with TED options",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Gardner TED",
                    plaintext: "e = Re{y_mid * conj(y_cur - y_prev)}",
                    latex: Some("e = \\Re\\{y_{\\text{mid}} \\cdot (y_n - y_{n-1})^*\\}"),
                    variables: &[("y_mid", "Midpoint sample"), ("y_n", "Current symbol"), ("y_{n-1}", "Previous symbol")],
                },
            ],
            tests: &[
                BlockTest { name: "test_alternating_bpsk", module: "r4w_core::symbol_sync::tests", description: "BPSK symbol recovery", expected_runtime_ms: 5 },
                BlockTest { name: "test_incremental_processing", module: "r4w_core::symbol_sync::tests", description: "Block-by-block processing", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n/sps)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "Gardner, IEEE Trans. Comm., 1986", section: Some("A BPSK/QPSK Timing-Error Detector for Sampled Receivers"), url: None },
            ],
            related_blocks: &["TimingRecovery", "ClockRecoveryMm", "FllBandEdge"],
            input_type: "IQ (oversampled)",
            output_type: "IQ (symbol-rate)",
            key_parameters: &["sps", "ted", "loop_bw"],
        });

    // === Batch 16 blocks ===

    m.insert("PfbClockSync", BlockMetadata {
            block_type: "PfbClockSync",
            display_name: "PFB Clock Sync",
            category: "Recovery",
            description: "Polyphase filterbank-based clock synchronizer. Uses matched filter and derivative filter for timing error detection. Industrial-grade approach from GNU Radio.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/pfb_clock_sync.rs",
                line: 35,
                symbol: "PfbClockSync",
                description: "PFB clock recovery with RRC matched filter",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Timing Error",
                    plaintext: "e = Re{d(t) * conj(y(t))}",
                    latex: Some("e = \\Re\\{d(t) \\cdot y^*(t)\\}"),
                    variables: &[("d(t)", "Derivative filter output"), ("y(t)", "Matched filter output")],
                },
            ],
            tests: &[
                BlockTest { name: "test_bpsk_signal", module: "r4w_core::pfb_clock_sync::tests", description: "BPSK timing recovery", expected_runtime_ms: 5 },
                BlockTest { name: "test_qpsk_signal", module: "r4w_core::pfb_clock_sync::tests", description: "QPSK timing recovery", expected_runtime_ms: 5 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n * taps_per_arm)", memory: "O(nfilts * taps)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "fred harris, Multirate Signal Processing", section: Some("Ch. 9: Polyphase Filterbanks"), url: None },
            ],
            related_blocks: &["SymbolSync", "ClockRecoveryMm", "TimingRecovery"],
            input_type: "IQ (oversampled)",
            output_type: "IQ (symbol-rate)",
            key_parameters: &["sps", "loop_bw", "nfilts"],
        });

    m.insert("HeaderPayloadDemux", BlockMetadata {
            block_type: "HeaderPayloadDemux",
            display_name: "Header/Payload Demux",
            category: "Coding",
            description: "Variable-length packet demultiplexer. Parses a fixed-length header to extract payload length, then reads the declared number of payload bytes. Supports configurable header format, endianness, and length field position.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/header_payload_demux.rs",
                line: 110,
                symbol: "HeaderPayloadDemux",
                description: "Variable-length packet header/payload demux",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Payload Length Extraction",
                    plaintext: "len = header[offset..offset+size] (LE or BE)",
                    latex: None,
                    variables: &[("offset", "Byte offset of length field"), ("size", "Length field width (1/2/4 bytes)")],
                },
            ],
            tests: &[
                BlockTest { name: "test_simple_packet", module: "r4w_core::header_payload_demux::tests", description: "Basic packet demux", expected_runtime_ms: 1 },
                BlockTest { name: "test_incremental_feeding", module: "r4w_core::header_payload_demux::tests", description: "Streaming input", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(max_payload)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["HdlcDeframer", "Ax25Decoder", "AccessCodeDetector"],
            input_type: "Bytes",
            output_type: "Packets (header + payload)",
            key_parameters: &["header_len", "length_offset", "length_size", "big_endian"],
        });

    m.insert("Ax25Decoder", BlockMetadata {
            block_type: "Ax25Decoder",
            display_name: "AX.25 Decoder",
            category: "Coding",
            description: "AX.25 amateur radio protocol decoder. Parses callsigns (shifted ASCII), SSIDs, digipeater paths, control fields, PIDs, and information fields from HDLC frames. Used by APRS and packet radio.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/ax25.rs",
                line: 186,
                symbol: "Ax25Decoder",
                description: "AX.25 frame parser for amateur radio",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Address Decoding",
                    plaintext: "callsign[i] = raw[i] >> 1 (shifted ASCII)",
                    latex: None,
                    variables: &[("raw[i]", "Raw address byte"), ("callsign[i]", "ASCII character")],
                },
            ],
            tests: &[
                BlockTest { name: "test_decode_ui_frame", module: "r4w_core::ax25::tests", description: "Decode APRS UI frame", expected_runtime_ms: 1 },
                BlockTest { name: "test_decode_with_digipeaters", module: "r4w_core::ax25::tests", description: "Decode with digi path", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(frame_size)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "AX.25 Link Access Protocol v2.2", section: Some("Section 3: Frame Structure"), url: Some("https://www.ax25.net/AX25.2.2-Jul%2098-2.pdf") },
            ],
            related_blocks: &["HdlcDeframer", "HeaderPayloadDemux"],
            input_type: "Bytes (HDLC frame)",
            output_type: "Decoded AX.25 fields",
            key_parameters: &[],
        });

    m.insert("RmsPower", BlockMetadata {
            block_type: "RmsPower",
            display_name: "RMS Power",
            category: "Recovery",
            description: "RMS power measurement with exponential smoothing. Computes running RMS of complex or real signal. Can also normalize output to unit power (RMS AGC). Alpha controls averaging speed.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/rms.rs",
                line: 37,
                symbol: "RmsPower",
                description: "RMS power measurement with IIR smoothing",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Smoothed RMS",
                    plaintext: "rms^2[n] = alpha * |x[n]|^2 + (1-alpha) * rms^2[n-1]",
                    latex: Some("\\hat{P}_n = \\alpha |x_n|^2 + (1-\\alpha) \\hat{P}_{n-1}"),
                    variables: &[("alpha", "Smoothing factor (0..1)"), ("|x[n]|^2", "Instantaneous power")],
                },
            ],
            tests: &[
                BlockTest { name: "test_rms_constant_signal", module: "r4w_core::rms::tests", description: "Convergence to known RMS", expected_runtime_ms: 1 },
                BlockTest { name: "test_normalizer_converges", module: "r4w_core::rms::tests", description: "RMS normalization", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["Agc", "PowerSquelch", "SnrEstimator"],
            input_type: "IQ or Real",
            output_type: "Real (power level)",
            key_parameters: &["alpha"],
        });

    m.insert("CorrelateAndSync", BlockMetadata {
            block_type: "CorrelateAndSync",
            display_name: "Correlate & Sync",
            category: "Synchronization",
            description: "Cross-correlation-based frame synchronizer. Slides a known preamble/sync word over the input, normalizes by local power (CFAR), and detects peaks above threshold. Reports timing, correlation magnitude, and carrier phase.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/correlate_sync.rs",
                line: 47,
                symbol: "CorrelateSync",
                description: "Cross-correlation frame synchronizer",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "Normalized Cross-Correlation",
                    plaintext: "R(n) = |sum_k(x[n+k] * ref[k]*)| / sqrt(sum_k |x[n+k]|^2)",
                    latex: Some("R(n) = \\frac{|\\sum_k x[n+k] \\cdot r^*[k]|}{\\sqrt{\\sum_k |x[n+k]|^2}}"),
                    variables: &[("x[n]", "Input samples"), ("r[k]", "Reference preamble"), ("R(n)", "Normalized correlation")],
                },
            ],
            tests: &[
                BlockTest { name: "test_perfect_detection", module: "r4w_core::correlate_sync::tests", description: "Detect embedded preamble", expected_runtime_ms: 1 },
                BlockTest { name: "test_no_false_alarms", module: "r4w_core::correlate_sync::tests", description: "No detections in noise", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n * ref_len)", memory: "O(ref_len)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["AccessCodeDetector", "PlateauDetector", "PreambleInsert"],
            input_type: "IQ",
            output_type: "Real (correlation magnitude)",
            key_parameters: &["threshold", "min_spacing"],
        });

    // Batch 17: Rotator, Puncturer, Depuncturer, SymbolSlicer, FrameSync, VectorSink

        m.insert("Rotator", BlockMetadata {
            block_type: "Rotator",
            display_name: "Rotator (NCO Frequency Shift)",
            category: "Filtering",
            description: "NCO-based frequency shift block. Multiplies input signal by a complex \
                exponential to shift the spectrum by the specified frequency. Uses phase accumulation \
                with periodic wrapping for long-term numerical stability.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/rotator.rs",
                line: 26,
                symbol: "Rotator::process",
                description: "NCO-based frequency shift",
            }),
            related_code: &[],
            formulas: &[BlockFormula {
                name: "Frequency Shift",
                plaintext: "y[n] = x[n] * exp(j*phi[n]), phi[n] = phi[n-1] + 2*pi*f/fs",
                latex: Some("y[n] = x[n] \\cdot e^{j\\phi[n]}, \\quad \\phi[n] = \\phi[n-1] + 2\\pi f / f_s"),
                variables: &[("f", "Shift frequency (Hz)"), ("f_s", "Sample rate (Hz)"), ("phi", "Accumulated phase (rad)")],
            }],
            tests: &[
                BlockTest { name: "test_dc_shift", module: "r4w_core::rotator::tests", description: "Shifts DC signal to specified frequency", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["FreqXlatingFir", "CostasLoop", "SignalSource"],
            input_type: "IQ",
            output_type: "IQ",
            key_parameters: &["frequency_hz", "sample_rate_hz"],
        });

        m.insert("Puncturer", BlockMetadata {
            block_type: "Puncturer",
            display_name: "Puncturer (FEC Rate Adaptation)",
            category: "Coding",
            description: "Removes coded bits according to a puncture pattern to achieve higher \
                code rates from a base rate-1/2 convolutional code. Standard patterns provided \
                for rates 2/3, 3/4, 5/6, and 7/8.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/puncture.rs",
                line: 101,
                symbol: "Puncturer::puncture",
                description: "Puncture coded bits to increase effective code rate",
            }),
            related_code: &[],
            formulas: &[BlockFormula {
                name: "Puncture Rate",
                plaintext: "R_eff = k / n_kept, n_kept = sum(P[i,j])",
                latex: Some("R_{eff} = \\frac{k}{n_{kept}}, \\quad n_{kept} = \\sum P[i,j]"),
                variables: &[("P", "Puncture pattern matrix (rows x cols)"), ("k", "Information bits per period"), ("n_kept", "Coded bits kept per period")],
            }],
            tests: &[
                BlockTest { name: "test_rate_3_4_puncture", module: "r4w_core::puncture::tests", description: "Rate 3/4 puncture pattern", expected_runtime_ms: 1 },
                BlockTest { name: "test_roundtrip_hard", module: "r4w_core::puncture::tests", description: "Puncture/depuncture roundtrip", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(pattern_size)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[StandardReference {
                name: "IEEE 802.11a/g",
                section: Some("17.3.5.6"),
                url: Some("https://standards.ieee.org/standard/802_11a-1999.html"),
            }],
            related_blocks: &["Depuncturer", "FecEncoder", "Interleaver"],
            input_type: "Bits",
            output_type: "Bits",
            key_parameters: &["rate"],
        });

        m.insert("Depuncturer", BlockMetadata {
            block_type: "Depuncturer",
            display_name: "Depuncturer (FEC Rate Restoration)",
            category: "Coding",
            description: "Restores punctured positions by inserting erasure values where bits were \
                removed. Used before Viterbi decoding to mark low-confidence positions.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/puncture.rs",
                line: 197,
                symbol: "Depuncturer::depuncture",
                description: "Restore punctured positions with erasure values",
            }),
            related_code: &[],
            formulas: &[BlockFormula {
                name: "Depuncture Insertion",
                plaintext: "y[i] = x[k] if P[i%N]=1, else erasure",
                latex: Some("y[i] = \\begin{cases} x[k], & P[i \\bmod N] = 1 \\\\ \\text{erasure}, & P[i \\bmod N] = 0 \\end{cases}"),
                variables: &[("P", "Puncture pattern"), ("N", "Pattern period"), ("erasure", "Erasure value (e.g., 128 for unsigned, 0 for LLR)")],
            }],
            tests: &[
                BlockTest { name: "test_roundtrip_hard", module: "r4w_core::puncture::tests", description: "Depuncture roundtrip with erasures", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(pattern_size)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Puncturer", "FecEncoder", "Interleaver"],
            input_type: "Bits",
            output_type: "Bits",
            key_parameters: &["rate"],
        });

        m.insert("SymbolSlicer", BlockMetadata {
            block_type: "SymbolSlicer",
            display_name: "Symbol Slicer (Hard Decision)",
            category: "Recovery",
            description: "Minimum-distance hard decision device for digital constellations. Maps \
                received IQ samples to nearest constellation point. Supports BPSK, QPSK, 8PSK, \
                16QAM, and 64QAM. Also computes EVM (Error Vector Magnitude).",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/symbol_slicer.rs",
                line: 46,
                symbol: "SymbolSlicer::slice",
                description: "Minimum-distance hard decision",
            }),
            related_code: &[],
            formulas: &[BlockFormula {
                name: "Minimum Distance Decision",
                plaintext: "s_hat = argmin_{s_k in C} |r - s_k|^2",
                latex: Some("\\hat{s} = \\arg\\min_{s_k \\in \\mathcal{C}} |r - s_k|^2"),
                variables: &[("r", "Received sample (complex)"), ("s_k", "Constellation points"), ("C", "Constellation set")],
            },
            BlockFormula {
                name: "EVM",
                plaintext: "EVM = sqrt(1/N * sum|r_n - s_n|^2)",
                latex: Some("\\text{EVM} = \\sqrt{\\frac{1}{N} \\sum_{n=1}^{N} |r_n - s_n|^2}"),
                variables: &[("r_n", "Received sample"), ("s_n", "Nearest constellation point")],
            }],
            tests: &[
                BlockTest { name: "test_bpsk_slicer", module: "r4w_core::symbol_slicer::tests", description: "BPSK hard decision", expected_runtime_ms: 1 },
                BlockTest { name: "test_qpsk_slicer", module: "r4w_core::symbol_slicer::tests", description: "QPSK hard decision", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(M) per sample", memory: "O(M)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["ConstellationRx", "PskDemodulator", "QamDemodulator"],
            input_type: "IQ",
            output_type: "Symbols + IQ (nearest points)",
            key_parameters: &["modulation"],
        });

        m.insert("FrameSync", BlockMetadata {
            block_type: "FrameSync",
            display_name: "Frame Synchronizer",
            category: "Synchronization",
            description: "Detects frame boundaries in a bit stream using known sync word patterns. \
                Shifts each bit into a register and compares against the sync word using Hamming \
                distance. After detection, extracts fixed-length frames from the stream.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/frame_sync.rs",
                line: 106,
                symbol: "FrameSync::process",
                description: "Bit-level frame synchronizer with Hamming distance detection",
            }),
            related_code: &[],
            formulas: &[BlockFormula {
                name: "Hamming Distance Check",
                plaintext: "d_H(r, s) = sum(r_i XOR s_i) <= T",
                latex: Some("d_H(\\mathbf{r}, \\mathbf{s}) = \\sum_{i=0}^{L-1} r_i \\oplus s_i \\leq T"),
                variables: &[("r", "Shift register contents"), ("s", "Sync word pattern"), ("L", "Sync word length"), ("T", "Max Hamming distance threshold")],
            }],
            tests: &[
                BlockTest { name: "test_basic_sync", module: "r4w_core::frame_sync::tests", description: "Detect sync word in bit stream", expected_runtime_ms: 1 },
                BlockTest { name: "test_hamming_tolerance", module: "r4w_core::frame_sync::tests", description: "Detect with bit errors", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n * L)", memory: "O(L + frame_length)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["AccessCodeDetector", "CorrelateAndSync", "SyncWordInsert"],
            input_type: "Bits",
            output_type: "Bits (extracted frames)",
            key_parameters: &["sync_word_hex", "frame_length", "max_hamming"],
        });

        m.insert("VectorSink", BlockMetadata {
            block_type: "VectorSink",
            display_name: "Vector Sink (Data Capture)",
            category: "Output",
            description: "Terminal block that captures samples into memory for analysis, testing, \
                and signal inspection. Supports complex IQ, real-valued, and bit stream data. \
                Optional maximum capacity prevents unbounded memory growth.",
            implementation: Some(CodeLocation {
                crate_name: "r4w-core",
                file_path: "src/vector_sink.rs",
                line: 57,
                symbol: "VectorSinkComplex::process",
                description: "Capture complex IQ samples into memory",
            }),
            related_code: &[],
            formulas: &[],
            tests: &[
                BlockTest { name: "test_complex_sink_basic", module: "r4w_core::vector_sink::tests", description: "Basic capture test", expected_runtime_ms: 1 },
                BlockTest { name: "test_complex_sink_capacity", module: "r4w_core::vector_sink::tests", description: "Capacity limiting", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(capacity)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["IqOutput", "BitOutput", "FileOutput"],
            input_type: "Any (IQ, Real, Bits)",
            output_type: "Same as input (passthrough)",
            key_parameters: &["max_capacity"],
        });

    // Batch 18: Arithmetic, Conjugate, Phase, Transcendental, ChunksToSymbols

        m.insert("AddConst", BlockMetadata {
            block_type: "AddConst",
            display_name: "Add Constant",
            category: "Filtering",
            description: "Adds a complex constant to every sample. Used for DC offset correction, level shifting, and bias adjustment.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/arithmetic.rs", line: 67, symbol: "add_const_complex", description: "Add complex constant to stream" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Add Const", plaintext: "y[n] = x[n] + c", latex: Some("y[n] = x[n] + c"), variables: &[("c", "Complex constant"), ("x", "Input"), ("y", "Output")] }],
            tests: &[BlockTest { name: "test_add_const_complex", module: "r4w_core::arithmetic::tests", description: "Add complex constant", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["MultiplyConst", "Rotator"],
            input_type: "IQ",
            output_type: "IQ",
            key_parameters: &["re", "im"],
        });

        m.insert("Conjugate", BlockMetadata {
            block_type: "Conjugate",
            display_name: "Complex Conjugate",
            category: "Filtering",
            description: "Computes the complex conjugate of each sample: y[n] = conj(x[n]). Flips the sign of the imaginary component.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/conjugate.rs", line: 30, symbol: "conjugate", description: "Complex conjugate" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Conjugate", plaintext: "y[n] = conj(x[n]) = Re(x) - j*Im(x)", latex: Some("y[n] = x^*[n]"), variables: &[("x", "Input (complex)"), ("y", "Output (complex conjugate)")] }],
            tests: &[BlockTest { name: "test_conjugate", module: "r4w_core::conjugate::tests", description: "Complex conjugate", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["MultiplyConjugate", "Rotator"],
            input_type: "IQ",
            output_type: "IQ",
            key_parameters: &[],
        });

        m.insert("MultiplyConjugate", BlockMetadata {
            block_type: "MultiplyConjugate",
            display_name: "Multiply Conjugate",
            category: "Filtering",
            description: "Multiplies first stream by conjugate of second: y[n] = a[n] * conj(b[n]). Used in cross-correlation and decision-directed loops.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/conjugate.rs", line: 35, symbol: "multiply_conjugate", description: "Multiply by conjugate" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Multiply Conjugate", plaintext: "y[n] = a[n] * conj(b[n])", latex: Some("y[n] = a[n] \\cdot b^*[n]"), variables: &[("a", "First input"), ("b", "Second input")] }],
            tests: &[BlockTest { name: "test_multiply_conjugate", module: "r4w_core::conjugate::tests", description: "Multiply conjugate", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Conjugate", "MultiplyConst"],
            input_type: "IQ",
            output_type: "IQ",
            key_parameters: &[],
        });

        m.insert("PhaseUnwrap", BlockMetadata {
            block_type: "PhaseUnwrap",
            display_name: "Phase Unwrap",
            category: "Recovery",
            description: "Reconstructs continuous phase trajectory from wrapped [-pi, pi] values. When the difference between consecutive samples exceeds pi, a 2*pi correction is applied.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/phase_ops.rs", line: 56, symbol: "phase_unwrap", description: "Phase unwrapping" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Phase Unwrap", plaintext: "y[n] = x[n] + 2*pi*k, k chosen to minimize |y[n]-y[n-1]|", latex: Some("y[n] = x[n] + 2\\pi k_n"), variables: &[("x", "Wrapped phase"), ("k_n", "Cumulative correction"), ("y", "Unwrapped phase")] }],
            tests: &[BlockTest { name: "test_phase_unwrap_positive_jump", module: "r4w_core::phase_ops::tests", description: "Unwrap positive jump", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["CostasLoop", "CarrierRecovery"],
            input_type: "Real (phase)",
            output_type: "Real (unwrapped phase)",
            key_parameters: &[],
        });

        m.insert("Normalize", BlockMetadata {
            block_type: "Normalize",
            display_name: "Signal Normalize",
            category: "Recovery",
            description: "Normalizes signal to unit power, unit RMS, or unit peak amplitude. Useful for AGC-like scaling and pre-processing.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/transcendental.rs", line: 152, symbol: "normalize_power", description: "Power normalization" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Power Normalize", plaintext: "y[n] = x[n] / sqrt(avg(|x|^2))", latex: Some("y[n] = \\frac{x[n]}{\\sqrt{\\bar{P}}}"), variables: &[("P_bar", "Average power"), ("x", "Input"), ("y", "Normalized output")] }],
            tests: &[BlockTest { name: "test_normalize_power", module: "r4w_core::transcendental::tests", description: "Normalize to unit power", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["Agc", "RmsPower"],
            input_type: "IQ",
            output_type: "IQ",
            key_parameters: &["mode"],
        });

        m.insert("ChunksToSymbols", BlockMetadata {
            block_type: "ChunksToSymbols",
            display_name: "Chunks to Symbols (LUT Mapper)",
            category: "Mapping",
            description: "Maps integer symbol indices to constellation points using a look-up table. Supports BPSK, QPSK, 8PSK, and 16QAM. Also maps bit streams by packing bits_per_symbol bits per index.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/chunks_to_symbols.rs", line: 103, symbol: "ChunksToSymbols::map", description: "LUT symbol mapping" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "LUT Mapping", plaintext: "y[n] = table[idx[n]]", latex: Some("y[n] = \\mathcal{C}[i_n]"), variables: &[("C", "Constellation table"), ("i_n", "Symbol index"), ("y", "Mapped point")] }],
            tests: &[BlockTest { name: "test_qpsk_map", module: "r4w_core::chunks_to_symbols::tests", description: "QPSK mapping", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(M)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["ConstellationMapper", "SymbolSlicer", "SymbolsToSoftBits"],
            input_type: "Symbols",
            output_type: "IQ",
            key_parameters: &["modulation"],
        });

        m.insert("SymbolsToSoftBits", BlockMetadata {
            block_type: "SymbolsToSoftBits",
            display_name: "Symbols to Soft Bits (LLR Demapper)",
            category: "Mapping",
            description: "Computes approximate log-likelihood ratio (LLR) soft decisions from received constellation points. Min-distance approximation over constellation subsets for each bit position.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/chunks_to_symbols.rs", line: 189, symbol: "SymbolsToSoftBits::demap", description: "Soft-decision LLR demapping" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Approximate LLR", plaintext: "LLR_k = (min_dist_1^2 - min_dist_0^2) / sigma^2", latex: Some("\\Lambda_k = \\frac{\\min_{s:b_k=1} |r-s|^2 - \\min_{s:b_k=0} |r-s|^2}{\\sigma^2}"), variables: &[("r", "Received sample"), ("s", "Constellation points"), ("sigma^2", "Noise variance"), ("b_k", "Bit position k")] }],
            tests: &[BlockTest { name: "test_soft_bits_bpsk_noiseless", module: "r4w_core::chunks_to_symbols::tests", description: "BPSK soft demapping", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n * M * k)", memory: "O(M)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[],
            related_blocks: &["ChunksToSymbols", "SymbolSlicer", "ConstellationRx"],
            input_type: "IQ",
            output_type: "Real (LLR values)",
            key_parameters: &["modulation", "noise_var"],
        });

        m.insert("Transcendental", BlockMetadata {
            block_type: "Transcendental",
            display_name: "Transcendental (Math Ops)",
            category: "Filtering",
            description: "Element-wise mathematical operations: Abs (magnitude), Power (dB), Ln (natural log), Exp (exponential). Also includes dB conversions and normalization.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/transcendental.rs", line: 38, symbol: "abs_complex", description: "Transcendental math operations" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Abs (Magnitude)", plaintext: "|x[n]| = sqrt(Re^2 + Im^2)", latex: Some("|x[n]| = \\sqrt{x_I^2 + x_Q^2}"), variables: &[("x_I", "Real component"), ("x_Q", "Imaginary component")] },
                BlockFormula { name: "Power dB", plaintext: "P_dB = 10*log10(|x|^2)", latex: Some("P_{dB} = 10\\log_{10}(|x|^2)"), variables: &[("x", "Input sample"), ("P_dB", "Power in dB")] },
            ],
            tests: &[BlockTest { name: "test_abs_complex", module: "r4w_core::transcendental::tests", description: "Complex magnitude", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["ComplexToMag", "LogPowerFft"],
            input_type: "Any (IQ or Real)",
            output_type: "Real",
            key_parameters: &["function"],
        });

        // ===== BATCH 20: HW Impairments, Stream/Vector, Tag Debug, Sample-and-Hold, Null Sink/Source =====

        m.insert("PhaseNoiseBlock", BlockMetadata {
            block_type: "PhaseNoiseBlock",
            display_name: "Phase Noise",
            category: "Impairments",
            description: "Adds filtered random phase jitter to simulate oscillator phase noise. Uses first-order IIR filtering for 1/f spectral shaping. Preserves signal envelope (constant modulus).",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/hw_impairments.rs", line: 35, symbol: "PhaseNoiseGenerator", description: "Phase noise generator" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Phase Noise", plaintext: "y[n] = x[n] * exp(j*φ_noise[n])", latex: Some("y[n] = x[n] \\cdot e^{j\\phi_{noise}[n]}"), variables: &[("φ_noise", "Filtered random phase"), ("x[n]", "Input signal")] },
            ],
            tests: &[BlockTest { name: "test_phase_noise_preserves_envelope", module: "r4w_core::hw_impairments::tests", description: "Envelope preservation", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["IqImbalanceBlock", "DcOffsetBlock", "AwgnChannel"],
            input_type: "IQ",
            output_type: "IQ (phase-noisy)",
            key_parameters: &["magnitude_db", "alpha"],
        });

        m.insert("IqImbalanceBlock", BlockMetadata {
            block_type: "IqImbalanceBlock",
            display_name: "IQ Imbalance",
            category: "Impairments",
            description: "Models receiver IQ gain and phase mismatch. Creates image spur at mirror frequency. Gain error: differential amplification of Q vs I. Phase error: imperfect 90-degree LO splitting.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/hw_impairments.rs", line: 110, symbol: "IqImbalanceGenerator", description: "IQ imbalance model" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "IQ Imbalance", plaintext: "I' = I, Q' = g*(Q*cos(φ) + I*sin(φ))", latex: Some("Q' = g(Q\\cos\\phi + I\\sin\\phi)"), variables: &[("g", "Gain error"), ("φ", "Phase error (rad)")] },
            ],
            tests: &[BlockTest { name: "test_iq_imbalance_identity", module: "r4w_core::hw_impairments::tests", description: "Zero imbalance = identity", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["PhaseNoiseBlock", "DcOffsetBlock"],
            input_type: "IQ",
            output_type: "IQ (imbalanced)",
            key_parameters: &["magnitude_db", "phase_deg"],
        });

        m.insert("DcOffsetBlock", BlockMetadata {
            block_type: "DcOffsetBlock",
            display_name: "DC Offset",
            category: "Impairments",
            description: "Adds constant DC bias to I and/or Q channels. Models LO leakage or ADC offset. Creates a spike at DC in the spectrum.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/hw_impairments.rs", line: 162, symbol: "DcOffset", description: "DC offset generator" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "DC Offset", plaintext: "y[n] = x[n] + (d_I + j*d_Q)", latex: None, variables: &[("d_I", "I-channel offset"), ("d_Q", "Q-channel offset")] },
            ],
            tests: &[BlockTest { name: "test_dc_offset_basic", module: "r4w_core::hw_impairments::tests", description: "Basic DC offset", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["PhaseNoiseBlock", "IqImbalanceBlock", "DcBlocker"],
            input_type: "IQ",
            output_type: "IQ (with DC bias)",
            key_parameters: &["i_offset", "q_offset"],
        });

        m.insert("StreamToVectorBlock", BlockMetadata {
            block_type: "StreamToVectorBlock",
            display_name: "Stream to Vector",
            category: "Filtering",
            description: "Collects N scalar samples into fixed-size vectors. Essential before FFT-based processing (OFDM, spectral analysis). Buffers partial blocks across calls.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/stream_to_vector.rs", line: 32, symbol: "StreamToVector", description: "Stream to vector conversion" }),
            related_code: &[CodeLocation { crate_name: "r4w-core", file_path: "src/stream_to_vector.rs", line: 88, symbol: "Interleave", description: "Stream interleaver" }],
            formulas: &[],
            tests: &[BlockTest { name: "test_stream_to_vector_basic", module: "r4w_core::stream_to_vector::tests", description: "Basic S2V", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(vector_size)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["LogPowerFft", "OfdmModulator"],
            input_type: "Real",
            output_type: "Real (vectors of size N)",
            key_parameters: &["vector_size"],
        });

        m.insert("QuantizerBlock", BlockMetadata {
            block_type: "QuantizerBlock",
            display_name: "Quantizer",
            category: "Filtering",
            description: "Maps continuous signal values to discrete quantization levels. Simulates ADC resolution effects. N-bit quantizer creates 2^N levels in range [-1, 1].",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/sample_and_hold.rs", line: 73, symbol: "Quantizer", description: "Signal quantizer" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Quantization", plaintext: "y = min + (floor((x-min)/step) + 0.5) * step", latex: None, variables: &[("step", "(max-min)/levels"), ("levels", "2^bits")] },
            ],
            tests: &[BlockTest { name: "test_quantizer_4_levels", module: "r4w_core::sample_and_hold::tests", description: "4-level quantization", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["RailBlock", "SampleAndHoldBlock"],
            input_type: "Real",
            output_type: "Real (quantized)",
            key_parameters: &["bits"],
        });

        m.insert("SampleAndHoldBlock", BlockMetadata {
            block_type: "SampleAndHoldBlock",
            display_name: "Sample & Hold",
            category: "Filtering",
            description: "Holds the last accepted value when control signal is false. Used for zero-order hold interpolation, AGC freezing during preamble, and squelch-driven value holding.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/sample_and_hold.rs", line: 26, symbol: "SampleAndHold", description: "Sample-and-hold block" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "S&H", plaintext: "y[n] = { x[n] if ctrl[n], y[n-1] otherwise }", latex: None, variables: &[("ctrl", "Control signal (bool)"), ("x", "Input"), ("y", "Output")] },
            ],
            tests: &[BlockTest { name: "test_sample_and_hold_basic", module: "r4w_core::sample_and_hold::tests", description: "Basic S&H", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["QuantizerBlock", "ValveBlock"],
            input_type: "Real",
            output_type: "Real (held)",
            key_parameters: &[],
        });

        m.insert("NullSourceBlock", BlockMetadata {
            block_type: "NullSourceBlock",
            display_name: "Null Source",
            category: "Source",
            description: "Generates zero-valued samples. Used as a placeholder source, for testing pipeline plumbing, or to create silence/baseline signals.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/null_sink_source.rs", line: 31, symbol: "NullSource", description: "Zero-valued sample generator" }),
            related_code: &[CodeLocation { crate_name: "r4w-core", file_path: "src/null_sink_source.rs", line: 66, symbol: "NullSink", description: "Sample consumer (bit bucket)" }],
            formulas: &[],
            tests: &[BlockTest { name: "test_null_source_complex", module: "r4w_core::null_sink_source::tests", description: "Complex zeros", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["VectorSourceBlock", "NoiseSource"],
            input_type: "None (source)",
            output_type: "IQ (zeros)",
            key_parameters: &[],
        });

        m.insert("VectorSourceBlock", BlockMetadata {
            block_type: "VectorSourceBlock",
            display_name: "Vector Source",
            category: "Source",
            description: "Outputs a predefined test signal (tone, impulse, or step). Useful for testing pipelines with known inputs. Supports repeat mode for continuous generation.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/null_sink_source.rs", line: 94, symbol: "VectorSource", description: "Predefined vector source" }),
            related_code: &[],
            formulas: &[],
            tests: &[BlockTest { name: "test_vector_source_repeat", module: "r4w_core::null_sink_source::tests", description: "Repeating output", expected_runtime_ms: 1 }],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(data_len)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["NullSourceBlock", "NoiseSource"],
            input_type: "None (source)",
            output_type: "IQ (test pattern)",
            key_parameters: &["pattern"],
        });

        // ===== BATCH 19: FM, Peak Detector, Selector/Valve/Mute/Rail/Threshold =====

        m.insert("FrequencyModulatorBlock", BlockMetadata {
            block_type: "FrequencyModulatorBlock",
            display_name: "FM Modulator",
            category: "Modulation",
            description: "Continuous-phase frequency modulator. Converts a real-valued baseband signal to FM IQ output with configurable sensitivity (max frequency deviation). Factory methods for NBFM (5 kHz) and WBFM (75 kHz).",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/frequency_modulator.rs", line: 36, symbol: "FrequencyModulator", description: "Continuous-phase FM modulator" }),
            related_code: &[CodeLocation { crate_name: "r4w-core", file_path: "src/frequency_modulator.rs", line: 113, symbol: "FskModulator", description: "M-ary FSK modulator" }],
            formulas: &[
                BlockFormula { name: "FM Phase", plaintext: "φ[n] = φ[n-1] + 2π·k·x[n]/fs", latex: Some("\\phi[n] = \\phi[n-1] + \\frac{2\\pi k\\, x[n]}{f_s}"), variables: &[("k", "Sensitivity (Hz/unit)"), ("x[n]", "Input signal"), ("fs", "Sample rate")] },
                BlockFormula { name: "FM Output", plaintext: "y[n] = exp(j·φ[n])", latex: Some("y[n] = e^{j\\phi[n]}"), variables: &[("φ[n]", "Accumulated phase")] },
            ],
            tests: &[
                BlockTest { name: "test_fm_constant_envelope", module: "r4w_core::frequency_modulator::tests", description: "Constant envelope verification", expected_runtime_ms: 1 },
                BlockTest { name: "test_fsk_binary", module: "r4w_core::frequency_modulator::tests", description: "Binary FSK modulation", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["FskModulator", "QuadratureDemod", "NbfmReceiver"],
            input_type: "Real (baseband audio/data)",
            output_type: "IQ (constant envelope)",
            key_parameters: &["sensitivity_hz", "sample_rate_hz"],
        });

        m.insert("PeakDetectorBlock", BlockMetadata {
            block_type: "PeakDetectorBlock",
            display_name: "Peak Detector",
            category: "Synchronization",
            description: "Detects peaks in real-valued signals with configurable threshold and minimum spacing. Uses parabolic interpolation for sub-sample peak position. Useful for timing recovery, sync detection, and spectral peaks.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/peak_detector.rs", line: 37, symbol: "PeakDetector", description: "Peak detector with threshold and min spacing" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Peak Condition", plaintext: "x[n] > threshold AND x[n] >= x[n-1] AND x[n] >= x[n+1]", latex: None, variables: &[("x[n]", "Input signal"), ("threshold", "Minimum peak value")] },
                BlockFormula { name: "Parabolic Interpolation", plaintext: "δ = (y[-1] - y[+1]) / (2·(2·y[0] - y[-1] - y[+1]))", latex: Some("\\delta = \\frac{y_{-1} - y_{+1}}{2(2y_0 - y_{-1} - y_{+1})}"), variables: &[("δ", "Fractional offset"), ("y", "Samples around peak")] },
            ],
            tests: &[
                BlockTest { name: "test_peak_detector_basic", module: "r4w_core::peak_detector::tests", description: "Basic peak detection", expected_runtime_ms: 1 },
                BlockTest { name: "test_peak_detector_min_spacing", module: "r4w_core::peak_detector::tests", description: "Minimum spacing enforcement", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["ThresholdDetector", "CorrelateAndSync", "PlateauDetector"],
            input_type: "Real",
            output_type: "Real (sparse peaks)",
            key_parameters: &["threshold", "min_spacing"],
        });

        m.insert("IntegrateAndDumpBlock", BlockMetadata {
            block_type: "IntegrateAndDumpBlock",
            display_name: "Integrate & Dump",
            category: "Filtering",
            description: "Accumulates N samples then outputs the sum or average and resets. Acts as a matched filter for rectangular (NRZ) pulses. Commonly used in timing recovery and energy detection.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/peak_detector.rs", line: 132, symbol: "IntegrateAndDump", description: "Integrate-and-dump matched filter" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Sum Mode", plaintext: "y[k] = Σ x[kN+i] for i=0..N-1", latex: Some("y[k] = \\sum_{i=0}^{N-1} x[kN+i]"), variables: &[("N", "Integration length"), ("x", "Input"), ("y", "Output")] },
                BlockFormula { name: "Average Mode", plaintext: "y[k] = (1/N) · Σ x[kN+i]", latex: Some("y[k] = \\frac{1}{N} \\sum_{i=0}^{N-1} x[kN+i]"), variables: &[("N", "Integration length")] },
            ],
            tests: &[
                BlockTest { name: "test_integrate_and_dump_sum", module: "r4w_core::peak_detector::tests", description: "Sum accumulation", expected_runtime_ms: 1 },
                BlockTest { name: "test_integrate_and_dump_average", module: "r4w_core::peak_detector::tests", description: "Average accumulation", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["MovingAverage", "KeepOneInN", "PeakDetectorBlock"],
            input_type: "Real",
            output_type: "Real (decimated by N)",
            key_parameters: &["length", "average"],
        });

        m.insert("RailBlock", BlockMetadata {
            block_type: "RailBlock",
            display_name: "Rail (Clamp)",
            category: "Impairments",
            description: "Clamps signal amplitude to [-max, max]. Commonly used to limit signal swing for DAC output or to prevent overflow. For complex signals, clamps real and imaginary parts independently.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/selector.rs", line: 207, symbol: "Rail", description: "Signal amplitude clamp" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Rail", plaintext: "y[n] = clamp(x[n], -max, max)", latex: None, variables: &[("max", "Maximum amplitude"), ("x[n]", "Input")] },
            ],
            tests: &[
                BlockTest { name: "test_rail_unit", module: "r4w_core::selector::tests", description: "Unit rail [-1, 1]", expected_runtime_ms: 1 },
                BlockTest { name: "test_rail_complex", module: "r4w_core::selector::tests", description: "Complex rail", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["MuteBlock", "Agc"],
            input_type: "Real",
            output_type: "Real (clamped)",
            key_parameters: &["max_amplitude"],
        });

        m.insert("ThresholdDetector", BlockMetadata {
            block_type: "ThresholdDetector",
            display_name: "Threshold Detector",
            category: "Synchronization",
            description: "Binary threshold detector — outputs 1.0 if input exceeds threshold, else 0.0. Used for energy detection, squelch triggering, and digital decision making.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/selector.rs", line: 252, symbol: "Threshold", description: "Binary threshold detector" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Threshold", plaintext: "y[n] = { 1.0 if x[n] > threshold, 0.0 otherwise }", latex: Some("y[n] = \\begin{cases} 1 & x[n] > \\tau \\\\ 0 & \\text{otherwise} \\end{cases}"), variables: &[("τ", "Threshold value"), ("x[n]", "Input signal")] },
            ],
            tests: &[
                BlockTest { name: "test_threshold_basic", module: "r4w_core::selector::tests", description: "Basic threshold", expected_runtime_ms: 1 },
                BlockTest { name: "test_threshold_detect", module: "r4w_core::selector::tests", description: "Boolean detection", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["BinarySlicer", "PeakDetectorBlock", "PowerSquelch"],
            input_type: "Real",
            output_type: "Real (0.0 or 1.0)",
            key_parameters: &["threshold"],
        });

        m.insert("MuteBlock", BlockMetadata {
            block_type: "MuteBlock",
            display_name: "Mute",
            category: "Impairments",
            description: "Replaces samples with zeros when muted. Unlike Valve which drops samples entirely, Mute preserves timing by outputting zero-valued samples.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/selector.rs", line: 169, symbol: "Mute", description: "Zero-fill mute block" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Mute", plaintext: "y[n] = { 0 if muted, x[n] if not muted }", latex: None, variables: &[("x[n]", "Input"), ("muted", "Mute state")] },
            ],
            tests: &[
                BlockTest { name: "test_mute_active", module: "r4w_core::selector::tests", description: "Muted → zeros", expected_runtime_ms: 1 },
                BlockTest { name: "test_mute_inactive", module: "r4w_core::selector::tests", description: "Unmuted → pass through", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["ValveBlock", "RailBlock"],
            input_type: "IQ",
            output_type: "IQ (zeros when muted)",
            key_parameters: &["muted"],
        });

        m.insert("ValveBlock", BlockMetadata {
            block_type: "ValveBlock",
            display_name: "Valve",
            category: "Impairments",
            description: "Gates a stream on/off. When open, samples pass through unchanged. When closed, output is empty (no samples). Useful for dynamic stream control.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/selector.rs", line: 120, symbol: "Valve", description: "Stream gate (on/off)" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Valve", plaintext: "y = { x if open, [] if closed }", latex: None, variables: &[("x", "Input stream"), ("open", "Valve state")] },
            ],
            tests: &[
                BlockTest { name: "test_valve_open", module: "r4w_core::selector::tests", description: "Open → pass through", expected_runtime_ms: 1 },
                BlockTest { name: "test_valve_closed", module: "r4w_core::selector::tests", description: "Closed → empty", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["MuteBlock", "PowerSquelch"],
            input_type: "IQ",
            output_type: "IQ (or empty)",
            key_parameters: &["open"],
        });

    // === Batch 21: PDU Conversion, OFDM Channel Est, SSB Modem, Wavelet ===

    m.insert("PDU to Tagged Stream", BlockMetadata {
            block_type: "PduToTaggedStreamBlock",
            display_name: "PDU to Tagged Stream",
            category: "Coding",
            description: "Converts PDU messages into a contiguous byte stream with packet_len tags at each packet boundary.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/pdu.rs", line: 69, symbol: "PduToTaggedStream", description: "PDU to tagged stream converter" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "Tag Insertion", plaintext: "tag[i] = (offset=sum(len[0..i]), length=len[i])", latex: None, variables: &[("len[i]", "PDU i length"), ("offset", "cumulative byte position")] }],
            tests: &[
                BlockTest { name: "test_pdu_to_tagged_stream_basic", module: "r4w_core::pdu::tests", description: "Two PDUs → tagged stream", expected_runtime_ms: 1 },
                BlockTest { name: "test_roundtrip_byte_pdus", module: "r4w_core::pdu::tests", description: "PDU roundtrip", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["TaggedStreamToPduBlock", "HeaderPayloadDemux"],
            input_type: "Bits (PDU bytes)",
            output_type: "Bits (tagged stream)",
            key_parameters: &["length_tag_key"],
        });

    m.insert("Tagged Stream to PDU", BlockMetadata {
            block_type: "TaggedStreamToPduBlock",
            display_name: "Tagged Stream to PDU",
            category: "Coding",
            description: "Collects samples from a tagged stream back into PDU messages using packet_len tags.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/pdu.rs", line: 123, symbol: "TaggedStreamToPdu", description: "Tagged stream to PDU collector" }),
            related_code: &[],
            formulas: &[BlockFormula { name: "PDU Collection", plaintext: "pdu[i] = stream[tag[i].offset .. tag[i].offset + tag[i].length]", latex: None, variables: &[("tag[i]", "packet boundary tag")] }],
            tests: &[
                BlockTest { name: "test_tagged_stream_to_pdu_basic", module: "r4w_core::pdu::tests", description: "Tagged stream → PDUs", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(max_pdu)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["PduToTaggedStreamBlock", "HeaderPayloadDemux"],
            input_type: "Bits (tagged stream)",
            output_type: "Bits (PDU bytes)",
            key_parameters: &["length_tag_key"],
        });

    m.insert("OFDM Channel Estimator", BlockMetadata {
            block_type: "OfdmChannelEstBlock",
            display_name: "OFDM Channel Estimator",
            category: "Recovery",
            description: "Pilot-based channel estimation and zero-forcing/MMSE equalization for OFDM systems.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/ofdm_channel_est.rs", line: 91, symbol: "OfdmChannelEstimator", description: "OFDM pilot-based channel estimator" }),
            related_code: &[
                CodeLocation { crate_name: "r4w-core", file_path: "src/ofdm.rs", line: 1, symbol: "ofdm", description: "OFDM modulator/demodulator" },
            ],
            formulas: &[
                BlockFormula { name: "LS Estimation", plaintext: "H_hat[k] = Y_pilot[k] / X_pilot[k]", latex: Some("\\hat{H}[k] = \\frac{Y_{pilot}[k]}{X_{pilot}[k]}"), variables: &[("Y_pilot", "received pilot"), ("X_pilot", "known pilot")] },
                BlockFormula { name: "ZF Equalization", plaintext: "X_hat[k] = Y[k] / H_hat[k]", latex: Some("\\hat{X}[k] = \\frac{Y[k]}{\\hat{H}[k]}"), variables: &[("Y", "received data"), ("H_hat", "channel estimate")] },
                BlockFormula { name: "MMSE Equalization", plaintext: "X_hat[k] = Y[k] * H*[k] / (|H[k]|^2 + sigma^2)", latex: Some("\\hat{X}[k] = Y[k] \\cdot \\frac{H^*[k]}{|H[k]|^2 + \\sigma^2}"), variables: &[("sigma^2", "noise variance")] },
            ],
            tests: &[
                BlockTest { name: "test_ls_estimation_identity_channel", module: "r4w_core::ofdm_channel_est::tests", description: "Identity channel", expected_runtime_ms: 1 },
                BlockTest { name: "test_zero_forcing_equalization", module: "r4w_core::ofdm_channel_est::tests", description: "ZF equalization", expected_runtime_ms: 1 },
                BlockTest { name: "test_mmse_equalization", module: "r4w_core::ofdm_channel_est::tests", description: "MMSE equalization", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(N)", memory: "O(N)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[
                StandardReference { name: "IEEE 802.11", section: Some("Pilot-based channel estimation"), url: None },
            ],
            related_blocks: &["OfdmModulator", "Equalizer"],
            input_type: "IQ (freq domain)",
            output_type: "IQ (equalized)",
            key_parameters: &["method", "averaging_alpha"],
        });

    m.insert("SSB Modulator", BlockMetadata {
            block_type: "SsbModulatorBlock",
            display_name: "SSB Modulator",
            category: "Modulation",
            description: "Single-sideband modulation using Hilbert transform (phasing method). USB or LSB.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/ssb_modem.rs", line: 122, symbol: "SsbModulator", description: "SSB modulator with Hilbert transform" }),
            related_code: &[
                CodeLocation { crate_name: "r4w-core", file_path: "src/ssb_modem.rs", line: 50, symbol: "HilbertTransform", description: "FIR Hilbert transform" },
            ],
            formulas: &[
                BlockFormula { name: "USB", plaintext: "z(t) = x(t) + j*H{x(t)}", latex: Some("z(t) = x(t) + j \\cdot \\mathcal{H}\\{x(t)\\}"), variables: &[("H{}", "Hilbert transform"), ("x(t)", "audio input")] },
                BlockFormula { name: "LSB", plaintext: "z(t) = x(t) - j*H{x(t)}", latex: Some("z(t) = x(t) - j \\cdot \\mathcal{H}\\{x(t)\\}"), variables: &[] },
            ],
            tests: &[
                BlockTest { name: "test_ssb_roundtrip", module: "r4w_core::ssb_modem::tests", description: "SSB modulate/demodulate roundtrip", expected_runtime_ms: 5 },
                BlockTest { name: "test_ssb_usb_vs_lsb_conjugate", module: "r4w_core::ssb_modem::tests", description: "USB/LSB conjugate relationship", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*K)", memory: "O(K)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "ITU Radio Regulations", section: Some("J3E emission designator"), url: None },
            ],
            related_blocks: &["SsbDemodulatorBlock", "FrequencyModulatorBlock"],
            input_type: "Real (audio)",
            output_type: "IQ",
            key_parameters: &["mode", "hilbert_order"],
        });

    m.insert("SSB Demodulator", BlockMetadata {
            block_type: "SsbDemodulatorBlock",
            display_name: "SSB Demodulator",
            category: "Recovery",
            description: "Recovers audio from SSB signal using product detection with DC removal.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/ssb_modem.rs", line: 152, symbol: "SsbDemodulator", description: "SSB demodulator with BFO support" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Product Detection", plaintext: "audio(t) = Re{z(t)}", latex: Some("audio(t) = \\text{Re}\\{z(t)\\}"), variables: &[("z(t)", "received baseband IQ")] },
            ],
            tests: &[
                BlockTest { name: "test_ssb_demodulator_basic", module: "r4w_core::ssb_modem::tests", description: "Basic demodulation", expected_runtime_ms: 1 },
                BlockTest { name: "test_ssb_demod_with_bfo", module: "r4w_core::ssb_modem::tests", description: "BFO demodulation", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["SsbModulatorBlock", "QuadratureDemod"],
            input_type: "IQ",
            output_type: "Real (audio)",
            key_parameters: &["mode"],
        });

    m.insert("DWT Analyzer", BlockMetadata {
            block_type: "DwtAnalyzerBlock",
            display_name: "DWT Analyzer",
            category: "Filtering",
            description: "Discrete Wavelet Transform decomposition for multi-resolution signal analysis.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/wavelet.rs", line: 161, symbol: "DwtAnalyzer", description: "Forward DWT decomposition" }),
            related_code: &[
                CodeLocation { crate_name: "r4w-core", file_path: "src/wavelet.rs", line: 230, symbol: "DwtSynthesizer", description: "Inverse DWT reconstruction" },
            ],
            formulas: &[
                BlockFormula { name: "Analysis", plaintext: "a[n] = Σk lo[k]*x[2n+k], d[n] = Σk hi[k]*x[2n+k]", latex: Some("a[n] = \\sum_k h_0[k] x[2n+k], \\quad d[n] = \\sum_k h_1[k] x[2n+k]"), variables: &[("a", "approximation"), ("d", "detail"), ("lo/hi", "filter bank")] },
            ],
            tests: &[
                BlockTest { name: "test_haar_roundtrip", module: "r4w_core::wavelet::tests", description: "Haar DWT roundtrip", expected_runtime_ms: 1 },
                BlockTest { name: "test_dwt_coefficient_structure", module: "r4w_core::wavelet::tests", description: "Coefficient structure", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*L)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "Mallat, A Wavelet Tour of Signal Processing", section: Some("Ch. 7: Wavelet Bases"), url: None },
            ],
            related_blocks: &["WaveletDenoiserBlock", "FirFilter"],
            input_type: "Real",
            output_type: "Real (coefficients)",
            key_parameters: &["wavelet", "levels"],
        });

    m.insert("Wavelet Denoiser", BlockMetadata {
            block_type: "WaveletDenoiserBlock",
            display_name: "Wavelet Denoiser",
            category: "Filtering",
            description: "Wavelet-domain denoising via soft/hard thresholding with automatic threshold estimation (MAD).",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/wavelet.rs", line: 293, symbol: "WaveletDenoiser", description: "DWT-based signal denoiser" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Universal Threshold", plaintext: "λ = σ * √(2*ln(N))", latex: Some("\\lambda = \\sigma \\sqrt{2 \\ln N}"), variables: &[("σ", "noise std (MAD estimate)"), ("N", "signal length")] },
                BlockFormula { name: "Soft Threshold", plaintext: "c_hat = sign(c) * max(|c| - λ, 0)", latex: Some("\\hat{c} = \\text{sgn}(c) \\cdot \\max(|c| - \\lambda, 0)"), variables: &[("c", "wavelet coefficient"), ("λ", "threshold")] },
            ],
            tests: &[
                BlockTest { name: "test_hard_threshold_denoising", module: "r4w_core::wavelet::tests", description: "Hard threshold denoising", expected_runtime_ms: 1 },
                BlockTest { name: "test_soft_threshold_denoising", module: "r4w_core::wavelet::tests", description: "Soft threshold denoising", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*L)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "Donoho & Johnstone (1994)", section: Some("Ideal spatial adaptation"), url: None },
            ],
            related_blocks: &["DwtAnalyzerBlock", "MovingAverage"],
            input_type: "Real",
            output_type: "Real (denoised)",
            key_parameters: &["wavelet", "levels", "method"],
        });

    // === Batch 22 ===

    m.insert("CPM Modulator", BlockMetadata {
            block_type: "CpmModulatorBlock",
            display_name: "CPM Modulator",
            category: "Modulation",
            description: "Continuous Phase Modulation — GMSK (GSM), GFSK (Bluetooth), MSK. Constant-envelope, smooth phase transitions for spectral efficiency.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/cpm.rs", line: 117, symbol: "CpmModulator", description: "CPM modulator with configurable phase pulse" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Instantaneous Phase", plaintext: "φ(t) = 2πh ∫ Σ_k a_k * g(t - kT) dt", latex: Some("\\varphi(t) = 2\\pi h \\int \\sum_k a_k g(t - kT) dt"), variables: &[("h", "modulation index"), ("a_k", "symbol"), ("g(t)", "frequency pulse"), ("T", "symbol period")] },
                BlockFormula { name: "Gaussian Pulse", plaintext: "g(t) = Q(2πBT(t-T/2)/√ln2) - Q(2πBT(t+T/2)/√ln2)", latex: Some("g(t) = Q\\left(\\frac{2\\pi BT(t-T/2)}{\\sqrt{\\ln 2}}\\right) - Q\\left(\\frac{2\\pi BT(t+T/2)}{\\sqrt{\\ln 2}}\\right)"), variables: &[("BT", "bandwidth-time product"), ("Q", "complementary Gaussian CDF")] },
            ],
            tests: &[
                BlockTest { name: "test_msk_constant_envelope", module: "r4w_core::cpm::tests", description: "MSK constant envelope verification", expected_runtime_ms: 1 },
                BlockTest { name: "test_msk_roundtrip", module: "r4w_core::cpm::tests", description: "MSK modulation/demodulation roundtrip", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*L*sps)", memory: "O(L*sps)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "3GPP TS 45.004", section: Some("GMSK for GSM"), url: None },
                StandardReference { name: "IEEE 802.15.1", section: Some("GFSK for Bluetooth"), url: None },
            ],
            related_blocks: &["CpmDemodulatorBlock", "FrequencyModulatorBlock"],
            input_type: "Bits",
            output_type: "IQ",
            key_parameters: &["cpm_type", "mod_index", "sps", "pulse_duration"],
        });

    m.insert("CPM Demodulator", BlockMetadata {
            block_type: "CpmDemodulatorBlock",
            display_name: "CPM Demodulator",
            category: "Recovery",
            description: "Non-coherent CPM demodulation using differential phase detection and integrate-and-dump.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/cpm.rs", line: 223, symbol: "CpmDemodulator", description: "CPM demodulator via quadrature detection" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Phase Difference", plaintext: "Δφ[n] = arg(x[n] * conj(x[n-1]))", latex: Some("\\Delta\\varphi[n] = \\arg(x[n] \\cdot x^*[n-1])"), variables: &[("x[n]", "received IQ sample")] },
            ],
            tests: &[
                BlockTest { name: "test_demod_reset", module: "r4w_core::cpm::tests", description: "Demodulator reset", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["CpmModulatorBlock", "QuadratureDemod"],
            input_type: "IQ",
            output_type: "Bits",
            key_parameters: &["cpm_type", "mod_index", "sps", "pulse_duration"],
        });

    m.insert("Dynamic Channel", BlockMetadata {
            block_type: "DynamicChannelBlock",
            display_name: "Dynamic Channel",
            category: "Impairments",
            description: "Composite time-varying channel: multipath fading + CFO drift + SRO drift + AWGN. Presets: Indoor, Urban, Vehicular, Satellite.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/dynamic_channel.rs", line: 281, symbol: "DynamicChannel", description: "Composite dynamic channel model" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Channel Output", plaintext: "y(t) = Σ_l h_l(t) * x(t-τ_l) * exp(j2πΔf*t) + n(t)", latex: Some("y(t) = \\sum_l h_l(t) x(t-\\tau_l) e^{j2\\pi\\Delta f \\cdot t} + n(t)"), variables: &[("h_l(t)", "time-varying tap gain"), ("τ_l", "tap delay"), ("Δf", "CFO drift"), ("n(t)", "AWGN")] },
            ],
            tests: &[
                BlockTest { name: "test_no_fading_passthrough", module: "r4w_core::dynamic_channel::tests", description: "No-impairment passthrough", expected_runtime_ms: 1 },
                BlockTest { name: "test_random_walk_bounded", module: "r4w_core::dynamic_channel::tests", description: "CFO/SRO drift bounds", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n*K)", memory: "O(K)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "3GPP TS 36.104", section: Some("EPA, EVA, ETU channel models"), url: None },
            ],
            related_blocks: &["AwgnChannel", "FadingChannel", "FrequencyOffset"],
            input_type: "IQ",
            output_type: "IQ",
            key_parameters: &["preset", "noise_amplitude"],
        });

    m.insert("Polar Encoder", BlockMetadata {
            block_type: "PolarEncoderBlock",
            display_name: "Polar Encoder",
            category: "Coding",
            description: "Polar code encoder (5G NR control channel FEC). Capacity-achieving codes with Bhattacharyya-based frozen bit selection.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/fec/polar.rs", line: 79, symbol: "PolarEncoder", description: "Polar code encoder with butterfly transform" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Polar Transform", plaintext: "x = u * F^⊗n, F = [[1,0],[1,1]]", latex: Some("\\mathbf{x} = \\mathbf{u} \\cdot F^{\\otimes n}, \\quad F = \\begin{bmatrix} 1 & 0 \\\\ 1 & 1 \\end{bmatrix}"), variables: &[("u", "input (info + frozen)"), ("F", "kernel matrix"), ("n", "log2(N)")] },
            ],
            tests: &[
                BlockTest { name: "test_polar_roundtrip_n8_k4", module: "r4w_core::fec::polar::tests", description: "N=8 K=4 encode/decode roundtrip", expected_runtime_ms: 1 },
                BlockTest { name: "test_polar_roundtrip_n16_k8", module: "r4w_core::fec::polar::tests", description: "N=16 K=8 roundtrip", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(N log N)", memory: "O(N)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[
                StandardReference { name: "3GPP TS 38.212", section: Some("Section 5.3.1 - Polar coding"), url: None },
                StandardReference { name: "Arikan (2009)", section: Some("Channel Polarization"), url: None },
            ],
            related_blocks: &["PolarDecoderBlock", "FecEncoder"],
            input_type: "Bits",
            output_type: "Bits",
            key_parameters: &["block_size", "info_bits"],
        });

    m.insert("Polar Decoder", BlockMetadata {
            block_type: "PolarDecoderBlock",
            display_name: "Polar Decoder",
            category: "Coding",
            description: "Polar code SC/SCL decoder. Successive cancellation decodes frozen and info bits sequentially.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/fec/polar.rs", line: 125, symbol: "PolarDecoder", description: "SC and SCL polar decoder" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "f function (min-sum)", plaintext: "f(a,b) = sign(a)*sign(b)*min(|a|,|b|)", latex: Some("f(a,b) = \\text{sgn}(a) \\cdot \\text{sgn}(b) \\cdot \\min(|a|,|b|)"), variables: &[("a,b", "LLR inputs")] },
                BlockFormula { name: "g function", plaintext: "g(a,b,u) = b + (1-2u)*a", latex: Some("g(a,b,u) = b + (1-2u) \\cdot a"), variables: &[("u", "partial sum bit")] },
            ],
            tests: &[
                BlockTest { name: "test_polar_soft_decode", module: "r4w_core::fec::polar::tests", description: "Soft LLR decoding", expected_runtime_ms: 1 },
                BlockTest { name: "test_polar_n32_k16", module: "r4w_core::fec::polar::tests", description: "N=32 K=16 roundtrip", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(N log N)", memory: "O(N log N)", simd_optimized: false, gpu_accelerable: true }),
            standards: &[
                StandardReference { name: "3GPP TS 38.212", section: Some("Section 5.3.1 - Polar coding"), url: None },
            ],
            related_blocks: &["PolarEncoderBlock", "FecEncoder"],
            input_type: "Bits (or soft LLR)",
            output_type: "Bits",
            key_parameters: &["block_size", "info_bits"],
        });

    m.insert("Burst Tagger", BlockMetadata {
            block_type: "BurstTaggerBlock",
            display_name: "Burst Tagger",
            category: "Synchronization",
            description: "Power-based burst detection with holdoff. Segments continuous IQ stream into tagged bursts for packet processing.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/burst_tagger.rs", line: 54, symbol: "BurstTagger", description: "Power-based burst segmentation" }),
            related_code: &[],
            formulas: &[
                BlockFormula { name: "Power Detection", plaintext: "P[n] = 10*log10(|x[n]|²)", latex: Some("P[n] = 10 \\log_{10}(|x[n]|^2)"), variables: &[("x[n]", "input sample"), ("P[n]", "power in dB")] },
            ],
            tests: &[
                BlockTest { name: "test_burst_tagger_single_burst", module: "r4w_core::burst_tagger::tests", description: "Single burst detection", expected_runtime_ms: 1 },
                BlockTest { name: "test_holdoff_extends_burst", module: "r4w_core::burst_tagger::tests", description: "Holdoff bridges short gaps", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(burst_len)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["BurstDetector", "PowerSquelch", "TaggedStreamMuxBlock"],
            input_type: "IQ",
            output_type: "IQ (tagged bursts)",
            key_parameters: &["threshold_db", "min_burst_len", "holdoff"],
        });

    m.insert("Tagged Stream Mux", BlockMetadata {
            block_type: "TaggedStreamMuxBlock",
            display_name: "Tagged Stream Mux",
            category: "Synchronization",
            description: "Multiplex multiple tagged burst streams into a single output stream.",
            implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/burst_tagger.rs", line: 168, symbol: "TaggedStreamMux", description: "Tagged stream multiplexer" }),
            related_code: &[],
            formulas: &[],
            tests: &[
                BlockTest { name: "test_tagged_stream_mux", module: "r4w_core::burst_tagger::tests", description: "Mux two bursts", expected_runtime_ms: 1 },
            ],
            performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
            standards: &[],
            related_blocks: &["BurstTaggerBlock", "StreamMux"],
            input_type: "IQ (multiple)",
            output_type: "IQ",
            key_parameters: &["num_inputs"],
        });

    // Batch 23: Feedforward AGC, Vector Insert/Remove, Cyclic Prefix, File Meta, PN Sync
    m.insert("Feedforward AGC", BlockMetadata {
        block_type: "FeedforwardAgcBlock",
        display_name: "Feedforward AGC",
        category: "Recovery",
        description: "Non-causal automatic gain control using a lookahead window. Computes gain from future samples for fast response without overshoot. Introduces a fixed delay equal to the window size.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/feedforward_agc.rs", line: 30, symbol: "FeedforwardAgc", description: "Feedforward AGC with lookahead" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Gain computation", plaintext: "gain[n] = ref / max(|x[n..n+W]|)", latex: None, variables: &[("ref", "target output amplitude"), ("W", "window size"), ("x", "input signal")] },
        ],
        tests: &[
            BlockTest { name: "test_constant_amplitude", module: "r4w_core::feedforward_agc::tests", description: "Normalizes constant input to reference", expected_runtime_ms: 1 },
            BlockTest { name: "test_varying_amplitude", module: "r4w_core::feedforward_agc::tests", description: "Handles amplitude transitions", expected_runtime_ms: 1 },
            BlockTest { name: "test_max_gain", module: "r4w_core::feedforward_agc::tests", description: "Gain clamped to max", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n*W)", memory: "O(W)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["Agc"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["window_size", "reference"],
    });

    m.insert("Vector Insert", BlockMetadata {
        block_type: "VectorInsertBlock",
        display_name: "Vector Insert",
        category: "Synchronization",
        description: "Periodically inserts pilot symbols, sync words, or training sequences into a sample stream. Essential for OFDM pilot-aided channel estimation and frame synchronization.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/vector_insert.rs", line: 24, symbol: "VectorInsert", description: "Periodic vector insertion" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Overhead", plaintext: "overhead = 1 + L_vec / period", latex: None, variables: &[("L_vec", "inserted vector length"), ("period", "insertion period in input samples")] },
        ],
        tests: &[
            BlockTest { name: "test_insert_remove_roundtrip", module: "r4w_core::vector_insert::tests", description: "Insert then remove preserves data", expected_runtime_ms: 1 },
            BlockTest { name: "test_offset_insertion", module: "r4w_core::vector_insert::tests", description: "Inserts at correct offset", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["VectorRemoveBlock", "PreambleInsert", "SyncWordInsert"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["period", "offset"],
    });

    m.insert("Vector Remove", BlockMetadata {
        block_type: "VectorRemoveBlock",
        display_name: "Vector Remove",
        category: "Synchronization",
        description: "Removes periodically inserted vectors from a sample stream. Inverse of Vector Insert — strips pilots or sync words at known positions.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/vector_insert.rs", line: 76, symbol: "VectorRemove", description: "Periodic vector removal" }),
        related_code: &[],
        formulas: &[],
        tests: &[
            BlockTest { name: "test_vector_remove_basic", module: "r4w_core::vector_insert::tests", description: "Strips inserted vectors", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["VectorInsertBlock"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["vector_len", "period", "offset"],
    });

    m.insert("CP Add", BlockMetadata {
        block_type: "CyclicPrefixAdderBlock",
        display_name: "CP Add",
        category: "Modulation",
        description: "Adds cyclic prefix (guard interval) to OFDM symbols by copying the tail of each symbol to its front. Absorbs multipath delay spread in OFDM systems.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/cyclic_prefix.rs", line: 28, symbol: "CyclicPrefixAdder", description: "OFDM cyclic prefix insertion" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "CP construction", plaintext: "y = [x[N-L:N], x[0:N]]", latex: None, variables: &[("N", "FFT size"), ("L", "CP length"), ("x", "OFDM symbol")] },
            BlockFormula { name: "Overhead", plaintext: "overhead = (N + L) / N", latex: None, variables: &[("N", "FFT size"), ("L", "CP length")] },
        ],
        tests: &[
            BlockTest { name: "test_add_remove_roundtrip", module: "r4w_core::cyclic_prefix::tests", description: "Add then remove CP is lossless", expected_runtime_ms: 1 },
            BlockTest { name: "test_cp_is_cyclic", module: "r4w_core::cyclic_prefix::tests", description: "CP matches symbol tail", expected_runtime_ms: 1 },
            BlockTest { name: "test_block_processing", module: "r4w_core::cyclic_prefix::tests", description: "Multi-symbol block processing", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(N+L)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[
            StandardReference { name: "IEEE 802.11a/g/n", section: Some("OFDM PHY"), url: None },
            StandardReference { name: "3GPP TS 36.211", section: Some("LTE OFDM"), url: None },
            StandardReference { name: "ETSI EN 300 744", section: Some("DVB-T"), url: None },
        ],
        related_blocks: &["CyclicPrefixRemoverBlock", "OfdmModulator"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["fft_size", "cp_length"],
    });

    m.insert("CP Remove", BlockMetadata {
        block_type: "CyclicPrefixRemoverBlock",
        display_name: "CP Remove",
        category: "Modulation",
        description: "Removes cyclic prefix from received OFDM symbols, recovering the original symbol for FFT demodulation.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/cyclic_prefix.rs", line: 81, symbol: "CyclicPrefixRemover", description: "OFDM cyclic prefix removal" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "CP removal", plaintext: "y = x[L:L+N]", latex: None, variables: &[("N", "FFT size"), ("L", "CP length")] },
        ],
        tests: &[
            BlockTest { name: "test_remove_cp_basic", module: "r4w_core::cyclic_prefix::tests", description: "Basic CP removal", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(N)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[
            StandardReference { name: "IEEE 802.11a/g/n", section: Some("OFDM PHY"), url: None },
            StandardReference { name: "3GPP TS 36.211", section: Some("LTE OFDM"), url: None },
        ],
        related_blocks: &["CyclicPrefixAdderBlock", "OfdmModulator"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["fft_size", "cp_length"],
    });

    m.insert("File Meta Source", BlockMetadata {
        block_type: "FileMetaSourceBlock",
        display_name: "File Meta Source",
        category: "Source",
        description: "Reads IQ samples from a self-describing file with embedded JSON metadata header. Supports cf64, cf32, and ci16 sample formats.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/file_meta.rs", line: 278, symbol: "FileMetaSource", description: "File meta IQ reader" }),
        related_code: &[],
        formulas: &[],
        tests: &[
            BlockTest { name: "test_file_roundtrip_cf64", module: "r4w_core::file_meta::tests", description: "cf64 file read/write roundtrip", expected_runtime_ms: 5 },
            BlockTest { name: "test_file_roundtrip_cf32", module: "r4w_core::file_meta::tests", description: "cf32 file read/write roundtrip", expected_runtime_ms: 5 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["FileMetaSinkBlock", "FileSource"],
        input_type: "None",
        output_type: "IQ",
        key_parameters: &["path", "data_type"],
    });

    m.insert("File Meta Sink", BlockMetadata {
        block_type: "FileMetaSinkBlock",
        display_name: "File Meta Sink",
        category: "Output",
        description: "Writes IQ samples to a self-describing file with JSON metadata header containing sample rate, data type, and custom properties.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/file_meta.rs", line: 209, symbol: "FileMetaSink", description: "File meta IQ writer" }),
        related_code: &[],
        formulas: &[],
        tests: &[
            BlockTest { name: "test_header_serialize_deserialize", module: "r4w_core::file_meta::tests", description: "Header roundtrip", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["FileMetaSourceBlock", "FileOutput"],
        input_type: "IQ",
        output_type: "None",
        key_parameters: &["path", "data_type", "sample_rate"],
    });

    m.insert("PN Correlator", BlockMetadata {
        block_type: "PnCorrelatorBlock",
        display_name: "PN Correlator",
        category: "Synchronization",
        description: "Correlates incoming signal against a known PN (Pseudo-Noise) sequence for synchronization and despreading. Uses LFSR-generated m-sequences or Gold codes.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/pn_sync.rs", line: 128, symbol: "PnCorrelator", description: "PN sequence correlator" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Normalized cross-correlation", plaintext: "R(tau) = sum(x[n+tau] * c[n]) / sqrt(E_x * E_c)", latex: None, variables: &[("x", "received signal"), ("c", "reference PN sequence"), ("tau", "offset"), ("E_x", "signal energy"), ("E_c", "reference energy")] },
            BlockFormula { name: "m-sequence period", plaintext: "P = 2^L - 1", latex: None, variables: &[("L", "LFSR register length"), ("P", "sequence period")] },
        ],
        tests: &[
            BlockTest { name: "test_correlator_autocorrelation", module: "r4w_core::pn_sync::tests", description: "Peak at zero lag", expected_runtime_ms: 1 },
            BlockTest { name: "test_correlator_offset_detection", module: "r4w_core::pn_sync::tests", description: "Detects embedded PN at offset", expected_runtime_ms: 1 },
            BlockTest { name: "test_correlator_noise", module: "r4w_core::pn_sync::tests", description: "Detection through noise", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n*P)", memory: "O(P)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[
            StandardReference { name: "IS-95 CDMA", section: None, url: None },
            StandardReference { name: "GPS ICD IS-GPS-200", section: Some("C/A code"), url: None },
            StandardReference { name: "ITU-R M.2012", section: Some("WCDMA"), url: None },
        ],
        related_blocks: &["DsssSpread", "CorrelateSync"],
        input_type: "IQ",
        output_type: "Real",
        key_parameters: &["length", "polynomial"],
    });

    m.insert("DTMF Decoder", BlockMetadata {
        block_type: "DtmfDecoderBlock",
        display_name: "DTMF Decoder",
        category: "Recovery",
        description: "Decodes DTMF (Dual-Tone Multi-Frequency) signals using a bank of 8 Goertzel filters. Detects the 4x4 grid of telephone tones (0-9, A-D, *, #) with twist checking and ambiguity rejection.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/dtmf.rs", line: 52, symbol: "DtmfDecoder", description: "DTMF tone decoder using Goertzel filters" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Goertzel Magnitude", plaintext: "|X(k)|^2 = s1^2 + s2^2 - 2*cos(2*pi*k/N)*s1*s2", latex: None, variables: &[("k", "frequency bin index"), ("N", "block size"), ("s1/s2", "Goertzel state variables")] },
        ],
        tests: &[
            BlockTest { name: "test_detect_all_keys", module: "r4w_core::dtmf::tests", description: "Detects all 16 DTMF keys", expected_runtime_ms: 5 },
            BlockTest { name: "test_generate_detect_roundtrip", module: "r4w_core::dtmf::tests", description: "Generate and detect roundtrip", expected_runtime_ms: 1 },
            BlockTest { name: "test_twist_detection", module: "r4w_core::dtmf::tests", description: "Rejects excessive twist", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[
            StandardReference { name: "ITU-T Q.23", section: Some("DTMF signaling"), url: None },
            StandardReference { name: "ITU-T Q.24", section: Some("Multi-frequency signaling"), url: None },
        ],
        related_blocks: &["GoertzelFilter"],
        input_type: "IQ",
        output_type: "Real",
        key_parameters: &["sample_rate", "block_size"],
    });

    m.insert("Noise Blanker", BlockMetadata {
        block_type: "NoiseBlankerBlock",
        display_name: "Noise Blanker",
        category: "Recovery",
        description: "Detects and blanks impulsive noise (ignition, power line, lightning) that appears as short-duration, high-amplitude spikes. Uses running average power with threshold-based detection and configurable blanking modes (Zero, Hold, Interpolate).",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/noise_blanker.rs", line: 25, symbol: "NoiseBlanker", description: "Impulse noise blanker" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Power Estimate", plaintext: "P_avg[n] = (1-alpha)*P_avg[n-1] + alpha*|x[n]|^2", latex: None, variables: &[("alpha", "smoothing factor"), ("P_avg", "running average power")] },
            BlockFormula { name: "Detection", plaintext: "blank if |x[n]|^2 > P_avg * threshold^2", latex: None, variables: &[("threshold", "detection multiplier")] },
        ],
        tests: &[
            BlockTest { name: "test_blanks_impulse", module: "r4w_core::noise_blanker::tests", description: "Blanks impulse noise", expected_runtime_ms: 1 },
            BlockTest { name: "test_passes_normal_signal", module: "r4w_core::noise_blanker::tests", description: "Passes normal signal unchanged", expected_runtime_ms: 1 },
            BlockTest { name: "test_hold_mode", module: "r4w_core::noise_blanker::tests", description: "Hold mode uses last good sample", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["ClipBlanker", "PowerSquelch"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["threshold", "blank_width"],
    });

    m.insert("Stream Add", BlockMetadata {
        block_type: "StreamAddBlock",
        display_name: "Stream Add",
        category: "Rate Conversion",
        description: "Element-wise addition of two complex IQ streams. Output[n] = A[n] + B[n]. Streams are truncated to the shorter length.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/stream_arithmetic.rs", line: 11, symbol: "StreamAdd", description: "Element-wise stream addition" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Addition", plaintext: "y[n] = a[n] + b[n]", latex: None, variables: &[("a", "first input stream"), ("b", "second input stream")] },
        ],
        tests: &[
            BlockTest { name: "test_stream_add", module: "r4w_core::stream_arithmetic::tests", description: "Element-wise complex addition", expected_runtime_ms: 1 },
            BlockTest { name: "test_add_multi", module: "r4w_core::stream_arithmetic::tests", description: "Multi-stream addition", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[],
        related_blocks: &["StreamSubtract", "Multiply"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &[],
    });

    m.insert("Stream Subtract", BlockMetadata {
        block_type: "StreamSubtractBlock",
        display_name: "Stream Subtract",
        category: "Rate Conversion",
        description: "Element-wise subtraction of two complex IQ streams. Output[n] = A[n] - B[n]. Streams are truncated to the shorter length.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/stream_arithmetic.rs", line: 29, symbol: "StreamSubtract", description: "Element-wise stream subtraction" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Subtraction", plaintext: "y[n] = a[n] - b[n]", latex: None, variables: &[("a", "first input stream"), ("b", "second input stream")] },
        ],
        tests: &[
            BlockTest { name: "test_stream_subtract", module: "r4w_core::stream_arithmetic::tests", description: "Element-wise complex subtraction", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[],
        related_blocks: &["StreamAdd", "Multiply"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &[],
    });

    m.insert("Probe Avg Power", BlockMetadata {
        block_type: "ProbeAvgPowerBlock",
        display_name: "Probe Avg Mag²",
        category: "Output",
        description: "Running average magnitude-squared power measurement with exponential averaging. Pass-through block: input samples are forwarded unchanged while power is measured as a side effect. Provides threshold-based gating for carrier sensing.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/probe_power.rs", line: 26, symbol: "ProbeAvgMagSqrd", description: "Running average power probe" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Power Averaging", plaintext: "P[n] = (1-alpha)*P[n-1] + alpha*|x[n]|^2", latex: None, variables: &[("alpha", "smoothing factor"), ("P", "running average power")] },
            BlockFormula { name: "Level (dB)", plaintext: "L = 10*log10(P)", latex: None, variables: &[("P", "average power (linear)")] },
        ],
        tests: &[
            BlockTest { name: "test_constant_power", module: "r4w_core::probe_power::tests", description: "Converges to constant power", expected_runtime_ms: 1 },
            BlockTest { name: "test_level_db", module: "r4w_core::probe_power::tests", description: "Correct dB level", expected_runtime_ms: 1 },
            BlockTest { name: "test_unmuted", module: "r4w_core::probe_power::tests", description: "Threshold gating works", expected_runtime_ms: 1 },
            BlockTest { name: "test_pass_through", module: "r4w_core::probe_power::tests", description: "Samples pass through unchanged", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["RmsPower", "PowerSquelch", "LogPowerFft"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["alpha", "threshold_db"],
    });

    m.insert("Envelope Detector", BlockMetadata {
        block_type: "EnvelopeDetectorBlock",
        display_name: "Envelope Detector",
        category: "Recovery",
        description: "Extracts the instantaneous amplitude envelope from a modulated signal. Supports Magnitude (|z|), MagnitudeSquared (|z|²), Smoothed (single-pole lowpass), and PeakHold (exponential decay) modes.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/envelope_detector.rs", line: 38, symbol: "EnvelopeDetector", description: "Envelope extraction" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Magnitude", plaintext: "y[n] = |x[n]| = sqrt(I² + Q²)", latex: None, variables: &[("I", "in-phase component"), ("Q", "quadrature component")] },
            BlockFormula { name: "Smoothed", plaintext: "y[n] = (1-alpha)*y[n-1] + alpha*|x[n]|", latex: None, variables: &[("alpha", "smoothing constant")] },
        ],
        tests: &[
            BlockTest { name: "test_magnitude_envelope", module: "r4w_core::envelope_detector::tests", description: "Constant magnitude extraction", expected_runtime_ms: 1 },
            BlockTest { name: "test_smoothed_envelope", module: "r4w_core::envelope_detector::tests", description: "Smoothed step response", expected_runtime_ms: 1 },
            BlockTest { name: "test_peak_hold", module: "r4w_core::envelope_detector::tests", description: "Peak hold with decay", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[],
        related_blocks: &["AmDemodulator", "ComplexToMag", "RmsPower"],
        input_type: "IQ",
        output_type: "Real",
        key_parameters: &["mode"],
    });

    m.insert("AM Demodulator", BlockMetadata {
        block_type: "AmDemodulatorBlock",
        display_name: "AM Demodulator",
        category: "Recovery",
        description: "Simple AM (Amplitude Modulation) demodulator using envelope detection with DC removal. Extracts the audio modulation from an AM carrier signal.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/envelope_detector.rs", line: 112, symbol: "AmDemodulator", description: "AM demodulation with DC removal" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Envelope", plaintext: "env[n] = |x[n]|", latex: None, variables: &[("x", "input complex signal")] },
            BlockFormula { name: "DC Removal", plaintext: "y[n] = env[n] - DC[n], DC[n] = (1-alpha)*DC[n-1] + alpha*env[n]", latex: None, variables: &[("alpha", "DC filter constant (0.005)")] },
        ],
        tests: &[
            BlockTest { name: "test_am_demodulator", module: "r4w_core::envelope_detector::tests", description: "AM signal demodulation", expected_runtime_ms: 1 },
            BlockTest { name: "test_am_demod_reset", module: "r4w_core::envelope_detector::tests", description: "Reset state", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["EnvelopeDetector", "SsbDemodulator", "NbfmReceiver"],
        input_type: "IQ",
        output_type: "Real",
        key_parameters: &[],
    });

    m.insert("Decimating FIR", BlockMetadata {
        block_type: "DecimatingFirBlock",
        display_name: "Decimating FIR",
        category: "Rate Conversion",
        description: "Combined FIR lowpass filtering and integer decimation. Only computes output samples at the decimation rate, saving computation. Auto-designs Hamming-windowed sinc lowpass with cutoff at pi/decimation.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/decimating_fir.rs", line: 31, symbol: "DecimatingFir", description: "Decimating FIR filter" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "FIR Output", plaintext: "y[m] = sum_k(h[k] * x[m*D - k])", latex: None, variables: &[("D", "decimation factor"), ("h", "FIR taps"), ("m", "output sample index")] },
        ],
        tests: &[
            BlockTest { name: "test_decimation_factor", module: "r4w_core::decimating_fir::tests", description: "Correct output length", expected_runtime_ms: 1 },
            BlockTest { name: "test_removes_high_frequency", module: "r4w_core::decimating_fir::tests", description: "Anti-alias filtering", expected_runtime_ms: 1 },
            BlockTest { name: "test_streaming", module: "r4w_core::decimating_fir::tests", description: "Streaming operation", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n*T/D)", memory: "O(T)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[],
        related_blocks: &["PolyphaseDecimator", "CicDecimator", "Downsampler"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["decimation", "num_taps"],
    });

    m.insert("AFC", BlockMetadata {
        block_type: "AfcBlock",
        display_name: "Automatic Frequency Control",
        category: "Recovery",
        description: "Closed-loop frequency tracking using phase discriminator feedback. Estimates and corrects carrier frequency offset. Second-order loop filter for steady-state tracking of frequency drift.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/afc.rs", line: 51, symbol: "Afc", description: "AFC frequency tracking loop" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Phase Discriminator", plaintext: "f_err = arg(z[n] * conj(z[n-1])) * fs / (2*pi)", latex: None, variables: &[("z", "input signal"), ("fs", "sample rate")] },
            BlockFormula { name: "Loop Filter", plaintext: "f_est += alpha*f_err + beta*integral(f_err)", latex: None, variables: &[("alpha", "proportional gain"), ("beta", "integral gain")] },
        ],
        tests: &[
            BlockTest { name: "test_afc_tracks_offset", module: "r4w_core::afc::tests", description: "Tracks frequency offset", expected_runtime_ms: 1 },
            BlockTest { name: "test_afc_zero_offset", module: "r4w_core::afc::tests", description: "Zero offset convergence", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["CostasLoop", "FllBandEdge", "CarrierRecovery"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["loop_gain", "sample_rate"],
    });

    m.insert("Moving Avg Decim", BlockMetadata {
        block_type: "MovingAvgDecimBlock",
        display_name: "Moving Average Decimator",
        category: "Rate Conversion",
        description: "Boxcar (rectangular window) averaging combined with decimation. Computes the mean of each block of N samples. Equivalent to CIC of order 1. Useful for coarse power detection and first-stage decimation.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/moving_avg_decim.rs", line: 28, symbol: "MovingAvgDecim", description: "Moving average decimator" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Average", plaintext: "y[m] = (1/N) * sum_{k=0}^{N-1} x[m*N + k]", latex: None, variables: &[("N", "averaging/decimation length"), ("m", "output index")] },
        ],
        tests: &[
            BlockTest { name: "test_decimation_length", module: "r4w_core::moving_avg_decim::tests", description: "Correct output length", expected_runtime_ms: 1 },
            BlockTest { name: "test_average_value", module: "r4w_core::moving_avg_decim::tests", description: "Correct average", expected_runtime_ms: 1 },
            BlockTest { name: "test_noise_reduction", module: "r4w_core::moving_avg_decim::tests", description: "Noise variance reduction", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[],
        related_blocks: &["DecimatingFir", "CicDecimator", "Downsampler"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["length"],
    });

    m.insert("DC Blocker", BlockMetadata {
        block_type: "DcBlockerBlock",
        display_name: "DC Blocker",
        category: "Recovery",
        description: "Removes DC offset from a signal using a single-pole IIR highpass filter. Essential for SDR receivers where hardware or ADC introduces DC offset. Configurable pole location controls notch width vs settling time.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/dc_blocker.rs", line: 35, symbol: "DcBlocker", description: "DC offset removal filter" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "IIR Highpass", plaintext: "y[n] = x[n] - x[n-1] + alpha * y[n-1]", latex: None, variables: &[("alpha", "pole location (0.99-0.9999)"), ("x", "input"), ("y", "output")] },
            BlockFormula { name: "Cutoff", plaintext: "f_c = (1-alpha) * fs / (2*pi)", latex: None, variables: &[("fs", "sample rate")] },
        ],
        tests: &[
            BlockTest { name: "test_removes_dc_offset", module: "r4w_core::dc_blocker::tests", description: "Removes DC component", expected_runtime_ms: 1 },
            BlockTest { name: "test_passes_ac_signal", module: "r4w_core::dc_blocker::tests", description: "AC signal preserved", expected_runtime_ms: 1 },
            BlockTest { name: "test_narrow_notch_preserves_low_freq", module: "r4w_core::dc_blocker::tests", description: "Narrow notch mode", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(1)", simd_optimized: false, gpu_accelerable: true }),
        standards: &[],
        related_blocks: &["DcOffsetBlock", "FirFilter", "IirFilter"],
        input_type: "IQ",
        output_type: "IQ",
        key_parameters: &["alpha"],
    });

    m.insert("Short to Complex", BlockMetadata {
        block_type: "InterleavedShortToComplexBlock",
        display_name: "Interleaved Short to Complex",
        category: "Output",
        description: "Converts interleaved i16 [I,Q,I,Q,...] to Complex64. Normalizes to [-1, 1] by dividing by 32768. Essential for interfacing with SDR hardware (USRP, HackRF, PlutoSDR) that outputs 16-bit integer I/Q pairs.",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/interleaved.rs", line: 27, symbol: "InterleavedShortToComplex", description: "i16 to Complex conversion" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Normalization", plaintext: "z[n] = (I[2n]/32768) + j*(Q[2n+1]/32768)", latex: None, variables: &[("I", "in-phase i16"), ("Q", "quadrature i16")] },
        ],
        tests: &[
            BlockTest { name: "test_short_to_complex", module: "r4w_core::interleaved::tests", description: "i16 to complex conversion", expected_runtime_ms: 1 },
            BlockTest { name: "test_short_roundtrip", module: "r4w_core::interleaved::tests", description: "Short roundtrip", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["ComplexToInterleavedShort", "RealToComplex", "TypeConversions"],
        input_type: "Real",
        output_type: "IQ",
        key_parameters: &[],
    });

    m.insert("Complex to Short", BlockMetadata {
        block_type: "ComplexToInterleavedShortBlock",
        display_name: "Complex to Interleaved Short",
        category: "Output",
        description: "Converts Complex64 to interleaved i16 [I,Q,I,Q,...] for SDR hardware transmit. Scales by 32767 and clamps to [-32768, 32767].",
        implementation: Some(CodeLocation { crate_name: "r4w-core", file_path: "src/interleaved.rs", line: 62, symbol: "ComplexToInterleavedShort", description: "Complex to i16 conversion" }),
        related_code: &[],
        formulas: &[
            BlockFormula { name: "Quantization", plaintext: "I[2n] = clamp(Re(z[n]) * 32767, -32768, 32767)", latex: None, variables: &[("z", "complex input")] },
        ],
        tests: &[
            BlockTest { name: "test_complex_to_short", module: "r4w_core::interleaved::tests", description: "Complex to i16", expected_runtime_ms: 1 },
            BlockTest { name: "test_saturation", module: "r4w_core::interleaved::tests", description: "Clipping at bounds", expected_runtime_ms: 1 },
        ],
        performance: Some(PerformanceInfo { complexity: "O(n)", memory: "O(n)", simd_optimized: false, gpu_accelerable: false }),
        standards: &[],
        related_blocks: &["InterleavedShortToComplex", "ComplexToReal", "TypeConversions"],
        input_type: "IQ",
        output_type: "Real",
        key_parameters: &[],
    });

    m
}

fn get_metadata_map() -> &'static HashMap<&'static str, BlockMetadata> {
    BLOCK_METADATA.get_or_init(init_block_metadata)
}

/// Get metadata for a block type
pub fn get_block_metadata(block_type: &str) -> Option<&'static BlockMetadata> {
    get_metadata_map().get(block_type)
}

/// Get all available block metadata
pub fn all_block_metadata() -> impl Iterator<Item = &'static BlockMetadata> {
    get_metadata_map().values()
}

/// Format code location as a clickable path
pub fn format_code_location(loc: &CodeLocation) -> String {
    format!("{}:{} ({}::{})",
        loc.file_path, loc.line, loc.crate_name, loc.symbol)
}

/// Get test command for a block
pub fn get_test_command(test: &BlockTest) -> String {
    // Module names use underscores (r4w_core), but cargo packages use hyphens (r4w-core)
    let package = test.module.split("::").next().unwrap_or("r4w-core").replace('_', "-");
    format!("cargo test --package {} {} -- --nocapture", package, test.name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_psk_metadata_exists() {
        let meta = get_block_metadata("PskModulator");
        assert!(meta.is_some());
        let meta = meta.unwrap();
        assert_eq!(meta.display_name, "PSK Modulator");
        assert!(!meta.formulas.is_empty());
    }

    #[test]
    fn test_all_metadata_has_description() {
        for meta in all_block_metadata() {
            assert!(!meta.description.is_empty(),
                "Block {} missing description", meta.block_type);
        }
    }
}
