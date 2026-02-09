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
                file_path: "src/whitening.rs",
                line: 40,
                symbol: "Whitening",
                description: "LFSR-based whitening/scrambling",
            }),
            related_code: &[],
            formulas: &[
                BlockFormula {
                    name: "LFSR Operation",
                    plaintext: "out[n] = data[n] XOR state XOR feedback(state)",
                    latex: Some(r"\text{out}_n = \text{data}_n \oplus \text{LFSR}_n"),
                    variables: &[
                        ("data[n]", "Input bit"),
                        ("state", "LFSR state"),
                        ("feedback", "Polynomial feedback"),
                    ],
                },
            ],
            tests: &[
                BlockTest {
                    name: "test_scrambler_descrambler",
                    module: "r4w_core::coding::scrambler::tests",
                    description: "Verify round-trip scrambling",
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
