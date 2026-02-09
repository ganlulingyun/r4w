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
