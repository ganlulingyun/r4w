# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**R4W - Rust for Waveforms** - A platform for developing, testing, and deploying SDR waveforms in Rust. Provides reusable DSP libraries, educational tools, and production-ready components for waveform development.

### Architecture

- **r4w-core**: Core DSP algorithms, timing, RT primitives, configuration
  - `waveform/`: 42+ waveform implementations (including GNSS: GPS L1 C/A, GPS L5, GLONASS L1OF, Galileo E1)
  - `waveform/gnss/environment/`: Keplerian orbits, Klobuchar ionosphere, Saastamoinen troposphere, multipath presets, antenna patterns
  - `waveform/gnss/scenario*.rs`: Multi-satellite GNSS IQ scenario generation with realistic channel effects
  - `coordinates.rs`: ECEF/LLA coordinate types, geodetic conversions, look angles, range rate, FSPL
  - `filters/`: Digital filters with trait-based architecture
    - `traits.rs`: Filter, RealFilter, FirFilterOps, FrequencyResponse traits
    - `fir.rs`: FirFilter with lowpass/highpass/bandpass/bandstop, Kaiser window design
    - `iir.rs`: IirFilter with Butterworth, Chebyshev I/II, Bessel via cascaded biquads
    - `polyphase.rs`: PolyphaseDecimator, PolyphaseInterpolator, Resampler, HalfbandFilter
    - `remez.rs`: Parks-McClellan equiripple FIR design (RemezSpec builder)
    - `pulse_shaping.rs`: RRC, RC, Gaussian filters (also implement Filter traits)
    - `windows.rs`: Hamming, Hann, Blackman, Kaiser window functions
  - `agc.rs`: Automatic Gain Control (Agc, Agc2, Agc3 - basic/dual-rate/fast-acquisition)
  - `carrier_recovery.rs`: Costas loop for BPSK/QPSK/8PSK carrier recovery
  - `clock_recovery.rs`: Mueller & Muller symbol timing recovery, FreqXlatingFirFilter
  - `crc.rs`: CRC engine (CRC-8, CRC-16 CCITT/IBM, CRC-32, CRC-32C) with table lookup
  - `equalizer.rs`: Adaptive equalizer (LMS, CMA, Decision-Directed) with Filter trait
  - `signal_source.rs`: Signal generator (Tone, TwoTone, Chirp, Noise, Square, DC, Impulse)
  - `squelch.rs`: Power squelch gate with ramp transitions (equiv. to GNU Radio pwr_squelch_cc)
  - `ofdm.rs`: OFDM modulator/demodulator (WiFi-like, DVB-T 2K, simple configs)
  - `pfb_channelizer.rs`: Polyphase filter bank channelizer with windowed-sinc prototype filter
  - `correlator.rs`: Cross-correlation sync word detector (Barker codes, dynamic threshold, holdoff)
  - `scrambler.rs`: LFSR scrambler/descrambler (additive/multiplicative, WiFi/DVB-S2/Bluetooth/V.34)
  - `differential.rs`: Differential encoder/decoder (DBPSK/DQPSK/D8PSK, complex-domain DPSK)
  - `packet_framing.rs`: Packet formatter/parser (sync word, headers, CRC, AX.25/ISM configs)
  - `nco.rs`: NCO/VCO + FM modulator/demodulator (PLL building block, frequency translation)
  - `snr_estimator.rs`: SNR estimation (M2M4, split-spectrum, signal+noise, EVM methods)
  - `symbol_mapping.rs`: Constellation mapper/demapper (BPSK/QPSK/8PSK/16QAM/64QAM, soft LLR)
  - `probe.rs`: Measurement probes (Power/PAPR, EVM, constellation scatter, freq offset, rate)
  - `pll.rs`: Second-order PLL, DC blocker (IIR highpass), sample delay (circular buffer)
  - `costas_loop.rs`: Costas loop carrier recovery (BPSK/QPSK/8PSK decision-directed)
  - `constellation_receiver.rs`: Combined AGC + Costas + symbol demapper receiver
  - `goertzel.rs`: Goertzel single-frequency DFT + MultiGoertzel + DTMF detector
  - `burst_detector.rs`: Power-based burst detector with hysteresis (SOB/EOB events)
  - `noise.rs`: Colored noise generator (white/pink/brown/blue/violet) + AWGN helpers
  - `stream_tags.rs`: Metadata propagation (TagStore, TagValue, range queries, well-known keys)
  - `filters/cic.rs`: CIC decimator/interpolator for high-ratio sample rate conversion
  - `filters/adaptive.rs`: Adaptive filters (LMS, NLMS, RLS) for equalization/cancellation
  - `filters/fractional_resampler.rs`: MMSE interpolating resampler for arbitrary rate conversion
  - `freq_xlating_fir.rs`: Frequency-translating FIR filter (mixing + FIR + decimation)
  - `fm_emphasis.rs`: FM pre-emphasis/de-emphasis (US 75us / EU 50us, 1-pole IIR)
  - `quadrature_demod.rs`: FM discriminator y[n] = gain * arg(x[n] * conj(x[n-1]))
  - `access_code_detector.rs`: Bit-level sync word detector with Hamming distance threshold
  - `fll_band_edge.rs`: FLL band-edge coarse frequency sync (band-edge FIR + 2nd-order loop)
  - `type_conversions.rs`: Complex/Real converters (Mag, MagSq, Arg, Real, Imag, MagPhase)
  - `stream_control.rs`: Head (first N), SkipHead (drop N), Throttle (rate-limit)
  - `log_power_fft.rs`: Windowed FFT to dB power with exponential averaging
  - `pdu.rs`: PDU↔tagged stream conversion, message debug (Hex/Text/Decimal/Summary)
  - `ofdm_channel_est.rs`: OFDM pilot-based channel estimation (LS/smoothed), ZF/MMSE equalization
  - `ssb_modem.rs`: SSB modulator/demodulator (Hilbert transform phasing method, BFO)
  - `wavelet.rs`: DWT analysis/synthesis (Haar/Db4/Sym4), soft/hard threshold denoising
  - `cpm.rs`: CPM/GMSK/GFSK/MSK modulation (constant-envelope, Gaussian/LRC pulse shapes)
  - `dynamic_channel.rs`: Composite time-varying channel (fading + CFO/SRO drift + AWGN)
  - `burst_tagger.rs`: Power-based burst detection, tagged stream mux/align
  - `stream_to_streams.rs`: Round-robin stream demux/mux (StreamToStreams, StreamsToStream), I/Q deinterleave
  - `argmax.rs`: Index-of-max (argmax/argmin for f64 and mag_sqrd), ArgmaxBlock, top_k functions
  - `threshold.rs`: Hysteresis threshold detector, rising/falling edge detection, stateless comparison
  - `pdu_filter.rs`: PDU metadata filtering (FilterRule, PduFilter AND-logic, PduRouter first-match)
  - `regenerate_bb.rs`: Bit regeneration pulse stretcher, PulseGenerator, rect/trapezoidal pulses
  - `patterned_interleaver.rs`: Custom-pattern stream interleaving/deinterleaving of N streams (f64, bytes)
  - `bitwise_ops.rs`: XOR/AND/OR/NOT for byte and boolean streams, hamming_distance, popcount, parity
  - `peak_hold.rs`: Signal peak tracking with exponential decay (PeakHold, AbsPeakHold, PeakHoldDb)
  - `multiply_matrix.rs`: Complex and real matrix-vector multiplication (identity, scalar, diagonal, from_rows)
  - `glfsr_source.rs`: Galois and Fibonacci LFSR PN sequence generators (maximal-length polynomials 2-31 bits)
  - `additive_scrambler.rs`: LFSR-based stream scrambling (DVB x^15+x^14+1, WiFi x^7+x^4+1 presets, auto-reset)
  - `stretch.rs`: Signal normalization and range mapping (stretch, stretch_to_range, clip, StreamStretch)
  - `nlog10.rs`: Logarithmic scaling (to_db, from_db, to_dbm, from_dbm, power_to_db, iq_to_db)
  - `dpll.rs`: Second-order digital PLL with PI loop filter (Dpll carrier tracking, BinaryDpll clock recovery)
  - `endian_swap.rs`: Byte order conversion (16/32/64-bit swap, typed swap_i16/u16/i32/u32/f32/f64, reverse_bits)
  - `iq_balance.rs`: IQ imbalance estimation/correction (IqBalanceCorrector fixed gain/phase, AdaptiveIqBalance LMS-based online)
  - `peak_to_average.rs`: PAPR/crest factor measurement (papr_db, papr_db_complex, crest_factor, PaprEstimator streaming windowed)
  - `correlate_estimate.rs`: Time-domain cross-correlation (cross_correlate, cross_correlate_complex, find_delay, autocorrelate, correlation_coefficient)
  - `bin_statistics.rs`: Per-FFT-bin statistical accumulation (BinStatistics min/max/mean/variance, accumulate_max_hold, dynamic_range)
  - `check_lfsr.rs`: LFSR sequence verification for BER testing (LfsrChecker, synchronize offset detection, generate_reference, cumulative BER)
  - `timing_error_detector.rs`: Gardner, Mueller-Muller, Early-Late Gate, Zero-Crossing TEDs for clock recovery (real/complex, streaming TimingErrorDetector block)
  - `constellation_demapper.rs`: Soft bit demapping via max-log-MAP (BPSK/QPSK fast-path, generic ConstellationDemapper for arbitrary constellations, hard/soft output)
  - `eye_diagram.rs`: Eye diagram generator with trace accumulation (mean_trace, envelope, eye_opening, timing_jitter_rms for ISI assessment)
  - `evm_calculator.rs`: Error Vector Magnitude measurement (RMS/peak/percentile EVM in linear/dB/percent, streaming EvmCalculator with history)
  - `valve.rs`: Stream gating and flow control (Open/Closed/CountedBurst/Triggered modes, gate_signal, extract_segments)
  - `pfb_arb_resampler.rs`: Polyphase filterbank arbitrary resampler (linear interpolation between branches, Blackman-Harris prototype, derivative filters)
  - `cyclic_autocorrelation.rs`: Cyclostationary signal analysis (cyclic autocorrelation, spectral correlation, feature detection, modulation classification)
  - `random_pdu_gen.rs`: Random PDU test traffic generator (Poisson/Uniform/Fixed inter-arrival, seeded LCG PRNG, traffic statistics)
  - `mmse_interpolator.rs`: 8-tap MMSE FIR fractional delay (128-step mu quantization, windowed-sinc with Nuttall window, cubic/linear helpers)
  - `fractional_delay.rs`: Thiran all-pass IIR and Lagrange FIR fractional sample delay (maximally flat group delay)
  - `linear_equalizer.rs`: Adaptive FIR equalizer (LMS, RLS, CMA, Kurtotic algorithms, training/decision-directed modes)
  - `trellis_coding.rs`: FSM-based trellis encoder + Viterbi decoder for TCM and convolutional codes
  - `protocol_formatter.rs`: Pluggable HeaderFormat trait, DefaultHeaderFormat (access code + repeated length + CRC), CounterHeaderFormat
  - `ctcss_squelch.rs`: CTCSS sub-audible tone encoder/decoder for FM repeater access (Goertzel detection, 38 standard EIA/TIA tones)
  - `ofdm_frame_equalizer.rs`: Per-subcarrier ZF/MMSE equalization with pilot-based channel estimation, linear interpolation
  - `power_amplifier_model.rs`: Nonlinear PA behavioral models (Saleh, Rapp, Ghorbani, polynomial, limiter) with AM/AM, AM/PM distortion, P1dB compression point
  - `power_amplifier_dpd.rs`: Digital Pre-Distortion with memory polynomial, LMS/RLS indirect learning architecture, NMSE metric
  - `median_filter.rs`: Sliding-window median filter with dual-heap O(log n), complex/weighted variants, 2D hybrid median for spectrogram denoising
  - `cordic.rs`: CORDIC rotator for sin/cos, atan2, polar/rect conversions, NCO, hardware-efficient iterative algorithm using shifts and adds
  - `tagged_file_sink.rs`: Stream-tag-triggered IQ file segmentation for burst recording
  - `cfar.rs`: CFAR detector (CA-CFAR, GO-CFAR, SO-CFAR, OS-CFAR) for 1D power data, Cfar2D for range-Doppler maps
  - `chirp_z_transform.rs`: Chirp-Z Transform (Bluestein's algorithm), zoom_fft for high-resolution spectral analysis
  - `permute.rs`: Vector index permutation with validation, inverse/compose/bit-reversal, StreamPermuter block-mode
  - `noise_reduction.rs`: Spectral subtraction and Wiener noise reduction with MMSE filtering
  - `savitzky_golay.rs`: Savitzky-Golay polynomial smoothing and differentiation filter with cached coefficients
  - `sigma_delta.rs`: Sigma-delta modulator/demodulator with error-feedback (EFB) structure, 1st/2nd/3rd order, multi-bit quantizer, CIC sinc³ decimation filter, theoretical SNR computation, NTF magnitude
  - `farrow_resampler.rs`: Farrow polynomial structure for continuously variable fractional resampling, linear/quadratic/cubic interpolation via Horner evaluation, variable ratio on-the-fly, resample_to_length utility
  - `empirical_mode.rs`: Empirical Mode Decomposition (EMD) via sifting process + Hilbert-Huang Transform, extracts IMFs (Intrinsic Mode Functions) from non-stationary signals, cubic spline envelope interpolation, instantaneous frequency/amplitude
  - `ambiguity_function.rs`: Radar waveform delay-Doppler analysis, full 2D ambiguity surface |χ(τ,ν)|², zero-Doppler and zero-delay cuts, LFM chirp and Barker code generators, mainlobe width measurement, complex signal support
  - `pulse_compressor.rs`: Matched filtering for radar (LFM chirp/Barker/polyphase code references), Hamming/Chebyshev/Taylor sidelobe windows, time-domain and FFT-domain processing, processing gain, range resolution
  - `mti_filter.rs`: Moving Target Indication/Detection for pulsed radar (single/double/triple cancellers, binomial weights, custom FIR slow-time filters, DopplerFilterBank for MTD with windowing, blind speed calculation, frequency response)
  - `fountain_code.rs`: Luby Transform rateless erasure codes (ideal/robust soliton degree distributions, belief propagation peeling decoder, streaming encode/decode, deterministic PRNG)
  - `cross_ambiguity_function.rs`: Passive bistatic radar CAF (direct and batched correlation, LMS/NLMS/ECA-B direct-path interference cancellation, target detection with SNR estimation, bistatic range conversion)
  - `feedforward_timing_estimator.rs`: Non-data-aided burst-mode timing recovery (Oerder-Meyr spectral line, M-th power, squaring, Gardner feedforward algorithms, linear/cubic/Farrow/sinc interpolation)
  - `adaptive_modcod.rs`: Adaptive Modulation and Coding link adaptation (DVB-S2 ACM 28-entry modcod table, LTE CQI 15 entries, Wi-Fi MCS 10 entries, MaxThroughput/MaxReliability/TargetEfficiency strategies, hysteresis/EMA SNR averaging/backoff margin)
  - `mimo_detector.rs`: MIMO detection algorithms (Schnorr-Euchner sphere decoder, K-best tree search, exhaustive ML, MMSE-SIC, QR decomposition, soft LLR output, ConstellationSet QPSK/16QAM/64QAM/256QAM)
  - `doppler_pre_correction.rs`: Satellite Doppler pre-compensation (Constant/LinearRamp/Polynomial/Tabulated profiles, phase-continuous NCO, Doppler rate computation, LEO satellite comms/deep-space links)
  - `cognitive_engine.rs`: Dynamic spectrum access decision engine (OODA loop IEEE 802.22, Greedy/EpsilonGreedy/UCB1/ThompsonSampling strategies, SpectrumBand with BandPriority/RegulatoryStatus, PU detection/handoff/vacancy prediction)
  - `raptor_code.rs`: RaptorQ erasure codes RFC 6330 (systematic rateless LDPC+HDPC pre-coding over LT, near-zero reception overhead, 3GPP MBMS/ATSC 3.0/DVB-H, belief propagation peeling decoder)
  - `polar_code.rs`: Arikan's 5G NR polar codes (PolarEncoder butterfly transform, PolarDecoder recursive SC decoding, Bhattacharyya channel reliability ordering)
  - `msk_modulator.rs`: MSK (Minimum Shift Keying, h=0.5 continuous-phase FSK) modulator/demodulator, GMSK variant with configurable BT product Gaussian pre-filter, phase-accumulation demod (GSM, satellite)
  - `oqpsk_modulator.rs`: Offset QPSK modulator/demodulator (Q delayed by T/2, +-pi/2 phase transitions, PAPR analysis, ZigBee 802.15.4, CDMA IS-95)
  - `frequency_hopping.rs`: FHSS controller (Pseudorandom/Sequential/Fixed/Adaptive hop patterns, Bluetooth 79ch/1600hps and military HF presets, processing gain)
  - `digital_down_converter.rs`: DDC with NCO mixer, 3-stage CIC decimation, windowed-sinc FIR compensation filter, retuning, reset
  - `esprit.rs`: ESPRIT DOA estimation (LS/TLS variants, ULA steering vectors, complements MUSIC DOA)
  - `convolutional_interleaver.rs`: Forney-type convolutional interleaver/deinterleaver (DVB-S2 I=12/M=17, GSM I=4/M=19 presets, burst error dispersal)
  - `unscented_kalman_filter.rs`: Unscented Kalman Filter with UkfModel trait, sigma point generation, predict/update cycle, NEES metric
  - `sar_processor.rs`: SAR Range-Doppler Algorithm (range compression, RCMC, azimuth compression, point target generation, Doppler centroid estimation)
  - `wola_channelizer.rs`: Weighted Overlap-Add analysis/synthesis filterbank (Hann/Hamming/Blackman/Kaiser windows, channel frequency response)
  - `dynamic_range_compressor.rs`: Compressor/limiter/expander/noise gate with attack/release envelope follower, soft knee, RMS/peak detection, makeup gain, static compression curve for visualization
  - `teager_kaiser_energy.rs`: Teager-Kaiser Energy Operator (TKEO) instantaneous energy, AM/FM demodulation via TKEO, streaming processor, transient detection
  - `wigner_ville_distribution.rs`: WVD/PWVD/SPWVD time-frequency analysis, analytic signal, instantaneous frequency extraction, 2D time-frequency surface with marginals
  - `lattice_filter.rs`: Lattice and lattice-ladder filter structures, Levinson-Durbin recursion, Burg's method for AR estimation, PARCOR coefficients, LSF
  - `prony_method.rs`: Prony's method for parametric exponential signal modeling, matrix pencil method, companion matrix eigenvalue solver, MDL order estimation, parametric PSD
  - `cepstral_analysis.rs`: Real/power/complex cepstrum, pitch detection, homomorphic filtering (source-filter separation), MFCCs, mel filterbank, spectral envelope
  - `blind_source_separation.rs`: FastICA with deflation, PCA whitening, 3 nonlinearities (LogCosh, Exp, Cube), kurtosis, negentropy, correlation metrics
  - `phase_vocoder.rs`: STFT-based time-stretch, pitch-shift, spectrogram, robotize, whisperize effects with overlap-add synthesis
  - `compressive_sensing.rs`: OMP (greedy), ISTA/FISTA (proximal gradient) sparse recovery from underdetermined systems, random/DCT sensing matrices, RIP constant estimation
  - `zero_crossing_detector.rs`: ZCR computation, frequency estimation, ZcrAnalyzer with energy, VoiceActivityDetector (energy+ZCR with hangover), spectral centroid/flatness, modulation classification
  - `subspace_tracker.rs`: PAST/OPAST adaptive rank-d subspace tracking, projection, projection error, dimension estimation, subspace angle
  - `cic_filter.rs`: CIC decimation/interpolation without multiplications, passband compensator FIR design
  - `overlap_save.rs`: Overlap-Save and Overlap-Add streaming FFT block convolution, direct convolution reference
  - `lms_filter.rs`: Standard LMS, Normalized LMS (NLMS), Leaky LMS adaptive filters for system identification and noise cancellation
  - `stft.rs`: Short-Time Fourier Transform with configurable windows (Hann/Hamming/Blackman), Inverse STFT with OLA perfect reconstruction, COLA constraint checking
  - `music_doa.rs`: MUSIC direction-of-arrival estimation with Hermitian eigendecomposition (augmented real form + Jacobi), MDL/AIC source enumeration, test snapshot generation
  - `rake_receiver.rs`: RAKE multipath combining for DSSS/CDMA (MRC, equal-gain, selection diversity), finger management, delay estimation
  - `modulation_classifier.rs`: Automatic modulation classification via higher-order cumulants (C20/C40/C42), kurtosis, sigma_af feature extraction
  - `tdoa_estimator.rs`: Time Difference of Arrival geolocation with GCC-PHAT cross-correlation, iterative least-squares solver
  - `fm_stereo_decoder.rs`: FM stereo multiplex decoder with 19 kHz pilot PLL, 38 kHz DSB-SC demodulation, de-emphasis filter
  - `phase_noise_model.rs`: Oscillator phase noise synthesis from L(f) PSD masks and Leeson model, configurable noise floor and corner frequencies
  - `ofdm_sync_schmidl_cox.rs`: Schmidl-Cox OFDM symbol timing and coarse/fine CFO estimation (delayed autocorrelation, timing metric M(d), preamble generator)
  - `ofdm_carrier_allocator.rs`: OFDM subcarrier mapping (CarrierAllocator TX, CarrierSerializer RX), WiFi 802.11a and LTE presets, guard bands, DC null, pilot insertion
  - `trellis_metrics.rs`: Branch metric computation for trellis-based decoding (Euclidean, squared Euclidean, Manhattan, Hamming, ViterbiCombined, metrics_to_llr)
  - `link_budget.rs`: RF link budget calculator (builder pattern, FSPL, cascaded NF via Friis, thermal noise floor, max range estimation)
  - `fec_generic_api.rs`: Unified FEC framework (GenericEncoder/GenericDecoder traits, streaming FecEncoderBlock/FecDecoderBlock, AsyncFecEncoder/AsyncFecDecoder, FecCodecRegistry, built-in Repetition and ParityCheck codecs)
  - `fec/`: Forward Error Correction
    - `convolutional.rs`: Convolutional encoder + Viterbi decoder (hard/soft decision)
    - `reed_solomon.rs`: RS encoder/decoder over GF(2^8) (CCSDS, DVB, custom configs)
    - `polar.rs`: Polar code encoder/decoder (5G NR, SC/SCL algorithm, Bhattacharyya construction)
    - Standard codes: NASA K=7 rate 1/2, GSM K=5, 3GPP K=9 rate 1/3
  - `analysis/`: Spectrum analyzer, waterfall generator, signal statistics, peak detection
  - `timing.rs`: Multi-clock model (SampleClock, WallClock, HardwareClock, SyncedTime)
  - `rt/`: Lock-free ring buffers, buffer pools, RT thread spawning
  - `config.rs`: YAML-based configuration system
- **r4w-sim**: SDR simulation, HAL traits, channel models
  - `hal/`: StreamHandle, TunerControl, ClockControl, SdrDeviceExt traits
  - `channel.rs`: AWGN, Rayleigh, Rician, CFO, TDL multipath (EPA/EVA/ETU), Jake's Doppler
  - `doppler.rs`: Jake's/Clarke's, Flat, and Gaussian Doppler models
  - `scenario/`: Generic multi-emitter IQ scenario engine (trajectory, emitter trait, Doppler/FSPL/noise)
  - `simulator.rs`: Software SDR simulator
- **r4w-fpga**: FPGA acceleration (Xilinx Zynq, Lattice iCE40/ECP5)
- **r4w-sandbox**: Waveform isolation (8 security levels)
- **r4w-gui**: Educational egui application (run with `cargo run --bin r4w-explorer`)
  - `views/pipeline_wizard.rs`: Visual pipeline builder with 278+ blocks in 11 categories (incl. GNSS), TX/RX/Channel loading, type-aware test panel
  - `views/block_metadata.rs`: Block documentation, formulas, code links, tests, performance info
- **r4w-cli**: Command-line interface (run with `cargo run --bin r4w`)
- **r4w-web**: WebAssembly entry point for browser deployment

### Key Commands

```bash
# Run GUI
cargo run --bin r4w-explorer

# CLI examples
cargo run --bin r4w -- info --sf 7 --bw 125
cargo run --bin r4w -- simulate --message "Hello R4W!" --snr 10.0
cargo run --bin r4w -- waveform --list

# Mesh networking
cargo run --bin r4w -- mesh info
cargo run --bin r4w -- mesh status --preset LongFast --region US
cargo run --bin r4w -- mesh send -m "Hello mesh!" --dest broadcast
cargo run --bin r4w -- mesh simulate --nodes 4 --messages 10

# Waveform comparison (BER vs SNR)
cargo run --bin r4w -- compare -w BPSK,QPSK,16QAM --snr-min -5 --snr-max 20
cargo run --bin r4w -- compare --list  # Show available waveforms

# Shell completions (bash/zsh/fish/powershell)
cargo run --bin r4w -- completions bash > ~/.local/share/bash-completion/completions/r4w

# Record/Playback signals (SigMF format)
cargo run --bin r4w -- record -o test.sigmf --generate tone --duration 5.0
cargo run --bin r4w -- playback -i test.sigmf --info

# GNSS signal exploration
cargo run --bin r4w -- gnss info --signal all
cargo run --bin r4w -- gnss compare
cargo run --bin r4w -- gnss code --prn 1 --cross-prn 7
cargo run --bin r4w -- gnss simulate --prn 1 --cn0 40 --doppler 1000
cargo run --bin r4w -- gnss generate --signal GPS-L1CA --prn 1 --bits 10

# GNSS scenario generation (multi-satellite IQ with channel effects)
cargo run --bin r4w -- gnss scenario --list-presets
cargo run --bin r4w -- gnss scenario --preset open-sky --duration 0.001 --output test.iq
cargo run --bin r4w -- gnss scenario --preset urban-canyon --duration 0.01
cargo run --bin r4w -- gnss scenario --preset multi-constellation --sample-rate 4092000

# GNSS with precise ephemeris (SP3) and ionosphere (IONEX) - requires 'ephemeris' feature
cargo run --bin r4w --features ephemeris -- gnss scenario --preset open-sky \
    --sp3 /path/to/COD0OPSFIN_20260050000_01D_05M_ORB.SP3 \
    --ionex /path/to/COD0OPSFIN_20260050000_01D_01H_GIM.INX \
    --duration 0.01 --output precise.iq

# Output formats: cf64, cf32/ettus (USRP-compatible), ci16/sc16, ci8, cu8/rtlsdr
cargo run --bin r4w -- gnss scenario --preset open-sky --format ettus --output usrp.iq
cargo run --bin r4w -- gnss scenario --preset open-sky --format sc16 --output compact.iq
cargo run --bin r4w -- gnss scenario --preset open-sky --format rtlsdr --output rtlsdr.iq

# Signal analysis
cargo run --bin r4w -- analyze spectrum -i file.sigmf-meta --fft-size 1024
cargo run --bin r4w -- analyze waterfall -i file.sigmf-meta --output waterfall.png
cargo run --bin r4w -- analyze stats -i file.sigmf-meta --format json
cargo run --bin r4w -- analyze peaks -i file.sigmf-meta --threshold 10

# Generate gallery images
cargo run --example gallery_generate -p r4w-sim --features image

# Prometheus metrics
cargo run --bin r4w -- metrics --format prometheus
cargo run --bin r4w -- metrics --serve --port 9090

# Run in browser (WASM)
cd crates/r4w-web && trunk serve

# Docker
docker run -it r4w:latest r4w --help
docker-compose run r4w-dev  # Development container

# cargo-binstall (pre-built binaries)
cargo binstall r4w-cli

# Cross-compile for ARM
make build-cli-arm64
make deploy-arm64 REMOTE_HOST=joe@raspberrypi

# Generate HTML documentation from markdown (requires pandoc)
make docs-html     # Output: docs/html/ (21 files with TOC and syntax highlighting)
make docs-clean    # Remove generated HTML
```

### Waveform Development

All waveforms implement the `Waveform` trait:
- `modulate(&self, bits: &[bool]) -> Vec<IQSample>`
- `demodulate(&self, samples: &[IQSample]) -> Vec<bool>`
- `constellation_points(&self) -> Vec<IQSample>`
- `get_modulation_stages()` / `get_demodulation_steps()` for education

See OVERVIEW.md for the full Waveform Developer's Guide and Porting Guide.

### Technical Notes

- LoRa uses Chirp Spread Spectrum (CSS) modulation
- FFT-based demodulation (multiply by downchirp, find peak)
- Pipeline: Whitening → Hamming FEC → Interleaving → Gray Code → CSS
- Spreading factors 5-12, bandwidths 125/250/500 kHz
- PSK/FSK/QAM waveforms for comparison and education

### Recent Updates

- **Batch 58 DSP Blocks** - Polar Code (`polar_code.rs` - Arikan's 5G NR polar codes, PolarEncoder with butterfly polar transform, PolarDecoder with recursive SC successive cancellation decoding using f/g-function factor graph traversal, Bhattacharyya bounds for channel reliability ordering), MSK Modulator (`msk_modulator.rs` - MSK Minimum Shift Keying h=0.5 continuous-phase FSK modulator and demodulator, GMSK variant with configurable BT product Gaussian pre-filter, phase-accumulation demodulation, used in GSM and satellite comms), OQPSK Modulator (`oqpsk_modulator.rs` - Offset QPSK modulator/demodulator with Q channel delayed by T/2 limiting phase transitions to +-pi/2, PAPR analysis, used in ZigBee 802.15.4 and CDMA IS-95), Frequency Hopping (`frequency_hopping.rs` - FHSS Frequency Hopping Spread Spectrum controller, HopPattern: Pseudorandom LFSR/Sequential/Fixed/Adaptive with blacklist, Bluetooth 79ch 1600 hops/s and military HF presets, processing gain calculation), Digital Down Converter (`digital_down_converter.rs` - DDC with NCO mixer, 3-stage CIC decimation, FIR compensation filter with windowed-sinc Hamming lowpass design, retuning and reset). ~262+ DSP modules total, ~2870+ unit tests.

- **Batch 57 DSP Blocks** - ESPRIT DOA (`esprit.rs` - ESPRIT direction-of-arrival estimation with LS and TLS variants, ULA steering vector generation, rotation matrix extraction, complements existing MUSIC DOA for subspace-based angle estimation), Convolutional Interleaver (`convolutional_interleaver.rs` - Forney-type convolutional interleaver/deinterleaver for burst error dispersal, DVB-S2 I=12/M=17 and GSM I=4/M=19 presets, shift-register based delay structure, sync marker insertion), Unscented Kalman Filter (`unscented_kalman_filter.rs` - UKF with UkfModel trait for nonlinear state estimation, Merwe scaled sigma point generation, predict/update cycle, NEES consistency metric, constant-velocity and coordinated-turn presets), SAR Processor (`sar_processor.rs` - SAR Range-Doppler Algorithm for synthetic aperture radar imaging, range compression via matched filtering, RCMC range cell migration correction, azimuth compression, point target scene generation, Doppler centroid estimation), WOLA Channelizer (`wola_channelizer.rs` - Weighted Overlap-Add analysis/synthesis filterbank for wideband channelization, multiple window types Hann/Hamming/Blackman/Kaiser, configurable overlap factor, channel frequency response extraction, near-perfect reconstruction). ~257+ DSP modules total, ~2820+ unit tests.

- **Batch 56 DSP Blocks** - Adaptive ModCod (`adaptive_modcod.rs` - Adaptive Modulation and Coding link adaptation engine, DVB-S2 ACM 28-entry modcod table per ETSI EN 302 307-1, LTE CQI 15 entries per 3GPP TS 36.213, Wi-Fi MCS 10 entries for 802.11n/ac, strategies: MaxThroughput/MaxReliability/TargetEfficiency, hysteresis/EMA SNR averaging/backoff margin), MIMO Detector (`mimo_detector.rs` - Schnorr-Euchner sphere decoder, K-best tree search, exhaustive ML, MMSE-SIC MIMO detection, QR decomposition via modified Gram-Schmidt, soft LLR output max-log approximation, ConstellationSet QPSK/16QAM/64QAM/256QAM, ChannelMatrix Hermitian/mat_mul operations), Doppler Pre-Correction (`doppler_pre_correction.rs` - satellite Doppler pre-compensation, profiles: Constant/LinearRamp/Polynomial/Tabulated, phase-continuous NCO correction, Doppler rate computation, utilities: doppler_from_range_rate/max_leo_doppler, for LEO satellite comms Iridium/Starlink and deep-space links), Cognitive Engine (`cognitive_engine.rs` - dynamic spectrum access decision engine, OODA loop for cognitive radio per IEEE 802.22, strategies: Greedy/EpsilonGreedy/UCB1/ThompsonSampling, SpectrumBand with BandPriority/RegulatoryStatus, PU detection/handoff/vacancy prediction/spectrum utilization, builds on spectrum_sensor.rs), RaptorQ Code (`raptor_code.rs` - RaptorQ erasure codes RFC 6330, systematic rateless codes with LDPC+HDPC pre-coding over LT inner code, near-zero reception overhead, mandated by 3GPP MBMS/ATSC 3.0/DVB-H, RaptorQEncoder/RaptorQDecoder with belief propagation peeling decoder, builds on fountain_code.rs LT codes). ~252+ DSP modules total, ~2770+ unit tests.

- **Batch 55 DSP Blocks** - Pulse Compressor (`pulse_compressor.rs` - matched filtering for radar signal processing with LFM chirp/Barker/polyphase code reference generators, Hamming/Chebyshev/Taylor sidelobe windows, time-domain and FFT-domain processing, processing gain calculation, range resolution), MTI Filter (`mti_filter.rs` - Moving Target Indication/Detection for pulsed radar, single/double/triple cancellers, binomial weights, custom FIR slow-time filters, DopplerFilterBank for MTD with windowing, blind speed calculation, frequency response analysis), Fountain Code (`fountain_code.rs` - Luby Transform rateless erasure codes for broadcast/multicast channels, ideal/robust soliton degree distributions, belief propagation peeling decoder, streaming encode/decode with deterministic PRNG), Cross-Ambiguity Function (`cross_ambiguity_function.rs` - passive bistatic radar CAF, direct and batched correlation, LMS/NLMS/ECA-B direct-path interference cancellation, target detection with SNR estimation, bistatic range conversion), Feedforward Timing Estimator (`feedforward_timing_estimator.rs` - non-data-aided burst-mode timing recovery, Oerder-Meyr spectral line/M-th power/squaring/Gardner feedforward algorithms, linear/cubic/Farrow/sinc interpolation for resampling). ~247+ DSP modules total, ~2720+ unit tests.

- **Batch 54 DSP Blocks** - OFDM Schmidl-Cox Sync (`ofdm_sync_schmidl_cox.rs` - OFDM symbol timing and coarse/fine CFO estimation using delayed autocorrelation P(d) with half-symbol repetition detection, timing metric M(d)=|P(d)|^2/R(d)^2, preamble generator), OFDM Carrier Allocator (`ofdm_carrier_allocator.rs` - subcarrier mapping TX/RX with CarrierAllocator and CarrierSerializer, WiFi 802.11a 48+4 pilot and LTE resource block presets, guard bands, DC null), Trellis Metrics (`trellis_metrics.rs` - branch metric computation for Viterbi/BCJR: Euclidean, squared Euclidean, Manhattan, Hamming hard/soft, ViterbiCombined, metrics_to_llr), Link Budget (`link_budget.rs` - RF link budget calculator with builder pattern, cascaded receiver NF via Friis, FSPL, thermal noise floor, max range), FEC Generic API (`fec_generic_api.rs` - unified FEC framework with GenericEncoder/GenericDecoder traits, streaming FecEncoderBlock/FecDecoderBlock, async PDU mode, FecCodecRegistry runtime selection, built-in Repetition and ParityCheck codecs). ~242+ DSP modules total, ~2670+ unit tests.

- **Batch 53 DSP Blocks** - RAKE Receiver (`rake_receiver.rs` - RAKE multipath combining for DSSS/CDMA with MRC, equal-gain, selection diversity), Modulation Classifier (`modulation_classifier.rs` - automatic modulation classification via higher-order cumulants C20/C40/C42, kurtosis, sigma_af), TDOA Estimator (`tdoa_estimator.rs` - Time Difference of Arrival geolocation with GCC-PHAT cross-correlation and iterative least-squares), FM Stereo Decoder (`fm_stereo_decoder.rs` - FM stereo multiplex decoder with 19 kHz pilot PLL, 38 kHz DSB-SC demod, de-emphasis), Phase Noise Model (`phase_noise_model.rs` - oscillator phase noise synthesis from L(f) PSD masks and Leeson model). ~237+ DSP modules total, ~2620+ unit tests.

- **Batch 52 DSP Blocks** - CIC Filter (`cic_filter.rs` - CIC decimation/interpolation without multiplications, passband compensator FIR design), Overlap-Save (`overlap_save.rs` - Overlap-Save and Overlap-Add streaming FFT block convolution, direct convolution reference), LMS Filter (`lms_filter.rs` - Standard LMS, Normalized LMS NLMS, Leaky LMS adaptive filters for system identification and noise cancellation), STFT (`stft.rs` - Short-Time Fourier Transform with configurable windows Hann/Hamming/Blackman, Inverse STFT with OLA perfect reconstruction, COLA constraint checking), MUSIC DOA (`music_doa.rs` - MUSIC direction-of-arrival estimation with Hermitian eigendecomposition augmented real form + Jacobi, MDL/AIC source enumeration, test snapshot generation). ~232+ DSP modules total, ~2560+ unit tests.

- **Batch 51 DSP Blocks** - Blind Source Separation (`blind_source_separation.rs` - FastICA with deflation, PCA whitening, 3 nonlinearities LogCosh/Exp/Cube, kurtosis, negentropy, correlation metrics), Phase Vocoder (`phase_vocoder.rs` - STFT-based time-stretch, pitch-shift, spectrogram, robotize, whisperize effects with overlap-add synthesis), Compressive Sensing (`compressive_sensing.rs` - OMP greedy and ISTA/FISTA proximal gradient sparse recovery from underdetermined systems, random/DCT sensing matrices, RIP constant estimation), Zero Crossing Detector (`zero_crossing_detector.rs` - ZCR computation, frequency estimation, ZcrAnalyzer with energy, VoiceActivityDetector energy+ZCR with hangover, spectral centroid/flatness, modulation classification), Subspace Tracker (`subspace_tracker.rs` - PAST/OPAST adaptive rank-d subspace tracking, projection, projection error, dimension estimation, subspace angle). ~227+ DSP modules total, ~2500+ unit tests.

- **Batch 50 DSP Blocks** - Teager-Kaiser Energy Operator (`teager_kaiser_energy.rs` - TKEO instantaneous energy operator Psi[x(n)] = x(n)^2 - x(n-1)*x(n+1), AM/FM demodulation via TKEO, streaming processor, transient detection), Wigner-Ville Distribution (`wigner_ville_distribution.rs` - WVD/PWVD/SPWVD time-frequency analysis, analytic signal computation, instantaneous frequency extraction, 2D time-frequency surface with marginals), Lattice Filter (`lattice_filter.rs` - lattice and lattice-ladder filter structures, Levinson-Durbin recursion, Burg's method for AR estimation, PARCOR coefficients, step-up/step-down recursion, lattice predictor, lattice-ladder ARMA filter, PSD from reflection coefficients, Line Spectral Frequencies LSF), Prony's Method (`prony_method.rs` - parametric exponential signal modeling, standard and least-squares variants, matrix pencil method, companion matrix eigenvalue solver with QR iteration, MDL order estimation, parametric PSD), Cepstral Analysis (`cepstral_analysis.rs` - real/power/complex cepstrum with phase unwrapping, pitch detection via cepstral peak picking, homomorphic filtering for source-filter separation, MFCCs for speech/audio, mel filterbank, spectral envelope). 57 tests + 5 doctests. ~222+ DSP modules total, ~2443+ unit tests.

- **Batch 49 DSP Blocks** - Sigma-Delta Converter (`sigma_delta.rs` - sigma-delta modulator/demodulator with error-feedback EFB structure, 1st/2nd/3rd order noise shaping, multi-bit quantizer, CIC sinc³ decimation filter for ADC, theoretical SNR formula, NTF magnitude), Farrow Resampler (`farrow_resampler.rs` - Farrow polynomial structure for continuously variable fractional resampling, linear/quadratic/cubic interpolation via Horner evaluation, variable ratio on-the-fly, resample_to_length utility, no direct GR equivalent), Empirical Mode Decomposition (`empirical_mode.rs` - EMD via iterative sifting process + Hilbert-Huang Transform for non-stationary signal analysis, extracts IMFs Intrinsic Mode Functions, cubic spline envelope interpolation, instantaneous frequency and amplitude, no direct GR equivalent), Ambiguity Function (`ambiguity_function.rs` - radar waveform delay-Doppler analysis, full 2D ambiguity surface |χ(τ,ν)|² computation, zero-Doppler and zero-delay cuts, LFM chirp and Barker code generators, mainlobe width measurement, complex signal support, GR gr-radar ambiguity function equivalent), Dynamic Range Compressor (`dynamic_range_compressor.rs` - compressor/limiter/expander/noise gate with attack/release envelope follower, soft knee, RMS/peak detection, makeup gain, static compression curve for visualization, no direct GR equivalent). 61 tests + 5 doctests. ~217+ DSP modules total, ~2381+ unit tests.

- **Batch 48 DSP Blocks** - CFAR Detector (`cfar.rs` - Constant False Alarm Rate detector with CA-CFAR/GO-CFAR/SO-CFAR/OS-CFAR algorithms for 1D power data, Cfar2D for range-Doppler maps, GR gr-radar CFAR equivalent), Chirp-Z Transform (`chirp_z_transform.rs` - Bluestein's algorithm for Z-transform evaluation on arbitrary contours, zoom_fft for high-resolution narrow-band spectral analysis, no direct GR equivalent), Permute (`permute.rs` - vector index permutation with validation, inverse/compose/is_identity/bit_reversal, StreamPermuter for block-mode stream processing, GR custom permutation equivalent), Noise Reduction (`noise_reduction.rs` - spectral subtraction and Wiener noise reduction with MMSE filtering, Hann windowing, noise PSD estimation, GR gr-noise-cancel equivalent), Savitzky-Golay Filter (`savitzky_golay.rs` - polynomial smoothing and differentiation filter, least-squares fit, cached coefficients, 5/7/9-point presets, preserves peak shapes). 60 tests + 5 doctests. ~212+ DSP modules total, ~2320+ unit tests.
- **Batch 47 DSP Blocks** - Power Amplifier Model (`power_amplifier_model.rs` - nonlinear PA behavioral models: Saleh/Rapp/Ghorbani/polynomial/limiter with AM/AM and AM/PM distortion, P1dB compression point, GR gr::analog::distortion_* and gr-dpd PA models equivalent), Digital Pre-Distortion (`power_amplifier_dpd.rs` - memory polynomial DPD with LMS/RLS indirect learning architecture, NMSE performance metric, GR gr-dpd module equivalent), Median Filter (`median_filter.rs` - sliding-window median filter with dual-heap O(log n) algorithm, complex/weighted variants, 2D hybrid median for spectrogram denoising, GR out-of-tree median filters equivalent), CORDIC (`cordic.rs` - hardware-efficient CORDIC rotator for sin/cos, atan2, polar/rect conversions, NCO, iterative shifts-and-adds algorithm, GR gr::blocks::rotator_cc internal equivalent), Tagged File Sink (`tagged_file_sink.rs` - stream-tag-triggered I Q file segmentation for burst recording, GR gr::blocks::tagged_file_sink equivalent). 56 tests. ~207+ DSP modules total, ~2260+ unit tests.
- **Batch 46 DSP Blocks** - Linear Equalizer (`linear_equalizer.rs` - adaptive FIR equalizer with LMS, RLS, CMA, Kurtotic algorithms, training and decision-directed modes, constellation slicing, GR linear_equalizer equivalent), Trellis Coding (`trellis_coding.rs` - FSM-based trellis encoder + Viterbi decoder for TCM and convolutional codes, generator polynomial construction, GR gr::trellis equivalent), Protocol Formatter (`protocol_formatter.rs` - pluggable HeaderFormat trait, DefaultHeaderFormat with access code + repeated length + CRC, CounterHeaderFormat, protocol formatter/parser blocks, GR protocol_formatter_bb equivalent), CTCSS Squelch (`ctcss_squelch.rs` - CTCSS sub-audible tone encoder/decoder for FM repeater access, Goertzel detection, 38 standard EIA/TIA tones, GR ctcss_squelch_ff equivalent), OFDM Frame Equalizer (`ofdm_frame_equalizer.rs` - per-subcarrier ZF/MMSE equalization with pilot-based channel estimation, linear interpolation, GR ofdm_frame_equalizer_vcvc equivalent). ~202+ DSP modules total, ~238+ pipeline block types.
- **Batch 45 DSP Blocks** - PFB Arbitrary Resampler (`pfb_arb_resampler.rs` - polyphase filterbank arbitrary resampler with linear interpolation between filter branches, Blackman-Harris prototype, derivative filters, GR pfb_arb_resampler_ccf equivalent), Cyclic Autocorrelation (`cyclic_autocorrelation.rs` - cyclostationary signal analysis: cyclic autocorrelation function, spectral correlation function, feature detection, modulation classification CW/BPSK/QPSK/AM/FM/OFDM, GR CycloDSP equivalent), Random PDU Generator (`random_pdu_gen.rs` - random PDU test traffic generator with Poisson/Uniform/Fixed inter-arrival distributions, seeded LCG PRNG, traffic statistics, GR random_pdu_generator equivalent), MMSE Interpolator (`mmse_interpolator.rs` - 8-tap MMSE FIR fractional delay with 128-step mu quantization, windowed-sinc with Nuttall window, cubic and linear interpolation helpers, GR mmse_fir_interpolator_cc equivalent), Fractional Delay (`fractional_delay.rs` - Thiran all-pass IIR and Lagrange FIR fractional sample delay filters, maximally flat group delay, GR fractional_interpolator_cc equivalent). ~197+ DSP modules total, ~233+ pipeline block types.
- **Batch 41 DSP Blocks** - Decision Feedback Equalizer (`decision_feedback_equalizer.rs` - DFE with separate forward/feedback FIR, LMS/CMA/RLS adaptation, training mode with known sequence, BPSK/QPSK hard decisions), Probe Density (`probe_density.rs` - bit density and transition density measurement, exponential averaging, RunLengthAnalyzer for clock recovery/scrambler/DC balance), AIS Decoder (`ais_decoder.rs` - ITU-R M.1371 maritime protocol, message types 1-5/18/21/24, NRZI decode, 6-bit ASCII dearmor, CRC-16, AIVDM parsing), BCH Code (`bch_code.rs` - Bose-Chaudhuri-Hocquenghem encoder/decoder, standard codes (7,4,1)/(15,11,1)/(15,7,2)/(15,5,3)/(31,21,2), systematic LFSR encoding, brute-force correction up to t errors), Turbo Code (`turbo_code.rs` - parallel concatenated convolutional code, RSC constituent encoders, BCJR MAP iterative decoding, random and QPP 3GPP LTE interleavers, near-Shannon-limit FEC). ~177+ DSP modules total, ~213+ pipeline block types.
- **Batch 40 DSP Blocks** - Timing Error Detector (`timing_error_detector.rs` - Gardner, Mueller-Muller, Early-Late Gate, Zero-Crossing TEDs for symbol clock recovery, real and complex variants, streaming TimingErrorDetector block with TedAlgorithm enum), Constellation Demapper (`constellation_demapper.rs` - soft bit demapping via max-log-MAP, BPSK/QPSK fast-path functions, generic ConstellationDemapper for arbitrary constellations with hard/soft output), Eye Diagram (`eye_diagram.rs` - eye diagram generator with trace accumulation, mean_trace/envelope/eye_opening/timing_jitter_rms for signal quality visualization and ISI assessment), EVM Calculator (`evm_calculator.rs` - Error Vector Magnitude measurement: RMS/peak/percentile EVM in linear/dB/percent, streaming EvmCalculator with history and statistics), Valve (`valve.rs` - stream gating and flow control, Open/Closed/CountedBurst/Triggered modes, gate_signal and extract_segments utilities). ~172+ DSP modules total, ~208+ pipeline block types.
- **Batch 37 DSP Blocks** - Random Source (`random_source.rs` - xoshiro256** PRNG with uniform/Gaussian/range distributions, bit/byte/bipolar generation, seedable for reproducibility), Magnitude Squared (`magnitude_squared.rs` - complex-to-real |x|^2, magnitude, dB, RMS, peak power), Interp FIR (`interp_fir.rs` - interpolating FIR with polyphase decomposition, real + complex variants), Tag Share (`tag_share.rs` - stream tag propagation, range queries, offset scaling for rate changes), Exponentiate (`exponentiate.rs` - raise samples to arbitrary power, real/complex, integer/fractional, square/sqrt/reciprocal). ~157+ DSP modules total, ~193+ pipeline block types.
- **Batch 36 DSP Blocks** - IQ Balance (`iq_balance.rs` - IqBalanceCorrector for fixed gain/phase correction, estimate_iq_imbalance() for gain ratio and phase error measurement, AdaptiveIqBalance for LMS-based online correction), Peak to Average (`peak_to_average.rs` - papr_db/papr_db_complex for real/complex PAPR, crest_factor, papr_linear, PaprEstimator for streaming windowed measurement), Correlate Estimate (`correlate_estimate.rs` - cross_correlate/cross_correlate_complex time-domain correlation, find_delay for sample offset, autocorrelate, cross_correlate_normalized, correlation_coefficient), Bin Statistics (`bin_statistics.rs` - BinStatistics per-FFT-bin min/max/mean/variance accumulation, accumulate_max_hold for peak detection, dynamic_range per bin), Check LFSR (`check_lfsr.rs` - LfsrChecker for received-vs-expected LFSR comparison, synchronize() for offset detection, generate_reference utility, cumulative BER tracking). 60 tests. ~152+ DSP modules total, ~188+ pipeline block types.
- **Batch 35 DSP Blocks** - Additive Scrambler (`additive_scrambler.rs` - LFSR-based stream scrambling with DVB x^15+x^14+1 and WiFi x^7+x^4+1 presets, auto-reset period, bool and in-place APIs), Stretch (`stretch.rs` - signal normalization and range mapping, stretch/stretch_to_range/clip, StreamStretch for windowed streaming), Nlog10 (`nlog10.rs` - logarithmic scaling linear↔dB/dBm, nlog10/to_db/from_db/to_dbm/from_dbm, batch power_to_db/iq_to_db), DPLL (`dpll.rs` - second-order digital PLL with PI loop filter, Dpll carrier tracking with from_bandwidth, BinaryDpll for clock recovery from edge transitions), Endian Swap (`endian_swap.rs` - byte order conversion 16/32/64-bit, typed swap_i16/u16/i32/u32/f32/f64, reverse_bits for bit-level reversal). 70 tests. ~147+ DSP modules total, ~183+ pipeline block types.
- **Batch 34 DSP Blocks** - Patterned Interleaver (`patterned_interleaver.rs` - custom-pattern stream interleaving/deinterleaving of N streams, f64 and bytes), Bitwise Ops (`bitwise_ops.rs` - XOR/AND/OR/NOT for byte and boolean streams, _const/_inplace variants, hamming_distance/popcount/parity), Peak Hold (`peak_hold.rs` - signal peak tracking with exponential decay, PeakHold/AbsPeakHold/PeakHoldDb variants), Multiply Matrix (`multiply_matrix.rs` - complex and real matrix-vector multiplication, identity/scalar/diagonal/from_rows constructors), GLFSR Source (`glfsr_source.rs` - Galois and Fibonacci LFSR PN sequence generators, maximal-length polynomials 2-31 bits, bits/bools/bipolar output). 72 tests. ~142+ DSP modules total, ~178+ pipeline block types.
- **Batch 32 DSP Blocks** - Stream to Streams (`stream_to_streams.rs` - round-robin demux/mux by index, I/Q deinterleave/interleave, supports real/complex/bytes), Argmax (`argmax.rs` - find max/min index in vectors, ArgmaxBlock for windowed processing, top_k functions), Threshold (`threshold.rs` - hysteresis threshold detector with configurable low/high thresholds, rising/falling edge detection), PDU Filter (`pdu_filter.rs` - metadata-based filtering and routing with Require/Reject modes, first-match PduRouter), Regenerate BB (`regenerate_bb.rs` - bit regeneration pulse stretcher with guard intervals, PulseGenerator for one-shot pulses). 70 tests. GNU Radio equivalents: `stream_to_streams`, `argmax`, `threshold_ff`, `pdu_filter`, `regenerate_bb`. ~137+ DSP modules total, ~173+ pipeline block types.
- **Batch 31 DSP Blocks** - Keep M in N (`keep_m_in_n.rs` - selective M-of-N sample extraction with configurable offset for OFDM subcarrier extraction/guard removal), Phase Unwrap (`phase_unwrap.rs` - continuous phase tracking removing 2-pi discontinuities, configurable tolerance), Moving Average (`moving_average_block.rs` - O(1) sliding window mean filter, complex + real variants), Probe Avg Mag Sqrd (`probe_avg_mag_sqrd.rs` - exponential power measurement with threshold-based carrier sensing, pass-through design), Constellation Soft Decoder (`constellation_soft_decoder.rs` - LLR soft-decision demapping for BPSK/QPSK/8PSK/16QAM/64QAM). 80 tests + 6 doctests. ~132+ DSP modules total, ~168+ pipeline block types.
- **Batch 30 DSP Blocks** - TCP Source/Sink (`tcp_source_sink.rs` - reliable network IQ streaming over TCP with buffered writes and reconnection), FFT Filter (`fft_filter.rs` - frequency-domain FIR filtering via overlap-save/overlap-add for long kernels), WAV Source/Sink (`wav_source_sink.rs` - PCM audio file I/O supporting 8/16/24/32-bit and float32/64 formats), Channel Model (`channel_model.rs` - composable AWGN + frequency offset + timing offset + multipath with preset scenarios), BER Tool (`ber_tool.rs` - bit/symbol/frame error rate measurement with confidence intervals and error pattern analysis). 75 tests + 5 doctests. ~127+ DSP modules total, ~163+ pipeline block types.
- **Batch 29 DSP Blocks** - AM Demodulator (`am_demod.rs` - envelope detection + DC block + audio lowpass, GNU Radio `am_demod_cf`), Hilbert Transform (`hilbert.rs` - Hamming-windowed FIR real-to-analytic conversion, group delay compensation, GNU Radio `hilbert_fc`), Single-Pole IIR (`single_pole_iir.rs` - EMA smoothing H(z)=α/(1-(1-α)z⁻¹), real + complex variants, alpha or tau construction, GNU Radio `single_pole_iir_filter_ff`/`_cc`), Complex to Mag/Phase (`complex_to_mag_phase.rs` - simultaneous extraction + inverse MagPhaseToComplex, GNU Radio `complex_to_mag_phase`), Tagged Stream PDU (`tagged_stream_pdu.rs` - tagged stream ↔ PDU bridging with metadata, GNU Radio `tagged_stream_to_pdu`/`pdu_to_tagged_stream`). 77 tests + 5 doctests. 4 pipeline blocks. ~122+ DSP modules total, ~158+ pipeline block types.
- **Batch 28 DSP Blocks** - Rail (real clamp + ComplexRail component/magnitude clipping), UDP Source/Sink (network IQ streaming with sequence headers, packet loss detection), Repeat (zero-order hold upsample), KeepOneInN (decimate without filtering), Head (pass first N), SkipHead (discard first N), Integrate (running sum + decimation), WindowedIntegrate (sliding sum), LeakyIntegrate (exponential IIR). 70 tests + 6 doctests. 2 pipeline blocks. ~117+ DSP modules total, ~154+ pipeline block types.
- **Batch 27 DSP Blocks** - Phase modulators (PhaseModulator instantaneous + ContinuousPhaseModulator accumulated), VCO (VcoC complex + VcoF real frequency-controlled oscillators), File I/O (IqFileReader/Writer with cf64/cf32/ci16/ci8/cu8 auto-detection), Message system (MessageStrobe periodic PDU + Message/MessageFilter/MessageCounter/MessageBurst), Throttle (rate-limiting + ThroughputMonitor). 59 tests + 5 doctests. 6 pipeline blocks.
- **Batch 26 DSP Blocks** - Adaptive IIR notch filter (AdaptiveNotch + FixedNotch) for narrowband interference removal, energy-based SignalDetector with noise floor tracking, PreambleGenerator (Alternating, Barker codes, PN sequences, Zadoff-Chu), PacketEncoder with sync word/length/CRC-8/16/32/whitening, ArbitraryResampler using cubic Hermite interpolation for non-rational sample rate conversion.
- **DSP Batch 25** - Five modules: Decimating FIR (`decimating_fir.rs` - combined lowpass FIR + integer decimation, auto-designed Hamming sinc, streaming operation), Interleaved Conversions (`interleaved.rs` - i16/i8/u8/f32 ↔ Complex64 for SDR hardware: USRP, HackRF, RTL-SDR, PlutoSDR), AFC (`afc.rs` - closed-loop frequency tracking, phase/cross-product discriminators, second-order loop filter, FrequencyEstimator one-shot), Moving Average Decimator (`moving_avg_decim.rs` - boxcar averaging + decimation, sum mode, PowerAvgDecim for dB power), DC Blocker (`dc_blocker.rs` - single-pole IIR highpass, configurable pole/cutoff, RealDcBlocker). 60 tests. 6 pipeline blocks.
- **DSP Batch 24** - Five modules: DTMF Decoder (`dtmf.rs` - Goertzel filter bank, 4x4 tone grid, twist checking, ambiguity rejection), Noise Blanker (`noise_blanker.rs` - impulse detection with running average power, Zero/Hold/Interpolate blank modes, warmup period), Stream Arithmetic (`stream_arithmetic.rs` - element-wise add/subtract/multiply/divide for IQ streams), Probe Avg Mag² (`probe_power.rs` - running power measurement, threshold gating, crest factor), Envelope Detector (`envelope_detector.rs` - Magnitude/MagnitudeSquared/Smoothed/PeakHold modes, AM demodulator with DC removal). 52 tests. 7 pipeline blocks.
- **DSP Batch 23** - Five modules: Feedforward AGC (`feedforward_agc.rs` - non-causal AGC with lookahead window, max gain limiting), Vector Insert/Remove (`vector_insert.rs` - periodic pilot/sync word insertion + symmetric removal), Cyclic Prefix (`cyclic_prefix.rs` - OFDM CP add/remove, standard configs: WiFi/LTE/DVB-T, cyclic suffix), File Meta I/O (`file_meta.rs` - self-describing IQ files with JSON header, cf64/cf32/ci16), PN Sync (`pn_sync.rs` - LFSR m-sequence/Gold code gen, normalized cross-correlator, despread). 46 tests. 8 pipeline blocks.
- **DSP Batch 22** - Four modules: CPM Modulation (`cpm.rs` - GMSK/GFSK/MSK constant-envelope modulation, non-coherent demod), Dynamic Channel Model (`dynamic_channel.rs` - composite time-varying: multipath fading + CFO/SRO drift + AWGN, presets), Polar Codes (`fec/polar.rs` - 5G NR encoder/decoder, SC algorithm, Bhattacharyya construction), Burst Tagger (`burst_tagger.rs` - power-based burst detection, tagged stream mux/align). 48 tests. 7 pipeline blocks.
- **DSP Batch 21** - Four modules: PDU Conversion (`pdu.rs` - PDU↔tagged stream, message debug formatting), OFDM Channel Estimation (`ofdm_channel_est.rs` - LS/smoothed estimation, ZF/MMSE equalization), SSB Modem (`ssb_modem.rs` - Hilbert transform, USB/LSB phasing method, BFO demod), Wavelet Analysis (`wavelet.rs` - DWT Haar/Db4/Sym4, soft/hard threshold denoising). 52 tests. 7 pipeline blocks.
- **DSP Blocks Round 12** - Five modules: Quadrature Demodulator (FM discriminator), Access Code Detector (bit-level sync word with Hamming distance), Fractional Resampler (MMSE interpolating for irrational ratios), FLL Band-Edge (coarse frequency sync), Type Conversions (ComplexToMag/Arg/Real, RealToComplex). 52 new tests. All wired into pipeline builder with property editors and block metadata.
- **DSP Blocks Round 11** - Five modules: Frequency Xlating FIR Filter (mixing + FIR + decimation), FM Pre-emphasis/De-emphasis (US 75us/EU 50us IIR), CTCSS Tone Squelch (Goertzel-based 38-tone detection), Stream Control (Head/SkipHead/Throttle), Log Power FFT (windowed FFT with dB averaging). 47 new tests. All wired into pipeline builder.
- **DSP Blocks Round 10** - Constellation Receiver (`constellation_receiver.rs` - combined AGC + Costas loop + symbol demapper for BPSK/QPSK/8PSK, soft/hard decisions). 9 new tests.
- **DSP Blocks Round 9** - Three modules: Costas Loop (`costas_loop.rs` - decision-directed carrier recovery for BPSK/QPSK/8PSK), Goertzel Algorithm (`goertzel.rs` - efficient single-frequency DFT, MultiGoertzel, full DTMF detector with all 16 digits), Stream Tags (`stream_tags.rs` - metadata propagation with TagStore, range queries, well-known keys). 34 new tests.
- **DSP Blocks Round 8** - Four modules: CIC Filter (`filters/cic.rs` - multiplier-free decimator/interpolator for high sample rate conversion, compensation filter design), Adaptive Filters (`filters/adaptive.rs` - LMS/NLMS/RLS for equalization/echo cancellation), Burst Detector (`burst_detector.rs` - power-based SOB/EOB detection with hysteresis and holdoff), Colored Noise (`noise.rs` - white/pink/brown/blue/violet generators with AWGN helpers). 38 new tests.
- **DSP Blocks Round 7** - PLL (`pll.rs` - second-order PLL with PI loop filter and lock detection), DC Blocker (single-pole IIR highpass implementing Filter trait), Sample Delay (circular buffer implementing Filter trait). 11 new tests.
- **GNU Radio Parity Batch 16** - PFB Clock Sync (`pfb_clock_sync.rs` - polyphase filterbank timing recovery with RRC matched filter, derivative filter TED, PI loop), Header/Payload Demux (`header_payload_demux.rs` - variable-length packet demux with configurable header format/endianness/length field), AX.25 Decoder (`ax25.rs` - amateur radio protocol parser for callsigns, SSIDs, digipeater paths, APRS detection), RMS Power (`rms.rs` - IIR-smoothed RMS measurement and normalization), Correlate & Sync (`correlate_sync.rs` - CFAR cross-correlation frame synchronizer with bit-level correlator). 47 tests.
- **GNU Radio Parity Batch 15** - Binary Slicer (`binary_slicer.rs` - soft-to-hard bit decisions), HDLC Framer/Deframer (`hdlc.rs` - bit-stuffing, CRC-16/CCITT FCS for AX.25/APRS), Clock Recovery M&M (`clock_recovery_mm.rs` - Mueller & Müller timing recovery), FM Receiver (`fm_receiver.rs` - NBFM/WBFM composite blocks with de-emphasis), Symbol Sync (`symbol_sync.rs` - PFB-based with Gardner/ZC/M&M TEDs). 41 tests.
- **GNU Radio Parity Batch 14** - Delay (`delay.rs` - VecDeque sample delay), Multiply/MultiplyConst (`multiply.rs` - element-wise and scalar multiply), Pack/Unpack/Repack Bits (`bit_packing.rs` - MSB-first bit packing), Power Squelch (`power_squelch.rs` - power-based gating with hysteresis), Stream Mux/Demux (`stream_mux.rs` - round-robin interleaving), Plateau Detector (`plateau_detector.rs` - flat region detection for OFDM sync). 56 tests.
- **GNU Radio Parity Batch 13** - PFB Synthesizer (`pfb_synthesizer.rs` - multi-channel to wideband via IFFT + polyphase), Moving Average (`filters/moving_average.rs` - efficient circular buffer moving average), Sample Ops (`sample_ops.rs` - keep-one-in-N decimation and sample repeat). 24 tests.
- **DSP Blocks Round 6** - Four more r4w-core modules: NCO/VCO (`nco.rs` - phase-continuous oscillator, FM modulator/demodulator with arctangent discriminator), SNR Estimator (`snr_estimator.rs` - M2M4 moment-based, split-spectrum, signal+noise, EVM methods), Symbol Mapper (`symbol_mapping.rs` - BPSK/QPSK/8PSK/16QAM/64QAM with Gray coding, hard/soft LLR demapping), Measurement Probes (`probe.rs` - PowerProbe with PAPR, EvmProbe, ConstellationProbe with IQ imbalance, FreqOffsetProbe, RateProbe). 43 new unit tests.
- **DSP Blocks Round 5** - Pipeline builder integration: wired Scrambler, OfdmModulator, and DifferentialEncoder to real r4w-core implementations. Added OFDM and DifferentialEncoder block metadata.
- **DSP Blocks Round 4** - Four more r4w-core modules: Correlator (`correlator.rs` - cross-correlation sync word detector with Barker-7/11/13 codes, dynamic threshold, holdoff, phase estimation), LFSR Scrambler (`scrambler.rs` - additive/multiplicative scrambling for WiFi/DVB-S2/Bluetooth/V.34 protocols), Differential Encoder/Decoder (`differential.rs` - symbol-domain and complex-domain DPSK for DBPSK/DQPSK/D8PSK), Packet Framing (`packet_framing.rs` - formatter/parser with sync word detection, configurable headers, CRC-16/32, AX.25/ISM presets). 41 new unit tests.
- **DSP Blocks Round 3** - Two more r4w-core modules: OFDM modulator/demodulator (`ofdm.rs` - WiFi-like, DVB-T 2K, simple configs with CP insertion/removal, pilot insertion/extraction), Polyphase Filter Bank channelizer (`pfb_channelizer.rs` - windowed-sinc prototype filter design, stateful polyphase sub-filtering, M-point IFFT). 13 new unit tests.
- **GNSS Pipeline Blocks** - Added GNSS category (teal) to the visual pipeline builder with two blocks: `GnssScenarioSource` (multi-satellite IQ generation with preset selection, receiver position, sample rate, duration) and `GnssAcquisition` (PCPS-based signal acquisition with configurable Doppler search and detection threshold). GnssOpenSky preset pipeline template connects scenario source to acquisition to IQ output. Full property editors, block metadata with formulas and standards references, and processing logic integrated with r4w-core GNSS engine.
- **DSP Blocks Round 2** - Four more r4w-core modules: Adaptive Equalizer (`equalizer.rs` - LMS/CMA/Decision-Directed with Filter trait), Reed-Solomon codec (`fec/reed_solomon.rs` - GF(2^8) encoder/decoder with BM/Chien/Forney, CCSDS/DVB presets), Signal Source (`signal_source.rs` - Tone/TwoTone/Chirp/Noise/Square/DC/Impulse generators), Power Squelch (`squelch.rs` - signal gating by power level with ramp transitions). Pipeline builder Equalizer block wired to real implementations. 34 new unit tests.
- **Standalone DSP Blocks** - Six new r4w-core modules implementing GNU Radio-equivalent functionality: AGC (`agc.rs` - three variants: Agc/Agc2/Agc3 matching agc_cc/agc2_cc/agc3_cc), CRC engine (`crc.rs` - CRC-8/16/32/32C with table-based lookup), Convolutional encoder + Viterbi decoder (`fec/convolutional.rs` - configurable K/rate with hard/soft decision, NASA K=7 and 3GPP K=9 presets), Costas loop carrier recovery (`carrier_recovery.rs` - BPSK/QPSK/8PSK with 2nd-order PI loop filter), Mueller & Muller clock recovery (`clock_recovery.rs` - symbol timing with interpolation), FreqXlatingFirFilter (`clock_recovery.rs` - NCO mixing + FIR + decimation). All implement `Filter` trait for pipeline interoperability. 39 new unit tests.
- **Pipeline Builder DSP Integration** - Wired existing pipeline builder stub blocks (AGC, CarrierRecovery, TimingRecovery, FecEncoder, CrcGenerator) to real r4w-core implementations. Test panel now produces live output for all synchronization, FEC, and integrity blocks.
- **Meshtastic Interop Crypto Fix** - Fixed 7 critical compatibility bugs for real-device interoperability (MESH-012 through MESH-015). Nonce now matches firmware CryptoEngine.cpp: `[pkt_id_u64_LE, node_id_u32_LE, zeros]`. PSK used directly as AES key (no SHA-256 derivation). Channel hash uses XOR fold (`xorHash(name)^xorHash(psk)`). Removed MIC for CTR mode. Fixed Position proto field tags (fix_quality=17, fix_type=18, sats_in_view=19). Added missing PortNums (Alert, KeyVerification, etc.). Added Data.bitfield, User.public_key. CLI features: `crypto`, `meshtastic-interop`. Removed sha2/hmac dependencies.
- **Type-Aware Test Panel** - Test panel adapts to selected block's input type. Shows BitPattern options (Random, AllZeros, AllOnes, Alternating, Prbs7) for Bits inputs, SymbolPattern (Random, Sequential, AllZero, Alternating) for Symbols, IqPattern (Noise, Tone, Chirp, Impulse) for IQ. Block output caching enables pipeline chaining (use previous block's output as input).
- **Typed Port Support** - Port type system for pipeline connections. Types: Bits (blue), Symbols (purple), IQ (orange), Real (cyan), Any (gray). Visual feedback during connection: compatible ports brighten, incompatible show red with X. Type mismatch warnings in validation. Port types shown in Properties panel.
- **Pipeline Builder Menu Bar** - Traditional dropdown menu bar replacing flat toolbar. Menus: File (New/Load/Save/Export), Edit (Select All/Delete/Validate), View (panels, arrows, zoom), Options (snap, auto-connect, cascade drag, connection styles), Layout (modes), Presets. Resizable test panel with persistent height tracking.
- **Block Metadata System** - Comprehensive documentation for pipeline blocks (`block_metadata.rs`). Each block has: implementation location (file:line with "View Code" to open in VS Code), mathematical formulas with variable explanations, unit tests with "Run" buttons, performance info (complexity, SIMD/GPU support), standards references with links. Properties panel shows collapsible Documentation/Formulas/Implementation/Tests/Performance/Standards sections.
- **TX/RX/Channel Pipeline Separation** - Waveform specs v1.1 format with separate `tx`, `rx`, and `channel` sections. Load menu offers TX/RX/Loopback options. Block ID conventions: TX(1-99), RX(100-199), Channel(200-299). Loopback mode auto-connects TX → Channel → RX.
- **Demodulator Blocks** - Added PskDemodulator, QamDemodulator, FskDemodulator block types with property editors and YAML serialization.
- **Updated Waveform Specs** - All specs (bpsk, qpsk, fsk, lora, cw) updated to v1.1 format with complete TX, RX, and Channel pipeline definitions.
- **Visual Pipeline Builder** - Graphical signal processing pipeline designer (`r4w-gui/views/pipeline_wizard.rs`). 40+ block types in 10 categories (Source, Coding, Mapping, Modulation, Filtering, Rate Conversion, Synchronization, Impairments, Recovery, Output). Interactive canvas with zoom/pan, Bezier curve connections, click-to-connect ports. 12 preset templates (BPSK/QPSK/16-QAM/LoRa/OFDM/FSK/DSSS/DMR transmitters, TX-Channel-RX systems, Parallel I/Q demo). Auto-layout with topological sorting, pipeline validation (cycle detection, unconnected ports), YAML export, snap-to-grid, keyboard shortcuts.
- **Enhanced Waveform Wizard** - Filtering step (FIR/IIR options, sample rate conversion), Synchronization step (timing/carrier recovery, AGC, equalization), Frame Structure step (TDMA, packet formats, CRC). 11 wizard steps total.
- **Generic Filter Trait Architecture** - Extensible filter framework with `Filter`, `RealFilter`, `FirFilterOps`, `FrequencyResponse` traits. FirFilter supports lowpass/highpass/bandpass/bandstop with multiple window functions (Blackman, Hamming, Hann, Kaiser). Kaiser window design with auto β and order calculation. Pulse shaping filters (RRC, RC, Gaussian) now implement all traits for polymorphic usage and frequency response analysis. Special-purpose filters: `moving_average()`, `differentiator()`, `hilbert()`.
- **Parks-McClellan Equiripple FIR Design** - Optimal FIR filter design using Remez exchange algorithm. `RemezSpec` builder with `lowpass()`, `highpass()`, `bandpass()`, `bandstop()`, `differentiator()`, `hilbert()`. Configurable passband/stopband weights, `estimate_order()` for filter sizing. Produces equiripple (Chebyshev) error distribution for minimum filter length.
- **Polyphase Sample Rate Conversion** - Efficient decimation and interpolation using polyphase decomposition. `PolyphaseDecimator` for M-fold downsampling, `PolyphaseInterpolator` for L-fold upsampling, `Resampler` for rational L/M conversion (e.g., 48kHz↔44.1kHz), `HalfbandFilter` for 2x with ~50% compute savings. All include built-in anti-aliasing/anti-imaging filters.
- **IIR Filters** - IirFilter with cascaded biquad (SOS) implementation for numerical stability. Butterworth (maximally flat), Chebyshev Type I (equiripple passband), Chebyshev Type II (equiripple stopband), Bessel (maximally flat group delay). Bilinear transform with frequency pre-warping. Analysis: `frequency_response()`, `magnitude_response_db()`, `phase_response()`, `group_delay_at()`, `is_stable()`.
- **Real Galileo E1 ICD Codes** - Replaced simulated LFSR codes with real memory codes from Galileo OS SIS ICD v2.1. E1B (data) and E1C (pilot) codes for PRN 1-50, plus E1C secondary code. New API: `GalileoE1CodeGenerator::new_e1b(prn)`, `new_e1c(prn)`, `secondary_code()`. Source: GNSS-matlab repository. Binary size: ~330 KB embedded.
- **Unified IqFormat Module** - New `r4w_core::io::IqFormat` enum provides single source of truth for IQ sample formats across the codebase. Supports cf64 (16 bytes), cf32/ettus (8 bytes), ci16/sc16 (4 bytes), ci8 (2 bytes), cu8/rtlsdr (2 bytes). Replaces scattered format handling in CLI, benchmark, SigMF, and GUI code. Full roundtrip I/O, SigMF datatype strings, and comprehensive string aliases.
- **SP3 Precise Ephemeris & IONEX TEC Maps** - cm-level satellite positions from CODE FTP server SP3 files. Global ionospheric TEC grids from IONEX files for accurate ionospheric delay. Satellite clock corrections extracted from SP3 (microsecond-level biases displayed per-PRN). CLI: `--sp3 <path>`, `--ionex <path>`. Requires `--features ephemeris`.
- **Multi-Format IQ Output** - USRP/Ettus-compatible interleaved float32 format (`--format ettus`), plus sc16 (signed 16-bit) for compact storage. Formats: cf64 (16 bytes/sample), cf32/ettus (8 bytes), ci16/sc16 (4 bytes), ci8 (2 bytes), cu8/rtlsdr (2 bytes).
- **Corrected C/N0 Link Budget** - Fixed C/N0 calculation: EIRP(dBW) - FSPL + Gr + 204 dBW/Hz. Realistic 30-45 dB-Hz values for GPS L1 at varying elevations.
- **GNSS Scenario Enhancements** - Real-world PRN lookup tables (GPS 31 SVs from NAVCEN, Galileo 24 SVs from GSC, GLONASS 24 SVs). Galileo orbit calibration (Walker 24/3/1, RAAN/M0 offsets at 2026 epoch). Per-sample Doppler interpolation. GPS time conversion. CLI: `--lat/--lon/--alt`, `--time`, `--signals` filter, `--export-preset`, `--config` YAML file. Auto-discovery of visible satellites.
- **Emergency Distress Beacons** - 121.5 MHz / 243 MHz swept-tone AM beacon waveforms per ICAO Annex 10. ELT (aircraft), EPIRB (maritime), PLB (personal), Military (243 MHz). Factory names: ELT-121.5, EPIRB-121.5, PLB-121.5, Beacon-243.
- **GNSS IQ Scenario Generator** - Multi-satellite composite IQ generation with Keplerian orbits, Klobuchar ionosphere, Saastamoinen troposphere, multipath presets, antenna patterns, orbital Doppler, and FSPL. CLI: `r4w gnss scenario`. GUI: GNSS Simulator view with sky plot and C/N0 bars. Presets: OpenSky, UrbanCanyon, Driving, Walking, HighDynamics, MultiConstellation.
- **Generic Scenario Engine** - Reusable multi-emitter IQ composition framework in r4w-sim with `Emitter` trait, trajectory models, per-emitter Doppler/path-loss/channel, and composite noise.
- **Coordinate Library** - ECEF/LLA/ENU types with WGS-84 conversions, look angles, range rate, FSPL in r4w-core.
- **GNSS Environment Models** - Keplerian orbit propagation (GPS/Galileo/GLONASS nominal orbits), Klobuchar ionospheric delay, Saastamoinen tropospheric delay, multipath presets (OpenSky/Suburban/UrbanCanyon/Indoor), antenna patterns (Isotropic/Patch/ChokeRing).
- **Jupyter Workshops** - 12 interactive tutorials including GNSS scenario generation, environment models, precise ephemeris, and signal verification (`notebooks/09_*.ipynb` through `notebooks/12_*.ipynb`)
- **GNSS Waveforms** - GPS L1 C/A, GPS L5, GLONASS L1OF, Galileo E1 with PRN code generation, FFT-based PCPS acquisition, DLL/PLL tracking loops, navigation data encoding/decoding, and `r4w gnss` CLI subcommand
- **Enhanced Channel Simulation** - Jake's/Clarke's Doppler model, Tapped Delay Line (TDL) multipath with 3GPP profiles (EPA, EVA, ETU)
- **CLI Analysis Tools** - `r4w analyze` subcommands: spectrum, waterfall, stats, peaks
- **Signal Gallery** - 23 PNG images: constellations, spectra, channel effects (`gallery/`)
- **Jupyter Notebooks** - 8 interactive tutorials with Python wrapper (`notebooks/`)
- **Deployment Options** - Docker image, cargo-binstall manifest, GitHub Actions release workflow
- **CLI Enhancements** - Record/Playback (SigMF), Prometheus metrics, shell completions, waveform comparison
- **Example Gallery** - Getting-started examples for modulation, channels, LoRa, mesh networking
- **GitHub Actions CI** - Automated testing, cross-platform builds, WASM builds, performance regression tracking
- **Mesh CLI Commands** - `r4w mesh` subcommands for LoRa mesh networking (status, send, neighbors, simulate, info)
- **Mesh Networking Module** - Full mesh stack with MeshtasticNode, FloodRouter, CSMA/CA MAC, LoRaMesh integration
- **Physical Layer Architecture** (Session 18):
  - Multi-clock timing model (SampleClock, WallClock, HardwareClock, GPS/PTP SyncedTime)
  - Real-time primitives (lock-free SPSC ring buffers, buffer pools, RT thread spawning)
  - YAML configuration system with hardware profiles
  - Enhanced HAL traits (StreamHandle, TunerControl, ClockControl)
- Added TETRA, DMR, and 3G ALE waveform implementations
- Enhanced waveform-spec schema with TDMA/FDMA, radar, classified components
- Renamed project to "R4W - Rust for Waveforms"
- Created platform vision with Waveform Developer's Guide
- Documented FPGA integration architecture
- Remote Lab for distributed testing on Raspberry Pis

## Requirements Management

This project uses AIDA for requirements tracking. **Do NOT maintain a separate REQUIREMENTS.md file.**

Requirements database: `requirements.yaml`

### CLI Commands
```bash
aida list                              # List all requirements
aida list --status draft               # Filter by status
aida show <ID>                         # Show requirement details (e.g., FR-0042)
aida add --title "..." --description "..." --status draft  # Add new requirement
aida edit <ID> --status completed      # Update status
aida comment add <ID> "..."            # Add implementation note
```

### During Development
- When implementing a feature, update its requirement status
- Add comments to requirements with implementation decisions
- Create child requirements for sub-tasks discovered during implementation
- Link related requirements with: `aida rel add --from <FROM> --to <TO> --type <Parent|Verifies|References>`

### Session Workflow
If you work conversationally without explicit /aida-req calls, use `/aida-capture` at session end to review and capture any requirements that were discussed but not yet added to the database.

## Code Traceability

### Inline Trace Comments
When implementing requirements, add inline trace comments:

```rust
// trace:FR-0042 | ai:claude
fn implement_feature() {
    // Implementation
}
```

Format: `// trace:<SPEC-ID> | ai:<tool>[:<confidence>]`

### Commit Message Format
**Standard format:**
```
[AI:tool] type(scope): description (REQ-ID)
```

**Examples:**
```
[AI:claude] feat(auth): add login validation (FR-0042)
[AI:claude:med] fix(api): handle null response (BUG-0023)
chore(deps): update dependencies
docs: update README
```

**Rules:**
- `[AI:tool]` - Required when commit includes AI-assisted code
- `type` - Required: feat, fix, docs, style, refactor, perf, test, build, ci, chore, revert
- `(scope)` - Optional: component or area affected
- `(REQ-ID)` - Required for feat/fix commits, optional for chore/docs

**Confidence levels:**
- `[AI:claude]` - High confidence (implied, >80% AI-generated)
- `[AI:claude:med]` - Medium (40-80% AI with modifications)
- `[AI:claude:low]` - Low (<40% AI, mostly human)

**Configuration:**
Set `AIDA_COMMIT_STRICT=true` to reject non-conforming commits, or create `.aida/commit-config`.

## Claude Code Skills

This project uses AIDA requirements-driven development:

### /aida-req
Add new requirements with AI evaluation:
- Interactive requirement gathering
- Immediate database storage with draft status
- Background AI evaluation for quality feedback
- Follow-up actions: improve, split, link, accept

### /aida-implement
Implement requirements with traceability:
- Load and display requirement context
- Break down into child requirements as needed
- Update requirements during implementation
- Add inline traceability comments to code

### /aida-capture
Review session and capture missed requirements:
- Scan conversation for discussed features/bugs/ideas
- Identify implemented work not yet in requirements database
- Prompt to add missing requirements or update statuses
- Use at end of conversational sessions as a safety net
