//! # LoRa Core DSP Library
//!
//! This crate provides core Digital Signal Processing (DSP) algorithms for
//! implementing LoRa (Long Range) modulation and demodulation in software.
//!
//! ## Overview
//!
//! LoRa uses Chirp Spread Spectrum (CSS) modulation, which provides excellent
//! noise immunity and long-range communication at low data rates. This library
//! implements the full LoRa PHY layer including:
//!
//! - **Chirp Generation**: Create up-chirps and down-chirps for modulation
//! - **CSS Modulation**: Convert symbols to chirped waveforms
//! - **CSS Demodulation**: Extract symbols from received I/Q samples
//! - **Synchronization**: Preamble detection, CFO estimation, timing recovery
//! - **Coding**: Whitening, interleaving, Hamming FEC, Gray coding
//!
//! ## Signal Flow
//!
//! ```text
//! TX: Data → Whitening → Hamming FEC → Interleave → Gray Code → CSS Mod → I/Q
//! RX: I/Q → Sync → CSS Demod → Gray Decode → De-interleave → FEC → De-whiten → Data
//! ```
//!
//! ## Example
//!
//! ```rust,no_run
//! use r4w_core::{LoRaParams, Modulator, Demodulator};
//!
//! // Configure LoRa parameters using the builder pattern
//! let params = LoRaParams::builder()
//!     .spreading_factor(7)
//!     .bandwidth(125_000)
//!     .coding_rate(1)
//!     .build();
//!
//! // Create modulator and generate a packet
//! let mut modulator = Modulator::new(params.clone());
//! let payload = b"Hello LoRa!";
//! let samples = modulator.modulate(payload);
//!
//! // Demodulate (in real use, these would be received I/Q samples)
//! let mut demodulator = Demodulator::new(params);
//! let decoded = demodulator.demodulate(&samples);
//! ```

pub mod adaptive_notch;
pub mod afc;
pub mod agent;
pub mod agc;
pub mod ais_decoder;
pub mod am_demod;
pub mod aprs_decoder;
pub mod arbitrary_resampler;
pub mod argmax;
pub mod arq_engine;
pub mod arithmetic;
pub mod carrier_recovery;
pub mod channel_model;
pub mod channel_sounder;
pub mod clock_recovery;
pub mod clock_recovery_mm;
pub mod coordinates;
pub mod correlator;
pub mod crc;
pub mod dc_blocker;
pub mod decision_feedback_equalizer;
pub mod decimating_fir;
pub mod differential;
pub mod equalizer;
pub mod additive_scrambler;
pub mod analysis;
pub mod anti_jam;
pub mod bch_code;
pub mod beamformer;
pub mod benchmark;
pub mod ber_tool;
pub mod binary_slicer;
pub mod bin_statistics;
pub mod bit_packing;
pub mod bitwise_ops;
pub mod burst_detector;
pub mod burst_shaper;
pub mod burst_tagger;
pub mod cfo_estimator;
pub mod check_lfsr;
pub mod chirp;
pub mod chunks_to_symbols;
pub mod coding;
pub mod config;
pub mod complex_to_mag_phase;
pub mod conjugate;
pub mod constellation_receiver;
pub mod constellation_demapper;
pub mod constellation_soft_decoder;
pub mod costas_loop;
pub mod cpm;
pub mod cyclic_prefix;
pub mod dab_plus;
pub mod delay;
pub mod demodulation;
pub mod dpll;
pub mod dtmf;
pub mod dynamic_channel;
pub mod endian_swap;
pub mod envelope_detector;
pub mod evm_calculator;
pub mod exponentiate;
pub mod eye_diagram;
pub mod feedforward_agc;
pub mod file_meta;
pub mod file_source_sink;
pub mod acars_decoder;
pub mod access_code_detector;
pub mod ax25;
pub mod correlate_estimate;
pub mod correlate_sync;
pub mod fec;
pub mod fft_filter;
pub mod fft_utils;
pub mod filters;
pub mod fll_band_edge;
pub mod float_to_complex;
pub mod fm_emphasis;
pub mod frequency_modulator;
pub mod fm_receiver;
pub mod frame_sync;
pub mod freq_xlating_fir;
pub mod glfsr_source;
pub mod goertzel;
pub mod gps_time;
pub mod head;
pub mod hw_impairments;
pub mod integrate;
pub mod interleaved;
pub mod interp_fir;
pub mod io;
pub mod iq_balance;
pub mod kalman_filter;
pub mod keep_m_in_n;
pub mod ldpc_codec;
pub mod log_power_fft;
pub mod lpi_metrics;
pub mod magnitude_squared;
pub mod map_bb;
pub mod message_strobe;
pub mod mimo;
pub mod modulation;
pub mod moving_average;
pub mod moving_avg_decim;
pub mod moving_minmax;
pub mod multiply;
pub mod multiply_matrix;
pub mod mute;
pub mod nco;
pub mod nlog10;
pub mod noise;
pub mod noise_blanker;
pub mod null_sink_source;
pub mod numeric_conversions;
pub mod ofdm;
pub mod ofdm_channel_est;
pub mod observe;
pub mod phase_unwrap;
pub mod pfb_channelizer;
pub mod pfb_synthesizer;
pub mod plateau_detector;
pub mod power_squelch;
pub mod preamble_gen;
pub mod packet;
pub mod packet_encoder;
pub mod packet_framing;
pub mod packet_sink;
pub mod papr_reduction;
pub mod pfb_clock_sync;
pub mod probe;
pub mod probe_avg_mag_sqrd;
pub mod probe_density;
pub mod probe_power;
pub mod puncture;
pub mod params;
pub mod pdu;
pub mod patterned_interleaver;
pub mod pdu_filter;
pub mod peak_detector;
pub mod peak_hold;
pub mod peak_to_average;
pub mod phase_modulator;
pub mod phase_ops;
pub mod phase_shift;
pub mod pll;
pub mod pn_sync;
pub mod pocsag_decoder;
pub mod plugin;
pub mod quadrature_demod;
pub mod rail;
pub mod random_source;
pub mod rds_decoder;
pub mod regenerate_bb;
pub mod repeat;
pub mod rms;
pub mod rotator;
pub mod rt;
pub mod simd_utils;
pub mod spreading;
pub mod sample_and_hold;
pub mod sample_ops;
pub mod scrambler;
pub mod scheduler;
pub mod selector;
pub mod rt_scheduler;
pub mod signal_detector;
pub mod signal_source;
pub mod silence_detector;
pub mod single_pole_iir;
pub mod skiphead;
pub mod snr_estimator;
pub mod spectrum_sensor;
pub mod squelch;
pub mod ssb_modem;
pub mod hdlc;
pub mod header_payload_demux;
pub mod hilbert;
pub mod histogram;
pub mod symbol_sync;
pub mod stream_arithmetic;
pub mod stream_control;
pub mod stream_mux;
pub mod stream_tags;
pub mod stream_to_streams;
pub mod stream_to_tagged_stream;
pub mod stream_to_vector;
pub mod stretch;
pub mod symbol_mapping;
pub mod symbol_slicer;
pub mod sync;
pub mod synthesizer;
pub mod tag_debug;
pub mod tag_share;
pub mod tagged_stream_multiply_length;
pub mod tagged_stream_pdu;
pub mod tcp_source_sink;
pub mod threshold;
pub mod throttle;
pub mod time_sync;
pub mod timing;
pub mod timing_error_detector;
pub mod transcendental;
pub mod turbo_code;
pub mod type_conversions;
pub mod types;
pub mod udp_source_sink;
pub mod valve;
pub mod vco;
pub mod vector_insert;
pub mod vector_map;
pub mod vector_sink;
pub mod vocoder;
pub mod wav_source_sink;
pub mod wavelet;
pub mod waveform;
pub mod welch_psd;
pub mod whitening;

// Mesh networking support
pub mod mesh;

// Parallel processing (requires `parallel` feature)
#[cfg(feature = "parallel")]
pub mod parallel;

// FPGA hardware acceleration (requires `fpga` feature)
#[cfg(feature = "fpga")]
pub mod fpga_accel;

// Re-export main types
pub use chirp::{ChirpGenerator, ChirpType};
pub use coding::{GrayCode, HammingCode, Interleaver};
pub use demodulation::Demodulator;
pub use io::IqFormat;
pub use modulation::Modulator;
pub use packet::{LoRaPacket, PacketHeader};
pub use params::{CodingRate, LoRaParams, SpreadingFactor};
pub use sync::{PreambleDetector, Synchronizer};
pub use types::{Complex, IQSample, Sample};
pub use waveform::{CommonParams, DemodResult, Waveform, WaveformFactory, WaveformInfo, VisualizationData};
pub use whitening::Whitening;

// Analysis tools re-exports
pub use analysis::{
    Colormap, IQImbalance, PeakFinder, SignalStats, SpectralPeak,
    SpectrumAnalyzer, SpectrumResult, WaterfallGenerator, WaterfallResult, WindowFunction,
};

// Mesh networking re-exports
pub use mesh::{
    MeshNetwork, MeshPhy, MeshPacket, NodeId, FloodRouter, MacLayer,
    LoRaMesh, LoRaMeshPhy, LoRaMeshConfig, MeshtasticNode, ModemPreset, Region,
};

/// Prelude module for convenient imports
pub mod prelude {
    pub use crate::chirp::{ChirpGenerator, ChirpType};
    pub use crate::demodulation::Demodulator;
    pub use crate::modulation::Modulator;
    pub use crate::params::{CodingRate, LoRaParams, SpreadingFactor};
    pub use crate::types::{Complex, IQSample};
    // Mesh networking
    pub use crate::mesh::{MeshNetwork, MeshPhy, MeshPacket, NodeId, LoRaMesh, LoRaMeshConfig};
}
