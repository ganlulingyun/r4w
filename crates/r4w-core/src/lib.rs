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

pub mod adaptive_filter_rls;
pub mod adaptive_interference_canceller;
pub mod adaptive_modcod;
pub mod adaptive_notch;
pub mod adaptive_nulling_beamformer;
pub mod adpcm_codec;
pub mod aes_stream_cipher;
pub mod afc;
pub mod agent;
pub mod agc;
pub mod agc_attack_decay;
pub mod alamouti_codec;
pub mod ais_decoder;
pub mod ais_encoder;
pub mod am_demod;
pub mod ambient_backscatter_processor;
pub mod ambiguity_function;
pub mod antenna_array_response;
pub mod antenna_design_optimizer;
pub mod aprs_decoder;
pub mod arbitrary_resampler;
pub mod argmax;
pub mod arq_engine;
pub mod arithmetic;
pub mod carrier_aggregation_scheduler;
pub mod carrier_recovery;
pub mod ccsds_frame_processor;
pub mod char_to_float;
pub mod channel_capacity;
pub mod channel_estimator;
pub mod channel_model;
pub mod cepstral_analysis;
pub mod channel_sounder;
pub mod channel_sounding_processor;
pub mod clock_recovery;
pub mod clock_recovery_mm;
pub mod cma_equalizer;
pub mod comb_filter;
pub mod coordinates;
pub mod correlator;
pub mod crc;
pub mod crest_factor_reduction;
pub mod csma_ca_mac;
pub mod dc_blocker;
pub mod decision_feedback_equalizer;
pub mod decimating_fir;
pub mod diff_phasor;
pub mod differential;
pub mod digital_down_converter;
pub mod digital_predistortion;
pub mod digital_up_converter;
pub mod equalizer;
pub mod add_blk;
pub mod additive_scrambler;
pub mod analysis;
pub mod anti_jam;
pub mod barker_code;
pub mod beam_steering_controller;
pub mod bch_code;
pub mod beamformer;
pub mod benchmark;
pub mod block_gateway;
pub mod ber_tool;
pub mod binary_slicer;
pub mod bin_statistics;
pub mod biomedical_signal_processor;
pub mod bit_packing;
pub mod bitwise_ops;
pub mod blind_source_separation;
pub mod blind_timing_recovery;
pub mod burst_detector;
pub mod burst_gating_controller;
pub mod burst_shaper;
pub mod burst_synchronizer;
pub mod burst_tagger;
pub mod cfar;
pub mod cfar_2d;
pub mod cfo_corrector;
pub mod cfo_estimator;
pub mod check_lfsr;
pub mod cic_filter;
pub mod chirp;
pub mod chirp_compressor;
pub mod chirp_z_transform;
pub mod chunks_to_symbols;
pub mod clutter_filter;
pub mod coding;
pub mod coherent_integrator;
pub mod cognitive_engine;
pub mod cognitive_radio_spectrum_learner;
pub mod config;
pub mod control_loop;
pub mod complex_normalize;
pub mod complex_to_interleaved;
pub mod complex_to_arg;
pub mod complex_to_mag_phase;
pub mod compressive_sensing;
pub mod conjugate;
pub mod convolutional_encoder;
pub mod convolutional_interleaver;
pub mod cordic;
pub mod constellation_receiver;
pub mod constellation_rotation_detector;
pub mod constellation_demapper;
pub mod constellation_encoder;
pub mod constellation_soft_decoder;
pub mod constellation_tracer;
pub mod costas_loop;
pub mod covert_timing_encoder;
pub mod cpm;
pub mod ctcss_squelch;
pub mod cyclic_autocorrelation;
pub mod cyclic_prefix;
pub mod cyclic_redundancy_check_parallel;
pub mod cyclostationary_detector;
pub mod dab_plus;
pub mod dac_model;
pub mod delay;
pub mod delay_lock_loop;
pub mod demodulation;
pub mod depuncture;
pub mod doppler_pre_correction;
pub mod dpll;
pub mod dtmf;
pub mod dynamic_channel;
pub mod dynamic_spectrum_manager;
pub mod dynamic_range_compressor;
pub mod dvb_s2_deframer;
pub mod early_late_gate;
pub mod emitter_localization;
pub mod empirical_mode;
pub mod endian_swap;
pub mod energy_detector;
pub mod envelope_detector;
pub mod esprit;
pub mod esm_receiver;
pub mod evm_calculator;
pub mod exponentiate;
pub mod eye_diagram;
pub mod farrow_resampler;
pub mod fbmc_polyphase_mapper;
pub mod feedforward_agc;
pub mod feedforward_timing_estimator;
pub mod file_meta;
pub mod file_descriptor_source_sink;
pub mod file_source_sink;
pub mod acars_decoder;
pub mod abs_blk;
pub mod access_code_detector;
pub mod ax25;
pub mod correlate_estimate;
pub mod correlate_sync;
pub mod cross_ambiguity_function;
pub mod cross_correlator;
pub mod fec;
pub mod fec_generic_api;
pub mod fft_filter;
pub mod fft_utils;
pub mod filters;
pub mod filter_synthesis_engine;
pub mod fmcw_automotive_processor;
pub mod fmcw_radar;
pub mod fll_band_edge;
pub mod float_to_complex;
pub mod fm_emphasis;
pub mod fm_stereo_decoder;
pub mod fountain_code;
pub mod frequency_domain_channel_sounder;
pub mod frequency_domain_equalizer;
pub mod frequency_hopper;
pub mod frequency_hopping;
pub mod frequency_modulator;
pub mod frequency_shift;
pub mod fm_receiver;
pub mod fractional_delay;
pub mod frame_sync;
pub mod freq_lock_detector;
pub mod freq_xlating_fir;
pub mod frequency_xlating_fft_filter;
pub mod full_duplex_self_interference_canceller;
pub mod gardner_ted;
pub mod glfsr_source;
pub mod gmsk_modulator;
pub mod gold_code_generator;
pub mod goertzel;
pub mod golay_code;
pub mod gps_time;
pub mod group_delay_equalizer;
pub mod harq_manager;
pub mod head;
pub mod hier_block;
pub mod hw_impairments;
pub mod index_modulation_mapper;
pub mod integrate;
pub mod integrate_and_dump;
pub mod interleave;
pub mod interleaved;
pub mod interference_classifier;
pub mod interference_excision;
pub mod interp_fir;
pub mod interpolating_resampler;
pub mod io;
pub mod iq_balance;
pub mod iq_imbalance_corrector;
pub mod iq_imbalance_estimator;
pub mod irig_b_decoder;
pub mod jitter_analyzer;
pub mod kalman_filter;
pub mod keep_m_in_n;
pub mod lattice_filter;
pub mod ldpc_codec;
pub mod linear_equalizer;
pub mod link_adaptation_engine;
pub mod link_budget;
pub mod link_budget_optimizer;
pub mod lms_filter;
pub mod log_blk;
pub mod log_likelihood_ratio;
pub mod log_power_fft;
pub mod lpi_metrics;
pub mod magnitude_squared;
pub mod map_bb;
pub mod map_decoder;
pub mod matched_filter_bank;
pub mod matrix_eigenvalue;
pub mod max_blk;
pub mod median_filter;
pub mod melp_vocoder;
pub mod message_port;
pub mod message_strobe;
pub mod mimo;
pub mod mimo_detector;
pub mod mimo_precoder;
pub mod ml_sequence_detector;
pub mod mmse_equalizer;
pub mod mmse_interpolator;
pub mod modulation;
pub mod modulation_classifier;
pub mod modulation_recognition_classifier;
pub mod moving_autocorrelation;
pub mod moving_average;
pub mod moving_avg_decim;
pub mod moving_minmax;
pub mod moving_rms;
pub mod moving_variance;
pub mod msk_modulator;
pub mod mti_filter;
pub mod mu_law_codec;
pub mod mueller_muller_ted;
pub mod multi_rate_clock;
pub mod multiband_compressor;
pub mod multicarrier_allocation;
pub mod multiply;
pub mod multiply_matrix;
pub mod music_doa;
pub mod mute;
pub mod mvdr_beamformer;
pub mod network_analyzer;
pub mod nco;
pub mod nlog10;
pub mod noaa_weather_decoder;
pub mod noise;
pub mod noise_blanker;
pub mod noise_figure;
pub mod noise_gate;
pub mod noise_reduction;
pub mod noise_shaper;
pub mod noma_decoder;
pub mod nr_resource_grid_mapper;
pub mod null_sink_source;
pub mod numeric_conversions;
pub mod oam_beam_generator;
pub mod ofdm;
pub mod ofdm_carrier_allocator;
pub mod ofdm_channel_est;
pub mod ofdm_frame_equalizer;
pub mod ofdm_resource_mapper;
pub mod ofdm_sync_schmidl_cox;
pub mod oqpsk_modulator;
pub mod orthogonal_space_time_block_code;
pub mod observe;
pub mod overlap_add;
pub mod overlap_save;
pub mod phase_unwrap;
pub mod pfb_channelizer;
pub mod pfb_synthesizer;
pub mod plateau_detector;
pub mod power_squelch;
pub mod pre_emphasis;
pub mod preamble_gen;
pub mod packet;
pub mod packet_decoder;
pub mod packet_encoder;
pub mod packet_framing;
pub mod packet_header_parser;
pub mod packet_sink;
pub mod papr_reduction;
pub mod pfb_clock_sync;
pub mod probe;
pub mod probe_avg_mag_sqrd;
pub mod prony_method;
pub mod protocol_anomaly_detector;
pub mod protocol_frame_parser;
pub mod protocol_formatter;
pub mod probe_density;
pub mod probe_power;
pub mod probe_rate;
pub mod pulse_compressor;
pub mod puncture;
pub mod params;
pub mod particle_filter_tracker;
pub mod pdu;
pub mod patterned_interleaver;
pub mod pdu_filter;
pub mod pdu_router;
pub mod pdu_set;
pub mod pdu_to_tagged_stream;
pub mod peak_detector;
pub mod peak_hold;
pub mod peak_to_average;
pub mod periodogram_psd;
pub mod permute;
pub mod pfb_arb_resampler;
pub mod phase_coherence_analyzer;
pub mod phase_modulator;
pub mod phase_noise_model;
pub mod phase_ops;
pub mod phase_shift;
pub mod phase_vocoder;
pub mod phasor_measurement_unit;
pub mod pid_controller;
pub mod pilot_inserter;
pub mod pll;
pub mod pll_carrier_tracking;
pub mod pn_scrambler;
pub mod pn_sync;
pub mod pocsag_decoder;
pub mod polar_code;
pub mod power_amplifier_dpd;
pub mod power_amplifier_model;
pub mod power_control;
pub mod power_meter;
pub mod plugin;
pub mod quadrature_demod;
pub mod radar_cross_section_estimator;
pub mod radar_display;
pub mod radar_waveform_classifier;
pub mod rail;
pub mod rake_receiver;
pub mod random_pdu_gen;
pub mod random_source;
pub mod range_doppler_detector;
pub mod range_doppler_map;
pub mod raptor_code;
pub mod rate_matcher;
pub mod rds_decoder;
pub mod reed_solomon;
pub mod regenerate_bb;
pub mod repeat;
pub mod repetition_code;
pub mod rf_mixer;
pub mod rf_fingerprinting_engine;
pub mod rf_power_monitor;
pub mod rf_propagation_model;
pub mod ris_phase_controller;
pub mod rms;
pub mod rotator;
pub mod root_raised_cosine_matched_filter_bank;
pub mod rt;
pub mod simd_utils;
pub mod spreading;
pub mod sample_and_hold;
pub mod sample_counter;
pub mod sample_ops;
pub mod sample_rate_converter;
pub mod sar_processor;
pub mod seismic_processor;
pub mod satellite_link_predictor;
pub mod savitzky_golay;
pub mod scrambler;
pub mod sc_fdma;
pub mod scheduler;
pub mod selector;
pub mod rt_scheduler;
pub mod sigma_delta;
pub mod sigma_delta_modulator;
pub mod signal_clipper;
pub mod signal_detector;
pub mod signal_generator;
pub mod signal_quality_metrics;
pub mod signal_recorder_indexed;
pub mod signal_source;
pub mod sigfox_decoder;
pub mod silence_detector;
pub mod single_pole_iir;
pub mod skiphead;
pub mod snr_estimator;
pub mod sonar_processor;
pub mod soft_decision_decoder;
pub mod socket_pdu;
pub mod sparse_fir_filter;
pub mod spatio_temporal_fusion;
pub mod spectral_mask;
pub mod spectral_mask_painter;
pub mod spectral_correlation_analyzer;
pub mod spectral_kurtosis_detector;
pub mod spectral_occupancy_monitor;
pub mod spectral_subtraction_denoiser;
pub mod spectrum_coexistence_analyzer;
pub mod spectrum_hole_detector;
pub mod spectrum_sensor;
pub mod spurious_emission_scanner;
pub mod spurs_mitigation;
pub mod squelch;
pub mod ssb_modem;
pub mod stft;
pub mod subspace_tracker;
pub mod hdlc;
pub mod header_payload_demux;
pub mod hilbert;
pub mod histogram;
pub mod symbol_sync;
pub mod stream_arithmetic;
pub mod stream_byte_converter;
pub mod stream_control;
pub mod stream_demux;
pub mod stream_mux;
pub mod stream_switch;
pub mod stream_tags;
pub mod stream_to_streams;
pub mod stream_to_tagged_stream;
pub mod stream_to_vector;
pub mod tagged_stream_align;
pub mod stretch;
pub mod symbol_demapper;
pub mod symbol_mapping;
pub mod symbol_slicer;
pub mod sync;
pub mod sync_word_detector;
pub mod synthesizer;
pub mod tag_debug;
pub mod tag_share;
pub mod tagged_file_sink;
pub mod tapped_delay_line;
pub mod tagged_stream_multiply_length;
pub mod tagged_stream_mux;
pub mod tagged_stream_to_pdu;
pub mod tagged_stream_pdu;
pub mod tcp_source_sink;
pub mod tdoa_estimator;
pub mod teager_kaiser_energy;
pub mod threshold;
pub mod throttle;
pub mod throttle_blk;
pub mod time_domain_equalizer;
pub mod time_frequency_reassignment;
pub mod time_raster;
pub mod time_sync;
pub mod timing;
pub mod timing_advance_estimator;
pub mod timing_error_detector;
pub mod timing_phase_detector_hybrid;
pub mod transcendental;
pub mod transmission_line_simulator;
pub mod trellis_coding;
pub mod trellis_metrics;
pub mod tuning_estimator;
pub mod turbo_code;
pub mod turbo_equalizer;
pub mod type_conversions;
pub mod types;
pub mod udp_source_sink;
pub mod unpacked_to_packed;
pub mod unscented_kalman_filter;
pub mod valve;
pub mod variable_rate_cic;
pub mod vco;
pub mod viterbi_decoder;
pub mod viterbi_sova;
pub mod vlc_modulator;
pub mod vector_insert;
pub mod vector_map;
pub mod vector_sink;
pub mod vector_quantizer;
pub mod vector_signal_analyzer;
pub mod vector_to_stream;
pub mod vocoder;
pub mod voice_activity_detector;
pub mod wav_source_sink;
pub mod wavelet;
pub mod waveform;
pub mod waveform_diversity_scheduler;
pub mod welch_psd;
pub mod wiener_filter;
pub mod whitening;
pub mod wigner_ville_distribution;
pub mod wola_channelizer;
pub mod zadoff_chu_generator;
pub mod waterfilling;
pub mod zero_crossing_detector;

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
