# Advanced Waveform Development Workshops

These workshops explore the full depth of waveform development with R4W.

## Workshop Series

### Part 1: DSP Fundamentals
- [10_dsp_basics.rs](exercises/10_dsp_basics.rs) - Complex numbers, sampling, Nyquist
- [11_fft_fundamentals.rs](exercises/11_fft_fundamentals.rs) - FFT, windowing, spectral analysis
- [12_filtering.rs](exercises/12_filtering.rs) - FIR/IIR filters, design, implementation
- [13_resampling.rs](exercises/13_resampling.rs) - Sample rate conversion, interpolation

### Part 2: Modulation Deep Dive
- [20_psk_modem.rs](exercises/20_psk_modem.rs) - Build a complete PSK modem from scratch
- [21_qam_modem.rs](exercises/21_qam_modem.rs) - Build a QAM modem with Gray coding
- [22_ofdm_basics.rs](exercises/22_ofdm_basics.rs) - OFDM symbol generation and reception
- [23_css_lora.rs](exercises/23_css_lora.rs) - Deep dive into LoRa CSS modulation

### Part 3: Synchronization
- [30_symbol_timing.rs](exercises/30_symbol_timing.rs) - Symbol timing recovery
- [31_carrier_sync.rs](exercises/31_carrier_sync.rs) - Carrier frequency/phase recovery
- [32_frame_sync.rs](exercises/32_frame_sync.rs) - Frame synchronization and detection
- [33_clock_recovery.rs](exercises/33_clock_recovery.rs) - Clock recovery loops

### Part 4: Channel Effects
- [40_awgn_channel.rs](exercises/40_awgn_channel.rs) - AWGN channel simulation
- [41_multipath.rs](exercises/41_multipath.rs) - Multipath and fading channels
- [42_doppler.rs](exercises/42_doppler.rs) - Doppler effects and compensation
- [43_channel_estimation.rs](exercises/43_channel_estimation.rs) - Channel estimation techniques

### Part 5: Error Control
- [50_crc_basics.rs](exercises/50_crc_basics.rs) - CRC generation and checking
- [51_hamming_codes.rs](exercises/51_hamming_codes.rs) - Hamming codes for FEC
- [52_convolutional.rs](exercises/52_convolutional.rs) - Convolutional codes, Viterbi
- [53_interleaving.rs](exercises/53_interleaving.rs) - Interleaving strategies

### Part 6: Protocol Implementation
- [60_packet_framing.rs](exercises/60_packet_framing.rs) - Packet structure design
- [61_mac_protocol.rs](exercises/61_mac_protocol.rs) - MAC layer basics
- [62_aloha.rs](exercises/62_aloha.rs) - ALOHA protocol implementation
- [63_csma.rs](exercises/63_csma.rs) - CSMA/CA implementation

### Part 7: Performance Analysis
- [70_ber_testing.rs](exercises/70_ber_testing.rs) - Bit error rate measurement
- [71_sensitivity.rs](exercises/71_sensitivity.rs) - Receiver sensitivity testing
- [72_ber_curves.rs](exercises/72_ber_curves.rs) - BER vs SNR curves
- [73_spectrum_analysis.rs](exercises/73_spectrum_analysis.rs) - Spectrum and EVM analysis

### Part 8: Custom Waveform Development
- [80_waveform_trait.rs](exercises/80_waveform_trait.rs) - Implementing the Waveform trait
- [81_modulator_design.rs](exercises/81_modulator_design.rs) - Custom modulator design
- [82_demodulator_design.rs](exercises/82_demodulator_design.rs) - Custom demodulator design
- [83_full_waveform.rs](exercises/83_full_waveform.rs) - Complete waveform from scratch

## Prerequisites

- Completed USRP workshop (exercises 01-09)
- Understanding of basic signal processing concepts
- Familiarity with Rust programming

## Learning Objectives

After completing these workshops, you will be able to:

1. **DSP Mastery**: Implement core DSP algorithms from scratch
2. **Modulation Understanding**: Build any standard modulation scheme
3. **Synchronization**: Implement timing, carrier, and frame sync
4. **Channel Awareness**: Model and compensate for real-world channels
5. **Error Control**: Design and implement FEC schemes
6. **Protocol Design**: Create complete communication protocols
7. **Performance Analysis**: Measure and optimize waveform performance
8. **Custom Waveforms**: Design and implement novel waveforms

## Running the Workshops

```bash
# Run with simulator
cargo run --example <exercise_name> -- --simulator

# Run with USRP hardware
cargo run --example <exercise_name> -- --device "uhd://type=b200"
```
