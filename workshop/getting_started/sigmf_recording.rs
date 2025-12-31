//! # SigMF Recording Example
//!
//! Demonstrates recording and playing back signals in SigMF format.
//!
//! Run with: cargo run --example sigmf_recording

use r4w_core::waveform::{psk, CommonParams, Waveform};
use r4w_sim::hal::sigmf::{SigMfReader, SigMfWriter};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== SigMF Recording Example ===\n");

    let sample_rate = 48000.0;
    let center_freq = 915_000_000.0; // 915 MHz

    // Create a test signal
    let params = CommonParams {
        sample_rate,
        carrier_freq: 0.0,
        amplitude: 1.0,
    };
    let qpsk = psk::PSK::new_qpsk(params, 4800.0);

    let message = b"SigMF test recording";
    let samples = qpsk.modulate(message);

    println!(
        "Generated {} samples from message: {:?}",
        samples.len(),
        String::from_utf8_lossy(message)
    );

    // Write to file
    let recording_path = "/tmp/r4w_test_recording";
    println!("\nWriting to: {}.sigmf-*", recording_path);

    let mut writer = SigMfWriter::create(recording_path, sample_rate, center_freq)?;
    writer.set_description("R4W QPSK test signal");
    writer.set_waveform("QPSK");
    writer.add_annotation(0, samples.len() as u64, "QPSK modulated message");
    writer.write_samples(&samples)?;
    writer.close()?;

    println!("Wrote {} samples", samples.len());

    // Read back
    println!("\nReading back recording...");

    let mut reader = SigMfReader::open(recording_path)?;

    println!("Sample rate: {} Hz", reader.sample_rate());
    println!("Center freq: {} Hz", reader.frequency());
    println!("Total samples: {}", reader.total_samples());

    // Read all samples
    let read_samples = reader.read_all()?;
    println!("Read {} samples", read_samples.len());

    // Verify samples match
    let samples_match = samples
        .iter()
        .zip(read_samples.iter())
        .all(|(a, b)| (a.re - b.re).abs() < 1e-6 && (a.im - b.im).abs() < 1e-6);

    if samples_match {
        println!("\nSUCCESS: Samples match original!");
    } else {
        println!("\nWARNING: Sample mismatch detected");
    }

    // Demodulate the read-back samples
    let result = qpsk.demodulate(&read_samples);
    if result.bits.starts_with(message) {
        println!(
            "Message recovered: {:?}",
            String::from_utf8_lossy(&result.bits[..message.len()])
        );
    }

    println!("\n=== SigMF Format ===");
    println!("- Standard format for signal recordings");
    println!("- .sigmf-meta: JSON metadata file");
    println!("- .sigmf-data: Binary I/Q samples");
    println!("- Supports annotations, capture segments");
    println!("- Widely used in SDR community");

    Ok(())
}
