//! # Exercise 50: CRC Fundamentals
//!
//! Understand CRC generation and checking for error detection.
//!
//! ## Topics
//! - CRC algorithm explained
//! - Common CRC polynomials
//! - Implementation techniques
//! - Error detection capability
//!
//! ## Run
//! ```bash
//! cargo run --example 50_crc_basics
//! ```
//!
//! trace:FR-0095 | ai:claude

fn main() {
    println!("=== CRC Fundamentals Workshop ===\n");

    // Part 1: Basic CRC concept
    println!("== Part 1: CRC Concept ==\n");
    explain_crc_concept();

    // Part 2: CRC-8 implementation
    println!("\n== Part 2: CRC-8 Implementation ==\n");
    demonstrate_crc8();

    // Part 3: CRC-16 implementation
    println!("\n== Part 3: CRC-16 Implementation ==\n");
    demonstrate_crc16();

    // Part 4: Error detection
    println!("\n== Part 4: Error Detection Capability ==\n");
    demonstrate_error_detection();

    // Part 5: Common polynomials
    println!("\n== Part 5: Common CRC Polynomials ==\n");
    list_common_polynomials();

    println!("\n=== Workshop Complete ===");
}

fn explain_crc_concept() {
    println!("CRC = Cyclic Redundancy Check");
    println!();
    println!("How it works:");
    println!("1. Treat data as a large binary number");
    println!("2. Divide by a 'generator polynomial'");
    println!("3. Append the remainder to the data");
    println!("4. Receiver divides data+CRC by same polynomial");
    println!("5. If remainder = 0, no errors detected");
    println!();
    println!("Example with CRC-4 (polynomial 0x13 = x⁴+x+1):");
    println!();

    let data: u8 = 0b1101_0011;
    println!("Data: {:08b}", data);

    // Simplified CRC-4 calculation
    let poly: u8 = 0x13;  // x^4 + x + 1 = 10011
    println!("Polynomial: {:05b} (x⁴+x+1)", poly);

    // Shift data left by 4 bits and divide
    let shifted = (data as u16) << 4;
    println!("Shifted: {:012b}", shifted);

    let crc = crc4(data, poly);
    println!("CRC: {:04b}", crc);

    let transmitted = ((data as u16) << 4) | (crc as u16);
    println!("Transmitted: {:012b}", transmitted);
}

fn crc4(data: u8, poly: u8) -> u8 {
    let mut remainder = data as u16;
    let poly16 = poly as u16;

    // Process each bit
    for i in (4..12).rev() {
        if remainder & (1 << i) != 0 {
            remainder ^= poly16 << (i - 4);
        }
    }

    (remainder & 0x0F) as u8
}

fn demonstrate_crc8() {
    println!("CRC-8-CCITT: polynomial 0x07 (x⁸+x²+x+1)");
    println!();

    let test_data = [
        &b"Hello"[..],
        &b"hello"[..],
        &b"Hell0"[..],
        &b""[..],
        &b"The quick brown fox"[..],
    ];

    println!("{:25} {:>10}", "Data", "CRC-8");
    println!("{}", "-".repeat(37));

    for data in test_data {
        let crc = crc8_ccitt(data);
        let display = if data.is_empty() {
            "(empty)".to_string()
        } else {
            String::from_utf8_lossy(data).to_string()
        };
        println!("{:25} 0x{:02X}", display, crc);
    }

    // Verify CRC
    println!("\nVerification:");
    let data = b"Hello";
    let crc = crc8_ccitt(data);
    let mut with_crc = data.to_vec();
    with_crc.push(crc);

    let verify_crc = crc8_ccitt(&with_crc);
    println!("  Original data CRC: 0x{:02X}", crc);
    println!("  After appending CRC: 0x{:02X} (should be 0x00)", verify_crc);
}

fn crc8_ccitt(data: &[u8]) -> u8 {
    let poly: u8 = 0x07;  // CRC-8-CCITT
    let mut crc: u8 = 0x00;

    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    crc
}

fn demonstrate_crc16() {
    println!("CRC-16-CCITT: polynomial 0x1021 (x¹⁶+x¹²+x⁵+1)");
    println!();

    let test_data = [
        &b"123456789"[..],  // Standard test vector
        &b"Hello"[..],
        &b"hello"[..],
    ];

    println!("{:25} {:>10}", "Data", "CRC-16");
    println!("{}", "-".repeat(37));

    for data in test_data {
        let crc = crc16_ccitt(data);
        println!("{:25} 0x{:04X}", String::from_utf8_lossy(data), crc);
    }

    // Note about "123456789" standard
    println!("\nNote: CRC of \"123456789\" is the standard test vector.");
    println!("Expected CRC-16-CCITT: 0x29B1 (check your implementation)");
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    let poly: u16 = 0x1021;  // CRC-16-CCITT
    let mut crc: u16 = 0xFFFF;  // Common init value

    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    crc
}

fn demonstrate_error_detection() {
    let original = b"Test message";
    let crc = crc16_ccitt(original);

    println!("Original: \"{}\"", String::from_utf8_lossy(original));
    println!("CRC-16: 0x{:04X}", crc);
    println!();

    // Single bit errors
    println!("Single bit errors (all detected):");
    let mut errors_detected = 0;
    let mut errors_tested = 0;

    for byte_idx in 0..original.len() {
        for bit_idx in 0..8 {
            let mut corrupted = original.to_vec();
            corrupted[byte_idx] ^= 1 << bit_idx;
            let new_crc = crc16_ccitt(&corrupted);

            errors_tested += 1;
            if new_crc != crc {
                errors_detected += 1;
            }
        }
    }

    println!("  Tested: {} single-bit errors", errors_tested);
    println!("  Detected: {} ({}%)", errors_detected, 100 * errors_detected / errors_tested);

    // Burst errors
    println!("\nBurst error detection (up to 16 bits):");
    println!("  CRC-16 guarantees detection of all burst errors ≤ 16 bits");

    // Double bit errors
    println!("\nDouble bit errors (testing sample):");
    let mut double_detected = 0;
    let mut double_tested = 0;

    for i in 0..original.len() * 8 {
        for j in (i + 1)..original.len() * 8 {
            if double_tested >= 100 {
                break;
            }

            let mut corrupted = original.to_vec();
            corrupted[i / 8] ^= 1 << (i % 8);
            corrupted[j / 8] ^= 1 << (j % 8);
            let new_crc = crc16_ccitt(&corrupted);

            double_tested += 1;
            if new_crc != crc {
                double_detected += 1;
            }
        }
    }

    println!("  Tested: {} double-bit errors", double_tested);
    println!("  Detected: {} ({}%)", double_detected, 100 * double_detected / double_tested);
}

fn list_common_polynomials() {
    println!("{:20} {:12} {:20}", "Name", "Polynomial", "Used In");
    println!("{}", "-".repeat(55));
    println!("{:20} {:12} {:20}", "CRC-8-CCITT", "0x07", "ATM, ISDN");
    println!("{:20} {:12} {:20}", "CRC-8-Dallas", "0x31", "1-Wire");
    println!("{:20} {:12} {:20}", "CRC-16-CCITT", "0x1021", "X.25, HDLC");
    println!("{:20} {:12} {:20}", "CRC-16-IBM", "0x8005", "USB, Modbus");
    println!("{:20} {:12} {:20}", "CRC-32", "0x04C11DB7", "Ethernet, ZIP");
    println!("{:20} {:12} {:20}", "CRC-32C", "0x1EDC6F41", "iSCSI, SCTP");
    println!();
    println!("Selection factors:");
    println!("  - Hamming distance (number of bit errors guaranteed to detect)");
    println!("  - Data length being protected");
    println!("  - Hardware implementation efficiency");
}
