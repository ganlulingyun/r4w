//! Export R4W Galileo E1 code for verification
//!
//! Run with: cargo run --example export_galileo_code -p r4w-core

use r4w_core::spreading::PnSequence;
use r4w_core::waveform::gnss::prn::GalileoE1CodeGenerator;
use std::fs::File;
use std::io::Write;

fn main() {
    println!("Exporting Galileo E1 codes from R4W...\n");

    for prn in 1..=5 {
        let mut gen = GalileoE1CodeGenerator::new(prn);
        let code: Vec<i8> = (0..4092).map(|_| gen.next_chip()).collect();

        // Print first 50 chips
        let first_50: Vec<i8> = code[..50].to_vec();
        println!("PRN {}: first 50 chips = {:?}", prn, first_50);

        // Count balance
        let ones: usize = code.iter().filter(|&&c| c == 1).count();
        println!("       balance = {}/4092 ones\n", ones);

        // Export to file for Python comparison
        let filename = format!("galileo_e1_prn{}_r4w.txt", prn);
        let mut f = File::create(&filename).expect("create file");
        for chip in &code {
            writeln!(f, "{}", chip).expect("write chip");
        }
        println!("       Exported to {}", filename);
    }
}
