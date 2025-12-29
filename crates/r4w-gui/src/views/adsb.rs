//! ADS-B Message Display View
//!
//! Interactive view for displaying decoded ADS-B messages with
//! field breakdowns and visualizations.

use egui::{Color32, RichText, Ui};
use r4w_core::waveform::adsb::{
    AdsbMessage, AircraftCategory, MessageContent,
};

/// Predefined ADS-B test messages for demonstration
pub struct TestMessage {
    pub name: &'static str,
    pub description: &'static str,
    #[allow(dead_code)]
    pub hex: &'static str,
    pub bytes: [u8; 14],
}

/// Collection of real-world ADS-B test vectors
pub static TEST_MESSAGES: &[TestMessage] = &[
    TestMessage {
        name: "Aircraft ID (KLM1023)",
        description: "Aircraft identification message with callsign",
        hex: "8D4840D6202CC371C32CE0576098",
        bytes: [0x8D, 0x48, 0x40, 0xD6, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0, 0x57, 0x60, 0x98],
    },
    TestMessage {
        name: "Airborne Position",
        description: "Position with barometric altitude",
        hex: "8D40621D58C382D690C8AC2863A7",
        bytes: [0x8D, 0x40, 0x62, 0x1D, 0x58, 0xC3, 0x82, 0xD6, 0x90, 0xC8, 0xAC, 0x28, 0x63, 0xA7],
    },
    TestMessage {
        name: "Airborne Velocity",
        description: "Ground speed and heading",
        hex: "8D485020994409940838175B284F",
        bytes: [0x8D, 0x48, 0x50, 0x20, 0x99, 0x44, 0x09, 0x94, 0x08, 0x38, 0x17, 0x5B, 0x28, 0x4F],
    },
];

/// ADS-B message display view
pub struct AdsbView {
    /// Currently selected test message index
    selected_message_idx: usize,
    /// Custom hex input
    custom_hex: String,
    /// Use custom hex instead of test message
    use_custom: bool,
    /// Decoded message (if valid)
    decoded_message: Option<AdsbMessage>,
    /// Parse error message
    error_message: Option<String>,
    /// Show raw bits breakdown
    show_bits: bool,
    /// Show signal visualization
    show_signal: bool,
}

impl Default for AdsbView {
    fn default() -> Self {
        Self::new()
    }
}

impl AdsbView {
    pub fn new() -> Self {
        let mut view = Self {
            selected_message_idx: 0,
            custom_hex: String::new(),
            use_custom: false,
            decoded_message: None,
            error_message: None,
            show_bits: true,
            show_signal: false,
        };
        view.decode_selected();
        view
    }

    /// Parse hex string to bytes
    fn parse_hex(hex: &str) -> Result<[u8; 14], String> {
        let clean: String = hex.chars().filter(|c| c.is_ascii_hexdigit()).collect();
        if clean.len() != 28 {
            return Err(format!("Expected 28 hex characters (14 bytes), got {}", clean.len()));
        }

        let mut bytes = [0u8; 14];
        for (i, chunk) in clean.as_bytes().chunks(2).enumerate() {
            let s = std::str::from_utf8(chunk).unwrap();
            bytes[i] = u8::from_str_radix(s, 16)
                .map_err(|_| format!("Invalid hex at position {}", i * 2))?;
        }
        Ok(bytes)
    }

    /// Decode the currently selected message
    fn decode_selected(&mut self) {
        let bytes = if self.use_custom {
            match Self::parse_hex(&self.custom_hex) {
                Ok(b) => b,
                Err(e) => {
                    self.error_message = Some(e);
                    self.decoded_message = None;
                    return;
                }
            }
        } else {
            TEST_MESSAGES[self.selected_message_idx].bytes
        };

        self.error_message = None;
        self.decoded_message = Some(AdsbMessage::decode(&bytes));
    }

    /// Render the view
    pub fn render(&mut self, ui: &mut Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            self.render_header(ui);
            ui.add_space(12.0);

            self.render_message_selector(ui);
            ui.add_space(12.0);

            if let Some(ref msg) = self.decoded_message {
                self.render_message_summary(ui, msg);
                ui.add_space(12.0);

                self.render_message_content(ui, msg);
                ui.add_space(12.0);

                if self.show_bits {
                    self.render_bit_breakdown(ui, msg);
                }
            } else if let Some(ref err) = self.error_message {
                ui.colored_label(Color32::RED, format!("Error: {}", err));
            }
        });
    }

    fn render_header(&self, ui: &mut Ui) {
        ui.heading("ADS-B Message Decoder");
        ui.label(
            RichText::new(
                "Decode and analyze Mode S Extended Squitter (DF17) messages. \
                 Select a test message or enter custom hex data."
            ).weak()
        );
    }

    fn render_message_selector(&mut self, ui: &mut Ui) {
        egui::Frame::none()
            .fill(Color32::from_rgb(35, 40, 48))
            .inner_margin(12.0)
            .rounding(6.0)
            .show(ui, |ui| {
                ui.label(RichText::new("Message Input").strong());
                ui.add_space(8.0);

                // Test message dropdown
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.use_custom, false, "Test Message:");
                    let mut changed = false;
                    egui::ComboBox::from_id_salt("test_message")
                        .selected_text(TEST_MESSAGES[self.selected_message_idx].name)
                        .show_ui(ui, |ui| {
                            for (i, msg) in TEST_MESSAGES.iter().enumerate() {
                                if ui.selectable_value(&mut self.selected_message_idx, i, msg.name).changed() {
                                    changed = true;
                                }
                            }
                        });
                    if changed && !self.use_custom {
                        self.decode_selected();
                    }
                });

                if !self.use_custom {
                    ui.label(
                        RichText::new(TEST_MESSAGES[self.selected_message_idx].description)
                            .weak()
                            .italics()
                    );
                }

                ui.add_space(8.0);

                // Custom hex input
                ui.horizontal(|ui| {
                    ui.radio_value(&mut self.use_custom, true, "Custom Hex:");
                    let response = ui.add(
                        egui::TextEdit::singleline(&mut self.custom_hex)
                            .desired_width(300.0)
                            .hint_text("28 hex chars (e.g., 8D4840D6202CC371C32CE0576098)")
                            .font(egui::TextStyle::Monospace)
                    );
                    if response.changed() && self.use_custom {
                        self.decode_selected();
                    }
                });

                ui.add_space(8.0);

                // Display options
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.show_bits, "Show Bit Breakdown");
                    ui.checkbox(&mut self.show_signal, "Show Signal");
                });
            });
    }

    fn render_message_summary(&self, ui: &mut Ui, msg: &AdsbMessage) {
        egui::Frame::none()
            .fill(if msg.crc_valid {
                Color32::from_rgb(30, 50, 35)
            } else {
                Color32::from_rgb(50, 30, 30)
            })
            .inner_margin(12.0)
            .rounding(6.0)
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    // CRC status
                    let (status_text, status_color) = if msg.crc_valid {
                        ("CRC Valid", Color32::from_rgb(100, 255, 100))
                    } else {
                        ("CRC Invalid", Color32::from_rgb(255, 100, 100))
                    };
                    ui.label(RichText::new(status_text).color(status_color).strong());

                    ui.separator();

                    // ICAO address
                    ui.label("ICAO:");
                    ui.label(
                        RichText::new(msg.icao_hex())
                            .monospace()
                            .color(Color32::from_rgb(255, 200, 100))
                    );

                    ui.separator();

                    // Downlink format
                    ui.label("Format:");
                    ui.label(format!("{:?}", msg.downlink_format));

                    ui.separator();

                    // Type code
                    ui.label("Type:");
                    ui.label(format!("{:?}", msg.type_code));
                });
            });
    }

    fn render_message_content(&self, ui: &mut Ui, msg: &AdsbMessage) {
        egui::Frame::none()
            .fill(Color32::from_rgb(40, 45, 55))
            .inner_margin(12.0)
            .rounding(6.0)
            .show(ui, |ui| {
                ui.label(RichText::new("Decoded Content").strong().size(16.0));
                ui.add_space(8.0);

                match &msg.content {
                    MessageContent::Identification { category, callsign } => {
                        self.render_identification(ui, callsign, category);
                    }
                    MessageContent::AirbornePosition {
                        altitude,
                        cpr_lat,
                        cpr_lon,
                        cpr_odd,
                        surveillance_status,
                        single_antenna,
                        time_flag,
                    } => {
                        self.render_airborne_position(
                            ui,
                            *altitude,
                            *cpr_lat,
                            *cpr_lon,
                            *cpr_odd,
                            *surveillance_status,
                            *single_antenna,
                            *time_flag,
                        );
                    }
                    MessageContent::AirborneVelocity {
                        subtype,
                        heading,
                        ground_speed,
                        vertical_rate,
                        vr_source,
                    } => {
                        self.render_velocity(ui, *subtype, *heading, *ground_speed, *vertical_rate, *vr_source);
                    }
                    MessageContent::SurfacePosition {
                        ground_speed,
                        track,
                        cpr_lat,
                        cpr_lon,
                        cpr_odd,
                    } => {
                        self.render_surface_position(ui, *ground_speed, *track, *cpr_lat, *cpr_lon, *cpr_odd);
                    }
                    MessageContent::AircraftStatus { emergency, squawk } => {
                        self.render_aircraft_status(ui, *emergency, *squawk);
                    }
                    MessageContent::OperationalStatus {
                        version,
                        nic_supplement,
                        nac_p,
                        baro_alt_integrity,
                        sil,
                    } => {
                        self.render_operational_status(ui, *version, *nic_supplement, *nac_p, *baro_alt_integrity, *sil);
                    }
                    MessageContent::Unknown { me_data } => {
                        ui.label("Unknown message type");
                        ui.label(
                            RichText::new(format!(
                                "ME Data: {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                                me_data[0], me_data[1], me_data[2], me_data[3],
                                me_data[4], me_data[5], me_data[6]
                            ))
                            .monospace()
                        );
                    }
                }
            });
    }

    fn render_identification(&self, ui: &mut Ui, callsign: &str, category: &AircraftCategory) {
        egui::Grid::new("ident_grid")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("Callsign:");
                ui.label(
                    RichText::new(callsign)
                        .monospace()
                        .size(20.0)
                        .color(Color32::from_rgb(100, 200, 255))
                );
                ui.end_row();

                ui.label("Category:");
                ui.label(format!("{:?}", category));
                ui.end_row();
            });
    }

    fn render_airborne_position(
        &self,
        ui: &mut Ui,
        altitude: Option<i32>,
        cpr_lat: u32,
        cpr_lon: u32,
        cpr_odd: bool,
        surveillance_status: u8,
        single_antenna: bool,
        time_flag: bool,
    ) {
        egui::Grid::new("pos_grid")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("Altitude:");
                if let Some(alt) = altitude {
                    ui.label(
                        RichText::new(format!("{} ft", alt))
                            .size(18.0)
                            .color(Color32::from_rgb(100, 255, 150))
                    );
                } else {
                    ui.label("Not available");
                }
                ui.end_row();

                ui.label("CPR Frame:");
                ui.label(if cpr_odd { "Odd" } else { "Even" });
                ui.end_row();

                ui.label("CPR Latitude:");
                ui.label(RichText::new(format!("{}", cpr_lat)).monospace());
                ui.end_row();

                ui.label("CPR Longitude:");
                ui.label(RichText::new(format!("{}", cpr_lon)).monospace());
                ui.end_row();

                ui.label("Surveillance:");
                ui.label(format!("Status {}", surveillance_status));
                ui.end_row();

                ui.label("Single Antenna:");
                ui.label(if single_antenna { "Yes" } else { "No" });
                ui.end_row();

                ui.label("Time Flag:");
                ui.label(if time_flag { "Set" } else { "Clear" });
                ui.end_row();
            });

        ui.add_space(8.0);
        ui.label(
            RichText::new(
                "Note: CPR coordinates require two messages (odd + even) to decode position."
            )
            .weak()
            .italics()
        );
    }

    fn render_velocity(
        &self,
        ui: &mut Ui,
        subtype: u8,
        heading: Option<f64>,
        ground_speed: Option<f64>,
        vertical_rate: Option<i32>,
        vr_source: u8,
    ) {
        egui::Grid::new("vel_grid")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("Subtype:");
                ui.label(format!("{} ({})", subtype, if subtype <= 2 { "Ground Speed" } else { "Airspeed" }));
                ui.end_row();

                ui.label("Ground Speed:");
                if let Some(gs) = ground_speed {
                    ui.label(
                        RichText::new(format!("{:.0} kts", gs))
                            .size(18.0)
                            .color(Color32::from_rgb(100, 200, 255))
                    );
                } else {
                    ui.label("Not available");
                }
                ui.end_row();

                ui.label("Heading:");
                if let Some(hdg) = heading {
                    ui.label(
                        RichText::new(format!("{:.1}°", hdg))
                            .size(18.0)
                            .color(Color32::from_rgb(255, 200, 100))
                    );
                } else {
                    ui.label("Not available");
                }
                ui.end_row();

                ui.label("Vertical Rate:");
                if let Some(vr) = vertical_rate {
                    let color = if vr > 0 {
                        Color32::from_rgb(100, 255, 100)
                    } else if vr < 0 {
                        Color32::from_rgb(255, 150, 100)
                    } else {
                        Color32::GRAY
                    };
                    ui.label(RichText::new(format!("{:+} ft/min", vr)).color(color));
                } else {
                    ui.label("Not available");
                }
                ui.end_row();

                ui.label("VR Source:");
                ui.label(if vr_source == 0 { "GNSS" } else { "Barometric" });
                ui.end_row();
            });
    }

    fn render_surface_position(
        &self,
        ui: &mut Ui,
        ground_speed: Option<f64>,
        track: Option<f64>,
        cpr_lat: u32,
        cpr_lon: u32,
        cpr_odd: bool,
    ) {
        egui::Grid::new("surf_grid")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("Ground Speed:");
                if let Some(gs) = ground_speed {
                    ui.label(format!("{:.1} kts", gs));
                } else {
                    ui.label("Not available");
                }
                ui.end_row();

                ui.label("Track:");
                if let Some(t) = track {
                    ui.label(format!("{:.1}°", t));
                } else {
                    ui.label("Not available");
                }
                ui.end_row();

                ui.label("CPR Frame:");
                ui.label(if cpr_odd { "Odd" } else { "Even" });
                ui.end_row();

                ui.label("CPR Latitude:");
                ui.label(RichText::new(format!("{}", cpr_lat)).monospace());
                ui.end_row();

                ui.label("CPR Longitude:");
                ui.label(RichText::new(format!("{}", cpr_lon)).monospace());
                ui.end_row();
            });
    }

    fn render_aircraft_status(&self, ui: &mut Ui, emergency: u8, squawk: u16) {
        egui::Grid::new("status_grid")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("Squawk:");
                ui.label(
                    RichText::new(format!("{:04}", squawk))
                        .monospace()
                        .size(20.0)
                        .color(Color32::from_rgb(255, 200, 100))
                );
                ui.end_row();

                ui.label("Emergency:");
                let (emerg_text, emerg_color) = match emergency {
                    0 => ("None", Color32::from_rgb(100, 255, 100)),
                    1 => ("General Emergency", Color32::from_rgb(255, 100, 100)),
                    2 => ("Lifeguard/Medical", Color32::from_rgb(255, 200, 100)),
                    3 => ("Minimum Fuel", Color32::from_rgb(255, 200, 100)),
                    4 => ("No Communications", Color32::from_rgb(255, 150, 100)),
                    5 => ("Unlawful Interference", Color32::from_rgb(255, 50, 50)),
                    6 => ("Downed Aircraft", Color32::from_rgb(255, 100, 100)),
                    _ => ("Reserved", Color32::GRAY),
                };
                ui.label(RichText::new(emerg_text).color(emerg_color));
                ui.end_row();
            });

        // Special squawk codes
        let special = match squawk {
            7500 => Some("Hijack"),
            7600 => Some("Radio Failure"),
            7700 => Some("Emergency"),
            _ => None,
        };
        if let Some(code_meaning) = special {
            ui.add_space(8.0);
            ui.label(
                RichText::new(format!("Special Code: {}", code_meaning))
                    .color(Color32::from_rgb(255, 100, 100))
                    .strong()
            );
        }
    }

    fn render_operational_status(
        &self,
        ui: &mut Ui,
        version: u8,
        nic_supplement: bool,
        nac_p: u8,
        baro_alt_integrity: bool,
        sil: u8,
    ) {
        egui::Grid::new("op_status_grid")
            .num_columns(2)
            .spacing([20.0, 8.0])
            .show(ui, |ui| {
                ui.label("ADS-B Version:");
                ui.label(format!("{}", version));
                ui.end_row();

                ui.label("NIC Supplement:");
                ui.label(if nic_supplement { "Yes" } else { "No" });
                ui.end_row();

                ui.label("NAC-p:");
                ui.label(format!("{}", nac_p));
                ui.end_row();

                ui.label("Baro Alt Integrity:");
                ui.label(if baro_alt_integrity { "Cross-checked" } else { "Not cross-checked" });
                ui.end_row();

                ui.label("SIL:");
                let sil_text = match sil {
                    0 => "Unknown",
                    1 => "< 10^-3 per hour",
                    2 => "< 10^-5 per hour",
                    3 => "< 10^-7 per hour",
                    _ => "Reserved",
                };
                ui.label(sil_text);
                ui.end_row();
            });
    }

    fn render_bit_breakdown(&self, ui: &mut Ui, msg: &AdsbMessage) {
        egui::CollapsingHeader::new(RichText::new("Bit-Level Breakdown").strong())
            .default_open(true)
            .show(ui, |ui| {
                ui.add_space(4.0);

                // Raw hex
                ui.horizontal(|ui| {
                    ui.label("Raw Hex:");
                    let hex: String = msg.raw.iter().map(|b| format!("{:02X}", b)).collect();
                    ui.label(RichText::new(hex).monospace());
                });

                ui.add_space(8.0);

                // Field breakdown with colors
                egui::Frame::none()
                    .fill(Color32::from_rgb(25, 28, 32))
                    .inner_margin(10.0)
                    .rounding(4.0)
                    .show(ui, |ui| {
                        // DF (5 bits)
                        let df = msg.raw[0] >> 3;
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("DF").color(Color32::from_rgb(255, 150, 150)));
                            ui.label(format!("= {} (5 bits)", df));
                            ui.label(RichText::new(format!("→ {:?}", msg.downlink_format)).weak());
                        });

                        // CA (3 bits)
                        let ca = msg.raw[0] & 0x07;
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("CA").color(Color32::from_rgb(150, 255, 150)));
                            ui.label(format!("= {} (3 bits)", ca));
                            ui.label(RichText::new("→ Capability").weak());
                        });

                        // ICAO (24 bits)
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("ICAO").color(Color32::from_rgb(150, 150, 255)));
                            ui.label(format!("= {} (24 bits)", msg.icao_hex()));
                            ui.label(RichText::new("→ Aircraft Address").weak());
                        });

                        // Type Code (5 bits)
                        let tc = msg.raw[4] >> 3;
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("TC").color(Color32::from_rgb(255, 255, 150)));
                            ui.label(format!("= {} (5 bits)", tc));
                            ui.label(RichText::new(format!("→ {:?}", msg.type_code)).weak());
                        });

                        // ME Data (51 bits remaining)
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("ME").color(Color32::from_rgb(255, 200, 150)));
                            ui.label("= ... (51 bits)");
                            ui.label(RichText::new("→ Message Data").weak());
                        });

                        // PI/CRC (24 bits)
                        let pi = ((msg.raw[11] as u32) << 16)
                            | ((msg.raw[12] as u32) << 8)
                            | (msg.raw[13] as u32);
                        ui.horizontal(|ui| {
                            ui.label(RichText::new("PI").color(Color32::from_rgb(200, 150, 255)));
                            ui.label(format!("= {:06X} (24 bits)", pi));
                            ui.label(
                                RichText::new(if msg.crc_valid { "→ CRC Valid" } else { "→ CRC Invalid" })
                                    .weak()
                            );
                        });
                    });
            });
    }
}
