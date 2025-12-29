//! Remote SDR Lab View
//!
//! Provides control of remote SDR agents (Raspberry Pi) for distributed
//! TX/RX testing from the GUI.
//!
//! Architecture:
//! ```text
//!  ┌─────────────────────────────────────────────────────┐
//!  │              Remote SDR Lab View                    │
//!  │  ┌─────────────────┐    ┌─────────────────────┐     │
//!  │  │ TX Agent (RPi1) │    │ RX Agent (RPi2)     │     │
//!  │  │ 192.168.x.x     │    │ 192.168.x.x         │     │
//!  │  │ [Configure]     │    │ [Configure]         │     │
//!  │  │ [Start/Stop]    │    │ [Start/Stop]        │     │
//!  │  └─────────────────┘    └─────────────────────┘     │
//!  └─────────────────────────────────────────────────────┘
//! ```

use egui::{Color32, RichText, Ui};
use r4w_core::agent::{AgentClient, AgentStatus, TaskStatus, DEFAULT_AGENT_PORT};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::thread;
use std::time::{Duration, Instant};

/// Remote agent configuration
#[derive(Clone)]
pub struct RemoteAgent {
    /// Friendly name
    pub name: String,
    /// Host address
    pub host: String,
    /// Control port
    pub port: u16,
    /// Is this agent connected?
    pub connected: bool,
    /// Last known status
    pub status: Option<AgentStatus>,
    /// Last error message
    pub error: Option<String>,
    /// Last connection attempt
    #[allow(dead_code)]
    pub last_attempt: Instant,
}

impl Default for RemoteAgent {
    fn default() -> Self {
        Self {
            name: "RPi".to_string(),
            host: "192.168.1.100".to_string(),
            port: DEFAULT_AGENT_PORT,
            connected: false,
            status: None,
            error: None,
            last_attempt: Instant::now(),
        }
    }
}

/// TX configuration
#[derive(Clone)]
pub struct TxConfig {
    pub waveform: String,
    pub sample_rate: f64,
    pub message: String,
    pub snr: f64,
    pub pps: u32,
    pub repeat: bool,
}

impl Default for TxConfig {
    fn default() -> Self {
        Self {
            waveform: "BPSK".to_string(),
            sample_rate: 48000.0,
            message: "Hello SDR!".to_string(),
            snr: -1.0,
            pps: 100,
            repeat: true,
        }
    }
}

/// RX configuration
#[derive(Clone)]
pub struct RxConfig {
    pub waveform: String,
    pub sample_rate: f64,
    pub udp_port: u16,
}

impl Default for RxConfig {
    fn default() -> Self {
        Self {
            waveform: "BPSK".to_string(),
            sample_rate: 48000.0,
            udp_port: 5000,
        }
    }
}

/// Command to send to background thread
#[allow(dead_code)]
enum AgentRequest {
    Connect(String, u16),
    Disconnect,
    GetStatus,
    StartTx(TxConfig, String), // config, target address
    StopTx,
    StartRx(RxConfig),
    StopRx,
    Ping,
    ListWaveforms,
    Shutdown,
}

/// Response from background thread
#[derive(Clone)]
enum AgentResponse {
    Connected(AgentStatus),
    Disconnected,
    Status(AgentStatus),
    Error(String),
    Pong(u64), // latency in ms
    Waveforms(Vec<String>),
    CommandOk(String),
}

/// Handle for communicating with a background agent thread
struct AgentHandle {
    tx: Sender<AgentRequest>,
    rx: Receiver<AgentResponse>,
}

/// Remote SDR Lab View
pub struct RemoteLabView {
    /// TX agent
    tx_agent: RemoteAgent,
    /// RX agent
    rx_agent: RemoteAgent,
    /// TX configuration
    tx_config: TxConfig,
    /// RX configuration
    rx_config: RxConfig,
    /// Available waveforms
    available_waveforms: Vec<String>,
    /// Background thread handles
    tx_handle: Option<AgentHandle>,
    rx_handle: Option<AgentHandle>,
    /// Status message
    status_message: String,
    /// Edit mode for agents
    edit_tx_agent: bool,
    edit_rx_agent: bool,
    /// Test latency results
    tx_latency: Option<u64>,
    rx_latency: Option<u64>,
}

impl Default for RemoteLabView {
    fn default() -> Self {
        Self::new()
    }
}

impl RemoteLabView {
    pub fn new() -> Self {
        let default_waveforms = vec![
            "CW", "OOK", "BPSK", "QPSK", "8-PSK", "16-PSK",
            "16-QAM", "64-QAM", "BFSK", "4-FSK", "OFDM",
            "LoRa", "DSSS", "FHSS", "Zigbee",
        ].into_iter().map(String::from).collect();

        Self {
            tx_agent: RemoteAgent {
                name: "TX (RPi 1)".to_string(),
                host: "192.168.179.104".to_string(), // From user input
                ..Default::default()
            },
            rx_agent: RemoteAgent {
                name: "RX (RPi 2)".to_string(),
                host: "192.168.179.129".to_string(), // From user input
                ..Default::default()
            },
            tx_config: TxConfig::default(),
            rx_config: RxConfig::default(),
            available_waveforms: default_waveforms,
            tx_handle: None,
            rx_handle: None,
            status_message: "Configure agents and click Connect".to_string(),
            edit_tx_agent: false,
            edit_rx_agent: false,
            tx_latency: None,
            rx_latency: None,
        }
    }

    /// Render the remote lab view
    pub fn render(&mut self, ui: &mut Ui) {
        // Poll for responses from background threads
        self.poll_responses();

        ui.heading("Remote SDR Lab");
        ui.label("Control remote Raspberry Pi agents for distributed TX/RX testing");
        ui.add_space(8.0);

        // TX Agent panel
        ui.group(|ui| {
            self.render_agent_panel(ui, true);
        });

        ui.add_space(8.0);

        // RX Agent panel
        ui.group(|ui| {
            self.render_agent_panel(ui, false);
        });

        ui.add_space(16.0);
        ui.separator();
        ui.add_space(8.0);

        // TX/RX Configuration
        ui.heading("Waveform Configuration");
        ui.add_space(8.0);

        // TX Config
        ui.group(|ui| {
            self.render_tx_config(ui);
        });

        ui.add_space(8.0);

        // RX Config
        ui.group(|ui| {
            self.render_rx_config(ui);
        });

        ui.add_space(16.0);
        ui.separator();
        ui.add_space(8.0);

        // Status and quick actions
        self.render_status_bar(ui);

        ui.add_space(8.0);

        // Help section
        ui.collapsing("Setup Instructions", |ui| {
            self.render_help(ui);
        });
    }

    /// Render agent panel (TX or RX)
    fn render_agent_panel(&mut self, ui: &mut Ui, is_tx: bool) {
        // Extract values needed for display first
        let (connected, has_error, error_msg, name, host, port, status_clone, latency) = {
            let agent = if is_tx { &self.tx_agent } else { &self.rx_agent };
            (
                agent.connected,
                agent.error.is_some(),
                agent.error.clone(),
                agent.name.clone(),
                agent.host.clone(),
                agent.port,
                agent.status.clone(),
                if is_tx { self.tx_latency } else { self.rx_latency },
            )
        };

        let edit_mode = if is_tx { self.edit_tx_agent } else { self.edit_rx_agent };

        // Header with status indicator
        ui.horizontal(|ui| {
            let status_color = if connected {
                Color32::GREEN
            } else if has_error {
                Color32::RED
            } else {
                Color32::GRAY
            };

            ui.label(RichText::new("●").color(status_color));
            ui.heading(&name);
        });

        // Edit button on separate line
        if ui.small_button(if edit_mode { "Done" } else { "Edit" }).clicked() {
            if is_tx {
                self.edit_tx_agent = !self.edit_tx_agent;
            } else {
                self.edit_rx_agent = !self.edit_rx_agent;
            }
        }

        ui.add_space(4.0);

        // Connection settings
        if edit_mode {
            let agent = if is_tx { &mut self.tx_agent } else { &mut self.rx_agent };
            ui.horizontal(|ui| {
                ui.label("Name:");
                ui.text_edit_singleline(&mut agent.name);
            });
            ui.horizontal(|ui| {
                ui.label("Host:");
                ui.text_edit_singleline(&mut agent.host);
            });
            ui.horizontal(|ui| {
                ui.label("Port:");
                ui.add(egui::DragValue::new(&mut agent.port).range(1..=65535));
            });
        } else {
            ui.label(format!("Host: {}:{}", host, port));
        }

        ui.add_space(8.0);

        // Connection controls - track button clicks
        let mut do_connect = false;
        let mut do_disconnect = false;
        let mut do_ping = false;
        let mut do_refresh = false;

        ui.horizontal(|ui| {
            if !connected {
                if ui.button("Connect").clicked() {
                    do_connect = true;
                }
            } else {
                if ui.button("Disconnect").clicked() {
                    do_disconnect = true;
                }

                if ui.button("Ping").clicked() {
                    do_ping = true;
                }

                if ui.button("Refresh").clicked() {
                    do_refresh = true;
                }
            }
        });

        // Handle button clicks after the closure
        if do_connect {
            self.connect_agent(is_tx);
        }
        if do_disconnect {
            self.disconnect_agent(is_tx);
        }
        if do_ping {
            self.ping_agent(is_tx);
        }
        if do_refresh {
            self.get_status(is_tx);
        }

        // Show latency if available
        if let Some(lat) = latency {
            ui.label(RichText::new(format!("Latency: {} ms", lat)).color(Color32::LIGHT_BLUE));
        }

        // Show error if any
        if let Some(ref err) = error_msg {
            ui.colored_label(Color32::RED, format!("Error: {}", err));
        }

        ui.add_space(8.0);

        // Status details if connected
        if let Some(ref status) = status_clone {
            ui.separator();
            ui.add_space(4.0);

            ui.label(format!("Device: {}", status.device.hostname));
            ui.label(format!("Arch: {} / {}", status.device.os, status.device.arch));
            ui.label(format!("Memory: {} MB", status.device.memory_mb));
            ui.label(format!("Uptime: {}s", status.uptime_secs));

            ui.add_space(4.0);

            // Task status
            let task_status = if is_tx { &status.tx_task } else { &status.rx_task };
            let (task_label, task_color) = match task_status {
                TaskStatus::Idle => ("Idle", Color32::GRAY),
                TaskStatus::Running { config, .. } => {
                    ui.label(format!("Running: {}", config));
                    ("Running", Color32::GREEN)
                }
                TaskStatus::Failed { error, .. } => {
                    ui.colored_label(Color32::RED, format!("Failed: {}", error));
                    ("Failed", Color32::RED)
                }
            };
            ui.horizontal(|ui| {
                ui.label("Task:");
                ui.colored_label(task_color, task_label);
            });
        }
    }

    /// Render TX configuration
    fn render_tx_config(&mut self, ui: &mut Ui) {
        ui.label(RichText::new("TX Configuration").strong());
        ui.add_space(4.0);

        // Waveform selection
        ui.label("Waveform:");
        egui::ComboBox::from_id_salt("tx_waveform")
            .selected_text(&self.tx_config.waveform)
            .width(150.0)
            .show_ui(ui, |ui| {
                for wf in &self.available_waveforms {
                    ui.selectable_value(&mut self.tx_config.waveform, wf.clone(), wf);
                }
            });
        ui.add_space(4.0);

        ui.label("Sample Rate:");
        ui.add(egui::DragValue::new(&mut self.tx_config.sample_rate)
            .range(1000.0..=10_000_000.0)
            .speed(1000.0)
            .suffix(" Hz"));
        ui.add_space(4.0);

        ui.label("Message:");
        ui.text_edit_singleline(&mut self.tx_config.message);
        ui.add_space(4.0);

        ui.label("SNR (dB):  (-1 = no noise)");
        ui.add(egui::DragValue::new(&mut self.tx_config.snr)
            .range(-1.0..=40.0)
            .speed(0.5));
        ui.add_space(4.0);

        ui.label("Packets/sec:");
        ui.add(egui::DragValue::new(&mut self.tx_config.pps)
            .range(1..=10000)
            .speed(10.0));
        ui.add_space(4.0);

        ui.checkbox(&mut self.tx_config.repeat, "Repeat continuously");

        ui.add_space(8.0);

        // Start/Stop TX
        ui.horizontal(|ui| {
            let tx_running = self.tx_agent.status.as_ref()
                .map(|s| matches!(s.tx_task, TaskStatus::Running { .. }))
                .unwrap_or(false);

            if !tx_running {
                let enabled = self.tx_agent.connected && self.rx_agent.connected;
                if ui.add_enabled(enabled, egui::Button::new("Start TX")).clicked() {
                    self.start_tx();
                }
                if !enabled {
                    ui.label(RichText::new("Connect both agents first").color(Color32::GRAY));
                }
            } else {
                if ui.add(egui::Button::new("Stop TX").fill(Color32::from_rgb(180, 60, 60))).clicked() {
                    self.stop_tx();
                }
            }
        });
    }

    /// Render RX configuration
    fn render_rx_config(&mut self, ui: &mut Ui) {
        ui.label(RichText::new("RX Configuration").strong());
        ui.add_space(4.0);

        // Waveform selection (should match TX)
        ui.label("Waveform:");
        ui.horizontal(|ui| {
            egui::ComboBox::from_id_salt("rx_waveform")
                .selected_text(&self.rx_config.waveform)
                .width(150.0)
                .show_ui(ui, |ui| {
                    for wf in &self.available_waveforms {
                        ui.selectable_value(&mut self.rx_config.waveform, wf.clone(), wf);
                    }
                });
            // Sync with TX button
            if ui.small_button("Sync with TX").clicked() {
                self.rx_config.waveform = self.tx_config.waveform.clone();
                self.rx_config.sample_rate = self.tx_config.sample_rate;
            }
        });
        ui.add_space(4.0);

        ui.label("Sample Rate:");
        ui.add(egui::DragValue::new(&mut self.rx_config.sample_rate)
            .range(1000.0..=10_000_000.0)
            .speed(1000.0)
            .suffix(" Hz"));
        ui.add_space(4.0);

        ui.label("UDP Port:");
        ui.add(egui::DragValue::new(&mut self.rx_config.udp_port)
            .range(1024..=65535));

        ui.add_space(8.0);

        // Start/Stop RX
        ui.horizontal(|ui| {
            let rx_running = self.rx_agent.status.as_ref()
                .map(|s| matches!(s.rx_task, TaskStatus::Running { .. }))
                .unwrap_or(false);

            if !rx_running {
                let enabled = self.rx_agent.connected;
                if ui.add_enabled(enabled, egui::Button::new("Start RX")).clicked() {
                    self.start_rx();
                }
            } else {
                if ui.add(egui::Button::new("Stop RX").fill(Color32::from_rgb(180, 60, 60))).clicked() {
                    self.stop_rx();
                }
            }
        });
    }

    /// Render status bar
    fn render_status_bar(&mut self, ui: &mut Ui) {
        ui.horizontal(|ui| {
            ui.label("Status:");
            ui.label(&self.status_message);

            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                // Quick actions
                if ui.button("Stop All").clicked() {
                    self.stop_all();
                }

                if ui.button("Start TX→RX Test").clicked() {
                    self.start_test();
                }
            });
        });
    }

    /// Render help section
    fn render_help(&mut self, ui: &mut Ui) {
        ui.label("1. Deploy the agent to each Raspberry Pi:");
        ui.code("make deploy-arm64 REMOTE_HOST=192.168.x.x REMOTE_USER=pi");
        ui.add_space(4.0);

        ui.label("2. Start the agent on each Pi (via SSH or systemd):");
        ui.code("r4w agent --port 6000");
        ui.add_space(4.0);

        ui.label("3. Connect to both agents using the controls above");
        ui.add_space(4.0);

        ui.label("4. Configure TX and RX waveforms (should match)");
        ui.add_space(4.0);

        ui.label("5. Click 'Start TX→RX Test' to begin");
        ui.add_space(4.0);

        ui.separator();
        ui.label("The TX agent will send samples via UDP to the RX agent.");
        ui.label("The RX agent demodulates and reports metrics.");
    }

    // Background thread communication methods

    fn connect_agent(&mut self, is_tx: bool) {
        let agent = if is_tx { &self.tx_agent } else { &self.rx_agent };
        let host = agent.host.clone();
        let port = agent.port;

        // Create channel pair
        let (cmd_tx, cmd_rx) = channel();
        let (resp_tx, resp_rx) = channel();

        // Spawn background thread
        thread::spawn(move || {
            Self::agent_thread(host, port, cmd_rx, resp_tx);
        });

        let handle = AgentHandle { tx: cmd_tx, rx: resp_rx };

        // Send connect command
        let _ = handle.tx.send(AgentRequest::Connect(
            if is_tx { self.tx_agent.host.clone() } else { self.rx_agent.host.clone() },
            if is_tx { self.tx_agent.port } else { self.rx_agent.port },
        ));

        if is_tx {
            self.tx_handle = Some(handle);
        } else {
            self.rx_handle = Some(handle);
        }

        self.status_message = format!("Connecting to {}...", if is_tx { "TX agent" } else { "RX agent" });
    }

    fn disconnect_agent(&mut self, is_tx: bool) {
        let handle = if is_tx { &self.tx_handle } else { &self.rx_handle };
        if let Some(h) = handle {
            let _ = h.tx.send(AgentRequest::Disconnect);
        }

        if is_tx {
            self.tx_agent.connected = false;
            self.tx_agent.status = None;
            self.tx_handle = None;
        } else {
            self.rx_agent.connected = false;
            self.rx_agent.status = None;
            self.rx_handle = None;
        }
    }

    fn ping_agent(&mut self, is_tx: bool) {
        let handle = if is_tx { &self.tx_handle } else { &self.rx_handle };
        if let Some(h) = handle {
            let _ = h.tx.send(AgentRequest::Ping);
        }
    }

    fn get_status(&mut self, is_tx: bool) {
        let handle = if is_tx { &self.tx_handle } else { &self.rx_handle };
        if let Some(h) = handle {
            let _ = h.tx.send(AgentRequest::GetStatus);
        }
    }

    fn start_tx(&mut self) {
        // Build target address: RX agent's IP + UDP port
        let target = format!("{}:{}", self.rx_agent.host, self.rx_config.udp_port);

        if let Some(ref h) = self.tx_handle {
            let _ = h.tx.send(AgentRequest::StartTx(self.tx_config.clone(), target));
        }
        self.status_message = "Starting TX...".to_string();
    }

    fn stop_tx(&mut self) {
        if let Some(ref h) = self.tx_handle {
            let _ = h.tx.send(AgentRequest::StopTx);
        }
        self.status_message = "Stopping TX...".to_string();
    }

    fn start_rx(&mut self) {
        if let Some(ref h) = self.rx_handle {
            let _ = h.tx.send(AgentRequest::StartRx(self.rx_config.clone()));
        }
        self.status_message = "Starting RX...".to_string();
    }

    fn stop_rx(&mut self) {
        if let Some(ref h) = self.rx_handle {
            let _ = h.tx.send(AgentRequest::StopRx);
        }
        self.status_message = "Stopping RX...".to_string();
    }

    fn stop_all(&mut self) {
        self.stop_tx();
        self.stop_rx();
        self.status_message = "Stopping all...".to_string();
    }

    fn start_test(&mut self) {
        // Sync waveforms
        self.rx_config.waveform = self.tx_config.waveform.clone();
        self.rx_config.sample_rate = self.tx_config.sample_rate;

        // Start RX first, then TX
        self.start_rx();

        // Delay TX start slightly (user should see RX started first)
        self.status_message = "Starting test: RX first, then TX...".to_string();

        // TODO: Could use a timer to delay TX start
        self.start_tx();
    }

    fn poll_responses(&mut self) {
        // Collect responses first to avoid borrow conflicts
        let mut tx_responses = Vec::new();
        let mut rx_responses = Vec::new();

        // Poll TX handle
        if let Some(ref h) = self.tx_handle {
            while let Ok(resp) = h.rx.try_recv() {
                tx_responses.push(resp);
            }
        }

        // Poll RX handle
        if let Some(ref h) = self.rx_handle {
            while let Ok(resp) = h.rx.try_recv() {
                rx_responses.push(resp);
            }
        }

        // Now process collected responses
        for resp in tx_responses {
            self.handle_response(true, resp);
        }
        for resp in rx_responses {
            self.handle_response(false, resp);
        }
    }

    fn handle_response(&mut self, is_tx: bool, resp: AgentResponse) {
        // Track if we need to refresh status afterwards
        let mut do_refresh = false;
        let mut new_status_message: Option<String> = None;

        match resp {
            AgentResponse::Connected(status) => {
                let agent = if is_tx { &mut self.tx_agent } else { &mut self.rx_agent };
                let name = agent.name.clone();
                agent.connected = true;
                agent.status = Some(status);
                agent.error = None;
                new_status_message = Some(format!("{} connected", name));
            }
            AgentResponse::Disconnected => {
                let agent = if is_tx { &mut self.tx_agent } else { &mut self.rx_agent };
                agent.connected = false;
                agent.status = None;
            }
            AgentResponse::Status(status) => {
                let agent = if is_tx { &mut self.tx_agent } else { &mut self.rx_agent };
                agent.status = Some(status);
                agent.error = None;
            }
            AgentResponse::Error(err) => {
                let agent = if is_tx { &mut self.tx_agent } else { &mut self.rx_agent };
                let name = agent.name.clone();
                agent.error = Some(err.clone());
                new_status_message = Some(format!("{}: {}", name, err));
            }
            AgentResponse::Pong(latency) => {
                if is_tx {
                    self.tx_latency = Some(latency);
                } else {
                    self.rx_latency = Some(latency);
                }
            }
            AgentResponse::Waveforms(wfs) => {
                self.available_waveforms = wfs;
            }
            AgentResponse::CommandOk(msg) => {
                let name = if is_tx { &self.tx_agent.name } else { &self.rx_agent.name };
                new_status_message = Some(format!("{}: {}", name, msg));
                do_refresh = true;
            }
        }

        // Apply status message if set
        if let Some(msg) = new_status_message {
            self.status_message = msg;
        }

        // Refresh status after command if needed
        if do_refresh {
            self.get_status(is_tx);
        }
    }

    /// Background thread for agent communication
    fn agent_thread(
        _host: String,
        _port: u16,
        cmd_rx: Receiver<AgentRequest>,
        resp_tx: Sender<AgentResponse>,
    ) {
        let mut client: Option<AgentClient> = None;

        loop {
            // Receive command (with timeout to allow checking for disconnect)
            match cmd_rx.recv_timeout(Duration::from_millis(500)) {
                Ok(cmd) => {
                    match cmd {
                        AgentRequest::Connect(host, port) => {
                            match AgentClient::connect((&*host, port)) {
                                Ok(c) => {
                                    let mut c = c;
                                    match c.status() {
                                        Ok(status) => {
                                            client = Some(c);
                                            let _ = resp_tx.send(AgentResponse::Connected(status));
                                        }
                                        Err(e) => {
                                            let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                        }
                                    }
                                }
                                Err(e) => {
                                    let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                }
                            }
                        }
                        AgentRequest::Disconnect => {
                            drop(client.take());
                            let _ = resp_tx.send(AgentResponse::Disconnected);
                            break; // Exit thread
                        }
                        AgentRequest::GetStatus => {
                            if let Some(ref mut c) = client {
                                match c.status() {
                                    Ok(status) => {
                                        let _ = resp_tx.send(AgentResponse::Status(status));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::Ping => {
                            if let Some(ref mut c) = client {
                                match c.ping() {
                                    Ok(latency) => {
                                        let _ = resp_tx.send(AgentResponse::Pong(latency));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::StartTx(config, target) => {
                            if let Some(ref mut c) = client {
                                match c.start_tx(
                                    &target,
                                    &config.waveform,
                                    config.sample_rate,
                                    &config.message,
                                    config.snr,
                                    config.pps,
                                    config.repeat,
                                ) {
                                    Ok(msg) => {
                                        let _ = resp_tx.send(AgentResponse::CommandOk(msg));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::StopTx => {
                            if let Some(ref mut c) = client {
                                match c.stop_tx() {
                                    Ok(()) => {
                                        let _ = resp_tx.send(AgentResponse::CommandOk("TX stopped".to_string()));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::StartRx(config) => {
                            if let Some(ref mut c) = client {
                                match c.start_rx(config.udp_port, &config.waveform, config.sample_rate) {
                                    Ok(msg) => {
                                        let _ = resp_tx.send(AgentResponse::CommandOk(msg));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::StopRx => {
                            if let Some(ref mut c) = client {
                                match c.stop_rx() {
                                    Ok(()) => {
                                        let _ = resp_tx.send(AgentResponse::CommandOk("RX stopped".to_string()));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::ListWaveforms => {
                            if let Some(ref mut c) = client {
                                match c.list_waveforms() {
                                    Ok(wfs) => {
                                        let names: Vec<String> = wfs.iter().map(|w| w.name.clone()).collect();
                                        let _ = resp_tx.send(AgentResponse::Waveforms(names));
                                    }
                                    Err(e) => {
                                        let _ = resp_tx.send(AgentResponse::Error(e.to_string()));
                                    }
                                }
                            }
                        }
                        AgentRequest::Shutdown => {
                            if let Some(ref mut c) = client {
                                let _ = c.shutdown();
                            }
                            break;
                        }
                    }
                }
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    // No command, continue loop
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => {
                    // Channel closed, exit
                    break;
                }
            }
        }
    }
}
