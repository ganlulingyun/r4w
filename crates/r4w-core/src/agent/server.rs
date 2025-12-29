//! Agent server implementation
//!
//! Runs on remote devices and accepts commands from the controller.

use super::protocol::*;
use super::DEFAULT_AGENT_PORT;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// Agent server that accepts commands over TCP
pub struct AgentServer {
    port: u16,
    running: Arc<AtomicBool>,
    start_time: Instant,
    tx_process: Arc<Mutex<Option<Child>>>,
    rx_process: Arc<Mutex<Option<Child>>>,
    tx_status: Arc<Mutex<TaskStatus>>,
    rx_status: Arc<Mutex<TaskStatus>>,
    metrics_callback: Option<Box<dyn Fn(MetricsData) + Send + Sync>>,
}

impl AgentServer {
    /// Create a new agent server
    pub fn new(port: u16) -> Self {
        Self {
            port,
            running: Arc::new(AtomicBool::new(false)),
            start_time: Instant::now(),
            tx_process: Arc::new(Mutex::new(None)),
            rx_process: Arc::new(Mutex::new(None)),
            tx_status: Arc::new(Mutex::new(TaskStatus::Idle)),
            rx_status: Arc::new(Mutex::new(TaskStatus::Idle)),
            metrics_callback: None,
        }
    }

    /// Create with default port
    pub fn default() -> Self {
        Self::new(DEFAULT_AGENT_PORT)
    }

    /// Set metrics callback
    pub fn with_metrics_callback<F>(mut self, f: F) -> Self
    where
        F: Fn(MetricsData) + Send + Sync + 'static,
    {
        self.metrics_callback = Some(Box::new(f));
        self
    }

    /// Run the agent server (blocking)
    pub fn run(&mut self) -> std::io::Result<()> {
        let listener = TcpListener::bind(("0.0.0.0", self.port))?;
        listener.set_nonblocking(true)?;

        self.running.store(true, Ordering::SeqCst);

        eprintln!("Agent server listening on port {}", self.port);
        eprintln!("Device info: {:?}", DeviceInfo::gather());

        while self.running.load(Ordering::SeqCst) {
            match listener.accept() {
                Ok((stream, addr)) => {
                    eprintln!("Connection from {}", addr);
                    if let Err(e) = self.handle_connection(stream) {
                        eprintln!("Connection error: {}", e);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    // No connection, sleep a bit
                    std::thread::sleep(Duration::from_millis(100));
                }
                Err(e) => {
                    eprintln!("Accept error: {}", e);
                }
            }

            // Check process status
            self.check_processes();
        }

        // Cleanup
        self.stop_all();

        Ok(())
    }

    /// Stop the server
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }

    fn handle_connection(&mut self, mut stream: TcpStream) -> std::io::Result<()> {
        stream.set_read_timeout(Some(Duration::from_secs(30)))?;
        stream.set_write_timeout(Some(Duration::from_secs(10)))?;

        let reader = BufReader::new(stream.try_clone()?);

        for line in reader.lines() {
            let line = match line {
                Ok(l) => l,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => continue,
                Err(e) => {
                    eprintln!("Read error: {}", e);
                    break;
                }
            };

            if line.is_empty() {
                continue;
            }

            eprintln!("Received: {}", line);

            let response = match serde_json::from_str::<AgentCommand>(&line) {
                Ok(cmd) => self.handle_command(cmd),
                Err(e) => AgentResponse::Error {
                    message: format!("Invalid command: {}", e),
                    code: Some(400),
                },
            };

            let response_json = serde_json::to_string(&response).unwrap_or_else(|_| {
                r#"{"status":"error","message":"Serialization failed"}"#.to_string()
            });

            writeln!(stream, "{}", response_json)?;
            stream.flush()?;

            // Check for shutdown command
            if matches!(response, AgentResponse::Ok { .. })
                && line.contains("shutdown")
            {
                self.running.store(false, Ordering::SeqCst);
                break;
            }
        }

        Ok(())
    }

    fn handle_command(&mut self, cmd: AgentCommand) -> AgentResponse {
        match cmd {
            AgentCommand::Status => self.cmd_status(),
            AgentCommand::Ping => self.cmd_ping(),
            AgentCommand::StartTx {
                target,
                waveform,
                sample_rate,
                message,
                snr,
                pps,
                repeat,
            } => self.cmd_start_tx(target, waveform, sample_rate, message, snr, pps, repeat),
            AgentCommand::StopTx => self.cmd_stop_tx(),
            AgentCommand::StartRx {
                port,
                waveform,
                sample_rate,
                report_metrics: _,
                metrics_interval: _,
            } => self.cmd_start_rx(port, waveform, sample_rate),
            AgentCommand::StopRx => self.cmd_stop_rx(),
            AgentCommand::GetMetrics => self.cmd_get_metrics(),
            AgentCommand::Configure { settings: _ } => AgentResponse::Ok {
                message: Some("Configure not yet implemented".to_string()),
                data: None,
            },
            AgentCommand::ListWaveforms => self.cmd_list_waveforms(),
            AgentCommand::Shutdown => {
                self.stop_all();
                AgentResponse::Ok {
                    message: Some("Shutting down".to_string()),
                    data: None,
                }
            }
        }
    }

    fn cmd_status(&self) -> AgentResponse {
        let tx_status = self.tx_status.lock().unwrap().clone();
        let rx_status = self.rx_status.lock().unwrap().clone();

        AgentResponse::Status(AgentStatus {
            version: env!("CARGO_PKG_VERSION").to_string(),
            device: DeviceInfo::gather(),
            tx_task: tx_status,
            rx_task: rx_status,
            uptime_secs: self.start_time.elapsed().as_secs(),
        })
    }

    fn cmd_ping(&self) -> AgentResponse {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;

        AgentResponse::Pong { timestamp }
    }

    fn cmd_start_tx(
        &self,
        target: String,
        waveform: String,
        sample_rate: f64,
        message: String,
        snr: f64,
        pps: u32,
        repeat: bool,
    ) -> AgentResponse {
        // Stop existing TX if any
        self.stop_process(&self.tx_process, &self.tx_status);

        // Build command
        let sdr_cli_path = find_sdr_cli();
        eprintln!("Using r4w at: {:?}", sdr_cli_path);
        let mut cmd = Command::new(&sdr_cli_path);
        cmd.arg("udp-send")
            .arg("-t")
            .arg(&target)
            .arg("-w")
            .arg(&waveform)
            .arg("-s")
            .arg(sample_rate.to_string())
            .arg("-m")
            .arg(&message)
            .arg("--pps")
            .arg(pps.to_string());

        if snr >= 0.0 {
            cmd.arg("--snr").arg(snr.to_string());
        }

        if repeat {
            cmd.arg("--repeat");
        }

        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());

        match cmd.spawn() {
            Ok(child) => {
                let config = format!(
                    "TX {} -> {} @ {} Hz",
                    waveform, target, sample_rate
                );
                *self.tx_process.lock().unwrap() = Some(child);
                *self.tx_status.lock().unwrap() = TaskStatus::Running {
                    started_at: unix_timestamp(),
                    config,
                };

                AgentResponse::Ok {
                    message: Some(format!("Started TX: {} -> {}", waveform, target)),
                    data: None,
                }
            }
            Err(e) => {
                *self.tx_status.lock().unwrap() = TaskStatus::Failed {
                    error: e.to_string(),
                    failed_at: unix_timestamp(),
                };

                AgentResponse::Error {
                    message: format!("Failed to start TX: {}", e),
                    code: Some(500),
                }
            }
        }
    }

    fn cmd_stop_tx(&self) -> AgentResponse {
        self.stop_process(&self.tx_process, &self.tx_status);
        AgentResponse::Ok {
            message: Some("TX stopped".to_string()),
            data: None,
        }
    }

    fn cmd_start_rx(
        &self,
        port: u16,
        waveform: String,
        sample_rate: f64,
    ) -> AgentResponse {
        // Stop existing RX if any
        self.stop_process(&self.rx_process, &self.rx_status);

        // Build command
        let sdr_cli_path = find_sdr_cli();
        eprintln!("Using r4w at: {:?}", sdr_cli_path);
        let mut cmd = Command::new(&sdr_cli_path);
        cmd.arg("benchmark")
            .arg("-p")
            .arg(port.to_string())
            .arg("-w")
            .arg(&waveform)
            .arg("-s")
            .arg(sample_rate.to_string())
            .arg("--stats-interval")
            .arg("1");

        cmd.stdout(Stdio::piped());
        cmd.stderr(Stdio::piped());

        match cmd.spawn() {
            Ok(child) => {
                let config = format!(
                    "RX {} on port {} @ {} Hz",
                    waveform, port, sample_rate
                );
                *self.rx_process.lock().unwrap() = Some(child);
                *self.rx_status.lock().unwrap() = TaskStatus::Running {
                    started_at: unix_timestamp(),
                    config,
                };

                AgentResponse::Ok {
                    message: Some(format!("Started RX: {} on port {}", waveform, port)),
                    data: None,
                }
            }
            Err(e) => {
                *self.rx_status.lock().unwrap() = TaskStatus::Failed {
                    error: e.to_string(),
                    failed_at: unix_timestamp(),
                };

                AgentResponse::Error {
                    message: format!("Failed to start RX: {}", e),
                    code: Some(500),
                }
            }
        }
    }

    fn cmd_stop_rx(&self) -> AgentResponse {
        self.stop_process(&self.rx_process, &self.rx_status);
        AgentResponse::Ok {
            message: Some("RX stopped".to_string()),
            data: None,
        }
    }

    fn cmd_get_metrics(&self) -> AgentResponse {
        // Return current metrics (placeholder)
        AgentResponse::Metrics(MetricsData {
            timestamp: unix_timestamp(),
            kind: MetricsKind::Rx,
            samples: 0,
            bytes: 0,
            throughput_sps: 0.0,
            symbols_detected: None,
            avg_latency_us: None,
            ber_estimate: None,
            packets_sent: None,
        })
    }

    fn cmd_list_waveforms(&self) -> AgentResponse {
        use crate::waveform::WaveformFactory;

        let waveforms: Vec<WaveformInfo> = WaveformFactory::list()
            .iter()
            .filter_map(|name| {
                WaveformFactory::create(name, 48000.0).map(|wf| {
                    let info = wf.info();
                    WaveformInfo {
                        name: info.name.to_string(),
                        full_name: info.full_name.to_string(),
                        bits_per_symbol: info.bits_per_symbol,
                        carries_data: info.carries_data,
                    }
                })
            })
            .collect();

        AgentResponse::Waveforms { waveforms }
    }

    fn stop_process(
        &self,
        process: &Arc<Mutex<Option<Child>>>,
        status: &Arc<Mutex<TaskStatus>>,
    ) {
        let mut proc_guard = process.lock().unwrap();
        if let Some(ref mut child) = *proc_guard {
            let _ = child.kill();
            let _ = child.wait();
        }
        *proc_guard = None;
        *status.lock().unwrap() = TaskStatus::Idle;
    }

    fn stop_all(&self) {
        self.stop_process(&self.tx_process, &self.tx_status);
        self.stop_process(&self.rx_process, &self.rx_status);
    }

    fn check_processes(&self) {
        // Check TX process
        let mut tx_guard = self.tx_process.lock().unwrap();
        if let Some(ref mut child) = *tx_guard {
            match child.try_wait() {
                Ok(Some(status)) => {
                    // Process exited
                    if !status.success() {
                        *self.tx_status.lock().unwrap() = TaskStatus::Failed {
                            error: format!("Process exited with status: {}", status),
                            failed_at: unix_timestamp(),
                        };
                    } else {
                        *self.tx_status.lock().unwrap() = TaskStatus::Idle;
                    }
                    *tx_guard = None;
                }
                Ok(None) => {
                    // Still running
                }
                Err(e) => {
                    *self.tx_status.lock().unwrap() = TaskStatus::Failed {
                        error: format!("Error checking process: {}", e),
                        failed_at: unix_timestamp(),
                    };
                }
            }
        }

        // Check RX process
        let mut rx_guard = self.rx_process.lock().unwrap();
        if let Some(ref mut child) = *rx_guard {
            match child.try_wait() {
                Ok(Some(status)) => {
                    if !status.success() {
                        *self.rx_status.lock().unwrap() = TaskStatus::Failed {
                            error: format!("Process exited with status: {}", status),
                            failed_at: unix_timestamp(),
                        };
                    } else {
                        *self.rx_status.lock().unwrap() = TaskStatus::Idle;
                    }
                    *rx_guard = None;
                }
                Ok(None) => {
                    // Still running
                }
                Err(e) => {
                    *self.rx_status.lock().unwrap() = TaskStatus::Failed {
                        error: format!("Error checking process: {}", e),
                        failed_at: unix_timestamp(),
                    };
                }
            }
        }
    }
}

fn unix_timestamp() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs()
}

/// Find the r4w binary path
/// Looks in the same directory as the current executable first,
/// then falls back to common locations and PATH.
fn find_sdr_cli() -> PathBuf {
    // Try same directory as current executable
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let sdr_cli = dir.join("r4w");
            if sdr_cli.exists() {
                return sdr_cli;
            }
        }
    }

    // Try ~/.local/bin
    if let Some(home) = std::env::var_os("HOME") {
        let local_bin = PathBuf::from(home).join(".local/bin/r4w");
        if local_bin.exists() {
            return local_bin;
        }
    }

    // Try /usr/local/bin
    let usr_local = PathBuf::from("/usr/local/bin/r4w");
    if usr_local.exists() {
        return usr_local;
    }

    // Fall back to PATH lookup
    PathBuf::from("r4w")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_device_info() {
        let info = DeviceInfo::gather();
        assert!(!info.hostname.is_empty());
        assert!(!info.os.is_empty());
        assert!(!info.arch.is_empty());
    }
}
