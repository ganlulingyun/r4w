//! Agent client for connecting to remote agents
//!
//! Used by the GUI and CLI to control remote SDR agents.

use super::protocol::*;
use super::DEFAULT_AGENT_PORT;
use std::io::{BufRead, BufReader, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::time::Duration;

/// Client for communicating with a remote agent
pub struct AgentClient {
    stream: TcpStream,
    reader: BufReader<TcpStream>,
}

/// Connection error
#[derive(Debug)]
pub struct ConnectionError {
    pub message: String,
}

impl std::fmt::Display for ConnectionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for ConnectionError {}

impl From<std::io::Error> for ConnectionError {
    fn from(e: std::io::Error) -> Self {
        Self {
            message: e.to_string(),
        }
    }
}

impl AgentClient {
    /// Connect to an agent at the given address
    pub fn connect<A: ToSocketAddrs>(addr: A) -> Result<Self, ConnectionError> {
        let stream = TcpStream::connect(addr)?;
        stream.set_read_timeout(Some(Duration::from_secs(30)))?;
        stream.set_write_timeout(Some(Duration::from_secs(10)))?;

        let reader = BufReader::new(stream.try_clone()?);

        Ok(Self { stream, reader })
    }

    /// Connect to agent on default port
    pub fn connect_default(host: &str) -> Result<Self, ConnectionError> {
        Self::connect((host, DEFAULT_AGENT_PORT))
    }

    /// Send a command and receive the response
    pub fn send_command(&mut self, cmd: &AgentCommand) -> Result<AgentResponse, ConnectionError> {
        let json = serde_json::to_string(cmd).map_err(|e| ConnectionError {
            message: format!("Serialization error: {}", e),
        })?;

        writeln!(self.stream, "{}", json)?;
        self.stream.flush()?;

        let mut response_line = String::new();
        self.reader.read_line(&mut response_line)?;

        serde_json::from_str(&response_line).map_err(|e| ConnectionError {
            message: format!("Parse error: {} (response: {})", e, response_line),
        })
    }

    /// Get agent status
    pub fn status(&mut self) -> Result<AgentStatus, ConnectionError> {
        match self.send_command(&AgentCommand::Status)? {
            AgentResponse::Status(status) => Ok(status),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Ping the agent
    pub fn ping(&mut self) -> Result<u64, ConnectionError> {
        let start = std::time::Instant::now();
        match self.send_command(&AgentCommand::Ping)? {
            AgentResponse::Pong { timestamp: _ } => Ok(start.elapsed().as_millis() as u64),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Start transmitting
    pub fn start_tx(
        &mut self,
        target: &str,
        waveform: &str,
        sample_rate: f64,
        message: &str,
        snr: f64,
        pps: u32,
        repeat: bool,
    ) -> Result<String, ConnectionError> {
        let cmd = AgentCommand::StartTx {
            target: target.to_string(),
            waveform: waveform.to_string(),
            sample_rate,
            message: message.to_string(),
            snr,
            pps,
            repeat,
        };

        match self.send_command(&cmd)? {
            AgentResponse::Ok { message, .. } => {
                Ok(message.unwrap_or_else(|| "TX started".to_string()))
            }
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Stop transmitting
    pub fn stop_tx(&mut self) -> Result<(), ConnectionError> {
        match self.send_command(&AgentCommand::StopTx)? {
            AgentResponse::Ok { .. } => Ok(()),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Start receiving
    pub fn start_rx(
        &mut self,
        port: u16,
        waveform: &str,
        sample_rate: f64,
    ) -> Result<String, ConnectionError> {
        let cmd = AgentCommand::StartRx {
            port,
            waveform: waveform.to_string(),
            sample_rate,
            report_metrics: true,
            metrics_interval: 1,
        };

        match self.send_command(&cmd)? {
            AgentResponse::Ok { message, .. } => {
                Ok(message.unwrap_or_else(|| "RX started".to_string()))
            }
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Stop receiving
    pub fn stop_rx(&mut self) -> Result<(), ConnectionError> {
        match self.send_command(&AgentCommand::StopRx)? {
            AgentResponse::Ok { .. } => Ok(()),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Get current metrics
    pub fn get_metrics(&mut self) -> Result<MetricsData, ConnectionError> {
        match self.send_command(&AgentCommand::GetMetrics)? {
            AgentResponse::Metrics(data) => Ok(data),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// List available waveforms
    pub fn list_waveforms(&mut self) -> Result<Vec<WaveformInfo>, ConnectionError> {
        match self.send_command(&AgentCommand::ListWaveforms)? {
            AgentResponse::Waveforms { waveforms } => Ok(waveforms),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }

    /// Shutdown the remote agent
    pub fn shutdown(&mut self) -> Result<(), ConnectionError> {
        match self.send_command(&AgentCommand::Shutdown)? {
            AgentResponse::Ok { .. } => Ok(()),
            AgentResponse::Error { message, .. } => Err(ConnectionError { message }),
            other => Err(ConnectionError {
                message: format!("Unexpected response: {:?}", other),
            }),
        }
    }
}

/// Non-blocking agent client for GUI use
pub struct AsyncAgentClient {
    host: String,
    port: u16,
}

impl AsyncAgentClient {
    /// Create a new async client
    pub fn new(host: &str, port: u16) -> Self {
        Self {
            host: host.to_string(),
            port,
        }
    }

    /// Connect and send a single command
    pub fn send(&self, cmd: AgentCommand) -> Result<AgentResponse, ConnectionError> {
        let mut client = AgentClient::connect((&*self.host, self.port))?;
        client.send_command(&cmd)
    }

    /// Check if agent is reachable (non-blocking with short timeout)
    pub fn is_reachable(&self) -> bool {
        let addr = format!("{}:{}", self.host, self.port);
        if let Ok(stream) = TcpStream::connect_timeout(
            &addr.parse().unwrap(),
            Duration::from_millis(500),
        ) {
            drop(stream);
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_async_client_creation() {
        let client = AsyncAgentClient::new("192.168.1.100", 6000);
        assert_eq!(client.host, "192.168.1.100");
        assert_eq!(client.port, 6000);
    }
}
