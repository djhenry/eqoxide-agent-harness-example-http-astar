//! Agent Plugin API client (eqoxide spec §10) — connects to a running eqoxide instance's
//! `--agent-socket` Unix domain socket, performs the version handshake, and exchanges `Step`/
//! `Observation` over it. Mirrors eqoxide's own
//! `crates/eqoxide-agent-protocol/examples/fixed_sequence_client.rs`, productized as a reusable
//! type instead of a one-shot script.

use eqoxide_agent_protocol::framing::{decode_line, encode_line};
use eqoxide_agent_protocol::handshake::{HandshakeReply, Hello, PROTOCOL_VERSION};
use eqoxide_agent_protocol::observation::Observation;
use eqoxide_agent_protocol::step::Step;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::Path;

#[derive(Debug)]
pub enum ClientError {
    Io(std::io::Error),
    Json(serde_json::Error),
    HandshakeRejected {
        server_protocol_version: u32,
        message: String,
    },
    ConnectionClosed,
}

impl std::fmt::Display for ClientError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ClientError::Io(e) => write!(f, "io error: {e}"),
            ClientError::Json(e) => write!(f, "protocol json error: {e}"),
            ClientError::HandshakeRejected {
                server_protocol_version,
                message,
            } => {
                write!(f, "handshake rejected (server_protocol_version={server_protocol_version}): {message}")
            }
            ClientError::ConnectionClosed => write!(f, "connection closed by server"),
        }
    }
}

impl std::error::Error for ClientError {}
impl From<std::io::Error> for ClientError {
    fn from(e: std::io::Error) -> Self {
        ClientError::Io(e)
    }
}
impl From<serde_json::Error> for ClientError {
    fn from(e: serde_json::Error) -> Self {
        ClientError::Json(e)
    }
}

/// A connected, handshaken Agent Plugin API session.
pub struct AgentClient {
    writer: UnixStream,
    reader: BufReader<UnixStream>,
}

impl AgentClient {
    /// Connect to `path` and perform the version handshake. Returns `Err` on a `Rejected` reply,
    /// an I/O error, or a malformed line — matching the handshake contract in eqoxide's
    /// docs/agent-api.md.
    pub fn connect(path: impl AsRef<Path>) -> Result<Self, ClientError> {
        let stream = UnixStream::connect(path)?;
        let writer = stream.try_clone()?;
        let reader = BufReader::new(stream);
        let mut client = AgentClient { writer, reader };
        client.handshake()?;
        Ok(client)
    }

    fn handshake(&mut self) -> Result<(), ClientError> {
        let hello = encode_line(&Hello {
            protocol_version: PROTOCOL_VERSION,
        })?;
        self.writer.write_all(hello.as_bytes())?;

        let mut line = String::new();
        let n = self.reader.read_line(&mut line)?;
        if n == 0 {
            return Err(ClientError::ConnectionClosed);
        }
        let reply: HandshakeReply = decode_line(&line)?;
        match reply {
            HandshakeReply::Accepted => Ok(()),
            HandshakeReply::Rejected {
                server_protocol_version,
                message,
            } => Err(ClientError::HandshakeRejected {
                server_protocol_version,
                message,
            }),
        }
    }

    /// Send one `Step`. There is no per-`Step` acknowledgment — its effect shows up in the next
    /// `recv_observation()` call, not a direct reply (eqoxide's docs/agent-api.md).
    pub fn send_step(&mut self, step: &Step) -> Result<(), ClientError> {
        let line = encode_line(step)?;
        self.writer.write_all(line.as_bytes())?;
        Ok(())
    }

    /// Block for the next `Observation` line.
    pub fn recv_observation(&mut self) -> Result<Observation, ClientError> {
        let mut line = String::new();
        let n = self.reader.read_line(&mut line)?;
        if n == 0 {
            return Err(ClientError::ConnectionClosed);
        }
        Ok(decode_line(&line)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eqoxide_agent_protocol::movement::AgentMovement;
    use eqoxide_agent_protocol::observation::{LegalActionMask, OwnState};
    use std::os::unix::net::UnixListener;

    fn canned_observation(tick: u64) -> Observation {
        Observation {
            own: OwnState {
                pos: [1.0, 2.0, 3.0],
                heading: 0.0,
                hp: 100,
                hp_max: 100,
                hp_verified: true,
                mana: 0,
                mana_max: 0,
                endurance: 0,
                endurance_max: 0,
                endurance_confirmed: true,
                casting: None,
                buffs: vec![],
                zone_name: "qeynos".into(),
                target_id: None,
                target_name: None,
                auto_attack: false,
                sitting: false,
                held: false,
                player_class: "Warrior".into(),
                player_level: 1,
            },
            visible: vec![],
            legal_actions: LegalActionMask {
                gems: [false; 9],
                abilities: vec![],
            },
            dead: false,
            terminated: false,
            truncated: false,
            visibility_available: true,
            tick,
        }
    }

    /// A minimal fake server: accept one connection, handshake Accepted, then echo one
    /// Observation per Step received. Proves AgentClient round-trips against the real wire
    /// framing without needing a live eqoxide instance.
    #[test]
    fn connect_handshakes_and_round_trips_a_step_and_observation() {
        let dir = std::env::temp_dir().join(format!("harness-socket-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let sock_path = dir.join("agent.sock");
        let _ = std::fs::remove_file(&sock_path);
        let listener = UnixListener::bind(&sock_path).unwrap();

        let server = std::thread::spawn(move || {
            let (stream, _) = listener.accept().unwrap();
            let mut writer = stream.try_clone().unwrap();
            let mut reader = BufReader::new(stream);

            let mut line = String::new();
            reader.read_line(&mut line).unwrap();
            let _hello: Hello = decode_line(&line).unwrap();
            writer
                .write_all(encode_line(&HandshakeReply::Accepted).unwrap().as_bytes())
                .unwrap();

            let mut step_line = String::new();
            reader.read_line(&mut step_line).unwrap();
            let _step: Step = decode_line(&step_line).unwrap();
            writer
                .write_all(encode_line(&canned_observation(0)).unwrap().as_bytes())
                .unwrap();
        });

        let mut client = AgentClient::connect(&sock_path).expect("connect + handshake");
        client
            .send_step(&Step {
                movement: Some(AgentMovement {
                    dir: [1.0, 0.0],
                    up: 0.0,
                    jump: false,
                    wish_heading: None,
                }),
                verb: None,
            })
            .expect("send step");
        let obs = client.recv_observation().expect("recv observation");
        assert_eq!(obs.tick, 0);
        assert_eq!(obs.own.zone_name, "qeynos");

        server.join().unwrap();
        let _ = std::fs::remove_dir_all(&dir);
    }
}
