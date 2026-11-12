use crate::{identity, json, Client, Error, Result, Value};
use std::io;
use std::net::{Shutdown, TcpStream, ToSocketAddrs};
use std::time::{Duration, Instant};
use tungstenite::client::IntoClientRequest;
use tungstenite::handshake::HandshakeError;
use tungstenite::http::HeaderValue;
use tungstenite::protocol::WebSocketConfig;
use tungstenite::stream::MaybeTlsStream;
use tungstenite::{client_tls_with_config, Message, WebSocket};

type Socket = WebSocket<MaybeTlsStream<TcpStream>>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Frame {
    Text(String),
    Binary(Vec<u8>),
}

/// One explicit WebSocket connection. Each receive returns one complete text or binary message.
pub struct Feed {
    socket: Option<Socket>,
    timeout: Duration,
}

impl Client {
    pub fn websocket(&self, channel: &str, media_token: Option<&str>) -> Result<Feed> {
        let path = match channel {
            "cloud" | "/ws/cloud" => "/ws/cloud",
            "scan" | "/ws/scan" => "/ws/scan",
            "camera" | "/ws/camera" => "/ws/camera",
            _ => {
                return Err(Error::InvalidInput(
                    "WebSocket channel must be cloud, scan or camera".into(),
                ))
            }
        };
        self.connect_websocket(path, &[], media_token)
    }

    /// Open one operator session. Opening requests input readiness, without submitting velocity.
    pub fn teleop(&self, client_id: &str) -> Result<Teleop> {
        if client_id.is_empty() {
            return Err(Error::InvalidInput("teleop client_id is required".into()));
        }
        let mut teleop = Teleop {
            feed: self.connect_websocket("/ws/teleop", &[("client_id", client_id)], None)?,
        };
        let ready = teleop.request_input(&identity())?;
        if ready["type"] != "input_ack" {
            return Err(Error::Operation(ready));
        }
        Ok(teleop)
    }

    fn connect_websocket(
        &self,
        path: &str,
        query: &[(&str, &str)],
        media_token: Option<&str>,
    ) -> Result<Feed> {
        if self.timeout.is_zero() {
            return Err(Error::InvalidInput(
                "WebSocket timeout must be positive".into(),
            ));
        }
        let mut url = self.base.clone();
        let scheme = if url.scheme() == "https" { "wss" } else { "ws" };
        url.set_scheme(scheme)
            .map_err(|_| Error::InvalidInput("invalid WebSocket scheme".into()))?;
        url.set_path(path);
        if !query.is_empty() {
            url.query_pairs_mut().extend_pairs(query.iter().copied());
        }
        let host = url
            .host_str()
            .ok_or_else(|| Error::InvalidInput("Gateway host is missing".into()))?;
        let port = url
            .port_or_known_default()
            .ok_or_else(|| Error::InvalidInput("Gateway port is missing".into()))?;
        let mut request = url.as_str().into_client_request()?;
        if let Some(key) = &self.api_key {
            request.headers_mut().insert("X-API-Key", header(key)?);
        }
        if let Some(token) = media_token.or(self.media_token.as_deref()) {
            request
                .headers_mut()
                .insert("Authorization", header(&format!("Bearer {token}"))?);
        }

        let mut last_error = io::Error::new(
            io::ErrorKind::NotFound,
            "Gateway host resolved to no addresses",
        );
        let deadline = Instant::now() + self.timeout;
        let mut connected = None;
        for address in (host.trim_matches(['[', ']']), port).to_socket_addrs()? {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(Error::Timeout("WebSocket connection timed out".into()));
            }
            match TcpStream::connect_timeout(&address, remaining) {
                Ok(stream) => {
                    connected = Some(stream);
                    break;
                }
                Err(error) => last_error = error,
            }
        }
        let stream = connected.ok_or(Error::Io(last_error))?;
        stream.set_read_timeout(Some(self.timeout))?;
        stream.set_write_timeout(Some(self.timeout))?;
        let config = WebSocketConfig::default()
            .max_message_size(None)
            .max_frame_size(None);
        let (socket, _) = client_tls_with_config(request, stream, Some(config), None).map_err(
            |error| match error {
                HandshakeError::Failure(error) => Error::from(error),
                HandshakeError::Interrupted(_) => {
                    Error::Timeout("WebSocket handshake timed out".into())
                }
            },
        )?;
        Ok(Feed {
            socket: Some(socket),
            timeout: self.timeout,
        })
    }
}

fn header(value: &str) -> Result<HeaderValue> {
    HeaderValue::from_str(value)
        .map_err(|error| Error::InvalidInput(format!("invalid WebSocket header: {error}")))
}

fn tcp_stream(socket: &Socket) -> Result<&TcpStream> {
    match socket.get_ref() {
        MaybeTlsStream::Plain(stream) => Ok(stream),
        MaybeTlsStream::Rustls(stream) => Ok(&stream.sock),
        _ => Err(Error::Unsupported(
            "unsupported WebSocket TLS transport".into(),
        )),
    }
}

impl Feed {
    fn abort(&mut self) {
        if let Some(socket) = self.socket.take() {
            if let Ok(stream) = tcp_stream(&socket) {
                let _ = stream.shutdown(Shutdown::Both);
            }
        }
    }

    pub fn receive(&mut self) -> Result<Option<Frame>> {
        self.receive_until(Instant::now() + self.timeout)
    }

    fn receive_until(&mut self, deadline: Instant) -> Result<Option<Frame>> {
        loop {
            let Some(socket) = self.socket.as_mut() else {
                return Ok(None);
            };
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                return Err(Error::Timeout("WebSocket receive timed out".into()));
            }
            tcp_stream(socket)?.set_read_timeout(Some(remaining))?;
            match socket.read() {
                Ok(Message::Text(value)) => return Ok(Some(Frame::Text(value.to_string()))),
                Ok(Message::Binary(value)) => return Ok(Some(Frame::Binary(value.to_vec()))),
                Ok(Message::Close(_)) | Err(tungstenite::Error::ConnectionClosed) => {
                    let _ = socket.flush();
                    if let Ok(stream) = tcp_stream(socket) {
                        let _ = stream.shutdown(Shutdown::Both);
                    }
                    self.socket = None;
                    return Ok(None);
                }
                Ok(Message::Ping(_)) => socket.flush()?,
                Ok(Message::Pong(_) | Message::Frame(_)) => {}
                Err(error) => return Err(error.into()),
            }
        }
    }

    pub fn send_text(&mut self, text: &str) -> Result<()> {
        let socket = self
            .socket
            .as_mut()
            .ok_or_else(|| Error::from(tungstenite::Error::ConnectionClosed))?;
        socket.send(Message::Text(text.into()))?;
        Ok(())
    }

    /// Send Close and release the TCP connection. This supplies no physical-stop confirmation.
    pub fn close(&mut self) -> Result<()> {
        let Some(mut socket) = self.socket.take() else {
            return Ok(());
        };
        let result = socket.close(None);
        if let Ok(stream) = tcp_stream(&socket) {
            let _ = stream.shutdown(Shutdown::Both);
        }
        match result {
            Ok(())
            | Err(tungstenite::Error::ConnectionClosed | tungstenite::Error::AlreadyClosed) => {
                Ok(())
            }
            Err(error) => Err(error.into()),
        }
    }
}

impl Drop for Feed {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

/// One operator session with one fresh input window requested per explicit motion sample.
pub struct Teleop {
    feed: Feed,
}

impl Teleop {
    fn request_input(&mut self, request_id: &str) -> Result<Value> {
        let sent = self
            .feed
            .send_text(&json!({"type":"input_request","request_id":request_id}).to_string());
        let receipt = self.receipt(request_id, &["input_ack", "control_rejected"]);
        let result = match sent {
            Ok(()) => receipt,
            Err(error) => match receipt {
                Ok(payload) if payload["type"] == "control_rejected" => Ok(payload),
                _ => Err(error),
            },
        };
        if result.is_err() {
            self.feed.abort();
        }
        result
    }

    fn receipt(&mut self, request_id: &str, types: &[&str]) -> Result<Value> {
        let deadline = Instant::now() + self.feed.timeout;
        loop {
            let frame = self
                .feed
                .receive_until(deadline)?
                .ok_or_else(|| Error::from(tungstenite::Error::ConnectionClosed))?;
            let Frame::Text(text) = frame else {
                return Err(Error::InvalidInput("teleop reply must be JSON text".into()));
            };
            let payload: Value = serde_json::from_str(&text)?;
            let kind = payload["type"].as_str().unwrap_or("");
            if kind == "control_rejected" && payload["request_id"].is_null() {
                return Ok(payload);
            }
            if payload["request_id"] == request_id && types.contains(&kind) {
                return Ok(payload);
            }
        }
    }

    pub fn move_once(
        &mut self,
        vx_mps: f64,
        vy_mps: f64,
        yaw_rps: f64,
        manual_mode: bool,
        request_id: Option<&str>,
    ) -> Result<Value> {
        if ![vx_mps, vy_mps, yaw_rps]
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(Error::InvalidInput("teleop velocity must be finite".into()));
        }
        let request_id = request_id
            .filter(|value| !value.is_empty())
            .map(str::to_owned)
            .unwrap_or_else(identity);
        let ready = self.request_input(&request_id)?;
        if ready["type"] == "control_rejected" {
            return Ok(ready);
        }
        let Some(window) = ready["input_window"]
            .as_str()
            .filter(|value| !value.is_empty())
        else {
            self.feed.abort();
            return Err(Error::Operation(ready));
        };
        let sent = self.feed.send_text(
            &json!({
                "type":"velocity", "request_id":request_id, "input_window":window,
                "deadman":true, "manual_mode":manual_mode,
                "vx_mps":vx_mps, "vy_mps":vy_mps, "yaw_rps":yaw_rps
            })
            .to_string(),
        );
        if let Err(error) = sent {
            // A failed write may leave a buffered velocity frame. Never flush it on a later call.
            self.feed.abort();
            return Err(error);
        }
        let receipt = self.receipt(&request_id, &["ingress_ack", "control_rejected"]);
        if receipt.is_err() {
            self.feed.abort();
        }
        receipt
    }

    pub fn hold(&mut self, request_id: Option<&str>) -> Result<Value> {
        let request_id = request_id
            .filter(|value| !value.is_empty())
            .map(str::to_owned)
            .unwrap_or_else(identity);
        let sent = self.feed.send_text(
            &json!({
                "type":"velocity", "request_id":request_id, "deadman":false,
                "vx_mps":0, "vy_mps":0, "yaw_rps":0
            })
            .to_string(),
        );
        if let Err(error) = sent {
            self.feed.abort();
            return Err(error);
        }
        let receipt = self.receipt(&request_id, &["control_ack", "control_rejected"]);
        if receipt.is_err() {
            self.feed.abort();
        }
        receipt
    }

    pub fn close(&mut self) -> Result<()> {
        self.feed.close()
    }
}
