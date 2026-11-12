//! External blocking client for a LingTu Gateway, normally on port 5050.
//! Command receipts retain the owner's evidence. Acceptance alone does not prove execution.

mod domains;
mod events;
pub mod ffi;
mod websocket;

pub use domains::{Map, Navigation, Observations, System, Tasks};
pub use events::{Event, Events};
pub use serde_json::{json, Value};
pub use websocket::{Feed, Frame, Teleop};

use reqwest::blocking::{Client as HttpClient, Response};
use reqwest::{Method, Url};
use std::fmt;
use std::fs::{self, OpenOptions};
use std::io;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug)]
pub enum Error {
    InvalidInput(String),
    Transport(reqwest::Error),
    Http { status: u16, body: String },
    Json(serde_json::Error),
    Io(io::Error),
    Operation(Value),
    Timeout(String),
    Unsupported(String),
    WebSocket(Box<tungstenite::Error>),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidInput(message) => write!(f, "{message}"),
            Self::Transport(error) => write!(f, "Gateway transport failed: {error}"),
            Self::Http { status, body } => write!(f, "Gateway HTTP {status}: {body}"),
            Self::Json(error) => write!(f, "Invalid Gateway JSON: {error}"),
            Self::Io(error) => write!(f, "{error}"),
            Self::Operation(body) => write!(f, "Gateway operation failed: {body}"),
            Self::Timeout(message) => write!(f, "{message}"),
            Self::Unsupported(message) => write!(f, "{message}"),
            Self::WebSocket(error) => write!(f, "Gateway WebSocket failed: {error}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Transport(error) => Some(error),
            Self::Json(error) => Some(error),
            Self::Io(error) => Some(error),
            Self::WebSocket(error) => Some(error.as_ref()),
            _ => None,
        }
    }
}

impl From<reqwest::Error> for Error {
    fn from(error: reqwest::Error) -> Self {
        Self::Transport(error)
    }
}
impl From<io::Error> for Error {
    fn from(error: io::Error) -> Self {
        Self::Io(error)
    }
}
impl From<serde_json::Error> for Error {
    fn from(error: serde_json::Error) -> Self {
        Self::Json(error)
    }
}

impl From<tungstenite::Error> for Error {
    fn from(error: tungstenite::Error) -> Self {
        match error {
            tungstenite::Error::Http(response) => Self::Http {
                status: response.status().as_u16(),
                body: String::from_utf8_lossy(response.body().as_deref().unwrap_or_default())
                    .into_owned(),
            },
            tungstenite::Error::Io(error) => Self::Io(error),
            error => Self::WebSocket(Box::new(error)),
        }
    }
}

#[derive(Clone)]
pub struct Client {
    base: Url,
    http: HttpClient,
    api_key: Option<String>,
    media_token: Option<String>,
    timeout: Duration,
}

impl Client {
    pub fn new(base_url: &str, api_key: Option<&str>, timeout: Duration) -> Result<Self> {
        let base = Url::parse(base_url).map_err(|e| Error::InvalidInput(e.to_string()))?;
        if !matches!(base.scheme(), "http" | "https")
            || base.host_str().is_none()
            || base.query().is_some()
            || base.fragment().is_some()
        {
            return Err(Error::InvalidInput(
                "base URL must be an HTTP(S) endpoint without query or fragment".into(),
            ));
        }
        Ok(Self {
            base,
            http: HttpClient::builder()
                .timeout(timeout)
                .retry(reqwest::retry::never())
                .redirect(reqwest::redirect::Policy::none())
                .build()?,
            api_key: api_key.map(str::to_owned),
            media_token: None,
            timeout,
        })
    }

    /// Supply the Gateway's independent media-viewer credential for camera HTTP and WebSockets.
    pub fn with_media_token(mut self, token: &str) -> Self {
        self.media_token = Some(token.to_owned());
        self
    }

    pub fn navigation(&self) -> Navigation<'_> {
        Navigation(self)
    }
    pub fn map(&self) -> Map<'_> {
        Map(self)
    }
    pub fn tasks(&self) -> Tasks<'_> {
        Tasks(self)
    }
    pub fn observations(&self) -> Observations<'_> {
        Observations(self)
    }
    pub fn system(&self) -> System<'_> {
        System(self)
    }

    /// Send one request. Query pairs are URL encoded. Body and response retain all owner fields.
    pub fn request(
        &self,
        method: &str,
        path: &str,
        query: &[(&str, &str)],
        body: Option<&Value>,
    ) -> Result<Value> {
        let response = self.send(method, path, query, body, &[])?;
        let bytes = response.bytes()?;
        if bytes.is_empty() {
            return Ok(Value::Null);
        }
        Ok(serde_json::from_slice(&bytes)?)
    }

    pub fn bytes(&self, path: &str, query: &[(&str, &str)]) -> Result<Vec<u8>> {
        Ok(self.send("GET", path, query, None, &[])?.bytes()?.to_vec())
    }

    /// Stream into a unique sibling file and replace the target only after a complete transfer.
    pub fn download(&self, path: &str, target: impl AsRef<Path>) -> Result<PathBuf> {
        let mut response = self.send("GET", path, &[], None, &[])?;
        let expected = response.content_length();
        let target = target.as_ref();
        let name = target
            .file_name()
            .ok_or_else(|| Error::InvalidInput("download target needs a filename".into()))?;
        let mut temporary_name = name.to_os_string();
        temporary_name.push(format!(".{}.part", identity()));
        let temporary = target.with_file_name(temporary_name);
        let mut output = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&temporary)?;
        let result = (|| {
            let received = io::copy(&mut response, &mut output)?;
            if expected.is_some_and(|length| length != received) {
                return Err(Error::Io(io::Error::new(
                    io::ErrorKind::UnexpectedEof,
                    "incomplete download",
                )));
            }
            output.sync_all()?;
            drop(output);
            fs::rename(&temporary, target)?;
            Ok(target.to_path_buf())
        })();
        if result.is_err() {
            let _ = fs::remove_file(&temporary);
        }
        result
    }

    /// Open SSE with an idle read timeout. Drop the iterator to close its response.
    pub fn events(&self, query: &[(&str, &str)]) -> Result<Events> {
        self.events_after(query, None)
    }

    /// Explicitly reconnect using the last received SSE ID. Delivery policy belongs to the caller.
    pub fn events_after(
        &self,
        query: &[(&str, &str)],
        last_event_id: Option<&str>,
    ) -> Result<Events> {
        let mut headers = vec![("Accept", "text/event-stream")];
        if let Some(id) = last_event_id {
            headers.push(("Last-Event-ID", id));
        }
        Ok(Events::new(self.send(
            "GET",
            "/api/v1/events",
            query,
            None,
            &headers,
        )?))
    }

    fn send(
        &self,
        method: &str,
        path: &str,
        query: &[(&str, &str)],
        body: Option<&Value>,
        headers: &[(&str, &str)],
    ) -> Result<Response> {
        if !path.starts_with('/') || path.starts_with("//") || path.contains('#') {
            return Err(Error::InvalidInput(
                "request path must be an absolute Gateway path".into(),
            ));
        }
        let mut url = self.base.clone();
        let (path, embedded_query) = path
            .split_once('?')
            .map_or((path, None), |(p, q)| (p, Some(q)));
        url.set_path(path);
        url.set_query(embedded_query);
        if !query.is_empty() {
            url.query_pairs_mut().extend_pairs(query.iter().copied());
        }
        let method = Method::from_bytes(method.as_bytes())
            .map_err(|e| Error::InvalidInput(e.to_string()))?;
        let mut request = self.http.request(method, url);
        if let Some(key) = &self.api_key {
            request = request.header("X-API-Key", key);
        }
        if let Some(token) = &self.media_token {
            request = request.bearer_auth(token);
        }
        for (name, value) in headers {
            request = request.header(*name, *value);
        }
        if let Some(body) = body {
            request = request.json(body);
        }
        let response = request.send()?;
        if !response.status().is_success() {
            return Err(Error::Http {
                status: response.status().as_u16(),
                body: response.text()?,
            });
        }
        Ok(response)
    }

    fn get(&self, path: &str) -> Result<Value> {
        self.request("GET", path, &[], None)
    }

    fn command(&self, method: &str, path: &str, options: Value) -> Result<Value> {
        let mut body = object(options)?;
        body.entry("request_id")
            .or_insert_with(|| json!(identity()));
        let request_id = body["request_id"].clone();
        let mut response = self.request(method, path, &[], Some(&Value::Object(body)))?;
        if let Some(object) = response.as_object_mut() {
            object.entry("request_id").or_insert(request_id);
        }
        Ok(response)
    }
}

fn object(value: Value) -> Result<serde_json::Map<String, Value>> {
    match value {
        Value::Null => Ok(serde_json::Map::new()),
        Value::Object(object) => Ok(object),
        _ => Err(Error::InvalidInput(
            "options must be a JSON object or null".into(),
        )),
    }
}

fn fields(options: Value, fields: Value) -> Result<Value> {
    let mut result = object(options)?;
    result.extend(object(fields)?);
    Ok(Value::Object(result))
}

fn segment(value: &str) -> String {
    let mut encoded = String::new();
    for byte in value.bytes() {
        match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'~' => {
                encoded.push(byte as char)
            }
            _ => {
                use std::fmt::Write;
                write!(encoded, "%{byte:02X}").unwrap();
            }
        }
    }
    encoded
}

fn identity() -> String {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    format!(
        "sdk-{}-{nanos}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    )
}
