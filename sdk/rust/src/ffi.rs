//! C ABI used by the C++ SDK. Reply buffers are length-delimited and owned by Rust.

use crate::{fields, json, object, Client, Error, Feed, Frame, Result, Teleop, Value};
use std::ffi::{c_char, c_void, CStr};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::time::Duration;

#[repr(C)]
pub struct Reply {
    /// HTTP status for HTTP failures, otherwise zero.
    pub status: i32,
    /// Zero on success, one on failure.
    pub error: i32,
    pub data: *mut u8,
    pub len: usize,
}

#[repr(C)]
pub struct EventView {
    pub event: *const u8,
    pub event_len: usize,
    /// Null when the stream has supplied no event ID.
    pub id: *const u8,
    pub id_len: usize,
    pub data: *const u8,
    pub data_len: usize,
}

pub type EventCallback = unsafe extern "C" fn(*const EventView, *mut c_void) -> i32;

impl Reply {
    fn buffer(status: i32, error: i32, bytes: Vec<u8>) -> Self {
        let len = bytes.len();
        let data = if bytes.is_empty() {
            ptr::null_mut()
        } else {
            Box::into_raw(bytes.into_boxed_slice()).cast::<u8>()
        };
        Self {
            status,
            error,
            data,
            len,
        }
    }

    fn failure(error: Error) -> Self {
        if let Error::Http { status, body } = error {
            return Self::buffer(i32::from(status), 1, body.into_bytes());
        }
        let kind = match &error {
            Error::InvalidInput(_) => "invalid_input",
            Error::Transport(_) => "transport",
            Error::Json(_) => "json",
            Error::Io(_) => "io",
            Error::Operation(_) => "operation",
            Error::Timeout(_) => "timeout",
            Error::Unsupported(_) => "unsupported",
            Error::WebSocket(_) => "websocket",
            Error::Http { .. } => unreachable!(),
        };
        let body = match &error {
            Error::Operation(body) => body.clone(),
            _ => Value::Null,
        };
        Self::buffer(
            0,
            1,
            json!({"kind":kind,"message":error.to_string(),"body":body})
                .to_string()
                .into_bytes(),
        )
    }
}

fn boundary(operation: impl FnOnce() -> Result<Vec<u8>>) -> Reply {
    match catch_unwind(AssertUnwindSafe(operation)) {
        Ok(Ok(bytes)) => Reply::buffer(0, 0, bytes),
        Ok(Err(error)) => Reply::failure(error),
        Err(panic) => {
            let message = panic
                .downcast_ref::<&str>()
                .copied()
                .or_else(|| panic.downcast_ref::<String>().map(String::as_str))
                .unwrap_or("Rust SDK panicked");
            Reply::buffer(
                0,
                1,
                json!({"kind":"panic","message":message,"body":null})
                    .to_string()
                    .into_bytes(),
            )
        }
    }
}

unsafe fn text<'a>(value: *const c_char) -> Result<&'a str> {
    if value.is_null() {
        return Err(Error::InvalidInput("required string is null".into()));
    }
    CStr::from_ptr(value)
        .to_str()
        .map_err(|error| Error::InvalidInput(format!("string must be UTF-8: {error}")))
}

unsafe fn optional_text<'a>(value: *const c_char) -> Result<Option<&'a str>> {
    if value.is_null() {
        Ok(None)
    } else {
        text(value).map(Some)
    }
}

unsafe fn value(value: *const c_char) -> Result<Value> {
    match optional_text(value)? {
        None => Ok(Value::Null),
        Some(value) => serde_json::from_str(value)
            .map_err(|error| Error::InvalidInput(format!("invalid JSON argument: {error}"))),
    }
}

unsafe fn client<'a>(client: *const Client) -> Result<&'a Client> {
    client
        .as_ref()
        .ok_or_else(|| Error::InvalidInput("client is null".into()))
}

fn encoded(value: Value) -> Vec<u8> {
    value.to_string().into_bytes()
}

/// Create a client and initialize the output pointer to null on failure.
///
/// # Safety
/// Strings must be valid NUL-terminated buffers. `out_client` must point to writable storage.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_client_create(
    base: *const c_char,
    key: *const c_char,
    timeout_ms: u64,
    out_client: *mut *mut Client,
) -> Reply {
    lingtu_sdk_client_create_with_media_token(base, key, timeout_ms, ptr::null(), out_client)
}

/// Create a client with the independent media-viewer credential.
///
/// # Safety
/// Strings must be valid NUL-terminated buffers. `out_client` must point to writable storage.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_client_create_with_media_token(
    base: *const c_char,
    key: *const c_char,
    timeout_ms: u64,
    media_token: *const c_char,
    out_client: *mut *mut Client,
) -> Reply {
    boundary(|| {
        if out_client.is_null() {
            return Err(Error::InvalidInput("out_client is null".into()));
        }
        *out_client = ptr::null_mut();
        let mut created = Client::new(
            text(base)?,
            optional_text(key)?,
            Duration::from_millis(timeout_ms),
        )?;
        if let Some(token) = optional_text(media_token)? {
            created = created.with_media_token(token);
        }
        *out_client = Box::into_raw(Box::new(created));
        Ok(Vec::new())
    })
}

/// Release a client. Null is accepted.
///
/// # Safety
/// A non-null pointer must be a live client from `lingtu_sdk_client_create`, with no active calls.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_client_free(client: *mut Client) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if !client.is_null() {
            drop(Box::from_raw(client));
        }
    }));
}

/// Release one reply's data buffer. Null is accepted.
///
/// # Safety
/// Pass the original pointer and length from a reply exactly once.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_buffer_free(data: *mut u8, len: usize) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if !data.is_null() {
            drop(Box::from_raw(ptr::slice_from_raw_parts_mut(data, len)));
        }
    }));
}

/// Send one HTTP JSON request. A null body omits the request body.
///
/// # Safety
/// The client must remain live during the call. Strings must be valid NUL-terminated buffers.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_request(
    handle: *const Client,
    method: *const c_char,
    path: *const c_char,
    body: *const c_char,
) -> Reply {
    boundary(|| {
        let body = if body.is_null() {
            None
        } else {
            Some(value(body)?)
        };
        Ok(encoded(client(handle)?.request(
            text(method)?,
            text(path)?,
            &[],
            body.as_ref(),
        )?))
    })
}

/// Submit one command, supplying a request ID when absent.
///
/// # Safety
/// The client must remain live during the call. Strings must be valid NUL-terminated buffers.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_command(
    handle: *const Client,
    method: *const c_char,
    path: *const c_char,
    body: *const c_char,
) -> Reply {
    boundary(|| {
        Ok(encoded(client(handle)?.command(
            text(method)?,
            text(path)?,
            value(body)?,
        )?))
    })
}

/// Download response bytes without JSON decoding.
///
/// # Safety
/// The client must remain live during the call. `path` must be a valid NUL-terminated buffer.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_bytes(handle: *const Client, path: *const c_char) -> Reply {
    boundary(|| client(handle)?.bytes(text(path)?, &[]))
}

/// Read the current map-frame position through the existing state decoder.
///
/// # Safety
/// The client must remain live during the call.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_position(handle: *const Client) -> Reply {
    boundary(|| Ok(encoded(client(handle)?.navigation().position()?)))
}

/// Download a file safely and return its local path as a JSON string.
///
/// # Safety
/// The client must remain live during the call. Strings must be valid NUL-terminated UTF-8 buffers.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_download(
    handle: *const Client,
    path: *const c_char,
    target: *const c_char,
) -> Reply {
    boundary(|| {
        let target = client(handle)?.download(text(path)?, text(target)?)?;
        Ok(encoded(json!(target.to_string_lossy())))
    })
}

/// Merge JSON option objects, with required fields taking precedence.
///
/// # Safety
/// Non-null strings must be valid NUL-terminated buffers containing UTF-8 JSON.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_merge_json(
    options: *const c_char,
    required: *const c_char,
) -> Reply {
    boundary(|| Ok(encoded(fields(value(options)?, value(required)?)?)))
}

/// Submit a coordinate goal through the existing Gateway navigation owner.
///
/// # Safety
/// The client must remain live during the call. Non-null options must be a valid JSON C string.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_goal(
    handle: *const Client,
    x: f64,
    y: f64,
    z: f64,
    options: *const c_char,
    click: i32,
) -> Reply {
    boundary(|| {
        let navigation = client(handle)?.navigation();
        let result = if click != 0 {
            navigation.go_to_click(x, y, z, value(options)?)
        } else {
            navigation.go(x, y, z, value(options)?)
        };
        Ok(encoded(result?))
    })
}

/// Wait for the identified navigation task without resubmitting a goal.
///
/// # Safety
/// The client must remain live during the call. `task` must be a valid NUL-terminated UTF-8 string.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_wait_navigation(
    handle: *const Client,
    task: *const c_char,
    timeout_ms: u64,
    poll_ms: u64,
) -> Reply {
    boundary(|| {
        Ok(encoded(client(handle)?.navigation().wait_until_arrived(
            text(task)?,
            Duration::from_millis(timeout_ms),
            Duration::from_millis(poll_ms),
        )?))
    })
}

/// Submit one map save and poll its durable operation.
///
/// # Safety
/// The client must remain live during the call. Non-null options must be a valid JSON C string.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_save_and_wait(
    handle: *const Client,
    options: *const c_char,
    timeout_ms: u64,
    poll_ms: u64,
    wait_source: i32,
) -> Reply {
    boundary(|| {
        Ok(encoded(client(handle)?.map().save_and_wait(
            value(options)?,
            Duration::from_millis(timeout_ms),
            Duration::from_millis(poll_ms),
            wait_source != 0,
        )?))
    })
}

/// Deliver SSE events synchronously until EOF, error, or callback cancellation.
///
/// # Safety
/// The client, C strings and callback context must remain valid during the call. The callback must
/// not unwind or free the client. Event pointers are borrowed only for that callback invocation.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_events(
    handle: *const Client,
    query_json: *const c_char,
    last_id: *const c_char,
    callback: Option<EventCallback>,
    user_data: *mut c_void,
) -> Reply {
    boundary(|| {
        let callback =
            callback.ok_or_else(|| Error::InvalidInput("event callback is null".into()))?;
        let query = object(value(query_json)?)?;
        let query: Vec<_> = query
            .iter()
            .map(|(name, value)| {
                value
                    .as_str()
                    .map(|value| (name.as_str(), value))
                    .ok_or_else(|| Error::InvalidInput("event query values must be strings".into()))
            })
            .collect::<Result<_>>()?;
        let events = client(handle)?.events_after(&query, optional_text(last_id)?)?;
        for event in events {
            let event = event?;
            let id = event.id.as_deref();
            let view = EventView {
                event: event.event.as_ptr(),
                event_len: event.event.len(),
                id: id.map_or(ptr::null(), str::as_ptr),
                id_len: id.map_or(0, str::len),
                data: event.data.as_ptr(),
                data_len: event.data.len(),
            };
            if callback(&view, user_data) == 0 {
                break;
            }
        }
        Ok(Vec::new())
    })
}

/// Open one cloud, scan or camera WebSocket. A null media token uses the client's credential.
///
/// # Safety
/// The client and C strings must be live during the call. `out_feed` must be writable storage.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_websocket_open(
    handle: *const Client,
    channel: *const c_char,
    media_token: *const c_char,
    out_feed: *mut *mut Feed,
) -> Reply {
    boundary(|| {
        if out_feed.is_null() {
            return Err(Error::InvalidInput("out_feed is null".into()));
        }
        *out_feed = ptr::null_mut();
        let feed = client(handle)?.websocket(text(channel)?, optional_text(media_token)?)?;
        *out_feed = Box::into_raw(Box::new(feed));
        Ok(Vec::new())
    })
}

/// Receive a complete message. Kind is zero for clean close, one for text and two for binary.
///
/// # Safety
/// The feed must be live and exclusively borrowed. `out_kind` must point to writable storage.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_websocket_receive(
    handle: *mut Feed,
    out_kind: *mut i32,
) -> Reply {
    boundary(|| {
        if out_kind.is_null() {
            return Err(Error::InvalidInput("out_kind is null".into()));
        }
        *out_kind = 0;
        let feed = handle
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("feed is null".into()))?;
        match feed.receive()? {
            None => Ok(Vec::new()),
            Some(Frame::Text(text)) => {
                *out_kind = 1;
                Ok(text.into_bytes())
            }
            Some(Frame::Binary(bytes)) => {
                *out_kind = 2;
                Ok(bytes)
            }
        }
    })
}

/// Send one UTF-8 text message.
///
/// # Safety
/// The feed must be live and exclusively borrowed. `message` must be a valid UTF-8 C string.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_websocket_send_text(
    handle: *mut Feed,
    message: *const c_char,
) -> Reply {
    boundary(|| {
        let feed = handle
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("feed is null".into()))?;
        feed.send_text(text(message)?)?;
        Ok(Vec::new())
    })
}

/// Close a feed and release its socket. The handle remains valid until freed.
///
/// # Safety
/// The feed must be live and exclusively borrowed during this call.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_websocket_close(handle: *mut Feed) -> Reply {
    boundary(|| {
        handle
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("feed is null".into()))?
            .close()?;
        Ok(Vec::new())
    })
}

/// Close and free a feed. Null is accepted.
///
/// # Safety
/// The pointer must be a live feed from `lingtu_sdk_websocket_open`, with no active calls.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_websocket_free(handle: *mut Feed) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if !handle.is_null() {
            drop(Box::from_raw(handle));
        }
    }));
}

/// Explicitly open one operator session, without submitting a velocity sample.
///
/// # Safety
/// The client and C string must be live during the call. `out_teleop` must be writable storage.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_teleop_open(
    handle: *const Client,
    client_id: *const c_char,
    out_teleop: *mut *mut Teleop,
) -> Reply {
    boundary(|| {
        if out_teleop.is_null() {
            return Err(Error::InvalidInput("out_teleop is null".into()));
        }
        *out_teleop = ptr::null_mut();
        let teleop = client(handle)?.teleop(text(client_id)?)?;
        *out_teleop = Box::into_raw(Box::new(teleop));
        Ok(Vec::new())
    })
}

/// Submit one deadman-held sample using a freshly requested one-use input window.
///
/// # Safety
/// The teleop must be live and exclusively borrowed. A non-null request ID must be a valid C string.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_teleop_move(
    handle: *mut Teleop,
    vx_mps: f64,
    vy_mps: f64,
    yaw_rps: f64,
    manual_mode: i32,
    request_id: *const c_char,
) -> Reply {
    boundary(|| {
        let teleop = handle
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("teleop is null".into()))?;
        Ok(encoded(teleop.move_once(
            vx_mps,
            vy_mps,
            yaw_rps,
            manual_mode != 0,
            optional_text(request_id)?,
        )?))
    })
}

/// Release the deadman and return the owner's hold receipt.
///
/// # Safety
/// The teleop must be live and exclusively borrowed. A non-null request ID must be a valid C string.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_teleop_hold(
    handle: *mut Teleop,
    request_id: *const c_char,
) -> Reply {
    boundary(|| {
        let teleop = handle
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("teleop is null".into()))?;
        Ok(encoded(teleop.hold(optional_text(request_id)?)?))
    })
}

/// Close the operator socket. This supplies no physical-stop confirmation.
///
/// # Safety
/// The teleop must be live and exclusively borrowed during this call.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_teleop_close(handle: *mut Teleop) -> Reply {
    boundary(|| {
        handle
            .as_mut()
            .ok_or_else(|| Error::InvalidInput("teleop is null".into()))?
            .close()?;
        Ok(Vec::new())
    })
}

/// Close and free an operator session. Null is accepted.
///
/// # Safety
/// The pointer must be a live teleop from `lingtu_sdk_teleop_open`, with no active calls.
#[no_mangle]
pub unsafe extern "C" fn lingtu_sdk_teleop_free(handle: *mut Teleop) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if !handle.is_null() {
            drop(Box::from_raw(handle));
        }
    }));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Event;
    use std::ffi::CString;
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::slice;
    use std::thread::{self, JoinHandle};

    unsafe fn take(reply: Reply) -> (i32, i32, Vec<u8>) {
        let bytes = if reply.data.is_null() {
            Vec::new()
        } else {
            slice::from_raw_parts(reply.data, reply.len).to_vec()
        };
        lingtu_sdk_buffer_free(reply.data, reply.len);
        (reply.status, reply.error, bytes)
    }

    unsafe fn create(base: &str) -> *mut Client {
        let mut handle = ptr::null_mut();
        let base = CString::new(base).unwrap();
        let key = CString::new("ffi-key").unwrap();
        let (status, error, _) = take(lingtu_sdk_client_create(
            base.as_ptr(),
            key.as_ptr(),
            2000,
            &mut handle,
        ));
        assert_eq!((status, error), (0, 0));
        assert!(!handle.is_null());
        handle
    }

    fn server(status: &str, body: &[u8]) -> (String, JoinHandle<String>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let base = format!("http://{}", listener.local_addr().unwrap());
        let status = status.to_owned();
        let body = body.to_vec();
        let thread = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            stream
                .set_read_timeout(Some(Duration::from_secs(3)))
                .unwrap();
            let mut request = Vec::new();
            let mut byte = [0];
            while !request.ends_with(b"\r\n\r\n") {
                stream.read_exact(&mut byte).unwrap();
                request.push(byte[0]);
            }
            let headers = String::from_utf8(request.clone()).unwrap();
            let len = headers
                .lines()
                .find_map(|line| {
                    let (name, value) = line.split_once(':')?;
                    name.eq_ignore_ascii_case("content-length")
                        .then(|| value.trim().parse::<usize>().unwrap())
                })
                .unwrap_or(0);
            let mut payload = vec![0; len];
            stream.read_exact(&mut payload).unwrap();
            request.extend(payload);
            write!(
                stream,
                "HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                body.len()
            )
            .unwrap();
            stream.write_all(&body).unwrap();
            String::from_utf8(request).unwrap()
        });
        (base, thread)
    }

    #[test]
    fn merge_allocates_owned_length_delimited_json() {
        unsafe {
            let options =
                CString::new(r#"{"request_id":"original","x":99,"text":"a\u0000b"}"#).unwrap();
            let required = CString::new(r#"{"x":1,"y":2}"#).unwrap();
            let (status, error, bytes) =
                take(lingtu_sdk_merge_json(options.as_ptr(), required.as_ptr()));
            assert_eq!((status, error), (0, 0));
            let merged: Value = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(
                merged,
                json!({"request_id":"original","x":1,"y":2,"text":"a\0b"})
            );
            lingtu_sdk_buffer_free(ptr::null_mut(), 0);
        }
    }

    #[test]
    fn boundary_reports_panics_and_invalid_inputs() {
        unsafe {
            let (_, error, bytes) = take(boundary(|| panic!("caught ffi panic")));
            assert_eq!(error, 1);
            assert_eq!(
                serde_json::from_slice::<Value>(&bytes).unwrap()["kind"],
                "panic"
            );

            let invalid_utf8 = [255_u8, 0];
            let mut handle = ptr::dangling_mut();
            let (_, error, bytes) = take(lingtu_sdk_client_create(
                invalid_utf8.as_ptr().cast(),
                ptr::null(),
                100,
                &mut handle,
            ));
            assert_eq!(error, 1);
            assert!(handle.is_null());
            assert_eq!(
                serde_json::from_slice::<Value>(&bytes).unwrap()["kind"],
                "invalid_input"
            );

            let path = CString::new("/api/v1/health").unwrap();
            let (_, error, bytes) = take(lingtu_sdk_bytes(ptr::null(), path.as_ptr()));
            assert_eq!(error, 1);
            assert_eq!(
                serde_json::from_slice::<Value>(&bytes).unwrap()["kind"],
                "invalid_input"
            );
            lingtu_sdk_client_free(ptr::null_mut());
        }
    }

    #[test]
    fn request_and_command_keep_http_details_and_identity() {
        unsafe {
            let (base, server) = server("409 Conflict", br#"{"detail":"epoch mismatch"}"#);
            let handle = create(&base);
            let method = CString::new("GET").unwrap();
            let path = CString::new("/api/v1/health").unwrap();
            let (status, error, bytes) = take(lingtu_sdk_request(
                handle,
                method.as_ptr(),
                path.as_ptr(),
                ptr::null(),
            ));
            assert_eq!((status, error), (409, 1));
            assert_eq!(bytes, br#"{"detail":"epoch mismatch"}"#);
            lingtu_sdk_client_free(handle);
            assert!(server
                .join()
                .unwrap()
                .to_lowercase()
                .contains("x-api-key: ffi-key"));

            let (base, server) = self::server(
                "202 Accepted",
                br#"{"accepted":true,"task_id":"stable-task"}"#,
            );
            let handle = create(&base);
            let method = CString::new("POST").unwrap();
            let path = CString::new("/api/v1/instruction").unwrap();
            let body = CString::new(r#"{"text":"inspect","request_id":"stable-request"}"#).unwrap();
            let (_, error, bytes) = take(lingtu_sdk_command(
                handle,
                method.as_ptr(),
                path.as_ptr(),
                body.as_ptr(),
            ));
            assert_eq!(error, 0);
            let response: Value = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(response["request_id"], "stable-request");
            assert_eq!(response["task_id"], "stable-task");
            lingtu_sdk_client_free(handle);
            let request = server.join().unwrap();
            let body: Value =
                serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
            assert_eq!(body["request_id"], "stable-request");
            assert!(body.get("client_id").is_none());
        }
    }

    #[test]
    fn binary_reply_keeps_embedded_nul() {
        unsafe {
            let (base, server) = server("200 OK", b"\xff\0jpeg\0");
            let handle = create(&base);
            let path = CString::new("/api/v1/camera/snapshot").unwrap();
            let (status, error, bytes) = take(lingtu_sdk_bytes(handle, path.as_ptr()));
            assert_eq!((status, error), (0, 0));
            assert_eq!(bytes, b"\xff\0jpeg\0");
            lingtu_sdk_client_free(handle);
            server.join().unwrap();
        }
    }

    #[test]
    fn position_reuses_map_frame_state_evidence() {
        unsafe {
            let body = json!({"localization":{"odometry":{"frame_id":"map","x":1.0,"y":2.0,"z":3.0,"yaw":0.4}}});
            let (base, server) = server("200 OK", body.to_string().as_bytes());
            let handle = create(&base);
            let (_, error, bytes) = take(lingtu_sdk_position(handle));
            assert_eq!(error, 0);
            let position: Value = serde_json::from_slice(&bytes).unwrap();
            assert_eq!(position["frame_id"], "map");
            assert_eq!(position["z"], 3.0);
            lingtu_sdk_client_free(handle);
            assert!(server
                .join()
                .unwrap()
                .starts_with("GET /api/v1/state HTTP/1.1"));
        }
    }

    unsafe extern "C" fn collect_event(view: *const EventView, context: *mut c_void) -> i32 {
        let view = &*view;
        let events = &mut *context.cast::<Vec<Event>>();
        let copy =
            |data, len| String::from_utf8_lossy(slice::from_raw_parts(data, len)).into_owned();
        events.push(Event {
            event: copy(view.event, view.event_len),
            id: if view.id.is_null() {
                None
            } else {
                Some(copy(view.id, view.id_len))
            },
            data: copy(view.data, view.data_len),
        });
        0
    }

    #[test]
    fn events_callback_borrows_views_and_can_stop() {
        unsafe {
            let (base, server) = server(
                "200 OK",
                b"event: update\nid: 7\ndata: a\0b\n\ndata: ignored\n\n",
            );
            let handle = create(&base);
            let query = CString::new(r#"{"topic":"/nav/status"}"#).unwrap();
            let last_id = CString::new("6").unwrap();
            let mut events: Vec<Event> = Vec::new();
            let (status, error, bytes) = take(lingtu_sdk_events(
                handle,
                query.as_ptr(),
                last_id.as_ptr(),
                Some(collect_event),
                (&mut events as *mut Vec<Event>).cast(),
            ));
            assert_eq!((status, error), (0, 0));
            assert!(bytes.is_empty());
            assert_eq!(
                events,
                vec![Event {
                    event: "update".into(),
                    id: Some("7".into()),
                    data: "a\0b".into()
                }]
            );
            lingtu_sdk_client_free(handle);
            let request = server.join().unwrap();
            assert!(request.starts_with("GET /api/v1/events?topic=%2Fnav%2Fstatus HTTP/1.1"));
            assert!(request.to_lowercase().contains("last-event-id: 6"));
        }
    }
}
