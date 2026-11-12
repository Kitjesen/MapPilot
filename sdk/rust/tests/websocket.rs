use lingtu_sdk::{json, Client, Error, Frame, Value};
use std::io::{Read, Write};
use std::net::{Shutdown, TcpListener, TcpStream};
use std::thread::{self, JoinHandle};
use std::time::Duration;
use tungstenite::handshake::server::Request;
use tungstenite::protocol::frame::coding::{Data, OpCode};
use tungstenite::protocol::frame::Frame as WireFrame;
use tungstenite::{accept_hdr, Message, WebSocket};

#[allow(clippy::result_large_err)] // Tungstenite fixes the handshake callback's error response type.
fn peer(
    action: impl FnOnce(WebSocket<TcpStream>, Request) + Send + 'static,
) -> (Client, JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    let server = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        stream
            .set_write_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let mut request = None;
        let socket = accept_hdr(stream, |incoming: &Request, response| {
            request = Some(incoming.clone());
            Ok(response)
        })
        .unwrap();
        action(socket, request.unwrap());
    });
    (
        Client::new(&base, Some("api-secret"), Duration::from_millis(600)).unwrap(),
        server,
    )
}

fn read_json(socket: &mut WebSocket<TcpStream>) -> Value {
    let message = socket.read().unwrap();
    serde_json::from_str(message.to_text().unwrap()).unwrap()
}

fn send_json(socket: &mut WebSocket<TcpStream>, value: Value) {
    socket
        .send(Message::Text(value.to_string().into()))
        .unwrap();
}

fn input_ack(socket: &mut WebSocket<TcpStream>, window: &str) -> Value {
    let request = read_json(socket);
    assert_eq!(request["type"], "input_request");
    send_json(
        socket,
        json!({"type":"input_ack","request_id":request["request_id"],"input_window":window}),
    );
    request
}

#[test]
fn preview_reassembles_fragments_preserves_binary_and_closes() {
    let (client, server) = peer(|mut socket, request| {
        assert_eq!(request.uri().path(), "/ws/cloud");
        assert_eq!(request.headers()["X-API-Key"], "api-secret");
        socket
            .send(Message::Frame(WireFrame::message(
                b"abc\0".to_vec(),
                OpCode::Data(Data::Binary),
                false,
            )))
            .unwrap();
        socket
            .send(Message::Frame(WireFrame::message(
                b"xyz".to_vec(),
                OpCode::Data(Data::Continue),
                true,
            )))
            .unwrap();
        socket.send(Message::Text("状态".into())).unwrap();
        socket.close(None).unwrap();
        let _ = socket.read();
    });
    let mut feed = client.websocket("cloud", None).unwrap();
    assert_eq!(
        feed.receive().unwrap(),
        Some(Frame::Binary(b"abc\0xyz".to_vec()))
    );
    assert_eq!(feed.receive().unwrap(), Some(Frame::Text("状态".into())));
    assert_eq!(feed.receive().unwrap(), None);
    assert_eq!(feed.receive().unwrap(), None);
    feed.close().unwrap();
    server.join().unwrap();
}

#[test]
fn websocket_accepts_configured_previews_larger_than_grpc_limit() {
    let bytes = vec![17_u8; 11 * 1024 * 1024];
    let sent = bytes.clone();
    let (client, server) = peer(move |mut socket, request| {
        assert_eq!(request.uri().path(), "/ws/scan");
        socket.send(Message::Binary(sent.into())).unwrap();
        assert!(matches!(socket.read(), Ok(Message::Close(_))));
    });
    let mut feed = client.websocket("scan", None).unwrap();
    assert_eq!(feed.receive().unwrap(), Some(Frame::Binary(bytes)));
    feed.close().unwrap();
    server.join().unwrap();
}

#[test]
fn camera_uses_stored_token_or_explicit_override() {
    for override_token in [None, Some("override-token")] {
        let expected = override_token.unwrap_or("stored-token");
        let (client, server) = peer(move |mut socket, request| {
            assert_eq!(request.uri().path(), "/ws/camera");
            assert_eq!(
                request.headers()["Authorization"],
                format!("Bearer {expected}")
            );
            assert_eq!(request.headers()["X-API-Key"], "api-secret");
            socket
                .send(Message::Binary(b"\xff\0jpeg".to_vec().into()))
                .unwrap();
            assert!(matches!(socket.read(), Ok(Message::Close(_))));
        });
        let client = client.with_media_token("stored-token");
        let mut feed = client.websocket("camera", override_token).unwrap();
        assert_eq!(
            feed.receive().unwrap(),
            Some(Frame::Binary(b"\xff\0jpeg".to_vec()))
        );
        drop(feed);
        server.join().unwrap();
    }
}

#[test]
fn media_handshake_rejection_preserves_gateway_http_status() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let mut header = Vec::new();
        let mut byte = [0];
        while !header.ends_with(b"\r\n\r\n") {
            stream.read_exact(&mut byte).unwrap();
            header.push(byte[0]);
        }
        stream
            .write_all(b"HTTP/1.1 403 Forbidden\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
            .unwrap();
    });
    let client = Client::new(&base, None, Duration::from_secs(1)).unwrap();
    match client.websocket("camera", None) {
        Err(Error::Http { status, body }) => {
            assert_eq!(status, 403);
            assert_eq!(body, "");
        }
        _ => panic!("expected HTTP handshake failure"),
    }
    server.join().unwrap();
}

#[test]
fn read_timeout_and_ping_pong_use_the_configured_socket() {
    let (client, server) = peer(|mut socket, _| {
        socket
            .send(Message::Ping(b"probe".to_vec().into()))
            .unwrap();
        assert!(matches!(socket.read(), Ok(Message::Pong(_))));
        thread::sleep(Duration::from_millis(800));
        let _ = socket.read();
    });
    let mut feed = client.websocket("scan", None).unwrap();
    assert!(matches!(
        feed.receive(),
        Err(Error::Io(_)) | Err(Error::Timeout(_))
    ));
    feed.close().unwrap();
    server.join().unwrap();
}

#[test]
fn teleop_uses_fresh_windows_and_keeps_ingress_and_hold_evidence() {
    let (client, server) = peer(|mut socket, request| {
        assert_eq!(request.uri().path(), "/ws/teleop");
        assert_eq!(request.uri().query(), Some("client_id=operator+%2B"));
        input_ack(&mut socket, "open-window");
        let input = read_json(&mut socket);
        assert_eq!(input["type"], "input_request");
        assert_eq!(input["request_id"], "move-1");
        send_json(
            &mut socket,
            json!({"type":"input_ack","request_id":"unrelated","input_window":"stale"}),
        );
        send_json(
            &mut socket,
            json!({"type":"input_ack","request_id":"move-1","input_window":"fresh-1"}),
        );
        let velocity = read_json(&mut socket);
        assert_eq!(
            velocity,
            json!({
                "type":"velocity","request_id":"move-1","input_window":"fresh-1",
                "deadman":true,"manual_mode":true,"vx_mps":0.2,"vy_mps":0.1,"yaw_rps":0.3
            })
        );
        send_json(
            &mut socket,
            json!({
                "type":"ingress_ack","request_id":"move-1","ingress_accepted":true,
                "stage":"gateway_queue_accepted","final_cmd_vel_confirmed":false,
                "motor_confirmed":false,"input_window":"never-reuse"
            }),
        );
        let next = input_ack(&mut socket, "fresh-2");
        assert_eq!(next["request_id"], "move-2");
        let velocity = read_json(&mut socket);
        assert_eq!(velocity["input_window"], "fresh-2");
        send_json(
            &mut socket,
            json!({"type":"control_rejected","request_id":"move-2","error":"input_expired"}),
        );
        let hold = read_json(&mut socket);
        assert_eq!(
            hold,
            json!({"type":"velocity","request_id":"hold-1","deadman":false,"vx_mps":0,"vy_mps":0,"yaw_rps":0})
        );
        send_json(
            &mut socket,
            json!({"type":"control_ack","action":"hold","request_id":"hold-1","final_cmd_vel_confirmed":true,"motor_confirmed":false}),
        );
        assert!(matches!(socket.read(), Ok(Message::Close(_))));
    });
    let mut teleop = client.teleop("operator +").unwrap();
    let receipt = teleop
        .move_once(0.2, 0.1, 0.3, true, Some("move-1"))
        .unwrap();
    assert_eq!(receipt["type"], "ingress_ack");
    assert_eq!(receipt["stage"], "gateway_queue_accepted");
    assert_eq!(receipt["final_cmd_vel_confirmed"], false);
    assert_eq!(receipt["motor_confirmed"], false);
    let rejected = teleop
        .move_once(0.1, 0.0, 0.0, false, Some("move-2"))
        .unwrap();
    assert_eq!(rejected["error"], "input_expired");
    let held = teleop.hold(Some("hold-1")).unwrap();
    assert_eq!(held["final_cmd_vel_confirmed"], true);
    assert_eq!(held["motor_confirmed"], false);
    teleop.close().unwrap();
    server.join().unwrap();
}

#[test]
fn input_rejection_sends_no_velocity_and_disconnects() {
    let (client, server) = peer(|mut socket, _| {
        input_ack(&mut socket, "open");
        let input = read_json(&mut socket);
        send_json(
            &mut socket,
            json!({"type":"control_rejected","request_id":input["request_id"],"error":"control_unavailable"}),
        );
        assert!(matches!(socket.read(), Ok(Message::Close(_))));
    });
    let mut teleop = client.teleop("operator").unwrap();
    let rejected = teleop
        .move_once(0.1, 0.0, 0.0, false, Some("denied"))
        .unwrap();
    assert_eq!(rejected["error"], "control_unavailable");
    drop(teleop);
    server.join().unwrap();
}

#[test]
fn opening_busy_session_preserves_rejection() {
    let (client, server) = peer(|mut socket, _| {
        send_json(
            &mut socket,
            json!({"type":"control_rejected","error":"control_in_use","message":"Another operator is connected."}),
        );
        assert_eq!(read_json(&mut socket)["type"], "input_request");
        let _ = socket.read();
    });
    match client.teleop("operator") {
        Err(Error::Operation(payload)) => assert_eq!(payload["error"], "control_in_use"),
        _ => panic!("expected owner rejection"),
    }
    server.join().unwrap();
}

#[test]
fn immediate_busy_close_keeps_the_owner_rejection() {
    let (client, server) = peer(|mut socket, _| {
        send_json(
            &mut socket,
            json!({"type":"control_rejected","error":"control_in_use","message":"Another operator is connected."}),
        );
        socket.close(None).unwrap();
        let _ = socket.get_mut().read_to_end(&mut Vec::new());
    });
    match client.teleop("operator") {
        Err(Error::Operation(payload)) => assert_eq!(payload["error"], "control_in_use"),
        Err(error) => panic!("expected owner rejection before server Close, got {error:?}"),
        Ok(_) => panic!("busy session unexpectedly opened"),
    }
    server.join().unwrap();
}

#[test]
fn teleop_reports_disconnect_without_retrying_a_sample() {
    let (client, server) = peer(|mut socket, _| {
        input_ack(&mut socket, "open");
        input_ack(&mut socket, "move");
        assert_eq!(read_json(&mut socket)["request_id"], "uncertain");
        socket.get_mut().shutdown(Shutdown::Both).unwrap();
    });
    let mut teleop = client.teleop("operator").unwrap();
    assert!(teleop
        .move_once(0.1, 0.0, 0.0, false, Some("uncertain"))
        .is_err());
    server.join().unwrap();
}

#[test]
fn teleop_transaction_timeout_releases_the_connection() {
    for stage in ["input", "move", "hold", "malformed"] {
        let (client, server) = peer(move |mut socket, _| {
            input_ack(&mut socket, "open");
            match stage {
                "input" => assert_eq!(read_json(&mut socket)["type"], "input_request"),
                "move" => {
                    input_ack(&mut socket, "fresh");
                    assert_eq!(read_json(&mut socket)["deadman"], true);
                }
                "malformed" => {
                    let input = read_json(&mut socket);
                    send_json(
                        &mut socket,
                        json!({"type":"input_ack","request_id":input["request_id"]}),
                    );
                }
                _ => assert_eq!(read_json(&mut socket)["deadman"], false),
            }
            assert!(socket.read().is_err());
        });
        let mut teleop = client.teleop("operator").unwrap();
        let result = if stage == "hold" {
            teleop.hold(Some("timed-hold"))
        } else {
            teleop.move_once(0.1, 0.0, 0.0, false, Some("timed-move"))
        };
        assert!(result.is_err());
        assert!(teleop.hold(Some("after-timeout")).is_err());
        server.join().unwrap();
    }
}

#[test]
fn media_token_reaches_camera_http_requests() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("http://{}", listener.local_addr().unwrap());
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let mut request = Vec::new();
        let mut byte = [0];
        while !request.ends_with(b"\r\n\r\n") {
            stream.read_exact(&mut byte).unwrap();
            request.push(byte[0]);
        }
        stream
            .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nJ\0G")
            .unwrap();
        String::from_utf8(request).unwrap()
    });
    let client = Client::new(&base, Some("api-secret"), Duration::from_secs(1))
        .unwrap()
        .with_media_token("viewer-secret");
    assert_eq!(client.observations().camera_snapshot().unwrap(), b"J\0G");
    let request = server.join().unwrap().to_lowercase();
    assert!(request.contains("x-api-key: api-secret\r\n"));
    assert!(request.contains("authorization: bearer viewer-secret\r\n"));
}

#[test]
fn wss_handshake_uses_tls_and_the_socket_timeout() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let base = format!("https://{}", listener.local_addr().unwrap());
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let mut prefix = [0; 3];
        stream.read_exact(&mut prefix).unwrap();
        assert_eq!(prefix[0], 22);
        assert_eq!(prefix[1], 3);
        thread::sleep(Duration::from_millis(300));
    });
    let client = Client::new(&base, None, Duration::from_millis(100)).unwrap();
    assert!(matches!(
        client.websocket("camera", None),
        Err(Error::Io(_)) | Err(Error::Timeout(_))
    ));
    server.join().unwrap();
}

unsafe fn ffi_success(reply: lingtu_sdk::ffi::Reply) -> Vec<u8> {
    let bytes = if reply.data.is_null() {
        Vec::new()
    } else {
        std::slice::from_raw_parts(reply.data, reply.len).to_vec()
    };
    lingtu_sdk::ffi::lingtu_sdk_buffer_free(reply.data, reply.len);
    assert_eq!(reply.error, 0, "{}", String::from_utf8_lossy(&bytes));
    bytes
}

#[test]
fn ffi_websocket_preserves_frame_kinds_and_binary_lengths() {
    use lingtu_sdk::ffi::*;
    use std::ffi::CString;
    use std::ptr;

    let (client, server) = peer(|mut socket, _| {
        socket
            .send(Message::Binary(b"x\0y".to_vec().into()))
            .unwrap();
        socket.send(Message::Text("a\0b".into())).unwrap();
        assert_eq!(socket.read().unwrap().to_text().unwrap(), "client-text");
        assert!(matches!(socket.read(), Ok(Message::Close(_))));
    });
    unsafe {
        let channel = CString::new("cloud").unwrap();
        let message = CString::new("client-text").unwrap();
        let mut feed = ptr::null_mut();
        ffi_success(lingtu_sdk_websocket_open(
            &client,
            channel.as_ptr(),
            ptr::null(),
            &mut feed,
        ));
        let mut kind = -1;
        assert_eq!(
            ffi_success(lingtu_sdk_websocket_receive(feed, &mut kind)),
            b"x\0y"
        );
        assert_eq!(kind, 2);
        assert_eq!(
            ffi_success(lingtu_sdk_websocket_receive(feed, &mut kind)),
            b"a\0b"
        );
        assert_eq!(kind, 1);
        ffi_success(lingtu_sdk_websocket_send_text(feed, message.as_ptr()));
        ffi_success(lingtu_sdk_websocket_close(feed));
        assert!(ffi_success(lingtu_sdk_websocket_receive(feed, &mut kind)).is_empty());
        assert_eq!(kind, 0);
        lingtu_sdk_websocket_free(feed);
        lingtu_sdk_websocket_free(ptr::null_mut());
    }
    server.join().unwrap();
}

#[test]
fn ffi_teleop_keeps_original_ingress_and_hold_receipts() {
    use lingtu_sdk::ffi::*;
    use std::ffi::CString;
    use std::ptr;

    let (client, server) = peer(|mut socket, _| {
        input_ack(&mut socket, "open");
        input_ack(&mut socket, "fresh");
        let velocity = read_json(&mut socket);
        assert_eq!(velocity["request_id"], "ffi-move");
        assert_eq!(velocity["manual_mode"], true);
        send_json(
            &mut socket,
            json!({"type":"ingress_ack","request_id":"ffi-move","ingress_accepted":true,"stage":"gateway_queue_accepted","final_cmd_vel_confirmed":false,"motor_confirmed":false}),
        );
        assert_eq!(read_json(&mut socket)["deadman"], false);
        send_json(
            &mut socket,
            json!({"type":"control_rejected","request_id":"ffi-hold","error":"hold_unconfirmed","final_cmd_vel_confirmed":false}),
        );
        assert!(matches!(socket.read(), Ok(Message::Close(_))));
    });
    unsafe {
        let client_id = CString::new("ffi-operator").unwrap();
        let move_id = CString::new("ffi-move").unwrap();
        let hold_id = CString::new("ffi-hold").unwrap();
        let mut teleop = ptr::null_mut();
        ffi_success(lingtu_sdk_teleop_open(
            &client,
            client_id.as_ptr(),
            &mut teleop,
        ));
        let receipt: Value = serde_json::from_slice(&ffi_success(lingtu_sdk_teleop_move(
            teleop,
            0.1,
            0.0,
            0.0,
            1,
            move_id.as_ptr(),
        )))
        .unwrap();
        assert_eq!(receipt["type"], "ingress_ack");
        assert_eq!(receipt["final_cmd_vel_confirmed"], false);
        let receipt: Value = serde_json::from_slice(&ffi_success(lingtu_sdk_teleop_hold(
            teleop,
            hold_id.as_ptr(),
        )))
        .unwrap();
        assert_eq!(receipt["error"], "hold_unconfirmed");
        ffi_success(lingtu_sdk_teleop_close(teleop));
        lingtu_sdk_teleop_free(teleop);
        lingtu_sdk_teleop_free(ptr::null_mut());
    }
    server.join().unwrap();
}
