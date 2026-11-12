use lingtu_sdk::{json, Client, Error, Value};
use std::fs;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::path::PathBuf;
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

fn read_request(stream: &mut TcpStream) -> String {
    stream
        .set_read_timeout(Some(Duration::from_secs(5)))
        .unwrap();
    let mut request = Vec::new();
    let mut byte = [0];
    while !request.ends_with(b"\r\n\r\n") {
        stream.read_exact(&mut byte).unwrap();
        request.push(byte[0]);
    }
    let headers = String::from_utf8(request.clone()).unwrap();
    let length = headers
        .lines()
        .find_map(|line| {
            let (key, value) = line.split_once(':')?;
            key.eq_ignore_ascii_case("content-length")
                .then(|| value.trim().parse::<usize>().unwrap())
        })
        .unwrap_or(0);
    let mut payload = vec![0; length];
    stream.read_exact(&mut payload).unwrap();
    request.extend(payload);
    String::from_utf8(request).unwrap()
}

fn serve(
    status: &str,
    body: &[u8],
    declared_length: Option<usize>,
) -> (Client, JoinHandle<String>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let status = status.to_owned();
    let body = body.to_vec();
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let request = read_request(&mut stream);
        write!(stream, "HTTP/1.1 {status}\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
            declared_length.unwrap_or(body.len())
        ).unwrap();
        // Separate UTF-8 bytes and event lines across network writes.
        for chunk in body.chunks(3) {
            stream.write_all(chunk).unwrap();
        }
        request
    });
    (
        Client::new(&url, Some("test-secret"), Duration::from_secs(5)).unwrap(),
        server,
    )
}

fn target() -> PathBuf {
    let path = std::env::temp_dir().join(format!(
        "lingtu-rust-{}-{}",
        std::process::id(),
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    fs::create_dir(&path).unwrap();
    path.join("map.pcd")
}

#[test]
fn stop_resume_and_request_reconciliation_keep_owner_contracts() {
    let (client, server) = serve("202 Accepted", br#"{"state":"running"}"#, None);
    assert_eq!(
        client
            .system()
            .stop_product("session-1", json!({"request_id":"stop-1"}))
            .unwrap()["state"],
        "running"
    );
    let request = server.join().unwrap();
    assert!(request.starts_with("POST /api/v1/product-control/stop HTTP/1.1"));
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(
        body,
        json!({"expected_product_session_id":"session-1", "request_id":"stop-1"})
    );

    let (client, server) = serve("200 OK", br#"{"previous_motion_restored":false}"#, None);
    assert_eq!(
        client
            .system()
            .resume_motion(json!({"request_id":"resume-1", "client_id":"operator"}))
            .unwrap()["previous_motion_restored"],
        false
    );
    assert!(server
        .join()
        .unwrap()
        .starts_with("POST /api/v1/navigation/resume HTTP/1.1"));

    let (client, server) = serve("200 OK", br#"{"found":false}"#, None);
    assert_eq!(
        client.navigation().goal_status("goal/a?").unwrap()["found"],
        false
    );
    assert!(server
        .join()
        .unwrap()
        .starts_with("GET /api/v1/navigation/goals/goal%2Fa%3F HTTP/1.1"));

    let (client, server) = serve(
        "409 Conflict",
        br#"{"error":"recording_session_mismatch"}"#,
        None,
    );
    assert!(
        matches!(client.observations().stop_recording_session("recording-1"), Err(Error::Http {status:409, body}) if body.contains("recording_session_mismatch"))
    );
    let request = server.join().unwrap();
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(body, json!({"expected_session_id":"recording-1"}));
}

#[test]
fn product_switch_keeps_operation_and_session_identity() {
    let response = br#"{"request_id":"switch-1","state":"running","result":null}"#;
    let (client, server) = serve("202 Accepted", response, None);
    let operation = client
        .system()
        .switch_product(
            "nav",
            "session-1",
            json!({
                "request_id":"switch-1", "map_name":"hall",
                "initial_pose":{"x":1,"y":2,"z":0,"yaw":0}
            }),
        )
        .unwrap();
    assert_eq!(
        operation,
        serde_json::from_slice::<Value>(response).unwrap()
    );
    let request = server.join().unwrap();
    assert!(request.starts_with("POST /api/v1/product-control/switch HTTP/1.1"));
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(body["expected_product_session_id"], "session-1");
    assert_eq!(body["initial_pose"]["y"], 2);
    let (client, server) = serve("200 OK", b"{}", None);
    client.system().product_operation("switch/a?").unwrap();
    assert!(server
        .join()
        .unwrap()
        .starts_with("GET /api/v1/product-control/operations/switch%2Fa%3F HTTP/1.1"));
}

#[test]
fn recording_nested_file_preserves_binary_data() {
    let file = target();
    let (client, server) = serve("200 OK", b"\0\xffrecording", None);
    client
        .observations()
        .download_recording_file("record-1", "camera/front +.mkv", &file)
        .unwrap();
    assert_eq!(fs::read(&file).unwrap(), b"\0\xffrecording");
    assert!(server
        .join()
        .unwrap()
        .starts_with("GET /api/v1/recordings/record-1/files/camera/front%20%2B%2Emkv HTTP/1.1"));
    fs::remove_dir_all(file.parent().unwrap()).unwrap();
}

#[test]
fn recording_replay_queries_preserve_topics_and_empty_frame() {
    let (client, server) = serve("200 OK", b"{}", None);
    client
        .observations()
        .recording_scene("record-1", &[("path_topic", "/a"), ("path_topic", "/b")])
        .unwrap();
    assert!(server.join().unwrap().starts_with(
        "GET /api/v1/recordings/record-1/scene?path_topic=%2Fa&path_topic=%2Fb HTTP/1.1"
    ));
    let (client, server) = serve("204 No Content", b"", None);
    assert!(client
        .observations()
        .recording_camera_frame("record-1", 1.5, 2, &[])
        .unwrap()
        .is_empty());
    assert!(server
        .join()
        .unwrap()
        .starts_with("GET /api/v1/recordings/record-1/camera-frame?t=1.5&segment=2 HTTP/1.1"));
}

#[test]
fn inspection_review_is_a_finding_with_original_errors() {
    let (client, server) = serve("200 OK", b"{}", None);
    client
        .tasks()
        .review_inspection_evidence("evidence-1", "anomaly", "glass\nwall")
        .unwrap();
    let request = server.join().unwrap();
    assert!(request.starts_with("POST /api/v1/inspection/evidence/evidence-1/review HTTP/1.1"));
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(body, json!({"verdict":"anomaly", "note":"glass\nwall"}));
    let response = br#"{"detail":{"reason":"evidence_integrity_error"}}"#;
    let (client, server) = serve("409 Conflict", response, None);
    assert!(
        matches!(client.tasks().inspection_evidence("evidence-1"), Err(Error::Http { status:409, body })
        if body == String::from_utf8_lossy(response))
    );
    server.join().unwrap();
}

#[test]
fn goal_preserves_identity_and_coordinates() {
    let (client, server) = serve(
        "202 Accepted",
        br#"{"accepted":true,"task_id":"task-original"}"#,
        None,
    );
    let result = client
        .navigation()
        .go(
            1.0,
            2.0,
            3.0,
            json!({
                "yaw": 0.5,
                "request_id": "request-original",
                "task_id": "task-original",
                "max_speed_mps": 0.3
            }),
        )
        .unwrap();
    assert_eq!(result["request_id"], "request-original");
    assert_eq!(result["task_id"], "task-original");
    let request = server.join().unwrap();
    assert!(request.starts_with("POST /api/v1/goal HTTP/1.1\r\n"));
    assert!(request
        .to_lowercase()
        .contains("x-api-key: test-secret\r\n"));
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(body["request_id"], "request-original");
    assert_eq!(body["task_id"], "task-original");
    assert_eq!(body["z"], 3.0);
    assert_eq!(body["yaw"], 0.5);
}

#[test]
fn paths_and_queries_are_encoded() {
    let (client, server) = serve("200 OK", b"{}", None);
    client
        .tasks()
        .get_follow_task("task/中文?", "session +&?")
        .unwrap();
    let request = server.join().unwrap();
    assert!(request.starts_with("GET /api/v1/follow/task%2F%E4%B8%AD%E6%96%87%3F?product_session_id=session+%2B%26%3F HTTP/1.1"),"{request}");
}

#[test]
fn no_go_delete_sends_epoch_as_query() {
    let (client, server) = serve("200 OK", b"{}", None);
    client
        .map()
        .delete_no_go("warehouse", "region/a", 7)
        .unwrap();
    assert!(server.join().unwrap().starts_with(
        "DELETE /api/v1/maps/warehouse/no_go/region%2Fa?expected_content_epoch=7 HTTP/1.1"
    ));
}

#[test]
fn http_error_keeps_status_and_detail() {
    let body = br#"{"detail":{"reason_code":"content_epoch_mismatch"}}"#;
    let (client, server) = serve("409 Conflict", body, None);
    match client.system().health().unwrap_err() {
        Error::Http {
            status,
            body: actual,
        } => {
            assert_eq!(status, 409);
            assert_eq!(actual, String::from_utf8_lossy(body));
        }
        other => panic!("{other:?}"),
    }
    server.join().unwrap();
}

#[test]
fn transport_error_is_distinct() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    drop(listener);
    let client = Client::new(&url, None, Duration::from_millis(300)).unwrap();
    assert!(matches!(client.system().health(), Err(Error::Transport(_))));
}

#[test]
fn download_replaces_old_target_after_success() {
    let target = target();
    fs::write(&target, b"old").unwrap();
    let (client, server) = serve("200 OK", b"complete cloud", None);
    assert_eq!(
        client.map().download_pcd("warehouse", &target).unwrap(),
        target
    );
    assert_eq!(fs::read(&target).unwrap(), b"complete cloud");
    assert_eq!(fs::read_dir(target.parent().unwrap()).unwrap().count(), 1);
    assert!(server
        .join()
        .unwrap()
        .starts_with("GET /api/v1/maps/warehouse/pcd HTTP/1.1"));
    fs::remove_dir_all(target.parent().unwrap()).unwrap();
}

#[test]
fn incomplete_download_preserves_target_and_cleans_partial() {
    let target = target();
    fs::write(&target, b"old").unwrap();
    let (client, server) = serve("200 OK", b"partial", Some(100));
    assert!(client.map().download_octomap("warehouse", &target).is_err());
    assert_eq!(fs::read(&target).unwrap(), b"old");
    assert_eq!(fs::read_dir(target.parent().unwrap()).unwrap().count(), 1);
    server.join().unwrap();
    fs::remove_dir_all(target.parent().unwrap()).unwrap();
}

#[test]
fn events_parse_utf8_comments_multiline_ids_and_clean_eof() {
    let body = concat!(
        "\u{feff}: comment\r\nevent: discarded\r\n\r\n",
        "id: 17\r\nevent: update\r\ndata: 中文\r\ndata: second\r\n\r\n",
        ": heartbeat\rid: 18\rdata: next\r\rdata: unfinished"
    );
    let (client, server) = serve("200 OK", body.as_bytes(), None);
    let mut events = client.events(&[("topic", "/nav/status")]).unwrap();
    let first = events.next().unwrap().unwrap();
    assert_eq!(first.event, "update");
    assert_eq!(first.id.as_deref(), Some("17"));
    assert_eq!(first.data, "中文\nsecond");
    let second = events.next().unwrap().unwrap();
    assert_eq!(second.event, "message");
    assert_eq!(second.id.as_deref(), Some("18"));
    assert_eq!(second.data, "next");
    assert!(events.next().is_none());
    assert!(events.next().is_none());
    let request = server.join().unwrap();
    assert!(request.starts_with("GET /api/v1/events?topic=%2Fnav%2Fstatus HTTP/1.1"));
    assert!(request.to_lowercase().contains("accept: text/event-stream"));
}

#[test]
fn invalid_options_and_nonfinite_goals_fail_before_network() {
    let client = Client::new("http://127.0.0.1:1", None, Duration::from_millis(100)).unwrap();
    assert!(matches!(
        client.navigation().go(1.0, 2.0, f64::NAN, Value::Null),
        Err(Error::InvalidInput(_))
    ));
    assert!(matches!(
        client.map().save(json!([])),
        Err(Error::InvalidInput(_))
    ));
}

#[test]
fn events_continue_past_timeout_while_data_arrives() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let mut request = Vec::new();
        let mut byte = [0];
        while !request.ends_with(b"\r\n\r\n") {
            stream.read_exact(&mut byte).unwrap();
            request.push(byte[0]);
        }
        stream
            .write_all(
                b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nConnection: close\r\n\r\n",
            )
            .unwrap();
        for _ in 0..8 {
            stream.write_all(b"data: heartbeat\n\n").unwrap();
            stream.flush().unwrap();
            thread::sleep(Duration::from_millis(70));
        }
        String::from_utf8(request).unwrap()
    });
    let client = Client::new(&url, None, Duration::from_millis(300)).unwrap();
    let events = client.events_after(&[], Some("last-17")).unwrap();
    let received = events.collect::<Result<Vec<_>, _>>().unwrap();
    assert_eq!(received.len(), 8);
    assert!(server
        .join()
        .unwrap()
        .to_lowercase()
        .contains("last-event-id: last-17\r\n"));
}

#[test]
fn events_idle_reads_timeout() {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let url = format!("http://{}", listener.local_addr().unwrap());
    let server = thread::spawn(move || {
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
        stream.write_all(b"HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\nConnection: close\r\n\r\ndata: initial\n\n").unwrap();
        thread::sleep(Duration::from_millis(500));
    });
    let client = Client::new(&url, None, Duration::from_millis(300)).unwrap();
    let mut events = client.events(&[]).unwrap();
    assert_eq!(events.next().unwrap().unwrap().data, "initial");
    assert!(matches!(events.next(), Some(Err(Error::Io(_)))));
    assert!(events.next().is_none());
    drop(events);
    server.join().unwrap();
}

#[test]
fn navigation_wait_checks_task_identity_and_failure() {
    for (body, expected) in [
        (
            json!({"found":true,"task_id":"wanted","status":{"task_id":"wanted","lifecycle_state_name":"SUCCESS"}}),
            "success",
        ),
        (
            json!({"found":true,"task_id":"other","status":{"task_id":"other","lifecycle_state_name":"SUCCESS"}}),
            "timeout",
        ),
        (
            json!({"found":true,"task_id":"wanted","status":{"task_id":"wanted","lifecycle_state_name":"FAILED","failure_reason":"blocked"}}),
            "failure",
        ),
    ] {
        let (client, server) = serve("200 OK", body.to_string().as_bytes(), None);
        let result = client.navigation().wait_until_arrived(
            "wanted",
            Duration::from_millis(100),
            Duration::from_secs(1),
        );
        match expected {
            "success" => {
                assert!(result.is_ok());
            }
            "timeout" => assert!(matches!(result, Err(Error::Timeout(_)))),
            _ => assert!(matches!(result, Err(Error::Operation(_)))),
        }
        server.join().unwrap();
    }
}

#[test]
fn body_controls_forward_identity_to_gateway() {
    let (client, server) = serve("200 OK", br#"{"accepted":true}"#, None);
    let receipt = client
        .system()
        .switch_profile(
            "stairs",
            json!({"request_id":"profile-request","client_id":"operator"}),
        )
        .unwrap();
    assert_eq!(receipt["request_id"], "profile-request");
    let request = server.join().unwrap();
    assert!(request.starts_with("POST /api/v1/motion/profile HTTP/1.1"));
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(
        body,
        json!({"request_id":"profile-request","client_id":"operator","name":"stairs"})
    );
}

#[test]
fn strict_request_schemas_receive_only_supported_fields() {
    let (client, server) = serve("202 Accepted", br#"{"accepted":true}"#, None);
    client
        .map()
        .save(json!({"name":"warehouse","request_id":"save-request"}))
        .unwrap();
    let request = server.join().unwrap();
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(
        body,
        json!({"name":"warehouse","request_id":"save-request"})
    );

    let (client, server) = serve("200 OK", b"{}", None);
    client
        .navigation()
        .global_relocalize("warehouse", json!({"request_id":"align-request"}))
        .unwrap();
    let request = server.join().unwrap();
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(
        body,
        json!({"map_name":"warehouse","mode":"global","request_id":"align-request"})
    );

    let (client, server) = serve("200 OK", b"{}", None);
    client
        .map()
        .validate_plan("warehouse", json!({"x":1.0,"y":2.0,"z":3.0}))
        .unwrap();
    let request = server.join().unwrap();
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(body, json!({"x":1.0,"y":2.0,"z":3.0}));
}

#[test]
fn memory_search_uses_model_embedding() {
    let (client, server) = serve("200 OK", b"{}", None);
    client
        .observations()
        .search_memory_embedding(&[0.1, 0.2], json!({"top_k":3,"label":"door"}))
        .unwrap();
    let request = server.join().unwrap();
    assert!(request.starts_with("POST /api/v1/memory/temporal/semantic HTTP/1.1"));
    let body: Value = serde_json::from_str(request.split_once("\r\n\r\n").unwrap().1).unwrap();
    assert_eq!(
        body,
        json!({"embedding":[0.1,0.2],"top_k":3,"label":"door"})
    );
    assert!(matches!(
        client.observations().search_memory("door"),
        Err(Error::Unsupported(_))
    ));
}

#[test]
fn map_wait_rejects_rejected_or_mismatched_receipts() {
    for body in [
        json!({"ok":false,"accepted":false,"operation_id":"job-1","request_id":"wanted"}),
        json!({"ok":true,"accepted":true,"operation_id":"job-1","request_id":"other"}),
        json!({"ok":true,"accepted":true,"operation_id":"job-1","request_id":"wanted","operation":{"operation_id":"job-2"}}),
    ] {
        let (client, server) = serve("200 OK", body.to_string().as_bytes(), None);
        assert!(matches!(
            client.map().save_and_wait(
                json!({"request_id":"wanted"}),
                Duration::from_secs(1),
                Duration::from_millis(10),
                false
            ),
            Err(Error::Operation(_))
        ));
        server.join().unwrap();
    }
}

#[test]
fn map_wait_observes_owned_operation_without_resubmitting() {
    for (operation, source_only, expected) in [
        (
            json!({"operation_id":"job-1","state":"SUCCEEDED"}),
            false,
            "success",
        ),
        (
            json!({"operation_id":"job-1","state":"FAILED","source_saved":true}),
            true,
            "success",
        ),
        (
            json!({"operation_id":"other","state":"SUCCEEDED"}),
            false,
            "failure",
        ),
        (
            json!({"operation_id":"job-1","state":"FAILED","reason_code":"build_failed"}),
            false,
            "failure",
        ),
    ] {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let receipt =
            json!({"ok":true,"accepted":true,"operation_id":"job-1","request_id":"save-request"});
        let response = json!({"ok":true,"operation":operation});
        let server = thread::spawn(move || {
            let mut requests = Vec::new();
            for body in [receipt, response] {
                let (mut stream, _) = listener.accept().unwrap();
                requests.push(read_request(&mut stream));
                let body = body.to_string();
                write!(
                    stream,
                    "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
                    body.len()
                )
                .unwrap();
            }
            requests
        });
        let client = Client::new(&url, None, Duration::from_secs(2)).unwrap();
        let result = client.map().save_and_wait(
            json!({"request_id":"save-request"}),
            Duration::from_secs(3),
            Duration::from_millis(10),
            source_only,
        );
        match expected {
            "success" => assert!(result.is_ok(), "{result:?}"),
            _ => assert!(matches!(result, Err(Error::Operation(_)))),
        }
        let requests = server.join().unwrap();
        assert!(requests[0].starts_with("POST /api/v1/map/save HTTP/1.1"));
        assert!(requests[1].starts_with("GET /api/v1/maps/operations/job-1 HTTP/1.1"));
    }
}
