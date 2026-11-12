# LingTu Rust SDK

Blocking external Rust client for the LingTu Gateway HTTP, SSE and WebSocket APIs, normally served on port 5050. This crate lives independently of the Python Host and native runtime. Its version follows the LingTu 2.3.0 release.

## Build and use

```toml
[dependencies]
lingtu-sdk = { path = "path/to/lingtu/sdk/rust" }
```

```rust
use lingtu_sdk::Client;
use std::time::Duration;

fn main() -> lingtu_sdk::Result<()> {
    let client = Client::new(
        "http://127.0.0.1:5050",
        std::env::var("LINGTU_API_KEY").ok().as_deref(),
        Duration::from_secs(10),
    )?;
    println!("{}", client.system().health()?);
    Ok(())
}
```

The executable example only reads health.

```powershell
cargo run --manifest-path sdk/rust/Cargo.toml --example health -- http://127.0.0.1:5050
```

`Client` is cloneable and reuses its HTTP connection pool. Its direct dependencies are the approved `reqwest` 0.12.28, `serde_json` 1.0.154 and `tungstenite` 0.29.0. HTTPS and WSS use rustls with WebPKI roots. Authentication uses `X-API-Key`. Redirect following and automatic retries are disabled.

HTTP respects the system's proxy environment. Include `127.0.0.1,localhost,::1`
in `NO_PROXY` for direct local Gateway access when a proxy is configured.

## Domains

Every JSON operation returns `Result<serde_json::Value, Error>`. Keep the entire returned value when recording admission, execution, failure reasons and identities. Optional request bodies accept JSON objects or `Value::Null`. Required positional fields take precedence over fields in those options.

| Domain | Named operations |
| --- | --- |
| `navigation()` | `go`, `go_to_click`, planner selection, current status/path, task lookup, cancel/pause/resume/speed limit, exploration, seeded/global relocalization, map tracking, localization status, map-frame position, `wait_until_arrived` |
| `map()` | List/save/save-and-wait, list/query/cancel/retry/process/reoptimize operations, PCD/source/OctoMap downloads, live/saved points, surface query, 3D no-go CRUD, source-edit preview/save-as/crop, voxel editing/history, occupancy/OctoMap rebuild, plan preview, import/delete/rename, named locations |
| `tasks()` | Instructions, Agent run lookup/cancellation, owned Follow lookup/cancellation, inspection routes/start/status/pause/resume/cancel/report |
| `observations()` | Scene graph, JPEG snapshot bytes, camera/media status and diagnostics, filtered temporal memory, embedding search, native recording start/stop/status |
| `system()` | State/health/session, mode, capabilities/bootstrap/readiness, read-only field check, runtime contract, SLAM status, software emergency stop/reset, stop/stand/sit, motor enable/disable, profile selection, control leases |

Navigation coordinates are map-frame metres. The optional `yaw` is in radians. Supply `request_id` and `task_id` in options to retain caller-selected identities. A command gets a generated request ID when the field is absent, and a missing response request ID is filled with the submitted identity. Server-supplied identities remain intact.

The following call submits a motion request when executed against a robot. Use the read-only health example for connectivity checks.

```rust,ignore
let receipt = client.navigation().go(1.0, 2.0, 0.0, json!({
    "yaw": 0.4,
    "request_id": "operator-request-42",
    "task_id": "inspection-leg-3",
    "max_speed_mps": 0.3
}))?;
```

Requests continue through the Gateway's existing owners and native admission/safety path. A command receipt reports acceptance and owner evidence. Physical completion requires task/status evidence. `wait_until_arrived` reads the identified task ledger. `save_and_wait` submits once, then reads the durable operation. Stopping a local wait leaves the external task under its existing owner. A transport failure during submission leaves admission unknown, so reconcile using the supplied request identity before deciding to resubmit.

Map operations use the current HTTP schemas. `set_no_go(name, region, expected_content_epoch)` sends the epoch in the body. `delete_no_go(name, region_id, expected_content_epoch)` sends it in the query. Source edits use `expected_source_content_epoch` and `operations`. `save_as` also needs the new `name`. `crop` accepts the route's `bounds` options. `import_pcd` takes a source path inside the Gateway machine's configured import directory. Download targets are local to the SDK caller.

`surface` accepts query pairs for `headroom_m`, `radius_m`, `max_hole_area_m2` and `max_slope`. Surface inspection and plan preview remain evidence for their declared API operations. Native navigation retains motion authorization.

## Low-level requests and errors

```rust,ignore
let value = client.request("GET", "/api/v1/readiness", &[], None)?;
let bytes = client.bytes("/api/v1/camera/snapshot", &[])?;
client.map().download_pcd("warehouse", "warehouse.pcd")?;
```

Query pairs and named path arguments are URL encoded. Low-level paths are caller-supplied Gateway paths and can include an existing query string. The base URL selects the server origin. Gateway paths start at that origin.

`Error::Http { status, body }` retains the complete HTTP failure response for HTTP requests. `Transport` retains reqwest transport details. `Json` reports malformed JSON, `Io` reports local file or streamed read failures, `InvalidInput` reports invalid client arguments, and polling helpers return `Operation` with the owner payload or `Timeout`. `Unsupported` reports a requested capability with no matching Gateway operation. `WebSocket` retains protocol errors. WebSocket handshake failures retain HTTP status, but pinned tungstenite may omit the optional rejection response body.

`temporal_memory(query_pairs)` accepts `label`, `since`, `near_x`, `near_y`, `radius` and `limit`. Semantic search uses `search_memory_embedding(&vector, options)` with optional `top_k`, `since` and `label`. The vector must come from the model used by the observation store. The recognizable `search_memory(text)` method returns `Unsupported` because the Gateway provides no text encoder.

Downloads stream into a uniquely created sibling `.part` file, check a declared Content-Length, sync and close the completed file, then use the platform's replacement rename. Failed transfers remove partial files and retain an existing destination. Replacement has been exercised on Windows. The parent directory must already exist.

## Events

```rust,ignore
let mut events = client.events(&[("topic", "/nav/status")])?;
if let Some(event) = events.next() {
    let event = event?;
    println!("{} {:?} {}", event.event, event.id, event.data);
}
drop(events);
```

`Event` contains `event`, `id` and raw UTF-8 `data`. The parser handles comments, LF/CRLF/CR endings, multiline data, ID persistence and clean EOF. A partial event at EOF is discarded according to SSE framing. Dropping the iterator releases its owned response. Reqwest manages the underlying pooled socket lifetime. `events_after(query, Some(last_id))` supplies `Last-Event-ID` for an explicit reconnect.

The pinned blocking reqwest implementation applies the configured timeout independently to each streamed read. A stream can remain open beyond that duration while events or Gateway heartbeats arrive. An idle read fails after the configured timeout. Choose a timeout greater than the Gateway's one-second heartbeat interval. JSON response decoding has its own bounded body-read operation. Polling deadlines are checked between requests, so an in-flight request can consume its configured timeout after the polling deadline.

The shared schemas in `../proto` describe the separate gRPC API for consumers who generate a client with their chosen runtime.

## Product and saved data

`system().product_status()`, `switch_product(product, expected_session, options)`
and `product_operation(request_id)` retain the complete ProductControl operation.
Switch options carry `map_name`, `request_id` and an optional `initial_pose`.
The transaction's `state` and `result` remain available to the caller.

`stop_product(expected_session, options)` submits the existing ProductControl stop
transaction. Supply `request_id` before submission. Stopping can also close the
Gateway connection. The independent control service retains the operation for
lookup when the Gateway is reachable again or through the local control
transport. A lost response leaves completion unconfirmed.

`navigation().goal_status(request_id)` looks up a goal after a lost submission
reply. `system().resume_motion(options)` releases native motion hold and keeps
the owner's original receipt. `observations().stop_recording_session(session_id)`
preserves the native expected-session check. The original `stop_recording()`
continues to stop the active recording.

`observations()` includes recording catalog, stream and saved-session lookup,
summary, series, scene and historical camera queries, validation and deletion.
`recording_scene` accepts repeated `path_topic` query pairs. A camera query that
returns HTTP 204 produces an empty byte vector. `download_recording_file`
accepts a declared relative artifact path, including nested folders.
`download_recording_archive` saves the session tar archive. The recording owner
decides whether deletion is permitted.

`tasks()` also saves, reads and deletes inspection routes, lists retained tasks,
looks up and reviews evidence, and downloads artifacts or individual sensor
measurements. Review takes a verdict and note and records the human finding.
Route JSON uses the public map, revision and point fields. These downloads use
the existing atomic writer and preserve the destination after a failed transfer.

## WebSocket previews and media access

```rust,ignore
use lingtu_sdk::Frame;

let token = std::env::var("LINGTU_MEDIA_TOKEN")?;
let client = client.with_media_token(&token);
let mut camera = client.websocket("camera", None)?;
if let Some(Frame::Binary(jpeg)) = camera.receive()? {
    std::fs::write("snapshot.jpg", jpeg)?;
}
camera.close()?;
```

`websocket("cloud" | "scan" | "camera", optional_media_token)` opens one connection to its Gateway route. `Feed::receive` returns a complete `Frame::Binary` or UTF-8 `Frame::Text`, reassembling fragmented messages. Binary bytes stay unchanged. Clean close returns `None`. Configured large Gateway previews are accepted without borrowing the separate gRPC cloud frame limit. `send_text` sends one UTF-8 message. `close` sends Close and releases the TCP socket, and dropping a feed closes it.

`with_media_token` supplies the independent media-viewer Bearer credential for HTTP camera/media requests and WebSocket handshakes. `X-API-Key` continues to supply API authentication. A per-feed media token overrides the stored viewer token. Both WS and WSS use the configured timeout for socket reads and writes. Each receive call has a bounded message wait, including fragmented frames and Ping/Pong handling. After a preview read timeout, the caller can continue waiting on that connection or close it explicitly.

## Operator teleoperation

The following example acquires an operator session and submits one motion sample when executed against a robot.

```rust,ignore
let mut teleop = client.teleop("operator-app")?;
let receipt = teleop.move_once(0.1, 0.0, 0.0, false, Some("operator-sample-1"))?;
println!("{}", receipt);
let held = teleop.hold(Some("operator-release-1"))?;
println!("{}", held);
teleop.close()?;
```

Opening a session performs the Gateway's explicit ownership transaction and requests input readiness. Every `move_once` obtains a fresh matching `input_ack`, consumes that one-use `input_window`, and sends one velocity sample with `deadman=true`. Values use metres per second and radians per second. The caller controls `manual_mode` and the stable request identity. A successful submission returns the original `ingress_ack` with `stage="gateway_queue_accepted"` and its separate `final_cmd_vel_confirmed` and `motor_confirmed` fields. Rejections retain their original owner reason.

`hold` sends zero velocity with `deadman=false` and returns the original `control_ack` or `control_rejected`, including hold confirmation evidence. There is no automatic sample repetition, reconnect or session reacquisition. A failed teleop transaction closes its connection while preserving the error and unknown admission result. A subsequent command requires explicitly opening another session. `close` and drop release the socket so the Gateway runs its existing disconnect transaction. Physical stopping requires the native owner's confirmation evidence.

The C++ SDK uses this same implementation through the length-delimited C ABI in `ffi`. Rust owns its client, feed, teleop and reply-buffer allocations. Every C ABI allocation has a matching Rust release function.

## Verification

```powershell
cargo test --offline --manifest-path sdk/rust/Cargo.toml
cargo fmt --manifest-path sdk/rust/Cargo.toml -- --check
cargo clippy --offline --manifest-path sdk/rust/Cargo.toml --all-targets -- -D warnings
```

Tests use local `std::net::TcpListener` servers to check real HTTP requests and streaming behavior without a robot. They establish local client contracts. Native simulation, field readiness and physical motion require their own validation.

WSS currently uses WebPKI roots. A caller-supplied private CA is not yet configurable.
