# LingTu SDK

The external clients connect to the selected LingTu Gateway. HTTP and SSE use
port 5050 by default. Public gRPC uses port 5052 and has its own credentials.
Each command continues through its existing task, map, Product or native
motion owner.

| Distribution | Entry | Installation and examples |
| --- | --- | --- |
| Python | `lingtu.sdk.LingTuClient`, `AsyncLingTuClient`, `lingtu-sdk` | [Python package](python/README.md) |
| C++17 | `lingtu::sdk::Client`, CMake target `LingTu::SDK` | [C++ package](cpp/README.md) |
| Rust | `lingtu_sdk::Client` | [Rust crate](rust/README.md) |
| Public gRPC schemas | `doso.v1`, seven services and 73 RPCs | [Protocol source and generation](proto/README.md) |

The maintained HTTP route and Python method reference is generated from the
current source. See [API](../docs/api.md).

C++ and Rust share the same Rust HTTP, download, event and WebSocket implementation.
The C++ facade crosses a sized-buffer C ABI and uses the C++ standard library.
Source builds require Cargo. The installed C++ development package includes
the compiled core and its import library for use by external CMake projects.
The five C++ and Rust domain views borrow their client, which must outlive them.

```cpp
#include <lingtu/sdk.hpp>
#include <iostream>

int main() {
    lingtu::sdk::Client robot("http://robot-host:5050");
    std::cout << robot.system().health() << '\n';
}
```

```rust,no_run
use lingtu_sdk::Client;
use std::time::Duration;

let robot = Client::new("http://robot-host:5050", None, Duration::from_secs(10))?;
println!("{}", robot.system().health()?);
# Ok::<(), lingtu_sdk::Error>(())
```

## Shared business domains

| Domain | Responsibilities |
| --- | --- |
| `navigation` | XYZ goals, task status, cancel/pause/resume, speed limits, paths, exploration and relocalization |
| `map` | Save operations, archived PCD and OctoMap downloads, reoptimization, 3D no-go regions, support surfaces and map editing |
| `tasks` | Instructions, Agent runs, owned Follow executions and inspections |
| `observations` | Scenes, JPEG snapshots, camera status, temporal observations, vector queries and recordings |
| `system` | Health, session identity, readiness, leases, emergency stop and explicitly requested body commands |

Clients preserve the returned owner fields. Navigation admission and physical
completion have separate evidence. After a goal is accepted, retain its task
identity and inspect that task to determine its result. Ordinary navigation
completes on XYZ position. A goal's yaw field is pose metadata.

Follow uses both `follow_task_id` and `product_session_id`. Map mutations use
their declared content epoch. Source-edit operations use
`expected_source_content_epoch`. Product activation stays with ProductControl.

## Python installation and read-only check

Build from the repository root using the installed setuptools build backend.

```sh
python tools/sdk/package_python.py
python -m pip install --force-reinstall build/sdk/dist/lingtu_sdk-2.3.0-py3-none-any.whl
lingtu-sdk --host robot-host health
```

The base package declares zero runtime dependencies. Add the `websocket` extra
for live binary feeds and teleoperation.
Install the client in the external application's environment. Its package
namespace is also used by the full LingTu runtime distribution.

```python
from lingtu.sdk import LingTuClient

robot = LingTuClient("robot-host", timeout=10.0)
print(robot.system.health())
print(robot.system.session())
print(robot.map.list())
```

For an HTTPS Gateway, use an explicit origin with either Python client or the CLI.
Certificate verification uses the platform's normal trust configuration.

```python
robot = LingTuClient(base_url="https://robot-host:7443", api_key="gateway-key")
```

```sh
lingtu-sdk --base-url https://robot-host:7443 --api-key gateway-key health
```

## Map save and processing

The default save archives the point cloud first, then processes navigation
artifacts in the background. A caller can choose manual processing or return
as soon as the archive is saved.

```python
archived = robot.map.save_and_wait("hall", auto_process=False)
operation_id = archived["operation_id"]
robot.map.download_source(operation_id, "hall-source.pcd")
processing = robot.map.process_operation(operation_id)
state = robot.map.operation(operation_id)
```

`save_and_wait()` follows the durable operation identity. A local wait timeout
leaves the external operation under its owner. Continue querying the existing
operation after a connection failure. Download methods replace a local target
only after the complete transfer succeeds.

`map.surface()` reads observed and inferred support. `map.no_go()`,
`set_no_go()` and `delete_no_go()` address the independent 3D forbidden layer.
Source editing offers preview, crop and save-as. These calls use the map
service's actual data and version rules.

## Product, recording and inspection workflows

`system.product_status()`, `switch_product()`, `stop_product()` and `product_operation()` use
the existing ProductControl transport. Supply the expected Product session
when switching and retain the returned operation identity. Its `state` and
`result` describe the lifecycle transaction. The remote transport currently
provides status, switch, stop and operation lookup. Stopping requires the exact
expected Product session and runs the existing ProductControl stop transaction.
Switch and stop operations share their durable request ledger and execution queue.

Stopping the Product can also stop its Gateway. Supply a stable `request_id`
before submission. A disconnected response leaves the stop result unconfirmed.
The independent control service retains the operation. Query it through the
Gateway when that endpoint becomes available again, or through its local
ProductControl transport.

`navigation.goal_status(request_id)` reconciles a goal whose submission reply
was lost. `system.resume_motion()` releases the native motion hold. Its original
receipt retains `previous_motion_restored=false`. Task continuation uses the
existing task-scoped `navigation.resume(task_id)` operation.

`observations.recordings()` and `recording()` expose saved sessions and their
declared files. Download an individual file or the archive, inspect a summary,
or read historical series, 3D scenes and camera frames. `topics=[]` can select
camera-only capture with the corresponding recording profile. A historical
camera query with no frame returns empty bytes. The recording owner decides
whether a session can be deleted.

Python `stop_recording(expected_session_id=...)`, the C++ overload
`stop_recording(session_id)`, and Rust `stop_recording_session(session_id)` stop
the selected recording. A stale identity retains the owner's mismatch response
and leaves the current recording active. Omitting the identity in the original
stop method continues to stop the active recording.

`tasks` includes inspection route save/read/delete, task listing and evidence
lookup. Evidence review records a human finding. Artifact and measurement
downloads use the same atomic file writer as map and recording downloads.
Python inspection start/control methods also accept `expected_product_session_id`
and `expected_env` to carry the caller's selected runtime binding.

## Status events

Python offers synchronous and asynchronous SSE subscriptions. Stop a
subscription explicitly when leaving early.

```python
from contextlib import closing

with closing(robot.events(topic="/nav/state")) as events:
    for event in events:
        print(event.id, event.json())
        break
```

```python
from contextlib import aclosing
from lingtu.sdk import AsyncLingTuClient

async def read_one_event():
    async with AsyncLingTuClient("robot-host") as robot:
        async with aclosing(robot.events(topic="/nav/state")) as events:
            async for event in events:
                return event.json()
```

The Gateway normally emits a heartbeat every second. Choose a socket timeout
longer than that interval. Python asynchronous cancellation waits for the
active socket read to finish or reach its timeout, then closes the response.
Supply `last_event_id` for an explicit reconnect. Reconnection does not submit
a command.

## Live previews and operator sessions

Python and C++ expose `watch_cloud()`, `watch_scan()`, `watch_camera()` and
`teleop()`. Rust opens the same feeds with `websocket(channel, media_token)`
and `teleop(client_id)`. Constructing a Python session is lazy. Opening its
context connects. The native clients connect when the factory is called.

```python
with robot.watch_cloud() as frames:
    preview_bytes = next(frames)

with robot.watch_camera(viewer_token="viewer-token") as frames:
    jpeg = next(frames)
```

Install the optional Python sessions with the extra.

```sh
python -m pip install --force-reinstall "build/sdk/dist/lingtu_sdk-2.3.0-py3-none-any.whl[websocket]"
```

The WebSocket routes accept their configured preview sizes, including messages
above 10 MiB. The separate gRPC preview limit remains part of that transport's
contract. Frame bytes pass through the SDK without point-cloud resampling or
image decoding.

An operator session requests a fresh one-use input window for each explicit
velocity sample. Python and C++ call `move()`, and Rust calls `move_once()`.
The original `ingress_ack` identifies a queued command. Native output and motor
confirmation retain their own fields. `hold()` sends deadman release. A timeout,
cancellation or explicit close tears down the session, allowing the Gateway's
existing disconnect logic to hold and release its owner.

Media endpoints use an independent Bearer viewer token. Python camera methods
accept `viewer_token`. C++ accepts the token as its fourth constructor argument,
and Rust uses `with_media_token()`. Per-feed camera tokens can override the
stored native-client token. API-key and media credentials retain their distinct
headers.

The public gRPC schemas also describe streaming RPCs for consumers using their
language's gRPC toolchain.

## Observations and body commands

`search_memory_embedding()` requires a query vector from the encoder used by
the observation store. Text `search_memory()` reports that a matching text
encoder is required. The Gateway currently exposes vector search.

Automatic text search still needs a declared model and checkpoint shared by
observation writing and query encoding. Historical temporal rows store optional
vectors without that identity or original image crops. Equal vector dimensions
alone cannot establish model compatibility. Supporting those rows requires an
explicit treatment of their unknown origin.

Explicit stand, sit, motor enable/disable, motion stop and profile requests
share the same command processing used by gRPC. Retain `request_id` when
reconciling an unconfirmed submission. Native readiness, motion authority,
emergency-stop and driver decisions still determine admission and execution.

## Verification boundaries

SDK tests exercise request construction, actual Gateway HTTP schemas and
routes, errors, identity, downloads and stream lifetime. Native-owner adapters
are substituted in local contract tests. Read-only examples query health.
Simulation task completion and field motion require their own named-platform
evidence.

The gRPC source was recovered from the current generated descriptors. The
protocol check compiles it and compares all public definitions, including
error messages, optional fields, oneofs and streaming directions.

## Remaining work

- Text-memory queries need a declared encoder and checkpoint shared by writers
  and queries, plus a policy for historical vectors with unknown model identity.
- C++ and Rust currently use WebPKI roots for WSS. Private-CA configuration is
  still missing. Python HTTPS/WSS with normal verification has local positive
  evidence using a process-local test CA.
- SDK-to-native MuJoCo completion, NX/aarch64, native successful authenticated WSS,
  field validation and remote release publication need their own evidence.
