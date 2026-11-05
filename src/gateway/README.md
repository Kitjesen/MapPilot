# Gateway

`src/gateway/` is the outside-facing interface layer.

It exposes LingTu to dashboards, tools, frontend code, and remote agents. It
turns external requests into Module inputs, and turns Module state into API or
status responses. It should not decide navigation policy or run planning
algorithms.

## What It Owns

| Area | Files |
| --- | --- |
| Host ports and runtime state | `gateway_module.py` |
| HTTP app construction and registration | `app_factory.py` |
| navigation commands, tasks, and status | `navigation/` |
| other HTTP, WebSocket, SSE routes | `routes/` |
| MCP tool server | `mcp_server.py` |
| request and response schemas | `schemas.py` |
| shared transport, control receipts, and cross-domain snapshots | `services/` |
| inspection HTTP surface | `routes/inspection.py` plus native inspection/evidence service adapters |
| dashboard templates and static assets | `templates/` |
| separate MCP server authentication | `auth.py` |

The Web dashboard, main Gateway HTTP API, SSE and WebSocket streams open
directly without an API Key or login cookie. Existing `LINGTU_API_KEY`,
`LINGTU_MAP_API_KEY` and `LINGTU_GATEWAY_REQUIRE_API_KEY` settings do not gate
these routes. `/api/v1/auth/login` and `/api/v1/auth/check` are removed.
The separate MCP server keeps its own authentication policy.

This removes only the Web access gate. Product readiness, native motion
authority, obstacle checks, command arbitration and stop handling still apply
to every motion request.

## Request Flow

```text
Frontend / CLI / MCP client
  -> Gateway route or MCP tool
  -> gateway service helper
  -> Module port or skill call
  -> navigation, map, semantic, or safety module
  -> status/event response back through Gateway
```

Example goal flow:

```text
map click
  -> navigation/routes.py
  -> navigation/goals.py + commands.py
  -> PoseStamped goal
  -> nav.goals
  -> nav.commands
  -> native navigation endpoint
```

Example inspection flow:

```text
inspection route command
  -> Gateway inspection route
  -> typed inspection command
  -> native navigation endpoint
  -> inspection status/evidence response
```

Saved-location goals are references, not just cached coordinates. Preview
accepts `location_name`; the existing Web submit contract uses
`source: saved_location` and `metadata.location_name`. Both resolve the stored
entry and require its map ID, content epoch, and frame to match native
navigation state. Submission resolves again inside command execution, so a
successful preview does not authorize an entry that was deleted or became
stale. The command returns `invalid_goal` with the specific binding failure
and does not dispatch a goal. Lookup uses the exact saved name: deleting `dock`
must not redirect its navigation request to `dock-east`. Fuzzy interpretation
belongs to semantic input, not this structured target reference. This does not
replace native motion admission.

The Web's save/update-current-location actions use `use_current_pose: true`.
The backend supplies XYZ and yaw from one odometry snapshot; the browser does
not replace Z with zero or reuse the old location height. The snapshot must be
in the map frame, received within the existing 2-second pose freshness limit,
and not quarantined or reported lost/stale by localization. Rejection leaves
the saved entry unchanged. New unbound locations can be saved for inspection,
but cannot be submitted as navigation targets. An existing bound location
cannot be overwritten while its new map binding is unavailable.

The MCP `tag_location` tool calls the same `maps.locations.upsert_location`
operation and uses the Gateway pose snapshot. It does not write directly from
its separate telemetry cache. Missing Gateway, stale pose, and persistence
failure return errors rather than claiming a location was saved. The SDK
deletes locations using HTTP DELETE with an encoded location name.

## Folder Map

| Path | Role |
| --- | --- |
| `navigation/` | Navigation-facing commands, queries, and state projection |
| `maps/` | Map HTTP APIs, mapd transport, map-layer projection, and location APIs |
| `routes/` | Other FastAPI route registration and request handlers |
| `services/` | Shared control receipts, snapshots, transport, and remaining domain helpers |
| `../../tests/gateway/` | Gateway-owned tests |
| `templates/` | Dashboard HTML templates |

## Navigation: start here

| Question | Owner |
| --- | --- |
| Where are goal, cancel, pause/resume, and status HTTP endpoints? | `navigation/routes.py` |
| How is a coordinate, map click, or saved location converted to a goal? | `navigation/goals.py` |
| How do requests reach the assembled navigation capability and validate ACK identity? | `navigation/commands.py` |
| What does the latest cached navigation status suggest? | `navigation/status.py` |
| How do those facts become task/admission/control/motion axes? | `navigation/projection.py` |
| How is an exact task or request queried, including retained results? | `navigation/tasks.py` |
| Where are planned paths and raw native execution evidence exposed? | `navigation/diagnostics.py` |

`status.py` captures runtime facts, estimates readiness, and passes that evaluated
snapshot to the pure projector. HTTP status, SSE callbacks, the full state
snapshot, and readiness use the same evaluation/projection functions. No client
surface maintains its own navigation state machine.

Goal commands use the native acknowledgement for acceptance; cached readiness
and safety status are informational. An unconfirmed native acknowledgement is
reported as an unknown outcome, not a rejection. The Gateway retains its page
control lease and explicit map-name or saved-location binding, which is not in
the goal wire format.

Public URLs remain unchanged, including `/api/v1/goal`, `/api/v1/navigate/click`,
and `/api/v1/navigation/*`. Task resume continues a specific task; motion resume
only releases a control hold. Neither an accepted command nor quiet odometry
proves a confirmed stop.

Cross-command leases, retry deduplication, and safety checks stay shared in
`services/commands.py` and `services/control_commands.py`. Native control
transport stays in `services/native_control.py`. These are reused by navigation,
teleoperation, and other operator commands; they are not separate task owners.

## Maps and status: start here

| Question | Owner / public entry |
| --- | --- |
| List, save, edit, download, or inspect a saved-map operation? | `maps/routes.py`: `/api/v1/slam/maps`, `/api/v1/map/save`, `/api/v1/maps/*` |
| How does Gateway talk to mapd? | `maps/transport.py`: stateless request adaptation and artifact descriptors |
| Which environment layers are available and fresh? | `maps/status.py`, exposed at `/api/v1/maps/environment/layers` |
| Add a tagged navigation location bound to the active map? | `maps/locations.py`: `/api/v1/locations` |
| Provision or resolve a semantic place? | `maps/places.py`: `/api/v1/places` |
| Inspect aggregate state, scene, or localization? | `routes/status.py`: `/api/v1/state`, `/api/v1/scene_graph`, `/api/v1/localization/status` |
| Is the process live, ready, or healthy? | `routes/health.py`: `/health`, `/ready`, `/api/v1/health`, `/api/v1/readiness`, `/api/v1/metrics` |
| Inspect topic observations and plan an SSE subscription? | `routes/diagnostics.py`: `/api/v1/runtime/dataflow*` |
| Receive live updates? | `routes/realtime.py`: `/api/v1/events` and `/ws/*` |

Tagged locations and semantic places remain distinct existing stores; sharing a
directory does not merge their identities or persistence. Native `mapd` owns
map state, save jobs, and artifacts. ProductControl owns active-map switching.
Gateway validates the HTTP boundary and projects those owners' results.

`services/native_status.py` only reads native snapshots. Consumers still decide
freshness and evidence validity; reading JSON is not proof of readiness or a
confirmed stop. `app_factory.py` registers each route owner once. The generated
[API reference](../../docs/api.md) lists the public routes and source owners.

Map requests use `maps/transport.py::mapd_request`; there are no separate query
and command aliases. Session, Viewer, and command consumers call its map helpers
directly instead of routing back through GatewayModule methods. Request argument
conversion remains in the transport; `maps/routes.py` maps transport failures to
HTTP 503. Active-map switching remains outside this Gateway interface.

## Web Teleop Input Freshness

`/ws/teleop` carries operator intent, never proof of motor execution. Before a
nonzero/deadman velocity sample, request an input window with
`{"type":"input_request","request_id":"..."}`. Echo the `input_window` from
`input_ack` in the next velocity sample. Each window is single-use and expires
after 350 ms on the Gateway's monotonic clock; an accepted sample's
`ingress_ack` supplies the next window. This does not require browser/NX clock
synchronization. The deadline also covers native claim and publisher queuing.

The Web client permits only one outstanding input and never sends a stored
velocity when an ACK arrives. Its next timer tick reads current keys. A
`deadman:false` hold bypasses flow control and needs no input window; release
invalidates late input ACKs. Missing ACKs close the connection and clear held
input. Old browser bundles without input windows are rejected: deploy Gateway
and Web together and refresh operator pages. Native motion arbitration and the
driver watchdog remain the final command owners.

## Camera Transport

Scene's default-open camera preview and Console `CameraFeed` share the WHEP
lifecycle. Both show JPEG from `/ws/camera` until a WebRTC frame is decoded,
then display H.264. Opening or closing a preview manages browser subscriptions;
it does not start a Product, a camera process or recording.

The `map/camera` and `nav/camera` Products explicitly own `lt-video.service`.
Their RunPlan/session provides `LINGTU_WEBRTC_URL` and `LINGTU_VIDEO_STATUS_FILE`.
The native program reads existing color SHM and owns encoding/media; Gateway
only proxies SDP, status and session deletion. Video startup/readiness may
degrade without rolling back healthy core processes. Camera acquisition and
Follow observation requirements are unchanged.

`GET /api/v1/webrtc/status` separates `whep_available` from `source_ready`.
Managed runs require a fresh native status snapshot as well as a reachable
WHEP endpoint before reporting `available=true`. A foreground reference without
a status-file setting returns `source_ready=null`; endpoint availability alone
does not establish fresh camera evidence. Actual browser decoding remains a
separate client fact. No viewer is required for Product activation.

The real JPEG path is native camera → SHM frame ring → Host camera Module →
`CameraJpegRelayModule` → Gateway latest-frame cache → `/ws/camera` → `useCamera`.
RGB-D pixels use SHM at the native/Host boundary; the module named
`DdsCameraModule` does not mean raw browser video is sent over DDS or WebRTC.
Depth and camera intrinsics stay in the perception path, not in the browser JPEG.

The relay defaults to JPEG quality 60 and a 10 FPS encoding limit, without
resizing the source frame. It encodes for connected media clients and keeps
only the latest raw frame. Gateway checks its latest JPEG every 100 ms, skips
an unchanged sequence and limits each socket send to two seconds. The browser
decodes before displaying, drops arriving frames while decoding, reconnects
after three seconds, and clears video when no fresh decoded frame has arrived
for three seconds. These are configured limits, not measured field performance;
TCP buffering can still add delay on a constrained connection.

| Endpoint | Payload / role |
| --- | --- |
| `/ws/camera` | Binary JPEG frames for Scene and the Console fallback |
| `GET /api/v1/camera/status` | Source availability, color/depth rates and stale age, JPEG cache metadata |
| `GET /api/v1/camera/snapshot` | One JPEG image |
| `GET /api/v1/webrtc/status` | Selected backend, WHEP/source availability, native snapshot, ICE and rotation |
| `GET /api/v1/webrtc/go2rtc/status` | Reference go2rtc availability |
| `POST /api/v1/webrtc/whep?src=cam` | SDP offer/answer signalling, not the video bytes |
| `DELETE /api/v1/webrtc/sessions/{resource_path}` | Release the returned media session |

Two current boundaries remain: snapshot prioritizes an existing JPEG cache
before checking source freshness, so callers needing current evidence must
also inspect camera status; and bare WebSocket JPEG messages do not carry the
capture timestamp, so browser receipt age does not measure end-to-end latency.

For managed video, configure the target's exact encoder and prebuilt runtime
using `config/video.env.example`, then use the existing ProductControl entry.
Inspect media status and `lt-video.service` logs if unavailable. A stale source
withdraws old media and waits for fresh frames; fatal errors remain visible.
The native process does not restart the camera, navigation or Host.

`config/go2rtc.yaml` remains an Annex-B reference consumer of the same color
SHM branch, not another physical camera owner. It is not needed by the native
WHEP chain. Snapshot clients continue to use `GET /api/v1/camera/snapshot`.

## Boundary Rule

Gateway may ask other modules or native endpoint adapters for work or state.
Gateway should not import planner, SLAM, driver, inspection executor, or
perception internals to do the work itself. Put that logic in the owning package
and expose it through Module ports, typed commands, skills, or runtime status.

## Nearby BLE discovery

On a Linux robot with BlueZ `bluetoothctl`, an already powered LE controller,
advertising and GATT support, and permission to register both, set the existing
`DOSO_DEVICE_ID` to the device's canonical UUID v4 and run
`PYTHONPATH=src python -m gateway.discovery`. The command keeps its BlueZ
service and advertisement alive until Ctrl+C or SIGTERM. Registration has a
10-second deadline and success is reported only after BlueZ accepts both.
Stopping the process closes its D-Bus client and removes its registrations.

The shared discovery service UUID is
`81343a98-47b3-4a0b-af0c-d42d87c7070f` and the local name is `DOSO`. The short
name keeps the packet within the legacy 31-byte budget. Its read-only GATT
characteristic `44cecaf8-0fab-4594-96f9-6e864f1dedf2` returns the 16 bytes of
the configured UUID in network byte order. This public value identifies an
unverified candidate: it is not a manufacturing signature or account binding.
The command changes no adapter name, power, pairing or discovery settings and
has no automatic startup integration. BlueZ 5.64 command behavior was checked
against its official source. Robot radio behavior still requires field verification.
