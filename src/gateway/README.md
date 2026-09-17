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
| Can a goal be admitted, and what native evidence is available? | `navigation/status.py` |
| How do those facts become task/admission/control/motion axes? | `navigation/projection.py` |
| How is an exact task or request queried, including retained results? | `navigation/tasks.py` |
| Where are planned paths and raw native execution evidence exposed? | `navigation/diagnostics.py` |

`status.py` captures runtime facts, evaluates admission, and passes that evaluated
snapshot to the pure projector. HTTP status, SSE callbacks, the full state
snapshot, and readiness use the same evaluation/projection functions. No client
surface maintains its own navigation state machine.

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

## Camera Transport

The dashboard prefers go2rtc WHEP for low-latency H.264 video and falls back
to Gateway JPEG-over-WebSocket at `/ws/camera`. The browser probes
`GET /api/v1/webrtc/go2rtc/status` before posting SDP to
`POST /api/v1/webrtc/whep`. Bootstrap metadata describes WHEP support; the
status endpoint reports whether the optional sidecar is currently available.

Install and check the sidecar with:

```bash
sudo bash scripts/deploy/thunder/install_go2rtc.sh
sudo systemctl restart go2rtc
curl -s http://localhost:5050/api/v1/webrtc/go2rtc/status
```

The template is `config/go2rtc.yaml`; it keeps the go2rtc API on loopback so
browsers use the Gateway proxy. Snapshot clients should continue using
`GET /api/v1/camera/snapshot`, which is independent of dashboard streaming.
If WHEP fails, inspect the status endpoint and `go2rtc` service log; for an
established peer with no video, verify the camera device and inspect
`chrome://webrtc-internals/`.

## Boundary Rule

Gateway may ask other modules or native endpoint adapters for work or state.
Gateway should not import planner, SLAM, driver, inspection executor, or
perception internals to do the work itself. Put that logic in the owning package
and expose it through Module ports, typed commands, skills, or runtime status.
