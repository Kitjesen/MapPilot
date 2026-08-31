# Gateway

`src/gateway/` is the outside-facing interface layer.

It exposes LingTu to dashboards, tools, frontend code, and remote agents. It
turns external requests into Module inputs, and turns Module state into API or
status responses. It should not decide navigation policy or run planning
algorithms.

## What It Owns

| Area | Files |
| --- | --- |
| HTTP, WebSocket, SSE server | `gateway_module.py`, `routes/` |
| MCP tool server | `mcp_server.py` |
| request and response schemas | `schemas.py` |
| route helper logic | `services/` |
| inspection HTTP surface | `routes/inspection.py` plus native inspection/evidence service adapters |
| dashboard templates and static assets | `templates/` |
| auth helpers | `auth.py` |

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
  -> Gateway goal endpoint
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

## Folder Map

| Path | Role |
| --- | --- |
| `routes/` | FastAPI route registration and thin request handlers |
| `services/` | Shared route helpers such as status, goal building, map safety, traffic |
| `tests/` | Gateway-owned tests |
| `templates/` | Dashboard HTML templates |

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
