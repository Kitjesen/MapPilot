# LingTu API Docs

Generated API inventories. Regenerate them from source before relying on route
or tool counts.

## Index

| Doc | Description | Source |
|------|------|------|
| [MCP tools](mcp_tools.md) | `@skill` methods exposed through JSON-RPC MCP | Scans Module source files under `src/` |
| [Gateway REST API](gateway_rest.md) | Gateway REST/HTTP endpoints on port 5050 | Scans FastAPI route registrations under `src/gateway/routes/` |

## Regenerate

```bash
python scripts/docs/extract_api_docs.py
```

## Runtime boundary

```
L0  Safety    REST: /api/v1/stop, /api/v1/mode
L1  Hardware  native field services and the unique `lingtu-driver` speed exit
L2  Maps      REST: /api/v1/maps/*, /api/v1/slam/*
L3  Perception REST: /api/v1/scene_graph, /api/v1/camera/*
    Memory    REST: /api/v1/locations, /api/v1/memory/*
L4  Decision  REST: /api/v1/instruction, /api/v1/goal
L5  Planning  REST: /api/v1/navigation/*
L6  Interface REST: /api/v1/app/*, /api/v1/health
               MCP: http://<robot-ip-or-hostname>:8090/mcp
```

## Protocols

- **REST**: JSON over HTTP, Pydantic v2 request/response models, 422 on validation error
- **SSE**: `GET /api/v1/events` — `text/event-stream`
- **WebSocket**: `ws://<robot-ip-or-hostname>:5050/ws/teleop` for teleop and `ws://<robot-ip-or-hostname>:5050/ws/camera` as the JPEG camera fallback
- **WHEP**: `POST /api/v1/webrtc/whep` proxies low-latency camera signalling to the configured go2rtc sidecar
- **MCP**: `http://<robot-ip-or-hostname>:8090/mcp` — JSON-RPC 2.0
