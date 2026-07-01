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
| dashboard templates and static assets | `templates/`, `map_dashboard.py` |
| optional visualization bridge | `rerun_bridge_module.py` |
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
  -> nav.mission.goal_pose
  -> global/local planning chain
```

## Folder Map

| Path | Role |
| --- | --- |
| `routes/` | FastAPI route registration and thin request handlers |
| `services/` | Shared route helpers such as status, goal building, map safety, traffic |
| `tests/` | Gateway-owned tests |
| `templates/` | Dashboard HTML templates |

Gateway maintenance scripts live in `scripts/gateway/`, not inside the Python
package.

## Boundary Rule

Gateway may ask other modules for work or state. Gateway should not import
planner, SLAM, driver, or perception internals to do the work itself. Put that
logic in the owning package and expose it through Module ports, skills, or
runtime status.
