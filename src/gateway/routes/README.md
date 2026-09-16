# Gateway Routes

Route files register HTTP, WebSocket, and SSE endpoints on the Gateway FastAPI
app. Keep route handlers thin: parse input, call a service/helper, return a
response.

| File | Role |
| --- | --- |
| `app.py` | Client bootstrap, capabilities, and traffic endpoints |
| `../navigation/routes.py` | Navigation goals, task control, and status queries |
| `commands.py` | Shared operator commands: stop, mode, lease, instruction, visual servo |
| `../maps/` | Map operations, map layers, tagged locations, and semantic places |
| `../navigation/diagnostics.py` | Planned paths and native navigation evidence |
| `status.py` | Read-only aggregate state, scene graph, and localization queries |
| `health.py` | Health, metrics, liveness, and readiness probes |
| `realtime.py` | SSE and websocket realtime streams |
| `camera.py` | Camera snapshot and stream endpoints |
| `session.py` | Session lifecycle endpoints |
| `operations.py` | Operator actions such as restart/toggle endpoints |
| `diagnostics.py` | Diagnostic evidence and read-only runtime dataflow inspection |
| `inspection.py` | Inspection route commands, status, and evidence-facing endpoints |
| `assets.py` | Static asset serving |
| `auth.py` | Auth endpoint helpers |

If a route starts making navigation, SLAM, or map lifecycle decisions itself,
move that logic to the owning module/service and keep the route as a caller.

`../app_factory.py` registers each route owner explicitly. Do not register domain
routes again from `commands.py` or `status.py`. Aggregate state has no location
writes, health probes, or stream transport implementation.
