# Gateway Routes

Route files register HTTP, WebSocket, and SSE endpoints on the Gateway FastAPI
app. Keep route handlers thin: parse input, call a service/helper, return a
response.

| File | Role |
| --- | --- |
| `app.py` | FastAPI app factory and route registration |
| `commands.py` | Navigation, stop, cancel, mode command endpoints |
| `maps.py` | Map listing, selection, save/build/delete endpoints |
| `status.py` | Runtime health and module status endpoints |
| `realtime.py` | SSE and websocket realtime streams |
| `camera.py` | Camera snapshot and stream endpoints |
| `session.py` | Session lifecycle endpoints |
| `operations.py` | Operator actions such as restart/toggle endpoints |
| `diagnostics.py` | Diagnostic and evidence endpoints |
| `assets.py` | Static asset serving |
| `auth.py` | Auth endpoint helpers |

If a route starts making navigation, SLAM, or map lifecycle decisions itself,
move that logic to the owning module/service and keep the route as a caller.
