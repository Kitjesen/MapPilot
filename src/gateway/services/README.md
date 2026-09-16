# Gateway Services

Services hold shared Gateway transport, control receipts, and cross-domain
snapshots. Navigation-specific HTTP routes, command adaptation, task queries,
and state projection live together in `../navigation/`. Map APIs, mapd transport,
and environment-layer projection live in `../maps/`. Native navigation, mapd,
and SLAM remain the execution owners.

| File | Role |
| --- | --- |
| `app_bootstrap.py` | Startup payloads and frontend bootstrap state |
| `cloud_viewer.py` | Live/saved/scan point-cloud viewer buffering and throttling |
| `cloud_ws.py` | Binary point-cloud WebSocket helpers |
| `commands.py` | Command idempotency, control lease, and client policy helpers |
| `control_commands.py` | Teleop/control command helpers |
| `drift.py` | SLAM drift watchdog classification and recovery |
| `event_handlers.py` | Non-navigation Module callback serialization; navigation callbacks live in `../navigation/status.py` |
| `exploration.py` | Native DDS exploration status, readiness, and start/stop adapters |
| `init_state.py` | GatewayModule process-local state initialization helpers |
| `lifecycle.py` | Gateway background thread startup and shutdown helpers |
| `loc_cache.py` | Localization state cache |
| `localization_status.py` | Localization status normalization and SSE forwarding |
| `cloud_scene_cache.py` | Transient browser point-cloud cache; not a saved-map store |
| `media_status.py` | Camera/media status helpers |
| `module_refs.py` | Required Host module reference attachment |
| `native_control.py` | Native endpoint control helpers for command/status surfaces |
| `native_status.py` | Shared raw native snapshot reads; no freshness or permission inference |
| `odometry.py` | Odometry validation, cache update, and SSE forwarding |
| `pose_recovery.py` | Last-pose persistence, auto relocalization, map->odom TF, and reset helpers |
| `readiness.py` | Readiness summaries |
| `runtime_dataflow.py` | Read-only Gateway observability; never motion orchestration |
| `runtime_facts.py` | Capture one lock-consistent set of runtime facts for state consumers |
| `runtime_status.py` | Localization projection, session context, and runtime identity; no navigation gate or task projection |
| `safety_status.py` | Safety status aggregation |
| `server.py` | Uvicorn server runner |
| `session_view.py` | Read-only Product run projection from RunPlan, mapd, and native status |
| `slam_profile.py` | Current SLAM backend/profile and odometry-derived Hz helpers |
| `sse.py` | Server-sent event queues, event ids, drop accounting, and raster throttling |
| `state_snapshot.py` | Diagnostic state snapshot |
| `subscriptions.py` | Gateway stream subscription wiring |
| `teleop.py` | Gateway-side teleop transport state, UDP bridge, and typed DDS request publish |
| `telemetry_normalizers.py` | Telemetry shape/unit normalization |
| `traffic.py` | SSE/cloud traffic accounting |
| `viewer_events.py` | Viewer SSE serialization for paths and native traversability |

Map state, save coordination, artifact builds, validation, and activation data
belong to the native C++ `mapd` process. Gateway only validates its HTTP seam and
forwards canonical `mapd` v2 actions; it does not host a Python map manager.
