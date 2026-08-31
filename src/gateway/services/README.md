# Gateway Services

Service files hold small helpers used by Gateway routes. They should convert
Gateway-facing requests/status into module-facing data, not own navigation or
SLAM behavior.

| File | Role |
| --- | --- |
| `app_bootstrap.py` | Startup payloads and frontend bootstrap state |
| `cloud_viewer.py` | Live/saved/scan point-cloud viewer buffering and throttling |
| `cloud_ws.py` | Binary point-cloud WebSocket helpers |
| `commands.py` | Command idempotency, control lease, and client policy helpers |
| `control_commands.py` | Teleop/control command helpers |
| `drift.py` | SLAM drift watchdog classification and recovery |
| `event_handlers.py` | Module callback to Gateway state/SSE event serialization |
| `exploration.py` | Native DDS exploration status, readiness, and start/stop adapters |
| `goal_builder.py` | Goal request to typed pose/message conversion |
| `init_state.py` | GatewayModule process-local state initialization helpers |
| `lifecycle.py` | Gateway background thread startup and shutdown helpers |
| `loc_cache.py` | Localization state cache |
| `localization_status.py` | Localization status normalization and SSE forwarding |
| `cloud_scene_cache.py` | Transient browser point-cloud cache; not a saved-map store |
| `map_service.py` | Stateless UDS transport from Gateway HTTP requests to native C++ `mapd` v2 actions |
| `media_status.py` | Camera/media status helpers |
| `module_refs.py` | Required Host module reference attachment |
| `native_control.py` | Native endpoint control helpers for command/status surfaces |
| `odometry.py` | Odometry validation, cache update, and SSE forwarding |
| `pose_recovery.py` | Last-pose persistence, auto relocalization, map->odom TF, and reset helpers |
| `readiness.py` | Readiness summaries |
| `loc_cache.py` | Odometry and map/odom transform cache |
| `runtime_dataflow.py` | Read-only Gateway observability; never motion orchestration |
| `runtime_status.py` | Module/runtime status aggregation |
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
