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
| `driver_swap.py` | Runtime driver swap request handling |
| `drift.py` | SLAM drift watchdog classification and recovery |
| `event_handlers.py` | Module callback to Gateway state/SSE event serialization |
| `exploration.py` | Exploration backend status, readiness, and start/stop adapters |
| `goal_builder.py` | Goal request to typed pose/message conversion |
| `http_prewarm.py` | First-use HTTP route prewarm thread and request logic |
| `init_state.py` | GatewayModule process-local state initialization helpers |
| `lifecycle.py` | Gateway background thread startup and shutdown helpers |
| `loc_cache.py` | Localization state cache |
| `localization_status.py` | Localization status normalization and SSE forwarding |
| `cloud_scene_cache.py` | Transient browser point-cloud cache; not a saved-map store |
| `map_service.py` | Maps module service-contract adapter |
| `media_status.py` | Camera/media status helpers |
| `module_refs.py` | Runtime module discovery and reference attachment |
| `native_control.py` | Native endpoint control helpers for command/status surfaces |
| `odometry.py` | Odometry validation, cache update, and SSE forwarding |
| `pose_recovery.py` | Last-pose persistence, auto relocalization, map->odom TF, and reset helpers |
| `readiness.py` | Readiness summaries |
| `runtime_cache.py` | Runtime status cache primitives |
| `runtime_dataflow.py` | Runtime dataflow status summaries |
| `runtime_status.py` | Module/runtime status aggregation |
| `runtime_switch_execute.py` | Safe runtime switch execution |
| `runtime_switch_plan.py` | Safe runtime switch preview |
| `saved_map_loader.py` | Active saved-map point loading and SSE push loop |
| `safety_status.py` | Safety status aggregation |
| `server.py` | Uvicorn server runner |
| `session_cache.py` | Product/session runtime cache |
| `session_view.py` | Product session snapshot and current-mode detection |
| `slam_profile.py` | Current SLAM backend/profile and odometry-derived Hz helpers |
| `sse.py` | Server-sent event queues, event ids, drop accounting, and raster throttling |
| `state_snapshot.py` | Diagnostic state snapshot |
| `subscriptions.py` | Gateway stream subscription wiring |
| `teleop.py` | Gateway-side teleop transport state, UDP bridge, and typed DDS request publish |
| `telemetry_normalizers.py` | Telemetry shape/unit normalization |
| `traffic.py` | SSE/cloud traffic accounting |
| `viewer_events.py` | Viewer SSE serialization for paths, costmap, and slope grid |

Map save/build execution belongs in `maps.modules.service`; Gateway should expose
it, not become a second map owner.
