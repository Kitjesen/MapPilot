# Gateway Services

Service files hold small helpers used by Gateway routes. They should convert
Gateway-facing requests/status into module-facing data, not own navigation or
SLAM behavior.

| File | Role |
| --- | --- |
| `app_bootstrap.py` | Startup payloads and frontend bootstrap state |
| `commands.py` | Command dispatch helpers |
| `control_commands.py` | Teleop/control command helpers |
| `goal_builder.py` | Goal request to typed pose/message conversion |
| `map_paths.py` | Map path resolution |
| `map_safety.py` | Gateway-side map input validation |
| `media_status.py` | Camera/media status helpers |
| `readiness.py` | Readiness summaries |
| `runtime_dataflow.py` | Runtime dataflow status summaries |
| `runtime_status.py` | Module/runtime status aggregation |
| `runtime_switch_plan.py` | Safe runtime switch preview |
| `safety_status.py` | Safety status aggregation |
| `state_snapshot.py` | Diagnostic state snapshot |
| `telemetry_normalizers.py` | Telemetry shape/unit normalization |
| `traffic.py` | SSE/cloud traffic accounting |

Map save/build execution belongs in `nav.services.maps`; Gateway should expose
it, not become a second map owner.
