# scripts/ - LingTu Operations Entrypoints

This directory is for long-lived build, deployment, robot operations, runtime
gate, and diagnostic entrypoints. Offline analysis and developer-only helpers
belong under `tools/`; manual smoke checks belong under `tests/scripts/smoke/`.

Prefer `python lingtu.py` for normal local use. Use scripts here only when a
shell entrypoint is needed for robot operations, systemd, OTA, CI, or field
deployment.

## Common Commands

| Task | Command |
| --- | --- |
| Start navigation | `python lingtu.py thunder-nav` |
| Start mapping | `python lingtu.py thunder-map` |
| Start exploration | `python lingtu.py thunder-explore` |
| Environment diagnostics | `python lingtu.py doctor` |
| Robot-side operations CLI | `scripts/lingtu health` / `scripts/lingtu status` |
| Thunder field deploy | `bash scripts/deploy/deploy_thunder.sh` |
| Install Thunder LCM endpoint service | `bash scripts/deploy/thunder/install_services.sh lcm-endpoint` |
| Run Thunder field endpoint source group | `python scripts/deploy/thunder/run_lcm_endpoint_service.py --source thunder_field` |
| Run Thunder Brainstem endpoint source | `python scripts/deploy/thunder/run_lcm_endpoint_service.py --source thunder_brainstem` |
| Run no-ROS JSONL localization source | `LINGTU_ENDPOINT_JSONL_PATH=/data/thunder/localization.jsonl python scripts/deploy/thunder/run_lcm_endpoint_service.py --source thunder_field` |
| Validate Thunder JSONL endpoint feed | `python tools/validate/validate_lcm_jsonl_feed.py /data/thunder/localization.jsonl --require-field-inputs` |
| Smoke Thunder endpoint without ROS/LCM | `python scripts/deploy/thunder/run_lcm_endpoint_service.py --transport local --source smoke --once --json` |
| Install Thunder Lite service | `bash scripts/deploy/thunder/install_services.sh lite` |
| Install legacy ROS compatibility services | `bash scripts/deploy/thunder/install_services.sh ros-compat` |
| Legacy deploy alias | `bash scripts/deploy/deploy_s100p.sh` |
| Run tests | `bash scripts/dev/run_tests.sh` |
| Validate Thunder field deployment | `python tools/validate/validate_thunder_field_deployment.py` |
| Validate config | `python tools/validate/validate_config.py` |
| Validate topics | `python tools/validate/validate_topics.py` |
| OTA package and deploy | `bash scripts/ota/build_nav_package.sh && bash scripts/ota/deploy_to_robot.sh` |
| Generate protobuf files | `bash scripts/proto/proto_gen.sh` |
| Rerun visualization | `python lingtu.py rerun` |

## Root Entrypoints

| Script | Purpose |
| --- | --- |
| `lingtu` | Robot-side operations CLI: status, watch, map, nav, svc, log, health. |
| `lingtu.sh` | Shell compatibility wrapper around `python lingtu.py`. |

## Directory Map

| Directory | Purpose | Representative files |
| --- | --- | --- |
| `build/` | Native component and third-party builds | `build_nav_core.sh`, `build_dufomap.sh`, `build_ros_workspace.sh`, `build_tare.sh` |
| `deploy/` | Thunder field deployment, version switching, network, systemd services | `deploy_thunder.sh`, `setup_server_ros_pct.sh`, `sync_versions.sh`, `thunder/install_services.sh` |
| `ota/` | OTA package, push, install, and service startup | `build_nav_package.sh`, `deploy_to_robot.sh`, `generate_manifest.py` |
| `monitor/` | Feishu / Telegram status bots | `feishu_monitor_bot.py`, `telegram_monitor_bot.py` |
| `proto/` | Protobuf code generation | `proto_gen.sh`, `proto_gen.ps1` |
| `manager/` | Lightweight web management service | `manager.py` |
| `diagnostics/` | Local and robot diagnostics | `doctor.py`, `dufomap_offline_test.py`, `soak.py`, `static_localization_probe.py` |
| `gates/` | Runtime contracts, saved-map gates, and field evidence gates | `runtime_contract_audit.py`, `saved_map_artifact_gate.py`, `real_runtime_evidence_collect.py` |
| `planning/` | Planning preview and gate helpers | `plan_preview.py` |
| `perception/` | Live perception demos and tracking tools | `live_detect.py`, `live_track.py` |
| `visualization/` | Rerun visualization entrypoints | `rerun_live.py`, `run_rerun_mapping.py` |
| `webrtc/` | go2rtc/WebRTC helpers | `install_go2rtc.sh` |
| `hardware/` | Sensor bagging and IMU calibration helpers | `record_bag.sh`, `run_allan_variance.sh` |
| `docs/` | Documentation generation scripts | `extract_api_docs.py` |
| `dev/` | Developer convenience scripts | `run_tests.sh`, `scaffold_robot.py` |

## Rules

- New deployment, systemd, OTA, and robot operations scripts go under the
  matching `scripts/` subdirectory.
- New offline analysis, BPU export, and one-off visualization tools go under
  `tools/` unless they are durable operator entrypoints.
- Pytest-collected tests go under `tests/**/test_*.py`.
- Manual hardware or ROS smoke checks go under `tests/scripts/smoke/`.
- Keep product-facing field entrypoints named Thunder. Legacy board/path names
  may remain only as compatibility wrappers or stable robot-side contracts.
- Prefer `--endpoint thunder-field` or `--endpoint field` in operator-facing
  commands. `real_s100p` and `s100p` are compatibility aliases only.
- Prefer `--robot thunder` or `--robot thunder_remote` for physical robot
  presets. `s100p` and `navigate` are compatibility aliases only.
- Thunder service installation defaults to the no-ROS LCM endpoint path. Use
  `LINGTU_COMMAND_OUTPUT_MODE=endpoint_only` for field navigation so the
  endpoint source, not the LingTu module graph, owns hardware actuation. Use
  `LINGTU_ENDPOINT_SOURCES` for comma-separated field endpoint plugins, such
  as the `thunder_field` product source group. That group expands to
  Brainstem command sinking and automatically adds a `jsonl`
  localization/sensor provider when `LINGTU_ENDPOINT_JSONL_PATH` or
  `LINGTU_ENDPOINT_JSONL_COMMAND` is configured.
- Validate JSONL providers with
  `python tools/validate/validate_lcm_jsonl_feed.py <feed.jsonl> --require-field-inputs`
  before deploying them as a Thunder field endpoint source. The validator uses
  the same LCM endpoint contract and runtime frame contract as the runner.
- Validate the field deployment boundary with
  `python tools/validate/validate_thunder_field_deployment.py` before claiming
  Thunder navigation is endpoint-only/no-ROS by default.
  `ros-compat` only for the old ROS service chain.
