# scripts/ - LingTu Operations Entrypoints

This directory is for long-lived build, deployment, robot operations, runtime
gate, and diagnostic entrypoints. Offline analysis and developer-only helpers
belong under `tools/`; manual smoke checks belong under `tests/scripts/smoke/`.

Prefer `python lingtu.py` for local development. `scripts/lingtu` is the sole
canonical field operations entrypoint. Its mode-switch subcommands remain a
thin adapter over `python -m lingtu.control switch`; status, recording, and
diagnostic subcommands may dispatch their declared native tools, but never own
Product policy, readiness, or map activation. Use the other scripts here only
for systemd, OTA, CI, deployment, or explicit diagnostics.

## Common Commands

| Task | Command |
| --- | --- |
| Start field navigation | `scripts/lingtu --env real mode switch nav --map <map>` |
| Start field mapping | `scripts/lingtu --env real mode switch map` |
| Start exploration | `scripts/lingtu --env real mode switch explore` |
| Explain resolved Product parameters | `scripts/lingtu --env real inspect nav --explain` |
| Explain current RunPlan and process modes | `scripts/lingtu status --explain` |
| Environment diagnostics | `scripts/lingtu doctor` or `python -m diagnostics.field.doctor` |
| Robot-side operations CLI | `scripts/lingtu health` / `scripts/lingtu status` / `scripts/lingtu doctor` |
| Record native DDS | `scripts/lingtu record` |
| Record native DDS and camera | `scripts/lingtu record --camera` |
| Thunder field deploy/build | `bash scripts/deploy/deploy_thunder.sh` |
| Deploy and activate Product | `LINGTU_DEPLOY_PRODUCT=nav LINGTU_DEPLOY_MAP=<map> bash scripts/deploy/deploy_thunder.sh` |
| Cut Thunder release | `bash scripts/deploy/cut_release.sh v2.1.1` |
| Package downloadable native release | `bash scripts/deploy/package_native_release.sh v2.1.1 dist` |
| Install Thunder field services | `bash scripts/deploy/thunder/install_services.sh field-cpp` |
| Build native Thunder driver | `bash scripts/build/build_driver.sh` |
| Run native Thunder driver | `build/driver/lingtu_driver` |
| Run compatibility Python DDS endpoint | `PYTHONPATH=src python -m runtime.endpoints.dds.endpoint_runner --source thunder` |
| Run compatibility Python Brainstem sink | `PYTHONPATH=src python -m runtime.endpoints.dds.endpoint_runner --source thunder_brainstem` |
| Run no-ROS JSONL localization source | `PYTHONPATH=src LINGTU_ENDPOINT_JSONL_PATH=/data/thunder/localization.jsonl python -m runtime.endpoints.dds.endpoint_runner --source thunder` |
| Smoke Thunder endpoint without hardware | `PYTHONPATH=src python -m runtime.endpoints.dds.endpoint_runner --transport local --source smoke --once --json` |

| Run tests | `bash scripts/dev/run_tests.sh` |
| Validate `env=real` deployment | `python tools/validate/validate_real_deployment.py` |
| Validate config | `python tools/validate/validate_config.py` |
| Validate topics | `python tools/validate/validate_topics.py` |
| Generate protobuf files | `bash scripts/proto/proto_gen.sh` |
| Rerun visualization | `python lingtu.py rerun` |

## Root Operations Entrypoint

| Script | Purpose |
| --- | --- |
| `lingtu` | Sole canonical field thin adapter: `scripts/lingtu` status, watch, map, nav, record, replay, svc, log, health, doctor. |

## Native Recording and Replay

The field CLI keeps a familiar short record/play workflow, but its
implementation is native C++/CycloneDDS/MCAP rather than ROS or rosbag2. The
canonical operator behavior, safety policy, and roadmap live in
[`docs/04-deployment/native_recording.md`](../docs/04-deployment/native_recording.md).

`scripts/lingtu record` starts a foreground, DDS-only session. Press Ctrl-C to finish it cleanly. Add
`--camera` only when color video is needed:

```bash
scripts/lingtu record
scripts/lingtu record --camera
```

Sessions default to
`$HOME/data/lingtu/recordings/<UTC-timestamp>-<pid>`; override the root with
`LINGTU_RECORDING_ROOT`. An explicit session directory can be the first
argument. The remaining native recorder options are forwarded unchanged.

The C++ manager requires 5 GiB free on the target filesystem at startup by
default. Override it with `--min-free-gib N`; `0` disables only the minimum
free-space threshold, while the filesystem capacity query remains mandatory.
Runtime quota enforcement is not implemented yet.

```bash
scripts/lingtu record status <session-dir>
scripts/lingtu record stop <session-dir>
scripts/lingtu record info <session-dir-or-mcap>
scripts/lingtu record topics
scripts/lingtu record verify <session-dir-or-mcap>
scripts/lingtu replay <session-dir-or-mcap>
```

`record verify` checks DDS integrity and also verifies `camera_color.mcap`
when present. Replay defaults to isolated DDS domain `84`. `bag` aliases
`record`; `play` is an alias, and native player options such as `--rate` or
an explicit `--domain` are forwarded. The shell owns only these operator
convenience defaults and path/argument adaptation. Native tools own session
integrity and replay safety; ProductControl does not own individual sessions.

## Explicit ROS2 Compatibility Diagnostics

The default field Product, `scripts/lingtu doctor`, and real-runtime evidence
collection use native services and Gateway data. ROS2 probes are opt-in
compatibility diagnostics only; they are never default Product paths.

| Diagnostic | Explicit compatibility command |
| --- | --- |
| ROS2 compatibility tool index | `scripts/compat/ros2/README.md` |
| Inspect the legacy ROS graph from field doctor | `scripts/lingtu doctor --ros2` |
| Collect evidence from the legacy ROS graph | `python scripts/gates/real_runtime_evidence_collect.py --collector ros2` |

Without those flags, doctor does not source a ROS setup and the evidence
collector uses `--collector gateway` by default.

## Directory Map

Field Products are selected through `scripts/lingtu`; `lingtu.py` remains the
local Host Profile entrypoint.

| Directory | Purpose | Representative files |
| --- | --- | --- |
| `build/` | Native component and third-party builds | `build_native_runtime.sh`, `build_nav_kernel.sh`, `build_dufomap.sh` |
| `compat/ros2/` | Explicitly quarantined ROS2-only developer and diagnostic tools | `setup_fastlio2_validation_host.sh`, `datasets/ros2_bag_to_normalized_jsonl.py`, `hardware/record_bag.sh`, `perception/live_detect.py` |
| `deploy/` | Thunder field deployment, version switching, network, systemd services | `deploy_thunder.sh`, `sync_versions.sh`, `thunder/install_services.sh` |
| `monitor/` | Feishu / Telegram status bots | `feishu_monitor_bot.py`, `telegram_monitor_bot.py` |
| `proto/` | Protobuf code generation | `proto_gen.sh`, `proto_gen.ps1` |
| `diagnostics/` | Offline and component diagnostics; field doctor lives in `src/diagnostics/field/doctor.py` | `dufomap_offline_test.py`, `soak.py`, `static_localization_probe.py` |
| `gates/` | Runtime contracts, saved-map gates, and field evidence gates | `runtime_contract_audit.py`, `saved_map_artifact_gate.py`, `real_runtime_evidence_collect.py` |
| `planning/` | Retired legacy PCT/tomogram preview area | none |
| `perception/` | Migration notice for retired ROS2 demo paths | `README.md` |
| `visualization/` | Gateway/native DDS Rerun visualization entrypoints | `rerun_gateway_live.py`, `rerun_live.py`, `run_rerun_mapping.py` |
| `webrtc/` | go2rtc/WHEP setup helper | `install_go2rtc.sh` |
| `hardware/` | Hardware calibration helpers | `run_allan_variance.sh` |
| `docs/` | Documentation generation scripts | `extract_api_docs.py` |
| `dev/` | Developer convenience scripts | `run_tests.sh`, `scaffold_robot.py` |

## Rules

- Extend the existing deployment, native-release, and robot-operations
  entrypoints. Do not add another lifecycle script tree beside ProductControl
  and `scripts/lingtu`.
- New offline analysis, BPU export, and one-off visualization tools go under
  `tools/` unless they are durable operator entrypoints.
- Pytest-collected tests go under `tests/**/test_*.py`.
- Manual hardware or ROS smoke checks go under `tests/scripts/smoke/`.
- Keep product-facing field entrypoints named Thunder. Legacy board/path names
  may remain only as compatibility wrappers or stable robot-side contracts.
- Operator-facing Product commands use `--env real|sim`; simulation backend
  choice is internal env config.
- Physical robot parameters come from `config/robot_config.yaml` through the
  real env. Local Profile driver backends are not public field selectors.
- Thunder service installation defaults to the no-ROS native DDS field path.
  Use `LINGTU_COMMAND_OUTPUT_MODE=endpoint_only` and
  `LINGTU_HARDWARE_CONTROL_BOUNDARY=driver` for field navigation. The native
  nav endpoint owns logical `/nav/cmd_vel`, encoded on DDS wire topic
  `rt/nav/cmd_vel`; `lingtu-driver` is the only speed exit to the remote
  Brainstem gRPC endpoint configured by `brainstem.env`.
  `LINGTU_ENDPOINT_SOURCES` and JSONL providers are compatibility/development
  inputs, not the production motion sink.
- Validate the field deployment boundary with
  `python tools/validate/validate_real_deployment.py` before claiming
  Thunder navigation is endpoint-only/no-ROS by default.
