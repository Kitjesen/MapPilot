# LingTu Repository Layout

This file is a routing map, not a full architecture document. Use it to decide
where code belongs before adding a new file.

## Runtime Entry

| Path | Role |
| --- | --- |
| `lingtu.py` | Primary CLI entry. |
| `cli/` | Argument parsing, profiles, REPL, daemon lifecycle, terminal UI. |
| `src/lingtu/` | Package-facing API and local runtime handoff. |

## Source Packages

| Path | Put here |
| --- | --- |
| `src/runtime/` | Module, Blueprint, ports, registry, shared messages, transport, TF. |
| `src/diagnostics/` | Runtime evidence, acceptance gates, audit helpers, and migration catalogs. |
| `src/drivers/` | Real/sim robot and sensor backends. |
| `src/localization/` | SLAM/localization modules and portable SLAM code. |
| `src/nav/` | Navigation FSM, maps, safety, exploration, planner services. |
| `src/nav/services/plan/` | Navigation planning services, global/local planner contracts, and backend dispatch. |
| `src/nav/services/plan/global_planner/algorithm/pct/vendor/` | External PCT source/resources kept out of LingTu runtime packages. |
| `src/nav/local/` | Local planner, terrain, path follower C++/nanobind paths. |
| `src/perception/` | Detector, encoder, tracker, scene graph, reconstruction. |
| `src/decision/` | Semantic planner, goal resolution, visual servo, task logic. |
| `src/memory/` | Semantic, episodic, tagged, vector, temporal memories. |
| `src/gateway/` | REST/SSE/WS, MCP, teleop, runtime status. |
| `src/*/adapters/` | ROS2, LCM, portable compatibility adapters. |
| `src/kernels/` | Portable compute kernel paths and ABI contracts. |
| `src/gateway/media/` | Camera transport notes for go2rtc WHEP and JPEG fallback. |

## Non-Runtime Roots

| Path | Role |
| --- | --- |
| `config/` | Robot, device, topic, endpoint, package configuration. |
| `calibration/` | Camera, IMU, LiDAR, camera-LiDAR calibration tools. |
| `sim/` | Simulation engines, worlds, robots, assets, public script gates, validation, and tests. MuJoCo implementations live under `sim/scripts/mujoco/`; old `sim/scripts/<name>` entrypoints are compatibility wrappers. |
| `scripts/` | Build, deploy, OTA, diagnostics, native/test wrappers plus ROS workspace compatibility, ROS2 compatibility perception demos, robot-side operations. |
| `tools/` | Developer validation, benchmarks, offline analysis, one-shot packaging. |
| `tests/` | Remaining integration/script tests outside package-owned tests. |
| `web/` | React/Vite dashboard. |
| `docs/` | Current docs plus archived design material. |

## Placement Rules

| If the file... | Put it in |
| --- | --- |
| runs as a long-lived robot/operator command | `scripts/` |
| is a developer validator, benchmark, or offline helper | `tools/` |
| is normal runtime logic | `src/<owning package>/` |
| bridges ROS2, LCM, or legacy runtime protocols | `src/*/adapters/` |
| is generated cache/build output | nowhere; keep it ignored and delete it |

## Checks

```bash
python tools/validate/validate_architecture_boundaries.py --verbose
python tools/validate/validate_topics.py
python -m pytest tests/contracts/test_module_first_runtime_boundaries.py -q
```
