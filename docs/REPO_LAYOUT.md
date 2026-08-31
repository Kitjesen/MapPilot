# LingTu Repository Layout

This file is a routing map, not a full architecture document. Use it to decide
where code belongs before adding a new file.

## Runtime Entry

| Path | Role |
| --- | --- |
| `src/lingtu/control.py` | Public Product lifecycle: `switch`, `status`, `stop`. |
| `src/lingtu/real/`, `src/lingtu/sim/` | Environment-specific process execution. |
| `src/lingtu/assembly/` | Side-effect-free Product resolution and Host Blueprint construction. |

## Source Packages

| Path | Put here |
| --- | --- |
| `src/runtime/` | Module, Blueprint, ports, registry, shared messages, transport, TF. |
| `src/message/idl/` | Native DDS IDL and generated cross-language message contracts. |
| `src/diagnostics/` | Runtime evidence, acceptance gates, audit helpers, and migration catalogs. |
| `src/drivers/` | Real/sim robot and sensor backends. |
| `src/localization/` | SLAM/localization modules and portable SLAM code. |
| `src/nav/` | Host navigation commands/goals/skills/adapters and native C++ navigation. |
| `src/maps/` | Persistent map services, layers, artifacts, and native map adapters. |
| `src/explore/` | TARE exploration contracts, native bindings, and Host supervision. |
| `src/nav/cpp/` | Native global/local planning, tracking, recovery, safety, DDS endpoint, and clients. |
| `src/nav/commands/`, `src/nav/services/`, `src/nav/skills/` | Host command forwarding, goal admission, and Agent/MCP navigation tools. |
| `src/perception/` | Detector, encoder, tracker, scene graph, reconstruction. |
| `src/decision/` | Semantic planner, goal resolution, visual servo, task logic. |
| `src/memory/` | Semantic, episodic, tagged, vector, temporal memories. |
| `src/gateway/` | REST/SSE/WS, MCP, teleop, runtime status. |
| `src/*/adapters/` | ROS2, LCM, portable compatibility adapters. |
| `src/kernels/` | Portable compute kernel paths and ABI contracts. |

## Non-Runtime Roots

| Path | Role |
| --- | --- |
| `config/` | Robot, device, topic, endpoint, package configuration. |
| `tools/calibration/` | Offline camera, IMU, LiDAR, and camera-LiDAR calibration tools. |
| `sim/` | Simulation engines, worlds, robots, assets, public script gates, validation, and tests. `sim/scripts/mujoco/` is the only current MuJoCo command and test path. |
| `scripts/` | ProductControl adapter, native build/deploy, field gates, and simulation launchers. |
| `scripts/deploy/thunder/` | Thunder field systemd units and run/install wrappers, including the canonical `lingtu-driver` remote Brainstem boundary. |
| `tools/` | Developer validation, benchmarks, offline analysis, one-shot packaging. |
| `tests/` | Remaining integration/script tests outside package-owned tests. |
| `web/` | React/Vite dashboard. |
| `docs/` | Current guides/contracts, one active roadmap, non-authoritative research, and dated evidence. Git history is the archive. |

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
python tools/validate/validate_docs.py
python -m pytest tests/contracts/test_runtime_architecture_boundaries.py -q
```
