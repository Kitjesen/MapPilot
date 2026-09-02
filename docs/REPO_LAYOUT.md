# LingTu Repository Layout

This file is a routing map, not a full architecture document. Use it to decide
where code belongs before adding a new file.

`src/` is organized by functional ownership. Products select and compose those
functions through `config/runtime_graph/products/` and `src/lingtu/`; they do
not create parallel Product-owned algorithm trees.

## Four Independent Axes

| Axis | Source of truth |
| --- | --- |
| Functional source ownership | `src/<owner>/` |
| Runtime environment | `config/runtime_graph/envs/real.yaml` and `sim.yaml` |
| Simulation workspace | root `sim/`, preserving the `sim.*` namespace |
| Generated output stage | `build/` -> `install/` -> `dist/` |

`env=sim` means ProductControl selected the simulation runtime environment;
root `sim/` is the simulation workspace. Do not create root `real/`,
`environments/`, or a tracked `bin/` to represent either concept.

Generated files use one-way staging:

```text
build/<component>/                         developer compilation tree
install/<platform>-<arch>/<config>/bin/    native executables
install/<platform>-<arch>/<config>/lib/    runtime libraries
install/<platform>-<arch>/<config>/etc/lingtu/   immutable defaults
install/<platform>-<arch>/<config>/share/lingtu/ planner assets and schemas
dist/                                      final archives and manifests
```

Only the install prefix has a `bin/`; the repository root does not.

## Runtime Entry

| Path | Role |
| --- | --- |
| `src/lingtu/control.py` | Public Product lifecycle: `switch`, `status`, `stop`. |
| `src/lingtu/real/`, `src/lingtu/sim/` | Environment-specific process execution. |
| `src/lingtu/assembly/` | Side-effect-free Product resolution and Host Blueprint construction. |

## Source Families

These family names are navigation labels only. Keep the physical layout flat;
do not create matching wrapper directories under `src/`.

### product_control — Product 控制面

| Path | Put here |
| --- | --- |
| [`src/lingtu/`](../src/lingtu/README.md) | ProductControl, RunPlan, Product assembly, and real/sim lifecycle routing. |

### capabilities — 机器人功能域

| Path | Put here |
| --- | --- |
| [`src/decision/`](../src/decision/README.md) | Semantic planner, goal resolution, visual servo, task logic. |
| [`src/drivers/`](../src/drivers/README.md) | Real/sim robot and sensor backends. |
| [`src/explore/`](../src/explore/README.md) | TARE exploration contracts, native bindings, Host supervision, and `lingtu_explore_dds` source. |
| [`src/localization/`](../src/localization/README.md) | SLAM, localization, relocalization, and GNSS fusion. |
| [`src/maps/`](../src/maps/README.md) | Persistent map services, layers, artifacts, and native map adapters. |
| [`src/memory/`](../src/memory/README.md) | Semantic, episodic, tagged, vector, temporal memories. |
| [`src/nav/`](../src/nav/README.md) | Host navigation commands/goals/skills/adapters and native C++ navigation. |
| [`src/perception/`](../src/perception/README.md) | Detector, encoder, tracker, scene graph, reconstruction. |

### platform — 共享平台与接口

| Path | Put here |
| --- | --- |
| [`src/runtime/`](../src/runtime/README.md) | Module, Blueprint, ports, registry, shared messages, transport, TF. |
| [`src/message/`](../src/message/README.md) | Native DDS IDL, topic metadata, QoS, and generated cross-language contracts. |
| [`src/gateway/`](../src/gateway/README.md) | REST/SSE/WS, MCP, teleop, runtime status. |
| [`src/diagnostics/`](../src/diagnostics/README.md) | Runtime evidence, acceptance gates, audit helpers, and migration catalogs. |

### compute — 共享底层计算

| Path | Put here |
| --- | --- |
| [`src/kernels/`](../src/kernels/README.md) | Portable compute kernel paths and ABI contracts. |
| [`src/native/`](../src/native/README.md) | Shared native services that are neither domain-owned algorithms nor portable kernels. |

Create lower-level folders only when needed: `modules/`,
`adapters/<protocol>/`, `cpp/` or `rust/`, `real/` or `sim/`, `tests/`, and
`contracts/`. Do not add ambiguous buckets such as `common/`, `misc/`,
`helpers/`, `new/`, or `v2/`.

## Non-Runtime Roots

| Path | Role |
| --- | --- |
| `config/` | Robot, device, topic, endpoint, package configuration. |
| `tools/calibration/` | Offline camera, IMU, LiDAR, and camera-LiDAR calibration tools. |
| `sim/` | Single simulation package: `packages/` owns manifests/assets, `sessions/` owns SessionSpecs, `catalog/` compiles them, and `runtime/` runs resolved bundles. Compatibility, diagnostics, evaluation, distribution, tools, stable MuJoCo gates, and tests stay in their named roots. |
| `scripts/` | ProductControl adapter, native build/deploy, field gates, and simulation launchers. |
| `scripts/deploy/thunder/` | Thunder field systemd units and run/install wrappers, including the canonical `lingtu-driver` remote Brainstem boundary. |
| `tools/` | Developer validation, benchmarks, offline analysis, one-shot packaging. |
| `tests/` | Remaining integration/script tests outside package-owned tests. |
| `web/` | React/Vite dashboard. |
| `docs/` | Current guides/contracts, one active roadmap, non-authoritative research, and dated evidence. Git history is the archive. |
| `build/` | Ignored developer compilation trees; never a production runtime path. |
| `install/` | Ignored standard installation staging with `bin/lib/etc/share`. |
| `dist/` | Ignored final release artifacts. |

## Placement Rules

| If the file... | Put it in |
| --- | --- |
| runs as a long-lived robot/operator command | `scripts/` |
| is a simulation package manifest or its asset | `sim/packages/<kind>/` |
| is a simulation Product preset or example SessionSpec | `sim/sessions/` |
| implements resolved simulation runtime or transport | `sim/runtime/` or `sim/adapters/` |
| is simulation-only diagnostics, evaluation, or asset tooling | `sim/diagnostics/`, `sim/evaluation/`, or `sim/tools/` |
| is a stable MuJoCo Product/native acceptance entry | `sim/scripts/mujoco/` |
| is a developer validator, benchmark, or offline helper | `tools/` |
| is normal runtime logic | `src/<owning package>/` |
| declares a Product | `config/runtime_graph/products/` |
| resolves or controls a Product | `src/lingtu/` |
| bridges ROS2, LCM, or legacy runtime protocols | `src/*/adapters/` |
| is generated cache/build output | nowhere; keep it ignored and delete it |

## Checks

```bash
python tools/validate/validate_architecture_boundaries.py --verbose
python tools/validate/validate_topics.py
python tools/validate/validate_docs.py
python -m pytest tests/contracts/test_runtime_architecture_boundaries.py -q
```
