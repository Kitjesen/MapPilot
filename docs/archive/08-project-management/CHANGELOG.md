# Changelog

All notable changes to LingTu. Format inspired by [Keep a Changelog](https://keepachangelog.com/).

> The pre-2026-04 entries described the legacy `remote_monitoring` / `flutter_monitor` C++/Dart stack and predate the Module-First refactor. They have been pruned. Re-establish entries from this point forward.

---

## Unreleased

- Documentation pass: `docs/LINGTU_HANDBOOK.md` rewritten in English; obsolete Chinese / February-March test reports removed.

---

## 2026-04-21 — DUFOMap Phase 2 stabilised

- DUFOMap offline filter wired into `MapManager.save` between PGO and tomogram generation; clean point cloud written back to `map.pcd`, original backed up as `map.pcd.predufo`.
- Phase 1 voxel hit-count voting active in `gateway_module.py::_on_map_cloud` (`LINGTU_MAP_MIN_HITS`, default 3) — affects only the SSE web view, not on-disk PCD.
- Toggle: `LINGTU_SAVE_DYNAMIC_FILTER=0` disables the offline filter.
- Detail: `docs/05-specialized/dynamic_obstacle_removal.md`.

## 2026-04-19 — Drift watchdog + relocalization persistence

- Gateway background `_drift_watchdog_loop` detects Fast-LIO2 IEKF runaway (xy > 500 m or speed > 5 m/s for 5 frames), stops `slam` / `slam_pgo` / `localizer`, restarts via `svc.ensure(...)` for the active session, emits SSE `slam_drift`.
- Last successful `/api/v1/slam/relocalize` pose persisted to `~/.lingtu/last_nav_pose.json`; restored automatically 2.5 s after `session/start navigating`.
- Tunables: `LINGTU_DRIFT_WATCHDOG`, `_INTERVAL`, `_XY_LIMIT`, `_V_LIMIT`, `_COOLDOWN`.

## 2026-04-17 — Operations CLI + bag UI

- `scripts/lingtu` consolidated CLI replaces ad-hoc `curl` / `systemctl` invocations: `status`, `watch`, `map`, `nav`, `svc`, `log`, `health`.
- One-click bag recording from the dashboard topbar (13 topics by default, output `~/data/bags/`).
- `BACKLOG.md` becomes the single registry; L1 / L2 git hooks plus P0 acceptance scripts under `docs/07-testing/` enforce the discipline.

## 2026-04 — TARE exploration + GNSS RTK

- TARE Supervisor + Gateway SSE — hierarchical exploration backend integrated.
- GNSS module: `GnssModule` (LLA -> ENU + quality), `GnssBridgeModule` (DeviceManager -> fusion), `NtripClientModule` (RTCM injection). Antenna lever-arm compensation and `fusion_health` diagnostics in place.
- Dashboard SLAM / GNSS / Fusion floating widget with theme-aware drag handle.

## 2026-04 — Module-First waves

- **Wave 1 — hard guardrails**: 17 tests covering startup calibration check (`run_calibration_check`), priority-based `CmdVelMux`, `SafetyRing` localisation-status hookup.
- **Wave 2 — precision upgrades**: 39 tests covering perception + motion + bbox / VLM / agent integration.
- **Wave 3 — OSNet, PD, TSDF, Bayesian, config**: 28 tests; OSNet x1.0 128x256 BPU model integrated for person Re-ID.

## 2026-03 — Module-First architecture

- Composable factory pattern under `src/runtime/blueprints/stacks/`: `driver`, `slam`, `maps`, `perception`, `memory`, `planner`, `navigation`, `safety`, `gateway`, plus auxiliary `lidar`, `sim_lidar`, `exploration` factories.
- Plugin Registry across SLAM (`fastlio2`, `pointlio`, `localizer`, `bridge`), Detector (`yoloe`, `yolo_world`, `bpu`, `grounding_dino`), Encoder (`clip`, `mobileclip`), LLM (`kimi`, `openai`, `claude`, `qwen`, `mock`), Planner (`astar`, `pct`), PathFollower (`nav_core`, `pure_pursuit`, `pid`).
- Per-wire transport: `callback`, `dds`, `shm`. Five backpressure policies on `Out` ports.

---

## Roadmap

### 2026 Q2

- ROS 2 Phase 1: drop rclpy from `CameraBridge` and `ROS2SimDriver` (P3-01, P3-02).
- Localization-quality threshold calibration on robot.
- ICP `fitness_score` -> velocity mapping curve, real-data tuned.

### 2026 Q3+

- ROS 2 Phase 2: nanobind library wraps for Fast-LIO2 / Point-LIO (P3-03, P3-04).
- BehaviorTree replacing the mission FSM.
- Multi-robot coordination.
- Simulation regression framework (Gazebo / Isaac Sim) in CI.

---

*Last updated: 2026-04-25.*
