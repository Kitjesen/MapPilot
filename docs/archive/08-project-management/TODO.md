# TODO

Status legend: `[x]` done, `[ ]` outstanding, `[~]` in progress.

> This list is forward-looking. Closed items now live in CHANGELOG.

---

## 2026 Q2 — Near-term

### Build and test

- [ ] Full colcon build verification of every ROS 2 package shipped under `install/`.
- [ ] End-to-end integration test: gRPC channel -> NavigationModule -> dashboard closed loop.
- [ ] On-robot calibration sign-off (six-step SOP run + `verify.py` all-green).
- [ ] On-robot C++ benchmark (real ARM NEON xsimd + OpenMP gain measurement).

### Task system

- [ ] TaskManager JSON parsing upgrade (compound task descriptions).
- [ ] Mission history persistence + replay in dashboard.

### Reliability

- [ ] rosbag automatic archive of critical topics + remote export hooks.
- [ ] Localization-quality threshold calibration (ICP fitness -> velocity curve, on-robot).

### Navigation and perception

- [ ] Geofence editor UI (drag polygon on the dashboard map).
- [ ] GenZ-ICP `TransformPoints` SIMD batching (SE3 acceleration).
- [ ] ikd-Tree node memory pool (reduce malloc fragmentation, stabilise latency).
- [ ] Migrate `terrainAnalysis.cpp` ROS 2 node to `nav_core::TerrainAnalysisCore` (eliminate duplicated code).

### Dashboard

- [ ] Offline graceful degradation (cache last-known state when WS disconnected).

---

## 2026 Q3+

- [ ] BehaviorTree to replace the FSM (more flexible task choreography).
- [ ] Multi-robot coordination (fleet scheduling + collision avoidance + unified panel).
- [ ] CI-integrated simulation regression (Gazebo / Isaac Sim).
- [ ] 3D map preview (point-cloud renderer + live SLAM map).
- [ ] Mobile (Android / iOS) tuning pass.

---

## Recently Closed (full record in CHANGELOG)

- DUFOMap Phase 2 + offline filter wiring.
- Drift watchdog + relocalization persistence.
- `scripts/lingtu` operations CLI + dashboard bag UI.
- TARE Supervisor + GNSS RTK fusion + NTRIP injection.
- Module-First wave 1 / 2 / 3 — guardrails, precision, OSNet / PD / TSDF / Bayesian / config.
- Composable stack factories under `src/runtime/blueprints/stacks/`.

---

*Update this file whenever a task moves between sections.*
