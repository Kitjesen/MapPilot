# 2026-07-06 MuJoCo Continuous Mapping Quality Gate

This note records the first sunrise runs of the 3–5 minute continuous mapping
quality gate. It complements the short-window closure in
[2026-07-04 Native DDS Field Closure](./2026-07-04-native-dds-closure.md).

## Scope

- Target: `sunrise@192.168.66.13`
- Board path: `/home/sunrise/data/inovxio/lingtu`
- Validation mode: isolated CycloneDDS domain (not production domain `0`)
- Scene: `industrial_park`
- Drive: policy + `box_explore`
- Bridge contract: Fast-LIO-only (`imu_acc_mode=sensor`,
  `imu_acc_conditioning=realistic`, `publish_odom_prior=false`,
  `timestamp_clock=sim_hardware`, physical rolling subscan)
- SLAM config: `src/localization/fastlio2/config/mid360_mujoco_native_dds.yaml`

## Gate Groups

The continuous gate (`sim/scripts/mujoco/continuous_mapping_quality_gate.py`)
requires all four groups to pass:

| Group | Checks |
| --- | --- |
| Bridge | MuJoCo publishes `/imu/raw` + `/lidar/raw_frame`; native SLAM outputs exist |
| Continuity | Periodic `status.json` stays `TRACKING`; zero drops/rollbacks; bounded rates and velocity |
| Convergence | `sim_motion.jsonl` vs saved `trajectory.txt`: window ratios, cumulative path ratio, rigid ATE |
| Map quality | Native `save-map` PCD passes `saved_map_quality_gate.py` |

Do not treat endpoint-only motion ratio from a 30 s run as long-map closure
evidence.

## Verdict Matrix: Short vs Continuous Runs

| Run | Duration | Gate type | Motion / scale | Map quality | Overall |
| --- | ---: | --- | --- | --- | --- |
| sensor_fixed_30s | 30 s | endpoint + saved-map | XY ratio `0.982`, yaw `-0.0006 rad` | aligned near `99.65%`, far `0.0%` | **PASS** (short local) |
| continuous_gate domain232 | 180 s | continuous (first run) | endpoint ratio `1.19`; cumulative path ratio **5.04**; window ratio up to **8.17**; ATE RMSE `0.12 m` | aligned near **98.6%** | **FAIL** (scale drift) |
| sensor_fixed_240s | 240 s | endpoint + saved-map | path ratio **2.243**, yaw `-0.263 rad` | aligned near **45.73%**, far **40.78%** | **FAIL** (long trajectory) |

Interpretation:

- Short-window transport and local map shape can pass while continuous odometry
  scale still fails.
- Saved-map quality can pass (loop closure in save path) even when raw
  `trajectory.txt` over-integrates distance during low-motion turn phases.
- Endpoint displacement ratio alone is an insufficient acceptance metric for
  loop-like `box_explore` trajectories.

## 180 s Continuous Gate — Domain 232 (First Run)

Artifact (local pull):

```text
artifacts/sunrise_mujoco_continuous_mapping_gate_20260706_231109/
  summary.json
  bridge_report.json
  sim_motion.jsonl
  slam_status_samples.jsonl
  saved_map/map.pcd
  saved_map/trajectory.txt
  saved_map_quality.json
  scale_convergence.png
  trajectory_overlay.png
```

Command:

```bash
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host 192.168.66.13 \
  --duration 180 \
  --domain-id 232 \
  --drive-profile box_explore
```

### Group Results

| Group | Result | Key metrics |
| --- | --- | --- |
| Bridge | **PASS** | SLAM/MuJoCo endpoint ratio `1.19`; yaw error `0.024 rad`; bridge `ok=true` |
| Continuity (tracking) | **PASS** | `TRACKING` throughout; `0` LiDAR drops; `0` rollbacks; no scan stall |
| Continuity (rates) | **FAIL** (first run) | status median LiDAR `6.39 Hz`, IMU `127.65 Hz` vs thresholds `7` / `140` |
| Convergence | **FAIL** | sim path `4.39 m`, SLAM path `22.13 m`; cumulative ratio **5.04**; max window ratio **8.17** |
| Convergence (ATE) | **PASS** | rigid 2D ATE RMSE `0.12 m`, max `0.45 m` |
| Map quality | **PASS** | aligned near **98.6%**; loop closure applied in save path |

Bridge published counts over ~180 s drive:

```text
/imu/raw=36400
/lidar/raw_frame=1821
```

Effective bridge rates (~200 IMU Hz, ~10 LiDAR Hz) were healthy. The continuity
rate failure was partly a CPU backlog artifact: status JSON under-reported input
Hz when wall time exceeded simulated duration.

### Post-run Fix (Gate Code)

`analyze_status_continuity()` now uses
`max(status_median_hz, bridge_published_count / drive_duration)` for IMU and
LiDAR input rates when `bridge_report.published` is available. Re-run on sunrise
is pending to confirm continuity passes with unchanged scale verdict.

## Domain ID Constraint

CycloneDDS on sunrise rejects high domain IDs (multicast port out of range).
Domain **234** failed at startup. Use isolated domains in **`200–232`** for MuJoCo
gates. Production robot SLAM stays on domain **`0`**.

The gate and sunrise runner now validate `--domain-id` and default to **`231`**.

## Remaining Product Blocker

The continuous gate infrastructure is ready. The open product issue is **SLAM
odometry path-length scale drift** on long `box_explore` trajectories:

- Shape can look acceptable after rigid alignment (ATE pass).
- Raw `trajectory.txt` (pre-loop odometry history) can still over-integrate
  distance during near-stationary turn segments.
- Saved PCD can pass map-quality gate while trajectory scale fails convergence.

Next tuning experiments (not gate work):

1. Compare `arc` vs `box_explore` at 120–180 s under the same continuous gate.
2. Review Fast-LIO velocity/ZUPT constraints in `mid360_mujoco_native_dds.yaml`.
3. Evaluate exporting loop-optimized poses into `trajectory.txt` for convergence
   checks vs keeping raw odometry as the motion truth signal.

## Commands

Local:

```bash
export PYTHONPATH="$PWD/src:$PWD"
python sim/scripts/mujoco/continuous_mapping_quality_gate.py \
  --duration 180 \
  --domain-id 231 \
  --drive-profile box_explore
```

Sunrise remote:

```bash
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host 192.168.66.13 \
  --duration 180 \
  --domain-id 231 \
  --drive-profile box_explore
```

Unit tests:

```bash
python -m pytest sim/tests/test_continuous_mapping_quality_gate.py -q
```

## Related Docs

- Requirements and thresholds:
  `docs/07-testing/thunderv4_mujoco_lidar_recording_requirements.md`
- Short-window native DDS closure history:
  `docs/07-testing/field-runs/2026-07-04-native-dds-closure.md`
- Simulation entrypoints: `sim/README.md`

## Tuning Plan (2026-07-07)

Root cause hypothesis for cumulative path ratio **5.04** on `box_explore`:

- Turn segments (`vx=0`, `wz=0.35` for 4.5 s every 24 s cycle) let IMU velocity
  grow between LiDAR corrections because the old config allowed **3.0 m/s**
  state velocity and **1.0 m/s** per-scan velocity jumps.
- `trajectory.txt` is raw `pose_history_` odometry, not loop-optimized poses, so
  convergence fails even when saved PCD passes map quality.

### Fast-LIO config changes (corrected)

**Do not lower `max_update_velocity_mps`.** In `ieskf.cpp`, when `|v|` exceeds the
cap the entire LiDAR update is discarded and the state reverts to IMU
prediction. A 2026-07-07 rerun with `max_update_velocity_mps=0.40` diverged to
**7262 m** SLAM XY and **117 m/s** velocity spikes.

Safe knobs for MuJoCo policy long runs:

| Parameter | Baseline | Tuned default | `vel_tight` variant | Why |
| --- | ---: | ---: | ---: | --- |
| `max_update_velocity_mps` | 3.0 | **3.0** | **3.0** | keep high; absolute cap is a reject trigger |
| `max_update_velocity_delta_mps` | 1.0 | **0.35** | **0.25** | limit per-scan velocity jumps only |
| `max_update_translation_m` | 0.5 | **0.35** | **0.25** | cap per-scan pose step |
| `max_update_rotation_rad` | 0.35 | **0.35** | **0.30** | bound in-place turn steps |
| `zupt_min_static_frames` | 5 | **6** | **6** | clamp residual velocity on `vx=0` gaps |
| `zupt_sigma_v` | 0.02 | **0.015** | **0.012** | stronger ZUPT when static |

Drive-profile A/B (bridge side, no Fast-LIO reject risk):

- `arc`: continuous gentle motion; expect better scale than `box_explore`.
- `box_explore_gentle`: longer forward legs, turn rate capped at **0.20 rad/s**
  instead of **0.35**.

### 2026-07-07 Hz-fix rerun (domain 231)

Artifact:

```text
artifacts/sunrise_mujoco_continuous_mapping_gate_20260707_015451/
```

| Group | Result | Notes |
| --- | --- | --- |
| Continuity Hz | **PASS** | bridge effective IMU **202 Hz**, LiDAR **10.1 Hz** |
| Continuity tracking | **PASS** | zero drops/rollbacks |
| Convergence | **FAIL** | run used bad `max_update_velocity_mps=0.40` experiment; ignore numbers |
| Map quality | **FAIL** | consequence of odom divergence |

After reverting velocity cap, rerun with safe yaml + `arc180` / `box180_gentle`
comparison via sweep script.

### Comparison sweep

```bash
# Single rerun with Hz fix + updated default yaml
python sim/scripts/run_sunrise_continuous_mapping_gate.py \
  --host 192.168.66.13 --duration 180 --domain-id 231 \
  --drive-profile box_explore

# arc vs box_explore 120/180 s matrix (+ vel_tight A/B)
python sim/scripts/run_sunrise_continuous_mapping_sweep.py \
  --host 192.168.66.13 \
  --start-domain-id 226 \
  --json-out artifacts/sunrise_continuous_mapping_sweep.json

# Subset only
python sim/scripts/run_sunrise_continuous_mapping_sweep.py \
  --cases box180_baseline,arc180
```

Expected after Hz fix alone: **continuity pass**, **convergence still fail** on
`box_explore` until velocity caps take effect.

## Result

**BLOCKED** for full sim mapping acceptance. Transport and short local mapping
are closed; 3–5 minute continuous scale convergence is **not** closed on
`box_explore` until odometry path integration is fixed or trajectory export
semantics are aligned with save-path loop closure.
