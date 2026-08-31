# MuJoCo Native Navigation Acceptance

Status: current component-acceptance contract

This gate proves the ROS-free native navigation chain in MuJoCo. Python owns
physics, simulated sensors, process lifecycle, evidence collection, and optional
video. It does not run a global planner, local planner, path follower, safety
controller, or command mux.

LiDAR optical and scan-time fidelity is a separate claim. See
[`MUJOCO_MID360_FIDELITY.md`](MUJOCO_MID360_FIDELITY.md).

## Chain under test

```text
MuJoCo ThunderV4 + MID-360 + IMU
  -> typed DDS sensor publisher
  -> native Fast-LIO2 localization
  -> native mapd and traversability
  -> native navd
       -> OctoPlanner3D saved-map planning
       -> CMU or SCAN local planning
       -> embedded path tracking
       -> final safety and control authority
  -> typed DDS /nav/cmd_vel
  -> MuJoCo driver bridge
  -> ThunderV4 policy motion
```

The native implementation is under:

- `src/nav/cpp/planning/global/octoplanner/`;
- `src/nav/cpp/endpoint/nav/runtime/goal/`;
- `src/nav/cpp/planning/local/`;
- `src/nav/cpp/tracking/`;
- `src/nav/cpp/endpoint/nav/control/` and `src/nav/cpp/endpoint/nav/safety/`.

Planner-to-tracker handoff is a direct C++ call inside `navd`. Global and local
path DDS topics are telemetry; they are not a second execution path.

## Canonical scenario

The current long-range manifest is:

```text
config/runtime_graph/acceptance/mujoco_industrial_park_60m_navigation_acceptance.json
```

It selects the `nav` Product contract, the industrial-park MuJoCo world, an
existing `map.pcd`/`octomap.ot` map package, Fast-LIO2 localization, physical
rolling LiDAR timing, a 200 Hz IMU, and a 59.94 m start-goal chord.

The manifest is the scenario source of truth. Do not copy its thresholds or
binary paths into another runner.

## Run

PowerShell:

```powershell
$env:PYTHONPATH = "src;."
python sim/scripts/mujoco/native_navigation_acceptance.py `
  --manifest config/runtime_graph/acceptance/mujoco_industrial_park_60m_navigation_acceptance.json `
  --mode motion `
  --out-dir artifacts/mujoco_native_navigation_60m `
  --strict
```

Use `--mode no_motion` to exercise startup, localization, planning, and the
zero-command boundary without simulated robot motion. Use `--mode both` when
one report must contain both phases.

The command-line interface can be checked without launching native processes:

```powershell
python sim/scripts/mujoco/native_navigation_acceptance.py --help
```

## Pass conditions

The runner writes `report.json`; that report, not prose in this page, decides
the result. For the current 60 m manifest, a pass requires:

- ROS is not used;
- the ThunderV4 policy and every required native process are available;
- Fast-LIO2 reaches tracking and stays within the configured accuracy limits;
- the input gate remains ready within the configured stale/rejection budgets;
- the native goal reaches its terminal `Reached` state;
- executed path length, net XY displacement, and goal-distance reduction are
  each at least 50 m;
- terminal driver stop, zero command, authority clear, and owned-process cleanup
  are confirmed.

The exact numeric limits remain in the manifest.

## Required evidence

The acceptance directory must contain the machine-readable report and the
phase evidence referenced by it. Typical outputs include:

- `report.json`;
- native process logs and status snapshots;
- trajectory and command samples;
- localization and map-tracking observations;
- optional video when `--record-video` is selected.

A rendered video is presentation evidence. It does not replace goal lifecycle,
localization, final-command, stop, or cleanup evidence.

## Scope

This gate is native component evidence in MuJoCo. It does not prove:

- exact field ProductControl/systemd realization;
- MID-360 physical optics;
- Brainstem motor execution;
- physical S100P motion;
- long-term map quality or repeatability.

Use S100P no-motion and supervised-motion procedures for those claims. A local
pass must not be reported as field readiness.
