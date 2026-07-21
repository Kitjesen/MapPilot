# LingTu Open-RMF sidecar

This integration keeps the LingTu product runtime ROS-free. Open-RMF and ROS 2
run in an optional Ubuntu 24.04 / ROS 2 Jazzy sidecar and exchange only
high-level navigation commands and robot state with LingTu Gateway.

```text
Open-RMF task / traffic schedule
  -> EasyFullControl fleet adapter
  -> LingTu Gateway goal + lease + cancel
  -> native C++ navigation command client
  -> typed CycloneDDS request / ACK
  -> LingTu native autonomy endpoint
```

The sidecar never publishes velocity commands. `autonomy`, `teleop`, and
`teleop_avoid` remain mutually exclusive native endpoint modes.

Open-RMF owns facility-level orchestration: tasks, traffic schedules, named
building waypoints, doors, lifts, charging, and later multi-robot resource
conflicts. LingTu continues to own localization, maps, global/local planning,
terrain/obstacle safety, path following, and robot control.

## What this first integration supports

- One LingTu robot in one Open-RMF fleet.
- Explicit RMF level name to LingTu building/floor/map bindings.
- Same-floor high-level navigation goals.
- Native DDS request correlation and physical arrival evidence.
- Command lease acquire, unique renewals, cancel, and verified release.
- Read-only shadow configuration before task dispatch is enabled.

Cross-floor requests in the active Gateway sidecar path still fail closed. The
ROS-free LingTu core now contains `BuildingMissionOrchestrator`,
`LiftTransitionExecutor`, correlated native navigation observation, and native
map/relocalization gates. They are not enabled for live cross-floor dispatch
until a site-specific lift adapter and transition geometry are configured and
validated. The sidecar does not switch maps or use Z height to impersonate a
floor change.

Implementation: `src/nav/building/`. Runtime and safety contract:
`docs/architecture/BUILDING_MISSION_RUNTIME.md`.

## Multi-floor and lift roadmap

1. **Implemented and unit-tested:** ROS-free `BuildingMissionOrchestrator`
   expands a floor-aware objective into native navigation and an optional
   explicit transition.
2. **Implemented and unit-tested with fake ports:** `LiftTransitionExecutor`
   state machine:
   `approach -> call -> wait -> enter -> verify cabin -> ride -> switch map ->
   relocalize -> exit`.
3. **Implemented at simulator-kinematic scope:** a ROS-free MuJoCo named-joint
   lift/door adapter now completes one Thunder-model source-to-target mission
   and records a phase timeline/video. It does not prove gait, SLAM, saved-map
   relocalization, native DDS transport, or a real lift protocol.
4. **Still required per site:** connect the building lift controller to the
   Open-RMF lift protocol in a
   separate infrastructure adapter. The robot sidecar must not talk directly to
   a lift PLC.
5. **Still required:** bind semantic destinations to
   `(building_id, floor_id, map_id, waypoint)`
   before dispatch. Floor identity is never inferred from Z height.
6. **Still required:** validate one robot first: repeated same-floor tasks, then one lift round
   trip, then interrupted/recovery cases, before enabling multi-robot bidding.

## Optional external sidecar build

Do not install ROS 2 on ThunderV4 or make it part of the LingTu product
runtime. This section is only for a dedicated WSL2, container, or coordination
host that runs upstream Open-RMF outside the robot control stack. Single-robot
LingTu navigation does not require this sidecar.

Install ROS 2 Jazzy and Open-RMF from the official Jazzy packages, then:

```bash
export LINGTU_ROOT=/mnt/d/inovxio/brain/lingtu
export LINGTU_ENABLE_EXTERNAL_ROS2_SIDECAR=1
cd "$LINGTU_ROOT"
bash integrations/open_rmf/scripts/install_jazzy_open_rmf.sh
bash integrations/open_rmf/scripts/build_sidecar.sh
```

The equivalent manual workspace setup is:

```bash
mkdir -p ~/lingtu_rmf_ws/src
ln -s "$LINGTU_ROOT/integrations/open_rmf/ros_ws/src/lingtu_rmf_adapter" \
  ~/lingtu_rmf_ws/src/lingtu_rmf_adapter
cd ~/lingtu_rmf_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Run safely

Gateway must have a route-scoped key:

```bash
export LINGTU_RMF_API_KEY='replace-with-a-dedicated-secret'
```

The same key is used by the sidecar. It is accepted only for session/status,
lease, goal, and navigation-cancel routes. It is rejected for direct velocity,
teleoperation, mode switching, and unrelated Gateway APIs.

### Secure transport

The checked-in bridge config connects to `http://127.0.0.1:5050`; loopback HTTP
keeps the scoped key off the network. When the sidecar runs away from the robot,
use HTTPS or create an SSH tunnel before starting it:

```bash
export LINGTU_HOST=ROBOT_IP_OR_HOSTNAME
ssh -N -L 5050:127.0.0.1:5050 "sunrise@${LINGTU_HOST}"
```

Keep `base_url: http://127.0.0.1:5050` when using the tunnel. Remote plaintext
HTTP is rejected unless `allow_insecure_http` is deliberately enabled; that
escape hatch is for isolated diagnostics, not live dispatch.

### Coordinate alignment

Open-RMF coordinates and a LingTu saved-map frame are not assumed to be
identical. Jazzy EasyFullControl applies each `rmf_fleet.transforms` entry from
RMF canonical coordinates into robot coordinates and applies the inverse when
reporting robot state. Live LingTu commands fail closed unless every configured
level has an explicit transform with finite values and a positive scale.

Copy the live fleet config and add measured values for every enabled level:

```yaml
rmf_fleet:
  transforms:
    L1:
      translation: [measured_x_m, measured_y_m]
      rotation: measured_yaw_rad
      scale: measured_scale
    L2:
      translation: [measured_x_m, measured_y_m]
      rotation: measured_yaw_rad
      scale: measured_scale
```

Do not enter an identity transform merely to open the gate. Use at least three
well-separated common landmarks per floor, record the calibration artifact,
and verify both RMF-to-LingTu goals and LingTu-to-RMF reported poses. The
checked-in live fleet sample intentionally omits transforms, so it cannot issue
commands until a site-specific calibrated copy is supplied.
Start with commands disabled:

```bash
export LINGTU_ROOT=/mnt/d/inovxio/brain/lingtu
export LINGTU_RMF_COMMANDS_ENABLED=0
source /opt/ros/jazzy/setup.bash
source ~/lingtu_rmf_ws/install/setup.bash
ros2 run lingtu_rmf_adapter lingtu_rmf_adapter \
  --bridge-config "$LINGTU_ROOT/integrations/open_rmf/ros_ws/src/lingtu_rmf_adapter/config/lingtu_bridge.yaml" \
  --fleet-config "$LINGTU_ROOT/integrations/open_rmf/ros_ws/src/lingtu_rmf_adapter/config/lingtu_fleet_shadow.yaml" \
  --nav-graph /path/to/building_nav_graph.yaml
```

After state/floor and coordinate-transform verification, use a site-specific
copy of the live fleet config and explicitly enable commands:

```bash
export LINGTU_RMF_COMMANDS_ENABLED=1
ros2 run lingtu_rmf_adapter lingtu_rmf_adapter \
  --bridge-config /path/to/lingtu_bridge.yaml \
  --fleet-config /path/to/lingtu_fleet_single_robot.yaml \
  --nav-graph /path/to/building_nav_graph.yaml
```

Before live dispatch, the LingTu Gateway session must already be
`mode=navigating` with the matching saved map active. The sidecar deliberately
does not start/end sessions because that would mix building orchestration with
robot navigation ownership.

## Acceptance sequence

1. Shadow: floor name, map identity, pose freshness, and battery state are
   visible in RMF.
2. Coordinates: each level transform is measured, recorded, and checked in
   both directions against common landmarks.
3. Same-floor dry run: RMF destination is admitted and native command ACK is
   correlated to the same immutable request ID.
4. Same-floor motion: lease renews, `last_plan.goal` matches the requested
   target, and `last_local.goal_reached` transitions false to true.
5. Stop: RMF stop produces LingTu navigation cancel and releases the lease.
6. Only after those pass, enable loop/patrol dispatch.

Open-RMF upstream references:

- [EasyFullControl Jazzy API](https://docs.ros.org/en/jazzy/p/rmf_fleet_adapter/generated/classrmf__fleet__adapter_1_1agv_1_1EasyFullControl.html)
- [Official fleet adapter example](https://github.com/open-rmf/rmf_demos/blob/main/rmf_demos_fleet_adapter/rmf_demos_fleet_adapter/fleet_adapter.py)
- [Fleet adapter template](https://github.com/open-rmf/fleet_adapter_template)
