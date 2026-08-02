# Sunrise Navigation Read-Only Audit

- Date: 2026-07-27
- Target: `sunrise@192.168.66.13`
- Scope: read-only Sunrise runtime audit for mapping readiness, local
  avoidance readiness, and global navigation preview
- Motion boundary: no service switch, no map save, no navigation goal, no
  teleop command, and no robot motion

## Result Summary

| Gate | Result | Evidence |
| --- | --- | --- |
| Gateway reachability | PASS | HTTP Gateway responded on the target host |
| SSH command channel | BLOCKED | SSH key/password authentication was unavailable |
| Active product identity | OBSERVED | active Product/profile was `map`; session was `mapping` |
| Motion command state | SAFE | control source was `none`; command output was zero |
| Safety state | SAFE-BLOCKED | safety reported stop/fail-closed behavior |
| Camera stream | PASS | camera reported 30 Hz |
| Native services | OBSERVED | `lingtu-livox-dds`, `lingtu-slam-dds`, and navigation service processes were active; this proves process liveness only |
| DDS live sensor samples | OBSERVED-LIMITED | Gateway `services/status` `dds_check` typed probe reported zero `rt/lidar/raw_frame`, zero `rt/imu/raw`, zero SLAM odometry, and zero SLAM map-cloud samples within a limited sampling window; raw JSON, DDS domain, and window provenance were not archived |
| Localization health topic | OBSERVED | localization-health samples were present as runtime-loop liveness only; this does not prove sensor or localization readiness |
| SLAM output readiness | BLOCKED | native SLAM status snapshot reported a 50 Hz tick and `waiting_for_imu`; no odometry or map points were produced |
| Gateway readiness | DEGRADED | `/ready` returned HTTP 503 with `non_motion_safe=true` |
| Active map artifact gate | PASS | `map_flo` hash and frame checks passed for no-motion planning preview |
| Global plan preview | BLOCKED | no-motion preview to `(1, 0, 0)` published no motion, then blocked with `odometry_missing` |

## Runtime Evidence

The deployed robot was in mapping mode when inspected:

| Field | Value |
| --- | --- |
| Active profile | `map` |
| Product session | `mapping` |
| Control source | `none` |
| Command output | zero |
| Safety posture | stop/fail-closed |
| Gateway readiness | HTTP 503 degraded |
| Non-motion safety flag | `non_motion_safe=true` |

The observed state supports read-only inspection and no-motion API checks. It
does not support map building, map saving, or navigation motion until live
LiDAR, IMU, SLAM odometry, and map-cloud output recover.

## Sensor And SLAM State

Installed native LiDAR, SLAM, and navigation service processes were active, but
that only proves process liveness. The Gateway `services/status` `dds_check`
typed probe reported no live LiDAR or IMU samples in its limited sampling
window:

| Topic or Output | Observed Count |
| --- | ---: |
| `rt/lidar/raw_frame` | 0 |
| `rt/imu/raw` | 0 |
| SLAM odometry | 0 |
| SLAM map cloud | 0 |
| Localization health | samples present as runtime-loop liveness only |

This record did not preserve the raw `services/status` JSON, DDS domain, query
URL, request time, or sampling-window metadata. It also did not preserve proof
that the typed DDS probe had `checked=true`. Treat these zero counts as a
limited observed snapshot, not as evidence of continuous DDS silence or a
physical root cause.

Native SLAM status itself reported a 50 Hz tick rate and `waiting_for_imu`.
That status is a native status snapshot; by itself it does not prove LiDAR
availability or LiDAR failure. It does explain the downstream navigation
blocker at the time observed: without odometry, the planner can validate the
map artifact gate but cannot complete a real start-state plan.

## Map And Planning Evidence

The active map pointer was `map_flo`.

`map_flo` contained the expected saved-map artifacts:

- PCD map artifact
- occupancy artifact
- tomogram artifact
- OctoMap artifact
- metadata

The no-motion global preview to `(1, 0, 0)` passed the active-map artifact gate:
hash and frame validation succeeded. The preview did not publish robot motion.
It then blocked with `odometry_missing`, which is consistent with the SLAM
state above.

## Map Catalog Inconsistency

The map catalog needs cleanup before a supervised map operation:

| Finding | Evidence | Risk |
| --- | --- | --- |
| Active-map state disagreement | `map_flo` appeared as `READY` in the map list while `accept_mid360`, `accept_ready`, and `seq_profile_map` appeared as `ACTIVE` records | Operators may select the wrong active map or misread the current session state |
| Hidden backup surfaced as map | `.codex_backups` appeared in the map list | Internal backup directories can pollute operator map choices |

This is a catalog hygiene issue, not evidence that the deployed planner moved
or changed maps during the audit.

## Local Avoidance And Global Navigation Gap

Current deployed evidence shows the global planner can reach the saved-map
artifact gate but cannot proceed without odometry. It does not validate local
obstacle avoidance because live LiDAR/IMU samples and SLAM map-cloud output
were not observed in the limited typed probe window.

The next local-avoidance gate must prove:

- live `rt/lidar/raw_frame` and `rt/imu/raw` samples;
- SLAM odometry and map-cloud output;
- local obstacle points reaching the native navigation endpoint;
- stop/fail-closed behavior with a static obstacle and zero motion;
- only then, supervised low-speed obstacle avoidance.

The next global-navigation gate must prove:

- active map selection resolves to one canonical map record;
- the active map artifact gate passes;
- odometry is present in the same frame as the map;
- no-motion preview returns a valid path;
- only then, supervised navigation motion with an operator-held stop path.

## NO-GO Boundary

Do not run any of the following from this state:

- `map save`;
- `map end`;
- `nav start`;
- `nav goal`;
- teleop joystick commands;
- service restarts or Product switches that could change sensor ownership.

The current state is appropriate for read-only diagnostics only.

## Minimum Next Field Actions

1. Record the deployed release identity, active systemd units, and DDS domain
   before interpreting runtime probes.
2. Verify the MID-360 physical path: `eth1` carrier, expected interface IP,
   RX packet counters, and power.
3. Capture MID-360 generated JSON, discovery logs, and IMU-enable logs from the
   same diagnostic window.
4. Recheck the same-window data chain: raw packet input to `rt/lidar/raw_frame`,
   `rt/imu/raw`, SLAM odometry, and SLAM map cloud.
5. Confirm `/ready` no longer reports degraded readiness for non-motion
   planning.
6. Rerun the no-motion global preview and require a valid path instead of
   `odometry_missing`.
7. Run a supervised static-obstacle stop gate before any local avoidance motion.
8. Run supervised low-speed local avoidance only after the static stop gate
   passes.

## Evidence Boundary

This record describes the deployed Sunrise runtime observed through read-only
interfaces on 2026-07-27. It does not claim that local uncommitted source
changes in the development workspace were deployed, built, or validated on the
robot.

The following provenance was not archived with this note:

- exact Gateway URL and query parameters used for `services/status`;
- exact request time;
- DDS domain;
- typed probe sampling window;
- raw JSON response artifact;
- explicit proof that the DDS probe reported `checked=true`.

Because those artifacts are missing, this note must not be used to infer
continuous DDS silence, a persistent sensor outage, or a physical MID-360 root
cause.
