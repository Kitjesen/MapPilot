# Native frame ownership review, 2026-09-29

## Changes

- Resolve map/body/scan transforms in the SLAM-owned `frames.hpp`. Use the same
  calculation for DDS observations and Host status snapshots.
- Export map-frame odometry with the existing synchronized body twist, including
  angular velocity. Python no longer rotates Fast-LIO diagnostic velocity or
  reconstructs map poses from independently received streams.
- Export the scan-time map pose and mounted LiDAR origin. Correct the Python
  observation contract: body-frame point coordinates do not imply that the ray
  starts at the body origin. Pose/transform agreement is still required.
- Remove the SLAM adapter's second TF history, NavSkills' TF composition, the
  browser's TF projection and the unused Gateway matrix cache. Gateway retains
  a diagnostic frame-tree mirror only.
- Prevent a later TF sample from transforming an already-map navd odometry
  sample a second time.

## Verification

- Native full rotation/translation, body tilt, mounted sensor origin, alignment
  update and missing-alignment cases: `test_map_frames`.
- Actual C++ status JSON and DDS map-observation conversion: `test_frame_export`.
  The exported JSON was also loaded by the Python adapter to verify map position,
  height, body linear/angular velocity and physical ray origin.
- Native input projection including map odometry followed by TF:
  `test_input_projector`.
- Python status adapter, navigation skills and map messages: 74 tests passed.
- Gateway frame diagnostics and map messages: 26 tests passed (overlaps the map
  message cases above).
- Browser scene telemetry: 9 tests passed; TypeScript and targeted lint passed.

These are local checks. The existing Windows full SLAM configuration could not
regenerate because GTSAM development files are unavailable. Production export
code and native input projection were compiled and exercised under WSL; this
does not constitute an ARM package build, simulation acceptance or field test.

## Scope and deployment

Install SLAM, Host and web from the same build. An old status snapshot without
resolved map data no longer causes Python/web to reconstruct it independently.
No native DDS layout, collision rule, support rule or motion limit changed.

Python transform utilities remain for development/compatibility and diagnostic
views. Perception still has its existing RGB-D synchronization and camera
projection code; this change does not claim to migrate that algorithm into C++.
SLAM's small alignment-update threshold and navd's large TF-jump handling are
also unchanged; this review has not established a replacement policy for them.

No robot services were restarted and no movement was commanded for this change.
