# Go2 six-piece MID-360 mount: installed configuration and stationary check

The operator confirmed the six-piece blue PLA mount, 35-degree forward pitch,
no additional spacers, a firmly attached stationary robot, no scans to preserve,
and permission to restart acquisition. The NX was reachable through the small
computer. ProductControl reported `stopped` before this work.

## Installed changes

- Keep the original Go2 URDF intact; add `go2_mid360_flatpack.urdf` with all six
  actual assembly meshes, the MID-360 optical frame and package IMU frame.
  Generate the browser tree from this installed model, not the earlier box/frame.
- `T_body_lidar`: translation `[0.240, 0, 0.20262] m`, RPY
  `[0, 0.6108652381980153, 0] rad`.
- `T_body_imu`: translation
  `[0.22370448011537075, 0.02329, 0.16016967100610818] m`, rotation `Ry(35 deg)`.
  This is derived from `T_body_lidar * inverse(T_imu_lidar)`.
- The factory/internal `r_il` and `t_il` are unchanged. The old filter state was
  not resumed; a new stationary mapping diagnostic initialized the IMU/SLAM.
- Upper collision clearance is `0.24 m` above the body origin, covering the
  nominal assembly top at `0.232 m`. Horizontal collision parameters are unchanged.
  SCAN does not consume the visual URDF collision links.

NX keeps native release `v2.3.0-go2.20260929.88`, built from
`bcc0eea8d5979fda91a74244c30a1a41042d3b99`. This is a configuration and visual-asset
update on that release, not a new binary/ABI build. Only the two Go2 configuration
files and the mount's browser/model changes are deployed; other worktree edits
are excluded. The browser build uses source retrieved from the installed NX
release plus the new generated model tree.

## Evidence

Existing `diagnostics.field.go2_mid360_no_motion`: 10-second settling period,
20-second observation window. It starts only sensor/SLAM processes and stops them
after the check. It does not start navd or the motion driver.

| Observation | Result |
|---|---|
| Raw LiDAR | 199 DDS samples, 10.00 Hz, 19,968 points in the final sampled frame |
| IMU | 3,998 DDS samples, 200.19 Hz |
| Odometry / registered cloud | 200 samples each, 10.00 Hz |
| SLAM | TRACKING, converged, no degeneracy reported |
| LiDAR / IMU dropped frames and timestamp rollbacks | All zero |
| Final velocity topic | Zero samples; no motion process started |
| Final body position relative to diagnostic initialization | Approximately `[0.0102, -0.0024, 0.0024] m` |

Plane inspection uses saved snapshots, fixed ROI `0.3 < x < 3 m`, `abs(y) < 1.5 m`,
`-0.8 < z < -0.15 m`, 500 seeded RANSAC trials and a 25 mm inlier band, followed
by a least-squares plane fit. This is an offline measurement, not a planner gate.
Zero-range returns and nonfinite values are excluded before fitting. The raw scan
had 2,691 zero/near-zero returns and no nonfinite coordinates; its 19,968 entries
must not be described as 19,968 valid obstacle/ground measurements.

| Snapshot | Plane points | Plane RMS | Tilt in its coordinate frame | Floor Z at origin |
|---|---:|---:|---:|---:|
| Raw scan transformed to body using new nominal mount | 1,562 | 5.64 mm | 1.94 deg | -0.3446 m |
| Registered body cloud | 745 | 4.94 mm | 2.04 deg | -0.3493 m |
| Gravity-aligned map cloud | 742 | 4.87 mm | 0.95 deg | -0.3458 m |

Within the forward strip `abs(y) < 0.4 m`, the nearest raw floor return was at
body X `0.798 m`; 289 floor returns lay before X `1 m`. The processed registered
snapshot starts at X `0.887 m`, with 89 floor returns before X `1 m`. These are
single-frame observations, not a guarantee that every intervening surface is seen.
There is still a near-body blind area. No unobserved ground was filled in.

RobotConfig's sensor-scoped `verified` record now refers to this installed nominal
mount and this stationary check, rather than the obsolete 13-degree installation.
It does not certify a fitted six-DoF extrinsic, vibration rigidity, exact yaw or
translation accuracy, long-term thermal stability, old-map relocalization, or
moving navigation. No existing map was modified and no motion goal was sent.
After the diagnostic, the Product remains stopped.

Local validation: 10 mount/configuration tests, 16 browser robot/joint/display
tests, generated-tree consistency and touched Python lint passed. TypeScript and
the isolated installed-source production build passed; Vite retained large-chunk
and plugin-timing warnings. The first isolated build lacked the guide entry;
retrieving the installed guide/docs completed that build without changing product
code. The old fixed seven-mesh assertion was updated to count distinct resources,
preserving its check that shared meshes load once.

## Files and recovery

Local evidence: `build/go2-mount-20260929/` (`run1/`, `analysis.json`, `analyze.py`,
`returns.png`, installed-source browser build and deployment bundle).
NX evidence and original configuration backups:
`/home/unitree/mount-check-20260929/` (`run1/`, `robot.before.yaml`,
`slam.before.yaml`, `web-before.tgz`).

Do not restore the old 13-degree configuration while the raised mount remains
installed. If the original mount is physically restored, stop the Product with
ProductControl before restoring its matching configuration and restarting SLAM.
Browser rollback can restore `web-before.tgz` independently; it does not change
the native sensor transform. Use ProductControl for subsequent map/nav sessions.

## Follow-up: position in the saved room map

The operator subsequently confirmed the robot remained in 903room and could
stay stationary. ProductControl activated `nav` on `903room_v4_maps84`; native
relocalization completed using the newly installed extrinsics. A 10-second,
20-sample observation had all input gates ready and all output commands zero;
position spans were X 8.9 mm, Y 5.1 mm and Z 3.2 mm. These are stability spans,
not absolute positioning accuracy.

The final session restores both SCAN planner and follower forward caps to
0.5 m/s, with planner acceleration 1.0 m/s²:
`product-8d2941fcb6774a6982581e3b66bcb90f`. ProductControl is `active`, HTTP
`/ready` is true, SLAM is TRACKING and has completed relocalization. Latest pose
is `[0.541996, -0.577704, 0.027584] m`, yaw `-9.0398 deg`, in the saved map's
coordinates. Subsequent map matching succeeded 18/18 times at the final read.
There is no active path and all sampled final velocity components are zero.
No navigation goal was submitted. This updates the earlier post-diagnostic
stopped state: the navigation stack is now running and waiting for an operator.

Evidence: `position.json`, `position-final.json`, `nav-switch-final.json` in the
same NX/local evidence folders. `position.png` shows the saved 3D map projected
for inspection, with the localized body origin and heading; it is not a costmap.
