# Experimental semantic camera

`semantic_camera@1.0.0`: 64x48 pinhole at 10 Hz, `fx=fy=32`,
`cx=32`, `cy=24` pixels (edge coordinates), optical +X right/+Y down/+Z forward.
This is calibrated simulation ground truth, not RGB segmentation or a learned
perception result. Label noise is disabled, seed 0.

The consuming rig supplies the mount on a robot with physical ancestor
`base_link`. The selected World must explicitly declare `measurements` and
`semantic_labels` keyed by actual World MJCF geom names, with positive
`class_id`/`instance_id` and one class per instance. No labels are guessed from
mesh names or colors.

This camera shares the new thermal/semantic/acoustic acknowledged native
control and raw NPZ archive path; see the
[runtime status](../../../../runtime/sensors/README.md). Camera gimbal control
affects sampling; semantic label noise must stay disabled. The real HTTP-route
→ native DDS → MuJoCo control fixture exercised thermal, not a semantic-camera
Factory Product run. UE viewing/controls, the selected Fab physical world and
real Product inspection acceptance remain unfinished. Legacy RGB/Depth/LiDAR/IMU
are not yet all part of this unified remote-control path.
