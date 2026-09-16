# Go2 built-in LiDAR capture

Read-only acquisition for supplementary-LiDAR calibration. Uses the existing
Unitree SDK2 and native CycloneDDS. No ROS2 runtime, publishers, motion clients,
Product switches, or writes to active maps are involved.

On NX with the installed SDK:

```sh
cmake -S tools/diagnostics/go2_lidar -B build/go2_lidar -DCMAKE_PREFIX_PATH=/opt/unitree_robotics
cmake --build build/go2_lidar -j2
build/go2_lidar/go2_lidar_capture eth0 12 /tmp/go2-lidar-capture-01
```

The parent directory must exist and the output directory must be new. Duration
is 1–60 seconds; the DDS domain is 0, matching the Go2 factory topics. Output
contains the last finite XYZ cloud from each of `cloud`, `cloud_base`, and
`cloud_deskewed`, plus frame IDs, timestamps, observed rates, and LidarState.
It also records the NX wall-clock arrival time and arrival minus header time.
That difference includes clock offset as well as latency; it is not a direct
measurement of transmission delay.
PCD points stay in their reported frames: no alignment or fusion is performed.
Exit 2 means at least one cloud topic produced no sample; 1 means an error.

An optional final argument accepts the existing Fast-LIO cloud snapshot path
(normally `/dev/shm/lingtu_slam/registered_cloud.bin`). A readable snapshot is
copied immediately after freezing the captured clouds and before DDS shutdown.
Use an appropriately privileged diagnostic terminal if access is required.
Compare the embedded timestamps; this copy is not sensor synchronization.
The optional snapshot keeps LingTu's existing binary format.

Field validation on 2026-09-14: built on NX/aarch64 with the installed SDK,
received 184 samples per cloud topic in 12 seconds, approximately 15.36 Hz.
Frames were `utlidar_lidar`, `base_link`, and factory `odom`; LidarState error
was 0. Its firmware/SDK strings were empty, so this does not identify L1/L2.
The clock probe observed arrival minus header time of approximately 5.13 s
while NX reported `NTPSynchronized=no`. Clock alignment remains unresolved.

This is acquisition evidence, not permission to merge factory odometry into
LingTu. Check extrinsics, scan timing, self returns, observation origins, and
the saved-map path before integrating. See the
[Go2 field notes](../../../config/robots/unitree/go2/README.md).
