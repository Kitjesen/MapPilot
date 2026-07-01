# ROS2 + CycloneDDS Tuning Notes

> Configuration reference for the topics the navigation stack publishes /
> subscribes through CycloneDDS. No code changes are required.
>
> Production config files live at `config/cyclonedds.xml` and
> `config/qos_profiles.yaml`. See also `config/fastdds_no_shm.xml` for the
> Fast-DDS fallback we ship for sites that cannot use shared memory.

## 1. Enabling CycloneDDS

```bash
# Ubuntu 22.04 + ROS 2 Humble
sudo apt install ros-humble-rmw-cyclonedds-cpp

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml
ros2 doctor --report | grep -i middleware   # expect: rmw_cyclonedds_cpp
```

On S100P, CycloneDDS is built from source under `~/cyclonedds/install/` (the
Unitree workflow) and the env vars are set by `lingtu.service`.

## 2. QoS Recipe Per Channel

This matches the channels the gateway and bridges actually emit. Reliability
mismatches will silently kill subscriptions — confirm with `ros2 topic info -v`.

| Topic | Reliability | History | Depth | Lifespan |
|-------|-------------|---------|-------|----------|
| `/livox/lidar`, `/nav/map_cloud` (PointCloud2) | BEST_EFFORT | KEEP_LAST | 2 | 200 ms |
| `/nav/odometry`, `/nav/imu` | BEST_EFFORT | KEEP_LAST | 5 | 50 ms |
| `/nav/cmd_vel`, `/nav/stop` | RELIABLE | KEEP_LAST | 1 | — |
| `/nav/global_path` | RELIABLE | TRANSIENT_LOCAL | 1 | — |
| `/nav/health`, `/nav/session_state` | RELIABLE | TRANSIENT_LOCAL | 1 | — |
| `/nav/goal_pose`, `/nav/geofence` | RELIABLE | TRANSIENT_LOCAL | 1 | — |
| `/camera/.../compressed` | BEST_EFFORT | KEEP_LAST | 1 | 100 ms |
| TF | (ROS 2 default) | — | — | — |

Rules of thumb that drive this table:

- Sensor streams: lossy is fine, drop staleness fast (`lifespan`).
- Control / safety: must arrive — RELIABLE with a tiny depth.
- "Latched" state (path, geofence, last health): TRANSIENT_LOCAL so a late
  subscriber sees the latest value without a republish.

## 3. Compatibility Matrix (Why rviz Sometimes Sees Nothing)

```
Pub \ Sub        RELIABLE   BEST_EFFORT
RELIABLE         OK         OK
BEST_EFFORT      mismatch   OK

Pub \ Sub        T_LOCAL    VOLATILE
T_LOCAL          OK         OK
VOLATILE         mismatch   OK
```

If `rviz2` is silent on a topic, run `ros2 topic info -v /<topic>` and compare
the publisher's QoS with the subscriber's. Almost every "topic missing" report
on S100P traced back to a BEST_EFFORT publisher with a RELIABLE subscriber.

## 4. Profiling

```bash
ros2 topic delay /nav/odometry         # uses header.stamp
ros2 topic hz    /nav/map_cloud        # frequency
ros2 topic bw    /nav/map_cloud        # throughput
```

Targets we keep on S100P:

| Metric | Target | Alarm |
|--------|--------|-------|
| LiDAR rate | ≥ 10 Hz | < 8 Hz |
| Odometry rate | ≥ 100 Hz | < 80 Hz |
| `cmd_vel` end-to-end | < 20 ms | > 50 ms |
| terrain_map age | < 50 ms | > 100 ms |
| Discovery convergence | < 3 s | > 10 s |
| CycloneDDS thread CPU | < 5 % | > 15 % |

For deeper debugging, enable Cyclone tracing:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing>
  <Verbosity>fine</Verbosity>
  <OutputFile>/tmp/cyclonedds.log</OutputFile>
</Tracing></Domain></CycloneDDS>'
grep -E "matched|retransmit|drop" /tmp/cyclonedds.log
```

## 5. Network Topologies

| Layout | Config |
|--------|--------|
| Single host (dev) | `<Interfaces><NetworkInterface name="lo"/></Interfaces>` + `AllowMulticast=false` |
| Same subnet (default) | `<NetworkInterface autodetermine="true"/>` + multicast enabled |
| Cross-subnet / peer mode | List `<Peer address="..."/>` under `<Discovery>`, disable multicast |
| Multi-robot | Distinct `ROS_DOMAIN_ID` per robot (e.g. 42, 43, …) |

If sharing the AP with non-ROS traffic, raise `SocketReceiveBufferSize` in
`cyclonedds.xml` to avoid PointCloud2 stalls.

## 6. Optional Local Bypass: Iceoryx Shared Memory

CycloneDDS ships with iceoryx; enable it when both endpoints are on the same
host and the data is large (point clouds):

```xml
<SharedMemory>
  <Enable>true</Enable>
  <LogLevel>warning</LogLevel>
</SharedMemory>
```

Expected effect: PointCloud2 transfer drops from ~5 ms to ~0.1 ms (zero copy).
S100P deployment ships with this off by default — turn it on per-site after
verifying iceoryx-rt is running.

## 7. Common Failure Modes

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Two nodes can't see each other's topics | Different RMW | Confirm `RMW_IMPLEMENTATION` everywhere |
| rviz2 quiet on one topic | QoS mismatch | `ros2 topic info -v` |
| Discovery never completes | Multicast blocked | Open UDP 7400-7500 in firewall, or peer mode |
| PointCloud2 latency spikes | Receive buffer too small | Raise `SocketReceiveBufferSize` |
| Multi-NIC chaos | DDS bound to wrong NIC | Pin `<NetworkInterface name="..."/>` |
| Docker peers can't see host | Network mode | `network_mode: host` or peer-list config |

## 8. Where to Look in the Tree

| File | Purpose |
|------|---------|
| `config/cyclonedds.xml` | Active runtime config, sourced by `lingtu.service` |
| `config/fastdds_no_shm.xml` | Fast-DDS fallback (no shared memory) |
| `config/qos_profiles.yaml` | Per-channel QoS that the C++ nodes load |
| `src/runtime/dds.py` | Python-side cyclonedds bindings used by `nova_nav_bridge.py` |
| `src/gateway/gateway_module.py` | Re-publishes a subset over WS/SSE for the web UI |
