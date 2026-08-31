#!/usr/bin/env python3
# Thunder compatibility smoke; run from repository root:
#   python tests/scripts/smoke/mapping.py
# ruff: noqa: E402 - configure the repository source path before imports
import logging
import sys
import time

sys.path.insert(0, "src")

logging.basicConfig(level=logging.INFO)

from lingtu.assembly.stacks.slam import slam as external_slam

print("=== Observing external C++ mapping runtime ===")
bp = external_slam(
    "native_dds",
    localization_adapter="cpp_slam_status",
)
system = bp.build()
print("Modules: %d" % len(system.modules))

slam = system.modules["SlamAdapterModule"]
print("External SLAM adapter: yes")

system.start()
print("Adapter started. Waiting for an already-running C++ slamd...")

# Monitor for 15 seconds
for i in range(15):
    time.sleep(1)
    odom_count = slam.odometry.msg_count
    cloud_count = slam.map_cloud_frame.msg_count
    print("[%2ds] odom=%d  cloud=%d" % (i + 1, odom_count, cloud_count))
    if odom_count > 0 and cloud_count > 0:
        print("SLAM data flowing!")
        break

# Final status
print()
print("=== SLAM Status ===")
h = slam.health()
print("Adapter backend: %s" % h.get("configured_backend"))
print("Transport: %s" % h.get("transport"))
print("Status snapshot stale: %s" % h.get("status_snapshot_stale"))
print("Odometry msgs: %d" % slam.odometry.msg_count)
print("Cloud msgs: %d" % slam.map_cloud_frame.msg_count)

if slam.odometry.msg_count > 0:
    print("\nSLAM MAPPING: DATA FLOWING")
else:
    print("\nSLAM MAPPING: NO DATA (check external slamd + LiDAR runtime)")

system.stop()
print("DONE")
