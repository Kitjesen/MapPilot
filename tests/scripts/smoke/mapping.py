#!/usr/bin/env python3
# Thunder compatibility smoke; run from repository root:
#   python tests/scripts/smoke/mapping.py
import sys, os, time
sys.path.insert(0, "src")
for d in ["src/perception", "src/decision"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.INFO)

from runtime.blueprints.profile_builder import blueprint_for_resolved_profile

print("=== Starting mapping mode ===")
bp = blueprint_for_resolved_profile("stub", dict(
    robot="stub", slam_profile="fastlio2",
    enable_native=False, enable_semantic=False, enable_gateway=False,
))
system = bp.build()
print("Modules: %d" % len(system.modules))

# Check SLAM module
slam = system.modules.get("SlamBridgeModule")
print("SlamBridgeModule: %s" % (slam is not None))

system.start()
print("System started. Waiting for SLAM + LiDAR data...")

# Monitor for 15 seconds
for i in range(15):
    time.sleep(1)
    if slam:
        odom_count = slam.odometry.msg_count
        cloud_count = slam.map_cloud.msg_count
        print("[%2ds] odom=%d  cloud=%d" % (i + 1, odom_count, cloud_count))
        if odom_count > 0 and cloud_count > 0:
            print("SLAM data flowing!")
            break

# Final status
print()
print("=== SLAM Status ===")
if slam:
    h = slam.health()
    slam_info = h.get("slam", {})
    print("Backend: %s" % slam_info.get("backend"))
    node_info = slam_info.get("node", {})
    print("Node running: %s  pid: %s" % (node_info.get("running"), node_info.get("pid")))
    if slam._pgo_node:
        ph = slam._pgo_node.health().get("native", {})
        print("PGO running: %s  pid: %s" % (ph.get("running"), ph.get("pid")))
    print("Odometry msgs: %d" % slam.odometry.msg_count)
    print("Cloud msgs: %d" % slam.map_cloud.msg_count)

    if slam.odometry.msg_count > 0:
        print("\nSLAM MAPPING: DATA FLOWING")
    else:
        print("\nSLAM MAPPING: NO DATA (check LiDAR + livox driver)")

system.stop()
print("DONE")
