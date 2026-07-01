#!/usr/bin/env python3
"""SLAM mapping with Rerun 3D visualization via the shared profile builder."""

import logging
import os
import sys
import time

sys.path.insert(0, "src")
for path in ("src/semantic/perception", "src/semantic/planner", "src/semantic/common"):
    if os.path.isdir(path):
        sys.path.insert(0, path)

logging.basicConfig(level=logging.WARNING)

import rerun as rr
from core.blueprints.profile_builder import blueprint_for_resolved_profile

rr.init("lingtu_mapping")
server_uri = rr.serve_grpc(grpc_port=9877)
rr.serve_web_viewer(open_browser=False, web_port=9090, connect_to=server_uri)
print("Rerun: http://localhost:9090")

bp = blueprint_for_resolved_profile(
    "stub",
    {
        "robot": "stub",
        "slam_profile": "fastlio2",
        "enable_native": False,
        "enable_semantic": False,
        "enable_gateway": False,
        "enable_map_modules": False,
    },
)
system = bp.build()

bridge = system.modules.get("SlamBridgeModule")


def on_cloud(cloud):
    if cloud.points is not None and len(cloud.points) > 0:
        rr.log("world/point_cloud", rr.Points3D(cloud.points[:, :3], radii=0.02))


def on_odom(odom):
    pos = [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z]
    rr.log("world/robot", rr.Points3D([pos], radii=0.15, colors=[[255, 0, 0]]))


if bridge:
    bridge.map_cloud._add_callback(on_cloud)
    bridge.odometry._add_callback(on_odom)
    print("Rerun callbacks hooked to SlamBridgeModule")

system.start()
print("System started. Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
        if bridge:
            odom_count = bridge.odometry.msg_count
            cloud_count = bridge.map_cloud.msg_count
            print("odom=%d cloud=%d" % (odom_count, cloud_count))
            if odom_count > 0 and cloud_count > 0:
                print("DATA FLOWING; open http://localhost:9090")
except KeyboardInterrupt:
    pass

system.stop()
print("DONE")
