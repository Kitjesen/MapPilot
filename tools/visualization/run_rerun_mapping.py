#!/usr/bin/env python3
"""Developer SLAM mapping tool with Rerun 3D visualization."""

# ruff: noqa: E402 - configure source paths before importing optional tool dependencies

import logging
import sys
import time

sys.path.insert(0, "src")

logging.basicConfig(level=logging.WARNING)

import rerun as rr

from lingtu.assembly.stacks.slam import slam as external_slam

rr.init("lingtu_mapping")
server_uri = rr.serve_grpc(grpc_port=9877)
rr.serve_web_viewer(open_browser=False, web_port=9090, connect_to=server_uri)
print("Rerun: http://localhost:9090")

bp = external_slam(
    "native_dds",
    localization_adapter="cpp_slam_status",
)
system = bp.build()

slam = system.modules["SlamAdapterModule"]


def on_cloud(frame):
    if frame.points is not None and len(frame.points) > 0:
        rr.log("world/point_cloud", rr.Points3D(frame.points[:, :3], radii=0.02))


def on_odom(odom):
    pos = [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z]
    rr.log("world/robot", rr.Points3D([pos], radii=0.15, colors=[[255, 0, 0]]))


slam.map_cloud_frame._add_callback(on_cloud)
slam.odometry._add_callback(on_odom)
print("Rerun callbacks hooked to external C++ slamd adapter")

system.start()
print("Adapter started; external C++ slamd must already be running. Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
        odom_count = slam.odometry.msg_count
        cloud_count = slam.map_cloud_frame.msg_count
        print("odom=%d cloud=%d" % (odom_count, cloud_count))
        if odom_count > 0 and cloud_count > 0:
            print("DATA FLOWING; open http://localhost:9090")
except KeyboardInterrupt:
    pass

system.stop()
print("DONE")
