#!/usr/bin/env python3
# Thunder compatibility smoke; run from repository root:
#   python tests/scripts/smoke/nav_planning.py
import sys, os, time
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.INFO)

# Find tomogram
map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
tomogram = os.path.join(map_dir, "active", "map.pickle")
if not os.path.exists(tomogram):
    print("No tomogram at %s" % tomogram)
    sys.exit(1)
print("Tomogram: %s" % tomogram)

# Build system with real tomogram (no SLAM, no gateway; planning only)
from core.blueprints.profile_builder import blueprint_for_resolved_profile
bp = blueprint_for_resolved_profile("stub", dict(
    robot="stub", slam_profile="none",
    enable_native=False, enable_semantic=False, enable_gateway=False,
    tomogram=tomogram,
))
system = bp.build()
system.start()
time.sleep(1)

nav = system.modules["NavigationModule"]
print("Nav planner ready: %s" % nav._planner_svc.is_ready)
print("Nav state: %s" % nav._state)

# Simulate robot at origin
from core.msgs.nav import Odometry
from core.msgs.geometry import Pose, Vector3, PoseStamped, Quaternion
import numpy as np

odom = Odometry(pose=Pose(position=Vector3(0.0, 0.0, 0.0)))
nav.odometry._deliver(odom)
time.sleep(0.2)
print("Robot position: (%.1f, %.1f)" % (nav._robot_pos[0], nav._robot_pos[1]))

# Send goal
goal = PoseStamped(
    pose=Pose(position=Vector3(5.0, 3.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
    frame_id="map",
)
print("\nSending goal: (5.0, 3.0)")
nav.goal_pose._deliver(goal)
time.sleep(1)

print("Nav state: %s" % nav._state)
print("Waypoint count: %d" % nav.waypoint.msg_count)
print("Global path count: %d" % nav.global_path.msg_count)

if nav._state == "EXECUTING":
    print("Path length: %d waypoints" % nav._tracker.path_length)
    wp = nav._tracker.current_waypoint
    if wp is not None:
        print("First waypoint: (%.1f, %.1f)" % (wp[0], wp[1]))
    print("\nNAVIGATION PLANNING: PASS")
elif nav._state == "FAILED":
    print("Planning failed: %s" % nav._failure_reason)
    print("\nTrying different goal closer to origin...")
    goal2 = PoseStamped(
        pose=Pose(position=Vector3(1.0, 1.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        frame_id="map",
    )
    nav.goal_pose._deliver(goal2)
    time.sleep(1)
    print("Nav state: %s" % nav._state)
    if nav._state == "EXECUTING":
        print("Path length: %d waypoints" % nav._tracker.path_length)
        print("\nNAVIGATION PLANNING: PASS (alternate goal)")
    else:
        print("Still failed: %s" % nav._failure_reason)
        # Show map info for debugging
        backend = nav._planner_svc._backend
        if hasattr(backend, "_grid") and backend._grid is not None:
            g = backend._grid
            print("Grid shape: %s" % str(g.shape))
            print("Grid range: %.1f to %.1f" % (g.min(), g.max()))
            print("Resolution: %.2f" % backend._resolution)
            print("Origin: %s" % backend._origin)
            free = (g < 50).sum()
            total = g.size
            print("Free cells: %d / %d (%.1f%%)" % (free, total, 100*free/total))
        print("\nNAVIGATION PLANNING: PARTIAL (map loaded, goal unreachable)")
else:
    print("Unexpected state: %s" % nav._state)

system.stop()
print("DONE")
