import pytest

pytestmark = [pytest.mark.ros2]

"""Full integration verification 闂?36 tests across all new modules.

TODO: Replace time.sleep with threading.Event for module output
      synchronization. Many sleeps wait for module threads to produce
      results; using Events would be faster and more deterministic.
"""
import io
import json
import logging
import os
import sys
import time

import numpy as np

sys.path.insert(0, "src")
for d in ["src/perception", "src/decision"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
for k in ["MOONSHOT_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY", "DASHSCOPE_API_KEY"]:
    os.environ.pop(k, None)
logging.basicConfig(level=logging.WARNING)

results = []
def T(name, ok):
    results.append((name, ok))
    print(f'  {"PASS" if ok else "FAIL"} {name}')

from runtime.msgs.geometry import Pose, Vector3
from runtime.msgs.nav import OccupancyGrid, Odometry
from runtime.msgs.semantic import Detection3D, Region, SceneGraph
from runtime.msgs.sensor import PointCloud2

# ============================================================
print("=== 1-12: Individual Modules ===")
# ============================================================

# 1
from memory.modules.semantic_mapper_module import SemanticMapperModule

m = SemanticMapperModule(save_dir="/tmp/v_test1")
m.setup()
m._on_odom(Odometry(pose=Pose(position=Vector3(0, 0, 0))))
sg = SceneGraph(objects=[Detection3D(id="1", label="desk")],
                regions=[Region(name="office", object_ids=["1"], center=Vector3(1, 0, 0))])
m._on_scene_graph(sg)
T("1.SemanticMapper KG", "office" in (m._kg.room_types if m._kg else []))
T("1.SemanticMapper TSG", len(m._tsg.rooms) > 0 if m._tsg else False)
T("1.SemanticMapper count", m._sg_count > 0)

# 2
from memory.modules.vector_memory_module import VectorMemoryModule

v = VectorMemoryModule()
v.setup()
v._robot_xy = (15, 5)
v._store_snapshot(["backpack", "bench"])
T("2.VectorMemory query", json.loads(v.query_location("find backpack"))["found"])
T("2.VectorMemory stats", json.loads(v.get_memory_stats())["entries"] == 1)

# 3
from decision.modules.visual_servo_module import VisualServoModule

vs = VisualServoModule()
vs.setup()
vs._on_servo_target("find:chair")
T("3.VisualServo find", vs._mode == "find")
vs._on_servo_target("follow:person")
T("3.VisualServo follow", vs._mode == "follow")
vs._on_servo_target("stop")
T("3.VisualServo stop", vs._mode == "idle")
vs._engage_servo()
T("3.VisualServo engage", vs._servo_active)
vs._release_servo()
T("3.VisualServo release", not vs._servo_active)

# 4
from drivers.real.teleop_module import TeleopModule

tp = TeleopModule(release_timeout=0.01)
tp.setup()
T("4.Teleop ports", "cmd_vel" in tp.ports_out)
tp._on_joy({"lx": 0.5})
T("4.Teleop active", tp._active)
time.sleep(0.012)
tp._check_idle()
T("4.Teleop idle", not tp._active)

# 5
from decision.tasking.agent_loop import AGENT_TOOLS, AgentLoop

r = AgentLoop._parse_text_tool_call('{"tool":"navigate_to","args":{"x":5,"y":3}}')
T("5.AgentLoop parse", "tool_calls" in r)
r2 = AgentLoop._parse_text_tool_call("no json here")
T("5.AgentLoop no-json", "content" in r2)
T("5.AgentLoop tool registry", len(AGENT_TOOLS) >= 7)

# 6
from nav.mission.navigation import Navigation

nm = Navigation()
T("6.Nav no ros2 bridge state", not hasattr(nm, "_enable_ros2_bridge"))

# 7
from nav.services.plan.global_planner.service import GlobalPlanner

gps = GlobalPlanner()
class MockB:
    _grid = np.zeros((100, 100), dtype=np.float32)
    _resolution = 0.2
    _origin = np.array([0.0, 0.0])
    def plan(self, s, g): return [s, g]
gps._backend = MockB()
g = np.array([5.0, 5.0, 0.0])
T("7.GPS safe free", np.allclose(gps._find_safe_goal(g)[:2], g[:2]))
gps._backend._grid[25, 25] = 100
r = gps._find_safe_goal(g)
T("7.GPS safe obstacle", r is not None and not np.array_equal(r[:2], g[:2]))

# 8
from nav.services.map_layers.occupancy_grid_module import OccupancyGridModule

og = OccupancyGridModule()
og.setup()
og._on_odom(Odometry(pose=Pose(position=Vector3(0, 0, 0))))
og._on_cloud(PointCloud2(points=np.random.randn(100, 3).astype(np.float32) * 5))
T("8.OccupancyGrid", og.occupancy_grid.msg_count > 0)

# 9
from nav.services.map_layers.esdf_module import ESDFModule

es = ESDFModule()
es.setup()
grid = np.zeros((50, 50), dtype=np.int8)
grid[20:30, 20:30] = 100
es._on_grid(OccupancyGrid(grid=grid, resolution=0.2))
T("9.ESDF", es.esdf.msg_count > 0)

# 10
from nav.services.map_layers.elevation_map_module import ElevationMapModule

em = ElevationMapModule()
em.setup()
em._on_odom(Odometry(pose=Pose(position=Vector3(0, 0, 0))))
em._on_cloud(PointCloud2(points=np.random.randn(100, 3).astype(np.float32) * 5))
T("10.ElevationMap", em.elevation_map.msg_count > 0)

# 11
from localization.bridge import SlamBridgeModule

sb = SlamBridgeModule()
sb.setup()
T("11.SlamBridge state attrs", hasattr(sb, "_reader") and hasattr(sb, "_rclpy_node"))
T("11.SlamBridge ports", "map_cloud" in sb.ports_out and "odometry" in sb.ports_out)
try:
    sb.stop()
except Exception:
    pass

# ============================================================
print("\n=== 21-30: Cross-Module (dev profile) ===")
# ============================================================

from runtime.blueprints.full_stack import full_stack_blueprint

buf = io.StringIO()
h = logging.StreamHandler(buf)
h.setLevel(logging.WARNING)
logging.getLogger("runtime.blueprint").addHandler(h)

bp = full_stack_blueprint(robot="stub", slam_profile="none",
                          enable_native=False, enable_semantic=True, enable_gateway=True)
system = bp.build()
system.start()
time.sleep(0.06)
planner = system.modules["SemanticPlannerModule"]
nav = system.modules["nav.mission"]
vmem2 = system.modules["VectorMemoryModule"]
conns = system.connections

# 21
planner.scene_graph._deliver(SceneGraph(objects=[Detection3D(id="1", label="x")],
    regions=[Region(name="r", object_ids=["1"], center=Vector3(0, 0, 0))]))
time.sleep(0.03)
T("21.SG delivered", planner.scene_graph.msg_count > 0)

# 22
T("22.on_system vmem", planner._vector_memory is not None)
T("22.on_system tagged", planner._tagged_locations is not None)

# 23
T("23.Safety->Nav", any("nav.safety" in c[0] and "nav.mission" in c[2] for c in conns))
T("23.Safety->Driver", any("nav.safety" in c[0] and "Stub" in c[2] for c in conns))

# 24
T("24.instr GW->Planner", any("Gateway" in c[0] and "SemanticPlanner" in c[2] and "instruction" in c[1] for c in conns))
T("24.instr MCP->Planner", any("MCP" in c[0] and "SemanticPlanner" in c[2] and "instruction" in c[1] for c in conns))

# 25
T("25.goal Planner->Nav",
  any("SemanticPlanner" in c[0] and "nav.mission" in c[2] and "goal_pose" in c[1] for c in conns))

# 26 Fast Path
for mod in system.modules.values():
    if hasattr(mod, "odometry") and hasattr(mod.odometry, "_deliver"):
        mod.odometry._deliver(Odometry(pose=Pose(position=Vector3(0, 0, 0))))
chair = Detection3D(id="c", label="chair", confidence=0.9)
chair.position = Vector3(8, 2, 0)
planner.scene_graph._deliver(SceneGraph(objects=[chair]))
time.sleep(0.05)
prev = nav.goal_pose.msg_count
planner.instruction._deliver("go to the chair")
time.sleep(0.06)
T("26.FastPath->Nav", nav.goal_pose.msg_count - prev > 0)

# 27 Vector Memory
vmem2._robot_xy = (20, 10)
vmem2._store_snapshot(["fire extinguisher", "hose"])
prev = nav.goal_pose.msg_count
planner.instruction._deliver("find fire extinguisher")
time.sleep(0.3)  # longer wait for vector memory + goal resolve chain; TODO: migrate to threading.Event
T(
    "27.VectorMem->Nav",
    nav.goal_pose.msg_count - prev > 0
    or getattr(planner, "_last_vector_memory_query_only", False),
)

# 28 Zero ambiguity
warnings = [line for line in buf.getvalue().split("\n") if "ambiguity" in line.lower()]
T("28.Zero ambiguity", len(warnings) == 0)

# 29 MCP skills
mcp = system.modules.get("MCPServerModule")
T("29.MCP skills", mcp is not None and len(getattr(mcp, "_tool_list", [])) > 0)

# 30 Profile builds
for pname, kw in [
    ("stub", dict(robot="stub", slam_profile="none",
                  enable_native=False, enable_semantic=False, enable_gateway=True)),
    ("sim", dict(robot="sim_mujoco", slam_profile="bridge",
                 enable_native=True, enable_semantic=True, enable_gateway=True))]:
    try:
        full_stack_blueprint(**kw).build()
        T(f"30.Profile {pname}", True)
    except Exception:
        T(f"30.Profile {pname}", False)

system.stop()

# ============================================================
print("\n=== 31-36: Persistence + Edge Cases ===")
# ============================================================

# 31
m2 = SemanticMapperModule(save_dir="/tmp/v_test1")
m2.setup()
T("31.Persist KG", len(m2._kg.room_types) > 0 if m2._kg else False)

# 33
T("33.No API key GoalResolver", planner._goal_resolver is not None)

# 35
planner.scene_graph._deliver(SceneGraph())
T("35.Empty SG no crash", True)

# 36
from nav.mission.tracking.waypoint_tracker import EV_STUCK, EV_STUCK_WARN, WaypointTracker

wt = WaypointTracker(stuck_timeout=0.05, stuck_dist=0.01)
wt.reset([np.array([5, 5, 0]), np.array([10, 10, 0])], np.array([0, 0, 0]))
time.sleep(0.025)
s = wt.update(np.array([0.0, 0.0, 0.0]))
got_stuck = s.event in (EV_STUCK, EV_STUCK_WARN)
if not got_stuck:
    time.sleep(0.025)
    s = wt.update(np.array([0.0, 0.0, 0.0]))
    got_stuck = s.event in (EV_STUCK, EV_STUCK_WARN)
T("36.WaypointTracker stuck", got_stuck)

# ============================================================
print()
_passed = sum(1 for _, ok in results if ok)
_failed = sum(1 for _, ok in results if not ok)
print(f"========== {_passed} PASSED, {_failed} FAILED / {len(results)} total ==========")
if _failed:
    for name, ok in results:
        if not ok:
            print(f"  FAIL: {name}")

try:
    from runtime.adapters.ros2.context import shutdown_shared_executor

    shutdown_shared_executor()
    import rclpy

    if rclpy.ok():
        rclpy.shutdown()
except Exception:
    pass


def test_integration_36_all_pass() -> None:
    """Pytest entry point: assert every individual module integration check passed."""
    failed_tests = [(name, ok) for name, ok in results if not ok]
    assert not failed_tests, (
        "%d/%d integration checks FAILED:\n%s"
        % (len(failed_tests), len(results), "\n".join(f"  FAIL: {n}" for n, _ in failed_tests))
    )
