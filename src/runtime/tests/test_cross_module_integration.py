"""Cross-module integration tests for LingTu navigation system (tests 21-30)."""

import os
import sys

sys.path.insert(0, "src")
for d in ["src/perception", "src/decision"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
for k in ["MOONSHOT_API_KEY", "OPENAI_API_KEY", "ANTHROPIC_API_KEY", "DASHSCOPE_API_KEY"]:
    os.environ.pop(k, None)
import logging

logging.basicConfig(level=logging.ERROR)

import threading
import time

import numpy as np

# Capture warnings during build for test 30
_build_warnings = []
_orig_warn = logging.Logger.warning
def _capture_warn(self, msg, *args, **kwargs):
    try:
        formatted = str(msg) % args if args else str(msg)
    except Exception:
        formatted = str(msg)
    _build_warnings.append(formatted)
    _orig_warn(self, msg, *args, **kwargs)
logging.Logger.warning = _capture_warn

# Patch server start methods to avoid port binding / blocking in CI
import drivers.real.teleop_module as _tp_mod
import gateway.gateway_module as _gw_mod
import gateway.mcp_server as _mcp_mod

_orig_gw_start = _gw_mod.GatewayModule.start
_orig_tp_start = _tp_mod.TeleopModule.start
_orig_mcp_start = _mcp_mod.MCPServerModule.start
_gw_mod.GatewayModule.start = lambda self: None
_tp_mod.TeleopModule.start = lambda self: None
_mcp_mod.MCPServerModule.start = lambda self: None

from runtime.blueprints.full_stack import full_stack_blueprint

bp = full_stack_blueprint(
    robot="stub", slam_profile="none",
    enable_native=False, enable_semantic=True, enable_gateway=True,
)
system = bp.build()
system.start()
time.sleep(0.3)
_gw_mod.GatewayModule.start = _orig_gw_start
_tp_mod.TeleopModule.start = _orig_tp_start
_mcp_mod.MCPServerModule.start = _orig_mcp_start
logging.Logger.warning = _orig_warn

results = {}
def test(num, name, passed, detail=""):
    tag = "PASS" if passed else "FAIL"
    results[num] = passed
    msg = "Test %d: %s -- %s" % (num, tag, name)
    if detail and not passed:
        msg += "  [%s]" % detail
    print(msg, flush=True)

# ==== Test 20: Non-native stack builds Python autonomy chain ====
try:
    nav = system.get_module("nav.mission")
    lp = system.get_module("nav.local_planner")
    pf = system.get_module("nav.path_follower")
    wp_conns = [c for c in system.connections
                if c[0] == "nav.mission" and c[1] == "waypoint"
                   and c[2] == "nav.local_planner" and c[3] == "waypoint"]
    path_conns = [c for c in system.connections
                  if c[0] == "nav.local_planner" and c[1] == "local_path"
                     and c[2] == "nav.path_follower" and c[3] == "local_path"]
    # cmd_vel now goes through VelocityMux for priority arbitration:
    #   nav.path_follower.cmd_vel 闁?VelocityMux.path_follower_cmd_vel
    #   nav.velocity_mux.driver_cmd_vel   闁?StubDogModule.cmd_vel
    cmd_conns_to_mux = [c for c in system.connections
                        if c[0] == "nav.path_follower" and c[1] == "cmd_vel"
                           and c[2] == "nav.velocity_mux"]
    mux_to_driver = [c for c in system.connections
                     if c[0] == "nav.velocity_mux" and c[1] == "driver_cmd_vel"
                        and c[2] == "StubDogModule" and c[3] == "cmd_vel"]
    cmd_conns = cmd_conns_to_mux if (cmd_conns_to_mux and mux_to_driver) else []
    test(20, "Non-native stack uses Python autonomy chain",
         not hasattr(nav, "_enable_ros2_bridge")
         and lp._backend in ("nanobind", "cmu_py", "simple")
         and pf._backend in ("nav_kernel", "pid")
         and len(wp_conns) > 0
         and len(path_conns) > 0
         and len(cmd_conns) > 0,
         "nav_has_ros2_attr=%s lp=%s pf=%s wires=%d/%d/%d" % (
             hasattr(nav, "_enable_ros2_bridge"), lp._backend, pf._backend,
             len(wp_conns), len(path_conns), len(cmd_conns)))
except Exception as e:
    test(20, "Non-native stack uses Python autonomy chain", False, str(e))

# ==== Test 21: SceneGraph fan-out ====
# PerceptionModule (scene_graph Out) is not in full_stack; DetectorModule
# publishes detections, not scene_graph.  So we simulate fan-out by creating
# a temporary Out[SceneGraph], wiring it to every In[SceneGraph] in the system,
# publishing once, and verifying all targets receive the message.
try:
    from runtime.msgs.semantic import SceneGraph
    from runtime.stream import Out as OutPort
    sg = SceneGraph(objects=[], relations=[], regions=[])

    expected_targets = {"SemanticPlannerModule", "SemanticMapperModule",
                        "EpisodicMemoryModule", "VisualServoModule"}
    # Collect In ports
    target_ports = {}
    for tname in expected_targets:
        if tname in system.modules:
            mod = system.modules[tname]
            if "scene_graph" in mod.ports_in:
                target_ports[tname] = mod.ports_in["scene_graph"]

    # Create a temporary Out and wire to all targets (simulating fan-out)
    fake_out = OutPort(SceneGraph, "scene_graph_test_source")
    for _tname, in_port in target_ports.items():
        fake_out._add_callback(in_port._deliver)

    before = {tn: tp.msg_count for tn, tp in target_ports.items()}
    fake_out.publish(sg)
    time.sleep(0.05)

    received = 0
    for tname, bc in before.items():
        if target_ports[tname].msg_count > bc:
            received += 1

    test(21, "SceneGraph fan-out",
         received >= 2 and len(target_ports) >= 2,
         "delivered to %d/%d targets (%s)" % (received, len(target_ports),
                                               list(target_ports.keys())))
except Exception as e:
    test(21, "SceneGraph fan-out", False, str(e))

# ==== Test 22: Odometry fan-out ====
try:
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry
    odom = Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        twist=Twist(linear=Vector3(0.1, 0, 0), angular=Vector3(0, 0, 0)),
    )
    odom_conns = [c for c in system.connections if c[1] == "odometry" and c[3] == "odometry"]
    odom_targets = set(c[2] for c in odom_conns)
    expected_odom = {"nav.mission", "SemanticMapperModule", "VectorMemoryModule"}
    found_odom = odom_targets & expected_odom

    odom_sources = set(c[0] for c in odom_conns)
    if odom_sources:
        src_name = next(iter(odom_sources))
        src_port = system.modules[src_name].ports_out["odometry"]
        before = {}
        for tname in found_odom:
            if tname in system.modules:
                before[tname] = system.modules[tname].ports_in["odometry"].msg_count
        src_port.publish(odom)
        time.sleep(0.05)
        received = 0
        for tname, bc in before.items():
            if system.modules[tname].ports_in["odometry"].msg_count > bc:
                received += 1
        test(22, "Odometry fan-out", received >= 2,
             "received=%d/%d, targets=%s" % (received, len(before), odom_targets))
    else:
        test(22, "Odometry fan-out", False, "No odometry source")
except Exception as e:
    test(22, "Odometry fan-out", False, str(e))

# ==== Test 23: on_system_modules VectorMemory ref ====
try:
    sp = system.get_module("SemanticPlannerModule")
    vm = system.get_module("VectorMemoryModule")
    has_ref = sp._vector_memory is not None
    is_vm = sp._vector_memory is vm
    test(23, "on_system_modules: _vector_memory is VectorMemoryModule",
         has_ref and is_vm, "has_ref=%s, is_vm=%s" % (has_ref, is_vm))
except Exception as e:
    test(23, "on_system_modules VectorMemory ref", False, str(e))

# ==== Test 24: Costmap chain ====
try:
    from runtime.msgs.sensor import PointCloud2
    from nav.services.map_layers.occupancy_grid_module import OccupancyGridModule
    ogm = OccupancyGridModule(resolution=0.5, map_radius=5.0, z_min=0.1, z_max=2.0)
    ogm.setup()
    pts = np.array([[1.0, 1.0, 0.5], [1.5, 1.5, 0.8], [2.0, 2.0, 1.0], [-1.0, -1.0, 0.3]], dtype=np.float32)
    cloud = PointCloud2(points=pts)
    before_cost = ogm.costmap.msg_count
    ogm._on_cloud(cloud)
    after_cost = ogm.costmap.msg_count
    test(24, "Costmap chain: _on_cloud -> costmap published",
         after_cost > before_cost, "before=%d, after=%d" % (before_cost, after_cost))
except Exception as e:
    test(24, "Costmap chain", False, str(e))

# ==== Test 25: Safety stop wiring ====
try:
    stop_conns = [c for c in system.connections
                  if c[0] == "nav.safety" and c[1] == "stop_cmd"]
    stop_targets = set(c[2] for c in stop_conns)
    has_driver = "StubDogModule" in stop_targets
    has_nav = "nav.mission" in stop_targets
    test(25, "Safety stop: wired to driver + Navigation",
         has_driver and has_nav, "targets=%s" % stop_targets)
except Exception as e:
    test(25, "Safety stop wiring", False, str(e))

# ==== Test 26: Safety stop loop remains bounded ====
try:
    from runtime.msgs.geometry import Pose, Quaternion, Twist, Vector3
    from runtime.msgs.nav import Odometry

    safety = system.get_module("nav.safety")
    nav = system.get_module("nav.mission")
    lp = system.get_module("nav.local_planner")
    pf = system.get_module("nav.path_follower")
    mux = system.get_module("nav.velocity_mux")

    # Prime SafetyRing into a healthy state first so the LOST update below is
    # the only STOP edge under test. The real full-stack wiring feeds
    # nav.velocity_mux.driver_cmd_vel back into nav.services.safety.cmd_vel for auditing.
    safety._on_odom(Odometry(
        pose=Pose(position=Vector3(0.0, 0.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        twist=Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0, 0, 0)),
    ))
    safety._on_cmdvel(Twist.zero())
    safety._on_localization_status({"state": "OK", "confidence": 1.0})

    watched_ports = [
        safety.stop_cmd,
        nav.clear_path,
        nav.recovery_cmd_vel,
        lp.local_path,
        pf.cmd_vel,
        mux.driver_cmd_vel,
    ]
    before_errors = sum(getattr(p, "_publish_errors", 0) for p in watched_ports)
    before_stop_count = safety.stop_cmd.msg_count

    safety._on_localization_status({"state": "LOST", "confidence": 0.0})
    time.sleep(0.05)

    after_errors = sum(getattr(p, "_publish_errors", 0) for p in watched_ports)
    stop_delta = safety.stop_cmd.msg_count - before_stop_count
    test(26, "Safety stop loop remains bounded",
         after_errors == before_errors and 1 <= stop_delta <= 10,
         "errors %d->%d, stop_delta=%d" % (before_errors, after_errors, stop_delta))
except Exception as e:
    test(26, "Safety stop loop remains bounded", False, str(e))

# ==== Test 27: Teleop priority ====
try:
    teleop = system.get_module("TeleopModule")
    teleop._on_joy({"lx": 0.5, "ly": 0.0, "az": 0.1})
    active_after = teleop._active

    # timestamps. Mock an idle timestamp using the same clock so the
    # release branch in _check_idle actually fires.
    teleop._last_joy_time = time.monotonic() - teleop._release_timeout - 1.0
    teleop._check_idle()
    released = not teleop._active
    test(27, "Teleop priority: _on_joy active, check_idle releases",
         active_after and released,
         "active_after=%s, released=%s" % (active_after, released))
except Exception as e:
    test(27, "Teleop priority", False, str(e))

# ==== Test 28: Instruction fan-in ====
try:
    instr_conns = [c for c in system.connections
                   if c[3] == "instruction" and c[2] == "SemanticPlannerModule"]
    instr_sources = set(c[0] for c in instr_conns)
    has_gw = "GatewayModule" in instr_sources
    has_mcp = "MCPServerModule" in instr_sources
    test(28, "Instruction fan-in: Gateway + MCP -> SemanticPlanner",
         has_gw and has_mcp, "sources=%s" % instr_sources)
except Exception as e:
    test(28, "Instruction fan-in", False, str(e))

# ==== Test 29: goal_pose SemanticPlanner -> Navigation ====
try:
    from runtime.msgs.geometry import PoseStamped
    goal_conns = [c for c in system.connections
                  if c[0] == "SemanticPlannerModule" and c[1] == "goal_pose"
                     and c[2] == "nav.mission" and c[3] == "goal_pose"]
    nav = system.get_module("nav.mission")
    sp2 = system.get_module("SemanticPlannerModule")
    before_gp = nav.ports_in["goal_pose"].msg_count
    goal = PoseStamped(
        pose=Pose(position=Vector3(5.0, 3.0, 0.0), orientation=Quaternion(0, 0, 0, 1)),
        frame_id="map",
    )
    sp2.goal_pose.publish(goal)
    time.sleep(0.05)
    after_gp = nav.ports_in["goal_pose"].msg_count
    test(29, "goal_pose: SemanticPlanner -> Navigation",
         after_gp > before_gp and len(goal_conns) > 0,
         "wire=%d, before=%d, after=%d" % (len(goal_conns), before_gp, after_gp))
except Exception as e:
    test(29, "goal_pose fan-in", False, str(e))

# ==== Test 30: @skill MCP discovery ====
try:
    mcp = system.get_module("MCPServerModule")
    # MCPServerModule was refactored to a single _tool_list (no longer split
    # static TOOLS + _dynamic_tools). The list is populated in
    # on_system_modules() by walking @skill methods of all modules.
    tool_count = len(getattr(mcp, "_tool_list", []))
    test(30, "@skill MCP discovery: tool count > 0",
         tool_count > 0, "tool_list len=%d" % tool_count)
except Exception as e:
    test(30, "@skill MCP discovery", False, str(e))

# ==== Test 31: Zero auto_wire ambiguity ====
try:
    ambiguity_warnings = [w for w in _build_warnings if "ambiguity" in w.lower()]
    test(31, "Zero auto_wire ambiguity warnings",
         len(ambiguity_warnings) == 0,
         "found %d: %s" % (len(ambiguity_warnings), ambiguity_warnings[:3]))
except Exception as e:
    test(31, "Zero auto_wire ambiguity", False, str(e))

# ==== Summary ====
_passed = sum(1 for v in results.values() if v)
_failed = sum(1 for v in results.values() if not v)
_total = len(results)
print(flush=True)
print("=" * 60, flush=True)
print("Summary: %d/%d PASSED, %d/%d FAILED" % (_passed, _total, _failed, _total), flush=True)
print("=" * 60, flush=True)


def test_cross_module_integration_all_pass() -> None:
    """Pytest entry point: assert that every numbered integration check passed."""
    failed_tests = {k: v for k, v in results.items() if not v}
    assert not failed_tests, (
        "%d/%d integration tests FAILED: %s" % (len(failed_tests), _total, sorted(failed_tests))
    )


if __name__ == "__main__":
    import os as _os

    # Stop system (may block on server threads, so use os._exit as fallback)
    def _final_stop():
        time.sleep(5)
        _os._exit(0 if _failed == 0 else 1)

    t = threading.Thread(target=_final_stop, daemon=True)
    t.start()
    try:
        system.stop()
    except Exception:
        pass
    sys.exit(0 if _failed == 0 else 1)
