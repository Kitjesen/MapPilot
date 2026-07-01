#!/usr/bin/env python3
# Thunder compatibility smoke; run from repository root:
#   python tests/scripts/smoke/s100p_start.py
import sys, os, time
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.WARNING)

from core.blueprints.profile_builder import blueprint_for_resolved_profile

bp = blueprint_for_resolved_profile("dev", dict(
    robot="sim_ros2", slam_profile="bridge",
    enable_native=False, enable_semantic=True, enable_gateway=True,
))
system = bp.build()
system.start()
time.sleep(3)

print("=== THUNDER COMPAT SYSTEM STATUS ===")
n = len(system.modules)
c = len(system.connections)
print("Modules: %d" % n)
print("Connections: %d" % c)

for name in sorted(system.modules):
    mod = system.modules[name]
    layer = mod.layer if mod.layer is not None else "?"
    alive = ""
    if hasattr(mod, "alive"):
        alive = " alive=%d" % mod.alive.msg_count
    print("  [L%s] %s%s" % (layer, name, alive))

slam = system.modules.get("SlamBridgeModule")
cam = system.modules.get("CameraBridgeModule")
print("SlamBridge ROS2 node: %s" % (slam._node is not None if slam else False))
print("CameraBridge ROS2 node: %s" % (cam._node is not None if cam else False))

gw = system.modules.get("GatewayModule")
mcp = system.modules.get("MCPServerModule")
tp = system.modules.get("TeleopModule")
print("Gateway: port=%s" % (gw._port if gw else "N/A"))
print("MCP: tools=%d" % len(getattr(mcp, "_dynamic_tools", [])) if mcp else "N/A")
print("Teleop: port=%s" % (tp._port if tp else "N/A"))

system.stop()
print("STOP OK")
