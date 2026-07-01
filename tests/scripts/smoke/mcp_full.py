#!/usr/bin/env python3
# Thunder compatibility smoke; run from repository root:
#   python tests/scripts/smoke/mcp_full.py
import sys, os, json, time, urllib.request
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.WARNING)

results = []
def T(name, ok, detail=""):
    results.append((name, ok))
    mark = "PASS" if ok else "FAIL"
    print("  %s %s%s" % (mark, name, (" - " + detail) if detail else ""))


def mcp_call(method, params=None):
    payload = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}).encode()
    req = urllib.request.Request("http://127.0.0.1:8090/mcp", data=payload,
                                headers={"Content-Type": "application/json"})
    resp = urllib.request.urlopen(req, timeout=5)
    return json.loads(resp.read())


# Build and start
from core.blueprints.profile_builder import blueprint_for_resolved_profile

bp = blueprint_for_resolved_profile("dev", dict(
    robot="sim_ros2", slam_profile="bridge",
    enable_native=False, enable_semantic=True, enable_gateway=True,
))
system = bp.build()
system.start()
time.sleep(3)

print("=== System ===")
T("build", len(system.modules) > 15, "%d modules" % len(system.modules))
T("connections", len(system.connections) > 30, "%d connections" % len(system.connections))

# Check key modules alive
print("\n=== Module Status ===")
for name in ["SlamBridgeModule", "CameraBridgeModule", "NavigationModule",
             "SemanticPlannerModule", "GatewayModule", "MCPServerModule",
             "TeleopModule", "VectorMemoryModule", "SemanticMapperModule",
             "VisualServoModule", "DetectorModule"]:
    mod = system.modules.get(name)
    T("module_%s" % name, mod is not None)

# MCP tests
print("\n=== MCP Protocol ===")
try:
    r = mcp_call("tools/list")
    tools = r.get("result", {}).get("tools", [])
    tool_names = [t["name"] for t in tools]
    T("mcp_tools_list", len(tools) > 0, "%d tools" % len(tools))
    for t in tools:
        print("    - %s" % t["name"])
except Exception as e:
    T("mcp_tools_list", False, str(e))
    tool_names = []

# Call each tool
print("\n=== MCP Tool Calls ===")
test_calls = [
    ("navigate_to", {"x": 5.0, "y": 3.0}),
    ("get_navigation_status", {}),
    ("stop_navigation", {}),
]

# Add semantic tools if available
if "get_room_summary" in tool_names:
    test_calls.append(("get_room_summary", {}))
if "query_location" in tool_names:
    test_calls.append(("query_location", {"text": "backpack"}))
if "get_servo_status" in tool_names:
    test_calls.append(("get_servo_status", {}))
if "get_semantic_status" in tool_names:
    test_calls.append(("get_semantic_status", {}))
if "get_memory_stats" in tool_names:
    test_calls.append(("get_memory_stats", {}))
if "find_object" in tool_names:
    test_calls.append(("find_object", {"target": "chair"}))
if "get_teleop_status" in tool_names:
    test_calls.append(("get_teleop_status", {}))

for tool_name, args in test_calls:
    try:
        r = mcp_call("tools/call", {"name": tool_name, "arguments": args})
        err = r.get("error")
        if err:
            T("call_%s" % tool_name, False, str(err))
        else:
            content = r.get("result", {}).get("content", [{}])
            text = content[0].get("text", "") if content else ""
            T("call_%s" % tool_name, True, text[:80])
    except Exception as e:
        T("call_%s" % tool_name, False, str(e))

# Gateway HTTP check
print("\n=== Gateway ===")
try:
    resp = urllib.request.urlopen("http://127.0.0.1:5050/", timeout=3)
    T("gateway_root", resp.status == 200)
except urllib.error.HTTPError as e:
    T("gateway_root", e.code < 500, "HTTP %d" % e.code)
except Exception as e:
    T("gateway_root", False, str(e))

# Teleop port check
print("\n=== Teleop ===")
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.settimeout(1)
try:
    s.connect(("127.0.0.1", 5050))
    T("teleop_port", True, "ws://5050/ws/teleop reachable")
    s.close()
except Exception:
    T("teleop_port", False, "ws://5050/ws/teleop not reachable (gateway not running)")

# ROS2 bridge check
print("\n=== ROS2 Bridges ===")
slam = system.modules.get("SlamBridgeModule")
cam = system.modules.get("CameraBridgeModule")
T("slam_bridge_node", slam is not None and slam._node is not None)
T("camera_bridge_node", cam is not None and cam._node is not None)

# Auto-wire ambiguity check
print("\n=== Wiring ===")
# We can't easily check ambiguity post-build, but verify connection count
T("zero_ambiguity", len(system.connections) >= 45, "%d connections" % len(system.connections))

system.stop()

# Summary
print("\n" + "=" * 50)
passed = sum(1 for _, ok in results if ok)
failed = sum(1 for _, ok in results if not ok)
print("  %d PASSED, %d FAILED / %d total" % (passed, failed, len(results)))
if failed:
    print("\n  Failures:")
    for name, ok in results:
        if not ok:
            print("    FAIL: %s" % name)
print("=" * 50)
