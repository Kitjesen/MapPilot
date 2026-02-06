#!/usr/bin/env python3
"""
Integration test suite for remote_monitoring <-> navigation stack.

Usage:
    # Smoke test (gateway only, no navigation stack needed)
    python3 test_integration.py --gateway-only

    # Full integration test (requires navigation stack running)
    python3 test_integration.py --full

    # Custom host/port
    python3 test_integration.py --gateway-only --host 192.168.66.190 --port 50051

Prerequisites:
    - grpcurl installed (v1.8.x+)
    - remote_monitoring gateway node running
    - For --full: navigation stack running (Fast-LIO2, localizer, etc.)
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
import uuid


# ---------------------------------------------------------------------------
# Colour helpers
# ---------------------------------------------------------------------------

class Colors:
    GREEN = "\033[92m"
    RED = "\033[91m"
    YELLOW = "\033[93m"
    CYAN = "\033[96m"
    BOLD = "\033[1m"
    RESET = "\033[0m"


def _c(color, text):
    return f"{color}{text}{Colors.RESET}"


# ---------------------------------------------------------------------------
# Subprocess helpers
# ---------------------------------------------------------------------------

def grpcurl(host, port, method, data=None, timeout=5):
    """Call grpcurl and return (exit_code, stdout, stderr)."""
    cmd = ["grpcurl", "-plaintext"]
    if data is not None:
        cmd += ["-d", json.dumps(data) if isinstance(data, dict) else data]
    cmd += [f"{host}:{port}", method]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "TIMEOUT"


def grpcurl_stream(host, port, method, data=None, timeout=4):
    """Call a streaming grpcurl, capture output until timeout, return lines."""
    cmd = ["grpcurl", "-plaintext"]
    if data is not None:
        cmd += ["-d", json.dumps(data) if isinstance(data, dict) else data]
    cmd += [f"{host}:{port}", method]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired as e:
        # For streaming RPCs, timeout is expected (we got data)
        stdout = e.stdout.decode("utf-8", errors="replace") if e.stdout else ""
        stderr = e.stderr.decode("utf-8", errors="replace") if e.stderr else ""
        return 0, stdout, stderr


def ros2_topic_echo_once(topic, timeout=5):
    """Echo a single message from a ROS2 topic. Returns (success, output)."""
    cmd = ["ros2", "topic", "echo", topic, "--once"]
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return result.returncode == 0, result.stdout
    except subprocess.TimeoutExpired:
        return False, "TIMEOUT"


def ros2_topic_echo_background(topic, output_list, stop_event, timeout=10):
    """Echo topic in background thread, appending lines to output_list."""
    cmd = ["ros2", "topic", "echo", topic]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        start = time.time()
        while not stop_event.is_set() and (time.time() - start) < timeout:
            line = proc.stdout.readline()
            if line:
                output_list.append(line.strip())
            else:
                time.sleep(0.05)
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
    except Exception as e:
        output_list.append(f"ERROR: {e}")


def _rid():
    """Generate a unique request_id."""
    return str(uuid.uuid4())[:8]


# ---------------------------------------------------------------------------
# Test framework
# ---------------------------------------------------------------------------

class TestRunner:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.results = []  # list of (name, passed, detail)

    def record(self, name, passed, detail=""):
        self.results.append((name, passed, detail))
        status = _c(Colors.GREEN, "PASS") if passed else _c(Colors.RED, "FAIL")
        detail_str = f"  ({detail})" if detail else ""
        print(f"  [{status}] {name}{detail_str}")

    def summary(self):
        total = len(self.results)
        passed = sum(1 for _, p, _ in self.results if p)
        failed = total - passed

        print()
        print(_c(Colors.BOLD, "=" * 60))
        if failed == 0:
            print(_c(Colors.GREEN + Colors.BOLD,
                      f"  ALL TESTS PASSED: {passed}/{total}"))
        else:
            print(_c(Colors.RED + Colors.BOLD,
                      f"  FAILED: {failed}/{total} tests failed"))
            for name, p, detail in self.results:
                if not p:
                    print(f"    - {name}: {detail}")
        print(_c(Colors.BOLD, "=" * 60))
        return 0 if failed == 0 else 1

    # --- Helper to call grpcurl with common pattern ---
    def call(self, method, data=None, timeout=5):
        return grpcurl(self.host, self.port, method, data, timeout)

    def call_stream(self, method, data=None, timeout=4):
        return grpcurl_stream(self.host, self.port, method, data, timeout)


# ---------------------------------------------------------------------------
# Gateway-only tests (smoke tests)
# ---------------------------------------------------------------------------

def test_service_list(t: TestRunner):
    """Test 1: All 4 services registered."""
    rc, out, err = t.call("list")
    expected = [
        "robot.v1.ControlService",
        "robot.v1.DataService",
        "robot.v1.SystemService",
        "robot.v1.TelemetryService",
    ]
    all_found = all(svc in out for svc in expected)
    t.record("service_list", rc == 0 and all_found,
             f"found {sum(1 for s in expected if s in out)}/4 services")


def test_get_robot_info(t: TestRunner):
    """Test 2: GetRobotInfo returns configured robot_id."""
    rc, out, err = t.call("robot.v1.SystemService/GetRobotInfo")
    parsed = _parse_json(out)
    robot_id = parsed.get("robot_id", parsed.get("robotId", ""))
    has_id = robot_id != ""
    t.record("get_robot_info", rc == 0 and has_id,
             f"robot_id={robot_id or 'MISSING'}")


def test_get_capabilities(t: TestRunner):
    """Test 3: GetCapabilities includes terrain resource."""
    rc, out, err = t.call("robot.v1.SystemService/GetCapabilities")
    parsed = _parse_json(out)
    resources = parsed.get("supported_resources",
                           parsed.get("supportedResources", []))
    has_terrain = "pointcloud/terrain" in resources
    t.record("get_capabilities", rc == 0 and has_terrain,
             f"resources={resources}")


def test_set_mode_teleop(t: TestRunner):
    """Test 4: SetMode(TELEOP=3) returns OK."""
    data = {"base": {"request_id": _rid()}, "mode": 3}
    rc, out, err = t.call("robot.v1.ControlService/SetMode", data)
    parsed = _parse_json(out)
    mode = parsed.get("current_mode", parsed.get("currentMode"))
    mode_ok = mode == "ROBOT_MODE_TELEOP"
    base = parsed.get("base", {})
    err_ok = base.get("error_code", base.get("errorCode")) == "ERROR_CODE_OK"
    t.record("set_mode_teleop", rc == 0 and mode_ok and err_ok,
             f"mode={mode}")


def test_set_mode_autonomous(t: TestRunner):
    """Test 5: SetMode(AUTONOMOUS=4) returns OK."""
    data = {"base": {"request_id": _rid()}, "mode": 4}
    rc, out, err = t.call("robot.v1.ControlService/SetMode", data)
    parsed = _parse_json(out)
    mode = parsed.get("current_mode", parsed.get("currentMode"))
    mode_ok = mode == "ROBOT_MODE_AUTONOMOUS"
    t.record("set_mode_autonomous", rc == 0 and mode_ok,
             f"mode={mode}")


def test_emergency_stop(t: TestRunner):
    """Test 6: EmergencyStop returns stopped=true."""
    data = {"base": {"request_id": _rid()}}
    rc, out, err = t.call("robot.v1.ControlService/EmergencyStop", data)
    parsed = _parse_json(out)
    stopped = parsed.get("stopped", False)
    t.record("emergency_stop", rc == 0 and stopped,
             f"stopped={stopped}")


def test_start_nav_task(t: TestRunner):
    """Test 7: StartTask(NAVIGATION) returns task_id and status RUNNING."""
    # First switch to idle to reset
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 1})

    data = {
        "base": {"request_id": _rid()},
        "task_type": "TASK_TYPE_NAVIGATION",
        "params_json": '{"x":10.0,"y":5.0,"z":0.0}'
    }
    rc, out, err = t.call("robot.v1.ControlService/StartTask", data)
    parsed = _parse_json(out)
    task_id = parsed.get("task_id", parsed.get("taskId", ""))
    has_task_id = len(task_id) > 0
    task = parsed.get("task", {})
    is_running = task.get("status") == "TASK_STATUS_RUNNING"
    t.record("start_nav_task", rc == 0 and has_task_id and is_running,
             f"task_id={task_id or 'NONE'}, status={task.get('status')}")


def test_cancel_task(t: TestRunner):
    """Test 8: CancelTask returns status CANCELLED."""
    data = {"base": {"request_id": _rid()}, "task_id": ""}
    rc, out, err = t.call("robot.v1.ControlService/CancelTask", data)
    parsed = _parse_json(out)
    task = parsed.get("task", {})
    is_cancelled = task.get("status") == "TASK_STATUS_CANCELLED"
    t.record("cancel_task", rc == 0 and is_cancelled,
             f"status={task.get('status')}")


def test_list_resources(t: TestRunner):
    """Test 9: ListResources includes terrain resource."""
    rc, out, err = t.call("robot.v1.DataService/ListResources")
    parsed = _parse_json(out)
    resources = parsed.get("resources", [])
    names = [r.get("id", r.get("resource_id", {})).get("name", "")
             for r in resources]
    has_terrain = "terrain" in names
    t.record("list_resources", rc == 0 and has_terrain,
             f"names={names}")


def test_relocalize_unavailable(t: TestRunner):
    """Test 10: Relocalize returns SERVICE_UNAVAILABLE when localizer not running."""
    data = {
        "base": {"request_id": _rid()},
        "pcd_path": "/tmp/nonexistent.pcd",
        "x": 0, "y": 0, "z": 0,
    }
    rc, out, err = t.call("robot.v1.SystemService/Relocalize", data, timeout=15)
    parsed = _parse_json(out)
    base = parsed.get("base", {})
    err_code = base.get("error_code", base.get("errorCode", ""))
    # Either SERVICE_UNAVAILABLE or TIMEOUT is acceptable (service not up)
    acceptable = err_code in ("ERROR_CODE_SERVICE_UNAVAILABLE", "ERROR_CODE_TIMEOUT")
    t.record("relocalize_unavailable", rc == 0 and acceptable,
             f"error_code={err_code}")


def test_save_map_unavailable(t: TestRunner):
    """Test 11: SaveMap returns SERVICE_UNAVAILABLE when fastlio not running."""
    data = {
        "base": {"request_id": _rid()},
        "file_path": "/tmp/test_map.pcd",
    }
    rc, out, err = t.call("robot.v1.SystemService/SaveMap", data, timeout=15)
    parsed = _parse_json(out)
    base = parsed.get("base", {})
    err_code = base.get("error_code", base.get("errorCode", ""))
    acceptable = err_code in ("ERROR_CODE_SERVICE_UNAVAILABLE", "ERROR_CODE_TIMEOUT")
    t.record("save_map_unavailable", rc == 0 and acceptable,
             f"error_code={err_code}")


def test_slow_state_nav_fields(t: TestRunner):
    """Test 12: StreamSlowState contains navigation block."""
    rc, out, err = t.call_stream(
        "robot.v1.TelemetryService/StreamSlowState", timeout=4)
    has_navigation = ('"navigation"' in out or '"globalPlannerStatus"' in out
                      or '"global_planner_status"' in out)
    # grpcurl may use snake_case or camelCase depending on version
    has_topic_rates = ('"topic_rates"' in out or '"topicRates"' in out)
    t.record("slow_state_nav_fields", has_navigation and has_topic_rates,
             f"has_navigation={has_navigation}, has_topic_rates={has_topic_rates}")


def test_stop_topic_on_estop(t: TestRunner):
    """Test 13: /stop topic gets published when EmergencyStop is called."""
    # Background listener for /stop
    output = []
    stop_event = threading.Event()
    listener = threading.Thread(
        target=ros2_topic_echo_background,
        args=("/stop", output, stop_event, 8),
        daemon=True,
    )
    listener.start()
    time.sleep(1)  # let subscriber establish

    # Reset mode first, then trigger e-stop
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 1})
    time.sleep(0.5)
    t.call("robot.v1.ControlService/EmergencyStop",
           {"base": {"request_id": _rid()}})
    time.sleep(2)
    stop_event.set()
    listener.join(timeout=3)

    # Check if /stop topic received data=1
    got_stop = any("data: 1" in line for line in output)
    t.record("stop_topic_on_estop", got_stop,
             f"captured {len(output)} lines, got_stop={got_stop}")


def test_goal_pose_on_task(t: TestRunner):
    """Test 14: /goal_pose topic gets published when StartTask NAVIGATION is called."""
    # Background listener for /goal_pose
    output = []
    stop_event = threading.Event()
    listener = threading.Thread(
        target=ros2_topic_echo_background,
        args=("/goal_pose", output, stop_event, 8),
        daemon=True,
    )
    listener.start()
    time.sleep(1)

    # Reset mode, then start nav task
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 1})
    time.sleep(0.5)
    data = {
        "base": {"request_id": _rid()},
        "task_type": "TASK_TYPE_NAVIGATION",
        "params_json": '{"x":10.0,"y":5.0,"z":0.0}'
    }
    t.call("robot.v1.ControlService/StartTask", data)
    time.sleep(2)
    stop_event.set()
    listener.join(timeout=3)

    got_goal = any("position" in line or "x:" in line for line in output)
    t.record("goal_pose_on_task", got_goal,
             f"captured {len(output)} lines, got_goal={got_goal}")

    # Cleanup: cancel task and go idle
    t.call("robot.v1.ControlService/CancelTask",
           {"base": {"request_id": _rid()}, "task_id": ""})


# ---------------------------------------------------------------------------
# Full integration tests (require navigation stack)
# ---------------------------------------------------------------------------

def test_fast_state_live(t: TestRunner):
    """Test 15: FastState has non-zero pose from real odometry."""
    rc, out, err = t.call_stream(
        "robot.v1.TelemetryService/StreamFastState", timeout=4)
    parsed_list = _parse_json_stream(out)
    if not parsed_list:
        t.record("fast_state_live", False, "no FastState received")
        return

    state = parsed_list[0]
    pose = state.get("pose", {}).get("position", {})
    orientation = state.get("pose", {}).get("orientation", {})
    tf_ok = state.get("tf_ok", state.get("tfOk", False))
    # Check if pose data is present (orientation.w should be non-zero for valid quaternion)
    has_pose = orientation.get("w", 0) != 0
    t.record("fast_state_live", has_pose,
             f"pos=({pose.get('x',0):.2f},{pose.get('y',0):.2f},{pose.get('z',0):.2f}), "
             f"tf_ok={tf_ok}")


def test_topic_rates_live(t: TestRunner):
    """Test 16: SlowState topic_rates has non-zero hz values."""
    rc, out, err = t.call_stream(
        "robot.v1.TelemetryService/StreamSlowState", timeout=4)
    parsed_list = _parse_json_stream(out)
    if not parsed_list:
        t.record("topic_rates_live", False, "no SlowState received")
        return

    state = parsed_list[-1]  # take last for most up-to-date rates
    rates = state.get("topic_rates", state.get("topicRates", {}))
    odom_hz = rates.get("odom_hz", rates.get("odomHz", 0))
    lidar_hz = rates.get("lidar_hz", rates.get("lidarHz", 0))
    has_data = odom_hz > 0 or lidar_hz > 0
    terrain_hz = rates.get("terrain_map_hz", rates.get("terrainMapHz", 0))
    cmd_vel_hz = rates.get("cmd_vel_hz", rates.get("cmdVelHz", 0))
    t.record("topic_rates_live", has_data,
             f"odom={odom_hz:.1f}Hz, lidar={lidar_hz:.1f}Hz, "
             f"terrain={terrain_hz:.1f}Hz, "
             f"cmd_vel={cmd_vel_hz:.1f}Hz")


def test_localization_valid(t: TestRunner):
    """Test 17: navigation.localization_valid is true."""
    rc, out, err = t.call_stream(
        "robot.v1.TelemetryService/StreamSlowState", timeout=6)
    parsed_list = _parse_json_stream(out)
    if not parsed_list:
        t.record("localization_valid", False, "no SlowState received")
        return

    state = parsed_list[-1]
    nav = state.get("navigation", {})
    loc_valid = nav.get("localization_valid", nav.get("localizationValid", False))
    planner_status = nav.get("global_planner_status",
                             nav.get("globalPlannerStatus", "N/A"))
    t.record("localization_valid", loc_valid,
             f"localization_valid={loc_valid}, "
             f"planner_status={planner_status}")


def test_relocalize_success(t: TestRunner):
    """Test 18: Relocalize returns success=true with a real PCD map."""
    # Try to find an existing PCD file
    pcd_path = _find_pcd_file()
    if not pcd_path:
        t.record("relocalize_success", False, "no PCD map file found, skipping")
        return

    data = {
        "base": {"request_id": _rid()},
        "pcd_path": pcd_path,
        "x": 0, "y": 0, "z": 0,
        "yaw": 0, "pitch": 0, "roll": 0,
    }
    rc, out, err = t.call("robot.v1.SystemService/Relocalize", data, timeout=35)
    parsed = _parse_json(out)
    success = parsed.get("success", False)
    msg = parsed.get("message", "")
    t.record("relocalize_success", rc == 0 and success,
             f"success={success}, message={msg}")


def test_mode_switch_e2e(t: TestRunner):
    """Test 19: Mode switch publishes /stop correctly (TELEOP->AUTONOMOUS)."""
    output = []
    stop_event = threading.Event()
    listener = threading.Thread(
        target=ros2_topic_echo_background,
        args=("/stop", output, stop_event, 10),
        daemon=True,
    )
    listener.start()
    time.sleep(1)

    # Switch to TELEOP (should publish /stop=1)
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 3})
    time.sleep(1.5)

    # Switch to AUTONOMOUS (should publish /stop=0)
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 4})
    time.sleep(1.5)

    stop_event.set()
    listener.join(timeout=3)

    got_stop_1 = any("data: 1" in line for line in output)
    got_stop_0 = any("data: 0" in line for line in output)
    t.record("mode_switch_e2e", got_stop_1 and got_stop_0,
             f"got /stop=1: {got_stop_1}, got /stop=0: {got_stop_0}")

    # Cleanup
    t.call("robot.v1.ControlService/SetMode",
           {"base": {"request_id": _rid()}, "mode": 1})


# ---------------------------------------------------------------------------
# JSON parsing helpers
# ---------------------------------------------------------------------------

def _parse_json(text):
    """Parse a single JSON object from grpcurl output."""
    text = text.strip()
    if not text:
        return {}
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        # grpcurl sometimes outputs multiple JSON objects; take the first
        brace = 0
        start = text.find("{")
        if start < 0:
            return {}
        for i in range(start, len(text)):
            if text[i] == "{":
                brace += 1
            elif text[i] == "}":
                brace -= 1
                if brace == 0:
                    try:
                        return json.loads(text[start:i + 1])
                    except json.JSONDecodeError:
                        return {}
        return {}


def _parse_json_stream(text):
    """Parse multiple JSON objects from streaming grpcurl output."""
    results = []
    text = text.strip()
    if not text:
        return results

    brace = 0
    start = -1
    for i, ch in enumerate(text):
        if ch == "{":
            if brace == 0:
                start = i
            brace += 1
        elif ch == "}":
            brace -= 1
            if brace == 0 and start >= 0:
                try:
                    obj = json.loads(text[start:i + 1])
                    results.append(obj)
                except json.JSONDecodeError:
                    pass
                start = -1
    return results


def _find_pcd_file():
    """Try to find an existing PCD file for relocalize test."""
    candidates = [
        "/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd",
        "/home/sunrise/data/maps",
        "/tmp",
    ]
    for directory in candidates:
        if os.path.isdir(directory):
            for f in os.listdir(directory):
                if f.endswith(".pcd"):
                    return os.path.join(directory, f)
    return None


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Integration tests for remote_monitoring gRPC gateway")
    parser.add_argument("--gateway-only", action="store_true",
                        help="Run only smoke tests (no navigation stack needed)")
    parser.add_argument("--full", action="store_true",
                        help="Run all tests including integration with nav stack")
    parser.add_argument("--host", default="localhost",
                        help="gRPC server host (default: localhost)")
    parser.add_argument("--port", default="50051",
                        help="gRPC server port (default: 50051)")
    args = parser.parse_args()

    if not args.gateway_only and not args.full:
        print("Please specify --gateway-only or --full")
        parser.print_help()
        sys.exit(1)

    t = TestRunner(args.host, args.port)

    # --- Pre-flight check ---
    print(_c(Colors.BOLD, "\n=== Pre-flight Check ===\n"))
    rc, out, err = t.call("list", timeout=3)
    if rc != 0:
        print(_c(Colors.RED, f"  Cannot connect to gRPC server at {args.host}:{args.port}"))
        print(f"  Error: {err.strip()}")
        print(f"\n  Make sure the gateway is running:")
        print(f"    ros2 launch remote_monitoring grpc_gateway.launch.py")
        sys.exit(1)
    print(_c(Colors.GREEN, f"  Connected to gRPC server at {args.host}:{args.port}"))

    # --- Smoke tests (gateway-only) ---
    print(_c(Colors.BOLD, "\n=== Smoke Tests (Gateway Only) ===\n"))
    test_service_list(t)
    test_get_robot_info(t)
    test_get_capabilities(t)
    test_set_mode_teleop(t)
    test_set_mode_autonomous(t)
    test_emergency_stop(t)
    test_start_nav_task(t)
    test_cancel_task(t)
    test_list_resources(t)
    test_relocalize_unavailable(t)
    test_save_map_unavailable(t)
    test_slow_state_nav_fields(t)
    test_stop_topic_on_estop(t)
    test_goal_pose_on_task(t)

    # --- Full integration tests ---
    if args.full:
        print(_c(Colors.BOLD, "\n=== Integration Tests (Navigation Stack) ===\n"))
        test_fast_state_live(t)
        test_topic_rates_live(t)
        test_localization_valid(t)
        test_relocalize_success(t)
        test_mode_switch_e2e(t)

    # --- Summary ---
    sys.exit(t.summary())


if __name__ == "__main__":
    main()
