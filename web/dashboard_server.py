"""
LingTu Interactive Dashboard Server
====================================
Web UI for managing the LingTu navigation system on the robot.

Two connection methods:
  1. SSH (paramiko) to robot - systemd services, ROS2, system info, logs
  2. gRPC direct to robot:50051 - proto services (System, Control, Telemetry, Data)

Usage:
    python web/dashboard_server.py
    # Open http://localhost:8066
"""

import asyncio
import json
import os
import sys
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

import paramiko
import uvicorn
import yaml
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse
from pydantic import BaseModel

# ── gRPC Proto Imports (graceful fallback) ───────────────────
# The generated stubs use bare `import common_pb2` etc., so we
# must put proto_gen/ on sys.path before importing them.
_PROTO_DIR = str(Path(__file__).parent / "proto_gen")
if _PROTO_DIR not in sys.path:
    sys.path.insert(0, _PROTO_DIR)

GRPC_AVAILABLE = False
try:
    import control_pb2
    import control_pb2_grpc
    import data_pb2
    import data_pb2_grpc
    import grpc
    import system_pb2
    import system_pb2_grpc
    import telemetry_pb2
    import telemetry_pb2_grpc
    from google.protobuf import empty_pb2
    from google.protobuf.json_format import MessageToDict

    GRPC_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] gRPC stubs not available, gRPC features disabled: {e}")


# ── Robot Config ──────────────────────────────────────────────
ROBOT_HOST = "192.168.66.190"
ROBOT_USER = "sunrise"
ROBOT_PASS = os.environ.get("S100P_PASSWORD", "")
GRPC_PORT = 50051
GRPC_TIMEOUT = 5  # seconds

NAV_WS = "/opt/nova/lingtu/current"
MAP_DIR = "/home/sunrise/data/nova/maps"
DATA_DIR = "/home/sunrise/data/nova"
PATROL_DIR = "/home/sunrise/data/nova/patrol_routes"
ROS_SETUP = f"source /opt/ros/humble/setup.bash && source {NAV_WS}/install/setup.bash"

# All 16 managed services
SERVICES = [
    # Navigation
    "nav-lidar",
    "nav-slam",
    "nav-planning",
    "nav-autonomy",
    "nav-grpc",
    "nav-semantic",
    # Brainstem
    "brainstem",
    # Cortex Runtime (6 microservices)
    "cortex-arbiter",
    "cortex-telemetry",
    "cortex-safety",
    "cortex-control",
    "cortex-nav-gw",
    "cortex-askme-edge",
    # Askme
    "askme",
    # OTA
    "ota-agent",
]

# Stack start/stop orders
STACK_NAV_ORDER = ["nav-lidar", "nav-slam", "nav-planning", "nav-autonomy", "nav-grpc"]
STACK_CORTEX_ORDER = [
    "cortex-telemetry",
    "cortex-safety",
    "cortex-control",
    "cortex-nav-gw",
    "cortex-arbiter",
    "cortex-askme-edge",
]
STACK_ALL_ORDER = STACK_NAV_ORDER + ["nav-semantic", "brainstem"] + STACK_CORTEX_ORDER + ["askme", "ota-agent"]

TOPICS_MONITOR = [
    "/nav/odometry",
    "/nav/map_cloud",
    "/nav/registered_cloud",
    "/nav/terrain_map",
    "/nav/global_path",
    "/nav/local_path",
    "/nav/cmd_vel",
    "/nav/stop",
    "/nav/slow_down",
    "/nav/planner_status",
    "/nav/adapter_status",
    "/nav/goal_pose",
    "/nav/lidar_scan",
    "/nav/imu",
    "/driver/odometry",
    "/nav/localization_quality",
    "/nav/semantic/scene_graph",
    "/nav/semantic/detections_3d",
    "/nav/semantic/status",
]


# ═════════════════════════════════════════════════════════════
#  SSH Connection Pool
# ═════════════════════════════════════════════════════════════
_ssh_lock = threading.Lock()
_ssh_client: paramiko.SSHClient | None = None


def get_ssh() -> paramiko.SSHClient:
    global _ssh_client
    with _ssh_lock:
        need_reconnect = False
        if _ssh_client is None:
            need_reconnect = True
        else:
            transport = _ssh_client.get_transport()
            if transport is None or not transport.is_active():
                need_reconnect = True
            else:
                # Verify connection is truly alive with a quick keepalive
                try:
                    transport.send_ignore()
                except Exception:
                    need_reconnect = True
        if need_reconnect:
            if not ROBOT_PASS:
                raise RuntimeError("S100P_PASSWORD must be set for SSH dashboard features")
            # Close stale connection
            if _ssh_client is not None:
                try:
                    _ssh_client.close()
                except Exception:
                    pass
                _ssh_client = None
            _ssh_client = paramiko.SSHClient()
            _ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            _ssh_client.connect(ROBOT_HOST, username=ROBOT_USER, password=ROBOT_PASS, timeout=5)
            # Set keepalive to detect dead connections early
            _ssh_client.get_transport().set_keepalive(10)
        return _ssh_client


import re as _re

# ── 输入验证 (防命令注入) ──
_SAFE_TOPIC_RE = _re.compile(r"^/[a-zA-Z0-9_/]+$")
_SAFE_ROS_NAME_RE = _re.compile(r"^[a-zA-Z0-9_/.:-]+$")
_SAFE_MAP_NAME_RE = _re.compile(r"^[a-zA-Z0-9_.\-]+$")


def _validate_topic(topic: str) -> str:
    if not _SAFE_TOPIC_RE.match(topic):
        raise ValueError(f"Invalid topic name: {topic}")
    return topic


def _validate_ros_name(name: str) -> str:
    if not _SAFE_ROS_NAME_RE.match(name):
        raise ValueError(f"Invalid ROS name: {name}")
    return name


def _validate_map_name(name: str) -> str:
    if not _SAFE_MAP_NAME_RE.match(name) or ".." in name:
        raise ValueError(f"Invalid map name: {name}")
    return name


def ssh_exec(cmd: str, timeout: int = 15) -> tuple[str, str, int]:
    """Execute command on robot via SSH, return (stdout, stderr, exit_code)."""
    try:
        client = get_ssh()
        stdin, stdout, stderr = client.exec_command(cmd, timeout=timeout)
        # Set channel timeout to prevent indefinite blocking
        stdout.channel.settimeout(timeout)
        stderr.channel.settimeout(timeout)
        out = stdout.read().decode("utf-8", errors="replace")
        err = stderr.read().decode("utf-8", errors="replace")
        code = stdout.channel.recv_exit_status()
        return out, err, code
    except Exception as e:
        # Force reconnect on next call
        global _ssh_client
        with _ssh_lock:
            try:
                if _ssh_client:
                    _ssh_client.close()
            except Exception:
                pass
            _ssh_client = None
        return "", str(e), -1


def ssh_exec_ros(cmd: str, timeout: int = 15) -> tuple[str, str, int]:
    """Execute ROS2 command (with setup sourced)."""
    return ssh_exec(f"bash -c '{ROS_SETUP} && {cmd}'", timeout=timeout)


# ═════════════════════════════════════════════════════════════
#  gRPC Connection Manager
# ═════════════════════════════════════════════════════════════
_grpc_lock = threading.Lock()
_grpc_channel: Optional["grpc.Channel"] = None
_grpc_stubs: dict = {}


def _get_grpc_channel():
    """Lazily create and cache a gRPC channel. Reconnects on failure."""
    global _grpc_channel
    if not GRPC_AVAILABLE:
        return None
    with _grpc_lock:
        if _grpc_channel is not None:
            try:
                state = _grpc_channel._channel.check_connectivity_state(True)
                # grpc connectivity states: 0=IDLE, 1=CONNECTING, 2=READY, 3=TRANSIENT_FAILURE, 4=SHUTDOWN
                if state in (3, 4):  # TRANSIENT_FAILURE or SHUTDOWN
                    _grpc_channel.close()
                    _grpc_channel = None
            except Exception:
                # Channel object is broken, recreate
                try:
                    _grpc_channel.close()
                except Exception:
                    pass
                _grpc_channel = None
        if _grpc_channel is None:
            target = f"{ROBOT_HOST}:{GRPC_PORT}"
            _grpc_channel = grpc.insecure_channel(
                target,
                options=[
                    ("grpc.keepalive_time_ms", 10000),
                    ("grpc.keepalive_timeout_ms", 5000),
                    ("grpc.initial_reconnect_backoff_ms", 1000),
                    ("grpc.max_reconnect_backoff_ms", 5000),
                ],
            )
            _grpc_stubs.clear()
        return _grpc_channel


def _get_stub(service_name: str):
    """Get or create a gRPC stub for the given service."""
    if not GRPC_AVAILABLE:
        return None
    channel = _get_grpc_channel()
    if channel is None:
        return None
    if service_name not in _grpc_stubs:
        stub_map = {
            "system": system_pb2_grpc.SystemServiceStub,
            "control": control_pb2_grpc.ControlServiceStub,
            "telemetry": telemetry_pb2_grpc.TelemetryServiceStub,
            "data": data_pb2_grpc.DataServiceStub,
        }
        cls = stub_map.get(service_name)
        if cls is None:
            return None
        _grpc_stubs[service_name] = cls(channel)
    return _grpc_stubs[service_name]


def _grpc_call(service_name: str, method_name: str, request=None, timeout: float = GRPC_TIMEOUT):
    """
    Make a unary gRPC call. Returns dict on success, raises on failure.
    If request is None, uses google.protobuf.Empty.
    Retries once with a fresh channel on transient gRPC errors.
    """
    if request is None:
        request = empty_pb2.Empty()

    for attempt in range(2):
        stub = _get_stub(service_name)
        if stub is None:
            raise RuntimeError("gRPC not available")
        method = getattr(stub, method_name, None)
        if method is None:
            raise RuntimeError(f"Method {method_name} not found on {service_name}")
        try:
            response = method(request, timeout=timeout)
            return MessageToDict(response, preserving_proto_field_name=True, always_print_fields_with_no_presence=True)
        except grpc.RpcError as e:
            if (
                attempt == 0
                and GRPC_AVAILABLE
                and e.code()
                in (
                    grpc.StatusCode.UNAVAILABLE,
                    grpc.StatusCode.DEADLINE_EXCEEDED,
                )
            ):
                # Force channel recreation and retry once
                _close_grpc()
                continue
            raise
    # Should not reach here, but just in case
    raise RuntimeError("gRPC call failed after retry")


def _grpc_ok(result: dict) -> dict:
    return {"ok": True, **result}


def _grpc_err(e: Exception) -> dict:
    msg = str(e)
    if hasattr(e, "details"):
        msg = e.details()
    return {"ok": False, "error": msg}


def _close_grpc():
    global _grpc_channel
    with _grpc_lock:
        if _grpc_channel is not None:
            _grpc_channel.close()
            _grpc_channel = None
        _grpc_stubs.clear()


# ═════════════════════════════════════════════════════════════
#  Pydantic Request Models
# ═════════════════════════════════════════════════════════════
class ServiceAction(BaseModel):
    service: str
    action: str  # start | stop | restart


class NavGoal(BaseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


class ParamUpdate(BaseModel):
    node: str
    param: str
    value: str


class SemanticGoal(BaseModel):
    instruction: str


class TopicPub(BaseModel):
    topic: str
    msg_type: str
    data: str


class PatrolRoute(BaseModel):
    name: str
    waypoints: list  # [{x, y, z, name}, ...]
    loop: bool = False


class PatrolAction(BaseModel):
    name: str


class POIItem(BaseModel):
    name: str
    x: float
    y: float
    z: float = 0.0


class GeofenceItem(BaseModel):
    name: str
    polygon: list  # [[x,y], [x,y], ...]


class ScheduleItem(BaseModel):
    name: str
    patrol_route: str
    hour: int
    minute: int = 0
    weekdays: list = [0, 1, 2, 3, 4, 5, 6]
    enabled: bool = True


# gRPC request models
class GrpcLoginRequest(BaseModel):
    username: str
    password: str


class GrpcRelocalizeRequest(BaseModel):
    pcd_path: str
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


class GrpcSaveMapRequest(BaseModel):
    file_path: str
    save_patches: bool = True


class GrpcSetModeRequest(BaseModel):
    mode: str  # IDLE, MANUAL, TELEOP, AUTONOMOUS, MAPPING


class GrpcStartTaskRequest(BaseModel):
    task_type: str  # NAVIGATION, MAPPING, FOLLOW_PATH, SEMANTIC_NAV, FOLLOW_PERSON
    params_json: str = "{}"


class GrpcCancelTaskRequest(BaseModel):
    task_id: str


class GrpcReleaseLeaseRequest(BaseModel):
    lease_token: str


class GrpcSetRuntimeConfigRequest(BaseModel):
    config_json: str
    changed_fields: list = []


# ═════════════════════════════════════════════════════════════
#  FastAPI App
# ═════════════════════════════════════════════════════════════
@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    global _ssh_client
    if _ssh_client:
        _ssh_client.close()
    _close_grpc()


app = FastAPI(title="LingTu Dashboard", lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
def index():
    return FileResponse(Path(__file__).parent / "dashboard.html")


# ═════════════════════════════════════════════════════════════
#  SSH: Connection Test
# ═════════════════════════════════════════════════════════════
@app.get("/api/ping")
def ping():
    out, err, code = ssh_exec("echo OK && hostname && uptime -p", timeout=5)
    if code == 0:
        lines = out.strip().split("\n")
        return {
            "ok": True,
            "hostname": lines[1] if len(lines) > 1 else "?",
            "uptime": lines[2] if len(lines) > 2 else "?",
        }
    return {"ok": False, "error": err or "Connection failed"}


# ═════════════════════════════════════════════════════════════
#  SSH: Service Management
# ═════════════════════════════════════════════════════════════
SERVICE_GROUPS = {
    "导航": ["nav-lidar", "nav-slam", "nav-planning", "nav-autonomy", "nav-grpc", "nav-semantic"],
    "运动控制": ["brainstem"],
    "Cortex": [
        "cortex-arbiter",
        "cortex-telemetry",
        "cortex-safety",
        "cortex-control",
        "cortex-nav-gw",
        "cortex-askme-edge",
    ],
    "其他": ["askme", "ota-agent"],
}

SERVICE_DEPS = {
    "nav-slam": ["nav-lidar"],
    "nav-planning": ["nav-slam"],
    "nav-autonomy": ["nav-slam"],
    "nav-grpc": ["nav-planning"],
    "nav-semantic": ["nav-slam"],
    "cortex-safety": ["cortex-telemetry"],
    "cortex-control": ["cortex-telemetry"],
    "cortex-nav-gw": ["cortex-telemetry"],
    "cortex-arbiter": ["cortex-safety", "cortex-control", "cortex-nav-gw"],
    "cortex-askme-edge": ["cortex-arbiter"],
}


@app.get("/api/services")
def list_services():
    cmd = f'for s in {" ".join(SERVICES)}; do st=$(systemctl is-active $s 2>/dev/null || echo inactive); echo "$s $st"; done'
    out, err, code = ssh_exec(cmd)
    results = []
    for line in out.strip().split("\n"):
        parts = line.strip().split()
        if len(parts) >= 2:
            results.append({"name": parts[0], "status": parts[1]})
    return {"services": results}


@app.get("/api/services/grouped")
def list_services_grouped():
    """List services grouped by category with dependency info."""
    cmd = f'for s in {" ".join(SERVICES)}; do st=$(systemctl is-active $s 2>/dev/null || echo inactive); echo "$s $st"; done'
    out, err, code = ssh_exec(cmd)
    status_map = {}
    for line in out.strip().split("\n"):
        parts = line.strip().split()
        if len(parts) >= 2:
            status_map[parts[0]] = parts[1]
    groups = []
    for group_name, svcs in SERVICE_GROUPS.items():
        items = []
        for s in svcs:
            items.append(
                {
                    "name": s,
                    "status": status_map.get(s, "unknown"),
                    "depends_on": SERVICE_DEPS.get(s, []),
                }
            )
        active = sum(1 for i in items if i["status"] == "active")
        groups.append({"group": group_name, "services": items, "active": active, "total": len(items)})
    return {"groups": groups}


@app.post("/api/services/action")
def service_action(req: ServiceAction):
    if req.service not in SERVICES:
        return {"ok": False, "error": f"Unknown service: {req.service}"}
    if req.action not in ("start", "stop", "restart", "status"):
        return {"ok": False, "error": f"Unknown action: {req.action}"}
    out, err, code = ssh_exec(
        f"sudo systemctl {req.action} {req.service} 2>&1; systemctl is-active {req.service} 2>/dev/null",
        timeout=20,
    )
    lines = out.strip().split("\n")
    new_status = lines[-1] if lines else "unknown"
    return {"ok": code == 0 or req.action == "status", "output": out.strip(), "status": new_status}


CORTEX_PORTS = {
    "cortex-arbiter": 5050,
    "cortex-safety": 5051,
    "cortex-control": 5070,
    "cortex-telemetry": 5080,
    "cortex-nav-gw": 5090,
    "cortex-askme-edge": 5100,
}


@app.get("/api/cortex/detail")
def cortex_detail():
    """Get Cortex microservice details: status, ports, health."""
    services = []
    for svc, port in CORTEX_PORTS.items():
        out, _, _ = ssh_exec(
            f"systemctl is-active {svc} 2>/dev/null; "
            f"ss -tlnp 2>/dev/null | grep ':{port} ' | head -1; "
            f"systemctl show {svc} --property=ActiveEnterTimestamp --value 2>/dev/null"
        )
        lines = out.strip().split("\n")
        status = lines[0] if lines else "unknown"
        port_listening = str(port) in (lines[1] if len(lines) > 1 else "")
        started_at = lines[2] if len(lines) > 2 else ""
        services.append(
            {
                "name": svc,
                "status": status,
                "port": port,
                "port_listening": port_listening,
                "started_at": started_at.strip(),
                "depends_on": SERVICE_DEPS.get(svc, []),
            }
        )
    active = sum(1 for s in services if s["status"] == "active")
    return {"services": services, "active": active, "total": len(services)}


# ═════════════════════════════════════════════════════════════
#  SSH: Stack Control
# ═════════════════════════════════════════════════════════════
def _stack_start(order: list[str]) -> dict:
    results = []
    for svc in order:
        out, err, code = ssh_exec(
            f"sudo systemctl start {svc} 2>&1; sleep 1; systemctl is-active {svc} 2>/dev/null",
            timeout=15,
        )
        status = out.strip().split("\n")[-1]
        results.append({"service": svc, "status": status})
    return {"ok": True, "results": results}


def _stack_stop(order: list[str]) -> dict:
    results = []
    for svc in reversed(order):
        out, err, code = ssh_exec(
            f"sudo systemctl stop {svc} 2>&1; systemctl is-active {svc} 2>/dev/null",
            timeout=10,
        )
        status = out.strip().split("\n")[-1]
        results.append({"service": svc, "status": status})
    return {"ok": True, "results": results}


@app.post("/api/stack/nav/start")
def stack_nav_start():
    """Start navigation stack in dependency order."""
    return _stack_start(STACK_NAV_ORDER)


@app.post("/api/stack/nav/stop")
def stack_nav_stop():
    """Stop navigation stack in reverse order."""
    return _stack_stop(STACK_NAV_ORDER)


@app.post("/api/stack/cortex/start")
def stack_cortex_start():
    """Start cortex runtime stack."""
    return _stack_start(STACK_CORTEX_ORDER)


@app.post("/api/stack/cortex/stop")
def stack_cortex_stop():
    """Stop cortex runtime stack."""
    return _stack_stop(STACK_CORTEX_ORDER)


@app.post("/api/stack/all/start")
def stack_all_start():
    """Start all services in dependency order."""
    return _stack_start(STACK_ALL_ORDER)


@app.post("/api/stack/all/stop")
def stack_all_stop():
    """Stop all services in reverse order."""
    return _stack_stop(STACK_ALL_ORDER)


# Legacy endpoints (kept for compatibility)
@app.post("/api/stack/start")
def stack_start_legacy():
    return _stack_start(STACK_NAV_ORDER)


@app.post("/api/stack/stop")
def stack_stop_legacy():
    return _stack_stop(STACK_NAV_ORDER)


# ═════════════════════════════════════════════════════════════
#  SSH: Topic Monitor
# ═════════════════════════════════════════════════════════════
@app.get("/api/topics")
def list_topics():
    out, _, _ = ssh_exec_ros("ros2 topic list 2>/dev/null", timeout=10)
    topics = [t.strip() for t in out.strip().split("\n") if t.strip().startswith("/")]
    return {"topics": topics, "count": len(topics)}


@app.get("/api/topics/hz")
def topics_hz():
    """Get activity status for monitored topics."""
    topic_list = " ".join(TOPICS_MONITOR)
    cmd = f"""for t in {topic_list}; do
  cnt=$(ros2 topic info $t 2>/dev/null | grep -c 'Publisher count: [1-9]' || echo 0)
  if [ "$cnt" -gt 0 ]; then
    echo "$t ACTIVE"
  else
    echo "$t INACTIVE"
  fi
done"""
    out, _, _ = ssh_exec_ros(cmd, timeout=15)
    results = []
    for line in out.strip().split("\n"):
        parts = line.strip().split()
        if len(parts) >= 2:
            results.append({"topic": parts[0], "active": parts[1] == "ACTIVE"})
    return {"topics": results}


@app.get("/api/topics/echo/{topic_path:path}")
def topic_echo(topic_path: str):
    """Get one message from a topic."""
    topic = f"/{topic_path}"
    _validate_topic(topic)
    out, err, code = ssh_exec_ros(
        f"timeout 3 ros2 topic echo {topic} --once 2>/dev/null | head -50",
        timeout=8,
    )
    return {
        "topic": topic,
        "message": out.strip() if out.strip() else "(no message received within 3s)",
        "ok": bool(out.strip()),
    }


# ═════════════════════════════════════════════════════════════
#  SSH: Map Management
# ═════════════════════════════════════════════════════════════
@app.get("/api/maps")
def list_maps():
    out2, _, _ = ssh_exec(f"find {MAP_DIR} -maxdepth 2 -name '*.pcd' -o -name '*.pickle' 2>/dev/null | head -50")
    maps = []
    for line in out2.strip().split("\n"):
        if line.strip():
            fname = line.strip()
            info_out, _, _ = ssh_exec(f"stat --format='%s %y' '{fname}' 2>/dev/null")
            parts = info_out.strip().split(" ", 1)
            size_bytes = int(parts[0]) if parts[0].isdigit() else 0
            mtime = parts[1][:19] if len(parts) > 1 else ""
            size_mb = size_bytes / (1024 * 1024)
            maps.append(
                {
                    "path": fname,
                    "name": fname.replace(MAP_DIR + "/", ""),
                    "size": f"{size_mb:.1f} MB",
                    "size_bytes": size_bytes,
                    "modified": mtime,
                }
            )
    return {"maps": maps, "map_dir": MAP_DIR}


@app.post("/api/maps/save")
def save_map():
    """Trigger map save via ROS2 service."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    save_path = f"{MAP_DIR}/map_{ts}"
    out, err, code = ssh_exec_ros(
        f"ros2 service call /nav/save_map interface/srv/SaveMaps "
        f"\"{{file_path: '{save_path}', save_patches: true}}\" 2>&1",
        timeout=30,
    )
    return {"ok": "success" in out.lower() or code == 0, "path": save_path, "output": out.strip()}


@app.delete("/api/maps/{map_name:path}")
def delete_map(map_name: str):
    _validate_map_name(map_name)
    full_path = f"{MAP_DIR}/{map_name}"
    out, err, code = ssh_exec(f"rm -rf '{full_path}' 2>&1 && echo DELETED")
    return {"ok": "DELETED" in out, "output": out.strip()}


@app.get("/api/maps/detail")
def list_maps_detail():
    """List map directories with metadata and active status."""
    # Find map directories (each should contain .pcd + .pickle)
    out, _, _ = ssh_exec(f"ls -d {MAP_DIR}/*/ 2>/dev/null; readlink {MAP_DIR}/active 2>/dev/null")
    lines = [l.strip().rstrip("/") for l in out.strip().split("\n") if l.strip()]
    active_target = ""
    dirs = []
    for line in lines:
        if "/" not in line and not line.startswith(MAP_DIR):
            active_target = line  # This is the readlink result
        elif line != f"{MAP_DIR}/active":
            dirs.append(line)

    maps = []
    for d in dirs:
        name = d.split("/")[-1]
        # Check for map files
        info_out, _, _ = ssh_exec(
            f"stat --format='%s' '{d}/map.pcd' 2>/dev/null; "
            f"stat --format='%s' '{d}/map.pickle' 2>/dev/null; "
            f"cat '{d}/metadata.yaml' 2>/dev/null | head -10"
        )
        info_lines = info_out.strip().split("\n")
        pcd_size = int(info_lines[0]) if info_lines and info_lines[0].isdigit() else 0
        pickle_size = int(info_lines[1]) if len(info_lines) > 1 and info_lines[1].isdigit() else 0
        metadata_text = "\n".join(info_lines[2:]) if len(info_lines) > 2 else ""
        maps.append(
            {
                "name": name,
                "path": d,
                "active": name == active_target,
                "pcd_size_mb": round(pcd_size / (1024 * 1024), 1),
                "pickle_size_mb": round(pickle_size / (1024 * 1024), 1),
                "metadata": metadata_text,
            }
        )
    return {"maps": maps, "active": active_target, "map_dir": MAP_DIR}


class MapActivateRequest(BaseModel):
    name: str


@app.post("/api/maps/activate")
def activate_map(req: MapActivateRequest):
    """Switch active map symlink and optionally restart nav-planning."""
    target = f"{MAP_DIR}/{req.name}"
    out, err, code = ssh_exec(f"test -d '{target}' && cd {MAP_DIR} && ln -sfn {req.name} active && readlink active")
    if req.name in out:
        return {"ok": True, "active": req.name, "output": f"Active map → {req.name}"}
    return {"ok": False, "error": f"Failed to activate: {err or out}"}


@app.websocket("/ws/logs/{service}")
async def ws_log_stream(ws: WebSocket, service: str):
    """Stream journalctl logs in real-time via WebSocket."""
    await ws.accept()
    import subprocess

    try:
        ssh = get_ssh()
    except Exception:
        ssh = None
    if ssh is None:
        await ws.send_json({"error": "SSH not connected"})
        await ws.close()
        return
    try:
        stdin, stdout, stderr = ssh.exec_command(
            f"journalctl --user -u {service} -f --no-pager -n 0 2>/dev/null || "
            f"sudo journalctl -u {service} -f --no-pager -n 0 2>/dev/null",
            get_pty=False,
        )
        stdout.channel.settimeout(1.0)
        while True:
            try:
                line = stdout.readline()
                if line:
                    await ws.send_json({"line": line.rstrip()})
                else:
                    await asyncio.sleep(0.2)
            except Exception:
                # Check if websocket is still open
                try:
                    await asyncio.wait_for(ws.receive_text(), timeout=0.01)
                except Exception:
                    pass
                await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        try:
            await ws.send_json({"error": str(e)})
        except Exception:
            pass


# ═════════════════════════════════════════════════════════════
#  SSH: Navigation Control
# ═════════════════════════════════════════════════════════════
@app.post("/api/nav/goal")
def send_nav_goal(goal: NavGoal):
    cmd = (
        f"timeout 8 ros2 topic pub --once /nav/goal_pose geometry_msgs/msg/PoseStamped "
        f"\"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {goal.x}, y: {goal.y}, z: {goal.z}}}, "
        f'orientation: {{w: 1.0}}}}}}" 2>&1'
    )
    out, err, code = ssh_exec_ros(cmd, timeout=10)
    return {"ok": code == 0, "goal": {"x": goal.x, "y": goal.y, "z": goal.z}, "output": out.strip()}


@app.post("/api/nav/cancel")
def cancel_nav():
    out, _, code = ssh_exec_ros("timeout 5 ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}' 2>&1", timeout=5)
    return {"ok": True, "output": "Cancel signal sent"}


@app.post("/api/nav/estop")
def emergency_stop():
    out, _, code = ssh_exec_ros(
        'timeout 5 ros2 topic pub --once /nav/stop std_msgs/msg/Int8 "{data: 2}" 2>&1', timeout=5
    )
    return {"ok": True, "output": "E-STOP sent (stop=2)"}


@app.post("/api/nav/clear_stop")
def clear_stop():
    out, _, code = ssh_exec_ros(
        'timeout 5 ros2 topic pub --once /nav/stop std_msgs/msg/Int8 "{data: 0}" 2>&1', timeout=5
    )
    return {"ok": True, "output": "Stop cleared (stop=0)"}


@app.post("/api/nav/semantic")
def send_semantic_goal(goal: SemanticGoal):
    cmd = (
        f"timeout 8 ros2 topic pub --once /nav/semantic/instruction std_msgs/msg/String "
        f"\"{{data: '{goal.instruction}'}}\" 2>&1"
    )
    out, _, code = ssh_exec_ros(cmd, timeout=10)
    return {"ok": code == 0, "instruction": goal.instruction, "output": out.strip()}


# ═════════════════════════════════════════════════════════════
#  SSH: Planner Status
# ═════════════════════════════════════════════════════════════
@app.get("/api/nav/status")
def nav_status():
    cmds = [
        "timeout 2 ros2 topic echo /nav/planner_status --once 2>/dev/null || echo 'NO_DATA'",
        "timeout 2 ros2 topic echo /nav/adapter_status --once 2>/dev/null || echo 'NO_DATA'",
        "timeout 2 ros2 topic echo /nav/mission_status --once 2>/dev/null || echo 'NO_DATA'",
    ]
    cmd = " && echo '---SPLIT---' && ".join(cmds)
    out, _, _ = ssh_exec_ros(cmd, timeout=15)
    parts = out.split("---SPLIT---")
    result = {
        "planner_status": parts[0].strip() if len(parts) > 0 else "N/A",
        "adapter_status": parts[1].strip() if len(parts) > 1 else "N/A",
    }
    if len(parts) > 2:
        mission_raw = parts[2].strip()
        if mission_raw and mission_raw != "NO_DATA":
            for line in mission_raw.split("\n"):
                line = line.strip()
                if line.startswith("data:"):
                    json_str = line[5:].strip().strip("'\"")
                    try:
                        result["mission"] = json.loads(json_str)
                    except Exception:
                        result["mission"] = {"state": "PARSE_ERROR", "raw": json_str}
                    break
    return result


# ═════════════════════════════════════════════════════════════
#  SSH: Health Check
# ═════════════════════════════════════════════════════════════
@app.get("/api/health")
def health_check():
    checks = []

    # 1. SSH connectivity
    out, _, code = ssh_exec("echo OK", timeout=5)
    checks.append({"name": "SSH Connection", "ok": code == 0})

    # 2. ROS2 daemon
    out, _, code = ssh_exec_ros("ros2 node list 2>/dev/null | head -20", timeout=10)
    nodes = [n.strip() for n in out.strip().split("\n") if n.strip().startswith("/")]
    checks.append({"name": "ROS2 Nodes", "ok": len(nodes) > 0, "detail": f"{len(nodes)} nodes"})

    # 3. Key services
    key_services = ["nav-slam", "nav-autonomy", "nav-planning"]
    for svc in key_services:
        out, _, code = ssh_exec(f"systemctl is-active {svc} 2>/dev/null")
        checks.append({"name": f"Service: {svc}", "ok": out.strip() == "active", "detail": out.strip()})

    # 4. gRPC connectivity
    grpc_ok = False
    if GRPC_AVAILABLE:
        try:
            _grpc_call("system", "GetRobotInfo", timeout=3)
            grpc_ok = True
        except Exception:
            pass
    checks.append({"name": "gRPC Gateway", "ok": grpc_ok, "detail": "connected" if grpc_ok else "unreachable"})

    # 5. Disk space
    out, _, _ = ssh_exec("df -h / | tail -1 | awk '{print $5, $4}'")
    parts = out.strip().split()
    usage = parts[0] if parts else "?"
    avail = parts[1] if len(parts) > 1 else "?"
    usage_pct = int(usage.replace("%", "")) if "%" in usage else 0
    checks.append({"name": "Disk Space", "ok": usage_pct < 90, "detail": f"{usage} used, {avail} free"})

    # 6. Memory
    out, _, _ = ssh_exec("free -h | grep Mem | awk '{print $3, $2}'")
    checks.append({"name": "Memory", "ok": True, "detail": out.strip().replace(" ", " / ")})

    # 7. CPU temp
    out, _, _ = ssh_exec("cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0")
    temp_c = int(out.strip()) / 1000 if out.strip().isdigit() else 0
    checks.append({"name": "CPU Temp", "ok": temp_c < 80, "detail": f"{temp_c:.0f}C"})

    all_ok = all(c["ok"] for c in checks)
    return {"ok": all_ok, "checks": checks, "node_count": len(nodes), "nodes": nodes}


# ═════════════════════════════════════════════════════════════
#  SSH: Log Viewer
# ═════════════════════════════════════════════════════════════
@app.get("/api/logs/{service}")
def get_logs(service: str, lines: int = 50):
    if service not in SERVICES and service != "all":
        return {"ok": False, "error": "Unknown service"}
    if service == "all":
        out, _, _ = ssh_exec(f"journalctl --no-pager -n {lines} --output=short-iso 2>/dev/null")
    else:
        out, _, _ = ssh_exec(f"journalctl -u {service} --no-pager -n {lines} --output=short-iso 2>/dev/null")
    return {"service": service, "logs": out.strip(), "lines": lines}


# ═════════════════════════════════════════════════════════════
#  SSH: Parameters
# ═════════════════════════════════════════════════════════════
@app.get("/api/params/{node_name:path}")
def get_params(node_name: str):
    node = f"/{node_name}"
    _validate_topic(node)  # ROS2 node names follow topic format
    out, _, code = ssh_exec_ros(f"ros2 param list {node} 2>/dev/null | head -100", timeout=10)
    params = [p.strip() for p in out.strip().split("\n") if p.strip()]
    return {"node": node, "params": params, "count": len(params)}


@app.post("/api/params/set")
def set_param(req: ParamUpdate):
    _validate_ros_name(req.node)
    _validate_ros_name(req.param)
    # 参数值只允许数字、布尔和简单字符串
    if not _re.match(r"^[a-zA-Z0-9_.:\-/ ]+$", str(req.value)):
        return {"ok": False, "output": f"Invalid parameter value: {req.value}"}
    out, err, code = ssh_exec_ros(f"ros2 param set {req.node} {req.param} {req.value} 2>&1", timeout=10)
    return {"ok": "successfully" in out.lower(), "output": out.strip()}


# ═════════════════════════════════════════════════════════════
#  SSH: SLAM Profile
# ═════════════════════════════════════════════════════════════
@app.get("/api/slam/profile")
def get_slam_profile():
    out, _, _ = ssh_exec_ros("ros2 node list 2>/dev/null | grep -E 'pointlio|lio_node|fastlio'")
    nodes = out.strip()
    if "pointlio" in nodes:
        return {"profile": "pointlio"}
    elif "lio_node" in nodes or "fastlio" in nodes:
        return {"profile": "fastlio2"}
    return {"profile": "none"}


# ═════════════════════════════════════════════════════════════
#  SSH: System Info
# ═════════════════════════════════════════════════════════════
@app.get("/api/system")
def system_info():
    cmd = """echo "---HOST---" && hostname
echo "---ARCH---" && uname -m
echo "---OS---" && cat /etc/os-release 2>/dev/null | grep PRETTY_NAME | cut -d= -f2 | tr -d '"'
echo "---UPTIME---" && uptime -p
echo "---CPU---" && nproc
echo "---MEM---" && free -h | grep Mem | awk '{print $2}'
echo "---DISK---" && df -h / | tail -1 | awk '{print $2, $3, $4, $5}'
echo "---TEMP---" && cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0
echo "---IP---" && hostname -I | awk '{print $1}'
"""
    out, _, _ = ssh_exec(cmd, timeout=10)
    info = {}
    current_key = None
    for line in out.strip().split("\n"):
        if line.startswith("---") and line.endswith("---"):
            current_key = line.strip("-").lower()
        elif current_key:
            info[current_key] = line.strip()
            current_key = None
    if "temp" in info:
        try:
            info["temp"] = f"{int(info['temp']) / 1000:.0f}C"
        except ValueError:
            pass
    return info


# ═════════════════════════════════════════════════════════════
#  SSH: WebSocket for live status updates
# ═════════════════════════════════════════════════════════════
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            try:
                out, _, code = await asyncio.to_thread(ssh_exec, "echo OK", 3)
                connected = code == 0

                svc_cmd = f'for s in {" ".join(SERVICES)}; do st=$(systemctl is-active $s 2>/dev/null || echo inactive); echo "$s $st"; done'
                svc_out, _, _ = await asyncio.to_thread(ssh_exec, svc_cmd, 8)
                services = {}
                for line in svc_out.strip().split("\n"):
                    parts = line.strip().split()
                    if len(parts) >= 2:
                        services[parts[0]] = parts[1]

                await ws.send_json(
                    {
                        "type": "status",
                        "connected": connected,
                        "services": services,
                        "timestamp": time.time(),
                    }
                )
            except Exception as e:
                await ws.send_json(
                    {
                        "type": "status",
                        "connected": False,
                        "error": str(e),
                        "timestamp": time.time(),
                    }
                )

            await asyncio.sleep(5)
    except WebSocketDisconnect:
        pass


# ═════════════════════════════════════════════════════════════
#  SSH: Patrol Routes
# ═════════════════════════════════════════════════════════════
@app.get("/api/patrol/list")
def patrol_list():
    out, _, code = ssh_exec(f"ls {PATROL_DIR}/*.yaml 2>/dev/null", timeout=5)
    routes = []
    if code == 0 and out.strip():
        for fpath in out.strip().split("\n"):
            fpath = fpath.strip()
            if not fpath:
                continue
            content, _, rc = ssh_exec(f"cat '{fpath}' 2>/dev/null", timeout=5)
            if rc == 0 and content.strip():
                try:
                    data = yaml.safe_load(content)
                    routes.append(
                        {
                            "name": data.get("name", fpath.split("/")[-1].replace(".yaml", "")),
                            "waypoints": len(data.get("waypoints", [])),
                            "loop": data.get("loop", False),
                        }
                    )
                except Exception:
                    pass
    return {"ok": True, "routes": routes}


@app.post("/api/patrol/save")
def patrol_save(route: PatrolRoute):
    data = {"name": route.name, "waypoints": route.waypoints, "loop": route.loop}
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    filepath = f"{PATROL_DIR}/{route.name}.yaml"
    cmd = f"mkdir -p {PATROL_DIR} && cat > '{filepath}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    out, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "path": filepath, "error": err if code != 0 else None}


@app.delete("/api/patrol/{name}")
def patrol_delete(name: str):
    filepath = f"{PATROL_DIR}/{name}.yaml"
    out, _, code = ssh_exec(f"rm -f '{filepath}' 2>&1 && echo DELETED", timeout=5)
    return {"ok": "DELETED" in out, "output": out.strip()}


@app.post("/api/patrol/start")
def patrol_start(action: PatrolAction):
    filepath = f"{PATROL_DIR}/{action.name}.yaml"
    content, _, code = ssh_exec(f"cat '{filepath}' 2>/dev/null", timeout=5)
    if code != 0 or not content.strip():
        return {"ok": False, "error": f"Route '{action.name}' not found"}
    try:
        data = yaml.safe_load(content)
    except Exception as e:
        return {"ok": False, "error": f"Invalid YAML: {e}"}
    payload = json.dumps(data).replace('"', '\\"')
    cmd = f'timeout 8 ros2 topic pub --once /nav/patrol_goals std_msgs/msg/String "{{data: \\"{payload}\\"}}" 2>&1'
    out, _, code = ssh_exec_ros(cmd, timeout=10)
    return {"ok": code == 0, "route": action.name, "output": out.strip()}


@app.post("/api/patrol/stop")
def patrol_stop():
    out, _, code = ssh_exec_ros("timeout 5 ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}' 2>&1", timeout=5)
    return {"ok": True, "output": "Patrol cancel signal sent"}


# ═════════════════════════════════════════════════════════════
#  SSH: POI Management
# ═════════════════════════════════════════════════════════════
POIS_FILE = f"{DATA_DIR}/pois.yaml"


@app.get("/api/poi/list")
def poi_list():
    out, _, code = ssh_exec(f"cat {POIS_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    pois = data.get("pois", []) if isinstance(data, dict) else []
    return {"ok": True, "pois": pois}


@app.post("/api/poi/save")
def poi_save(poi: POIItem):
    out, _, _ = ssh_exec(f"cat {POIS_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        data = {}
    pois = data.get("pois", [])
    updated = False
    for i, p in enumerate(pois):
        if p.get("name") == poi.name:
            pois[i] = {"name": poi.name, "x": poi.x, "y": poi.y, "z": poi.z}
            updated = True
            break
    if not updated:
        pois.append({"name": poi.name, "x": poi.x, "y": poi.y, "z": poi.z})
    data["pois"] = pois
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"mkdir -p {DATA_DIR} && cat > '{POIS_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "action": "updated" if updated else "created", "error": err if code != 0 else None}


@app.delete("/api/poi/{name}")
def poi_delete(name: str):
    out, _, _ = ssh_exec(f"cat {POIS_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        return {"ok": False, "error": "No POIs file found"}
    pois = data.get("pois", [])
    new_pois = [p for p in pois if p.get("name") != name]
    if len(new_pois) == len(pois):
        return {"ok": False, "error": f"POI '{name}' not found"}
    data["pois"] = new_pois
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"cat > '{POIS_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "error": err if code != 0 else None}


@app.post("/api/poi/navigate")
def poi_navigate(action: PatrolAction):
    out, _, _ = ssh_exec(f"cat {POIS_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        return {"ok": False, "error": "No POIs file found"}
    pois = data.get("pois", [])
    target = None
    for p in pois:
        if p.get("name") == action.name:
            target = p
            break
    if not target:
        return {"ok": False, "error": f"POI '{action.name}' not found"}
    cmd = (
        f"timeout 8 ros2 topic pub --once /nav/goal_pose geometry_msgs/msg/PoseStamped "
        f"\"{{header: {{frame_id: 'map'}}, pose: {{position: {{x: {target['x']}, y: {target['y']}, z: {target.get('z', 0.0)}}}, "
        f'orientation: {{w: 1.0}}}}}}" 2>&1'
    )
    out, _, code = ssh_exec_ros(cmd, timeout=10)
    return {"ok": code == 0, "poi": target, "output": out.strip()}


# ═════════════════════════════════════════════════════════════
#  SSH: Geofence Management
# ═════════════════════════════════════════════════════════════
GEOFENCES_FILE = f"{DATA_DIR}/geofences.yaml"


@app.get("/api/geofence/list")
def geofence_list():
    out, _, _ = ssh_exec(f"cat {GEOFENCES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    fences = data.get("geofences", []) if isinstance(data, dict) else []
    return {"ok": True, "geofences": fences}


@app.post("/api/geofence/save")
def geofence_save(fence: GeofenceItem):
    out, _, _ = ssh_exec(f"cat {GEOFENCES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        data = {}
    fences = data.get("geofences", [])
    updated = False
    for i, f in enumerate(fences):
        if f.get("name") == fence.name:
            fences[i] = {"name": fence.name, "polygon": fence.polygon, "enabled": fences[i].get("enabled", True)}
            updated = True
            break
    if not updated:
        fences.append({"name": fence.name, "polygon": fence.polygon, "enabled": True})
    data["geofences"] = fences
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"mkdir -p {DATA_DIR} && cat > '{GEOFENCES_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "action": "updated" if updated else "created", "error": err if code != 0 else None}


@app.delete("/api/geofence/{name}")
def geofence_delete(name: str):
    out, _, _ = ssh_exec(f"cat {GEOFENCES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        return {"ok": False, "error": "No geofences file found"}
    fences = data.get("geofences", [])
    new_fences = [f for f in fences if f.get("name") != name]
    if len(new_fences) == len(fences):
        return {"ok": False, "error": f"Geofence '{name}' not found"}
    data["geofences"] = new_fences
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"cat > '{GEOFENCES_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "error": err if code != 0 else None}


@app.post("/api/geofence/toggle")
def geofence_toggle(action: PatrolAction):
    out, _, _ = ssh_exec(f"cat {GEOFENCES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        return {"ok": False, "error": "No geofences file found"}
    fences = data.get("geofences", [])
    found = False
    new_state = None
    for f in fences:
        if f.get("name") == action.name:
            f["enabled"] = not f.get("enabled", True)
            new_state = f["enabled"]
            found = True
            break
    if not found:
        return {"ok": False, "error": f"Geofence '{action.name}' not found"}
    data["geofences"] = fences
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"cat > '{GEOFENCES_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "enabled": new_state, "error": err if code != 0 else None}


# ═════════════════════════════════════════════════════════════
#  SSH: Scheduled Tasks
# ═════════════════════════════════════════════════════════════
SCHEDULES_FILE = f"{DATA_DIR}/schedules.yaml"


@app.get("/api/schedule/list")
def schedule_list():
    out, _, _ = ssh_exec(f"cat {SCHEDULES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    schedules = data.get("schedules", []) if isinstance(data, dict) else []
    return {"ok": True, "schedules": schedules}


@app.post("/api/schedule/save")
def schedule_save(item: ScheduleItem):
    out, _, _ = ssh_exec(f"cat {SCHEDULES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        data = {}
    schedules = data.get("schedules", [])
    entry = {
        "name": item.name,
        "patrol_route": item.patrol_route,
        "hour": item.hour,
        "minute": item.minute,
        "weekdays": item.weekdays,
        "enabled": item.enabled,
    }
    updated = False
    for i, s in enumerate(schedules):
        if s.get("name") == item.name:
            schedules[i] = entry
            updated = True
            break
    if not updated:
        schedules.append(entry)
    data["schedules"] = schedules
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"mkdir -p {DATA_DIR} && cat > '{SCHEDULES_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "action": "updated" if updated else "created", "error": err if code != 0 else None}


@app.delete("/api/schedule/{name}")
def schedule_delete(name: str):
    out, _, _ = ssh_exec(f"cat {SCHEDULES_FILE} 2>/dev/null", timeout=5)
    data = yaml.safe_load(out) if out.strip() else {}
    if not isinstance(data, dict):
        return {"ok": False, "error": "No schedules file found"}
    schedules = data.get("schedules", [])
    new_schedules = [s for s in schedules if s.get("name") != name]
    if len(new_schedules) == len(schedules):
        return {"ok": False, "error": f"Schedule '{name}' not found"}
    data["schedules"] = new_schedules
    yaml_str = yaml.safe_dump(data, allow_unicode=True, default_flow_style=False)
    cmd = f"cat > '{SCHEDULES_FILE}' << 'YAMLEOF'\n{yaml_str}YAMLEOF"
    _, err, code = ssh_exec(cmd, timeout=10)
    return {"ok": code == 0, "error": err if code != 0 else None}


# ═════════════════════════════════════════════════════════════
#  SSH: Mission History
# ═════════════════════════════════════════════════════════════
HISTORY_DIR = f"{DATA_DIR}/mission_history"


@app.get("/api/history/list")
def history_list():
    out, _, code = ssh_exec(f"ls -t {HISTORY_DIR}/*.json 2>/dev/null | head -50", timeout=10)
    missions = []
    if code == 0 and out.strip():
        for fpath in out.strip().split("\n"):
            fpath = fpath.strip()
            if not fpath:
                continue
            content, _, rc = ssh_exec(f"cat '{fpath}' 2>/dev/null", timeout=5)
            if rc == 0 and content.strip():
                try:
                    data = json.loads(content)
                    mission_id = fpath.split("/")[-1].replace(".json", "")
                    missions.append(
                        {
                            "id": mission_id,
                            "status": data.get("status", "unknown"),
                            "start_time": data.get("start_time"),
                            "end_time": data.get("end_time"),
                            "type": data.get("type"),
                            "summary": data.get("summary", ""),
                        }
                    )
                except Exception:
                    pass
    return {"ok": True, "missions": missions}


@app.get("/api/history/{mission_id}")
def history_detail(mission_id: str):
    filepath = f"{HISTORY_DIR}/{mission_id}.json"
    content, _, code = ssh_exec(f"cat '{filepath}' 2>/dev/null", timeout=5)
    if code != 0 or not content.strip():
        return {"ok": False, "error": f"Mission '{mission_id}' not found"}
    try:
        data = json.loads(content)
        return {"ok": True, "mission": data}
    except Exception as e:
        return {"ok": False, "error": f"Invalid JSON: {e}"}


# ═════════════════════════════════════════════════════════════
#  SSH: Real-time Status Topics
# ═════════════════════════════════════════════════════════════
def _parse_ros_string_topic(out: str) -> dict | None:
    """Parse a ROS2 String topic echo output and extract JSON from data: field."""
    for line in out.strip().split("\n"):
        line = line.strip()
        if line.startswith("data:"):
            json_str = line[5:].strip().strip("'\"")
            try:
                return json.loads(json_str)
            except Exception:
                return json_str
    return None


def _read_ros_string_topic(topic: str, timeout_sec: int = 3) -> dict | None:
    """Read one ROS2 String topic via helper script (avoids ros2 echo truncation)."""
    out, _, code = ssh_exec_ros(
        f"timeout {timeout_sec + 2} python3 /opt/nav/tools/read_topic.py {topic} {timeout_sec} 2>/dev/null",
        timeout=timeout_sec + 5,
    )
    raw = out.strip()
    if not raw:
        return None
    try:
        return json.loads(raw)
    except Exception:
        return raw


@app.get("/api/nav/semantic/status")
def nav_semantic_status():
    """Get semantic planner status from /nav/semantic/status topic."""
    parsed = _read_ros_string_topic("/nav/semantic/status", timeout_sec=3)
    if parsed is None:
        return {"ok": False, "error": "No semantic status received within 3s"}
    if isinstance(parsed, dict):
        return {"ok": True, "status": parsed}
    return {"ok": True, "raw": str(parsed)}


@app.get("/api/nav/semantic/scene_graph_stats")
def nav_semantic_scene_graph_stats():
    """Get scene graph statistics from /nav/semantic/scene_graph topic."""
    parsed = _read_ros_string_topic("/nav/semantic/scene_graph", timeout_sec=3)
    if parsed is None:
        return {"ok": False, "error": "No scene graph received within 3s"}
    if isinstance(parsed, dict):
        objects = parsed.get("objects", [])
        regions = parsed.get("regions", [])
        relations = parsed.get("relations", [])
        return {
            "ok": True,
            "object_count": len(objects),
            "region_count": len(regions),
            "relation_count": len(relations),
            "objects": [
                {"id": o.get("id", ""), "label": o.get("label", ""), "score": o.get("score", 0)} for o in objects[:20]
            ],
            "regions": [{"name": r.get("name", ""), "object_count": len(r.get("object_ids", []))} for r in regions],
        }
    return {"ok": False, "error": "Could not parse scene graph JSON"}


@app.get("/api/status/dialogue")
def status_dialogue():
    out, _, code = ssh_exec_ros(
        "timeout 3 ros2 topic echo /nav/dialogue_state --once 2>/dev/null | head -50", timeout=8
    )
    if not out.strip():
        return {"ok": False, "error": "No dialogue state received within 3s"}
    parsed = _parse_ros_string_topic(out)
    if parsed is not None:
        return {"ok": True, "state": parsed}
    return {"ok": True, "raw": out.strip()}


@app.get("/api/status/safety")
def status_safety():
    out, _, code = ssh_exec_ros("timeout 3 ros2 topic echo /nav/safety_state --once 2>/dev/null | head -50", timeout=8)
    if not out.strip():
        return {"ok": False, "error": "No safety state received within 3s"}
    parsed = _parse_ros_string_topic(out)
    if parsed is not None:
        return {"ok": True, "state": parsed}
    return {"ok": True, "raw": out.strip()}


@app.get("/api/status/eval")
def status_eval():
    out, _, code = ssh_exec_ros(
        "timeout 3 ros2 topic echo /nav/execution_eval --once 2>/dev/null | head -50", timeout=8
    )
    if not out.strip():
        return {"ok": False, "error": "No execution eval received within 3s"}
    parsed = _parse_ros_string_topic(out)
    if parsed is not None:
        return {"ok": True, "eval": parsed}
    return {"ok": True, "raw": out.strip()}


# ═════════════════════════════════════════════════════════════
#  gRPC: Connection Status
# ═════════════════════════════════════════════════════════════
@app.get("/api/grpc/status")
def grpc_status():
    """Check gRPC connection health."""
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC libraries not installed", "available": False}
    try:
        result = _grpc_call("system", "GetRobotInfo", timeout=3)
        return {"ok": True, "available": True, "robot_info": result}
    except Exception as e:
        return {"ok": False, "available": True, "error": _grpc_err(e)["error"]}


# ═════════════════════════════════════════════════════════════
#  gRPC: SystemService
# ═════════════════════════════════════════════════════════════
@app.post("/api/grpc/system/login")
def grpc_system_login(req: GrpcLoginRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.LoginRequest(username=req.username, password=req.password)
        result = _grpc_call("system", "Login", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/system/heartbeat")
def grpc_system_heartbeat():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.HeartbeatRequest()
        result = _grpc_call("system", "Heartbeat", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/system/info")
def grpc_system_info():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        result = _grpc_call("system", "GetRobotInfo")
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/system/capabilities")
def grpc_system_capabilities():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        result = _grpc_call("system", "GetCapabilities")
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/system/relocalize")
def grpc_system_relocalize(req: GrpcRelocalizeRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.RelocalizeRequest(
            pcd_path=req.pcd_path,
            x=req.x,
            y=req.y,
            z=req.z,
            yaw=req.yaw,
        )
        result = _grpc_call("system", "Relocalize", request, timeout=30)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/system/save_map")
def grpc_system_save_map(req: GrpcSaveMapRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.SaveMapRequest(
            file_path=req.file_path,
            save_patches=req.save_patches,
        )
        result = _grpc_call("system", "SaveMap", request, timeout=30)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/system/maps")
def grpc_system_list_maps():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.ListMapsRequest(directory=MAP_DIR)
        result = _grpc_call("system", "ListMaps", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/system/config")
def grpc_system_get_config():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.GetRuntimeConfigRequest()
        result = _grpc_call("system", "GetRuntimeConfig", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/system/config")
def grpc_system_set_config(req: GrpcSetRuntimeConfigRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = system_pb2.SetRuntimeConfigRequest(
            config_json=req.config_json,
            changed_fields=req.changed_fields,
        )
        result = _grpc_call("system", "SetRuntimeConfig", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


# ═════════════════════════════════════════════════════════════
#  gRPC: ControlService
# ═════════════════════════════════════════════════════════════
@app.post("/api/grpc/control/lease/acquire")
def grpc_control_acquire_lease():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.AcquireLeaseRequest()
        result = _grpc_call("control", "AcquireLease", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/lease/release")
def grpc_control_release_lease(req: GrpcReleaseLeaseRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.ReleaseLeaseRequest(lease_token=req.lease_token)
        result = _grpc_call("control", "ReleaseLease", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/mode")
def grpc_control_set_mode(req: GrpcSetModeRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        mode_map = {
            "IDLE": control_pb2.ROBOT_MODE_IDLE,
            "MANUAL": control_pb2.ROBOT_MODE_MANUAL,
            "TELEOP": control_pb2.ROBOT_MODE_TELEOP,
            "AUTONOMOUS": control_pb2.ROBOT_MODE_AUTONOMOUS,
            "MAPPING": control_pb2.ROBOT_MODE_MAPPING,
        }
        mode_val = mode_map.get(req.mode.upper(), control_pb2.ROBOT_MODE_UNSPECIFIED)
        request = control_pb2.SetModeRequest(mode=mode_val)
        result = _grpc_call("control", "SetMode", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/estop")
def grpc_control_estop():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.EmergencyStopRequest(hard_stop=True)
        result = _grpc_call("control", "EmergencyStop", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/clear_estop")
def grpc_control_clear_estop():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.ClearEmergencyStopRequest()
        result = _grpc_call("control", "ClearEmergencyStop", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/task/start")
def grpc_control_start_task(req: GrpcStartTaskRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        import common_pb2
        from google.protobuf import json_format as _jf

        type_map = {
            "NAVIGATION": common_pb2.TASK_TYPE_NAVIGATION,
            "MAPPING": common_pb2.TASK_TYPE_MAPPING,
            "FOLLOW_PATH": common_pb2.TASK_TYPE_FOLLOW_PATH,
            "SEMANTIC_NAV": common_pb2.TASK_TYPE_SEMANTIC_NAV,
            "FOLLOW_PERSON": common_pb2.TASK_TYPE_FOLLOW_PERSON,
            "INSPECTION": common_pb2.TASK_TYPE_INSPECTION,
            "RETURN_HOME": common_pb2.TASK_TYPE_RETURN_HOME,
        }
        task_type_val = type_map.get(req.task_type.upper(), common_pb2.TASK_TYPE_UNSPECIFIED)
        request = control_pb2.StartTaskRequest(
            task_type=task_type_val,
            params_json=req.params_json,
        )
        result = _grpc_call("control", "StartTask", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/task/cancel")
def grpc_control_cancel_task(req: GrpcCancelTaskRequest):
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.CancelTaskRequest(task_id=req.task_id)
        result = _grpc_call("control", "CancelTask", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/control/waypoints")
def grpc_control_get_waypoints():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.GetActiveWaypointsRequest()
        result = _grpc_call("control", "GetActiveWaypoints", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


@app.post("/api/grpc/control/waypoints/clear")
def grpc_control_clear_waypoints():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        request = control_pb2.ClearWaypointsRequest()
        result = _grpc_call("control", "ClearWaypoints", request)
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


# ═════════════════════════════════════════════════════════════
#  gRPC: TelemetryService
# ═════════════════════════════════════════════════════════════
@app.get("/api/grpc/telemetry/fast")
def grpc_telemetry_fast(count: int = 5):
    """Get a few FastState frames from the stream."""
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        stub = _get_stub("telemetry")
        if stub is None:
            return {"ok": False, "error": "Cannot create telemetry stub"}
        request = telemetry_pb2.FastStateRequest(desired_hz=10.0)
        frames = []
        stream = stub.StreamFastState(request, timeout=GRPC_TIMEOUT)
        for i, msg in enumerate(stream):
            if i >= count:
                stream.cancel()
                break
            frames.append(
                MessageToDict(msg, preserving_proto_field_name=True, always_print_fields_with_no_presence=True)
            )
        return {"ok": True, "frames": frames, "count": len(frames)}
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/telemetry/slow")
def grpc_telemetry_slow():
    """Get one SlowState frame from the stream."""
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        stub = _get_stub("telemetry")
        if stub is None:
            return {"ok": False, "error": "Cannot create telemetry stub"}
        request = telemetry_pb2.SlowStateRequest()
        stream = stub.StreamSlowState(request, timeout=GRPC_TIMEOUT)
        for msg in stream:
            result = MessageToDict(msg, preserving_proto_field_name=True, always_print_fields_with_no_presence=True)
            stream.cancel()
            return {"ok": True, "state": result}
        return {"ok": False, "error": "No SlowState received"}
    except Exception as e:
        return _grpc_err(e)


@app.get("/api/grpc/telemetry/events")
def grpc_telemetry_events(count: int = 10):
    """Get recent events from the event stream."""
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        stub = _get_stub("telemetry")
        if stub is None:
            return {"ok": False, "error": "Cannot create telemetry stub"}
        request = telemetry_pb2.EventStreamRequest()
        events = []
        stream = stub.StreamEvents(request, timeout=GRPC_TIMEOUT)
        for i, msg in enumerate(stream):
            if i >= count:
                stream.cancel()
                break
            events.append(
                MessageToDict(msg, preserving_proto_field_name=True, always_print_fields_with_no_presence=True)
            )
        return {"ok": True, "events": events, "count": len(events)}
    except Exception as e:
        return _grpc_err(e)


# ═════════════════════════════════════════════════════════════
#  gRPC: DataService
# ═════════════════════════════════════════════════════════════
@app.get("/api/grpc/data/resources")
def grpc_data_list_resources():
    if not GRPC_AVAILABLE:
        return {"ok": False, "error": "gRPC not available"}
    try:
        result = _grpc_call("data", "ListResources")
        return _grpc_ok(result)
    except Exception as e:
        return _grpc_err(e)


# ═════════════════════════════════════════════════════════════
#  WebSocket: Live Telemetry via gRPC streaming
# ═════════════════════════════════════════════════════════════
@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    """
    Stream FastState + SlowState to browser via WebSocket.
    FastState arrives at ~20Hz, SlowState at ~1Hz.
    We read them in separate threads and push to the WS.
    """
    await ws.accept()

    if not GRPC_AVAILABLE:
        await ws.send_json({"type": "error", "error": "gRPC not available"})
        await ws.close()
        return

    stop_event = threading.Event()

    async def _send_loop():
        """Main loop: pull from queues and send to websocket."""
        import queue

        fast_q: queue.Queue = queue.Queue(maxsize=50)
        slow_q: queue.Queue = queue.Queue(maxsize=10)

        def _stream_fast():
            try:
                stub = _get_stub("telemetry")
                if stub is None:
                    fast_q.put({"_error": "Cannot create telemetry stub"})
                    return
                request = telemetry_pb2.FastStateRequest(desired_hz=10.0)
                stream = stub.StreamFastState(request, timeout=600)
                for msg in stream:
                    if stop_event.is_set():
                        stream.cancel()
                        break
                    data = MessageToDict(
                        msg, preserving_proto_field_name=True, always_print_fields_with_no_presence=True
                    )
                    try:
                        fast_q.put_nowait(data)
                    except queue.Full:
                        try:
                            fast_q.get_nowait()
                        except queue.Empty:
                            pass
                        fast_q.put_nowait(data)
            except Exception as e:
                if not stop_event.is_set():
                    try:
                        fast_q.put_nowait({"_error": f"FastState stream failed: {e}"})
                    except queue.Full:
                        pass

        def _stream_slow():
            try:
                stub = _get_stub("telemetry")
                if stub is None:
                    slow_q.put({"_error": "Cannot create telemetry stub"})
                    return
                request = telemetry_pb2.SlowStateRequest()
                stream = stub.StreamSlowState(request, timeout=600)
                for msg in stream:
                    if stop_event.is_set():
                        stream.cancel()
                        break
                    data = MessageToDict(
                        msg, preserving_proto_field_name=True, always_print_fields_with_no_presence=True
                    )
                    try:
                        slow_q.put_nowait(data)
                    except queue.Full:
                        try:
                            slow_q.get_nowait()
                        except queue.Empty:
                            pass
                        slow_q.put_nowait(data)
            except Exception as e:
                if not stop_event.is_set():
                    try:
                        slow_q.put_nowait({"_error": f"SlowState stream failed: {e}"})
                    except queue.Full:
                        pass

        fast_thread = threading.Thread(target=_stream_fast, daemon=True)
        slow_thread = threading.Thread(target=_stream_slow, daemon=True)
        fast_thread.start()
        slow_thread.start()

        try:
            while not stop_event.is_set():
                sent = False
                # Drain fast queue
                try:
                    while True:
                        data = fast_q.get_nowait()
                        if "_error" in data:
                            await ws.send_json(
                                {
                                    "type": "error",
                                    "source": "fast_state",
                                    "error": data["_error"],
                                    "timestamp": time.time(),
                                }
                            )
                        else:
                            await ws.send_json({"type": "fast_state", "data": data, "timestamp": time.time()})
                        sent = True
                except Exception:
                    pass
                # Drain slow queue
                try:
                    while True:
                        data = slow_q.get_nowait()
                        if "_error" in data:
                            await ws.send_json(
                                {
                                    "type": "error",
                                    "source": "slow_state",
                                    "error": data["_error"],
                                    "timestamp": time.time(),
                                }
                            )
                        else:
                            await ws.send_json({"type": "slow_state", "data": data, "timestamp": time.time()})
                        sent = True
                except Exception:
                    pass
                if not sent:
                    await asyncio.sleep(0.05)
        finally:
            stop_event.set()

    try:
        await _send_loop()
    except WebSocketDisconnect:
        stop_event.set()
    except Exception:
        stop_event.set()


# ═════════════════════════════════════════════════════════════
#  Main
# ═════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print()
    print("  LingTu Dashboard Server")
    print(f"  Robot:  {ROBOT_HOST}")
    print(f"  gRPC:   {ROBOT_HOST}:{GRPC_PORT} ({'enabled' if GRPC_AVAILABLE else 'DISABLED - install grpcio'})")
    print("  Open:   http://localhost:8066")
    print()
    uvicorn.run(app, host="0.0.0.0", port=8066, log_level="info")
