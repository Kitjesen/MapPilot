#!/usr/bin/env python3
"""LingTu Manager 鈥?product-grade control plane.

Always-on process that manages the robot's operation mode.
Exposes REST API + Web UI for customers to control the robot
from any browser 鈥?no SSH needed.

Endpoints:
  GET  /                    鈫?Web UI
  GET  /api/status          鈫?system status (mode, SLAM hz, battery, etc.)
  POST /api/mode            鈫?switch mode: {"mode": "map|nav|explore|idle"}
  POST /api/map/save        鈫?save map: {"name": "lab_01"}
  POST /api/map/use         鈫?activate map: {"name": "lab_01"}
  GET  /api/map/list        鈫?list available maps
  POST /api/navigate        鈫?send goal: {"x": 5.0, "y": 3.0} or {"instruction": "go to the door"}
  POST /api/stop            鈫?emergency stop
  POST /api/rerun           鈫?toggle rerun: {"enabled": true}
  GET  /api/stream          鈫?SSE telemetry stream

Boot:
  sudo systemctl enable lingtu-manager
  # Starts on boot, manages everything else
"""

import asyncio
import json
import logging
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, JSONResponse, StreamingResponse

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("lingtu-manager")

app = FastAPI(title="LingTu Robot Manager", version="1.0.0")

# 鈹€鈹€ Config 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

NAV_DIR = os.environ.get("NAV_DIR", "/home/sunrise/data/inovxio/lingtu")
MAP_DIR = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/inovxio/data/maps"))
MANAGER_PORT = int(os.environ.get("LINGTU_PORT", "5050"))
MANAGER_HOST = os.environ.get("LINGTU_MANAGER_HOST", "127.0.0.1")
GATEWAY_URL = os.environ.get("LINGTU_GATEWAY_URL", "http://127.0.0.1:5051").rstrip("/")
MANAGER_ALLOWED_ORIGINS = [
    origin.strip()
    for origin in os.environ.get(
        "LINGTU_MANAGER_ORIGINS",
        f"http://127.0.0.1:{MANAGER_PORT},http://localhost:{MANAGER_PORT}",
    ).split(",")
    if origin.strip()
]
_ALLOWED_PROFILES = {"map", "nav", "explore", "idle", "stub", "dev", "sim", "sim_nav"}
_MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

app.add_middleware(
    CORSMiddleware,
    allow_origins=MANAGER_ALLOWED_ORIGINS,
    allow_methods=["GET", "POST"],
    allow_headers=["Authorization", "Content-Type", "X-LingTu-Token"],
)


def _safe_map_path(name: str) -> Path:
    """Resolve a user map name under MAP_DIR and reject traversal/shell payloads."""
    if not _MAP_NAME_RE.fullmatch(name):
        raise ValueError("Map name must be 1-64 chars: letters, numbers, _, ., -")
    root = Path(MAP_DIR).expanduser().resolve()
    path = (root / name).resolve()
    if path == root or root not in path.parents:
        raise ValueError("Map path escapes map directory")
    return path


def _gateway_url(path: str) -> str:
    """Build a URL for the LingTu Gateway service."""
    return f"{GATEWAY_URL}/{path.lstrip('/')}"

# 鈹€鈹€ State 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

class RobotState:
    mode: str = "idle"          # idle | map | nav | explore
    slam_profile: str = "none"  # none | fastlio2 | localizer
    active_map: str = ""
    lingtu_pid: Optional[int] = None
    last_error: str = ""

state = RobotState()


# 鈹€鈹€ Service helpers 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def _systemctl(action: str, service: str, timeout: int = 15) -> bool:
    try:
        result = subprocess.run(
            ["sudo", "systemctl", action, service],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
        return result.returncode == 0
    except Exception as e:
        logger.error("systemctl %s %s: %s", action, service, e)
        return False


def _is_active(service: str) -> bool:
    try:
        result = subprocess.run(
            ["systemctl", "is-active", service],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=5,
        )
        return result.stdout.strip() == "active"
    except Exception:
        return False


def _start_lingtu(profile: str, extra_args: str = "") -> bool:
    """Start lingtu navigation system as background process."""
    if profile not in _ALLOWED_PROFILES:
        state.last_error = f"Invalid profile: {profile}"
        return False
    if state.lingtu_pid and _pid_alive(state.lingtu_pid):
        _stop_lingtu()

    args = ["python3", "lingtu.py", profile, "--no-repl"]
    if extra_args:
        args.extend(shlex.split(extra_args))
    try:
        proc = subprocess.Popen(
            args,
            cwd=NAV_DIR,
            stdout=open("/tmp/lingtu_nav.log", "a"),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        state.lingtu_pid = proc.pid
        logger.info("Started lingtu %s (PID %d)", profile, proc.pid)
        return True
    except Exception as e:
        state.last_error = str(e)
        logger.error("Failed to start lingtu: %s", e)
        return False


def _stop_lingtu() -> bool:
    if state.lingtu_pid and _pid_alive(state.lingtu_pid):
        try:
            os.killpg(os.getpgid(state.lingtu_pid), 15)  # SIGTERM
            time.sleep(2)
            if _pid_alive(state.lingtu_pid):
                os.killpg(os.getpgid(state.lingtu_pid), 9)  # SIGKILL
            logger.info("Stopped lingtu (PID %d)", state.lingtu_pid)
        except Exception as e:
            logger.warning("Stop lingtu: %s", e)
    state.lingtu_pid = None
    return True


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except (OSError, ProcessLookupError):
        return False


# 鈹€鈹€ Mode switching 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def _switch_to_idle():
    _stop_lingtu()
    _stop_rerun()
    _systemctl("stop", "slam")
    _systemctl("stop", "slam_pgo")
    state.mode = "idle"
    state.slam_profile = "none"


def _start_rerun():
    """Start the Gateway-backed Rerun viewer for 3D visualization."""
    import subprocess as _sp
    _sp.Popen(
        [
            "python3",
            "-u",
            "scripts/visualization/rerun_gateway_live.py",
            "--gateway-url",
            GATEWAY_URL,
        ],
        cwd=NAV_DIR,
        stdout=open("/tmp/rerun_live.log", "a"),
        stderr=_sp.STDOUT,
        preexec_fn=os.setsid,
    )
    logger.info("Rerun started")


def _stop_rerun():
    import subprocess as _sp
    _sp.run(["pkill", "-9", "-f", "rerun_live"], capture_output=True)
    _sp.run(["pkill", "-9", "-f", "rerun_gateway_live"], capture_output=True)
    _sp.run(["fuser", "-k", "9090/tcp"], capture_output=True)
    _sp.run(["fuser", "-k", "9877/tcp"], capture_output=True)


def _switch_to_map():
    _stop_lingtu()
    _stop_rerun()
    _systemctl("start", "slam")
    _systemctl("start", "slam_pgo")
    _start_rerun()
    state.mode = "map"
    state.slam_profile = "fastlio2"


def _switch_to_nav():
    _stop_lingtu()
    _stop_rerun()
    active_tomo = os.path.join(MAP_DIR, "active", "tomogram.pickle")
    if not os.path.exists(active_tomo):
        state.last_error = "No active map. Build a map first."
        return False
    _systemctl("stop", "slam_pgo")
    _systemctl("start", "slam")
    _start_lingtu("nav")
    _start_rerun()
    state.mode = "nav"
    state.slam_profile = "localizer"
    return True


def _switch_to_explore():
    _stop_lingtu()
    _stop_rerun()
    _systemctl("start", "slam")
    _systemctl("stop", "slam_pgo")
    _start_lingtu("explore")
    _start_rerun()
    state.mode = "explore"
    state.slam_profile = "fastlio2"
    return True


# 鈹€鈹€ Map management 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

def _list_maps() -> list:
    maps = []
    if not os.path.isdir(MAP_DIR):
        return maps
    active_name = ""
    active_link = os.path.join(MAP_DIR, "active")
    if os.path.islink(active_link):
        active_name = os.path.basename(os.readlink(active_link))
    for d in sorted(os.listdir(MAP_DIR)):
        dp = os.path.join(MAP_DIR, d)
        if not os.path.isdir(dp) or d in ("active", "_staging"):
            continue
        maps.append({
            "name": d,
            "active": d == active_name,
            "has_pcd": os.path.exists(os.path.join(dp, "map.pcd")),
            "has_tomogram": os.path.exists(os.path.join(dp, "tomogram.pickle")),
        })
    return maps


def _save_map(name: str) -> dict:
    try:
        map_dir_path = _safe_map_path(name)
    except ValueError as e:
        return {"success": False, "error": str(e)}
    map_dir_path.mkdir(parents=True, exist_ok=True)
    pcd_path = map_dir_path / "map.pcd"
    try:
        result = _save_nav_map_snapshot(pcd_path)
        if result.get("success") is False:
            return {"success": False, "error": str(result.get("message", "map save failed"))}
        return {"success": True, "pcd": str(pcd_path)}
    except Exception as e:
        return {"success": False, "error": str(e)}


def _save_nav_map_snapshot(pcd_path: Path) -> dict:
    src_dir = Path(NAV_DIR).expanduser().resolve() / "src"
    if src_dir.is_dir() and str(src_dir) not in sys.path:
        sys.path.insert(0, str(src_dir))

    from runtime.map_save import save_nav_map_with_adapter
    from runtime.plugin_seed import seed_registered_plugins
    from lingtu.plugin_seed import install_builtin_plugin_catalog

    install_builtin_plugin_catalog()
    seed_registered_plugins(groups=("map_save_adapter",), reload_loaded=False)
    return save_nav_map_with_adapter(None, pcd_path, timeout_sec=30.0)


def _use_map(name: str) -> dict:
    try:
        map_path = _safe_map_path(name)
    except ValueError as e:
        return {"success": False, "error": str(e)}
    if not map_path.is_dir():
        return {"success": False, "error": f"Map not found: {name}"}
    active_link = Path(MAP_DIR).expanduser().resolve() / "active"
    if active_link.is_symlink() or active_link.exists():
        active_link.unlink()
    active_link.symlink_to(map_path)
    state.active_map = name
    return {"success": True, "active": name}


# 鈹€鈹€ API Routes 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

@app.get("/api/status")
async def api_status():
    return {
        "mode": state.mode,
        "slam_profile": state.slam_profile,
        "active_map": state.active_map,
        "services": {
            "slam": _is_active("slam"),
            "slam_pgo": _is_active("slam_pgo"),
            "lidar": _is_active("lidar"),
            "camera": _is_active("camera"),
            "brainstem": _is_active("brainstem"),
        },
        "lingtu_running": state.lingtu_pid is not None and _pid_alive(state.lingtu_pid),
        "last_error": state.last_error,
    }


@app.post("/api/mode")
async def api_mode(request: Request):
    body = await request.json()
    mode = body.get("mode", "")
    state.last_error = ""

    if mode == "idle":
        _switch_to_idle()
    elif mode == "map":
        _switch_to_map()
    elif mode == "nav":
        if not _switch_to_nav():
            return JSONResponse({"success": False, "error": state.last_error}, status_code=400)
    elif mode == "explore":
        _switch_to_explore()
    else:
        return JSONResponse({"success": False, "error": f"Unknown mode: {mode}"}, status_code=400)

    return {"success": True, "mode": state.mode}


@app.get("/api/map/list")
async def api_map_list():
    return {"maps": _list_maps()}


@app.post("/api/map/save")
async def api_map_save(request: Request):
    body = await request.json()
    name = body.get("name", "")
    if not name:
        return JSONResponse({"success": False, "error": "Missing map name"}, status_code=400)
    return _save_map(name)


@app.post("/api/map/use")
async def api_map_use(request: Request):
    body = await request.json()
    name = body.get("name", "")
    if not name:
        return JSONResponse({"success": False, "error": "Missing map name"}, status_code=400)
    return _use_map(name)


@app.post("/api/stop")
async def api_stop():
    """Emergency stop 鈥?halt all motion."""
    # Send stop to brainstem
    try:
        import grpc
        from brainstem_api import cms_pb2_grpc
        from google.protobuf.empty_pb2 import Empty
        channel = grpc.insecure_channel("127.0.0.1:13145")
        stub = cms_pb2_grpc.RobotControlStub(channel)
        stub.SitDown(Empty(), timeout=3)
        channel.close()
    except Exception as e:
        logger.warning("Stop via brainstem failed: %s", e)
    return {"success": True, "action": "emergency_stop"}


@app.post("/api/navigate")
async def api_navigate(request: Request):
    body = await request.json()
    if state.mode != "nav":
        return JSONResponse({"success": False, "error": "Not in nav mode"}, status_code=400)
    # Forward to lingtu via its Gateway API
    try:
        import httpx
        async with httpx.AsyncClient() as client:
            resp = await client.post(_gateway_url("/api/v1/goal"), json=body, timeout=5)
            return resp.json()
    except Exception as e:
        return JSONResponse({"success": False, "error": str(e)}, status_code=500)


@app.get("/api/stream")
async def api_stream():
    """SSE telemetry stream."""
    async def generate():
        while True:
            data = {
                "mode": state.mode,
                "slam_active": _is_active("slam"),
                "timestamp": time.time(),
            }
            yield f"data: {json.dumps(data)}\n\n"
            await asyncio.sleep(1)
    return StreamingResponse(generate(), media_type="text/event-stream")


@app.get("/api/camera/snapshot")
async def api_camera_snapshot():
    """Proxy one JPEG frame from the LingTu Gateway camera endpoint."""
    try:
        import httpx
        from fastapi.responses import Response

        async with httpx.AsyncClient() as client:
            resp = await client.get(_gateway_url("/api/v1/camera/snapshot"), timeout=5)
        if resp.status_code == 200:
            return Response(
                content=resp.content,
                media_type=resp.headers.get("content-type", "image/jpeg"),
            )
        try:
            detail = resp.json()
        except Exception:
            detail = {"error": resp.text or "Gateway camera snapshot unavailable"}
        return JSONResponse(detail, status_code=resp.status_code)
    except Exception as e:
        return JSONResponse({"error": str(e)}, status_code=500)


@app.get("/api/rerun/status")
async def api_rerun_status():
    """Check if Rerun web viewer is running."""
    rerun_running = False
    try:
        import socket
        s = socket.socket()
        s.settimeout(1)
        s.connect(("127.0.0.1", 9090))
        s.close()
        rerun_running = True
    except Exception:
        pass
    return {"running": rerun_running, "url": "http://localhost:9090" if rerun_running else None}


# 鈹€鈹€ Web UI 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

@app.get("/", response_class=HTMLResponse)
async def web_ui():
    return WEB_UI_HTML


WEB_UI_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>LingTu 鎺у埗闈㈡澘</title>
<style>
:root{
  --bg:#1a1a2e;--bg2:#121727;--panel:#1c2336;
  --line:rgba(255,255,255,.08);--text:#eef3ff;--muted:#98a3bf;
  --accent:#ffb703;--cyan:#4cc9f0;--green:#22c55e;--red:#ff4d4f;--orange:#ff7a18;
  --shadow:0 16px 40px rgba(0,0,0,.28);--radius:18px;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{
  color:var(--text);
  font:15px/1.45 "Bahnschrift","Segoe UI","PingFang SC","Microsoft YaHei",sans-serif;
  background:radial-gradient(900px 500px at 100% 0,rgba(255,183,3,.08),transparent 55%),
    radial-gradient(700px 400px at 0 100%,rgba(76,201,240,.07),transparent 55%),
    linear-gradient(180deg,#1a1a2e,#151a29 55%,#101521);
  min-height:100vh;
}
.shell{max-width:1360px;margin:0 auto;padding:14px}
/* Topbar */
.topbar{
  position:sticky;top:0;z-index:50;display:flex;align-items:center;justify-content:space-between;gap:14px;
  margin-bottom:14px;padding:12px 14px;border:1px solid var(--line);border-radius:20px;
  background:rgba(18,23,39,.86);backdrop-filter:blur(10px);box-shadow:var(--shadow);
}
.brand{display:flex;align-items:center;gap:12px}
.brand-mark{
  width:48px;height:48px;display:grid;place-items:center;border-radius:14px;
  font-weight:800;letter-spacing:.08em;color:#111;
  background:linear-gradient(135deg,var(--accent),var(--orange));
  box-shadow:0 10px 24px rgba(255,183,3,.18);
}
.eyebrow{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.14em}
h1{font-size:22px;line-height:1.05}
.top-actions{display:flex;align-items:center;gap:10px;flex-wrap:wrap;justify-content:flex-end}
.live-chip{
  display:inline-flex;align-items:center;gap:8px;padding:10px 12px;border-radius:999px;
  background:rgba(255,255,255,.04);border:1px solid var(--line);color:var(--muted);white-space:nowrap;
}
.dot{width:10px;height:10px;border-radius:50%;background:#666;box-shadow:0 0 0 3px rgba(255,255,255,.05);flex-shrink:0}
.dot.on{background:var(--green);box-shadow:0 0 10px rgba(34,197,94,.75),0 0 0 3px rgba(34,197,94,.16)}
.dot.off{background:var(--red);box-shadow:0 0 10px rgba(255,77,79,.75),0 0 0 3px rgba(255,77,79,.16)}
/* E-Stop */
.estop-wrap{display:flex;flex-direction:column;align-items:center;padding:16px 0}
.estop{
  width:100px;height:100px;border-radius:50%;border:none;cursor:pointer;position:relative;
  background:#801020;color:#fff;font:800 20px inherit;letter-spacing:.1em;
  box-shadow:0 8px 0 #500810,0 15px 30px rgba(233,69,96,.4);transition:.1s;
}
.estop:active{transform:translateY(4px);box-shadow:0 4px 0 #500810,0 8px 15px rgba(233,69,96,.4)}
.estop-ring{
  position:absolute;inset:-12px;border:2px solid rgba(255,77,79,.4);border-radius:50%;
  animation:pulse 2s infinite;pointer-events:none;
}
@keyframes pulse{0%{transform:scale(1);opacity:.5}100%{transform:scale(1.2);opacity:0}}
.estop-label{margin-top:8px;font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.1em}
/* Buttons */
.btn{
  appearance:none;border:0;cursor:pointer;padding:12px 16px;border-radius:14px;
  color:var(--text);background:linear-gradient(180deg,#2b3550,#20273b);
  border:1px solid rgba(255,255,255,.08);font:600 15px inherit;transition:.18s;
}
.btn:hover{transform:translateY(-1px);box-shadow:0 10px 22px rgba(0,0,0,.24)}
.btn:active{transform:translateY(0)}
.btn:disabled{opacity:.45;cursor:not-allowed;transform:none}
.btn.success{background:linear-gradient(180deg,#169f55,#137c45)}
.btn.warn{background:linear-gradient(180deg,#ffb703,#d58800);color:#161616}
.btn.secondary{background:linear-gradient(180deg,#25304a,#1b2234)}
/* Layout */
.layout{display:grid;grid-template-columns:1.2fr .8fr;gap:14px}
.panel{
  background:linear-gradient(180deg,rgba(35,43,67,.92),rgba(24,29,44,.94));
  border:1px solid var(--line);border-radius:var(--radius);box-shadow:var(--shadow);padding:16px;min-width:0;
}
.span2{grid-column:1/-1}
.panel-head{display:flex;justify-content:space-between;align-items:center;gap:10px;margin-bottom:14px}
.panel-title{font-size:18px;font-weight:800}
.panel-sub{font-size:13px;color:var(--muted)}
/* Mode buttons */
.mode-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-bottom:16px}
.mode-btn{
  position:relative;overflow:hidden;text-align:left;min-height:90px;padding:16px;border-radius:18px;
  background:linear-gradient(180deg,rgba(255,255,255,.04),rgba(255,255,255,.015)),linear-gradient(180deg,#27314a,#1c2336);
  border:1px solid rgba(255,255,255,.08);cursor:pointer;
}
.mode-btn .t{display:block;font-size:18px;font-weight:800;margin-bottom:4px}
.mode-btn .s{display:block;font-size:13px;color:var(--muted)}
.mode-btn.active{
  border-color:rgba(255,183,3,.52);
  box-shadow:0 0 0 1px rgba(255,183,3,.22),0 14px 34px rgba(255,183,3,.12);
  background:linear-gradient(180deg,rgba(255,183,3,.14),rgba(255,183,3,.03)),linear-gradient(180deg,#2a3651,#20283c);
}
.mode-btn.active::after{
  content:"";position:absolute;inset:-20px;border:2px solid rgba(255,183,3,.18);border-radius:26px;animation:pulse 1.8s infinite;
}
/* Two-col sub areas */
.two-col{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.subcard{padding:14px;border-radius:16px;background:rgba(255,255,255,.03);border:1px solid var(--line)}
.subhead{font-size:13px;color:var(--muted);margin-bottom:10px}
.input-row{display:flex;gap:10px}
input{
  width:100%;padding:12px 14px;border-radius:14px;background:#111727;color:var(--text);
  border:1px solid rgba(255,255,255,.09);outline:none;font:500 15px inherit;
}
input:focus{border-color:rgba(76,201,240,.45);box-shadow:0 0 0 3px rgba(76,201,240,.12)}
.hint{margin-top:8px;font-size:12px;color:var(--muted)}
/* Stats */
.stats{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:14px}
.stat{padding:14px;border-radius:16px;background:rgba(255,255,255,.03);border:1px solid var(--line)}
.stat .k{font-size:12px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
.stat .v{margin-top:6px;font-size:18px;font-weight:800;word-break:break-word}
/* Services */
.services{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:14px}
.service{
  display:flex;align-items:center;justify-content:space-between;gap:10px;
  padding:12px;border-radius:14px;background:rgba(255,255,255,.03);border:1px solid var(--line);
}
.service .left{display:flex;align-items:center;gap:10px}
.service .name{font-weight:700}
.service .state{font-size:12px;color:var(--muted)}
.error-box{padding:12px 14px;border-radius:14px;background:rgba(255,255,255,.03);border:1px solid var(--line);color:#ffdede;word-break:break-word}
.error-box.clean{color:var(--muted)}
/* Maps */
.maps{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.map-card{
  padding:14px;border-radius:16px;
  background:linear-gradient(180deg,rgba(255,255,255,.04),rgba(255,255,255,.02));
  border:1px solid var(--line);transition:.18s;
}
.map-card:hover{transform:translateY(-1px);box-shadow:0 10px 24px rgba(0,0,0,.2)}
.map-card.active{border-color:rgba(34,197,94,.68);box-shadow:0 0 0 1px rgba(34,197,94,.2),0 14px 28px rgba(34,197,94,.08)}
.map-head{display:flex;justify-content:space-between;gap:10px;align-items:flex-start;margin-bottom:10px}
.map-name{font-size:17px;font-weight:800;word-break:break-word}
.tag{
  display:inline-flex;padding:4px 8px;border-radius:999px;font-size:11px;font-weight:800;letter-spacing:.05em;
  background:rgba(255,255,255,.06);color:var(--muted);border:1px solid var(--line);white-space:nowrap;
}
.tag.active{color:#d7ffe6;background:rgba(34,197,94,.13);border-color:rgba(34,197,94,.35)}
.badges{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:12px}
.badge{font-size:12px;padding:6px 8px;border-radius:10px;background:rgba(255,255,255,.04);border:1px solid var(--line);color:var(--muted)}
.badge.ok{color:#d7ffe6;border-color:rgba(34,197,94,.24);background:rgba(34,197,94,.1)}
.empty{padding:18px;border-radius:16px;text-align:center;color:var(--muted);border:1px dashed rgba(255,255,255,.16);background:rgba(255,255,255,.02)}
/* Toast */
.toast-wrap{position:fixed;right:14px;bottom:14px;z-index:80;display:flex;flex-direction:column;gap:10px;max-width:min(420px,calc(100vw - 28px))}
.toast{padding:12px 14px;border-radius:14px;border:1px solid rgba(255,255,255,.08);background:rgba(16,21,33,.96);box-shadow:var(--shadow);animation:tin .2s ease-out}
.toast.info{border-left:4px solid var(--cyan)}
.toast.success{border-left:4px solid var(--green)}
.toast.warn{border-left:4px solid var(--accent)}
.toast.error{border-left:4px solid var(--red)}
@keyframes tin{from{transform:translateY(8px);opacity:0}to{transform:translateY(0);opacity:1}}
/* Responsive */
@media(max-width:980px){.layout{grid-template-columns:1fr}.span2{grid-column:auto}.mode-grid{grid-template-columns:repeat(2,1fr)}}
@media(max-width:720px){.topbar{flex-direction:column;align-items:stretch}.top-actions{justify-content:stretch}.top-actions>*{flex:1 1 auto}.two-col,.stats,.services{grid-template-columns:1fr}}
@media(max-width:480px){.shell{padding:10px}h1{font-size:20px}.mode-btn{min-height:80px}.mode-btn .t{font-size:16px}}
</style>
</head>
<body>
<div class="shell">
  <header class="topbar">
    <div class="brand">
      <div class="brand-mark">LT</div>
      <div><div class="eyebrow">LingTu Quadruped Control</div><h1>鐏甸€旀帶鍒堕潰鏉?/h1></div>
    </div>
    <div class="top-actions">
      <div class="live-chip"><span id="sseDot" class="dot off"></span><span id="sseText">閬ユ祴閲嶈繛涓?/span></div>
      <div class="live-chip"><span id="runDot" class="dot off"></span><span id="runText">绂荤嚎</span></div>
    </div>
  </header>

  <!-- E-Stop -->
  <div class="estop-wrap">
    <button class="estop" id="stopBtn">STOP<span class="estop-ring"></span></button>
    <div class="estop-label">Emergency Brake</div>
  </div>

  <main class="layout">
    <!-- Left Column: Controls -->
    <section class="panel">
      <div class="panel-head"><div><div class="panel-title">妯″紡鍒囨崲</div><div class="panel-sub">Mode Selection</div></div></div>
      <div class="mode-grid">
        <button class="btn mode-btn" data-mode="map"><span class="t">寤哄浘</span><span class="s">Map / SLAM</span></button>
        <button class="btn mode-btn" data-mode="nav"><span class="t">瀵艰埅</span><span class="s">Navigate</span></button>
        <button class="btn mode-btn" data-mode="explore"><span class="t">鎺㈢储</span><span class="s">Explore</span></button>
        <button class="btn mode-btn" data-mode="idle"><span class="t">寰呮満</span><span class="s">Idle</span></button>
      </div>
      <div class="two-col">
        <div class="subcard">
          <div class="subhead">瀵艰埅鐩爣</div>
          <div class="input-row"><input id="navInput" placeholder="5 3 鎴?鍘婚棬鍙?><button id="navBtn" class="btn warn">鍙戦€?/button></div>
          <div class="hint">鑷姩璇嗗埆鍧愭爣鎴栬涔夋寚浠?/div>
        </div>
        <div class="subcard">
          <div class="subhead">淇濆瓨鍦板浘</div>
          <div class="input-row"><input id="mapNameInput" placeholder="鍦板浘鍚嶇О"><button id="saveMapBtn" class="btn success">淇濆瓨</button></div>
          <div class="hint">淇濆瓨褰撳墠 SLAM 鍦板浘</div>
        </div>
      </div>
    </section>

    <!-- Right Column: Status -->
    <section class="panel">
      <div class="panel-head"><div><div class="panel-title">鐘舵€佹€昏</div><div class="panel-sub">Real-time Telemetry</div></div></div>
      <div class="stats">
        <div class="stat"><div class="k">褰撳墠妯″紡</div><div class="v" id="modeVal">--</div></div>
        <div class="stat"><div class="k">SLAM</div><div class="v" id="slamVal">--</div></div>
        <div class="stat"><div class="k">褰撳墠鍦板浘</div><div class="v" id="mapVal">--</div></div>
        <div class="stat"><div class="k">閬ユ祴鏃堕棿</div><div class="v" id="timeVal">--</div></div>
      </div>
      <div id="svcGrid" class="services"></div>
      <div class="subhead">鏈€杩戦敊璇?/div>
      <div id="errBox" class="error-box clean">鏃?/div>
    </section>

    <!-- Camera + 3D View -->
    <section class="panel">
      <div class="panel-head"><div><div class="panel-title">鐩告満</div><div class="panel-sub">Camera Feed</div></div>
        <button class="btn secondary" onclick="refreshCam()">鍒锋柊</button>
      </div>
      <div style="text-align:center">
        <img id="camImg" src="/api/camera/snapshot" style="max-width:100%;border-radius:12px;background:#111;min-height:200px" onerror="this.style.opacity=0.3" alt="camera">
      </div>
    </section>

    <section class="panel">
      <div class="panel-head"><div><div class="panel-title">3D 瑙嗗浘</div><div class="panel-sub">Rerun Point Cloud</div></div>
        <span id="rerunStatus" style="font-size:12px;color:var(--muted)">妫€娴嬩腑...</span>
      </div>
      <div id="rerunWrap" style="border-radius:12px;overflow:hidden;background:#111;min-height:200px;display:flex;align-items:center;justify-content:center">
        <span style="color:var(--muted)">Rerun 鏈繍琛?/span>
      </div>
    </section>

    <!-- Full Width: Maps -->
    <section class="panel span2">
      <div class="panel-head">
        <div><div class="panel-title">鍦板浘鍒楄〃</div><div class="panel-sub">娲诲姩鍦板浘缁胯壊楂樹寒</div></div>
        <button id="refreshMaps" class="btn secondary">鍒锋柊</button>
      </div>
      <div id="mapsGrid" class="maps"><div class="empty">鍔犺浇涓?..</div></div>
    </section>
  </main>
</div>

<div id="toasts" class="toast-wrap"></div>

<script>
const $=s=>document.querySelector(s),$$=s=>[...document.querySelectorAll(s)];
const S={status:null,maps:[],sse:null,connected:false,ever:false,ttime:"--"};

function esc(v){return String(v??"").replace(/[&<>"']/g,m=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"})[m])}

function toast(msg,type="info",ttl=2600){
  const w=$("#toasts"),n=document.createElement("div");
  n.className="toast "+type;n.textContent=msg;w.appendChild(n);
  setTimeout(()=>{n.style.opacity="0";n.style.transform="translateY(6px)";setTimeout(()=>n.remove(),180)},ttl);
}

async function api(url,method="GET",body){
  const r=await fetch(url,{method,headers:body?{"Content-Type":"application/json"}:{},body:body?JSON.stringify(body):undefined});
  const t=await r.text();let d={};if(t)try{d=JSON.parse(t)}catch{d={detail:t}}
  if(!r.ok)throw new Error(d.detail||d.error||r.status+" "+r.statusText);return d;
}

function setDot(el,on){el.classList.remove("on","off");el.classList.add(on?"on":"off")}
function setMode(mode){
  $("#modeVal").textContent=mode||"--";
  $$(".mode-btn").forEach(b=>b.classList.toggle("active",b.dataset.mode===mode));
}

const SVC=[["slam","SLAM"],["slam_pgo","PGO"],["lidar","LiDAR"],["camera","Camera"],["brainstem","Brainstem"]];
function renderSvc(svcs={}){
  $("#svcGrid").innerHTML=SVC.map(([k,l])=>{const a=!!svcs[k];return`<div class="service"><div class="left"><span class="dot ${a?"on":"off"}"></span><span class="name">${l}</span></div><span class="state">${a?"ACTIVE":"OFF"}</span></div>`}).join("");
}

function renderStatus(s={}){
  S.status=s;setMode(s.mode);
  $("#slamVal").textContent=s.slam_profile||"--";
  $("#mapVal").textContent=s.active_map||"--";
  $("#timeVal").textContent=S.ttime;
  setDot($("#runDot"),!!s.lingtu_running);
  $("#runText").textContent=s.lingtu_running?"鍦ㄧ嚎":"绂荤嚎";
  renderSvc(s.services||{});
  const eb=$("#errBox"),he=!!s.last_error;
  eb.textContent=he?s.last_error:"鏃?;eb.classList.toggle("clean",!he);
}

function renderMaps(){
  const m=S.maps||[],g=$("#mapsGrid");
  if(!m.length){g.innerHTML='<div class="empty">鏆傛棤鍦板浘</div>';return}
  g.innerHTML=m.map(mp=>`
    <article class="map-card ${mp.active?"active":""}">
      <div class="map-head"><div class="map-name">${esc(mp.name)}</div>${mp.active?'<span class="tag active">ACTIVE</span>':'<span class="tag">READY</span>'}</div>
      <div class="badges">
        <span class="badge ${mp.has_pcd?"ok":""}">PCD ${mp.has_pcd?"OK":"--"}</span>
        <span class="badge ${mp.has_tomogram?"ok":""}">TOMO ${mp.has_tomogram?"OK":"--"}</span>
      </div>
      <button class="btn secondary" data-use="${encodeURIComponent(mp.name)}" ${mp.active?"disabled":""}>${mp.active?"褰撳墠鍦板浘":"鍒囨崲"}</button>
    </article>
  `).join("");
}

async function refresh(silent=false){
  try{renderStatus(await api("/api/status"))}catch(e){if(!silent)toast("鐘舵€佽幏鍙栧け璐?,"error")}
}
async function refreshMaps(silent=false){
  try{S.maps=(await api("/api/map/list")).maps||[];renderMaps()}catch(e){if(!silent)toast("鍦板浘鍒楄〃澶辫触","error")}
}

function connectSSE(){
  if(S.sse)S.sse.close();const es=new EventSource("/api/stream");S.sse=es;
  es.onopen=()=>{const rec=S.ever&&!S.connected;S.connected=true;S.ever=true;setDot($("#sseDot"),true);$("#sseText").textContent="閬ユ祴鍦ㄧ嚎";if(rec)toast("閬ユ祴宸叉仮澶?,"success")};
  es.onmessage=e=>{try{const d=JSON.parse(e.data);if(d.mode)setMode(d.mode);if(d.timestamp){S.ttime=new Date(d.timestamp*1000).toLocaleTimeString("zh-CN",{hour12:false});$("#timeVal").textContent=S.ttime}}catch{}};
  es.onerror=()=>{if(S.connected)toast("閬ユ祴涓柇锛岄噸杩炰腑","warn",3000);S.connected=false;setDot($("#sseDot"),false);$("#sseText").textContent="閲嶈繛涓?;es.close();setTimeout(connectSSE,2500)};
}

function parseNav(raw){
  const t=(raw||"").trim();if(!t)return null;
  const m=t.match(/^\(?\s*(-?\d+(?:\.\d+)?)\s*[,\s]\s*(-?\d+(?:\.\d+)?)\s*\)?$/);
  return m?{x:+m[1],y:+m[2],type:"coord"}:{instruction:t,type:"inst"};
}

async function sendMode(mode){try{await api("/api/mode","POST",{mode});setMode(mode);toast("宸插垏鎹? "+mode,"success");refresh(true)}catch(e){toast("鍒囨崲澶辫触: "+e.message,"error",3200)}}
async function sendNav(){
  const p=parseNav($("#navInput").value);if(!p)return toast("璇疯緭鍏ョ洰鏍?,"warn");
  try{if(p.type==="coord"){await api("/api/navigate","POST",{x:p.x,y:p.y});toast("鍧愭爣瀵艰埅: "+p.x+","+p.y,"success")}else{await api("/api/navigate","POST",{instruction:p.instruction});toast("璇箟瀵艰埅宸插彂閫?,"success")}$("#navInput").value=""}catch(e){toast("瀵艰埅澶辫触: "+e.message,"error",3200)}
}
async function saveMap(){
  const n=$("#mapNameInput").value.trim();if(!n)return toast("璇疯緭鍏ュ悕绉?,"warn");
  try{await api("/api/map/save","POST",{name:n});$("#mapNameInput").value="";toast("鍦板浘宸蹭繚瀛? "+n,"success");refreshMaps(true);refresh(true)}catch(e){toast("淇濆瓨澶辫触: "+e.message,"error",3200)}
}
async function stopRobot(){try{await api("/api/stop","POST");toast("鎬ュ仠宸茶Е鍙?,"error",3200)}catch(e){toast("鎬ュ仠澶辫触: "+e.message,"error",3200)}}

function bind(){
  $$(".mode-btn").forEach(b=>b.addEventListener("click",()=>sendMode(b.dataset.mode)));
  $("#navBtn").addEventListener("click",sendNav);
  $("#saveMapBtn").addEventListener("click",saveMap);
  $("#stopBtn").addEventListener("click",stopRobot);
  $("#refreshMaps").addEventListener("click",()=>refreshMaps());
  $("#navInput").addEventListener("keydown",e=>{if(e.key==="Enter")sendNav()});
  $("#mapNameInput").addEventListener("keydown",e=>{if(e.key==="Enter")saveMap()});
  $("#mapsGrid").addEventListener("click",async e=>{
    const b=e.target.closest("[data-use]");if(!b||b.disabled)return;
    const name=decodeURIComponent(b.dataset.use);
    try{await api("/api/map/use","POST",{name});toast("宸插垏鎹? "+name,"success");refreshMaps(true);refresh(true)}catch(e){toast("鍒囨崲澶辫触: "+e.message,"error",3200)}
  });
}

function refreshCam(){$("#camImg").src="/api/camera/snapshot?t="+Date.now()}
async function checkRerun(){
  try{
    const r=await api("/api/rerun/status");
    const w=$("#rerunWrap"),s=$("#rerunStatus");
    if(r.running){
      s.textContent="ACTIVE";s.style.color="var(--green)";
      w.innerHTML='<iframe src="http://'+location.hostname+':9090" style="width:100%;height:400px;border:none;border-radius:12px"></iframe>';
    }else{
      s.textContent="OFF";s.style.color="var(--muted)";
      w.innerHTML='<span style="color:var(--muted)">Rerun 鏈繍琛?鈥?鍚姩: python3 scripts/visualization/rerun_gateway_live.py</span>';
    }
  }catch{}
}
setInterval(refreshCam,3000);
setInterval(checkRerun,10000);

async function init(){bind();await Promise.all([refresh(true),refreshMaps(true)]);connectSSE();refreshCam();checkRerun()}
init();
</script>
</body>
</html>"""

# 鈹€鈹€ Main 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

if __name__ == "__main__":
    # Detect active map on startup
    active_link = os.path.join(MAP_DIR, "active")
    if os.path.islink(active_link):
        state.active_map = os.path.basename(os.readlink(active_link))

    # Detect current mode from running services
    if _is_active("slam"):
        state.mode = "map"
        state.slam_profile = "fastlio2"
    else:
        state.mode = "idle"

    logger.info(
        "LingTu Manager starting on %s:%d (mode=%s, origins=%s)",
        MANAGER_HOST,
        MANAGER_PORT,
        state.mode,
        MANAGER_ALLOWED_ORIGINS,
    )
    uvicorn.run(app, host=MANAGER_HOST, port=MANAGER_PORT, log_level="warning")
