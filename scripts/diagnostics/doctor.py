#!/usr/bin/env python3
"""lingtu doctor - check field services, hardware, and Module-first data flow."""

from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)


def check(name: str, ok: bool, detail: str = "") -> bool:
    mark = "\033[32mOK\033[0m" if ok else "\033[31mFAIL\033[0m"
    print("  [%s] %s%s" % (mark, name, ("  " + detail) if detail else ""))
    return ok


def skip(name: str, detail: str = "") -> None:
    print("  [\033[33m--\033[0m] %s%s" % (name, ("  " + detail) if detail else ""))


def service_active(name: str) -> bool:
    try:
        result = subprocess.run(["systemctl", "is-active", "--quiet", name], timeout=3)
        return result.returncode == 0
    except Exception:
        return False


def port_open(host: str, port: int) -> bool:
    sock = socket.socket()
    sock.settimeout(1)
    try:
        sock.connect((host, port))
        sock.close()
        return True
    except Exception:
        return False


def _read_environment_file(path: Path) -> dict[str, str]:
    if not path.is_file():
        return {}
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return {}
    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        if key not in {"LINGTU_BRAINSTEM_HOST", "LINGTU_BRAINSTEM_PORT"}:
            continue
        values[key] = value.strip().strip('"').strip("'")
    return values


def _brainstem_endpoint(
    env: Mapping[str, str] = os.environ,
    env_path: Path | None = None,
) -> tuple[str, int]:
    if env_path is None:
        config_dir = Path(env.get("LINGTU_CONFIG_DIR", "/opt/lingtu/config"))
        env_path = config_dir / "brainstem.env"
    configured = _read_environment_file(env_path)
    host = env.get("LINGTU_BRAINSTEM_HOST", "").strip() or configured.get(
        "LINGTU_BRAINSTEM_HOST", ""
    ).strip()
    port_text = env.get("LINGTU_BRAINSTEM_PORT", "").strip() or configured.get(
        "LINGTU_BRAINSTEM_PORT", "13145"
    ).strip()
    try:
        port = int(port_text)
    except ValueError:
        port = 13145
    if not 1 <= port <= 65535:
        port = 13145
    return host, port


def run(cmd: list[str], timeout: int = 5) -> str:
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
            check=False,
        )
        return result.stdout.strip()
    except Exception:
        return ""


def _safe_float_env(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def read_run_state() -> dict[str, object]:
    path = REPO_ROOT / ".lingtu" / "run.json"
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _gateway_json(base_url: str, path: str, *, timeout_sec: float) -> Mapping[str, Any]:
    url = f"{base_url.rstrip('/')}/{path.lstrip('/')}"
    request = Request(url, headers={"Accept": "application/json"})
    try:
        with urlopen(request, timeout=timeout_sec) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"{url}: {exc}") from exc
    return payload if isinstance(payload, Mapping) else {}


def _nested(payload: Mapping[str, Any], *keys: str) -> Mapping[str, Any]:
    current: Any = payload
    for key in keys:
        if not isinstance(current, Mapping):
            return {}
        current = current.get(key)
    return current if isinstance(current, Mapping) else {}


def _count_live_topics(topics: Any) -> tuple[int, int]:
    if not isinstance(topics, list):
        return (0, 0)
    live = 0
    for topic in topics:
        if not isinstance(topic, Mapping):
            continue
        inspection = _nested(topic, "inspection")
        if inspection.get("live") is True or inspection.get("observable") is True:
            live += 1
    return (len(topics), live)


def _count_live_stages(stages: Any) -> tuple[int, int, int]:
    if not isinstance(stages, list):
        return (0, 0, 0)
    live = 0
    missing = 0
    for stage in stages:
        if not isinstance(stage, Mapping):
            continue
        if stage.get("live") is True:
            live += 1
        if stage.get("status") == "missing" or stage.get("observable") is False:
            missing += 1
    return (len(stages), live, missing)


def run_gateway_dataflow_checks(args: argparse.Namespace) -> None:
    print("\n  --- Runtime Data Flow (Gateway) ---")
    try:
        dataflow = _gateway_json(
            args.gateway_url,
            "/api/v1/runtime/dataflow",
            timeout_sec=args.gateway_timeout_sec,
        )
    except RuntimeError as exc:
        skip("Gateway runtime dataflow", str(exc))
        return

    transport_layers = _nested(dataflow, "transport_layers")
    module_port_bus = _nested(transport_layers, "module_port_bus")
    ros2_adapter = _nested(transport_layers, "ros2_adapter")
    control_boundary = _nested(dataflow, "control_boundary")
    runtime_boundary = _nested(dataflow, "runtime_boundary")
    topics_total, topics_live = _count_live_topics(dataflow.get("topics"))
    stages_total, stages_live, stages_missing = _count_live_stages(dataflow.get("stage_evidence"))

    check("Gateway runtime dataflow", True, args.gateway_url)
    check("ModulePort bus primary", module_port_bus.get("primary") is True)
    check("ROS2 adapter not primary", ros2_adapter.get("primary") is not True)
    check(
        "Command publish boundary",
        control_boundary.get("arbitrary_publish_supported") is not True,
        "arbitrary publish disabled",
    )
    if runtime_boundary:
        check("Runtime boundary", runtime_boundary.get("ok") is True)
    else:
        skip("Runtime boundary", "not reported by Gateway")
    check("Observable topics", topics_total > 0, f"{topics_live}/{topics_total} live")
    check(
        "Observable stages",
        stages_total > 0,
        f"{stages_live}/{stages_total} live, {stages_missing} missing",
    )


def run_service_checks() -> None:
    print("  --- Services ---")
    for svc in ["brainstem", "lidar", "camera"]:
        check(svc, service_active(svc))
    for svc in ["slam", "slam_pgo", "localizer"]:
        active = service_active(svc)
        check(svc, True, "active" if active else "inactive (on-demand)")


def run_hardware_checks() -> None:
    print("\n  --- Hardware ---")
    lsusb = run(["lsusb"])
    livox_ping = run(["ping", "-c1", "-W1", "192.168.1.115"])
    check("LiDAR (Livox)", "192.168.1.115" in livox_ping, "192.168.1.115")
    check("Camera (Orbbec)", "orbbec" in lsusb.lower() or "2bc5" in lsusb.lower())
    check("IMU (CP210x)", "cp210x" in lsusb.lower() or "10c4" in lsusb.lower())
    check("Xbox Controller", "8bitdo" in lsusb.lower() or "xbox" in lsusb.lower())
    check("CAN Bus", os.path.exists("/sys/class/net/can0"))


def run_port_checks() -> None:
    print("\n  --- Ports ---")
    brainstem_host, brainstem_port = _brainstem_endpoint()
    if brainstem_host:
        check(
            f"brainstem gRPC {brainstem_host}:{brainstem_port}",
            port_open(brainstem_host, brainstem_port),
        )
    else:
        skip("brainstem gRPC", "LINGTU_BRAINSTEM_HOST is not configured")
    for name, port in [("Gateway :5050", 5050), ("MCP :8090", 8090)]:
        if port_open("127.0.0.1", port):
            check(name, True)
        else:
            skip(name, "lingtu not running")


def run_state_checks() -> None:
    print("\n  --- Runtime State ---")
    state = read_run_state()
    if state:
        check(
            "run.json",
            True,
            "profile=%s pid=%s" % (state.get("profile", "?"), state.get("pid", "?")),
        )
    else:
        check("run.json", False, "LingTu daemon state not found")


def run_map_checks() -> None:
    print("\n  --- Maps ---")
    map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
    if os.path.isdir(map_dir):
        maps = [name for name in os.listdir(map_dir) if os.path.isdir(os.path.join(map_dir, name)) and name != "active"]
        active_path = os.path.join(map_dir, "active")
        active = os.path.basename(os.readlink(active_path)) if os.path.islink(active_path) else "none"
        check("Maps directory", True, "%d maps, active=%s" % (len(maps), active))
    else:
        check("Maps directory", False, map_dir)


def run_python_checks() -> None:
    print("\n  --- Python ---")
    sys.path.insert(0, str(REPO_ROOT / "src"))
    try:
        import lingtu

        check("lingtu importable", True)
    except Exception as exc:
        check("lingtu importable", False, str(exc))

    try:
        from lingtu.assembly.profile_builder import blueprint_for_resolved_profile
        from runtime.profiles.resolver import resolve_profile_config

        cfg = resolve_profile_config("nav", overrides={"run_startup_checks": False})
        blueprint_for_resolved_profile("nav", cfg)
        check("Thunder profile builder", True)
    except Exception as exc:
        check("Thunder profile builder", False, str(exc))

    try:
        from nav.kernel import require_nav_kernel

        module = require_nav_kernel(context="doctor native kernel")
        check("LingTu native navigation kernel", True, getattr(module, "__file__", "loaded"))
    except Exception as exc:
        check("LingTu native navigation kernel", False, str(exc).splitlines()[0])


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run LingTu field diagnostics.")
    parser.add_argument(
        "--gateway-url",
        default=os.environ.get("LINGTU_GATEWAY_URL", "http://127.0.0.1:5050"),
        help="Gateway base URL for Module-first runtime dataflow checks.",
    )
    parser.add_argument(
        "--gateway-timeout-sec",
        type=float,
        default=_safe_float_env("LINGTU_GATEWAY_TIMEOUT_SEC", 2.0),
        help="Gateway request timeout in seconds.",
    )
    parser.add_argument(
        "--skip-gateway",
        action="store_true",
        help="Skip Gateway runtime dataflow checks.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    print("\n  \033[1mLingTu Doctor\033[0m\n")
    run_service_checks()
    run_hardware_checks()
    run_port_checks()
    run_state_checks()
    if args.skip_gateway:
        print("\n  --- Runtime Data Flow (Gateway) ---")
        skip("Gateway runtime dataflow", "skipped")
    else:
        run_gateway_dataflow_checks(args)
    run_map_checks()
    run_python_checks()
    print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
