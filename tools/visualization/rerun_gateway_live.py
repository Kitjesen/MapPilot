#!/usr/bin/env python3
"""Gateway-backed Rerun live viewer for LingTu.

This viewer does not subscribe to ROS2 topics. It polls read-only Gateway
endpoints and logs the available Product runtime state to Rerun.
Native DDS inspection is handled by ``python -m diagnostics.field.dds_readiness``.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Mapping, Sequence
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

logger = logging.getLogger(__name__)


def find_repo_root(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "pyproject.toml").is_file() and (path / "AGENTS.md").is_file():
            return path
    raise RuntimeError(f"Could not find repository root from {start}")


REPO_ROOT = find_repo_root(Path(__file__).resolve().parent)
sys.path.insert(0, str(REPO_ROOT / "src"))


class GatewayError(RuntimeError):
    """Raised when the Gateway viewer cannot read a payload."""


def _fetch_json(
    base_url: str,
    path: str,
    *,
    timeout_sec: float,
    params: Mapping[str, Any] | None = None,
) -> Mapping[str, Any]:
    url = f"{base_url.rstrip('/')}/{path.lstrip('/')}"
    if params:
        url = f"{url}?{urlencode(params)}"
    request = Request(url, headers={"Accept": "application/json"})  # noqa: S310 - operator-provided Gateway URL
    try:
        with urlopen(request, timeout=timeout_sec) as response:  # noqa: S310 - operator-provided Gateway URL
            payload = json.loads(response.read().decode("utf-8"))
    except (HTTPError, URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
        raise GatewayError(f"{url}: {exc}") from exc
    return payload if isinstance(payload, Mapping) else {}


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _xyz(value: Any, *, default_z: float = 0.35) -> list[float] | None:
    payload = _mapping(value)
    for nested_key in ("position", "translation", "pose"):
        nested = payload.get(nested_key)
        if isinstance(nested, Mapping):
            found = _xyz(nested, default_z=default_z)
            if found is not None:
                return found
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        if len(value) >= 2:
            x = _number(value[0])
            y = _number(value[1])
            z = _number(value[2]) if len(value) >= 3 else default_z
            if x is not None and y is not None:
                return [x, y, z if z is not None else default_z]
    x = _number(payload.get("x"))
    y = _number(payload.get("y"))
    z = _number(payload.get("z"))
    if x is None or y is None:
        return None
    return [x, y, z if z is not None else default_z]


def _extract_points(payload: Any) -> list[list[float]]:
    if isinstance(payload, Sequence) and not isinstance(payload, (str, bytes, bytearray)):
        return [point for item in payload if (point := _xyz(item, default_z=0.3))]

    mapping = _mapping(payload)
    for key in ("points", "poses", "path", "waypoints", "items"):
        value = mapping.get(key)
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
            return [point for item in value if (point := _xyz(item, default_z=0.3))]

    nested = _mapping(mapping.get("data") or mapping.get("latest_payload"))
    if nested:
        return _extract_points(nested)
    return []


def _height_colors(points: Sequence[Sequence[float]]) -> list[list[int]]:
    if not points:
        return []
    zs = [float(point[2]) for point in points]
    z_min = min(zs)
    z_span = max(max(zs) - z_min, 0.01)
    colors: list[list[int]] = []
    for z in zs:
        t = max(0.0, min(1.0, (z - z_min) / z_span))
        if t < 0.5:
            u = t * 2.0
            colors.append(
                [
                    round(46 + (140 - 46) * u),
                    140,
                    round(128 + (140 - 128) * u),
                ]
            )
        else:
            u = (t - 0.5) * 2.0
            colors.append(
                [
                    round(140 + (199 - 140) * u),
                    round(140 + (153 - 140) * u),
                    round(140 + (89 - 140) * u),
                ]
            )
    return colors


def _init_rerun(args: argparse.Namespace):
    try:
        import rerun as rr
    except ImportError as exc:
        raise SystemExit("rerun-sdk is not installed; install rerun-sdk to view live data") from exc

    rr.init("lingtu_gateway_live")
    if args.native:
        rr.spawn(connect=True)
        print("Rerun native viewer started")
    elif not args.no_serve:
        if hasattr(rr, "serve_grpc") and hasattr(rr, "serve_web_viewer"):
            server_uri = rr.serve_grpc(grpc_port=args.grpc_port)
            rr.serve_web_viewer(
                open_browser=False,
                web_port=args.web_port,
                connect_to=server_uri,
            )
        elif hasattr(rr, "serve_web"):
            rr.serve_web(open_browser=False, web_port=args.web_port)
        print(f"Rerun web: http://localhost:{args.web_port}")
    try:
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    except Exception as exc:
        logger.debug("Rerun coordinate-system log failed: %s", exc)
    return rr


def _log_state(rr: Any, payload: Mapping[str, Any], trajectory: list[list[float]]) -> None:
    state = _mapping(payload.get("state") or payload)
    odom = _mapping(state.get("odometry") or state.get("odom") or payload.get("odometry"))
    position = _xyz(odom) or _xyz(state)
    if position is None:
        return
    rr.log("world/robot/position", rr.Points3D([position], colors=[[0, 255, 0]], radii=0.15))
    trajectory.append(position)
    if len(trajectory) > 1000:
        del trajectory[:-1000]
    if len(trajectory) >= 2:
        rr.log("world/robot/trajectory", rr.LineStrips3D([trajectory], colors=[[0, 120, 255]]))


def _log_path(rr: Any, payload: Mapping[str, Any]) -> None:
    points = _extract_points(payload)
    if len(points) >= 2:
        rr.log("world/navigation/path", rr.LineStrips3D([points], colors=[[0, 220, 120]], radii=0.04))


def _log_map_points(rr: Any, payload: Mapping[str, Any], *, max_points: int) -> None:
    points = _extract_points(payload)
    if not points:
        return
    if len(points) > max_points:
        step = max(1, len(points) // max_points)
        points = points[::step][:max_points]
    rr.log("world/map/points", rr.Points3D(points, colors=_height_colors(points), radii=0.035))


def _log_dataflow(rr: Any, payload: Mapping[str, Any]) -> None:
    topics = payload.get("topics") if isinstance(payload.get("topics"), list) else []
    stages = payload.get("stage_evidence") if isinstance(payload.get("stage_evidence"), list) else []
    live_topics = sum(1 for topic in topics if _mapping(_mapping(topic).get("inspection")).get("live") is True)
    live_stages = sum(1 for stage in stages if _mapping(stage).get("live") is True)
    text = (
        f"contract={payload.get('runtime_contract') or '?'} "
        f"topics={live_topics}/{len(topics)} live "
        f"stages={live_stages}/{len(stages)} live"
    )
    try:
        rr.log("metrics/runtime_dataflow", rr.TextLog(text))
    except Exception as exc:
        logger.debug("Rerun dataflow log failed: %s", exc)


def _poll_once(rr: Any, args: argparse.Namespace, trajectory: list[list[float]]) -> None:
    endpoints = (
        ("state", "/api/v1/state"),
        ("path", "/api/v1/path"),
        ("map", "/api/v1/map/points"),
        ("dataflow", "/api/v1/runtime/dataflow"),
    )
    errors: list[str] = []
    for name, path in endpoints:
        try:
            payload = _fetch_json(
                args.gateway_url,
                path,
                timeout_sec=args.gateway_timeout_sec,
            )
        except GatewayError as exc:
            errors.append(str(exc))
            continue
        if name == "state":
            _log_state(rr, payload, trajectory)
        elif name == "path":
            _log_path(rr, payload)
        elif name == "map":
            _log_map_points(rr, payload, max_points=args.max_map_points)
        elif name == "dataflow":
            _log_dataflow(rr, payload)
    if errors:
        print("Gateway warnings: " + " | ".join(errors[:2]))


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="LingTu Gateway Rerun live viewer")
    parser.add_argument("--gateway-url", default="http://127.0.0.1:5050")
    parser.add_argument("--gateway-timeout-sec", type=float, default=2.0)
    parser.add_argument("--poll-sec", type=float, default=0.25)
    parser.add_argument("--web-port", type=int, default=9090)
    parser.add_argument("--grpc-port", type=int, default=9877)
    parser.add_argument("--max-map-points", type=int, default=20000)
    parser.add_argument("--native", action="store_true")
    parser.add_argument("--no-serve", action="store_true")
    parser.add_argument("--once", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(list(argv or sys.argv[1:]))
    rr = _init_rerun(args)
    trajectory: list[list[float]] = []
    print(f"Streaming Gateway data from {args.gateway_url}")
    while True:
        _poll_once(rr, args, trajectory)
        if args.once:
            return 0
        time.sleep(max(args.poll_sec, 0.05))


if __name__ == "__main__":
    raise SystemExit(main())
