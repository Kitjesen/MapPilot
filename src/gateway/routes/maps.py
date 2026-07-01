"""Map lifecycle and viewer routes for GatewayModule."""

from __future__ import annotations

import logging
import math
import os
import pathlib
import shutil
import struct
import time
from datetime import datetime
from typing import Any

from fastapi import HTTPException
from fastapi.responses import FileResponse, JSONResponse
from starlette.responses import HTMLResponse

from core.map_save import (
    MapSaveError,
    save_nav_map_with_adapter,
    save_pgo_map_with_adapter,
)
from core.msgs.numpy_compat import is_numpy_array, np
from core.plugin_seed import seed_registered_plugins
from core.runtime_interface import TOPICS, topic_default_frame_id
from core.same_source_map_artifacts import (
    validate_saved_map_artifact_dir,
)
from gateway.schemas import (
    MapLifecycleResponse,
    MapListResponse,
    MapNameRequest,
    MapPointsResponse,
    MapRenameRequest,
    MapSaveRequest,
)
from gateway.services.map_paths import active_map_name, nav_map_root_str
from gateway.services.map_safety import (
    safe_map_name,
)
from gateway.services.runtime_status import backend_capability_defaults

logger = logging.getLogger(__name__)


def _apply_dynamic_filter_step1half(*args: Any, **kwargs: Any) -> Any:
    from core.dynamic_filter import apply_dynamic_filter_step1half

    return apply_dynamic_filter_step1half(*args, **kwargs)


apply_dynamic_filter_step1half = _apply_dynamic_filter_step1half


def _map_dir() -> str:
    return nav_map_root_str()


def _body_mapping(body: Any) -> dict[str, Any]:
    """Normalise request body to a plain dict.

    Why raw dicts are accepted:
      Pydantic models define fixed fields, but ROS2 frontends / WebSocket
      messages send JSON with backend-variant keys (e.g. "map_name" vs.
      "map", "slam_profile" vs. "slam_backend"). Accepting raw dicts avoids
      per-endpoint model proliferation and keeps route handlers flexible.

    Trade-off:
      Pydantic type coercion (float, int, str trim) is bypassed at the
      boundary. Call sites MUST manually coerce numeric fields with
      float() / int() — see slam_relocalize (x, y, yaw) and bag_start
      (duration) for examples.
    """
    if hasattr(body, "model_dump"):
        return body.model_dump(exclude_none=True)
    assert isinstance(body, dict), f"expected dict or Pydantic model, got {type(body).__name__}"
    return body


def map_lifecycle_payload(success: bool, **fields: Any) -> dict[str, Any]:
    payload = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    payload.update({key: value for key, value in fields.items() if value is not None})
    return payload


def _map_lifecycle_response(
    success: bool,
    *,
    status_code: int,
    **fields: Any,
) -> JSONResponse:
    return JSONResponse(
        map_lifecycle_payload(success, **fields),
        status_code=status_code,
    )


def _safe_map_file(name: str, filename: str) -> pathlib.Path:
    base = pathlib.Path(_map_dir()).resolve()
    path = (base / name / filename).resolve()
    try:
        path.relative_to(base)
    except ValueError as exc:
        raise HTTPException(status_code=403) from exc
    return path


def _write_binary_xyz_pcd(path: pathlib.Path, points: Any) -> int:
    if is_numpy_array(points):
        return _write_binary_xyz_pcd_array(path, points)

    rows = points.tolist() if hasattr(points, "tolist") else points
    pts: list[tuple[float, float, float]] = []
    for row in rows or []:
        try:
            x, y, z = float(row[0]), float(row[1]), float(row[2])
        except (TypeError, ValueError, IndexError):
            continue
        if (
            math.isfinite(x)
            and math.isfinite(y)
            and math.isfinite(z)
            and max(abs(x), abs(y), abs(z)) < 500.0
        ):
            pts.append((x, y, z))

    if not pts:
        raise ValueError("no valid points to save")
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {len(pts)}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {len(pts)}\n"
        "DATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        for point in pts:
            f.write(struct.pack("<fff", *point))
    return len(pts)


def _write_binary_xyz_pcd_array(path: pathlib.Path, points: Any) -> int:
    arr = np.asarray(points, dtype=np.float32)
    if arr.ndim != 2 or arr.shape[1] < 3:
        raise ValueError("point cloud must have shape Nx3")
    xyz = arr[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    bounded = np.max(np.abs(xyz), axis=1) < 500.0
    valid = np.ascontiguousarray(xyz[finite & bounded], dtype=np.float32)
    if valid.size == 0:
        raise ValueError("no valid points to save")
    count = int(valid.shape[0])
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(valid.astype("<f4", copy=False).tobytes())
    return count


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) * 0.5


def _sample_xyz_points(
    points: list[tuple[float, float, float]],
    max_points: int,
) -> list[tuple[float, float, float]]:
    if max_points <= 0:
        return []
    if len(points) <= max_points:
        return points
    step = len(points) / float(max_points)
    return [points[min(int(index * step), len(points) - 1)] for index in range(max_points)]


def _binary_pcd_xyz_payload(
    data: bytes,
    *,
    n_points: int,
    point_step: int,
    max_points: int,
) -> tuple[int, list[float]]:
    if n_points <= 0 or point_step < 12:
        return 0, []

    points: list[tuple[float, float, float]] = []
    available = min(n_points, len(data) // point_step)
    for index in range(available):
        x, y, z = struct.unpack_from("<fff", data, index * point_step)
        if all(math.isfinite(value) for value in (x, y, z)):
            points.append((float(x), float(y), float(z)))

    if points:
        med = (
            _median([point[0] for point in points]),
            _median([point[1] for point in points]),
            _median([point[2] for point in points]),
        )
        points = [
            point
            for point in points
            if max(abs(point[axis] - med[axis]) for axis in range(3)) < 100.0
        ]

    points = _sample_xyz_points(points, int(max_points))
    flat = [coord for point in points for coord in point]
    return len(points), flat


def register_map_routes(app, gw) -> None:
    @app.get(
        "/api/v1/slam/maps",
        summary="List maps from filesystem",
        response_model=MapListResponse,
    )
    async def slam_maps():
        map_dir = _map_dir()
        maps = []
        active_target = active_map_name() or ""

        if os.path.isdir(map_dir):
            for d in sorted(os.listdir(map_dir)):
                full = os.path.join(map_dir, d)
                if not os.path.isdir(full) or d.startswith("_") or d == "active":
                    continue
                pcd = os.path.join(full, "map.pcd")
                has_pcd = os.path.isfile(pcd)
                patches_dir = os.path.join(full, "patches")
                patch_count = (
                    len(os.listdir(patches_dir)) if os.path.isdir(patches_dir) else 0
                )
                has_tomogram = os.path.isfile(os.path.join(full, "tomogram.pickle"))
                size_mb: float | None = None
                if has_pcd:
                    sz = os.path.getsize(pcd)
                    size_mb = round(sz / 1024 / 1024, 1)
                maps.append(
                    {
                        "name": d,
                        "has_pcd": has_pcd,
                        "has_tomogram": has_tomogram,
                        "is_active": d == active_target,
                        "size_mb": size_mb,
                        "patch_count": patch_count,
                    }
                )
        return {
            "schema_version": 1,
            "maps": maps,
            "count": len(maps),
            "active": active_target,
            "map_dir": map_dir,
            "ts": time.time(),
        }

    @app.get(
        "/api/v1/maps/{name}/pcd",
        summary="Serve raw PCD file for inline preview",
        responses={
            200: {
                "content": {
                    "application/octet-stream": {
                        "schema": {"type": "string", "format": "binary"}
                    }
                }
            }
        },
    )
    async def get_map_pcd(name: str):
        pcd_path = _safe_map_file(name, "map.pcd")
        if not pcd_path.is_file():
            raise HTTPException(status_code=404, detail=f"No PCD for map: {name}")
        return FileResponse(
            str(pcd_path),
            media_type="application/octet-stream",
            filename="map.pcd",
        )

    @app.get(
        "/api/v1/maps/{name}/points",
        summary="Saved map point cloud as JSON",
        response_model=MapPointsResponse,
    )
    async def get_saved_map_points(name: str, max_points: int = 30000):
        pcd_path = _safe_map_file(name, "map.pcd")
        if not pcd_path.is_file():
            raise HTTPException(status_code=404, detail=f"Map not found: {name}")

        with open(pcd_path, "rb") as f:
            n_points, point_step = 0, 16
            while True:
                line = f.readline().decode("ascii", errors="ignore").strip()
                if "POINTS" in line:
                    n_points = int(line.split()[-1])
                if "SIZE" in line:
                    point_step = sum(int(s) for s in line.split()[1:])
                if line.startswith("DATA"):
                    break
            data = f.read(n_points * point_step)
        count, flat = _binary_pcd_xyz_payload(
            data[: n_points * point_step],
            n_points=n_points,
            point_step=point_step,
            max_points=max_points,
        )
        return {
            "schema_version": 1,
            "count": count,
            "layout": "flat_xyz",
            "frame_id": "map",
            "source": "saved_map_pcd",
            "name": name,
            "points": flat,
            "ts": time.time(),
        }

    @app.get(
        "/api/v1/map/points",
        summary="Map point cloud as JSON (from ikd-tree snapshot)",
        response_model=MapPointsResponse,
    )
    async def get_map_points(max_points: int = 80000):
        with gw._map_cloud_lock:
            pts = gw._map_points
        if pts is None or len(pts) == 0:
            return {
                "schema_version": 1,
                "count": 0,
                "layout": "xyz_rows",
                "frame_id": "map",
                "source": "live_map_cloud",
                "points": [],
                "ts": time.time(),
            }
        if len(pts) > max_points:
            idx = np.random.choice(len(pts), max_points, replace=False)
            pts = pts[idx]
        return {
            "count": len(pts),
            "layout": "xyz_rows",
            "frame_id": "map",
            "source": "live_map_cloud",
            "bounds": {
                "x": [float(pts[:, 0].min()), float(pts[:, 0].max())],
                "y": [float(pts[:, 1].min()), float(pts[:, 1].max())],
                "z": [float(pts[:, 2].min()), float(pts[:, 2].max())],
            },
            "points": pts[:, :3].tolist(),
            "ts": time.time(),
        }

    @app.post(
        "/api/v1/map_cloud/reset",
        summary="Clear accumulated map cloud (viz only, SLAM ikd-tree untouched)",
        response_model=MapLifecycleResponse,
    )
    async def reset_map_cloud():
        with gw._map_cloud_lock:
            gw._map_points = None
            gw._map_cloud_count = 0
            gw._voxel_hits.clear()
        gw.push_event({"type": "map_cloud", "points": [], "count": 0})
        return map_lifecycle_payload(
            True,
            message="Accumulated map cloud cleared",
        )

    @app.get("/map/viewer", summary="Interactive 3D map viewer")
    async def map_viewer(map: str = ""):
        if map:
            html = gw._generate_viewer_from_pcd(map)
        else:
            html = gw._generate_viewer_live()
        return HTMLResponse(html)

    @app.get("/robot/meshes/{filename}", summary="Serve robot STL mesh files")
    async def serve_robot_mesh(filename: str):
        mesh_dir = os.environ.get(
            "DOG_MESH_DIR",
            os.path.join(
                os.path.dirname(__file__),
                "../../../../products/quadruped_ws/dog_arm/meshes",
            ),
        )
        safe_name = os.path.basename(filename)
        path = os.path.join(mesh_dir, safe_name)
        if not os.path.isfile(path):
            return JSONResponse(
                status_code=404,
                content={"error": "mesh not found", "name": safe_name},
            )
        return FileResponse(
            path,
            media_type="application/octet-stream",
            headers={
                "Access-Control-Allow-Origin": "*",
                "Cache-Control": "public, max-age=3600",
            },
        )

    @app.post(
        "/api/v1/map/restore_predufo",
        summary="Restore map.pcd from DUFOMap pre-filter backup",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def restore_predufo(body: MapNameRequest):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        map_dir = _map_dir()
        target = pathlib.Path(map_dir) / name
        pcd = target / "map.pcd"
        backup = target / "map.pcd.predufo"
        if not backup.is_file():
            return _map_lifecycle_response(
                False,
                message=(
                    f"No predufo backup for {name}. "
                    "DUFOMap may not have run on this map."
                ),
                status_code=404,
            )
        try:
            import time as _t

            if pcd.is_file():
                shutil.copy(pcd, target / f"map.pcd.replaced-{_t.time_ns()}")
            tmp_new = pcd.with_suffix(".pcd.tmp")
            shutil.copy(backup, tmp_new)
            os.replace(tmp_new, pcd)

            replaced = sorted(
                target.glob("map.pcd.replaced-*"),
                key=lambda p: p.stat().st_mtime,
                reverse=True,
            )
            pruned = 0
            for old in replaced[3:]:
                try:
                    old.unlink()
                    pruned += 1
                except Exception as e:
                    logger.warning("cleanup old backup failed: %s", e)

            return map_lifecycle_payload(
                True,
                name=name,
                restored_size=pcd.stat().st_size,
                replaced_backups_kept=min(len(replaced), 3),
                replaced_backups_pruned=pruned,
                note="tomogram/occupancy must be rebuilt before planner use",
            )
        except Exception as e:
            logger.exception("restore_predufo failed")
            return _map_lifecycle_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/map/activate",
        summary="Set active map (symlink)",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def activate_map(body: MapNameRequest):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        map_dir = _map_dir()
        target = os.path.join(map_dir, name)
        if not os.path.isdir(target):
            return _map_lifecycle_response(
                False,
                message=f"Map does not exist: {name}",
                status_code=404,
            )
        artifact_gate = validate_saved_map_artifact_dir(
            target,
            require_tomogram=True,
            expected_frame_id=topic_default_frame_id(TOPICS.saved_map_cloud),
        )
        artifact_gate["required"] = True
        if artifact_gate.get("ok") is not True:
            blockers = [
                str(item)
                for item in (artifact_gate.get("blockers") or [])
                if str(item)
            ]
            detail = "; ".join(blockers) if blockers else "unknown blocker"
            return _map_lifecycle_response(
                False,
                message=f"saved map artifact gate failed: {detail}",
                artifact_gate=artifact_gate,
                status_code=409,
            )
        active_link = pathlib.Path(map_dir) / "active"
        try:
            if active_link.is_symlink() or active_link.exists():
                active_link.unlink()
            active_link.symlink_to(name)
            return map_lifecycle_payload(True, active=name)
        except Exception as e:
            return _map_lifecycle_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/map/rename",
        summary="Rename a saved map",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            404: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def rename_map(body: MapRenameRequest):
        payload = _body_mapping(body)
        old = payload.get("old_name", "")
        new = payload.get("new_name", "")
        err_old = safe_map_name(old)
        err_new = safe_map_name(new)
        if err_old or err_new:
            return _map_lifecycle_response(
                False,
                message=err_old or err_new,
                status_code=400,
            )
        map_dir = _map_dir()
        old_path = os.path.join(map_dir, old)
        new_path = os.path.join(map_dir, new)
        if not os.path.isdir(old_path):
            return _map_lifecycle_response(
                False,
                message=f"Map does not exist: {old}",
                status_code=404,
            )
        if os.path.exists(new_path):
            return _map_lifecycle_response(
                False,
                message=f"Name already exists: {new}",
                status_code=409,
            )
        try:
            os.rename(old_path, new_path)
            active_link = pathlib.Path(map_dir) / "active"
            if active_link.is_symlink() and active_link.resolve().name == old:
                active_link.unlink()
                active_link.symlink_to(new)
            return map_lifecycle_payload(True, old_name=old, new_name=new)
        except Exception as e:
            return _map_lifecycle_response(False, message=str(e), status_code=500)

    @app.post(
        "/api/v1/map/save",
        summary="Save current SLAM map",
        response_model=MapLifecycleResponse,
        responses={
            400: {"model": MapLifecycleResponse},
            409: {"model": MapLifecycleResponse},
            500: {"model": MapLifecycleResponse},
        },
    )
    async def save_map_now(body: MapSaveRequest = MapSaveRequest()):
        payload = _body_mapping(body)
        name = payload.get("name", "")
        if not name:
            name = "map_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        err = safe_map_name(name)
        if err is not None:
            return _map_lifecycle_response(False, message=err, status_code=400)
        errors = []
        save_source = "slam_service"
        slam_profile = "unknown"
        try:
            slam_profile = gw._get_slam_profile()
        except Exception:
            slam_profile = getattr(gw, "_session_slam_profile", "unknown")

        if slam_profile == "super_lio_relocation":
            capability_defaults = backend_capability_defaults(slam_profile)
            return _map_lifecycle_response(
                False,
                status_code=409,
                name=name,
                message=(
                    "Map save is disabled for Super-LIO relocation; "
                    "use the active saved map instead"
                ),
                errors=[
                    "super_lio_relocation uses active_map as its map source "
                    "and must not overwrite saved-map evidence during evaluation"
                ],
                slam_profile=slam_profile,
                source=capability_defaults["map_save_source"],
                map_save_source=capability_defaults["map_save_source"],
                relocalization_supported=capability_defaults[
                    "relocalization_supported"
                ],
                saved_map_relocalization_supported=capability_defaults[
                    "saved_map_relocalization_supported"
                ],
                restart_recovery_supported=capability_defaults[
                    "restart_recovery_supported"
                ],
                recovery_method=capability_defaults["recovery_method"],
            )

        map_dir = _map_dir()
        save_dir = os.path.join(map_dir, name)
        os.makedirs(save_dir, exist_ok=True)
        pcd_path = os.path.join(save_dir, "map.pcd")
        map_save_adapter = getattr(gw, "_map_save_adapter", None)
        if map_save_adapter is None:
            seed_registered_plugins(groups=("map_save_adapter",), reload_loaded=False)

        try:
            save_nav_map_with_adapter(
                map_save_adapter,
                pcd_path,
                timeout_sec=30.0,
            )
        except Exception as e:
            errors.append(f"Fast-LIO2: {e}")

        try:
            save_pgo_map_with_adapter(
                map_save_adapter,
                save_dir,
                save_patches=True,
                timeout_sec=30.0,
            )
        except MapSaveError as e:
            logger.debug("PGO SaveMaps compatibility adapter failed: %s", e)

        if not os.path.isfile(pcd_path) and slam_profile == "super_lio":
            try:
                with gw._map_cloud_lock:
                    pts = None if gw._map_points is None else gw._map_points.copy()
                if pts is None or len(pts) == 0:
                    errors.append("Super-LIO: live map_cloud snapshot unavailable")
                else:
                    point_count = _write_binary_xyz_pcd(pathlib.Path(pcd_path), pts)
                    save_source = "live_map_cloud_snapshot"
                    errors.append(
                        "Super-LIO: saved live map_cloud snapshot "
                        f"({point_count} points); relocalize service is unsupported"
                    )
            except Exception as e:
                errors.append(f"Super-LIO snapshot: {e}")

        dufo_result = apply_dynamic_filter_step1half(save_dir)

        has_pcd = os.path.isfile(pcd_path)
        if has_pcd:
            size = os.path.getsize(pcd_path)
            capability_defaults = backend_capability_defaults(slam_profile)
            resp = map_lifecycle_payload(
                True,
                name=name,
                path=save_dir,
                size=f"{size / 1024 / 1024:.1f}MB",
                slam_profile=slam_profile,
                source=save_source,
                map_save_source=save_source,
                relocalization_supported=capability_defaults[
                    "relocalization_supported"
                ],
                saved_map_relocalization_supported=capability_defaults[
                    "saved_map_relocalization_supported"
                ],
                restart_recovery_supported=capability_defaults[
                    "restart_recovery_supported"
                ],
                recovery_method=capability_defaults["recovery_method"],
            )
            if dufo_result is not None:
                resp["dynamic_filter"] = dufo_result
            if errors:
                resp["warnings"] = errors
            return resp
        return _map_lifecycle_response(
            False,
            name=name,
            message="Map save failed",
            errors=errors,
            status_code=500,
        )
