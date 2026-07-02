"""Voxel edit helpers for saved OctoPlanner3D OctoMap artifacts."""

from __future__ import annotations

import json
import os
import shlex
import shutil
import subprocess
import time
from pathlib import Path
from typing import Any, Sequence

from runtime.same_source_map_artifacts import sha256_file

EDIT_STATES = {"occupied", "free", "preblocked", "traversable", "clear"}
EDIT_SHAPES = {"sphere", "box"}


def edit_saved_octomap(
    map_dir: Path,
    cmd: dict[str, Any],
    *,
    editor_command: Sequence[str] | str | None = None,
    timeout_sec: float = 15.0,
) -> dict[str, Any]:
    octomap = _octomap_path(map_dir)
    if octomap is None:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": "map has no octomap.ot or octomap.bt artifact",
        }

    state = str(cmd.get("state") or cmd.get("mode") or "").strip().lower()
    if state not in EDIT_STATES:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": f"state must be one of {sorted(EDIT_STATES)}",
        }
    shape = str(cmd.get("shape") or "sphere").strip().lower()
    if shape not in EDIT_SHAPES:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": f"shape must be one of {sorted(EDIT_SHAPES)}",
        }

    try:
        x, y, z = _center_xyz(cmd)
        radius = _radius(cmd)
    except (TypeError, ValueError) as exc:
        return {"action": "edit_voxels", "success": False, "message": str(exc)}

    editor = editor_command or os.environ.get("LINGTU_OCTOMAP_EDITOR") or _default_editor()
    if not editor:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": "OctoMap editor binary not configured; set LINGTU_OCTOMAP_EDITOR",
        }

    before_sha = sha256_file(octomap)
    stamp = time.time_ns()
    tmp_out = octomap.with_name(f"{octomap.name}.edit-{stamp}.tmp")
    backup = octomap.with_name(f"{octomap.name}.preedit-{stamp}")
    argv = _editor_argv(
        editor,
        octomap=octomap,
        output=tmp_out,
        state=state,
        x=x,
        y=y,
        z=z,
        radius=radius,
        shape=shape,
    )
    try:
        completed = subprocess.run(
            argv,
            cwd=str(map_dir),
            capture_output=True,
            text=True,
            timeout=float(timeout_sec),
            check=False,
        )
    except FileNotFoundError as exc:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": f"OctoMap editor binary not found: {exc}",
            "argv": argv,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": f"OctoMap editor timed out after {float(timeout_sec):.3f}s",
            "stdout": _decode(exc.stdout),
            "stderr": _decode(exc.stderr),
            "argv": argv,
        }

    if completed.returncode != 0:
        _unlink_quietly(tmp_out)
        return {
            "action": "edit_voxels",
            "success": False,
            "message": completed.stderr.strip() or completed.stdout.strip() or "OctoMap editor failed",
            "returncode": int(completed.returncode),
            "stdout": completed.stdout,
            "stderr": completed.stderr,
            "argv": argv,
        }
    if not tmp_out.is_file() or tmp_out.stat().st_size <= 0:
        return {
            "action": "edit_voxels",
            "success": False,
            "message": "OctoMap editor did not write an output artifact",
            "argv": argv,
        }

    shutil.copy2(octomap, backup)
    os.replace(tmp_out, octomap)
    after_sha = sha256_file(octomap)
    editor_report = _json_object(completed.stdout)
    edit = {
        "ts": time.time(),
        "state": state,
        "effective_state": editor_report.get(
            "effective_state",
            "occupied" if state in {"occupied", "preblocked"} else "free",
        ),
        "shape": shape,
        "center": {"x": x, "y": y, "z": z},
        "radius": radius,
        "edited_voxels": int(editor_report.get("edited_voxels") or 0),
        "octomap_before_sha256": before_sha,
        "octomap_after_sha256": after_sha,
    }
    overlay_path = _append_overlay(map_dir, edit)
    _update_metadata(map_dir, octomap, before_sha=before_sha, after_sha=after_sha, edit=edit)
    return {
        "action": "edit_voxels",
        "success": True,
        "map_id": map_dir.name,
        "octomap": str(octomap),
        "backup": str(backup),
        "overlay": str(overlay_path),
        "edit": edit,
        "editor": editor_report,
    }


def _octomap_path(map_dir: Path) -> Path | None:
    for filename in ("octomap.ot", "octomap.bt"):
        path = map_dir / filename
        if path.is_file():
            return path
    return None


def _center_xyz(cmd: dict[str, Any]) -> tuple[float, float, float]:
    center = cmd.get("center")
    if isinstance(center, dict):
        x, y, z = center.get("x"), center.get("y"), center.get("z", 0.0)
    elif isinstance(center, (list, tuple)):
        if len(center) < 2:
            raise ValueError("center must contain at least x and y")
        x, y = center[0], center[1]
        z = center[2] if len(center) > 2 else 0.0
    else:
        x, y, z = cmd.get("x"), cmd.get("y"), cmd.get("z", 0.0)
    values = (float(x), float(y), float(z))
    if any(abs(value) > 500.0 for value in values):
        raise ValueError("voxel edit center is outside the 500m map safety bound")
    return values


def _radius(cmd: dict[str, Any]) -> float:
    radius = float(cmd.get("radius", 0.2))
    if radius <= 0.0 or radius > 10.0:
        raise ValueError("radius must be in (0, 10]")
    return radius


def _editor_argv(
    editor: Sequence[str] | str,
    *,
    octomap: Path,
    output: Path,
    state: str,
    x: float,
    y: float,
    z: float,
    radius: float,
    shape: str,
) -> list[str]:
    tokens = shlex.split(editor, posix=(os.name != "nt")) if isinstance(editor, str) else [str(part) for part in editor]
    values = {
        "map": str(octomap),
        "input": str(octomap),
        "output": str(output),
        "state": state,
        "x": f"{x:g}",
        "y": f"{y:g}",
        "z": f"{z:g}",
        "radius": f"{radius:g}",
        "shape": shape,
    }
    if any("{" in token and "}" in token for token in tokens):
        return [token.format(**values) for token in tokens]
    return [
        *tokens,
        "--map",
        str(octomap),
        "--output",
        str(output),
        "--state",
        state,
        "--x",
        f"{x:g}",
        "--y",
        f"{y:g}",
        "--z",
        f"{z:g}",
        "--radius",
        f"{radius:g}",
        "--shape",
        shape,
    ]


def _default_editor() -> str:
    suffix = ".exe" if os.name == "nt" else ""
    root = Path(__file__).resolve().parents[4]
    candidates = [
        root / "build" / "octoplanner3d_headless" / f"octoplanner3d_edit_octomap{suffix}",
        root
        / "src"
        / "nav"
        / "services"
        / "plan"
        / "global_planner"
        / "algorithm"
        / "OctoPlanner3D"
        / "runtime"
        / "build"
        / f"octoplanner3d_edit_octomap{suffix}",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


def _append_overlay(map_dir: Path, edit: dict[str, Any]) -> Path:
    path = map_dir / "voxel_edits.json"
    payload = _json_file(path) or {"schema_version": 1, "edits": []}
    edits = payload.setdefault("edits", [])
    if isinstance(edits, list):
        edits.append(edit)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


def _update_metadata(
    map_dir: Path,
    octomap: Path,
    *,
    before_sha: str,
    after_sha: str,
    edit: dict[str, Any],
) -> None:
    path = map_dir / "metadata.json"
    payload = _json_file(path)
    if not isinstance(payload, dict):
        return
    artifacts = payload.setdefault("artifacts", {})
    octomap_entry = artifacts.setdefault("octomap", {})
    if isinstance(octomap_entry, dict):
        octomap_entry["path"] = octomap.name
        octomap_entry["sha256"] = after_sha
        octomap_entry["manual_voxel_edit"] = True
    summary = payload.setdefault("manual_voxel_edits", {"count": 0})
    if isinstance(summary, dict):
        summary["count"] = int(summary.get("count") or 0) + 1
        summary["last_edit"] = edit
        summary["last_before_sha256"] = before_sha
        summary["last_after_sha256"] = after_sha
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _json_file(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return data if isinstance(data, dict) else None


def _json_object(text: str) -> dict[str, Any]:
    try:
        data = json.loads(str(text or "").strip() or "{}")
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _decode(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value or "")


def _unlink_quietly(path: Path) -> None:
    try:
        path.unlink()
    except OSError:
        pass
