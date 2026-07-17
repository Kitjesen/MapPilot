"""Native SLAM map-save adapter.

The adapter sends a small command to the C++ SLAM DDS runtime.  The runtime
performs the actual function call into the active SLAM backend and writes the
map locally, so Python does not need to subscribe to point clouds or link the
DDS Python package.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import Any

from localization.slam_control import last_json_object, slam_control_binary
from maps.map_save import MapSaveError, MapSaveTimeout, MapSaveUnavailable
from runtime.registry import register


def _control_binary() -> str:
    try:
        return slam_control_binary()
    except FileNotFoundError as exc:
        raise MapSaveUnavailable(str(exc)) from exc


def _last_json_line(text: str) -> dict[str, Any]:
    return last_json_object(text)


def _pcd_point_count(path: Path) -> int:
    try:
        with path.open("rb") as fh:
            for raw in fh:
                line = raw.decode("ascii", errors="ignore").strip()
                upper = line.upper()
                if upper.startswith("POINTS"):
                    parts = line.split()
                    if len(parts) >= 2:
                        return int(parts[1])
                if upper.startswith("DATA"):
                    break
    except Exception:
        return 0
    return 0


def _normalize_output_ownership(path: Path) -> dict[str, Any]:
    """Best-effort fix for root-owned artifacts emitted by native helpers."""
    if os.name != "posix":
        return {"ownership_normalized": False, "ownership_note": "non_posix"}
    root = path.parent
    uid = os.getuid()
    gid = os.getgid()
    try:
        stat = root.stat()
    except OSError as exc:
        return {
            "ownership_normalized": False,
            "ownership_warning": f"stat failed: {exc}",
        }
    if stat.st_uid == uid and os.access(root, os.W_OK):
        patches = root / "patches"
        if not patches.exists() or os.access(patches, os.W_OK):
            return {"ownership_normalized": True, "ownership_note": "already_owned"}

    chown = shutil.which("chown")
    sudo = shutil.which("sudo")
    if not chown:
        return {
            "ownership_normalized": False,
            "ownership_warning": "chown not available",
        }
    argv = [chown, "-R", f"{uid}:{gid}", str(root)]
    if uid != 0 and sudo:
        argv = [sudo, "-n", *argv]
    try:
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=15.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "ownership_normalized": False,
            "ownership_warning": f"chown failed: {exc}",
        }
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        return {
            "ownership_normalized": False,
            "ownership_warning": f"chown exit {completed.returncode}: {detail}",
        }
    return {"ownership_normalized": True, "ownership_note": "chown_applied"}


@register(
    "map_save_adapter",
    "native_slam",
    description="ROS-free C++ SLAM map-save command adapter",
)
class NativeSlamMapSaveAdapter:
    """Map-save adapter backed by the C++ SLAM runtime control tool."""

    def save_nav_map(
        self,
        pcd_path: str | Path,
        *,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        return self._save(pcd_path, timeout_sec=timeout_sec)

    def save_slam_map(
        self,
        file_path: str | Path,
        *,
        save_patches: bool = True,
        timeout_sec: float = 30.0,
    ) -> dict[str, Any]:
        return self._save(file_path, timeout_sec=timeout_sec)

    def _save(self, path: str | Path, *, timeout_sec: float) -> dict[str, Any]:
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        binary = _control_binary()
        domain_id = os.environ.get("LINGTU_DDS_DOMAIN_ID", "0").strip() or "0"
        argv = [
            binary,
            "save-map",
            str(target),
            "--domain-id",
            domain_id,
            "--timeout-s",
            f"{float(timeout_sec):g}",
        ]
        try:
            completed = subprocess.run(
                argv,
                capture_output=True,
                text=True,
                timeout=float(timeout_sec) + 5.0,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise MapSaveTimeout(f"native SLAM map save timed out after {float(timeout_sec):.1f}s") from exc
        except OSError as exc:
            raise MapSaveUnavailable(f"failed to run native SLAM control: {exc}") from exc

        payload = _last_json_line(completed.stdout)
        if completed.returncode != 0:
            detail = (
                (payload.get("message") if isinstance(payload, dict) else "")
                or completed.stderr.strip()
                or completed.stdout.strip()
            )
            raise MapSaveError(f"native SLAM map save failed with code {completed.returncode}: {detail}")

        if not payload:
            raise MapSaveError("native SLAM map save returned no JSON response")
        if payload.get("success") is not True:
            raise MapSaveError(str(payload.get("message") or "native SLAM map save failed"))
        if not target.is_file():
            raise MapSaveError(f"native SLAM map save reported success but did not write {target}")
        point_count = _pcd_point_count(target)
        if point_count <= 0:
            raise MapSaveError(f"native SLAM map save wrote empty PCD at {target}")
        ownership = _normalize_output_ownership(target)

        return {
            **payload,
            "success": True,
            "source": "native_slam_dds_control",
            "point_count": point_count,
            **ownership,
            "command": argv[:2] + ["<map-path>", *argv[3:]],
        }
