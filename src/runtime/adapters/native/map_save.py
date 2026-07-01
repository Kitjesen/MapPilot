"""Native SLAM map-save adapter.

The adapter sends a small command to the C++ SLAM DDS runtime.  The runtime
performs the actual function call into the active SLAM backend and writes the
map locally, so Python does not need to subscribe to point clouds or link the
DDS Python package.
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path
from typing import Any

from runtime.map_save import MapSaveError, MapSaveTimeout, MapSaveUnavailable
from runtime.registry import register


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _control_binary() -> str:
    explicit = os.environ.get("LINGTU_SLAM_CONTROL", "").strip()
    if explicit:
        return explicit

    candidates = [
        _repo_root() / "build" / "slam_core" / "lingtu_slam_control",
        Path("/opt/lingtu/current/build/slam_core/lingtu_slam_control"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    resolved = shutil.which("lingtu_slam_control")
    if resolved:
        return resolved

    raise MapSaveUnavailable(
        "native SLAM map-save control binary not found; build "
        "src/localization/slam/cpp with LINGTU_SLAM_BUILD_DDS_RUNTIME=ON "
        "or set LINGTU_SLAM_CONTROL"
    )


def _last_json_line(text: str) -> dict[str, Any]:
    for line in reversed([item.strip() for item in text.splitlines()]):
        if not line:
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            return value
    return {}


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

    def save_pgo_map(
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
            raise MapSaveTimeout(
                f"native SLAM map save timed out after {float(timeout_sec):.1f}s"
            ) from exc
        except OSError as exc:
            raise MapSaveUnavailable(f"failed to run native SLAM control: {exc}") from exc

        payload = _last_json_line(completed.stdout)
        if completed.returncode != 0:
            detail = (
                payload.get("message")
                if isinstance(payload, dict)
                else ""
            ) or completed.stderr.strip() or completed.stdout.strip()
            raise MapSaveError(
                f"native SLAM map save failed with code {completed.returncode}: {detail}"
            )

        if not payload:
            raise MapSaveError("native SLAM map save returned no JSON response")
        if payload.get("success") is not True:
            raise MapSaveError(
                str(payload.get("message") or "native SLAM map save failed")
            )

        return {
            **payload,
            "success": True,
            "source": "native_slam_dds_control",
            "command": argv[:2] + ["<map-path>", *argv[3:]],
        }
