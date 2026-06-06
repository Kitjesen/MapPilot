"""ROS relocalization service-call helpers.

Gateway endpoints may request relocalization, but SLAM-side code owns the ROS
service names, shell environment, subprocess execution, and output parsing.
"""

from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path
from typing import Mapping

from core.relocalization import RelocalizationResult
from core.runtime_interface import TOPICS


_ROS_SETUP = (
    "source /opt/ros/humble/setup.bash && "
    "source ~/data/SLAM/navigation/install/setup.bash 2>/dev/null; "
)
RELOCALIZE_SERVICE = TOPICS.relocalize_service
GLOBAL_RELOCALIZE_SERVICE = TOPICS.global_relocalize_service


def _timeout_message(timeout_s: float) -> str:
    return f"call timeout > {timeout_s:g}s"


def _tail_output(stdout: str, stderr: str) -> str:
    if stdout:
        return stdout[-300:]
    return stderr[-300:] or "no output"


def parse_quality(stdout: str) -> float | None:
    for line in stdout.splitlines():
        normalized = line.lower().strip()
        if not any(key in normalized for key in ("quality:", "score:", "fitness:")):
            continue
        try:
            return float(normalized.split(":", 1)[-1].strip())
        except ValueError:
            continue
    return None


def _completed_result(process: subprocess.CompletedProcess[str]) -> RelocalizationResult:
    stdout = process.stdout or ""
    stderr = process.stderr or ""
    success = "success=True" in stdout
    return RelocalizationResult(
        success=success,
        message=_tail_output(stdout, stderr),
        quality=parse_quality(stdout),
        stdout=stdout,
        stderr=stderr,
        returncode=process.returncode,
    )


def _run(args: list[str], *, timeout_s: float, env: Mapping[str, str] | None = None) -> RelocalizationResult:
    try:
        process = subprocess.run(
            args,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_s,
            env=dict(env) if env is not None else None,
        )
    except subprocess.TimeoutExpired:
        return RelocalizationResult(
            success=False,
            message=_timeout_message(timeout_s),
            timed_out=True,
        )
    return _completed_result(process)


def global_relocalize_command() -> list[str]:
    command = (
        _ROS_SETUP
        + "unset RMW_IMPLEMENTATION; "
        f"ros2 service call {GLOBAL_RELOCALIZE_SERVICE} "
        "std_srvs/srv/Trigger '{}'"
    )
    return ["bash", "-c", command]


def saved_map_relocalize_command(
    pcd_path: str | os.PathLike[str],
    x: float,
    y: float,
    yaw: float,
) -> list[str]:
    safe_pcd = shlex.quote(str(Path(pcd_path)))
    command = (
        _ROS_SETUP
        + "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
        f"ros2 service call {RELOCALIZE_SERVICE} interface/srv/Relocalize "
        f"\"{{pcd_path: {safe_pcd}, x: {float(x)}, y: {float(y)}, z: 0.0, "
        f"yaw: {float(yaw)}, pitch: 0.0, roll: 0.0}}\""
    )
    return ["bash", "-c", command]


def saved_map_relocalize_env(
    pcd_path: str | os.PathLike[str],
    x: float,
    y: float,
    yaw: float,
    *,
    base_env: Mapping[str, str] | None = None,
) -> dict[str, str]:
    env = dict(os.environ if base_env is None else base_env)
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    env["LINGTU_PCD_PATH"] = str(Path(pcd_path))
    env["LINGTU_RELOC_X"] = str(float(x))
    env["LINGTU_RELOC_Y"] = str(float(y))
    env["LINGTU_RELOC_YAW"] = str(float(yaw))
    return env


def saved_map_relocalize_env_command() -> list[str]:
    command = (
        _ROS_SETUP
        + f"ros2 service call {RELOCALIZE_SERVICE} interface/srv/Relocalize "
        '"{pcd_path: \'$LINGTU_PCD_PATH\', x: $LINGTU_RELOC_X, '
        'y: $LINGTU_RELOC_Y, z: 0.0, '
        'yaw: $LINGTU_RELOC_YAW, pitch: 0.0, roll: 0.0}"'
    )
    return ["bash", "-c", command]


def trigger_global_relocalize(*, timeout_s: float = 10.0) -> RelocalizationResult:
    return _run(global_relocalize_command(), timeout_s=timeout_s)


def relocalize_saved_map(
    pcd_path: str | os.PathLike[str],
    x: float,
    y: float,
    yaw: float,
    *,
    timeout_s: float = 30.0,
) -> RelocalizationResult:
    return _run(
        saved_map_relocalize_command(pcd_path, x, y, yaw),
        timeout_s=timeout_s,
    )


def relocalize_saved_map_with_env(
    pcd_path: str | os.PathLike[str],
    x: float,
    y: float,
    yaw: float,
    *,
    timeout_s: float = 20.0,
    base_env: Mapping[str, str] | None = None,
) -> RelocalizationResult:
    env = saved_map_relocalize_env(pcd_path, x, y, yaw, base_env=base_env)
    return _run(saved_map_relocalize_env_command(), timeout_s=timeout_s, env=env)
