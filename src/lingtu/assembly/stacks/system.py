"""System service stack helpers for full-stack composition."""

from __future__ import annotations


def run_startup_preflight(
    *,
    enable_semantic: bool,
    slam_profile: str,
) -> None:
    from runtime.utils.calibration_check import run_calibration_check

    needs_camera = enable_semantic
    needs_slam = slam_profile not in ("", "none")
    calib = run_calibration_check(
        require_camera=needs_camera,
        require_slam=needs_slam,
    )
    if not calib.ok:
        raise RuntimeError(f"Calibration self-check failed ({len(calib.errors)} error(s)): " + "; ".join(calib.errors))
