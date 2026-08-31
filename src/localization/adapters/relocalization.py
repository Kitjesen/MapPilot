"""Native DDS saved-map relocalization adapter."""

from __future__ import annotations

import os
import subprocess
from typing import Any

from localization.service import RelocalizationResult
from localization.slam_control import (
    last_json_object as _last_json_line,
)
from localization.slam_control import (
    slam_control_binary as _control_binary,
)


def _float_or_none(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed == parsed else None


def _run_control(
    argv: list[str],
    *,
    timeout_s: float,
) -> RelocalizationResult:
    env = os.environ.copy()
    try:
        completed = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=float(timeout_s) + 5.0,
            check=False,
            env=env,
        )
    except subprocess.TimeoutExpired as exc:
        return RelocalizationResult(
            success=False,
            message=f"native SLAM relocalization timeout > {float(timeout_s):g}s",
            timed_out=True,
            stdout=exc.stdout or "",
            stderr=exc.stderr or "",
        )
    except OSError as exc:
        return RelocalizationResult(
            success=False,
            message=f"failed to run native SLAM relocalization control: {exc}",
        )

    payload = _last_json_line(completed.stdout)
    success = completed.returncode == 0 and payload.get("success") is True
    quality = _float_or_none(payload.get("relocalization_quality", payload.get("quality")))
    message = str(
        payload.get("last_relocalization_message")
        or payload.get("message")
        or completed.stderr.strip()
        or completed.stdout.strip()
        or ("relocalized" if success else "native SLAM relocalization failed")
    )
    return RelocalizationResult(
        success=success,
        message=message,
        quality=quality,
        stdout=completed.stdout,
        stderr=completed.stderr,
        returncode=completed.returncode,
        details=payload,
    )


class NativeSlamRelocalizationService:
    """RelocalizationService backed by the C++ CycloneDDS SLAM control tool."""

    def available(self) -> bool:
        try:
            _control_binary()
        except Exception:
            return False
        return True

    def trigger_global_relocalize(
        self,
        *,
        timeout_s: float = 10.0,
    ) -> RelocalizationResult:
        try:
            binary = _control_binary()
        except Exception as exc:
            return RelocalizationResult(
                success=False,
                message=f"native SLAM relocalization unavailable: {exc}",
            )
        domain_id = os.environ.get("LINGTU_DDS_DOMAIN_ID", "0").strip() or "0"
        return _run_control(
            [
                binary,
                "global-relocalize",
                "--domain-id",
                domain_id,
                "--timeout-s",
                f"{float(timeout_s):g}",
            ],
            timeout_s=timeout_s,
        )

    def query_global_relocalize_status(
        self,
        *,
        timeout_s: float = 5.0,
    ) -> RelocalizationResult:
        try:
            binary = _control_binary()
        except Exception as exc:
            return RelocalizationResult(
                success=False,
                message=f"native SLAM relocalization unavailable: {exc}",
            )
        domain_id = os.environ.get("LINGTU_DDS_DOMAIN_ID", "0").strip() or "0"
        return _run_control(
            [
                binary,
                "status",
                "--domain-id",
                domain_id,
                "--timeout-s",
                f"{float(timeout_s):g}",
            ],
            timeout_s=timeout_s,
        )

    def relocalize_saved_map(
        self,
        map_id: str,
        x: float,
        y: float,
        yaw: float,
        *,
        timeout_s: float = 30.0,
    ) -> RelocalizationResult:
        try:
            binary = _control_binary()
        except Exception as exc:
            return RelocalizationResult(
                success=False,
                message=f"native SLAM relocalization unavailable: {exc}",
            )

        domain_id = os.environ.get("LINGTU_DDS_DOMAIN_ID", "0").strip() or "0"
        argv = [
            binary,
            "relocalize",
            "--x",
            f"{float(x):g}",
            "--y",
            f"{float(y):g}",
            "--yaw",
            f"{float(yaw):g}",
            "--domain-id",
            domain_id,
            "--timeout-s",
            f"{float(timeout_s):g}",
        ]
        # ProductControl already loaded map_id into slamd; the native command
        # intentionally has no second map-selection argument.
        return _run_control(argv, timeout_s=timeout_s)

    def track_against_map(
        self,
        *,
        timeout_s: float = 10.0,
    ) -> RelocalizationResult:
        try:
            binary = _control_binary()
        except Exception as exc:
            return RelocalizationResult(
                success=False,
                message=f"native SLAM relocalization unavailable: {exc}",
            )

        domain_id = os.environ.get("LINGTU_DDS_DOMAIN_ID", "0").strip() or "0"
        return _run_control(
            [
                binary,
                "track-against-map",
                "--domain-id",
                domain_id,
                "--timeout-s",
                f"{float(timeout_s):g}",
            ],
            timeout_s=timeout_s,
        )
