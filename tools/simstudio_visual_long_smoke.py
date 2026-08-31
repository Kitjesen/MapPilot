from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
import traceback
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.simstudio.service.application import SimulationStudioService


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _write(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2, default=str), encoding="utf-8")
    tmp.replace(path)


def _cleanup_marker_processes(marker: str) -> None:
    try:
        command = (
            "Get-CimInstance Win32_Process -Filter \"name = 'UnrealEditor.exe' OR name = 'lingtu_mujoco_headless.exe'\" "
            f"| Where-Object {{ $_.CommandLine -match '{marker}' }} "
            "| Select-Object -ExpandProperty ProcessId"
        )
        cp = subprocess.run(
            ["powershell", "-NoProfile", "-Command", command],
            capture_output=True,
            text=True,
            timeout=15,
        )
        for line in cp.stdout.splitlines():
            pid = line.strip()
            if pid.isdigit():
                subprocess.run(
                    ["powershell", "-NoProfile", "-Command", f"Stop-Process -Id {pid} -Force"],
                    timeout=15,
                )
    except Exception:
        pass


def _wait_for_file(path: Path, *, timeout_s: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            if path.is_file() and path.stat().st_size > 0:
                return {
                    "exists": True,
                    "path": str(path),
                    "size": path.stat().st_size,
                    "waited_s": round(timeout_s - max(0.0, deadline - time.monotonic()), 3),
                }
        except OSError:
            pass
        time.sleep(0.25)
    return {"exists": path.is_file(), "path": str(path), "size": path.stat().st_size if path.exists() else 0, "waited_s": timeout_s}


def main() -> int:
    repo_root = REPO_ROOT
    out_dir = repo_root / ".manual_tmp_test" / "simstudio-visual-long-smoke"
    status_path = out_dir / "status.json"
    marker = "codex_visual_long_smoke"
    unreal_editor = Path(r"D:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe")
    if out_dir.exists():
        shutil.rmtree(out_dir)
    out_dir.mkdir(parents=True)
    _write(status_path, {"phase": "starting", "updated_at": _now(), "out_dir": str(out_dir)})
    os.environ["LINGTU_SIMSTUDIO_UNREAL_EDITOR"] = str(unreal_editor)
    os.environ["LINGTU_SIMSTUDIO_VISUAL_READY_TIMEOUT_S"] = "240"
    service = SimulationStudioService.from_repository(repo_root, artifact_root=out_dir)
    intent = {
        "schema": "lingtu.sim.session-intent.v1",
        "session": {
            "session_id": marker,
            "mujoco_version": "3.10.0",
            "seed": 20260809,
            "world": "open_field@1.0.0",
            "robots": [
                {
                    "instance_id": "cart_01",
                    "package": "omni_cart@1.0.0",
                    "sensor_rig": None,
                    "controller": "omni_cart_differential_drive@1.0.0",
                    "spawn": {
                        "position_m": [0.0, 0.0, 0.0],
                        "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                    },
                }
            ],
            "runtime": {
                "backend": "mujoco",
                "mode": "unreal",
                "required_bindings": ["physics", "visual", "control"],
            },
        },
    }
    run: dict[str, Any] | None = None
    try:
        draft = service.create_draft(intent, idempotency_key="long-visual-draft")
        bundle = service.compose_draft(draft["id"], revision=draft["revision"], idempotency_key="long-visual-compose")
        run = service.create_run(bundle_id=bundle["id"], launch_profile="visual", idempotency_key="long-visual-run")
        _write(
            status_path,
            {
                "phase": "preparing",
                "updated_at": _now(),
                "draft": draft,
                "bundle": bundle,
                "run": run,
                "state_root": str(out_dir),
            },
        )
        try:
            prepared = service.run_operation(
                "prepare",
                run["id"],
                revision=run["revision"],
                idempotency_key="long-visual-prepare",
            )
            latest = service.get_run(run["id"])
            readiness = service.run_readiness(run["id"])
            run_root = out_dir / "artifacts" / "runs" / run["id"]
            screenshot = _wait_for_file(run_root / "logs" / "visual-first-frame.png", timeout_s=30.0)
            artifacts = service.list_artifacts(run["id"])
            _write(
                status_path,
                {
                    "phase": "prepare_ok",
                    "updated_at": _now(),
                    "prepared": prepared,
                    "latest": latest,
                    "readiness": readiness,
                    "screenshot": screenshot,
                    "artifacts": artifacts,
                },
            )
            return 0
        except Exception as exc:
            latest = service.get_run(run["id"])
            artifacts = service.list_artifacts(run["id"])
            _write(
                status_path,
                {
                    "phase": "prepare_failed",
                    "updated_at": _now(),
                    "error_type": type(exc).__name__,
                    "error": str(exc),
                    "latest": latest,
                    "artifacts": artifacts,
                },
            )
            return 2
    except Exception as exc:
        _write(
            status_path,
            {
                "phase": "script_failed",
                "updated_at": _now(),
                "error_type": type(exc).__name__,
                "error": str(exc),
                "traceback": traceback.format_exc(),
            },
        )
        return 1
    finally:
        if run is not None:
            try:
                latest = service.get_run(run["id"])
                if latest.get("status") not in {"STOPPED", "FAILED"}:
                    service.run_operation(
                        "stop",
                        run["id"],
                        revision=latest["revision"],
                        idempotency_key="long-visual-stop",
                    )
            except Exception as exc:
                current = json.loads(status_path.read_text(encoding="utf-8")) if status_path.exists() else {}
                current["stop_error"] = f"{type(exc).__name__}: {exc}"
                current["updated_at"] = _now()
                _write(status_path, current)
        _cleanup_marker_processes(marker)


if __name__ == "__main__":
    raise SystemExit(main())
