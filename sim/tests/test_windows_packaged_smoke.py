# ruff: noqa: S101

from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from sim.distribution.windows import DistributionError, resolve_verified_windows_release
from sim.distribution.windows import smoke as smoke_module


def _release(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    root = tmp_path / "release"
    launcher = root / "package" / "Windows" / "RobotSimUE.exe"
    shipping_executable = (
        root
        / "package"
        / "Windows"
        / "RobotSimUE"
        / "Binaries"
        / "Win64"
        / "RobotSimUE-Win64-Shipping.exe"
    )
    runtime_executable = shipping_executable.with_name("RobotSimUE-Win64-Release.exe")
    launcher_data = b"MZ-packaged-robotsimue-launcher"
    runtime_data = b"MZ-packaged-robotsimue-runtime"
    launcher.parent.mkdir(parents=True)
    shipping_executable.parent.mkdir(parents=True)
    launcher.write_bytes(launcher_data)
    shipping_executable.write_bytes(runtime_data)
    runtime_executable.write_bytes(runtime_data)
    manifest = {
        "schema": "lingtu.sim.windows-distribution-manifest.v1",
        "state": "packaged",
        "product": {"id": "robotsimue"},
        "claims": {
            "cook_completed": True,
            "stage_completed": True,
            "package_completed": True,
            "shipping_build_produced": True,
            "packaged_smoke_passed": False,
        },
        "artifacts": [
            {
                "path": "package/Windows/RobotSimUE.exe",
                "bytes": len(launcher_data),
            },
            {
                "path": (
                    "package/Windows/RobotSimUE/Binaries/Win64/"
                    "RobotSimUE-Win64-Shipping.exe"
                ),
                "bytes": len(runtime_data),
            },
            {
                "path": (
                    "package/Windows/RobotSimUE/Binaries/Win64/"
                    "RobotSimUE-Win64-Release.exe"
                ),
                "bytes": len(runtime_data),
            },
        ],
    }
    manifest_path = root / "distribution.manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    return manifest_path, launcher, shipping_executable, runtime_executable


def test_packaged_release_verifies_manifest_and_executable(tmp_path: Path) -> None:
    manifest, launcher, shipping_executable, runtime_executable = _release(tmp_path)

    release = resolve_verified_windows_release(manifest)

    assert release.launcher == launcher.resolve()
    assert release.shipping_executable == shipping_executable.resolve()
    assert release.executable == runtime_executable.resolve()


def test_packaged_release_rejects_wrong_executable_size(tmp_path: Path) -> None:
    manifest, _, _, runtime_executable = _release(tmp_path)
    runtime_executable.write_bytes(b"MZ-tampered")

    with pytest.raises(DistributionError, match="does not match manifest"):
        resolve_verified_windows_release(manifest)


def test_packaged_release_rejects_unpublished_manifest(
    tmp_path: Path,
) -> None:
    manifest, _, _, _ = _release(tmp_path)
    raw = json.loads(manifest.read_text(encoding="utf-8"))
    raw["state"] = "planned"
    manifest.write_text(json.dumps(raw), encoding="utf-8")
    with pytest.raises(DistributionError, match="not in packaged state"):
        resolve_verified_windows_release(manifest)

def test_packaged_smoke_commits_separate_evidence_without_mutating_release(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    manifest, _, shipping_executable, runtime_executable = _release(tmp_path)
    manifest_before = manifest.read_bytes()
    run_root = tmp_path / "runs"
    screenshot = run_root / "smoke-run" / "logs" / "visual-first-frame.png"
    observed: dict[str, Any] = {}

    def fake_create_live_visual_launch(**kwargs: Any) -> Any:
        observed.update(kwargs)
        return SimpleNamespace(
            coordinator=SimpleNamespace(
                plan=SimpleNamespace(
                    session_id="b" * 64,
                    model_generation=4,
                    reset_generation=2,
                )
            )
        )

    def fake_run_live_visual(launch: Any, **kwargs: Any) -> dict[str, Any]:
        observed["run"] = kwargs
        screenshot.parent.mkdir(parents=True)
        screenshot.write_bytes(b"\x89PNG\r\n\x1a\npackaged-frame")
        return {
            "run_id": "smoke-run",
            "frames": 3,
            "first_frame_screenshot": str(screenshot),
        }

    monkeypatch.setattr(smoke_module, "create_live_visual_launch", fake_create_live_visual_launch)
    monkeypatch.setattr(smoke_module, "run_live_visual", fake_run_live_visual)

    result = smoke_module.run_windows_packaged_smoke(
        smoke_module.WindowsPackagedSmokeConfig(
            manifest_path=manifest,
            bundle_dir=tmp_path / "bundle",
            repo_root=tmp_path,
            run_root=run_root,
            mujoco_host=tmp_path / "lingtu_mujoco_headless.exe",
            run_id="smoke-run",
            frames=3,
            steps_per_frame=5,
        )
    )

    assert observed["packaged_executable"] == runtime_executable.resolve()
    assert observed["gate"] == "visual-applied"
    assert observed["run"] == {
        "gate": "visual-applied",
        "frames": 3,
        "steps_per_frame": 5,
    }
    assert manifest.read_bytes() == manifest_before
    evidence_path = Path(result["smoke_evidence"])
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    assert evidence["schema"] == "lingtu.sim.windows-package-smoke-evidence.v1"
    assert evidence["session_id"] == "b" * 64
    assert evidence["shipping_executable"]["path"] == str(
        shipping_executable.resolve()
    )
    assert evidence["executable"]["path"] == str(runtime_executable.resolve())
    assert evidence["screenshot"]["bytes"] == screenshot.stat().st_size
