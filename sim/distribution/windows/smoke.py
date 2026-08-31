"""Verified launch inputs for a packaged RobotSimUE smoke run."""

from __future__ import annotations

import argparse
import json
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from sim.runtime.coordinator import CoordinatorError, RunAllocationError
from sim.runtime.coordinator.live_visual import (
    create_live_visual_launch,
    run_live_visual,
)

from .core import DistributionError

_DISTRIBUTION_SCHEMA = "lingtu.sim.windows-distribution-manifest.v1"
_LAUNCHER_ARTIFACT = "package/Windows/RobotSimUE.exe"
_SHIPPING_EXECUTABLE_ARTIFACT = (
    "package/Windows/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Shipping.exe"
)
_RUNTIME_EXECUTABLE_ARTIFACT = (
    "package/Windows/RobotSimUE/Binaries/Win64/RobotSimUE-Win64-Release.exe"
)


@dataclass(frozen=True)
class VerifiedWindowsRelease:
    """One packaged release with the required Windows runtime files."""

    manifest_path: Path
    release_root: Path
    launcher: Path
    shipping_executable: Path
    executable: Path
    manifest: Mapping[str, Any]


@dataclass(frozen=True)
class WindowsPackagedSmokeConfig:
    """Trusted local inputs for one bounded packaged-runtime acceptance run."""

    manifest_path: Path
    bundle_dir: Path
    repo_root: Path
    run_root: Path
    mujoco_host: Path
    run_id: str | None = None
    map_name: str | None = None
    motion_camera_stable_id: str | None = None
    snapshot_port: int = 25125
    frames: int = 12
    steps_per_frame: int = 8
    ready_timeout_s: float = 180.0
    screenshot_timeout_s: float = 90.0


def _safe_artifact_path(value: object, *, context: str) -> str:
    if not isinstance(value, str) or not value:
        raise DistributionError(f"{context} must be a non-empty relative path")
    if (
        "\\" in value
        or value.startswith("/")
        or ":" in value
        or any(part in {"", ".", ".."} for part in value.split("/"))
    ):
        raise DistributionError(f"{context} is not a safe package-relative POSIX path")
    return value


def resolve_verified_windows_release(manifest_path: Path) -> VerifiedWindowsRelease:
    """Resolve the required files from a completed package manifest."""

    source_path = Path(manifest_path)
    if source_path.is_symlink():
        raise DistributionError(f"packaged distribution manifest is missing or unsafe: {source_path}")
    path = source_path.resolve()
    if path.name != "distribution.manifest.json" or not path.is_file():
        raise DistributionError(f"packaged distribution manifest is missing or unsafe: {path}")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise DistributionError(f"packaged distribution manifest is invalid: {exc}") from exc
    if not isinstance(raw, dict) or raw.get("schema") != _DISTRIBUTION_SCHEMA:
        raise DistributionError("unsupported packaged distribution manifest schema")
    if raw.get("state") != "packaged":
        raise DistributionError("Windows distribution is not in packaged state")
    release_root = path.parent.resolve()
    product = raw.get("product")
    if not isinstance(product, dict) or product.get("id") != "robotsimue":
        raise DistributionError("Windows distribution is not the RobotSimUE product")
    claims = raw.get("claims")
    required_claims = (
        "cook_completed",
        "stage_completed",
        "package_completed",
        "shipping_build_produced",
    )
    if not isinstance(claims, dict) or any(claims.get(name) is not True for name in required_claims):
        raise DistributionError("Windows distribution does not contain completed package claims")
    artifacts = raw.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        raise DistributionError("Windows distribution contains no packaged artifacts")

    seen: set[str] = set()
    launcher: Path | None = None
    shipping_executable: Path | None = None
    runtime_executable: Path | None = None
    for index, item in enumerate(artifacts):
        context = f"artifacts[{index}]"
        if not isinstance(item, dict) or set(item) != {"path", "bytes"}:
            raise DistributionError(f"{context} must contain path and bytes")
        relative = _safe_artifact_path(item["path"], context=f"{context}.path")
        if relative in seen:
            raise DistributionError(f"Windows distribution repeats artifact path: {relative}")
        seen.add(relative)
        expected_size = item["bytes"]
        if (
            isinstance(expected_size, bool)
            or not isinstance(expected_size, int)
            or expected_size < 0
        ):
            raise DistributionError(f"{context}.bytes must be a non-negative integer")
        artifact_path = release_root / relative
        if artifact_path.is_symlink():
            raise DistributionError(f"packaged artifact is missing or unsafe: {relative}")
        artifact = artifact_path.resolve()
        try:
            artifact.relative_to(release_root)
        except ValueError as exc:
            raise DistributionError(f"packaged artifact escapes release root: {relative}") from exc
        if not artifact.is_file():
            raise DistributionError(f"packaged artifact is missing or unsafe: {relative}")
        actual_size = artifact.stat().st_size
        if actual_size != expected_size:
            raise DistributionError(f"packaged artifact does not match manifest: {relative}")
        if relative == _LAUNCHER_ARTIFACT:
            launcher = artifact
        elif relative == _SHIPPING_EXECUTABLE_ARTIFACT:
            shipping_executable = artifact
        elif relative == _RUNTIME_EXECUTABLE_ARTIFACT:
            runtime_executable = artifact

    if launcher is None:
        raise DistributionError(f"Windows distribution omits {_LAUNCHER_ARTIFACT}")
    if shipping_executable is None:
        raise DistributionError(
            f"Windows distribution omits {_SHIPPING_EXECUTABLE_ARTIFACT}"
        )
    if runtime_executable is None:
        raise DistributionError(
            f"Windows distribution omits {_RUNTIME_EXECUTABLE_ARTIFACT}"
        )
    with launcher.open("rb") as stream:
        if stream.read(2) != b"MZ":
            raise DistributionError("packaged RobotSimUE.exe is not a Windows PE executable")
    with shipping_executable.open("rb") as stream:
        if stream.read(2) != b"MZ":
            raise DistributionError(
                "packaged RobotSimUE Shipping build is not a Windows PE executable"
            )
    with runtime_executable.open("rb") as stream:
        if stream.read(2) != b"MZ":
            raise DistributionError(
                "packaged RobotSimUE product runtime is not a Windows PE executable"
            )
    if shipping_executable.stat().st_size != runtime_executable.stat().st_size:
        raise DistributionError(
            "packaged RobotSimUE product runtime does not match the Shipping build"
        )
    return VerifiedWindowsRelease(
        manifest_path=path,
        release_root=release_root,
        launcher=launcher,
        shipping_executable=shipping_executable,
        executable=runtime_executable,
        manifest=raw,
    )


def _canonical_json(value: object) -> bytes:
    return (
        json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        + "\n"
    ).encode("utf-8")


def run_windows_packaged_smoke(config: WindowsPackagedSmokeConfig) -> dict[str, Any]:
    """Run MuJoCo plus the verified packaged executable and commit evidence."""

    release = resolve_verified_windows_release(config.manifest_path)
    run_id = config.run_id or f"windows-package-{uuid.uuid4().hex[:12]}"
    evidence_path = (
        Path(config.run_root).resolve()
        / run_id
        / "logs"
        / "windows-package-smoke.evidence.json"
    )
    try:
        evidence_path.unlink(missing_ok=True)
    except OSError as exc:
        raise DistributionError(f"cannot clear stale packaged smoke evidence: {exc}") from exc

    launch = create_live_visual_launch(
        bundle=config.bundle_dir,
        repo_root=Path(config.repo_root).resolve(),
        run_root=Path(config.run_root).resolve(),
        mujoco_host=Path(config.mujoco_host).resolve(),
        packaged_executable=release.executable,
        map_name=config.map_name,
        gate="visual-applied",
        snapshot_port=config.snapshot_port,
        run_id=run_id,
        motion_camera_stable_id=config.motion_camera_stable_id,
        ready_timeout_s=config.ready_timeout_s,
        screenshot_timeout_s=config.screenshot_timeout_s,
    )
    runtime_result = run_live_visual(
        launch,
        gate="visual-applied",
        frames=config.frames,
        steps_per_frame=config.steps_per_frame,
    )
    screenshot = Path(str(runtime_result["first_frame_screenshot"])).resolve()
    evidence = {
        "schema": "lingtu.sim.windows-package-smoke-evidence.v1",
        "distribution_manifest": {
            "path": str(release.manifest_path),
        },
        "launcher": {
            "path": str(release.launcher),
            "bytes": release.launcher.stat().st_size,
        },
        "shipping_executable": {
            "path": str(release.shipping_executable),
            "bytes": release.shipping_executable.stat().st_size,
        },
        "executable": {
            "path": str(release.executable),
            "bytes": release.executable.stat().st_size,
        },
        "session_id": launch.coordinator.plan.session_id,
        "model_generation": launch.coordinator.plan.model_generation,
        "reset_generation": launch.coordinator.plan.reset_generation,
        "run_id": run_id,
        "runtime": runtime_result,
        "screenshot": {
            "path": str(screenshot),
            "bytes": screenshot.stat().st_size,
        },
    }
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = evidence_path.with_suffix(evidence_path.suffix + ".tmp")
    temporary.write_bytes(_canonical_json(evidence))
    temporary.replace(evidence_path)
    return {**runtime_result, "smoke_evidence": str(evidence_path)}


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Verify and launch one packaged RobotSimUE SessionBundle smoke run."
    )
    parser.add_argument("manifest", type=Path)
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument(
        "--run-root",
        type=Path,
        default=Path("build/distribution/windows/smoke-runs"),
    )
    parser.add_argument(
        "--mujoco-host",
        type=Path,
        default=Path(
            "build/mujoco-runtime-physics-win/Release/lingtu_mujoco_headless.exe"
        ),
    )
    parser.add_argument("--run-id")
    parser.add_argument("--map", dest="map_name")
    parser.add_argument("--motion-camera-stable-id")
    parser.add_argument("--snapshot-port", type=int, default=25125)
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--steps-per-frame", type=int, default=8)
    parser.add_argument("--ready-timeout-s", type=float, default=180.0)
    parser.add_argument("--screenshot-timeout-s", type=float, default=90.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        result = run_windows_packaged_smoke(
            WindowsPackagedSmokeConfig(
                manifest_path=args.manifest,
                bundle_dir=args.bundle,
                repo_root=args.repo_root,
                run_root=args.run_root,
                mujoco_host=args.mujoco_host,
                run_id=args.run_id,
                map_name=args.map_name,
                motion_camera_stable_id=args.motion_camera_stable_id,
                snapshot_port=args.snapshot_port,
                frames=args.frames,
                steps_per_frame=args.steps_per_frame,
                ready_timeout_s=args.ready_timeout_s,
                screenshot_timeout_s=args.screenshot_timeout_s,
            )
        )
    except (CoordinatorError, DistributionError, OSError, RunAllocationError, ValueError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False), file=sys.stderr)
        return 1
    print(json.dumps({"ok": True, **result}, ensure_ascii=False, indent=2))
    return 0


__all__ = [
    "VerifiedWindowsRelease",
    "WindowsPackagedSmokeConfig",
    "resolve_verified_windows_release",
    "run_windows_packaged_smoke",
]


if __name__ == "__main__":
    raise SystemExit(main())
