#!/usr/bin/env python3
"""Build LingTu portable Rust kernel artifacts.

The script is intentionally small and cross-platform so Windows developer
machines and Linux/S100P deployment flows use the same Rust build entrypoint.
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


SCHEMA = "lingtu.rust_kernel_build.v1"
ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class RustKernelTarget:
    key: str
    manifest: Path
    description: str
    required_binaries: tuple[str, ...] = ()
    required_libraries: tuple[str, ...] = ()


def _exe_name(name: str) -> str:
    return f"{name}.exe" if sys.platform.startswith("win") else name


RUST_KERNEL_TARGETS: tuple[RustKernelTarget, ...] = (
    RustKernelTarget(
        key="path_safety",
        manifest=ROOT / "src" / "kernels" / "nav" / "path_safety" / "Cargo.toml",
        description="2D path safety C ABI kernel",
        required_libraries=("lingtu_path_safety",),
    ),
    RustKernelTarget(
        key="pose_graph_opt",
        manifest=ROOT / "src" / "kernels" / "slam" / "pose_graph_opt" / "Cargo.toml",
        description="SE3 pose graph optimizer used by the PGO/HBA migration surface",
        required_libraries=("lingtu_pose_graph_opt",),
    ),
    RustKernelTarget(
        key="gpmp_trajectory_optimizer",
        manifest=ROOT
        / "src"
        / "kernels"
        / "planning"
        / "gpmp_trajectory_optimizer"
        / "Cargo.toml",
        description="PCT/GPMP trajectory optimizer binary/library used by rust_process runtime",
        required_binaries=("gpmp_optimize",),
        required_libraries=("lingtu_gpmp_trajectory_optimizer",),
    ),
    RustKernelTarget(
        key="camera_lidar_optimizer",
        manifest=ROOT
        / "src"
        / "kernels"
        / "calibration"
        / "camera_lidar_optimizer"
        / "Cargo.toml",
        description="Camera-LiDAR CT-ICP/CT-GICP optimizer C ABI",
        required_libraries=("lingtu_camera_lidar_optimizer",),
    ),
)


def target_by_key() -> dict[str, RustKernelTarget]:
    return {target.key: target for target in RUST_KERNEL_TARGETS}


def select_targets(names: list[str] | None) -> list[RustKernelTarget]:
    requested = names or ["all"]
    if "all" in requested:
        return list(RUST_KERNEL_TARGETS)
    known = target_by_key()
    selected: list[RustKernelTarget] = []
    for name in requested:
        if name not in known:
            raise KeyError(f"unknown Rust kernel target {name!r}")
        if known[name] not in selected:
            selected.append(known[name])
    return selected


def cargo_command(target: RustKernelTarget, *, release: bool) -> list[str]:
    command = ["cargo", "build", "--manifest-path", str(target.manifest)]
    if release:
        command.append("--release")
    return command


def expected_binary_paths(target: RustKernelTarget, *, release: bool) -> list[Path]:
    profile = "release" if release else "debug"
    target_dir = target.manifest.parent / "target" / profile
    return [target_dir / _exe_name(name) for name in target.required_binaries]


def expected_library_candidate_groups(target: RustKernelTarget, *, release: bool) -> list[list[Path]]:
    profile = "release" if release else "debug"
    target_dir = target.manifest.parent / "target" / profile
    groups: list[list[Path]] = []
    for name in target.required_libraries:
        if sys.platform.startswith("win"):
            groups.append(
                [
                    target_dir / f"{name}.lib",
                    target_dir / f"{name}.dll",
                    target_dir / f"{name}.dll.lib",
                    target_dir / f"lib{name}.a",
                ]
            )
        elif sys.platform == "darwin":
            groups.append([target_dir / f"lib{name}.dylib", target_dir / f"lib{name}.a"])
        else:
            groups.append([target_dir / f"lib{name}.so", target_dir / f"lib{name}.a"])
    return groups


def build_target(
    target: RustKernelTarget,
    *,
    release: bool,
    dry_run: bool = False,
) -> dict[str, Any]:
    if not target.manifest.is_file():
        raise FileNotFoundError(f"missing Rust kernel manifest: {target.manifest}")
    command = cargo_command(target, release=release)
    if not dry_run:
        if shutil.which("cargo") is None:
            raise RuntimeError("cargo is required to build LingTu Rust kernels")
        subprocess.run(command, cwd=ROOT, check=True)
    binary_artifacts = expected_binary_paths(target, release=release)
    library_candidate_groups = expected_library_candidate_groups(target, release=release)
    required_binaries_present = all(path.is_file() for path in binary_artifacts)
    required_libraries_present = all(
        any(candidate.is_file() for candidate in candidates)
        for candidates in library_candidate_groups
    )
    return {
        "key": target.key,
        "description": target.description,
        "manifest": str(target.manifest),
        "command": command,
        "required_binaries": [str(path) for path in binary_artifacts],
        "required_binaries_present": required_binaries_present,
        "required_library_candidates": [
            [str(path) for path in candidates] for candidates in library_candidate_groups
        ],
        "required_libraries_present": required_libraries_present,
        "required_artifacts_present": required_binaries_present and required_libraries_present,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    choices = ["all", *(target.key for target in RUST_KERNEL_TARGETS)]
    parser.add_argument(
        "--target",
        action="append",
        choices=choices,
        help="Rust kernel target to build. Repeatable. Defaults to all.",
    )
    parser.add_argument(
        "--release",
        action="store_true",
        help="Build release artifacts. Recommended for runtime/deployment use.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print commands without running cargo")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        selected = select_targets(args.target)
        results = [
            build_target(target, release=bool(args.release), dry_run=bool(args.dry_run))
            for target in selected
        ]
        payload = {
            "schema": SCHEMA,
            "ok": bool(args.dry_run)
            or all(item["required_artifacts_present"] for item in results),
            "release": bool(args.release),
            "dry_run": bool(args.dry_run),
            "targets": results,
        }
    except Exception as exc:  # noqa: BLE001 - CLI should return structured failure.
        payload = {
            "schema": SCHEMA,
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        status = "OK" if payload["ok"] else "FAILED"
        print(f"Rust kernel build: {status}")
        for item in payload.get("targets", []):
            print(f"  {item['key']}: {' '.join(item['command'])}")
            for artifact in item["required_binaries"]:
                print(f"    required: {artifact}")
            for candidates in item["required_library_candidates"]:
                print(f"    required one of: {', '.join(candidates)}")
        if payload.get("error"):
            print(f"  error: {payload['error']}", file=sys.stderr)
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
