#!/usr/bin/env python3
"""Build a manifest-driven Thunder Lite runtime package."""

from __future__ import annotations

import argparse
import ast
import json
import re
import shutil
import sys
from pathlib import Path, PurePosixPath
from typing import Any

import yaml

ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = ROOT_DIR / "artifacts" / "thunder-lite-package"
SUMMARY_FILENAME = "THUNDER_LITE_PACKAGE.json"

if str(ROOT_DIR) not in sys.path:
    sys.path.insert(0, str(ROOT_DIR))

from tools.validate import validate_thunder_lite_package as contract  # noqa: E402


class PackageBuildError(RuntimeError):
    """Raised when the Lite package cannot be built safely."""


def build_package(
    *,
    output_dir: Path = DEFAULT_OUTPUT_DIR,
    manifest_path: Path = contract.MANIFEST_PATH,
    force: bool = False,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Build the Thunder Lite package and return a machine-readable summary."""

    validation = contract.validate(manifest_path)
    if not validation["ok"]:
        raise PackageBuildError(
            "Thunder Lite package contract failed: "
            + "; ".join(str(item) for item in validation["blockers"])
        )

    manifest = _load_manifest(manifest_path)
    package_cfg = manifest.get("package") or {}
    include_paths = tuple(
        contract._normalize_manifest_path(path)
        for path in package_cfg.get("include_paths") or ()
    )
    exclude_paths = tuple(
        contract._normalize_manifest_path(path)
        for path in package_cfg.get("exclude_paths") or ()
    )
    omit_patterns = tuple(
        contract._normalize_manifest_path(path)
        for path in package_cfg.get("omit_paths") or ()
    )

    output_dir = output_dir.resolve()
    if not dry_run:
        _prepare_output_dir(output_dir, force=force)

    copied_files: list[str] = []
    skipped_files: list[str] = []
    for rel_path in include_paths:
        source = ROOT_DIR / rel_path
        if source.is_dir():
            for file_path in sorted(path for path in source.rglob("*") if path.is_file()):
                rel_file = contract._normalize_manifest_path(file_path.relative_to(ROOT_DIR))
                if _is_excluded(rel_file, exclude_paths) or _is_omitted(rel_file, omit_patterns):
                    skipped_files.append(rel_file)
                    continue
                copied_files.append(rel_file)
                if not dry_run:
                    _copy_file(file_path, output_dir / rel_file)
            continue

        if source.is_file():
            if _is_excluded(rel_path, exclude_paths) or _is_omitted(rel_path, omit_patterns):
                skipped_files.append(rel_path)
                continue
            copied_files.append(rel_path)
            if not dry_run:
                _copy_file(source, output_dir / rel_path)
            continue

        raise PackageBuildError(f"package include path is missing: {rel_path}")

    summary = {
        "schema_version": "lingtu.thunder_lite_package.build.v1",
        "profile": manifest.get("profile"),
        "output_dir": str(output_dir),
        "dry_run": dry_run,
        "file_count": len(copied_files),
        "skipped_count": len(skipped_files),
        "copied_files": copied_files,
        "skipped_files": skipped_files,
        "contract_checked_files": validation["checked_files"],
    }

    blockers = _audit_package_summary(
        summary,
        output_dir=output_dir,
        exclude_paths=exclude_paths,
        forbidden_markers=tuple(str(item) for item in package_cfg.get("forbidden_markers") or ()),
        dry_run=dry_run,
    )
    if blockers:
        raise PackageBuildError("; ".join(blockers))

    if not dry_run:
        _write_summary(output_dir / SUMMARY_FILENAME, summary)
    return summary


def _load_manifest(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    if not isinstance(data, dict):
        raise PackageBuildError(f"{path} must contain a YAML mapping")
    return data


def _prepare_output_dir(output_dir: Path, *, force: bool) -> None:
    _ensure_safe_output_dir(output_dir)
    if output_dir.exists():
        if not force and any(output_dir.iterdir()):
            raise PackageBuildError(
                f"output directory is not empty: {output_dir}. "
                "Use --force to replace it."
            )
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)


def _ensure_safe_output_dir(output_dir: Path) -> None:
    root = ROOT_DIR.resolve()
    if output_dir == root or output_dir == root.parent:
        raise PackageBuildError(f"refusing unsafe output directory: {output_dir}")
    for protected in ("src", "cli", "config", "scripts", "tools"):
        if output_dir == (root / protected).resolve():
            raise PackageBuildError(f"refusing to package over source directory: {output_dir}")


def _copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _is_excluded(rel_path: str, exclude_paths: tuple[str, ...]) -> bool:
    rel_path = contract._normalize_manifest_path(rel_path)
    return any(rel_path == item or rel_path.startswith(f"{item}/") for item in exclude_paths)


def _is_omitted(rel_path: str, omit_patterns: tuple[str, ...]) -> bool:
    rel_path = contract._normalize_manifest_path(rel_path)
    candidates = [rel_path]
    candidates.extend(
        contract._normalize_manifest_path(parent)
        for parent in PurePosixPath(rel_path).parents
        if str(parent) != "."
    )
    for pattern in omit_patterns:
        if not pattern:
            continue
        has_glob = any(char in pattern for char in "*?[")
        if has_glob:
            if any(PurePosixPath(candidate).match(pattern) for candidate in candidates):
                return True
            continue
        if any(candidate == pattern or candidate.startswith(f"{pattern}/") for candidate in candidates):
            return True
    return False


def _audit_package_summary(
    summary: dict[str, Any],
    *,
    output_dir: Path,
    exclude_paths: tuple[str, ...],
    forbidden_markers: tuple[str, ...],
    dry_run: bool,
) -> list[str]:
    blockers: list[str] = []
    copied_files = tuple(str(path) for path in summary.get("copied_files") or ())
    for rel_file in copied_files:
        if _is_excluded(rel_file, exclude_paths):
            blockers.append(f"excluded path copied into package: {rel_file}")

    if dry_run:
        return blockers

    forbidden_import_roots = _forbidden_import_roots(forbidden_markers)
    for file_path in sorted(path for path in output_dir.rglob("*") if path.is_file()):
        rel_file = contract._normalize_manifest_path(file_path.relative_to(output_dir))
        if _is_excluded(rel_file, exclude_paths):
            blockers.append(f"excluded path exists in output: {rel_file}")
        if file_path.suffix == ".py":
            blockers.extend(_python_import_blockers(file_path, rel_file, forbidden_import_roots))
        if file_path.suffix in {".sh", ".service"}:
            blockers.extend(_text_marker_blockers(file_path, rel_file, forbidden_markers))
    return blockers


def _forbidden_import_roots(forbidden_markers: tuple[str, ...]) -> set[str]:
    return {
        marker
        for marker in forbidden_markers
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*", marker)
    }


def _python_import_blockers(
    path: Path,
    rel_file: str,
    forbidden_import_roots: set[str],
) -> list[str]:
    try:
        tree = ast.parse(path.read_text(encoding="utf-8-sig"), filename=str(path))
    except (SyntaxError, UnicodeDecodeError) as exc:
        return [f"{rel_file}: cannot parse Python file: {exc}"]

    blockers: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            imports = [node.module] if node.module else []
        else:
            continue
        for module_name in imports:
            root = str(module_name).split(".", 1)[0]
            if root in forbidden_import_roots:
                blockers.append(f"{rel_file}: imports forbidden runtime module {module_name}")
    return blockers


def _text_marker_blockers(
    path: Path,
    rel_file: str,
    forbidden_markers: tuple[str, ...],
) -> list[str]:
    text = path.read_text(encoding="utf-8-sig", errors="ignore").lower()
    return [
        f"{rel_file}: contains forbidden deployment marker {marker!r}"
        for marker in forbidden_markers
        if marker.lower() in text
    ]


def _write_summary(path: Path, summary: dict[str, Any]) -> None:
    serializable = dict(summary)
    serializable["copied_files"] = sorted(serializable["copied_files"])
    serializable["skipped_files"] = sorted(serializable["skipped_files"])
    path.write_text(
        json.dumps(serializable, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build a Thunder Lite runtime package")
    parser.add_argument(
        "--manifest",
        default=str(contract.MANIFEST_PATH),
        help="Path to config/thunder_lite_package.yaml",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Output directory for the generated package",
    )
    parser.add_argument("--force", action="store_true", help="Replace an existing output directory")
    parser.add_argument("--dry-run", action="store_true", help="Validate and report without copying files")
    parser.add_argument("--json", action="store_true", help="Print machine-readable JSON")
    args = parser.parse_args(argv)

    try:
        summary = build_package(
            output_dir=Path(args.output),
            manifest_path=Path(args.manifest),
            force=args.force,
            dry_run=args.dry_run,
        )
    except PackageBuildError as exc:
        if args.json:
            print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False, indent=2))
        else:
            print(f"Thunder Lite package build: FAIL\n  ERROR: {exc}")
        return 1

    if args.json:
        payload = {"ok": True, **summary}
        print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        mode = "dry run" if summary["dry_run"] else "built"
        print(
            f"Thunder Lite package {mode}: {summary['output_dir']} "
            f"({summary['file_count']} file(s), {summary['skipped_count']} skipped)"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
