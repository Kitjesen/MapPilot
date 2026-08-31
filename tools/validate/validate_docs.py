#!/usr/bin/env python3
"""Validate first-party documentation structure and local links."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]
SCAN_ROOTS = (
    ROOT,
    ROOT / "docs",
    ROOT / "src",
    ROOT / "sim",
    ROOT / "scripts",
    ROOT / "tools",
    ROOT / "web",
    ROOT / "config",
    ROOT / "cli",
)
SKIP_PARTS = {
    ".git",
    ".github",
    ".venv",
    "3rdparty",
    "build",
    "dist",
    "node_modules",
    "third_party",
    "vendor",
    "__pycache__",
}
EXCLUDED_MARKDOWN_PREFIXES = (
    ROOT / "src" / "drivers" / "real" / "camera" / "deps",
    ROOT / "src" / "drivers" / "real" / "lidar" / "deps",
    ROOT / "tools" / "calibration" / "camera_lidar",
    ROOT / "tools" / "calibration" / "lidar_imu" / "LiDAR_IMU_Init",
)
FORBIDDEN_DOC_DIRS = (
    ROOT / "docs" / "archive",
    ROOT / "docs" / "references",
    ROOT / "docs" / "superpowers",
)
ALLOWED_PLAN_FILES = {
    "README.md",
    "current-roadmap.md",
    "message-dds-cleanup.md",
    "robot-mounted-weapon-gameplay-tdd.md",
    "sensor-noise-injection-tdd.md",
    "ue5-playable-vertical-slice.md",
}
STATUS_RE = re.compile(r"^(?:Status|状态)\s*[:：]", re.IGNORECASE)
DATE_RECORD_RE = re.compile(r"^\d{4}-\d{2}-\d{2}(?:-|\.md$)")
INLINE_LINK_RE = re.compile(r"!?\[[^\]]*\]\(\s*(?:<([^>]+)>|([^\s)]+))(?:\s+['\"][^)]*['\"])?\s*\)")
REFERENCE_LINK_RE = re.compile(r"^\s*\[[^\]]+\]:\s*(?:<([^>]+)>|([^\s]+))")
EXTERNAL_SCHEME_RE = re.compile(r"^[a-z][a-z0-9+.-]*:", re.IGNORECASE)
WINDOWS_ABSOLUTE_RE = re.compile(r"^[A-Za-z]:[/\\]")


def _repo_rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def _is_skipped(path: Path) -> bool:
    absolute = ROOT / path
    return bool(SKIP_PARTS.intersection(path.parts)) or any(
        absolute == prefix or prefix in absolute.parents for prefix in EXCLUDED_MARKDOWN_PREFIXES
    )


def markdown_files(root: Path = ROOT) -> list[Path]:
    """Return maintained first-party Markdown files without vendored content."""

    files = set(path for path in root.glob("*.md") if path.is_file())
    for scan_root in SCAN_ROOTS[1:]:
        if not scan_root.exists():
            continue
        files.update(
            path for path in scan_root.rglob("*.md") if path.is_file() and not _is_skipped(path.relative_to(root))
        )
    return sorted(path for path in files if path.exists())


def _content_without_fenced_code(text: str) -> str:
    lines: list[str] = []
    fenced = False
    marker = ""
    for line in text.splitlines():
        stripped = line.lstrip()
        if stripped.startswith(("```", "~~~")):
            current = stripped[:3]
            if not fenced:
                fenced = True
                marker = current
            elif current == marker:
                fenced = False
                marker = ""
            continue
        if not fenced:
            lines.append(line)
    return "\n".join(lines)


def _link_targets(text: str) -> list[str]:
    visible = _content_without_fenced_code(text)
    targets = [left or right for left, right in INLINE_LINK_RE.findall(visible)]
    for line in visible.splitlines():
        match = REFERENCE_LINK_RE.match(line)
        if match:
            targets.append(match.group(1) or match.group(2))
    return targets


def _local_link_path(source: Path, raw_target: str) -> Path | None:
    target = unquote(raw_target.strip())
    if not target or target.startswith(("#", "/", "//")):
        return None
    if WINDOWS_ABSOLUTE_RE.match(target) or EXTERNAL_SCHEME_RE.match(target):
        return None

    target = target.split("#", 1)[0].split("?", 1)[0]
    if not target:
        return None
    candidate = source.parent.joinpath(*Path(target.replace("\\", "/")).parts)
    if candidate.exists():
        return candidate

    line_suffix = re.match(r"^(.*?):\d+(?::\d+)?$", target)
    if line_suffix:
        return source.parent.joinpath(*Path(line_suffix.group(1)).parts)
    return candidate


def _has_status(path: Path) -> bool:
    lines = path.read_text(encoding="utf-8-sig", errors="replace").splitlines()[:12]
    return any(STATUS_RE.match(line.strip()) for line in lines)


def validate_repository(root: Path = ROOT) -> tuple[list[str], int]:
    """Return documentation violations and the number of Markdown files scanned."""

    errors: list[str] = []
    files = markdown_files(root)

    for directory in FORBIDDEN_DOC_DIRS:
        if directory.exists():
            errors.append(f"retired documentation directory exists: {_repo_rel(directory)}")

    plan_dir = root / "docs" / "plans"
    plan_files = {path.name for path in plan_dir.glob("*.md")}
    unexpected_plans = sorted(plan_files - ALLOWED_PLAN_FILES)
    if unexpected_plans:
        errors.append("unexpected active plan files: " + ", ".join(unexpected_plans))

    binary_docs = sorted(path for suffix in ("*.pdf", "*.docx") for path in (root / "docs").rglob(suffix))
    for path in binary_docs:
        errors.append(f"binary snapshot belongs outside docs or in generated artifacts: {_repo_rel(path)}")

    field_runs = root / "docs" / "07-testing" / "field-runs"
    field_run_exceptions = {"README.md"}
    for path in field_runs.glob("*.md"):
        if path.name not in field_run_exceptions and not DATE_RECORD_RE.match(path.name):
            errors.append(f"field evidence must be date-prefixed: {_repo_rel(path)}")

    for path in files:
        rel = _repo_rel(path)
        text = path.read_text(encoding="utf-8-sig", errors="replace")
        first_nonempty = next((line.strip() for line in text.splitlines() if line.strip()), "")
        if not first_nonempty.startswith("# "):
            errors.append(f"missing H1 title: {rel}")

        if (
            path.parent == root / "docs" / "architecture"
            or path.parent == root / "docs" / "product"
            or path.parent == root / "docs" / "research"
            or path.parent == root / "docs" / "plans"
        ) and not _has_status(path):
            errors.append(f"missing status near top: {rel}")

        for target in _link_targets(text):
            candidate = _local_link_path(path, target)
            if candidate is not None and not candidate.exists():
                errors.append(f"broken local link: {rel} -> {target}")

    return sorted(set(errors)), len(files)


def main(argv: list[str] | None = None) -> int:
    """Run the documentation guard as a command-line program."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    errors, scanned = validate_repository()
    if errors:
        print(f"FAILED: {len(errors)} documentation violation(s), {scanned} Markdown files scanned")
        for error in errors:
            print(f"- {error}")
        return 1

    if args.verbose:
        print("Checks: titles, statuses, local links, plan/evidence layout, retired directories")
    print(f"PASSED: {scanned} first-party Markdown files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
