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
DOCS_MARKDOWN = {
    "README.md",
    "getting-started.md",
    "architecture.md",
    "runtime.md",
    "simulation.md",
    "development.md",
    "operations.md",
    "api.md",
    "testing.md",
    "roadmap.md",
}
METADATA_RE = {
    "Status": re.compile(r"^\*\*Status:\*\*\s+\S", re.IGNORECASE),
    "Audience": re.compile(r"^\*\*Audience:\*\*\s+\S", re.IGNORECASE),
    "Runs on": re.compile(r"^\*\*Runs on:\*\*\s+\S", re.IGNORECASE),
}
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


def _missing_metadata(path: Path) -> list[str]:
    lines = path.read_text(encoding="utf-8-sig", errors="replace").splitlines()[:12]
    return [
        name
        for name, pattern in METADATA_RE.items()
        if not any(pattern.match(line.strip()) for line in lines)
    ]


def validate_repository(root: Path = ROOT) -> tuple[list[str], int]:
    """Return documentation violations and the number of Markdown files scanned."""

    errors: list[str] = []
    files = markdown_files(root)

    docs_root = root / "docs"
    docs_markdown = {
        path.relative_to(docs_root).as_posix()
        for path in docs_root.rglob("*.md")
        if path.is_file()
    }
    missing_docs = sorted(DOCS_MARKDOWN - docs_markdown)
    unexpected_docs = sorted(docs_markdown - DOCS_MARKDOWN)
    if missing_docs:
        errors.append("missing maintained docs: " + ", ".join(missing_docs))
    if unexpected_docs:
        errors.append("unexpected docs Markdown: " + ", ".join(unexpected_docs))

    binary_docs = sorted(path for suffix in ("*.pdf", "*.docx") for path in (root / "docs").rglob(suffix))
    for path in binary_docs:
        errors.append(f"binary snapshot belongs outside docs or in generated artifacts: {_repo_rel(path)}")

    for path in files:
        rel = _repo_rel(path)
        text = path.read_text(encoding="utf-8-sig", errors="replace")
        first_nonempty = next((line.strip() for line in text.splitlines() if line.strip()), "")
        if not first_nonempty.startswith("# "):
            errors.append(f"missing H1 title: {rel}")

        if path.parent == docs_root and path.name in DOCS_MARKDOWN:
            missing_metadata = _missing_metadata(path)
            if missing_metadata:
                errors.append(
                    f"missing metadata near top: {rel} -> "
                    + ", ".join(missing_metadata)
                )

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
        print("Checks: flat ten-page docs set, metadata, titles, and local links")
    print(f"PASSED: {scanned} first-party Markdown files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
