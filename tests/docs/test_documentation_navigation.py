"""Regression checks for the curated documentation entry points.

The documentation tree contains design records, plans, and dated validation
evidence alongside product documentation.  These checks keep the public-facing
landing pages present and prevent their local Markdown links from silently
drifting as files are reorganized.
"""

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DOCS_ROOT = REPOSITORY_ROOT / "docs"

LANDING_PAGES = (
    DOCS_ROOT / "README.md",
    DOCS_ROOT / "01-getting-started" / "README.md",
    DOCS_ROOT / "02-concepts" / "README.md",
    DOCS_ROOT / "03-development" / "README.md",
    DOCS_ROOT / "05-guides" / "README.md",
    DOCS_ROOT / "06-operations" / "README.md",
    DOCS_ROOT / "08-reference" / "README.md",
    DOCS_ROOT / "09-integrations" / "README.md",
    DOCS_ROOT / "10-safety" / "README.md",
)

DETAILED_GUIDES = (
    DOCS_ROOT / "QUICKSTART.md",
    DOCS_ROOT / "01-getting-started" / "BUILD_GUIDE.md",
    DOCS_ROOT / "03-development" / "TROUBLESHOOTING.md",
    DOCS_ROOT / "04-deployment" / "WEB_GUIDE.md",
    DOCS_ROOT / "07-testing" / "WEB_GUIDE.md",
)

CURATED_PAGES = LANDING_PAGES + DETAILED_GUIDES

WEB_GUIDE_ENTRY = REPOSITORY_ROOT / "web" / "guide" / "index.html"
WEB_GUIDE_REGISTRY = REPOSITORY_ROOT / "web" / "src" / "guide" / "docsRegistry.ts"
WEB_GUIDE_SOURCES = re.compile(r"(?:sourcePath|contentSourcePath):\s*'([^']+)'")
WEB_GUIDE_RAW_DOC_IMPORT = re.compile(r"from '([^']*docs/[^']+)\?raw'")

MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)]+)\)")
FIELD_ENDPOINT = re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b|\bnatapp\b", re.IGNORECASE)
UNCLOSED_CODE_LINK = re.compile(r"\[[^\]\n]*`\([^\)\n]+\)")


def _local_link_path(source: Path, target: str) -> Path | None:
    """Return the target for a relative Markdown link, if it is local."""

    target = unquote(target.strip().strip("<>"))
    target = target.partition("#")[0]
    if not target or "://" in target or target.startswith(("mailto:", "tel:")):
        return None
    return (source.parent / target).resolve()


def _has_exact_local_path_case(source: Path, target: str) -> bool:
    """Check link spelling component-by-component on case-insensitive hosts."""

    target = unquote(target.strip().strip("<>"))
    target = target.partition("#")[0]
    if not target or "://" in target or target.startswith(("mailto:", "tel:")):
        return True

    current = source.parent
    for component in Path(target).parts:
        if component in ("", "."):
            continue
        if component == "..":
            current = current.parent
            continue
        if not current.is_dir() or component not in {entry.name for entry in current.iterdir()}:
            return False
        current /= component
    return True


def test_curated_documentation_landing_pages_exist() -> None:
    missing = [path.relative_to(REPOSITORY_ROOT).as_posix() for path in LANDING_PAGES if not path.is_file()]
    assert not missing, f"missing documentation landing pages: {missing}"


def test_curated_documentation_links_resolve() -> None:
    missing: list[str] = []
    case_mismatches: list[str] = []
    for source in CURATED_PAGES:
        for target in MARKDOWN_LINK.findall(source.read_text(encoding="utf-8")):
            local_target = _local_link_path(source, target)
            if local_target is not None and not local_target.exists():
                missing.append(
                    f"{source.relative_to(REPOSITORY_ROOT).as_posix()} -> {target}"
                )
            elif local_target is not None and not _has_exact_local_path_case(source, target):
                case_mismatches.append(
                    f"{source.relative_to(REPOSITORY_ROOT).as_posix()} -> {target}"
                )

    assert not missing, "broken curated documentation links:\n" + "\n".join(missing)
    assert not case_mismatches, "case-mismatched curated documentation links:\n" + "\n".join(case_mismatches)


def test_curated_documentation_pages_have_reader_metadata() -> None:
    required_markers = ("**Status:**", "**Audience:**", "**Runs on:**")

    for source in LANDING_PAGES:
        text = source.read_text(encoding="utf-8")
        assert text.startswith("# "), f"{source.relative_to(REPOSITORY_ROOT)} needs one H1 title"
        for marker in required_markers:
            assert marker in text, (
                f"{source.relative_to(REPOSITORY_ROOT)} is missing reader metadata: {marker}"
            )


def test_curated_documentation_does_not_embed_field_endpoints() -> None:
    leaked: list[str] = []
    for source in CURATED_PAGES:
        match = FIELD_ENDPOINT.search(source.read_text(encoding="utf-8"))
        if match:
            leaked.append(f"{source.relative_to(REPOSITORY_ROOT).as_posix()}: {match.group(0)}")

    assert not leaked, (
        "curated public documentation must use a target placeholder rather than a field endpoint:\n"
        + "\n".join(leaked)
    )


def test_curated_documentation_has_no_unclosed_code_links() -> None:
    malformed: list[str] = []
    for source in CURATED_PAGES:
        for line_number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), start=1):
            if UNCLOSED_CODE_LINK.search(line):
                malformed.append(f"{source.relative_to(REPOSITORY_ROOT).as_posix()}:{line_number}: {line}")

    assert not malformed, "malformed inline Markdown links:\n" + "\n".join(malformed)


def test_curated_documentation_has_no_trailing_whitespace() -> None:
    trailing: list[str] = []
    for source in CURATED_PAGES:
        for line_number, line in enumerate(source.read_text(encoding="utf-8").splitlines(), start=1):
            if line.rstrip() != line:
                trailing.append(f"{source.relative_to(REPOSITORY_ROOT).as_posix()}:{line_number}")

    assert not trailing, "trailing whitespace in curated documentation:\n" + "\n".join(trailing)


def test_curated_documentation_has_no_replacement_characters() -> None:
    corrupted: list[str] = []
    for source in CURATED_PAGES:
        if "\ufffd" in source.read_text(encoding="utf-8"):
            corrupted.append(source.relative_to(REPOSITORY_ROOT).as_posix())

    assert not corrupted, "replacement characters in curated documentation:\n" + "\n".join(corrupted)


def test_web_guide_is_static_and_catalogs_the_curated_sources() -> None:
    """Keep the public Web guide separate from robot-control runtime surfaces."""

    assert WEB_GUIDE_ENTRY.is_file(), "the /guide Vite entry is missing"
    assert WEB_GUIDE_REGISTRY.is_file(), "the Web guide source registry is missing"

    registry = WEB_GUIDE_REGISTRY.read_text(encoding="utf-8")
    registered_sources = set(WEB_GUIDE_SOURCES.findall(registry))
    expected_sources = {
        path.relative_to(REPOSITORY_ROOT).as_posix()
        for path in (*CURATED_PAGES, DOCS_ROOT / "CURRENT.md")
    }

    missing = sorted(expected_sources - registered_sources)
    assert not missing, f"curated Markdown missing from the Web guide registry: {missing}"

    guide_source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (REPOSITORY_ROOT / "web" / "src" / "guide").glob("*.ts*")
    )
    forbidden_runtime_access = (
        "fetch(",
        "WebSocket",
        "'/api/",
        '"/api/',
        "'/ws/",
        '"/ws/',
        "'/mcp/",
        '"/mcp/',
    )
    leaked_access = [token for token in forbidden_runtime_access if token in guide_source]
    assert not leaked_access, (
        "the public Web guide must not make robot-control runtime calls: " + ", ".join(leaked_access)
    )


def test_web_guide_does_not_bundle_field_endpoints() -> None:
    """The static guide may bundle only source material safe for public reading."""

    registry = WEB_GUIDE_REGISTRY.read_text(encoding="utf-8")
    imported_sources = [
        (WEB_GUIDE_REGISTRY.parent / relative_path).resolve()
        for relative_path in WEB_GUIDE_RAW_DOC_IMPORT.findall(registry)
    ]
    assert imported_sources, "the Web guide should declare its raw Markdown sources"

    missing = [path.relative_to(REPOSITORY_ROOT).as_posix() for path in imported_sources if not path.is_file()]
    assert not missing, f"missing raw Markdown bundled by the Web guide: {missing}"

    leaked: list[str] = []
    for source in imported_sources:
        match = FIELD_ENDPOINT.search(source.read_text(encoding="utf-8"))
        if match:
            leaked.append(f"{source.relative_to(REPOSITORY_ROOT).as_posix()}: {match.group(0)}")

    assert not leaked, (
        "the static Web guide must not bundle field endpoints or tunnel names:\n"
        + "\n".join(leaked)
    )
