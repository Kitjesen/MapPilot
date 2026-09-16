"""Regression checks for LingTu's ten maintained documentation pages."""

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DOCS_ROOT = REPOSITORY_ROOT / "docs"

DOCUMENT_NAMES = (
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
)
CURATED_PAGES = tuple(DOCS_ROOT / name for name in DOCUMENT_NAMES)
DOCUMENT_IDS = {
    "home",
    "getting-started",
    "architecture",
    "runtime",
    "simulation",
    "development",
    "operations",
    "api",
    "testing",
    "roadmap",
}

WEB_GUIDE_ENTRY = REPOSITORY_ROOT / "web" / "guide" / "index.html"
WEB_GUIDE_APP = REPOSITORY_ROOT / "web" / "src" / "guide" / "DocsApp.tsx"
WEB_GUIDE_REGISTRY = REPOSITORY_ROOT / "web" / "src" / "guide" / "docsRegistry.ts"
WEB_GUIDE_SOURCES = re.compile(r"sourcePath:\s*'([^']+)'")
WEB_GUIDE_RAW_DOC_IMPORT = re.compile(r"from '([^']*docs/[^']+)\?raw'")
WEB_GUIDE_DOCUMENT_ID = re.compile(r"^\s+id:\s*'([^']+)',", re.MULTILINE)
WEB_GUIDE_GROUP_IDS = re.compile(r"ids:\s*\[([^\]]*)\]")
WEB_GUIDE_QUOTED_ID = re.compile(r"'([^']+)'")
WEB_GUIDE_STATIC_HASH = re.compile(r'href="#([a-z][a-z0-9-]*)"')
WEB_GUIDE_NAVIGATE_ID = re.compile(r"onNavigate\('([^']+)'\)")
WEB_GUIDE_HOME_PATH_ID = re.compile(r"\{ id: '([^']+)', title:")

MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)]+)\)")
FIELD_ENDPOINT = re.compile(
    r"\b(?!127\.0\.0\.1\b)(?:\d{1,3}\.){3}\d{1,3}\b|\bnatapp\b",
    re.IGNORECASE,
)
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


def test_documentation_tree_contains_exactly_ten_root_markdown_pages() -> None:
    expected = {f"docs/{name}" for name in DOCUMENT_NAMES}
    actual = {
        path.relative_to(REPOSITORY_ROOT).as_posix()
        for path in DOCS_ROOT.rglob("*.md")
    }

    assert actual == expected
    assert all(path.parent == DOCS_ROOT for path in CURATED_PAGES)


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

    for source in CURATED_PAGES:
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
    corrupted = [
        source.relative_to(REPOSITORY_ROOT).as_posix()
        for source in CURATED_PAGES
        if "\ufffd" in source.read_text(encoding="utf-8")
    ]

    assert not corrupted, "replacement characters in curated documentation:\n" + "\n".join(corrupted)


def test_web_guide_is_static_and_catalogs_exactly_the_ten_docs() -> None:
    """Keep the public Web guide separate from robot-control runtime surfaces."""

    assert WEB_GUIDE_ENTRY.is_file(), "the /guide Vite entry is missing"
    assert WEB_GUIDE_REGISTRY.is_file(), "the Web guide source registry is missing"

    registry = WEB_GUIDE_REGISTRY.read_text(encoding="utf-8")
    expected_sources = {f"docs/{name}" for name in DOCUMENT_NAMES}
    registered_sources = set(WEB_GUIDE_SOURCES.findall(registry))
    assert registered_sources == expected_sources

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


def test_web_guide_navigation_uses_only_canonical_document_ids() -> None:
    registry = WEB_GUIDE_REGISTRY.read_text(encoding="utf-8")
    app = WEB_GUIDE_APP.read_text(encoding="utf-8")

    registry_ids = WEB_GUIDE_DOCUMENT_ID.findall(registry)
    assert len(registry_ids) == len(DOCUMENT_IDS)
    assert set(registry_ids) == DOCUMENT_IDS

    group_ids = [
        document_id
        for group in WEB_GUIDE_GROUP_IDS.findall(registry)
        for document_id in WEB_GUIDE_QUOTED_ID.findall(group)
    ]
    assert len(group_ids) == len(DOCUMENT_IDS) - 1
    assert set(group_ids) == DOCUMENT_IDS - {"home"}

    app_targets = (
        (set(WEB_GUIDE_STATIC_HASH.findall(app)) - {"main-content"})
        | set(WEB_GUIDE_NAVIGATE_ID.findall(app))
        | set(WEB_GUIDE_HOME_PATH_ID.findall(app))
    )
    assert app_targets <= DOCUMENT_IDS


def test_web_guide_bundles_only_the_ten_docs_without_field_endpoints() -> None:
    """The static guide may bundle only source material safe for public reading."""

    registry = WEB_GUIDE_REGISTRY.read_text(encoding="utf-8")
    imported_sources = {
        (WEB_GUIDE_REGISTRY.parent / relative_path).resolve()
        for relative_path in WEB_GUIDE_RAW_DOC_IMPORT.findall(registry)
    }
    assert imported_sources == set(CURATED_PAGES)

    leaked: list[str] = []
    for source in imported_sources:
        match = FIELD_ENDPOINT.search(source.read_text(encoding="utf-8"))
        if match:
            leaked.append(f"{source.relative_to(REPOSITORY_ROOT).as_posix()}: {match.group(0)}")

    assert not leaked, (
        "the static Web guide must not bundle field endpoints or tunnel names:\n"
        + "\n".join(leaked)
    )
