# ruff: noqa: D103, S101
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
FUNCTIONAL_ROOTS = {
    "decision",
    "diagnostics",
    "drivers",
    "explore",
    "gateway",
    "kernels",
    "lingtu",
    "localization",
    "maps",
    "memory",
    "message",
    "native",
    "nav",
    "perception",
    "runtime",
}
SOURCE_FAMILIES = {
    "product_control": {"lingtu"},
    "capabilities": {
        "decision",
        "drivers",
        "explore",
        "localization",
        "maps",
        "memory",
        "nav",
        "perception",
    },
    "platform": {"diagnostics", "gateway", "message", "runtime"},
    "compute": {"kernels", "native"},
}


def test_src_uses_one_documented_functional_root_model() -> None:
    roots = {
        path.name
        for path in SRC.iterdir()
        if path.is_dir()
        and path.name != "__pycache__"
        and not path.name.endswith(".egg-info")
    }

    assert roots == FUNCTIONAL_ROOTS
    assert all((SRC / name / "README.md").is_file() for name in roots)

    index = (SRC / "README.md").read_text(encoding="utf-8")
    for name in FUNCTIONAL_ROOTS:
        assert f"({name}/README.md)" in index


def test_src_readme_groups_every_root_once_without_wrapper_packages() -> None:
    index = (SRC / "README.md").read_text(encoding="utf-8")
    source_families = index.split("## Source families", 1)[1].split(
        "## Runtime entries", 1
    )[0]

    assert set().union(*SOURCE_FAMILIES.values()) == FUNCTIONAL_ROOTS
    assert sum(map(len, SOURCE_FAMILIES.values())) == len(FUNCTIONAL_ROOTS)

    family_starts = {
        family: source_families.index(f"### {family}")
        for family in SOURCE_FAMILIES
    }
    ordered_families = sorted(family_starts, key=family_starts.get)
    for position, family in enumerate(ordered_families):
        start = family_starts[family]
        end = (
            family_starts[ordered_families[position + 1]]
            if position + 1 < len(ordered_families)
            else len(source_families)
        )
        section = source_families[start:end]
        for root in FUNCTIONAL_ROOTS:
            assert (f"({root}/README.md)" in section) == (
                root in SOURCE_FAMILIES[family]
            )

    for family in SOURCE_FAMILIES:
        assert not (SRC / family).exists()


def test_exploration_endpoint_is_owned_by_exploration_domain() -> None:
    endpoint = ROOT / "src/explore/cpp/endpoint"
    nav_endpoint = ROOT / "src/nav/cpp/endpoint"
    cmake = (nav_endpoint / "CMakeLists.txt").read_text(encoding="utf-8")

    assert (endpoint / "main.cpp").is_file()
    assert not (nav_endpoint / "explore").exists()
    assert 'set(_EXPLORE_ENDPOINT_DIR "${_EXPLORE_CPP_DIR}/endpoint")' in cmake
    assert "lingtu_explore_dds" in cmake


def test_removed_empty_roots_are_not_current_documented_owners() -> None:
    current_docs = (
        ROOT / "docs/03-development/README.md",
        ROOT / "docs/architecture/NAVIGATION_CAPABILITY_MATRIX.md",
    )
    text = "\n".join(path.read_text(encoding="utf-8") for path in current_docs)

    assert "src/nav/building" not in text
    assert "src/runtime/devices" not in text
