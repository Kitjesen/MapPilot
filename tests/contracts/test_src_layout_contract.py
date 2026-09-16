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
    "Product control": {"lingtu"},
    "Capability": {
        "decision",
        "drivers",
        "explore",
        "localization",
        "maps",
        "memory",
        "nav",
        "perception",
    },
    "Runtime platform": {"diagnostics", "gateway", "message", "runtime"},
    "Shared compute": {"kernels"},
    "Shared native": {"native"},
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


def test_src_readme_assigns_every_root_to_one_category_without_wrappers() -> None:
    index = (SRC / "README.md").read_text(encoding="utf-8")
    directory_map = index.split("## Directory map", 1)[1].split(
        "## Source and install boundary", 1
    )[0]

    assert set().union(*SOURCE_FAMILIES.values()) == FUNCTIONAL_ROOTS
    assert sum(map(len, SOURCE_FAMILIES.values())) == len(FUNCTIONAL_ROOTS)

    for family, roots in SOURCE_FAMILIES.items():
        for root in roots:
            row_prefix = f"| {family} | [`{root}/`]({root}/README.md) |"
            assert directory_map.count(row_prefix) == 1

    for family in SOURCE_FAMILIES:
        assert not (SRC / family.lower().replace(" ", "_")).exists()


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
        ROOT / "docs/architecture.md",
        ROOT / "docs/development.md",
        ROOT / "docs/runtime.md",
    )
    text = "\n".join(path.read_text(encoding="utf-8") for path in current_docs)

    assert "src/nav/building" not in text
    assert "src/runtime/devices" not in text
