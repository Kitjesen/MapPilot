"""Filesystem contract tests for the root sim/ layout."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "sim"

CANONICAL_ROOTS = (
    "worlds",
    "assets",
    "robots",
    "scripts",
    "validation",
    "tests",
)

PUBLIC_SCRIPT_ENTRYPOINTS = (
    "launch_mujoco_fastlio2_live.sh",
    "server_sim_closure.py",
    "native_pct_mujoco_gate.py",
    "pct_saved_map_navigation_gate.py",
    "saved_map_relocalize_runtime_gate.py",
    "routecheck_preflight_gate.py",
    "multifloor_nav_validation.py",
    "policy_nav_smoke.py",
    "run_dimos_linux_closure.sh",
    "launch_lingtu_gazebo_industrial_demo.sh",
    "launch_cmu_unity_lingtu_runtime.sh",
)


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_sim_canonical_roots_exist():
    for name in CANONICAL_ROOTS:
        assert (SIM_ROOT / name).is_dir(), name


def test_sim_public_script_entrypoints_stay_in_place():
    scripts_root = SIM_ROOT / "scripts"

    for name in PUBLIC_SCRIPT_ENTRYPOINTS:
        assert (scripts_root / name).is_file(), name


def test_sim_canonical_assets_are_discoverable():
    assert (SIM_ROOT / "assets" / "livox" / "mid360.npy").is_file()
    assert (SIM_ROOT / "robots" / "nova_dog" / "policy_manifest.json").is_file()
    assert any((SIM_ROOT / "worlds" / "mujoco").glob("*.xml"))
    assert any((SIM_ROOT / "worlds" / "gazebo").glob("*.sdf"))


def test_sim_active_docs_define_stable_root_contract():
    sim_readme = _read(SIM_ROOT / "README.md")
    scripts_index = _read(SIM_ROOT / "scripts" / "README.md")
    repo_layout = _read(REPO_ROOT / "docs" / "REPO_LAYOUT.md")

    assert "Stable Root Contract" in sim_readme
    assert "`sim/scripts/<name>` paths are public repository entrypoints" in sim_readme
    assert "Public Entrypoints" in scripts_index
    assert "This directory is a stable script contract" in scripts_index
    assert "stable roots: worlds, assets, robots, scripts, validation, tests" in repo_layout
    assert "`sim/scripts/<name>` is a stable path contract" in repo_layout


def test_superseded_physical_split_guidance_is_marked():
    old_plan = _read(
        REPO_ROOT
        / "docs"
        / "superpowers"
        / "plans"
        / "2026-05-30-repo-structure-redesign.md"
    )
    current_plan = _read(
        REPO_ROOT
        / "docs"
        / "superpowers"
        / "plans"
        / "2026-05-31-sim-folder-modularization-goals.md"
    )

    assert "Superseded note (2026-06-12)" in old_plan
    assert "`sim/scripts/<name>` as a stable public path contract" in old_plan
    assert "Do not physically split; add indexes/tests instead" in current_plan
