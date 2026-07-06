"""Filesystem contract tests for the root sim/ layout."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_ROOT = REPO_ROOT / "sim"

CANONICAL_ROOTS = (
    "engine",
    "worlds",
    "assets",
    "robots",
    "sensors",
    "bridge",
    "planning",
    "following",
    "datasets",
    "evaluation",
    "experiments",
    "scripts",
    "validation",
    "tests",
    "launch",
    "external_scenes",
    "semantic",
    "configs",
    "maps",
)

CANONICAL_MUJOCO_ENTRYPOINTS = (
    "mujoco/launch_fastlio2_live.sh",
    "mujoco/live_gate.py",
    "mujoco/native_dds_sensors.py",
    "mujoco/saved_map_plan_gate.py",
    "mujoco/saved_map_quality_gate.py",
    "mujoco/native_pct_gate.py",
    "mujoco/navigation_audit.py",
    "mujoco/record_policy_nav_video.py",
    "mujoco/record_thunderv4_mid360_policy.py",
    "mujoco/record_thunderv4_stair_showcase.py",
)

COMPAT_SCRIPT_WRAPPERS = (
    "launch_mujoco_fastlio2_live.sh",
    "mujoco_live_gate.py",
    "mujoco_native_dds_sensors.py",
    "mujoco_saved_map_plan_gate.py",
    "mujoco_saved_map_quality_gate.py",
    "native_pct_mujoco_gate.py",
    "mujoco_navigation_audit.py",
    "record_policy_nav_video.py",
    "record_thunderv4_mid360_policy.py",
    "record_thunderv4_stair_showcase.py",
)

PUBLIC_SCRIPT_ENTRYPOINTS = (
    "server_sim_closure.py",
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


def test_sim_mujoco_scripts_have_canonical_implementation_paths():
    scripts_root = SIM_ROOT / "scripts"

    for name in CANONICAL_MUJOCO_ENTRYPOINTS:
        assert (scripts_root / name).is_file(), name


def test_sim_mujoco_compat_wrappers_stay_in_place():
    scripts_root = SIM_ROOT / "scripts"

    for name in COMPAT_SCRIPT_WRAPPERS:
        wrapper = scripts_root / name
        assert wrapper.is_file(), name
        text = _read(wrapper)
        assert (
            "Compatibility wrapper" in text
            or "sim/scripts/mujoco/launch_fastlio2_live.sh" in text
        ), name


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
    assert "`sim/scripts/mujoco/*` holds the canonical MuJoCo" in sim_readme
    assert "Canonical MuJoCo Entrypoints" in scripts_index
    assert "Public Entrypoints And Wrappers" in scripts_index
    assert "This directory is a stable script contract" in scripts_index
    assert "MuJoCo implementations live under `sim/scripts/mujoco/`" in repo_layout
    assert "old `sim/scripts/<name>` entrypoints are compatibility wrappers" in repo_layout


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
