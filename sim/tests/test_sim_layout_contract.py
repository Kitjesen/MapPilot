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
    "maps",
)

CANONICAL_MUJOCO_ENTRYPOINTS = (
    "mujoco/launch_fastlio2_live.sh",
    "mujoco/live_gate.py",
    "mujoco/native_dds_sensors.py",
    "mujoco/saved_map_plan_gate.py",
    "mujoco/saved_map_tracking_gate.py",
    "mujoco/saved_map_quality_gate.py",
    "mujoco/native_pct_gate.py",
    "mujoco/navigation_audit.py",
    "mujoco/record_policy_nav_video.py",
    "mujoco/record_thunderv4_mid360_policy.py",
    "mujoco/record_thunderv4_stair_showcase.py",
    "mujoco/continuous_mapping_quality_gate.py",
)

RETIRED_MUJOCO_COMPAT_WRAPPERS = (
    "launch_mujoco_fastlio2_live.sh",
    "mujoco_live_gate.py",
    "mujoco_native_dds_sensors.py",
    "mujoco_saved_map_plan_gate.py",
    "mujoco_saved_map_tracking_gate.py",
    "mujoco_saved_map_quality_gate.py",
    "native_pct_mujoco_gate.py",
    "mujoco_navigation_audit.py",
    "record_policy_nav_video.py",
    "record_thunderv4_mid360_policy.py",
    "record_thunderv4_stair_showcase.py",
    "mujoco_continuous_mapping_quality_gate.py",
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
)


RETIRED_CMU_PRODUCT_ENTRYPOINTS = (
    "cmu_unity_lingtu_stack.py",
    "launch_cmu_unity_lingtu_runtime.sh",
    "cmu_unity_sim_gate.py",
    "cmu_unity_runtime_gate.py",
    "cmu_unity_tomogram_capture.py",
)


RETIRED_CMU_PRODUCT_TESTS = (
    "test_cmu_unity_sim_gate.py",
    "test_cmu_unity_runtime_gate.py",
    "test_cmu_unity_tare_strategy_quality.py",
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


def test_retired_cmu_product_runner_is_absent_but_external_baseline_remains():
    scripts_root = SIM_ROOT / "scripts"
    tests_root = SIM_ROOT / "tests"

    assert (scripts_root / "launch_cmu_unity_baseline.sh").is_file()
    assert (SIM_ROOT / "engine" / "bridge" / "cmu_unity_lingtu_adapter.py").is_file()
    for name in RETIRED_CMU_PRODUCT_ENTRYPOINTS:
        assert not (scripts_root / name).exists(), name
    for name in RETIRED_CMU_PRODUCT_TESTS:
        assert not (tests_root / name).exists(), name


def test_sim_mujoco_scripts_have_canonical_implementation_paths():
    scripts_root = SIM_ROOT / "scripts"

    for name in CANONICAL_MUJOCO_ENTRYPOINTS:
        assert (scripts_root / name).is_file(), name


def test_retired_mujoco_compat_wrappers_are_absent():
    scripts_root = SIM_ROOT / "scripts"

    for name in RETIRED_MUJOCO_COMPAT_WRAPPERS:
        assert not (scripts_root / name).exists(), name


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
    assert "Top-level MuJoCo compatibility wrappers are retired" in sim_readme
    assert "Canonical MuJoCo Entrypoints" in scripts_index
    assert "Other Public Entrypoints" in scripts_index
    assert "This directory is a stable script contract" in scripts_index
    assert "command and test interface lives only under" in scripts_index
    assert "only current MuJoCo command and test path" in repo_layout
