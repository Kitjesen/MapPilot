from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PRUNE_CPP = ROOT / "src" / "maps" / "prune" / "cpp"


def test_production_prune_has_no_research_backend() -> None:
    cmake = (PRUNE_CPP / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "add_library(prune_core" in cmake
    assert "add_executable(prune" in cmake
    assert "install(TARGETS prune" in cmake
    for reference in ("erasor2", "lingtu_maps_ground", "incremental", "core/score.cpp", "core/flow.cpp"):
        assert reference not in cmake.lower()


def test_prune_build_only_builds_production_cleaner() -> None:
    script = (ROOT / "scripts/build/build_prune.sh").read_text(encoding="utf-8")
    assert "--target prune" in script
    assert "src/maps/prune/cpp" in script
    assert "ERASOR2" not in script


def test_save_pipeline_uses_prune() -> None:
    pipeline = (ROOT / "src/maps/cpp/build/pipeline.cpp").read_text(encoding="utf-8")
    assert "LINGTU_PRUNE_BIN" in pipeline
    assert " --overwrite --apply" in pipeline
    assert "erasor2_clean" not in pipeline


def test_operational_check_does_not_require_research_dependencies() -> None:
    check = (ROOT / "tools/maps/prune_check.sh").read_text(encoding="utf-8")
    assert "build_default_prune" in check
    assert "prune_smoke" in check
    for reference in ("ERASOR2", "pkg_or_config", "opencv4", "pcl_common"):
        assert reference not in check
