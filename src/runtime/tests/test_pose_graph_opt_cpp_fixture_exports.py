from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PGO_HEADER = ROOT / "src" / "localization" / "pgo" / "src" / "pgos" / "simple_pgo.h"
PGO_SOURCE = ROOT / "src" / "localization" / "pgo" / "src" / "pgos" / "simple_pgo.cpp"
PGO_NODE = ROOT / "src" / "localization" / "pgo" / "src" / "pgo_node.cpp"
HBA_HEADER = ROOT / "src" / "localization" / "hba" / "src" / "hba" / "hba.h"
HBA_SOURCE = ROOT / "src" / "localization" / "hba" / "src" / "hba" / "hba.cpp"
HBA_NODE = ROOT / "src" / "localization" / "hba" / "src" / "hba_node.cpp"
PGO_CMAKE = ROOT / "src" / "localization" / "pgo" / "CMakeLists.txt"
HBA_CMAKE = ROOT / "src" / "localization" / "hba" / "CMakeLists.txt"


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def test_pgo_fixture_export_hook_is_declared_and_implemented():
    header = _read(PGO_HEADER)
    source = _read(PGO_SOURCE)

    assert "bool exportPoseGraphFixture(const std::string &path) const;" in header
    assert "bool SimplePGO::exportPoseGraphFixture" in source
    assert "lingtu.pose_graph_opt.fixture.v1" in source
    assert "pgo_loop" in source
    assert "pgo_loop_prior_between" in source
    assert "poses" in source
    assert "priors" in source
    assert "betweens" in source
    assert "offset" in source
    assert "offsetR" in source
    assert "offsetT" in source
    assert "m_cache_pairs" in source
    assert "fillInformationUpper" in source


def test_hba_fixture_export_hook_is_declared_and_implemented():
    header = _read(HBA_HEADER)
    source = _read(HBA_SOURCE)

    assert "bool exportPoseGraphFixture(const std::string &path);" in header
    assert "bool HBA::exportPoseGraphFixture" in source
    assert "lingtu.pose_graph_opt.fixture.v1" in source
    assert "hba_full_info" in source
    assert "hba_full_info_between_only" in source
    assert "poses" in source
    assert "priors" in source
    assert "betweens" in source
    assert "getAllFactors" in source
    assert "m_lbas.size()" in source
    assert "pose_idx + 1 < blam.poses().size()" in source
    assert "symmetricInformation" in source
    assert "fillInformationUpper" in source


def test_cpp_fixture_exports_do_not_reintroduce_gtsam_dependency():
    texts = "\n".join(_read(path) for path in [PGO_HEADER, PGO_SOURCE, PGO_NODE, HBA_HEADER, HBA_SOURCE, HBA_NODE])
    lower = texts.lower()

    assert "gtsam" not in lower
    assert "nonlinearfactorgraph" not in lower
    assert "isam" not in lower


def test_pgo_and_hba_nodes_expose_optional_fixture_export_path():
    pgo_node = _read(PGO_NODE)
    hba_node = _read(HBA_NODE)

    for source in [pgo_node, hba_node]:
        assert 'declare_parameter("fixture_export_path", "")' in source
        assert "fixture_export_path" in source
        assert "exportFixtureIfConfigured" in source
        assert "exportPoseGraphFixture" in source
        assert "create_directories" in source


def test_pgo_and_hba_cmake_keep_exports_inside_existing_nodes():
    pgo_cmake = _read(PGO_CMAKE)
    hba_cmake = _read(HBA_CMAKE)

    assert "src/pgos/simple_pgo.cpp" in pgo_cmake
    assert "src/hba/hba.cpp" in hba_cmake
    assert "lingtu_pose_graph_opt" in pgo_cmake
    assert "lingtu_pose_graph_opt" in hba_cmake
    assert "find_package(GTSAM" not in pgo_cmake
    assert "find_package(GTSAM" not in hba_cmake
