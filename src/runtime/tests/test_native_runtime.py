from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
NATIVE_RUNTIME = ROOT / "src" / "native" / "runtime"


def _read(name: str) -> str:
    return (NATIVE_RUNTIME / name).read_text(encoding="utf-8")


def test_native_runtime_manifest_declares_product_cpp_components() -> None:
    text = _read("components.cpp")

    assert '"lidar"' in text
    assert '"src/drivers/real/lidar/sdk2_stream"' in text
    assert '"slam"' in text
    assert '"src/localization/slam/cpp"' in text
    assert '"nav"' in text
    assert '"src/nav/services/endpoint/cpp"' in text


def test_pgo_hba_are_product_native_save_time_commands() -> None:
    text = _read("components.cpp")

    pgo_start = text.index('"pgo"')
    pgo_block = text[pgo_start : text.index("},", pgo_start) + 2]
    assert "State::Ready" in pgo_block
    assert '"lt_pgo"' in pgo_block
    assert "true," in pgo_block
    assert "false," in pgo_block

    hba_start = text.index('"hba"')
    hba_block = text[hba_start : text.index("},", hba_start) + 2]
    assert "State::Ready" in hba_block
    assert '"lt_hba"' in hba_block
    assert "false," in hba_block


def test_manifest_is_cpp_boundary_not_gateway_api_catalog() -> None:
    header = _read("components.hpp")
    source = _read("components.cpp")

    assert "operations.py" not in header
    assert "operations.py" not in source
    assert "runtime/service_catalogs" not in source
    assert "gateway/routes" not in source


def test_architecture_doc_names_opt_pgo_hba_files() -> None:
    doc = (ROOT / "docs" / "architecture" / "NATIVE_RUNTIME.md").read_text(encoding="utf-8")

    assert "`src/localization/opt/pgo.cpp`" in doc
    assert "`src/localization/opt/hba.cpp`" in doc
    assert "`lt_pgo`" in doc
    assert "`lt_hba`" in doc


def test_native_runner_sources_call_pose_graph_kernel() -> None:
    pgo = (ROOT / "src" / "localization" / "opt" / "pgo.cpp").read_text(encoding="utf-8")
    hba = (ROOT / "src" / "localization" / "opt" / "hba.cpp").read_text(encoding="utf-8")
    artifacts = (ROOT / "src" / "localization" / "opt" / "map.cpp").read_text(encoding="utf-8")
    graph = (ROOT / "src" / "localization" / "opt" / "graph.cpp").read_text(encoding="utf-8")
    cli = (ROOT / "src" / "localization" / "opt" / "cli.cpp").read_text(encoding="utf-8")

    assert "optimize_map(map, options)" in pgo
    assert "pgo_not_ready" not in pgo
    assert "optimize_map(map, options)" in hba
    assert "hba_not_ready" not in hba
    assert "lt_pose_graph_opt_process_se3" in graph
    assert "map_optimization.json" in graph
    assert "patches_empty" in artifacts
    assert "--map is required" in cli
    cmake = (ROOT / "src" / "native" / "runtime" / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "lt_pgo" in cmake
    assert "lt_pose_graph_opt" in cmake
