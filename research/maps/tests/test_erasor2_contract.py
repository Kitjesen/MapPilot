from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ERASOR2_REF_DIR = ROOT / "research" / "maps" / "erasor2"


def test_erasor2_stage_is_lingtu_owned_boundary_code() -> None:
    combined = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            ERASOR2_REF_DIR / "stager.hpp",
            ERASOR2_REF_DIR / "stager.cpp",
            ERASOR2_REF_DIR / "stage.cpp",
        )
    )
    assert "third_party/research_nav/ERASOR2" not in combined
    assert '#include "erasor2/' not in combined
    assert "#include <pcl/" not in combined
    assert "stageErasor2Dataset" in combined


def test_erasor2_stage_contract_outputs_expected_dataset_shape() -> None:
    source = (ERASOR2_REF_DIR / "stager.cpp").read_text(encoding="utf-8")
    for required in (
        '"velodyne"',
        '"patchwork"',
        '"hdbscan"',
        '"poses.txt"',
        '"erasor2.yaml"',
        "ground_z_threshold",
        "instance_grid_m",
    ):
        assert required in source


def test_rerun_stub_keeps_sunrise_erasor2_build_offline() -> None:
    stub_cmake = (ERASOR2_REF_DIR / "rerun_stub" / "CMakeLists.txt").read_text(encoding="utf-8")
    stub_header = (ERASOR2_REF_DIR / "rerun_stub" / "include" / "rerun.hpp").read_text(encoding="utf-8")
    assert "add_library(rerun_sdk INTERFACE)" in stub_cmake
    assert "class RecordingStream" in stub_header
    assert "class Points3D" in stub_header
    assert "class Image" in stub_header
    assert "connect_grpc" in stub_header
