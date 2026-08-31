from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
PRUNE_DIR = ROOT / "src" / "maps" / "prune"
CPP_DIR = PRUNE_DIR / "cpp"
ERASOR2_REF_DIR = CPP_DIR / "refs" / "erasor2"


def test_prune_readme_defines_live_vs_saved_map_boundary() -> None:
    readme = (PRUNE_DIR / "README.md").read_text(encoding="utf-8")
    assert "Live navigation keeps current obstacles" in readme
    assert "Saved-map" in readme
    assert "dynamic-object ghosts" in readme
    assert "LingTu-owned product path" in readme
    assert "without bringing ERASOR2 GPLv3 code" in readme
    assert "cpp/prune" in readme
    assert "third_party/research_nav/ERASOR2" in readme
    assert "Do not include upstream ERASOR2 headers" in readme
    assert "Extracted ERASOR2 Workflow" in readme
    assert "Runtime Local Planning" in readme
    assert "Saved Map / Rebuild" in readme
    assert "Current obstacles stay in `rt/nav/traversability`" in readme
    assert "Do not feed a saved-map batch cleaner directly into the 10 Hz local planner" in readme
    assert "Product Framework" in readme
    assert "load -> label -> submap -> evidence -> protect -> score -> split -> save" in readme
    assert "`score` | partial" in readme
    assert "`lingtu_field_v1`" in readme


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


def test_prune_is_clean_room_product_path() -> None:
    combined = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            CPP_DIR / "cleaner.hpp",
            CPP_DIR / "cleaner.cpp",
            CPP_DIR / "prune.cpp",
        )
    )
    assert "third_party" not in combined
    assert '#include "erasor2/' not in combined
    assert "#include <pcl/" not in combined
    assert "cleanStaticMap" in combined
    assert "temporal_occupancy_v1" in combined
    assert "lingtu_field_v1" in combined
    assert "S100P/MID-360 field maps" in combined
    assert "prune" in combined
    assert "map.clean.pcd" in combined
    assert "map.removed.pcd" in combined
    assert "map.pcd.preclean" in combined
    assert "--apply" in combined
    assert "flowJson()" in combined
    assert "scoreMovingInstances" in combined
    assert "moving_instances" in combined


def test_prune_defaults_are_lingtu_field_tuned() -> None:
    header = (CPP_DIR / "cleaner.hpp").read_text(encoding="utf-8")
    cli = (CPP_DIR / "prune.cpp").read_text(encoding="utf-8")
    assert 'preset{"lingtu_field_v1"}' in header
    assert "voxel_size_m{0.20F}" in header
    assert "ground_z_threshold{-0.45F}" in header
    assert "min_hit_support{3}" in header
    assert "min_instance_points{6}" in header
    assert "S100P/MID-360 field maps" in cli


def test_prune_core_flow_contract_is_explicit_and_honest() -> None:
    flow = (CPP_DIR / "core" / "flow.cpp").read_text(encoding="utf-8")
    readme = (CPP_DIR / "core" / "README.md").read_text(encoding="utf-8")
    for stage in ("load", "label", "submap", "evidence", "protect", "score", "split", "save"):
        assert f'"{stage}"' in flow
    assert '"score"' in flow
    assert '"partial"' in flow
    assert "scoreMovingInstances" in (CPP_DIR / "core" / "score.cpp").read_text(encoding="utf-8")
    assert '"label"' in flow
    assert '"partial"' in flow
    assert "Do not add ERASOR2 headers or source includes here" in readme
    assert "label.hpp" in readme
    assert "score.hpp" in readme


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


def test_prune_cmake_declares_product_and_reference_targets() -> None:
    cmake = (CPP_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "project(lingtu_prune_cpp" in cmake
    assert "add_executable(prune" in cmake
    assert "add_library(prune_core" in cmake
    assert "add_executable(map_sift" not in cmake
    assert "add_executable(lingtu_static_cleaner" not in cmake
    assert "add_executable(erasor2_stage" in cmake
    assert "add_executable(lingtu_erasor2_stage" not in cmake
    assert "LINGTU_PRUNE_ERASOR2" in cmake
    assert "LINGTU_ERASOR2_USE_RERUN_STUB" in cmake
    assert "FETCHCONTENT_SOURCE_DIR_RERUN_SDK" in cmake
    assert "third_party/research_nav/ERASOR2" in cmake
    assert "add_executable(erasor2_clean" in cmake
    assert "add_executable(lingtu_erasor2_clean" not in cmake
    assert 'add_subdirectory("${ERASOR2_SOURCE_DIR}"' in cmake
    assert "RUNTIME_OUTPUT_DIRECTORY" in cmake
    assert "core/flow.cpp" in cmake
    assert "core/score.cpp" in cmake
    assert "refs/erasor2" in cmake
    assert cmake.index("if(LINGTU_PRUNE_ERASOR2)") < cmake.index(
        "add_executable(erasor2_stage"
    )


def test_optional_erasor2_backend_is_gpl_and_maps_reference_algorithm() -> None:
    readme = (ERASOR2_REF_DIR / "README.md").read_text(encoding="utf-8")
    notice = (ERASOR2_REF_DIR / "NOTICE.md").read_text(encoding="utf-8")
    source = (ERASOR2_REF_DIR / "clean.cpp").read_text(encoding="utf-8")
    assert "GPL-3.0-only" in readme
    assert "GPL-3.0-only" in notice
    assert "SPDX-License-Identifier: GPL-3.0-only" in source
    assert "run_erasor2" in readme
    assert "erasor2_stage" in readme
    for symbol in (
        "ERASOR2::setSubmap()",
        "ERASOR2::updateSteppableRegion()",
        "ERASOR2::detectMovingObjects()",
        "ERASOR2::filterDynamicObjects()",
        "ERASOR2::saveStaticMap()",
    ):
        assert symbol in readme
    assert "--skip-run" in source
    assert '\\"license\\": \\"GPL-3.0-only\\"' in source


def test_prune_build_script_builds_primary_binary() -> None:
    script = (ROOT / "scripts" / "build" / "build_prune.sh").read_text(encoding="utf-8")
    assert "BUILD_TARGETS=(prune)" in script
    assert "LINGTU_PRUNE_ERASOR2" in script
    assert "-DLINGTU_PRUNE_ERASOR2=OFF" in script
    assert "LINGTU_ERASOR2_USE_RERUN_STUB" in script
    assert "BUILD_TARGETS+=(erasor2_stage erasor2_clean)" in script
    assert "LINGTU_PRUNE_BUILD_DIR" in script
    assert "src/maps/prune/cpp" in script


def test_save_pipeline_uses_only_the_prune_binary_name() -> None:
    pipeline = (ROOT / "src" / "maps" / "cpp" / "build" / "pipeline.cpp").read_text(
        encoding="utf-8"
    )
    assert "LINGTU_PRUNE_BIN" in pipeline
    assert "LINGTU_STATIC_CLEANER_BIN" not in pipeline
    assert "lingtu_static_cleaner" not in pipeline
    assert "lingtu_map_cleaning" not in pipeline


def test_sunrise_prune_check_has_fetch_build_and_smoke_steps() -> None:
    fetch = (ROOT / "scripts" / "build" / "fetch_erasor2.sh").read_text(encoding="utf-8")
    check = (ROOT / "tools" / "maps" / "prune_check.sh").read_text(encoding="utf-8")
    wrapper = (ROOT / "tools" / "maps" / "erasor2_check.sh").read_text(encoding="utf-8")
    assert "d43d94f7e06a900456042979e29c3c933a39fd48" in fetch
    assert "third_party/research_nav/ERASOR2" in fetch
    assert "--fetch" in check
    assert "--build" in check
    assert "build_default_prune" in check
    assert "prune_smoke" in check
    assert "erasor2_stage_smoke" in check
    assert "build_erasor2_backend" in check
    assert "lingtu.prune_check.v1" in check
    assert "prune_check.sh" in wrapper


def test_rerun_stub_keeps_sunrise_erasor2_build_offline() -> None:
    stub_cmake = (ERASOR2_REF_DIR / "rerun_stub" / "CMakeLists.txt").read_text(encoding="utf-8")
    stub_header = (ERASOR2_REF_DIR / "rerun_stub" / "include" / "rerun.hpp").read_text(encoding="utf-8")
    assert "add_library(rerun_sdk INTERFACE)" in stub_cmake
    assert "class RecordingStream" in stub_header
    assert "class Points3D" in stub_header
    assert "class Image" in stub_header
    assert "connect_grpc" in stub_header
