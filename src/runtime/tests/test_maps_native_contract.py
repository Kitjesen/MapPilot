"""Static boundaries for the native maps migration."""

from __future__ import annotations

import ast
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]


def test_retired_python_live_map_layers_are_absent() -> None:
    retired = (
        REPO / "src" / "maps" / "modules" / "occupancy.py",
        REPO / "src" / "maps" / "modules" / "voxel_grid.py",
        REPO / "src" / "maps" / "modules" / "elevation.py",
        REPO / "src" / "maps" / "modules" / "esdf.py",
        REPO / "src" / "maps" / "modules" / "traversability.py",
        REPO / "src" / "maps" / "adapters" / "python" / "kernels.py",
        REPO / "src" / "maps" / "adapters" / "python" / "voxel.py",
    )
    for path in retired:
        assert not path.exists(), path


def test_mapd_status_uses_product_session_identity() -> None:
    source = (REPO / "src" / "maps" / "cpp" / "mapd" / "main.cpp").read_text(encoding="utf-8")

    assert source.count("options.product_session_id") >= 4
    assert 'options.product = EnvOr("LINGTU_PRODUCT", "")' in source
    assert 'options.product_session_id = EnvOr("LINGTU_PRODUCT_SESSION_ID", "")' in source
    assert '\\"product_session_id\\":\\"' in source


def test_runtime_maps_stack_does_not_expose_python_map_management() -> None:
    assert not (REPO / "src" / "lingtu" / "assembly" / "stacks" / "maps.py").exists()


def test_legacy_nav_map_service_entry_is_removed() -> None:
    assert not (REPO / "src" / "nav" / "services" / "maps.py").exists()
    assert not (REPO / "src" / "nav" / "services" / "map").exists()
    old_service = "nav.services" + ".maps"
    files = (REPO / "src" / "lingtu" / "assembly" / "plugins.py",)
    for path in files:
        assert old_service not in path.read_text(encoding="utf-8")


def test_live_maps_modules_do_not_depend_on_scipy_fallbacks() -> None:
    module_dir = REPO / "src" / "maps" / "modules"
    for path in module_dir.glob("*.py"):
        text = path.read_text(encoding="utf-8").lower()
        assert "scipy" not in text
        assert "_scipy_available" not in text


def test_save_source_postprocess_uses_native_commit_contract() -> None:
    native = (REPO / "src" / "maps" / "cpp" / "save.cpp").read_text(encoding="utf-8")
    pipeline = (REPO / "src" / "maps" / "cpp" / "build" / "pipeline.cpp").read_text(encoding="utf-8")
    commit_saved_source = pipeline.split("std::string MapPipelineCore::CommitSavedSourceJson", 1)[1].split(
        "std::string MapPipelineCore::", 1
    )[0]
    assert "CommitSavedSourceJson(" in native
    assert commit_saved_source.count("LoadPcdXyz(source_pcd_path)") == 1
    assert "BuildNavigationPackageJson(" in native
    assert "VerifyRequiredArtifacts(commit_stage" in native
    assert "CommitStage(store_.RootDir()" in native
    assert "CommitBackup(" in native
    assert "current_version.txt" not in native
    assert ".versions" not in native


def test_save_map_commits_one_locked_checked_canonical_directory() -> None:
    save = (REPO / "src" / "maps" / "cpp" / "save.cpp").read_text(encoding="utf-8")
    store = (REPO / "src" / "maps" / "cpp" / "store.cpp").read_text(encoding="utf-8")
    assert "MapLock::TryAcquire" in save
    assert "VerifyRequiredArtifacts(commit_stage" in save
    assert 'root / (".save-staging-" + job_id)' in save
    assert 'root / (".save-backup-" + map_id + "-" + job_id)' in save
    assert "std::filesystem::rename(commit_stage, map_dir)" in save
    assert "std::filesystem::rename(commit_backup, map_dir" in save
    assert "std::filesystem::remove_all(commit_backup)" in save
    assert "LoadPcdXyz" not in save
    assert "snapshot_receipt_point_count_mismatch" not in save
    assert "validating snapshot" not in save
    assert "verifying required artifacts" not in save
    assert "return MapPath(map_id);" in store
    assert "MapLock::TryAcquire" in store
    assert 'content / "map.pcd"' in store
    assert '{"octomap", "octomap.ot", ArtifactType::kOctomap3D' in store
    assert 'content / "metadata.json"' in store
    assert not (REPO / "src" / "maps" / "cpp" / "version.cpp").exists()
    assert not (REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "version.hpp").exists()
    for retired in (
        "current_version.txt",
        ".versions",
        "save_manifest.json",
        "source_sha256",
        "artifact_checksums.sha256",
        "save_manifest.sha256",
        "command_sha256",
    ):
        assert retired not in save.lower()


def test_python_map_sources_do_not_live_in_cmake_build_directory() -> None:
    build_dir = REPO / "src" / "maps" / "build"
    assert not (build_dir / "artifacts.py").exists()
    assert not (build_dir / "__init__.py").exists()
    assert not (REPO / "src" / "maps" / "artifact_builder.py").exists()


def test_retired_python_map_builders_and_prune_runtime_are_removed() -> None:
    assert not (REPO / "src" / "maps" / "artifact_builder.py").exists()
    assert not (REPO / "src" / "maps" / "services" / "voxel_edit.py").exists()
    assert not (REPO / "src" / "maps" / "prune" / "runtime.py").exists()
    assert not (REPO / "src" / "maps" / "services" / "records.py").exists()


def test_python_map_management_layers_are_absent() -> None:
    python_sources = tuple((REPO / "src" / "maps").rglob("*.py"))
    assert python_sources == ()


def test_retired_python_artifact_validator_is_absent() -> None:
    files = (
        REPO / "src" / "gateway" / "routes" / "diagnostics.py",
        REPO / "src" / "diagnostics" / "field" / "field_check.py",
        REPO / "scripts" / "gates" / "saved_map_artifact_gate.py",
        REPO / "scripts" / "gates" / "saved_map_field_acceptance.py",
    )
    for path in files:
        assert "validate_saved_map_artifact_dir" not in path.read_text(encoding="utf-8")


def test_retired_map_c_apis_are_absent() -> None:
    retired = (
        REPO / "src" / "maps" / "cpp" / "store_c_api.cpp",
        REPO / "src" / "maps" / "cpp" / "pcd_c_api.cpp",
        REPO / "src" / "maps" / "cpp" / "rolling_occupancy_c_api.cpp",
        REPO / "src" / "maps" / "cpp" / "grid_layers_c_api.cpp",
        REPO / "src" / "maps" / "cpp" / "service_c_api.cpp",
        REPO / "src" / "maps" / "cpp" / "voxel_c_api.cpp",
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "store.h",
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "pcd.h",
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "rolling_occupancy.h",
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "grid_layers.h",
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "service.h",
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "voxel_layer.h",
        REPO / "src" / "maps" / "adapters" / "python" / "store.py",
        REPO / "src" / "maps" / "adapters" / "python" / "pcd.py",
    )
    for path in retired:
        assert not path.exists(), path

    cmake = (REPO / "src" / "maps" / "CMakeLists.txt").read_text(encoding="utf-8")
    for retired_name in (
        "store_c_api",
        "pcd_c_api",
        "rolling_occupancy_c_api",
        "grid_layers_c_api",
        "service_c_api",
        "voxel_c_api",
        "lingtu_maps_c_api",
    ):
        assert retired_name not in cmake


def test_gateway_uses_public_maps_contract() -> None:
    files = (
        REPO / "src" / "gateway" / "services" / "mapd_transport.py",
        REPO / "src" / "gateway" / "routes" / "session.py",
        REPO / "src" / "gateway" / "routes" / "diagnostics.py",
    )
    forbidden = (
        "_map_list",
        "_get_active_map",
        "_get_map_bundle",
        "_get_map_points",
        "_validate_map_artifacts",
        "_map_set_active",
    )
    for path in files:
        text = path.read_text(encoding="utf-8")
        for name in forbidden:
            assert name not in text, f"{path}: private maps API {name}"


def test_gateway_does_not_construct_maps_service() -> None:
    text = (REPO / "src" / "gateway" / "services" / "mapd_transport.py").read_text(encoding="utf-8")
    assert "MapsModule(" not in text
    assert "manager.setup()" not in text


def test_maps_service_does_not_expose_a_second_activation_owner() -> None:
    files = (
        REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "service.hpp",
        REPO / "src" / "maps" / "cpp" / "service_lifecycle.cpp",
    )
    for path in files:
        text = path.read_text(encoding="utf-8")
        assert "SetActiveMapJson" not in text, path
        assert "ClearActiveMapJson" not in text, path


def test_retired_abstract_map_api_is_absent() -> None:
    assert not (REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "api.hpp").exists()


def test_gateway_map_routes_use_typed_maps_boundary() -> None:
    route = (REPO / "src" / "gateway" / "routes" / "maps.py").read_text(encoding="utf-8")
    assert "mapd_command(" in route
    assert "MapControlRequest" not in route
    assert 'pathlib.Path(str(resp.get("map_dir")' not in route
    tree = ast.parse(route)
    pcd_handler = next(
        node for node in ast.walk(tree) if isinstance(node, ast.AsyncFunctionDef) and node.name == "get_map_pcd"
    )
    pcd_source = ast.unparse(pcd_handler)
    assert "open_artifact" in pcd_source
    assert "StreamingResponse" in pcd_source
    assert "_mapd_command" not in pcd_source
    assert "artifact_path" not in pcd_source
    assert "resolve_exchange_path(" in route
    assert "map_import_root(" in route
    assert ".read_bytes()" not in route


def test_map_package_exchange_paths_are_confined() -> None:
    native = (REPO / "src" / "maps" / "cpp" / "service_jobs.cpp").read_text(encoding="utf-8")
    paths = (REPO / "src" / "runtime" / "endpoints" / "map_paths.py").read_text(encoding="utf-8")
    assert "unsafe_exchange_path" in native
    assert "LINGTU_MAP_IMPORT_DIR" in paths


def test_maps_core_declares_optional_native_octomap_builder() -> None:
    cmake = (REPO / "src" / "maps" / "CMakeLists.txt").read_text(encoding="utf-8")
    pipeline = (REPO / "src" / "maps" / "cpp" / "build" / "pipeline.cpp").read_text(encoding="utf-8")
    assert "LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP" in cmake
    assert "find_package(octomap QUIET)" in cmake
    assert "native_octomap" in pipeline
    assert "native_octomap_unavailable" in pipeline


def test_gateway_map_routes_delegate_remaining_persistent_mutations() -> None:
    route = (REPO / "src" / "gateway" / "routes" / "maps.py").read_text(encoding="utf-8")
    assert '"action": "restore_source"' not in route
    assert '"action": "rename_map"' in route
    assert "shutil.copy" not in route
    assert "os.rename" not in route
    session_route = (REPO / "src" / "gateway" / "routes" / "session.py").read_text(encoding="utf-8")
    assert "os.symlink" not in session_route


def test_runtime_uses_maps_service_alias() -> None:
    production_roots = (
        REPO / "src" / "runtime",
        REPO / "src" / "gateway",
        REPO / "src" / "lingtu",
        REPO / "cli",
    )
    stale_alias = "nav" + ".maps"
    for root in production_roots:
        for path in root.rglob("*.py"):
            if "tests" in path.parts:
                continue
            assert stale_alias not in path.read_text(encoding="utf-8"), path
