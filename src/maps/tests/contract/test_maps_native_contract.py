"""Static boundaries for the native maps migration."""

from __future__ import annotations

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]


def test_maps_python_kernels_do_not_import_nav_kernel() -> None:
    text = (REPO / "src" / "maps" / "adapters" / "python" / "kernels.py").read_text(encoding="utf-8")
    nav_kernel = "nav" + ".kernel"
    forbidden = (
        "from " + nav_kernel,
        "import " + nav_kernel,
        "try_import_nav_kernel",
    )
    for needle in forbidden:
        assert needle not in text


def test_runtime_maps_stack_uses_maps_modules() -> None:
    text = (REPO / "src" / "runtime" / "blueprints" / "stacks" / "maps.py").read_text(encoding="utf-8")
    assert "maps.modules.occupancy.OccupancyGridModule" in text
    assert "maps.modules.voxel_grid.VoxelGridModule" in text
    assert "nav.services" + ".map_layers" not in text


def test_legacy_nav_map_service_entry_is_removed() -> None:
    assert not (REPO / "src" / "nav" / "services" / "maps.py").exists()
    assert not (REPO / "src" / "nav" / "services" / "map").exists()
    old_service = "nav.services" + ".maps"
    files = (
        REPO / "src" / "runtime" / "blueprints" / "stacks" / "maps.py",
        REPO / "src" / "lingtu" / "plugin_seed.py",
    )
    for path in files:
        assert old_service not in path.read_text(encoding="utf-8")


def test_live_maps_modules_do_not_depend_on_scipy_fallbacks() -> None:
    module_dir = REPO / "src" / "maps" / "modules"
    for path in module_dir.glob("*.py"):
        text = path.read_text(encoding="utf-8").lower()
        assert "scipy" not in text
        assert "_scipy_available" not in text


def test_save_source_postprocess_uses_native_commit_contract() -> None:
    text = (REPO / "src" / "maps" / "services" / "pipeline.py").read_text(encoding="utf-8")
    assert "begin_save_map(" in text
    assert "provide_save_map_snapshot(" in text
    assert "apply_dynamic_filter_step1half" not in text
    assert ".map_opt.run(" not in text
    native = (REPO / "src" / "maps" / "cpp" / "save.cpp").read_text(encoding="utf-8")
    assert "CommitSavedSourceJson(" in native
    assert "BuildNavigationPackageJson(" in native
    assert "current_version.txt" in native


def test_save_map_versions_are_locked_and_fully_verified() -> None:
    save = (REPO / "src" / "maps" / "cpp" / "save.cpp").read_text(encoding="utf-8")
    store = (REPO / "src" / "maps" / "cpp" / "store.cpp").read_text(encoding="utf-8")
    version = (REPO / "src" / "maps" / "cpp" / "version.cpp").read_text(encoding="utf-8")
    assert "BuildArtifactChecksums(hashes)" in save
    assert "VerifyMapVersion(version_stage" in save
    assert "MapLock::TryAcquire" in save
    assert "VerifyMapVersion(content)" in store
    assert "MapLock::TryAcquire" in store
    assert "version artifact hash mismatch" in version


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


def test_map_lifecycle_control_is_native_only() -> None:
    control = (REPO / "src" / "maps" / "services" / "control.py").read_text(encoding="utf-8")
    for forbidden in (
        "write_map_record",
        "load_map_record",
        "symlink_to",
        "subprocess",
        "os.replace",
        "shutil",
    ):
        assert forbidden not in control

    adapter = (REPO / "src" / "maps" / "adapters" / "python" / "service.py").read_text(encoding="utf-8")
    c_api = (REPO / "src" / "maps" / "include" / "lingtu" / "maps" / "c_api" / "service.h").read_text(encoding="utf-8")
    for symbol in (
        "lingtu_maps_service_retire_map_json",
        "lingtu_maps_service_restore_source_backup_json",
        "lingtu_maps_service_edit_octomap_voxels_json",
        "lingtu_maps_service_validate_artifacts_json",
    ):
        assert symbol in adapter
        assert symbol in c_api


def test_maps_domain_does_not_import_nav_map_services() -> None:
    forbidden = "nav.services" + ".map"
    for path in (REPO / "src" / "maps").rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        assert forbidden not in text, path


def test_maps_storage_has_no_python_file_fallback() -> None:
    text = (REPO / "src" / "maps" / "services" / "storage.py").read_text(encoding="utf-8")
    assert "_FileMapStore" not in text
    assert "_FileMapsService" not in text
    assert "python_file_fallback" not in text
    assert "requires the native lingtu_maps library" in text
    assert "active_map.yaml" not in text
    assert "pois.yaml" not in text
    assert "_read_legacy_active_link" not in text


def test_maps_storage_fails_fast_when_native_is_missing(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    import maps.services.storage as storage

    def unavailable(*_args, **_kwargs):
        raise storage.MapStoreNativeUnavailable("native test failure")

    monkeypatch.setattr(storage, "NativeMapStore", unavailable)
    with pytest.raises(RuntimeError, match="requires the native lingtu_maps"):
        storage.MapStorageService(
            data_dir=tmp_path / "data",
            map_dir=tmp_path / "maps",
        )


def test_map_queries_do_not_rebuild_native_records_in_python() -> None:
    text = (REPO / "src" / "maps" / "services" / "api.py").read_text(encoding="utf-8")
    assert "build_record_for_dir" not in text
    assert "artifact_for_capability" not in text
    assert "map_type_catalog" not in text


def test_planner_saved_map_lookup_uses_native_bundle() -> None:
    assets = (REPO / "src" / "maps" / "services" / "active_assets.py").read_text(encoding="utf-8")
    assert "NativeMapStore" in assets
    assert ".bundle(" in assets
    for forbidden in (
        "active_map_dir",
        "load_map_record",
        "artifact_for_capability",
        "map_record.json",
    ):
        assert forbidden not in assets


def test_runtime_artifact_gate_is_native() -> None:
    files = (
        REPO / "src" / "maps" / "services" / "api.py",
        REPO / "src" / "maps" / "services" / "control.py",
        REPO / "src" / "maps" / "services" / "active_assets.py",
    )
    for path in files:
        assert "validate_saved_map_artifact_dir" not in path.read_text(encoding="utf-8")
    control = files[1].read_text(encoding="utf-8")
    assert "native_service.validate_artifacts(" in control


def test_active_map_artifact_gate_rejects_active_switch_during_validation(
    monkeypatch,
    tmp_path,
) -> None:
    from maps.services import active_assets as active_assets_module

    artifact = tmp_path / "map_a" / "occupancy.npz"

    class SwitchingStore:
        def __init__(self, _root):
            self.active_reads = 0

        def active_map_id(self):
            self.active_reads += 1
            return "map_a" if self.active_reads == 1 else "map_b"

        def bundle(self, map_id, _capability):
            return {
                "success": True,
                "map_id": map_id,
                "frame_id": "map",
                "artifact": {
                    "uri": str(artifact),
                    "hash": "a" * 64,
                },
            }

        def validate_artifacts(self, map_id, **_kwargs):
            return {
                "success": True,
                "map_id": map_id,
                "gate": {
                    "ok": True,
                    "checked_frame_id": "map",
                    "blockers": [],
                },
            }

        def close(self):
            pass

    monkeypatch.setattr(active_assets_module, "NativeMapStore", SwitchingStore)
    gate = active_assets_module.ActiveMapAssets(
        map_roots=(tmp_path,),
    ).validate_artifact_path(
        str(artifact),
        require_occupancy=True,
        expected_frame_id="map",
    )

    assert gate["ok"] is False
    assert "active map changed during artifact validation" in gate["blockers"]


def test_gateway_uses_public_maps_contract() -> None:
    files = (
        REPO / "src" / "gateway" / "services" / "map_service.py",
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
    text = (REPO / "src" / "gateway" / "services" / "map_service.py").read_text(encoding="utf-8")
    assert "from maps.modules.service import MapsModule" not in text
    assert "MapsModule(" not in text
    assert "manager.setup()" not in text
    assert "ProductGraph/Blueprint must inject maps.service" in text


def test_gateway_map_routes_use_typed_maps_boundary() -> None:
    route = (REPO / "src" / "gateway" / "routes" / "maps.py").read_text(encoding="utf-8")
    assert "map_service_command(" in route
    assert "MapControlRequest" not in route
    assert 'pathlib.Path(str(resp.get("map_dir")' not in route
    assert 'artifact_path(gw, name, "source_pointcloud")' in route
    assert "resolve_exchange_path(" in route
    assert "map_import_root(" in route
    assert ".read_bytes()" not in route


def test_map_package_exchange_paths_are_confined() -> None:
    api = (REPO / "src" / "maps" / "services" / "api.py").read_text(encoding="utf-8")
    native = (REPO / "src" / "maps" / "cpp" / "service.cpp").read_text(encoding="utf-8")
    paths = (REPO / "src" / "maps" / "paths.py").read_text(encoding="utf-8")
    assert "resolve_exchange_path(" in api
    assert "unsafe_exchange_path" in native
    assert "LINGTU_MAP_IMPORT_DIR" in paths
    assert "LINGTU_MAP_EXPORT_DIR" in paths


def test_maps_root_exports_persistent_domain_api() -> None:
    import maps

    expected = {
        "MapAPIService",
        "MapControlService",
        "MapPipelineService",
        "MapRuntimeBridge",
        "MapStorageService",
        "safe_map_name",
        "validate_map_name",
    }
    assert expected <= set(maps.__all__)


def test_maps_core_declares_optional_native_octomap_builder() -> None:
    cmake = (REPO / "src" / "maps" / "CMakeLists.txt").read_text(encoding="utf-8")
    pipeline = (REPO / "src" / "maps" / "cpp" / "build" / "pipeline.cpp").read_text(encoding="utf-8")
    assert "LINGTU_MAPS_ENABLE_NATIVE_OCTOMAP" in cmake
    assert "find_package(octomap QUIET)" in cmake
    assert "native_octomap" in pipeline
    assert "native_octomap_unavailable" in pipeline


def test_gateway_map_routes_delegate_persistent_mutations() -> None:
    route = (REPO / "src" / "gateway" / "routes" / "maps.py").read_text(encoding="utf-8")
    assert '"action": "restore_source"' in route
    assert '"action": "rename"' in route
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
