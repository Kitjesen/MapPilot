from pathlib import Path
from types import SimpleNamespace

from maps.services.command_router import dispatch_map_command
from maps.services.pipeline import MapPipelineService
from runtime.msgs.map import MapControlRequest


class _RouteTarget:
    def __init__(self) -> None:
        self.call = None

    def _import_unity_semantic_artifact(self, name, scene_dir, **options):
        self.call = (name, scene_dir, options)
        return {
            "action": "import_unity_semantic_artifact",
            "success": True,
            "map_id": name,
        }


class _NativeTarget:
    def __init__(self) -> None:
        self.call = None

    def import_unity_semantic_artifact(self, name, scene_dir, **options):
        self.call = (name, scene_dir, options)
        return {
            "action": "import_unity_semantic_artifact",
            "success": True,
            "mode": "native_transaction",
        }


def test_typed_control_routes_unity_semantics_with_bounded_defaults():
    service = _RouteTarget()
    request = MapControlRequest.from_mapping(
        {
            "action": "import_unity_semantics",
            "name": "office",
            "scene_dir": "/exchange/unity/office",
        }
    )

    result = dispatch_map_command(service, request.to_mapping())

    assert result["success"] is True
    assert service.call is not None
    name, scene_dir, options = service.call
    assert name == "office"
    assert scene_dir == "/exchange/unity/office"
    assert options["voxel_size_m"] == 0.20
    assert options["max_objects"] == 100_000
    assert options["max_voxels"] == 2_000_000
    assert options["max_voxel_checks"] == 50_000_000
    assert options["exclude_dynamic_classes"] is True


def test_pipeline_delegates_unity_import_to_native_service(tmp_path):
    native = _NativeTarget()
    pipeline = MapPipelineService.__new__(MapPipelineService)
    pipeline.storage = SimpleNamespace(native_service=native)
    pipeline.semantic_taxonomy_path = tmp_path / "taxonomy.json"

    result = pipeline.import_unity_semantic_artifact(
        "office",
        tmp_path / "scene",
        generation=12,
    )

    assert result["mode"] == "native_transaction"
    assert native.call is not None
    name, scene_dir, options = native.call
    assert name == "office"
    assert scene_dir == tmp_path / "scene"
    assert options["taxonomy_path"] == Path(tmp_path / "taxonomy.json")
    assert options["generation"] == 12


def test_pipeline_rejects_missing_scene_before_native_call(tmp_path):
    native = _NativeTarget()
    pipeline = MapPipelineService.__new__(MapPipelineService)
    pipeline.storage = SimpleNamespace(native_service=native)
    pipeline.semantic_taxonomy_path = tmp_path / "taxonomy.json"

    result = pipeline.import_unity_semantic_artifact("office", "")

    assert result["success"] is False
    assert result["reason_code"] == "missing_scene_dir"
    assert native.call is None
