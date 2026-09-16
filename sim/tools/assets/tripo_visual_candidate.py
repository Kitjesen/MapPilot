"""Compile inspected Tripo output into a fail-closed visual candidate manifest."""

from __future__ import annotations

import argparse
import base64
import binascii
import ctypes
import hashlib
import json
import math
import os
import re
import secrets
import stat
import struct
from collections.abc import Iterator, Mapping, Sequence
from contextlib import contextmanager
from pathlib import Path
from typing import Any

from sim.catalog.importers.contracts import digest_document
from sim.tools.assets.tripo_high_fidelity import PROFILE_PATH, validate_profile_document

_IDENTITY = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_AXES = {"x": 0, "y": 1, "z": 2}
_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
_JSON_LIMIT = 8 * 1024 * 1024
_IMAGE_LIMIT = 256 * 1024 * 1024
_MODEL_LIMIT = 512 * 1024 * 1024
_REQUIRED_BLOCKERS = {
    "license_and_usage_rights_unverified",
    "unreal_import_not_verified",
}


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{context} must be a JSON object")
    return value


def _absolute(path: Path) -> Path:
    return Path(os.path.abspath(os.fspath(path)))


def _is_reparse(path: Path) -> bool:
    metadata = os.lstat(path)
    return stat.S_ISLNK(metadata.st_mode) or bool(
        getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
    )


def _assert_link_free(path: Path, context: str) -> Path:
    candidate = _absolute(path)
    current = Path(candidate.anchor)
    for component in candidate.parts[1:]:
        current /= component
        try:
            if _is_reparse(current):
                raise ValueError(f"{context} contains a link or reparse point")
        except FileNotFoundError as exc:
            raise ValueError(f"{context} does not exist") from exc
        except OSError as exc:
            raise ValueError(f"cannot inspect {context}: {exc}") from exc
    return candidate


def _open_regular_nofollow(path: Path, context: str) -> int:
    candidate = _assert_link_free(path, context)
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0)
    if os.name == "nt":
        import ctypes
        import msvcrt

        create_file = ctypes.windll.kernel32.CreateFileW
        create_file.argtypes = [
            ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
        ]
        create_file.restype = ctypes.c_void_p
        handle = create_file(
            os.fspath(candidate),
            0x80000000,
            0x00000001 | 0x00000002,
            None,
            3,
            0x00200000 | 0x08000000,
            None,
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise ValueError(f"cannot open {context}")
        try:
            descriptor = msvcrt.open_osfhandle(int(handle), flags)
        except Exception:
            ctypes.windll.kernel32.CloseHandle(handle)
            raise
    else:
        try:
            descriptor = os.open(candidate, flags | getattr(os, "O_NOFOLLOW", 0))
        except OSError as exc:
            raise ValueError(f"cannot open {context}: {exc}") from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or bool(
            getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
        ):
            raise ValueError(f"{context} must identify one regular file")
    except Exception:
        os.close(descriptor)
        raise
    return descriptor


def _snapshot_file(
    path: Path,
    context: str,
    *,
    limit: int,
    keep_bytes: bool,
) -> tuple[dict[str, Any], bytes | None]:
    candidate = _absolute(path)
    descriptor = _open_regular_nofollow(candidate, context)
    try:
        before = os.fstat(descriptor)
        if before.st_size <= 0 or before.st_size > limit:
            raise ValueError(f"{context} has an invalid byte length")
        digest = hashlib.sha256()
        chunks: list[bytes] = []
        observed = 0
        while observed < before.st_size:
            chunk = os.read(descriptor, min(1024 * 1024, before.st_size - observed))
            if not chunk:
                break
            digest.update(chunk)
            if keep_bytes:
                chunks.append(chunk)
            observed += len(chunk)
        after = os.fstat(descriptor)
        path_after = os.stat(candidate, follow_symlinks=False)
        stable = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
        if (
            observed != before.st_size
            or stable != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
            or (before.st_dev, before.st_ino) != (path_after.st_dev, path_after.st_ino)
        ):
            raise ValueError(f"{context} changed while it was snapshotted")
        record = {
            "path": candidate.as_posix(),
            "bytes": before.st_size,
            "sha256": digest.hexdigest(),
        }
        return record, b"".join(chunks) if keep_bytes else None
    finally:
        os.close(descriptor)


def _validate_glb_v2(body: bytes, context: str) -> None:
    if len(body) < 12:
        raise ValueError(f"{context} has a truncated GLB header")
    magic, version, declared_length = struct.unpack_from("<4sII", body)
    if magic != b"glTF" or version != 2:
        raise ValueError(f"{context} must be one GLB v2 artifact")
    if declared_length != len(body):
        raise ValueError(f"{context} GLB declared length does not match its bytes")
    offset = 12
    chunk_index = 0
    document: Mapping[str, Any] | None = None
    bin_chunk: bytes | None = None
    while offset < len(body):
        if len(body) - offset < 8:
            raise ValueError(f"{context} has a truncated GLB chunk header")
        chunk_length, chunk_type = struct.unpack_from("<II", body, offset)
        offset += 8
        if chunk_length % 4 or chunk_length > len(body) - offset:
            raise ValueError(f"{context} has an invalid GLB chunk boundary")
        chunk = body[offset : offset + chunk_length]
        offset += chunk_length
        if chunk_index == 0:
            if chunk_type != 0x4E4F534A:
                raise ValueError(f"{context} first GLB chunk must be JSON")
            try:
                raw_document = json.loads(chunk.rstrip(b" \t\r\n\x00").decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError(f"{context} first GLB JSON chunk is invalid") from exc
            if (
                not isinstance(raw_document, Mapping)
                or not isinstance(raw_document.get("asset"), Mapping)
                or raw_document["asset"].get("version") != "2.0"
            ):
                raise ValueError(f"{context} GLB JSON must declare asset.version 2.0")
            document = raw_document
            stack = [document]
            while stack:
                value = stack.pop()
                if isinstance(value, Mapping):
                    for key, child in value.items():
                        if key == "uri" and isinstance(child, str) and not child.startswith("data:"):
                            raise ValueError(f"{context} GLB references an external URI")
                        stack.append(child)
                elif isinstance(value, list):
                    stack.extend(value)
        elif chunk_index != 1 or chunk_type != 0x004E4942:
            raise ValueError(f"{context} has an unsupported GLB chunk layout")
        else:
            bin_chunk = chunk
        chunk_index += 1
    if offset != len(body) or chunk_index == 0:
        raise ValueError(f"{context} has an incomplete GLB chunk table")
    assert document is not None
    raw_buffers = document.get("buffers", [])
    if not isinstance(raw_buffers, list):
        raise ValueError(f"{context} GLB buffers must be a list")
    buffer_lengths: list[int] = []
    bin_owner_count = 0
    for index, raw_buffer in enumerate(raw_buffers):
        buffer = _mapping(raw_buffer, f"{context}.buffers[{index}]")
        byte_length = _non_negative_int(
            buffer.get("byteLength"), f"{context}.buffers[{index}].byteLength"
        )
        uri = buffer.get("uri")
        if uri is None:
            bin_owner_count += 1
            if bin_chunk is None:
                raise ValueError(f"{context} GLB buffer without URI requires one BIN chunk")
            padding = len(bin_chunk) - byte_length
            if padding < 0 or padding > 3 or any(bin_chunk[byte_length:]):
                raise ValueError(f"{context} GLB BIN length does not match buffer.byteLength")
            buffer_lengths.append(byte_length)
        else:
            if not isinstance(uri, str) or not uri.startswith("data:") or ";base64," not in uri:
                raise ValueError(f"{context} GLB references an external URI")
            try:
                payload = base64.b64decode(uri.split(",", 1)[1], validate=True)
            except (ValueError, binascii.Error) as exc:
                raise ValueError(f"{context} GLB has an invalid data URI") from exc
            if len(payload) != byte_length:
                raise ValueError(f"{context} GLB data URI length does not match buffer.byteLength")
            buffer_lengths.append(byte_length)
    if bin_owner_count > 1 or (bin_chunk is not None and bin_owner_count != 1):
        raise ValueError(f"{context} GLB must have exactly one owner for its BIN chunk")
    raw_views = document.get("bufferViews", [])
    if not isinstance(raw_views, list):
        raise ValueError(f"{context} GLB bufferViews must be a list")
    for index, raw_view in enumerate(raw_views):
        view = _mapping(raw_view, f"{context}.bufferViews[{index}]")
        buffer_index = _non_negative_int(view.get("buffer"), f"{context}.bufferViews[{index}].buffer")
        if buffer_index >= len(buffer_lengths):
            raise ValueError(f"{context} GLB bufferView references a missing buffer")
        start = _non_negative_int(view.get("byteOffset", 0), f"{context}.bufferViews[{index}].byteOffset")
        length = _positive_int(view.get("byteLength"), f"{context}.bufferViews[{index}].byteLength")
        if start + length > buffer_lengths[buffer_index]:
            raise ValueError(f"{context} GLB bufferView exceeds its buffer boundary")
    for collection, field in (("accessors", "bufferView"), ("images", "bufferView")):
        values = document.get(collection, [])
        if not isinstance(values, list):
            raise ValueError(f"{context} GLB {collection} must be a list")
        for index, raw_value in enumerate(values):
            value = _mapping(raw_value, f"{context}.{collection}[{index}]")
            if collection == "images" and (("uri" in value) == ("bufferView" in value)):
                raise ValueError(
                    f"{context} GLB image must use exactly one URI or bufferView"
                )
            if field in value:
                view_index = _non_negative_int(value[field], f"{context}.{collection}[{index}].{field}")
                if view_index >= len(raw_views):
                    raise ValueError(f"{context} GLB {collection} references a missing bufferView")
            if collection == "images" and "uri" in value:
                uri = value["uri"]
                if not isinstance(uri, str) or not uri.startswith("data:"):
                    raise ValueError(f"{context} GLB image references an external URI")
    raw_textures = document.get("textures", [])
    if not isinstance(raw_textures, list):
        raise ValueError(f"{context} GLB textures must be a list")
    raw_images = document.get("images", [])
    for index, raw_texture in enumerate(raw_textures):
        texture = _mapping(raw_texture, f"{context}.textures[{index}]")
        if "source" in texture:
            source = _non_negative_int(texture["source"], f"{context}.textures[{index}].source")
            if source >= len(raw_images):
                raise ValueError(f"{context} GLB texture references a missing image")


def _content_record(record: Mapping[str, Any], *, category: str, name: str, suffix: str) -> dict[str, Any]:
    sha256 = _sha256(record.get("sha256"), f"{name}.sha256")
    return {
        "path": f"{category}/{name}.{sha256}{suffix}",
        "bytes": record["bytes"],
        "sha256": sha256,
    }


def _load_mapping_snapshot(path: Path, context: str) -> tuple[Mapping[str, Any], dict[str, Any]]:
    record, body = _snapshot_file(path, context, limit=_JSON_LIMIT, keep_bytes=True)
    assert body is not None
    try:
        return _mapping(json.loads(body.decode("utf-8")), context), record
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read {context}: {exc}") from exc


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{context} must be a non-empty string")
    return value


def _identity(value: Any, context: str) -> str:
    result = _string(value, context)
    if _IDENTITY.fullmatch(result) is None:
        raise ValueError(f"{context} is not a stable identifier")
    return result


def _positive_vector(value: Any, context: str) -> list[float]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != 3:
        raise ValueError(f"{context} must contain exactly three numbers")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError(f"{context}[{index}] must be numeric")
        number = float(item)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{context}[{index}] must be finite and positive")
        result.append(number)
    return result


def _non_negative_int(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{context} must be a non-negative integer")
    return value


def _positive_int(value: Any, context: str) -> int:
    result = _non_negative_int(value, context)
    if result == 0:
        raise ValueError(f"{context} must be a positive integer")
    return result


def _positive_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{context} must be finite and positive")
    return result


def _exact_keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    observed = set(value)
    if observed != expected:
        missing = sorted(expected - observed)
        unexpected = sorted(observed - expected)
        raise ValueError(
            f"{context} keys must match the contract; "
            f"missing={missing}, unexpected={unexpected}"
        )


def _portable_artifact_path(value: Any, context: str) -> str:
    path = _string(value, context)
    components = path.split("/")
    if (
        path.startswith("/")
        or re.match(r"^[A-Za-z]:", path)
        or ":" in path
        or "\\" in path
        or any(character.isspace() for character in path)
        or "//" in path
        or path.endswith("/")
        or any(component in {"", ".", ".."} for component in components)
    ):
        raise ValueError(f"{context} must be one canonical relative path")
    return path


def _sha256(value: Any, context: str) -> str:
    result = _string(value, context)
    if _SHA256.fullmatch(result) is None:
        raise ValueError(f"{context} must be one lowercase SHA-256 digest")
    return result


def _evidence_record(value: Any, context: str) -> dict[str, Any]:
    record = _mapping(value, context)
    _exact_keys(record, {"path", "bytes", "sha256"}, context)
    raw_path = _string(record.get("path"), f"{context}.path")
    path = _portable_artifact_path(raw_path, f"{context}.path")
    sha256 = _sha256(record.get("sha256"), f"{context}.sha256")
    if sha256 not in Path(path).name:
        raise ValueError(f"{context}.path must be content-addressed by its SHA-256")
    return {
        "path": path,
        "bytes": _positive_int(record.get("bytes"), f"{context}.bytes"),
        "sha256": sha256,
    }


def _game_asset_path(value: str) -> str:
    path = _string(value, "unreal_asset_path")
    if (
        not path.startswith("/Game/")
        or "\\" in path
        or any(character.isspace() for character in path)
        or "//" in path
        or any(component in {".", ".."} for component in path.split("/"))
    ):
        raise ValueError("unreal_asset_path must be one canonical /Game asset path")
    return path


def _axis_order(value: Sequence[str]) -> tuple[str, str, str]:
    if tuple(sorted(value)) != ("x", "y", "z"):
        raise ValueError("source_axis_order must be one permutation of x, y, z")
    return value[0], value[1], value[2]


def _validate_evidence_documents(
    *,
    task: Mapping[str, Any],
    inspection: Mapping[str, Any],
    assessment: Mapping[str, Any],
    profile_document: Mapping[str, Any],
    asset_id: str,
    model_record: Mapping[str, Any],
    source_records: Sequence[Mapping[str, Any]],
) -> None:
    profile = validate_profile_document(profile_document)
    _exact_keys(task, {"task_id", "type", "status", "input", "credits_consumed"}, "task")
    _exact_keys(
        inspection,
        {"schema", "source", "aabb_m", "mesh_objects", "vertices", "triangles", "uv_layers", "non_manifold_edges"},
        "inspection",
    )
    _exact_keys(
        assessment,
        {"schema", "asset_id", "task_id", "artifact", "qualification"},
        "assessment",
    )
    if inspection.get("schema") != "lingtu.sim.tripo-blender-inspection.v1":
        raise ValueError("inspection.schema must be lingtu.sim.tripo-blender-inspection.v1")
    if assessment.get("schema") != "lingtu.sim.visual-candidate-assessment.v1":
        raise ValueError("assessment.schema must be lingtu.sim.visual-candidate-assessment.v1")
    if task.get("status") != "success" or task.get("type") != "multiview_to_model":
        raise ValueError("Tripo task must be one successful multiview_to_model task")
    task_id = _identity(task.get("task_id"), "task.task_id")
    task_input = _mapping(task.get("input"), "task.input")
    _exact_keys(
        task_input,
        {
            "model_version", "model_seed", "texture_seed", "geometry_quality",
            "texture", "pbr", "texture_quality", "auto_size", "export_uv",
            "face_limit", "smart_low_poly", "source_images",
        },
        "task.input",
    )
    generation = _mapping(profile.get("generation"), "profile.generation")
    request_contract = {
        "model_version": generation["model"],
        "geometry_quality": generation["geometry_quality"],
        "texture": generation["texture"],
        "pbr": generation["pbr"],
        "texture_quality": generation["texture_quality"],
        "auto_size": generation["auto_size"],
        "export_uv": generation["export_uv"],
        "face_limit": generation["face_limit"],
        "smart_low_poly": generation["smart_low_poly"],
    }
    if any(task_input.get(key) != expected for key, expected in request_contract.items()):
        raise ValueError("task request does not match the complete bound generation profile")
    _non_negative_int(task_input.get("model_seed"), "task.input.model_seed")
    _non_negative_int(task_input.get("texture_seed"), "task.input.texture_seed")
    inspection_source = _mapping(inspection.get("source"), "inspection.source")
    assessment_artifact = _mapping(assessment.get("artifact"), "assessment.artifact")
    _exact_keys(inspection_source, {"task_id", "bytes", "sha256"}, "inspection.source")
    _exact_keys(assessment_artifact, {"path", "bytes", "sha256"}, "assessment.artifact")
    if (
        inspection_source.get("task_id") != task_id
        or assessment.get("task_id") != task_id
        or assessment.get("asset_id") != asset_id
    ):
        raise ValueError("candidate task and asset identity do not agree")
    _portable_artifact_path(assessment_artifact.get("path"), "assessment.artifact.path")
    for evidence in (inspection_source, assessment_artifact):
        if (
            evidence.get("bytes") != model_record.get("bytes")
            or evidence.get("sha256") != model_record.get("sha256")
        ):
            raise ValueError("model artifact identity does not match inspection and assessment")
    qualification = _mapping(assessment.get("qualification"), "assessment.qualification")
    _exact_keys(
        qualification,
        {
            "background_or_midground_visual", "hero_closeup_visual",
            "unreal_collision_mesh", "mujoco_collision_authority",
        },
        "assessment.qualification",
    )
    if qualification.get("unreal_collision_mesh") != "reject":
        raise ValueError("assessment must reject the generated render mesh as Unreal collision")
    if qualification.get("mujoco_collision_authority") != "required":
        raise ValueError("assessment must preserve MuJoCo collision authority")
    raw_sources = task_input.get("source_images")
    if not isinstance(raw_sources, list) or not raw_sources:
        raise ValueError("task.input.source_images must be a non-empty list")
    expected_by_view: dict[str, tuple[int, str]] = {}
    for index, raw_source in enumerate(raw_sources):
        source = _mapping(raw_source, f"task.input.source_images[{index}]")
        _exact_keys(source, {"view", "path", "bytes", "sha256"}, f"task.input.source_images[{index}]")
        view = _identity(source.get("view"), f"task.input.source_images[{index}].view")
        if view in expected_by_view or view not in {"front", "left", "back", "right"}:
            raise ValueError("task source image views must be supported and unique")
        _portable_artifact_path(source.get("path"), f"task.input.source_images[{index}].path")
        expected_by_view[view] = (
            _positive_int(source.get("bytes"), f"task.input.source_images[{index}].bytes"),
            _sha256(source.get("sha256"), f"task.input.source_images[{index}].sha256"),
        )
    observed_by_view = {
        _identity(record.get("view"), "source evidence view"): (
            _positive_int(record.get("bytes"), "source evidence bytes"),
            _sha256(record.get("sha256"), "source evidence sha256"),
        )
        for record in source_records
    }
    if expected_by_view != observed_by_view or "front" not in observed_by_view:
        raise ValueError("source image evidence does not match the task request")
    if len(observed_by_view) < profile["input"]["minimum_views"]:
        raise ValueError("source image evidence does not satisfy the multiview profile")


def validate_visual_candidate_manifest(document: Mapping[str, Any]) -> dict[str, Any]:
    """Validate and canonicalize one quarantined Tripo visual candidate."""

    manifest = _mapping(document, "visual candidate manifest")
    _exact_keys(
        manifest,
        {"schema", "asset", "provenance", "evidence", "geometry", "binding", "qualification", "digest"},
        "visual candidate manifest",
    )
    if manifest.get("schema") != "lingtu.sim.visual-asset-candidate.v1":
        raise ValueError(
            "schema must be lingtu.sim.visual-asset-candidate.v1"
        )
    digest = _sha256(manifest.get("digest"), "digest")
    body = {key: value for key, value in manifest.items() if key != "digest"}
    if digest != digest_document(body):
        raise ValueError("visual candidate manifest digest does not match its content")

    asset = _mapping(manifest.get("asset"), "asset")
    _exact_keys(asset, {"id", "kind", "role", "artifact"}, "asset")
    asset_id = _identity(asset.get("id"), "asset.id")
    if asset.get("kind") != "static_mesh":
        raise ValueError("asset.kind must be static_mesh")
    if asset.get("role") != "world_visual_candidate":
        raise ValueError("asset.role must be world_visual_candidate")
    artifact = _mapping(asset.get("artifact"), "asset.artifact")
    _exact_keys(artifact, {"path", "bytes", "sha256"}, "asset.artifact")
    artifact_path = _portable_artifact_path(artifact.get("path"), "asset.artifact.path")
    artifact_bytes = _positive_int(artifact.get("bytes"), "asset.artifact.bytes")
    artifact_sha256 = _sha256(artifact.get("sha256"), "asset.artifact.sha256")
    if artifact_sha256 not in Path(artifact_path).name or not artifact_path.endswith(".glb"):
        raise ValueError("asset.artifact.path must be one content-addressed GLB path")

    provenance = _mapping(manifest.get("provenance"), "provenance")
    _exact_keys(
        provenance,
        {
            "provider",
            "task_id",
            "task_type",
            "model",
            "model_seed",
            "texture_seed",
            "credits_consumed",
        },
        "provenance",
    )
    if provenance.get("provider") != "tripo3d":
        raise ValueError("provenance.provider must be tripo3d")
    task_id = _identity(provenance.get("task_id"), "provenance.task_id")
    if provenance.get("task_type") != "multiview_to_model":
        raise ValueError("provenance.task_type must be multiview_to_model")
    model = _identity(provenance.get("model"), "provenance.model")
    model_seed = _non_negative_int(provenance.get("model_seed"), "provenance.model_seed")
    texture_seed = _non_negative_int(
        provenance.get("texture_seed"), "provenance.texture_seed"
    )
    credits_consumed = _non_negative_int(
        provenance.get("credits_consumed"), "provenance.credits_consumed"
    )

    evidence = _mapping(manifest.get("evidence"), "evidence")
    _exact_keys(
        evidence,
        {"task", "inspection", "assessment", "profile", "source_images"},
        "evidence",
    )
    task_evidence = _evidence_record(evidence.get("task"), "evidence.task")
    inspection_evidence = _evidence_record(
        evidence.get("inspection"), "evidence.inspection"
    )
    assessment_evidence = _evidence_record(
        evidence.get("assessment"), "evidence.assessment"
    )
    profile_evidence = _evidence_record(evidence.get("profile"), "evidence.profile")
    raw_images = evidence.get("source_images")
    if not isinstance(raw_images, list) or not raw_images:
        raise ValueError("evidence.source_images must be a non-empty list")
    source_images: list[dict[str, Any]] = []
    observed_views: set[str] = set()
    for index, raw_image in enumerate(raw_images):
        image = _mapping(raw_image, f"evidence.source_images[{index}]")
        _exact_keys(
            image,
            {"view", "path", "bytes", "sha256"},
            f"evidence.source_images[{index}]",
        )
        view = _identity(image.get("view"), f"evidence.source_images[{index}].view")
        if view not in {"front", "left", "back", "right"} or view in observed_views:
            raise ValueError("evidence.source_images views must be supported and unique")
        observed_views.add(view)
        record = _evidence_record(
            {key: image[key] for key in ("path", "bytes", "sha256")},
            f"evidence.source_images[{index}]",
        )
        source_images.append({"view": view, **record})
    if "front" not in observed_views:
        raise ValueError("evidence.source_images must include the front view")

    geometry = _mapping(manifest.get("geometry"), "geometry")
    _exact_keys(
        geometry,
        {
            "source_aabb_dimensions_m",
            "mesh_objects",
            "vertices",
            "triangles",
            "uv_layers",
            "non_manifold_edges",
            "normalization",
        },
        "geometry",
    )
    source_dimensions = _positive_vector(
        geometry.get("source_aabb_dimensions_m"),
        "geometry.source_aabb_dimensions_m",
    )
    mesh_objects = _positive_int(geometry.get("mesh_objects"), "geometry.mesh_objects")
    vertices = _positive_int(geometry.get("vertices"), "geometry.vertices")
    triangles = _positive_int(geometry.get("triangles"), "geometry.triangles")
    uv_layers = _non_negative_int(geometry.get("uv_layers"), "geometry.uv_layers")
    non_manifold_edges = _non_negative_int(
        geometry.get("non_manifold_edges"), "geometry.non_manifold_edges"
    )
    normalization = _mapping(geometry.get("normalization"), "geometry.normalization")
    _exact_keys(
        normalization,
        {"policy", "source_axis_order", "uniform_scale", "normalized_dimensions_m"},
        "geometry.normalization",
    )
    if normalization.get("policy") != "uniform_fit_inside_mujoco_proxy":
        raise ValueError(
            "geometry.normalization.policy must be uniform_fit_inside_mujoco_proxy"
        )
    raw_axis_order = normalization.get("source_axis_order")
    if (
        not isinstance(raw_axis_order, Sequence)
        or isinstance(raw_axis_order, (str, bytes))
        or len(raw_axis_order) != 3
        or not all(isinstance(axis, str) for axis in raw_axis_order)
    ):
        raise ValueError(
            "geometry.normalization.source_axis_order must contain three axes"
        )
    axis_order = _axis_order(tuple(raw_axis_order))
    uniform_scale = _positive_number(
        normalization.get("uniform_scale"), "geometry.normalization.uniform_scale"
    )
    normalized_dimensions = _positive_vector(
        normalization.get("normalized_dimensions_m"),
        "geometry.normalization.normalized_dimensions_m",
    )

    binding = _mapping(manifest.get("binding"), "binding")
    _exact_keys(binding, {"world_entity_id", "semantic_class", "physics", "unreal"}, "binding")
    world_entity_id = _identity(binding.get("world_entity_id"), "binding.world_entity_id")
    semantic_class = _identity(binding.get("semantic_class"), "binding.semantic_class")
    physics = _mapping(binding.get("physics"), "binding.physics")
    _exact_keys(physics, {"authority", "proxy", "render_mesh_is_collider"}, "binding.physics")
    if physics.get("authority") != "mujoco":
        raise ValueError("binding.physics.authority must be mujoco")
    if physics.get("render_mesh_is_collider") is not False:
        raise ValueError("binding.physics.render_mesh_is_collider must be false")
    proxy = _mapping(physics.get("proxy"), "binding.physics.proxy")
    _exact_keys(proxy, {"shape", "size_m"}, "binding.physics.proxy")
    if proxy.get("shape") != "box":
        raise ValueError("binding.physics.proxy.shape must be box")
    proxy_size = _positive_vector(proxy.get("size_m"), "binding.physics.proxy.size_m")

    mapped_dimensions = [source_dimensions[_AXES[axis]] for axis in axis_order]
    expected_scale = min(
        limit / observed for limit, observed in zip(proxy_size, mapped_dimensions)
    )
    if not math.isclose(uniform_scale, expected_scale, rel_tol=1.0e-12, abs_tol=1.0e-12):
        raise ValueError(
            "geometry.normalization.uniform_scale must uniformly fit the MuJoCo proxy"
        )
    expected_dimensions = [dimension * uniform_scale for dimension in mapped_dimensions]
    if any(
        not math.isclose(observed, expected, rel_tol=1.0e-12, abs_tol=1.0e-12)
        for observed, expected in zip(normalized_dimensions, expected_dimensions)
    ):
        raise ValueError(
            "geometry.normalization.normalized_dimensions_m does not match axis mapping and scale"
        )
    if any(
        observed > limit + 1.0e-12
        for observed, limit in zip(normalized_dimensions, proxy_size)
    ):
        raise ValueError("normalized render mesh exceeds its MuJoCo proxy")

    unreal = _mapping(binding.get("unreal"), "binding.unreal")
    _exact_keys(
        unreal,
        {
            "asset_path",
            "collision_profile",
            "collision_enabled",
            "simulate_physics",
            "generate_overlap_events",
            "can_ever_affect_navigation",
            "auto_spawn_in_production_map",
        },
        "binding.unreal",
    )
    unreal_asset_path = _game_asset_path(
        _string(unreal.get("asset_path"), "binding.unreal.asset_path")
    )
    if unreal.get("collision_profile") != "NoCollision":
        raise ValueError("binding.unreal.collision_profile must be NoCollision")
    for field in (
        "collision_enabled",
        "simulate_physics",
        "generate_overlap_events",
        "can_ever_affect_navigation",
        "auto_spawn_in_production_map",
    ):
        if unreal.get(field) is not False:
            raise ValueError(f"binding.unreal.{field} must be false")

    qualification = _mapping(manifest.get("qualification"), "qualification")
    _exact_keys(qualification, {"state", "blockers", "promotion_target"}, "qualification")
    if qualification.get("state") != "QUARANTINED":
        raise ValueError("qualification.state must be QUARANTINED")
    raw_blockers = qualification.get("blockers")
    if (
        not isinstance(raw_blockers, list)
        or not raw_blockers
        or not all(isinstance(blocker, str) and blocker for blocker in raw_blockers)
    ):
        raise ValueError("qualification.blockers must be a non-empty string list")
    blockers = [_identity(blocker, "qualification.blockers[]") for blocker in raw_blockers]
    if blockers != sorted(set(blockers)):
        raise ValueError("qualification.blockers must be sorted and unique")
    missing_required = sorted(_REQUIRED_BLOCKERS - set(blockers))
    if missing_required:
        raise ValueError(f"qualification.required blockers are missing: {missing_required}")
    if qualification.get("promotion_target") != "WorldPackage.visual facet":
        raise ValueError(
            "qualification.promotion_target must be WorldPackage.visual facet"
        )

    canonical_body: dict[str, Any] = {
        "schema": "lingtu.sim.visual-asset-candidate.v1",
        "asset": {
            "id": asset_id,
            "kind": "static_mesh",
            "role": "world_visual_candidate",
            "artifact": {
                "path": artifact_path,
                "bytes": artifact_bytes,
                "sha256": artifact_sha256,
            },
        },
        "provenance": {
            "provider": "tripo3d",
            "task_id": task_id,
            "task_type": "multiview_to_model",
            "model": model,
            "model_seed": model_seed,
            "texture_seed": texture_seed,
            "credits_consumed": credits_consumed,
        },
        "evidence": {
            "task": task_evidence,
            "inspection": inspection_evidence,
            "assessment": assessment_evidence,
            "profile": profile_evidence,
            "source_images": source_images,
        },
        "geometry": {
            "source_aabb_dimensions_m": source_dimensions,
            "mesh_objects": mesh_objects,
            "vertices": vertices,
            "triangles": triangles,
            "uv_layers": uv_layers,
            "non_manifold_edges": non_manifold_edges,
            "normalization": {
                "policy": "uniform_fit_inside_mujoco_proxy",
                "source_axis_order": list(axis_order),
                "uniform_scale": uniform_scale,
                "normalized_dimensions_m": normalized_dimensions,
            },
        },
        "binding": {
            "world_entity_id": world_entity_id,
            "semantic_class": semantic_class,
            "physics": {
                "authority": "mujoco",
                "proxy": {"shape": "box", "size_m": proxy_size},
                "render_mesh_is_collider": False,
            },
            "unreal": {
                "asset_path": unreal_asset_path,
                "collision_profile": "NoCollision",
                "collision_enabled": False,
                "simulate_physics": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
                "auto_spawn_in_production_map": False,
            },
        },
        "qualification": {
            "state": "QUARANTINED",
            "blockers": blockers,
            "promotion_target": "WorldPackage.visual facet",
        },
    }
    return {**canonical_body, "digest": digest_document(canonical_body)}


def validate_visual_candidate_closure(manifest_path: Path) -> dict[str, Any]:
    """Revalidate a published candidate and every content-addressed file."""

    document, _ = _load_mapping_snapshot(manifest_path, "candidate manifest")
    manifest = validate_visual_candidate_manifest(document)
    root = _absolute(Path(manifest_path).parent)
    artifact = _mapping(_mapping(manifest["asset"], "asset")["artifact"], "asset.artifact")
    evidence = _mapping(manifest["evidence"], "evidence")
    snapshots: dict[str, Mapping[str, Any]] = {}

    def snapshot_record(
        record: Mapping[str, Any],
        *,
        limit: int,
        keep_bytes: bool,
    ) -> bytes | None:
        relative = _portable_artifact_path(record.get("path"), "closure path")
        path = _absolute(root / Path(relative))
        try:
            path.relative_to(root)
        except ValueError as exc:
            raise ValueError("candidate closure path escapes its package root") from exc
        observed, body = _snapshot_file(path, relative, limit=limit, keep_bytes=keep_bytes)
        if observed["bytes"] != record.get("bytes") or observed["sha256"] != record.get("sha256"):
            raise ValueError(f"candidate closure identity mismatch: {relative}")
        return body

    model_body = snapshot_record(artifact, limit=_MODEL_LIMIT, keep_bytes=True)
    assert model_body is not None
    _validate_glb_v2(model_body, str(artifact["path"]))
    for name in ("task", "inspection", "assessment", "profile"):
        record = _mapping(evidence[name], f"evidence.{name}")
        body = snapshot_record(record, limit=_JSON_LIMIT, keep_bytes=True)
        assert body is not None
        try:
            snapshots[name] = _mapping(json.loads(body.decode("utf-8")), name)
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"candidate {name} evidence is not valid JSON") from exc
    source_records = [
        _mapping(image, "evidence.source_images[]")
        for image in evidence["source_images"]
    ]
    for image in source_records:
        snapshot_record(image, limit=_IMAGE_LIMIT, keep_bytes=False)
    asset = _mapping(manifest["asset"], "asset")
    _validate_evidence_documents(
        task=snapshots["task"],
        inspection=snapshots["inspection"],
        assessment=snapshots["assessment"],
        profile_document=snapshots["profile"],
        asset_id=_identity(asset.get("id"), "asset.id"),
        model_record=artifact,
        source_records=source_records,
    )
    return manifest


def build_visual_candidate_manifest(
    *,
    asset_id: str,
    world_entity_id: str,
    semantic_class: str,
    model_path: Path,
    task_path: Path,
    inspection_path: Path,
    assessment_path: Path,
    source_axis_order: Sequence[str],
    proxy_size_m: Sequence[float],
    unreal_asset_path: str,
    profile_path: Path = PROFILE_PATH,
) -> dict[str, Any]:
    """Bind one visual mesh to a distinct MuJoCo proxy without promoting it."""

    canonical_asset_id = _identity(asset_id, "asset_id")
    canonical_entity_id = _identity(world_entity_id, "world_entity_id")
    canonical_semantic_class = _identity(semantic_class, "semantic_class")
    canonical_unreal_path = _game_asset_path(unreal_asset_path)
    canonical_axis_order = _axis_order(tuple(source_axis_order))
    canonical_proxy_size = _positive_vector(proxy_size_m, "proxy_size_m")

    if Path(model_path).suffix.lower() != ".glb":
        raise ValueError("model_path must identify one self-contained GLB")
    task, task_evidence = _load_mapping_snapshot(task_path, "task")
    inspection, inspection_evidence = _load_mapping_snapshot(inspection_path, "inspection")
    assessment, assessment_evidence = _load_mapping_snapshot(assessment_path, "assessment")
    profile_document, profile_evidence = _load_mapping_snapshot(profile_path, "profile")
    profile = validate_profile_document(profile_document)
    _exact_keys(task, {"task_id", "type", "status", "input", "credits_consumed"}, "task")
    _exact_keys(
        inspection,
        {"schema", "source", "aabb_m", "mesh_objects", "vertices", "triangles", "uv_layers", "non_manifold_edges"},
        "inspection",
    )
    _exact_keys(
        assessment,
        {"schema", "asset_id", "task_id", "artifact", "qualification"},
        "assessment",
    )
    if inspection.get("schema") != "lingtu.sim.tripo-blender-inspection.v1":
        raise ValueError("inspection.schema must be lingtu.sim.tripo-blender-inspection.v1")
    if assessment.get("schema") != "lingtu.sim.visual-candidate-assessment.v1":
        raise ValueError("assessment.schema must be lingtu.sim.visual-candidate-assessment.v1")
    if task.get("status") != "success" or task.get("type") != "multiview_to_model":
        raise ValueError("Tripo task must be one successful multiview_to_model task")
    task_id = _string(task.get("task_id"), "task.task_id")
    inspection_source = _mapping(inspection.get("source"), "inspection.source")
    if (
        inspection_source.get("task_id") != task_id
        or assessment.get("task_id") != task_id
        or assessment.get("asset_id") != canonical_asset_id
    ):
        raise ValueError("candidate task and asset identity do not agree")

    model_evidence, model_body = _snapshot_file(
        model_path,
        "model artifact",
        limit=_MODEL_LIMIT,
        keep_bytes=True,
    )
    assert model_body is not None
    _validate_glb_v2(model_body, "model artifact")
    artifact_bytes = _positive_int(model_evidence["bytes"], "model artifact bytes")
    artifact_sha256 = _sha256(model_evidence["sha256"], "model artifact sha256")
    inspection_source = _mapping(inspection.get("source"), "inspection.source")
    assessment_artifact = _mapping(assessment.get("artifact"), "assessment.artifact")
    _exact_keys(inspection_source, {"task_id", "bytes", "sha256"}, "inspection.source")
    _exact_keys(assessment_artifact, {"path", "bytes", "sha256"}, "assessment.artifact")
    if assessment_artifact.get("path") != Path(model_path).name:
        raise ValueError("assessment artifact must identify the self-contained GLB")
    if any(
        size != artifact_bytes or sha256 != artifact_sha256
        for size, sha256 in (
            (inspection_source.get("bytes"), inspection_source.get("sha256")),
            (assessment_artifact.get("bytes"), assessment_artifact.get("sha256")),
        )
    ):
        raise ValueError("model artifact identity does not match inspection and assessment")
    source_dimensions = _positive_vector(
        _mapping(inspection.get("aabb_m"), "inspection.aabb_m").get("dimensions"),
        "inspection.aabb_m.dimensions",
    )
    mapped_dimensions = [source_dimensions[_AXES[axis]] for axis in canonical_axis_order]
    uniform_scale = min(
        limit / observed
        for limit, observed in zip(canonical_proxy_size, mapped_dimensions)
    )
    normalized_dimensions = [dimension * uniform_scale for dimension in mapped_dimensions]
    if any(
        observed > limit + 1.0e-12
        for observed, limit in zip(normalized_dimensions, canonical_proxy_size)
    ):
        raise ValueError("normalized render mesh exceeds its MuJoCo proxy")

    task_input = _mapping(task.get("input"), "task.input")
    _exact_keys(
        task_input,
        {
            "model_version", "model_seed", "texture_seed", "geometry_quality",
            "texture", "pbr", "texture_quality", "auto_size", "export_uv",
            "face_limit", "smart_low_poly", "source_images",
        },
        "task.input",
    )
    generation = _mapping(profile.get("generation"), "profile.generation")
    request_contract = {
        "model_version": generation["model"],
        "geometry_quality": generation["geometry_quality"],
        "texture": generation["texture"],
        "pbr": generation["pbr"],
        "texture_quality": generation["texture_quality"],
        "auto_size": generation["auto_size"],
        "export_uv": generation["export_uv"],
        "face_limit": generation["face_limit"],
        "smart_low_poly": generation["smart_low_poly"],
    }
    if any(task_input.get(key) != expected for key, expected in request_contract.items()):
        raise ValueError("task request does not match the complete bound generation profile")
    raw_source_images = task_input.get("source_images")
    if not isinstance(raw_source_images, list) or not raw_source_images:
        raise ValueError("task.input.source_images must be a non-empty list")
    source_evidence: list[dict[str, Any]] = []
    source_root = _absolute(Path(task_path).parent)
    views: set[str] = set()
    for index, raw_source in enumerate(raw_source_images):
        source = _mapping(raw_source, f"task.input.source_images[{index}]")
        _exact_keys(
            source,
            {"view", "path", "bytes", "sha256"},
            f"task.input.source_images[{index}]",
        )
        view = _identity(source.get("view"), f"task.input.source_images[{index}].view")
        if view not in {"front", "left", "back", "right"} or view in views:
            raise ValueError("task source image views must be supported and unique")
        views.add(view)
        relative = _portable_artifact_path(
            source.get("path"), f"task.input.source_images[{index}].path"
        )
        source_path = _absolute(source_root / Path(relative))
        try:
            source_path.relative_to(source_root)
        except ValueError as exc:
            raise ValueError("task source image escapes its evidence root") from exc
        record, _ = _snapshot_file(
            source_path,
            f"source image {view}",
            limit=_IMAGE_LIMIT,
            keep_bytes=False,
        )
        if record["bytes"] != source.get("bytes") or record["sha256"] != source.get("sha256"):
            raise ValueError(f"source image {view} identity does not match task evidence")
        source_evidence.append(
            {
                "view": view,
                **_content_record(
                    record,
                    category="sources",
                    name=view,
                    suffix=Path(relative).suffix.lower(),
                ),
            }
        )
    if "front" not in views or len(views) < profile["input"]["minimum_views"]:
        raise ValueError("task source images do not satisfy the bound multiview profile")
    _validate_evidence_documents(
        task=task,
        inspection=inspection,
        assessment=assessment,
        profile_document=profile_document,
        asset_id=canonical_asset_id,
        model_record=model_evidence,
        source_records=source_evidence,
    )
    blockers = {
        "license_and_usage_rights_unverified",
        "pbr_materials_missing",
        "unreal_import_not_verified",
    }
    non_manifold_edges = _non_negative_int(
        inspection.get("non_manifold_edges"),
        "inspection.non_manifold_edges",
    )
    if non_manifold_edges > 0:
        blockers.add("topology_review_required")
    assessment_qualification = _mapping(
        assessment.get("qualification"),
        "assessment.qualification",
    )
    _exact_keys(
        assessment_qualification,
        {
            "background_or_midground_visual", "hero_closeup_visual",
            "unreal_collision_mesh", "mujoco_collision_authority",
        },
        "assessment.qualification",
    )
    if assessment_qualification.get("hero_closeup_visual") != "pass":
        blockers.add("hero_closeup_not_qualified")
    if assessment_qualification.get("unreal_collision_mesh") != "reject":
        raise ValueError("assessment must reject the generated render mesh as Unreal collision")
    if assessment_qualification.get("mujoco_collision_authority") != "required":
        raise ValueError("assessment must preserve MuJoCo collision authority")

    body: dict[str, Any] = {
        "schema": "lingtu.sim.visual-asset-candidate.v1",
        "asset": {
            "id": canonical_asset_id,
            "kind": "static_mesh",
            "role": "world_visual_candidate",
            "artifact": {
                "path": _content_record(
                    model_evidence,
                    category="artifacts",
                    name="model",
                    suffix=".glb",
                )["path"],
                "bytes": artifact_bytes,
                "sha256": artifact_sha256,
            },
        },
        "provenance": {
            "provider": "tripo3d",
            "task_id": task_id,
            "task_type": "multiview_to_model",
            "model": task_input.get("model_version"),
            "model_seed": task_input.get("model_seed"),
            "texture_seed": task_input.get("texture_seed"),
            "credits_consumed": task.get("credits_consumed"),
        },
        "evidence": {
            "task": _content_record(task_evidence, category="evidence", name="task", suffix=".json"),
            "inspection": _content_record(inspection_evidence, category="evidence", name="inspection", suffix=".json"),
            "assessment": _content_record(assessment_evidence, category="evidence", name="assessment", suffix=".json"),
            "profile": _content_record(profile_evidence, category="evidence", name="profile", suffix=".json"),
            "source_images": source_evidence,
        },
        "geometry": {
            "source_aabb_dimensions_m": source_dimensions,
            "mesh_objects": _non_negative_int(inspection.get("mesh_objects"), "inspection.mesh_objects"),
            "vertices": _non_negative_int(inspection.get("vertices"), "inspection.vertices"),
            "triangles": _non_negative_int(inspection.get("triangles"), "inspection.triangles"),
            "uv_layers": _non_negative_int(inspection.get("uv_layers"), "inspection.uv_layers"),
            "non_manifold_edges": non_manifold_edges,
            "normalization": {
                "policy": "uniform_fit_inside_mujoco_proxy",
                "source_axis_order": list(canonical_axis_order),
                "uniform_scale": uniform_scale,
                "normalized_dimensions_m": normalized_dimensions,
            },
        },
        "binding": {
            "world_entity_id": canonical_entity_id,
            "semantic_class": canonical_semantic_class,
            "physics": {
                "authority": "mujoco",
                "proxy": {"shape": "box", "size_m": canonical_proxy_size},
                "render_mesh_is_collider": False,
            },
            "unreal": {
                "asset_path": canonical_unreal_path,
                "collision_profile": "NoCollision",
                "collision_enabled": False,
                "simulate_physics": False,
                "generate_overlap_events": False,
                "can_ever_affect_navigation": False,
                "auto_spawn_in_production_map": False,
            },
        },
        "qualification": {
            "state": "QUARANTINED",
            "blockers": sorted(blockers),
            "promotion_target": "WorldPackage.visual facet",
        },
    }
    return validate_visual_candidate_manifest(
        {**body, "digest": digest_document(body)}
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compile a Tripo visual candidate manifest without contacting the API."
    )
    parser.add_argument("--asset-id", required=True)
    parser.add_argument("--world-entity-id", required=True)
    parser.add_argument("--semantic-class", required=True)
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--task", required=True, type=Path)
    parser.add_argument("--inspection", required=True, type=Path)
    parser.add_argument("--assessment", required=True, type=Path)
    parser.add_argument(
        "--source-axis-order",
        required=True,
        nargs=3,
        choices=tuple(_AXES),
        metavar=("ENTITY_X", "ENTITY_Y", "ENTITY_Z"),
    )
    parser.add_argument("--proxy-size-m", required=True, nargs=3, type=float)
    parser.add_argument("--unreal-asset-path", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def _ensure_link_free_directory(path: Path) -> Path:
    candidate = _absolute(path)
    current = Path(candidate.anchor)
    for component in candidate.parts[1:]:
        current /= component
        try:
            metadata = os.lstat(current)
        except FileNotFoundError:
            try:
                current.mkdir(mode=0o700)
            except FileExistsError:
                metadata = os.lstat(current)
            else:
                metadata = os.lstat(current)
        if stat.S_ISLNK(metadata.st_mode) or bool(
            getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
        ):
            raise ValueError("output parent contains a link or reparse point")
        if not stat.S_ISDIR(metadata.st_mode):
            raise ValueError("output parent must be a directory")
    return candidate


def _copy_verified(
    source: Path,
    record: Mapping[str, Any],
    *,
    limit: int,
    is_glb: bool = False,
) -> tuple[str, bytes]:
    observed, body = _snapshot_file(source, str(source), limit=limit, keep_bytes=True)
    assert body is not None
    if observed["bytes"] != record.get("bytes") or observed["sha256"] != record.get("sha256"):
        raise ValueError(f"source changed after manifest construction: {source}")
    if is_glb:
        _validate_glb_v2(body, str(source))
    relative = _portable_artifact_path(record.get("path"), "publication path")
    return relative, body


def _write_relative_fd(root_fd: int, relative: str, body: bytes) -> None:
    parts = Path(relative).parts
    directory_fd = os.dup(root_fd)
    try:
        for part in parts[:-1]:
            try:
                os.mkdir(part, mode=0o700, dir_fd=directory_fd)
            except FileExistsError:
                pass
            next_fd = os.open(
                part,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=directory_fd,
            )
            os.close(directory_fd)
            directory_fd = next_fd
        descriptor = os.open(
            parts[-1],
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_BINARY", 0),
            0o600,
            dir_fd=directory_fd,
        )
        try:
            view = memoryview(body)
            while view:
                written = os.write(descriptor, view)
                view = view[written:]
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        os.close(directory_fd)


def _write_relative_path(root: Path, relative: str, body: bytes) -> None:
    destination = root / Path(relative)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as handle:
        handle.write(body)
        handle.flush()
        os.fsync(handle.fileno())


@contextmanager
def _stable_publication_parent(path: Path) -> Iterator[tuple[Path, int | None]]:
    parent = _ensure_link_free_directory(path)
    expected = os.stat(parent, follow_symlinks=False)
    if os.name == "nt":
        create_file = ctypes.windll.kernel32.CreateFileW
        create_file.argtypes = [
            ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p,
        ]
        create_file.restype = ctypes.c_void_p
        handle = create_file(
            os.fspath(parent),
            0x80000000,
            0x00000001 | 0x00000002,
            None,
            3,
            0x00200000 | 0x02000000,
            None,
        )
        if handle in (None, ctypes.c_void_p(-1).value):
            raise ValueError("cannot lock publication parent")
        try:
            current = os.stat(parent, follow_symlinks=False)
            if (current.st_dev, current.st_ino) != (expected.st_dev, expected.st_ino):
                raise ValueError("publication parent changed while it was locked")
            yield parent, None
        finally:
            ctypes.windll.kernel32.CloseHandle(handle)
        return
    descriptor = os.open(
        parent,
        os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
    )
    identity = os.fstat(descriptor)
    if (identity.st_dev, identity.st_ino) != (expected.st_dev, expected.st_ino):
        os.close(descriptor)
        raise ValueError("publication parent changed before it was locked")
    try:
        yield parent, descriptor
        current = os.stat(parent, follow_symlinks=False)
        if (current.st_dev, current.st_ino) != (identity.st_dev, identity.st_ino):
            raise ValueError("publication parent changed while it was locked")
    finally:
        os.close(descriptor)


def _rename_directory_noreplace(
    parent: Path,
    parent_fd: int | None,
    source_name: str,
    target_name: str,
) -> None:
    if os.name == "nt":
        move_file = ctypes.windll.kernel32.MoveFileW
        move_file.argtypes = [ctypes.c_wchar_p, ctypes.c_wchar_p]
        move_file.restype = ctypes.c_int
        if not move_file(os.fspath(parent / source_name), os.fspath(parent / target_name)):
            raise FileExistsError(target_name)
        return
    assert parent_fd is not None
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, "renameat2", None)
    if renameat2 is None:
        raise ValueError("atomic directory no-replace publication is unsupported")
    renameat2.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    renameat2.restype = ctypes.c_int
    if renameat2(parent_fd, os.fsencode(source_name), parent_fd, os.fsencode(target_name), 1) != 0:
        error = ctypes.get_errno()
        if error in {17, 39}:  # EEXIST / ENOTEMPTY
            raise FileExistsError(target_name)
        raise OSError(error, os.strerror(error))


def _remove_staging(parent: Path, parent_fd: int | None, name: str) -> None:
    if os.name == "nt":
        root = parent / name
        if not root.exists():
            return
        for path in sorted(root.rglob("*"), reverse=True):
            if path.is_file():
                path.unlink()
            else:
                path.rmdir()
        root.rmdir()
        return
    assert parent_fd is not None
    try:
        stage_fd = os.open(
            name,
            os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=parent_fd,
        )
    except FileNotFoundError:
        return
    try:
        def remove_children(directory_fd: int) -> None:
            for entry in os.scandir(directory_fd):
                if entry.is_dir(follow_symlinks=False):
                    child_fd = os.open(
                        entry.name,
                        os.O_RDONLY
                        | getattr(os, "O_DIRECTORY", 0)
                        | getattr(os, "O_NOFOLLOW", 0),
                        dir_fd=directory_fd,
                    )
                    try:
                        remove_children(child_fd)
                    finally:
                        os.close(child_fd)
                    os.rmdir(entry.name, dir_fd=directory_fd)
                else:
                    os.unlink(entry.name, dir_fd=directory_fd)

        remove_children(stage_fd)
    finally:
        os.close(stage_fd)
    os.rmdir(name, dir_fd=parent_fd)


def _publish_package_no_replace(
    output: Path,
    manifest: Mapping[str, Any],
    *,
    model_path: Path,
    task_path: Path,
    inspection_path: Path,
    assessment_path: Path,
    profile_path: Path,
) -> None:
    target_manifest = _absolute(output)
    target_root = target_manifest.parent
    manifest_body = (
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    with _stable_publication_parent(target_root.parent) as (publication_parent, parent_fd):
        try:
            if parent_fd is None:
                target_metadata = os.lstat(target_root)
            else:
                target_metadata = os.stat(
                    target_root.name, dir_fd=parent_fd, follow_symlinks=False
                )
        except FileNotFoundError:
            target_metadata = None
        if target_metadata is not None:
            if stat.S_ISLNK(target_metadata.st_mode) or bool(
                getattr(target_metadata, "st_file_attributes", 0) & _REPARSE_POINT
            ):
                raise ValueError("output target contains a link or reparse point")
            try:
                existing = validate_visual_candidate_closure(target_manifest)
            except ValueError as exc:
                raise ValueError("output already exists with different content") from exc
            if existing != manifest:
                raise ValueError("output already exists with different content")
            return
        staging_name = f".t-{secrets.token_hex(8)}"
        if parent_fd is None:
            staging_dir = publication_parent / staging_name
            staging_dir.mkdir(mode=0o700)
            stage_fd = None
        else:
            os.mkdir(staging_name, mode=0o700, dir_fd=parent_fd)
            stage_fd = os.open(
                staging_name,
                os.O_RDONLY
                | getattr(os, "O_DIRECTORY", 0)
                | getattr(os, "O_NOFOLLOW", 0),
                dir_fd=parent_fd,
            )

        def write(relative: str, body: bytes) -> None:
            if stage_fd is None:
                _write_relative_path(staging_dir, relative, body)
            else:
                _write_relative_fd(stage_fd, relative, body)

        artifact = _mapping(_mapping(manifest["asset"], "asset")["artifact"], "asset.artifact")
        evidence = _mapping(manifest["evidence"], "evidence")
        try:
            relative, body = _copy_verified(
                model_path, artifact, limit=_MODEL_LIMIT, is_glb=True
            )
            write(relative, body)
            for source, name in (
                (task_path, "task"),
                (inspection_path, "inspection"),
                (assessment_path, "assessment"),
                (profile_path, "profile"),
            ):
                relative, body = _copy_verified(
                    source, _mapping(evidence[name], name), limit=_JSON_LIMIT
                )
                write(relative, body)
            task, _ = _load_mapping_snapshot(task_path, "task")
            task_input = _mapping(task.get("input"), "task.input")
            source_by_view = {
                _string(item.get("view"), "source view"): _absolute(
                    Path(task_path).parent
                    / Path(_portable_artifact_path(item.get("path"), "source path"))
                )
                for item in (
                    _mapping(raw, "source image") for raw in task_input["source_images"]
                )
            }
            for record in evidence["source_images"]:
                image = _mapping(record, "source image evidence")
                view = _string(image.get("view"), "source image view")
                relative, body = _copy_verified(
                    source_by_view[view], image, limit=_IMAGE_LIMIT
                )
                write(relative, body)
            write(target_manifest.name, manifest_body)
            _rename_directory_noreplace(
                publication_parent,
                parent_fd,
                staging_name,
                target_root.name,
            )
        except FileExistsError as exc:
            try:
                existing = validate_visual_candidate_closure(target_manifest)
            except ValueError as validation_exc:
                raise ValueError("output already exists with different content") from validation_exc
            if existing != manifest:
                raise ValueError("output already exists with different content") from exc
        finally:
            if stage_fd is not None:
                os.close(stage_fd)
            _remove_staging(publication_parent, parent_fd, staging_name)


def main(argv: list[str] | None = None) -> int:
    """Write one deterministic, credential-free visual candidate manifest."""

    args = _parser().parse_args(argv)
    manifest = build_visual_candidate_manifest(
        asset_id=args.asset_id,
        world_entity_id=args.world_entity_id,
        semantic_class=args.semantic_class,
        model_path=args.model,
        task_path=args.task,
        inspection_path=args.inspection,
        assessment_path=args.assessment,
        source_axis_order=args.source_axis_order,
        proxy_size_m=args.proxy_size_m,
        unreal_asset_path=args.unreal_asset_path,
    )
    _publish_package_no_replace(
        args.output,
        manifest,
        model_path=args.model,
        task_path=args.task,
        inspection_path=args.inspection,
        assessment_path=args.assessment,
        profile_path=PROFILE_PATH,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
