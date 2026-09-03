"""Build the canonical instance-agnostic Robot Visual Projection for Unreal."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence, cast

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from sim.catalog import (
    compile_robot_visual_manifest,
    compile_robot_visual_projection,
    validate_robot_visual_projection,
)


class RobotVisualProjectionToolError(ValueError):
    """Raised when asset projection inputs cannot be resolved without guessing."""


_ASSET_INDEX_SCHEMA = "lingtu.sim.fbx-asset-index.v1"
_DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")


def _load_json(path: Path, context: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RobotVisualProjectionToolError(f"cannot read {context} {path}: {exc}") from exc
    if not isinstance(value, Mapping):
        raise RobotVisualProjectionToolError(f"{context} must contain a JSON object")
    return dict(value)


def _safe_destination_path(value: str) -> str:
    if not isinstance(value, str) or not value:
        raise RobotVisualProjectionToolError("destination_path must be a non-empty /Game path")
    if (
        value != value.strip()
        or any(char.isspace() for char in value)
        or "\\" in value
        or not value.startswith("/Game/")
        or value.endswith("/")
        or "//" in value
    ):
        raise RobotVisualProjectionToolError(
            "destination_path must be a safe /Game/ Unreal asset path"
        )
    parts = value.split("/")
    if any(part in ("", ".", "..") for part in parts[2:]):
        raise RobotVisualProjectionToolError(
            "destination_path must be a safe /Game/ Unreal asset path"
        )
    return value


def _safe_asset_name(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise RobotVisualProjectionToolError(f"{context} must be a non-empty asset name")
    if any(char.isspace() for char in value) or "/" in value or "\\" in value:
        raise RobotVisualProjectionToolError(f"{context} must be a safe Unreal asset name")
    if value in (".", ".."):
        raise RobotVisualProjectionToolError(f"{context} must be a safe Unreal asset name")
    return value


def _safe_fbx_name(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise RobotVisualProjectionToolError(f"{context} must be a non-empty FBX filename")
    path = PurePosixPath(value)
    if (
        "\\" in value
        or any(char.isspace() for char in value)
        or path.is_absolute()
        or any(part in ("", ".", "..") for part in value.split("/"))
        or path.suffix.lower() != ".fbx"
    ):
        raise RobotVisualProjectionToolError(f"{context} must be a safe relative .fbx filename")
    return value


def _digest(value: Any, context: str) -> str:
    if not isinstance(value, str) or _DIGEST_RE.fullmatch(value) is None:
        raise RobotVisualProjectionToolError(
            f"{context} must be 64 lowercase hexadecimal characters"
        )
    return value


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mesh_expectations(
    visual_manifest: Mapping[str, Any],
    robot_package: Path,
) -> dict[str, str]:
    visuals = visual_manifest.get("visuals")
    if not isinstance(visuals, Sequence) or isinstance(visuals, (str, bytes)):
        raise RobotVisualProjectionToolError("compiled visual manifest visuals must be an array")
    expected: dict[str, str] = {}
    for index, visual in enumerate(visuals):
        if not isinstance(visual, Mapping):
            raise RobotVisualProjectionToolError(f"visual manifest visual {index} must be an object")
        geometry = visual.get("geometry")
        if not isinstance(geometry, Mapping):
            raise RobotVisualProjectionToolError(
                f"visual manifest visual {index}.geometry must be an object"
            )
        if geometry.get("kind") != "mesh":
            continue
        mesh = _safe_asset_name(geometry.get("mesh"), f"visual manifest visual {index}.mesh")
        source_mesh = geometry.get("source_mesh")
        if not isinstance(source_mesh, str) or not source_mesh:
            raise RobotVisualProjectionToolError(
                f"visual manifest visual {index}.source_mesh must be a package-relative path"
            )
        source_path = (robot_package / PurePosixPath(source_mesh)).resolve()
        try:
            source_path.relative_to(robot_package)
        except ValueError as exc:
            raise RobotVisualProjectionToolError(
                f"visual manifest visual {index}.source_mesh escapes the robot package"
            ) from exc
        if not source_path.is_file():
            raise RobotVisualProjectionToolError(
                f"visual manifest visual {index}.source_mesh does not exist"
            )
        source_sha256 = _sha256_file(source_path)
        previous = expected.get(mesh)
        if previous is not None and previous != source_sha256:
            raise RobotVisualProjectionToolError(
                f"mesh {mesh!r} has conflicting source hashes in the visual manifest"
            )
        expected[mesh] = source_sha256
    return expected


def build_asset_bindings(
    visual_manifest: Mapping[str, Any],
    asset_index: Mapping[str, Any],
    destination_path: str,
    asset_index_path: Path,
    robot_package: Path,
) -> dict[str, str]:
    """Build mesh-name to cooked Unreal asset bindings from one FBX index."""

    expected = _mesh_expectations(visual_manifest, robot_package)
    if asset_index.get("schema") != _ASSET_INDEX_SCHEMA:
        raise RobotVisualProjectionToolError(
            f"asset index schema must be {_ASSET_INDEX_SCHEMA!r}"
        )
    assets = asset_index.get("assets")
    if not isinstance(assets, Sequence) or isinstance(assets, (str, bytes)):
        raise RobotVisualProjectionToolError("asset index assets must be an array")
    destination = _safe_destination_path(destination_path)
    seen_names: set[str] = set()
    for index, raw_asset in enumerate(assets):
        if not isinstance(raw_asset, Mapping):
            raise RobotVisualProjectionToolError(f"asset index entry {index} must be an object")
        asset_name = _safe_asset_name(raw_asset.get("asset_name"), f"asset index entry {index}.asset_name")
        if asset_name in seen_names:
            raise RobotVisualProjectionToolError(f"asset index contains duplicate asset name {asset_name!r}")
        seen_names.add(asset_name)
    indexed: dict[str, Mapping[str, Any]] = {}
    for index, raw_asset in enumerate(assets):
        if not isinstance(raw_asset, Mapping):
            raise RobotVisualProjectionToolError(f"asset index entry {index} must be an object")
        asset_name = _safe_asset_name(raw_asset.get("asset_name"), f"asset index entry {index}.asset_name")
        if asset_name in indexed:
            raise RobotVisualProjectionToolError(f"asset index contains duplicate asset name {asset_name!r}")
        if asset_name not in expected:
            raise RobotVisualProjectionToolError(
                f"asset index contains unexpected asset {asset_name!r}"
            )
        fbx_name = _safe_fbx_name(raw_asset.get("fbx"), f"asset index entry {index}.fbx")
        source_sha256 = _digest(
            raw_asset.get("source_sha256"),
            f"asset index entry {index}.source_sha256",
        )
        if source_sha256 != expected[asset_name]:
            raise RobotVisualProjectionToolError(
                f"asset {asset_name!r} source SHA-256 does not match the visual manifest"
            )
        asset_index_parent = Path(asset_index_path).resolve().parent
        fbx_path = (asset_index_parent / fbx_name).resolve()
        try:
            fbx_path.relative_to(asset_index_parent)
        except ValueError as exc:
            raise RobotVisualProjectionToolError(
                f"asset index entry {index}.fbx must resolve under the asset index directory"
            ) from exc
        if not fbx_path.is_file():
            raise RobotVisualProjectionToolError(
                f"asset index entry {index}.fbx must resolve to an existing regular file"
            )
        indexed[asset_name] = raw_asset

    missing = sorted(set(expected) - set(indexed))
    if missing:
        raise RobotVisualProjectionToolError(
            f"asset index is missing mesh entries: {', '.join(missing)}"
        )
    return {
        asset_name: f"{destination}/{asset_name}.{asset_name}"
        for asset_name in sorted(expected)
    }


def build_robot_visual_projection(
    robot_package: Path,
    asset_index_path: Path | None = None,
    destination_path: str | None = None,
) -> dict[str, Any]:
    """Compile one robot package into a canonical instance-agnostic projection."""

    try:
        manifest = compile_robot_visual_manifest(Path(robot_package)).to_dict()
    except ValueError as exc:
        raise RobotVisualProjectionToolError(str(exc)) from exc
    robot_package = Path(robot_package).resolve()
    expected_meshes = _mesh_expectations(manifest, robot_package)
    if bool(asset_index_path) != bool(destination_path):
        raise RobotVisualProjectionToolError(
            "asset_index_path and destination_path must be provided together"
        )
    if expected_meshes:
        if asset_index_path is None or destination_path is None:
            raise RobotVisualProjectionToolError(
                "mesh visual packages require an FBX asset index and /Game destination path"
            )
        asset_index = _load_json(Path(asset_index_path), "FBX asset index")
        bindings = build_asset_bindings(
            manifest,
            asset_index,
            destination_path,
            Path(asset_index_path),
            robot_package,
        )
    else:
        if asset_index_path is not None or destination_path is not None:
            raise RobotVisualProjectionToolError(
                "primitive-only packages do not accept FBX asset bindings"
            )
        bindings = None
    try:
        return cast(
            dict[str, Any],
            compile_robot_visual_projection(manifest, bindings).to_dict(),
        )
    except ValueError as exc:
        raise RobotVisualProjectionToolError(str(exc)) from exc


def write_robot_visual_projection(
    projection: Mapping[str, Any],
    output_path: Path,
) -> Path:
    """Write a pretty, sorted, deterministic projection document ending in LF."""

    try:
        normalized_projection = validate_robot_visual_projection(projection).to_dict()
    except ValueError as exc:
        raise RobotVisualProjectionToolError(str(exc)) from exc
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(
            json.dumps(
                normalized_projection,
                ensure_ascii=False,
                sort_keys=True,
                indent=2,
                allow_nan=False,
            )
            + "\n"
        )
    return output_path


def build_and_write_robot_visual_projection(
    *,
    robot_package: Path,
    asset_index_path: Path | None = None,
    destination_path: str | None = None,
    output_path: Path,
) -> Path:
    """Compile and write one canonical Robot Visual Projection document."""

    projection = build_robot_visual_projection(
        robot_package=robot_package,
        asset_index_path=asset_index_path,
        destination_path=destination_path,
    )
    return write_robot_visual_projection(projection, output_path)


def _parse_arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Build a Robot Visual Projection for RobotSimUE")
    parser.add_argument("--robot-package", required=True, type=Path)
    parser.add_argument("--asset-index", type=Path)
    parser.add_argument("--destination-path")
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(list(argv))


def main(argv: Sequence[str] | None = None) -> int:
    """Run the Robot Visual Projection command-line interface."""

    args = _parse_arguments(sys.argv[1:] if argv is None else argv)
    output = build_and_write_robot_visual_projection(
        robot_package=args.robot_package,
        asset_index_path=args.asset_index,
        destination_path=args.destination_path,
        output_path=args.output,
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
