"""Build a UE staging recipe from compiled visuals and MuJoCo body truth."""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence, cast

from sim.catalog.visual_binding import compile_robot_visual_manifest


class UnrealPreviewError(ValueError):
    """Raised when a preview recipe cannot be built without guessing."""


def _robot_asset_root(robot_package: Path) -> Path:
    """Return the repository asset root for a catalog RobotPackage directory."""

    robot_package = Path(robot_package).resolve()
    if (
        len(robot_package.parents) >= 4
        and robot_package.parents[1].name == "packages"
        and robot_package.parents[2].name == "sim"
    ):
        return robot_package.parents[3]
    return robot_package


def _vector(value: Any, size: int, context: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != size:
        raise UnrealPreviewError(f"{context} must contain exactly {size} values")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise UnrealPreviewError(f"{context} must contain finite values")
    return result


def _normalized_quaternion(value: Any, context: str) -> tuple[float, float, float, float]:
    w, x, y, z = _vector(value, 4, context)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-12:
        raise UnrealPreviewError(f"{context} must not be a zero quaternion")
    return w / norm, x / norm, y / norm, z / norm


def _multiply_quaternions(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _rotate_vector(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _unreal_position_cm(position_m: tuple[float, float, float]) -> list[float]:
    return [100.0 * position_m[0], -100.0 * position_m[1], 100.0 * position_m[2]]


def _unreal_quaternion_xyzw(quaternion_wxyz: tuple[float, float, float, float]) -> list[float]:
    w, x, y, z = quaternion_wxyz
    return [-x, y, -z, w]


def build_unreal_preview_recipe(
    visual_manifest: Mapping[str, Any],
    asset_index: Mapping[str, Any],
    snapshot: Mapping[str, Any],
    *,
    instance_id: str,
    destination_path: str,
) -> dict[str, Any]:
    """Build stable per-visual UE component transforms for one robot instance."""

    if not instance_id.strip():
        raise UnrealPreviewError("instance_id must be non-empty")
    destination_path = destination_path.rstrip("/")
    if not destination_path.startswith("/Game/"):
        raise UnrealPreviewError("destination_path must be a /Game asset path")

    assets = {str(item["asset_name"]): item for item in asset_index.get("assets", ())}
    body_items = list(snapshot.get("bodies", ()))
    stable_bodies = {
        str(item["stable_id"]): item for item in body_items if item.get("stable_id")
    }
    if len(stable_bodies) != sum(bool(item.get("stable_id")) for item in body_items):
        raise UnrealPreviewError("truth snapshot contains duplicate stable body IDs")
    legacy_bodies = {str(item["name"]): item for item in body_items}
    components: list[dict[str, Any]] = []
    body_bindings: dict[str, dict[str, Any]] = {}
    for visual in visual_manifest.get("visuals", ()):
        body_name = str(visual["body"])
        body = (
            stable_bodies.get(f"{instance_id}/{body_name}")
            if stable_bodies
            else legacy_bodies.get(body_name)
        )
        if body is None:
            raise UnrealPreviewError(
                f"truth snapshot is missing body {instance_id}/{body_name}"
            )
        asset_name = str(visual["mesh"])
        asset = assets.get(asset_name)
        if asset is None:
            raise UnrealPreviewError(f"FBX asset index is missing {asset_name!r}")
        body_position = cast(
            tuple[float, float, float],
            _vector(body["position_m"], 3, f"body {body_name}.position_m"),
        )
        body_quaternion = _normalized_quaternion(
            body["quaternion_wxyz"], f"body {body_name}.quaternion_wxyz"
        )
        local_position = cast(
            tuple[float, float, float],
            _vector(visual["pos"], 3, f"visual {asset_name}.pos"),
        )
        local_quaternion = _normalized_quaternion(
            visual["quat"], f"visual {asset_name}.quat"
        )
        local_scale = list(
            _vector(visual["scale"], 3, f"visual {asset_name}.scale")
        )
        link_to_mesh = {
            "location_cm": _unreal_position_cm(local_position),
            "quaternion_xyzw": _unreal_quaternion_xyzw(local_quaternion),
            "scale": list(local_scale),
        }
        rotated_offset = _rotate_vector(body_quaternion, local_position)
        world_position = cast(
            tuple[float, float, float],
            tuple(body_position[axis] + rotated_offset[axis] for axis in range(3)),
        )
        world_quaternion = _normalized_quaternion(
            _multiply_quaternions(body_quaternion, local_quaternion),
            f"visual {asset_name}.world_quaternion",
        )
        visual_frame_id = str(visual["visual_frame_id"])
        body_frame_id = str(visual["body_frame_id"])
        body_stable_id = f"{instance_id}/{body_frame_id}"
        body_binding = {
            "stable_id": body_stable_id,
            "location_cm": _unreal_position_cm(body_position),
            "quaternion_xyzw": _unreal_quaternion_xyzw(body_quaternion),
        }
        existing_binding = body_bindings.get(body_stable_id)
        if existing_binding is not None and existing_binding != body_binding:
            raise UnrealPreviewError(
                f"visuals disagree about body transform {body_stable_id}"
            )
        body_bindings[body_stable_id] = body_binding
        components.append(
            {
                "stable_id": f"{instance_id}/{visual_frame_id}",
                "body_frame_id": body_stable_id,
                "asset_key": str(visual["asset_key"]),
                "asset_name": asset_name,
                "unreal_asset": f"{destination_path}/{asset_name}.{asset_name}",
                "material_key": visual.get("material"),
                "link_to_mesh": link_to_mesh,
                "location_cm": _unreal_position_cm(world_position),
                "quaternion_xyzw": _unreal_quaternion_xyzw(world_quaternion),
                "scale": list(local_scale),
            }
        )

    return {
        "schema": "lingtu.sim.unreal-preview-recipe.v1",
        "session_id": snapshot.get("session_id", ""),
        "instance_id": instance_id,
        "binding": visual_manifest.get("binding"),
        "model_generation": int(snapshot.get("model_generation", 0)),
        "reset_generation": int(snapshot.get("reset_generation", 0)),
        "bodies": sorted(body_bindings.values(), key=lambda item: item["stable_id"]),
        "components": components,
    }


def stage_unreal_preview_recipe(
    *,
    robot_package: Path,
    asset_index_path: Path,
    snapshot_executable: Path | None,
    snapshot_path: Path | None,
    keyframe: str,
    instance_id: str,
    destination_path: str,
    output_path: Path,
) -> Path:
    """Run the native pose compiler and write one deterministic UE staging recipe."""

    robot_package = Path(robot_package).resolve()
    manifest = compile_robot_visual_manifest(robot_package).to_dict()
    mjcf_path = _robot_asset_root(robot_package) / str(manifest["mjcf"])
    try:
        asset_index = json.loads(Path(asset_index_path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise UnrealPreviewError(f"cannot read FBX asset index {asset_index_path}: {exc}") from exc

    if (snapshot_executable is None) == (snapshot_path is None):
        raise UnrealPreviewError(
            "provide exactly one of snapshot_executable or snapshot_path"
        )
    if snapshot_path is not None:
        if keyframe:
            raise UnrealPreviewError("keyframe cannot be used with snapshot_path")
        try:
            snapshot = json.loads(Path(snapshot_path).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise UnrealPreviewError(
                f"cannot read truth snapshot {snapshot_path}: {exc}"
            ) from exc
        if snapshot.get("schema") != "lingtu.sim.truth-snapshot.v1":
            raise UnrealPreviewError(f"unsupported truth snapshot: {snapshot_path}")
    else:
        if snapshot_executable is None:
            raise UnrealPreviewError("snapshot_executable is required")
        command = [str(Path(snapshot_executable).resolve()), str(mjcf_path)]
        if keyframe:
            command.append(keyframe)
        try:
            result = subprocess.run(  # noqa: S603 - explicit executable and argv
                command, check=False, capture_output=True, text=True
            )
        except OSError as exc:
            raise UnrealPreviewError(
                f"cannot run native MuJoCo snapshot exporter: {exc}"
            ) from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or f"exit code {result.returncode}"
            raise UnrealPreviewError(
                f"native MuJoCo snapshot exporter failed: {detail}"
            )
        try:
            snapshot = json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise UnrealPreviewError(
                f"native MuJoCo snapshot exporter returned invalid JSON: {exc}"
            ) from exc

    recipe = build_unreal_preview_recipe(
        manifest,
        asset_index,
        snapshot,
        instance_id=instance_id,
        destination_path=destination_path,
    )
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(recipe, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return output_path


def _parse_arguments(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Stage a MuJoCo robot pose for RobotSimUE")
    parser.add_argument("--robot-package", required=True, type=Path)
    parser.add_argument("--asset-index", required=True, type=Path)
    snapshot_source = parser.add_mutually_exclusive_group(required=True)
    snapshot_source.add_argument("--snapshot-exe", type=Path)
    snapshot_source.add_argument("--snapshot", type=Path)
    parser.add_argument("--keyframe", default="")
    parser.add_argument("--instance-id", required=True)
    parser.add_argument("--destination-path", required=True)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(list(argv))


def main(argv: Sequence[str] | None = None) -> int:
    """Build one deterministic RobotSimUE preview recipe."""

    args = _parse_arguments(sys.argv[1:] if argv is None else argv)
    output = stage_unreal_preview_recipe(
        robot_package=args.robot_package,
        asset_index_path=args.asset_index,
        snapshot_executable=args.snapshot_exe,
        snapshot_path=args.snapshot,
        keyframe=args.keyframe,
        instance_id=args.instance_id,
        destination_path=args.destination_path,
        output_path=args.output,
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
