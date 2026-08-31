"""Build deterministic, credential-free Tripo high-fidelity request payloads."""

from __future__ import annotations

import argparse
import json
import os
import stat
from collections.abc import Mapping
from pathlib import Path
from typing import Any, cast

PROFILE_PATH = Path(__file__).with_name("tripo_hero_static_pbr.v1.json")
_VIEW_ORDER = ("front", "left", "back", "right")
_REPARSE_POINT = getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400)
_PROFILE_KEYS = {
    "schema", "id", "version", "purpose", "api", "input", "generation",
    "required_pbr_maps", "ingress", "unreal_candidate_policy", "promotion",
    "cost_and_secret_policy",
}


def _exact_keys(value: Mapping[str, Any], expected: set[str], context: str) -> None:
    observed = set(value)
    if observed != expected:
        raise ValueError(
            f"{context} keys must match the contract; "
            f"missing={sorted(expected - observed)}, unexpected={sorted(observed - expected)}"
        )


def _read_profile_bytes(path: Path) -> bytes:
    candidate = Path(os.path.abspath(os.fspath(path)))
    current = Path(candidate.anchor)
    for component in candidate.parts[1:]:
        current /= component
        try:
            metadata = os.lstat(current)
        except OSError as exc:
            raise ValueError(f"cannot inspect Tripo profile path: {exc}") from exc
        if stat.S_ISLNK(metadata.st_mode) or bool(
            getattr(metadata, "st_file_attributes", 0) & _REPARSE_POINT
        ):
            raise ValueError("Tripo profile path contains a link or reparse point")
    flags = os.O_RDONLY | getattr(os, "O_BINARY", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(candidate, flags)
    except OSError as exc:
        raise ValueError(f"cannot open Tripo profile: {exc}") from exc
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size > 1024 * 1024:
            raise ValueError("Tripo profile must be one bounded regular file")
        body = os.read(descriptor, before.st_size + 1)
        after = os.fstat(descriptor)
        if len(body) != before.st_size or (
            before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns
        ) != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError("Tripo profile changed while it was read")
        return body
    finally:
        os.close(descriptor)


def load_profile(path: Path = PROFILE_PATH) -> dict[str, Any]:
    """Load and validate the checked-in high-fidelity generation policy."""

    profile = cast(dict[str, Any], json.loads(_read_profile_bytes(path).decode("utf-8")))
    return validate_profile_document(profile)


def validate_profile_document(document: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one already snapshotted generation profile."""

    profile = dict(document)
    _exact_keys(profile, _PROFILE_KEYS, "profile")
    if profile.get("schema") != "lingtu.sim.tripo-generation-profile.v1":
        raise ValueError("unsupported Tripo generation profile schema")
    if profile.get("id") != "tripo.hero-static-pbr.high-fidelity" or profile.get("version") != "1.0.0":
        raise ValueError("Tripo profile identity must match the checked-in v1 contract")
    if profile.get("purpose") != (
        "Generate one game-ready static hero prop candidate for qualified "
        "WorldPackage visual facets."
    ):
        raise ValueError("Tripo profile purpose must match the checked-in trust contract")
    input_policy = cast(Mapping[str, Any], profile.get("input", {}))
    _exact_keys(
        input_policy,
        {"mode", "minimum_views", "required_views", "preferred_view_order", "same_object_and_lighting"},
        "profile.input",
    )
    if input_policy.get("mode") != "multiview":
        raise ValueError("high-fidelity Tripo profile must use multiview input")
    if (
        input_policy.get("minimum_views") != 2
        or input_policy.get("required_views") != ["front"]
        or input_policy.get("preferred_view_order") != list(_VIEW_ORDER)
        or input_policy.get("same_object_and_lighting") is not True
    ):
        raise ValueError("profile.input must match the multiview evidence contract")

    api = cast(Mapping[str, Any], profile.get("api", {}))
    _exact_keys(
        api,
        {"base_url", "create_endpoint", "task_endpoint_template", "model_url_ttl_seconds", "download_immediately"},
        "profile.api",
    )
    if (
        api.get("base_url") != "https://openapi.tripo3d.com/v3"
        or api.get("create_endpoint") != "/generation/multiview-to-model"
        or api.get("task_endpoint_template") != "/tasks/{task_id}"
    ):
        raise ValueError("profile.api must use the allowlisted official Tripo API routes")
    if api.get("model_url_ttl_seconds") != 300 or api.get("download_immediately") is not True:
        raise ValueError("profile.api must require immediate bounded-lifetime artifact download")

    generation = profile.get("generation", {})
    _exact_keys(
        cast(Mapping[str, Any], generation),
        {
            "model", "geometry_quality", "texture", "pbr", "texture_quality",
            "auto_size", "export_uv", "face_limit", "smart_low_poly",
            "require_model_seed", "require_texture_seed",
        },
        "profile.generation",
    )
    required_generation = {
        "model": "v3.1-20260211",
        "geometry_quality": "detailed",
        "texture": True,
        "pbr": True,
        "texture_quality": "extreme",
        "auto_size": True,
        "export_uv": True,
        "smart_low_poly": False,
        "require_model_seed": True,
        "require_texture_seed": True,
    }
    for key, expected in required_generation.items():
        if generation.get(key) != expected:
            raise ValueError(f"Tripo profile generation.{key} must be {expected!r}")

    face_limit = generation.get("face_limit")
    if face_limit != 100_000 or isinstance(face_limit, bool):
        raise ValueError("Tripo v1 hero asset face_limit must be exactly 100000")

    if profile.get("required_pbr_maps") != ["base_color", "normal", "roughness", "metallic"]:
        raise ValueError("Tripo profile must require the complete metallic-roughness PBR map set")
    if profile.get("unreal_candidate_policy", {}).get("collision") != "NoCollision":
        raise ValueError("Tripo visual candidates must not become a second collision authority")
    if profile.get("unreal_candidate_policy", {}).get("auto_spawn_in_production_map") is not False:
        raise ValueError("Tripo candidates must not auto-spawn into production maps")
    unreal_policy = cast(Mapping[str, Any], profile.get("unreal_candidate_policy", {}))
    _exact_keys(
        unreal_policy,
        {
            "canonical_candidate_root", "auto_spawn_in_production_map", "collision",
            "nanite_for_static_mesh", "skeletal_mesh_allowed",
        },
        "profile.unreal_candidate_policy",
    )
    if (
        unreal_policy.get("canonical_candidate_root") != "/Game/RobotSim/Staging/Tripo"
        or unreal_policy.get("nanite_for_static_mesh") is not True
        or unreal_policy.get("skeletal_mesh_allowed") is not False
    ):
        raise ValueError("profile.unreal_candidate_policy must match the static visual-only contract")
    secret_policy = cast(Mapping[str, Any], profile.get("cost_and_secret_policy", {}))
    _exact_keys(
        secret_policy,
        {"execute_requires_explicit_credit_confirmation", "persist_api_key", "log_authorization_header"},
        "profile.cost_and_secret_policy",
    )
    if secret_policy.get("persist_api_key") is not False:
        raise ValueError("Tripo API credentials must never be persisted in generation profiles")
    if (
        secret_policy.get("execute_requires_explicit_credit_confirmation") is not True
        or secret_policy.get("log_authorization_header") is not False
    ):
        raise ValueError("profile cost and secret policy must remain fail-closed")
    ingress = cast(Mapping[str, Any], profile.get("ingress", {}))
    _exact_keys(ingress, {"official_api", "ue_bridge"}, "profile.ingress")
    official_api = cast(Mapping[str, Any], ingress.get("official_api", {}))
    _exact_keys(official_api, {"artifact_format", "route"}, "profile.ingress.official_api")
    if official_api.get("artifact_format") != "glb":
        raise ValueError("profile ingress must require a self-contained GLB")
    if official_api.get("route") != (
        "immediate download -> SimStudio SourceInbox -> import draft"
    ):
        raise ValueError("profile official API route must match the allowlisted ingress route")
    ue_bridge = cast(Mapping[str, Any], ingress.get("ue_bridge", {}))
    _exact_keys(ue_bridge, {"plugin", "endpoint", "import_root", "role"}, "profile.ingress.ue_bridge")
    if (
        ue_bridge.get("plugin") != "Tripo3DUEBridge@1.0.4"
        or ue_bridge.get("endpoint") != "ws://127.0.0.1:60620"
        or ue_bridge.get("import_root") != "/Game/TripoModels"
        or ue_bridge.get("role") != "editor_preview_only"
    ):
        raise ValueError("profile UE bridge must use the allowlisted local preview endpoint")
    promotion = cast(Mapping[str, Any], profile.get("promotion", {}))
    _exact_keys(
        promotion,
        {
            "target", "required_candidate_blockers", "required_evidence",
            "requires_stable_entity_id", "requires_explicit_mujoco_collision_decision",
        },
        "profile.promotion",
    )
    required_evidence = promotion.get("required_evidence")
    required_contract = [
        "source_image_sha256", "api_request_without_credentials", "task_id", "model_version",
        "model_seed", "texture_seed", "downloaded_artifact_sha256", "license_and_usage_rights",
        "metre_scale_aabb", "material_slot_inventory", "pbr_map_inventory", "triangle_count",
        "uv_validation", "unreal_no_collision_validation",
    ]
    if required_evidence != required_contract:
        raise ValueError("profile promotion.required_evidence must match the promotion evidence contract")
    if (
        promotion.get("target") != "WorldPackage.visual facet"
        or promotion.get("required_candidate_blockers")
        != [
            "license_and_usage_rights_unverified",
            "unreal_import_not_verified",
        ]
        or promotion.get("requires_stable_entity_id") is not True
        or promotion.get("requires_explicit_mujoco_collision_decision") is not True
    ):
        raise ValueError("profile promotion policy must preserve world identity and MuJoCo authority")
    return profile


def _required_seed(name: str, value: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def build_multiview_payload(
    views: Mapping[str, str],
    *,
    model_seed: int,
    texture_seed: int,
    profile_path: Path = PROFILE_PATH,
) -> dict[str, Any]:
    """Return the exact POST body without performing a network request."""

    profile = load_profile(profile_path)
    unknown = sorted(set(views) - set(_VIEW_ORDER))
    if unknown:
        raise ValueError(f"unsupported Tripo view keys: {', '.join(unknown)}")

    normalized: dict[str, str] = {}
    for key in _VIEW_ORDER:
        raw = views.get(key)
        if raw is None:
            continue
        value = raw.strip()
        if not value:
            raise ValueError(f"Tripo view {key} must not be empty")
        normalized[key] = value

    if "front" not in normalized:
        raise ValueError("Tripo multiview input requires a front view")
    if len(normalized) < profile["input"]["minimum_views"]:
        raise ValueError("Tripo high-fidelity generation requires at least two views")
    if len(set(normalized.values())) != len(normalized):
        raise ValueError("Tripo multiview inputs must identify distinct images")

    generation = profile["generation"]
    payload = {
        "inputs": [{key: normalized[key]} for key in _VIEW_ORDER if key in normalized],
        "model": generation["model"],
        "geometry_quality": generation["geometry_quality"],
        "texture": generation["texture"],
        "pbr": generation["pbr"],
        "texture_quality": generation["texture_quality"],
        "auto_size": generation["auto_size"],
        "export_uv": generation["export_uv"],
        "face_limit": generation["face_limit"],
        "smart_low_poly": generation["smart_low_poly"],
        "model_seed": _required_seed("model_seed", model_seed),
        "texture_seed": _required_seed("texture_seed", texture_seed),
    }
    return payload


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Write a deterministic Tripo multiview request body; this command never calls the API."
    )
    for view in _VIEW_ORDER:
        parser.add_argument(f"--{view}")
    parser.add_argument("--model-seed", required=True, type=int)
    parser.add_argument("--texture-seed", required=True, type=int)
    parser.add_argument("--output", type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Write one validated request body without contacting Tripo."""

    args = _parser().parse_args(argv)
    views = {key: value for key in _VIEW_ORDER if (value := getattr(args, key)) is not None}
    payload = build_multiview_payload(
        views,
        model_seed=args.model_seed,
        texture_seed=args.texture_seed,
    )
    body = json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output is None:
        print(body, end="")
    else:
        args.output.write_text(body, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
