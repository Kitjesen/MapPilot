"""Strict readback proof validators for scenario qualification evidence."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any, TypeGuard


def has_strict_physics_observability(
    evidence: Mapping[str, Any],
    runtime: Mapping[str, Any],
    *,
    expected_entities: Mapping[str, str],
) -> bool:
    """Return true only for concrete same-stamp MuJoCo raycast/contact readback."""

    sequence = evidence.get("sequence")
    sim_time_ns = evidence.get("sim_time_ns")
    if not _non_negative_int(sequence) or not _non_negative_int(sim_time_ns):
        return False
    raycasts = evidence.get("raycast_observations")
    if isinstance(raycasts, list) and any(
        _raycast_observation_proves_proxy(
            item,
            runtime,
            expected_entities=expected_entities,
            sequence=sequence,
            sim_time_ns=sim_time_ns,
        )
        for item in raycasts
    ):
        return True
    contacts = evidence.get("contact_observations")
    if isinstance(contacts, list) and any(
        _contact_observation_proves_proxy(
            item,
            runtime,
            expected_entities=expected_entities,
            sequence=sequence,
            sim_time_ns=sim_time_ns,
        )
        for item in contacts
    ):
        return True
    return False


def _raycast_observation_proves_proxy(
    value: object,
    runtime: Mapping[str, Any],
    *,
    expected_entities: Mapping[str, str],
    sequence: object,
    sim_time_ns: object,
) -> bool:
    if type(value) is not dict or not _same_readback_stamp(
        value,
        runtime,
        sequence=sequence,
        sim_time_ns=sim_time_ns,
    ):
        return False
    query = value.get("query")
    result = value.get("result")
    if type(query) is not dict or type(result) is not dict:
        return False
    if not _finite_vector3(_first_present(query, "origin_m", "origin_world_m")):
        return False
    if not _finite_vector3(_first_present(query, "direction_m", "direction_world")):
        return False
    range_max_m = _first_present(query, "range_m", "range_max_m")
    if not _positive_finite_number(range_max_m):
        return False
    range_min_m = query.get("range_min_m", 0.0)
    if not _non_negative_finite_number(range_min_m):
        return False
    range_min = float(range_min_m)
    range_max = float(range_max_m)
    if range_min >= range_max:
        return False
    if result.get("hit") is not True:
        return False
    entity_id = result.get("entity_id")
    if not isinstance(entity_id, str) or entity_id not in expected_entities:
        return False
    if result.get("body_stable_id") != expected_entities[entity_id]:
        return False
    distance_m = result.get("distance_m")
    if not _non_negative_finite_number(distance_m):
        return False
    distance = float(distance_m)
    if distance < range_min or distance > range_max:
        return False
    return _finite_vector3(_first_present(result, "position_m", "position_world_m"))


def _contact_observation_proves_proxy(
    value: object,
    runtime: Mapping[str, Any],
    *,
    expected_entities: Mapping[str, str],
    sequence: object,
    sim_time_ns: object,
) -> bool:
    if type(value) is not dict or not _same_readback_stamp(
        value,
        runtime,
        sequence=sequence,
        sim_time_ns=sim_time_ns,
    ):
        return False
    count = value.get("contact_count")
    if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
        return False
    if not _contact_side_matches_expected(value, expected_entities):
        return False
    return (
        _finite_vector3(value.get("contact_position_m"))
        or _non_negative_finite_number(value.get("penetration_m"))
        or _non_negative_finite_number(value.get("impulse_n_s"))
    )


def _same_readback_stamp(
    evidence: Mapping[str, Any],
    runtime: Mapping[str, Any],
    *,
    sequence: object,
    sim_time_ns: object,
) -> bool:
    return (
        evidence.get("source") == "mujoco_readback"
        and evidence.get("applied") is True
        and evidence.get("session_id") == runtime.get("session_id")
        and evidence.get("model_generation") == runtime.get("model_generation")
        and evidence.get("reset_generation") == runtime.get("reset_generation")
        and evidence.get("sequence") == sequence
        and evidence.get("sim_time_ns") == sim_time_ns
    )


def _contact_side_matches_expected(
    evidence: Mapping[str, Any],
    expected_entities: Mapping[str, str],
) -> bool:
    sides = (
        (evidence.get("entity_id"), evidence.get("body_stable_id")),
        (evidence.get("other_entity_id"), evidence.get("other_body_stable_id")),
    )
    for entity_id, body_stable_id in sides:
        if isinstance(entity_id, str) and expected_entities.get(entity_id) == body_stable_id:
            return True
    return False


def _finite_vector3(value: object) -> bool:
    return (
        isinstance(value, list)
        and len(value) == 3
        and all(_finite_number(item) for item in value)
    )


def _first_present(document: Mapping[str, Any], *keys: str) -> object:
    for key in keys:
        if key in document:
            return document[key]
    return None


def _positive_finite_number(value: object) -> TypeGuard[int | float]:
    return _finite_number(value) and float(value) > 0.0


def _non_negative_finite_number(value: object) -> TypeGuard[int | float]:
    return _finite_number(value) and float(value) >= 0.0


def _finite_number(value: object) -> TypeGuard[int | float]:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    )


def _non_negative_int(value: object) -> bool:
    return not isinstance(value, bool) and isinstance(value, int) and value >= 0


__all__ = ["has_strict_physics_observability"]
