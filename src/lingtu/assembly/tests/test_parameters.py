from __future__ import annotations

import pytest

from lingtu.assembly.parameters import resolve_parameters

PRODUCT_PARAMETERS = {
    "segment.max_distance_m": 4.0,
    "risk.stop_threshold": 60.0,
    "risk.resume_threshold": 45.0,
}


def test_parameter_precedence_is_session_then_product_then_environment() -> None:
    resolved = resolve_parameters(
        product_parameters=PRODUCT_PARAMETERS,
        env_overrides={
            "segment.max_distance_m": 3.0,
            "map_input.max_age_s": 0.5,
        },
        session_overrides={"segment.max_distance_m": 2.0},
        map_publish_hz=10.0,
    )

    assert resolved.values["segment.max_distance_m"].value == 2.0
    assert resolved.values["risk.stop_threshold"].value == 60.0
    assert resolved.values["map_input.max_age_s"].value == 0.5
    assert resolved.values["map_input.max_cells"].value == 262_144


def test_product_parameters_have_higher_priority_than_env() -> None:
    resolved = resolve_parameters(
        product_parameters=PRODUCT_PARAMETERS,
        env_overrides={"risk.stop_threshold": 55.0},
    )
    assert resolved.values["risk.stop_threshold"].value == 60.0


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"segment.max_waypoints": 1}, "outside"),
        ({"map_input.max_cells": 1}, "outside"),
        ({"risk.unknown": 1}, "unknown runtime parameters"),
        (
            {"risk.stop_threshold": 40.0, "risk.resume_threshold": 50.0},
            "resume_threshold",
        ),
    ],
)
def test_invalid_parameters_fail_before_launch(
    overrides: dict[str, float], message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        resolve_parameters(
            session_overrides=overrides,
        )


def test_map_age_must_cover_two_publication_periods() -> None:
    with pytest.raises(ValueError, match="two configured map publication periods"):
        resolve_parameters(
            session_overrides={"map_input.max_age_s": 0.35},
            map_publish_hz=5.0,
        )


def test_environment_uses_native_contract_names() -> None:
    resolved = resolve_parameters(
        session_overrides={
            "segment.max_distance_m": 2.5,
            "segment.max_waypoints": 20,
        },
    )
    environment = resolved.environment()
    assert environment["LINGTU_NAV_SEGMENT_MAX_DISTANCE_M"] == "2.5"
    assert environment["LINGTU_NAV_SEGMENT_MAX_WAYPOINTS"] == "20"
    assert environment["LINGTU_NAV_SEGMENT_MAP_MAX_AGE_S"] == "0.35"
