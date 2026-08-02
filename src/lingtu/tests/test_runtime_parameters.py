from __future__ import annotations

import pytest

from lingtu.runtime_parameters import resolve_runtime_parameters


PROFILES = {
    "explore": {
        "segment.max_distance_m": 4.0,
        "risk.stop_threshold": 60.0,
        "risk.resume_threshold": 45.0,
    }
}


def test_parameter_precedence_and_sources_are_explicit() -> None:
    resolved = resolve_runtime_parameters(
        parameter_profile="explore",
        profiles=PROFILES,
        env_overrides={
            "segment.max_distance_m": 3.0,
            "map_input.max_age_s": 0.5,
        },
        session_overrides={"segment.max_distance_m": 2.0},
        map_publish_hz=10.0,
        robot_max_speed_mps=0.4,
    )

    explanation = resolved.explanation()
    assert explanation["parameters"]["segment.max_distance_m"] == {
        "value": 2.0,
        "source": "session_override",
        "env_key": "LINGTU_NAV_SEGMENT_MAX_DISTANCE_M",
    }
    assert explanation["parameters"]["risk.stop_threshold"]["source"] == (
        "parameter_profile:explore"
    )
    assert explanation["parameters"]["map_input.max_age_s"]["source"] == (
        "env_override"
    )
    assert explanation["parameters"]["map_input.max_cells"]["source"] == (
        "code_default"
    )
    assert explanation["diagnostics"]["map_age_budget_frames"] == pytest.approx(5.0)
    assert explanation["diagnostics"]["travel_during_map_age_m"] == pytest.approx(0.2)


def test_parameter_profile_has_higher_priority_than_env() -> None:
    resolved = resolve_runtime_parameters(
        parameter_profile="explore",
        profiles=PROFILES,
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
        resolve_runtime_parameters(
            parameter_profile=None,
            profiles={},
            session_overrides=overrides,
        )


def test_map_age_must_cover_two_publication_periods() -> None:
    with pytest.raises(ValueError, match="two configured map publication periods"):
        resolve_runtime_parameters(
            parameter_profile=None,
            profiles={},
            session_overrides={"map_input.max_age_s": 0.35},
            map_publish_hz=5.0,
        )


def test_environment_uses_native_contract_names() -> None:
    resolved = resolve_runtime_parameters(
        parameter_profile=None,
        profiles={},
        session_overrides={
            "segment.max_distance_m": 2.5,
            "segment.max_waypoints": 20,
        },
    )
    environment = resolved.environment()
    assert environment["LINGTU_NAV_SEGMENT_MAX_DISTANCE_M"] == "2.5"
    assert environment["LINGTU_NAV_SEGMENT_MAX_WAYPOINTS"] == "20"
    assert environment["LINGTU_NAV_SEGMENT_MAP_MAX_AGE_S"] == "0.35"
