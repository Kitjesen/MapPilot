"""Tests for immutable session binding qualification."""

# ruff: noqa: S101

from __future__ import annotations

import pytest

from sim.runtime.coordinator.readiness import (
    BindingFacet,
    BindingReadiness,
    BindingReadinessError,
    BindingState,
)


def test_headless_session_requires_physics_and_control_only() -> None:
    readiness = BindingReadiness.from_session(
        {"runtime": {"mode": "headless", "required_bindings": ["physics", "control"]}},
        model_generation=4,
        reset_generation=2,
    )

    assert readiness.required_facets == frozenset(
        {BindingFacet.PHYSICS, BindingFacet.CONTROL}
    )
    assert readiness.state(BindingFacet.VISUAL) is BindingState.UNBOUND
    assert not readiness.is_ready

    readiness = readiness.mark_prepared(BindingFacet.PHYSICS).mark_active(
        BindingFacet.PHYSICS
    )
    with pytest.raises(BindingReadinessError, match="control"):
        readiness.require_ready()

    readiness = readiness.mark_prepared(BindingFacet.CONTROL).mark_active(
        BindingFacet.CONTROL
    )
    assert readiness.is_ready
    readiness.require_ready()


def test_session_required_facets_are_derived_and_optional_headless_facets_do_not_block() -> None:
    readiness = BindingReadiness.from_session(
        {"runtime": {"mode": "headless", "required_bindings": ["physics", "control"]}}
    )
    ready = (
        readiness.mark_prepared(BindingFacet.PHYSICS)
        .mark_active(BindingFacet.PHYSICS)
        .mark_prepared(BindingFacet.CONTROL)
        .mark_active(BindingFacet.CONTROL)
    )

    assert ready.is_ready
    assert ready.state(BindingFacet.VISUAL) is BindingState.UNBOUND
    assert ready.state(BindingFacet.SENSORS) is BindingState.UNBOUND


def test_active_requires_current_generations_and_model_change_invalidates_old_bindings() -> None:
    readiness = BindingReadiness.for_required(
        {"physics", "control"}, model_generation=7, reset_generation=3
    )
    with pytest.raises(BindingReadinessError, match="PREPARED"):
        readiness.mark_active("physics")
    active = (
        readiness.mark_prepared("physics")
        .mark_active("physics")
        .mark_prepared("control")
        .mark_active("control")
    )
    assert active.is_ready

    with pytest.raises(BindingReadinessError, match="model_generation"):
        active.mark_active("physics", model_generation=8)

    rebuilt = active.with_generations(model_generation=8, reset_generation=0)
    assert rebuilt.model_generation == 8
    assert rebuilt.reset_generation == 0
    assert rebuilt.state("physics") is BindingState.UNBOUND
    assert rebuilt.state("control") is BindingState.UNBOUND
    assert not rebuilt.is_ready


def test_reset_generation_preserves_bindings_while_model_generation_invalidates_them() -> None:
    readiness = BindingReadiness.for_required(
        {"physics", "control"}, model_generation=1, reset_generation=9
    )
    active = (
        readiness.mark_prepared("physics")
        .mark_active("physics")
        .mark_prepared("control")
        .mark_active("control")
    )

    reset = active.with_generations(model_generation=1, reset_generation=10)
    assert reset.is_ready
    assert reset.state("physics") is BindingState.ACTIVE
    assert reset.state("control") is BindingState.ACTIVE
    assert reset.bindings[BindingFacet.CONTROL].reset_generation == 10

    rebuilt = reset.with_generations(model_generation=2, reset_generation=0)
    assert rebuilt.state("physics") is BindingState.UNBOUND
    assert rebuilt.state("control") is BindingState.UNBOUND


def test_active_binding_can_be_retracted_to_prepared_and_block_readiness() -> None:
    readiness = BindingReadiness.for_required(
        {"physics"}, model_generation=1, reset_generation=0
    )
    active = readiness.mark_prepared("physics").mark_active("physics")

    retracted = active.mark_prepared("physics")

    assert retracted.state("physics") is BindingState.PREPARED
    assert not retracted.is_ready
    with pytest.raises(BindingReadinessError, match="physics"):
        retracted.require_ready()


def test_failed_reason_is_visible_and_survives_reset_generation() -> None:
    readiness = BindingReadiness.for_required(
        {"physics", "control"}, model_generation=1, reset_generation=9
    )
    failed = readiness.mark_failed("control", "controller asset did not load")

    assert failed.failure_reason("control") == "controller asset did not load"
    assert failed.failures == {BindingFacet.CONTROL: "controller asset did not load"}
    with pytest.raises(BindingReadinessError, match="controller asset did not load") as exc:
        failed.require_ready()
    assert exc.value.reasons[BindingFacet.CONTROL] == "controller asset did not load"

    reset = failed.with_generations(model_generation=1, reset_generation=10)
    assert reset.state("control") is BindingState.FAILED
    assert reset.failure_reason("control") == "controller asset did not load"


def test_transitions_are_immutable_and_invalid_facets_are_rejected() -> None:
    readiness = BindingReadiness.for_required({"physics", "control"})
    prepared = readiness.mark_prepared("physics")

    assert readiness.state("physics") is BindingState.UNBOUND
    assert prepared.state("physics") is BindingState.PREPARED
    with pytest.raises(BindingReadinessError, match="must be PREPARED"):
        readiness.mark_failed("physics", "asset missing").mark_active("physics")
    with pytest.raises(ValueError, match="unsupported facet"):
        readiness.mark_active("camera")


def test_non_headless_session_can_require_visual_and_sensors() -> None:
    readiness = BindingReadiness.from_session(
        {
            "runtime": {
                "mode": "realtime",
                "required_bindings": ["physics", "visual", "sensors", "control"],
            }
        }
    )

    active = readiness
    for facet in BindingFacet:
        active = active.mark_prepared(facet).mark_active(facet)
    assert active.is_ready
