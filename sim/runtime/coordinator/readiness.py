"""Immutable qualification state for simulation session bindings.

The coordinator owns the lifecycle; this module only validates and records
which session facets are usable for the current model/reset generation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Any, Iterable, Mapping


class BindingFacet(str, Enum):
    """A runtime facet that may be bound for a simulation session."""

    PHYSICS = "physics"
    VISUAL = "visual"
    SENSORS = "sensors"
    CONTROL = "control"


class BindingState(str, Enum):
    """Qualification state of one binding facet."""

    UNBOUND = "UNBOUND"
    PREPARED = "PREPARED"
    ACTIVE = "ACTIVE"
    FAILED = "FAILED"


class BindingReadinessError(RuntimeError):
    """Raised when a session is not qualified for the READY state."""

    def __init__(self, message: str, *, reasons: Mapping[BindingFacet, str] | None = None) -> None:
        self.reasons = MappingProxyType(dict(reasons or {}))
        super().__init__(message)


@dataclass(frozen=True)
class BindingQualification:
    """Immutable state and generation stamp for one facet."""

    facet: BindingFacet
    state: BindingState = BindingState.UNBOUND
    model_generation: int = 0
    reset_generation: int = 0
    failure_reason: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.facet, BindingFacet):
            raise ValueError("facet must be a BindingFacet")
        if not isinstance(self.state, BindingState):
            raise ValueError("state must be a BindingState")
        _validate_generation(self.model_generation, "model_generation")
        _validate_generation(self.reset_generation, "reset_generation")
        if self.state is BindingState.FAILED:
            if not isinstance(self.failure_reason, str) or not self.failure_reason.strip():
                raise ValueError("FAILED binding requires a non-empty failure_reason")
        elif self.failure_reason is not None:
            raise ValueError("failure_reason is only valid for FAILED bindings")


@dataclass(frozen=True)
class BindingReadiness:
    """Validated, immutable qualification state for one session generation."""

    required_facets: frozenset[BindingFacet]
    _bindings: Mapping[BindingFacet, BindingQualification] = field(
        repr=False, compare=True
    )
    model_generation: int = 0
    reset_generation: int = 0

    def __post_init__(self) -> None:
        _validate_generation(self.model_generation, "model_generation")
        _validate_generation(self.reset_generation, "reset_generation")
        required = _normalize_facets(self.required_facets, "required_facets")
        if BindingFacet.PHYSICS not in required:
            raise ValueError("required_facets must include physics")
        bindings = dict(self._bindings)
        expected = set(BindingFacet)
        if set(bindings) != expected:
            raise ValueError("bindings must contain exactly all simulation facets")
        for facet, binding in bindings.items():
            if not isinstance(binding, BindingQualification) or binding.facet is not facet:
                raise ValueError("bindings must contain matching BindingQualification values")
            if (
                binding.model_generation != self.model_generation
                or binding.reset_generation != self.reset_generation
            ):
                raise ValueError("binding generation does not match readiness generation")
        object.__setattr__(self, "required_facets", required)
        object.__setattr__(self, "_bindings", MappingProxyType(bindings))

    @classmethod
    def for_required(
        cls,
        required_facets: Iterable[BindingFacet | str],
        *,
        model_generation: int = 0,
        reset_generation: int = 0,
    ) -> BindingReadiness:
        """Create a session qualification object from an explicit requirement set."""

        required = _normalize_facets(required_facets, "required_facets")
        return cls(
            required,
            _initial_bindings(model_generation, reset_generation),
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    @classmethod
    def from_session(
        cls,
        declaration: Mapping[str, Any] | Any,
        *,
        model_generation: int = 0,
        reset_generation: int = 0,
    ) -> BindingReadiness:
        """Derive requirements from a session declaration.

        The canonical declaration stores them under ``runtime``.  A headless
        declaration without an explicit list defaults to physics/control;
        other modes default to all four facets.  Explicit lists always win.
        """

        if isinstance(declaration, Mapping):
            runtime = declaration.get("runtime", declaration)
            if not isinstance(runtime, Mapping):
                raise ValueError("session runtime declaration must be a mapping")
            mode = runtime.get("mode")
            values = runtime.get("required_bindings", runtime.get("required_facets"))
        else:
            runtime = getattr(declaration, "runtime", declaration)
            mode = getattr(runtime, "mode", None)
            values = getattr(runtime, "required_bindings", None)
            if values is None:
                values = getattr(runtime, "required_facets", None)
        if values is None:
            values = (
                (BindingFacet.PHYSICS, BindingFacet.CONTROL)
                if str(mode).lower() == "headless"
                else tuple(BindingFacet)
            )
        return cls.for_required(
            values,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    @property
    def bindings(self) -> Mapping[BindingFacet, BindingQualification]:
        """Read-only qualification records keyed by facet."""

        return self._bindings

    @property
    def is_ready(self) -> bool:
        """Whether every required facet is active for this generation."""

        return not self.blocking_reasons

    @property
    def blocking_reasons(self) -> Mapping[BindingFacet, str]:
        """Return why each required facet currently blocks READY."""

        reasons: dict[BindingFacet, str] = {}
        for facet in sorted(self.required_facets, key=lambda item: item.value):
            binding = self._bindings[facet]
            if binding.model_generation != self.model_generation:
                reasons[facet] = "activation belongs to an older model_generation"
            elif binding.reset_generation != self.reset_generation:
                reasons[facet] = "activation belongs to an older reset_generation"
            elif binding.state is BindingState.FAILED:
                reasons[facet] = binding.failure_reason or "binding failed"
            elif binding.state is not BindingState.ACTIVE:
                reasons[facet] = f"binding is {binding.state.value}"
        return MappingProxyType(reasons)

    @property
    def failures(self) -> Mapping[BindingFacet, str]:
        """Failure reasons for failed facets, including optional facets."""

        return MappingProxyType(
            {
                facet: binding.failure_reason
                for facet, binding in self._bindings.items()
                if binding.state is BindingState.FAILED and binding.failure_reason is not None
            }
        )

    def state(self, facet: BindingFacet | str) -> BindingState:
        """Return the current state of one facet."""

        return self._bindings[_facet(facet)].state

    def failure_reason(self, facet: BindingFacet | str) -> str | None:
        """Return the recorded failure reason for one facet, when present."""

        return self._bindings[_facet(facet)].failure_reason

    def require_ready(self) -> None:
        """Raise with facet-specific reasons unless the session is READY-safe."""

        if not self.is_ready:
            reasons = dict(self.blocking_reasons)
            detail = "; ".join(f"{facet.value}: {reason}" for facet, reason in reasons.items())
            raise BindingReadinessError(
                f"session is not READY; required bindings are not ACTIVE ({detail})",
                reasons=reasons,
            )

    def mark_prepared(
        self,
        facet: BindingFacet | str,
        *,
        model_generation: int | None = None,
        reset_generation: int | None = None,
    ) -> BindingReadiness:
        """Return a copy with one facet in PREPARED for this generation."""

        return self._transition(
            facet,
            BindingState.PREPARED,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    def mark_active(
        self,
        facet: BindingFacet | str,
        *,
        model_generation: int | None = None,
        reset_generation: int | None = None,
    ) -> BindingReadiness:
        """Return a copy with one prepared facet promoted to ACTIVE."""

        return self._transition(
            facet,
            BindingState.ACTIVE,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    def mark_failed(
        self,
        facet: BindingFacet | str,
        reason: str,
        *,
        model_generation: int | None = None,
        reset_generation: int | None = None,
    ) -> BindingReadiness:
        """Return a copy with one facet failed and its reason retained."""

        if not isinstance(reason, str) or not reason.strip():
            raise ValueError("failure reason must be a non-empty string")
        return self._transition(
            facet,
            BindingState.FAILED,
            model_generation=model_generation,
            reset_generation=reset_generation,
            failure_reason=reason.strip(),
        )

    def with_generations(
        self, *, model_generation: int, reset_generation: int
    ) -> BindingReadiness:
        """Advance generation stamps without confusing reset with model rebinding.

        A new MuJoCo model invalidates every dense runtime binding.  A reset of
        the same model preserves bindings and failures; consumers only receive
        the new reset-generation stamp.
        """

        _validate_generation(model_generation, "model_generation")
        _validate_generation(reset_generation, "reset_generation")
        if (model_generation, reset_generation) == (
            self.model_generation,
            self.reset_generation,
        ):
            return self
        if model_generation != self.model_generation:
            return BindingReadiness.for_required(
                self.required_facets,
                model_generation=model_generation,
                reset_generation=reset_generation,
            )
        rebound = {
            facet: BindingQualification(
                facet=facet,
                state=binding.state,
                model_generation=model_generation,
                reset_generation=reset_generation,
                failure_reason=binding.failure_reason,
            )
            for facet, binding in self._bindings.items()
        }
        return BindingReadiness(
            self.required_facets,
            rebound,
            model_generation=model_generation,
            reset_generation=reset_generation,
        )

    def _transition(
        self,
        facet: BindingFacet | str,
        target: BindingState,
        *,
        model_generation: int | None,
        reset_generation: int | None,
        failure_reason: str | None = None,
    ) -> BindingReadiness:
        key = _facet(facet)
        _check_generation(self.model_generation, model_generation, "model_generation")
        _check_generation(self.reset_generation, reset_generation, "reset_generation")
        current = self._bindings[key]
        if target is BindingState.PREPARED and current.state not in {
            BindingState.UNBOUND,
            BindingState.PREPARED,
            BindingState.ACTIVE,
        }:
            raise BindingReadinessError(f"{key.value} must be UNBOUND before PREPARED")
        if target is BindingState.ACTIVE and current.state not in {
            BindingState.PREPARED,
            BindingState.ACTIVE,
        }:
            raise BindingReadinessError(f"{key.value} must be PREPARED before ACTIVE")
        updated = dict(self._bindings)
        updated[key] = BindingQualification(
            key,
            target,
            self.model_generation,
            self.reset_generation,
            failure_reason,
        )
        return BindingReadiness(
            self.required_facets,
            updated,
            model_generation=self.model_generation,
            reset_generation=self.reset_generation,
        )


def _validate_generation(value: int, field_name: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")


def _facet(value: BindingFacet | str) -> BindingFacet:
    if isinstance(value, BindingFacet):
        return value
    if not isinstance(value, str):
        raise ValueError("unsupported facet; expected physics, visual, sensors, or control")
    try:
        return BindingFacet(value.strip().lower())
    except ValueError as exc:
        raise ValueError(f"unsupported facet: {value!r}") from exc


def _normalize_facets(values: Iterable[BindingFacet | str], field_name: str) -> frozenset[BindingFacet]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{field_name} must be an iterable of facets")
    try:
        facets = tuple(_facet(value) for value in values)
    except TypeError as exc:
        raise ValueError(f"{field_name} must be an iterable of facets") from exc
    if len(set(facets)) != len(facets):
        raise ValueError(f"{field_name} must contain unique facets")
    return frozenset(facets)


def _initial_bindings(model_generation: int, reset_generation: int) -> dict[BindingFacet, BindingQualification]:
    return {
        facet: BindingQualification(facet, BindingState.UNBOUND, model_generation, reset_generation)
        for facet in BindingFacet
    }


def _check_generation(expected: int, actual: int | None, field_name: str) -> None:
    if actual is not None:
        _validate_generation(actual, field_name)
        if actual != expected:
            raise BindingReadinessError(
                f"{field_name} {actual} does not match current {field_name} {expected}"
            )
