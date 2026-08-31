"""Rollback journal for one simulation Product switch."""

from __future__ import annotations

import json
import os
from collections.abc import Mapping
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from lingtu.switch_contracts import (
    is_product_session_id,
    map_identity_from_native,
    optional_map_identity_from_native,
)

_SCHEMA = "lingtu.sim_switch_journal.v3"
_NAME = "switch.json"
_STATES = {
    "prepared",
    "previous_stopping",
    "target_starting",
    "current_committed",
}


@dataclass(frozen=True)
class _SwitchPlanRef:
    """RunPlan identity needed to settle an interrupted switch."""

    path: Path
    product_session_id: str


@dataclass(frozen=True)
class _SimSwitchJournal:
    """One durable simulation switch operation."""

    state: str
    target: _SwitchPlanRef
    previous: _SwitchPlanRef | None
    map_activation: dict[str, Any] | None = None

    def with_state(self, state: str) -> _SimSwitchJournal:
        return replace(self, state=state)

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema": _SCHEMA,
            "state": self.state,
            "target": _ref_dict(self.target),
            "previous": None if self.previous is None else _ref_dict(self.previous),
            "map_activation": self.map_activation,
        }


def _new_switch_journal(
    *,
    target_path: Path,
    target_product_session_id: str,
    previous_path: Path | None,
    previous_product_session_id: str | None,
    map_activation: Mapping[str, Any] | None = None,
) -> _SimSwitchJournal:
    if (previous_path is None) != (previous_product_session_id is None):
        raise RuntimeError("previous RunPlan path and product_session_id must be paired")
    return _SimSwitchJournal(
        state="prepared",
        target=_SwitchPlanRef(target_path, target_product_session_id),
        previous=(
            None
            if previous_path is None
            else _SwitchPlanRef(previous_path, str(previous_product_session_id))
        ),
        map_activation=(
            None if map_activation is None else _validated_map_activation(map_activation)
        ),
    )


def _load_switch_journal(state_root: Path) -> _SimSwitchJournal | None:
    root = state_root
    try:
        payload = json.loads((root / _NAME).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("simulation switch journal is invalid") from exc
    try:
        if not isinstance(payload, dict) or payload.get("schema") != _SCHEMA:
            raise RuntimeError("simulation switch journal schema is invalid")
        journal = _SimSwitchJournal(
            state=payload["state"],
            target=_parse_ref(payload["target"], root, "target"),
            previous=(
                None
                if payload.get("previous") is None
                else _parse_ref(payload["previous"], root, "previous")
            ),
            map_activation=(
                None
                if payload.get("map_activation") is None
                else _validated_map_activation(payload["map_activation"])
            ),
        )
        _validate_journal(journal, root)
        return journal
    except (KeyError, TypeError) as exc:
        raise RuntimeError("simulation switch journal fields are invalid") from exc


def _publish_switch_journal(state_root: Path, journal: _SimSwitchJournal) -> None:
    _validate_journal(journal, state_root)
    path = state_root / _NAME
    if path.exists():
        raise RuntimeError("an incomplete simulation switch journal already exists")
    _atomic_replace(path, _json_bytes(journal))


def _advance_switch_journal(
    state_root: Path,
    journal: _SimSwitchJournal,
    state: str,
) -> _SimSwitchJournal:
    updated = journal.with_state(state)
    if state not in _STATES:
        raise RuntimeError("simulation switch journal state is invalid")
    _atomic_replace(state_root / _NAME, _json_bytes(updated))
    return updated


def _remove_switch_journal(state_root: Path, _journal: _SimSwitchJournal) -> None:
    try:
        (state_root / _NAME).unlink()
    except OSError as exc:
        raise RuntimeError("simulation switch journal cannot be removed") from exc


def _validate_journal(journal: _SimSwitchJournal, root: Path) -> None:
    if journal.state not in _STATES:
        raise RuntimeError("simulation switch journal state is invalid")
    _validate_ref(journal.target, root, "target")
    if journal.previous is not None:
        _validate_ref(journal.previous, root, "previous")


def _validated_map_activation(value: Any) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RuntimeError("simulation switch journal map_activation is invalid")
    token = value.get("activation_token")
    if not isinstance(token, str) or not token:
        raise RuntimeError("simulation switch journal map_activation token is invalid")
    return {
        "activation_token": token,
        "target": _validated_map_identity(value.get("target"), "target", present=True),
        "previous": _validated_map_identity(value.get("previous"), "previous"),
    }


def _validated_map_identity(
    value: Any,
    field: str,
    *,
    present: bool | None = None,
) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise RuntimeError(f"simulation switch journal map_activation.{field} is invalid")
    payload = dict(value)
    if present is True:
        map_identity_from_native(payload, field_name=field)
    else:
        optional_map_identity_from_native(payload, field_name=field)
    return payload


def _parse_ref(value: Any, root: Path, field: str) -> _SwitchPlanRef:
    if not isinstance(value, Mapping):
        raise RuntimeError(f"simulation switch journal {field} is invalid")
    ref = _SwitchPlanRef(Path(value["run_plan_path"]), value["product_session_id"])
    _validate_ref(ref, root, field)
    return ref


def _validate_ref(ref: _SwitchPlanRef, root: Path, field: str) -> None:
    if not is_product_session_id(ref.product_session_id):
        raise RuntimeError(f"simulation switch journal {field} identity is invalid")
    if ref.path != root / f"plan-{ref.product_session_id}.json":
        raise RuntimeError(
            f"simulation switch journal {field} path must be an exact state-root child"
        )


def _atomic_replace(path: Path, payload: bytes) -> None:
    temporary = path.with_suffix(".tmp")
    try:
        temporary.write_bytes(payload)
        os.replace(temporary, path)
    except OSError as exc:
        raise RuntimeError("simulation switch journal cannot be persisted") from exc
    finally:
        temporary.unlink(missing_ok=True)


def _json_bytes(journal: _SimSwitchJournal) -> bytes:
    return (json.dumps(journal.as_dict(), separators=(",", ":")) + "\n").encode()


def _ref_dict(ref: _SwitchPlanRef) -> dict[str, str]:
    return {
        "run_plan_path": str(ref.path),
        "product_session_id": ref.product_session_id,
    }


__all__: list[str] = []
