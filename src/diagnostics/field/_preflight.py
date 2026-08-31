"""Shared parsing and result assembly for field preflight checks."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

ZERO_EPSILON = 1e-6
STATUS_MAX_AGE_S = 3.0
NAV_STATUS_SCHEMA = "lingtu.nav.endpoint.status.v1"
DRIVER_STATUS_SCHEMA = "lingtu.driver.status.v2"
DRIVER_MOTION_PRINCIPAL = "lingtu-driver@robot"
SUPPORTED_DRIVER_BACKENDS = frozenset({"go2", "doso"})


def mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def text(value: Any) -> str:
    return str(value or "").strip()


def strings(value: Any) -> set[str]:
    if not isinstance(value, (list, tuple, set, frozenset)):
        return set()
    return {item.strip() for item in value if isinstance(item, str) and item.strip()}


def status_entry(snapshot: Mapping[str, Any], name: str) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    entry = mapping(mapping(snapshot.get("status_files")).get(name))
    return entry, mapping(entry.get("json"))


def native_environment(current_run: Mapping[str, Any]) -> Mapping[str, Any]:
    run_plan = mapping(current_run.get("run_plan"))
    return mapping(mapping(run_plan.get("launch")).get("native_process_environment"))


def target_host(value: Any) -> str:
    target = text(value).lower()
    if not target:
        return ""
    if "://" in target:
        target = target.split("://", 1)[1]
    target = target.split("/", 1)[0]
    if target.startswith("["):
        closing = target.find("]")
        return target[1:closing] if closing > 0 else target
    return target.rsplit(":", 1)[0] if target.count(":") == 1 else target


def sdk2_interface(target: str) -> str:
    prefix = "dds://"
    suffix = "/rt/api/sport/request"
    if target.startswith(prefix) and target.endswith(suffix):
        return target[len(prefix) : -len(suffix)]
    return ""


def number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def fresh(entry: Mapping[str, Any], max_age_s: float) -> bool:
    age_s = number(entry.get("age_s"))
    return bool(entry.get("exists") is True and age_s is not None and 0.0 <= age_s <= max_age_s)


def zero_twist(value: Any, aliases: Sequence[Sequence[str]]) -> bool:
    twist = mapping(value)
    components = (next((twist[name] for name in names if name in twist), None) for names in aliases)
    parsed = tuple(number(component) for component in components)
    return all(component is not None and abs(component) <= ZERO_EPSILON for component in parsed)


class Evaluation:
    def __init__(self, stage: str, schema_version: str) -> None:
        self.stage = stage
        self.schema_version = schema_version
        self.checks: list[dict[str, Any]] = []
        self.blockers: list[str] = []

    def check(
        self,
        check_id: str,
        ok: bool,
        *,
        expected: Any,
        observed: Any,
        detail: str = "",
    ) -> None:
        passed = bool(ok)
        check = {
            "id": check_id,
            "ok": passed,
            "expected": expected,
            "observed": observed,
        }
        if detail:
            check["detail"] = detail
        self.checks.append(check)
        if not passed:
            self.blockers.append(check_id)

    def result(self) -> dict[str, Any]:
        motion_allowed = self.stage == "motion" and not self.blockers
        if motion_allowed:
            motion_blockers: list[str] = []
        elif self.stage == "motion":
            motion_blockers = list(self.blockers)
        else:
            motion_blockers = ["motion_stage_not_evaluated"]
        return {
            "schema_version": self.schema_version,
            "stage": self.stage,
            "read_only": True,
            "authority_acquired": False,
            "command_published": False,
            "nonzero_motion_allowed": motion_allowed,
            "nonzero_motion_blockers": motion_blockers,
            "ok": not self.blockers,
            "blockers": list(self.blockers),
            "checks": list(self.checks),
        }
