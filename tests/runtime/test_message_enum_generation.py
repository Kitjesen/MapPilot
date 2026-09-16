"""Wire enum regression: numeric compatibility and both generated consumers."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest
from tools.generate_topic_contracts import load_constants

from message.generated import enums
from message.generated.schema import MESSAGE_FIELDS
from message.topics import TOPIC_SPECS

ROOT = Path(__file__).resolve().parents[2]

# Frozen pre-migration wire values, not a runtime registry.
WIRE_VALUES = {
    "NavigationControlMode": {"UNKNOWN": 0, "AUTONOMY": 1, "TELEOP": 2, "TELEOP_AVOID": 3},
    "NavigationLifecycle": {
        "IDLE": 0,
        "PLANNING": 1,
        "EXECUTING": 2,
        "PAUSED": 3,
        "RECOVERING": 4,
        "SUCCESS": 5,
        "FAILED": 6,
        "CANCELLED": 7,
    },
    "NavigationPlanningState": {"IDLE": 0, "PLANNING": 1, "READY": 2, "FAILED": 3},
    "NavigationExecutionState": {"IDLE": 0, "FOLLOWING": 1, "REACHED": 2, "BLOCKED": 3},
    "NavigationRecoveryState": {"IDLE": 0, "ACTIVE": 1, "SUCCEEDED": 2, "FAILED": 3},
    "NavigationGoalState": {"PLANNING": 1, "PATH_ACTIVE": 2, "FAILED": 3, "REACHED": 4, "CANCELLED": 5, "PAUSED": 6},
    "InspectionTaskEventKind": {
        "TASK_ACCEPTED": 1,
        "STATE_CHANGED": 2,
        "MILESTONE": 3,
        "STOP_CONFIRMATION_FAILED": 4,
        "EVIDENCE_RECORDED": 5,
    },
    "InspectionTaskState": {
        "IDLE": 0,
        "VALIDATING": 1,
        "PLANNING": 2,
        "NAVIGATING": 3,
        "DWELLING": 4,
        "PAUSED": 5,
        "RECOVERING": 6,
        "SUCCEEDED": 7,
        "FAILED": 8,
        "CANCELLED": 9,
        "SETTLING": 10,
        "ACTION_PENDING": 11,
        "PAUSING": 12,
        "CANCELLING": 13,
    },
    "ExplorationRunEventKind": {"ADMITTED": 1, "STATE_CHANGED": 2, "STOP_CONFIRMATION_FAILED": 3},
    "ExplorationRunState": {
        "ADMITTED": 1,
        "RUNNING": 2,
        "PAUSING": 3,
        "PAUSED": 4,
        "CANCELLING": 5,
        "COMPLETED": 6,
        "CANCELLED": 7,
        "FAILED": 8,
    },
    "NavigationCommandKind": {
        "GOAL": 1,
        "TASK_CANCEL": 2,
        "STOP": 4,
        "ESTOP": 5,
        "CLEAR_ESTOP": 6,
        "RESUME_AUTONOMY": 7,
        "PAUSE_TASK": 8,
        "RESUME_TASK": 9,
    },
    "OperatorMotionAction": {"CLAIM": 1, "RELEASE": 2, "HOLD": 3},
    "ExplorationCommandKind": {
        "START": 1,
        "PAUSE": 2,
        "RESUME": 3,
        "STOP": 4,
        "SET_DIRECTED_TARGET": 5,
        "CLEAR_DIRECTED_TARGET": 6,
    },
    "InspectionCommandKind": {"START": 1, "PAUSE": 2, "RESUME": 3, "CANCEL": 4},
    "GeofenceAction": {"ADD": 1, "REMOVE": 2, "CLEAR": 3, "ENABLE": 4, "DISABLE": 5, "LIST": 6},
}


def test_idl_and_python_preserve_wire_values() -> None:
    assert load_constants() == WIRE_VALUES
    for name, values in WIRE_VALUES.items():
        assert {member.name: member.value for member in getattr(enums, name)} == values


def test_every_dds_topic_has_an_idl_payload() -> None:
    for spec in TOPIC_SPECS.values():
        assert spec.message_type in MESSAGE_FIELDS, spec.topic


def test_cpp_protocol_consumers_compile_with_preserved_values() -> None:
    compiler = shutil.which("clang++") or shutil.which("g++")
    if compiler is None:
        pytest.skip("C++ compiler is not installed")
    source = "\n".join(
        f'#include "message/protocol/{domain}.hpp"'
        for domain in ("navigation", "operator_motion", "inspection", "exploration", "geofence")
    )
    source += '\n#include "nav/cpp/endpoint/nav/status/navigation_state.hpp"'
    source += '\n#include "nav/inspection/inspection.hpp"\n'
    cpp_names = {
        "NavigationControlMode": "NavigationControlState",
        "NavigationLifecycle": "NavigationLifecycleState",
    }
    for name, values in WIRE_VALUES.items():
        for member, value in values.items():
            cpp_member = "".join(part.title() for part in member.split("_"))
            if name == "NavigationCommandKind":
                cpp_member = {"PAUSE_TASK": "TaskPause", "RESUME_TASK": "TaskResume"}.get(member, cpp_member)
            if name not in {"NavigationCommandKind", "NavigationGoalState", "OperatorMotionAction"}:
                cpp_member = "k" + cpp_member
            source += (
                f"static_assert(static_cast<int>(lingtu::message::{cpp_names.get(name, name)}"
                f"::{cpp_member}) == {value});\n"
            )
    result = subprocess.run(
        [compiler, "-std=c++17", "-I", str(ROOT / "src"), "-x", "c++", "-fsyntax-only", "-"],
        input=source,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
