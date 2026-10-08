"""Inspection commands exposed by the thin SDK CLI."""

from __future__ import annotations

import json
import sys
from unittest.mock import Mock, patch

import pytest

from lingtu.sdk import CommandResult
from lingtu.sdk import cli as sdk_cli


def test_inspection_start_cli_calls_sdk_and_prints_gateway_receipt(capsys) -> None:
    robot = Mock()
    robot.start_inspection.return_value = CommandResult(
        ok=True,
        accepted=True,
        task_id="inspection-task-1",
        request_id="request-1",
        raw={
            "ok": True,
            "accepted": True,
            "task_id": "inspection-task-1",
            "request_id": "request-1",
        },
    )

    with (
        patch.object(
            sys,
            "argv",
            [
                "lingtu-sdk",
                "inspection-start",
                "route-1",
                "--map-id",
                "factory",
                "--revision",
                "4",
                "--request-id",
                "request-1",
            ],
        ),
        patch.object(sdk_cli, "LingTuClient", return_value=robot),
    ):
        sdk_cli.main()

    robot.start_inspection.assert_called_once_with(
        "route-1",
        map_id="factory",
        revision=4,
        request_id="request-1",
    )
    assert json.loads(capsys.readouterr().out)["task_id"] == "inspection-task-1"
    robot.close.assert_called_once_with()


def test_inspection_status_cli_calls_sdk(capsys) -> None:
    robot = Mock()
    robot.inspection_task.return_value = {
        "found": True,
        "task_id": "inspection-task-1",
        "current_state": "EXECUTING",
    }

    with (
        patch.object(
            sys,
            "argv",
            ["lingtu-sdk", "inspection-status", "inspection-task-1"],
        ),
        patch.object(sdk_cli, "LingTuClient", return_value=robot),
    ):
        sdk_cli.main()

    robot.inspection_task.assert_called_once_with("inspection-task-1")
    assert json.loads(capsys.readouterr().out)["current_state"] == "EXECUTING"
    robot.close.assert_called_once_with()


@pytest.mark.parametrize("action", ["pause", "resume", "cancel"])
def test_inspection_control_cli_calls_sdk(action: str, capsys) -> None:
    robot = Mock()
    method = getattr(robot, f"{action}_inspection")
    method.return_value = CommandResult(
        ok=True,
        accepted=True,
        task_id="inspection-task-1",
        request_id=f"request-{action}",
        raw={
            "ok": True,
            "action": action,
            "task_id": "inspection-task-1",
            "request_id": f"request-{action}",
        },
    )

    with (
        patch.object(
            sys,
            "argv",
            [
                "lingtu-sdk",
                f"inspection-{action}",
                "inspection-task-1",
                "--request-id",
                f"request-{action}",
            ],
        ),
        patch.object(sdk_cli, "LingTuClient", return_value=robot),
    ):
        sdk_cli.main()

    method.assert_called_once_with(
        "inspection-task-1",
        reason=f"operator_{action}",
        request_id=f"request-{action}",
    )
    assert json.loads(capsys.readouterr().out)["action"] == action
    robot.close.assert_called_once_with()


@pytest.mark.parametrize(
    ("command", "sdk_method"),
    [("inspection-routes", "inspection_routes"), ("inspection-report", "inspection_report")],
)
def test_inspection_read_cli_calls_sdk(command: str, sdk_method: str, capsys) -> None:
    robot = Mock()
    method = getattr(robot, sdk_method)
    method.return_value = {"ok": True}
    argv = ["lingtu-sdk", command]
    expected_args: tuple[str, ...] | tuple[None]
    if command == "inspection-routes":
        expected_args = (None,)
    else:
        argv.append("inspection-task-1")
        expected_args = ("inspection-task-1",)

    with (
        patch.object(sys, "argv", argv),
        patch.object(sdk_cli, "LingTuClient", return_value=robot),
    ):
        sdk_cli.main()

    method.assert_called_once_with(*expected_args)
    assert json.loads(capsys.readouterr().out) == {"ok": True}
    robot.close.assert_called_once_with()
