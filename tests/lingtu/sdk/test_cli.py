"""Inspection commands exposed by the thin SDK CLI."""

from __future__ import annotations

import json
import sys
from unittest.mock import Mock, patch

import pytest

from lingtu.sdk import CommandResult
from lingtu.sdk import cli as sdk_cli


def test_stop_cli_uses_emergency_control_without_cancelling_navigation(capsys) -> None:
    robot = Mock()
    robot.system.emergency_stop.return_value = CommandResult(ok=True, message="stop requested")

    with (
        patch.object(sys, "argv", ["lingtu-sdk", "stop"]),
        patch.object(sdk_cli, "LingTuClient", return_value=robot),
    ):
        sdk_cli.main()

    robot.system.emergency_stop.assert_called_once_with()
    robot.navigation.cancel.assert_not_called()
    assert capsys.readouterr().out == "stop requested\n"
    robot.close.assert_called_once_with()


def test_inspection_start_cli_calls_sdk_and_prints_gateway_receipt(capsys) -> None:
    robot = Mock()
    robot.tasks.start_inspection.return_value = CommandResult(
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

    robot.tasks.start_inspection.assert_called_once_with(
        "route-1",
        map_id="factory",
        revision=4,
        request_id="request-1",
    )
    assert json.loads(capsys.readouterr().out)["task_id"] == "inspection-task-1"
    robot.close.assert_called_once_with()


def test_inspection_status_cli_calls_sdk(capsys) -> None:
    robot = Mock()
    robot.tasks.inspection_task.return_value = {
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

    robot.tasks.inspection_task.assert_called_once_with("inspection-task-1")
    assert json.loads(capsys.readouterr().out)["current_state"] == "EXECUTING"
    robot.close.assert_called_once_with()


@pytest.mark.parametrize("action", ["pause", "resume", "cancel"])
def test_inspection_control_cli_calls_sdk(action: str, capsys) -> None:
    robot = Mock()
    method = getattr(robot.tasks, f"{action}_inspection")
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
    method = getattr(robot.tasks, sdk_method)
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


@pytest.mark.parametrize("heading", [[], ["--yaw", "0"]])
def test_go_cli_preserves_xyz_units_and_optional_heading(heading):
    robot = Mock()
    robot.navigation.go.return_value = CommandResult(ok=True)
    with (patch.object(sys, "argv", ["lingtu-sdk", "go", "1", "2", "--z", "3",
                                     "--max-speed-mps", "0.2", "--tolerance-m", "0.1", *heading]),
          patch.object(sdk_cli, "LingTuClient", return_value=robot)):
        sdk_cli.main()
    robot.navigation.go.assert_called_once_with(1, 2, 0 if heading else None, z=3,
                                                max_speed_mps=.2, acceptance_radius_m=.1)


@pytest.mark.parametrize("action", ["pause", "resume", "cancel", "speed"])
def test_navigation_cli_control_uses_task_identity(action, capsys):
    robot = Mock()
    name = "set_speed_limit" if action == "speed" else action
    method = getattr(robot.navigation, name)
    method.return_value = CommandResult(ok=True, raw={"accepted": True})
    argv = ["lingtu-sdk", f"nav-{action}", "task-1", "--request-id", "request-1"]
    if action == "speed":
        argv.append("0.2")
    with patch.object(sys, "argv", argv), patch.object(sdk_cli, "LingTuClient", return_value=robot):
        sdk_cli.main()
    if action == "cancel":
        method.assert_called_once_with("operator_cancel", task_id="task-1", request_id="request-1")
    elif action == "speed":
        method.assert_called_once_with("task-1", 0.2, request_id="request-1")
    else:
        method.assert_called_once_with("task-1", reason=f"operator_{action}", request_id="request-1")
    assert json.loads(capsys.readouterr().out) == {"accepted": True}


@pytest.mark.parametrize("wait", [False, True])
def test_save_cli_manual_processing(wait, capsys):
    robot = Mock()
    robot.map.save.return_value = CommandResult(ok=True, raw={"accepted": True, "operation_id": "save_1"})
    robot.map.save_and_wait.return_value = {"operation": {"source_saved": True}}
    argv = ["lingtu-sdk", "save-map", "hall", "--no-auto-process", "--request-id", "save_request"]
    if wait:
        argv.append("--wait-for-source")
    with patch.object(sys, "argv", argv), patch.object(sdk_cli, "LingTuClient", return_value=robot):
        sdk_cli.main()
    if wait:
        robot.map.save_and_wait.assert_called_once_with("hall", request_id="save_request", auto_process=False,
                                                       timeout=300, wait_for_source=True)
        assert json.loads(capsys.readouterr().out)["operation"]["source_saved"]
    else:
        robot.map.save.assert_called_once_with("hall", request_id="save_request", auto_process=False)
        assert json.loads(capsys.readouterr().out)["operation_id"] == "save_1"


@pytest.mark.parametrize("action", ["process", "retry", "cancel", "reoptimize"])
def test_map_operation_cli(action, capsys):
    robot = Mock()
    method = getattr(robot.map, f"{action}_operation")
    method.return_value = CommandResult(ok=True, raw={"accepted": True})
    with (patch.object(sys, "argv", ["lingtu-sdk", f"map-{action}", "save_1"]),
          patch.object(sdk_cli, "LingTuClient", return_value=robot)):
        sdk_cli.main()
    if action == "reoptimize":
        method.assert_called_once_with("save_1", request_id=None)
    else:
        method.assert_called_once_with("save_1")
    assert json.loads(capsys.readouterr().out)["accepted"]


def test_cli_api_key_from_environment_is_not_printed(monkeypatch, capsys):
    robot = Mock()
    robot.map.download_source.return_value = "source.pcd"
    monkeypatch.setenv("LINGTU_API_KEY", "private-api-key")
    with (patch.object(sys, "argv", ["lingtu-sdk", "map-download-source", "save_1", "source.pcd"]),
          patch.object(sdk_cli, "LingTuClient", return_value=robot) as client):
        sdk_cli.main()
    client.assert_called_once_with("127.0.0.1", 5050, api_key="private-api-key", base_url=None)
    robot.map.download_source.assert_called_once_with("save_1", "source.pcd")
    assert capsys.readouterr().out == "source.pcd\n"


@pytest.mark.parametrize(("options", "host", "port", "base_url"), [
    (["--host", "robot.local", "--port", "6060"], "robot.local", 6060, None),
    (["--base-url", "https://robot.example:7443"], "127.0.0.1", 5050, "https://robot.example:7443"),
])
def test_cli_connection_origin(options, host, port, base_url, capsys):
    robot = Mock()
    robot.tasks.inspection_routes.return_value = {"ok": True}
    with (
        patch.object(sys, "argv", ["lingtu-sdk", *options, "--api-key", "test-key", "inspection-routes"]),
        patch.object(sdk_cli, "LingTuClient", return_value=robot) as client,
    ):
        sdk_cli.main()
    client.assert_called_once_with(host, port, api_key="test-key", base_url=base_url)
    assert json.loads(capsys.readouterr().out) == {"ok": True}
    robot.close.assert_called_once_with()
