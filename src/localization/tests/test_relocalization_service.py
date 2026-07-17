from __future__ import annotations

import ast
import subprocess
from pathlib import Path

from runtime.adapters.native.relocalization import NativeSlamRelocalizationService

ROOT = Path(__file__).resolve().parents[3]


def _function_defs(tree: ast.AST, name: str) -> list[ast.FunctionDef | ast.AsyncFunctionDef]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == name
    ]


def _contains_subprocess_call(tree: ast.AST) -> bool:
    return any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr in {"run", "Popen", "check_output"}
        and isinstance(node.func.value, ast.Name)
        and node.func.value.id == "subprocess"
        for node in ast.walk(tree)
    )


def test_gateway_relocalization_routes_do_not_spawn_processes() -> None:
    tree = ast.parse(
        (ROOT / "src" / "gateway" / "routes" / "operations.py").read_text(encoding="utf-8-sig"),
        filename="operations.py",
    )

    for function_name in ("slam_auto_relocalize", "slam_relocalize"):
        matches = _function_defs(tree, function_name)
        assert len(matches) == 1
        assert not _contains_subprocess_call(matches[0])


def test_removed_ros_relocalization_shell_does_not_return() -> None:
    assert not (ROOT / "src" / "localization" / "relocalization.py").exists()
    assert not (ROOT / "src" / "localization" / "adapters" / "ros2" / "slam_bridge.py").exists()


def _capture_native_call(monkeypatch, result: str):
    calls = []

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(args, 0, stdout=result, stderr="")

    monkeypatch.setattr(
        "runtime.adapters.native.relocalization.subprocess.run",
        fake_run,
    )
    return calls


def test_native_seeded_relocalization_uses_control_binary(monkeypatch, tmp_path) -> None:
    binary = str(tmp_path / "lingtu_slam_control")
    pcd_path = tmp_path / "map with spaces" / "map.pcd"
    monkeypatch.setenv("LINGTU_SLAM_CONTROL", binary)
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "7")
    calls = _capture_native_call(
        monkeypatch,
        '{"success":true,"message":"relocalized","relocalization_quality":0.123}\n',
    )

    result = NativeSlamRelocalizationService().relocalize_saved_map(pcd_path, 1.0, 2.0, 0.3, timeout_s=12.0)

    assert result.success is True
    assert result.quality == 0.123
    args, kwargs = calls[0]
    assert args == [
        binary,
        "relocalize",
        str(pcd_path),
        "--x",
        "1",
        "--y",
        "2",
        "--yaw",
        "0.3",
        "--domain-id",
        "7",
        "--timeout-s",
        "12",
    ]
    assert kwargs["timeout"] == 17.0
    assert kwargs["check"] is False


def test_native_global_relocalization_uses_control_binary(monkeypatch, tmp_path) -> None:
    binary = str(tmp_path / "lingtu_slam_control")
    monkeypatch.setenv("LINGTU_SLAM_CONTROL", binary)
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "7")
    calls = _capture_native_call(
        monkeypatch,
        '{"success":true,"message":"relocalized",'
        '"last_relocalization_message":"native_global_relocalized","quality":0.05}\n',
    )

    result = NativeSlamRelocalizationService().trigger_global_relocalize(timeout_s=9.0)

    assert result.success is True
    assert result.message == "native_global_relocalized"
    assert result.quality == 0.05
    args, kwargs = calls[0]
    assert args == [
        binary,
        "global-relocalize",
        "--domain-id",
        "7",
        "--timeout-s",
        "9",
    ]
    assert kwargs["timeout"] == 14.0


def test_native_global_status_uses_typed_control_binary(monkeypatch, tmp_path) -> None:
    binary = str(tmp_path / "lingtu_slam_control")
    monkeypatch.setenv("LINGTU_SLAM_CONTROL", binary)
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "7")
    calls = _capture_native_call(
        monkeypatch,
        '{"success":true,"message":"status","relocalization_quality":0.91,"relocalization_state":"TRACKING"}\n',
    )

    result = NativeSlamRelocalizationService().query_global_relocalize_status(timeout_s=4.0)

    assert result.success is True
    assert result.quality == 0.91
    args, kwargs = calls[0]
    assert args == [
        binary,
        "status",
        "--domain-id",
        "7",
        "--timeout-s",
        "4",
    ]
    assert kwargs["timeout"] == 9.0


def test_native_track_against_map_uses_typed_control_binary(monkeypatch, tmp_path) -> None:
    binary = str(tmp_path / "lingtu_slam_control")
    pcd_path = tmp_path / "map" / "map.pcd"
    monkeypatch.setenv("LINGTU_SLAM_CONTROL", binary)
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "7")
    calls = _capture_native_call(
        monkeypatch,
        '{"success":true,"message":"track_against_map_started","track_against_map_enabled":true}\n',
    )

    result = NativeSlamRelocalizationService().track_against_map(pcd_path, 1.0, 2.0, 0.3, timeout_s=6.0)

    assert result.success is True
    assert result.message == "track_against_map_started"
    args, kwargs = calls[0]
    assert args == [
        binary,
        "track-against-map",
        str(pcd_path),
        "--x",
        "1",
        "--y",
        "2",
        "--yaw",
        "0.3",
        "--domain-id",
        "7",
        "--timeout-s",
        "6",
    ]
    assert kwargs["timeout"] == 11.0
