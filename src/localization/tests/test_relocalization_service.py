from __future__ import annotations

import ast
import platform
import subprocess
from pathlib import Path

import localization.slam_control as slam_control
from localization.adapters.relocalization import NativeSlamRelocalizationService

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

    for function_name in ("localization_relocalize", "localization_map_tracking"):
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
        "localization.adapters.relocalization.subprocess.run",
        fake_run,
    )
    return calls


def test_slam_control_discovery_selects_native_windows_binary(monkeypatch, tmp_path) -> None:
    module_path = tmp_path / "src" / "localization" / "slam_control.py"
    module_path.parent.mkdir(parents=True)
    linux_binary = tmp_path / "build" / "slam_core" / "slamctl"
    windows_binary = tmp_path / "build" / "slam-core-windows-x64" / "stage" / "bin" / "slamctl.exe"
    linux_binary.parent.mkdir(parents=True)
    windows_binary.parent.mkdir(parents=True)
    linux_binary.write_bytes(b"ELF")
    windows_binary.write_bytes(b"MZ")
    monkeypatch.delenv("LINGTU_SLAM_CONTROL", raising=False)
    monkeypatch.setattr(slam_control, "__file__", str(module_path))
    monkeypatch.setattr(platform, "system", lambda: "Windows")
    monkeypatch.setattr(slam_control.shutil, "which", lambda _name: None)

    assert Path(slam_control.slam_control_binary()) == windows_binary


def test_native_seeded_relocalization_uses_control_binary(monkeypatch, tmp_path) -> None:
    binary = str(tmp_path / "slamctl")
    monkeypatch.setenv("LINGTU_SLAM_CONTROL", binary)
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "7")
    calls = _capture_native_call(
        monkeypatch,
        '{"success":true,"message":"relocalized","relocalization_quality":0.123}\n',
    )

    result = NativeSlamRelocalizationService().relocalize_saved_map("factory", 1.0, 2.0, 0.3, timeout_s=12.0)

    assert result.success is True
    assert result.quality == 0.123
    args, kwargs = calls[0]
    assert args == [
        binary,
        "relocalize",
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
    binary = str(tmp_path / "slamctl")
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
    binary = str(tmp_path / "slamctl")
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


def test_native_track_against_map_uses_current_tracking_state(monkeypatch, tmp_path) -> None:
    binary = str(tmp_path / "slamctl")
    monkeypatch.setenv("LINGTU_SLAM_CONTROL", binary)
    monkeypatch.setenv("LINGTU_DDS_DOMAIN_ID", "7")
    calls = _capture_native_call(
        monkeypatch,
        '{"success":true,"message":"track_against_map_started","track_against_map_enabled":true}\n',
    )

    result = NativeSlamRelocalizationService().track_against_map(timeout_s=6.0)

    assert result.success is True
    assert result.message == "track_against_map_started"
    args, kwargs = calls[0]
    assert args == [
        binary,
        "track-against-map",
        "--domain-id",
        "7",
        "--timeout-s",
        "6",
    ]
    assert kwargs["timeout"] == 11.0
