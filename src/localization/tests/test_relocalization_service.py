from __future__ import annotations

import ast
import shlex
import subprocess
from pathlib import Path

from runtime.relocalization import RelocalizationResult
from localization import relocalization


ROOT = Path(__file__).resolve().parents[3]


def _runtime_topic_default(field_name: str) -> str:
    tree = ast.parse(
        (ROOT / "src" / "runtime" / "runtime_interface.py").read_text(
            encoding="utf-8-sig",
        )
    )
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef) or node.name != "RuntimeTopics":
            continue
        for item in node.body:
            if (
                isinstance(item, ast.AnnAssign)
                and isinstance(item.target, ast.Name)
                and item.target.id == field_name
                and isinstance(item.value, ast.Constant)
                and isinstance(item.value.value, str)
            ):
                return item.value.value
    raise AssertionError(f"RuntimeTopics.{field_name} default not found")


def _function_defs(tree: ast.AST, name: str) -> list[ast.FunctionDef | ast.AsyncFunctionDef]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    ]


def _contains_subprocess_call(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(alias.name == "subprocess" for alias in node.names):
                return True
        elif (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr in {"run", "Popen", "check_output"}
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "subprocess"
        ):
            return True
    return False


def _imports_slam(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(alias.name.split(".", 1)[0] == "slam" for alias in node.names):
                return True
        elif isinstance(node, ast.ImportFrom) and node.module:
            if node.module.split(".", 1)[0] == "slam":
                return True
    return False


def test_relocalization_service_tokens_match_runtime_interface_without_runtime_import():
    assert relocalization.RELOCALIZE_SERVICE == _runtime_topic_default(
        "relocalize_service"
    )
    assert relocalization.GLOBAL_RELOCALIZE_SERVICE == _runtime_topic_default(
        "global_relocalize_service"
    )
    assert relocalization.GLOBAL_RELOCALIZE_STATUS_SERVICE == _runtime_topic_default(
        "global_relocalize_status_service"
    )


def test_gateway_relocalization_routes_delegate_subprocess_execution():
    tree = ast.parse(
        (
            ROOT / "src" / "gateway" / "routes" / "operations.py"
        ).read_text(encoding="utf-8-sig"),
        filename="operations.py",
    )
    assert not _imports_slam(tree)
    violations = []

    for function_name in ("slam_auto_relocalize", "slam_relocalize"):
        matches = _function_defs(tree, function_name)
        assert len(matches) == 1
        if _contains_subprocess_call(matches[0]):
            violations.append(function_name)

    assert violations == []


def test_slam_relocalization_service_uses_core_result_contract():
    tree = ast.parse(
        (
            ROOT / "src" / "localization" / "relocalization.py"
        ).read_text(encoding="utf-8-sig"),
        filename="relocalization.py",
    )
    assert not any(
        isinstance(node, ast.ClassDef) and node.name == "RelocalizationResult"
        for node in ast.walk(tree)
    )
    assert relocalization.RelocalizationResult is RelocalizationResult


def test_slam_bridge_auto_relocalize_checks_service_success_before_completion():
    tree = ast.parse(
        (
            ROOT / "src" / "localization" / "adapters" / "ros2" / "slam_bridge.py"
        ).read_text(encoding="utf-8-sig"),
        filename="slam_bridge.py",
    )
    matches = _function_defs(tree, "_auto_relocalize")
    assert len(matches) == 1

    success_checked = False
    failed_return_before_completion = False
    for node in ast.walk(matches[0]):
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not):
            operand = node.operand
            if (
                isinstance(operand, ast.Attribute)
                and operand.attr == "success"
                and isinstance(operand.value, ast.Name)
                and operand.value.id == "result"
            ):
                success_checked = True
        if isinstance(node, ast.If):
            assigned_failed = any(
                isinstance(child, ast.Assign)
                and any(
                    isinstance(target, ast.Attribute)
                    and target.attr == "_relocalization_state"
                    for target in child.targets
                )
                and isinstance(child.value, ast.Constant)
                and child.value.value == "failed"
                for child in node.body
            )
            returns = any(isinstance(child, ast.Return) for child in node.body)
            if assigned_failed and returns:
                failed_return_before_completion = True

    assert success_checked is True
    assert failed_return_before_completion is True


def test_slam_bridge_auto_relocalize_keeps_missing_map_fallback_reachable():
    text = (ROOT / "src" / "localization" / "adapters" / "ros2" / "slam_bridge.py").read_text(
        encoding="utf-8-sig",
    )
    tree = ast.parse(text, filename="slam_bridge.py")
    matches = _function_defs(tree, "_auto_relocalize")
    assert len(matches) == 1
    source = ast.get_source_segment(text, matches[0]) or ""

    assert 'if not contract["relocalization_supported"]:' in source
    assert "if not os.path.isfile(pcd_path):" in source
    assert 'or not os.path.isfile(pcd_path)' not in source


def test_trigger_global_relocalize_uses_global_service_and_unsets_rmw(monkeypatch):
    calls = []

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(args, 0, stdout="success=True\n", stderr="")

    monkeypatch.setattr(relocalization.subprocess, "run", fake_run)

    result = relocalization.trigger_global_relocalize(timeout_s=10.0)

    assert result.success is True
    assert calls
    args, kwargs = calls[0]
    command = args[2]
    assert args[:2] == ["bash", "-c"]
    assert _runtime_topic_default("global_relocalize_service") in command
    assert "unset RMW_IMPLEMENTATION" in command
    assert kwargs["timeout"] == 10.0


def test_query_global_relocalize_status_uses_status_service(monkeypatch):
    calls = []

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(
            args,
            0,
            stdout='success=True\nmessage="enabled=true map_loaded=true"\n',
            stderr="",
        )

    monkeypatch.setattr(relocalization.subprocess, "run", fake_run)

    result = relocalization.query_global_relocalize_status(timeout_s=5.0)

    assert result.success is True
    args, kwargs = calls[0]
    command = args[2]
    assert _runtime_topic_default("global_relocalize_status_service") in command
    assert "std_srvs/srv/Trigger" in command
    assert "unset RMW_IMPLEMENTATION" in command
    assert kwargs["timeout"] == 5.0


def test_relocalize_saved_map_quotes_path_and_parses_quality(monkeypatch, tmp_path):
    calls = []
    pcd_path = tmp_path / "map with spaces" / "map.pcd"

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        stdout = "success=True\nquality: 0.456\n"
        return subprocess.CompletedProcess(args, 0, stdout=stdout, stderr="")

    monkeypatch.setattr(relocalization.subprocess, "run", fake_run)

    result = relocalization.relocalize_saved_map(
        pcd_path,
        1.0,
        2.0,
        0.3,
        timeout_s=30.0,
    )

    assert result.success is True
    assert result.quality == 0.456
    args, kwargs = calls[0]
    command = args[2]
    assert _runtime_topic_default("relocalize_service") in command
    assert shlex.quote(str(pcd_path)) in command
    assert "x: 1.0" in command
    assert "y: 2.0" in command
    assert "yaw: 0.3" in command
    assert kwargs["timeout"] == 30.0


def test_relocalize_saved_map_timeout_returns_timed_out_result(monkeypatch, tmp_path):
    def raise_timeout(args, **kwargs):
        raise subprocess.TimeoutExpired(args, kwargs["timeout"])

    monkeypatch.setattr(relocalization.subprocess, "run", raise_timeout)

    result = relocalization.relocalize_saved_map(
        tmp_path / "demo" / "map.pcd",
        1.0,
        2.0,
        0.3,
        timeout_s=30.0,
    )

    assert result.success is False
    assert result.timed_out is True
    assert result.message == "call timeout > 30s"


def test_relocalize_saved_map_with_env_keeps_path_out_of_shell_command(
    monkeypatch,
    tmp_path,
):
    calls = []
    pcd_path = tmp_path / "map with spaces" / "map.pcd"

    def fake_run(args, **kwargs):
        calls.append((args, kwargs))
        return subprocess.CompletedProcess(args, 0, stdout="success=True\n", stderr="")

    monkeypatch.setattr(relocalization.subprocess, "run", fake_run)

    result = relocalization.relocalize_saved_map_with_env(
        pcd_path,
        1.0,
        2.0,
        0.3,
        timeout_s=20.0,
        base_env={"KEEP": "1"},
    )

    assert result.success is True
    args, kwargs = calls[0]
    command = args[2]
    env = kwargs["env"]
    assert "$LINGTU_PCD_PATH" in command
    assert str(pcd_path) not in command
    assert env["KEEP"] == "1"
    assert env["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert env["LINGTU_PCD_PATH"] == str(pcd_path)
    assert env["LINGTU_RELOC_X"] == "1.0"
    assert env["LINGTU_RELOC_Y"] == "2.0"
    assert env["LINGTU_RELOC_YAW"] == "0.3"
